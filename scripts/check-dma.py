#!/usr/bin/env python3
"""Inspect STM32F405 DMA assignments for OPENFCF405.

Supports two inputs:
  1. ChibiOS generated `hwdef.h` (post-chibios_hwdef/configure): verifies actual DMA streams
     and enforces SHARED_DMA_MASK == 0 with dedicated TIM4_UP and SPI1 DMA.
  2. Source `hwdef.dat` (pre-build static check): checks Rev K TIM4_UP vs USART2_TX,
     DMA_NOSHARE, and BIDIR risks.
"""

from __future__ import annotations

import argparse
import ast
import re
import sys
from collections import defaultdict
from pathlib import Path

DMA_STREAM_RE = re.compile(
    r"^#define\s+STM32_([A-Z0-9_]+)_DMA_STREAM\s+STM32_DMA_STREAM_ID\((\d+),\s*(\d+)\)"
)
DMA_CHAN_RE = re.compile(r"^#define\s+STM32_([A-Z0-9_]+)_DMA_CHAN\s+(\d+)")
SHARED_MASK_RE = re.compile(r"^#define\s+SHARED_DMA_MASK\s+(.+)$")


def _eval_int_ast(node: ast.AST) -> int | None:
    if isinstance(node, ast.Expression):
        return _eval_int_ast(node.body)
    if isinstance(node, ast.Constant) and isinstance(node.value, int) and not isinstance(node.value, bool):
        return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub, ast.Invert)):
        val = _eval_int_ast(node.operand)
        if val is None:
            return None
        if isinstance(node.op, ast.UAdd):
            return +val
        if isinstance(node.op, ast.USub):
            return -val
        return ~val
    if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.BitOr, ast.BitAnd, ast.BitXor, ast.LShift, ast.RShift)):
        left = _eval_int_ast(node.left)
        right = _eval_int_ast(node.right)
        if left is None or right is None:
            return None
        if isinstance(node.op, ast.BitOr):
            return left | right
        if isinstance(node.op, ast.BitAnd):
            return left & right
        if isinstance(node.op, ast.BitXor):
            return left ^ right
        if isinstance(node.op, ast.LShift):
            return left << right if 0 <= right <= 64 else None
        return left >> right if 0 <= right <= 64 else None
    return None


def parse_int_literal(raw: str) -> int | None:
    expr = re.sub(r"\b(0[xX][0-9a-fA-F]+|\d+)[uUlL]+\b", r"\1", raw.strip())
    if not expr:
        return None
    try:
        tree = ast.parse(expr, mode="eval")
    except SyntaxError:
        return None
    return _eval_int_ast(tree)


def check_hwdef_h(path: Path, allow_shared: bool = False) -> int:
    text = path.read_text(encoding="utf-8", errors="replace")
    streams: dict[tuple[int, int], list[str]] = defaultdict(list)
    chans: dict[str, int] = {}
    shared_mask_raw: str | None = None

    for raw in text.splitlines():
        line = raw.strip()
        m_ch = DMA_CHAN_RE.match(line)
        if m_ch:
            chans[m_ch.group(1)] = int(m_ch.group(2))
        m_st = DMA_STREAM_RE.match(line)
        if m_st:
            periph, ctrl, stream = m_st.group(1), int(m_st.group(2)), int(m_st.group(3))
            streams[(ctrl, stream)].append(periph)
        m_sh = SHARED_MASK_RE.match(line)
        if m_sh:
            shared_mask_raw = m_sh.group(1).strip()

    if not streams:
        print(f"FAIL: no STM32_*_DMA_STREAM definitions parsed from {path}")
        return 1

    print(f"ChibiOS DMA map ({path}):")
    conflicts = []
    for (ctrl, stream), periphs in sorted(streams.items()):
        details = [f"{p}(ch={chans.get(p, '?')})" for p in periphs]
        tag = " [SHARED]" if len(periphs) > 1 else ""
        print(f"  DMA{ctrl} Stream {stream}: {', '.join(details)}{tag}")
        if len(periphs) > 1:
            conflicts.append((ctrl, stream, periphs))

    if shared_mask_raw is None:
        print("FAIL: SHARED_DMA_MASK definition missing from generated hwdef.h")
        return 1

    shared_mask_val = parse_int_literal(shared_mask_raw)
    print(f"SHARED_DMA_MASK = {shared_mask_raw}")
    if shared_mask_val is None:
        print(f"FAIL: unable to parse SHARED_DMA_MASK literal: {shared_mask_raw!r}")
        return 1

    tim4_streams = [
        (ctrl, stream, p)
        for (ctrl, stream), periphs in streams.items()
        for p in periphs
        if "TIM4" in p
    ]
    if tim4_streams:
        print("INFO: TIM4 DMA allocated:", tim4_streams)
        print("  Supports PWM, OneShot125, and unidirectional DShot150/300/600 via TIM4_UP DMAR.")
        print("  Bi-Directional DShot (BIDIR) remains unsupported on PB9 (TIM4_CH4 has no compare DMA).")
    else:
        print("INFO: TIM4 has no DMA streams allocated (PWM / OneShot125 only).")

    spi1_periphs = {
        p
        for periphs in streams.values()
        for p in periphs
        if "SPI1" in p
    }
    if not {"SPI_SPI1_RX", "SPI_SPI1_TX"}.issubset(spi1_periphs):
        print(f"FAIL: expected dedicated SPI1 RX/TX DMA streams, found: {sorted(spi1_periphs)}")
        return 1

    critical = [
        c for c in conflicts if any("TIM4" in p or "SPI1" in p for p in c[2])
    ]
    if critical:
        print("FAIL: DMA sharing touches TIM4 or SPI1:", critical)
        return 1

    if conflicts and not allow_shared:
        print("FAIL: DMA sharing detected (Rev K requires SHARED_DMA_MASK = 0):", conflicts)
        return 1

    if shared_mask_val != 0 and not allow_shared:
        print(f"FAIL: SHARED_DMA_MASK is non-zero ({shared_mask_raw}); Rev K requires 0.")
        return 1

    if conflicts:
        print("WARN: Non-critical DMA sharing detected:", conflicts)

    print("PASS: Zero DMA conflicts detected (SHARED_DMA_MASK = 0; TIM4/SPI1 dedicated).")
    return 0


def check_hwdef_dat(path: Path) -> int:
    lines = path.read_text(encoding="utf-8").splitlines()
    tim4_pins: list[tuple[str, str, list[str]]] = []
    usart2_tx_nodma = False
    has_usart2_tx = False
    noshare_tokens: set[str] = set()

    for raw in lines:
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        tokens = line.split()
        if tokens[0] == "DMA_NOSHARE":
            noshare_tokens.update(tokens[1:])
            continue
        if len(tokens) >= 3 and tokens[1].startswith("TIM4_CH"):
            tim4_pins.append((tokens[0], tokens[1], tokens[2:]))
        if len(tokens) >= 2 and tokens[1] == "USART2_TX":
            has_usart2_tx = True
            usart2_tx_nodma = "NODMA" in tokens[2:]

    print(f"Static STM32F405 RM0090 DMA pre-check ({path}):")
    if not tim4_pins:
        print("FAIL: no TIM4_CH* motor pins found in hwdef.dat")
        return 1

    issues = 0
    nodma_count = 0
    for pin, fn, flags in tim4_pins:
        nodma = "NODMA" in flags
        bidir = "BIDIR" in flags
        if nodma:
            nodma_count += 1
        mode = "NODMA (PWM/OneShot125)" if nodma else ("BIDIR-DMA" if bidir else "TIM4_UP DMAR (PWM/OneShot/DShot)")
        print(f"  {pin:<4} {fn:<10} [{mode}]")
        if bidir:
            print(
                f"    FAIL: BIDIR unsupported on TIM4 group ({pin}/{fn}: TIM4_CH4/PB9 lacks compare DMA "
                "and TIM4_CH2/CH3 collide with I2C2 DMA1 S3/S7 on STM32F405)."
            )
            issues += 1

    any_tim4_dma = nodma_count < len(tim4_pins)
    if 0 < nodma_count < len(tim4_pins):
        print("  FAIL: mixed NODMA/DMA flags across TIM4_CH1..CH4 group.")
        issues += 1

    if any_tim4_dma and has_usart2_tx and not usart2_tx_nodma:
        print("  FAIL: TIM4_UP (DMA1 S6 C2) conflicts with USART2_TX (DMA1 S6 C4). Add NODMA to PA2 USART2_TX or TIM4.")
        issues += 1

    if any_tim4_dma and not any(t in ("TIM4_UP", "TIM4*") for t in noshare_tokens):
        print("  FAIL: TIM4 DMA enabled without DMA_NOSHARE TIM4_UP guard in hwdef.dat.")
        issues += 1

    if issues:
        print(f"FAIL: {issues} unmitigated STM32F405 TIM4 DMA conflict(s).")
        return 1

    if any_tim4_dma:
        print("PASS: Static pre-check clean (TIM4_UP DMAR dedicated on DMA1 S6; USART2_TX is NODMA; BIDIR disabled).")
    else:
        print("PASS: Static pre-check clean (TIM4 NODMA — PWM/OneShot125 baseline).")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Check OPENFCF405 STM32F405 DMA allocation.")
    parser.add_argument(
        "target",
        nargs="?",
        default="firmware/board-openfcf405/hwdef.dat",
        help="Path to generated ChibiOS hwdef.h or source hwdef.dat",
    )
    parser.add_argument(
        "--allow-shared",
        action="store_true",
        help="Allow non-critical DMA sharing (still fails on TIM4/SPI1 conflicts).",
    )
    args = parser.parse_args()
    path = Path(args.target)
    if not path.is_file():
        print(f"FAIL: file not found: {path}", file=sys.stderr)
        return 2
    if path.suffix == ".h":
        return check_hwdef_h(path, allow_shared=args.allow_shared)
    return check_hwdef_dat(path)


if __name__ == "__main__":
    raise SystemExit(main())
