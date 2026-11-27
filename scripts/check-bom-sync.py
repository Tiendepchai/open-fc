#!/usr/bin/env python3
"""Validate docs/bom-rev-k.csv against Rev K (100% THT, JST-XH) rules and optional KiCad BOM."""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

REQUIRED_COLS = {
    "refdes",
    "category",
    "part_value",
    "package_mount",
    "qty",
    "kicad_footprint",
    "sheet",
    "status",
    "notes",
}
VALID_STATUSES = {"confirmed", "measured", "design", "candidate", "validated"}
BANNED_PATTERNS = [
    re.compile(r"\bTMUX1511", re.IGNORECASE),
    re.compile(r"\bTSSOP14\b", re.IGNORECASE),
    re.compile(r"\bKF2510\b", re.IGNORECASE),
    re.compile(r"\bRuiBet\b", re.IGNORECASE),
]

TARGET_DISCRETE = 23  # ADR-003: removed Q1/R11/R13 onboard SBUS inverter (external inverted SBUS carrier)
TARGET_HEADERS_CONNECTORS = 12  # 2x(2x15) U1 + U2 + U3 + 6x JST-XH + JP_C1 + J_TEST1 (JP_SBUS1 removed per ADR-003)


def load_bom(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or not REQUIRED_COLS.issubset(set(reader.fieldnames)):
            missing = sorted(REQUIRED_COLS - set(reader.fieldnames or []))
            raise ValueError(f"Missing required CSV columns in {path}: {missing}")
        return list(reader)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check Rev K BOM consistency and THT compliance.")
    parser.add_argument(
        "--bom",
        default="docs/bom-rev-k.csv",
        help="Path to Rev K master BOM CSV (default: docs/bom-rev-k.csv)",
    )
    parser.add_argument(
        "--kicad-bom",
        default=None,
        help="Optional KiCad exported CSV BOM to cross-check refdes presence",
    )
    args = parser.parse_args()

    bom_path = Path(args.bom)
    if not bom_path.is_file():
        print(f"FAIL: BOM file not found: {bom_path}", file=sys.stderr)
        return 2

    rows = load_bom(bom_path)
    errors: list[str] = []
    warnings: list[str] = []

    discrete_qty = 0
    conn_hdr_qty = 0
    tbd_rows = 0

    for idx, row in enumerate(rows, start=2):
        if None in row or any(v is None for v in row.values()):
            errors.append(f"Line {idx}: malformed CSV column count (check unquoted commas)")
            continue
        ref = row["refdes"].strip()
        cat = row["category"].strip()
        mount = row["package_mount"].strip().upper()
        status = row["status"].strip().lower()
        joined = " ".join(str(v) for v in row.values())

        try:
            qty = int(row["qty"].strip())
            if qty <= 0:
                errors.append(f"Line {idx} ({ref}): qty must be > 0")
        except ValueError:
            errors.append(f"Line {idx} ({ref}): invalid integer qty={row['qty']!r}")
            qty = 0

        if mount != "THT":
            errors.append(f"Line {idx} ({ref}): non-THT mount {mount!r} violates Rev K 100% THT baseline")

        if status not in VALID_STATUSES:
            errors.append(f"Line {idx} ({ref}): invalid epistemic status {status!r}")

        for pat in BANNED_PATTERNS:
            if pat.search(joined):
                errors.append(f"Line {idx} ({ref}): banned Rev J superseded part matched ({pat.pattern})")

        if "TBD" in ref or "TBD" in row["kicad_footprint"]:
            tbd_rows += 1

        if cat.lower() == "discrete":
            discrete_qty += qty
        else:
            conn_hdr_qty += qty

    print(f"BOM check ({bom_path}): {len(rows)} rows | discrete qty={discrete_qty} (target {TARGET_DISCRETE}) | headers/connectors qty={conn_hdr_qty}")
    if discrete_qty != TARGET_DISCRETE:
        warnings.append(f"Discrete total qty is {discrete_qty}, expected {TARGET_DISCRETE}")
    if tbd_rows:
        warnings.append(f"{tbd_rows} row(s) still contain TBD refdes/footprint placeholders pending KiCad Rev K sync")

    if args.kicad_bom:
        kicad_path = Path(args.kicad_bom)
        if not kicad_path.is_file():
            errors.append(f"KiCad BOM not found: {kicad_path}")
        else:
            kicad_text = kicad_path.read_text(encoding="utf-8", errors="replace")
            for row in rows:
                ref = row["refdes"].strip()
                if "TBD" in ref or "-" in ref:
                    continue
                if ref not in kicad_text:
                    errors.append(f"RefDes {ref} missing from KiCad BOM {kicad_path}")

    for w in warnings:
        print(f"WARN: {w}")
    for e in errors:
        print(f"FAIL: {e}")

    if errors:
        return 1
    print("PASS: Rev K BOM rules check passed (100% THT, no superseded parts).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
