#!/usr/bin/env python3
"""Minimal pure-Python ChibiOS Makefile runner for ArduPilot waf (`pass`, `lib`, `clean` targets).

Used only as a fallback in scripts/build-fc.sh when host GNU `make` is unavailable.
Evaluates ArduPilot's `chibios_board.mk` include tree and invokes `arm-none-eabi-gcc/g++/ar`
directly with the exact flags specified in `chibios_board.mk` and `chibios_common.mk`.
"""

from __future__ import annotations

import concurrent.futures
import os
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

VAR_REF_RE = re.compile(r"\$\(([^()${}]+)\)|\$\{([^()${}]+)\}")


def expand(text: str, env: dict[str, str], depth: int = 0) -> str:
    if depth > 20 or "$" not in text:
        return text

    def repl(match: re.Match[str]) -> str:
        expr = (match.group(1) or match.group(2) or "").strip()
        if expr.startswith("sort "):
            words = expand(expr[5:], env, depth + 1).split()
            return " ".join(sorted(set(words)))
        if expr.startswith("strip "):
            return " ".join(expand(expr[6:], env, depth + 1).split())
        if expr.startswith("addprefix "):
            rest = expand(expr[10:], env, depth + 1)
            prefix, items = rest.split(",", 1)
            return " ".join(prefix + w for w in items.split())
        if expr.startswith("patsubst "):
            rest = expand(expr[9:], env, depth + 1)
            pat, rep, items = rest.split(",", 2)
            pat, rep = pat.strip(), rep.strip()
            out = []
            for w in items.split():
                if "%" in pat:
                    pre, suf = pat.split("%", 1)
                    if w.startswith(pre) and w.endswith(suf):
                        stem = w[len(pre) : len(w) - len(suf) if suf else len(w)]
                        out.append(rep.replace("%", stem))
                        continue
                out.append(w)
            return " ".join(out)
        if expr.startswith("notdir "):
            words = expand(expr[7:], env, depth + 1).split()
            return " ".join(os.path.basename(w) for w in words)
        if expr.startswith("dir "):
            words = expand(expr[4:], env, depth + 1).split()
            return " ".join((os.path.dirname(w) or ".") + "/" for w in words)
        if expr.startswith("findstring "):
            rest = expand(expr[11:], env, depth + 1)
            needle, haystack = rest.split(",", 1)
            return needle if needle in haystack else ""
        if expr.startswith("shell "):
            return ""
        return expand(env.get(expr, ""), env, depth + 1)

    prev = None
    cur = text
    while cur != prev and depth < 20:
        prev = cur
        cur = VAR_REF_RE.sub(repl, cur)
        depth += 1
    return cur


def split_logical_lines(path: Path) -> list[str]:
    lines: list[str] = []
    buf = ""
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if raw.endswith("\\"):
            buf += raw[:-1] + " "
        else:
            lines.append(buf + raw)
            buf = ""
    if buf:
        lines.append(buf)
    return lines


def strip_comment(line: str) -> str:
    in_quote = False
    for i, ch in enumerate(line):
        if ch == '"':
            in_quote = not in_quote
        elif ch == "#" and not in_quote:
            return line[:i]
    return line


def eval_cond(directive: str, env: dict[str, str]) -> bool:
    if directive.startswith(("ifeq ", "ifneq ")):
        neg = directive.startswith("ifneq ")
        arg = directive[6:].strip() if neg else directive[5:].strip()
        if arg.startswith("(") and arg.endswith(")"):
            inner = arg[1:-1]
            a, b = inner.split(",", 1)
            a_val = expand(a.strip(), env).strip()
            b_val = expand(b.strip(), env).strip()
        else:
            parts = shlex.split(arg)
            a_val = expand(parts[0], env) if len(parts) > 0 else ""
            b_val = expand(parts[1], env) if len(parts) > 1 else ""
        return (a_val != b_val) if neg else (a_val == b_val)
    neg = directive.startswith("ifndef ")
    var = expand(directive.split(None, 1)[1].strip(), env).strip()
    present = bool(env.get(var, ""))
    return (not present) if neg else present


def parse_makefile(path: Path, env: dict[str, str], cli_vars: set[str]) -> None:
    if path.name == "chibios_common.mk":
        return
    # Stack entries: (parent_active, any_branch_taken, current_active)
    cond_stack: list[tuple[bool, bool, bool]] = []

    def is_active() -> bool:
        return all(entry[2] for entry in cond_stack)

    for raw in split_logical_lines(path):
        if raw.startswith("\t"):
            continue
        line = strip_comment(raw).strip()
        if not line:
            continue

        if line.startswith(("ifeq ", "ifneq ", "ifdef ", "ifndef ")):
            parent_active = is_active()
            if not parent_active:
                cond_stack.append((False, True, False))
            else:
                cond = eval_cond(line, env)
                cond_stack.append((True, cond, cond))
            continue

        if line == "else" or line.startswith("else "):
            if cond_stack:
                parent_active, taken, _ = cond_stack[-1]
                rest = line[4:].strip()
                if not parent_active or taken:
                    cond_stack[-1] = (parent_active, True, False)
                elif not rest:
                    cond_stack[-1] = (parent_active, True, True)
                elif rest.startswith(("ifeq ", "ifneq ", "ifdef ", "ifndef ")):
                    cond = eval_cond(rest, env)
                    cond_stack[-1] = (parent_active, cond, cond)
            continue

        if line == "endif":
            if cond_stack:
                cond_stack.pop()
            continue

        if not is_active():
            continue

        if line.startswith(("include ", "-include ")):
            inc_arg = expand(line.split(None, 1)[1].strip(), env)
            for inc in inc_arg.split():
                inc_path = Path(inc)
                if inc_path.is_file():
                    parse_makefile(inc_path, env, cli_vars)
            continue

        m = re.match(r"^([A-Za-z0-9_]+)\s*(\+=|:=|\?=|=)\s*(.*)$", line)
        if not m:
            continue
        name, op, rhs = m.group(1), m.group(2), m.group(3).strip()
        if name in cli_vars:
            continue
        if op == ":=":
            env[name] = expand(rhs, env)
        elif op == "+=":
            prev = expand(env.get(name, ""), env) if name in env else ""
            add = expand(rhs, env)
            env[name] = (prev + " " + add).strip()
        elif op == "?=":
            if name not in env:
                env[name] = expand(rhs, env)
        else:
            env[name] = expand(rhs, env)


def parse_depfile(dep_path: Path) -> list[Path] | None:
    """Return prerequisite paths from a GCC -MD .d file, or None if unreadable."""
    if not dep_path.is_file():
        return None
    raw = dep_path.read_text(encoding="utf-8", errors="replace")
    joined = raw.replace("\\\n", " ")
    deps: list[Path] = []
    for line in joined.splitlines():
        if ":" not in line:
            continue
        _, rhs = line.split(":", 1)
        for tok in rhs.split():
            deps.append(Path(tok))
    return deps


def is_up_to_date(obj: Path, src: Path, dep_path: Path, ref_mtime: float) -> bool:
    if not obj.is_file():
        return False
    obj_mtime = obj.stat().st_mtime
    if obj_mtime <= ref_mtime or obj_mtime <= src.stat().st_mtime:
        return False
    deps = parse_depfile(dep_path)
    if deps is None:
        return False
    for dep in deps:
        if not dep.is_file() or obj_mtime <= dep.stat().st_mtime:
            return False
    return True


def run_pass(env: dict[str, str], incdirs: list[str]) -> None:
    builddir = Path(env.get("BUILDDIR", "build"))
    builddir.mkdir(parents=True, exist_ok=True)
    out = " ".join(f"-I{d};" for d in incdirs) + "\n"
    (builddir / "include_dirs").write_text(out, encoding="utf-8")


def run_clean(env: dict[str, str]) -> None:
    builddir = Path(env.get("BUILDDIR", "build"))
    shutil.rmtree(".dep", ignore_errors=True)
    shutil.rmtree(builddir, ignore_errors=True)


def archive_objects(lib_path: Path, objs: list[Path]) -> int:
    # Remove any prior archive so stale members cannot linger across reconfigures.
    lib_path.unlink(missing_ok=True)
    ar_cmd = ["arm-none-eabi-ar", "-rcs", str(lib_path), *[str(o) for o in objs]]
    res = subprocess.run(ar_cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print(res.stdout + res.stderr, file=sys.stderr)
        return res.returncode
    return 0


def run_lib(env: dict[str, str], incdirs: list[str], jobs: int) -> int:
    builddir = Path(env.get("BUILDDIR", "build"))
    objdir = builddir / "obj"
    depdir = Path(".dep")
    objdir.mkdir(parents=True, exist_ok=True)
    depdir.mkdir(parents=True, exist_ok=True)

    mcu = env.get("MCU", "cortex-m4")
    use_opt = env.get("USE_OPT", "-fomit-frame-pointer -falign-functions=16")
    use_copt = env.get("USE_COPT", "-Os")
    use_cppopt = env.get("USE_CPPOPT", "-fno-rtti -std=gnu++11")
    use_asopt = env.get("USE_ASOPT", "")
    use_asxopt = env.get("USE_ASXOPT", "")
    use_fpu = env.get("USE_FPU", "hard")
    use_fpu_opt = env.get(
        "USE_FPU_OPT",
        f"-mfloat-abi={use_fpu} -mfpu=fpv4-sp-d16 -fsingle-precision-constant",
    )
    opt_parts = shlex.split(use_opt)
    if env.get("USE_LINK_GC", "yes") == "yes":
        opt_parts += ["-ffunction-sections", "-fdata-sections", "-fno-common"]
    if env.get("USE_LTO", "no") == "yes":
        opt_parts += ["-flto"]

    ddefs: list[str] = []
    dadefs: list[str] = []
    fpu_flags: list[str] = []
    if use_fpu != "no":
        fpu_flags = shlex.split(use_fpu_opt)
        opt_parts += fpu_flags
        ddefs.append("-DCORTEX_USE_FPU=TRUE")
        dadefs.append("-DCORTEX_USE_FPU=TRUE")
    else:
        ddefs.append("-DCORTEX_USE_FPU=FALSE")
        dadefs.append("-DCORTEX_USE_FPU=FALSE")

    udefs_raw = env.get("UDEFS", "")
    # Preserve quoted -DHAL_BOARD_NAME=\"...\" literally as passed to gcc
    udefs = shlex.split(udefs_raw.replace('\\"', '"'))
    udefs = [
        f'-DHAL_BOARD_NAME="{env["HAL_BOARD_NAME"]}"'
        if u.startswith("-DHAL_BOARD_NAME=") and "HAL_BOARD_NAME" in env
        else u
        for u in udefs
    ]
    uadefs = shlex.split(env.get("UADEFS", ""))
    extra_asxflags = shlex.split(env.get("ASXFLAGS", ""))
    cwarn = shlex.split(env.get("CWARN", "-Wall -Wextra -Wundef -Wstrict-prototypes -Werror"))
    cppwarn = shlex.split(env.get("CPPWARN", "-Wall -Wextra -Wundef -Werror"))
    topt = shlex.split(env.get("TOPT", "-mthumb -DTHUMB"))
    inc_flags = ["-I."] + [f"-I{d}" for d in incdirs]

    base_thumb = ["-DTHUMB_PRESENT", "-mno-thumb-interwork", "-DTHUMB_NO_INTERWORKING"]
    cflags = [f"-mcpu={mcu}", *opt_parts, *shlex.split(use_copt), *cwarn, *ddefs, *udefs, "-D_ARDUPILOT_", *base_thumb, *topt]
    cppflags = [f"-mcpu={mcu}", *opt_parts, *shlex.split(use_cppopt), *cppwarn, *ddefs, *udefs, *base_thumb, *topt]
    asflags = [f"-mcpu={mcu}", *dadefs, *uadefs, *shlex.split(use_asopt), *base_thumb, "-mthumb"]
    asxflags = [f"-mcpu={mcu}", *dadefs, *uadefs, *shlex.split(use_asxopt), *extra_asxflags, *base_thumb, "-mthumb", *topt]
    libcc_asxflags = [*asxflags, *fpu_flags]

    csrc = list(dict.fromkeys(env.get("CSRC", "").split()))
    cppsrc = list(dict.fromkeys(env.get("CPPSRC", "").split()))
    asmxsrc = list(dict.fromkeys(env.get("ASMXSRC", "").split()))
    asmsrc = list(dict.fromkeys(env.get("ASMSRC", "").split()))
    libcc_csrc = list(dict.fromkeys(env.get("LIBCC_CSRC", "").split()))
    libcc_asmxsrc = list(dict.fromkeys(env.get("LIBCC_ASMXSRC", "").split()))

    buildroot = Path(env.get("BUILDROOT", "."))
    ref_mtime = max(
        (p.stat().st_mtime for p in (buildroot / "chibios_flags.h", buildroot / "hwdef.h") if p.exists()),
        default=0.0,
    )

    tasks: list[tuple[str, Path, list[str]]] = []

    def queue_sources(groups: tuple[tuple[list[str], str, list[str]], ...]) -> tuple[list[Path], int]:
        out_objs: list[Path] = []
        for src_list, compiler, flags in groups:
            for s in src_list:
                sp = Path(s)
                if not sp.is_file():
                    print(f"FAIL: ChibiOS source file not found: {sp}", file=sys.stderr)
                    return out_objs, 2
                obj = objdir / (sp.stem + ".o")
                dep_path = depdir / (obj.name + ".d")
                out_objs.append(obj)
                if is_up_to_date(obj, sp, dep_path, ref_mtime):
                    continue
                cmd = [compiler, "-c", *flags, *inc_flags, "-MD", "-MP", "-MF", str(dep_path), str(sp), "-o", str(obj)]
                tasks.append((sp.name, obj, cmd))
        return out_objs, 0

    objs, rc = queue_sources(
        (
            (asmxsrc, "arm-none-eabi-gcc", asxflags),
            (asmsrc, "arm-none-eabi-gcc", ["-x", "assembler-with-cpp", *asflags]),
            (csrc, "arm-none-eabi-gcc", cflags),
            (cppsrc, "arm-none-eabi-g++", cppflags),
        )
    )
    if rc != 0:
        return rc

    libcc_objs: list[Path] = []
    if env.get("CRASHCATCHER", "") and (libcc_csrc or libcc_asmxsrc):
        libcc_objs, rc = queue_sources(
            (
                (libcc_asmxsrc, "arm-none-eabi-gcc", libcc_asxflags),
                (libcc_csrc, "arm-none-eabi-gcc", cflags),
            )
        )
        if rc != 0:
            return rc

    def compile_one(item: tuple[str, Path, list[str]]) -> tuple[str, int, str]:
        name, _, cmd = item
        res = subprocess.run(cmd, capture_output=True, text=True)
        return name, res.returncode, (res.stdout + res.stderr)

    if tasks:
        print(f"ChibiOS (chibios_make.py): compiling {len(tasks)} files with {jobs} workers...")
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, jobs)) as pool:
            for name, compile_rc, output in pool.map(compile_one, tasks):
                if compile_rc != 0:
                    print(f"FAIL compiling {name}:\n{output}", file=sys.stderr)
                    return compile_rc
                if output.strip():
                    print(output.strip())

    lib_path = builddir / f"lib{env.get('PROJECT', 'ch')}.a"
    rc = archive_objects(lib_path, objs)
    if rc != 0:
        return rc

    if libcc_objs:
        libcc_path = builddir / "libcc.a"
        rc = archive_objects(libcc_path, libcc_objs)
        if rc != 0:
            return rc

    run_pass(env, incdirs)
    print(f"ChibiOS: Done ({lib_path}, {len(objs)} objects)")
    return 0


def main() -> int:
    args = sys.argv[1:]
    if "--version" in args or "-v" in args:
        print("GNU Make 4.4 (chibios_make.py fallback for ArduPilot ChibiOS)")
        return 0

    makefile = Path("Makefile")
    chdir_target: str | None = None
    jobs = os.cpu_count() or 4
    targets: list[str] = []
    env = dict(os.environ)
    cli_vars: set[str] = set()

    i = 0
    while i < len(args):
        a = args[i]
        if a == "-f" and i + 1 < len(args):
            makefile = Path(args[i + 1])
            i += 2
        elif a == "-C" and i + 1 < len(args):
            chdir_target = args[i + 1]
            i += 2
        elif a.startswith("-C") and len(a) > 2:
            chdir_target = a[2:]
            i += 1
        elif a.startswith("-j"):
            val = a[2:] or (args[i + 1] if i + 1 < len(args) and args[i + 1].isdigit() else "")
            if not a[2:] and val:
                i += 1
            if val.isdigit():
                jobs = int(val)
            i += 1
        elif "=" in a and not a.startswith("-"):
            k, v = a.split("=", 1)
            env[k] = v
            cli_vars.add(k)
            i += 1
        elif a.startswith("-"):
            i += 1
        else:
            targets.append(a)
            i += 1

    if chdir_target is not None:
        os.chdir(chdir_target)

    if not makefile.is_file():
        print(f"chibios_make.py: makefile not found: {makefile}", file=sys.stderr)
        return 2

    parse_makefile(makefile, env, cli_vars)

    inc_raw = expand(
        " ".join(env.get(k, "") for k in ("INCDIR", "DINCDIR", "UINCDIR")),
        env,
    ).split()
    incdirs = list(dict.fromkeys(inc_raw))

    if not targets:
        targets = ["lib"]

    for t in targets:
        if t == "pass":
            run_pass(env, incdirs)
        elif t == "lib":
            rc = run_lib(env, incdirs, jobs)
            if rc != 0:
                return rc
        elif t == "clean":
            run_clean(env)
        else:
            print(f"chibios_make.py: unsupported target {t!r}", file=sys.stderr)
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
