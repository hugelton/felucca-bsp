#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Kerem Kilic (Ellic Studio)
"""Build/link the BSP examples and loader; audit RAM-only flash/CPU1 code.

JIELI_TOOLCHAIN=/path/to/toolchain python3 tests/check_flash_target.py
Uses Docker on macOS/non-x86 Linux (override JIELI_DOCKER=0 or 1).
Artifacts live in a temporary directory, removed after the audit.
"""
import os
from pathlib import Path
import platform
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
TC = Path(os.environ["JIELI_TOOLCHAIN"]).expanduser().resolve()
DOCKER = os.environ.get("JIELI_DOCKER", "0" if platform.system() == "Linux" and platform.machine() in ("x86_64", "AMD64") else "1") == "1"

with tempfile.TemporaryDirectory(prefix="fm1-flash-target-") as tmp:
    out = Path(tmp)
    work = "/work" if DOCKER else str(ROOT)
    dest = "/out" if DOCKER else str(out)
    tc = "/opt/jieli" if DOCKER else str(TC)
    commands = []

    def command(*args):
        commands.append(shlex.join(map(str, args)))

    cc = [f"{tc}/pi32v2/bin/clang", "-target", "pi32v2"]
    flags = ["-Os", "-ffunction-sections", "-fno-builtin", "-Wall", "-Werror", "-Wno-unused-function", "-Ichip", "-Iboards/fm1"]
    for src, name in [("chip/crt0.S", "crt0"), ("chip/fm1_vec.S", "vec"), ("chip/fm1_isr.S", "isr"),
                      ("chip/fm1_cpu1.S", "cpu1-entry"), ("loader/crt0_ldr.S", "ldr-start")]:
        command(*cc, "-c", src, "-o", f"{dest}/{name}.o")
    for name, opts in [("safe", []), ("cpu1", ["-DHWTEST_EXPERIMENTAL=1"]),
                       ("fallback", ["-DHWTEST_EXPERIMENTAL=1", "-DFM1_CPU1_FALLBACK=1"]),
                       ("expanded", ["-DFM1_FLASH_SIZE=0x400000u", "-DFL_UID=1",
                                     "-DFM1_FLASH_WRITE_REGIONS(X)=X(0x100000u,0x400000u)"])]:
        command(*cc, *flags, *opts, "-c", "examples/hwtest/hwtest.c", "-o", f"{dest}/{name}.o")
        extra = [f"{dest}/cpu1-entry.o"] if name in ("cpu1", "fallback") else []
        command(f"{tc}/pi32v2/bin/ld", "-T", "examples/hwtest/app.ld",
                f"{dest}/crt0.o", f"{dest}/vec.o", f"{dest}/isr.o", *extra,
                f"{dest}/{name}.o", "-o", f"{dest}/{name}.elf")
        command(f"{tc}/common/bin/objdump", "-d", "-mcpu=r3", "-mattr=+fprev1", "-j", ".ram_text", f"{dest}/{name}.elf")
        commands[-1] += f" > {shlex.quote(f'{dest}/{name}.dis')}"
    command(*cc, *flags, "-c", "loader/loader.c", "-o", f"{dest}/loader.o")
    command(f"{tc}/pi32v2/bin/ld", "--gc-sections", "-e", "_start", "-T", "loader/loader.ld",
            f"{dest}/ldr-start.o", f"{dest}/loader.o", "-o", f"{dest}/loader.elf")
    command(f"{tc}/common/bin/objdump", "-h", f"{dest}/loader.elf")
    commands[-1] += f" > {shlex.quote(f'{dest}/loader.sections')}"
    command(f"{tc}/common/bin/objdump", "-d", "-mcpu=r3", "-mattr=+fprev1", f"{dest}/loader.elf")
    commands[-1] += f" > {shlex.quote(f'{dest}/loader.dis')}"
    script = "set -eu\n" + "\n".join(commands)
    if DOCKER:
        run = ["docker", "run", "--rm", "--platform", "linux/amd64", "-v", f"{ROOT}:/work:ro",
               "-v", f"{TC}:/opt/jieli:ro", "-v", f"{out}:/out", "-w", work,
               "debian:bookworm-slim", "/bin/sh", "-c", script]
    else:
        run = ["/bin/sh", "-c", script]
    subprocess.run(run, cwd=ROOT, check=True)
    for name in ("safe", "cpu1", "fallback", "expanded"):
        dis = (out / f"{name}.dis").read_text()
        current = ""
        found = set()
        for line in dis.splitlines():
            match = re.match(r"^([A-Za-z_][\w.$]*):$", line)
            if match:
                current = match[1]
                if current.startswith("fl_"):
                    found.add(current)
            if re.search(r"\bcall\b", line):
                # The RAM worker calls only the supplied job, with XIP on.
                assert current == "fm1_cpu1_main" and re.search(r"\bcall\s+r\d+", line), line
            if re.search(r"\b(?:call|goto)\b", line):
                target = re.search(r": ([0-9a-f]+) >", line)
                if target:
                    assert 0x1C00000 <= int(target[1], 16) < 0x1C80000, line
        assert {"fl_jedec_ram", "fl_status_ram", "fl_erase4k_ram", "fl_prog_ram", "fl_read_ram"} <= found, found
        if name == "expanded":
            assert "fl_uid_ram" in found
        print(f"{name}: linked; flash RAM routines contain no calls or direct XIP branches")
    sections = set()
    for line in (out / "loader.sections").read_text().splitlines():
        m = re.match(r"\s*\d+\s+(\.\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)", line)
        if m and int(m[2], 16) and m[1] in (".text", ".bss"):
            start, size = int(m[3], 16), int(m[2], 16)
            assert 0x1C00000 <= start and start + size <= 0x1C80000, line
            sections.add(m[1])
    assert sections == {".text", ".bss"}, sections
    for line in (out / "loader.dis").read_text().splitlines():
        if re.search(r"\b(?:call|goto)\b", line):
            target = re.search(r": ([0-9a-f]+) >", line)
            if target:
                assert 0x1C00000 <= int(target[1], 16) < 0x1C80000, line
    print("loader: linked entirely in RAM; no direct calls/branches into XIP")
