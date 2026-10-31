#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (c) 2026 Julius Vitkauskas
"""The optional hooks of chip/fm1_flash.h and chip/fm1_isr.S are off unless a firmware defines them.

Runs the host C preprocessor ($CC, default cc) only, no target toolchain:
  - by default the flash window (irq_save / irq_restore / fl_erase4k / fl_write) is token for token what it was
    before the options existed (cli ... csync; sti, no callback), ICFG is not touched, and the late code is -4;
  - FL_IRQ_MASK_LEVEL=6, FL_CAN_START() and FL_ERR_LATE_READY do what they say, and other levels do not compile;
  - erase and program wait with fl_wait_issued (normal / extended bounds), never with fl_wait_ready;
  - fm1_isr.S emits exactly its two default entries, with no calls but the body; FM1_ISR_ENTRIES replaces them.
    python3 tests/flash_hooks_test.py
"""
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CC = os.environ.get("CC", "cc").split()
fails = 0


def check(what, ok):
    global fails
    print(f"{'ok  ' if ok else 'FAIL'} {what}")
    fails += not ok


def pp(source, *flags, lang="c"):
    with tempfile.TemporaryDirectory() as d:
        src = Path(d) / ("tu.c" if lang == "c" else "tu.S")
        src.write_text(source)
        r = subprocess.run([*CC, "-E", "-P", "-x", lang if lang == "c" else "assembler-with-cpp",
                            f"-I{ROOT / 'chip'}", f"-I{d}", *flags, str(src)], capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


def norm(s):
    return " ".join(re.findall(r'"(?:\\.|[^"\\])*"|\w+|[^\s\w]', s))


def body(text, name):
    """the definition of function name (from its return type to the closing brace), normalized"""
    m = re.search(r"(?:^|(?<=[;}]))[^;{}]*\b" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{", text)
    i, j = m.start(), m.end() - 1
    depth, k = 0, j
    while True:
        depth += {"{": 1, "}": -1}.get(text[k], 0)
        k += 1
        if not depth:
            return norm(text[i:k])


FLASH = '#include "fm1_flash.h"\n'
# chip/fm1_flash.h before the options, preprocessed (felucca-bsp 2cd696a)
BEFORE = {
    "irq_save":
        'static inline uint32_t irq_save(void) { __asm__ volatile("cli" ::: "memory"); return 0; }',
    "irq_restore":
        'static inline void irq_restore(uint32_t f) { (void)f; __asm__ volatile("csync\\n\\tsti" ::: "memory"); }',
    "fl_erase4k":
        "static int fl_erase4k(uint32_t off, uint32_t *took_us) { uint32_t f = irq_save(); "
        "int rc = ((__typeof__(&fl_erase4k_ram))fl_far((void *)&fl_erase4k_ram))(off, took_us); "
        "irq_restore(f); return rc; }",
    "fl_write":
        "static int fl_write(uint32_t off, const uint8_t *src_ram, uint32_t n) { uint32_t took; while (n) { "
        "uint32_t chunk = 256u - (off & 0xFFu); int rc; uint32_t f; if (chunk > n) chunk = n; f = irq_save(); "
        "rc = ((__typeof__(&fl_prog_ram))fl_far((void *)&fl_prog_ram))(off, src_ram, chunk, &took); "
        "irq_restore(f); if (rc) return rc; off += chunk; src_ram += chunk; n -= chunk; } return 0; }",
}

rc, out = pp(FLASH)
check("fm1_flash.h preprocesses with no option defined", rc == 0)
for sig, want in BEFORE.items():
    check(f"default {sig}: as before the options", body(out, sig) == norm(want))
check("default: ICFG untouched, no gate callback", "icfg" not in out and "FL_CAN_START" not in out)
wait = body(out, "fl_wait_issued")
check("default late code: -4 (fl_wait_ready's)", "FL_ERR_LATE_READY" not in wait and norm("return dt <= normal ? 0 : (-4)") in wait)
erase, prog = body(out, "fl_erase4k_ram"), body(out, "fl_prog_ram")
check("erase waits for idle: normal 500 ms, extended 2 s", "fl_wait_issued ( issued , 500000u , 2000000u" in erase)
check("program waits for idle: normal 10 ms, extended 50 ms", "fl_wait_issued ( issued , 10000u , 50000u" in prog)
check("issue time read before the command", erase.index("issued = (") < erase.index("0x20") and
      prog.index("issued = (") < prog.index("0x02 )"))
check("no erase/program returns through fl_wait_ready", "fl_wait_ready" not in erase + prog)
check("the stuck-part reset is RAM-inline (no call out of .ram_text)",
      "fl_reset_busy" in wait and "static inline __attribute__((always_inline)) void fl_reset_busy" in out)

rc, out = pp("int gate(void);\n#define FL_CAN_START() gate()\n" + FLASH, "-DFL_IRQ_MASK_LEVEL=6", "-DFL_ERR_LATE_READY=(-9)")
check("options preprocess", rc == 0)
save, restore = body(out, "irq_save"), body(out, "irq_restore")
check("level 6: ICFG bit 8 cleared, saved ICFG restored, no cli/sti",
      "icfg" in save and "~ 0x100u" in save and "icfg" in restore and "cli" not in save and "sti" not in restore)
for sig in ("fl_erase4k", "fl_write"):
    b = body(out, sig)
    check(f"gate in {sig}: after irq_save, before the RAM driver, refusal -8 after restore",
          b.count("gate ( )") == 1 and b.index("irq_save") < b.index("gate ( )") < b.index("fl_far") and
          "if ( ! gate ( ) ) { irq_restore ( f ) ; return ( - 8 ) ; }" in b)
check("late code overridable", "( - 9 )" in body(out, "fl_wait_issued"))
rc, out = pp(FLASH, "-DFL_IRQ_MASK_LEVEL=7")
check("level 7 = the default", rc == 0 and body(out, "irq_save") == norm(BEFORE["irq_save"]))
for level in ("5", "0", "8"):
    rc, out = pp(FLASH, f"-DFL_IRQ_MASK_LEVEL={level}")
    check(f"level {level} refused", rc != 0 and "FL_IRQ_MASK_LEVEL" in out)

ISR = '#include "fm1_isr.S"\n'
rc, out = pp(ISR, lang="asm")
entries = [norm(l) for l in out.splitlines() if l.strip().startswith("FM1_ISR ")]
check("fm1_isr.S default: exactly the two entries, body only",
      rc == 0 and entries == [norm("FM1_ISR isr_timer5, fm1_timer5_irq"), norm("FM1_ISR isr_alnk0, fm1_alnk0_irq")])
macro = norm(out[out.index(".macro"):out.index(".endm")])
check("macro: calls only body when enter/leave are blank",
      ". ifnb \\ enter call \\ enter . endif call \\ body . ifnb \\ leave call \\ leave . endif" in macro)
with tempfile.TemporaryDirectory() as d:
    (Path(d) / "entries.inc").write_text("\tFM1_ISR isr_timer5, fm1_timer5_irq, a_in, a_out\n\tFM1_ISR isr_x, x_irq\n")
    rc, out = pp(ISR, f"-I{d}", '-DFM1_ISR_ENTRIES="entries.inc"', lang="asm")
entries = [norm(l) for l in out.splitlines() if l.strip().startswith("FM1_ISR ")]
check("FM1_ISR_ENTRIES replaces the defaults",
      rc == 0 and entries == [norm("FM1_ISR isr_timer5, fm1_timer5_irq, a_in, a_out"), norm("FM1_ISR isr_x, x_irq")])
print("flash/ISR hooks: all checks passed" if not fails else f"{fails} failed")
sys.exit(fails != 0)
