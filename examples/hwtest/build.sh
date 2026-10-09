#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Build the hardware test: examples/hwtest/build/hwtest.bin (safe) and hwtest-exp.bin (with the experimental parts).
#   JIELI_TOOLCHAIN=~/.jieli/toolchain examples/hwtest/build.sh
set -e
TC="${JIELI_TOOLCHAIN:?JIELI_TOOLCHAIN must point at the JieLi Linux toolchain}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$HERE/../.."
OUT="$HERE/build"
rm -rf "$OUT"; mkdir -p "$OUT"
CC="$TC/pi32v2/bin/clang -target pi32v2"
# two images: hwtest (safe: screen, knob, keys, LEDs, USB serial) and hwtest-exp (the same plus the experimental parts)
build() {   # NAME EXPERIMENTAL
    O="$OUT/$1"; mkdir -p "$O"
    $CC -c "$ROOT/chip/crt0.S" -o "$O/crt0.o"
    $CC -c "$ROOT/chip/fm1_vec.S" -o "$O/vec.o"
    $CC -c "$ROOT/chip/fm1_isr.S" -o "$O/isr.o"
    EXTRA=""
    if [ "$2" = 1 ]; then $CC -c "$ROOT/chip/fm1_cpu1.S" -o "$O/cpu1.o"; EXTRA="$O/cpu1.o"; fi
    $CC -Os -ffunction-sections -fno-builtin -Wall -Wno-unused-function -DHWTEST_EXPERIMENTAL="$2" \
        -I"$ROOT/chip" -I"$ROOT/boards/fm1" -c "$HERE/hwtest.c" -o "$O/hwtest.o"
    "$TC/pi32v2/bin/ld" -T "$HERE/app.ld" "$O/crt0.o" "$O/vec.o" "$O/isr.o" $EXTRA "$O/hwtest.o" -o "$O/hwtest.elf"
    "$TC/common/bin/objcopy" -O binary -j .text "$O/hwtest.elf" "$O/text.bin"
    "$TC/common/bin/objcopy" -O binary -j .data "$O/hwtest.elf" "$O/data.bin"
    python3 - "$TC" "$O" "$OUT/$1.bin" <<'PY'
import re, subprocess, sys
tc, out, dest = sys.argv[1:]
syms = subprocess.run([f"{tc}/common/bin/objdump", "-t", f"{out}/hwtest.elf"], capture_output=True, text=True, check=True).stdout
load = int(re.search(r"^([0-9a-f]+) .*\s_data_load$", syms, re.M).group(1), 16)
start = int(re.search(r"^([0-9a-f]+) .*\s_start$", syms, re.M).group(1), 16)
assert start == 0x02000120, hex(start)
img = bytearray(open(f"{out}/text.bin", "rb").read())
data = open(f"{out}/data.bin", "rb").read()
if data:
    img += b"\xff" * (load - 0x02000120 - len(img)) + data
img += b"\xff" * (-len(img) % 4)
open(dest, "wb").write(img)
print(f"{dest.rsplit('/', 1)[-1]}: {len(img)} bytes")
PY
}
build hwtest 0
build hwtest-exp 1
