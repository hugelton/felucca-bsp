#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Build the hardware test: examples/hwtest/build/hwtest.bin (the app image the SPL maps at 0x02000120).
#   JIELI_TOOLCHAIN=~/.jieli/toolchain examples/hwtest/build.sh
set -e
TC="${JIELI_TOOLCHAIN:?JIELI_TOOLCHAIN must point at the JieLi Linux toolchain}"
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$HERE/../.."
OUT="$HERE/build"
rm -rf "$OUT"; mkdir -p "$OUT"
CC="$TC/pi32v2/bin/clang -target pi32v2"
$CC -c "$ROOT/chip/crt0.S" -o "$OUT/crt0.o"
$CC -c "$ROOT/chip/fm1_vec.S" -o "$OUT/vec.o"
$CC -c "$ROOT/chip/fm1_isr.S" -o "$OUT/isr.o"
$CC -Os -ffunction-sections -fno-builtin -Wall -Wno-unused-function \
    -I"$ROOT/chip" -I"$ROOT/boards/fm1" -c "$HERE/hwtest.c" -o "$OUT/hwtest.o"
"$TC/pi32v2/bin/ld" -T "$HERE/app.ld" "$OUT/crt0.o" "$OUT/vec.o" "$OUT/isr.o" "$OUT/hwtest.o" -o "$OUT/hwtest.elf"
"$TC/common/bin/objcopy" -O binary -j .text "$OUT/hwtest.elf" "$OUT/text.bin"
"$TC/common/bin/objcopy" -O binary -j .data "$OUT/hwtest.elf" "$OUT/data.bin"
# .data follows .text at its load address
python3 - "$TC" "$OUT" <<'PY'
import re, subprocess, sys
tc, out = sys.argv[1:]
syms = subprocess.run([f"{tc}/common/bin/objdump", "-t", f"{out}/hwtest.elf"], capture_output=True, text=True, check=True).stdout
load = int(re.search(r"^([0-9a-f]+) .*\s_data_load$", syms, re.M).group(1), 16)
start = int(re.search(r"^([0-9a-f]+) .*\s_start$", syms, re.M).group(1), 16)
assert start == 0x02000120, hex(start)
img = bytearray(open(f"{out}/text.bin", "rb").read())
data = open(f"{out}/data.bin", "rb").read()
if data:
    img += b"\xff" * (load - 0x02000120 - len(img)) + data
img += b"\xff" * (-len(img) % 4)
open(f"{out}/hwtest.bin", "wb").write(img)
print(f"hwtest.bin: {len(img)} bytes")
PY
