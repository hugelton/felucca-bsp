#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Kerem Kilic (Ellic Studio)
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
OUT=$(mktemp -d "${TMPDIR:-/tmp}/fm1-flash-tests.XXXXXX")
trap 'rm -rf "$OUT"' EXIT HUP INT TERM
CC=${CC:-cc}
# FLASH_TEST_SANITIZE= (empty) builds without the sanitizers (on a Mac where ASan hangs at start-up).
SANITIZE=${FLASH_TEST_SANITIZE--fsanitize=address,undefined}
for layout in 0 1 2 3 4 5 0-nocpu1; do
    cpu1=1
    case "$layout" in *-nocpu1) layout=${layout%-nocpu1}; cpu1=0 ;; esac
    "$CC" -std=c99 -Wall -Wextra -Werror -Wno-unused-function -O2 \
        $SANITIZE -I"$ROOT/chip" -DTEST_LAYOUT="$layout" -DFM1_CPU1="$cpu1" \
        "$ROOT/tests/flash_test.c" -o "$OUT/flash-$layout"
    "$OUT/flash-$layout"
done
for plain_hi in 0x100000u 0xA0000u; do
    "$CC" -std=c99 -Wall -Wextra -Werror -Wno-unused-function -Wno-int-to-pointer-cast -O2 \
        $SANITIZE -I"$ROOT/chip" -DFM1_FLASH_PLAIN_HI="$plain_hi" \
        "$ROOT/tests/flash_driver_test.c" -o "$OUT/driver"
    "$OUT/driver"
done
# the SMK-37's map over the same driver (boards/smk37/fm1_flash.h shadows the chip's: its regions, its plain window)
"$CC" -std=c99 -Wall -Wextra -Werror -Wno-unused-function -Wno-int-to-pointer-cast -O2 \
    $SANITIZE -I"$ROOT/boards/smk37" -I"$ROOT/chip" "$ROOT/tests/flash_driver_test.c" -o "$OUT/driver-smk37"
"$OUT/driver-smk37"
for config in '-DFM1_FLASH_SIZE=0x2000000u' '-DFM1_FLASH_SIZE=0x80000u' \
    '-DFM1_FLASH_XIP_SIZE=0x200000u' '-DFM1_FLASH_PLAIN_LO=0u' \
    '-DFM1_FLASH_PLAIN_HI=0x92000u'; do
    if printf '#include "fm1_flash_policy.h"\n' | \
        "$CC" -I"$ROOT/chip" "$config" -x c -c -o "$OUT/invalid.o" - 2>"$OUT/error"; then
        echo "FAIL: accepted invalid flash configuration $config" >&2
        exit 1
    fi
done
# a board's map after the chip's (chip/fm1_clock.h includes the chip's fm1_flash.h by name): the build stops
if printf '#include "fm1_clock.h"\n#include "fm1_flash.h"\n' | \
    "$CC" -I"$ROOT/boards/smk37" -I"$ROOT/chip" -x c -E -o "$OUT/pp.i" - 2>"$OUT/error"; then
    echo "FAIL: accepted the SMK-37's map after the chip's" >&2
    exit 1
fi
grep -q "include it before any chip flash header" "$OUT/error" || { cat "$OUT/error" >&2; exit 1; }
printf '#include "fm1_flash.h"\n#include "fm1_clock.h"\n' | \
    "$CC" -I"$ROOT/boards/smk37" -I"$ROOT/chip" -x c -E -o "$OUT/pp.i" -     # (the right order: through)
echo 'flash configuration rejection checks passed'
# a board's own map under the policy (FL_BOARD_MAP): the SMK-37's windows, its protected ranges, the loader's window
"$CC" -std=c99 -Wall -Wextra -Werror -Wno-unused-function -O2 $SANITIZE -I"$ROOT/boards/smk37" -I"$ROOT/chip" \
    "$ROOT/tests/smk37_flash_map_test.c" -o "$OUT/smk37-map"
"$OUT/smk37-map"
