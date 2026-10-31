#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
OUT=$(mktemp -d "${TMPDIR:-/tmp}/fm1-flash-tests.XXXXXX")
trap 'rm -rf "$OUT"' EXIT HUP INT TERM
CC=${CC:-cc}
for layout in 0 1 2 3 4; do
    "$CC" -std=c99 -Wall -Wextra -Werror -Wno-unused-function -O2 \
        -fsanitize=address,undefined -I"$ROOT/chip" -DTEST_LAYOUT="$layout" \
        "$ROOT/tests/flash_test.c" -o "$OUT/flash-$layout"
    "$OUT/flash-$layout"
done
for plain_hi in 0x100000u 0xA0000u; do
    "$CC" -std=c99 -Wall -Wextra -Werror -Wno-unused-function -Wno-int-to-pointer-cast -O2 \
        -fsanitize=address,undefined -I"$ROOT/chip" -DFM1_FLASH_PLAIN_HI="$plain_hi" \
        "$ROOT/tests/flash_driver_test.c" -o "$OUT/driver"
    "$OUT/driver"
done
for config in '-DFM1_FLASH_SIZE=0x2000000u' '-DFM1_FLASH_SIZE=0x80000u' \
    '-DFM1_FLASH_XIP_SIZE=0x200000u' '-DFM1_FLASH_PLAIN_LO=0u' \
    '-DFM1_FLASH_PLAIN_HI=0x92000u'; do
    if printf '#include "fm1_flash_policy.h"\n' | \
        "$CC" -I"$ROOT/chip" "$config" -x c -c -o "$OUT/invalid.o" - 2>"$OUT/error"; then
        echo "FAIL: accepted invalid flash configuration $config" >&2
        exit 1
    fi
done
echo 'flash configuration rejection checks passed'
