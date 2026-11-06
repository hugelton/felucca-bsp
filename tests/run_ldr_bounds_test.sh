#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Where the update loader (loader/ldr_core.c) erases and programs: tests/ldr_bounds_test.c, with the defaults
# (0x93000 slot, no slot switch), and the SMK-37's loader (LDR_SMK37: loader/ldr_smk37.h, boards/smk37/fm1_flash.h).
# The bigger slot's sets (FL_APP_END, LDR_SLOT_SWITCH, LDR_CLEAR_END) come with it
set -eu
cd "$(dirname "$0")/.."
LDR_BOUNDS_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$LDR_BOUNDS_TEST_DIR"' EXIT HUP INT TERM
i=0
for defs in "" "-DLDR_SMK37"; do
    i=$((i + 1))
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function -fsanitize=address,undefined \
        -fno-sanitize-recover=undefined -Ichip $defs tests/ldr_bounds_test.c -o "$LDR_BOUNDS_TEST_DIR/t$i"
    "$LDR_BOUNDS_TEST_DIR/t$i"
done
