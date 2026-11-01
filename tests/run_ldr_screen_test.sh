#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# The update loader's screen (loader/ldr_screen.c) on a model of the panel: tests/ldr_screen_test.c; and the fast
# install's table check (loader/ldr_core.c ldr_table_ok): tests/ldr_table_test.c
set -eu
cd "$(dirname "$0")/.."
LDR_SCREEN_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$LDR_SCREEN_TEST_DIR"' EXIT HUP INT TERM
for t in ldr_screen_test ldr_table_test; do
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function -fsanitize=address,undefined -Ichip \
        "tests/$t.c" -o "$LDR_SCREEN_TEST_DIR/$t"
    "$LDR_SCREEN_TEST_DIR/$t"
done
