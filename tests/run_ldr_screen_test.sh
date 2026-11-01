#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# The update loader's screen (loader/ldr_screen.c) on a model of the panel: tests/ldr_screen_test.c
set -eu
cd "$(dirname "$0")/.."
LDR_SCREEN_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$LDR_SCREEN_TEST_DIR"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function -fsanitize=address,undefined \
    tests/ldr_screen_test.c -o "$LDR_SCREEN_TEST_DIR/ldr_screen_test"
"$LDR_SCREEN_TEST_DIR/ldr_screen_test"
