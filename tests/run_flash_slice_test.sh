#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# The sliced sector erase (chip/fm1_flash_slice.h) on a model of the P25Q80H: tests/flash_slice_test.c
set -eu
cd "$(dirname "$0")/.."
FLASH_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$FLASH_TEST_DIR"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -Ichip tests/flash_slice_test.c \
    -o "$FLASH_TEST_DIR/flash_slice_test"
"$FLASH_TEST_DIR/flash_slice_test"
