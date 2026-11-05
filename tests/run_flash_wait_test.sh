#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# The erase / program completion model (tests/flash_wait_test.c) around chip/fm1_flash_wait.h.
# FLASH_TEST_SANITIZE= (empty) builds without the sanitizers (on a Mac where ASan hangs at start-up).
set -eu
cd "$(dirname "$0")/.."
FLASH_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$FLASH_TEST_DIR"' EXIT HUP INT TERM
SANITIZE=${FLASH_TEST_SANITIZE--fsanitize=address,undefined}
"${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror $SANITIZE -o "$FLASH_TEST_DIR/flash_wait_test" tests/flash_wait_test.c
"$FLASH_TEST_DIR/flash_wait_test"
