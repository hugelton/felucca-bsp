#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
OUT=$(mktemp -d "${TMPDIR:-/tmp}/fm1-board-tests.XXXXXX")
trap 'rm -rf "$OUT"' EXIT HUP INT TERM
CC=${CC:-cc}
for test in lcd lcd_hw audio; do
    "$CC" -std=c99 -D_GNU_SOURCE -Wall -Wextra -Werror -Wno-unused-function \
        -Wno-int-to-pointer-cast -O2 -fsanitize=address,undefined \
        -I"$ROOT/chip" -I"$ROOT/boards/fm1" "$ROOT/tests/${test}_test.c" -o "$OUT/$test"
    "$OUT/$test"
done
