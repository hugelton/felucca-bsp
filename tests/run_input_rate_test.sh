#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Charles Vestal
# The input scan (boards/fm1/fm1_input.h fm1_input_tick) at each of its rates, FM1_INPUT_HZ 10000, 5000, 2000 and
# 1000, on the host: tests/input_rate_test.c
set -eu
cd "$(dirname "$0")/.."
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT HUP INT TERM
# INPUT_TEST_SANITIZE= (empty) builds without the sanitizers (on a Mac where ASan hangs at start-up).
SANITIZE=${INPUT_TEST_SANITIZE--fsanitize=address,undefined}
for hz in 10000 5000 2000 1000; do
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function $SANITIZE -Ichip -Iboards/fm1 \
        -DFM1_INPUT_HZ=$hz tests/input_rate_test.c -o "$OUT/input_rate_test_$hz"
    "$OUT/input_rate_test_$hz"
done
