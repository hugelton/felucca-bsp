#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Henri David
# The dim pulse at any width (boards/fm1/fm1_input.h fm1_led_dim_ns) on the host: tests/dim_test.c
set -eu
cd "$(dirname "$0")/.."
DIM_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$DIM_TEST_DIR"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function -fsanitize=address,undefined -Ichip -Iboards/fm1 \
    tests/dim_test.c -o "$DIM_TEST_DIR/dim_test"
"$DIM_TEST_DIR/dim_test"
