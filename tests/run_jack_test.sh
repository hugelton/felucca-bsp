#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Henri David
# The headphone jack's switch (boards/fm1/fm1_jack.h fm1_hp_in) on the host: tests/jack_test.c
set -eu
cd "$(dirname "$0")/.."
JACK_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$JACK_TEST_DIR"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -Ichip -Iboards/fm1 tests/jack_test.c \
    -o "$JACK_TEST_DIR/jack_test"
"$JACK_TEST_DIR/jack_test"
