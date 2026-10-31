#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Henri David
# fm1_wifi_up (chip/fm1_wifi.h) on the host, with a test table of its own: tests/wifi_up_test.c; and the Wi-Fi tuning's
# register facts (rates, fallback tables, RX filter): tests/wifi_tune_test.c
set -eu
cd "$(dirname "$0")/.."
WIFI_UP_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$WIFI_UP_TEST_DIR"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -Ichip tests/wifi_up_test.c \
    -o "$WIFI_UP_TEST_DIR/wifi_up_test"
"$WIFI_UP_TEST_DIR/wifi_up_test"
"${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -Ichip tests/wifi_tune_test.c \
    -o "$WIFI_UP_TEST_DIR/wifi_tune_test"
"$WIFI_UP_TEST_DIR/wifi_tune_test"
