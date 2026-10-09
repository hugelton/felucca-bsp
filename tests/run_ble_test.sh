#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Host test of ble/ against a fake controller (ASan + UBSan).
set -e
D="$(cd "$(dirname "$0")/.." && pwd)"
cc -std=c99 -Wall -Wextra -g -fsanitize=address,undefined -I"$D/ble" "$D/ble/ble_midi_host.c" "$D/tests/ble_midi_host_test.c" -o /tmp/ble_midi_host_test
/tmp/ble_midi_host_test
