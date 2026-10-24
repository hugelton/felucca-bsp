#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Kerem Kilic (Ellic Studio)
set -eu
cd "$(dirname "$0")/.."
USB_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$USB_TEST_DIR"' EXIT HUP INT TERM
for mode in host host-audio device loader; do
    case "$mode" in
        host) flags="" ;;
        host-audio) flags="-DTEST_UAC=1" ;;
        device) flags="-DTEST_HOST=0" ;;
        loader) flags="-DFELUCCA_LOADER=1" ;;
    esac
    # Word splitting is intentional for the fixed compiler options above.
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function \
        -Wno-unused-variable -Wno-missing-field-initializers \
        -fsanitize=address,undefined -Ichip $flags tests/usb_host_test.c -o "$USB_TEST_DIR/$mode"
    "$USB_TEST_DIR/$mode"
done
