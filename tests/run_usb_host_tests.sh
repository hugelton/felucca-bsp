#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Kerem Kilic (Ellic Studio)
# USB_TEST_SANITIZE= (empty) builds without the sanitizers (on a Mac where ASan hangs at start-up).
set -eu
cd "$(dirname "$0")/.."
USB_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$USB_TEST_DIR"' EXIT HUP INT TERM
SANITIZE=${USB_TEST_SANITIZE--fsanitize=address,undefined}
D="-DTEST_HOST=0 -DTEST_UAC=1"
# desc-*: the device descriptors in an option set. TEST_GOLDEN_* are the CRC-32s the bsp built before
# FELUCCA_UAC_MIDI_LAST (with it and FELUCCA_UAC_IAD 0, a build's descriptors stay byte for byte).
for mode in host host-audio device loader \
        desc-midi desc-in desc-in-cdc desc-in-cdc1 desc-in-48k desc-asfirst desc-out desc-out-asfirst-cdc \
        desc-out-48k-asfirst desc-last desc-last-cdc desc-last-cdc1 desc-last-48k desc-last-out \
        desc-last-out-cdc desc-last-out-48k-cdc2 desc-last-asfirst desc-last-iad desc-last-out-iad-cdc desc-iad \
        desc-last2-out desc-last2-out-iad-cdc; do
    case "$mode" in
        host) flags="" ;;
        host-audio) flags="-DTEST_UAC=1" ;;
        device) flags="-DTEST_HOST=0" ;;
        loader) flags="-DFELUCCA_LOADER=1" ;;
        desc-midi) flags="-DTEST_HOST=0 -DTEST_GOLDEN_PLAIN=0xD8A90D6A -DTEST_GOLDEN_CDC=0xB0107332" ;;
        desc-in) flags="$D -DTEST_OUT=0 -DTEST_CDC=0 -DTEST_GOLDEN_PLAIN=0xDBD0AE87" ;;
        desc-in-cdc) flags="$D -DTEST_OUT=0 -DTEST_GOLDEN_PLAIN=0xDBD0AE87 -DTEST_GOLDEN_CDC=0x14DC9CA2" ;;
        desc-in-cdc1) flags="$D -DTEST_OUT=0 -DFELUCCA_USB_LAYOUT=1 -DTEST_GOLDEN_CDC=0x529E01CB" ;;
        desc-in-48k) flags="$D -DTEST_OUT=0 -DTEST_48K=1 -DTEST_GOLDEN_PLAIN=0x211CD46E -DTEST_GOLDEN_CDC=0x09511215" ;;
        desc-asfirst) flags="$D -DTEST_OUT=0 -DFELUCCA_UAC_AS_FIRST=1 -DTEST_GOLDEN_PLAIN=0xC8CC0B95 -DTEST_GOLDEN_CDC=0x0990BD2A" ;;
        desc-out) flags="$D -DTEST_GOLDEN_PLAIN=0xACCA5115 -DTEST_GOLDEN_CDC=0x8CAFAC33" ;;
        desc-out-asfirst-cdc) flags="$D -DFELUCCA_UAC_AS_FIRST=1 -DTEST_GOLDEN_PLAIN=0x9339B801 -DTEST_GOLDEN_CDC=0x062D6670" ;;
        desc-out-48k-asfirst) flags="$D -DTEST_48K=1 -DFELUCCA_UAC_AS_FIRST=1 -DTEST_GOLDEN_PLAIN=0x1CC512F3 -DTEST_GOLDEN_CDC=0x4D64E0C8" ;;
        desc-last) flags="$D -DTEST_OUT=0 -DTEST_CDC=0 -DFELUCCA_UAC_MIDI_LAST=1" ;;
        desc-last-cdc) flags="$D -DTEST_OUT=0 -DFELUCCA_UAC_MIDI_LAST=1" ;;
        desc-last-cdc1) flags="$D -DTEST_OUT=0 -DFELUCCA_UAC_MIDI_LAST=1 -DFELUCCA_USB_LAYOUT=1" ;;
        desc-last-48k) flags="$D -DTEST_OUT=0 -DTEST_48K=1 -DFELUCCA_UAC_MIDI_LAST=1" ;;
        desc-last-out) flags="$D -DTEST_CDC=0 -DFELUCCA_UAC_MIDI_LAST=1" ;;
        desc-last-out-cdc) flags="$D -DFELUCCA_UAC_MIDI_LAST=1" ;;
        desc-last-out-48k-cdc2) flags="$D -DTEST_48K=1 -DFELUCCA_UAC_MIDI_LAST=1 -DFELUCCA_USB_LAYOUT=2" ;;
        desc-last-asfirst) flags="$D -DFELUCCA_UAC_MIDI_LAST=1 -DFELUCCA_UAC_AS_FIRST=1" ;;
        desc-last-iad) flags="$D -DTEST_OUT=0 -DTEST_CDC=0 -DFELUCCA_UAC_MIDI_LAST=1 -DFELUCCA_UAC_IAD=1" ;;
        desc-last-out-iad-cdc) flags="$D -DFELUCCA_UAC_MIDI_LAST=1 -DFELUCCA_UAC_IAD=1" ;;
        desc-iad) flags="$D -DFELUCCA_UAC_IAD=1" ;;
        desc-last2-out) flags="$D -DTEST_CDC=0 -DFELUCCA_UAC_MIDI_LAST=2" ;;
        desc-last2-out-iad-cdc) flags="$D -DFELUCCA_UAC_MIDI_LAST=2 -DFELUCCA_UAC_IAD=1" ;;
    esac
    # Word splitting is intentional for the fixed compiler options above.
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function \
        -Wno-unused-variable -Wno-missing-field-initializers \
        $SANITIZE -Ichip -Itests/stub $flags tests/usb_host_test.c -o "$USB_TEST_DIR/$mode"
    echo "$mode:"
    "$USB_TEST_DIR/$mode"
done
