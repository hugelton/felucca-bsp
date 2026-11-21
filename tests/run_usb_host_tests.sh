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
for mode in host host-audio host-name device loader \
        desc-midi desc-in desc-in-cdc desc-in-cdc1 desc-in-48k desc-asfirst desc-out desc-out-asfirst-cdc \
        desc-out-48k-asfirst desc-last desc-last-cdc desc-last-cdc1 desc-last-48k desc-last-out \
        desc-last-out-cdc desc-last-out-48k-cdc2 desc-last-asfirst desc-last-iad desc-last-out-iad-cdc desc-iad \
        desc-last2-out desc-last2-out-iad-cdc desc-ch10 desc-ch10-last-cdc desc-ch4-deep desc-serial desc-serial-cdc; do
    case "$mode" in
        host) flags="" ;;
        host-audio) flags="-DTEST_UAC=1" ;;
        host-name) flags="-DFELUCCA_USB_HOST_NAME=1" ;;
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
        desc-ch10) flags="$D -DTEST_OUT=0 -DTEST_CDC=0 -DFELUCCA_UAC_CH=10" ;;
        desc-ch10-last-cdc) flags="$D -DFELUCCA_UAC_CH=10 -DFELUCCA_UAC_MIDI_LAST=1 -DFELUCCA_UAC_AS_FIRST=1" ;;
        desc-ch4-deep) flags="$D -DTEST_OUT=0 -DFELUCCA_UAC_CH=4 -DFELUCCA_UAC_DEEP=1" ;;
        desc-serial) flags="-DTEST_HOST=0 -DTEST_SERIAL=1" ;;
        desc-serial-cdc) flags="$D -DTEST_SERIAL=1" ;;
    esac
    # Word splitting is intentional for the fixed compiler options above.
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function \
        -Wno-unused-variable -Wno-missing-field-initializers \
        $SANITIZE -Ichip -Itests/stub $flags tests/usb_host_test.c -o "$USB_TEST_DIR/$mode"
    echo "$mode:"
    "$USB_TEST_DIR/$mode"
done
# USB audio (tests/uac_test.c): the default configuration (byte for byte the descriptors before FELUCCA_UAC_CH and
# FELUCCA_UAC_24), the input alone, and the channel counts and sample sizes around it, with and without the console;
# on the host's clock (FELUCCA_UAC_HOST_CLOCK, TEST_HC: minutes of a model host and crystal) by default, in 4 channels
# 24-bit, and in 8 without the output; the output with its feedback (FELUCCA_UAC_OUT_FEEDBACK, TEST_FB: minutes of a
# model host that follows it) by default, in 4 channels 24-bit, and with the input on the host's clock too; the
# output in 24-bit too (FELUCCA_UAC_OUT_24, TEST_O24) adaptive, with the feedback, and with all of them; the input
# and the output as two functions (FELUCCA_UAC_SPLIT, TEST_SPLIT), named or not, alone, and with all of the above;
# the rings lent by the app (FELUCCA_UAC_MEM, TEST_MEM) in stereo, in 4 channels with the options on, and in 8 without
# the output; the stereo input as a capture, and the streaming interfaces first (FELUCCA_UAC_AS_FIRST), for the check below.
# Each run prints its bcdDevice and a CRC of its descriptors (bcdDevice and the serial string's index left out): two
# configurations with one bcdDevice must have the same descriptors, so a host never takes two for one
for flags in "" "-DTEST_OUT=0" "-DTEST_CH=4" "-DTEST_CH=4 -DTEST_24=1" "-DTEST_CH=4 -DTEST_24=1 -DTEST_CDC=1" "-DTEST_CH=4 -DTEST_TAP=1 -DTEST_OUT=0" "-DTEST_CH=8 -DTEST_TAP=1 -DTEST_OUT=0" \
             "-DTEST_CH=2 -DTEST_24=1" "-DTEST_CH=8" "-DTEST_HC=1" "-DTEST_HC=1 -DTEST_CH=4 -DTEST_24=1" \
             "-DTEST_HC=1 -DTEST_CH=8 -DTEST_OUT=0" "-DTEST_FB=1" "-DTEST_FB=1 -DTEST_CH=4 -DTEST_24=1" \
             "-DTEST_FB=1 -DTEST_HC=1 -DTEST_CH=4 -DTEST_24=1" "-DTEST_O24=1" "-DTEST_FB=1 -DTEST_O24=1" \
             "-DTEST_FB=1 -DTEST_O24=1 -DTEST_HC=1 -DTEST_CH=4 -DTEST_24=1" "-DTEST_SPLIT=1" \
             "-DTEST_SPLIT=1 -DTEST_NAMES=1" "-DTEST_SPLIT=1 -DTEST_OUT=0" "-DTEST_SPLIT=1 -DTEST_NAMEFN=1" \
             "-DTEST_SPLIT=1 -DTEST_NAMEFN=1 -DTEST_OUT=0 -DTEST_CH=4" "-DTEST_NAMEFN=1" "-DTEST_NAMEFN=1 -DTEST_SERIAL=1" \
             "-DTEST_SPLIT=1 -DTEST_NAMES=1 -DTEST_NAMEFN=1 -DTEST_SERIAL=1 -DTEST_CH=4" \
             "-DTEST_SPLIT=1 -DTEST_NAMES=1 -DTEST_FB=1 -DTEST_O24=1 -DTEST_HC=1 -DTEST_CH=4 -DTEST_24=1" \
             "-DTEST_MEM=1" "-DTEST_MEM=1 -DTEST_OUT=0" "-DTEST_MEM=1 -DTEST_CH=4 -DTEST_24=1 -DTEST_HC=1 -DTEST_FB=1 -DTEST_O24=1" \
             "-DTEST_MEM=1 -DTEST_CH=8 -DTEST_OUT=0" \
             "-DFELUCCA_UAC_CAPTURE=1 -DTEST_GOLDEN=0" "-DFELUCCA_UAC_AS_FIRST=1 -DTEST_GOLDEN=0"; do
    # Word splitting is intentional for the fixed compiler options above.
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function \
        -Wno-unused-variable -Wno-missing-field-initializers \
        -fsanitize=address,undefined -Ichip $flags tests/uac_test.c -o "$USB_TEST_DIR/uac" -lm
    "$USB_TEST_DIR/uac" > "$USB_TEST_DIR/uac.run"       # (not through a pipe: its exit status is the test's)
    cat "$USB_TEST_DIR/uac.run"
    cat "$USB_TEST_DIR/uac.run" >> "$USB_TEST_DIR/uac.out"
done
awk '/^bcdDevice /{ if ($2 in crc && crc[$2] != $3) { print "bcdDevice " $2 " is shared by two different descriptors"; bad = 1 } crc[$2] = $3; n++ }
     END { if (bad || !n) exit 1; print "bcdDevice: " n " configurations, each bcdDevice has one set of descriptors: OK" }' "$USB_TEST_DIR/uac.out"
# A combination that cannot be built says so (#error), instead of failing in the middle of the driver: FELUCCA_UAC_MEM lends
# the ring of the stereo or FELUCCA_UAC_CAPTURE input, not that of FELUCCA_UAC_CH 4..10 without FELUCCA_UAC_CAPTURE
# (TEST_TAP); FELUCCA_UAC_CAPTURE is 2, 4 or 8 channels
for flags in "-DTEST_MEM=1 -DTEST_CH=4 -DTEST_TAP=1:FELUCCA_UAC_MEM lends" "-DTEST_CH=6:FELUCCA_UAC_CAPTURE: FELUCCA_UAC_CH is 2, 4 or 8"; do
    msg=${flags#*:}
    flags=${flags%%:*}
    # Word splitting is intentional for the fixed compiler options above.
    if "${CC:-cc}" -std=c11 -O1 -Wno-unused-function -Ichip $flags -c tests/uac_test.c -o "$USB_TEST_DIR/never.o" 2> "$USB_TEST_DIR/err"; then
        echo "uac_test $flags: built, an #error was wanted"
        exit 1
    fi
    grep -q "error: .*$msg" "$USB_TEST_DIR/err" || { echo "uac_test $flags: failed, but not with the #error '$msg':"; cat "$USB_TEST_DIR/err"; exit 1; }
    echo "uac_test $flags: #error '$msg': OK"
done
