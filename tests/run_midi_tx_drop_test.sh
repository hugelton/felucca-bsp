#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
# chip/usb.c's MIDI IN to the host when nobody reads it (tests/midi_tx_drop_test.c): the default stall polls and two
# others. MIDI_TX_TEST_SANITIZE= (empty) builds without the sanitizers.
set -eu
cd "$(dirname "$0")/.."
D=$(mktemp -d)
trap 'rm -rf "$D"' EXIT HUP INT TERM
SANITIZE=${MIDI_TX_TEST_SANITIZE--fsanitize=address,undefined}
for polls in default 10 255; do
    case "$polls" in default) flags="" ;; *) flags="-DFELUCCA_MIDI_TX_STALL_POLLS=${polls}u" ;; esac
    # Word splitting is intentional for the fixed compiler options above.
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function -Wno-unused-variable \
        -Wno-missing-field-initializers $SANITIZE -Ichip -Itests/stub $flags tests/midi_tx_drop_test.c -o "$D/t"
    "$D/t"
done
