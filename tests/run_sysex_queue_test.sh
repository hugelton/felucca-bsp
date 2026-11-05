#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Charles Vestal
# chip/usb.c's queue of received SysEx frames on the host (tests/sysex_queue_test.c): buffers from 300 to 2000
# bytes (FELUCCA_SX_IN), with frames longer than the guaranteed size and without, and the update loader's build (4744:
# the fast install's 4 KiB replies).
# The test is built from a copy of usb.c with TEST_PREEMPT() after each statement of ota_frame_get and
# ota_frame_done, where the ISR may run (the main loop's side of the queue). SYSEX_TEST_SANITIZE= (empty) builds
# without the sanitizers (on a Mac where ASan hangs at start-up).
set -eu
cd "$(dirname "$0")/.."
SYSEX_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$SYSEX_TEST_DIR"' EXIT HUP INT TERM
SANITIZE=${SYSEX_TEST_SANITIZE--fsanitize=address,undefined}
awk '/^static int ota_frame_get\(/ { f = 1 }
     { print }
     f && /;[ \t]*(\/\*.*\*\/)?[ \t]*$/ { print "    TEST_PREEMPT();"; n++ }
     f && /^}/ { if (++done == 2) f = 0 }
     END { if (n < 8) { print "run_sysex_queue_test.sh: ota_frame_get / ota_frame_done not found" > "/dev/stderr"; exit 1 } }' \
    chip/usb.c > "$SYSEX_TEST_DIR/usb.c"
for size in 300 301 512 648 1000 2000 loader; do
    for long in 0 3; do
        case "$size" in
            loader) flags="-DFELUCCA_LOADER=1" ;;
            *) flags="-DFELUCCA_SX_IN=${size}u" ;;
        esac
        # Word splitting is intentional for the fixed compiler options above.
        "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function -Wno-unused-variable \
            -Wno-missing-field-initializers $SANITIZE -Ichip -Itests/stub $flags -DTEST_LONG=${long}u \
            -DTEST_USB_C="\"$SYSEX_TEST_DIR/usb.c\"" tests/sysex_queue_test.c -o "$SYSEX_TEST_DIR/q"
        echo "$size, ${long}% longer:"
        "$SYSEX_TEST_DIR/q"
    done
done
