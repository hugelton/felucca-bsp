#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Charles Vestal
# chip/ota.c's handshake on the host (tests/ota_test.c): a firmware's long identity, a short one (a dev build's, three
# digits already, and no flash ID given: OTA_FLASH_ID undefined) and the update loader's; an SMK-37's (boards/smk37/, its
# flash EB6014) and its loader's (loader/ldr_smk37.h). OTA_TEST_SANITIZE= (empty) builds without the sanitizers (on a Mac where
# ASan hangs at start-up).
set -eu
cd "$(dirname "$0")/.."
OTA_TEST_DIR=$(mktemp -d)
trap 'rm -rf "$OTA_TEST_DIR"' EXIT HUP INT TERM
SANITIZE=${OTA_TEST_SANITIZE--fsanitize=address,undefined}
for mode in long short loader smk37 smk37-loader; do
    id=""
    case "$mode" in
        long) flags="" ;;
        short) flags="-DTEST_ID=\"FM-1_600\" -DTEST_JEDEC=0" ;;
        loader) flags="-DFELUCCA_LOADER=1" ;;
        smk37) flags="-DTEST_JEDEC=0xEB6014u"; id="SMK-37 Elite_6010000" ;;
        smk37-loader) flags="-DFELUCCA_LOADER=1 -DTEST_JEDEC=0xEB6014u"; id="ota-SMK-37 Elite_900" ;;
    esac
    # Word splitting is intentional for the fixed compiler options above (an identity with a space goes on its own).
    "${CC:-cc}" -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function $SANITIZE $flags ${id:+"-DTEST_ID=\"$id\""} \
        tests/ota_test.c -o "$OTA_TEST_DIR/$mode"
    echo "$mode:"
    "$OTA_TEST_DIR/$mode"
done
