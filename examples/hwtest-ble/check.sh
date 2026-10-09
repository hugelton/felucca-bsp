#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Compile ble_glue.c against the JieLi SDK headers (a check that the glue matches the controller's interface).
#   AC79_SDK=~/.jieli/ac79-sdk JIELI_TOOLCHAIN=~/.jieli/toolchain examples/hwtest-ble/check.sh
set -e
S="${AC79_SDK:?run tools/get_sdk.sh and set AC79_SDK}"
TC="${JIELI_TOOLCHAIN:?JIELI_TOOLCHAIN must point at the JieLi Linux toolchain}"
HERE="$(cd "$(dirname "$0")" && pwd)"
L="$S/include_lib"
"$TC/pi32v2/bin/clang" -target pi32v2 -Os -Wall -I"$HERE/../../ble" -I"$L/system/generic" -I"$L/btctrler" -I"$L/system" \
    -I"$L/driver/cpu/wl82" -I"$L/driver" -I"$L" -I"$S/apps/include_lib" -I"$L/btstack" -I"$L/newlib/include" \
    -c "$HERE/ble_glue.c" -o /tmp/ble_glue.o
echo "ble_glue.c compiles against the SDK headers"
