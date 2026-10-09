#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Fetch the JieLi AC79 SDK (Apache-2.0, from JieLi's own repository) at the revision this repository was tested with.
# The SDK is not copied into this repository: you fetch it from JieLi, and its LICENSE and notices stay with it.
#   tools/get_sdk.sh [DEST]      (default: ~/.jieli/ac79-sdk)     then: export AC79_SDK=DEST
#   SDK_URL=...   another remote (a mirror)       SDK_REV=...   another revision (the checks below then fail)
set -e
DEST="${1:-$HOME/.jieli/ac79-sdk}"
URL="${SDK_URL:-https://github.com/Jieli-Tech/fw-AC79_AIoT_SDK}"
REV="${SDK_REV:-d179b4484759423312073f5fbb232501aa491047}"      # AC79NN_SDK_V1.2.1_2023-12-13
mkdir -p "$DEST"
cd "$DEST"
[ -d .git ] || git init -q .
git fetch -q --depth 1 --filter=blob:none "$URL" "$REV"
git sparse-checkout init --no-cone
git sparse-checkout set /LICENSE /README.md /cpu/wl82/ /include_lib/ /apps/include_lib/
git checkout -q FETCH_HEAD
check() { echo "$2  $1" | sha256sum -c - >/dev/null || { echo "get_sdk: $1 is not the file this repository was tested with" >&2; exit 1; }; }
check cpu/wl82/tools/uboot.boot       4e3b4c220dc96641cb5a723f41e68ce41d5261ae9434bb33fbd7f2c59976ded4
check cpu/wl82/tools/cfg_tool.bin     276579954f076886a6a7694f65dc71c034a63a2c204b76749065c0ac7b010d1b
check cpu/wl82/tools/cfg/eq_cfg_hw.bin 41167491bffed4651750719c973d2758adeb9021a5670d02d6a53c85ed80ea7d
check cpu/wl82/liba/btctrler.a        9ef4262444b49171c4a5eab906e3939b1d4943350ac4993c18d8474d01d773de
check cpu/wl82/liba/btstack.a         c3f4ebfade12823c0b160f202bbe626b74bb2dfc5e509a034af965c5da852a2b
echo "AC79_SDK=$DEST ($(git rev-parse --short HEAD), Apache-2.0: see $DEST/LICENSE)"
