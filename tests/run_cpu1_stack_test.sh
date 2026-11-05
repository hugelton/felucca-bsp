#!/bin/sh
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Henri David
# FM1_CPU1_USTACK_BYTES (chip/fm1_cpu1.h, fm1_cpu1.S) on the host's compiler: unset, the jobs' stack is 2,816 B in
# both files; a build's size (1024, 4096, 16384) reaches the .S's .space and FM1_CPU1_USTACK_WORDS alike; a size that
# is not a multiple of 8, or not above the 256 B guard, stops the build with the static check's message. Not the
# second core: only the AC79 toolchain assembles fm1_cpu1.S into code. An ELF host (Linux, as the workflows): on
# macOS the chip headers' section names do not build.
set -eu
cd "$(dirname "$0")/.."
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT HUP INT TERM
CC="${CC:-cc}"
fails=0
say() { printf '  %s %s\n' "$1" "$2"; [ "$1" = ok ] || fails=$((fails + 1)); }
printf '#include "fm1_cpu1.h"\n_Static_assert(FM1_CPU1_USTACK_WORDS * 4u == WANT, "FM1_CPU1_USTACK_WORDS");\n' > "$T/words.c"
# a size the build may give: the .S's .space (the jobs' stack, then the 256 B of exceptions) and the header's words
for n in default 1024 2816 4096 16384; do
    if [ "$n" = default ]; then d=""; want=2816; else d="-DFM1_CPU1_USTACK_BYTES=$n"; want=$n; fi
    space=$("$CC" -E -x assembler-with-cpp -Ichip $d chip/fm1_cpu1.S | grep -m1 '\.space' | tr -d ' \t')
    if [ "$space" = ".space$want" ]; then say ok "$n: fm1_cpu1.S's jobs' stack .space $want"; else
        say FAIL "$n: fm1_cpu1.S's jobs' stack ($space, wanted .space $want)"; fi
    if "$CC" -std=c11 -fsyntax-only -Ichip $d -DWANT="${want}u" "$T/words.c" 2> "$T/err"; then
        say ok "$n: FM1_CPU1_USTACK_WORDS = $want / 4"; else say FAIL "$n: FM1_CPU1_USTACK_WORDS ($(head -3 "$T/err"))"; fi
done
# a size the static check refuses: the build stops, naming the macro
for n in 1004 2820 256 0; do
    if "$CC" -std=c11 -fsyntax-only -Ichip -DFM1_CPU1_USTACK_BYTES=$n -DWANT="${n}u" "$T/words.c" 2> "$T/err"; then
        say FAIL "$n: built (the static check let it through)"
    elif grep -q 'FM1_CPU1_USTACK_BYTES: a multiple of 8, above the 256 B guard' "$T/err"; then
        say ok "$n: refused by the static check"
    else
        say FAIL "$n: refused, not by the static check ($(head -3 "$T/err"))"
    fi
done
[ $fails -eq 0 ] && echo "cpu1 stack: all passed" || echo "cpu1 stack: $fails FAILED"
exit $fails
