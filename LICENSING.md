# Licensing

Every file under `chip/`, `boards/`, `loader/` and `examples/` is MPL-2.0 (SPDX header in each file), except
`chip/usb.c`, which is GPL-3.0-only (text in `LICENSES/GPL-3.0-only.txt`). The full text
is in `LICENSE`. The files carry no "Incompatible With Secondary Licenses" notice, so they can
be combined with GPL code under MPL-2.0 section 3.3.

Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments.

The JieLi AC79 SDK is Apache-2.0 and is not part of this repository. See `NOTICE`.

Files were first published in Felucca under GPL-3.0-only. The copyright holder relicensed
these files here; copies already distributed under GPL-3.0-only stay under that licence.

## chip/usb.c

`chip/usb.c` stays GPL-3.0-only for now. Parts of it came into Felucca from contributors who offered them under
GPL-3.0-only; it moves to MPL-2.0 when those parts are replaced or the contributors agree. A firmware that includes it
is GPL-3.0 as a whole. The loader includes it, so a loader image built from this tree is GPL-3.0 too.

A new project that wants to be MPL-2.0 must leave `chip/usb.c` out and supply its own USB code, until this file moves to MPL-2.0.

## Forks of Felucca

Using this repository does not relicense a fork of Felucca. A fork still contains Felucca's code, which is
GPL-3.0-only, and only its copyright holders can change that. Files copied from Felucca's tree are GPL-3.0-only even
when this repository carries the same file under MPL-2.0; only the copy taken from here is MPL-2.0.

To publish under MPL-2.0 (or any other licence), start a new project, import this repository, and write the rest
yourself. Do not copy Felucca's code into it, and leave out `chip/usb.c` (see above). The other direction is fine: MPL-2.0 files from here can go into a
GPL project.

## Tools

`tools/fm1pkg.py` (package builder) and `tools/fm1_install.py` (USB-MIDI installer) are MPL-2.0, moved here from Felucca
by their only author. Packages built by `fm1pkg.py` contain three files of the JieLi AC79 SDK (`uboot.boot`,
`cfg_tool.bin`, `eq_cfg_hw.bin`), which are Apache-2.0: ship `LICENSES/Apache-2.0.txt` with every package.
