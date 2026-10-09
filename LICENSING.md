# Licensing

Every file under `chip/`, `boards/`, `loader/` and `examples/` is MPL-2.0 (SPDX header in each file), except
`chip/usb.c`, which is GPL-3.0-only (text in `LICENSES/GPL-3.0-only.txt`). The full text
is in `LICENSE`. The files carry no "Incompatible With Secondary Licenses" notice, so they can
be combined with GPL code under MPL-2.0 section 3.3.

Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments.

The JieLi AC79 SDK is Apache-2.0 and is not part of this repository. See `NOTICE`.

Files were first published in Felucca under GPL-3.0-only. The copyright holder relicensed
these files here; copies already distributed under GPL-3.0-only stay under that licence.

`chip/usb.c` stays GPL-3.0-only for now. Parts of it came into Felucca from contributors who offered them under
GPL-3.0-only; it moves to MPL-2.0 when those parts are replaced or the contributors agree. A firmware that includes it
is GPL-3.0 as a whole. The loader includes it, so a loader image built from this tree is GPL-3.0 too.

## Forks of Felucca

Using this repository does not relicense a fork of Felucca. A fork still contains Felucca's code, which is
GPL-3.0-only, and only its copyright holders can change that. Files copied from Felucca's tree are GPL-3.0-only even
when this repository carries the same file under MPL-2.0; only the copy taken from here is MPL-2.0.

To publish under MPL-2.0 (or any other licence), start a new project, import this repository, and write the rest
yourself. Do not copy Felucca's code into it. The other direction is fine: MPL-2.0 files from here can go into a
GPL project.
