# hwtest

A small FM-1 firmware that exercises this repository on real hardware. It is a test, not a synth.

| Part | What you see |
| --- | --- |
| Screen | Six colour bars (red, green, blue, white, yellow, cyan) at the top: panel and colour order. |
| Knob, battery | Two bars: MASTER pot and battery level (ADC). |
| Buttons, keys | 14 button cells and 27 key cells light while pressed. |
| Encoders | 7 cells; a click flashes green (clockwise) or red, and moves a bar. |
| LEDs | A walk over all 41 LEDs at power-on, then each key lights its own LED. |
| USB | CDC serial, 115200 8N1 (the rate is ignored). A status line every 250 ms. |
| Corner | Bottom right: red = USB not configured, blinking green = configured. Yellow bottom: the last run crashed. |

Serial commands: `h` help, `p` print now, `c` last crash, `x` crash on purpose (tests the fault report),
`w` LED walk, `u` UBOOT, `r` reboot. The SysEx soft key (UBOOT) works too.

Two boots in a row that do not reach the main loop's healthy point (2 s) drop into UBOOT, so a bad build
can be replaced.

## Build

    JIELI_TOOLCHAIN=~/.jieli/toolchain examples/hwtest/build.sh     # -> examples/hwtest/build/hwtest.bin

About 9 KB. The toolchain is JieLi's own (see Felucca's `tools/get_toolchain.sh`).

## Install

This repository has no packer yet. Use Felucca's tools (GPL-3.0, outside this repository) with a loader built
from `loader/`:

    python3 tools/fm1pkg_make.py hwtest.bin loader.bin hwtest.fwsc --product FM-1_9T0
    python3 tools/fm1_install.py hwtest.fwsc

## Status

Built and packaged; not yet run on a device. The first run is the test. Expected: colour bars, then the
LED walk; if nothing appears, reset twice and the device enters UBOOT.

## Licence

The sources are MPL-2.0, but the image includes `chip/usb.c` (GPL-3.0-only), so a built firmware is
GPL-3.0 as a whole.
