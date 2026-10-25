# hwtest

Small FM-1 firmware that exercises this repository on real hardware. It is a test, not a synth. Two images:

| Image | What it has |
| --- | --- |
| `hwtest.bin` — **safe** | Screen, knob, buttons, LEDs, USB serial. Nothing experimental is in the image. Start here. |
| `hwtest-exp.bin` — experimental | The same, plus the beta parts of [docs/EXPERIMENTAL.md](../../docs/EXPERIMENTAL.md): `k` starts the second core, `x` crashes on purpose. Not in the safe image at all. |

| Part | What you see |
| --- | --- |
| Screen | Six colour bars (red, green, blue, white, yellow, cyan) at the top: panel and colour order. |
| Knob, battery | Two bars: MASTER pot and battery level (ADC). |
| Buttons, keys | 14 button cells and 27 key cells light while pressed. |
| Encoders | 7 cells; a click flashes green (clockwise) or red, and moves a bar. |
| LEDs | A walk over all 41 LEDs at power-on, then each key lights its own LED. |
| USB | CDC serial, 115200 8N1 (the rate is ignored). A status line every 250 ms. The update entry (M-UPGRADE over USB-MIDI): the installer replaces hwtest, and a bar across the bottom of the screen shows the steps. |
| Corner | Bottom right: red = USB not configured, blinking green = configured. Yellow bottom: the last run crashed. |

Panel initialization and rectangle transfers use the reusable board driver
`boards/fm1/fm1_lcd.h`. The example contains only its colours and layout.
See [board I/O contracts](../../docs/BOARD_IO.md) for image transfers and the
audio API (hwtest does not enable audio).

Serial commands (both images): `h` help, `p` print now, `c` last crash, `w` LED walk, `u` UBOOT, `r` reboot.
The SysEx soft key (UBOOT) works too. Only in `hwtest-exp`: `k` start the second core (each `p` then hands it a thousand jobs: `cpu1=` counts them, `late=`
the ones not back in time, `gave_up=` why it was retired, `rejected=` its misreads, `stack=` its depth),
`x` crash on purpose (tests the fault report).

Two boots in a row that do not reach the main loop's healthy point (2 s) drop into UBOOT, so a bad build
can be replaced. hwtest has the update entry, so the installer replaces it over USB-MIDI like any firmware. Only if it ends in
UBOOT (two failed boots, or `u`) does it need `tools/fm1_rescue.sh` (macOS): the stock firmware, then install over it.

## Build

    JIELI_TOOLCHAIN=~/.jieli/toolchain examples/hwtest/build.sh     # -> build/hwtest.bin and build/hwtest-exp.bin

About 9 KB each. The toolchain is JieLi's own (see Felucca's `tools/get_toolchain.sh`).

## Install

Build the loader (`loader/`, see the README), then pack and install with the tools in this repository:

    tools/get_sdk.sh                                  # JieLi's SDK, fetched from JieLi, checked by hash (~0.5 GB)
    AC79_SDK=~/.jieli/ac79-sdk python3 tools/fm1pkg.py hwtest.bin loader.bin hwtest.fwsc --product FM-1_1200     # hwtest-exp: FM-1_1210
    python3 tools/fm1_install.py hwtest.fwsc          # needs mido and python-rtmidi

`fm1pkg.py` takes three small files from the JieLi AC79 SDK (Apache-2.0) and puts them in the package.

## Status

Runs on an FM-1 (2026-10-16): it stays up, the serial status and commands work, and the installer replaces it. Run the safe image first. The first run is the test. Expected: colour bars, then the
LED walk; if nothing appears, reset twice and the device enters UBOOT.

## Licence

MPL-2.0, like the rest of the repository.
