# Experimental features

Status of the advanced features planned for this repository. **Beta**: the second core (runs in Melodee; this version not yet on a device) and expanded flash (code exists, never run on a device). **Concept**: USB host, the clock (nothing runs). Each item says what is
established, what is guessed, and what the first bench test is. All of it stays behind explicit calls: no firmware
starts any of it by itself.

## Second core (CPU1): `chip/fm1_cpu1.h`, `chip/fm1_cpu1.S`, `chip/fm1_cpu1_stop.h`

Melodee's second core (keremimo, under MPL-2.0 by agreement: #5), which runs on the device in Melodee, with the API
and the start X0X, NuSaw, SPH-16 and LOOM-8 use: a worker that runs one job at a time for CPU0 (`fm1_cpu1_run`,
`fm1_cpu1_wait`). Established on the device (Melodee, X0X): the start at power-on, before the audio and the timers,
with the PC limits open while CPU1 boots through the ROM; CPU1's own guards (the stack limit in its EMU bank);
holding it first thing at a start, and safely (the delayed bus flag cleared: `fm1_cpu1_stop`); the units whose CPU1
misreads shared RAM (keremimo/melodee#17, charlesvestal/fm1-x0x#10, measured by Jangada): a check word on each job,
retirement on a misread, a fault or a late job, one core until power-off after that (`fm1_cpu1_bar`), CPU1's faults
handled without a crash (`fm1_irq.h` with `FM1_CPU1`, the resume in `fm1_vec.S`).
This file as a whole has not run on a device yet. First test: `hwtest-exp`, `k`, then `p` a few times: `cpu1=` grows
by 1000 each time, `late=`, `rejected=` and `faults=` stay 0 (on a unit that misreads: `gave_up=1`, one core).

## Expanded flash: `chip/fm1_flash_ext.h`

For a larger part soldered in place of the stock SOIC-8 flash. Done: size from the JEDEC id, the range check for the
room above the stock 1 MiB, a host test (`tests/flash_ext_test.c`). The runtime driver accepts an explicit physical capacity and application write
regions up to 16 MiB ([configuration](FLASH.md#replacement-flash-parts)); JEDEC discovery does not automatically
widen write permissions. Not done: hardware qualification and 4-byte addressing above 16 MiB.
Unknown: how much flash the XIP window maps, the quad-enable bit of other vendors' parts. First test: read the
JEDEC id of a swapped part and check that the stock layout still boots.

## USB host

The USB0 core has host-mode functions in the SDK (`usb_h_sie_init`, `usb_h_chirp_and_reset`, `usb_h_ep_read/write`
in `cpu.a`), on the same controller `chip/usb.c` drives as a device. Not started. Unknown and possibly blocking: the
FM-1's USB-C port may not be able to supply 5 V (VBUS) or to switch to host at all. First step: look at the board.

## Bluetooth

Not here. Bluetooth MIDI that runs on the FM-1, on JieLi's own controller and stack (binary, Apache-2.0), is in
[fm1-base](https://github.com/charlesvestal/fm1-base); what it took is in
[#2](https://github.com/hugelton/felucca-bsp/issues/2). The earlier concept of a host of our own over HCI is in
this repository's history.

## Clock up to 320 MHz

The SDK sets the clock from its own tables (`clk_set`, `sys_clock_update` in `cpu.a`; 320 MHz is its maximum for the
system clock). The PLL, voltage and flash-timing changes it makes are not reproduced here. Raising the clock also
moves everything derived from it: the SFC (flash) timing, SPI and UART dividers, USB's 48 MHz, the audio clock.
Plan: first measure the running clock (a calibrated loop against TIMER4's 24 MHz), then read the SDK's sequence for
one step up, and try it from a serial command with an automatic fall-back after a short time.
