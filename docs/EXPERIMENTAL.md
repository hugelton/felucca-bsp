# Experimental features

Status of the advanced features planned for this repository. Nothing here has run on a device; each item says what is
established, what is guessed, and what the first bench test is. All of it stays behind explicit calls: no firmware
starts any of it by itself.

## Second core (CPU1): `chip/fm1_cpu1.h`, `chip/fm1_cpu1.S`

Established from the SDK (`EnableOtherCpu` in `system.a`, `cpu1_start` in `cpu.a`): the entry word at `0x01C7FFF8`,
the SYS_DIV bit 3 during the start, CORE_CON `0x1EEE004` (set bit 3, clear bit 1), the stub that sets `usp`, `sp`,
`reti` and does `rti`. The stub assembles to the same bytes as the SDK's. `examples/hwtest` has it behind the serial
command `k` (CPU1 counts, the status line shows `cpu1=`).
Unknown: CPU1's interrupt bank, whether the guard windows of `fm1_guard.h` apply to it, cache behaviour between the
cores. First test: `k`, then `p` twice; the count must grow.

## Expanded flash: `chip/fm1_flash_ext.h`

For a larger part soldered in place of the stock SOIC-8 flash. Done: size from the JEDEC id, the range check for the
room above the stock 1 MiB, a host test (`tests/flash_ext_test.c`). Not done: hooking it into the erase/program
entry points of `fm1_flash.h`, 4-byte addressing above 16 MiB.
Unknown: how much flash the XIP window maps, the quad-enable bit of other vendors' parts. First test: read the
JEDEC id of a swapped part and check that the stock layout still boots.

## USB host

The USB0 core has host-mode functions in the SDK (`usb_h_sie_init`, `usb_h_chirp_and_reset`, `usb_h_ep_read/write`
in `cpu.a`), on the same controller `chip/usb.c` drives as a device. Not started. Unknown and possibly blocking: the
FM-1's USB-C port may not be able to supply 5 V (VBUS) or to switch to host at all. First step: look at the board.

## BLE

See `ble/` and `examples/hwtest-ble/`: a host that runs on a PC against a fake controller, and a survey of what
the JieLi controller needs. Concept only.

## Clock up to 320 MHz

The SDK sets the clock from its own tables (`clk_set`, `sys_clock_update` in `cpu.a`; 320 MHz is its maximum for the
system clock). The PLL, voltage and flash-timing changes it makes are not reproduced here. Raising the clock also
moves everything derived from it: the SFC (flash) timing, SPI and UART dividers, USB's 48 MHz, the audio clock.
Plan: first measure the running clock (a calibrated loop against TIMER4's 24 MHz), then read the SDK's sequence for
one step up, and try it from a serial command with an automatic fall-back after a short time.
