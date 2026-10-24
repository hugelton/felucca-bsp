# USB MIDI host

Enable `FELUCCA_USB_HOST=1` before including `chip/usb.c` in the application's single
compilation unit. The default is 0; `FELUCCA_LOADER` always forces it to 0. There is
no UI, saved setting or automatic mode change in the BSP. Boot starts as a device.

## Application contract

Use the existing `usb_start()`, `usb_poll()` and `usb_retry(now_ms)` device API.
For host mode:

- Keep `fm1_ms` advancing in an interrupt, including during enumeration. It must use
  the same monotonic millisecond clock as `now_ms`. Host waits and EP0 timeouts feed
  the watchdog; without that interrupt clock they cannot finish.
- Call `usb_host_switch(1)` from the main loop to request host mode, or pass 0 to
  return to the computer-facing device. Call `usb_host_task(now_ms)` every main-loop
  iteration, followed by `usb_retry(now_ms)`. Switching off makes device retry due
  immediately. Do not call `usb_start()` directly while hosting.
- Keep calling `usb_poll()` from the existing 2 kHz USB poll interrupt on CPU0.
  Main-loop enumeration owns the SIE until it publishes the connected state;
  afterwards only that poll interrupt accesses the host endpoints. Keep audio and
  other interrupt code out of the SIE. Enumeration blocks the main loop, sometimes
  for seconds with a hub or an unresponsive device; interrupts continue running.
- `usb_host_active()` follows the selected port mode; `usb_host_connected()` means
  that a MIDI endpoint is running. `usb_host_message()` consumes the latest
  `UH_MSG_*` transition (NONE, ON, OFF, MIDI, NOMIDI, LOW or LOST). It is one latest
  message, not a queue. The application chooses how to present it.
- Incoming channel messages and clock/start/continue/stop enter the existing MIDI
  ring with source 1 and `fm1_ms` timestamps. Drain it normally. Host input drops
  cable numbers and ignores SysEx, including update/editor commands.
- Use `midi_out_try(packet)` to enqueue a four-byte USB MIDI 1.0 event on cable 0;
  0 means disconnected, no OUT endpoint or ring full. `midi_out_event` retains its
  existing fire-and-forget API. `midi_out_room()` and `midi_out_link()` expose output
  capacity and connection generation. The application supplies valid event packets.
- Consume and clear `midi_usb_lost` in the CPU0 MIDI consumer. Discard input events
  from the old USB session and release its held notes/pedals while retaining local
  and TRS owners. Arrange output note/pedal cleanup on a changed link before new
  notes: a lost session can include an OUT packet with an uncertain ACK. The BSP
  drops stale queued output; it cannot reconstruct the application's note owners.
- `usb_detach()` stops either mode and prevents further task/retry attachment.

Host entry stops USB audio streams, CDC and pending device SysEx/mirror traffic.
CDC descriptor preferences can change while hosting; they apply on return to device
mode without touching host DMA. USB device audio services stay gated by `usb.up` and
`usb.config`, both zero while hosting. Keep using the existing `RING_PUBLISH()` barrier.

## Devices and power

Full-speed USB MIDI 1.0 streaming interfaces, bulk or interrupt IN and/or OUT, are
supported, directly or behind one hub (up to seven ports). The first usable
interface/alternate setting wins; its endpoints share one device address. Endpoint
packets are at most 64 bytes. Configuration descriptors are read up to 320 bytes.
Output-only synths and input-only controllers are supported.

The PHY setup does not supply 5 V VBUS. Use a powered hub or powered adapter. USB-C
CC wiring and power negotiation depend on the board and adapter. Low-speed devices,
nested hubs, MIDI 2.0, controller SysEx and device-specific LED protocols are not
supported. While one device is active, another hub port is not scanned.

OUT DMA and queue slots are retained until ACK. NAK/transient errors retry the same
packet/toggle. A stall, 16 errors or 500 ms without completion stops the session and
returns recovery to the main loop. Input back-pressure leaves RX data pending.
Disconnect detection requires 16 idle samples (8 ms at 2 kHz), so one SE0 sample
during a packet does not disconnect it. Connection faults trigger re-enumeration.

## Verification

Run `tests/run_usb_host_tests.sh` with a host C compiler. It runs the production BSP
USB source against a keyboard/hub/storage SIE model under ASan/UBSan: parsing,
malformed/truncated descriptors, full input ring, direct/hub duplex I/O, ACK-owned
output, NAK/errors/stalls/timeouts, input/output-only endpoints, disconnect/reconnect,
audio/CDC mode handover, host-disabled builds and forced device-only loaders.

Melodee's original host input was reported working by its contributor and maintainer.
This adaptation and the later output logic still need physical testing: powered-hub
input, output to a synth, simultaneous input/output, unplug/replug, and return to
computer-facing MIDI/audio/update. Measure interrupt deadlines under sustained MIDI
traffic in the consuming firmware; host-model tests cannot establish real-time audio
headroom. No package is flashed or published by this port.

Source commits, both authors and MPL permission are recorded in [NOTICE](../NOTICE).
