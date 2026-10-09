# ble (concept: does not run on a device)

A minimal BLE-MIDI sender: a peripheral that advertises, takes one connection, serves a small GATT table (Generic
Access and the BLE-MIDI service) and sends MIDI as notifications. About 3 KB of code on pi32v2, no malloc, no vendor code.

It speaks standard HCI (H4 packet types), so it needs a controller below it and nothing else:

    bleh_init(&h, send_fn, ctx, name)   bleh_start(&h)   bleh_rx(&h, h4_type, pkt, len)
    bleh_midi_send(&h, msg, len, ts13)   bleh_poll(&h)

Fixed on purpose: ATT MTU 23, no pairing, no scanning, one connection, MIDI messages of 1..3 bytes (no SysEx).

## Status: concept

- Tested on the host against a fake controller and a scripted central (`tests/run_ble_test.sh`, 193 checks, ASan and UBSan).
- Compiles for pi32v2.
- **Does not run on a device, and nothing here is promised to.** The part that is missing is the link to a controller: the FM-1's radio controller is
  JieLi's closed library, which is not in this repository and is not redistributed here. Whoever builds a BLE firmware
  supplies it (`tools/get_sdk.sh` fetches JieLi's SDK from JieLi; nothing of it is copied here) and an `hci_send` function for it. Whether the vendor controller accepts these HCI packets as written
  is the open question the first hardware spike answers.
- Radio is off by default in any firmware. A BLE build is a separate build, and the radio rules of your country
  (for example, Japan's 技適 or an equivalent approval) apply to the device you put it on, not to this code.

## Licence

MPL-2.0, same as the rest. Written for this project; no third-party code.
