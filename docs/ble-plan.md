# BLE: making the controller light (plan)

Status: **plan only**. Nothing here has been built or run. Related: `ble/`, `examples/hwtest-ble/`, `docs/EXPERIMENTAL.md`.

## Goal

BLE-MIDI send only, one peripheral connection, no pairing. Smaller than the SDK demo's controller (about 55 KB of
flash in the demo's configuration), and RAM that fits the room an application leaves (about 60 KB at best in Felucca 1.5).

## What the libraries are

`btctrler.a`, `cpu.a`, `system.a` and the rest are LLVM bitcode built by JieLi and published in the SDK repository,
which declares Apache-2.0. We use them as the SDK's copyright holder published them. They are not copied into this
repository: `tools/get_sdk.sh` fetches them from JieLi and checks hashes. If we ever modify one, Apache-2.0 allows it;
the modified file must say so. No explicit sentence about the `.a` files exists; this plan relies on the repository-level
declaration.

## Steps, in this order

1. **Link as it is.** Write `stubs.c` and a linker script for the 343 symbols the closure leaves undefined
   (`examples/hwtest-ble/closure.py`): 48 `config_*` values, 108 log tags (empty), 41 section markers, 146 functions.
   Link `ble_glue.c` with the controller. Record flash, RAM and the member list. This is the baseline.
2. **Fix the configuration so LTO can drop code.** The `config_*` values are meant to be link-time constants
   (`btcontroller_modules.h`). Set: LE only (no classic), peripheral role only, one link, the smallest packet length and
   buffer counts, no TWS, no extended advertising, no ISO, no AFH user control, no test modes. Link again; compare
   with the baseline, symbol by symbol (which archive members disappeared).
3. **Cut what is left, if worth it.** Look at what the closure still holds that BLE-MIDI does not need (for example
   classic `bredr_*` code reachable through tables). Options: another configuration value; or replace a function
   in the bitcode with a stub (`llvm-link` / a rewritten member), marked as modified. Only after 2, and only with a
   reason from step 2's list.
4. **Give the stubs a body.** The 146 functions: interrupt lock, `jiffies` from TIMER4, the task table, settings storage
   (a small flash page), libc routines, and empty stubs for the parts the closure carries but BLE does not use.
5. **On a device.** HCI Reset and the Command Complete; advertise; connect from a phone; one note. Measure RAM in use,
   the radio's current, and what the audio ISR's jitter does while the radio is busy.

## Measures of success

| Measure | Target | Why |
| --- | --- | --- |
| Flash (controller + OS layer + host) | under 60 KB | fits Felucca's free flash (about 122 KB in 1.4.1) with room |
| RAM (`.bss` + pool) | under 40 KB | Felucca 1.5 has about 59.7 KB free in total (18.9 KB RAM + 40.8 KB pool) |
| Audio ISR jitter with the radio on | no audible dropout | the controller shares the CPU unless it runs on the second core |

## Risks

- The controller is a task of JieLi's RTOS. If it needs more of the scheduler than a tick and a task table, the OS layer
  grows (10-30 KB was the earlier guess).
- Bitcode from JieLi's clang 4.0.1 only links with that toolchain's LTO; newer LLVM tools read it but cannot link it.
- Higher RAM than hoped even after step 2: the second core (`chip/fm1_cpu1.h`) does not reduce RAM, only isolates timing.
- Radio use on a product needs its own approval (技適 or the local equivalent) and Bluetooth SIG terms for the name;
  both are per device and not part of this repository's licence.
- GPL: a firmware that links the controller cannot also be GPL (no source for the libraries). A BLE build is a separate
  build from the GPL firmware; this repository's MPL files are fine in it.

## Not in this plan

Pairing and bonding, scanning, more than one connection, BLE-MIDI receive and SysEx, classic Bluetooth, Wi-Fi.
