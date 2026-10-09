# hwtest-ble (beta, not complete)

The BLE counterpart of `../hwtest`: the glue between `ble/` (the BLE-MIDI host) and JieLi's radio controller
(`btctrler.a`, from the JieLi AC79 SDK).

## What is here

- `ble_glue.c`: opens the controller's HCI transport (`hci_transport_h4_controller_instance()`), gives its
  packets to `bleh_rx`, sends the host's packets back, and starts the controller (`btctrler_task_init`).
- `check.sh`: compiles the glue against the SDK headers. This passes: the types and calls match the SDK.

## What is not here

The controller is not a plain library. It is a task of JieLi's RTOS (`btctrler_task_init` posts to the SDK's task
and message queues) and needs the SDK's system layer: scheduler, timers, memory, interrupts. Making that run beside
a bare-metal app is the open work. Measured earlier: the controller itself is about 45-55 KB, the system layer a
further 10-30 KB (not yet narrowed down). Until a build links and a device is on the bench, this directory is an
interface check, not a firmware.

## Link survey (2026-10-09, SDK d179b44)

`closure.py` follows the undefined symbols from `btctrler_task_init` and `hci_transport_h4_controller_instance`
through the SDK archives (`btctrler.a`, `cpu.a`, `system.a`, `wl_rf_common.a`, `common_lib.a`, `event.a`; not
`btstack.a`, which our host replaces):

- Pulled: 153 archive members (btctrler 86, cpu 26, system 22, wl_rf_common 13, event 3, common_lib 3).
- Still undefined: 343 symbols that the application must provide:

| Kind | Count | What it means |
| --- | --- | --- |
| `config_*` / `CONFIG_*` | 48 | Controller options as variables: roles, number of links, packet sizes, features. Set by us. |
| `log_tag_const_*` | 108 | Constants for the SDK's log macros. Empty stubs are enough. |
| Section markers (`*_begin`, `*_end`, `*_vma`, heap, pools) | 41 | From the SDK's linker script (`sdk_ld.c`): heap, ACL pools, handler tables. We define them in our own `app.ld`. |
| Functions and data | 146 | See below. |

The 146 are the real work. Grouped: interrupts (`local_irq_*`, `cpu_in_irq`, `irq_info_table`), time (`jiffies`),
tasks and queues (`task_info_table`, `hci_packet_handler`), settings storage (`syscfg_read/write`), libc
(`memcmp`, `strcmp`, `strlen`, ...), crypto (`uECC_*`, `aes_cmac`, `bi_*`, for pairing), radio trim and Wi-Fi
coexistence (`wifi_*`, `wl_*`, `rtc_init_data`), and parts that come with the closure but BLE does not use
(TWS, eSCO, SDIO, USB, `sniff_*`). Many of the last group are stubs.

This is a bounded list, not an RTOS port: no scheduler symbol such as `os_start` is among them. Whether the
controller runs on a plain tick and a small task table, or needs more, only a device will tell.

## Next steps

1. Write `stubs.c` and a linker script for the 343 symbols (constants and empty stubs first), link, and see what
   the linker still refuses.
2. Give the real ones a body: IRQ lock, `jiffies` from TIMER4, a task table, `syscfg_*` on a small flash page.
3. On a device: send `HCI Reset`, expect `Command Complete`; then advertise and connect from a phone.
