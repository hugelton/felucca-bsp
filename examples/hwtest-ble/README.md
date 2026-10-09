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

## Next steps

1. Link `btctrler.a` and the needed part of `system.a` with a small app and list what is still undefined.
2. Provide those pieces (a tick, a queue, memory, the radio interrupts), or start the SDK's own RTOS in the app.
3. On a device: send `HCI Reset`, expect `Command Complete`; then advertise and connect from a phone.
