/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Glue between ble/ (the host) and JieLi's controller (btctrler.a, from the JieLi AC79 SDK, fetched by
 * tools/get_sdk.sh). Beta: it compiles against the SDK headers; it has not run on a device.
 *
 *   ble_glue_init(name)   open the controller's HCI transport and hand its packets to the host
 *   ble_glue_poll()       call from the main loop
 *   ble_glue_midi(...)    queue a MIDI message (1..3 bytes) for the central
 *
 * The controller is a task of JieLi's RTOS (btctrler_task_init); starting that RTOS next to the app is
 * the part this file does not do (see README.md). */
#include "ble_midi_host.h"
#include "typedef.h"
#include "hci_transport.h"

extern int btctrler_task_init(const void *transport, const void *config);

static bleh_t host;
static const hci_transport_t *tp;

static void from_controller(int packet_type, const u8 *p, int n)
{
    bleh_rx(&host, (uint8_t)packet_type, p, (uint16_t)n);
}

static int to_controller(uint8_t h4_type, const uint8_t *p, uint16_t n, void *ctx)
{
    (void)ctx;
    if (tp->can_send_packet_now && !tp->can_send_packet_now(h4_type))
        return -1;
    return tp->send_packet(h4_type, p, n) == 0 ? 0 : -1;     /* packet bytes without the H4 type byte */
}

int ble_glue_init(const char *name, const void *ctrl_config)
{
    tp = hci_transport_h4_controller_instance();
    bleh_init(&host, to_controller, 0, name);
    tp->register_packet_handler(from_controller);
    if (btctrler_task_init(tp, ctrl_config))
        return -1;
    bleh_start(&host);
    return 0;
}

void ble_glue_poll(void) { bleh_poll(&host); }

int ble_glue_midi(const uint8_t *msg, uint8_t len, uint16_t ts13) { return bleh_midi_send(&host, msg, len, ts13); }
