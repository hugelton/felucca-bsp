/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* A minimal BLE host that only sends BLE-MIDI: a peripheral that advertises, takes one connection, serves a
 * tiny GATT table (Generic Access + the BLE-MIDI service) and sends MIDI as notifications.
 *
 * It talks to a controller through standard HCI packets (H4 packet types), so it needs no vendor code:
 *   bleh_init()      set the name and the send function
 *   bleh_start()     reset the controller and start advertising (the sequence goes on from bleh_rx)
 *   bleh_rx()        every packet that comes from the controller (event or ACL)
 *   bleh_midi_send() queue one MIDI message (1..3 bytes, no SysEx) with a 13-bit millisecond timestamp
 *   bleh_poll()      call from the main loop: packs queued messages into notifications
 * Fixed: ATT MTU 23, no pairing, no scanning, one connection. No malloc, no vendor code. */
#pragma once
#include <stdint.h>

#define BLEH_MTU 23u
#define BLEH_NAME_MAX 20u
#define BLEH_MIDIQ 32u

enum { BLEH_H4_CMD = 1, BLEH_H4_ACL = 2, BLEH_H4_EVT = 4 };
enum { BLEH_IDLE = 0, BLEH_STARTING, BLEH_ADVERTISING, BLEH_CONNECTED, BLEH_ERROR };

/* send one packet to the controller: h4_type, then the bytes after the H4 type byte; returns 0 when sent */
typedef int (*bleh_send_fn)(uint8_t h4_type, const uint8_t *p, uint16_t n, void *ctx);

typedef struct {
    uint8_t len, b[3];
    uint16_t ts;
} bleh_msg_t;

typedef struct {
    bleh_send_fn send;
    void *ctx;
    char name[BLEH_NAME_MAX + 1];
    uint8_t namelen;
    uint8_t state, step, cmd_wait, want_adv;
    uint16_t cmd_op;
    uint16_t conn;
    uint8_t cccd[2];
    uint8_t credits, acl_total;
    uint8_t rx[BLEH_MTU + 8];       /* one L2CAP PDU being collected */
    uint8_t rx_have, rx_need, rx_on;
    bleh_msg_t q[BLEH_MIDIQ];
    uint8_t q_r, q_w;
} bleh_t;

void bleh_init(bleh_t *h, bleh_send_fn send, void *ctx, const char *name);
void bleh_start(bleh_t *h);
void bleh_rx(bleh_t *h, uint8_t h4_type, const uint8_t *p, uint16_t n);
int bleh_midi_send(bleh_t *h, const uint8_t *msg, uint8_t len, uint16_t ts13);
void bleh_poll(bleh_t *h);
static inline int bleh_connected(const bleh_t *h) { return h->state == BLEH_CONNECTED; }
static inline int bleh_notifying(const bleh_t *h) { return h->state == BLEH_CONNECTED && (h->cccd[0] & 1u); }
