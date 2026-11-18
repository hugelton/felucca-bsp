/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* MIDI IN to the host when nobody reads it (chip/usb.c ep1_tx, FELUCCA_MIDI_TX_STALL_POLLS), the production code on the
 * host with the SIE mocked: the IN FIFO holds a packet until the host takes it. Checked:
 *   - while the host reads, every packet the app queues goes out in order, none dropped;
 *   - when the host stops reading (macOS: no app has the port open), the FIFO is flushed after
 *     FELUCCA_MIDI_TX_STALL_POLLS polls and from then on the rings are dropped at every poll; one packet is tried each
 *     stall, so the ring never holds more than a poll's worth of stale notes and clocks;
 *   - when the host reads again, the next packet goes out at once with only what was queued since (no stale burst);
 *   - a bus reset in between leaves nothing behind either. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../chip/fm1_usb.h"

/* ------------------------------------------------------- the SIE: an IN FIFO the host may or may not read --- */
static uint32_t rd_reg;
static int fifo_full, host_reads, flushes, sent_pkts;
static uint8_t last_pkt[64];
static uint32_t last_n;
static void mock_write(uint32_t r, uint32_t v)
{
    if (r == 17u && (v & 0x08u)) { fifo_full = 0; flushes++; }   /* S_TXCSR1: FlushFIFO */
}
static void mock_rd(uint32_t r) { rd_reg = r; }
static uint32_t mock_dt(void) { return rd_reg == 17u && fifo_full ? 0x01u : 0u; }   /* TXCSR1 bit 0: a packet waits */
static void mock_ep(uint32_t ep) { (void)ep; }
static void mock_buf(uint32_t ep, void *p) { (void)ep; (void)p; }
static void mock_send(uint32_t ep, void *p, uint32_t n)
{
    if (ep != 1u) return;
    assert(!fifo_full);
    memcpy(last_pkt, p, n); last_n = n; sent_pkts++;
    fifo_full = 1;
}
static void mock_ep0_send(void *p, uint32_t n) { (void)p; (void)n; }
static void mock_ep0_buf(void *p) { (void)p; }
static void mock_nop(void) {}
static void mock_attach(void *p) { (void)p; }
static uint32_t mock_one(void) { return 1; }
static uint32_t mock_zero(void) { return 0; }
static void fm1_delay_ms(uint32_t ms) { (void)ms; }

#define fm1_usb_ep_enable mock_ep
#define fm1_usb_ep_txbuf mock_buf
#define fm1_usb_ep_rxbuf mock_buf
#define fm1_usb_ep_send mock_send
#define fm1_usb_ep0_send mock_ep0_send
#define fm1_usb_ep0_buf mock_ep0_buf
#define fm1_usb_ep4_rxbuf(p) mock_buf(4, p)
#define fm1_usb_ep4_send(p, n) mock_send(4, p, n)
#define fm1_usb_rx_sync mock_nop
#define fm1_usb_sie_on mock_one
#define fm1_usb_sie_done mock_one
#define fm1_usb_sie_wr_start mock_write
#define fm1_usb_sie_rd_start mock_rd
#define fm1_usb_sie_data mock_dt
#define fm1_usb_reset mock_nop
#define fm1_usb_off mock_nop
#define fm1_usb_attach mock_attach
#define fm1_usb_sof_take mock_zero
#define fm1_wdt_feed() ((void)0)
#define FELUCCA_OTA 1
#define FELUCCA_UAC 0
#define FELUCCA_CDC 0
#define FELUCCA_USB_HOST 0
#define HALF_FRAMES 128u
static volatile uint32_t render_peak;
static volatile uint8_t render_busy;
#define RING_PUBLISH() ((void)0)
#include "../chip/usb.c"
static uint32_t ota_now_ms(void) { return 0; }
static void ota_idle(void) {}

static uint32_t note(uint32_t k) { return 0x09u | 0x90u << 8 | (k & 0x7Fu) << 16 | 100u << 24; }   /* CIN 9: a note on */
static void poll(void) { if (host_reads) fifo_full = 0; ep1_tx(); }   /* (the host takes what waits, then the poll) */

int main(void)
{
    uint32_t k, got = 0, i;
    usb.config = 1;
    /* 1. the host reads: everything goes out, in order */
    host_reads = 1;
    for (k = 0; k < 200u; k++) {
        assert(midi_out_try(note(k)));
        poll();
        for (i = 0; i + 4u <= last_n && sent_pkts; i += 4u) {
            uint32_t p = last_pkt[i] | last_pkt[i + 1] << 8 | (uint32_t)last_pkt[i + 2] << 16 | (uint32_t)last_pkt[i + 3] << 24;
            assert(p == note(got)); got++;
        }
        last_n = 0;
    }
    assert(got == 200u && flushes == 0);

    /* 2. the host stops reading: the FIFO stays full; after the stall polls it is flushed once, the ring dropped */
    host_reads = 0;
    assert(midi_out_try(note(1000)));
    poll();                                             /* (this one goes into the FIFO) */
    assert(fifo_full);
    for (k = 0; k < FELUCCA_MIDI_TX_STALL_POLLS - 1u; k++) { (void)midi_out_try(note(2000 + k)); poll(); }
    assert(flushes == 0 && fifo_full);                 /* (not before the stall) */
    for (k = 0; k < 3u; k++) { (void)midi_out_try(note(2500 + k)); poll(); }
    assert(flushes == 1);
    {   const int before = sent_pkts, f0 = flushes;
        const uint32_t polls = 10u * (FELUCCA_MIDI_TX_STALL_POLLS + 2u);
        for (k = 0; k < polls; k++) { assert(midi_out_try(note(3000 + k))); poll(); assert(mo_w - mo_r <= 1u); }   /* (the ring never fills) */
        assert(sent_pkts - before <= (int)(polls / FELUCCA_MIDI_TX_STALL_POLLS) + 1);   /* (one try a stall) */
        assert(flushes - f0 >= (int)(polls / (FELUCCA_MIDI_TX_STALL_POLLS + 2u)) - 1);
    }

    /* 3. the host reads again: the next packet holds only what was queued since */
    host_reads = 1;
    fifo_full = 0;
    poll();
    last_n = 0;
    assert(midi_out_try(note(4000)));
    poll();
    assert(last_n == 4u && (last_pkt[2] & 0x7Fu) == (4000u & 0x7Fu));

    /* 4. a stall, then a bus reset: nothing left behind */
    host_reads = 0;
    for (k = 0; k < FELUCCA_MIDI_TX_STALL_POLLS + 2u; k++) { (void)midi_out_try(note(5000 + k)); poll(); }
    mo_r = mo_w;                                       /* (as usb_poll's bus reset does) */
    host_reads = 1; fifo_full = 0; poll();
    assert(mo_w == mo_r);

    printf("midi tx drop: OK (stall %u polls)\n", (unsigned)FELUCCA_MIDI_TX_STALL_POLLS);
    return 0;
}
