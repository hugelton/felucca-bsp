/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Charles Vestal */
/* The received SysEx frames' queue (chip/usb.c: sx_start .. sx_queue, sx_fits in ep1_take, ota_frame_get /
 * ota_frame_done), the production code on the host. The ISR's side is ep1_take given the host's EP1 OUT packets
 * (16 event packets, 48 SysEx bytes at most, a packet it leaves sent again later, as the host does after a NAK);
 * the main loop takes the oldest frame at random times and holds it for a random while, and the ISR also runs
 * inside ota_frame_get and ota_frame_done: at their barriers (RING_PUBLISH) and, built from the copy of usb.c
 * run_sysex_queue_test.sh makes (TEST_USB_C, TEST_PREEMPT() after each of their statements), between any two of
 * their statements. Checked, for the buffer
 * FELUCCA_SX_IN this is built with (run_sysex_queue_test.sh: 300 to 2000 bytes, and the update loader's build, 4744):
 *   - every frame up to FELUCCA_SX_IN - 54 bytes arrives whole and in order (TEST_LONG 0: all of them); a longer one
 *     may be dropped, never anything else, one longer than FELUCCA_SX_IN - 2 always is, and so may the frames that
 *     begin in the USB packet a longer one ends in (TEST_LONG 3: 3% of the frames longer; the buffer has no room
 *     for them then, and the packet can't be left: it ends the long frame);
 *   - a packet is left (NAK) only while a frame waits (or the MIDI ring is full), so nothing stalls, and the ISR
 *     never writes over the frame the main loop holds;
 *   - a frame arriving when none waits starts the buffer over: FELUCCA_SX_IN - 2 bytes (an M-UPGRADE frame, 640,
 *     with the default 648) always fit then, wherever the buffer was left;
 *   - sx_epoch (8 bits) moves at most once between the reader's looks (sx_ep), so it can't wrap unseen. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../chip/fm1_usb.h"

/* ------------------------------------------- the SIE: nothing on it --- */
static void mock_write(uint32_t r, uint32_t v) { (void)r; (void)v; }
static void mock_rd(uint32_t r) { (void)r; }
static uint32_t mock_dt(void) { return 0; }
static void mock_ep(uint32_t ep) { (void)ep; }
static void mock_buf(uint32_t ep, void *p) { (void)ep; (void)p; }
static void mock_send(uint32_t ep, void *p, uint32_t n) { (void)ep; (void)p; (void)n; }
static void mock_ep0_send(void *p, uint32_t n) { (void)p; (void)n; }
static void mock_ep0_buf(void *p) { (void)p; }
static void mock_nop(void) {}
static void mock_attach(void *p) { (void)p; }
static uint32_t mock_one(void) { return 1; }
static uint32_t mock_zero(void) { return 0; }
static void fm1_delay_ms(uint32_t ms) { (void)ms; }
static volatile uint32_t fm1_ms;

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

/* the ISR may run anywhere in the main loop: here at its barriers, and between the statements of the copy */
static int in_isr;
static void preempt(void);
#define RING_PUBLISH() (in_isr ? (void)0 : preempt())
#define TEST_PREEMPT() (in_isr ? (void)0 : preempt())
#ifndef TEST_USB_C
#define TEST_USB_C "../chip/usb.c"
#endif
#include TEST_USB_C
static uint32_t ota_now_ms(void) { return fm1_ms; }  /* (ota_wire_send's, not used here) */
static void ota_idle(void) {}

#ifndef TEST_FRAMES
#define TEST_FRAMES 130000u
#endif
#ifndef TEST_LONG
#define TEST_LONG 3u                                    /* frames longer than LIMIT, percent */
#endif
#define LIMIT (FELUCCA_SX_IN - 54u)                     /* frames up to this always arrive */

static uint32_t rng = 1u;
static uint32_t rnd(uint32_t n)                         /* 0 .. n-1 */
{
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng % n;
}
static uint8_t frame_byte(uint32_t k, uint32_t i)       /* frame k's i-th byte (never the soft key's 22 24 35) */
{
    return i == 0 ? (uint8_t)(0x40u | (k & 0x3Fu)) : (uint8_t)((k * 31u + i * 7u + (i >> 7)) & 0x7Fu);
}

/* ------------------------------------------------------------ the host --- */
static uint32_t *flen;                                  /* the frames sent, by number */
static uint32_t sent;                                   /* the frames started on the wire */
static uint8_t *after_long;                             /* the frame started in the packet a longer one ended in */
static uint32_t ev[FELUCCA_SX_IN / 2u + 200u], ev_n, ev_at;   /* the frame's event packets (3 bytes and some
                                                         * clocks each); the next to send */
static uint8_t pkt[64];
static uint32_t pkt_n;                                  /* the packet the host sends (again after a NAK) */
static uint8_t clocks;                                  /* F8s between the events (not in the update loader) */

static void frame_events(uint32_t k)                    /* frame k as USB-MIDI event packets (CIN 4, 5..7) */
{
    uint32_t n = flen[k] + 2u, i, b[3], m = 0;
    ev_n = ev_at = 0;
    for (i = 0; i < n; i++) {
        b[m++] = i == 0 ? 0xF0u : i == n - 1u ? 0xF7u : frame_byte(k, i - 1u);
        if (m == 3u || i == n - 1u) {
            uint32_t cin = i == n - 1u ? 4u + m : 4u;
            ev[ev_n++] = cin | b[0] << 8 | (m > 1u ? b[1] : 0u) << 16 | (m > 2u ? b[2] : 0u) << 24;
            m = 0;
        }
        if (clocks && rnd(64) == 0)
            ev[ev_n++] = 0x0Fu | 0xF8u << 8;            /* a clock inside the frame */
    }
    assert(ev_n <= sizeof ev / sizeof ev[0]);
}
/* the next USB packet: up to 16 events from the stream (mostly 16), the frames one after another */
static int next_packet(uint32_t upto)
{
    uint32_t want = rnd(4) ? 16u : 1u + rnd(16), e = 0;
    uint8_t long_ended = 0;
    while (e < want) {
        if (ev_at == ev_n) {
            if (sent >= upto)
                break;
            if (e && sent && flen[sent - 1u] > LIMIT)
                long_ended = 1;
            after_long[sent] = long_ended;
            frame_events(sent++);
        }
        memcpy(pkt + 4u * e++, &ev[ev_at++], 4);
    }
    pkt_n = 4u * e;
    return e != 0;
}

/* ------------------------------------------------------------- the ISR --- */
static uint64_t epoch_full, epoch_seen;                 /* sx_epoch and sx_ep without the wrap */
static uint8_t epoch_last;
static uint32_t isr_runs, naks, naks_none_waiting;
static int isr_tick(uint32_t upto)                      /* one poll: the packet waiting, or a new one; 1: one went in */
{
    int took, ring_full;
    uint8_t d;
    if (!pkt_n && !next_packet(upto))
        return 0;
#ifdef FELUCCA_LOADER
    ring_full = 0;
#else
    ring_full = MQ - (mi_w - mi_r) < EP1_ROOM;          /* (the MIDI ring's back-pressure: the clocks) */
#endif
    in_isr = 1;
    took = ep1_take(pkt, pkt_n);
    in_isr = 0;
    isr_runs++;
    d = (uint8_t)(sx_epoch - epoch_last);
    assert(d <= 1u);                                    /* one frame a buffer started over, at most, a packet */
    epoch_full += d;
    epoch_last = sx_epoch;
    assert(epoch_full - epoch_seen <= 1u);              /* never more than one ahead of the reader's look */
    if (took) {
        pkt_n = 0;
    } else {
        naks++;
        if (sx_made == sx_taken && !ring_full)
            naks_none_waiting++;
    }
    return took;
}

static uint32_t preempt_runs, preempt_upto;
static void preempt(void)
{
    uint32_t n = rnd(8) ? rnd(3) : rnd(24);   /* (sometimes long: the ISR takes several packets) */
    while (n--) {
        isr_tick(preempt_upto);
        preempt_runs++;
    }
}

/* ------------------------------------------------------- the main loop --- */
static uint32_t expect;                                 /* the next frame to arrive, by number */
static uint32_t got, dropped, dropped_after;
static uint8_t held[FELUCCA_SX_IN];

static int take(void)                                   /* one frame from the queue, checked; 0: none */
{
    const uint8_t *p;
    uint32_t n, i;
    if (!ota_frame_get(&p, &n))
        return 0;
    epoch_seen = epoch_full - (uint8_t)(sx_epoch - sx_ep);
    assert(epoch_seen == epoch_full);
    assert(n >= 1u && n <= FELUCCA_SX_IN - 2u);
    assert(p >= sx_frame + 2 && p + n <= sx_frame + FELUCCA_SX_IN);
    while (expect < sent && (flen[expect] != n || p[0] != frame_byte(expect, 0))) {        /* the frames before it: only long ones are dropped, and */
        if (flen[expect] <= LIMIT && !after_long[expect]) { /* those in the packet a long one ended in */
            fprintf(stderr, "FAIL: frame %u (%u bytes) lost, got %u bytes\n", expect, flen[expect], n);
            exit(1);
        }
        dropped++;
        dropped_after += flen[expect] <= LIMIT;
        expect++;
    }
    assert(expect < sent);
    for (i = 0; i < n; i++)
        if (p[i] != frame_byte(expect, i)) {
            fprintf(stderr, "FAIL: frame %u byte %u: %02X, not %02X\n", expect, i, p[i], frame_byte(expect, i));
            exit(1);
        }
    memcpy(held, p, n);
    {                                                   /* held for a while: the ISR goes on */
        uint32_t h = rnd(8) ? rnd(6) : rnd(200);
        while (h--)
            isr_tick(preempt_upto);
    }
    assert(memcmp(held, p, n) == 0);                    /* the ISR left the held frame alone */
    ota_frame_done();
    expect++;
    got++;
    return 1;
}

static void drain_midi(void) { mi_r = mi_w; }

static void run_random(uint32_t upto)
{
    uint32_t idle = 0;
    preempt_upto = upto;
    while (sent < upto || pkt_n || ev_at < ev_n || sx_made != sx_taken) {
        uint32_t isr = rnd(16) < (rnd(8) ? 12u : 15u) ? 1u + rnd(4) : 0u;
        while (isr--)
            isr_tick(upto);
        if (rnd(4) == 0) {
            drain_midi();
            take();
        }
        idle = sx_made == sx_taken && !pkt_n && ev_at == ev_n && sent >= upto ? idle + 1 : 0;
        assert(idle < 4u);
    }
}

/* a frame of FELUCCA_SX_IN - 2 bytes with none waiting, wherever the buffer was: the main loop doesn't look until
 * it is in */
static void big_alone(void)
{
    uint32_t k = sent;
    assert(sx_made == sx_taken && !pkt_n && ev_at == ev_n);
    flen[k] = FELUCCA_SX_IN - 2u;
    preempt_upto = k + 1u;
    while (sx_made == sx_taken && (sent <= k || pkt_n || ev_at < ev_n)) {
        uint32_t before = naks;
        drain_midi();                                   /* (the MIDI ring: the clocks) */
        isr_tick(k + 1u);
        assert(naks == before);                         /* never left: none waits */
    }
    assert(sx_made == sx_taken + 1u);
    assert(take());
    assert(expect == k + 1u);
}

int main(void)
{
    uint32_t k, bigs = 0, longs = 0, afters = 0;
#ifdef FELUCCA_LOADER
    clocks = 0;
#else
    clocks = 1;
#endif
    rng = 0x9E3779B9u ^ FELUCCA_SX_IN;
    flen = calloc(TEST_FRAMES + TEST_FRAMES / 64u + 16u, sizeof *flen);
    after_long = calloc(TEST_FRAMES + TEST_FRAMES / 64u + 16u, 1);
    assert(flen && after_long);
    for (k = 0; k < TEST_FRAMES + TEST_FRAMES / 64u + 16u; k++) {
        uint32_t r = rnd(100);
        flen[k] = r < 50 ? 1u + rnd(24) : r < 80 ? 1u + rnd(160) : r < 100u - TEST_LONG ? 1u + rnd(LIMIT) :
                  LIMIT + 1u + rnd(100);                /* longer than the limit, some longer than the buffer */
    }
    /* in rounds: random traffic, then a frame the buffer's size less 2 from wherever it was left */
    while (sent < TEST_FRAMES) {
        uint32_t upto = sent + 1u + rnd(2000);
        if (upto > TEST_FRAMES)
            upto = TEST_FRAMES;
        run_random(upto);
        while (expect < sent) {                         /* the long ones at the end, dropped */
            assert(flen[expect] > LIMIT || after_long[expect]);
            dropped_after += flen[expect] <= LIMIT;
            dropped++;
            expect++;
        }
        big_alone();
        bigs++;
    }
    for (k = 0; k < sent; k++) {
        longs += flen[k] > LIMIT;
        afters += flen[k] <= LIMIT && after_long[k];
    }
    assert(got + dropped == sent);
    assert(TEST_LONG || dropped == 0);                  /* no long frames: none lost */
    assert(naks_none_waiting == 0);
    assert(epoch_full > 1000u);                         /* (it wrapped, many times) */
    printf("FELUCCA_SX_IN %u: %u frames in %u packets (%u left, NAK, and sent again); %u up to %u bytes, all in "
           "but %u of the %u that began in the packet a longer one ended in; %u longer, %u of them in; %u of %u "
           "bytes with none waiting, all in; %u ISR runs at the main loop's barriers; epoch %llu\n",
           (unsigned)FELUCCA_SX_IN, sent, isr_runs - naks, naks, sent - longs, (unsigned)LIMIT, dropped_after, afters,
           longs, longs - (dropped - dropped_after), bigs, (unsigned)FELUCCA_SX_IN - 2u, preempt_runs,
           (unsigned long long)epoch_full);
    return 0;
}
