/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Host test of the BLE-MIDI host against a fake controller and a scripted central (phone / Mac).
 * Expected bytes are written out by hand from the Bluetooth Core and BLE-MIDI specifications.
 *   cc -std=c99 -Wall -Wextra -g -fsanitize=address,undefined -I. ble_midi_host.c tests/ble_midi_host_test.c -o /tmp/t && /tmp/t */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ble_midi_host.h"

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("  FAIL line %d: %s\n", __LINE__, #c); } } while (0)
static void check_bytes(int line, const char *what, const uint8_t *got, unsigned gn, const uint8_t *exp, unsigned en)
{
    unsigned i;
    checks++;
    if (gn == en && !memcmp(got, exp, en)) return;
    fails++;
    printf("  FAIL line %d: %s\n    got  (%u):", line, what, gn);
    for (i = 0; i < gn; i++) printf(" %02X", got[i]);
    printf("\n    want (%u):", en);
    for (i = 0; i < en; i++) printf(" %02X", exp[i]);
    printf("\n");
}
#define BYTES(what, got, gn, ...) do { static const uint8_t e_[] = {__VA_ARGS__}; check_bytes(__LINE__, what, got, gn, e_, sizeof e_); } while (0)

/* ---- the fake controller side: everything the host sends is logged ---- */
typedef struct { uint8_t type; uint16_t n; uint8_t b[64]; } pkt_t;
static pkt_t lg[256];
static unsigned nlg;
static int send_cb(uint8_t t, const uint8_t *p, uint16_t n, void *ctx)
{
    (void)ctx;
    if (nlg < 256 && n <= 64) { lg[nlg].type = t; lg[nlg].n = n; memcpy(lg[nlg].b, p, n); nlg++; }
    return 0;
}
static const pkt_t *last(void) { return &lg[nlg - 1]; }
static void evt(bleh_t *h, unsigned code, const uint8_t *p, unsigned n)
{
    uint8_t b[64];
    b[0] = (uint8_t)code; b[1] = (uint8_t)n;
    memcpy(b + 2, p, n);
    bleh_rx(h, BLEH_H4_EVT, b, (uint16_t)(2 + n));
}
static void cc(bleh_t *h, unsigned op, unsigned status, const uint8_t *ret, unsigned rn)
{
    uint8_t p[16] = {1, (uint8_t)op, (uint8_t)(op >> 8), (uint8_t)status};
    if (rn) memcpy(p + 4, ret, rn);
    evt(h, 0x0E, p, 4 + rn);
}
/* a PDU from the central: ACL (handle 0x0040, start), L2CAP, ATT */
static void central(bleh_t *h, const uint8_t *att, unsigned n)
{
    uint8_t b[64];
    b[0] = 0x40; b[1] = 0x20;                       /* handle 0x040, PB = 2 */
    b[2] = (uint8_t)(4 + n); b[3] = 0;
    b[4] = (uint8_t)n; b[5] = 0; b[6] = 4; b[7] = 0;
    memcpy(b + 8, att, n);
    bleh_rx(h, BLEH_H4_ACL, b, (uint16_t)(8 + n));
}
#define ATT(h, ...) do { static const uint8_t a_[] = {__VA_ARGS__}; central(h, a_, sizeof a_); } while (0)
/* the ATT part of the last ACL packet the host sent (after handle, length, L2CAP length, CID) */
static const uint8_t *att_out(unsigned *n)
{
    const pkt_t *p = last();
    CHECK(p->type == BLEH_H4_ACL && p->n >= 9);
    CHECK(p->b[0] == 0x40 && p->b[1] == 0x00);                   /* handle 0x040, PB 0 */
    CHECK((unsigned)p->b[2] == p->n - 4u && (unsigned)p->b[4] == p->n - 8u);
    CHECK(p->b[6] == 4 && p->b[7] == 0);                         /* ATT channel */
    *n = p->n - 8u;
    return p->b + 8;
}
#define EXPECT_ATT(h, ...) do { unsigned n_; const uint8_t *o_ = att_out(&n_); (void)h; BYTES("att reply", o_, n_, __VA_ARGS__); } while (0)

static const uint8_t SVC[16] = {0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7, 0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03};
#define SVC_BYTES 0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7, 0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
#define CHR_BYTES 0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1, 0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77

static bleh_t H;
static void ack(unsigned n)                         /* the controller reports n ACL packets as sent */
{
    uint8_t p[] = {1, 0x40, 0x00, (uint8_t)n, 0x00};
    evt(&H, 0x13, p, sizeof p);
}
static void bring_up(void)
{
    static const uint8_t buf[] = {27, 0, 3};        /* LE ACL length 27, 3 buffers */
    nlg = 0;
    bleh_init(&H, send_cb, 0, "Felucca");
    bleh_start(&H);
    cc(&H, 0x0C03, 0, 0, 0);
    cc(&H, 0x2002, 0, buf, 3);
    cc(&H, 0x2006, 0, 0, 0);
    cc(&H, 0x2008, 0, 0, 0);
    cc(&H, 0x2009, 0, 0, 0);
    cc(&H, 0x200A, 0, 0, 0);
}
static void connect(void)
{
    static const uint8_t c[] = {0x01, 0x00, 0x40, 0x00, 0x01, 0, 1, 2, 3, 4, 5, 6, 0x18, 0, 0, 0, 0x48, 0, 0};
    evt(&H, 0x3E, c, sizeof c);                     /* LE connection complete: handle 0x40, role 1 (peripheral) */
}

static void test_init(void)
{
    static const uint8_t buf[] = {27, 0, 3};
    printf("init sequence\n");
    nlg = 0;
    bleh_init(&H, send_cb, 0, "Felucca");
    bleh_start(&H);
    CHECK(nlg == 1 && lg[0].type == BLEH_H4_CMD);
    BYTES("reset", lg[0].b, lg[0].n, 0x03, 0x0C, 0x00);
    bleh_poll(&H);
    CHECK(nlg == 1);                                /* one command at a time */
    cc(&H, 0x0C03, 0, 0, 0);
    BYTES("LE read buffer size", last()->b, last()->n, 0x02, 0x20, 0x00);
    cc(&H, 0x2002, 0, buf, 3);
    BYTES("adv params", last()->b, last()->n, 0x06, 0x20, 15, 0x40, 0x00, 0x80, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0);
    cc(&H, 0x2006, 0, 0, 0);
    BYTES("adv data", last()->b, last()->n, 0x08, 0x20, 32, 21, 2, 1, 6, 17, 7, SVC_BYTES,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    cc(&H, 0x2008, 0, 0, 0);
    {   static const uint8_t e[] = {0x09, 0x20, 32, 9, 8, 9, 'F', 'e', 'l', 'u', 'c', 'c', 'a'};
        unsigned i, z = 1;
        check_bytes(__LINE__, "scan response", last()->b, sizeof e, e, sizeof e);
        CHECK(last()->n == 35);
        for (i = sizeof e; i < last()->n; i++) z &= last()->b[i] == 0;
        CHECK(z); }
    CHECK(H.state == BLEH_STARTING);
    cc(&H, 0x2009, 0, 0, 0);
    BYTES("adv enable", last()->b, last()->n, 0x0A, 0x20, 0x01, 0x01);
    cc(&H, 0x200A, 0, 0, 0);
    CHECK(H.state == BLEH_ADVERTISING && H.credits == 3);
    CHECK(nlg == 6);
    bring_up();
    cc(&H, 0x1234, 0, 0, 0);                        /* a complete for a command we did not send is ignored */
    CHECK(H.state == BLEH_ADVERTISING);
    nlg = 0;
    bleh_init(&H, send_cb, 0, "x");
    bleh_start(&H);
    cc(&H, 0x0C03, 0x0C, 0, 0);
    CHECK(H.state == BLEH_ERROR);
}

static void test_connect_and_discovery(void)
{
    unsigned n;
    const uint8_t *o;
    printf("connection and discovery\n");
    bring_up();
    {   static const uint8_t m[] = {0x01, 0x00, 0x41, 0x00, 0x00, 0, 1, 2, 3, 4, 5, 6, 0x18, 0, 0, 0, 0x48, 0, 0};
        evt(&H, 0x3E, m, sizeof m);                 /* we are not a central: role 0 is ignored */
        CHECK(H.state == BLEH_ADVERTISING); }
    connect();
    CHECK(bleh_connected(&H) && H.conn == 0x40);
    ATT(&H, 0x02, 0xB9, 0x00);                      /* exchange MTU 185 */
    EXPECT_ATT(&H, 0x03, 23, 0);
    ATT(&H, 0x10, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x28);  /* primary services */
    EXPECT_ATT(&H, 0x11, 6, 0x01, 0x00, 0x03, 0x00, 0x00, 0x18);
    ATT(&H, 0x10, 0x04, 0x00, 0xFF, 0xFF, 0x00, 0x28);
    EXPECT_ATT(&H, 0x11, 20, 0x04, 0x00, 0x07, 0x00, SVC_BYTES);
    ATT(&H, 0x10, 0x08, 0x00, 0xFF, 0xFF, 0x00, 0x28);
    EXPECT_ATT(&H, 0x01, 0x10, 0x08, 0x00, 0x0A);   /* attribute not found */
    ATT(&H, 0x06, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x28, SVC_BYTES);   /* find the BLE-MIDI service by UUID */
    EXPECT_ATT(&H, 0x07, 0x04, 0x00, 0x07, 0x00);
    ATT(&H, 0x06, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x28, 0x00, 0x18);  /* ... and the generic access service */
    EXPECT_ATT(&H, 0x07, 0x01, 0x00, 0x03, 0x00);
    ATT(&H, 0x06, 0x01, 0x00, 0xFF, 0xFF, 0x00, 0x28, 0x0F, 0x18);  /* a service we do not have */
    EXPECT_ATT(&H, 0x01, 0x06, 0x01, 0x00, 0x0A);
    ATT(&H, 0x08, 0x04, 0x00, 0x07, 0x00, 0x03, 0x28);  /* characteristics of the MIDI service */
    EXPECT_ATT(&H, 0x09, 21, 0x05, 0x00, 0x16, 0x06, 0x00, CHR_BYTES);
    ATT(&H, 0x08, 0x01, 0x00, 0x03, 0x00, 0x03, 0x28);  /* characteristics of generic access */
    EXPECT_ATT(&H, 0x09, 7, 0x02, 0x00, 0x02, 0x03, 0x00, 0x00, 0x2A);
    ATT(&H, 0x04, 0x07, 0x00, 0x07, 0x00);          /* find information: the CCCD */
    EXPECT_ATT(&H, 0x05, 0x01, 0x07, 0x00, 0x02, 0x29);
    ATT(&H, 0x04, 0x06, 0x00, 0x06, 0x00);          /* ... the MIDI characteristic (128-bit) */
    EXPECT_ATT(&H, 0x05, 0x02, 0x06, 0x00, CHR_BYTES);
    ATT(&H, 0x04, 0x08, 0x00, 0xFF, 0xFF);
    EXPECT_ATT(&H, 0x01, 0x04, 0x08, 0x00, 0x0A);
    ATT(&H, 0x0A, 0x03, 0x00);                      /* read the device name */
    EXPECT_ATT(&H, 0x0B, 'F', 'e', 'l', 'u', 'c', 'c', 'a');
    ATT(&H, 0x0A, 0x06, 0x00);                      /* BLE-MIDI: a read of the characteristic answers empty */
    EXPECT_ATT(&H, 0x0B);
    ATT(&H, 0x0A, 0x09, 0x00);
    EXPECT_ATT(&H, 0x01, 0x0A, 0x09, 0x00, 0x01);   /* invalid handle */
    ATT(&H, 0x12, 0x07, 0x00, 0x01);                /* CCCD with one byte */
    EXPECT_ATT(&H, 0x01, 0x12, 0x07, 0x00, 0x04);   /* invalid PDU */
    ATT(&H, 0x12, 0x06, 0x00, 0x90, 0x3C, 0x64);    /* a write request to the MIDI value: not permitted */
    EXPECT_ATT(&H, 0x01, 0x12, 0x06, 0x00, 0x03);
    CHECK(!bleh_notifying(&H));
    ATT(&H, 0x12, 0x07, 0x00, 0x01, 0x00);          /* enable notifications */
    EXPECT_ATT(&H, 0x13);
    CHECK(bleh_notifying(&H));
    n = nlg;
    ATT(&H, 0x52, 0x06, 0x00, 0x80, 0x80, 0x90, 0x3C, 0x64);   /* write without response: MIDI from the phone, ignored */
    CHECK(nlg == n);
    ATT(&H, 0x20, 0x00);                            /* an opcode we do not know */
    EXPECT_ATT(&H, 0x01, 0x20, 0x00, 0x00, 0x06);
    ATT(&H, 0x10, 0x01);                            /* truncated */
    EXPECT_ATT(&H, 0x01, 0x10, 0x00, 0x00, 0x04);
    ATT(&H, 0x12, 0x07, 0x00, 0x00, 0x00);          /* disable */
    EXPECT_ATT(&H, 0x13);
    CHECK(!bleh_notifying(&H));
    (void)o;
}

static void sent_att_ok(const uint8_t *exp, unsigned en)
{
    unsigned n;
    const uint8_t *o = att_out(&n);
    check_bytes(__LINE__, "notification", o, n, exp, en);
}

static void test_midi(void)
{
    static const uint8_t on[] = {0x90, 0x3C, 0x64}, off[] = {0x80, 0x3C, 0x00}, sx[] = {0xF0, 0x7E, 0xF7}, rt[] = {0xF8};
    static const uint8_t data[] = {0x3C, 0x64};
    unsigned n0, i;
    printf("MIDI notifications\n");
    bring_up(); connect();
    ATT(&H, 0x12, 0x07, 0x00, 0x01, 0x00);
    ack(1);                                          /* the answer above has gone out */
    CHECK(H.credits == 3);
    CHECK(bleh_midi_send(&H, on, 3, 0x0123) == 0);
    n0 = nlg;
    bleh_poll(&H);
    CHECK(nlg == n0 + 1);
    {   static const uint8_t e[] = {0x1B, 0x06, 0x00, 0x82, 0xA3, 0x90, 0x3C, 0x64};   /* 0x80|(ts>>7), 0x80|ts low, message */
        sent_att_ok(e, sizeof e); }
    BYTES("acl framing", last()->b, 8, 0x40, 0x00, 0x0C, 0x00, 0x08, 0x00, 0x04, 0x00);
    bleh_poll(&H);
    CHECK(nlg == n0 + 1);                            /* nothing more to send */
    ack(1);
    CHECK(bleh_midi_send(&H, off, 3, 0x0125) == 0 && bleh_midi_send(&H, on, 3, 0x0126) == 0);
    bleh_poll(&H);
    {   static const uint8_t e[] = {0x1B, 0x06, 0x00, 0x82, 0xA5, 0x80, 0x3C, 0x00, 0xA6, 0x90, 0x3C, 0x64};   /* two messages, one packet */
        sent_att_ok(e, sizeof e); }
    ack(1);
    CHECK(bleh_midi_send(&H, rt, 1, 0x0200) == 0);   /* a realtime byte */
    CHECK(bleh_midi_send(&H, on, 3, 0x0201) == 0);
    CHECK(bleh_midi_send(&H, on, 3, 0x0300) == 0);   /* the timestamp's high part differs: a packet of its own */
    bleh_poll(&H);
    {   static const uint8_t e[] = {0x1B, 0x06, 0x00, 0x84, 0x80, 0xF8, 0x81, 0x90, 0x3C, 0x64};
        sent_att_ok(e, sizeof e); }
    bleh_poll(&H);
    {   static const uint8_t e[] = {0x1B, 0x06, 0x00, 0x86, 0x80, 0x90, 0x3C, 0x64};
        sent_att_ok(e, sizeof e); }
    CHECK(H.credits == 1);
    CHECK(bleh_midi_send(&H, sx, 3, 0) < 0);         /* no SysEx */
    CHECK(bleh_midi_send(&H, data, 2, 0) < 0);       /* a data byte cannot start a message */
    CHECK(bleh_midi_send(&H, on, 0, 0) < 0 && bleh_midi_send(&H, on, 4, 0) < 0);
    /* flow control: one buffer is kept for ATT answers, so with one left nothing goes out */
    for (i = 0; i < 6; i++) CHECK(bleh_midi_send(&H, on, 3, 0x0400) == 0);
    n0 = nlg;
    bleh_poll(&H);
    CHECK(nlg == n0);
    ack(2);
    bleh_poll(&H);                                   /* 20 bytes of payload: header + 4 messages of (ts + 3 bytes) */
    { unsigned n; att_out(&n); CHECK(n == 3 + 1 + 4 * 4); }
    bleh_poll(&H);
    { unsigned n; att_out(&n); CHECK(n == 3 + 1 + 2 * 4); }
    CHECK(nlg == n0 + 2);
    /* "number of completed packets" lists all handles first and then all counts; ours is the second of two */
    H.credits = 0;
    evt(&H, 0x13, (const uint8_t[]){2, 0x41, 0x00, 0x40, 0x00, 0x09, 0x00, 0x02, 0x00}, 9);
    CHECK(H.credits == 2);
    evt(&H, 0x13, (const uint8_t[]){5, 0x40, 0x00, 0x01, 0x00}, 5);        /* claims 5 handles but is cut short: ignored */
    CHECK(H.credits == 2);
    /* the queue is bounded */
    {   int r = 0;
        for (i = 0; i < 40; i++) r = bleh_midi_send(&H, on, 3, 0);
        CHECK(r == -2); }
    /* with notifications off nothing goes out */
    ATT(&H, 0x12, 0x07, 0x00, 0x00, 0x00);
    ack(3);
    n0 = nlg;
    bleh_poll(&H);
    CHECK(nlg == n0);
}

static void test_disconnect(void)
{
    static const uint8_t on[] = {0x90, 0x3C, 0x64};
    printf("disconnect and advertise again\n");
    bring_up(); connect();
    ATT(&H, 0x12, 0x07, 0x00, 0x01, 0x00);
    bleh_midi_send(&H, on, 3, 1);
    evt(&H, 0x05, (const uint8_t[]){0x00, 0x40, 0x00, 0x13}, 4);
    CHECK(H.state == BLEH_ADVERTISING && !bleh_notifying(&H) && H.q_r == H.q_w);
    BYTES("adv enable again", last()->b, last()->n, 0x0A, 0x20, 0x01, 0x01);
    cc(&H, 0x200A, 0, 0, 0);
    evt(&H, 0x05, (const uint8_t[]){0x00, 0x77, 0x00, 0x13}, 4);   /* some other handle: ignored */
    CHECK(H.state == BLEH_ADVERTISING);
    connect();                                       /* a second connection works the same */
    CHECK(bleh_connected(&H) && H.credits == 3 && !bleh_notifying(&H));
    /* ACL for another handle is not ours */
    {   static const uint8_t a[] = {0x41, 0x20, 0x07, 0x00, 0x03, 0x00, 0x04, 0x00, 0x02, 0xB9, 0x00};
        unsigned n = nlg;
        bleh_rx(&H, BLEH_H4_ACL, a, sizeof a);
        CHECK(nlg == n); }
    /* an L2CAP PDU split in two ACL packets (start + continuation) is put back together */
    {   static const uint8_t st[] = {0x40, 0x20, 0x06, 0x00, 0x07, 0x00, 0x04, 0x00, 0x10, 0x01};   /* PB = 2, ATT bytes 1..2 of 7 */
        static const uint8_t co[] = {0x40, 0x10, 0x05, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x28};          /* PB = 1, the other 5 */
        unsigned n = nlg;
        bleh_rx(&H, BLEH_H4_ACL, st, sizeof st);
        CHECK(nlg == n);                             /* not complete yet */
        bleh_rx(&H, BLEH_H4_ACL, co, sizeof co);
        EXPECT_ATT(&H, 0x11, 6, 0x01, 0x00, 0x03, 0x00, 0x00, 0x18);
    }
}

static unsigned long rs = 12345;
static unsigned rnd(void) { rs = rs * 6364136223846793005ull + 1442695040888963407ull; return (unsigned)(rs >> 33); }
static void test_fuzz(void)
{
    unsigned i, k;
    printf("random input (ASan / UBSan)\n");
    bring_up(); connect();
    for (i = 0; i < 300000; i++) {
        uint8_t b[64];
        unsigned n = rnd() % 40, t = rnd() % 3;
        for (k = 0; k < n; k++) b[k] = (uint8_t)rnd();
        if (t == 0) bleh_rx(&H, BLEH_H4_EVT, b, (uint16_t)n);
        else if (t == 1) bleh_rx(&H, BLEH_H4_ACL, b, (uint16_t)n);
        else if (n >= 1) {                           /* valid framing, random ATT */
            uint8_t a[8 + 40] = {0x40, 0x20};
            a[2] = (uint8_t)(4 + n); a[4] = (uint8_t)n; a[6] = 4;
            if (n > 23) n = 23, a[2] = 27, a[4] = 23;
            memcpy(a + 8, b, n);
            bleh_rx(&H, BLEH_H4_ACL, a, (uint16_t)(8 + n));
        }
        if ((i & 63) == 0) bleh_poll(&H);
        nlg = 0;
        if (H.state != BLEH_CONNECTED && (i & 255) == 0) { bring_up(); connect(); }
    }
    CHECK(H.rx_have <= sizeof H.rx);
}

int main(void)
{
    (void)SVC;
    test_init();
    test_connect_and_discovery();
    test_midi();
    test_disconnect();
    test_fuzz();
    printf("%d checks, %d failed\n", checks, fails);
    return fails != 0;
}
