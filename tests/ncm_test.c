/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 w0ts */
/* The network adapter (chip/usb.c FELUCCA_NCM, chip/usb_ncm.h) against a model of the SIE and of a computer's NCM driver.
 * Built once per layout (tests/run_ncm_tests.sh):
 *   TEST_FB 1     the audio input and output split, the output with its feedback (EP3 IN): the adapter gives way by
 *                 default, and once switched on takes the output function's interfaces (4, 5) and EP3; MIDI and the input
 *                 stay as they were, byte for byte
 *   TEST_CDC 1    the console (EP2, EP3), no audio: the adapter is off by default, switched on the console is gone
 *   neither       the audio input and output split, the output without feedback: the adapter is there from the start,
 *                 beside all of them (IF 6, 7)
 * In each: the descriptors walk (lengths, the configuration's total, interfaces, the IADs, endpoints used once each, the
 * NCM functional descriptors, the MAC string), the class requests (GET_NTB_PARAMETERS, the input size both ways, the
 * packet filter), the data interface's alt settings and the two notifications, blocks from the computer in 64-byte
 * packets (one, several datagrams, the whole 2048 bytes with no short packet, one too large, the next one held while the
 * main loop has the last), blocks to it (one frame each, a pad byte when a block would end on a full packet, too large
 * refused, busy while one goes out), a bus reset; and usb_ncm.h's NTB16 reader against malformed and random blocks.
 * It does not test the controller (its register sequences are the CDC console's) nor a real computer's driver. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#pragma GCC diagnostic push                        /* (as tests/uac_test.c: the register header casts the chip's 32-bit */
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"   /* addresses to pointers, which a 64-bit host warns about; */
#include "../chip/fm1_usb.h"                       /* the model below replaces every access, none is made) */
#pragma GCC diagnostic pop

#ifndef TEST_FB
#define TEST_FB 0
#endif
#ifndef TEST_CDC
#define TEST_CDC 0
#endif
#define TEST_AUDIO (!TEST_CDC)                          /* (a console build has no audio: Optimist's) */

/* ------------------------------------------------------- the SIE model --- */
static uint8_t regs[5][32], common[16], index_reg, last_read;
static uint8_t in3[64], in2[64];                        /* the packet queued on EP3 IN, EP2 IN */
static uint32_t in3_len, in2_len;
static uint8_t e0_in[256];                              /* what EP0 sent back for the last request */
static uint32_t e0_in_len;

static void mock_write(uint32_t r, uint32_t v)
{
    if (r == 14)
        index_reg = (uint8_t)(v & 7u);
    else if (r < 16)
        common[r] = (uint8_t)v;
    else if (index_reg == 0 && r == 17)
        regs[0][17] = 0;                                /* EP0: each request handled at once */
    else if ((index_reg == 3 || index_reg == 4) && r == 20)
        regs[index_reg][20] = (uint8_t)(v & 0x10u ? 0u : v);   /* the OUT packet taken (ServicedRxPktRdy / flush) */
    else
        regs[index_reg][r] = (uint8_t)v;
}
static void mock_rd(uint32_t r) { last_read = (uint8_t)r; }
static uint32_t mock_dt(void) { return last_read < 16 ? common[last_read] : regs[index_reg][last_read]; }
static void mock_buf(uint32_t ep, void *p) { (void)ep, (void)p; }
static void mock_ep4_buf(void *p) { (void)p; }
static void mock_send(uint32_t ep, void *p, uint32_t n)
{
    assert(n <= 64u);
    if (ep == 3u) {
        memcpy(in3, p, n);
        in3_len = n;
    } else if (ep == 2u) {
        memcpy(in2, p, n);
        in2_len = n;
    }
}
static void mock_ep4_send(void *p, uint32_t n) { (void)p, (void)n; }
static void mock_ep0_send(void *p, uint32_t n)
{
    assert(e0_in_len + n <= sizeof e0_in);
    memcpy(e0_in + e0_in_len, p, n);
    e0_in_len += n;
}
static void mock_ep0_buf(void *p) { (void)p; }
static void mock_attach(void *p) { (void)p; }
static void mock_nop(void) {}
static void mock_enable(uint32_t m) { (void)m; }
static uint32_t mock_one(void) { return 1; }
static uint32_t mock_zero(void) { return 0; }
static uint32_t mock_line(void) { return 0; }
static uint32_t off_calls;
static void mock_off(void) { off_calls++; }
static void fm1_delay_ms(uint32_t ms) { (void)ms; }

#define fm1_usb_ep_enable mock_enable
#define fm1_usb_ep_txbuf mock_buf
#define fm1_usb_ep_rxbuf mock_buf
#define fm1_usb_ep_send mock_send
#define fm1_usb_ep0_send mock_ep0_send
#define fm1_usb_ep0_buf mock_ep0_buf
#define fm1_usb_ep4_txbuf mock_ep4_buf
#define fm1_usb_ep4_rxbuf mock_ep4_buf
#define fm1_usb_ep4_send mock_ep4_send
#define fm1_usb_rx_sync mock_nop
#define fm1_usb_sie_on mock_one
#define fm1_usb_sie_done mock_one
#define fm1_usb_sie_wr_start mock_write
#define fm1_usb_sie_rd_start mock_rd
#define fm1_usb_sie_data mock_dt
#define fm1_usb_reset mock_nop
#define fm1_usb_off mock_off
#define fm1_usb_attach mock_attach
#define fm1_usb_host_attach mock_attach
#define fm1_usb_line mock_line
#define fm1_usb_sof_take mock_zero
#define RING_PUBLISH() ((void)0)
#define FELUCCA_NCM 1
#define FELUCCA_UAC TEST_AUDIO
#define FELUCCA_UAC_OUT TEST_AUDIO
#define FELUCCA_UAC_SPLIT TEST_AUDIO
#define FELUCCA_UAC_OUT_FEEDBACK TEST_FB
#define FELUCCA_UAC_48K 0
#define FELUCCA_CDC TEST_CDC
#define FELUCCA_OTA 0
#define FELUCCA_USB_HOST 0
#define HALF_FRAMES 256u
static uint32_t uac_test_clock(void) { static uint32_t t; return t += 24000u; }
#define FELUCCA_UAC_CLOCK() uac_test_clock()
#include "../chip/usb.c"

/* ------------------------------------------------------------ helpers --- */
static void request(uint8_t rt, uint8_t rq, uint16_t val, uint16_t idx, uint16_t len)   /* one EP0 SETUP, IN data kept */
{
    uint8_t s[8] = {rt, rq, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)idx, (uint8_t)(idx >> 8), (uint8_t)len,
                    (uint8_t)(len >> 8)};
    uint32_t guard = 0;
    e0_in_len = 0;
    memcpy(ep0buf, s, 8);
    regs[0][17] = 1;                                    /* RxPktRdy */
    common[S_INTRTX1] = 1;
    usb_poll();
    while (usb.e0_tx && guard++ < 16u)                  /* the rest of an IN data stage */
        usb_poll();
    common[S_INTRTX1] = 0;
    assert(!usb.e0_tx);
}
static void data_out(const uint8_t *d, uint32_t n)      /* an OUT data stage after request() */
{
    memcpy(ep0buf, d, n);
    regs[0][17] = 1;
    regs[0][22] = (uint8_t)n;
    common[S_INTRTX1] = 1;
    usb_poll();
    common[S_INTRTX1] = 0;
}
static int stalled(void) { return 0; }                  /* (the model keeps no stall bit: a stalled request sends nothing) */

/* the computer sends n bytes on EP3 OUT, 64 a packet (ending with a short one unless it ends on a full ncm_rx);
 * the packets the device leaves in the FIFO (NAK) are retried each poll, up to polls_max */
static uint32_t host_out(const uint8_t *b, uint32_t n, uint32_t polls_max)
{
    uint32_t off = 0, polls = 0;
    int zlp = n % 64u == 0 && n < NCM_RX_MAX;
    while ((off < n || zlp) && polls < polls_max) {
        uint32_t k = n - off > 64u ? 64u : n - off;
        if (!(regs[3][20] & 1u)) {                      /* the FIFO is free: the next packet */
            memcpy(ep3rx, b + off, k);
            regs[3][20] = 1;
            regs[3][22] = (uint8_t)k;
            regs[3][23] = 0;
            if (off == n)
                zlp = 0;                                /* (that was the zero-length one) */
            off += k;
        }
        common[S_INTRRX1] = 0x08;
        usb_poll();
        common[S_INTRRX1] = 0;
        polls++;
    }
    return off;
}

/* the computer reads EP3 IN until a short packet: the block, its length (0: nothing came) */
static uint32_t host_in(uint8_t *b, uint32_t cap, uint32_t polls_max)
{
    uint32_t n = 0, polls;
    for (polls = 0; polls < polls_max; polls++) {
        if (!(regs[3][17] & 1u))                        /* (a packet may wait from an earlier poll: taken first) */
            usb_poll();
        if (regs[3][17] & 1u) {                         /* a packet queued: taken */
            assert(in3_len <= 64u && n + in3_len <= cap);
            memcpy(b + n, in3, in3_len);
            n += in3_len;
            regs[3][17] &= (uint8_t)~1u;
            if (in3_len < 64u)
                return n;
        }
    }
    return n;
}

static uint32_t host_note(uint8_t *b)                   /* one notification from EP2 (0: none) */
{
    uint32_t i;
    for (i = 0; i < 4u; i++) {
        if (!(regs[2][17] & 1u))                        /* (a packet may wait from an earlier poll: taken first) */
            usb_poll();
        if (regs[2][17] & 1u) {
            memcpy(b, in2, in2_len);
            regs[2][17] &= (uint8_t)~1u;
            return in2_len;
        }
    }
    return 0;
}

/* --------------------------------------------------- the descriptors --- */
static void walk(const uint8_t *d, uint32_t l, int ncm_on)
{
    uint32_t i, nif = 0, cur_if = 0xFF, cur_alt = 0, iads = 0, ncm_fd = 0, eth_fd = 0, cur_cls = 0;
    uint8_t seen_if[16] = {0}, ep_used[2][8] = {{0}};
    assert(d[1] == 2 && (uint32_t)(d[2] | d[3] << 8) == l);
    for (i = d[0]; i < l; i += d[i]) {
        const uint8_t *p = d + i;
        assert(p[0] >= 2 && i + p[0] <= l);
        if (p[1] == 0x0B) {                             /* IAD */
            iads++;
            if (p[4] == 2) {                            /* CDC: the adapter's (the console's in a CDC build) */
                assert(!ncm_on || (p[2] == NCM_IF && p[3] == 2 && p[5] == 0x0D && p[6] == 0));
            }
        } else if (p[1] == 4) {                         /* interface */
            cur_if = p[2];
            cur_alt = p[3];
            cur_cls = p[5];
            if (!seen_if[cur_if & 15u]++)
                nif++;
            if (ncm_on && cur_if == NCM_IF) {
                assert(p[5] == 2 && p[6] == 0x0D && p[7] == 0 && p[4] == 1);
            }
            if (ncm_on && cur_if == NCM_IF + 1u) {
                assert(p[5] == 0x0A && p[6] == 0 && p[7] == 1 && p[4] == (cur_alt ? 2 : 0));
            }
        } else if (p[1] == 0x24 && cur_cls == 2) {      /* CDC functional descriptors */
            if (p[2] == 0x0F) {                         /* Ethernet networking */
                assert(p[0] == 13 && p[3] == USB_STR_MAC && (p[8] | p[9] << 8) == 1514);
                eth_fd++;
            }
            if (p[2] == 0x1A) {                         /* NCM */
                assert(p[0] == 6 && p[3] == 0x00 && p[4] == 0x01);
                ncm_fd++;
            }
            if (p[2] == 0x06)
                assert(p[3] == cur_if && p[4] == cur_if + 1u);   /* union: comm, data */
        } else if (p[1] == 5) {                         /* endpoint: each address once, in the alt that uses it */
            uint32_t dir = p[2] >> 7, num = p[2] & 0x0Fu;
            assert(num >= 1u && num <= 4u);
            if (!(ncm_on && cur_if == NCM_IF + 1u && !cur_alt))
                assert(!ep_used[dir][num]++ || (p[3] & 3u) == 1u);   /* (an isochronous one may repeat in its alts) */
            if (ncm_on && p[2] == 0x83)
                assert((p[3] & 3u) == 2u && cur_if == NCM_IF + 1u);   /* EP3 IN: the adapter's bulk, never the feedback */
        }
    }
    assert(nif == d[4]);
    assert(ncm_on ? (eth_fd == 1 && ncm_fd == 1) : (eth_fd == 0 && ncm_fd == 0));
    (void)iads;
}

static void test_descriptors(void)
{
    const uint8_t *d, *dv, *dp;
    uint16_t l, dl, lp, i;
    static const uint8_t MAC[6] = {0x02, 0x66, 0x6D, 0x31, 0xA0, 0x0F};
    static const char MACS[] = "02666D31A00F";
    /* off (when something gives way), presented as without the adapter */
    if (!FELUCCA_NCM_DEFAULT) {
        assert(!usb_ncm_on && get_desc(0x0200, &dp, &lp));
        walk(dp, lp, 0);
        assert(!get_desc(0x0300 | USB_STR_MAC, &d, &l));
#if TEST_FB
        assert(memmem(dp, lp, (const uint8_t[]){7, 5, 0x83}, 3) || memmem(dp, lp, (const uint8_t[]){9, 5, 0x83}, 3));
#endif
        usb_ncm_switch(1);
    }
    assert(usb_ncm_on);
    usb_ncm_set_mac(MAC);
    assert(get_desc(0x0100, &dv, &dl) && dl == 18);
    assert(dv[4] == 0xEF && dv[5] == 0x02 && dv[6] == 0x01 && dv[12] == (uint8_t)BCD_LO(0) && dv[13] == BCD_HI);
    assert(get_desc(0x0200, &d, &l) && l == CFG_LEN_NCM);
    walk(d, l, 1);
#if TEST_FB
    assert(NCM_IF == 4 && d[4] == 6);                   /* the output's two interfaces are the adapter's */
    assert(!memmem(d, l, (const uint8_t[]){9, 5, 0x04}, 3));   /* no EP4 OUT: the output is not there */
    {                                                   /* MIDI and the input as before, byte for byte */
        const uint8_t *plain;
        uint16_t pl;
        uint8_t sw = usb_ncm_on;
        usb_ncm_on = 0;
        assert(get_desc(0x0200, &plain, &pl));
        usb_ncm_on = sw;
        assert(!memcmp(plain + 9, d + 9, (uint32_t)(AUD_LEN_NET)));
    }
#elif TEST_CDC
    assert(NCM_IF == AUD_NIF && d[4] == AUD_NIF + 2);
    assert(!usb_cdc_on);
#else
    assert(NCM_IF == 6 && d[4] == 8);                   /* MIDI, the input, the output, the adapter */
    assert(memmem(d, l, (const uint8_t[]){9, 5, 0x04}, 3));   /* the output stays (no feedback: EP4 alone) */
#endif
    assert(get_desc(0x0300 | USB_STR_MAC, &d, &l) && l == 26 && d[1] == 3);
    for (i = 0; i < 12u; i++)
        assert(d[2 + 2 * i] == (uint8_t)MACS[i] && d[3 + 2 * i] == 0);
    printf("NCM descriptors (%s): IF %u-%u, %u bytes: OK\n", TEST_FB ? "split, feedback" : TEST_CDC ? "console" : "split",
           (unsigned)NCM_IF, (unsigned)NCM_IF + 1u, (unsigned)CFG_LEN_NCM);
}

/* ------------------------------------------------------- the requests --- */
static void test_requests(void)
{
    uint8_t v[4] = {0x00, 0x40, 0x00, 0x00};            /* 16384 */
    uint8_t note[16];
    usb_start();
    assert(usb.up);
    request(0x00, 0x09, 1, 0, 0);                       /* SET_CONFIGURATION 1 */
    assert(usb.config == 1u && !usb_ncm_ready());
    request(0xA1, 0x80, 0, NCM_IF, 28);                 /* GET_NTB_PARAMETERS */
    assert(e0_in_len == 28u && e0_in[0] == 28 && e0_in[2] == 1);
    assert(ncm_rd16(e0_in + 4) == NCM_IN_MAX && ncm_rd16(e0_in + 16) == NCM_RX_MAX);
    assert(ncm_rd16(e0_in + 8) == 4 && ncm_rd16(e0_in + 12) == 4 && ncm_rd16(e0_in + 20) == 4 && ncm_rd16(e0_in + 24) == 4);
    assert(ncm_rd16(e0_in + 26) == 1);                  /* one datagram a block from the computer */
    request(0xA1, 0x80, 0, NCM_IF, 8);                  /* a short read: the first 8 */
    assert(e0_in_len == 8u);
    request(0xA1, 0x80, 0, NCM_IF + 1u, 28);            /* not to the communication interface: nothing */
    assert(e0_in_len == 0u);
    (void)stalled;
    request(0xA1, 0x85, 0, NCM_IF, 4);                  /* GET_NTB_INPUT_SIZE: ours until told */
    assert(e0_in_len == 4u && ncm_rd16(e0_in) == NCM_IN_MAX);
    request(0x21, 0x86, 0, NCM_IF, 4);                  /* SET_NTB_INPUT_SIZE: 4 bytes follow */
    assert(ncm.e0_rx);
    data_out(v, 4);
    assert(!ncm.e0_rx && ncm.in_max == 16384u);
    request(0xA1, 0x85, 0, NCM_IF, 4);
    assert(ncm_rd16(e0_in) == 0x4000);
    request(0x21, 0x43, 0x000C, NCM_IF, 0);             /* SET_ETHERNET_PACKET_FILTER */
    request(0x81, 0x0A, 0, NCM_IF + 1u, 1);             /* GET_INTERFACE: alt 0 */
    assert(e0_in_len == 1u && e0_in[0] == 0);
    request(0x01, 0x0B, 1, NCM_IF + 1u, 0);             /* SET_INTERFACE alt 1: the link up */
    assert(usb_ncm_ready() && ncm.in_max == 16384u);    /* (the input size set in alt 0 holds in alt 1: NCM 1.0) */
#if TEST_AUDIO
    assert(!uo.alt);                                    /* (never the output's stream, whose number it may have) */
#endif
    request(0x81, 0x0A, 0, NCM_IF + 1u, 1);
    assert(e0_in[0] == 1);
    assert(host_note(note) == 16u && note[0] == 0xA1 && note[1] == 0x2A && note[4] == NCM_IF && note[6] == 8);
    assert(ncm_rd16(note + 8) == (NCM_BPS & 0xFFFFu) && ncm_rd16(note + 10) == NCM_BPS >> 16);
    assert(host_note(note) == 8u && note[0] == 0xA1 && note[1] == 0x00 && note[2] == 1 && note[4] == NCM_IF);
    assert(host_note(note) == 0u);                      /* nothing more */
    printf("NCM requests and notifications: OK\n");
}

/* ------------------------------------------------- the blocks, both ways --- */
static uint8_t got[8][1600];
static uint32_t got_len[8], got_n;
static void take(void *ctx, const uint8_t *d, uint32_t len)
{
    (void)ctx;
    assert(got_n < 8u && len <= sizeof got[0]);
    memcpy(got[got_n], d, len);
    got_len[got_n++] = len;
}
static uint32_t make_ntb(uint8_t *b, uint32_t pad_to, uint32_t nd, const uint32_t *lens, uint8_t seed)
{
    uint32_t off = 12u + 8u + 4u * (nd + 1u < 2u ? 2u : nd + 1u), i, k;   /* NTH, the NDP (pairs, the end), the datagrams */
    memset(b, 0, pad_to ? pad_to : 4096u);
    memcpy(b, "NCMH", 4);
    ncm_wr16(b + 4, 12);
    ncm_wr16(b + 10, 12);
    memcpy(b + 12, "NCM0", 4);
    ncm_wr16(b + 16, 8u + 4u * (nd + 1u < 2u ? 2u : nd + 1u));   /* (16 at least: NCM 1.0) */
    off = (off + 3u) & ~3u;
    for (i = 0; i < nd; i++) {
        ncm_wr16(b + 20 + 4 * i, off);
        ncm_wr16(b + 22 + 4 * i, lens[i]);
        for (k = 0; k < lens[i]; k++)
            b[off + k] = (uint8_t)(seed + i * 31u + k);
        off = (off + lens[i] + 3u) & ~3u;
    }
    if (pad_to)
        off = pad_to;
    ncm_wr16(b + 8, off);
    return off;
}

static void test_blocks(void)
{
    static uint8_t b[4096], back[2048];
    static const uint32_t one[1] = {98}, three[3] = {60, 590, 14}, big[1] = {1514};
    const uint8_t *p;
    uint32_t n, len, k;
    uint8_t *f;
    /* one datagram */
    n = make_ntb(b, 0, 1, one, 7);
    assert(host_out(b, n, 40) == n);
    len = usb_ncm_rx(&p);
    assert(len == n && !memcmp(p, b, n));
    got_n = 0;
    assert(ncm_ntb_each(p, len, take, 0) == 1 && got_len[0] == 98 && got[0][0] == 7);
    /* the next block waits (NAK) while the main loop has this one */
    n = make_ntb(b, 0, 3, three, 50);
    assert(host_out(b, n, 4) == 64u && (regs[3][20] & 1u));   /* the first packet taken, the second left in the FIFO */
    usb_ncm_rx_done();
    assert(host_out(b + 64, n - 64u, 60) == n - 64u);
    len = usb_ncm_rx(&p);
    assert(len == n);
    got_n = 0;
    assert(ncm_ntb_each(p, len, take, 0) == 3 && got_len[1] == 590 && got[1][0] == (uint8_t)(50 + 31));
    usb_ncm_rx_done();
    /* the whole 2048 bytes, no short packet: the block ends when ncm_rx is full */
    n = make_ntb(b, NCM_RX_MAX, 1, big, 3);
    assert(n == NCM_RX_MAX && host_out(b, n, 80) == n);
    len = usb_ncm_rx(&p);
    assert(len == NCM_RX_MAX);
    got_n = 0;
    assert(ncm_ntb_each(p, len, take, 0) == 1 && got_len[0] == 1514);
    usb_ncm_rx_done();
    /* longer than dwNtbOutMaxSize (a host breaking its word): a full ncm_rx is a block's end, so it comes as two, the
     * 2048 and the tail, each one the reader refuses; the next one whole */
    memset(b, 0xAB, sizeof b);
    assert(host_out(b, 2100, 80) == 2100u && (regs[3][20] & 1u));   /* (the tail waits in the FIFO, NAKed) */
    assert((len = usb_ncm_rx(&p)) == NCM_RX_MAX && ncm_ntb_each(p, len, take, 0) == -1);
    usb_ncm_rx_done();
    common[S_INTRRX1] = 0x08;
    usb_poll();
    common[S_INTRRX1] = 0;
    assert((len = usb_ncm_rx(&p)) == 52u && ncm_ntb_each(p, len, take, 0) == -1);
    usb_ncm_rx_done();
    n = make_ntb(b, 0, 1, one, 9);
    assert(host_out(b, n, 40) == n && usb_ncm_rx(&p) == n);
    usb_ncm_rx_done();
    /* a block of 128 bytes, a multiple of 64: the zero-length packet ends it */
    n = make_ntb(b, 128, 1, one, 11);
    assert(n == 128u && host_out(b, n, 40) == n && usb_ncm_rx(&p) == 128u);
    usb_ncm_rx_done();
    printf("NCM blocks from the computer: OK\n");

    /* to the computer: one frame a block */
    f = usb_ncm_tx_frame();
    assert(f);
    for (k = 0; k < 300u; k++)
        f[k] = (uint8_t)(k * 7u);
    assert(usb_ncm_tx_send(300));
    assert(!usb_ncm_tx_frame() && !usb_ncm_tx_send(300));   /* busy while it goes out */
    n = host_in(back, sizeof back, 40);
    assert(n == NCM_OUT_HDR + 300u);
    got_n = 0;
    assert(ncm_ntb_each(back, n, take, 0) == 1 && got_len[0] == 300u && got[0][299] == (uint8_t)(299u * 7u));
    host_in(back, sizeof back, 4);                      /* (the poll that sees it all taken frees the buffer) */
    /* a block that would end on a full packet: a pad byte */
    f = usb_ncm_tx_frame();
    assert(f);
    memset(f, 0x5A, 192u - NCM_OUT_HDR);
    assert(usb_ncm_tx_send(192u - NCM_OUT_HDR));
    n = host_in(back, sizeof back, 40);
    assert(n == 193u && ncm_rd16(back + 8) == 193u);
    got_n = 0;
    assert(ncm_ntb_each(back, n, take, 0) == 1 && got_len[0] == 192u - NCM_OUT_HDR);
    host_in(back, sizeof back, 4);
    /* too large: refused, counted, nothing sent */
    k = ncm.tx_big;
    assert(usb_ncm_tx_frame() && !usb_ncm_tx_send(FELUCCA_NCM_TX_MAX + 1u) && ncm.tx_big == k + 1u);
    assert(host_in(back, sizeof back, 4) == 0u);
    /* the largest: whole */
    f = usb_ncm_tx_frame();
    memset(f, 0x11, FELUCCA_NCM_TX_MAX);
    assert(usb_ncm_tx_send(FELUCCA_NCM_TX_MAX));
    n = host_in(back, sizeof back, 80);
    assert(n >= NCM_OUT_HDR + FELUCCA_NCM_TX_MAX && ncm_ntb_each(back, n, take, 0) == 1);
    host_in(back, sizeof back, 4);
    printf("NCM blocks to the computer: OK\n");

    /* alt 0: the link down, nothing goes out; a bus reset forgets it all */
    request(0x01, 0x0B, 0, NCM_IF + 1u, 0);
    assert(!usb_ncm_ready() && !usb_ncm_tx_frame());
    request(0x01, 0x0B, 1, NCM_IF + 1u, 0);
    assert(usb_ncm_ready());
    {                                                   /* a block the main loop holds survives a reset untouched */
        static const uint32_t one[1] = {60};
        static uint8_t b2[4096];
        const uint8_t *p2;
        uint32_t n2 = make_ntb(b2, 0, 1, one, 21);
        assert(host_out(b2, n2, 20) == n2 && usb_ncm_rx(&p2) == n2);
        request(0x01, 0x0B, 1, NCM_IF + 1u, 0);         /* (the host sets the alt again meanwhile) */
        assert(usb_ncm_rx(&p2) == n2 && !memcmp(p2, b2, n2));
        usb_ncm_rx_done();
    }
    common[S_INTRUSB] = 0x04;
    usb_poll();
    common[S_INTRUSB] = 0;
    assert(!usb_ncm_ready() && !ncm.alt && !usb.config);
    /* switched off while attached: off the bus (usb_retry brings it back), the other function's descriptors */
    if (!FELUCCA_NCM_DEFAULT) {
        uint32_t offs = off_calls;
        const uint8_t *d;
        uint16_t l;
        usb_ncm_switch(0);
        assert(!usb.up && off_calls == offs + 1u && !usb_ncm_on);
        assert(get_desc(0x0200, &d, &l) && !memmem(d, l, (const uint8_t[]){13, 0x24, 0x0F}, 3));
#if TEST_CDC
        usb_cdc_switch(1);
        usb_ncm_switch(1);
        assert(!usb_cdc_on && usb_ncm_on);              /* (one or the other) */
        usb_ncm_switch(0);
        assert(usb_cdc_on && !usb_ncm_on);              /* the adapter off: the console back */
        usb_ncm_switch(1);
        usb_cdc_switch(1);
        assert(usb_cdc_on && !usb_ncm_on);
        usb_cdc_switch(0);
        usb_ncm_switch(1);
        usb_ncm_switch(0);
        assert(!usb_cdc_on);                            /* (off before: off after) */
#endif
    }
    printf("NCM alt settings, bus reset, switching: OK\n");
}

/* -------------------------------------------- the NTB16 reader alone --- */
static void nop_take(void *ctx, const uint8_t *d, uint32_t len)
{
    volatile uint8_t s = 0;
    uint32_t i;
    (void)ctx;
    for (i = 0; i < len; i++)
        s ^= d[i];                                      /* (every byte read: ASan sees one out of the block) */
    (void)s;
}
static void test_reader(void)
{
    static uint8_t b[4096];                             /* (make_ntb clears 4096) */
    static const uint32_t two[2] = {20, 40};
    uint32_t n, i, r;
    n = make_ntb(b, 0, 2, two, 1);
    assert(ncm_ntb_each(b, n, nop_take, 0) == 2);
    assert(ncm_ntb_each(b, n - 1u, nop_take, 0) == -1);   /* wBlockLength past what came */
    b[0] = 'X';
    assert(ncm_ntb_each(b, n, nop_take, 0) == -1);
    n = make_ntb(b, 0, 2, two, 1);
    ncm_wr16(b + 10, 13);                               /* the NDP not on 4 bytes */
    assert(ncm_ntb_each(b, n, nop_take, 0) == -1);
    n = make_ntb(b, 0, 2, two, 1);
    ncm_wr16(b + 22, 500);                              /* a datagram past the block */
    assert(ncm_ntb_each(b, n, nop_take, 0) == -1);
    n = make_ntb(b, 0, 2, two, 1);
    ncm_wr16(b + 18, 12);                               /* the NDP its own next: the loop guard */
    assert(ncm_ntb_each(b, n, nop_take, 0) == -1);
    n = make_ntb(b, 0, 2, two, 1);
    ncm_wr16(b + 22, 4);                                /* shorter than an Ethernet header */
    assert(ncm_ntb_each(b, n, nop_take, 0) == -1);
    n = make_ntb(b, 0, 0, two, 1);                      /* no datagram at all: a block, none given */
    assert(ncm_ntb_each(b, n, nop_take, 0) == 0);
    srand(1);
    for (i = 0; i < 200000u; i++) {                     /* random blocks with a valid start: never out of bounds */
        uint32_t len = 12u + (uint32_t)rand() % 500u, k;
        uint8_t *h = malloc(len);
        for (k = 0; k < len; k++)
            h[k] = (uint8_t)rand();
        memcpy(h, "NCMH", 4);
        ncm_wr16(h + 4, 12);
        ncm_wr16(h + 8, (uint32_t)rand() % (len + 1u));
        ncm_wr16(h + 10, ((uint32_t)rand() % len) & ~3u);
        if (rand() & 1)
            memcpy(h + (ncm_rd16(h + 10) + 4u <= len ? ncm_rd16(h + 10) : 0), "NCM0", 4);
        r = (uint32_t)ncm_ntb_each(h, len, nop_take, 0);
        (void)r;
        free(h);
    }
    printf("NTB16 reader, malformed and random blocks: OK\n");
}

int main(void)
{
    test_reader();
    test_descriptors();
    test_requests();
    test_blocks();
    return 0;
}
