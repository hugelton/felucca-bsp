/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 henri (hdavid) */
/* USB audio input and output (chip/usb.c, FELUCCA_UAC): the driver against a model of the SIE and of a host that
 * takes one isochronous packet a millisecond, with the I2S at its own clock (44,117.6 Hz). Built once per
 * configuration (tests/run_usb_host_tests.sh): FELUCCA_UAC_CH (TEST_CH), FELUCCA_UAC_24 (TEST_24), FELUCCA_UAC_OUT
 * (TEST_OUT), FELUCCA_CDC (TEST_CDC), FELUCCA_UAC_HOST_CLOCK (TEST_HC), FELUCCA_UAC_OUT_FEEDBACK (TEST_FB),
 * FELUCCA_UAC_OUT_24 (TEST_O24), FELUCCA_UAC_SPLIT (TEST_SPLIT; TEST_NAMES: FELUCCA_UAC_CH_NAMES and the functions' names).
 *   - the configuration descriptor walks as UAC1 (lengths, interfaces, the AC header's total, the input terminal, each
 *     streaming alt's format and packet size); in the default configuration, byte for byte the descriptors before
 *     FELUCCA_UAC_CH / FELUCCA_UAC_24 (TEST_GOLDEN)
 *   - each alt streams: the frames the render gave, in order and whole once primed (none lost, none repeated), packed
 *     16 or 24-bit; uac_tap(0, n) is silence; uac_capturing follows the host
 *   - uac_play_take hands over exactly what uac_play would have added to silence at unity, and silence when there is
 *     nothing to play
 *   - on the host's clock (TEST_HC): the descriptors as the default's but EP4 IN synchronous and bcdDevice +2.00;
 *     minutes of a model crystal off by +-100, +-500 ppm and drifting: the strict 44.1 pattern, no ring under- or
 *     overrun, the servo's settling and wander, the output against a reference sine (THD+N, the largest error)
 *   - the output against a sine the host plays (THD+N), the FM-1's crystal off: resampled (adaptive), or with the
 *     feedback (TEST_FB): the descriptors as the default's but the output's endpoint asynchronous with its feedback
 *     endpoint and bcdDevice +0.80; a host that sends what the feedback asks, minutes at 0, +-100, +-500 ppm and
 *     drifting: every sample played once, in order, as sent (bit for bit), no under- or overrun, the fill near its
 *     target and the feedback the I2S's true rate
 *   - the output in 24-bit too (TEST_O24): its alt 2 in the descriptors (bcdDevice +8.00), the same streams in it,
 *     every 24-bit sample reaching the render whole
 *   - the input and the output as two functions (TEST_SPLIT): three IADs, three audio control interfaces each with its
 *     one streaming interface (MIDI's alone), byte for byte as written out here in the default's options; the names'
 *     string descriptors; every stream test above on the moved interfaces
 * It does not test the controller (the SIE's register sequences are the device's own) nor a real host's driver. */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846   /* (not in strict C11's math.h) */
#endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#include "../chip/fm1_usb.h"
#pragma GCC diagnostic pop

#ifndef TEST_CH
#define TEST_CH 2
#endif
#ifndef TEST_24
#define TEST_24 0
#endif
#ifndef TEST_OUT
#define TEST_OUT 1
#endif
#ifndef TEST_CDC
#define TEST_CDC 0
#endif
#ifndef TEST_HC
#define TEST_HC 0                                       /* FELUCCA_UAC_HOST_CLOCK */
#endif
#ifndef TEST_FB
#define TEST_FB 0                                       /* FELUCCA_UAC_OUT_FEEDBACK */
#endif
#ifndef TEST_O24
#define TEST_O24 0                                      /* FELUCCA_UAC_OUT_24 */
#endif
#ifndef TEST_SPLIT
#define TEST_SPLIT 0                                    /* FELUCCA_UAC_SPLIT */
#endif
#ifndef TEST_NAMEFN
#define TEST_NAMEFN 0                                   /* with it: FELUCCA_USB_NAME_FN gives "FM1-1A2B" (the product, " In", " Out") */
#endif
#ifndef TEST_NAMES
#define TEST_NAMES 0                                    /* with it: the names given */
#endif
#if TEST_SPLIT || TEST_NAMES
#define SERIAL_STR 0x30                                 /* the serial string: clear of the names' (3, 4, 5..) */
#else
#define SERIAL_STR 3                                    /* .. else upstream's */
#endif
#ifndef TEST_GOLDEN
#define TEST_GOLDEN (TEST_CH == 2 && !TEST_24 && !TEST_CDC && !TEST_SPLIT)   /* the configuration felucca-bsp had before them */
#endif

/* ------------------------------------------------------- the SIE model --- */
static uint8_t regs[5][32], common[16], index_reg, last_read;
static uint8_t *ep4rx_buf;
static uint8_t in_pkt[1024];                            /* the packet queued on EP4 IN */
static uint32_t in_len;
static uint8_t fb_pkt[4];                               /* the packet queued on EP3 IN (the output's feedback) */
static uint32_t fb_len;

static void mock_write(uint32_t r, uint32_t v)
{
    if (r == 14)
        index_reg = (uint8_t)(v & 7u);
    else if (r < 16)
        common[r] = (uint8_t)v;
    else if (index_reg == 0 && r == 17)
        regs[0][17] = 0;                                /* EP0: each request handled at once */
    else if ((index_reg == 4 || (TEST_FB && index_reg == 3)) && r == 17)
        regs[index_reg][17] = (uint8_t)(v & 1u);        /* TxPktRdy until the host takes the packet */
    else if (index_reg == 4 && r == 20)
        regs[4][20] = (uint8_t)(v & 0x10u ? 0u : v);    /* the OUT packet taken */
    else
        regs[index_reg][r] = (uint8_t)v;
}
static void mock_rd(uint32_t r) { last_read = (uint8_t)r; }
static uint32_t mock_dt(void) { return last_read < 16 ? common[last_read] : regs[index_reg][last_read]; }
static void mock_buf(uint32_t ep, void *p) { (void)ep, (void)p; }
static void mock_ep4_txbuf(void *p) { (void)p; }
static void mock_ep4_rxbuf(void *p) { ep4rx_buf = p; }
static void mock_send(uint32_t ep, void *p, uint32_t n)
{
    if (ep == 3u && TEST_FB) {                          /* the feedback */
        assert(n <= sizeof fb_pkt);
        memcpy(fb_pkt, p, n);
        fb_len = n;
    }
}
static void mock_ep4_send(void *p, uint32_t n)
{
    assert(n <= sizeof in_pkt);
    memcpy(in_pkt, p, n);
    in_len = n;
}
static void mock_ep0_send(void *p, uint32_t n) { (void)p, (void)n; }
static void mock_ep0_buf(void *p) { (void)p; }
static void mock_attach(void *p) { (void)p; }
static void mock_nop(void) {}
static void mock_enable(uint32_t m) { (void)m; }
static uint32_t mock_one(void) { return 1; }
static uint32_t mock_zero(void) { return 0; }
static uint32_t mock_line(void) { return 0; }
static void fm1_delay_ms(uint32_t ms) { (void)ms; }

#define fm1_usb_ep_enable mock_enable
#define fm1_usb_ep_txbuf mock_buf
#define fm1_usb_ep_rxbuf mock_buf
#define fm1_usb_ep_send mock_send
#define fm1_usb_ep0_send mock_ep0_send
#define fm1_usb_ep0_buf mock_ep0_buf
#define fm1_usb_ep4_txbuf mock_ep4_txbuf
#define fm1_usb_ep4_rxbuf mock_ep4_rxbuf
#define fm1_usb_ep4_send mock_ep4_send
#define fm1_usb_rx_sync mock_nop
#define fm1_usb_sie_on mock_one
#define fm1_usb_sie_done mock_one
#define fm1_usb_sie_wr_start mock_write
#define fm1_usb_sie_rd_start mock_rd
#define fm1_usb_sie_data mock_dt
#define fm1_usb_reset mock_nop
#define fm1_usb_off mock_nop
#define fm1_usb_attach mock_attach
#define fm1_usb_host_attach mock_attach
#define fm1_usb_line mock_line
#define fm1_usb_sof_take mock_zero
#define RING_PUBLISH() ((void)0)
#define FELUCCA_UAC 1
#define FELUCCA_UAC_48K 0
#define FELUCCA_UAC_OUT TEST_OUT
#define FELUCCA_UAC_CH TEST_CH
#if !defined(TEST_TAP) && !defined(FELUCCA_UAC_CAPTURE)
#define FELUCCA_UAC_CAPTURE (TEST_CH != 2)          /* the caller fills every channel; TEST_TAP: the master and uac_ch_frame */
#endif
#define FELUCCA_UAC_24 TEST_24
#define FELUCCA_UAC_HOST_CLOCK TEST_HC
#define FELUCCA_UAC_OUT_FEEDBACK TEST_FB
#define FELUCCA_UAC_OUT_24 TEST_O24
#define FELUCCA_UAC_SPLIT TEST_SPLIT
#ifndef TEST_MEM
#define TEST_MEM 0                                      /* FELUCCA_UAC_MEM: the rings a block the app lends (uac_mem) */
#endif
#define FELUCCA_UAC_MEM TEST_MEM
#if TEST_NAMES
#define FELUCCA_UAC_IN_NAME "Optimist In"
#define FELUCCA_UAC_OUT_NAME "Optimist Out"
#if TEST_CH == 2
#define FELUCCA_UAC_CH_NAMES "MASTER L", "MASTER R"
#elif TEST_CH == 4
#define FELUCCA_UAC_CH_NAMES "CH1", "CH2", "CH3", "CH4"
#endif
#endif
#if TEST_NAMEFN
const char *test_name_fn(void) { return "FM1-1A2B"; }       /* FELUCCA_USB_NAME_FN: the name asked at run time */
#define FELUCCA_USB_NAME_FN test_name_fn
#endif
#ifndef TEST_SERIAL
#define TEST_SERIAL 0                                   /* with it: FELUCCA_USB_SERIAL_FN gives "0123456789ABCDEF" */
#endif
#if TEST_SERIAL
const char *test_serial_fn(void) { return "0123456789ABCDEF"; }   /* FELUCCA_USB_SERIAL_FN: this device's serial number */
#define FELUCCA_USB_SERIAL_FN test_serial_fn
#endif
#define FELUCCA_CDC TEST_CDC
#define FELUCCA_OTA 0
#define FELUCCA_USB_HOST 0
#define HALF_FRAMES 256u
/* uac_size's clock (the FM-1's crystal, 24 MHz): a packet a call, each a host's ms later (the pacing itself:
 * tests/usb_host_test.c test_uac_pace) */
static uint32_t uac_test_clock(void) { static uint32_t t; return t += 24000u; }
#define FELUCCA_UAC_CLOCK() uac_test_clock()
#include "../chip/usb.c"

/* ------------------------------------------------------------ helpers --- */
static void setup(uint8_t rt, uint8_t rq, uint16_t val, uint16_t idx)   /* one EP0 request with no data stage */
{
    uint8_t s[8] = {rt, rq, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)idx, (uint8_t)(idx >> 8), 0, 0};
    memcpy(ep0buf, s, 8);
    regs[0][17] = 1;                                    /* RxPktRdy */
    common[S_INTRTX1] = 1;
    usb_poll();
    common[S_INTRTX1] = 0;
    assert(!usb.e0_tx);
}

static void render_start_idle(void);
static uint32_t frame_bytes(uint32_t alt) { return UA_CH * (alt == 2u ? 3u : 2u); }

/* --------------------------------------------------- the descriptors --- */
#if TEST_GOLDEN
/* felucca-bsp 2cd696a, FELUCCA_UAC 1, FELUCCA_UAC_OUT TEST_OUT, FELUCCA_UAC_48K 0, FELUCCA_CDC 0 (fm1-base's) */
#if TEST_OUT
static const uint8_t GOLD_CFG[] = {
    9, 2, 249, 0, 4, 1, 0, 128, 50, 9, 4, 0, 0, 0, 1, 1,
    0, 0, 11, 36, 1, 0, 1, 53, 0, 3, 1, 2, 3, 12, 36, 2,
    1, 3, 6, 0, 2, 3, 0, 0, 0, 9, 36, 3, 2, 1, 1, 0,
    1, 0, 12, 36, 2, 3, 1, 1, 0, 2, 3, 0, 0, 0, 9, 36,
    3, 4, 1, 3, 0, 3, 0, 9, 4, 1, 0, 2, 1, 3, 0, 0,
    7, 36, 1, 0, 1, 65, 0, 6, 36, 2, 1, 1, 0, 6, 36, 2,
    2, 2, 0, 9, 36, 3, 1, 3, 1, 2, 1, 0, 9, 36, 3, 2,
    4, 1, 1, 1, 0, 9, 5, 1, 2, 64, 0, 0, 0, 0, 5, 37,
    1, 1, 1, 9, 5, 129, 2, 64, 0, 0, 0, 0, 5, 37, 1, 1,
    3, 9, 4, 2, 0, 0, 1, 2, 0, 0, 9, 4, 2, 1, 1, 1,
    2, 0, 0, 7, 36, 1, 2, 1, 1, 0, 11, 36, 2, 1, 2, 2,
    16, 1, 68, 172, 0, 9, 5, 132, 5, 184, 0, 1, 0, 0, 7, 37,
    1, 1, 0, 0, 0, 9, 4, 3, 0, 0, 1, 2, 0, 0, 9, 4,
    3, 1, 1, 1, 2, 0, 0, 7, 36, 1, 3, 1, 1, 0, 11, 36,
    2, 1, 2, 2, 16, 1, 68, 172, 0, 9, 5, 4, 9, 196, 0, 1,
    0, 0, 7, 37, 1, 0, 0, 0, 0
};
#else
static const uint8_t GOLD_CFG[] = {
    9, 2, 175, 0, 3, 1, 0, 128, 50, 9, 4, 0, 0, 0, 1, 1,
    0, 0, 10, 36, 1, 0, 1, 31, 0, 2, 1, 2, 12, 36, 2, 1,
    3, 6, 0, 2, 3, 0, 0, 0, 9, 36, 3, 2, 1, 1, 0, 1,
    0, 9, 4, 1, 0, 2, 1, 3, 0, 0, 7, 36, 1, 0, 1, 65,
    0, 6, 36, 2, 1, 1, 0, 6, 36, 2, 2, 2, 0, 9, 36, 3,
    1, 3, 1, 2, 1, 0, 9, 36, 3, 2, 4, 1, 1, 1, 0, 9,
    5, 1, 2, 64, 0, 0, 0, 0, 5, 37, 1, 1, 1, 9, 5, 129,
    2, 64, 0, 0, 0, 0, 5, 37, 1, 1, 3, 9, 4, 2, 0, 0,
    1, 2, 0, 0, 9, 4, 2, 1, 1, 1, 2, 0, 0, 7, 36, 1,
    2, 1, 1, 0, 11, 36, 2, 1, 2, 2, 16, 1, 68, 172, 0, 9,
    5, 132, 5, 184, 0, 1, 0, 0, 7, 37, 1, 1, 0, 0, 0
};
#endif
static const uint8_t GOLD_DEV[18] = {18, 1, 0x10, 0x01, 0, 0, 0, 64, 0x09, 0x12, 0x01, 0x00,
                                     0x10 + 0x40 * TEST_OUT, 0x03, 1, 2, 0, 1};
#endif

/* the device's bcdDevice and a CRC-32 of its descriptors (the device's and the configuration's, bcdDevice and the serial
 * string's index left out), one line a run: tests/run_usb_host_tests.sh checks that one bcdDevice is one set of
 * descriptors. Not with the channels' names (TEST_NAMES: their string indices are part of the descriptors), nor with
 * FELUCCA_UAC_CH 4..10 without FELUCCA_UAC_CAPTURE (TEST_TAP: bcdDevice has never told those channel counts apart) */
static uint32_t bcd_crc(const uint8_t *p, uint32_t n, uint32_t c)
{
    uint32_t k;
    while (n--) {
        c ^= *p++;
        for (k = 0; k < 8u; k++)
            c = (c >> 1) ^ (c & 1u ? 0xEDB88320u : 0u);
    }
    return c;
}
static void bcd_line(void)
{
#if !TEST_NAMES
    const uint8_t *d, *dv;
    uint16_t l, dl;
    uint8_t v[18];
    uint32_t c;
    assert(get_desc(0x0100, &dv, &dl) && dl == 18 && get_desc(0x0200, &d, &l));
    memcpy(v, dv, 18);
    v[12] = v[13] = v[16] = 0;
    c = bcd_crc(d, l, bcd_crc(v, 18, 0xFFFFFFFFu));
    printf("bcdDevice %02X%02X %08X\n", dv[13], dv[12], ~c);
#endif
}

#if TEST_TAP
/* the other way to fill FELUCCA_UAC_CH (FELUCCA_UAC_CAPTURE 0): the master by uac_tap on 1 and 2, the app's channels by
 * uac_ch_frame on 3..N, each packet in frame order across the ring's end */
static void test_tap(void)
{
    static int32_t out[2u * HALF_FRAMES];
    uint32_t i, pass, sent = 1, got = 1, run, c;
    static uint32_t pkt[(UAC_MAXP + 3u) / 4u];
    usb.config = 1;
    uac.alt = 1;
    uac.flowing = 1;
    uac.go = 0;
    for (pass = 0; pass < 200u; pass++) {
        uac_render_start();
        for (i = 0; i < HALF_FRAMES; i++) {
            int16_t *p = uac_ch_frame(i, &run);
            assert(!uac.feed || (p && run));
            if (p)
                for (c = 0; c < UAC_NCH - 2u; c++)
                    p[c] = (int16_t)((sent + i) * (c + 2u));
            out[2u * i] = (int32_t)(((sent + i) % 30000u) + 1u) << 8;
            out[2u * i + 1u] = -((int32_t)(((sent + i) % 30000u) + 1u) << 8);
        }
        uac_tap(out, HALF_FRAMES);
        sent += uac.feed ? HALF_FRAMES : 0u;
        while (ua_fill(ua_w, ua_r) > 64u + UA_MAXF) {   /* (some kept in hand: the ring's old low band is gone) */
            uint32_t n = uac_packet(pkt);
            const int16_t *d = (const int16_t *)pkt;
            for (i = 0; i < n; i++, d += UAC_NCH) {
                uint32_t want = (uint32_t)d[0];
                if (!want)
                    continue;                   /* (the silence the stream starts with) */
                assert(want == (got % 30000u) + 1u && d[1] == -d[0]);
                for (c = 2; c < UAC_NCH; c++)
                    assert(d[c] == (int16_t)(got * c));
                got++;
            }
        }
    }
    assert(got > 1000u && !uac.underruns);
    printf("USB audio: the tap's master and uac_ch_frame's channels, %u frames, %u channels: OK\n", (unsigned)got, (unsigned)UAC_NCH);
}

int main(void)
{
    test_tap();
    return 0;
}
#else
static void test_descriptors(void)
{
    const uint8_t *d, *dv;
    uint16_t l, dl, i;
    uint32_t nif = 0, ac_total = 0, ac_sum = 0, alts = 0, it_ok = 0, cur_if = 0xFF, cur_alt = 0, sub = 0;
    uint32_t out_eps = 0, fb_eps = 0, out_alts = 0, osub = 0, cur_ac = 0, acs = 0, iads = 0;
    uint8_t seen_if[16] = {0};
    assert(get_desc(0x0100, &dv, &dl) && dl == 18);
    assert(get_desc(0x0200, &d, &l));
    assert(d[1] == 2 && (d[2] | d[3] << 8) == l);
    assert(dv[12] == (uint8_t)BCD_LO(USB_CDC_ON) && dv[13] == BCD_HI);
    assert(dv[13] == 0x03 + 8 * TEST_O24 + 0x10 * (!FELUCCA_UAC_CAPTURE ? 0 : TEST_CH == 8 ? 3 : TEST_CH == 4 ? 2 : 1) + 0x40 * TEST_HC +
                         0x80 * TEST_24);   /* (2, 4 and 8 channels: three descriptors, three bcdDevices) */
    for (i = 0; i < l; i += d[i]) {
        const uint8_t *p = d + i;
        assert(p[0] >= 2 && i + p[0] <= l);
        if (p[1] == 4) {                                /* interface */
            if (cur_ac && ac_total)
                assert(ac_sum == ac_total);             /* the AC header's total: its class-specific descriptors */
            cur_if = p[2];
            cur_alt = p[3];
            cur_ac = p[5] == 1 && p[6] == 1;            /* audio control */
            ac_total = 0;
            if (cur_ac) {
                acs++;
                assert(p[8] == (!TEST_SPLIT || cur_if == AUD_IF ? 0 : cur_if == UAC_AS_IF - 1u ? 3 : 4));   /* named */
            }
            if (!seen_if[cur_if & 15u]++)
                nif++;
            if (cur_if == UAC_AS_IF && cur_alt)
                alts++;
#if TEST_OUT
            if (cur_if == UAC_ASO_IF && cur_alt) {
                assert(p[4] == 1u + TEST_FB);           /* the stream, and with TEST_FB its feedback */
                out_alts++;
            }
#endif
        } else if (TEST_SPLIT && p[1] == 0x0B) {        /* IAD: MIDI's, the input's, the output's */
            assert(TEST_SPLIT && p[0] == 8 && p[2] == 2u * iads && p[3] == 2 && p[4] == 1 && p[5] == 1);
            assert(p[7] == (iads == 0 ? 0 : iads == 1 ? 3 : 4));
            iads++;
        } else if (p[1] == 0x24 && cur_ac) {            /* audio control */
            if (p[2] == 1) {
                ac_total = p[5] | p[6] << 8;
                ac_sum = 0;
                if (TEST_SPLIT)                         /* its collection: its one streaming interface */
                    assert(p[7] == 1 && p[8] == cur_if + 1u);
            }
            ac_sum += p[0];
            if (p[2] == 2 && p[3] == 1) {               /* input terminal 1, the line: the input's channels */
                uint32_t cfg = p[8] | p[9] << 8, bits = 0, m;
                for (m = cfg; m; m >>= 1)
                    bits += m & 1u;
                assert(p[7] == TEST_CH && (cfg == 0 || bits == TEST_CH));
                assert(p[10] == (TEST_SPLIT && TEST_NAMES ? 5 : 0));   /* iChannelNames */
                it_ok = 1;
            }
        } else if (TEST_OUT && p[1] == 0x24 && cur_if == UAC_ASO_IF && p[2] == 2) {   /* the output's format */
            assert(p[3] == 1 && p[4] == 2 && (cur_alt == 1 ? (p[5] == 2 && p[6] == 16) : (p[5] == 3 && p[6] == 24)));
            osub = p[5];
        } else if (p[1] == 0x24 && cur_if == UAC_AS_IF && p[2] == 2) {   /* format type I */
            assert(p[3] == 1 && p[4] == TEST_CH);
            sub = p[5];
            assert(cur_alt == 1 ? (p[5] == 2 && p[6] == 16) : (p[5] == 3 && p[6] == 24));
        } else if (p[1] == 5 && p[2] == 0x84) {         /* EP4 IN: a packet of 46 frames at most (44.1 + 1 + 1) */
            uint32_t mp = p[4] | p[5] << 8;
            assert(cur_if == UAC_AS_IF && p[3] == (TEST_HC ? 0x0D : 0x05));   /* asynchronous, or synchronous */
            assert(mp == 46u * TEST_CH * sub && mp <= 1023u);
        } else if (p[1] == 5 && p[2] == 0x04) {         /* EP4 OUT: adaptive, or asynchronous with EP3 IN's feedback */
            assert(TEST_OUT && p[3] == (TEST_FB ? 0x05 : 0x09) && p[8] == (TEST_FB ? 0x83 : 0));
            assert((uint32_t)(p[4] | p[5] << 8) == 49u * 2u * osub);  /* 44.1 + 0.5 % and more: 49 frames */
            out_eps++;
        } else if (TEST_FB && p[1] == 5 && p[2] == 0x83) {   /* EP3 IN: the feedback, 3 bytes, every 2^4 ms */
            assert(p[3] == 0x11 && (p[4] | p[5] << 8) == 3u && p[6] == 1 && p[7] == 4 && p[8] == 0);
            fb_eps++;
        }
    }
    if (cur_ac && ac_total)
        assert(ac_sum == ac_total);
    assert(i == l);
    assert(nif == d[4]);
    assert(acs == (TEST_SPLIT ? 2u + TEST_OUT : 1u) && iads == (TEST_SPLIT ? 2u + TEST_OUT : 0u));
#if TEST_SPLIT
    {   /* misc / IAD; the names */
        static const char *const want[] = {"Optimist In", "Optimist Out", "MASTER L", "MASTER R", "CH1", "CH2", "CH3", "CH4"};
        const uint8_t *sd;
        uint16_t sl;
        uint32_t k;
        assert(dv[2] == 0x00 && dv[3] == 0x02 && dv[4] == 0xEF && dv[5] == 0x02 && dv[6] == 0x01);
        assert((dv[12] & 0x08u) == 0x08u);
        for (k = 3; k < 16u; k++) {
            int has = get_desc(0x0300 | k, &sd, &sl);
            const char *w = !TEST_NAMES ? (TEST_NAMEFN ? (k == 3 ? "FM1-1A2B In" : k == 4 && TEST_OUT ? "FM1-1A2B Out" : 0)
                                                       : (k == 3 ? "Felucca In" : k == 4 && TEST_OUT ? "Felucca Out" : 0))
                          : k == 3 ? want[0] : k == 4 ? (TEST_OUT ? want[1] : 0)
                          : k - 5u < TEST_CH ? want[(TEST_CH == 2 ? 2u : 4u) + k - 5u] : 0;
            uint32_t j;
            assert(has == (w != 0));
            if (!w)
                continue;
            assert(sd[1] == 3 && sl == sd[0] && sl == 2u + 2u * strlen(w));
            for (j = 0; w[j]; j++)
                assert(sd[2 + 2 * j] == (uint8_t)w[j] && sd[3 + 2 * j] == 0);
        }
    }
#endif
#if TEST_NAMEFN
    {   /* the product string (the MIDI device and its ports) is the function's, whole */
        const uint8_t *sd;
        uint16_t sl;
        uint32_t j;
        assert(get_desc(0x0302, &sd, &sl) && sd[1] == 3 && sl == sd[0] && sl == 2u + 2u * 8u);
        for (j = 0; j < 8u; j++)
            assert(sd[2 + 2 * j] == (uint8_t)"FM1-1A2B"[j] && sd[3 + 2 * j] == 0);
    }
#endif
    {   /* the serial number: string SERIAL_STR with FELUCCA_USB_SERIAL_FN, none (iSerialNumber 0) without */
        const uint8_t *sd;
        uint16_t sl;
        uint32_t j;
#if TEST_SERIAL
        assert(dv[16] == SERIAL_STR);
        assert(get_desc(0x0300 | SERIAL_STR, &sd, &sl) && sd[1] == 3 && sl == sd[0] && sl == 2u + 2u * 16u);
        for (j = 0; j < 16u; j++)
            assert(sd[2 + 2 * j] == (uint8_t)"0123456789ABCDEF"[j] && sd[3 + 2 * j] == 0);
#else
        (void)sd; (void)sl; (void)j;
        assert(dv[16] == 0);
#endif
    }
    assert(it_ok && alts == 1u + TEST_24);
    assert(out_eps == TEST_OUT * (1u + TEST_O24) && fb_eps == TEST_FB * (1u + TEST_O24));
    assert(out_alts == TEST_OUT * (1u + TEST_O24));
#if TEST_GOLDEN
    {   /* on the host's clock, the same but for EP4 IN's synchronization (0x05 -> 0x0D) and bcdDevice (+2.00); with
         * the feedback, the output's alt 1 two endpoints, EP4 OUT asynchronous (0x09 -> 0x05) with EP3 IN as its
         * bSynchAddress, EP3 IN's 9 bytes after its CS endpoint, and bcdDevice +0.80 */
        uint8_t g[sizeof GOLD_CFG + 9u + 52u], gd[18];
        uint32_t k, eps = 0, gl = sizeof GOLD_CFG;
        memcpy(g, GOLD_CFG, sizeof GOLD_CFG);
        memcpy(gd, GOLD_DEV, 18);
        for (k = 0; TEST_FB && k < gl; k += g[k]) {
            if (g[k + 1] == 4 && g[k + 2] == UAC_ASO_IF && g[k + 3] == 1)
                g[k + 4] = 2;
            if (g[k + 1] == 5 && g[k + 2] == 0x04) {
                static const uint8_t FB[9] = {9, 5, 0x83, 0x11, 3, 0, 1, 4, 0};
                uint32_t at = k + g[k] + g[k + g[k]];  /* after its CS endpoint */
                g[k + 3] = 0x05;
                g[k + 8] = 0x83;
                memmove(g + at + 9, g + at, gl - at);
                memcpy(g + at, FB, 9);
                gl += 9;
                g[2] = (uint8_t)(gl & 0xFF);
                g[3] = (uint8_t)(gl >> 8);
                gd[12] = (uint8_t)(gd[12] + 0x04);
            }
        }
        if (TEST_O24) {                                 /* the output's alt 2 last: 24-bit, its feedback too */
            static const uint8_t A2[] = {9, 4, 3, 2, 1 + TEST_FB, 1, 2, 0, 0, 7, 36, 1, 3, 1, 1, 0,
                                         11, 36, 2, 1, 2, 3, 24, 1, 0x44, 0xAC, 0,
                                         9, 5, 4, TEST_FB ? 5 : 9, 294 & 0xFF, 294 >> 8, 1, 0, TEST_FB ? 0x83 : 0,
                                         7, 37, 1, 0, 0, 0, 0, 9, 5, 0x83, 0x11, 3, 0, 1, 4, 0};
            const uint32_t a2 = sizeof A2 - (TEST_FB ? 0u : 9u);
            memcpy(g + gl, A2, a2);
            gl += a2;
            g[2] = (uint8_t)(gl & 0xFF);
            g[3] = (uint8_t)(gl >> 8);
            gd[13] = (uint8_t)(gd[13] + 8);
        }
        for (k = 0; TEST_HC && k < gl; k += g[k])
            if (g[k + 1] == 5 && g[k + 2] == 0x84) {
                g[k + 3] = 0x0D;
                eps++;
            }
        if (TEST_HC)
            gd[13] = (uint8_t)(gd[13] + 0x40);
        assert(eps == (TEST_HC ? 1u : 0u));
        assert(l == gl && !memcmp(d, g, l));
        if (TEST_SERIAL)
            gd[16] = SERIAL_STR;                         /* (iSerialNumber: the serial string) */
        assert(!memcmp(dv, gd, 18));
    }
#endif
#if TEST_SPLIT && TEST_CH == 2 && !TEST_24 && TEST_OUT && !TEST_FB && !TEST_O24 && !TEST_HC
    {   /* the two functions, written out: the default's options (2 channels, 16-bit, the output adaptive) */
        static const uint8_t G[] = {
            9, 2, 51, 1, 6, 1, 0, 0x80, 50,                             /* configuration: 307 bytes, 6 interfaces */
            8, 0x0B, 0, 2, 1, 1, 0, 0,                                  /* IAD: MIDI, IF 0-1 */
            9, 4, 0, 0, 0, 1, 1, 0, 0, 9, 0x24, 1, 0, 1, 9, 0, 1, 1,     /* AC 0: its collection MIDI's IF 1 alone */
            9, 4, 1, 0, 2, 1, 3, 0, 0, 7, 0x24, 1, 0, 1, 0x41, 0,       /* MIDI streaming (as before) */
            6, 0x24, 2, 1, 1, 0, 6, 0x24, 2, 2, 2, 0, 9, 0x24, 3, 1, 3, 1, 2, 1, 0, 9, 0x24, 3, 2, 4, 1, 1, 1, 0,
            9, 5, 0x01, 2, 64, 0, 0, 0, 0, 5, 0x25, 1, 1, 1, 9, 5, 0x81, 2, 64, 0, 0, 0, 0, 5, 0x25, 1, 1, 3,
            8, 0x0B, 2, 2, 1, 1, 0, 3,                                  /* IAD: the input, IF 2-3, string 3 */
            9, 4, 2, 0, 0, 1, 1, 0, 3, 9, 0x24, 1, 0, 1, 30, 0, 1, 3,    /* AC 2, named; its collection IF 3 */
            12, 0x24, 2, 1, 0x03, 0x06, 0, 2, 3, 0, TEST_NAMES ? 5 : 0, 0,   /* IT 1: line, L R (names from 5) */
            9, 0x24, 3, 2, 1, 1, 0, 1, 0,                               /* OT 2: USB streaming */
            9, 4, 3, 0, 0, 1, 2, 0, 0, 9, 4, 3, 1, 1, 1, 2, 0, 0, 7, 0x24, 1, 2, 1, 1, 0,
            11, 0x24, 2, 1, 2, 2, 16, 1, 0x44, 0xAC, 0, 9, 5, 0x84, 5, 184, 0, 1, 0, 0, 7, 0x25, 1, 1, 0, 0, 0,
            8, 0x0B, 4, 2, 1, 1, 0, 4,                                  /* IAD: the output, IF 4-5, string 4 */
            9, 4, 4, 0, 0, 1, 1, 0, 4, 9, 0x24, 1, 0, 1, 30, 0, 1, 5,    /* AC 4, named; its collection IF 5 */
            12, 0x24, 2, 3, 1, 1, 0, 2, 3, 0, 0, 0, 9, 0x24, 3, 4, 1, 3, 0, 3, 0,   /* IT 3 USB streaming, OT 4 speaker */
            9, 4, 5, 0, 0, 1, 2, 0, 0, 9, 4, 5, 1, 1, 1, 2, 0, 0, 7, 0x24, 1, 3, 1, 1, 0,
            11, 0x24, 2, 1, 2, 2, 16, 1, 0x44, 0xAC, 0, 9, 5, 0x04, 9, 196, 0, 1, 0, 0, 7, 0x25, 1, 0, 0, 0, 0,
        };
        static const uint8_t GD[18] = {18, 1, 0x00, 0x02, 0xEF, 0x02, 0x01, 64, 0x09, 0x12, 0x01, 0x00,
                                       0x10 + 0x40 + 0x08 + 0x02, 0x03, 1, 2, 0, 1};
        assert(l == sizeof G && !memcmp(d, G, l));
        assert(!memcmp(dv, GD, 18));
    }
#endif
#ifdef TEST_DUMP
    for (i = 0; i < l; i++)
        printf("%u,%s", d[i], i % 16u == 15u ? "\n" : " ");
    printf("\n");
#endif
    bcd_line();
    printf("USB audio (%u ch%s%s): descriptors, %u bytes: OK\n", TEST_CH, TEST_24 ? ", 24-bit" : "",
           TEST_OUT ? ", output" : "", l);
}

/* ------------------------------------------------------ the stream --- */
/* the render's sample for frame f, channel c: a count the packet keeps exactly in 16 bits (<< 8) or 24 */
static int32_t smp(uint32_t f, uint32_t c, uint32_t wide)
{
    uint32_t k = f * UA_CH + c;
    return wide ? (int32_t)(k & 0x7FFFFFu) : (int32_t)((k & 0x7FFFu) << 8);
}

static void test_stream(uint32_t alt, int silent)
{
    static int32_t blk[HALF_FRAMES * 8u];
    uint32_t ms, f = 0, acc = 0, got = 0, next = 0, started = 0, pkts = 0, ok = 1, wide = alt == 2u;
    uint32_t under = uac.underruns, over = uac.overruns, up = uac.adj_up, down = uac.adj_down;
    usb_start();
    setup(0x00, 9, 1, 0);                               /* SET_CONFIGURATION 1 */
    setup(0x01, 11, (uint16_t)alt, UAC_AS_IF);          /* SET_INTERFACE: the alt */
    assert(uac.alt == alt);
    for (ms = 0; ms < 3000u; ms++) {
        uint32_t i, c;
        if (regs[4][17] & 1u) {                         /* the host's IN token: takes the queued packet */
            uint32_t fb = frame_bytes(alt), n;
            assert(in_len % fb == 0);
            n = in_len / fb;
            assert(n >= 43u && n <= 46u);
            for (i = 0; i < n; i++)
                for (c = 0; c < UA_CH; c++) {
                    const uint8_t *b = in_pkt + i * fb + c * (wide ? 3u : 2u);
                    int32_t v = wide ? (int32_t)((uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16)
                                     : (int32_t)(int16_t)(b[0] | b[1] << 8);
                    if (silent) {
                        ok &= v == 0;
                        continue;
                    }
                    if (!started && v == 0)
                        continue;                       /* the priming's silence (and the count's own 0) */
                    if (!started) {
                        started = 1;                    /* the first rendered sample: frame 0 channel 1 or later */
                        next = (uint32_t)v;
                    }
                    ok &= (uint32_t)v == (wide ? next & 0x7FFFFFu : next & 0x7FFFu);
                    next++;
                    got++;
                }
            regs[4][17] = 0;
            pkts++;
        }
        usb_poll();
        usb_poll();
        for (acc += 441176u; acc >= HALF_FRAMES * 10000u; acc -= HALF_FRAMES * 10000u) {   /* the I2S: 44,117.6 Hz */
            uac_render_start();
            if (ms > 50u)
                assert(uac_capturing());
            if (uac_capturing()) {
                for (i = 0; i < HALF_FRAMES; i++)
                    for (c = 0; c < UA_CH; c++)
                        blk[i * UA_CH + c] = smp(f + i, c, wide);
                uac_tap(silent ? 0 : blk, HALF_FRAMES);
                f += HALF_FRAMES;
            }
        }
    }
    assert(ok);
    assert(pkts > 2900u && uac.underruns == under && uac.overruns == over);
    if (!silent)
        assert(got > 2900u * 44u * UA_CH - 2u * HALF_FRAMES * UA_CH);
    printf("USB audio (%u ch): alt %u%s, %u packets, every frame once: OK (fill %u..%u, +%u -%u)\n", TEST_CH, alt,
           silent ? " silence" : "", pkts, uac.fill_lo, uac.fill_hi, uac.adj_up - up, uac.adj_down - down);
    setup(0x01, 11, 0, UAC_AS_IF);
    render_start_idle();
}

static void render_start_idle(void)
{
    uac_render_start();
    assert(!uac_capturing());                           /* alt 0: nobody records, the stems may rest */
}

#if TEST_HC || TEST_OUT
/* a sine's least-squares fit over a window (the sine, its amplitude's and phase's slopes, DC): what is left of it */
#define HC_WIN 2048u             /* the fit's window: 46 ms */
#define HC_R 5                   /* its terms: the sine, its amplitude's and phase's slope over the window, DC */
static double hc_fit(const double *y, uint32_t n, double w, double *maxe)   /* residual / fundamental, power */
{
    double m[HC_R][HC_R + 1] = {{0}}, a[HC_R], res = 0, fund = 0;
    uint32_t i, r, c, k;
    for (i = 0; i < n; i++) {
        double u = (double)i / n - 0.5, v[HC_R] = {sin(w * i), cos(w * i), u * sin(w * i), u * cos(w * i), 1.0};
        for (r = 0; r < HC_R; r++) {
            for (c = 0; c < HC_R; c++)
                m[r][c] += v[r] * v[c];
            m[r][HC_R] += v[r] * y[i];
        }
    }
    for (k = 0; k < HC_R; k++)                          /* Gauss-Jordan, the normal equations */
        for (r = 0; r < HC_R; r++)
            if (r != k) {
                double f = m[r][k] / m[k][k];
                for (c = 0; c <= HC_R; c++)
                    m[r][c] -= f * m[k][c];
            }
    for (k = 0; k < HC_R; k++)
        a[k] = m[k][HC_R] / m[k][k];
    for (i = 0; i < n; i++) {
        double u = (double)i / n - 0.5, s = (a[0] + a[2] * u) * sin(w * i) + (a[1] + a[3] * u) * cos(w * i);
        double e = y[i] - s - a[4];
        res += e * e;
        fund += s * s;
        if (fabs(e) > *maxe)
            *maxe = fabs(e);
    }
    return res / fund;
}

#endif

#if TEST_HC
/* ------------------------------------------------- the host's clock --- */
/* The FM-1's crystal off by ppm0 + drift x minutes (its error, its warming): the I2S and the polls with it; the
 * render a sine of hz in true time (channel c a phase c later), half of full scale; the host a SOF a millisecond
 * (test_hc). Checks that every packet is the 44.1 pattern (from the stream's first: 44 x 9, then 45), the ring
 * never runs dry or over, and the servo settles by 40 s: from then on its low-passed fill within HC_E1 frames of
 * UA_PRIME (the slow loop's band) and its step within HC_WANDER of the clocks' true ratio. After the settling, the
 * output against the sine at the host's clock: a least-squares fit over each 46 ms window (the sine, its
 * amplitude's and phase's slopes: a slow change of the latency does not count, as in a THD+N meter's tracking
 * notch), its residual the THD+N, and the largest residual sample. Returns the worst window's THD+N (dB). */
typedef struct {
    double ppm0, drift;          /* ppm, ppm a minute */
    uint32_t minutes, hz, alt;
    uint32_t keep;               /* 1: the trim the case before learned (a new stream, the same crystal) */
} hc_case;

#define HC_WANDER 40.0           /* ppm: the step's bound after the settling */
static struct {                 /* the measurement */
    double y[8][HC_WIN], sum, worst, maxe;
    uint32_t iw, nwin, started;
} hc_m;

static void hc_host_take(const hc_case *t, uint32_t *pacc, uint32_t settled_ms, uint32_t now_ms)
{
    const uint32_t wide = t->alt == 2u, fb = frame_bytes(t->alt);
    uint32_t n = in_len / fb, want = 44u, i, c;
    assert(in_len % fb == 0);
    *pacc += 100u;                                      /* the 44.1 pattern, counted from the stream's start */
    if (*pacc >= 1000u) {
        *pacc -= 1000u;
        want = 45u;
    }
    assert(n == want);
    for (i = 0; i < n; i++) {
        int nz = 0;
        for (c = 0; c < UA_CH; c++) {
            const uint8_t *b = in_pkt + i * fb + c * (wide ? 3u : 2u);
            int32_t v = wide ? (int32_t)((uint32_t)b[0] << 8 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 24) >> 8
                             : (int32_t)(int16_t)(b[0] | b[1] << 8) * 256;
            hc_m.y[c][hc_m.iw] = (double)v / 8388608.0;
            nz |= v != 0;
        }
        if (now_ms < settled_ms || !(hc_m.started || nz))
            continue;
        hc_m.started = 1;
        if (++hc_m.iw == HC_WIN) {                      /* a window: fit each channel */
            for (c = 0; c < UA_CH; c++) {
                double r = hc_fit(hc_m.y[c], HC_WIN, 2.0 * M_PI * t->hz / 44100.0, &hc_m.maxe);
                hc_m.sum += r;
                hc_m.worst = r > hc_m.worst ? r : hc_m.worst;
            }
            hc_m.nwin++;
            hc_m.iw = 0;
        }
    }
    regs[4][17] = 0;
}

/* Real time in ms. The host: a SOF every 1 ms, taking the queued packet. The FM-1's crystal, off by ppm(t): its
 * I2S (44,117.647 Hz nominal: 24 MHz / 544) and its TIMER5 (the polls, 2 kHz) together, so the polls slide against
 * the SOFs, as on the device. */
static double test_hc(hc_case t)
{
    static int32_t blk[HALF_FRAMES * 8u];
    const double nominal = 44117.647, amp = 0.5 * 8388608.0, total = t.minutes * 60000.0, settle_by = 40000.0;
    double now = 0.0, sof = 0.0, poll = 0.137, ren = 0.311, tk = 0.0, k = 1.0, smin = 1e9, smax = -1e9;
    uint32_t pacc = 0, pkts = 0, settled = 0, i, c;
    uint32_t under = uac.underruns, over = uac.overruns, lo = 0xFFFFFFFFu, hi = 0;
    memset(&hc_m, 0, sizeof hc_m);
    usb_start();
    setup(0x00, 9, 1, 0);
    setup(0x01, 11, (uint16_t)t.alt, UAC_AS_IF);
    if (!t.keep)
        hc.learned = 0;                                 /* from a start: the trim not learned yet */
    while (now < total) {
        now = sof < poll ? (sof < ren ? sof : ren) : (poll < ren ? poll : ren);
        k = 1.0 + (t.ppm0 + t.drift * now / 60000.0) * 1e-6;   /* the crystal's speed */
        if (now == sof) {                               /* the host's IN token takes the queued packet */
            if (regs[4][17] & 1u) {
                hc_host_take(&t, &pacc, (uint32_t)settle_by, (uint32_t)now);
                pkts++;
            }
            sof += 1.0;
        } else if (now == poll) {
            usb_poll();
            poll += 0.5 / k;
        } else {                                        /* a render: its start, then its block */
            uac_render_start();
            assert(now < 50.0 || uac_capturing());
            ren += HALF_FRAMES * 1000.0 / (nominal * k);
            if (!uac_capturing())
                continue;
            if (now > 100.0) {
                uint32_t f = ua_w - ua_r;               /* the fill as this render starts */
                double s = ((double)hc.step / HC_ONE / (nominal * k / 44100.0) - 1.0) * 1e6;
                lo = f < lo ? f : lo;
                hi = f > hi ? f : hi;
                if (abs(hc.lp2 - (int32_t)(UA_PRIME << 8)) > HC_E1 * 256 || fabs(s) >= HC_WANDER)
                    settled = (uint32_t)now;            /* the last render still out of the bands */
                if (now >= settle_by) {
                    smin = s < smin ? s : smin;
                    smax = s > smax ? s : smax;
                }
            }
            for (i = 0; i < HALF_FRAMES; i++, tk += 1.0 / (nominal * k))
                for (c = 0; c < UA_CH; c++)
                    blk[i * UA_CH + c] = (int32_t)lrint(amp * sin(2.0 * M_PI * t.hz * tk + c));
            uac_tap(blk, HALF_FRAMES);
        }
    }
    printf("USB audio on the host's clock (%u ch, alt %u): I2S %+.0f ppm %+.0f ppm/min, %u min, %u Hz: %u packets "
           "of the 44.1 pattern, fill %u..%u, settled in %.1f s, then the step %+.1f..%+.1f ppm of the clocks' "
           "ratio; THD+N %.1f dB (worst window %.1f), max error %.1f dBFS: ", TEST_CH, t.alt, t.ppm0, t.drift,
           t.minutes, t.hz, pkts, lo, hi, settled / 1000.0, smin, smax, 10.0 * log10(hc_m.sum / (hc_m.nwin * UA_CH)),
           10.0 * log10(hc_m.worst), 20.0 * log10(hc_m.maxe));
    assert(uac.underruns == under && uac.overruns == over);
    assert(pkts > total - 10u && hc_m.nwin > 0);
    assert(settled < (t.keep ? 1000.0 : settle_by));    /* settled before the measurement; at once with a trim */
    printf("OK\n");
    setup(0x01, 11, 0, UAC_AS_IF);
    render_start_idle();
    return 10.0 * log10(hc_m.worst);
}

static void test_hc_all(void)
{
    static const hc_case cases[] = {
        {0, 0, 3, 1000, 1}, {100, 0, 3, 1000, 1}, {-100, 0, 3, 1000, 1}, {500, 0, 3, 1000, 1}, {-500, 0, 3, 1000, 1},
        {-200, 40, 10, 1000, 1},               /* a drift: -200 to +200 ppm over 10 minutes */
        {200, 0, 1, 1000, 1, 1},               /* the stream again: the trim learned (+200 ppm) */
        {500, 0, 1, 5000, 1}, {-500, 0, 1, 10000, 1},
#if TEST_24
        {0, 0, 1, 1000, 2}, {500, 0, 1, 1000, 2}, {-500, 0, 1, 5000, 2}, {500, 0, 1, 10000, 2},
#endif
    };
    uint32_t k;
    for (k = 0; k < sizeof cases / sizeof cases[0]; k++) {
        double d = test_hc(cases[k]);
        /* the bounds, a few dB under what was measured: 16-bit, its quantization; 24-bit, the filter's */
        double bound = cases[k].hz <= 1000u ? (cases[k].alt == 2u ? -95.0 : -86.0)
                                            : cases[k].hz <= 5000u ? -80.0 : -74.0;
        assert(d < bound);
    }
}
#endif

#if TEST_OUT
/* ---------------------------------------------------------- playback --- */
static uint32_t host_alt = 1;                           /* the output's alt the host streams (2: 24-bit, TEST_O24) */
static int32_t sext24(uint32_t v) { return (int32_t)(v << 8) >> 8; }
/* the ramp's frame t as the render gets it (24-bit): alt 1 16 bits << 8; alt 2 a low byte of its own too */
static int32_t ramp_l(uint32_t t)
{
    return host_alt == 2u ? sext24(((t * 7u) << 8 | ((t * 13u) & 0xFFu)) & 0xFFFFFFu) : (int16_t)(uint16_t)(t * 7u) * 256;
}
static int32_t ramp_r(uint32_t t)
{
    return host_alt == 2u ? sext24(((0x8000u - t * 3u) << 8 | ((t * 5u) & 0xFFu)) & 0xFFFFFFu)
                          : (int16_t)(uint16_t)(0x8000u - t * 3u) * 256;
}
static void host_put(uint32_t i, int32_t l, int32_t r)  /* frame i of the packet (24-bit values) in the alt's format */
{
    if (host_alt == 2u) {
        uint8_t *d = ep4rx_buf + 6u * i;
        d[0] = (uint8_t)l, d[1] = (uint8_t)(l >> 8), d[2] = (uint8_t)(l >> 16);
        d[3] = (uint8_t)r, d[4] = (uint8_t)(r >> 8), d[5] = (uint8_t)(r >> 16);
    } else {
        int16_t a = (int16_t)(l >> 8), b = (int16_t)(r >> 8);
        memcpy(ep4rx_buf + 4u * i, &a, 2);
        memcpy(ep4rx_buf + 4u * i + 2u, &b, 2);
    }
}
static void host_send(uint32_t n)
{
    uint32_t bytes = n * (host_alt == 2u ? 6u : 4u);
    regs[4][22] = (uint8_t)bytes;
    regs[4][23] = (uint8_t)(bytes >> 8);
    regs[4][20] = 1;                                    /* RxPktRdy */
}
static void host_out(uint32_t n, uint32_t *t)           /* an OUT packet of n stereo frames, a ramp */
{
    uint32_t i;
    for (i = 0; i < n; i++, (*t)++)
        host_put(i, ramp_l(*t), ramp_r(*t));
    host_send(n);
}

/* the host's side of the output: one OUT packet a millisecond. Adaptive: the 44.1 pattern (44, 44 ... 45). With the
 * feedback: the frames the last feedback it read asks for, the fraction carried (as a host's audio engine does), the
 * feedback read every 2^4 ms (its bRefresh), the nominal 44.1 until the first */
static struct {
    uint32_t acc, fb, reads;
    double fb_sum;
    uint32_t fb_n;
} hp;
static void host_play_start(void)
{
    hp.acc = 0;
    hp.fb = 44100u * 16384u / 1000u;
    hp.reads = 0;
    hp.fb_sum = 0;
    hp.fb_n = 0;
}
static uint32_t host_play_n(uint32_t ms, int measure)
{
    uint32_t n;
#if TEST_FB
    if (!(ms & 15u) && (regs[3][17] & 1u)) {            /* the host's IN token on EP3: the feedback */
        assert(fb_len == 3u);
        hp.fb = fb_pkt[0] | (uint32_t)fb_pkt[1] << 8 | (uint32_t)fb_pkt[2] << 16;
        assert(hp.fb >= UO_FB_NOM - UO_FB_MAX && hp.fb <= UO_FB_NOM + UO_FB_MAX);
        regs[3][17] = 0;
        hp.reads++;
        if (measure) {
            hp.fb_sum += hp.fb;
            hp.fb_n++;
        }
    }
    hp.acc += hp.fb;
    n = hp.acc >> 14;
    hp.acc &= 0x3FFFu;
#else
    (void)ms, (void)measure;
    hp.acc += 441u;
    n = hp.acc / 10u;
    hp.acc %= 10u;
#endif
    return n;
}

static void test_play_take(void)
{
    static int32_t a[2u * HALF_FRAMES], b[2u * HALF_FRAMES];
    uint32_t ms, t = 0, acc = 0, blocks = 0, with = 0, i, under = uo.underruns;
    usb_start();
    setup(0x00, 9, 1, 0);
    for (i = 0; i < 2u * HALF_FRAMES; i++)
        b[i] = 12345;
    assert(uac_play_take(b, HALF_FRAMES) == 0);         /* nothing streams: silence */
    for (i = 0; i < 2u * HALF_FRAMES; i++)
        assert(b[i] == 0);
    setup(0x01, 11, 1, UAC_ASO_IF);
    host_play_start();
    for (ms = 0; ms < 2000u; ms++) {
        host_out(host_play_n(ms, 0), &t);
        usb_poll();
        usb_poll();
        for (acc += 441176u; acc >= HALF_FRAMES * 10000u; acc -= HALF_FRAMES * 10000u) {
            /* the same render twice from the same state: uac_play into silence at 4096, then uac_play_take */
            __typeof__(uo) s = uo;
            uint32_t r = uo_r, w = uo_w;
            int got;
            memset(a, 0, sizeof a);
            uac_play(a, HALF_FRAMES, 4096u);
            uo = s;
            uo_r = r;
            uo_w = w;
            for (i = 0; i < 2u * HALF_FRAMES; i++)
                b[i] = -1;
            got = uac_play_take(b, HALF_FRAMES);
            assert(!memcmp(a, b, sizeof a));
            for (i = 0; i < 2u * HALF_FRAMES && !got; i++)
                assert(b[i] == 0);
            with += (uint32_t)got;
            blocks++;
        }
    }
    assert(with > blocks - 10u && uo.underruns == under);
    printf("USB audio output: uac_play_take = uac_play into silence at unity, %u of %u blocks: OK\n", with, blocks);
    setup(0x01, 11, 0, UAC_ASO_IF);
    assert(uac_play_take(b, HALF_FRAMES) == 0);
}

/* The output against the FM-1's crystal off by ppm0 + drift x minutes (real time in ms; the I2S and the polls with
 * it, as test_hc): the host one OUT packet a millisecond (host_play_n). sine 0: a ramp (L t x 7, R 0x8000 - t x 3), and
 * with the feedback every frame played must be the next one sent, bit for bit (adaptive: resampled, not checked so).
 * sine hz: a sine at half of full scale in the host's samples, the output fitted over 46 ms windows after 10 s
 * (hc_fit: its THD+N). Checks no under- or overrun once playing and, with the feedback, the fill within UO_FB_BAND of
 * UO_TGT after 10 s and the feedback's mean the I2S's true rate within 20 ppm. Returns the worst window's THD+N. */
typedef struct {
    double ppm0, drift;
    uint32_t minutes, hz, alt;
} play_case;
#define UO_FB_BAND 48u                                  /* frames: the low-passed fill's distance from its target */

static void host_out_sine(uint32_t n, uint32_t *t, uint32_t hz)   /* n frames of the host's sine, L and R alike */
{
    uint32_t i;
    for (i = 0; i < n; i++, (*t)++) {
        double x = sin(2.0 * M_PI * hz * (double)*t / 44100.0);
        int32_t v = host_alt == 2u ? (int32_t)lrint(4194304.0 * x) : (int32_t)lrint(16384.0 * x) * 256;
        host_put(i, v, v);
    }
    host_send(n);
}

static double test_play_clock(play_case c)
{
    static int32_t b[2u * HALF_FRAMES];
    static double y[HC_WIN];
    const double nominal = 44117.647, total = c.minutes * 60000.0, settle = 10000.0;
    double now = 0.0, sof = 0.0, poll = 0.137, ren = 0.311, k = 1.0, sum = 0, worst = 0, maxe = 0, w;
    uint32_t t = 0, ms = 0, started = 0, next = 0, frames = 0, ok = 1, i, iw = 0, nwin = 0;
    uint32_t lo = 0xFFFFFFFFu, hi = 0, under, over;
    int32_t emax = 0;
    memset(regs, 0, sizeof regs);
    usb_start();
    setup(0x00, 9, 1, 0);
    host_alt = c.alt;
    setup(0x01, 11, (uint16_t)c.alt, UAC_ASO_IF);
    host_play_start();
    under = uo.underruns, over = uo.overruns;
    while (now < total) {
        now = sof < poll ? (sof < ren ? sof : ren) : (poll < ren ? poll : ren);
        k = 1.0 + (c.ppm0 + c.drift * now / 60000.0) * 1e-6;
        if (now == sof) {                               /* the host's frame: an OUT packet */
            uint32_t n = host_play_n(ms++, now >= settle);
            if (c.hz)
                host_out_sine(n, &t, c.hz);
            else
                host_out(n, &t);
            sof += 1.0;
        } else if (now == poll) {
            usb_poll();
            poll += 0.5 / k;
        } else {                                        /* a render takes its block */
            uint32_t f0 = uo_w - uo_r;                  /* (the fill a render starts at) */
            int got = uac_play_take(b, HALF_FRAMES);
            ren += HALF_FRAMES * 1000.0 / (nominal * k);
            if (!got) {
                assert(!started);                       /* (priming only: never once playing) */
                continue;
            }
#if TEST_FB
            if (now >= settle) {
                int32_t e = uo.fill_q8 / 256 - (int32_t)UO_TGT;
                lo = f0 < lo ? f0 : lo;
                hi = f0 > hi ? f0 : hi;
                emax = abs(e) > emax ? abs(e) : emax;
            }
#else
            (void)f0;
#endif
            for (i = 0; i < HALF_FRAMES; i++) {
                int32_t l = b[2u * i], r = b[2u * i + 1u];
                if (!c.hz) {
                    if (!started) {                     /* the first frame played: the host's frame next */
                        uint32_t f;
                        for (f = 0; f < t && ramp_l(f) != l; f++)
                            ;
                        next = f;
                    }
                    if (TEST_FB)
                        ok &= l == ramp_l(next) && r == ramp_r(next);
                    next++;
                } else if (now >= settle) {
                    y[iw] = (double)l / 8388608.0;
                    if (++iw == HC_WIN) {
                        double q;
                        w = 2.0 * M_PI * c.hz / (TEST_FB ? 44100.0 : nominal * k);   /* a frame's phase: the host's
                                                                         * samples as they are, or at the I2S's rate */
                        q = hc_fit(y, HC_WIN, w, &maxe);
                        sum += q;
                        worst = q > worst ? q : worst;
                        nwin++;
                        iw = 0;
                    }
                }
                frames++;
                started = 1;
            }
        }
    }
    printf("USB audio output (%s, alt %u): I2S %+.0f ppm %+.0f ppm/min, %u min, %s %u Hz: %u frames played", TEST_FB ?
           "the feedback" : "adaptive", c.alt, c.ppm0, c.drift, c.minutes, c.hz ? "sine" : "ramp", c.hz, frames);
    if (TEST_FB)
        printf(", the fill at a render's start %u..%u, low-passed within %d of its target %u, feedback %.5f frames a ms "
               "(the I2S %.5f) over %u reads", lo, hi, emax, (unsigned)UO_TGT, hp.fb_sum / hp.fb_n / 16384.0,
               nominal * k / 1000.0, hp.fb_n);
    if (c.hz)
        printf(", THD+N %.1f dB (worst window %.1f)", 10.0 * log10(sum / nwin), 10.0 * log10(worst));
    else if (TEST_FB)
        printf(", every one as sent, in order");
    assert(ok && uo.underruns == under && uo.overruns == over);
#if TEST_FB
    assert(lo >= HALF_FRAMES + HALF_FRAMES / 2u && emax <= (int32_t)UO_FB_BAND);   /* a render and half of one in
                                                         * hand at its start; the fill near its target */
    assert(fabs(hp.fb_sum / hp.fb_n / 16384.0 / (nominal * (1.0 + (c.ppm0 + c.drift * c.minutes / 2.0) * 1e-6) /
                1000.0) - 1.0) < (c.drift ? 1e-4 : 2e-5));
#endif
    printf(": OK\n");
    setup(0x01, 11, 0, UAC_ASO_IF);
    host_alt = 1;
    return c.hz ? 10.0 * log10(worst) : 0.0;
}

static void test_play_clocks(void)
{
    static const play_case cases[] = {
        {0, 0, 1, 0, 1}, {100, 0, 1, 0, 1}, {-100, 0, 1, 0, 1}, {500, 0, 2, 0, 1}, {-500, 0, 2, 0, 1},
        {-200, 40, 10, 0, 1}, {500, 0, 1, 1000, 1}, {-500, 0, 1, 1000, 1}, {500, 0, 1, 10000, 1}, {-500, 0, 1, 10000, 1},
#if TEST_O24
        {0, 0, 1, 0, 2}, {500, 0, 1, 0, 2}, {-500, 0, 1, 0, 2}, {500, 0, 1, 1000, 2}, {-500, 0, 1, 10000, 2},
#endif
    };
    uint32_t k;
    for (k = 0; k < sizeof cases / sizeof cases[0]; k++) {
        double d = test_play_clock(cases[k]);
        if (TEST_FB && cases[k].hz)                     /* the samples as sent: their own quantization */
            assert(d < (cases[k].alt == 2u ? -130.0 : -88.0));
    }
}
#endif

#if TEST_MEM
/* FELUCCA_UAC_MEM: the rings lent by the app (uac_mem). Not lent: the computer streams both ways for a second, the
 * input sends whole silence packets and nobody captures, the output's packets are dropped (not counted as overruns)
 * and nothing plays; nothing is written anywhere (the rings do not exist). Then lent (a heap block, every stream test
 * after this runs on it), and at the end taken back and freed (under ASan any later touch of it fails): the streams
 * go on silent again. Lent and taken back 20 times with a stream each time: every lend streams whole */
static uint8_t *mem_blk;
static void mem_lend(void)
{
    mem_blk = calloc(1, uac_mem_bytes());
    assert(mem_blk && uac_mem_bytes() % 4u == 0u);
    uac_mem(mem_blk);
}
static void mem_back(void)
{
    uac_mem(0);
    free(mem_blk);
    mem_blk = 0;
}
static void test_mem_unlent(const char *when)
{
    static int32_t blk[HALF_FRAMES * 8u];
    uint32_t ms, acc = 0, pkts = 0, i, over = uac.overruns, under = uac.underruns, t = 0;
#if TEST_OUT
    static int32_t lr[2u * HALF_FRAMES];
    uint32_t oover = uo.overruns, oun = uo.underruns;
#endif
    usb_start();
    setup(0x00, 9, 1, 0);
    setup(0x01, 11, 1, UAC_AS_IF);
#if TEST_OUT
    setup(0x01, 11, 1, UAC_ASO_IF);
    host_play_start();
#endif
    for (i = 0; i < HALF_FRAMES * UA_CH; i++)
        blk[i] = 0x123456;
    for (ms = 0; ms < 1000u; ms++) {
        if (regs[4][17] & 1u) {
            for (i = 0; i < in_len; i++)
                assert(in_pkt[i] == 0);                 /* silence */
            regs[4][17] = 0;
            pkts++;
        }
#if TEST_OUT
        host_out(host_play_n(ms, 0), &t);
#endif
        usb_poll();
        usb_poll();
        for (acc += 441176u; acc >= HALF_FRAMES * 10000u; acc -= HALF_FRAMES * 10000u) {
            uac_render_start();
            assert(!uac_capturing());
            uac_tap(blk, HALF_FRAMES);                  /* (given anyway: dropped) */
#if TEST_OUT
            assert(uac_play_take(lr, HALF_FRAMES) == 0);
            for (i = 0; i < 2u * HALF_FRAMES; i++)
                assert(lr[i] == 0);
#endif
        }
    }
    assert(pkts > 900u && uac.overruns == over && uac.underruns == under);
#if TEST_OUT
    assert(uo.overruns == oover && uo.underruns == oun);
    setup(0x01, 11, 0, UAC_ASO_IF);
#endif
    (void)t;
    setup(0x01, 11, 0, UAC_AS_IF);
    render_start_idle();
    printf("USB audio rings not lent (%s): %u silence packets, nothing captured or played, nothing counted: OK\n", when,
           pkts);
}
/* the lend as the device does it: the computer streams first (alt set, reading silence packets), the block arrives a main
 * loop pass later, mid-stream; later taken back mid-stream (its question's NO) and lent again; then the computer stops
 * reading for 30 ms before it sets alt 0 (a recorder closing), and the block goes back. The counters of each phase */
static void test_mem_midstream(void)
{
    static int32_t blk[HALF_FRAMES * 8u];
    uint32_t ms, acc = 0, i, c, f = 0, t = 0, reading = 1, ph = 0;
    uint32_t u0 = uac.underruns, o0 = uac.overruns;
#if TEST_OUT
    static int32_t lr[2u * HALF_FRAMES];
    uint32_t pu0 = uo.underruns, po0 = uo.overruns;
#endif
    static const char *const PH[] = {"unlent, streaming", "lent mid-stream", "taken back mid-stream", "lent again",
                                     "a 12 ms pause both ways, then on (a real gap)", "on again",
                                     "the computer stops reading", "alt 0, taken back"};
    uint32_t cu[8] = {0}, co[8] = {0}, pu[8] = {0}, pu_at = 0;
#if TEST_OUT
    pu_at = pu0;
#endif
    usb_start();
    setup(0x00, 9, 1, 0);
    setup(0x01, 11, 1, UAC_AS_IF);
#if TEST_OUT
    setup(0x01, 11, 1, UAC_ASO_IF);
    host_play_start();
#endif
    for (ms = 0; ms < 2600u; ms++) {
        uint32_t nph = ms < 100u ? 0u : ms < 1100u ? 1u : ms < 1300u ? 2u : ms < 1700u ? 3u : ms < 1800u ? 4u : ms < 2300u ? 5u :
                       ms < 2330u ? 6u : 7u;
        uint32_t paused = ms >= 1700u && ms < 1712u;
        if (nph != ph) {
            cu[ph] = uac.underruns - u0, co[ph] = uac.overruns - o0;
            u0 = uac.underruns, o0 = uac.overruns;
#if TEST_OUT
            pu[ph] = uo.underruns - pu_at, pu_at = uo.underruns;
#endif
            ph = nph;
            if (ph == 1u || ph == 3u)
                mem_lend();
            else if (ph == 2u)
                mem_back();
            else if (ph == 6u)
                reading = 0;
            else if (ph == 7u) {
                setup(0x01, 11, 0, UAC_AS_IF);
#if TEST_OUT
                setup(0x01, 11, 0, UAC_ASO_IF);
#endif
                mem_back();
            }
        }
        if (reading && !paused && (regs[4][17] & 1u))
            regs[4][17] = 0;                            /* (the host takes the packet) */
#if TEST_OUT
        if (ph < 6u && !paused)
            host_out(host_play_n(ms, 0), &t);
#endif
        usb_poll();
        usb_poll();
        for (acc += 441176u; acc >= HALF_FRAMES * 10000u; acc -= HALF_FRAMES * 10000u) {
            uac_render_start();
            if (uac_capturing()) {
                for (i = 0; i < HALF_FRAMES; i++)
                    for (c = 0; c < UA_CH; c++)
                        blk[i * UA_CH + c] = smp(f + i, c, 0);
                uac_tap(blk, HALF_FRAMES);
                f += HALF_FRAMES;
            }
#if TEST_OUT
            (void)uac_play_take(lr, HALF_FRAMES);
#endif
        }
    }
    cu[7] = uac.underruns - u0, co[7] = uac.overruns - o0;
#if TEST_OUT
    pu[7] = uo.underruns - pu_at;
#endif
    for (i = 0; i < 8u; i++)
        printf("USB audio rings, %s: %u underruns, %u overruns; output %u underruns\n", PH[i], cu[i], co[i], pu[i]);
    for (i = 0; i < 8u; i++)                            /* (only the real gap counts) */
        assert(i == 4u ? co[i] > 0u : cu[i] == 0u && co[i] == 0u && pu[i] == 0u);
#if TEST_OUT
    assert(pu[4] > 0u && uo.overruns == po0);
#endif
    (void)t;
    render_start_idle();
    printf("USB audio rings lent and taken back while the computer streams, and as it stops: no under- or overrun counted: OK\n");
}
static void test_mem_cycles(void)
{
    uint32_t k;
    for (k = 0; k < 20u; k++) {
        mem_lend();
        test_stream(1, 0);
        mem_back();
    }
    printf("USB audio rings lent and taken back 20 times, a whole stream each: OK\n");
}
#endif

int main(void)
{
#if TEST_MEM
    test_mem_unlent("before the first lend");
    mem_lend();
#endif
    test_descriptors();
#if TEST_HC
    test_stream(1, 1);                                  /* silence (resampled) is silence */
    test_hc_all();
#else
    test_stream(1, 0);
#if !UA_IN_PLAIN
    test_stream(1, 1);                                  /* (the plain input has no uac_tap(0, n): it is the options') */
#endif
#if TEST_24
    test_stream(2, 0);
#endif
#endif
#if TEST_OUT
    test_play_take();
    test_play_clocks();
#endif
#if TEST_MEM
    mem_back();
    test_mem_unlent("taken back");
#if !TEST_HC
    test_mem_cycles();
    test_mem_midstream();
#endif
#endif
    printf("USB audio: all OK\n");
    return 0;
}
#endif
