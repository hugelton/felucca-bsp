/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio)
 * Copyright (C) 2026 Adrien Tibi (Recursive Field Instruments)
 * Ported from Melodee; source revisions and MPL permission: see NOTICE. */
/* USB MIDI host (chip/usb_host.c, usb_host_desc.h): the production code against a model of the SIE and of
 * what hangs on the cable. The model answers EP0 requests the way a keyboard, a hub and a mass-storage stick do,
 * so this tests the enumeration order, the hub handling, the packet filter and the fault paths. It does not test
 * the controller: the register sequences come from the AC79 SDK's USB0 host code and need a bench. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"
#include "../chip/fm1_usb.h"
#pragma GCC diagnostic pop

/* ------------------------------------------------------- the devices --- */
enum { KBD, HUB, STICK, NDEV };
typedef struct {
    uint8_t addr, cfg, alt, present, in_default;
    uint8_t cls, mps0;
    const uint8_t *cfgd;
    uint16_t cfglen;
    uint8_t ports;                      /* hub: downstream devices by port (1-based), 0 = empty */
    uint8_t port_dev[8], port_pow[8], port_en[8], port_chg[8];
    uint32_t seen_set_cfg, seen_set_if, seen_probe;
} dev_t_;
static dev_t_ dev[NDEV];
static uint8_t kbd_cfg[128], stick_cfg[64], hub_cfg[32];
static uint16_t kbd_cfg_len;
static int root;                        /* the device on the root port, -1 none */
static dev_t_ *dflt;                    /* the device answering at address 0 */

static uint16_t build_kbd_cfg(uint8_t *d, int with_out)
{
    static const uint8_t body[] = {
        9, 4, 0, 0, 0, 1, 1, 0, 0,                      /* IF0 audio control */
        9, 0x24, 1, 0, 1, 9, 0, 1, 1,
        9, 4, 1, 0, 2, 1, 3, 0, 0,                      /* IF1 MIDI streaming, two endpoints */
        7, 0x24, 1, 0, 1, 0x41, 0,
        6, 0x24, 2, 1, 1, 0,
        9, 0x24, 3, 1, 3, 1, 2, 1, 0,
        9, 5, 0x02, 2, 64, 0, 0, 0, 0,                  /* OUT first, as many devices list it */
        5, 0x25, 1, 1, 1,
        9, 5, 0x81, 2, 64, 0, 0, 0, 0,                  /* EP1 IN bulk */
        5, 0x25, 1, 1, 3,
    };
    uint16_t n = 9u + sizeof body;
    static const uint8_t head[9] = {9, 2, 0, 0, 2, 1, 0, 0x80, 50};
    memcpy(d, head, 9);
    memcpy(d + 9, body, sizeof body);
    if (!with_out) {
        memmove(d + 58, d + 72, n - 72u);              /* remove OUT and its class-specific descriptor */
        n -= 14;
        d[31] = 1;
    }
    d[2] = (uint8_t)n;
    d[3] = (uint8_t)(n >> 8);
    return n;
}

static void devices_init(void)
{
    static const uint8_t stick[] = {9, 2, 32, 0, 1, 1, 0, 0x80, 50, 9, 4, 0, 0, 2, 8, 6, 0x50, 0,
                                    7, 5, 0x81, 2, 64, 0, 0, 7, 5, 0x02, 2, 64, 0, 0};
    static const uint8_t hub[] = {9, 2, 25, 0, 1, 1, 0, 0xE0, 50, 9, 4, 0, 0, 1, 9, 0, 0, 0,
                                  7, 5, 0x81, 3, 1, 0, 12};
    memset(dev, 0, sizeof dev);
    kbd_cfg_len = build_kbd_cfg(kbd_cfg, 1);
    memcpy(stick_cfg, stick, sizeof stick);
    memcpy(hub_cfg, hub, sizeof hub);
    dev[KBD] = (dev_t_){.present = 1, .cls = 0, .mps0 = 64, .cfgd = kbd_cfg, .cfglen = kbd_cfg_len};
    dev[HUB] = (dev_t_){.present = 1, .cls = 9, .mps0 = 64, .cfgd = hub_cfg, .cfglen = 25, .ports = 4};
    dev[STICK] = (dev_t_){.present = 1, .cls = 0, .mps0 = 8, .cfgd = stick_cfg, .cfglen = 32};
    root = -1;
    dflt = 0;
}

/* --------------------------------------------------- the SIE and bus --- */
static uint8_t regs[4][32], common[16], index_reg, rsp[512];
static uint32_t rsp_len, rsp_pos, ep_cnt[5];
static void *ep_radr[5];
static void *ep_tadr[5];
static uint32_t USB_CON0, line_state;
static uint8_t bus_dead;                /* nothing answers (cable pulled) */
static uint8_t ep0_timeout;             /* SETUP stays busy until the driver's deadline */
static uint8_t devctl_ls;               /* the device is low speed */
static uint32_t ctl_requests, setup_requests, rx_toggle_clears;
static uint32_t tx_toggle_clears, off_count;
static uint32_t root_resets, host_attaches, port_resets[8];
static uint32_t wire[512], wire_n;
static dev_t_ *pend_dev;
static uint8_t pend_addr;
static uint8_t rx_q[8][64];
static uint8_t rx_n[8];
static uint32_t rx_head, rx_tail;

static dev_t_ *by_addr(uint8_t a)
{
    int i;
    if (a == 0)
        return dflt;
    for (i = 0; i < NDEV; i++)
        if (dev[i].present && dev[i].addr == a && !dev[i].in_default)
            return &dev[i];
    return 0;
}

static void bus_reset(void)
{
    int i;
    root_resets++;
    for (i = 0; i < NDEV; i++) {
        dev[i].addr = 0;
        dev[i].cfg = 0;
        memset(dev[i].port_pow, 0, sizeof dev[i].port_pow);
        memset(dev[i].port_en, 0, sizeof dev[i].port_en);
        memset(dev[i].port_chg, 0, sizeof dev[i].port_chg);
    }
    dflt = root >= 0 ? &dev[root] : 0;
}

static int hub_port_dev(dev_t_ *h, uint32_t port)
{
    return port >= 1 && port <= h->ports ? (int)h->port_dev[port] - 1 : -1;
}

/* a SETUP packet to `d`: fill rsp, return 1 if answered, 0 if stalled */
static int answer(dev_t_ *d, const uint8_t *s)
{
    uint8_t rt = s[0], rq = s[1];
    uint16_t val = (uint16_t)(s[2] | s[3] << 8), idx = (uint16_t)(s[4] | s[5] << 8), len = (uint16_t)(s[6] | s[7] << 8);
    uint32_t n = 0;
    rsp_len = rsp_pos = 0;
    ctl_requests++;
    if (rt == 0x80 && rq == 6 && (val >> 8) == 1) {
        d->seen_probe++;
        uint8_t dd[18] = {18, 1, 0x00, 0x02, d->cls, 0, 0, d->mps0, 0x09, 0x12, 1, 0, 0, 1, 1, 2, 0, 1};
        memcpy(rsp, dd, 18);
        n = 18;
    } else if (rt == 0x80 && rq == 6 && (val >> 8) == 2) {
        memcpy(rsp, d->cfgd, d->cfglen);
        n = d->cfglen;
    } else if (rt == 0xA0 && rq == 6 && val == 0x2900 && d->ports) {
        uint8_t hd[9] = {9, 0x29, d->ports, 0, 0, 10, 0, 0, 0xFF};
        memcpy(rsp, hd, 9);
        n = 9;
    } else if (rt == 0x00 && rq == 5) {
        pend_dev = d;                                   /* takes effect after the status stage */
        pend_addr = (uint8_t)val;
    } else if (rt == 0x00 && rq == 9) {
        d->cfg = (uint8_t)val;
        d->seen_set_cfg++;
    } else if (rt == 0x01 && rq == 11) {
        d->alt = (uint8_t)val;
        d->seen_set_if++;
    } else if (rt == 0x23 && rq == 3 && d->ports) {
        int c = hub_port_dev(d, idx);
        if (val == 8)
            d->port_pow[idx] = 1;
        if (val == 4 && d->port_pow[idx]) {
            port_resets[idx]++;
            d->port_en[idx] = 1;
            d->port_chg[idx] |= 0x10;
            if (c >= 0) {
                dev[c].addr = 0;
                dev[c].in_default = 1;
                dflt = &dev[c];
            }
        }
    } else if (rt == 0x23 && rq == 1 && d->ports) {
        if (val == 16)
            d->port_chg[idx] &= (uint8_t)~1u;
        if (val == 20)
            d->port_chg[idx] &= (uint8_t)~0x10u;
    } else if (rt == 0xA3 && rq == 0 && d->ports) {
        int c = hub_port_dev(d, idx);
        uint32_t st = (c >= 0 && d->port_pow[idx] ? 1u : 0u) | (d->port_en[idx] ? 2u : 0u) | 0x100u;
        rsp[0] = (uint8_t)st;
        rsp[1] = (uint8_t)(st >> 8);
        rsp[2] = d->port_chg[idx];
        rsp[3] = 0;
        n = 4;
    } else {
        return 0;
    }
    rsp_len = n < len ? n : len;
    return 1;
}

static void deliver_rx(void)            /* an IN request is pending on EP1 and the keyboard has something */
{
    dev_t_ *d;
    if (regs[1][20] & 1u || !(regs[1][20] & 0x20u))
        return;
    if (bus_dead) {
        regs[1][20] = 0x04;
        return;
    }
    d = by_addr(common[0]);
    if (!d || d != &dev[KBD] || rx_head == rx_tail)
        return;
    memcpy(ep_radr[1], rx_q[rx_tail % 8], rx_n[rx_tail % 8]);
    regs[1][22] = rx_n[rx_tail % 8];
    regs[1][23] = 0;
    rx_tail++;
    regs[1][20] = 1;
}

static void mock_write(uint32_t r, uint32_t v)
{
    if (r == 14) {
        index_reg = (uint8_t)v;
    } else if (r == 1 && (v & 8u)) {                    /* POWER: bus reset */
        bus_reset();
        common[r] = (uint8_t)v;
    } else if (r < 16) {
        common[r] = (uint8_t)v;
    } else if (index_reg == 0 && r == 17) {             /* EP0 CSR, host */
        dev_t_ *d = by_addr(common[0]);
        if (v == 0x0A) {
            setup_requests++;
            uint8_t s[8];
            memcpy(s, ep_radr[0], 8);
            if (ep0_timeout)
                regs[0][17] = 2;
            else if (bus_dead || !d || !answer(d, s))
                regs[0][17] = 0x10;                     /* no answer */
            else
                regs[0][17] = 0;
        } else if (v == 0x20) {
            uint32_t mps = d && d->mps0 ? d->mps0 : 8u, n = rsp_len - rsp_pos;
            if (n > mps)
                n = mps;
            memcpy(ep_radr[0], rsp + rsp_pos, n);
            rsp_pos += n;
            regs[0][22] = (uint8_t)n;
            regs[0][17] = 1;
        } else if (v == 0x60) {
            regs[0][22] = 0;
            regs[0][17] = bus_dead || !d ? 0x10 : 1;
            if (pend_dev) {
                pend_dev->addr = pend_addr;
                pend_dev->in_default = 0;
                if (pend_dev == dflt)
                    dflt = 0;
                pend_dev = 0;
            }
        } else if (v == 0x42) {
            regs[0][17] = bus_dead || !d ? 0x10 : 0;
        } else {
            regs[0][17] = (uint8_t)v;
        }
    } else if (index_reg == 1 && r == 17) {
        if (v & 0x40u) tx_toggle_clears++;
        regs[1][17] = v & 1u ? 3u : 0u;               /* TXPKTRDY/FIFONotEmpty until an explicit ACK */
    } else if (index_reg == 1 && r == 20) {
        if (v == 0x20) {
            regs[1][20] = 0x20;
            deliver_rx();
        } else if (v == 0x10) {
            regs[1][20] &= (uint8_t)~1u;
        } else if (v == 0x80) {
            rx_toggle_clears++;                         /* ClrDataTog with nothing else */
            regs[1][20] = 0;
        } else {
            regs[1][20] = (uint8_t)v;
        }
    } else {
        regs[index_reg][r] = (uint8_t)v;
    }
}
static uint8_t last_read;
static void mock_rd(uint32_t r) { last_read = (uint8_t)r; }
static uint32_t mock_dt(void)
{
    uint32_t r = last_read;
    if (r == 15)
        return devctl_ls ? 0x20u : 0x40u;
    return r < 16 ? common[r] : regs[index_reg][r];
}
static void mock_enable(uint32_t eps) { (void)eps; }
static void mock_txbuf(uint32_t ep, void *p) { ep_tadr[ep] = p; }
static void mock_rxbuf(uint32_t ep, void *p) { ep_radr[ep] = p; }
static void mock_send(uint32_t ep, void *p, uint32_t n) { ep_tadr[ep] = p; ep_cnt[ep] = n; }
static void mock_ep0_send(void *p, uint32_t n) { mock_send(0, p, n); }
static void mock_ep0_buf(void *p) { ep_radr[0] = p; }
static void mock_nop(void) {}
static void mock_off(void) { off_count++; USB_CON0 = 0; }
static void mock_attach(void *p) { (void)p; }
static void mock_host_attach(void *p) { host_attaches++; ep_radr[0] = p; USB_CON0 = 0x25; }
static uint32_t mock_one(void) { return 1; }
static uint32_t mock_zero(void) { return 0; }
static uint32_t mock_line(void) { return line_state; }
static void fm1_delay_ms(uint32_t ms) { (void)ms; }

#define fm1_usb_ep_enable mock_enable
#define fm1_usb_ep_txbuf mock_txbuf
#define fm1_usb_ep_rxbuf mock_rxbuf
#define fm1_usb_ep_send mock_send
#define fm1_usb_ep0_send mock_ep0_send
#define fm1_usb_ep0_buf mock_ep0_buf
#define fm1_usb_ep4_rxbuf(p) mock_rxbuf(4, p)
#define fm1_usb_ep4_send(p, n) mock_send(4, p, n)
#define fm1_usb_rx_sync mock_nop
#define fm1_usb_sie_on mock_one
#define fm1_usb_sie_done mock_one
#define fm1_usb_sie_wr_start mock_write
#define fm1_usb_sie_rd_start mock_rd
#define fm1_usb_sie_data mock_dt
#define fm1_usb_reset mock_nop
#define fm1_usb_off mock_off
#define fm1_usb_attach mock_attach
#define fm1_usb_host_attach mock_host_attach
#define fm1_usb_line mock_line
#define fm1_usb_sof_take mock_zero
#define fm1_wdt_feed() (fm1_ms++)                       /* the host code's waits run on this clock */
#define RING_PUBLISH() ((void)0)
#ifndef TEST_UAC
#define TEST_UAC 0
#endif
#ifndef TEST_HOST
#define TEST_HOST 1
#endif
#ifndef TEST_CDC
#define TEST_CDC 1
#endif
#ifndef TEST_48K
#define TEST_48K 0
#endif
#ifndef TEST_OUT
#define TEST_OUT TEST_UAC
#endif
#define FELUCCA_UAC TEST_UAC
#define FELUCCA_CDC TEST_CDC
#define FELUCCA_OTA 0
#define FELUCCA_USB_HOST TEST_HOST
#define FELUCCA_UAC_48K TEST_48K
#define FELUCCA_UAC_OUT TEST_OUT
#define HALF_FRAMES 128u
static volatile uint32_t render_peak;
static volatile uint8_t render_busy;
#include "../chip/usb.c"

/* ------------------------------------------- the device's descriptors --- */
/* The configuration descriptor read as a host reads it: the interfaces in order, the audio control header's
 * collection, the endpoints, the IADs; then SET_INTERFACE through ep0_service, as the host sends it. Built in every
 * option set (run_usb_host_tests.sh); TEST_GOLDEN_PLAIN / TEST_GOLDEN_CDC hold the CRC-32 of the device +
 * configuration descriptors as the bsp built them before FELUCCA_UAC_MIDI_LAST, for the option sets that existed. */
#ifndef FELUCCA_UAC_MIDI_LAST
#define FELUCCA_UAC_MIDI_LAST 0
#endif
#ifndef FELUCCA_UAC_IAD
#define FELUCCA_UAC_IAD 0
#endif
typedef struct {
    uint8_t num, cls, sub, alts, nep[2], ep[2][2];
} t_if;
enum { T_AC, T_AS, T_ASO, T_MIDI, T_COMM, T_DATA };
static uint32_t t_crc32(uint32_t c, const uint8_t *p, uint32_t n)
{
    c = ~c;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++)
            c = c >> 1 ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}
static uint8_t t_role(const t_if *f)
{
    if (f->cls == 1)
        return f->sub == 1 ? T_AC : f->sub == 3 ? T_MIDI : f->ep[1][0] == 0x84 ? T_AS : T_ASO;
    return f->cls == 2 ? T_COMM : T_DATA;
}
static const char *T_NAME[] = {"AC", "AS in", "AS out", "MIDI", "CDC comm", "CDC data"};

static void t_setup(uint8_t rt, uint8_t rq, uint16_t val, uint16_t idx, uint16_t len)
{
    const uint8_t s[8] = {rt, rq, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)idx, (uint8_t)(idx >> 8),
                          (uint8_t)len, (uint8_t)(len >> 8)};
    memcpy(ep0buf, s, 8);
    usb.e0_tx = 0;
    sie_wr(S_INDEX, 0);
    regs[0][17] = 0x01;                                 /* RxPktRdy: a SETUP waits */
    ep0_service();
}
static int t_acked(void) { return regs[0][17] == 0x48; }

/* one presentation: the device + configuration descriptors as get_desc gives them; returns their CRC-32 */
static uint32_t check_descriptors(int cdc_on, int verbose)
{
    const uint8_t *dv, *c;
    uint16_t dl, cl;
    t_if f[8];
    uint8_t nf = 0, coll[4] = {0}, ncoll = 0, iad[3][2], niad = 0, iad_at[3], ac_cs = 0, i;
    uint16_t ac_total = 0, ac_sum = 0;
    int in_ac = 0;
#if FELUCCA_CDC
    usb_cdc_on = (uint8_t)cdc_on;
#endif
    assert(get_desc(0x0100, &dv, &dl) && dl == 18 && dv[0] == 18 && dv[1] == 1);
    assert(get_desc(0x0200, &c, &cl) && c[1] == 2);
    assert((c[2] | c[3] << 8) == cl);                   /* wTotalLength: the whole descriptor */
    memset(f, 0, sizeof f);
    for (uint32_t p = c[0]; p < cl; p += c[p]) {
        const uint8_t *d = c + p;
        assert(d[0] >= 2 && p + d[0] <= cl);
        if (d[1] == 4) {                                /* interface */
            in_ac = d[5] == 1 && d[6] == 1;
            if (d[3] == 0) {
                assert(nf < 8 && d[2] == nf);           /* numbered 0.. in the order they come */
                f[nf].num = d[2];
                f[nf].cls = d[5];
                f[nf].sub = d[6];
                nf++;
            } else
                assert(nf && d[2] == f[nf - 1].num && d[3] == f[nf - 1].alts + 1u);
            f[nf - 1].alts = d[3];
        } else if (d[1] == 5) {                         /* endpoint, of the interface setting before it */
            t_if *g = &f[nf - 1];
            uint8_t a = g->alts > 1 ? 1 : g->alts;
            assert(g->nep[a] < 2);
            g->ep[a][g->nep[a]++] = d[2];
        } else if (d[1] == 0x0B) {                      /* IAD: before the first interface it groups */
            assert(niad < 3 && d[0] == 8 && d[2] == nf);
            iad[niad][0] = d[2];
            iad[niad][1] = d[3];
            iad_at[niad++] = d[4];
        } else if (d[1] == 0x24 && in_ac) {             /* class-specific audio control */
            if (d[2] == 1) {
                ac_total = (uint16_t)(d[5] | d[6] << 8);
                ncoll = d[7];
                assert(d[0] == 8 + ncoll && ncoll <= 4);
                memcpy(coll, d + 8, ncoll);
            }
            ac_sum = (uint16_t)(ac_sum + d[0]);
            ac_cs++;
        }
    }
    assert(c[4] == nf && ac_cs && ac_total == ac_sum);

    /* what it should be, written out independently of the descriptor macros */
    const int has_uac = FELUCCA_UAC, out = FELUCCA_UAC_OUT, ml = FELUCCA_UAC_MIDI_LAST, nas = has_uac + out;
    const int as_first = FELUCCA_UAC_AS_FIRST && !ml;
    const int cdc_first = cdc_on && FELUCCA_USB_LAYOUT == 1;
    uint8_t want[8], nw = 0, aud;
    if (cdc_first) {
        want[nw++] = T_COMM;
        want[nw++] = T_DATA;
    }
    aud = nw;
    want[nw++] = T_AC;
    if (!ml)
        want[nw++] = T_MIDI;
    for (i = 0; i < nas; i++)
        want[nw++] = (uint8_t)(i ? T_ASO : T_AS);
    if (ml)
        want[nw++] = T_MIDI;
    if (cdc_on && !cdc_first) {
        want[nw++] = T_COMM;
        want[nw++] = T_DATA;
    }
    assert(nf == nw);
    for (i = 0; i < nf; i++)
        assert(t_role(&f[i]) == want[i]);
    if (verbose) {
        printf("  %s: interfaces", cdc_on ? "with the console" : "plain");
        for (i = 0; i < nf; i++)
            printf(" %u %s%s", f[i].num, T_NAME[t_role(&f[i])], i + 1 < nf ? "," : "");
        printf("; AC collection");
        for (i = 0; i < ncoll; i++)
            printf(" %u", coll[i]);
        printf("; %u bytes, bcdDevice %02X.%02X\n", cl, dv[13], dv[12]);
    }
    /* the collection: the audio function's streaming interfaces and MIDI, in the order the option asks */
    {
        uint8_t wc[4], nwc = 0, midi = 0, as0 = 0;
        for (i = 0; i < nf; i++) {
            if (want[i] == T_MIDI)
                midi = i;
            if (want[i] == T_AS && !as0)
                as0 = i;
        }
        if (!has_uac)
            wc[nwc++] = midi;
        else if (ml == 2)
            for (i = 0; i < nas; i++)
                wc[nwc++] = (uint8_t)(as0 + i);
        else if (ml || as_first) {
            for (i = 0; i < nas; i++)
                wc[nwc++] = (uint8_t)(as0 + i);
            wc[nwc++] = midi;
        } else {
            wc[nwc++] = midi;
            for (i = 0; i < nas; i++)
                wc[nwc++] = (uint8_t)(as0 + i);
        }
        assert(ncoll == nwc && !memcmp(coll, wc, nwc));
        assert(ac_total == (has_uac ? (out ? 53 : 31) - (ml == 2) : 9));
    }
    /* the endpoints never move */
    for (i = 0; i < nf; i++) {
        const t_if *g = &f[i];
        switch (want[i]) {
        case T_AC: assert(g->alts == 0 && g->nep[0] == 0); break;
        case T_MIDI: assert(g->alts == 0 && g->nep[0] == 2 && g->ep[0][0] == 0x01 && g->ep[0][1] == 0x81); break;
        case T_AS: assert(g->alts == 1 && !g->nep[0] && g->nep[1] == 1 && g->ep[1][0] == 0x84); break;
        case T_ASO: assert(g->alts == 1 && !g->nep[0] && g->nep[1] == 1 && g->ep[1][0] == 0x04); break;
        case T_COMM: assert(g->nep[0] == 1 && g->ep[0][0] == 0x82); break;
        case T_DATA: assert(g->nep[0] == 2 && g->ep[0][0] == 0x03 && g->ep[0][1] == 0x83); break;
        }
    }
    /* IADs: with the console, one per function; plain, the audio function's only with FELUCCA_UAC_IAD */
    {
        uint8_t fn = (uint8_t)(1 + nas + (ml != 2)), k = 0;
        int plain_iad = FELUCCA_UAC_IAD && has_uac;
        if (cdc_on) {
            assert(niad == 2);
            if (cdc_first)
                assert(iad[0][0] == 0 && iad[0][1] == 2 && iad_at[0] == 2), k = 1;
            assert(iad[k][0] == aud && iad[k][1] == fn && iad_at[k] == 1);
            if (!cdc_first)
                assert(iad[1][0] == nf - 2 && iad[1][1] == 2 && iad_at[1] == 2);
            assert(dv[2] == 0x00 && dv[3] == 0x02);
        } else {
            assert(niad == (plain_iad ? 1 : 0));
            if (plain_iad)
                assert(iad[0][0] == 0 && iad[0][1] == fn && iad_at[0] == 1);
            if (plain_iad)
                assert(dv[4] == 0xEF && dv[5] == 2 && dv[6] == 1 && dv[2] == 0x00 && dv[3] == 0x02);
            else
                assert(dv[4] == 0 && dv[5] == 0 && dv[6] == 0 && dv[2] == 0x10 && dv[3] == 0x01);
        }
    }
    /* bcdDevice: each option set its own, so a host never reuses what it cached for another */
    {
        uint32_t lo = 0x10u * (uint32_t)(has_uac + FELUCCA_UAC_48K) + 0x40u * (uint32_t)out + 0x08u * (uint32_t)as_first
                      + 0x80u * (uint32_t)(ml == 1) + (cdc_on ? 0x01u + 2u * FELUCCA_USB_LAYOUT : 0u);
        uint32_t hi = 3u + (uint32_t)(ml == 2) + 2u * (uint32_t)(FELUCCA_UAC_IAD && has_uac);
        assert(dv[12] == lo && dv[13] == hi);
    }
    /* SET_INTERFACE: the stream starts and stops on its own interface only */
    usb.config = 1;
    for (i = 0; i < nf; i++) {
        uint8_t n = f[i].num;
        if (want[i] == T_AS || want[i] == T_ASO) {
#if FELUCCA_UAC
            volatile uint8_t *alt = &uac.alt;
#if FELUCCA_UAC_OUT
            if (want[i] == T_ASO)
                alt = &uo.alt;
#endif
            t_setup(0x01, 0x0B, 1, n, 0);
            assert(t_acked() && *alt == 1);
            t_setup(0x81, 0x0A, 0, n, 1);
            assert(ep_cnt[0] == 1 && ep0buf[0] == 1);
            for (uint8_t j = 0; j < nf; j++)            /* alt 1 elsewhere: the other stream's, or stalled */
                if (j != i) {
                    int stream = want[j] == T_AS || want[j] == T_ASO;
                    t_setup(0x01, 0x0B, 1, f[j].num, 0);
                    assert(t_acked() == stream && *alt == 1);
                    t_setup(0x01, 0x0B, 0, f[j].num, 0);
                    assert(t_acked() && *alt == 1);
                }
            t_setup(0x01, 0x0B, 0, n, 0);
            assert(t_acked() && *alt == 0);
#endif
        } else {
            t_setup(0x01, 0x0B, 1, n, 0);
            assert(!t_acked());
            t_setup(0x01, 0x0B, 0, n, 0);
            assert(t_acked());
            t_setup(0x81, 0x0A, 0, n, 1);
            assert(ep_cnt[0] == 1 && ep0buf[0] == 0);
        }
    }
    usb.config = 0;
    return t_crc32(t_crc32(0, dv, 18), c, cl);
}

static void test_descriptors(void)
{
    uint32_t plain, cdc = 0;
    int verbose = getenv("USB_DESC_DUMP") != 0;
    usb_start();
    plain = check_descriptors(0, verbose);
#if FELUCCA_CDC
    cdc = check_descriptors(1, verbose);
    usb_cdc_on = FELUCCA_CDC_DEFAULT != 0;
#endif
    if (getenv("USB_DESC_CRC"))
        printf("crc plain %08X cdc %08X\n", plain, cdc);
#ifdef TEST_GOLDEN_PLAIN
    assert(plain == (uint32_t)TEST_GOLDEN_PLAIN);
#endif
#ifdef TEST_GOLDEN_CDC
    assert(cdc == (uint32_t)TEST_GOLDEN_CDC);
#endif
    usb_detach();
    puts("USB device: descriptors as a host reads them, SET_INTERFACE on the streams: OK");
}

#if FELUCCA_USB_HOST
/* ---------------------------------------------------------- the tests --- */
static void connect(int which)
{
    devices_init();
    root = which;
    bus_dead = devctl_ls = ep0_timeout = 0;
    memset(common, 0, sizeof common);
    memset(regs, 0, sizeof regs);
    dflt = 0;
    rx_head = rx_tail = 0;
    mi_r = mi_w = 0;
    mo_r = mo_w = 0;
    wire_n = 0;
    root_resets = host_attaches = 0;
    memset(port_resets, 0, sizeof port_resets);
    midi_in_overflow = 0;
    if (which == HUB) {
        dev[HUB].port_dev[1] = STICK + 1;               /* port 1: a mass-storage stick, port 3: the keyboard */
        dev[HUB].port_dev[3] = KBD + 1;
    }
    line_state = 2;
    uh.pkts = uh.events = 0;
}

static void host_on(void)
{
    usb_host_switch(1);
    usb_host_task(fm1_ms);
    assert(uh.on && uh.msg == UH_MSG_ON && !usb.up);
    uh.msg = 0;
}

static int enumerate(void)                              /* the main loop until run or give-up */
{
    int k;
    for (k = 0; k < 400 && !uh.run; k++) {
        fm1_ms += 20;
        usb_host_task(fm1_ms);
    }
    return uh.run;
}

static void push_packet(uint32_t pkt)
{
    uint8_t *q = rx_q[rx_head % 8];
    q[0] = (uint8_t)pkt;
    q[1] = (uint8_t)(pkt >> 8);
    q[2] = (uint8_t)(pkt >> 16);
    q[3] = (uint8_t)(pkt >> 24);
    rx_n[rx_head % 8] = 4;
    rx_head++;
}

static void poll_n(int n)
{
    while (n--) {
        usb_poll();
        deliver_rx();
    }
}

static void test_parse(void)
{
    uh_midi_t m;
    uint8_t cut[128];
    assert(uh_parse_midi(kbd_cfg, kbd_cfg_len, &m));
    assert(m.found && m.cfg == 1 && m.iface == 1 && m.alt == 0 && m.in_ep == 1 && m.in_type == 2 &&
           m.in_mps == 64 && m.out_ep == 2);
    assert(!uh_parse_midi(stick_cfg, 32, &m) && !m.found);
    assert(!uh_parse_midi(hub_cfg, 25, &m));
    memcpy(cut, kbd_cfg, kbd_cfg_len);
    assert(uh_parse_midi(cut, 76, &m) && m.out_ep && !m.in_ep); /* OUT remains usable when IN is truncated */
    assert(uh_parse_midi(cut, kbd_cfg_len, &m));
    cut[72] = 0;                                            /* a zero-length descriptor ends the walk safely */
    assert(uh_parse_midi(cut, kbd_cfg_len, &m) && m.out_ep && !m.in_ep);
    assert(uh_midi_ok(0x643C9009u) == 0x643C9009u);        /* note on ch0 */
    assert(uh_midi_ok(0x643C9019u) == 0x643C9009u);        /* cable 1 is dropped */
    assert(uh_midi_ok(0x0000F80Fu) == 0x0000F80Fu);        /* clock */
    assert(uh_midi_ok(0x0000F70Fu) == 0 && uh_midi_ok(0x4F7DF004u) == 0);   /* SysEx, other realtime */
    assert(uh_midi_ok(0x643C8009u) == 0);                  /* CIN does not match status */
    assert(uh_midi_ok(0x003CC00Cu) == 0x003CC00Cu);        /* program change */
    assert(uh_midi_ok(0x64BC9009u) == 0 && uh_midi_ok(0xE43C9009u) == 0);   /* data bytes with bit 7 */
    puts("USB host: descriptor walk and packet filter: OK");
}

static void test_keyboard(int via_hub)
{
    connect(via_hub ? HUB : KBD);
    host_on();
    assert(enumerate());
    assert(uh.run && uh.msg == UH_MSG_MIDI);
    uh.msg = 0;
    assert(dev[KBD].cfg == 1 && dev[KBD].addr == uh.dev && dev[KBD].addr != 0);
    if (via_hub) {
        assert(dev[HUB].cfg == 1 && dev[HUB].addr == 1);
        assert(dev[STICK].addr != 0 && dev[STICK].addr != dev[KBD].addr);   /* the stick was addressed, then passed over */
        assert(dev[KBD].addr == 4);                    /* stable address per hub port */
    } else {
        assert(dev[KBD].addr == 1);
    }
    assert(common[0] == dev[KBD].addr);                 /* FADDR is the keyboard's for the endpoint */
    assert(regs[1][26] == (2 << 4 | 1));                /* RXTYPE: bulk, endpoint 1 */
    assert(regs[1][19] == 0xFF);

    /* events: note on, SysEx (dropped), clock, note on from cable 1 */
    push_packet(0x643C9009u);                           /* cin 9: 90 3C 64 */
    push_packet(0x4F7DF004u);                           /* SysEx start: dropped */
    push_packet(0x0000F80Fu);                           /* clock */
    push_packet(0x7F409119u);                           /* cable 1 note on ch1 */
    poll_n(8);
    assert(mi_w - mi_r == 3);
    assert(midi_in_q[mi_r % MQ] == 0x643C9009u && midi_in_source[mi_r % MQ] == 1);
    assert(midi_in_q[(mi_r + 1) % MQ] == 0x0000F80Fu);
    assert(midi_in_q[(mi_r + 2) % MQ] == 0x7F409109u);  /* the cable bits are gone */
    assert(uh.pkts == 4 && uh.events == 3);

    /* a full ring holds the packet back instead of overflowing */
    mi_r = mi_w = 0;
    mi_w = MQ - 10u;                                    /* fewer than EP1_ROOM free */
    mi_r = 0;
    push_packet(0x643C9009u);
    poll_n(4);
    assert(mi_w == MQ - 10u && !midi_in_overflow);
    mi_r = mi_w;                                        /* the main loop drained it */
    poll_n(2);
    assert(mi_w == mi_r + 1u);
    mi_r = mi_w;

    /* the cable goes: errors, then a fault the main loop recovers from */
    bus_dead = 1;
    line_state = 0;
    rx_toggle_clears = 0;
    poll_n(40);
    assert(!uh.run && uh.fault);
    assert(rx_toggle_clears == 0);                      /* errors never reset the data toggle */
    fm1_ms += 20;
    usb_host_task(fm1_ms);
    assert(!uh.fault && !uh.run && uh.msg == UH_MSG_LOST);
    uh.msg = 0;
    for (int k = 0; k < 100; k++) {                     /* nothing attached: nothing happens */
        fm1_ms += 20;
        usb_host_task(fm1_ms);
    }
    assert(!uh.run && uh.msg == 0);

    /* plugged in again */
    bus_dead = 0;
    bus_reset();
    line_state = 2;
    assert(enumerate() && uh.msg == UH_MSG_MIDI);
    uh.msg = 0;

    /* back to a device: the host leaves the port */
    usb_host_switch(0);
    usb_host_task(fm1_ms);
    assert(!uh.on && !uh.run && uh.msg == UH_MSG_OFF);
    uh.msg = 0;
    usb.up = 0;
    usb_retry(fm1_ms + 5000u);                          /* usb_start again as a device */
    assert(usb.up);
    usb.up = 0;
    printf("USB host: keyboard %s: enumeration, events, ring back-pressure, unplug, replug, back to device: OK\n",
           via_hub ? "behind a hub" : "direct");
}

static void test_no_midi(void)
{
    connect(STICK);
    host_on();
    assert(!enumerate());
    assert(uh.msg == UH_MSG_NOMIDI);                    /* said once, then quiet */
    assert(root_resets == 1 && host_attaches == 1 && dev[STICK].seen_probe == 1);
    uh.msg = 0;
    {
        int k;
        for (k = 0; k < 400; k++) {
            fm1_ms += 20;
            usb_host_task(fm1_ms);
        }
    }
    assert(uh.msg == 0 && !uh.run);
    assert(root_resets == 1 && host_attaches == 1 && dev[STICK].seen_probe == 1);
    /* A real root unplug rearms detection, even after an unsupported device. */
    line_state = 0;
    for (int k = 0; k < 5; k++) { fm1_ms += 20; usb_host_task(fm1_ms); }
    root = KBD; line_state = 2;
    assert(enumerate() && uh.msg == UH_MSG_MIDI && root_resets == 2);
    usb_host_switch(0);
    usb_host_task(fm1_ms);
    uh.msg = 0;

    connect(KBD);
    devctl_ls = 1;
    host_on();
    assert(!enumerate() && uh.msg == UH_MSG_LOW);
    assert(!root_resets && host_attaches == 1);
    uh.msg = 0;
    for (int k = 0; k < 600; k++) {                     /* 12 s with the device still attached: said once */
        fm1_ms += 20;
        usb_host_task(fm1_ms);
    }
    assert(uh.msg == 0 && !uh.run);
    assert(!root_resets && host_attaches == 1);
    usb_host_switch(0);
    usb_host_task(fm1_ms);
    uh.msg = 0;
    puts("USB host: a stick without MIDI, a low-speed device: reported, no endpoint started: OK");
}

/* Idle hub polls may use EP0, but must never reset/reconfigure devices or enter any timed waits. */
static void idle_hub(int ticks)
{
    uint32_t resets = root_resets, attaches = host_attaches, probes = dev[STICK].seen_probe;
    uint32_t configs = dev[HUB].seen_set_cfg;
    uint32_t ports[8];
    memcpy(ports, port_resets, sizeof ports);
    while (ticks--) {
        fm1_ms += 20;
        uint32_t now = fm1_ms;
        usb_host_task(now);
        assert(fm1_ms == now && !uh.run && !uh.msg);
    }
    assert(root_resets == resets && host_attaches == attaches && dev[STICK].seen_probe == probes);
    assert(!memcmp(ports, port_resets, sizeof ports));
    assert(dev[HUB].cfg == 1 && dev[HUB].seen_set_cfg == configs);
}

static void test_idle_hub(int with_stick)
{
    connect(HUB);
    dev[HUB].port_dev[3] = 0;
    if (!with_stick) dev[HUB].port_dev[1] = 0;
    host_on();
    assert(!enumerate() && uh.msg == UH_MSG_NOMIDI);
    assert(root_resets == 1 && host_attaches == 1);
    assert(dev[STICK].seen_probe == (uint32_t)with_stick);
    uh.msg = 0;
    idle_hub(3000);                                    /* one minute: no reset, power-good or reset waits */

    /* An unsupported downstream attachment is tried once, and its change is acknowledged. */
    dev[HUB].port_dev[1] = STICK + 1;
    dev[HUB].port_chg[1] |= 1;
    uint32_t probes = dev[STICK].seen_probe;
    assert(!enumerate() && dev[STICK].seen_probe == probes + 1 && !dev[HUB].port_chg[1]);
    idle_hub(600);
    dev[HUB].port_dev[1] = 0;
    dev[HUB].port_en[1] = 0;
    dev[HUB].port_chg[1] |= 1;                         /* disconnects are acknowledged without port reset */
    idle_hub(100);
    assert(!dev[HUB].port_chg[1]);

    /* An idle MIDI hot-plug resets only its port, preserving the powered hub and the stick. */
    uint32_t stick_resets = port_resets[1];
    dev[HUB].port_dev[3] = KBD + 1;
    dev[HUB].port_chg[3] |= 1;
    assert(enumerate() && uh.msg == UH_MSG_MIDI);
    assert(root_resets == 1 && host_attaches == 1 && port_resets[1] == stick_resets);
    assert(dev[KBD].addr == 4 && port_resets[3] == 1 && !dev[HUB].port_chg[3]);
    usb_host_switch(0); usb_host_task(fm1_ms); uh.msg = 0;
    printf("USB host: idle hub %s: no repeated resets/waits, downstream hot-plug: OK\n",
           with_stick ? "with a stick" : "empty");
}

static void test_failed_probe(void)
{
    connect(-1);                                      /* a powered line with nothing answering EP0 */
    ep0_timeout = 1;
    host_on();
    assert(!enumerate() && uh.msg == UH_MSG_NOMIDI);
    uh.msg = 0;
    uint32_t requests = setup_requests;
    for (int k = 0; k < 3000; k++) {
        fm1_ms += 20;
        uint32_t now = fm1_ms;
        usb_host_task(now);
        assert(fm1_ms == now && !uh.run && !uh.msg);
    }
    assert(root_resets == 1 && host_attaches == 1 && setup_requests == requests);
    usb_host_switch(0); usb_host_task(fm1_ms); uh.msg = 0;

    connect(HUB);
    memset(dev[HUB].port_dev, 0, sizeof dev[HUB].port_dev);
    host_on(); assert(!enumerate()); uh.msg = 0;
    ep0_timeout = 1;                                  /* configured hub stops answering without line drop */
    for (int k = 0; k < 30; k++) { fm1_ms += 20; usb_host_task(fm1_ms); }
    assert(!uh.hub_addr && root_resets == 1);
    requests = setup_requests;
    idle_hub(600);
    assert(setup_requests == requests);
    usb_host_switch(0); usb_host_task(fm1_ms); uh.msg = 0;
    puts("USB host: failed EP0 probe and unresponsive idle hub wait for unplug/replug: OK");
}

static void test_hub_synth_unplug(void)
{
    static const uint8_t synth[] = {9,2,25,0,1,1,0,0x80,50, 9,4,1,0,1,1,3,0,0, 7,5,2,2,8,0,0};
    connect(HUB);
    memcpy(kbd_cfg, synth, sizeof synth);
    dev[KBD].cfglen = sizeof synth;
    host_on(); assert(enumerate() && !uh.in_ep && uh.out_mps);
    assert(midi_out_try(0x643c9009u)); usb_poll();
    dev[HUB].port_dev[3] = 0;
    dev[HUB].port_en[3] = 0;
    dev[HUB].port_chg[3] |= 1;
    dev[KBD].present = 0;
    /* OUT errors while the root hub remains on the line. */
    for (uint32_t k = 0; k < UH_MAX_ERRORS; k++) { regs[1][17] = 4; usb_poll(); }
    assert(uh.fault && !uh.run && line_state == 2);
    usb_host_task(fm1_ms); assert(!uh.fault && uh.msg == UH_MSG_LOST && mo_r == mo_w);
    uh.msg = 0;
    assert(!enumerate() && uh.msg == UH_MSG_NOMIDI);
    uh.msg = 0;
    assert(root_resets == 2 && host_attaches == 2);
    idle_hub(3000);
    dev[KBD].present = 1;
    dev[HUB].port_dev[3] = KBD + 1;
    dev[HUB].port_chg[3] |= 1;
    assert(enumerate() && !uh.in_ep && uh.out_mps && uh.msg == UH_MSG_MIDI);
    assert(root_resets == 2 && host_attaches == 2 && dev[KBD].addr == 4);
    usb_host_switch(0); usb_host_task(fm1_ms); uh.msg = 0;
    puts("USB host: unplugged OUT-only synth leaves hub idle without retry resets; replug works: OK");
}

static void ack_tx(void)
{
    const uint8_t *p = ep_tadr[1];
    assert(regs[1][17] & 1u);
    assert(ep_cnt[1] && ep_cnt[1] <= uh.out_mps && !(ep_cnt[1] & 3u));
    for (uint32_t i = 0; i < ep_cnt[1]; i += 4u) {
        assert(wire_n < sizeof wire / sizeof wire[0]);
        wire[wire_n++] = (uint32_t)p[i] | (uint32_t)p[i+1] << 8 | (uint32_t)p[i+2] << 16 | (uint32_t)p[i+3] << 24;
    }
    regs[1][17] = 0;                                   /* ACK empties the FIFO and clears TXPKTRDY */
}

static void recover_host(void)
{
    usb_host_task(fm1_ms);
    assert(!uh.run && !uh.fault && midi_usb_lost && mo_r == mo_w);
    midi_usb_lost = 0;                                 /* the production audio ISR consumes this */
    assert(enumerate());
}

static void test_host_output(int via_hub)
{
    connect(via_hub ? HUB : KBD);
    host_on(); assert(enumerate());
    assert(usb_midi_out_ready() && !usb.config);
    line_state = 0; usb_poll(); assert(uh.run);        /* an EOP sample is not an unplug */
    line_state = 2; usb_poll(); assert(uh.run && !uh.line_idle);
    assert(regs[1][24] == 0x22 && regs[1][25] == 0 && regs[1][18] == 0x20);
    uint32_t first = mo_r, toggles = tx_toggle_clears;
    for (uint32_t i = 0; i < MQ; i++)
        assert(midi_out_try(0x09u | (0x90u | (i & 15u)) << 8 | i << 16 | 93u << 24));
    assert(!midi_out_try(0x003c8008u));               /* full: releases remain CPU0's responsibility */
    usb_poll();
    assert(uh.tx_count == 16 && mo_r == first && ep_cnt[1] == 64);
    uint8_t pending[64]; memcpy(pending, ep_tadr[1], 64);
    push_packet(0x643c9009u); poll_n(8);                /* input continues while output is busy */
    assert(mi_w == 1 && mo_r == first && !memcmp(pending, ep_tadr[1], 64));
    regs[1][17] = 0x80; usb_poll();                    /* NAK: retain ring and data */
    assert(mo_r == first && tx_toggle_clears == toggles);
    regs[1][17] = 0x04; usb_poll();                    /* transient bus error: same packet/toggle */
    assert(mo_r == first && !memcmp(pending, ep_tadr[1], 64) && tx_toggle_clears == toggles);
    while (mo_r != mo_w) { ack_tx(); usb_poll(); }
    assert(wire_n == MQ && !uh.tx_count);
    for (uint32_t i = 0; i < MQ; i++)
        assert(wire[i] == (0x09u | (0x90u | (i & 15u)) << 8 | i << 16 | 93u << 24));
    /* Ring wrap and short packets preserve release/controller order. */
    assert(midi_out_try(0x003c8008u)); assert(midi_out_try(0x000040bbu));
    usb_poll(); assert(ep_cnt[1] == 8 && mo_w - mo_r == 2);
    ack_tx(); usb_poll(); assert(wire[MQ] == 0x003c8008u && wire[MQ+1] == 0x000040bbu);
    /* A stall discards old queued note-ons and requests cleanup, then re-enumerates. */
    assert(midi_out_try(0x643c9009u)); usb_poll(); regs[1][17] = 0x20; usb_poll();
    assert(uh.fault && !uh.run && !midi_out_try(0x643d9009u)); recover_host();
    assert(mo_r == mo_w && !uh.tx_count);
    /* Persistent errors and an unresponsive/NAKing output have bounded recovery. */
    assert(midi_out_try(0x643c9009u)); usb_poll();
    for (uint32_t i = 0; i < UH_MAX_ERRORS; i++) { regs[1][17] = 4; usb_poll(); }
    assert(uh.fault); recover_host();
    assert(midi_out_try(0x643c9009u)); usb_poll(); fm1_ms += 500; usb_poll();
    assert(uh.fault); recover_host();
    /* A console-layout change must not take host DMA away from its ISR. */
    uint32_t off = off_count, type = regs[1][24];
    usb_cdc_switch(!usb_cdc_on);
    assert(uh.run && off_count == off && regs[1][24] == type);
    usb_host_switch(0); usb_host_task(fm1_ms);
    assert(!uh.on && !usb_midi_out_ready());
    usb_retry(fm1_ms + 1000); usb.config = 1;
    assert(midi_out_try(0x643c9009u));               /* ordinary computer-facing output remains available */
    usb.config = usb.up = 0; mo_r = mo_w;
    printf("USB host OUT %s: duplex, ACK ownership, NAK/error retries, bounded faults, mode settings: OK\n",
           via_hub ? "behind a hub" : "direct");
}

static void test_endpoint_selection(void)
{
    uh_midi_t m;
    uint8_t d[] = {9,2,41,0,1,1,0,0x80,50,
                  9,4,2,1,1,1,3,0,0, 7,5,2,2,8,0,0,
                  9,4,2,2,1,1,3,0,0, 7,5,0x81,2,64,0,0};
    assert(uh_parse_midi(d, sizeof d, &m) && m.alt == 1 && m.out_ep == 2 && m.out_mps == 8 && !m.in_ep);
    d[14] = 0;                                        /* first alternate isn't MIDI: use second */
    assert(uh_parse_midi(d, sizeof d, &m) && m.alt == 2 && m.in_ep == 1 && !m.out_ep);
    d[32] = 2;                                        /* MIDI 2.0 protocol is not a MIDI 1.0 event stream */
    assert(!uh_parse_midi(d, sizeof d, &m));
    d[32] = 0; d[38] = 0;                             /* invalid packet size */
    assert(!uh_parse_midi(d, sizeof d, &m));
    /* A receive-only controller rejects output; an output-only synth is still enumerated. */
    connect(KBD); kbd_cfg_len = build_kbd_cfg(kbd_cfg, 0);
    host_on(); assert(enumerate() && uh.in_ep && !uh.out_mps && !midi_out_try(0x643c9009u));
    usb_host_switch(0); usb_host_task(fm1_ms);
    connect(KBD);
    kbd_cfg_len = 25; memcpy(kbd_cfg, d, 25); kbd_cfg[14] = 1;
    kbd_cfg[2] = 25;
    host_on(); assert(enumerate() && !uh.in_ep && uh.out_mps == 8 && dev[KBD].alt == 1);
    for (uint32_t i = 0; i < 3; i++) assert(midi_out_try(0x643c9009u + (i << 16)));
    usb_poll(); assert(ep_cnt[1] == 8); ack_tx(); usb_poll();
    assert(ep_cnt[1] == 4); ack_tx(); usb_poll(); assert(wire_n == 3 && mo_r == mo_w);
    line_state = 0; poll_n(16); assert(!uh.run && uh.fault); /* idle output-only unplug still detected */
    usb_host_switch(0); usb_host_task(fm1_ms);
    puts("USB host: output-only synth, input-only controller, alternate boundaries, protocol/packet validation: OK");
}

static void test_device_mode_untouched(void)
{
    /* with the host off, usb_poll is the device driver and usb_host_task does nothing */
    connect(KBD);
    uh.want = uh.on = uh.run = 0;
    usb.up = 0;
    usb_host_task(100);
    assert(!uh.on && !uh.msg);
    usb_retry(fm1_ms + 5000u);
    assert(usb.up);
    usb.up = 0;
    puts("USB host: off at boot, the device side starts as before: OK");
}

static void test_mode_lifecycle(void)
{
    connect(KBD);
    usb_start(); usb.config = 1;
    cdc.dtr = 1;
#if FELUCCA_UAC
    uac_stream(1);
#endif
#if FELUCCA_UAC_OUT
    uo_stream(1);
#endif
    uint32_t link = midi_out_link();
    host_on();
    assert(usb_host_active() && !usb_host_connected() && !cdc.dtr);
#if FELUCCA_UAC
    assert(!uac.alt);
#endif
#if FELUCCA_UAC_OUT
    assert(!uo.alt);
#endif
    assert(enumerate() && usb_host_connected() && midi_out_link() == link + 1u);
    assert(usb_host_message() == UH_MSG_MIDI && usb_host_message() == UH_MSG_NONE);
    uint32_t off = off_count;
    usb_retry(fm1_ms + 5000u);
    assert(uh.run && off_count == off && !usb.up);
    usb_detach();
    assert(!usb_host_active() && !uh.run && midi_usb_lost);
    usb_host_switch(1); usb_host_task(fm1_ms + 5000u); usb_retry(fm1_ms + 5000u);
    assert(!uh.on && !usb.up && off_count == off + 1u);
    puts("USB host: mode API, audio/CDC stop, link generation, retry exclusion and detach: OK");
}

int main(void)
{
    devices_init();
    test_parse();
    test_device_mode_untouched();
    test_keyboard(0);
    test_keyboard(1);
    test_no_midi();
    test_idle_hub(0);
    test_idle_hub(1);
    test_failed_probe();
    test_hub_synth_unplug();
    test_host_output(0);
    test_host_output(1);
    test_endpoint_selection();
    test_mode_lifecycle();
    test_descriptors();
    puts("USB host: all OK");
    return 0;
}

#else
int main(void)
{
    test_descriptors();
    usb_start();
    assert(usb.up && !usb.config);
    usb.config = 1;
    assert(midi_out_try(0x643c9009u));
    assert(midi_out_room() == MQ - 1u);
    usb_detach();
    usb_retry(5000u);
    assert(!usb.up);
    puts("USB device-only/loader: startup, MIDI output and detach: OK");
    return 0;
}
#endif
