/* SPDX-License-Identifier: GPL-3.0-only OR MPL-2.0 */
/* Copyright (C) 2026 Henri David */
/* FM-1 (AC791N / WL82) Wi-Fi MAC: JieLi's "wl30" core (MAC_CSR0 reads 0x00300100 on the FM-1) with Ralink's RT3070
 * register layout. MAC register r (Ralink's numbering: MAC_CSR0 0x1000, BBP_CSR_CFG 0x101C ...) is plain MMIO at
 * 0x30000 + r, as the JieLi AC79 SDK reaches it (wl_sdio.c wl30_mmc_io_rw_extended, Apache-2.0). Frames go by DMA
 * (0x30F00-0x30F1C), the Wi-Fi clock / reset block is at 0x14040-0x1405C. The register addresses and values are facts
 * read from JieLi's SDK (Apache-2.0: NOTICE), the code around them is Optimist's (docs/WIFI-HW-FACTS.md there: every
 * measurement). Used by fm1-base's own Wi-Fi stack (wifi_stack = "own": wifi/, device/wifi_own.c).
 *
 * The Wi-Fi side needs the radio started: the Bluetooth start-up (fm1_ble.h fm1_ble_rf_init, captured from stock V15)
 * already is JieLi's Wi-Fi radio quick start (33,357 of its 34,020 writes match), so Wi-Fi comes up on a radio BT has
 * started, with three more steps (fm1_wifi_up): the station's MAC settings, both RF paths on (0x14000 |= 3) and the
 * coexistence controller's set-up words (to 0x14004), which made reception work. Those words are JieLi's (565 of them,
 * recorded in an emulator, Apache-2.0) and are not in this BSP: the caller passes them to fm1_wifi_up (fm1-base keeps
 * them in device/fm1_wifi_wlc.h).
 *
 *   fm1_wifi_up(hi, code, n)      the Wi-Fi side on (the radio started by BT first), with the caller's WLC words
 *   fm1_wifi_is_up()              .. its RF path is on
 *   fm1_wifi_set_channel(ch)      1..13 (byte 0 of 0x1405C = channel + 101)
 *   fm1_wifi_rx(buf, max)         one received frame by the RX DMA: RXWI (16 B) then the 802.11 frame
 *   fm1_wifi_tx_prepare / _tx     one frame out by the TX DMA (a TXWI before it; _prepare_rate: at a rate, FM1_WIFI_RATE_*)
 *   fm1_wifi_mac_rd / _wr         a MAC register; the key tables' places below (FM1_WIFI_WCID_* ...) */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"
#include "fm1_time.h"

/* a register read / write of fm1_wifi_mac_rd / _wr and fm1_wifi_rd / _wr (so all of fm1_wifi_up's); a host test may
 * define both before it includes this header (tests/wifi_up_test.c) */
#ifndef FM1_WIFI_RD
#define FM1_WIFI_RD(a) (*(const volatile uint32_t *)(uintptr_t)(a))
#define FM1_WIFI_WR(a, v) (*(volatile uint32_t *)(uintptr_t)(a) = (v))
#endif

#define FM1_WIFI_MAC 0x30000u            /* MAC register r at FM1_WIFI_MAC + r */
#define FM1_WIFI_WDMA1 0x30F04u          /* the frame DMA's status: bit 17 a frame waits, 20 TX done, 21 RX done */
#define FM1_WIFI_RX_PEND (1u << 17)
#define FM1_WIFI_RX_DONE (1u << 21)
#define FM1_WIFI_RX_CLR (1u << 11)       /* written before / after each RX transfer */
#define FM1_WIFI_RX_ADDR 0x30F18u        /* RX DMA: the buffer */
#define FM1_WIFI_RX_LEN 0x30F1Cu         /* .. and the byte count */
#define FM1_WIFI_RX_ACK 0x324u           /* MAC register written 2 before a frame is fetched (sdio_recv_int_hdl) */
#define FM1_WIFI_RX_MAX 4088u            /* the SDK takes a length below 4089 */
#define FM1_WIFI_TX_ADDR 0x30F10u
#define FM1_WIFI_TX_LEN 0x30F14u
#define FM1_WIFI_TX_CLR (1u << 10)       /* written before / after each TX transfer */
#define FM1_WIFI_TX_DONE (1u << 20)
#define FM1_WIFI_TX_HDR 24u              /* bytes before the 802.11 frame in a TX buffer: length, queue, TXWI */
#define FM1_WIFI_RXWI 16u                /* bytes before the 802.11 frame in a received one */

/* The frame DMA reads main RAM: a TX buffer placed 128 KiB or more above the SRAM base (0x01C20000..) went out as
 * garbage (the DMA address likely wraps there), so the buffers it moves must sit below FM1_WIFI_DMA_TOP (measured on
 * the FM-1: below works, above did not; why is not proven) */
#define FM1_WIFI_DMA_TOP 0x01C20000u

/* one frame out by the TX DMA, as the SDK's usb_submit_urb / wl30_mmc_io_rw_extended (addr 0) hand it over. buf
 * (4-aligned): word 0 = MPDU length + 22; word 1 = 4 (queue 2); a 16-byte Ralink TXWI (word 0 the rate: 0xB0, 1 Mb/s
 * CCK; word 1 ACK | BA window 13 << 2 | WCID << 8 | MPDU length << 16); then the 802.11 frame. A frame with the
 * Protected bit set is encrypted by the MAC with the WCID's key (CCMP; the IV / EIV from the WCID's table); a frame
 * at a WCID with no key goes out as it is. ack 1: the MAC waits for the receiver's ACK and retries (0 for beacons and
 * group frames). WCID 0 is the MAC's drop entry: never used */
FM1_INLINE void fm1_wifi_tx_prepare_rate(uint32_t *buf, uint32_t mpdu_len, uint32_t wcid, uint32_t ack, uint32_t w0)
{
    buf[0] = mpdu_len + 22u;
    buf[1] = 4u;
    buf[2] = w0;
    buf[3] = (ack & 1u) | 13u << 2 | (wcid & 0xFFu) << 8 | (mpdu_len & 0xFFFu) << 16;
    buf[4] = buf[5] = 0;
}
/* TXWI word 0 (Ralink's _TXWI_STRUC, the SDK's debug info in wl_wifi_sta.a cmm_data.c: MCS [22:16], PHYMODE [31:30], 0 CCK,
 * 1 OFDM; bits 4-7 the SDK's own 0xB0, kept): the CCK MCS 0..3 = 1, 2, 5.5, 11 Mb/s (long preamble), the OFDM MCS 0..7 =
 * 6 .. 54 Mb/s. JieLi's rate table (wl_usb_to_sdio.c mcs_mode_tab, wifi_set_tx_rate_control_tab) runs CCK 1M, CCK 2M,
 * CCK 5.5M, OFDM 6M, .. in that order */
#define FM1_WIFI_TXWI(phy, mcs) (0xB0u | ((uint32_t)(mcs) & 0x7Fu) << 16 | ((uint32_t)(phy) & 3u) << 30)
#define FM1_WIFI_RATE_CCK1 FM1_WIFI_TXWI(0, 0)   /* 0xB0: every frame until fm1-base's Wi-Fi tuning */
#define FM1_WIFI_RATE_CCK2 FM1_WIFI_TXWI(0, 1)
#define FM1_WIFI_RATE_OFDM6 FM1_WIFI_TXWI(1, 0)
FM1_INLINE void fm1_wifi_tx_prepare(uint32_t *buf, uint32_t mpdu_len, uint32_t wcid, uint32_t ack)
{
    fm1_wifi_tx_prepare_rate(buf, mpdu_len, wcid, ack, FM1_WIFI_RATE_CCK1);
}
/* the frame fm1_wifi_tx_prepare set up, out; 0, or -1 if the DMA did not finish within 10 ms (one frame at a time) */
FM1_INLINE int fm1_wifi_tx(uint32_t *buf)
{
    volatile uint32_t *s = (volatile uint32_t *)(uintptr_t)FM1_WIFI_WDMA1;
    uint32_t t0;
    *s |= FM1_WIFI_TX_CLR;
    __asm__ volatile("csync;");
    *(volatile uint32_t *)(uintptr_t)FM1_WIFI_TX_ADDR = (uint32_t)(uintptr_t)buf + 4u;
    *(volatile uint32_t *)(uintptr_t)FM1_WIFI_TX_LEN = buf[0] + 4u;
    __asm__ volatile("csync;");
    t0 = FM1_T4_CNT;
    while (!(*s & FM1_WIFI_TX_DONE))
        if (FM1_T4_CNT - t0 > 10000u * FM1_TICKS_PER_US)
            return -1;
    *s |= FM1_WIFI_TX_CLR;
    return 0;
}

/* a received frame (after the buffer's 4-byte length word): Ralink's RXWI, 16 bytes, then the 802.11 frame. RXWI word
 * 0: WCID [7:0] (0xFF: no station), MPDU length [27:16]; bytes 8-10 the RSSI per chain (signed dBm). Measured on the
 * FM-1 against beacons and an action frame. The MAC decrypts a CCMP frame for a keyed WCID and strips its header */
FM1_INLINE uint32_t fm1_wifi_rx_mpdu_len(const uint8_t *f) { return (uint32_t)(f[2] | f[3] << 8) & 0xFFFu; }
FM1_INLINE int32_t fm1_wifi_rx_rssi(const uint8_t *f) { return (int8_t)f[8]; }

FM1_INLINE uint32_t fm1_wifi_mac_rd(uint32_t r) { return FM1_WIFI_RD(FM1_WIFI_MAC + r); }
FM1_INLINE void fm1_wifi_mac_wr(uint32_t r, uint32_t v) { FM1_WIFI_WR(FM1_WIFI_MAC + r, v); }
FM1_INLINE uint32_t fm1_wifi_rd(uint32_t a) { return FM1_WIFI_RD(a); }
FM1_INLINE void fm1_wifi_wr(uint32_t a, uint32_t v) { FM1_WIFI_WR(a, v); }

/* the MAC's address and key tables (Ralink's layout; the SDK's AsicUpdateWcidAttributeEntry, RTMPSetWcidSecurityInfo):
 * a station is WCID 1 (its AP), an access point's clients WCID 1..N */
#define FM1_WIFI_ADDR_OWN 0x1008u        /* our address: bytes 0-3, then 4-5 | 0x00FF0000 */
#define FM1_WIFI_ADDR_BSSID 0x1010u      /* the BSSID: bytes 0-3, then 4-5 */
#define FM1_WIFI_RX_FILTER 0x1400u       /* Ralink's RX_FILTR_CFG (bit n set = drop that kind) */
#define FM1_WIFI_RX_FILTER_ALL 0x1u      /* .. drop only frames with a bad CRC */
/* .. Ralink's STANORMAL, the SDK's STA driver's (wl_wifi_sta.a, written in fm1_wifi_sta_init): dropped are bad CRC (bit 0), PHY
 * errors (1), unicast not to our address (2: 0x1008, all six bytes compared, 0x100C [23:16] 0xFF), a wrong version (4),
 * retried duplicates (7), and the control frames: CF-End+ACK, CF-End, ACK, CTS, RTS, PS-Poll, BA, the rest (8-14, 16; the
 * MAC takes the ACK of its own frames itself: the host never needs them). Kept: other networks' frames (3 clear), multicast
 * (5 clear: Link's group), broadcast (6 clear: ARP, DHCP), BAR (15) */
#define FM1_WIFI_RX_FILTER_STA 0x17F97u
/* The MAC's own retries step down a rate each (TX_RTY_CFG 0x134C bit 30, TX_AUTO_FB_ENABLE: set in fm1_wifi_sta_init's
 * 0x47D01F0F; the retry limits [7:0] 15, [15:8] 31) by the legacy fallback tables, Ralink's LG_FBK_CFG0 (0x135C, nibble i:
 * where OFDM rate i goes, 8 + its index) and LG_FBK_CFG1 (0x1360, nibble i: where CCK rate i goes). JieLi's values with every
 * rate allowed (wl_usb_to_sdio.c wifi_set_tx_rate_control_tab: rate i to the next lower one allowed, the lowest to itself;
 * read-modify-write of the low 32 / 16 bits): CCK 2M retries at 1M; OFDM 6M at 6M (it never falls to CCK) */
#define FM1_WIFI_LG_FBK0 0x135Cu
#define FM1_WIFI_LG_FBK1 0x1360u
#define FM1_WIFI_LG_FBK0_SDK 0xEDCBA988u
#define FM1_WIFI_LG_FBK1_SDK 0x00002100u
FM1_INLINE void fm1_wifi_fallback_sdk(void)
{
    fm1_wifi_mac_wr(FM1_WIFI_LG_FBK0, FM1_WIFI_LG_FBK0_SDK);
    fm1_wifi_mac_wr(FM1_WIFI_LG_FBK1, (fm1_wifi_mac_rd(FM1_WIFI_LG_FBK1) & ~0xFFFFu) | FM1_WIFI_LG_FBK1_SDK);
}
/* The receiver's highest AGC gain: entry 44 of the BBP's window C ("cca"), JieLi's adj_agc_max_gain (wl_rf_common.a
 * wf_soft_adj.c: cca_reg_set(44, g), i.e. window C's three BBP writes) by the joined AP's signal: above -36 dBm 34, above -46 43,
 * above -60 51, above -76 57, else 66; 66 when nothing from the AP was heard for 220 ms (wl_hw_init.c WifiRecvPkgHook,
 * agc_adj_interval 2 x 110 ms). The start-up program writes 66 there (ble_rf_prog's window write C[44]), the SDK's first value too */
#define FM1_WIFI_AGC_WIN 0u
#define FM1_WIFI_AGC_ENTRY 44u
#define FM1_WIFI_AGC_START 66u
#define FM1_WIFI_WCID_ADDR(w) (0x1800u + 8u * (w))   /* WCID w's peer address: bytes 0-3, then 4-5 */
#define FM1_WIFI_PKEY(w) (0x4000u + 32u * (w))       /* its pairwise key (16 B, then 16 B of MIC keys: 0 for CCMP) */
#define FM1_WIFI_IVEIV(w) (0x6000u + 8u * (w))       /* its TX IV and EIV: ((key id << 6 | 0x20 ExtIV) << 24) | PN */
#define FM1_WIFI_WCID_ATTR(w) (0x6800u + 4u * (w))   /* its attribute: bit 0 the pairwise table, bits 1-3 the cipher */
#define FM1_WIFI_SKEY(i) (0x6C00u + 32u * (i))       /* shared (group) key i of BSS 0 */
#define FM1_WIFI_SKEY_MODE 0x7000u                   /* the shared keys' ciphers, 4 bits each */
#define FM1_WIFI_CIPHER_CCMP 4u
FM1_INLINE void fm1_wifi_key_wr(uint32_t r, const uint8_t *key)   /* a 32-byte key entry: key (16 B, or 0 = none), 0s */
{
    uint32_t i;
    for (i = 0; i < 8u; i++)
        fm1_wifi_mac_wr(r + 4u * i, key && i < 4u ? (uint32_t)key[4 * i] | (uint32_t)key[4 * i + 1] << 8 |
                                                         (uint32_t)key[4 * i + 2] << 16 | (uint32_t)key[4 * i + 3] << 24 : 0u);
}
FM1_INLINE void fm1_wifi_addr_wr(uint32_t r, const uint8_t *mac, uint32_t hi_or)   /* an address pair of words */
{
    fm1_wifi_mac_wr(r, (uint32_t)mac[0] | (uint32_t)mac[1] << 8 | (uint32_t)mac[2] << 16 | (uint32_t)mac[3] << 24);
    fm1_wifi_mac_wr(r + 4u, (uint32_t)mac[4] | (uint32_t)mac[5] << 8 | hi_or);
}

#define FM1_WIFI_CHAN 0x1405Cu           /* byte 0 = channel + 101 (the SDK's wifi_set_channel); byte 2 the TX power (6) */
FM1_INLINE uint32_t fm1_wifi_channel(void) { return (fm1_wifi_rd(FM1_WIFI_CHAN) & 0xFFu) - 101u; }
FM1_INLINE void fm1_wifi_set_channel(uint32_t ch)
{
    volatile uint32_t *c = (volatile uint32_t *)(uintptr_t)FM1_WIFI_CHAN;
    *c = (*c & ~0xFFu) | ((ch + 101u) & 0xFFu);
}

/* the Ralink STA driver's MAC settings when the interface comes up (rt28xx_init / NICInitializeAsic / RTMPEnableRxTx,
 * after the radio start-up), in its order, as the SDK's wl_wifi_sta.a writes them in the emulator; the WCID / key
 * table clears left out */
static const uint32_t fm1_wifi_sta_init[][2] = {
    {0x042Cu, 0xF8F0E8E0u}, {0x0430u, 0x6F77D0C8u}, {0x1408u, 0x0000013Fu}, {0x140Cu, 0x00008003u},
    {0x1400u, 0x00017F97u}, {0x1350u, 0x00001020u}, {0x1018u, 0x00001F00u}, {0x040Cu, 0x1F3FBF9Fu},
    {0x134Cu, 0x47D01F0Fu}, {0x1404u, 0x00000013u}, {0x1364u, 0x05740003u}, {0x1368u, 0x05740003u},
    {0x0408u, 0x00F40006u}, {0x1370u, 0x03F44084u}, {0x1374u, 0x01744004u}, {0x1378u, 0x03F44084u},
    {0x136Cu, 0x01744004u}, {0x1340u, 0x0000583Fu}, {0x1344u, 0x00092B20u}, {0x1380u, 0x002400CAu},
    {0x1608u, 0x00000002u}, {0x1204u, 0x00000003u}, {0x1114u, 0x00000000u}, {0x1340u, 0x0000583Fu},
    {0x1008u, 0x00000000u}, {0x100Cu, 0x00FF0000u}, {0x1300u, 0x00064300u}, {0x1304u, 0x000A4700u},
    {0x1308u, 0x00043538u}, {0x130Cu, 0x0003212Fu}, {0x1400u, 0x00017F97u},
};
#define FM1_WL_EN 0x14000u               /* bit 0 BT, bit 1 Wi-Fi: the RF paths (the SDK's wifi_rf_on / wifi_rf_off) */
#define FM1_WL_EN_WIFI 2u
/* The coexistence controller's ("WLC") port: the SDK's wlc_main (cpu.a wlc.c) writes its set-up words here, in order,
 * after both RF paths are on. The words are the caller's, packed in two arrays: word i =
 * FM1_WIFI_WLC_WORD(hi[i], code[i]) = hi[i] << 16 | code[i] << 4 | 1 (every recorded word has that form). This BSP
 * keeps none of them: they are JieLi's data (Apache-2.0), and fm1-base passes its device/fm1_wifi_wlc.h */
#define FM1_WLC_PORT 0x14004u
#define FM1_WIFI_WLC_WORD(hi, code) ((uint32_t)(hi) << 16 | (uint32_t)(code) << 4 | 1u)
/* the Wi-Fi side on, on a radio BT has started: the STA settings, then the SDK's wlc_main (both RF paths on, the
 * coexistence controller's n set-up words from hi / code) */
FM1_INLINE void fm1_wifi_up(const uint16_t *hi, const uint8_t *code, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < sizeof fm1_wifi_sta_init / sizeof *fm1_wifi_sta_init; i++)
        fm1_wifi_mac_wr(fm1_wifi_sta_init[i][0], fm1_wifi_sta_init[i][1]);
    fm1_wifi_wr(FM1_WL_EN, fm1_wifi_rd(FM1_WL_EN) | 3u);
    for (i = 0; i < n; i++)
        fm1_wifi_wr(FM1_WLC_PORT, FM1_WIFI_WLC_WORD(hi[i], code[i]));
}
FM1_INLINE int fm1_wifi_is_up(void) { return (fm1_wifi_rd(FM1_WL_EN) & FM1_WL_EN_WIFI) != 0u; }

FM1_INLINE int fm1_wifi_dma_wait(uint32_t bit, uint32_t ticks)   /* 1: the bit came up within ticks (TIMER4) */
{
    uint32_t t0 = FM1_T4_CNT;
    while (!(fm1_wifi_rd(FM1_WIFI_WDMA1) & bit))
        if (FM1_T4_CNT - t0 > ticks)
            return 0;
    return 1;
}
/* one waiting frame into buf (4-aligned, max + 8 bytes; max <= FM1_WIFI_RX_MAX), as the SDK's wl30_mmc_io_rw_extended
 * (addr 12): the length word first (big endian), then length + 4 bytes from buf + 4. Returns the length, 0 if none
 * waited, -1 if the DMA did not finish within 10 ms or the length was out of range. No cache maintenance around it (the
 * SDK has a csync only) */
FM1_INLINE int32_t fm1_wifi_rx(uint32_t *buf, uint32_t max)
{
    volatile uint32_t *s = (volatile uint32_t *)(uintptr_t)FM1_WIFI_WDMA1;
    uint32_t n, w;
    if (!(*s & FM1_WIFI_RX_PEND))
        return 0;
    fm1_wifi_mac_wr(FM1_WIFI_RX_ACK, 2u);
    *s |= FM1_WIFI_RX_CLR;
    __asm__ volatile("csync;");
    *(volatile uint32_t *)(uintptr_t)FM1_WIFI_RX_ADDR = (uint32_t)(uintptr_t)buf;
    *(volatile uint32_t *)(uintptr_t)FM1_WIFI_RX_LEN = 4u;
    __asm__ volatile("csync;");
    if (!fm1_wifi_dma_wait(FM1_WIFI_RX_DONE, 10000u * FM1_TICKS_PER_US))
        return -1;
    *s |= FM1_WIFI_RX_CLR;
    __asm__ volatile("csync;");
    w = *(volatile uint32_t *)buf;
    n = w >> 24 | (w >> 8 & 0xFF00u) | (w << 8 & 0xFF0000u) | w << 24;
    if (!w || n > max || n > FM1_WIFI_RX_MAX)
        return -1;
    *(volatile uint32_t *)(uintptr_t)FM1_WIFI_RX_ADDR = (uint32_t)(uintptr_t)buf + 4u;
    *(volatile uint32_t *)(uintptr_t)FM1_WIFI_RX_LEN = n + 4u;
    __asm__ volatile("csync;");
    if (!fm1_wifi_dma_wait(FM1_WIFI_RX_DONE, 10000u * FM1_TICKS_PER_US))
        return -1;
    *s |= FM1_WIFI_RX_CLR;
    return (int32_t)n;
}
