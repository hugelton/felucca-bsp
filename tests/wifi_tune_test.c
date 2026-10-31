/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Henri David */
/* The Wi-Fi tuning's register facts in chip/fm1_wifi.h on the host: the TXWI word 0 of a frame at a rate
 * (fm1_wifi_tx_prepare_rate; fm1_wifi_tx_prepare stays the 1 Mb/s CCK frame it always was), the MAC's legacy
 * fallback tables at JieLi's values (fm1_wifi_fallback_sdk: 0x135C whole, 0x1360's low 16 bits, its other bits kept),
 * and the station RX filter's bits. Not the radio: this checks the words and the values written.
 *
 *   cc -std=c11 -O1 -Wall -Wextra -Werror -Ichip tests/wifi_tune_test.c -o wifi_tune_test
 *   ./wifi_tune_test */
#include <stdio.h>
#include <stdint.h>

static uint32_t sim_reg[2];                       /* 0x135C, 0x1360 */
static unsigned sim_wr_n;
static uint32_t sim_rd(uint32_t a) { return a == 0x31360u ? sim_reg[1] : 0u; }
static void sim_wr(uint32_t a, uint32_t v)
{
    if (a == 0x3135Cu)
        sim_reg[0] = v;
    else if (a == 0x31360u)
        sim_reg[1] = v;
    sim_wr_n++;
}
#define FM1_WIFI_RD(a) sim_rd((uint32_t)(a))
#define FM1_WIFI_WR(a, v) sim_wr((uint32_t)(a), (uint32_t)(v))
#include "fm1_wifi.h"

static int fails;
static void want(const char *what, int ok)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    fails += !ok;
}

int main(void)
{
    uint32_t b[6];

    want("CCK 1M is the SDK's 0xB0 alone", FM1_WIFI_RATE_CCK1 == 0xB0u);
    want("CCK 2M: MCS 1, PHYMODE CCK", FM1_WIFI_RATE_CCK2 == 0x000100B0u);
    want("OFDM 6M: MCS 0, PHYMODE OFDM (bit 30)", FM1_WIFI_RATE_OFDM6 == 0x400000B0u);
    want("MCS is masked to 7 bits, PHYMODE to 2", FM1_WIFI_TXWI(5, 0xFF) == (0xB0u | 0x7Fu << 16 | 1u << 30));

    fm1_wifi_tx_prepare_rate(b, 100u, 3u, 1u, FM1_WIFI_RATE_OFDM6);
    want("tx_prepare_rate: length + 22, the TXWI size, the rate in word 2",
         b[0] == 122u && b[1] == 4u && b[2] == FM1_WIFI_RATE_OFDM6);
    want("tx_prepare_rate: ack, 13, the WCID, the MPDU length in word 3", b[3] == (1u | 13u << 2 | 3u << 8 | 100u << 16));
    want("tx_prepare_rate: words 4 and 5 zero", b[4] == 0u && b[5] == 0u);
    fm1_wifi_tx_prepare(b, 100u, 3u, 1u);
    want("tx_prepare is the CCK 1M frame as before", b[2] == 0xB0u && b[0] == 122u && b[3] == (1u | 13u << 2 | 3u << 8 | 100u << 16));
    fm1_wifi_tx_prepare_rate(b, 0x1234u, 0x1FFu, 2u, FM1_WIFI_RATE_CCK2);
    want("tx_prepare_rate: ack and the WCID masked, the length to 12 bits",
         b[3] == (0u | 13u << 2 | 0xFFu << 8 | 0x234u << 16));

    sim_reg[1] = 0xABCD5555u;
    sim_wr_n = 0;
    fm1_wifi_fallback_sdk();
    want("fallback: 0x135C is JieLi's 0xEDCBA988", sim_reg[0] == 0xEDCBA988u);
    want("fallback: 0x1360's low 16 bits are 0x2100, the upper 16 kept", sim_reg[1] == 0xABCD2100u);
    want("fallback: two writes, nothing else", sim_wr_n == 2u);

    want("the station RX filter keeps other networks' frames (bit 3), multicast (5), broadcast (6) and BAR (15)",
         (FM1_WIFI_RX_FILTER_STA & (1u << 3 | 1u << 5 | 1u << 6 | 1u << 15)) == 0u);
    want("the station RX filter drops a bad CRC (0), PHY errors (1), other addresses (2), a wrong version (4), duplicates (7)",
         (FM1_WIFI_RX_FILTER_STA & (1u << 0 | 1u << 1 | 1u << 2 | 1u << 4 | 1u << 7)) == (1u << 0 | 1u << 1 | 1u << 2 | 1u << 4 | 1u << 7));
    want("the AGC's entry is window C's 44, started at 66", FM1_WIFI_AGC_ENTRY == 44u && FM1_WIFI_AGC_START == 66u);
    printf(fails ? "wifi_tune: %d FAILED\n" : "wifi_tune: all passed\n", fails);
    return fails != 0;
}
