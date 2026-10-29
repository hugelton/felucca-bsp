/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Henri David */
/* fm1_wifi_up (chip/fm1_wifi.h) on the host, its register reads and writes logged: the station's MAC settings in
 * fm1_wifi_sta_init's order, then both RF paths on (0x14000 |= 3, its other bits kept), then the caller's
 * coexistence ("WLC") words to 0x14004, each FM1_WIFI_WLC_WORD(hi, code), in order, and nothing else. The words here
 * are a made-up table of this test's (the real ones are JieLi's and live with the caller, fm1-base's
 * device/fm1_wifi_wlc.h, which fm1-base's tests/wifi/wifi_wlc_test.c runs through this same fm1_wifi_up).
 * Not the radio: this checks the order and the values written, not what the chip does with them.
 *
 *   cc -std=c11 -O1 -Wall -Wextra -Werror -Ichip tests/wifi_up_test.c -o wifi_up_test
 *   ./wifi_up_test */
#include <stdio.h>
#include <stdint.h>

#define SIM_MAX 1024u
static struct { uint32_t a, v, wr; } sim_log[SIM_MAX];
static unsigned sim_n, sim_over;
static uint32_t sim_wl_en;                        /* 0x14000's value (every other register reads 0) */
static uint32_t sim_rd(uint32_t a)
{
    uint32_t v = a == 0x14000u ? sim_wl_en : 0u;
    if (sim_n < SIM_MAX) {
        sim_log[sim_n].a = a, sim_log[sim_n].v = v, sim_log[sim_n].wr = 0;
        sim_n++;
    } else
        sim_over++;
    return v;
}
static void sim_wr(uint32_t a, uint32_t v)
{
    if (a == 0x14000u)
        sim_wl_en = v;
    if (sim_n < SIM_MAX) {
        sim_log[sim_n].a = a, sim_log[sim_n].v = v, sim_log[sim_n].wr = 1;
        sim_n++;
    } else
        sim_over++;
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

/* the test's own table: edge values of both halves, not JieLi's words */
static const uint16_t t_hi[] = {0x0000u, 0xFFFFu, 0x1234u, 0x8001u, 0x00FFu, 0xA5A5u};
static const uint8_t t_code[] = {0x00u, 0xFFu, 0x56u, 0x80u, 0x01u, 0x5Au};
static const uint32_t t_word[] = {0x00000001u, 0xFFFF0FF1u, 0x12340561u, 0x80010801u, 0x00FF0011u, 0xA5A505A1u};
#define T_N (sizeof t_hi / sizeof *t_hi)

static void run(uint32_t wl_en, const uint16_t *hi, const uint8_t *code, uint32_t n)
{
    sim_n = sim_over = 0;
    sim_wl_en = wl_en;
    fm1_wifi_up(hi, code, n);
}

int main(void)
{
    const unsigned nsta = (unsigned)(sizeof fm1_wifi_sta_init / sizeof *fm1_wifi_sta_init);
    unsigned i, bad = 0;

    run(0x1u, t_hi, t_code, (uint32_t)T_N);       /* BT's path on, as the Bluetooth start-up leaves it */
    want("as many accesses as STA settings + read and write of 0x14000 + words", sim_n == nsta + 2u + T_N && !sim_over);
    for (i = 0; i < nsta && i < sim_n; i++)
        bad += !sim_log[i].wr || sim_log[i].a != FM1_WIFI_MAC + fm1_wifi_sta_init[i][0] ||
               sim_log[i].v != fm1_wifi_sta_init[i][1];
    want("the STA settings first, to 0x30000 + r, in their order", bad == 0u && nsta > 0u);
    want("then 0x14000 read", sim_n > nsta && !sim_log[nsta].wr && sim_log[nsta].a == FM1_WL_EN);
    want("then 0x14000 written with both RF paths on (1 | 3 = 3)",
         sim_n > nsta + 1u && sim_log[nsta + 1u].wr && sim_log[nsta + 1u].a == FM1_WL_EN && sim_log[nsta + 1u].v == 3u);
    for (bad = 0, i = 0; i < T_N && nsta + 2u + i < sim_n; i++)
        bad += !sim_log[nsta + 2u + i].wr || sim_log[nsta + 2u + i].a != 0x14004u || sim_log[nsta + 2u + i].v != t_word[i];
    want("then the caller's words to 0x14004, hi << 16 | code << 4 | 1, in order", bad == 0u);
    for (bad = 0, i = 0; i < T_N; i++)
        bad += FM1_WIFI_WLC_WORD(t_hi[i], t_code[i]) != t_word[i];
    want("FM1_WIFI_WLC_WORD packs as documented", bad == 0u);
    want("fm1_wifi_is_up after it", fm1_wifi_is_up());

    run(0xA5A50004u, t_hi, t_code, (uint32_t)T_N);
    want("0x14000's other bits kept (0xA5A50004 -> 0xA5A50007)",
         sim_n > nsta + 1u && sim_log[nsta + 1u].v == 0xA5A50007u);

    run(0x1u, t_hi, t_code, 0u);
    want("n = 0: no word to 0x14004", sim_n == nsta + 2u);
    for (bad = 0, i = 0; i < sim_n; i++)
        bad += sim_log[i].a == 0x14004u;
    want("n = 0: 0x14004 untouched", bad == 0u);

    run(0x1u, t_hi, t_code, 2u);
    want("n = 2: two words, the table's first two",
         sim_n == nsta + 4u && sim_log[nsta + 2u].v == t_word[0] && sim_log[nsta + 3u].v == t_word[1]);

    sim_wl_en = 1u;
    want("fm1_wifi_is_up: BT's path alone is not Wi-Fi", !fm1_wifi_is_up());
    want("FM1_WLC_PORT is 0x14004", FM1_WLC_PORT == 0x14004u);
    printf(fails ? "wifi_up: %d FAILED\n" : "wifi_up: all passed\n", fails);
    return fails != 0;
}
