/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Henri David */
/* fm1_led_dim_ns (boards/fm1/fm1_input.h) on the host, the GPIO ports a plain array: the pulse the scan's tick aims at
 * (fm1__dim_t, TIMER4 ticks) for any width from FM1_LED_DIM_MIN_NS to FM1_LED_DIM_MAX_NS, rounded as FM1__DIM_T rounds,
 * clamped at both ends; the two glows' widths give fm1_led_dim_level's own pulses; the breath's peak and the mid
 * level stay as fm1_led_dim_level left them; fm1_led_dim_level afterwards takes the pulse back; no register written.
 * Not the LEDs: how bright a pulse looks is only seen on an FM-1.
 *
 *   cc -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function -Ichip -Iboards/fm1 tests/dim_test.c -o dim_test
 *   ./dim_test */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "fm1_gpio.h"

static uint32_t sim_port[8][16], sim_zero[8][16];  /* the GPIO ports' registers (never written by what is tested) */
#undef FM1_PR
#define FM1_PR(p, r) sim_port[p][(r) / 4u]
#include "fm1_input.h"

static int fails;
static void want(const char *what, int ok)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    fails += !ok;
}
static uint32_t ticks(uint32_t ns) { return (ns * FM1_TICKS_PER_US + 500u) / 1000u; }   /* (FM1__DIM_T) */

int main(void)
{
    uint32_t ns, bad = 0, hi, lo;
    fm1_led_dim_level(0);
    hi = fm1__dim_t;
    fm1_led_dim_level(1);
    lo = fm1__dim_t;
    fm1_led_dim_ns(FM1_LED_DIM_NS);
    want("the high glow's width: fm1_led_dim_level(0)'s pulse", fm1__dim_t == hi);
    fm1_led_dim_ns(FM1_LED_DIM_LO_NS);
    want("the low glow's width: fm1_led_dim_level(1)'s pulse", fm1__dim_t == lo);
    for (ns = FM1_LED_DIM_MIN_NS; ns <= FM1_LED_DIM_MAX_NS; ns++) {
        fm1_led_dim_ns(ns);
        bad += fm1__dim_t != ticks(ns);
    }
    want("every width from the shortest to the longest, rounded to the tick", bad == 0u);
    fm1_led_dim_ns(0);
    want("under the shortest: clamped to it", fm1__dim_t == ticks(FM1_LED_DIM_MIN_NS));
    fm1_led_dim_ns(FM1_LED_DIM_MIN_NS - 1u);
    want("just under: clamped", fm1__dim_t == ticks(FM1_LED_DIM_MIN_NS));
    fm1_led_dim_ns(FM1_LED_DIM_MAX_NS + 1u);
    want("just over the longest: clamped to it", fm1__dim_t == ticks(FM1_LED_DIM_MAX_NS));
    fm1_led_dim_ns(0xFFFFFFFFu);
    want("far over: clamped, no overflow", fm1__dim_t == ticks(FM1_LED_DIM_MAX_NS));

    fm1_led_dim_level(1);                         /* the low glow: its breath and mid level */
    fm1_led_dim_ns(2000u);
    want("the breath's peak and the mid level as the glow left them", fm1__br_sel == 1u &&
                                                                      fm1__mid_n == FM1_LED_MID_N_LO);
    fm1_led_dim_level(0);
    want("fm1_led_dim_level afterwards: its own pulse again", fm1__dim_t == hi && fm1__br_sel == 0u &&
                                                               fm1__mid_n == FM1_LED_MID_N);
    want("no GPIO register written", !memcmp(sim_port, sim_zero, sizeof sim_port));
    printf(fails ? "dim: %d FAILED\n" : "dim: all passed\n", fails);
    return fails != 0;
}
