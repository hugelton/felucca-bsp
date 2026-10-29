/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Henri David */
/* fm1_hp_in (boards/fm1/fm1_jack.h) on the host, the GPIO ports a plain array: it reads port B's input register only,
 * bit 5, inverted (0 on the pin = headphones in = 1); the port's other bits and the other ports change nothing; no
 * register is written (direction, pull and function as the boot left them). Not the pin: its levels were read on an
 * FM-1 with a probe.
 *
 *   cc -std=c11 -O1 -Wall -Wextra -Werror -Ichip -Iboards/fm1 tests/jack_test.c -o jack_test
 *   ./jack_test */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "fm1_gpio.h"

static uint32_t sim_port[8][16];                  /* the GPIO ports' registers, 0x40 apart */
static unsigned sim_reads, sim_strays;            /* accesses; those not to PB's input register */
static volatile uint32_t *sim_reg(uint32_t p, uint32_t r)
{
    sim_reads++;
    sim_strays += p != FM1_PB || r != FM1_IN;
    return &sim_port[p][r / 4u];
}
#undef FM1_PR
#define FM1_PR(p, r) (*sim_reg((p), (r)))
#include "fm1_jack.h"

static int fails;
static void want(const char *what, int ok)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    fails += !ok;
}

int main(void)
{
    static uint32_t before[8][16];
    uint32_t v, bad = 0;
    sim_port[FM1_PB][FM1_IN / 4u] = 1u << 5;      /* the pin high: nothing plugged */
    want("PB5 high: no headphones (0)", fm1_hp_in() == 0u);
    sim_port[FM1_PB][FM1_IN / 4u] = 0u;           /* pulled low by the plug's switch */
    want("PB5 low: headphones in (1)", fm1_hp_in() == 1u);
    for (v = 0; v < 256u; v++) {                  /* PB's other bits, and every other port's input, anything */
        uint32_t noise = v * 0x01010101u & ~(1u << 5), p;
        for (p = 0; p < 8u; p++)
            sim_port[p][FM1_IN / 4u] = p == FM1_PB ? noise : ~0u;
        bad += fm1_hp_in() != 1u;
        sim_port[FM1_PB][FM1_IN / 4u] = noise | 1u << 5;
        bad += fm1_hp_in() != 0u;
    }
    want("only bit 5 counts (PB's other bits, the other ports)", bad == 0u);
    want("only PB's input register read", sim_reads > 0u && sim_strays == 0u);
    memcpy(before, sim_port, sizeof before);
    (void)fm1_hp_in();
    want("no register written", !memcmp(before, sim_port, sizeof before));
    want("FM1_HP_PIN is PB5", FM1_HP_PIN == 5u);
    printf(fails ? "jack: %d FAILED\n" : "jack: all passed\n", fails);
    return fails != 0;
}
