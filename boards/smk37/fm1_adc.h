/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio)
 * The converter's registers and sequence are the FM-1 board's (boards/fm1/fm1_adc.h):
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * Copyright (C) 2026 Charles Vestal (conversions in steps: fm1-velocity) */
/* SMK-37 (Pro / Elite / MKE-P37) SARADC: three 8:1 analog multiplexers, their address on bits 3..5 of the 595
 * word (fm1_input.h), one converter channel each:
 *   mux A  ch1 (PA8)   pads 0..7 (the top row), by address: 1 2 3 0 4 7 5 6
 *   mux B  ch8 (PC9)   pads 8..15 (the bottom row), the same order + 8
 *   mux C  ch2 (PA10)  0..3 the faders (in the order 1 2 3 0), 4 the pitch wheel, 5 the mod wheel, 6 the sustain
 *                      pedal jack, 7 the battery divider
 * MASTER is not a pot on this board (K5 is an encoder: a virtual master, device/plat_fm1.c).
 *
 * Board facts: M. Geisler's analysis of the stock v016 firmware (fm1-to-smk37, docs/01-hardware.md): the channels
 * and the mux tables are marked verified there (on an Elite running Melodee); nothing has been checked on a unit by
 * this layer yet (fm1-base's SMK-37 port, step 2). The SDK's adc_api.h gives the same channel numbers for PA8, PA10
 * and PC9.
 *
 *   fm1_adc_init()               the three mux outputs analog inputs, no pulls
 *   fm1_adc_read(ch)             blocking, 0..1023, -1 on timeout (the SDK's clock settings)
 *   fm1_adc_start(ch, con)       start one conversion; con = FM1_ADC_CON(div, delay)
 *   fm1_adc_done()               the conversion finished
 *   fm1_adc_take()               the result, and the converter idle again
 *   fm1_adc_abort()              give up on a conversion
 *
 * One converter: fm1_input.h's tick owns it (one conversion a tick, the 24 mux inputs in turn), so the main loop
 * must not call fm1_adc_read while the scan runs. CON as the FM-1's: bits 0-2 the clock divider, bit 3 set, bit 4
 * enable, bit 5 interrupt enable, bit 6 start / clear pending, bit 7 pending, bits 8-11 the channel, bits 12-15 the
 * start delay in units of 8 ADC clocks. */
#pragma once
#include <stdint.h>
#include "fm1_gpio.h"

#define FM1_ADC_CON_R (*(volatile uint32_t *)0x13100u)
#define FM1_ADC_RES (*(volatile uint32_t *)0x13104u)
#define FM1_WLA_CON0 (*(volatile uint32_t *)0x11900u)
enum { FM1_ADC_MUX_A = 1, FM1_ADC_MUX_C = 2, FM1_ADC_MUX_B = 8 };   /* the channel of each mux's output */
#define FM1_ADC_CON(div, delay) ((uint32_t)((div) & 7u) | 8u | ((uint32_t)((delay) & 15u) << 12))
#define FM1_ADC_CON_SDK FM1_ADC_CON(6, 15)
/* mux address (0..7) -> the pad it carries (mux A; + 8 on mux B) */
static const uint8_t FM1_ADC_PAD_OF[8] = {1, 2, 3, 0, 4, 7, 5, 6};
/* mux C: address 0..3 -> the fader it carries */
static const uint8_t FM1_ADC_FADER_OF[4] = {1, 2, 3, 0};
enum { FM1_ADC_C_PITCH = 4, FM1_ADC_C_MOD = 5, FM1_ADC_C_PEDAL = 6, FM1_ADC_C_BATT = 7 };

/* the mux outputs are only ever analog inputs. PB1 (the charger line, low = charging) is never driven and never
 * pulled down: held low for 8 s it would reset the chip (isd_config), as on the FM-1 */
static void fm1_adc_init(void)
{
    uint32_t a = (1u << 8) | (1u << 10), c = 1u << 9;
    FM1_PR(FM1_PA, FM1_DIE) &= ~a;
    FM1_PR(FM1_PA, FM1_PU) &= ~a;
    FM1_PR(FM1_PA, FM1_PD) &= ~a;
    FM1_PR(FM1_PA, FM1_DIR) |= a;
    FM1_PR(FM1_PC, FM1_DIE) &= ~c;
    FM1_PR(FM1_PC, FM1_PU) &= ~c;
    FM1_PR(FM1_PC, FM1_PD) &= ~c;
    FM1_PR(FM1_PC, FM1_DIR) |= c;
}

static void fm1_adc_start(uint32_t ch, uint32_t con)
{
    FM1_ADC_CON_R = 0;
    if (FM1_WLA_CON0 & (1u << 14))
        FM1_WLA_CON0 &= ~(1u << 14);
    FM1_ADC_CON_R = (con & 0xF00Fu) | 0x40u | ((ch & 0xFu) << 8);
    FM1_ADC_CON_R |= 0x10u;
    FM1_ADC_CON_R |= 0x40u;
}
static inline int fm1_adc_done(void) { return (FM1_ADC_CON_R & 0x80u) != 0; }
static inline uint32_t fm1_adc_take(void)
{
    uint32_t v = FM1_ADC_RES & 0x3FFu;
    FM1_ADC_CON_R = 0x40u;
    FM1_ADC_CON_R = 0;
    return v;
}
static inline void fm1_adc_abort(void)
{
    FM1_ADC_CON_R = 0x40u;
    FM1_ADC_CON_R = 0;
}

static int32_t fm1_adc_read(uint32_t ch)
{
    uint32_t t;
    fm1_adc_start(ch, FM1_ADC_CON_SDK);
    for (t = 0; t < 20000u; t++)
        if (fm1_adc_done())
            break;
    if (t == 20000u) {
        fm1_adc_abort();
        return -1;
    }
    return (int32_t)fm1_adc_take();
}
