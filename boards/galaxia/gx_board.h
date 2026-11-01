/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 David Hilowitz */
/* M-VAVE Galaxia (board PD49 V03) for felucca-bsp. The Galaxia is a guitar pedal on the same JieLi chip family as
 * the FM-1. GalaxiaSDK (https://github.com/dhilowitz/GalaxiaSDK) builds its firmware on this file; its runtime
 * (runtime/gx_runtime.c) shows the audio interrupt and the 10 kHz scan wired up. Every pin here was checked on a
 * real unit (2026-10-30, with test firmware built on the stock image's own settings):
 *
 *   audio      ALNK0 on the port A group (IOMAP_CON5 bits 6-7 = 1): PA04 MCLK, PA05 SCLK, PA06 LRCLK,
 *              lane 0 = PA07 transmit to the converter's DAC, lane 1 = PA08 receive from its ADC;
 *              PA09 holds the converter in reset until the clocks run. 11.2896 MHz MCLK, 44,118 Hz.
 *   knobs      two rows of three on the top panel: top-left, selector, top-right over bottom-left,
 *              bottom-middle, bottom-right. SARADC channels 4..9: bottom-left, top-left, bottom-right,
 *              top-right, the selector, bottom-middle (pins PB06, PB07, PC00, PC01, PC09/PC10);
 *              every knob reads 0 fully anticlockwise, 1023 fully clockwise
 *   selector   nine detents, unevenly spaced in its reading: gx_selector_position()
 *   switches   left footswitch PA01, right footswitch PA00: low while pressed
 *   jacks      input jack detects: left PC05, right PC06, low with a plug in (outputs have none)
 *   LED        two colours, active low: PA02 red, PA03 green (both: yellow)
 *
 *   gx_board_init()            pins: knobs analog, switches and jacks pulled up, LED off
 *   gx_audio_init(tx, rx, half_words, isr, prio)
 *                              converter reset, ALNK0 on port A, both lanes double-buffered:
 *                              tx and rx each hold two halves of half_words int32 (L, R; signed 24-bit)
 *   in the ISR: p = gx_audio_pending(); gx_audio_ack(p);
 *               if (p & GX_AUDIO_RX) { h = gx_audio_free_half(); rx half h in, tx half h out }
 *   gx_scan_tick()             10 kHz (TIMER5): debounce switches and jacks, step the knob conversions
 *   gx_led(red, green)
 *   gx_selector_position(raw, current)   the selector's detent, 0..8 (raw: its 0..1023 reading)
 *   gx_knob_steady(&held, raw)           a knob's value with hysteresis: steady while the knob is left alone
 */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"
#include "fm1_time.h"
#include "fm1_irq.h"
#include "fm1_gpio.h"
#include "fm1_adc.h"

/* ---- pins ---- */
#define GX_PA_SWITCHES ((1u << 0) | (1u << 1))
#define GX_PA_LEDS ((1u << 2) | (1u << 3))
#define GX_PA_AUDIO_OUT ((1u << 4) | (1u << 5) | (1u << 6) | (1u << 7))
#define GX_PA_AUDIO_IN (1u << 8)
#define GX_PA_CODEC_RESET (1u << 9)
#define GX_PB_ANALOG ((1u << 6) | (1u << 7))
#define GX_PC_ANALOG ((1u << 0) | (1u << 1) | (1u << 9) | (1u << 10))
#define GX_PC_JACKS ((1u << 5) | (1u << 6))

enum { GX_KNOB_LEFT_BOTTOM, GX_KNOB_LEFT_TOP, GX_KNOB_RIGHT_BOTTOM, GX_KNOB_RIGHT_TOP,
       GX_KNOB_SELECTOR, GX_KNOB_MIDDLE_BOTTOM, GX_KNOBS };
#define GX_KNOB_ADC(i) (4u + (i))
enum { GX_SW_LEFT = 1, GX_SW_RIGHT = 2 };          /* gx_in.switches bits */
enum { GX_JACK_LEFT = 1, GX_JACK_RIGHT = 2 };      /* gx_in.jacks bits */

#define GX_DEBOUNCE 200u                           /* scans: 20 ms at 10 kHz */

static struct {
    volatile uint16_t knob[GX_KNOBS];              /* 0..1023 */
    volatile uint8_t switches, jacks;              /* debounced, 1 = pressed / plugged */
    uint8_t count[4];
    uint8_t adc_ch, adc_busy;
    volatile uint32_t adc_rounds, adc_timeouts;
    uint16_t adc_wait;
} gx_in;

static void gx__pins(uint32_t port, uint32_t mask, int analog)
{
    if (analog) {
        FM1_PR(port, FM1_DIE) &= ~mask;
        FM1_PR(port, FM1_PU) &= ~mask;
    } else {
        FM1_PR(port, FM1_DIE) |= mask;
        FM1_PR(port, FM1_PU) |= mask;
    }
    FM1_PR(port, FM1_PD) &= ~mask;
    FM1_PR(port, FM1_DIR) |= mask;
}

static void gx_led(int red, int green)
{
    uint32_t out = FM1_PR(FM1_PA, FM1_OUT) | GX_PA_LEDS;
    if (red)
        out &= ~(1u << 2);
    if (green)
        out &= ~(1u << 3);
    FM1_PR(FM1_PA, FM1_OUT) = out;
}

/* A knob's reading wanders by a few counts with nobody touching it. gx_knob_steady(&held, raw) moves the value
 * it keeps in held only when the reading leaves it by more than that (GX_KNOB_HOLD counts), and returns it:
 * steady when the knob is left alone, still every value when it turns, and the ends reachable. It matters
 * wherever a value glides or is heard as a pitch (a delay time follows a jittering knob as a warble). */
#define GX_KNOB_HOLD 4u
static uint32_t gx_knob_steady(uint32_t *held, uint32_t raw)
{
    if (raw > *held + GX_KNOB_HOLD || raw + GX_KNOB_HOLD < *held)
        *held = raw;
    else if (raw <= GX_KNOB_HOLD)
        *held = 0;
    else if (raw >= 1023u - GX_KNOB_HOLD)
        *held = 1023u;
    return *held;
}

/* The selector's reading at each of its nine detents, left to right (measured 2026-10-31): closer
 * together at the ends. gx_selector_position(raw, current) gives the detent, 0..8; it leaves the current
 * one only a margin past the midpoint to a neighbour, so noise cannot make it flicker. */
#define GX_SELECTOR_POSITIONS 9u
static const uint16_t gx_selector_detent[GX_SELECTOR_POSITIONS] = {0, 72, 238, 392, 524, 663, 807, 974, 1023};

static uint32_t gx_selector_position(uint32_t raw, uint32_t current)
{
    const uint32_t margin = 8u;
    uint32_t p = current < GX_SELECTOR_POSITIONS ? current : 0u;
    while (p + 1u < GX_SELECTOR_POSITIONS &&
           raw > (gx_selector_detent[p] + gx_selector_detent[p + 1u]) / 2u + margin)
        p++;
    while (p > 0u && raw + margin < (gx_selector_detent[p - 1u] + gx_selector_detent[p]) / 2u)
        p--;
    return p;
}

static void gx_board_init(void)
{
    uint32_t a, c;
    gx__pins(FM1_PB, GX_PB_ANALOG, 1);
    gx__pins(FM1_PC, GX_PC_ANALOG, 1);
    gx__pins(FM1_PA, GX_PA_SWITCHES, 0);
    gx__pins(FM1_PC, GX_PC_JACKS, 0);
    FM1_PR(FM1_PA, FM1_DIE) |= GX_PA_LEDS;
    FM1_PR(FM1_PA, FM1_OUT) |= GX_PA_LEDS;         /* active low: off */
    FM1_PR(FM1_PA, FM1_DIR) &= ~GX_PA_LEDS;
    fm1_delay_ms(1);
    a = FM1_PR(FM1_PA, FM1_IN);
    c = FM1_PR(FM1_PC, FM1_IN);
    gx_in.switches = (uint8_t)(((a >> 1) & 1u ? 0u : GX_SW_LEFT) | (a & 1u ? 0u : GX_SW_RIGHT));
    gx_in.jacks = (uint8_t)(((c >> 5) & 1u ? 0u : GX_JACK_LEFT) | ((c >> 6) & 1u ? 0u : GX_JACK_RIGHT));
}

/* one active-low input, debounced into bit `bit` of *state */
static void gx__debounce(volatile uint8_t *state, uint32_t bit, uint32_t level, uint32_t slot)
{
    uint32_t active = !level, was = (*state & bit) != 0;
    if (active == was) {
        gx_in.count[slot] = 0;
        return;
    }
    if (++gx_in.count[slot] >= GX_DEBOUNCE) {
        gx_in.count[slot] = 0;
        *state ^= (uint8_t)bit;
    }
}

/* 10 kHz: switches and jacks, and one step of the knob conversions (the SDK's ADC timing; a
 * conversion spans a few ticks, the six knobs come round about every 2 ms) */
static void gx_scan_tick(void)
{
    uint32_t a = FM1_PR(FM1_PA, FM1_IN), c = FM1_PR(FM1_PC, FM1_IN);
    gx__debounce(&gx_in.switches, GX_SW_LEFT, (a >> 1) & 1u, 0);
    gx__debounce(&gx_in.switches, GX_SW_RIGHT, a & 1u, 1);
    gx__debounce(&gx_in.jacks, GX_JACK_LEFT, (c >> 5) & 1u, 2);
    gx__debounce(&gx_in.jacks, GX_JACK_RIGHT, (c >> 6) & 1u, 3);
    if (gx_in.adc_busy) {
        if (fm1_adc_done()) {
            gx_in.knob[gx_in.adc_ch] = (uint16_t)fm1_adc_take();
            gx_in.adc_busy = 0;
            if (++gx_in.adc_ch >= GX_KNOBS) {
                gx_in.adc_ch = 0;
                gx_in.adc_rounds++;
            }
        } else if (++gx_in.adc_wait > 50u) {      /* 5 ms: give up on this one */
            fm1_adc_abort();
            gx_in.adc_busy = 0;
            gx_in.adc_timeouts++;
            if (++gx_in.adc_ch >= GX_KNOBS)
                gx_in.adc_ch = 0;
        }
        return;
    }
    gx_in.adc_wait = 0;
    fm1_adc_start(GX_KNOB_ADC(gx_in.adc_ch), FM1_ADC_CON_SDK);
    gx_in.adc_busy = 1;
}

/* ---- audio: ALNK0, port A group, lane 0 out, lane 1 in ---- */
#define GX_ALNK 0x12E00u
#define GX_ALNK_CON0 (*(volatile uint16_t *)(GX_ALNK + 0x00u))
#define GX_ALNK_CON1 (*(volatile uint16_t *)(GX_ALNK + 0x04u))
#define GX_ALNK_CON2 (*(volatile uint8_t *)(GX_ALNK + 0x08u))
#define GX_ALNK_CON3 (*(volatile uint8_t *)(GX_ALNK + 0x0Cu))
#define GX_ALNK_ADR0 (*(volatile uint32_t *)(GX_ALNK + 0x10u))
#define GX_ALNK_ADR1 (*(volatile uint32_t *)(GX_ALNK + 0x14u))
#define GX_ALNK_LEN (*(volatile uint16_t *)(GX_ALNK + 0x20u))
#define GX_CLK_CON2 (*(volatile uint32_t *)0x10014u)
#define GX_IOMAP_CON5 (*(volatile uint32_t *)0x51030u)
#define GX_AUDIO_TX 1u          /* pending bits (CON2 >> 4): lane 0 */
#define GX_AUDIO_RX 2u          /* lane 1: a received half is ready, the matching transmit half is free */
#define GX_AUDIO_RATE 44118u    /* 11.2896 MHz / 256, as the stock firmware's clock trim measures it */

FM1_INLINE uint8_t gx_audio_pending(void) { return (uint8_t)((GX_ALNK_CON2 >> 4) & 15u); }
FM1_INLINE void gx_audio_ack(uint8_t p) { GX_ALNK_CON2 = (uint8_t)(p & 15u); }
/* CON0 bits 12..15: each lane's current DMA half (the FM-1's lane 3 is bit 15); lane 0 is ours */
FM1_INLINE uint32_t gx_audio_free_half(void) { return ((GX_ALNK_CON0 >> 12) & 1u) ^ 1u; }
FM1_INLINE void gx_audio_stop(void) { GX_ALNK_CON0 &= ~0x800u; }

/* The register sequence of the stock Galaxia_002 firmware (its 0x2F2E..0x30D0), with the lane
 * set-up it reads from its table at 0x0202BD74 (lane 0 transmit, lane 1 receive). */
static void gx_audio_init(int32_t *tx, int32_t *rx, uint32_t half_words, void (*isr)(void), uint32_t prio)
{
    FM1_PR(FM1_PA, FM1_DIE) |= GX_PA_CODEC_RESET;  /* converter held in reset meanwhile */
    FM1_PR(FM1_PA, FM1_OUT) &= ~GX_PA_CODEC_RESET;
    FM1_PR(FM1_PA, FM1_DIR) &= ~GX_PA_CODEC_RESET;
    fm1_delay_ms(10);
    GX_ALNK_CON0 = 0;
    GX_ALNK_CON1 = 0;
    GX_ALNK_CON2 = 0;
    GX_ALNK_CON3 = 0;
    GX_IOMAP_CON5 = (GX_IOMAP_CON5 & ~0xC0u) | 0x40u;   /* the port A group */
    FM1_PR(FM1_PA, FM1_OUT) |= GX_PA_AUDIO_OUT;
    FM1_PR(FM1_PA, FM1_DIE) |= GX_PA_AUDIO_OUT | GX_PA_AUDIO_IN;
    FM1_PR(FM1_PA, FM1_DIR) = (FM1_PR(FM1_PA, FM1_DIR) & ~GX_PA_AUDIO_OUT) | GX_PA_AUDIO_IN;
    FM1_PR(FM1_PA, FM1_PD) &= ~GX_PA_AUDIO_IN;
    FM1_PR(FM1_PA, FM1_PU) |= GX_PA_AUDIO_IN;
    GX_CLK_CON2 &= ~0xF00u;                        /* audio clock 11.2896 MHz (256 fs) */
    GX_ALNK_CON3 = (uint8_t)(3u | (4u << 5));
    GX_ALNK_CON0 = (uint16_t)((1u << 8) | (1u << 7));
    GX_ALNK_LEN = (uint16_t)half_words;
    GX_ALNK_ADR0 = (uint32_t)(uintptr_t)tx;
    GX_ALNK_ADR1 = (uint32_t)(uintptr_t)rx;
    GX_ALNK_CON1 = (uint16_t)(0x5u | (0xDu << 4)); /* lane 0: on, 24-bit; lane 1: on, 24-bit, receive */
    GX_ALNK_CON2 = 0x0Fu;
    fm1_irq_attach(FM1_IRQ_ALNK0, isr, prio);
    GX_ALNK_CON0 |= 0x800u;
    fm1_delay_ms(10);
    FM1_PR(FM1_PA, FM1_OUT) |= GX_PA_CODEC_RESET;  /* release the converter */
}
