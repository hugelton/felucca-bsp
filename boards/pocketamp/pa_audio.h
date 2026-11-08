/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Pocket AMP audio: ALNK0 I2S on port A, full duplex, to an external converter with no control bus.
 *   PA4 MCLK, PA5 SCLK, PA6 LRCK, PA7 DAT0 (out, channel 0), PA8 DAT1 (in, channel 1)
 * The FM-1 uses the same block on port C, output only (boards/fm1/fm1_audio.h); this adds the input.
 *
 * Channel modes: CON1 nibbles 0x5 out / 0xD in.
 *
 * Before the link: PC5..PC9 are driven high (probably the five button LEDs); PA2 is low while the link starts and
 * high after it (an enable or mute of the amplifier or the converter);
 * PC0, PC1, PA10 and PC10 are inputs (PC0 / PC1 the encoder, PC10 the battery): pa_board_inputs() reports them.
 * Samples are 24-bit, right-aligned in int32. Output as FM-1's (Q15 << 7, -6 dBFS). The input (verified 2026-11-01)
 * arrives 24-bit right-aligned and NOT sign-extended (bits 24..31 zero): take it as (int32_t)(x << 8) >> 8.
 *
 * The application owns two double buffers (out and in), each two halves of half_words int32, zeroed:
 *   pa_audio_init(out, in, half_words, isr, prio)
 *   in the ISR:  p = pa_audio_pending();
 *                if (p & PA_AUDIO_IN) pa_audio_ack_in();
 *                if (p & PA_AUDIO_OUT) { h = pa_audio_free_half(); read in[h], fill out[h]; pa_audio_ack_out(); }
 *   pa_audio_stop()
 * Verified 2026-11-01 on a unit: the link runs, one interrupt per channel per half (44118 / 64 * 2 a
 * second). CON2's pending bits are per channel, 0x10 << ch, cleared by writing 1 << ch; CON0 bit 12 + ch is the
 * half the channel's DMA is in. (FM-1 uses channel 3 only: its "half" bit 0x80 is channel 3's pending.)
 */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"
#include "fm1_time.h"
#include "fm1_irq.h"
#include "fm1_gpio.h"

#define PA_ALNK 0x12E00u
#define PA_ALNK_CON0 (*(volatile uint16_t *)(PA_ALNK + 0x00u))
#define PA_ALNK_CON1 (*(volatile uint16_t *)(PA_ALNK + 0x04u))
#define PA_ALNK_CON2 (*(volatile uint8_t *)(PA_ALNK + 0x08u))
#define PA_ALNK_CON3 (*(volatile uint8_t *)(PA_ALNK + 0x0Cu))
#define PA_ALNK_ADR0 (*(volatile uint32_t *)(PA_ALNK + 0x10u))
#define PA_ALNK_ADR1 (*(volatile uint32_t *)(PA_ALNK + 0x14u))
#define PA_ALNK_LEN (*(volatile uint16_t *)(PA_ALNK + 0x20u))
#define PA_CLK_CON2 (*(volatile uint32_t *)0x10014u)
#define PA_IOMAP_CON5 (*(volatile uint32_t *)0x51030u)
#define PA_AUDIO_OUT 0x10u                 /* channel 0 (out) finished a half */
#define PA_AUDIO_IN 0x20u                  /* channel 1 (in) */

static void pa__pa_out(uint32_t bit, int v)
{
    uint32_t m = 1u << bit;
    FM1_PR(FM1_PA, FM1_DIE) |= m;
    if (v)
        FM1_PR(FM1_PA, FM1_OUT) |= m;
    else
        FM1_PR(FM1_PA, FM1_OUT) &= ~m;
    FM1_PR(FM1_PA, FM1_DIR) &= ~m;
}

static void pa__pa_in(uint32_t bit)
{
    uint32_t m = 1u << bit;
    FM1_PR(FM1_PA, FM1_DIR) |= m;
    FM1_PR(FM1_PA, FM1_DIE) |= m;
    FM1_PR(FM1_PA, FM1_PU) &= ~m;
    FM1_PR(FM1_PA, FM1_PD) &= ~m;
}

static void pa__pc_out(uint32_t bit, int v)
{
    uint32_t m = 1u << bit;
    FM1_PR(FM1_PC, FM1_DIE) |= m;
    if (v)
        FM1_PR(FM1_PC, FM1_OUT) |= m;
    else
        FM1_PR(FM1_PC, FM1_OUT) &= ~m;
    FM1_PR(FM1_PC, FM1_DIR) &= ~m;
}

/* PC5..PC9 high: the enables (UNVERIFIED) */
FM1_INLINE void pa_audio_power(void)
{
    uint32_t b;
    for (b = 5; b <= 9u; b++)
        pa__pc_out(b, 1);
}

/* bit 0 PC0, 1 PC1, 2 PA10, 3 PC10: inputs, no pulls (the encoder's pull-ups are the application's to set) */
FM1_INLINE uint32_t pa_board_inputs(void)
{
    uint32_t pc = (1u << 0) | (1u << 1) | (1u << 10), pa = 1u << 10, ic, ia;
    FM1_PR(FM1_PC, FM1_DIR) |= pc;
    FM1_PR(FM1_PC, FM1_DIE) |= pc;
    FM1_PR(FM1_PC, FM1_PU) &= ~pc;
    FM1_PR(FM1_PC, FM1_PD) &= ~pc;
    FM1_PR(FM1_PA, FM1_DIR) |= pa;
    FM1_PR(FM1_PA, FM1_DIE) |= pa;
    FM1_PR(FM1_PA, FM1_PU) &= ~pa;
    FM1_PR(FM1_PA, FM1_PD) &= ~pa;
    ic = FM1_PR(FM1_PC, FM1_IN);
    ia = FM1_PR(FM1_PA, FM1_IN);
    return (ic & 3u) | ((ia >> 10) & 1u) << 2 | ((ic >> 10) & 1u) << 3;
}

FM1_INLINE void pa_audio_init(int32_t *out, int32_t *in, uint32_t half_words, void (*isr)(void), uint32_t prio)
{
    pa_audio_power();
    pa__pa_out(2, 0);
    fm1_delay_ms(5);
    PA_ALNK_CON0 = 0;
    PA_ALNK_CON1 = 0;
    PA_ALNK_CON2 = 0;
    PA_ALNK_CON3 = 0;
    PA_IOMAP_CON5 = (PA_IOMAP_CON5 & ~0xC0u) | 0x40u;    /* ALNK0 on port A */
    PA_ALNK_CON3 |= 3u;
    pa__pa_out(4, 1);
    pa__pa_out(6, 1);
    pa__pa_out(5, 1);
    pa__pa_in(8);
    pa__pa_out(7, 1);
    PA_ALNK_CON0 |= 0x100u;
    PA_ALNK_CON0 |= 0x80u;
    PA_ALNK_CON0 &= ~0x40u;
    PA_ALNK_LEN = (uint16_t)half_words;
    PA_ALNK_CON0 &= ~0x400u;
    PA_ALNK_CON0 &= ~0x200u;
    PA_CLK_CON2 &= 0xFFFFF0FFu;
    PA_ALNK_CON3 &= ~0x1Cu;
    PA_ALNK_CON3 = (uint8_t)((PA_ALNK_CON3 & 0x1Fu) | 0x80u);
    PA_ALNK_ADR1 = (uint32_t)(uintptr_t)in;
    PA_ALNK_CON1 |= 0x10u | 0x80u | 0x40u;               /* channel 1: in */
    PA_ALNK_ADR0 = (uint32_t)(uintptr_t)out;
    PA_ALNK_CON1 |= 0x1u;                                /* channel 0: out */
    PA_ALNK_CON1 &= ~0x8u;
    PA_ALNK_CON1 |= 0x4u;
    PA_ALNK_CON2 = 0x0Fu;
    fm1_irq_attach(FM1_IRQ_ALNK0, isr, prio);
    PA_ALNK_CON0 |= 0x800u;
    fm1_delay_ms(5);
    pa__pa_out(2, 1);
}

FM1_INLINE uint8_t pa_audio_pending(void) { return PA_ALNK_CON2; }
FM1_INLINE void pa_audio_ack_out(void) { PA_ALNK_CON2 |= 1u; }
FM1_INLINE void pa_audio_ack_in(void) { PA_ALNK_CON2 |= 2u; }
FM1_INLINE uint32_t pa_audio_free_half(void) { return ((PA_ALNK_CON0 >> 12) & 1u) ^ 1u; }   /* channel 0's */
FM1_INLINE uint32_t pa_audio_in_half(void) { return ((PA_ALNK_CON0 >> 13) & 1u) ^ 1u; }     /* channel 1's */
FM1_INLINE void pa_audio_stop(void) { PA_ALNK_CON0 &= ~0x800u; }
