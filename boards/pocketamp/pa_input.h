/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Pocket AMP buttons: six lines read directly, no matrix.
 *   bit 0 PH6, bit 1 PH8, bit 2 PH9, bit 3 PA0, bit 4 PA1, bit 5 PA3
 *
 * Checked on a unit (2026-11-01): low = pressed, with pull-ups. Left to right on the panel: PH6 the parameter
 * knob's push, PH8 HOME, PH9 VOL-, PA0 VOL+, PA1 LEFT, PA3 RIGHT. The knob's encoder is on PC0 / PC1 (both high at a
 * detent; the application reads it, with pull-ups on: pa_audio.h pa_board_inputs).
 *
 *   pa_input_init()
 *   every ~10 ms: pa_input_poll();  then pa_in.down (bit set = held), pa_input_edges(&pressed, &released)
 * Debounce: a change counts after PA_DEB_TICKS polls in a row agree.
 */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"
#include "fm1_gpio.h"

#define PA_KEYS 6
#define PA_KEY_PA_MASK ((1u << 0) | (1u << 1) | (1u << 3))
#define PA_KEY_PH_MASK ((1u << 6) | (1u << 8) | (1u << 9))
#ifndef PA_DEB_TICKS
#define PA_DEB_TICKS 3u
#endif

typedef struct {
    uint8_t down, raw, count, pressed, released;
} pa_input_t;

static pa_input_t pa_in;

FM1_INLINE void pa_input_init(void)
{
    FM1_PR(FM1_PA, FM1_DIR) |= PA_KEY_PA_MASK;
    FM1_PR(FM1_PA, FM1_DIE) |= PA_KEY_PA_MASK;
    FM1_PR(FM1_PA, FM1_PU) |= PA_KEY_PA_MASK;
    FM1_PR(FM1_PH, FM1_DIR) |= PA_KEY_PH_MASK;
    FM1_PR(FM1_PH, FM1_DIE) |= PA_KEY_PH_MASK;
    FM1_PR(FM1_PH, FM1_PU) |= PA_KEY_PH_MASK;
    pa_in.down = pa_in.raw = 0;
}

FM1_INLINE uint8_t pa_input_read(void)   /* bit set = pressed (low) */
{
    uint32_t a = ~FM1_PR(FM1_PA, FM1_IN), h = ~FM1_PR(FM1_PH, FM1_IN);
    return (uint8_t)(((h >> 6) & 1u) | ((h >> 7) & 2u) | ((h >> 7) & 4u) | ((a << 3) & 8u) | ((a << 3) & 16u) |
                     ((a << 2) & 32u));
}

FM1_INLINE void pa_input_poll(void)
{
    uint8_t r = pa_input_read();
    if (r != pa_in.raw) {
        pa_in.raw = r;
        pa_in.count = 0;
        return;
    }
    if (pa_in.count < PA_DEB_TICKS && ++pa_in.count == PA_DEB_TICKS && r != pa_in.down) {
        pa_in.pressed |= (uint8_t)(r & ~pa_in.down);
        pa_in.released |= (uint8_t)(~r & pa_in.down);
        pa_in.down = r;
    }
}

FM1_INLINE void pa_input_edges(uint8_t *pressed, uint8_t *released)
{
    *pressed = pa_in.pressed;
    *released = pa_in.released;
    pa_in.pressed = pa_in.released = 0;
}
