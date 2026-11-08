/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Pocket AMP panel init: the FM-1's ST7789-class sequence with display-off first and MADCTL 0x60.
 * Checked on a Pocket AMP with examples/pa-hwtest in pendel-bsp.
 *
 * Each record: command, data length, data. PA_LCD_INIT_DELAY marks the 120 ms wait after sleep-out.
 * Display-on (0x29) is not in the table: send it after the first frame. */
#pragma once
#include <stdint.h>

#define PA_LCD_INIT_DELAY 0xFFu

typedef struct {
    uint8_t cmd, len;
    uint8_t data[14];
} pa_lcd_init_t;

static const pa_lcd_init_t PA_LCD_INIT[] = {
    { 0x28, 0, { 0 } },                                     /* display off */
    { 0x11, 0, { 0 } },                                     /* sleep out */
    { PA_LCD_INIT_DELAY, 120, { 0 } },
    { 0x2A, 4, { 0x00, 0x00, 0x00, 0xEF } },
    { 0x2B, 4, { 0x00, 0x28, 0x01, 0x17 } },
    { 0xB2, 5, { 0x0C, 0x0C, 0x0C, 0x00, 0x33 } },
    { 0x20, 0, { 0 } },                                     /* inversion off */
    { 0xB7, 1, { 0x56 } },
    { 0xBB, 1, { 0x18 } },
    { 0xC0, 1, { 0x2C } },
    { 0xC2, 1, { 0x01 } },
    { 0xC3, 1, { 0x1F } },
    { 0xC4, 1, { 0x20 } },
    { 0xC6, 1, { 0x0F } },
    { 0xD0, 2, { 0xA6, 0xA1 } },
    { 0xE0, 14, { 0xD0, 0x0D, 0x14, 0x0B, 0x0B, 0x07, 0x3A, 0x44, 0x50, 0x08, 0x13, 0x13, 0x2D, 0x32 } },
    { 0xE1, 14, { 0xD0, 0x0D, 0x14, 0x0B, 0x0B, 0x07, 0x3A, 0x44, 0x50, 0x08, 0x13, 0x13, 0x2D, 0x32 } },
    { 0x36, 1, { 0x60 } },                                  /* MADCTL: a quarter turn from the FM-1 */
    { 0x3A, 1, { 0x55 } },                                  /* RGB565 */
    { 0xE7, 1, { 0x00 } },
    { 0x51, 1, { 0xFF } },                                  /* brightness */
    { 0x21, 0, { 0 } },                                     /* inversion on */
};
