/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* FM-1 ST7789 panel: blocking, top-left origin, native RGB565 colours.
 * No framebuffer required; 480 bytes of shared RAM stage each pixel row.
 * Calls must be serialized (including against users of fm1_lcd_hw.h).
 * 0 = success (also empty rectangles), -1 = SPI timeout, -2 = bad bounds/buffer.
 * A timeout deselects and disables SPI; call fm1_lcd_init() before retrying.
 * See docs/BOARD_IO.md for ownership, lifetime and backlight details. */
#pragma once
#include <stdint.h>
#include "fm1_time.h"
#include "fm1_lcd_hw.h"

#define FM1_LCD_WIDTH 240u
#define FM1_LCD_HEIGHT 240u
static uint8_t fm1_lcd_row[FM1_LCD_WIDTH * 2u];

static int fm1_lcd_command(uint8_t cmd, const uint8_t *data, uint32_t n)
{
    static uint8_t args[4];                    /* DMA reads RAM, not XIP constants */
    uint32_t i;
    int rc;
    if (n > sizeof args || (n && !data))
        return -2;
    fm1_lcd_send_cmd(cmd);
    rc = fm1_lcd_wait();
    fm1_lcd_deselect();
    if (rc || !n)
        return rc;
    for (i = 0; i < n; i++)
        args[i] = data[i];
    fm1_lcd_send_data(args, n);
    rc = fm1_lcd_wait();
    fm1_lcd_deselect();
    return rc;
}

static int fm1_lcd_init(void)
{
    static const uint8_t colmod = 0x55, madctl = 0;
    fm1_lcd_hw_init();
    fm1_delay_ms(50);
    if (fm1_lcd_command(0x01, 0, 0)) return -1; /* SWRESET */
    fm1_delay_ms(150);
    if (fm1_lcd_command(0x11, 0, 0)) return -1; /* SLPOUT */
    fm1_delay_ms(120);
    if (fm1_lcd_command(0x3A, &colmod, 1) ||    /* RGB565 */
        fm1_lcd_command(0x36, &madctl, 1) ||    /* top-left, RGB */
        fm1_lcd_command(0x21, 0, 0) ||         /* IPS inversion */
        fm1_lcd_command(0x13, 0, 0) ||         /* normal mode */
        fm1_lcd_command(0x29, 0, 0))           /* display on */
        return -1;
    fm1_lcd_baud(4);
    return 0;
}

FM1_INLINE int fm1_lcd_bounds(uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
    return x < FM1_LCD_WIDTH && y < FM1_LCD_HEIGHT &&
           w <= FM1_LCD_WIDTH - x && h <= FM1_LCD_HEIGHT - y;
}

static int fm1_lcd_window(uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
    uint8_t a[4] = { 0, (uint8_t)x, 0, (uint8_t)(x + w - 1u) };
    if (fm1_lcd_command(0x2A, a, 4)) return -1;
    a[1] = (uint8_t)y;
    a[3] = (uint8_t)(y + h - 1u);
    if (fm1_lcd_command(0x2B, a, 4)) return -1;
    return fm1_lcd_command(0x2C, 0, 0);
}

static int fm1_lcd_row_send(uint32_t w)
{
    int rc;
    fm1_lcd_send_data(fm1_lcd_row, w * 2u);
    rc = fm1_lcd_wait();
    fm1_lcd_deselect();
    return rc;
}

static int fm1_lcd_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t rgb)
{
    uint32_t i;
    if (!w || !h) return 0;
    if (!fm1_lcd_bounds(x, y, w, h)) return -2;
    if (fm1_lcd_window(x, y, w, h)) return -1;
    for (i = 0; i < w; i++) {
        fm1_lcd_row[2u * i] = (uint8_t)(rgb >> 8);
        fm1_lcd_row[2u * i + 1u] = (uint8_t)rgb;
    }
    for (i = 0; i < h; i++)
        if (fm1_lcd_row_send(w)) return -1;
    return 0;
}

/* Contiguous row-major native uint16_t RGB565, in RAM or XIP; pixel_count
 * is the caller's available element count. Source may be reused on return. */
static int fm1_lcd_blit(uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                        const uint16_t *pixels, uint32_t pixel_count)
{
    uint32_t row, col;
    if (!w || !h) return 0;
    if (!fm1_lcd_bounds(x, y, w, h) || !pixels || pixel_count < w * h)
        return -2;
    if (fm1_lcd_window(x, y, w, h)) return -1;
    for (row = 0; row < h; row++) {
        for (col = 0; col < w; col++) {
            uint16_t rgb = pixels[row * w + col];
            fm1_lcd_row[2u * col] = (uint8_t)(rgb >> 8);
            fm1_lcd_row[2u * col + 1u] = (uint8_t)rgb;
        }
        if (fm1_lcd_row_send(w)) return -1;
    }
    return 0;
}
