/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Pocket AMP LCD wiring: SPI1, CS PB1, D/C PB5, PB6/PB7 the bus side; PA9 the panel's power or backlight.
 * The panel is the same 240x240 RGB565 ST7789-class part as the FM-1's, turned a quarter (MADCTL 0x60).
 *
 * Checked on a unit (2026-11-01): the panel runs with the pins as set below (SPI1's IOMAP left at reset); PA9 is the
 * backlight, on when low (PA_LCD_PA9_ON 0).
 *
 *   pa_lcd_hw_init()          pins, PA9 on, SPI1 master
 *   pa_lcd_send_cmd(c)        D/C low, CS low, one byte (then pa_lcd_wait)
 *   pa_lcd_send_data(p, n)    D/C high, CS low, DMA n bytes from RAM p
 *   pa_lcd_wait()             SPI done (or timeout, counted), pending cleared
 *   pa_lcd_deselect()         CS high
 *   pa_lcd_power(on)          PA9 (PA_LCD_PA9_ON is the level taken as on)
 */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"
#include "fm1_gpio.h"

#define PA_LCD_SPI_CON (*(volatile uint32_t *)0x11D00u)
#define PA_LCD_SPI_BAUD (*(volatile uint32_t *)0x11D04u)
#define PA_LCD_SPI_BUF (*(volatile uint32_t *)0x11D08u)
#define PA_LCD_SPI_ADR (*(volatile uint32_t *)0x11D0Cu)
#define PA_LCD_SPI_CNT (*(volatile uint32_t *)0x11D10u)
#define PA_LCD_CS (1u << 1)            /* PB1 */
#define PA_LCD_DC (1u << 5)            /* PB5 */
#define PA_LCD_B6 (1u << 6)            /* PB6: output */
#define PA_LCD_B7 (1u << 7)            /* PB7: input */
#define PA_LCD_PWR (1u << 9)           /* PA9 */
#ifndef PA_LCD_PA9_ON
#define PA_LCD_PA9_ON 0                /* verified 2026-11-01: the panel lights with PA9 low */
#endif
#ifndef PA_LCD_BAUD
#define PA_LCD_BAUD 1u                 /* SPI1 clock = lsb / (BAUD + 1); 0 may work, start one slower */
#endif
#define PA_LCD_MADCTL 0x60u

static uint32_t pa_lcd_timeouts;

FM1_INLINE void pa_lcd_power(uint32_t on)
{
    if (!on == !PA_LCD_PA9_ON)
        FM1_PR(FM1_PA, FM1_OUT) |= PA_LCD_PWR;
    else
        FM1_PR(FM1_PA, FM1_OUT) &= ~PA_LCD_PWR;
    FM1_PR(FM1_PA, FM1_DIR) &= ~PA_LCD_PWR;
}

FM1_INLINE void pa_lcd_hw_init(void)
{
    FM1_PR(FM1_PB, FM1_OUT) |= PA_LCD_CS;
    FM1_PR(FM1_PB, FM1_OUT) &= ~(PA_LCD_DC | PA_LCD_B6);
    FM1_PR(FM1_PB, FM1_DIR) &= ~(PA_LCD_CS | PA_LCD_DC | PA_LCD_B6);
    FM1_PR(FM1_PB, FM1_DIR) |= PA_LCD_B7;
    pa_lcd_power(1);
    PA_LCD_SPI_CON = 0x4021u;
    PA_LCD_SPI_BAUD = PA_LCD_BAUD;
}

static void pa_lcd_wait(void)
{
    uint32_t n;
    for (n = 0; n < 4000000u && !(PA_LCD_SPI_CON & 0x8000u); n++)
        ;
    if (n == 4000000u)
        pa_lcd_timeouts++;
    PA_LCD_SPI_CON |= 0x4000u;
}

FM1_INLINE void pa_lcd_deselect(void) { FM1_PR(FM1_PB, FM1_OUT) |= PA_LCD_CS; }

FM1_INLINE void pa_lcd_send_cmd(uint8_t c)
{
    FM1_PR(FM1_PB, FM1_OUT) &= ~PA_LCD_DC;
    FM1_PR(FM1_PB, FM1_OUT) &= ~PA_LCD_CS;
    PA_LCD_SPI_CON |= 0x4000u;
    PA_LCD_SPI_BUF = c;
}

FM1_INLINE void pa_lcd_send_data(const void *p, uint32_t n)
{
    FM1_PR(FM1_PB, FM1_OUT) |= PA_LCD_DC;
    FM1_PR(FM1_PB, FM1_OUT) &= ~PA_LCD_CS;
    PA_LCD_SPI_CON |= 0x4000u;
    PA_LCD_SPI_ADR = (uint32_t)(uintptr_t)p;
    PA_LCD_SPI_CNT = n;
}
