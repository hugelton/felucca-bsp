/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio)
 * The SPI1 sequence and the panel's transfer helpers are the FM-1 board's (boards/fm1/fm1_lcd_hw.h):
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* SMK-37 LCD wiring: the FM-1's 1.54" 240 x 240 panel (device/lcd.c's protocol) on SPI1 pin group A: PB6 CLK,
 * PB7 DO, PC8 CS, PB5 D/C (IOMAP_CON1 bit 4 clear). No backlight pin: the panel is always on
 * (fm1_lcd_backlight does nothing). No reset pin.
 *
 * The 16 pad LEDs (WS2812 class, GRB) hang on PC10, which is SPI1's DO in pin group B (IOMAP_CON1 bit 4 set:
 * the FM-1's LCD group). Between two LCD transfers the main loop switches SPI1 over for one frame:
 *   fm1_padled_send(p, n)      after lcd_sync (the LCD's DMA done, CS high): group B, the LED baud, DMA n bytes
 *                              from RAM p (one SPI byte per LED bit: 0xF0 a one, 0xC0 a zero, FM1_PADLED_BAUD
 *                              makes a byte ~1.2 us; the frame: 250 zero bytes of reset, ~300 us, then 16 x 24 bytes:
 *                              a shorter reset lets a glitch at the pin switch shift the chain by one LED)
 *   fm1_padled_done()          wait for it, then group A and the LCD's baud again; PC10 a low output between
 *                              frames (the latch time always met)
 * PC9 (group B's CLK) is mux B's analog line (fm1_adc.h): it is left an input, never driven; the reference port
 * runs so (verified). Conversions on mux B that straddle a frame are dropped by the scan (fm1_padled_on).
 *
 * Board facts: M. Geisler's analysis of the stock v016 firmware (fm1-to-smk37, docs/01-hardware.md §3-4: the
 * pins, the groups, the LED encoding and frame are marked verified there, on an Elite running Melodee; the SDK's
 * spi.h gives the same SPI1 groups). Not yet checked on a unit by this layer (fm1-base's SMK-37 port, step 2).
 *
 *   fm1_lcd_hw_init()          pins, SPI1 master at BAUD 4, group A
 *   fm1_lcd_baud(b)            SPI1 clock = lsb / (b + 1)
 *   fm1_lcd_send_cmd(c)        D/C low, CS low, one byte (then fm1_lcd_wait)
 *   fm1_lcd_send_data(p, n)    D/C high, CS low, DMA n bytes from RAM p
 *   fm1_lcd_wait()             0 done, -1 timeout (counted); pending cleared, the SPI left on either way
 *   fm1_lcd_spi_off()          SPI1 off (after a timeout; fm1_lcd_hw_init turns it on again)
 *   fm1_lcd_deselect()         CS high
 *   fm1_lcd_backlight(on)      nothing (no backlight pin)
 */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"

#ifndef FM1_LCD_REG
#define FM1_LCD_REG(addr) (*(volatile uint32_t *)(addr))
#endif
#define FM1_LCD_PB_OUT FM1_LCD_REG(0x50040u)
#define FM1_LCD_PB_DIR FM1_LCD_REG(0x50048u)
#define FM1_LCD_PC_OUT FM1_LCD_REG(0x50080u)
#define FM1_LCD_PC_DIR FM1_LCD_REG(0x50088u)
#define FM1_LCD_IOMAP_CON1 FM1_LCD_REG(0x51020u)
#define FM1_LCD_SPI_CON FM1_LCD_REG(0x11D00u)
#define FM1_LCD_SPI_BAUD FM1_LCD_REG(0x11D04u)
#define FM1_LCD_SPI_BUF FM1_LCD_REG(0x11D08u)
#define FM1_LCD_SPI_ADR FM1_LCD_REG(0x11D0Cu)
#define FM1_LCD_SPI_CNT FM1_LCD_REG(0x11D10u)
#define FM1_LCD_DC (1u << 5)               /* PB5 */
#define FM1_LCD_CLK (1u << 6)              /* PB6 */
#define FM1_LCD_DO (1u << 7)               /* PB7 */
#define FM1_LCD_CS (1u << 8)               /* PC8 */
#define FM1_PADLED_DO (1u << 10)           /* PC10: the pad LEDs' data line (SPI1 DO in group B) */
#define FM1_LCD_GROUP_B 0x10u              /* IOMAP_CON1 bit 4: SPI1 on PC9 / PC10 (set) or PB6 / PB7 (clear) */
#define FM1_PADLED_BAUD 8u                 /* lsb / 9: ~6.67 MHz, one SPI byte (an LED bit) ~1.2 us */
#define FM1_PADLED_RESET 250u              /* zero bytes before the 16 LEDs: the latch (~300 us low) */
#define FM1_PADLED_FRAME (FM1_PADLED_RESET + 16u * 24u)   /* 634 bytes, ~0.8 ms */
#define FM1_LCD_NO_BACKLIGHT 1

static uint32_t fm1_lcd_timeouts;
static uint32_t fm1_lcd_baud_now = 4u;     /* the LCD's baud, put back after a pad frame */
static volatile uint8_t fm1_padled_on;     /* a pad frame is on the wire (the scan drops mux B meanwhile) */

FM1_INLINE void fm1_lcd_hw_init(void)
{
    FM1_LCD_IOMAP_CON1 &= ~FM1_LCD_GROUP_B;
    FM1_LCD_PC_OUT |= FM1_LCD_CS;
    FM1_LCD_PC_OUT &= ~FM1_PADLED_DO;
    FM1_LCD_PC_DIR &= ~(FM1_LCD_CS | FM1_PADLED_DO);
    FM1_LCD_PB_OUT &= ~(FM1_LCD_DC | FM1_LCD_CLK | FM1_LCD_DO);
    FM1_LCD_PB_DIR &= ~(FM1_LCD_DC | FM1_LCD_CLK | FM1_LCD_DO);
    FM1_LCD_SPI_CON = 0x4021u;
    FM1_LCD_SPI_BAUD = 4u;
    fm1_lcd_baud_now = 4u;
}

FM1_INLINE void fm1_lcd_backlight(uint32_t on) { (void)on; }

FM1_INLINE void fm1_lcd_baud(uint32_t b)
{
    fm1_lcd_baud_now = b;
    FM1_LCD_SPI_BAUD = b;
}

/* as the FM-1's (boards/fm1/fm1_lcd_hw.h): a timeout is counted and reported, and the SPI stays on */
static int fm1_lcd_wait(void)
{
    uint32_t n;
    int rc = 0;
    for (n = 0; n < 4000000u && !(FM1_LCD_SPI_CON & 0x8000u); n++)
        ;
    if (n == 4000000u) {
        fm1_lcd_timeouts++;
        rc = -1;
    }
    FM1_LCD_SPI_CON |= 0x4000u;
    return rc;
}

FM1_INLINE void fm1_lcd_spi_off(void) { FM1_LCD_SPI_CON &= ~1u; }

FM1_INLINE void fm1_lcd_deselect(void) { FM1_LCD_PC_OUT |= FM1_LCD_CS; }

FM1_INLINE void fm1_lcd_send_cmd(uint8_t c)
{
    FM1_LCD_PB_OUT &= ~FM1_LCD_DC;
    FM1_LCD_PC_OUT &= ~FM1_LCD_CS;
    FM1_LCD_SPI_CON |= 0x4000u;
    FM1_LCD_SPI_BUF = c;
}

FM1_INLINE void fm1_lcd_send_data(const void *p, uint32_t n)
{
    FM1_LCD_PB_OUT |= FM1_LCD_DC;
    FM1_LCD_PC_OUT &= ~FM1_LCD_CS;
    FM1_LCD_SPI_CON |= 0x4000u;
    FM1_LCD_SPI_ADR = (uint32_t)(uintptr_t)p;
    FM1_LCD_SPI_CNT = n;
}

/* one pad LED frame (p: FM1_PADLED_FRAME bytes in RAM, the encoding above), between LCD transfers only: the
 * caller has waited for the LCD's DMA (lcd_sync: CS high). Never switch the pin group with a DMA running */
FM1_INLINE void fm1_padled_send(const void *p, uint32_t n)
{
    fm1_padled_on = 1;
    FM1_LCD_PC_OUT |= FM1_LCD_CS;              /* the panel ignores the bus */
    FM1_LCD_SPI_BAUD = FM1_PADLED_BAUD;
    FM1_LCD_IOMAP_CON1 |= FM1_LCD_GROUP_B;
    FM1_LCD_SPI_CON |= 0x4000u;
    FM1_LCD_SPI_ADR = (uint32_t)(uintptr_t)p;
    FM1_LCD_SPI_CNT = n;
}
static void fm1_padled_done(void)
{
    fm1_lcd_wait();
    FM1_LCD_IOMAP_CON1 &= ~FM1_LCD_GROUP_B;
    FM1_LCD_SPI_BAUD = fm1_lcd_baud_now;
    FM1_LCD_PC_OUT &= ~FM1_PADLED_DO;
    fm1_padled_on = 0;
}
