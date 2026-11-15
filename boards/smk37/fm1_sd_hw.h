/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Julius Vitkauskas */
/* A microSD card on the LCD's SPI1, in SPI mode, on the SMK-37: a hand-wired mod as m-bootloader's SD card item
 * (github.com/jvitkauskas/m-bootloader, docs/SPEC.md §11.5, where this session's sequence has run on an SMK-37 Pro),
 * the FM-1's (boards/fm1/fm1_sd_hw.h, the m-card board) on this board's pins, SPI1 in pin group A:
 *
 *   card CLK   PB6, SPI1's clock (the panel's)
 *   card DI    PB7, SPI1's DO (the panel's data); a pull-up to 3.3 V wanted (the m-card board has 47k)
 *   card DO    PB5, the panel's D/C (series resistor wanted, ~220 R): SPI1's DI in this group while a session is open
 *   card CS    NOT PC8 (an inverter): the card is selected while the panel's CS is high, the panel while it's low
 *
 * The pad LEDs switch SPI1 to pin group B for a frame (fm1_lcd_hw.h fm1_padled_send / _done, synchronous, between
 * LCD transfers); a session asserts group A. During a pad frame the panel's CS is high, so the card is selected, but
 * its clock (PB6, group A's) stays still: it sees nothing.
 *
 * The interface is the FM-1's (boards/fm1/fm1_sd_hw.h); see there. Port B and C's pins here are written from the
 * main loop only (as the LCD's are). */
#pragma once
#include <stdint.h>
#include "fm1_board_map.h"                    /* this board's flash map, not the chip's */
#include "fm1_cc.h"
#include "fm1_gpio.h"

#define FM1_SD_SPI_CON (*(volatile uint32_t *)0x11D00u)   /* b0 on, b3 separate DI / DO, b12 in, b14 clear, b15 done */
#define FM1_SD_SPI_BAUD (*(volatile uint32_t *)0x11D04u)
#define FM1_SD_SPI_BUF (*(volatile uint32_t *)0x11D08u)
#define FM1_SD_SPI_ADR (*(volatile uint32_t *)0x11D0Cu)
#define FM1_SD_SPI_CNT (*(volatile uint32_t *)0x11D10u)
#define FM1_SD_CS (1u << 8)               /* PC8: the panel's CS; the card's, inverted */
#define FM1_SD_DO (1u << 5)               /* PB5: the panel's D/C; the card's DO in a session */
#define FM1_SD_IOMAP_CON1 (*(volatile uint32_t *)0x51020u)
#define FM1_SD_GROUP_B 0x10u              /* IOMAP_CON1 bit 4: SPI1 on group B (the pad LEDs), clear: group A */
#define FM1_SD_CON_LCD 0x4021u            /* SPI1 as fm1_lcd_hw.h runs it */
#define FM1_SD_CON 0x4029u                /* the same, DI and DO on separate lines */
#define FM1_SD_WAIT 400000u               /* polls for one byte: far above a byte at 400 kHz (~20 us) */

static uint32_t fm1_sd_timeouts;
static uint32_t fm1_sd_pu, fm1_sd_die;    /* PB5's pull-up and input enable before the session */

FM1_INLINE void fm1_sd_sync(void) { __asm__ volatile("csync" ::: "memory"); }

static int fm1_sd_wait(uint32_t polls)
{
    uint32_t n;
    for (n = 0; n < polls && !(FM1_SD_SPI_CON & 0x8000u); n++)
        ;
    FM1_SD_SPI_CON |= 0x4000u;
    if (n == polls) {
        fm1_sd_timeouts++;
        return -1;
    }
    return 0;
}

FM1_INLINE void fm1_sd_select(uint32_t on)
{
    if (on)
        FM1_PR(FM1_PC, FM1_OUT) |= FM1_SD_CS;
    else
        FM1_PR(FM1_PC, FM1_OUT) &= ~FM1_SD_CS;
}

FM1_INLINE void fm1_sd_baud(uint32_t b) { FM1_SD_SPI_BAUD = b; }

static uint8_t fm1_sd_xfer(uint8_t b)
{
    FM1_SD_SPI_CON |= 0x4000u;
    FM1_SD_SPI_BUF = b;
    fm1_sd_sync();
    if (fm1_sd_wait(FM1_SD_WAIT))
        return 0xFFu;
    return (uint8_t)FM1_SD_SPI_BUF;
}

/* a DMA left running: started here, the caller free meanwhile (fm1_sd_dma_done), then finished (fm1_sd_dma_end) */
FM1_INLINE void fm1_sd_rx_start(void *p, uint32_t n)
{
    FM1_SD_SPI_CON |= 0x1000u;            /* in */
    FM1_SD_SPI_CON |= 0x4000u;
    FM1_SD_SPI_ADR = (uint32_t)(uintptr_t)p;
    fm1_sd_sync();
    FM1_SD_SPI_CNT = n;                   /* starts it */
}

FM1_INLINE void fm1_sd_tx_start(const void *p, uint32_t n)
{
    FM1_SD_SPI_CON &= ~0x1000u;           /* out */
    FM1_SD_SPI_CON |= 0x4000u;
    FM1_SD_SPI_ADR = (uint32_t)(uintptr_t)p;
    fm1_sd_sync();
    FM1_SD_SPI_CNT = n;
}

FM1_INLINE int fm1_sd_dma_done(void) { return (FM1_SD_SPI_CON & 0x8000u) != 0; }

static int fm1_sd_dma_end(uint32_t n)     /* the DMA of n bytes waited for: 0 done, -1 timeout; SPI1 back to bytes */
{
    int rc = fm1_sd_wait(FM1_SD_WAIT + n * 64u);
    FM1_SD_SPI_CON &= ~0x1000u;
    __asm__ volatile("ssync" ::: "memory");   /* (the DMA's writes seen before its buffer is read) */
    return rc;
}

static int fm1_sd_rx(void *p, uint32_t n)
{
    fm1_sd_rx_start(p, n);
    return fm1_sd_dma_end(n);
}

static int fm1_sd_tx(const void *p, uint32_t n)
{
    fm1_sd_tx_start(p, n);
    return fm1_sd_dma_end(n);
}

static void fm1_sd_open(uint32_t baud)
{
    FM1_SD_IOMAP_CON1 &= ~FM1_SD_GROUP_B;    /* group A (a pad frame is over: fm1_padled_done) */
    fm1_sd_select(0);                        /* the panel's NOP: CS low, D/C low (a command) */
    FM1_PR(FM1_PB, FM1_OUT) &= ~FM1_SD_DO;
    FM1_SD_SPI_CON = FM1_SD_CON_LCD;
    fm1_sd_xfer(0x00u);
    fm1_sd_pu = FM1_PR(FM1_PB, FM1_PU) & FM1_SD_DO;
    fm1_sd_die = FM1_PR(FM1_PB, FM1_DIE) & FM1_SD_DO;
    FM1_PR(FM1_PB, FM1_DIR) |= FM1_SD_DO;    /* PB5 the card's DO: an input, pulled up */
    FM1_PR(FM1_PB, FM1_DIE) |= FM1_SD_DO;
    FM1_PR(FM1_PB, FM1_PU) |= FM1_SD_DO;
    FM1_PR(FM1_PB, FM1_PD) &= ~FM1_SD_DO;
    FM1_SD_SPI_CON = FM1_SD_CON;
    FM1_SD_SPI_BAUD = baud;
}

static void fm1_sd_close(uint32_t lcd_baud)
{
    fm1_sd_select(0);
    fm1_sd_xfer(0xFFu);                      /* the card lets go of its DO */
    FM1_SD_SPI_CON = FM1_SD_CON_LCD;
    FM1_SD_SPI_BAUD = lcd_baud;
    FM1_PR(FM1_PB, FM1_OUT) |= FM1_SD_DO;
    FM1_PR(FM1_PB, FM1_DIR) &= ~FM1_SD_DO;   /* D/C an output again, high */
    FM1_PR(FM1_PB, FM1_PU) = (FM1_PR(FM1_PB, FM1_PU) & ~FM1_SD_DO) | fm1_sd_pu;
    FM1_PR(FM1_PB, FM1_DIE) = (FM1_PR(FM1_PB, FM1_DIE) & ~FM1_SD_DO) | fm1_sd_die;
    fm1_sd_select(1);                        /* the panel deselected, as between its transfers */
}

FM1_INLINE void fm1_sd_park(void) { FM1_PR(FM1_PB, FM1_OUT) |= FM1_SD_DO; }

/* whether SPI1's DMA may use p for a transfer: 4-byte aligned, in the main RAM below its reserved top (not the
 * flash, not the cache's RAM) */
FM1_INLINE int fm1_sd_dma_ok(const void *p)
{
    uintptr_t a = (uintptr_t)p;
    return !(a & 3u) && a >= 0x01C00000u && a < 0x01C7F000u;
}
