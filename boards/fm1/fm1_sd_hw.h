/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Julius Vitkauskas */
/* A microSD card on the LCD's SPI1, in SPI mode: the m-card board (github.com/jvitkauskas/m-card), the wiring of
 * m-bootloader's SD card item (github.com/jvitkauskas/m-bootloader, docs/SPEC.md §11.5), where this session's
 * sequence has run on FM-1s. The board passes the panel's ten lines through and taps four of them:
 *
 *   card CLK   PC9, SPI1's clock (the panel's), through 470 R
 *   card DI    PC10, SPI1's DO (the panel's data), through 470 R, 47k pull-up
 *   card DO    PC8, the panel's D/C, through 220 R, 47k pull-up: SPI1's DI in this pin group while a session is open
 *   card CS    NOT PC7 (an inverter): the card is selected while the panel's CS is high, the panel while it's low
 *
 * No card-detect line: a card is found by asking it (fm1-base device/sd.c). The protocol (commands, CRCs, the
 * clock's steps) is the caller's; this file only moves bytes.
 *
 * A session borrows SPI1 between two LCD transfers: the caller has waited for the panel's DMA (fm1-base lcd_sync, CS
 * high) and keeps the LCD away until fm1_sd_close.
 *
 *   fm1_sd_open(baud)      the panel sent a NOP, so that what is clocked later with the card deselected (the panel
 *                          selected, D/C pulled high: data) follows no command and is ignored; PC8 an input with
 *                          its pull-up (the card's DO); SPI1 in and out on separate lines at lsb / (baud + 1); the
 *                          card deselected
 *   fm1_sd_baud(b)         SPI1's clock = lsb / (b + 1) (LSB 60 MHz at 360 MHz, 66 at 396)
 *   fm1_sd_select(on)      the card's CS (on: PC7 high, the panel deselected)
 *   fm1_sd_xfer(b)         one byte out, the byte that came in back
 *   fm1_sd_rx(p, n)        n (1..65535) bytes in by DMA to RAM p, 4-byte aligned; 0 done, -1 timeout. What SPI1 puts
 *                          on its DO meanwhile is not established: the board's pull-up keeps the card's DI high if
 *                          SPI1 lets go of it, so the caller checks this way against byte reads before using it
 *   fm1_sd_tx(p, n)        n bytes out by DMA from RAM p, 4-byte aligned; 0 done, -1 timeout
 *   fm1_sd_rx_start(p, n), fm1_sd_tx_start(p, n), fm1_sd_dma_done(), fm1_sd_dma_end(n)
 *                          the same, left running: started, the caller free meanwhile, done or not, waited for
 *   fm1_sd_close(baud)     the card deselected and given 8 clocks to let go of its DO, SPI1 as the panel uses it at
 *                          the panel's baud, PC8 an output again (high), its pull and input as they were, the panel
 *                          deselected (as fm1-base's lcd.c leaves it between transfers)
 *   fm1_sd_park()          PC8 high: the panel's D/C between transfers, the level the card's DO idles at while
 *                          the card is selected (the panel deselected), so the two don't pull against each other
 *                          through the 220 R
 *   fm1_sd_timeouts        the waits that gave up (each bounded; the protocol then sees 0xFF bytes and fails)
 *
 * Port C is written here from the main loop only (as the LCD's pins are): no interrupt writes PC7..PC10. */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"
#include "fm1_gpio.h"

#define FM1_SD_SPI_CON (*(volatile uint32_t *)0x11D00u)   /* b0 on, b3 separate DI / DO, b12 in, b14 clear, b15 done */
#define FM1_SD_SPI_BAUD (*(volatile uint32_t *)0x11D04u)
#define FM1_SD_SPI_BUF (*(volatile uint32_t *)0x11D08u)
#define FM1_SD_SPI_ADR (*(volatile uint32_t *)0x11D0Cu)
#define FM1_SD_SPI_CNT (*(volatile uint32_t *)0x11D10u)
#define FM1_SD_CS (1u << 7)               /* PC7: the panel's CS; the card's, inverted */
#define FM1_SD_DO (1u << 8)               /* PC8: the panel's D/C; the card's DO in a session */
#define FM1_SD_CON_LCD 0x4021u            /* SPI1 as fm1_lcd_hw.h runs it */
#define FM1_SD_CON 0x4029u                /* the same, DI and DO on separate lines */
#define FM1_SD_WAIT 400000u               /* polls for one byte: far above a byte at 400 kHz (~20 us) */

static uint32_t fm1_sd_timeouts;
static uint32_t fm1_sd_pu, fm1_sd_die;    /* PC8's pull-up and input enable before the session */

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
    fm1_sd_select(0);                        /* the panel's NOP: CS low, D/C low (a command) */
    FM1_PR(FM1_PC, FM1_OUT) &= ~FM1_SD_DO;
    FM1_SD_SPI_CON = FM1_SD_CON_LCD;
    fm1_sd_xfer(0x00u);
    fm1_sd_pu = FM1_PR(FM1_PC, FM1_PU) & FM1_SD_DO;
    fm1_sd_die = FM1_PR(FM1_PC, FM1_DIE) & FM1_SD_DO;
    FM1_PR(FM1_PC, FM1_DIR) |= FM1_SD_DO;    /* PC8 the card's DO: an input, pulled up */
    FM1_PR(FM1_PC, FM1_DIE) |= FM1_SD_DO;
    FM1_PR(FM1_PC, FM1_PU) |= FM1_SD_DO;
    FM1_PR(FM1_PC, FM1_PD) &= ~FM1_SD_DO;
    FM1_SD_SPI_CON = FM1_SD_CON;
    FM1_SD_SPI_BAUD = baud;
}

static void fm1_sd_close(uint32_t lcd_baud)
{
    fm1_sd_select(0);
    fm1_sd_xfer(0xFFu);                      /* the card lets go of its DO */
    FM1_SD_SPI_CON = FM1_SD_CON_LCD;
    FM1_SD_SPI_BAUD = lcd_baud;
    FM1_PR(FM1_PC, FM1_OUT) |= FM1_SD_DO;
    FM1_PR(FM1_PC, FM1_DIR) &= ~FM1_SD_DO;   /* D/C an output again, high */
    FM1_PR(FM1_PC, FM1_PU) = (FM1_PR(FM1_PC, FM1_PU) & ~FM1_SD_DO) | fm1_sd_pu;
    FM1_PR(FM1_PC, FM1_DIE) = (FM1_PR(FM1_PC, FM1_DIE) & ~FM1_SD_DO) | fm1_sd_die;
    fm1_sd_select(1);                        /* the panel deselected, as between its transfers */
}

FM1_INLINE void fm1_sd_park(void) { FM1_PR(FM1_PC, FM1_OUT) |= FM1_SD_DO; }

/* whether SPI1's DMA may use p for a transfer: 4-byte aligned, in the main RAM below its reserved top (not the
 * flash, not the cache's RAM) */
FM1_INLINE int fm1_sd_dma_ok(const void *p)
{
    uintptr_t a = (uintptr_t)p;
    return !(a & 3u) && a >= 0x01C00000u && a < 0x01C7F000u;
}
