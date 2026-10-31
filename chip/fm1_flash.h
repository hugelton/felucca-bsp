/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio): configurable policy and access guards
 * Written against the public JieLi AC79 SDK (Apache-2.0); see NOTICE. */
/* FM-1 SPI NOR at runtime: erase / program / read.
 *
 * Everything that executes while the SFC (XIP) is switched off lives in
 * .ram_text and touches only SFRs, the stack and RAM buffers: no .rodata, no
 * libcalls, no XIP-resident helpers. Call the fl_*_ram() entry points with
 * interrupts disabled; fl_enter also checks CPU0/CPU1 access before disabling XIP.
 * Use the fl_* wrappers below from normal code. Configure capacity and write
 * regions before inclusion (fm1_flash_policy.h); see docs/FLASH.md. Same sequence as the AC79 SDK flash driver
 * (cpu.a/vm_sfc.c.o: enter_spi_code, norflash_erase, norflash_wait_ok, exit_spi_code).
 */
#pragma once
#include <stdint.h>
#include "fm1_xip.h"
#include "fm1_flash_policy.h"
#include "fm1_flash_access.h"

#ifndef RAMFN
#define RAMFN  __attribute__((section(".ram_text"), noinline, used))
#endif
#define RAMINL static inline __attribute__((always_inline))

/* XIP (0x02xxxxxx) -> RAM (0x01C0xxxx) is beyond the 23-bit call range:
 * call RAM code through a pointer the compiler cannot fold back. */
static inline void *fl_far(void *p) { void *volatile q = p; return q; }
#define FL_FAR(fn) ((__typeof__(&fn))fl_far((void *)&fn))

#ifndef REG32
#define REG32(a) (*(volatile uint32_t *)(a))
#endif
#ifndef FM1_FLASH_REG8
#define FM1_FLASH_REG8(a) (*(volatile uint8_t *)(a))
#endif
#ifndef FM1_FLASH_FLUSHINV
#define FM1_FLASH_FLUSHINV(a) __asm__ volatile("flushinv [%0]" :: "r"(a) : "memory")
#endif
#define SFC_CON         REG32(0x40200u)            /* bit31 busy; [11:8] read mode */
#define SFCENC_CON      FM1_FLASH_REG8(0x40300u)        /* b0 decrypt, b1 plain window */
#define SFCENC_UNENC_H  REG32(0x40308u)            /* WO, XIP (CPU) address */
#define SFCENC_UNENC_L  REG32(0x4030Cu)            /* WO, XIP (CPU) address */
#define SPI0_CON        REG32(0x11C00u)            /* b15 PND, b14 PCLR, b12 DIR=rx */
#define SPI0_BUF        REG32(0x11C08u)
#define IOMAP_CON0      REG32(0x5101Cu)            /* b5: 1 = SFC owns the flash pins */
#define PORTD_OUT       REG32(0x500C0u)            /* PD0 = flash CS# */
#define PORTD_DIR       REG32(0x500C8u)
#define CACHE_CON       REG32(0x1EEE008u)          /* b14 = cache idle */
#define T4_CNT          REG32(0x10804u)            /* TIMER4, 24 MHz free-running */

#define FL_XIP(off)     FM1_XIP(off)              /* fm1_xip.h */
typedef struct { uint32_t sfc_con, spi_con; } fl_saved_t;

RAMINL void fl_csync(void) { FM1_FLASH_SYNC(); }
RAMINL void fl_cache_idle(void) { while (!(CACHE_CON & 0x4000u)) ; }
RAMINL void fl_delay(uint32_t n) { volatile uint32_t i = n; while (i--) ; }

RAMINL void fl_cs(int high)
{
    PORTD_DIR &= ~1u;
    if (high) PORTD_OUT |= 1u; else PORTD_OUT &= ~1u;
    fl_delay(10);
}
RAMINL int fl_spi_wait(void)
{
    uint32_t n = 50000000u;
    fl_csync();
    while (!(SPI0_CON & 0x8000u))
        if (--n == 0) break;
    SPI0_CON |= 0x4000u;
    return n ? 0 : -1;
}
RAMINL void fl_tx(uint8_t b)
{
    SPI0_CON |= 8u; SPI0_CON &= ~0x1000u;
    SPI0_BUF = b; fl_spi_wait();
    SPI0_CON &= ~8u;
}
RAMINL uint8_t fl_rx(void)
{
    uint8_t v;
    SPI0_CON |= 8u; SPI0_CON |= 0x1000u;
    SPI0_BUF = 0xFFu; fl_spi_wait();
    v = (uint8_t)SPI0_BUF;
    SPI0_CON &= ~8u;
    return v;
}
RAMINL void fl_addr(uint32_t a) { fl_tx((uint8_t)(a >> 16)); fl_tx((uint8_t)(a >> 8)); fl_tx((uint8_t)a); }
RAMINL uint8_t fl_rdsr(uint8_t cmd) { uint8_t v; fl_cs(0); fl_tx(cmd); v = fl_rx(); fl_cs(1); return v; }
RAMINL void fl_wren(void) { fl_cs(0); fl_tx(0x06); fl_cs(1); }

RAMINL int fl_enter(fl_saved_t *s)
{
    uint32_t mode;
    if (!fm1_flash_access_ok()) return FL_ERR_BUSY;
    fl_cache_idle();
    s->sfc_con = SFC_CON;
    s->spi_con = SPI0_CON;
    mode = (s->sfc_con >> 8) & 0xFu;
    if (mode == 6u || mode == 7u)                 /* continuous-read XIP: not handled here */
        return FL_ERR_MODE;
    fl_csync();
    while ((int32_t)SFC_CON < 0) ;                /* SFC idle */
    SFC_CON = 0;                                  /* XIP OFF from here on */
    IOMAP_CON0 &= ~0x20u;                         /* pins -> SPI0 / GPIO */
    SPI0_CON = 0;
    SPI0_CON = 0x1029u;                           /* SDK default */
    fl_csync();
    return 0;
}
RAMINL void fl_exit(const fl_saved_t *s)
{
    SPI0_CON = s->spi_con;
    IOMAP_CON0 |= 0x20u;
    fl_csync();
    while ((int32_t)SFC_CON < 0) ;
    SFC_CON = s->sfc_con;                         /* XIP back on */
    fl_csync();
}
RAMINL void fl_inval(uint32_t off, uint32_t len)
{
    uint32_t a, e;
    if (off >= FM1_FLASH_XIP_SIZE) return;
    if (len > FM1_FLASH_XIP_SIZE - off) len = FM1_FLASH_XIP_SIZE - off;
    a = FL_XIP(off) & ~31u; e = FL_XIP(off) + len;
    fl_cache_idle();
    for (; a < e; a += 32u) {
        fl_csync();
        FM1_FLASH_FLUSHINV(a);
        fl_csync();
        fl_cache_idle();
    }
}
RAMINL int fl_wait_ready(uint32_t max_us, uint32_t *took_us)
{
    uint32_t t0 = T4_CNT, dt;
    for (;;) {
        dt = (T4_CNT - t0) / 24u;
        if (!(fl_rdsr(0x05) & 1u)) { *took_us = dt; return 0; }
        if (dt > max_us) { *took_us = dt; return -4; }
    }
}

/* ---- entry points: IRQs OFF, args/buffers in RAM ---- */

static RAMFN uint32_t fl_jedec_ram(void)                 /* expect 0x856014 */
{
    fl_saved_t s; uint32_t id;
    if (fl_enter(&s)) return 0;
    fl_cs(0); fl_tx(0x9F);
    id = (uint32_t)fl_rx() << 16; id |= (uint32_t)fl_rx() << 8; id |= fl_rx();
    fl_cs(1);
    fl_exit(&s);
    return id;
}

/* the part's factory unique ID (0x4B, four dummy bytes; the stock Puya P25Q80 gives 16 bytes, a Winbond-style part
 * 8): dst (RAM) gets 16 bytes; the same on every read of the same part. Only with FL_UID defined: RAM code costs
 * every firmware that includes this */
#ifdef FL_UID
static RAMFN int fl_uid_ram(uint8_t *dst)
{
    fl_saved_t s; uint32_t i;
    if (!FL_RAM_OK(dst, 16u)) return FL_ERR_BUFFER;
    { int rc = fl_enter(&s); if (rc) return rc; }
    fl_cs(0);
    for (i = 0; i < 5u; i++) fl_tx(i ? 0 : 0x4B);
    for (i = 0; i < 16u; i++) dst[i] = fl_rx();
    fl_cs(1);
    fl_exit(&s);
    return 0;
}
#endif

static RAMFN uint32_t fl_status_ram(void)                /* SR1 | SR2 << 8 */
{
    fl_saved_t s; uint32_t v;
    if (fl_enter(&s)) return 0xFFFFFFFFu;
    v = fl_rdsr(0x05) | ((uint32_t)fl_rdsr(0x35) << 8);
    fl_exit(&s);
    return v;
}

static RAMFN int fl_erase4k_ram(uint32_t off, uint32_t *took_us)
{
    fl_saved_t s; int rc;
    if (!FL_WRITE_OK(off, 0x1000u) || (off & 0xFFFu))
        return FL_ERR_RANGE;
    if (!FL_RAM_OK(took_us, sizeof *took_us)) return FL_ERR_BUFFER;
    { int rc = fl_enter(&s); if (rc) return rc; }
    fl_wren();
    if (!(fl_rdsr(0x05) & 2u)) rc = -3;           /* WEL did not set */
    else {
        fl_cs(0); fl_tx(0x20); fl_addr(off); fl_cs(1);
        fl_inval(off, 4096u);
        rc = fl_wait_ready(500000u, took_us);     /* generic NOR max 400 ms */
    }
    fl_exit(&s);
    return rc;
}

static RAMFN int fl_prog_ram(uint32_t off, const uint8_t *src, uint32_t n, uint32_t *took_us)
{
    fl_saved_t s; int rc; uint32_t i;
    if (!FL_WRITE_OK(off, n) ||
        n == 0 || n > 256u || ((off & 0xFFu) + n) > 256u) return FL_ERR_RANGE;    /* one page, no wrap */
    if (!FL_RAM_OK(src, n) || !FL_RAM_OK(took_us, sizeof *took_us)) return FL_ERR_BUFFER;
    { int rc = fl_enter(&s); if (rc) return rc; }
    fl_wren();
    if (!(fl_rdsr(0x05) & 2u)) rc = -3;
    else {
        fl_cs(0); fl_tx(0x02); fl_addr(off);
        for (i = 0; i < n; i++) fl_tx(src[i]);
        fl_cs(1);
        fl_inval(off, n);
        rc = fl_wait_ready(10000u, took_us);      /* generic NOR max 3 ms */
    }
    fl_exit(&s);
    return rc;
}

static RAMFN int fl_read_ram(uint32_t off, uint8_t *dst, uint32_t n)   /* 0x0B fast read */
{
    fl_saved_t s; uint32_t i;
    if (!FL_READ_OK(off, n)) return FL_ERR_RANGE;
    if (!FL_RAM_OK(dst, n)) return FL_ERR_BUFFER;
    { int rc = fl_enter(&s); if (rc) return rc; }
    fl_cs(0); fl_tx(0x0B); fl_addr(off); fl_tx(0xFF);
    for (i = 0; i < n; i++) dst[i] = fl_rx();
    fl_cs(1);
    fl_exit(&s);
    return 0;
}

/* ---- XIP side (normal .text) ---- */

/* Once at boot, before any XIP read of the data region: map the data area
 * as an unencrypted XIP window (as the SDK's IOCTL_SET_VM_INFO does). */
static int fl_plain_window_init(void)
{
    uint32_t f = irq_save();
    if (!fm1_flash_access_ok()) { irq_restore(f); return FL_ERR_BUSY; }
    SFCENC_CON &= (uint8_t)~2u;
    SFCENC_UNENC_L = FL_XIP(FM1_FLASH_PLAIN_LO);
    /* Preserve the established full-window setup for the default layout. */
    SFCENC_UNENC_H = FM1_FLASH_PLAIN_HI == FM1_FLASH_XIP_SIZE ?
        0x07FFFFFFu : FL_XIP(FM1_FLASH_PLAIN_HI) - 1u;
    SFCENC_CON |= 2u;
    fl_inval(FM1_FLASH_PLAIN_LO, FM1_FLASH_PLAIN_HI - FM1_FLASH_PLAIN_LO);
    irq_restore(f);
    return 0;
}

#include "fm1_flash_api.h"
