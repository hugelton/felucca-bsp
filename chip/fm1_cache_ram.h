/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Henri David: from Optimist (github.com/w0ts/optimist, firmware/hal/fm1_cache.h, 9a1360ab), offered
 *   there under GPL-3.0-only OR MPL-2.0 (Optimist 3815869f, LICENSING.md "The cache RAM"); taken here
 *   under MPL-2.0
 * Written against the public JieLi AC79 SDK (Apache-2.0); see NOTICE. */
/* Data-cache ways as RAM: 1..7 of the 8 data-cache ways (4 KiB each) used as plain RAM at 0x01F28000 + way x 4 KiB,
 * as M-VAVE's FM-1 firmware (V15, 7 ways) and the AC79 SDK's flash-only builds do (cpu/wl82/sdk_ld_sfc.c
 * FREE_DACHE_WAY = 7, its cache_ram region; it keeps its system tables and lwIP's pools there). Ways 0..n-1 become
 * RAM, the top 8 - n stay a cache; all 8 instruction-cache ways (the flash's code) stay caches.
 *
 * The registers (corex2, the SDK's asm/csfr.h): CACHE_CON 0x01EEE008 (bit 14 idle, read only; bit 8 the cache on, bit
 * 9 on for SDRAM / PSRAM), DCACHE_WAY 0x01EEE00C, ICACHE_WAY 0x01EEE010; one set for the corex2 cluster (beside its
 * C0_CON and C1_CON), not one per core. A set bit is a way kept as a cache: DCACHE_WAY holds three 8-bit masks (bits
 * 0..7, 8..15, 16..23; which requester each serves is not known: all three get the same), ICACHE_WAY two. The
 * sequence (the SDK's cache_way_config, as V15 runs it from RAM): wait for idle; CACHE_CON bits 8-9 off; the masks;
 * every tag invalid (0x01F00000..0x01F04000, 0x01F08000..0x01F0A200, 0x01F0B000..0x01F0B200 zeroed); CACHE_CON back
 * as it was. With the cache off nothing may be fetched from the flash: fm1_cache_set is RAM code (.ram_text, copied
 * before it runs), calls nothing (its loops volatile: no memset), and runs with the interrupts off and the second
 * core stopped.
 *
 *   fm1_cache_ram_init(n)   once at the start, after .ram_text is copied, before the second core, the interrupts and
 *                           anything that uses the memory: n ways as RAM, tested (each word its address pattern, then
 *                           the complement, read back after each whole pass: an aliased way shows), zeroed. A cache
 *                           that never idles, masks that do not read back, or a failed test: the masks back as the
 *                           boot loader left them, the status says which, and fm1_cache_ram_bytes() is 0
 *   fm1_cache_ram_bytes()   the bytes usable from FM1_CR_LO (0: none: not asked for, or the test failed)
 *   fm1_cr                  the result and the registers before and after (a readout)
 *
 * Measured on one FM-1 (Optimist, 2026-10-18 .. 25): the test passes; the memory is as fast as main RAM (35 ns a read);
 * 2 ways in daily use for days, no fault; with 7 ways flash data reads over 16 KiB are 57x slower (1.7 us against
 * 32 ns: the flash's data goes through this cache, 4 KiB of it left), so fewer ways keep the flash's data fast. Not
 * measured: the second core's and DMA's access to it.
 *
 * The host (tests/cache_ram_test.c) builds the same code over a model: FM1_CR_P maps an address, FM1_CR_RD reads a
 * word of the test, FM1_CR_SET_DONE runs after the masks are written, FM1_CR_SYNC and FM1_CR_RAMFN as below. */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"

#define FM1_CR_LO 0x01F28000u                         /* data way 0 */
#define FM1_CR_WAY 4096u
#define FM1_CR_MAX 7u                                 /* (one way at least stays a cache) */
enum { FM1_CR_OFF, FM1_CR_OK, FM1_CR_BUSY, FM1_CR_REG, FM1_CR_BAD };   /* not asked / ready / never idle / masks not
                                                                        * taken / the test failed (all put back) */
typedef struct {
    uint32_t status, ways;                            /* the result; the ways asked for */
    uint32_t con, dway0, iway0;                       /* CACHE_CON and the masks as the boot loader left them */
    uint32_t dway, iway, bad_at;                      /* the masks read back after; the first failing address */
} fm1_cr_t;
static fm1_cr_t fm1_cr;

FM1_INLINE uint32_t fm1_cache_ram_bytes(void) { return fm1_cr.status == FM1_CR_OK ? fm1_cr.ways * FM1_CR_WAY : 0u; }

#ifndef FM1_CR_P
#define FM1_CR_P(a) ((volatile uint32_t *)(uintptr_t)(a))
#endif
#ifndef FM1_CR_RD
#define FM1_CR_RD(p) (*(p))
#endif
#ifndef FM1_CR_SET_DONE
#define FM1_CR_SET_DONE() ((void)0)
#endif
#ifndef FM1_CR_SYNC
#ifdef __PI32V2__
#define FM1_CR_SYNC() __asm__ volatile("csync\n\tssync" ::: "memory")
#else
#define FM1_CR_SYNC() __asm__ volatile("" ::: "memory")
#endif
#endif
#ifndef FM1_CR_RAMFN
#define FM1_CR_RAMFN __attribute__((section(".ram_text"), noinline, used))
#endif
#define FM1_CR_CON FM1_CR_P(0x01EEE008u)
#define FM1_CR_DWAY FM1_CR_P(0x01EEE00Cu)
#define FM1_CR_IWAY FM1_CR_P(0x01EEE010u)

FM1_INLINE void fm1_cr_zero(uint32_t lo, uint32_t hi)
{
    uint32_t a;
    for (a = lo; a < hi; a += 4u)
        *FM1_CR_P(a) = 0;
}

/* the masks set, every tag invalid; RAM only, interrupts off. 0: the cache never went idle (nothing changed) */
static FM1_CR_RAMFN uint32_t fm1_cache_set(uint32_t dway, uint32_t iway)
{
    uint32_t con, n = 2400000u;                       /* (~10 ms at 240 MHz: a fill never takes that long) */
    while (!(*FM1_CR_CON & 0x4000u))
        if (!--n)
            return 0;
    con = *FM1_CR_CON;
    *FM1_CR_CON = con & ~0x300u;
    FM1_CR_SYNC();
    *FM1_CR_DWAY = dway;
    *FM1_CR_IWAY = iway;
    FM1_CR_SET_DONE();
    fm1_cr_zero(0x01F00000u, 0x01F04000u);
    fm1_cr_zero(0x01F08000u, 0x01F0A200u);
    fm1_cr_zero(0x01F0B000u, 0x01F0B200u);
    FM1_CR_SYNC();
    *FM1_CR_CON = con;                                /* (bits 8-9 as they were) */
    FM1_CR_SYNC();
    return 1;
}

/* lo .. hi: each word its address pattern, then the complement, read back after the whole pass. 0 good, else the
 * first failing address. Ends zeroed */
static uint32_t fm1_cache_test(uint32_t lo, uint32_t hi)
{
    uint32_t pass, x, a;
    for (pass = 0; pass < 2u; pass++) {
        x = pass ? 0xFFFFFFFFu : 0u;
        for (a = lo; a < hi; a += 4u)
            *FM1_CR_P(a) = (a ^ 0x5AC3A53Cu) ^ x;
        FM1_CR_SYNC();
        for (a = lo; a < hi; a += 4u)
            if (FM1_CR_RD(FM1_CR_P(a)) != ((a ^ 0x5AC3A53Cu) ^ x))
                return a;
    }
    fm1_cr_zero(lo, hi);
    return 0;
}

/* once at the start (see the top): ways 0 does nothing (FM1_CR_OFF) */
static void fm1_cache_ram_init(uint32_t ways)
{
    uint32_t (*volatile set)(uint32_t, uint32_t) = fm1_cache_set;   /* (XIP -> RAM: past the 23-bit call range) */
    uint32_t keep, dw, iw;
    fm1_cr.ways = ways;
    if (!ways || ways > FM1_CR_MAX) {
        fm1_cr.status = FM1_CR_OFF;
        return;
    }
    keep = (0xFFu << ways) & 0xFFu;                   /* (a mask byte: the ways kept as a cache) */
    fm1_cr.con = *FM1_CR_CON;
    fm1_cr.dway0 = *FM1_CR_DWAY;
    fm1_cr.iway0 = *FM1_CR_IWAY;
    dw = (fm1_cr.dway0 & 0xFF000000u) | keep | keep << 8 | keep << 16;
    iw = (fm1_cr.iway0 & 0xFFFF0000u) | 0x0000FFFFu;
    if (!set(dw, iw)) {
        fm1_cr.status = FM1_CR_BUSY;
        return;
    }
    fm1_cr.dway = *FM1_CR_DWAY;
    fm1_cr.iway = *FM1_CR_IWAY;
    if (fm1_cr.dway != dw || fm1_cr.iway != iw)
        fm1_cr.status = FM1_CR_REG;
    else if ((fm1_cr.bad_at = fm1_cache_test(FM1_CR_LO, FM1_CR_LO + ways * FM1_CR_WAY)) != 0)
        fm1_cr.status = FM1_CR_BAD;
    else {
        fm1_cr.status = FM1_CR_OK;
        return;
    }
    set(fm1_cr.dway0, fm1_cr.iway0);                  /* back as the boot loader left them: nothing is placed there */
}
