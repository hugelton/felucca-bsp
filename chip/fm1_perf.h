/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Charles Vestal */
/* FM-1 (AC79) performance counters, from JieLi's AC79 SDK (include_lib/driver/cpu/wl82/asm/csfr.h,
 * the corex2 block at 0x1EEE000; cpu/wl82/debug.c's cpu_effic_calc reads them):
 *
 *   C0_TL_CKCNT  core 0's clock cycles (64-bit; the low word is enough for deltas under ~17 s)
 *   C0_IF_UACNT  ... instruction-fetch cycles the SDK reports against TL_CKCNT: read as stalls
 *   C0_RD_UACNT  ... data reads, the same
 *   C0_WR_UACNT  ... data writes, the same
 *
 * DBG_CON bits 0-2 ("if/of/ex_inv_en" in the SDK) start all four, TL_CKCNT too: with DBG_CON at 0, as at
 * boot, TL_CKCNT stays still (measured on an FM-1, felucca-bsp#18: 360,031 counts a ms with the bits set,
 * 0 before and after; the SDK's cpu_effic_init sets them before it reads any counter). So
 * fm1_perf_stalls_enable(1) is what starts the cycle counter as well; until then fm1_perf_cycles_run
 * is 0 and a caller falls back to TIMER4. DBG_CON is written behind the debug unlock (DBG_WR_EN, as
 * fm1_guard.h), which is left as it was found. The bits are set only when asked, never at boot: whether
 * they are safe while the app runs (they may also arm fault exceptions) is not known. Felucca's guards use
 * DBG_EN (0x340), not DBG_CON (0x344). */
#pragma once
#include <stdint.h>
#include "fm1_time.h"

#define FM1_C0_IF_UACNTL (*(volatile uint32_t *)0x1EEE200u)
#define FM1_C0_RD_UACNTL (*(volatile uint32_t *)0x1EEE208u)
#define FM1_C0_WR_UACNTL (*(volatile uint32_t *)0x1EEE210u)
#define FM1_C0_TL_CKCNTL (*(volatile uint32_t *)0x1EEE218u)
#define FM1_DBG_CON (*(volatile uint32_t *)0x1EEE344u)
#ifndef FM1_DBG_WR_EN
#define FM1_DBG_WR_EN (*(volatile uint32_t *)0x1EEE240u)   /* (as fm1_irq.h) */
#endif

static inline uint32_t fm1_perf_cycles(void) { return FM1_C0_TL_CKCNTL; }

/* the cycle counter moves: two reads 100 us apart differ */
static inline int fm1_perf_cycles_run(void)
{
    uint32_t c0 = FM1_C0_TL_CKCNTL, t0 = fm1_ticks();
    while ((uint32_t)(fm1_ticks() - t0) < 100u * FM1_TICKS_PER_US)
        ;
    return FM1_C0_TL_CKCNTL != c0;
}

static inline void fm1_perf_stalls(uint32_t s[3])
{
    s[0] = FM1_C0_IF_UACNTL;
    s[1] = FM1_C0_RD_UACNTL;
    s[2] = FM1_C0_WR_UACNTL;
}

/* DBG_CON bits 0-2: the three stall counts and the cycle counter (TL_CKCNT) on or off */
static inline void fm1_perf_stalls_enable(int on)
{
    uint32_t was = FM1_DBG_WR_EN & 1u;          /* (writing 0xE7 toggles the unlock) */
    if (!was)
        FM1_DBG_WR_EN = 0xE7u;
    if (on)
        FM1_DBG_CON |= 0x7u;
    else
        FM1_DBG_CON &= ~0x7u;
    if (!was)
        FM1_DBG_WR_EN = 0xE7u;
}
