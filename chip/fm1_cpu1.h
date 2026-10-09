/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * Written against the public JieLi AC79 SDK (Apache-2.0); see NOTICE. */
/* FM-1 second core (CPU1), EXPERIMENTAL: compiles, has not run on a device.
 *
 * The AC79 has two cores (CPU_CORE_NUM 2 in the SDK); the firmware so far runs on CPU0 only. The steps below are
 * those of the SDK's EnableOtherCpu (system.a, port.c):
 *   1. the entry address goes to the word at 0x01C7FFF8 (in the reserved top of RAM: call this before
 *      fm1_guard_lock_top, or after fm1_guard_unlock_top)
 *   2. SYS_DIV (0x10008) gets bit 3 set for the start and is restored when CPU1 reports in
 *   3. CORE_CON (0x1EEE004): set bit 3, clear bit 1 (release CPU1)
 *   4. CPU1 runs fm1_cpu1_entry -> fm1_cpu1_main and calls fm1_cpu1_ready()
 * The application provides: void fm1_cpu1_main(void); and the symbols _cpu1_ustack_top, _cpu1_sstack_top
 * (SDK sizes: 0x300 user, 0x1000 supervisor). CPU1 has its own interrupt bank (the CPU0 bank is what fm1_irq.h
 * sets up): run it polled, without interrupts, until that is understood.
 * Shared RAM: SRAM is read and written directly (no D-cache in between as far as the SDK shows); use volatile and
 * single-writer words, and add csync before reading what the other core wrote.
 *
 *   fm1_cpu1_start(timeout_ms)   0 when CPU1 reported in, -1 on timeout (CPU1 stays held; nothing waits forever)
 */
#pragma once
#include <stdint.h>
#include "fm1_time.h"

#define FM1_CPU1_ENTRY_WORD (*(volatile uint32_t *)0x01C7FFF8u)
#define FM1_CPU1_SYS_DIV    (*(volatile uint32_t *)0x10008u)
#define FM1_CPU1_CORE_CON   (*(volatile uint32_t *)0x1EEE004u)

extern void fm1_cpu1_entry(void);
extern void fm1_cpu1_main(void);       /* application: runs on CPU1, calls fm1_cpu1_ready() first */

static volatile uint8_t fm1_cpu1_up;
static inline void fm1_cpu1_ready(void) { fm1_cpu1_up = 1; }

static int fm1_cpu1_start(uint32_t timeout_ms)
{
    uint32_t div = FM1_CPU1_SYS_DIV, t = 0;
    fm1_cpu1_up = 0;
    FM1_CPU1_ENTRY_WORD = (uint32_t)(uintptr_t)fm1_cpu1_entry;
    FM1_CPU1_SYS_DIV = div | 8u;
    FM1_CPU1_CORE_CON |= 8u;
    FM1_CPU1_CORE_CON &= ~2u;
    while (!fm1_cpu1_up && t < timeout_ms * 1000u) {
        fm1_delay_us(1);
        t++;
    }
    FM1_CPU1_SYS_DIV = div;
    return fm1_cpu1_up ? 0 : -1;
}
