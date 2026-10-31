/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio) */
/* Shared by the RAM flash driver and the CPU1 worker. Include in the same
 * compilation unit as both: like the rest of this BSP, the state is static.
 * Only CPU0 may enter a flash transaction, with its interrupts masked.
 * A running CPU1 must have acknowledged that it is in its RAM-only idle loop.
 * Unknown workers are refused; they are never forcibly stopped mid-job. */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"

/* These primitives may be supplied by a host harness before inclusion. */
#ifndef FM1_FLASH_ICFG_READ
FM1_INLINE uint32_t fm1_flash_icfg_read(void)
{
    uint32_t v;
    __asm__ volatile("%0 = icfg" : "=r"(v) :: "memory");
    return v;
}
#define FM1_FLASH_ICFG_READ() fm1_flash_icfg_read()
#endif
#ifndef FM1_FLASH_ICFG_WRITE
FM1_INLINE void fm1_flash_icfg_write(uint32_t v)
{
    __asm__ volatile("icfg = %0" :: "r"(v) : "memory");
}
#define FM1_FLASH_ICFG_WRITE(v) fm1_flash_icfg_write(v)
#endif
#ifndef FM1_FLASH_SYNC
#define FM1_FLASH_SYNC() __asm__ volatile("csync" ::: "memory")
#endif
#ifndef FM1_FLASH_CORE
FM1_INLINE uint32_t fm1_flash_core(void)
{
    uint32_t v;
    __asm__ volatile("%0 = cnum" : "=r"(v));
    return v;
}
#define FM1_FLASH_CORE() fm1_flash_core()
#endif
#ifndef FM1_FLASH_C1_CON
#define FM1_FLASH_C1_CON (*(volatile uint32_t *)0x1EEE004u)
#endif

static volatile uint32_t fm1_flash_cpu1_idle;

FM1_INLINE uint32_t irq_save(void)
{
    uint32_t f = FM1_FLASH_ICFG_READ();
    /* ICFG bit 8 is the global IRQ enable, as in the AC79 SDK irq.h.
     * Preserve all other bits, including the current interrupt context. */
    FM1_FLASH_ICFG_WRITE(f & ~0x100u);
    FM1_FLASH_SYNC();
    return f;
}

FM1_INLINE void irq_restore(uint32_t f)
{
    FM1_FLASH_SYNC();
    FM1_FLASH_ICFG_WRITE(f);
}

FM1_INLINE int fm1_flash_access_ok(void)
{
    uint32_t c;
    if (FM1_FLASH_CORE() != 0 || (FM1_FLASH_ICFG_READ() & 0x100u))
        return 0;
    FM1_FLASH_SYNC();
    c = FM1_FLASH_C1_CON;
    return !(c & 8u) || (c & 2u) || fm1_flash_cpu1_idle == 1u;
}
