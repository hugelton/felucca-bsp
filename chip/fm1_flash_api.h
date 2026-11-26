/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio)
 * Copyright (C) 2026 hdavid (the sliced erase's wrappers and the hooks, from Optimist) */
/* XIP-side wrappers. Included by fm1_flash.h after the RAM entry points;
 * kept separate so host tests exercise the same paging/locking code. */
#pragma once

#ifndef FM1_FLASH_ENTER                            /* the firmware's step in each window, IRQs off (none by default) */
#define FM1_FLASH_ENTER() ((void)0)
#endif
#ifndef FM1_FLASH_LEAVE                            /* and at its end, before the IRQs are on again */
#define FM1_FLASH_LEAVE() ((void)0)
#endif

static uint32_t fl_jedec(void)
{
    uint32_t f = irq_save(), id = FL_FAR(fl_jedec_ram)();
    irq_restore(f);
    return id;
}

static uint32_t fl_status(void)
{
    uint32_t f = irq_save(), v = FL_FAR(fl_status_ram)();
    irq_restore(f);
    return v;
}

#ifdef FL_UID
static int fl_uid(uint8_t *dst)
{
    uint32_t f = irq_save();
    int rc = FL_FAR(fl_uid_ram)(dst);
    irq_restore(f);
    return rc;
}
#endif

static int fl_erase4k(uint32_t off, uint32_t *took_us)
{
    uint32_t f = irq_save();
    int rc;
    FM1_FLASH_ENTER();
    rc = FL_FAR(fl_erase4k_ram)(off, took_us);
    FM1_FLASH_LEAVE();
    irq_restore(f);
    return rc;
}

#if FL_ERASE_SLICED
/* ---- a sector erase in short windows (fm1_flash_slice.h): suspended every FLS_SLICE_US (1 ms), the interrupts on
 * between. Call it where an fl_erase4k is called, from the main loop (FLS_GAP turns the interrupts on), never from an
 * interrupt.
 *   - The interrupts that run between two windows run from the flash with the erase suspended: the part reads,
 *     except the sector being erased (no code there; a read of that sector meanwhile gives anything).
 *   - The XIP is off in each window, as in fl_write's: a second core running from the flash must have no job
 *     outstanding when one starts (fm1_cpu1.h: the render waits for its job before it returns, so the main loop
 *     never finds one; FM1_FLASH_ENTER can wait for it).
 *   - The P25Q80H needs tERS >= 200 us between a resume and the next suspend to progress: FLS_SLICE_US >= 200.
 *   - A part that does not suspend (or an erase that does not progress in FLS_SLICES_MAX windows) is waited out in one
 *     window, as fl_erase4k: that window silences quiet[0..quiet_n) first (the audio buffer the DMA loops) and
 *     FL_ERASE_HELD() says so. An erase that went through a suspend is read back blank through the plain window
 *     (fl_plain_window_init); not blank, or no window: erased once more in one window (FL_ERASE_HELD() too).
 *   - Each window is irq_save / FM1_FLASH_ENTER ... FM1_FLASH_LEAVE / irq_restore, as each page program.
 * 0 = erased; < 0 as fl_erase4k (FL_ERR_*). took_us: the windows' time, summed (the longest one is about
 * FLS_SLICE_US + 30 us). */
#ifndef FL_ERASE_HELD                              /* an erase that held the interrupts off for the whole of it */
#define FL_ERASE_HELD() ((void)0)
#endif
static int fl_erase_window(uint32_t off, uint32_t op, fls_job_t *j)
{
    uint32_t f, t;
    int rc;
    f = irq_save();
    FM1_FLASH_ENTER();
    t = T4_CNT;
    j->windows++;
    rc = FL_FAR(fl_erase_window_ram)(off, op, j);
    j->took_us += (T4_CNT - t) / 24u;
    FM1_FLASH_LEAVE();
    irq_restore(f);
    return rc;
}
static int fl_blank_xip(uint32_t off)              /* the sector reads all 0xFF through the plain window */
{
    const volatile uint32_t *p = (const volatile uint32_t *)(uintptr_t)FL_XIP(off);
    uint32_t i;
    for (i = 0; i < 1024u; i++)
        if (p[i] != 0xFFFFFFFFu)
            return 0;
    return 1;
}
static int fl_erase4k_sliced(uint32_t off, uint32_t *took_us, int32_t *quiet, uint32_t quiet_n)
{
    fls_job_t j = { FLS_SLICE_US, quiet, quiet_n, 0, 0, 0, 0 };   /* (volatile: the DMA reads it) */
    int rc;
    if (!FL_WRITE_OK(off, 0x1000u) || (off & 0xFFFu))
        return FL_ERR_RANGE;
    rc = fls_erase(off, &j);
    if (j.held)
        FL_ERASE_HELD();
    if (!rc && j.windows > 1u && !((SFCENC_CON & 2u) && off >= FM1_FLASH_PLAIN_LO && fl_blank_xip(off))) {
        fls_job_t k = { 0, quiet, quiet_n, 0, 0, 0, 0 };
        FL_ERASE_HELD();
        rc = fl_erase_window(off, FLS_RESUME, &k);   /* (an erase still suspended: ended first; a part idle ignores it) */
        if (rc >= 0) {
            k.held = 0;                           /* (silenced again: a half may have been rendered since) */
            rc = fls_erase(off, &k);              /* (then once more in one window, as fl_erase4k) */
        }
        j.took_us += k.took_us;
    }
    *took_us = j.took_us;
    return rc < 0 ? rc : 0;
}
#endif

static int fl_write(uint32_t off, const uint8_t *src_ram, uint32_t n)
{
    uint32_t took, o, left;
    if (!n) return 0;                           /* nothing to write: 0, whatever off and src_ram are */
    /* Validate the WHOLE request before the first page can change: inside the part, without wrapping, and every
     * page inside a write region, as fl_prog_ram checks it. A write may run across regions that touch (the
     * defaults' AUTO and XDATA1 at 0xE7000), as it could when each page was checked only as it was written. */
    if (!FL_READ_OK(off, n)) return FL_ERR_RANGE;
    for (o = off, left = n; left; ) {
        uint32_t chunk = 256u - (o & 0xFFu);
        if (chunk > left) chunk = left;
        if (!FL_WRITE_OK(o, chunk)) return FL_ERR_RANGE;
        o += chunk; left -= chunk;
    }
    if (!FL_RAM_OK(src_ram, n)) return FL_ERR_BUFFER;
    while (n) {
        uint32_t chunk = 256u - (off & 0xFFu), f;
        int rc;
        if (chunk > n) chunk = n;
        f = irq_save();
        FM1_FLASH_ENTER();
        rc = FL_FAR(fl_prog_ram)(off, src_ram, chunk, &took);
        FM1_FLASH_LEAVE();
        irq_restore(f);
        if (rc) return rc;
        off += chunk; src_ram += chunk; n -= chunk;
    }
    return 0;
}

static int fl_read(uint32_t off, uint8_t *dst_ram, uint32_t n)
{
    if (!n) return 0;                           /* as fl_write */
    if (!FL_READ_OK(off, n)) return FL_ERR_RANGE;
    if (!FL_RAM_OK(dst_ram, n)) return FL_ERR_BUFFER;
    while (n) {
        uint32_t chunk = n > 256u ? 256u : n, f = irq_save();
        int rc = FL_FAR(fl_read_ram)(off, dst_ram, chunk);
        irq_restore(f);
        if (rc) return rc;
        off += chunk; dst_ram += chunk; n -= chunk;
    }
    return 0;
}

/* Plain-window XIP reads are only valid within the explicitly mapped area. */
static int fl_verify_xip(uint32_t off, const uint8_t *ref, uint32_t n)
{
    const volatile uint8_t *p;
    uint32_t i;
    if (!FL_PLAIN_OK(off, n)) return FL_ERR_RANGE;
    if (!FL_RAM_OK(ref, n)) return FL_ERR_BUFFER;
    p = (const volatile uint8_t *)(uintptr_t)FL_XIP(off);
    for (i = 0; i < n; i++) if (p[i] != ref[i]) return -(int)(i + 1);
    return 0;
}
