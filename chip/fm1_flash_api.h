/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio) */
/* XIP-side wrappers. Included by fm1_flash.h after the RAM entry points;
 * kept separate so host tests exercise the same paging/locking code. */
#pragma once

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
    int rc = FL_FAR(fl_erase4k_ram)(off, took_us);
    irq_restore(f);
    return rc;
}

static int fl_write(uint32_t off, const uint8_t *src_ram, uint32_t n)
{
    uint32_t took;
    /* Validate the WHOLE request before the first page can change. */
    if (!FL_WRITE_OK(off, n)) return FL_ERR_RANGE;
    if (!FL_RAM_OK(src_ram, n)) return FL_ERR_BUFFER;
    while (n) {
        uint32_t chunk = 256u - (off & 0xFFu), f;
        int rc;
        if (chunk > n) chunk = n;
        f = irq_save();
        rc = FL_FAR(fl_prog_ram)(off, src_ram, chunk, &took);
        irq_restore(f);
        if (rc) return rc;
        off += chunk; src_ram += chunk; n -= chunk;
    }
    return 0;
}

static int fl_read(uint32_t off, uint8_t *dst_ram, uint32_t n)
{
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
