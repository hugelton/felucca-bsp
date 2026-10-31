/* SPDX-License-Identifier: MPL-2.0
 * Copyright (c) 2026 Julius Vitkauskas */
/* Completion of an issued erase / page program, for fm1_flash.h's RAM driver and its host model
 * (tests/flash_wait_test.c). The includer supplies RAMINL, fl_rdsr(cmd), T4_CNT (24 MHz, free-running) and
 * fl_reset_busy() (does not return).
 *
 * fl_wait_issued(issued, normal, extended, took): issued is T4_CNT read before the command went out, so the time
 * counts the command's SPI transfer and the cache invalidation too. It returns only once SR1.BUSY reads clear:
 *   - clear within `normal` us: 0;
 *   - clear later (but before `extended`): FL_ERR_LATE_READY, -4 unless the firmware defines it: the code a part
 *     still busy at the normal bound got from fl_wait_ready, so callers see the same codes for the same
 *     cases;
 *   - still busy at `extended` us: fl_reset_busy().
 * *took gets the microseconds to the status read that saw BUSY clear.
 * Before, a part busy at the normal bound made fl_wait_ready return -4 and the driver switched XIP back on while the
 * part was still erasing or programming. A NOR part does not answer reads while it erases or programs (no
 * read-while-write), so the next code or data fetched through XIP was whatever the bus returned. A slow part now
 * finishes first; a stuck one ends in a reset, never in a fetch from it.
 * fm1_flash.h's bounds are generic SPI NOR figures, not measurements of the FM-1's part: normal = the bounds
 * fl_wait_ready had (sector erase 500 ms, page program 10 ms, above the generic NOR maxima of 400 ms and 3 ms),
 * extended = 2 s and 50 ms. What the chip's boot does with a part that stays busy after the reset is not known.
 * The unsigned difference survives one TIMER4 wrap (178.9 s), far beyond either bound. */
#pragma once
#ifndef FL_ERR_LATE_READY
#define FL_ERR_LATE_READY (-4)
#endif
RAMINL int fl_wait_issued(uint32_t issued, uint32_t normal, uint32_t extended, uint32_t *took_us)
{
    for (;;) {
        uint32_t status = fl_rdsr(0x05), dt = (T4_CNT - issued) / 24u;
        if (!(status & 1u)) { *took_us = dt; return dt <= normal ? 0 : FL_ERR_LATE_READY; }
        if (dt >= extended) fl_reset_busy();
    }
}
