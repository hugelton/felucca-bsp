/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Expanded flash, EXPERIMENTAL: what to do when the FM-1's SOIC-8 flash (stock: a 1 MiB part, JEDEC 85 60 14) is
 * replaced by a larger one. Pure logic, no registers: it decides the size and where the extra room is. The
 * read/erase/program entry points stay those of fm1_flash.h; they only need the limit passed in.
 *
 * Standard SPI NOR (Winbond, GigaDevice, Puya, Macronix, ...) puts the capacity in the last JEDEC byte as
 * log2(bytes): 0x14 = 1 MiB, 0x15 = 2, 0x16 = 4, 0x17 = 8, 0x18 = 16 MiB, 0x19 = 32 MiB.
 *   - The 3-byte address of fm1_flash.h reaches 16 MiB. Above that a part needs 4-byte mode: not supported here.
 *   - The first 1 MiB keeps the stock layout (head, app slot, Felucca's regions): the mask-ROM and the SPL read it.
 *   - The extra room [1 MiB, size) is plain data: the application decides what goes there. It is not covered by
 *     the update loader's window.
 *   - Not known yet: how much of the flash the XIP window can map, and whether the quad-enable bit of the new part
 *     is where the stock part has it (the SR2 bit 1 of the common Winbond-style parts).
 * fm1_flash_ext_ok(off, n, size) is the range check of fm1_flash.h's FL_IN, with the size taken from the part. */
#pragma once
#include <stdint.h>

#define FM1_FLASH_STOCK_SIZE 0x100000u       /* 1 MiB: the region the stock layout lives in */
#define FM1_FLASH_MAX_3BYTE  0x1000000u      /* 16 MiB */

/* bytes for a JEDEC id (mfr << 16 | type << 8 | capacity code), 0 when it is not a standard NOR capacity code */
static inline uint32_t fm1_flash_size_from_jedec(uint32_t id)
{
    uint32_t c = id & 0xFFu;
    if (id == 0 || id == 0xFFFFFFu || c < 0x14u || c > 0x19u)       /* 1 MiB .. 32 MiB */
        return 0;
    return 1u << c;
}

/* usable with 3-byte addressing: the size, capped at 16 MiB */
static inline uint32_t fm1_flash_usable(uint32_t size)
{
    return size > FM1_FLASH_MAX_3BYTE ? FM1_FLASH_MAX_3BYTE : size;
}

/* [off, off+n) inside the extra room [1 MiB, usable), without wrapping */
static inline int fm1_flash_ext_ok(uint32_t off, uint32_t n, uint32_t size)
{
    uint32_t hi = fm1_flash_usable(size);
    return hi > FM1_FLASH_STOCK_SIZE && off >= FM1_FLASH_STOCK_SIZE && off <= hi && n <= hi - off;
}
