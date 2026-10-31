/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio) */
/* Flash capacity and application-owned write regions. No register accesses.
 * Define FM1_FLASH_WRITE_REGIONS(X) before inclusion, for example:
 *   #define FM1_FLASH_WRITE_REGIONS(X) X(0x97000u, 0xA0000u)
 * Each X(lo, hi) is a half-open region. A request must fit wholly in one.
 * Bounds must be constants: the RAM driver must not fetch a table from XIP.
 * FL_RANGE_OK remains an override for existing firmware and the loader.
 * The legacy Felucca layout below is only the compatibility default. */
#pragma once
#include <stdint.h>

#ifndef FM1_FLASH_SIZE
#define FM1_FLASH_SIZE 0x100000u
#endif
#if FM1_FLASH_SIZE < 0x100000u || FM1_FLASH_SIZE > 0x1000000u
#error "FM1_FLASH_SIZE must be 1..16 MiB (3-byte addressing only)"
#endif
/* Physical capacity does not establish how much the SFC can map. */
#ifndef FM1_FLASH_XIP_SIZE
#define FM1_FLASH_XIP_SIZE 0x100000u
#endif
#ifndef FM1_FLASH_PLAIN_LO
#define FM1_FLASH_PLAIN_LO 0x93000u
#endif
#ifndef FM1_FLASH_PLAIN_HI
#define FM1_FLASH_PLAIN_HI FM1_FLASH_XIP_SIZE
#endif
#if FM1_FLASH_XIP_SIZE > FM1_FLASH_SIZE || FM1_FLASH_PLAIN_LO < 0x4000u || \
    FM1_FLASH_PLAIN_LO >= FM1_FLASH_PLAIN_HI || FM1_FLASH_PLAIN_HI > FM1_FLASH_XIP_SIZE
#error "Invalid flash XIP/plain window"
#endif

#define FL_IN(off, n, lo, hi) ((uint32_t)(hi) >= (uint32_t)(lo) && \
    (uint32_t)(off) >= (uint32_t)(lo) && (uint32_t)(off) <= (uint32_t)(hi) && \
    (uint32_t)(n) <= (uint32_t)(hi) - (uint32_t)(off))

/* Compatibility names; new applications supply their own regions above. */
#define FL_DATA_LO 0x00097000u
#define FL_DATA_HI 0x000E0000u
#define FL_GLOB_LO 0x000FC000u
#define FL_GLOB_HI 0x000FF000u
#define FL_OTA_LO  0x000E0000u
#define FL_OTA_HI  0x000E5000u
#define FL_AUTO_LO 0x000E5000u
#define FL_AUTO_HI 0x000E7000u
#define FL_XDATA0_LO 0x00093000u
#define FL_XDATA0_HI 0x00097000u
#define FL_XDATA1_LO 0x000E7000u
#define FL_XDATA1_HI 0x000FC000u
#define FL_STORE_OK(off, n) (FL_IN(off, n, FL_DATA_LO, FL_DATA_HI) || \
    FL_IN(off, n, FL_GLOB_LO, FL_GLOB_HI) || FL_IN(off, n, FL_AUTO_LO, FL_AUTO_HI))
#define FL_XDATA_OK(off, n) (FL_IN(off, n, FL_XDATA0_LO, FL_XDATA0_HI) || \
    FL_IN(off, n, FL_XDATA1_LO, FL_XDATA1_HI))
#ifndef FL_RANGE_OK
#ifndef FM1_FLASH_WRITE_REGIONS
#define FM1_FLASH_WRITE_REGIONS(X) \
    X(FL_DATA_LO, FL_DATA_HI) X(FL_GLOB_LO, FL_GLOB_HI) \
    X(FL_AUTO_LO, FL_AUTO_HI) X(FL_XDATA0_LO, FL_XDATA0_HI) \
    X(FL_XDATA1_LO, FL_XDATA1_HI) X(FL_OTA_LO, FL_OTA_HI)
#endif
#define FM1_FLASH_REGION_OK(lo, hi) || FL_IN(off, n, lo, hi)
static inline __attribute__((always_inline)) int fm1_flash_region_ok(uint32_t off, uint32_t n)
{
    (void)off; (void)n;                         /* an empty region list is read-only */
    return 0 FM1_FLASH_WRITE_REGIONS(FM1_FLASH_REGION_OK);
}
#undef FM1_FLASH_REGION_OK
#define FL_RANGE_OK(off, n) fm1_flash_region_ok(off, n)
#endif

#define FL_READ_OK(off, n) FL_IN(off, n, 0u, FM1_FLASH_SIZE)
#define FL_WRITE_OK(off, n) (FL_READ_OK(off, n) && FL_RANGE_OK(off, n))
#define FL_PLAIN_OK(off, n) FL_IN(off, n, FM1_FLASH_PLAIN_LO, FM1_FLASH_PLAIN_HI)
#define FL_RAM_OK(p, n) ((uintptr_t)(p) >= 0x01C00000u && \
    (uintptr_t)(p) <= 0x01C80000u && (uint32_t)(n) <= 0x01C80000u - (uintptr_t)(p))

#define FL_ERR_RANGE   (-1)
#define FL_ERR_MODE    (-2)
#define FL_ERR_WEL     (-3)
#define FL_ERR_TIMEOUT (-4)
#define FL_ERR_BUSY    (-5)
#define FL_ERR_BUFFER  (-6)
