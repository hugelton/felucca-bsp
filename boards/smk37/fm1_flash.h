/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio) */
/* SMK-37 (Pro / Elite / MKE-P37) flash map, over the chip's SPI NOR driver (chip/fm1_flash.h, included at the
 * end: this header shadows it on the include path, -Iboards/smk37 before -Ichip). It gives the chip's flash gate
 * (chip/fm1_flash_policy.h, docs/FLASH.md) this board's plain window and write regions, and the FM-1's names for
 * them (FL_DATA_LO, FL_OTA_LO, ...) at this board's places.
 *
 * The part is 1 MiB (JEDEC 0xEB6014; the FM-1's is 0x856014: verified on an Elite running Melodee). The stock
 * v016 layout, from M. Geisler's analysis of the stock firmware (fm1-to-smk37 README and docs/01-hardware.md §2,
 * the region table marked analysis, the protected ranges the README's rule):
 *   0x00000-0x03FFF  flash header, second-stage bootloader, boot config   NEVER WRITTEN
 *   0x04000-0x9CFFF  the app area (PRCT): app head, app.bin from 0x4120, the config blobs; the update loader is
 *                    its only writer. 0x9D000 bytes with the head, 0xA000 more than the FM-1's 0x93000
 *   0x9D000-0xBFFFF  the stock VM (its settings store): ours below
 *   0xC0000-0xC0FFF  BTIF: Bluetooth pairing                                NEVER WRITTEN
 *   0xC1000-0xC1FFF  USRTRIM: the factory calibration record                NEVER WRITTEN, read for the curves
 *   0xC2000-0xEAFFF  the stock USRFLASH (its presets): ours below
 *   0xF4000-0xFDFFF  USR: the stock resources                               NEVER WRITTEN
 *   0xFF000-0xFFFFF  key_mac: the chip key and MAC                          NEVER WRITTEN
 * Everything else (0xEB000-0xF3FFF, 0xFE000-0xFEFFF) is free to the stock table.
 *
 * This firmware's windows (the same names as the FM-1's, so device/ and an app's store map compile unchanged):
 *   FL_XDATA0   0x9D000-0xA0FFF   16 KiB just above the app area (an app's big data: plat.h PLAT_XDATA0)
 *   FL_AUTO     0xA1000-0xA2FFF   the autosave pair (storage.c OBJ_AUTOSAVE); with the next, plat.h's PLAT_XDATA1
 *   FL_XDATA1   0xA3000-0xBAFFF   96 KiB
 *   FL_OTA      0xBB000-0xBFFFF   the update loader's staging, 5 sectors (chip/ota.c), its record at 0xBFF00
 *   FL_DATA     0xC2000-0xE9FFF   160 KiB, 40 sectors: the store (an app's objects, APP_STORE_MAP) and its data
 *   FL_STOCK_MARK 0xEA000         the stock firmware's marker sector: RELEASED (erased) once at this firmware's
 *                                 first boot, before anything is written, so a return to the stock package
 *                                 rebuilds the stock mapping (as Melodee does). Never written otherwise
 *   FL_GLOB     0xEB000-0xEDFFF   the settings pair (Felucca's superblock / globals; the FM-1's is 0xFC000, inside
 *                                 USR here)
 *   FL_FREE     0xEE000-0xF3FFF   24 KiB the stock table leaves unlisted, below USR: an app's own data. It touches
 *                                 FL_GLOB, so 0xEB000-0xF3FFF is one run of writable sectors (Melodee's store C).
 *                                 The update loader never looks there for records: no 0xF00 rule
 * The update records (4 KiB boundary - 256, bsp/loader/ldr_core.c) are looked for only in the stock VM, 0x9D000 up to
 * BTIF at 0xC0000, as the reference port does, which stages its own at 0xBB000-0xBFFFF too.
 *   FL_OWNER    0xAF000           who owns the store: "SMK37DATA" + a firmware family, the same sector in every
 *                                 port, so no firmware parses another's data. Not ours: the windows are cleared first
 *
 * USRTRIM (0xC1000, read only; the stock firmware writes defaults when its marker is wrong or a curve does not
 * rise): +0x008 the 16 pad curves (5 x u16 each), +0x0A8 the 37 key velocity curves (5 x u16 each), +0x21A the
 * pedal curve, +0x224 the marker "trimdat\0". fm1_input.h reads the curves through the plain XIP window.
 *
 * Nothing here has been checked on a unit by this layer yet (fm1-base's SMK-37 port, step 2). Every window is
 * checked at compile time against the protected ranges, the app area and each other. */
#pragma once
#include <stdint.h>

/* the chip's flash policy must not be in before this map: it would hold the FM-1's names and write regions (its store
 * runs over BTIF and USRTRIM here). chip/fm1_clock.h includes the chip's fm1_flash.h by name, from its own directory:
 * include this header first */
#ifdef FL_DATA_LO
#error "boards/smk37/fm1_flash.h: include it before any chip flash header (fm1_flash.h, fm1_clock.h): the FM-1's map is in"
#endif
#define FL_BOARD_MAP 1                                /* the flash gate takes these instead of the FM-1's */
#define FL_JEDEC_ID     0x00EB6014u                   /* the part named in the board notes */
/* the parts found on these keyboards: units differ, so a build must not lock itself to one (a part it refuses
 * leaves the store RAM-only and the app refusing updates) */
#define FL_JEDEC_OK(id) ((id) == 0x00EB6014u || (id) == 0x00856014u || (id) == 0x00B36014u)
#define FL_FLASH_SIZE   0x00100000u
#define FL_SECTOR       0x1000u
#define FL_APP_LO       0x00004000u                   /* the app area: the update loader's window */
#define FL_APP_HI       0x0009D000u
#define FL_PROT0_LO     0x00000000u                   /* the boot head */
#define FL_PROT0_HI     0x00004000u
#define FL_PROT1_LO     0x000C0000u                   /* BTIF and USRTRIM */
#define FL_PROT1_HI     0x000C2000u
#define FL_PROT2_LO     0x000F4000u                   /* USR */
#define FL_PROT2_HI     0x000FE000u
#define FL_PROT3_LO     0x000FF000u                   /* key_mac */
#define FL_PROT3_HI     0x00100000u
#define FL_BTIF         0x000C0000u
#define FL_USRTRIM      0x000C1000u
#define FL_TRIM_PADS    (FL_USRTRIM + 0x008u)         /* 16 x 5 x u16 */
#define FL_TRIM_KEYS    (FL_USRTRIM + 0x0A8u)         /* 37 x 5 x u16 */
#define FL_TRIM_PEDAL   (FL_USRTRIM + 0x21Au)         /* 5 x u16 */
#define FL_TRIM_MARK    (FL_USRTRIM + 0x224u)         /* "trimdat\0" */
#define FL_XDATA0_LO    0x0009D000u
#define FL_XDATA0_HI    0x000A1000u
#define FL_AUTO_LO      0x000A1000u
#define FL_AUTO_HI      0x000A3000u
#define FL_XDATA1_LO    0x000A3000u
#define FL_XDATA1_HI    0x000BB000u
#define FL_OTA_LO       0x000BB000u
#define FL_OTA_HI       0x000C0000u
#define FL_DATA_LO      0x000C2000u
#define FL_DATA_HI      0x000EA000u
#define FL_STOCK_MARK   0x000EA000u
#define FL_GLOB_LO      0x000EB000u
#define FL_GLOB_HI      0x000EE000u
#define FL_FREE_LO      0x000EE000u
#define FL_FREE_HI      0x000F4000u
#define FL_REC_LO       0x0009D000u                   /* where update records may lie (the loader clears them) */
#define FL_REC_HI       0x000C0000u
#define FL_OWNER        0x000AF000u                   /* the store's owner tag, inside FL_XDATA1 */
/* the update loader's window (loader/ldr_smk37.h): the app area and the records in the stock VM above it, up to BTIF;
 * never BTIF, USRTRIM, the store, the settings, USR nor key_mac */
#define FL_LOADER_RANGE_OK(off, n) FL_IN(off, n, FL_APP_LO, FL_REC_HI)
/* the flash gate (chip/fm1_flash_policy.h): the unencrypted XIP window starts where the app area ends (fm1_input.h
 * reads USRTRIM through it); the part and the XIP map are the chip's 1 MiB */
#define FM1_FLASH_PLAIN_LO FL_APP_HI
/* Where the RAM driver may erase / program (FL_IN, FL_RANGE_OK: the chip's policy): an app build only its own
 * data regions and the stock marker sector (released once); an app may name fewer before this header. The update
 * loader (loader/ldr_smk37.h) has its own window (FL_LOADER_RANGE_OK, as its FL_RANGE_OK) */
#ifndef FM1_FLASH_WRITE_REGIONS
#define FM1_FLASH_WRITE_REGIONS(X) \
    X(FL_DATA_LO, FL_DATA_HI) X(FL_GLOB_LO, FL_GLOB_HI) X(FL_AUTO_LO, FL_AUTO_HI) \
    X(FL_XDATA0_LO, FL_XDATA0_HI) X(FL_XDATA1_LO, FL_XDATA1_HI) X(FL_OTA_LO, FL_OTA_HI) \
    X(FL_STOCK_MARK, FL_STOCK_MARK + FL_SECTOR) X(FL_FREE_LO, FL_FREE_HI)
#endif
#include "../../chip/fm1_flash_policy.h"            /* FL_IN, FL_RANGE_OK, FL_WRITE_OK (no registers: FL_MAP_ONLY too) */
/* the firmware's own store (projects, user samples; settings; the autosave) */
#define FL_STORE_OK(off, n) (FL_IN(off, n, FL_DATA_LO, FL_DATA_HI) || FL_IN(off, n, FL_GLOB_LO, FL_GLOB_HI) || \
                             FL_IN(off, n, FL_AUTO_LO, FL_AUTO_HI))
/* more flash for a firmware's own data, outside the store. The update loader clears an update record it finds at
 * a 4 KiB boundary - 256 here (ldr_core.c: "AT" at bytes 6..7 and a valid CRC), so a firmware keeps bytes
 * 0xF00..0xF07 of each of these sectors erased (or anything but a record) */
#define FL_XDATA_OK(off, n) (FL_IN(off, n, FL_XDATA0_LO, FL_XDATA0_HI) || FL_IN(off, n, FL_XDATA1_LO, FL_XDATA1_HI))
#define FL_PROTECTED(off, n) (FL_IN(off, n, FL_PROT0_LO, FL_PROT0_HI) || FL_IN(off, n, FL_PROT1_LO, FL_PROT1_HI) || \
                              FL_IN(off, n, FL_PROT2_LO, FL_PROT2_HI) || FL_IN(off, n, FL_PROT3_LO, FL_PROT3_HI))

/* every window: sector aligned, inside the part, above the app area, clear of the protected ranges and of each
 * other. A window that moves must pass these before it is built */
#define FL__ALIGNED(lo, hi) (((lo) & (FL_SECTOR - 1u)) == 0u && ((hi) & (FL_SECTOR - 1u)) == 0u && (lo) < (hi))
#define FL__CLEAR(lo, hi) ((hi) <= FL_APP_HI ? 0 : (lo) >= FL_APP_HI && (hi) <= FL_FLASH_SIZE && \
                           ((hi) <= FL_PROT1_LO || (lo) >= FL_PROT1_HI) && ((hi) <= FL_PROT2_LO || (lo) >= FL_PROT2_HI) && \
                           ((hi) <= FL_PROT3_LO || (lo) >= FL_PROT3_HI))
#define FL__APART(a0, a1, b0, b1) ((a1) <= (b0) || (b1) <= (a0))
#define FL__WIN(name) _Static_assert(FL__ALIGNED(name##_LO, name##_HI) && FL__CLEAR(name##_LO, name##_HI), \
                                     #name " is not a sector-aligned window clear of the app area and the protected ranges")
FL__WIN(FL_XDATA0);
FL__WIN(FL_XDATA1);
FL__WIN(FL_AUTO);
FL__WIN(FL_OTA);
FL__WIN(FL_DATA);
FL__WIN(FL_GLOB);
FL__WIN(FL_FREE);
_Static_assert(FL__CLEAR(FL_STOCK_MARK, FL_STOCK_MARK + FL_SECTOR), "the stock marker sector is in a protected range");
_Static_assert(FL_PROT0_HI == FL_APP_LO && FL_APP_HI == FL_XDATA0_LO, "the app area follows the boot head, the windows the app area");
_Static_assert(FL__APART(FL_XDATA0_LO, FL_XDATA0_HI, FL_XDATA1_LO, FL_XDATA1_HI) &&
               FL__APART(FL_XDATA0_LO, FL_XDATA0_HI, FL_AUTO_LO, FL_AUTO_HI) &&
               FL__APART(FL_AUTO_LO, FL_AUTO_HI, FL_XDATA1_LO, FL_XDATA1_HI) &&
               FL__APART(FL_XDATA1_LO, FL_XDATA1_HI, FL_OTA_LO, FL_OTA_HI) &&
               FL__APART(FL_OTA_LO, FL_OTA_HI, FL_DATA_LO, FL_DATA_HI) &&
               FL__APART(FL_DATA_LO, FL_DATA_HI, FL_GLOB_LO, FL_GLOB_HI) &&
               FL__APART(FL_DATA_LO, FL_DATA_HI, FL_STOCK_MARK, FL_STOCK_MARK + FL_SECTOR) &&
               FL__APART(FL_GLOB_LO, FL_GLOB_HI, FL_STOCK_MARK, FL_STOCK_MARK + FL_SECTOR) &&
               FL__APART(FL_XDATA0_LO, FL_XDATA0_HI, FL_DATA_LO, FL_DATA_HI) &&
               FL__APART(FL_XDATA1_LO, FL_XDATA1_HI, FL_DATA_LO, FL_DATA_HI) &&
               FL__APART(FL_OTA_LO, FL_OTA_HI, FL_GLOB_LO, FL_GLOB_HI) &&
               FL__APART(FL_GLOB_LO, FL_GLOB_HI, FL_FREE_LO, FL_FREE_HI) &&
               FL__APART(FL_DATA_LO, FL_DATA_HI, FL_FREE_LO, FL_FREE_HI) &&
               FL__APART(FL_STOCK_MARK, FL_STOCK_MARK + FL_SECTOR, FL_FREE_LO, FL_FREE_HI), "two windows overlap");
_Static_assert(FL_GLOB_HI == FL_FREE_LO && FL_FREE_HI == FL_PROT2_LO, "FL_FREE: from the settings pair up to USR");
_Static_assert(FL_OTA_HI - FL_OTA_LO == 5u * FL_SECTOR, "the update's staging is 5 sectors (chip/ota.c)");
_Static_assert(FL_OWNER >= FL_XDATA1_LO && FL_OWNER + FL_SECTOR <= FL_XDATA1_HI, "the owner tag sits in FL_XDATA1");
_Static_assert(FL_AUTO_HI == FL_XDATA1_LO, "the autosave pair sits just under FL_XDATA1 (plat.h PLAT_XDATA1 spans both)");
_Static_assert(FL_USRTRIM >= FL_PROT1_LO && FL_TRIM_MARK + 8u <= FL_PROT1_HI, "USRTRIM is inside its protected window");
_Static_assert(FL_REC_LO >= FL_APP_HI && FL_REC_HI <= FL_PROT2_LO, "update records: above the app area, below USR");
_Static_assert(FL_FLASH_SIZE == FM1_FLASH_SIZE && FL_FLASH_SIZE == FM1_FLASH_XIP_SIZE &&
               FM1_FLASH_PLAIN_HI == FM1_FLASH_XIP_SIZE, "the 1 MiB part, all of it mapped, plain from the app area's end");

#ifndef FL_MAP_ONLY                               /* (a host test of the map alone: the driver is the chip's) */
#include "../../chip/fm1_flash.h"
#endif
