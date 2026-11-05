/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio) */
/* The SMK-37's flash map (boards/smk37/fm1_flash.h) under the chip's policy (chip/fm1_flash_policy.h): an app build's
 * default write regions take every window of the map and the stock marker sector, a request wholly inside one, and
 * refuse the boot head, the app area, BTIF, USRTRIM, USR and key_mac; 0xEB000..0xF3FFF (the settings pair and
 * FL_FREE) is one run of writable sectors; the update loader's window (FL_LOADER_RANGE_OK, loader/ldr_smk37.h: the
 * app area and the stock VM up to BTIF) takes the app area and the records, never BTIF, USRTRIM, the store, USR nor
 * key_mac; the plain XIP window starts where the app area ends. The map's own _Static_asserts compile here too.
 *   cc -std=c99 -Iboards/smk37 -Ichip tests/smk37_flash_map_test.c */
#include <stdio.h>
#include <stdint.h>
#define FL_MAP_ONLY 1                                    /* the map and the policy, without the RAM driver */
#include "fm1_flash.h"

#ifndef FL_BOARD_MAP
#error "boards/smk37/fm1_flash.h not first on the include path"
#endif
static int app_ok(uint32_t off, uint32_t n) { return FL_RANGE_OK(off, n); }
static int ldr_ok(uint32_t off, uint32_t n) { return FL_LOADER_RANGE_OK(off, n); }   /* loader/ldr_smk37.h's */

static int bad;
static void check(const char *what, int ok)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    bad += !ok;
}

int main(void)
{
    static const struct { const char *name; uint32_t lo, hi; } WIN[] = {
        {"FL_XDATA0", FL_XDATA0_LO, FL_XDATA0_HI}, {"FL_AUTO", FL_AUTO_LO, FL_AUTO_HI},
        {"FL_XDATA1", FL_XDATA1_LO, FL_XDATA1_HI}, {"FL_OTA", FL_OTA_LO, FL_OTA_HI},
        {"FL_DATA", FL_DATA_LO, FL_DATA_HI}, {"FL_GLOB", FL_GLOB_LO, FL_GLOB_HI}, {"FL_FREE", FL_FREE_LO, FL_FREE_HI},
        {"FL_STOCK_MARK", FL_STOCK_MARK, FL_STOCK_MARK + FL_SECTOR}};
    static const struct { const char *name; uint32_t lo, hi; } NEVER[] = {
        {"the boot head", FL_PROT0_LO, FL_PROT0_HI}, {"the app area", FL_APP_LO, FL_APP_HI},
        {"BTIF + USRTRIM", FL_PROT1_LO, FL_PROT1_HI}, {"USR", FL_PROT2_LO, FL_PROT2_HI},
        {"key_mac", FL_PROT3_LO, FL_PROT3_HI}};
    uint32_t i, s, ok;
    char b[96];
    for (i = 0; i < sizeof WIN / sizeof WIN[0]; i++) {
        for (s = WIN[i].lo, ok = 1; s < WIN[i].hi; s += FL_SECTOR)
            ok &= app_ok(s, FL_SECTOR) && app_ok(s + 0xF00u, 0x100u);
        snprintf(b, sizeof b, "an app may erase and write %s, sector by sector", WIN[i].name);
        check(b, (int)ok);
        snprintf(b, sizeof b, "  %s: a request across its end is refused", WIN[i].name);
        check(b, !app_ok(WIN[i].hi - 1u, 2u) && !app_ok(WIN[i].lo - 1u, 2u));
    }
    check("a request across two touching windows is refused (FL_AUTO into FL_XDATA1)", !app_ok(FL_AUTO_HI - 0x100u, 0x200u));
    for (s = 0xEB000u, ok = 1; s < 0xF4000u; s += FL_SECTOR)
        ok &= app_ok(s, FL_SECTOR);
    check("0xEB000..0xF3FFF, the settings pair and FL_FREE up to USR, every sector writable (Melodee's store C)",
          (int)ok && FL_FREE_HI == FL_PROT2_LO && !app_ok(FL_PROT2_LO, 1u));
    check("a request that wraps is refused", !app_ok(0xFFFFFF00u, 0x200u) && !app_ok(FL_DATA_LO, 0xFFFFFFFFu));
    for (i = 0; i < sizeof NEVER / sizeof NEVER[0]; i++) {
        for (s = NEVER[i].lo, ok = 1; s < NEVER[i].hi; s += FL_SECTOR)
            ok &= !app_ok(s, FL_SECTOR) && !app_ok(s + 0xFFFu, 1u);
        snprintf(b, sizeof b, "an app never touches %s", NEVER[i].name);
        check(b, (int)ok);
    }
    for (s = 0, ok = 1; s < FL_FLASH_SIZE; s += FL_SECTOR) {
        int in_app = s >= FL_APP_LO && s < FL_APP_HI, rec = s >= FL_REC_LO && s < FL_REC_HI;
        int never = s < FL_PROT0_HI || (s >= FL_PROT1_LO && s < FL_PROT1_HI) || (s >= FL_PROT2_LO && s < FL_PROT2_HI) ||
                    s >= FL_PROT3_LO || (s >= FL_DATA_LO && s < FL_DATA_HI);
        if (ldr_ok(s, FL_SECTOR) != (in_app || rec) || (never && ldr_ok(s, 1u)))
            ok = 0;
    }
    check("the loader's window: the app area and the stock VM's records, never BTIF, USRTRIM, the store, USR, key_mac",
          (int)ok);
    check("the plain XIP window starts at the app area's end (0x9D000)",
          FM1_FLASH_PLAIN_LO == FL_APP_HI && FL_PLAIN_OK(FL_USRTRIM, 0x228u) && !FL_PLAIN_OK(FL_APP_HI - 1u, 1u));
    check("the FM-1's parts and the board's are accepted", FL_JEDEC_OK(0x856014u) && FL_JEDEC_OK(0xB36014u) &&
          FL_JEDEC_OK(FL_JEDEC_ID) && !FL_JEDEC_OK(0xEF4014u));
    printf("%s\n", bad ? "SMK37 FLASH MAP TEST FAILED" : "smk37 flash map test passed");
    return bad != 0;
}
