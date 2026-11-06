/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio) */
/* The update loader on the SMK-37 (loader.c built with -DLDR_SMK37 -DLDR_MODEL="SMK-37 Elite" and -Iboards/smk37
 * before -Ichip, as fm1-base's tools/build.py does for board = "smk37"): its identity, its windows from the board's
 * flash map (boards/smk37/fm1_flash.h), its staging, and the layout check before any erase (ldr_core.c 2a).
 * tests/ldr_bounds_test.c runs ldr_core.c and ota.c with the same. Included before the map: its names are used only
 * where these are expanded.
 *
 * The app area [0x4000, 0x9D000) and update records only in the stock VM above it, up to BTIF (0xC0000), as the
 * reference port does; never BTIF, USRTRIM, the store, USR nor key_mac. Its staging (ota.c's OTA_AREA, the record
 * at 0xBFF00) is the board's, inside that window. */
#pragma once
#define FELUCCA_ID "ota-" LDR_MODEL "_900"           /* "ota-SMK-37 Pro_900", "ota-SMK-37 Elite_900", "ota-MKE-P37_900" */
#define FM1_INPUT_TRIM 0                            /* (no factory curves needed: the buttons only) */
#define FL_RANGE_OK(off, n) FL_LOADER_RANGE_OK(off, n)   /* the board map's: the app area, the records up to BTIF */
#define OTA_AREA FL_OTA_LO
#define LDR_APP_HI FL_APP_HI
#define LDR_REC_LO FL_REC_LO
#define LDR_REC_HI FL_REC_HI
#define LDR_DATA_LO FL_DATA_LO
#define LDR_DATA_HI FL_DATA_HI
#define LDR_LAYOUT 1                                /* refuse a package not laid out for the SMK before any erase */
#define LDR_APP_SLOT 0x97FBCu                       /* app.bin's slot (fm1-base tools/fm1pkg_make.py: the FM-1's 0x8DFBC + 0xA000) */
#define LDR_VM_OFF FL_APP_HI                        /* the package's VM descriptor: the stock SMK's settings store, 0x9D000 */
/* (checked where the map is in: loader.c, tests/ldr_bounds_test.c) */
#define LDR_SMK37_WINDOWS_OK (FL_REC_LO == FL_APP_HI && FL_REC_HI == FL_PROT1_LO && FL_OTA_LO >= FL_REC_LO && \
                              FL_OTA_HI <= FL_REC_HI && sizeof FELUCCA_ID <= 24u)
