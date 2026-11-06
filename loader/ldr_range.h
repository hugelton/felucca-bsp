/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 hdavid */
/* Where the update loader's flash driver lets it write (loader.c's FL_RANGE_OK, and tests/ldr_bounds_test.c's model of the
 * driver): the app area up to the store [FL_DATA_LO, 0xE0000) and the staging area from 0xE0000 up to the globals, never
 * the head, never the store, but for one window [lo, hi) that ldr_core.c opens (ldr_res_window) around the writes it
 * needs there: an app's resource region, and a slot switch's erases (ldr_slot_switch). Needs FL_IN and FL_DATA_LO
 * (chip/fm1_flash_policy.h), used where it expands. */
#ifndef LDR_RANGE_H
#define LDR_RANGE_H
#define LDR_RANGE_OK(off, n, win_lo, win_hi) \
    (FL_IN(off, n, 0x4000u, FL_DATA_LO) || FL_IN(off, n, 0xE0000u, 0xFC000u) || \
     ((win_hi) && FL_IN(off, n, win_lo, win_hi)))
#endif
