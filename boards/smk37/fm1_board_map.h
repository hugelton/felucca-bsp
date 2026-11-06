/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Charles Vestal */
/* Every SMK-37 header includes this: the board must have its own flash map (fm1_flash.h here), never the chip's,
 * whose write regions run over BTIF and USRTRIM (the factory calibration) on this board. With -Ichip before
 * -Iboards/smk37, or the chip's map in first, the build stops: here if the chip's map is already in, in
 * chip/fm1_flash_policy.h if it comes after (FL_BOARD_NEEDS_MAP without FL_BOARD_MAP). */
#pragma once
#define FL_BOARD_NEEDS_MAP 1
#if defined(FL_DATA_LO) && !defined(FL_BOARD_MAP)
#error "boards/smk37: the chip's flash map is in, not the board's (-Iboards/smk37 before -Ichip, the board's fm1_flash.h first)"
#endif
