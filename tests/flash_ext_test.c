/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Host test of chip/fm1_flash_ext.h. */
#include <stdio.h>
#include "fm1_flash_ext.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

int main(void)
{
    CHECK(fm1_flash_size_from_jedec(0x856014u) == 0x100000u);      /* the stock Puya 1 MiB */
    CHECK(fm1_flash_size_from_jedec(0xEF4016u) == 0x400000u);      /* Winbond 4 MiB */
    CHECK(fm1_flash_size_from_jedec(0xEF4018u) == 0x1000000u);     /* 16 MiB */
    CHECK(fm1_flash_size_from_jedec(0xC84019u) == 0x2000000u);     /* 32 MiB */
    CHECK(fm1_flash_size_from_jedec(0) == 0);
    CHECK(fm1_flash_size_from_jedec(0xFFFFFFu) == 0);              /* no part answered */
    CHECK(fm1_flash_size_from_jedec(0xEF4013u) == 0);              /* 512 KiB: below the stock layout */
    CHECK(fm1_flash_size_from_jedec(0xEF401Au) == 0);              /* not handled */
    CHECK(fm1_flash_usable(0x2000000u) == 0x1000000u);
    CHECK(fm1_flash_usable(0x400000u) == 0x400000u);
    /* the stock part has no extra room */
    CHECK(!fm1_flash_ext_ok(0x100000u, 0x1000u, 0x100000u));
    /* a 4 MiB part */
    CHECK(fm1_flash_ext_ok(0x100000u, 0x1000u, 0x400000u));
    CHECK(fm1_flash_ext_ok(0x3FF000u, 0x1000u, 0x400000u));
    CHECK(!fm1_flash_ext_ok(0x3FF000u, 0x1001u, 0x400000u));       /* one byte past the end */
    CHECK(!fm1_flash_ext_ok(0xFF000u, 0x2000u, 0x400000u));        /* starts inside the stock layout */
    CHECK(!fm1_flash_ext_ok(0xFFFFFFFFu, 2u, 0x400000u));          /* off + n wraps */
    CHECK(!fm1_flash_ext_ok(0x400000u, 1u, 0x400000u));
    CHECK(fm1_flash_ext_ok(0x400000u, 0u, 0x400000u));             /* empty range at the end */
    /* a 32 MiB part is capped at 16 MiB */
    CHECK(fm1_flash_ext_ok(0xFFF000u, 0x1000u, 0x2000000u));
    CHECK(!fm1_flash_ext_ok(0x1000000u, 1u, 0x2000000u));
    printf(fails ? "%d failed\n" : "flash_ext: all checks passed\n", fails);
    return fails != 0;
}
