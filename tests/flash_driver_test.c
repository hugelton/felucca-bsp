/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio) */
/* Actual low-level driver, with register/CPU primitives replaced. Verify that
 * invalid requests and unsafe contexts return before touching the bus. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t cfg, core, c1, syncs, flushes;
static uint32_t regs[9];
static uint8_t enc;
static volatile uint32_t *reg32(uint32_t a)
{
    switch (a) {
    case 0x40200u: return &regs[0]; /* SFC */
    case 0x11C00u: return &regs[1]; /* SPI */
    case 0x5101Cu: return &regs[2]; /* pin mapping */
    case 0x500C0u: return &regs[3]; /* CS */
    case 0x500C8u: return &regs[4]; /* direction */
    case 0x1EEE008u: return &regs[5]; /* cache */
    case 0x10804u: return &regs[6]; /* timer */
    case 0x40308u: return &regs[7]; /* encryption high */
    case 0x4030Cu: return &regs[8]; /* encryption low */
    default: fprintf(stderr, "unexpected register access: %x\n", a); abort();
    }
}
#define REG32(a) (*reg32(a))
#define FM1_FLASH_REG8(a) enc
#define FM1_FLASH_FLUSHINV(a) ((void)(a), (void)++flushes)
#define FM1_FLASH_ICFG_READ() cfg
#define FM1_FLASH_ICFG_WRITE(v) (cfg = (v))
#define FM1_FLASH_SYNC() ((void)++syncs)
#define FM1_FLASH_CORE() core
#define FM1_FLASH_C1_CON c1
#define FL_UID 1
/* Mach-O does not accept the target's ELF section name. */
#define RAMFN __attribute__((noinline))
#include "fm1_flash.h"

static unsigned checks, fails;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL %d: %s\n", __LINE__, #c); fails++; } } while (0)
#define RAM(p) ((uint8_t *)(uintptr_t)(p))
#define TOOK ((uint32_t *)RAM(0x1C10000u))

static void reset(void)
{
    unsigned i;
    for (i = 0; i < 9; i++) regs[i] = 0x12300000u + i;
    regs[5] = 0x4000u;
    enc = 0xA1u;
    cfg = core = c1 = syncs = flushes = fm1_flash_cpu1_idle = 0;
}

static void refused(int expected)
{
    uint32_t before[9], state = cfg;
    uint8_t before_enc = enc;
    memcpy(before, regs, sizeof before);
    CHECK(fl_erase4k_ram(0x97000u, TOOK) == expected);
    CHECK(fl_prog_ram(0x97000u, RAM(0x1C20000u), 16, TOOK) == expected);
    CHECK(fl_read_ram(0, RAM(0x1C20000u), 16) == expected);
    CHECK(fl_uid_ram(RAM(0x1C20000u)) == expected);
    CHECK(fl_jedec_ram() == 0);
    CHECK(fl_status_ram() == UINT32_MAX);
    CHECK(memcmp(before, regs, sizeof before) == 0 && enc == before_enc);
    CHECK(cfg == state && flushes == 0);
}

int main(void)
{
    uint32_t before[9];
    unsigned i;
    reset(); cfg = 0x100u;
    refused(FL_ERR_BUSY); /* IRQs still enabled */
    reset(); core = 1;
    refused(FL_ERR_BUSY); /* wrong core */
    reset(); c1 = 8;
    refused(FL_ERR_BUSY); /* CPU1 running with no idle acknowledgement */
    CHECK(fl_plain_window_init() == FL_ERR_BUSY && enc == 0xA1u);
    reset(); regs[0] = 6u << 8;
    refused(FL_ERR_MODE); /* unsupported XIP mode */
    reset();
    memcpy(before, regs, sizeof before);
    CHECK(fl_erase4k_ram(0x97001u, TOOK) == FL_ERR_RANGE);
    CHECK(fl_erase4k_ram(0x4000u, TOOK) == FL_ERR_RANGE);
    CHECK(fl_erase4k_ram(0x97000u, 0) == FL_ERR_BUFFER);
    CHECK(fl_prog_ram(0x970F0u, RAM(0x1C20000u), 17u, TOOK) == FL_ERR_RANGE);
    CHECK(fl_prog_ram(0x97000u, RAM(0x1C20000u), 0u, TOOK) == FL_ERR_RANGE);
    CHECK(fl_prog_ram(0x97000u, RAM(0x1C20000u), 257u, TOOK) == FL_ERR_RANGE);
    CHECK(fl_prog_ram(0x97000u, RAM(0x2000000u), 16u, TOOK) == FL_ERR_BUFFER);
    CHECK(fl_prog_ram(0x97000u, RAM(0x1C20000u), 16u, 0) == FL_ERR_BUFFER);
    CHECK(fl_read_ram(0xFFFFFu, RAM(0x1C20000u), 2u) == FL_ERR_RANGE);
    CHECK(fl_read_ram(UINT32_MAX, RAM(0x1C20000u), 2u) == FL_ERR_RANGE);
    CHECK(fl_read_ram(0, RAM(0x1C7FFFFu), 2u) == FL_ERR_BUFFER);
    CHECK(fl_uid_ram(RAM(0x1C7FFFFu)) == FL_ERR_BUFFER);
    CHECK(memcmp(before, regs, sizeof before) == 0 && flushes == 0);
    for (i = 0; i < 2; i++) {
        uint32_t state = i ? 0x5B5u : 0x4B5u;
        reset(); cfg = state;
        CHECK(fl_plain_window_init() == 0 && cfg == state);
        CHECK(enc == 0xA3u && regs[8] == FL_XIP(FM1_FLASH_PLAIN_LO));
        CHECK(regs[7] == (FM1_FLASH_PLAIN_HI == FM1_FLASH_XIP_SIZE ?
                          0x07FFFFFFu : FL_XIP(FM1_FLASH_PLAIN_HI) - 1u));
        CHECK(flushes == (FM1_FLASH_PLAIN_HI - FM1_FLASH_PLAIN_LO) / 32u);
    }
    reset();
    fl_inval(FM1_FLASH_XIP_SIZE, 4096u);
    CHECK(flushes == 0);
    fl_inval(FM1_FLASH_XIP_SIZE - 32u, 4096u);
    CHECK(flushes == 1); /* clip invalidation to the mapped capacity */
    printf("flash driver: %u checks, %u failures\n", checks, fails);
    return fails != 0;
}
