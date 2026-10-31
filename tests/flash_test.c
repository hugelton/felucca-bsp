/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio) */
/* Actual policy, IRQ/CPU1 gate and wrapper code with a simulated RAM driver.
 * Run in default, custom-region, loader-override and expanded-capacity modes;
 * tests/run_flash_tests.sh also checks invalid configurations. */
#include <stdint.h>
#include <stdio.h>

#if TEST_LAYOUT == 1
#define FM1_FLASH_WRITE_REGIONS(X) X(0x98000u, 0x99000u) X(0x9A000u, 0x9B000u)
#elif TEST_LAYOUT == 2
#define FL_RANGE_OK(off, n) (FL_IN(off, n, 0x4000u, 0x97000u) || FL_IN(off, n, 0xE0000u, 0xFC000u))
#elif TEST_LAYOUT == 3
#define FM1_FLASH_SIZE 0x400000u
#define FM1_FLASH_WRITE_REGIONS(X) X(0x100000u, 0x400000u)
#elif TEST_LAYOUT == 4
#define FM1_FLASH_WRITE_REGIONS(X)
#endif
#include "fm1_flash_policy.h"

static uint32_t cfg, core, c1, syncs;
#define FM1_FLASH_ICFG_READ() cfg
#define FM1_FLASH_ICFG_WRITE(v) (cfg = (v))
#define FM1_FLASH_SYNC() ((void)++syncs)
#define FM1_FLASH_CORE() core
#define FM1_FLASH_C1_CON c1
#include "fm1_flash_access.h"

static unsigned calls, fail_at;
static uint32_t seen_off[8], seen_n[8];
static uintptr_t seen_buf[8];
static int transact(uint32_t off, const void *p, uint32_t n)
{
    if (!fm1_flash_access_ok()) return FL_ERR_BUSY;
    if (calls < 8) {
        seen_off[calls] = off; seen_n[calls] = n; seen_buf[calls] = (uintptr_t)p;
    }
    calls++;
    return fail_at == calls ? FL_ERR_TIMEOUT : 0;
}
static int fl_prog_ram(uint32_t off, const uint8_t *p, uint32_t n, uint32_t *took)
{ (void)took; return transact(off, p, n); }
static int fl_read_ram(uint32_t off, uint8_t *p, uint32_t n)
{ return transact(off, p, n); }
static int fl_erase4k_ram(uint32_t off, uint32_t *took)
{ return transact(off, took, 4096); }
static uint32_t fl_jedec_ram(void)
{ return transact(0, 0, 0) ? 0 : 0x856014u; }
static uint32_t fl_status_ram(void)
{ return transact(0, 0, 0) ? UINT32_MAX : 0; }
#define FL_UID 1
static int fl_uid_ram(uint8_t *p) { return transact(0, p, 16); }
#define FL_FAR(fn) (fn)
#define FL_XIP(off) (0x02000000u + (off) - 0x4000u)
#include "fm1_flash_api.h"

static unsigned checks, fails;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL %d: %s\n", __LINE__, #c); fails++; } } while (0)
#define RAM(p) ((uint8_t *)(uintptr_t)(p))

static void reset(void)
{
    calls = fail_at = core = c1 = syncs = 0;
    cfg = 0x5B5u;
    fm1_flash_cpu1_idle = 0;
}

static void policy(void)
{
    CHECK(!FL_IN(0x90000u, 1u, 0xA0000u, 0x90000u));
    CHECK(!FL_READ_OK(UINT32_MAX, 2u));
    CHECK(FL_READ_OK(FM1_FLASH_SIZE, 0u));
    CHECK(!FL_READ_OK(FM1_FLASH_SIZE, 1u));
    CHECK(!FL_READ_OK(FM1_FLASH_SIZE - 1u, 2u));
    CHECK(FL_RAM_OK(RAM(0x1C00000u), 0x80000u));
    CHECK(FL_RAM_OK(RAM(0x1C80000u), 0u));
    CHECK(!FL_RAM_OK(RAM(0x1C7FFFFu), 2u));
    CHECK(!FL_RAM_OK(RAM(0x2000000u), 1u));
    CHECK(!FL_RAM_OK(RAM(UINTPTR_MAX), 2u));
    CHECK(!FL_WRITE_OK(FM1_FLASH_SIZE, 1u));
#if TEST_LAYOUT == 0
    CHECK(FL_WRITE_OK(0x97000u, 0x49000u));
    CHECK(FL_WRITE_OK(FL_OTA_LO, FL_OTA_HI - FL_OTA_LO));
    CHECK(FL_WRITE_OK(FL_AUTO_LO, FL_AUTO_HI - FL_AUTO_LO));
    CHECK(FL_WRITE_OK(0xFC000u, 0x3000u));
    CHECK(!FL_WRITE_OK(0xFFFFFu, 1u));
    CHECK(!FL_WRITE_OK(0x4000u, 4096u));
#elif TEST_LAYOUT == 1
    CHECK(FL_WRITE_OK(0x98000u, 4096u));
    CHECK(!FL_WRITE_OK(0x98FFFu, 2u));
    CHECK(!FL_WRITE_OK(0x98000u, 0x3000u));
    CHECK(!FL_WRITE_OK(FL_OTA_LO, 1u));
#elif TEST_LAYOUT == 2
    CHECK(FL_WRITE_OK(0x4000u, 0x93000u));
    CHECK(!FL_WRITE_OK(0x97000u, 4096u));
    CHECK(FL_WRITE_OK(0xE0000u, 0x1C000u));
    CHECK(!FL_WRITE_OK(0xFC000u, 1u));
#elif TEST_LAYOUT == 3
    CHECK(FL_WRITE_OK(0x100000u, 0x300000u));
    CHECK(!FL_WRITE_OK(0xFFFFFu, 1u));
    CHECK(!FL_PLAIN_OK(0x100000u, 1u)); /* capacity does not extend XIP */
#else
    CHECK(!FL_WRITE_OK(0x98000u, 1u));
    CHECK(!FL_WRITE_OK(FL_OTA_LO, 1u));
#endif
}

static void locking(void)
{
    const uint32_t states[] = {0u, 0x100u, 0x5B5u, 0x4B5u, UINT32_MAX};
    unsigned i;
    for (i = 0; i < sizeof states / sizeof states[0]; i++) {
        uint32_t outer, inner;
        reset(); cfg = states[i];
        outer = irq_save();
        CHECK(outer == states[i]);
        CHECK(cfg == (states[i] & ~0x100u));
        inner = irq_save();
        CHECK(inner == cfg);
        irq_restore(inner);
        CHECK(cfg == (states[i] & ~0x100u));
        irq_restore(outer);
        CHECK(cfg == states[i]);
    }
    reset();
    CHECK(!fm1_flash_access_ok()); /* enabled IRQs */
    cfg &= ~0x100u;
    CHECK(fm1_flash_access_ok()); /* CPU1 never started */
    c1 = 10u;
    CHECK(fm1_flash_access_ok()); /* CPU1 held */
    c1 = 8u;
    CHECK(!fm1_flash_access_ok()); /* unknown/busy CPU1 */
    fm1_flash_cpu1_idle = 1;
    CHECK(fm1_flash_access_ok()); /* registered RAM idle */
    core = 1;
    CHECK(!fm1_flash_access_ok()); /* CPU1 may not own SFC */
}

static void wrappers(void)
{
    uint32_t off, limit, state;
    int rc;
#if TEST_LAYOUT == 0
    off = 0x970F0u; limit = FL_DATA_HI;
#elif TEST_LAYOUT == 1
    off = 0x980F0u; limit = 0x99000u;
#elif TEST_LAYOUT == 2
    off = 0x4000u + 0xF0u; limit = 0x97000u;
#elif TEST_LAYOUT == 3
    off = 0x1000F0u; limit = FM1_FLASH_SIZE;
#else
    reset();
    CHECK(fl_write(0x98000u, RAM(0x1C10000u), 1u) == FL_ERR_RANGE);
    CHECK(calls == 0);
    return;
#endif
    reset(); state = cfg;
    CHECK(fl_write(off, RAM(0x1C10000u), 600) == 0);
    CHECK(cfg == state && calls == 4);
    CHECK(seen_n[0] == 16 && seen_n[1] == 256 && seen_n[2] == 256 && seen_n[3] == 72);
    CHECK(seen_off[1] == off + 16 && seen_buf[3] == 0x1C10210u);
    reset(); state = cfg;
    CHECK(fl_write(limit - 16u, RAM(0x1C10000u), 17u) == FL_ERR_RANGE);
    CHECK(calls == 0 && cfg == state); /* reject before any page changes */
    CHECK(fl_write(off, RAM(0x1C7FFFFu), 2u) == FL_ERR_BUFFER);
    CHECK(fl_write(off, RAM(0x2000000u), 1u) == FL_ERR_BUFFER);
    CHECK(calls == 0);
    CHECK(fl_write(off, RAM(0x1C10000u), 0u) == 0 && calls == 0);
    reset(); cfg &= ~0x100u; state = cfg; fail_at = 2;
    CHECK(fl_write(off, RAM(0x1C10000u), 600) == FL_ERR_TIMEOUT);
    CHECK(calls == 2 && cfg == state); /* disabled stays disabled on failure */
    reset(); c1 = 8;
    CHECK(fl_write(off, RAM(0x1C10000u), 1u) == FL_ERR_BUSY);
    CHECK(calls == 0 && cfg == 0x5B5u);
    fm1_flash_cpu1_idle = 1;
    CHECK(fl_write(off, RAM(0x1C10000u), 1u) == 0 && calls == 1);
    reset(); state = cfg;
    CHECK(fl_read(0x20u, RAM(0x1C10000u), 600) == 0);
    CHECK(calls == 3 && cfg == state && seen_n[2] == 88 && seen_off[2] == 0x220u);
    reset();
    CHECK(fl_read(FM1_FLASH_SIZE - 1u, RAM(0x1C10000u), 2u) == FL_ERR_RANGE);
    CHECK(fl_read(0, RAM(0x1C7FFFFu), 2u) == FL_ERR_BUFFER);
    CHECK(calls == 0);
    fail_at = 2;
    CHECK(fl_read(0, RAM(0x1C10000u), 600) == FL_ERR_TIMEOUT && calls == 2);
    CHECK(cfg == 0x5B5u);
    reset(); state = cfg;
    rc = fl_erase4k(off & ~0xFFFu, (uint32_t *)RAM(0x1C10000u));
    CHECK(rc == 0 && calls == 1 && cfg == state);
    c1 = 8;
    CHECK(fl_erase4k(off & ~0xFFFu, (uint32_t *)RAM(0x1C10000u)) == FL_ERR_BUSY);
    CHECK(calls == 1 && cfg == state);
    reset(); cfg &= ~0x100u; state = cfg;
    CHECK(fl_jedec() == 0x856014u && cfg == state);
    CHECK(fl_status() == 0 && cfg == state);
    CHECK(fl_uid(RAM(0x1C10000u)) == 0 && cfg == state);
    c1 = 8;
    CHECK(fl_jedec() == 0 && cfg == state);
    CHECK(fl_status() == UINT32_MAX && cfg == state);
    CHECK(fl_verify_xip(FM1_FLASH_PLAIN_HI - 1u, RAM(0x1C10000u), 2u) == FL_ERR_RANGE);
    CHECK(fl_verify_xip(FM1_FLASH_PLAIN_LO, RAM(0x2000000u), 1u) == FL_ERR_BUFFER);
}

int main(void)
{
    policy(); locking(); wrappers();
    printf("flash layout %d: %u checks, %u failures\n", TEST_LAYOUT, checks, fails);
    return fails != 0;
}
