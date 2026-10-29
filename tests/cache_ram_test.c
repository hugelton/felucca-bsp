/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Henri David */
/* chip/fm1_cache_ram.h on a model of the corex2 cache registers, the tag RAM and the data ways: 2, 4 and 7 ways
 * (the masks written, the tags invalid, CACHE_CON's bits 8-9 off while they change and back after, the memory zeroed,
 * its bytes); 0 and 8 ways (nothing touched); and the failures, each put back as the boot loader left it with 0 bytes:
 * a cache that never idles, masks that do not read back, a stuck bit in each way.
 *
 *   cc -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -Ichip tests/cache_ram_test.c -o cache_ram_test
 *   ./cache_ram_test */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* ---- the model: the three registers, the tag RAM's three ranges, the eight data ways ---- */
static uint32_t m_reg[3];                             /* CACHE_CON, DCACHE_WAY, ICACHE_WAY */
static uint32_t m_tag0[0x4000 / 4], m_tag1[0x2200 / 4], m_tag2[0x200 / 4];
static uint32_t m_way[8 * 4096 / 4];
static uint32_t m_stray;                              /* (any other address: counted) */
static unsigned m_strays;
static int m_drop_masks;                              /* the masks do not take */
static uint32_t m_stuck_at, m_stuck_bit;              /* a stuck-at-0 bit of the data ways (0: none) */
static uint32_t m_con_at_set;                         /* CACHE_CON as the masks were written */

static volatile uint32_t *m_map(uint32_t a)
{
    if (a >= 0x01EEE008u && a < 0x01EEE014u)
        return &m_reg[(a - 0x01EEE008u) / 4u];
    if (a >= 0x01F00000u && a < 0x01F04000u)
        return &m_tag0[(a - 0x01F00000u) / 4u];
    if (a >= 0x01F08000u && a < 0x01F0A200u)
        return &m_tag1[(a - 0x01F08000u) / 4u];
    if (a >= 0x01F0B000u && a < 0x01F0B200u)
        return &m_tag2[(a - 0x01F0B000u) / 4u];
    if (a >= 0x01F28000u && a < 0x01F30000u)
        return &m_way[(a - 0x01F28000u) / 4u];
    m_strays++;
    return &m_stray;
}
static uint32_t m_read(volatile uint32_t *p)
{
    uint32_t v = *p;
    if (m_stuck_at && p == m_map(m_stuck_at))
        v &= ~m_stuck_bit;
    return v;
}
static void m_set_done(void)
{
    m_con_at_set = m_reg[0];
    if (m_drop_masks) {                               /* (once: the masks put back after take) */
        m_drop_masks = 0;
        m_reg[1] = m_reg[2] = 0xFFFFFFFFu;
    }
}
#define FM1_CR_P(a) m_map((uint32_t)(a))
#define FM1_CR_RD(p) m_read(p)
#define FM1_CR_SET_DONE() m_set_done()
#define FM1_CR_RAMFN __attribute__((noinline))
#include "../chip/fm1_cache_ram.h"

static int fails;
static void check(int ok, const char *what)
{
    printf("cache ram: %-84s %s\n", what, ok ? "ok" : "FAIL");
    fails += !ok;
}

#define BOOT_CON 0x00004100u                          /* idle, the cache on (bit 9 off: no SDRAM) */
#define BOOT_DWAY 0x00FFFFFFu                         /* every way a cache (the boot loader's) */
#define BOOT_IWAY 0x0000FFFFu
static void boot(void)
{
    memset(&fm1_cr, 0, sizeof fm1_cr);
    m_reg[0] = BOOT_CON;
    m_reg[1] = BOOT_DWAY;
    m_reg[2] = BOOT_IWAY;
    memset(m_tag0, 0xA5, sizeof m_tag0);
    memset(m_tag1, 0xA5, sizeof m_tag1);
    memset(m_tag2, 0xA5, sizeof m_tag2);
    memset(m_way, 0x5A, sizeof m_way);
    m_drop_masks = 0;
    m_stuck_at = m_stuck_bit = 0;
    m_con_at_set = 0;
    m_strays = 0;
}
static int all(const uint32_t *p, size_t n, uint32_t v)
{
    size_t i;
    for (i = 0; i < n; i++)
        if (p[i] != v)
            return 0;
    return 1;
}
static int put_back(void)                             /* the boot loader's masks and CACHE_CON again */
{
    return m_reg[0] == BOOT_CON && m_reg[1] == BOOT_DWAY && m_reg[2] == BOOT_IWAY;
}

static void t_ways(uint32_t n)
{
    char what[96];
    uint32_t keep = (0xFFu << n) & 0xFFu;
    boot();
    fm1_cache_ram_init(n);
    snprintf(what, sizeof what, "%u ways: ready, %u bytes from 0x01F28000", (unsigned)n, (unsigned)(n * 4096u));
    check(fm1_cr.status == FM1_CR_OK && fm1_cache_ram_bytes() == n * 4096u && fm1_cr.bad_at == 0u, what);
    snprintf(what, sizeof what, "%u ways: DCACHE_WAY %06X in its three masks, ICACHE_WAY all kept", (unsigned)n,
             (unsigned)(keep | keep << 8 | keep << 16));
    check(m_reg[1] == (keep | keep << 8 | keep << 16) && m_reg[2] == 0xFFFFu && fm1_cr.dway == m_reg[1] &&
          fm1_cr.iway == m_reg[2], what);
    check(fm1_cr.con == BOOT_CON && fm1_cr.dway0 == BOOT_DWAY && fm1_cr.iway0 == BOOT_IWAY,
          "the registers as the boot loader left them kept");
    check((m_con_at_set & 0x300u) == 0u && m_reg[0] == BOOT_CON, "CACHE_CON's bits 8-9 off while the masks change, back after");
    check(all(m_tag0, sizeof m_tag0 / 4, 0) && all(m_tag1, sizeof m_tag1 / 4, 0) && all(m_tag2, sizeof m_tag2 / 4, 0),
          "every tag invalid");
    check(all(m_way, n * 1024u, 0), "the ways used as RAM zeroed after the test");
    check(all(m_way + n * 1024u, (8u - n) * 1024u, 0x5A5A5A5Au), "the ways kept as a cache untouched");
    check(m_strays == 0u, "no access outside the registers, the tags and the ways");
}

static void t_off(uint32_t n)
{
    char what[96];
    boot();
    fm1_cache_ram_init(n);
    snprintf(what, sizeof what, "%u ways: off, 0 bytes, nothing touched", (unsigned)n);
    check(fm1_cr.status == FM1_CR_OFF && fm1_cache_ram_bytes() == 0u && put_back() &&
          all(m_tag0, sizeof m_tag0 / 4, 0xA5A5A5A5u) && all(m_way, sizeof m_way / 4, 0x5A5A5A5Au) && m_strays == 0u,
          what);
}

static void t_busy(void)
{
    boot();
    m_reg[0] = BOOT_CON & ~0x4000u;                   /* (never idle) */
    fm1_cache_ram_init(2);
    check(fm1_cr.status == FM1_CR_BUSY && fm1_cache_ram_bytes() == 0u && m_reg[1] == BOOT_DWAY &&
          m_reg[2] == BOOT_IWAY && all(m_tag0, sizeof m_tag0 / 4, 0xA5A5A5A5u),
          "a cache that never idles: nothing changed, 0 bytes");
}

static void t_reg(void)
{
    boot();
    m_drop_masks = 1;
    fm1_cache_ram_init(2);
    check(fm1_cr.status == FM1_CR_REG && fm1_cache_ram_bytes() == 0u, "masks that do not read back: refused, 0 bytes");
    check(fm1_cr.dway == 0xFFFFFFFFu, ".. what was read back kept for the readout");
}

static void t_bad(uint32_t n, uint32_t way, uint32_t bit)
{
    char what[96];
    boot();
    m_stuck_at = 0x01F28000u + way * 4096u + 0x7F0u;
    m_stuck_bit = 1u << bit;
    fm1_cache_ram_init(n);
    snprintf(what, sizeof what, "%u ways, bit %u stuck in way %u: the test fails at it, put back, 0 bytes", (unsigned)n,
             (unsigned)bit, (unsigned)way);
    check(fm1_cr.status == FM1_CR_BAD && fm1_cr.bad_at == m_stuck_at && fm1_cache_ram_bytes() == 0u && put_back(), what);
}

int main(void)
{
    uint32_t n, w;
    t_ways(2);
    t_ways(4);
    t_ways(7);
    t_off(0);
    t_off(8);
    t_busy();
    t_reg();
    check(put_back(), ".. and the boot loader's masks back");
    for (n = 2; n <= 7u; n += n == 2u ? 2u : 3u)
        for (w = 0; w < n; w++)
            t_bad(n, w, (w * 7u + n) % 32u);
    t_bad(7, 6, 31);
    printf("cache ram: %d failed\n", fails);
    return fails != 0;
}
