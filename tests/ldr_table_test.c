/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 hdavid */
/* The fast install's table check (loader/ldr_core.c ldr_table_ok): the CRC16 advanced over 4 KiB of zeros in one step
 * (ldr_zeros4k) gives what ota_crc16 over the zeros gives, for every bit of the register (the CRC is linear: so for
 * every value) and for random ones; the whole check gives what the old one (4 KiB of zeros through ota_crc16 per
 * sector, kept here) gives, on random and zero tables, and takes a real flash.bin's table. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FELUCCA_ID "ota-FM-1_900"
#define FELUCCA_OTA_DRYRUN 0
#define FELUCCA_OTA_RAMONLY 1
#define LDR_FAST_CHUNK 4096u
#define OTA_FRAME_MAX 4200u

/* ---- the hooks: no host, no flash (the table check uses neither) ---- */
static int ota_wire_send(const uint8_t *p, uint32_t n) { (void)p; (void)n; return 0; }
static int ota_frame_get(const uint8_t **p, uint32_t *n) { (void)p; (void)n; return 0; }
static void ota_frame_done(void) {}
static uint32_t ota_now_ms(void) { return 0; }
static void ota_idle(void) {}
static int ota_erase(uint32_t off) { (void)off; return -1; }
static int ota_prog(uint32_t off, const void *p, uint32_t n) { (void)off; (void)p; (void)n; return -1; }
static int ota_fread(uint32_t off, void *p, uint32_t n) { (void)off; memset(p, 0xFF, n); return 0; }
static void ota_show(uint32_t step, int32_t code) { (void)step; (void)code; }
static void ota_commit(const uint8_t *parm) { (void)parm; }
#include "../chip/ota.c"

static int ldr_fread(uint32_t off, void *p, uint32_t n) { return ota_fread(off, p, n); }
static int ldr_erase(uint32_t off) { (void)off; return -1; }
static int ldr_prog(uint32_t off, const void *p, uint32_t n) { (void)off; (void)p; (void)n; return -1; }
static void ldr_record_clear(void) {}
static int ldr_erase_begin(uint32_t off) { (void)off; return -1; }
static int ldr_erase_end(void) { return -1; }
static void ldr_progress(uint32_t done, uint32_t total) { (void)done; (void)total; }
#include "../loader/ldr_core.c"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); \
                                       printf("\n"); } } while (0)

static uint8_t zero4k[0x1000];

static uint32_t old_zeros4k(uint32_t c) { return ota_crc16(zero4k, 0x1000u, c); }

static uint32_t old_table_crc(const uint8_t *t, uint32_t n)   /* the check as it was: the only want it takes */
{
    uint32_t k, i, c = 0;
    for (k = 0; k < n; k++) {
        for (i = 0; i < 0x1000u; i += 0x200u)
            c = ota_crc16(zero4k, 0x200u, c);
        c ^= ota_rd16(t + k * 6u);
    }
    return c;
}

static uint32_t rnd(void)
{
    static uint32_t s = 0x2545F491u;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}

int main(void)
{
    static uint8_t t[LDR_NMAX * 6u], img[40u * 0x1000u];
    static const uint32_t NS[] = {1, 2, 31, 32, 33, 143, 147, 256};
    uint32_t i, k, c, bad = 0, ns;

    /* one step over 4 KiB of zeros: every register bit (linear: so every value), and random values */
    for (i = 0; i < 16u; i++)
        bad += ldr_zeros4k(1u << i) != old_zeros4k(1u << i);
    CHECK(bad == 0, "%u of the 16 register bits advance differently", bad);
    bad = 0;
    for (i = 0; i < 4000u; i++) {
        c = i < 2u ? (i ? 0xFFFFu : 0u) : rnd() & 0xFFFFu;
        bad += ldr_zeros4k(c) != old_zeros4k(c);
    }
    CHECK(bad == 0, "%u of 4000 values advance differently", bad);

    /* the whole check as the old one: random and zero tables, the old one's want taken, another refused */
    for (i = 0; i < sizeof NS / sizeof NS[0]; i++) {
        uint32_t n = NS[i], z;
        for (z = 0; z < 2u; z++) {
            uint32_t want;
            for (k = 0; k < n * 6u; k++)
                t[k] = z ? 0 : (uint8_t)rnd();
            want = old_table_crc(t, n);
            CHECK(ldr_table_ok(t, n, want),
                  "%s table of %u: the old check's CRC %#x not taken", z ? "a zero" : "a random", n, want);
            CHECK(!ldr_table_ok(t, n, want ^ 0x0100u),
                  "%s table of %u: a wrong CRC taken", z ? "a zero" : "a random", n);
        }
    }

    /* a real flash.bin's table: the CRC16 of each sector, against the CRC16 of the whole */
    ns = sizeof img / 0x1000u;
    for (k = 0; k < sizeof img; k++)
        img[k] = (uint8_t)rnd();
    for (k = 0; k < ns; k++) {
        c = ota_crc16(img + k * 0x1000u, 0x1000u, 0);
        t[k * 6u] = (uint8_t)c;
        t[k * 6u + 1u] = (uint8_t)(c >> 8);
    }
    c = ota_crc16(img, sizeof img, 0);
    CHECK(ldr_table_ok(t, ns, c), "a real table of %u sectors not taken", ns);
    t[12] ^= 1;                                     /* (sector 2's CRC16) */
    CHECK(!ldr_table_ok(t, ns, c), "a table with a wrong entry taken");

    printf("%s: ldr_table_test\n", fails ? "FAIL" : "ok");
    return fails != 0;
}
