/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 hdavid */
/* Where the update loader (loader/ldr_core.c) may erase and program, against a model of the FM-1 flash that records
 * every erase and program, and a fake host that serves a package built here (no .fwsc, no SDK). Built with the
 * defaults and with the slot switch (tests/run_ldr_bounds_test.sh), it runs the real ldr_core.c and ota.c through:
 * classic and fast installs, the same package again, power cut at many points of each and resumed, a flash that
 * fails, the slot switch up and down (and installed heads with hostile lengths), damaged packages and tables, hostile
 * UFW headers (flash.bin offsets that wrap, sizes out of range, a valid package whose flash.bin brings its own head
 * and 1 MiB of zeros over the staging, the globals and key_mac), and hosts that answer wrong addresses, lengths,
 * garbage, data that changes between passes, or nothing.
 *
 * The rule each erase / program is held to (stricter than loader.c's FL_RANGE_OK):
 *   never [0, 0x4000) (the boot area), never [0xE0000, 0xE4000) (the staged loader), never [0xFC000, 0x100000)
 *   (globals, key_mac), never the staging's last sector 0xE4000 (the record 0xE4F00) before the finish;
 *   before the finish: the app area [0x4000, FL_APP_END), and with LDR_SLOT_SWITCH erases in [FL_APP_END, 0xE0000);
 *   in the finish (after ldr_record_clear): erase 0xE4000; program zeros (<= 80 B) over an update record that was
 *   there at a 4 KiB boundary - 256 in (FL_APP_END, 0xFC000] outside Felucca's store; or erase that record's sector.
 * After every session that did not finish, and at every power cut: the boot area, the staged loader, the record and
 * the globals are as they were, or (a cut inside the finish) the record is gone and the app area is the package's. */
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FELUCCA_ID "ota-FM-1_900"
#define FELUCCA_OTA_DRYRUN 0
#define LDR_FAST_CHUNK 4096u
#define OTA_FRAME_MAX 4200u

#define NOR_SIZE 0x100000u
#define STG_LO 0xE0000u                                  /* ota.c OTA_AREA */
#define STG_REC_SEC 0xE4000u                             /* the staging's last sector: the record at 0xE4F00 */
#define STG_HI 0xE5000u
#define GLOB_LO 0xFC000u
#define CHIP_KEY 0x980Fu

enum { OP_ERASE, OP_PROG, OP_ERASE_BEGIN };

static uint8_t nor[NOR_SIZE], before[NOR_SIZE], snap[NOR_SIZE];
static int in_finish, violations, case_viol;
static long op_index, cut_at = -1, fail_at = -1;
static jmp_buf cut_jmp;
static uint32_t n_erase, n_prog;
static const char *case_name = "";

static void tb_op(int kind, uint32_t off, const uint8_t *p, uint32_t n);

/* ---- the host: a package (UFW header + flash.bin + the fast install's table), replies as ota.c wants them ---- */
enum { H_OK, H_SILENT_AFTER, H_WRONG_ADDR, H_WRONG_LEN, H_GARBAGE, H_FLIP_LATER, H_TBL_LEN, H_NO_SUCCESS };
static uint8_t h_hdr[0x400], h_fl[NOR_SIZE + 0x1000], h_tbl[256 * 6];
static uint32_t h_fl_off, h_fl_size, h_tbl_n, h_mode, h_arg, h_reads, h_flip_reads;
static uint8_t rx[4800];
static uint32_t rx_len, rx_full, now_ms;

static uint32_t rng = 1;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

static uint32_t pack7h(const uint8_t *in, uint32_t n, uint8_t *out)
{
    uint32_t acc = 0, nb = 0, o = 0;
    while (n--) {
        acc |= (uint32_t)*in++ << nb;
        nb += 8;
        while (nb >= 7) { out[o++] = acc & 0x7F; acc >>= 7; nb -= 7; }
    }
    if (nb) out[o++] = acc & 0x7F;
    return o;
}
static uint32_t unpack7h(const uint8_t *in, uint32_t n, uint8_t *out, uint32_t max)
{
    uint32_t acc = 0, nb = 0, o = 0;
    while (n--) {
        acc |= (uint32_t)(*in++ & 0x7Fu) << nb;
        nb += 7;
        if (nb >= 8) { if (o < max) out[o] = (uint8_t)acc; o++; acc >>= 8; nb -= 8; }
    }
    return o;
}

/* what the host has at a logical address (the fast install's table at 0xD0000000 + flash.bin's offset) */
static void host_data(uint32_t addr, uint8_t *d, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        uint32_t a = addr + i, t = a - (0xD0000000u + h_fl_off), f = a - h_fl_off;
        if (a >= 0xF0000000u)
            d[i] = i < 8u ? (uint8_t)"success"[i] : 0;
        else if (t < h_tbl_n * 6u)
            d[i] = h_tbl[t];
        else if (a < 0x400u)
            d[i] = h_hdr[a];
        else if (f < h_fl_size)
            d[i] = h_fl[f];
        else
            d[i] = 0;                                    /* past the package: zeros */
    }
}

static int ota_wire_send(const uint8_t *p, uint32_t n)
{
    static uint8_t u[64], m[4200];
    uint32_t addr, len, i, s = 0, rlen, raddr;
    if (n < 3 || unpack7h(p + 1, n - 2, u, sizeof u) < 8)
        return 0;
    if (u[2] != 0x30)
        return 0;                                        /* 0x11, 0x31, 0x32: nothing to answer */
    addr = u[7] | u[8] << 8 | u[9] << 16 | (uint32_t)u[10] << 24;
    len = u[11] | u[12] << 8;
    h_reads++;
    rlen = len;
    raddr = addr;
    if (h_mode == H_SILENT_AFTER && h_reads > h_arg)
        return 0;
    if (h_mode == H_NO_SUCCESS && addr >= 0xF0000000u)
        return 0;
    if (h_mode == H_WRONG_ADDR)
        raddr = addr + 0x1000u;
    if (h_mode == H_WRONG_LEN || (h_mode == H_TBL_LEN && addr >= 0xD0000000u && addr < 0xE0000000u))
        rlen = len ? len - 1u : 1u;
    memset(m, 0, sizeof m);
    m[1] = 0x59;
    m[2] = 0x30;
    m[3] = (uint8_t)(rlen + 8);
    m[4] = (uint8_t)((rlen + 8) >> 8);
    memcpy(m + 7, &raddr, 4);
    m[11] = (uint8_t)rlen;
    m[12] = (uint8_t)(rlen >> 8);
    host_data(addr, m + 14, rlen);
    if (h_mode == H_GARBAGE)
        for (i = 0; i < rlen; i++)
            m[14 + i] = (uint8_t)rnd();
    if (h_mode == H_FLIP_LATER && addr - h_fl_off - h_arg < 0x1000u && ++h_flip_reads > 8u)
        m[14] ^= 0x01;                                   /* the sector at h_arg: right in the check pass only */
    for (i = 6; i < 14 + rlen; i++)
        s += m[i];
    m[14 + rlen] = (uint8_t)~s;
    rx_len = pack7h(m, 15 + rlen, rx);
    rx_full = 1;
    return 0;
}
static int ota_frame_get(const uint8_t **p, uint32_t *n) { if (!rx_full) return 0; *p = rx; *n = rx_len; return 1; }
static void ota_frame_done(void) { rx_full = 0; }
static uint32_t ota_now_ms(void) { return now_ms; }
static void ota_idle(void) { now_ms++; }
static int ota_erase(uint32_t off) { (void)off; return -1; }     /* step 1 only: never by the loader */
static int ota_prog(uint32_t off, const void *p, uint32_t n) { (void)off; (void)p; (void)n; return -1; }
static int ota_fread(uint32_t off, void *p, uint32_t n) { memcpy(p, nor + off, n); return 0; }
static void ota_show(uint32_t step, int32_t code) { (void)step; (void)code; }
static void ota_commit(const uint8_t *parm) { (void)parm; }
#include "../chip/ota.c"

/* ---- the flash: every erase / program checked and counted; a cut (longjmp) or a failure at op cut_at / fail_at ---- */
static int flash_op(int kind, uint32_t off, const uint8_t *p, uint32_t n)
{
    uint32_t i;
    tb_op(kind, off, p, n);
    if (op_index == fail_at) {
        op_index++;
        return -1;
    }
    if (op_index++ == cut_at) {                          /* power cut in the middle of it */
        if (kind == OP_PROG)
            for (i = 0; i < n / 2u && off + i < NOR_SIZE; i++)
                nor[off + i] &= p[i];
        else if (off + 0x800u <= NOR_SIZE)
            memset(nor + off, 0xFF, 0x800);
        longjmp(cut_jmp, 1);
    }
    if (off >= NOR_SIZE || n > NOR_SIZE - off)
        return -1;                                       /* (counted as a violation by tb_op) */
    if (kind == OP_PROG) {
        for (i = 0; i < n; i++)
            nor[off + i] &= p[i];                        /* NOR: bits only go to 0 */
        n_prog++;
    } else {
        memset(nor + off, 0xFF, 0x1000);
        n_erase++;
    }
    return 0;
}
static int ldr_fread(uint32_t off, void *p, uint32_t n)
{
    if (off >= NOR_SIZE || n > NOR_SIZE - off)
        return -1;
    memcpy(p, nor + off, n);
    return 0;
}
static int ldr_erase(uint32_t off) { return flash_op(OP_ERASE, off, 0, 0x1000); }
static int ldr_prog(uint32_t off, const void *p, uint32_t n) { return flash_op(OP_PROG, off, p, n); }
static void ldr_record_clear(void)                       /* the finish starts here */
{
    in_finish = 1;
    memcpy(snap, nor, sizeof nor);
}
static int ldr_erase_begin(uint32_t off) { return flash_op(OP_ERASE_BEGIN, off, 0, 0x1000); }
static int ldr_erase_end(void) { return 0; }
static void ldr_progress(uint32_t done, uint32_t total) { (void)done; (void)total; }
#include "../loader/ldr_core.c"

#if defined(LDR_SLOT_SWITCH) && LDR_SLOT_SWITCH
#define SWITCH 1
#else
#define SWITCH 0
#endif
#define APP_HI LDR_APP_HI

static void viol(const char *why, int kind, uint32_t off, uint32_t n)
{
    violations++;
    case_viol++;
    if (case_viol <= 5)
        printf("  VIOLATION [%s] %s: %s %#x +%#x (%s)\n", case_name, why,
               kind == OP_PROG ? "program" : kind == OP_ERASE ? "erase" : "erase while receiving", off, n,
               in_finish ? "finish" : "install");
}

static int in_store_window(uint32_t sec) { return sec >= LDR_DATA_LO && sec < LDR_DATA_HI; }

static void tb_op(int kind, uint32_t off, const uint8_t *p, uint32_t n)
{
    uint32_t end = off + n, i, sec = off & ~0xFFFu;
    if (n == 0 || end < off || end > NOR_SIZE) { viol("out of the part", kind, off, n); return; }
    if (kind != OP_PROG && (off & 0xFFFu)) { viol("erase not on a sector", kind, off, n); return; }
    if (kind == OP_PROG && ((off & 0xFFu) + n) > 256u) { viol("program across a page", kind, off, n); return; }
    if (off < 0x4000u) { viol("BOOT AREA", kind, off, n); return; }
    if (end > STG_LO && off < STG_REC_SEC) { viol("STAGED LOADER", kind, off, n); return; }
    if (end > GLOB_LO) { viol("GLOBALS / KEY_MAC", kind, off, n); return; }
    if (!in_finish) {
        if (end > STG_REC_SEC && off < STG_HI) { viol("UPDATE RECORD before the finish", kind, off, n); return; }
        if (off >= 0x4000u && end <= APP_HI)
            return;
        if (SWITCH && kind == OP_ERASE && off >= APP_HI && end <= LDR_DATA_HI)
            return;                                      /* the slot switch's clear */
        viol("outside the app area", kind, off, n);
        return;
    }
    if (kind == OP_ERASE_BEGIN || (off < APP_HI && end > 0x4000u)) { viol("app write in the finish", kind, off, n); return; }
    if (kind == OP_ERASE && off == STG_REC_SEC)
        return;
    if (sec + 0x1000u <= APP_HI || sec + 0x1000u > GLOB_LO || in_store_window(sec) ||
        !ldr_is_record(snap + sec + 0xF00u)) { viol("finish write where no record was", kind, off, n); return; }
    if (kind == OP_ERASE)
        return;                                          /* a record that would not go in place */
    if (off != sec + 0xF00u || n > 80u) { viol("record kill not on the record", kind, off, n); return; }
    for (i = 0; i < n; i++)
        if (p[i]) { viol("record kill programs nonzero", kind, off, n); return; }
}

/* ---- the device and the package ---- */
static void put_record(uint8_t *at)                      /* an update record (ota.c ota_session's layout) */
{
    uint32_t i;
    for (i = 0; i < 112u; i++) at[i] = 0;
    ota_wr16(at + 2, 0x5A0Du);
    ota_wr16(at + 4, 0x5A01u);
    ota_wr16(at + 6, 0x5441u);
    memcpy(at + 8, "ota-FM-1_015", 12);
    ota_wr32(at + 72, STG_LO);
    ota_wr16(at, ota_crc16(at + 2, 78, 0));
}

static void app_head(uint8_t *at, uint32_t len, uint32_t key)   /* app_area_head, SFC with key */
{
    uint8_t h[32];
    uint32_t i;
    for (i = 0; i < 32u; i++) h[i] = (uint8_t)(0x30 + i);
    ota_wr32(h + 8, len);
    ota_wr16(h, ota_crc16(h + 2, 30, 0));
    ldr_sfc(h, 32, 0, key);
    memcpy(at, h, 32);
}

static void fill(uint32_t lo, uint32_t hi, uint32_t seed)
{
    rng = seed;
    for (; lo < hi; lo++) nor[lo] = (uint8_t)rnd();
}

#define REC_XDATA0 0x95F00u                              /* only below LDR_DATA_LO (the 0x93000 slot) */
#define REC_STORE 0xD5F00u                               /* in Felucca's store: user data, kept */
#define REC_STOCK 0xE8F00u                               /* the stock firmware's */
#define REC_GLOB 0xFDF00u                                /* in the globals: never looked at */

/* the device: a head with the chip key, an app whose head says old_len, data above, the staged loader and its
 * record, the stock record, globals and key_mac */
static void dev_init(uint32_t old_len, uint32_t old_end)
{
    uint8_t e[32], *blob = nor + 0x1000;
    uint32_t i;
    fill(0, 0x4000, 10);
    memset(e, 0, 32);
    ota_wr32(e + 4, 0x1000);
    ota_wr32(e + 8, 34);
    memcpy(e + 16, "isd_config.ini", 15);
    ota_wr16(e, ota_crc16(e + 2, 30, 0));
    ota_jl_enc(e, 32);
    memcpy(nor + 32, e, 32);
    for (i = 0; i < 16u; i++) blob[i] = 0x08;           /* sum 0x80: a key bit is (b[16+i] ^ b[15-i]) < 0x80 */
    for (i = 0; i < 16u; i++) blob[16 + i] = blob[15 - i] ^ ((CHIP_KEY >> i) & 1u ? 0x00 : 0xFF);
    ota_wr16(blob + 32, ota_crc16(blob, 32, 0));
    fill(0x4000, old_end, 11);                           /* the installed app */
    app_head(nor + 0x4000, old_len, CHIP_KEY);
    fill(old_end, STG_LO, 12);                           /* data */
    if (APP_HI < 0x97000u) put_record(nor + REC_XDATA0);
    put_record(nor + REC_STORE);
    fill(STG_LO, STG_HI, 13);                            /* the staged loader ... */
    memset(nor + 0xE4E00, 0xFF, 0x200);
    put_record(nor + 0xE4F00);                           /* ... and its record */
    fill(STG_HI, GLOB_LO, 14);
    put_record(nor + REC_STOCK);
    fill(GLOB_LO, NOR_SIZE, 15);                         /* globals, key_mac */
    put_record(nor + REC_GLOB);
    memcpy(before, nor, sizeof nor);
}

typedef struct {
    uint32_t fl_off, fl_size, new_len, key;
    int head_zero, above_zero, flip_fl, flip_t16, flip_t32, bad_ufw, nent;
} pkg_t;
#define FLIP_AT (0x7000u + 5u)                           /* in sector 7: one the package changes (7 % 3 == 1) */

static pkg_t pkg_default(void)
{
    pkg_t p;
    memset(&p, 0, sizeof p);
    p.fl_off = 0x400;
    p.fl_size = APP_HI;
    p.new_len = APP_HI - 0x4000u - 0x100u;              /* its end: this slot's */
    p.key = CHIP_KEY;
    p.nent = 2;
    return p;
}

static void pkg_build(const pkg_t *p)
{
    uint8_t ent[2][0x50];
    uint32_t i, k, fl_crc, sz = p->fl_size < NOR_SIZE ? p->fl_size : NOR_SIZE;
    memset(h_fl, 0, sizeof h_fl);
    memcpy(h_fl, before, sz < 0x4000u ? sz : 0x4000u);  /* flash.bin's head: the device's ... */
    if (p->head_zero) memset(h_fl, 0, 0x4000);           /* ... or a hostile one */
    rng = 20;
    for (k = 0x4000; k < APP_HI && k < sz; k += 0x1000u)  /* every third sector new, the others as installed */
        for (i = 0; i < 0x1000u; i++)
            h_fl[k + i] = ((k >> 12) % 3u == 1u) ? (uint8_t)rnd() : before[k + i];
    if (sz > 0x4000u) app_head(h_fl + 0x4000, p->new_len, p->key);
    if (sz > APP_HI) memset(h_fl + APP_HI, p->above_zero ? 0x00 : 0x5A, sz - APP_HI);
    h_fl_off = p->fl_off;
    h_fl_size = sz;
    h_tbl_n = sz <= NOR_SIZE ? sz >> 12 : 0;
    fl_crc = ota_crc16(h_fl, sz, 0);
    if (p->flip_fl == 2) h_fl[FLIP_AT] ^= 0x10;          /* damaged before the host made its table */
    for (k = 0; k < h_tbl_n; k++) {
        ota_wr16(h_tbl + k * 6u, ota_crc16(h_fl + k * 0x1000u, 0x1000u, 0));
        ota_wr32(h_tbl + k * 6u + 2, ldr_crc32(h_fl + k * 0x1000u, 0x1000u));
    }
    memset(h_hdr, 0, sizeof h_hdr);
    memset(ent, 0, sizeof ent);
    ota_wr16(ent[0] + 4, fl_crc);                        /* flash.bin, its CRC16 */
    ota_wr32(ent[0] + 8, p->fl_off);
    ota_wr32(ent[0] + 12, p->fl_size);
    memcpy(ent[0] + 0x40, "flash.bin", 9);
    ota_wr16(ent[1], 100);                               /* ota.bin (the loader takes no part of it) */
    ota_wr32(ent[1] + 8, 0x200);
    ota_wr32(ent[1] + 12, 0x200);
    memcpy(ent[1] + 0x40, "ota.bin", 7);
    for (i = 0; i < 2u; i++) {
        ota_jl_enc(ent[i], 0x50);
        memcpy(h_hdr + 0x40 + i * 0x50u, ent[i], 0x50);
    }
    ota_wr32(h_hdr + 4, 0x400 + sz);
    ota_wr16(h_hdr + 8, (uint32_t)p->nent);
    ota_wr16(h_hdr + 2, ota_crc16(h_hdr + 0x40, 2 * 0x50u, 0));
    ota_wr16(h_hdr, ota_crc16(h_hdr + 2, 0x3E, 0) ^ (p->bad_ufw ? 1u : 0u));
    ota_jl_enc(h_hdr, 0x40);
    if (p->flip_fl == 1) h_fl[FLIP_AT] ^= 0x10;          /* damaged on the way: after the package's and table's CRCs */
    if (p->flip_t16) h_tbl[9 * 6] ^= 0x01;
    if (p->flip_t32) h_tbl[10 * 6 + 3] ^= 0x01;          /* the CRC16s still combine; sector 10's CRC-32 is wrong */
}

/* ---- running and checking ---- */
static int run(int fast)
{
    rx_full = 0;
    in_finish = 0;
    ldr_fast_on = (uint8_t)fast;
    return ldr_session();
}

static int same(uint32_t lo, uint32_t hi) { return !memcmp(nor + lo, before + lo, hi - lo); }
static int app_is_pkg(void) { return !memcmp(nor + 0x4000, h_fl + 0x4000, APP_HI - 0x4000u); }
static int rec_valid(uint32_t at) { return ldr_is_record(nor + at); }
static int sector_erased(uint32_t sec) { return nor[sec] == 0xFF && !memcmp(nor + sec, nor + sec + 1, 0xFFF); }

static int fails, lenient;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("  FAIL [%s] %s:%d: ", case_name, __FILE__, __LINE__); \
                                       printf(__VA_ARGS__); printf("\n"); } } while (0)

/* the recovery path, after a session that did not finish or at a power cut */
static void check_recovery(void)
{
    CHECK(same(0, 0x4000), "boot area changed");
    CHECK(same(STG_LO, STG_REC_SEC), "staged loader changed");
    CHECK(same(GLOB_LO, NOR_SIZE), "globals / key_mac changed");
    if (!in_finish)
        CHECK(same(STG_REC_SEC, STG_HI), "the record's sector changed before the finish");
    else
        CHECK(rec_valid(0xE4F00) || app_is_pkg(), "record gone but the app not whole");
}

/* after a whole install: the app is the package's, [clo, chi) erased, the records gone where they were killed, the
 * rest as it was */
static void check_final(uint32_t clo, uint32_t chi)
{
    uint32_t i, a;
    static const uint32_t killed[] = {REC_XDATA0, REC_STOCK};
    CHECK(app_is_pkg(), "app area is not the package's");
    CHECK(same(0, 0x4000), "boot area changed");
    CHECK(same(STG_LO, STG_REC_SEC), "staged loader changed");
    CHECK(same(GLOB_LO, NOR_SIZE), "globals / key_mac changed");
    CHECK(!rec_valid(0xE4F00) && sector_erased(STG_REC_SEC), "the loader's record sector not erased");
    CHECK(rec_valid(REC_GLOB), "a record lookalike in the globals was touched");
    for (i = clo; i < chi; i++)
        if (nor[i] != 0xFF) { CHECK(0, "slot clear [%#x, %#x) not erased at %#x", clo, chi, i); break; }
    for (a = APP_HI; a < GLOB_LO; a++) {                 /* above the app: only the clear and the killed records */
        int kill = 0;
        if ((a >= clo && a < chi) || (a >= STG_LO && a < STG_HI)) continue;
        for (i = 0; i < 2u; i++)                         /* in place (zeros), or (its program failed) its sector */
            kill |= (a >= killed[i] && a < killed[i] + 80u && nor[a] == 0) ||
                    ((a & ~0xFFFu) == (killed[i] & ~0xFFFu) && sector_erased(a & ~0xFFFu));
        if (nor[a] != before[a] && !kill) { CHECK(0, "data changed at %#x", a); break; }
    }
    if (APP_HI < 0x97000u && !(REC_XDATA0 >= clo && REC_XDATA0 < chi) && !lenient)
        CHECK(!rec_valid(REC_XDATA0), "XDATA0 record not killed");
    if (!(REC_STORE >= clo && REC_STORE < chi))
        CHECK(rec_valid(REC_STORE), "a record lookalike in Felucca's store was touched");
    CHECK(!rec_valid(REC_STOCK), "the stock record is still there");
}

static void begin(const char *name)
{
    case_name = name;
    case_viol = 0;
    op_index = 0;
    cut_at = fail_at = -1;
    n_erase = n_prog = 0;
    h_mode = H_OK;
    h_reads = h_flip_reads = 0;
}

static void end(int rc, int want_rc)
{
    printf("  %-58s rc %4d  erases %4u  programs %5u  %s\n", case_name, rc, n_erase, n_prog,
           case_viol ? "VIOLATION" : "in bounds");
    if (want_rc != 999)
        CHECK(rc == want_rc, "rc %d, wanted %d", rc, want_rc);
}

/* a whole install of p on a device whose app head says old_len */
static void install(const char *name, const pkg_t *p, uint32_t old_len, uint32_t old_end, int fast, uint32_t clo,
                    uint32_t chi)
{
    int rc;
    begin(name);
    dev_init(old_len, old_end);
    pkg_build(p);
    rc = run(fast);
    end(rc, 0);
    check_final(clo, chi);
}

/* nothing written: refused before any erase (want_rc), the device as it was */
static void refused(const char *name, const pkg_t *p, int fast, uint32_t mode, int want_rc)
{
    int rc;
    begin(name);
    dev_init(APP_HI - 0x4000u - 0x100u, APP_HI);
    pkg_build(p);
    h_mode = mode;
    rc = run(fast);
    end(rc, want_rc);
    CHECK(rc != 0, "accepted");
    CHECK(n_erase == 0 && n_prog == 0, "%u erases, %u programs before the refusal", n_erase, n_prog);
    CHECK(!memcmp(nor, before, sizeof nor), "the flash changed");
}

/* power cut at op k (or the flash failing at op k), then power-on: the SPL runs the loader again while the record
 * is there; each cut point checked, the resumed install finished and checked */
static void cut_sweep(const char *name, const pkg_t *p, uint32_t old_len, uint32_t old_end, int fast, uint32_t clo,
                      uint32_t chi, int failing)
{
    long total, k, step, points = 0, resumed = 0, left = 0;
    int rc, tries, fails0 = fails, v0 = violations;
    begin(name);
    dev_init(old_len, old_end);
    pkg_build(p);
    rc = run(fast);
    total = op_index;
    CHECK(rc == 0, "the uncut install failed: %d", rc);
    step = total / 60 + 1;
    for (k = 0; k <= total; k += (k < 24 || k > total - 40) ? 1 : step) {
        begin(name);
        dev_init(old_len, old_end);
        pkg_build(p);
        if (failing)
            fail_at = k;
        else
            cut_at = k;
        if (!setjmp(cut_jmp)) {
            rc = run(fast);
            if (rc == 0 && !rec_valid(0xE4F00)) { check_final(clo, chi); points++; continue; }
            if (rc == 0)
                left++;                                  /* the finish's erase of 0xE4000 failed: the record stays
                                                          * (the new app's ota_boot_cleanup erases it) */
        }
        check_recovery();                                /* cut, or the session failed */
        cut_at = fail_at = -1;
        resumed += rec_valid(0xE4F00);
        lenient = !rec_valid(0xE4F00);                   /* a cut in the finish after the loader's record went:
                                                          * a foreign record below it may still be there */
        for (tries = 0; tries < 2 && rec_valid(0xE4F00); tries++)
            rc = run(fast);                              /* power-on: the record brings the loader back */
        CHECK(!rec_valid(0xE4F00), "after resume the record is still there (op %ld)", k);
        check_final(clo, chi);
        lenient = 0;
        points++;
    }
    printf("  %-58s %ld ops, %ld %s points (%ld resumed%s)  %s\n", name, total, points,
           failing ? "failure" : "cut", resumed, left ? ", record left by a failed finish erase" : "",
           fails == fails0 && violations == v0 ? "in bounds" : "FAILED");
}

/* the fast install: a sector that comes in unlike the table, three times: erased (while it came in), never written;
 * the sectors before it the package's, the recovery path whole; a good package then finishes the install */
static void fetch_fails(const char *name, const pkg_t *bad, uint32_t sec)
{
    pkg_t good = pkg_default();
    int rc;
    begin(name);
    dev_init(APP_HI - 0x4000u - 0x100u, APP_HI);
    pkg_build(bad);
    rc = run(1);
    end(rc, -13);
    check_recovery();
    CHECK(!memcmp(nor + 0x4000, h_fl + 0x4000, sec - 0x4000u), "a sector below %#x not the package's", sec);
    CHECK(sector_erased(sec), "sector %#x written", sec);
    pkg_build(&good);
    rc = run(1);
    CHECK(rc == 0, "resume failed: %d", rc);
    check_final(0, 0);
}

/* a hostile header the loader may take or refuse: in bounds either way */
static void either(const char *name, const pkg_t *p, int fast)
{
    int rc;
    begin(name);
    dev_init(APP_HI - 0x4000u - 0x100u, APP_HI);
    pkg_build(p);
    rc = run(fast);
    end(rc, 999);
    if (rc == 0)
        check_final(0, 0);
    else
        check_recovery();
}

/* the host goes wrong partway: the session fails, the recovery path is whole, a good host finishes it */
static void bad_host_then_resume(const char *name, const pkg_t *p, int fast, uint32_t mode, uint32_t arg)
{
    int rc;
    begin(name);
    dev_init(APP_HI - 0x4000u - 0x100u, APP_HI);
    pkg_build(p);
    h_mode = mode;
    h_arg = arg;
    rc = run(fast);
    end(rc, 999);
    CHECK(rc != 0, "accepted");
    check_recovery();
    if (mode == H_FLIP_LATER) {                          /* written sectors are the package's, never the flipped */
        CHECK(!memcmp(nor + 0x4000, h_fl + 0x4000, arg - 0x4000u), "a sector below %#x not the package's", arg);
        CHECK(!memcmp(nor + arg, before + arg, 0x1000), "the sector that changed was written");
    }
    h_mode = H_OK;
    rc = run(fast);
    CHECK(rc == 0, "resume failed: %d", rc);
    check_final(0, 0);
}

int main(void)
{
    pkg_t p = pkg_default(), q;
    const uint32_t same_len = APP_HI - 0x4000u - 0x100u;
    int fast;
    printf("ldr_bounds_test: FL_APP_END %#x, LDR_DATA_LO %#x, slot switch %d", APP_HI, LDR_DATA_LO, SWITCH);
#ifdef LDR_CLEAR_END
    printf(", LDR_CLEAR_END %#x", (unsigned)LDR_CLEAR_END);
#endif
    printf("\n");
    for (fast = 0; fast < 2; fast++) {
        const char *m = fast ? "fast" : "classic";
        char nm[96];
        printf(" %s\n", m);
        snprintf(nm, sizeof nm, "%s: install", m);
        install(nm, &p, same_len, APP_HI, fast, 0, 0);
        snprintf(nm, sizeof nm, "%s: the same package again", m);
        begin(nm);
        {
            int rc = run(fast);                          /* on the flash the install left */
            end(rc, 0);
            CHECK(n_erase == 0 && n_prog == 0, "%u erases, %u programs", n_erase, n_prog);
            check_final(0, 0);
        }
        snprintf(nm, sizeof nm, "%s: power cut at op k, resumed", m);
        cut_sweep(nm, &p, same_len, APP_HI, fast, 0, 0, 0);
        snprintf(nm, sizeof nm, "%s: flash fails at op k, resumed", m);
        cut_sweep(nm, &p, same_len, APP_HI, fast, 0, 0, 1);

        /* damaged packages: nothing erased */
        q = pkg_default(); q.flip_fl = fast ? 2 : 1;     /* (fast: a table made from it; damage on the way below) */
        snprintf(nm, sizeof nm, "%s: flash.bin damaged (its CRC16 wrong)", m);
        refused(nm, &q, fast, H_OK, -12);
        q = pkg_default(); q.key = 0x1234;
        snprintf(nm, sizeof nm, "%s: another chip key", m);
        refused(nm, &q, fast, H_OK, -6);
        q = pkg_default(); q.bad_ufw = 1;
        snprintf(nm, sizeof nm, "%s: UFW header CRC wrong", m);
        refused(nm, &q, fast, H_OK, -2);
        q = pkg_default(); q.nent = 12;
        snprintf(nm, sizeof nm, "%s: UFW header: 12 entries", m);
        refused(nm, &q, fast, H_OK, -2);
        /* hostile UFW headers: flash.bin's offset / size */
        q = pkg_default(); q.fl_size = 0x4000;
        snprintf(nm, sizeof nm, "%s: flash.bin size 0x4000 (the boot area only)", m);
        refused(nm, &q, fast, H_OK, -3);
        q = pkg_default(); q.fl_size = APP_HI - 0x1000u;
        snprintf(nm, sizeof nm, "%s: flash.bin size below the app's end", m);
        refused(nm, &q, fast, H_OK, -3);
        q = pkg_default(); q.fl_size = 0x100001u;
        snprintf(nm, sizeof nm, "%s: flash.bin size 1 MiB + 1", m);
        refused(nm, &q, fast, H_OK, -3);
        q = pkg_default(); q.fl_size = 0xFFFFFFFFu;
        snprintf(nm, sizeof nm, "%s: flash.bin size 0xFFFFFFFF", m);
        refused(nm, &q, fast, H_OK, -3);
        q = pkg_default(); q.fl_off = 0;
        snprintf(nm, sizeof nm, "%s: flash.bin offset 0", m);
        refused(nm, &q, fast, H_OK, -3);
        q = pkg_default(); q.fl_off = 0xFFFFF000u;
        snprintf(nm, sizeof nm, "%s: flash.bin offset 0xFFFFF000 (host reads wrap)", m);
        either(nm, &q, fast);
        q = pkg_default(); q.fl_off = 0x30000400u;
        snprintf(nm, sizeof nm, "%s: flash.bin offset 0x30000400 (table at 0x400)", m);
        either(nm, &q, fast);
        q = pkg_default(); q.fl_off = 0xF0000000u - 0x4000u;
        snprintf(nm, sizeof nm, "%s: flash.bin offset: its app area at 0xF0000000", m);
        either(nm, &q, fast);
        q = pkg_default(); q.fl_size = NOR_SIZE; q.head_zero = 1; q.above_zero = 1;
        snprintf(nm, sizeof nm, "%s: valid 1 MiB flash.bin, zero head + zeros above", m);
        install(nm, &q, same_len, APP_HI, fast, 0, 0);
        q = pkg_default(); q.fl_size = APP_HI + 0x123u;
        snprintf(nm, sizeof nm, "%s: flash.bin not whole sectors", m);
        install(nm, &q, same_len, APP_HI, fast, 0, 0);
        /* hostile hosts */
        snprintf(nm, sizeof nm, "%s: host answers another address", m);
        refused(nm, &p, fast, H_WRONG_ADDR, -1);
        snprintf(nm, sizeof nm, "%s: host answers another length", m);
        refused(nm, &p, fast, H_WRONG_LEN, -1);
        snprintf(nm, sizeof nm, "%s: host answers garbage", m);
        refused(nm, &p, fast, H_GARBAGE, -2);
        snprintf(nm, sizeof nm, "%s: host silent after %u reads (checking)", m, fast ? 3u : 40u);
        bad_host_then_resume(nm, &p, fast, H_SILENT_AFTER, fast ? 3u : 40u);
        snprintf(nm, sizeof nm, "%s: host silent after %u reads (writing)", m, fast ? 20u : 1400u);
        bad_host_then_resume(nm, &p, fast, H_SILENT_AFTER, fast ? 20u : 1400u);
        snprintf(nm, sizeof nm, "%s: host never says success", m);
        begin(nm);
        dev_init(same_len, APP_HI);
        pkg_build(&p);
        h_mode = H_NO_SUCCESS;
        { int rc = run(fast); end(rc, 0); }
        check_final(0, 0);
    }
    printf(" classic only\n");
    bad_host_then_resume("classic: sector 0x10000 changes after the check pass", &p, 0, H_FLIP_LATER, 0x10000);
    printf(" fast only\n");
    q = pkg_default(); q.flip_t16 = 1;
    refused("fast: table CRC16 damaged", &q, 1, H_OK, -12);
    refused("fast: host answers the table short", &p, 1, H_TBL_LEN, -7);
    q = pkg_default(); q.flip_fl = 1;
    fetch_fails("fast: sector 7 damaged on the way (table right)", &q, 0x7000);
    q = pkg_default(); q.flip_t32 = 1;
    fetch_fails("fast: table CRC-32 wrong for sector 10", &q, 0xA000);
#if SWITCH
    printf(" slot switch\n");
    {
        uint32_t down_end = APP_HI + 0x8000u, up_end = APP_HI - 0x20000u;
#ifdef LDR_CLEAR_END
        const uint32_t up_lo = APP_HI, up_hi = LDR_CLEAR_END;
#else
        const uint32_t up_lo = 0, up_hi = 0;
#endif
        for (fast = 0; fast < 2; fast++) {
            char nm[96];
            const char *m = fast ? "fast" : "classic";
            snprintf(nm, sizeof nm, "%s: down (old app ends %#x)", m, down_end);
            install(nm, &p, down_end - 0x4000u - 0x80u, down_end, fast, APP_HI, down_end);
            snprintf(nm, sizeof nm, "%s: up (old app ends %#x)", m, up_end);
            install(nm, &p, up_end - 0x4000u - 0x80u, up_end, fast, up_lo, up_hi);
            snprintf(nm, sizeof nm, "%s: down, power cut, resumed", m);
            cut_sweep(nm, &p, down_end - 0x4000u - 0x80u, down_end, fast, APP_HI, down_end, 0);
            snprintf(nm, sizeof nm, "%s: up, power cut, resumed", m);
            cut_sweep(nm, &p, up_end - 0x4000u - 0x80u, up_end, fast, up_lo, up_hi, 0);
            /* installed heads with hostile lengths (read from the device's flash, not the package) */
            snprintf(nm, sizeof nm, "%s: installed head length 0xFFFFFFFF (wraps)", m);
            install(nm, &p, 0xFFFFFFFFu, APP_HI, fast, up_lo, up_hi);
            snprintf(nm, sizeof nm, "%s: installed head ends at 0xE0000", m);
            install(nm, &p, 0xE0000u - 0x4000u, APP_HI, fast, APP_HI, 0xE0000u);
            snprintf(nm, sizeof nm, "%s: installed head ends at 0xE1000 (staging)", m);
            install(nm, &p, 0xE1000u - 0x4000u, APP_HI, fast, 0, 0);
            snprintf(nm, sizeof nm, "%s: installed head ends at 0xFF000 (key_mac)", m);
            install(nm, &p, 0xFF000u - 0x4000u, APP_HI, fast, 0, 0);
            snprintf(nm, sizeof nm, "%s: installed head ends at 0x7FFFF000", m);
            install(nm, &p, 0x7FFFF000u - 0x4000u, APP_HI, fast, 0, 0);
        }
    }
#endif
    printf("%s: %d failures, %d out-of-bounds writes\n", fails || violations ? "LDR BOUNDS TEST FAILED"
                                                                           : "ldr bounds test passed",
           fails, violations);
    return fails || violations;
}
