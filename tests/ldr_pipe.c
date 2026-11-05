/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Charles Vestal
 * The device and package model after tests/ldr_bounds_test.c (Henri David). */
/* The update loader (loader/ldr_core.c and chip/ota.c, the real code) on a pipe, for tests/install_fast_test.py: the
 * SysEx frames of the USB-MIDI port as raw bytes, F0 .. F7, on stdin (from the host) and stdout (to it). The flash is a
 * 1 MiB file mapped shared, so a killed process leaves it as a power cut would (between two flash operations), and the
 * next run is the loader the SPL starts again at the next power-on.
 *
 *   ldr_pipe init FLASH IMAGE   a device (head with its chip key, an installed app, data, the staged loader and its
 *                               record, the stock firmware's record) in FLASH, and a package for it whose logical
 *                               image (what the host serves: the UFW header, then flash.bin; every third app sector
 *                               new; the Felucca loader's mark where ota.bin would be) goes to IMAGE
 *   ldr_pipe rearm FLASH        the update record back (the loader runs again: the same package once more)
 *   ldr_pipe run FLASH          the loader: the handshake, the fast install's hello, sessions on the upgrade command
 *                               (F0 22 24 35 7F F7); exits 0 when a session finished (the FM-1 restarts), 3 at the
 *                               end of stdin. Each session's code goes to stderr ("session N"). A read's try waits
 *                               LDR_PIPE_TRY_MS (default 400 ms; ota.c's 5 s on the FM-1)
 *
 * Built with -DLDR_FAST_CHUNK=4096u -DOTA_FRAME_MAX=4200u as the loader (loader.c), or without them: a loader from
 * before the fast install. */
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#define FELUCCA_ID "ota-FM-1_900"
#define FELUCCA_OTA_DRYRUN 0
#define NOR_SIZE 0x100000u
#define STG_LO 0xE0000u
#define STG_HI 0xE5000u
#define GLOB_LO 0xFC000u
#define CHIP_KEY 0x980Fu
#define REC_STOCK 0xE8F00u

static uint8_t *nor;

/* ---- the port: frames in from stdin, out to stdout ---- */
static uint8_t in_buf[8192], frame[8192];
static uint32_t in_len, frame_full, frame_len, ota_req, eof;

static void port_poll(void)
{
    fd_set r;
    struct timeval tv = {0, 0};
    uint8_t b[4096];
    ssize_t n, i;
    FD_ZERO(&r);
    FD_SET(0, &r);
    if (eof || select(1, &r, 0, 0, &tv) <= 0)
        return;
    n = read(0, b, sizeof b);
    if (n <= 0) {
        eof = 1;
        return;
    }
    for (i = 0; i < n; i++) {
        if (b[i] == 0xF0)
            in_len = 0;
        if (in_len < sizeof in_buf)
            in_buf[in_len++] = b[i];
        if (b[i] == 0xF7 && in_buf[0] == 0xF0) {
            if (in_len == 6 && in_buf[1] == 0x22 && in_buf[2] == 0x24 && in_buf[3] == 0x35 && in_buf[4] == 0x7F)
                ota_req = 1;                             /* usb.c: the upgrade command, a flag */
            else if (!frame_full && in_len > 2) {        /* usb.c: one frame at a time here (the host waits) */
                memcpy(frame, in_buf + 1, in_len - 2);
                frame_len = in_len - 2;
                frame_full = 1;
            }
            in_len = 0;
        }
    }
}

static int ota_wire_send(const uint8_t *p, uint32_t n)
{
    if (fwrite(p, 1, n, stdout) != n)
        return -1;
    fflush(stdout);
    return 0;
}
static int ota_frame_get(const uint8_t **p, uint32_t *n)
{
    port_poll();
    if (!frame_full)
        return 0;
    *p = frame;
    *n = frame_len;
    return 1;
}
static void ota_frame_done(void) { frame_full = 0; }
static uint32_t ota_now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint32_t)(t.tv_sec * 1000 + t.tv_nsec / 1000000);
}
static void ota_idle(void)
{
    if (eof)
        exit(3);                                         /* the host went: as the FM-1 unplugged for good */
    port_poll();
    usleep(100);
}
static int ota_erase(uint32_t off) { (void)off; return -1; }
static int ota_prog(uint32_t off, const void *p, uint32_t n) { (void)off; (void)p; (void)n; return -1; }
static int ota_fread(uint32_t off, void *p, uint32_t n) { memcpy(p, nor + off, n); return 0; }
static void ota_show(uint32_t step, int32_t code) { (void)step; (void)code; }
static void ota_commit(const uint8_t *parm) { (void)parm; }
#include "../chip/ota.c"

/* ---- the flash: loader.c's FL_RANGE_OK ---- */
static int in_range(uint32_t off, uint32_t n)
{
    return (off >= 0x4000u && n <= 0x97000u - off && off <= 0x97000u) ||
           (off >= 0xE0000u && off <= 0xFC000u && n <= 0xFC000u - off);
}
static int ldr_fread(uint32_t off, void *p, uint32_t n)
{
    if (off >= NOR_SIZE || n > NOR_SIZE - off)
        return -1;
    memcpy(p, nor + off, n);
    return 0;
}
static int ldr_erase(uint32_t off)
{
    if ((off & 0xFFFu) || !in_range(off, 0x1000u))
        return -1;
    memset(nor + off, 0xFF, 0x1000);
    return 0;
}
static int ldr_prog(uint32_t off, const void *p, uint32_t n)
{
    uint32_t i;
    if (!in_range(off, n) || ((off & 0xFFu) + n) > 256u)
        return -1;
    for (i = 0; i < n; i++)
        nor[off + i] &= ((const uint8_t *)p)[i];
    return 0;
}
static void ldr_record_clear(void) {}
static int ldr_erase_begin(uint32_t off) { return ldr_erase(off); }
static int ldr_erase_end(void) { return 0; }
static void ldr_progress(uint32_t done, uint32_t total) { (void)done; (void)total; port_poll(); }
#include "../loader/ldr_core.c"

/* ---- the device and the package (as tests/ldr_bounds_test.c) ---- */
static uint32_t rng = 1;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
static void fill(uint32_t lo, uint32_t hi, uint32_t seed)
{
    rng = seed;
    for (; lo < hi; lo++) nor[lo] = (uint8_t)rnd();
}
static void put_record(uint8_t *at)
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
static void app_head(uint8_t *at, uint32_t len)
{
    uint8_t h[32];
    uint32_t i;
    for (i = 0; i < 32u; i++) h[i] = (uint8_t)(0x30 + i);
    ota_wr32(h + 8, len);
    ota_wr16(h, ota_crc16(h + 2, 30, 0));
    ldr_sfc(h, 32, 0, CHIP_KEY);
    memcpy(at, h, 32);
}
static void dev_init(void)
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
    for (i = 0; i < 16u; i++) blob[i] = 0x08;
    for (i = 0; i < 16u; i++) blob[16 + i] = blob[15 - i] ^ ((CHIP_KEY >> i) & 1u ? 0x00 : 0xFF);
    ota_wr16(blob + 32, ota_crc16(blob, 32, 0));
    fill(0x4000, LDR_APP_HI, 11);
    app_head(nor + 0x4000, LDR_APP_HI - 0x4000u - 0x100u);
    fill(LDR_APP_HI, STG_LO, 12);
    fill(STG_LO, STG_HI, 13);
    memset(nor + 0xE4E00, 0xFF, 0x200);
    put_record(nor + 0xE4F00);
    fill(STG_HI, GLOB_LO, 14);
    put_record(nor + REC_STOCK);
    fill(GLOB_LO, NOR_SIZE, 15);
}
static int pkg_write(const char *path)
{
    static uint8_t img[0x400 + LDR_APP_HI];
    uint8_t ent[2][0x50], *hdr = img, *fl = img + 0x400;
    uint32_t i, k, sz = LDR_APP_HI;
    FILE *f;
    memcpy(fl, nor, 0x4000);
    rng = 20;
    for (k = 0x4000; k < sz; k += 0x1000u)              /* every third sector new, the others as installed */
        for (i = 0; i < 0x1000u; i++)
            fl[k + i] = ((k >> 12) % 3u == 1u) ? (uint8_t)rnd() : nor[k + i];
    app_head(fl + 0x4000, sz - 0x4000u - 0x100u);
    memset(ent, 0, sizeof ent);
    ota_wr16(ent[0] + 4, ota_crc16(fl, sz, 0));
    ota_wr32(ent[0] + 8, 0x400);
    ota_wr32(ent[0] + 12, sz);
    memcpy(ent[0] + 0x40, "flash.bin", 9);
    ota_wr16(ent[1], 100);
    ota_wr32(ent[1] + 8, 0x200);
    ota_wr32(ent[1] + 12, 0x200);
    memcpy(ent[1] + 0x40, "ota.bin", 7);
    for (i = 0; i < 2u; i++) {
        ota_jl_enc(ent[i], 0x50);
        memcpy(hdr + 0x40 + i * 0x50u, ent[i], 0x50);
    }
    ota_wr32(hdr + 4, 0x400 + sz);
    ota_wr16(hdr + 8, 2);
    ota_wr16(hdr + 2, ota_crc16(hdr + 0x40, 2 * 0x50u, 0));
    ota_wr16(hdr, ota_crc16(hdr + 2, 0x3E, 0));
    ota_jl_enc(hdr, 0x40);
    memcpy(hdr + 0x200, "FELUCCA-LOADER-1", 16);      /* where ota.bin's entry says the loader is: this family's mark */
    if (!(f = fopen(path, "wb")) || fwrite(img, 1, sizeof img, f) != sizeof img || fclose(f))
        return 1;
    return 0;
}

int main(int argc, char **argv)
{
    int fd;
    if (argc < 3)
        return 2;
    fd = open(argv[2], O_RDWR | O_CREAT, 0644);
    if (fd < 0 || ftruncate(fd, NOR_SIZE))
        return 2;
    nor = mmap(0, NOR_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (nor == MAP_FAILED)
        return 2;
    if (!strcmp(argv[1], "init")) {
        dev_init();
        return argc > 3 ? pkg_write(argv[3]) : 2;
    }
    if (!strcmp(argv[1], "rearm")) {
        memset(nor + 0xE4000, 0xFF, 0x1000);
        put_record(nor + 0xE4F00);
        return 0;
    }
    ota_try_ms = getenv("LDR_PIPE_TRY_MS") ? (uint32_t)atoi(getenv("LDR_PIPE_TRY_MS")) : 400u;   /* a read's try (5 s
                                                          * on the FM-1): the tests' hosts come back quickly */
    for (;;) {
        port_poll();
        if (eof)
            return 3;
#ifdef LDR_FAST_CHUNK
        if (ota_service() == 10u && ota_dec[2] == LDR_FAST_CMD)
            ldr_fast_hello();
#else
        ota_service();
#endif
        if (ota_req) {
            int rc;
            ota_req = 0;
            rc = ldr_session();
            fprintf(stderr, "session %d\n", rc);
            if (rc == 0)
                return 0;
        }
        usleep(200);
    }
}
