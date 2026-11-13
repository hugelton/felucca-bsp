/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* Felucca update loader, the portable part (step 2 of the M-UPGRADE update
 * protocol).
 *
 * The host serves the package ("logical image") with cmd 0x30 reads (ota.c).
 * The loader:
 *   1. reads the UFW header and finds flash.bin;
 *   2. checks that the package's app area decrypts with THIS chip's key
 *      (taken from isd_config.ini in the device's own flash head), so a
 *      package for another key is refused before anything is erased;
 *   2b. reads the whole flash.bin once and checks it against the CRC16 of its
 *      UFW entry (the vendor tools' check), keeping each app sector's CRC16:
 *      a damaged or truncated package is refused before anything is erased;
 *   3. writes ONLY the app area [0x4000, 0x93000), sector by sector, skipping
 *      sectors that are already equal, and verifies each one; a sector that
 *      comes in different from the checked pass is read again, never written;
 *      the flash head (SPL, isd_config) is never written;
 *   3b. writes the resource region from the package's res.bin entry, if it has one (ldr_res: an app's read-only
 *      data kept out of its image, fm1-base device/res.c), the app being written already: nothing there decides
 *      whether the FM-1 starts, and a failure there never fails the install (the app does without and says so);
 *   4. asks 0xF0000000 ("success"), then invalidates the update record (RAM
 *      and flash: 4K boundary - 256 between 0x93000 and 0xFC000, outside
 *      Felucca's store, where only user data lives) so the SPL boots the new
 *      app, and resets. Only the records go: the loader's own staging sector
 *      (ota.c's 0xE4F00) is erased, as before; a record anywhere else (the
 *      stock firmware's in the SDK's VM, an app's data that reads as one) is
 *      made invalid where it is, its first bytes programmed to 0, and the
 *      rest of its sector is kept (ldr_record_kill).
 * Power loss during 3: the record is still there, the SPL runs the loader
 * again on the next power-on and the host can resume.
 * The fast install (LDR_FAST_CHUNK, when the host says it can: ldr_fast below) replaces 2b and 3 by one pass over a
 * per-sector CRC table: only the sectors that differ are sent, 4 KiB at a time, each erased while it comes in.
 *
 * Hooks from the platform (loader.c, ldr_test.c):
 *   ldr_fread(off, p, n)  ldr_erase(off)  ldr_prog(off, p, n)   flash, 0 = ok
 *   ldr_record_clear()    forget the RAM update record
 *   ldr_progress(done, total)  both passes on one scale: the check (2b, flash.bin) then the write (3, the app area);
 *                              the fast install: the table's bytes, then each app sector compared or written
 *   with LDR_FAST_CHUNK (the fast install, ldr_fast below): ldr_erase_begin(off) ldr_erase_end(), an erase while a
 *   sector comes in (no flash access between)
 *   ldr_res_window(lo, hi) the data area's sectors [lo, hi) open to ldr_erase / ldr_prog while 3b writes them (0, 0:
 *                          closed again); the platform keeps the rest of the data area shut
 * plus the ota.c hooks (frames, time, idle). */
#define LDR_APP_LO 0x4000u
#define LDR_APP_HI 0x93000u
#define LDR_REC_LO 0x93000u                     /* update records live above the app ... */
#define LDR_REC_HI 0xFC000u                     /* ... and below Felucca's globals (Felucca's records: 0xE4F00, */
#define LDR_DATA_LO 0x97000u                    /* the stock firmware's: 0xE8F00), never in Felucca's store, whose */
#define LDR_DATA_HI 0xE0000u                    /* bytes (user samples) the host writes */
#define LDR_NSEC ((LDR_APP_HI - LDR_APP_LO) >> 12)
#define LDR_FL_MAX 0x100000u                    /* flash.bin: at most the 1 MiB part */
#define LDR_RES_TYPE 101u                       /* the package's res.bin (fm1-base tools/fm1pkg_make.py RES_TYPE, as
                                                 * Melodee's); its entry's word at LDR_RES_AT: where the region goes */
#define LDR_RES_AT 0x14u
#define LDR_RES_MAGIC 0x31534552u               /* "RES1": the region's commit record (fm1-base device/res.c) */
#define LDR_ST_MAGIC 0x30583058u                /* "X0X0": a saved object's commit record (fm1-base device/storage.c) */

static int ldr_fread(uint32_t off, void *p, uint32_t n);
static int ldr_erase(uint32_t off);
static int ldr_prog(uint32_t off, const void *p, uint32_t n);
static void ldr_record_clear(void);
static void ldr_progress(uint32_t done, uint32_t total);
static void ldr_res_window(uint32_t lo, uint32_t hi);

/* SFC cipher: per 32-byte block, key ^ (block offset >> 2) */
static void ldr_sfc(uint8_t *p, uint32_t n, uint32_t base_off, uint32_t key)
{
    uint32_t i, j;
    for (i = 0; i < n; i += 32u) {
        uint32_t k = key ^ ((base_off + i) >> 2);
        for (j = 0; j < 32u && i + j < n; j++) {
            p[i + j] ^= (uint8_t)k;
            k = ((k << 1) ^ (k & 0x8000u ? 0x1021u : 0u)) & 0xFFFFu;
        }
    }
}

/* chip key from the device's flash head: JLFS top entries (ENC 0xFFFF) -> isd_config.ini blob */
static int ldr_chip_key(uint32_t *key)
{
    uint8_t e[32], blob[34];
    uint32_t off, i, sum, k;
    for (off = 32; off < 0x400u; off += 32u) {
        if (ldr_fread(off, e, 32))
            return -1;
        ota_jl_enc(e, 32);
        if (ota_crc16(e + 2, 30, 0) != ota_rd16(e))
            return -2;
        if (!ota_memeq(e + 16, (const uint8_t *)"isd_config.ini", 15)) {
            if (ota_rd16(e + 14))
                return -3;                               /* last entry, not found */
            continue;
        }
        if (ldr_fread(ota_rd32(e + 4), blob, 34) || ota_crc16(blob, 32, 0) != ota_rd16(blob + 32))
            return -4;
        for (i = 0, sum = 0; i < 16u; i++)
            sum += blob[i];
        sum &= 0xFFu;
        sum = sum >= 0xE0u ? 0xAAu : sum <= 0x10u ? 0x55u : sum;
        for (i = 0, k = 0; i < 16u; i++)
            if ((uint32_t)(blob[16 + i] ^ blob[15 - i]) < sum)
                k |= 1u << i;
        *key = k;
        return 0;
    }
    return -3;
}

/* 2b. flash.bin [0, fl_size) from the host, before any erase: its CRC16 must be the UFW entry's (want); sc gets
 * the CRC16 of each app sector as it came in. total: the bar's scale */
static int ldr_check(uint32_t fl_off, uint32_t fl_size, uint32_t want, uint16_t *sc, uint8_t *b, uint32_t total)
{
    uint32_t i, n, crc = 0;
    for (i = 0; i < fl_size; i += n) {
        n = fl_size - i > 512u ? 512u : fl_size - i;
        if (ota_read(fl_off + i, b, n))
            return -7;
        crc = ota_crc16(b, n, crc);
        if (!((i + n) & 0xFFFu))
            ldr_progress(i + n, total);
        if (i >= LDR_APP_LO && i < LDR_APP_HI) {     /* (512-byte reads from 0: never across a sector) */
            uint32_t k = (i - LDR_APP_LO) >> 12;
            sc[k] = (uint16_t)ota_crc16(b, n, (i & 0xFFFu) ? sc[k] : 0u);
        }
    }
    return crc == want ? 0 : -12;
}

/* an update record as the loader takes one: "AT" at bytes 6..7, a nonzero CRC-16 at 0 over the 78 bytes from 2 */
static int ldr_is_record(const uint8_t *r)
{
    return ota_rd16(r + 6) == 0x5441u && ota_rd16(r) && ota_rd16(r) == ota_crc16(r + 2, 78, 0);
}

/* a record outside the loader's own staging, at at (r: its 80 bytes as read): made invalid where it is, nothing else
 * of its sector changed. Its first k bytes are programmed to 0 (NOR: bits only go to 0, no erase): the CRC field 0,
 * the layout's magic and "AT" gone, and k grows from 8 until the CRC over bytes 2..79 is nonzero, so neither a CRC
 * check nor "AT" can take it any more. 0 = read back, no record there now */
static int ldr_record_kill(uint32_t at, uint8_t *r)
{
    static uint8_t zero[80];                         /* (in RAM, as the driver wants its sources) */
    uint32_t k, i;
    for (k = 0; k < 8u; k++)
        r[k] = 0;
    while (k < 80u && !ota_crc16(r + 2, 78, 0))
        r[k++] = 0;
    if (!ota_crc16(r + 2, 78, 0) || ldr_prog(at, zero, k) || ldr_fread(at, r, 80))
        return -1;
    for (i = 0; i < k; i++)
        if (r[i])
            return -1;
    return ota_crc16(r + 2, 78, 0) ? 0 : -1;
}

/* 1. and 2.: the UFW header and flash.bin's entry (its data CRC16 in want), the chip key */
static int ldr_head(uint8_t *hdr, uint8_t *b, uint32_t *fl_off, uint32_t *fl_size, uint32_t *want, uint32_t *key)
{
    uint32_t i, ota_off, ota_len;
    int rc;
    *want = ~0u;                                         /* (no CRC16: no flash.bin entry) */
    /* 1. UFW header + entry list (ota.c; it leaves the entries decoded in hdr) */
    if ((rc = ota_ufw(hdr, fl_off, fl_size, &ota_off, &ota_len)) != 0)
        return rc;
    if (!*fl_off || *fl_size < LDR_APP_HI || *fl_size > LDR_FL_MAX)
        return -3;
    for (i = 0; i < ota_rd16(hdr + 8); i++)
        if (ota_rd16(hdr + 0x40 + i * 0x50u) == 0)
            *want = ota_rd16(hdr + 0x40 + i * 0x50u + 4u);   /* flash.bin's data CRC16 */
    /* 2. the package's app area must decrypt with this chip's key */
    if ((rc = ldr_chip_key(key)) != 0)
        return -40 + rc;
    if (ota_read(*fl_off + LDR_APP_LO, b, 32))
        return -5;
    ldr_sfc(b, 32, 0, *key);
    if (ota_crc16(b + 2, 30, 0) != ota_rd16(b))
        return -6;                                       /* package for another chip key */
    return 0;
}

/* erase (unless erased already: the fast install's first try), program, verify: twice at most */
static int ldr_write_sector(uint32_t s, const uint8_t *sec, uint8_t *cur, uint32_t erased)
{
    uint32_t tries, k;
    for (tries = 0; tries < 2u; tries++) {
        if (!(erased && !tries) && ldr_erase(s))
            return -9;
        for (k = 0; k < 0x1000u; k += 256u)
            if (ldr_prog(s + k, sec + k, 256))
                return -10;
        if (ldr_fread(s, cur, 0x1000u))
            return -8;
        if (ota_memeq(sec, cur, 0x1000u))
            return 0;
    }
    return -11;                                          /* verify failed twice */
}

/* CRC-32 (zlib's), bit by bit; running: from 0xFFFFFFFF, the CRC is the complement of the end */
static uint32_t ldr_crc32_run(uint32_t c, const uint8_t *p, uint32_t n)
{
    uint32_t k;
    while (n--) {
        c ^= *p++;
        for (k = 0; k < 8u; k++)
            c = (c >> 1) ^ (c & 1u ? 0xEDB88320u : 0u);
    }
    return c;
}
static uint32_t ldr_crc32(const uint8_t *p, uint32_t n) { return ~ldr_crc32_run(0xFFFFFFFFu, p, n); }

/* a 32-byte commit record (device/res.c's, storage.c's): its magic first, the CRC-32 of the 28 bytes before it last */
static int ldr_record(const uint8_t *h, uint32_t magic)
{
    return ota_rd32(h) == magic && ota_rd32(h + 28) == ldr_crc32(h, 28);
}

/* the package's res.bin entry (the entry list as ota_ufw left it, decoded): its offset, length and place, 0 if none */
static void ldr_res_entry(const uint8_t *hdr, uint32_t *off, uint32_t *size, uint32_t *base)
{
    uint32_t i;
    *off = *size = *base = 0;
    for (i = 0; i < ota_rd16(hdr + 8); i++) {
        const uint8_t *e = hdr + 0x40 + i * 0x50u;
        if (ota_rd16(e) == LDR_RES_TYPE) {
            *off = ota_rd32(e + 8);
            *size = ota_rd32(e + 12);
            *base = ota_rd32(e + LDR_RES_AT);
        }
    }
}

/* n bytes of the package from src, chunk at a time (512: every host; 4 KiB: one that said yes to the fast install) */
static int ldr_pkg(uint32_t src, uint8_t *b, uint32_t n, uint32_t chunk)
{
    uint32_t k;
    for (k = 0; k < n; k += chunk)
        if (ota_read(src + k, b + k, n - k < chunk ? n - k : chunk))
            return -1;
    return 0;
}

/* the CRC-32 of the flash's [at, at + n), through cur */
static int ldr_flash_crc(uint32_t at, uint32_t n, uint8_t *cur, uint32_t *crc)
{
    uint32_t k, c = 0xFFFFFFFFu;
    for (; n; at += k, n -= k) {
        k = n > 0x1000u ? 0x1000u : n;
        if (ldr_fread(at, cur, k))
            return -1;
        c = ldr_crc32_run(c, cur, k);
    }
    *crc = ~c;
    return 0;
}

/* 3b. the resource region (fm1-base device/res.c) from the package's res.bin: the region as the FM-1 holds it, its
 * commit record (magic, the payload's length and CRC-32, its own CRC) then the payload, padded to whole sectors; off,
 * size: its entry, base: where the region starts (whole sectors of the data area, or nothing is done). Kept when it
 * holds this build's region whole already (an update that keeps the resources reads one header from the host). Left
 * alone when any of its sectors holds a saved object (storage.c's commit record): saves an app has not moved out of
 * the region yet, or another firmware's; the app then does without and says so, and the host's own write (res.c
 * RES_REQ: fm1_resources.py, the installer page) or the next install can do it. Otherwise the record's sector goes
 * blank first, every other sector is written and verified, then the first one's payload; the record last, only over a
 * payload whose CRC-32 is its own, so a valid record only ever stands over a whole region. done, total: the bar.
 * After Melodee's (firmware/loader/ldr_core.c ldr_res, by Kerem Kilic, Ellic Studio; used under MPL-2.0 with
 * permission) */
static void ldr_res(uint32_t off, uint32_t size, uint32_t base, uint32_t chunk, uint8_t *sec, uint8_t *cur,
                    uint32_t done, uint32_t total)
{
    uint32_t s, k, len, crc, have;
    if (!off || !size || (size & 0xFFFu) || (base & 0xFFFu) || base < LDR_DATA_LO || base >= LDR_DATA_HI ||
        size > LDR_DATA_HI - base || ldr_pkg(off, sec, 512u, 512u) || !ldr_record(sec, LDR_RES_MAGIC) ||
        (len = ota_rd32(sec + 4)) > size - 32u)
        return;
    crc = ota_rd32(sec + 8);
    if (ldr_fread(base, cur, 32))
        return;
    if (ota_memeq(sec, cur, 32) && !ldr_flash_crc(base + 32u, len, cur, &have) && have == crc)
        return;                                          /* this build's, whole */
    for (s = base; s < base + size; s += 0x1000u)
        if (ldr_fread(s, cur, 32) || ldr_record(cur, LDR_ST_MAGIC))
            return;                                      /* saves there */
    ldr_res_window(base, base + size);
    if (ldr_erase(base))                                 /* the commit record goes first */
        goto out;
    for (s = base + 0x1000u; s < base + size; s += 0x1000u) {
        if (ldr_pkg(off + (s - base), sec, 0x1000u, chunk) || ldr_fread(s, cur, 0x1000u) ||
            (!ota_memeq(sec, cur, 0x1000u) && ldr_write_sector(s, sec, cur, 0)))
            goto out;
        ldr_progress(done += 0x1000u, total);
    }
    if (ldr_pkg(off, sec, 0x1000u, chunk) || ldr_prog(base + 32u, sec + 32, 224))   /* the first page's payload */
        goto out;
    for (k = 0x100u; k < 0x1000u; k += 0x100u)
        if (ldr_prog(base + k, sec + k, 256))
            goto out;
    if (!ldr_flash_crc(base + 32u, len, cur, &have) && have == crc)
        (void)ldr_prog(base, sec, 32);                   /* the commit record */
    ldr_progress(done + 0x1000u, total);
out:
    ldr_res_window(0, 0);
}

/* 4. finish: the host confirms, the record goes, the SPL boots the app */
static int ldr_finish(uint8_t *b)
{
    uint32_t i, s;
    for (i = 0; i < 4u; i++)
        if (!ota_read(0xF0000000u, b, 8))
            break;
    ldr_record_clear();
    for (s = LDR_REC_HI; s > LDR_REC_LO; s -= 0x1000u) {   /* flash records: 4K boundary - 256 */
        uint8_t r[80];
        if (s - 0x1000u >= LDR_DATA_LO && s - 0x1000u < LDR_DATA_HI)
            continue;                                    /* Felucca's store: user data that may look like one */
        if (ldr_fread(s - 0x100u, r, 80))
            break;
        if (!ldr_is_record(r))
            continue;
        if (s - 0x100u == OTA_RES || ldr_record_kill(s - 0x100u, r))
            ldr_erase(s - 0x1000u);                      /* the loader's own staging sector (ota.c stages it whole
                                                          * anyway); else a record that would not go in place: a
                                                          * record left would bring the SPL back here at every start */
    }
    return 0;
}

static uint8_t ldr_hdr[0x400], ldr_sec[0x1000], ldr_cur[0x1000];

static int ldr_classic(void)
{
    static uint16_t sc[LDR_NSEC];
    uint8_t *const hdr = ldr_hdr, *const sec = ldr_sec, *const cur = ldr_cur;
    uint32_t k, fl_off, fl_size, key, s, want, done = 0, r_off, r_size, r_base, total;
    int rc;
    if ((rc = ldr_head(hdr, sec, &fl_off, &fl_size, &want, &key)) != 0)
        return rc;
    ldr_res_entry(hdr, &r_off, &r_size, &r_base);
    total = fl_size + (LDR_APP_HI - LDR_APP_LO) + r_size;
    /* 2b. the whole package first: nothing is erased for a damaged one */
    if ((rc = ldr_check(fl_off, fl_size, want, sc, sec, total)) != 0)
        return rc;
    /* 3. app area, sector by sector */
    for (s = LDR_APP_LO; s < LDR_APP_HI; s += 0x1000u) {
        uint32_t again;
        for (again = 0;; again++) {                      /* as checked in 2b, else read it again */
            for (k = 0; k < 0x1000u; k += 512u)
                if (ota_read(fl_off + s + k, sec + k, 512))
                    return -7;
            if (ota_crc16(sec, 0x1000u, 0) == sc[(s - LDR_APP_LO) >> 12])
                break;
            if (again == 2u)
                return -13;
        }
        if (ldr_fread(s, cur, 0x1000u))
            return -8;
        if (!ota_memeq(sec, cur, 0x1000u) && (rc = ldr_write_sector(s, sec, cur, 0)) != 0)
            return rc;
        done += 0x1000u;
        ldr_progress(fl_size + done, total);
    }
    ldr_res(r_off, r_size, r_base, 512u, sec, cur, fl_size + done, total);
    return ldr_finish(sec);
}

#ifdef LDR_FAST_CHUNK
/* The fast install (the host said so before the upgrade command: ldr_fast_hello; Optimist's docs/INSTALL-SPEED.md).
 * One pass instead of two, 4 KiB reads instead of 512 B, and only the sectors that differ come over the wire:
 *   2b'. the host's table of flash.bin, one entry per 4 KiB sector [CRC16 (the UFW's), CRC-32 (zlib's)], read at
 *        LDR_TBL + flash.bin's offset; the CRC16s combined must give the UFW entry's CRC16 of the whole flash.bin, so
 *        the table is the package's (a damaged package or table is refused before anything is erased, as 2b);
 *   3'.  a sector whose flash already has both CRCs is kept and never sent; any other one is erased while it comes in
 *        (one 4 KiB read), checked against both CRCs (three tries: never written otherwise), programmed and verified
 *        byte for byte. A cut-off install resumes the same way: what was written already is skipped.
 *        The host learns how far the comparison is (cmd 0x32, ldr_check_note): its bar moves over the sectors kept,
 *        which it is never asked for (a resumed install: most of them).
 * A flash.bin that is not whole sectors goes the classic way. */
#define LDR_FAST_CMD 0x31u
#define LDR_CHECK_CMD 0x32u                     /* [sectors compared, all sectors: 16-bit LE each], to the host */
#define LDR_CHECK_EVERY 8u                      /* at most a 0x32 frame every 8 sectors ... */
#define LDR_CHECK_MS 100u                       /* ... or 100 ms, and the last */
#define LDR_TBL 0xD0000000u
#define LDR_NMAX (LDR_FL_MAX >> 12)
_Static_assert(LDR_FAST_CHUNK == 0x1000u && LDR_NMAX * 6u <= LDR_FAST_CHUNK, "one read per sector, the table in one");
static uint8_t ldr_fast_on;

/* cmd 0x31 [version 1, the host's largest read, 16-bit LE] -> the same frame with ours; the word holds for the next
 * session only. A host that says nothing (every older one) gets the classic protocol */
static void ldr_fast_hello(void)
{
    static uint8_t r[3] = {1, (uint8_t)LDR_FAST_CHUNK, (uint8_t)(LDR_FAST_CHUNK >> 8)};
    if (ota_dec[6] != 1u || ota_rd16(ota_dec + 7) < LDR_FAST_CHUNK)
        return;
    ldr_fast_on = 1;
    ota_send_msg(LDR_FAST_CMD, r, 3);
}

/* the CRC16 (ota_crc16's) of 4 KiB of zeros from c. CCITT's polynomial is (x + 1) times a primitive one of degree 15, so
 * x^32767 = 1 and 32768 zero bits move the register as one bit does: one step, not 32768 (ota_crc16 over 4096 zero
 * bytes ran ~46 instructions a byte: ~48 M for a 1 MiB flash.bin before the bar moved; tests/ldr_table_test.c: the same
 * as ota_crc16) */
static uint32_t ldr_zeros4k(uint32_t c)
{
    return (c & 0x8000u ? (c << 1) ^ 0x1021u : c << 1) & 0xFFFFu;
}

/* the table's CRC16s combined: the UFW entry's CRC16 of the whole flash.bin (n steps of 16 bits: no time to show) */
static int ldr_table_ok(const uint8_t *t, uint32_t n, uint32_t want)
{
    uint32_t k, c = 0;
    for (k = 0; k < n; k++)                              /* crc(A || B) = crc(A shifted over |B| zeros) ^ crc(B) */
        c = ldr_zeros4k(c) ^ ota_rd16(t + k * 6u);
    return c == want;
}

static int ldr_same(const uint8_t *b, const uint8_t *t)
{
    return ota_crc16(b, 0x1000u, 0) == ota_rd16(t) && ldr_crc32(b, 0x1000u) == ota_rd32(t + 2);
}

/* one sector from the host, as the table says, while its erase runs */
static int ldr_fetch(uint32_t src, uint8_t *b, const uint8_t *t)
{
    uint32_t again;
    for (again = 0; again < 3u; again++) {
        if (ota_read(src, b, 0x1000u))
            return -7;
        if (ldr_same(b, t))
            return 0;
    }
    return -13;                                          /* it came in different three times: not written */
}

/* cmd 0x32 [k sectors of the app area compared, of n: 16-bit LE each]: queued, sent by the USB polls (ldr_progress) */
static void ldr_check_note(uint32_t k, uint32_t n)
{
    uint8_t b[4];
    b[0] = (uint8_t)k;
    b[1] = (uint8_t)(k >> 8);
    b[2] = (uint8_t)n;
    b[3] = (uint8_t)(n >> 8);
    ota_send_msg(LDR_CHECK_CMD, b, 4);
}

static int ldr_fast(void)
{
    static uint8_t tbl[LDR_NMAX * 6u];
    uint8_t *const hdr = ldr_hdr, *const sec = ldr_sec, *const cur = ldr_cur;
    uint32_t fl_off, fl_size, key, s, want, n, total, done = 0, k, note_k = 0, note_ms, r_off, r_size, r_base;
    int rc;
    if ((rc = ldr_head(hdr, sec, &fl_off, &fl_size, &want, &key)) != 0)
        return rc;
    if (fl_size & 0xFFFu)
        return ldr_classic();
    ldr_res_entry(hdr, &r_off, &r_size, &r_base);
    n = fl_size >> 12;
    total = n * 6u + (LDR_APP_HI - LDR_APP_LO) + r_size;  /* the bar: the table read, then every app sector compared
                                                          * (kept or written), from 0: not flash.bin's half of the
                                                          * classic scale, whose check pass takes no time here; then
                                                          * the resource region's sectors */
    if (ota_read(LDR_TBL + fl_off, tbl, n * 6u))
        return -7;
    if (!ldr_table_ok(tbl, n, want))
        return -12;                                      /* nothing erased */
    ldr_progress(n * 6u, total);                         /* (the table's bytes: the bar's scale is what takes time) */
    note_ms = ota_now_ms();
    for (s = LDR_APP_LO; s < LDR_APP_HI; s += 0x1000u) {
        const uint8_t *t = tbl + (s >> 12) * 6u;
        if (ldr_fread(s, cur, 0x1000u))
            return -8;
        if (!ldr_same(cur, t)) {
            if (ldr_erase_begin(s))
                return -9;
            rc = ldr_fetch(fl_off + s, sec, t);
            if (ldr_erase_end())
                return -9;
            if (rc)
                return rc;
            if ((rc = ldr_write_sector(s, sec, cur, 1)) != 0)
                return rc;
        }
        done += 0x1000u;
        k = done >> 12;
        if (k == LDR_NSEC || k - note_k >= LDR_CHECK_EVERY || ota_now_ms() - note_ms >= LDR_CHECK_MS) {
            note_k = k;
            note_ms = ota_now_ms();
            ldr_check_note(k, LDR_NSEC);
        }
        ldr_progress(n * 6u + done, total);
    }
    ldr_res(r_off, r_size, r_base, LDR_FAST_CHUNK, sec, cur, n * 6u + done, total);
    return ldr_finish(sec);
}

static int ldr_session(void)
{
    const uint32_t fast = ldr_fast_on;
    ldr_fast_on = 0;
    return fast ? ldr_fast() : ldr_classic();
}
#else
static int ldr_session(void) { return ldr_classic(); }
#endif
