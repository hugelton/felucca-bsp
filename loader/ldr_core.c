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
 *   2c. with LDR_SLOT_SWITCH (an app whose slot size is set: FL_APP_END), when the installed app's area ends
 *      elsewhere: going down, erases the old app's code above this one's end (it would read as data); going up,
 *      erases [FL_APP_END, LDR_CLEAR_END) when the app sets it (the data a smaller app kept where this one's now is);
 *   3. writes ONLY the app area [0x4000, FL_APP_END) (0x93000, or the app's bigger slot), sector by sector, skipping
 *      sectors that are already equal, and verifies each one; a sector that
 *      comes in different from the checked pass is read again, never written;
 *      the flash head (SPL, isd_config) is never written;
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
 *
 * Hooks from the platform (loader.c, ldr_test.c):
 *   ldr_fread(off, p, n)  ldr_erase(off)  ldr_prog(off, p, n)   flash, 0 = ok
 *   ldr_record_clear()    forget the RAM update record
 *   ldr_progress(done, total)  both passes on one scale: the check (2b, flash.bin) then the write (3, the app area)
 * plus the ota.c hooks (frames, time, idle). */
#define LDR_APP_LO 0x4000u
#ifndef FL_APP_END
#define FL_APP_END 0x93000u                     /* (fm1_flash.h's: the app area's end, or the app's bigger slot) */
#endif
#define LDR_APP_HI FL_APP_END
#define LDR_REC_LO FL_APP_END                   /* update records live above the app ... */
#define LDR_REC_HI 0xFC000u                     /* ... and below Felucca's globals (Felucca's records: 0xE4F00, */
#define LDR_DATA_LO (FL_APP_END > 0x97000u ? FL_APP_END : 0x97000u)   /* (the stock firmware's: 0xE8F00), never in
                                                 * Felucca's store (fm1_flash.h FL_DATA_LO), whose */
#define LDR_DATA_HI 0xE0000u                    /* bytes (user samples) the host writes */
#define LDR_NSEC ((LDR_APP_HI - LDR_APP_LO) >> 12)
#define LDR_FL_MAX 0x100000u                    /* flash.bin: at most the 1 MiB part */

static int ldr_fread(uint32_t off, void *p, uint32_t n);
static int ldr_erase(uint32_t off);
static int ldr_prog(uint32_t off, const void *p, uint32_t n);
static void ldr_record_clear(void);
static void ldr_progress(uint32_t done, uint32_t total);

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
 * the CRC16 of each app sector as it came in */
static int ldr_check(uint32_t fl_off, uint32_t fl_size, uint32_t want, uint16_t *sc, uint8_t *b)
{
    uint32_t i, n, crc = 0;
    for (i = 0; i < fl_size; i += n) {
        n = fl_size - i > 512u ? 512u : fl_size - i;
        if (ota_read(fl_off + i, b, n))
            return -7;
        crc = ota_crc16(b, n, crc);
        if (!((i + n) & 0xFFFu))
            ldr_progress(i + n, fl_size + (LDR_APP_HI - LDR_APP_LO));
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

#if defined(LDR_SLOT_SWITCH) && LDR_SLOT_SWITCH
/* 2c. the installed app's slot against this one's (before anything is written, so a resumed install, the old head
 * still there, does the same): its end from its app area head (0x4000, the chip key's cipher: the block it heads, as
 * fm1pkg_make.py writes it, rounded up to a sector); no head of that format: nothing cleared */
static int ldr_slot_switch(uint32_t key, uint8_t *b)
{
    uint32_t end, s, lo = 0, hi = 0;
    if (ldr_fread(LDR_APP_LO, b, 32))
        return -8;
    ldr_sfc(b, 32, 0, key);
    if (ota_crc16(b + 2, 30, 0) != ota_rd16(b))
        return 0;
    end = (LDR_APP_LO + ota_rd32(b + 8) + 0xFFFu) & ~0xFFFu;
    if (end > LDR_APP_HI && end <= LDR_DATA_HI)
        lo = LDR_APP_HI, hi = end;                   /* (a bigger app's code tail) */
#ifdef LDR_CLEAR_END
    else if (end < LDR_APP_HI)
        lo = LDR_APP_HI, hi = LDR_CLEAR_END;         /* (a smaller app's data where this one keeps its own) */
#endif
    for (s = lo; s < hi; s += 0x1000u)
        if (ldr_erase(s))
            return -9;
    return 0;
}
#endif

static int ldr_session(void)
{
    static uint8_t hdr[0x400], sec[0x1000], cur[0x1000];
    static uint16_t sc[LDR_NSEC];
    uint32_t i, k, fl_off, fl_size, ota_off, ota_len, key, s, want = ~0u, done = 0;   /* (no CRC16: no flash.bin entry) */
    int rc;
    /* 1. UFW header + entry list (ota.c; it leaves the entries decoded in hdr) */
    if ((rc = ota_ufw(hdr, &fl_off, &fl_size, &ota_off, &ota_len)) != 0)
        return rc;
    if (!fl_off || fl_size < LDR_APP_HI || fl_size > LDR_FL_MAX)
        return -3;
    for (i = 0; i < ota_rd16(hdr + 8); i++)
        if (ota_rd16(hdr + 0x40 + i * 0x50u) == 0)
            want = ota_rd16(hdr + 0x40 + i * 0x50u + 4u);   /* flash.bin's data CRC16 */
    /* 2. the package's app area must decrypt with this chip's key */
    if ((rc = ldr_chip_key(&key)) != 0)
        return -40 + rc;
    if (ota_read(fl_off + LDR_APP_LO, sec, 32))
        return -5;
    ldr_sfc(sec, 32, 0, key);
    if (ota_crc16(sec + 2, 30, 0) != ota_rd16(sec))
        return -6;                                       /* package for another chip key */
    /* 2b. the whole package first: nothing is erased for a damaged one */
    if ((rc = ldr_check(fl_off, fl_size, want, sc, sec)) != 0)
        return rc;
#if defined(LDR_SLOT_SWITCH) && LDR_SLOT_SWITCH
    if ((rc = ldr_slot_switch(key, sec)) != 0)
        return rc;
#endif
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
        if (!ota_memeq(sec, cur, 0x1000u)) {
            uint32_t tries;
            for (tries = 0; tries < 2u; tries++) {
                if (ldr_erase(s))
                    return -9;
                for (k = 0; k < 0x1000u; k += 256u)
                    if (ldr_prog(s + k, sec + k, 256))
                        return -10;
                if (ldr_fread(s, cur, 0x1000u))
                    return -8;
                if (ota_memeq(sec, cur, 0x1000u))
                    break;
            }
            if (tries == 2u)
                return -11;                              /* verify failed twice */
        }
        done += 0x1000u;
        ldr_progress(fl_size + done, fl_size + (LDR_APP_HI - LDR_APP_LO));
    }
    /* 4. finish: the host confirms, the record goes, the SPL boots the app */
    for (i = 0; i < 4u; i++)
        if (!ota_read(0xF0000000u, sec, 8))
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
