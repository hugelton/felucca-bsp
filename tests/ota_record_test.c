/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* chip/ota.c's update record (ota_record: UPDATA_PARM, update_mode_api_v2 layout) on the host, for the FM-1's defaults
 * and for a board that sets its own staging area and name (OTA_AREA, OTA_REC_NAME): type 5A0D, result 5A01, magic
 * 5441, the name at +8 (zero-padded to +40), the loader's address at +72, a CRC16 over 2..79 that the SPL accepts, the
 * rest of the buffer cleared. run_ota_test.sh builds it both ways.
 *
 *   cc -std=c11 -O1 -Wall -Wextra -Werror -Wno-unused-function [-DOTA_AREA=0xBB000u -DOTA_REC_NAME='"ota-X_016"']
 *      tests/ota_record_test.c -o ota_record_test && ./ota_record_test */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define FELUCCA_ID "FM-1_900"
static int ota_wire_send(const uint8_t *p, uint32_t n) { (void)p; (void)n; return 0; }
static int ota_frame_get(const uint8_t **p, uint32_t *n) { (void)p; (void)n; return 0; }
static void ota_frame_done(void) {}
static uint32_t ota_now_ms(void) { return 0; }
static void ota_idle(void) {}
static int ota_erase(uint32_t off) { (void)off; return 0; }
static int ota_prog(uint32_t off, const void *p, uint32_t n) { (void)off; (void)p; (void)n; return 0; }
static int ota_fread(uint32_t off, void *p, uint32_t n) { (void)off; memset(p, 0xFF, n); return 0; }
static void ota_show(uint32_t step, int32_t code) { (void)step; (void)code; }
static void ota_commit(const uint8_t *parm) { (void)parm; }
#include "../chip/ota.c"

static int fails;
static void want(const char *what, int ok) { printf("  %s %s\n", ok ? "ok  " : "FAIL", what); fails += !ok; }

int main(void)
{
    uint8_t p[112];
    uint32_t i, rest = 0;
    memset(p, 0xA5, sizeof p);
    ota_record(p, sizeof p);
    want("type 5A0D, result 5A01, magic 5441", ota_rd16(p + 2) == 0x5A0Du && ota_rd16(p + 4) == 0x5A01u && ota_rd16(p + 6) == 0x5441u);
    want("the name at +8: " OTA_REC_NAME, !memcmp(p + 8, OTA_REC_NAME, sizeof OTA_REC_NAME - 1u));
    for (i = 8u + sizeof OTA_REC_NAME - 1u; i < 72u; i++)
        rest |= p[i];
    for (i = 80u; i < sizeof p; i++)
        rest |= p[i];
    want("zero after the name, and past the record", rest == 0u);
    want("the loader's address at +72", ota_rd32(p + 72) == OTA_AREA);
    want("the CRC16 over 2..79 at +0", ota_rd16(p) == ota_crc16(p + 2, 78, 0) && ota_rd16(p) != 0u);
    want("the flash copy at a 4 KiB boundary - 256", (OTA_RES & 0xFFFu) == 0xF00u);
    printf("ota record (%s at 0x%05X): %s\n", OTA_REC_NAME, (unsigned)OTA_AREA, fails ? "FAILED" : "all passed");
    return fails != 0;
}
