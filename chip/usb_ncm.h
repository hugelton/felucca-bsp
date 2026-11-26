/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 w0ts */
/* CDC-NCM 1.0's transfer blocks, 16-bit form (NTB16): what chip/usb.c's network function (FELUCCA_NCM) reads from the
 * computer and writes to it. Plain data, no USB here (tests/ncm_test.c runs it on the host).
 *   a block   NTH16 (12 bytes: "NCMH", its length 12, a sequence, the block's length, the first NDP's offset), then one
 *             or more NDP16s ("NCM0", its length, the next NDP's offset, then (offset, length) pairs ended by (0, 0)),
 *             each pair a datagram (an Ethernet frame without its CRC) somewhere in the block
 *   in        what the computer sends: any number of NDPs and datagrams, each checked against the block's bounds;
 *             a block that is not one is dropped whole (ncm_ntb_each's -1)
 *   out       what the FM-1 sends: one datagram a block, NTH16 at 0, the NDP at 12 (16 bytes: one pair and the end),
 *             the datagram at NCM_OUT_HDR = 28 (a multiple of 4: wNdpInDivisor 4, remainder 0, as GET_NTB_PARAMETERS
 *             says) */
#ifndef USB_NCM_H
#define USB_NCM_H
#include <stdint.h>

#define NCM_NTH16_LEN 12u
#define NCM_NDP16_MIN 16u                 /* the signature, its length, the next one's, one pair and the end */
#define NCM_OUT_HDR (NCM_NTH16_LEN + NCM_NDP16_MIN)   /* 28: where the one datagram of a block the FM-1 sends begins */
#define NCM_ETH_MIN 14u                   /* a datagram shorter than an Ethernet header is no frame */
#define NCM_NDP_MAX 8u                    /* NDPs followed in one block (a loop guard: hosts send one) */

static inline uint32_t ncm_rd16(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8; }
static inline void ncm_wr16(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

/* every datagram of the block at b (n bytes received), in order, to f; returns the datagrams given, or -1 when the
 * block is not an NTB16 (then none were given: the whole block is checked before the first) */
static inline int ncm_ntb_each(const uint8_t *b, uint32_t n, void (*f)(void *ctx, const uint8_t *d, uint32_t len), void *ctx)
{
    uint32_t pass, blk, ndp, hops, i, cnt = 0;
    if (n < NCM_NTH16_LEN || b[0] != 'N' || b[1] != 'C' || b[2] != 'M' || b[3] != 'H' || ncm_rd16(b + 4) != NCM_NTH16_LEN)
        return -1;
    blk = ncm_rd16(b + 8);
    if (blk > n || blk < NCM_NTH16_LEN + NCM_NDP16_MIN)
        return -1;
    for (pass = 0; pass < 2u; pass++) {             /* the first pass checks, the second gives */
        ndp = ncm_rd16(b + 10);
        for (hops = 0; ndp; hops++) {
            uint32_t len;
            if (hops >= NCM_NDP_MAX || (ndp & 3u) || ndp < NCM_NTH16_LEN || ndp + 8u > blk)
                return -1;
            if (b[ndp] != 'N' || b[ndp + 1] != 'C' || b[ndp + 2] != 'M' || b[ndp + 3] != '0')
                return -1;
            len = ncm_rd16(b + ndp + 4);
            if (len < NCM_NDP16_MIN || (len & 3u) || ndp + len > blk)
                return -1;
            for (i = 8u; i + 4u <= len; i += 4u) {
                uint32_t off = ncm_rd16(b + ndp + i), dl = ncm_rd16(b + ndp + i + 2u);
                if (!off || !dl)
                    break;                          /* the end of this NDP's list */
                if (off < NCM_NTH16_LEN || off + dl > blk || dl < NCM_ETH_MIN)
                    return -1;
                if (pass)
                    f(ctx, b + off, dl);
                else
                    cnt++;
            }
            ndp = ncm_rd16(b + ndp + 6);
        }
    }
    return (int)cnt;
}

/* the block carrying the frame (len bytes, at least an Ethernet header) into out (cap bytes), the sequence seq; the
 * block's length, or 0 when it does not fit. The frame may already be at out + NCM_OUT_HDR (then it is not copied) */
static inline uint32_t ncm_ntb_one(uint8_t *out, uint32_t cap, uint32_t seq, const uint8_t *frame, uint32_t len)
{
    uint32_t i, total = NCM_OUT_HDR + len;
    if (len < NCM_ETH_MIN || total > cap || total > 0xFFFFu)
        return 0;
    if (frame != out + NCM_OUT_HDR)
        for (i = 0; i < len; i++)
            out[NCM_OUT_HDR + i] = frame[i];
    out[0] = 'N', out[1] = 'C', out[2] = 'M', out[3] = 'H';
    ncm_wr16(out + 4, NCM_NTH16_LEN);
    ncm_wr16(out + 6, seq);
    ncm_wr16(out + 8, total);
    ncm_wr16(out + 10, NCM_NTH16_LEN);              /* the NDP right after */
    out[12] = 'N', out[13] = 'C', out[14] = 'M', out[15] = '0';
    ncm_wr16(out + 16, NCM_NDP16_MIN);
    ncm_wr16(out + 18, 0);                          /* no next NDP */
    ncm_wr16(out + 20, NCM_OUT_HDR);
    ncm_wr16(out + 22, len);
    ncm_wr16(out + 24, 0);                          /* the list's end */
    ncm_wr16(out + 26, 0);
    return total;
}
#endif
