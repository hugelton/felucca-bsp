/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
#include "ble_midi_host.h"

enum { OP_RESET = 0x0C03, OP_LE_BUF = 0x2002, OP_ADV_PARAM = 0x2006, OP_ADV_DATA = 0x2008, OP_SCAN_RSP = 0x2009,
       OP_ADV_EN = 0x200A };
enum { ST_RESET, ST_BUF, ST_PARAM, ST_DATA, ST_SCAN, ST_ENABLE, ST_DONE };
enum { H_GAP = 1, H_NAME_DECL, H_NAME, H_MIDI_SVC, H_MIDI_DECL, H_MIDI, H_CCCD, H_LAST = H_CCCD };
enum { A_ERR = 1, A_MTU = 2, A_MTU_R = 3, A_INFO = 4, A_INFO_R = 5, A_FVAL = 6, A_FVAL_R = 7, A_RTYPE = 8, A_RTYPE_R = 9,
       A_READ = 10, A_READ_R = 11, A_GROUP = 16, A_GROUP_R = 17, A_WRITE = 18, A_WRITE_R = 19, A_NOTIFY = 27,
       A_WCMD = 0x52 };
enum { E_HANDLE = 1, E_READ = 2, E_WRITE = 3, E_PDU = 4, E_UNSUPP = 6, E_NOTFOUND = 10, E_GROUP = 16 };

/* BLE-MIDI service 03B80E5A-EDE8-4B33-A751-6CE34EC4C700, characteristic 7772E5DB-3868-4112-A1A9-F2669D106BF3 (little endian) */
static const uint8_t U_SVC[16] = {0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7, 0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03};
static const uint8_t U_CHR[16] = {0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1, 0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77};
static const uint8_t T_PRIM[2] = {0x00, 0x28}, T_DECL[2] = {0x03, 0x28}, T_NAME[2] = {0x00, 0x2A}, T_CCCD[2] = {0x02, 0x29};
static const uint8_t V_GAP[2] = {0x00, 0x18};
static const uint8_t D_NAME[5] = {0x02, H_NAME, 0x00, 0x00, 0x2A};                      /* read; value handle 3; 0x2A00 */
static const uint8_t D_MIDI[19] = {0x16, H_MIDI, 0x00, 0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1, 0x12, 0x41,
                                   0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77};                  /* read, write w/o response, notify */
static const struct { uint8_t tlen; const uint8_t *type; } ATTR[H_LAST] = {
    {2, T_PRIM}, {2, T_DECL}, {2, T_NAME}, {2, T_PRIM}, {2, T_DECL}, {16, U_CHR}, {2, T_CCCD}};

static uint8_t value_of(const bleh_t *h, unsigned hd, const uint8_t **v)
{
    switch (hd) {
    case H_GAP: *v = V_GAP; return 2;
    case H_NAME_DECL: *v = D_NAME; return 5;
    case H_NAME: *v = (const uint8_t *)h->name; return h->namelen;
    case H_MIDI_SVC: *v = U_SVC; return 16;
    case H_MIDI_DECL: *v = D_MIDI; return 19;
    case H_CCCD: *v = h->cccd; return 2;
    default: *v = 0; return 0;
    }
}
static unsigned group_end(unsigned hd) { return hd <= H_NAME ? H_NAME : H_LAST; }
static unsigned rd16(const uint8_t *p) { return (unsigned)p[0] | (unsigned)p[1] << 8; }
static void wr16(uint8_t *p, unsigned v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static int tmatch(unsigned hd, const uint8_t *t, unsigned tl)
{
    unsigned i;
    if (ATTR[hd - 1].tlen != tl) return 0;
    for (i = 0; i < tl; i++) if (ATTR[hd - 1].type[i] != t[i]) return 0;
    return 1;
}

/* ---- HCI out ---- */
static void cmd(bleh_t *h, unsigned op, const uint8_t *p, unsigned n)
{
    uint8_t b[3 + 32];
    unsigned i;
    wr16(b, op);
    b[2] = (uint8_t)n;
    for (i = 0; i < n; i++) b[3 + i] = p[i];
    h->cmd_op = (uint16_t)op;
    h->cmd_wait = 1;
    h->send(BLEH_H4_CMD, b, (uint16_t)(3 + n), h->ctx);
}
static void adv_enable(bleh_t *h, unsigned on)
{
    uint8_t e = (uint8_t)on;
    cmd(h, OP_ADV_EN, &e, 1);
}
static void next_cmd(bleh_t *h)
{
    uint8_t b[32];
    unsigned i, n;
    switch (h->step) {
    case ST_RESET: cmd(h, OP_RESET, 0, 0); break;
    case ST_BUF: cmd(h, OP_LE_BUF, 0, 0); break;
    case ST_PARAM:                                  /* 40..80 ms, ADV_IND, public address, all 3 channels, no filter */
        wr16(b, 0x40); wr16(b + 2, 0x80);
        b[4] = 0; b[5] = 0; b[6] = 0;
        for (i = 7; i < 13; i++) b[i] = 0;
        b[13] = 7; b[14] = 0;
        cmd(h, OP_ADV_PARAM, b, 15);
        break;
    case ST_DATA:                                   /* flags + the 128-bit service UUID */
        for (i = 0; i < 32; i++) b[i] = 0;
        b[0] = 21; b[1] = 2; b[2] = 1; b[3] = 6; b[4] = 17; b[5] = 7;
        for (i = 0; i < 16; i++) b[6 + i] = U_SVC[i];
        cmd(h, OP_ADV_DATA, b, 32);
        break;
    case ST_SCAN:                                   /* the name */
        for (i = 0; i < 32; i++) b[i] = 0;
        n = h->namelen;
        b[0] = (uint8_t)(n + 2); b[1] = (uint8_t)(n + 1); b[2] = 9;
        for (i = 0; i < n; i++) b[3 + i] = (uint8_t)h->name[i];
        cmd(h, OP_SCAN_RSP, b, 32);
        break;
    case ST_ENABLE: adv_enable(h, 1); break;
    default: break;
    }
}

/* ---- ATT out ---- */
static void att_send(bleh_t *h, const uint8_t *att, unsigned n)
{
    uint8_t b[8 + BLEH_MTU];
    unsigned i;
    wr16(b, h->conn & 0x0FFFu);                     /* PB = 0: first packet, host to controller */
    wr16(b + 2, 4 + n);
    wr16(b + 4, n);                                 /* L2CAP length, then the ATT fixed channel */
    wr16(b + 6, 4);
    for (i = 0; i < n; i++) b[8 + i] = att[i];
    if (h->credits) h->credits--;
    h->send(BLEH_H4_ACL, b, (uint16_t)(8 + n), h->ctx);
}
static void att_err(bleh_t *h, unsigned op, unsigned hd, unsigned code)
{
    uint8_t r[5] = {A_ERR, (uint8_t)op, 0, 0, (uint8_t)code};
    wr16(r + 2, hd);
    att_send(h, r, 5);
}

static void att_in(bleh_t *h, const uint8_t *p, unsigned n)
{
    uint8_t r[BLEH_MTU];
    unsigned op = p[0], s, e, hd, k, rl;
    if (op & 0x40u) return;                         /* commands (write without response, ...): nothing to answer */
    switch (op) {
    case A_MTU:
        if (n != 3) { att_err(h, op, 0, E_PDU); return; }
        r[0] = A_MTU_R; wr16(r + 1, BLEH_MTU);
        att_send(h, r, 3);
        return;
    case A_INFO:
    case A_GROUP:
    case A_RTYPE:
    case A_FVAL:
        if (n < 5) { att_err(h, op, 0, E_PDU); return; }
        s = rd16(p + 1); e = rd16(p + 3);
        if (!s || s > e) { att_err(h, op, s, E_HANDLE); return; }
        break;
    case A_READ:
    case A_WRITE:
        if (n < 3) { att_err(h, op, 0, E_PDU); return; }
        s = rd16(p + 1); e = 0;
        break;
    default:
        att_err(h, op, 0, E_UNSUPP);
        return;
    }
    if (op == A_INFO) {
        unsigned fmt = 0, len = 2;
        for (hd = s; hd <= e && hd <= H_LAST; hd++) {
            unsigned f = ATTR[hd - 1].tlen == 2 ? 1 : 2, el = 2 + ATTR[hd - 1].tlen;
            if (!fmt) { fmt = f; r[0] = A_INFO_R; r[1] = (uint8_t)f; }
            else if (f != fmt) break;
            if (len + el > BLEH_MTU) break;
            wr16(r + len, hd);
            for (k = 0; k < ATTR[hd - 1].tlen; k++) r[len + 2 + k] = ATTR[hd - 1].type[k];
            len += el;
        }
        if (!fmt) att_err(h, op, s, E_NOTFOUND); else att_send(h, r, len);
    } else if (op == A_GROUP) {
        unsigned len = 2, el = 0;
        if (n != 7 || rd16(p + 5) != 0x2800) { att_err(h, op, s, E_GROUP); return; }
        for (hd = s; hd <= e && hd <= H_LAST; hd++) {
            const uint8_t *v;
            unsigned vl;
            if (!tmatch(hd, T_PRIM, 2)) continue;
            vl = value_of(h, hd, &v);
            if (!el) { el = 4 + vl; r[0] = A_GROUP_R; r[1] = (uint8_t)el; }
            else if (4 + vl != el) break;
            if (len + el > BLEH_MTU) break;
            wr16(r + len, hd); wr16(r + len + 2, group_end(hd));
            for (k = 0; k < vl; k++) r[len + 4 + k] = v[k];
            len += el;
        }
        if (!el) att_err(h, op, s, E_NOTFOUND); else att_send(h, r, len);
    } else if (op == A_FVAL) {
        unsigned len = 1, found = 0;
        if (n < 7 || rd16(p + 5) != 0x2800) { att_err(h, op, s, E_NOTFOUND); return; }
        r[0] = A_FVAL_R;
        for (hd = s; hd <= e && hd <= H_LAST && len + 4 <= BLEH_MTU; hd++) {
            const uint8_t *v;
            unsigned vl = value_of(h, hd, &v);
            if (!tmatch(hd, T_PRIM, 2) || vl != n - 7) continue;
            for (k = 0; k < vl && v[k] == p[7 + k]; k++) ;
            if (k != vl) continue;
            wr16(r + len, hd); wr16(r + len + 2, group_end(hd));
            len += 4; found = 1;
        }
        if (!found) att_err(h, op, s, E_NOTFOUND); else att_send(h, r, len);
    } else if (op == A_RTYPE) {
        unsigned len = 2, el = 0, tl = n - 5;
        if (tl != 2 && tl != 16) { att_err(h, op, s, E_PDU); return; }
        for (hd = s; hd <= e && hd <= H_LAST; hd++) {
            const uint8_t *v;
            unsigned vl;
            if (!tmatch(hd, p + 5, tl)) continue;
            vl = value_of(h, hd, &v);
            if (vl > BLEH_MTU - 4) vl = BLEH_MTU - 4;
            if (!el) { el = 2 + vl; r[0] = A_RTYPE_R; r[1] = (uint8_t)el; }
            else if (2 + vl != el) break;
            if (len + el > BLEH_MTU) break;
            wr16(r + len, hd);
            for (k = 0; k < vl; k++) r[len + 2 + k] = v[k];
            len += el;
        }
        if (!el) att_err(h, op, s, E_NOTFOUND); else att_send(h, r, len);
    } else if (op == A_READ) {
        const uint8_t *v;
        unsigned vl;
        if (s < 1 || s > H_LAST) { att_err(h, op, s, E_HANDLE); return; }
        vl = value_of(h, s, &v);
        if (vl > BLEH_MTU - 1) vl = BLEH_MTU - 1;
        r[0] = A_READ_R;
        for (k = 0; k < vl; k++) r[1 + k] = v[k];
        att_send(h, r, 1 + vl);
    } else {                                        /* A_WRITE */
        if (s != H_CCCD) { att_err(h, op, s, s >= 1 && s <= H_LAST ? E_WRITE : E_HANDLE); return; }
        rl = n - 3;
        if (rl != 2) { att_err(h, op, s, E_PDU); return; }
        h->cccd[0] = p[3] & 1u; h->cccd[1] = 0;     /* notifications only */
        r[0] = A_WRITE_R;
        att_send(h, r, 1);
    }
}

/* ---- ACL in: collect one L2CAP PDU (the controller may split it), hand ATT on ---- */
static void acl_in(bleh_t *h, const uint8_t *p, unsigned n)
{
    unsigned hd, pb, dl;
    if (n < 4) return;
    hd = rd16(p); pb = (hd >> 12) & 3u; dl = rd16(p + 2);
    if ((hd & 0x0FFFu) != h->conn || dl > n - 4) return;
    p += 4;
    if (pb == 2) {                                  /* start */
        unsigned ll;
        h->rx_on = 0;
        if (dl < 4) return;
        ll = rd16(p);
        if (ll > BLEH_MTU || rd16(p + 2) != 4) return;      /* only ATT, only up to the MTU */
        h->rx_need = (uint8_t)(ll + 4); h->rx_have = 0; h->rx_on = 1;
    } else if (pb != 1 || !h->rx_on) {
        return;
    }
    if (h->rx_have + dl > h->rx_need) { h->rx_on = 0; return; }
    { unsigned i; for (i = 0; i < dl; i++) h->rx[h->rx_have + i] = p[i]; }
    h->rx_have = (uint8_t)(h->rx_have + dl);
    if (h->rx_have == h->rx_need) {
        h->rx_on = 0;
        if (h->rx_need > 4) att_in(h, h->rx + 4, (unsigned)h->rx_need - 4u);
    }
}

/* ---- events ---- */
static void evt_in(bleh_t *h, const uint8_t *p, unsigned n)
{
    unsigned code, pl;
    if (n < 2) return;
    code = p[0]; pl = p[1];
    if (pl > n - 2) return;
    p += 2;
    if (code == 0x0E && pl >= 4) {                  /* command complete: credits, opcode, status, return parameters */
        if (!h->cmd_wait || rd16(p + 1) != h->cmd_op) return;
        h->cmd_wait = 0;
        if (p[3] != 0) { h->state = BLEH_ERROR; return; }
        if (h->state != BLEH_STARTING) {
            if (h->want_adv && h->state == BLEH_ADVERTISING) { h->want_adv = 0; adv_enable(h, 1); }
            return;
        }
        if (h->step == ST_BUF) {
            h->acl_total = pl >= 7 && p[6] ? p[6] : 4;
            h->credits = h->acl_total;
        }
        if (++h->step == ST_DONE) { h->state = BLEH_ADVERTISING; return; }
        next_cmd(h);
    } else if (code == 0x3E && pl >= 5 && (p[0] == 0x01 || p[0] == 0x0A)) {     /* (enhanced) connection complete */
        if (p[1] == 0 && p[4] == 1) {               /* success, we are the peripheral */
            h->conn = (uint16_t)(rd16(p + 2) & 0x0FFFu);
            h->state = BLEH_CONNECTED;
            h->cccd[0] = h->cccd[1] = 0;
            h->credits = h->acl_total;
            h->rx_on = 0;
        } else if (h->state == BLEH_ADVERTISING) {
            h->want_adv = 1;
        }
    } else if (code == 0x05 && pl >= 4) {           /* disconnection complete */
        if (h->state == BLEH_CONNECTED && (rd16(p + 1) & 0x0FFFu) == h->conn) {
            h->state = BLEH_ADVERTISING;
            h->cccd[0] = h->cccd[1] = 0;
            h->q_r = h->q_w;
            h->rx_on = 0;
            if (h->cmd_wait) h->want_adv = 1; else adv_enable(h, 1);
        }
    } else if (code == 0x13 && pl >= 1) {           /* number of completed packets: all handles, then all counts */
        unsigned i, c = p[0];
        if (pl < 1 + 4 * c) return;
        for (i = 0; i < c; i++)
            if ((rd16(p + 1 + 2 * i) & 0x0FFFu) == h->conn) {
                unsigned k = (unsigned)h->credits + rd16(p + 1 + 2 * c + 2 * i);
                h->credits = (uint8_t)(k > h->acl_total ? h->acl_total : k);
            }
    }
}

/* ---- public ---- */
void bleh_init(bleh_t *h, bleh_send_fn send, void *ctx, const char *name)
{
    unsigned i;
    uint8_t *z = (uint8_t *)h;
    for (i = 0; i < sizeof *h; i++) z[i] = 0;
    h->send = send; h->ctx = ctx;
    for (i = 0; name && name[i] && i < BLEH_NAME_MAX; i++) h->name[i] = name[i];
    h->namelen = (uint8_t)i;
    h->acl_total = 4;
}
void bleh_start(bleh_t *h)
{
    h->state = BLEH_STARTING; h->step = ST_RESET; h->cmd_wait = 0; h->want_adv = 0;
    next_cmd(h);
}
void bleh_rx(bleh_t *h, uint8_t t, const uint8_t *p, uint16_t n)
{
    if (t == BLEH_H4_EVT) evt_in(h, p, n);
    else if (t == BLEH_H4_ACL && h->state == BLEH_CONNECTED) acl_in(h, p, n);
}
int bleh_midi_send(bleh_t *h, const uint8_t *m, uint8_t len, uint16_t ts)
{
    bleh_msg_t *q;
    unsigned i;
    if (!len || len > 3 || !(m[0] & 0x80u) || m[0] == 0xF0 || m[0] == 0xF7) return -1;   /* no SysEx, no data-only */
    if ((uint8_t)(h->q_w - h->q_r) >= BLEH_MIDIQ) return -2;
    q = &h->q[h->q_w % BLEH_MIDIQ];
    q->len = len; q->ts = ts & 0x1FFFu;
    for (i = 0; i < len; i++) q->b[i] = m[i];
    h->q_w++;
    return 0;
}
void bleh_poll(bleh_t *h)
{
    uint8_t r[BLEH_MTU];
    unsigned len = 4, hi;
    if (h->state == BLEH_ADVERTISING && h->want_adv && !h->cmd_wait) { h->want_adv = 0; adv_enable(h, 1); }
    if (!bleh_notifying(h) || h->q_r == h->q_w) return;
    if (h->credits < (h->acl_total > 1 ? 2u : 1u)) return;                  /* keep one buffer for ATT answers */
    r[0] = A_NOTIFY; wr16(r + 1, H_MIDI);
    hi = (h->q[h->q_r % BLEH_MIDIQ].ts >> 7) & 0x3Fu;
    r[3] = (uint8_t)(0x80u | hi);                   /* header: timestamp high */
    while (h->q_r != h->q_w) {
        const bleh_msg_t *q = &h->q[h->q_r % BLEH_MIDIQ];
        unsigned i;
        if (((q->ts >> 7) & 0x3Fu) != hi || len + 1 + q->len > BLEH_MTU) break;
        r[len++] = (uint8_t)(0x80u | (q->ts & 0x7Fu));              /* timestamp low, then the message */
        for (i = 0; i < q->len; i++) r[len++] = q->b[i];
        h->q_r++;
    }
    att_send(h, r, len);
}
