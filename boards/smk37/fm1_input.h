/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio)
 * The quadrature decoder with its detent counting and the button debounce are the FM-1 board's
 * (boards/fm1/fm1_input.h): Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* SMK-37 (Pro / Elite / MKE-P37) input HAL: the key, button and encoder matrix behind four 74HC595s, the 19
 * button LEDs, the pads, faders, wheels, pedal and battery behind three analog muxes.
 *
 * The matrix: 16 columns x 8 rows. Four 74HC595s (32 bits) on SPI2 pin group C (PA3 CLK, PA4 DO; IOMAP_CON1 bit
 * 17), latch PA1, /OE PA0 held low. The word: bits 31..24 columns 0..7 and bits 23..16 columns 8..15 (active low,
 * one-hot), bits 15..0 the button LED lines (active low) with bits 5..3 the analog mux address (fm1_adc.h).
 * The rows come back on PA2, PA5, PA6, PA7, PA9, PH6, PH8, PH9 (rows 0..7) with pull-ups, low = closed. A cell's
 * code is column << 4 | row. Rows 0..4 the keybed, 5..6 the buttons, 7 the encoders.
 *   keys      37 (C3..C6), two contacts each: the first (FM1_KEY_C1) closes before the second (FM1_KEY_C2); the
 *             time between them is the velocity, through the key's five-point curve from the factory record
 *             (USRTRIM, FM1_TRIM_KEYS; the stock algorithm). The FM-1's 27 (F3..G5) are keys 5..31: fm1_in.notes
 *   buttons   18 (FM1_BTN): KNOB BANK, FADER BANK, <, >, OCTAVE-, OCTAVE+, ARP, NOTE REPEAT, SCALE, CHORD, GLOBE,
 *             BT, PATCH, PARA, FX, SEQ, SEQ PLAY, SEQ REC
 *   encoders  8 (K1..K8), quadrature, one detent one cycle
 *   LEDs      19 on 16 lines in two commons (group 0 and 1) on the second USB PHY's pads as GPIO (SFR 0x16A0C
 *             bits 17 / 18, low = on): one scan frame shows group 0, the next group 1 (FM1_LEDMAP)
 *   analog    one conversion a tick, the 24 mux inputs in turn (8 addresses x muxes A, B, C): the 16 pads, the
 *             4 faders, the 2 wheels, the pedal jack, the battery (fm1_adc.h)
 *
 *   fm1_input_init();
 *   polled:  for (;;) { fm1_input_scan(); ... }     (the loader: a whole frame, the keys and buttons only)
 *   IRQ:     fm1_input_tick() from the 10 kHz timer ISR: one column per call, 16 ticks a frame (1.6 ms, 625 Hz);
 *            the rows are sampled one tick after the column was latched. The main loop reads fm1_in and takes
 *            edges / steps with fm1_input_edges() / fm1_input_note_edges() / fm1_enc_take(), which are IRQ-safe.
 *
 * Timing: a tick shifts the next column's word (4 bytes by SPI2 DMA, ~3 us), latches it, starts one conversion,
 * then handles the column just read; nothing waits on the converter (taken next tick). The velocity's clock is
 * the tick (100 us = the stock's unit).
 * Keys: once a frame (fm1__keys): a key sounds when its second contact closes, its velocity from the ticks since the
 * first closed; stops when both have been open FM1_KEY_RELEASE frames; struck again before a full lift it restarts.
 * The keys, buttons and scan order follow the reference port (Melodee's SMK-37 build, run on an Elite).
 * Buttons: the FM-1's asymmetric debounce (FM1_DEB_PRESS frames closed, FM1_DEB_RELEASE open).
 * Encoders: the FM-1's decoder (the stock tables 0x2814/0x4182, detents, a two-state jump forgiven).
 * LEDs: fm1_led[g] (bit = line of group g) lit on every column of the group's frame; fm1_led_dim[g] on one column
 * of 16 (~1/16 of lit; DIM LO: one frame in two, ~1/32); fm1_led_mid[g] on two (~1/8); fm1_led_breath[g] on
 * 0..16 columns following a slow curve. No pulse tricks: the 595 lines switch with the column word.
 * Pads: the stock behaviour, written new (fm1-to-smk37 docs/01 §6.3, the reference port's constants): a baseline
 * follows each pad's rest reading; a hit starts FM1_PAD_HIT above it; the peak of the next FM1_PAD_PEAK_N samples
 * through the pad's curve gives the velocity; pressure after, from the reading; released below base + FM1_PAD_REL
 * for two samples.
 * Wheels and pedal: raw readings; the rest level of each is its first reading after init (the device layer
 * scales the pitch wheel around it and takes the pedal as down when the jack's reading leaves its rest level
 * by a margin either way: a normally-open or a normally-closed pedal).
 *
 * Board facts: M. Geisler's analysis of the stock v016 firmware (fm1-to-smk37 docs/01-hardware.md §5-6: the
 * pins, the word layout, the rows, the key, button, encoder and LED tables and the mux tables are marked
 * verified there, on an Elite running Melodee; the 595's SPI mode, /OE and the commons' register sequence are
 * analysis). The SDK's spi.h gives the same SPI2 group. Run by this layer on an Elite (Saadet, 2026-11-05): the
 * key, button, encoder, pad and fader tables work there; the wheels, the pedal, the LED tables and the battery are
 * still to be confirmed on a unit. */
#pragma once
#include <stdint.h>
#include "fm1_time.h"
#include "fm1_gpio.h"
#include "fm1_adc.h"
#include "fm1_lcd_hw.h"                   /* fm1_padled_on: mux B's line is the LED frame's clock pin */
#include "fm1_xip.h"                      /* the factory curves, read through the plain XIP window */

#ifndef FM1_INPUT_IDLE
#define FM1_INPUT_IDLE() ((void)0)
#endif
#ifndef FM1_INPUT_NOW
#define FM1_INPUT_NOW() fm1_ticks()       /* (input tests: simulated) */
#endif
#ifndef FM1_INPUT_TRIM
#define FM1_INPUT_TRIM 1                  /* read the factory curves at init (the loader: 0) */
#endif
#ifndef FM1_SR_LATCH_TRACE
#define FM1_SR_LATCH_TRACE() ((void)0)
#endif
#define FM1_NCOL 16u
#define FM1_NROW 8u
#define FM1_NBTN 18u
#define FM1_NENC 8u
#define FM1_NKEY37 37u
#define FM1_NPAD 16u
#define FM1_NLED 19u
#define FM1_NKEY FM1_NBTN                 /* (the FM-1's id space, 0..13 buttons then keys: buttons only here) */
#define FM1_KEY_FM1_FIRST 5u              /* key 5 = F3: fm1_in.notes bit 0 */
#define FM1_DEB_PRESS 2u                  /* buttons: frames closed in a row (a frame = 16 ticks, 1.6 ms) */
#define FM1_DEB_RELEASE 4u                /* frames open in a row (~6.4 ms) */
#define FM1_KEY_RELEASE 3u                /* keys: frames with both contacts open (~4.8 ms) */
#define FM1_KEY_RETRIG 1u                 /* frames the second contact stays open before closing again is a new strike */
#define FM1_SETTLE_US 10u                 /* the polled scan: the rows after a latch */
#define FM1_REST_FRAMES 600u              /* ~1 s still off the detent state: that is the detent (power-on) */
#define FM1_ENC_GO 14u                    /* a 2-state jump from the detent goes the way of a click this recent */
#define FM1_PAD_BASE0 100u                /* a pad's baseline at init */
#define FM1_PAD_HIT 24u                   /* a hit: this much over the baseline */
#define FM1_PAD_REL 12u                   /* released: under base + this for two samples */
#define FM1_PAD_PEAK_N 3u                 /* the samples whose peak is the velocity */
#define FM1_LED_DIM_COLS 1u               /* a dim LED: lit on this many of the group's 16 columns */
#define FM1_LED_MID_COLS 2u
#define FM1_LED_BREATH_FRAMES 512u        /* a breath: group frames dark -> peak -> dark (~1.6 s; a power of 2) */
#define FM1_LED_BREATH_PK 160u            /* its peak, /256 of lit */
#define FM1_LED_BREATH_PK_LO 80u
#define FM1_TRIM_BASE 0xC1000u            /* USRTRIM (fm1_flash.h FL_USRTRIM: the device checks they agree) */
#define FM1_TRIM_PADS (FM1_TRIM_BASE + 0x008u)
#define FM1_TRIM_KEYS (FM1_TRIM_BASE + 0x0A8u)
#define FM1_TRIM_PEDAL (FM1_TRIM_BASE + 0x21Au)
#define FM1_TRIM_MARK (FM1_TRIM_BASE + 0x224u)

/* ---- the 595 chain on SPI2 and the LED commons (registers: analysis) ---- */
#define FM1_SR_SPI_CON (*(volatile uint32_t *)0x11E00u)
#define FM1_SR_SPI_BAUD (*(volatile uint32_t *)0x11E04u)
#define FM1_SR_SPI_BUF (*(volatile uint32_t *)0x11E08u)
#define FM1_SR_SPI_ADR (*(volatile uint32_t *)0x11E0Cu)
#define FM1_SR_SPI_CNT (*(volatile uint32_t *)0x11E10u)
#define FM1_SR_IOMAP_CON1 (*(volatile uint32_t *)0x51020u)
#define FM1_SR_GROUP_C (1u << 17)         /* SPI2 on PA3 / PA4 (bit 16 clear, bit 17 set) */
#define FM1_SR_BAUD 5u
#define FM1_SR_LATCH (1u << 1)            /* PA1 */
#define FM1_SR_OE (1u << 0)               /* PA0, low = outputs on */
#define FM1_SR_CLK (1u << 3)
#define FM1_SR_DO (1u << 4)
#define FM1_LED_COMMONS (*(volatile uint32_t *)0x16A0Cu)   /* the second USB PHY's pads as GPIO */
#define FM1_LED_COMMON0 (1u << 17)        /* low = group 0 on */
#define FM1_LED_COMMON1 (1u << 18)
#define FM1_LED_COMMONS_KEEP 0x1000Fu     /* its GPIO-mode bits, kept set on every write */
#define FM1_SR_LED_LINES 0xFFC7u          /* the word's low 16 bits that are LED lines (bits 3..5: the mux) */

/* ---- the tables (to be confirmed on a unit) ---- */
/* key k (0 = C3 .. 36 = C6): its first and second contact's cell, column << 4 | row */
static const uint8_t FM1_KEY_C1[FM1_NKEY37] = {
    0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xC0, 0xD0, 0xE0, 0xF0, 0xB0,
    0x11, 0x21, 0x31, 0x41, 0x51, 0x61, 0x71, 0x81, 0x91, 0xA1, 0xB1, 0xC1, 0xD1, 0xE1, 0xF1,
    0x64, 0x74, 0x84, 0x94, 0xA4, 0xB4, 0xC4};
static const uint8_t FM1_KEY_C2[FM1_NKEY37] = {
    0x52, 0x72, 0x92, 0xB2, 0xD2, 0xF2, 0xE2, 0xC2, 0xA2, 0x82, 0x62, 0x42, 0x22, 0x12, 0x32,
    0x33, 0x43, 0x53, 0x63, 0x73, 0x83, 0x93, 0xA3, 0xB3, 0xC3, 0xD3, 0xE3, 0x23, 0x13, 0x14,
    0x24, 0x34, 0x44, 0x54, 0xD4, 0xF4, 0xE4};
/* button id -> its cell */
enum { FM1_BTN_KBANK, FM1_BTN_FBANK, FM1_BTN_LEFT, FM1_BTN_RIGHT, FM1_BTN_OCT_DOWN, FM1_BTN_OCT_UP, FM1_BTN_ARP,
       FM1_BTN_REPEAT, FM1_BTN_SCALE, FM1_BTN_CHORD, FM1_BTN_GLOBE, FM1_BTN_BT, FM1_BTN_PATCH, FM1_BTN_PARA,
       FM1_BTN_FX, FM1_BTN_SEQ, FM1_BTN_PLAY, FM1_BTN_REC };
static const uint8_t FM1_BTN[FM1_NBTN] = {0x26, 0x76, 0x96, 0x56, 0x66, 0x36, 0x75, 0x65, 0x25, 0x45, 0x15, 0x86,
                                          0x95, 0x55, 0x85, 0x35, 0x16, 0x46};
/* encoder K1..K8: A at (col, row), B at (col, row) */
static const uint8_t FM1_ENC[FM1_NENC][4] = {
    {9, 7, 10, 7}, {3, 7, 4, 7}, {1, 7, 2, 7}, {11, 7, 12, 7}, {5, 7, 6, 7}, {7, 7, 8, 7}, {13, 7, 14, 7}, {15, 7, 0, 7},
};
/* LED n -> group << 4 | line bit. 0..17 the buttons' (in FM1_BTN's order); 18 SEQ PLAY's second colour, red
 * (0x1E; SEQ REC's own line is 0x1B: Melodee's table, confirmed on a unit) */
static const uint8_t FM1_LEDMAP[FM1_NLED] = {0x0C, 0x12, 0x00, 0x10, 0x0D, 0x1D, 0x02, 0x01, 0x09, 0x08, 0x0B, 0x0A,
                                             0x11, 0x19, 0x18, 0x1A, 0x1C, 0x1B, 0x1E};
#define FM1_LED_PLAY2 18u
/* the stock default curves (USRTRIM's, when the record is missing or bad) */
static const uint16_t FM1_CURVE_KEY_DEFAULT[5] = {5, 640, 800, 920, 980};
static const uint16_t FM1_CURVE_PAD_DEFAULT[5] = {50, 550, 720, 850, 950};

static volatile struct {
    uint32_t notes;                       /* debounced: bit n = note key n (0 = F3 .. 26 = G5), keys 5..31 */
    uint32_t buttons;                     /* debounced: bit i = button i (0..17) */
    uint32_t pressed, released;           /* button edges since the last fm1_input_edges() */
    uint32_t notes_pressed;               /* note-key press edges since the last fm1_input_note_edges() */
    uint32_t keys37_lo, keys37_hi;        /* bit k = key k held (0 = C3 .. 36 = C6): keys 0..31, then 32..36 */
    uint32_t keys37_pressed_lo, keys37_pressed_hi;   /* press edges since the last fm1_input_key37_edges() */
    uint32_t keys37_retrig_lo, keys37_retrig_hi;     /* struck again while down (their bit stays set) */
    uint8_t key_vel[FM1_NKEY37];          /* the last press's velocity, 1..127 */
    uint32_t pads;                        /* bit p = pad p down (0..7 the top row, 8..15 the bottom) */
    uint32_t pads_pressed;                /* press edges since the last fm1_input_pad_edges() */
    uint8_t pad_vel[FM1_NPAD];            /* the last hit's velocity, 1..127 */
    uint8_t pad_press[FM1_NPAD];          /* pressure while down, 0..127 */
    uint8_t fader[4];                     /* 0..127: a median of three, smoothed, a dead band of one */
    uint16_t fad_acc[4], fad_hist[4][2];  /* .. x16 smoothed; the two readings before */
    uint16_t wheel[2];                    /* raw 0..1023, smoothed: 0 pitch, 1 mod (the device layer scales them) */
    uint16_t pedal_raw;                   /* the jack's reading (the device layer learns its rest) */
    uint16_t battery;                     /* raw 0..1023 (0: none yet) */
    uint8_t raw[FM1_NCOL];                /* last frame, packed rows (bit r = row r), 1 = closed */
    uint8_t cnt[FM1_NBTN];                /* frames in a row against the debounced state */
    uint8_t kst[FM1_NKEY37], kcnt[FM1_NKEY37];   /* a key's contacts settled (0, 1, 3); frames with both open */
    uint8_t kopen[FM1_NKEY37];            /* frames the second has been open with the note on (saturates) */
    uint32_t kt0[FM1_NKEY37];             /* the tick the first closed, or the second opened with the note on */
    uint8_t enc_prev[FM1_NENC], enc_last[FM1_NENC];
    uint8_t enc_rest[FM1_NENC];           /* the detent state (0..3) */
    uint16_t enc_still[FM1_NENC];         /* frames since the last state change */
    int8_t enc_sub[FM1_NENC];             /* net transitions since the last rest state */
    int16_t enc_steps[FM1_NENC];          /* + = clockwise */
    int8_t enc_dir[FM1_NENC];             /* the last click's direction, and its frame */
    uint32_t enc_dfr[FM1_NENC];
    uint16_t pbase[FM1_NPAD], ppeak[FM1_NPAD];   /* a pad: its baseline, the hit's peak so far */
    uint8_t pst[FM1_NPAD], pn[FM1_NPAD], prel[FM1_NPAD];   /* 0 idle, 1 peaking, 2 down; samples; releases */
    uint32_t frames, ticks;
    uint8_t trim_ok;                      /* the factory record's marker was there */
} fm1_in;
static uint16_t fm1_led[2];               /* lit: bit = line of group g (FM1_LEDMAP) */
static uint16_t fm1_led_dim[2];           /* the glow */
static uint16_t fm1_led_mid[2];           /* a steady mid level */
static uint16_t fm1_led_breath[2];        /* breathing, in step */
static uint16_t fm1_curve_key[FM1_NKEY37][5], fm1_curve_pad[FM1_NPAD][5];
static uint8_t fm1_vel_mode;               /* keys and pads: 0 the curves as they are, 1 softer, 2 harder, 3 fixed at 100 */
static volatile struct {
    uint16_t con;                         /* the conversion's clock / start delay (FM1_ADC_CON); 0 = analog off */
    uint8_t pend;                         /* a conversion runs for (mux, addr) */
    uint8_t mux, addr, slot;              /* this conversion's; the next slot (0..23: addr * 3 + mux) */
    uint8_t padled;                       /* the LED frame was on the wire when it started */
    uint16_t timeouts;                    /* conversions not done after a tick (the clock too slow) */
} fm1_an;
static uint8_t fm1__tick_col, fm1__led_grp, fm1__dim_div = 1u, fm1__br_pk = FM1_LED_BREATH_PK;
static uint16_t fm1__br_ph, fm1__br_acc, fm1__br_cols;   /* the breath: its frame, the dither, this frame's columns */
static uint32_t fm1__sr_word_ram;         /* the word the SPI2 DMA reads (RAM) */

/* ---- the chain ---- */
static void fm1__sr_init(void)
{
    uint32_t pa = FM1_SR_LATCH | FM1_SR_OE | FM1_SR_CLK | FM1_SR_DO;
    FM1_SR_IOMAP_CON1 = (FM1_SR_IOMAP_CON1 & ~(3u << 16)) | FM1_SR_GROUP_C;
    FM1_PR(FM1_PA, FM1_DIE) |= pa;
    FM1_PR(FM1_PA, FM1_PU) &= ~pa;
    FM1_PR(FM1_PA, FM1_PD) &= ~pa;
    FM1_PR(FM1_PA, FM1_OUT) &= ~pa;       /* /OE low: outputs on; latch low */
    FM1_PR(FM1_PA, FM1_DIR) &= ~pa;
    FM1_SR_SPI_CON = 0x4020u;             /* master, transmit (analysis) */
    FM1_SR_SPI_BAUD = FM1_SR_BAUD;
    FM1_SR_SPI_CON |= 1u;
}
static void fm1__sr_wait(void)
{
    uint32_t n;
    for (n = 0; n < 200000u && !(FM1_SR_SPI_CON & 0x8000u); n++)
        ;
    FM1_SR_SPI_CON |= 0x4000u;
}
static void fm1__sr_latch(void)
{
    FM1_SR_LATCH_TRACE();
    FM1_PR(FM1_PA, FM1_OUT) |= FM1_SR_LATCH;
    FM1_PR(FM1_PA, FM1_OUT) &= ~FM1_SR_LATCH;
}
/* the 32-bit word into the chain (4 bytes by DMA, little-endian in memory as the stock sends it); shifted only:
 * the 595 outputs change at the latch, which the next tick gives it */
static void fm1__sr_shift(uint32_t w)
{
    fm1__sr_word_ram = w;
    FM1_SR_SPI_CON |= 0x4000u;
    FM1_SR_SPI_ADR = (uint32_t)(uintptr_t)&fm1__sr_word_ram;
    FM1_SR_SPI_CNT = 4u;
}
static void fm1__sr_word(uint32_t w)                  /* shifted and latched at once (init, the polled scan) */
{
    fm1__sr_shift(w);
    fm1__sr_wait();
    fm1__sr_latch();
}
static inline uint32_t fm1__col_bits(uint32_t c)   /* the column select: one bit low */
{
    return 0xFFFF0000u ^ (c < 8u ? 0x01000000u << c : 0x00010000u << (c - 8u));
}
static uint32_t fm1__rows(void)
{
    uint32_t a = FM1_PR(FM1_PA, FM1_IN), h = FM1_PR(FM1_PH, FM1_IN);
    uint32_t r = ((a >> 2) & 1u) | ((a >> 5) & 1u) << 1 | ((a >> 6) & 1u) << 2 | ((a >> 7) & 1u) << 3 |
                 ((a >> 9) & 1u) << 4 | ((h >> 6) & 1u) << 5 | ((h >> 8) & 1u) << 6 | ((h >> 9) & 1u) << 7;
    return (~r) & 0xFFu;
}
static void fm1__wait(uint32_t us)
{
    uint32_t t0 = fm1_ticks(), span = us * FM1_TICKS_PER_US;
    while ((uint32_t)(fm1_ticks() - t0) < span)
        FM1_INPUT_IDLE();
}
static void fm1__commons(uint32_t g)      /* group g's common on (both off: g > 1) */
{
    uint32_t v = (FM1_LED_COMMONS | FM1_LED_COMMON0 | FM1_LED_COMMON1) & ~(g == 0u ? FM1_LED_COMMON0 : g == 1u ? FM1_LED_COMMON1 : 0u);
    FM1_LED_COMMONS = v | FM1_LED_COMMONS_KEEP;
}

/* ---- the factory curves ---- */
static int fm1__curve_ok(const uint16_t *t)
{
    uint32_t i;
    for (i = 1; i < 5u; i++)
        if (t[i] <= t[i - 1] || t[i] > 1023u)
            return 0;
    return t[0] < 1024u;
}
static void fm1__trim_load(void)
{
    uint32_t i, j;
    const uint8_t *m = fm1_xip_ptr(FM1_TRIM_MARK);
    fm1_in.trim_ok = FM1_INPUT_TRIM && m[0] == 't' && m[1] == 'r' && m[2] == 'i' && m[3] == 'm' && m[4] == 'd' &&
                     m[5] == 'a' && m[6] == 't' && m[7] == 0u;
    for (i = 0; i < FM1_NKEY37; i++) {
        const uint8_t *p = fm1_xip_ptr(FM1_TRIM_KEYS + i * 10u);
        for (j = 0; j < 5u; j++)
            fm1_curve_key[i][j] = fm1_in.trim_ok ? (uint16_t)(p[2u * j] | p[2u * j + 1u] << 8) : FM1_CURVE_KEY_DEFAULT[j];
        if (!fm1__curve_ok(fm1_curve_key[i]))
            for (j = 0; j < 5u; j++)
                fm1_curve_key[i][j] = FM1_CURVE_KEY_DEFAULT[j];
    }
    for (i = 0; i < FM1_NPAD; i++) {
        const uint8_t *p = fm1_xip_ptr(FM1_TRIM_PADS + i * 10u);
        for (j = 0; j < 5u; j++)
            fm1_curve_pad[i][j] = fm1_in.trim_ok ? (uint16_t)(p[2u * j] | p[2u * j + 1u] << 8) : FM1_CURVE_PAD_DEFAULT[j];
        if (!fm1__curve_ok(fm1_curve_pad[i]))
            for (j = 0; j < 5u; j++)
                fm1_curve_pad[i][j] = FM1_CURVE_PAD_DEFAULT[j];
    }
}
/* the stock mapping: x (1..1023) through a five-point curve t -> a MIDI velocity 1..127 */
static uint32_t fm1__curve(uint32_t x, const uint16_t *t)
{
    uint32_t i, v;
    if (x <= t[0])
        v = 1u;
    else if (x > t[4])
        v = 1023u;
    else {
        for (i = 1; x > t[i]; i++)
            ;
        v = (i - 1u) * 256u + (x - t[i - 1u]) * 256u / (uint32_t)(t[i] - t[i - 1u]);
    if (v > 1023u)
        v = 1023u;
    }
    if (fm1_vel_mode == 3u)
        return 100u;
    if (fm1_vel_mode == 1u)
        v = 1023u - (1023u - v) * (1023u - v) / 1023u;
    else if (fm1_vel_mode == 2u)
        v = v * v / 1023u;
    v >>= 3;
    return v ? v : 1u;
}

/* ---- init ---- */
static void fm1_input_init(void)
{
    const uint32_t rows_a = (1u << 2) | (1u << 5) | (1u << 6) | (1u << 7) | (1u << 9);
    const uint32_t rows_h = (1u << 6) | (1u << 8) | (1u << 9);
    uint32_t i;
    FM1_PR(FM1_PA, FM1_DIE) |= rows_a;
    FM1_PR(FM1_PA, FM1_DIR) |= rows_a;
    FM1_PR(FM1_PA, FM1_PD) &= ~rows_a;
    FM1_PR(FM1_PA, FM1_PU) |= rows_a;
    FM1_PR(FM1_PH, FM1_DIE) |= rows_h;
    FM1_PR(FM1_PH, FM1_DIR) |= rows_h;
    FM1_PR(FM1_PH, FM1_PD) &= ~rows_h;
    FM1_PR(FM1_PH, FM1_PU) |= rows_h;
    fm1__sr_init();
    fm1__commons(2u);                     /* both groups off */
    fm1__sr_word(0xFFFFFFFFu);            /* no column, no LED */
    for (i = 0; i < FM1_NENC; i++)
        fm1_in.enc_prev[i] = fm1_in.enc_last[i] = 0xFF;   /* seeded by the first frame */
    for (i = 0; i < FM1_NPAD; i++)
        fm1_in.pbase[i] = FM1_PAD_BASE0;
    fm1__trim_load();
}

/* ---- keys and buttons ---- */
static void fm1__btn(uint32_t id, uint32_t closed)   /* the FM-1's debounce */
{
    volatile uint8_t *c = &fm1_in.cnt[id];
    uint32_t bit = 1u << id, on = (fm1_in.buttons & bit) != 0u;
    if (closed == on) {
        *c = 0;
        return;
    }
    if (++*c < (on ? FM1_DEB_RELEASE : FM1_DEB_PRESS))
        return;
    *c = 0;
    fm1_in.buttons ^= bit;
    if (!on)
        fm1_in.pressed |= bit;
    else
        fm1_in.released |= bit;
}
static inline int fm1__key_held(uint32_t k)
{
    return k < 32u ? (fm1_in.keys37_lo >> k) & 1u : (fm1_in.keys37_hi >> (k - 32u)) & 1u;
}
static void fm1__key_on(uint32_t k, uint32_t vel)
{
    fm1_in.key_vel[k] = (uint8_t)vel;
    if (k < 32u) {
        fm1_in.keys37_lo |= 1u << k;
        fm1_in.keys37_pressed_lo |= 1u << k;
    } else {
        fm1_in.keys37_hi |= 1u << (k - 32u);
        fm1_in.keys37_pressed_hi |= 1u << (k - 32u);
    }
    if (k >= FM1_KEY_FM1_FIRST && k < FM1_KEY_FM1_FIRST + 27u) {
        fm1_in.notes |= 1u << (k - FM1_KEY_FM1_FIRST);
        fm1_in.notes_pressed |= 1u << (k - FM1_KEY_FM1_FIRST);
    }
}
static void fm1__key_off(uint32_t k)
{
    if (k < 32u)
        fm1_in.keys37_lo &= ~(1u << k);
    else
        fm1_in.keys37_hi &= ~(1u << (k - 32u));
    if (k >= FM1_KEY_FM1_FIRST && k < FM1_KEY_FM1_FIRST + 27u)
        fm1_in.notes &= ~(1u << (k - FM1_KEY_FM1_FIRST));
}
/* a note starts, its velocity from the ticks since kt0 (the stock's 0.1 ms units, from 10); again: the key was struck
 * again before a full lift, so its bit stays set and the app restarts the note (fm1_input_key37_retrig) */
static void fm1__key_strike(uint32_t k, uint32_t again)
{
    uint32_t t = 10u + (fm1_in.ticks - fm1_in.kt0[k]), x = t <= 0x3FEu ? 0x3FFu - t : 1u;
    if (again) {
        if (k < 32u)
            fm1_in.keys37_retrig_lo |= 1u << k;
        else
            fm1_in.keys37_retrig_hi |= 1u << (k - 32u);
    }
    fm1__key_on(k, fm1__curve(x, fm1_curve_key[k]));
}
/* The keys, once a frame, from a whole frame's rows. The two contacts settle as 0 open, 1 the first closed, 3 both; the
 * second without the first counts as both, so a dirty first contact neither loses a note nor cuts one. A note ends
 * when both are open for FM1_KEY_RELEASE frames. Second contact opening and closing again while the first holds is a
 * new strike, timed from where it opened. */
static void fm1__keys(void)
{
    uint32_t k;
    for (k = 0; k < FM1_NKEY37; k++) {
        uint32_t c1 = FM1_KEY_C1[k], c2 = FM1_KEY_C2[k], st = fm1_in.kst[k];
        uint32_t s = ((fm1_in.raw[c1 >> 4] >> (c1 & 15u)) & 1u) | ((fm1_in.raw[c2 >> 4] >> (c2 & 15u)) & 1u) << 1;
        uint32_t on = (uint32_t)fm1__key_held(k);
        if (s == 2u)
            s = 3u;
        if (s == 0u) {
            if (st == 0u) {
                fm1_in.kcnt[k] = 0;
            } else if (++fm1_in.kcnt[k] >= FM1_KEY_RELEASE) {
                fm1_in.kcnt[k] = 0;
                fm1_in.kst[k] = 0;
                if (on)
                    fm1__key_off(k);
            }
            continue;
        }
        fm1_in.kcnt[k] = 0;
        if (s == st) {
            if (on && st == 1u && fm1_in.kopen[k] != 0xFFu)
                fm1_in.kopen[k]++;
            continue;
        }
        if (st == 0u)
            fm1_in.kt0[k] = fm1_in.ticks;
        if (s == 3u) {
            if (!on)
                fm1__key_strike(k, 0);
            else if (st == 1u && fm1_in.kopen[k] >= FM1_KEY_RETRIG)
                fm1__key_strike(k, 1);
        } else if (st == 3u && on) {
            fm1_in.kt0[k] = fm1_in.ticks;
            fm1_in.kopen[k] = 0;
        }
        fm1_in.kst[k] = (uint8_t)s;
    }
    for (k = 0; k < FM1_NBTN; k++)
        fm1__btn(k, (fm1_in.raw[FM1_BTN[k] >> 4] >> (FM1_BTN[k] & 15u)) & 1u);
}

/* ---- once a frame: the encoders (the FM-1's decoder), the breath ---- */
static void fm1__frame(void)
{
    uint32_t e;
    fm1__keys();
    for (e = 0; e < FM1_NENC; e++) {
        const uint8_t *m = FM1_ENC[e];
        uint32_t cur = ((fm1_in.raw[m[0]] >> m[1]) & 1u) << 1 | ((fm1_in.raw[m[2]] >> m[3]) & 1u);
        uint32_t idx;
        volatile int8_t *sub = &fm1_in.enc_sub[e];
        if (cur != fm1_in.enc_last[e]) {
            fm1_in.enc_last[e] = (uint8_t)cur;
            fm1_in.enc_still[e] = 0;
        }
        if (fm1_in.enc_prev[e] == 0xFF) {            /* first frame: the knob rests here */
            fm1_in.enc_prev[e] = (uint8_t)cur;
            fm1_in.enc_rest[e] = (uint8_t)cur;
        }
        if (fm1_in.enc_still[e] < 0xFFFFu && ++fm1_in.enc_still[e] == FM1_REST_FRAMES &&
            cur != fm1_in.enc_rest[e]) {             /* parked a long time off the detent state: that is it */
            fm1_in.enc_rest[e] = (uint8_t)cur;
            *sub = 0;
        }
        if (cur == fm1_in.enc_prev[e])
            continue;
        idx = (uint32_t)fm1_in.enc_prev[e] << 2 | cur;
        if ((0x4182u >> idx) & 1u)
            (*sub)++;
        else if ((0x2814u >> idx) & 1u)
            (*sub)--;
        else {                                       /* two states in one sample: a fast turn, the way it was going */
            int32_t d = *sub > 0 ? 1 : *sub < 0 ? -1 :
                        (uint32_t)(fm1_in.frames - fm1_in.enc_dfr[e]) <= FM1_ENC_GO ? fm1_in.enc_dir[e] : 0;
            *sub = (int8_t)(*sub + 2 * d);
        }
        fm1_in.enc_prev[e] = (uint8_t)cur;
        if (*sub > 100 || *sub < -100)
            *sub = 0;
        if (cur == fm1_in.enc_rest[e]) {            /* back on the detent: whole cycles */
            int32_t n = *sub < 0 ? -*sub : *sub;
            n = n >= 2 ? (n + 2) / 4 : 0;
            fm1_in.enc_steps[e] = (int16_t)(fm1_in.enc_steps[e] + (*sub < 0 ? -n : n));
            if (n) {
                fm1_in.enc_dir[e] = (int8_t)(*sub < 0 ? -1 : 1);
                fm1_in.enc_dfr[e] = fm1_in.frames;
            }
            *sub = 0;
        }
    }
    fm1_in.frames++;
}
static void fm1__breath_frame(void)                  /* this group frame's lit columns of a breathing LED */
{
    uint32_t x, any = fm1_led_breath[0] | fm1_led_breath[1];
    if (!any) {
        fm1__br_ph = FM1_LED_BREATH_FRAMES / 2u;     /* (a breath starting from none starts at its peak) */
        fm1__br_cols = 0;
        return;
    }
    x = fm1__br_ph = (uint16_t)((fm1__br_ph + 1u) & (FM1_LED_BREATH_FRAMES - 1u));
    x = x * 512u / FM1_LED_BREATH_FRAMES;            /* 0..511 */
    x = x & 256u ? 511u - x : x;                     /* a triangle 0..255 */
    x = x * x * (768u - 2u * x) >> 16;               /* smoothstep, 0..255 */
    x = x * x * fm1__br_pk >> 8;                     /* squared (the eye), x the peak: /65536 of lit */
    x = fm1__br_acc + (x >> 4);                      /* columns of 16, with a 1/256 column dither */
    fm1__br_cols = (uint16_t)(x >> 8);
    fm1__br_acc = (uint16_t)(x & 0xFFu);
}
/* the LED lines (active low) of group g on column c of its frame, with the mux address in bits 3..5 */
static uint32_t fm1__led_low16(uint32_t g, uint32_t c, uint32_t addr)
{
    uint32_t lit = fm1_led[g];
    if (c < FM1_LED_MID_COLS)
        lit |= fm1_led_mid[g];
    if (c < FM1_LED_DIM_COLS && (fm1_in.frames >> 1) % fm1__dim_div == 0u)
        lit |= fm1_led_dim[g];
    if (c < fm1__br_cols)
        lit |= fm1_led_breath[g];
    return ((~lit) & FM1_SR_LED_LINES) | (addr & 7u) << 3;
}

/* ---- the analog inputs ---- */
static void fm1__pad_sample(uint32_t p, uint32_t v)
{
    uint32_t bit = 1u << p;
    switch (fm1_in.pst[p]) {
    case 0:
        if (v > fm1_in.pbase[p] + FM1_PAD_HIT) {
            fm1_in.pst[p] = 1;
            fm1_in.ppeak[p] = (uint16_t)v;
            fm1_in.pn[p] = 1;
            return;
        }
        if (v < fm1_in.pbase[p])
            fm1_in.pbase[p]--;                        /* the baseline follows the rest reading down */
        else if (fm1_in.pbase[p] < fm1_curve_pad[p][0])
            fm1_in.pbase[p] = (uint16_t)(fm1_in.pbase[p] + 2u);   /* and up, only as far as the curve's first point */
        return;
    case 1:
        if (v > fm1_in.ppeak[p])
            fm1_in.ppeak[p] = (uint16_t)v;
        if (++fm1_in.pn[p] < FM1_PAD_PEAK_N)
            return;
        fm1_in.pad_vel[p] = (uint8_t)fm1__curve(fm1_in.ppeak[p], fm1_curve_pad[p]);
        fm1_in.pst[p] = 2;
        fm1_in.prel[p] = 0;
        fm1_in.pads |= bit;
        fm1_in.pads_pressed |= bit;
        /* fall through: the pressure from this reading */
    default:
        fm1_in.pad_press[p] = (uint8_t)(v <= 512u ? v >> 4 : (v - 512u) / 4u + 32u > 127u ? 127u : (v - 512u) / 4u + 32u);
        if (v < fm1_in.pbase[p] + FM1_PAD_REL) {
            if (++fm1_in.prel[p] >= 2u) {
                fm1_in.pst[p] = 0;
                fm1_in.pads &= ~bit;
                fm1_in.pad_press[p] = 0;
            }
        } else {
            fm1_in.prel[p] = 0;
        }
        return;
    }
}
static void fm1__an_result(uint32_t mux, uint32_t addr, uint32_t v)
{
    if (mux == 0u) {                                 /* A: the top row of pads */
        fm1__pad_sample(FM1_ADC_PAD_OF[addr], v);
    } else if (mux == 1u) {                          /* B: the bottom row (its line is the LED frame's clock pin) */
        if (!fm1_an.padled && !fm1_padled_on)
            fm1__pad_sample(8u + FM1_ADC_PAD_OF[addr], v);
    } else if (addr < 4u) {                          /* C: the faders: a lone spike dropped, smoothed, 0..127 */
        uint32_t i = FM1_ADC_FADER_OF[addr], h0 = fm1_in.fad_hist[i][0], h1 = fm1_in.fad_hist[i][1], f = fm1_in.fader[i], m, n;
        int32_t acc = fm1_in.fad_acc[i];
        fm1_in.fad_hist[i][1] = (uint16_t)h0;
        fm1_in.fad_hist[i][0] = (uint16_t)v;
        m = v > h0 ? (h0 > h1 ? h0 : v > h1 ? h1 : v) : (v > h1 ? v : h0 > h1 ? h1 : h0);
        acc += ((int32_t)m * 16 - acc) / 4;
        fm1_in.fad_acc[i] = (uint16_t)acc;
        n = (uint32_t)acc / 128u;
        if (n > 127u)
            n = 127u;
        if (n > f + 1u || n + 1u < f || (n != f && (n == 0u || n == 127u)))
            fm1_in.fader[i] = (uint8_t)n;
    } else if (addr == FM1_ADC_C_PITCH || addr == FM1_ADC_C_MOD) {
        uint32_t w = addr - FM1_ADC_C_PITCH;
        fm1_in.wheel[w] = (uint16_t)((int32_t)fm1_in.wheel[w] + ((int32_t)v - (int32_t)fm1_in.wheel[w]) / 4);
    } else if (addr == FM1_ADC_C_PEDAL) {
        fm1_in.pedal_raw = (uint16_t)v;
    } else {
        fm1_in.battery = (uint16_t)(v ? v : 1u);
    }
}
static void fm1__an_take(void)                       /* the conversion started on the previous tick */
{
    fm1_an.pend = 0;
    if (fm1_adc_done())
        fm1__an_result(fm1_an.mux, fm1_an.addr, fm1_adc_take());
    else {
        fm1_adc_abort();
        fm1_an.timeouts++;
    }
}
static void fm1__an_start(void)                      /* the next slot's conversion, the mux address just latched */
{
    static const uint8_t CH[3] = {FM1_ADC_MUX_A, FM1_ADC_MUX_B, FM1_ADC_MUX_C};
    fm1_an.padled = fm1_padled_on;
    fm1_adc_start(CH[fm1_an.mux], fm1_an.con);
    fm1_an.pend = 1;
}
static inline uint32_t fm1__an_addr_after(void)      /* the mux address of the slot after this one (the next word's) */
{
    uint32_t s = fm1_an.slot + 1u >= 24u ? 0u : fm1_an.slot + 1u;
    return s / 3u;
}
static inline void fm1__an_next(void)                /* the slot after this one: 24 of them, address first */
{
    uint32_t s = fm1_an.slot + 1u >= 24u ? 0u : fm1_an.slot + 1u;
    fm1_an.slot = (uint8_t)s;
    fm1_an.mux = (uint8_t)(s % 3u);
    fm1_an.addr = (uint8_t)(s / 3u);
}

/* ---- the scans ---- */
/* The stock order, which the cell codes above assume (checked against the reference HAL, 2026-11-04): a tick reads
 * the rows first, files them under the index of the word shifted last tick (whose latch comes next), then latches that
 * word and shifts the next one. So raw[c] holds the rows seen while the chain's column c - 1 was selected: the codes'
 * columns run one ahead of the 595 bit, and only this order makes them land on the right keys. */
static void fm1_input_scan(void)                     /* one polled frame: keys and buttons (the loader) */
{
    uint32_t p;
    fm1__sr_shift(fm1__col_bits(0) | 0xFFFFu);
    fm1__sr_wait();
    for (p = 0; p < FM1_NCOL; p++) {
        uint32_t n = p + 1u == FM1_NCOL ? 0u : p + 1u;
        fm1_in.raw[p] = (uint8_t)fm1__rows();
        fm1__sr_latch();
        fm1__sr_shift(fm1__col_bits(n) | 0xFFFFu);
        fm1__sr_wait();
        fm1__wait(FM1_SETTLE_US);
        fm1_in.ticks++;
    }
    fm1__frame();
}
/* one column per call, from the timer ISR (see top) */
static void fm1_input_tick(void)
{
    uint32_t p = fm1__tick_col, n = p + 1u == FM1_NCOL ? 0u : p + 1u, g = fm1__led_grp;
    fm1_in.ticks++;
    fm1_in.raw[p] = (uint8_t)fm1__rows();          /* the rows under word p's index, before its latch */
    if (fm1_an.pend)
        fm1__an_take();
    if (p == 0u) {                                   /* a new frame: the other group's common, the breath */
        g = fm1__led_grp = (uint8_t)(g ^ 1u);
        fm1__commons(g);
        if (g == 0u)
            fm1__breath_frame();
    }
    fm1__sr_latch();                                 /* word p, shifted last tick with the next slot's mux address */
    if (fm1_an.con) {
        fm1__an_next();                              /* that slot: its conversion starts at the address just latched */
        fm1__an_start();
    }
    /* (word 0 is latched just after the next frame's common changes: it carries that group's lines) */
    fm1__sr_shift(fm1__col_bits(n) | fm1__led_low16(n == 0u ? g ^ 1u : g, n, fm1__an_addr_after()));
    fm1__tick_col = (uint8_t)n;
    if (n == 0u)
        fm1__frame();
}

/* main-loop critical section against fm1_input_tick (main loop only: it re-enables interrupts unconditionally) */
static inline uint32_t fm1__lock(void)
{
    __asm__ volatile("cli" ::: "memory");
    return 0;
}
static inline void fm1__unlock(uint32_t v)
{
    (void)v;
    __asm__ volatile("csync\n\tsti" ::: "memory");
}
/* detent steps turned since the last call, + = clockwise */
static int32_t fm1_enc_take(uint32_t e)
{
    uint32_t k = fm1__lock();
    int32_t s = fm1_in.enc_steps[e];
    fm1_in.enc_steps[e] = 0;
    fm1__unlock(k);
    return s;
}
static uint32_t fm1_input_edges(uint32_t *released)
{
    uint32_t k = fm1__lock();
    uint32_t p = fm1_in.pressed;
    if (released)
        *released = fm1_in.released;
    fm1_in.pressed = fm1_in.released = 0;
    fm1__unlock(k);
    return p;
}
static uint32_t fm1_input_note_edges(void)
{
    uint32_t k = fm1__lock();
    uint32_t p = fm1_in.notes_pressed;
    fm1_in.notes_pressed = 0;
    fm1__unlock(k);
    return p;
}
static void fm1_input_key37_retrig(uint32_t *lo, uint32_t *hi)  /* keys struck again while down, since the last call */
{
    uint32_t k = fm1__lock();
    *lo = fm1_in.keys37_retrig_lo;
    *hi = fm1_in.keys37_retrig_hi;
    fm1_in.keys37_retrig_lo = fm1_in.keys37_retrig_hi = 0;
    fm1__unlock(k);
}
static void fm1_input_key37_edges(uint32_t *lo, uint32_t *hi)   /* the 37 keys' press edges */
{
    uint32_t k = fm1__lock();
    *lo = fm1_in.keys37_pressed_lo;
    *hi = fm1_in.keys37_pressed_hi;
    fm1_in.keys37_pressed_lo = fm1_in.keys37_pressed_hi = 0;
    fm1__unlock(k);
}
static uint32_t fm1_input_pad_edges(void)
{
    uint32_t k = fm1__lock();
    uint32_t p = fm1_in.pads_pressed;
    fm1_in.pads_pressed = 0;
    fm1__unlock(k);
    return p;
}

/* ---- LEDs ---- */
static void fm1_led_set(uint16_t *grp, uint32_t led, int on)   /* LED led (0..18) in one of the four pictures */
{
    uint32_t m = FM1_LEDMAP[led];
    if (on)
        grp[m >> 4] |= (uint16_t)(1u << (m & 15u));
    else
        grp[m >> 4] &= (uint16_t)~(1u << (m & 15u));
}
static void fm1_led_key(uint32_t id, int on)        /* (the FM-1's name: button id's LED) */
{
    if (id < FM1_NBTN)
        fm1_led_set(fm1_led, id, on);
}
/* the glow (main loop, any time): 0 DIM HI (one column of 16, every group frame), 1 DIM LO (every other frame);
 * and the breath's peak with it */
static void fm1_led_dim_level(uint32_t lo)
{
    fm1__dim_div = (uint8_t)(lo ? 2u : 1u);
    fm1__br_pk = (uint8_t)(lo ? FM1_LED_BREATH_PK_LO : FM1_LED_BREATH_PK);
}
/* the power-on sweep and the idle animation of the FM-1 board (fm1_led_anim.h): none on the SMK yet; the UI
 * writes the LEDs from the start */
static void fm1_led_anim_start(uint32_t glow) { (void)glow; }
static int fm1_led_anim_on(void) { return 0; }
static void fm1_led_idle_start(uint32_t glow) { (void)glow; }
static int fm1_led_idle_on(void) { return 0; }
static void fm1_led_anim_stop(void) {}
