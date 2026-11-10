/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Charles Vestal */
/* fm1_input_tick (boards/fm1/fm1_input.h) at its rate FM1_INPUT_HZ (10000, 5000, 2000, 1000: build with
 * -DFM1_INPUT_HZ=N), on the host: the GPIO ports an array (each access 2 TIMER4 ticks, about the board's bus), the
 * clock simulated, the ticks FM1_TICK_US apart, each column's rows set from a model of the keys and knobs at the
 * moment the tick reads them. Checked, at every rate:
 *   - at 10000 every constant derived from the rate is what it was before the rate could change;
 *   - a clean press counts within FM1_DEB_PRESS frames (and a tick) of the contact, a release within FM1_DEB_RELEASE;
 *   - bouncy presses (closed / open runs of 30..700 us for up to 3 ms on the way down, 5 ms on the way up) of every
 *     note key and button: one press and one release each, none early, none late;
 *   - a stray closed sample (150 us) plays nothing where the press takes two frames;
 *   - fast repeats (40 ms on, 40 off) are all heard;
 *   - the encoders: 20 clicks each way at a normal turn (10 clicks/s) and a fast one (80 % of one frame a state),
 *     a flick (1.6 states a frame, once moving), all counted; the fastest even turn that keeps every step printed;
 *   - the LEDs: a lit LED lit about one tick every frame, the glow every frame at FM1_LED_DIM_NS (+-30 %), the mid
 *     level lit 1 frame in FM1_LED_MID_N, the breath's period (~1.1 s) and the power-on sweep's length (~0.7 s) in
 *     time, the refresh of each printed.
 * Not the look of the LEDs (only an FM-1 shows that), nor the analog rows. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "fm1_time.h"
#include "fm1_gpio.h"

static uint32_t sim_now;                           /* TIMER4 ticks, 24 MHz */
static uint32_t sim_reg[8][16];                    /* the GPIO ports' registers */
static uint32_t sim_ticks(void) { return ++sim_now; }
#define fm1_ticks sim_ticks
#undef FM1_PR
#define FM1_PR(p, r) (*(sim_now += 2u, &sim_reg[(p) & 7u][((r) / 4u) & 15u]))
static void led_trace(uint32_t rowmask);
static void latch_trace(void);
#define FM1_LED_TRACE(rowmask) led_trace(rowmask)
#define FM1_SR_LATCH_TRACE() latch_trace()
#include "fm1_input.h"

#define T4_TICK (FM1_TICK_US * FM1_TICKS_PER_US)    /* a tick, TIMER4 ticks */
#define T4_FRAME (FM1_NCOL * T4_TICK)
#define MS(t) ((double)(t) / (1000.0 * FM1_TICKS_PER_US))

static int fails;
static void want(const char *what, int ok)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    fails += !ok;
}

/* ------------------------------------------------------------------ the keys and knobs: a model in time --- */
#define NB 40
typedef struct {
    int on;                                        /* contact model in use */
    uint32_t t[NB + 2];                            /* times at which the contact flips, starting open */
    int n;
} contact_t;
static contact_t key_c[FM1_NKEY];                  /* by key id */
static int key_closed(uint32_t id, uint32_t t)
{
    const contact_t *c = &key_c[id];
    int i, s = 0;
    if (!c->on)
        return 0;
    for (i = 0; i < c->n && (int32_t)(t - c->t[i]) >= 0; i++)
        s ^= 1;
    return s;
}
/* encoder 0's state over time: transitions at enc_t[i], to enc_s[i] */
#define NE 1024
static uint32_t enc_t[NE], enc_n;
static uint8_t enc_s[NE];
static uint32_t enc_state(uint32_t t)
{
    uint32_t lo = 0, hi = enc_n;                   /* the last transition at or before t */
    while (lo < hi) {
        uint32_t m = (lo + hi) / 2u;
        if ((int32_t)(t - enc_t[m]) >= 0)
            lo = m + 1u;
        else
            hi = m;
    }
    return lo ? enc_s[lo - 1u] : 0u;
}

static void set_rows(uint32_t col, uint32_t t)    /* the rows of the column latched, as the tick will read them */
{
    uint32_t pa = 0xFFFFFFFFu, pb = 0xFFFFFFFFu, r;
    for (r = 1; r < 5u; r++) {
        int id = FM1_KEYMAP[r][col];
        if (id >= 0 && key_closed((uint32_t)id, t))
            pa &= ~(1u << (r + 4u));               /* PA5..PA8, low = closed */
    }
    if (col == 0u && (enc_state(t) >> 1 & 1u))    /* encoder 0: A at (0, PA0), B at (1, PA0) */
        pa &= ~1u;
    if (col == 1u && (enc_state(t) & 1u))
        pa &= ~1u;
    sim_reg[FM1_PA][FM1_IN / 4u] = pa;
    sim_reg[FM1_PB][FM1_IN / 4u] = pb;
}

/* ------------------------------------------------------------------ the LEDs: on-time per (column, row) --- */
static uint32_t cur_lines, cur_col = 0xFF, last_t, in_tick;
static uint64_t on_t[FM1_NCOL][8];                 /* TIMER4 ticks lit, since the last read */
static void led_account(void)
{
    uint32_t r, d = sim_now - last_t;
    if (cur_col < FM1_NCOL)
        for (r = 1; r < 5u; r++)
            if (cur_lines >> r & 1u)
                on_t[cur_col][r] += d;
    last_t = sim_now;
}
static void led_trace(uint32_t rowmask)
{
    led_account();
    cur_lines = rowmask;
}
static void latch_trace(void)
{
    led_account();
    cur_col = in_tick ? (fm1__tick_col + 1u == FM1_NCOL ? 0u : fm1__tick_col + 1u) : 0xFFu;
}

/* ------------------------------------------------------------------ the clock --- */
static uint32_t next_tick;                         /* the next tick's time */
static void tick(void)
{
    if ((int32_t)(sim_now - next_tick) > 0) {
        printf("  FAIL a tick ran past the next (%u ticks over)\n", (unsigned)(sim_now - next_tick));
        fails++;
        next_tick = sim_now;
    }
    sim_now = next_tick;
    led_account();                                 /* (the lines as the last tick left them, until now) */
    set_rows(fm1__tick_col, sim_now);
    in_tick = 1;
    fm1_input_tick();
    in_tick = 0;
    next_tick += T4_TICK;
}
static void run_until(uint32_t t) { while ((int32_t)(t - next_tick) > 0) tick(); }
static void run_ms(double ms) { run_until(next_tick + (uint32_t)(ms * 1000.0 * FM1_TICKS_PER_US)); }

/* (fm1_enc_take and fm1_input_note_edges, without their lock: one thread here) */
static int32_t enc_take(void)
{
    int32_t v = fm1_in.enc_steps[0];
    fm1_in.enc_steps[0] = 0;
    return v;
}
static uint32_t note_edges(void)
{
    uint32_t v = fm1_in.notes_pressed;
    fm1_in.notes_pressed = 0;
    return v;
}

static void reset(void)
{
    uint32_t i;
    memset((void *)&fm1_in, 0, sizeof fm1_in);
    memset(key_c, 0, sizeof key_c);
    memset(fm1_led, 0, sizeof fm1_led);
    memset(fm1_led_dim, 0, sizeof fm1_led_dim);
    memset(fm1_led_breath, 0, sizeof fm1_led_breath);
    memset(fm1_led_mid, 0, sizeof fm1_led_mid);
    enc_n = 0;
    fm1_input_init();
    for (i = 0; i < FM1_NENC; i++)
        fm1_in.enc_prev[i] = fm1_in.enc_last[i] = 0xFF;
    run_ms(30);
    fm1_in.pressed = fm1_in.released = 0;
    (void)note_edges();
}

static uint32_t rng = 12345u;
static uint32_t rnd(uint32_t n) { rng = rng * 1103515245u + 12345u; return (rng >> 8) % n; }

static int is_on(uint32_t id) { return id >= 14u ? (fm1_in.notes >> (id - 14u)) & 1u : (fm1_in.buttons >> id) & 1u; }

/* ------------------------------------------------------------------ the tests --- */
static void constants(void)
{
#if FM1_INPUT_HZ == 10000
    want("10000: the constants as before (2, 8, 20, 900, 1024, 8 / 12, 100 us, 3200 / 1600 ns, 64, 1)",
         FM1_DEB_PRESS == 2u && FM1_DEB_RELEASE == 8u && FM1_ENC_GO == 20u && FM1_REST_FRAMES == 900u &&
         FM1_LED_BREATH_FRAMES == 1024u && FM1_LED_MID_N == 8u && FM1_LED_MID_N_LO == 12u && FM1_LED_TICK_US == 100u &&
         FM1_LED_DIM_NS == 3200u && FM1_LED_DIM_LO_NS == 1600u && FM1_BATT_EVERY == 64u && FM1_ANIM_STEP == 1u);
#endif
    printf("  rate %u: tick %u us, frame %u us; press %u frames, release %u, ENC_GO %u, REST %u, breath %u frames, "
           "mid 1 in %u / %u, glow %u / %u ns, battery every %u, sweep step %u\n",
           (unsigned)FM1_INPUT_HZ, (unsigned)FM1_TICK_US, (unsigned)FM1_FRAME_US, (unsigned)FM1_DEB_PRESS,
           (unsigned)FM1_DEB_RELEASE, (unsigned)FM1_ENC_GO, (unsigned)FM1_REST_FRAMES, (unsigned)FM1_LED_BREATH_FRAMES,
           (unsigned)FM1_LED_MID_N, (unsigned)FM1_LED_MID_N_LO, (unsigned)FM1_LED_DIM_NS, (unsigned)FM1_LED_DIM_LO_NS,
           (unsigned)FM1_BATT_EVERY, (unsigned)FM1_ANIM_STEP);
    want("times kept: release ~9 ms (or 2 frames), ENC_GO ~22 ms (or 3 frames), REST ~1 s, breath 0.9..1.5 s",
         FM1_DEB_RELEASE * FM1_FRAME_US >= 8800u - FM1_FRAME_US / 2u && FM1_DEB_RELEASE >= 2u &&
         FM1_ENC_GO * FM1_FRAME_US >= 22000u - FM1_FRAME_US / 2u && FM1_ENC_GO >= 3u &&
         FM1_REST_FRAMES * FM1_FRAME_US > 940000u && FM1_REST_FRAMES * FM1_FRAME_US < 1040000u &&
         FM1_LED_BREATH_FRAMES * FM1_FRAME_US > 900000u && FM1_LED_BREATH_FRAMES * FM1_FRAME_US < 1500000u);
}

/* a clean press and release of key id at time t0 (a contact closed for `held` ms) */
static void contact(uint32_t id, uint32_t t0, const uint32_t *flips, int n)
{
    int i;
    key_c[id].on = 1;
    key_c[id].n = n;
    for (i = 0; i < n; i++)
        key_c[id].t[i] = t0 + flips[i];
}
#define US(x) ((uint32_t)(x) * FM1_TICKS_PER_US)

static void clean(void)
{
    static const uint32_t ids[] = {23, 14, 40, 5, 12, 0};   /* note keys on each row, buttons */
    uint32_t i, k, lat_max = 0, lat_min = 0xFFFFFFFFu, rel_max = 0, rel_min = 0xFFFFFFFFu;
    double lat_sum = 0, rel_sum = 0;
    uint32_t n = 0;
    char b[200];
    reset();
    for (k = 0; k < 6u; k++)
        for (i = 0; i < 40u; i++) {
            uint32_t id = ids[k], t0 = next_tick + T4_TICK * 3u + rnd(T4_FRAME), t, flips[2];
            flips[0] = 0;
            flips[1] = US(60000);                  /* held 60 ms */
            contact(id, t0, flips, 2);
            while (!is_on(id) && (int32_t)(next_tick - t0) < (int32_t)US(50000))
                tick();
            t = sim_now - t0;
            lat_sum += t;
            lat_max = t > lat_max ? t : lat_max;
            lat_min = t < lat_min ? t : lat_min;
            run_until(t0 + flips[1]);
            while (is_on(id) && (int32_t)(next_tick - t0) < (int32_t)US(200000))
                tick();
            t = sim_now - (t0 + flips[1]);
            rel_sum += t;
            rel_max = t > rel_max ? t : rel_max;
            rel_min = t < rel_min ? t : rel_min;
            n++;
            key_c[id].on = 0;
            run_ms(5);
        }
    snprintf(b, sizeof b, "a clean press counts within %u frames and a tick: %.2f .. %.2f ms, mean %.2f (bound %.2f)",
             (unsigned)FM1_DEB_PRESS, MS(lat_min), MS(lat_max), MS(lat_sum / n), MS(FM1_DEB_PRESS * T4_FRAME + T4_TICK));
    want(b, lat_max <= FM1_DEB_PRESS * T4_FRAME + T4_TICK && lat_min + 1u > (FM1_DEB_PRESS - 1u) * T4_FRAME);
    snprintf(b, sizeof b, "a clean release counts within %u frames and a tick: %.2f .. %.2f ms, mean %.2f",
             (unsigned)FM1_DEB_RELEASE, MS(rel_min), MS(rel_max), MS(rel_sum / n));
    want(b, rel_max <= FM1_DEB_RELEASE * T4_FRAME + T4_TICK && rel_min >= (FM1_DEB_RELEASE - 1u) * T4_FRAME);
}

/* a bounce: closed / open runs of 30..700 us for up to `span` us, from `start` (first flip: closed) */
static int bounce(uint32_t *f, int n, uint32_t start, uint32_t span)
{
    uint32_t t = start;
    int k = 0;
    while (n < NB && t - start < span) {
        f[n++] = US(t);
        t += 30u + rnd(670u);
        k++;
    }
    if (k % 2 == 0) {                              /* end closed (on the way down) or open (up): odd flips */
        f[n++] = US(t);
    }
    return n;
}

static void bouncy(void)
{
    uint32_t id, rep, presses = 0, releases = 0, early = 0, double_on = 0, missing = 0, hang = 0;
    char b[160];
    reset();
    for (rep = 0; rep < 8u; rep++)
        for (id = 0; id < FM1_NKEY; id++) {
            uint32_t f[NB + 2], t0 = next_tick + rnd(T4_FRAME), down_end, up0, n_on, n_off;
            int n = bounce(f, 0, 0, 200u + rnd(2800u));             /* down: ends closed */
            down_end = (n ? f[n - 1] / FM1_TICKS_PER_US : 0u);
            up0 = down_end + 60000u;                                 /* held 60 ms */
            {
                uint32_t g[NB + 2];
                int m = bounce(g, 0, 0, 200u + rnd(4800u)), i;
                for (i = 0; i < m && n < NB; i++)
                    f[n++] = US(up0) + g[i];
                if (n % 2)                                           /* (cut short: end open) */
                    f[n++] = US(up0) + US(5000);
            }
            contact(id, t0, f, n);
            n_on = n_off = 0;
            fm1_in.pressed = fm1_in.released = 0;
            (void)note_edges();
            while ((int32_t)(next_tick - (t0 + US(up0))) < 0) {
                uint32_t was = (uint32_t)is_on(id);
                tick();
                if (!was && is_on(id))
                    n_on++;
                if (was && !is_on(id))
                    early++;                                         /* ended while held */
            }
            while ((int32_t)(next_tick - (t0 + US(up0) + US(5000) + US(FM1_DEB_RELEASE * FM1_FRAME_US) +
                                          2u * T4_FRAME)) < 0) {
                uint32_t was = (uint32_t)is_on(id);
                tick();
                if (!was && is_on(id))
                    n_on++;
                if (was && !is_on(id))
                    n_off++;
            }
            hang += is_on(id) != 0;
            presses += n_on == 1u;
            double_on += n_on > 1u;
            missing += n_on == 0u;
            releases += n_off >= 1u;
            key_c[id].on = 0;
            run_ms(3);
        }
    snprintf(b, sizeof b, "%u bouncy presses: %u once, %u twice, %u missed, %u ended early, %u released, %u hang",
             (unsigned)(8u * FM1_NKEY), (unsigned)presses, (unsigned)double_on, (unsigned)missing, (unsigned)early,
             (unsigned)releases, (unsigned)hang);
    want(b, presses == 8u * FM1_NKEY && releases == 8u * FM1_NKEY && !early && !hang);
}

static void glitch(void)
{
    uint32_t i, played = 0;
    char b[160];
    reset();
    for (i = 0; i < 400u; i++) {
        uint32_t f[2] = {0, US(150)}, t0 = next_tick + rnd(T4_FRAME);
        contact(23, t0, f, 2);
        run_until(t0 + US(150) + 3u * T4_FRAME);
        played += (note_edges() >> 9) & 1u;
        run_until(next_tick + (FM1_DEB_RELEASE + 2u) * T4_FRAME);
        key_c[23].on = 0;
    }
    snprintf(b, sizeof b, "a 150 us glitch: %u of 400 played (a press of %u frame%s)", (unsigned)played,
             (unsigned)FM1_DEB_PRESS, FM1_DEB_PRESS > 1u ? "s: none may" : ": caught when a sample lands in it");
    want(b, FM1_DEB_PRESS > 1u ? played == 0u : played <= 400u * (150u + FM1_TICK_US) / FM1_FRAME_US + 20u);
}

static void repeats(void)
{
    uint32_t f[20], i, heard = 0, t0;
    char b[120];
    reset();
    for (i = 0; i < 20u; i++)
        f[i] = US(i * 40000u);                     /* 40 ms on, 40 off, 10 times */
    t0 = next_tick + rnd(T4_FRAME);
    contact(30, t0, f, 20);
    for (i = 0; i < 45u; i++) {
        run_ms(20);
        heard += (note_edges() >> 16) & 1u ? 1u : 0u;
    }
    run_until(t0 + US(820000));
    snprintf(b, sizeof b, "fast repeats (40 ms on, 40 off): %u of 10 heard", (unsigned)heard);
    want(b, heard == 10u);
}

/* encoder 0: `clicks` clicks (4 transitions each, sign = direction), evenly at `tps` transitions a second, from
 * `t0`; then rest. Returns the steps counted */
static const uint8_t CW[4] = {1, 3, 2, 0}, CCW[4] = {2, 3, 1, 0};   /* 0 -> .. -> 0: a click */
static int32_t turn(int clicks, double tps, uint32_t t0, int lead)
{
    uint32_t i, n = (uint32_t)(clicks < 0 ? -clicks : clicks) * 4u, st = 0;
    double t = 0;
    int32_t s;
    enc_n = 0;
    for (i = 0; i < n && enc_n < NE; i++) {        /* a lead-in click at 10 clicks/s first (the flick) */
        double dt = (lead && i < 4u) ? 1.0 / 40.0 : 1.0 / tps;
        t += dt;
        st = clicks > 0 ? CW[i & 3u] : CCW[i & 3u];
        enc_t[enc_n] = t0 + (uint32_t)(t * 1e6 * FM1_TICKS_PER_US);
        enc_s[enc_n++] = (uint8_t)st;
    }
    run_until(t0 + (uint32_t)(t * 1e6 * FM1_TICKS_PER_US) + US(100000));
    s = enc_take();
    return s;
}

static int all_kept(double cps, int lead, int clicks)
{
    int ph, ok = 1;
    for (ph = 0; ph < 6 && ok; ph++) {
        uint32_t t0 = next_tick + (uint32_t)ph * T4_FRAME / 6u;
        ok &= turn(clicks, cps * 4.0, t0, lead) == clicks;
        ok &= turn(-clicks, cps * 4.0, next_tick + (uint32_t)ph * T4_FRAME / 7u, lead) == -clicks;
    }
    return ok;
}

static void encoders(void)
{
    double frame_hz = 1e6 / FM1_FRAME_US, cps, best = 0, best_flick = 0;
    char b[160];
    reset();
    want("encoder: 20 clicks each way at 10 clicks/s", all_kept(10.0, 0, 20));
    snprintf(b, sizeof b, "encoder: 20 clicks each way at %.0f clicks/s (a state each 1.25 frames)", frame_hz / 5.0);
    want(b, all_kept(frame_hz / 5.0, 0, 20));
    snprintf(b, sizeof b, "encoder: a flick, 1 click at 10 clicks/s then 19 at %.0f clicks/s (1.6 states a frame)",
             frame_hz * 1.6 / 4.0);
    want(b, all_kept(frame_hz * 1.6 / 4.0, 1, 20));
    for (cps = frame_hz / 10.0; cps < frame_hz; cps *= 1.05)
        if (all_kept(cps, 0, 20))
            best = cps;
        else
            break;
    for (cps = frame_hz / 10.0; cps < frame_hz; cps *= 1.05)
        if (all_kept(cps, 1, 20))
            best_flick = cps;
        else
            break;
    printf("  encoder: every step kept up to ~%.0f clicks/s turned evenly from rest, ~%.0f once moving (frames %.0f Hz)\n",
           best, best_flick, frame_hz);
}

static uint32_t take_on(uint32_t c, uint32_t r)
{
    uint64_t v = on_t[c][r];
    on_t[c][r] = 0;
    return (uint32_t)v;
}
static void frame_sync(void)                       /* run to the start of a frame (column 0 about to be latched) */
{
    while (fm1__tick_col != FM1_NCOL - 1u)
        tick();
}

static void leds(void)
{
    uint32_t f, lit_min = 0xFFFFFFFFu, lit_max = 0, dim_min = 0xFFFFFFFFu, dim_max = 0, mid_lit = 0, mid_bad = 0;
    uint32_t c0 = 3, r0 = 3;                        /* column 3, row PA7 (key 26) */
    char b[200];
    reset();
    fm1_led[c0] = 1u << r0;                        /* lit */
    fm1_led_dim[c0 + 1u] = 1u << r0;               /* the glow */
    frame_sync();
    for (f = 0; f < 200u; f++) {                   /* (the glow settles in a few frames: fm1__dim_k) */
        uint32_t i, lit, dim;
        for (i = 0; i < FM1_NCOL; i++)
            tick();
        lit = take_on(c0, r0);
        dim = take_on(c0 + 1u, r0);
        if (f < 50u)
            continue;
        lit_min = lit < lit_min ? lit : lit_min;
        lit_max = lit > lit_max ? lit : lit_max;
        dim_min = dim < dim_min ? dim : dim_min;
        dim_max = dim > dim_max ? dim : dim_max;
    }
    snprintf(b, sizeof b, "a lit LED: %.1f .. %.1f us a frame (a tick %u us), every frame: %.0f Hz", lit_min / 24.0,
             lit_max / 24.0, (unsigned)FM1_TICK_US, 1e6 / FM1_FRAME_US);
    want(b, lit_min >= T4_TICK * 8u / 10u && lit_max <= T4_TICK);
    snprintf(b, sizeof b, "the glow: %.2f .. %.2f us a frame (FM1_LED_DIM_NS %u), every frame: ~1/%.0f of lit",
             dim_min / 24.0, dim_max / 24.0, (unsigned)FM1_LED_DIM_NS, (double)lit_max / (dim_min ? dim_min : 1u));
    want(b, dim_min * 1000u >= FM1_LED_DIM_NS * 24u * 7u / 10u && dim_max * 1000u <= FM1_LED_DIM_NS * 24u * 13u / 10u);

    /* the mid level */
    fm1_led[c0] = 0;
    fm1_led_dim[c0 + 1u] = 0;
    fm1_led_mid[c0] = 1u << r0;
    frame_sync();
    (void)take_on(c0, r0);
    for (f = 0; f < 120u * FM1_LED_MID_N; f++) {
        uint32_t i, on;
        for (i = 0; i < FM1_NCOL; i++)
            tick();
        on = take_on(c0, r0);
        if (on > T4_TICK / 2u)
            mid_lit++;
        else if (on)
            mid_bad++;
    }
    snprintf(b, sizeof b, "the mid level: lit %u of %u frames (1 in %u: %.0f Hz; DIM LO 1 in %u: %.0f Hz)",
             (unsigned)mid_lit, (unsigned)(120u * FM1_LED_MID_N), (unsigned)FM1_LED_MID_N,
             1e6 / FM1_FRAME_US / FM1_LED_MID_N, (unsigned)FM1_LED_MID_N_LO, 1e6 / FM1_FRAME_US / FM1_LED_MID_N_LO);
    want(b, mid_lit == 120u && !mid_bad);
    fm1_led_mid[c0] = 0;

    /* the breath: from its peak (lit frames), dark at half a period, back at its peak a period on */
    {
        uint32_t lit_frames[4] = {0}, q, nfr = FM1_LED_BREATH_FRAMES, dark_run = 0, run = 0, max_run_hi = 0;
        fm1_led_breath[c0] = 1u << r0;
        frame_sync();
        (void)take_on(c0, r0);
        for (f = 0; f < nfr; f++) {
            uint32_t i, on;
            for (i = 0; i < FM1_NCOL; i++)
                tick();
            on = take_on(c0, r0);
            q = f * 4u / nfr;                      /* quarters: around the peak, falling, dark, rising */
            if (on > T4_TICK / 2u) {
                lit_frames[q]++;
                if (run > dark_run)
                    dark_run = run;
                if ((f < nfr / 8u || f > nfr - nfr / 8u) && run > max_run_hi)
                    max_run_hi = run;
                run = 0;
            } else {
                run++;
            }
        }
        snprintf(b, sizeof b, "the breath: %u frames, %.2f s; lit frames by quarter %u %u %u %u; near the peak a dark "
                 "run of at most %u frames (%.0f Hz)", (unsigned)nfr, nfr * FM1_FRAME_US / 1e6, (unsigned)lit_frames[0],
                 (unsigned)lit_frames[1], (unsigned)lit_frames[2], (unsigned)lit_frames[3], (unsigned)max_run_hi,
                 1e6 / FM1_FRAME_US / (max_run_hi + 1u));
        want(b, lit_frames[0] > lit_frames[1] && lit_frames[3] > lit_frames[2] && lit_frames[0] > nfr / 16u &&
                nfr * FM1_FRAME_US > 900000u && nfr * FM1_FRAME_US < 1500000u);
        fm1_led_breath[c0] = 0;
    }

    /* the power-on sweep: ~0.7 s at every rate */
    {
        uint32_t t0, d;
        frame_sync();
        fm1_led_anim_start(1);
        t0 = next_tick;
        while (fm1_led_anim_on() && (int32_t)(next_tick - t0) < (int32_t)US(3000000))
            tick();
        d = next_tick - t0;
        snprintf(b, sizeof b, "the power-on sweep: %.0f ms (%u frames of 1.1 ms at 10000: 704)", MS(d),
                 (unsigned)FM1_ANIM_FRAMES);
        want(b, MS(d) > 690.0 && MS(d) < 720.0 + MS(T4_FRAME));
        fm1_led_idle_start(1);                     /* the idle animation: runs, a key ends it */
        run_ms(500);
        want("the idle animation runs, and a key ends it", fm1_led_idle_on());
        {
            uint32_t fl[2] = {0, US(100000)};
            contact(23, next_tick, fl, 2);
            run_ms(50);
            want("  (ended by a key)", !fm1_led_idle_on());
            key_c[23].on = 0;
            run_ms(50);
        }
    }
}

int main(void)
{
    printf("input at %u Hz\n", (unsigned)FM1_INPUT_HZ);
    next_tick = 1000u;
    constants();
    clean();
    bouncy();
    glitch();
    repeats();
    encoders();
    leds();
    printf(fails ? "input %u Hz: %d FAILED\n" : "input %u Hz: all passed\n", (unsigned)FM1_INPUT_HZ, fails);
    return fails != 0;
}
