/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 hdavid */
/* The update loader's screen (loader/ldr_screen.c) on a model of the panel: the commands and pixels it is sent land
 * in a 240x240 frame, and each line of text is read back off it. Checked: the percentage under the bar; "USB LOST" /
 * "RECONNECT" once the host has been silent 2 s of polls mid-install (never before an install, never after it is
 * done), the bar and the percentage kept; "UPDATING" / "DO NOT UNPLUG" again at the host's next frame; "DONE" /
 * "RESTARTING" at the end; and nothing sent to the panel when nothing changes (the write path's polls stay cheap). */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* ---- the panel: CASET / RASET / RAMWR into a frame, the bytes sent counted ---- */
static uint16_t frame[240][240];
static uint32_t win[4], wx, wy, sent;
static uint8_t cmd, inits;

static void fm1_lcd_hw_init(void) { inits++; }
static void fm1_delay_ms(uint32_t ms) { (void)ms; }
static void fm1_lcd_wait(void) {}
static void fm1_lcd_deselect(void) {}
static void fm1_lcd_send_cmd(uint8_t c)
{
    cmd = c;
    sent++;
    if (c == 0x2C) {
        wx = win[0];
        wy = win[2];
    }
}
static void fm1_lcd_send_data(const void *p, uint32_t n)
{
    const uint8_t *b = p;
    uint32_t i;
    sent += n;
    if (cmd == 0x2A || cmd == 0x2B) {
        uint32_t *w = win + (cmd == 0x2B ? 2 : 0);
        w[0] = (uint32_t)b[0] << 8 | b[1];
        w[1] = (uint32_t)b[2] << 8 | b[3];
        return;
    }
    if (cmd != 0x2C)
        return;
    for (i = 0; i + 1 < n; i += 2) {                /* MSB first */
        if (wx < 240 && wy < 240)
            frame[wy][wx] = (uint16_t)(b[i] << 8 | b[i + 1]);
        if (++wx > win[1]) {
            wx = win[0];
            wy++;
        }
    }
}

#define LDR_SCREEN_TEST 1
#include "../loader/ldr_screen.c"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); \
                                       printf("\n"); } } while (0)

/* the band [y, y + 7 * scale) is exactly s, centred, on the cream: every other pixel of the 240 columns the background */
static int line_is(uint32_t y, const char *s, uint32_t scale)
{
    uint32_t len = (uint32_t)strlen(s), w = len ? len * 6u * scale - scale : 0, x0 = (240u - w) / 2u, x, r, i, f;
    for (i = 0; i < len; i++) {
        int found = s[i] == ' ';
        for (f = 0; f < sizeof SCR_FONT / sizeof SCR_FONT[0]; f++)
            found |= SCR_FONT[f].c == s[i];
        if (!found) {
            printf("  no glyph for '%c'\n", s[i]);
            return 0;
        }
    }
    for (r = 0; r < 7u * scale; r++)
        for (x = 0; x < 240u; x++) {
            uint16_t want = SCR_BG;
            if (x >= x0 && x < x0 + w && (x - x0) % (6u * scale) < 5u * scale) {
                const uint8_t *g = 0;
                i = (x - x0) / (6u * scale);
                for (f = 0; f < sizeof SCR_FONT / sizeof SCR_FONT[0]; f++)
                    if (SCR_FONT[f].c == s[i])
                        g = SCR_FONT[f].rows;
                if (g && g[r / scale] & (0x10u >> ((x - x0) % (6u * scale) / scale)))
                    want = SCR_FG;
            }
            if (frame[y + r][x] != want) {
                printf("  \"%s\": pixel (%u, %u) is %04x, not %04x\n", s, x, y + r, frame[y + r][x], want);
                return 0;
            }
        }
    return 1;
}

static uint32_t bar_px(void)                        /* the bar's filled pixels, its middle row */
{
    uint32_t x, n = 0;
    for (x = 0; x < 240u; x++)
        n += frame[SCR_BAR_Y + SCR_BAR_H / 2u][x] == SCR_BAR;
    return n;
}

/* the lines, the percentage and the bar; the power warning on every screen but DONE's */
static void screen(const char *big, const char *small, const char *pc, uint32_t bar, int line)
{
    const char *warn = strcmp(big, "DONE") ? "DO NOT POWER OFF" : "";
    CHECK(line_is(SCR_BIG_Y, big, 3), "line %d: the big line is not \"%s\"", line, big);
    CHECK(line_is(SCR_SMALL_Y, small, 2), "line %d: the small line is not \"%s\"", line, small);
    CHECK(line_is(SCR_POWER_Y, warn, 2), "line %d: the power line is not \"%s\"", line, warn);
    CHECK(line_is(SCR_PC_Y, pc, 2), "line %d: the percentage is not \"%s\"", line, pc);
    CHECK(bar_px() == bar, "line %d: the bar has %u pixels, not %u", line, bar_px(), bar);
}

static void clear(void)
{
    uint32_t x, y;
    for (y = 0; y < 240u; y++)
        for (x = 0; x < 240u; x++)
            frame[y][x] = SCR_BG;
}

static uint32_t sweep_x(void)                       /* the sweep's first pixel in the bar's row, 240: none */
{
    uint32_t x;
    for (x = 0; x < 240u; x++)
        if (frame[SCR_BAR_Y + SCR_BAR_H / 2u][x] == SCR_BAR)
            return x;
    return 240u;
}

static void reset(void)
{
    clear();
    scr_on = 0, scr_shown = 0, scr_pc = 0, scr_state = 0, scr_armed = 0, scr_quiet = 0, scr_wait = 0, scr_sweep = 0;
}

static void ticks(uint32_t n)
{
    while (n--)
        ldr_screen_tick();
}

int main(void)
{
    uint32_t i;
    ldr_screen_init();
    CHECK(inits == 1, "the panel is started once");
    screen("UPDATING", "DO NOT UNPLUG", "0%", 0, __LINE__);

    /* waiting for a host before any install: silence is not a lost cable; the sweep steps every 100 ms, and only then */
    sent = 0;
    ticks(SCR_SWEEP_POLLS - 1u);
    CHECK(sent == 0 && sweep_x() == 240u, "%u bytes sent before the first sweep step", sent);
    ticks(1);
    screen("UPDATING", "DO NOT UNPLUG", "0%", SCR_SWEEP_W, __LINE__);
    {
        uint32_t x0 = sweep_x(), moved = 0, back = 0, last = x0;
        for (i = 0; i < 3u * SCR_LOST_POLLS / SCR_SWEEP_POLLS; i++) {
            ticks(SCR_SWEEP_POLLS);
            moved += sweep_x() != last;
            back += sweep_x() < last;
            last = sweep_x();
            if (bar_px() != SCR_SWEEP_W || last < SCR_BAR_X || last + SCR_SWEEP_W > SCR_BAR_X + SCR_BAR_W)
                break;
        }
        CHECK(x0 == SCR_BAR_X && moved == 3u * SCR_LOST_POLLS / SCR_SWEEP_POLLS && back >= 1u && back <= 3u,
              "the sweep: from x %u, moved %u times, back to the start %u times, stopped at step %u", x0, moved, back, i);
    }
    screen("UPDATING", "DO NOT UNPLUG", "0%", SCR_SWEEP_W, __LINE__);

    /* the install starts: the sweep goes; 42 %: the bar and the percentage */
    ldr_screen_progress(0, 1);
    screen("UPDATING", "DO NOT UNPLUG", "0%", 0, __LINE__);
    sent = 0;
    ticks(SCR_SWEEP_POLLS * 3u);
    CHECK(sent == 0, "%u bytes sent: the sweep after the install started", sent);
    ldr_screen_heard();
    ldr_screen_progress(42, 100);
    screen("UPDATING", "DO NOT UNPLUG", "42%", 42u * SCR_BAR_W / 100u, __LINE__);
    sent = 0;
    ldr_screen_progress(42, 100);                   /* the same: nothing sent */
    ldr_screen_progress(4201, 10000);               /* the same percentage and bar */
    CHECK(sent == 0, "%u bytes sent for no change", sent);

    /* frames keep coming: never lost */
    for (i = 0; i < 10u; i++) {
        ticks(SCR_LOST_POLLS - 1u);
        ldr_screen_heard();
    }
    CHECK(sent == 0, "%u bytes sent while the host talks", sent);
    screen("UPDATING", "DO NOT UNPLUG", "42%", 42u * SCR_BAR_W / 100u, __LINE__);

    /* the host goes silent: lost after 2 s of polls, not one poll before */
    ticks(SCR_LOST_POLLS - 1u);
    CHECK(sent == 0, "%u bytes sent before 2 s of silence", sent);
    screen("UPDATING", "DO NOT UNPLUG", "42%", 42u * SCR_BAR_W / 100u, __LINE__);
    ticks(1);
    screen("USB LOST", "RECONNECT THE CABLE", "42%", 42u * SCR_BAR_W / 100u, __LINE__);
    sent = 0;
    ticks(10u * SCR_LOST_POLLS);                    /* drawn once */
    CHECK(sent == 0, "%u bytes sent while lost", sent);

    /* the host again (the session failed: it starts over, the loader skips what is written): back, the fill kept */
    ldr_screen_heard();
    screen("UPDATING", "DO NOT UNPLUG", "42%", 42u * SCR_BAR_W / 100u, __LINE__);
    sent = 0;
    ldr_screen_heard();
    CHECK(sent == 0, "%u bytes sent for a second frame", sent);
    ldr_screen_progress(0, 1);
    ldr_screen_progress(10, 100);                   /* the new session's first steps: below what is shown */
    CHECK(sent == 0, "%u bytes sent for a smaller step", sent);
    screen("UPDATING", "DO NOT UNPLUG", "42%", 42u * SCR_BAR_W / 100u, __LINE__);
    ldr_screen_progress(99, 100);
    screen("UPDATING", "DO NOT UNPLUG", "99%", 99u * SCR_BAR_W / 100u, __LINE__);

    /* lost again, then the end */
    ticks(SCR_LOST_POLLS);
    screen("USB LOST", "RECONNECT THE CABLE", "99%", 99u * SCR_BAR_W / 100u, __LINE__);
    ldr_screen_heard();
    ldr_screen_progress(100, 100);
    screen("UPDATING", "DO NOT UNPLUG", "100%", SCR_BAR_W, __LINE__);
    ldr_screen_done();
    screen("DONE", "RESTARTING", "100%", SCR_BAR_W, __LINE__);
    sent = 0;
    ticks(3 * SCR_LOST_POLLS);                      /* done: the host may go */
    ldr_screen_heard();
    CHECK(sent == 0, "%u bytes sent after done", sent);
    screen("DONE", "RESTARTING", "100%", SCR_BAR_W, __LINE__);

    /* every percentage reads back */
    for (i = 0; i <= 100u; i++) {
        char want[8];
        clear();
        scr_percent(i);
        snprintf(want, sizeof want, "%u%%", i);
        CHECK(line_is(SCR_PC_Y, want, 2), "the percentage %u", i);
    }

    /* done with a frame that never reached 100 %: the bar is full */
    reset();
    ldr_screen_init();
    ldr_screen_progress(0, 1);
    ldr_screen_progress(7, 10);
    ldr_screen_done();
    screen("DONE", "RESTARTING", "100%", SCR_BAR_W, __LINE__);

    /* the fast install's calls (loader/ldr_core.c ldr_fast): the table read (its bytes), then every app sector, kept or
     * written, one step: the bar and the percentage move from 0 over the sectors compared, from the first one */
    {
        const uint32_t tb = (0x100000u >> 12) * 6u, app = 0x93000u - 0x4000u, total = tb + app;
        uint32_t k, moved = 0, last_pc = 0;
        reset();
        ldr_screen_init();
        ldr_screen_progress(0, 1);
        ldr_screen_progress(tb, total);
        CHECK(bar_px() <= 2u && scr_pc == 0, "after the table: %u bar pixels, %u%%", bar_px(), scr_pc);
        for (k = 1; k <= app >> 12; k++) {
            uint32_t done = tb + (k << 12), pc = done * 100u / total;
            char want[8];
            ldr_screen_progress(done, total);
            snprintf(want, sizeof want, "%u%%", pc);
            if (!line_is(SCR_PC_Y, want, 2) || bar_px() != (k == app >> 12 ? SCR_BAR_W : done * SCR_BAR_W / total)) {
                CHECK(0, "fast install, sector %u: not %s / its bar", k, want);
                break;
            }
            moved += pc != last_pc;
            last_pc = pc;
        }
        CHECK(moved >= 95u, "the percentage moved %u times over the sectors", moved);
        screen("UPDATING", "DO NOT UNPLUG", "100%", SCR_BAR_W, __LINE__);
    }

    printf("%s: ldr_screen_test\n", fails ? "FAIL" : "ok");
    return fails != 0;
}
