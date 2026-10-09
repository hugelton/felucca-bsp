/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* felucca-bsp hardware test for the FM-1. Not a synth: it only exercises the BSP.
 *
 *   knob     MASTER pot and battery level as bars (ADC)
 *   buttons  14 buttons and 27 note keys light a cell on the screen and their own LED
 *   encoders 7 counters, one cell each (a click moves the bar; green = clockwise)
 *   screen   colour bars at the top confirm the panel and its colour order
 *   LEDs     a walk over all 41 LEDs at power-on, then each key lights its own
 *   USB      CDC serial "debug port" (115200 8N1, the rate is ignored): a status line every 250 ms
 *            and one-letter commands: h help, p print now, c last crash, x crash on purpose, w LED walk,
 *            k start the second core (experimental, 'cpu1=' then counts in the status line), u UBOOT, r reboot
 *
 * Two boots in a row that never reach the main loop's "healthy" point drop into UBOOT, so a
 * broken build can always be replaced. Includes usb.c, so the image is GPL-3.0 as a whole. */
#include <stdint.h>
#ifndef HWTEST_EXPERIMENTAL
#define HWTEST_EXPERIMENTAL 0     /* 1: also the experimental parts (second core, a deliberate crash): hwtest-exp */
#endif
#define FELUCCA_CDC 1
#define FELUCCA_OTA 0
#define FELUCCA_ID (HWTEST_EXPERIMENTAL ? "felucca-bsp hwtest-exp" : "felucca-bsp hwtest")
#define RING_PUBLISH() __asm__ volatile("" ::: "memory")
#include "fm1_time.h"
#include "fm1_sys.h"
#include "fm1_irq.h"
#include "fm1_guard.h"
#if HWTEST_EXPERIMENTAL
#include "fm1_cpu1.h"
#endif
#include "fm1_timer.h"
#include "fm1_input.h"
#include "fm1_adc.h"
#include "fm1_lcd_hw.h"
#include "libc.c"
#include "usb.c"

extern uint32_t _data_start[], _data_end[], _data_load[], _bss_start[], _bss_end[];

static volatile uint32_t ms;
#if HWTEST_EXPERIMENTAL
static volatile uint32_t cpu1_count;               /* counted by CPU1 once started ('k') */
static uint8_t cpu1_started;
void fm1_cpu1_main(void)                            /* runs on the second core: polled, no interrupts */
{
    fm1_cpu1_ready();
    for (;;)
        cpu1_count++;
}
#endif
static uint32_t boots_ok;
static struct { uint32_t magic, pending, failed; } guard __attribute__((section(".noinit")));
void fm1_alnk0_irq(void) {}                     /* isr_alnk0 is linked but no audio runs */

void fm1_timer5_irq(void)                       /* 10 kHz: key scan, ms, USB at 2 kHz */
{
    static uint32_t sub, last, acc;
    uint32_t t = fm1_ticks();
    fm1_timer5_ack();
    fm1_input_tick();
    acc += t - last;
    last = t;
    while (acc >= 1000u * FM1_TICKS_PER_US) {
        acc -= 1000u * FM1_TICKS_PER_US;
        ms++;
    }
    if (++sub == 5u) {
        sub = 0;
        usb_poll();
    }
}
extern void isr_timer5(void);

/* ---- screen: ST7789 240x240, RGB565 sent high byte first (colours below are pre-swapped) ---- */
#define C(x) ((uint16_t)((((x) >> 8) | ((x) << 8)) & 0xFFFFu))
enum { RED = 0xF800, GRN = 0x07E0, BLU = 0x001F, WHT = 0xFFFF, BLK = 0, DIM = 0x2104, CYN = 0x07FF, YEL = 0xFFE0 };
static uint16_t row_px[240];

static void lcd_cmd(uint8_t c, const uint8_t *d, uint32_t n)
{
    static uint8_t buf[4];
    uint32_t i;
    fm1_lcd_send_cmd(c);
    fm1_lcd_wait();
    fm1_lcd_deselect();
    if (!n)
        return;
    for (i = 0; i < n; i++)
        buf[i] = d[i];
    fm1_lcd_send_data(buf, n);
    fm1_lcd_wait();
    fm1_lcd_deselect();
}

static void lcd_init(void)
{
    static const uint8_t colmod = 0x55, madctl = 0x00;
    fm1_lcd_hw_init();
    fm1_delay_ms(50);
    lcd_cmd(0x01, 0, 0);                        /* SWRESET */
    fm1_delay_ms(150);
    lcd_cmd(0x11, 0, 0);                        /* SLPOUT */
    fm1_delay_ms(120);
    lcd_cmd(0x3A, &colmod, 1);                  /* RGB565 */
    lcd_cmd(0x36, &madctl, 1);                  /* top-left origin, RGB */
    lcd_cmd(0x21, 0, 0);                        /* INVON: this panel is IPS */
    lcd_cmd(0x13, 0, 0);                        /* NORON */
    fm1_lcd_baud(4);
}

static void lcd_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t rgb)
{
    uint8_t a[4];
    uint32_t i;
    for (i = 0; i < w; i++)
        row_px[i] = C(rgb);
    a[0] = 0; a[1] = (uint8_t)x; a[2] = 0; a[3] = (uint8_t)(x + w - 1u);
    lcd_cmd(0x2A, a, 4);
    a[1] = (uint8_t)y; a[3] = (uint8_t)(y + h - 1u);
    lcd_cmd(0x2B, a, 4);
    lcd_cmd(0x2C, 0, 0);
    for (i = 0; i < h; i++) {
        fm1_lcd_send_data(row_px, w * 2u);
        fm1_lcd_wait();
        fm1_lcd_deselect();
    }
}

/* a fault: red screen, then reset; the record in fm1_crash survives and is shown on the next boot ('c') */
static void fm1_fault(const fm1_crash_t *c)
{
    (void)c;
    lcd_rect(0, 0, 240, 240, RED);
    fm1_delay_ms(1500);
    fm1_reboot();
}

/* ---- layout ---- */
#define BTN_X(i) (4u + ((i) % 7u) * 33u)
#define BTN_Y(i) (104u + ((i) / 7u) * 26u)
#define KEY_X(i) (4u + ((i) % 9u) * 26u)
#define KEY_Y(i) (160u + ((i) / 9u) * 22u)
#define ENC_X(i) (4u + (i) * 33u)

static void draw_static(void)
{
    uint32_t i;
    lcd_rect(0, 0, 240, 240, BLK);
    for (i = 0; i < 6; i++) {                   /* colour bars: R G B white yellow cyan */
        static const uint16_t bars[6] = { RED, GRN, BLU, WHT, YEL, CYN };
        lcd_rect(i * 40u, 0, 40, 20, bars[i]);
    }
    for (i = 0; i < 14; i++)
        lcd_rect(BTN_X(i), BTN_Y(i), 30, 22, DIM);
    for (i = 0; i < 27; i++)
        lcd_rect(KEY_X(i), KEY_Y(i), 24, 20, DIM);
    for (i = 0; i < 7; i++)
        lcd_rect(ENC_X(i), 72, 30, 22, DIM);
}

static void draw_bar(uint32_t y, int32_t v, uint16_t col)   /* v 0..1023 over 232 px */
{
    uint32_t w = v < 0 ? 0u : (uint32_t)v * 232u / 1023u;
    lcd_rect(4, y, 232, 12, DIM);
    if (w)
        lcd_rect(4, y, w, 12, col);
}

/* ---- serial console ---- */
static void put(char c)
{
    if (!cdc.dtr || co_w - co_r >= CO_N)
        return;
    cdc_out[co_w % CO_N] = (uint8_t)c;
    RING_PUBLISH();
    co_w++;
}
static void puts_(const char *s) { while (*s) put(*s++); }
static void putd(const char *k, uint32_t v)
{
    char b[10];
    uint32_t n = 0;
    puts_(k);
    do { b[n++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (n)
        put(b[--n]);
    put(' ');
}
static void puthex(const char *k, uint32_t v)
{
    uint32_t i;
    puts_(k);
    for (i = 8; i--;)
        put("0123456789ABCDEF"[(v >> (4u * i)) & 15u]);
    put(' ');
}

static int32_t enc_pos[FM1_NENC];
static uint32_t walk_until;

static void status(int32_t knob, int32_t batt)
{
    uint32_t i;
    putd("ms=", ms); putd("knob=", (uint32_t)knob); putd("batt=", (uint32_t)batt);
    puthex("btn=", fm1_in.buttons); puthex("keys=", fm1_in.notes);
    puts_("enc=");
    for (i = 0; i < FM1_NENC; i++) {
        putd("", (uint32_t)enc_pos[i]);
    }
    putd("usb_cfg=", usb.config);
#if HWTEST_EXPERIMENTAL
    if (cpu1_started)
        putd("cpu1=", cpu1_count);
#endif
    puts_("\r\n");
}

static void command(char c, int32_t knob, int32_t batt)
{
    switch (c) {
    case 'h':
        puts_(HWTEST_EXPERIMENTAL ? "h help, p print, c last crash, w LED walk, u UBOOT, r reboot, k start CPU1, x crash on purpose\r\n"
                                  : "h help, p print, c last crash, w LED walk, u UBOOT, r reboot\r\n");
        break;
    case 'p':
        status(knob, batt);
        break;
    case 'c':                                   /* the last crash, if any */
        if (fm1_crash.magic != FM1_CRASH_MAGIC) {
            puts_("no crash on record\r\n");
            break;
        }
        putd("count=", fm1_crash.count); putd("vec=", fm1_crash.vec); putd("up_ms=", fm1_crash.uptime_ms);
        puthex("pc=", fm1_crash.pc); puthex("rets=", fm1_crash.rets); puthex("emu=", fm1_crash.emu);
        puthex("dbg=", fm1_crash.dbg); puthex("sp=", fm1_crash.sp);
        puts_("\r\n");
        break;
#if HWTEST_EXPERIMENTAL
    case 'k':                                   /* start the second core (experimental) */
        if (cpu1_started) {
            puts_("cpu1 already started\r\n");
            break;
        }
        fm1_guard_unlock_top();
        cpu1_started = fm1_cpu1_start(100) == 0;
        fm1_guard_lock_top();
        puts_(cpu1_started ? "cpu1 up\r\n" : "cpu1 did not report in\r\n");
        break;
    case 'x':                                   /* test the guards: write to the NULL page */
        puts_("crashing\r\n");
        fm1_delay_ms(30);
        *(volatile uint32_t *)0 = 1;
        break;
#endif
    case 'w':
        walk_until = ms + 41u * 60u;
        break;
    case 'u':
        puts_("UBOOT\r\n");
        fm1_delay_ms(30);
        usb_detach();
        fm1_enter_uboot();
        break;
    case 'r':
        puts_("reboot\r\n");
        fm1_delay_ms(30);
        usb_detach();
        fm1_reboot();
        break;
    }
}

static void app_main(void)
{
    uint32_t i, last_status = 0, last_bar = 0, shown_btn = ~0u, shown_keys = ~0u, shown_enc = 0;
    int32_t knob = 0, batt = 0;
    lcd_init();
    draw_static();
    fm1_input_init();
    fm1_adc_init();
    usb_start();
    fm1_timer5_start(isr_timer5, 4);
    fm1_guard_lock_top();
    fm1_irq_enable_all();
    fm1_lcd_backlight(1);
    if (fm1_crash.magic == FM1_CRASH_MAGIC)     /* orange mark: the last run crashed ('c' has the details) */
        lcd_rect(200, 228, 24, 10, YEL);
    walk_until = 41u * 60u;
    for (;;) {
        uint32_t now = ms;
        fm1_wdt_feed();
        usb_retry(now);
        if (usb.uboot_req)                      /* the SysEx soft key */
            command('u', 0, 0);
        if (!boots_ok && now > 2000u) {         /* it runs: the next boot is not a loop */
            boots_ok = 1;
            guard.pending = 0;
            guard.failed = 0;
        }
        if (now - last_bar >= 40u) {            /* ADC: polled, a few hundred us */
            last_bar = now;
            knob = fm1_adc_read(FM1_ADC_MASTER);
            batt = fm1_adc_read(FM1_ADC_BATT);
            draw_bar(28, knob, WHT);
            draw_bar(46, batt, GRN);
        }
        for (i = 0; i < FM1_NENC; i++) {
            int32_t s = fm1_enc_take(i);
            if (s) {
                enc_pos[i] += s;
                lcd_rect(ENC_X(i), 72, 30, 22, s > 0 ? GRN : RED);
                lcd_rect(ENC_X(i) + 2u, 90, (uint32_t)(enc_pos[i] & 15) * 28u / 15u + 1u, 2, WHT);
                shown_enc |= 1u << i;
            }
        }
        if (shown_btn != fm1_in.buttons) {
            uint32_t d = shown_btn ^ fm1_in.buttons;
            for (i = 0; i < 14; i++)
                if (d & (1u << i))
                    lcd_rect(BTN_X(i), BTN_Y(i), 30, 22, (fm1_in.buttons >> i) & 1u ? WHT : DIM);
            shown_btn = fm1_in.buttons;
        }
        if (shown_keys != fm1_in.notes) {
            uint32_t d = shown_keys ^ fm1_in.notes;
            for (i = 0; i < 27; i++)
                if (d & (1u << i))
                    lcd_rect(KEY_X(i), KEY_Y(i), 24, 20, (fm1_in.notes >> i) & 1u ? CYN : DIM);
            shown_keys = fm1_in.notes;
        }
        if (now < walk_until) {                 /* walk: one LED at a time, then back to key = LED */
            uint32_t at = (41u * 60u - (walk_until - now)) / 60u;
            for (i = 0; i < FM1_NKEY; i++)
                fm1_led_key(i, i == at);
        } else {
            for (i = 0; i < 14; i++)
                fm1_led_key(i, (fm1_in.buttons >> i) & 1u);
            for (i = 0; i < 27; i++)
                fm1_led_key(14u + i, (fm1_in.notes >> i) & 1u);
        }
        while (ci_r != ci_w) {
            char c = (char)cdc_in[ci_r % CI_N];
            ci_r++;
            command(c, knob, batt);
        }
        if (now - last_status >= 250u) {
            last_status = now;
            status(knob, batt);
        }
        lcd_rect(232, 232, 8, 8, usb.config ? ((now >> 9) & 1u ? GRN : DIM) : RED);   /* USB + heartbeat */
        (void)shown_enc;
    }
}

void fm1_cstart(void)
{
    uint32_t *s, *d;
    fm1_time_init();
    fm1_wdt_arm(0x0D);
    if (guard.magic != 0x48575431u) {
        guard.magic = 0x48575431u;
        guard.failed = 0;
        guard.pending = 0;
    }
    if (guard.pending)
        guard.failed++;
    guard.pending = 1;
    if (guard.failed >= 2u) {
        guard.failed = 0;
        guard.pending = 0;
        fm1_enter_uboot();
    }
    fm1_irq_init();
    for (d = _bss_start; d < _bss_end; d++)
        *d = 0;
    for (s = _data_load, d = _data_start; d < _data_end; s++, d++)
        *d = *s;
    fm1_guard_enable(FM1_GUARD_STACK | FM1_GUARD_WRITE | FM1_GUARD_BUS | FM1_GUARD_PC);
    app_main();
    for (;;)
        ;
}
