/* SPDX-License-Identifier: MPL-2.0 */
/* Exercise the real panel protocol with a recording SPI transport. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "fm1_lcd_hw.h"
#include "fm1_time.h"

struct event { uint32_t n; uint8_t data[480]; };
static struct event events[1024];
static uint32_t count, waits, selects, deselects, init_calls, baud, delay_ms, fail_wait;
static int selected;
static void record(const void *data, uint32_t n)
{
    assert(count < 1024 && n <= 480);
    events[count].n = n;
    memcpy(events[count++].data, data, n);
    selected = 1;
    selects++;
}
static void command(uint8_t c) { record(&c, 1); }
static int wait_done(void) { return ++waits == fail_wait ? -1 : 0; }
static void deselect(void) { assert(selected); selected = 0; deselects++; }
static void init(void) { init_calls++; }
static void set_baud(uint32_t b) { baud = b; }
static void delay(uint32_t ms) { delay_ms += ms; }
#define fm1_lcd_send_cmd command
#define fm1_lcd_send_data record
#define fm1_lcd_wait wait_done
#define fm1_lcd_deselect deselect
#define fm1_lcd_hw_init init
#define fm1_lcd_baud set_baud
#define fm1_delay_ms delay
#include "fm1_lcd.h"

static void reset(void)
{
    count = waits = selects = deselects = fail_wait = 0;
    selected = 0;
}
static void expect(uint32_t at, const uint8_t *bytes, uint32_t n)
{
    assert(at < count && events[at].n == n && !memcmp(events[at].data, bytes, n));
}
int main(void)
{
    static const uint8_t init_bytes[] = { 0x01, 0x11, 0x3A, 0x55, 0x36, 0, 0x21, 0x13, 0x29 };
    static const uint16_t pixels[] = { 0xF800, 0x07E0, 0x001F, 0xFFFF };
    uint32_t i;
    assert(fm1_lcd_init() == 0);
    assert(init_calls == 1 && baud == 4 && delay_ms == 320 && count == sizeof init_bytes);
    for (i = 0; i < count; i++) expect(i, init_bytes + i, 1);
    assert(!selected && selects == deselects);
    reset();
    assert(fm1_lcd_blit(238, 238, 2, 2, pixels, 4) == 0);
    assert(count == 7);
    expect(0, (uint8_t[]){ 0x2A }, 1);
    expect(1, (uint8_t[]){ 0, 238, 0, 239 }, 4);
    expect(2, (uint8_t[]){ 0x2B }, 1);
    expect(3, (uint8_t[]){ 0, 238, 0, 239 }, 4);
    expect(4, (uint8_t[]){ 0x2C }, 1);
    expect(5, (uint8_t[]){ 0xF8, 0, 0x07, 0xE0 }, 4);
    expect(6, (uint8_t[]){ 0, 0x1F, 0xFF, 0xFF }, 4);
    reset();
    assert(fm1_lcd_fill_rect(0, 0, 240, 240, 0x1234) == 0);
    assert(count == 245);
    for (i = 0; i < 480; i += 2) {
        assert(events[244].data[i] == 0x12);
        assert(events[244].data[i + 1] == 0x34);
    }
    reset();
    assert(fm1_lcd_fill_rect(239, 239, 1, 1, 0x001F) == 0);
    expect(5, (uint8_t[]){ 0, 0x1F }, 2);
    reset();
    assert(fm1_lcd_fill_rect(0, 0, 0, UINT32_MAX, 0) == 0);
    assert(fm1_lcd_blit(0, 0, 0, 1, 0, 0) == 0);
    assert(fm1_lcd_fill_rect(239, 0, 2, 1, 0) == -2);
    assert(fm1_lcd_fill_rect(0, 240, 1, 1, 0) == -2);
    assert(fm1_lcd_fill_rect(UINT32_MAX, 0, 2, 1, 0) == -2);
    assert(fm1_lcd_fill_rect(0, 0, UINT32_MAX, 1, 0) == -2);
    assert(fm1_lcd_blit(0, 0, 2, 2, pixels, 3) == -2);
    assert(fm1_lcd_blit(0, 0, 1, 1, 0, 1) == -2);
    assert(count == 0);
    /* Each command, argument and pixel completion can fail. No later transfer
     * is issued and CS is released even if the failing transfer is a row. */
    for (i = 1; i <= 7; i++) {
        reset(); fail_wait = i;
        assert(fm1_lcd_blit(0, 0, 2, 2, pixels, 4) == -1);
        assert(count == i && !selected && selects == deselects);
    }
    for (i = 1; i <= 9; i++) {
        reset(); fail_wait = i;
        assert(fm1_lcd_init() == -1);
        assert(count == i && !selected);
    }
    puts("LCD: init, bounds, RGB565 wire bytes, full frame and timeout checks passed");
    return 0;
}
