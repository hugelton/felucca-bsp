/* SPDX-License-Identifier: MPL-2.0 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
static uint32_t spi[5], pc_out, pc_dir, pa_out, pa_dir, map;
static uint32_t *reg(uint32_t addr)
{
    switch (addr) {
    case 0x50080u: return &pc_out;
    case 0x50088u: return &pc_dir;
    case 0x50000u: return &pa_out;
    case 0x50008u: return &pa_dir;
    case 0x51020u: return &map;
    default: assert(addr >= 0x11D00u && addr <= 0x11D10u); return &spi[(addr - 0x11D00u) / 4u];
    }
}
#define FM1_LCD_REG(addr) (*reg(addr))
#include "fm1_lcd_hw.h"
int main(void)
{
    static uint8_t pixels[480];
    pc_out = pc_dir = pa_out = pa_dir = 0xFFFFFFFFu;
    fm1_lcd_hw_init();
    assert(spi[0] == 0x4021 && spi[1] == 4);
    assert(pc_out & FM1_LCD_CS);
    assert(!(pc_dir & (FM1_LCD_CS | FM1_LCD_DC | FM1_LCD_CLK | FM1_LCD_DO)));
    assert(!(pa_out & FM1_LCD_BL) && !(pa_dir & FM1_LCD_BL));
    fm1_lcd_send_cmd(0x2C);
    assert(!(pc_out & (FM1_LCD_CS | FM1_LCD_DC)) && spi[2] == 0x2C);
    spi[0] |= 0x8000u;
    assert(fm1_lcd_wait() == 0 && fm1_lcd_timeouts == 0);
    fm1_lcd_deselect();
    assert(pc_out & FM1_LCD_CS);
    fm1_lcd_send_data(pixels, sizeof pixels);
    assert(!(pc_out & FM1_LCD_CS) && (pc_out & FM1_LCD_DC));
    assert(spi[3] == (uint32_t)(uintptr_t)pixels && spi[4] == sizeof pixels);
    spi[0] &= ~0x8000u;
    assert(fm1_lcd_wait() == -1 && fm1_lcd_timeouts == 1 && !(spi[0] & 1));
    fm1_lcd_deselect();
    assert(pc_out & FM1_LCD_CS);
    fm1_lcd_hw_init();
    assert(spi[0] & 1u); /* reinit recovers the disabled controller */
    fm1_lcd_backlight(0);
    assert(pa_out & FM1_LCD_BL);
    fm1_lcd_backlight(1);
    assert(!(pa_out & FM1_LCD_BL));
    puts("LCD transport: pins, DMA address/count, timeout disable and recovery checks passed");
    return 0;
}
