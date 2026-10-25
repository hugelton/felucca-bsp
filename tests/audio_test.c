/* SPDX-License-Identifier: MPL-2.0 */
/* Real audio driver, fake MMIO/IRQs; Linux maps a DMA buffer in FM-1 SRAM's
 * address range so successful startup uses the same validation as the target. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#ifdef __linux__
#include <sys/mman.h>
#endif
/* CPU instructions aren't executable in a host fixture. Target compilation
 * separately checks these instructions; transport and register operations below
 * are recorded. No hardware initialization function from these headers is called. */
#define __asm__
#define volatile(...)
#define __attribute__(...)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "fm1_irq.h"
#pragma GCC diagnostic pop
#undef __attribute__
#include "fm1_time.h"
#include "fm1_gpio.h"
const char fm1_fatal_stubs[1] = { 0 };
static void fm1_fault(const fm1_crash_t *c) { (void)c; assert(0); }
static uint32_t regs[12], pll0, pll1, clk, map, ports[8][8];
static uint32_t accesses, masks, attaches, delays;
static void isr(void) {}
static void *reg(uint32_t addr)
{
    accesses++;
    if (addr == 0x119A0u) return &pll0;
    if (addr == 0x119A4u) return &pll1;
    if (addr == 0x10014u) return &clk;
    if (addr == 0x51030u) return &map;
    assert(addr >= 0x12E00u && addr < 0x12E30u);
    return &regs[(addr - 0x12E00u) / 4];
}
static void mask(uint32_t n) { assert(n == FM1_IRQ_ALNK0); masks++; }
static void attach(uint32_t n, void (*handler)(void), uint32_t prio)
{
    assert(n == FM1_IRQ_ALNK0 && handler == isr && prio <= 7);
    assert(masks && !(regs[0] & 0x800u));
    attaches++;
}
static void delay(uint32_t ms) { delays += ms; }
#undef FM1_PR
#define FM1_PR(p,r) ports[p][(r)/4]
#define FM1_AUDIO_REG(type,addr) (*(type *)reg(addr))
#define fm1_irq_mask mask
#define fm1_irq_attach attach
#define fm1_delay_ms delay
#include "fm1_audio.h"
#undef __asm__
#undef volatile

static int32_t *ptr(uintptr_t addr) { return (int32_t *)addr; }
static void reset(void)
{
    memset(regs, 0, sizeof regs);
    memset(ports, 0, sizeof ports);
    pll0 = 0x45400203u; pll1 = 0x3F503026u;
    clk = 0xABCDEF06u; map = 0xFFFFFFFFu;
    accesses = masks = attaches = delays = 0;
}
static void validate(void)
{
    assert(FM1_AUDIO_SAMPLE_BITS == 24 && FM1_AUDIO_CHANNELS == 2);
    assert(FM1_AUDIO_SAMPLE_MIN == -8388608 && FM1_AUDIO_SAMPLE_MAX == 8388607);
    assert(FM1_AUDIO_RATE == 44100 && FM1_AUDIO_RATE_NUM == 24000000 && FM1_AUDIO_RATE_DEN == 544);
    assert(fm1_audio_buffer_valid(ptr(0x1C00000), 4, 2));
    assert(fm1_audio_buffer_valid(ptr(0x1C00000), 130900, 65450));
    assert(!fm1_audio_buffer_valid(ptr(0x1C00000), 131068, 65534)); /* overlaps reserved SRAM */
    assert(fm1_audio_buffer_valid(ptr(0x1C7FD40), 4, 2));
    assert(!fm1_audio_buffer_valid(ptr(0x1C7FD44), 4, 2));
    assert(!fm1_audio_buffer_valid(ptr(0x1C00000), 3, 2));
    assert(!fm1_audio_buffer_valid(ptr(0x1C00000), UINT32_MAX, 0));
    assert(!fm1_audio_buffer_valid(ptr(0x1C00000), UINT32_MAX, 3));
    assert(!fm1_audio_buffer_valid(ptr(0x1C00000), UINT32_MAX, 65536));
    assert(!fm1_audio_buffer_valid(ptr(0x1C00000), UINT32_MAX, UINT32_MAX));
    assert(!fm1_audio_buffer_valid(ptr(0x1C00002), 4, 2));
    assert(!fm1_audio_buffer_valid(ptr(0x1BFFFFC), 4, 2));
    assert(!fm1_audio_buffer_valid(ptr(0x2000000), 4, 2));
    assert(!fm1_audio_buffer_valid(ptr(UINTPTR_MAX - 3u), 4, 2));
    assert(!fm1_audio_buffer_valid(0, 4, 2));
    reset();
    assert(fm1_audio_init_checked(0, 4, 2, 44100, isr, 3) == FM1_AUDIO_ERR_BUFFER);
    assert(fm1_audio_init_checked(ptr(0x1C00000), 3, 2, 44100, isr, 3) == FM1_AUDIO_ERR_BUFFER);
    assert(fm1_audio_init(ptr(0x1C00000), UINT32_MAX, isr, 3) == FM1_AUDIO_ERR_BUFFER);
    assert(fm1_audio_init_checked(ptr(0x1C00000), 4, 2, 48000, isr, 3) == FM1_AUDIO_ERR_RATE);
    assert(fm1_audio_init_checked(ptr(0x1C00000), 4, 2, 44100, 0, 3) == FM1_AUDIO_ERR_IRQ);
    assert(fm1_audio_init_checked(ptr(0x1C00000), 4, 2, 44100, isr, 8) == FM1_AUDIO_ERR_IRQ);
    assert(accesses == 0 && masks == 0 && attaches == 0);
    pll0 ^= 1u;
    assert(fm1_audio_init(ptr(0x1C00000), 2, isr, 3) == FM1_AUDIO_ERR_CLOCK);
    assert(masks == 0 && attaches == 0 && delays == 0 && !regs[0]);
    reset(); pll1 ^= 1u;
    assert(fm1_audio_init(ptr(0x1C00000), 2, isr, 3) == FM1_AUDIO_ERR_CLOCK);
    assert(masks == 0 && attaches == 0 && delays == 0 && !regs[0]);
}
int main(void)
{
    validate();
#ifdef __linux__
    uint32_t i;
    int32_t *samples = mmap(ptr(0x1C08000), 4096, PROT_READ | PROT_WRITE,
                            MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    assert(samples == ptr(0x1C08000));
    reset();
    for (i = 0; i < 256; i++) samples[i] = 0x123456;
    assert(fm1_audio_init_checked(samples, 256, 128, 44100, isr, 3) == FM1_AUDIO_OK);
    for (i = 0; i < 256; i++) assert(samples[i] == 0);
    assert(regs[0] & 0x800u);
    assert(regs[0] == 0x980u && regs[1] == 0x5000u && regs[3] == 0x83u); /* stereo 24-bit I2S */
    assert(regs[0x20 / 4] == 128 && regs[0x1C / 4] == (uint32_t)(uintptr_t)samples);
    assert(attaches == 1 && masks == 1 && delays == 10);
    assert(clk == (0xABCDEF06u & 0xFFFFF0FFu) && map == (0xFFFFFFFFu & ~0xC0u));
    samples[0] = FM1_AUDIO_SAMPLE_MAX;
    assert(fm1_audio_init(samples, 128, isr, 3) == FM1_AUDIO_ERR_BUSY);
    assert(samples[0] == FM1_AUDIO_SAMPLE_MAX && attaches == 1 && masks == 1);
    assert(fm1_audio_free_half() == 1);
    regs[0] |= 0x8000;
    assert(fm1_audio_free_half() == 0);
    regs[2] = FM1_AUDIO_HALF;
    assert(fm1_audio_pending() == FM1_AUDIO_HALF);
    fm1_audio_ack_half();
    assert(regs[2] & 8u);
    fm1_audio_ack_aux(0x70);
    assert((regs[2] & 7u) == 7u);
    fm1_audio_stop();
    assert(!(regs[0] & 0x800u) && regs[2] == 0x0F && masks == 2);
    assert(!(ports[FM1_PC][FM1_OUT / 4] & (1u << 6)));
    fm1_audio_stop();
    assert(!(regs[0] & 0x800u) && masks == 3);
    /* Restart with a new buffer and block size, leaving old application data
     * intact. Only the new active DMA span is zeroed. */
    samples[128] = 77;
    assert(fm1_audio_init_checked(samples + 64, 64, 32, 44100, isr, 7) == FM1_AUDIO_OK);
    assert(regs[0x20 / 4] == 32 && regs[0x1C / 4] == (uint32_t)(uintptr_t)(samples + 64));
    assert(samples[0] == FM1_AUDIO_SAMPLE_MAX && samples[128] == 77);
    for (i = 64; i < 128; i++) assert(samples[i] == 0);
    assert(fm1_audio_free_half() == 1 && attaches == 2);
    fm1_audio_stop();
    assert(munmap(samples, 4096) == 0);
    puts("Audio: buffer/rate/clock rejection, startup, IRQs and stop/restart checks passed");
#else
    puts("Audio: validation checks passed; startup/stop/restart fixture requires Linux mmap");
#endif
    return 0;
}
