/* SPDX-License-Identifier: MPL-2.0 */
/* Compile-only target fixture: hwtest intentionally doesn't enable audio. */
#include "fm1_audio.h"
#include "fm1_lcd.h"
static void fm1_fault(const fm1_crash_t *c) { (void)c; for (;;) {} }
extern void isr_alnk0(void);
static int32_t audio[1024] __attribute__((aligned(4)));
int board_compile_start(void)
{
    return fm1_audio_init_checked(audio, 1024, 512, FM1_AUDIO_RATE, isr_alnk0, 3);
}
void board_compile_audio(void)
{
    uint8_t p = fm1_audio_pending();
    fm1_audio_ack_aux(p);
    if (p & FM1_AUDIO_HALF) {
        audio[fm1_audio_free_half() * 512u] = FM1_AUDIO_SAMPLE_MAX;
        fm1_audio_ack_half();
    }
    fm1_audio_stop();
}
int board_compile_pixels(const uint16_t *pixels)
{
    return fm1_lcd_blit(0, 0, 240, 240, pixels, 240u * 240u);
}
