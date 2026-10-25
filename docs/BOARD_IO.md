# FM-1 panel and audio contracts

Both drivers are header implementations. Include them in one translation unit,
like the other board headers, with `-Ichip -Iboards/fm1`. Initialize TIMER4 with
`fm1_time_init()` before either driver's startup delays. Applications serialize
access to each peripheral; neither driver supplies locks or a scheduler.

## Panel: `boards/fm1/fm1_lcd.h`

`fm1_lcd_init()` configures the SPI pins and the FM-1's ST7789 panel, waits for
reset and sleep-out, and selects RGB565, RGB colour order, IPS inversion and a
240 by 240 top-left coordinate space. The default SPI divider is 4. Its clock is
LSB / 5, so applications that change the system clock must account for that.
The original `fm1_lcd_hw.h` transport remains available for existing users.

- `fm1_lcd_fill_rect(x, y, w, h, rgb)` fills a rectangle with a native RGB565 value:
  red is `0xF800`, green `0x07E0`, blue `0x001F`.
- `fm1_lcd_blit(x, y, w, h, pixels, pixel_count)` sends contiguous row-major
  `uint16_t` RGB565 pixels. `pixel_count` is the available source element count,
  at least `w * h`. No stride, clipping, scaling or colour conversion is implicit.
- Zero width or height succeeds without touching hardware or reading pixels.
  Nonempty rectangles must fit completely inside the display. Overflowing
  coordinates, out-of-bounds rectangles, null pixel pointers and short sources
  return `-2` before a transfer starts.
- Both calls block until completion. They stage one row in 480 bytes of internal
  RAM, sending the high byte first. The source may be in RAM or XIP and may be
  reused when the call returns. Normal C pointer validity/alignment and the
  supplied source capacity are the caller's responsibility.
- `0` means success. A transport timeout returns `-1`, increments the existing
  `fm1_lcd_timeouts`, disables SPI and releases CS. Reinitialize before retrying;
  the panel may contain a partial rectangle. `fm1_lcd_wait()` in the low-level
  transport now returns this status too; existing callers can still ignore it.

Calls share staging RAM and SPI1. Do not call from nested interrupts or another
core while a transfer is active, or mix raw transport calls with panel operations
without the same serialization. The driver initializes the backlight on, as the
original transport did. `fm1_lcd_backlight(on)` still controls it; applications
using TIMER5 key scanning must protect this shared port-A read/modify/write
against that ISR, as documented in `fm1_lcd_hw.h`.

## Audio: `boards/fm1/fm1_audio.h`

### Fixed sample format and clock

Every frame is two `int32_t` words in left, right order. Samples are signed Q23:
`-8388608` through `8388607`, sign-extended into the 32-bit container, with zero
for silence. The significant bits occupy bits 23:0, **not** bits 31:8. For
example, positive full scale is `0x007FFFFF`, negative full scale is `0xFF800000`.
A signed PCM16 sample becomes this format by multiplying by 256. Clip before
storing; the driver does not saturate, convert float/PCM16, or resample.

The supported request is `FM1_AUDIO_RATE` (44100), the **nominal** 44.1 kHz family.
On the recognized stock audio PLL configuration, the hardware frame period is
544 ticks of the 24 MHz reference: `FM1_AUDIO_RATE_NUM / FM1_AUDIO_RATE_DEN`, or
about **44,117.647 Hz**, roughly +400 ppm relative to exact 44,100. Reference
oscillator tolerance applies too. A 256-frame half lasts about 5.803 ms.
Use this physical ratio for deadline calculations; nominal 44100 is appropriate
only when a protocol explicitly requires that advertised rate.

This fixed-divider ratio was corroborated by prior FM-1 DMA-event measurements
against TIMER4 and USB frame timing; it is not a new physical measurement in
this change. The driver's fixed ALNK settings are preserved. Startup now rejects
unrecognized audio PLL register values (`0x119A0 != 0x45400203` or
`0x119A4 != 0x3F503026`) instead of promising a rate under an unknown clock.
It configures the 44.1-family selector/dividers without retuning the shared PLL
or changing the CPU clock. Keep those PLL and ALNK clock settings unchanged
while running. Exact 44,100 Hz and 48 kHz are not supported contracts here.
JieLi describes the peripheral's channel, width and clock roles in its
[official IIS/ALNK documentation](https://doc.zh-jieli.com/AC79/zh-cn/release_v1.2.0/module_example/audio/iis.html).

### Startup and buffer ownership

Use the checked entry point for new applications:

```c
#define HALF_WORDS (256u * FM1_AUDIO_CHANNELS)
static int32_t audio[2u * HALF_WORDS] __attribute__((aligned(4)));
extern void isr_alnk0(void); /* chip/fm1_isr.S -> application fm1_alnk0_irq */

/* CPU0, global interrupts already disabled; TIMER4 and interrupt layer ready */
int rc = fm1_audio_init_checked(audio, sizeof audio / sizeof audio[0],
                                HALF_WORDS, FM1_AUDIO_RATE, isr_alnk0, 3);
/* Check rc before enabling global interrupts or assuming that audio runs. */
```

`half_words` counts 32-bit sample words, **not bytes or stereo frames**. Each half
contains `half_words / 2` frames; the whole DMA span contains `2 * half_words`
words (`8 * half_words` bytes). Requirements:

- Non-null, 4-byte-aligned buffer in ordinary FM-1 SRAM
  `[0x01C00000, 0x01C7FD50)`. The entire DMA span must fit below the reserved
  boot info, mailbox and vectors. Choose memory that does not overlap code,
  stacks or other application allocations.
- Nonzero, even `half_words`, no larger than 65534 (16-bit hardware LEN).
  The SRAM-span check further limits the usable size.
- `buffer_words >= 2 * half_words`, with actual allocated storage for that span.
  A C pointer does not reveal its allocation size: the caller must report its
  real capacity. Validation never probes or writes invalid addresses.
- A non-null assembly ISR wrapper and a priority in 0..7. The driver cannot
  distinguish a valid assembly wrapper from another non-null function pointer.

Validation failures return `FM1_AUDIO_ERR_BUFFER`, `FM1_AUDIO_ERR_RATE`, or
`FM1_AUDIO_ERR_IRQ` before MMIO. A running DMA returns `FM1_AUDIO_ERR_BUSY`;
an unknown PLL returns `FM1_AUDIO_ERR_CLOCK`. Failures leave the registers,
interrupt configuration and sample data unchanged (register reads may occur).
On success, the driver masks the source, clears **both** DMA halves to silence,
resets ALNK, sets the address/length/format/clock, clears stale pending events,
attaches/unmasks the ISR and starts DMA. Call from CPU0 with global interrupts
disabled, outside the audio ISR, and enable interrupts promptly afterward.
Initialization takes about 10 ms of existing startup delays; it does not preserve
prefilled samples. Very small halves may already have pending events on return.

The four-argument `fm1_audio_init(buf, half_words, isr, prio)` remains available
and now returns status. It assumes capacity of at least `2 * half_words` words
and uses `FM1_AUDIO_RATE`; it cannot detect a short allocation. Migrate to the
checked API and inspect the return value, including when an old caller previously
ignored the `void` result. Code storing the old function type must update it.

### Refill, stop/restart and underruns

In `fm1_alnk0_irq`, read `fm1_audio_pending()`, acknowledge auxiliary pending
bits with `fm1_audio_ack_aux(p)`, and service `FM1_AUDIO_HALF`:

```c
uint8_t p = fm1_audio_pending();
fm1_audio_ack_aux(p);
if (p & FM1_AUDIO_HALF) {
    uint32_t half = fm1_audio_free_half();
    /* Fill audio + half * HALF_WORDS, exactly HALF_WORDS words. */
    render(audio + half * HALF_WORDS, HALF_WORDS / FM1_AUDIO_CHANNELS);
    fm1_audio_ack_half();
}
```

The free-half number is a snapshot (0 or 1), valid only while DMA is consuming
the other half. Finish rendering within **one half-buffer period**, including
interrupt latency and higher-priority interrupts. Acknowledgement publishes
sample stores with `csync` and clears the half event; it does not pause DMA or
extend the deadline.

Hardware continuously loops the two halves. If service is delayed, old data
plays again; a late write can overlap DMA reads and produce mixed old/new samples.
There is **no automatic silence, software underrun counter or queued callback**.
Pending bits can coalesce. Comparing the free-half snapshot before and after
rendering can detect some late refills, but misses an even number of transitions;
it cannot prove that no underrun occurred. Applications needing detection should
also measure elapsed time against the physical frame period and track ISR gaps.
On overload, shed work, produce silence **while the half is still free**, or stop
and restart. Do not zero an active DMA buffer from an unrelated context as a
recovery strategy.

`fm1_audio_stop()` is repeatable and must run with global interrupts disabled,
serialized against the ISR. It masks the ALNK interrupt, disables DMA, clears
peripheral pending bits and drives the data GPIO low. It does not free, zero or
retain ownership of the application's allocation, install a new vector, or alter
global interrupt enable state. Do not manually unmask the old source to restart.
After stop returns, the buffer can be replaced/reused. Restart through either
init entry point: the selected buffer's two halves are cleared, DMA position is
reset and the new ISR is attached. Invalid restart arguments leave audio stopped.
This is a fresh start rather than sample-continuous resume. Stop/start electrical
transients and audible pop behavior require physical testing; PC6 is the I2S data
line, not a codec reset or mute pin.

## Verification

Run `tests/run_board_tests.sh` with a C compiler and AddressSanitizer/UBSan. The
panel fixture checks initialization, byte order, windows, a full frame, bounds
and failures at each transfer. The audio fixture checks parameter rejection,
SRAM boundaries and clocks. On Linux it also maps an SRAM-address buffer to test
successful startup, busy rejection, ISR helpers and restart with a different
buffer/size; other hosts explicitly report that fixture portion as skipped.
`tests/lcd_hw_test.c` checks the actual SPI completion and timeout handling.

Build both hwtest images with `examples/hwtest/build.sh` and the JieLi toolchain.
Compile `tests/board_compile.c` with the same include paths to check audio and
pixel-transfer APIs against the target compiler as well. No tests flash hardware.
These checks do not establish electrical timing, colour order on a physical
panel, or audible restart/underrun behavior on a connected FM-1.
