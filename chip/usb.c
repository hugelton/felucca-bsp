/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio) — USB host integration
 * MIDI clock and transport sync: keremimo (Felucca 019e2ad), under MPL-2.0 by agreement: https://github.com/hugelton/felucca-bsp/pull/3
 * Copyright (C) 2026 Julius Vitkauskas — disk mode at the host's pace (usb_msc_pump, the INDEX cache) */
/* USB full-speed device on USB0: a
 * class-compliant USB-MIDI interface. All SIE traffic happens in one place,
 * usb_poll(), called from the TIMER5 ISR at 2 kHz, so INDEX is never shared
 * and no USB IRQ is needed; one exception: disk mode (FELUCCA_MSC) also moves
 * EP3's packets from the main loop (usb_msc_pump), with the interrupts off,
 * and INDEX is then written only when it changes (sie_index). MIDI in goes to a ring the audio ISR drains;
 * MIDI out (the panel keys) comes from a ring the audio ISR fills.
 * SysEx F0 22 24 35 7D F7 (the stock soft key, tools/fm1_softkey.py) asks
 * the main loop to enter UBOOT. VID 0x1209 / PID 0x0001 is the pid.codes
 * TEST id: fine for the bench, must be replaced before any release.
 * FELUCCA_CDC=1 adds a CDC-ACM serial function (IAD composite: EP2 notify,
 * EP3 bulk data) for the console in console.c; FELUCCA_USB_LAYOUT picks how
 * the composite presents itself, usb_cdc_switch / FELUCCA_CDC_DEFAULT=0 leave
 * the console out (descriptors, below).
 * FELUCCA_UAC=1 adds a USB audio input (UAC1, class compliant): the master
 * output (the 24-bit samples the I2S takes) as 16-bit stereo at 44.1 kHz on an isochronous asynchronous IN
 * endpoint (EP4), so a computer can record the FM-1 over the cable. audio.c
 * fills a ring (uac_render_start, uac_tap); uac_service sends one packet per
 * USB frame from the TIMER5 ISR, nested in the render too (main.c). The
 * update loader leaves both off: MIDI only.
 * FELUCCA_UAC_48K=1 (the default with FELUCCA_UAC) offers 48 kHz too: the host picks the rate with
 * SET_CUR (UAC1 sampling frequency control, two discrete rates in the one alt setting). The engines stay
 * at 44.1 kHz; at 48 kHz uac_tap resamples (polyphase FIR 160 / 147, uac_fir.h) into the ring. 44.1 kHz,
 * the rate until a host asks for another, sends the samples as before, bit for bit.
 * FELUCCA_UAC_OUT=1 (with FELUCCA_UAC) adds a USB audio output too: the computer plays through the FM-1 (16-bit
 * stereo, 44.1 kHz, EP4 OUT isochronous adaptive). uac_service takes each packet into a ring (nested in the render
 * too); uac_play, after each render, adds it to the output, resampled to the I2S clock (~44,117.6 Hz): the ring's
 * fill steers the rate, so it neither runs dry nor overflows. uac_play_take instead hands the same samples to the
 * caller (unity, written not added) for a firmware that mixes the computer's audio itself.
 * FELUCCA_UAC_OUT_FEEDBACK=1 (with FELUCCA_UAC_OUT, without FELUCCA_CDC) makes the output asynchronous: the computer
 * sends at the I2S's own rate, which EP3 IN tells it (explicit feedback, 10.14 frames a millisecond, from the ring's
 * fill), so the samples play as sent, one for one, nothing resampled. Off (the default), nothing changes.
 * FELUCCA_UAC_OUT_24=1 (with FELUCCA_UAC_OUT) adds alt setting 2 of the output stream, 24-bit (3-byte subframes; alt 1
 * stays 16-bit, the host picks): the ring keeps 24 bits (and alt 1's 16 as 24), so the computer's audio reaches the
 * render whole. Off (the default), nothing changes.
 * FELUCCA_UAC_SPLIT=1 (with FELUCCA_UAC, without FELUCCA_CDC) presents the audio input and the audio output as two audio
 * functions of their own beside MIDI's (three IADs: MIDI IF 0-1, the input IF 2-3, the output IF 4-5), each named
 * (FELUCCA_UAC_IN_NAME, FELUCCA_UAC_OUT_NAME: the product's name + " In" / " Out" by default), its channels named
 * too (FELUCCA_UAC_CH_NAMES, a list of strings, one a channel). A computer lists them as two devices, each on its
 * own clock (macOS times a duplex device from its input's packets: a late one there cost the output), and no audio
 * streaming interface shares an audio control interface's collection with MIDI (macOS 13..15's kernel driver
 * stops at MIDI there: FELUCCA_UAC_AS_FIRST's case). Off (the default), nothing changes.
 * FELUCCA_UAC_CH=n (1..8, default 2) makes the input n channels: the caller's uac_tap gives n samples a frame (the
 * firmware's own stems, say); 2 keeps the stereo master and today's descriptors. FELUCCA_UAC_24=1 adds 24-bit: alt
 * setting 2 of the input stream (alt 1 stays 16-bit), the host picks either. uac_capturing says whether the host takes
 * the input this render (the caller skips its stems otherwise). Neither changes anything when left at its default.
 * FELUCCA_UAC_HOST_CLOCK=1 puts the input on the computer's clock: uac_tap resamples the I2S's frames to the USB
 * frames' 44.1 kHz (a 16-tap fractional-delay filter, uac_hc_fir.h), its ratio steered by the ring's fill, and every
 * packet is the plain 44.1 pattern; the endpoint is synchronous. Off (the default), nothing changes (below).
 * FELUCCA_USB_HOST=1 adds opt-in MIDI host mode; see docs/USB_HOST.md for the main-loop contract.
 * FELUCCA_MSC=1 adds a disk mode: usb_msc_switch(1) drops off the bus and comes back as a USB mass-storage device
 * (bulk-only transport, EP3 bulk in and out; nothing else: no MIDI, no audio) until usb_msc_switch(0). The poll
 * only moves packets: the commands (SCSI) and the disk are the main loop's (usb_msc_read, usb_msc_send).
 * FELUCCA_MIDI_RT=1 adds usb_midi_rt: MIDI clock ticks and transport sent to the host at once, from the firmware's own
 * fast tick (usb_midi_rt_kick, below). Off by default: the ring and the kick in usb_poll are not built.
 * FELUCCA_MIDI_IN_RAW, a function of the firmware's (void f(uint32_t pkt)), is given every EP1 OUT event packet as
 * it came, for a MIDI parser of its own (midi_in_event). */
#include "fm1_usb.h"   /* registers (chip/) */
/* Host is opt-in and excluded from update loaders, even with a global build flag. */
#ifndef FELUCCA_USB_HOST
#define FELUCCA_USB_HOST 0
#endif
#ifdef FELUCCA_LOADER
#undef FELUCCA_USB_HOST
#define FELUCCA_USB_HOST 0
#endif
/* FELUCCA_USB_HOST_NAME=1 (with the host): the attached device's product string, read once at enumeration
 * (usb_host_name). Off by default: the host's code and its requests stay as they were. */
#ifndef FELUCCA_USB_HOST_NAME
#define FELUCCA_USB_HOST_NAME 0
#endif
#ifndef FELUCCA_CDC
#define FELUCCA_CDC 0
#endif
#ifndef FELUCCA_MSC
#define FELUCCA_MSC 0
#endif
#ifdef FELUCCA_LOADER
#undef FELUCCA_MSC
#define FELUCCA_MSC 0
#endif
#ifndef FELUCCA_UAC
#define FELUCCA_UAC 0
#endif
#ifndef FELUCCA_UAC_48K
#define FELUCCA_UAC_48K 1
#endif
#ifndef FELUCCA_UAC_OUT
#define FELUCCA_UAC_OUT 0
#endif
#ifndef FELUCCA_UAC_AS_FIRST
#define FELUCCA_UAC_AS_FIRST 0
#endif
#ifndef FELUCCA_UAC_CH
#define FELUCCA_UAC_CH 2
#endif
/* FELUCCA_UAC_CAPTURE=1: the caller fills every channel (uac_tap's frames: its own routing); 0: the master on 1 and 2,
 * the caller's on 3..N by uac_ch_frame */
#ifndef FELUCCA_UAC_CAPTURE
#define FELUCCA_UAC_CAPTURE 0
#endif
#ifndef FELUCCA_UAC_24
#define FELUCCA_UAC_24 0
#endif
#ifndef FELUCCA_UAC_HOST_CLOCK
#define FELUCCA_UAC_HOST_CLOCK 0
#endif
#ifndef FELUCCA_UAC_OUT_FEEDBACK
#define FELUCCA_UAC_OUT_FEEDBACK 0
#endif
#ifndef FELUCCA_UAC_OUT_24
#define FELUCCA_UAC_OUT_24 0
#endif
#ifndef FELUCCA_UAC_SPLIT
#define FELUCCA_UAC_SPLIT 0
#endif
#ifndef FELUCCA_UAC_MIDI_LAST
#define FELUCCA_UAC_MIDI_LAST 0
#endif
#ifndef FELUCCA_UAC_IAD
#define FELUCCA_UAC_IAD 0
#endif
#if FELUCCA_UAC_MIDI_LAST < 0 || FELUCCA_UAC_MIDI_LAST > 2
#error "FELUCCA_UAC_MIDI_LAST is 0..2"
#endif
#if !FELUCCA_UAC
#undef FELUCCA_UAC_SPLIT
#define FELUCCA_UAC_SPLIT 0
#endif
#if !FELUCCA_UAC || !FELUCCA_UAC_OUT
#undef FELUCCA_UAC_OUT_FEEDBACK
#define FELUCCA_UAC_OUT_FEEDBACK 0
#undef FELUCCA_UAC_OUT_24
#define FELUCCA_UAC_OUT_24 0
#endif
#if !FELUCCA_UAC
#undef FELUCCA_UAC_HOST_CLOCK
#define FELUCCA_UAC_HOST_CLOCK 0
#undef FELUCCA_UAC_48K
#define FELUCCA_UAC_48K 0
#undef FELUCCA_UAC_OUT
#define FELUCCA_UAC_OUT 0
#undef FELUCCA_UAC_AS_FIRST
#define FELUCCA_UAC_AS_FIRST 0
#undef FELUCCA_UAC_MIDI_LAST
#define FELUCCA_UAC_MIDI_LAST 0
#undef FELUCCA_UAC_IAD
#define FELUCCA_UAC_IAD 0
#undef FELUCCA_UAC_CH
#define FELUCCA_UAC_CH 2
#undef FELUCCA_UAC_24
#define FELUCCA_UAC_24 0
#endif
#if FELUCCA_UAC_CAPTURE && FELUCCA_UAC_CH != 2 && FELUCCA_UAC_CH != 4 && FELUCCA_UAC_CH != 8
#error "FELUCCA_UAC_CAPTURE: FELUCCA_UAC_CH is 2, 4 or 8"
#endif
/* the input of FELUCCA_UAC_CAPTURE in bcdDevice (high byte, x 0x10): 2, 4 and 8 channels are three descriptors */
#define UA_CAPCODE (FELUCCA_UAC && FELUCCA_UAC_CAPTURE ? (FELUCCA_UAC_CH == 8 ? 3 : FELUCCA_UAC_CH == 4 ? 2 : 1) : 0)
#if FELUCCA_UAC_48K && ((FELUCCA_UAC_CAPTURE && FELUCCA_UAC_CH != 2) || FELUCCA_UAC_24)
#error "FELUCCA_UAC_48K resamples stereo 16-bit only: set it to 0 with FELUCCA_UAC_CH != 2 or FELUCCA_UAC_24"
#endif
#if FELUCCA_UAC_HOST_CLOCK && FELUCCA_UAC_48K
#error "FELUCCA_UAC_HOST_CLOCK resamples to 44.1 kHz only: set FELUCCA_UAC_48K to 0 with it"
#endif
#if FELUCCA_UAC_OUT_FEEDBACK && FELUCCA_CDC
#error "FELUCCA_UAC_OUT_FEEDBACK sends its feedback on EP3 IN, the console's: set FELUCCA_CDC to 0 with it"
#endif
#if FELUCCA_UAC_SPLIT && FELUCCA_CDC
#error "FELUCCA_UAC_SPLIT presents the audio functions without the console: set FELUCCA_CDC to 0 with it"
#endif
#if FELUCCA_UAC_SPLIT && (FELUCCA_UAC_MIDI_LAST || FELUCCA_UAC_IAD)
#error "FELUCCA_UAC_SPLIT has its own interface order and IADs: set FELUCCA_UAC_MIDI_LAST and FELUCCA_UAC_IAD to 0 with it"
#endif
#if FELUCCA_UAC_MIDI_LAST                 /* the collection follows the interfaces: the streams come first anyway */
#undef FELUCCA_UAC_AS_FIRST
#define FELUCCA_UAC_AS_FIRST 0
#endif
/* FELUCCA_UAC_CH=N (4..10, even, with FELUCCA_UAC; 2, the default, is the stereo input above, as it was): the input carries
 * N channels of 16 bits, the master on 1 and 2, the app's own on 3..N (uac_ch_frame: written while the render runs,
 * sent with the master's frames). 44.1 kHz only (no FELUCCA_UAC_48K: resampling N channels costs too much); a packet
 * of 46 frames is 92 N bytes, 920 at 10, under full speed's 1023. bcdDevice +0.20 (FELUCCA_UAC_48K's is then off) */
#ifndef FELUCCA_UAC_CH
#define FELUCCA_UAC_CH 2
#endif
#if !FELUCCA_UAC
#undef FELUCCA_UAC_CH
#define FELUCCA_UAC_CH 2
#endif
#if FELUCCA_UAC_CH < 2 || FELUCCA_UAC_CH > 10 || (FELUCCA_UAC_CH & 1)
#error "FELUCCA_UAC_CH is 2, 4, 6, 8 or 10"
#endif
#if FELUCCA_UAC_CH > 2 && !FELUCCA_UAC_CAPTURE
#define UA_MC 1
#undef FELUCCA_UAC_48K
#define FELUCCA_UAC_48K 0
#else
#define UA_MC 0
#endif
#if UA_MC && (FELUCCA_UAC_24 || FELUCCA_UAC_HOST_CLOCK)
#error "FELUCCA_UAC_24 and FELUCCA_UAC_HOST_CLOCK with more than 2 channels: FELUCCA_UAC_CAPTURE 1"
#endif
#ifndef FELUCCA_UAC_MEM
#define FELUCCA_UAC_MEM 0
#endif
#if UA_MC && FELUCCA_UAC_MEM
#error "FELUCCA_UAC_MEM lends the ring of the stereo or FELUCCA_UAC_CAPTURE input: not with FELUCCA_UAC_CH 4..10 and FELUCCA_UAC_CAPTURE 0 (the master and the app's channels: a ring of its own)"
#endif
/* With none of the options that change the input (more than 2 channels, FELUCCA_UAC_CAPTURE, 24-bit, the computer's clock,
 * a lent ring) the
 * input is the code it was, byte for byte, and with none that change the output (24-bit, the feedback, a lent ring) the
 * output is: UA_IN_PLAIN, UA_OUT_PLAIN pick it in each place below (the general code follows it, in the #else) */
#define UA_IN_PLAIN (!UA_MC && FELUCCA_UAC_CH == 2 && !FELUCCA_UAC_CAPTURE && !FELUCCA_UAC_24 && !FELUCCA_UAC_HOST_CLOCK && !FELUCCA_UAC_MEM)
#define UA_OUT_PLAIN (!FELUCCA_UAC_OUT_24 && !FELUCCA_UAC_OUT_FEEDBACK && !FELUCCA_UAC_MEM)
enum { S_FADDR = 0, S_POWER = 1, S_INTRTX1 = 2, S_INTRTX2 = 3, S_INTRRX1 = 4, S_INTRRX2 = 5, S_INTRUSB = 6,
       S_INTRTX1E = 7, S_INTRTX2E = 8, S_INTRRX1E = 9, S_INTRRX2E = 10, S_INTRUSBE = 11, S_FRAME1 = 12,
       S_FRAME2 = 13, S_INDEX = 14,
       S_TXMAXP = 16, S_CSR0 = 17, S_TXCSR1 = 17, S_TXCSR2 = 18, S_RXMAXP = 19, S_RXCSR1 = 20, S_RXCSR2 = 21,
       S_COUNT0 = 22, S_RXCOUNT1 = 22, S_RXCOUNT2 = 23 };

static uint8_t ep0buf[64 + 4] __attribute__((aligned(4)));
static uint8_t ep1tx[64] __attribute__((aligned(4)));
static uint8_t ep1rx[64 + 4] __attribute__((aligned(4)));
#if FELUCCA_CDC || FELUCCA_MSC
static uint8_t ep3tx[64] __attribute__((aligned(4)));
static uint8_t ep3rx[64 + 4] __attribute__((aligned(4)));
#endif
#if FELUCCA_MSC
/* disk mode: the host's bulk OUT bytes in a ring (the poll writes, the main loop reads); one segment going out at a
 * time (the main loop's buffer, sent in 64-byte packets: a segment never shares a packet with the next) */
#define MSI_N 2048u
static uint8_t msc_in[MSI_N];
static volatile uint32_t msi_w, msi_r;
static const uint8_t *volatile msc_tx_p;
static volatile uint32_t msc_tx_n;               /* bytes of the segment still to send */
static volatile uint8_t msc_rx_pend, msc_resets;
static uint8_t usb_msc_on;                       /* presented as a disk (read by the TIMER5 ISR) */
#define USB_MSC_ON usb_msc_on
#else
#define USB_MSC_ON 0
#endif
#if FELUCCA_CDC
static uint8_t ep2tx[8] __attribute__((aligned(4)));
/* serial rings: in = host -> console (ISR writes), out = console -> host (ISR reads) */
#define CI_N 256u
#define CO_N 2048u
static uint8_t cdc_in[CI_N], cdc_out[CO_N];
static volatile uint32_t ci_w, ci_r, co_w, co_r;
static struct {
    uint8_t line[7];             /* line coding: just stored and echoed back */
    volatile uint8_t dtr;        /* host has the port open */
    uint8_t e0_rx;               /* SET_LINE_CODING data stage pending */
    uint8_t rx_pend;             /* EP3 OUT packet seen, not yet taken (the ring was full) */
    uint32_t rx_pkts, tx_pkts;
} cdc = {{0x00, 0xC2, 0x01, 0x00, 0, 0, 8}};   /* 115200 8N1 */
#endif

#if FELUCCA_UAC
/* USB audio input. The I2S clock is not locked to USB (about 44,117.6 Hz against the host's 44,100):
 * the endpoint is asynchronous, so the host takes the device's rate from the packets' sizes, and they
 * follow the FM-1's own clock (below: uac_size), not the ring. The render writes HALF_FRAMES frames every
 * half; the packets take them out at the clock's rate, so the ring's fill only moves with when the renders
 * land: it starts at UA_PRIME and needs no steering. A frame waits in the ring ~3..6 ms. */
/* The ring is written when a render ends, so it must last through the render: the priming holds 2.9 ms with
 * 256-frame halves, enough while a render takes under ~40 % of its time. FELUCCA_UAC_DEEP=1 (an app whose renders
 * run longer) keeps a whole half more in hand: a 1024-frame ring (4 KB, FELUCCA_UAC_RING_ATTR may place it in
 * another section), the priming a half higher (+5.8 ms of latency), so a render may take its whole time. Without
 * it a long render runs the ring dry: the packets hold their last frame (a hold, heard as a crackle in the
 * recording, not on the FM-1's own output), and the frames in hand grow by the hold, so after the first few
 * (measured with renders at 60-90 %: 5 in the first 10 s, one in the next 3 minutes) the renders are in time. */
#ifndef FELUCCA_UAC_DEEP
#define FELUCCA_UAC_DEEP 0
#endif
/* the polls (uac_service calls) a queued packet may wait before the host counts as gone (20 ms at the poll's 2 kHz).
 * A caller that services the stream more often says how many make 20 ms: FELUCCA_UAC_CH's packets (~0.6 ms on the
 * bus at 10 channels) are refilled every 0.1 ms (fm1-base's 10 kHz tick: 200), or the next frame's IN finds none */
#ifndef FELUCCA_UAC_STALL_POLLS
#define FELUCCA_UAC_STALL_POLLS 40u
#endif
#if FELUCCA_UAC_STALL_POLLS > 255
#error "FELUCCA_UAC_STALL_POLLS is 1..255 (uac.wait is a byte)"
#endif
#ifndef FELUCCA_UAC_RING_ATTR
#define FELUCCA_UAC_RING_ATTR
#endif
#if FELUCCA_UAC_DEEP
#define UA_N 1024u               /* ring, frames (UA_CH samples each: 16-bit, L then R); a power of two */
#define UA_DEEP HALF_FRAMES
#else
#define UA_N 512u
#define UA_DEEP 0u
#endif
/* UA_PRIME: the silence ahead of the first rendered frame, the frames in hand when a render starts (a render may
 * land late: keep a packet and more). UA_HI: the most the ring holds then, for its size. A render landing later than
 * that runs the ring dry once (a hold, uac_packet), and the frames in hand grow by as much: the next is in time */
#if HALF_FRAMES > 128u           /* 256-frame halves: as X0X measured them */
#define UA_PRIME (128u + UA_DEEP)
#define UA_HI (160u + UA_DEEP)
#else
#define UA_PRIME (104u + UA_DEEP)
#define UA_HI (128u + UA_DEEP)
#endif
#define UAC_RATE 44100u
#if FELUCCA_UAC_48K
/* 48 kHz: the same times in 48 kHz frames (x 160 / 147); a render of HALF_FRAMES gives 139..140 frames */
#define UA_REND48 ((HALF_FRAMES * 160u + 146u) / 147u)
#define UA_DEEP48 (UA_DEEP ? UA_REND48 : 0u)
#define UA_PRIME48 (113u + UA_DEEP48)
#define UA_HI48 (139u + UA_DEEP48)
#define UA_MAXF 49u              /* frames in the largest packet: 48 + 1 */
_Static_assert(UA_HI48 + UA_REND48 + UA_MAXF < UA_N, "the ring holds the band, a render and a packet (48 kHz)");
#else
#define UA_MAXF 46u              /* frames in the largest packet */
#endif
#define UA_CH ((uint32_t)FELUCCA_UAC_CH)
/* The packets follow the I2S clock (the 24 MHz crystal / 544 = 44,117.647 Hz; resampled x 160 / 147 at 48 kHz,
 * 48,019.21 Hz) as an asynchronous device's clock: a steady pattern, 44 frames a ms and one more every ~8.5 ms (48
 * and one more every ~52 ms at 48 kHz), its rate trimmed to the host's ms as the FM-1's crystal counts it (uac_size),
 * and a frame more or less at most every UA_GAP packets while the frames sent part from the frames the crystal
 * counted (FELUCCA_UAC_HOST_CLOCK: the pattern alone, the input resampled to the host's clock). A host derives the device's rate from those sizes (macOS: the HAL's clock for the device, which apps that
 * resample to their own clock follow), so they must not follow the ring: steering by the ring's fill (a frame a
 * render while it left a band) sent a frame more every render for as long as late renders held the fill measured at
 * a render's start high, +0.39 % for minutes, and the ring ran dry (repeated frames) every render: macOS's clock for
 * the FM-1 at 44,292 Hz, a recording through Voice Memos or an aggregate device warbled and ticked. A host that
 * follows the device's rate closely (iOS's Camera) fills a run of short packets with silence: never two corrections
 * in a row (the old steering's 5..6 short or long ones in a row were heard there as dropouts) */
#define UA_ACC_ONE 100000u
#if FELUCCA_UAC_HOST_CLOCK
#define UA_FRAC44 10000u                         /* (on the host's clock the strict 44.1: the input is resampled to it) */
#else
#define UA_FRAC44 11765u
#endif
#define UA_FRAC48 1921u
/* the FM-1's clock: a free-running 32-bit count, UA_TPF of its ticks an I2S frame (TIMER4 counts the crystal that
 * also drives the audio PLL: 24 MHz / 544 exactly; measured on an FM-1 against the renders, within 1 ppm). fm1_ticks
 * is fm1_time.h's, which the includer has (as FM1_TICKS_PER_US for FELUCCA_MIDI_STAMP) */
#ifndef FELUCCA_UAC_CLOCK
#define FELUCCA_UAC_CLOCK() fm1_ticks()
#endif
#define UA_TPF 544u
#define UA_TPF48 2499u           /* 48 kHz: 5 frames every 2499 ticks (x 160 / 147: 24 MHz x 160 / (147 x 544)) */
/* the loop (uac_size): the frames the clock counted less the frames sent (the phase) is read at each packet, at the
 * first poll after the host took the last one, so it carries that poll's lateness: up to UA_Q frames, a saw tooth
 * as the poll's grid (the crystal's) slides against the host's ms. Its least over UA_WIN packets (a held-up poll
 * only adds) against the first window's: past UA_BAND, a frame more or less, at most one every UA_GAP packets
 * (+-180 ppm: the crystals are tens of ppm apart), and each nudges the pattern's rate (UA_TRIM_STEP: ~5.7 ppm) so the
 * pattern alone carries the host's ms in the end; past UA_JUMP (the host missed a packet, a poll held up for ms) the
 * phase is taken as it is. FELUCCA_UAC_STALL_POLLS polls make 20 ms: UA_Q from it */
#define UA_Q ((960u + FELUCCA_UAC_STALL_POLLS - 1u) / FELUCCA_UAC_STALL_POLLS)   /* 2 kHz: 24, 10 kHz: 5 */
#define UA_BAND ((int32_t)(UA_Q / 2u + 3u))
#define UA_JUMP ((int32_t)(UA_Q + UA_Q / 2u + 3u + 8u))
#define UA_WIN 16u
#define UA_GAP 128u
#define UA_TRIM_STEP 25
#define UA_TRIM_MAX 2000         /* 1/UA_ACC_ONE frames a ms: +-450 ppm */
#define UA_DT_MAX (24000u * 16u) /* a packet more than 16 ms after the last: the loop starts again */
#define UA_HOLD 32u              /* the frames a hold adds past the ring's running dry (uac_packet) */
#if UA_MC
/* FELUCCA_UAC_CH channels: a frame is FELUCCA_UAC_CH 16-bit samples, as the packet carries them. The ring is the
 * smallest number of halves that holds the band, a render and a packet (FELUCCA_UAC_RING_FRAMES may give it more), so
 * it need not be a power of two: the indices run modulo UA_WRAP, a multiple of it (ua_add, ua_fill), and a slot is an
 * index modulo UA_N. A render feeds the ring only when it has room for the whole half (uac_render_start): the taps
 * then write without a check of their own, the app's channels and the master's alike */
#undef UA_N
#ifdef FELUCCA_UAC_RING_FRAMES
#define UA_N ((uint32_t)FELUCCA_UAC_RING_FRAMES)
#else
#define UA_N ((UA_HI + HALF_FRAMES + UA_MAXF) / HALF_FRAMES * HALF_FRAMES + HALF_FRAMES)
#endif
#define UA_WRAP (0x80000000u / UA_N * UA_N)
#define UAC_NCH FELUCCA_UAC_CH
#define UAC_MAXP (UA_MAXF * 2u * UAC_NCH)
#define UAC_MAXP16 UAC_MAXP
#define UA_ALTS 1u
_Static_assert(UAC_MAXP <= 1023u, "a full-speed isochronous packet is 1023 bytes at most");
_Static_assert(UA_HI + HALF_FRAMES + UA_MAXF < UA_N, "the ring holds the band, a render and a packet");
static int16_t ua_ring[UA_N][UAC_NCH] FELUCCA_UAC_RING_ATTR __attribute__((aligned(4)));
static volatile uint32_t ua_w, ua_r;
static uint32_t ep4tx[(UAC_MAXP + 3u) / 4u] __attribute__((aligned(4)));
static int16_t ua_last[UAC_NCH];                        /* the last frame sent, repeated on an underrun */
static inline uint32_t ua_add(uint32_t i, uint32_t n) { return i + n >= UA_WRAP ? i + n - UA_WRAP : i + n; }
static inline uint32_t ua_fill(uint32_t w, uint32_t r) { return w >= r ? w - r : w + UA_WRAP - r; }
typedef int16_t ua_smp_t;
#else
#if UA_IN_PLAIN
#define UAC_NCH 2u
#define UAC_MAXP (UA_MAXF * 4u)
_Static_assert(UA_HI + HALF_FRAMES + UA_MAXF < UA_N, "the ring holds the band, a render and a packet");
static uint32_t ua_ring[UA_N] FELUCCA_UAC_RING_ATTR;
static volatile uint32_t ua_w, ua_r;
static uint32_t ep4tx[UA_MAXF] __attribute__((aligned(4)));
#define UAC_MAXP16 UAC_MAXP
#define UA_ALTS 1u
#else
/* The input's channels (FELUCCA_UAC_CH) and sample sizes: alt 1 is 16-bit, alt 2 (FELUCCA_UAC_24) 24-bit. The ring
 * holds a frame's UA_CH samples side by side, 16-bit as they go out (so with 2 channels its bytes are the packet's),
 * or with FELUCCA_UAC_24 as the I2S takes them (24-bit in 32), packed for the alt the host chose as each packet goes. */
#define UAC_NCH UA_CH
#define UA_ALTS (1u + FELUCCA_UAC_24)    /* the input stream's alt settings that stream */
#define UAC_MAXP16 (UA_MAXF * UA_CH * 2u)
#define UAC_MAXP24 (UA_MAXF * UA_CH * 3u)
#if FELUCCA_UAC_24
#define UAC_MAXP UAC_MAXP24
typedef int32_t ua_smp_t;
#else
#define UAC_MAXP UAC_MAXP16
typedef int16_t ua_smp_t;
#endif
_Static_assert(UAC_MAXP <= 1023u, "a full-speed isochronous packet holds 1023 bytes: fewer channels, or 16-bit");
_Static_assert(UA_HI + HALF_FRAMES + UA_MAXF < UA_N, "the ring holds the band, a render and a packet");
/* FELUCCA_UAC_MEM 1 (fm1.toml usb_audio_mem = "app"): the input's and the output's rings are a block the app lends
 * only while the computer streams (uac_mem, below), not static RAM: 0 until lent, and then the input sends silence
 * packets (as before it is primed) and the output's packets are dropped (as when nobody plays them). Each user reads
 * the pointer once (UA_RING, UO_RING): the app takes the block back with the interrupts off and frees it two of its
 * audio blocks later, so a render still under way on the other core finishes on the block it started with */
#if FELUCCA_UAC_MEM
static ua_smp_t *volatile ua_mem;            /* the ring, lent (uac_mem); 0: none */
#define UA_HAVE(p) ((p) != 0)
#else
static ua_smp_t ua_ring_s[UA_N * UA_CH] FELUCCA_UAC_RING_ATTR __attribute__((aligned(4)));
#define ua_mem ua_ring_s
#define UA_HAVE(p) 1
#endif
#define UA_RING() ua_smp_t *const ua_ring = (ua_smp_t *)ua_mem
static volatile uint32_t ua_w, ua_r;
static uint32_t ep4tx[(UAC_MAXP + 3u) / 4u] __attribute__((aligned(4)));
#endif
#endif
static struct {
    volatile uint8_t alt;        /* streaming interface setting: 1 = the host records (2: in 24-bit) */
    volatile uint8_t r48;        /* the host's rate (SET_CUR): 1 = 48 kHz, 0 = 44.1 (bus reset) */
    uint8_t ring48;              /* the rate the ring was primed for (the render's; a change re-primes) */
    volatile uint8_t go;         /* the ring is primed: the consumer reads it (0: silence packets) */
    volatile uint8_t flowing;    /* the host takes our packets (it may set alt 1 well before it reads) */
    uint8_t queued;              /* a packet was queued since the stream (re)started */
    uint8_t feed;                /* this render feeds the ring (sampled when it starts) */
    uint8_t wait;                /* polls the queued packet has waited */
    uint8_t e0_rx;               /* SET_CUR sampling frequency: data stage pending */
    uint8_t lock;                /* uac_size's loop: 0 to start, 1 its first window, 2 with its reference phase */
    int8_t pend;                 /* the correction the last window asked for */
    uint8_t wn, since;           /* packets in this window; packets since the last correction (to UA_GAP) */
    uint8_t owe;                 /* the frames of a hold still to send (uac_packet) */
    uint8_t part;                /* 48 kHz: the frames counted of the clock's block begun (uac_size) */
    int16_t trim;                /* the pattern's rate against the host's ms, in 1/UA_ACC_ONE frames a ms (kept) */
    uint16_t relocks;            /* the phase taken as it was (a jump past UA_JUMP), and the loop's restarts */
    int32_t acc;                 /* the I2S clock's frames a ms past the whole ones, in 1/UA_ACC_ONE frames */
    uint32_t t_prev;             /* the clock at the last whole frame counted */
    int32_t due;                 /* the frames the clock counted less the frames sent (the phase, plus a constant) */
    int32_t ref, wmin;           /* the reference phase; the least phase in this window */
#if UA_IN_PLAIN
    uint32_t last;               /* the last frame sent, repeated on an underrun */
#else
    ua_smp_t last[UA_CH];        /* the last frame sent, repeated on an underrun */
#endif
    uint32_t starts, pkts, frames;               /* frames in packets sent from the ring */
    uint32_t underruns, overruns, missed;        /* glitches: ring empty (repeat), ring full (drop), no packet ready */
#if !UA_IN_PLAIN
    uint32_t ovr_pend;                           /* blocks dropped, ring full: overruns once the host takes a packet again
                                                  * (a gap in its recording); let go when it stalls or the stream changes
                                                  * (it stopped reading: nothing of a recording lost) */
#endif
    uint32_t stalls;                             /* the host stopped reading for > 20 ms (alt still 1) */
    uint32_t adj_up, adj_down;                   /* packets one frame longer / shorter than the pattern */
    uint32_t fill_lo, fill_hi;                   /* range of the fill a packet found (the frames in hand) */
    uint32_t rate_sets;                          /* SET_CUR sampling frequency requests */
} uac;
#if FELUCCA_UAC_OUT
#if UA_OUT_PLAIN
/* USB audio output: the packets in a ring (16-bit L | R << 16), read by uac_play at the I2S clock */
#define UO_N 2048u               /* frames; a power of two */
#define UO_MAXF 49u              /* frames in the largest packet the host may send (44.1: 44 or 45) */
#define UO_MAXP (UO_MAXF * 4u)
#define UO_TGT (2u * HALF_FRAMES)   /* the fill a render aims to start at: ~11.6 ms with 256-frame halves */
static uint32_t uo_ring[UO_N];
static volatile uint32_t uo_w, uo_r;
static uint32_t ep4rx[UO_MAXF + 1u] __attribute__((aligned(4)));
static struct {
    volatile uint8_t alt;        /* the output's streaming interface: 1 = the host plays */
    uint8_t go;                  /* primed: uac_play reads the ring */
    uint32_t pos;                /* the read position's fraction, Q16 */
    float err;                   /* the fill's distance from UO_TGT, smoothed (frames) */
    uint32_t pkts, frames, underruns, overruns, starts;
} uo;
#else
/* USB audio output: the packets in a ring (16-bit L | R << 16), read by uac_play at the I2S clock. With
 * FELUCCA_UAC_OUT_24 a frame is two 24-bit samples (alt 1's 16 bits << 8), in as many bytes as before: half the frames */
#define UO_MAXF 49u              /* frames in the largest packet the host may send (44.1: 44 or 45) */
#define UO_MAXP (UO_MAXF * 4u)
#define UO_TGT (2u * HALF_FRAMES)   /* the fill a render aims to start at: ~11.6 ms with 256-frame halves */
#if FELUCCA_UAC_OUT_24
#define UO_N 1024u               /* frames; a power of two */
#define UO_MAXP24 (UO_MAXF * 6u)
typedef struct {
    int32_t l, r;
} uo_frm_t;
static uint32_t ep4rx[(UO_MAXP24 + 3u) / 4u + 1u] __attribute__((aligned(4)));
_Static_assert(UO_TGT + HALF_FRAMES + UO_MAXF < UO_N, "the ring holds the target, a render and a packet");
#else
#define UO_N 2048u               /* frames; a power of two */
typedef uint32_t uo_frm_t;
static uint32_t ep4rx[UO_MAXF + 1u] __attribute__((aligned(4)));
#endif
#if FELUCCA_UAC_MEM
static uo_frm_t *volatile uo_mem;            /* the ring, lent (uac_mem, after the input's); 0: none */
#define UO_HAVE(p) ((p) != 0)
#else
static uo_frm_t uo_ring_s[UO_N];
#define uo_mem uo_ring_s
#define UO_HAVE(p) 1
#endif
#define UO_RING() uo_frm_t *const uo_ring = (uo_frm_t *)uo_mem
static volatile uint32_t uo_w, uo_r;
static struct {
    volatile uint8_t alt;        /* the output's streaming interface: 1 = the host plays */
    uint8_t go;                  /* primed: uac_play reads the ring */
    uint32_t pos;                /* the read position's fraction, Q16 */
    float err;                   /* the fill's distance from UO_TGT, smoothed (frames) */
    uint32_t pkts, frames, underruns, overruns, starts;
    uint32_t und_pend;           /* ran dry: an underrun once the computer sends again; let go when the stream changes
                                  * (it stopped sending: nothing of its audio lost) */
#if FELUCCA_UAC_OUT_FEEDBACK
    int32_t fill_q8;             /* the ring's fill as each packet arrives, low-passed (Q8 frames): the feedback */
    uint32_t fbs;                /* feedback packets queued */
    uint32_t fb;                 /* the last one (10.14 frames a ms) */
    volatile uint32_t w0;        /* uo_w when the stream (re)started: the priming counts from it (no old frames) */
#endif
} uo;
#endif
#if FELUCCA_UAC_OUT_FEEDBACK
/* The explicit feedback (UAC1 / USB 2.0 5.12.4.2): the frames a millisecond the computer is to send, 10.14 in 3 bytes.
 * The I2S's nominal rate (24 MHz / 544 = 44,117.647 Hz: 44.1176 frames a ms), less the fill's distance from UO_TGT:
 * a frame of it 1/1024 of a frame a ms (as Optimist's, from Melodee: ~1 s to close it), within +-0.5 frame a ms. A
 * crystal off by 500 ppm leaves the fill ~22 frames off the target. The computer reads it every 2^UO_FB_REFRESH ms. */
#define UO_FB_NOM ((uint32_t)((24000000ull * 16384u + 272000u) / 544000u))   /* 722824 */
#define UO_FB_MAX 8192
#define UO_FB_REFRESH 4
static uint8_t uo_fb[4] __attribute__((aligned(4)));
#endif
#endif
#if FELUCCA_UAC_MEM
/* the rings' block (FELUCCA_UAC_MEM): the input's ring, then the output's; uac_mem_bytes its size (a multiple of 4).
 * uac_mem(p), the main loop with the interrupts off: lent (p: that many bytes, 4-aligned, zeroed) or taken back (0;
 * the app frees it only after its audio's next two blocks). Either way both streams prime again from silence */
#define UAC_MEM_IN ((uint32_t)sizeof(ua_smp_t) * UA_N * UA_CH)
#if FELUCCA_UAC_OUT
#define UAC_MEM_OUT ((uint32_t)sizeof(uo_frm_t) * UO_N)
#else
#define UAC_MEM_OUT 0u
#endif
_Static_assert(UAC_MEM_IN % 8u == 0u, "the output's ring 8-aligned after the input's");
static uint32_t uac_mem_bytes(void) { return UAC_MEM_IN + UAC_MEM_OUT; }
static void uac_mem(void *p)
{
    uac.go = 0;
    uac.ovr_pend = 0;
    RING_PUBLISH();
    ua_mem = (ua_smp_t *)p;
#if FELUCCA_UAC_OUT
    uo.go = 0;
    uo.und_pend = 0;
    RING_PUBLISH();
    uo_mem = p ? (uo_frm_t *)((uint8_t *)p + UAC_MEM_IN) : 0;
#endif
}
#endif
#if FELUCCA_UAC_48K
#include "uac_fir.h"
/* the resampler's input history: the last 32 frames, written twice so a window of UAC_FIR_N never wraps */
#define RS_H 32u
_Static_assert(RS_H >= UAC_FIR_N && (RS_H & (RS_H - 1u)) == 0, "the history holds a window");
static struct {
    int16_t l[2u * RS_H], r[2u * RS_H];   /* r right after l (the FIR loop reads both from one pointer) */
    uint32_t w;                  /* input frames written */
    uint32_t ph;                 /* the next output's phase, 0..159 (its newest input frame: the next written) */
#if FELUCCA_UAC_TONE
    uint32_t tone;
#endif
} rs;
#endif
#if FELUCCA_UAC_HOST_CLOCK
/* The input on the computer's clock (FELUCCA_UAC_HOST_CLOCK). The ring holds frames at the host's 44.1 kHz, so
 * every packet is the 44.1 pattern and nothing else: the endpoint is synchronous (UAC1 / USB 2.0 5.12.4.1: its
 * rate follows SOF). uac_tap resamples each block from the I2S's clock: step, the I2S frames an output frame
 * advances (Q30, ~1.0004), interpolates between frames with a 16-tap fractional-delay filter (uac_hc_fir.h: a
 * Kaiser-windowed sinc at 64 positions, two rows blended), per channel 16 multiply-adds a frame, 24 bits in and
 * out. A frame waits 8 input frames more (0.18 ms).
 * The step is steered once a render (uac_render_start) from the ring's fill when a render starts, low-passed twice
 * (HC_LPS), against UA_PRIME: a proportional part plus the trim, the error integrated (the learned ratio of the two
 * clocks, kept over stream restarts: the same crystal). The fill is read in whole packets, as the polls see them
 * taken, so it carries up to ~22 frames of error that slides with the polls against the host's SOFs (a 100 ppm
 * crystal: once every 5 s). Near UA_PRIME the loop is slow (wn 0.05 rad/s), so that error moves the step by some
 * tens of ppm at most, slowly (the latency wanders; a pitch error under a twentieth of a cent). Past HC_E1 frames,
 * and through the first stream's first 20 s, it is fast (wn 0.9 rad/s): the ratio is learned in seconds. */
#include "uac_hc_fir.h"
#define HC_N UAC_HC_N
#define HC_ONE (1u << 30)        /* a step of one frame, Q30 */
/* the I2S frames in a USB frame, nominal: 44,117.647 / 44,100 (the trim learns the rest) */
#define HC_STEP0 ((uint32_t)(((uint64_t)HC_ONE * 44117647u + 22050000u) / 44100000u))
/* the gains (Q38 step a Q8 frame of error, Q46 a render of 256 frames a Q8 frame): wn 0.05 rad/s, critically damped,
 * near UA_PRIME; wn 0.9 rad/s from HC_E1 frames away (blended up to HC_E0) and for the first stream's HC_ACQ */
#define HC_KP_N 2435             /* 2 zeta wn / 44100 a frame */
#define HC_KI_N 90               /* wn^2 / 44100 a frame a second, x a render's time */
#define HC_KP_W 73044
#define HC_KI_W 30000            /* (zeta 1.65: for the low-passes' lag) */
#define HC_E1 32
#define HC_E0 64
#define HC_LPS 6                 /* each low-pass: 1 / 64 a render (~0.37 s with 256-frame halves) */
#define HC_ACQ (20u * 44100u / HALF_FRAMES)   /* the first 20 s at the wide gains */
#define HC_TRIM_MAX ((int64_t)2147484 << 16)   /* the trim: +-2000 ppm, Q46 */
#define HC_ADJ_MAX 5368709       /* the step's whole correction: +-0.5 %, Q30 */
_Static_assert((HC_N & (HC_N - 1u)) == 0 && HC_N == 16u, "the history wraps on a power of two; 16 taps (by 4)");
static struct {
    int32_t x[UA_CH][2u * HC_N]; /* each channel's last HC_N input frames, written twice (a window never wraps) */
    uint32_t w;                  /* input frames written */
    uint32_t pos;                /* the next output's place past frame w - 9, Q30 (0 .. HC_ONE) */
    uint32_t step;               /* input frames an output frame advances, Q30 */
    float co[HC_N], dco[HC_N];   /* the filter at cpos, and its change an output frame (dpos) within row crow */
    uint32_t cpos, crow;
    int32_t dpos;
    uint8_t cok;                 /* co is the filter at cpos */
    int32_t lp1, lp2;            /* the fill at a render's start, low-passed twice, Q8 frames */
    int64_t trim;                /* the learned step - HC_STEP0, Q46 */
    uint32_t acq;                /* renders left at the wide gains: the first stream, the trim to learn */
    uint8_t learned;             /* the trim has been learned once (it stays over streams) */
#if FELUCCA_UAC_TONE
    uint32_t tone;
#endif
} hc;
#endif
#endif

static struct {
    uint8_t up, config, pend_addr, has_pend_addr, e0_tx, e0_zlp;
    const uint8_t *e0_src;
    uint16_t e0_left;
    uint32_t resets, setups, rx_pkts, tx_pkts, sof_seen, timeouts, no_sof;
    uint32_t suspends, max_gap, retries, frame_stalls;   /* diagnostics: suspend events, longest run of polls without SOF */
    uint16_t frame, frame_same;  /* last SOF frame number and how long it has not moved (x 10 ms) */
    uint8_t detached;            /* usb_detach() was called: no retry */
    volatile uint8_t suspended;  /* bus idle > 3 ms (unplugged or host asleep): the UI hides "USB" */
    uint8_t last_setup[8];
    uint8_t sysex[16];
    uint8_t sx_len, sx_on;
    uint8_t rx_pend;             /* EP1 OUT packet seen, not yet taken (the MIDI ring was too full) */
    volatile uint8_t uboot_req;
    volatile uint8_t ota_req;    /* F0 22 24 35 7F F7: M-UPGRADE upgrade command (FELUCCA_OTA) */
} usb;

#if FELUCCA_OTA
/* M-UPGRADE SysEx (ota.c): the received frames (7-bit bytes between F0 and
 * F7) queued for the main loop, and a TX ring of SysEx event packets sent
 * before any MIDI. The queue: each frame its length (2 bytes, little-endian)
 * and its bytes, one after another; a frame that would run past the end goes
 * to the start instead (a length of 0 left where it would have been, or no
 * room for one: the reader goes on from the start), and one arriving when none
 * waits starts the buffer over (sx_epoch moves: the reader too), so a frame of
 * FELUCCA_SX_IN - 2 bytes always fits then. A frame with no room is dropped
 * (the host retries). The ISR writes and counts sx_made; the main loop reads
 * and counts sx_taken (sx_r, sx_ep its own). */
#ifndef FELUCCA_SX_IN
#if FELUCCA_LOADER
#define FELUCCA_SX_IN 4744u                       /* the update loader's fast install: a 4 KiB read's reply (4111 B,
                                                   * 4699 as it comes in, 7-bit packed) whole (loader.c checks it) */
#else
#define FELUCCA_SX_IN 648u                        /* an M-UPGRADE frame (640) whole; shorter ones several at once */
#endif
#endif
static uint8_t sx_frame[FELUCCA_SX_IN];
static uint32_t sx_pos, sx_at, sx_top, sx_w;      /* the ISR's: the frame's bytes so far, where they go, where it
                                                   * started (its length's place), where the next goes */
static uint8_t sx_from0, sx_moved;                /* it starts the buffer over; it went to the start */
static volatile uint32_t sx_made, sx_taken, sx_r; /* frames queued, taken; the oldest's place (the reader's) */
static volatile uint8_t sx_epoch, sx_ep;          /* the buffer started over (the ISR's), seen (the reader's) */
static volatile uint8_t sx_collect, sx_busy;
#ifndef SXQ
#define SXQ 64u                                   /* a firmware may give it more (a long screen-reader message whole) */
#endif
static uint32_t sx_out_q[SXQ];
static volatile uint32_t so_w, so_r;
#endif
#ifndef FELUCCA_MIRROR
#define FELUCCA_MIRROR 0
#endif
#if FELUCCA_MIRROR
/* a ring for big SysEx frames that must never hold up the music (a screen mirrored to the host): whole F0..F7
 * frames as bytes, sent last (after the SysEx ring and the notes), each whole: tx_mid says a frame is half sent
 * (nothing else goes in between) */
#ifndef MRQ
#define MRQ 4096u
#endif
static uint8_t mr_q[MRQ];
static volatile uint32_t mr_w, mr_r;
static uint8_t tx_mid;                                  /* 0 between messages, 2 a mirror frame half sent */
#endif

/* MIDI rings: 4-byte USB-MIDI event packets */
#define MQ 64u
static uint32_t midi_in_q[MQ], midi_out_q[MQ];
static uint32_t midi_in_ms[MQ];
static uint8_t midi_in_source[MQ];                    /* 1 USB, 2 TRS */
static volatile uint32_t fm1_ms;                    /* also declared by core.h; parser-only host tests */
static volatile uint32_t mi_w, mi_r, mo_w, mo_r;
static volatile uint8_t midi_usb_lost;                  /* consume in the app: release USB input owners */
static uint32_t usb_retry_ms;
static int usb_midi_out_ready(void);
static volatile uint8_t midi_in_overflow;               /* audio ISR discards a broken stream and releases notes */
/* FELUCCA_MIDI_STAMP=1: when each message in came in, in midi_in_t24 beside it, on the caller's free-running clock
 * (TIMER4's 24 MHz count, say). The caller runs usb_midi_peek(t) from a tick faster than the poll (10 kHz), the ticks
 * that wait for the render too; an EP1 OUT packet seen there for the first time came in at about t (the tick's time
 * less half the time since the last one), and ep1_rx gives its events that time when it takes the packet, however
 * late the poll. A packet first seen by the poll itself takes the last peek's t. A producer that writes midi_in_q
 * itself (a TRS parser) stores its own stamp. Within half a peek interval of the packet's landing, against the poll's
 * half interval (2 kHz: 0.25 ms) plus the time a poll waits */
#ifndef FELUCCA_MIDI_STAMP
#define FELUCCA_MIDI_STAMP 0
#endif
#if FELUCCA_MIDI_STAMP
static uint32_t midi_in_t24[MQ];
static uint32_t midi_rx_t;                              /* the stamp midi_enqueue gives (ep1_rx sets it) */
#endif

static int midi_enqueue(uint32_t pkt, uint32_t source)
{
    uint32_t at = mi_w % MQ;
    if (midi_in_overflow)
        return 0;
    if (mi_w - mi_r >= MQ) {
        midi_in_overflow = 1;
        return 0;
    }
    midi_in_q[at] = pkt;
    midi_in_ms[at] = fm1_ms;
#if FELUCCA_MIDI_STAMP
    midi_in_t24[at] = midi_rx_t;
#endif
    midi_in_source[at] = (uint8_t)source;
    RING_PUBLISH();
    mi_w++;
    return 1;
}

/* Returns 0 when disconnected or full; the application retains responsibility for unsent releases. */
static int midi_out_try(uint32_t pkt)                /* from the audio ISR */
{
    if (!usb_midi_out_ready() || mo_w - mo_r >= MQ)
        return 0;
    midi_out_q[mo_w % MQ] = pkt;
    RING_PUBLISH();
    mo_w++;
    return 1;
}
static void midi_out_event(uint32_t pkt) { (void)midi_out_try(pkt); }
static volatile uint32_t midi_links;                /* counts up at each new connection (SET_CONFIGURATION 1) */
static uint32_t midi_out_link(void) { return midi_links; }
static uint32_t midi_out_room(void)                  /* free slots in the out ring (0: no host) */
{
    return usb_midi_out_ready() ? MQ - (mo_w - mo_r) : 0u;
}

/* ------------------------------------------------------- descriptors --- */
/* Two functions: audio + MIDI (0 audio control, 1 MIDI streaming, 2 audio streaming with FELUCCA_UAC, 3 the
 * output stream with FELUCCA_UAC_OUT; with FELUCCA_UAC_MIDI_LAST: 0 audio control, 1.. the streams, MIDI last)
 * and, with FELUCCA_CDC, the CDC-ACM console (communication + data). The endpoints never move:
 * EP1 MIDI, EP2 CDC notify, EP3 CDC data, EP4 audio.
 * bcdDevice (high byte . low byte, hex): 3.00 MIDI, +0.10 the audio input (+0.20 since 1.1: 44.1 and 48 kHz,
 * FELUCCA_UAC_48K; +0.20 more with FELUCCA_UAC_CH 4..10 without FELUCCA_UAC_CAPTURE), +0.40 the audio output
 * (FELUCCA_UAC_OUT), +0.08 the audio streaming interfaces first in the AC collection (FELUCCA_UAC_AS_FIRST), +0.80 MIDI the
 * last interface (FELUCCA_UAC_MIDI_LAST=1; =2: +1.00), +2.00 the IAD without the console (FELUCCA_UAC_IAD), +0.01 CDC
 * (+0.02 per FELUCCA_USB_LAYOUT step, so a host never reuses what it learnt about another layout); the options of this
 * file, each with a bit of its own, so that two different descriptors never share a bcdDevice: +0.04 the output's feedback
 * (FELUCCA_UAC_OUT_FEEDBACK), +0.02 and +0.08 the two audio functions apart (FELUCCA_UAC_SPLIT: its +0.08 is AS_FIRST's, both
 * take the audio streaming interfaces out of MIDI's way, the +0.02 tells the two apart), +8.00 the output in 24-bit too
 * (FELUCCA_UAC_OUT_24), +10.00 x 1, 2 or 3 the input of FELUCCA_UAC_CAPTURE in 2, 4 or 8 channels (FELUCCA_UAC_CH),
 * +40.00 on the computer's clock (FELUCCA_UAC_HOST_CLOCK: a synchronous endpoint), +80.00 with 24-bit (FELUCCA_UAC_24).
 * FELUCCA_USB_LAYOUT, how the CDC build presents itself (#67: macOS 13-15 attach Apple's CDC composite
 * driver to a misc / IAD device that has a CDC function, and their kernel audio driver then never
 * takes the audio interfaces; macOS 26 / 27 (usbaudiod, matched per interface) and iOS / iPadOS work):
 *   0  1.0: device class EF 02 01 (misc / IAD), audio + MIDI IF 0.., CDC last         (the default)
 *   1  CDC first: CDC IF 0-1, then audio + MIDI, both with IADs, EF 02 01
 *   2  as 0 with device class 00 00 01: outside AppleUSBCDC's device personalities
 *      (02 * *, EF 02 01, 00 00 00); the IADs stay for Windows / Linux (test build)
 *   3  as 0 with device class 00 00 00 (test build: does AppleUSBCDC's "Vendor" personality take it?)
 * usb_cdc_on (FELUCCA_CDC_DEFAULT at boot; usb_cdc_switch re-enumerates): 0 presents the device without
 * the console, byte for byte as a FELUCCA_CDC=0 build: device class 0, audio + MIDI, no IAD. */
#ifndef FELUCCA_USB_LAYOUT
#define FELUCCA_USB_LAYOUT 0
#endif
#ifndef FELUCCA_CDC_DEFAULT
#define FELUCCA_CDC_DEFAULT 1
#endif
/* FELUCCA_USB_MANUFACTURER, FELUCCA_USB_PRODUCT: optional ASCII strings for the USB names (string
 * descriptors 1 and 2); without them the Hügelton / Felucca names below.
 * FELUCCA_USB_NAME_FN: the name of a function `const char *fn(void)` the product name is asked of each time a host reads it
 * (string 2, the MIDI device and its ports; with FELUCCA_UAC_SPLIT that name + " In" / " Out", unless FELUCCA_UAC_IN_NAME /
 * FELUCCA_UAC_OUT_NAME say otherwise): for a name that carries something of this device (its address), known only at
 * run time. ASCII, at most 25 characters (a name + " Out" is 31 at most); NULL or "" = FELUCCA_USB_PRODUCT / the default. */
/* FELUCCA_USB_SERIAL_FN: the name of a function `const char *fn(void)` giving this device's serial number (ASCII, at most
 * 31 characters; NULL or "" reads "0"), asked each time a host reads it. Without it the device has no serial number
 * (iSerialNumber 0), and a host tells two of them apart by the USB port only: macOS's MIDI setup gives a device the name
 * it kept for that port, so two FM-1s swapped between ports, or one with a new name, show the old name. */
#ifdef FELUCCA_USB_SERIAL_FN
#if defined(FELUCCA_UAC_SPLIT) && FELUCCA_UAC_SPLIT || defined(FELUCCA_UAC_CH_NAMES)
#define USB_STR_SERIAL 0x30   /* (clear of 3, 4 and the input's channel names from 5) */
#else
#define USB_STR_SERIAL 3      /* (as before: no other string past 2) */
#endif
#else
#define USB_STR_SERIAL 0
#endif
#ifndef FELUCCA_USB_PID
#define FELUCCA_USB_PID 0x0001   /* the update loader is 0x0002 */
#endif
#if FELUCCA_USB_LAYOUT < 0 || FELUCCA_USB_LAYOUT > 3
#error "FELUCCA_USB_LAYOUT is 0..3"
#endif

#define D_CFG(len, nif) 9, 2, (len) & 0xFF, (len) >> 8, (nif), 1, 0, 0x80, 50
#define D_IAD(first, n, cls, sub, proto) 8, 0x0B, (first), (n), (cls), (sub), (proto), 0
/* the input's format, type I: UA_CH channels, sub bytes a sample, bits of them; the rates */
#if FELUCCA_UAC_48K
#define D_FMT(sub, bits) 14, 0x24, 2, 1, UA_CH, (sub), (bits), 2, 0x44, 0xAC, 0x00, 0x80, 0xBB, 0x00,   /* 44100, 48000 */
#define FMT_LEN 14
#elif FELUCCA_UAC
#define D_FMT(sub, bits) 11, 0x24, 2, 1, UA_CH, (sub), (bits), 1, UAC_RATE & 0xFF, (UAC_RATE >> 8) & 0xFF, UAC_RATE >> 16,
#define FMT_LEN 11
#else
#define FMT_LEN 0
#endif
/* the AC header's collection (FELUCCA_UAC_AS_FIRST=1, Felucca since 1.5.1, its #67): the audio streaming
 * interfaces first, then the MIDI one. The kernel AppleUSBAudio (macOS up to 15; its last published source,
 * AppleUSBAudio-273.4.1, AUAConfigurationDictionary::parseConfigurationDescriptor) drops a MIDI streaming
 * interface from the collection while it walks it with the count from bInCollection: with MIDI listed first, the
 * next look-up runs past the shortened list, the parser gives up before the audio streaming interface and no
 * audio device appears (MIDI still works). Listed last, MIDI is dropped at the end of the walk. The class spec
 * gives the order no meaning (Linux takes each listed interface on its own). 0 (the default): MIDI first. */
/* FELUCCA_UAC_MIDI_LAST (fm1-loom#7, Felucca #67, bisected by darosh on macOS 15.8.1): the interface order
 * matters to macOS 13-15, the collection's order does not. With MIDI streaming as interface 1, between audio
 * control and audio streaming, no audio device appears, whatever the collection lists or in which order
 * (FELUCCA_UAC_AS_FIRST alone does not help there); with the audio streaming interface(s) right after audio
 * control and MIDI after them, it does, with or without MIDI in the collection.
 *   0  audio control, MIDI, audio streaming (in, out)                                     (the default)
 *   1  audio control, audio streaming (in, out), MIDI; the collection lists them in that order
 *   2  as 1, the collection without MIDI (the streaming interfaces only). A test build: Linux's snd-usb-audio
 *      creates a MIDI port only for a MIDI streaming interface in the collection (sound/usb/card.c,
 *      snd_usb_create_streams), so MIDI over USB is gone there.
 * The interface numbers below follow; the requests (SET_INTERFACE, GET_INTERFACE) use them by name. */
#if FELUCCA_UAC_MIDI_LAST
#define IF_AS 1                                            /* from the audio function's first interface */
#define IF_ASO 2
#define IF_MIDI (1 + FELUCCA_UAC + FELUCCA_UAC_OUT)
#else
#define IF_MIDI 1
#define IF_AS 2
#define IF_ASO 3
#endif
#if FELUCCA_UAC_OUT
#define AC_STREAMS(a) (a) + IF_AS, (a) + IF_ASO
#else
#define AC_STREAMS(a) (a) + IF_AS
#endif
#if FELUCCA_UAC_MIDI_LAST == 2
#define AC_NCOLL (FELUCCA_UAC + FELUCCA_UAC_OUT)
#define AC_COLL(a) AC_STREAMS(a)
#elif FELUCCA_UAC_MIDI_LAST || FELUCCA_UAC_AS_FIRST
#define AC_NCOLL (1 + FELUCCA_UAC + FELUCCA_UAC_OUT)
#define AC_COLL(a) AC_STREAMS(a), (a) + IF_MIDI
#else
#define AC_NCOLL (1 + FELUCCA_UAC + FELUCCA_UAC_OUT)
#define AC_COLL(a) (a) + IF_MIDI, AC_STREAMS(a)
#endif
#define D_AC_HEADER(a, terminals) 8 + AC_NCOLL, 0x24, 1, 0x00, 0x01, 8 + AC_NCOLL + (terminals), 0, AC_NCOLL, AC_COLL(a),
/* the input terminal's channels: 2 are L R (0x0003); any other count non-spatial (0: the host numbers them) */
#define UA_CHCFG (FELUCCA_UAC_CAPTURE && FELUCCA_UAC_CH != 2 ? 0u : 0x0003u)
/* EP4 IN: isochronous asynchronous (the I2S's clock), or with FELUCCA_UAC_HOST_CLOCK synchronous (the host's) */
#if FELUCCA_UAC_HOST_CLOCK
#define UA_EP_ATTR 0x0D
#else
#define UA_EP_ATTR 0x05
#endif
#if FELUCCA_UAC_SPLIT
/* the two functions' names (string descriptors 3, 4) and the input's channels' (5 ..: iChannelNames) */
#define UA_STR_IN 3
#define UA_STR_OUT 4
#ifdef FELUCCA_UAC_CH_NAMES
#define UA_STR_CH 5
#else
#define UA_STR_CH 0
#endif
#if defined(FELUCCA_USB_SERIAL_FN) && defined(FELUCCA_UAC_CH_NAMES)
_Static_assert(UA_STR_CH + FELUCCA_UAC_CH <= USB_STR_SERIAL, "the channel names' strings reach the serial number's");
#endif
#if !defined(FELUCCA_UAC_IN_NAME) && !defined(FELUCCA_USB_NAME_FN)
#ifdef FELUCCA_USB_PRODUCT
#define FELUCCA_UAC_IN_NAME FELUCCA_USB_PRODUCT " In"
#else
#define FELUCCA_UAC_IN_NAME "Felucca In"
#endif
#endif
#if !defined(FELUCCA_UAC_OUT_NAME) && !defined(FELUCCA_USB_NAME_FN)
#ifdef FELUCCA_USB_PRODUCT
#define FELUCCA_UAC_OUT_NAME FELUCCA_USB_PRODUCT " Out"
#else
#define FELUCCA_UAC_OUT_NAME "Felucca Out"
#endif
#endif
#define D_IT_LINE 12, 0x24, 2, 1, 0x03, 0x06, 0, UA_CH, UA_CHCFG & 0xFF, UA_CHCFG >> 8, UA_STR_CH, 0,
#else
#define D_IT_LINE 12, 0x24, 2, 1, 0x03, 0x06, 0, UA_CH, UA_CHCFG & 0xFF, UA_CHCFG >> 8, 0, 0,
#endif
#if FELUCCA_UAC
#if FELUCCA_UAC_OUT
#define D_AC(a)                                                                                      \
    9, 4, (a), 0, 0, 1, 1, 0, 0,                                                                     \
    D_AC_HEADER((a), 42)                                   /* AC header: MIDI, both streams */          \
    D_IT_LINE                                              /* input terminal 1: line, UA_CH ch (2: L R) */ \
    9, 0x24, 3, 2, 0x01, 0x01, 0, 1, 0,                    /* output terminal 2: USB streaming, from 1 */ \
    12, 0x24, 2, 3, 0x01, 0x01, 0, 2, 0x03, 0x00, 0, 0,    /* input terminal 3: USB streaming, 2 ch */   \
    9, 0x24, 3, 4, 0x01, 0x03, 0, 3, 0,                    /* output terminal 4: speaker, from 3 */
#if FELUCCA_UAC_OUT_FEEDBACK
#define D_ASO(o)                                                                                     \
    9, 4, (o), 0, 0, 1, 2, 0, 0,                           /* the output stream, alt 0: no bandwidth */ \
    9, 4, (o), 1, 2, 1, 2, 0, 0,                           /* alt 1: the stream and its feedback */     \
    7, 0x24, 1, 3, 1, 0x01, 0x00,                          /* AS general: terminal 3, delay 1, PCM */   \
    11, 0x24, 2, 1, 2, 2, 16, 1, 0x44, 0xAC, 0x00,         /* type I: 2 ch, 16 bit, 44100 */            \
    9, 5, 0x04, 0x05, UO_MAXP & 0xFF, UO_MAXP >> 8, 1, 0, 0x83,   /* EP4 OUT isochronous asynchronous: */ \
    7, 0x25, 1, 0x00, 0, 0, 0,                             /* CS endpoint: no controls (its rate: EP3) */ \
    9, 5, 0x83, 0x11, 3, 0, 1, UO_FB_REFRESH, 0,           /* EP3 IN: explicit feedback, 10.14 */  \
    D_ASO24(o)
#if FELUCCA_UAC_OUT_24
#define D_ASO24(o)                                                                                   \
    9, 4, (o), 2, 2, 1, 2, 0, 0,                           /* alt 2: the stream in 24-bit, its feedback */ \
    7, 0x24, 1, 3, 1, 0x01, 0x00,                          /* AS general: terminal 3, delay 1, PCM */   \
    11, 0x24, 2, 1, 2, 3, 24, 1, 0x44, 0xAC, 0x00,         /* type I: 2 ch, 3 bytes, 24 bit, 44100 */   \
    9, 5, 0x04, 0x05, UO_MAXP24 & 0xFF, UO_MAXP24 >> 8, 1, 0, 0x83,   /* EP4 OUT isochronous asynchronous */ \
    7, 0x25, 1, 0x00, 0, 0, 0,                             /* CS endpoint: no controls (its rate: EP3) */ \
    9, 5, 0x83, 0x11, 3, 0, 1, UO_FB_REFRESH, 0,           /* EP3 IN: explicit feedback, 10.14 */
#else
#define D_ASO24(o)
#endif
#else
#define D_ASO(o)                                                                                     \
    9, 4, (o), 0, 0, 1, 2, 0, 0,                           /* the output stream, alt 0: no bandwidth */ \
    9, 4, (o), 1, 1, 1, 2, 0, 0,                           /* alt 1: the stream */                      \
    7, 0x24, 1, 3, 1, 0x01, 0x00,                          /* AS general: terminal 3, delay 1, PCM */   \
    11, 0x24, 2, 1, 2, 2, 16, 1, 0x44, 0xAC, 0x00,         /* type I: 2 ch, 16 bit, 44100 */            \
    9, 5, 0x04, 0x09, UO_MAXP & 0xFF, UO_MAXP >> 8, 1, 0, 0,   /* EP4 OUT isochronous adaptive, 1 ms */ \
    7, 0x25, 1, 0x00, 0, 0, 0,                             /* CS endpoint: no controls */                \
    D_ASO24(o)
#if FELUCCA_UAC_OUT_24
#define D_ASO24(o)                                                                                   \
    9, 4, (o), 2, 1, 1, 2, 0, 0,                           /* alt 2: the stream in 24-bit */            \
    7, 0x24, 1, 3, 1, 0x01, 0x00,                          /* AS general: terminal 3, delay 1, PCM */   \
    11, 0x24, 2, 1, 2, 3, 24, 1, 0x44, 0xAC, 0x00,         /* type I: 2 ch, 3 bytes, 24 bit, 44100 */   \
    9, 5, 0x04, 0x09, UO_MAXP24 & 0xFF, UO_MAXP24 >> 8, 1, 0, 0,   /* EP4 OUT isochronous adaptive, 1 ms */ \
    7, 0x25, 1, 0x00, 0, 0, 0,                             /* CS endpoint: no controls */
#else
#define D_ASO24(o)
#endif
#endif
#else
#define D_AC(a)                                                                                      \
    9, 4, (a), 0, 0, 1, 1, 0, 0,                                                                     \
    D_AC_HEADER((a), 21)                                   /* AC header 1.00: MIDI + audio streaming */ \
    D_IT_LINE                                              /* input terminal 1: line, UA_CH ch (2: L R) */ \
    9, 0x24, 3, 2, 0x01, 0x01, 0, 1, 0,                    /* output terminal 2: USB streaming, from 1 */
#define D_ASO(o)
#endif
#define D_AS(s)                                                                                      \
    9, 4, (s), 0, 0, 1, 2, 0, 0,                           /* audio streaming, alt 0: no bandwidth */    \
    9, 4, (s), 1, 1, 1, 2, 0, 0,                           /* alt 1: the stream */                      \
    7, 0x24, 1, 2, 1, 0x01, 0x00,                          /* AS general: terminal 2, delay 1, PCM */   \
    D_FMT(2, 16)                                           /* type I: UA_CH ch, 16 bit, the rates */    \
    9, 5, 0x84, UA_EP_ATTR, UAC_MAXP16 & 0xFF, UAC_MAXP16 >> 8, 1, 0, 0,   /* EP4 IN isochronous, 1 ms */ \
    7, 0x25, 1, 0x01, 0, 0, 0,                             /* CS endpoint: sampling frequency control */ \
    D_AS24(s)
#if FELUCCA_UAC_24
#define D_AS24(s)                                                                                    \
    9, 4, (s), 2, 1, 1, 2, 0, 0,                           /* alt 2: the stream in 24-bit */            \
    7, 0x24, 1, 2, 1, 0x01, 0x00,                          /* AS general: terminal 2, delay 1, PCM */   \
    D_FMT(3, 24)                                           /* type I: UA_CH ch, 3 bytes, 24 bit */      \
    9, 5, 0x84, UA_EP_ATTR, UAC_MAXP24 & 0xFF, UAC_MAXP24 >> 8, 1, 0, 0,   /* EP4 IN isochronous, 1 ms */ \
    7, 0x25, 1, 0x01, 0, 0, 0,                             /* CS endpoint: sampling frequency control */
#else
#define D_AS24(s)
#endif
#else
#define D_AC(a) 9, 4, (a), 0, 0, 1, 1, 0, 0, 9, 0x24, 1, 0x00, 0x01, 9, 0, 1, (a) + 1,
#define D_AS(s)
#define D_ASO(o)
#endif
#define D_MIDI(m)                                                                                    \
    9, 4, (m), 0, 2, 1, 3, 0, 0,                                                                     \
    7, 0x24, 1, 0x00, 0x01, 0x41, 0x00,                                                              \
    6, 0x24, 2, 1, 1, 0,                                                                             \
    6, 0x24, 2, 2, 2, 0,                                                                             \
    9, 0x24, 3, 1, 3, 1, 2, 1, 0,                                                                    \
    9, 0x24, 3, 2, 4, 1, 1, 1, 0,                                                                    \
    9, 5, 0x01, 2, 64, 0, 0, 0, 0,                                                                   \
    5, 0x25, 1, 1, 1,                                                                                \
    9, 5, 0x81, 2, 64, 0, 0, 0, 0,                                                                   \
    5, 0x25, 1, 1, 3,
#if FELUCCA_UAC_SPLIT
/* three functions, each an IAD and an audio control interface of its own: MIDI (IF 0-1), the input (IF 2-3, its terminals
 * 1 and 2), the output (IF 4-5, its terminals 3 and 4), each control's collection its one streaming interface */
#define D_IADS(first, str) 8, 0x0B, (first), 2, 1, 1, 0, (str),
#define D_AC_MIDI(a) 9, 4, (a), 0, 0, 1, 1, 0, 0, 9, 0x24, 1, 0x00, 0x01, 9, 0, 1, (a) + 1,   /* AC header: MIDI */
#define D_AC_IN(a)                                                                                   \
    9, 4, (a), 0, 0, 1, 1, 0, UA_STR_IN,                   /* audio control: the input, named */        \
    9, 0x24, 1, 0x00, 0x01, 30, 0, 1, (a) + 1,             /* AC header: its stream */                  \
    D_IT_LINE                                              /* input terminal 1: line, UA_CH ch (named) */ \
    9, 0x24, 3, 2, 0x01, 0x01, 0, 1, 0,                    /* output terminal 2: USB streaming, from 1 */
#define D_AC_OUT(a)                                                                                  \
    9, 4, (a), 0, 0, 1, 1, 0, UA_STR_OUT,                  /* audio control: the output, named */       \
    9, 0x24, 1, 0x00, 0x01, 30, 0, 1, (a) + 1,             /* AC header: its stream */                  \
    12, 0x24, 2, 3, 0x01, 0x01, 0, 2, 0x03, 0x00, 0, 0,    /* input terminal 3: USB streaming, 2 ch */   \
    9, 0x24, 3, 4, 0x01, 0x03, 0, 3, 0,                    /* output terminal 4: speaker, from 3 */
#if FELUCCA_UAC_OUT
#define D_FN_OUT(a) D_IADS((a), UA_STR_OUT) D_AC_OUT(a) D_ASO((a) + 1)
#else
#define D_FN_OUT(a)
#endif
#define D_AUDIO(a) D_IADS((a), 0) D_AC_MIDI(a) D_MIDI((a) + 1) D_IADS((a) + 2, UA_STR_IN) D_AC_IN((a) + 2) \
                   D_AS((a) + 3) D_FN_OUT((a) + 4)
#define AUD_NIF (2 + 2 * FELUCCA_UAC + 2 * FELUCCA_UAC_OUT)
#define AUD_LEN (100 + (88 + FMT_LEN) * FELUCCA_UAC + (32 + FMT_LEN) * FELUCCA_UAC_24 + 99 * FELUCCA_UAC_OUT + \
                 9 * FELUCCA_UAC_OUT_FEEDBACK + (43 + 9 * FELUCCA_UAC_OUT_FEEDBACK) * FELUCCA_UAC_OUT_24)
#else
/* audio + MIDI from interface a, in the order FELUCCA_UAC_MIDI_LAST picks */
#if FELUCCA_UAC_MIDI_LAST
#define D_AUDIO(a) D_AC(a) D_AS((a) + IF_AS) D_ASO((a) + IF_ASO) D_MIDI((a) + IF_MIDI)
#else
#define D_AUDIO(a) D_AC(a) D_MIDI((a) + IF_MIDI) D_AS((a) + IF_AS) D_ASO((a) + IF_ASO)
#endif
#define AUD_NIF (2 + FELUCCA_UAC + FELUCCA_UAC_OUT)
#define AUD_LEN (92 + (63 + FMT_LEN) * FELUCCA_UAC + (32 + FMT_LEN) * FELUCCA_UAC_24 + 74 * FELUCCA_UAC_OUT + \
                 9 * FELUCCA_UAC_OUT_FEEDBACK + (43 + 9 * FELUCCA_UAC_OUT_FEEDBACK) * FELUCCA_UAC_OUT_24 - (FELUCCA_UAC_MIDI_LAST == 2))
#endif
#define AUD_FN_NIF (AUD_NIF - (FELUCCA_UAC_MIDI_LAST == 2))   /* the audio function's (its IAD): AC + its collection */
#define D_CDC(c)                                                                                     \
    D_IAD((c), 2, 2, 2, 1),                                /* IAD: CDC ACM */                           \
    9, 4, (c), 0, 1, 2, 2, 1, 0,                           /* communication */                          \
    5, 0x24, 0x00, 0x10, 0x01,                             /* header 1.10 */                            \
    5, 0x24, 0x01, 0x00, (c) + 1,                          /* call management: the data IF */           \
    4, 0x24, 0x02, 0x02,                                   /* ACM: line coding + state */               \
    5, 0x24, 0x06, (c), (c) + 1,                           /* union */                                  \
    7, 5, 0x82, 3, 8, 0, 16,                               /* EP2 IN interrupt (never sent) */          \
    9, 4, (c) + 1, 0, 2, 0x0A, 0, 0, 0,                    /* data */                                   \
    7, 5, 0x03, 2, 64, 0, 0,                               /* EP3 OUT bulk */                           \
    7, 5, 0x83, 2, 64, 0, 0,                               /* EP3 IN bulk */
#define CDC_LEN 66                                         /* with its IAD */
#define BCD_LO(cdc) (0x10 * (FELUCCA_UAC + FELUCCA_UAC_48K) + 0x20 * UA_MC + 0x40 * FELUCCA_UAC_OUT + 0x08 * (FELUCCA_UAC_AS_FIRST || FELUCCA_UAC_SPLIT) + \
                     0x80 * (FELUCCA_UAC_MIDI_LAST == 1) + 0x04 * FELUCCA_UAC_OUT_FEEDBACK + 0x02 * FELUCCA_UAC_SPLIT + \
                     ((cdc) ? 0x01 + 2 * FELUCCA_USB_LAYOUT : 0))
#define BCD_HI (0x03 + (FELUCCA_UAC_MIDI_LAST == 2) + 2 * FELUCCA_UAC_IAD + 0x08 * FELUCCA_UAC_OUT_24 + 0x10 * UA_CAPCODE + \
                0x40 * FELUCCA_UAC_HOST_CLOCK + 0x80 * FELUCCA_UAC_24)
#define D_DEV(bcdusb, cls, sub, proto, cdc)                                                          \
    18, 1, (bcdusb) & 0xFF, (bcdusb) >> 8, (cls), (sub), (proto), 64, 0x09, 0x12, FELUCCA_USB_PID & 0xFF, \
    FELUCCA_USB_PID >> 8, BCD_LO(cdc), BCD_HI, 1, 2, USB_STR_SERIAL, 1

/* without the console (FELUCCA_CDC=0, or usb_cdc_on = 0): class 0, no IAD, as a plain USB-MIDI / audio device.
 * FELUCCA_UAC_IAD=1: an IAD over the audio function (audio control + its collection), so the device is misc / IAD
 * (EF 02 01, USB 2.00); with the console the IADs are there anyway. */
#if FELUCCA_UAC_IAD
#define CFG_LEN_PLAIN (9 + 8 + AUD_LEN)
static const uint8_t DEV_DESC_PLAIN[18] = {D_DEV(0x0200, 0xEF, 0x02, 0x01, 0)};
static const uint8_t CFG_DESC_PLAIN[] = {D_CFG(CFG_LEN_PLAIN, AUD_NIF), D_IAD(0, AUD_FN_NIF, 1, 1, 0), D_AUDIO(0)};
#else
#define CFG_LEN_PLAIN (9 + AUD_LEN)
#if FELUCCA_UAC_SPLIT
static const uint8_t DEV_DESC_PLAIN[18] = {D_DEV(0x0200, 0xEF, 0x02, 0x01, 0)};   /* misc / IAD: its functions */
#else
static const uint8_t DEV_DESC_PLAIN[18] = {D_DEV(0x0110, 0, 0, 0, 0)};
#endif
static const uint8_t CFG_DESC_PLAIN[] = {D_CFG(CFG_LEN_PLAIN, AUD_NIF), D_AUDIO(0)};
#endif
typedef char cfg_plain_len_ok[sizeof CFG_DESC_PLAIN == CFG_LEN_PLAIN ? 1 : -1];
#if FELUCCA_CDC
#define CFG_LEN (9 + 8 + AUD_LEN + CDC_LEN)
#define USB_NIF (AUD_NIF + 2)
#if FELUCCA_USB_LAYOUT == 1
#define AUD_IF_CDC 2u                                      /* the audio function's first interface */
#define CDC_IF_CDC 0u
#else
#define AUD_IF_CDC 0u
#define CDC_IF_CDC AUD_NIF
#endif
#if FELUCCA_USB_LAYOUT == 2
static const uint8_t DEV_DESC[18] = {D_DEV(0x0200, 0x00, 0x00, 0x01, 1)};
#elif FELUCCA_USB_LAYOUT == 3
static const uint8_t DEV_DESC[18] = {D_DEV(0x0200, 0x00, 0x00, 0x00, 1)};
#else
static const uint8_t DEV_DESC[18] = {D_DEV(0x0200, 0xEF, 0x02, 0x01, 1)};   /* misc / IAD */
#endif
static const uint8_t CFG_DESC[] = {
    D_CFG(CFG_LEN, USB_NIF),
#if FELUCCA_USB_LAYOUT == 1
    D_CDC(CDC_IF_CDC)
#endif
    D_IAD(AUD_IF_CDC, AUD_FN_NIF, 1, 1, 0),               /* IAD: audio + MIDI */
    D_AUDIO(AUD_IF_CDC)
#if FELUCCA_USB_LAYOUT != 1
    D_CDC(CDC_IF_CDC)
#endif
};
typedef char cfg_len_ok[sizeof CFG_DESC == CFG_LEN ? 1 : -1];
static uint8_t usb_cdc_on = FELUCCA_CDC_DEFAULT != 0;      /* the console is presented (read by the TIMER5 ISR) */
#define USB_CDC_ON usb_cdc_on
#define AUD_IF (usb_cdc_on ? AUD_IF_CDC : 0u)
#else
#define DEV_DESC DEV_DESC_PLAIN
#define CFG_DESC CFG_DESC_PLAIN
#define USB_CDC_ON 0
#define AUD_IF 0u
#endif
#if FELUCCA_UAC_SPLIT
#define UAC_AS_IF (AUD_IF + 3u)             /* the input's function: its control IF 2, its stream IF 3 */
#define UAC_ASO_IF (AUD_IF + 5u)            /* the output's: 4 and 5 (FELUCCA_UAC_OUT) */
#else
#define UAC_AS_IF (AUD_IF + IF_AS)
#define UAC_ASO_IF (AUD_IF + IF_ASO)        /* the output stream (FELUCCA_UAC_OUT) */
#endif
#if FELUCCA_MSC
/* disk mode: one interface, mass storage (SCSI transparent, bulk-only), EP3 OUT and IN bulk; its own bcdDevice (a
 * changed device to the computer) and a serial number (the bulk-only transport asks for one) */
static const uint8_t DEV_DESC_MSC[18] = {18, 1, 0x00, 0x02, 0, 0, 0, 64, 0x09, 0x12, FELUCCA_USB_PID & 0xFF,
                                         FELUCCA_USB_PID >> 8, 0x00, 0x0D, 1, 2, 3, 1};
static const uint8_t CFG_DESC_MSC[32] = {D_CFG(32, 1),
                                         9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0,   /* mass storage, SCSI, bulk-only */
                                         7, 5, 0x03, 2, 64, 0, 0,              /* EP3 OUT bulk */
                                         7, 5, 0x83, 2, 64, 0, 0};             /* EP3 IN bulk */
static const uint8_t STR3_MSC[] = {26, 3, '0', 0, '0', 0, '0', 0, '0', 0, 'F', 0, 'M', 0, '1', 0, 'D', 0, 'I', 0, 'S', 0,
                                   'K', 0, '1', 0};
#endif
static const uint8_t STR0[4] = {4, 3, 0x09, 0x04};
static const uint8_t STR1[] = {42, 3, 'H', 0, 0xFC, 0, 'g', 0, 'e', 0, 'l', 0, 't', 0, 'o', 0, 'n', 0, ' ', 0, 'I', 0,
                               'n', 0, 's', 0, 't', 0, 'r', 0, 'u', 0, 'm', 0, 'e', 0, 'n', 0, 't', 0, 's', 0};
#ifdef FELUCCA_LOADER
static const uint8_t STR2[] = {30, 3, 'F', 0, 'e', 0, 'l', 0, 'u', 0, 'c', 0, 'c', 0, 'a', 0, ' ', 0, 'U', 0, 'p', 0,
                               'd', 0, 'a', 0, 't', 0, 'e', 0};
#else
static const uint8_t STR2[] = {16, 3, 'F', 0, 'e', 0, 'l', 0, 'u', 0, 'c', 0, 'c', 0, 'a', 0};
#endif

#if defined(FELUCCA_USB_MANUFACTURER) || defined(FELUCCA_USB_PRODUCT) || defined(FELUCCA_USB_NAME_FN) || FELUCCA_UAC_SPLIT || \
    defined(FELUCCA_USB_SERIAL_FN)
#ifdef FELUCCA_USB_NAME_FN
extern const char *FELUCCA_USB_NAME_FN(void);
#endif
#ifdef FELUCCA_USB_SERIAL_FN
extern const char *FELUCCA_USB_SERIAL_FN(void);
#endif
/* A string descriptor built from a plain ASCII string (the build settings FELUCCA_USB_MANUFACTURER and
 * FELUCCA_USB_PRODUCT, e.g. -DFELUCCA_USB_PRODUCT='"CTL-1 FM-1"'), at most 31 characters, and a suffix after it
 * (usb_str_desc2: " In" after a name asked at run time). EP0 sends one descriptor at a time, so one buffer serves all. */
static uint8_t usb_str_buf[2 + 2 * 31];

static int usb_str_desc(const char *s, const uint8_t **d, uint16_t *l)
{
    uint32_t n = 0;
    while (s[n] && n < 31u) {
        usb_str_buf[2 + 2 * n] = (uint8_t)s[n];
        usb_str_buf[3 + 2 * n] = 0;
        n++;
    }
    usb_str_buf[0] = (uint8_t)(2 + 2 * n);
    usb_str_buf[1] = 3;
    *d = usb_str_buf;
    *l = (uint16_t)(2 + 2 * n);
    return 1;
}
#if FELUCCA_UAC_SPLIT
static int usb_str_desc2(const char *s, const char *tail, const uint8_t **d, uint16_t *l)
{
    uint32_t n = 0, k = 0;
    while (s[k] && n < 31u) {
        usb_str_buf[2 + 2 * n] = (uint8_t)s[k++];
        usb_str_buf[3 + 2 * n] = 0;
        n++;
    }
    for (k = 0; tail[k] && n < 31u; k++, n++) {
        usb_str_buf[2 + 2 * n] = (uint8_t)tail[k];
        usb_str_buf[3 + 2 * n] = 0;
    }
    usb_str_buf[0] = (uint8_t)(2 + 2 * n);
    usb_str_buf[1] = 3;
    *d = usb_str_buf;
    *l = (uint16_t)(2 + 2 * n);
    return 1;
}
#endif

#ifdef FELUCCA_USB_NAME_FN
static const char *usb_product_name(void)    /* the function's name, else FELUCCA_USB_PRODUCT, else the default */
{
    const char *n = FELUCCA_USB_NAME_FN();
    if (n && n[0])
        return n;
#ifdef FELUCCA_USB_PRODUCT
    return FELUCCA_USB_PRODUCT;
#else
    return "Felucca";
#endif
}
#endif
#endif

#if FELUCCA_UAC_SPLIT && defined FELUCCA_UAC_CH_NAMES
static const char *const UA_CH_NAME[] = {FELUCCA_UAC_CH_NAMES};   /* the input's channels, in order */
_Static_assert(sizeof UA_CH_NAME / sizeof UA_CH_NAME[0] == UA_CH, "FELUCCA_UAC_CH_NAMES: a name for each input channel");
#endif
static int get_desc(uint32_t wvalue, const uint8_t **d, uint16_t *l)
{
#if FELUCCA_MSC
    if (usb_msc_on) {
        if ((wvalue >> 8) == 1) {
            *d = DEV_DESC_MSC;
            *l = 18;
            return 1;
        }
        if ((wvalue >> 8) == 2) {
            *d = CFG_DESC_MSC;
            *l = sizeof CFG_DESC_MSC;
            return 1;
        }
        if (wvalue == 0x0303u) {
            *d = STR3_MSC;
            *l = sizeof STR3_MSC;
            return 1;
        }
    }
#endif
    switch (wvalue >> 8) {
    case 1:
        *d = USB_CDC_ON ? DEV_DESC : DEV_DESC_PLAIN;
        *l = 18;
        return 1;
    case 2:
        *d = USB_CDC_ON ? CFG_DESC : CFG_DESC_PLAIN;
        *l = (uint16_t)(USB_CDC_ON ? sizeof CFG_DESC : sizeof CFG_DESC_PLAIN);
        return 1;
    case 3:
        switch (wvalue & 0xFFu) {
        case 0:
            *d = STR0;
            *l = sizeof STR0;
            return 1;
        case 1:
#ifdef FELUCCA_USB_MANUFACTURER
            return usb_str_desc(FELUCCA_USB_MANUFACTURER, d, l);
#else
            *d = STR1;
            *l = sizeof STR1;
            return 1;
#endif
        case 2:
#ifdef FELUCCA_USB_NAME_FN
            return usb_str_desc(usb_product_name(), d, l);
#elif defined(FELUCCA_USB_PRODUCT)
            return usb_str_desc(FELUCCA_USB_PRODUCT, d, l);
#else
            *d = STR2;
            *l = sizeof STR2;
            return 1;
#endif
#ifdef FELUCCA_USB_SERIAL_FN
        case USB_STR_SERIAL: {
            const char *sn = FELUCCA_USB_SERIAL_FN();
            return usb_str_desc(sn && sn[0] ? sn : "0", d, l);
        }
#endif
#if FELUCCA_UAC_SPLIT
        case UA_STR_IN:
#ifdef FELUCCA_UAC_IN_NAME
            return usb_str_desc(FELUCCA_UAC_IN_NAME, d, l);
#else
            return usb_str_desc2(usb_product_name(), " In", d, l);
#endif
#if FELUCCA_UAC_OUT
        case UA_STR_OUT:
#ifdef FELUCCA_UAC_OUT_NAME
            return usb_str_desc(FELUCCA_UAC_OUT_NAME, d, l);
#else
            return usb_str_desc2(usb_product_name(), " Out", d, l);
#endif
#endif
#endif
        default:
#if FELUCCA_UAC_SPLIT && defined FELUCCA_UAC_CH_NAMES
            if ((wvalue & 0xFFu) - UA_STR_CH < UA_CH)   /* the input's channels */
                return usb_str_desc(UA_CH_NAME[(wvalue & 0xFFu) - UA_STR_CH], d, l);
#endif
            return 0;
        }
    default:
        return 0;
    }
}

/* ---------------------------------------------------------- the SIE --- */
/* out of line on purpose (felucca-bsp#14): inlined at -O2, back-to-back SIE accesses ran a few cycles apart and EP1 IN
 * stopped on the FM-1 (built at -O2 -mcpu=r3); as calls they keep the spacing every build has been tested with */
#if FELUCCA_MSC
#define SIE_INDEX_CACHE 1                               /* (disk mode's tight polls: one SIE access fewer a packet) */
#else
#define SIE_INDEX_CACHE 0                               /* (without disk mode: every INDEX written, as ever) */
#endif
static uint8_t sie_index = 0xFFu;                       /* the INDEX last written (0xFF: not known): written
                                                         * again only when it changes. Every INDEX write comes through
                                                         * sie_wr, the compare and the write with the interrupts off,
                                                         * so the poll (ISR) and the main loop's disk pump share it;
                                                         * forgotten at a bus reset and a restart */
static __attribute__((noinline)) void sie_wr(uint32_t r, uint32_t v)
{
    uint32_t n = 20000, f = 0;
    if (!fm1_usb_sie_on())
        return;
#if SIE_INDEX_CACHE
    if (r == S_INDEX) {
        f = irq_save();
        if (v == sie_index) {
            irq_restore(f);
            return;
        }
        sie_index = 0xFFu;                              /* (known again once this write is done) */
    }
#endif
    fm1_usb_sie_wr_start(r, v);
    while (!fm1_usb_sie_done() && --n)
        ;
#if SIE_INDEX_CACHE
    if (r == S_INDEX) {
        if (n)
            sie_index = (uint8_t)v;                     /* (a write that timed out leaves it unknown) */
        irq_restore(f);
    }
#endif
    (void)f;
    if (n)
        usb.timeouts = 0;                               /* consecutive failures only */
    else if (++usb.timeouts > 50u)
        usb.up = 0;                                     /* SIE dead (no clock?): stop polling it */
}

static __attribute__((noinline)) uint32_t sie_rd(uint32_t r)
{
    uint32_t n = 20000;
    if (!fm1_usb_sie_on())
        return 0;
    fm1_usb_sie_rd_start(r);
    while (!fm1_usb_sie_done())
        if (!--n) {
            if (++usb.timeouts > 50u)
                usb.up = 0;
            return 0;
        }
    usb.timeouts = 0;
    return fm1_usb_sie_data();
}

#if FELUCCA_UAC
static void uac_ep4_reset(void)                         /* drop a queued packet; isochronous mode */
{
    sie_wr(S_INDEX, 4);
    sie_wr(S_TXCSR1, 0x48);                             /* ClrDataTog + FlushFIFO (SDK usb_g_ep_config) */
    sie_wr(S_TXCSR2, 0);
    sie_wr(S_TXCSR1, 0);
    sie_wr(S_TXCSR2, 0x40);                             /* ISO */
}

#if FELUCCA_UAC_OUT
static void uo_ep4_reset(void)                          /* EP4 OUT: isochronous, nothing pending */
{
    sie_wr(S_INDEX, 4);
    sie_wr(S_RXCSR1, 0x90);                             /* ClrDataTog + FlushFIFO */
    sie_wr(S_RXCSR2, 0x40);                             /* ISO */
}
#if FELUCCA_UAC_OUT_FEEDBACK
static void uo_fb_reset(void)                           /* EP3 IN, the feedback: isochronous, nothing queued */
{
    sie_wr(S_INDEX, 3);
    sie_wr(S_TXCSR1, 0x48);                             /* ClrDataTog + FlushFIFO (as EP4 IN) */
    sie_wr(S_TXCSR2, 0);
    sie_wr(S_TXCSR1, 0);
    sie_wr(S_TXCSR2, 0x40);                             /* ISO */
}
#endif
static void uo_stream(uint32_t alt)                     /* SET_INTERFACE, configuration, bus reset */
{
    uo.go = 0;
#if !UA_OUT_PLAIN
    uo.und_pend = 0;
#endif
    RING_PUBLISH();
    uo.alt = (uint8_t)alt;
#if FELUCCA_UAC_OUT_FEEDBACK
    uo.fill_q8 = (int32_t)(UO_TGT << 8);                /* the nominal rate until the ring has been read */
    uo.w0 = uo_w;                                       /* what the last stream left is not played */
#endif
    if (alt)
        uo.starts++;
}
#endif

static void uac_stream(uint32_t alt)                    /* SET_INTERFACE, configuration, bus reset */
{
#if !UA_IN_PLAIN
    uint32_t c;
#endif
    uac.go = 0;                                         /* the next render primes the ring again */
    uac.flowing = 0;                                    /* ... once the host has taken a packet */
    uac.queued = 0;
    uac.wait = 0;
#if !UA_IN_PLAIN
    uac.ovr_pend = 0;
#endif
    RING_PUBLISH();
    uac.alt = (uint8_t)alt;
    uac.acc = 0;
    uac.lock = 0;                                       /* the loop starts again with the next packet */
    uac.owe = 0;
#if UA_IN_PLAIN
    uac.last = 0;
#elif UA_MC
    for (c = 0; c < UAC_NCH; c++)
        ua_last[c] = 0;
#else
    for (c = 0; c < UA_CH; c++)
        uac.last[c] = 0;
#endif
    if (alt) {                                          /* (the fill range of the last stream stays readable) */
        uac.starts++;
        uac.fill_lo = 0xFFFFFFFFu;
        uac.fill_hi = 0;
    }
}

/* SET_CUR sampling frequency (any time: before or after SET_INTERFACE; a bus reset goes back to 44.1).
 * 48 kHz when built with it and asked for (the nearer of the two), else 44.1. A change restarts the
 * stream: silence packets until the next render primes the ring at the new rate. */
static void uac_rate_set(uint32_t hz)
{
    uint8_t r48 = FELUCCA_UAC_48K && hz >= (UAC_RATE + 48000u) / 2u;
    if (r48 == uac.r48)
        return;
    uac.go = 0;
    RING_PUBLISH();
    uac.r48 = r48;
    uac.acc = 0;
    uac.lock = 0;
    uac.pkts = uac.frames = 0;                          /* the measured rate (console) starts again */
}
#endif

#if FELUCCA_MSC
static void msc_config(void)                            /* disk mode's SET_CONFIGURATION 1: EP3 bulk both ways */
{
    fm1_usb_ep_txbuf(3, ep3tx);
    fm1_usb_ep_rxbuf(3, ep3rx);
    sie_wr(S_INDEX, 3);
    sie_wr(S_TXMAXP, 0xFF);
    sie_wr(S_TXCSR1, 0x48);
    sie_wr(S_TXCSR2, 0);
    sie_wr(S_RXMAXP, 0xFF);
    sie_wr(S_RXCSR1, 0x90);
    sie_wr(S_RXCSR2, 0);
    sie_wr(S_INTRRX1E, 0x08);
    fm1_usb_ep_enable(1u << 3);
    msc_rx_pend = 1;                                    /* look once: a packet may already wait */
}
#endif

static void ep1_config(void)
{
    fm1_usb_ep_txbuf(1, ep1tx);
    fm1_usb_ep_rxbuf(1, ep1rx);
    sie_wr(S_INDEX, 1);
    sie_wr(S_TXMAXP, 0xFF);
    sie_wr(S_TXCSR1, 0x48);
    sie_wr(S_TXCSR2, 0);
    sie_wr(S_RXMAXP, 0xFF);
    sie_wr(S_RXCSR1, 0x90);
    sie_wr(S_RXCSR2, 0);
    sie_wr(S_INTRRX1E, 0x02);
    fm1_usb_ep_enable(1u << 1);
#if FELUCCA_CDC
    if (!usb_cdc_on)
        goto no_cdc;                                    /* presented without the console: EP2 / EP3 unused */
    fm1_usb_ep_txbuf(2, ep2tx);
    sie_wr(S_INDEX, 2);
    sie_wr(S_TXMAXP, 0xFF);
    sie_wr(S_TXCSR1, 0x48);
    sie_wr(S_TXCSR2, 0);
    fm1_usb_ep_txbuf(3, ep3tx);
    fm1_usb_ep_rxbuf(3, ep3rx);
    sie_wr(S_INDEX, 3);
    sie_wr(S_TXMAXP, 0xFF);
    sie_wr(S_TXCSR1, 0x48);
    sie_wr(S_TXCSR2, 0);
    sie_wr(S_RXMAXP, 0xFF);
    sie_wr(S_RXCSR1, 0x90);
    sie_wr(S_RXCSR2, 0);
    sie_wr(S_INTRRX1E, 0x0A);
    fm1_usb_ep_enable((1u << 2) | (1u << 3));
    cdc.rx_pend = 1;                                    /* look once: a packet may already wait */
no_cdc:;
#endif
#if FELUCCA_UAC
    fm1_usb_ep4_txbuf(ep4tx);
    sie_wr(S_INDEX, 4);
    sie_wr(S_TXMAXP, 0xFF);
    uac_ep4_reset();
#if FELUCCA_UAC_OUT
    fm1_usb_ep4_rxbuf(ep4rx);
    sie_wr(S_INDEX, 4);
    sie_wr(S_RXMAXP, 0xFF);
    uo_ep4_reset();
    uo_stream(0);
#endif
#if FELUCCA_UAC_OUT_FEEDBACK
    fm1_usb_ep_txbuf(3, uo_fb);                         /* EP3 IN: the output's feedback */
    sie_wr(S_INDEX, 3);
    sie_wr(S_TXMAXP, 0xFF);
    uo_fb_reset();
    fm1_usb_ep_enable(1u << 3);
#endif
    fm1_usb_ep_enable(1u << 4);
    uac_stream(0);
#endif
}

static void e0_chunk(void)
{
    uint32_t n = usb.e0_left > 64u ? 64u : usb.e0_left, i;
    int last;
    for (i = 0; i < n; i++)
        ep0buf[i] = usb.e0_src[i];
    fm1_usb_ep0_send(ep0buf, n);
    usb.e0_src += n;
    usb.e0_left = (uint16_t)(usb.e0_left - n);
    last = usb.e0_left == 0 && !(n == 64u && usb.e0_zlp);
    if (usb.e0_left == 0 && n == 64u)
        usb.e0_zlp = 0;
    sie_wr(S_INDEX, 0);
    sie_wr(S_CSR0, last ? 0x0A : 0x02);
    if (last)
        usb.e0_tx = 0;
}

static void e0_send(const uint8_t *p, uint16_t len, uint16_t wlen)
{
    if (len > wlen)
        len = wlen;
    usb.e0_src = p;
    usb.e0_left = len;
    usb.e0_zlp = (len < wlen) && (len % 64u == 0);
    usb.e0_tx = 1;
    sie_wr(S_INDEX, 0);
    sie_wr(S_CSR0, 0x40);
    e0_chunk();
}

static void ep0_service(void)
{
    static const uint8_t zero2[2];
    uint32_t csr, i;
    uint16_t wvalue, wlength;
    uint8_t *s = usb.last_setup;
    if (usb.has_pend_addr) {
        sie_wr(S_FADDR, usb.pend_addr);
        usb.has_pend_addr = 0;
    }
    sie_wr(S_INDEX, 0);
    csr = sie_rd(S_CSR0);
    if (csr & 0x04u) {                                  /* SentStall */
        sie_wr(S_CSR0, 0);
        usb.e0_tx = 0;
#if FELUCCA_CDC
        cdc.e0_rx = 0;
#endif
#if FELUCCA_UAC
        uac.e0_rx = 0;
#endif
        csr = sie_rd(S_CSR0);                           /* a SETUP may be here already: its interrupt was this one */
    }
    if (csr & 0x10u) {                                  /* SetupEnd: the host abandoned the transfer */
        sie_wr(S_CSR0, 0x80);
        usb.e0_tx = 0;
#if FELUCCA_CDC
        cdc.e0_rx = 0;                                  /* else the next SETUP is taken as line coding */
#endif
#if FELUCCA_UAC
        uac.e0_rx = 0;
#endif
    }
    if (usb.e0_tx) {
        if (!(csr & 0x02u))
            e0_chunk();
        return;
    }
    if (!(csr & 0x01u))
        return;
    fm1_usb_rx_sync();
#if FELUCCA_CDC
    if (cdc.e0_rx) {                                    /* SET_LINE_CODING data stage */
        uint32_t n = sie_rd(S_COUNT0);
        for (i = 0; i < 7u && i < n; i++)
            cdc.line[i] = ep0buf[i];
        cdc.e0_rx = 0;
        goto ack;
    }
#endif
#if FELUCCA_UAC
    if (uac.e0_rx) {                                    /* SET_CUR sampling frequency: 3 bytes, Hz */
        uac.rate_sets++;
        if (sie_rd(S_COUNT0) >= 3u)
            uac_rate_set(ep0buf[0] | (uint32_t)ep0buf[1] << 8 | (uint32_t)ep0buf[2] << 16);
        uac.e0_rx = 0;
        goto ack;
    }
#endif
    for (i = 0; i < 8u; i++)
        s[i] = ep0buf[i];
    usb.setups++;
    wvalue = (uint16_t)(s[2] | s[3] << 8);
    wlength = (uint16_t)(s[6] | s[7] << 8);
    switch ((uint32_t)s[0] << 8 | s[1]) {
    case 0x0005:                                        /* SET_ADDRESS */
        usb.pend_addr = s[2] & 0x7Fu;
        usb.has_pend_addr = 1;
        goto ack;
    case 0x8006: {                                      /* GET_DESCRIPTOR */
        const uint8_t *d;
        uint16_t l;
        if (get_desc(wvalue, &d, &l)) {
            if (!wlength)
                goto ack;
            e0_send(d, l, wlength);
            return;
        }
        goto stall;
    }
    case 0x0009:                                        /* SET_CONFIGURATION: 0 or 1 only */
        if (s[2] > 1u)
            goto stall;
        usb.config = s[2];
#if FELUCCA_MSC
        if (usb_msc_on) {
            if (usb.config == 1u)
                msc_config();
            else
                sie_wr(S_INTRRX1E, 0);
            goto ack;
        }
#endif
        if (usb.config == 1u) {
            ep1_config();
            midi_links++;
        } else
            sie_wr(S_INTRRX1E, 0);
#if FELUCCA_UAC
        if (usb.config != 1u)
            uac_stream(0);
        uac_rate_set(UAC_RATE);                         /* a new configuration starts at 44.1 */
#endif
#if FELUCCA_UAC_OUT
        if (usb.config != 1u)
            uo_stream(0);
#endif
        goto ack;
    case 0x8008:
        e0_send(&usb.config, 1, wlength);
        return;
    case 0x8000:
    case 0x8100:
    case 0x8200:
        e0_send(zero2, 2, wlength);
        return;
    case 0x010B:                                        /* SET_INTERFACE: alt 0 (alt 1 for the audio stream) */
#if FELUCCA_UAC
        if (s[4] == UAC_AS_IF && wvalue <= UA_ALTS && usb.config) {
            uac_ep4_reset();
            uac_stream(wvalue);
            goto ack;
        }
#endif
#if FELUCCA_UAC_OUT
        if (s[4] == UAC_ASO_IF && wvalue <= 1u + FELUCCA_UAC_OUT_24 && usb.config) {
            uo_ep4_reset();
#if FELUCCA_UAC_OUT_FEEDBACK
            uo_fb_reset();
#endif
            uo_stream(wvalue);
            goto ack;
        }
#endif
        if (wvalue == 0)
            goto ack;
        goto stall;
    case 0x810A:
#if FELUCCA_UAC
        if (s[4] == UAC_AS_IF) {
            static uint8_t alt;
            alt = uac.alt;
            e0_send(&alt, 1, wlength);
            return;
        }
#endif
#if FELUCCA_UAC_OUT
        if (s[4] == UAC_ASO_IF) {
            static uint8_t alt_o;
            alt_o = uo.alt;
            e0_send(&alt_o, 1, wlength);
            return;
        }
#endif
        e0_send(zero2, 1, wlength);
        return;
    case 0x0201: {                                      /* CLEAR_FEATURE(ENDPOINT_HALT): data toggle reset */
        uint32_t ep = s[4] & 0x0Fu, last = (USB_CDC_ON || USB_MSC_ON) ? 3u : 1u;
#if FELUCCA_UAC
        if (wvalue == 0 && (s[4] == 0x84u || (FELUCCA_UAC_OUT && s[4] == 0x04u) ||
                            (FELUCCA_UAC_OUT_FEEDBACK && s[4] == 0x83u)))
            goto ack;                                   /* isochronous: no halt, no toggle */
#endif
        if (wvalue != 0 || ep > last)
            goto stall;                                 /* not an endpoint we have */
        if (ep) {                                       /* (EP0 has no halt to clear) */
            sie_wr(S_INDEX, ep);
            if (s[4] & 0x80u)
                sie_wr(S_TXCSR1, 0x40);
            else
                sie_wr(S_RXCSR1, 0x80);
        }
        goto ack;
    }
#if FELUCCA_MSC
    case 0xA1FE:                                        /* GET_MAX_LUN: one disk, LUN 0 */
        if (!usb_msc_on)
            goto stall;
        e0_send(zero2, 1, wlength);
        return;
    case 0x21FF:                                        /* bulk-only mass storage reset */
        if (!usb_msc_on)
            goto stall;
        msi_r = msi_w;
        msc_tx_n = 0;
        msc_resets++;
        goto ack;
#endif
#if FELUCCA_CDC
    case 0x2120:                                        /* SET_LINE_CODING: 7 bytes follow */
        if (!usb_cdc_on)
            goto stall;
        if (wlength) {
            cdc.e0_rx = 1;
            sie_wr(S_INDEX, 0);
            sie_wr(S_CSR0, 0x40);                       /* ServicedRxPktRdy, no DataEnd yet */
            return;
        }
        goto ack;
    case 0xA121:                                        /* GET_LINE_CODING */
        if (!usb_cdc_on)
            goto stall;
        e0_send(cdc.line, 7, wlength);
        return;
    case 0x2122:                                        /* SET_CONTROL_LINE_STATE */
        if (!usb_cdc_on)
            goto stall;
        cdc.dtr = s[2] & 1u;
        goto ack;
    case 0x2123:                                        /* SEND_BREAK */
        if (!usb_cdc_on)
            goto stall;
        goto ack;
#endif
#if FELUCCA_UAC
    case 0x2201:                                        /* SET_CUR, endpoint: sampling frequency */
        if (s[4] != 0x84u || s[3] != 1u)
            goto stall;
        if (wlength) {
            uac.e0_rx = 1;
            sie_wr(S_INDEX, 0);
            sie_wr(S_CSR0, 0x40);                       /* the 3 bytes follow */
            return;
        }
        goto ack;
    case 0xA281:                                        /* GET_CUR / MIN / MAX: the rate / the range */
    case 0xA282:
    case 0xA283: {
        static const uint8_t RATE[3] = {UAC_RATE & 0xFF, (UAC_RATE >> 8) & 0xFF, UAC_RATE >> 16};
#if FELUCCA_UAC_48K
        static const uint8_t RATE48[3] = {48000u & 0xFF, (48000u >> 8) & 0xFF, 48000u >> 16};
#endif
        if (s[4] != 0x84u || s[3] != 1u)
            goto stall;
#if FELUCCA_UAC_48K
        if (s[1] == 0x83u || (s[1] == 0x81u && uac.r48)) {
            e0_send(RATE48, 3, wlength);
            return;
        }
#endif
        e0_send(RATE, 3, wlength);
        return;
    }
#endif
    default:
        goto stall;
    }
ack:
    sie_wr(S_INDEX, 0);
    sie_wr(S_CSR0, 0x48);
    return;
stall:
    sie_wr(S_INDEX, 0);
    sie_wr(S_CSR0, 0x60);
}

#if FELUCCA_OTA
static void sx_start(void)                         /* F0: a frame's place (see sx_frame) */
{
    sx_pos = 0;
    sx_from0 = sx_made == sx_taken;
    sx_moved = 0;
    sx_top = sx_from0 ? 0u : sx_w;
    sx_at = sx_top + 2u;
    sx_collect = 1;
}
static void sx_over(int from0)                     /* the frame so far to the start: the buffer over (none waits) or
                                                   * past the end (the oldest leaves room there) */
{
    uint32_t i;
    for (i = 0; i < sx_pos; i++)
        sx_frame[2u + i] = sx_frame[sx_at + i];
    sx_at = 2u;
    sx_from0 = (uint8_t)from0;
    sx_moved = (uint8_t)!from0;
}
static int sx_room(void)                           /* room for the frame's next byte (moved to the start if need be) */
{
    uint32_t r;
    if (sx_from0)
        return sx_at + sx_pos < FELUCCA_SX_IN;
    r = sx_ep != sx_epoch ? 0u : sx_r;            /* (the reader's place: 0 once it sees the buffer started over) */
    if (sx_moved || sx_top < r) {                  /* before the oldest: up to it, a byte short (full never meets it) */
        if (sx_at + sx_pos + 1u < r)
            return 1;
    } else {
        if (sx_at + sx_pos < FELUCCA_SX_IN)
            return 1;
        if (2u + sx_pos + 1u < r) {                /* past the end: to the start, if the oldest leaves room there */
            sx_over(0);
            return 1;
        }
    }
    if (sx_made != sx_taken)
        return 0;
    sx_over(1);                                    /* none waits now: the buffer starts over with it */
    return sx_at + sx_pos < FELUCCA_SX_IN;
}
/* room for a packet's k bytes after the frame's (the next frame's too, its length before them, in place or at the
 * start), or none waits (nothing to wait for: a frame now starts the buffer over, moved there if it's in the way of
 * the next, or is too long and dropped); ep1_take leaves a packet until then */
static int sx_fits(uint32_t k)
{
    uint32_t r, top = sx_collect ? sx_top : sx_w, end = sx_collect ? sx_at + sx_pos : sx_w + 2u, here;
    if (sx_collect && sx_from0)
        return 1;
    r = sx_ep != sx_epoch ? 0u : sx_r;
    end += k + 2u;
    here = (sx_collect && sx_moved) || top < r ? end < r : end <= FELUCCA_SX_IN;
    if (sx_made == sx_taken) {
        if (sx_collect && !here)
            sx_over(1);
        return 1;
    }
    return here || (!(sx_collect && sx_moved) && top >= r && 2u + (sx_collect ? sx_pos : 0u) + k + 2u < r);
}
static void sx_queue(void)                         /* F7: the frame to the main loop */
{
    sx_frame[sx_at - 2u] = (uint8_t)sx_pos;
    sx_frame[sx_at - 1u] = (uint8_t)(sx_pos >> 8);
    if (sx_moved && sx_top + 2u <= FELUCCA_SX_IN)
        sx_frame[sx_top] = sx_frame[sx_top + 1u] = 0;   /* (the reader: on from the start) */
    sx_w = sx_at + sx_pos;
    if (sx_from0)
        sx_epoch++;
    RING_PUBLISH();
    sx_made++;
}
#endif

/* SysEx assembly: only short commands matter here */
static void sysex_byte(uint8_t b)
{
    static const uint8_t UBOOT_KEY[6] = {0xF0, 0x22, 0x24, 0x35, 0x7D, 0xF7};
    if (b >= 0xF8u)
        return;                                        /* realtime may occur anywhere in SysEx */
    if ((b & 0x80u) && b != 0xF0u && b != 0xF7u) {
        usb.sx_on = 0;
#if FELUCCA_OTA
        sx_collect = 0;
#endif
        return;                                        /* another status aborts the unfinished frame */
    }
#ifdef FELUCCA_SYSEX_BYTE
    FELUCCA_SYSEX_BYTE(b);                             /* the firmware's: every byte of a SysEx from USB, F0 and F7
                                                        * too (realtime left out), for its own frames longer than the
                                                        * buffer below (a DX7 bank: 4104 bytes); USB interrupt */
#endif
    if (b == 0xF0) {
        usb.sx_on = 1;
        usb.sx_len = 0;
#if FELUCCA_OTA
        sx_start();
#endif
    }
    if (!usb.sx_on)
        return;
#if FELUCCA_OTA
    if (sx_collect && b != 0xF0 && b != 0xF7) {
        if (sx_room())
            sx_frame[sx_at + sx_pos++] = b;
        else
            sx_collect = 0;                             /* too long, or no room: dropped */
    }
#endif
    if (usb.sx_len < sizeof usb.sysex)
        usb.sysex[usb.sx_len++] = b;
    if (b == 0xF7) {
        uint32_t i, ok = usb.sx_len == 6u;
        for (i = 0; ok && i < 6u; i++)
            ok = usb.sysex[i] == UBOOT_KEY[i];
        if (ok)
            usb.uboot_req = 1;
#if FELUCCA_OTA
        else if (usb.sx_len == 6u && usb.sysex[1] == 0x22 && usb.sysex[2] == 0x24 && usb.sysex[3] == 0x35 &&
                 usb.sysex[4] == 0x7F)
            usb.ota_req = 1;
        else if (sx_collect && sx_pos)
            sx_queue();                                 /* main loop: ota_take() / ed_service() */
        sx_collect = 0;
#endif
        usb.sx_on = 0;
    }
}

/* FELUCCA_MIDI_IN_RAW (felucca-bsp#15, optional): every packet as it came, before the handling below, which goes on
 * as it is, for a firmware that parses MIDI itself: the ring takes channel messages and F8 FA FB FC only, SysEx
 * goes to the soft key and the update protocol only, and system common (song position, song select, tune request)
 * and the other real-time bytes are dropped. Called from usb_poll (the TIMER5 ISR), so it must be short; ep1_take
 * holds the host back on this file's ring only. Undefined: nothing. */
static void midi_in_event(uint32_t pkt)                 /* one USB-MIDI event packet */
{
    uint32_t cin = pkt & 15u, st = (pkt >> 8) & 0xFFu;
#ifdef FELUCCA_MIDI_IN_RAW
    FELUCCA_MIDI_IN_RAW(pkt);
#endif
    if (cin >= 4u && cin <= 7u) {
        uint32_t k, nb = cin == 4u || cin == 7u ? 3u : cin == 6u ? 2u : 1u;
        for (k = 0; k < nb; k++)
            sysex_byte((uint8_t)(pkt >> (8u * (k + 1u))));
        return;
    }
    if (cin == 0xFu && st < 0xF8u && (usb.sx_on || st == 0xF0u)) {
        sysex_byte((uint8_t)st);                       /* a SysEx byte alone (CIN 0xF, a single byte): macOS sends some
                                                        * so inside a long frame (Melodee 55a0d62, for DX7 banks) */
        return;
    }
    if (st >= 0x80u && st < 0xF8u) {
        usb.sx_on = 0;                                 /* channel/system common also ends SysEx */
#if FELUCCA_OTA
        sx_collect = 0;
#endif
    }
    if (cin == 0xFu && (st == 0xF8u || st == 0xFAu || st == 0xFBu || st == 0xFCu)) {
        midi_enqueue(0xFu | st << 8, 1u);
        return;
    }
    if (cin >= 8u && cin <= 0xEu && (st >> 4) == cin && !((pkt >> 16) & 0x80u) &&
        ((cin == 0xCu || cin == 0xDu) || !(pkt & 0x80000000u))) {
        midi_enqueue(pkt, 1u);
    }
}

/* one EP1 OUT packet (<= 16 events) into the MIDI ring, or 0: fewer than 16 + 8 slots free, so the packet
 * stays (the host is NAKed) and is retried next poll; a burst never overflows the ring (that panics), and
 * 8 slots stay for TRS MIDI, which shares the ring and cannot wait (31250 baud: about 3 messages a half).
 * The update loader never drains the ring (it wants SysEx only): no back-pressure there. The same for SysEx:
 * the packet stays while the frames waiting leave no room for its bytes (16 x 3), so frames sent back to back
 * (a bank of dumps) are none of them lost. */
#define EP1_ROOM (16u + 8u)
static int ep1_take(const uint8_t *b, uint32_t n)
{
    uint32_t i;
#ifndef FELUCCA_LOADER
    if (MQ - (mi_w - mi_r) < EP1_ROOM)
        return 0;
#endif
#if FELUCCA_OTA
    if (!sx_fits(48u))
        return 0;
#endif
    for (i = 0; i + 3u < n; i += 4u)
        midi_in_event((uint32_t)b[i] | (uint32_t)b[i + 1] << 8 | (uint32_t)b[i + 2] << 16 | (uint32_t)b[i + 3] << 24);
    return 1;
}

#include "usb_host.c"

static int usb_midi_out_ready(void)
{
#if FELUCCA_USB_HOST
    if (uh.on) return uh.run && uh.out_mps;
#endif
    return usb.config;
}

#if FELUCCA_MIDI_STAMP
/* the peek (FELUCCA_MIDI_STAMP, above): one SIE read and a store a tick. INTRRX1 clears on read, so what it held goes
 * to the poll in usb_rx_seen. It runs in the same interrupt as usb_poll, never inside it (INDEX is not touched), and
 * only while the poll would read the endpoint (a host configured us, not the USB host) */
static uint32_t ep1_t, usb_peek_t;
static uint8_t ep1_seen, usb_rx_seen;
static void usb_midi_peek(uint32_t t)
{
    uint32_t ir;
    usb_peek_t = t;
#if FELUCCA_USB_HOST
    if (uh.on)
        return;
#endif
    if (!usb.up || !usb.config || ep1_seen)
        return;
    ir = sie_rd(S_INTRRX1);
    usb_rx_seen |= (uint8_t)ir;
    if (ir & 0x02u) {
        ep1_t = t;
        ep1_seen = 1;
    }
}
#endif

static void ep1_rx(void)                                /* leaves the packet (NAK) while the ring is too full */
{
    uint32_t csr, n;
    sie_wr(S_INDEX, 1);
    csr = sie_rd(S_RXCSR1) | (sie_rd(S_RXCSR2) << 8);
    if (!(csr & 1u)) {
        usb.rx_pend = 0;
#if FELUCCA_MIDI_STAMP
        ep1_seen = 0;
#endif
        return;
    }
    n = sie_rd(S_RXCOUNT1) | (sie_rd(S_RXCOUNT2) << 8);
    if (n > 64u)
        n = 64u;
    fm1_usb_rx_sync();
#if FELUCCA_MIDI_STAMP
    midi_rx_t = ep1_seen ? ep1_t : usb_peek_t;
#endif
    if (!ep1_take(ep1rx, n))
        return;                                         /* rx_pend stays: retried next poll */
#if FELUCCA_MIDI_STAMP
    ep1_seen = 0;
#endif
    usb.rx_pend = 0;
    usb.rx_pkts++;
    csr = (csr & ~0x164u) | 0x10u;
    sie_wr(S_RXCSR1, csr & 0xFFu);
    sie_wr(S_RXCSR2, csr >> 8);
}

#if FELUCCA_OTA
/* ---- SysEx frames for the main loop (ota.c / editor.c hooks; felucca.c and the
 * update loader supply ota_now_ms / ota_idle) ---- */
static uint32_t ota_now_ms(void);
static void ota_idle(void);

static int ota_wire_send(const uint8_t *p, uint32_t n)   /* F0..F7 -> USB-MIDI SysEx packets */
{
    uint32_t i = 0, t0 = ota_now_ms();
    sx_busy = 1;
    while (i < n) {
        uint32_t k = n - i >= 3u ? 3u : n - i, pkt;
        uint32_t cin = k == 3u && i + 3u < n ? 4u : k == 3u ? 7u : 4u + k;   /* 4 continues; 5/6/7 end */
        pkt = cin | (uint32_t)p[i] << 8 | (k > 1u ? (uint32_t)p[i + 1] << 16 : 0u) |
              (k > 2u ? (uint32_t)p[i + 2] << 24 : 0u);
        while (so_w - so_r >= SXQ) {
            if (!*(volatile uint8_t *)&usb.config || ota_now_ms() - t0 > 200u) {
                sx_busy = 0;
                return -1;
            }
            ota_idle();
        }
        sx_out_q[so_w % SXQ] = pkt;
        RING_PUBLISH();
        so_w++;
        i += k;
    }
    sx_busy = 0;
    return 0;
}
static int ota_frame_get(const uint8_t **p, uint32_t *n)   /* the oldest frame (the same until ota_frame_done) */
{
    uint32_t r;
    if (sx_made == sx_taken)
        return 0;
    RING_PUBLISH();                                     /* read the frame only after the count */
    r = sx_ep != sx_epoch ? 0u : sx_r;                  /* (the epoch holds while a frame waits) */
    if (r + 2u > FELUCCA_SX_IN || !(sx_frame[r] | sx_frame[r + 1u]))
        r = 0;                                          /* no frame here: on from the start */
    sx_r = r;                                           /* (before sx_ep: the ISR sees 0 either way) */
    sx_ep = sx_epoch;
    *p = sx_frame + r + 2u;
    *n = sx_frame[r] | (uint32_t)sx_frame[r + 1u] << 8;
    return 1;
}
static void ota_frame_done(void)
{
    uint32_t r = sx_r;
    r += 2u + (sx_frame[r] | (uint32_t)sx_frame[r + 1u] << 8);
    RING_PUBLISH();                                     /* done with the frame before the ISR may reuse it */
    sx_r = r;
    sx_taken++;
}
#endif

/* the polls (ep1_tx calls) a MIDI packet may wait in the IN FIFO before the host counts as not reading (50 ms at the
 * poll's 2 kHz). macOS polls the MIDI IN endpoint only while an app has the port open: without a reader, notes and clock
 * filled the queues and came out later as a stale burst. Past it the notes, the clock and the mirror's frames that wait
 * are dropped (as a bus reset does), and it stays so (tx_deaf: dropped at each poll) until the host takes the packet in
 * the FIFO: a host that starts reading gets that one packet, then only fresh ones. The FIFO itself is not flushed: the
 * packet in it may be the start of a SysEx answer, cut if it went. SysEx is kept: it is the answer to a request the host
 * just sent (an updater, an editor), never stale, and its writers give up by themselves when nobody reads (200 ms).
 * A caller that polls more often says how many make 50 ms */
#ifndef FELUCCA_MIDI_TX_STALL_POLLS
#define FELUCCA_MIDI_TX_STALL_POLLS 100u
#endif
#ifndef FELUCCA_MIDI_RT
#define FELUCCA_MIDI_RT 0
#endif
#if FELUCCA_MIDI_RT
#define RTN 8u
static uint32_t rt_q[RTN];                              /* the real-time ring (usb_midi_rt, below) */
static uint32_t rt_qw, rt_qr;
#define TX_IDLE(c) ((c) && rt_qr == rt_qw)               /* ticks waiting count as work: the stall must see them too */
#else
#define TX_IDLE(c) (c)
#endif
static uint32_t tx_wait;
static uint8_t tx_deaf;                                 /* nobody reading: notes, clock and frames dropped */
static void ep1_tx_(uint32_t pump)                     /* pump: the main loop's call (usb_mirror_pump), not the poll */
{
    uint32_t csr, n = 0;
#if FELUCCA_MIRROR
    if TX_IDLE(mo_w == mo_r && so_w == so_r && mr_w == mr_r)
        return;
#elif FELUCCA_OTA
    if TX_IDLE(mo_w == mo_r && so_w == so_r)
        return;
#else
    if TX_IDLE(mo_w == mo_r)
        return;
#endif
    sie_wr(S_INDEX, 1);
    csr = sie_rd(S_TXCSR1);
    if (csr & 0x01u) {                                  /* previous packet still pending */
        if (pump)
            return;                                     /* (the stall counts the poll's waits only: a pump calls fast) */
        if (tx_wait < FELUCCA_MIDI_TX_STALL_POLLS)
            tx_wait++;
        else
            tx_deaf = 1;                                /* nobody reads (the packet stays in the FIFO) */
        if (tx_deaf) {                                  /* drop what waits (as a bus reset does): no stale burst later */
            mo_r = mo_w;
#if FELUCCA_MIDI_RT
            rt_qr = rt_qw;                              /* (and the clock ticks waiting for a packet) */
#endif
#if FELUCCA_MIRROR
            mr_r = mr_w;
            tx_mid = 0;
#endif
        }
        return;
    }
    tx_deaf = 0;                                        /* the host took the packet: it reads */
    tx_wait = 0;
    if (csr & 0x80u)
        sie_wr(S_TXCSR1, csr & ~0x80u);
#if FELUCCA_OTA
    while (so_r != so_w && n < 64u) {                   /* SysEx first, never split by notes */
        uint32_t pkt = sx_out_q[so_r % SXQ];
#if FELUCCA_MIRROR
        if (tx_mid == 2u)
            break;                                      /* (a mirror frame half sent: it ends first; a SysEx
                                                         * message half written is sx_busy's, so a writer that
                                                         * gave up never holds the notes back) */
#endif
        ep1tx[n] = (uint8_t)pkt;
        ep1tx[n + 1] = (uint8_t)(pkt >> 8);
        ep1tx[n + 2] = (uint8_t)(pkt >> 16);
        ep1tx[n + 3] = (uint8_t)(pkt >> 24);
        n += 4u;
        so_r++;
    }
#if FELUCCA_MIRROR
    while (!tx_mid && !sx_busy && so_r == so_w && mo_r != mo_w && n < 64u) {
#else
    while (!sx_busy && so_r == so_w && mo_r != mo_w && n < 64u) {
#endif
#else
    while (mo_r != mo_w && n < 64u) {
#endif
        uint32_t pkt;
        RING_PUBLISH();                                 /* the audio ISR (producer) can preempt us: */
        pkt = midi_out_q[mo_r % MQ];                    /* slot read strictly between the index checks */
        ep1tx[n] = (uint8_t)pkt;
        ep1tx[n + 1] = (uint8_t)(pkt >> 8);
        ep1tx[n + 2] = (uint8_t)(pkt >> 16);
        ep1tx[n + 3] = (uint8_t)(pkt >> 24);
        n += 4u;
        RING_PUBLISH();
        mo_r++;
    }
#if FELUCCA_MIRROR
    /* the mirror's frames last: only between other messages, and the one begun goes on to its end */
    while (mr_r != mr_w && n < 64u && (tx_mid == 2u || (!tx_mid && !sx_busy && so_r == so_w && mo_r == mo_w))) {
        uint32_t r = mr_r, k = 0, b[3] = {0, 0, 0};
        RING_PUBLISH();                                 /* (whole frames are published: an F7 ends each) */
        do
            b[k] = mr_q[r++ % MRQ];
        while (++k < 3u && b[k - 1u] != 0xF7u);
        tx_mid = b[k - 1u] == 0xF7u ? 0u : 2u;
        ep1tx[n] = (uint8_t)(tx_mid ? 4u : 4u + k);     /* CIN 4 goes on; 5, 6, 7: ends with 1, 2, 3 bytes */
        ep1tx[n + 1] = (uint8_t)b[0];
        ep1tx[n + 2] = (uint8_t)b[1];
        ep1tx[n + 3] = (uint8_t)b[2];
        n += 4u;
        RING_PUBLISH();
        mr_r = r;
    }
#endif
    if (!n)
        return;
    fm1_usb_ep_send(1, ep1tx, n);
    sie_wr(S_TXCSR1, sie_rd(S_TXCSR1) | 0x01u);
    usb.tx_pkts++;
}
static void ep1_tx(void) { ep1_tx_(0); }
#if FELUCCA_MIRROR
/* the screen mirror, from the main loop: EP1's next packet now if the last has gone (the 2 kHz poll alone sends at most
 * two a millisecond; the host takes more). Notes and SysEx still go first: the same sender. The interrupts off around
 * it: the poll uses the same SIE index */
static __attribute__((unused)) void usb_mirror_pump(void)
{
    uint32_t f;
    if (!usb.config || mr_r == mr_w)
        return;
    f = irq_save();
    ep1_tx_(1);
    irq_restore(f);
}
#endif

#if FELUCCA_CDC
static void ep3_rx(void)                                /* leaves the packet (NAK) while the ring is full */
{
    uint32_t csr, n, i;
    sie_wr(S_INDEX, 3);
    csr = sie_rd(S_RXCSR1) | (sie_rd(S_RXCSR2) << 8);
    if (!(csr & 1u)) {
        cdc.rx_pend = 0;
        return;
    }
    n = sie_rd(S_RXCOUNT1) | (sie_rd(S_RXCOUNT2) << 8);
    if (n > 64u)
        n = 64u;
    if (CI_N - (ci_w - ci_r) < n)
        return;                                         /* rx_pend stays: retried next poll */
    cdc.rx_pend = 0;
    fm1_usb_rx_sync();
    for (i = 0; i < n; i++)
        cdc_in[(ci_w + i) % CI_N] = ep3rx[i];
    RING_PUBLISH();
    ci_w += n;
    cdc.rx_pkts++;
    csr = (csr & ~0x164u) | 0x10u;
    sie_wr(S_RXCSR1, csr & 0xFFu);
    sie_wr(S_RXCSR2, csr >> 8);
}

static void ep3_tx(void)                                /* <= 63 bytes per packet: never needs a ZLP */
{
    uint32_t csr, n = 0;
    if (co_w == co_r)
        return;
    sie_wr(S_INDEX, 3);
    csr = sie_rd(S_TXCSR1);
    if (csr & 0x01u)
        return;
    if (csr & 0x80u)
        sie_wr(S_TXCSR1, csr & ~0x80u);
    while (co_r != co_w && n < 63u)
        ep3tx[n++] = cdc_out[co_r++ % CO_N];
    fm1_usb_ep_send(3, ep3tx, n);
    sie_wr(S_TXCSR1, sie_rd(S_TXCSR1) | 0x01u);
    cdc.tx_pkts++;
}
#endif

#if FELUCCA_MSC
static void msc_ep3_rx(void)                            /* leaves the packet (NAK) while the ring is too full */
{
    uint32_t csr, n, i;
    sie_wr(S_INDEX, 3);
    csr = sie_rd(S_RXCSR1);                             /* (RxPktRdy first: the rest only with a packet there) */
    if (!(csr & 1u)) {
        msc_rx_pend = 0;
        return;
    }
    csr |= sie_rd(S_RXCSR2) << 8;
    n = sie_rd(S_RXCOUNT1);                             /* (64 at most: RXCOUNT2 is 0) */
    if (n > 64u)
        n = 64u;
    if (MSI_N - (msi_w - msi_r) < n)
        return;                                         /* rx_pend stays: retried next poll */
    msc_rx_pend = 0;
    fm1_usb_rx_sync();
    for (i = 0; i < n; i++)
        msc_in[(msi_w + i) % MSI_N] = ep3rx[i];
    RING_PUBLISH();
    msi_w += n;
    csr = (csr & ~0x164u) | 0x10u;
    sie_wr(S_RXCSR1, csr & 0xFFu);
    sie_wr(S_RXCSR2, csr >> 8);
}

static void msc_ep3_tx(void)                            /* the segment's next packet, once the last one is gone */
{
    uint32_t csr, n, i;
    const uint8_t *p = msc_tx_p;
    if (!msc_tx_n)
        return;
    sie_wr(S_INDEX, 3);
    csr = sie_rd(S_TXCSR1);
    if (csr & 0x01u)
        return;
    if (csr & 0x80u)
        sie_wr(S_TXCSR1, csr & ~0x80u);
    n = msc_tx_n > 64u ? 64u : msc_tx_n;
    for (i = 0; i < n; i++)
        ep3tx[i] = p[i];
    fm1_usb_ep_send(3, ep3tx, n);
    sie_wr(S_TXCSR1, (csr & ~0x80u) | 0x01u);       /* (the CSR read above: as m-apps' usbdisk, one access fewer) */
    msc_tx_p = p + n;
    RING_PUBLISH();
    msc_tx_n -= n;
}

/* the main loop's side of disk mode */
static int usb_msc_ready(void) { return usb_msc_on && usb.up && usb.config == 1u; }
static uint32_t usb_msc_avail(void) { return msi_w - msi_r; }
static uint32_t usb_msc_read(uint8_t *d, uint32_t max)  /* bytes from the host (bulk OUT), at most max */
{
    uint32_t n = msi_w - msi_r, i;
    if (n > max)
        n = max;
    for (i = 0; i < n; i++)
        d[i] = msc_in[(msi_r + i) % MSI_N];
    RING_PUBLISH();
    msi_r += n;
    return n;
}
static int usb_msc_busy(void) { return msc_tx_n != 0; }
static int usb_msc_send(const uint8_t *p, uint32_t n)   /* a segment to the host (p kept until !usb_msc_busy) */
{
    if (msc_tx_n || !n)
        return -1;
    msc_tx_p = p;
    RING_PUBLISH();
    msc_tx_n = n;
    return 0;
}
static uint32_t usb_msc_resets(void) { return msc_resets; }
/* disk mode, from the main loop: EP3's packets moved now, in and out, as the endpoint takes them. The 2 kHz poll alone
 * moves one 64-byte packet each way a tick (128 KB/s at most); a main loop that waits on a transfer calls this to go at
 * the host's pace. The interrupts off around it: the poll uses the same SIE index */
static __attribute__((unused)) void usb_msc_pump(uint32_t out)   /* out: the host's packets taken, else ours sent */
{
    uint32_t f;
    if (!usb_msc_ready())
        return;
    f = irq_save();
    if (out)
        msc_ep3_rx();
    else
        msc_ep3_tx();
    irq_restore(f);
}

/* present the FM-1 as a disk (on) or as itself (off): main loop. As usb_cdc_switch: a change while attached drops
 * off the bus, and usb_retry attaches again within a second with the other descriptors */
static __attribute__((unused)) void usb_msc_switch(uint32_t on)
{
    on = on != 0;
    if (on == usb_msc_on)
        return;
    if (usb.up) {
        usb.up = 0;                                     /* usb_poll (TIMER5) stops before the SIE goes */
        RING_PUBLISH();
        fm1_usb_off();
    }
    usb.config = 0;
    msi_r = msi_w;
    msc_tx_n = 0;
    msc_rx_pend = 0;
    usb_msc_on = (uint8_t)on;
}
#endif

#if FELUCCA_UAC
/* ---- the audio input: audio.c produces (render start, then each block), TIMER5 consumes ---- */
#if FELUCCA_UAC_HOST_CLOCK
static void uac_hc_prime(uint32_t fill)                 /* the stream (re)starts: silence in, the learned ratio */
{
    uint32_t c, i;
    for (c = 0; c < UA_CH; c++)
        for (i = 0; i < 2u * HC_N; i++)
            hc.x[c][i] = 0;
    hc.w = 0;
    hc.pos = 0;
    hc.cok = 0;
    hc.lp1 = hc.lp2 = (int32_t)(fill << 8);
    if (!hc.learned) {
        hc.learned = 1;
        hc.acq = HC_ACQ;
    }
    hc.step = HC_STEP0 + (uint32_t)(int32_t)(hc.trim >> 16);
}

/* a 32 x 32 -> 64-bit product out of line: on the FM-1 such a multiply inlined among loads now and then leaves a register
 * a bit off (docs/CPU.md); here its product is used at once, before any load */
static __attribute__((noinline)) int64_t uac_hc_mul(int32_t a, int32_t b) { return (int64_t)a * b; }
/* once a render, the fill when it starts: the step that brings it back to UA_PRIME (see hc) */
static void uac_hc_steer(uint32_t fill)
{
    const int32_t e0 = (HC_E0 - HC_E1) << 8;
    int32_t e, g, adj, kp, ki;
    hc.lp1 += ((int32_t)(fill << 8) - hc.lp1) >> HC_LPS;
    hc.lp2 += (hc.lp1 - hc.lp2) >> HC_LPS;
    e = hc.lp2 - (int32_t)(UA_PRIME << 8);
    e = e > (int32_t)(UA_N << 6) ? (int32_t)(UA_N << 6) : e < -(int32_t)(UA_N << 6) ? -(int32_t)(UA_N << 6) : e;
    g = (e < 0 ? -e : e) - (HC_E1 << 8);              /* the wide gains past HC_E1 frames, all of them at HC_E0 */
    g = g > e0 || hc.acq ? e0 : g < 0 ? 0 : g;
    if (hc.acq)
        hc.acq--;
    kp = HC_KP_N + (HC_KP_W - HC_KP_N) * g / e0;      /* (g <= e0: in 32 bits) */
    ki = (HC_KI_N + (HC_KI_W - HC_KI_N) * g / e0) * (int32_t)HALF_FRAMES / 256;
    hc.trim += uac_hc_mul(e, ki);
    hc.trim = hc.trim > HC_TRIM_MAX ? HC_TRIM_MAX : hc.trim < -HC_TRIM_MAX ? -HC_TRIM_MAX : hc.trim;
    adj = (int32_t)(hc.trim >> 16) + (int32_t)(uac_hc_mul(e, kp) >> 8);
    adj = adj > HC_ADJ_MAX ? HC_ADJ_MAX : adj < -HC_ADJ_MAX ? -HC_ADJ_MAX : adj;
    hc.step = HC_STEP0 + (uint32_t)adj;
}
#endif

#if UA_MC
static void ua_clear_app(uint32_t w, uint32_t n)        /* the app's channels of n frames from w: silence */
{
    uint32_t i, c, k = w % UA_N;
    for (i = 0; i < n; i++, k = k + 1u == UA_N ? 0u : k + 1u)
        for (c = 2; c < UAC_NCH; c++)
            ua_ring[k][c] = 0;
}
/* FELUCCA_UAC_CH: a render feeds the ring only with room for the whole half (the taps do not check); the app's
 * channels of that half are cleared first (the app adds into them: uac_ch_frame) */
static __attribute__((noinline)) void uac_render_start(void)   /* audio ISR, before a half buffer renders */
{
    uint32_t fill, r;
    uac.feed = uac.alt && usb.config && uac.flowing;
    if (!uac.feed) {
        uac.go = 0;
        return;
    }
    r = ua_r;
    if (!uac.go) {                                      /* (re)start: silence ahead, then this render */
        uint32_t i, c;
        for (i = 0; i < UA_PRIME; i++)
            for (c = 0; c < UAC_NCH; c++)
                ua_ring[(r + i) % UA_N][c] = 0;      /* (once a stream) */
        RING_PUBLISH();
        ua_w = ua_add(r, UA_PRIME);
        RING_PUBLISH();
        uac.go = 1;
        ua_clear_app(ua_w, HALF_FRAMES);
        return;
    }
    fill = ua_fill(ua_w, r);
    if (UA_N - fill < HALF_FRAMES) {                    /* nobody reads: drop the half */
        uac.overruns++;
        uac.feed = 0;
        return;
    }
    ua_clear_app(ua_w, HALF_FRAMES);
}

/* the app's channels (3..FELUCCA_UAC_CH) of the frame off frames into the render's next block (off 0: the frame the
 * next uac_tap writes first), for the app to add into while it renders that block; *run: the frames from it that lie
 * one after another in memory before the ring wraps (a block may straddle the end). The audio ISR (or the second core
 * while the first waits for it), between uac_render_start and that block's uac_tap. NULL when this render does not feed
 * the ring: nobody records, or no room */
static int16_t *uac_ch_frame(uint32_t off, uint32_t *run)
{
    uint32_t s;
    if (!uac.feed) {
        *run = 0;
        return 0;
    }
    s = (ua_w + off) % UA_N;
    *run = UA_N - s;
    return &ua_ring[s][2];
}
#define UAC_CH_STRIDE UAC_NCH                           /* int16s from one frame to the next */
#else
#if UA_IN_PLAIN
static __attribute__((noinline)) void uac_render_start(void)   /* audio ISR, before a half buffer renders */
{
    uac.feed = uac.alt && usb.config && uac.flowing;
    if (!uac.feed) {
        uac.go = 0;                                     /* not read (yet / any more): prime when it is */
        return;
    }
    if (!uac.go || uac.ring48 != uac.r48) {             /* (re)start: silence ahead, then this render */
        uint32_t r = ua_r, i, prime = UA_PRIME;
        uac.go = 0;
#if FELUCCA_UAC_48K
        uac.ring48 = uac.r48;
        if (uac.ring48) {                               /* the resampler starts from silence */
            prime = UA_PRIME48;
            for (i = 0; i < 2u * RS_H; i++)
                rs.l[i] = rs.r[i] = 0;
            rs.w = rs.ph = 0;
        }
#endif
        for (i = 0; i < prime; i++)
            ua_ring[(r + i) & (UA_N - 1u)] = 0;
        RING_PUBLISH();
        ua_w = r + prime;
        RING_PUBLISH();
        uac.go = 1;
    }
}
#else
static __attribute__((noinline)) void uac_render_start(void)   /* audio ISR, before a half buffer renders */
{
    UA_RING();
    uac.feed = uac.alt && usb.config && uac.flowing && UA_HAVE(ua_ring);   /* (no ring lent yet: not fed) */
    if (!uac.feed) {
        uac.go = 0;                                     /* not read (yet / any more): prime when it is */
        return;
    }
    if (!uac.go || uac.ring48 != uac.r48) {             /* (re)start: silence ahead, then this render */
        uint32_t r = ua_r, i, prime = UA_PRIME;
        uac.go = 0;
#if FELUCCA_UAC_48K
        uac.ring48 = uac.r48;
        if (uac.ring48) {                               /* the resampler starts from silence */
            prime = UA_PRIME48;
            for (i = 0; i < 2u * RS_H; i++)
                rs.l[i] = rs.r[i] = 0;
            rs.w = rs.ph = 0;
        }
#endif
        for (i = 0; i < prime; i++) {
            uint32_t c;
            for (c = 0; c < UA_CH; c++)
                ua_ring[((r + i) & (UA_N - 1u)) * UA_CH + c] = 0;
        }
#if FELUCCA_UAC_HOST_CLOCK
        uac_hc_prime(prime);
#endif
        RING_PUBLISH();
        ua_w = r + prime;
        RING_PUBLISH();
        uac.go = 1;
        return;
    }
#if FELUCCA_UAC_HOST_CLOCK
    uac_hc_steer(ua_w - ua_r);
#endif
}
#endif

#endif
/* the host takes the input this render (sampled by uac_render_start): uac_tap's frames go out; 0, it drops them, so
 * the caller may skip what it would have given it (its stems) */
static __attribute__((unused)) int uac_capturing(void) { return uac.feed; }

static inline int32_t uac_s16(int32_t v)
{
    return v > 32767 ? 32767 : v < -32768 ? -32768 : v;
}
/* a sample as the I2S takes it (24-bit, fm1_audio.h) to 16 bits, rounded */
static inline int32_t uac_in16(int32_t v) { return uac_s16((v + 128) >> 8); }
#if FELUCCA_UAC_24
static inline int32_t uac_in24(int32_t v) { return v > 8388607 ? 8388607 : v < -8388608 ? -8388608 : v; }
#define UA_SMP(v) uac_in24(v)    /* the ring keeps 24 bits: each packet packs them for its alt */
#else
#define UA_SMP(v) uac_in16(v)
#endif

#if FELUCCA_UAC_48K
/* one tap into a sum, out of line and in registers only (docs/CPU.md: 64-bit multiplies inlined among loads, the
 * history's 16-bit ones above all, are what fails on the FM-1; here nothing is loaded and the product is used at once) */
static __attribute__((noinline)) int64_t uac_mac48(int64_t a, int32_t t, int32_t x)
{
    return a + (int64_t)t * x;
}
/* 44.1 -> 48 kHz (x 160 / 147), one block: each input frame enters the history, then the 1 or 2 output
 * frames whose newest input it is: phase ph, the window of UAC_FIR_N frames ending at it, the taps of
 * phase ph (or of 159 - ph backwards: the prototype is symmetric), Q30 into 64-bit sums. */
#if UA_IN_PLAIN
static __attribute__((noinline)) void uac_tap48(const int32_t *out, uint32_t n)
{
    uint32_t w = ua_w, i, j, k = 0, h = rs.w, ph = rs.ph;
    if (UA_N - (w - ua_r) < n + n / 8u + 2u) {         /* nobody reads: drop the block (<= n * 160 / 147 + 1) */
        uac.overruns++;
        return;
    }
    for (i = 0; i < n; i++) {
        uint32_t s = h & (RS_H - 1u);
        int16_t l = (int16_t)uac_in16(out[2u * i]), r = (int16_t)uac_in16(out[2u * i + 1u]);
        rs.l[s] = rs.l[s + RS_H] = l;
        rs.r[s] = rs.r[s + RS_H] = r;
        h++;
        s = (h - UAC_FIR_N) & (RS_H - 1u);              /* the window: oldest .. this frame */
        do {
            const int16_t *x = &rs.l[s];                /* R: x[2 * RS_H] */
            int64_t al = 0, ar = 0;
            int32_t yl, yr;
            if (ph < UAC_FIR_L / 2u) {
                const int32_t *c = UAC_FIR[ph];
                for (j = UAC_FIR_N; j; j--, x++) {
                    int32_t t = *c++;
                    al = uac_mac48(al, t, x[0]);
                    ar = uac_mac48(ar, t, x[2u * RS_H]);
                }
            } else {
                const int32_t *c = &UAC_FIR[UAC_FIR_L - 1u - ph][UAC_FIR_N - 1u];
                for (j = UAC_FIR_N; j; j--, x++) {
                    int32_t t = *c--;
                    al = uac_mac48(al, t, x[0]);
                    ar = uac_mac48(ar, t, x[2u * RS_H]);
                }
            }
            yl = uac_s16((int32_t)((al + (1 << (UAC_FIR_Q - 1))) >> UAC_FIR_Q));
            yr = uac_s16((int32_t)((ar + (1 << (UAC_FIR_Q - 1))) >> UAC_FIR_Q));
#if FELUCCA_UAC_TONE
            {   /* bench: the triangles at 48 kHz (not resampled), one per output frame */
                uint32_t t = rs.tone, q = t < 50u ? t : 100u - t, q2 = t < 50u ? 50u - t : t - 50u;
                rs.tone = t == 99u ? 0u : t + 1u;
                yl = (int32_t)q * 1000 - 25000;
                yr = (int32_t)q2 * 1000 - 25000;
            }
#endif
            ua_ring[(w + k++) & (UA_N - 1u)] = (uint32_t)(uint16_t)yl | (uint32_t)(uint16_t)yr << 16;
            ph += UAC_FIR_M;
        } while (ph < UAC_FIR_L);
        ph -= UAC_FIR_L;
    }
    rs.w = h;
    rs.ph = ph;
    RING_PUBLISH();
    ua_w = w + k;
}
#else
static __attribute__((noinline)) void uac_tap48(const int32_t *out, uint32_t n)
{
    UA_RING();
    uint32_t w = ua_w, i, j, k = 0, h = rs.w, ph = rs.ph;
    if (!UA_HAVE(ua_ring))
        return;
    if (UA_N - (w - ua_r) < n + n / 8u + 2u) {         /* nobody reads: drop the block (<= n * 160 / 147 + 1) */
        uac.ovr_pend++;                             /* (a loss only if the host reads on: uac_service) */
        return;
    }
    for (i = 0; i < n; i++) {
        uint32_t s = h & (RS_H - 1u);
        int16_t l = out ? (int16_t)uac_in16(out[2u * i]) : 0, r = out ? (int16_t)uac_in16(out[2u * i + 1u]) : 0;
        rs.l[s] = rs.l[s + RS_H] = l;
        rs.r[s] = rs.r[s + RS_H] = r;
        h++;
        s = (h - UAC_FIR_N) & (RS_H - 1u);              /* the window: oldest .. this frame */
        do {
            const int16_t *x = &rs.l[s];                /* R: x[2 * RS_H] */
            int64_t al = 0, ar = 0;
            int32_t yl, yr;
            if (ph < UAC_FIR_L / 2u) {
                const int32_t *c = UAC_FIR[ph];
                for (j = UAC_FIR_N; j; j--, x++) {
                    int32_t t = *c++;
                    al = uac_mac48(al, t, x[0]);
                    ar = uac_mac48(ar, t, x[2u * RS_H]);
                }
            } else {
                const int32_t *c = &UAC_FIR[UAC_FIR_L - 1u - ph][UAC_FIR_N - 1u];
                for (j = UAC_FIR_N; j; j--, x++) {
                    int32_t t = *c--;
                    al = uac_mac48(al, t, x[0]);
                    ar = uac_mac48(ar, t, x[2u * RS_H]);
                }
            }
            yl = uac_s16((int32_t)((al + (1 << (UAC_FIR_Q - 1))) >> UAC_FIR_Q));
            yr = uac_s16((int32_t)((ar + (1 << (UAC_FIR_Q - 1))) >> UAC_FIR_Q));
#if FELUCCA_UAC_TONE
            {   /* bench: the triangles at 48 kHz (not resampled), one per output frame */
                uint32_t t = rs.tone, q = t < 50u ? t : 100u - t, q2 = t < 50u ? 50u - t : t - 50u;
                rs.tone = t == 99u ? 0u : t + 1u;
                yl = (int32_t)q * 1000 - 25000;
                yr = (int32_t)q2 * 1000 - 25000;
            }
#endif
            {
                ua_smp_t *d = &ua_ring[((w + k++) & (UA_N - 1u)) * 2u];   /* (stereo 16-bit only) */
                d[0] = (int16_t)yl;
                d[1] = (int16_t)yr;
            }
            ph += UAC_FIR_M;
        } while (ph < UAC_FIR_L);
        ph -= UAC_FIR_L;
    }
    rs.w = h;
    rs.ph = ph;
    RING_PUBLISH();
    ua_w = w + k;
}
#endif
#endif

#if FELUCCA_UAC_HOST_CLOCK
/* the filter at place pos (Q30 past frame w - 9) into hc.co: rows pos / 2^24 and the next, blended by the rest
 * (a row past UAC_HC_P / 2 is its mirror backwards), in single precision (the FPU: no 64-bit integer multiply in
 * this loop, docs/CPU.md). The place moves by step - HC_ONE an output frame, so within a row the blend moves by the
 * same each time: hc.dco, added (16 additions a frame, shared by the channels); a new row, a new step or any other
 * move (an input frame skipped or doubled) blends it again (16 multiplications). */
static inline void uac_hc_filter(uint32_t pos, uint32_t step)
{
    uint32_t p = pos >> 24, j;
    int32_t d = (int32_t)(pos - hc.cpos);
    hc.cpos = pos;
    if (hc.cok && p == hc.crow && d == hc.dpos) {
        float *c = hc.co;
        const float *dc = hc.dco;
        for (j = 0; j < HC_N; j += 4u, c += 4, dc += 4) {
            c[0] += dc[0];
            c[1] += dc[1];
            c[2] += dc[2];
            c[3] += dc[3];
        }
        return;
    }
    {
        const float q = 1.0f / (float)(1u << UAC_HC_Q);  /* (the table's Q30 to 1.0) */
        float fr = (float)((pos >> 8) & 0xFFFFu) * (1.0f / 65536.0f);
        float dfr = (float)((int32_t)(step - HC_ONE) >> 8) * (1.0f / 65536.0f);
        const int32_t *a = UAC_HC[p <= UAC_HC_P / 2u ? p : UAC_HC_P - p];
        const int32_t *b = UAC_HC[p + 1u <= UAC_HC_P / 2u ? p + 1u : UAC_HC_P - p - 1u];
        int ra = p > UAC_HC_P / 2u, rb = p + 1u > UAC_HC_P / 2u;
        for (j = 0; j < HC_N; j++) {
            float x = (float)a[ra ? HC_N - 1u - j : j] * q, y = (float)b[rb ? HC_N - 1u - j : j] * q;
            hc.co[j] = x + (y - x) * fr;
            hc.dco[j] = (y - x) * dfr;
        }
        hc.crow = p;
        hc.dpos = (int32_t)(step - HC_ONE);
        hc.cok = 1;
    }
}

/* the I2S's frames (n, UA_CH samples each) to the host's clock: each input frame enters the history, then the
 * output frames (0, 1 or 2) whose place falls before the next: the filter at their place over the last HC_N */
static __attribute__((noinline)) void uac_tap_hc(const int32_t *out, uint32_t n)
{
    UA_RING();
    uint32_t w = ua_w, i, k = 0, h = hc.w, pos = hc.pos, step = hc.step;
    if (!UA_HAVE(ua_ring))
        return;
    if (UA_N - (w - ua_r) < n + n / 128u + 2u) {       /* nobody reads: drop the block (<= n x 1.005 + 1) */
        uac.ovr_pend++;
        return;
    }
    for (i = 0; i < n; i++) {
        uint32_t s = h & (HC_N - 1u), c;
        for (c = 0; c < UA_CH; c++) {
            int32_t v = out ? out[UA_CH * i + c] : 0;
            v = v > 8388607 ? 8388607 : v < -8388608 ? -8388608 : v;   /* 24 bits: exact in a float */
            hc.x[c][s] = hc.x[c][s + HC_N] = v;
        }
        h++;
        s = h & (HC_N - 1u);                            /* the window: frames h - HC_N .. h - 1 */
        while (pos < HC_ONE) {
            const float *co = hc.co;
            ua_smp_t *d = &ua_ring[((w + k++) & (UA_N - 1u)) * UA_CH];
            uac_hc_filter(pos, step);
            for (c = 0; c < UA_CH; c++) {
                const int32_t *x = &hc.x[c][s];
                const float *q = co;
                float acc = 0.0f;
                uint32_t j;
                for (j = 0; j < HC_N; j += 4u, x += 4, q += 4)    /* (by four: the loads and sums in registers) */
                    acc += q[0] * (float)x[0] + q[1] * (float)x[1] + q[2] * (float)x[2] + q[3] * (float)x[3];
                d[c] = (ua_smp_t)UA_SMP((int32_t)(acc + (acc < 0.0f ? -0.5f : 0.5f)));   /* (rounded) */
            }
#if FELUCCA_UAC_TONE
            {   /* bench: the triangles at the host's clock (not resampled), one per output frame */
                uint32_t t = hc.tone, q = t < 50u ? t : 100u - t, q2 = t < 50u ? 50u - t : t - 50u;
                hc.tone = t == 99u ? 0u : t + 1u;
                d[0] = (int16_t)((int32_t)q * 1000 - 25000);
                d[1] = (int16_t)((int32_t)q2 * 1000 - 25000);
            }
#endif
            pos += step;
        }
        pos -= HC_ONE;
    }
    hc.w = h;
    hc.pos = pos;
    RING_PUBLISH();
    ua_w = w + k;
}
#endif

/* audio ISR: one block, n frames of UA_CH samples side by side (2: L R), 24-bit as the I2S takes it (the master
 * output, or the caller's own channels); out 0: n frames of silence */
#if UA_MC
static __attribute__((noinline)) void uac_tap(const int32_t *out, uint32_t n)   /* audio ISR: one block, 24-bit as the I2S takes it */
{
    uint32_t w = ua_w, i, k;
    if (!uac.feed)
        return;
    for (i = 0, k = w % UA_N; i < n; i++, k = k + 1u == UA_N ? 0u : k + 1u) {   /* the master on 1 and 2 (the room:
                                                                               * uac_render_start) */
#if FELUCCA_UAC_TONE
        static uint32_t ph;                             /* bench: the triangles below, on 1 and 2 */
        uint32_t q = ph < 50u ? ph : 100u - ph, q2 = ph < 50u ? 50u - ph : ph - 50u;
        ph = ph == 99u ? 0u : ph + 1u;
        ua_ring[k][0] = (int16_t)((int32_t)q * 1000 - 25000);
        ua_ring[k][1] = (int16_t)((int32_t)q2 * 1000 - 25000);
        (void)out;
#else
        ua_ring[k][0] = (int16_t)uac_s16((out[2u * i] + 128) >> 8);
        ua_ring[k][1] = (int16_t)uac_s16((out[2u * i + 1u] + 128) >> 8);
#endif
    }
    RING_PUBLISH();
    ua_w = ua_add(w, n);
}
#else
#if UA_IN_PLAIN
static __attribute__((noinline)) void uac_tap(const int32_t *out, uint32_t n)   /* audio ISR: one block, 24-bit as the I2S takes it */
{
    uint32_t w = ua_w, i;
    if (!uac.feed)
        return;
#if FELUCCA_UAC_48K
    if (uac.ring48) {
        uac_tap48(out, n);
        return;
    }
#endif
    if (UA_N - (w - ua_r) < n) {                        /* nobody reads: drop the block */
        uac.overruns++;
        return;
    }
#if FELUCCA_UAC_TONE
    {   /* bench: triangles of period 100 frames instead of the music (L, R in antiphase): every
         * recorded frame must follow the one before, so a lost or repeated frame shows exactly */
        static uint32_t ph;
        for (i = 0; i < n; i++, ph = ph == 99u ? 0u : ph + 1u) {
            uint32_t q = ph < 50u ? ph : 100u - ph, q2 = ph < 50u ? 50u - ph : ph - 50u;
            ua_ring[(w + i) & (UA_N - 1u)] = (uint32_t)(uint16_t)(int16_t)((int32_t)q * 1000 - 25000) |
                                             (uint32_t)(uint16_t)(int16_t)((int32_t)q2 * 1000 - 25000) << 16;
        }
        (void)out;
    }
#else
    for (i = 0; i < n; i++)
        ua_ring[(w + i) & (UA_N - 1u)] = (uint32_t)(uint16_t)uac_in16(out[2u * i]) |
                                         (uint32_t)(uint16_t)uac_in16(out[2u * i + 1u]) << 16;
#endif
    RING_PUBLISH();
    ua_w = w + n;
}
#else
static __attribute__((noinline)) void uac_tap(const int32_t *out, uint32_t n)
{
    UA_RING();
    uint32_t w = ua_w, i;
    if (!uac.feed || !UA_HAVE(ua_ring))
        return;
#if FELUCCA_UAC_48K
    if (uac.ring48) {
        uac_tap48(out, n);
        return;
    }
#endif
#if FELUCCA_UAC_HOST_CLOCK
    uac_tap_hc(out, n);
    return;
#endif
    if (UA_N - (w - ua_r) < n) {                        /* nobody reads: drop the block */
        uac.ovr_pend++;
        return;
    }
#if FELUCCA_UAC_TONE
#if FELUCCA_UAC_CH != 2 || FELUCCA_UAC_24
#error "FELUCCA_UAC_TONE is stereo 16-bit"
#endif
    {   /* bench: triangles of period 100 frames instead of the music (L, R in antiphase): every
         * recorded frame must follow the one before, so a lost or repeated frame shows exactly */
        static uint32_t ph;
        for (i = 0; i < n; i++, ph = ph == 99u ? 0u : ph + 1u) {
            uint32_t q = ph < 50u ? ph : 100u - ph, q2 = ph < 50u ? 50u - ph : ph - 50u;
            ua_smp_t *d = &ua_ring[((w + i) & (UA_N - 1u)) * 2u];
            d[0] = (int16_t)((int32_t)q * 1000 - 25000);
            d[1] = (int16_t)((int32_t)q2 * 1000 - 25000);
        }
        (void)out;
    }
#else
    for (i = 0; i < n; i++) {
        ua_smp_t *d = &ua_ring[((w + i) & (UA_N - 1u)) * UA_CH];
        const int32_t *x = out ? out + UA_CH * i : 0;
        uint32_t c;
        for (c = 0; c < UA_CH; c++)
            d[c] = x ? (ua_smp_t)UA_SMP(x[c]) : 0;
    }
#endif
    RING_PUBLISH();
    ua_w = w + n;
}
#endif

#endif

/* the next packet's frames by the FM-1's clock (above): the pattern at the rate trimmed to the host's ms, and the
 * loop's correction, a frame at most. Its phase is kept from the stream's first packet, silence or not, as the host
 * measures the device's rate from then on */
static uint32_t uac_size(void)
{
    uint32_t now = FELUCCA_UAC_CLOCK(), dt = now - uac.t_prev, n = 44u, whole;
    int32_t frac = UA_FRAC44;
#if FELUCCA_UAC_48K
    if (uac.r48) {                                      /* 48 frames a ms and the I2S clock's 0.0192 */
        n = 48u;
        frac = UA_FRAC48;
    }
#endif
    if (!uac.lock || dt > UA_DT_MAX) {                  /* the stream's first packet, or one after a long wait */
        if (uac.lock)
            uac.relocks++;
        uac.lock = 1;
        uac.due = 0;
        uac.wn = 0;
        uac.wmin = 0x7FFFFFFF;
        uac.pend = 0;
        uac.since = 0;
        uac.part = 0;
        uac.t_prev = now;
        dt = 0;
    }
#if FELUCCA_UAC_48K
    if (uac.r48) {                                      /* 5 frames every UA_TPF48 ticks: whole blocks of them, and
                                                         * the frames of the block begun (part of them counted) */
        uint32_t blocks = dt / UA_TPF48, part = (dt - blocks * UA_TPF48) * 5u / UA_TPF48;
        uac.t_prev += blocks * UA_TPF48;
        whole = blocks * 5u + part - uac.part;
        uac.part = (uint8_t)part;
    } else
#endif
    {
        whole = dt / UA_TPF;
        uac.t_prev += whole * UA_TPF;                   /* (the ticks past the whole frames stay for the next) */
    }
    uac.due += (int32_t)whole;
    if (uac.due < uac.wmin)
        uac.wmin = uac.due;
    if (++uac.wn == UA_WIN) {                           /* a window: its least phase */
        int32_t d = uac.wmin - uac.ref;
        uac.wn = 0;
        if (uac.lock == 1 || d > UA_JUMP || d < -UA_JUMP) {   /* the reference: the first window's, or after a jump */
            if (uac.lock == 2)
                uac.relocks++;
            uac.lock = 2;
            uac.ref = uac.wmin;
            uac.pend = 0;
        } else
            uac.pend = (int8_t)(d > UA_BAND ? 1 : d < -UA_BAND ? -1 : 0);
        uac.wmin = 0x7FFFFFFF;
    }
    uac.acc += frac + uac.trim;
    while (uac.acc >= (int32_t)UA_ACC_ONE) {
        uac.acc -= (int32_t)UA_ACC_ONE;
        n++;
    }
    while (uac.acc < 0) {
        uac.acc += (int32_t)UA_ACC_ONE;
        n--;
    }
    if (uac.since < UA_GAP)
        uac.since++;
    if (uac.since >= UA_GAP && uac.pend) {            /* behind the clock: a frame more; ahead: one fewer */
        n = (uint32_t)((int32_t)n + uac.pend);
        uac.trim = (int16_t)(uac.trim + uac.pend * UA_TRIM_STEP);
        if (uac.trim > UA_TRIM_MAX)
            uac.trim = UA_TRIM_MAX;
        if (uac.trim < -UA_TRIM_MAX)
            uac.trim = -UA_TRIM_MAX;
        if (uac.pend > 0)
            uac.adj_up++;
        else
            uac.adj_down++;
        uac.pend = 0;
        uac.since = 0;
    }
    if (n > UA_MAXF)
        n = UA_MAXF;
    uac.due -= (int32_t)n;
    return n;
}

/* the next packet into d: uac_size's frames, from the ring or, until it is primed, silence. An empty ring holds the
 * last frame, and UA_HOLD frames more: the frames in hand grow by as many (the stream keeps the clock's time), so the
 * next render as late is in time. Returns the frame count */
#if UA_MC
static uint32_t uac_packet(uint32_t *dw)              /* as below, FELUCCA_UAC_CH samples a frame */
{
    int16_t *d = (int16_t *)dw;
    uint32_t n = uac_size(), r, fill, h, k, i, c;
    if (!uac.go) {
        for (i = 0; i < n * UAC_NCH; i++)
            d[i] = 0;
        return n;
    }
    r = ua_r;
    fill = ua_fill(ua_w, r);
    RING_PUBLISH();
    if (fill < uac.fill_lo)
        uac.fill_lo = fill;
    if (fill > uac.fill_hi)
        uac.fill_hi = fill;
    h = uac.owe < n ? uac.owe : n;                      /* the rest of a hold first */
    uac.owe -= h;
    for (i = 0; i < h; i++)
        for (c = 0; c < UAC_NCH; c++)
            d[i * UAC_NCH + c] = ua_last[c];
    k = fill < n - h ? fill : n - h;
    {
        uint32_t at = r % UA_N, m = UA_N - at < k ? UA_N - at : k;   /* to the ring's end, then from its start */
        const uint32_t *f = (const uint32_t *)(const void *)ua_ring[at];
        uint32_t *o = (uint32_t *)(void *)(d + h * UAC_NCH);
        for (i = 0; i < m * UAC_NCH / 2u; i++)          /* (two samples a word: UAC_NCH even) */
            o[i] = f[i];
        f = (const uint32_t *)(const void *)ua_ring[0];
        for (; i < k * UAC_NCH / 2u; i++)
            o[i] = f[i - m * UAC_NCH / 2u];
        i = h + k;
    }
    if (i < n) {                                        /* dry: hold the last frame, UA_HOLD more (above) */
        const int16_t *last = i ? d + (i - 1u) * UAC_NCH : ua_last;
        uac.underruns++;
        uac.owe = UA_HOLD;
        for (; i < n; i++)
            for (c = 0; c < UAC_NCH; c++)
                d[i * UAC_NCH + c] = last[c];
    }
    for (c = 0; c < UAC_NCH; c++)
        ua_last[c] = d[(n - 1u) * UAC_NCH + c];
    RING_PUBLISH();
    ua_r = ua_add(r, k);
    uac.pkts++;
    uac.frames += n;
    return n;
}
#else
#if UA_IN_PLAIN
static uint32_t uac_packet(uint32_t *d)
{
    uint32_t n = uac_size(), r, fill, h, k, i;
    if (!uac.go || uac.ring48 != uac.r48) {             /* not primed (at this rate) yet */
        for (i = 0; i < n; i++)
            d[i] = 0;
        return n;
    }
    r = ua_r;
    fill = ua_w - r;
    RING_PUBLISH();                                     /* the slots after the index */
    if (fill < uac.fill_lo)
        uac.fill_lo = fill;
    if (fill > uac.fill_hi)
        uac.fill_hi = fill;
    h = uac.owe < n ? uac.owe : n;                      /* the rest of a hold first */
    uac.owe -= h;
    for (i = 0; i < h; i++)
        d[i] = uac.last;
    k = fill < n - h ? fill : n - h;
    for (i = 0; i < k; i++)
        d[h + i] = ua_ring[(r + i) & (UA_N - 1u)];
    i = h + k;
    if (i < n) {                                        /* dry: hold the last frame, UA_HOLD more (above) */
        uint32_t last = i ? d[i - 1u] : uac.last;
        uac.underruns++;
        uac.owe = UA_HOLD;
        for (; i < n; i++)
            d[i] = last;
    }
    uac.last = d[n - 1u];
    RING_PUBLISH();
    ua_r = r + k;
    uac.pkts++;
    uac.frames += n;
    return n;
}
#else
/* one frame of the ring to the wire at p, little-endian: 16-bit, or with FELUCCA_UAC_24 in alt 2 (wide) 24-bit */
static inline uint8_t *uac_put(uint8_t *p, const ua_smp_t *f, uint32_t wide)
{
#if FELUCCA_UAC_24
    uint32_t c;
    for (c = 0; c < UA_CH; c++) {
        int32_t v = f[c];
        if (wide) {
            *p++ = (uint8_t)v;
            *p++ = (uint8_t)(v >> 8);
            *p++ = (uint8_t)(v >> 16);
            continue;
        }
        v = uac_in16(v);
        *p++ = (uint8_t)v;
        *p++ = (uint8_t)(v >> 8);
    }
    return p;
#else
    (void)wide;
    __builtin_memcpy(p, f, UA_CH * 2u);                 /* the ring's frame is the packet's (16-bit, little-endian) */
    return p + UA_CH * 2u;
#endif
}

/* the next packet into d: uac_size's frames (the pattern alone with FELUCCA_UAC_HOST_CLOCK), from the ring or, until
 * it is primed, silence. An empty ring holds the last frame (and UA_HOLD more, uac_size's clock). Returns the frame count (a
 * frame: UA_CH samples, 2 bytes each, 3 when wide). */
static uint32_t uac_packet(uint8_t *d, uint32_t wide)
{
    UA_RING();
    uint32_t n = 44u, r, fill, h, k, i, frac = UA_FRAC44;
#if FELUCCA_UAC_48K
    if (uac.r48) {                                      /* 48 frames a ms and the I2S clock's 0.0192 */
        n = 48u;
        frac = UA_FRAC48;
    }
#endif
#if FELUCCA_UAC_HOST_CLOCK
    uac.acc += (int32_t)frac;                           /* the pattern alone: uac_tap's step holds the fill */
    if (uac.acc >= (int32_t)UA_ACC_ONE) {
        uac.acc -= (int32_t)UA_ACC_ONE;
        n++;
    }
#else
    (void)frac;
    n = uac_size();
#endif
    if (!uac.go || uac.ring48 != uac.r48 || !UA_HAVE(ua_ring)) {   /* not primed (at this rate) yet; no ring */
        for (i = 0; i < n * UA_CH * (wide ? 3u : 2u); i++)
            d[i] = 0;
        return n;
    }
    r = ua_r;
    fill = ua_w - r;
    RING_PUBLISH();                                     /* the slots after the index */
    if (fill < uac.fill_lo)
        uac.fill_lo = fill;
    if (fill > uac.fill_hi)
        uac.fill_hi = fill;
    h = uac.owe < n ? uac.owe : n;                      /* the rest of a hold first */
    uac.owe = (uint8_t)(uac.owe - h);
    for (i = 0; i < h; i++)
        d = uac_put(d, uac.last, wide);
    k = fill < n - h ? fill : n - h;
    {
        const ua_smp_t *last = uac.last;
        uint32_t c;
        for (i = 0; i < k; i++) {
            last = &ua_ring[((r + i) & (UA_N - 1u)) * UA_CH];
            d = uac_put(d, last, wide);
        }
        if (h + k < n) {                                /* dry: hold the last frame, UA_HOLD more (above) */
            uac.underruns++;
#if !FELUCCA_UAC_HOST_CLOCK
            uac.owe = UA_HOLD;
#endif
            for (i = h + k; i < n; i++)
                d = uac_put(d, last, wide);
        }
        if (last != uac.last)
            for (c = 0; c < UA_CH; c++)
                uac.last[c] = last[c];
    }
    RING_PUBLISH();
    ua_r = r + k;
    uac.pkts++;
    uac.frames += n;
    return n;
}
#endif

#endif

/* TIMER5, 2 kHz, also nested in the render: the host takes one packet per 1 ms frame; queue the
 * next one as soon as the last has gone (TxPktRdy clear). Not paced on SOF-pending (see usb_poll).
 * The ring is fed only while the host reads: it often selects alt 1 long before its first IN token,
 * and a recorder may stop reading without going back to alt 0 (a packet waiting > 20 ms). */
#if FELUCCA_UAC_OUT
#if UA_OUT_PLAIN
static void uo_service(void)                            /* an EP4 OUT packet into the ring (TIMER5, nested too) */
{
    uint32_t csr, n, i, w, f;
    sie_wr(S_INDEX, 4);
    csr = sie_rd(S_RXCSR1) | (sie_rd(S_RXCSR2) << 8);
    if (!(csr & 1u))
        return;
    n = (sie_rd(S_RXCOUNT1) | (sie_rd(S_RXCOUNT2) << 8)) / 4u;
    if (n > UO_MAXF)
        n = UO_MAXF;
    fm1_usb_rx_sync();
    w = uo_w;
    f = w - uo_r;
    if (UO_N - f < n) {
        uo.overruns++;                                  /* nobody plays it (or the rates part): drop */
    } else {
        for (i = 0; i < n; i++)
            uo_ring[(w + i) & (UO_N - 1u)] = ep4rx[i];
        RING_PUBLISH();
        uo_w = w + n;
    }
    uo.pkts++;
    uo.frames += n;
    csr = (csr & ~0x164u) | 0x10u;
    sie_wr(S_RXCSR1, csr & 0xFFu);
    sie_wr(S_RXCSR2, csr >> 8);
}
#else
static void uo_service(void)                            /* an EP4 OUT packet into the ring (TIMER5, nested too) */
{
    UO_RING();
    uint32_t csr, n, i, w, f;
    sie_wr(S_INDEX, 4);
    csr = sie_rd(S_RXCSR1) | (sie_rd(S_RXCSR2) << 8);
    if (!(csr & 1u))
        return;
#if FELUCCA_UAC_OUT_24
    n = (sie_rd(S_RXCOUNT1) | (sie_rd(S_RXCOUNT2) << 8)) / (uo.alt == 2u ? 6u : 4u);   /* (alt 2: 3-byte subframes) */
#else
    n = (sie_rd(S_RXCOUNT1) | (sie_rd(S_RXCOUNT2) << 8)) / 4u;
#endif
    if (n > UO_MAXF)
        n = UO_MAXF;
    fm1_usb_rx_sync();
    w = uo_w;
    f = w - uo_r;
#if FELUCCA_UAC_OUT_FEEDBACK
    if (!uo.go) {                                       /* priming: uac_play_mix starts UO_TGT frames back from uo_w, */
        f = w - uo.w0;                                  /* whatever an old stream left before them */
        f = f < UO_TGT ? f : UO_TGT;
    }
#endif
    if (!UO_HAVE(uo_ring)) {
        /* (no ring lent yet: dropped, not counted) */
    } else if (UO_N - f < n) {
        uo.overruns++;                                  /* nobody plays it (or the rates part): drop */
    } else {
#if FELUCCA_UAC_OUT_24
        const uint8_t *p = (const uint8_t *)ep4rx;
        if (uo.alt == 2u)
            for (i = 0; i < n; i++, p += 6) {           /* 24-bit little-endian, sign-extended */
                uo_frm_t *d = &uo_ring[(w + i) & (UO_N - 1u)];
                d->l = (int32_t)((uint32_t)p[0] << 8 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 24) >> 8;
                d->r = (int32_t)((uint32_t)p[3] << 8 | (uint32_t)p[4] << 16 | (uint32_t)p[5] << 24) >> 8;
            }
        else
            for (i = 0; i < n; i++, p += 4) {           /* 16-bit, as 24 */
                uo_frm_t *d = &uo_ring[(w + i) & (UO_N - 1u)];
                d->l = (int32_t)(int16_t)(p[0] | p[1] << 8) * 256;
                d->r = (int32_t)(int16_t)(p[2] | p[3] << 8) * 256;
            }
#else
        for (i = 0; i < n; i++)
            uo_ring[(w + i) & (UO_N - 1u)] = ep4rx[i];
#endif
        RING_PUBLISH();
        uo_w = w + n;
    }
#if FELUCCA_UAC_OUT_FEEDBACK
    if (uo.go)                                          /* the fill as the packets come, for the feedback */
        uo.fill_q8 += ((int32_t)((uo_w - uo_r) << 8) - uo.fill_q8) / 32;
#endif
    uo.underruns += uo.und_pend;                        /* (more came after it ran dry: a gap in what it plays) */
    uo.und_pend = 0;
    uo.pkts++;
    uo.frames += n;
    csr = (csr & ~0x164u) | 0x10u;
    sie_wr(S_RXCSR1, csr & 0xFFu);
    sie_wr(S_RXCSR2, csr >> 8);
}
#endif

#if UA_OUT_PLAIN
/* audio ISR, after the render: the computer's audio added to the block (24-bit, as the I2S takes it), at gain
 * gain_q12 (4096 = as sent), resampled from the ring: the read rate is 44.1 / 44.1176 kHz and steered by how far
 * the fill at a render's start is from UO_TGT (a frame's error: 1/32768 faster or slower) */
static __attribute__((noinline)) void uac_play(int32_t *out, uint32_t n, uint32_t gain_q12)
{
    uint32_t r = uo_r, fill = uo_w - r, i, pos = uo.pos, step;
    float g = (float)gain_q12 * (1.0f / 16.0f), adj;            /* 16-bit to 24-bit (x 256), x gain / 4096 */
    if (!uo.alt || !usb.config) {
        uo.go = 0;
        return;
    }
    if (!uo.go) {
        if (fill < UO_TGT)
            return;                                     /* priming */
        r = uo_w - UO_TGT;                              /* start at the target: the excess is old */
        pos = 0;
        uo.err = 0.0f;
        uo.go = 1;
        fill = UO_TGT;
    }
    uo.err += ((float)fill - (float)UO_TGT - uo.err) * 0.05f;
    adj = uo.err * 2.0f;                                /* Q16 per frame of error */
    adj = adj > 400.0f ? 400.0f : adj < -400.0f ? -400.0f : adj;
    step = (uint32_t)(65510.0f + adj);                  /* 65536 x 44100 / 44117.6 */
    if (fill < (uint32_t)(((uint64_t)n * step) >> 16) + 2u) {
        uo.underruns++;                                 /* ran dry: prime again */
        uo.go = 0;
        RING_PUBLISH();
        uo_r = uo_w;
        return;
    }
    RING_PUBLISH();
    for (i = 0; i < n; i++) {
        uint32_t k = pos >> 16, a = uo_ring[(r + k) & (UO_N - 1u)], b = uo_ring[(r + k + 1u) & (UO_N - 1u)];
        float fr = (float)(pos & 0xFFFFu) * (1.0f / 65536.0f);
        float l = (float)(int16_t)a + ((float)(int16_t)b - (float)(int16_t)a) * fr;
        float rr = (float)(int16_t)(a >> 16) + ((float)(int16_t)(b >> 16) - (float)(int16_t)(a >> 16)) * fr;
        out[2u * i] += (int32_t)(l * g);
        out[2u * i + 1u] += (int32_t)(rr * g);
        pos += step;
    }
    RING_PUBLISH();
    uo_r = r + (pos >> 16);
    uo.pos = pos & 0xFFFFu;
}
/* for a firmware that mixes the computer's audio itself (a mixer track of its own): n stereo frames of it into lr, at
 * unity, as uac_play adds them into silence; 1 = lr holds the computer's audio (uo.go: it was mixed), 0 = none (not
 * streaming, priming, ran dry): lr is silence */
static __attribute__((unused)) int uac_play_take(int32_t *lr, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < 2u * n; i++)
        lr[i] = 0;
    uac_play(lr, n, 4096u);
    return uo.go;
}
#else
#if FELUCCA_UAC_OUT_FEEDBACK
/* audio ISR, after the render: the computer's audio added to the block (24-bit, as the I2S takes it), at gain
 * gain_q12 (4096 = as sent), n frames of the ring as they came: the computer sends at the I2S's rate (the feedback,
 * uo_fb_service), so nothing is resampled. set: written into the block instead of added (silence while there is
 * none). Returns 1 when the block has the computer's audio */
static __attribute__((noinline)) int uac_play_mix(int32_t *out, uint32_t n, uint32_t gain_q12, uint32_t set)
{
    UO_RING();
    uint32_t r = uo_r, fill = uo_w - r, i;
#if FELUCCA_UAC_OUT_24
    float g = (float)gain_q12 * (1.0f / 4096.0f);       /* the ring's 24 bits, x gain / 4096 */
#else
    float g = (float)gain_q12 * (1.0f / 16.0f);         /* 16-bit to 24-bit (x 256), x gain / 4096 */
#endif
    if (!uo.alt || !usb.config || !UO_HAVE(uo_ring)) {
        uo.go = 0;
        goto none;
    }
    if (!uo.go) {
        if (uo_w - uo.w0 < UO_TGT)
            goto none;                                  /* priming: UO_TGT frames of this stream's */
        r = uo_w - UO_TGT;                              /* start at the target: the excess is old */
        uo_r = r;                                       /* (before go: uo_service's fill is from it then) */
        RING_PUBLISH();
        uo.go = 1;
        fill = UO_TGT;
    }
    if (fill < n) {
        uo.und_pend++;                                  /* ran dry: prime again (a loss if more comes: uo_service) */
        uo.go = 0;
        uo.fill_q8 = (int32_t)(UO_TGT << 8);
        uo.w0 = uo_w;
        RING_PUBLISH();
        uo_r = uo_w;
        goto none;
    }
    RING_PUBLISH();
    for (i = 0; i < n; i++) {
#if FELUCCA_UAC_OUT_24
        const uo_frm_t *a = &uo_ring[(r + i) & (UO_N - 1u)];
        int32_t l = (int32_t)((float)a->l * g), rr = (int32_t)((float)a->r * g);
#else
        uint32_t a = uo_ring[(r + i) & (UO_N - 1u)];
        int32_t l = (int32_t)((float)(int16_t)a * g), rr = (int32_t)((float)(int16_t)(a >> 16) * g);
#endif
        if (set) {
            out[2u * i] = l;
            out[2u * i + 1u] = rr;
        } else {
            out[2u * i] += l;
            out[2u * i + 1u] += rr;
        }
    }
    RING_PUBLISH();
    uo_r = r + n;
    return 1;
none:
    if (set)
        for (i = 0; i < 2u * n; i++)
            out[i] = 0;
    return 0;
}

/* TIMER5: the feedback queued on EP3 IN whenever the last one has gone (the computer reads it every 2^UO_FB_REFRESH
 * ms): the I2S's nominal rate less the fill's distance from UO_TGT (UO_FB_NOM) */
static void uo_fb_service(void)
{
    int32_t c = ((int32_t)(UO_TGT << 8) - uo.fill_q8) / 16;
    uint32_t v;
    sie_wr(S_INDEX, 3);
    if (sie_rd(S_TXCSR1) & 0x01u)
        return;                                         /* the last one still queued */
    c = c > UO_FB_MAX ? UO_FB_MAX : c < -UO_FB_MAX ? -UO_FB_MAX : c;
    v = (uint32_t)((int32_t)UO_FB_NOM + c);
    uo_fb[0] = (uint8_t)v;
    uo_fb[1] = (uint8_t)(v >> 8);
    uo_fb[2] = (uint8_t)(v >> 16);
    uo.fb = v;
    fm1_usb_ep_send(3, uo_fb, 3);
    sie_wr(S_TXCSR1, 0x01);                             /* TxPktRdy */
    uo.fbs++;
}
#else
/* audio ISR, after the render: the computer's audio added to the block (24-bit, as the I2S takes it), at gain
 * gain_q12 (4096 = as sent), resampled from the ring: the read rate is 44.1 / 44.1176 kHz and steered by how far
 * the fill at a render's start is from UO_TGT (a frame's error: 1/32768 faster or slower). set: written into the
 * block instead of added (silence while there is none). Returns 1 when the block has the computer's audio */
static __attribute__((noinline)) int uac_play_mix(int32_t *out, uint32_t n, uint32_t gain_q12, uint32_t set)
{
    UO_RING();
    uint32_t r = uo_r, fill = uo_w - r, i, pos = uo.pos, step;
#if FELUCCA_UAC_OUT_24
    float g = (float)gain_q12 * (1.0f / 4096.0f), adj;          /* the ring's 24 bits, x gain / 4096 */
#else
    float g = (float)gain_q12 * (1.0f / 16.0f), adj;            /* 16-bit to 24-bit (x 256), x gain / 4096 */
#endif
    if (!uo.alt || !usb.config || !UO_HAVE(uo_ring)) {
        uo.go = 0;
        goto none;
    }
    if (!uo.go) {
        if (fill < UO_TGT)
            goto none;                                  /* priming */
        r = uo_w - UO_TGT;                              /* start at the target: the excess is old */
        pos = 0;
        uo.err = 0.0f;
        uo.go = 1;
        fill = UO_TGT;
    }
    uo.err += ((float)fill - (float)UO_TGT - uo.err) * 0.05f;
    adj = uo.err * 2.0f;                                /* Q16 per frame of error */
    adj = adj > 400.0f ? 400.0f : adj < -400.0f ? -400.0f : adj;
    step = (uint32_t)(65510.0f + adj);                  /* 65536 x 44100 / 44117.6 */
    if (fill < (uint32_t)(((uint64_t)n * step) >> 16) + 2u) {
        uo.und_pend++;                                  /* ran dry: prime again (a loss if more comes: uo_service) */
        uo.go = 0;
        RING_PUBLISH();
        uo_r = uo_w;
        goto none;
    }
    RING_PUBLISH();
    for (i = 0; i < n; i++) {
#if FELUCCA_UAC_OUT_24
        uint32_t k = pos >> 16;
        const uo_frm_t *a = &uo_ring[(r + k) & (UO_N - 1u)], *b = &uo_ring[(r + k + 1u) & (UO_N - 1u)];
        float fr = (float)(pos & 0xFFFFu) * (1.0f / 65536.0f);
        float l = (float)a->l + ((float)b->l - (float)a->l) * fr;
        float rr = (float)a->r + ((float)b->r - (float)a->r) * fr;
#else
        uint32_t k = pos >> 16, a = uo_ring[(r + k) & (UO_N - 1u)], b = uo_ring[(r + k + 1u) & (UO_N - 1u)];
        float fr = (float)(pos & 0xFFFFu) * (1.0f / 65536.0f);
        float l = (float)(int16_t)a + ((float)(int16_t)b - (float)(int16_t)a) * fr;
        float rr = (float)(int16_t)(a >> 16) + ((float)(int16_t)(b >> 16) - (float)(int16_t)(a >> 16)) * fr;
#endif
        if (set) {
            out[2u * i] = (int32_t)(l * g);
            out[2u * i + 1u] = (int32_t)(rr * g);
        } else {
            out[2u * i] += (int32_t)(l * g);
            out[2u * i + 1u] += (int32_t)(rr * g);
        }
        pos += step;
    }
    RING_PUBLISH();
    uo_r = r + (pos >> 16);
    uo.pos = pos & 0xFFFFu;
    return 1;
none:
    if (set)
        for (i = 0; i < 2u * n; i++)
            out[i] = 0;
    return 0;
}
#endif
/* the computer's audio mixed into the block at MASTER: the platform's place for it (after the render) */
static void uac_play(int32_t *out, uint32_t n, uint32_t gain_q12) { (void)uac_play_mix(out, n, gain_q12, 0); }
/* instead of uac_play, for a firmware that mixes the computer's audio itself (a mixer track of its own): n stereo
 * frames of it into lr, at unity (4096: a 16-bit sample << 8, as the I2S takes it), resampled to the I2S clock as
 * uac_play does. Once a render with the render's n (the servo counts on it), from the audio ISR. 1 = lr holds the
 * computer's audio; 0 = none (not streaming, priming, ran dry): lr is silence */
static __attribute__((unused)) int uac_play_take(int32_t *lr, uint32_t n) { return uac_play_mix(lr, n, 4096u, 1); }
#endif
#endif

static void uac_service(void)
{
    uint32_t csr, n;
#if FELUCCA_UAC_OUT
    if (usb.up && usb.config && uo.alt)
        uo_service();
#endif
#if FELUCCA_UAC_OUT_FEEDBACK
    if (usb.up && usb.config && uo.alt)
        uo_fb_service();
#endif
    if (!usb.up || !usb.config || !uac.alt)
        return;
    sie_wr(S_INDEX, 4);
    csr = sie_rd(S_TXCSR1);
    if (csr & 0x01u) {                                  /* still queued */
        if (uac.wait < 255u && ++uac.wait == FELUCCA_UAC_STALL_POLLS && uac.flowing) {
            uac.flowing = 0;
            uac.stalls++;
#if !UA_IN_PLAIN
            uac.ovr_pend = 0;                           /* (it stopped reading: what was dropped meanwhile lost nothing) */
#endif
        }
        return;
    }
    if (uac.queued) {
        if (csr & 0x04u)
            uac.missed++;                               /* UnderRun: an IN token found nothing queued */
        uac.flowing = 1;
#if !UA_IN_PLAIN
        uac.overruns += uac.ovr_pend;                   /* (it reads on: the blocks dropped are a gap in the recording) */
        uac.ovr_pend = 0;
#endif
    }
    uac.queued = 1;
    uac.wait = 0;
#if UA_MC || UA_IN_PLAIN
    n = uac_packet(ep4tx);
    fm1_usb_ep4_send(ep4tx, n * 2u * UAC_NCH);
#else
    n = uac_packet((uint8_t *)ep4tx, uac.alt == 2u);
    fm1_usb_ep4_send(ep4tx, n * UA_CH * (uac.alt == 2u ? 3u : 2u));
#endif
    sie_wr(S_TXCSR1, 0x01);                             /* TxPktRdy (and UnderRun cleared) */
}
#endif

#if FELUCCA_MIDI_RT
/* Real-time messages sent at once (usb_midi_rt): a MIDI clock tick loaded into EP1 the moment it is due, not at the next
 * 2 kHz poll, and not held while the poll waits for the audio render (usb_poll never runs nested; a tick could wait out
 * a whole render). Measured on a Mac before this: a clock out through midi_out_q, released by a 1 kHz tick, scattered
 * 418 us (sd) around its own line, up to 1.5 ms. Called from the TIMER5 tick (0.1 ms), nested in the render or not:
 * the only other SIE user, usb_poll, runs in the same interrupt, never inside it, and sets INDEX itself. One producer
 * (the tick), so the ring needs no lock. While a packet is still on EP1 (TxPktRdy), what is waiting goes at the next
 * tick (usb_midi_rt_kick) or the poll, ahead of midi_out_q. USB-MIDI packets are whole events, so one between two of
 * a SysEx message is legal. When the host does not read (tx_deaf), the ring is dropped with midi_out_q (ep1_tx). */
static uint8_t ep1rt[4u * RTN] __attribute__((aligned(4)));
static void usb_midi_rt_kick(void)
{
    uint32_t csr, n = 0;
    if (rt_qr == rt_qw)
        return;
#if FELUCCA_USB_HOST
    if (uh.on) {
        rt_qr = rt_qw;                                  /* (no computer on the port: nothing to send to) */
        return;
    }
#endif
#if FELUCCA_MSC
    if (usb_msc_on) {
        rt_qr = rt_qw;
        return;
    }
#endif
    if (!usb.up || !usb.config) {
        rt_qr = rt_qw;
        return;
    }
    sie_wr(S_INDEX, 1);
    csr = sie_rd(S_TXCSR1);
    if (csr & 0x01u)
        return;                                         /* the last packet not taken yet: the next tick */
    tx_deaf = 0;                                        /* the host took the packet: it reads (as in ep1_tx) */
    tx_wait = 0;
    if (csr & 0x80u)
        sie_wr(S_TXCSR1, csr & ~0x80u);
    while (rt_qr != rt_qw) {
        uint32_t pkt = rt_q[rt_qr % RTN];
        ep1rt[n] = (uint8_t)pkt;
        ep1rt[n + 1] = (uint8_t)(pkt >> 8);
        ep1rt[n + 2] = (uint8_t)(pkt >> 16);
        ep1rt[n + 3] = (uint8_t)(pkt >> 24);
        n += 4u;
        rt_qr++;
    }
    fm1_usb_ep_send(1, ep1rt, n);
    sie_wr(S_TXCSR1, sie_rd(S_TXCSR1) | 0x01u);
    usb.tx_pkts++;
}
/* a USB-MIDI packet (a clock tick, Start, Stop) to the computer now; 0 when 8 are already waiting (dropped) */
static __attribute__((unused)) int usb_midi_rt(uint32_t pkt)
{
    if (rt_qw - rt_qr >= RTN)
        return 0;
    rt_q[rt_qw % RTN] = pkt;
    rt_qw++;
    usb_midi_rt_kick();
    return 1;
}
#endif

static void usb_poll(void)                              /* TIMER5 ISR, 2 kHz */
{
    uint32_t iu, it, ir;
#if FELUCCA_USB_HOST
    if (uh.on) {
#if FELUCCA_MIDI_STAMP
        midi_rx_t = usb_peek_t;                         /* (a device's MIDI in: the poll's time) */
#endif
        if (uh.run) uh_poll();
        return;
    }
#endif
    if (!usb.up)
        return;
    if (fm1_usb_sof_take()) {                           /* SOF pending: NOT a reliable "host is there" */
                                                        /* (it keeps firing with the cable out) */
        usb.sof_seen++;
        usb.no_sof = 0;
    } else if (++usb.no_sof > usb.max_gap) {
        usb.max_gap = usb.no_sof;
    }
    if (usb.config) {                                   /* the frame number only advances with a real host */
        static uint32_t tick;
        if (++tick >= 20u) {                            /* every 20 polls = 10 ms */
            uint16_t f;
            tick = 0;
            f = (uint16_t)(sie_rd(S_FRAME1) | (sie_rd(S_FRAME2) & 7u) << 8);
            if (f != usb.frame) {
                usb.frame = f;
                usb.frame_same = 0;
                usb.suspended = 0;
            } else if (++usb.frame_same >= 10u && !usb.suspended) {   /* 100 ms frozen: unplugged / host asleep */
                usb.suspended = 1;
                usb.frame_stalls++;
            }
        }
    }
    iu = sie_rd(S_INTRUSB);
#ifdef FELUCCA_LOADER
    it = sie_rd(S_INTRTX1) | sie_rd(S_INTRTX2) << 8;    /* the update loader reads both */
    ir = sie_rd(S_INTRRX1) | sie_rd(S_INTRRX2) << 8;
#else
    it = sie_rd(S_INTRTX1);                             /* EP0..7 only: skipping INTR*2 saves */
    ir = sie_rd(S_INTRRX1);                             /* 2 SIE round trips per poll */
#endif
#if FELUCCA_MIDI_STAMP
    ir |= usb_rx_seen;                                  /* (what the peeks read since the last poll) */
    usb_rx_seen = 0;
#endif
    if (iu & 0x01u) {                                   /* suspend: the bus went idle */
        usb.suspends++;
        usb.suspended = 1;
    }
    if (iu & 0x02u)                                     /* resume */
        usb.suspended = 0;
    if (iu & 0x04u) {                                   /* bus reset */
        usb.resets++;
        sie_index = 0xFFu;
        sie_wr(S_FADDR, 0);
        usb.config = 0;
        usb.suspended = 0;
        mo_r = mo_w;                                    /* nothing stale for the next host */
        usb.sx_on = 0;
#if FELUCCA_OTA
        so_r = so_w;
        sx_collect = 0;
#endif
#if FELUCCA_MIRROR
        mr_r = mr_w;
        tx_mid = 0;
#endif
        usb.e0_tx = 0;
        usb.has_pend_addr = 0;
        usb.rx_pend = 0;
#if FELUCCA_MIDI_STAMP
        ep1_seen = 0;
#endif
#if FELUCCA_CDC
        cdc.dtr = 0;
        cdc.e0_rx = 0;
        cdc.rx_pend = 0;
#endif
#if FELUCCA_MSC
        msi_r = msi_w;
        msc_tx_n = 0;
        msc_rx_pend = 0;
#endif
#if FELUCCA_UAC
        uac.e0_rx = 0;
        uac_stream(0);
        uac_rate_set(UAC_RATE);
#endif
#if FELUCCA_UAC_OUT
        uo_stream(0);
#endif
        fm1_usb_ep0_buf(ep0buf);
        sie_wr(S_INTRUSBE, 0x07);
        sie_wr(S_INTRTX1E, 0x01);
        sie_wr(S_INTRRX1E, 0);
    }
    if (it & 0x01u)
        ep0_service();
#if FELUCCA_MSC
    if (usb_msc_on) {                                   /* disk mode: EP3 only */
        if (usb.config) {
            if (ir & 0x08u)
                msc_rx_pend = 1;
            if (msc_rx_pend)
                msc_ep3_rx();
            msc_ep3_tx();
        }
        return;
    }
#endif
    if (ir & 0x02u)
        usb.rx_pend = 1;
    if (usb.rx_pend)                                    /* not every poll: 3 SIE round trips each */
        ep1_rx();
#if FELUCCA_MIDI_RT
    if (usb.config) {
        usb_midi_rt_kick();                             /* (real-time first: usb_midi_rt) */
        ep1_tx();
    }
#else
    if (usb.config)
        ep1_tx();
#endif
#if FELUCCA_CDC
    if (usb.config && usb_cdc_on) {
        if (ir & 0x08u)
            cdc.rx_pend = 1;
        if (cdc.rx_pend)                                /* not every poll: 3 SIE round trips each */
            ep3_rx();
        if (cdc.dtr)
            ep3_tx();
    }
#endif
#if FELUCCA_UAC
    uac_service();
#endif
}

static void usb_start(void)                             /* boot, or main-loop retry while usb.up == 0 */
{
    usb.timeouts = 0;
    sie_index = 0xFFu;
    fm1_usb_reset();                                    /* reset whatever the ROM left */
    fm1_delay_ms(25);
    fm1_usb_attach(ep0buf);
    sie_wr(S_POWER, 0x60);
    sie_wr(S_INTRUSBE, 0x07);                           /* suspend, resume, reset (polled) */
    sie_wr(S_INTRTX1E, 0x01);
    sie_wr(S_INTRTX2E, 0);
    sie_wr(S_INTRRX1E, 0);
    sie_wr(S_INTRRX2E, 0);
    fm1_usb_ep_enable(0x1Fu);
    usb.up = usb.timeouts < 3u;                         /* a dead SIE leaves USB off */
}

/* Powered on without a cable, the SIE does not answer and usb_start leaves
 * USB off: retry once a second from the main loop so a cable plugged in
 * later still enumerates. */
static void usb_retry(uint32_t now_ms)
{
#if FELUCCA_USB_HOST
    if (uh.on)
        return;
#endif
    if (usb.up || usb.detached || now_ms - usb_retry_ms < 1000u)
        return;
    usb_retry_ms = now_ms;
    usb.retries++;
    usb_start();
}

static void usb_detach(void)
{
    usb.detached = 1;
    usb.up = 0;
    RING_PUBLISH();
#if FELUCCA_USB_HOST
    if (uh.on) uh_session_end();
    uh.want = uh.on = 0;
    RING_PUBLISH();
#endif
    fm1_usb_off();
}

#if FELUCCA_CDC
/* Present the device with or without the serial console (main loop; at boot before usb_start, or later
 * from a setting). A change while attached drops off the bus; usb_retry attaches again within a second
 * with the other descriptors, and the host enumerates the device afresh. */
static __attribute__((unused)) void usb_cdc_switch(uint32_t on)
{
    on = on != 0;
    if (on == usb_cdc_on)
        return;
    if (usb.up) {
        usb.up = 0;                                     /* usb_poll (TIMER5) stops before the SIE goes */
        RING_PUBLISH();
        fm1_usb_off();
    }
    usb.config = 0;
    cdc.dtr = 0;                                        /* the console stops writing */
    cdc.rx_pend = 0;
    usb_cdc_on = (uint8_t)on;
}
#endif
