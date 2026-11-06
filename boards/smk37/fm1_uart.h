/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Kerem Kilic (Ellic Studio)
 * The UART1 clock and baud setup follow the FM-1's MIDI IN driver (chip/fm1_uart.h):
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
/* SMK-37 TRS jack: MIDI OUT only. UART1 TX on PC7 at 31250 baud, one byte at a time from the scan's tick
 * (device/midi_trs_out.c keeps the ring and never waits). The jack has no MIDI IN; UART1 RX is never enabled:
 * PH8, the FM-1's MIDI IN pin, is matrix row 6 here (fm1_input.h). A build that includes the FM-1's MIDI IN
 * (device/midi_uart.c) on this board is refused below.
 *
 * Board facts (all FROM ANALYSIS of the stock v016 firmware, M. Geisler, fm1-to-smk37 docs/01-hardware.md §9 and
 * docs/08: not yet checked on a unit): IOMAP_CON3 bit 7 clear and bits [23:20] = 1 route UT1 TX onto an output
 * channel that reaches PC7; PC7 with its pull-up and pull-down set, output low, input enabled, as the stock
 * firmware leaves it; CON0 0x3400 (pendings cleared) then the baud and UTEN; a byte goes by writing BUF after
 * setting CON0 bit 13 (clear TPND); bit 15 (TPND) says it has gone. The UART clock and the baud divisor are the
 * FM-1's MIDI IN's (48 MHz PLL, BAUD 383). PC7 is the FM-1's LCD chip select: SMK-only.
 *
 *   fm1_uart1_tx_init()          before TIMER5 starts (an IOMAP read-modify-write)
 *   fm1_uart1_tx_ready()         the last byte has gone (or none was sent): the next may be written
 *   fm1_uart1_tx_byte(b)         one byte on its way
 */
#pragma once
#include <stdint.h>
#include "fm1_cc.h"

#define FM1_UT1_CON0   (*(volatile uint32_t *)0x12100u)
#define FM1_UT1_CON1   (*(volatile uint32_t *)0x12104u)
#define FM1_UT1_BAUD   (*(volatile uint32_t *)0x12108u)
#define FM1_UT1_BUF    (*(volatile uint8_t *)0x1210Cu)
#define FM1_UM_CLK_CON1  (*(volatile uint32_t *)0x10010u)   /* [11:10] UART clock: 1 = PLL48M */
#define FM1_UM_IOMAP3    (*(volatile uint32_t *)0x51028u)   /* UT1: b7 fixed IO, [23:20] TX output select */
#define FM1_UM_PC_OUT    (*(volatile uint32_t *)0x50080u)
#define FM1_UM_PC_DIR    (*(volatile uint32_t *)0x50088u)
#define FM1_UM_PC_DIE    (*(volatile uint32_t *)0x5008Cu)
#define FM1_UM_PC_PU     (*(volatile uint32_t *)0x50090u)
#define FM1_UM_PC_PD     (*(volatile uint32_t *)0x50094u)
#define FM1_UT1_TPND     0x8000u                            /* CON0 bit 15: the byte has gone */
#define FM1_UT1_CLR_TPND 0x2000u                            /* CON0 bit 13 */
#define FM1_UART1_RX 0                                      /* never: PH8 is a matrix row */
#define FM1_UART1_TX 1

static uint8_t fm1_uart1_sent;                              /* a byte has been written since init */

FM1_INLINE void fm1_uart1_tx_init(void)
{
    FM1_UT1_CON0 = 0x3400u;                                 /* off, pendings cleared */
    FM1_UT1_CON1 = 0;
    FM1_UM_CLK_CON1 = (FM1_UM_CLK_CON1 & ~(3u << 10)) | (1u << 10);
    FM1_UM_IOMAP3 &= ~0x80u;
    FM1_UM_IOMAP3 = (FM1_UM_IOMAP3 & ~(0xFu << 20)) | (1u << 20);   /* UT1 TX -> the output channel on PC7 */
    FM1_UM_PC_OUT &= ~(1u << 7);
    FM1_UM_PC_PU |= 1u << 7;
    FM1_UM_PC_PD |= 1u << 7;
    FM1_UM_PC_DIE |= 1u << 7;
    FM1_UM_PC_DIR &= ~(1u << 7);
    FM1_UT1_BAUD = 48000000u / 31250u / 4u - 1u;            /* 383 */
    __asm__ volatile("csync" ::: "memory");
    FM1_UT1_CON0 |= 1u;                                     /* UTEN; no RX, no interrupts */
    __asm__ volatile("csync" ::: "memory");
    fm1_uart1_sent = 0;
}

FM1_INLINE int fm1_uart1_tx_ready(void) { return !fm1_uart1_sent || (FM1_UT1_CON0 & FM1_UT1_TPND) != 0u; }

FM1_INLINE void fm1_uart1_tx_byte(uint8_t b)
{
    FM1_UT1_CON0 |= FM1_UT1_CLR_TPND;
    FM1_UT1_BUF = b;
    __asm__ volatile("csync" ::: "memory");
    fm1_uart1_sent = 1;
}

/* the FM-1's MIDI IN driver (chip/fm1_uart.h) must not be built for this board */
#define fm1_uart1_midi_init(ring, len) FM1_UART1_RX_IS_NOT_ON_THE_SMK37_PH8_IS_A_MATRIX_ROW
