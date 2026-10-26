/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * Copyright (c) 2026 Julius Vitkauskas (optional entry/exit calls, firmware-supplied entries) */
/* IRQ entry wrappers: save what a C call may
 * clobber, call the plain C body, restore, csync, rti. Our clang never emits
 * the csync stock ISRs have before rti, so ISRs are not written in C directly.
 *
 * FM1_ISR name, body[, enter, leave]: enter and leave, if given, are plain C functions (no arguments) called just
 * before and just after body, at the same priority, e.g. to account the time a handler takes from what it interrupts.
 *
 * Optional: build this file with -DFM1_ISR_ENTRIES='"file"' (and -I for its directory) and it emits that file's
 * FM1_ISR lines instead of the two below. The file keeps isr_timer5 and isr_alnk0 (the names the firmware attaches)
 * and may add entries for vectors of its own. Nothing else changes: the vector table, the fatal stubs (fm1_vec.S) and
 * the second core (fm1_cpu1.S, no interrupts of its own) are untouched, and fm1_irq_attach still sets each priority.
 * The macro puts each entry in .text.<name> (XIP), like the defaults. Without FM1_ISR_ENTRIES the output is what it
 * was before the option existed. */
	.macro FM1_ISR name, body, enter=, leave=
	.section .text.\name,"ax",@progbits
	.globl \name
\name:
	[--sp] = {psr, rets, reti}
	[--sp] = {r3-r0}
	.ifnb \enter
	call \enter
	.endif
	call \body
	.ifnb \leave
	call \leave
	.endif
	{r3-r0} = [sp++]
	{psr, rets, reti} = [sp++]
	csync
	rti
	.endm

#ifdef FM1_ISR_ENTRIES
#include FM1_ISR_ENTRIES
#else
	FM1_ISR isr_timer5, fm1_timer5_irq
	FM1_ISR isr_alnk0, fm1_alnk0_irq
#endif
