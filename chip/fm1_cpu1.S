/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments
 * Written against the public JieLi AC79 SDK (Apache-2.0); see NOTICE. */
/* Entry of the second core (CPU1), experimental. Same steps as the SDK's cpu1_start: the two stacks, the return
 * address, then rti into fm1_cpu1_main (the application's). _cpu1_ustack_top and _cpu1_sstack_top come from the
 * linker script. */
	.section .text.fm1_cpu1,"ax",@progbits
	.globl fm1_cpu1_entry
fm1_cpu1_entry:
	usp = _cpu1_ustack_top
	sp = _cpu1_sstack_top
	reti = fm1_cpu1_main
	rti
