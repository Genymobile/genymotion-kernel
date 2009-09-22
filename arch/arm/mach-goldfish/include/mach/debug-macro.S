/* arch/arm/mach-goldfish/include/mach/debug-macro.S
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: David Turner <digit@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <mach/hardware.h>

#define GOLDFISH_TTY_PUT_CHAR_PHYS (IO_START + GOLDFISH_TTY_BASE)
#define GOLDFISH_TTY_PUT_CHAR_BASE (IO_BASE + GOLDFISH_TTY_BASE)

	.macro	addruart,rx
	@ see if the MMU is enabled and select appropriate base address
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1
	ldreq	\rx, =GOLDFISH_TTY_PUT_CHAR_PHYS
	ldrne	\rx, =GOLDFISH_TTY_PUT_CHAR_BASE
	.endm

	.macro	senduart,rd,rx
	teq	\rx, #0
	strne	\rd, [\rx]
	.endm

	.macro	waituart,rd,rx
	.endm

	.macro	busyuart,rd,rx
	.endm
