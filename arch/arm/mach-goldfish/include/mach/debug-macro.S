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

	.macro	addruart, rp, rv, rtmp
	ldr		\rp, =GOLDFISH_TTY_PUT_CHAR_PHYS
	ldr		\rv, =GOLDFISH_TTY_PUT_CHAR_BASE
	.endm

	.macro	senduart,rd,rx
	teq	\rx, #0
	strne	\rd, [\rx]
	.endm

	.macro	waituart,rd,rx
	.endm

	.macro	busyuart,rd,rx
	.endm
