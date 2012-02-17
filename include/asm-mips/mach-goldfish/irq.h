/* include/asm-mips/mach-goldfish/irq.h
**
** Copyright (C) 2007 Google, Inc.
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
*/

#ifndef __ASM_MACH_GOLDFISH_IRQ_H
#define __ASM_MACH_GOLDFISH_IRQ_H

/* 0..7 MIPS CPU interrupts */
#define MIPS_CPU_IRQ_BASE	0
#define MIPS_CPU_IRQ_PIC	2
#define MIPS_CPU_IRQ_FIQ	3 /* Not used? */
#define MIPS_CPU_IRQ_COMPARE	7

/* 8..39 Cascaded Goldfish PIC interrupts */
#define GOLDFISH_IRQ_BASE	8
#define GOLDFISH_IRQ_PBUS	1
#define GOLDFISH_IRQ_RTC	3
#define GOLDFISH_IRQ_TTY	4

#define NR_IRQS			40

#endif
