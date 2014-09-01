/* arch/x86/include/asm/mach-goldfish/hardware.h
**
** Copyright (C) 2007 Google, Inc.
** Copyright (C) 2011 Intel, Corp.
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

#ifndef __X86_ARCH_HARDWARE_H
#define __X86_ARCH_HARDWARE_H

/*
 * Where in virtual memory the IO devices (timers, system controllers
 * and so on)
 */
#define GOLDFISH_IO_SIZE	0x00800000                 // How much?
#define GOLDFISH_IO_START	0xff000000                 // PA of IO

#define GOLDFISH_PDEV_BUS_BASE      (0x1000)
#define GOLDFISH_PDEV_BUS_END       (0x100)

#define GOLDFISH_TTY_BASE       (0x2000)

#endif
