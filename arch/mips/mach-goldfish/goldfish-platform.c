/* arch/mips/mach-goldfish/goldfish-platform.c
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/input.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/bootinfo.h>
#include <asm/mach-goldfish/irq.h>

int GOLDFISH_READY;

static struct resource goldfish_pdev_bus_resources[] = {
	{
		.start  = GOLDFISH_PDEV_BUS_BASE,
		.end    = GOLDFISH_PDEV_BUS_BASE + GOLDFISH_PDEV_BUS_END - 1,
		.flags  = IORESOURCE_IO,
	},
	{
		.start	= GOLDFISH_IRQ_BASE+GOLDFISH_IRQ_PBUS,
		.end	= GOLDFISH_IRQ_BASE+GOLDFISH_IRQ_PBUS,
		.flags	= IORESOURCE_IRQ,
	}
};

struct platform_device goldfish_pdev_bus_device = {
	.name = "goldfish_pdev_bus",
	.id = -1,
	.num_resources = ARRAY_SIZE(goldfish_pdev_bus_resources),
	.resource = goldfish_pdev_bus_resources
};


static int __init goldfish_setup_devinit(void)
{
	platform_device_register(&goldfish_pdev_bus_device);
	GOLDFISH_READY = 1;
	return 0;
}
device_initcall(goldfish_setup_devinit);


void __init prom_init(void)
{
	char *cmdline = (char *)fw_arg0;
	strcpy(arcs_cmdline, cmdline);
}

void plat_mem_setup(void)
{
	unsigned int ramsize = fw_arg1;
	add_memory_region(0x0, ramsize, BOOT_MEM_RAM);
}

void prom_free_prom_memory(void)
{
}

#define GOLDFISH_TTY_PUT_CHAR (*(volatile unsigned int *)0xbf002000)
void prom_putchar(int c)
{
	GOLDFISH_TTY_PUT_CHAR = c;
}

const char *get_system_type(void)
{
	return "MIPS-Goldfish";
}

