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
#include <asm/irq_cpu.h>
#include <asm/mach-goldfish/irq.h>

static void __iomem *goldfish_interrupt;

void goldfish_mask_irq(unsigned int irq)
{
	writel(irq-GOLDFISH_IRQ_BASE,
	       goldfish_interrupt + GOLDFISH_INTERRUPT_DISABLE);
}

void goldfish_unmask_irq(unsigned int irq)
{
	writel(irq-GOLDFISH_IRQ_BASE,
	       goldfish_interrupt + GOLDFISH_INTERRUPT_ENABLE);
}

static struct irq_chip goldfish_irq_chip = {
	.name	= "goldfish",
	.mask	= goldfish_mask_irq,
	.mask_ack = goldfish_mask_irq,
	.unmask = goldfish_unmask_irq,
};

void goldfish_init_irq(void)
{
	unsigned int i;
	goldfish_interrupt = IO_ADDRESS(GOLDFISH_INTERRUPT_BASE);

	/*
	 * Disable all interrupt sources
	 */
	writel(1, goldfish_interrupt + GOLDFISH_INTERRUPT_DISABLE_ALL);

	for (i = GOLDFISH_IRQ_BASE; i < GOLDFISH_IRQ_BASE+32; i++) {
		set_irq_chip(i, &goldfish_irq_chip);
		set_irq_handler(i, handle_level_irq);
#if 0
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
#endif
	}
}

void goldfish_irq_dispatch(void)
{
	uint32_t irq;
	/*
	 * Disable all interrupt sources
	 */
	irq = readl(goldfish_interrupt + GOLDFISH_INTERRUPT_NUMBER);
	do_IRQ(GOLDFISH_IRQ_BASE+irq);
}

void goldfish_fiq_dispatch(void)
{
	panic("goldfish_fiq_dispatch");
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending = read_c0_cause() & read_c0_status() & ST0_IM;

	if (pending & CAUSEF_IP2)
		goldfish_irq_dispatch();
	else if (pending & CAUSEF_IP3)
		goldfish_fiq_dispatch();
	else if (pending & CAUSEF_IP7)
		do_IRQ(MIPS_CPU_IRQ_BASE + 7);
	else
		spurious_interrupt();
}

static struct irqaction cascade = {
	.handler	= no_action,
	.mask		= CPU_MASK_NONE,
	.name		= "cascade",
};

static void mips_timer_dispatch(void)
{
	do_IRQ(MIPS_CPU_IRQ_BASE + MIPS_CPU_IRQ_COMPARE);
}

void __init arch_init_irq(void)
{
	mips_cpu_irq_init();
	goldfish_init_irq();

	if (cpu_has_vint) {
		set_vi_handler(MIPS_CPU_IRQ_PIC, goldfish_irq_dispatch);
		set_vi_handler(MIPS_CPU_IRQ_PIC, goldfish_fiq_dispatch);
	}
	setup_irq(MIPS_CPU_IRQ_BASE+MIPS_CPU_IRQ_PIC, &cascade);
	setup_irq(MIPS_CPU_IRQ_BASE+MIPS_CPU_IRQ_FIQ, &cascade);

	if (cpu_has_vint)
		set_vi_handler(MIPS_CPU_IRQ_COMPARE, mips_timer_dispatch);
}
