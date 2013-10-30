/*
 *  linux/arch/arm/mach-vexpress/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of_fdt.h>
#include <linux/vexpress.h>

#include <asm/smp_scu.h>
#include <asm/mach/map.h>

#include <plat/platsmp.h>

#include "core.h"

#if defined(CONFIG_OF)

static int __init lionhead_dt_cpus_num(unsigned long node, const char *uname,
		int depth, void *data)
{
	static int prev_depth = -1;
	static int nr_cpus = -1;

	if (prev_depth > depth && nr_cpus > 0)
		return nr_cpus;

	if (nr_cpus < 0 && strcmp(uname, "cpus") == 0)
		nr_cpus = 0;

	if (nr_cpus >= 0) {
		const char *device_type = of_get_flat_dt_prop(node,
				"device_type", NULL);

		if (device_type && strcmp(device_type, "cpu") == 0)
			nr_cpus++;
	}

	prev_depth = depth;

	return 0;
}

static void __init lionhead_dt_smp_init_cpus(void)
{
	int ncores = 0, i;

	ncores = of_scan_flat_dt(lionhead_dt_cpus_num, NULL);

	if (ncores < 2)
		return;

	if (ncores > nr_cpu_ids) {
		pr_warn("SMP: %u cores greater than maximum (%u), clipping\n",
				ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; ++i)
		set_cpu_possible(i, true);
}

static void __init lionhead_dt_smp_prepare_cpus(unsigned int max_cpus)
{
	int i;

	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);
}

#else

static void __init lionhead_dt_smp_init_cpus(void)
{
	WARN_ON(1);
}

void __init lionhead_dt_smp_prepare_cpus(unsigned int max_cpus)
{
	WARN_ON(1);
}

#endif

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
static void __init lionhead_smp_init_cpus(void)
{
	lionhead_dt_smp_init_cpus();
}

static void __init lionhead_smp_prepare_cpus(unsigned int max_cpus)
{
	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	lionhead_dt_smp_prepare_cpus(max_cpus);

	/*
	 * Write the address of secondary startup into the
	 * system-wide flags register. The boot monitor waits
	 * until it receives a soft interrupt, and then the
	 * secondary CPU branches to this address.
	 */
	vexpress_flags_set(virt_to_phys(versatile_secondary_startup));
}

struct smp_operations __initdata lionhead_smp_ops = {
	.smp_init_cpus		= lionhead_smp_init_cpus,
	.smp_prepare_cpus	= lionhead_smp_prepare_cpus,
	.smp_secondary_init	= versatile_secondary_init,
	.smp_boot_secondary	= versatile_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= lionhead_cpu_die,
#endif
};
