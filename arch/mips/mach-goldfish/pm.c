/* arch/arm/mach-msm/pm.c
 *
 * Goldfish Power Management Routines
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/suspend.h>

extern void (*cpu_wait)(void);

static int goldfish_pm_enter(suspend_state_t state)
{
	if (cpu_wait)
		(*cpu_wait)();
	return 0;
}

static struct platform_suspend_ops goldfish_pm_ops = {
	.enter		= goldfish_pm_enter,
	.valid		= suspend_valid_only_mem,
};

static int __init goldfish_pm_init(void)
{
	suspend_set_ops(&goldfish_pm_ops);
	return 0;
}

device_initcall(goldfish_pm_init);

