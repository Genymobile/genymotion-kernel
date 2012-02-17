/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2007 by Ralf Baechle
 */
#include <linux/clocksource.h>
#include <linux/cnt32_to_63.h>
#include <linux/timer.h>
#include <linux/init.h>

#include <asm/time.h>

/*
 * MIPS' sched_clock implementation.
 *
 * because cnt32_to_63() needs to be called at least once per half period to
 * work properly, and some of the MIPS' frequency is very low, perhaps a kernel
 * timer is needed to be set up to ensure this requirement is always met.
 * please refer to  arch/arm/plat-orion/time.c and include/linux/cnt32_to_63.h
 */
static unsigned long __maybe_unused tclk2ns_scale;
static unsigned long __maybe_unused tclk2ns_scale_factor;

#ifdef CONFIG_HR_SCHED_CLOCK
unsigned long long notrace sched_clock(void)
{
   unsigned long long v = cnt32_to_63(read_c0_count());
   return (v * tclk2ns_scale) >> tclk2ns_scale_factor;
}
#endif

static void __init __maybe_unused setup_sched_clock(struct clocksource *cs)
{
   unsigned long long v;

   v = cs->mult;
   /*
    * We want an even value to automatically clear the top bit
    * returned by cnt32_to_63() without an additional run time
    * instruction. So if the LSB is 1 then round it up.
    */
   if (v & 1)
       v++;
   tclk2ns_scale = v;
   tclk2ns_scale_factor = cs->shift;
}

static struct timer_list __maybe_unused cnt32_to_63_keepwarm_timer;

static void __maybe_unused cnt32_to_63_keepwarm(unsigned long data)
{
   mod_timer(&cnt32_to_63_keepwarm_timer, round_jiffies(jiffies + data));
   (void) sched_clock();
}

static void __maybe_unused setup_sched_clock_update(unsigned long tclk)
{
   unsigned long data;

   data = (0xffffffffUL / tclk / 2 - 2) * HZ;
   setup_timer(&cnt32_to_63_keepwarm_timer, cnt32_to_63_keepwarm, data);
   mod_timer(&cnt32_to_63_keepwarm_timer, round_jiffies(jiffies + data));
}

static cycle_t c0_hpt_read(void)
{
	return read_c0_count();
}

static struct clocksource clocksource_mips = {
	.name		= "MIPS",
	.read		= c0_hpt_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

int __init init_r4k_clocksource(void)
{
	if (!cpu_has_counter || !mips_hpt_frequency)
		return -ENXIO;

	/* Calculate a somewhat reasonable rating value */
	clocksource_mips.rating = 200 + mips_hpt_frequency / 10000000;

	clocksource_set_clock(&clocksource_mips, mips_hpt_frequency);

#ifdef CONFIG_HR_SCHED_CLOCK
	setup_sched_clock(&clocksource_mips);
#endif
	clocksource_register(&clocksource_mips);

#ifdef CONFIG_HR_SCHED_CLOCK_UPDATE
	setup_sched_clock_update(mips_hpt_frequency);
#endif
	return 0;
}
