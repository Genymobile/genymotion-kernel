/* arch/arm/mach-goldfish/timer.c
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

#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <mach/timer.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach/time.h>

#include <linux/platform_device.h>

static DEFINE_SPINLOCK(goldfish_timer_lock);
static int goldfish_timer_ready;

static irqreturn_t goldfish_timer_interrupt(int irq, void *dev_id)
{
	uint32_t timer_base = IO_ADDRESS(GOLDFISH_TIMER_BASE);
	struct clock_event_device *evt = dev_id;

	writel(1, timer_base + TIMER_CLEAR_INTERRUPT);
	if (evt->event_handler)
		evt->event_handler(evt);
	return IRQ_HANDLED;
}

static cycle_t goldfish_timer_read(struct clocksource *cs)
{
	uint32_t timer_base = IO_ADDRESS(GOLDFISH_TIMER_BASE);
	unsigned long irqflags;
	cycle_t rv;

	spin_lock_irqsave(&goldfish_timer_lock, irqflags);
	rv = readl(timer_base + TIMER_TIME_LOW);
	rv |= (int64_t)readl(timer_base + TIMER_TIME_HIGH) << 32;
	spin_unlock_irqrestore(&goldfish_timer_lock, irqflags);
	return rv;
}

static int goldfish_timer_set_next_event(unsigned long cycles,
                                         struct clock_event_device *evt)
{
	uint32_t timer_base = IO_ADDRESS(GOLDFISH_TIMER_BASE);
	unsigned long irqflags;
	uint64_t alarm;

	spin_lock_irqsave(&goldfish_timer_lock, irqflags);
	alarm = readl(timer_base + TIMER_TIME_LOW);
	alarm |= (int64_t)readl(timer_base + TIMER_TIME_HIGH) << 32;
	alarm += cycles;
	writel(alarm >> 32, timer_base + TIMER_ALARM_HIGH);
	writel(alarm, timer_base + TIMER_ALARM_LOW);
	spin_unlock_irqrestore(&goldfish_timer_lock, irqflags);
	return 0;
}

static void goldfish_timer_set_mode(enum clock_event_mode mode,
                                    struct clock_event_device *evt)
{
	uint32_t timer_base = IO_ADDRESS(GOLDFISH_TIMER_BASE);
	switch (mode) {
		case CLOCK_EVT_MODE_RESUME:
		case CLOCK_EVT_MODE_PERIODIC:
			break;
		case CLOCK_EVT_MODE_ONESHOT:
			break;
		case CLOCK_EVT_MODE_UNUSED:
		case CLOCK_EVT_MODE_SHUTDOWN:
			writel(1, timer_base + TIMER_CLEAR_ALARM);
			break;
	}
}

/*
 * TODO: the mechanism for overriding the default sched_clock() has
 * changed since 2.6, but I'm not sure how the new mechanism works, so
 * I'm just reverting to the default to avoid a link-time multiple
 * definition error.
unsigned long long sched_clock(void)
{
	if(goldfish_timer_ready)
		return ktime_to_ns(ktime_get());
	else
		return 0;
}
 */

static struct clock_event_device goldfish_clockevent = {
	.name           = "goldfish_timer",
	.features       = CLOCK_EVT_FEAT_ONESHOT,
	.max_delta_ns   = ULONG_MAX,
	.min_delta_ns   = 1,
	.mult           = 1,
	.shift          = 0,
	.rating         = 200,
	.set_next_event = goldfish_timer_set_next_event,
	.set_mode       = goldfish_timer_set_mode,
};

static struct clocksource goldfish_clocksource = {
	.name           = "goldfish_timer",
	.rating         = 200,
	.read           = goldfish_timer_read,
	.mult           = 1,
	.mask           = CLOCKSOURCE_MASK(64),
	.shift          = 0,
	.flags          = CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct irqaction goldfish_timer_irq = {
	.name		= "Goldfish Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER,
	.handler	= goldfish_timer_interrupt,
	.dev_id		= &goldfish_clockevent,
};

static void __init goldfish_timer_init(void)
{
	int res;

	res = clocksource_register(&goldfish_clocksource);
	if (res)
		printk(KERN_ERR "goldfish_timer_init: "
		       "clocksource_register failed\n");

	res = setup_irq(IRQ_TIMER, &goldfish_timer_irq);
	if (res)
		printk(KERN_ERR "goldfish_timer_init: setup_irq failed\n");

	goldfish_clockevent.cpumask = cpumask_of(0);
	clockevents_register_device(&goldfish_clockevent);

	goldfish_timer_ready = 1;
}

struct sys_timer goldfish_timer = {
	.init		= goldfish_timer_init,
};

