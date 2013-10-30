/*
 * Versatile Express V2M Motherboard Support
 */
#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/vexpress.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>

#include <asm/mach-types.h>
#include <asm/sizes.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/timer-sp.h>

#include <plat/sched_clock.h>
#include <plat/platsmp.h>

#include "core.h"

static struct map_desc lionhead_io_desc __initdata = {
	.virtual	= LIONHEAD_PERIPH,
	.pfn		= __phys_to_pfn(0x1c000000),
	.length		= SZ_2M,
	.type		= MT_DEVICE,
};

static void __init lionhead_map_io(void)
{
	iotable_init(&lionhead_io_desc, 1);
}

static void __init lionhead_init_early(void)
{
	vexpress_sysreg_of_early_init();
}

static void __init lionhead_timer_init(void)
{
	of_clk_init(NULL);

	clocksource_of_init();

	versatile_sched_clock_init(vexpress_get_24mhz_clock_base(),
				24000000);
}

static const struct of_device_id lionhead_dt_bus_match[] __initconst = {
	{ .compatible = "simple-bus", },
	{ .compatible = "arm,amba-bus", },
	{ .compatible = "arm,vexpress,config-bus", },
	{}
};

static void __init lionhead_init(void)
{
	l2x0_of_init(0x00400000, 0xfe0fffff);
	of_platform_populate(NULL, lionhead_dt_bus_match, NULL, NULL);
}

static const char * const lionhead_dt_match[] __initconst = {
	"generic,lionhead",
	NULL,
};

DT_MACHINE_START(VEXPRESS_DT, "lionhead virtual platform")
	.dt_compat	= lionhead_dt_match,
	.smp		= smp_ops(lionhead_smp_ops),
	.map_io		= lionhead_map_io,
	.init_early	= lionhead_init_early,
	.init_irq	= irqchip_init,
	.init_time	= lionhead_timer_init,
	.init_machine	= lionhead_init,
MACHINE_END
