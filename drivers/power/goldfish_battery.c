/* drivers/power/goldfish_battery.c
 *
 * Power supply driver for the goldfish emulator
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/io.h>
#ifdef CONFIG_ARM
#include <mach/hardware.h>
#endif


struct goldfish_battery_data {
	void __iomem *reg_base;
	int irq;
	spinlock_t lock;

	struct power_supply battery;
	struct power_supply ac;
};

#define GOLDFISH_BATTERY_READ(data, addr)   (readl(data->reg_base + addr))
#define GOLDFISH_BATTERY_WRITE(data, addr, x)   (writel(x, data->reg_base + addr))


/* temporary variable used between goldfish_battery_probe() and goldfish_battery_open() */
static struct goldfish_battery_data *battery_data;

enum {
	/* status register */
	BATTERY_INT_STATUS	    = 0x00,
	/* set this to enable IRQ */
	BATTERY_INT_ENABLE	    = 0x04,

	BATTERY_AC_ONLINE       = 0x08,
	BATTERY_STATUS          = 0x0C,
	BATTERY_HEALTH          = 0x10,
	BATTERY_PRESENT         = 0x14,
	BATTERY_CAPACITY        = 0x18,

	BATTERY_STATUS_CHANGED	= 1U << 0,
	AC_STATUS_CHANGED   	= 1U << 1,
	BATTERY_INT_MASK        = BATTERY_STATUS_CHANGED | AC_STATUS_CHANGED,
};


static int goldfish_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	struct goldfish_battery_data *data = container_of(psy,
		struct goldfish_battery_data, ac);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = GOLDFISH_BATTERY_READ(data, BATTERY_AC_ONLINE);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int goldfish_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct goldfish_battery_data *data = container_of(psy,
		struct goldfish_battery_data, battery);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = GOLDFISH_BATTERY_READ(data, BATTERY_STATUS);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = GOLDFISH_BATTERY_READ(data, BATTERY_HEALTH);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = GOLDFISH_BATTERY_READ(data, BATTERY_PRESENT);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = GOLDFISH_BATTERY_READ(data, BATTERY_CAPACITY);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property goldfish_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property goldfish_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static irqreturn_t goldfish_battery_interrupt(int irq, void *dev_id)
{
	unsigned long irq_flags;
	struct goldfish_battery_data *data = dev_id;
	uint32_t status;

	spin_lock_irqsave(&data->lock, irq_flags);

	/* read status flags, which will clear the interrupt */
	status = GOLDFISH_BATTERY_READ(data, BATTERY_INT_STATUS);
	status &= BATTERY_INT_MASK;

	if (status & BATTERY_STATUS_CHANGED)
		power_supply_changed(&data->battery);
	if (status & AC_STATUS_CHANGED)
		power_supply_changed(&data->ac);

	spin_unlock_irqrestore(&data->lock, irq_flags);
	return status ? IRQ_HANDLED : IRQ_NONE;
}


static int goldfish_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *r;
	struct goldfish_battery_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	spin_lock_init(&data->lock);

	data->battery.properties = goldfish_battery_props;
	data->battery.num_properties = ARRAY_SIZE(goldfish_battery_props);
	data->battery.get_property = goldfish_battery_get_property;
	data->battery.name = "battery";
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY;

	data->ac.properties = goldfish_ac_props;
	data->ac.num_properties = ARRAY_SIZE(goldfish_ac_props);
	data->ac.get_property = goldfish_ac_get_property;
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		printk(KERN_ERR "%s: platform_get_resource failed\n", pdev->name);
		ret = -ENODEV;
		goto err_no_io_base;
	}
#if defined(CONFIG_ARM)
	data->reg_base = (void __iomem *)IO_ADDRESS(r->start - IO_START);
#elif defined(CONFIG_X86) || defined(CONFIG_MIPS)
	data->reg_base = ioremap(r->start, r->end - r->start + 1);
#else
#error NOT SUPPORTED
#endif

	data->irq = platform_get_irq(pdev, 0);
	if (data->irq < 0) {
		printk(KERN_ERR "%s: platform_get_irq failed\n", pdev->name);
		ret = -ENODEV;
		goto err_no_irq;
	}

	ret = request_irq(data->irq, goldfish_battery_interrupt, IRQF_SHARED, pdev->name, data);
	if (ret)
		goto err_request_irq_failed;

	ret = power_supply_register(&pdev->dev, &data->ac);
	if (ret)
		goto err_ac_failed;

	ret = power_supply_register(&pdev->dev, &data->battery);
	if (ret)
		goto err_battery_failed;

	platform_set_drvdata(pdev, data);
	battery_data = data;

	GOLDFISH_BATTERY_WRITE(data, BATTERY_INT_ENABLE, BATTERY_INT_MASK);
	return 0;

err_battery_failed:
	power_supply_unregister(&data->ac);
err_ac_failed:
	free_irq(data->irq, data);
err_request_irq_failed:
err_no_irq:
#if defined(CONFIG_ARM)
#elif defined(CONFIG_X86) || defined(CONFIG_MIPS)
	iounmap(data->reg_base);
#else
#error NOT SUPPORTED
#endif
err_no_io_base:
	kfree(data);
err_data_alloc_failed:
	return ret;
}

static int goldfish_battery_remove(struct platform_device *pdev)
{
	struct goldfish_battery_data *data = platform_get_drvdata(pdev);

	power_supply_unregister(&data->battery);
	power_supply_unregister(&data->ac);

	free_irq(data->irq, data);
	kfree(data);
	battery_data = NULL;
	return 0;
}

static struct platform_driver goldfish_battery_device = {
	.probe		= goldfish_battery_probe,
	.remove		= goldfish_battery_remove,
	.driver = {
		.name = "goldfish-battery"
	}
};

static int __init goldfish_battery_init(void)
{
	return platform_driver_register(&goldfish_battery_device);
}

static void __exit goldfish_battery_exit(void)
{
	platform_driver_unregister(&goldfish_battery_device);
}

module_init(goldfish_battery_init);
module_exit(goldfish_battery_exit);

MODULE_AUTHOR("Mike Lockwood lockwood@android.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Battery driver for the Goldfish emulator");
