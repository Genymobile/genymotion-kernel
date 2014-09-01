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

#include <linux/err.h>
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/stat.h>

#include <mach/timer.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/mach/time.h>

#include <linux/platform_device.h>

enum {
	SW_NAME_LEN     = 0x00,
	SW_NAME_PTR     = 0x04,
	SW_FLAGS        = 0x08,
	SW_STATE        = 0x0c,
	SW_INT_STATUS   = 0x10,
	SW_INT_ENABLE   = 0x14,

	SW_FLAGS_OUTPUT = 1U << 0
};

static struct class *goldfish_switch_class;

struct goldfish_switch {
	uint32_t base;
	int irq;
	uint32_t state;
	uint32_t flags;
	struct device *cdev;
	struct work_struct work;
	char name[0];
};

static irqreturn_t
goldfish_switch_interrupt(int irq, void *dev_id)
{
	struct goldfish_switch  *qs = dev_id;
	uint32_t status;

	status = readl(qs->base + SW_INT_STATUS);
	if(status) {
		qs->state = readl(qs->base + SW_STATE);
		schedule_work(&qs->work);
	}

	return status ? IRQ_HANDLED : IRQ_NONE;
}

static ssize_t goldfish_switch_state_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct goldfish_switch  *qs = dev_get_drvdata(dev);
	uint32_t state;

	if (!(qs->flags & SW_FLAGS_OUTPUT))
		return -EPERM;

	if (sscanf(buf, "%d", &state) != 1)
		return -EINVAL;

	writel(state, qs->base + SW_STATE);
	qs->state = readl(qs->base + SW_STATE);
	if(state != qs->state)
		return -EINVAL;

	return count;
}

static ssize_t goldfish_switch_state_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct goldfish_switch  *qs = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", qs->state);
}

static ssize_t goldfish_switch_direction_show(struct device *dev, 
					      struct device_attribute *attr,
					      char *buf)
{
	struct goldfish_switch  *qs = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", (qs->flags & SW_FLAGS_OUTPUT)  ? "output" : "input");
}


static DEVICE_ATTR(state, S_IRUGO | S_IWUSR, goldfish_switch_state_show, goldfish_switch_state_store);
static DEVICE_ATTR(direction, S_IRUGO, goldfish_switch_direction_show, NULL);

void goldfish_switch_work(struct work_struct *work)
{
#if 0 /* TODO: use some other update notification */
	struct goldfish_switch *qs = container_of(work, struct goldfish_switch, work);
	int ret;
	ret = sysfs_update_file(&qs->cdev->kobj, &class_device_attr_state.attr);
#endif
}

static int __devinit goldfish_switch_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *r;
	struct goldfish_switch *qs;
	uint32_t base;
	uint32_t name_len;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(r == NULL) {
		ret = -ENODEV;
		goto err_no_io_base;
	}
	base = IO_ADDRESS(r->start - IO_START);
	name_len = readl(base + SW_NAME_LEN);

	qs = kzalloc(sizeof(*qs) + name_len + 1, GFP_KERNEL);
	if(qs == NULL) {
		ret = -ENOMEM;
		goto err_qs_alloc_failed;
	}
	platform_set_drvdata(pdev, qs);
	qs->base = base;
	r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if(r == NULL) {
		ret = -ENODEV;
		goto err_no_irq;
	}
	qs->irq = r->start;

	writel(qs->name, base + SW_NAME_PTR);
	qs->name[name_len] = '\0';
	writel(0, base + SW_INT_ENABLE);

	qs->flags = readl(base + SW_FLAGS);
	qs->state = readl(base + SW_STATE);
	INIT_WORK(&qs->work, goldfish_switch_work);

	qs->cdev = device_create(goldfish_switch_class, &pdev->dev, 0,
						NULL, "%s", qs->name);
	if(unlikely(IS_ERR(qs->cdev))) {
		ret = PTR_ERR(qs->cdev);
		goto err_device_create_failed;
	}
	dev_set_drvdata(qs->cdev, qs);

	ret = device_create_file(qs->cdev, &dev_attr_state);
	if(ret)
		goto err_device_create_file_failed;

	ret = device_create_file(qs->cdev, &dev_attr_direction);
	if(ret)
		goto err_device_create_file_failed;
	
	ret = request_irq(qs->irq, goldfish_switch_interrupt, IRQF_SHARED, "goldfish_switch", qs);
	if(ret)
		goto err_request_irq_failed;
	writel(1, base + SW_INT_ENABLE);

	return 0;


//	free_irq(qs->irq, qs);
err_request_irq_failed:
err_device_create_file_failed:
	device_unregister(qs->cdev);
err_device_create_failed:
err_no_irq:
	kfree(qs);
err_qs_alloc_failed:
err_no_io_base:
	printk("goldfish_switch_probe failed %d\n", ret);
	return ret;
}

static int __devexit goldfish_switch_remove(struct platform_device *pdev)
{
	struct goldfish_switch *qs = platform_get_drvdata(pdev);
	writel(0, qs->base + SW_INT_ENABLE);
	free_irq(qs->irq, qs);
	device_unregister(qs->cdev);
	kfree(qs);
	return 0;
}

static struct platform_driver goldfish_switch_driver = {
	.probe = goldfish_switch_probe,
	.remove = goldfish_switch_remove,
	.driver = {
		.name = "goldfish-switch"
	}
};

static int __init goldfish_switch_init(void)
{
	goldfish_switch_class = class_create(THIS_MODULE, "switch");
	if (IS_ERR(goldfish_switch_class))
		return PTR_ERR(goldfish_switch_class);
	return platform_driver_register(&goldfish_switch_driver);
}

static void goldfish_switch_exit(void)
{
	platform_driver_unregister(&goldfish_switch_driver);
	class_destroy(goldfish_switch_class);
}

module_init(goldfish_switch_init);
module_exit(goldfish_switch_exit);

