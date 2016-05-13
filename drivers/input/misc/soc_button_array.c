/*
 * Supports for the button array on SoC tablets originally running
 * Windows 8.
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio_keys.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/input/soc_button_array.h>

/*
 * Some of the buttons like volume up/down are auto repeat, while others
 * are not. To support both, we register two platform devices, and put
 * buttons into them based on whether the key should be auto repeat.
 */
#define BUTTON_TYPES	2

struct soc_button_data {
	struct platform_device *children[BUTTON_TYPES];
};

/*
 * Get the Nth GPIO number from the ACPI object.
 */
static int soc_button_lookup_gpio(struct device *dev, const char *con_id,
				  int acpi_index)
{
	struct gpio_desc *desc;
	int gpio;

	desc = gpiod_get_index(dev, con_id, acpi_index, GPIOD_ASIS);
	if (IS_ERR(desc))
		return PTR_ERR(desc);

	gpio = desc_to_gpio(desc);

	gpiod_put(desc);

	return gpio;
}

static struct platform_device *
soc_button_device_create(struct device *dev,
			 const char *gpiod_con_id,
			 const struct soc_button_info *button_info,
			 bool autorepeat)
{
	const struct soc_button_info *info;
	struct platform_device *pd;
	struct gpio_keys_button *gpio_keys;
	struct gpio_keys_platform_data *gpio_keys_pdata;
	int n_buttons = 0;
	int gpio;
	int i, error;

	gpio_keys_pdata = devm_kzalloc(dev,
				       sizeof(*gpio_keys_pdata) +
					sizeof(*gpio_keys) * MAX_NBUTTONS,
				       GFP_KERNEL);
	if (!gpio_keys_pdata)
		return ERR_PTR(-ENOMEM);

	gpio_keys = (void *)(gpio_keys_pdata + 1);

	for (info = button_info; info->name; info++) {
		if (info->autorepeat != autorepeat)
			continue;

		gpio = soc_button_lookup_gpio(dev,
					      gpiod_con_id,
					      info->acpi_index);
		if (!gpio_is_valid(gpio))
			continue;

		for (i = 0; i < n_buttons; i++) {
			if (gpio_keys[i].gpio == gpio)
				break;
		}
		if (i < n_buttons)
			continue; /* the GPIO has already been assigned */

		gpio_keys[n_buttons].type = info->event_type;
		gpio_keys[n_buttons].code = info->event_code;
		gpio_keys[n_buttons].gpio = gpio;
		gpio_keys[n_buttons].active_low = info->active_low;
		gpio_keys[n_buttons].desc = info->name;
		gpio_keys[n_buttons].wakeup = info->wakeup;
		n_buttons++;
	}

	if (n_buttons == 0) {
		error = -ENODEV;
		goto err_free_mem;
	}

	gpio_keys_pdata->buttons = gpio_keys;
	gpio_keys_pdata->nbuttons = n_buttons;
	gpio_keys_pdata->rep = autorepeat;

	pd = platform_device_alloc("gpio-keys", PLATFORM_DEVID_AUTO);
	if (!pd) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	error = platform_device_add_data(pd, gpio_keys_pdata,
					 sizeof(*gpio_keys_pdata));
	if (error)
		goto err_free_pdev;

	error = platform_device_add(pd);
	if (error)
		goto err_free_pdev;

	return pd;

err_free_pdev:
	platform_device_put(pd);
err_free_mem:
	devm_kfree(dev, gpio_keys_pdata);
	return ERR_PTR(error);
}

int soc_dev_button_remove(struct soc_button_data *priv)
{
	int i;

	for (i = 0; i < BUTTON_TYPES; i++)
		if (priv->children[i])
			platform_device_unregister(priv->children[i]);

	return 0;
}
EXPORT_SYMBOL_GPL(soc_dev_button_remove);

int soc_dev_button_enumerate(struct device *dev, struct soc_button_data *priv,
			     const char *gpiod_con_id,
			     struct soc_button_info *button_info)
{
	struct platform_device *pd;
	int i;
	int error;

	if (gpiod_count(dev, gpiod_con_id) <= 0) {
		dev_info(dev, "no GPIO attached, ignoring...\n");
		return -ENODEV;
	}

	for (i = 0; i < BUTTON_TYPES; i++) {
		pd = soc_button_device_create(dev, gpiod_con_id, button_info,
					      i == 0);
		if (IS_ERR(pd)) {
			error = PTR_ERR(pd);
			if (error != -ENODEV) {
				soc_dev_button_remove(priv);
				return error;
			}
			continue;
		}

		priv->children[i] = pd;
	}

	if (!priv->children[0] && !priv->children[1])
		return -ENODEV;

	return 0;
}
EXPORT_SYMBOL_GPL(soc_dev_button_enumerate);

struct soc_button_data *soc_dev_button_data_allocate(struct device *dev)
{
	return devm_kzalloc(dev, sizeof(struct soc_button_data), GFP_KERNEL);
}
EXPORT_SYMBOL_GPL(soc_dev_button_data_allocate);

static int soc_button_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct acpi_device_id *id;
	struct soc_button_info *button_info;
	struct soc_button_data *priv;
	int error;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id)
		return -ENODEV;

	button_info = (struct soc_button_info *)id->driver_data;

	priv = soc_dev_button_data_allocate(dev);
	if (!priv)
		return -ENOMEM;

	error = soc_dev_button_enumerate(dev, priv, KBUILD_MODNAME,
					 button_info);
	if (error)
		return error;

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int soc_button_remove(struct platform_device *pdev)
{
	return soc_dev_button_remove(platform_get_drvdata(pdev));
}

static struct soc_button_info soc_button_PNP0C40[] = {
	{ "power", 0, EV_KEY, KEY_POWER, false, true, true },
	{ "home", 1, EV_KEY, KEY_LEFTMETA, false, true, true },
	{ "volume_up", 2, EV_KEY, KEY_VOLUMEUP, true, false, true },
	{ "volume_down", 3, EV_KEY, KEY_VOLUMEDOWN, true, false, true },
	{ "rotation_lock", 4, EV_SW, SW_ROTATE_LOCK, false, false, true },
	{ }
};

static const struct acpi_device_id soc_button_acpi_match[] = {
	{ "PNP0C40", (unsigned long)soc_button_PNP0C40 },
	{ }
};

MODULE_DEVICE_TABLE(acpi, soc_button_acpi_match);

static struct platform_driver soc_button_driver = {
	.probe          = soc_button_probe,
	.remove		= soc_button_remove,
	.driver		= {
		.name = KBUILD_MODNAME,
		.acpi_match_table = ACPI_PTR(soc_button_acpi_match),
	},
};
module_platform_driver(soc_button_driver);

MODULE_LICENSE("GPL");
