/*
 * Supports for the button array on the Surface tablets.
 *
 * (C) Copyright 2016 Red Hat, Inc
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/input/soc_button_array.h>
#include <uapi/linux/input-event-codes.h>

struct soc_device_info {
	const char * const *obj_names;
	struct soc_button_info *buttons;
};

static int surface3_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	const char *bid, *obj_name;
	struct soc_device_info *device_info;
	struct soc_button_data *priv;
	int error;
	int i;

	if (!id->driver_data)
		return -EINVAL;

	device_info = (struct soc_device_info *)id->driver_data;

	if (device_info->obj_names) {
		bid = acpi_device_bid(ACPI_COMPANION(&client->dev));
		i = 0;
		do {
			obj_name = device_info->obj_names[i++];
			if (obj_name && !strcmp(bid, obj_name))
				break;
		} while (obj_name);
		/* no acpi_device_bid match, bail out */
		if (!obj_name)
			return -ENODEV;
	}

	priv = soc_dev_button_data_allocate(&client->dev);
	if (!priv)
		return -ENOMEM;

	error = soc_dev_button_enumerate(&client->dev, priv, NULL,
					 device_info->buttons);
	if (error)
		return error;

	i2c_set_clientdata(client, priv);

	return 0;
}

static int surface3_remove(struct i2c_client *client)
{
	return soc_dev_button_remove(i2c_get_clientdata(client));
}

static struct soc_button_info soc_button_surface3[] = {
	{ "power", 0, EV_KEY, KEY_POWER, false, true, true },
	{ "home", 1, EV_KEY, KEY_LEFTMETA, false, true, false },
	{ "volume_up", 2, EV_KEY, KEY_VOLUMEUP, true, false, true },
	{ "volume_down", 3, EV_KEY, KEY_VOLUMEDOWN, true, false, true },
	{ }
};

static const char * const soc_device_obj_names_surface3[] = {
	"TEV2",
	0,
};

static const struct soc_device_info soc_device_surface3 = {
	.obj_names = soc_device_obj_names_surface3,
	.buttons = soc_button_surface3,
};

static const struct i2c_device_id surface3_id[] = {
	{ "MSHW0028:00", (unsigned long)&soc_device_surface3 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, surface3_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id surface3_acpi_match[] = {
	{ "MSHW0028", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, surface3_acpi_match);
#endif

static struct i2c_driver surface3_driver = {
	.probe = surface3_probe,
	.remove = surface3_remove,
	.id_table = surface3_id,
	.driver = {
		.name = "surface3",
		.acpi_match_table = ACPI_PTR(surface3_acpi_match),
	},
};
module_i2c_driver(surface3_driver);

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@gmail.com>");
MODULE_DESCRIPTION("surface3 button array driver");
MODULE_LICENSE("GPL v2");
