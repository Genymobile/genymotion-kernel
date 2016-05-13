/*
 * Supports for the button array on SoC tablets originally running
 * Windows 8.
 *
 * (C) Copyright 2014 Intel Corporation
 * (C) Copyright 2016 Red Hat, Inc
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

/*
 * Definition of buttons on the tablet. The ACPI index of each button
 * is defined in section 2.8.7.2 of "Windows ACPI Design Guide for SoC
 * Platforms"
 */
#define MAX_NBUTTONS	5

struct soc_button_info {
	const char *name;
	int acpi_index;
	unsigned int event_type;
	unsigned int event_code;
	bool autorepeat;
	bool wakeup;
	bool active_low;
};

struct soc_button_data;

struct soc_button_data *soc_dev_button_data_allocate(struct device *dev);
int soc_dev_button_remove(struct soc_button_data *priv);
int soc_dev_button_enumerate(struct device *dev, struct soc_button_data *priv,
			     const char *gpiod_con_id,
			     struct soc_button_info *button_info);
