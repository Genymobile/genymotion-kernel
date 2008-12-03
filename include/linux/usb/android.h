/*
 * Platform data for Android USB
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
#ifndef	__LINUX_USB_ANDROID_H
#define	__LINUX_USB_ANDROID_H

struct android_usb_platform_data {
	/* USB device descriptor fields */
	__u16 vendor_id;

	/* Default product ID. */
	__u16 product_id;

	/* Product ID when adb is enabled. */
	__u16 adb_product_id;

	__u16 version;

	char *product_name;
	char *manufacturer_name;
	char *serial_number;

	/* number of LUNS for mass storage function */
	int nluns;
};

#endif	/* __LINUX_USB_ANDROID_H */
