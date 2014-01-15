/*
 * HID driver for Google Fiber TV remote controls
 *
 * Copyright (c) 2014 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include "hid-ids.h"

#define GFRM100  1  /* Google Fiber GFRM100 (Bluetooth classic) */
#define GFRM200  2  /* Texas Instruments CC2541ARC (Bluetooth LE) */

static int gfrm_input_mapped(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	int type = (int)hid_get_drvdata(hdev);

	if (type == GFRM200) {
		if (usage->code == KEY_LEFTMETA) {
			hid_map_usage(hi, usage, bit, max, usage->type, KEY_MENU);
		} else if (usage->code == KEY_BACKSPACE) {
			hid_map_usage(hi, usage, bit, max, usage->type, KEY_BACK);
		} else if (usage->code == KEY_MUTE) {
			hid_map_usage(hi, usage, bit, max, usage->type, KEY_PROGRAM);
		}
	}

	return 0;
}

static int gfrm_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;

	hid_set_drvdata(hdev, id->driver_data);

	ret = hid_parse(hdev);
	if (!ret) {
		ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	}

	return ret;
}

static void gfrm_remove(struct hid_device *hdev)
{
	hid_hw_stop(hdev);
	hid_set_drvdata(hdev, NULL);
}

static const struct hid_device_id gfrm_devices[] = {
	{ HID_BLUETOOTH_DEVICE(0x58, 0x2000),
		.driver_data = GFRM100 },
	{ HID_BLUETOOTH_DEVICE(0xD, 0x0),
		.driver_data = GFRM200 },
	{ }
};
MODULE_DEVICE_TABLE(hid, gfrm_devices);

static struct hid_driver gfrm_driver = {
	.name = "gfrm",
	.id_table = gfrm_devices,
	.probe = gfrm_probe,
	.remove = gfrm_remove,
	.input_mapped = gfrm_input_mapped,
};

static int __init gfrm_init(void)
{
	return hid_register_driver(&gfrm_driver);
}

static void __exit gfrm_exit(void)
{
	hid_unregister_driver(&gfrm_driver);
}

module_init(gfrm_init);
module_exit(gfrm_exit);
MODULE_LICENSE("GPL");
