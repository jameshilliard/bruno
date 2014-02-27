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

#define GFRM100_SEARCH_KEY_REPORT_ID   0xF7
#define GFRM100_SEARCH_KEY_DOWN        0x0
#define GFRM100_SEARCH_KEY_AUDIO_DATA  0x1
#define GFRM100_SEARCH_KEY_UP          0x2

static u8 search_key_dn[3] = {0x40, 0xF7, 0x00};
static u8 search_key_up[3] = {0x40, 0x00, 0x00};

static int gfrm_input_mapped(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	int hdev_type = (int)hid_get_drvdata(hdev);

	if (hdev_type == GFRM200) {
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

static int gfrm_raw_event(struct hid_device *hdev, struct hid_report *report,
		u8 *data, int size)
{
	int hdev_type = (int)hid_get_drvdata(hdev);
	int ret = 0;

	if (hdev_type != GFRM100)
		return 0;

	if (size < 2 || data[0] != GFRM100_SEARCH_KEY_REPORT_ID)
		return 0;

	/*
	 * Convert GFRM100 Search key reports into Consumer.00f7 (Key.Search)
	 * reports. Ignore audio data.
	 */
	switch (data[1]) {
	case GFRM100_SEARCH_KEY_DOWN:
		ret = hid_input_report(hdev, HID_INPUT_REPORT, search_key_dn,
					sizeof(search_key_dn), 1);
		break;

	case GFRM100_SEARCH_KEY_AUDIO_DATA:
		break;

	case GFRM100_SEARCH_KEY_UP:
		ret = hid_input_report(hdev, HID_INPUT_REPORT, search_key_up,
					sizeof(search_key_up), 1);
		break;

	default:
		break;
	}

	return (ret < 0) ? ret : 1;
}

static int gfrm_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;

	hid_set_drvdata(hdev, (void *)id->driver_data);

	ret = hid_parse(hdev);
	if (ret)
		goto done;

	if (id->driver_data == GFRM100) {
		/*
		 * GFRM100 HID Report Descriptor does not describe the Search
		 * key reports. Thus, we need to add it manually here, so that
		 * those reports reach gfrm_raw_event() from hid_input_report().
		 */
		if (!hid_register_report(hdev, HID_INPUT_REPORT,
					 GFRM100_SEARCH_KEY_REPORT_ID)) {
			ret = -ENOMEM;
			goto done;
		}
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
done:
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
	.raw_event = gfrm_raw_event,
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
