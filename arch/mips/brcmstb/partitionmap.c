/*
 *  Copyright (C) 2012 Google Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#ifdef CONFIG_BRUNO
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/map.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <mtd/partitionmap.h>

#define CFE_NAME         "cfe"
#define HNVRAM_NAME      "hnvram"
#define RESERVED0_NAME   "reserved0"
#define RESERVED1_NAME   "reserved1"
#define RESERVED2_NAME   "reserved2"
#define RESERVED3_NAME   "reserved3"
#define RESERVED4_NAME   "reserved4"
#define DRMREGION0_NAME  "drmregion0"
#define DRMREGION1_NAME  "drmregion1"
#define NVRAM_NAME       "nvram"

#define KERNEL0_NAME     "kernel0"
#define KERNEL1_NAME     "kernel1"
#define ROOTFS0_NAME     "rootfs0"
#define ROOTFS1_NAME     "rootfs1"
#define DATA_NAME        "data+ubi"
#define MISC_NAME        "misc+ubi"
#define EMERGENCY_NAME   "emergency"

#define CFE_SIZE         0x00200000UL
#define HNVRAM_SIZE      0x00170000UL
#define RESERVED0_SIZE   0x00010000UL
#define RESERVED1_SIZE   0x00010000UL
#define RESERVED2_SIZE   0x00010000UL
#define RESERVED3_SIZE   0x00010000UL
#define RESERVED4_SIZE   0x00010000UL
#define DRMREGION0_SIZE  0x00010000UL
#define DRMREGION1_SIZE  0x00010000UL
#define NVRAM_SIZE       0x00020000UL

#define KERNEL0_SIZE_V1     0x10000000UL
#define KERNEL1_SIZE_V1     0x10000000UL
#define ROOTFS0_SIZE_V1     0x40000000UL
#define ROOTFS1_SIZE_V1     0x40000000UL
#define DATA_SIZE_V1        0x40000000UL
#define MISC_SIZE_V1        0x20000000UL

#define KERNEL0_SIZE_V2     0x02000000UL
#define KERNEL1_SIZE_V2     0x02000000UL
#define ROOTFS0_SIZE_V2     0x12000000UL
#define ROOTFS1_SIZE_V2     0x12000000UL
#define EMERGENCY_SIZE_V2   0x02000000UL
#define DATA_SIZE_V2        0x15F00000UL

#define CFE_OFFSET          0x00000000UL
#define HNVRAM_OFFSET       (CFE_OFFSET+CFE_SIZE)
#define RESERVED0_OFFSET    (HNVRAM_OFFSET+HNVRAM_SIZE)
#define RESERVED1_OFFSET    (RESERVED0_OFFSET+RESERVED0_SIZE)
#define RESERVED2_OFFSET    (RESERVED1_OFFSET+RESERVED1_SIZE)
#define RESERVED3_OFFSET    (RESERVED2_OFFSET+RESERVED2_SIZE)
#define RESERVED4_OFFSET    (RESERVED3_OFFSET+RESERVED3_SIZE)
#define DRMREGION0_OFFSET   (RESERVED4_OFFSET+RESERVED4_SIZE)
#define DRMREGION1_OFFSET   (DRMREGION0_OFFSET+DRMREGION0_SIZE)
#define NVRAM_OFFSET        (DRMREGION1_OFFSET+DRMREGION1_SIZE)

#define KERNEL0_OFFSET_V1   0x00000000UL
#define KERNEL1_OFFSET_V1   (KERNEL0_OFFSET_V1+KERNEL0_SIZE_V1)
#define ROOTFS0_OFFSET_V1   (KERNEL1_OFFSET_V1+KERNEL1_SIZE_V1)
#define ROOTFS1_OFFSET_V1   (ROOTFS0_OFFSET_V1+ROOTFS0_SIZE_V1)
#define DATA_OFFSET_V1      (ROOTFS1_OFFSET_V1+ROOTFS1_SIZE_V1)
#define MISC_OFFSET_V1      (DATA_OFFSET_V1+DATA_SIZE_V1)

#define KERNEL0_OFFSET_V2   0x00000000UL
#define KERNEL1_OFFSET_V2   (KERNEL0_OFFSET_V2+KERNEL0_SIZE_V2)
#define ROOTFS0_OFFSET_V2   (KERNEL1_OFFSET_V2+KERNEL1_SIZE_V2)
#define ROOTFS1_OFFSET_V2   (ROOTFS0_OFFSET_V2+ROOTFS0_SIZE_V2)
#define EMERGENCY_OFFSET_V2 (ROOTFS1_OFFSET_V2+ROOTFS1_SIZE_V2)
#define DATA_OFFSET_V2      (EMERGENCY_OFFSET_V2+EMERGENCY_SIZE_V2)

/* Partition map V1 */
static struct mtd_partition fixed_nor_partition_map_v1[] =
{
	{name: CFE_NAME, size: CFE_SIZE, offset: CFE_OFFSET},
	{name: HNVRAM_NAME, size: HNVRAM_SIZE, offset: HNVRAM_OFFSET},
	{name: RESERVED0_NAME, size: RESERVED0_SIZE, offset: RESERVED0_OFFSET},
	{name: RESERVED1_NAME, size: RESERVED1_SIZE, offset: RESERVED1_OFFSET},
	{name: RESERVED2_NAME, size: RESERVED2_SIZE, offset: RESERVED2_OFFSET},
	{name: RESERVED3_NAME, size: RESERVED3_SIZE, offset: RESERVED3_OFFSET},
	{name: RESERVED4_NAME, size: RESERVED4_SIZE, offset: RESERVED4_OFFSET},
	{name: DRMREGION0_NAME, size: DRMREGION0_SIZE, offset: DRMREGION0_OFFSET},
	{name: DRMREGION1_NAME, size: DRMREGION1_SIZE, offset: DRMREGION1_OFFSET},
	{name: NVRAM_NAME, size: NVRAM_SIZE, offset: NVRAM_OFFSET }
};

static struct mtd_partition fixed_nand_partition_map_v1[] =
{
	{name: KERNEL0_NAME, size: KERNEL0_SIZE_V1, offset: KERNEL0_OFFSET_V1},
	{name: KERNEL1_NAME, size: KERNEL1_SIZE_V1, offset: KERNEL1_OFFSET_V1},
	{name: ROOTFS0_NAME, size: ROOTFS0_SIZE_V1, offset: ROOTFS0_OFFSET_V1},
	{name: ROOTFS1_NAME, size: ROOTFS1_SIZE_V1, offset: ROOTFS1_OFFSET_V1},
	{name: DATA_NAME, size: DATA_SIZE_V1, offset: DATA_OFFSET_V1},
	{name: MISC_NAME, size: MISC_SIZE_V1, offset: MISC_OFFSET_V1}
};

/* Partition map V2 */
#define fixed_nor_partition_map_v2 fixed_nor_partition_map_v1
static struct mtd_partition fixed_nand_partition_map_v2[] =
{
	{name: KERNEL0_NAME, size: KERNEL0_SIZE_V2, offset: KERNEL0_OFFSET_V2},
	{name: KERNEL1_NAME, size: KERNEL1_SIZE_V2, offset: KERNEL1_OFFSET_V2},
	{name: ROOTFS0_NAME, size: ROOTFS0_SIZE_V2, offset: ROOTFS0_OFFSET_V2},
	{name: ROOTFS1_NAME, size: ROOTFS1_SIZE_V2, offset: ROOTFS1_OFFSET_V2},
	{name: EMERGENCY_NAME, size: DATA_SIZE_V2, offset: EMERGENCY_OFFSET_V2},
	{name: DATA_NAME, size: DATA_SIZE_V2, offset: DATA_OFFSET_V2}
};

/* By default, use partition map v2. */
struct mtd_partition *fixed_nor_partition_map = fixed_nor_partition_map_v1;
int fixed_nor_partition_map_size = ARRAY_SIZE(fixed_nor_partition_map_v1);
EXPORT_SYMBOL(fixed_nor_partition_map_size);

struct mtd_partition *fixed_nand_partition_map = fixed_nand_partition_map_v2;
int fixed_nand_partition_map_size = ARRAY_SIZE(fixed_nand_partition_map_v2);
EXPORT_SYMBOL(fixed_nand_partition_map_size);

static DEFINE_MUTEX(partitionmap_mutex);
static struct list_head mtd_dev_list = LIST_HEAD_INIT(mtd_dev_list);

static DEFINE_MUTEX(bb_mutex);
static struct list_head bb_list = LIST_HEAD_INIT(bb_list);

int partitionmap_version = 2;	/* partition map version */
EXPORT_SYMBOL(partitionmap_version);

struct mtd_dev_entry {
	struct list_head list;
	struct platform_device *pdev;
};

struct bb_entry {
	struct list_head list;
	loff_t offset;
};

size_t partitionmap_print_bbinfo(char *buffer, size_t size)
{
	size_t ret;
	size_t pos = 0;
	int i;
	struct mtd_partition *mtd;
	struct bb_entry *bb;
	size_t *bb_map = kzalloc(fixed_nand_partition_map_size*sizeof(size_t),
				 GFP_KERNEL);
	if (!bb_map)
		return 0;

	ret = scnprintf(buffer + pos, size - pos, "partition: badblocks\n");
	if (!ret) {
		kfree(bb_map);
		return 0;
	}

	pos += ret;

	mutex_lock(&bb_mutex);
	list_for_each_entry(bb, &bb_list, list) {
		for (i= 0, mtd = &fixed_nand_partition_map[0];
		     i < fixed_nand_partition_map_size; ++i, ++mtd) {
			if ((bb->offset >= mtd->offset) &&
			    (bb->offset < (mtd->offset + mtd->size))) {
				++bb_map[i];
			}
		}
	}
	mutex_unlock(&bb_mutex);

	for (i = 0, mtd = &fixed_nand_partition_map[0];
	     i < fixed_nand_partition_map_size; ++i, ++mtd) {
		if (pos < size) {
			ret = (size_t) scnprintf(
					buffer + pos, size - pos,
					"%s: %u\n", mtd->name,
					bb_map[i]);
		} else {
			ret = 0;
		}
		if (!ret) {
			kfree(bb_map);
			return ret;
		}
		pos += ret;
	}

	if (pos && buffer[pos - 1] == '\n') {
		buffer[pos - 1] = '\0';
	}

	kfree(bb_map);

	return pos;
}
EXPORT_SYMBOL(partitionmap_print_bbinfo);

size_t partitionmap_print_info(char *buffer, size_t size)
{
	size_t ret;
	size_t pos = 0;
	struct mtd_dev_entry *mtd;

	ret = scnprintf(buffer + pos, size - pos,
			"Partition map version: %d\n",
			partitionmap_version);
	if (!ret)
		return 0;

	pos += ret;
	mutex_lock(&partitionmap_mutex);
	list_for_each_entry(mtd, &mtd_dev_list, list) {
		if (pos < size) {
			ret = (size_t)scnprintf(buffer + pos, size - pos,
						"%s\n",
						dev_name(&mtd->pdev->dev));
		} else {
			ret = 0;
		}
		if (!ret) {
			mutex_unlock(&partitionmap_mutex);
			return ret;
		}
		pos += ret;
	}
	mutex_unlock(&partitionmap_mutex);

	if (pos && buffer[pos - 1] == '\n') {
		buffer[pos - 1] = '\0';
	}

	return pos;
}
EXPORT_SYMBOL(partitionmap_print_info);

int switch_partition(int pver) {
	if (partitionmap_version == pver) {
		return 1;
	}

	switch (pver) {
		case 1:
			fixed_nor_partition_map = fixed_nor_partition_map_v1;
			fixed_nor_partition_map_size =
					ARRAY_SIZE(fixed_nor_partition_map_v1);
			fixed_nand_partition_map = fixed_nand_partition_map_v1;
			fixed_nand_partition_map_size =
					ARRAY_SIZE(fixed_nand_partition_map_v1);
			break;
		case 2:
			fixed_nor_partition_map = fixed_nor_partition_map_v2;
			fixed_nor_partition_map_size =
					ARRAY_SIZE(fixed_nor_partition_map_v2);
			fixed_nand_partition_map = fixed_nand_partition_map_v2;
			fixed_nand_partition_map_size =
					ARRAY_SIZE(fixed_nand_partition_map_v2);
			break;
		default:
			/* Keep the default setting */
			pr_info("Invalid partition version %d, ignore.\n", pver);
			return 2;
	}
	pr_info("Switched partition from version %d to version %d.\n",
		partitionmap_version, pver);
	partitionmap_version = pver;
	return 0;
}
EXPORT_SYMBOL(switch_partition);

void register_badblock(loff_t offset)
{
	struct bb_entry* obj = (struct bb_entry *)
			kmalloc(sizeof(struct bb_entry), GFP_KERNEL);

	if (!obj)
		panic("Insufficient memory to allocate MTD device entry\n");

	obj->offset = offset;

	mutex_lock(&bb_mutex);
	list_add(&obj->list, &bb_list);
	mutex_unlock(&bb_mutex);
}
EXPORT_SYMBOL(register_badblock);

void register_nand(struct platform_device *pdev)
{
	struct mtd_dev_entry* obj = (struct mtd_dev_entry *)
			kmalloc(sizeof(struct mtd_dev_entry), GFP_KERNEL);

	if (!obj)
		panic("Insufficient memory to allocate MTD device entry\n");

	obj->pdev = pdev;

	mutex_lock(&partitionmap_mutex);
	list_add(&obj->list, &mtd_dev_list);
	mutex_unlock(&partitionmap_mutex);
}
EXPORT_SYMBOL(register_nand);

void flush_nand(void)
{
	struct mtd_dev_entry *mtd;
	struct list_head *pos, *q;

	mutex_lock(&partitionmap_mutex);
	list_for_each_safe(pos, q, &mtd_dev_list){
		mtd= list_entry(pos, struct mtd_dev_entry, list);
		pr_info("Remove mtd device '%s'.\n", dev_name(&mtd->pdev->dev));
		platform_device_unregister(mtd->pdev);
		list_del(pos);
		kfree(mtd);
	}
	mutex_unlock(&partitionmap_mutex);
}
EXPORT_SYMBOL(flush_nand);

static int __init partitionver_setup(char *options)
{
	int pver;
	char* endp;
	if (*options == 0)
		return 0;
	pver = simple_strtol(options, &endp, 10);
	switch_partition(pver);
	return 0;
}
__setup("partitionver=", partitionver_setup);

#endif  /* CONFIG_BRUNO */
