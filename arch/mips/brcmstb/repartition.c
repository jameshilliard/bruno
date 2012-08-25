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
#ifdef CONFIG_REPARTITION
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include "partitionmap.h"
#include "repartition.h"

static int version;
static int disable, disable_min = 0, disable_max = 1;
static int nand_size_mb;

static struct ctl_table_header *repartition_sysctl_header;

static void reinit_nand(void)
{
	flush_nand();
	init_nand();
}

static int repartition_sysctl_version(ctl_table *ctl, int write,
				      void __user *buffer, size_t *lenp,
				      loff_t *ppos)
{
	int ret = 0;
	if (write) {
		if (disable) {
			pr_err("repartition is disabled\n");
			return -EINVAL;
		}
		ret = proc_dointvec(ctl, write, buffer, lenp, ppos);
		if (0 == switch_partition(version)) {
			reinit_nand();
		}
	} else {
		version = partitionmap_version;
		ret = proc_dointvec(ctl, write, buffer, lenp, ppos);
	}
	return ret;
}

static int repartition_sysctl_disable(ctl_table *ctl, int write,
				      void __user *buffer, size_t *lenp,
				      loff_t *ppos)
{
	int ret = 0;
	if (write && disable) {
		pr_err("repartition is disabled\n");
		return -EINVAL;
	}
	ret = proc_dointvec_minmax(ctl, write, buffer, lenp, ppos);
	return ret;
}

static ctl_table repartition_data_table[] = {
	{
		.procname       = "disable",
		.data           = &disable,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.extra1		= &disable_min,
		.extra2		= &disable_max,
		.proc_handler   = repartition_sysctl_disable,
	},
	{
		.procname       = "version",
		.data           = &version,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.proc_handler   = repartition_sysctl_version,
	},
	{
		.procname	= "nand_size_mb",
		.data		= &nand_size_mb,
		.maxlen		= sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
	{ }
};

static ctl_table repartition_dir_table[] = {
	{
		.procname     = "repartition",
		.mode         = 0555,
		.child        = repartition_data_table
	},
	{ }
};

/* Make sure that /proc/sys/dev is there */
static ctl_table repartition_root_table[] = {
	{
		.procname       = "dev",
		.maxlen         = 0,
		.mode           = 0555,
		.child          = repartition_dir_table,
	},
	{ }
};

static int mtd_parse(struct mtd_info *mtd,
	struct mtd_partition **parts, unsigned long origin)
{
	nand_size_mb = mtd->size / 1024 / 1024;

	/* TODO(apenwarr): use Redboot partition tables where available. */
	if (!partitionmap_version) {
		/* not set by partitionver */
		if (mtd->size == (uint64_t)4096*1024*1024) {
			/* default to v1 partition layout for 4GB flash */
			switch_partition(1);
		} else if (mtd->size == (uint64_t)1024*1024*1024) {
			/* default to v2 partition layout for 1GB flash */
			switch_partition(2);
		}
		/*
		 * Anything else doesn't get any default partition tables,
		 * because scribbling over the wrong areas of flash when
		 * you don't know what's going on is unwise.
		 */
	}
	*parts = fixed_nand_partition_map;
	return fixed_nand_partition_map_size;
}

static struct mtd_part_parser mtdp = {
	name: "brunopart",
	parse_fn: mtd_parse,
};

static int __init repartition_init(void)
{
	static int initialized;

	if (initialized == 1)
		return 0;

	repartition_sysctl_header =
			register_sysctl_table(repartition_root_table);
	register_mtd_parser(&mtdp);
	initialized = 1;
	return 0;
}

static int __init partitionver_setup(char *options)
{
	int pver;
        char *endp;
	if (*options == 0)
		return 0;
	pver = simple_strtol(options, &endp, 10);
	switch_partition(pver);
	return 0;
}
__setup("partitionver=", partitionver_setup);

#ifdef MODULE
static int __exit repartition_exit(void)
{
	if (repartition_sysctl_header)
		unregister_sysctl_table(repartition_sysctl_header);
	unregister_mtd_parser(&mtdp);
        return 0;
}
module_exit(repartition_exit);
#endif  /* MODULE */

module_init(repartition_init);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ke Dong <kedong@google.com>");
MODULE_DESCRIPTION("Partition map");

#endif  /* CONFIG_REPARTITION */
