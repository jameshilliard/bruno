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
#include "partitionmap.h"
#include "repartition.h"

#define PARTITIONMAP_STR_SIZE 512

static const char repartition_proc_name[] = "repartition";

static struct repartition_sysctl_setting {
	char info[PARTITIONMAP_STR_SIZE];   /* partition map info */
	int version;
	int disable;
	int disable_min;
	int disable_max;
} setting = {
	.disable = 0,
	.disable_min = 0,
	.disable_max = 1,
};

static struct ctl_table_header *repartition_sysctl_header;

static int repartition_print_info(void)
{
	int ret = partitionmap_print_info(setting.info, sizeof(setting.info));
	if (!ret)
		return 1;

	return 0;
}

static void reinit_nand(void)
{
	flush_nand();
	init_nand();
}

static int repartition_sysctl_info(ctl_table *ctl, int write,
				    void __user *buffer, size_t *lenp,
				    loff_t *ppos)
{
	if (!*lenp || (*ppos && !write)) {
		*lenp = 0;
		return 0;
	}

	if(repartition_print_info()) {
		*lenp = 0;
		pr_err("insufficient info buffer\n");
		return -ENOMEM;
	}
	proc_dostring(ctl, write, buffer, lenp, ppos);
        return 0;
}

static int repartition_sysctl_version(ctl_table *ctl, int write,
				      void __user *buffer, size_t *lenp,
				      loff_t *ppos)
{
	int ret = 0;
	if (write) {
		if (setting.disable) {
			pr_err("repartition is disabled\n");
			return -EINVAL;
		}
		ret = proc_dointvec(ctl, write, buffer, lenp, ppos);
		if (0 == switch_partition(setting.version)) {
			reinit_nand();
		}
	} else {
		setting.version = partitionmap_version;
		ret = proc_dointvec(ctl, write, buffer, lenp, ppos);
	}
	return ret;
}

static int repartition_sysctl_disable(ctl_table *ctl, int write,
				      void __user *buffer, size_t *lenp,
				      loff_t *ppos)
{
	int ret = 0;
	if (write && setting.disable) {
		pr_err("repartition is disabled\n");
		return -EINVAL;
	}
	ret = proc_dointvec_minmax(ctl, write, buffer, lenp, ppos);
	return ret;
}

static ctl_table repartition_data_table[] = {
	{
		.procname       = "info",
		.data           = setting.info,
		.maxlen         = PARTITIONMAP_STR_SIZE,
		.mode           = 0444,
		.proc_handler   = repartition_sysctl_info,
	},
	{
		.procname       = "disable",
		.data           = &setting.disable,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.extra1		= &setting.disable_min,
		.extra2		= &setting.disable_max,
		.proc_handler   = repartition_sysctl_disable,
	},
	{
		.procname       = "version",
		.data           = &setting.version,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.proc_handler   = repartition_sysctl_version,
	},
	{ }
};

static ctl_table repartition_dir_table[] = {
	{
		.procname     = repartition_proc_name,
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

static int __init repartition_init(void)
{
	static int initialized;

	if (initialized == 1)
		return 0;

	repartition_sysctl_header =
			register_sysctl_table(repartition_root_table);
	initialized = 1;
	return 0;
}

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

#ifdef MODULE
static int __exit repartition_exit(void)
{
	if (repartition_sysctl_header)
		unregister_sysctl_table(repartition_sysctl_header);
        return 0;
}
module_exit(repartition_exit);
#endif  /* MODULE */

module_init(repartition_init);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ke Dong <kedong@google.com>");
MODULE_DESCRIPTION("Partition map");

#endif  /* CONFIG_REPARTITION */
