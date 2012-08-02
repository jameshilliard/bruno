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
#ifndef __PARTITIONMAP_H__
#define __PARTITIONMAP_H__

extern int partitionmap_version;
extern int fixed_nor_partition_map_size;
extern int fixed_nand_partition_map_size;
extern struct mtd_partition *fixed_nor_partition_map;
extern struct mtd_partition *fixed_nand_partition_map;
extern size_t partitionmap_print_info(char *buffer, size_t size);
extern int switch_partition(int pver);
extern void register_nand(struct platform_device *pdev);
extern void flush_nand(void);

#endif  /* __PARTITIONMAP_H__ */
