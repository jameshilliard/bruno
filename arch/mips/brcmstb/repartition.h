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
#ifndef __REPARTITION_H__
#define __REPARTITION_H__

extern void register_nand(struct platform_device *pdev);
/* init_nand is defined in setup.c, because most of the code uses the local
 * variable cs_info and local function brcm_setup_cs in setup.c. */
extern void init_nand(void);

#endif  /* __REPARTITION_H__ */
