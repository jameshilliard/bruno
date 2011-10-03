/***************************************************************************
 *     Copyright (c) 1999-2011, Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Module Description:
 *                     DO NOT EDIT THIS FILE DIRECTLY
 *
 * This module was generated magically with RDB from a source description
 * file. You must edit the source file for changes to be made to this file.
 *
 *
 * Date:           Generated on         Wed Apr 20 10:24:02 2011
 *                 MD5 Checksum         f8b208c9aa3ad321e844687836b90339
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/35330/rdb/a0/bchp_irq0.h $
 * 
 * Hydra_Software_Devel/4   4/21/11 11:09a yuxiaz
 * SWDTV-6742: Updated RDB files.
 *
 ***************************************************************************/

#ifndef BCHP_IRQ0_H__
#define BCHP_IRQ0_H__

/***************************************************************************
 *IRQ0 - Level 2 CPU Interrupt Enable/Status
 ***************************************************************************/
#define BCHP_IRQ0_IRQEN                          0x004062b0 /* Interrupt Enable */
#define BCHP_IRQ0_IRQSTAT                        0x004062b4 /* Interrupt Status */

/***************************************************************************
 *IRQEN - Interrupt Enable
 ***************************************************************************/
/* IRQ0 :: IRQEN :: reserved0 [31:28] */
#define BCHP_IRQ0_IRQEN_reserved0_MASK                             0xf0000000
#define BCHP_IRQ0_IRQEN_reserved0_SHIFT                            28

/* IRQ0 :: IRQEN :: reserved_for_eco1 [27:24] */
#define BCHP_IRQ0_IRQEN_reserved_for_eco1_MASK                     0x0f000000
#define BCHP_IRQ0_IRQEN_reserved_for_eco1_SHIFT                    24
#define BCHP_IRQ0_IRQEN_reserved_for_eco1_DEFAULT                  0

/* IRQ0 :: IRQEN :: reserved2 [23:21] */
#define BCHP_IRQ0_IRQEN_reserved2_MASK                             0x00e00000
#define BCHP_IRQ0_IRQEN_reserved2_SHIFT                            21

/* IRQ0 :: IRQEN :: reserved_for_eco3 [20:16] */
#define BCHP_IRQ0_IRQEN_reserved_for_eco3_MASK                     0x001f0000
#define BCHP_IRQ0_IRQEN_reserved_for_eco3_SHIFT                    16
#define BCHP_IRQ0_IRQEN_reserved_for_eco3_DEFAULT                  0

/* IRQ0 :: IRQEN :: reserved4 [15:10] */
#define BCHP_IRQ0_IRQEN_reserved4_MASK                             0x0000fc00
#define BCHP_IRQ0_IRQEN_reserved4_SHIFT                            10

/* IRQ0 :: IRQEN :: reserved_for_eco5 [09:07] */
#define BCHP_IRQ0_IRQEN_reserved_for_eco5_MASK                     0x00000380
#define BCHP_IRQ0_IRQEN_reserved_for_eco5_SHIFT                    7
#define BCHP_IRQ0_IRQEN_reserved_for_eco5_DEFAULT                  0

/* IRQ0 :: IRQEN :: gio_irqen [06:06] */
#define BCHP_IRQ0_IRQEN_gio_irqen_MASK                             0x00000040
#define BCHP_IRQ0_IRQEN_gio_irqen_SHIFT                            6
#define BCHP_IRQ0_IRQEN_gio_irqen_DEFAULT                          0

/* IRQ0 :: IRQEN :: reserved_for_eco6 [05:00] */
#define BCHP_IRQ0_IRQEN_reserved_for_eco6_MASK                     0x0000003f
#define BCHP_IRQ0_IRQEN_reserved_for_eco6_SHIFT                    0
#define BCHP_IRQ0_IRQEN_reserved_for_eco6_DEFAULT                  0

/***************************************************************************
 *IRQSTAT - Interrupt Status
 ***************************************************************************/
/* IRQ0 :: IRQSTAT :: reserved0 [31:07] */
#define BCHP_IRQ0_IRQSTAT_reserved0_MASK                           0xffffff80
#define BCHP_IRQ0_IRQSTAT_reserved0_SHIFT                          7

/* IRQ0 :: IRQSTAT :: gioirq [06:06] */
#define BCHP_IRQ0_IRQSTAT_gioirq_MASK                              0x00000040
#define BCHP_IRQ0_IRQSTAT_gioirq_SHIFT                             6
#define BCHP_IRQ0_IRQSTAT_gioirq_DEFAULT                           0

/* IRQ0 :: IRQSTAT :: reserved1 [05:00] */
#define BCHP_IRQ0_IRQSTAT_reserved1_MASK                           0x0000003f
#define BCHP_IRQ0_IRQSTAT_reserved1_SHIFT                          0

#endif /* #ifndef BCHP_IRQ0_H__ */

/* End of File */
