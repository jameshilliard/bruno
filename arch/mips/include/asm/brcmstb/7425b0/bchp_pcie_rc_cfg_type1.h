/***************************************************************************
 *     Copyright (c) 1999-2010, Broadcom Corporation
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
 * Date:           Generated on         Mon May 17 04:38:33 2010
 *                 MD5 Checksum         d3eddea23beec7e33d13b4bcb5193126
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: $
 *
 ***************************************************************************/

#ifndef BCHP_PCIE_RC_CFG_TYPE1_H__
#define BCHP_PCIE_RC_CFG_TYPE1_H__

/***************************************************************************
 *PCIE_RC_CFG_TYPE1
 ***************************************************************************/
#define BCHP_PCIE_RC_CFG_TYPE1_DEVICE_VENDOR_ID  0x00410000 /* device_vendor_id */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND    0x00410004 /* status_command */
#define BCHP_PCIE_RC_CFG_TYPE1_REV_ID_CLASS_CODE 0x00410008 /* rev_id_class_code */
#define BCHP_PCIE_RC_CFG_TYPE1_HEADERTYPE_LAT_CACHELINESIZE 0x0041000c /* headertype_lat_cachelinesize */
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_1             0x00410010 /* bar_1 */
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_2             0x00410014 /* bar_2 */
#define BCHP_PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO    0x00410018 /* pri_sec_bus_no */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT 0x0041001c /* sec_status_io_base_limit */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT 0x00410020 /* rc_mem_base_limit */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT 0x00410024 /* rc_pref_base_limit */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_HI   0x00410028 /* rc_pref_base_hi */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_LIM_HI    0x0041002c /* rc_pref_lim_hi */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_IO_BASE_LIMIT  0x00410030 /* rc_io_base_limit */
#define BCHP_PCIE_RC_CFG_TYPE1_CAP_POINTER       0x00410034 /* cap_pointer */
#define BCHP_PCIE_RC_CFG_TYPE1_EXP_ROM_BAR       0x00410038 /* exp_rom_bar */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL    0x0041003c /* bridge_control */

/***************************************************************************
 *DEVICE_VENDOR_ID - device_vendor_id
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: DEVICE_VENDOR_ID :: DEVICE_ID [31:16] */
#define BCHP_PCIE_RC_CFG_TYPE1_DEVICE_VENDOR_ID_DEVICE_ID_MASK     0xffff0000
#define BCHP_PCIE_RC_CFG_TYPE1_DEVICE_VENDOR_ID_DEVICE_ID_SHIFT    16

/* PCIE_RC_CFG_TYPE1 :: DEVICE_VENDOR_ID :: VENDOR_ID [15:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_DEVICE_VENDOR_ID_VENDOR_ID_MASK     0x0000ffff
#define BCHP_PCIE_RC_CFG_TYPE1_DEVICE_VENDOR_ID_VENDOR_ID_SHIFT    0

/***************************************************************************
 *STATUS_COMMAND - status_command
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: PAR_ERR [31:31] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_PAR_ERR_MASK         0x80000000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_PAR_ERR_SHIFT        31

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: SIG_SERR [30:30] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_SIG_SERR_MASK        0x40000000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_SIG_SERR_SHIFT       30

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: RCV_MSTR_ABT [29:29] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_RCV_MSTR_ABT_MASK    0x20000000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_RCV_MSTR_ABT_SHIFT   29

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: RCV_TGT_ABT [28:28] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_RCV_TGT_ABT_MASK     0x10000000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_RCV_TGT_ABT_SHIFT    28

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: SIG_TGT_ABT [27:27] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_SIG_TGT_ABT_MASK     0x08000000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_SIG_TGT_ABT_SHIFT    27

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: DEVSEL_TIMING [26:25] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_DEVSEL_TIMING_MASK   0x06000000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_DEVSEL_TIMING_SHIFT  25

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: MSTR_PERR [24:24] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_MSTR_PERR_MASK       0x01000000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_MSTR_PERR_SHIFT      24

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: FAST_B2B_CAP [23:23] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_FAST_B2B_CAP_MASK    0x00800000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_FAST_B2B_CAP_SHIFT   23

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: RESERVED1 [22:22] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_RESERVED1_MASK       0x00400000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_RESERVED1_SHIFT      22

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: CAP_66MHZ [21:21] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_CAP_66MHZ_MASK       0x00200000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_CAP_66MHZ_SHIFT      21

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: CAP_LIST [20:20] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_CAP_LIST_MASK        0x00100000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_CAP_LIST_SHIFT       20

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: INT_STATUS [19:19] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_INT_STATUS_MASK      0x00080000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_INT_STATUS_SHIFT     19

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: RESERVED2 [18:16] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_RESERVED2_MASK       0x00070000
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_RESERVED2_SHIFT      16

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: RESERVED [15:11] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_RESERVED_MASK        0x0000f800
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_RESERVED_SHIFT       11

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: INT_DISABLE [10:10] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_INT_DISABLE_MASK     0x00000400
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_INT_DISABLE_SHIFT    10

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: FAST_B2B [09:09] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_FAST_B2B_MASK        0x00000200
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_FAST_B2B_SHIFT       9

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: SERR_ENA [08:08] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_SERR_ENA_MASK        0x00000100
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_SERR_ENA_SHIFT       8

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: STEPPING [07:07] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_STEPPING_MASK        0x00000080
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_STEPPING_SHIFT       7

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: PERR_ENA [06:06] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_PERR_ENA_MASK        0x00000040
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_PERR_ENA_SHIFT       6

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: VGA_SNOOP [05:05] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_VGA_SNOOP_MASK       0x00000020
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_VGA_SNOOP_SHIFT      5

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: MWI_CYCLES [04:04] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_MWI_CYCLES_MASK      0x00000010
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_MWI_CYCLES_SHIFT     4

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: SPECIAL_CYCLES [03:03] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_SPECIAL_CYCLES_MASK  0x00000008
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_SPECIAL_CYCLES_SHIFT 3

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: BUS_MASTER [02:02] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_BUS_MASTER_MASK      0x00000004
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_BUS_MASTER_SHIFT     2

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: MEM_SPACE [01:01] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_MEM_SPACE_MASK       0x00000002
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_MEM_SPACE_SHIFT      1

/* PCIE_RC_CFG_TYPE1 :: STATUS_COMMAND :: IO_SPACE [00:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_IO_SPACE_MASK        0x00000001
#define BCHP_PCIE_RC_CFG_TYPE1_STATUS_COMMAND_IO_SPACE_SHIFT       0

/***************************************************************************
 *REV_ID_CLASS_CODE - rev_id_class_code
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: REV_ID_CLASS_CODE :: CLASS_CODE [31:08] */
#define BCHP_PCIE_RC_CFG_TYPE1_REV_ID_CLASS_CODE_CLASS_CODE_MASK   0xffffff00
#define BCHP_PCIE_RC_CFG_TYPE1_REV_ID_CLASS_CODE_CLASS_CODE_SHIFT  8

/* PCIE_RC_CFG_TYPE1 :: REV_ID_CLASS_CODE :: REV_ID [07:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_REV_ID_CLASS_CODE_REV_ID_MASK       0x000000ff
#define BCHP_PCIE_RC_CFG_TYPE1_REV_ID_CLASS_CODE_REV_ID_SHIFT      0

/***************************************************************************
 *HEADERTYPE_LAT_CACHELINESIZE - headertype_lat_cachelinesize
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: HEADERTYPE_LAT_CACHELINESIZE :: BIST [31:24] */
#define BCHP_PCIE_RC_CFG_TYPE1_HEADERTYPE_LAT_CACHELINESIZE_BIST_MASK 0xff000000
#define BCHP_PCIE_RC_CFG_TYPE1_HEADERTYPE_LAT_CACHELINESIZE_BIST_SHIFT 24

/* PCIE_RC_CFG_TYPE1 :: HEADERTYPE_LAT_CACHELINESIZE :: HEADER_TYPE [23:16] */
#define BCHP_PCIE_RC_CFG_TYPE1_HEADERTYPE_LAT_CACHELINESIZE_HEADER_TYPE_MASK 0x00ff0000
#define BCHP_PCIE_RC_CFG_TYPE1_HEADERTYPE_LAT_CACHELINESIZE_HEADER_TYPE_SHIFT 16

/* PCIE_RC_CFG_TYPE1 :: HEADERTYPE_LAT_CACHELINESIZE :: LATENCY_TIMER [15:08] */
#define BCHP_PCIE_RC_CFG_TYPE1_HEADERTYPE_LAT_CACHELINESIZE_LATENCY_TIMER_MASK 0x0000ff00
#define BCHP_PCIE_RC_CFG_TYPE1_HEADERTYPE_LAT_CACHELINESIZE_LATENCY_TIMER_SHIFT 8

/* PCIE_RC_CFG_TYPE1 :: HEADERTYPE_LAT_CACHELINESIZE :: CACHE_LINE_SIZE [07:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_HEADERTYPE_LAT_CACHELINESIZE_CACHE_LINE_SIZE_MASK 0x000000ff
#define BCHP_PCIE_RC_CFG_TYPE1_HEADERTYPE_LAT_CACHELINESIZE_CACHE_LINE_SIZE_SHIFT 0

/***************************************************************************
 *BAR_1 - bar_1
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: BAR_1 :: ADDRESS [31:04] */
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_1_ADDRESS_MASK                  0xfffffff0
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_1_ADDRESS_SHIFT                 4

/* PCIE_RC_CFG_TYPE1 :: BAR_1 :: PREFETCH [03:03] */
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_1_PREFETCH_MASK                 0x00000008
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_1_PREFETCH_SHIFT                3

/* PCIE_RC_CFG_TYPE1 :: BAR_1 :: SPACE_TYPE [02:01] */
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_1_SPACE_TYPE_MASK               0x00000006
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_1_SPACE_TYPE_SHIFT              1

/* PCIE_RC_CFG_TYPE1 :: BAR_1 :: MEM_SPACE [00:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_1_MEM_SPACE_MASK                0x00000001
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_1_MEM_SPACE_SHIFT               0

/***************************************************************************
 *BAR_2 - bar_2
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: BAR_2 :: ADDR [31:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_2_ADDR_MASK                     0xffffffff
#define BCHP_PCIE_RC_CFG_TYPE1_BAR_2_ADDR_SHIFT                    0

/***************************************************************************
 *PRI_SEC_BUS_NO - pri_sec_bus_no
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: PRI_SEC_BUS_NO :: SEC_LATENCY_TIMER [31:24] */
#define BCHP_PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO_SEC_LATENCY_TIMER_MASK 0xff000000
#define BCHP_PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO_SEC_LATENCY_TIMER_SHIFT 24

/* PCIE_RC_CFG_TYPE1 :: PRI_SEC_BUS_NO :: SUB_BUS_NO [23:16] */
#define BCHP_PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO_SUB_BUS_NO_MASK      0x00ff0000
#define BCHP_PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO_SUB_BUS_NO_SHIFT     16

/* PCIE_RC_CFG_TYPE1 :: PRI_SEC_BUS_NO :: SEC_BUS_NO [15:08] */
#define BCHP_PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO_SEC_BUS_NO_MASK      0x0000ff00
#define BCHP_PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO_SEC_BUS_NO_SHIFT     8

/* PCIE_RC_CFG_TYPE1 :: PRI_SEC_BUS_NO :: PRI_BUS_NO [07:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO_PRI_BUS_NO_MASK      0x000000ff
#define BCHP_PCIE_RC_CFG_TYPE1_PRI_SEC_BUS_NO_PRI_BUS_NO_SHIFT     0

/***************************************************************************
 *SEC_STATUS_IO_BASE_LIMIT - sec_status_io_base_limit
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: SEC_DETECTED_PARITY_ERROR [31:31] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_DETECTED_PARITY_ERROR_MASK 0x80000000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_DETECTED_PARITY_ERROR_SHIFT 31

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: SEC_RECEIVED_SYSTEM_ERROR [30:30] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_RECEIVED_SYSTEM_ERROR_MASK 0x40000000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_RECEIVED_SYSTEM_ERROR_SHIFT 30

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: SEC_RECEIVED_MASTER_ABORT [29:29] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_RECEIVED_MASTER_ABORT_MASK 0x20000000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_RECEIVED_MASTER_ABORT_SHIFT 29

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: SEC_RECEIVED_TARGET_ABORT [28:28] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_RECEIVED_TARGET_ABORT_MASK 0x10000000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_RECEIVED_TARGET_ABORT_SHIFT 28

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: SEC_SIGNALED_TARGET_ABORT [27:27] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_SIGNALED_TARGET_ABORT_MASK 0x08000000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_SIGNALED_TARGET_ABORT_SHIFT 27

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: DEVICE_TIMING [26:25] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_DEVICE_TIMING_MASK 0x06000000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_DEVICE_TIMING_SHIFT 25

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: SEC_MASTER_DATA_PARITY_ERROR [24:24] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_MASTER_DATA_PARITY_ERROR_MASK 0x01000000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_SEC_MASTER_DATA_PARITY_ERROR_SHIFT 24

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: FAST_B2B [23:23] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_FAST_B2B_MASK 0x00800000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_FAST_B2B_SHIFT 23

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: RESERVED3 [22:22] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_RESERVED3_MASK 0x00400000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_RESERVED3_SHIFT 22

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: CAP_66_MHZ [21:21] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_CAP_66_MHZ_MASK 0x00200000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_CAP_66_MHZ_SHIFT 21

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: RESERVED4 [20:16] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_RESERVED4_MASK 0x001f0000
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_RESERVED4_SHIFT 16

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: IO_LIMIT [15:08] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_IO_LIMIT_MASK 0x0000ff00
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_IO_LIMIT_SHIFT 8

/* PCIE_RC_CFG_TYPE1 :: SEC_STATUS_IO_BASE_LIMIT :: IO_BASE [07:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_IO_BASE_MASK 0x000000ff
#define BCHP_PCIE_RC_CFG_TYPE1_SEC_STATUS_IO_BASE_LIMIT_IO_BASE_SHIFT 0

/***************************************************************************
 *RC_MEM_BASE_LIMIT - rc_mem_base_limit
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: RC_MEM_BASE_LIMIT :: RC_MEM_LIMIT [31:20] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT_RC_MEM_LIMIT_MASK 0xfff00000
#define BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT_RC_MEM_LIMIT_SHIFT 20

/* PCIE_RC_CFG_TYPE1 :: RC_MEM_BASE_LIMIT :: RC_MEM_LIMIT_3TO0 [19:16] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT_RC_MEM_LIMIT_3TO0_MASK 0x000f0000
#define BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT_RC_MEM_LIMIT_3TO0_SHIFT 16

/* PCIE_RC_CFG_TYPE1 :: RC_MEM_BASE_LIMIT :: RC_MEM_BASE [15:04] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT_RC_MEM_BASE_MASK  0x0000fff0
#define BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT_RC_MEM_BASE_SHIFT 4

/* PCIE_RC_CFG_TYPE1 :: RC_MEM_BASE_LIMIT :: RC_MEM_BASE_3TO0 [03:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT_RC_MEM_BASE_3TO0_MASK 0x0000000f
#define BCHP_PCIE_RC_CFG_TYPE1_RC_MEM_BASE_LIMIT_RC_MEM_BASE_3TO0_SHIFT 0

/***************************************************************************
 *RC_PREF_BASE_LIMIT - rc_pref_base_limit
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: RC_PREF_BASE_LIMIT :: RC_PREF_LIM [31:20] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT_RC_PREF_LIM_MASK 0xfff00000
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT_RC_PREF_LIM_SHIFT 20

/* PCIE_RC_CFG_TYPE1 :: RC_PREF_BASE_LIMIT :: RC_PREF_LIMIT_3TO0 [19:16] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT_RC_PREF_LIMIT_3TO0_MASK 0x000f0000
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT_RC_PREF_LIMIT_3TO0_SHIFT 16

/* PCIE_RC_CFG_TYPE1 :: RC_PREF_BASE_LIMIT :: RC_PREF_BASE [15:04] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT_RC_PREF_BASE_MASK 0x0000fff0
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT_RC_PREF_BASE_SHIFT 4

/* PCIE_RC_CFG_TYPE1 :: RC_PREF_BASE_LIMIT :: RC_PREF_BASE_3TO0 [03:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT_RC_PREF_BASE_3TO0_MASK 0x0000000f
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_LIMIT_RC_PREF_BASE_3TO0_SHIFT 0

/***************************************************************************
 *RC_PREF_BASE_HI - rc_pref_base_hi
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: RC_PREF_BASE_HI :: RC_PREF_BASE_HI [31:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_HI_RC_PREF_BASE_HI_MASK 0xffffffff
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_BASE_HI_RC_PREF_BASE_HI_SHIFT 0

/***************************************************************************
 *RC_PREF_LIM_HI - rc_pref_lim_hi
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: RC_PREF_LIM_HI :: RC_PREF_BASE_HI [31:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_LIM_HI_RC_PREF_BASE_HI_MASK 0xffffffff
#define BCHP_PCIE_RC_CFG_TYPE1_RC_PREF_LIM_HI_RC_PREF_BASE_HI_SHIFT 0

/***************************************************************************
 *RC_IO_BASE_LIMIT - rc_io_base_limit
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: RC_IO_BASE_LIMIT :: RC_IO_LIM_HI [31:16] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_IO_BASE_LIMIT_RC_IO_LIM_HI_MASK  0xffff0000
#define BCHP_PCIE_RC_CFG_TYPE1_RC_IO_BASE_LIMIT_RC_IO_LIM_HI_SHIFT 16

/* PCIE_RC_CFG_TYPE1 :: RC_IO_BASE_LIMIT :: RC_IO_BASE_HI [15:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_RC_IO_BASE_LIMIT_RC_IO_BASE_HI_MASK 0x0000ffff
#define BCHP_PCIE_RC_CFG_TYPE1_RC_IO_BASE_LIMIT_RC_IO_BASE_HI_SHIFT 0

/***************************************************************************
 *CAP_POINTER - cap_pointer
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: CAP_POINTER :: RESERVED0 [31:08] */
#define BCHP_PCIE_RC_CFG_TYPE1_CAP_POINTER_RESERVED0_MASK          0xffffff00
#define BCHP_PCIE_RC_CFG_TYPE1_CAP_POINTER_RESERVED0_SHIFT         8

/* PCIE_RC_CFG_TYPE1 :: CAP_POINTER :: CAP_POINTER [07:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_CAP_POINTER_CAP_POINTER_MASK        0x000000ff
#define BCHP_PCIE_RC_CFG_TYPE1_CAP_POINTER_CAP_POINTER_SHIFT       0

/***************************************************************************
 *EXP_ROM_BAR - exp_rom_bar
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: EXP_ROM_BAR :: RESERVED0 [31:08] */
#define BCHP_PCIE_RC_CFG_TYPE1_EXP_ROM_BAR_RESERVED0_MASK          0xffffff00
#define BCHP_PCIE_RC_CFG_TYPE1_EXP_ROM_BAR_RESERVED0_SHIFT         8

/* PCIE_RC_CFG_TYPE1 :: EXP_ROM_BAR :: EXP_ROM_BAR [07:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_EXP_ROM_BAR_EXP_ROM_BAR_MASK        0x000000ff
#define BCHP_PCIE_RC_CFG_TYPE1_EXP_ROM_BAR_EXP_ROM_BAR_SHIFT       0

/***************************************************************************
 *BRIDGE_CONTROL - bridge_control
 ***************************************************************************/
/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: RESERVED1 [31:28] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_RESERVED1_MASK       0xf0000000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_RESERVED1_SHIFT      28

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: DISCARD_TIMER_SERR_EN [27:27] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_DISCARD_TIMER_SERR_EN_MASK 0x08000000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_DISCARD_TIMER_SERR_EN_SHIFT 27

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: DISCARD_TIMER_STATUS [26:26] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_DISCARD_TIMER_STATUS_MASK 0x04000000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_DISCARD_TIMER_STATUS_SHIFT 26

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: SEC_DISCARD_TIMER [25:25] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_SEC_DISCARD_TIMER_MASK 0x02000000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_SEC_DISCARD_TIMER_SHIFT 25

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: PRIM_DISCARD_TIMER [24:24] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_PRIM_DISCARD_TIMER_MASK 0x01000000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_PRIM_DISCARD_TIMER_SHIFT 24

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: FAST_B2B_EN [23:23] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_FAST_B2B_EN_MASK     0x00800000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_FAST_B2B_EN_SHIFT    23

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: SEC_BUS_RESET [22:22] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_SEC_BUS_RESET_MASK   0x00400000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_SEC_BUS_RESET_SHIFT  22

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: MASTER_ABORT_MODE [21:21] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_MASTER_ABORT_MODE_MASK 0x00200000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_MASTER_ABORT_MODE_SHIFT 21

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: RESERVED0 [20:18] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_RESERVED0_MASK       0x001c0000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_RESERVED0_SHIFT      18

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: SEC_SERR_EN [17:17] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_SEC_SERR_EN_MASK     0x00020000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_SEC_SERR_EN_SHIFT    17

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: SEC_PERR_RESP_EN [16:16] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_SEC_PERR_RESP_EN_MASK 0x00010000
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_SEC_PERR_RESP_EN_SHIFT 16

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: INT_PIN [15:08] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_INT_PIN_MASK         0x0000ff00
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_INT_PIN_SHIFT        8

/* PCIE_RC_CFG_TYPE1 :: BRIDGE_CONTROL :: INT_LINE [07:00] */
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_INT_LINE_MASK        0x000000ff
#define BCHP_PCIE_RC_CFG_TYPE1_BRIDGE_CONTROL_INT_LINE_SHIFT       0

#endif /* #ifndef BCHP_PCIE_RC_CFG_TYPE1_H__ */

/* End of File */
