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
 * Date:           Generated on         Tue Mar 15 16:00:03 2011
 *                 MD5 Checksum         0ee19441ea736d2ffddf4654d321c8fb
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7425/rdb/a0/bchp_pcie_dma.h $
 * 
 * Hydra_Software_Devel/2   3/17/11 11:32a vanessah
 * SW7425-6: sync with RDB
 *
 ***************************************************************************/

#ifndef BCHP_PCIE_DMA_H__
#define BCHP_PCIE_DMA_H__

/***************************************************************************
 *PCIE_DMA - PCI-E DMA Registers
 ***************************************************************************/
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST0 0x00414400 /* Tx Descriptor List0 First Descriptor lower Address */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_U_ADDR_LIST0 0x00414404 /* Tx Descriptor List0 First Descriptor Upper Address */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST1 0x00414408 /* Tx Descriptor List1 First Descriptor Lower Address */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_U_ADDR_LIST1 0x0041440c /* Tx Descriptor List1 First Descriptor Upper Address */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS   0x00414410 /* Tx Software Descriptor List Control and Status */
#define BCHP_PCIE_DMA_TX_WAKE_CTRL               0x00414414 /* Tx Wake Control */
#define BCHP_PCIE_DMA_TX_ERROR_STATUS            0x00414418 /* Tx Engine Error Status */
#define BCHP_PCIE_DMA_TX_LIST0_CUR_DESC_L_ADDR   0x0041441c /* Tx List0 Current Descriptor Lower Address */
#define BCHP_PCIE_DMA_TX_LIST0_CUR_DESC_U_ADDR   0x00414420 /* Tx List0 Current Descriptor Upper Address */
#define BCHP_PCIE_DMA_TX_LIST0_CUR_BYTE_CNT      0x00414424 /* Tx List0 Current Descriptor Byte Count */
#define BCHP_PCIE_DMA_TX_LIST1_CUR_DESC_L_ADDR   0x00414428 /* Tx List1 Current Descriptor Lower Address */
#define BCHP_PCIE_DMA_TX_LIST1_CUR_DESC_U_ADDR   0x0041442c /* Tx List1 Current Descriptor Upper Address */
#define BCHP_PCIE_DMA_TX_LIST1_CUR_BYTE_CNT      0x00414430 /* Tx List1 Current Descriptor Byte Count */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST0 0x00414434 /* Rx Descriptor List0 First Descriptor Lower Address */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_U_ADDR_LIST0 0x00414438 /* Rx Descriptor List0 First Descriptor Upper Address */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST1 0x0041443c /* Rx Descriptor List1 First Descriptor Lower Address */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_U_ADDR_LIST1 0x00414440 /* Rx Descriptor List1 First Descriptor Upper Address */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS   0x00414444 /* Rx Software Descriptor List Control and Status */
#define BCHP_PCIE_DMA_RX_WAKE_CTRL               0x00414448 /* Rx DMA Wake Control */
#define BCHP_PCIE_DMA_RX_ERROR_STATUS            0x0041444c /* Rx Engine Error Status */
#define BCHP_PCIE_DMA_RX_LIST0_CUR_DESC_L_ADDR   0x00414450 /* Rx List0 Current Descriptor Lower Address */
#define BCHP_PCIE_DMA_RX_LIST0_CUR_DESC_U_ADDR   0x00414454 /* Rx List0 Current Descriptor Upper Address */
#define BCHP_PCIE_DMA_RX_LIST0_CUR_BYTE_CNT      0x00414458 /* Rx List0 Current Descriptor Byte Count */
#define BCHP_PCIE_DMA_RX_LIST1_CUR_DESC_L_ADDR   0x0041445c /* Rx List1 Current Descriptor Lower address */
#define BCHP_PCIE_DMA_RX_LIST1_CUR_DESC_U_ADDR   0x00414460 /* Rx List1 Current Descriptor Upper address */
#define BCHP_PCIE_DMA_RX_LIST1_CUR_BYTE_CNT      0x00414464 /* Rx List1 Current Descriptor Byte Count */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG      0x00414468 /* DMA Debug Options Register */
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS  0x0041446c /* Read Channel Error Status */

/***************************************************************************
 *TX_FIRST_DESC_L_ADDR_LIST0 - Tx Descriptor List0 First Descriptor lower Address
 ***************************************************************************/
/* PCIE_DMA :: TX_FIRST_DESC_L_ADDR_LIST0 :: DESC_ADDR [31:05] */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST0_DESC_ADDR_MASK    0xffffffe0
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST0_DESC_ADDR_SHIFT   5
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST0_DESC_ADDR_DEFAULT 0

/* PCIE_DMA :: TX_FIRST_DESC_L_ADDR_LIST0 :: reserved0 [04:01] */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST0_reserved0_MASK    0x0000001e
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST0_reserved0_SHIFT   1

/* PCIE_DMA :: TX_FIRST_DESC_L_ADDR_LIST0 :: TX_DESC_LIST0_VALID [00:00] */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST0_TX_DESC_LIST0_VALID_MASK 0x00000001
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST0_TX_DESC_LIST0_VALID_SHIFT 0
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST0_TX_DESC_LIST0_VALID_DEFAULT 0

/***************************************************************************
 *TX_FIRST_DESC_U_ADDR_LIST0 - Tx Descriptor List0 First Descriptor Upper Address
 ***************************************************************************/
/* PCIE_DMA :: TX_FIRST_DESC_U_ADDR_LIST0 :: DESC_ADDR [31:00] */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_U_ADDR_LIST0_DESC_ADDR_MASK    0xffffffff
#define BCHP_PCIE_DMA_TX_FIRST_DESC_U_ADDR_LIST0_DESC_ADDR_SHIFT   0
#define BCHP_PCIE_DMA_TX_FIRST_DESC_U_ADDR_LIST0_DESC_ADDR_DEFAULT 0

/***************************************************************************
 *TX_FIRST_DESC_L_ADDR_LIST1 - Tx Descriptor List1 First Descriptor Lower Address
 ***************************************************************************/
/* PCIE_DMA :: TX_FIRST_DESC_L_ADDR_LIST1 :: DESC_ADDR [31:05] */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST1_DESC_ADDR_MASK    0xffffffe0
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST1_DESC_ADDR_SHIFT   5
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST1_DESC_ADDR_DEFAULT 0

/* PCIE_DMA :: TX_FIRST_DESC_L_ADDR_LIST1 :: reserved0 [04:01] */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST1_reserved0_MASK    0x0000001e
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST1_reserved0_SHIFT   1

/* PCIE_DMA :: TX_FIRST_DESC_L_ADDR_LIST1 :: TX_DESC_LIST1_VALID [00:00] */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST1_TX_DESC_LIST1_VALID_MASK 0x00000001
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST1_TX_DESC_LIST1_VALID_SHIFT 0
#define BCHP_PCIE_DMA_TX_FIRST_DESC_L_ADDR_LIST1_TX_DESC_LIST1_VALID_DEFAULT 0

/***************************************************************************
 *TX_FIRST_DESC_U_ADDR_LIST1 - Tx Descriptor List1 First Descriptor Upper Address
 ***************************************************************************/
/* PCIE_DMA :: TX_FIRST_DESC_U_ADDR_LIST1 :: DESC_ADDR [31:00] */
#define BCHP_PCIE_DMA_TX_FIRST_DESC_U_ADDR_LIST1_DESC_ADDR_MASK    0xffffffff
#define BCHP_PCIE_DMA_TX_FIRST_DESC_U_ADDR_LIST1_DESC_ADDR_SHIFT   0
#define BCHP_PCIE_DMA_TX_FIRST_DESC_U_ADDR_LIST1_DESC_ADDR_DEFAULT 0

/***************************************************************************
 *TX_SW_DESC_LIST_CTRL_STS - Tx Software Descriptor List Control and Status
 ***************************************************************************/
/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: reserved0 [31:14] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_reserved0_MASK      0xffffc000
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_reserved0_SHIFT     14

/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: DESC_ENDIAN_MODE [13:12] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DESC_ENDIAN_MODE_MASK 0x00003000
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DESC_ENDIAN_MODE_SHIFT 12
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DESC_ENDIAN_MODE_DEFAULT 0

/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: reserved1 [11:10] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_reserved1_MASK      0x00000c00
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_reserved1_SHIFT     10

/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: LOCAL_DESC [09:09] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_LOCAL_DESC_MASK     0x00000200
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_LOCAL_DESC_SHIFT    9
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_LOCAL_DESC_DEFAULT  0

/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: DMA_MODE [08:08] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DMA_MODE_MASK       0x00000100
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DMA_MODE_SHIFT      8
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DMA_MODE_DEFAULT    0

/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: reserved2 [07:06] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_reserved2_MASK      0x000000c0
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_reserved2_SHIFT     6

/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: DMA_STATUS [05:04] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DMA_STATUS_MASK     0x00000030
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DMA_STATUS_SHIFT    4
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DMA_STATUS_DEFAULT  0

/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: DMA_DATA_SERV_PTR [03:03] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DMA_DATA_SERV_PTR_MASK 0x00000008
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DMA_DATA_SERV_PTR_SHIFT 3
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DMA_DATA_SERV_PTR_DEFAULT 0

/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: DESC_SERV_PTR [02:02] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DESC_SERV_PTR_MASK  0x00000004
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DESC_SERV_PTR_SHIFT 2
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_DESC_SERV_PTR_DEFAULT 0

/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: reserved3 [01:01] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_reserved3_MASK      0x00000002
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_reserved3_SHIFT     1

/* PCIE_DMA :: TX_SW_DESC_LIST_CTRL_STS :: TX_DMA_RUN_STOP [00:00] */
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_TX_DMA_RUN_STOP_MASK 0x00000001
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_TX_DMA_RUN_STOP_SHIFT 0
#define BCHP_PCIE_DMA_TX_SW_DESC_LIST_CTRL_STS_TX_DMA_RUN_STOP_DEFAULT 0

/***************************************************************************
 *TX_WAKE_CTRL - Tx Wake Control
 ***************************************************************************/
/* PCIE_DMA :: TX_WAKE_CTRL :: reserved0 [31:02] */
#define BCHP_PCIE_DMA_TX_WAKE_CTRL_reserved0_MASK                  0xfffffffc
#define BCHP_PCIE_DMA_TX_WAKE_CTRL_reserved0_SHIFT                 2

/* PCIE_DMA :: TX_WAKE_CTRL :: WAKE_MODE [01:01] */
#define BCHP_PCIE_DMA_TX_WAKE_CTRL_WAKE_MODE_MASK                  0x00000002
#define BCHP_PCIE_DMA_TX_WAKE_CTRL_WAKE_MODE_SHIFT                 1
#define BCHP_PCIE_DMA_TX_WAKE_CTRL_WAKE_MODE_DEFAULT               0

/* PCIE_DMA :: TX_WAKE_CTRL :: WAKE [00:00] */
#define BCHP_PCIE_DMA_TX_WAKE_CTRL_WAKE_MASK                       0x00000001
#define BCHP_PCIE_DMA_TX_WAKE_CTRL_WAKE_SHIFT                      0
#define BCHP_PCIE_DMA_TX_WAKE_CTRL_WAKE_DEFAULT                    0

/***************************************************************************
 *TX_ERROR_STATUS - Tx Engine Error Status
 ***************************************************************************/
/* PCIE_DMA :: TX_ERROR_STATUS :: reserved0 [31:10] */
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_reserved0_MASK               0xfffffc00
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_reserved0_SHIFT              10

/* PCIE_DMA :: TX_ERROR_STATUS :: TX_L1_DESC_TX_ABORT_ERRORS [09:09] */
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L1_DESC_TX_ABORT_ERRORS_MASK 0x00000200
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L1_DESC_TX_ABORT_ERRORS_SHIFT 9
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L1_DESC_TX_ABORT_ERRORS_DEFAULT 0

/* PCIE_DMA :: TX_ERROR_STATUS :: reserved1 [08:08] */
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_reserved1_MASK               0x00000100
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_reserved1_SHIFT              8

/* PCIE_DMA :: TX_ERROR_STATUS :: TX_L0_DESC_TX_ABORT_ERRORS [07:07] */
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L0_DESC_TX_ABORT_ERRORS_MASK 0x00000080
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L0_DESC_TX_ABORT_ERRORS_SHIFT 7
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L0_DESC_TX_ABORT_ERRORS_DEFAULT 0

/* PCIE_DMA :: TX_ERROR_STATUS :: reserved2 [06:06] */
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_reserved2_MASK               0x00000040
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_reserved2_SHIFT              6

/* PCIE_DMA :: TX_ERROR_STATUS :: TX_L1_DATA_TX_ABORT_ERRORS [05:05] */
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L1_DATA_TX_ABORT_ERRORS_MASK 0x00000020
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L1_DATA_TX_ABORT_ERRORS_SHIFT 5
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L1_DATA_TX_ABORT_ERRORS_DEFAULT 0

/* PCIE_DMA :: TX_ERROR_STATUS :: reserved3 [04:03] */
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_reserved3_MASK               0x00000018
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_reserved3_SHIFT              3

/* PCIE_DMA :: TX_ERROR_STATUS :: TX_L0_DATA_TX_ABORT_ERRORS [02:02] */
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L0_DATA_TX_ABORT_ERRORS_MASK 0x00000004
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L0_DATA_TX_ABORT_ERRORS_SHIFT 2
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_TX_L0_DATA_TX_ABORT_ERRORS_DEFAULT 0

/* PCIE_DMA :: TX_ERROR_STATUS :: reserved4 [01:00] */
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_reserved4_MASK               0x00000003
#define BCHP_PCIE_DMA_TX_ERROR_STATUS_reserved4_SHIFT              0

/***************************************************************************
 *TX_LIST0_CUR_DESC_L_ADDR - Tx List0 Current Descriptor Lower Address
 ***************************************************************************/
/* PCIE_DMA :: TX_LIST0_CUR_DESC_L_ADDR :: TX_L0_CUR_DESC_L_ADDR [31:05] */
#define BCHP_PCIE_DMA_TX_LIST0_CUR_DESC_L_ADDR_TX_L0_CUR_DESC_L_ADDR_MASK 0xffffffe0
#define BCHP_PCIE_DMA_TX_LIST0_CUR_DESC_L_ADDR_TX_L0_CUR_DESC_L_ADDR_SHIFT 5
#define BCHP_PCIE_DMA_TX_LIST0_CUR_DESC_L_ADDR_TX_L0_CUR_DESC_L_ADDR_DEFAULT 0

/* PCIE_DMA :: TX_LIST0_CUR_DESC_L_ADDR :: reserved0 [04:00] */
#define BCHP_PCIE_DMA_TX_LIST0_CUR_DESC_L_ADDR_reserved0_MASK      0x0000001f
#define BCHP_PCIE_DMA_TX_LIST0_CUR_DESC_L_ADDR_reserved0_SHIFT     0

/***************************************************************************
 *TX_LIST0_CUR_DESC_U_ADDR - Tx List0 Current Descriptor Upper Address
 ***************************************************************************/
/* PCIE_DMA :: TX_LIST0_CUR_DESC_U_ADDR :: TX_L0_CUR_DESC_U_ADDR [31:00] */
#define BCHP_PCIE_DMA_TX_LIST0_CUR_DESC_U_ADDR_TX_L0_CUR_DESC_U_ADDR_MASK 0xffffffff
#define BCHP_PCIE_DMA_TX_LIST0_CUR_DESC_U_ADDR_TX_L0_CUR_DESC_U_ADDR_SHIFT 0
#define BCHP_PCIE_DMA_TX_LIST0_CUR_DESC_U_ADDR_TX_L0_CUR_DESC_U_ADDR_DEFAULT 0

/***************************************************************************
 *TX_LIST0_CUR_BYTE_CNT - Tx List0 Current Descriptor Byte Count
 ***************************************************************************/
/* PCIE_DMA :: TX_LIST0_CUR_BYTE_CNT :: TX_L0_CUR_BYTE_CNT [31:00] */
#define BCHP_PCIE_DMA_TX_LIST0_CUR_BYTE_CNT_TX_L0_CUR_BYTE_CNT_MASK 0xffffffff
#define BCHP_PCIE_DMA_TX_LIST0_CUR_BYTE_CNT_TX_L0_CUR_BYTE_CNT_SHIFT 0
#define BCHP_PCIE_DMA_TX_LIST0_CUR_BYTE_CNT_TX_L0_CUR_BYTE_CNT_DEFAULT 0

/***************************************************************************
 *TX_LIST1_CUR_DESC_L_ADDR - Tx List1 Current Descriptor Lower Address
 ***************************************************************************/
/* PCIE_DMA :: TX_LIST1_CUR_DESC_L_ADDR :: TX_L1_CUR_DESC_L_ADDR [31:05] */
#define BCHP_PCIE_DMA_TX_LIST1_CUR_DESC_L_ADDR_TX_L1_CUR_DESC_L_ADDR_MASK 0xffffffe0
#define BCHP_PCIE_DMA_TX_LIST1_CUR_DESC_L_ADDR_TX_L1_CUR_DESC_L_ADDR_SHIFT 5
#define BCHP_PCIE_DMA_TX_LIST1_CUR_DESC_L_ADDR_TX_L1_CUR_DESC_L_ADDR_DEFAULT 0

/* PCIE_DMA :: TX_LIST1_CUR_DESC_L_ADDR :: reserved0 [04:00] */
#define BCHP_PCIE_DMA_TX_LIST1_CUR_DESC_L_ADDR_reserved0_MASK      0x0000001f
#define BCHP_PCIE_DMA_TX_LIST1_CUR_DESC_L_ADDR_reserved0_SHIFT     0

/***************************************************************************
 *TX_LIST1_CUR_DESC_U_ADDR - Tx List1 Current Descriptor Upper Address
 ***************************************************************************/
/* PCIE_DMA :: TX_LIST1_CUR_DESC_U_ADDR :: TX_L1_CUR_DESC_U_ADDR [31:00] */
#define BCHP_PCIE_DMA_TX_LIST1_CUR_DESC_U_ADDR_TX_L1_CUR_DESC_U_ADDR_MASK 0xffffffff
#define BCHP_PCIE_DMA_TX_LIST1_CUR_DESC_U_ADDR_TX_L1_CUR_DESC_U_ADDR_SHIFT 0
#define BCHP_PCIE_DMA_TX_LIST1_CUR_DESC_U_ADDR_TX_L1_CUR_DESC_U_ADDR_DEFAULT 0

/***************************************************************************
 *TX_LIST1_CUR_BYTE_CNT - Tx List1 Current Descriptor Byte Count
 ***************************************************************************/
/* PCIE_DMA :: TX_LIST1_CUR_BYTE_CNT :: TX_L1_CUR_BYTE_CNT [31:00] */
#define BCHP_PCIE_DMA_TX_LIST1_CUR_BYTE_CNT_TX_L1_CUR_BYTE_CNT_MASK 0xffffffff
#define BCHP_PCIE_DMA_TX_LIST1_CUR_BYTE_CNT_TX_L1_CUR_BYTE_CNT_SHIFT 0
#define BCHP_PCIE_DMA_TX_LIST1_CUR_BYTE_CNT_TX_L1_CUR_BYTE_CNT_DEFAULT 0

/***************************************************************************
 *RX_FIRST_DESC_L_ADDR_LIST0 - Rx Descriptor List0 First Descriptor Lower Address
 ***************************************************************************/
/* PCIE_DMA :: RX_FIRST_DESC_L_ADDR_LIST0 :: DESC_ADDR [31:05] */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST0_DESC_ADDR_MASK    0xffffffe0
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST0_DESC_ADDR_SHIFT   5
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST0_DESC_ADDR_DEFAULT 0

/* PCIE_DMA :: RX_FIRST_DESC_L_ADDR_LIST0 :: reserved0 [04:01] */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST0_reserved0_MASK    0x0000001e
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST0_reserved0_SHIFT   1

/* PCIE_DMA :: RX_FIRST_DESC_L_ADDR_LIST0 :: RX_DESC_LIST0_VALID [00:00] */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST0_RX_DESC_LIST0_VALID_MASK 0x00000001
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST0_RX_DESC_LIST0_VALID_SHIFT 0
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST0_RX_DESC_LIST0_VALID_DEFAULT 0

/***************************************************************************
 *RX_FIRST_DESC_U_ADDR_LIST0 - Rx Descriptor List0 First Descriptor Upper Address
 ***************************************************************************/
/* PCIE_DMA :: RX_FIRST_DESC_U_ADDR_LIST0 :: DESC_ADDR [31:00] */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_U_ADDR_LIST0_DESC_ADDR_MASK    0xffffffff
#define BCHP_PCIE_DMA_RX_FIRST_DESC_U_ADDR_LIST0_DESC_ADDR_SHIFT   0
#define BCHP_PCIE_DMA_RX_FIRST_DESC_U_ADDR_LIST0_DESC_ADDR_DEFAULT 0

/***************************************************************************
 *RX_FIRST_DESC_L_ADDR_LIST1 - Rx Descriptor List1 First Descriptor Lower Address
 ***************************************************************************/
/* PCIE_DMA :: RX_FIRST_DESC_L_ADDR_LIST1 :: DESC_ADDR [31:05] */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST1_DESC_ADDR_MASK    0xffffffe0
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST1_DESC_ADDR_SHIFT   5
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST1_DESC_ADDR_DEFAULT 0

/* PCIE_DMA :: RX_FIRST_DESC_L_ADDR_LIST1 :: reserved0 [04:01] */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST1_reserved0_MASK    0x0000001e
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST1_reserved0_SHIFT   1

/* PCIE_DMA :: RX_FIRST_DESC_L_ADDR_LIST1 :: RX_DESC_LIST1_VALID [00:00] */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST1_RX_DESC_LIST1_VALID_MASK 0x00000001
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST1_RX_DESC_LIST1_VALID_SHIFT 0
#define BCHP_PCIE_DMA_RX_FIRST_DESC_L_ADDR_LIST1_RX_DESC_LIST1_VALID_DEFAULT 0

/***************************************************************************
 *RX_FIRST_DESC_U_ADDR_LIST1 - Rx Descriptor List1 First Descriptor Upper Address
 ***************************************************************************/
/* PCIE_DMA :: RX_FIRST_DESC_U_ADDR_LIST1 :: DESC_ADDR [31:00] */
#define BCHP_PCIE_DMA_RX_FIRST_DESC_U_ADDR_LIST1_DESC_ADDR_MASK    0xffffffff
#define BCHP_PCIE_DMA_RX_FIRST_DESC_U_ADDR_LIST1_DESC_ADDR_SHIFT   0
#define BCHP_PCIE_DMA_RX_FIRST_DESC_U_ADDR_LIST1_DESC_ADDR_DEFAULT 0

/***************************************************************************
 *RX_SW_DESC_LIST_CTRL_STS - Rx Software Descriptor List Control and Status
 ***************************************************************************/
/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: reserved0 [31:14] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_reserved0_MASK      0xffffc000
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_reserved0_SHIFT     14

/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: DESC_ENDIAN_MODE [13:12] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DESC_ENDIAN_MODE_MASK 0x00003000
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DESC_ENDIAN_MODE_SHIFT 12
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DESC_ENDIAN_MODE_DEFAULT 0

/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: reserved1 [11:10] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_reserved1_MASK      0x00000c00
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_reserved1_SHIFT     10

/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: LOCAL_DESC [09:09] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_LOCAL_DESC_MASK     0x00000200
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_LOCAL_DESC_SHIFT    9
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_LOCAL_DESC_DEFAULT  0

/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: DMA_MODE [08:08] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DMA_MODE_MASK       0x00000100
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DMA_MODE_SHIFT      8
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DMA_MODE_DEFAULT    0

/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: reserved2 [07:06] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_reserved2_MASK      0x000000c0
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_reserved2_SHIFT     6

/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: DMA_STATUS [05:04] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DMA_STATUS_MASK     0x00000030
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DMA_STATUS_SHIFT    4
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DMA_STATUS_DEFAULT  0

/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: DMA_DATA_SERV_PTR [03:03] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DMA_DATA_SERV_PTR_MASK 0x00000008
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DMA_DATA_SERV_PTR_SHIFT 3
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DMA_DATA_SERV_PTR_DEFAULT 0

/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: DESC_SERV_PTR [02:02] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DESC_SERV_PTR_MASK  0x00000004
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DESC_SERV_PTR_SHIFT 2
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_DESC_SERV_PTR_DEFAULT 0

/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: reserved3 [01:01] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_reserved3_MASK      0x00000002
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_reserved3_SHIFT     1

/* PCIE_DMA :: RX_SW_DESC_LIST_CTRL_STS :: RX_DMA_RUN_STOP [00:00] */
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_RX_DMA_RUN_STOP_MASK 0x00000001
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_RX_DMA_RUN_STOP_SHIFT 0
#define BCHP_PCIE_DMA_RX_SW_DESC_LIST_CTRL_STS_RX_DMA_RUN_STOP_DEFAULT 0

/***************************************************************************
 *RX_WAKE_CTRL - Rx DMA Wake Control
 ***************************************************************************/
/* PCIE_DMA :: RX_WAKE_CTRL :: reserved0 [31:02] */
#define BCHP_PCIE_DMA_RX_WAKE_CTRL_reserved0_MASK                  0xfffffffc
#define BCHP_PCIE_DMA_RX_WAKE_CTRL_reserved0_SHIFT                 2

/* PCIE_DMA :: RX_WAKE_CTRL :: WAKE_MODE [01:01] */
#define BCHP_PCIE_DMA_RX_WAKE_CTRL_WAKE_MODE_MASK                  0x00000002
#define BCHP_PCIE_DMA_RX_WAKE_CTRL_WAKE_MODE_SHIFT                 1
#define BCHP_PCIE_DMA_RX_WAKE_CTRL_WAKE_MODE_DEFAULT               0

/* PCIE_DMA :: RX_WAKE_CTRL :: WAKE [00:00] */
#define BCHP_PCIE_DMA_RX_WAKE_CTRL_WAKE_MASK                       0x00000001
#define BCHP_PCIE_DMA_RX_WAKE_CTRL_WAKE_SHIFT                      0
#define BCHP_PCIE_DMA_RX_WAKE_CTRL_WAKE_DEFAULT                    0

/***************************************************************************
 *RX_ERROR_STATUS - Rx Engine Error Status
 ***************************************************************************/
/* PCIE_DMA :: RX_ERROR_STATUS :: reserved0 [31:10] */
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_reserved0_MASK               0xfffffc00
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_reserved0_SHIFT              10

/* PCIE_DMA :: RX_ERROR_STATUS :: RX_L1_DESC_TX_ABORT_ERRORS [09:09] */
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L1_DESC_TX_ABORT_ERRORS_MASK 0x00000200
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L1_DESC_TX_ABORT_ERRORS_SHIFT 9
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L1_DESC_TX_ABORT_ERRORS_DEFAULT 0

/* PCIE_DMA :: RX_ERROR_STATUS :: reserved1 [08:08] */
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_reserved1_MASK               0x00000100
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_reserved1_SHIFT              8

/* PCIE_DMA :: RX_ERROR_STATUS :: RX_L0_DESC_TX_ABORT_ERRORS [07:07] */
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L0_DESC_TX_ABORT_ERRORS_MASK 0x00000080
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L0_DESC_TX_ABORT_ERRORS_SHIFT 7
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L0_DESC_TX_ABORT_ERRORS_DEFAULT 0

/* PCIE_DMA :: RX_ERROR_STATUS :: reserved2 [06:06] */
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_reserved2_MASK               0x00000040
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_reserved2_SHIFT              6

/* PCIE_DMA :: RX_ERROR_STATUS :: RX_L1_DATA_TX_ABORT_ERRORS [05:05] */
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L1_DATA_TX_ABORT_ERRORS_MASK 0x00000020
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L1_DATA_TX_ABORT_ERRORS_SHIFT 5
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L1_DATA_TX_ABORT_ERRORS_DEFAULT 0

/* PCIE_DMA :: RX_ERROR_STATUS :: reserved3 [04:03] */
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_reserved3_MASK               0x00000018
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_reserved3_SHIFT              3

/* PCIE_DMA :: RX_ERROR_STATUS :: RX_L0_DATA_TX_ABORT_ERRORS [02:02] */
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L0_DATA_TX_ABORT_ERRORS_MASK 0x00000004
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L0_DATA_TX_ABORT_ERRORS_SHIFT 2
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_RX_L0_DATA_TX_ABORT_ERRORS_DEFAULT 0

/* PCIE_DMA :: RX_ERROR_STATUS :: reserved4 [01:00] */
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_reserved4_MASK               0x00000003
#define BCHP_PCIE_DMA_RX_ERROR_STATUS_reserved4_SHIFT              0

/***************************************************************************
 *RX_LIST0_CUR_DESC_L_ADDR - Rx List0 Current Descriptor Lower Address
 ***************************************************************************/
/* PCIE_DMA :: RX_LIST0_CUR_DESC_L_ADDR :: RX_L0_CUR_DESC_L_ADDR [31:05] */
#define BCHP_PCIE_DMA_RX_LIST0_CUR_DESC_L_ADDR_RX_L0_CUR_DESC_L_ADDR_MASK 0xffffffe0
#define BCHP_PCIE_DMA_RX_LIST0_CUR_DESC_L_ADDR_RX_L0_CUR_DESC_L_ADDR_SHIFT 5
#define BCHP_PCIE_DMA_RX_LIST0_CUR_DESC_L_ADDR_RX_L0_CUR_DESC_L_ADDR_DEFAULT 0

/* PCIE_DMA :: RX_LIST0_CUR_DESC_L_ADDR :: reserved0 [04:00] */
#define BCHP_PCIE_DMA_RX_LIST0_CUR_DESC_L_ADDR_reserved0_MASK      0x0000001f
#define BCHP_PCIE_DMA_RX_LIST0_CUR_DESC_L_ADDR_reserved0_SHIFT     0

/***************************************************************************
 *RX_LIST0_CUR_DESC_U_ADDR - Rx List0 Current Descriptor Upper Address
 ***************************************************************************/
/* PCIE_DMA :: RX_LIST0_CUR_DESC_U_ADDR :: RX_L0_CUR_DESC_U_ADDR [31:00] */
#define BCHP_PCIE_DMA_RX_LIST0_CUR_DESC_U_ADDR_RX_L0_CUR_DESC_U_ADDR_MASK 0xffffffff
#define BCHP_PCIE_DMA_RX_LIST0_CUR_DESC_U_ADDR_RX_L0_CUR_DESC_U_ADDR_SHIFT 0
#define BCHP_PCIE_DMA_RX_LIST0_CUR_DESC_U_ADDR_RX_L0_CUR_DESC_U_ADDR_DEFAULT 0

/***************************************************************************
 *RX_LIST0_CUR_BYTE_CNT - Rx List0 Current Descriptor Byte Count
 ***************************************************************************/
/* PCIE_DMA :: RX_LIST0_CUR_BYTE_CNT :: RX_L0_CUR_BYTE_CNT [31:00] */
#define BCHP_PCIE_DMA_RX_LIST0_CUR_BYTE_CNT_RX_L0_CUR_BYTE_CNT_MASK 0xffffffff
#define BCHP_PCIE_DMA_RX_LIST0_CUR_BYTE_CNT_RX_L0_CUR_BYTE_CNT_SHIFT 0
#define BCHP_PCIE_DMA_RX_LIST0_CUR_BYTE_CNT_RX_L0_CUR_BYTE_CNT_DEFAULT 0

/***************************************************************************
 *RX_LIST1_CUR_DESC_L_ADDR - Rx List1 Current Descriptor Lower address
 ***************************************************************************/
/* PCIE_DMA :: RX_LIST1_CUR_DESC_L_ADDR :: RX_L1_CUR_DESC_L_ADDR [31:05] */
#define BCHP_PCIE_DMA_RX_LIST1_CUR_DESC_L_ADDR_RX_L1_CUR_DESC_L_ADDR_MASK 0xffffffe0
#define BCHP_PCIE_DMA_RX_LIST1_CUR_DESC_L_ADDR_RX_L1_CUR_DESC_L_ADDR_SHIFT 5
#define BCHP_PCIE_DMA_RX_LIST1_CUR_DESC_L_ADDR_RX_L1_CUR_DESC_L_ADDR_DEFAULT 0

/* PCIE_DMA :: RX_LIST1_CUR_DESC_L_ADDR :: reserved0 [04:00] */
#define BCHP_PCIE_DMA_RX_LIST1_CUR_DESC_L_ADDR_reserved0_MASK      0x0000001f
#define BCHP_PCIE_DMA_RX_LIST1_CUR_DESC_L_ADDR_reserved0_SHIFT     0

/***************************************************************************
 *RX_LIST1_CUR_DESC_U_ADDR - Rx List1 Current Descriptor Upper address
 ***************************************************************************/
/* PCIE_DMA :: RX_LIST1_CUR_DESC_U_ADDR :: RX_L1_CUR_DESC_U_ADDR [31:00] */
#define BCHP_PCIE_DMA_RX_LIST1_CUR_DESC_U_ADDR_RX_L1_CUR_DESC_U_ADDR_MASK 0xffffffff
#define BCHP_PCIE_DMA_RX_LIST1_CUR_DESC_U_ADDR_RX_L1_CUR_DESC_U_ADDR_SHIFT 0
#define BCHP_PCIE_DMA_RX_LIST1_CUR_DESC_U_ADDR_RX_L1_CUR_DESC_U_ADDR_DEFAULT 0

/***************************************************************************
 *RX_LIST1_CUR_BYTE_CNT - Rx List1 Current Descriptor Byte Count
 ***************************************************************************/
/* PCIE_DMA :: RX_LIST1_CUR_BYTE_CNT :: RX_L1_CUR_BYTE_CNT [31:00] */
#define BCHP_PCIE_DMA_RX_LIST1_CUR_BYTE_CNT_RX_L1_CUR_BYTE_CNT_MASK 0xffffffff
#define BCHP_PCIE_DMA_RX_LIST1_CUR_BYTE_CNT_RX_L1_CUR_BYTE_CNT_SHIFT 0
#define BCHP_PCIE_DMA_RX_LIST1_CUR_BYTE_CNT_RX_L1_CUR_BYTE_CNT_DEFAULT 0

/***************************************************************************
 *DMA_DEBUG_OPTIONS_REG - DMA Debug Options Register
 ***************************************************************************/
/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_TX_DMA_SOFT_RST [31:31] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_TX_DMA_SOFT_RST_MASK 0x80000000
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_TX_DMA_SOFT_RST_SHIFT 31
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_TX_DMA_SOFT_RST_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_RX_DMA_SOFT_RST [30:30] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_RX_DMA_SOFT_RST_MASK 0x40000000
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_RX_DMA_SOFT_RST_SHIFT 30
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_RX_DMA_SOFT_RST_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_TX_DMA_RD_Q_SOFT_RST [29:29] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_TX_DMA_RD_Q_SOFT_RST_MASK 0x20000000
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_TX_DMA_RD_Q_SOFT_RST_SHIFT 29
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_TX_DMA_RD_Q_SOFT_RST_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_RX_DMA_WR_Q_SOFT_RST [28:28] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_RX_DMA_WR_Q_SOFT_RST_MASK 0x10000000
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_RX_DMA_WR_Q_SOFT_RST_SHIFT 28
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_RX_DMA_WR_Q_SOFT_RST_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: RSVD_DMA_DEBUG_0 [27:10] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_RSVD_DMA_DEBUG_0_MASK  0x0ffffc00
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_RSVD_DMA_DEBUG_0_SHIFT 10
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_RSVD_DMA_DEBUG_0_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_SPQ_SNAP_PW_DIS [09:09] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SPQ_SNAP_PW_DIS_MASK 0x00000200
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SPQ_SNAP_PW_DIS_SHIFT 9
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SPQ_SNAP_PW_DIS_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_OUTCP_Q_RO [08:08] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_OUTCP_Q_RO_MASK 0x00000100
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_OUTCP_Q_RO_SHIFT 8
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_OUTCP_Q_RO_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_NO_TX_DESC [07:07] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_NO_TX_DESC_MASK 0x00000080
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_NO_TX_DESC_SHIFT 7
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_NO_TX_DESC_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_NO_RX_DESC [06:06] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_NO_RX_DESC_MASK 0x00000040
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_NO_RX_DESC_SHIFT 6
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_NO_RX_DESC_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_SEL_RX_CNT [05:05] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SEL_RX_CNT_MASK 0x00000020
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SEL_RX_CNT_SHIFT 5
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SEL_RX_CNT_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_EN_RX_DMA_XFER_CNT [04:04] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_EN_RX_DMA_XFER_CNT_MASK 0x00000010
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_EN_RX_DMA_XFER_CNT_SHIFT 4
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_EN_RX_DMA_XFER_CNT_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_SEL_TX_CNT [03:03] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SEL_TX_CNT_MASK 0x00000008
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SEL_TX_CNT_SHIFT 3
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SEL_TX_CNT_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_SINGLE_RD_Q [02:02] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SINGLE_RD_Q_MASK 0x00000004
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SINGLE_RD_Q_SHIFT 2
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SINGLE_RD_Q_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_SINGLE_WR_Q [01:01] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SINGLE_WR_Q_MASK 0x00000002
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SINGLE_WR_Q_SHIFT 1
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_SINGLE_WR_Q_DEFAULT 0

/* PCIE_DMA :: DMA_DEBUG_OPTIONS_REG :: DMA_DEBUG_RD_Q_RO [00:00] */
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_RD_Q_RO_MASK 0x00000001
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_RD_Q_RO_SHIFT 0
#define BCHP_PCIE_DMA_DMA_DEBUG_OPTIONS_REG_DMA_DEBUG_RD_Q_RO_DEFAULT 0

/***************************************************************************
 *READ_CHANNEL_ERROR_STATUS - Read Channel Error Status
 ***************************************************************************/
/* PCIE_DMA :: READ_CHANNEL_ERROR_STATUS :: TX_ERR_STS_CHAN_7 [31:28] */
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_7_MASK 0xf0000000
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_7_SHIFT 28
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_7_DEFAULT 0

/* PCIE_DMA :: READ_CHANNEL_ERROR_STATUS :: TX_ERR_STS_CHAN_6 [27:24] */
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_6_MASK 0x0f000000
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_6_SHIFT 24
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_6_DEFAULT 0

/* PCIE_DMA :: READ_CHANNEL_ERROR_STATUS :: TX_ERR_STS_CHAN_5 [23:20] */
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_5_MASK 0x00f00000
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_5_SHIFT 20
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_5_DEFAULT 0

/* PCIE_DMA :: READ_CHANNEL_ERROR_STATUS :: TX_ERR_STS_CHAN_4 [19:16] */
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_4_MASK 0x000f0000
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_4_SHIFT 16
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_4_DEFAULT 0

/* PCIE_DMA :: READ_CHANNEL_ERROR_STATUS :: TX_ERR_STS_CHAN_3 [15:12] */
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_3_MASK 0x0000f000
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_3_SHIFT 12
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_3_DEFAULT 0

/* PCIE_DMA :: READ_CHANNEL_ERROR_STATUS :: TX_ERR_STS_CHAN_2 [11:08] */
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_2_MASK 0x00000f00
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_2_SHIFT 8
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_2_DEFAULT 0

/* PCIE_DMA :: READ_CHANNEL_ERROR_STATUS :: TX_ERR_STS_CHAN_1 [07:04] */
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_1_MASK 0x000000f0
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_1_SHIFT 4
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_1_DEFAULT 0

/* PCIE_DMA :: READ_CHANNEL_ERROR_STATUS :: TX_ERR_STS_CHAN_0 [03:00] */
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_0_MASK 0x0000000f
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_0_SHIFT 0
#define BCHP_PCIE_DMA_READ_CHANNEL_ERROR_STATUS_TX_ERR_STS_CHAN_0_DEFAULT 0

#endif /* #ifndef BCHP_PCIE_DMA_H__ */

/* End of File */
