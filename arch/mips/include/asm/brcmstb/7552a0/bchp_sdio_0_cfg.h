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
 * Date:           Generated on         Mon May 16 21:06:56 2011
 *                 MD5 Checksum         f5f09b2bf7ad40890d2e5dc57d4789b6
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7552/rdb/a0/bchp_sdio_0_cfg.h $
 * 
 * Hydra_Software_Devel/2   5/18/11 4:30p xhuang
 * SW7552-2: update with central RDB
 *
 ***************************************************************************/

#ifndef BCHP_SDIO_0_CFG_H__
#define BCHP_SDIO_0_CFG_H__

/***************************************************************************
 *SDIO_0_CFG - SDIO_0 (CARD) Configuration Registers
 ***************************************************************************/
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1          0x00410100 /* SDIO EMMC Control Register */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2          0x00410104 /* SDIO EMMC Control Register */
#define BCHP_SDIO_0_CFG_TP_OUT_SEL               0x00410108 /* SDIO TP_OUT Control Register */
#define BCHP_SDIO_0_CFG_CAP_REG0                 0x0041010c /* SDIO CAPABILITIES override Register */
#define BCHP_SDIO_0_CFG_CAP_REG1                 0x00410110 /* SDIO CAPABILITIES override Register */
#define BCHP_SDIO_0_CFG_PRESET1                  0x00410114 /* SDIO CAPABILITIES override Register */
#define BCHP_SDIO_0_CFG_PRESET2                  0x00410118 /* SDIO CAPABILITIES override Register */
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY           0x0041011c /* SDIO Clock delay register */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV               0x00410120 /* SDIO Clock delay register */
#define BCHP_SDIO_0_CFG_IP_DLY                   0x00410130 /* SDIO Host input delay register */
#define BCHP_SDIO_0_CFG_OP_DLY                   0x00410134 /* SDIO Host output delay register */
#define BCHP_SDIO_0_CFG_SCRATCH                  0x004101fc /* SDIO Scratch Register */
#define BCHP_SDIO_0_CFG_VERSION                  0x004101f0 /* SDIO VERSION Register */

/***************************************************************************
 *SDIO_EMMC_CTRL1 - SDIO EMMC Control Register
 ***************************************************************************/
/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: SDCD_N_TEST_SEL_EN [31:31] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SDCD_N_TEST_SEL_EN_MASK    0x80000000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SDCD_N_TEST_SEL_EN_SHIFT   31
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SDCD_N_TEST_SEL_EN_DEFAULT 0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: SDCD_N_TEST_LEV [30:30] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SDCD_N_TEST_LEV_MASK       0x40000000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SDCD_N_TEST_LEV_SHIFT      30
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SDCD_N_TEST_LEV_DEFAULT    1

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: CFG_RESERVED [29:29] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_CFG_RESERVED_MASK          0x20000000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_CFG_RESERVED_SHIFT         29
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_CFG_RESERVED_DEFAULT       0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: RETUNING_REQ [28:28] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_RETUNING_REQ_MASK          0x10000000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_RETUNING_REQ_SHIFT         28
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_RETUNING_REQ_DEFAULT       0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: DDR_TAP_DELAY [27:24] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_DDR_TAP_DELAY_MASK         0x0f000000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_DDR_TAP_DELAY_SHIFT        24

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: DELAY_CTRL [23:21] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_DELAY_CTRL_MASK            0x00e00000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_DELAY_CTRL_SHIFT           21

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: HREADY_IDLE_ENA [20:20] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_HREADY_IDLE_ENA_MASK       0x00100000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_HREADY_IDLE_ENA_SHIFT      20
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_HREADY_IDLE_ENA_DEFAULT    1

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: HREADY_IDLE_PULSE [19:19] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_HREADY_IDLE_PULSE_MASK     0x00080000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_HREADY_IDLE_PULSE_SHIFT    19
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_HREADY_IDLE_PULSE_DEFAULT  0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: DATA_PENDING [18:18] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_DATA_PENDING_MASK          0x00040000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_DATA_PENDING_SHIFT         18
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_DATA_PENDING_DEFAULT       0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: WR_FLUSH [17:17] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_WR_FLUSH_MASK              0x00020000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_WR_FLUSH_SHIFT             17
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_WR_FLUSH_DEFAULT           0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: MF_NUM_WR [16:16] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_MF_NUM_WR_MASK             0x00010000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_MF_NUM_WR_SHIFT            16
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_MF_NUM_WR_DEFAULT          0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: WORD_ABO [15:15] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_WORD_ABO_MASK              0x00008000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_WORD_ABO_SHIFT             15
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_WORD_ABO_DEFAULT           0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: FRAME_NBO [14:14] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_FRAME_NBO_MASK             0x00004000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_FRAME_NBO_SHIFT            14
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_FRAME_NBO_DEFAULT          0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: FRAME_NHW [13:13] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_FRAME_NHW_MASK             0x00002000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_FRAME_NHW_SHIFT            13
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_FRAME_NHW_DEFAULT          1

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: BUFFER_ABO [12:12] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_BUFFER_ABO_MASK            0x00001000
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_BUFFER_ABO_SHIFT           12
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_BUFFER_ABO_DEFAULT         1

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: SCB_BUF_ACC [11:11] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_BUF_ACC_MASK           0x00000800
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_BUF_ACC_SHIFT          11
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_BUF_ACC_DEFAULT        1

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: SCB_SEQ_EN [10:10] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_SEQ_EN_MASK            0x00000400
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_SEQ_EN_SHIFT           10
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_SEQ_EN_DEFAULT         0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: SCB_RD_THRESH [09:05] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_RD_THRESH_MASK         0x000003e0
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_RD_THRESH_SHIFT        5
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_RD_THRESH_DEFAULT      2

/* SDIO_0_CFG :: SDIO_EMMC_CTRL1 :: SCB_SIZE [04:00] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_SIZE_MASK              0x0000001f
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_SIZE_SHIFT             0
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL1_SCB_SIZE_DEFAULT           4

/***************************************************************************
 *SDIO_EMMC_CTRL2 - SDIO EMMC Control Register
 ***************************************************************************/
/* SDIO_0_CFG :: SDIO_EMMC_CTRL2 :: reserved0 [31:08] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_reserved0_MASK             0xffffff00
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_reserved0_SHIFT            8

/* SDIO_0_CFG :: SDIO_EMMC_CTRL2 :: REG_ADDR_MAP_BYTE [07:06] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_ADDR_MAP_BYTE_MASK     0x000000c0
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_ADDR_MAP_BYTE_SHIFT    6
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_ADDR_MAP_BYTE_DEFAULT  0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL2 :: reserved1 [05:05] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_reserved1_MASK             0x00000020
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_reserved1_SHIFT            5

/* SDIO_0_CFG :: SDIO_EMMC_CTRL2 :: REG_ADDR_MAP_HW [04:04] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_ADDR_MAP_HW_MASK       0x00000010
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_ADDR_MAP_HW_SHIFT      4
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_ADDR_MAP_HW_DEFAULT    0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL2 :: REG_DATA_SWAP_RD [03:02] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_DATA_SWAP_RD_MASK      0x0000000c
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_DATA_SWAP_RD_SHIFT     2
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_DATA_SWAP_RD_DEFAULT   0

/* SDIO_0_CFG :: SDIO_EMMC_CTRL2 :: REG_DATA_SWAP_WR [01:00] */
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_DATA_SWAP_WR_MASK      0x00000003
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_DATA_SWAP_WR_SHIFT     0
#define BCHP_SDIO_0_CFG_SDIO_EMMC_CTRL2_REG_DATA_SWAP_WR_DEFAULT   0

/***************************************************************************
 *TP_OUT_SEL - SDIO TP_OUT Control Register
 ***************************************************************************/
/* SDIO_0_CFG :: TP_OUT_SEL :: reserved0 [31:01] */
#define BCHP_SDIO_0_CFG_TP_OUT_SEL_reserved0_MASK                  0xfffffffe
#define BCHP_SDIO_0_CFG_TP_OUT_SEL_reserved0_SHIFT                 1

/* SDIO_0_CFG :: TP_OUT_SEL :: TP_OUT_SELECT [00:00] */
#define BCHP_SDIO_0_CFG_TP_OUT_SEL_TP_OUT_SELECT_MASK              0x00000001
#define BCHP_SDIO_0_CFG_TP_OUT_SEL_TP_OUT_SELECT_SHIFT             0
#define BCHP_SDIO_0_CFG_TP_OUT_SEL_TP_OUT_SELECT_DEFAULT           0

/***************************************************************************
 *CAP_REG0 - SDIO CAPABILITIES override Register
 ***************************************************************************/
/* SDIO_0_CFG :: CAP_REG0 :: DDR50_SUPPORT [31:31] */
#define BCHP_SDIO_0_CFG_CAP_REG0_DDR50_SUPPORT_MASK                0x80000000
#define BCHP_SDIO_0_CFG_CAP_REG0_DDR50_SUPPORT_SHIFT               31
#define BCHP_SDIO_0_CFG_CAP_REG0_DDR50_SUPPORT_DEFAULT             1

/* SDIO_0_CFG :: CAP_REG0 :: SD104_SUPPORT [30:30] */
#define BCHP_SDIO_0_CFG_CAP_REG0_SD104_SUPPORT_MASK                0x40000000
#define BCHP_SDIO_0_CFG_CAP_REG0_SD104_SUPPORT_SHIFT               30
#define BCHP_SDIO_0_CFG_CAP_REG0_SD104_SUPPORT_DEFAULT             0

/* SDIO_0_CFG :: CAP_REG0 :: SDR50 [29:29] */
#define BCHP_SDIO_0_CFG_CAP_REG0_SDR50_MASK                        0x20000000
#define BCHP_SDIO_0_CFG_CAP_REG0_SDR50_SHIFT                       29
#define BCHP_SDIO_0_CFG_CAP_REG0_SDR50_DEFAULT                     1

/* SDIO_0_CFG :: CAP_REG0 :: SLOT_TYPE [28:27] */
#define BCHP_SDIO_0_CFG_CAP_REG0_SLOT_TYPE_MASK                    0x18000000
#define BCHP_SDIO_0_CFG_CAP_REG0_SLOT_TYPE_SHIFT                   27
#define BCHP_SDIO_0_CFG_CAP_REG0_SLOT_TYPE_DEFAULT                 2

/* SDIO_0_CFG :: CAP_REG0 :: ASYNCH_INT_SUPPORT [26:26] */
#define BCHP_SDIO_0_CFG_CAP_REG0_ASYNCH_INT_SUPPORT_MASK           0x04000000
#define BCHP_SDIO_0_CFG_CAP_REG0_ASYNCH_INT_SUPPORT_SHIFT          26
#define BCHP_SDIO_0_CFG_CAP_REG0_ASYNCH_INT_SUPPORT_DEFAULT        0

/* SDIO_0_CFG :: CAP_REG0 :: 64B_SYS_BUS_SUPPORT [25:25] */
#define BCHP_SDIO_0_CFG_CAP_REG0_64B_SYS_BUS_SUPPORT_MASK          0x02000000
#define BCHP_SDIO_0_CFG_CAP_REG0_64B_SYS_BUS_SUPPORT_SHIFT         25
#define BCHP_SDIO_0_CFG_CAP_REG0_64B_SYS_BUS_SUPPORT_DEFAULT       0

/* SDIO_0_CFG :: CAP_REG0 :: 1_8V_SUPPORT [24:24] */
#define BCHP_SDIO_0_CFG_CAP_REG0_1_8V_SUPPORT_MASK                 0x01000000
#define BCHP_SDIO_0_CFG_CAP_REG0_1_8V_SUPPORT_SHIFT                24
#define BCHP_SDIO_0_CFG_CAP_REG0_1_8V_SUPPORT_DEFAULT              1

/* SDIO_0_CFG :: CAP_REG0 :: 3_0V_SUPPORT [23:23] */
#define BCHP_SDIO_0_CFG_CAP_REG0_3_0V_SUPPORT_MASK                 0x00800000
#define BCHP_SDIO_0_CFG_CAP_REG0_3_0V_SUPPORT_SHIFT                23
#define BCHP_SDIO_0_CFG_CAP_REG0_3_0V_SUPPORT_DEFAULT              0

/* SDIO_0_CFG :: CAP_REG0 :: 3_3V_SUPPORT [22:22] */
#define BCHP_SDIO_0_CFG_CAP_REG0_3_3V_SUPPORT_MASK                 0x00400000
#define BCHP_SDIO_0_CFG_CAP_REG0_3_3V_SUPPORT_SHIFT                22
#define BCHP_SDIO_0_CFG_CAP_REG0_3_3V_SUPPORT_DEFAULT              1

/* SDIO_0_CFG :: CAP_REG0 :: SUSP_RES_SUPPORT [21:21] */
#define BCHP_SDIO_0_CFG_CAP_REG0_SUSP_RES_SUPPORT_MASK             0x00200000
#define BCHP_SDIO_0_CFG_CAP_REG0_SUSP_RES_SUPPORT_SHIFT            21
#define BCHP_SDIO_0_CFG_CAP_REG0_SUSP_RES_SUPPORT_DEFAULT          1

/* SDIO_0_CFG :: CAP_REG0 :: SDMA_SUPPORT [20:20] */
#define BCHP_SDIO_0_CFG_CAP_REG0_SDMA_SUPPORT_MASK                 0x00100000
#define BCHP_SDIO_0_CFG_CAP_REG0_SDMA_SUPPORT_SHIFT                20
#define BCHP_SDIO_0_CFG_CAP_REG0_SDMA_SUPPORT_DEFAULT              1

/* SDIO_0_CFG :: CAP_REG0 :: HIGH_SPEED_SUPPORT [19:19] */
#define BCHP_SDIO_0_CFG_CAP_REG0_HIGH_SPEED_SUPPORT_MASK           0x00080000
#define BCHP_SDIO_0_CFG_CAP_REG0_HIGH_SPEED_SUPPORT_SHIFT          19
#define BCHP_SDIO_0_CFG_CAP_REG0_HIGH_SPEED_SUPPORT_DEFAULT        1

/* SDIO_0_CFG :: CAP_REG0 :: ADMA2_SUPPORT [18:18] */
#define BCHP_SDIO_0_CFG_CAP_REG0_ADMA2_SUPPORT_MASK                0x00040000
#define BCHP_SDIO_0_CFG_CAP_REG0_ADMA2_SUPPORT_SHIFT               18
#define BCHP_SDIO_0_CFG_CAP_REG0_ADMA2_SUPPORT_DEFAULT             1

/* SDIO_0_CFG :: CAP_REG0 :: EXTENDED_MEDIA_SUPPORT [17:17] */
#define BCHP_SDIO_0_CFG_CAP_REG0_EXTENDED_MEDIA_SUPPORT_MASK       0x00020000
#define BCHP_SDIO_0_CFG_CAP_REG0_EXTENDED_MEDIA_SUPPORT_SHIFT      17
#define BCHP_SDIO_0_CFG_CAP_REG0_EXTENDED_MEDIA_SUPPORT_DEFAULT    1

/* SDIO_0_CFG :: CAP_REG0 :: MAX_BL [16:15] */
#define BCHP_SDIO_0_CFG_CAP_REG0_MAX_BL_MASK                       0x00018000
#define BCHP_SDIO_0_CFG_CAP_REG0_MAX_BL_SHIFT                      15
#define BCHP_SDIO_0_CFG_CAP_REG0_MAX_BL_DEFAULT                    1

/* SDIO_0_CFG :: CAP_REG0 :: BASE_FREQ [14:07] */
#define BCHP_SDIO_0_CFG_CAP_REG0_BASE_FREQ_MASK                    0x00007f80
#define BCHP_SDIO_0_CFG_CAP_REG0_BASE_FREQ_SHIFT                   7
#define BCHP_SDIO_0_CFG_CAP_REG0_BASE_FREQ_DEFAULT                 100

/* SDIO_0_CFG :: CAP_REG0 :: TIMEOUT_CLK_UNIT [06:06] */
#define BCHP_SDIO_0_CFG_CAP_REG0_TIMEOUT_CLK_UNIT_MASK             0x00000040
#define BCHP_SDIO_0_CFG_CAP_REG0_TIMEOUT_CLK_UNIT_SHIFT            6
#define BCHP_SDIO_0_CFG_CAP_REG0_TIMEOUT_CLK_UNIT_DEFAULT          1

/* SDIO_0_CFG :: CAP_REG0 :: TIMEOUT_FREQ [05:00] */
#define BCHP_SDIO_0_CFG_CAP_REG0_TIMEOUT_FREQ_MASK                 0x0000003f
#define BCHP_SDIO_0_CFG_CAP_REG0_TIMEOUT_FREQ_SHIFT                0
#define BCHP_SDIO_0_CFG_CAP_REG0_TIMEOUT_FREQ_DEFAULT              50

/***************************************************************************
 *CAP_REG1 - SDIO CAPABILITIES override Register
 ***************************************************************************/
/* SDIO_0_CFG :: CAP_REG1 :: CAP_REG_OVERRIDE [31:31] */
#define BCHP_SDIO_0_CFG_CAP_REG1_CAP_REG_OVERRIDE_MASK             0x80000000
#define BCHP_SDIO_0_CFG_CAP_REG1_CAP_REG_OVERRIDE_SHIFT            31
#define BCHP_SDIO_0_CFG_CAP_REG1_CAP_REG_OVERRIDE_DEFAULT          0

/* SDIO_0_CFG :: CAP_REG1 :: reserved0 [30:21] */
#define BCHP_SDIO_0_CFG_CAP_REG1_reserved0_MASK                    0x7fe00000
#define BCHP_SDIO_0_CFG_CAP_REG1_reserved0_SHIFT                   21

/* SDIO_0_CFG :: CAP_REG1 :: CAP_1_stuff [20:20] */
#define BCHP_SDIO_0_CFG_CAP_REG1_CAP_1_stuff_MASK                  0x00100000
#define BCHP_SDIO_0_CFG_CAP_REG1_CAP_1_stuff_SHIFT                 20
#define BCHP_SDIO_0_CFG_CAP_REG1_CAP_1_stuff_DEFAULT               0

/* SDIO_0_CFG :: CAP_REG1 :: SPI_BLK_MODE [19:19] */
#define BCHP_SDIO_0_CFG_CAP_REG1_SPI_BLK_MODE_MASK                 0x00080000
#define BCHP_SDIO_0_CFG_CAP_REG1_SPI_BLK_MODE_SHIFT                19
#define BCHP_SDIO_0_CFG_CAP_REG1_SPI_BLK_MODE_DEFAULT              0

/* SDIO_0_CFG :: CAP_REG1 :: SPI_MODE [18:18] */
#define BCHP_SDIO_0_CFG_CAP_REG1_SPI_MODE_MASK                     0x00040000
#define BCHP_SDIO_0_CFG_CAP_REG1_SPI_MODE_SHIFT                    18
#define BCHP_SDIO_0_CFG_CAP_REG1_SPI_MODE_DEFAULT                  1

/* SDIO_0_CFG :: CAP_REG1 :: CLK_MULT [17:10] */
#define BCHP_SDIO_0_CFG_CAP_REG1_CLK_MULT_MASK                     0x0003fc00
#define BCHP_SDIO_0_CFG_CAP_REG1_CLK_MULT_SHIFT                    10
#define BCHP_SDIO_0_CFG_CAP_REG1_CLK_MULT_DEFAULT                  0

/* SDIO_0_CFG :: CAP_REG1 :: RETUNING_MODES [09:08] */
#define BCHP_SDIO_0_CFG_CAP_REG1_RETUNING_MODES_MASK               0x00000300
#define BCHP_SDIO_0_CFG_CAP_REG1_RETUNING_MODES_SHIFT              8
#define BCHP_SDIO_0_CFG_CAP_REG1_RETUNING_MODES_DEFAULT            2

/* SDIO_0_CFG :: CAP_REG1 :: USE_TUNING [07:07] */
#define BCHP_SDIO_0_CFG_CAP_REG1_USE_TUNING_MASK                   0x00000080
#define BCHP_SDIO_0_CFG_CAP_REG1_USE_TUNING_SHIFT                  7
#define BCHP_SDIO_0_CFG_CAP_REG1_USE_TUNING_DEFAULT                1

/* SDIO_0_CFG :: CAP_REG1 :: RETUNING_TIMER [06:03] */
#define BCHP_SDIO_0_CFG_CAP_REG1_RETUNING_TIMER_MASK               0x00000078
#define BCHP_SDIO_0_CFG_CAP_REG1_RETUNING_TIMER_SHIFT              3
#define BCHP_SDIO_0_CFG_CAP_REG1_RETUNING_TIMER_DEFAULT            10

/* SDIO_0_CFG :: CAP_REG1 :: Driver_D_SUPPORT [02:02] */
#define BCHP_SDIO_0_CFG_CAP_REG1_Driver_D_SUPPORT_MASK             0x00000004
#define BCHP_SDIO_0_CFG_CAP_REG1_Driver_D_SUPPORT_SHIFT            2
#define BCHP_SDIO_0_CFG_CAP_REG1_Driver_D_SUPPORT_DEFAULT          0

/* SDIO_0_CFG :: CAP_REG1 :: Driver_C_SUPPORT [01:01] */
#define BCHP_SDIO_0_CFG_CAP_REG1_Driver_C_SUPPORT_MASK             0x00000002
#define BCHP_SDIO_0_CFG_CAP_REG1_Driver_C_SUPPORT_SHIFT            1
#define BCHP_SDIO_0_CFG_CAP_REG1_Driver_C_SUPPORT_DEFAULT          0

/* SDIO_0_CFG :: CAP_REG1 :: Driver_A_SUPPORT [00:00] */
#define BCHP_SDIO_0_CFG_CAP_REG1_Driver_A_SUPPORT_MASK             0x00000001
#define BCHP_SDIO_0_CFG_CAP_REG1_Driver_A_SUPPORT_SHIFT            0
#define BCHP_SDIO_0_CFG_CAP_REG1_Driver_A_SUPPORT_DEFAULT          0

/***************************************************************************
 *PRESET1 - SDIO CAPABILITIES override Register
 ***************************************************************************/
/* SDIO_0_CFG :: PRESET1 :: PRESET1_OVERRIDE [31:31] */
#define BCHP_SDIO_0_CFG_PRESET1_PRESET1_OVERRIDE_MASK              0x80000000
#define BCHP_SDIO_0_CFG_PRESET1_PRESET1_OVERRIDE_SHIFT             31
#define BCHP_SDIO_0_CFG_PRESET1_PRESET1_OVERRIDE_DEFAULT           0

/* SDIO_0_CFG :: PRESET1 :: reserved0 [30:29] */
#define BCHP_SDIO_0_CFG_PRESET1_reserved0_MASK                     0x60000000
#define BCHP_SDIO_0_CFG_PRESET1_reserved0_SHIFT                    29

/* SDIO_0_CFG :: PRESET1 :: PRESET100 [28:16] */
#define BCHP_SDIO_0_CFG_PRESET1_PRESET100_MASK                     0x1fff0000
#define BCHP_SDIO_0_CFG_PRESET1_PRESET100_SHIFT                    16
#define BCHP_SDIO_0_CFG_PRESET1_PRESET100_DEFAULT                  0

/* SDIO_0_CFG :: PRESET1 :: reserved1 [15:13] */
#define BCHP_SDIO_0_CFG_PRESET1_reserved1_MASK                     0x0000e000
#define BCHP_SDIO_0_CFG_PRESET1_reserved1_SHIFT                    13

/* SDIO_0_CFG :: PRESET1 :: PRESET50 [12:00] */
#define BCHP_SDIO_0_CFG_PRESET1_PRESET50_MASK                      0x00001fff
#define BCHP_SDIO_0_CFG_PRESET1_PRESET50_SHIFT                     0
#define BCHP_SDIO_0_CFG_PRESET1_PRESET50_DEFAULT                   1

/***************************************************************************
 *PRESET2 - SDIO CAPABILITIES override Register
 ***************************************************************************/
/* SDIO_0_CFG :: PRESET2 :: PRESET2_OVERRIDE [31:31] */
#define BCHP_SDIO_0_CFG_PRESET2_PRESET2_OVERRIDE_MASK              0x80000000
#define BCHP_SDIO_0_CFG_PRESET2_PRESET2_OVERRIDE_SHIFT             31
#define BCHP_SDIO_0_CFG_PRESET2_PRESET2_OVERRIDE_DEFAULT           0

/* SDIO_0_CFG :: PRESET2 :: reserved0 [30:29] */
#define BCHP_SDIO_0_CFG_PRESET2_reserved0_MASK                     0x60000000
#define BCHP_SDIO_0_CFG_PRESET2_reserved0_SHIFT                    29

/* SDIO_0_CFG :: PRESET2 :: PRESET25 [28:16] */
#define BCHP_SDIO_0_CFG_PRESET2_PRESET25_MASK                      0x1fff0000
#define BCHP_SDIO_0_CFG_PRESET2_PRESET25_SHIFT                     16
#define BCHP_SDIO_0_CFG_PRESET2_PRESET25_DEFAULT                   2

/* SDIO_0_CFG :: PRESET2 :: reserved1 [15:13] */
#define BCHP_SDIO_0_CFG_PRESET2_reserved1_MASK                     0x0000e000
#define BCHP_SDIO_0_CFG_PRESET2_reserved1_SHIFT                    13

/* SDIO_0_CFG :: PRESET2 :: PRESET12P5 [12:00] */
#define BCHP_SDIO_0_CFG_PRESET2_PRESET12P5_MASK                    0x00001fff
#define BCHP_SDIO_0_CFG_PRESET2_PRESET12P5_SHIFT                   0
#define BCHP_SDIO_0_CFG_PRESET2_PRESET12P5_DEFAULT                 3

/***************************************************************************
 *SD_CLOCK_DELAY - SDIO Clock delay register
 ***************************************************************************/
/* SDIO_0_CFG :: SD_CLOCK_DELAY :: INPUT_CLOCK_SEL [31:31] */
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INPUT_CLOCK_SEL_MASK        0x80000000
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INPUT_CLOCK_SEL_SHIFT       31
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INPUT_CLOCK_SEL_DEFAULT     1

/* SDIO_0_CFG :: SD_CLOCK_DELAY :: INPUT_CLOCK_DELAY_OVERRIDE [30:30] */
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INPUT_CLOCK_DELAY_OVERRIDE_MASK 0x40000000
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INPUT_CLOCK_DELAY_OVERRIDE_SHIFT 30
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INPUT_CLOCK_DELAY_OVERRIDE_DEFAULT 0

/* SDIO_0_CFG :: SD_CLOCK_DELAY :: reserved0 [29:12] */
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_reserved0_MASK              0x3ffff000
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_reserved0_SHIFT             12

/* SDIO_0_CFG :: SD_CLOCK_DELAY :: OUTPUT_CLOCK_DELAY [11:08] */
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_OUTPUT_CLOCK_DELAY_MASK     0x00000f00
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_OUTPUT_CLOCK_DELAY_SHIFT    8
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_OUTPUT_CLOCK_DELAY_DEFAULT  0

/* SDIO_0_CFG :: SD_CLOCK_DELAY :: INTERNAL_CLOCK_DELAY [07:04] */
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INTERNAL_CLOCK_DELAY_MASK   0x000000f0
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INTERNAL_CLOCK_DELAY_SHIFT  4
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INTERNAL_CLOCK_DELAY_DEFAULT 0

/* SDIO_0_CFG :: SD_CLOCK_DELAY :: INPUT_CLOCK_DELAY [03:00] */
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INPUT_CLOCK_DELAY_MASK      0x0000000f
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INPUT_CLOCK_DELAY_SHIFT     0
#define BCHP_SDIO_0_CFG_SD_CLOCK_DELAY_INPUT_CLOCK_DELAY_DEFAULT   0

/***************************************************************************
 *SD_PAD_DRV - SDIO Clock delay register
 ***************************************************************************/
/* SDIO_0_CFG :: SD_PAD_DRV :: OVERRIDE_EN [31:31] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_OVERRIDE_EN_MASK                0x80000000
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_OVERRIDE_EN_SHIFT               31
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_OVERRIDE_EN_DEFAULT             0

/* SDIO_0_CFG :: SD_PAD_DRV :: reserved0 [30:23] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved0_MASK                  0x7f800000
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved0_SHIFT                 23

/* SDIO_0_CFG :: SD_PAD_DRV :: CLK_VAL [22:20] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_CLK_VAL_MASK                    0x00700000
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_CLK_VAL_SHIFT                   20
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_CLK_VAL_DEFAULT                 5

/* SDIO_0_CFG :: SD_PAD_DRV :: reserved1 [19:19] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved1_MASK                  0x00080000
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved1_SHIFT                 19

/* SDIO_0_CFG :: SD_PAD_DRV :: CMD_VAL [18:16] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_CMD_VAL_MASK                    0x00070000
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_CMD_VAL_SHIFT                   16
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_CMD_VAL_DEFAULT                 5

/* SDIO_0_CFG :: SD_PAD_DRV :: reserved2 [15:15] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved2_MASK                  0x00008000
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved2_SHIFT                 15

/* SDIO_0_CFG :: SD_PAD_DRV :: DAT3_VAL [14:12] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT3_VAL_MASK                   0x00007000
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT3_VAL_SHIFT                  12
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT3_VAL_DEFAULT                5

/* SDIO_0_CFG :: SD_PAD_DRV :: reserved3 [11:11] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved3_MASK                  0x00000800
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved3_SHIFT                 11

/* SDIO_0_CFG :: SD_PAD_DRV :: DAT2_VAL [10:08] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT2_VAL_MASK                   0x00000700
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT2_VAL_SHIFT                  8
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT2_VAL_DEFAULT                5

/* SDIO_0_CFG :: SD_PAD_DRV :: reserved4 [07:07] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved4_MASK                  0x00000080
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved4_SHIFT                 7

/* SDIO_0_CFG :: SD_PAD_DRV :: DAT1_VAL [06:04] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT1_VAL_MASK                   0x00000070
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT1_VAL_SHIFT                  4
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT1_VAL_DEFAULT                5

/* SDIO_0_CFG :: SD_PAD_DRV :: reserved5 [03:03] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved5_MASK                  0x00000008
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_reserved5_SHIFT                 3

/* SDIO_0_CFG :: SD_PAD_DRV :: DAT0_VAL [02:00] */
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT0_VAL_MASK                   0x00000007
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT0_VAL_SHIFT                  0
#define BCHP_SDIO_0_CFG_SD_PAD_DRV_DAT0_VAL_DEFAULT                5

/***************************************************************************
 *IP_DLY - SDIO Host input delay register
 ***************************************************************************/
/* SDIO_0_CFG :: IP_DLY :: IP_TAP_EN [31:31] */
#define BCHP_SDIO_0_CFG_IP_DLY_IP_TAP_EN_MASK                      0x80000000
#define BCHP_SDIO_0_CFG_IP_DLY_IP_TAP_EN_SHIFT                     31
#define BCHP_SDIO_0_CFG_IP_DLY_IP_TAP_EN_DEFAULT                   1

/* SDIO_0_CFG :: IP_DLY :: reserved0 [30:18] */
#define BCHP_SDIO_0_CFG_IP_DLY_reserved0_MASK                      0x7ffc0000
#define BCHP_SDIO_0_CFG_IP_DLY_reserved0_SHIFT                     18

/* SDIO_0_CFG :: IP_DLY :: IP_DELAY_CTRL [17:16] */
#define BCHP_SDIO_0_CFG_IP_DLY_IP_DELAY_CTRL_MASK                  0x00030000
#define BCHP_SDIO_0_CFG_IP_DLY_IP_DELAY_CTRL_SHIFT                 16
#define BCHP_SDIO_0_CFG_IP_DLY_IP_DELAY_CTRL_DEFAULT               3

/* SDIO_0_CFG :: IP_DLY :: reserved1 [15:06] */
#define BCHP_SDIO_0_CFG_IP_DLY_reserved1_MASK                      0x0000ffc0
#define BCHP_SDIO_0_CFG_IP_DLY_reserved1_SHIFT                     6

/* SDIO_0_CFG :: IP_DLY :: IP_TAP_DELAY [05:00] */
#define BCHP_SDIO_0_CFG_IP_DLY_IP_TAP_DELAY_MASK                   0x0000003f
#define BCHP_SDIO_0_CFG_IP_DLY_IP_TAP_DELAY_SHIFT                  0
#define BCHP_SDIO_0_CFG_IP_DLY_IP_TAP_DELAY_DEFAULT                40

/***************************************************************************
 *OP_DLY - SDIO Host output delay register
 ***************************************************************************/
/* SDIO_0_CFG :: OP_DLY :: OP_TAP_EN [31:31] */
#define BCHP_SDIO_0_CFG_OP_DLY_OP_TAP_EN_MASK                      0x80000000
#define BCHP_SDIO_0_CFG_OP_DLY_OP_TAP_EN_SHIFT                     31
#define BCHP_SDIO_0_CFG_OP_DLY_OP_TAP_EN_DEFAULT                   1

/* SDIO_0_CFG :: OP_DLY :: reserved0 [30:18] */
#define BCHP_SDIO_0_CFG_OP_DLY_reserved0_MASK                      0x7ffc0000
#define BCHP_SDIO_0_CFG_OP_DLY_reserved0_SHIFT                     18

/* SDIO_0_CFG :: OP_DLY :: OP_DELAY_CTRL [17:16] */
#define BCHP_SDIO_0_CFG_OP_DLY_OP_DELAY_CTRL_MASK                  0x00030000
#define BCHP_SDIO_0_CFG_OP_DLY_OP_DELAY_CTRL_SHIFT                 16
#define BCHP_SDIO_0_CFG_OP_DLY_OP_DELAY_CTRL_DEFAULT               3

/* SDIO_0_CFG :: OP_DLY :: reserved1 [15:04] */
#define BCHP_SDIO_0_CFG_OP_DLY_reserved1_MASK                      0x0000fff0
#define BCHP_SDIO_0_CFG_OP_DLY_reserved1_SHIFT                     4

/* SDIO_0_CFG :: OP_DLY :: OP_TAP_DELAY [03:00] */
#define BCHP_SDIO_0_CFG_OP_DLY_OP_TAP_DELAY_MASK                   0x0000000f
#define BCHP_SDIO_0_CFG_OP_DLY_OP_TAP_DELAY_SHIFT                  0
#define BCHP_SDIO_0_CFG_OP_DLY_OP_TAP_DELAY_DEFAULT                15

/***************************************************************************
 *SCRATCH - SDIO Scratch Register
 ***************************************************************************/
/* SDIO_0_CFG :: SCRATCH :: SCRATCH_BITS [31:00] */
#define BCHP_SDIO_0_CFG_SCRATCH_SCRATCH_BITS_MASK                  0xffffffff
#define BCHP_SDIO_0_CFG_SCRATCH_SCRATCH_BITS_SHIFT                 0
#define BCHP_SDIO_0_CFG_SCRATCH_SCRATCH_BITS_DEFAULT               0

/***************************************************************************
 *VERSION - SDIO VERSION Register
 ***************************************************************************/
/* SDIO_0_CFG :: VERSION :: SD_VER [31:24] */
#define BCHP_SDIO_0_CFG_VERSION_SD_VER_MASK                        0xff000000
#define BCHP_SDIO_0_CFG_VERSION_SD_VER_SHIFT                       24
#define BCHP_SDIO_0_CFG_VERSION_SD_VER_DEFAULT                     48

/* SDIO_0_CFG :: VERSION :: MMC_VER [23:16] */
#define BCHP_SDIO_0_CFG_VERSION_MMC_VER_MASK                       0x00ff0000
#define BCHP_SDIO_0_CFG_VERSION_MMC_VER_SHIFT                      16
#define BCHP_SDIO_0_CFG_VERSION_MMC_VER_DEFAULT                    67

/* SDIO_0_CFG :: VERSION :: REV [15:08] */
#define BCHP_SDIO_0_CFG_VERSION_REV_MASK                           0x0000ff00
#define BCHP_SDIO_0_CFG_VERSION_REV_SHIFT                          8
#define BCHP_SDIO_0_CFG_VERSION_REV_DEFAULT                        160

/* SDIO_0_CFG :: VERSION :: A2S_VER [07:00] */
#define BCHP_SDIO_0_CFG_VERSION_A2S_VER_MASK                       0x000000ff
#define BCHP_SDIO_0_CFG_VERSION_A2S_VER_SHIFT                      0
#define BCHP_SDIO_0_CFG_VERSION_A2S_VER_DEFAULT                    1

#endif /* #ifndef BCHP_SDIO_0_CFG_H__ */

/* End of File */
