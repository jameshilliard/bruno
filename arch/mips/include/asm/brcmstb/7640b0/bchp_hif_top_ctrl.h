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
 * Date:           Generated on         Tue Apr 12 13:36:55 2011
 *                 MD5 Checksum         161bc6c4c68f438ad316017c113c5764
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7640/rdb/c0/bchp_hif_top_ctrl.h $
 * 
 * Hydra_Software_Devel/1   4/13/11 5:03p albertl
 * SWBLURAY-25497: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_HIF_TOP_CTRL_H__
#define BCHP_HIF_TOP_CTRL_H__

/***************************************************************************
 *HIF_TOP_CTRL - HIF Top Control Registers
 ***************************************************************************/
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL          0x01402400 /* External IRQ Active Level Control Register */
#define BCHP_HIF_TOP_CTRL_TM_CTRL                0x01402404 /* HIF MBIST_TM_CTRL Register */
#define BCHP_HIF_TOP_CTRL_SCRATCH                0x01402408 /* HIF Scratch Register */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0              0x0140240c /* HIF Power Management Control Register 0 */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1              0x01402410 /* HIF Power Management Control Register 1 */
#define BCHP_HIF_TOP_CTRL_STRAP_INTERCEPT        0x01402414 /* HIF Strap Intercept Register for Testing */
#define BCHP_HIF_TOP_CTRL_FLASH_TYPE             0x01402418 /* HIF Decoded Flash Type */

/***************************************************************************
 *EXT_IRQ_LEVEL - External IRQ Active Level Control Register
 ***************************************************************************/
/* HIF_TOP_CTRL :: EXT_IRQ_LEVEL :: reserved0 [31:05] */
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_reserved0_MASK             0xffffffe0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_reserved0_SHIFT            5

/* HIF_TOP_CTRL :: EXT_IRQ_LEVEL :: ext_irq_4_level [04:04] */
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_4_level_MASK       0x00000010
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_4_level_SHIFT      4
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_4_level_DEFAULT    0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_4_level_LOW        0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_4_level_HIGH       1

/* HIF_TOP_CTRL :: EXT_IRQ_LEVEL :: ext_irq_3_level [03:03] */
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_3_level_MASK       0x00000008
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_3_level_SHIFT      3
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_3_level_DEFAULT    0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_3_level_LOW        0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_3_level_HIGH       1

/* HIF_TOP_CTRL :: EXT_IRQ_LEVEL :: ext_irq_2_level [02:02] */
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_2_level_MASK       0x00000004
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_2_level_SHIFT      2
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_2_level_DEFAULT    0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_2_level_LOW        0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_2_level_HIGH       1

/* HIF_TOP_CTRL :: EXT_IRQ_LEVEL :: ext_irq_1_level [01:01] */
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_1_level_MASK       0x00000002
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_1_level_SHIFT      1
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_1_level_DEFAULT    0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_1_level_LOW        0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_1_level_HIGH       1

/* HIF_TOP_CTRL :: EXT_IRQ_LEVEL :: ext_irq_0_level [00:00] */
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_0_level_MASK       0x00000001
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_0_level_SHIFT      0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_0_level_DEFAULT    0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_0_level_LOW        0
#define BCHP_HIF_TOP_CTRL_EXT_IRQ_LEVEL_ext_irq_0_level_HIGH       1

/***************************************************************************
 *TM_CTRL - HIF MBIST_TM_CTRL Register
 ***************************************************************************/
/* HIF_TOP_CTRL :: TM_CTRL :: reserved0 [31:28] */
#define BCHP_HIF_TOP_CTRL_TM_CTRL_reserved0_MASK                   0xf0000000
#define BCHP_HIF_TOP_CTRL_TM_CTRL_reserved0_SHIFT                  28

/* HIF_TOP_CTRL :: TM_CTRL :: M2MD_TM [27:16] */
#define BCHP_HIF_TOP_CTRL_TM_CTRL_M2MD_TM_MASK                     0x0fff0000
#define BCHP_HIF_TOP_CTRL_TM_CTRL_M2MD_TM_SHIFT                    16
#define BCHP_HIF_TOP_CTRL_TM_CTRL_M2MD_TM_DEFAULT                  0
#define BCHP_HIF_TOP_CTRL_TM_CTRL_M2MD_TM_DISABLE                  0
#define BCHP_HIF_TOP_CTRL_TM_CTRL_M2MD_TM_ENABLE                   1

/* HIF_TOP_CTRL :: TM_CTRL :: reserved1 [15:08] */
#define BCHP_HIF_TOP_CTRL_TM_CTRL_reserved1_MASK                   0x0000ff00
#define BCHP_HIF_TOP_CTRL_TM_CTRL_reserved1_SHIFT                  8

/* HIF_TOP_CTRL :: TM_CTRL :: MPI_NAND_FIFO_TM [07:00] */
#define BCHP_HIF_TOP_CTRL_TM_CTRL_MPI_NAND_FIFO_TM_MASK            0x000000ff
#define BCHP_HIF_TOP_CTRL_TM_CTRL_MPI_NAND_FIFO_TM_SHIFT           0
#define BCHP_HIF_TOP_CTRL_TM_CTRL_MPI_NAND_FIFO_TM_DEFAULT         0
#define BCHP_HIF_TOP_CTRL_TM_CTRL_MPI_NAND_FIFO_TM_DISABLE         0
#define BCHP_HIF_TOP_CTRL_TM_CTRL_MPI_NAND_FIFO_TM_ENABLE          1

/***************************************************************************
 *SCRATCH - HIF Scratch Register
 ***************************************************************************/
/* HIF_TOP_CTRL :: SCRATCH :: SCRATCH_BIT [31:00] */
#define BCHP_HIF_TOP_CTRL_SCRATCH_SCRATCH_BIT_MASK                 0xffffffff
#define BCHP_HIF_TOP_CTRL_SCRATCH_SCRATCH_BIT_SHIFT                0
#define BCHP_HIF_TOP_CTRL_SCRATCH_SCRATCH_BIT_DEFAULT              0

/***************************************************************************
 *PM_CTRL_0 - HIF Power Management Control Register 0
 ***************************************************************************/
/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_15 [31:30] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_15_MASK                 0xc0000000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_15_SHIFT                30
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_15_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_15_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_15_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_15_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_15_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_14 [29:28] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_14_MASK                 0x30000000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_14_SHIFT                28
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_14_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_14_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_14_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_14_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_14_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_13 [27:26] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_13_MASK                 0x0c000000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_13_SHIFT                26
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_13_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_13_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_13_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_13_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_13_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_12 [25:24] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_12_MASK                 0x03000000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_12_SHIFT                24
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_12_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_12_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_12_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_12_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_12_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_11 [23:22] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_11_MASK                 0x00c00000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_11_SHIFT                22
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_11_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_11_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_11_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_11_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_11_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_10 [21:20] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_10_MASK                 0x00300000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_10_SHIFT                20
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_10_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_10_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_10_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_10_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_10_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_09 [19:18] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_09_MASK                 0x000c0000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_09_SHIFT                18
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_09_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_09_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_09_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_09_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_09_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_08 [17:16] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_08_MASK                 0x00030000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_08_SHIFT                16
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_08_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_08_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_08_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_08_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_08_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_07 [15:14] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_07_MASK                 0x0000c000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_07_SHIFT                14
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_07_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_07_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_07_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_07_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_07_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_06 [13:12] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_06_MASK                 0x00003000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_06_SHIFT                12
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_06_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_06_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_06_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_06_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_06_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_05 [11:10] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_05_MASK                 0x00000c00
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_05_SHIFT                10
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_05_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_05_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_05_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_05_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_05_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_04 [09:08] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_04_MASK                 0x00000300
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_04_SHIFT                8
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_04_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_04_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_04_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_04_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_04_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_03 [07:06] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_03_MASK                 0x000000c0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_03_SHIFT                6
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_03_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_03_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_03_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_03_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_03_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_02 [05:04] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_02_MASK                 0x00000030
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_02_SHIFT                4
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_02_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_02_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_02_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_02_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_02_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_01 [03:02] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_01_MASK                 0x0000000c
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_01_SHIFT                2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_01_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_01_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_01_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_01_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_01_TRISTATE             3

/* HIF_TOP_CTRL :: PM_CTRL_0 :: EBI_AD_00 [01:00] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_00_MASK                 0x00000003
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_00_SHIFT                0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_00_DEFAULT              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_00_DISABLE              0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_00_DRIVE_LOW            1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_00_DRIVE_HIGH           2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_0_EBI_AD_00_TRISTATE             3

/***************************************************************************
 *PM_CTRL_1 - HIF Power Management Control Register 1
 ***************************************************************************/
/* HIF_TOP_CTRL :: PM_CTRL_1 :: reserved0 [31:24] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_reserved0_MASK                 0xff000000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_reserved0_SHIFT                24

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_DSb [23:22] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_DSb_MASK                   0x00c00000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_DSb_SHIFT                  22
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_DSb_DEFAULT                0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_DSb_DISABLE                0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_DSb_DRIVE_LOW              1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_DSb_DRIVE_HIGH             2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_DSb_TRISTATE               3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_TSb [21:20] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_TSb_MASK                   0x00300000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_TSb_SHIFT                  20
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_TSb_DEFAULT                0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_TSb_DISABLE                0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_TSb_DRIVE_LOW              1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_TSb_DRIVE_HIGH             2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_TSb_TRISTATE               3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_RDb [19:18] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RDb_MASK                   0x000c0000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RDb_SHIFT                  18
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RDb_DEFAULT                0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RDb_DISABLE                0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RDb_DRIVE_LOW              1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RDb_DRIVE_HIGH             2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RDb_TRISTATE               3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_WE1b [17:16] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE1b_MASK                  0x00030000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE1b_SHIFT                 16
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE1b_DEFAULT               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE1b_DISABLE               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE1b_DRIVE_LOW             1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE1b_DRIVE_HIGH            2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE1b_TRISTATE              3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_WE0b [15:14] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE0b_MASK                  0x0000c000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE0b_SHIFT                 14
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE0b_DEFAULT               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE0b_DISABLE               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE0b_DRIVE_LOW             1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE0b_DRIVE_HIGH            2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_WE0b_TRISTATE              3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_RWb [13:12] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RWb_MASK                   0x00003000
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RWb_SHIFT                  12
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RWb_DEFAULT                0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RWb_DISABLE                0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RWb_DRIVE_LOW              1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RWb_DRIVE_HIGH             2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_RWb_TRISTATE               3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_CS3b [11:10] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS3b_MASK                  0x00000c00
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS3b_SHIFT                 10
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS3b_DEFAULT               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS3b_DISABLE               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS3b_DRIVE_LOW             1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS3b_DRIVE_HIGH            2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS3b_TRISTATE              3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_CS2b [09:08] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS2b_MASK                  0x00000300
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS2b_SHIFT                 8
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS2b_DEFAULT               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS2b_DISABLE               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS2b_DRIVE_LOW             1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS2b_DRIVE_HIGH            2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS2b_TRISTATE              3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_CS1b [07:06] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS1b_MASK                  0x000000c0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS1b_SHIFT                 6
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS1b_DEFAULT               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS1b_DISABLE               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS1b_DRIVE_LOW             1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS1b_DRIVE_HIGH            2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS1b_TRISTATE              3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_CS0b [05:04] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS0b_MASK                  0x00000030
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS0b_SHIFT                 4
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS0b_DEFAULT               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS0b_DISABLE               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS0b_DRIVE_LOW             1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS0b_DRIVE_HIGH            2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_CS0b_TRISTATE              3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_ADDR25 [03:02] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR25_MASK                0x0000000c
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR25_SHIFT               2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR25_DEFAULT             0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR25_DISABLE             0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR25_DRIVE_LOW           1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR25_DRIVE_HIGH          2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR25_TRISTATE            3

/* HIF_TOP_CTRL :: PM_CTRL_1 :: EBI_ADDR24 [01:00] */
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR24_MASK                0x00000003
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR24_SHIFT               0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR24_DEFAULT             0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR24_DISABLE             0
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR24_DRIVE_LOW           1
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR24_DRIVE_HIGH          2
#define BCHP_HIF_TOP_CTRL_PM_CTRL_1_EBI_ADDR24_TRISTATE            3

/***************************************************************************
 *STRAP_INTERCEPT - HIF Strap Intercept Register for Testing
 ***************************************************************************/
/* HIF_TOP_CTRL :: STRAP_INTERCEPT :: reserved0 [31:01] */
#define BCHP_HIF_TOP_CTRL_STRAP_INTERCEPT_reserved0_MASK           0xfffffffe
#define BCHP_HIF_TOP_CTRL_STRAP_INTERCEPT_reserved0_SHIFT          1

/* HIF_TOP_CTRL :: STRAP_INTERCEPT :: STRAP_CLIENT_MODE [00:00] */
#define BCHP_HIF_TOP_CTRL_STRAP_INTERCEPT_STRAP_CLIENT_MODE_MASK   0x00000001
#define BCHP_HIF_TOP_CTRL_STRAP_INTERCEPT_STRAP_CLIENT_MODE_SHIFT  0
#define BCHP_HIF_TOP_CTRL_STRAP_INTERCEPT_STRAP_CLIENT_MODE_DEFAULT 1

/***************************************************************************
 *FLASH_TYPE - HIF Decoded Flash Type
 ***************************************************************************/
/* HIF_TOP_CTRL :: FLASH_TYPE :: reserved0 [31:03] */
#define BCHP_HIF_TOP_CTRL_FLASH_TYPE_reserved0_MASK                0xfffffff8
#define BCHP_HIF_TOP_CTRL_FLASH_TYPE_reserved0_SHIFT               3

/* HIF_TOP_CTRL :: FLASH_TYPE :: InvalidStrap [02:02] */
#define BCHP_HIF_TOP_CTRL_FLASH_TYPE_InvalidStrap_MASK             0x00000004
#define BCHP_HIF_TOP_CTRL_FLASH_TYPE_InvalidStrap_SHIFT            2
#define BCHP_HIF_TOP_CTRL_FLASH_TYPE_InvalidStrap_DEFAULT          0

/* HIF_TOP_CTRL :: FLASH_TYPE :: FLASH_TYPE [01:00] */
#define BCHP_HIF_TOP_CTRL_FLASH_TYPE_FLASH_TYPE_MASK               0x00000003
#define BCHP_HIF_TOP_CTRL_FLASH_TYPE_FLASH_TYPE_SHIFT              0
#define BCHP_HIF_TOP_CTRL_FLASH_TYPE_FLASH_TYPE_DEFAULT            0

#endif /* #ifndef BCHP_HIF_TOP_CTRL_H__ */

/* End of File */
