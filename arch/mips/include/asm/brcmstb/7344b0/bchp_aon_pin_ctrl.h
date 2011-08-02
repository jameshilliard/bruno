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
 * Date:           Generated on         Fri Apr  1 16:33:52 2011
 *                 MD5 Checksum         d03d08c4839c3311c9d35c4cd5e10373
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7344/rdb/b0/bchp_aon_pin_ctrl.h $
 * 
 * Hydra_Software_Devel/1   4/4/11 12:42p albertl
 * SW7344-40: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_AON_PIN_CTRL_H__
#define BCHP_AON_PIN_CTRL_H__

/***************************************************************************
 *AON_PIN_CTRL - AON Pinmux Control Registers
 ***************************************************************************/
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0         0x00408500 /* Pinmux control register 0 */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1         0x00408504 /* Pinmux control register 1 */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2         0x00408508 /* Pinmux control register 2 */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3         0x0040850c /* Pinmux control register 3 */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0     0x00408510 /* Pad pull-up/pull-down control register 0 */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1     0x00408514 /* Pad pull-up/pull-down control register 1 */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2     0x00408518 /* Pad pull-up/pull-down control register 2 */
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0     0x0040851c /* Bypass clock unselect register 0 */

/***************************************************************************
 *PIN_MUX_CTRL_0 - Pinmux control register 0
 ***************************************************************************/
/* AON_PIN_CTRL :: PIN_MUX_CTRL_0 :: aon_gpio_14 [31:28] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_14_MASK          0xf0000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_14_SHIFT         28
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_14_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_14_AON_GPIO_14   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_14_SDS0_DSEC_VCTL 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_14_SDS0_IF_AGC   2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_14_AIO_EXTMCLK0  3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_14_PM_AON_GPIO_14 4

/* AON_PIN_CTRL :: PIN_MUX_CTRL_0 :: aon_gpio_13 [27:24] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_13_MASK          0x0f000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_13_SHIFT         24
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_13_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_13_AON_GPIO_13   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_13_SDS0_DSEC_SELVTOP 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_13_I2S_BIDIR_CLK_IN 2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_13_I2S_BIDIR_CLK_OUT 3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_13_SDS0_RF_AGC   4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_13_TP_OUT_26     5
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_13_PM_AON_GPIO_13 6

/* AON_PIN_CTRL :: PIN_MUX_CTRL_0 :: aon_gpio_12 [23:20] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_12_MASK          0x00f00000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_12_SHIFT         20
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_12_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_12_AON_GPIO_12   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_12_SDS0_DSEC_TXEN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_12_I2S_BIDIR_SYNC_IN 2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_12_I2S_BIDIR_SYNC_OUT 3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_12_TP_OUT_25     4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_12_PM_AON_GPIO_12 5

/* AON_PIN_CTRL :: PIN_MUX_CTRL_0 :: aon_gpio_11 [19:16] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_11_MASK          0x000f0000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_11_SHIFT         16
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_11_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_11_AON_GPIO_11   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_11_SDS0_DSEC_TXOUT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_11_I2S_BIDIR_DATA_IN 2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_11_I2S_BIDIR_DATA_OUT 3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_11_TP_OUT_24     4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_11_PM_AON_GPIO_11 5

/* AON_PIN_CTRL :: PIN_MUX_CTRL_0 :: aon_gpio_03 [15:12] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_03_MASK          0x0000f000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_03_SHIFT         12
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_03_DEFAULT       4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_03_AON_GPIO_03   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_03_LED_KD_3      1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_03_EXT_IRQB_5    2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_03_UART_RTSCB    3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_03_AON_PM_LED_OUT 4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_03_TP_OUT_03     5
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_03_PM_AON_GPIO_03 6

/* AON_PIN_CTRL :: PIN_MUX_CTRL_0 :: aon_gpio_02 [11:08] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_02_MASK          0x00000f00
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_02_SHIFT         8
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_02_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_02_AON_GPIO_02   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_02_LED_KD_2      1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_02_EXT_IRQB_4    2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_02_UART_CTSCB    3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_02_SC0_RST_ALT   4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_02_TP_OUT_02     5
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_02_PM_AON_GPIO_02 6

/* AON_PIN_CTRL :: PIN_MUX_CTRL_0 :: aon_gpio_01 [07:04] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_01_MASK          0x000000f0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_01_SHIFT         4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_01_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_01_AON_GPIO_01   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_01_LED_KD_1      1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_01_EXT_IRQB_3    2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_01_UART_TXDC     3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_01_SC0_CLK_OUT_ALT 4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_01_TP_OUT_01     5
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_01_PM_AON_GPIO_01 6

/* AON_PIN_CTRL :: PIN_MUX_CTRL_0 :: aon_gpio_00 [03:00] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_MASK          0x0000000f
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_SHIFT         0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_AON_GPIO_00   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_LED_KD_0      1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_EXT_IRQB_2    2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_UART_RXDC     3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_SC0_IO_ALT    4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_TP_IN_00      5
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_TP_OUT_00     6
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_0_aon_gpio_00_PM_AON_GPIO_00 7

/***************************************************************************
 *PIN_MUX_CTRL_1 - Pinmux control register 1
 ***************************************************************************/
/* AON_PIN_CTRL :: PIN_MUX_CTRL_1 :: aon_gpio_09 [31:28] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_09_MASK          0xf0000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_09_SHIFT         28
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_09_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_09_AON_GPIO_09   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_09_IR_IN0        1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_09_UHF_LNA_PWRDN 2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_09_ALT_TP_OUT_08 3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_09_PM_AON_GPIO_09 4

/* AON_PIN_CTRL :: PIN_MUX_CTRL_1 :: aon_gpio_08 [27:24] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_08_MASK          0x0f000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_08_SHIFT         24
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_08_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_08_AON_GPIO_08   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_08_EXT_IRQB_10   1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_08_IR_INT        2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_08_RMX_PAUSE1    3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_08_SC0_VPP_ALT   4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_08_TP_OUT_31     5
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_08_PM_AON_GPIO_08 6

/* AON_PIN_CTRL :: PIN_MUX_CTRL_1 :: aon_gpio_07 [23:20] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_07_MASK          0x00f00000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_07_SHIFT         20
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_07_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_07_AON_GPIO_07   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_07_EXT_IRQB_9    1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_07_LED_LS_3      2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_07_RMX_VALID1    3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_07_SC0_IO_AUX2_ALT 4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_07_TP_OUT_30     5
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_07_PM_AON_GPIO_07 6

/* AON_PIN_CTRL :: PIN_MUX_CTRL_1 :: aon_gpio_06 [19:16] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_06_MASK          0x000f0000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_06_SHIFT         16
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_06_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_06_AON_GPIO_06   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_06_EXT_IRQB_8    1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_06_LED_LS_2      2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_06_RMX_SYNC1     3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_06_SC0_IO_AUX1_ALT 4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_06_TP_OUT_29     5
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_06_PM_AON_GPIO_06 6

/* AON_PIN_CTRL :: PIN_MUX_CTRL_1 :: aon_gpio_05 [15:12] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_05_MASK          0x0000f000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_05_SHIFT         12
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_05_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_05_AON_GPIO_05   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_05_EXT_IRQB_7    1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_05_LED_LS_1      2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_05_RMX_DATA1     3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_05_SC0_VCC_ALT   4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_05_TP_OUT_28     5
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_05_PM_AON_GPIO_05 6

/* AON_PIN_CTRL :: PIN_MUX_CTRL_1 :: aon_gpio_04 [11:08] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_04_MASK          0x00000f00
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_04_SHIFT         8
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_04_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_04_AON_GPIO_04   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_04_EXT_IRQB_6    1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_04_LED_LS_0      2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_04_RMX_CLK1      3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_04_SC0_PRES_ALT  4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_04_TP_OUT_27     5
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_04_PM_AON_GPIO_04 6

/* AON_PIN_CTRL :: PIN_MUX_CTRL_1 :: aon_gpio_16 [07:04] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_16_MASK          0x000000f0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_16_SHIFT         4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_16_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_16_AON_GPIO_16   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_16_AON_PM_LED_OUT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_16_PM_AON_GPIO_16 2

/* AON_PIN_CTRL :: PIN_MUX_CTRL_1 :: aon_gpio_15 [03:00] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_15_MASK          0x0000000f
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_15_SHIFT         0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_15_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_15_AON_GPIO_15   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_15_EXT_IRQB_16   1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_15_P1_INTR       2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_15_AIO_EXTMCLK1  3
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_1_aon_gpio_15_PM_AON_GPIO_15 4

/***************************************************************************
 *PIN_MUX_CTRL_2 - Pinmux control register 2
 ***************************************************************************/
/* AON_PIN_CTRL :: PIN_MUX_CTRL_2 :: aon_spi_m_miso [31:28] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_miso_MASK       0xf0000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_miso_SHIFT      28
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_miso_DEFAULT    0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_miso_AON_SPI_M_MISO 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_miso_ALT_TP_OUT_04 1

/* AON_PIN_CTRL :: PIN_MUX_CTRL_2 :: aon_spi_m_mosi [27:24] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_mosi_MASK       0x0f000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_mosi_SHIFT      24
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_mosi_DEFAULT    0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_mosi_AON_SPI_M_MOSI 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_mosi_SPI_M_MOSI 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_mosi_ALT_TP_OUT_03 2

/* AON_PIN_CTRL :: PIN_MUX_CTRL_2 :: aon_spi_m_sck [23:20] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_sck_MASK        0x00f00000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_sck_SHIFT       20
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_sck_DEFAULT     0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_sck_AON_SPI_M_SCK 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_spi_m_sck_ALT_TP_OUT_02 1

/* AON_PIN_CTRL :: PIN_MUX_CTRL_2 :: aon_sgpio_01 [19:16] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_01_MASK         0x000f0000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_01_SHIFT        16
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_01_DEFAULT      0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_01_AON_SGPIO_01 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_01_BSC_M2_SDA   1

/* AON_PIN_CTRL :: PIN_MUX_CTRL_2 :: aon_sgpio_00 [15:12] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_00_MASK         0x0000f000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_00_SHIFT        12
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_00_DEFAULT      0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_00_AON_SGPIO_00 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_00_BSC_M2_SCL   1

/* AON_PIN_CTRL :: PIN_MUX_CTRL_2 :: aon_sgpio_03 [11:08] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_03_MASK         0x00000f00
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_03_SHIFT        8
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_03_DEFAULT      0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_03_AON_SGPIO_03 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_03_AON_BSC_M0_SDA 1

/* AON_PIN_CTRL :: PIN_MUX_CTRL_2 :: aon_sgpio_02 [07:04] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_02_MASK         0x000000f0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_02_SHIFT        4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_02_DEFAULT      0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_02_AON_SGPIO_02 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_sgpio_02_AON_BSC_M0_SCL 1

/* AON_PIN_CTRL :: PIN_MUX_CTRL_2 :: aon_gpio_10 [03:00] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_gpio_10_MASK          0x0000000f
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_gpio_10_SHIFT         0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_gpio_10_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_gpio_10_AON_GPIO_10   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_gpio_10_IR_IN1        1
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_gpio_10_UHF_LNA_PWRDN 2
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_2_aon_gpio_10_PM_AON_GPIO_10 3

/***************************************************************************
 *PIN_MUX_CTRL_3 - Pinmux control register 3
 ***************************************************************************/
/* AON_PIN_CTRL :: PIN_MUX_CTRL_3 :: reserved0 [31:16] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_reserved0_MASK            0xffff0000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_reserved0_SHIFT           16

/* AON_PIN_CTRL :: PIN_MUX_CTRL_3 :: aon_gpio_18 [15:12] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_gpio_18_MASK          0x0000f000
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_gpio_18_SHIFT         12
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_gpio_18_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_gpio_18_AON_GPIO_18   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_gpio_18_AON_IRQB1     1

/* AON_PIN_CTRL :: PIN_MUX_CTRL_3 :: aon_gpio_17 [11:08] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_gpio_17_MASK          0x00000f00
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_gpio_17_SHIFT         8
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_gpio_17_DEFAULT       0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_gpio_17_AON_GPIO_17   0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_gpio_17_AON_IRQB0     1

/* AON_PIN_CTRL :: PIN_MUX_CTRL_3 :: aon_spi_m_ss1b [07:04] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_spi_m_ss1b_MASK       0x000000f0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_spi_m_ss1b_SHIFT      4
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_spi_m_ss1b_DEFAULT    0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_spi_m_ss1b_AON_SPI_M_SS1B 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_spi_m_ss1b_ALT_TP_OUT_06 1

/* AON_PIN_CTRL :: PIN_MUX_CTRL_3 :: aon_spi_m_ss0b [03:00] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_spi_m_ss0b_MASK       0x0000000f
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_spi_m_ss0b_SHIFT      0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_spi_m_ss0b_DEFAULT    0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_spi_m_ss0b_AON_SPI_M_SS0B 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_CTRL_3_aon_spi_m_ss0b_ALT_TP_OUT_05 1

/***************************************************************************
 *PIN_MUX_PAD_CTRL_0 - Pad pull-up/pull-down control register 0
 ***************************************************************************/
/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: spare_pad_ctrl_0 [31:30] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_spare_pad_ctrl_0_MASK 0xc0000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_spare_pad_ctrl_0_SHIFT 30
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_spare_pad_ctrl_0_DEFAULT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_spare_pad_ctrl_0_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_spare_pad_ctrl_0_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_spare_pad_ctrl_0_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_08_pad_ctrl [29:28] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_08_pad_ctrl_MASK 0x30000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_08_pad_ctrl_SHIFT 28
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_08_pad_ctrl_DEFAULT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_08_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_08_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_08_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_07_pad_ctrl [27:26] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_07_pad_ctrl_MASK 0x0c000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_07_pad_ctrl_SHIFT 26
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_07_pad_ctrl_DEFAULT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_07_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_07_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_07_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_06_pad_ctrl [25:24] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_06_pad_ctrl_MASK 0x03000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_06_pad_ctrl_SHIFT 24
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_06_pad_ctrl_DEFAULT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_06_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_06_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_06_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_05_pad_ctrl [23:22] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_05_pad_ctrl_MASK 0x00c00000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_05_pad_ctrl_SHIFT 22
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_05_pad_ctrl_DEFAULT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_05_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_05_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_05_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_04_pad_ctrl [21:20] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_04_pad_ctrl_MASK 0x00300000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_04_pad_ctrl_SHIFT 20
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_04_pad_ctrl_DEFAULT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_04_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_04_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_04_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_16_pad_ctrl [19:18] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_16_pad_ctrl_MASK 0x000c0000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_16_pad_ctrl_SHIFT 18
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_16_pad_ctrl_DEFAULT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_16_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_16_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_16_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_15_pad_ctrl [17:16] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_15_pad_ctrl_MASK 0x00030000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_15_pad_ctrl_SHIFT 16
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_15_pad_ctrl_DEFAULT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_15_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_15_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_15_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_14_pad_ctrl [15:14] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_14_pad_ctrl_MASK 0x0000c000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_14_pad_ctrl_SHIFT 14
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_14_pad_ctrl_DEFAULT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_14_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_14_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_14_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_13_pad_ctrl [13:12] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_13_pad_ctrl_MASK 0x00003000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_13_pad_ctrl_SHIFT 12
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_13_pad_ctrl_DEFAULT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_13_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_13_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_13_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_12_pad_ctrl [11:10] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_12_pad_ctrl_MASK 0x00000c00
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_12_pad_ctrl_SHIFT 10
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_12_pad_ctrl_DEFAULT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_12_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_12_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_12_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_11_pad_ctrl [09:08] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_11_pad_ctrl_MASK 0x00000300
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_11_pad_ctrl_SHIFT 8
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_11_pad_ctrl_DEFAULT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_11_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_11_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_11_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_03_pad_ctrl [07:06] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_03_pad_ctrl_MASK 0x000000c0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_03_pad_ctrl_SHIFT 6
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_03_pad_ctrl_DEFAULT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_03_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_03_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_03_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_02_pad_ctrl [05:04] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_02_pad_ctrl_MASK 0x00000030
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_02_pad_ctrl_SHIFT 4
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_02_pad_ctrl_DEFAULT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_02_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_02_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_02_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_01_pad_ctrl [03:02] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_01_pad_ctrl_MASK 0x0000000c
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_01_pad_ctrl_SHIFT 2
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_01_pad_ctrl_DEFAULT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_01_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_01_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_01_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_0 :: aon_gpio_00_pad_ctrl [01:00] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_00_pad_ctrl_MASK 0x00000003
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_00_pad_ctrl_SHIFT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_00_pad_ctrl_DEFAULT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_00_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_00_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_0_aon_gpio_00_pad_ctrl_PULL_UP 2

/***************************************************************************
 *PIN_MUX_PAD_CTRL_1 - Pad pull-up/pull-down control register 1
 ***************************************************************************/
/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_1 :: spare_pad_ctrl_1 [31:30] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_spare_pad_ctrl_1_MASK 0xc0000000
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_spare_pad_ctrl_1_SHIFT 30
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_spare_pad_ctrl_1_DEFAULT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_spare_pad_ctrl_1_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_spare_pad_ctrl_1_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_spare_pad_ctrl_1_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_1 :: reserved0 [29:04] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_reserved0_MASK        0x3ffffff0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_reserved0_SHIFT       4

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_1 :: aon_gpio_10_pad_ctrl [03:02] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_10_pad_ctrl_MASK 0x0000000c
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_10_pad_ctrl_SHIFT 2
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_10_pad_ctrl_DEFAULT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_10_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_10_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_10_pad_ctrl_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_1 :: aon_gpio_09_pad_ctrl [01:00] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_09_pad_ctrl_MASK 0x00000003
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_09_pad_ctrl_SHIFT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_09_pad_ctrl_DEFAULT 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_09_pad_ctrl_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_09_pad_ctrl_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_1_aon_gpio_09_pad_ctrl_PULL_UP 2

/***************************************************************************
 *PIN_MUX_PAD_CTRL_2 - Pad pull-up/pull-down control register 2
 ***************************************************************************/
/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_2 :: reserved0 [31:06] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2_reserved0_MASK        0xffffffc0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2_reserved0_SHIFT       6

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_2 :: spare_pad_ctrl_2 [05:04] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2_spare_pad_ctrl_2_MASK 0x00000030
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2_spare_pad_ctrl_2_SHIFT 4
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2_spare_pad_ctrl_2_DEFAULT 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2_spare_pad_ctrl_2_PULL_NONE 0
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2_spare_pad_ctrl_2_PULL_DOWN 1
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2_spare_pad_ctrl_2_PULL_UP 2

/* AON_PIN_CTRL :: PIN_MUX_PAD_CTRL_2 :: reserved1 [03:00] */
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2_reserved1_MASK        0x0000000f
#define BCHP_AON_PIN_CTRL_PIN_MUX_PAD_CTRL_2_reserved1_SHIFT       0

/***************************************************************************
 *BYP_CLK_UNSELECT_0 - Bypass clock unselect register 0
 ***************************************************************************/
/* AON_PIN_CTRL :: BYP_CLK_UNSELECT_0 :: reserved0 [31:03] */
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_reserved0_MASK        0xfffffff8
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_reserved0_SHIFT       3

/* AON_PIN_CTRL :: BYP_CLK_UNSELECT_0 :: unsel_byp_clk_on_aon_nmib [02:02] */
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_unsel_byp_clk_on_aon_nmib_MASK 0x00000004
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_unsel_byp_clk_on_aon_nmib_SHIFT 2
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_unsel_byp_clk_on_aon_nmib_DEFAULT 0

/* AON_PIN_CTRL :: BYP_CLK_UNSELECT_0 :: unsel_byp_clk_on_aon_gpio_10 [01:01] */
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_unsel_byp_clk_on_aon_gpio_10_MASK 0x00000002
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_unsel_byp_clk_on_aon_gpio_10_SHIFT 1
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_unsel_byp_clk_on_aon_gpio_10_DEFAULT 0

/* AON_PIN_CTRL :: BYP_CLK_UNSELECT_0 :: unsel_byp_clk_on_aon_gpio_14 [00:00] */
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_unsel_byp_clk_on_aon_gpio_14_MASK 0x00000001
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_unsel_byp_clk_on_aon_gpio_14_SHIFT 0
#define BCHP_AON_PIN_CTRL_BYP_CLK_UNSELECT_0_unsel_byp_clk_on_aon_gpio_14_DEFAULT 0

#endif /* #ifndef BCHP_AON_PIN_CTRL_H__ */

/* End of File */
