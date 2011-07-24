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
 * Date:           Generated on         Wed Aug  4 16:27:03 2010
 *                 MD5 Checksum         27c5a1259680e8176595e9e88d9958c6
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7408/rdb/b0/bchp_ddr23_phy_byte_lane_0_0.h $
 * 
 * Hydra_Software_Devel/1   8/4/10 6:04p pntruong
 * SW7408-118: Initial version.
 *
 ***************************************************************************/

#ifndef BCHP_DDR23_PHY_BYTE_LANE_0_0_H__
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_H__

/***************************************************************************
 *DDR23_PHY_BYTE_LANE_0_0 - DDR23 DDR23 byte lane #0 control registers
 ***************************************************************************/
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_REVISION    0x003b6200 /* Byte lane revision register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_VDL_CALIBRATE 0x003b6204 /* Byte lane VDL calibration control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_VDL_STATUS  0x003b6208 /* Byte lane VDL calibration status register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_VDL_OVERRIDE_0 0x003b6210 /* Read DQSP VDL static override control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_VDL_OVERRIDE_1 0x003b6214 /* Read DQSN VDL static override control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_VDL_OVERRIDE_2 0x003b6218 /* Read Enable VDL static override control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_VDL_OVERRIDE_3 0x003b621c /* Write data and mask VDL static override control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_VDL_OVERRIDE_4 0x003b6220 /* Read DQSP VDL dynamic override control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_VDL_OVERRIDE_5 0x003b6224 /* Read DQSN VDL dynamic override control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_VDL_OVERRIDE_6 0x003b6228 /* Read Enable VDL dynamic override control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_VDL_OVERRIDE_7 0x003b622c /* Write data and mask VDL dynamic override control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_READ_CONTROL 0x003b6230 /* Byte Lane read channel control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_READ_FIFO_STATUS 0x003b6234 /* Read fifo status register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_READ_FIFO_CLEAR 0x003b6238 /* Read fifo status clear register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_IDLE_PAD_CONTROL 0x003b623c /* Idle mode SSTL pad control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_DRIVE_PAD_CTL 0x003b6240 /* SSTL pad drive characteristics control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_CLOCK_PAD_DISABLE 0x003b6244 /* Clock pad disable register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_WR_PREAMBLE_MODE 0x003b6248 /* Write cycle preamble control register */
#define BCHP_DDR23_PHY_BYTE_LANE_0_0_CLOCK_REG_CONTROL 0x003b624c /* Clock Regulator control register */

#endif /* #ifndef BCHP_DDR23_PHY_BYTE_LANE_0_0_H__ */

/* End of File */
