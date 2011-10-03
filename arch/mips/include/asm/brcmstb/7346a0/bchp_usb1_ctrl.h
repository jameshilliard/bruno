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
 * Date:           Generated on         Mon Aug  2 14:59:05 2010
 *                 MD5 Checksum         616c5da46e7f2e811fb57b174083665d
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7346/rdb/a0/bchp_usb1_ctrl.h $
 * 
 * Hydra_Software_Devel/1   8/2/10 8:09p pntruong
 * SW7346-2: Initial version.
 *
 ***************************************************************************/

#ifndef BCHP_USB1_CTRL_H__
#define BCHP_USB1_CTRL_H__

/***************************************************************************
 *USB1_CTRL - USB Control Registers
 ***************************************************************************/
#define BCHP_USB1_CTRL_SETUP                     0x00490200 /* Setup Register */
#define BCHP_USB1_CTRL_PLL_CTL                   0x00490204 /* PLL Control Register */
#define BCHP_USB1_CTRL_FLADJ_VALUE               0x00490208 /* Frame Adjust Value */
#define BCHP_USB1_CTRL_EBRIDGE                   0x0049020c /* Control Register for EHCI Bridge */
#define BCHP_USB1_CTRL_OBRIDGE                   0x00490210 /* Control Register for OHCI Bridge */
#define BCHP_USB1_CTRL_MDIO                      0x00490214 /* MDIO Interface Programming Register */
#define BCHP_USB1_CTRL_MDIO2                     0x00490218 /* MDIO Interface Read Register */
#define BCHP_USB1_CTRL_TEST_PORT_CTL             0x0049021c /* Test Port Control Register */
#define BCHP_USB1_CTRL_USB_SIMCTL                0x00490220 /* Simulation Register */
#define BCHP_USB1_CTRL_USB_TESTCTL               0x00490224 /* Throutput Test Control */
#define BCHP_USB1_CTRL_USB_TESTMON               0x00490228 /* Throughput Test Monitor */
#define BCHP_USB1_CTRL_UTMI_CTL_1                0x0049022c /* UTMI Control Register */
#define BCHP_USB1_CTRL_UTMI_CTL_2                0x00490230 /* UTMI Control 2 Register */
#define BCHP_USB1_CTRL_SPARE1                    0x00490234 /* Spare1 Register for future use */
#define BCHP_USB1_CTRL_SPARE2                    0x00490238 /* Spare2 Register for future use */

#endif /* #ifndef BCHP_USB1_CTRL_H__ */

/* End of File */
