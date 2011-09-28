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
 * Date:           Generated on         Mon May  2 17:18:09 2011
 *                 MD5 Checksum         871c66a57ba55431af8d77c548644752
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7346/rdb/a0/bchp_moca_hostmisc.h $
 * 
 * Hydra_Software_Devel/3   5/3/11 4:30p pntruong
 * SW7346-170: Resync with central RDB.  Magnum needs to support 3rd
 * master SPI channel.
 *
 ***************************************************************************/

#ifndef BCHP_MOCA_HOSTMISC_H__
#define BCHP_MOCA_HOSTMISC_H__

/***************************************************************************
 *MOCA_HOSTMISC - MOCA_HOSTMISC registers
 ***************************************************************************/
#define BCHP_MOCA_HOSTMISC_SW_RESET              0x002a2040 /* Moca Software Reset */
#define BCHP_MOCA_HOSTMISC_SCRATCH               0x002a2044 /* Moca Scratch Register */
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG      0x002a204c /* MoCA version register */
#define BCHP_MOCA_HOSTMISC_HOST_MMP0             0x002a2050 /* Moca Host Messaging Register 0 */
#define BCHP_MOCA_HOSTMISC_HOST_MMP1             0x002a2054 /* Moca Host Messaging Register 1 */
#define BCHP_MOCA_HOSTMISC_HOST_MMP2             0x002a2058 /* Moca Host Messaging Register 2 */
#define BCHP_MOCA_HOSTMISC_HOST_MMP3             0x002a205c /* Moca Host Messaging Register 3 */
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG          0x002a2060 /* Host-to-MoCA Interrupt Trigger */
#define BCHP_MOCA_HOSTMISC_WAKEUP                0x002a2064 /* Host-to-MoCA Wakeup Interrupt */

/***************************************************************************
 *SW_RESET - Moca Software Reset
 ***************************************************************************/
/* MOCA_HOSTMISC :: SW_RESET :: spare_control [31:15] */
#define BCHP_MOCA_HOSTMISC_SW_RESET_spare_control_MASK             0xffff8000
#define BCHP_MOCA_HOSTMISC_SW_RESET_spare_control_SHIFT            15
#define BCHP_MOCA_HOSTMISC_SW_RESET_spare_control_DEFAULT          131071

/* MOCA_HOSTMISC :: SW_RESET :: spare_status [14:08] */
#define BCHP_MOCA_HOSTMISC_SW_RESET_spare_status_MASK              0x00007f00
#define BCHP_MOCA_HOSTMISC_SW_RESET_spare_status_SHIFT             8
#define BCHP_MOCA_HOSTMISC_SW_RESET_spare_status_DEFAULT           127

/* MOCA_HOSTMISC :: SW_RESET :: moca_disable_clocks [07:07] */
#define BCHP_MOCA_HOSTMISC_SW_RESET_moca_disable_clocks_MASK       0x00000080
#define BCHP_MOCA_HOSTMISC_SW_RESET_moca_disable_clocks_SHIFT      7
#define BCHP_MOCA_HOSTMISC_SW_RESET_moca_disable_clocks_DEFAULT    1

/* MOCA_HOSTMISC :: SW_RESET :: spare_reset [06:02] */
#define BCHP_MOCA_HOSTMISC_SW_RESET_spare_reset_MASK               0x0000007c
#define BCHP_MOCA_HOSTMISC_SW_RESET_spare_reset_SHIFT              2
#define BCHP_MOCA_HOSTMISC_SW_RESET_spare_reset_DEFAULT            31

/* MOCA_HOSTMISC :: SW_RESET :: moca_sys_reset [01:01] */
#define BCHP_MOCA_HOSTMISC_SW_RESET_moca_sys_reset_MASK            0x00000002
#define BCHP_MOCA_HOSTMISC_SW_RESET_moca_sys_reset_SHIFT           1
#define BCHP_MOCA_HOSTMISC_SW_RESET_moca_sys_reset_DEFAULT         1

/* MOCA_HOSTMISC :: SW_RESET :: moca_cpu_reset [00:00] */
#define BCHP_MOCA_HOSTMISC_SW_RESET_moca_cpu_reset_MASK            0x00000001
#define BCHP_MOCA_HOSTMISC_SW_RESET_moca_cpu_reset_SHIFT           0
#define BCHP_MOCA_HOSTMISC_SW_RESET_moca_cpu_reset_DEFAULT         1

/***************************************************************************
 *SCRATCH - Moca Scratch Register
 ***************************************************************************/
/* MOCA_HOSTMISC :: SCRATCH :: VALUE [31:00] */
#define BCHP_MOCA_HOSTMISC_SCRATCH_VALUE_MASK                      0xffffffff
#define BCHP_MOCA_HOSTMISC_SCRATCH_VALUE_SHIFT                     0
#define BCHP_MOCA_HOSTMISC_SCRATCH_VALUE_DEFAULT                   0

/***************************************************************************
 *MOCA_VERSION_REG - MoCA version register
 ***************************************************************************/
/* MOCA_HOSTMISC :: MOCA_VERSION_REG :: moca_id [31:16] */
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_moca_id_MASK           0xffff0000
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_moca_id_SHIFT          16
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_moca_id_DEFAULT        26146

/* MOCA_HOSTMISC :: MOCA_VERSION_REG :: moca_spec_ver [15:12] */
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_moca_spec_ver_MASK     0x0000f000
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_moca_spec_ver_SHIFT    12
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_moca_spec_ver_DEFAULT  1

/* MOCA_HOSTMISC :: MOCA_VERSION_REG :: core_version [11:08] */
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_core_version_MASK      0x00000f00
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_core_version_SHIFT     8
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_core_version_DEFAULT   0

/* MOCA_HOSTMISC :: MOCA_VERSION_REG :: core_revision [07:04] */
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_core_revision_MASK     0x000000f0
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_core_revision_SHIFT    4
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_core_revision_DEFAULT  0

/* MOCA_HOSTMISC :: MOCA_VERSION_REG :: core_mask [03:00] */
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_core_mask_MASK         0x0000000f
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_core_mask_SHIFT        0
#define BCHP_MOCA_HOSTMISC_MOCA_VERSION_REG_core_mask_DEFAULT      0

/***************************************************************************
 *HOST_MMP0 - Moca Host Messaging Register 0
 ***************************************************************************/
/* MOCA_HOSTMISC :: HOST_MMP0 :: MMP [31:00] */
#define BCHP_MOCA_HOSTMISC_HOST_MMP0_MMP_MASK                      0xffffffff
#define BCHP_MOCA_HOSTMISC_HOST_MMP0_MMP_SHIFT                     0
#define BCHP_MOCA_HOSTMISC_HOST_MMP0_MMP_DEFAULT                   0

/***************************************************************************
 *HOST_MMP1 - Moca Host Messaging Register 1
 ***************************************************************************/
/* MOCA_HOSTMISC :: HOST_MMP1 :: MMP [31:00] */
#define BCHP_MOCA_HOSTMISC_HOST_MMP1_MMP_MASK                      0xffffffff
#define BCHP_MOCA_HOSTMISC_HOST_MMP1_MMP_SHIFT                     0
#define BCHP_MOCA_HOSTMISC_HOST_MMP1_MMP_DEFAULT                   0

/***************************************************************************
 *HOST_MMP2 - Moca Host Messaging Register 2
 ***************************************************************************/
/* MOCA_HOSTMISC :: HOST_MMP2 :: MMP [31:00] */
#define BCHP_MOCA_HOSTMISC_HOST_MMP2_MMP_MASK                      0xffffffff
#define BCHP_MOCA_HOSTMISC_HOST_MMP2_MMP_SHIFT                     0
#define BCHP_MOCA_HOSTMISC_HOST_MMP2_MMP_DEFAULT                   0

/***************************************************************************
 *HOST_MMP3 - Moca Host Messaging Register 3
 ***************************************************************************/
/* MOCA_HOSTMISC :: HOST_MMP3 :: MMP [31:00] */
#define BCHP_MOCA_HOSTMISC_HOST_MMP3_MMP_MASK                      0xffffffff
#define BCHP_MOCA_HOSTMISC_HOST_MMP3_MMP_SHIFT                     0
#define BCHP_MOCA_HOSTMISC_HOST_MMP3_MMP_DEFAULT                   0

/***************************************************************************
 *H2M_INT_TRIG - Host-to-MoCA Interrupt Trigger
 ***************************************************************************/
/* MOCA_HOSTMISC :: H2M_INT_TRIG :: reserved0 [31:08] */
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG_reserved0_MASK             0xffffff00
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG_reserved0_SHIFT            8

/* MOCA_HOSTMISC :: H2M_INT_TRIG :: INT_TRIG [07:00] */
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG_INT_TRIG_MASK              0x000000ff
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG_INT_TRIG_SHIFT             0
#define BCHP_MOCA_HOSTMISC_H2M_INT_TRIG_INT_TRIG_DEFAULT           0

/***************************************************************************
 *WAKEUP - Host-to-MoCA Wakeup Interrupt
 ***************************************************************************/
/* MOCA_HOSTMISC :: WAKEUP :: reserved0 [31:01] */
#define BCHP_MOCA_HOSTMISC_WAKEUP_reserved0_MASK                   0xfffffffe
#define BCHP_MOCA_HOSTMISC_WAKEUP_reserved0_SHIFT                  1

/* MOCA_HOSTMISC :: WAKEUP :: wakeup_int [00:00] */
#define BCHP_MOCA_HOSTMISC_WAKEUP_wakeup_int_MASK                  0x00000001
#define BCHP_MOCA_HOSTMISC_WAKEUP_wakeup_int_SHIFT                 0
#define BCHP_MOCA_HOSTMISC_WAKEUP_wakeup_int_DEFAULT               0

#endif /* #ifndef BCHP_MOCA_HOSTMISC_H__ */

/* End of File */
