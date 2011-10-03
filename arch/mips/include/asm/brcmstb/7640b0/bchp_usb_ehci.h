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
 * Date:           Generated on         Tue Apr 12 13:40:24 2011
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
 * $brcm_Log: /magnum/basemodules/chp/7640/rdb/c0/bchp_usb_ehci.h $
 * 
 * Hydra_Software_Devel/1   4/13/11 6:43p albertl
 * SWBLURAY-25497: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_USB_EHCI_H__
#define BCHP_USB_EHCI_H__

/***************************************************************************
 *USB_EHCI - USB EHCI  Control Registers
 ***************************************************************************/
#define BCHP_USB_EHCI_HCCAPBASE                  0x00580300 /* EHCI Capability Register */
#define BCHP_USB_EHCI_HCSPARAMS                  0x00580304 /* EHCI Structural Parameter */
#define BCHP_USB_EHCI_HCCPARAMS                  0x00580308 /* EHCI Capability Parameter */
#define BCHP_USB_EHCI_USBCMD                     0x00580310 /* USB Command Register */
#define BCHP_USB_EHCI_USBSTS                     0x00580314 /* USB Status  Register */
#define BCHP_USB_EHCI_USBINTR                    0x00580318 /* USB Interrupt Enable Register */
#define BCHP_USB_EHCI_FRINDEX                    0x0058031c /* USB Frame Index Register */
#define BCHP_USB_EHCI_PERIODICLISTBASE           0x00580324 /* Periodic Frame List Base Address Register */
#define BCHP_USB_EHCI_ASYNCLISTADDR              0x00580328 /* Asynchronous List Address */
#define BCHP_USB_EHCI_CONFIGFLAG                 0x00580350 /* Configured Flag Register */
#define BCHP_USB_EHCI_PORTSC_0                   0x00580354 /* Port Status/Control Register for Port 0 */
#define BCHP_USB_EHCI_PORTSC_1                   0x00580358 /* Port Status/Control Register for Port 1 */
#define BCHP_USB_EHCI_INSNREG00                  0x00580390 /* Microframe Base Value Register */
#define BCHP_USB_EHCI_INSNREG01                  0x00580394 /* Packet Buffer OUT/IN Threshold Register */
#define BCHP_USB_EHCI_INSNREG02                  0x00580398 /* Packet Buffer Depth Register */
#define BCHP_USB_EHCI_INSNREG03                  0x0058039c /* Break Memory Transfer Register */
#define BCHP_USB_EHCI_INSNREG04                  0x005803a0 /* Debug Register */
#define BCHP_USB_EHCI_INSNREG05                  0x005803a4 /* UTMI Control and Status Register */

/***************************************************************************
 *HCCAPBASE - EHCI Capability Register
 ***************************************************************************/
/* USB_EHCI :: HCCAPBASE :: HCIVERSION [31:16] */
#define BCHP_USB_EHCI_HCCAPBASE_HCIVERSION_MASK                    0xffff0000
#define BCHP_USB_EHCI_HCCAPBASE_HCIVERSION_SHIFT                   16
#define BCHP_USB_EHCI_HCCAPBASE_HCIVERSION_DEFAULT                 256

/* USB_EHCI :: HCCAPBASE :: reserved0 [15:08] */
#define BCHP_USB_EHCI_HCCAPBASE_reserved0_MASK                     0x0000ff00
#define BCHP_USB_EHCI_HCCAPBASE_reserved0_SHIFT                    8

/* USB_EHCI :: HCCAPBASE :: CAPLENGTH [07:00] */
#define BCHP_USB_EHCI_HCCAPBASE_CAPLENGTH_MASK                     0x000000ff
#define BCHP_USB_EHCI_HCCAPBASE_CAPLENGTH_SHIFT                    0
#define BCHP_USB_EHCI_HCCAPBASE_CAPLENGTH_DEFAULT                  16

/***************************************************************************
 *HCSPARAMS - EHCI Structural Parameter
 ***************************************************************************/
/* USB_EHCI :: HCSPARAMS :: reserved0 [31:24] */
#define BCHP_USB_EHCI_HCSPARAMS_reserved0_MASK                     0xff000000
#define BCHP_USB_EHCI_HCSPARAMS_reserved0_SHIFT                    24

/* USB_EHCI :: HCSPARAMS :: DEBUG_PORT_NUMBER [23:20] */
#define BCHP_USB_EHCI_HCSPARAMS_DEBUG_PORT_NUMBER_MASK             0x00f00000
#define BCHP_USB_EHCI_HCSPARAMS_DEBUG_PORT_NUMBER_SHIFT            20
#define BCHP_USB_EHCI_HCSPARAMS_DEBUG_PORT_NUMBER_DEFAULT          0

/* USB_EHCI :: HCSPARAMS :: reserved1 [19:17] */
#define BCHP_USB_EHCI_HCSPARAMS_reserved1_MASK                     0x000e0000
#define BCHP_USB_EHCI_HCSPARAMS_reserved1_SHIFT                    17

/* USB_EHCI :: HCSPARAMS :: P_INDICATOR [16:16] */
#define BCHP_USB_EHCI_HCSPARAMS_P_INDICATOR_MASK                   0x00010000
#define BCHP_USB_EHCI_HCSPARAMS_P_INDICATOR_SHIFT                  16
#define BCHP_USB_EHCI_HCSPARAMS_P_INDICATOR_DEFAULT                0

/* USB_EHCI :: HCSPARAMS :: N_CC [15:12] */
#define BCHP_USB_EHCI_HCSPARAMS_N_CC_MASK                          0x0000f000
#define BCHP_USB_EHCI_HCSPARAMS_N_CC_SHIFT                         12
#define BCHP_USB_EHCI_HCSPARAMS_N_CC_DEFAULT                       1

/* USB_EHCI :: HCSPARAMS :: N_PCC [11:08] */
#define BCHP_USB_EHCI_HCSPARAMS_N_PCC_MASK                         0x00000f00
#define BCHP_USB_EHCI_HCSPARAMS_N_PCC_SHIFT                        8
#define BCHP_USB_EHCI_HCSPARAMS_N_PCC_DEFAULT                      2

/* USB_EHCI :: HCSPARAMS :: PORT_ROUTING_RULES [07:07] */
#define BCHP_USB_EHCI_HCSPARAMS_PORT_ROUTING_RULES_MASK            0x00000080
#define BCHP_USB_EHCI_HCSPARAMS_PORT_ROUTING_RULES_SHIFT           7
#define BCHP_USB_EHCI_HCSPARAMS_PORT_ROUTING_RULES_DEFAULT         0

/* USB_EHCI :: HCSPARAMS :: reserved2 [06:05] */
#define BCHP_USB_EHCI_HCSPARAMS_reserved2_MASK                     0x00000060
#define BCHP_USB_EHCI_HCSPARAMS_reserved2_SHIFT                    5

/* USB_EHCI :: HCSPARAMS :: PPC [04:04] */
#define BCHP_USB_EHCI_HCSPARAMS_PPC_MASK                           0x00000010
#define BCHP_USB_EHCI_HCSPARAMS_PPC_SHIFT                          4
#define BCHP_USB_EHCI_HCSPARAMS_PPC_DEFAULT                        1

/* USB_EHCI :: HCSPARAMS :: N_PORTS [03:00] */
#define BCHP_USB_EHCI_HCSPARAMS_N_PORTS_MASK                       0x0000000f
#define BCHP_USB_EHCI_HCSPARAMS_N_PORTS_SHIFT                      0
#define BCHP_USB_EHCI_HCSPARAMS_N_PORTS_DEFAULT                    2

/***************************************************************************
 *HCCPARAMS - EHCI Capability Parameter
 ***************************************************************************/
/* USB_EHCI :: HCCPARAMS :: reserved0 [31:16] */
#define BCHP_USB_EHCI_HCCPARAMS_reserved0_MASK                     0xffff0000
#define BCHP_USB_EHCI_HCCPARAMS_reserved0_SHIFT                    16

/* USB_EHCI :: HCCPARAMS :: EECP [15:08] */
#define BCHP_USB_EHCI_HCCPARAMS_EECP_MASK                          0x0000ff00
#define BCHP_USB_EHCI_HCCPARAMS_EECP_SHIFT                         8
#define BCHP_USB_EHCI_HCCPARAMS_EECP_DEFAULT                       160

/* USB_EHCI :: HCCPARAMS :: ISOCHRONOUS_SCHEDULING_THRESHOLD [07:04] */
#define BCHP_USB_EHCI_HCCPARAMS_ISOCHRONOUS_SCHEDULING_THRESHOLD_MASK 0x000000f0
#define BCHP_USB_EHCI_HCCPARAMS_ISOCHRONOUS_SCHEDULING_THRESHOLD_SHIFT 4
#define BCHP_USB_EHCI_HCCPARAMS_ISOCHRONOUS_SCHEDULING_THRESHOLD_DEFAULT 1

/* USB_EHCI :: HCCPARAMS :: reserved1 [03:03] */
#define BCHP_USB_EHCI_HCCPARAMS_reserved1_MASK                     0x00000008
#define BCHP_USB_EHCI_HCCPARAMS_reserved1_SHIFT                    3

/* USB_EHCI :: HCCPARAMS :: ASYNCHRONOUS_SCHEDULE_PARK_CAPABILITY [02:02] */
#define BCHP_USB_EHCI_HCCPARAMS_ASYNCHRONOUS_SCHEDULE_PARK_CAPABILITY_MASK 0x00000004
#define BCHP_USB_EHCI_HCCPARAMS_ASYNCHRONOUS_SCHEDULE_PARK_CAPABILITY_SHIFT 2
#define BCHP_USB_EHCI_HCCPARAMS_ASYNCHRONOUS_SCHEDULE_PARK_CAPABILITY_DEFAULT 0

/* USB_EHCI :: HCCPARAMS :: PROGRAMMABLE_FRAME_LIST_FLAG [01:01] */
#define BCHP_USB_EHCI_HCCPARAMS_PROGRAMMABLE_FRAME_LIST_FLAG_MASK  0x00000002
#define BCHP_USB_EHCI_HCCPARAMS_PROGRAMMABLE_FRAME_LIST_FLAG_SHIFT 1
#define BCHP_USB_EHCI_HCCPARAMS_PROGRAMMABLE_FRAME_LIST_FLAG_DEFAULT 0

/* USB_EHCI :: HCCPARAMS :: SIXTY_FOUR_BIT_ADDRESSING_CAPABILITY [00:00] */
#define BCHP_USB_EHCI_HCCPARAMS_SIXTY_FOUR_BIT_ADDRESSING_CAPABILITY_MASK 0x00000001
#define BCHP_USB_EHCI_HCCPARAMS_SIXTY_FOUR_BIT_ADDRESSING_CAPABILITY_SHIFT 0
#define BCHP_USB_EHCI_HCCPARAMS_SIXTY_FOUR_BIT_ADDRESSING_CAPABILITY_DEFAULT 0

/***************************************************************************
 *USBCMD - USB Command Register
 ***************************************************************************/
/* USB_EHCI :: USBCMD :: reserved0 [31:24] */
#define BCHP_USB_EHCI_USBCMD_reserved0_MASK                        0xff000000
#define BCHP_USB_EHCI_USBCMD_reserved0_SHIFT                       24

/* USB_EHCI :: USBCMD :: INTERRUPT_THRESHOLD_CONTROL [23:16] */
#define BCHP_USB_EHCI_USBCMD_INTERRUPT_THRESHOLD_CONTROL_MASK      0x00ff0000
#define BCHP_USB_EHCI_USBCMD_INTERRUPT_THRESHOLD_CONTROL_SHIFT     16
#define BCHP_USB_EHCI_USBCMD_INTERRUPT_THRESHOLD_CONTROL_DEFAULT   8

/* USB_EHCI :: USBCMD :: reserved1 [15:12] */
#define BCHP_USB_EHCI_USBCMD_reserved1_MASK                        0x0000f000
#define BCHP_USB_EHCI_USBCMD_reserved1_SHIFT                       12

/* USB_EHCI :: USBCMD :: ASYNCHRONOUS_SCHEDULE_PARK_MODE_ENABLE [11:11] */
#define BCHP_USB_EHCI_USBCMD_ASYNCHRONOUS_SCHEDULE_PARK_MODE_ENABLE_MASK 0x00000800
#define BCHP_USB_EHCI_USBCMD_ASYNCHRONOUS_SCHEDULE_PARK_MODE_ENABLE_SHIFT 11
#define BCHP_USB_EHCI_USBCMD_ASYNCHRONOUS_SCHEDULE_PARK_MODE_ENABLE_DEFAULT 0

/* USB_EHCI :: USBCMD :: reserved2 [10:10] */
#define BCHP_USB_EHCI_USBCMD_reserved2_MASK                        0x00000400
#define BCHP_USB_EHCI_USBCMD_reserved2_SHIFT                       10

/* USB_EHCI :: USBCMD :: ASYNCHRONOUS_SCHEDULE_PARK_MODE_COUNT [09:08] */
#define BCHP_USB_EHCI_USBCMD_ASYNCHRONOUS_SCHEDULE_PARK_MODE_COUNT_MASK 0x00000300
#define BCHP_USB_EHCI_USBCMD_ASYNCHRONOUS_SCHEDULE_PARK_MODE_COUNT_SHIFT 8
#define BCHP_USB_EHCI_USBCMD_ASYNCHRONOUS_SCHEDULE_PARK_MODE_COUNT_DEFAULT 0

/* USB_EHCI :: USBCMD :: LIGHT_HOST_CONTROLLER_RESET [07:07] */
#define BCHP_USB_EHCI_USBCMD_LIGHT_HOST_CONTROLLER_RESET_MASK      0x00000080
#define BCHP_USB_EHCI_USBCMD_LIGHT_HOST_CONTROLLER_RESET_SHIFT     7
#define BCHP_USB_EHCI_USBCMD_LIGHT_HOST_CONTROLLER_RESET_DEFAULT   0

/* USB_EHCI :: USBCMD :: INTERRUPT_ON_ASYNC_ADVANCE_DOORBELL [06:06] */
#define BCHP_USB_EHCI_USBCMD_INTERRUPT_ON_ASYNC_ADVANCE_DOORBELL_MASK 0x00000040
#define BCHP_USB_EHCI_USBCMD_INTERRUPT_ON_ASYNC_ADVANCE_DOORBELL_SHIFT 6
#define BCHP_USB_EHCI_USBCMD_INTERRUPT_ON_ASYNC_ADVANCE_DOORBELL_DEFAULT 0

/* USB_EHCI :: USBCMD :: ASYNCHRONOUS_SCHEDULE_ENABLE [05:05] */
#define BCHP_USB_EHCI_USBCMD_ASYNCHRONOUS_SCHEDULE_ENABLE_MASK     0x00000020
#define BCHP_USB_EHCI_USBCMD_ASYNCHRONOUS_SCHEDULE_ENABLE_SHIFT    5
#define BCHP_USB_EHCI_USBCMD_ASYNCHRONOUS_SCHEDULE_ENABLE_DEFAULT  0

/* USB_EHCI :: USBCMD :: PERIODIC_SCHEDULE_ENABLE [04:04] */
#define BCHP_USB_EHCI_USBCMD_PERIODIC_SCHEDULE_ENABLE_MASK         0x00000010
#define BCHP_USB_EHCI_USBCMD_PERIODIC_SCHEDULE_ENABLE_SHIFT        4
#define BCHP_USB_EHCI_USBCMD_PERIODIC_SCHEDULE_ENABLE_DEFAULT      0

/* USB_EHCI :: USBCMD :: FRAME_LIST_SIZE [03:02] */
#define BCHP_USB_EHCI_USBCMD_FRAME_LIST_SIZE_MASK                  0x0000000c
#define BCHP_USB_EHCI_USBCMD_FRAME_LIST_SIZE_SHIFT                 2
#define BCHP_USB_EHCI_USBCMD_FRAME_LIST_SIZE_DEFAULT               0

/* USB_EHCI :: USBCMD :: HCRESET [01:01] */
#define BCHP_USB_EHCI_USBCMD_HCRESET_MASK                          0x00000002
#define BCHP_USB_EHCI_USBCMD_HCRESET_SHIFT                         1
#define BCHP_USB_EHCI_USBCMD_HCRESET_DEFAULT                       0

/* USB_EHCI :: USBCMD :: RUN_STOP [00:00] */
#define BCHP_USB_EHCI_USBCMD_RUN_STOP_MASK                         0x00000001
#define BCHP_USB_EHCI_USBCMD_RUN_STOP_SHIFT                        0
#define BCHP_USB_EHCI_USBCMD_RUN_STOP_DEFAULT                      0

/***************************************************************************
 *USBSTS - USB Status  Register
 ***************************************************************************/
/* USB_EHCI :: USBSTS :: reserved0 [31:16] */
#define BCHP_USB_EHCI_USBSTS_reserved0_MASK                        0xffff0000
#define BCHP_USB_EHCI_USBSTS_reserved0_SHIFT                       16

/* USB_EHCI :: USBSTS :: ASYNCHRONOUS_SCHEDULE_STATUS [15:15] */
#define BCHP_USB_EHCI_USBSTS_ASYNCHRONOUS_SCHEDULE_STATUS_MASK     0x00008000
#define BCHP_USB_EHCI_USBSTS_ASYNCHRONOUS_SCHEDULE_STATUS_SHIFT    15
#define BCHP_USB_EHCI_USBSTS_ASYNCHRONOUS_SCHEDULE_STATUS_DEFAULT  0

/* USB_EHCI :: USBSTS :: PERIODIC_SCHEDULE_STATUS [14:14] */
#define BCHP_USB_EHCI_USBSTS_PERIODIC_SCHEDULE_STATUS_MASK         0x00004000
#define BCHP_USB_EHCI_USBSTS_PERIODIC_SCHEDULE_STATUS_SHIFT        14
#define BCHP_USB_EHCI_USBSTS_PERIODIC_SCHEDULE_STATUS_DEFAULT      0

/* USB_EHCI :: USBSTS :: RECLAMATION [13:13] */
#define BCHP_USB_EHCI_USBSTS_RECLAMATION_MASK                      0x00002000
#define BCHP_USB_EHCI_USBSTS_RECLAMATION_SHIFT                     13
#define BCHP_USB_EHCI_USBSTS_RECLAMATION_DEFAULT                   0

/* USB_EHCI :: USBSTS :: HCHALTED [12:12] */
#define BCHP_USB_EHCI_USBSTS_HCHALTED_MASK                         0x00001000
#define BCHP_USB_EHCI_USBSTS_HCHALTED_SHIFT                        12
#define BCHP_USB_EHCI_USBSTS_HCHALTED_DEFAULT                      1

/* USB_EHCI :: USBSTS :: reserved1 [11:06] */
#define BCHP_USB_EHCI_USBSTS_reserved1_MASK                        0x00000fc0
#define BCHP_USB_EHCI_USBSTS_reserved1_SHIFT                       6

/* USB_EHCI :: USBSTS :: INTERRUPT_ON_ASYNC_ADVANCE [05:05] */
#define BCHP_USB_EHCI_USBSTS_INTERRUPT_ON_ASYNC_ADVANCE_MASK       0x00000020
#define BCHP_USB_EHCI_USBSTS_INTERRUPT_ON_ASYNC_ADVANCE_SHIFT      5
#define BCHP_USB_EHCI_USBSTS_INTERRUPT_ON_ASYNC_ADVANCE_DEFAULT    0

/* USB_EHCI :: USBSTS :: HOST_SYSTEM_ERROR [04:04] */
#define BCHP_USB_EHCI_USBSTS_HOST_SYSTEM_ERROR_MASK                0x00000010
#define BCHP_USB_EHCI_USBSTS_HOST_SYSTEM_ERROR_SHIFT               4
#define BCHP_USB_EHCI_USBSTS_HOST_SYSTEM_ERROR_DEFAULT             0

/* USB_EHCI :: USBSTS :: FRAME_LIST_ROLLOVER [03:03] */
#define BCHP_USB_EHCI_USBSTS_FRAME_LIST_ROLLOVER_MASK              0x00000008
#define BCHP_USB_EHCI_USBSTS_FRAME_LIST_ROLLOVER_SHIFT             3
#define BCHP_USB_EHCI_USBSTS_FRAME_LIST_ROLLOVER_DEFAULT           0

/* USB_EHCI :: USBSTS :: PORT_CHANGE_DETECT [02:02] */
#define BCHP_USB_EHCI_USBSTS_PORT_CHANGE_DETECT_MASK               0x00000004
#define BCHP_USB_EHCI_USBSTS_PORT_CHANGE_DETECT_SHIFT              2
#define BCHP_USB_EHCI_USBSTS_PORT_CHANGE_DETECT_DEFAULT            0

/* USB_EHCI :: USBSTS :: USBERRINT [01:01] */
#define BCHP_USB_EHCI_USBSTS_USBERRINT_MASK                        0x00000002
#define BCHP_USB_EHCI_USBSTS_USBERRINT_SHIFT                       1
#define BCHP_USB_EHCI_USBSTS_USBERRINT_DEFAULT                     0

/* USB_EHCI :: USBSTS :: USBINT [00:00] */
#define BCHP_USB_EHCI_USBSTS_USBINT_MASK                           0x00000001
#define BCHP_USB_EHCI_USBSTS_USBINT_SHIFT                          0
#define BCHP_USB_EHCI_USBSTS_USBINT_DEFAULT                        0

/***************************************************************************
 *USBINTR - USB Interrupt Enable Register
 ***************************************************************************/
/* USB_EHCI :: USBINTR :: reserved0 [31:06] */
#define BCHP_USB_EHCI_USBINTR_reserved0_MASK                       0xffffffc0
#define BCHP_USB_EHCI_USBINTR_reserved0_SHIFT                      6

/* USB_EHCI :: USBINTR :: INTERRUPT_ON_ASYNC_ADVANCE_ENABLE [05:05] */
#define BCHP_USB_EHCI_USBINTR_INTERRUPT_ON_ASYNC_ADVANCE_ENABLE_MASK 0x00000020
#define BCHP_USB_EHCI_USBINTR_INTERRUPT_ON_ASYNC_ADVANCE_ENABLE_SHIFT 5
#define BCHP_USB_EHCI_USBINTR_INTERRUPT_ON_ASYNC_ADVANCE_ENABLE_DEFAULT 0

/* USB_EHCI :: USBINTR :: HOST_SYSTEM_ERROR_ENABLE [04:04] */
#define BCHP_USB_EHCI_USBINTR_HOST_SYSTEM_ERROR_ENABLE_MASK        0x00000010
#define BCHP_USB_EHCI_USBINTR_HOST_SYSTEM_ERROR_ENABLE_SHIFT       4
#define BCHP_USB_EHCI_USBINTR_HOST_SYSTEM_ERROR_ENABLE_DEFAULT     0

/* USB_EHCI :: USBINTR :: FRAME_LIST_ROLLOVER_ENABLE [03:03] */
#define BCHP_USB_EHCI_USBINTR_FRAME_LIST_ROLLOVER_ENABLE_MASK      0x00000008
#define BCHP_USB_EHCI_USBINTR_FRAME_LIST_ROLLOVER_ENABLE_SHIFT     3
#define BCHP_USB_EHCI_USBINTR_FRAME_LIST_ROLLOVER_ENABLE_DEFAULT   0

/* USB_EHCI :: USBINTR :: PORT_CHANGE_INTERRUPT_ENABLE [02:02] */
#define BCHP_USB_EHCI_USBINTR_PORT_CHANGE_INTERRUPT_ENABLE_MASK    0x00000004
#define BCHP_USB_EHCI_USBINTR_PORT_CHANGE_INTERRUPT_ENABLE_SHIFT   2
#define BCHP_USB_EHCI_USBINTR_PORT_CHANGE_INTERRUPT_ENABLE_DEFAULT 0

/* USB_EHCI :: USBINTR :: USB_ERROR_INTERRUPT_ENABLE [01:01] */
#define BCHP_USB_EHCI_USBINTR_USB_ERROR_INTERRUPT_ENABLE_MASK      0x00000002
#define BCHP_USB_EHCI_USBINTR_USB_ERROR_INTERRUPT_ENABLE_SHIFT     1
#define BCHP_USB_EHCI_USBINTR_USB_ERROR_INTERRUPT_ENABLE_DEFAULT   0

/* USB_EHCI :: USBINTR :: USB_INTERRUPT_ENABLE [00:00] */
#define BCHP_USB_EHCI_USBINTR_USB_INTERRUPT_ENABLE_MASK            0x00000001
#define BCHP_USB_EHCI_USBINTR_USB_INTERRUPT_ENABLE_SHIFT           0
#define BCHP_USB_EHCI_USBINTR_USB_INTERRUPT_ENABLE_DEFAULT         0

/***************************************************************************
 *FRINDEX - USB Frame Index Register
 ***************************************************************************/
/* USB_EHCI :: FRINDEX :: reserved0 [31:14] */
#define BCHP_USB_EHCI_FRINDEX_reserved0_MASK                       0xffffc000
#define BCHP_USB_EHCI_FRINDEX_reserved0_SHIFT                      14

/* USB_EHCI :: FRINDEX :: FRAME_INDEX [13:00] */
#define BCHP_USB_EHCI_FRINDEX_FRAME_INDEX_MASK                     0x00003fff
#define BCHP_USB_EHCI_FRINDEX_FRAME_INDEX_SHIFT                    0
#define BCHP_USB_EHCI_FRINDEX_FRAME_INDEX_DEFAULT                  0

/***************************************************************************
 *PERIODICLISTBASE - Periodic Frame List Base Address Register
 ***************************************************************************/
/* USB_EHCI :: PERIODICLISTBASE :: BASE_ADDRESS_LOW [31:12] */
#define BCHP_USB_EHCI_PERIODICLISTBASE_BASE_ADDRESS_LOW_MASK       0xfffff000
#define BCHP_USB_EHCI_PERIODICLISTBASE_BASE_ADDRESS_LOW_SHIFT      12
#define BCHP_USB_EHCI_PERIODICLISTBASE_BASE_ADDRESS_LOW_DEFAULT    0

/* USB_EHCI :: PERIODICLISTBASE :: reserved0 [11:00] */
#define BCHP_USB_EHCI_PERIODICLISTBASE_reserved0_MASK              0x00000fff
#define BCHP_USB_EHCI_PERIODICLISTBASE_reserved0_SHIFT             0

/***************************************************************************
 *ASYNCLISTADDR - Asynchronous List Address
 ***************************************************************************/
/* USB_EHCI :: ASYNCLISTADDR :: LPL [31:05] */
#define BCHP_USB_EHCI_ASYNCLISTADDR_LPL_MASK                       0xffffffe0
#define BCHP_USB_EHCI_ASYNCLISTADDR_LPL_SHIFT                      5
#define BCHP_USB_EHCI_ASYNCLISTADDR_LPL_DEFAULT                    0

/* USB_EHCI :: ASYNCLISTADDR :: reserved0 [04:00] */
#define BCHP_USB_EHCI_ASYNCLISTADDR_reserved0_MASK                 0x0000001f
#define BCHP_USB_EHCI_ASYNCLISTADDR_reserved0_SHIFT                0

/***************************************************************************
 *CONFIGFLAG - Configured Flag Register
 ***************************************************************************/
/* USB_EHCI :: CONFIGFLAG :: reserved0 [31:01] */
#define BCHP_USB_EHCI_CONFIGFLAG_reserved0_MASK                    0xfffffffe
#define BCHP_USB_EHCI_CONFIGFLAG_reserved0_SHIFT                   1

/* USB_EHCI :: CONFIGFLAG :: CONFIGURE_FLAG [00:00] */
#define BCHP_USB_EHCI_CONFIGFLAG_CONFIGURE_FLAG_MASK               0x00000001
#define BCHP_USB_EHCI_CONFIGFLAG_CONFIGURE_FLAG_SHIFT              0
#define BCHP_USB_EHCI_CONFIGFLAG_CONFIGURE_FLAG_DEFAULT            0

/***************************************************************************
 *PORTSC_0 - Port Status/Control Register for Port 0
 ***************************************************************************/
/* USB_EHCI :: PORTSC_0 :: reserved0 [31:23] */
#define BCHP_USB_EHCI_PORTSC_0_reserved0_MASK                      0xff800000
#define BCHP_USB_EHCI_PORTSC_0_reserved0_SHIFT                     23

/* USB_EHCI :: PORTSC_0 :: WKOC_E [22:22] */
#define BCHP_USB_EHCI_PORTSC_0_WKOC_E_MASK                         0x00400000
#define BCHP_USB_EHCI_PORTSC_0_WKOC_E_SHIFT                        22
#define BCHP_USB_EHCI_PORTSC_0_WKOC_E_DEFAULT                      0

/* USB_EHCI :: PORTSC_0 :: WKDSCNNT_E [21:21] */
#define BCHP_USB_EHCI_PORTSC_0_WKDSCNNT_E_MASK                     0x00200000
#define BCHP_USB_EHCI_PORTSC_0_WKDSCNNT_E_SHIFT                    21
#define BCHP_USB_EHCI_PORTSC_0_WKDSCNNT_E_DEFAULT                  0

/* USB_EHCI :: PORTSC_0 :: WKCNNT_E [20:20] */
#define BCHP_USB_EHCI_PORTSC_0_WKCNNT_E_MASK                       0x00100000
#define BCHP_USB_EHCI_PORTSC_0_WKCNNT_E_SHIFT                      20
#define BCHP_USB_EHCI_PORTSC_0_WKCNNT_E_DEFAULT                    0

/* USB_EHCI :: PORTSC_0 :: PORT_TEST_CONTROL [19:16] */
#define BCHP_USB_EHCI_PORTSC_0_PORT_TEST_CONTROL_MASK              0x000f0000
#define BCHP_USB_EHCI_PORTSC_0_PORT_TEST_CONTROL_SHIFT             16
#define BCHP_USB_EHCI_PORTSC_0_PORT_TEST_CONTROL_DEFAULT           0

/* USB_EHCI :: PORTSC_0 :: PORT_INDICATOR_CONTROL [15:14] */
#define BCHP_USB_EHCI_PORTSC_0_PORT_INDICATOR_CONTROL_MASK         0x0000c000
#define BCHP_USB_EHCI_PORTSC_0_PORT_INDICATOR_CONTROL_SHIFT        14
#define BCHP_USB_EHCI_PORTSC_0_PORT_INDICATOR_CONTROL_DEFAULT      0

/* USB_EHCI :: PORTSC_0 :: PORT_OWNER [13:13] */
#define BCHP_USB_EHCI_PORTSC_0_PORT_OWNER_MASK                     0x00002000
#define BCHP_USB_EHCI_PORTSC_0_PORT_OWNER_SHIFT                    13
#define BCHP_USB_EHCI_PORTSC_0_PORT_OWNER_DEFAULT                  1

/* USB_EHCI :: PORTSC_0 :: PP [12:12] */
#define BCHP_USB_EHCI_PORTSC_0_PP_MASK                             0x00001000
#define BCHP_USB_EHCI_PORTSC_0_PP_SHIFT                            12
#define BCHP_USB_EHCI_PORTSC_0_PP_DEFAULT                          0

/* USB_EHCI :: PORTSC_0 :: LINE_STATUS [11:10] */
#define BCHP_USB_EHCI_PORTSC_0_LINE_STATUS_MASK                    0x00000c00
#define BCHP_USB_EHCI_PORTSC_0_LINE_STATUS_SHIFT                   10
#define BCHP_USB_EHCI_PORTSC_0_LINE_STATUS_DEFAULT                 0

/* USB_EHCI :: PORTSC_0 :: reserved1 [09:09] */
#define BCHP_USB_EHCI_PORTSC_0_reserved1_MASK                      0x00000200
#define BCHP_USB_EHCI_PORTSC_0_reserved1_SHIFT                     9

/* USB_EHCI :: PORTSC_0 :: PORT_RESET [08:08] */
#define BCHP_USB_EHCI_PORTSC_0_PORT_RESET_MASK                     0x00000100
#define BCHP_USB_EHCI_PORTSC_0_PORT_RESET_SHIFT                    8
#define BCHP_USB_EHCI_PORTSC_0_PORT_RESET_DEFAULT                  0

/* USB_EHCI :: PORTSC_0 :: SUSPEND [07:07] */
#define BCHP_USB_EHCI_PORTSC_0_SUSPEND_MASK                        0x00000080
#define BCHP_USB_EHCI_PORTSC_0_SUSPEND_SHIFT                       7
#define BCHP_USB_EHCI_PORTSC_0_SUSPEND_DEFAULT                     0

/* USB_EHCI :: PORTSC_0 :: FORCE_PORT_RESUME [06:06] */
#define BCHP_USB_EHCI_PORTSC_0_FORCE_PORT_RESUME_MASK              0x00000040
#define BCHP_USB_EHCI_PORTSC_0_FORCE_PORT_RESUME_SHIFT             6
#define BCHP_USB_EHCI_PORTSC_0_FORCE_PORT_RESUME_DEFAULT           0

/* USB_EHCI :: PORTSC_0 :: OVER_CURRENT_CHANGE [05:05] */
#define BCHP_USB_EHCI_PORTSC_0_OVER_CURRENT_CHANGE_MASK            0x00000020
#define BCHP_USB_EHCI_PORTSC_0_OVER_CURRENT_CHANGE_SHIFT           5
#define BCHP_USB_EHCI_PORTSC_0_OVER_CURRENT_CHANGE_DEFAULT         0

/* USB_EHCI :: PORTSC_0 :: OVER_CURRENT_ACTIVE [04:04] */
#define BCHP_USB_EHCI_PORTSC_0_OVER_CURRENT_ACTIVE_MASK            0x00000010
#define BCHP_USB_EHCI_PORTSC_0_OVER_CURRENT_ACTIVE_SHIFT           4
#define BCHP_USB_EHCI_PORTSC_0_OVER_CURRENT_ACTIVE_DEFAULT         0

/* USB_EHCI :: PORTSC_0 :: PORT_ENABLE_DISABLE_CHANGE [03:03] */
#define BCHP_USB_EHCI_PORTSC_0_PORT_ENABLE_DISABLE_CHANGE_MASK     0x00000008
#define BCHP_USB_EHCI_PORTSC_0_PORT_ENABLE_DISABLE_CHANGE_SHIFT    3
#define BCHP_USB_EHCI_PORTSC_0_PORT_ENABLE_DISABLE_CHANGE_DEFAULT  0

/* USB_EHCI :: PORTSC_0 :: PORT_ENABLED_DISABLED [02:02] */
#define BCHP_USB_EHCI_PORTSC_0_PORT_ENABLED_DISABLED_MASK          0x00000004
#define BCHP_USB_EHCI_PORTSC_0_PORT_ENABLED_DISABLED_SHIFT         2
#define BCHP_USB_EHCI_PORTSC_0_PORT_ENABLED_DISABLED_DEFAULT       0

/* USB_EHCI :: PORTSC_0 :: CONNECT_STATUS_CHANGE [01:01] */
#define BCHP_USB_EHCI_PORTSC_0_CONNECT_STATUS_CHANGE_MASK          0x00000002
#define BCHP_USB_EHCI_PORTSC_0_CONNECT_STATUS_CHANGE_SHIFT         1
#define BCHP_USB_EHCI_PORTSC_0_CONNECT_STATUS_CHANGE_DEFAULT       0

/* USB_EHCI :: PORTSC_0 :: CURRENT_CONNECT_STATUS [00:00] */
#define BCHP_USB_EHCI_PORTSC_0_CURRENT_CONNECT_STATUS_MASK         0x00000001
#define BCHP_USB_EHCI_PORTSC_0_CURRENT_CONNECT_STATUS_SHIFT        0
#define BCHP_USB_EHCI_PORTSC_0_CURRENT_CONNECT_STATUS_DEFAULT      0

/***************************************************************************
 *PORTSC_1 - Port Status/Control Register for Port 1
 ***************************************************************************/
/* USB_EHCI :: PORTSC_1 :: reserved0 [31:23] */
#define BCHP_USB_EHCI_PORTSC_1_reserved0_MASK                      0xff800000
#define BCHP_USB_EHCI_PORTSC_1_reserved0_SHIFT                     23

/* USB_EHCI :: PORTSC_1 :: WKOC_E [22:22] */
#define BCHP_USB_EHCI_PORTSC_1_WKOC_E_MASK                         0x00400000
#define BCHP_USB_EHCI_PORTSC_1_WKOC_E_SHIFT                        22
#define BCHP_USB_EHCI_PORTSC_1_WKOC_E_DEFAULT                      0

/* USB_EHCI :: PORTSC_1 :: WKDSCNNT_E [21:21] */
#define BCHP_USB_EHCI_PORTSC_1_WKDSCNNT_E_MASK                     0x00200000
#define BCHP_USB_EHCI_PORTSC_1_WKDSCNNT_E_SHIFT                    21
#define BCHP_USB_EHCI_PORTSC_1_WKDSCNNT_E_DEFAULT                  0

/* USB_EHCI :: PORTSC_1 :: WKCNNT_E [20:20] */
#define BCHP_USB_EHCI_PORTSC_1_WKCNNT_E_MASK                       0x00100000
#define BCHP_USB_EHCI_PORTSC_1_WKCNNT_E_SHIFT                      20
#define BCHP_USB_EHCI_PORTSC_1_WKCNNT_E_DEFAULT                    0

/* USB_EHCI :: PORTSC_1 :: PORT_TEST_CONTROL [19:16] */
#define BCHP_USB_EHCI_PORTSC_1_PORT_TEST_CONTROL_MASK              0x000f0000
#define BCHP_USB_EHCI_PORTSC_1_PORT_TEST_CONTROL_SHIFT             16
#define BCHP_USB_EHCI_PORTSC_1_PORT_TEST_CONTROL_DEFAULT           0

/* USB_EHCI :: PORTSC_1 :: PORT_INDICATOR_CONTROL [15:14] */
#define BCHP_USB_EHCI_PORTSC_1_PORT_INDICATOR_CONTROL_MASK         0x0000c000
#define BCHP_USB_EHCI_PORTSC_1_PORT_INDICATOR_CONTROL_SHIFT        14
#define BCHP_USB_EHCI_PORTSC_1_PORT_INDICATOR_CONTROL_DEFAULT      0

/* USB_EHCI :: PORTSC_1 :: PORT_OWNER [13:13] */
#define BCHP_USB_EHCI_PORTSC_1_PORT_OWNER_MASK                     0x00002000
#define BCHP_USB_EHCI_PORTSC_1_PORT_OWNER_SHIFT                    13
#define BCHP_USB_EHCI_PORTSC_1_PORT_OWNER_DEFAULT                  1

/* USB_EHCI :: PORTSC_1 :: PP [12:12] */
#define BCHP_USB_EHCI_PORTSC_1_PP_MASK                             0x00001000
#define BCHP_USB_EHCI_PORTSC_1_PP_SHIFT                            12
#define BCHP_USB_EHCI_PORTSC_1_PP_DEFAULT                          0

/* USB_EHCI :: PORTSC_1 :: LINE_STATUS [11:10] */
#define BCHP_USB_EHCI_PORTSC_1_LINE_STATUS_MASK                    0x00000c00
#define BCHP_USB_EHCI_PORTSC_1_LINE_STATUS_SHIFT                   10
#define BCHP_USB_EHCI_PORTSC_1_LINE_STATUS_DEFAULT                 0

/* USB_EHCI :: PORTSC_1 :: reserved1 [09:09] */
#define BCHP_USB_EHCI_PORTSC_1_reserved1_MASK                      0x00000200
#define BCHP_USB_EHCI_PORTSC_1_reserved1_SHIFT                     9

/* USB_EHCI :: PORTSC_1 :: PORT_RESET [08:08] */
#define BCHP_USB_EHCI_PORTSC_1_PORT_RESET_MASK                     0x00000100
#define BCHP_USB_EHCI_PORTSC_1_PORT_RESET_SHIFT                    8
#define BCHP_USB_EHCI_PORTSC_1_PORT_RESET_DEFAULT                  0

/* USB_EHCI :: PORTSC_1 :: SUSPEND [07:07] */
#define BCHP_USB_EHCI_PORTSC_1_SUSPEND_MASK                        0x00000080
#define BCHP_USB_EHCI_PORTSC_1_SUSPEND_SHIFT                       7
#define BCHP_USB_EHCI_PORTSC_1_SUSPEND_DEFAULT                     0

/* USB_EHCI :: PORTSC_1 :: FORCE_PORT_RESUME [06:06] */
#define BCHP_USB_EHCI_PORTSC_1_FORCE_PORT_RESUME_MASK              0x00000040
#define BCHP_USB_EHCI_PORTSC_1_FORCE_PORT_RESUME_SHIFT             6
#define BCHP_USB_EHCI_PORTSC_1_FORCE_PORT_RESUME_DEFAULT           0

/* USB_EHCI :: PORTSC_1 :: OVER_CURRENT_CHANGE [05:05] */
#define BCHP_USB_EHCI_PORTSC_1_OVER_CURRENT_CHANGE_MASK            0x00000020
#define BCHP_USB_EHCI_PORTSC_1_OVER_CURRENT_CHANGE_SHIFT           5
#define BCHP_USB_EHCI_PORTSC_1_OVER_CURRENT_CHANGE_DEFAULT         0

/* USB_EHCI :: PORTSC_1 :: OVER_CURRENT_ACTIVE [04:04] */
#define BCHP_USB_EHCI_PORTSC_1_OVER_CURRENT_ACTIVE_MASK            0x00000010
#define BCHP_USB_EHCI_PORTSC_1_OVER_CURRENT_ACTIVE_SHIFT           4
#define BCHP_USB_EHCI_PORTSC_1_OVER_CURRENT_ACTIVE_DEFAULT         0

/* USB_EHCI :: PORTSC_1 :: PORT_ENABLE_DISABLE_CHANGE [03:03] */
#define BCHP_USB_EHCI_PORTSC_1_PORT_ENABLE_DISABLE_CHANGE_MASK     0x00000008
#define BCHP_USB_EHCI_PORTSC_1_PORT_ENABLE_DISABLE_CHANGE_SHIFT    3
#define BCHP_USB_EHCI_PORTSC_1_PORT_ENABLE_DISABLE_CHANGE_DEFAULT  0

/* USB_EHCI :: PORTSC_1 :: PORT_ENABLED_DISABLED [02:02] */
#define BCHP_USB_EHCI_PORTSC_1_PORT_ENABLED_DISABLED_MASK          0x00000004
#define BCHP_USB_EHCI_PORTSC_1_PORT_ENABLED_DISABLED_SHIFT         2
#define BCHP_USB_EHCI_PORTSC_1_PORT_ENABLED_DISABLED_DEFAULT       0

/* USB_EHCI :: PORTSC_1 :: CONNECT_STATUS_CHANGE [01:01] */
#define BCHP_USB_EHCI_PORTSC_1_CONNECT_STATUS_CHANGE_MASK          0x00000002
#define BCHP_USB_EHCI_PORTSC_1_CONNECT_STATUS_CHANGE_SHIFT         1
#define BCHP_USB_EHCI_PORTSC_1_CONNECT_STATUS_CHANGE_DEFAULT       0

/* USB_EHCI :: PORTSC_1 :: CURRENT_CONNECT_STATUS [00:00] */
#define BCHP_USB_EHCI_PORTSC_1_CURRENT_CONNECT_STATUS_MASK         0x00000001
#define BCHP_USB_EHCI_PORTSC_1_CURRENT_CONNECT_STATUS_SHIFT        0
#define BCHP_USB_EHCI_PORTSC_1_CURRENT_CONNECT_STATUS_DEFAULT      0

/***************************************************************************
 *INSNREG00 - Microframe Base Value Register
 ***************************************************************************/
/* USB_EHCI :: INSNREG00 :: INSNREG00 [31:00] */
#define BCHP_USB_EHCI_INSNREG00_INSNREG00_MASK                     0xffffffff
#define BCHP_USB_EHCI_INSNREG00_INSNREG00_SHIFT                    0
#define BCHP_USB_EHCI_INSNREG00_INSNREG00_DEFAULT                  0

/***************************************************************************
 *INSNREG01 - Packet Buffer OUT/IN Threshold Register
 ***************************************************************************/
/* USB_EHCI :: INSNREG01 :: INSNREG01 [31:00] */
#define BCHP_USB_EHCI_INSNREG01_INSNREG01_MASK                     0xffffffff
#define BCHP_USB_EHCI_INSNREG01_INSNREG01_SHIFT                    0
#define BCHP_USB_EHCI_INSNREG01_INSNREG01_DEFAULT                  2097184

/***************************************************************************
 *INSNREG02 - Packet Buffer Depth Register
 ***************************************************************************/
/* USB_EHCI :: INSNREG02 :: INSNREG02 [31:00] */
#define BCHP_USB_EHCI_INSNREG02_INSNREG02_MASK                     0xffffffff
#define BCHP_USB_EHCI_INSNREG02_INSNREG02_SHIFT                    0
#define BCHP_USB_EHCI_INSNREG02_INSNREG02_DEFAULT                  128

/***************************************************************************
 *INSNREG03 - Break Memory Transfer Register
 ***************************************************************************/
/* USB_EHCI :: INSNREG03 :: INSNREG03 [31:00] */
#define BCHP_USB_EHCI_INSNREG03_INSNREG03_MASK                     0xffffffff
#define BCHP_USB_EHCI_INSNREG03_INSNREG03_SHIFT                    0
#define BCHP_USB_EHCI_INSNREG03_INSNREG03_DEFAULT                  0

/***************************************************************************
 *INSNREG04 - Debug Register
 ***************************************************************************/
/* USB_EHCI :: INSNREG04 :: INSNREG04 [31:00] */
#define BCHP_USB_EHCI_INSNREG04_INSNREG04_MASK                     0xffffffff
#define BCHP_USB_EHCI_INSNREG04_INSNREG04_SHIFT                    0
#define BCHP_USB_EHCI_INSNREG04_INSNREG04_DEFAULT                  0

/***************************************************************************
 *INSNREG05 - UTMI Control and Status Register
 ***************************************************************************/
/* USB_EHCI :: INSNREG05 :: INSNREG05 [31:00] */
#define BCHP_USB_EHCI_INSNREG05_INSNREG05_MASK                     0xffffffff
#define BCHP_USB_EHCI_INSNREG05_INSNREG05_SHIFT                    0
#define BCHP_USB_EHCI_INSNREG05_INSNREG05_DEFAULT                  4096

#endif /* #ifndef BCHP_USB_EHCI_H__ */

/* End of File */
