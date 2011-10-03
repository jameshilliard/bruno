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
 * Date:           Generated on         Tue May 31 13:27:13 2011
 *                 MD5 Checksum         3d981376c3cc0d4c52a81813284994b8
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/7344/rdb/a0/bchp_mem_dma_0.h $
 * 
 * Hydra_Software_Devel/2   5/31/11 2:26p albertl
 * SW7344-40: Updated to match RDB.
 *
 ***************************************************************************/

#ifndef BCHP_MEM_DMA_0_H__
#define BCHP_MEM_DMA_0_H__

/***************************************************************************
 *MEM_DMA_0 - MEM_DMA_0 Registers
 ***************************************************************************/
#define BCHP_MEM_DMA_0_REVISION                  0x000a0200 /* MEM_DMA REVISION */
#define BCHP_MEM_DMA_0_FIRST_DESC                0x000a0204 /* MEM_DMA First Descriptor Address Register */
#define BCHP_MEM_DMA_0_CTRL                      0x000a0208 /* MEM_DMA Control Register */
#define BCHP_MEM_DMA_0_WAKE_CTRL                 0x000a020c /* MEM_DMA Wake Control Register */
#define BCHP_MEM_DMA_0_STATUS                    0x000a0214 /* MEM_DMA Status Register */
#define BCHP_MEM_DMA_0_CUR_DESC                  0x000a0218 /* MEM_DMA Current Descriptor Address Register */
#define BCHP_MEM_DMA_0_CUR_BYTE                  0x000a021c /* MEM_DMA Current Byte Count Register */
#define BCHP_MEM_DMA_0_SCRATCH                   0x000a0224 /* MEM_DMA Scratch Register */

/***************************************************************************
 *REVISION - MEM_DMA REVISION
 ***************************************************************************/
/* MEM_DMA_0 :: REVISION :: reserved0 [31:16] */
#define BCHP_MEM_DMA_0_REVISION_reserved0_MASK                     0xffff0000
#define BCHP_MEM_DMA_0_REVISION_reserved0_SHIFT                    16

/* MEM_DMA_0 :: REVISION :: MAJOR [15:08] */
#define BCHP_MEM_DMA_0_REVISION_MAJOR_MASK                         0x0000ff00
#define BCHP_MEM_DMA_0_REVISION_MAJOR_SHIFT                        8
#define BCHP_MEM_DMA_0_REVISION_MAJOR_DEFAULT                      2

/* MEM_DMA_0 :: REVISION :: MINOR [07:00] */
#define BCHP_MEM_DMA_0_REVISION_MINOR_MASK                         0x000000ff
#define BCHP_MEM_DMA_0_REVISION_MINOR_SHIFT                        0
#define BCHP_MEM_DMA_0_REVISION_MINOR_DEFAULT                      10

/***************************************************************************
 *FIRST_DESC - MEM_DMA First Descriptor Address Register
 ***************************************************************************/
/* MEM_DMA_0 :: FIRST_DESC :: ADDR [31:00] */
#define BCHP_MEM_DMA_0_FIRST_DESC_ADDR_MASK                        0xffffffff
#define BCHP_MEM_DMA_0_FIRST_DESC_ADDR_SHIFT                       0
#define BCHP_MEM_DMA_0_FIRST_DESC_ADDR_DEFAULT                     0

/***************************************************************************
 *CTRL - MEM_DMA Control Register
 ***************************************************************************/
/* MEM_DMA_0 :: CTRL :: reserved0 [31:01] */
#define BCHP_MEM_DMA_0_CTRL_reserved0_MASK                         0xfffffffe
#define BCHP_MEM_DMA_0_CTRL_reserved0_SHIFT                        1

/* MEM_DMA_0 :: CTRL :: RUN [00:00] */
#define BCHP_MEM_DMA_0_CTRL_RUN_MASK                               0x00000001
#define BCHP_MEM_DMA_0_CTRL_RUN_SHIFT                              0
#define BCHP_MEM_DMA_0_CTRL_RUN_DEFAULT                            0

/***************************************************************************
 *WAKE_CTRL - MEM_DMA Wake Control Register
 ***************************************************************************/
/* MEM_DMA_0 :: WAKE_CTRL :: reserved0 [31:02] */
#define BCHP_MEM_DMA_0_WAKE_CTRL_reserved0_MASK                    0xfffffffc
#define BCHP_MEM_DMA_0_WAKE_CTRL_reserved0_SHIFT                   2

/* MEM_DMA_0 :: WAKE_CTRL :: WAKE_MODE [01:01] */
#define BCHP_MEM_DMA_0_WAKE_CTRL_WAKE_MODE_MASK                    0x00000002
#define BCHP_MEM_DMA_0_WAKE_CTRL_WAKE_MODE_SHIFT                   1
#define BCHP_MEM_DMA_0_WAKE_CTRL_WAKE_MODE_DEFAULT                 0

/* MEM_DMA_0 :: WAKE_CTRL :: WAKE [00:00] */
#define BCHP_MEM_DMA_0_WAKE_CTRL_WAKE_MASK                         0x00000001
#define BCHP_MEM_DMA_0_WAKE_CTRL_WAKE_SHIFT                        0
#define BCHP_MEM_DMA_0_WAKE_CTRL_WAKE_DEFAULT                      0

/***************************************************************************
 *STATUS - MEM_DMA Status Register
 ***************************************************************************/
/* MEM_DMA_0 :: STATUS :: reserved0 [31:02] */
#define BCHP_MEM_DMA_0_STATUS_reserved0_MASK                       0xfffffffc
#define BCHP_MEM_DMA_0_STATUS_reserved0_SHIFT                      2

/* MEM_DMA_0 :: STATUS :: DMA_STATUS [01:00] */
#define BCHP_MEM_DMA_0_STATUS_DMA_STATUS_MASK                      0x00000003
#define BCHP_MEM_DMA_0_STATUS_DMA_STATUS_SHIFT                     0
#define BCHP_MEM_DMA_0_STATUS_DMA_STATUS_DEFAULT                   0
#define BCHP_MEM_DMA_0_STATUS_DMA_STATUS_Idle                      0
#define BCHP_MEM_DMA_0_STATUS_DMA_STATUS_Busy                      1
#define BCHP_MEM_DMA_0_STATUS_DMA_STATUS_Sleep                     2
#define BCHP_MEM_DMA_0_STATUS_DMA_STATUS_Reserved                  3

/***************************************************************************
 *CUR_DESC - MEM_DMA Current Descriptor Address Register
 ***************************************************************************/
/* MEM_DMA_0 :: CUR_DESC :: ADDR [31:00] */
#define BCHP_MEM_DMA_0_CUR_DESC_ADDR_MASK                          0xffffffff
#define BCHP_MEM_DMA_0_CUR_DESC_ADDR_SHIFT                         0
#define BCHP_MEM_DMA_0_CUR_DESC_ADDR_DEFAULT                       0

/***************************************************************************
 *CUR_BYTE - MEM_DMA Current Byte Count Register
 ***************************************************************************/
/* MEM_DMA_0 :: CUR_BYTE :: reserved0 [31:25] */
#define BCHP_MEM_DMA_0_CUR_BYTE_reserved0_MASK                     0xfe000000
#define BCHP_MEM_DMA_0_CUR_BYTE_reserved0_SHIFT                    25

/* MEM_DMA_0 :: CUR_BYTE :: COUNT [24:00] */
#define BCHP_MEM_DMA_0_CUR_BYTE_COUNT_MASK                         0x01ffffff
#define BCHP_MEM_DMA_0_CUR_BYTE_COUNT_SHIFT                        0
#define BCHP_MEM_DMA_0_CUR_BYTE_COUNT_DEFAULT                      0

/***************************************************************************
 *SCRATCH - MEM_DMA Scratch Register
 ***************************************************************************/
/* MEM_DMA_0 :: SCRATCH :: SCRATCH_BIT [31:00] */
#define BCHP_MEM_DMA_0_SCRATCH_SCRATCH_BIT_MASK                    0xffffffff
#define BCHP_MEM_DMA_0_SCRATCH_SCRATCH_BIT_SHIFT                   0
#define BCHP_MEM_DMA_0_SCRATCH_SCRATCH_BIT_DEFAULT                 0

#endif /* #ifndef BCHP_MEM_DMA_0_H__ */

/* End of File */
