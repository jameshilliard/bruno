/*
 * Copyright (C) 2011 Broadcom Corporation
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
 */
#include <stdarg.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include <asm/brcmstb/brcmstb.h>

#if 0
#define DBG	printk
#else
#define DBG(...)		do { } while (0)
#endif

#ifdef CONFIG_BRCM_HAS_STANDBY

#define MAX_CLIENT_INFO_NUM		128
#define MAX_DDR_PARAMS_NUM		16
#define MAX_DDR_APHY_PARAMS_NUM		16
#define MAX_ARB_PARAMS_NUM		2

struct memc_config {
	u32	client_info[MAX_CLIENT_INFO_NUM];
	u32	ddr23_aphy_params[MAX_DDR_APHY_PARAMS_NUM];
	u32	ddr_params[MAX_DDR_PARAMS_NUM];
	u32	arb_params[MAX_ARB_PARAMS_NUM];

	u32	vcdl[4];
	int	shmoo_value[8];

	int	shmoo_valid:1;
	int	valid:1;

};

static struct memc_config __maybe_unused memc1_config;

#if defined(CONFIG_BCM7420)

static void bcm7420_memc1_sspd_control(int enable)
{
	if (enable) {
		BDEV_WR_F_RB(MEMC_DDR_1_SSPD_CMD, SSPD, 1);
		while (!BDEV_RD_F(MEMC_DDR_1_POWER_DOWN_STATUS, SSPD))
			udelay(1);
	} else {
		BDEV_WR_F_RB(MEMC_DDR_1_SSPD_CMD, SSPD, 0);
		while (BDEV_RD_F(MEMC_DDR_1_POWER_DOWN_STATUS, SSPD))
			udelay(1);
	}
}

static void bcm7420_memc1_suspend(int mode)
{
	DBG("%s\n", __func__);

	if (!memc1_config.shmoo_valid) {
		memc1_config.shmoo_value[0] =
			BDEV_RD(BCHP_MEMC_GEN_1_MSA_WR_DATA0);
		memc1_config.shmoo_value[1] =
			BDEV_RD(BCHP_MEMC_GEN_1_MSA_WR_DATA1);
		memc1_config.shmoo_value[2] =
			BDEV_RD(BCHP_MEMC_GEN_1_MSA_WR_DATA2);
		memc1_config.shmoo_value[3] =
			BDEV_RD(BCHP_MEMC_GEN_1_MSA_WR_DATA3);
		memc1_config.shmoo_value[4] =
			BDEV_RD(BCHP_MEMC_GEN_1_MSA_WR_DATA4);
		memc1_config.shmoo_value[5] =
			BDEV_RD(BCHP_MEMC_GEN_1_MSA_WR_DATA5);
		memc1_config.shmoo_value[6] =
			BDEV_RD(BCHP_MEMC_GEN_1_MSA_WR_DATA6);
		memc1_config.shmoo_value[7] =
			BDEV_RD(BCHP_MEMC_MISC_1_SCRATCH_0);
		memc1_config.shmoo_valid = 1;
		DBG("%s: SHMOO values saved\n", __func__);
	}

	/* save VCDL values */
	memc1_config.vcdl[0] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_WL0_1_BYTE0_VCDL_PHASE_CNTL);
	memc1_config.vcdl[1] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_WL0_1_BYTE1_VCDL_PHASE_CNTL);
	memc1_config.vcdl[2] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_WL1_1_BYTE0_VCDL_PHASE_CNTL);
	memc1_config.vcdl[3] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_WL1_1_BYTE1_VCDL_PHASE_CNTL);

	/* MEMC1 */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL,
		DEVCLK_OFF_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL,
		HIZ_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_POWERDOWN,
		PLLCLKS_OFF_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_POWERDOWN,
		LOWPWR_EN, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL0_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL1_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL0_1_WORDSLICE_CNTRL_1,
		PWRDN_DLL_ON_SELFREF, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL1_1_WORDSLICE_CNTRL_1,
		PWRDN_DLL_ON_SELFREF, 1);

	bcm7420_memc1_sspd_control(1);

	if (mode) {
		BDEV_WR_F_RB(MEMC_MISC_1_SOFT_RESET,
			MEMC_DRAM_INIT, 1);
		BDEV_WR_F_RB(MEMC_MISC_1_SOFT_RESET,
			MEMC_CORE, 1);
		BDEV_WR_F_RB(MEMC_DDR_1_DRAM_INIT_CNTRL,
			DDR3_INIT_MODE, 1);
		mdelay(1);
		printk(KERN_DEBUG "memc1: powered down\n");
	} else
		printk(KERN_DEBUG "memc1: suspended\n");

	/* Power down LDOs */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_PLL_CTRL1_REG,
		LDO_PWRDN, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL0_1_WORDSLICE_CNTRL_1,
		LDO_PWRDN, 1);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL1_1_WORDSLICE_CNTRL_1,
		LDO_PWRDN, 1);

	/* Stop the clocks */
	BDEV_WR_F_RB(CLK_DDR23_APHY_1_PM_CTRL, DIS_216M_CLK, 1);
	BDEV_WR_F_RB(CLK_DDR23_APHY_1_PM_CTRL, DIS_108M_CLK, 1);
	BDEV_WR_F_RB(CLK_MEMC_1_PM_CTRL, DIS_216M_CLK, 1);
	BDEV_WR_F_RB(CLK_MEMC_1_PM_CTRL, DIS_108M_CLK, 1);

}

static void bcm7420_memc1_resume(int mode)
{
	u32 val, cur_val;
	s32 sval, scur_val, inc_val;

	DBG("%s\n", __func__);
	/* Restart the clocks */
	BDEV_WR_F_RB(CLK_DDR23_APHY_1_PM_CTRL, DIS_216M_CLK, 0);
	BDEV_WR_F_RB(CLK_DDR23_APHY_1_PM_CTRL, DIS_108M_CLK, 0);
	BDEV_WR_F_RB(CLK_MEMC_1_PM_CTRL, DIS_216M_CLK, 0);
	BDEV_WR_F_RB(CLK_MEMC_1_PM_CTRL, DIS_108M_CLK, 0);

	/* power up LDOs */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_PLL_CTRL1_REG,
		LDO_PWRDN, 0);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL0_1_WORDSLICE_CNTRL_1,
		LDO_PWRDN, 0);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL1_1_WORDSLICE_CNTRL_1,
		LDO_PWRDN, 0);

	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH4_WL0_DQS0_PHASE_CNTRL, 0);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH5_WL0_DQS1_PHASE_CNTRL, 0);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH8_WL1_DQS0_PHASE_CNTRL, 0);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH9_WL1_DQS1_PHASE_CNTRL, 0);

	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH3_WL0_DQ_PHASE_CNTRL, 0);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH7_WL1_DQ_PHASE_CNTRL, 0);

	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH2_CLOCK_PHASE_CNTRL, 0);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_DESKEW_DLL_CNTRL, BYPASS_PHASE, 0);

	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL0_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 0);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL1_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 0);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL0_1_WORDSLICE_CNTRL_1,
		PWRDN_DLL_ON_SELFREF, 0);
	BDEV_WR_F_RB(MEMC_DDR23_APHY_WL1_1_WORDSLICE_CNTRL_1,
		PWRDN_DLL_ON_SELFREF, 0);

	BDEV_UNSET(BCHP_MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL,
	BCHP_MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL_DEVCLK_OFF_ON_SELFREF_MASK |
	BCHP_MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL_IDDQ_MODE_ON_SELFREF_MASK);
	mdelay(1);

	/* reset the freq divider */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_RESET, FREQ_DIV_RESET, 1);
	mdelay(1);
	/* reset DATAPATH_216, RD_DATAPATH_RESET, RESET_DATAPATH_DDR  */
	BDEV_SET_RB(BCHP_MEMC_DDR23_APHY_AC_1_RESET,
		BCHP_MEMC_DDR23_APHY_AC_1_RESET_DATAPATH_216_RESET_MASK |
		BCHP_MEMC_DDR23_APHY_AC_1_RESET_RD_DATAPATH_RESET_MASK |
		BCHP_MEMC_DDR23_APHY_AC_1_RESET_DATAPATH_DDR_RESET_MASK);
	mdelay(1);
	/* reset the vcxo */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_RESET, VCXO_RESET, 1);
	mdelay(1);
	/* de-assert reset the vcxo */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_RESET, VCXO_RESET, 0);

	/* de-assert reset the freq divider */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_RESET, FREQ_DIV_RESET, 0);
	mdelay(1);

	/* check for pll lock */
	while (!BDEV_RD_F(MEMC_DDR23_APHY_AC_1_DDR_PLL_LOCK_STATUS,
		LOCK_STATUS))
		;

	/* reload shmoo values */
	/* set wl0_dq phase */
	val = memc1_config.shmoo_value[4];
	cur_val = 0;
	while (cur_val <= val) {
		BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH3_WL0_DQ_PHASE_CNTRL,
			cur_val);
		cur_val++;
	}

	/* set wl1_dq phase */
	val = memc1_config.shmoo_value[5];
	cur_val = 0;
	while (cur_val <= val) {
		BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH7_WL1_DQ_PHASE_CNTRL,
			cur_val);
		cur_val++;
	}
	/* set ch2 phase */
	val = memc1_config.shmoo_value[6];
	cur_val = 0;
	while (cur_val < val) {
		BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH2_CLOCK_PHASE_CNTRL,
			cur_val);
		cur_val++;
	}

	/* set ch6 phase */
	val = memc1_config.shmoo_value[7];
	cur_val = 0;
	while (cur_val < val) {
		BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_DESKEW_DLL_CNTRL,
			BYPASS_PHASE, cur_val);
		cur_val++;
	}

	/* set wl0_dqs0 phases */
	sval = memc1_config.shmoo_value[0];
	scur_val = 0;
	inc_val = sval > 0 ? 1 : -1;
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH4_WL0_DQS0_PHASE_CNTRL,
		0);
	while (sval != scur_val) {
		scur_val += inc_val;
		BDEV_WR_RB(
			BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH4_WL0_DQS0_PHASE_CNTRL,
			scur_val);
	}

	/* set wl0_dqs1 phases */
	sval = memc1_config.shmoo_value[1];
	scur_val = 0;
	inc_val = sval > 0 ? 1 : -1;
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH5_WL0_DQS1_PHASE_CNTRL,
		0);
	while (sval != scur_val) {
		scur_val += inc_val;
		BDEV_WR_RB(
			BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH5_WL0_DQS1_PHASE_CNTRL,
			scur_val);
	}

	/* set wl1_dqs0 phases */
	sval = memc1_config.shmoo_value[2];
	scur_val = 0;
	inc_val = sval > 0 ? 1 : -1;
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH8_WL1_DQS0_PHASE_CNTRL,
		0);
	while (sval != scur_val) {
		scur_val += inc_val;
		BDEV_WR_RB(
			BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH8_WL1_DQS0_PHASE_CNTRL,
			scur_val);
	}

	/* set wl1_dqs1 phases */
	sval = memc1_config.shmoo_value[3];
	scur_val = 0;
	inc_val = sval > 0 ? 1 : -1;
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH9_WL1_DQS1_PHASE_CNTRL,
		0);
	while (sval != scur_val) {
		scur_val += inc_val;
		BDEV_WR_RB(
			BCHP_MEMC_DDR23_APHY_AC_1_PLL_CH9_WL1_DQS1_PHASE_CNTRL,
			scur_val);
	}

	/* reset the word slice dll */
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL0_1_WORD_SLICE_DLL_RESET, 1);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL1_1_WORD_SLICE_DLL_RESET, 1);
	mdelay(1);

	/* reset VCDL values */
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL0_1_BYTE0_VCDL_PHASE_CNTL, 0);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL0_1_BYTE1_VCDL_PHASE_CNTL, 0);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL1_1_BYTE0_VCDL_PHASE_CNTL, 0);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL1_1_BYTE1_VCDL_PHASE_CNTL, 0);

	/* de-assert reset of the word slice dll */
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL0_1_WORD_SLICE_DLL_RESET, 0);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL1_1_WORD_SLICE_DLL_RESET, 0);
	mdelay(1);

	/* de-assert reset from DATAPATH_216_RESET */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_RESET, DATAPATH_216_RESET, 0);
	/* de-assert reset from RD_DATAPATH_RESET */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_RESET, RD_DATAPATH_RESET, 0);
	/* de-assert reset from DATAPATH_DDR_RESET */
	BDEV_WR_F_RB(MEMC_DDR23_APHY_AC_1_RESET, DATAPATH_DDR_RESET, 0);
	mdelay(1);

	/*
	 * Reload VCDL values:
	 */
	cur_val = 0x0101;
	while (cur_val <= memc1_config.vcdl[0]) {
		BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL0_1_BYTE0_VCDL_PHASE_CNTL,
			cur_val);
		cur_val += 0x0101;
	}
	cur_val = 0x0101;
	while (cur_val <= memc1_config.vcdl[1]) {
		BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL0_1_BYTE1_VCDL_PHASE_CNTL,
			cur_val);
		cur_val += 0x0101;
	}
	cur_val = 0x0101;
	while (cur_val <= memc1_config.vcdl[2]) {
		BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL1_1_BYTE0_VCDL_PHASE_CNTL,
			cur_val);
		cur_val += 0x0101;
	}
	cur_val = 0x0101;
	while (cur_val <= memc1_config.vcdl[3]) {
		BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL1_1_BYTE1_VCDL_PHASE_CNTL,
			cur_val);
		cur_val += 0x0101;
	}

	if (mode) {
		BDEV_WR_F_RB(MEMC_MISC_1_SOFT_RESET,
			MEMC_DRAM_INIT, 0);
		BDEV_WR_F_RB(MEMC_MISC_1_SOFT_RESET,
			MEMC_CORE, 0);
		mdelay(1);
		printk(KERN_DEBUG "memc1: powered up\n");
	} else
		printk(KERN_DEBUG "memc1: resumed\n");

}

void brcm_pm_memc1_suspend(void)
{
	bcm7420_memc1_suspend(0);
}

void brcm_pm_memc1_resume(void)
{
	bcm7420_memc1_resume(0);
	bcm7420_memc1_sspd_control(0);
}

void brcm_pm_memc1_powerdown(void)
{
	int ii;
	u32 addr = BCHP_MEMC_ARB_1_REG_START + 4;

	DBG("%s\n", __func__);

	/* Save MEMC1 configuration */
	for (ii = 0; ii < MAX_CLIENT_INFO_NUM; ii++) {
		memc1_config.client_info[ii] = BDEV_RD(addr);
		addr += 4;
	}

	ii = 0;
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_MODE_2);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_MODE_3);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_5);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_MODE_0);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_MODE_1);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_0);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_1);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_2);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_3);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_TIMING_4);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_CNTRLR_CONFIG);
	memc1_config.ddr_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR_1_DRAM_INIT_CNTRL);

	BUG_ON(ii > MAX_DDR_PARAMS_NUM);

	ii = 0;
	memc1_config.ddr23_aphy_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CTRL1_REG);
	memc1_config.ddr23_aphy_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_AC_1_PLL_FREQ_CNTL);
	memc1_config.ddr23_aphy_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_AC_1_CONFIG);
	memc1_config.ddr23_aphy_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_AC_1_PAD_SSTL_DDR2_MODE);
	memc1_config.ddr23_aphy_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_WL1_1_PAD_SSTL_DDR2_MODE);
	memc1_config.ddr23_aphy_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_WL0_1_PAD_SSTL_DDR2_MODE);
	memc1_config.ddr23_aphy_params[ii++] =
		BDEV_RD(BCHP_MEMC_DDR23_APHY_AC_1_ODT_CONFIG);

	BUG_ON(ii > MAX_DDR_APHY_PARAMS_NUM);

	ii = 0;
	memc1_config.arb_params[ii++] =
		BDEV_RD(BCHP_MEMC_ARB_1_FULLNESS_THRESHOLD);
	memc1_config.arb_params[ii++] =
		BDEV_RD(BCHP_MEMC_ARB_1_MINIMUM_COMMAND_SIZE);

	BUG_ON(ii > MAX_ARB_PARAMS_NUM);

	/* CKE_IDDQ */
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL,
		BDEV_RD(BCHP_MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL) | 4);

	bcm7420_memc1_suspend(1);

	memc1_config.valid = 1;

}

int brcm_pm_memc1_powerup(void)
{
	int ii;
	u32 addr = BCHP_MEMC_ARB_1_REG_START + 4;

	DBG("%s\n", __func__);
	/* Restore MEMC1 configuration */

	if (!memc1_config.valid || !memc1_config.shmoo_valid) {
		printk(KERN_ERR "%s: no valid saved configuration\n", __func__);
		return -1;
	}

	bcm7420_memc1_resume(1);

	/* Remove CKE_IDDQ */
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL,
		BDEV_RD(BCHP_MEMC_DDR23_APHY_AC_1_DDR_PAD_CNTRL) & ~4);

	/* program ddr iobuf registers */
	ii = 0;
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_MODE_2,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_MODE_3,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_5,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_MODE_0,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_MODE_1,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_0,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_1,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_2,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_3,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_TIMING_4,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_CNTRLR_CONFIG,
		memc1_config.ddr_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR_1_DRAM_INIT_CNTRL,
		memc1_config.ddr_params[ii++]);

	ii = 0;
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_CTRL1_REG,
		memc1_config.ddr23_aphy_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PLL_FREQ_CNTL,
		memc1_config.ddr23_aphy_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_CONFIG,
		memc1_config.ddr23_aphy_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_PAD_SSTL_DDR2_MODE,
		memc1_config.ddr23_aphy_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL1_1_PAD_SSTL_DDR2_MODE,
		memc1_config.ddr23_aphy_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_WL0_1_PAD_SSTL_DDR2_MODE,
		memc1_config.ddr23_aphy_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_DDR23_APHY_AC_1_ODT_CONFIG,
		memc1_config.ddr23_aphy_params[ii++]);

	ii = 0;
	BDEV_WR_RB(BCHP_MEMC_ARB_1_FULLNESS_THRESHOLD,
		memc1_config.arb_params[ii++]);
	BDEV_WR_RB(BCHP_MEMC_ARB_1_MINIMUM_COMMAND_SIZE,
		memc1_config.arb_params[ii++]);

	for (ii = 0; ii < MAX_CLIENT_INFO_NUM; ii++) {
		BDEV_WR_RB(addr, memc1_config.client_info[ii]);
		addr += 4;
	}

	bcm7420_memc1_sspd_control(0);

	return 0;
}
#endif

#if defined(CONFIG_BCM7425)

void brcm_pm_memc1_suspend(void)
{
	u32 result;

	DBG("%s\n", __func__);

	/* power down the pads */
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 0);
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		PHY_IDLE_ENABLE, 1);
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		HIZ_ON_SELFREF, 1);
	BDEV_WR_RB(BCHP_DDR40_PHY_CONTROL_REGS_1_IDLE_PAD_CONTROL,
		0x132);
	BDEV_WR_RB(BCHP_DDR40_PHY_WORD_LANE_0_1_IDLE_PAD_CONTROL,
		BDEV_RD(BCHP_DDR40_PHY_WORD_LANE_0_1_IDLE_PAD_CONTROL) |
			0xFFFFF);
	BDEV_WR_RB(BCHP_DDR40_PHY_WORD_LANE_1_1_IDLE_PAD_CONTROL,
		BDEV_RD(BCHP_DDR40_PHY_WORD_LANE_1_1_IDLE_PAD_CONTROL) |
			0xFFFFF);

	/* enter SSPD */
	BDEV_WR_F_RB(MEMC_DDR_1_SSPD_CMD, SSPD, 1);
	do {
		result = BDEV_RD_F(MEMC_DDR_1_POWER_DOWN_STATUS, SSPD);
	} while (!result);

	/* power down the PLLs */
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_SYS_PLL_PWRDN_ref_clk_sel,
		PWRDN, 1);
	BDEV_WR_RB(BCHP_DDR40_PHY_CONTROL_REGS_1_PLL_CONFIG, 3);

	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_POWER_SWITCH_MEMORY,
		DDR1_POWER_SWITCH_MEMORY, 3);
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_MEMORY_STANDBY_ENABLE,
		DDR1_MEMORY_STANDBY_ENABLE, 1);

	BDEV_WR_F_RB(CLKGEN_MEMSYS_1_32_POWER_MANAGEMENT,
		MEMSYS_PLL_PWRDN_POWER_MANAGEMENT, 1);

	/* stop the clocks to MEMSYS1 */
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_CLOCK_ENABLE,
		DDR1_SCB_CLOCK_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_CLOCK_ENABLE,
		DDR1_108_CLOCK_ENABLE, 0);

}

void brcm_pm_memc1_resume(void)
{
	u32 result;

	DBG("%s\n", __func__);

	/* re-enable the clocks */
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_CLOCK_ENABLE,
		DDR1_SCB_CLOCK_ENABLE, 1);
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_CLOCK_ENABLE,
		DDR1_108_CLOCK_ENABLE, 1);

	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_SYS_PLL_PWRDN_ref_clk_sel,
		PWRDN, 0);
	BDEV_WR_RB(BCHP_DDR40_PHY_CONTROL_REGS_1_PLL_CONFIG, 0);

	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_POWER_SWITCH_MEMORY,
		DDR1_POWER_SWITCH_MEMORY, 2);
	mdelay(1);
	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_POWER_SWITCH_MEMORY,
		DDR1_POWER_SWITCH_MEMORY, 0);

	BDEV_WR_F_RB(CLKGEN_MEMSYS_32_1_INST_MEMORY_STANDBY_ENABLE,
		DDR1_MEMORY_STANDBY_ENABLE, 0);
	BDEV_WR_F_RB(CLKGEN_MEMSYS_1_32_POWER_MANAGEMENT,
		MEMSYS_PLL_PWRDN_POWER_MANAGEMENT, 0);

	/* power up the pads */
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		IDDQ_MODE_ON_SELFREF, 0);
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		PHY_IDLE_ENABLE, 0);
	BDEV_WR_F_RB(MEMC_DDR23_SHIM_ADDR_CNTL_1_DDR_PAD_CNTRL,
		HIZ_ON_SELFREF, 0);
	BDEV_WR_RB(BCHP_DDR40_PHY_CONTROL_REGS_1_IDLE_PAD_CONTROL,
		0);
	BDEV_WR_RB(BCHP_DDR40_PHY_WORD_LANE_0_1_IDLE_PAD_CONTROL,
		BDEV_RD(BCHP_DDR40_PHY_WORD_LANE_0_1_IDLE_PAD_CONTROL) |
			0);
	BDEV_WR_RB(BCHP_DDR40_PHY_WORD_LANE_1_1_IDLE_PAD_CONTROL,
		BDEV_RD(BCHP_DDR40_PHY_WORD_LANE_1_1_IDLE_PAD_CONTROL) |
			0);

	/* leave SSPD */
	BDEV_WR_F_RB(MEMC_DDR_1_SSPD_CMD, SSPD, 0);
	do {
		result = BDEV_RD_F(MEMC_DDR_1_POWER_DOWN_STATUS, SSPD);
	} while (result);

}
#endif

#endif
