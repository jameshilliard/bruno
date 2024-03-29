/*
 *
 * Copyright (c) 2002-2005 Broadcom Corporation
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
 *
 *File Name  : bcmmii.c
 *
 *Description: Broadcom PHY/GPHY/Ethernet Switch Configuration
 *Revision:	09/25/2008, L.Sun created.
*/

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <asm/brcmstb/brcmstb.h>

#include "bcmgenet_map.h"
#include "bcmgenet.h"
#include "bcmmii.h"

/* read a value from the MII */
int mii_read(struct net_device *dev, int phy_id, int location)
{
	int ret;
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	volatile struct uniMacRegs *umac = pDevCtrl->umac;

	if (phy_id == BRCM_PHY_ID_NONE) {
		switch (location) {
		case MII_BMCR:
			return pDevCtrl->phyType == BRCM_PHY_TYPE_EXT_MII ?
				BMCR_FULLDPLX | BMCR_SPEED100 :
				BMCR_FULLDPLX | BMCR_SPEED1000;
		case MII_BMSR:
			return BMSR_LSTATUS;
		default:
			return 0;
		}
	}

	mutex_lock(&pDevCtrl->mdio_mutex);

	umac->mdio_cmd = (MDIO_RD | (phy_id << MDIO_PMD_SHIFT) |
			(location << MDIO_REG_SHIFT));
	/* Start MDIO transaction*/
	umac->mdio_cmd |= MDIO_START_BUSY;
	wait_event_timeout(pDevCtrl->wq, !(umac->mdio_cmd & MDIO_START_BUSY),
			HZ/100);
	mutex_unlock(&pDevCtrl->mdio_mutex);
	ret = umac->mdio_cmd;
	if (ret & MDIO_READ_FAIL) {
		TRACE(("MDIO read failure\n"));
		ret = 0;
	}
	return ret & 0xffff;
}

/* write a value to the MII */
void mii_write(struct net_device *dev, int phy_id, int location, int val)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	volatile struct uniMacRegs *umac = pDevCtrl->umac;

	if (phy_id == BRCM_PHY_ID_NONE)
		return;
	mutex_lock(&pDevCtrl->mdio_mutex);
	umac->mdio_cmd = (MDIO_WR | (phy_id << MDIO_PMD_SHIFT) |
			(location << MDIO_REG_SHIFT) | (0xffff & val));
	umac->mdio_cmd |= MDIO_START_BUSY;
	wait_event_timeout(pDevCtrl->wq, !(umac->mdio_cmd & MDIO_START_BUSY),
			HZ/100);
	mutex_unlock(&pDevCtrl->mdio_mutex);
}
/* probe for an external PHY via MDIO; return PHY address */
int mii_probe(struct net_device *dev, void *p)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	int i;
	struct bcmemac_platform_data *cfg = p;

	if (cfg->phy_type != BRCM_PHY_TYPE_EXT_MII) {
		/*
		 * Enable RGMII to interface external PHY, disable
		 * internal 10/100 MII.
		 */
		GENET_RGMII_OOB_CTRL(pDevCtrl) |= RGMII_MODE_EN;
	}
	/* Power down EPHY */
	if (pDevCtrl->ext)
		pDevCtrl->ext->ext_pwr_mgmt |= (EXT_PWR_DOWN_PHY |
			EXT_PWR_DOWN_DLL | EXT_PWR_DOWN_BIAS);

	for (i = 31; i >= 0; i--) {
		if (mii_read(dev, i, MII_BMSR) != 0) {
			pDevCtrl->phyAddr = i;
			if (i == 1)
				continue;
			return 0;
		}
		TRACE(("I=%d\n", i));
	}
	return -ENODEV;
}

/*
 * setup netdev link state when PHY link status change and
 * update UMAC and RGMII block when link up
 */
void mii_setup(struct net_device *dev)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	struct ethtool_cmd ecmd ;
	volatile struct uniMacRegs *umac = pDevCtrl->umac;
	int cur_link;
	int prev_link;

	TRACE(("%s: %s\n", __func__, netif_carrier_ok(pDevCtrl->dev) ?
				"netif_carrier_on" : "netif_carrier_off"));
	if (pDevCtrl->phyType == BRCM_PHY_TYPE_MOCA) {
		netif_carrier_on(pDevCtrl->dev);
		return;
	}
	cur_link = mii_link_ok(&pDevCtrl->mii);
	prev_link = netif_carrier_ok(pDevCtrl->dev);
	if (cur_link && !prev_link) {
		mii_ethtool_gset(&pDevCtrl->mii, &ecmd);
		/*
		 * program UMAC and RGMII block accordingly, if the PHY is
		 * not capable of in-band signaling.
		 */
		if (pDevCtrl->phyType != BRCM_PHY_TYPE_EXT_RGMII_IBS) {
			GENET_RGMII_OOB_CTRL(pDevCtrl) &= ~OOB_DISABLE;
			GENET_RGMII_OOB_CTRL(pDevCtrl) |= RGMII_LINK;
			if (ecmd.duplex == DUPLEX_FULL)
				umac->cmd &= ~CMD_HD_EN;
			else
				umac->cmd |= CMD_HD_EN;
			/* speed */
			umac->cmd = umac->cmd &
					~(CMD_SPEED_MASK << CMD_SPEED_SHIFT);
			if (ecmd.speed == SPEED_10)
				umac->cmd |=
					(UMAC_SPEED_10 << CMD_SPEED_SHIFT);
			else if (ecmd.speed == SPEED_100)
				umac->cmd |=
					(UMAC_SPEED_100 << CMD_SPEED_SHIFT);
			else if (ecmd.speed == SPEED_1000)
				umac->cmd |=
					(UMAC_SPEED_1000 << CMD_SPEED_SHIFT);
		}
		/* pause capability */
		if (pDevCtrl->phyType == BRCM_PHY_TYPE_INT ||
		    pDevCtrl->phyType == BRCM_PHY_TYPE_EXT_MII) {
			unsigned int val;
			val = mii_read(dev, pDevCtrl->phyAddr, MII_LPA);
			if (!(val & LPA_PAUSE_CAP)) {
				umac->cmd |= CMD_RX_PAUSE_IGNORE;
				umac->cmd |= CMD_TX_PAUSE_IGNORE;
			}
		} else if (pDevCtrl->phyType == BRCM_PHY_TYPE_EXT_RGMII ||
			   pDevCtrl->phyType == BRCM_PHY_TYPE_EXT_RGMII_IBS) {
			unsigned int val;
			val = mii_read(dev,
				pDevCtrl->phyAddr, MII_BRCM_AUX_STAT_SUM);
			if (!(val & MII_BRCM_AUX_GPHY_RX_PAUSE))
				umac->cmd |= CMD_RX_PAUSE_IGNORE;
			if (!(val & MII_BRCM_AUX_GPHY_TX_PAUSE))
				umac->cmd |= CMD_TX_PAUSE_IGNORE;
		}
		netif_carrier_on(pDevCtrl->dev);
		printk(KERN_INFO "%s: Link up, %d Mbps %s Duplex\n",
			pDevCtrl->dev->name,
			ecmd.speed,
			ecmd.duplex == DUPLEX_FULL ? "Full" : "Half");
	} else if (!cur_link && prev_link) {
		netif_carrier_off(pDevCtrl->dev);
		printk(KERN_INFO "%s: Link down\n", pDevCtrl->dev->name);
		return;
	}
}

int mii_init(struct net_device *dev)
{
	struct BcmEnet_devctrl *pDevCtrl = netdev_priv(dev);
	volatile struct uniMacRegs *umac;
	int bmcr;

	umac = pDevCtrl->umac;
	pDevCtrl->mii.phy_id = pDevCtrl->phyAddr;
	pDevCtrl->mii.phy_id_mask = 0x1f;
	pDevCtrl->mii.reg_num_mask = 0x1f;
	pDevCtrl->mii.dev = dev;
	pDevCtrl->mii.mdio_read = mii_read;
	pDevCtrl->mii.mdio_write = mii_write;

	/* Enable autoneg if it's not */
	bmcr = mii_read(dev, pDevCtrl->phyAddr, MII_BMCR);
	switch (pDevCtrl->phyType) {

	case BRCM_PHY_TYPE_INT:
		pDevCtrl->mii.supports_gmii = 0;
		pDevCtrl->sys->sys_port_ctrl = PORT_MODE_INT_EPHY;
		/* enable APD */
		pDevCtrl->ext->ext_pwr_mgmt |= EXT_PWR_DN_EN_LD;
		mii_write(dev, pDevCtrl->phyAddr, MII_BMCR, BMCR_RESET);
		udelay(1);
		/* enable 64 clock MDIO */
		mii_write(dev, pDevCtrl->phyAddr, 0x1d, 0x1000);
		mii_read(dev, pDevCtrl->phyAddr, 0x1d);
		printk(KERN_INFO "Config internal EPHY through MDIO\n");
		break;
	case BRCM_PHY_TYPE_EXT_MII:
		pDevCtrl->mii.supports_gmii = 0;
		pDevCtrl->sys->sys_port_ctrl = PORT_MODE_EXT_EPHY;
		mii_write(dev, pDevCtrl->phyAddr, MII_BMCR, bmcr|MII_BMCR);
		printk(KERN_INFO "Config EPHY through MDIO\n");
		break;
	case BRCM_PHY_TYPE_EXT_RGMII:
		GENET_RGMII_OOB_CTRL(pDevCtrl) |= RGMII_MODE_EN;
		/* Don't shift tx clock by 90 degree, does it still needed? */
		GENET_RGMII_OOB_CTRL(pDevCtrl) |= (1 << 16);
		pDevCtrl->mii.supports_gmii = 1;
		pDevCtrl->sys->sys_port_ctrl = PORT_MODE_EXT_GPHY;
		mii_write(dev, pDevCtrl->phyAddr, MII_BMCR, bmcr|MII_BMCR);
		printk(KERN_INFO "Config GPHY through MDIO\n");
		break;
	case BRCM_PHY_TYPE_EXT_RGMII_IBS:
		GENET_RGMII_OOB_CTRL(pDevCtrl) |= RGMII_MODE_EN;
		GENET_RGMII_OOB_CTRL(pDevCtrl) |= (1 << 16);
		/* Use in-band signaling for auto config.*/
		GENET_RGMII_OOB_CTRL(pDevCtrl) |= OOB_DISABLE;
		umac->cmd |= CMD_AUTO_CONFIG;
		pDevCtrl->mii.supports_gmii = 1;
		pDevCtrl->sys->sys_port_ctrl = PORT_MODE_EXT_GPHY;
		mii_write(dev, pDevCtrl->phyAddr, MII_BMCR, bmcr|MII_BMCR);
		printk(KERN_INFO "Automatic Config GPHY\n");
		break;
	case BRCM_PHY_TYPE_MOCA:
		printk(KERN_INFO "Config MoCA...\n");
		umac->cmd = umac->cmd  | (UMAC_SPEED_1000 << CMD_SPEED_SHIFT);
		pDevCtrl->mii.force_media = 1;
		pDevCtrl->sys->sys_port_ctrl = PORT_MODE_INT_GPHY |
			LED_ACT_SOURCE_MAC;
		break;
	default:
		printk(KERN_ERR "unknown phy_type : %d\n", pDevCtrl->phyType);
		break;
	}

	return 0;
}
