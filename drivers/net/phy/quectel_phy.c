/**
 * @file quectel_phy.c
 * @brief Quectel PHY driver.
 * 
 * @note
 */
/*=============================================================================
  Copyright (c) 2018 Quectel Wireless Solution, Co., Ltd.  All Rights Reserved.
  Quectel Wireless Solution Proprietary and Confidential.
=============================================================================*/

/*=====================================================================

                          EDIT HISTORY FOR MODULE
  
  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.
  
  WHEN             WHO         WHAT, WHERE, WHY
  ------------     -------     ----------------------------------------
  24/07/2018       Mike        Support dp83tc811s-q1(TI) PHY
  17/03/2022       jayden      Support bcm89834 PHY
  17/09/2022       jayden      Support bcm89832 PHY
=======================================================================*/
#include <linux/phy.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include "../ethernet/qualcomm/emac/emac.h"

#define QUECTEL_PHY_ID       0x12345678
#define AR8033_PHY_ID        0x004dd074
#define BCM89820_PHY_ID      0x03625cd2
#define DP83TC811S_PHY_ID    0x2000a253
#define BCM89834_PHY_ID      0x35905050
#define BCM89832_PHY_ID      0x35905048
#define MARVELL_88Q111_PHY_ID 0x002b0b21

static unsigned phy_speed = SPEED_1000;
module_param(phy_speed, uint, 0644);
static unsigned phy_debug = 0;
module_param(phy_debug, uint, 0644);

#define quecphy_debug(phydev) do {\
	if (phy_debug) {\
		dev_info(&phydev->dev, "%s\n", __func__);\
		quecphy_dump(phydev);\
	}\
} while(0)

static void quecphy_dump(const struct phy_device *phydev)
{
	dev_info(&phydev->dev, "state=%d, link=%d, autoneg=%d, speed=%d, duplex=%d, pause=%d, asym_pause=%d",
		phydev->state, phydev->link, phydev->autoneg,
		phydev->speed, phydev->duplex, phydev->pause, phydev->asym_pause);
}

static ssize_t phy_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct phy_device *phydev = to_phy_device(dev);

	return sprintf(buf, "state=%d, link=%d\n", phydev->state, phydev->link);
}
static DEVICE_ATTR_RO(phy_state);

int quecphy_config_aneg(struct phy_device *phydev)
{
    //dump_stack();
    quecphy_debug(phydev);
    
    return 0;
}

static int phy_88q111_softrst(struct phy_device *phydev)
{
	phy_write(phydev, 0xd, 0x1);
	phy_write(phydev, 0xe, 0x00);
	phy_write(phydev, 0xd, 0x4001);
	phy_write(phydev, 0xe,BMCR_RESET);
	
	printk("Marvell 88Q1111 Soft Reset: ID:0x%X\n", phydev->phy_id);

	return 0;
}

static int quecphy_read_status(struct phy_device *phydev)
{
    //phy_state_machine() call me per 1 seconds
    int ret = 0;
    //dump_stack();
   
    if ((phydev->phy_id == QUECTEL_PHY_ID) || (phydev->phy_id == BCM89834_PHY_ID) || 
		 (phydev->phy_id == BCM89832_PHY_ID) || (phydev->phy_id == MARVELL_88Q111_PHY_ID))
        phydev->link = 1;
    else
        genphy_update_link(phydev);

    phydev->speed = phy_speed;
    phydev->duplex = DUPLEX_FULL;
    phydev->autoneg = AUTONEG_DISABLE;
    phydev->pause = 0;
    phydev->asym_pause = 0;

    quecphy_debug(phydev);
    return ret;
}

static int quecphy_aneg_done(struct phy_device *phydev)
{
    //dump_stack();
    quecphy_debug(phydev);

    return BMSR_ANEGCOMPLETE;
}

static int quecphy_config_init(struct phy_device *phydev)
{
    //dump_stack();
    quecphy_debug(phydev);
        
	return 0;
}

//2022/03/17 jayden add
static void quecphy_bcm89834_reset(struct phy_device *phydev)
{
	struct emac_adapter *adpt = netdev_priv(phydev->attached_dev);
	
	adpt->gpio_off(adpt, false, true);
	mdelay(30);
	adpt->gpio_on(adpt, true, true);
	mdelay(50);
	
	phy_write(phydev, 0x1e, 0x045);
	phy_write(phydev, 0x1f, 0x0000);
	
	phy_write(phydev, 0x1e, 0x02f);
	phy_write(phydev, 0x1f, 0x7067);
	
	phy_write(phydev, 0x1e, 0x811);
	phy_write(phydev, 0x1f, 0x9060);
	
	phy_write(phydev, 0x1e, 0x8b3);
	phy_write(phydev, 0x1f, 0x8300);
	
	phy_write(phydev, 0x1e, 0x1d4);
	phy_write(phydev, 0x1f, 0x0000);
	
	phy_write(phydev,0x0d,0x7);
	phy_write(phydev,0x0e,0x98b3);
	phy_write(phydev,0x0d,0x4007);
	phy_write(phydev,0x0e,0x0300);
	
	phy_write(phydev,0x0d,0x7);
	phy_write(phydev,0x0e,0x98b2);
	phy_write(phydev,0x0d,0x4007);
	phy_write(phydev,0x0e,0x8300);
	
	phy_write(phydev,0x0d,0x7);
	phy_write(phydev,0x0e,0x9c00);
	phy_write(phydev,0x0d,0x4007);
	phy_write(phydev,0x0e,0x0120);
	
	phy_write(phydev,0x0d,0x7);
	phy_write(phydev,0x0e,0x98b2);
	phy_write(phydev,0x0d,0x4007);
	phy_write(phydev,0x0e,0xffe0);
	
	phy_write(phydev,0x0d,0x7);
	phy_write(phydev,0x0e,0x9ce0);
	phy_write(phydev,0x0d,0x4007);
	phy_write(phydev,0x0e,0x3100);
	
	phy_write(phydev,0x0d,0x7);
	phy_write(phydev,0x0e,0x98b2);
	phy_write(phydev,0x0d,0x4007);
	phy_write(phydev,0x0e,0x8300);
	
	phy_write(phydev,0x0d,0x7);
	phy_write(phydev,0x0e,0x9c01);
	phy_write(phydev,0x0d,0x4007);
	phy_write(phydev,0x0e,0x0046);
	
	phy_write(phydev,0x0d,0x7);
	phy_write(phydev,0x0e,0x98b2);
	phy_write(phydev,0x0d,0x4007);
	phy_write(phydev,0x0e,0x0000);
	
	phy_write(phydev,0x0d,0x7);
	phy_write(phydev,0x0e,0x98b3);
	phy_write(phydev,0x0d,0x4007);
	phy_write(phydev,0x0e,0x8300);
	
	printk("=================== ***init bcm89834 done*** =====================\n");
}

static void quecphy_bcm89832_reset(struct phy_device *phydev)
{
	struct emac_adapter *adpt = netdev_priv(phydev->attached_dev);
    
	adpt->gpio_off(adpt, false, true);
	mdelay(30);
	adpt->gpio_on(adpt, true, true);
	mdelay(50);

	phy_write(phydev, 0x0d, 0x7);
	phy_write(phydev, 0x0e, 0x98b3);
	phy_write(phydev, 0x0d, 0x4007);
	phy_write(phydev, 0x0e, 0x300);

	phy_write(phydev, 0x0d, 0x5);
	phy_write(phydev, 0x0e, 0x8300);
	phy_write(phydev, 0x0d, 0x4005);
	phy_write(phydev, 0x0e, 0x120);


	phy_write(phydev, 0x0d, 0x5);
	phy_write(phydev, 0x0e, 0xffe0);
	phy_write(phydev, 0x0d, 0x4005);
	phy_write(phydev, 0x0e, 0x3100);

	phy_write(phydev, 0x0d, 0x5);
	phy_write(phydev, 0x0e, 0x8301);
	phy_write(phydev, 0x0d, 0x4005);
	phy_write(phydev, 0x0e, 0x46);

	printk("*************init bcm89832 done*******************\n");
}


static int quecphy_soft_reset(struct phy_device *phydev)
{
    //dump_stack();
    quecphy_debug(phydev);
    
    if (phydev->phy_id != QUECTEL_PHY_ID && 
		phydev->phy_id != BCM89834_PHY_ID && 
		phydev->phy_id != BCM89832_PHY_ID &&
		phydev->phy_id != MARVELL_88Q111_PHY_ID)
        genphy_soft_reset(phydev);

    if (phydev->phy_id == BCM89820_PHY_ID) {
        phy_write(phydev, 0x00, 0x0200);  //0x0208~masterm,0x0200~slave
    } else if (phydev->phy_id == DP83TC811S_PHY_ID) {
        phy_write(phydev, 0x01, 0x0065);
    }else if(phydev->phy_id == BCM89834_PHY_ID){
		quecphy_bcm89834_reset(phydev);
    }else if(phydev->phy_id == BCM89832_PHY_ID){
		quecphy_bcm89832_reset(phydev);	
	}else if (phydev->phy_id == MARVELL_88Q111_PHY_ID){
		phy_88q111_softrst(phydev);
    }

    return 0;
}


static int quecphy_match_phy_device(struct phy_device *phydev) 
{
    dev_info(&phydev->dev, "phy_id=0x%08x\n", phydev->phy_id);
	
    if (phydev->phy_id == BCM89820_PHY_ID || 
		phydev->phy_id == DP83TC811S_PHY_ID || 
		phydev->phy_id == BCM89834_PHY_ID || 
		phydev->phy_id == BCM89832_PHY_ID || 
		phydev->phy_id == MARVELL_88Q111_PHY_ID)
        phy_speed = SPEED_100;

    return 1;
}

static int quecphy_probe(struct phy_device *phydev)
{
	device_create_file(&phydev->dev, &dev_attr_phy_state);
	return 0;
}

static void quecphy_remove(struct phy_device *phydev)
{
	device_remove_file(&phydev->dev, &dev_attr_phy_state);
	return;
}

static struct phy_driver quecphy_driver = {
	.phy_id			= 0x12345678,
	.name			= "Quectel PHY",
	.phy_id_mask		= 0xffffffff,
	.match_phy_device = quecphy_match_phy_device,
	.probe			= quecphy_probe,
	.soft_reset     = quecphy_soft_reset,
	.config_init		= quecphy_config_init,
	.features		= PHY_GBIT_FEATURES,
	.flags			= 0,
	.config_aneg		= quecphy_config_aneg,
	.read_status		= quecphy_read_status,
	.aneg_done		= quecphy_aneg_done,
	.remove			= quecphy_remove,
	.driver		= { .owner = THIS_MODULE },
};

static int __init quecphy_init(void)
{
    return phy_driver_register(&quecphy_driver);
}

static void __exit quecphy_exit(void)
{
    phy_driver_unregister(&quecphy_driver);
}

module_init(quecphy_init);
module_exit(quecphy_exit);
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:quecphy");
