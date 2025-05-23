/*
 * Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/of_gpio.h>
#include <linux/clk/msm-clk.h>
#include <soc/qcom/subsystem_restart.h>
#include <soc/qcom/ramdump.h>
#include <soc/qcom/smem.h>
#include <soc/qcom/smsm.h>

#include "peripheral-loader.h"
#include "pil-q6v5.h"
#include "pil-msa.h"

//quectel add 20180622
#include "linux/fs.h"  
#include "linux/string.h" 
#include "linux/timer.h"

#define MAX_VDD_MSS_UV		1150000
#define PROXY_TIMEOUT_MS	10000
#define MAX_SSR_REASON_LEN	130U
#define STOP_ACK_TIMEOUT_MS	1000

#define subsys_to_drv(d) container_of(d, struct modem_data, subsys_desc)

#define QUECTEL_EFS_ERROR_BACKUP//quectel add 20180622
#ifdef QUECTEL_EFS_ERROR_BACKUP

#define MODEM_NORMAL_TIMEOUT_S 60000  //  if modem load and runnig this times not restart,  wo think modem runnig ok, then clear the modemFatalErrorRecord cnt.
#define  MODEM_FATALERROR_RECORD_CNT_PATH    "/data/modemFatalErrorRecord" // modem fatal error  count record path 
#define  MODEM_FATALERROR_ALLOW_TIMES  3  // modem fatal error this times,  set efs  restore flag,   must < 10

struct timer_list  ModemOntimer;
struct work_struct MomdemOn_timer_work;
struct work_struct ModemFatalError_handler_work;


extern unsigned int Quectel_Set_Partition_RestoreFlag(const char* partition_name,int mtd_nub, int where);

/******************************************************************************************
who-2018/06/25:Description....
Refer to [Issue-Depot].[IS0000197][Submitter:ramos.zhang,Date:2018-06-25]
<模块现场efs损坏无法还原modem不停重启或者cfun=7无法上网>
******************************************************************************************/
void Quectel_clean_modemFatalTimes(void)
{

	struct file *file = NULL;
    
	file = filp_open(MODEM_FATALERROR_RECORD_CNT_PATH, O_RDONLY | O_TRUNC ,0); // O_TRUNC  open file and delete old data 
	if (IS_ERR(file))
	{
        pr_err("%s, clean modemFatalError cnt  filp_open fail !\n",__func__);
		return;
	}

	if(NULL != file)
		filp_close(file, NULL);
	
	pr_err("%s,line:%d  success !\n",__func__,__LINE__);
	return;
}

void Quectel_modemOnLine_timer_handler(unsigned long data) 
{
	schedule_work(&MomdemOn_timer_work);	
}

void Quectel_modemOnLine_timer_start(void)
{
	ModemOntimer.function = (void *)Quectel_modemOnLine_timer_handler; 
	ModemOntimer.expires = jiffies + msecs_to_jiffies(MODEM_NORMAL_TIMEOUT_S);
	add_timer(&ModemOntimer);
}

void Quectel_modemOnLine_timer_stop(void)
{
	if(timer_pending(&ModemOntimer))
	{
		pr_err("%s, ModemOntimer stop delete now!!!!\n", __func__);
		del_timer(&ModemOntimer);
	}
}

void  Quectel_modemOnLine_timer_init(void)
{
	init_timer(&ModemOntimer);
    //pr_err("%s, timer pending:%d \n",__func__, timer_pending(&ModemOntimer));	
}

static  int Quectel_ModemFatalErrProcess( void )
{
	char tmpbuf[5]={0};
	int wr=0,rc=0,cnt=0,i=0;
	struct file *file=NULL;

	Quectel_modemOnLine_timer_stop();//

	file = filp_open(MODEM_FATALERROR_RECORD_CNT_PATH, O_RDWR| O_DSYNC | O_CREAT,0600); 
	if (IS_ERR(file))
	{
		pr_err("%s, filp_open fail\n",__func__);
		goto fail1;
	}

	rc = kernel_read(file, 0, tmpbuf, 3);
	if (rc < 0 ) 
	{
		pr_err("%s, kernel read fail rc=%d\n",__func__,rc);
		goto fail2;
	}
	while(tmpbuf[i])
	{
		if (tmpbuf[i] == '\r' || tmpbuf[i] == '\t' || tmpbuf[i] == '\n' )
		{
			tmpbuf[i] = 0;
		}
		i++;
	}
	sscanf(tmpbuf, "%d", &cnt);
	pr_err("%s,modem Fatal times=%d.\n",__func__, cnt);
	if( cnt  < MODEM_FATALERROR_ALLOW_TIMES)
	{
		cnt++;
		memset(tmpbuf,0x00,sizeof(tmpbuf));
		sprintf(tmpbuf, "%d", cnt); 
		//pr_err("%s, tmpbuf=[%s] tmpbuf_len=%d\n",__func__,tmpbuf,strlen(tmpbuf));
		wr = kernel_write(file, tmpbuf, 1, 0);
		if(NULL != file)
			filp_close(file, NULL);
		pr_err("%s,tmpbuf=[%s],write sucess !!!\n",__func__,tmpbuf);
		return 1;
	}
    else
    {
		pr_err("%s, fatal error 3 times, set efs restore flag\n",__func__);
		wr = kernel_write(file, "0", 1, 0);
		if(wr != 1)
		{
			wr = kernel_write(file, "0", 1, 0);
		}
		if(NULL != file)
			filp_close(file, NULL);
		// set  efs restore flag
		Quectel_Set_Partition_RestoreFlag("efs2",-1,9);
		return 0;
    }
// modify by hans @20201223
fail1:
	pr_err("%s, goto fail1\n",__func__);      
	return 0;
fail2:
	pr_err("%s, goto fail2\n",__func__);
	if(NULL != file)
	filp_close(file, NULL);        
	return 0;
}
#else

ERROR: Please make sure QUECTEL_EFS_ERROR_BACKUP is defined !!!, Is it not need on your project
#endif

static void log_modem_sfr(void)
{
	u32 size;
	char *smem_reason, reason[MAX_SSR_REASON_LEN];

	smem_reason = smem_get_entry_no_rlock(SMEM_SSR_REASON_MSS0, &size, 0,
							SMEM_ANY_HOST_FLAG);
	if (!smem_reason || !size) {
		pr_err("modem subsystem failure reason: (unknown, smem_get_entry_no_rlock failed).\n");
		return;
	}
	if (!smem_reason[0]) {
		pr_err("modem subsystem failure reason: (unknown, empty string found).\n");
		return;
	}

	strlcpy(reason, smem_reason, min(size, MAX_SSR_REASON_LEN));
	pr_err("modem subsystem failure reason: %s.\n", reason);

	smem_reason[0] = '\0';
	wmb();
}

static void restart_modem(struct modem_data *drv)
{
	log_modem_sfr();
	drv->ignore_errors = true;
#ifdef QUECTEL_EFS_ERROR_BACKUP
	schedule_work(&ModemFatalError_handler_work);	
#else
	ERROR: make sure QUECTEL_EFS_ERROR_BACKUP is not need on your project
#endif
	subsystem_restart_dev(drv->subsys);
}

static irqreturn_t modem_err_fatal_intr_handler(int irq, void *dev_id)
{
	struct modem_data *drv = subsys_to_drv(dev_id);

	/* Ignore if we're the one that set the force stop GPIO */
	if (drv->crash_shutdown)
		return IRQ_HANDLED;

	pr_err("Fatal error on the modem.\n");
	subsys_set_crash_status(drv->subsys, true);
	restart_modem(drv);
	return IRQ_HANDLED;
}

static irqreturn_t modem_stop_ack_intr_handler(int irq, void *dev_id)
{
	struct modem_data *drv = subsys_to_drv(dev_id);
	pr_info("Received stop ack interrupt from modem\n");
	complete(&drv->stop_ack);
	return IRQ_HANDLED;
}

static int modem_shutdown(const struct subsys_desc *subsys, bool force_stop)
{
	struct modem_data *drv = subsys_to_drv(subsys);
	unsigned long ret;

	if (subsys->is_not_loadable)
		return 0;

	if (!subsys_get_crash_status(drv->subsys) && force_stop &&
	    subsys->force_stop_gpio) {
		gpio_set_value(subsys->force_stop_gpio, 1);
		ret = wait_for_completion_timeout(&drv->stop_ack,
				msecs_to_jiffies(STOP_ACK_TIMEOUT_MS));
		if (!ret)
			pr_warn("Timed out on stop ack from modem.\n");
		gpio_set_value(subsys->force_stop_gpio, 0);
	}

	if (drv->subsys_desc.ramdump_disable_gpio) {
		drv->subsys_desc.ramdump_disable = gpio_get_value(
					drv->subsys_desc.ramdump_disable_gpio);
		 pr_warn("Ramdump disable gpio value is %d\n",
			drv->subsys_desc.ramdump_disable);
	}

	pil_shutdown(&drv->q6->desc);

	return 0;
}

static int modem_powerup(const struct subsys_desc *subsys)
{
	struct modem_data *drv = subsys_to_drv(subsys);

	if (subsys->is_not_loadable)
		return 0;
	/*
	 * At this time, the modem is shutdown. Therefore this function cannot
	 * run concurrently with the watchdog bite error handler, making it safe
	 * to unset the flag below.
	 */
	reinit_completion(&drv->stop_ack);
	drv->subsys_desc.ramdump_disable = 0;
	drv->ignore_errors = false;
	drv->q6->desc.fw_name = subsys->fw_name;
#ifdef QUECTEL_EFS_ERROR_BACKUP
	Quectel_modemOnLine_timer_start();
#else
	ERROR: Please moke sure QUECTEL_EFS_ERROR_BACKUP  is not need on you project 
#endif
	return pil_boot(&drv->q6->desc);
}

static void modem_crash_shutdown(const struct subsys_desc *subsys)
{
	struct modem_data *drv = subsys_to_drv(subsys);
	drv->crash_shutdown = true;
	if (!subsys_get_crash_status(drv->subsys) &&
		subsys->force_stop_gpio) {
		gpio_set_value(subsys->force_stop_gpio, 1);
		mdelay(STOP_ACK_TIMEOUT_MS);
	}
}

static int modem_ramdump(int enable, const struct subsys_desc *subsys)
{
	struct modem_data *drv = subsys_to_drv(subsys);
	int ret;

	if (!enable)
		return 0;

	ret = pil_mss_make_proxy_votes(&drv->q6->desc);
	if (ret)
		return ret;

	ret = pil_mss_reset_load_mba(&drv->q6->desc);
	if (ret)
		return ret;

	ret = pil_do_ramdump(&drv->q6->desc, drv->ramdump_dev);
	if (ret < 0)
		pr_err("Unable to dump modem fw memory (rc = %d).\n", ret);

	ret = __pil_mss_deinit_image(&drv->q6->desc, false);
	if (ret < 0)
		pr_err("Unable to free up resources (rc = %d).\n", ret);

	pil_mss_remove_proxy_votes(&drv->q6->desc);
	return ret;
}

static irqreturn_t modem_wdog_bite_intr_handler(int irq, void *dev_id)
{
	struct modem_data *drv = subsys_to_drv(dev_id);
	if (drv->ignore_errors)
		return IRQ_HANDLED;

	pr_err("Watchdog bite received from modem software!\n");
	if (drv->subsys_desc.system_debug &&
			!gpio_get_value(drv->subsys_desc.err_fatal_gpio))
		panic("%s: System ramdump requested. Triggering device restart!\n",
							__func__);
	subsys_set_crash_status(drv->subsys, true);
	restart_modem(drv);
	return IRQ_HANDLED;
}

static int pil_subsys_init(struct modem_data *drv,
					struct platform_device *pdev)
{
	int ret;

#ifdef QUECTEL_EFS_ERROR_BACKUP
    Quectel_modemOnLine_timer_init();
    INIT_WORK(&MomdemOn_timer_work,Quectel_clean_modemFatalTimes);
    INIT_WORK(&ModemFatalError_handler_work,  Quectel_ModemFatalErrProcess);
#else
        ERROR:  Please make sure QUECTEL_EFS_ERROR_BACKUP is defined,is it not need on your project !!
#endif

	drv->subsys_desc.name = "modem";
	drv->subsys_desc.dev = &pdev->dev;
	drv->subsys_desc.owner = THIS_MODULE;
	drv->subsys_desc.shutdown = modem_shutdown;
	drv->subsys_desc.powerup = modem_powerup;
	drv->subsys_desc.ramdump = modem_ramdump;
	drv->subsys_desc.crash_shutdown = modem_crash_shutdown;
	drv->subsys_desc.err_fatal_handler = modem_err_fatal_intr_handler;
	drv->subsys_desc.stop_ack_handler = modem_stop_ack_intr_handler;
	drv->subsys_desc.wdog_bite_handler = modem_wdog_bite_intr_handler;

	drv->q6->desc.modem_ssr = false;
	drv->subsys = subsys_register(&drv->subsys_desc);
	if (IS_ERR(drv->subsys)) {
		ret = PTR_ERR(drv->subsys);
		goto err_subsys;
	}

	drv->q6->desc.subsys_dev = drv->subsys;
	drv->ramdump_dev = create_ramdump_device("modem", &pdev->dev);
	if (!drv->ramdump_dev) {
		pr_err("%s: Unable to create a modem ramdump device.\n",
			__func__);
		ret = -ENOMEM;
		goto err_ramdump;
	}

	return 0;

err_ramdump:
	subsys_unregister(drv->subsys);
err_subsys:
	return ret;
}

static int pil_mss_loadable_init(struct modem_data *drv,
					struct platform_device *pdev)
{
	struct q6v5_data *q6;
	struct pil_desc *q6_desc;
	struct resource *res;
	struct property *prop;
	int ret;

	q6 = pil_q6v5_init(pdev);
	if (IS_ERR_OR_NULL(q6))
		return PTR_ERR(q6);
	drv->q6 = q6;
	drv->xo = q6->xo;

	q6_desc = &q6->desc;
	q6_desc->owner = THIS_MODULE;
	q6_desc->proxy_timeout = PROXY_TIMEOUT_MS;

	q6_desc->ops = &pil_msa_mss_ops;

	q6->self_auth = of_property_read_bool(pdev->dev.of_node,
							"qcom,pil-self-auth");
	if (q6->self_auth) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						    "rmb_base");
		q6->rmb_base = devm_ioremap_resource(&pdev->dev, res);
		if (!q6->rmb_base)
			return -ENOMEM;
		drv->rmb_base = q6->rmb_base;
		q6_desc->ops = &pil_msa_mss_ops_selfauth;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "restart_reg");
	if (!res) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"restart_reg_sec");
		q6->restart_reg_sec = true;
	}

	q6->restart_reg = devm_ioremap_resource(&pdev->dev, res);
	if (!q6->restart_reg)
		return -ENOMEM;

	q6->vreg = NULL;

	prop = of_find_property(pdev->dev.of_node, "vdd_mss-supply", NULL);
	if (prop) {
		q6->vreg = devm_regulator_get(&pdev->dev, "vdd_mss");
		if (IS_ERR(q6->vreg))
			return PTR_ERR(q6->vreg);

		ret = regulator_set_voltage(q6->vreg, VDD_MSS_UV,
						MAX_VDD_MSS_UV);
		if (ret)
			dev_err(&pdev->dev, "Failed to set vreg voltage.\n");

		ret = regulator_set_optimum_mode(q6->vreg, 100000);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to set vreg mode.\n");
			return ret;
		}
	}

	q6->vreg_mx = devm_regulator_get(&pdev->dev, "vdd_mx");
	if (IS_ERR(q6->vreg_mx))
		return PTR_ERR(q6->vreg_mx);
	prop = of_find_property(pdev->dev.of_node, "vdd_mx-uV", NULL);
	if (!prop) {
		dev_err(&pdev->dev, "Missing vdd_mx-uV property\n");
		return -EINVAL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
		"cxrail_bhs_reg");
	if (res)
		q6->cxrail_bhs = devm_ioremap(&pdev->dev, res->start,
					  resource_size(res));

	q6->ahb_clk = devm_clk_get(&pdev->dev, "iface_clk");
	if (IS_ERR(q6->ahb_clk))
		return PTR_ERR(q6->ahb_clk);

	q6->axi_clk = devm_clk_get(&pdev->dev, "bus_clk");
	if (IS_ERR(q6->axi_clk))
		return PTR_ERR(q6->axi_clk);

	q6->rom_clk = devm_clk_get(&pdev->dev, "mem_clk");
	if (IS_ERR(q6->rom_clk))
		return PTR_ERR(q6->rom_clk);

	ret = of_property_read_u32(pdev->dev.of_node,
					"qcom,pas-id", &drv->pas_id);
	if (ret)
		dev_warn(&pdev->dev, "Failed to find the pas_id.\n");

	drv->subsys_desc.pil_mss_memsetup =
	of_property_read_bool(pdev->dev.of_node, "qcom,pil-mss-memsetup");

	/* Optional. */
	if (of_property_match_string(pdev->dev.of_node,
			"qcom,active-clock-names", "gpll0_mss_clk") >= 0)
		q6->gpll0_mss_clk = devm_clk_get(&pdev->dev, "gpll0_mss_clk");

	if (of_property_match_string(pdev->dev.of_node,
			"qcom,active-clock-names", "snoc_axi_clk") >= 0)
		q6->snoc_axi_clk = devm_clk_get(&pdev->dev, "snoc_axi_clk");

	if (of_property_match_string(pdev->dev.of_node,
			"qcom,active-clock-names", "mnoc_axi_clk") >= 0)
		q6->mnoc_axi_clk = devm_clk_get(&pdev->dev, "mnoc_axi_clk");

	ret = pil_desc_init(q6_desc);

	return ret;
}

static int pil_mss_driver_probe(struct platform_device *pdev)
{
	struct modem_data *drv;
	int ret, is_not_loadable;

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;
	platform_set_drvdata(pdev, drv);

	is_not_loadable = of_property_read_bool(pdev->dev.of_node,
							"qcom,is-not-loadable");
	if (is_not_loadable) {
		drv->subsys_desc.is_not_loadable = 1;
	} else {
		ret = pil_mss_loadable_init(drv, pdev);
		if (ret)
			return ret;
	}
	init_completion(&drv->stop_ack);

	/* Probe the MBA mem device if present */
	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret)
		return ret;

	return pil_subsys_init(drv, pdev);
}

static int pil_mss_driver_exit(struct platform_device *pdev)
{
	struct modem_data *drv = platform_get_drvdata(pdev);

	subsys_unregister(drv->subsys);
	destroy_ramdump_device(drv->ramdump_dev);
	pil_desc_release(&drv->q6->desc);
	return 0;
}

static int pil_mba_mem_driver_probe(struct platform_device *pdev)
{
	struct modem_data *drv;

	if (!pdev->dev.parent) {
		pr_err("No parent found.\n");
		return -EINVAL;
	}
	drv = dev_get_drvdata(pdev->dev.parent);
	drv->mba_mem_dev_fixed = &pdev->dev;
	return 0;
}

static struct of_device_id mba_mem_match_table[] = {
	{ .compatible = "qcom,pil-mba-mem" },
	{}
};

static struct platform_driver pil_mba_mem_driver = {
	.probe = pil_mba_mem_driver_probe,
	.driver = {
		.name = "pil-mba-mem",
		.of_match_table = mba_mem_match_table,
		.owner = THIS_MODULE,
	},
};

static struct of_device_id mss_match_table[] = {
	{ .compatible = "qcom,pil-q6v5-mss" },
	{ .compatible = "qcom,pil-q6v55-mss" },
	{ .compatible = "qcom,pil-q6v56-mss" },
	{}
};

static struct platform_driver pil_mss_driver = {
	.probe = pil_mss_driver_probe,
	.remove = pil_mss_driver_exit,
	.driver = {
		.name = "pil-q6v5-mss",
		.of_match_table = mss_match_table,
		.owner = THIS_MODULE,
	},
};

static int __init pil_mss_init(void)
{
	int ret;

	ret = platform_driver_register(&pil_mba_mem_driver);
	if (!ret)
		ret = platform_driver_register(&pil_mss_driver);
	return ret;
}
module_init(pil_mss_init);

static void __exit pil_mss_exit(void)
{
	platform_driver_unregister(&pil_mss_driver);
}
module_exit(pil_mss_exit);

MODULE_DESCRIPTION("Support for booting modem subsystems with QDSP6v5 Hexagon processors");
MODULE_LICENSE("GPL v2");
