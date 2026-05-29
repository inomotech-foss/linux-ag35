// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm Emergency Download (EDL) reboot handler
 *
 * Triggers EDL/Sahara on `reboot edl` by writing SCM_EDLOAD_MODE to the
 * TCSR boot-misc-detect register (the path used by the downstream LK/SBL
 * on mdm9607) and the IMEM EDL cookies that older PBLs check.
 *
 * On mdm9607 the SCM SVC_BOOT/SET_DLOAD_MODE call is rejected by TZ
 * (returns QCOM_SCM_ERROR), so the only working path is the TCSR write
 * via the always-available SCM_SVC_IO/IO_WRITE call.
 *
 * Runs as a sys_off_handler at SYS_OFF_PRIO_HIGH so it executes after
 * device_shutdown() (which otherwise lets qcom_scm clear the TCSR) and
 * before the pshold restart handler (priority 128) drops PS_HOLD.
 */

#include <linux/delay.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/string.h>

/* IMEM EDL magic cookies (written at edload_offset) */
#define EDLOAD_MAGIC_0		0x322A4F99
#define EDLOAD_MAGIC_1		0xC67E4350
#define EDLOAD_MAGIC_2		0x77777777

/* SCM dload mode types written to TCSR_BOOT_MISC_DETECT */
#define SCM_EDLOAD_MODE		0x01
#define SCM_DLOAD_MODE		0x10

struct qcom_edl_reboot {
	struct regmap *imem;
	u32 edload_offset;
	phys_addr_t tcsr_boot_misc;
};

static int qcom_edl_restart_handler(struct sys_off_data *data)
{
	struct qcom_edl_reboot *edl = data->cb_data;
	int ret;

	if (!data->cmd || strcmp(data->cmd, "edl"))
		return NOTIFY_DONE;

	pr_emerg("qcom-edl-reboot: entering EDL handler\n");

	/*
	 * Write SCM_EDLOAD_MODE (0x01) to TCSR_BOOT_MISC_DETECT via SCM IO.
	 * device_shutdown() ran qcom_scm_shutdown() which cleared the dump
	 * bits; restore the EDL flag here so PBL sees it on warm reset.
	 */
	ret = qcom_scm_io_writel(edl->tcsr_boot_misc, SCM_EDLOAD_MODE);
	pr_emerg("qcom-edl-reboot: TCSR write returned %d\n", ret);

	/* Write IMEM EDL magic cookies (older PBLs check these) */
	regmap_write(edl->imem, edl->edload_offset + 0, EDLOAD_MAGIC_0);
	regmap_write(edl->imem, edl->edload_offset + 4, EDLOAD_MAGIC_1);
	regmap_write(edl->imem, edl->edload_offset + 8, EDLOAD_MAGIC_2);

	pr_emerg("qcom-edl-reboot: cookies written, dropping PS_HOLD\n");

	/* Give the UART time to flush before pshold drops the rail */
	mdelay(200);

	return NOTIFY_DONE;
}

static int qcom_edl_reboot_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qcom_edl_reboot *edl;
	u64 addr;
	int ret;

	edl = devm_kzalloc(dev, sizeof(*edl), GFP_KERNEL);
	if (!edl)
		return -ENOMEM;

	edl->imem = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(edl->imem))
		return dev_err_probe(dev, PTR_ERR(edl->imem),
				     "no parent IMEM syscon\n");

	ret = of_property_read_u32(dev->of_node, "qcom,edload-offset",
				   &edl->edload_offset);
	if (ret)
		return dev_err_probe(dev, ret, "missing 'qcom,edload-offset'\n");

	ret = of_property_read_u64(dev->of_node, "qcom,tcsr-boot-misc", &addr);
	if (ret)
		return dev_err_probe(dev, ret, "missing 'qcom,tcsr-boot-misc'\n");
	edl->tcsr_boot_misc = (phys_addr_t)addr;

	return devm_register_sys_off_handler(dev, SYS_OFF_MODE_RESTART,
					     SYS_OFF_PRIO_HIGH,
					     qcom_edl_restart_handler, edl);
}

static const struct of_device_id qcom_edl_reboot_of_match[] = {
	{ .compatible = "qcom,edl-reboot" },
	{}
};
MODULE_DEVICE_TABLE(of, qcom_edl_reboot_of_match);

static struct platform_driver qcom_edl_reboot_driver = {
	.probe = qcom_edl_reboot_probe,
	.driver = {
		.name = "qcom-edl-reboot",
		.of_match_table = qcom_edl_reboot_of_match,
	},
};
module_platform_driver(qcom_edl_reboot_driver);

MODULE_DESCRIPTION("Qualcomm Emergency Download (EDL) reboot handler");
MODULE_LICENSE("GPL");
