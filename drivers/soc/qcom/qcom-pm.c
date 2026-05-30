// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM/MDM platform suspend driver.
 *
 * Provides PM_SUSPEND_MEM ("deep" / "mem") for older Qualcomm SoCs that
 * have an SPM/SAW2 power controller but no PSCI firmware (e.g. MDM9607).
 *
 * The SoC does not actually power-collapse its single CPU during system
 * suspend on this driver: TZ TERMINATE_PC never returns on this part.
 * Instead, we arm the SAW2 sequencer in PC mode with SLP_CMD_MODE set,
 * then execute WFI.  The SAW2 then:
 *
 *   1. Detects WFI on the AP side.
 *   2. Asserts SLP_CMD over the RPM IPC bit (SLP_CMD_MODE in SPM_CTL).
 *   3. Waits for RPM sleep ACK -- RPM uses this signal to apply its
 *      aggregated sleep set: PMIC sleep voltages, XO off, sleep clock
 *      only, etc.  This is the single largest source of suspend power
 *      savings on this SoC.
 *   4. Clock-gates the APSS.
 *   5. On wake IRQ (delivered via the MPM while RPM transitions back to
 *      active set), the SAW2 walks the reverse sequence and the CPU
 *      resumes from WFI.
 *
 * The CPU itself stays powered (just clock-gated), so no warm-boot vector
 * setup and no SCM call are needed -- both of which are what tripped the
 * earlier cpuidle-style SPC attempts on this device.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>

#include <soc/qcom/spm.h>

#include <asm/proc-fns.h>

static struct spm_driver_data *qcom_pm_boot_spm;

static int qcom_pm_enter(suspend_state_t state)
{
	/*
	 * Arm SAW2 with the PC sequence (PC_MODE | SLP_CMD_MODE).  Once the
	 * core enters WFI below, SAW2 will run the sequence -- including the
	 * RPM sleep handshake that triggers PMIC sleep voltages and XO off.
	 */
	spm_set_low_power_mode(qcom_pm_boot_spm, PM_SLEEP_MODE_PC);

	/*
	 * IRQs are already masked at the CPU by the suspend core; a pending
	 * wakeup IRQ will exit WFI without being delivered, and we re-enable
	 * IRQs on the way back out so the handler runs after we restore
	 * SAW2 to standby.
	 */
	cpu_do_idle();

	/*
	 * Reset SPM so that any incidental WFI from cpuidle after resume
	 * doesn't accidentally re-trigger the PC sequence.
	 */
	spm_set_low_power_mode(qcom_pm_boot_spm, PM_SLEEP_MODE_STBY);

	return 0;
}

static const struct platform_suspend_ops qcom_pm_ops = {
	.enter = qcom_pm_enter,
	.valid = suspend_valid_only_mem,
};

static int qcom_pm_probe(struct platform_device *pdev)
{
	struct device_node *cpu_node, *saw_node;
	struct platform_device *saw_pdev;

	cpu_node = of_get_cpu_node(0, NULL);
	if (!cpu_node)
		return -ENODEV;

	saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
	of_node_put(cpu_node);
	if (!saw_node)
		return -ENODEV;

	saw_pdev = of_find_device_by_node(saw_node);
	of_node_put(saw_node);
	if (!saw_pdev)
		return -EPROBE_DEFER;

	qcom_pm_boot_spm = dev_get_drvdata(&saw_pdev->dev);
	put_device(&saw_pdev->dev);
	if (!qcom_pm_boot_spm)
		return -EPROBE_DEFER;

	suspend_set_ops(&qcom_pm_ops);
	dev_info(&pdev->dev,
		 "registered PM_SUSPEND_MEM via SAW2 PC + RPM SLP_CMD\n");
	return 0;
}

static struct platform_driver qcom_pm_driver = {
	.probe = qcom_pm_probe,
	.driver = {
		.name = "qcom-pm",
		.suppress_bind_attrs = true,
	},
};

static bool __init qcom_pm_have_saw(void)
{
	struct device_node *cpu_node, *saw_node;
	bool ok = false;

	cpu_node = of_get_cpu_node(0, NULL);
	if (!cpu_node)
		return false;

	saw_node = of_parse_phandle(cpu_node, "qcom,saw", 0);
	of_node_put(cpu_node);
	if (saw_node && of_device_is_available(saw_node))
		ok = true;
	of_node_put(saw_node);

	return ok;
}

static int __init qcom_pm_init(void)
{
	struct platform_device *pdev;
	int ret;

	if (!qcom_pm_have_saw())
		return 0;

	ret = platform_driver_register(&qcom_pm_driver);
	if (ret)
		return ret;

	pdev = platform_device_register_simple("qcom-pm", -1, NULL, 0);
	if (IS_ERR(pdev)) {
		platform_driver_unregister(&qcom_pm_driver);
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(qcom_pm_init);

MODULE_DESCRIPTION("Qualcomm MSM platform suspend driver");
MODULE_LICENSE("GPL");
