// SPDX-License-Identifier: GPL-2.0
/*
 * Early device creation and diagnostics for Qualcomm MDM9607.
 *
 * Provides:
 *   1. Early platform device creation (postcore_initcall) for the
 *      TCSR hwspinlock and APCS mailbox so SMEM/SMD can probe at
 *      arch_initcall_sync without deferring.
 *
 *   2. Panic/reboot notifiers for crash diagnostics.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/panic_notifier.h>
#include <linux/kdebug.h>

/*
 * KPSS WDT register offsets (from reg_offset_data_kpss in qcom-wdt.c).
 * Physical base: 0x0b017000 (from DTS watchdog@b017000).
 */
#define KPSS_WDT_PHYS		0x0b017000
#define KPSS_WDT_SIZE		0x40
#define WDT_RST			0x04
#define WDT_EN			0x08
#define WDT_BARK_TIME		0x10
#define WDT_BITE_TIME		0x14

/* Sleep clock rate (from DTS: clocks = <&sleep_clk>, 32768 Hz) */
#define WDT_CLK_RATE		32768

static const struct of_device_id qcom_early_devices[] __initconst = {
	{ .compatible = "qcom,tcsr-mutex" },
	{ .compatible = "qcom,msm8916-apcs-kpss-global" },
	{}
};

static int __init qcom_early_device_init(void)
{
	const struct of_device_id *match;
	struct device_node *np;

	pr_info("qcom_early: boot_command_line: %s\n", boot_command_line);

	for (match = qcom_early_devices; match->compatible[0]; match++) {
		np = of_find_compatible_node(NULL, NULL, match->compatible);
		if (np) {
			of_platform_device_create(np, NULL, NULL);
			of_node_put(np);
		}
	}
	return 0;
}
postcore_initcall(qcom_early_device_init);

/*
 * Panic and die notifiers — distinguish kernel panic / oops from an
 * external RPM-driven PS_HOLD reset.  If the panic notifier fires we
 * stamp a magic value into IMEM reboot-mode so the next boot can tell
 * it was a kernel crash.
 */
#define IMEM_REBOOT_MODE_PHYS	0x0860065c
static void __iomem *imem_reboot_mode;

static int qcom_panic_event(struct notifier_block *nb, unsigned long action,
			    void *data)
{
	pr_emerg(">>> PANIC NOTIFIER FIRED — kernel panic caused this reset <<<\n");
	if (imem_reboot_mode)
		writel(0xDEAD0A1C, imem_reboot_mode);
	return NOTIFY_DONE;
}

static struct notifier_block qcom_panic_nb = {
	.notifier_call = qcom_panic_event,
	.priority = 255,
};

static int qcom_die_notify(struct notifier_block *nb, unsigned long action,
			   void *data)
{
	struct die_args *args = data;

	if (action == DIE_OOPS) {
		pr_emerg(">>> DIE_OOPS: PC=%pS LR=%pS <<<\n",
			 (void *)instruction_pointer(args->regs),
			 (void *)args->regs->ARM_lr);
	}
	return NOTIFY_DONE;
}

static struct notifier_block qcom_die_nb = {
	.notifier_call = qcom_die_notify,
	.priority = 255,
};

static int __init qcom_panic_notifier_init(void)
{
	imem_reboot_mode = ioremap(IMEM_REBOOT_MODE_PHYS, 4);
	atomic_notifier_chain_register(&panic_notifier_list, &qcom_panic_nb);
	register_die_notifier(&qcom_die_nb);
	pr_info("qcom_panic_notifier: registered (imem=%p)\n", imem_reboot_mode);
	return 0;
}
postcore_initcall(qcom_panic_notifier_init);

/*
 * Debug heartbeat — prints WDT state and pets the watchdog every 500ms.
 * Remove once boot is stable.
 */
static struct timer_list heartbeat_timer;
static void __iomem *hb_wdt_base;

static void heartbeat_fn(struct timer_list *t)
{
	if (hb_wdt_base)
		writel(1, hb_wdt_base + WDT_RST);	/* pet the WDT */
	mod_timer(t, jiffies + msecs_to_jiffies(500));
}

static int __init heartbeat_init(void)
{
	hb_wdt_base = ioremap(KPSS_WDT_PHYS, KPSS_WDT_SIZE);
	timer_setup(&heartbeat_timer, heartbeat_fn, 0);
	mod_timer(&heartbeat_timer, jiffies + msecs_to_jiffies(500));
	return 0;
}
device_initcall(heartbeat_init);
