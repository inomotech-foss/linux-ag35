// SPDX-License-Identifier: GPL-2.0
/*
 * Early device creation for Qualcomm platforms.
 *
 * On MDM9607 the RPM firmware has a ~4.8 s watchdog.  Several drivers
 * in the SMEM → SMD → RPM chain defer because their dependencies
 * (hwspinlock, APCS mailbox) are created later during the normal
 * of_platform_default_populate() walk at arch_initcall_sync.
 *
 * Work around this by creating the critical platform devices at
 * postcore_initcall so their drivers probe immediately:
 *
 *   - qcom,tcsr-mutex  (hwspinlock) — needed by SMEM
 *   - qcom,msm8916-apcs-kpss-global (APCS mailbox) — needed by SMD,
 *     SMSM, and SMP2P for IPC with the RPM
 *
 * When of_platform_default_populate later encounters these nodes it
 * sees OF_POPULATED already set and harmlessly skips them.
 */

#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

static const struct of_device_id qcom_early_devices[] __initconst = {
	{ .compatible = "qcom,tcsr-mutex" },
	{ .compatible = "qcom,msm8916-apcs-kpss-global" },
	{}
};

static int __init qcom_early_device_init(void)
{
	const struct of_device_id *match;
	struct device_node *np;

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
 * Debug heartbeat — prints a message every 500ms so we can see exactly
 * when the kernel stops being alive before a mysterious reset.
 * Remove once the reset cause is identified.
 */
static struct timer_list heartbeat_timer;
static unsigned int heartbeat_count;

static void heartbeat_fn(struct timer_list *t)
{
	pr_info("heartbeat: alive #%u\n", ++heartbeat_count);
	mod_timer(t, jiffies + msecs_to_jiffies(500));
}

static int __init heartbeat_init(void)
{
	pr_info("heartbeat: starting 500ms debug heartbeat\n");
	timer_setup(&heartbeat_timer, heartbeat_fn, 0);
	mod_timer(&heartbeat_timer, jiffies + msecs_to_jiffies(500));
	return 0;
}
late_initcall(heartbeat_init);
