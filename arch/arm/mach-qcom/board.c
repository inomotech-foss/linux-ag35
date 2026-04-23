// SPDX-License-Identifier: GPL-2.0
/*
 * Early device creation for Qualcomm platforms.
 *
 * On MDM9607, the SMEM driver depends on the TCSR hwspinlock.  Both
 * devices are created during of_platform_default_populate() at
 * arch_initcall_sync, but SMEM's device appears first in DT order so it
 * probes (and defers) before the hwspinlock is even registered.  By the
 * time deferred probe re-runs, the RPM firmware watchdog has already
 * fired.
 *
 * Work around this by creating the hwspinlock platform_device at
 * postcore_initcall — the hwspinlock-qcom driver is registered at the
 * same initcall level, so it matches and probes immediately.  When
 * of_platform_default_populate later walks the DT it sees OF_POPULATED
 * already set on this node and skips it.
 */

#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_platform.h>

static int __init qcom_early_hwspinlock_init(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "qcom,tcsr-mutex");
	if (np) {
		of_platform_device_create(np, NULL, NULL);
		of_node_put(np);
	}
	return 0;
}
postcore_initcall(qcom_early_hwspinlock_init);
