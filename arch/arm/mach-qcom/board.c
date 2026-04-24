// SPDX-License-Identifier: GPL-2.0
/*
 * Early device creation and modem handshake for Qualcomm MDM9607.
 *
 * On MDM9607 the modem firmware is loaded by SBL1 and starts running
 * from POR.  It expects APPS to complete an SMSM handshake (set
 * SMSM_INIT | SMSM_SMDINIT | SMSM_RPCINIT in the APPS state entry)
 * within ~4.8s from POR.  If APPS doesn't respond, the modem pulls
 * PS_HOLD low and resets the SoC.
 *
 * The normal SMSM driver probes too late (~3.7s from POR on this
 * slow Cortex-A7).  This file provides:
 *
 *   1. Early platform device creation (postcore_initcall) for the
 *      TCSR hwspinlock and APCS mailbox so SMEM/SMD can probe at
 *      arch_initcall_sync without deferring.
 *
 *   2. Early SMSM handshake (subsys_initcall) that writes directly
 *      to the SMEM shared state and kicks the modem via the APCS
 *      IPC register, completing the handshake at ~1.3s from POR.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/soc/qcom/smem.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/sched.h>

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

/* SMEM item IDs — must match drivers/soc/qcom/smsm.c */
#define SMEM_SMSM_SHARED_STATE	85

/* SMSM state bits — must match downstream msm_smsm.h */
#define SMSM_INIT		BIT(0)
#define SMSM_SMDINIT		BIT(3)
#define SMSM_RPCINIT		BIT(5)

/* Number of SMSM entries (apps, modem, adsp, wcnss) */
#define SMSM_NUM_ENTRIES	4

/*
 * APCS IPC register for kicking remote processors.
 * Physical address: APCS base (0x0b011000) + IPC offset (0x8).
 * Bit 13 = modem SMSM notification (from DT: qcom,ipc-1 = <&apcs 8 13>).
 */
#define APCS_IPC_PHYS		0x0b011008
#define APCS_IPC_MODEM_SMSM	BIT(13)

static const struct of_device_id qcom_early_devices[] __initconst = {
	{ .compatible = "qcom,tcsr-mutex" },
	{ .compatible = "qcom,msm8916-apcs-kpss-global" },
	{}
};

/*
 * Enable the APPS KPSS watchdog as early as possible.
 *
 * The RPM firmware monitors the APPS WDT0_EN register.  When it sees
 * the WDT enabled and being petted, it considers APPS alive and holds
 * off its own ~5s supervision timer.  The downstream 3.18 kernel
 * enables WDT at pure_initcall (~0.168s); upstream never enables it
 * until the qcom_wdt driver probes at device_initcall (~2.8s kernel
 * time), which is too late.
 *
 * The qcom_wdt driver handles handoff automatically: if WDT_EN reads
 * back as set, it reconfigures timeouts, sets WDOG_HW_RUNNING, and
 * the watchdog core starts auto-pinging.
 */
static int __init qcom_early_wdt_init(void)
{
	void __iomem *base;

	base = ioremap(KPSS_WDT_PHYS, KPSS_WDT_SIZE);
	if (!base)
		return 0;

	/* Set bark and bite timeouts to 30 seconds */
	writel(30 * WDT_CLK_RATE, base + WDT_BARK_TIME);
	writel(30 * WDT_CLK_RATE, base + WDT_BITE_TIME);

	/* Enable the watchdog */
	writel(1, base + WDT_EN);

	/* Pet it once to start the countdown */
	writel(1, base + WDT_RST);

	iounmap(base);

	pr_info("qcom_early_wdt: APPS WDT enabled (30s timeout)\n");
	return 0;
}
postcore_initcall(qcom_early_wdt_init);

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
 * Early SMSM handshake — tell the modem that APPS is alive before the
 * full SMSM driver has a chance to probe.  This runs at subsys_initcall,
 * right after SMEM probes at arch_initcall_sync.
 */
static int __init qcom_early_smsm_handshake(void)
{
	void __iomem *apcs_ipc;
	u32 *states;
	u32 val;
	int ret;

	if (!qcom_smem_is_available())
		return 0;

	/*
	 * Allocate the SMSM shared state if it doesn't already exist.
	 * The modem firmware usually allocates it first, so expect -EEXIST.
	 */
	ret = qcom_smem_alloc(QCOM_SMEM_HOST_ANY, SMEM_SMSM_SHARED_STATE,
			      SMSM_NUM_ENTRIES * sizeof(u32));
	if (ret < 0 && ret != -EEXIST)
		return 0;

	states = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_SMSM_SHARED_STATE,
			       NULL);
	if (IS_ERR(states))
		return 0;

	/* Set APPS entry (index 0) with the handshake bits */
	val = readl(&states[0]);
	val |= SMSM_INIT | SMSM_SMDINIT | SMSM_RPCINIT;
	writel(val, &states[0]);

	/* Ensure the write is visible before kicking the modem */
	wmb();

	/* Kick modem via APCS IPC register */
	apcs_ipc = ioremap(APCS_IPC_PHYS, 4);
	if (apcs_ipc) {
		writel(APCS_IPC_MODEM_SMSM, apcs_ipc);
		iounmap(apcs_ipc);
	}

	pr_info("qcom_early_smsm: APPS SMSM state set to %#x, modem notified\n",
		val);
	return 0;
}
subsys_initcall(qcom_early_smsm_handshake);

/*
 * Diagnostic RT thread — polls WDT registers every ~10ms using udelay()
 * so it doesn't depend on timer interrupts or softirqs.  Runs as
 * SCHED_FIFO priority 99.  If the CPU is alive and executing, this
 * thread will print; if the prints stop, the CPU was externally reset.
 *
 * On single-core MDM9607, schedule() is called each iteration to let
 * the rest of the system (initcalls, deferred probes) make progress.
 * Remove once the reset cause is identified.
 */
static int heartbeat_thread_fn(void *data)
{
	void __iomem *wdt_base = data;
	unsigned int count = 0;
	u32 sts, en;

	while (!kthread_should_stop()) {
		sts = readl(wdt_base + 0x0C);	/* WDT_STS */
		en = readl(wdt_base + 0x08);	/* WDT_EN */
		pr_info("hb #%u WDT_STS=%#x EN=%#x\n", ++count, sts, en);

		/*
		 * Yield so the single-core system can make progress,
		 * then busy-wait 10ms.  schedule() doesn't need timer
		 * interrupts — it just invokes the scheduler.  The
		 * udelay ensures we don't spin faster than 10ms even
		 * if schedule() returns immediately.
		 */
		schedule();
		mdelay(10);
	}
	return 0;
}

static int __init heartbeat_init(void)
{
	struct task_struct *t;
	void __iomem *wdt_base;

	wdt_base = ioremap(KPSS_WDT_PHYS, KPSS_WDT_SIZE);
	if (!wdt_base) {
		pr_err("heartbeat: failed to map WDT registers\n");
		return -ENOMEM;
	}

	t = kthread_create(heartbeat_thread_fn, wdt_base, "hb_diag");
	if (IS_ERR(t)) {
		iounmap(wdt_base);
		return PTR_ERR(t);
	}

	sched_set_fifo(t);
	wake_up_process(t);

	pr_info("heartbeat: started RT diagnostic thread (10ms udelay poll)\n");
	return 0;
}
late_initcall(heartbeat_init);
