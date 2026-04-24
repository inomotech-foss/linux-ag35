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

/* SMEM item IDs — must match drivers/soc/qcom/smsm.c */
#define SMEM_SMSM_SHARED_STATE	85

/* SMSM state bits — must match downstream msm_smsm.h */
#define SMSM_INIT		BIT(0)
#define SMSM_SMDINIT		BIT(3)
#define SMSM_RPCINIT		BIT(5)
#define SMSM_PROC_AWAKE	BIT(12)

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
	val |= SMSM_INIT | SMSM_SMDINIT | SMSM_RPCINIT | SMSM_PROC_AWAKE;
	writel(val, &states[0]);

	/* Ensure the write is visible before kicking the modem */
	wmb();

	/* Kick modem via APCS IPC register */
	apcs_ipc = ioremap(APCS_IPC_PHYS, 4);
	if (apcs_ipc) {
		writel(APCS_IPC_MODEM_SMSM, apcs_ipc);
		iounmap(apcs_ipc);
	}

	pr_info("qcom_early_smsm: APPS SMSM state set to %#x (INIT|SMDINIT|RPCINIT|PROC_AWAKE), modem notified\n",
		val);
	return 0;
}
subsys_initcall(qcom_early_smsm_handshake);

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
static unsigned int heartbeat_count;
static void __iomem *hb_wdt_base;

static void heartbeat_fn(struct timer_list *t)
{
	u32 sts = 0, en = 0;

	if (hb_wdt_base) {
		sts = readl(hb_wdt_base + 0x0C);	/* WDT_STS */
		en = readl(hb_wdt_base + 0x08);		/* WDT_EN */
		writel(1, hb_wdt_base + WDT_RST);	/* pet the WDT */
	}
	pr_info("hb #%u WDT_STS=%#x EN=%#x\n", ++heartbeat_count, sts, en);
	mod_timer(t, jiffies + msecs_to_jiffies(500));
}

static int __init heartbeat_init(void)
{
	hb_wdt_base = ioremap(KPSS_WDT_PHYS, KPSS_WDT_SIZE);
	pr_info("heartbeat: starting 500ms debug heartbeat (with WDT pet)\n");
	timer_setup(&heartbeat_timer, heartbeat_fn, 0);
	mod_timer(&heartbeat_timer, jiffies + msecs_to_jiffies(500));
	return 0;
}
late_initcall(heartbeat_init);
