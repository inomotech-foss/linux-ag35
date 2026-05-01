/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_ARM_IRQ_WORK_H
#define __ASM_ARM_IRQ_WORK_H

#include <asm/smp_plat.h>
#include <linux/cpumask.h>

static inline bool arch_irq_work_has_interrupt(void)
{
	/*
	 * Self-IPI via smp_cross_call() requires that IPI infrastructure
	 * has been set up by bringing secondary CPUs online.  On systems
	 * where SMP is configured but only one CPU is present (e.g.
	 * Qualcomm MDM9607), the GIC SGI routing for self-IPI may not
	 * work.  Fall back to tick-based irq_work processing in that case.
	 */
	return is_smp() && num_online_cpus() > 1;
}

#endif /* _ASM_ARM_IRQ_WORK_H */
