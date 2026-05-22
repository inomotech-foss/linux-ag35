// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm BAM-DMUX WWAN network driver
 *
 * "Nuclear" rewrite: direct BAM register access, no DMA engine dependency.
 *
 * Attempt 37: Fix double first_connect race (modem crash + kernel panic):
 *
 *  Attempt 36 proved that register ordering + timing are correct:
 *    P5 CTRL=0x2a, P4 CTRL=0x22, SW_RST+P_RST work, modem responds.
 *  BUT modem crashed with "a2_power.c:2556:A2 Assertion Failed" because
 *  first_connect was called TWICE (boot_work + pc_irq race).
 *  Fixed with pipes_active re-entry guard at top of first_connect.
 *
 * Attempt 38: Fix APPS bit 1 timing (pipe 5 RX stuck at 0):
 *
 *  Attempt 37 fixed the crash.  TX works (CMD_OPENs consumed by modem).
 *  But pipe 5 RX stayed at SW=0 — modem never sends data back.
 *
 *  Root cause: APPS bit 1 was set BEFORE pipe 5 was configured.
 *  The IPC to modem takes ~70ms (not 1-5ms as assumed).  Modem's
 *  a2_apps_smsm_callback fires, sets apps_bam_link_ready=true, and
 *  modem immediately sends CMD_OPEN response on pipe 5.  But pipe 5
 *  was still being configured (P_EN=0 until 136ms after bit 1 set).
 *  Data hitting a disabled producer pipe is LOST.
 *
 *  The modem's a2_sio_channel_open() only responds to the FIRST
 *  CMD_OPEN per channel (subsequent ones just increment open_count).
 *  So our CMD_OPEN retries never get a response either.
 *
 *  Fix: Set APPS bit 1 as the LAST step, after pipes are configured
 *  and 32 RX descriptors are queued.  By the time modem receives the
 *  IPC (70ms later), pipe 5 has been ready for 70ms+.
 *
 * This driver maps BAM registers directly (like downstream SPS bam.c)
 * and manages descriptor FIFOs in software (like downstream sps_bam.c).
 * It matches the downstream 3.18 kernel's exact register write sequence.
 *
 * Copyright (c) 2020, Stephan Gerhold <stephan@gerhold.net>
 * Copyright (c) 2025, inomotech
 */

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/notifier.h>
#include <linux/remoteproc/qcom_rproc.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <net/pkt_sched.h>

/* ──────────────────────────────────────────────────────────────────────────
 * BAM Register Definitions — NDP_4K layout (BAM v1.7.0)
 *
 * Register offsets from Qualcomm BAM v1.7.0 hardware specification.
 * Pipe registers use base + pipe * 0x1000 stride.
 * ────────────────────────────────────────────────────────────────────────── */

/* Global BAM registers */
#define BAM_CTRL			0x00000
#define BAM_REVISION			0x01000
#define BAM_NUM_PIPES			0x01008
#define BAM_DESC_CNT_TRSHLD		0x00008
#define BAM_IRQ_STTS			0x00014
#define BAM_IRQ_CLR			0x00018
#define BAM_IRQ_EN			0x0001C
#define BAM_CNFG_BITS			0x0007C
#define BAM_IRQ_SRCS_EE(ee)		(0x03000 + (ee) * 0x1000)
#define BAM_IRQ_SRCS_MSK_EE(ee)	(0x03004 + (ee) * 0x1000)
#define BAM_PIPE_ATTR_EE(ee)		(0x0300C + (ee) * 0x1000)

/* Per-pipe registers — pipe offset = 0x13000 + pipe * 0x1000 */
#define BAM_P_CTRL(p)			(0x13000 + (p) * 0x1000)
#define BAM_P_RST(p)			(0x13004 + (p) * 0x1000)
#define BAM_P_HALT(p)			(0x13008 + (p) * 0x1000)
#define BAM_P_IRQ_STTS(p)		(0x13010 + (p) * 0x1000)
#define BAM_P_IRQ_CLR(p)		(0x13014 + (p) * 0x1000)
#define BAM_P_IRQ_EN(p)			(0x13018 + (p) * 0x1000)
/* Trust/security registers — in BAM v1.7.0 these are in a separate block */
#define BAM_TRUST_REG			0x2000
#define BAM_P_TRUST_REG(p)		(0x2020 + (p) * 0x4)

/* Per-pipe event registers — event offset = 0x13800 + pipe * 0x1000 */
#define BAM_P_SW_OFSTS(p)		(0x13800 + (p) * 0x1000)
#define BAM_P_EVNT_REG(p)		(0x13818 + (p) * 0x1000)
#define BAM_P_DESC_FIFO_ADDR(p)		(0x1381C + (p) * 0x1000)
#define BAM_P_FIFO_SIZES(p)		(0x13820 + (p) * 0x1000)
#define BAM_P_EVNT_GEN_TRSHLD(p)	(0x13828 + (p) * 0x1000)

/* BAM2BAM registers (to detect modem pipe config) */
#define BAM_P_DATA_FIFO_ADDR(p)		(0x13824 + (p) * 0x1000)
#define BAM_P_EVNT_DEST_ADDR(p)		(0x1382C + (p) * 0x1000)

/* Producer/Consumer sideband registers — NDP_4K layout */
#define BAM_P_PRDCR_SDBND(p)		(0x13024 + (p) * 0x1000)
#define BAM_P_CNSMR_SDBND(p)		(0x13028 + (p) * 0x1000)

/* BAM_CTRL bits */
#define BAM_SW_RST			BIT(0)
#define BAM_EN				BIT(1)

/* BAM_CNFG_BITS default — all workarounds on except BAM_FULL_PIPE */
#define BAM_CNFG_BITS_DEFAULT		0xFFFFF7FF

/* BAM_IRQ_EN bits */
#define BAM_IRQ_TIMER_EN		BIT(4)
#define BAM_IRQ_ERROR_EN		BIT(2)
#define BAM_IRQ_HRESP_ERR_EN		BIT(1)
#define BAM_IRQ_MSK			BIT(31)

/* P_CTRL bits */
#define P_EN				BIT(1)
#define P_DIRECTION			BIT(3)
/*
 * P_SYS_MODE (bit 5): BAM_PIPE_MODE_SYSTEM = 1 in the downstream SPS.
 * The downstream does: bam_write_reg_field(P_CTRL, P_SYS_MODE, 1)
 * which SETS bit 5 for system-mode pipes.
 * BIT(5) clear = BAM2BAM mode (BAM_PIPE_MODE_BAM2BAM = 0).
 */
#define P_SYS_MODE			BIT(5)

/* P_SW_OFSTS */
#define P_SW_OFSTS_MASK			0xFFFF

/* P_IRQ_EN bits */
#define P_TRNSFR_END_EN			BIT(5)

/* Descriptor count threshold — downstream uses 0x1000 (A2_SUMMING_THRESHOLD=4096) */
#define BAM_DESC_CNT_TRSHLD_VAL		0x1000

/* ──────────────────────────────────────────────────────────────────────────
 * BAM Descriptor Hardware Format
 * ────────────────────────────────────────────────────────────────────────── */

/* Descriptor flags (upper 16 bits of size_flags) */
#define DESC_FLAG_INT			BIT(15)
#define DESC_FLAG_EOT			BIT(14)
#define DESC_FLAG_EOB			BIT(13)

struct bam_desc_hw {
	__le32 addr;
	__le16 size;
	__le16 flags;
} __packed;

/* Descriptor FIFO: 2KB = 256 entries (matches downstream SPS default) */
#define BAM_DESC_FIFO_SIZE		SZ_2K
#define BAM_NUM_DESCS			(BAM_DESC_FIFO_SIZE / sizeof(struct bam_desc_hw))

/* ──────────────────────────────────────────────────────────────────────────
 * BAM-DMUX Protocol Constants
 * ────────────────────────────────────────────────────────────────────────── */

#define BAM_DMUX_BUFFER_SIZE		SZ_2K
#define BAM_DMUX_HDR_SIZE		sizeof(struct bam_dmux_hdr)
#define BAM_DMUX_MAX_DATA_SIZE		(BAM_DMUX_BUFFER_SIZE - BAM_DMUX_HDR_SIZE)
#define BAM_DMUX_NUM_SKB		32
#define BAM_DMUX_HDR_MAGIC		0x33fc
#define BAM_DMUX_AUTOSUSPEND_DELAY	1000
#define BAM_DMUX_REMOTE_TIMEOUT		msecs_to_jiffies(2000)

#define BAM_DMUX_TX_PIPE		4
#define BAM_DMUX_RX_PIPE		5
#define BAM_DMUX_EE			0

enum {
	BAM_DMUX_CMD_DATA,
	BAM_DMUX_CMD_OPEN,
	BAM_DMUX_CMD_CLOSE,
};

enum {
	BAM_DMUX_CH_DATA_0,
	BAM_DMUX_CH_DATA_1,
	BAM_DMUX_CH_DATA_2,
	BAM_DMUX_CH_DATA_3,
	BAM_DMUX_CH_DATA_4,
	BAM_DMUX_CH_DATA_5,
	BAM_DMUX_CH_DATA_6,
	BAM_DMUX_CH_DATA_7,
	BAM_DMUX_NUM_CH
};

struct bam_dmux_hdr {
	u16 magic;
	u8 signal;
	u8 cmd;
	u8 pad;
	u8 ch;
	u16 len;
};

/* ──────────────────────────────────────────────────────────────────────────
 * Per-pipe descriptor ring state (managed in software)
 * ────────────────────────────────────────────────────────────────────────── */
struct bam_pipe {
	struct bam_desc_hw *desc_fifo;	/* Descriptor FIFO (DMA coherent) */
	dma_addr_t desc_fifo_phys;	/* Physical address of FIFO */
	u32 desc_offset;		/* Next write position (bytes) */
	u32 sw_offset;			/* Last read P_SW_OFSTS (bytes) */
	u32 pipe_index;			/* Hardware pipe number (4 or 5) */

	/* Per-descriptor user cookies for completion callbacks */
	void *user_data[BAM_NUM_DESCS];
};

/* ──────────────────────────────────────────────────────────────────────────
 * Main driver structure
 * ────────────────────────────────────────────────────────────────────────── */
struct bam_dmux_skb_dma {
	struct bam_dmux *dmux;
	struct sk_buff *skb;
	dma_addr_t addr;
	u16 len;
};

struct bam_dmux {
	struct device *dev;
	void __iomem *bam_base;		/* BAM register base */
	int bam_irq;			/* BAM hardware IRQ */
	bool bam_irq_registered;

	/* SMSM power control */
	int pc_irq;
	bool pc_state, pc_ack_state;
	struct qcom_smem_state *pc, *pc_ack;
	u32 pc_mask, pc_ack_mask;
	wait_queue_head_t pc_wait;
	struct completion pc_ack_completion;

	/* BAM pipes — direct register access */
	struct bam_pipe tx_pipe;
	struct bam_pipe rx_pipe;
	bool pipes_active;

	/* SKB management */
	struct bam_dmux_skb_dma rx_skbs[BAM_DMUX_NUM_SKB];
	struct bam_dmux_skb_dma tx_skbs[BAM_DMUX_NUM_SKB];
	spinlock_t tx_lock;
	unsigned int tx_next_skb;
	atomic_long_t tx_deferred_skb;
	struct work_struct tx_wakeup_work;

	/* Boot / lifecycle */
	struct delayed_work boot_work;
	struct delayed_work ack_work;	/* Delayed ACK toggle after pc_irq */
	struct timer_list rx_poll_timer;
	bool boot_done;

	/* CMD_OPEN retry — modem A2 DMA needs time to initialize */
	unsigned int cmd_open_retries;
	unsigned long cmd_open_next_retry;
	bool cmd_open_acked;

	DECLARE_BITMAP(remote_channels, BAM_DMUX_NUM_CH);
	struct work_struct register_netdev_work;
	struct net_device *netdevs[BAM_DMUX_NUM_CH];

	/*
	 * Modem SSR (mpss) notifier.  Without this, when the modem crashes
	 * (e.g. "a2_power.c assertion") the q6v5 driver tears down the BAM
	 * clock/power domain while our rx_poll_timer is still armed.  The
	 * next bam_clear_irqs() then takes an external abort on the now-
	 * dead AHB slave and panics the kernel from softirq context.
	 */
	struct notifier_block ssr_nb;
	void *ssr_cookie;
};

struct bam_dmux_netdev {
	struct bam_dmux *dmux;
	u8 ch;
};

/* ──────────────────────────────────────────────────────────────────────────
 * BAM Register Access Helpers
 * ────────────────────────────────────────────────────────────────────────── */

static inline u32 bam_readl(struct bam_dmux *dmux, u32 off)
{
	return readl_relaxed(dmux->bam_base + off);
}

static inline void bam_writel(struct bam_dmux *dmux, u32 off, u32 val)
{
	writel_relaxed(val, dmux->bam_base + off);
}

static inline void bam_writel_sync(struct bam_dmux *dmux, u32 off, u32 val)
{
	writel(val, dmux->bam_base + off);
}

/* ──────────────────────────────────────────────────────────────────────────
 * SMSM Power Control
 * ────────────────────────────────────────────────────────────────────────── */

static void bam_dmux_pc_vote(struct bam_dmux *dmux, bool enable)
{
	reinit_completion(&dmux->pc_ack_completion);
	qcom_smem_state_update_bits(dmux->pc, dmux->pc_mask,
				    enable ? dmux->pc_mask : 0);
}

static void bam_dmux_pc_ack(struct bam_dmux *dmux)
{
	qcom_smem_state_update_bits(dmux->pc_ack, dmux->pc_ack_mask,
				    dmux->pc_ack_state ? 0 : dmux->pc_ack_mask);
	dmux->pc_ack_state = !dmux->pc_ack_state;
}

/* ──────────────────────────────────────────────────────────────────────────
 * BAM Hardware Init — Attempt 22: NO SW_RST, preserve modem's BAM state
 *
 * Previous attempts all did SW_RST which wipes the ENTIRE BAM including
 * whatever the modem configured during a2_bam_init().  While the downstream
 * SPS also does SW_RST, something about our re-initialization is different
 * enough that the modem's producer side of pipe 5 never works afterward.
 *
 * New approach: preserve the modem's BAM_CTRL (including its
 * LOCAL_CLK_GATING=2 setting), just add BAM_EN and set CNFG_BITS +
 * DESC_CNT_TRSHLD + IRQ config.  The individual pipe P_RSTs in
 * bam_pipe_hw_init() are still performed since the downstream also
 * does those.
 *
 * Key difference from attempt 18 (which also skipped SW_RST):
 *   - Attempt 18 was MISSING CNFG_BITS (left at 0)
 *   - Attempt 18 cleared APPS bit 1 before ACK
 *   - Attempt 18 had wrong LOCAL_CLK_GATING (forced BIT(17))
 *   This attempt preserves modem's BAM_CTRL and adds CNFG_BITS.
 * ────────────────────────────────────────────────────────────────────────── */

/* BAM_CTRL field masks (NDP BAM) */
#define BAM_CACHE_MISS_ERR_RESP_EN	BIT(19)
#define BAM_LOCAL_CLK_GATING_MASK	(BIT(17) | BIT(16))

static void __maybe_unused bam_hw_init(struct bam_dmux *dmux)
{
	u32 val;

	dev_info(dmux->dev,
		 "bam_hw_init: PRE BAM_CTRL=0x%08x CNFG=0x%08x\n",
		 bam_readl(dmux, BAM_CTRL),
		 bam_readl(dmux, BAM_CNFG_BITS));

	/*
	 * Step 1: SW_RST — matching downstream sps_bam_enable() → bam_init().
	 *
	 * MDM9607 DTS has no "qcom,satellite-mode" so downstream SPS treats
	 * this BAM as locally managed and does full SW_RST on every reconnect.
	 * The modem's P5 PRE state is all zeros — modem never configures the
	 * APPS-side pipe registers.  SW_RST is safe and expected.
	 */
	bam_writel_sync(dmux, BAM_CTRL, BAM_SW_RST);
	bam_writel_sync(dmux, BAM_CTRL, 0);

	/*
	 * Step 2: BAM_EN + LOCAL_CLK_GATING=1 (matching downstream bam_init).
	 *
	 * Downstream: bam_write_reg_field(CTRL, LOCAL_CLK_GATING, 1) sets
	 * bits[17:16] = 1 (= BIT(16)).  Clear CACHE_MISS_ERR_RESP_EN.
	 */
	val = BAM_EN | BIT(16);  /* LOCAL_CLK_GATING = 1 */
	bam_writel(dmux, BAM_CTRL, val);

	/* Step 3: Descriptor count threshold */
	bam_writel(dmux, BAM_DESC_CNT_TRSHLD, BAM_DESC_CNT_TRSHLD_VAL);

	/* Step 4: Config bits — all workarounds enabled except bit 11 */
	bam_writel(dmux, BAM_CNFG_BITS, BAM_CNFG_BITS_DEFAULT);

	/* Step 5: Global IRQ mask for EE 0 — set BAM_IRQ bit */
	val = bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE));
	val |= BAM_IRQ_MSK;
	bam_writel(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE), val);

	/* Step 6: BAM-level IRQ enable */
	bam_writel(dmux, BAM_IRQ_EN,
		   BAM_IRQ_TIMER_EN | BAM_IRQ_ERROR_EN | BAM_IRQ_HRESP_ERR_EN);

	dev_info(dmux->dev,
		 "bam_hw_init: POST BAM_CTRL=0x%08x CNFG=0x%08x\n",
		 bam_readl(dmux, BAM_CTRL),
		 bam_readl(dmux, BAM_CNFG_BITS));
}

/* ──────────────────────────────────────────────────────────────────────────
 * Pipe Init — matching downstream SPS bam_pipe_init() EXACTLY
 *
 * Attempt 36 fix: Match downstream register write ORDER precisely.
 * Downstream sequence (from sps/bam.c bam_pipe_init):
 *   1. P_RST (reset pipe state machine)
 *   2. IRQ_SRCS_MSK_EE |= pipe bit (unmask pipe IRQ)
 *   3. P_IRQ_EN = irq mask
 *   4. P_CTRL.P_DIRECTION = dir (WITHOUT P_EN!)
 *   5. P_CTRL.P_SYS_MODE = mode (WITHOUT P_EN!)
 *   6. P_EVNT_GEN_TRSHLD = threshold
 *   7. P_DESC_FIFO_ADDR = phys
 *   8. P_FIFO_SIZES.P_DESC_FIFO_SIZE = size
 *   9. P_CTRL.P_SYS_STRM = 0
 *  10. P_CTRL.P_LOCK_GROUP = 0
 *  11. P_CTRL.P_EN = 1 (LAST — after all config is set)
 *
 * Previous attempts set P_EN in the same write as DIR and SYS_MODE,
 * AFTER configuring FIFO.  The hardware may interpret FIFO addresses
 * differently depending on the current direction setting.  Writing
 * FIFO config while direction=0 (consumer default after P_RST) may
 * set up the wrong internal DMA path for a producer pipe (pipe 5).
 * ────────────────────────────────────────────────────────────────────────── */

static int bam_pipe_hw_init(struct bam_dmux *dmux, struct bam_pipe *pipe,
			    u32 pipe_index, bool producer)
{
	u32 val;

	pipe->pipe_index = pipe_index;
	pipe->desc_offset = 0;
	pipe->sw_offset = 0;

	/* Allocate descriptor FIFO (DMA coherent memory) */
	pipe->desc_fifo = dma_alloc_coherent(dmux->dev, BAM_DESC_FIFO_SIZE,
					     &pipe->desc_fifo_phys, GFP_KERNEL);
	if (!pipe->desc_fifo)
		return -ENOMEM;

	memset(pipe->desc_fifo, 0, BAM_DESC_FIFO_SIZE);

	/* Step 1: P_RST — reset the pipe state machine.
	 * Downstream bam_pipe_init() always does P_RST.
	 */
	bam_writel_sync(dmux, BAM_P_RST(pipe_index), 1);
	bam_writel_sync(dmux, BAM_P_RST(pipe_index), 0);

	/* Step 2: Unmask this pipe in EE 0's IRQ sources */
	val = bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE));
	val |= BIT(pipe_index);
	bam_writel(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE), val);

	/* Step 3: Enable pipe IRQ (EOT) */
	bam_writel(dmux, BAM_P_IRQ_EN(pipe_index), P_TRNSFR_END_EN);

	/* Step 4: Set P_DIRECTION in P_CTRL (WITHOUT P_EN!)
	 * After P_RST, P_CTRL is 0 (direction=consumer, sys_mode=BAM2BAM).
	 * Must set direction BEFORE configuring FIFO so hardware knows
	 * how to interpret the descriptor FIFO (producer vs consumer).
	 */
	val = 0;
	if (producer)
		val |= P_DIRECTION;
	bam_writel_sync(dmux, BAM_P_CTRL(pipe_index), val);

	/* Step 5: Set P_SYS_MODE (still WITHOUT P_EN) */
	val |= P_SYS_MODE;
	bam_writel_sync(dmux, BAM_P_CTRL(pipe_index), val);

	/* Step 6: Event threshold */
	bam_writel(dmux, BAM_P_EVNT_GEN_TRSHLD(pipe_index), 0x10);

	/* Step 7: Descriptor FIFO address */
	bam_writel(dmux, BAM_P_DESC_FIFO_ADDR(pipe_index),
		   lower_32_bits(pipe->desc_fifo_phys));

	/* Step 8: Descriptor FIFO size (lower 16 bits of FIFO_SIZES) */
	bam_writel(dmux, BAM_P_FIFO_SIZES(pipe_index), BAM_DESC_FIFO_SIZE);

	/* Step 9-10: P_SYS_STRM=0, P_LOCK_GROUP=0 (already 0 after P_RST) */

	/* Step 11: Enable the pipe — LAST operation */
	val |= P_EN;
	bam_writel_sync(dmux, BAM_P_CTRL(pipe_index), val);

	dev_info(dmux->dev, "pipe%u_init: P_CTRL=0x%08x phys=0x%pad\n",
		 pipe_index, bam_readl(dmux, BAM_P_CTRL(pipe_index)),
		 &pipe->desc_fifo_phys);

	return 0;
}

static void bam_pipe_deinit(struct bam_dmux *dmux, struct bam_pipe *pipe)
{
	if (!pipe->desc_fifo)
		return;

	/*
	 * Do NOT write BAM_P_CTRL here — this function is called from
	 * bam_dmux_power_off() which runs when the modem has crashed or
	 * powered down.  The BAM clock domain may already be gated;
	 * any MMIO write would cause an external abort and kernel panic.
	 * The next SW_RST on reconnect will clear all pipe state anyway.
	 */

	dma_free_coherent(dmux->dev, BAM_DESC_FIFO_SIZE,
			  pipe->desc_fifo, pipe->desc_fifo_phys);
	pipe->desc_fifo = NULL;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Descriptor Ring Operations
 *
 * The descriptor FIFO is a circular ring of bam_desc_hw entries.
 * We write descriptors at desc_offset and advance it.
 * The hardware reads descriptors and updates P_SW_OFSTS.
 * We write P_EVNT_REG to ring the doorbell after queueing descriptors.
 * ────────────────────────────────────────────────────────────────────────── */

static int bam_pipe_submit_desc(struct bam_dmux *dmux, struct bam_pipe *pipe,
				dma_addr_t addr, u16 size, u16 flags,
				void *user_data)
{
	struct bam_desc_hw *desc;
	u32 next_offset;
	u32 desc_idx;

	next_offset = pipe->desc_offset + sizeof(struct bam_desc_hw);
	if (next_offset >= BAM_DESC_FIFO_SIZE)
		next_offset = 0;

	/* Check if FIFO is full (one slot must stay empty) */
	if (next_offset == (bam_readl(dmux, BAM_P_SW_OFSTS(pipe->pipe_index))
			    & P_SW_OFSTS_MASK)) {
		dev_warn(dmux->dev, "pipe%u: descriptor FIFO full\n",
			 pipe->pipe_index);
		return -ENOSPC;
	}

	desc_idx = pipe->desc_offset / sizeof(struct bam_desc_hw);
	desc = &pipe->desc_fifo[desc_idx];
	desc->addr = cpu_to_le32(lower_32_bits(addr));
	desc->size = cpu_to_le16(size);
	desc->flags = cpu_to_le16(flags);

	pipe->user_data[desc_idx] = user_data;
	pipe->desc_offset = next_offset;

	return 0;
}

static void bam_pipe_doorbell(struct bam_dmux *dmux, struct bam_pipe *pipe)
{
	/* Ensure descriptors are visible before doorbell */
	wmb();
	bam_writel_sync(dmux, BAM_P_EVNT_REG(pipe->pipe_index),
			pipe->desc_offset);
}

/*
 * Clear all pending BAM and pipe interrupt status.
 *
 * The downstream SPS ISR (bam_isr → bam_pipe_get_and_clear_irq_status)
 * clears these on every interrupt.  Without clearing, the P_WAKE IRQ
 * status (bit 2) accumulates on producer pipes after doorbell, and may
 * prevent the BAM DMA engine from starting descriptor processing.
 */
static void bam_clear_irqs(struct bam_dmux *dmux)
{
	u32 stts;

	/* BAM-level IRQ */
	stts = bam_readl(dmux, BAM_IRQ_STTS);
	if (stts)
		bam_writel(dmux, BAM_IRQ_CLR, stts);

	/* Pipe 4 (TX) IRQ */
	stts = bam_readl(dmux, BAM_P_IRQ_STTS(BAM_DMUX_TX_PIPE));
	if (stts)
		bam_writel(dmux, BAM_P_IRQ_CLR(BAM_DMUX_TX_PIPE), stts);

	/* Pipe 5 (RX) IRQ */
	stts = bam_readl(dmux, BAM_P_IRQ_STTS(BAM_DMUX_RX_PIPE));
	if (stts)
		bam_writel(dmux, BAM_P_IRQ_CLR(BAM_DMUX_RX_PIPE), stts);
}

/*
 * BAM hardware ISR — matches downstream SPS bam_isr().
 *
 * The downstream SPS layer registers this ISR for BAM IRQ 29.
 * It reads IRQ_SRCS_EE, clears BAM-level and pipe-level interrupt
 * status via IRQ_CLR / P_IRQ_CLR.  Without this, accumulated
 * interrupt status may prevent the BAM DMA engine from processing
 * producer pipe (RX) descriptors.
 */
static irqreturn_t bam_dmux_bam_isr(int irq, void *data)
{
	struct bam_dmux *dmux = data;
	u32 srcs, stts;

	if (!READ_ONCE(dmux->pipes_active))
		return IRQ_HANDLED;

	srcs = bam_readl(dmux, BAM_IRQ_SRCS_EE(BAM_DMUX_EE));

	/* BAM-level IRQ */
	if (srcs & BIT(31)) {
		stts = bam_readl(dmux, BAM_IRQ_STTS);
		bam_writel(dmux, BAM_IRQ_CLR, stts);
	}

	/* Pipe 4 (TX) */
	if (srcs & BIT(BAM_DMUX_TX_PIPE)) {
		stts = bam_readl(dmux, BAM_P_IRQ_STTS(BAM_DMUX_TX_PIPE));
		bam_writel(dmux, BAM_P_IRQ_CLR(BAM_DMUX_TX_PIPE), stts);
	}

	/* Pipe 5 (RX) */
	if (srcs & BIT(BAM_DMUX_RX_PIPE)) {
		stts = bam_readl(dmux, BAM_P_IRQ_STTS(BAM_DMUX_RX_PIPE));
		bam_writel(dmux, BAM_P_IRQ_CLR(BAM_DMUX_RX_PIPE), stts);
	}

	return IRQ_HANDLED;
}

/* ──────────────────────────────────────────────────────────────────────────
 * SKB DMA Helpers
 * ────────────────────────────────────────────────────────────────────────── */

static bool bam_dmux_skb_dma_map(struct bam_dmux_skb_dma *skb_dma,
				 enum dma_data_direction dir)
{
	struct device *dev = skb_dma->dmux->dev;

	skb_dma->addr = dma_map_single(dev, skb_dma->skb->data,
				       skb_dma->len, dir);
	if (dma_mapping_error(dev, skb_dma->addr)) {
		dev_err(dev, "Failed to DMA map buffer\n");
		skb_dma->addr = 0;
		return false;
	}
	return true;
}

static void bam_dmux_skb_dma_unmap(struct bam_dmux_skb_dma *skb_dma,
				   enum dma_data_direction dir)
{
	dma_unmap_single(skb_dma->dmux->dev, skb_dma->addr,
			 skb_dma->len, dir);
	skb_dma->addr = 0;
}

/* ──────────────────────────────────────────────────────────────────────────
 * TX Path
 * ────────────────────────────────────────────────────────────────────────── */

static void bam_dmux_tx_wake_queues(struct bam_dmux *dmux)
{
	int i;

	for (i = 0; i < BAM_DMUX_NUM_CH; ++i) {
		struct net_device *netdev = dmux->netdevs[i];

		if (netdev && netif_running(netdev))
			netif_wake_queue(netdev);
	}
}

static void bam_dmux_tx_stop_queues(struct bam_dmux *dmux)
{
	int i;

	for (i = 0; i < BAM_DMUX_NUM_CH; ++i) {
		struct net_device *netdev = dmux->netdevs[i];

		if (netdev)
			netif_stop_queue(netdev);
	}
}

static void bam_dmux_tx_done(struct bam_dmux_skb_dma *skb_dma)
{
	struct bam_dmux *dmux = skb_dma->dmux;
	unsigned long flags;

	pm_runtime_put_autosuspend(dmux->dev);

	if (skb_dma->addr)
		bam_dmux_skb_dma_unmap(skb_dma, DMA_TO_DEVICE);

	spin_lock_irqsave(&dmux->tx_lock, flags);
	skb_dma->skb = NULL;
	if (skb_dma == &dmux->tx_skbs[dmux->tx_next_skb % BAM_DMUX_NUM_SKB])
		bam_dmux_tx_wake_queues(dmux);
	spin_unlock_irqrestore(&dmux->tx_lock, flags);
}

static struct bam_dmux_skb_dma *
bam_dmux_tx_queue(struct bam_dmux *dmux, struct sk_buff *skb)
{
	struct bam_dmux_skb_dma *skb_dma;
	unsigned long flags;

	spin_lock_irqsave(&dmux->tx_lock, flags);

	skb_dma = &dmux->tx_skbs[dmux->tx_next_skb % BAM_DMUX_NUM_SKB];
	if (skb_dma->skb) {
		bam_dmux_tx_stop_queues(dmux);
		spin_unlock_irqrestore(&dmux->tx_lock, flags);
		return NULL;
	}
	skb_dma->skb = skb;

	dmux->tx_next_skb++;
	if (dmux->tx_skbs[dmux->tx_next_skb % BAM_DMUX_NUM_SKB].skb)
		bam_dmux_tx_stop_queues(dmux);

	spin_unlock_irqrestore(&dmux->tx_lock, flags);
	return skb_dma;
}

static bool bam_dmux_submit_tx(struct bam_dmux *dmux,
			       struct bam_dmux_skb_dma *skb_dma)
{
	int ret;

	ret = bam_pipe_submit_desc(dmux, &dmux->tx_pipe, skb_dma->addr,
				   skb_dma->len, DESC_FLAG_EOT | DESC_FLAG_INT,
				   skb_dma);
	if (ret) {
		dev_err(dmux->dev, "tx submit failed: %d\n", ret);
		return false;
	}
	bam_pipe_doorbell(dmux, &dmux->tx_pipe);
	return true;
}

/* ──────────────────────────────────────────────────────────────────────────
 * RX Path
 * ────────────────────────────────────────────────────────────────────────── */

static void bam_dmux_cmd_data(struct bam_dmux_skb_dma *skb_dma)
{
	struct bam_dmux *dmux = skb_dma->dmux;
	struct sk_buff *skb = skb_dma->skb;
	struct bam_dmux_hdr *hdr = (struct bam_dmux_hdr *)skb->data;
	struct net_device *netdev = dmux->netdevs[hdr->ch];

	if (!netdev || !netif_running(netdev)) {
		dev_warn(dmux->dev, "Data for inactive channel %u\n", hdr->ch);
		return;
	}

	if (hdr->len > BAM_DMUX_MAX_DATA_SIZE) {
		dev_err(dmux->dev, "Data too large: %u > %u\n",
			hdr->len, (u16)BAM_DMUX_MAX_DATA_SIZE);
		return;
	}

	skb_dma->skb = NULL; /* Hand over to network stack */

	skb_pull(skb, sizeof(*hdr));
	skb_trim(skb, hdr->len);
	skb->dev = netdev;

	switch (skb->data[0] & 0xf0) {
	case 0x40:
		skb->protocol = htons(ETH_P_IP);
		break;
	case 0x60:
		skb->protocol = htons(ETH_P_IPV6);
		break;
	default:
		skb->protocol = htons(ETH_P_MAP);
		break;
	}

	netif_receive_skb(skb);
}

static void bam_dmux_cmd_open(struct bam_dmux *dmux, struct bam_dmux_hdr *hdr)
{
	struct net_device *netdev = dmux->netdevs[hdr->ch];

	dev_info(dmux->dev, "CMD_OPEN received for channel %u\n", hdr->ch);
	dmux->cmd_open_acked = true;

	if (__test_and_set_bit(hdr->ch, dmux->remote_channels))
		return;

	if (netdev)
		netif_device_attach(netdev);
	else
		schedule_work(&dmux->register_netdev_work);
}

static void bam_dmux_cmd_close(struct bam_dmux *dmux, struct bam_dmux_hdr *hdr)
{
	struct net_device *netdev = dmux->netdevs[hdr->ch];

	dev_dbg(dmux->dev, "close channel: %u\n", hdr->ch);

	if (!__test_and_clear_bit(hdr->ch, dmux->remote_channels))
		return;

	if (netdev)
		netif_device_detach(netdev);
}

static void bam_dmux_rx_handle(struct bam_dmux_skb_dma *skb_dma)
{
	struct bam_dmux *dmux = skb_dma->dmux;
	struct sk_buff *skb = skb_dma->skb;
	struct bam_dmux_hdr *hdr = (struct bam_dmux_hdr *)skb->data;

	bam_dmux_skb_dma_unmap(skb_dma, DMA_FROM_DEVICE);

	dev_info(dmux->dev,
		 "rx: magic=%#06x cmd=%u ch=%u len=%u pad=%u\n",
		 hdr->magic, hdr->cmd, hdr->ch, hdr->len, hdr->pad);

	if (hdr->magic != BAM_DMUX_HDR_MAGIC) {
		dev_err(dmux->dev, "Invalid magic: %#x\n", hdr->magic);
		return;
	}

	if (hdr->ch >= BAM_DMUX_NUM_CH) {
		dev_dbg(dmux->dev, "Unsupported channel: %u\n", hdr->ch);
		return;
	}

	switch (hdr->cmd) {
	case BAM_DMUX_CMD_DATA:
		bam_dmux_cmd_data(skb_dma);
		break;
	case BAM_DMUX_CMD_OPEN:
		bam_dmux_cmd_open(dmux, hdr);
		break;
	case BAM_DMUX_CMD_CLOSE:
		bam_dmux_cmd_close(dmux, hdr);
		break;
	default:
		dev_err(dmux->dev, "Unsupported cmd %u on ch %u\n",
			hdr->cmd, hdr->ch);
		break;
	}
}

/* ──────────────────────────────────────────────────────────────────────────
 * Send CMD_OPEN directly on TX pipe (no netdev needed)
 *
 * The Quectel firmware does not spontaneously send CMD_OPEN during boot.
 * In the stock firmware, the modem sends CMD_OPEN first, then APPS responds.
 * But with Quectel OCPU firmware, we must send CMD_OPEN first to trigger
 * the modem's a2_sio_channel_open, which echoes CMD_OPEN back.
 * ────────────────────────────────────────────────────────────────────────── */

static bool bam_dmux_send_open_cmd(struct bam_dmux *dmux, u8 ch_id)
{
	struct bam_dmux_skb_dma *skb_dma;
	struct bam_dmux_hdr *hdr;
	struct sk_buff *skb;
	int ret;

	/* Safety: don't overwrite a pending CMD_OPEN */
	skb_dma = &dmux->tx_skbs[0];
	if (skb_dma->skb)
		return false;

	skb = alloc_skb(sizeof(*hdr), GFP_ATOMIC);
	if (!skb)
		return false;

	hdr = skb_put_zero(skb, sizeof(*hdr));
	hdr->magic = BAM_DMUX_HDR_MAGIC;
	hdr->cmd = BAM_DMUX_CMD_OPEN;
	hdr->ch = ch_id;

	/* Use a dedicated skb_dma slot for this init command */
	skb_dma->skb = skb;
	skb_dma->dmux = dmux;
	skb_dma->len = skb->len;

	if (!bam_dmux_skb_dma_map(skb_dma, DMA_TO_DEVICE)) {
		dev_kfree_skb(skb);
		skb_dma->skb = NULL;
		return false;
	}

	ret = bam_pipe_submit_desc(dmux, &dmux->tx_pipe, skb_dma->addr,
				   skb_dma->len, DESC_FLAG_EOT | DESC_FLAG_INT,
				   skb_dma);
	if (ret) {
		bam_dmux_skb_dma_unmap(skb_dma, DMA_TO_DEVICE);
		dev_kfree_skb(skb);
		skb_dma->skb = NULL;
		return false;
	}

	bam_pipe_doorbell(dmux, &dmux->tx_pipe);

	dev_info(dmux->dev, "sent CMD_OPEN for channel %u on pipe %u\n",
		 ch_id, BAM_DMUX_TX_PIPE);
	return true;
}

static bool bam_dmux_queue_rx(struct bam_dmux *dmux,
			      struct bam_dmux_skb_dma *skb_dma, gfp_t gfp)
{
	int ret;

	if (!skb_dma->skb) {
		skb_dma->skb = __netdev_alloc_skb(NULL, BAM_DMUX_BUFFER_SIZE,
						  gfp);
		if (!skb_dma->skb)
			return false;
		skb_put(skb_dma->skb, BAM_DMUX_BUFFER_SIZE);
	}

	skb_dma->len = BAM_DMUX_BUFFER_SIZE;

	if (!bam_dmux_skb_dma_map(skb_dma, DMA_FROM_DEVICE))
		return false;

	ret = bam_pipe_submit_desc(dmux, &dmux->rx_pipe, skb_dma->addr,
				   skb_dma->len, 0, skb_dma);
	if (ret) {
		bam_dmux_skb_dma_unmap(skb_dma, DMA_FROM_DEVICE);
		return false;
	}
	return true;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Completion Polling
 *
 * On MDM9607, TrustZone blocks the BAM interrupt from reaching Linux.
 * We poll P_SW_OFSTS to detect completed descriptors.
 * ────────────────────────────────────────────────────────────────────────── */

static void bam_dmux_process_tx_completions(struct bam_dmux *dmux)
{
	struct bam_pipe *pipe = &dmux->tx_pipe;
	u32 hw_offset;

	hw_offset = bam_readl(dmux, BAM_P_SW_OFSTS(pipe->pipe_index))
		    & P_SW_OFSTS_MASK;

	while (pipe->sw_offset != hw_offset) {
		u32 idx = pipe->sw_offset / sizeof(struct bam_desc_hw);
		struct bam_dmux_skb_dma *skb_dma = pipe->user_data[idx];

		if (skb_dma && skb_dma->skb) {
			struct sk_buff *skb = skb_dma->skb;

			bam_dmux_tx_done(skb_dma);
			dev_consume_skb_any(skb);
		}

		pipe->sw_offset += sizeof(struct bam_desc_hw);
		if (pipe->sw_offset >= BAM_DESC_FIFO_SIZE)
			pipe->sw_offset = 0;
	}
}

static void bam_dmux_process_rx_completions(struct bam_dmux *dmux)
{
	struct bam_pipe *pipe = &dmux->rx_pipe;
	u32 hw_offset;
	bool requeue = false;

	hw_offset = bam_readl(dmux, BAM_P_SW_OFSTS(pipe->pipe_index))
		    & P_SW_OFSTS_MASK;

	while (pipe->sw_offset != hw_offset) {
		u32 idx = pipe->sw_offset / sizeof(struct bam_desc_hw);
		struct bam_dmux_skb_dma *skb_dma = pipe->user_data[idx];

		if (skb_dma && skb_dma->skb) {
			bam_dmux_rx_handle(skb_dma);

			/* Re-queue the RX buffer */
			if (bam_dmux_queue_rx(dmux, skb_dma, GFP_ATOMIC))
				requeue = true;
		}

		pipe->sw_offset += sizeof(struct bam_desc_hw);
		if (pipe->sw_offset >= BAM_DESC_FIFO_SIZE)
			pipe->sw_offset = 0;
	}

	if (requeue)
		bam_pipe_doorbell(dmux, &dmux->rx_pipe);
}

static void bam_dmux_dump_pipes(struct bam_dmux *dmux, const char *label)
{
	int p;

	dev_info(dmux->dev,
		 "%s: BAM_CTRL=0x%08x CNFG=0x%08x IRQ_EN=0x%08x "
		 "PIPE_ATTR_EE0=0x%08x\n",
		 label,
		 bam_readl(dmux, BAM_CTRL),
		 bam_readl(dmux, BAM_CNFG_BITS),
		 bam_readl(dmux, BAM_IRQ_EN),
		 bam_readl(dmux, BAM_PIPE_ATTR_EE(BAM_DMUX_EE)));

	for (p = 0; p < 4; p++)
		dev_info(dmux->dev,
			 "%s: EE%d IRQ_SRCS=0x%08x MSK=0x%08x\n",
			 label, p,
			 bam_readl(dmux, BAM_IRQ_SRCS_EE(p)),
			 bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE(p)));

	for (p = 4; p <= 5; p++)
		dev_info(dmux->dev,
			 "%s: P%d CTRL=0x%08x HALT=0x%08x IRQ_STTS=0x%08x "
			 "IRQ_EN=0x%08x SW=0x%08x EVNT=0x%08x "
			 "DESC=0x%08x FIFO_SZ=0x%08x TRSHLD=0x%08x "
			 "DATA_FIFO=0x%08x EVNT_DEST=0x%08x\n",
			 label, p,
			 bam_readl(dmux, BAM_P_CTRL(p)),
			 bam_readl(dmux, BAM_P_HALT(p)),
			 bam_readl(dmux, BAM_P_IRQ_STTS(p)),
			 bam_readl(dmux, BAM_P_IRQ_EN(p)),
			 bam_readl(dmux, BAM_P_SW_OFSTS(p)),
			 bam_readl(dmux, BAM_P_EVNT_REG(p)),
			 bam_readl(dmux, BAM_P_DESC_FIFO_ADDR(p)),
			 bam_readl(dmux, BAM_P_FIFO_SIZES(p)),
			 bam_readl(dmux, BAM_P_EVNT_GEN_TRSHLD(p)),
			 bam_readl(dmux, BAM_P_DATA_FIFO_ADDR(p)),
			 bam_readl(dmux, BAM_P_EVNT_DEST_ADDR(p)));
}

static void bam_dmux_poll_timer_fn(struct timer_list *t)
{
	struct bam_dmux *dmux = container_of(t, struct bam_dmux, rx_poll_timer);
	static unsigned int poll_count;

	/*
	 * Check flags with READ_ONCE — bam_dmux_power_off() sets these
	 * to false BEFORE canceling the timer.  This is our primary
	 * guard against accessing BAM registers after modem crash.
	 *
	 * If pc_state is not yet set (modem hasn't responded), still
	 * re-arm the timer so we don't lose it.  Only skip the BAM
	 * register accesses.
	 */
	if (!READ_ONCE(dmux->pipes_active))
		return;

	if (!READ_ONCE(dmux->pc_state))
		goto rearm;

	/*
	 * Clear accumulated IRQ status at every poll iteration.
	 * The downstream SPS ISR does this on every BAM interrupt.
	 * Without clearing, the BAM DMA engine may stall on producer
	 * pipes (RX) due to unacknowledged P_WAKE or EOT status.
	 */
	bam_clear_irqs(dmux);

	bam_dmux_process_tx_completions(dmux);
	bam_dmux_process_rx_completions(dmux);

	if (poll_count < 20 || !(poll_count % 1000)) {
		u32 tx_hw = bam_readl(dmux, BAM_P_SW_OFSTS(BAM_DMUX_TX_PIPE));
		u32 rx_hw = bam_readl(dmux, BAM_P_SW_OFSTS(BAM_DMUX_RX_PIPE));

		dev_info(dmux->dev,
			 "poll[%u]: tx_sw=0x%04x rx_sw=0x%04x "
			 "tx_hw=0x%08x rx_hw=0x%08x\n", poll_count,
			 dmux->tx_pipe.sw_offset, dmux->rx_pipe.sw_offset,
			 tx_hw, rx_hw);
	}

	/* One-shot detailed error/status dump at poll[20] */
	if (poll_count == 20) {
		dev_info(dmux->dev,
			 "DIAG: BAM_CTRL=0x%08x CNFG=0x%08x "
			 "IRQ_SRCS_EE=0x%08x IRQ_SRCS_MSK=0x%08x\n",
			 bam_readl(dmux, BAM_CTRL),
			 bam_readl(dmux, BAM_CNFG_BITS),
			 bam_readl(dmux, BAM_IRQ_SRCS_EE(BAM_DMUX_EE)),
			 bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE)));
		/* Dump ALL 6 pipes to see if modem configured any */
		for (int p = 0; p < 6; p++)
			dev_info(dmux->dev,
				 "DIAG: P%d CTRL=0x%08x HALT=0x%08x "
				 "SW=0x%08x EVNT=0x%08x\n", p,
				 bam_readl(dmux, BAM_P_CTRL(p)),
				 bam_readl(dmux, BAM_P_HALT(p)),
				 bam_readl(dmux, BAM_P_SW_OFSTS(p)),
				 bam_readl(dmux, BAM_P_EVNT_REG(p)));
	}

	/*
	 * CMD_OPEN fallback — downstream protocol: modem sends CMD_OPEN
	 * first.  Wait 15s for modem's CMD_OPEN.  If nothing arrives,
	 * send CMD_OPEN from APPS as fallback.
	 * Retry every 5 seconds up to 12 times.
	 */
	if (!dmux->cmd_open_acked &&
	    dmux->rx_pipe.sw_offset == 0 &&
	    dmux->cmd_open_retries < 12 &&
	    time_after(jiffies, dmux->cmd_open_next_retry)) {
		pm_runtime_get_noresume(dmux->dev);
		if (bam_dmux_send_open_cmd(dmux, 0)) {
			dmux->cmd_open_retries++;
			dmux->cmd_open_next_retry = jiffies + 5 * HZ;
			dev_info(dmux->dev,
				 "CMD_OPEN fallback %u sent\n",
				 dmux->cmd_open_retries);
			/* Full BAM dump at each CMD_OPEN retry */
			bam_dmux_dump_pipes(dmux, "RETRY-DIAG");
		} else {
			pm_runtime_put_noidle(dmux->dev);
			if (dmux->cmd_open_retries == 0)
				dev_warn(dmux->dev,
					 "CMD_OPEN fallback send failed\n");
		}
	}

	poll_count++;

	/* Re-check flags before rearming — modem may have gone down */
rearm:
	if (READ_ONCE(dmux->pipes_active))
		mod_timer(&dmux->rx_poll_timer,
			  jiffies + msecs_to_jiffies(20));
}

/* ──────────────────────────────────────────────────────────────────────────
 * Network Device Operations
 * ────────────────────────────────────────────────────────────────────────── */

static int bam_dmux_send_cmd(struct bam_dmux_netdev *bndev, u8 cmd)
{
	struct bam_dmux *dmux = bndev->dmux;
	struct bam_dmux_skb_dma *skb_dma;
	struct bam_dmux_hdr *hdr;
	struct sk_buff *skb;
	int ret;

	skb = alloc_skb(sizeof(*hdr), GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hdr = skb_put_zero(skb, sizeof(*hdr));
	hdr->magic = BAM_DMUX_HDR_MAGIC;
	hdr->cmd = cmd;
	hdr->ch = bndev->ch;

	skb_dma = bam_dmux_tx_queue(dmux, skb);
	if (!skb_dma) {
		ret = -EAGAIN;
		goto free_skb;
	}

	ret = pm_runtime_get_sync(dmux->dev);
	if (ret < 0)
		goto tx_fail;

	skb_dma->len = skb->len;
	if (!bam_dmux_skb_dma_map(skb_dma, DMA_TO_DEVICE)) {
		ret = -ENOMEM;
		goto tx_fail;
	}

	if (!bam_dmux_submit_tx(dmux, skb_dma)) {
		ret = -EIO;
		goto tx_fail;
	}
	return 0;

tx_fail:
	bam_dmux_tx_done(skb_dma);
free_skb:
	dev_kfree_skb(skb);
	return ret;
}

static int bam_dmux_netdev_open(struct net_device *netdev)
{
	struct bam_dmux_netdev *bndev = netdev_priv(netdev);
	int ret;

	ret = bam_dmux_send_cmd(bndev, BAM_DMUX_CMD_OPEN);
	if (ret)
		return ret;

	netif_start_queue(netdev);
	return 0;
}

static int bam_dmux_netdev_stop(struct net_device *netdev)
{
	struct bam_dmux_netdev *bndev = netdev_priv(netdev);

	netif_stop_queue(netdev);
	bam_dmux_send_cmd(bndev, BAM_DMUX_CMD_CLOSE);
	return 0;
}

static unsigned int needed_room(unsigned int avail, unsigned int needed)
{
	if (avail >= needed)
		return 0;
	return needed - avail;
}

static int bam_dmux_tx_prepare_skb(struct bam_dmux_netdev *bndev,
				   struct sk_buff *skb)
{
	unsigned int head = needed_room(skb_headroom(skb), BAM_DMUX_HDR_SIZE);
	unsigned int pad = sizeof(u32) - skb->len % sizeof(u32);
	unsigned int tail = needed_room(skb_tailroom(skb), pad);
	struct bam_dmux_hdr *hdr;
	int ret;

	if (head || tail || skb_cloned(skb)) {
		ret = pskb_expand_head(skb, head, tail, GFP_ATOMIC);
		if (ret)
			return ret;
	}

	hdr = skb_push(skb, sizeof(*hdr));
	hdr->magic = BAM_DMUX_HDR_MAGIC;
	hdr->signal = 0;
	hdr->cmd = BAM_DMUX_CMD_DATA;
	hdr->pad = pad;
	hdr->ch = bndev->ch;
	hdr->len = skb->len - sizeof(*hdr);
	if (pad)
		skb_put_zero(skb, pad);

	return 0;
}

static netdev_tx_t bam_dmux_netdev_start_xmit(struct sk_buff *skb,
					      struct net_device *netdev)
{
	struct bam_dmux_netdev *bndev = netdev_priv(netdev);
	struct bam_dmux *dmux = bndev->dmux;
	struct bam_dmux_skb_dma *skb_dma;
	int active, ret;

	skb_dma = bam_dmux_tx_queue(dmux, skb);
	if (!skb_dma)
		return NETDEV_TX_BUSY;

	active = pm_runtime_get(dmux->dev);
	if (active < 0 && active != -EINPROGRESS)
		goto drop;

	ret = bam_dmux_tx_prepare_skb(bndev, skb);
	if (ret)
		goto drop;

	skb_dma->len = skb->len;
	if (!bam_dmux_skb_dma_map(skb_dma, DMA_TO_DEVICE))
		goto drop;

	if (active <= 0) {
		if (!atomic_long_fetch_or(BIT(skb_dma - dmux->tx_skbs),
					  &dmux->tx_deferred_skb))
			queue_pm_work(&dmux->tx_wakeup_work);
		return NETDEV_TX_OK;
	}

	if (!bam_dmux_submit_tx(dmux, skb_dma))
		goto drop;

	return NETDEV_TX_OK;

drop:
	bam_dmux_tx_done(skb_dma);
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static void bam_dmux_tx_wakeup_work(struct work_struct *work)
{
	struct bam_dmux *dmux = container_of(work, struct bam_dmux,
					     tx_wakeup_work);
	unsigned long pending;
	int ret, i;

	ret = pm_runtime_resume_and_get(dmux->dev);
	if (ret < 0)
		return;

	pending = atomic_long_xchg(&dmux->tx_deferred_skb, 0);
	if (!pending)
		goto out;

	for_each_set_bit(i, &pending, BAM_DMUX_NUM_SKB)
		bam_dmux_submit_tx(dmux, &dmux->tx_skbs[i]);

out:
	pm_runtime_put_autosuspend(dmux->dev);
}

static const struct net_device_ops bam_dmux_ops = {
	.ndo_open	= bam_dmux_netdev_open,
	.ndo_stop	= bam_dmux_netdev_stop,
	.ndo_start_xmit	= bam_dmux_netdev_start_xmit,
};

static const struct device_type wwan_type = {
	.name = "wwan",
};

static void bam_dmux_netdev_setup(struct net_device *dev)
{
	dev->netdev_ops = &bam_dmux_ops;

	dev->type = ARPHRD_RAWIP;
	SET_NETDEV_DEVTYPE(dev, &wwan_type);
	dev->flags = IFF_POINTOPOINT | IFF_NOARP;

	dev->mtu = ETH_DATA_LEN;
	dev->max_mtu = BAM_DMUX_MAX_DATA_SIZE;
	dev->needed_headroom = sizeof(struct bam_dmux_hdr);
	dev->needed_tailroom = sizeof(u32);
	dev->tx_queue_len = DEFAULT_TX_QUEUE_LEN;

	dev->addr_assign_type = NET_ADDR_RANDOM;
	eth_random_addr(dev->perm_addr);
}

static void bam_dmux_register_netdev_work(struct work_struct *work)
{
	struct bam_dmux *dmux = container_of(work, struct bam_dmux,
					     register_netdev_work);
	struct bam_dmux_netdev *bndev;
	struct net_device *netdev;
	int ch, ret;

	for_each_set_bit(ch, dmux->remote_channels, BAM_DMUX_NUM_CH) {
		if (dmux->netdevs[ch])
			continue;

		netdev = alloc_netdev(sizeof(*bndev), "wwan%d", NET_NAME_ENUM,
				      bam_dmux_netdev_setup);
		if (!netdev)
			return;

		SET_NETDEV_DEV(netdev, dmux->dev);
		netdev->dev_port = ch;

		bndev = netdev_priv(netdev);
		bndev->dmux = dmux;
		bndev->ch = ch;

		ret = register_netdev(netdev);
		if (ret) {
			dev_err(dmux->dev,
				"Failed to register netdev ch %u: %d\n",
				ch, ret);
			free_netdev(netdev);
			return;
		}

		dmux->netdevs[ch] = netdev;
	}
}

/* ──────────────────────────────────────────────────────────────────────────
 * Power Off
 * ────────────────────────────────────────────────────────────────────────── */

static void bam_dmux_power_off(struct bam_dmux *dmux)
{
	int i;

	/* Set flags FIRST so poll timer won't access BAM registers */
	WRITE_ONCE(dmux->pipes_active, false);
	WRITE_ONCE(dmux->pc_state, false);
	dmux->boot_done = false;

	/*
	 * Use timer_delete (not timer_delete_sync) — this may be
	 * called from IRQ context (pc_irq) where sync would deadlock
	 * on single-CPU systems if the timer callback is running.
	 */
	timer_delete(&dmux->rx_poll_timer);

	bam_pipe_deinit(dmux, &dmux->tx_pipe);
	bam_pipe_deinit(dmux, &dmux->rx_pipe);

	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		struct bam_dmux_skb_dma *skb_dma = &dmux->rx_skbs[i];

		if (skb_dma->addr)
			bam_dmux_skb_dma_unmap(skb_dma, DMA_FROM_DEVICE);
		if (skb_dma->skb) {
			dev_kfree_skb(skb_dma->skb);
			skb_dma->skb = NULL;
		}
	}

	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		struct bam_dmux_skb_dma *skb_dma = &dmux->tx_skbs[i];

		if (skb_dma->addr)
			bam_dmux_skb_dma_unmap(skb_dma, DMA_TO_DEVICE);
		if (skb_dma->skb) {
			dev_kfree_skb(skb_dma->skb);
			skb_dma->skb = NULL;
		}
	}
}

/* ──────────────────────────────────────────────────────────────────────────
 * SMSM IRQ Handlers + Boot Sequence (Attempt 38)
 *
 * Root cause analysis from attempts 1-37:
 *   Attempts 1-35: pipe 5 RX stuck at 0 (register ordering + timing bugs).
 *   Attempt 36: Register ordering and timing FIXED (P5 CTRL=0x2a works!)
 *     BUT modem crashed due to double first_connect race condition.
 *   Attempt 37: Double-entry fixed (pipes_active guard works!).
 *     No crash.  TX works (CMD_OPENs consumed by modem).
 *     BUT pipe 5 RX still stuck at 0 — modem never sends data back.
 *
 *   The timing bug (attempt 37 analysis):
 *     We set APPS bit 1 at [33.156] — BEFORE pipe 5 init.
 *     Modem receives it at [33.226] (70ms IPC delivery, NOT 1-5ms!).
 *     Modem's a2_apps_smsm_callback fires → apps_bam_link_ready=true.
 *     Modem may immediately send CMD_OPEN response on pipe 5.
 *     But pipe 5 isn't configured until [33.292] (136ms after bit 1).
 *     Data hitting disabled pipe 5 (P_EN=0) is LOST permanently.
 *     Modem's channel_open() only responds to FIRST CMD_OPEN per channel
 *     (subsequent ones just increment open_count → no response).
 *     Our CMD_OPEN retries therefore never get a response.
 *
 * Fix (attempt 38):
 *   - Do ALL init (SW_RST + pipe config + queue RX) BEFORE setting bit 1.
 *   - Set APPS bit 1 as the LAST operation (step 11 of 13).
 *   - When modem receives IPC 70ms later, pipe 5 has been ready for 70ms+
 *     with 32 empty buffers queued.  Modem's CMD_OPEN response goes
 *     directly into our empty-buffer descriptors.
 *   - Keep pipes_active re-entry guard from attempt 37.
 *   - Keep register ordering fix from attempt 36.
 * ────────────────────────────────────────────────────────────────────────── */

static void bam_dmux_first_connect(struct bam_dmux *dmux)
{
	int i, ret;
	u32 val;

	/*
	 * Re-entry guard: On single-core MDM9607, the threaded pc_irq
	 * (SCHED_FIFO prio 50) can preempt this workqueue function after
	 * we set APPS bit 1 and modem responds (~70ms).  Without this
	 * guard, pc_irq sees pipes_active=false and calls first_connect
	 * again, causing double SW_RST + double bit 11 toggle which
	 * crashes the modem (a2_power.c assertion).
	 */
	if (dmux->pipes_active) {
		dev_info(dmux->dev,
			 "first_connect: already active/connecting, skip\n");
		return;
	}
	dmux->pipes_active = true;

	dev_info(dmux->dev,
		 "first_connect: attempt 42 - SATELLITE mode (no BAM global init)\n");

	/*
	 * Attempt 42: SATELLITE / SPS_BAM_MGR_DEVICE_REMOTE behaviour.
	 *
	 * Downstream's bam_dmux sets a2_props.manage = SPS_BAM_MGR_DEVICE_REMOTE,
	 * meaning the BAM hardware is owned/managed by the MODEM, not APPS.
	 * sps_bam_enable() then calls bam_check() (a read-only verification),
	 * NOT bam_init(): no SW_RST, no BAM_CTRL write, no CNFG_BITS write,
	 * no BAM_IRQ_EN write, no DESC_CNT_TRSHLD write.
	 *
	 * Our previous attempts did a full bam_init() which wiped the modem's
	 * BAM setup, breaking the modem-side A2 pipe-5 producer path.  That is
	 * almost certainly why every prior attempt saw the modem accept our
	 * CMD_OPEN (P4 SW advances) but never reply on P5.
	 *
	 * In satellite mode APPS only:
	 *   - configures its own pipes (P4 consumer, P5 producer)
	 *   - sets IRQ_SRCS_MSK_EE(0) bits for its pipes so pipe IRQs route
	 *     to EE0 (RMW, do not touch other EEs)
	 *   - registers a Linux IRQ handler for the BAM hw IRQ
	 *
	 * Do NOT touch BAM_CTRL, BAM_CNFG_BITS, BAM_IRQ_EN or
	 * BAM_DESC_CNT_TRSHLD.  Those are global state owned by the modem.
	 */
	dev_info(dmux->dev,
		 "bam_hw: SAT PRE BAM_CTRL=0x%08x CNFG=0x%08x SRCS_MSK=0x%08x\n",
		 bam_readl(dmux, BAM_CTRL),
		 bam_readl(dmux, BAM_CNFG_BITS),
		 bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE)));

	/*
	 * Step 1: Ensure pipe-level IRQ routing to EE0 (RMW).
	 * BIT(31)=BAM-global IRQ visible to EE0; BIT(4)=P4; BIT(5)=P5.
	 */
	val = bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE));
	val |= BAM_IRQ_MSK | BIT(BAM_DMUX_TX_PIPE) | BIT(BAM_DMUX_RX_PIPE);
	bam_writel(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE), val);

	/* Step 2: Register BAM IRQ */
	if (dmux->bam_irq > 0 && !dmux->bam_irq_registered) {
		ret = devm_request_irq(dmux->dev, dmux->bam_irq,
				       bam_dmux_bam_isr,
				       IRQF_TRIGGER_HIGH | IRQF_SHARED,
				       "bam-dmux-bam", dmux);
		if (!ret) {
			dmux->bam_irq_registered = true;
			dev_info(dmux->dev,
				 "BAM IRQ %d registered\n", dmux->bam_irq);
		}
	}

	/* Step 7: Init TX pipe (pipe 4, consumer) */
	ret = bam_pipe_hw_init(dmux, &dmux->tx_pipe,
			       BAM_DMUX_TX_PIPE, false);
	if (ret) {
		dev_err(dmux->dev, "TX pipe init failed: %d\n", ret);
		dmux->pipes_active = false;
		return;
	}

	/* Step 8: Init RX pipe (pipe 5, producer) */
	dev_info(dmux->dev,
		 "P5 PRE: CTRL=0x%08x DESC=0x%08x FIFO_SZ=0x%08x "
		 "SW=0x%08x EVNT=0x%08x HALT=0x%08x\n",
		 bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_DESC_FIFO_ADDR(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_FIFO_SIZES(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_SW_OFSTS(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_EVNT_REG(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_HALT(BAM_DMUX_RX_PIPE)));
	ret = bam_pipe_hw_init(dmux, &dmux->rx_pipe,
			       BAM_DMUX_RX_PIPE, true);
	if (ret) {
		dev_err(dmux->dev, "RX pipe init failed: %d\n", ret);
		bam_pipe_deinit(dmux, &dmux->tx_pipe);
		dmux->pipes_active = false;
		return;
	}

	dev_info(dmux->dev,
		 "P5 POST: CTRL=0x%08x HALT=0x%08x P4_CTRL=0x%08x\n",
		 bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_HALT(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_TX_PIPE)));

	dmux->boot_done = true;

	/* PM setup */
	pm_runtime_disable(dmux->dev);
	pm_runtime_set_active(dmux->dev);
	pm_runtime_enable(dmux->dev);
	pm_runtime_get_noresume(dmux->dev);
	pm_runtime_get_noresume(dmux->dev);

	/* Step 9: Queue 32 RX descriptors + doorbell.
	 *
	 * Submit empty buffers BEFORE notifying modem.  This ensures that
	 * when modem's apps_bam_link_ready becomes true, pipe 5 already
	 * has 32 empty descriptors waiting for peripheral data.
	 */
	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		if (!bam_dmux_queue_rx(dmux, &dmux->rx_skbs[i], GFP_KERNEL))
			dev_err(dmux->dev, "RX queue %d failed\n", i);
	}
	bam_pipe_doorbell(dmux, &dmux->rx_pipe);

	/* Step 10: Clear pending IRQ status */
	bam_clear_irqs(dmux);

	dev_info(dmux->dev,
		 "first_connect: pipes ready, P4_CTRL=0x%08x P5_CTRL=0x%08x "
		 "IRQ_SRCS_MSK=0x%08x\n",
		 bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_TX_PIPE)),
		 bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE)));

	/*
	 * Step 11: Toggle APPS bit 11 (ACK).
	 *
	 * Downstream toggles ACK after pipe connect + queue_rx and leaves
	 * APPS bit 1 for later UL wake requests.
	 */
	dev_info(dmux->dev, "first_connect: toggling APPS bit 11 (ACK)\n");
	bam_dmux_pc_ack(dmux);

	/* Step 12: Schedule CMD_OPEN after 1s delay */
	dmux->cmd_open_retries = 0;
	dmux->cmd_open_acked = false;
	dmux->cmd_open_next_retry = jiffies + HZ;

	mod_timer(&dmux->rx_poll_timer, jiffies + msecs_to_jiffies(20));
}

static void bam_dmux_ack_work_fn(struct work_struct *work)
{
	struct bam_dmux *dmux = container_of(work, struct bam_dmux,
					     ack_work.work);

	if (!dmux->pipes_active) {
		dev_warn(dmux->dev, "ack_work: pipes not active, skip\n");
		return;
	}

	/*
	 * Attempt 27: ack_work is no longer used for initial connect
	 * (bits 1+11 are set simultaneously in first_connect).
	 * This function is kept for potential reconnect flows.
	 */
	dev_info(dmux->dev, "ack_work: toggling APPS bit 11 (ACK)\n");
	bam_dmux_pc_ack(dmux);
}

static irqreturn_t bam_dmux_pc_irq(int irq, void *data)
{
	struct bam_dmux *dmux = data;
	bool new_state;

	irq_get_irqchip_state(dmux->pc_irq, IRQCHIP_STATE_LINE_LEVEL,
			      &new_state);

	dev_info(dmux->dev, "pc_irq: modem_bit1=%d pc_state=%d pipes=%d\n",
		 new_state, dmux->pc_state, dmux->pipes_active);

	if (new_state && !dmux->pipes_active) {
		/*
		 * Modem set A2_POWER_CONTROL (bit 1) — ready for init.
		 *
		 * The modem finished its a2_subsystem_boot() and is now
		 * waiting for APPS to init BAM + toggle bit 11 (ACK).
		 * Cancel the diagnostic boot_work timer and proceed.
		 */
		cancel_delayed_work(&dmux->boot_work);
		dmux->pc_state = true;
		bam_dmux_first_connect(dmux);

	} else if (new_state && dmux->pc_state) {
		/* Already connected or connecting — re-issue doorbell
		 * only if pipes are fully configured (boot_done).
		 * During first_connect, pipes_active is true but boot_done
		 * is false — rx_pipe isn't configured yet.
		 */
		if (dmux->boot_done) {
			bam_pipe_doorbell(dmux, &dmux->rx_pipe);
			dev_info(dmux->dev,
				 "pc_irq: already up, re-issued doorbell\n");
		} else {
			dev_info(dmux->dev,
				 "pc_irq: init in progress, skip\n");
		}

	} else if (!new_state) {
		dev_info(dmux->dev, "pc_irq: modem powering down\n");
		cancel_delayed_work(&dmux->ack_work);
		bam_dmux_power_off(dmux);
		bam_dmux_pc_ack(dmux);
		pm_runtime_mark_last_busy(dmux->dev);
		pm_runtime_put_autosuspend(dmux->dev);
	}

	dmux->pc_state = new_state;
	wake_up_all(&dmux->pc_wait);

	return IRQ_HANDLED;
}

static irqreturn_t bam_dmux_pc_ack_irq(int irq, void *data)
{
	struct bam_dmux *dmux = data;

	dev_dbg(dmux->dev, "pc ack\n");
	complete_all(&dmux->pc_ack_completion);
	return IRQ_HANDLED;
}

static int bam_dmux_runtime_suspend(struct device *dev)
{
	struct bam_dmux *dmux = dev_get_drvdata(dev);

	if (!dmux->boot_done || !dmux->pc_state)
		return 0;

	bam_dmux_pc_vote(dmux, false);
	return 0;
}

static int __maybe_unused bam_dmux_runtime_resume(struct device *dev)
{
	struct bam_dmux *dmux = dev_get_drvdata(dev);

	if (!dmux->boot_done)
		return 0;

	bam_dmux_pc_vote(dmux, true);

	if (!wait_for_completion_timeout(&dmux->pc_ack_completion,
					 BAM_DMUX_REMOTE_TIMEOUT)) {
		bam_dmux_pc_vote(dmux, false);
		return -ETIMEDOUT;
	}

	if (!wait_event_timeout(dmux->pc_wait, dmux->pc_state,
				BAM_DMUX_REMOTE_TIMEOUT)) {
		bam_dmux_pc_vote(dmux, false);
		return -ETIMEDOUT;
	}

	return 0;
}

static void bam_dmux_boot_work_fn(struct work_struct *work)
{
	struct bam_dmux *dmux = container_of(work, struct bam_dmux,
					     boot_work.work);

	/*
	 * Attempt 41: actively request A2 wake by setting APPS bit 1.
	 *
	 * Per modem firmware (a2_power.c::a2_apps_smsm_callback), when APPS
	 * sets SMSM_A2_POWER_CONTROL (bit 1), the modem will:
	 *   1. call a2_power_vote(A2_CLIENT_APPS=11, true) - powers on A2 HW
	 *   2. set MODEM SMSM bit 1 (A2_POWER_CONTROL) - 'A2 is ready'
	 *   3. toggle MODEM SMSM bit 11 (A2_POWER_CONTROL_ACK)
	 *   4. set apps_bam_link_ready = true
	 *
	 * On downstream (3.18 kernel), the modem set its bit 1 autonomously
	 * about 1.5s after modem boot - presumably because LTE/data activity
	 * triggered an internal A2 client (clients 2-10 per firmware string
	 * 'A2 turned ON by client=%d[(0,1)=INT,(2,4,7,9)=L/W/TD/DO UL,...]').
	 * On mainline, no internal client triggers, so modem never sets bit 1
	 * without an explicit APPS request.
	 *
	 * We schedule this work ~500ms after remote_ready_irq to give the
	 * modem time to register its a2_apps_smsm_callback during its own
	 * boot sequence.  Setting bit 1 too early would be silently ignored.
	 */

	if (dmux->pc_state || dmux->pipes_active) {
		dev_info(dmux->dev, "boot_work: already connected, nop\n");
		return;
	}

	dev_info(dmux->dev,
		 "boot_work: asserting APPS bit 1 to request A2 wake from modem\n");
	bam_dmux_pc_vote(dmux, true);
}

static irqreturn_t bam_dmux_remote_ready_irq(int irq, void *data)
{
	struct bam_dmux *dmux = data;

	dev_info(dmux->dev,
		 "remote ready (SMDINIT), scheduling A2 wake request in 500ms\n");

	/*
	 * Defer the APPS-bit-1 assertion so the modem has time to bring up
	 * its A2 SMSM callback after MPSS finishes loading.  See boot_work
	 * comment for the full sequence.
	 */
	schedule_delayed_work(&dmux->boot_work, msecs_to_jiffies(500));

	return IRQ_HANDLED;
}

static int bam_dmux_ssr_notify(struct notifier_block *nb, unsigned long action,
			       void *data)
{
	struct bam_dmux *dmux = container_of(nb, struct bam_dmux, ssr_nb);

	switch (action) {
	case QCOM_SSR_BEFORE_SHUTDOWN:
		dev_info(dmux->dev,
			 "SSR: modem going down, disabling BAM access\n");
		WRITE_ONCE(dmux->pipes_active, false);
		WRITE_ONCE(dmux->pc_state, false);
		dmux->boot_done = false;
		timer_delete(&dmux->rx_poll_timer);
		cancel_delayed_work(&dmux->boot_work);
		cancel_delayed_work(&dmux->ack_work);
		break;
	case QCOM_SSR_AFTER_SHUTDOWN:
		dev_info(dmux->dev, "SSR: modem stopped\n");
		break;
	case QCOM_SSR_AFTER_POWERUP:
		dev_info(dmux->dev,
			 "SSR: modem restarted, awaiting remote-ready\n");
		break;
	default:
		break;
	}

	return NOTIFY_DONE;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Probe / Remove
 * ────────────────────────────────────────────────────────────────────────── */

static int bam_dmux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *bam_node;
	struct bam_dmux *dmux;
	struct resource res;
	int ret, pc_ack_irq, i;
	unsigned int bit;

	dmux = devm_kzalloc(dev, sizeof(*dmux), GFP_KERNEL);
	if (!dmux)
		return -ENOMEM;

	dmux->dev = dev;
	platform_set_drvdata(pdev, dmux);

	/*
	 * Get BAM register base from the DMA controller node referenced
	 * by the dmas property.  We ioremap it directly — no DMA engine.
	 */
	bam_node = of_parse_phandle(dev->of_node, "dmas", 0);
	if (!bam_node) {
		dev_err(dev, "No BAM DMA controller node found\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(bam_node, 0, &res);
	if (ret) {
		of_node_put(bam_node);
		dev_err(dev, "Failed to get BAM resource: %d\n", ret);
		return ret;
	}

	/* Get BAM hardware IRQ from the DMA controller node.
	 * The downstream SPS layer registers bam_isr() for this IRQ.
	 * Without handling it, accumulated P_IRQ_STTS (especially P_WAKE)
	 * may prevent the BAM DMA engine from processing RX descriptors.
	 */
	dmux->bam_irq = of_irq_get(bam_node, 0);
	of_node_put(bam_node);

	dmux->bam_base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!dmux->bam_base) {
		dev_err(dev, "Failed to ioremap BAM at %pa\n", &res.start);
		return -ENOMEM;
	}

	dev_info(dev, "BAM at %pa size 0x%llx mapped to %px\n",
		 &res.start, (u64)resource_size(&res), dmux->bam_base);

	/*
	 * Do NOT read BAM registers here — the BAM clock domain is
	 * powered by the modem, which hasn't booted yet.  Reading
	 * causes an external abort (bus fault).
	 */

	/* SMSM power control */
	dmux->pc_irq = platform_get_irq_byname(pdev, "pc");
	if (dmux->pc_irq < 0)
		return dmux->pc_irq;

	pc_ack_irq = platform_get_irq_byname(pdev, "pc-ack");
	if (pc_ack_irq < 0)
		return pc_ack_irq;

	dmux->pc = devm_qcom_smem_state_get(dev, "pc", &bit);
	if (IS_ERR(dmux->pc))
		return dev_err_probe(dev, PTR_ERR(dmux->pc),
				     "Failed to get pc state\n");
	dmux->pc_mask = BIT(bit);

	dmux->pc_ack = devm_qcom_smem_state_get(dev, "pc-ack", &bit);
	if (IS_ERR(dmux->pc_ack))
		return dev_err_probe(dev, PTR_ERR(dmux->pc_ack),
				     "Failed to get pc-ack state\n");
	dmux->pc_ack_mask = BIT(bit);

	init_waitqueue_head(&dmux->pc_wait);
	init_completion(&dmux->pc_ack_completion);
	complete_all(&dmux->pc_ack_completion);

	spin_lock_init(&dmux->tx_lock);
	INIT_WORK(&dmux->tx_wakeup_work, bam_dmux_tx_wakeup_work);
	INIT_WORK(&dmux->register_netdev_work, bam_dmux_register_netdev_work);
	INIT_DELAYED_WORK(&dmux->boot_work, bam_dmux_boot_work_fn);
	INIT_DELAYED_WORK(&dmux->ack_work, bam_dmux_ack_work_fn);
	timer_setup(&dmux->rx_poll_timer, bam_dmux_poll_timer_fn, 0);

	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		dmux->rx_skbs[i].dmux = dmux;
		dmux->tx_skbs[i].dmux = dmux;
	}

	/* Runtime PM */
	pm_runtime_set_autosuspend_delay(dev, BAM_DMUX_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	ret = devm_request_threaded_irq(dev, pc_ack_irq, NULL,
					bam_dmux_pc_ack_irq,
					IRQF_ONESHOT, NULL, dmux);
	if (ret)
		goto err_disable_pm;

	ret = devm_request_threaded_irq(dev, dmux->pc_irq, NULL,
					bam_dmux_pc_irq,
					IRQF_ONESHOT, NULL, dmux);
	if (ret)
		goto err_disable_pm;

	ret = irq_get_irqchip_state(dmux->pc_irq, IRQCHIP_STATE_LINE_LEVEL,
				    &dmux->pc_state);
	if (ret)
		goto err_disable_pm;

	/* Register remote-ready IRQ (modem SMDINIT) */
	{
		int remote_ready_irq;

		remote_ready_irq = platform_get_irq_byname(pdev,
							    "remote-ready");
		if (remote_ready_irq > 0) {
			ret = devm_request_threaded_irq(dev, remote_ready_irq,
							NULL,
							bam_dmux_remote_ready_irq,
							IRQF_ONESHOT,
							NULL, dmux);
			if (ret)
				dev_warn(dev,
					 "remote-ready IRQ failed: %d\n", ret);
		}
	}

	dev_info(dev, "probe complete: pc_state=%d pc_irq=%d (NUCLEAR)\n",
		 dmux->pc_state, dmux->pc_irq);

	dmux->ssr_nb.notifier_call = bam_dmux_ssr_notify;
	dmux->ssr_cookie = qcom_register_ssr_notifier("mpss", &dmux->ssr_nb);
	if (IS_ERR(dmux->ssr_cookie)) {
		ret = PTR_ERR(dmux->ssr_cookie);
		dmux->ssr_cookie = NULL;
		dev_err(dev, "Failed to register mpss SSR notifier: %d\n", ret);
		goto err_disable_pm;
	}

	/* If modem already has bit 1 set, go directly to first_connect.
	 * This handles the case where modem boot completed before our
	 * driver probed (no pc_irq edge to trigger on). */
	if (dmux->pc_state)
		bam_dmux_first_connect(dmux);

	return 0;

err_disable_pm:
	pm_runtime_disable(dev);
	return ret;
}

static void bam_dmux_remove(struct platform_device *pdev)
{
	struct bam_dmux *dmux = platform_get_drvdata(pdev);
	int i;

	if (dmux->ssr_cookie) {
		qcom_unregister_ssr_notifier(dmux->ssr_cookie, &dmux->ssr_nb);
		dmux->ssr_cookie = NULL;
	}

	cancel_delayed_work_sync(&dmux->boot_work);
	cancel_delayed_work_sync(&dmux->ack_work);
	cancel_work_sync(&dmux->register_netdev_work);
	cancel_work_sync(&dmux->tx_wakeup_work);
	timer_delete_sync(&dmux->rx_poll_timer);

	bam_dmux_power_off(dmux);

	for (i = 0; i < BAM_DMUX_NUM_CH; i++) {
		if (dmux->netdevs[i]) {
			unregister_netdev(dmux->netdevs[i]);
			free_netdev(dmux->netdevs[i]);
		}
	}

	pm_runtime_disable(&pdev->dev);
}

static const struct dev_pm_ops bam_dmux_pm_ops = {
	SET_RUNTIME_PM_OPS(bam_dmux_runtime_suspend,
			   bam_dmux_runtime_resume, NULL)
};

static const struct of_device_id bam_dmux_of_match[] = {
	{ .compatible = "qcom,bam-dmux" },
	{}
};
MODULE_DEVICE_TABLE(of, bam_dmux_of_match);

static struct platform_driver bam_dmux_driver = {
	.probe = bam_dmux_probe,
	.remove = bam_dmux_remove,
	.driver = {
		.name = "bam-dmux",
		.pm = &bam_dmux_pm_ops,
		.of_match_table = bam_dmux_of_match,
	},
};
module_platform_driver(bam_dmux_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm BAM-DMUX WWAN (direct BAM register access)");
