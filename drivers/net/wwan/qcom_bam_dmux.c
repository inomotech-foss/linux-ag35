// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm BAM-DMUX WWAN network driver
 *
 * "Nuclear" rewrite: direct BAM register access, no DMA engine dependency.
 *
 * Attempt 20: Key changes from all previous attempts:
 *  - Do NOT set APPS bit 1 to trigger modem (wait for modem's autonomous
 *    bit 1 from a2_subsystem_boot).  Setting APPS bit 1 triggers the
 *    modem's uplink wakeup path, not the initial connect path.
 *  - Do BAM init directly in pc_irq handler (no workqueue delay).
 *  - Fix LOCAL_CLK_GATING: BIT(16) not BIT(17) (2-bit field, value=1).
 *  - Strip verbose diagnostic dumps for speed.
 *  - Register BAM IRQ before pipe init (matching downstream order).
 *  - 5s fallback timer: if modem doesn't set bit 1 autonomously,
 *    set APPS bit 1 as last resort.
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

static void bam_hw_init(struct bam_dmux *dmux)
{
	u32 val;

	val = bam_readl(dmux, BAM_CTRL);

	dev_info(dmux->dev,
		 "bam_hw_init: PRE BAM_CTRL=0x%08x CNFG=0x%08x\n",
		 val, bam_readl(dmux, BAM_CNFG_BITS));

	/*
	 * Step 1: NO SW_RST — preserve modem's BAM state.
	 *
	 * The modem's a2_bam_init() has already configured the BAM
	 * (including internal A2 DMA connections to pipes 4 and 5).
	 * SW_RST destroys all of that, and the modem never re-initializes
	 * because it doesn't know we reset.
	 */

	/*
	 * Step 2: Enable BAM, preserve modem's LOCAL_CLK_GATING value.
	 *
	 * The modem had BAM_CTRL=0x00020000 (LOCAL_CLK_GATING=2, BAM_EN=0).
	 * We add BAM_EN and clear CACHE_MISS_ERR_RESP_EN, but keep the
	 * modem's LOCAL_CLK_GATING setting intact.
	 */
	val |= BAM_EN;
	val &= ~BAM_CACHE_MISS_ERR_RESP_EN;
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
 * Pipe Init — matching downstream SPS bam_pipe_init()
 *
 * Sequence: P_RST → IRQ_SRCS_MSK → P_IRQ_EN → P_CTRL(dir+sys) →
 *           P_EVNT_GEN_TRSHLD → P_DESC_FIFO_ADDR → P_FIFO_SIZES → P_EN
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

	/*
	 * Reset pipe — but SKIP for the RX/producer pipe (pipe 5).
	 *
	 * The modem's a2_bam_init() configured the internal A2 DMA → pipe 5
	 * connection before we run.  P_RST destroys that internal connection
	 * and the modem never re-establishes it.  For pipe 4 (consumer/TX),
	 * P_RST is safe because we are the producer writing descriptors.
	 */
	if (!producer) {
		bam_writel_sync(dmux, BAM_P_RST(pipe_index), 1);
		bam_writel_sync(dmux, BAM_P_RST(pipe_index), 0);
	}

	/* Unmask this pipe in EE 0's IRQ sources */
	val = bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE));
	val |= BIT(pipe_index);
	bam_writel(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE), val);

	/* Enable pipe IRQ — P_TRNSFR_END_EN = 0x20 (EOT) */
	bam_writel(dmux, BAM_P_IRQ_EN(pipe_index), P_TRNSFR_END_EN);

	/* Direction + system mode (NOT yet P_EN) */
	val = P_SYS_MODE;
	if (producer)
		val |= P_DIRECTION;
	bam_writel(dmux, BAM_P_CTRL(pipe_index), val);

	/* Event threshold */
	bam_writel(dmux, BAM_P_EVNT_GEN_TRSHLD(pipe_index), 0x10);

	/* Descriptor FIFO address + size */
	bam_writel(dmux, BAM_P_DESC_FIFO_ADDR(pipe_index),
		   lower_32_bits(pipe->desc_fifo_phys));
	bam_writel(dmux, BAM_P_FIFO_SIZES(pipe_index), BAM_DESC_FIFO_SIZE);

	/* Enable pipe (LAST) */
	val = bam_readl(dmux, BAM_P_CTRL(pipe_index));
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
	 */
	if (!READ_ONCE(dmux->pipes_active) || !READ_ONCE(dmux->pc_state))
		return;

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
 * SMSM IRQ Handlers + Boot Sequence (Attempt 22)
 *
 * Downstream flow (from bam_dmux.c + modem pseudocode):
 *   1. Modem's a2_subsystem_boot() sets SMSM_A2_POWER_CONTROL autonomously
 *   2. APPS bam_dmux_smsm_cb fires synchronously
 *   3. APPS does: SW_RST → BAM_EN → pipe4 → pipe5 → toggle_ack → queue_rx
 *   4. Modem's a2_apps_smsm_ack_callback fires → apps_bam_link_ready=true
 *   5. Modem sends CMD_OPEN on pipe 5
 *
 * Key insight from attempt 20-21 failure analysis:
 *   - The modem on this firmware does NOT set bit 1 autonomously.
 *     It only responds when APPS sets bit 1 (the UL wakeup path).
 *   - APPS bit 1 must stay SET throughout the reconnect flow.
 *   - SW_RST destroys the modem's a2_bam_init() BAM configuration.
 *     Even though the downstream also does SW_RST, something about our
 *     re-initialization differs.  Attempt 22 skips SW_RST.
 *
 * Attempt 22 changes:
 *   - NO SW_RST: preserve modem's BAM_CTRL (LOCAL_CLK_GATING=2) and
 *     internal A2 DMA connections.  Just add BAM_EN + CNFG_BITS.
 *   - RX descriptor flags=0 (matching downstream sps_transfer_one)
 *   - Keep APPS bit 1 SET from attempt 21
 *   - Keep fast inline init from attempt 20
 * ────────────────────────────────────────────────────────────────────────── */

/* ──────────────────────────────────────────────────────────────────────────
 * First-connect: BAM init + SMSM ACK + queue RX
 *
 * Called from pc_irq handler (threaded) when modem sets bit 1.
 * Does everything inline for minimum latency — matching the downstream's
 * synchronous bam_dmux_smsm_cb → bam_init → toggle_ack → queue_rx flow.
 *
 * Attempt 22 key changes vs. attempt 21:
 *  - NO SW_RST in bam_hw_init() — preserve modem's BAM state including
 *    its LOCAL_CLK_GATING=2 and A2 DMA pipe connections.
 *    All previous attempts (17-21) did SW_RST which wipes the modem's
 *    a2_bam_init() configuration.  Attempt 18 skipped SW_RST but missed
 *    CNFG_BITS.  This is the first attempt with no-SW_RST + CNFG_BITS +
 *    keep-bit-1 all combined.
 *  - RX descriptor flags=0 (matching downstream sps_transfer_one flags=0).
 *    Attempt 21 wrongly added DESC_FLAG_INT; downstream passes 0.
 *  - Keep APPS bit 1 SET from attempt 21.
 *  - Keep inline init, BAM IRQ before pipes
 * ────────────────────────────────────────────────────────────────────────── */

static void bam_dmux_first_connect(struct bam_dmux *dmux)
{
	int i, ret;

	dev_info(dmux->dev,
		 "first_connect: BAM init (attempt 23: no SW_RST, no P_RST on "
		 "pipe 5, preserve modem BAM_CTRL)\n");

	/* Step 1: Full BAM global init with SW_RST */
	bam_hw_init(dmux);

	/* Step 2: Register BAM IRQ BEFORE pipes (matching downstream) */
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

	/* Step 3: Init TX pipe (pipe 4, consumer) */
	ret = bam_pipe_hw_init(dmux, &dmux->tx_pipe,
			       BAM_DMUX_TX_PIPE, false);
	if (ret) {
		dev_err(dmux->dev, "TX pipe init failed: %d\n", ret);
		return;
	}

	/* Step 4: Init RX pipe (pipe 5, producer) — dump PRE state */
	dev_info(dmux->dev,
		 "P5 PRE: CTRL=0x%08x DESC=0x%08x FIFO_SZ=0x%08x SW=0x%08x EVNT=0x%08x\n",
		 bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_DESC_FIFO_ADDR(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_FIFO_SIZES(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_SW_OFSTS(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_P_EVNT_REG(BAM_DMUX_RX_PIPE)));
	ret = bam_pipe_hw_init(dmux, &dmux->rx_pipe,
			       BAM_DMUX_RX_PIPE, true);
	if (ret) {
		dev_err(dmux->dev, "RX pipe init failed: %d\n", ret);
		bam_pipe_deinit(dmux, &dmux->tx_pipe);
		return;
	}

	dmux->pipes_active = true;
	dmux->boot_done = true;

	/* PM setup */
	pm_runtime_disable(dmux->dev);
	pm_runtime_set_active(dmux->dev);
	pm_runtime_enable(dmux->dev);
	pm_runtime_get_noresume(dmux->dev);
	pm_runtime_get_noresume(dmux->dev);

	/*
	 * Step 5: DO NOT clear APPS bit 1!
	 *
	 * In the downstream, APPS bit 1 stays SET throughout the entire
	 * reconnect flow (reconnect_to_bam → toggle_ack → queue_rx).
	 * It is only cleared later by ul_powerdown() when the link idles.
	 *
	 * Clearing APPS bit 1 here triggers the modem's a2_apps_smsm_callback
	 * with bit 1 = 0, which the modem interprets as "APPS wants to power
	 * down A2."  This causes the modem to shut down its A2 DMA engine
	 * right before our ACK arrives, so the modem never sets
	 * apps_bam_link_ready and never writes to pipe 5.
	 *
	 * Previous attempts 1-19 had wrong LOCAL_CLK_GATING (BIT(17)),
	 * attempt 20 cleared bit 1 here.  This is the first attempt with
	 * both fixes correct simultaneously.
	 */

	/* Step 6: Toggle ACK (APPS bit 11) — BEFORE queue_rx!
	 * Downstream: toggle_apps_ack() before queue_rx() in bam_init().
	 */
	dev_info(dmux->dev, "first_connect: toggling APPS bit 11 (ACK)\n");
	bam_dmux_pc_ack(dmux);

	/* Step 7: Queue 32 RX descriptors + doorbell */
	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		if (!bam_dmux_queue_rx(dmux, &dmux->rx_skbs[i], GFP_KERNEL))
			dev_err(dmux->dev, "RX queue %d failed\n", i);
	}
	bam_pipe_doorbell(dmux, &dmux->rx_pipe);

	/* Step 8: Clear all pending IRQ status */
	bam_clear_irqs(dmux);

	dev_info(dmux->dev,
		 "first_connect: done, P4_CTRL=0x%08x P5_CTRL=0x%08x "
		 "IRQ_SRCS_MSK=0x%08x\n",
		 bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_TX_PIPE)),
		 bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE(BAM_DMUX_EE)));

	/* Step 9: Set up CMD_OPEN fallback timer + start polling */
	dmux->cmd_open_retries = 0;
	dmux->cmd_open_acked = false;
	dmux->cmd_open_next_retry = jiffies + 15 * HZ;

	mod_timer(&dmux->rx_poll_timer, jiffies + msecs_to_jiffies(20));
}

static void bam_dmux_ack_work_fn(struct work_struct *work)
{
	struct bam_dmux *dmux = container_of(work, struct bam_dmux,
					     ack_work.work);

	/* ack_work is now only used as a fallback; main path is pc_irq. */
	if (dmux->pipes_active) {
		dev_info(dmux->dev, "ack_work: already active, skip\n");
		return;
	}

	if (!dmux->pc_state) {
		dev_warn(dmux->dev, "ack_work: modem bit 1 not set, skip\n");
		return;
	}

	bam_dmux_first_connect(dmux);
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
		 * Modem set A2_POWER_CONTROL (bit 1) — first connect.
		 *
		 * Do BAM init DIRECTLY here for minimum latency.
		 * The downstream bam_dmux_smsm_cb runs synchronously in
		 * the SMSM callback context (<1ms total).  Previous
		 * attempts used workqueue scheduling (ack_work) which
		 * added 34ms+ of scheduling delay on top of ~85ms of
		 * diagnostic dumps.
		 *
		 * Cancel boot_work — modem responded (autonomously or
		 * to our fallback APPS bit 1 trigger).
		 */
		dmux->pc_state = true;
		cancel_delayed_work(&dmux->boot_work);
		bam_dmux_first_connect(dmux);

	} else if (new_state && dmux->pc_state) {
		if (dmux->pipes_active)
			bam_pipe_doorbell(dmux, &dmux->rx_pipe);
		dev_info(dmux->dev,
			 "pc_irq: already up, re-issued doorbell\n");
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

	if (dmux->pc_state || dmux->pipes_active)
		return;

	/*
	 * Fallback: modem didn't set bit 1 autonomously within 5 seconds.
	 *
	 * In the downstream, the modem sets SMSM_A2_POWER_CONTROL on its
	 * own during a2_subsystem_boot() — APPS never sets bit 1 for the
	 * initial handshake.  APPS bit 1 is only for uplink wakeup.
	 *
	 * Setting APPS bit 1 triggers modem's a2_apps_smsm_callback which
	 * is the "uplink wakeup" path, NOT the "initial connect" path.
	 * This may cause the modem to enter a different state machine.
	 *
	 * Attempt 20: Wait 5s for modem's autonomous bit 1.  If it
	 * doesn't come, set APPS bit 1 as a last resort.
	 */
	dev_info(dmux->dev,
		 "boot_work: modem didn't set bit 1 in 5s, "
		 "setting APPS bit 1 as fallback trigger\n");

	bam_dmux_pc_vote(dmux, true);
}

static irqreturn_t bam_dmux_remote_ready_irq(int irq, void *data)
{
	struct bam_dmux *dmux = data;

	dev_info(dmux->dev,
		 "remote ready (SMDINIT), scheduling boot_work in 5s\n");
	schedule_delayed_work(&dmux->boot_work, msecs_to_jiffies(5000));

	return IRQ_HANDLED;
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

	/* If modem already has bit 1 set, go directly to BAM init
	 * (matching downstream check after smsm_state_cb_register) */
	if (dmux->pc_state)
		schedule_delayed_work(&dmux->ack_work, 0);

	return 0;

err_disable_pm:
	pm_runtime_disable(dev);
	return ret;
}

static void bam_dmux_remove(struct platform_device *pdev)
{
	struct bam_dmux *dmux = platform_get_drvdata(pdev);
	int i;

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
