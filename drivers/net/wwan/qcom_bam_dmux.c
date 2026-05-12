// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm BAM-DMUX WWAN network driver — direct BAM register access
 *
 * Port of the downstream msm_bam_dmux.c driver to upstream Linux.
 * Bypasses the DMA engine framework and talks to the BAM hardware
 * directly, matching the downstream SPS (Smart Peripheral System)
 * register-level operations 1:1.
 *
 * This is necessary on MDM9607 with Quectel OCPU firmware because:
 * 1. The modem does a BAM SW_RST after APPS sets SMSM bit 1,
 *    wiping all pipe configuration. APPS must re-init pipes AFTER
 *    the modem responds.
 * 2. TrustZone blocks the BAM interrupt, requiring polling.
 * 3. The DMA engine abstraction doesn't support the modem-resets-BAM
 *    lifecycle.
 *
 * Copyright (c) 2020, Stephan Gerhold <stephan@gerhold.net> (original upstream)
 * Copyright (c) 2011-2016, The Linux Foundation (downstream msm_bam_dmux)
 * Modified for direct BAM access, 2025.
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <net/pkt_sched.h>

/* ===== Protocol constants ===== */
#define BAM_DMUX_BUFFER_SIZE		SZ_2K
#define BAM_DMUX_HDR_SIZE		sizeof(struct bam_dmux_hdr)
#define BAM_DMUX_MAX_DATA_SIZE		(BAM_DMUX_BUFFER_SIZE - BAM_DMUX_HDR_SIZE)
#define BAM_DMUX_NUM_SKB		32
#define BAM_DMUX_HDR_MAGIC		0x33fc

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
	__le16 magic;
	u8 signal;
	u8 cmd;
	u8 pad;
	u8 ch;
	__le16 len;
};

/* ===== BAM hardware register definitions (v1.7.0 / NDP 4K) ===== */

/* Descriptor FIFO: 2KB = 256 slots of 8 bytes each */
#define BAM_DESC_FIFO_SIZE		SZ_2K
#define BAM_DESC_SIZE			8
#define BAM_NUM_DESC			(BAM_DESC_FIFO_SIZE / BAM_DESC_SIZE)

/* BAM global registers */
#define BAM_CTRL			0x00000
#define BAM_REVISION			0x01000
#define BAM_NUM_PIPES			0x01008
#define BAM_DESC_CNT_TRSHLD		0x00008
#define BAM_IRQ_SRCS_EE			0x03000	/* EE0: pending IRQ sources */
#define BAM_IRQ_SRCS_MSK_EE		0x03004	/* EE0: IRQ source mask */
#define BAM_IRQ_SRCS_UNMASKED_EE	0x03008	/* EE0 */
#define BAM_PIPE_ATTR_EE		0x0300C	/* EE0: pipe attributes */
#define BAM_IRQ_EN			0x0001C
#define BAM_CNFG_BITS			0x0007C
#define BAM_IRQ_STTS			0x00014
#define BAM_IRQ_CLR			0x00018

/* BAM trust/security registers (NDP 4K) */
#define BAM_TRUST_REG			0x02000
#define BAM_P_TRUST_REG(n)		(0x02020 + (n) * 0x4)

/* BAM_CTRL bits */
#define BAM_SW_RST			BIT(0)
#define BAM_EN				BIT(1)

/* BAM_IRQ_EN bits */
#define BAM_TIMER_EN			BIT(4)
#define BAM_ERROR_EN			BIT(2)
#define BAM_HRESP_ERR_EN		BIT(1)

/* BAM per-pipe registers (pipe N at base + 0x13000 + N*0x1000) */
#define BAM_P_CTRL(n)			(0x13000 + (n) * 0x1000)
#define BAM_P_RST(n)			(0x13004 + (n) * 0x1000)
#define BAM_P_HALT(n)			(0x13008 + (n) * 0x1000)
#define BAM_P_IRQ_STTS(n)		(0x13010 + (n) * 0x1000)
#define BAM_P_IRQ_CLR(n)		(0x13014 + (n) * 0x1000)
#define BAM_P_IRQ_EN(n)			(0x13018 + (n) * 0x1000)

/* BAM per-pipe event registers */
#define BAM_P_SW_OFSTS(n)		(0x13800 + (n) * 0x1000)
#define BAM_P_EVNT_REG(n)		(0x13818 + (n) * 0x1000)
#define BAM_P_DESC_FIFO_ADDR(n)		(0x1381C + (n) * 0x1000)
#define BAM_P_FIFO_SIZES(n)		(0x13820 + (n) * 0x1000)
#define BAM_P_EVNT_GEN_TRSHLD(n)	(0x13828 + (n) * 0x1000)

/* BAM per-pipe context registers (read-only, HW state) */
#define BAM_P_AU_PSM_CNTXT_1(n)		(0x13804 + (n) * 0x1000)
#define BAM_P_PSM_CNTXT_2(n)		(0x13808 + (n) * 0x1000)
#define BAM_P_PRDCR_SDBND(n)		(0x13024 + (n) * 0x1000)
#define BAM_P_CNSMR_SDBND(n)		(0x13028 + (n) * 0x1000)
#define BAM_P_RETR_CNTXT(n)		(0x13834 + (n) * 0x1000)

/* BAM_P_CTRL bits */
#define P_EN				BIT(1)
#define P_DIRECTION			BIT(3)
#define P_SYS_MODE			BIT(5)

/* BAM_P_SW_OFSTS: offset in lower 16 bits */
#define P_SW_OFSTS_MASK			0xFFFF

/* BAM_P_IRQ_EN bits */
#define P_PRCSD_DESC_EN			BIT(0)
#define P_ERR_EN			BIT(4)
#define P_TRNSFR_END_EN			BIT(5)

/* Descriptor flags */
#define DESC_FLAG_INT			cpu_to_le16(BIT(15))
#define DESC_FLAG_EOT			cpu_to_le16(BIT(14))

/* Pipe numbers (hardcoded in BAM DMUX protocol) */
#define BAM_DMUX_TX_PIPE		4
#define BAM_DMUX_RX_PIPE		5

/* Downstream constants */
#define A2_NUM_PIPES			6
#define DEFAULT_CNT_THRSHLD		0x1000
#define BAM_CNFG_BITS_DEFAULT		0xFFFFF7FF

/* ===== Hardware descriptor (8 bytes, naturally aligned) ===== */
struct bam_desc_hw {
	__le32 addr;
	__le16 size;
	__le16 flags;
} __packed;

/* ===== Per-pipe state ===== */
struct bam_dmux_pipe {
	u32 pipe_num;

	/* Descriptor FIFO (DMA coherent) */
	struct bam_desc_hw *desc_fifo;
	dma_addr_t desc_fifo_phys;

	/*
	 * Circular buffer indices (in descriptor units, not bytes).
	 * head: next slot to read a completion from
	 * tail: next slot to write a new descriptor to
	 */
	u16 head;
	u16 tail;
};

/* ===== Per-buffer tracking ===== */
struct bam_dmux_skb_dma {
	struct sk_buff *skb;
	dma_addr_t addr;
};

/* ===== Main device state ===== */
struct bam_dmux {
	struct device *dev;
	void __iomem *bam_base;

	/* SMSM */
	int pc_irq;
	int bam_irq;	/* GIC SPI 29 — BAM hardware interrupt */
	bool pc_state, pc_ack_state;
	struct qcom_smem_state *pc, *pc_ack;
	u32 pc_mask, pc_ack_mask;

	/* BAM pipes */
	struct bam_dmux_pipe tx_pipe;
	struct bam_dmux_pipe rx_pipe;

	/* RX buffer pool */
	struct bam_dmux_skb_dma rx_skbs[BAM_DMUX_NUM_SKB];

	/* TX buffer pool */
	struct bam_dmux_skb_dma tx_skbs[BAM_DMUX_NUM_SKB];
	spinlock_t tx_lock;
	unsigned int tx_next_skb;	/* next slot for TX submission */

	/* Work/timers */
	struct delayed_work boot_work;
	struct timer_list poll_timer;
	struct work_struct register_netdev_work;

	/* State */
	bool bam_initialized;

	/* Netdevs */
	DECLARE_BITMAP(remote_channels, BAM_DMUX_NUM_CH);
	struct net_device *netdevs[BAM_DMUX_NUM_CH];
};

struct bam_dmux_netdev {
	struct bam_dmux *dmux;
	u8 ch;
};

/* ===== BAM register access helpers ===== */

static inline u32 bam_readl(struct bam_dmux *dmux, u32 offset)
{
	return readl_relaxed(dmux->bam_base + offset);
}

/*
 * Use writel() (ordered) for all BAM register writes.
 * Matches downstream iowrite32 semantics.
 */
static inline void bam_writel(struct bam_dmux *dmux, u32 offset, u32 val)
{
	writel(val, dmux->bam_base + offset);
}

/* ===== BAM global init (matches downstream bam_init / sps_bam_reset) ===== */

static void bam_dmux_bam_init(struct bam_dmux *dmux)
{
	u32 val;
	u32 pre_trust, pre_p4_trust, pre_p5_trust, pre_pipe_attr;
	u32 pre_p4_ctrl, pre_p5_ctrl;

	val = bam_readl(dmux, BAM_CTRL);

	/* Read ALL state BEFORE SW_RST to see what TZ/modem configured */
	pre_trust = bam_readl(dmux, BAM_TRUST_REG);
	pre_p4_trust = bam_readl(dmux, BAM_P_TRUST_REG(BAM_DMUX_TX_PIPE));
	pre_p5_trust = bam_readl(dmux, BAM_P_TRUST_REG(BAM_DMUX_RX_PIPE));
	pre_pipe_attr = bam_readl(dmux, BAM_PIPE_ATTR_EE);
	pre_p4_ctrl = bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_TX_PIPE));
	pre_p5_ctrl = bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_RX_PIPE));

	dev_info(dmux->dev, "bam_init: PRE-RST BAM_CTRL=0x%08x "
		 "TRUST=0x%x P_TR[4]=0x%x P_TR[5]=0x%x "
		 "PIPE_ATTR=0x%x P4_CTRL=0x%x P5_CTRL=0x%x\n",
		 val, pre_trust, pre_p4_trust, pre_p5_trust,
		 pre_pipe_attr, pre_p4_ctrl, pre_p5_ctrl);

	/*
	 * SW_RST: downstream sps_register_bam_device does this.
	 * Trust registers are all-zero before and after reset on
	 * MDM9607. Re-enabled for 1:1 downstream match.
	 */
	bam_writel(dmux, BAM_CTRL, val | BAM_SW_RST);
	bam_writel(dmux, BAM_CTRL, val & ~BAM_SW_RST);
	wmb();

	/* Enable BAM + LOCAL_CLK_GATING (match downstream RMW sequence) */
	val = bam_readl(dmux, BAM_CTRL);
	val |= BAM_EN;
	val &= ~(3 << 17);	/* clear LOCAL_CLK_GATING field (bits 17:16) */
	val |= (1 << 17);	/* set LOCAL_CLK_GATING = 1 */
	val &= ~BIT(19);	/* clear CACHE_MISS_ERR_RESP_EN */
	val &= ~BIT(20);	/* clear BAM_MESS_ONLY_CANCEL_WB */
	bam_writel(dmux, BAM_CTRL, val);

	/* Descriptor count threshold */
	bam_writel(dmux, BAM_DESC_CNT_TRSHLD, DEFAULT_CNT_THRSHLD);

	/* Configuration bits (all workarounds enabled except BAM_FULL_PIPE) */
	bam_writel(dmux, BAM_CNFG_BITS, BAM_CNFG_BITS_DEFAULT);

	/* IRQ enable: error + timer */
	bam_writel(dmux, BAM_IRQ_EN,
		   BAM_TIMER_EN | BAM_ERROR_EN | BAM_HRESP_ERR_EN);

	/* Unmask global BAM interrupt + pipe interrupts */
	bam_writel(dmux, BAM_IRQ_SRCS_MSK_EE,
		   BIT(31) | BIT(BAM_DMUX_TX_PIPE) | BIT(BAM_DMUX_RX_PIPE));

	dev_info(dmux->dev, "bam_init: BAM_CTRL=0x%08x CNFG=0x%08x "
		 "IRQ_MSK=0x%08x REV=0x%08x PIPES=0x%08x\n",
		 bam_readl(dmux, BAM_CTRL),
		 bam_readl(dmux, BAM_CNFG_BITS),
		 bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE),
		 bam_readl(dmux, BAM_REVISION),
		 bam_readl(dmux, BAM_NUM_PIPES));

	/* Trust/security diagnostics — critical for EE ownership debugging */
	dev_info(dmux->dev, "bam_init: TRUST_REG=0x%08x "
		 "P_TRUST[4]=0x%08x P_TRUST[5]=0x%08x "
		 "IRQ_SRCS=0x%08x PIPE_ATTR=0x%08x\n",
		 bam_readl(dmux, BAM_TRUST_REG),
		 bam_readl(dmux, BAM_P_TRUST_REG(BAM_DMUX_TX_PIPE)),
		 bam_readl(dmux, BAM_P_TRUST_REG(BAM_DMUX_RX_PIPE)),
		 bam_readl(dmux, BAM_IRQ_SRCS_EE),
		 bam_readl(dmux, BAM_PIPE_ATTR_EE));
}

/* ===== Pipe init (matches downstream sps_connect) ===== */

static void bam_dmux_pipe_init(struct bam_dmux *dmux,
			       struct bam_dmux_pipe *pipe,
			       bool dir_producer)
{
	u32 p = pipe->pipe_num;
	u32 val;

	dev_info(dmux->dev, "pipe %u init: PRE-RESET P_CTRL=0x%x "
		 "P_DESC=0x%x P_SIZES=0x%x P_EVNT=0x%x P_OFSTS=0x%x\n",
		 p, bam_readl(dmux, BAM_P_CTRL(p)),
		 bam_readl(dmux, BAM_P_DESC_FIFO_ADDR(p)),
		 bam_readl(dmux, BAM_P_FIFO_SIZES(p)),
		 bam_readl(dmux, BAM_P_EVNT_REG(p)),
		 bam_readl(dmux, BAM_P_SW_OFSTS(p)));

	/*
	 * Downstream bam_pipe_init sequence (1:1 match):
	 * 1. P_RST
	 * 2. IRQ_SRCS_MSK_EE: set pipe bit
	 * 3. P_IRQ_EN = 0 (disabled during init)
	 * 4. P_CTRL: direction
	 * 5. P_CTRL: sys_mode
	 * 6. P_EVNT_GEN_TRSHLD
	 * 7. P_DESC_FIFO_ADDR
	 * 8. P_FIFO_SIZES
	 * 9. P_CTRL: sys_strm = 0
	 * 10. P_CTRL: lock_group = 0
	 * 11. P_CTRL: P_EN = 1
	 */

	/* Step 1: Pipe reset */
	bam_writel(dmux, BAM_P_RST(p), 1);
	bam_writel(dmux, BAM_P_RST(p), 0);

	/* Step 2: Enable pipe interrupt source at BAM level */
	val = bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE);
	val |= BIT(p);
	bam_writel(dmux, BAM_IRQ_SRCS_MSK_EE, val);

	/* Step 3: Disable pipe IRQ during init (downstream sets 0 here) */
	bam_writel(dmux, BAM_P_IRQ_EN(p), 0);

	/* Steps 4-5: Set direction and system mode via RMW */
	val = bam_readl(dmux, BAM_P_CTRL(p));
	if (dir_producer)
		val |= P_DIRECTION;
	else
		val &= ~P_DIRECTION;
	val |= P_SYS_MODE;
	bam_writel(dmux, BAM_P_CTRL(p), val);

	/* Step 6: Event threshold */
	bam_writel(dmux, BAM_P_EVNT_GEN_TRSHLD(p), 0x10);

	/* Step 7-8: Descriptor FIFO base + size */
	bam_writel(dmux, BAM_P_DESC_FIFO_ADDR(p), pipe->desc_fifo_phys);
	bam_writel(dmux, BAM_P_FIFO_SIZES(p), BAM_DESC_FIFO_SIZE);

	/* Steps 9-10: sys_strm=0, lock_group=0 (already 0 after reset) */

	/* Step 11: Enable pipe (RMW to preserve direction + mode) */
	val = bam_readl(dmux, BAM_P_CTRL(p));
	val |= P_EN;
	bam_writel(dmux, BAM_P_CTRL(p), val);

	/* Read back to confirm */
	val = bam_readl(dmux, BAM_P_CTRL(p));

	/* Reset FIFO pointers */
	pipe->head = 0;
	pipe->tail = 0;

	dev_info(dmux->dev, "pipe %u init: POST P_CTRL=0x%x "
		 "P_DESC=0x%x P_SIZES=0x%x P_HALT=0x%x "
		 "P_IRQ_EN=0x%x IRQ_MSK=0x%08x\n",
		 p, val,
		 bam_readl(dmux, BAM_P_DESC_FIFO_ADDR(p)),
		 bam_readl(dmux, BAM_P_FIFO_SIZES(p)),
		 bam_readl(dmux, BAM_P_HALT(p)),
		 bam_readl(dmux, BAM_P_IRQ_EN(p)),
		 bam_readl(dmux, BAM_IRQ_SRCS_MSK_EE));
}

/* ===== Descriptor FIFO operations ===== */

static int bam_dmux_pipe_submit(struct bam_dmux *dmux,
				struct bam_dmux_pipe *pipe,
				dma_addr_t addr, u16 size, __le16 flags)
{
	u16 next = (pipe->tail + 1) % BAM_NUM_DESC;

	if (next == pipe->head) {
		dev_err(dmux->dev, "pipe %u: FIFO full\n", pipe->pipe_num);
		return -ENOSPC;
	}

	pipe->desc_fifo[pipe->tail].addr = cpu_to_le32(addr);
	pipe->desc_fifo[pipe->tail].size = cpu_to_le16(size);
	pipe->desc_fifo[pipe->tail].flags = flags;

	/* Ensure descriptor is written before advancing tail */
	wmb();

	pipe->tail = next;
	return 0;
}

static void bam_dmux_pipe_doorbell(struct bam_dmux *dmux,
				   struct bam_dmux_pipe *pipe)
{
	bam_writel(dmux, BAM_P_EVNT_REG(pipe->pipe_num),
		   pipe->tail * BAM_DESC_SIZE);
}

static bool bam_dmux_pipe_get_completion(struct bam_dmux *dmux,
					 struct bam_dmux_pipe *pipe,
					 dma_addr_t *addr, u16 *size)
{
	u32 offset;
	u16 hw_head;

	offset = bam_readl(dmux, BAM_P_SW_OFSTS(pipe->pipe_num));
	hw_head = (offset & P_SW_OFSTS_MASK) / BAM_DESC_SIZE;

	if (hw_head == pipe->head)
		return false;

	*addr = le32_to_cpu(pipe->desc_fifo[pipe->head].addr);
	*size = le16_to_cpu(pipe->desc_fifo[pipe->head].size);

	pipe->head = (pipe->head + 1) % BAM_NUM_DESC;
	return true;
}

/* ===== SMSM helpers ===== */

static void bam_dmux_pc_vote(struct bam_dmux *dmux, bool enable)
{
	qcom_smem_state_update_bits(dmux->pc, dmux->pc_mask,
				    enable ? dmux->pc_mask : 0);
}

static void bam_dmux_pc_ack(struct bam_dmux *dmux)
{
	qcom_smem_state_update_bits(dmux->pc_ack, dmux->pc_ack_mask,
				    dmux->pc_ack_state ? 0 : dmux->pc_ack_mask);
	dmux->pc_ack_state = !dmux->pc_ack_state;
}

/* ===== Network device operations ===== */

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

static int bam_dmux_tx_skb(struct bam_dmux *dmux, struct sk_buff *skb)
{
	struct bam_dmux_skb_dma *skb_dma;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dmux->tx_lock, flags);

	skb_dma = &dmux->tx_skbs[dmux->tx_next_skb % BAM_DMUX_NUM_SKB];
	if (skb_dma->skb) {
		bam_dmux_tx_stop_queues(dmux);
		spin_unlock_irqrestore(&dmux->tx_lock, flags);
		return -EBUSY;
	}
	skb_dma->skb = skb;
	skb_dma->addr = dma_map_single(dmux->dev, skb->data, skb->len,
				       DMA_TO_DEVICE);
	if (dma_mapping_error(dmux->dev, skb_dma->addr)) {
		skb_dma->skb = NULL;
		spin_unlock_irqrestore(&dmux->tx_lock, flags);
		return -ENOMEM;
	}

	ret = bam_dmux_pipe_submit(dmux, &dmux->tx_pipe, skb_dma->addr,
				   skb->len, DESC_FLAG_EOT);
	if (ret) {
		dma_unmap_single(dmux->dev, skb_dma->addr, skb->len,
				 DMA_TO_DEVICE);
		skb_dma->addr = 0;
		skb_dma->skb = NULL;
		spin_unlock_irqrestore(&dmux->tx_lock, flags);
		return ret;
	}
	bam_dmux_pipe_doorbell(dmux, &dmux->tx_pipe);

	dmux->tx_next_skb++;
	if (dmux->tx_skbs[dmux->tx_next_skb % BAM_DMUX_NUM_SKB].skb)
		bam_dmux_tx_stop_queues(dmux);

	spin_unlock_irqrestore(&dmux->tx_lock, flags);
	return 0;
}

static int bam_dmux_send_cmd(struct bam_dmux *dmux, u8 cmd, u8 ch)
{
	struct bam_dmux_hdr *hdr;
	struct sk_buff *skb;
	int ret;

	skb = alloc_skb(sizeof(*hdr), GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	hdr = skb_put_zero(skb, sizeof(*hdr));
	hdr->magic = cpu_to_le16(BAM_DMUX_HDR_MAGIC);
	hdr->cmd = cmd;
	hdr->ch = ch;

	ret = bam_dmux_tx_skb(dmux, skb);
	if (ret)
		dev_kfree_skb(skb);

	return ret;
}

static int bam_dmux_netdev_open(struct net_device *netdev)
{
	struct bam_dmux_netdev *bndev = netdev_priv(netdev);
	struct bam_dmux *dmux = bndev->dmux;
	int ret;

	if (!dmux->bam_initialized)
		return -ENODEV;

	ret = bam_dmux_send_cmd(dmux, BAM_DMUX_CMD_OPEN, bndev->ch);
	if (ret)
		return ret;

	netif_start_queue(netdev);
	return 0;
}

static int bam_dmux_netdev_stop(struct net_device *netdev)
{
	struct bam_dmux_netdev *bndev = netdev_priv(netdev);
	struct bam_dmux *dmux = bndev->dmux;

	netif_stop_queue(netdev);
	if (dmux->bam_initialized)
		bam_dmux_send_cmd(dmux, BAM_DMUX_CMD_CLOSE, bndev->ch);
	return 0;
}

static unsigned int needed_room(unsigned int avail, unsigned int needed)
{
	if (avail >= needed)
		return 0;
	return needed - avail;
}

static netdev_tx_t bam_dmux_netdev_start_xmit(struct sk_buff *skb,
					       struct net_device *netdev)
{
	struct bam_dmux_netdev *bndev = netdev_priv(netdev);
	struct bam_dmux *dmux = bndev->dmux;
	unsigned int head, pad_len, tail;
	struct bam_dmux_hdr *hdr;
	int ret;

	if (!dmux->bam_initialized) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	head = needed_room(skb_headroom(skb), BAM_DMUX_HDR_SIZE);
	pad_len = sizeof(u32) - skb->len % sizeof(u32);
	if (pad_len == sizeof(u32))
		pad_len = 0;
	tail = needed_room(skb_tailroom(skb), pad_len);

	if (head || tail || skb_cloned(skb)) {
		ret = pskb_expand_head(skb, head, tail, GFP_ATOMIC);
		if (ret) {
			dev_kfree_skb_any(skb);
			return NETDEV_TX_OK;
		}
	}

	hdr = skb_push(skb, sizeof(*hdr));
	hdr->magic = cpu_to_le16(BAM_DMUX_HDR_MAGIC);
	hdr->signal = 0;
	hdr->cmd = BAM_DMUX_CMD_DATA;
	hdr->pad = pad_len;
	hdr->ch = bndev->ch;
	hdr->len = cpu_to_le16(skb->len - sizeof(*hdr));
	if (pad_len)
		skb_put_zero(skb, pad_len);

	ret = bam_dmux_tx_skb(dmux, skb);
	if (ret == -EBUSY)
		return NETDEV_TX_BUSY;
	if (ret)
		dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;
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

/* ===== RX path ===== */

static bool bam_dmux_submit_rx(struct bam_dmux *dmux, int idx)
{
	struct bam_dmux_skb_dma *skb_dma = &dmux->rx_skbs[idx];
	int ret;

	if (!skb_dma->skb) {
		skb_dma->skb = __netdev_alloc_skb(NULL, BAM_DMUX_BUFFER_SIZE,
						   GFP_ATOMIC);
		if (!skb_dma->skb)
			return false;
		skb_put(skb_dma->skb, BAM_DMUX_BUFFER_SIZE);
	}

	skb_dma->addr = dma_map_single(dmux->dev, skb_dma->skb->data,
				       skb_dma->skb->len, DMA_FROM_DEVICE);
	if (dma_mapping_error(dmux->dev, skb_dma->addr)) {
		dev_err(dmux->dev, "RX DMA map failed for skb %d\n", idx);
		skb_dma->addr = 0;
		return false;
	}

	/* Flags = 0 for RX producer pipe (no EOT from APPS side) */
	ret = bam_dmux_pipe_submit(dmux, &dmux->rx_pipe,
				   skb_dma->addr, skb_dma->skb->len, 0);
	if (ret) {
		dma_unmap_single(dmux->dev, skb_dma->addr,
				 skb_dma->skb->len, DMA_FROM_DEVICE);
		skb_dma->addr = 0;
		return false;
	}

	return true;
}

static void bam_dmux_queue_rx(struct bam_dmux *dmux)
{
	int i;

	/*
	 * Downstream sps_transfer_one rings doorbell after EACH descriptor.
	 * Match this exactly: submit + doorbell per descriptor.
	 */
	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		if (dmux->rx_skbs[i].addr)
			continue;
		if (!bam_dmux_submit_rx(dmux, i))
			break;
		bam_dmux_pipe_doorbell(dmux, &dmux->rx_pipe);
	}
}

static void bam_dmux_cmd_data(struct bam_dmux *dmux,
			      struct bam_dmux_skb_dma *skb_dma, u16 sps_size)
{
	struct sk_buff *skb = skb_dma->skb;
	struct bam_dmux_hdr *hdr = (struct bam_dmux_hdr *)skb->data;
	struct net_device *netdev;
	u16 pkt_len;

	if (hdr->ch >= BAM_DMUX_NUM_CH) {
		dev_warn(dmux->dev, "Data for invalid channel %u\n", hdr->ch);
		return;
	}

	netdev = dmux->netdevs[hdr->ch];
	if (!netdev || !netif_running(netdev)) {
		dev_warn(dmux->dev, "Data for inactive channel %u\n", hdr->ch);
		return;
	}

	pkt_len = le16_to_cpu(hdr->len);
	if (pkt_len == 0xffff)
		pkt_len = sps_size - sizeof(*hdr);

	if (pkt_len > BAM_DMUX_MAX_DATA_SIZE) {
		dev_err(dmux->dev, "Data too large: %u > %u\n",
			pkt_len, (u16)BAM_DMUX_MAX_DATA_SIZE);
		return;
	}

	skb_dma->skb = NULL;

	skb_pull(skb, sizeof(*hdr));
	skb_trim(skb, pkt_len);
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
	dev_info(dmux->dev, "remote CMD_OPEN ch=%u\n", hdr->ch);

	if (__test_and_set_bit(hdr->ch, dmux->remote_channels)) {
		dev_warn(dmux->dev, "Channel already open: %u\n", hdr->ch);
		return;
	}

	if (dmux->netdevs[hdr->ch]) {
		netif_device_attach(dmux->netdevs[hdr->ch]);
	} else {
		schedule_work(&dmux->register_netdev_work);
	}

	/*
	 * Downstream: APPS responds to modem CMD_OPEN by sending
	 * CMD_OPEN back. This completes the bidirectional handshake.
	 * (In downstream, this happens in msm_bam_dmux_open() when
	 * rmnet_bam probes onto the platform device. We simplify by
	 * responding immediately.)
	 */
	if (bam_dmux_send_cmd(dmux, BAM_DMUX_CMD_OPEN, hdr->ch))
		dev_err(dmux->dev, "Failed to send CMD_OPEN response ch=%u\n",
			hdr->ch);
	else
		dev_info(dmux->dev, "CMD_OPEN response sent ch=%u\n", hdr->ch);
}

static void bam_dmux_cmd_close(struct bam_dmux *dmux, struct bam_dmux_hdr *hdr)
{
	dev_info(dmux->dev, "remote CMD_CLOSE ch=%u\n", hdr->ch);

	if (!__test_and_clear_bit(hdr->ch, dmux->remote_channels)) {
		dev_err(dmux->dev, "Channel not open: %u\n", hdr->ch);
		return;
	}

	if (dmux->netdevs[hdr->ch])
		netif_device_detach(dmux->netdevs[hdr->ch]);
}

static void bam_dmux_process_rx(struct bam_dmux *dmux,
				struct bam_dmux_skb_dma *skb_dma,
				u16 sps_size)
{
	struct sk_buff *skb = skb_dma->skb;
	struct bam_dmux_hdr *hdr;

	dma_unmap_single(dmux->dev, skb_dma->addr, skb->len, DMA_FROM_DEVICE);
	skb_dma->addr = 0;

	hdr = (struct bam_dmux_hdr *)skb->data;

	if (le16_to_cpu(hdr->magic) != BAM_DMUX_HDR_MAGIC) {
		dev_err(dmux->dev, "Bad magic: %#x (bytes: %*ph)\n",
			le16_to_cpu(hdr->magic),
			min_t(int, 16, skb->len), skb->data);
		return;
	}

	if (hdr->ch >= BAM_DMUX_NUM_CH) {
		dev_dbg(dmux->dev, "Unsupported channel: %u\n", hdr->ch);
		return;
	}

	switch (hdr->cmd) {
	case BAM_DMUX_CMD_DATA:
		bam_dmux_cmd_data(dmux, skb_dma, sps_size);
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

/* ===== TX completion handling ===== */

static void bam_dmux_process_tx_completions(struct bam_dmux *dmux)
{
	struct bam_dmux_skb_dma *skb_dma;
	dma_addr_t addr;
	unsigned long flags;
	u16 size;
	int i;

	while (bam_dmux_pipe_get_completion(dmux, &dmux->tx_pipe,
					    &addr, &size)) {
		spin_lock_irqsave(&dmux->tx_lock, flags);
		for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
			skb_dma = &dmux->tx_skbs[i];
			if (skb_dma->addr == addr && skb_dma->skb) {
				dma_unmap_single(dmux->dev, skb_dma->addr,
						 skb_dma->skb->len,
						 DMA_TO_DEVICE);
				dev_consume_skb_any(skb_dma->skb);
				skb_dma->skb = NULL;
				skb_dma->addr = 0;
				break;
			}
		}
		bam_dmux_tx_wake_queues(dmux);
		spin_unlock_irqrestore(&dmux->tx_lock, flags);
	}
}

/*
 * BAM+pipe hardware init — configures BAM global registers, both pipes.
 * Matches downstream bam_init() which is called from bam_dmux_smsm_cb
 * AFTER the modem asserts A2_POWER_CONTROL (bit 1).
 *
 * Downstream sequence: pipe_init → pipe_set_irq → ACK → queue_rx.
 */
static bool bam_dmux_hw_init(struct bam_dmux *dmux)
{
	/* Step 1: BAM global init (SW_RST + BAM_EN + CNFG) */
	bam_dmux_bam_init(dmux);

	/* Step 2: TX pipe (consumer, MEM->DEV) */
	bam_dmux_pipe_init(dmux, &dmux->tx_pipe, false);

	/* Step 3: RX pipe (producer, DEV->MEM) */
	bam_dmux_pipe_init(dmux, &dmux->rx_pipe, true);

	/*
	 * Step 4: Enable pipe IRQs (downstream pipe_set_irq).
	 * Downstream sets P_IRQ_EN=0x20 (EOT only) AFTER pipe init,
	 * then clears stale P_IRQ_STTS.
	 */
	bam_writel(dmux, BAM_P_IRQ_EN(dmux->tx_pipe.pipe_num),
		   P_TRNSFR_END_EN);
	bam_writel(dmux, BAM_P_IRQ_CLR(dmux->tx_pipe.pipe_num), 0xFF);
	bam_writel(dmux, BAM_P_IRQ_EN(dmux->rx_pipe.pipe_num),
		   P_TRNSFR_END_EN);
	bam_writel(dmux, BAM_P_IRQ_CLR(dmux->rx_pipe.pipe_num), 0xFF);

	/* Step 5: Toggle ACK (downstream: toggle_apps_ack before queue_rx) */
	bam_dmux_pc_ack(dmux);

	/* Step 6: Submit RX buffers (per-descriptor doorbell) */
	bam_dmux_queue_rx(dmux);

	dmux->bam_initialized = true;

	/* Diagnostic: dump descriptor FIFO content and BAM error status */
	{
		int i;
		u32 p = dmux->rx_pipe.pipe_num;

		dev_info(dmux->dev,
			 "hw_init diag: RX P_IRQ_STTS=0x%x P_EVNT=0x%x "
			 "P_SW_OFSTS=0x%x P_HALT=0x%x BAM_IRQ_STTS=0x%x\n",
			 bam_readl(dmux, BAM_P_IRQ_STTS(p)),
			 bam_readl(dmux, BAM_P_EVNT_REG(p)),
			 bam_readl(dmux, BAM_P_SW_OFSTS(p)),
			 bam_readl(dmux, BAM_P_HALT(p)),
			 bam_readl(dmux, BAM_IRQ_STTS));

		/* Dump first 4 RX descriptors from FIFO */
		for (i = 0; i < 4 && i < BAM_DMUX_NUM_SKB; i++) {
			struct bam_desc_hw *d = &dmux->rx_pipe.desc_fifo[i];

			dev_info(dmux->dev,
				 "  rx_desc[%d]: addr=0x%08x size=%u flags=0x%04x "
				 "skb_dma=0x%pad\n",
				 i, le32_to_cpu(d->addr), le16_to_cpu(d->size),
				 le16_to_cpu(d->flags), &dmux->rx_skbs[i].addr);
		}

		p = dmux->tx_pipe.pipe_num;
		dev_info(dmux->dev,
			 "hw_init diag: TX P_IRQ_STTS=0x%x P_EVNT=0x%x "
			 "P_SW_OFSTS=0x%x\n",
			 bam_readl(dmux, BAM_P_IRQ_STTS(p)),
			 bam_readl(dmux, BAM_P_EVNT_REG(p)),
			 bam_readl(dmux, BAM_P_SW_OFSTS(p)));
	}

	dev_info(dmux->dev, "hw_init complete: pipes + RX buffers ready\n");
	return true;
}

static void bam_dmux_hw_deinit(struct bam_dmux *dmux)
{
	int i;

	dmux->bam_initialized = false;

	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		struct bam_dmux_skb_dma *skb_dma = &dmux->rx_skbs[i];

		if (skb_dma->addr) {
			dma_unmap_single(dmux->dev, skb_dma->addr,
					 skb_dma->skb->len, DMA_FROM_DEVICE);
			skb_dma->addr = 0;
		}
		if (skb_dma->skb) {
			dev_kfree_skb_any(skb_dma->skb);
			skb_dma->skb = NULL;
		}
	}

	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		struct bam_dmux_skb_dma *skb_dma = &dmux->tx_skbs[i];

		if (skb_dma->addr) {
			dma_unmap_single(dmux->dev, skb_dma->addr,
					 skb_dma->skb->len, DMA_TO_DEVICE);
			skb_dma->addr = 0;
		}
		if (skb_dma->skb) {
			dev_kfree_skb_any(skb_dma->skb);
			skb_dma->skb = NULL;
		}
	}

	dmux->tx_pipe.head = 0;
	dmux->tx_pipe.tail = 0;
	dmux->rx_pipe.head = 0;
	dmux->rx_pipe.tail = 0;
	dmux->tx_next_skb = 0;
}

/* ===== Poll timer ===== */

static void bam_dmux_poll_timer_fn(struct timer_list *t)
{
	struct bam_dmux *dmux = container_of(t, struct bam_dmux, poll_timer);
	static unsigned int poll_count;
	dma_addr_t addr;
	u16 size;
	bool got_rx = false;
	int i;

	if (!dmux->bam_initialized || !dmux->pc_state) {
		mod_timer(&dmux->poll_timer, jiffies + msecs_to_jiffies(20));
		return;
	}

	/* Process TX completions */
	bam_dmux_process_tx_completions(dmux);

	/* Process RX completions */
	while (bam_dmux_pipe_get_completion(dmux, &dmux->rx_pipe,
					    &addr, &size)) {
		for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
			if (dmux->rx_skbs[i].addr == addr) {
				bam_dmux_process_rx(dmux, &dmux->rx_skbs[i],
						    size);
				got_rx = true;
				break;
			}
		}
		if (i == BAM_DMUX_NUM_SKB)
			dev_err(dmux->dev,
				"RX completion addr %pad not found!\n", &addr);
	}

	if (got_rx)
		bam_dmux_queue_rx(dmux);

	if (poll_count < 20 || !(poll_count % 5000)) {
		dev_info(dmux->dev,
			 "poll[%u]: rx P_OFSTS=0x%x P_EVNT=0x%x "
			 "tx P_OFSTS=0x%x P_EVNT=0x%x "
			 "P5_CTRL=0x%x P4_CTRL=0x%x BAM_CTRL=0x%x "
			 "rx_IRQ=0x%x tx_IRQ=0x%x "
			 "rx_DESC=0x%x rx_SIZES=0x%x\n",
			 poll_count,
			 bam_readl(dmux, BAM_P_SW_OFSTS(BAM_DMUX_RX_PIPE)),
			 bam_readl(dmux, BAM_P_EVNT_REG(BAM_DMUX_RX_PIPE)),
			 bam_readl(dmux, BAM_P_SW_OFSTS(BAM_DMUX_TX_PIPE)),
			 bam_readl(dmux, BAM_P_EVNT_REG(BAM_DMUX_TX_PIPE)),
			 bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_RX_PIPE)),
			 bam_readl(dmux, BAM_P_CTRL(BAM_DMUX_TX_PIPE)),
			 bam_readl(dmux, BAM_CTRL),
			 bam_readl(dmux, BAM_P_IRQ_STTS(BAM_DMUX_RX_PIPE)),
			 bam_readl(dmux, BAM_P_IRQ_STTS(BAM_DMUX_TX_PIPE)),
			 bam_readl(dmux, BAM_P_DESC_FIFO_ADDR(BAM_DMUX_RX_PIPE)),
			 bam_readl(dmux, BAM_P_FIFO_SIZES(BAM_DMUX_RX_PIPE)));

		/* Extended trust/security + pipe context diagnostics */
		dev_info(dmux->dev,
			 "poll[%u]: TRUST=0x%x P_TR[4]=0x%x P_TR[5]=0x%x "
			 "SRCS=0x%x PIPE_ATTR=0x%x "
			 "P5_RETR=0x%x P5_PSM1=0x%x P5_PSM2=0x%x "
			 "P5_PRDCR=0x%x P5_CNSMR=0x%x\n",
			 poll_count,
			 bam_readl(dmux, BAM_TRUST_REG),
			 bam_readl(dmux, BAM_P_TRUST_REG(BAM_DMUX_TX_PIPE)),
			 bam_readl(dmux, BAM_P_TRUST_REG(BAM_DMUX_RX_PIPE)),
			 bam_readl(dmux, BAM_IRQ_SRCS_EE),
			 bam_readl(dmux, BAM_PIPE_ATTR_EE),
			 bam_readl(dmux, BAM_P_RETR_CNTXT(BAM_DMUX_RX_PIPE)),
			 bam_readl(dmux, BAM_P_AU_PSM_CNTXT_1(BAM_DMUX_RX_PIPE)),
			 bam_readl(dmux, BAM_P_PSM_CNTXT_2(BAM_DMUX_RX_PIPE)),
			 bam_readl(dmux, BAM_P_PRDCR_SDBND(BAM_DMUX_RX_PIPE)),
			 bam_readl(dmux, BAM_P_CNSMR_SDBND(BAM_DMUX_RX_PIPE)));
	}

	/*
	 * Clear P_IRQ_STTS for both pipes every poll cycle.
	 * Downstream SPS ISR clears these; we never did until now.
	 * This might unblock the BAM if it stalls on pending IRQs.
	 */
	bam_writel(dmux, BAM_P_IRQ_CLR(BAM_DMUX_RX_PIPE), 0xFF);
	bam_writel(dmux, BAM_P_IRQ_CLR(BAM_DMUX_TX_PIPE), 0xFF);
	/* Also clear global BAM IRQ status */
	bam_writel(dmux, BAM_IRQ_CLR, 0xFF);
	poll_count++;

	mod_timer(&dmux->poll_timer, jiffies + msecs_to_jiffies(1));
}

/* ===== BAM hardware ISR (downstream: SPS handles GIC SPI 29) ===== */

/*
 * The BAM asserts GIC SPI 29 when pipe interrupts are pending.
 * Downstream, the SPS framework registers this ISR, reads
 * BAM_IRQ_SRCS_EE, and clears pipe interrupt status.
 * Without handling this interrupt, the BAM may misbehave.
 */
static irqreturn_t bam_dmux_bam_isr(int irq, void *data)
{
	struct bam_dmux *dmux = data;
	u32 srcs, stts;

	srcs = bam_readl(dmux, BAM_IRQ_SRCS_EE);
	if (!srcs)
		return IRQ_NONE;

	/* Clear global BAM interrupt status */
	if (srcs & BIT(31)) {
		stts = bam_readl(dmux, BAM_IRQ_STTS);
		bam_writel(dmux, BAM_IRQ_CLR, stts);
	}

	/* Clear pipe 4 (TX) interrupt status */
	if (srcs & BIT(BAM_DMUX_TX_PIPE)) {
		stts = bam_readl(dmux, BAM_P_IRQ_STTS(BAM_DMUX_TX_PIPE));
		bam_writel(dmux, BAM_P_IRQ_CLR(BAM_DMUX_TX_PIPE), stts);
	}

	/* Clear pipe 5 (RX) interrupt status */
	if (srcs & BIT(BAM_DMUX_RX_PIPE)) {
		stts = bam_readl(dmux, BAM_P_IRQ_STTS(BAM_DMUX_RX_PIPE));
		bam_writel(dmux, BAM_P_IRQ_CLR(BAM_DMUX_RX_PIPE), stts);
	}

	return IRQ_HANDLED;
}

/* ===== IRQ handlers ===== */

static irqreturn_t bam_dmux_pc_irq(int irq, void *data)
{
	struct bam_dmux *dmux = data;
	bool new_state;

	irq_get_irqchip_state(dmux->pc_irq, IRQCHIP_STATE_LINE_LEVEL,
			      &new_state);

	dev_info(dmux->dev, "pc_irq: modem_bit1=%d pc_state=%d\n",
		 new_state, dmux->pc_state);

	cancel_delayed_work(&dmux->boot_work);

	if (new_state && !dmux->pc_state) {
		/*
		 * Modem asserted A2_POWER_CONTROL.
		 * Downstream bam_dmux_smsm_cb does bam_init() HERE
		 * (after modem bit 1).  Match exactly.
		 */
		if (!bam_dmux_hw_init(dmux)) {
			dev_err(dmux->dev, "pc_irq: hw_init failed\n");
			bam_dmux_hw_deinit(dmux);
			goto out;
		}

		dmux->pc_state = new_state;

		/*
		 * Keep APPS bit 1 SET. On OCPU firmware, clearing APPS
		 * bit 1 after hw_init causes the modem to immediately
		 * power down A2. The modem requires APPS bit 1 to stay
		 * asserted to keep the A2 data path alive.
		 *
		 * In downstream, APPS bit 1 is the "A2_POWER_CONTROL"
		 * vote — it means "I need A2 uplink". Clearing it tells
		 * the modem "I'm done, you can power collapse A2."
		 */

		/* Start polling */
		mod_timer(&dmux->poll_timer, jiffies + msecs_to_jiffies(1));

		/*
		 * Don't send CMD_OPEN — match downstream flow:
		 * Modem sends CMD_OPEN first after seeing our ACK toggle.
		 * We process it and respond with CMD_OPEN back.
		 */
		dev_info(dmux->dev,
			 "pc_irq: waiting for modem CMD_OPEN on RX pipe\n");

	} else if (!new_state && dmux->pc_state) {
		dev_info(dmux->dev, "pc_irq: modem powering down\n");
		timer_delete_sync(&dmux->poll_timer);
		bam_dmux_hw_deinit(dmux);
		bam_dmux_pc_ack(dmux);
		dmux->pc_state = new_state;
	}

out:
	return IRQ_HANDLED;
}

static irqreturn_t bam_dmux_pc_ack_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

/* ===== Boot sequence ===== */

static void bam_dmux_boot_work_fn(struct work_struct *work)
{
	struct bam_dmux *dmux = container_of(work, struct bam_dmux,
					     boot_work.work);

	if (dmux->pc_state)
		return;

	/*
	 * Downstream: modem auto-sets bit 1 without APPS requesting it.
	 * On OCPU firmware, the modem waits for APPS to request A2 power
	 * by setting APPS bit 1. We set it here; BAM init happens in
	 * pc_irq after modem responds. We clear APPS bit 1 AFTER hw_init
	 * to match the downstream state (APPS bit 1 NOT set during initial
	 * link setup — it's only used for uplink wakeup).
	 */
	dev_info(dmux->dev,
		 "boot_work: set APPS bit 1 (BAM init deferred to pc_irq)\n");

	bam_dmux_pc_vote(dmux, true);

	dev_info(dmux->dev,
		 "boot_work: APPS bit 1 set, waiting for modem pc_irq\n");
}

static irqreturn_t bam_dmux_remote_ready_irq(int irq, void *data)
{
	struct bam_dmux *dmux = data;

	dev_info(dmux->dev, "remote ready (SMDINIT), scheduling boot_work\n");
	schedule_delayed_work(&dmux->boot_work, msecs_to_jiffies(1000));

	return IRQ_HANDLED;
}

/* ===== Probe / Remove ===== */

static int bam_dmux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *bam_node;
	struct bam_dmux *dmux;
	struct resource res;
	int ret, pc_ack_irq, remote_ready_irq;
	unsigned int bit;

	dmux = devm_kzalloc(dev, sizeof(*dmux), GFP_KERNEL);
	if (!dmux)
		return -ENOMEM;

	dmux->dev = dev;
	platform_set_drvdata(pdev, dmux);

	/*
	 * Get BAM register base. First try our own "reg" property,
	 * then fall back to parsing the DMA controller phandle.
	 */
	if (of_property_present(dev->of_node, "reg")) {
		dmux->bam_base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(dmux->bam_base))
			return dev_err_probe(dev, PTR_ERR(dmux->bam_base),
					     "Failed to map BAM registers\n");
	} else {
		bam_node = of_parse_phandle(dev->of_node, "dmas", 0);
		if (!bam_node)
			return dev_err_probe(dev, -ENODEV,
					     "No BAM address source\n");

		ret = of_address_to_resource(bam_node, 0, &res);
		if (ret) {
			of_node_put(bam_node);
			return dev_err_probe(dev, ret,
					     "Failed to get BAM resource\n");
		}

		/* Get BAM hardware IRQ (GIC SPI 29) from DMA phandle */
		dmux->bam_irq = irq_of_parse_and_map(bam_node, 0);

		of_node_put(bam_node);

		dmux->bam_base = devm_ioremap(dev, res.start,
					      resource_size(&res));
		if (!dmux->bam_base)
			return dev_err_probe(dev, -ENOMEM,
					     "Failed to ioremap BAM\n");

		dev_info(dev, "BAM registers from DMA phandle: %pR\n", &res);
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* SMSM state handles */
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

	/* IRQs */
	dmux->pc_irq = platform_get_irq_byname(pdev, "pc");
	if (dmux->pc_irq < 0)
		return dmux->pc_irq;

	pc_ack_irq = platform_get_irq_byname(pdev, "pc-ack");
	if (pc_ack_irq < 0)
		return pc_ack_irq;

	/* Allocate descriptor FIFOs */
	dmux->tx_pipe.pipe_num = BAM_DMUX_TX_PIPE;
	dmux->tx_pipe.desc_fifo = dma_alloc_coherent(dev, BAM_DESC_FIFO_SIZE,
						      &dmux->tx_pipe.desc_fifo_phys,
						      GFP_KERNEL);
	if (!dmux->tx_pipe.desc_fifo)
		return -ENOMEM;

	dmux->rx_pipe.pipe_num = BAM_DMUX_RX_PIPE;
	dmux->rx_pipe.desc_fifo = dma_alloc_coherent(dev, BAM_DESC_FIFO_SIZE,
						      &dmux->rx_pipe.desc_fifo_phys,
						      GFP_KERNEL);
	if (!dmux->rx_pipe.desc_fifo) {
		ret = -ENOMEM;
		goto err_free_tx_fifo;
	}

	memset(dmux->tx_pipe.desc_fifo, 0, BAM_DESC_FIFO_SIZE);
	memset(dmux->rx_pipe.desc_fifo, 0, BAM_DESC_FIFO_SIZE);

	spin_lock_init(&dmux->tx_lock);
	INIT_WORK(&dmux->register_netdev_work, bam_dmux_register_netdev_work);
	INIT_DELAYED_WORK(&dmux->boot_work, bam_dmux_boot_work_fn);
	timer_setup(&dmux->poll_timer, bam_dmux_poll_timer_fn, 0);

	ret = devm_request_threaded_irq(dev, pc_ack_irq, NULL,
					bam_dmux_pc_ack_irq,
					IRQF_ONESHOT, NULL, dmux);
	if (ret)
		goto err_free_rx_fifo;

	ret = devm_request_threaded_irq(dev, dmux->pc_irq, NULL,
					bam_dmux_pc_irq,
					IRQF_ONESHOT, NULL, dmux);
	if (ret)
		goto err_free_rx_fifo;

	ret = irq_get_irqchip_state(dmux->pc_irq, IRQCHIP_STATE_LINE_LEVEL,
				    &dmux->pc_state);
	if (ret)
		goto err_free_rx_fifo;

	remote_ready_irq = platform_get_irq_byname(pdev, "remote-ready");
	if (remote_ready_irq > 0) {
		ret = devm_request_threaded_irq(dev, remote_ready_irq, NULL,
						bam_dmux_remote_ready_irq,
						IRQF_ONESHOT, NULL, dmux);
		if (ret)
			dev_warn(dev,
				 "failed to request remote-ready IRQ: %d\n",
				 ret);
	}

	/*
	 * Request BAM hardware interrupt (GIC SPI 29).
	 * Downstream SPS framework handles this to clear P_IRQ_STTS.
	 * Without it, pending BAM interrupts are never acknowledged.
	 */
	if (dmux->bam_irq > 0) {
		ret = devm_request_irq(dev, dmux->bam_irq, bam_dmux_bam_isr,
				       IRQF_TRIGGER_HIGH, "bam_dmux_bam", dmux);
		if (ret)
			dev_warn(dev, "failed to request BAM IRQ %d: %d\n",
				 dmux->bam_irq, ret);
		else
			dev_info(dev, "BAM IRQ %d registered\n", dmux->bam_irq);
	}

	dev_info(dev, "probe complete: pc_state=%d bam_base=%p bam_irq=%d\n",
		 dmux->pc_state, dmux->bam_base, dmux->bam_irq);

	if (dmux->pc_state) {
		dev_info(dev, "modem already powered, initializing\n");
		bam_dmux_hw_init(dmux);
		mod_timer(&dmux->poll_timer, jiffies + msecs_to_jiffies(1));
	}

	return 0;

err_free_rx_fifo:
	dma_free_coherent(dev, BAM_DESC_FIFO_SIZE,
			   dmux->rx_pipe.desc_fifo,
			   dmux->rx_pipe.desc_fifo_phys);
err_free_tx_fifo:
	dma_free_coherent(dev, BAM_DESC_FIFO_SIZE,
			   dmux->tx_pipe.desc_fifo,
			   dmux->tx_pipe.desc_fifo_phys);
	return ret;
}

static void bam_dmux_remove(struct platform_device *pdev)
{
	struct bam_dmux *dmux = platform_get_drvdata(pdev);
	LIST_HEAD(list);
	int i;

	cancel_delayed_work_sync(&dmux->boot_work);
	timer_delete_sync(&dmux->poll_timer);

	cancel_work_sync(&dmux->register_netdev_work);
	rtnl_lock();
	for (i = 0; i < BAM_DMUX_NUM_CH; ++i)
		if (dmux->netdevs[i])
			unregister_netdevice_queue(dmux->netdevs[i], &list);
	unregister_netdevice_many(&list);
	rtnl_unlock();

	disable_irq(dmux->pc_irq);
	bam_dmux_hw_deinit(dmux);

	dma_free_coherent(dmux->dev, BAM_DESC_FIFO_SIZE,
			   dmux->rx_pipe.desc_fifo,
			   dmux->rx_pipe.desc_fifo_phys);
	dma_free_coherent(dmux->dev, BAM_DESC_FIFO_SIZE,
			   dmux->tx_pipe.desc_fifo,
			   dmux->tx_pipe.desc_fifo_phys);
}

static const struct of_device_id bam_dmux_of_match[] = {
	{ .compatible = "qcom,bam-dmux" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bam_dmux_of_match);

static struct platform_driver bam_dmux_driver = {
	.probe = bam_dmux_probe,
	.remove = bam_dmux_remove,
	.driver = {
		.name = "bam-dmux",
		.of_match_table = bam_dmux_of_match,
	},
};
module_platform_driver(bam_dmux_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm BAM-DMUX WWAN Network Driver (direct BAM access)");
MODULE_AUTHOR("Stephan Gerhold <stephan@gerhold.net>");
