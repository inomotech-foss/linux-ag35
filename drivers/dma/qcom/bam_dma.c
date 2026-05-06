// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 */
/*
 * QCOM BAM DMA engine driver
 *
 * QCOM BAM DMA blocks are distributed amongst a number of the on-chip
 * peripherals on the MSM 8x74.  The configuration of the channels are dependent
 * on the way they are hard wired to that specific peripheral.  The peripheral
 * device tree entries specify the configuration of each channel.
 *
 * The DMA controller requires the use of external memory for storage of the
 * hardware descriptors for each channel.  The descriptor FIFO is accessed as a
 * circular buffer and operations are managed according to the offset within the
 * FIFO.  After pipe/channel reset, all of the pipe registers and internal state
 * are back to defaults.
 *
 * During DMA operations, we write descriptors to the FIFO, being careful to
 * handle wrapping and then write the last FIFO offset to that channel's
 * P_EVNT_REG register to kick off the transaction.  The P_SW_OFSTS register
 * indicates the current FIFO offset that is being processed, so there is some
 * indication of where the hardware is currently working.
 */

#include <linux/circ_buf.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/hrtimer.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "../dmaengine.h"
#include "../virt-dma.h"

struct bam_desc_hw {
	__le32 addr;		/* Buffer physical address */
	__le16 size;		/* Buffer size in bytes */
	__le16 flags;
};

#define BAM_DMA_AUTOSUSPEND_DELAY 100

#define DESC_FLAG_INT BIT(15)
#define DESC_FLAG_EOT BIT(14)
#define DESC_FLAG_EOB BIT(13)
#define DESC_FLAG_NWD BIT(12)
#define DESC_FLAG_CMD BIT(11)
#define DESC_FLAG_LOCK BIT(10)
#define DESC_FLAG_UNLOCK BIT(9)

struct bam_async_desc {
	struct virt_dma_desc vd;

	u32 num_desc;
	u32 xfer_len;

	/* transaction flags, EOT|EOB|NWD */
	u16 flags;

	struct bam_desc_hw *curr_desc;

	/* list node for the desc in the bam_chan list of descriptors */
	struct list_head desc_node;
	enum dma_transfer_direction dir;
	size_t length;
	struct bam_desc_hw desc[] __counted_by(num_desc);
};

enum bam_reg {
	BAM_CTRL,
	BAM_REVISION,
	BAM_NUM_PIPES,
	BAM_DESC_CNT_TRSHLD,
	BAM_IRQ_SRCS,
	BAM_IRQ_SRCS_MSK,
	BAM_IRQ_SRCS_UNMASKED,
	BAM_IRQ_STTS,
	BAM_IRQ_CLR,
	BAM_IRQ_EN,
	BAM_CNFG_BITS,
	BAM_IRQ_SRCS_EE,
	BAM_IRQ_SRCS_MSK_EE,
	BAM_P_CTRL,
	BAM_P_RST,
	BAM_P_HALT,
	BAM_P_IRQ_STTS,
	BAM_P_IRQ_CLR,
	BAM_P_IRQ_EN,
	BAM_P_EVNT_DEST_ADDR,
	BAM_P_EVNT_REG,
	BAM_P_SW_OFSTS,
	BAM_P_DATA_FIFO_ADDR,
	BAM_P_DESC_FIFO_ADDR,
	BAM_P_EVNT_GEN_TRSHLD,
	BAM_P_FIFO_SIZES,
};

struct reg_offset_data {
	u32 base_offset;
	unsigned int pipe_mult, evnt_mult, ee_mult;
};

static const struct reg_offset_data bam_v1_3_reg_info[] = {
	[BAM_CTRL]		= { 0x0F80, 0x00, 0x00, 0x00 },
	[BAM_REVISION]		= { 0x0F84, 0x00, 0x00, 0x00 },
	[BAM_NUM_PIPES]		= { 0x0FBC, 0x00, 0x00, 0x00 },
	[BAM_DESC_CNT_TRSHLD]	= { 0x0F88, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS]		= { 0x0F8C, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS_MSK]	= { 0x0F90, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS_UNMASKED]	= { 0x0FB0, 0x00, 0x00, 0x00 },
	[BAM_IRQ_STTS]		= { 0x0F94, 0x00, 0x00, 0x00 },
	[BAM_IRQ_CLR]		= { 0x0F98, 0x00, 0x00, 0x00 },
	[BAM_IRQ_EN]		= { 0x0F9C, 0x00, 0x00, 0x00 },
	[BAM_CNFG_BITS]		= { 0x0FFC, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS_EE]	= { 0x1800, 0x00, 0x00, 0x80 },
	[BAM_IRQ_SRCS_MSK_EE]	= { 0x1804, 0x00, 0x00, 0x80 },
	[BAM_P_CTRL]		= { 0x0000, 0x80, 0x00, 0x00 },
	[BAM_P_RST]		= { 0x0004, 0x80, 0x00, 0x00 },
	[BAM_P_HALT]		= { 0x0008, 0x80, 0x00, 0x00 },
	[BAM_P_IRQ_STTS]	= { 0x0010, 0x80, 0x00, 0x00 },
	[BAM_P_IRQ_CLR]		= { 0x0014, 0x80, 0x00, 0x00 },
	[BAM_P_IRQ_EN]		= { 0x0018, 0x80, 0x00, 0x00 },
	[BAM_P_EVNT_DEST_ADDR]	= { 0x102C, 0x00, 0x40, 0x00 },
	[BAM_P_EVNT_REG]	= { 0x1018, 0x00, 0x40, 0x00 },
	[BAM_P_SW_OFSTS]	= { 0x1000, 0x00, 0x40, 0x00 },
	[BAM_P_DATA_FIFO_ADDR]	= { 0x1024, 0x00, 0x40, 0x00 },
	[BAM_P_DESC_FIFO_ADDR]	= { 0x101C, 0x00, 0x40, 0x00 },
	[BAM_P_EVNT_GEN_TRSHLD]	= { 0x1028, 0x00, 0x40, 0x00 },
	[BAM_P_FIFO_SIZES]	= { 0x1020, 0x00, 0x40, 0x00 },
};

static const struct reg_offset_data bam_v1_4_reg_info[] = {
	[BAM_CTRL]		= { 0x0000, 0x00, 0x00, 0x00 },
	[BAM_REVISION]		= { 0x0004, 0x00, 0x00, 0x00 },
	[BAM_NUM_PIPES]		= { 0x003C, 0x00, 0x00, 0x00 },
	[BAM_DESC_CNT_TRSHLD]	= { 0x0008, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS]		= { 0x000C, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS_MSK]	= { 0x0010, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS_UNMASKED]	= { 0x0030, 0x00, 0x00, 0x00 },
	[BAM_IRQ_STTS]		= { 0x0014, 0x00, 0x00, 0x00 },
	[BAM_IRQ_CLR]		= { 0x0018, 0x00, 0x00, 0x00 },
	[BAM_IRQ_EN]		= { 0x001C, 0x00, 0x00, 0x00 },
	[BAM_CNFG_BITS]		= { 0x007C, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS_EE]	= { 0x0800, 0x00, 0x00, 0x80 },
	[BAM_IRQ_SRCS_MSK_EE]	= { 0x0804, 0x00, 0x00, 0x80 },
	[BAM_P_CTRL]		= { 0x1000, 0x1000, 0x00, 0x00 },
	[BAM_P_RST]		= { 0x1004, 0x1000, 0x00, 0x00 },
	[BAM_P_HALT]		= { 0x1008, 0x1000, 0x00, 0x00 },
	[BAM_P_IRQ_STTS]	= { 0x1010, 0x1000, 0x00, 0x00 },
	[BAM_P_IRQ_CLR]		= { 0x1014, 0x1000, 0x00, 0x00 },
	[BAM_P_IRQ_EN]		= { 0x1018, 0x1000, 0x00, 0x00 },
	[BAM_P_EVNT_DEST_ADDR]	= { 0x182C, 0x00, 0x1000, 0x00 },
	[BAM_P_EVNT_REG]	= { 0x1818, 0x00, 0x1000, 0x00 },
	[BAM_P_SW_OFSTS]	= { 0x1800, 0x00, 0x1000, 0x00 },
	[BAM_P_DATA_FIFO_ADDR]	= { 0x1824, 0x00, 0x1000, 0x00 },
	[BAM_P_DESC_FIFO_ADDR]	= { 0x181C, 0x00, 0x1000, 0x00 },
	[BAM_P_EVNT_GEN_TRSHLD]	= { 0x1828, 0x00, 0x1000, 0x00 },
	[BAM_P_FIFO_SIZES]	= { 0x1820, 0x00, 0x1000, 0x00 },
};

static const struct reg_offset_data bam_v1_7_reg_info[] = {
	[BAM_CTRL]		= { 0x00000, 0x00, 0x00, 0x00 },
	[BAM_REVISION]		= { 0x01000, 0x00, 0x00, 0x00 },
	[BAM_NUM_PIPES]		= { 0x01008, 0x00, 0x00, 0x00 },
	[BAM_DESC_CNT_TRSHLD]	= { 0x00008, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS]		= { 0x03010, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS_MSK]	= { 0x03014, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS_UNMASKED]	= { 0x03018, 0x00, 0x00, 0x00 },
	[BAM_IRQ_STTS]		= { 0x00014, 0x00, 0x00, 0x00 },
	[BAM_IRQ_CLR]		= { 0x00018, 0x00, 0x00, 0x00 },
	[BAM_IRQ_EN]		= { 0x0001C, 0x00, 0x00, 0x00 },
	[BAM_CNFG_BITS]		= { 0x0007C, 0x00, 0x00, 0x00 },
	[BAM_IRQ_SRCS_EE]	= { 0x03000, 0x00, 0x00, 0x1000 },
	[BAM_IRQ_SRCS_MSK_EE]	= { 0x03004, 0x00, 0x00, 0x1000 },
	[BAM_P_CTRL]		= { 0x13000, 0x1000, 0x00, 0x00 },
	[BAM_P_RST]		= { 0x13004, 0x1000, 0x00, 0x00 },
	[BAM_P_HALT]		= { 0x13008, 0x1000, 0x00, 0x00 },
	[BAM_P_IRQ_STTS]	= { 0x13010, 0x1000, 0x00, 0x00 },
	[BAM_P_IRQ_CLR]		= { 0x13014, 0x1000, 0x00, 0x00 },
	[BAM_P_IRQ_EN]		= { 0x13018, 0x1000, 0x00, 0x00 },
	[BAM_P_EVNT_DEST_ADDR]	= { 0x1382C, 0x00, 0x1000, 0x00 },
	[BAM_P_EVNT_REG]	= { 0x13818, 0x00, 0x1000, 0x00 },
	[BAM_P_SW_OFSTS]	= { 0x13800, 0x00, 0x1000, 0x00 },
	[BAM_P_DATA_FIFO_ADDR]	= { 0x13824, 0x00, 0x1000, 0x00 },
	[BAM_P_DESC_FIFO_ADDR]	= { 0x1381C, 0x00, 0x1000, 0x00 },
	[BAM_P_EVNT_GEN_TRSHLD]	= { 0x13828, 0x00, 0x1000, 0x00 },
	[BAM_P_FIFO_SIZES]	= { 0x13820, 0x00, 0x1000, 0x00 },
};

/* BAM CTRL */
#define BAM_SW_RST			BIT(0)
#define BAM_EN				BIT(1)
#define BAM_EN_ACCUM			BIT(4)
#define BAM_TESTBUS_SEL_SHIFT		5
#define BAM_TESTBUS_SEL_MASK		0x3F
#define BAM_DESC_CACHE_SEL_SHIFT	13
#define BAM_DESC_CACHE_SEL_MASK		0x3
#define BAM_CACHED_DESC_STORE		BIT(15)
#define IBC_DISABLE			BIT(16)

/* BAM REVISION */
#define REVISION_SHIFT		0
#define REVISION_MASK		0xFF
#define NUM_EES_SHIFT		8
#define NUM_EES_MASK		0xF
#define CE_BUFFER_SIZE		BIT(13)
#define AXI_ACTIVE		BIT(14)
#define USE_VMIDMT		BIT(15)
#define SECURED			BIT(16)
#define BAM_HAS_NO_BYPASS	BIT(17)
#define HIGH_FREQUENCY_BAM	BIT(18)
#define INACTIV_TMRS_EXST	BIT(19)
#define NUM_INACTIV_TMRS	BIT(20)
#define DESC_CACHE_DEPTH_SHIFT	21
#define DESC_CACHE_DEPTH_1	(0 << DESC_CACHE_DEPTH_SHIFT)
#define DESC_CACHE_DEPTH_2	(1 << DESC_CACHE_DEPTH_SHIFT)
#define DESC_CACHE_DEPTH_3	(2 << DESC_CACHE_DEPTH_SHIFT)
#define DESC_CACHE_DEPTH_4	(3 << DESC_CACHE_DEPTH_SHIFT)
#define CMD_DESC_EN		BIT(23)
#define INACTIV_TMR_BASE_SHIFT	24
#define INACTIV_TMR_BASE_MASK	0xFF

/* BAM NUM PIPES */
#define BAM_NUM_PIPES_SHIFT		0
#define BAM_NUM_PIPES_MASK		0xFF
#define PERIPH_NON_PIPE_GRP_SHIFT	16
#define PERIPH_NON_PIP_GRP_MASK		0xFF
#define BAM_NON_PIPE_GRP_SHIFT		24
#define BAM_NON_PIPE_GRP_MASK		0xFF

/* BAM CNFG BITS */
#define BAM_PIPE_CNFG		BIT(2)
#define BAM_FULL_PIPE		BIT(11)
#define BAM_NO_EXT_P_RST	BIT(12)
#define BAM_IBC_DISABLE		BIT(13)
#define BAM_SB_CLK_REQ		BIT(14)
#define BAM_PSM_CSW_REQ		BIT(15)
#define BAM_PSM_P_RES		BIT(16)
#define BAM_AU_P_RES		BIT(17)
#define BAM_SI_P_RES		BIT(18)
#define BAM_WB_P_RES		BIT(19)
#define BAM_WB_BLK_CSW		BIT(20)
#define BAM_WB_CSW_ACK_IDL	BIT(21)
#define BAM_WB_RETR_SVPNT	BIT(22)
#define BAM_WB_DSC_AVL_P_RST	BIT(23)
#define BAM_REG_P_EN		BIT(24)
#define BAM_PSM_P_HD_DATA	BIT(25)
#define BAM_AU_ACCUMED		BIT(26)
#define BAM_CMD_ENABLE		BIT(27)

#define BAM_CNFG_BITS_DEFAULT	(BAM_PIPE_CNFG |	\
				 BAM_NO_EXT_P_RST |	\
				 BAM_IBC_DISABLE |	\
				 BAM_SB_CLK_REQ |	\
				 BAM_PSM_CSW_REQ |	\
				 BAM_PSM_P_RES |	\
				 BAM_AU_P_RES |		\
				 BAM_SI_P_RES |		\
				 BAM_WB_P_RES |		\
				 BAM_WB_BLK_CSW |	\
				 BAM_WB_CSW_ACK_IDL |	\
				 BAM_WB_RETR_SVPNT |	\
				 BAM_WB_DSC_AVL_P_RST |	\
				 BAM_REG_P_EN |		\
				 BAM_PSM_P_HD_DATA |	\
				 BAM_AU_ACCUMED |	\
				 BAM_CMD_ENABLE)

/* PIPE CTRL */
#define P_EN			BIT(1)
#define P_DIRECTION		BIT(3)
#define P_SYS_STRM		BIT(4)
#define P_SYS_MODE		BIT(5)
#define P_AUTO_EOB		BIT(6)
#define P_AUTO_EOB_SEL_SHIFT	7
#define P_AUTO_EOB_SEL_512	(0 << P_AUTO_EOB_SEL_SHIFT)
#define P_AUTO_EOB_SEL_256	(1 << P_AUTO_EOB_SEL_SHIFT)
#define P_AUTO_EOB_SEL_128	(2 << P_AUTO_EOB_SEL_SHIFT)
#define P_AUTO_EOB_SEL_64	(3 << P_AUTO_EOB_SEL_SHIFT)
#define P_PREFETCH_LIMIT_SHIFT	9
#define P_PREFETCH_LIMIT_32	(0 << P_PREFETCH_LIMIT_SHIFT)
#define P_PREFETCH_LIMIT_16	(1 << P_PREFETCH_LIMIT_SHIFT)
#define P_PREFETCH_LIMIT_4	(2 << P_PREFETCH_LIMIT_SHIFT)
#define P_WRITE_NWD		BIT(11)
#define P_LOCK_GROUP_SHIFT	16
#define P_LOCK_GROUP_MASK	0x1F

/* BAM_DESC_CNT_TRSHLD */
#define CNT_TRSHLD		0xffff
#define DEFAULT_CNT_THRSHLD	0x4

/* BAM_IRQ_SRCS */
#define BAM_IRQ			BIT(31)
#define P_IRQ			0x7fffffff

/* BAM_IRQ_SRCS_MSK */
#define BAM_IRQ_MSK		BAM_IRQ
#define P_IRQ_MSK		P_IRQ

/* BAM_IRQ_STTS */
#define BAM_TIMER_IRQ		BIT(4)
#define BAM_EMPTY_IRQ		BIT(3)
#define BAM_ERROR_IRQ		BIT(2)
#define BAM_HRESP_ERR_IRQ	BIT(1)

/* BAM_IRQ_CLR */
#define BAM_TIMER_CLR		BIT(4)
#define BAM_EMPTY_CLR		BIT(3)
#define BAM_ERROR_CLR		BIT(2)
#define BAM_HRESP_ERR_CLR	BIT(1)

/* BAM_IRQ_EN */
#define BAM_TIMER_EN		BIT(4)
#define BAM_EMPTY_EN		BIT(3)
#define BAM_ERROR_EN		BIT(2)
#define BAM_HRESP_ERR_EN	BIT(1)

/* BAM_P_IRQ_EN */
#define P_PRCSD_DESC_EN		BIT(0)
#define P_TIMER_EN		BIT(1)
#define P_WAKE_EN		BIT(2)
#define P_OUT_OF_DESC_EN	BIT(3)
#define P_ERR_EN		BIT(4)
#define P_TRNSFR_END_EN		BIT(5)
#define P_DEFAULT_IRQS_EN	(P_PRCSD_DESC_EN | P_ERR_EN | P_TRNSFR_END_EN)

/* BAM_P_SW_OFSTS */
#define P_SW_OFSTS_MASK		0xffff

/*
 * Downstream Qualcomm SPS uses (SPS_MAX_DESC_NUM + 1) * 8 = 65 * 8 = 520
 * bytes for the descriptor FIFO.  Using a larger FIFO (e.g. SZ_32K) causes
 * AHB bus lockup on controlled-remotely BAMs like QPIC when multiple pipes
 * are active simultaneously.
 *
 * IMPORTANT: The size MUST be a power of 2 because the driver uses
 * CIRC_CNT/CIRC_SPACE macros from circ_buf.h which use bitwise AND
 * with (size-1).  2048 bytes = 256 descriptors (255 usable + 1 empty
 * slot for circular buffer management).  Matches downstream SPS desc_size.
 */
#define BAM_DESC_FIFO_SIZE	SZ_2K
#define MAX_DESCRIPTORS (BAM_DESC_FIFO_SIZE / sizeof(struct bam_desc_hw) - 1)
#define BAM_MAX_DATA_SIZE	(SZ_32K - 8)
#define IS_BUSY(chan)	(CIRC_SPACE(bchan->tail, bchan->head,\
			 MAX_DESCRIPTORS + 1) == 0)

struct bam_chan {
	struct virt_dma_chan vc;

	struct bam_device *bdev;

	/* configuration from device tree */
	u32 id;

	/* runtime configuration */
	struct dma_slave_config slave;

	/* fifo storage */
	struct bam_desc_hw *fifo_virt;
	dma_addr_t fifo_phys;

	/* fifo markers */
	unsigned short head;		/* start of active descriptor entries */
	unsigned short tail;		/* end of active descriptor entries */

	unsigned int initialized;	/* is the channel hw initialized? */
	unsigned int paused;		/* is the channel paused? */
	unsigned int reconfigure;	/* new slave config? */

	/* last known DMA direction (set by bam_prep_slave_sg) */
	enum dma_transfer_direction last_dir;
	bool dir_known;

	/* list of descriptors currently processed */
	struct list_head desc_list;

	struct list_head node;
};

static inline struct bam_chan *to_bam_chan(struct dma_chan *common)
{
	return container_of(common, struct bam_chan, vc.chan);
}

struct bam_device {
	void __iomem *regs;
	struct device *dev;
	struct dma_device common;
	struct bam_chan *channels;
	u32 num_channels;
	u32 num_ees;

	/* execution environment ID, from DT */
	u32 ee;
	bool controlled_remotely;
	bool powered_remotely;
	u32 active_channels;

	const struct reg_offset_data *layout;

	struct clk *bamclk;
	int irq;

	/* dma start transaction tasklet */
	struct tasklet_struct task;

	/* polling mode for controlled-remotely BAMs */
	bool polling;
	struct hrtimer poll_timer;
	atomic_t poll_timer_active;

	/* diagnostic dump (powered_remotely debug) */
	struct delayed_work diag_work;
	bool diag_scheduled;
};

/**
 * bam_addr - returns BAM register address
 * @bdev: bam device
 * @pipe: pipe instance (ignored when register doesn't have multiple instances)
 * @reg:  register enum
 */
static inline void __iomem *bam_addr(struct bam_device *bdev, u32 pipe,
		enum bam_reg reg)
{
	const struct reg_offset_data r = bdev->layout[reg];

	return bdev->regs + r.base_offset +
		r.pipe_mult * pipe +
		r.evnt_mult * pipe +
		r.ee_mult * bdev->ee;
}

/**
 * bam_reset() - reset and initialize BAM registers
 * @bdev: bam device
 */
static void bam_reset(struct bam_device *bdev)
{
	u32 val;

	/* s/w reset bam */
	/* after reset all pipes are disabled and idle */
	val = readl_relaxed(bam_addr(bdev, 0, BAM_CTRL));
	val |= BAM_SW_RST;
	writel_relaxed(val, bam_addr(bdev, 0, BAM_CTRL));
	val &= ~BAM_SW_RST;
	writel_relaxed(val, bam_addr(bdev, 0, BAM_CTRL));

	/* make sure previous stores are visible before enabling BAM */
	wmb();

	/* enable bam */
	val |= BAM_EN;
	writel_relaxed(val, bam_addr(bdev, 0, BAM_CTRL));

	/* set descriptor threshold, start with 4 bytes */
	writel_relaxed(DEFAULT_CNT_THRSHLD,
			bam_addr(bdev, 0, BAM_DESC_CNT_TRSHLD));

	/* Enable default set of h/w workarounds, ie all except BAM_FULL_PIPE */
	writel_relaxed(BAM_CNFG_BITS_DEFAULT, bam_addr(bdev, 0, BAM_CNFG_BITS));

	/* enable irqs for errors */
	writel_relaxed(BAM_ERROR_EN | BAM_HRESP_ERR_EN,
			bam_addr(bdev, 0, BAM_IRQ_EN));

	/* unmask global bam interrupt */
	writel_relaxed(BAM_IRQ_MSK, bam_addr(bdev, 0, BAM_IRQ_SRCS_MSK_EE));
}

/**
 * bam_enable_irqs - Full BAM reset + initialization for powered-remotely BAMs
 * @bdev: bam device
 *
 * Called when the first DMA channel is allocated on a powered-remotely BAM.
 * At this point the modem has already signaled readiness (SMSM bit 1 set),
 * so the BAM hardware is powered and accessible.
 *
 * The downstream Qualcomm SPS driver performs a FULL SW_RST even for
 * "remotely managed" BAMs (SPS_BAM_MGR_DEVICE_REMOTE).  After reset,
 * both AP and modem reconfigure their respective pipes.  The modem
 * reconfigures its side after seeing the APPS bit 11 (ACK) toggle.
 *
 * Without SW_RST, the BAM may be in a partially-configured state from
 * the modem's initial A2 power-up, which can prevent DMA transactions
 * from completing (P_SW_OFSTS stays at 0 despite valid descriptors
 * and doorbell).
 */
static void bam_enable_irqs(struct bam_device *bdev)
{
	u32 val;

	val = readl_relaxed(bam_addr(bdev, 0, BAM_CTRL));
	dev_info(bdev->dev, "BAM init: BAM_CTRL=0x%08x (before reset)\n", val);

	/* SW_RST: reset all pipes, FIFOs, and internal state */
	val |= BAM_SW_RST;
	writel_relaxed(val, bam_addr(bdev, 0, BAM_CTRL));
	val &= ~BAM_SW_RST;
	writel_relaxed(val, bam_addr(bdev, 0, BAM_CTRL));

	/* make sure reset completes before enabling BAM */
	wmb();

	/* enable BAM */
	val |= BAM_EN;
	writel_relaxed(val, bam_addr(bdev, 0, BAM_CTRL));

	/* set descriptor threshold to match downstream A2_SUMMING_THRESHOLD=4 */
	writel_relaxed(DEFAULT_CNT_THRSHLD, bam_addr(bdev, 0, BAM_DESC_CNT_TRSHLD));

	/*
	 * Configure BAM behavior bits.  Downstream sets ALL bits except
	 * BAM_FULL_PIPE (bit 11): 0xFFFFF7FF.
	 */
	writel_relaxed(0xFFFFF7FF, bam_addr(bdev, 0, BAM_CNFG_BITS));

	/* enable irqs for errors */
	writel_relaxed(BAM_ERROR_EN | BAM_HRESP_ERR_EN,
			bam_addr(bdev, 0, BAM_IRQ_EN));

	/* unmask global bam interrupt (BAM-level errors) */
	writel_relaxed(BAM_IRQ_MSK, bam_addr(bdev, 0, BAM_IRQ_SRCS_MSK_EE));

	dev_info(bdev->dev, "BAM configured: CTRL=0x%08x CNFG_BITS=0x%08x IRQ_SRCS_MSK=0x%08x\n",
		 readl_relaxed(bam_addr(bdev, 0, BAM_CTRL)),
		 readl_relaxed(bam_addr(bdev, 0, BAM_CNFG_BITS)),
		 readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_SRCS_MSK_EE)));
}

/**
 * bam_diag_dump_work - Comprehensive BAM diagnostics for powered_remotely BAMs
 * @work: delayed work struct
 *
 * Fires ~1s after the first doorbell write. Dumps all registers, descriptor
 * FIFO contents, and polls P_SW_OFSTS to detect any late DMA activity.
 */
static void bam_diag_dump_work(struct work_struct *work)
{
	struct bam_device *bdev = container_of(work, struct bam_device,
					       diag_work.work);
	struct bam_chan *bchan;
	struct bam_desc_hw *fifo;
	int i, j, n_polls;
	u32 sw_ofsts, evnt_reg;

	dev_info(bdev->dev, "=== BAM DIAG DUMP (1s after doorbell) ===\n");
	dev_info(bdev->dev, "BAM_CTRL=0x%08x BAM_REVISION=0x%08x\n",
		 readl_relaxed(bam_addr(bdev, 0, BAM_CTRL)),
		 readl_relaxed(bam_addr(bdev, 0, BAM_REVISION)));
	dev_info(bdev->dev, "BAM_DESC_CNT_TRSHLD=0x%08x BAM_IRQ_STTS=0x%08x\n",
		 readl_relaxed(bam_addr(bdev, 0, BAM_DESC_CNT_TRSHLD)),
		 readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_STTS)));
	dev_info(bdev->dev, "BAM_IRQ_SRCS_EE=0x%08x BAM_IRQ_SRCS_MSK_EE=0x%08x\n",
		 readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_SRCS_EE)),
		 readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_SRCS_MSK_EE)));
	dev_info(bdev->dev, "BAM_IRQ_EN=0x%08x BAM_CNFG_BITS=0x%08x\n",
		 readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_EN)),
		 readl_relaxed(bam_addr(bdev, 0, BAM_CNFG_BITS)));

	for (i = 0; i < bdev->num_channels && i < 6; i++) {
		u32 p_ctrl = readl_relaxed(bam_addr(bdev, i, BAM_P_CTRL));

		if (!p_ctrl)
			continue; /* skip unconfigured pipes */

		dev_info(bdev->dev,
			"pipe %d: P_CTRL=0x%08x P_EVNT_REG=0x%08x "
			"P_SW_OFSTS=0x%08x P_IRQ_STTS=0x%08x "
			"P_IRQ_EN=0x%08x\n",
			i, p_ctrl,
			readl_relaxed(bam_addr(bdev, i, BAM_P_EVNT_REG)),
			readl_relaxed(bam_addr(bdev, i, BAM_P_SW_OFSTS)),
			readl_relaxed(bam_addr(bdev, i, BAM_P_IRQ_STTS)),
			readl_relaxed(bam_addr(bdev, i, BAM_P_IRQ_EN)));
		dev_info(bdev->dev,
			"pipe %d: P_DESC_FIFO_ADDR=0x%08x P_FIFO_SIZES=0x%08x "
			"P_EVNT_GEN_TRSHLD=0x%08x\n",
			i,
			readl_relaxed(bam_addr(bdev, i, BAM_P_DESC_FIFO_ADDR)),
			readl_relaxed(bam_addr(bdev, i, BAM_P_FIFO_SIZES)),
			readl_relaxed(bam_addr(bdev, i, BAM_P_EVNT_GEN_TRSHLD)));
	}

	/* Dump first 8 descriptors in FIFO for active pipes (4 and 5) */
	for (i = 4; i <= 5; i++) {
		bchan = &bdev->channels[i];
		if (!bchan->fifo_virt)
			continue;
		fifo = bchan->fifo_virt;
		dev_info(bdev->dev,
			"pipe %d desc FIFO (virt=%px phys=0x%llx head=%u tail=%u):\n",
			i, fifo, (u64)bchan->fifo_phys, bchan->head, bchan->tail);
		for (j = 0; j < 8 && j < (BAM_DESC_FIFO_SIZE / sizeof(struct bam_desc_hw)); j++) {
			dev_info(bdev->dev,
				"  desc[%d]: addr=0x%08x size=%u flags=0x%04x\n",
				j, le32_to_cpu(fifo[j].addr),
				le16_to_cpu(fifo[j].size),
				le16_to_cpu(fifo[j].flags));
		}
	}

	/* Poll P_SW_OFSTS on pipe 5 every 200ms for 1s to detect late activity */
	dev_info(bdev->dev, "Polling pipe 5 P_SW_OFSTS (5 reads, 200ms apart):\n");
	for (n_polls = 0; n_polls < 5; n_polls++) {
		sw_ofsts = readl_relaxed(bam_addr(bdev, 5, BAM_P_SW_OFSTS));
		evnt_reg = readl_relaxed(bam_addr(bdev, 5, BAM_P_EVNT_REG));
		dev_info(bdev->dev, "  poll[%d]: P_SW_OFSTS=0x%08x P_EVNT_REG=0x%08x\n",
			 n_polls, sw_ofsts, evnt_reg);
		if (sw_ofsts != 0)
			break; /* modem started DMAing! */
		if (n_polls < 4)
			msleep(200);
	}

	dev_info(bdev->dev, "=== END BAM DIAG DUMP ===\n");
}

/**
 * bam_reset_channel - Reset individual BAM DMA channel
 * @bchan: bam channel
 *
 * This function resets a specific BAM channel
 */
static void bam_reset_channel(struct bam_chan *bchan)
{
	struct bam_device *bdev = bchan->bdev;

	lockdep_assert_held(&bchan->vc.lock);

	/*
	 * Reset channel.  Use writel() (ordered) to match downstream
	 * iowrite32 semantics — ensures the reset assert/deassert
	 * sequence completes at the BAM before subsequent register writes.
	 */
	writel(1, bam_addr(bdev, bchan->id, BAM_P_RST));
	writel(0, bam_addr(bdev, bchan->id, BAM_P_RST));

	/* make sure hw is initialized when channel is used the first time  */
	bchan->initialized = 0;
}

/**
 * bam_chan_init_hw - Initialize channel hardware
 * @bchan: bam channel
 * @dir: DMA transfer direction
 * @is_cmd_pipe: true if this pipe carries BAM command descriptors
 *
 * This function resets and initializes the BAM channel.
 *
 * Downstream Qualcomm SPS driver initializes all pipes eagerly at
 * sps_connect() time, well before any DMA operations.  It also uses
 * iowrite32 (= writel on ARM, with implicit __iowmb barrier) for ALL
 * register writes.  We match both behaviors here:
 *  - Use writel() (ordered) instead of writel_relaxed() for pipe
 *    register writes, matching downstream's iowrite32 semantics.
 *  - This function can be called from bam_prep_slave_sg() (early
 *    init path) to initialize pipes before any doorbell is written,
 *    avoiding the problematic pattern where pipe N is reset while
 *    pipe M has an active DMA session.
 */
static void bam_chan_init_hw(struct bam_chan *bchan,
	enum dma_transfer_direction dir, bool is_cmd_pipe)
{
	struct bam_device *bdev = bchan->bdev;
	u32 val;

	/* Reset the channel to clear internal state of the FIFO */
	bam_reset_channel(bchan);

	/*
	 * write out 8 byte aligned address.  We have enough space for this
	 * because we allocated 1 more descriptor (8 bytes) than we can use.
	 *
	 * Use writel() (ordered writes) to match downstream iowrite32
	 * semantics.  The implicit dsb(st) barrier in writel() ensures
	 * each register write completes at the BAM before the next one
	 * is issued.  writel_relaxed() could allow the CPU write buffer
	 * to reorder or coalesce these writes.
	 */
	writel(ALIGN(bchan->fifo_phys, sizeof(struct bam_desc_hw)),
			bam_addr(bdev, bchan->id, BAM_P_DESC_FIFO_ADDR));
	writel(BAM_DESC_FIFO_SIZE,
			bam_addr(bdev, bchan->id, BAM_P_FIFO_SIZES));

	/* Set event threshold to 0x10 to match downstream SPS configuration */
	writel(0x10, bam_addr(bdev, bchan->id, BAM_P_EVNT_GEN_TRSHLD));

	if (bdev->polling) {
		/*
		 * Downstream SPS sets P_IRQ_EN to the ACTUAL mask even
		 * for polling pipes (step 15a in bam_pipe_set_irq),
		 * then clears IRQ_SRCS_MSK_EE to prevent interrupt
		 * delivery.  Match this exactly — the BAM hardware
		 * may behave differently when P_IRQ_EN=0 vs non-zero.
		 */
		writel(P_DEFAULT_IRQS_EN,
				bam_addr(bdev, bchan->id, BAM_P_IRQ_EN));
	} else {
		/* enable the per pipe interrupts, enable EOT, ERR, and INT irqs */
		writel(P_DEFAULT_IRQS_EN,
				bam_addr(bdev, bchan->id, BAM_P_IRQ_EN));
	}

	/*
	 * IRQ_SRCS_MSK_EE handling: match the downstream SPS driver.
	 *
	 * Downstream bam_pipe_init() unconditionally SETS the pipe's
	 * bit in IRQ_SRCS_MSK_EE.  Then pipe_set_irq() immediately
	 * CLEARS it for polling-mode pipes.  The net effect for polling
	 * pipes: bit CLEARED.  For interrupt-mode: bit SET.
	 *
	 * TZ pre-sets IRQ_SRCS_MSK_EE = 0x80000007 (BAM + pipes 0-2).
	 * The downstream clears all pipe bits since QPIC uses poll mode.
	 * We match this behavior exactly.
	 */
	val = readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_SRCS_MSK_EE));
	if (bdev->polling) {
		/*
		 * Clear ALL pipe bits, not just the current pipe's.
		 * TZ pre-sets bits for pipes 0-2.  Downstream clears
		 * all of them (each pipe's pipe_set_irq clears its own).
		 * We must clear stale bits for pipes we never init
		 * (e.g. pipe 0/TX) because a set bit with no IRQ
		 * handler can cause undefined BAM behavior.
		 */
		val &= ~((1 << bdev->num_channels) - 1);
	} else
		val |= BIT(bchan->id);
	writel(val, bam_addr(bdev, 0, BAM_IRQ_SRCS_MSK_EE));

	/*
	 * Use read-modify-write for P_CTRL to preserve any bits that
	 * TrustZone may have set (e.g. on controlled-remotely BAMs).
	 * The downstream SPS driver uses individual field RMW writes
	 * for P_DIRECTION, P_SYS_MODE, P_LOCK_GROUP, and P_EN.
	 */
	val = readl_relaxed(bam_addr(bdev, bchan->id, BAM_P_CTRL));
	val |= P_EN | P_SYS_MODE;
	if (dir == DMA_DEV_TO_MEM)
		val |= P_DIRECTION;
	else
		val &= ~P_DIRECTION;

	/*
	 * CMD pipes must be in a separate lock group from data pipes.
	 * The downstream SPS driver uses lock_group=1 for CMD and 0 for
	 * data.  Without this, the BAM's internal arbitration can deadlock
	 * when CMD and data pipes share the same group.
	 */
	val &= ~(P_LOCK_GROUP_MASK << P_LOCK_GROUP_SHIFT);
	if (is_cmd_pipe)
		val |= 1 << P_LOCK_GROUP_SHIFT;

	writel(val, bam_addr(bdev, bchan->id, BAM_P_CTRL));

	/*
	 * Read back P_CTRL to ensure the BAM has processed the pipe
	 * enable before any subsequent register writes (e.g. doorbell).
	 * This matches downstream behavior where iowrite32's implicit
	 * barrier ensures completion.
	 */
	val = readl_relaxed(bam_addr(bdev, bchan->id, BAM_P_CTRL));

	bchan->initialized = 1;

	if (bdev->powered_remotely)
		dev_info(bdev->dev,
			"pipe %u init: dir=%d P_CTRL=0x%x "
			"P_DESC_FIFO_ADDR=0x%x P_FIFO_SIZES=0x%x "
			"P_EVNT_REG=0x%x P_SW_OFSTS=0x%x "
			"P_EVNT_GEN_TRSHLD=0x%x\n",
			bchan->id, dir,
			readl_relaxed(bam_addr(bdev, bchan->id, BAM_P_CTRL)),
			readl_relaxed(bam_addr(bdev, bchan->id,
					    BAM_P_DESC_FIFO_ADDR)),
			readl_relaxed(bam_addr(bdev, bchan->id,
					    BAM_P_FIFO_SIZES)),
			readl_relaxed(bam_addr(bdev, bchan->id,
					    BAM_P_EVNT_REG)),
			readl_relaxed(bam_addr(bdev, bchan->id,
					    BAM_P_SW_OFSTS)),
			readl_relaxed(bam_addr(bdev, bchan->id,
					    BAM_P_EVNT_GEN_TRSHLD)));

	/* init FIFO pointers */
	bchan->head = 0;
	bchan->tail = 0;
}

/**
 * bam_init_peer_pipes - Initialize all allocated but uninitialized peer pipes
 * @bdev: bam device
 *
 * The downstream Qualcomm SPS driver initializes ALL pipes at sps_connect()
 * time during probe, before any DMA operations.  The QPIC BAM appears to
 * require all participating pipes (0=TX, 1=RX, 2=CMD) to be enabled (P_EN=1)
 * before multi-pipe operations.  Without this, writing a doorbell to one pipe
 * while another pipe is enabled-but-uninitialized can hang the BAM's internal
 * AHB bus.
 *
 * This function is called when the first pipe is initialized.  It iterates
 * all channels and initializes any that have a descriptor FIFO allocated
 * (i.e. dma_request_chan was called) but are not yet hardware-initialized.
 *
 * For pipes whose DMA direction has been set via bam_prep_slave_sg(), the
 * stored direction is used.  For pipes that have never been prepped (e.g.
 * TX pipe during read-only operations), MEM_TO_DEV (consumer) is used as
 * a safe default — the BAM won't process any descriptors on these pipes
 * since no doorbell will be written.
 */
static void bam_init_peer_pipes(struct bam_device *bdev)
{
	unsigned int i;

	for (i = 0; i < bdev->num_channels; i++) {
		struct bam_chan *peer = &bdev->channels[i];
		enum dma_transfer_direction dir;

		if (!peer->fifo_virt || peer->initialized)
			continue;

		/* Use stored direction if known, otherwise default to consumer */
		if (peer->dir_known)
			dir = peer->last_dir;
		else
			dir = DMA_MEM_TO_DEV;

		dev_dbg(bdev->dev,
			"pipe %u: force-init peer (dir=%d known=%d)\n",
			peer->id, dir, peer->dir_known);

		scoped_guard(spinlock_irqsave, &peer->vc.lock)
			bam_chan_init_hw(peer, dir, false);
	}
}

/**
 * bam_alloc_chan - Allocate channel resources for DMA channel.
 * @chan: specified channel
 *
 * This function allocates the FIFO descriptor memory
 */
static int bam_alloc_chan(struct dma_chan *chan)
{
	struct bam_chan *bchan = to_bam_chan(chan);
	struct bam_device *bdev = bchan->bdev;

	if (bchan->fifo_virt)
		return 0;

	/* allocate FIFO descriptor space, but only if necessary */
	bchan->fifo_virt = dma_alloc_coherent(bdev->dev, BAM_DESC_FIFO_SIZE,
					&bchan->fifo_phys, GFP_KERNEL);

	if (!bchan->fifo_virt) {
		dev_err(bdev->dev, "Failed to allocate desc fifo\n");
		return -ENOMEM;
	}

	if (bdev->active_channels++ == 0 && bdev->powered_remotely)
		bam_enable_irqs(bdev);

	return 0;
}

/**
 * bam_free_chan - Frees dma resources associated with specific channel
 * @chan: specified channel
 *
 * Free the allocated fifo descriptor memory and channel resources
 *
 */
static void bam_free_chan(struct dma_chan *chan)
{
	struct bam_chan *bchan = to_bam_chan(chan);
	struct bam_device *bdev = bchan->bdev;
	u32 val;
	int ret;

	ret = pm_runtime_get_sync(bdev->dev);
	if (ret < 0)
		return;

	vchan_free_chan_resources(to_virt_chan(chan));

	if (!list_empty(&bchan->desc_list)) {
		dev_err(bchan->bdev->dev, "Cannot free busy channel\n");
		goto err;
	}

	scoped_guard(spinlock_irqsave, &bchan->vc.lock)
		bam_reset_channel(bchan);

	dma_free_coherent(bdev->dev, BAM_DESC_FIFO_SIZE, bchan->fifo_virt,
		    bchan->fifo_phys);
	bchan->fifo_virt = NULL;

	/* mask irq for pipe/channel */
	if (!bdev->polling) {
		val = readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_SRCS_MSK_EE));
		val &= ~BIT(bchan->id);
		writel_relaxed(val, bam_addr(bdev, 0, BAM_IRQ_SRCS_MSK_EE));
	}

	/* disable irq */
	writel_relaxed(0, bam_addr(bdev, bchan->id, BAM_P_IRQ_EN));

	if (--bdev->active_channels == 0 && bdev->powered_remotely) {
		/* s/w reset bam */
		val = readl_relaxed(bam_addr(bdev, 0, BAM_CTRL));
		val |= BAM_SW_RST;
		writel_relaxed(val, bam_addr(bdev, 0, BAM_CTRL));
	}

err:
	pm_runtime_mark_last_busy(bdev->dev);
	pm_runtime_put_autosuspend(bdev->dev);
}

/**
 * bam_slave_config - set slave configuration for channel
 * @chan: dma channel
 * @cfg: slave configuration
 *
 * Sets slave configuration for channel
 *
 */
static int bam_slave_config(struct dma_chan *chan,
			    struct dma_slave_config *cfg)
{
	struct bam_chan *bchan = to_bam_chan(chan);

	guard(spinlock_irqsave)(&bchan->vc.lock);

	memcpy(&bchan->slave, cfg, sizeof(*cfg));
	bchan->reconfigure = 1;

	return 0;
}

/**
 * bam_prep_slave_sg - Prep slave sg transaction
 *
 * @chan: dma channel
 * @sgl: scatter gather list
 * @sg_len: length of sg
 * @direction: DMA transfer direction
 * @flags: DMA flags
 * @context: transfer context (unused)
 */
static struct dma_async_tx_descriptor *bam_prep_slave_sg(struct dma_chan *chan,
	struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction, unsigned long flags,
	void *context)
{
	struct bam_chan *bchan = to_bam_chan(chan);
	struct bam_device *bdev = bchan->bdev;
	struct bam_async_desc *async_desc;
	struct scatterlist *sg;
	u32 i;
	struct bam_desc_hw *desc;
	unsigned int num_alloc;

	if (!is_slave_direction(direction)) {
		dev_err(bdev->dev, "invalid dma direction\n");
		return NULL;
	}

	/*
	 * Track init state before updating direction.  A pipe that was
	 * force-initialized by bam_init_peer_pipes() has initialized=1
	 * but dir_known=false.  Such pipes need their P_DIRECTION bit
	 * updated when they're actually prepped with the real direction.
	 */
	{
		bool needs_init = !bchan->initialized;
		bool needs_dir_fix = bchan->initialized &&
				     !bchan->dir_known;

		bchan->last_dir = direction;
		bchan->dir_known = true;

		if (needs_init) {
			bool is_cmd = !!(flags & DMA_PREP_CMD);

			scoped_guard(spinlock_irqsave, &bchan->vc.lock) {
				bam_chan_init_hw(bchan, direction, is_cmd);
			}

			/*
			 * Match downstream SPS: initialize all peer pipes
			 * now (before any DMA doorbell) to avoid resetting
			 * a pipe while other pipes have active DMA sessions.
			 * Downstream sps_connect() inits all pipes eagerly
			 * during probe.
			 */
			bam_init_peer_pipes(bdev);
		} else if (needs_dir_fix) {
			/*
			 * Pipe was force-initialized with default direction
			 * (consumer).  The correct direction is now known.
			 * Do a full P_RST + re-init rather than modifying
			 * P_DIRECTION on a live (P_EN=1) pipe, which may
			 * corrupt BAM internal pipe state.
			 */
			dev_dbg(bdev->dev,
				"pipe %u: dir fix via re-init (dir=%d)\n",
				bchan->id, direction);

			bchan->initialized = 0;
			scoped_guard(spinlock_irqsave, &bchan->vc.lock)
				bam_chan_init_hw(bchan, direction, false);
		}
	}

	/* allocate enough room to accommodate the number of entries */
	num_alloc = sg_nents_for_dma(sgl, sg_len, BAM_MAX_DATA_SIZE);
	async_desc = kzalloc_flex(*async_desc, desc, num_alloc, GFP_NOWAIT);
	if (!async_desc)
		return NULL;

	if (flags & DMA_PREP_FENCE)
		async_desc->flags |= DESC_FLAG_NWD;

	if (flags & DMA_PREP_INTERRUPT)
		async_desc->flags |= DESC_FLAG_EOT;

	async_desc->num_desc = num_alloc;
	async_desc->curr_desc = async_desc->desc;
	async_desc->dir = direction;

	/* fill in temporary descriptors */
	desc = async_desc->desc;
	for_each_sg(sgl, sg, sg_len, i) {
		unsigned int remainder = sg_dma_len(sg);
		unsigned int curr_offset = 0;

		do {
			if (flags & DMA_PREP_CMD)
				desc->flags |= cpu_to_le16(DESC_FLAG_CMD);

			desc->addr = cpu_to_le32(sg_dma_address(sg) +
						 curr_offset);

			if (remainder > BAM_MAX_DATA_SIZE) {
				desc->size = cpu_to_le16(BAM_MAX_DATA_SIZE);
				remainder -= BAM_MAX_DATA_SIZE;
				curr_offset += BAM_MAX_DATA_SIZE;
			} else {
				desc->size = cpu_to_le16(remainder);
				remainder = 0;
			}

			async_desc->length += le16_to_cpu(desc->size);
			desc++;
		} while (remainder > 0);
	}

	return vchan_tx_prep(&bchan->vc, &async_desc->vd, flags);
}

/**
 * bam_dma_terminate_all - terminate all transactions on a channel
 * @chan: bam dma channel
 *
 * Dequeues and frees all transactions
 * No callbacks are done
 *
 */
static int bam_dma_terminate_all(struct dma_chan *chan)
{
	struct bam_chan *bchan = to_bam_chan(chan);
	struct bam_async_desc *async_desc, *tmp;
	LIST_HEAD(head);

	/* remove all transactions, including active transaction */
	scoped_guard(spinlock_irqsave, &bchan->vc.lock) {
		/*
		 * If we have transactions queued, then some might be committed to the
		 * hardware in the desc fifo.  The only way to reset the desc fifo is
		 * to do a hardware reset (either by pipe or the entire block).
		 * bam_chan_init_hw() will trigger a pipe reset, and also reinit the
		 * pipe.  If the pipe is left disabled (default state after pipe reset)
		 * and is accessed by a connected hardware engine, a fatal error in
		 * the BAM will occur.  There is a small window where this could happen
		 * with bam_chan_init_hw(), but it is assumed that the caller has
		 * stopped activity on any attached hardware engine.  Make sure to do
		 * this first so that the BAM hardware doesn't cause memory corruption
		 * by accessing freed resources.
		 */
		if (!list_empty(&bchan->desc_list)) {
			async_desc = list_first_entry(&bchan->desc_list,
						      struct bam_async_desc, desc_node);
			bam_chan_init_hw(bchan, async_desc->dir,
					 async_desc->desc[0].flags &
					 cpu_to_le16(DESC_FLAG_CMD));
		}

		list_for_each_entry_safe(async_desc, tmp,
					 &bchan->desc_list, desc_node) {
			list_add(&async_desc->vd.node, &bchan->vc.desc_issued);
			list_del(&async_desc->desc_node);
		}

		vchan_get_all_descriptors(&bchan->vc, &head);
	}

	vchan_dma_desc_free_list(&bchan->vc, &head);

	return 0;
}

/**
 * bam_pause - Pause DMA channel
 * @chan: dma channel
 *
 */
static int bam_pause(struct dma_chan *chan)
{
	struct bam_chan *bchan = to_bam_chan(chan);
	struct bam_device *bdev = bchan->bdev;
	int ret;

	ret = pm_runtime_get_sync(bdev->dev);
	if (ret < 0)
		return ret;

	scoped_guard(spinlock_irqsave, &bchan->vc.lock) {
		writel_relaxed(1, bam_addr(bdev, bchan->id, BAM_P_HALT));
		bchan->paused = 1;
	}
	pm_runtime_mark_last_busy(bdev->dev);
	pm_runtime_put_autosuspend(bdev->dev);

	return 0;
}

/**
 * bam_resume - Resume DMA channel operations
 * @chan: dma channel
 *
 */
static int bam_resume(struct dma_chan *chan)
{
	struct bam_chan *bchan = to_bam_chan(chan);
	struct bam_device *bdev = bchan->bdev;
	int ret;

	ret = pm_runtime_get_sync(bdev->dev);
	if (ret < 0)
		return ret;

	scoped_guard(spinlock_irqsave, &bchan->vc.lock) {
		writel_relaxed(0, bam_addr(bdev, bchan->id, BAM_P_HALT));
		bchan->paused = 0;
	}
	pm_runtime_mark_last_busy(bdev->dev);
	pm_runtime_put_autosuspend(bdev->dev);

	return 0;
}

/**
 * bam_process_pipe_completions - process completions on a single pipe
 * @bdev: bam controller
 * @pipe: pipe/channel index
 *
 * Reads the pipe IRQ status and SW offset to determine which descriptors
 * have completed.  Usable from both IRQ and polling contexts.
 */
static void bam_process_pipe_completions(struct bam_device *bdev, u32 pipe)
{
	struct bam_chan *bchan = &bdev->channels[pipe];
	struct bam_async_desc *async_desc, *tmp;
	u32 offset;
	unsigned int avail;

	if (bdev->polling) {
		u32 pipe_stts;

		/*
		 * Match downstream SPS driver: read and clear P_IRQ_STTS
		 * even in polling mode.  The downstream pipe_handler()
		 * calls bam_pipe_get_and_clear_irq_status() on every
		 * poll iteration, which reads P_IRQ_STTS and writes the
		 * value back to P_IRQ_CLR.  Without this, accumulated
		 * IRQ status bits (P_PRCSD_DESC, P_WAKE, etc.) from
		 * completed operations are never cleared, which may
		 * affect BAM internal state on subsequent operations.
		 */
		pipe_stts = readl_relaxed(bam_addr(bdev, pipe,
						   BAM_P_IRQ_STTS));
		if (pipe_stts)
			writel_relaxed(pipe_stts, bam_addr(bdev, pipe,
						   BAM_P_IRQ_CLR));

		dev_dbg(bdev->dev,
			"poll: pipe %u P_SW_OFSTS=0x%x P_IRQ_STTS=0x%x head=%u\n",
			pipe,
			readl_relaxed(bam_addr(bdev, pipe, BAM_P_SW_OFSTS)),
			pipe_stts,
			bchan->head);
	} else {
		u32 pipe_stts;

		pipe_stts = readl_relaxed(bam_addr(bdev, pipe,
						   BAM_P_IRQ_STTS));
		if (!pipe_stts)
			return;

		writel_relaxed(pipe_stts, bam_addr(bdev, pipe,
						   BAM_P_IRQ_CLR));
	}

	guard(spinlock_irqsave)(&bchan->vc.lock);

	offset = readl_relaxed(bam_addr(bdev, pipe, BAM_P_SW_OFSTS)) &
			       P_SW_OFSTS_MASK;
	offset /= sizeof(struct bam_desc_hw);

	avail = CIRC_CNT(offset, bchan->head, MAX_DESCRIPTORS + 1);

	list_for_each_entry_safe(async_desc, tmp,
				 &bchan->desc_list, desc_node) {
		if (avail < async_desc->xfer_len)
			break;

		bchan->head += async_desc->xfer_len;
		bchan->head %= (MAX_DESCRIPTORS + 1);

		async_desc->num_desc -= async_desc->xfer_len;
		async_desc->curr_desc += async_desc->xfer_len;
		avail -= async_desc->xfer_len;

		if (!async_desc->num_desc) {
			vchan_cookie_complete(&async_desc->vd);
		} else {
			list_add(&async_desc->vd.node,
				 &bchan->vc.desc_issued);
		}
		list_del(&async_desc->desc_node);
	}
}

/**
 * process_channel_irqs - processes the channel interrupts
 * @bdev: bam controller
 *
 * This function processes the channel interrupts
 *
 */
static u32 process_channel_irqs(struct bam_device *bdev)
{
	u32 i, srcs;

	srcs = readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_SRCS_EE));

	/* return early if no pipe/channel interrupts are present */
	if (!(srcs & P_IRQ))
		return srcs;

	for (i = 0; i < bdev->num_channels; i++) {
		if (srcs & BIT(i))
			bam_process_pipe_completions(bdev, i);
	}

	return srcs;
}

/**
 * bam_poll_timer_fn - hrtimer callback for polling-mode BAMs
 * @timer: hrtimer embedded in bam_device
 *
 * For controlled-remotely BAMs where IRQs may not reach the APPS CPU,
 * this timer polls pipe completion status periodically.
 */
static enum hrtimer_restart bam_poll_timer_fn(struct hrtimer *timer)
{
	struct bam_device *bdev = container_of(timer, struct bam_device,
					       poll_timer);
	bool any_active = false;
	unsigned int i;

	dev_dbg(bdev->dev, "poll_timer fired\n");

	for (i = 0; i < bdev->num_channels; i++) {
		struct bam_chan *bchan = &bdev->channels[i];

		if (list_empty(&bchan->desc_list))
			continue;

		bam_process_pipe_completions(bdev, i);
	}

	tasklet_schedule(&bdev->task);

	for (i = 0; i < bdev->num_channels; i++) {
		struct bam_chan *bchan = &bdev->channels[i];

		if (!list_empty(&bchan->desc_list) ||
		    !list_empty(&bchan->vc.desc_issued)) {
			any_active = true;
			break;
		}
	}

	if (any_active) {
		hrtimer_forward_now(timer, ns_to_ktime(100000));
		return HRTIMER_RESTART;
	}

	atomic_set(&bdev->poll_timer_active, 0);
	return HRTIMER_NORESTART;
}

static void bam_start_poll_timer(struct bam_device *bdev)
{
	if (!bdev->polling)
		return;

	if (!atomic_xchg(&bdev->poll_timer_active, 1))
		hrtimer_start(&bdev->poll_timer, ns_to_ktime(50000),
			      HRTIMER_MODE_REL);
}

/**
 * bam_dma_irq - irq handler for bam controller
 * @irq: IRQ of interrupt
 * @data: callback data
 *
 * IRQ handler for the bam controller
 */
static irqreturn_t bam_dma_irq(int irq, void *data)
{
	struct bam_device *bdev = data;
	u32 clr_mask = 0, srcs = 0;
	int ret;

	if (bdev->powered_remotely)
		dev_info(bdev->dev, "IRQ! srcs_ee=0x%x\n",
			 readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_SRCS_EE)));

	srcs |= process_channel_irqs(bdev);

	/* kick off tasklet to start next dma transfer */
	if (srcs & P_IRQ)
		tasklet_schedule(&bdev->task);

	ret = pm_runtime_get_sync(bdev->dev);
	if (ret < 0)
		return IRQ_NONE;

	if (srcs & BAM_IRQ) {
		clr_mask = readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_STTS));

		/*
		 * don't allow reorder of the various accesses to the BAM
		 * registers
		 */
		mb();

		writel_relaxed(clr_mask, bam_addr(bdev, 0, BAM_IRQ_CLR));
	}

	pm_runtime_mark_last_busy(bdev->dev);
	pm_runtime_put_autosuspend(bdev->dev);

	return IRQ_HANDLED;
}

/**
 * bam_tx_status - returns status of transaction
 * @chan: dma channel
 * @cookie: transaction cookie
 * @txstate: DMA transaction state
 *
 * Return status of dma transaction
 */
static enum dma_status bam_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
		struct dma_tx_state *txstate)
{
	struct bam_chan *bchan = to_bam_chan(chan);
	struct bam_device *bdev = bchan->bdev;
	struct bam_async_desc *async_desc;
	struct virt_dma_desc *vd;
	int ret;
	size_t residue = 0;
	unsigned int i;

	/*
	 * In polling mode, process hardware completions on ALL active
	 * pipes before checking cookie status.  This is needed because
	 * a multi-pipe peripheral (e.g. QPIC NAND with cmd/tx/rx pipes)
	 * requires all pipes to be drained — otherwise completed
	 * descriptors remain in desc_list, IS_BUSY stays true, and
	 * subsequent operations on that pipe never get doorbelled.
	 * This matches what bam_poll_timer_fn() already does.
	 */
	if (bdev->polling) {
		for (i = 0; i < bdev->num_channels; i++) {
			struct bam_chan *bc = &bdev->channels[i];

			if (bc->initialized && !list_empty(&bc->desc_list))
				bam_process_pipe_completions(bdev, i);
		}
	}

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE)
		return ret;

	if (!txstate)
		return bchan->paused ? DMA_PAUSED : ret;

	scoped_guard(spinlock_irqsave, &bchan->vc.lock) {
		vd = vchan_find_desc(&bchan->vc, cookie);
		if (vd) {
			residue = container_of(vd, struct bam_async_desc, vd)->length;
		} else {
			list_for_each_entry(async_desc, &bchan->desc_list, desc_node) {
				if (async_desc->vd.tx.cookie != cookie)
					continue;

				for (i = 0; i < async_desc->num_desc; i++)
					residue += le16_to_cpu(
							async_desc->curr_desc[i].size);
			}
		}
	}

	dma_set_residue(txstate, residue);

	if (ret == DMA_IN_PROGRESS && bchan->paused)
		ret = DMA_PAUSED;

	return ret;
}

/**
 * bam_apply_new_config
 * @bchan: bam dma channel
 * @dir: DMA direction
 */
static void bam_apply_new_config(struct bam_chan *bchan,
	enum dma_transfer_direction dir)
{
	struct bam_device *bdev = bchan->bdev;
	u32 maxburst;

	if (!bdev->controlled_remotely && !bdev->powered_remotely) {
		if (dir == DMA_DEV_TO_MEM)
			maxburst = bchan->slave.src_maxburst;
		else
			maxburst = bchan->slave.dst_maxburst;

		writel_relaxed(maxburst,
			       bam_addr(bdev, 0, BAM_DESC_CNT_TRSHLD));
	}

	bchan->reconfigure = 0;
}

/**
 * bam_start_dma - start next transaction
 * @bchan: bam dma channel
 */
static void bam_start_dma(struct bam_chan *bchan)
{
	struct virt_dma_desc *vd = vchan_next_desc(&bchan->vc);
	struct bam_device *bdev = bchan->bdev;
	struct bam_async_desc *async_desc = NULL;
	struct bam_desc_hw *desc;
	struct bam_desc_hw *fifo = PTR_ALIGN(bchan->fifo_virt,
					sizeof(struct bam_desc_hw));
	int ret;
	unsigned int avail;
	struct dmaengine_desc_callback cb;
	unsigned int fifo_start;
	int first_cmd_fifo_idx = -1;
	int last_cmd_fifo_idx = -1;

	lockdep_assert_held(&bchan->vc.lock);

	if (!vd)
		return;

	ret = pm_runtime_get_sync(bdev->dev);
	if (ret < 0)
		return;

	fifo_start = bchan->tail;

	while (vd && !IS_BUSY(bchan)) {
		list_del(&vd->node);

		async_desc = container_of(vd, struct bam_async_desc, vd);

		/*
		 * Fallback init: normally the pipe is initialized eagerly
		 * in bam_prep_slave_sg().  This handles edge cases where
		 * prep didn't initialize (e.g. terminate_all + reuse).
		 */
		if (!bchan->initialized) {
			bam_chan_init_hw(bchan, async_desc->dir,
					 async_desc->desc[0].flags &
					 cpu_to_le16(DESC_FLAG_CMD));
		}

		/* apply new slave config changes, if necessary */
		if (bchan->reconfigure)
			bam_apply_new_config(bchan, async_desc->dir);

		desc = async_desc->curr_desc;
		avail = CIRC_SPACE(bchan->tail, bchan->head,
				   MAX_DESCRIPTORS + 1);

		if (async_desc->num_desc > avail)
			async_desc->xfer_len = avail;
		else
			async_desc->xfer_len = async_desc->num_desc;

		/* set any special flags on the last descriptor */
		if (async_desc->num_desc == async_desc->xfer_len)
			desc[async_desc->xfer_len - 1].flags |=
						cpu_to_le16(async_desc->flags);

		vd = vchan_next_desc(&bchan->vc);

		dmaengine_desc_get_callback(&async_desc->vd.tx, &cb);

		/*
		 * An interrupt is generated at this desc, if
		 *  - FIFO is FULL.
		 *  - No more descriptors to add.
		 *  - If a callback completion was requested for this DESC,
		 *     In this case, BAM will deliver the completion callback
		 *     for this desc and continue processing the next desc.
		 */
		if (((avail <= async_desc->xfer_len) || !vd ||
		     dmaengine_desc_callback_valid(&cb)) &&
		    !(async_desc->flags & DESC_FLAG_EOT))
			desc[async_desc->xfer_len - 1].flags |=
				cpu_to_le16(DESC_FLAG_INT);

		if (bchan->tail + async_desc->xfer_len > MAX_DESCRIPTORS + 1) {
			u32 partial = MAX_DESCRIPTORS + 1 - bchan->tail;

			memcpy(&fifo[bchan->tail], desc,
			       partial * sizeof(struct bam_desc_hw));
			memcpy(fifo, &desc[partial],
			       (async_desc->xfer_len - partial) *
				sizeof(struct bam_desc_hw));
		} else {
			memcpy(&fifo[bchan->tail], desc,
			       async_desc->xfer_len *
			       sizeof(struct bam_desc_hw));
		}

		/*
		 * Track first and last CMD descriptors in FIFO for
		 * LOCK/UNLOCK flag insertion (see below).
		 */
		{
			unsigned int i;

			for (i = 0; i < async_desc->xfer_len; i++) {
				unsigned int idx = (bchan->tail + i) %
						   (MAX_DESCRIPTORS + 1);
				if (fifo[idx].flags &
				    cpu_to_le16(DESC_FLAG_CMD)) {
					if (first_cmd_fifo_idx < 0)
						first_cmd_fifo_idx = idx;
					last_cmd_fifo_idx = idx;
				}
			}
		}

		bchan->tail += async_desc->xfer_len;
		bchan->tail %= (MAX_DESCRIPTORS + 1);
		list_add_tail(&async_desc->desc_node, &bchan->desc_list);
	}

	/*
	 * Add LOCK/UNLOCK flags to CMD descriptors.
	 *
	 * The downstream Qualcomm SPS NAND driver sets LOCK (0x0400) on
	 * the first CMD descriptor and UNLOCK (0x0200) on the last CMD
	 * descriptor for every multi-descriptor CMD sequence.  These
	 * flags control the BAM's internal pipe group arbiter:
	 *
	 *   LOCK:   Acquire the pipe's lock group before processing.
	 *           Prevents the BAM from switching to pipes in other
	 *           lock groups until UNLOCK is seen.
	 *   UNLOCK: Release the pipe's lock group after processing.
	 *
	 * Without LOCK/UNLOCK, when the BAM sees pending descriptors on
	 * pipes in different lock groups (e.g. CMD pipe in group 1 and
	 * data pipe in group 0), the arbiter may deadlock internally,
	 * causing the BAM to stop responding to register writes.
	 */
	if (first_cmd_fifo_idx >= 0) {
		fifo[first_cmd_fifo_idx].flags |=
			cpu_to_le16(DESC_FLAG_LOCK);
		fifo[last_cmd_fifo_idx].flags |=
			cpu_to_le16(DESC_FLAG_UNLOCK);
	}

	/* Dump ALL FIFO descriptors for debugging (disabled for freeze test) */
#if 0
	{
		unsigned int n, count;

		count = (bchan->tail >= fifo_start)
			? bchan->tail - fifo_start
			: (MAX_DESCRIPTORS + 1) - fifo_start + bchan->tail;
		for (n = 0; n < count; n++) {
			unsigned int idx = (fifo_start + n) % (MAX_DESCRIPTORS + 1);

			dev_info(bdev->dev,
				"pipe %u: desc[%u] addr=0x%08x size=%u flags=0x%04x\n",
				bchan->id, idx,
				le32_to_cpu(fifo[idx].addr),
				le16_to_cpu(fifo[idx].size),
				le16_to_cpu(fifo[idx].flags));
		}
	}
#endif

	/*
	 * Ensure descriptor FIFO writes are visible to the BAM before
	 * the doorbell write.
	 */
	wmb();
	{
		u32 db_val = bchan->tail * sizeof(struct bam_desc_hw);

		if (bdev->powered_remotely)
			dev_info(bdev->dev,
				"pipe %u doorbell: val=0x%x (tail=%u descs=%u)\n",
				bchan->id, db_val, bchan->tail,
				db_val / (u32)sizeof(struct bam_desc_hw));

		writel(db_val, bam_addr(bdev, bchan->id, BAM_P_EVNT_REG));
	}

	/* Schedule one-shot diagnostic dump 1s after first doorbell */
	if (bdev->powered_remotely && !bdev->diag_scheduled) {
		bdev->diag_scheduled = true;
		schedule_delayed_work(&bdev->diag_work, msecs_to_jiffies(1000));
	}

	bam_start_poll_timer(bdev);

	pm_runtime_mark_last_busy(bdev->dev);
	pm_runtime_put_autosuspend(bdev->dev);
}

/**
 * dma_tasklet - DMA IRQ tasklet
 * @t: tasklet argument (bam controller structure)
 *
 * Sets up next DMA operation and then processes all completed transactions
 */
static void dma_tasklet(struct tasklet_struct *t)
{
	struct bam_device *bdev = from_tasklet(bdev, t, task);
	struct bam_chan *bchan;
	unsigned int i;

	dev_dbg(bdev->dev, "dma_tasklet running\n");

	/* go through the channels and kick off transactions */
	for (i = 0; i < bdev->num_channels; i++) {
		bchan = &bdev->channels[i];

		guard(spinlock_irqsave)(&bchan->vc.lock);

		if (!list_empty(&bchan->vc.desc_issued) && !IS_BUSY(bchan))
			bam_start_dma(bchan);
	}

}

/**
 * bam_issue_pending - starts pending transactions
 * @chan: dma channel
 *
 * Calls tasklet directly which in turn starts any pending transactions
 */
static void bam_issue_pending(struct dma_chan *chan)
{
	struct bam_chan *bchan = to_bam_chan(chan);

	guard(spinlock_irqsave)(&bchan->vc.lock);

	/* if work pending and idle, start a transaction */
	if (vchan_issue_pending(&bchan->vc) && !IS_BUSY(bchan))
		bam_start_dma(bchan);
}

/**
 * bam_dma_free_desc - free descriptor memory
 * @vd: virtual descriptor
 *
 */
static void bam_dma_free_desc(struct virt_dma_desc *vd)
{
	struct bam_async_desc *async_desc = container_of(vd,
			struct bam_async_desc, vd);

	kfree(async_desc);
}

static struct dma_chan *bam_dma_xlate(struct of_phandle_args *dma_spec,
		struct of_dma *of)
{
	struct bam_device *bdev = container_of(of->of_dma_data,
					struct bam_device, common);
	unsigned int request;

	if (dma_spec->args_count != 1)
		return NULL;

	request = dma_spec->args[0];
	if (request >= bdev->num_channels)
		return NULL;

	return dma_get_slave_channel(&(bdev->channels[request].vc.chan));
}

/**
 * bam_init
 * @bdev: bam device
 *
 * Initialization helper for global bam registers
 */
static int bam_init(struct bam_device *bdev)
{
	u32 val;

	/* read revision and configuration information */
	if (!bdev->num_ees) {
		val = readl_relaxed(bam_addr(bdev, 0, BAM_REVISION));
		bdev->num_ees = (val >> NUM_EES_SHIFT) & NUM_EES_MASK;
	}

	/* check that configured EE is within range */
	if (bdev->ee >= bdev->num_ees)
		return -EINVAL;

	if (!bdev->num_channels) {
		val = readl_relaxed(bam_addr(bdev, 0, BAM_NUM_PIPES));
		bdev->num_channels = val & BAM_NUM_PIPES_MASK;
	}

	/* Reset BAM now if fully controlled locally */
	if (!bdev->controlled_remotely && !bdev->powered_remotely) {
		bam_reset(bdev);
	} else {
		/*
		 * For remotely-controlled BAMs, TZ already initialized the
		 * global config including BAM_IRQ_EN.  Don't touch global
		 * registers here — they may be XPU-protected.  Per-pipe
		 * IRQ setup (P_IRQ_EN + BAM_IRQ_SRCS_MSK_EE per-pipe bit)
		 * is handled later by bam_chan_init_hw().
		 */
	}

	return 0;
}

static void bam_channel_init(struct bam_device *bdev, struct bam_chan *bchan,
	u32 index)
{
	bchan->id = index;
	bchan->bdev = bdev;

	vchan_init(&bchan->vc, &bdev->common);
	bchan->vc.desc_free = bam_dma_free_desc;
	INIT_LIST_HEAD(&bchan->desc_list);
}

static const struct of_device_id bam_of_match[] = {
	{ .compatible = "qcom,bam-v1.3.0", .data = &bam_v1_3_reg_info },
	{ .compatible = "qcom,bam-v1.4.0", .data = &bam_v1_4_reg_info },
	{ .compatible = "qcom,bam-v1.7.0", .data = &bam_v1_7_reg_info },
	{}
};

MODULE_DEVICE_TABLE(of, bam_of_match);

static int bam_dma_probe(struct platform_device *pdev)
{
	struct bam_device *bdev;
	const struct of_device_id *match;
	int ret, i;

	bdev = devm_kzalloc(&pdev->dev, sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;

	bdev->dev = &pdev->dev;

	match = of_match_node(bam_of_match, pdev->dev.of_node);
	if (!match) {
		dev_err(&pdev->dev, "Unsupported BAM module\n");
		return -ENODEV;
	}

	bdev->layout = match->data;

	bdev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bdev->regs))
		return PTR_ERR(bdev->regs);

	bdev->irq = platform_get_irq(pdev, 0);
	if (bdev->irq < 0)
		return bdev->irq;

	ret = of_property_read_u32(pdev->dev.of_node, "qcom,ee", &bdev->ee);
	if (ret) {
		dev_err(bdev->dev, "Execution environment unspecified\n");
		return ret;
	}

	bdev->controlled_remotely = of_property_read_bool(pdev->dev.of_node,
						"qcom,controlled-remotely");
	bdev->powered_remotely = of_property_read_bool(pdev->dev.of_node,
						"qcom,powered-remotely");

	bdev->polling = bdev->controlled_remotely;
	if (bdev->polling) {
		hrtimer_setup(&bdev->poll_timer, bam_poll_timer_fn,
			      CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		atomic_set(&bdev->poll_timer_active, 0);
	}

	if (bdev->powered_remotely)
		INIT_DELAYED_WORK(&bdev->diag_work, bam_diag_dump_work);

	if (bdev->controlled_remotely || bdev->powered_remotely)
		bdev->bamclk = devm_clk_get_optional(bdev->dev, "bam_clk");
	else
		bdev->bamclk = devm_clk_get(bdev->dev, "bam_clk");

	if (IS_ERR(bdev->bamclk))
		return PTR_ERR(bdev->bamclk);

	/*
	 * Read num-channels / num-ees from DT when available, regardless
	 * of whether a clock was found.  The upstream code only reads these
	 * when bamclk is NULL, but controlled-remotely BAMs may provide
	 * both a clock AND these properties to avoid register reads during
	 * bam_init() (the BAM registers may be inaccessible until the
	 * clock is enabled).
	 */
	of_property_read_u32(pdev->dev.of_node, "num-channels",
			     &bdev->num_channels);
	of_property_read_u32(pdev->dev.of_node, "qcom,num-ees",
			     &bdev->num_ees);

	if (!bdev->bamclk && !bdev->num_channels) {
		dev_err(bdev->dev, "num-channels unspecified in dt\n");
		return -EINVAL;
	}

	if (!bdev->bamclk && !bdev->num_ees) {
		dev_err(bdev->dev, "num-ees unspecified in dt\n");
		return -EINVAL;
	}

	ret = clk_prepare_enable(bdev->bamclk);
	if (ret) {
		dev_err(bdev->dev, "failed to prepare/enable clock\n");
		return ret;
	}

	ret = bam_init(bdev);
	if (ret)
		goto err_disable_clk;

	if (bdev->controlled_remotely || bdev->powered_remotely) {
		dev_dbg(bdev->dev, "BAM TZ state: CTRL=0x%x IRQ_SRCS_MSK=0x%x\n",
			readl_relaxed(bam_addr(bdev, 0, BAM_CTRL)),
			readl_relaxed(bam_addr(bdev, 0, BAM_IRQ_SRCS_MSK_EE)));
	}

	tasklet_setup(&bdev->task, dma_tasklet);

	bdev->channels = devm_kcalloc(bdev->dev, bdev->num_channels,
				sizeof(*bdev->channels), GFP_KERNEL);

	if (!bdev->channels) {
		ret = -ENOMEM;
		goto err_tasklet_kill;
	}

	/* allocate and initialize channels */
	INIT_LIST_HEAD(&bdev->common.channels);

	for (i = 0; i < bdev->num_channels; i++)
		bam_channel_init(bdev, &bdev->channels[i], i);

	if (!bdev->polling) {
		ret = devm_request_irq(bdev->dev, bdev->irq, bam_dma_irq,
				IRQF_TRIGGER_HIGH, "bam_dma", bdev);
		if (ret)
			goto err_bam_channel_exit;
	}

	/* set max dma segment size */
	bdev->common.dev = bdev->dev;
	dma_set_max_seg_size(bdev->common.dev, BAM_MAX_DATA_SIZE);

	platform_set_drvdata(pdev, bdev);

	/* set capabilities */
	dma_cap_zero(bdev->common.cap_mask);
	dma_cap_set(DMA_SLAVE, bdev->common.cap_mask);

	/* initialize dmaengine apis */
	bdev->common.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	bdev->common.residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;
	bdev->common.src_addr_widths = DMA_SLAVE_BUSWIDTH_4_BYTES;
	bdev->common.dst_addr_widths = DMA_SLAVE_BUSWIDTH_4_BYTES;
	bdev->common.device_alloc_chan_resources = bam_alloc_chan;
	bdev->common.device_free_chan_resources = bam_free_chan;
	bdev->common.device_prep_slave_sg = bam_prep_slave_sg;
	bdev->common.device_config = bam_slave_config;
	bdev->common.device_pause = bam_pause;
	bdev->common.device_resume = bam_resume;
	bdev->common.device_terminate_all = bam_dma_terminate_all;
	bdev->common.device_issue_pending = bam_issue_pending;
	bdev->common.device_tx_status = bam_tx_status;
	bdev->common.dev = bdev->dev;

	ret = dma_async_device_register(&bdev->common);
	if (ret) {
		dev_err(bdev->dev, "failed to register dma async device\n");
		goto err_bam_channel_exit;
	}

	ret = of_dma_controller_register(pdev->dev.of_node, bam_dma_xlate,
					&bdev->common);
	if (ret)
		goto err_unregister_dma;

	pm_runtime_irq_safe(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev, BAM_DMA_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;

err_unregister_dma:
	dma_async_device_unregister(&bdev->common);
err_bam_channel_exit:
	for (i = 0; i < bdev->num_channels; i++)
		tasklet_kill(&bdev->channels[i].vc.task);
err_tasklet_kill:
	tasklet_kill(&bdev->task);
err_disable_clk:
	clk_disable_unprepare(bdev->bamclk);

	return ret;
}

static void bam_dma_remove(struct platform_device *pdev)
{
	struct bam_device *bdev = platform_get_drvdata(pdev);
	u32 i;

	pm_runtime_force_suspend(&pdev->dev);

	if (bdev->polling)
		hrtimer_cancel(&bdev->poll_timer);

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&bdev->common);

	/* mask all interrupts for this execution environment */
	if (!bdev->polling)
		writel_relaxed(0, bam_addr(bdev, 0,  BAM_IRQ_SRCS_MSK_EE));

	devm_free_irq(bdev->dev, bdev->irq, bdev);

	for (i = 0; i < bdev->num_channels; i++) {
		bam_dma_terminate_all(&bdev->channels[i].vc.chan);
		tasklet_kill(&bdev->channels[i].vc.task);

		if (!bdev->channels[i].fifo_virt)
			continue;

		dma_free_coherent(bdev->dev, BAM_DESC_FIFO_SIZE,
			    bdev->channels[i].fifo_virt,
			    bdev->channels[i].fifo_phys);
	}

	tasklet_kill(&bdev->task);

	clk_disable_unprepare(bdev->bamclk);
}

static int __maybe_unused bam_dma_runtime_suspend(struct device *dev)
{
	struct bam_device *bdev = dev_get_drvdata(dev);

	clk_disable(bdev->bamclk);

	return 0;
}

static int __maybe_unused bam_dma_runtime_resume(struct device *dev)
{
	struct bam_device *bdev = dev_get_drvdata(dev);
	int ret;

	ret = clk_enable(bdev->bamclk);
	if (ret < 0) {
		dev_err(dev, "clk_enable failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int __maybe_unused bam_dma_suspend(struct device *dev)
{
	struct bam_device *bdev = dev_get_drvdata(dev);

	pm_runtime_force_suspend(dev);
	clk_unprepare(bdev->bamclk);

	return 0;
}

static int __maybe_unused bam_dma_resume(struct device *dev)
{
	struct bam_device *bdev = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare(bdev->bamclk);
	if (ret)
		return ret;

	pm_runtime_force_resume(dev);

	return 0;
}

static const struct dev_pm_ops bam_dma_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(bam_dma_suspend, bam_dma_resume)
	SET_RUNTIME_PM_OPS(bam_dma_runtime_suspend, bam_dma_runtime_resume,
				NULL)
};

static struct platform_driver bam_dma_driver = {
	.probe = bam_dma_probe,
	.remove = bam_dma_remove,
	.driver = {
		.name = "bam-dma-engine",
		.pm = &bam_dma_pm_ops,
		.of_match_table = bam_of_match,
	},
};

module_platform_driver(bam_dma_driver);

MODULE_AUTHOR("Andy Gross <agross@codeaurora.org>");
MODULE_DESCRIPTION("QCOM BAM DMA engine driver");
MODULE_LICENSE("GPL v2");
