// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm BAM-DMUX WWAN network driver
 * Copyright (c) 2020, Stephan Gerhold <stephan@gerhold.net>
 */

#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <net/pkt_sched.h>

#define BAM_DMUX_BUFFER_SIZE		SZ_2K
#define BAM_DMUX_HDR_SIZE		sizeof(struct bam_dmux_hdr)
#define BAM_DMUX_MAX_DATA_SIZE		(BAM_DMUX_BUFFER_SIZE - BAM_DMUX_HDR_SIZE)
#define BAM_DMUX_NUM_SKB		32

#define BAM_DMUX_HDR_MAGIC		0x33fc

#define BAM_DMUX_AUTOSUSPEND_DELAY	1000
#define BAM_DMUX_REMOTE_TIMEOUT		msecs_to_jiffies(2000)

/*
 * Module parameter: when true (default) the driver proactively injects a
 * CMD_OPEN for channel 0 inside pc_irq right after the power handshake.
 * Set to false to disable auto-injection and trigger it manually from
 * userspace via debugfs (bam_dmux/trigger_inject_open) — useful to
 * separate timing issues from protocol issues.
 */
static bool auto_inject_cmd_open = true;
module_param(auto_inject_cmd_open, bool, 0644);
MODULE_PARM_DESC(auto_inject_cmd_open,
		 "Auto-inject CMD_OPEN for ch0 in pc_irq (default: true)");

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

struct bam_dmux_skb_dma {
	struct bam_dmux *dmux;
	struct sk_buff *skb;
	dma_addr_t addr;
};

struct bam_dmux {
	struct device *dev;

	int pc_irq;
	bool pc_state, pc_ack_state;
	struct qcom_smem_state *pc, *pc_ack;
	u32 pc_mask, pc_ack_mask;
	wait_queue_head_t pc_wait;
	struct completion pc_ack_completion;

	struct dma_chan *rx, *tx;
	struct bam_dmux_skb_dma rx_skbs[BAM_DMUX_NUM_SKB];
	struct bam_dmux_skb_dma tx_skbs[BAM_DMUX_NUM_SKB];
	spinlock_t tx_lock; /* Protect tx_skbs, tx_next_skb */
	unsigned int tx_next_skb;
	atomic_long_t tx_deferred_skb;
	struct work_struct tx_wakeup_work;

	struct delayed_work boot_work;

	/* debugfs */
	struct dentry *dbg_dir;

	DECLARE_BITMAP(remote_channels, BAM_DMUX_NUM_CH);
	struct work_struct register_netdev_work;
	struct net_device *netdevs[BAM_DMUX_NUM_CH];
};

struct bam_dmux_netdev {
	struct bam_dmux *dmux;
	u8 ch;
};

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

static bool bam_dmux_skb_dma_map(struct bam_dmux_skb_dma *skb_dma,
				 enum dma_data_direction dir)
{
	struct device *dev = skb_dma->dmux->dev;

	skb_dma->addr = dma_map_single(dev, skb_dma->skb->data, skb_dma->skb->len, dir);
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
	dma_unmap_single(skb_dma->dmux->dev, skb_dma->addr, skb_dma->skb->len, dir);
	skb_dma->addr = 0;
}

static void bam_dmux_tx_wake_queues(struct bam_dmux *dmux)
{
	int i;

	dev_dbg(dmux->dev, "wake queues\n");

	for (i = 0; i < BAM_DMUX_NUM_CH; ++i) {
		struct net_device *netdev = dmux->netdevs[i];

		if (netdev && netif_running(netdev))
			netif_wake_queue(netdev);
	}
}

static void bam_dmux_tx_stop_queues(struct bam_dmux *dmux)
{
	int i;

	dev_dbg(dmux->dev, "stop queues\n");

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

static void bam_dmux_tx_callback(void *data)
{
	struct bam_dmux_skb_dma *skb_dma = data;
	struct sk_buff *skb = skb_dma->skb;

	bam_dmux_tx_done(skb_dma);
	dev_consume_skb_any(skb);
}

static bool bam_dmux_skb_dma_submit_tx(struct bam_dmux_skb_dma *skb_dma)
{
	struct bam_dmux *dmux = skb_dma->dmux;
	struct dma_async_tx_descriptor *desc;

	desc = dmaengine_prep_slave_single(dmux->tx, skb_dma->addr,
					   skb_dma->skb->len, DMA_MEM_TO_DEV,
					   DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(dmux->dev, "Failed to prepare TX DMA buffer\n");
		return false;
	}

	desc->callback = bam_dmux_tx_callback;
	desc->callback_param = skb_dma;
	desc->cookie = dmaengine_submit(desc);
	return true;
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

/*
 * bam_dmux_inject_tx_callback() - completion callback for boot-time
 * injected control commands (e.g. proactive CMD_OPEN).
 *
 * Unlike bam_dmux_tx_callback() which goes through bam_dmux_tx_done()
 * and calls pm_runtime_put_autosuspend(), this callback does NOT touch
 * pm_runtime, because the injection path never took a pm_runtime
 * reference (the modem is already powered via boot_work's direct
 * pc_vote() and the subsequent pc_irq handshake).
 */
static void bam_dmux_inject_tx_callback(void *data)
{
	struct bam_dmux_skb_dma *skb_dma = data;
	struct bam_dmux *dmux = skb_dma->dmux;
	struct sk_buff *skb = skb_dma->skb;
	unsigned long flags;

	if (skb_dma->addr)
		bam_dmux_skb_dma_unmap(skb_dma, DMA_TO_DEVICE);

	spin_lock_irqsave(&dmux->tx_lock, flags);
	skb_dma->skb = NULL;
	if (skb_dma == &dmux->tx_skbs[dmux->tx_next_skb % BAM_DMUX_NUM_SKB])
		bam_dmux_tx_wake_queues(dmux);
	spin_unlock_irqrestore(&dmux->tx_lock, flags);

	dev_info(dmux->dev, "inject_tx: CMD_OPEN delivered to modem\n");
	dev_consume_skb_any(skb);
}

/*
 * bam_dmux_inject_cmd_open() - send a CMD_OPEN to the modem for channel @ch
 * directly from the boot/pc_irq path, bypassing pm_runtime.
 *
 * Background: the mainline qcom_bam_dmux driver is purely reactive — it
 * registers a netdev only after the modem sends an unsolicited CMD_OPEN.
 * The Quectel MDM9607 modem firmware does NOT initiate CMD_OPEN; instead
 * it waits for APPS to open the channel first (matching the downstream
 * msm_bam_dmux_open() flow in drivers/soc/qcom/bam_dmux.c).  This causes
 * a chicken-and-egg deadlock: modem won't push data on pipe 5 because
 * APPS never opened a channel; APPS can't open a channel because no
 * netdev exists; no netdev exists because modem never sent CMD_OPEN.
 *
 * Break the cycle by sending CMD_OPEN for channel 0 right after the
 * power-state handshake completes.  If the modem responds with its own
 * CMD_OPEN (the normal protocol acknowledgement), bam_dmux_cmd_open()
 * schedules register_netdev_work and the channel becomes usable.
 *
 * We use a custom path (not bam_dmux_send_cmd) because send_cmd ties
 * into pm_runtime, and triggering autosuspend right after this command
 * would cause us to drop the PC vote and lose the modem connection
 * before userspace has a chance to take a reference on the netdev.
 */
static int bam_dmux_inject_cmd_open(struct bam_dmux *dmux, u8 ch)
{
	struct bam_dmux_skb_dma *skb_dma;
	struct dma_async_tx_descriptor *desc;
	struct bam_dmux_hdr *hdr;
	struct sk_buff *skb;
	unsigned long flags;

	skb = alloc_skb(sizeof(*hdr), GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hdr = skb_put_zero(skb, sizeof(*hdr));
	hdr->magic = BAM_DMUX_HDR_MAGIC;
	hdr->cmd = BAM_DMUX_CMD_OPEN;
	hdr->ch = ch;

	/* Reserve TX slot without taking a pm_runtime reference */
	spin_lock_irqsave(&dmux->tx_lock, flags);
	skb_dma = &dmux->tx_skbs[dmux->tx_next_skb % BAM_DMUX_NUM_SKB];
	if (skb_dma->skb) {
		spin_unlock_irqrestore(&dmux->tx_lock, flags);
		dev_err(dmux->dev, "inject_cmd_open: TX queue full\n");
		dev_kfree_skb(skb);
		return -EAGAIN;
	}
	skb_dma->skb = skb;
	dmux->tx_next_skb++;
	spin_unlock_irqrestore(&dmux->tx_lock, flags);

	if (!bam_dmux_skb_dma_map(skb_dma, DMA_TO_DEVICE))
		goto fail;

	desc = dmaengine_prep_slave_single(dmux->tx, skb_dma->addr, skb->len,
					   DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(dmux->dev, "inject_cmd_open: prep_slave_single failed\n");
		bam_dmux_skb_dma_unmap(skb_dma, DMA_TO_DEVICE);
		goto fail;
	}

	desc->callback = bam_dmux_inject_tx_callback;
	desc->callback_param = skb_dma;
	desc->cookie = dmaengine_submit(desc);
	dma_async_issue_pending(dmux->tx);

	dev_info(dmux->dev, "inject_cmd_open: submitted CMD_OPEN for ch %u\n", ch);
	return 0;

fail:
	spin_lock_irqsave(&dmux->tx_lock, flags);
	skb_dma->skb = NULL;
	spin_unlock_irqrestore(&dmux->tx_lock, flags);
	dev_kfree_skb(skb);
	return -EIO;
}

/*
 * debugfs: write a channel id (0..BAM_DMUX_NUM_CH-1) to trigger an on-demand
 * CMD_OPEN injection. Useful when auto_inject_cmd_open is disabled so the
 * timing of injection vs modem-side BAM A2 power-on / DPM_OPEN_PORT can be
 * controlled from userspace.
 */
static ssize_t bam_dmux_dbg_inject_open_write(struct file *file,
					      const char __user *ubuf,
					      size_t len, loff_t *ppos)
{
	struct bam_dmux *dmux = file->private_data;
	char buf[16];
	unsigned int ch;
	int ret;

	if (len == 0 || len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';

	if (kstrtouint(buf, 0, &ch))
		return -EINVAL;
	if (ch >= BAM_DMUX_NUM_CH)
		return -ERANGE;

	if (!dmux->pc_state) {
		dev_warn(dmux->dev, "dbg inject_open: BAM A2 not powered\n");
		return -ENETDOWN;
	}

	dev_info(dmux->dev, "dbg inject_open: userspace requested ch %u\n", ch);
	ret = bam_dmux_inject_cmd_open(dmux, (u8)ch);
	return ret ? ret : len;
}

static const struct file_operations bam_dmux_dbg_inject_open_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = bam_dmux_dbg_inject_open_write,
	.llseek = noop_llseek,
};

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

	if (!bam_dmux_skb_dma_map(skb_dma, DMA_TO_DEVICE)) {
		ret = -ENOMEM;
		goto tx_fail;
	}

	if (!bam_dmux_skb_dma_submit_tx(skb_dma)) {
		ret = -EIO;
		goto tx_fail;
	}

	dma_async_issue_pending(dmux->tx);
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

	if (!bam_dmux_skb_dma_map(skb_dma, DMA_TO_DEVICE))
		goto drop;

	if (active <= 0) {
		/* Cannot sleep here so mark skb for wakeup handler and return */
		if (!atomic_long_fetch_or(BIT(skb_dma - dmux->tx_skbs),
					  &dmux->tx_deferred_skb))
			queue_pm_work(&dmux->tx_wakeup_work);
		return NETDEV_TX_OK;
	}

	if (!bam_dmux_skb_dma_submit_tx(skb_dma))
		goto drop;

	dma_async_issue_pending(dmux->tx);
	return NETDEV_TX_OK;

drop:
	bam_dmux_tx_done(skb_dma);
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static void bam_dmux_tx_wakeup_work(struct work_struct *work)
{
	struct bam_dmux *dmux = container_of(work, struct bam_dmux, tx_wakeup_work);
	unsigned long pending;
	int ret, i;

	ret = pm_runtime_resume_and_get(dmux->dev);
	if (ret < 0) {
		dev_err(dmux->dev, "Failed to resume: %d\n", ret);
		return;
	}

	pending = atomic_long_xchg(&dmux->tx_deferred_skb, 0);
	if (!pending)
		goto out;

	dev_dbg(dmux->dev, "pending skbs after wakeup: %#lx\n", pending);
	for_each_set_bit(i, &pending, BAM_DMUX_NUM_SKB) {
		bam_dmux_skb_dma_submit_tx(&dmux->tx_skbs[i]);
	}
	dma_async_issue_pending(dmux->tx);

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
	dev->needed_tailroom = sizeof(u32); /* word-aligned */
	dev->tx_queue_len = DEFAULT_TX_QUEUE_LEN;

	/* This perm addr will be used as interface identifier by IPv6 */
	dev->addr_assign_type = NET_ADDR_RANDOM;
	eth_random_addr(dev->perm_addr);
}

static void bam_dmux_register_netdev_work(struct work_struct *work)
{
	struct bam_dmux *dmux = container_of(work, struct bam_dmux, register_netdev_work);
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
			dev_err(dmux->dev, "Failed to register netdev for channel %u: %d\n",
				ch, ret);
			free_netdev(netdev);
			return;
		}

		dmux->netdevs[ch] = netdev;
	}
}

static void bam_dmux_rx_callback(void *data);

static bool bam_dmux_skb_dma_submit_rx(struct bam_dmux_skb_dma *skb_dma)
{
	struct bam_dmux *dmux = skb_dma->dmux;
	struct dma_async_tx_descriptor *desc;

	desc = dmaengine_prep_slave_single(dmux->rx, skb_dma->addr,
					   skb_dma->skb->len, DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(dmux->dev, "Failed to prepare RX DMA buffer\n");
		return false;
	}

	desc->callback = bam_dmux_rx_callback;
	desc->callback_param = skb_dma;
	desc->cookie = dmaengine_submit(desc);
	return true;
}

static bool bam_dmux_skb_dma_queue_rx(struct bam_dmux_skb_dma *skb_dma, gfp_t gfp)
{
	if (!skb_dma->skb) {
		skb_dma->skb = __netdev_alloc_skb(NULL, BAM_DMUX_BUFFER_SIZE, gfp);
		if (!skb_dma->skb)
			return false;
		skb_put(skb_dma->skb, BAM_DMUX_BUFFER_SIZE);
	}

	return bam_dmux_skb_dma_map(skb_dma, DMA_FROM_DEVICE) &&
	       bam_dmux_skb_dma_submit_rx(skb_dma);
}

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
		dev_err(dmux->dev, "Data larger than buffer? (%u > %u)\n",
			hdr->len, (u16)BAM_DMUX_MAX_DATA_SIZE);
		return;
	}

	skb_dma->skb = NULL; /* Hand over to network stack */

	skb_pull(skb, sizeof(*hdr));
	skb_trim(skb, hdr->len);
	skb->dev = netdev;

	/* Only Raw-IP/QMAP is supported by this driver */
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

	dev_dbg(dmux->dev, "open channel: %u\n", hdr->ch);

	if (__test_and_set_bit(hdr->ch, dmux->remote_channels)) {
		dev_warn(dmux->dev, "Channel already open: %u\n", hdr->ch);
		return;
	}

	if (netdev) {
		netif_device_attach(netdev);
	} else {
		/* Cannot sleep here, schedule work to register the netdev */
		schedule_work(&dmux->register_netdev_work);
	}
}

static void bam_dmux_cmd_close(struct bam_dmux *dmux, struct bam_dmux_hdr *hdr)
{
	struct net_device *netdev = dmux->netdevs[hdr->ch];

	dev_dbg(dmux->dev, "close channel: %u\n", hdr->ch);

	if (!__test_and_clear_bit(hdr->ch, dmux->remote_channels)) {
		dev_err(dmux->dev, "Channel not open: %u\n", hdr->ch);
		return;
	}

	if (netdev)
		netif_device_detach(netdev);
}

static void bam_dmux_rx_callback(void *data)
{
	struct bam_dmux_skb_dma *skb_dma = data;
	struct bam_dmux *dmux = skb_dma->dmux;
	struct sk_buff *skb = skb_dma->skb;
	struct bam_dmux_hdr *hdr = (struct bam_dmux_hdr *)skb->data;

	bam_dmux_skb_dma_unmap(skb_dma, DMA_FROM_DEVICE);

	if (hdr->magic != BAM_DMUX_HDR_MAGIC) {
		dev_err(dmux->dev, "Invalid magic in header: %#x\n", hdr->magic);
		goto out;
	}

	if (hdr->ch >= BAM_DMUX_NUM_CH) {
		dev_dbg(dmux->dev, "Unsupported channel: %u\n", hdr->ch);
		goto out;
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
		dev_err(dmux->dev, "Unsupported command %u on channel %u\n",
			hdr->cmd, hdr->ch);
		break;
	}

out:
	if (bam_dmux_skb_dma_queue_rx(skb_dma, GFP_ATOMIC))
		dma_async_issue_pending(dmux->rx);
}

static bool bam_dmux_power_on(struct bam_dmux *dmux)
{
	struct device *dev = dmux->dev;
	struct dma_slave_config dma_rx_conf = {
		.direction = DMA_DEV_TO_MEM,
		.src_maxburst = BAM_DMUX_BUFFER_SIZE,
	};
	int i;

	/* Already powered on (e.g. pre-initialized from boot_work) */
	if (dmux->rx_skbs[0].addr)
		return true;

	if (!dmux->rx) {
		dmux->rx = dma_request_chan(dev, "rx");
		if (IS_ERR(dmux->rx)) {
			dev_err(dev, "Failed to request RX DMA channel: %pe\n", dmux->rx);
			dmux->rx = NULL;
			return false;
		}
		dmaengine_slave_config(dmux->rx, &dma_rx_conf);
	}

	if (!dmux->tx) {
		dmux->tx = dma_request_chan(dev, "tx");
		if (IS_ERR(dmux->tx)) {
			dev_err(dev, "Failed to request TX DMA channel: %pe\n",
				dmux->tx);
			dmux->tx = NULL;
			return false;
		}
	}

	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		if (!bam_dmux_skb_dma_queue_rx(&dmux->rx_skbs[i], GFP_KERNEL))
			return false;
	}
	dma_async_issue_pending(dmux->rx);

	return true;
}

static void bam_dmux_free_skbs(struct bam_dmux_skb_dma skbs[],
			       enum dma_data_direction dir)
{
	int i;

	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		struct bam_dmux_skb_dma *skb_dma = &skbs[i];

		if (skb_dma->addr)
			bam_dmux_skb_dma_unmap(skb_dma, dir);
		if (skb_dma->skb) {
			dev_kfree_skb(skb_dma->skb);
			skb_dma->skb = NULL;
		}
	}
}

static void bam_dmux_power_off(struct bam_dmux *dmux)
{
	if (dmux->tx) {
		dmaengine_terminate_sync(dmux->tx);
		dma_release_channel(dmux->tx);
		dmux->tx = NULL;
	}

	if (dmux->rx) {
		dmaengine_terminate_sync(dmux->rx);
		dma_release_channel(dmux->rx);
		dmux->rx = NULL;
	}

	bam_dmux_free_skbs(dmux->rx_skbs, DMA_FROM_DEVICE);
}

static irqreturn_t bam_dmux_pc_irq(int irq, void *data)
{
	struct bam_dmux *dmux = data;
	bool new_state = !dmux->pc_state;

	dev_info(dmux->dev, "pc_irq: new_state=%d\n", new_state);
	cancel_delayed_work(&dmux->boot_work);

	if (new_state) {
		if (bam_dmux_power_on(dmux)) {
			bam_dmux_pc_ack(dmux);
			/*
			 * Modem-initiated power-up: the MDM9607 modem firmware
			 * raises pc to ask apps to bring BAM up, but if apps
			 * does not also assert its own pc vote, the modem
			 * powers BAM down again after a short timeout
			 * (~2 s) before it ever sends the initial CMD_OPEN.
			 *
			 * Assert our pc vote so the modem keeps BAM up long
			 * enough to complete the open handshake.  The vote is
			 * cleared in the down path below.
			 */
			bam_dmux_pc_vote(dmux, true);
			/*
			 * Kick the modem: send a proactive CMD_OPEN for
			 * channel 0.  See bam_dmux_inject_cmd_open() for
			 * rationale.  Failure here is non-fatal — userspace
			 * can still retry via netdev open once a netdev
			 * exists.
			 */
			if (auto_inject_cmd_open)
				bam_dmux_inject_cmd_open(dmux, BAM_DMUX_CH_DATA_0);
			else
				dev_info(dmux->dev, "pc_irq: auto-inject disabled, waiting for userspace trigger\n");
		} else {
			dev_err(dmux->dev, "power_on failed\n");
			bam_dmux_power_off(dmux);
		}
	} else {
		bam_dmux_power_off(dmux);
		bam_dmux_pc_vote(dmux, false);
		bam_dmux_pc_ack(dmux);
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

	dev_dbg(dev, "runtime suspend\n");
	bam_dmux_pc_vote(dmux, false);

	return 0;
}

static int __maybe_unused bam_dmux_runtime_resume(struct device *dev)
{
	struct bam_dmux *dmux = dev_get_drvdata(dev);

	dev_dbg(dev, "runtime resume\n");

	/* Wait until previous power down was acked */
	if (!wait_for_completion_timeout(&dmux->pc_ack_completion,
					 BAM_DMUX_REMOTE_TIMEOUT))
		return -ETIMEDOUT;

	/* Vote for power state */
	bam_dmux_pc_vote(dmux, true);

	/* Wait for ack */
	if (!wait_for_completion_timeout(&dmux->pc_ack_completion,
					 BAM_DMUX_REMOTE_TIMEOUT)) {
		bam_dmux_pc_vote(dmux, false);
		return -ETIMEDOUT;
	}

	/* Wait until we're up */
	if (!wait_event_timeout(dmux->pc_wait, dmux->pc_state,
				BAM_DMUX_REMOTE_TIMEOUT)) {
		bam_dmux_pc_vote(dmux, false);
		return -ETIMEDOUT;
	}

	/* Ensure that we actually initialized successfully */
	if (!dmux->rx) {
		bam_dmux_pc_vote(dmux, false);
		return -ENXIO;
	}

	/* Request TX channel if necessary */
	if (dmux->tx)
		return 0;

	dmux->tx = dma_request_chan(dev, "tx");
	if (IS_ERR(dmux->tx)) {
		dev_err(dev, "Failed to request TX DMA channel: %pe\n", dmux->tx);
		dmux->tx = NULL;
		bam_dmux_runtime_suspend(dev);
		return -ENXIO;
	}

	return 0;
}

/**
 * bam_dmux_boot_work() - kick-start BAM-DMUX on firmwares that need APPS to
 *                        initiate the power handshake.
 *
 * Some modem firmwares (e.g. Quectel OCPU variants) do not assert
 * A2_POWER_CONTROL until APPS requests first.  This work fires when the
 * modem signals readiness (SMDINIT via remote-ready IRQ), then triggers the
 * runtime resume path which sets APPS A2_POWER_CONTROL and waits for
 * the modem's response.
 */
static void bam_dmux_boot_work_fn(struct work_struct *work)
{
	struct bam_dmux *dmux = container_of(work, struct bam_dmux, boot_work.work);
	struct device *dev = dmux->dev;

	/* If the modem already initiated, nothing to do */
	if (dmux->pc_state)
		return;

	/*
	 * Request power-on by asserting APPS A2_POWER_CONTROL (bit 1) FIRST.
	 *
	 * Per the modem firmware A2 power state machine (confirmed by
	 * decompilation of MDM9607 modem fw, see scratch/references/modem-fw/
	 * decomp/FINDINGS.md): the modem only turns on the A2 hardware AFTER
	 * APPS sets the SMSM_A2_POWER_CONTROL bit.  The modem then sets its
	 * own PWR_CTRL bit + the ACK bit, which fires pc_irq.  pc_irq's
	 * handler calls bam_dmux_power_on() at the correct moment (after A2
	 * is powered up and the modem has programmed its side of the BAM
	 * pipes).  Writing BAM/pipe registers before this point hits A2
	 * hardware that is still powered down, leaving the BAM and pipe
	 * state inconsistent with what the modem later programs.
	 *
	 * Do NOT use pm_runtime here: autosuspend would clear bit 1 after
	 * 1 second, before the modem has time to send CMD_OPEN.  Vote
	 * directly; power-down happens only when the modem clears its bit 1.
	 */
	dev_info(dev, "modem did not initiate, requesting power on\n");
	bam_dmux_pc_vote(dmux, true);
}

/**
 * bam_dmux_remote_ready_irq - Modem has set SMDINIT, indicating it is ready
 *
 * This replaces the fixed 30-second delay with a deterministic trigger.
 * The downstream kernel used smsm_state_cb_register() to watch for this
 * same transition.
 */
static irqreturn_t bam_dmux_remote_ready_irq(int irq, void *data)
{
	struct bam_dmux *dmux = data;

	dev_info(dmux->dev, "remote ready (SMDINIT), scheduling power request\n");

	/* Use a short delay to let SMD channels finish opening */
	schedule_delayed_work(&dmux->boot_work, msecs_to_jiffies(500));

	return IRQ_HANDLED;
}

static int bam_dmux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bam_dmux *dmux;
	int ret, pc_ack_irq, i;
	unsigned int bit;

	dmux = devm_kzalloc(dev, sizeof(*dmux), GFP_KERNEL);
	if (!dmux)
		return -ENOMEM;

	dmux->dev = dev;
	platform_set_drvdata(pdev, dmux);

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

	for (i = 0; i < BAM_DMUX_NUM_SKB; i++) {
		dmux->rx_skbs[i].dmux = dmux;
		dmux->tx_skbs[i].dmux = dmux;
	}

	/* Runtime PM manages our own power vote.
	 * Note that the RX path may be active even if we are runtime suspended,
	 * since it is controlled by the remote side.
	 */
	pm_runtime_set_autosuspend_delay(dev, BAM_DMUX_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	ret = devm_request_threaded_irq(dev, pc_ack_irq, NULL, bam_dmux_pc_ack_irq,
					IRQF_ONESHOT, NULL, dmux);
	if (ret)
		goto err_disable_pm;

	ret = devm_request_threaded_irq(dev, dmux->pc_irq, NULL, bam_dmux_pc_irq,
					IRQF_ONESHOT, NULL, dmux);
	if (ret)
		goto err_disable_pm;

	ret = irq_get_irqchip_state(dmux->pc_irq, IRQCHIP_STATE_LINE_LEVEL,
				    &dmux->pc_state);
	if (ret)
		goto err_disable_pm;

	/* Register remote-ready IRQ (modem SMDINIT) for APPS-initiates mode */
	{
		int remote_ready_irq = platform_get_irq_byname(pdev, "remote-ready");

		if (remote_ready_irq > 0) {
			ret = devm_request_threaded_irq(dev, remote_ready_irq, NULL,
							bam_dmux_remote_ready_irq,
							IRQF_ONESHOT, NULL, dmux);
			if (ret)
				dev_warn(dev, "failed to request remote-ready IRQ: %d\n", ret);
		}
	}

	dev_info(dev, "probe complete: pc_state=%d pc_irq=%d\n",
		 dmux->pc_state, dmux->pc_irq);

	/* debugfs: expose on-demand CMD_OPEN injection trigger */
	dmux->dbg_dir = debugfs_create_dir("bam_dmux", NULL);
	if (!IS_ERR_OR_NULL(dmux->dbg_dir))
		debugfs_create_file("trigger_inject_open", 0200, dmux->dbg_dir,
				    dmux, &bam_dmux_dbg_inject_open_fops);

	/* Check if remote finished initialization before us */
	if (dmux->pc_state) {
		dev_info(dev, "remote already powered on, initializing\n");
		if (bam_dmux_power_on(dmux))
			bam_dmux_pc_ack(dmux);
		else
			bam_dmux_power_off(dmux);
	}

	return 0;

err_disable_pm:
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);
	return ret;
}

static void bam_dmux_remove(struct platform_device *pdev)
{
	struct bam_dmux *dmux = platform_get_drvdata(pdev);
	struct device *dev = dmux->dev;
	LIST_HEAD(list);
	int i;

	cancel_delayed_work_sync(&dmux->boot_work);

	debugfs_remove_recursive(dmux->dbg_dir);

	/* Unregister network interfaces */
	cancel_work_sync(&dmux->register_netdev_work);
	rtnl_lock();
	for (i = 0; i < BAM_DMUX_NUM_CH; ++i)
		if (dmux->netdevs[i])
			unregister_netdevice_queue(dmux->netdevs[i], &list);
	unregister_netdevice_many(&list);
	rtnl_unlock();
	cancel_work_sync(&dmux->tx_wakeup_work);

	/* Drop our own power vote */
	pm_runtime_disable(dev);
	pm_runtime_dont_use_autosuspend(dev);
	bam_dmux_runtime_suspend(dev);
	pm_runtime_set_suspended(dev);

	/* Try to wait for remote side to drop power vote */
	if (!wait_event_timeout(dmux->pc_wait, !dmux->rx, BAM_DMUX_REMOTE_TIMEOUT))
		dev_err(dev, "Timed out waiting for remote side to suspend\n");

	/* Make sure everything is cleaned up before we return */
	disable_irq(dmux->pc_irq);
	bam_dmux_power_off(dmux);
	bam_dmux_free_skbs(dmux->tx_skbs, DMA_TO_DEVICE);
}

static const struct dev_pm_ops bam_dmux_pm_ops = {
	SET_RUNTIME_PM_OPS(bam_dmux_runtime_suspend, bam_dmux_runtime_resume, NULL)
};

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
		.pm = &bam_dmux_pm_ops,
		.of_match_table = bam_dmux_of_match,
	},
};
module_platform_driver(bam_dmux_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm BAM-DMUX WWAN Network Driver");
MODULE_AUTHOR("Stephan Gerhold <stephan@gerhold.net>");
