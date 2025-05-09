/*
 * Copyright (c) 2011-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/spinlock.h>
#include <linux/usb_bam.h>

#include "usb_gadget_xport.h"
#include "u_ether.h"
#include "u_rmnet.h"
#include "gadget_chips.h"

static unsigned int rmnet_dl_max_pkt_per_xfer = 7;
module_param(rmnet_dl_max_pkt_per_xfer, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(rmnet_dl_max_pkt_per_xfer,
	"Maximum packets per transfer for DL aggregation");

#define RMNET_NOTIFY_INTERVAL	5
#define RMNET_MAX_NOTIFY_SIZE	sizeof(struct usb_cdc_notification)

#define QUECTEL_MULTI_IP_PACKAGES
#define ACM_CTRL_DTR	(1 << 0)

/* TODO: use separate structures for data and
 * control paths
 */
struct f_rmnet {
	struct gether			gether_port;
	struct grmnet			port;
	int				ifc_id;
	u8				port_num;
	atomic_t			online;
	atomic_t			ctrl_online;
	struct usb_composite_dev	*cdev;

	spinlock_t			lock;

	/* usb eps*/
	struct usb_ep			*notify;
	struct usb_request		*notify_req;

	/* control info */
	struct list_head		cpkt_resp_q;
	unsigned long			notify_count;
	unsigned long			cpkts_len;
	const struct usb_endpoint_descriptor *in_ep_desc_backup;
	const struct usb_endpoint_descriptor *out_ep_desc_backup;
};

#ifdef QUECTEL_MULTI_IP_PACKAGES

#define USB_CDC_SET_MULTI_PACKAGE_COMMAND (0x5C)

extern unsigned int multi_package_max_len;;
extern unsigned int wait_for_package_timeout; //us
extern unsigned int package_max_count_in_queue;
extern unsigned int multi_package_enabled;
#endif


static unsigned int nr_rmnet_ports;
static unsigned int no_ctrl_smd_ports;
static unsigned int no_ctrl_qti_ports;
static unsigned int no_ctrl_hsic_ports;
static unsigned int no_ctrl_hsuart_ports;
static unsigned int no_data_bam_ports;
static unsigned int no_data_bam2bam_ports;
static unsigned int no_data_hsic_ports;
static unsigned int no_data_hsuart_ports;
static struct rmnet_ports {
	enum transport_type		data_xport;
	enum transport_type		ctrl_xport;
	unsigned			data_xport_num;
	unsigned			ctrl_xport_num;
	unsigned			port_num;
	struct f_rmnet			*port;
} rmnet_ports[NR_RMNET_PORTS];

static struct usb_interface_descriptor rmnet_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bNumEndpoints =	3,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceProtocol =	USB_CLASS_VENDOR_SPEC,
	/* .iInterface = DYNAMIC */
};

/* Full speed support */
static struct usb_endpoint_descriptor rmnet_fs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(RMNET_MAX_NOTIFY_SIZE),
	.bInterval =		1 << RMNET_NOTIFY_INTERVAL,
};

static struct usb_endpoint_descriptor rmnet_fs_in_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(64),
};

static struct usb_endpoint_descriptor rmnet_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize   = __constant_cpu_to_le16(64),
};

static struct usb_descriptor_header *rmnet_fs_function[] = {
	(struct usb_descriptor_header *) &rmnet_interface_desc,
	(struct usb_descriptor_header *) &rmnet_fs_notify_desc,
	(struct usb_descriptor_header *) &rmnet_fs_in_desc,
	(struct usb_descriptor_header *) &rmnet_fs_out_desc,
	NULL,
};

/* High speed support */
static struct usb_endpoint_descriptor rmnet_hs_notify_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(RMNET_MAX_NOTIFY_SIZE),
	.bInterval =		RMNET_NOTIFY_INTERVAL + 4,
};

static struct usb_endpoint_descriptor rmnet_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor rmnet_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *rmnet_hs_function[] = {
	(struct usb_descriptor_header *) &rmnet_interface_desc,
	(struct usb_descriptor_header *) &rmnet_hs_notify_desc,
	(struct usb_descriptor_header *) &rmnet_hs_in_desc,
	(struct usb_descriptor_header *) &rmnet_hs_out_desc,
	NULL,
};

/* Super speed support */
static struct usb_endpoint_descriptor rmnet_ss_notify_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	__constant_cpu_to_le16(RMNET_MAX_NOTIFY_SIZE),
	.bInterval =		RMNET_NOTIFY_INTERVAL + 4,
};

static struct usb_ss_ep_comp_descriptor rmnet_ss_notify_comp_desc = {
	.bLength =		sizeof rmnet_ss_notify_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 3 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
	.wBytesPerInterval =	cpu_to_le16(RMNET_MAX_NOTIFY_SIZE),
};

static struct usb_endpoint_descriptor rmnet_ss_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor rmnet_ss_in_comp_desc = {
	.bLength =		sizeof rmnet_ss_in_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_endpoint_descriptor rmnet_ss_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor rmnet_ss_out_comp_desc = {
	.bLength =		sizeof rmnet_ss_out_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	/* the following 2 values can be tweaked if necessary */
	/* .bMaxBurst =		0, */
	/* .bmAttributes =	0, */
};

static struct usb_descriptor_header *rmnet_ss_function[] = {
	(struct usb_descriptor_header *) &rmnet_interface_desc,
	(struct usb_descriptor_header *) &rmnet_ss_notify_desc,
	(struct usb_descriptor_header *) &rmnet_ss_notify_comp_desc,
	(struct usb_descriptor_header *) &rmnet_ss_in_desc,
	(struct usb_descriptor_header *) &rmnet_ss_in_comp_desc,
	(struct usb_descriptor_header *) &rmnet_ss_out_desc,
	(struct usb_descriptor_header *) &rmnet_ss_out_comp_desc,
	NULL,
};

/* String descriptors */

static struct usb_string rmnet_string_defs[] = {
	[0].s = "RmNet",
	{  } /* end of list */
};

static struct usb_gadget_strings rmnet_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		rmnet_string_defs,
};

static struct usb_gadget_strings *rmnet_strings[] = {
	&rmnet_string_table,
	NULL,
};

static void frmnet_ctrl_response_available(struct f_rmnet *dev);

/* ------- misc functions --------------------*/

static inline struct f_rmnet *func_to_rmnet(struct usb_function *f)
{
	return container_of(f, struct f_rmnet, gether_port.func);
}

static inline struct f_rmnet *port_to_rmnet(struct grmnet *r)
{
	return container_of(r, struct f_rmnet, port);
}

static struct usb_request *
frmnet_alloc_req(struct usb_ep *ep, unsigned len, size_t extra_buf_alloc,
		gfp_t flags)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, flags);
	if (!req)
		return ERR_PTR(-ENOMEM);

	req->buf = kmalloc(len + extra_buf_alloc, flags);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return ERR_PTR(-ENOMEM);
	}

	req->length = len;

	return req;
}

void frmnet_free_req(struct usb_ep *ep, struct usb_request *req)
{
	kfree(req->buf);
	usb_ep_free_request(ep, req);
}

static struct rmnet_ctrl_pkt *rmnet_alloc_ctrl_pkt(unsigned len, gfp_t flags)
{
	struct rmnet_ctrl_pkt *pkt;

	pkt = kzalloc(sizeof(struct rmnet_ctrl_pkt), flags);
	if (!pkt)
		return ERR_PTR(-ENOMEM);

	pkt->buf = kmalloc(len, flags);
	if (!pkt->buf) {
		kfree(pkt);
		return ERR_PTR(-ENOMEM);
	}
	pkt->len = len;

	return pkt;
}

static void rmnet_free_ctrl_pkt(struct rmnet_ctrl_pkt *pkt)
{
	kfree(pkt->buf);
	kfree(pkt);
}

/* -------------------------------------------*/

static int rmnet_gport_setup(void)
{
	int	ret;
	int	port_idx;
	int	i;
	u8 base;

	pr_debug("%s: bam ports: %u bam2bam ports: %u data hsic ports: %u data hsuart ports: %u"
		" smd ports: %u ctrl hsic ports: %u ctrl hsuart ports: %u"
		" nr_rmnet_ports: %u\n",
		__func__, no_data_bam_ports, no_data_bam2bam_ports,
		no_data_hsic_ports, no_data_hsuart_ports, no_ctrl_smd_ports,
		no_ctrl_hsic_ports, no_ctrl_hsuart_ports, nr_rmnet_ports);

	if (no_data_bam_ports) {
		ret = gbam_setup(no_data_bam_ports);
		if (ret < 0)
			return ret;
	}

	if (no_data_bam2bam_ports) {
		ret = gbam2bam_setup(no_data_bam2bam_ports);
		if (ret < 0)
			return ret;
	}

	if (no_ctrl_smd_ports) {
		ret = gsmd_ctrl_setup(FRMNET_CTRL_CLIENT,
				no_ctrl_smd_ports, &base);
		if (ret)
			return ret;
		for (i = 0; i < nr_rmnet_ports; i++)
			if (rmnet_ports[i].port)
				rmnet_ports[i].port->port_num += base;
	}

	if (no_data_hsic_ports) {
		port_idx = ghsic_data_setup(no_data_hsic_ports,
				USB_GADGET_RMNET);
		if (port_idx < 0)
			return port_idx;
		for (i = 0; i < nr_rmnet_ports; i++) {
			if (rmnet_ports[i].data_xport ==
					USB_GADGET_XPORT_HSIC) {
				rmnet_ports[i].data_xport_num = port_idx;
				port_idx++;
			}
		}
	}

	if (no_ctrl_hsic_ports) {
		port_idx = ghsic_ctrl_setup(no_ctrl_hsic_ports,
				USB_GADGET_RMNET);
		if (port_idx < 0)
			return port_idx;
		for (i = 0; i < nr_rmnet_ports; i++) {
			if (rmnet_ports[i].ctrl_xport ==
					USB_GADGET_XPORT_HSIC) {
				rmnet_ports[i].ctrl_xport_num = port_idx;
				port_idx++;
			}
		}
	}

	return 0;
}

static int gport_rmnet_connect(struct f_rmnet *dev, unsigned intf)
{
	int			ret;
	unsigned		port_num;
	enum transport_type	cxport = rmnet_ports[dev->port_num].ctrl_xport;
	enum transport_type	dxport = rmnet_ports[dev->port_num].data_xport;
	int			src_connection_idx = 0, dst_connection_idx = 0;
	struct usb_gadget	*gadget = dev->cdev->gadget;
	enum usb_ctrl		usb_bam_type;
	void			*net;

	pr_debug("%s: ctrl xport: %s data xport: %s dev: %pK portno: %d\n",
			__func__, xport_to_str(cxport), xport_to_str(dxport),
			dev, dev->port_num);

	port_num = rmnet_ports[dev->port_num].ctrl_xport_num;
	switch (cxport) {
	case USB_GADGET_XPORT_SMD:
		ret = gsmd_ctrl_connect(&dev->port, port_num);
		if (ret) {
			pr_err("%s: gsmd_ctrl_connect failed: err:%d\n",
					__func__, ret);
			return ret;
		}
		break;
	case USB_GADGET_XPORT_QTI:
		ret = gqti_ctrl_connect(&dev->port, port_num, dev->ifc_id,
						dxport, USB_GADGET_RMNET);
		if (ret) {
			pr_err("%s: gqti_ctrl_connect failed: err:%d\n",
					__func__, ret);
			return ret;
		}
		break;
	case USB_GADGET_XPORT_HSIC:
		ret = ghsic_ctrl_connect(&dev->port, port_num);
		if (ret) {
			pr_err("%s: ghsic_ctrl_connect failed: err:%d\n",
					__func__, ret);
			return ret;
		}
		break;
	case USB_GADGET_XPORT_NONE:
		break;
	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				xport_to_str(cxport));
		return -ENODEV;
	}

	port_num = rmnet_ports[dev->port_num].data_xport_num;

	switch (dxport) {
	case USB_GADGET_XPORT_BAM_DMUX:
		ret = gbam_connect(&dev->port, port_num,
			dxport, src_connection_idx, dst_connection_idx);
		if (ret) {
			pr_err("%s: gbam_connect failed: err:%d\n",
				__func__, ret);
			gsmd_ctrl_disconnect(&dev->port, port_num);
			return ret;
		}
		break;
	case USB_GADGET_XPORT_BAM2BAM_IPA:
		usb_bam_type = usb_bam_get_bam_type(gadget->name);
		src_connection_idx = usb_bam_get_connection_idx(usb_bam_type,
			IPA_P_BAM, USB_TO_PEER_PERIPHERAL, USB_BAM_DEVICE,
			port_num);
		dst_connection_idx = usb_bam_get_connection_idx(usb_bam_type,
			IPA_P_BAM, PEER_PERIPHERAL_TO_USB, USB_BAM_DEVICE,
			port_num);
		if (dst_connection_idx < 0 || src_connection_idx < 0) {
			pr_err("%s: usb_bam_get_connection_idx failed\n",
				__func__);
			gsmd_ctrl_disconnect(&dev->port, port_num);
			return -EINVAL;
		}
		ret = gbam_connect(&dev->port, port_num,
			dxport, src_connection_idx, dst_connection_idx);
		if (ret) {
			pr_err("%s: gbam_connect failed: err:%d\n",
					__func__, ret);
			if (cxport == USB_GADGET_XPORT_QTI)
				gqti_ctrl_disconnect(&dev->port, port_num);
			else
				gsmd_ctrl_disconnect(&dev->port, port_num);
			return ret;
		}
		break;
	case USB_GADGET_XPORT_HSIC:
		ret = ghsic_data_connect(&dev->port, port_num);
		if (ret) {
			pr_err("%s: ghsic_data_connect failed: err:%d\n",
					__func__, ret);
			ghsic_ctrl_disconnect(&dev->port, port_num);
			return ret;
		}
		break;
	case USB_GADGET_XPORT_ETHER:
		gether_enable_sg(&dev->gether_port, true);
		net = gether_connect(&dev->gether_port);
		if (IS_ERR(net)) {
			pr_err("%s: gether_connect failed: err:%ld\n",
					__func__, PTR_ERR(net));
			if (cxport == USB_GADGET_XPORT_QTI)
				gqti_ctrl_disconnect(&dev->port, port_num);
			else
				gsmd_ctrl_disconnect(&dev->port, port_num);

			return PTR_ERR(net);
		}
		gether_update_dl_max_pkts_per_xfer(&dev->gether_port,
			rmnet_dl_max_pkt_per_xfer);
		gether_update_dl_max_xfer_size(&dev->gether_port, 16384);
		break;
	case USB_GADGET_XPORT_NONE:
		 break;
	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				xport_to_str(dxport));
		return -ENODEV;
	}

	return 0;
}

static int gport_rmnet_disconnect(struct f_rmnet *dev)
{
	unsigned		port_num;
	enum transport_type	cxport = rmnet_ports[dev->port_num].ctrl_xport;
	enum transport_type	dxport = rmnet_ports[dev->port_num].data_xport;

	pr_debug("%s: ctrl xport: %s data xport: %s dev: %pK portno: %d\n",
			__func__, xport_to_str(cxport), xport_to_str(dxport),
			dev, dev->port_num);

	port_num = rmnet_ports[dev->port_num].ctrl_xport_num;
	switch (cxport) {
	case USB_GADGET_XPORT_SMD:
		gsmd_ctrl_disconnect(&dev->port, port_num);
		break;
	case USB_GADGET_XPORT_QTI:
		gqti_ctrl_disconnect(&dev->port, port_num);
		break;
	case USB_GADGET_XPORT_HSIC:
		ghsic_ctrl_disconnect(&dev->port, port_num);
		break;
	case USB_GADGET_XPORT_NONE:
		break;
	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				xport_to_str(cxport));
		return -ENODEV;
	}

	port_num = rmnet_ports[dev->port_num].data_xport_num;
	switch (dxport) {
	case USB_GADGET_XPORT_BAM_DMUX:
	case USB_GADGET_XPORT_BAM2BAM_IPA:
		gbam_disconnect(&dev->port, port_num, dxport);
		break;
	case USB_GADGET_XPORT_HSIC:
		ghsic_data_disconnect(&dev->port, port_num);
		break;
	case USB_GADGET_XPORT_ETHER:
		gether_disconnect(&dev->gether_port);
		break;
	case USB_GADGET_XPORT_NONE:
		break;
	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				xport_to_str(dxport));
		return -ENODEV;
	}

	return 0;
}

static void frmnet_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_rmnet *dev = func_to_rmnet(f);
	enum transport_type	dxport = rmnet_ports[dev->port_num].data_xport;

	pr_debug("%s: portno:%d\n", __func__, dev->port_num);
	if (gadget_is_superspeed(c->cdev->gadget))
		usb_free_descriptors(f->ss_descriptors);
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->fs_descriptors);

	frmnet_free_req(dev->notify, dev->notify_req);
	if (dxport == USB_GADGET_XPORT_BAM2BAM_IPA) {
		gbam_data_flush_workqueue();
		c->cdev->gadget->bam2bam_func_enabled = false;
	}
	kfree(f->name);
}

static void frmnet_purge_responses(struct f_rmnet *dev)
{
	unsigned long flags;
	struct rmnet_ctrl_pkt *cpkt;

	pr_debug("%s: port#%d\n", __func__, dev->port_num);

	spin_lock_irqsave(&dev->lock, flags);
	while (!list_empty(&dev->cpkt_resp_q)) {
		cpkt = list_first_entry(&dev->cpkt_resp_q,
				struct rmnet_ctrl_pkt, list);

		list_del(&cpkt->list);
		rmnet_free_ctrl_pkt(cpkt);
	}
	dev->notify_count = 0;
	spin_unlock_irqrestore(&dev->lock, flags);
}

static void frmnet_suspend(struct usb_function *f)
{
	struct f_rmnet *dev = func_to_rmnet(f);
	unsigned		port_num;
	enum transport_type	dxport = rmnet_ports[dev->port_num].data_xport;
	bool			remote_wakeup_allowed;

	/* Check if function is already suspended in frmnet_func_suspend() */
	if (f->func_is_suspended) {
		pr_debug("%s: func already suspended!\n", __func__);
		return;
	}

	if (f->config->cdev->gadget->speed == USB_SPEED_SUPER)
		remote_wakeup_allowed = f->func_wakeup_allowed;
	else
		remote_wakeup_allowed = f->config->cdev->gadget->remote_wakeup;

	pr_debug("%s: data xport: %s dev: %pK portno: %d remote_wakeup: %d\n",
		__func__, xport_to_str(dxport),
		dev, dev->port_num, remote_wakeup_allowed);

	usb_ep_dequeue(dev->notify, dev->notify_req);
	frmnet_purge_responses(dev);

	port_num = rmnet_ports[dev->port_num].data_xport_num;
	switch (dxport) {
	case USB_GADGET_XPORT_BAM_DMUX:
		break;
	case USB_GADGET_XPORT_BAM2BAM_IPA:
		if (remote_wakeup_allowed) {
			gbam_suspend(&dev->port, port_num, dxport);
		} else {
			/*
			 * When remote wakeup is disabled, IPA is disconnected
			 * because it cannot send new data until the USB bus is
			 * resumed. Endpoint descriptors info is saved before it
			 * gets reset by the BAM disconnect API. This lets us
			 * restore this info when the USB bus is resumed.
			 */
			dev->in_ep_desc_backup  = dev->port.in->desc;
			dev->out_ep_desc_backup  = dev->port.out->desc;
			pr_debug("in_ep_desc_bkup = %pK, out_ep_desc_bkup = %pK",
			       dev->in_ep_desc_backup, dev->out_ep_desc_backup);
			pr_debug("%s(): Disconnecting\n", __func__);
			if (gadget_is_dwc3(f->config->cdev->gadget)) {
				msm_ep_unconfig(dev->port.out);
				msm_ep_unconfig(dev->port.in);
			}
			gport_rmnet_disconnect(dev);
		}
		break;
	case USB_GADGET_XPORT_HSIC:
		break;
	case USB_GADGET_XPORT_HSUART:
		break;
	case USB_GADGET_XPORT_ETHER:
		break;
	case USB_GADGET_XPORT_NONE:
		break;
	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				xport_to_str(dxport));
	}
}

static void frmnet_resume(struct usb_function *f)
{
	struct f_rmnet *dev = func_to_rmnet(f);
	unsigned		port_num;
	enum transport_type	dxport = rmnet_ports[dev->port_num].data_xport;
	int  ret;
	bool remote_wakeup_allowed;

	/*
	 * If the function is in USB3 Function Suspend state, resume is
	 * canceled. In this case resume is done by a Function Resume request.
	 */
	if ((f->config->cdev->gadget->speed == USB_SPEED_SUPER) &&
		f->func_is_suspended)
		return;

	if (f->config->cdev->gadget->speed == USB_SPEED_SUPER)
		remote_wakeup_allowed = f->func_wakeup_allowed;
	else
		remote_wakeup_allowed = f->config->cdev->gadget->remote_wakeup;

	pr_debug("%s: data xport: %s dev: %pK portno: %d remote_wakeup: %d\n",
		__func__, xport_to_str(dxport),
		dev, dev->port_num, remote_wakeup_allowed);

	port_num = rmnet_ports[dev->port_num].data_xport_num;
	switch (dxport) {
	case USB_GADGET_XPORT_BAM_DMUX:
		break;
	case USB_GADGET_XPORT_BAM2BAM_IPA:
		if (remote_wakeup_allowed) {
			gbam_resume(&dev->port, port_num, dxport);
		} else {
			dev->port.in->desc = dev->in_ep_desc_backup;
			dev->port.out->desc = dev->out_ep_desc_backup;
			pr_debug("%s(): Connecting\n", __func__);
			ret = gport_rmnet_connect(dev, dev->ifc_id);
			if (ret) {
				pr_err("%s: gport_rmnet_connect failed: err:%d\n",
								__func__, ret);
			}
		}
		break;
	case USB_GADGET_XPORT_HSIC:
		break;
	case USB_GADGET_XPORT_HSUART:
		break;
	case USB_GADGET_XPORT_ETHER:
		break;
	case USB_GADGET_XPORT_NONE:
		break;
	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				xport_to_str(dxport));
	}
}

static int frmnet_func_suspend(struct usb_function *f, u8 options)
{
	bool func_wakeup_allowed;

	pr_debug("func susp %u cmd for %s", options, f->name ? f->name : "");

	func_wakeup_allowed =
		((options & FUNC_SUSPEND_OPT_RW_EN_MASK) != 0);

	if (options & FUNC_SUSPEND_OPT_SUSP_MASK) {
		f->func_wakeup_allowed = func_wakeup_allowed;
		if (!f->func_is_suspended) {
			frmnet_suspend(f);
			f->func_is_suspended = true;
		}
	} else {
		if (f->func_is_suspended) {
			f->func_is_suspended = false;
			frmnet_resume(f);
		}
		f->func_wakeup_allowed = func_wakeup_allowed;
	}

	return 0;
}

static int frmnet_get_status(struct usb_function *f)
{
	unsigned remote_wakeup_en_status = f->func_wakeup_allowed ? 1 : 0;

	return (remote_wakeup_en_status << FUNC_WAKEUP_ENABLE_SHIFT) |
		(1 << FUNC_WAKEUP_CAPABLE_SHIFT);
}

static void frmnet_disable(struct usb_function *f)
{
	struct f_rmnet *dev = func_to_rmnet(f);
	enum transport_type	dxport = rmnet_ports[dev->port_num].data_xport;
	struct usb_composite_dev	*cdev = dev->cdev;

	pr_debug("%s: port#%d\n", __func__, dev->port_num);

	usb_ep_disable(dev->notify);
	dev->notify->driver_data = NULL;

	atomic_set(&dev->online, 0);

	frmnet_purge_responses(dev);

	if (dxport == USB_GADGET_XPORT_BAM2BAM_IPA &&
	    gadget_is_dwc3(cdev->gadget)) {
		msm_ep_unconfig(dev->port.out);
		msm_ep_unconfig(dev->port.in);
	}
	gport_rmnet_disconnect(dev);
}

static int
frmnet_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_rmnet			*dev = func_to_rmnet(f);
	struct usb_composite_dev	*cdev = dev->cdev;
	int				ret;
	struct list_head *cpkt;

	pr_debug("%s:dev:%pK port#%d\n", __func__, dev, dev->port_num);

	if (dev->notify->driver_data) {
		pr_debug("%s: reset port:%d\n", __func__, dev->port_num);
		usb_ep_disable(dev->notify);
	}

	ret = config_ep_by_speed(cdev->gadget, f, dev->notify);
	if (ret) {
		dev->notify->desc = NULL;
		ERROR(cdev, "config_ep_by_speed failes for ep %s, result %d\n",
					dev->notify->name, ret);
		return ret;
	}
	ret = usb_ep_enable(dev->notify);

	if (ret) {
		pr_err("%s: usb ep#%s enable failed, err#%d\n",
				__func__, dev->notify->name, ret);
		dev->notify->desc = NULL;
		return ret;
	}
	dev->notify->driver_data = dev;

	if (!dev->port.in->desc || !dev->port.out->desc) {
		if (config_ep_by_speed(cdev->gadget, f, dev->port.in) ||
			config_ep_by_speed(cdev->gadget, f, dev->port.out)) {
				pr_err("%s(): config_ep_by_speed failed.\n",
								__func__);
				ret = -EINVAL;
				goto err_disable_ep;
		}
		dev->port.gadget = dev->cdev->gadget;
	}

	ret = gport_rmnet_connect(dev, intf);
	if (ret) {
		pr_err("%s(): gport_rmnet_connect fail with err:%d\n",
							__func__, ret);
		goto err_disable_ep;
	}

	atomic_set(&dev->online, 1);

	/* In case notifications were aborted, but there are pending control
	   packets in the response queue, re-add the notifications */
	list_for_each(cpkt, &dev->cpkt_resp_q)
		frmnet_ctrl_response_available(dev);

	return ret;
err_disable_ep:
	dev->port.in->desc = NULL;
	dev->port.out->desc = NULL;
	usb_ep_disable(dev->notify);

	return ret;
}

static void frmnet_ctrl_response_available(struct f_rmnet *dev)
{
	struct usb_request		*req = dev->notify_req;
	struct usb_cdc_notification	*event;
	unsigned long			flags;
	int				ret;
	struct rmnet_ctrl_pkt	*cpkt;

	pr_debug("%s:dev:%pK portno#%d\n", __func__, dev, dev->port_num);

	spin_lock_irqsave(&dev->lock, flags);
	if (!atomic_read(&dev->online) || !req || !req->buf) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return;
	}

	if (++dev->notify_count != 1) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return;
	}

	event = req->buf;
	event->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS
			| USB_RECIP_INTERFACE;
	event->bNotificationType = USB_CDC_NOTIFY_RESPONSE_AVAILABLE;
	event->wValue = cpu_to_le16(0);
	event->wIndex = cpu_to_le16(dev->ifc_id);
	event->wLength = cpu_to_le16(0);
	spin_unlock_irqrestore(&dev->lock, flags);

	ret = usb_ep_queue(dev->notify, dev->notify_req, GFP_ATOMIC);
#if 1 //add by carl, PC maybe not open qmi-channel, store this qmi and wait PC to read
	if (ret == -EBUSY) {
		unsigned long notify_count = dev->notify_count;
		pr_info("frmnet ep enqueue busy notify_count = %ld\n", notify_count);
		if (notify_count < 1000) //OFFLINE_UL_Q_LIMIT
			ret = 0;
	}
#endif
	if (ret) {
		spin_lock_irqsave(&dev->lock, flags);
		if (!list_empty(&dev->cpkt_resp_q)) {
			if (dev->notify_count > 0)
				dev->notify_count--;
			else {
				pr_debug("%s: Invalid notify_count=%lu to decrement\n",
					 __func__, dev->notify_count);
				spin_unlock_irqrestore(&dev->lock, flags);
				return;
			}
			cpkt = list_first_entry(&dev->cpkt_resp_q,
					struct rmnet_ctrl_pkt, list);
			list_del(&cpkt->list);
			rmnet_free_ctrl_pkt(cpkt);
		}
		spin_unlock_irqrestore(&dev->lock, flags);
		pr_debug("ep enqueue error %d\n", ret);
	}
}

static void frmnet_connect(struct grmnet *gr)
{
	struct f_rmnet			*dev;

	if (!gr) {
		pr_err("%s: Invalid grmnet:%pK\n", __func__, gr);
		return;
	}

	dev = port_to_rmnet(gr);

	atomic_set(&dev->ctrl_online, 1);
}

static void frmnet_disconnect(struct grmnet *gr)
{
	struct f_rmnet			*dev;
	struct usb_cdc_notification	*event;
	int				status;

	if (!gr) {
		pr_err("%s: Invalid grmnet:%pK\n", __func__, gr);
		return;
	}

	dev = port_to_rmnet(gr);

	atomic_set(&dev->ctrl_online, 0);

	if (!atomic_read(&dev->online)) {
		pr_debug("%s: nothing to do\n", __func__);
		return;
	}

	usb_ep_dequeue(dev->notify, dev->notify_req);

#if 0 //comment by carl, this will make dev->notify_count un-corrent and one qmi response miss to notify to PC, I think PC donot care this cdc-msg, so remove it
	event = dev->notify_req->buf;
	event->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS
			| USB_RECIP_INTERFACE;
	event->bNotificationType = USB_CDC_NOTIFY_NETWORK_CONNECTION;
	event->wValue = cpu_to_le16(0);
	event->wIndex = cpu_to_le16(dev->ifc_id);
	event->wLength = cpu_to_le16(0);

	status = usb_ep_queue(dev->notify, dev->notify_req, GFP_ATOMIC);
	if (status < 0) {
		if (!atomic_read(&dev->online))
			return;
		pr_err("%s: rmnet notify ep enqueue error %d\n",
				__func__, status);
	}
#endif

	frmnet_purge_responses(dev);
}

static int
frmnet_send_cpkt_response(void *gr, void *buf, size_t len)
{
	struct f_rmnet		*dev;
	struct rmnet_ctrl_pkt	*cpkt;
	unsigned long		flags;

	if (!gr || !buf) {
		pr_err("%s: Invalid grmnet/buf, grmnet:%pK buf:%pK\n",
				__func__, gr, buf);
		return -ENODEV;
	}
	cpkt = rmnet_alloc_ctrl_pkt(len, GFP_ATOMIC);
	if (IS_ERR(cpkt)) {
		pr_err("%s: Unable to allocate ctrl pkt\n", __func__);
		return -ENOMEM;
	}
	memcpy(cpkt->buf, buf, len);
	cpkt->len = len;

	dev = port_to_rmnet(gr);

	pr_debug("%s: dev:%pK port#%d\n", __func__, dev, dev->port_num);

	if (!atomic_read(&dev->online) || !atomic_read(&dev->ctrl_online)) {
		rmnet_free_ctrl_pkt(cpkt);
		return 0;
	}

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&cpkt->list, &dev->cpkt_resp_q);
	spin_unlock_irqrestore(&dev->lock, flags);

	frmnet_ctrl_response_available(dev);

	return 0;
}

static void
frmnet_cmd_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_rmnet			*dev = req->context;
	struct usb_composite_dev	*cdev;
	unsigned			port_num;

	if (!dev) {
		pr_err("%s: rmnet dev is null\n", __func__);
		return;
	}

	pr_debug("%s: dev:%pK port#%d\n", __func__, dev, dev->port_num);

	cdev = dev->cdev;

	if (dev->port.send_encap_cmd) {
		port_num = rmnet_ports[dev->port_num].ctrl_xport_num;
		dev->port.send_encap_cmd(port_num, req->buf, req->actual);
	}
}

static void frmnet_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_rmnet *dev = req->context;
	int status = req->status;
	unsigned long		flags;
	struct rmnet_ctrl_pkt	*cpkt;

	pr_debug("%s: dev:%pK port#%d\n", __func__, dev, dev->port_num);

	switch (status) {
	case -ECONNRESET:
	case -ESHUTDOWN:
		/* connection gone */
		spin_lock_irqsave(&dev->lock, flags);
		dev->notify_count = 0;
		spin_unlock_irqrestore(&dev->lock, flags);
		break;
	default:
		pr_err("rmnet notify ep error %d\n", status);
		/* FALLTHROUGH */
	case 0:
		if (!atomic_read(&dev->ctrl_online))
			break;

		spin_lock_irqsave(&dev->lock, flags);
		if (dev->notify_count > 0) {
			dev->notify_count--;
			if (dev->notify_count == 0) {
				spin_unlock_irqrestore(&dev->lock, flags);
				break;
			}
		} else {
			pr_debug("%s: Invalid notify_count=%lu to decrement\n",
					__func__, dev->notify_count);
			spin_unlock_irqrestore(&dev->lock, flags);
			break;
		}
		spin_unlock_irqrestore(&dev->lock, flags);

		status = usb_ep_queue(dev->notify, req, GFP_ATOMIC);
		if (status) {
			spin_lock_irqsave(&dev->lock, flags);
			if (!list_empty(&dev->cpkt_resp_q)) {
				if (dev->notify_count > 0)
					dev->notify_count--;
				else {
					pr_err("%s: Invalid notify_count=%lu to decrement\n",
						__func__, dev->notify_count);
					spin_unlock_irqrestore(&dev->lock,
								flags);
					break;
				}
				cpkt = list_first_entry(&dev->cpkt_resp_q,
						struct rmnet_ctrl_pkt, list);
				list_del(&cpkt->list);
				rmnet_free_ctrl_pkt(cpkt);
			}
			spin_unlock_irqrestore(&dev->lock, flags);
			pr_debug("ep enqueue error %d\n", status);
		}
		break;
	}
}

#ifdef QUECTEL_MULTI_IP_PACKAGES
static void
frmnet_set_multi_package_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct multi_package_config {
		__le32 enable;
		__le32 package_max_len;
		__le32 package_max_count_in_queue;
		__le32 timeout;
	} __packed;

	struct multi_package_config cfg;
	
	if (sizeof(cfg) != req->actual) {
		return;
	}

	memcpy(&cfg, req->buf, sizeof(cfg));
	multi_package_enabled = le32_to_cpu(cfg.enable);
	multi_package_max_len = le32_to_cpu(cfg.package_max_len);
	package_max_count_in_queue = le32_to_cpu(cfg.package_max_count_in_queue);
	wait_for_package_timeout = le32_to_cpu(cfg.timeout);
}
#endif

static int
frmnet_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_rmnet			*dev = func_to_rmnet(f);
	struct usb_composite_dev	*cdev = dev->cdev;
	struct usb_request		*req = cdev->req;
	unsigned			port_num;
	u16				w_index = le16_to_cpu(ctrl->wIndex);
	u16				w_value = le16_to_cpu(ctrl->wValue);
	u16				w_length = le16_to_cpu(ctrl->wLength);
	int				ret = -EOPNOTSUPP;

	pr_debug("%s:dev:%pK port#%d\n", __func__, dev, dev->port_num);

	if (!atomic_read(&dev->online)) {
		pr_warning("%s: usb cable is not connected\n", __func__);
		return -ENOTCONN;
	}

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {
#ifdef QUECTEL_MULTI_IP_PACKAGES
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SET_MULTI_PACKAGE_COMMAND:
		ret = w_length;
		req->complete = frmnet_set_multi_package_complete;
		req->context = dev;
		break;
#endif

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SEND_ENCAPSULATED_COMMAND:
		pr_debug("%s: USB_CDC_SEND_ENCAPSULATED_COMMAND\n"
				 , __func__);
		ret = w_length;
		req->complete = frmnet_cmd_complete;
		req->context = dev;
		break;


	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_GET_ENCAPSULATED_RESPONSE:
		pr_debug("%s: USB_CDC_GET_ENCAPSULATED_RESPONSE\n", __func__);
		if (w_value) {
			pr_err("%s: invalid w_value = %04x\n",
				   __func__ , w_value);
			goto invalid;
		} else {
			unsigned len;
			struct rmnet_ctrl_pkt *cpkt;

			spin_lock(&dev->lock);
			if (list_empty(&dev->cpkt_resp_q)) {
				pr_err("ctrl resp queue empty "
					" req%02x.%02x v%04x i%04x l%d\n",
					ctrl->bRequestType, ctrl->bRequest,
					w_value, w_index, w_length);
				ret = 0;
				spin_unlock(&dev->lock);
				goto invalid;
			}

			cpkt = list_first_entry(&dev->cpkt_resp_q,
					struct rmnet_ctrl_pkt, list);
			list_del(&cpkt->list);
			spin_unlock(&dev->lock);

			len = min_t(unsigned, w_length, cpkt->len);
			memcpy(req->buf, cpkt->buf, len);
			ret = len;

			rmnet_free_ctrl_pkt(cpkt);
		}
		break;
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		pr_debug("%s: USB_CDC_REQ_SET_CONTROL_LINE_STATE: DTR:%d\n",
				__func__, w_value & ACM_CTRL_DTR ? 1 : 0);
		if (dev->port.notify_modem) {
			port_num = rmnet_ports[dev->port_num].ctrl_xport_num;
			dev->port.notify_modem(&dev->port, port_num, w_value);
		}
		ret = 0;

		break;
	default:

invalid:
		DBG(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (ret >= 0) {
		VDBG(cdev, "rmnet req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = (ret < w_length);
		req->length = ret;
		ret = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (ret < 0)
			ERROR(cdev, "rmnet ep0 enqueue err %d\n", ret);
	}

	return ret;
}

static int frmnet_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_rmnet			*dev = func_to_rmnet(f);
	struct usb_ep			*ep;
	struct usb_composite_dev	*cdev = c->cdev;
	int				ret = -ENODEV;
	pr_debug("%s: start binding\n", __func__);
	dev->ifc_id = usb_interface_id(c, f);
	if (dev->ifc_id < 0) {
		pr_err("%s: unable to allocate ifc id, err:%d\n",
				__func__, dev->ifc_id);
		return dev->ifc_id;
	}
	rmnet_interface_desc.bInterfaceNumber = dev->ifc_id;

	ep = usb_ep_autoconfig(cdev->gadget, &rmnet_fs_in_desc);
	if (!ep) {
		pr_err("%s: usb epin autoconfig failed\n", __func__);
		return -ENODEV;
	}
	dev->port.in = ep;
	/* Update same for u_ether which uses gether port struct */
	dev->gether_port.in_ep = ep;
	ep->driver_data = cdev;

	ep = usb_ep_autoconfig(cdev->gadget, &rmnet_fs_out_desc);
	if (!ep) {
		pr_err("%s: usb epout autoconfig failed\n", __func__);
		ret = -ENODEV;
		goto ep_auto_out_fail;
	}
	dev->port.out = ep;
	/* Update same for u_ether which uses gether port struct */
	dev->gether_port.out_ep = ep;
	ep->driver_data = cdev;

	ep = usb_ep_autoconfig(cdev->gadget, &rmnet_fs_notify_desc);
	if (!ep) {
		pr_err("%s: usb epnotify autoconfig failed\n", __func__);
		ret = -ENODEV;
		goto ep_auto_notify_fail;
	}
	dev->notify = ep;
	ep->driver_data = cdev;

	dev->notify_req = frmnet_alloc_req(ep,
				sizeof(struct usb_cdc_notification),
				cdev->gadget->extra_buf_alloc,
				GFP_KERNEL);
	if (IS_ERR(dev->notify_req)) {
		pr_err("%s: unable to allocate memory for notify req\n",
				__func__);
		ret = -ENOMEM;
		goto ep_notify_alloc_fail;
	}

	dev->notify_req->complete = frmnet_notify_complete;
	dev->notify_req->context = dev;

	ret = -ENOMEM;
	f->fs_descriptors = usb_copy_descriptors(rmnet_fs_function);

	if (!f->fs_descriptors) {
		pr_err("%s: no descriptors,usb_copy descriptors(fs)failed\n",
			__func__);
		goto fail;
	}
	if (gadget_is_dualspeed(cdev->gadget)) {
		rmnet_hs_in_desc.bEndpointAddress =
				rmnet_fs_in_desc.bEndpointAddress;
		rmnet_hs_out_desc.bEndpointAddress =
				rmnet_fs_out_desc.bEndpointAddress;
		rmnet_hs_notify_desc.bEndpointAddress =
				rmnet_fs_notify_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(rmnet_hs_function);

		if (!f->hs_descriptors) {
			pr_err("%s: no hs_descriptors,usb_copy descriptors(hs)failed\n",
			__func__);
			goto fail;
		}
	}

	if (gadget_is_superspeed(cdev->gadget)) {
		rmnet_ss_in_desc.bEndpointAddress =
				rmnet_fs_in_desc.bEndpointAddress;
		rmnet_ss_out_desc.bEndpointAddress =
				rmnet_fs_out_desc.bEndpointAddress;
		rmnet_ss_notify_desc.bEndpointAddress =
				rmnet_fs_notify_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->ss_descriptors = usb_copy_descriptors(rmnet_ss_function);

		if (!f->ss_descriptors) {
			pr_err("%s: no ss_descriptors,usb_copy descriptors(ss)failed\n",
			__func__);
			goto fail;
		}
	}

	pr_debug("%s: RmNet(%d) %s Speed, IN:%s OUT:%s\n",
			__func__, dev->port_num,
			gadget_is_dualspeed(cdev->gadget) ? "dual" : "full",
			dev->port.in->name, dev->port.out->name);

	return 0;

fail:
	if (f->ss_descriptors)
		usb_free_descriptors(f->ss_descriptors);
	if (f->hs_descriptors)
		usb_free_descriptors(f->hs_descriptors);
	if (f->fs_descriptors)
		usb_free_descriptors(f->fs_descriptors);
	if (dev->notify_req)
		frmnet_free_req(dev->notify, dev->notify_req);
ep_notify_alloc_fail:
	dev->notify->driver_data = NULL;
	dev->notify = NULL;
ep_auto_notify_fail:
	dev->port.out->driver_data = NULL;
	dev->port.out = NULL;
ep_auto_out_fail:
	dev->port.in->driver_data = NULL;
	dev->port.in = NULL;

	return ret;
}

static int frmnet_bind_config(struct usb_configuration *c, unsigned portno)
{
	int			status;
	struct f_rmnet		*dev;
	struct usb_function	*f;
	unsigned long		flags;

	pr_debug("%s: usb config:%pK\n", __func__, c);

	if (portno >= nr_rmnet_ports) {
		pr_err("%s: supporting ports#%u port_id:%u\n", __func__,
				nr_rmnet_ports, portno);
		return -ENODEV;
	}

	dev = rmnet_ports[portno].port;

	if (rmnet_ports[portno].data_xport == USB_GADGET_XPORT_ETHER) {
		struct net_device *net = gether_setup_name_default("usb_rmnet");
		if (IS_ERR(net)) {
			pr_err("%s: gether_setup failed\n", __func__);
			return PTR_ERR(net);
		}
		dev->gether_port.ioport = netdev_priv(net);
		gether_set_gadget(net, c->cdev->gadget);
		status = gether_register_netdev(net);
		if (status < 0) {
			pr_err("%s: gether_register_netdev failed\n", __func__);
			free_netdev(net);
			return status;
		}
	}

	if (rmnet_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0) {
			pr_err("%s: failed to get string id, err:%d\n",
					__func__, status);
			return status;
		}
		rmnet_string_defs[0].id = status;
	}

	spin_lock_irqsave(&dev->lock, flags);
	dev->cdev = c->cdev;
	f = &dev->gether_port.func;
	dev->port.f = f;
	f->name = kasprintf(GFP_ATOMIC, "rmnet%d", portno);
	spin_unlock_irqrestore(&dev->lock, flags);
	if (!f->name) {
		pr_err("%s: cannot allocate memory for name\n", __func__);
		return -ENOMEM;
	}

	f->strings = rmnet_strings;
	f->bind = frmnet_bind;
	f->unbind = frmnet_unbind;
	f->disable = frmnet_disable;
	f->set_alt = frmnet_set_alt;
	f->setup = frmnet_setup;
	f->suspend = frmnet_suspend;
	f->resume = frmnet_resume;
	f->func_suspend = frmnet_func_suspend;
	f->get_status = frmnet_get_status;
	dev->port.send_cpkt_response = frmnet_send_cpkt_response;
	dev->port.disconnect = frmnet_disconnect;
	dev->port.connect = frmnet_connect;
	dev->gether_port.cdc_filter = 0;

	status = usb_add_function(c, f);
	if (status) {
		pr_err("%s: usb add function failed: %d\n",
				__func__, status);
		kfree(f->name);
		return status;
	}
	if (rmnet_ports[portno].data_xport ==
			USB_GADGET_XPORT_BAM2BAM_IPA)
		c->cdev->gadget->bam2bam_func_enabled = true;

	pr_debug("%s: complete\n", __func__);

	return status;
}

static void frmnet_unbind_config(void)
{
	int i;

	for (i = 0; i < nr_rmnet_ports; i++)
		if (rmnet_ports[i].data_xport == USB_GADGET_XPORT_ETHER) {
			gether_cleanup(rmnet_ports[i].port->gether_port.ioport);
			rmnet_ports[i].port->gether_port.ioport = NULL;
		}
}

static int rmnet_init(void)
{
	return gqti_ctrl_init();
}

static void frmnet_cleanup(void)
{
	int i;

	gqti_ctrl_cleanup();

	for (i = 0; i < nr_rmnet_ports; i++)
		kfree(rmnet_ports[i].port);

	gbam_cleanup();
	nr_rmnet_ports = 0;
	no_ctrl_smd_ports = 0;
	no_ctrl_qti_ports = 0;
	no_data_bam_ports = 0;
	no_data_bam2bam_ports = 0;
	no_ctrl_hsic_ports = 0;
	no_data_hsic_ports = 0;
	no_ctrl_hsuart_ports = 0;
	no_data_hsuart_ports = 0;
}

static int frmnet_init_port(const char *ctrl_name, const char *data_name,
		const char *port_name)
{
	struct f_rmnet			*dev;
	struct rmnet_ports		*rmnet_port;
	int				ret;
	int				i;

	if (nr_rmnet_ports >= NR_RMNET_PORTS) {
		pr_err("%s: Max-%d instances supported\n",
				__func__, NR_RMNET_PORTS);
		return -EINVAL;
	}

	pr_debug("%s: port#:%d, ctrl port: %s data port: %s\n",
		__func__, nr_rmnet_ports, ctrl_name, data_name);

	dev = kzalloc(sizeof(struct f_rmnet), GFP_KERNEL);
	if (!dev) {
		pr_err("%s: Unable to allocate rmnet device\n", __func__);
		return -ENOMEM;
	}

	dev->port_num = nr_rmnet_ports;
	spin_lock_init(&dev->lock);
	INIT_LIST_HEAD(&dev->cpkt_resp_q);

	rmnet_port = &rmnet_ports[nr_rmnet_ports];
	rmnet_port->port = dev;
	rmnet_port->port_num = nr_rmnet_ports;
	rmnet_port->ctrl_xport = str_to_xport(ctrl_name);
	rmnet_port->data_xport = str_to_xport(data_name);

	switch (rmnet_port->ctrl_xport) {
	case USB_GADGET_XPORT_SMD:
		rmnet_port->ctrl_xport_num = no_ctrl_smd_ports;
		no_ctrl_smd_ports++;
		break;
	case USB_GADGET_XPORT_QTI:
		rmnet_port->ctrl_xport_num = no_ctrl_qti_ports;
		no_ctrl_qti_ports++;
		break;
	case USB_GADGET_XPORT_HSIC:
		ghsic_ctrl_set_port_name(port_name, ctrl_name);
		rmnet_port->ctrl_xport_num = no_ctrl_hsic_ports;
		no_ctrl_hsic_ports++;
		break;
	case USB_GADGET_XPORT_HSUART:
		rmnet_port->ctrl_xport_num = no_ctrl_hsuart_ports;
		no_ctrl_hsuart_ports++;
		break;
	case USB_GADGET_XPORT_NONE:
		break;
	default:
		pr_err("%s: Un-supported transport: %u\n", __func__,
				rmnet_port->ctrl_xport);
		ret = -ENODEV;
		goto fail_probe;
	}

	switch (rmnet_port->data_xport) {
	case USB_GADGET_XPORT_BAM2BAM:
		/* Override BAM2BAM to BAM_DMUX for old ABI compatibility */
		rmnet_port->data_xport = USB_GADGET_XPORT_BAM_DMUX;
		/* fall-through */
	case USB_GADGET_XPORT_BAM_DMUX:
		rmnet_port->data_xport_num = no_data_bam_ports;
		no_data_bam_ports++;
		break;
	case USB_GADGET_XPORT_BAM2BAM_IPA:
		rmnet_port->data_xport_num = no_data_bam2bam_ports;
		no_data_bam2bam_ports++;
		break;
	case USB_GADGET_XPORT_HSIC:
		ghsic_data_set_port_name(port_name, data_name);
		rmnet_port->data_xport_num = no_data_hsic_ports;
		no_data_hsic_ports++;
		break;
	case USB_GADGET_XPORT_HSUART:
		rmnet_port->data_xport_num = no_data_hsuart_ports;
		no_data_hsuart_ports++;
		break;
	case USB_GADGET_XPORT_ETHER:
	case USB_GADGET_XPORT_NONE:
		break;
	default:
		pr_err("%s: Un-supported transport: %u\n", __func__,
				rmnet_port->data_xport);
		ret = -ENODEV;
		goto fail_probe;
	}
	nr_rmnet_ports++;

	return 0;

fail_probe:
	for (i = 0; i < nr_rmnet_ports; i++)
		kfree(rmnet_ports[i].port);

	nr_rmnet_ports = 0;
	no_ctrl_smd_ports = 0;
	no_ctrl_qti_ports = 0;
	no_data_bam_ports = 0;
	no_ctrl_hsic_ports = 0;
	no_data_hsic_ports = 0;
	no_ctrl_hsuart_ports = 0;
	no_data_hsuart_ports = 0;

	return ret;
}
static void frmnet_deinit_port(void)
{
	int i;

	for (i = 0; i < nr_rmnet_ports; i++)
		kfree(rmnet_ports[i].port);

	nr_rmnet_ports = 0;
	no_ctrl_smd_ports = 0;
	no_ctrl_qti_ports = 0;
	no_data_bam_ports = 0;
	no_data_bam2bam_ports = 0;
	no_ctrl_hsic_ports = 0;
	no_data_hsic_ports = 0;
	no_ctrl_hsuart_ports = 0;
	no_data_hsuart_ports = 0;
}
