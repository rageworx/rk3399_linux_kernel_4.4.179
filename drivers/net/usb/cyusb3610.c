/*
 * Cypress Semiconductor CYUSB3610 based USB 3.0 Ethernet Devices
 * Copyright (C) 2003-2005 David Hollis <dhollis@davehollis.com>
 * Copyright (C) 2005 Phil Chang <pchang23@sbcglobal.net>
 * Copyright (c) 2002-2003 TiVo Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* debug messages, extra info */
/* #define	DEBUG */

#include <linux/version.h>
/*#include <linux/config.h>*/
#ifdef	CONFIG_USB_DEBUG
#define DEBUG
#endif
#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <linux/if_vlan.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)
#include <linux/usb/usbnet.h>
#else
#include <../drivers/usb/net/usbnet.h>
#endif

#include "cyusb3610.h"

#define DRV_VERSION	"1.14.2"

static char version[] =
KERN_INFO "Cypress USB Ethernet Adapter:v" DRV_VERSION
//	" " __TIME__ " " __DATE__ "\n"
"		http://www.cypress.com\n";

static int msg_enable;
module_param(msg_enable, int, 0);
MODULE_PARM_DESC(msg_enable, "usbnet msg_enable");

static int bsize = -1;
module_param(bsize, int, 0);
MODULE_PARM_DESC(bsize, "RX Bulk IN Queue Size");

static int ifg = -1;
module_param(ifg, int, 0);
MODULE_PARM_DESC(ifg, "RX Bulk IN Inter Frame Gap");


/* EEE advertisement is disabled in default setting */
static int bEEE = 0;
module_param(bEEE, int, 0);
MODULE_PARM_DESC(bEEE, "EEE advertisement configuration");

/* Green ethernet advertisement is disabled in default setting */
static int bGETH = 0;
module_param(bGETH, int, 0);
MODULE_PARM_DESC(bGETH, "Green ethernet configuration");
/* Cypress cyusb3610 based USB 3.0/2.0 Gigabit Ethernet Devices */

static int __cyusb3610_read_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			      u16 size, void *data, int in_pm)
{
	int ret;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	int (*fn)(struct usbnet *, u8, u8, u16, u16, void *, u16);

	BUG_ON(!dev);

	if (!in_pm)
		fn = usbnet_read_cmd;
	else
		fn = usbnet_read_cmd_nopm;

	ret = fn(dev, cmd, USB_DIR_IN | USB_TYPE_VENDOR |
		 USB_RECIP_DEVICE, value, index, data, size);

	if (unlikely(ret < 0))
		netdev_warn(dev->net, "Failed to read reg index 0x%04x: %d\n",
			    index, ret);
#else
	ret = usb_control_msg(
		dev->udev,
		usb_rcvctrlpipe(dev->udev, 0),
		cmd,
		USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		value,
		index,
		data,
		size,
		USB_CTRL_GET_TIMEOUT);
#endif
	return ret;
}

static int __cyusb3610_write_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			       u16 size, void *data, int in_pm)
{
	int ret;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	int (*fn)(struct usbnet *, u8, u8, u16, u16, const void *, u16);

	BUG_ON(!dev);

	if (!in_pm)
		fn = usbnet_write_cmd;
	else
		fn = usbnet_write_cmd_nopm;

	ret = fn(dev, cmd, USB_DIR_OUT | USB_TYPE_VENDOR |
		 USB_RECIP_DEVICE, value, index, data, size);

	if (unlikely(ret < 0))
		netdev_warn(dev->net, "Failed to write reg index 0x%04x: %d\n",
			    index, ret);
#else
	ret = usb_control_msg(
		dev->udev,
		usb_sndctrlpipe(dev->udev, 0),
		cmd,
		USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		value,
		index,
		data,
		size,
		USB_CTRL_SET_TIMEOUT);

#endif
	return ret;
}

static int cyusb3610_read_cmd_nopm(struct usbnet *dev, u8 cmd, u16 value,
				 u16 index, u16 size, void *data, int eflag)
{
	int ret;

	if (eflag && (2 == size)) {
		u16 buf = 0;
		ret = __cyusb3610_read_cmd(dev, cmd, value, index, size, &buf, 1);
		le16_to_cpus(&buf);
		*((u16 *)data) = buf;
	} else if (eflag && (4 == size)) {
		u32 buf = 0;
		ret = __cyusb3610_read_cmd(dev, cmd, value, index, size, &buf, 1);
		le32_to_cpus(&buf);
		*((u32 *)data) = buf;
	} else {
		ret = __cyusb3610_read_cmd(dev, cmd, value, index, size, data, 1);
	}

	return ret;
}

static int cyusb3610_write_cmd_nopm(struct usbnet *dev, u8 cmd, u16 value,
				  u16 index, u16 size, void *data)
{
	int ret;

	if (2 == size) {
		u16 buf = 0;
		buf = *((u16 *)data);
		cpu_to_le16s(&buf);
		ret = __cyusb3610_write_cmd(dev, cmd, value, index,
					  size, &buf, 1);
	} else {
		ret = __cyusb3610_write_cmd(dev, cmd, value, index,
					  size, data, 1);
	}

	return ret;
}

static int cyusb3610_read_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			    u16 size, void *data, int eflag)
{

	int ret;

	if (eflag && (2 == size)) {
		u16 buf = 0;
		ret = __cyusb3610_read_cmd(dev, cmd, value, index, size, &buf, 0);
		le16_to_cpus(&buf);
		*((u16 *)data) = buf;
	} else if (eflag && (4 == size)) {
		u32 buf = 0;
		ret = __cyusb3610_read_cmd(dev, cmd, value, index, size, &buf, 0);
		le32_to_cpus(&buf);
		*((u32 *)data) = buf;
	} else {
		ret = __cyusb3610_read_cmd(dev, cmd, value, index, size, data, 0);
	}

	return ret;
}

static int cyusb3610_write_cmd(struct usbnet *dev, u8 cmd, u16 value, u16 index,
			     u16 size, void *data)
{
	int ret;

	if (2 == size) {
		u16 buf = 0;
		buf = *((u16 *)data);
		cpu_to_le16s(&buf);
		ret = __cyusb3610_write_cmd(dev, cmd, value, index,
					  size, &buf, 0);
	} else {
		ret = __cyusb3610_write_cmd(dev, cmd, value, index,
					  size, data, 0);
	}

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static void cyusb3610_async_cmd_callback(struct urb *urb, struct pt_regs *regs)
#else
static void cyusb3610_async_cmd_callback(struct urb *urb)
#endif
{
	struct cyusb3610_async_handle *asyncdata = (struct cyusb3610_async_handle *)urb->context;

	if (urb->status < 0)
		printk(KERN_ERR "cyusb3610_async_cmd_callback() failed with %d",
		       urb->status);

	kfree(asyncdata->req);
	kfree(asyncdata);	
	usb_free_urb(urb);
	
}

static void
cyusb3610_write_cmd_async(struct usbnet *dev, u8 cmd, u16 value, u16 index,
				    u16 size, void *data)
{
	struct usb_ctrlrequest *req = NULL;
	int status = 0;
	struct urb *urb = NULL;
	void *buf = NULL;
	struct cyusb3610_async_handle *asyncdata = NULL;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (urb == NULL) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Error allocating URB in write_cmd_async!");
#else
		deverr(dev, "Error allocating URB in write_cmd_async!");
#endif
		return;
	}

	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_ATOMIC);
	if (req == NULL) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Failed to allocate memory for control request");
#else
		deverr(dev, "Failed to allocate memory for control request");
#endif
		usb_free_urb(urb);
		return;
	}

	asyncdata = (struct cyusb3610_async_handle*)
			kmalloc(sizeof(struct cyusb3610_async_handle), GFP_ATOMIC);
	if (asyncdata == NULL) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Failed to allocate memory for async data");
#else
		deverr(dev, "Failed to allocate memory for async data");
#endif
		kfree(req);
		usb_free_urb(urb);
		return;
	}

	asyncdata->req = req;
	
	if (size == 2) {
		asyncdata->rxctl = *((u16 *)data);
		cpu_to_le16s(&asyncdata->rxctl);
		buf = &asyncdata->rxctl;
	} else {
		memcpy(asyncdata->m_filter, data, size);
		buf = asyncdata->m_filter;
	}

	req->bRequestType = USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	req->bRequest = cmd;
	req->wValue = cpu_to_le16(value);
	req->wIndex = cpu_to_le16(index);
	req->wLength = cpu_to_le16(size);

	usb_fill_control_urb(urb, dev->udev,
			     usb_sndctrlpipe(dev->udev, 0),
			     (void *)req, buf, size,
			     cyusb3610_async_cmd_callback, asyncdata);

	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status < 0) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Error submitting the control message: status=%d",
			   status);
#else
		deverr(dev, "Error submitting the control message: status=%d",
		       status);
#endif
		kfree(req);
		kfree(asyncdata);
		usb_free_urb(urb);
	}
}

static void cyusb3610_status(struct usbnet *dev, struct urb *urb)
{
	struct cyusb3610_int_data *event = NULL;
	int link = 0;

	if (urb->actual_length < 8)
		return;

	event = urb->transfer_buffer;
	link = event->link & CY_INT_PPLS_LINK;

	if (netif_carrier_ok(dev->net) != link) {
		if (link) {
			netif_carrier_on(dev->net);
			usbnet_defer_kevent(dev, EVENT_LINK_RESET);
		}
		else
			netif_carrier_off(dev->net);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_info(dev->net, "cyusb3610 - Link status is: %d\n",
			    link);
#else
		devinfo(dev, "cyusb3610 - Link status is: %d\n", link);
#endif
	}
}

static int cyusb3610_mdio_read(struct net_device *netdev, int phy_id, int loc)
{
	struct usbnet *dev = netdev_priv(netdev);
	u16 res = 0;

	cyusb3610_read_cmd(dev, CY_ACCESS_PHY, phy_id, (__u16)loc, 2, &res, 1);
	return res;
}

static void cyusb3610_mdio_write(struct net_device *netdev, int phy_id, int loc,
			       int val)
{
	struct usbnet *dev = netdev_priv(netdev);
	u16 res = (u16)val;

	cyusb3610_write_cmd(dev, CY_ACCESS_PHY, phy_id, (__u16)loc, 2, &res);
}

static int cyusb3610_suspend(struct usb_interface *intf,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 10)
			   pm_message_t message)
#else
			   u32 message)
#endif
{
	struct usbnet *dev = usb_get_intfdata(intf);
	u16 tmp16 = 0;
	u8 tmp8 = 0;

	usbnet_suspend(intf, message);

	/* Disable RX path */
	cyusb3610_read_cmd_nopm(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
			      2, 2, &tmp16, 1);
	tmp16 &= ~CY_MEDIUM_RECEIVE_EN;
	cyusb3610_write_cmd_nopm(dev, CY_ACCESS_MAC,  CY_MEDIUM_STATUS_MODE,
			       2, 2, &tmp16);

	/* Force bz */
	cyusb3610_read_cmd_nopm(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL,
			      2, 2, &tmp16, 1);
	tmp16 |= CY_PHYPWR_RSTCTL_BZ | CY_PHYPWR_RSTCTL_IPRL;
	cyusb3610_write_cmd_nopm(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL,
			       2, 2, &tmp16);

	/* change clock */
	tmp8 = 0;
	cyusb3610_write_cmd_nopm(dev, CY_ACCESS_MAC, CY_CLK_SELECT, 1, 1, &tmp8);

	/* Configure RX control register => stop operation */
	tmp16 = CY_RX_CTL_STOP;
	cyusb3610_write_cmd_nopm(dev, CY_ACCESS_MAC, CY_RX_CTL, 2, 2, &tmp16);

	return 0;
}


static void cyusb3610_EEE_setting(struct usbnet *dev)
{
	u16 tmp16;
	
	if (bEEE) {
		// Enable EEE
		tmp16 = 0x07;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  GMII_PHY_MACR, 2, &tmp16);

		tmp16 = 0x3c;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  GMII_PHY_MAADR, 2, &tmp16);

		tmp16 = 0x4007;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  GMII_PHY_MACR, 2, &tmp16);

		tmp16 = 0x06;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  GMII_PHY_MAADR, 2, &tmp16);
	} else {
		// Disable EEE
		tmp16 = 0x07;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  GMII_PHY_MACR, 2, &tmp16);

		tmp16 = 0x3c;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  GMII_PHY_MAADR, 2, &tmp16);

		tmp16 = 0x4007;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  GMII_PHY_MACR, 2, &tmp16);

		tmp16 = 0x00;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  GMII_PHY_MAADR, 2, &tmp16);
	}
}

static void cyusb3610_Gether_setting(struct usbnet *dev)
{

	u16 tmp16;

	if (bGETH) {
		// Enable Green Ethernet
		tmp16 = 0x03;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  31, 2, &tmp16);

		tmp16 = 0x3247;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  25, 2, &tmp16);

		tmp16 = 0x05;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  31, 2, &tmp16);

		tmp16 = 0x0680;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  1, 2, &tmp16);

		tmp16 = 0;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  31, 2, &tmp16);
	} else {
		// Disable Green Ethernet
		tmp16 = 0x03;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  31, 2, &tmp16);

		tmp16 = 0x3246;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  25, 2, &tmp16);

		tmp16 = 0;
		cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
				  31, 2, &tmp16);
	}
}

static int cyusb3610_resume(struct usb_interface *intf)
{
	struct usbnet *dev = usb_get_intfdata(intf);
	u16 tmp16 = 0;
	u8 tmp8 = 0;

	netif_carrier_off(dev->net);

	/* Power up ethernet PHY */
	tmp16 = 0;
	cyusb3610_write_cmd_nopm(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL,
			       2, 2, &tmp16);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
	usleep_range(1000, 2000);
#else
	msleep(1);
#endif
	tmp16 = CY_PHYPWR_RSTCTL_IPRL;
	cyusb3610_write_cmd_nopm(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL,
			       2, 2, &tmp16);
	msleep(200);

	/* Ethernet PHY Auto Detach*/
	cyusb3610_AutoDetach(dev, 1);

	/* change clock */
	cyusb3610_read_cmd_nopm(dev, CY_ACCESS_MAC,  CY_CLK_SELECT,
			      1, 1, &tmp8, 0);
	tmp8 |= CY_CLK_SELECT_ACS | CY_CLK_SELECT_BCS;
	cyusb3610_write_cmd_nopm(dev, CY_ACCESS_MAC, CY_CLK_SELECT, 1, 1, &tmp8);
	msleep(100);

	/* Configure RX control register => start operation */
	tmp16 = CY_RX_CTL_DROPCRCERR | CY_RX_CTL_START | CY_RX_CTL_AP |
		 CY_RX_CTL_AMALL | CY_RX_CTL_AB;
	if (NET_IP_ALIGN == 0)
		tmp16 |= CY_RX_CTL_IPE;
	cyusb3610_write_cmd_nopm(dev, CY_ACCESS_MAC, CY_RX_CTL, 2, 2, &tmp16);

	return usbnet_resume(intf);
}

static void
cyusb3610_get_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{

	struct usbnet *dev = netdev_priv(net);
	u8 *opt = NULL;

	opt = kmalloc(1, GFP_KERNEL);
	if (!opt)
		return;

	if (cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_MONITOR_MODE,
			     1, 1, opt, 0) < 0) {
		wolinfo->supported = 0;
		wolinfo->wolopts = 0;
		kfree(opt);
		return;
	}

	wolinfo->supported = WAKE_PHY | WAKE_MAGIC;

	if (*opt & CY_MONITOR_MODE_RWLC)
		wolinfo->wolopts |= WAKE_PHY;
	if (*opt & CY_MONITOR_MODE_RWMP)
		wolinfo->wolopts |= WAKE_MAGIC;

	kfree(opt);
}

static int
cyusb3610_set_wol(struct net_device *net, struct ethtool_wolinfo *wolinfo)
{
	struct usbnet *dev = netdev_priv(net);
	u8 *opt = NULL;

	opt = kmalloc(1, GFP_KERNEL);
	if (!opt)
		return -ENOMEM;

	*opt = 0;

	if (wolinfo->wolopts & WAKE_PHY)
		*opt |= CY_MONITOR_MODE_RWLC;
	else
		*opt &= ~CY_MONITOR_MODE_RWLC;

	if (wolinfo->wolopts & WAKE_MAGIC)
		*opt |= CY_MONITOR_MODE_RWMP;
	else
		*opt &= ~CY_MONITOR_MODE_RWMP;

	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_MONITOR_MODE, 1, 1, opt);

	kfree(opt);

	return 0;
}

static int cyusb3610_get_eeprom_len(struct net_device *net)
{
	return CY_EEPROM_LEN;
}

static int
cyusb3610_get_eeprom(struct net_device *net, struct ethtool_eeprom *eeprom,
		   u8 *data)
{
	struct usbnet *dev = netdev_priv(net);
	u16 *eeprom_buff = NULL;
	int first_word = 0, last_word = 0;
	int i = 0;

	if (eeprom->len == 0)
		return -EINVAL;

	eeprom->magic = cyusb3610_EEPROM_MAGIC;

	first_word = eeprom->offset >> 1;
	last_word = (eeprom->offset + eeprom->len - 1) >> 1;
	eeprom_buff = kmalloc(sizeof(u16) * (last_word - first_word + 1),
			      GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	/* cyusb3610 returns 2 bytes from eeprom on read */
	for (i = first_word; i <= last_word; i++) {
		if (cyusb3610_read_cmd(dev, CY_ACCESS_EEPROM, i, 1, 2,
				     &(eeprom_buff[i - first_word]), 0) < 0) {
			kfree(eeprom_buff);
			return -EIO;
		}
	}

	memcpy(data, (u8 *)eeprom_buff + (eeprom->offset & 1), eeprom->len);
	kfree(eeprom_buff);
	return 0;
}

static void cyusb3610_get_drvinfo(struct net_device *net,
				struct ethtool_drvinfo *info)
{
	/* Inherit standard device info */
	usbnet_get_drvinfo(net, info);
	info->eedump_len = 0x3e;
}

static int cyusb3610_get_settings(struct net_device *net, struct ethtool_cmd *cmd)
{
	struct usbnet *dev = netdev_priv(net);
	return mii_ethtool_gset(&dev->mii, cmd);
}

static int cyusb3610_set_settings(struct net_device *net, struct ethtool_cmd *cmd)
{
	struct usbnet *dev = netdev_priv(net);
	return mii_ethtool_sset(&dev->mii, cmd);
}

static int cyusb3610_ioctl(struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);
	return  generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 28)
static int cyusb3610_netdev_stop(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	u16 tmp16 = 0;

	cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
			 2, 2, &tmp16, 1);
	tmp16 &= ~CY_MEDIUM_RECEIVE_EN;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
			  2, 2, &tmp16);
	return 0;
}
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
static int cyusb3610_set_csums(struct usbnet *dev)
{
	struct cyusb3610_data *cyusb_data = (struct cyusb3610_data *)dev->data;
	u8 checksum = 0;

	if (cyusb_data->checksum & CY_RX_CHECKSUM)
		checksum = CY_RXCOE_DEF_CSUM;
	else
		checksum = 0;

	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RXCOE_CTL, 1, 1, &checksum);

	if (cyusb_data->checksum & CY_TX_CHECKSUM)
		checksum = CY_TXCOE_DEF_CSUM;
	else
		checksum = 0;

	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_TXCOE_CTL, 1, 1, &checksum);

	return 0;
}

static u32 cyusb3610_get_tx_csum(struct net_device *netdev)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct cyusb3610_data *cyusb_data = (struct cyusb3610_data *)dev->data;
	return cyusb_data->checksum & CY_TX_CHECKSUM;
}

static u32 cyusb3610_get_rx_csum(struct net_device *netdev)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct cyusb3610_data *cyusb_data = (struct cyusb3610_data *)dev->data;
	return cyusb_data->checksum & CY_RX_CHECKSUM;
}

static int cyusb3610_set_rx_csum(struct net_device *netdev, u32 val)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct cyusb3610_data *cyusb_data = (struct cyusb3610_data *)dev->data;

	if (val)
		cyusb_data->checksum |= CY_RX_CHECKSUM;
	else
		cyusb_data->checksum &= ~CY_RX_CHECKSUM;
	return cyusb3610_set_csums(dev);
}

static int cyusb3610_set_tx_csum(struct net_device *netdev, u32 val)
{
	struct usbnet *dev = netdev_priv(netdev);
	struct cyusb3610_data *cyusb_data = (struct cyusb3610_data *)dev->data;

	if (val)
		cyusb_data->checksum |= CY_TX_CHECKSUM;
	else
		cyusb_data->checksum &= ~CY_TX_CHECKSUM;

	ethtool_op_set_tx_csum(netdev, val);

	return cyusb3610_set_csums(dev);
}

static int cyusb3610_set_tso(struct net_device *netdev, u32 data)
{
	if (data)
		netdev->features |= NETIF_F_TSO;
	else
		netdev->features &= ~NETIF_F_TSO;

	return 0;
}
#endif

static struct ethtool_ops cyusb3610_ethtool_ops = {
	.get_drvinfo		= cyusb3610_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_msglevel		= usbnet_get_msglevel,
	.set_msglevel		= usbnet_set_msglevel,
	.get_wol		= cyusb3610_get_wol,
	.set_wol		= cyusb3610_set_wol,
	.get_eeprom_len		= cyusb3610_get_eeprom_len,
	.get_eeprom		= cyusb3610_get_eeprom,
	.get_settings		= cyusb3610_get_settings,
	.set_settings		= cyusb3610_set_settings,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
	.set_tx_csum		= cyusb3610_set_tx_csum,
	.get_tx_csum		= cyusb3610_get_tx_csum,
	.get_rx_csum		= cyusb3610_get_rx_csum,
	.set_rx_csum		= cyusb3610_set_rx_csum,
	.get_tso		= ethtool_op_get_tso,
	.set_tso		= cyusb3610_set_tso,
	.get_sg			= ethtool_op_get_sg,
	.set_sg			= ethtool_op_set_sg
#endif
};

static void cyusb3610_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct cyusb3610_data *data = (struct cyusb3610_data *)&dev->data;
	u8 *m_filter = ((u8 *)dev->data) + 12;
	int mc_count = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
	mc_count = net->mc_count;
#else
	mc_count = netdev_mc_count(net);
#endif

	data->rxctl = (CY_RX_CTL_START | CY_RX_CTL_AB);
	if (NET_IP_ALIGN == 0)
		data->rxctl |= CY_RX_CTL_IPE;

	if (net->flags & IFF_PROMISC) {
		data->rxctl |= CY_RX_CTL_PRO;
	} else if (net->flags & IFF_ALLMULTI
		   || mc_count > CY_MAX_MCAST) {
		data->rxctl |= CY_RX_CTL_AMALL;
	} else if (mc_count == 0) {
		/* just broadcast and directed */
	} else {
		/* We use the 20 byte dev->data
		 * for our 8 byte filter buffer
		 * to avoid allocating memory that
		 * is tricky to free later */
		u32 crc_bits = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
		struct dev_mc_list *mc_list = net->mc_list;
		int i = 0;

		memset(m_filter, 0, CY_MCAST_FILTER_SIZE);

		/* Build the multicast hash filter. */
		for (i = 0; i < net->mc_count; i++) {
			crc_bits =
			    ether_crc(ETH_ALEN,
				      mc_list->dmi_addr) >> 26;
			*(m_filter + (crc_bits >> 3)) |=
				1 << (crc_bits & 7);
			mc_list = mc_list->next;
		}
#else
		struct netdev_hw_addr *ha = NULL;
		memset(m_filter, 0, CY_MCAST_FILTER_SIZE);
		netdev_for_each_mc_addr(ha, net) {
			crc_bits = ether_crc(ETH_ALEN, ha->addr) >> 26;
			*(m_filter + (crc_bits >> 3)) |=
				1 << (crc_bits & 7);
		}
#endif
		cyusb3610_write_cmd_async(dev, CY_ACCESS_MAC,
					CY_MULTI_FILTER_ARRY,
					CY_MCAST_FILTER_SIZE,
					CY_MCAST_FILTER_SIZE, m_filter);

		data->rxctl |= CY_RX_CTL_AM;
	}

	cyusb3610_write_cmd_async(dev, CY_ACCESS_MAC, CY_RX_CTL,
				2, 2, &data->rxctl);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
static int
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
cyusb3610_set_features(struct net_device *net, netdev_features_t features)
#else
cyusb3610_set_features(struct net_device *net, u32 features)
#endif

{
	u8 tmp = 0;
	struct usbnet *dev = netdev_priv(net);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	netdev_features_t changed = net->features ^ features;
#else
	u32 changed = net->features ^ features;
#endif

	if (changed & NETIF_F_IP_CSUM) {
		cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_TXCOE_CTL,
				 1, 1, &tmp, 0);
		tmp ^= CY_TXCOE_TCP | CY_TXCOE_UDP;
		cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_TXCOE_CTL, 1, 1, &tmp);
	}

	if (changed & NETIF_F_IPV6_CSUM) {
		cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_TXCOE_CTL,
				 1, 1, &tmp, 0);
		tmp ^= CY_TXCOE_TCPV6 | CY_TXCOE_UDPV6;
		cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_TXCOE_CTL, 1, 1, &tmp);
	}

	if (changed & NETIF_F_RXCSUM) {
		cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_RXCOE_CTL,
				 1, 1, &tmp, 0);
		tmp ^= CY_RXCOE_IP | CY_RXCOE_TCP | CY_RXCOE_UDP |
		       CY_RXCOE_TCPV6 | CY_RXCOE_UDPV6;
		cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RXCOE_CTL, 1, 1, &tmp);
	}

	return 0;
}
#endif

static int cyusb3610_change_mtu(struct net_device *net, int new_mtu)
{
	struct usbnet *dev = netdev_priv(net);
	u16 tmp16 = 0;

	if (new_mtu <= 0 || new_mtu > 4088)
		return -EINVAL;

	net->mtu = new_mtu;
	dev->hard_mtu = net->mtu + net->hard_header_len;

	if (net->mtu > 1500) {
		cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
				 2, 2, &tmp16, 1);
		tmp16 |= CY_MEDIUM_JUMBO_EN;
		cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
				  2, 2, &tmp16);
	} else {
		cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
				 2, 2, &tmp16, 1);
		tmp16 &= ~CY_MEDIUM_JUMBO_EN;
		cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
				  2, 2, &tmp16);
	}

	return 0;
}

static int cyusb3610_set_mac_addr(struct net_device *net, void *p)
{
	struct usbnet *dev = netdev_priv(net);
	struct sockaddr *addr = p;

	if (netif_running(net))
		return -EBUSY;
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(net->dev_addr, addr->sa_data, ETH_ALEN);

	/* Set the MAC address */
	return cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_NODE_ID, ETH_ALEN,
				 ETH_ALEN, net->dev_addr);

}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 29)
static const struct net_device_ops cyusb3610_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= usbnet_start_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_change_mtu		= cyusb3610_change_mtu,
	.ndo_do_ioctl		= cyusb3610_ioctl,
	.ndo_set_mac_address	= cyusb3610_set_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 2, 0)
	.ndo_set_multicast_list	= cyusb3610_set_multicast,
#else
	.ndo_set_rx_mode	= cyusb3610_set_multicast,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	.ndo_set_features	= cyusb3610_set_features,
#endif
};
#endif

static int cyusb3610_check_eeprom(struct usbnet *dev)
{
	u8 i = 0;
	u8 buf[2] = {0};
	u8 eeprom[20] = {0};
	u16 csum = 0, delay = HZ / 10;
	unsigned long jtimeout = 0;

	/* Read EEPROM content */
	for (i = 0 ; i < 6; i++) {

		buf[0] = i;
		if (cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_SROM_ADDR,
				      1, 1, buf) < 0)
			return -EINVAL;

		buf[0] = EEP_RD;
		if (cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_SROM_CMD,
				      1, 1, buf) < 0)
			return -EINVAL;

		jtimeout = jiffies + delay;
		do {
			cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_SROM_CMD,
					 1, 1, buf, 0);

			if (time_after(jiffies, jtimeout))
				return -EINVAL;
		} while (buf[0] & EEP_BUSY);

		cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_SROM_DATA_LOW,
				 2, 2, &eeprom[i * 2], 0);

		if ((i == 0) && (eeprom[0] == 0xFF))
			return -EINVAL;
	}

	csum = eeprom[6] + eeprom[7] + eeprom[8] + eeprom[9];
	csum = (csum >> 8) + (csum & 0xff);

	if ((csum + eeprom[10]) == 0xff)
		return CY_EEP_EFUSE_CORRECT;
	else
		return -EINVAL;
}

static int cyusb3610_check_efuse(struct usbnet *dev, void *ledmode)
{
	u8	i = 0;
	u8	efuse[64] = {0x00};
	u16	csum = 0;

	if (cyusb3610_read_cmd(dev, CY_ACCESS_EFUSE, 0, 64, 64, efuse, 0) < 0)
		return -EINVAL;

	if (efuse[0] == 0xFF)
		return -EINVAL;

	for (i = 0; i < 64; i++)
		csum = csum + efuse[i];

	while (csum > 255)
		csum = (csum & 0x00FF) + ((csum >> 8) & 0x00FF);

	if (csum == 0xFF) {
		memcpy((u8 *)ledmode, &efuse[51], 2);
		return CY_EEP_EFUSE_CORRECT;
	} else {
		return -EINVAL;
	}
}

static int cyusb3610_convert_old_led(struct usbnet *dev, u8 efuse, void *ledvalue)
{
	u8 ledmode = 0;
	u16 tmp = 0;
	u16 led = 0;

	/* loaded the old eFuse LED Mode */
	if (efuse) {
		if (cyusb3610_read_cmd(dev, CY_ACCESS_EFUSE, 0x18,
				     1, 2, &tmp, 1) < 0)
			return -EINVAL;
		ledmode = (u8)(tmp & 0xFF);
	} else { /* loaded the old EEprom LED Mode */
		if (cyusb3610_read_cmd(dev, CY_ACCESS_EEPROM, 0x3C,
				     1, 2, &tmp, 1) < 0)
			return -EINVAL;
		ledmode = (u8) (tmp >> 8);
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
	netdev_dbg(dev->net, "Old LED Mode = %02X\n", ledmode);
#else
	devdbg(dev, "Old LED Mode = %02X\n", ledmode);
#endif
	switch (ledmode) {
	case 0xFF:
		led = LED0_ACTIVE | LED1_LINK_10 | LED1_LINK_100 |
		      LED1_LINK_1000 | LED2_ACTIVE | LED2_LINK_10 |
		      LED2_LINK_100 | LED2_LINK_1000 | LED_VALID;
		break;
	case 0xFE:
		led = LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 | LED_VALID;
		break;
	case 0xFD:
		led = LED0_ACTIVE | LED1_LINK_1000 | LED2_LINK_100 |
		      LED2_LINK_10 | LED_VALID;
		break;
	case 0xFC:
		led = LED0_ACTIVE | LED1_ACTIVE | LED1_LINK_1000 | LED2_ACTIVE |
		      LED2_LINK_100 | LED2_LINK_10 | LED_VALID;
		break;
	default:
		led = LED0_ACTIVE | LED1_LINK_10 | LED1_LINK_100 |
		      LED1_LINK_1000 | LED2_ACTIVE | LED2_LINK_10 |
		      LED2_LINK_100 | LED2_LINK_1000 | LED_VALID;
		break;
	}

	memcpy((u8 *)ledvalue, &led, 2);

	return 0;
}

static int cyusb3610_led_setting(struct usbnet *dev)
{
	u8 ledfd = 0, value = 0;
	u16 tmp = 0, ledact = 0, ledlink = 0, ledvalue = 0, delay = HZ / 10;
	unsigned long jtimeout = 0;

	/* Check cyusb3610 version. UA1 or UA2 */
	cyusb3610_read_cmd(dev, CY_ACCESS_MAC, GENERAL_STATUS, 1, 1, &value, 0);

	/* UA1 */
	if (!(value & CY_SECLD)) {
		value = CY_GPIO_CTRL_GPIO3EN | CY_GPIO_CTRL_GPIO2EN |
			CY_GPIO_CTRL_GPIO1EN;
		if (cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_GPIO_CTRL,
				      1, 1, &value) < 0)
			return -EINVAL;
	}

	/* check EEprom */
	if (cyusb3610_check_eeprom(dev) == CY_EEP_EFUSE_CORRECT) {
		value = 0x42;
		if (cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_SROM_ADDR,
				      1, 1, &value) < 0)
			return -EINVAL;

		value = EEP_RD;
		if (cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_SROM_CMD,
				      1, 1, &value) < 0)
			return -EINVAL;

		jtimeout = jiffies + delay;
		do {
			cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_SROM_CMD,
					 1, 1, &value, 0);

			cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_SROM_CMD,
					 1, 1, &value, 0);

			if (time_after(jiffies, jtimeout))
				return -EINVAL;
		} while (value & EEP_BUSY);

		cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_SROM_DATA_HIGH,
				 1, 1, &value, 0);
		ledvalue = (value << 8);
		cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_SROM_DATA_LOW,
				 1, 1, &value, 0);
		ledvalue |= value;

		/* load internal ROM for defaule setting */
		if ((ledvalue == 0xFFFF) || ((ledvalue & LED_VALID) == 0))
			cyusb3610_convert_old_led(dev, 0, &ledvalue);

	} else if (cyusb3610_check_efuse(dev, &ledvalue) ==
				       CY_EEP_EFUSE_CORRECT) { /* check efuse */
		if ((ledvalue == 0xFFFF) || ((ledvalue & LED_VALID) == 0))
			cyusb3610_convert_old_led(dev, 0, &ledvalue);
	} else {
		cyusb3610_convert_old_led(dev, 0, &ledvalue);
	}

	tmp = GMII_PHY_PAGE_SELECT_EXT;
	cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
			  GMII_PHY_PAGE_SELECT, 2, &tmp);

	tmp = 0x2c;
	cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
			  GMII_PHYPAGE, 2, &tmp);

	cyusb3610_read_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
			 GMII_LED_ACTIVE, 2, &ledact, 1);

	cyusb3610_read_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
			 GMII_LED_LINK, 2, &ledlink, 1);

	ledact &= GMII_LED_ACTIVE_MASK;
	ledlink &= GMII_LED_LINK_MASK;

	if (ledvalue & LED0_ACTIVE)
		ledact |= GMII_LED0_ACTIVE;
	if (ledvalue & LED1_ACTIVE)
		ledact |= GMII_LED1_ACTIVE;
	if (ledvalue & LED2_ACTIVE)
		ledact |= GMII_LED2_ACTIVE;

	if (ledvalue & LED0_LINK_10)
		ledlink |= GMII_LED0_LINK_10;
	if (ledvalue & LED1_LINK_10)
		ledlink |= GMII_LED1_LINK_10;
	if (ledvalue & LED2_LINK_10)
		ledlink |= GMII_LED2_LINK_10;

	if (ledvalue & LED0_LINK_100)
		ledlink |= GMII_LED0_LINK_100;
	if (ledvalue & LED1_LINK_100)
		ledlink |= GMII_LED1_LINK_100;
	if (ledvalue & LED2_LINK_100)
		ledlink |= GMII_LED2_LINK_100;

	if (ledvalue & LED0_LINK_1000)
		ledlink |= GMII_LED0_LINK_1000;
	if (ledvalue & LED1_LINK_1000)
		ledlink |= GMII_LED1_LINK_1000;
	if (ledvalue & LED2_LINK_1000)
		ledlink |= GMII_LED2_LINK_1000;

	tmp = ledact;
	cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
			  GMII_LED_ACTIVE, 2, &tmp);

	tmp = ledlink;
	cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
			  GMII_LED_LINK, 2, &tmp);

	tmp = GMII_PHY_PAGE_SELECT_PAGE0;
	cyusb3610_write_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
			  GMII_PHY_PAGE_SELECT, 2, &tmp);

	/* LED full duplex setting */
	ledfd = 0;
	if (ledvalue & LED0_FD)
		ledfd |= 0x01;
	else if ((ledvalue & LED0_USB3_MASK) == 0)
		ledfd |= 0x02;


	if (ledvalue & LED1_FD)
		ledfd |= 0x04;
	else if ((ledvalue & LED1_USB3_MASK) == 0)
		ledfd |= 0x08;

	if (ledvalue & LED2_FD) /* LED2_FD */
		ledfd |= 0x10;
	else if ((ledvalue & LED2_USB3_MASK) == 0) /* LED2_USB3 */
		ledfd |= 0x20;

	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, 0x73, 1, 1, &ledfd);

	return 0;
}

static int cyusb3610_AutoDetach(struct usbnet *dev, int in_pm)
{
	u16 tmp16 = 0;
	u8 tmp8 = 0;
	int (*fnr)(struct usbnet *, u8, u16, u16, u16, void *, int);
	int (*fnw)(struct usbnet *, u8, u16, u16, u16, void *);

	if (!in_pm) {
		fnr = cyusb3610_read_cmd;
		fnw = cyusb3610_write_cmd;
	} else {
		fnr = cyusb3610_read_cmd_nopm;
		fnw = cyusb3610_write_cmd_nopm;
	}

	if (fnr(dev, CY_ACCESS_EEPROM, 0x43, 1, 2, &tmp16, 1) < 0)
		return 0;

	if ((tmp16 == 0xFFFF) || (!(tmp16 & 0x0100)))
		return 0;

	/* Enable Auto Detach bit */
	tmp8 = 0;
	fnr(dev, CY_ACCESS_MAC, CY_CLK_SELECT, 1, 1, &tmp8, 0);
	tmp8 |= CY_CLK_SELECT_ULR;
	fnw(dev, CY_ACCESS_MAC, CY_CLK_SELECT, 1, 1, &tmp8);

	fnr(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL, 2, 2, &tmp16, 1);
	tmp16 |= CY_PHYPWR_RSTCTL_AUTODETACH;
	fnw(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL, 2, 2, &tmp16);

	return 0;
}

static int access_eeprom_mac(struct usbnet *dev, u8 *buf, u8 offset, bool wflag)
{
	int ret = 0, i;
	u16* tmp = (u16*)buf;

	for (i = 0; i < (ETH_ALEN >> 1); i++) {
		if (wflag) {
			u16 wd = cpu_to_le16(*(tmp + i));
			ret = cyusb3610_write_cmd(dev, CY_ACCESS_EEPROM,
						offset + i, 1, 2, &wd);
			if (ret < 0)
				break;

			mdelay(15);
		}
		else {
			ret = cyusb3610_read_cmd(dev, CY_ACCESS_EEPROM,
						offset + i, 1, 2, tmp + i, 0);
			if (ret < 0)
				break;
		}
	}

	if (!wflag) {
		if (ret < 0) {
			#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
				netdev_dbg(dev->net, "Failed to read MAC address from EEPROM: %d\n", ret);
			#else
				devdbg(dev, "Failed to read MAC address from EEPROM: %d\n", ret);
			#endif
			return ret;
		}
		memcpy(dev->net->dev_addr, buf, ETH_ALEN);
	}
	else {
		/* reload eeprom data */
		ret = cyusb3610_write_cmd(dev, CY_RELOAD_EEPROM_EFUSE, 0, 0, 0, 0);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int cyusb3610_check_ether_addr(struct usbnet *dev)
{
	unsigned char *tmp = (unsigned char*)dev->net->dev_addr;
	u8 default_mac[6] = {0, 0x0e, 0xc6, 0x81, 0x79, 0x01};
	u8 default_mac_cyusb_a[6] = {0, 0x0e, 0xc6, 0x81, 0x78, 0x01};

	if (((*((u8*)tmp) == 0) && (*((u8*)tmp + 1) == 0) && (*((u8*)tmp + 2) == 0)) ||
	    !is_valid_ether_addr((u8*)tmp) ||
	    !memcmp(dev->net->dev_addr, default_mac, ETH_ALEN) ||
	    !memcmp(dev->net->dev_addr, default_mac_cyusb_a, ETH_ALEN)) {
		int i;

		printk("Found invalid EEPROM MAC address value ");

		for (i = 0; i < ETH_ALEN; i++) {
			printk("%02X", *((u8*)tmp + i));
			if (i != 5)
				printk("-");
		}
		printk("\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
		eth_hw_addr_random(dev->net);
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
		dev->net->addr_assign_type |= NET_ADDR_RANDOM;
#endif
		random_ether_addr(dev->net->dev_addr); 
#endif
		*tmp = 0;
		*(tmp + 1) = 0x0E;
		*(tmp + 2) = 0xC6;
		*(tmp + 3) = 0x8E;

		return -EADDRNOTAVAIL;	
	} 
	return 0;
}

static int cyusb3610_get_mac(struct usbnet *dev, u8* buf)
{
	int ret, i;

	ret = access_eeprom_mac(dev, buf, 0x0, 0);
	if (ret < 0)
		goto out;

	if (cyusb3610_check_ether_addr(dev)) {
		ret = access_eeprom_mac(dev, dev->net->dev_addr, 0x0, 1);
		if (ret < 0) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Failed to write MAC to EEPROM: %d", ret);
#else
		deverr(dev, "Failed to write MAC to EEPROM: %d", ret);
#endif
			goto out;
		}

		msleep(5);

		ret = cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_NODE_ID,
				       ETH_ALEN, ETH_ALEN, buf, 0);
		if (ret < 0) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Failed to read MAC address: %d", ret);
#else
		deverr(dev, "Failed to read MAC address: %d", ret);
#endif
			goto out;
		}

		for (i = 0; i < ETH_ALEN; i++)
			if (*(dev->net->dev_addr + i) != *((u8*)buf + i)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_warn(dev->net, "Found invalid EEPROM part or non-EEPROM");
#else
		devwarn(dev, "Found invalid EEPROM part or non-EEPROM");
#endif
				break;
			}
	}

	memcpy(dev->net->perm_addr, dev->net->dev_addr, ETH_ALEN);

	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_NODE_ID, ETH_ALEN,
			  ETH_ALEN, dev->net->dev_addr);
	
	if (ret < 0) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Failed to write MAC address: %d", ret);
#else
		deverr(dev, "Failed to write MAC address: %d", ret);
#endif
		goto out;
	}

	return 0;
out:
	return ret;
}

static int cyusb3610_bind(struct usbnet *dev, struct usb_interface *intf)
{
	void *buf = NULL;
	u16 *tmp16 = NULL;
	u8 *tmp = NULL;
	int ret;

	struct cyusb3610_data *cyusb_data = (struct cyusb3610_data *)dev->data;

	usbnet_get_endpoints(dev, intf);

	if (msg_enable != 0)
		dev->msg_enable = msg_enable;

	buf = kmalloc(6, GFP_KERNEL);
	if (!buf) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Cannot allocate memory for buffer");
#else
		deverr(dev, "Cannot allocate memory for buffer");
#endif
		return -ENOMEM;
	}
	tmp16 = (u16 *)buf;
	tmp = (u8 *)buf;

	memset(cyusb_data, 0, sizeof(*cyusb_data));

	/* Power up ethernet PHY */
	*tmp16 = 0;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL, 2, 2, tmp16);
	*tmp16 = CY_PHYPWR_RSTCTL_IPRL;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL, 2, 2, tmp16);
	msleep(200);

	*tmp = CY_CLK_SELECT_ACS | CY_CLK_SELECT_BCS;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_CLK_SELECT, 1, 1, tmp);
	msleep(100);

	/* Get the MAC address */
	memset(buf, 0, ETH_ALEN);
	ret = cyusb3610_get_mac(dev, buf);
	if (ret)
		goto out;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_dbg(dev->net, "MAC [%02x-%02x-%02x-%02x-%02x-%02x]\n",
			   dev->net->dev_addr[0], dev->net->dev_addr[1],
			   dev->net->dev_addr[2], dev->net->dev_addr[3],
			   dev->net->dev_addr[4], dev->net->dev_addr[5]);
#else
		devdbg(dev, "MAC [%02x-%02x-%02x-%02x-%02x-%02x]\n",
		       dev->net->dev_addr[0], dev->net->dev_addr[1],
		       dev->net->dev_addr[2], dev->net->dev_addr[3],
		       dev->net->dev_addr[4], dev->net->dev_addr[5]);
#endif

	/* RX bulk configuration, default for USB3.0 to Giga*/
	memcpy(tmp, &CYUSB3610_BULKIN_SIZE[0], 5);
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RX_BULKIN_QCTRL, 5, 5, tmp);

	dev->rx_urb_size = 1024 * 20;

	tmp[0] = 0x34;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_PAUSE_WATERLVL_LOW, 1, 1, tmp);

	tmp[0] = 0x52;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_PAUSE_WATERLVL_HIGH,
			  1, 1, tmp);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 30)
	dev->net->do_ioctl = cyusb3610_ioctl;
	dev->net->set_multicast_list = cyusb3610_set_multicast;
	dev->net->set_mac_address = cyusb3610_set_mac_addr;
	dev->net->change_mtu = cyusb3610_change_mtu;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 28)
	dev->net->stop = cyusb3610_netdev_stop;
#endif
#else
	dev->net->netdev_ops = &cyusb3610_netdev_ops;
#endif

	dev->net->ethtool_ops = &cyusb3610_ethtool_ops;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 30)
	dev->net->needed_headroom = 8;
#endif

	/* Initialize MII structure */
	dev->mii.dev = dev->net;
	dev->mii.mdio_read = cyusb3610_mdio_read;
	dev->mii.mdio_write = cyusb3610_mdio_write;
	dev->mii.phy_id_mask = 0xff;
	dev->mii.reg_num_mask = 0xff;
	dev->mii.phy_id = 0x03;
	dev->mii.supports_gmii = 1;

	dev->net->features |= NETIF_F_IP_CSUM;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 22)
	dev->net->features |= NETIF_F_IPV6_CSUM;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	if (usb_device_no_sg_constraint(dev->udev))
		dev->can_dma_sg = 1;
	dev->net->features |= NETIF_F_SG | NETIF_F_TSO;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	dev->net->hw_features |= NETIF_F_IP_CSUM;
	dev->net->hw_features |= NETIF_F_IPV6_CSUM;
	dev->net->hw_features |= NETIF_F_SG | NETIF_F_TSO;
#endif

	/* Enable checksum offload */
	*tmp = CY_RXCOE_IP | CY_RXCOE_TCP | CY_RXCOE_UDP |
	       CY_RXCOE_TCPV6 | CY_RXCOE_UDPV6;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RXCOE_CTL, 1, 1, tmp);

	*tmp = CY_TXCOE_IP | CY_TXCOE_TCP | CY_TXCOE_UDP |
	       CY_TXCOE_TCPV6 | CY_TXCOE_UDPV6;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_TXCOE_CTL, 1, 1, tmp);

	cyusb_data->checksum |= CY_RX_CHECKSUM | CY_TX_CHECKSUM;

	/* Configure RX control register => start operation */
	*tmp16 = CY_RX_CTL_DROPCRCERR | CY_RX_CTL_START | CY_RX_CTL_AP |
		 CY_RX_CTL_AMALL | CY_RX_CTL_AB;
	if (NET_IP_ALIGN == 0)
		*tmp16 |= CY_RX_CTL_IPE;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RX_CTL, 2, 2, tmp16);

	*tmp = CY_MONITOR_MODE_PMETYPE | CY_MONITOR_MODE_PMEPOL |
						CY_MONITOR_MODE_RWMP;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_MONITOR_MODE, 1, 1, tmp);

	cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_MONITOR_MODE, 1, 1, tmp, 0);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_dbg(dev->net, "Monitor mode = 0x%02x\n", *tmp);
#else
		devdbg(dev, "Monitor mode = 0x%02x\n", *tmp);
#endif
	/* Configure default medium type => giga */
	*tmp16 = CY_MEDIUM_RECEIVE_EN	 | CY_MEDIUM_TXFLOW_CTRLEN |
		 CY_MEDIUM_RXFLOW_CTRLEN | CY_MEDIUM_FULL_DUPLEX   |
		 CY_MEDIUM_GIGAMODE;

	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
			  2, 2, tmp16);

	cyusb3610_led_setting(dev);

	cyusb3610_EEE_setting(dev);

	cyusb3610_Gether_setting(dev);

	/* Restart autoneg */
	mii_nway_restart(&dev->mii);

	netif_carrier_off(dev->net);

	kfree(buf);
	printk(version);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_info(dev->net, "mtu %d\n", dev->net->mtu);
#else
		devinfo(dev, "mtu %d\n", dev->net->mtu);
#endif
	return 0;

out:
	kfree(buf);
	return ret;

}

static void cyusb3610_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	u16 tmp16 = 0;
	u8 tmp8 = 0;
	struct cyusb3610_data *cyusb_data = (struct cyusb3610_data *) dev->data;

	if (cyusb_data) {
		/* Configure RX control register => stop operation */
		tmp16 = CY_RX_CTL_STOP;
		cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RX_CTL, 2, 2, &tmp16);

		tmp8 = 0x0;
		cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_CLK_SELECT,
				  1, 1, &tmp8);

		/* Power down ethernet PHY */
		tmp16 = CY_PHYPWR_RSTCTL_BZ | CY_PHYPWR_RSTCTL_IPRL;
		cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL,
				  2, 2, &tmp16);
		msleep(200);
	}
}

static void
cyusb3610_rx_checksum(struct sk_buff *skb, u32 *pkt_hdr)
{
	skb->ip_summed = CHECKSUM_NONE;

	/* checksum error bit is set */
	if ((*pkt_hdr & CY_RXHDR_L3CSUM_ERR) ||
	    (*pkt_hdr & CY_RXHDR_L4CSUM_ERR))
		return;

	/* It must be a TCP or UDP packet with a valid checksum */
	if (((*pkt_hdr & CY_RXHDR_L4_TYPE_MASK) == CY_RXHDR_L4_TYPE_TCP) ||
	    ((*pkt_hdr & CY_RXHDR_L4_TYPE_MASK) == CY_RXHDR_L4_TYPE_UDP))
		skb->ip_summed = CHECKSUM_UNNECESSARY;
}

static int cyusb3610_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	struct sk_buff *CY_skb = NULL;
	int pkt_cnt = 0;
	u32 rx_hdr = 0;
	u16 hdr_off = 0;
	u32 *pkt_hdr = NULL;

	skb_trim(skb, skb->len - 4);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 22)
	memcpy(&rx_hdr, skb_tail_pointer(skb), sizeof(rx_hdr));
#else
	memcpy(&rx_hdr, skb->tail, sizeof(rx_hdr));
#endif
	le32_to_cpus(&rx_hdr);

	pkt_cnt = (u16)rx_hdr;
	hdr_off = (u16)(rx_hdr >> 16);
	pkt_hdr = (u32 *)(skb->data + hdr_off);

	while (pkt_cnt--) {
		u16 pkt_len;

		le32_to_cpus(pkt_hdr);
		pkt_len = (*pkt_hdr >> 16) & 0x1fff;

		/* Check CRC or runt packet */
		if ((*pkt_hdr & CY_RXHDR_CRC_ERR) ||
		    (*pkt_hdr & CY_RXHDR_DROP_ERR)) {
			skb_pull(skb, (pkt_len + 7) & 0xFFF8);
			pkt_hdr++;
			continue;
		}

		if (pkt_cnt == 0) {			
			skb->len = pkt_len;

			/* Skip IP alignment psudo header */
			if (NET_IP_ALIGN == 0)
				skb_pull(skb, 2);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
			skb->tail = skb->data + skb->len;
#else
			skb_set_tail_pointer(skb, skb->len);
#endif
			skb->truesize = skb->len + sizeof(struct sk_buff);
			cyusb3610_rx_checksum(skb, pkt_hdr);

			return 1;
		}

#ifndef RX_SKB_COPY
		CY_skb = skb_clone(skb, GFP_ATOMIC);
#else
		CY_skb = alloc_skb(pkt_len + NET_IP_ALIGN, GFP_ATOMIC);
		skb_reserve(CY_skb, NET_IP_ALIGN);
#endif

		if (CY_skb) {
#ifndef RX_SKB_COPY
			CY_skb->len = pkt_len;
	
			/* Skip IP alignment psudo header */
			if (NET_IP_ALIGN == 0)
				skb_pull(CY_skb, 2);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
			CY_skb->tail = CY_skb->data + CY_skb->len;
#else
			skb_set_tail_pointer(CY_skb, CY_skb->len);
#endif

#else
			skb_put(CY_skb, pkt_len);
			memcpy(CY_skb->data, skb->data, pkt_len);

			if (NET_IP_ALIGN == 0)
				skb_pull(CY_skb, 2);
#endif
			CY_skb->truesize = CY_skb->len + sizeof(struct sk_buff);
			cyusb3610_rx_checksum(CY_skb, pkt_hdr);
			usbnet_skb_return(dev, CY_skb);
		} else {
			return 0;
		}

		skb_pull(skb, (pkt_len + 7) & 0xFFF8);
		pkt_hdr++;
	}
	return 1;
}

static struct sk_buff *
cyusb3610_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
{
	u32 tx_hdr1 = 0, tx_hdr2 = 0;
	int frame_size = dev->maxpacket;
	int mss = skb_shinfo(skb)->gso_size;
	int headroom = 0;
	int tailroom = 0;

	tx_hdr1 = skb->len;
	tx_hdr2 = mss;
	if (((skb->len + 8) % frame_size) == 0)
		tx_hdr2 |= 0x80008000;	/* Enable padding */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	if (!dev->can_dma_sg && (dev->net->features & NETIF_F_SG) &&
	    skb_linearize(skb))
		return NULL;
#else
	if ((dev->net->features & NETIF_F_SG) && skb_linearize(skb))
		return NULL;
#endif

	headroom = skb_headroom(skb);
	tailroom = skb_tailroom(skb);

	if ((headroom + tailroom) >= 8) {
		if (headroom < 8) {
			skb->data = memmove(skb->head + 8, skb->data, skb->len);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
			skb->tail = skb->data + skb->len;
#else
			skb_set_tail_pointer(skb, skb->len);
#endif
		}
	} else {
		struct sk_buff *skb2 = NULL;
		skb2 = skb_copy_expand(skb, 8, 0, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

	skb_push(skb, 4);
	cpu_to_le32s(&tx_hdr2);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
	memcpy(skb->data, &tx_hdr2, 4);
#else
	skb_copy_to_linear_data(skb, &tx_hdr2, 4);
#endif

	skb_push(skb, 4);
	cpu_to_le32s(&tx_hdr1);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
	memcpy(skb->data, &tx_hdr1, 4);
#else
	skb_copy_to_linear_data(skb, &tx_hdr1, 4);
#endif

	return skb;
}

static int cyusb3610_link_reset(struct usbnet *dev)
{
	struct cyusb3610_data *data = (struct cyusb3610_data *)&dev->data;
	u8 tmp[5] = {0}, link_sts = 0;
	u16 mode = 0, tmp16 = 0, delay = HZ/10;
	u32 tmp32 = 0x40000000;
	unsigned long jtimeout = 0;

	jtimeout = jiffies + delay;

	while (tmp32 & 0x40000000) {
		mode = 0;
		cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RX_CTL, 2, 2, &mode);
		cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RX_CTL,
				  2, 2, &data->rxctl);

		/* link up, check the usb device control TX FIFO full or empty*/
		cyusb3610_read_cmd(dev, 0x81, 0x8c, 0, 4, &tmp32, 1);

		if (time_after(jiffies, jtimeout))
			return 0;
	}

	mode = CY_MEDIUM_RECEIVE_EN    | CY_MEDIUM_TXFLOW_CTRLEN |
		   CY_MEDIUM_RXFLOW_CTRLEN;

	cyusb3610_read_cmd(dev, CY_ACCESS_MAC, PHYSICAL_LINK_STATUS,
			 1, 1, &link_sts, 0);
	cyusb3610_read_cmd(dev, CY_ACCESS_PHY, CYUSB3610_PHY_ID,
			 GMII_PHY_PHYSR, 2, &tmp16, 1);

	if (!(tmp16 & GMII_PHY_PHYSR_LINK))
		return 0;
	else if (GMII_PHY_PHYSR_GIGA == (tmp16 & GMII_PHY_PHYSR_SMASK)) {
		mode |= CY_MEDIUM_GIGAMODE | CY_MEDIUM_EN_125MHZ;
		if (dev->net->mtu > 1500)
			mode |= CY_MEDIUM_JUMBO_EN;

		if (link_sts & CY_USB_SS)
			memcpy(tmp, &CYUSB3610_BULKIN_SIZE[0], 5);
		else if (link_sts & CY_USB_HS)
			memcpy(tmp, &CYUSB3610_BULKIN_SIZE[1], 5);
		else
			memcpy(tmp, &CYUSB3610_BULKIN_SIZE[3], 5);
	} else if (GMII_PHY_PHYSR_100 == (tmp16 & GMII_PHY_PHYSR_SMASK)) {
		mode |= CY_MEDIUM_PS;	/* Bit 9 : PS */
		if (link_sts & (CY_USB_SS | CY_USB_HS))
			memcpy(tmp, &CYUSB3610_BULKIN_SIZE[2], 5);
		else
			memcpy(tmp, &CYUSB3610_BULKIN_SIZE[3], 5);
	} else
		memcpy(tmp, &CYUSB3610_BULKIN_SIZE[3], 5);

	if (bsize != -1) {
		if (bsize > 24)
			bsize = 24;

		else if (bsize == 0) {
			tmp[1] = 0;
			tmp[2] = 0;
		}

		tmp[3] = (u8)bsize;
	}

	if (ifg != -1) {
		if (ifg > 255)
			ifg = 255;
		tmp[4] = (u8)ifg;
	}

	/* RX bulk configuration */
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RX_BULKIN_QCTRL, 5, 5, tmp);

	if (tmp16 & GMII_PHY_PHYSR_FULL)
		mode |= CY_MEDIUM_FULL_DUPLEX;	/* Bit 1 : FD */

	dev->rx_urb_size = (1024 * (tmp[3] + 2));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_info(dev->net, "Write medium type: 0x%04x\n", mode);
#else
		devinfo(dev, "Write medium type: 0x%04x\n", mode);
#endif
	/* Configure default medium type => giga */
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
			  2, 2, &mode);
	mii_check_media(&dev->mii, 1, 1);

	return 0;
}

static int cyusb3610_reset(struct usbnet *dev)
{
	void *buf = NULL;
	u16 *tmp16 = NULL;
	u8 *tmp = NULL;
	struct cyusb3610_data *cyusb_data = (struct cyusb3610_data *) dev->data;
	buf = kmalloc(6, GFP_KERNEL);

	if (!buf) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
		netdev_err(dev->net, "Cannot allocate memory for buffer");
#else
		deverr(dev, "Cannot allocate memory for buffer");
#endif
		return -ENOMEM;
	}

	tmp16 = (u16 *)buf;
	tmp = (u8 *)buf;

	/* Power up ethernet PHY */
	*tmp16 = 0;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL, 2, 2, tmp16);
	*tmp16 = CY_PHYPWR_RSTCTL_IPRL;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_PHYPWR_RSTCTL, 2, 2, tmp16);
	msleep(200);

	*tmp = CY_CLK_SELECT_ACS | CY_CLK_SELECT_BCS;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_CLK_SELECT, 1, 1, tmp);
	msleep(100);

	/* Ethernet PHY Auto Detach*/
	cyusb3610_AutoDetach(dev, 0);

	/* Set the MAC address */
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_NODE_ID, ETH_ALEN,
			  ETH_ALEN, dev->net->dev_addr);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
	netdev_dbg(dev->net, "MAC [%02x-%02x-%02x-%02x-%02x-%02x]\n",
	dev->net->dev_addr[0], dev->net->dev_addr[1],
	dev->net->dev_addr[2], dev->net->dev_addr[3],
	dev->net->dev_addr[4], dev->net->dev_addr[5]);
#else
	devdbg(dev, "MAC [%02x-%02x-%02x-%02x-%02x-%02x]\n",
	dev->net->dev_addr[0], dev->net->dev_addr[1],
	dev->net->dev_addr[2], dev->net->dev_addr[3],
	dev->net->dev_addr[4], dev->net->dev_addr[5]);
#endif

	/* RX bulk configuration */
	memcpy(tmp, &CYUSB3610_BULKIN_SIZE[0], 5);
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RX_BULKIN_QCTRL, 5, 5, tmp);

	dev->rx_urb_size = 1024 * 20;

	tmp[0] = 0x34;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_PAUSE_WATERLVL_LOW, 1, 1, tmp);

	tmp[0] = 0x52;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_PAUSE_WATERLVL_HIGH,
			  1, 1, tmp);

	dev->net->features |= NETIF_F_IP_CSUM;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 22)
	dev->net->features |= NETIF_F_IPV6_CSUM;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0)
	if (usb_device_no_sg_constraint(dev->udev))
		dev->can_dma_sg = 1;
	dev->net->features |= NETIF_F_SG | NETIF_F_TSO;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	dev->net->hw_features |= NETIF_F_IP_CSUM;
	dev->net->hw_features |= NETIF_F_IPV6_CSUM;
	dev->net->hw_features |= NETIF_F_SG | NETIF_F_TSO;
#endif

	/* Enable checksum offload */
	*tmp = CY_RXCOE_IP | CY_RXCOE_TCP | CY_RXCOE_UDP |
	       CY_RXCOE_TCPV6 | CY_RXCOE_UDPV6;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RXCOE_CTL, 1, 1, tmp);

	*tmp = CY_TXCOE_IP | CY_TXCOE_TCP | CY_TXCOE_UDP |
	       CY_TXCOE_TCPV6 | CY_TXCOE_UDPV6;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_TXCOE_CTL, 1, 1, tmp);

	cyusb_data->checksum |= CY_RX_CHECKSUM | CY_TX_CHECKSUM;

	/* Configure RX control register => start operation */
	*tmp16 = CY_RX_CTL_DROPCRCERR | CY_RX_CTL_START | CY_RX_CTL_AP |
		 CY_RX_CTL_AMALL | CY_RX_CTL_AB;
	if (NET_IP_ALIGN == 0)
		*tmp16 |= CY_RX_CTL_IPE;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_RX_CTL, 2, 2, tmp16);

	*tmp = CY_MONITOR_MODE_PMETYPE | CY_MONITOR_MODE_PMEPOL |
						CY_MONITOR_MODE_RWMP;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_MONITOR_MODE, 1, 1, tmp);

	cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_MONITOR_MODE, 1, 1, tmp, 0);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
	netdev_dbg(dev->net, "Monitor mode = 0x%02x\n", *tmp);
#else
	devdbg(dev, "Monitor mode = 0x%02x\n", *tmp);
#endif

	/* Configure default medium type => giga */
	*tmp16 = CY_MEDIUM_RECEIVE_EN	 | CY_MEDIUM_TXFLOW_CTRLEN |
		 CY_MEDIUM_RXFLOW_CTRLEN | CY_MEDIUM_FULL_DUPLEX   |
		 CY_MEDIUM_GIGAMODE;

	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
			  2, 2, tmp16);

	cyusb3610_led_setting(dev);

	cyusb3610_EEE_setting(dev);

	cyusb3610_Gether_setting(dev);

	/* Restart autoneg */
	mii_nway_restart(&dev->mii);

	netif_carrier_off(dev->net);

	kfree(buf);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
	netdev_dbg(dev->net, "mtu %d\n", dev->net->mtu);
#else
	devdbg(dev, "mtu %d\n", dev->net->mtu);
#endif

	return 0;

}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
static int cyusb3610_stop(struct usbnet *dev)
{
	u16 tmp16 = 0;

	cyusb3610_read_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
			 2, 2, &tmp16, 1);
	tmp16 &= ~CY_MEDIUM_RECEIVE_EN;
	cyusb3610_write_cmd(dev, CY_ACCESS_MAC, CY_MEDIUM_STATUS_MODE,
			  2, 2, &tmp16);
	return 0;
}
#endif

static const struct driver_info cyusb3610_info = {
	.description = "Cypress CYUSB3610 USB 3.0 Gigabit Ethernet",
	.bind	= cyusb3610_bind,
	.unbind	= cyusb3610_unbind,
	.status	= cyusb3610_status,
	.link_reset = cyusb3610_link_reset,
	.reset	= cyusb3610_reset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
	.stop	= cyusb3610_stop,
	.flags	= FLAG_ETHER | FLAG_FRAMING_AX | FLAG_AVOID_UNLINK_URBS,
#else
	.flags	= FLAG_ETHER | FLAG_FRAMING_AX,
#endif
	.rx_fixup = cyusb3610_rx_fixup,
	.tx_fixup = cyusb3610_tx_fixup,
};


static const struct usb_device_id	products[] = {
{
	/* Cypress cyusb3610 10/100/1000 */
	USB_DEVICE(0x04b4, 0x3610),
	.driver_info = (unsigned long) &cyusb3610_info,
}, 
	{ },		/* END */
};
MODULE_DEVICE_TABLE(usb, products);

static struct usb_driver cyusb3610_driver = {
	.name =		"cyusb3610",
	.id_table =	products,
	.probe =	usbnet_probe,
	.suspend =	cyusb3610_suspend,
	.resume =	cyusb3610_resume,
	.disconnect =	usbnet_disconnect,
};


static int __init cyusb3610_init(void)
{
	return usb_register(&cyusb3610_driver);
}
module_init(cyusb3610_init);

static void __exit cyusb3610_exit(void)
{
	usb_deregister(&cyusb3610_driver);
}
module_exit(cyusb3610_exit);

MODULE_AUTHOR("David Hollis");
MODULE_DESCRIPTION("Cypress cyusb3610 based USB 2.0/3.0 Gigabit Ethernet Devices");
MODULE_LICENSE("GPL");
