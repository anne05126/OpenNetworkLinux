/*
 * A IOBM IO and EEPROM IPMI kernel dirver for extremenetworks 8730-32d
 *
 * Copyright (C) 2023 Alphanetworks Technology Corporation.
 * Anne Liou <anne_liou@alphanetworks.com>
 * 
 * Based on:
 * Copyright (C) 2014 Accton Technology Corporation.
 * Brandon Chuang <brandon_chuang@accton.com.tw>
 *
 * Based on ad7414.c
 * Copyright 2006 Stefan Roese <sr at denx.de>, DENX Software Engineering
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/ipmi.h>
#include <linux/ipmi_smi.h>
#include <linux/platform_device.h>

#define DRVNAME "8730_iobm_io_eeprom"

#define IPMI_SENSOR_NETFN           0x34
#define IPMI_SFP_IO_READ_CMD 		0x1A
#define IPMI_SFP_IO_WRITE_CMD 		0x1B
#define IPMI_EEPROM_READ_CMD 		0x1C
#define IPMI_QSFP28_IO_READ_CMD 	0x1D
#define IPMI_QSFP28_IO_WRITE_CMD 	0x1E

#define IPMI_IOBM_OFFSET_SFP1		0x01
#define IPMI_IOBM_OFFSET_SFP2		0x02
#define IPMI_IOBM_OFFSET_QSFP1		0x03

#define IPMI_PAGE_A0_READ_CMD		0x01
#define IPMI_PAGE_A2_READ_CMD		0x02

#define IPMI_TIMEOUT				(20 * HZ)
#define IPMI_ERR_RETRY_TIMES		1

#define EEPROM_SIZE      			256

static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");

#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)

static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data);
static ssize_t set_io(struct device *dev, struct device_attribute *da, const char *buf, size_t count);
static ssize_t show_io(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_eeprom(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_dom(struct device *dev, struct device_attribute *attr, char *buf);
static int extreme8730_32d_iobm_probe(struct platform_device *pdev);
static int extreme8730_32d_iobm_remove(struct platform_device *pdev);

enum iobm_id {
	SFP_1,
	SFP_2,
	QSFP28_1,
	NUM_OF_IOBM
};

enum sfp_io_index {
	SFP_PRESENT = 0,
	SFP_TXFAULT,
	SFP_RXLOS,
	SFP_TXDIS,
	NUM_OF_SFP_IO
};

enum qsfp28_status_bit_index {
	QSFP_MOD_SEL = 0,
	QSFP_LP_MODE,
	QSFP_RST_MOD,
	QSFP_MOD_PRESENT
};

typedef struct ipmi_user *ipmi_user_t;

struct ipmi_data {
	struct completion	read_complete;
	struct ipmi_addr	address;
	ipmi_user_t 		user;
	int 				interface;

	struct kernel_ipmi_msg tx_message;
	long				   tx_msgid;

	void			*rx_msg_data;
	unsigned short	 rx_msg_len;
	unsigned char	 rx_result;
	int 			 rx_recv_type;

	struct ipmi_user_hndl ipmi_hndlrs;
};

struct ipmi_iobm_resp_data {
	unsigned char   eeprom[EEPROM_SIZE];
	unsigned char   dom[EEPROM_SIZE];
};

struct extreme8730_32d_iobm_data {
	struct platform_device *pdev;
	struct mutex	 update_lock;
	char			 valid[3]; /* 0: SFP1, 1: SFP2, 2:QSFP28 */
	char			 valid_status[2]; /* != 0 if registers are valid */
							    	  /* 0: SFP1, 1: SFP2, 2:QSFP28 */
	unsigned long	 last_updated[3];	 /* In jiffies, 0: SFP1, 1: SFP2, 2:QSFP28 */
	unsigned long	 last_updated_status[2]; /* In jiffies, 0: SFP1, SFP2, 1:QSFP28 */
	struct ipmi_data ipmi;
	struct ipmi_iobm_resp_data ipmi_resp[3]; /* 0: SFP1, 1: SFP2, 2:QSFP28 */
	unsigned char ipmi_sfp_status;
	unsigned char ipmi_qsfp_status;
	unsigned char ipmi_tx_data[2];
};

struct extreme8730_32d_iobm_data *data = NULL;

static struct platform_driver extreme8730_32d_iobm_driver = {
	.probe		= extreme8730_32d_iobm_probe,
	.remove 	= extreme8730_32d_iobm_remove,
	.driver 	= {
		.name	= DRVNAME,
		.owner	= THIS_MODULE,
	},
};

#define SFP_PRESENT_ATTR_ID(index)			SFP##index##_PRESENT
#define SFP_TXFAULT_ATTR_ID(index)			SFP##index##_TXFAULT
#define SFP_RXLOS_ATTR_ID(index)			SFP##index##_RXLOS
#define SFP_TXDIS_ATTR_ID(index)			SFP##index##_TXDIS
#define SFP_EEPROM_ATTR_ID(index)			SFP##index##_EEPROM
#define SFP_DOM_ATTR_ID(index)				SFP##index##_DOM
#define QSFP28_MOD_SEL_ATTR_ID(index)		QSFP##index##_MOD_SEL
#define QSFP28_LP_MODE_ATTR_ID(index)		QSFP##index##_LP_MODE
#define QSFP28_RST_MOD_ATTR_ID(index)		QSFP##index##_RST_MOD
#define QSFP28_MOD_PRESENT_ATTR_ID(index)	QSFP##index##_MOD_PRESENT
#define QSFP28_EEPROM_ATTR_ID(index)		QSFP##index##_EEPROM
#define QSFP28_DOM_ATTR_ID(index)			QSFP##index##_DOM

#define SFP_ATTR(iobm_id) \
		SFP_PRESENT_ATTR_ID(iobm_id),		\
		SFP_TXFAULT_ATTR_ID(iobm_id),		\
		SFP_RXLOS_ATTR_ID(iobm_id),		\
		SFP_TXDIS_ATTR_ID(iobm_id),		\
		SFP_EEPROM_ATTR_ID(iobm_id),		\
		SFP_DOM_ATTR_ID(iobm_id)

#define QSFP28_ATTR(iobm_id) \
		QSFP28_MOD_SEL_ATTR_ID(iobm_id),		\
		QSFP28_LP_MODE_ATTR_ID(iobm_id),		\
		QSFP28_RST_MOD_ATTR_ID(iobm_id),		\
		QSFP28_MOD_PRESENT_ATTR_ID(iobm_id),		\
		QSFP28_EEPROM_ATTR_ID(iobm_id),		\
		QSFP28_DOM_ATTR_ID(iobm_id)
	
enum extreme8730_32d_iobm_sysfs_attrs {
	/* psu attributes */
	SFP_ATTR(1),
	SFP_ATTR(2),
	QSFP28_ATTR(1),
	NUM_OF_IOBM_ATTR,
	NUM_OF_PER_IOBM_ATTR = (NUM_OF_IOBM_ATTR/NUM_OF_IOBM)
};

/* iobm attributes */
#define DECLARE_SFP_SENSOR_DEVICE_ATTR(index) \
		static SENSOR_DEVICE_ATTR(sfp##index##_present, S_IRUGO, show_io,	 NULL, SFP##index##_PRESENT); \
		static SENSOR_DEVICE_ATTR(sfp##index##_txfault, S_IRUGO, show_io,  NULL, SFP##index##_TXFAULT); \
		static SENSOR_DEVICE_ATTR(sfp##index##_rxlos, S_IRUGO, show_io,	 NULL, SFP##index##_RXLOS); \
		static SENSOR_DEVICE_ATTR(sfp##index##_txdis, S_IWUSR | S_IRUGO, show_io,  set_io, SFP##index##_TXDIS); \	
		static SENSOR_DEVICE_ATTR(sfp##index##_eeprom, S_IRUGO, show_eeprom,	 NULL, SFP##index##_EEPROM); \
		static SENSOR_DEVICE_ATTR(sfp##index##_dom, S_IRUGO, show_dom,	 NULL, SFP##index##_DOM)
#define DECLARE_QSFP28_SENSOR_DEVICE_ATTR(index) \
		static SENSOR_DEVICE_ATTR(qsfp##index##_mod_sel, S_IRUGO, show_io,  NULL, QSFP##index##_MOD_SEL); \
		static SENSOR_DEVICE_ATTR(qsfp##index##_lp_mode, S_IWUSR | S_IRUGO, show_io,  set_io, QSFP##index##_LP_MODE); \
		static SENSOR_DEVICE_ATTR(qsfp##index##_rst_mod, S_IWUSR | S_IRUGO, show_io,	 set_io, QSFP##index##_RST_MOD); \
		static SENSOR_DEVICE_ATTR(qsfp##index##_mod_present, S_IRUGO, show_io,  NULL, QSFP##index##_MOD_PRESENT); \	
		static SENSOR_DEVICE_ATTR(qsfp##index##_eeprom, S_IRUGO, show_eeprom,	 NULL, QSFP##index##_EEPROM); \
		static SENSOR_DEVICE_ATTR(qsfp##index##_dom, S_IRUGO, show_dom,	 NULL, QSFP##index##_DOM)

#define DECLARE_SFP_ATTR(index) \
		&sensor_dev_attr_sfp##index##_present.dev_attr.attr, \
		&sensor_dev_attr_sfp##index##_txfault.dev_attr.attr, \
		&sensor_dev_attr_sfp##index##_rxlos.dev_attr.attr, \
		&sensor_dev_attr_sfp##index##_txdis.dev_attr.attr, \
		&sensor_dev_attr_sfp##index##_eeprom.dev_attr.attr, \
		&sensor_dev_attr_sfp##index##_dom.dev_attr.attr
#define DECLARE_QSFP28_ATTR(index) \
		&sensor_dev_attr_qsfp##index##_mod_sel.dev_attr.attr, \
		&sensor_dev_attr_qsfp##index##_lp_mode.dev_attr.attr, \
		&sensor_dev_attr_qsfp##index##_rst_mod.dev_attr.attr, \
		&sensor_dev_attr_qsfp##index##_mod_present.dev_attr.attr, \
		&sensor_dev_attr_qsfp##index##_eeprom.dev_attr.attr, \
		&sensor_dev_attr_qsfp##index##_dom.dev_attr.attr
		
DECLARE_SFP_SENSOR_DEVICE_ATTR(1);
DECLARE_SFP_SENSOR_DEVICE_ATTR(2);
DECLARE_QSFP28_SENSOR_DEVICE_ATTR(1);

static struct attribute *extreme8730_32d_iobm_attributes[] = {
	/* iobm attributes */
	DECLARE_SFP_ATTR(1),
	DECLARE_SFP_ATTR(2),
	DECLARE_QSFP28_ATTR(1),
	NULL
};

static const struct attribute_group extreme8730_32d_iobm_group = {
	.attrs = extreme8730_32d_iobm_attributes,
};

/* Functions to talk to the IPMI layer */

/* Initialize IPMI address, message buffers and user data */
static int init_ipmi_data(struct ipmi_data *ipmi, int iface,
				  struct device *dev)
{
	int err;

	init_completion(&ipmi->read_complete);

	/* Initialize IPMI address */
	ipmi->address.addr_type = IPMI_SYSTEM_INTERFACE_ADDR_TYPE;
	ipmi->address.channel = IPMI_BMC_CHANNEL;
	ipmi->address.data[0] = 0;
	ipmi->interface = iface;

	/* Initialize message buffers */
	ipmi->tx_msgid = 0;
	ipmi->tx_message.netfn = IPMI_SENSOR_NETFN;

	ipmi->ipmi_hndlrs.ipmi_recv_hndl = ipmi_msg_handler;

	/* Create IPMI messaging interface user */
	err = ipmi_create_user(ipmi->interface, &ipmi->ipmi_hndlrs,
				   ipmi, &ipmi->user);
	if (err < 0) {
		dev_err(dev, "Unable to register user with IPMI "
			"interface %d\n", ipmi->interface);
		return -EACCES;
	}

	return 0;
}

/* Send an IPMI command */
static int ipmi_send_message(struct ipmi_data *ipmi, unsigned char cmd,
									 unsigned char *tx_data, unsigned short tx_len,
									 unsigned char *rx_data, unsigned short rx_len)
{
	int err;

	ipmi->tx_message.cmd	  = cmd;
	ipmi->tx_message.data	  = tx_data;
	ipmi->tx_message.data_len = tx_len;
	ipmi->rx_msg_data		  = rx_data;
	ipmi->rx_msg_len		  = rx_len;

	err = ipmi_validate_addr(&ipmi->address, sizeof(ipmi->address));
	if (err)
		goto addr_err;

	ipmi->tx_msgid++;
	err = ipmi_request_settime(ipmi->user, &ipmi->address, ipmi->tx_msgid,
				   &ipmi->tx_message, ipmi, 0, 0, 0);
	if (err)
		goto ipmi_req_err;

	err = wait_for_completion_timeout(&ipmi->read_complete, IPMI_TIMEOUT);
	if (!err)
		goto ipmi_timeout_err;

	return 0;

ipmi_timeout_err:
	err = -ETIMEDOUT;
	dev_err(&data->pdev->dev, "request_timeout=%x\n", err);
	return err;
ipmi_req_err:
	dev_err(&data->pdev->dev, "request_settime=%x\n", err);
	return err;
addr_err:
	dev_err(&data->pdev->dev, "validate_addr=%x\n", err);
	return err;
}

/* Dispatch IPMI messages to callers */
static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data)
{
	unsigned short rx_len;
	struct ipmi_data *ipmi = user_msg_data;

	if (msg->msgid != ipmi->tx_msgid) {
		dev_err(&data->pdev->dev, "Mismatch between received msgid "
			"(%02x) and transmitted msgid (%02x)!\n",
			(int)msg->msgid,
			(int)ipmi->tx_msgid);
		ipmi_free_recv_msg(msg);
		return;
	}

	ipmi->rx_recv_type = msg->recv_type;
	if (msg->msg.data_len > 0)
		ipmi->rx_result = msg->msg.data[0];
	else
		ipmi->rx_result = IPMI_UNKNOWN_ERR_COMPLETION_CODE;

	if (msg->msg.data_len > 1) {
		rx_len = msg->msg.data_len - 1;
		if (ipmi->rx_msg_len < rx_len)
			rx_len = ipmi->rx_msg_len;
		ipmi->rx_msg_len = rx_len;
		memcpy(ipmi->rx_msg_data, msg->msg.data + 1, ipmi->rx_msg_len);
	} else
		ipmi->rx_msg_len = 0;

	ipmi_free_recv_msg(msg);
	complete(&ipmi->read_complete);
}
static struct extreme8730_32d_iobm_data *extreme8730_32d_iobm_update_device(struct device_attribute *da)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char iobmid = attr->index / NUM_OF_PER_IOBM_ATTR;
	unsigned char iobm_status_cmd;
	unsigned char *iobm_status_resp;
	int status = 0, group=0;

	switch (iobmid) 
    {
		case SFP_1:
		case SFP_2:
			group = 0; 
			break;
		case QSFP28_1:
			group = 1;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	if (time_before(jiffies, data->last_updated_status[group] + HZ) && data->valid_status[group]) {
		return data;
	}

	data->valid_status[group] = 0;


	switch (iobmid) 
    {
		case SFP_1:
		case SFP_2:
			iobm_status_cmd = IPMI_SFP_IO_READ_CMD;
			iobm_status_resp = &data->ipmi_sfp_status;
			break;
		case QSFP28_1:
			iobm_status_cmd = IPMI_QSFP28_IO_READ_CMD;
			iobm_status_resp = &data->ipmi_qsfp_status;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	/* Get status from ipmi */
	status = ipmi_send_message(&data->ipmi, iobm_status_cmd,
				   data->ipmi_tx_data, 0,
				   iobm_status_resp,
				   sizeof(*iobm_status_resp));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated_status[group] = jiffies;
	data->valid_status[group] = 1;

exit:
	return data;
}

static struct extreme8730_32d_iobm_data *extreme8730_32d_iobm_update_eeprom(struct device_attribute *da)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char iobmid = attr->index / NUM_OF_PER_IOBM_ATTR;
	int status = 0;

	if (time_before(jiffies, data->last_updated[iobmid] + HZ * 5) && data->valid[iobmid]) {
		return data;
	}

	data->valid[iobmid] = 0;


	switch (iobmid) 
    {
		case SFP_1:
			data->ipmi_tx_data[0] = IPMI_IOBM_OFFSET_SFP1;
			break;
		case SFP_2:
			data->ipmi_tx_data[0] = IPMI_IOBM_OFFSET_SFP2;
			break;
		case QSFP28_1:
			data->ipmi_tx_data[0] = IPMI_IOBM_OFFSET_QSFP1;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	/* Get eeprom from ipmi */
	data->ipmi_tx_data[1] = IPMI_PAGE_A0_READ_CMD;
	status = ipmi_send_message(&data->ipmi, IPMI_EEPROM_READ_CMD,
				   data->ipmi_tx_data, 2,
				   data->ipmi_resp[iobmid].eeprom,
				   sizeof(data->ipmi_resp[iobmid].eeprom));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated[iobmid] = jiffies;
	data->valid[iobmid] = 1;

exit:
	return data;
}

static struct extreme8730_32d_iobm_data *extreme8730_32d_iobm_update_dom(struct device_attribute *da)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char iobmid = attr->index / NUM_OF_PER_IOBM_ATTR;
	int status = 0;

	if (time_before(jiffies, data->last_updated[iobmid] + HZ * 5) && data->valid[iobmid]) {
		return data;
	}

	data->valid[iobmid] = 0;


	switch (iobmid) 
    {
		case SFP_1:
			data->ipmi_tx_data[0] = IPMI_IOBM_OFFSET_SFP1;
			break;
		case SFP_2:
			data->ipmi_tx_data[0] = IPMI_IOBM_OFFSET_SFP2;
			break;			
		case QSFP28_1:
			data->ipmi_tx_data[0] = IPMI_IOBM_OFFSET_QSFP1;
			break;			
		default:
			status = -EIO;
			goto exit;
	}

	/* Get dom from ipmi */
	data->ipmi_tx_data[1] = IPMI_PAGE_A2_READ_CMD;
	status = ipmi_send_message(&data->ipmi, IPMI_EEPROM_READ_CMD,
				   data->ipmi_tx_data, 2,
				   data->ipmi_resp[iobmid].dom,
				   sizeof(data->ipmi_resp[iobmid].dom));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated[iobmid] = jiffies;
	data->valid[iobmid] = 1;

exit:
	return data;
}

static ssize_t show_io(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char iobmid = attr->index / NUM_OF_PER_IOBM_ATTR;
	int value = 0, group = 0;
	int error = 0;
	int present = 0;
	u8 mask = 0;

	switch (iobmid) 
    {
		case SFP_1:
		case SFP_2:
			group = 0; 
			break;
		case QSFP28_1:
			group = 1;
			break;
		default:
			error = -EIO;
			goto exit;
	}

	mutex_lock(&data->update_lock);

	data = extreme8730_32d_iobm_update_device(da);	
	if (!data->valid_status[group]) {
		error = -EIO;
		goto exit;
	}

	switch (attr->index) {
		case SFP1_PRESENT:
		case SFP2_PRESENT:
            mask = 0x1 << ((NUM_OF_SFP_IO * iobmid) + SFP_PRESENT);
			value = !(!!(data->ipmi_sfp_status & mask));	/* BMC 0 = Present, 1 = Not Present. need to convert */
			break;
		case SFP1_TXFAULT:
		case SFP2_TXFAULT:
			mask = 0x1 << ((NUM_OF_SFP_IO * iobmid) + SFP_TXFAULT);
			value = !!(data->ipmi_sfp_status & mask);	/* BMC 0 = De-asserted, 1 = Asserted. */
			break;
		case SFP1_RXLOS:
		case SFP2_RXLOS:
			mask = 0x1 << ((NUM_OF_SFP_IO * iobmid) + SFP_RXLOS);
			value = !!(data->ipmi_sfp_status & mask);	/* BMC 0 = De-asserted, 1 = Asserted. */
			break;
		case SFP1_TXDIS:
		case SFP2_TXDIS:
			mask = 0x1 << ((NUM_OF_SFP_IO * iobmid) + SFP_TXDIS);
			value = !!(data->ipmi_sfp_status & mask);	/* BMC 0 = Disabled, 1 = Enabled. */
			break;
		case QSFP1_MOD_SEL:
			mask = 0x1 << QSFP_MOD_SEL;
			value = !(!!(data->ipmi_qsfp_status & mask));	/* BMC 0 = Enabled, 1 = Disabled. need to convert */
			break;
		case QSFP1_LP_MODE:
			mask = 0x1 << QSFP_LP_MODE;
			value = !!(data->ipmi_qsfp_status & mask);	/* BMC 0 = High Power Mode, 1 = Low Power Mode. */
			break;
		case QSFP1_RST_MOD:
			mask = 0x1 << QSFP_RST_MOD;
			value = !(!!(data->ipmi_qsfp_status & mask));	/* BMC 0 = Reset Mode, 1 = Not Reset Mode. need to convert */
			break;
		case QSFP1_MOD_PRESENT:
			mask = 0x1 << QSFP_MOD_PRESENT;
			value = !(!!(data->ipmi_qsfp_status & mask));	/* BMC 0 = Present, 1 = Not Present. need to convert */
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	DEBUG_PRINT("\n 8730_32d_iobm_io_eeprom show_io: iobmid:%d, attr:%d, value:%d \n", iobmid, attr->index, value);
	
	mutex_unlock(&data->update_lock);
	return sprintf(buf, "%d\n", value);

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static ssize_t set_io(struct device *dev, struct device_attribute *da,
		       const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char iobmid = attr->index / NUM_OF_PER_IOBM_ATTR;
	unsigned char iobm_setio_cmd;
	long on;
	int error;
	u8 mask = 0;
	int value = 0, group = 0;
	
	switch (iobmid) 
    {
		case SFP_1:
		case SFP_2:
			group = 0; 
			break;
		case QSFP28_1:
			group = 1;
			break;
		default:
			error = -EIO;
			goto exit;
	}
	
	error = kstrtol(buf, 10, &on);
	if (error)
		return error;

	if ((on != 1) && (on != 0))
        return -EINVAL;

	mutex_lock(&data->update_lock);

	data = extreme8730_32d_iobm_update_device(da);
	if (!data->valid_status[group]) {
		error = -EIO;
		goto exit;
	}

	switch (attr->index) {
		case SFP1_TXDIS:
		case SFP2_TXDIS:
			iobm_setio_cmd = IPMI_SFP_IO_WRITE_CMD;
			mask = 0x1 << ((NUM_OF_SFP_IO * iobmid) + SFP_TXDIS);
			value = data->ipmi_sfp_status;
			break;
		case QSFP1_LP_MODE:
			iobm_setio_cmd = IPMI_QSFP28_IO_WRITE_CMD;
			mask = 0x1 << QSFP_LP_MODE;
			value = data->ipmi_qsfp_status;
			break;
		case QSFP1_RST_MOD:
			iobm_setio_cmd = IPMI_QSFP28_IO_WRITE_CMD;
			mask = 0x1 << QSFP_RST_MOD;
			value = data->ipmi_qsfp_status;
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	
	if (on) {
		if(attr->index == QSFP1_RST_MOD) {		
			value &= ~mask;
		}
		else {		
			value |= mask;
		}
	}
	else {
		if(attr->index == QSFP1_RST_MOD) {		
			value |= mask;
		}
		else {		
			value &= ~mask;
		}
	}

	DEBUG_PRINT("\n set_io: attr->index:%d, mask:0x%x, value:0x%x", attr->index, mask, value);
	
	/* Send IPMI write command */	
	data->ipmi_tx_data[0] = value;
	
	error = ipmi_send_message(&data->ipmi, iobm_setio_cmd,
				   data->ipmi_tx_data, 1,
				   NULL, 0);

	if (unlikely(error != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		error = -EIO;
		goto exit;
	}

	error = count;

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static ssize_t show_eeprom(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char iobmid = attr->index / NUM_OF_PER_IOBM_ATTR;
	int error = 0;
	int count = 0;

	mutex_lock(&data->update_lock);

	data = extreme8730_32d_iobm_update_eeprom(da);
	if (!data->valid[iobmid]) {
		error = -EIO;
		goto exit;
	}
	
	switch (attr->index) {
		case SFP1_EEPROM:
		case SFP2_EEPROM:
		case QSFP1_EEPROM:
			memcpy(buf, data->ipmi_resp[iobmid].eeprom, sizeof(data->ipmi_resp[iobmid].eeprom));
			count = sizeof(data->ipmi_resp[iobmid].eeprom);
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	mutex_unlock(&data->update_lock);

	DEBUG_PRINT("\n 8730_32d_iobm_io_eeprom show_eeprom: iobmid:%d, attr:%d, count:%d \n", iobmid, attr->index, count);

	return count;

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static ssize_t show_dom(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char iobmid = attr->index / NUM_OF_PER_IOBM_ATTR;
	int error = 0;
	int count = 0;

	mutex_lock(&data->update_lock);

	data = extreme8730_32d_iobm_update_dom(da);
	if (!data->valid[iobmid]) {
		error = -EIO;
		goto exit;
	}
	
	switch (attr->index) {
		case SFP1_DOM:
		case SFP2_DOM:
		case QSFP1_DOM:
			memcpy(buf, data->ipmi_resp[iobmid].dom, sizeof(data->ipmi_resp[iobmid].dom));
			count = sizeof(data->ipmi_resp[iobmid].dom);
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	mutex_unlock(&data->update_lock);
	
	DEBUG_PRINT("\n 8730_32d_iobm_io_eeprom show_eeprom: iobmid:%d, attr:%d, count:%d \n", iobmid, attr->index, count);

	return count;

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static int extreme8730_32d_iobm_probe(struct platform_device *pdev)
{
	int status = -1;

	/* Register sysfs hooks */	
	status = sysfs_create_group(&pdev->dev.kobj, &extreme8730_32d_iobm_group);
	if (status) {
		goto exit;
	}


	dev_info(&pdev->dev, "device created\n");

	return 0;

exit:
	return status;
}

static int extreme8730_32d_iobm_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &extreme8730_32d_iobm_group);
	return 0;
}

static int __init extreme8730_32d_iobm_init(void)
{
	int ret;

	data = kzalloc(sizeof(struct extreme8730_32d_iobm_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	mutex_init(&data->update_lock);

	ret = platform_driver_register(&extreme8730_32d_iobm_driver);
	if (ret < 0) {
		goto dri_reg_err;
	}

	data->pdev = platform_device_register_simple(DRVNAME, -1, NULL, 0);
	if (IS_ERR(data->pdev)) {
		ret = PTR_ERR(data->pdev);
		goto dev_reg_err;
	}

	/* Set up IPMI interface */
	ret = init_ipmi_data(&data->ipmi, 0, &data->pdev->dev);
	if (ret)
		goto ipmi_err;
	

	return 0;

ipmi_err:
	platform_device_unregister(data->pdev);
dev_reg_err:
	platform_driver_unregister(&extreme8730_32d_iobm_driver);
dri_reg_err:
	kfree(data);
alloc_err:
	return ret;
}

static void __exit extreme8730_32d_iobm_exit(void)
{
	ipmi_destroy_user(data->ipmi.user);
	platform_device_unregister(data->pdev);
	platform_driver_unregister(&extreme8730_32d_iobm_driver);
	kfree(data);
}

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extremenetworks 8730-32d IOBM IO and EEPROM driver");
MODULE_LICENSE("GPL");

module_init(extreme8730_32d_iobm_init);
module_exit(extreme8730_32d_iobm_exit);