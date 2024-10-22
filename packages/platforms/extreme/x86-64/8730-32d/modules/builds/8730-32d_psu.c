/*
 * An hwmon driver for extremenetworks 8730-32d AcBel Power Module
 *
 * Copyright (C) 2024 Alphanetworks Technology Corporation.
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

#define DRVNAME "8730_psu"

#define IPMI_SENSOR_NETFN           	0x34
#define IPMI_SENSOR_READ_CMD 			0x11
#define IPMI_PSU_THERMAL_READ_CMD 		0x12
#define IPMI_PSU_FAN_READ_CMD 			0x13
#define IPMI_PSU_FRU_READ_CMD 			0x14
#define IPMI_PSU_STATUS_READ_CMD 		0x15
#define IPMI_PSU_FANDIR_READ_CMD 		0x16

/* For read PSU AC_OK */
#define IPMI_APP_NETFN					0x06
#define IPMI_READ_WRITE_CMD        		0x52
#define IPMI_PWRCPLD_BUS				0x09
#define IPMI_PWRCPLD_ADDRESS			0xbe	/* 0x5f << 1 = 0xbe */
#define IPMI_READ_BYTE					0x01
#define IPMI_SENSOR_OFFSET_PSU1_STAT    0x11
#define IPMI_SENSOR_OFFSET_PSU2_STAT    0x12

#define IPMI_TIMEOUT					(20 * HZ)
#define IPMI_ERR_RETRY_TIMES			1

#define IPMI_SENSOR_OFFSET_PSU1			0x01
#define IPMI_SENSOR_OFFSET_PSU2			0x02

static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");

#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)

static void ipmi_msg_handler(struct ipmi_recv_msg *msg, void *user_msg_data);
static ssize_t show_psu(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_string(struct device *dev, struct device_attribute *attr, char *buf);
static int extreme8730_32d_psu_probe(struct platform_device *pdev);
static int extreme8730_32d_psu_remove(struct platform_device *pdev);

enum psu_id {
	PSU_1,
	PSU_2,
	NUM_OF_PSU
};

enum psu_status_byte_index {
	PSU_PRESENT = 0,
	PSU_POWER_GOOD_CPLD,
	PSU_AC_OK
};

enum psu_power_value_byte_index {
    VIN_BYTE0,
	VIN_BYTE1,
	VIN_BYTE2,
	VIN_BYTE3,
    VOUT_BYTE0,
    VOUT_BYTE1,
    VOUT_BYTE2,
    VOUT_BYTE3,
    IIN_BYTE0,
    IIN_BYTE1,
    IIN_BYTE2,
    IIN_BYTE3,
    IOUT_BYTE0,
    IOUT_BYTE1,
    IOUT_BYTE2,
    IOUT_BYTE3,
    PIN_BYTE0,
    PIN_BYTE1,
    PIN_BYTE2,
    PIN_BYTE3,
    POUT_BYTE0,
    POUT_BYTE1,
    POUT_BYTE2,
    POUT_BYTE3
};

enum psu_temp_byte_index {
    TEMP0_BYTE0,
	TEMP0_BYTE1,
	TEMP0_BYTE2,
	TEMP0_BYTE3,
    TEMP1_BYTE0,
    TEMP1_BYTE1,
    TEMP1_BYTE2,
    TEMP1_BYTE3,
    TEMP2_BYTE0,
    TEMP2_BYTE1,
    TEMP2_BYTE2,
    TEMP2_BYTE3
};

enum psu_fan_byte_index {
    FAN0_BYTE0,
	FAN0_BYTE1,
	FAN0_BYTE2,
	FAN0_BYTE3,
    FAN1_BYTE0,
    FAN1_BYTE1,
    FAN1_BYTE2,
    FAN1_BYTE3
};

enum psu_fru_len_index {
    VENDOR_LEN,
	MODEL_LEN,
	SERIAL_LEN,
	NUM_OF_LEN
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

struct ipmi_psu_resp_data {
	char   fru[50];
	unsigned char   status[2];	/* 0: Present(1) Not Present(0), 1: Power Good(1) No Power Good(0) */
	unsigned char   status_acok_raw;	/* bit0: Present, bit1: Power Good, bit2: Input Power good(1) Input Power no good(0), bit3: ALERT, bit4-7: Reserved */
	unsigned char   power_value[24];	/* 0-3: VIN, 4-7: VOUT, 8-11: IIN, 12-15: IOUT, 16-19: PIN, 20-23: POUT */
	unsigned char   temp_input[12];	/* 0-3: Temp0, 4-7: Temp1, 8-11: Temp2 */
	unsigned char   fan_input[8];		/* 0-3: FAN0, 4-7: FAN1 */
};

struct extreme8730_32d_psu_data {
	struct platform_device *pdev;
	struct mutex	 update_lock;
	char			 valid[2]; /* 0: PSU1, 1: PSU2 */
	char			 valid_status[2]; /* != 0 if registers are valid */
							    	  /* 0: PSU1, 1: PSU2 */
	unsigned long	 last_updated[2];	 /* In jiffies, 0: PSU1, 1: PSU2 */
	unsigned long	 last_updated_status[2]; /* In jiffies, 0: PSU1, 1: PSU2 */
	struct ipmi_data ipmi;
	struct ipmi_psu_resp_data ipmi_resp[2]; /* 0: PSU1, 1: PSU2 */
	unsigned char ipmi_tx_data[4];
};

struct extreme8730_32d_psu_data *data = NULL;

static struct platform_driver extreme8730_32d_psu_driver = {
	.probe		= extreme8730_32d_psu_probe,
	.remove 	= extreme8730_32d_psu_remove,
	.driver 	= {
		.name	= DRVNAME,
		.owner	= THIS_MODULE,
	},
};

#define PSU_PRESENT_ATTR_ID(index)      PSU##index##_PRESENT
#define PSU_POWERGOOD_ATTR_ID(index)    PSU##index##_POWER_GOOD
#define PSU_ACOK_ATTR_ID(index)    		PSU##index##_AC_OK
#define PSU_VIN_ATTR_ID(index)          PSU##index##_VIN
#define PSU_VOUT_ATTR_ID(index)         PSU##index##_VOUT
#define PSU_IIN_ATTR_ID(index)         	PSU##index##_IIN
#define PSU_IOUT_ATTR_ID(index)         PSU##index##_IOUT
#define PSU_PIN_ATTR_ID(index)          PSU##index##_PIN
#define PSU_POUT_ATTR_ID(index)         PSU##index##_POUT
#define PSU_MODEL_ATTR_ID(index)        PSU##index##_MODEL
#define PSU_SERIAL_ATTR_ID(index)       PSU##index##_SERIAL
#define PSU_TEMP1_INPUT_ATTR_ID(index)  PSU##index##_TEMP1_INPUT
#define PSU_TEMP2_INPUT_ATTR_ID(index)  PSU##index##_TEMP2_INPUT
#define PSU_TEMP3_INPUT_ATTR_ID(index)  PSU##index##_TEMP3_INPUT
#define PSU_FAN1_INPUT_ATTR_ID(index)   PSU##index##_FAN1_INPUT
#define PSU_FAN2_INPUT_ATTR_ID(index)   PSU##index##_FAN2_INPUT

#define PSU_ATTR(psu_id) \
		PSU_PRESENT_ATTR_ID(psu_id),		\
		PSU_POWERGOOD_ATTR_ID(psu_id),		\
		PSU_ACOK_ATTR_ID(psu_id),		\
		PSU_VIN_ATTR_ID(psu_id),		\
		PSU_VOUT_ATTR_ID(psu_id),		\
		PSU_IIN_ATTR_ID(psu_id),		\
		PSU_IOUT_ATTR_ID(psu_id),		\
		PSU_PIN_ATTR_ID(psu_id),		\
		PSU_POUT_ATTR_ID(psu_id),		\
		PSU_MODEL_ATTR_ID(psu_id),		\
		PSU_SERIAL_ATTR_ID(psu_id), 	\
		PSU_TEMP1_INPUT_ATTR_ID(psu_id), \
		PSU_TEMP2_INPUT_ATTR_ID(psu_id), \
		PSU_TEMP3_INPUT_ATTR_ID(psu_id), \
		PSU_FAN1_INPUT_ATTR_ID(psu_id), \
		PSU_FAN2_INPUT_ATTR_ID(psu_id)
	
enum extreme8730_32d_psu_sysfs_attrs {
	/* psu attributes */
	PSU_ATTR(1),
	PSU_ATTR(2),
	NUM_OF_PSU_ATTR,
	NUM_OF_PER_PSU_ATTR = (NUM_OF_PSU_ATTR/NUM_OF_PSU)
};

/* psu attributes */
#define DECLARE_PSU_SENSOR_DEVICE_ATTR(index) \
		static SENSOR_DEVICE_ATTR(psu##index##_present, S_IRUGO, show_psu,	 NULL, PSU##index##_PRESENT); \
		static SENSOR_DEVICE_ATTR(psu##index##_power_good, S_IRUGO, show_psu,  NULL, PSU##index##_POWER_GOOD); \
		static SENSOR_DEVICE_ATTR(psu##index##_ac_ok, S_IRUGO, show_psu,  NULL, PSU##index##_AC_OK); \
		static SENSOR_DEVICE_ATTR(psu##index##_vin, S_IRUGO, show_psu,	 NULL, PSU##index##_VIN); \
		static SENSOR_DEVICE_ATTR(psu##index##_vout, S_IRUGO, show_psu,  NULL, PSU##index##_VOUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_iin, S_IRUGO, show_psu,  NULL, PSU##index##_IIN); \
		static SENSOR_DEVICE_ATTR(psu##index##_iout, S_IRUGO, show_psu,  NULL, PSU##index##_IOUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_pin, S_IRUGO, show_psu,  NULL, PSU##index##_PIN); \
		static SENSOR_DEVICE_ATTR(psu##index##_pout, S_IRUGO, show_psu,  NULL, PSU##index##_POUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_model, S_IRUGO, show_string,  NULL, PSU##index##_MODEL); \
		static SENSOR_DEVICE_ATTR(psu##index##_serial, S_IRUGO, show_string,  NULL, PSU##index##_SERIAL);\
		static SENSOR_DEVICE_ATTR(psu##index##_temp1_input, S_IRUGO, show_psu,	NULL, PSU##index##_TEMP1_INPUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_temp2_input, S_IRUGO, show_psu,	NULL, PSU##index##_TEMP2_INPUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_temp3_input, S_IRUGO, show_psu,	NULL, PSU##index##_TEMP3_INPUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_fan1_input, S_IRUGO, show_psu,	NULL, PSU##index##_FAN1_INPUT); \
		static SENSOR_DEVICE_ATTR(psu##index##_fan2_input, S_IRUGO, show_psu,  NULL, PSU##index##_FAN2_INPUT)	
#define DECLARE_PSU_ATTR(index) \
		&sensor_dev_attr_psu##index##_present.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_power_good.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_ac_ok.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_vin.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_vout.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_iin.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_iout.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_pin.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_pout.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_model.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_serial.dev_attr.attr,\
		&sensor_dev_attr_psu##index##_temp1_input.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_temp2_input.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_temp3_input.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_fan1_input.dev_attr.attr, \
		&sensor_dev_attr_psu##index##_fan2_input.dev_attr.attr
	
DECLARE_PSU_SENSOR_DEVICE_ATTR(1);
DECLARE_PSU_SENSOR_DEVICE_ATTR(2);

static struct attribute *extreme8730_32d_psu_attributes[] = {
	/* psu attributes */
	DECLARE_PSU_ATTR(1),
	DECLARE_PSU_ATTR(2),
	NULL
};

static const struct attribute_group extreme8730_32d_psu_group = {
	.attrs = extreme8730_32d_psu_attributes,
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
static struct extreme8730_32d_psu_data *extreme8730_32d_psu_update_device(struct device_attribute *da)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char pid = attr->index / NUM_OF_PER_PSU_ATTR;
	int status = 0;

	if (time_before(jiffies, data->last_updated_status[pid] + HZ * 5) && data->valid_status[pid]) {
		return data;
	}

	data->valid_status[pid] = 0;
	data->ipmi.tx_message.netfn = IPMI_SENSOR_NETFN;

	switch (pid) 
    {
		case PSU_1:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_PSU1;
			break;
		case PSU_2:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_PSU2;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	/* Get status from ipmi */
	status = ipmi_send_message(&data->ipmi, IPMI_PSU_STATUS_READ_CMD,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp[pid].status,
				   sizeof(data->ipmi_resp[pid].status));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		DEBUG_PRINT("%s:%d \n", __FUNCTION__, __LINE__);
		goto exit;
	}

	/* Get power value from ipmi */
	status = ipmi_send_message(&data->ipmi, IPMI_SENSOR_READ_CMD,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp[pid].power_value,
				   sizeof(data->ipmi_resp[pid].power_value));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	/* Get psu thermal from ipmi */
	status = ipmi_send_message(&data->ipmi, IPMI_PSU_THERMAL_READ_CMD,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp[pid].temp_input,
				   sizeof(data->ipmi_resp[pid].temp_input));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	/* Get psu fan from ipmi */
	status = ipmi_send_message(&data->ipmi, IPMI_PSU_FAN_READ_CMD,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp[pid].fan_input,
				   sizeof(data->ipmi_resp[pid].fan_input));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	/* Get AC_OK from ipmi */
	data->ipmi.tx_message.netfn = IPMI_APP_NETFN;
	data->ipmi_tx_data[0] = IPMI_PWRCPLD_BUS;
	data->ipmi_tx_data[1] = IPMI_PWRCPLD_ADDRESS;
	data->ipmi_tx_data[2] = IPMI_READ_BYTE;

	switch (pid) 
    {
		case PSU_1:
			data->ipmi_tx_data[3] = IPMI_SENSOR_OFFSET_PSU1_STAT;
			break;
		case PSU_2:
			data->ipmi_tx_data[3] = IPMI_SENSOR_OFFSET_PSU2_STAT;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	status = ipmi_send_message(&data->ipmi, IPMI_READ_WRITE_CMD,
				   data->ipmi_tx_data, 4,
				   &data->ipmi_resp[pid].status_acok_raw,
				   sizeof(data->ipmi_resp[pid].status_acok_raw));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated_status[pid] = jiffies;
	data->valid_status[pid] = 1;

exit:
	return data;
}

static struct extreme8730_32d_psu_data *extreme8730_32d_psu_update_string(struct device_attribute *da)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char pid = attr->index / NUM_OF_PER_PSU_ATTR;
	int status = 0;

	if (time_before(jiffies, data->last_updated[pid] + HZ * 5) && data->valid[pid]) {
		return data;
	}

	data->valid[pid] = 0;
	data->ipmi.tx_message.netfn = IPMI_SENSOR_NETFN;

	switch (pid) 
    {
		case PSU_1:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_PSU1;
			break;
		case PSU_2:
			data->ipmi_tx_data[0] = IPMI_SENSOR_OFFSET_PSU2;
			break;
		default:
			status = -EIO;
			goto exit;
	}

	/* Get fru from ipmi */
	status = ipmi_send_message(&data->ipmi, IPMI_PSU_FRU_READ_CMD,
				   data->ipmi_tx_data, 1,
				   data->ipmi_resp[pid].fru,
				   sizeof(data->ipmi_resp[pid].fru));

	if (unlikely(status != 0))
		goto exit;

	if (unlikely(data->ipmi.rx_result != 0)) {
		status = -EIO;
		goto exit;
	}

	data->last_updated[pid] = jiffies;
	data->valid[pid] = 1;

exit:
	return data;
}

#define VALIDATE_PRESENT_RETURN(id) \
do { \
	if (present == 0) { \
		mutex_unlock(&data->update_lock);   \
		return -ENXIO; \
	} \
} while (0)

static ssize_t show_psu(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char pid = attr->index / NUM_OF_PER_PSU_ATTR;
	int value = 0;
	int error = 0;
	int present = 0;

	mutex_lock(&data->update_lock);

	data = extreme8730_32d_psu_update_device(da);
	if (!data->valid_status[pid]) {
		error = -EIO;
		goto exit;
	}

	present = data->ipmi_resp[pid].status[PSU_PRESENT];

	switch (attr->index) {
		case PSU1_PRESENT:
		case PSU2_PRESENT:
            value = present;
			break;
		case PSU1_POWER_GOOD:
		case PSU2_POWER_GOOD:
            VALIDATE_PRESENT_RETURN(pid);
			value = data->ipmi_resp[pid].status[PSU_POWER_GOOD_CPLD];
			break;		
		case PSU1_AC_OK:
		case PSU2_AC_OK:
            VALIDATE_PRESENT_RETURN(pid);
			value = !!((int)data->ipmi_resp[pid].status_acok_raw & (1 << PSU_AC_OK));
			break;		
		case PSU1_VIN:
		case PSU2_VIN:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].power_value[VIN_BYTE0] | 
				(int)data->ipmi_resp[pid].power_value[VIN_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].power_value[VIN_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].power_value[VIN_BYTE3] << 32);
			break;
		case PSU1_VOUT:
		case PSU2_VOUT:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].power_value[VOUT_BYTE0] | 
				(int)data->ipmi_resp[pid].power_value[VOUT_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].power_value[VOUT_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].power_value[VOUT_BYTE3] << 32);
			break;
		case PSU1_IIN:
		case PSU2_IIN:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].power_value[IIN_BYTE0] | 
				(int)data->ipmi_resp[pid].power_value[IIN_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].power_value[IIN_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].power_value[IIN_BYTE3] << 32);
			break;
		case PSU1_IOUT:
		case PSU2_IOUT:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].power_value[IOUT_BYTE0] | 
				(int)data->ipmi_resp[pid].power_value[IOUT_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].power_value[IOUT_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].power_value[IOUT_BYTE3] << 32);
			break;
		case PSU1_PIN:
		case PSU2_PIN:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].power_value[PIN_BYTE0] | 
				(int)data->ipmi_resp[pid].power_value[PIN_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].power_value[PIN_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].power_value[PIN_BYTE3] << 32);
			break;
		case PSU1_POUT:
		case PSU2_POUT:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].power_value[POUT_BYTE0] | 
				(int)data->ipmi_resp[pid].power_value[POUT_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].power_value[POUT_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].power_value[POUT_BYTE3] << 32);
			break;
		case PSU1_TEMP1_INPUT:
		case PSU2_TEMP1_INPUT:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].temp_input[TEMP0_BYTE0] | 
				(int)data->ipmi_resp[pid].temp_input[TEMP0_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].temp_input[TEMP0_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].temp_input[TEMP0_BYTE3] << 32);
			break;
		case PSU1_TEMP2_INPUT:
		case PSU2_TEMP2_INPUT:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].temp_input[TEMP1_BYTE0] | 
				(int)data->ipmi_resp[pid].temp_input[TEMP1_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].temp_input[TEMP1_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].temp_input[TEMP1_BYTE3] << 32);
			break;
		case PSU1_TEMP3_INPUT:
		case PSU2_TEMP3_INPUT:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].temp_input[TEMP2_BYTE0] | 
				(int)data->ipmi_resp[pid].temp_input[TEMP2_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].temp_input[TEMP2_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].temp_input[TEMP2_BYTE3] << 32);
			break;
		case PSU1_FAN1_INPUT:
		case PSU2_FAN1_INPUT:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].fan_input[FAN0_BYTE0] | 
				(int)data->ipmi_resp[pid].fan_input[FAN0_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].fan_input[FAN0_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].fan_input[FAN0_BYTE3] << 32);
			break;
		case PSU1_FAN2_INPUT:
		case PSU2_FAN2_INPUT:
			VALIDATE_PRESENT_RETURN(pid);
			value = ((int)data->ipmi_resp[pid].fan_input[FAN1_BYTE0] | 
				(int)data->ipmi_resp[pid].fan_input[FAN1_BYTE1] << 8 |
				(int)data->ipmi_resp[pid].fan_input[FAN1_BYTE2] << 16 |
				(int)data->ipmi_resp[pid].fan_input[FAN1_BYTE3] << 32);
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	DEBUG_PRINT("8730_32d_psu show_psu: pid:%d, attr:%d, value:%d \n", pid, attr->index, value);
	
	mutex_unlock(&data->update_lock);
	return sprintf(buf, "%d\n", value);

exit:
	mutex_unlock(&data->update_lock);
	return error;
}

static ssize_t show_string(struct device *dev, struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	unsigned char pid = attr->index / NUM_OF_PER_PSU_ATTR;
	char tmp_str[50];
	char *str = NULL;
	int error = 0;
	int vendor_length = 0;
	int model_length = 0;
	int serial_length = 0;
	int i = 0;

	mutex_lock(&data->update_lock);

	data = extreme8730_32d_psu_update_string(da);
	if (!data->valid[pid]) {
		error = -EIO;
		goto exit;
	}

	vendor_length = data->ipmi_resp[pid].fru[VENDOR_LEN];
	model_length = data->ipmi_resp[pid].fru[MODEL_LEN];
	serial_length = data->ipmi_resp[pid].fru[SERIAL_LEN];

	switch (attr->index) {
		case PSU1_MODEL:
		case PSU2_MODEL:
			strcpy(tmp_str, data->ipmi_resp[pid].fru);
			str = tmp_str;
			memmove(str, &tmp_str[NUM_OF_LEN + vendor_length], model_length);
			str[model_length] = '\0';
			break;
		case PSU1_SERIAL:
		case PSU2_SERIAL:
			strcpy(tmp_str, data->ipmi_resp[pid].fru);
			str = tmp_str;
			memmove(str, &tmp_str[NUM_OF_LEN + vendor_length + model_length], serial_length);
			str[serial_length] = '\0';
			break;
		default:
			error = -EINVAL;
			goto exit;
	}

	DEBUG_PRINT("8730_32d_psu show_string: pid:%d, attr:%d, string:%s \n", pid, attr->index, str);

	mutex_unlock(&data->update_lock);
	return sprintf(buf, "%s\n", str);

exit:
	mutex_unlock(&data->update_lock);
	return error;
}
	
static int extreme8730_32d_psu_probe(struct platform_device *pdev)
{
	int status = -1;

	/* Register sysfs hooks */
	status = sysfs_create_group(&pdev->dev.kobj, &extreme8730_32d_psu_group);
	if (status) {
		goto exit;
	}


	dev_info(&pdev->dev, "device created\n");

	return 0;

exit:
	return status;
}

static int extreme8730_32d_psu_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &extreme8730_32d_psu_group);
	return 0;
}

static int __init extreme8730_32d_psu_init(void)
{
	int ret;

	data = kzalloc(sizeof(struct extreme8730_32d_psu_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	mutex_init(&data->update_lock);

	ret = platform_driver_register(&extreme8730_32d_psu_driver);
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
	platform_driver_unregister(&extreme8730_32d_psu_driver);
dri_reg_err:
	kfree(data);
alloc_err:
	return ret;
}

static void __exit extreme8730_32d_psu_exit(void)
{
	ipmi_destroy_user(data->ipmi.user);
	platform_device_unregister(data->pdev);
	platform_driver_unregister(&extreme8730_32d_psu_driver);
	kfree(data);
}

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extremenetworks 8730-32d PSU driver");
MODULE_LICENSE("GPL");

module_init(extreme8730_32d_psu_init);
module_exit(extreme8730_32d_psu_exit);
