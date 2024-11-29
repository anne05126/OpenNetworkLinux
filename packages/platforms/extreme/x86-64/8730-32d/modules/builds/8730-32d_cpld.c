/*
 * An I2C multiplexer dirver for extremenetworks 8730-32d CPLD
 *
 * Copyright (C) 2024 Alphanetworks Technology Corporation.
 * Anne Liou <anne_liou@alphanetworks.com>
 *
 * Based on:
 *
 * Copyright (C) 2021 Alphanetworks Technology Corporation.
 * Fillmore Chen <fillmore_chen@alphanetworks.com>
 *
 * Based on:
 *
 * Copyright (C) 2015 Accton Technology Corporation.
 * Brandon Chuang <brandon_chuang@accton.com.tw>
 *
 * Based on:
 *    pca954x.c from Kumar Gala <galak@kernel.crashing.org>
 * Copyright (C) 2006
 *
 * Based on:
 *    pca954x.c from Ken Harrenstien
 * Copyright (C) 2004 Google, Inc. (Ken Harrenstien)
 *
 * Based on:
 *    i2c-virtual_cb.c from Brian Kuschak <bkuschak@yahoo.com>
 * and
 *    pca9540.c from Jean Delvare <khali@linux-fr.org>.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/version.h>
#include <linux/stat.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>
#include <linux/printk.h>

#define DRIVER_NAME                       "8730_i2c_cpld"
#define I2C_RW_RETRY_COUNT                10
#define I2C_RW_RETRY_INTERVAL             60 /* ms */

#define CPLD_DESELECT_CHANNEL             0x00

#define CPLD_REG_ADDR_REVISION            0x00
#define CPLD_REG_ADDR_INTR_1              0x01
#define CPLD_REG_ADDR_INTR_2              0x02
#define CPLD_REG_ADDR_PRESENT_1           0x05
#define CPLD_REG_ADDR_PRESENT_2           0x06
#define CPLD_REG_ADDR_RESET_1             0x07
#define CPLD_REG_ADDR_RESET_2             0x08
#define CPLD_REG_ADDR_LOWPOWERMODE_1      0x09
#define CPLD_REG_ADDR_LOWPOWERMODE_2      0x0a
#define CPLD_REG_ADDR_MODSELECT_1         0x0b
#define CPLD_REG_ADDR_MODSELECT_2         0x0c
#define CPLD_REG_ADDR_PORT_LED_CONTROL    0x0d
/* Starting from version 5 of the port CPLD */
#define CPLD_REG_ADDR_RESET_05_1          0x09
#define CPLD_REG_ADDR_RESET_05_2          0x0a
#define CPLD_REG_ADDR_LOWPOWERMODE_05_1   0x0b
#define CPLD_REG_ADDR_LOWPOWERMODE_05_2   0x0c
#define CPLD_REG_ADDR_MODSELECT_05_1      0x0d
#define CPLD_REG_ADDR_MODSELECT_05_2      0x0e
#define CPLD_REG_ADDR_PORT_LED_CONTROL_05 0x0f

#define CPLD_VERSION_BITS_MASK    		  0xF

#define DEBUG_MODE 1

static unsigned int debug = 0;
module_param(debug, uint, S_IRUGO);
MODULE_PARM_DESC(debug, "Set DEBUG mode. Default is disabled.");


#define DEBUG_PRINT(fmt, args...)                                        \
    if (debug == 1)                                                      \
		printk (KERN_INFO "[%s,%d]: " fmt "\r\n", __FUNCTION__, __LINE__, ##args)

static LIST_HEAD(cpld_client_list);
static struct mutex     list_lock;

struct cpld_client_node {
    struct i2c_client *client;
    struct list_head   list;
};

enum cpld_type {
    extreme8730_32d_cpld1,
    extreme8730_32d_cpld2
};

struct extreme8730_32d_cpld_data {
    enum cpld_type type;
    struct device      *hwmon_dev;
    struct mutex        update_lock;
};

struct chip_desc {
    u8   nchans;
    u8   deselectChan;
};

static const struct i2c_device_id extreme8730_32d_cpld_id[] = {
    { "8730_32d_cpld1", extreme8730_32d_cpld1 },
    { "8730_32d_cpld2", extreme8730_32d_cpld2 },
    { }
};
MODULE_DEVICE_TABLE(i2c, extreme8730_32d_cpld_id);

#define TRANSCEIVER_PRESENT_ATTR_ID(index)          MODULE_PRESENT_##index
#define TRANSCEIVER_LPMODE_ATTR_ID(index)   	    MODULE_LPMODE_##index
#define TRANSCEIVER_RESET_ATTR_ID(index)   	        MODULE_RESET_##index
#define CPLD_INTR_ATTR_ID(index)   	                CPLD_INTR_##index
#define CPLD_PORT_LED_ENABLE_ATTR_ID(index)   	    CPLD_PORT_LED_ENABLE_##index

enum extreme8730_32d_cpld_sysfs_attributes {
    CPLD_VERSION,
    ACCESS,
    MODULE_PRESENT_ALL,
    /* transceiver attributes */
    TRANSCEIVER_PRESENT_ATTR_ID(1),
    TRANSCEIVER_PRESENT_ATTR_ID(2),
    TRANSCEIVER_PRESENT_ATTR_ID(3),
    TRANSCEIVER_PRESENT_ATTR_ID(4),
    TRANSCEIVER_PRESENT_ATTR_ID(5),
    TRANSCEIVER_PRESENT_ATTR_ID(6),
    TRANSCEIVER_PRESENT_ATTR_ID(7),
    TRANSCEIVER_PRESENT_ATTR_ID(8),
    TRANSCEIVER_PRESENT_ATTR_ID(9),
    TRANSCEIVER_PRESENT_ATTR_ID(10),
    TRANSCEIVER_PRESENT_ATTR_ID(11),
    TRANSCEIVER_PRESENT_ATTR_ID(12),
    TRANSCEIVER_PRESENT_ATTR_ID(13),
    TRANSCEIVER_PRESENT_ATTR_ID(14),
    TRANSCEIVER_PRESENT_ATTR_ID(15),
    TRANSCEIVER_PRESENT_ATTR_ID(16),
    TRANSCEIVER_PRESENT_ATTR_ID(17),
    TRANSCEIVER_PRESENT_ATTR_ID(18),
    TRANSCEIVER_PRESENT_ATTR_ID(19),
    TRANSCEIVER_PRESENT_ATTR_ID(20),
    TRANSCEIVER_PRESENT_ATTR_ID(21),
    TRANSCEIVER_PRESENT_ATTR_ID(22),
    TRANSCEIVER_PRESENT_ATTR_ID(23),
    TRANSCEIVER_PRESENT_ATTR_ID(24),
    TRANSCEIVER_PRESENT_ATTR_ID(25),
    TRANSCEIVER_PRESENT_ATTR_ID(26),
    TRANSCEIVER_PRESENT_ATTR_ID(27),
    TRANSCEIVER_PRESENT_ATTR_ID(28),
    TRANSCEIVER_PRESENT_ATTR_ID(29),
    TRANSCEIVER_PRESENT_ATTR_ID(30),
    TRANSCEIVER_PRESENT_ATTR_ID(31),
    TRANSCEIVER_PRESENT_ATTR_ID(32),

    TRANSCEIVER_LPMODE_ATTR_ID(1),
    TRANSCEIVER_LPMODE_ATTR_ID(2),
    TRANSCEIVER_LPMODE_ATTR_ID(3),
    TRANSCEIVER_LPMODE_ATTR_ID(4),
    TRANSCEIVER_LPMODE_ATTR_ID(5),
    TRANSCEIVER_LPMODE_ATTR_ID(6),
    TRANSCEIVER_LPMODE_ATTR_ID(7),
    TRANSCEIVER_LPMODE_ATTR_ID(8),
    TRANSCEIVER_LPMODE_ATTR_ID(9),
    TRANSCEIVER_LPMODE_ATTR_ID(10),
    TRANSCEIVER_LPMODE_ATTR_ID(11),
    TRANSCEIVER_LPMODE_ATTR_ID(12),
    TRANSCEIVER_LPMODE_ATTR_ID(13),
    TRANSCEIVER_LPMODE_ATTR_ID(14),
    TRANSCEIVER_LPMODE_ATTR_ID(15),
    TRANSCEIVER_LPMODE_ATTR_ID(16),
    TRANSCEIVER_LPMODE_ATTR_ID(17),
    TRANSCEIVER_LPMODE_ATTR_ID(18),
    TRANSCEIVER_LPMODE_ATTR_ID(19),
    TRANSCEIVER_LPMODE_ATTR_ID(20),
    TRANSCEIVER_LPMODE_ATTR_ID(21),
    TRANSCEIVER_LPMODE_ATTR_ID(22),
    TRANSCEIVER_LPMODE_ATTR_ID(23),
    TRANSCEIVER_LPMODE_ATTR_ID(24),
    TRANSCEIVER_LPMODE_ATTR_ID(25),
    TRANSCEIVER_LPMODE_ATTR_ID(26),
    TRANSCEIVER_LPMODE_ATTR_ID(27),
    TRANSCEIVER_LPMODE_ATTR_ID(28),
    TRANSCEIVER_LPMODE_ATTR_ID(29),
    TRANSCEIVER_LPMODE_ATTR_ID(30),
    TRANSCEIVER_LPMODE_ATTR_ID(31),
    TRANSCEIVER_LPMODE_ATTR_ID(32),
    TRANSCEIVER_RESET_ATTR_ID(1),
    TRANSCEIVER_RESET_ATTR_ID(2),
    TRANSCEIVER_RESET_ATTR_ID(3),
    TRANSCEIVER_RESET_ATTR_ID(4),
    TRANSCEIVER_RESET_ATTR_ID(5),
    TRANSCEIVER_RESET_ATTR_ID(6),
    TRANSCEIVER_RESET_ATTR_ID(7),
    TRANSCEIVER_RESET_ATTR_ID(8),
    TRANSCEIVER_RESET_ATTR_ID(9),
    TRANSCEIVER_RESET_ATTR_ID(10),
    TRANSCEIVER_RESET_ATTR_ID(11),
    TRANSCEIVER_RESET_ATTR_ID(12),
    TRANSCEIVER_RESET_ATTR_ID(13),
    TRANSCEIVER_RESET_ATTR_ID(14),
    TRANSCEIVER_RESET_ATTR_ID(15),
    TRANSCEIVER_RESET_ATTR_ID(16),
    TRANSCEIVER_RESET_ATTR_ID(17),
    TRANSCEIVER_RESET_ATTR_ID(18),
    TRANSCEIVER_RESET_ATTR_ID(19),
    TRANSCEIVER_RESET_ATTR_ID(20),
    TRANSCEIVER_RESET_ATTR_ID(21),
    TRANSCEIVER_RESET_ATTR_ID(22),
    TRANSCEIVER_RESET_ATTR_ID(23),
    TRANSCEIVER_RESET_ATTR_ID(24),
    TRANSCEIVER_RESET_ATTR_ID(25),
    TRANSCEIVER_RESET_ATTR_ID(26),
    TRANSCEIVER_RESET_ATTR_ID(27),
    TRANSCEIVER_RESET_ATTR_ID(28),
    TRANSCEIVER_RESET_ATTR_ID(29),
    TRANSCEIVER_RESET_ATTR_ID(30),
    TRANSCEIVER_RESET_ATTR_ID(31),
    TRANSCEIVER_RESET_ATTR_ID(32),
	CPLD_INTR_ATTR_ID(1),
	CPLD_INTR_ATTR_ID(2),
	CPLD_INTR_ATTR_ID(3),
	CPLD_INTR_ATTR_ID(4),
	CPLD_PORT_LED_ENABLE_ATTR_ID(1),
	CPLD_PORT_LED_ENABLE_ATTR_ID(2),
};
/* sysfs attributes for hwmon
 */
static ssize_t show_led_control(struct device *dev, struct device_attribute *da,
                                char *buf);
static ssize_t show_interrupt(struct device *dev, struct device_attribute *da,
                              char *buf);
static ssize_t show_status(struct device *dev, struct device_attribute *da,
                           char *buf);
static ssize_t show_present_all(struct device *dev, struct device_attribute *da,
                                char *buf);
static ssize_t set_led_control(struct device *dev, struct device_attribute *da,
                               const char *buf, size_t count);
static ssize_t set_lp_mode(struct device *dev, struct device_attribute *da,
                           const char *buf, size_t count);
static ssize_t set_mode_reset(struct device *dev, struct device_attribute *da,
                              const char *buf, size_t count);
static ssize_t access(struct device *dev, struct device_attribute *da,
                      const char *buf, size_t count);
static ssize_t show_version(struct device *dev, struct device_attribute *da,
                            char *buf);
static int extreme8730_32d_cpld_read_internal(struct i2c_client *client, u8 reg);
static int extreme8730_32d_cpld_write_internal(struct i2c_client *client, u8 reg, u8 value);

/* transceiver attributes */
#define DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(module_present_##index, S_IRUGO, show_status, NULL, MODULE_PRESENT_##index)
#define DECLARE_TRANSCEIVER_PRESENT_ATTR(index)  &sensor_dev_attr_module_present_##index.dev_attr.attr

#define DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(index) \
    static SENSOR_DEVICE_ATTR(module_lp_mode_##index, S_IRUGO | S_IWUSR, show_status, set_lp_mode, MODULE_LPMODE_##index); \
    static SENSOR_DEVICE_ATTR(module_reset_##index,   S_IRUGO | S_IWUSR, show_status, set_mode_reset, MODULE_RESET_##index)

#define DECLARE_QSFP_TRANSCEIVER_ATTR(index)  \
	&sensor_dev_attr_module_lp_mode_##index.dev_attr.attr,	\
	&sensor_dev_attr_module_reset_##index.dev_attr.attr

/* cpld interrupt */
#define DECLARE_CPLD_INTR_DEVICE_ATTR(index) \
	static SENSOR_DEVICE_ATTR(cpld_intr_##index, S_IRUGO, show_interrupt, NULL, CPLD_INTR_##index)
#define DECLARE_CPLD_INTR_ATTR(index)  &sensor_dev_attr_cpld_intr_##index.dev_attr.attr

/* cpld port_led_control */
#define DECLARE_CPLD_PORT_LED_ENABLE_DEVICE_ATTR(index) \
	static SENSOR_DEVICE_ATTR(cpld_port_led_enable_##index, S_IRUGO | S_IWUSR, show_led_control, set_led_control, CPLD_PORT_LED_ENABLE_##index)
#define DECLARE_CPLD_PORT_LED_ENABLE_ATTR(index)  &sensor_dev_attr_cpld_port_led_enable_##index.dev_attr.attr

static SENSOR_DEVICE_ATTR(version, S_IRUGO, show_version, NULL, CPLD_VERSION);
static SENSOR_DEVICE_ATTR(access, S_IWUSR, NULL, access, ACCESS);
/* transceiver attributes */
static SENSOR_DEVICE_ATTR(module_present_all, S_IRUGO, show_present_all, NULL, MODULE_PRESENT_ALL);

DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(1);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(2);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(3);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(4);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(5);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(6);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(7);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(8);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(9);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(10);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(11);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(12);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(13);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(14);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(15);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(16);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(17);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(18);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(19);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(20);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(21);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(22);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(23);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(24);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(25);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(26);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(27);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(28);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(29);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(30);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(31);
DECLARE_TRANSCEIVER_PRESENT_SENSOR_DEVICE_ATTR(32);

DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(1);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(2);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(3);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(4);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(5);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(6);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(7);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(8);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(9);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(10);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(11);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(12);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(13);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(14);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(15);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(16);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(17);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(18);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(19);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(20);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(21);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(22);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(23);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(24);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(25);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(26);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(27);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(28);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(29);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(30);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(31);
DECLARE_QSFP_TRANSCEIVER_SENSOR_DEVICE_ATTR(32);
DECLARE_CPLD_INTR_DEVICE_ATTR(1);
DECLARE_CPLD_INTR_DEVICE_ATTR(2);
DECLARE_CPLD_INTR_DEVICE_ATTR(3);
DECLARE_CPLD_INTR_DEVICE_ATTR(4);
DECLARE_CPLD_PORT_LED_ENABLE_DEVICE_ATTR(1);
DECLARE_CPLD_PORT_LED_ENABLE_DEVICE_ATTR(2);

static struct attribute *extreme8730_32d_cpld1_attributes[] = {
    &sensor_dev_attr_version.dev_attr.attr,
    &sensor_dev_attr_access.dev_attr.attr,
    /* transceiver attributes */
    &sensor_dev_attr_module_present_all.dev_attr.attr,
    DECLARE_TRANSCEIVER_PRESENT_ATTR(1),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(2),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(3),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(4),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(5),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(6),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(7),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(8),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(9),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(10),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(11),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(12),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(13),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(14),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(15),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(16),
    DECLARE_QSFP_TRANSCEIVER_ATTR(1),
    DECLARE_QSFP_TRANSCEIVER_ATTR(2),
    DECLARE_QSFP_TRANSCEIVER_ATTR(3),
    DECLARE_QSFP_TRANSCEIVER_ATTR(4),
    DECLARE_QSFP_TRANSCEIVER_ATTR(5),
    DECLARE_QSFP_TRANSCEIVER_ATTR(6),
    DECLARE_QSFP_TRANSCEIVER_ATTR(7),
    DECLARE_QSFP_TRANSCEIVER_ATTR(8),
    DECLARE_QSFP_TRANSCEIVER_ATTR(9),
    DECLARE_QSFP_TRANSCEIVER_ATTR(10),
    DECLARE_QSFP_TRANSCEIVER_ATTR(11),
    DECLARE_QSFP_TRANSCEIVER_ATTR(12),
    DECLARE_QSFP_TRANSCEIVER_ATTR(13),
    DECLARE_QSFP_TRANSCEIVER_ATTR(14),
    DECLARE_QSFP_TRANSCEIVER_ATTR(15),
    DECLARE_QSFP_TRANSCEIVER_ATTR(16),
    DECLARE_CPLD_INTR_ATTR(1),
    DECLARE_CPLD_INTR_ATTR(2),
    DECLARE_CPLD_PORT_LED_ENABLE_ATTR(1),
    NULL
};

static const struct attribute_group extreme8730_32d_cpld1_group = {
    .attrs = extreme8730_32d_cpld1_attributes,
};

static struct attribute *extreme8730_32d_cpld2_attributes[] = {
    &sensor_dev_attr_version.dev_attr.attr,
    &sensor_dev_attr_access.dev_attr.attr,
    /* transceiver attributes */
    &sensor_dev_attr_module_present_all.dev_attr.attr,
    DECLARE_TRANSCEIVER_PRESENT_ATTR(17),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(18),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(19),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(20),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(21),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(22),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(23),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(24),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(25),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(26),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(27),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(28),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(29),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(30),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(31),
    DECLARE_TRANSCEIVER_PRESENT_ATTR(32),
    DECLARE_QSFP_TRANSCEIVER_ATTR(17),
    DECLARE_QSFP_TRANSCEIVER_ATTR(18),
    DECLARE_QSFP_TRANSCEIVER_ATTR(19),
    DECLARE_QSFP_TRANSCEIVER_ATTR(20),
    DECLARE_QSFP_TRANSCEIVER_ATTR(21),
    DECLARE_QSFP_TRANSCEIVER_ATTR(22),
    DECLARE_QSFP_TRANSCEIVER_ATTR(23),
    DECLARE_QSFP_TRANSCEIVER_ATTR(24),
    DECLARE_QSFP_TRANSCEIVER_ATTR(25),
    DECLARE_QSFP_TRANSCEIVER_ATTR(26),
    DECLARE_QSFP_TRANSCEIVER_ATTR(27),
    DECLARE_QSFP_TRANSCEIVER_ATTR(28),
    DECLARE_QSFP_TRANSCEIVER_ATTR(29),
    DECLARE_QSFP_TRANSCEIVER_ATTR(30),
    DECLARE_QSFP_TRANSCEIVER_ATTR(31),
    DECLARE_QSFP_TRANSCEIVER_ATTR(32),
    DECLARE_CPLD_INTR_ATTR(3),
    DECLARE_CPLD_INTR_ATTR(4),
    DECLARE_CPLD_PORT_LED_ENABLE_ATTR(2),
    NULL
};

static const struct attribute_group extreme8730_32d_cpld2_group = {
    .attrs = extreme8730_32d_cpld2_attributes,
};

static ssize_t show_present_all(struct device *dev, struct device_attribute *da,
                                char *buf)
{
    int i, status, num_regs = 0;
    u8 values[4]  = {0};
    u8 regs[] = {CPLD_REG_ADDR_PRESENT_1, CPLD_REG_ADDR_PRESENT_2};
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme8730_32d_cpld_data *data = i2c_get_clientdata(client);

    int cpld_version;
    cpld_version = extreme8730_32d_cpld_read_internal(client, CPLD_REG_ADDR_REVISION);

    mutex_lock(&data->update_lock);

    num_regs = 2;

    for (i = 0; i < num_regs; i++) {
        status = extreme8730_32d_cpld_read_internal(client, regs[i]);

        if (status < 0) {
            goto exit;
        }

        values[i] = (u8)status;
    }

    mutex_unlock(&data->update_lock);

    /* Return values 1 -> 8 in order */
    if(cpld_version > 4)
        status = sprintf(buf, "%.2x %.2x\n", (u8)~values[0], (u8)~values[1]);
    else
        status = sprintf(buf, "%.2x %.2x\n", values[0], values[1]);

    return status;

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t show_led_control(struct device *dev, struct device_attribute *da,
                                char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme8730_32d_cpld_data *data = i2c_get_clientdata(client);
    int status = 0;
    u8 reg = 0, mask = 0, revert = 1;

    int cpld_version;
    cpld_version = extreme8730_32d_cpld_read_internal(client, CPLD_REG_ADDR_REVISION);
    
    switch (attr->index)
	{   
        case CPLD_PORT_LED_ENABLE_1:
            if(cpld_version > 4)
                reg  = CPLD_REG_ADDR_PORT_LED_CONTROL_05;
            else
                reg  = CPLD_REG_ADDR_PORT_LED_CONTROL;
			mask = 0x1;
            break;
        case CPLD_PORT_LED_ENABLE_2:
            if(cpld_version > 4)
                reg  = CPLD_REG_ADDR_PORT_LED_CONTROL_05;
            else
                reg  = CPLD_REG_ADDR_PORT_LED_CONTROL;
			mask = 0x1;
            break;
        default:
            return -ENODEV;
    }

    mutex_lock(&data->update_lock);
    status = extreme8730_32d_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
        goto exit;
    }
    mutex_unlock(&data->update_lock);

    return sprintf(buf, "%d\n", revert ? !(status & mask) : !!(status & mask));

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t show_interrupt(struct device *dev, struct device_attribute *da,
                              char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme8730_32d_cpld_data *data = i2c_get_clientdata(client);
    int status = 0;
    u8 reg = 0; 

    int cpld_version;
    cpld_version = extreme8730_32d_cpld_read_internal(client, CPLD_REG_ADDR_REVISION);
    
    switch (attr->index)
	{   
        case CPLD_INTR_1:
            reg  = CPLD_REG_ADDR_INTR_1;
            break;
        case CPLD_INTR_2:
            reg  = CPLD_REG_ADDR_INTR_2;
            break;
        case CPLD_INTR_3:
            reg  = CPLD_REG_ADDR_INTR_1;
            break;
        case CPLD_INTR_4:
            reg  = CPLD_REG_ADDR_INTR_2;
            break; 
        default:
            return -ENODEV;
    }

    mutex_lock(&data->update_lock);
    status = extreme8730_32d_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
        goto exit;
    }
    mutex_unlock(&data->update_lock);

    if(cpld_version > 4)
        return sprintf(buf, "0x%x\n", (u8)~status); /* 0: interrupt, 1: no interrupt*/
    else
        return sprintf(buf, "0x%x\n", status);

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t show_status(struct device *dev, struct device_attribute *da,
                           char *buf)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme8730_32d_cpld_data *data = i2c_get_clientdata(client);
    int status = 0;
    u8 reg = 0, mask = 0;

    int cpld_version;
    cpld_version = extreme8730_32d_cpld_read_internal(client, CPLD_REG_ADDR_REVISION);

    switch (attr->index) {
    case MODULE_PRESENT_1 ... MODULE_PRESENT_8:
        reg  = CPLD_REG_ADDR_PRESENT_1;
        mask = 0x1 << (attr->index - MODULE_PRESENT_1);
        break;
    case MODULE_PRESENT_9 ... MODULE_PRESENT_16:
        reg  = CPLD_REG_ADDR_PRESENT_2;
        mask = 0x1 << (attr->index - MODULE_PRESENT_9);
        break;
    case MODULE_PRESENT_17 ... MODULE_PRESENT_24:
        reg  = CPLD_REG_ADDR_PRESENT_1;
        mask = 0x1 << (attr->index - MODULE_PRESENT_17);
        break;
    case MODULE_PRESENT_25 ... MODULE_PRESENT_32:
        reg  = CPLD_REG_ADDR_PRESENT_2;
        mask = 0x1 << (attr->index - MODULE_PRESENT_25);
        break;
    case MODULE_LPMODE_1 ... MODULE_LPMODE_8:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_05_1;
        else
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_1;
        mask = 0x1 << (attr->index - MODULE_LPMODE_1);
        break;
    case MODULE_LPMODE_9 ... MODULE_LPMODE_16:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_05_2;
        else
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_2;
        mask = 0x1 << (attr->index - MODULE_LPMODE_9);
        break;
    case MODULE_LPMODE_17 ... MODULE_LPMODE_24:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_05_1;
        else
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_1;
        mask = 0x1 << (attr->index - MODULE_LPMODE_17);
        break;
    case MODULE_LPMODE_25 ... MODULE_LPMODE_32:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_05_2;
        else
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_2;
        mask = 0x1 << (attr->index - MODULE_LPMODE_25);
        break;
    case MODULE_RESET_1 ... MODULE_RESET_8:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_RESET_05_1;
        else
            reg  = CPLD_REG_ADDR_RESET_1;
        mask = 0x1 << (attr->index - MODULE_RESET_1);
        break;
    case MODULE_RESET_9 ... MODULE_RESET_16:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_RESET_05_2;
        else
            reg  = CPLD_REG_ADDR_RESET_2;
        mask = 0x1 << (attr->index - MODULE_RESET_9);
        break;
    case MODULE_RESET_17 ... MODULE_RESET_24:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_RESET_05_1;
        else
            reg  = CPLD_REG_ADDR_RESET_1;
        mask = 0x1 << (attr->index - MODULE_RESET_17);
        break;
    case MODULE_RESET_25 ... MODULE_RESET_32:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_RESET_05_2;
        else
            reg  = CPLD_REG_ADDR_RESET_2;
        mask = 0x1 << (attr->index - MODULE_RESET_25);
        break;
    default:
        return 0;
    }

    mutex_lock(&data->update_lock);
    status = extreme8730_32d_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
        goto exit;
    }
    mutex_unlock(&data->update_lock);

    switch (attr->index) {
    case MODULE_PRESENT_1 ... MODULE_PRESENT_8:
    case MODULE_PRESENT_9 ... MODULE_PRESENT_16:
    case MODULE_PRESENT_17 ... MODULE_PRESENT_24:
    case MODULE_PRESENT_25 ... MODULE_PRESENT_32:
    case MODULE_RESET_1 ... MODULE_RESET_8:
    case MODULE_RESET_9 ... MODULE_RESET_16:
    case MODULE_RESET_17 ... MODULE_RESET_24:
    case MODULE_RESET_25 ... MODULE_RESET_32:
        if(cpld_version > 4)
            return sprintf(buf, "%d\n", !(status & mask)); /* 0: present, 1: no present; 0: reset, 1: normal */
        break;
    default:
        break;
    }

    return sprintf(buf, "%d\n", !!(status & mask));

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static ssize_t set_led_control(struct device *dev, struct device_attribute *da,
                               const char *buf, size_t count)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme8730_32d_cpld_data *data = i2c_get_clientdata(client);

    long on;
    int status= -ENOENT;
    u8 reg = 0, mask = 0;

    int cpld_version;
    cpld_version = extreme8730_32d_cpld_read_internal(client, CPLD_REG_ADDR_REVISION);

    if(attr->index < CPLD_PORT_LED_ENABLE_1 || attr->index > CPLD_PORT_LED_ENABLE_2)
        return status;

    status = kstrtol(buf, 10, &on);
    if (status) {
        return status;
    }

	if ((on != 1) && (on != 0))
        return -EINVAL;
		
    switch (attr->index)
	{   
        case CPLD_PORT_LED_ENABLE_1:
            if(cpld_version > 4)
                reg  = CPLD_REG_ADDR_PORT_LED_CONTROL_05;
            else
                reg  = CPLD_REG_ADDR_PORT_LED_CONTROL;
			mask = 0x1;
            break;
        case CPLD_PORT_LED_ENABLE_2:
            if(cpld_version > 4)
                reg  = CPLD_REG_ADDR_PORT_LED_CONTROL_05;
            else
                reg  = CPLD_REG_ADDR_PORT_LED_CONTROL;
			mask = 0x1;
            break;
        default:
            return -ENODEV;
    }

    /* Read current status */
    mutex_lock(&data->update_lock);
    status = extreme8730_32d_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
        goto exit;
    }

    /* Update led_control status */
    if (on) {
        status &= ~mask;
    }
    else {
        status |= mask;
    }

    status = extreme8730_32d_cpld_write_internal(client, reg, status);
    if (unlikely(status < 0)) {
        goto exit;
    }

    mutex_unlock(&data->update_lock);
    return count;

exit:
    mutex_unlock(&data->update_lock);
    return status;

}

static ssize_t set_lp_mode(struct device *dev, struct device_attribute *da,
                           const char *buf, size_t count)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme8730_32d_cpld_data *data = i2c_get_clientdata(client);

    long on;
    int status= -ENOENT;
    u8 reg = 0, mask = 0;

    int cpld_version;
    cpld_version = extreme8730_32d_cpld_read_internal(client, CPLD_REG_ADDR_REVISION);

    if(attr->index < MODULE_LPMODE_1 || attr->index > MODULE_LPMODE_32)
        return status;

    status = kstrtol(buf, 10, &on);
    if (status) {
        return status;
    }

	if ((on != 1) && (on != 0))
        return -EINVAL;

    switch (attr->index) {
    case MODULE_LPMODE_1 ... MODULE_LPMODE_8:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_05_1;
        else
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_1;
        mask = 0x1 << (attr->index - MODULE_LPMODE_1);
        break;
    case MODULE_LPMODE_9 ... MODULE_LPMODE_16:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_05_2;
        else
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_2;
        mask = 0x1 << (attr->index - MODULE_LPMODE_9);
        break;
    case MODULE_LPMODE_17 ... MODULE_LPMODE_24:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_05_1;
        else
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_1;
        mask = 0x1 << (attr->index - MODULE_LPMODE_17);
        break;
    case MODULE_LPMODE_25 ... MODULE_LPMODE_32:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_05_2;
        else
            reg  = CPLD_REG_ADDR_LOWPOWERMODE_2;
        mask = 0x1 << (attr->index - MODULE_LPMODE_25);
        break;
    default:
        return 0;
    }

    /* Read current status */
    mutex_lock(&data->update_lock);
    status = extreme8730_32d_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
        goto exit;
    }

    /* Update lp_mode status */
    if (on) {
        status |= mask;
    }
    else {
        status &= ~mask;
    }

    status = extreme8730_32d_cpld_write_internal(client, reg, status);
    if (unlikely(status < 0)) {
        goto exit;
    }

    mutex_unlock(&data->update_lock);
    return count;

exit:
    mutex_unlock(&data->update_lock);
    return status;

}

static ssize_t set_mode_reset(struct device *dev, struct device_attribute *da,
                              const char *buf, size_t count)
{
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct i2c_client *client = to_i2c_client(dev);
    struct extreme8730_32d_cpld_data *data = i2c_get_clientdata(client);

    long on;
    int status= -ENOENT;
    u8 reg = 0, mask = 0;

    int cpld_version;
    cpld_version = extreme8730_32d_cpld_read_internal(client, CPLD_REG_ADDR_REVISION);

    if(attr->index < MODULE_RESET_1 || attr->index > MODULE_RESET_32)
        return status;

    status = kstrtol(buf, 10, &on);
    if (status) {
        return status;
    }

	if ((on != 1) && (on != 0))
        return -EINVAL;

    switch (attr->index) {
    case MODULE_RESET_1 ... MODULE_RESET_8:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_RESET_05_1;
        else
            reg  = CPLD_REG_ADDR_RESET_1;
        mask = 0x1 << (attr->index - MODULE_RESET_1);
        break;
    case MODULE_RESET_9 ... MODULE_RESET_16:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_RESET_05_2;
        else
            reg  = CPLD_REG_ADDR_RESET_2;
        mask = 0x1 << (attr->index - MODULE_RESET_9);
        break;
    case MODULE_RESET_17 ... MODULE_RESET_24:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_RESET_05_1;
        else
            reg  = CPLD_REG_ADDR_RESET_1;
        mask = 0x1 << (attr->index - MODULE_RESET_17);
        break;
    case MODULE_RESET_25 ... MODULE_RESET_32:
        if(cpld_version > 4)
            reg  = CPLD_REG_ADDR_RESET_05_2;
        else
            reg  = CPLD_REG_ADDR_RESET_2;
        mask = 0x1 << (attr->index - MODULE_RESET_25);
        break;
    default:
        return 0;
    }

    /* Read current status */
    mutex_lock(&data->update_lock);
    status = extreme8730_32d_cpld_read_internal(client, reg);
    if (unlikely(status < 0)) {
        goto exit;
    }

    /* Update reset status */
    if (on) {
        if(cpld_version > 4)
            status &= ~mask;
        else
            status |= mask;
    }
    else {
        if(cpld_version > 4)
            status |= mask;
        else
            status &= ~mask;
    }

    status = extreme8730_32d_cpld_write_internal(client, reg, status);
    if (unlikely(status < 0)) {
        goto exit;
    }

    mutex_unlock(&data->update_lock);
    return count;

exit:
    mutex_unlock(&data->update_lock);
    return status;

}

static ssize_t access(struct device *dev, struct device_attribute *da,
                      const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);

    struct extreme8730_32d_cpld_data *data = i2c_get_clientdata(client);
    int status;
    u32 addr, val;

    if (sscanf(buf, "0x%x 0x%x", &addr, &val) != 2) {
        return -EINVAL;
    }

    if (addr > 0xFF || val > 0xFF) {
        return -EINVAL;
    }

    mutex_lock(&data->update_lock);
    status = extreme8730_32d_cpld_write_internal(client, addr, val);
    if (unlikely(status < 0)) {
        goto exit;
    }
    mutex_unlock(&data->update_lock);
    return count;

exit:
    mutex_unlock(&data->update_lock);
    return status;
}

static void extreme8730_32d_cpld_add_client(struct i2c_client *client)
{
    struct cpld_client_node *node = kzalloc(sizeof(struct cpld_client_node), GFP_KERNEL);

    if (!node) {
        dev_dbg(&client->dev, "Can't allocate cpld_client_node (0x%x)\n", client->addr);
        return;
    }

    node->client = client;

    mutex_lock(&list_lock);
    list_add(&node->list, &cpld_client_list);
    mutex_unlock(&list_lock);
}

static void extreme8730_32d_cpld_remove_client(struct i2c_client *client)
{
    struct list_head    *list_node = NULL;
    struct cpld_client_node *cpld_node = NULL;
    int found = 0;

    mutex_lock(&list_lock);

    list_for_each(list_node, &cpld_client_list) {
        cpld_node = list_entry(list_node, struct cpld_client_node, list);

        if (cpld_node->client == client) {
            found = 1;
            break;
        }
    }

    if (found) {
        list_del(list_node);
        kfree(cpld_node);
    }

    mutex_unlock(&list_lock);
}

static ssize_t show_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    int val = 0;
    struct i2c_client *client = to_i2c_client(dev);

    val = i2c_smbus_read_byte_data(client, CPLD_REG_ADDR_REVISION);

    if (val < 0) {
        dev_dbg(&client->dev, "cpld(0x%x) reg(CPLD_REG_ADDR_REVISION) err %d\n", client->addr, val);
    }

	val = val & CPLD_VERSION_BITS_MASK;

    return sprintf(buf, "%d", val);
}

/*
 * I2C init/probing/exit functions
 */
static int extreme8730_32d_cpld_probe(struct i2c_client *client,
                                     const struct i2c_device_id *id)
{
    struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
    struct extreme8730_32d_cpld_data *data;
    int ret = 0;
    const struct attribute_group *group = NULL;

    if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE))
        return -ENODEV;

    data = kzalloc(sizeof(struct extreme8730_32d_cpld_data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    i2c_set_clientdata(client, data);
    data->type = id->driver_data;
    mutex_init(&data->update_lock);

    /* Register sysfs hooks */
    int cpld_version;
    cpld_version = extreme8730_32d_cpld_read_internal(client, CPLD_REG_ADDR_REVISION);
    switch (data->type) {
    case extreme8730_32d_cpld1:
        group = &extreme8730_32d_cpld1_group;
        if(cpld_version > 4)
        {
            /* Bring QSFPs out of reset for Port 1-8 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_RESET_05_1, 0xFF);
		    /* Bring QSFPs out of reset for Port 9-16 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_RESET_05_2, 0xFF);
            /* Set Module Selector for transceiver EEPROM for Port 1-8 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_MODSELECT_05_1, 0x0);
		    /* Set Module Selector for transceiver EEPROM for Port 9-16 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_MODSELECT_05_2, 0x0);
        }
        else
        {
            /* Bring QSFPs out of reset for Port 1-8 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_RESET_1, 0x0);
		    /* Bring QSFPs out of reset for Port 9-16 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_RESET_2, 0x0);
            /* Set Module Selector for transceiver EEPROM for Port 1-8 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_MODSELECT_1, 0xFF);
		    /* Set Module Selector for transceiver EEPROM for Port 9-16 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_MODSELECT_2, 0xFF);
        }
        break;
    case extreme8730_32d_cpld2:
        group = &extreme8730_32d_cpld2_group;
        if(cpld_version > 4)
        {
            /* Bring QSFPs out of reset for Port 17-24 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_RESET_05_1, 0xFF);
		    /* Bring QSFPs out of reset for Port 25-32 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_RESET_05_2, 0xFF);
            /* Set Module Selector for transceiver EEPROM for Port 17-24 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_MODSELECT_05_1, 0x0);
		    /* Set Module Selector for transceiver EEPROM for Port 25-32 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_MODSELECT_05_2, 0x0);
        }
        else
        {
            /* Bring QSFPs out of reset for Port 17-24 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_RESET_1, 0x0);
		    /* Bring QSFPs out of reset for Port 25-32 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_RESET_2, 0x0);
            /* Set Module Selector for transceiver EEPROM for Port 17-24 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_MODSELECT_1, 0xFF);
		    /* Set Module Selector for transceiver EEPROM for Port 25-32 */
            extreme8730_32d_cpld_write_internal(client, CPLD_REG_ADDR_MODSELECT_2, 0xFF);
        }
        break;
    default:
        break;
    }

    if (group) {
        ret = sysfs_create_group(&client->dev.kobj, group);
        if (ret) {
            goto add_failed;
        }
    }

    dev_info(&client->dev,
             "device %s registered\n", client->name);

    extreme8730_32d_cpld_add_client(client);

    return 0;

add_failed:
    kfree(data);
    return ret;
}

static int extreme8730_32d_cpld_remove(struct i2c_client *client)
{
    struct extreme8730_32d_cpld_data *data = i2c_get_clientdata(client);
    const struct attribute_group *group = NULL;

    extreme8730_32d_cpld_remove_client(client);

    /* Remove sysfs hooks */
    switch (data->type) {
    case extreme8730_32d_cpld1:
        group = &extreme8730_32d_cpld1_group;
        break;
    case extreme8730_32d_cpld2:
        group = &extreme8730_32d_cpld2_group;
        break;
    default:
        break;
    }

    if (group) {
        sysfs_remove_group(&client->dev.kobj, group);
    }

    kfree(data);

    return 0;
}

static int extreme8730_32d_cpld_read_internal(struct i2c_client *client, u8 reg)
{
    int status = 0, retry = I2C_RW_RETRY_COUNT;

    while (retry) {
        status = i2c_smbus_read_byte_data(client, reg);
        if (unlikely(status < 0)) {
            msleep(I2C_RW_RETRY_INTERVAL);
            retry--;
            continue;
        }

        break;
    }
    DEBUG_PRINT("extreme8730_32d_cpld_read_internal: reg:0x%x, status:0x%x", reg, status);
    return status;
}

static int extreme8730_32d_cpld_write_internal(struct i2c_client *client, u8 reg, u8 value)
{
    int status = 0, retry = I2C_RW_RETRY_COUNT;

    while (retry) {
        status = i2c_smbus_write_byte_data(client, reg, value);
        if (unlikely(status < 0)) {
            msleep(I2C_RW_RETRY_INTERVAL);
            retry--;
            continue;
        }

        break;
    }
    DEBUG_PRINT("extreme8730_32d_cpld_write_internal: reg:0x%x, val:0x%x , status:0x%x", reg, value, status);
    return status;
}

int extreme8730_32d_cpld_read(unsigned short cpld_addr, u8 reg)
{
    struct list_head   *list_node = NULL;
    struct cpld_client_node *cpld_node = NULL;
    int ret = -EPERM;

    mutex_lock(&list_lock);

    list_for_each(list_node, &cpld_client_list) {
        cpld_node = list_entry(list_node, struct cpld_client_node, list);

        if (cpld_node->client->addr == cpld_addr) {
            ret = extreme8730_32d_cpld_read_internal(cpld_node->client, reg);
            break;
        }
    }

    mutex_unlock(&list_lock);

    return ret;
}
EXPORT_SYMBOL(extreme8730_32d_cpld_read);

int extreme8730_32d_cpld_write(unsigned short cpld_addr, u8 reg, u8 value)
{
    struct list_head   *list_node = NULL;
    struct cpld_client_node *cpld_node = NULL;
    int ret = -EIO;

    mutex_lock(&list_lock);

    list_for_each(list_node, &cpld_client_list) {
        cpld_node = list_entry(list_node, struct cpld_client_node, list);

        if (cpld_node->client->addr == cpld_addr) {
            ret = extreme8730_32d_cpld_write_internal(cpld_node->client, reg, value);
            break;
        }
    }

    mutex_unlock(&list_lock);

    return ret;
}
EXPORT_SYMBOL(extreme8730_32d_cpld_write);

static struct i2c_driver extreme8730_32d_cpld_driver = {
    .driver        = {
        .name     = DRIVER_NAME,
        .owner    = THIS_MODULE,
    },
    .probe         = extreme8730_32d_cpld_probe,
    .remove        = extreme8730_32d_cpld_remove,
    .id_table      = extreme8730_32d_cpld_id,
};

static int __init extreme8730_32d_cpld_init(void)
{
    mutex_init(&list_lock);
    return i2c_add_driver(&extreme8730_32d_cpld_driver);
}

static void __exit extreme8730_32d_cpld_exit(void)
{
    i2c_del_driver(&extreme8730_32d_cpld_driver);
}

MODULE_AUTHOR("Alpha-SID2");
MODULE_DESCRIPTION("Extreme 8730_32d CPLD driver");
MODULE_LICENSE("GPL");

module_init(extreme8730_32d_cpld_init);
module_exit(extreme8730_32d_cpld_exit);

