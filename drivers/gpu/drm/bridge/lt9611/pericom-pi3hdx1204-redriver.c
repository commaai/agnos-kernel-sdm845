/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>

static struct i2c_client *this_client = NULL;

static int i2c_read(struct i2c_client *client, u8 *data, int size)
{
	struct i2c_msg msg[2];
	u8 buf[1] = {0x00};

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size;
	msg[1].buf = data;

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		printk("%s: i2c read failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int i2c_write(struct i2c_client *client, u8 *data, int size)
{
	struct i2c_msg msg;

	if (!client) {
		printk("%s: Invalid params\n", __func__);
		return -EINVAL;
	}

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = size;
	msg.buf = data;

	if (i2c_transfer(client->adapter, &msg, 1) != 1) {
		printk("%s: i2c write failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static ssize_t pericom_pi3hdx1204_redriver_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	u8 data[16];
	int i;

	ret = i2c_read(this_client, data, 16);
	if (ret) {
		printk("%s: can not read tuning register\n", __func__);
		return 0;
	}

	for (i = 0; i < 16; i++) {
		printk("%s : reg = 0x%02x, value = 0x%02x\n", __func__, (u8)i, data[i]);
	}

	return sprintf(buf, "0x%02x\n", data[5]);
}

static ssize_t pericom_pi3hdx1204_redriver_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	long value;
	u8 data[16];

	value = simple_strtol(buf, NULL, 16);
	if (value > 255 || value < 0) {
		value = 0xf0;
	}

	i2c_read(this_client, data, 16);
	data[5] = (u8)(value & 0xFF);
	i2c_write(this_client, data, 16);

	return count;
}

static DEVICE_ATTR(timing, 0660, pericom_pi3hdx1204_redriver_show, pericom_pi3hdx1204_redriver_store);

struct attribute *pericom_pi3hdx1204_redriver_attributes[] = {
	&dev_attr_timing.attr,
	NULL
};

struct attribute_group pericom_pi3hdx1204_redriver_attribute_group = {
	    .attrs = pericom_pi3hdx1204_redriver_attributes
};


static int pericom_pi3hdx1204_redriver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	u8 data[16];

	printk(KERN_ERR "%s: i2c addr = 0x%02x enter!\n", __func__, client->addr);
	if (!client->dev.of_node) {
		printk(KERN_ERR "%s invalid argument\n", __func__);
		return -EINVAL;
	}

	this_client = client;
	i2c_read(this_client, data, 16);
	data[5] = 0x00;
	i2c_write(this_client, data, 16);

	ret = sysfs_create_group(&client->dev.kobj, &pericom_pi3hdx1204_redriver_attribute_group);
	if (ret) {
		printk(KERN_ERR "%s: create debug sysfs fail!\n", __func__);
	}

	return ret;
}


static int pericom_pi3hdx1204_redriver_remove(struct i2c_client *client)
{
	this_client = NULL;
	sysfs_remove_group(&client->dev.kobj, &pericom_pi3hdx1204_redriver_attribute_group);
	printk("%s: remove pericom_pi3hdx1204_redriver driver successfully", __func__);
	return 0;
}


static struct i2c_device_id pericom_pi3hdx1204_redriver_id[] = {
	{ "timing-tuning", 0 },
	{}
};

static const struct of_device_id pericom_pi3hdx1204_redriver_match_table[] = {
	{ .compatible = "pericom,pi3hdx1204-redriver" },
	{}
};

MODULE_DEVICE_TABLE(of, pericom_pi3hdx1204_redriver_match_table);

static struct i2c_driver pericom_pi3hdx1204_redriver_driver = {
	.driver			= {
		.name		= "timing-tuning",
		.owner		= THIS_MODULE,
		.of_match_table = pericom_pi3hdx1204_redriver_match_table,
	},
	.probe			= pericom_pi3hdx1204_redriver_probe,
	.remove			= pericom_pi3hdx1204_redriver_remove,
	.id_table		= pericom_pi3hdx1204_redriver_id,
};

module_i2c_driver(pericom_pi3hdx1204_redriver_driver);

MODULE_AUTHOR("liusd@thundersoft.com");
MODULE_DESCRIPTION("pericom_pi3hdx1204_redriver");
MODULE_LICENSE("GPL");
