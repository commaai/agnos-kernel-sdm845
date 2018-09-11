/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/irqreturn.h>
#include <linux/kd.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/errno.h>
#include <linux/serio.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <asm/irq.h>

#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/regmap.h>

#define MIPI_1080P

//#define LT8912_ON_WORK

/**
 * both dsi0-2-hdmi and dsi1-2-hdmi share
 * the same reset and pwren gpios
 */
struct lt8912_public {
	int reset_gpio;
	int pwren_gpio;
	bool is_reset;
	bool is_poweron;
	bool is_reset_gpio_requested;
	bool is_pwren_gpio_requested;
};

/**
 * struct lt8912_private - Cached chip configuration data
 * @client: I2C client
 * @dev: device structure
 */
struct lt8912_private {
	struct i2c_client *lt8912_client;
	struct regmap *regmap;
	int hpd_gpio;
	int hpd_irq;
	struct delayed_work on_work;
	int index;
	bool is_initialized;
	struct lt8912_public *public;
};

static struct lt8912_private *g_data[2] = {NULL, NULL};
static struct lt8912_public g_lt_public = {-1, -1, false, false, false, false};

void lt8912_cont_splash_enabled(void)
{
	int index = 0;
	for (; index < 2; index++) {
		struct lt8912_private *data = g_data[index];
		if (data) {
			data->is_initialized = true;
		}
	}
}

static int lt8912_i2c_write_byte(struct lt8912_private *data,
					unsigned int reg, unsigned int val)
{
	int rc = 0;

	rc = regmap_write(data->regmap, reg, val);
	if (rc) {
		dev_err(&data->lt8912_client->dev,
				"write 0x%02x register failed\n", reg);
		return rc;
	}

	return 0;
}

static const struct of_device_id of_rk_lt8912_match[] = {
	{ .compatible = "lontium,lt8912"},
	{  },
};

static const struct i2c_device_id lt8912_id[] = {
	{"lt8912", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, lt8912_id);

static int digital_clock_enable(struct lt8912_private *data)
{
	int rc = 0;

	data->lt8912_client->addr = 0x48;
	rc = lt8912_i2c_write_byte(data, 0x08, 0xff);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x09, 0xff);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x0a, 0xff);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x0b, 0xff);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x0c, 0xff);
	if (rc)
		return rc;

	return 0;
}

static int tx_analog(struct lt8912_private *data)
{
	int rc = 0;

	data->lt8912_client->addr = 0x48;
	rc = lt8912_i2c_write_byte(data, 0x31, 0xa1);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x32, 0xb9);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x33, 0x17);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x37, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x38, 0x22);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x60, 0x82);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3a, 0x00);
	if (rc)
		return rc;

	return 0;
}

static int cbus_analog(struct lt8912_private *data)
{
	int rc = 0;

	data->lt8912_client->addr = 0x48;
	rc = lt8912_i2c_write_byte(data, 0x39, 0x45);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3b, 0x00);
	if (rc)
		return rc;

	return 0;
}

static int hdmi_pll_analog(struct lt8912_private *data)
{
	int rc = 0;

	data->lt8912_client->addr = 0x48;
	rc = lt8912_i2c_write_byte(data, 0x44, 0x31);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x55, 0x44);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x57, 0x01);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x5a, 0x02);
	if (rc)
		return rc;
	return 0;
}

static int mipi_rx_logic_res(struct lt8912_private *data)
{
	int rc = 0;

	data->lt8912_client->addr = 0x48;
	rc = lt8912_i2c_write_byte(data, 0x03, 0x7f);
	if (rc)
		return rc;

	mdelay(100);

	rc = lt8912_i2c_write_byte(data, 0x03, 0xff);
	if (rc)
		return rc;

	return 0;
}

static int mipi_basic_set(struct lt8912_private *data)
{
	int rc = 0;

	data->lt8912_client->addr = 0x49;

	/* term en  To analog phy for trans lp mode to hs mode */
	rc = lt8912_i2c_write_byte(data, 0x10, 0x20);
	if (rc)
		return rc;

	/* settle Set timing for dphy trans state from PRPR to SOT state */
	rc = lt8912_i2c_write_byte(data, 0x11, 0x04);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x12, 0x04);
	if (rc)
		return rc;

	/* 4 lane, 01 lane, 02 2 lane, 03 3lane */
	rc = lt8912_i2c_write_byte(data, 0x13, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x14, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x15, 0x00);
	if (rc)
		return rc;

	/* hshift 3 */
	rc = lt8912_i2c_write_byte(data, 0x1a, 0x03);
	if (rc)
		return rc;

	/* vshift 3 */
	rc = lt8912_i2c_write_byte(data, 0x1b, 0x03);
	if (rc)
		return rc;
	return 0;
}

#ifdef MIPI_1080P
static int mipi_dig_1080p(struct lt8912_private *data)
{
	int rc = 0;

	data->lt8912_client->addr = 0x49;
	rc = lt8912_i2c_write_byte(data, 0x18, 0x2c);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x19, 0x05);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x1c, 0x80);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x1d, 0x07);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x2f, 0x0c);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x34, 0x98);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x35, 0x08);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x36, 0x65);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x37, 0x04);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x38, 0x24);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x39, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3a, 0x04);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3b, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3c, 0x94);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3d, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3e, 0x58);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3f, 0x00);
	if (rc)
		return rc;
	return 0;
}
#endif

#ifdef MIPI_720P
static int mipi_dig_720p(struct lt8912_private *data)
{
	int rc = 0;

	data->lt8912_client->addr = 0x49;
	rc = lt8912_i2c_write_byte(data, 0x18, 0x28);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x19, 0x05);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x1c, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x1d, 0x05);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x1e, 0x67);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x2f, 0x0c);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x34, 0x72);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x35, 0x06);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x36, 0xee);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x37, 0x02);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x38, 0x14);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x39, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3a, 0x05);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3b, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3c, 0xdc);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3d, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3e, 0x6e);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3f, 0x00);
	if (rc)
		return rc;
	return 0;
}
#endif

#ifdef MIPI_480P
static int mipi_dig_480p(struct lt8912_private *data)
{
	int rc = 0;

	data->lt8912_client->addr = 0x49;
	rc = lt8912_i2c_write_byte(data, 0x18, 0x60);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x19, 0x02);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x1c, 0x80);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x1d, 0x02);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x1e, 0x67);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x2f, 0x0c);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x34, 0x20);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x35, 0x03);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x36, 0x0d);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x37, 0x02);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x38, 0x20);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x39, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3a, 0x0a);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3b, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3c, 0x30);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3d, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3e, 0x10);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x3f, 0x00);
	if (rc)
		return rc;
	return 0;
}
#endif

static int dds_config(struct lt8912_private *data)
{
	int rc = 0;

	data->lt8912_client->addr = 0x49;

	/* strm_sw_freq_word[ 7: 0] */
	rc = lt8912_i2c_write_byte(data, 0x4e, 0x6A);
	if (rc)
		return rc;

	/* strm_sw_freq_word[15: 8] */
	rc = lt8912_i2c_write_byte(data, 0x4f, 0x4D);
	if (rc)
		return rc;

	/* strm_sw_freq_word[23:16] */
	rc = lt8912_i2c_write_byte(data, 0x50, 0xF3);
	if (rc)
		return rc;

	/* [0]=strm_sw_freq_word[24]//[7]=strm_sw_freq_word_en=0,
	[6]=strm_err_clr=0 */
	rc = lt8912_i2c_write_byte(data, 0x51, 0x80);
	if (rc)
		return rc;

	/* full_value  464 */
	rc = lt8912_i2c_write_byte(data, 0x1f, 0x90);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x20, 0x01);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x21, 0x68);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x22, 0x01);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x23, 0x5E);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x24, 0x01);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x25, 0x54);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x26, 0x01);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x27, 0x90);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x28, 0x01);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x29, 0x68);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x2a, 0x01);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x2b, 0x5E);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x2c, 0x01);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x2d, 0x54);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x2e, 0x01);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x42, 0x64);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x43, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x44, 0x04);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x45, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x46, 0x59);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x47, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x48, 0xf2);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x49, 0x06);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x4a, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x4b, 0x72);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x4c, 0x45);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x4d, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x52, 0x08);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x53, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x54, 0xb2);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x55, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x56, 0xe4);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x57, 0x0d);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x58, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x59, 0xe4);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x5a, 0x8a);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x5b, 0x00);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x5c, 0x34);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x1e, 0x4f);
	if (rc)
		return rc;
	rc = lt8912_i2c_write_byte(data, 0x51, 0x00);
	if (rc)
		return rc;
	return 0;
}

static int lt8912_parse_dt(struct device *dev, struct lt8912_private *data)
{
	struct device_node *np = dev->of_node;

	if (data->public) {
		if (data->public->pwren_gpio < 0) {
			data->public->pwren_gpio = of_get_named_gpio(np, "lt,pwren-gpio", 0);
		}

		if (data->public->reset_gpio < 0) {
			data->public->reset_gpio = of_get_named_gpio(np, "lt,reset-gpio", 0);
		}
	}

    data->hpd_gpio = of_get_named_gpio(np, "lt,hpd-gpio", 0);
	if (data->hpd_gpio < 0) {
		dev_info(&data->lt8912_client->dev, "lt,hpd-gpio not defined.\n");
	}

	if (of_property_read_u32(np, "lt,index", &data->index)) {
		dev_info(&data->lt8912_client->dev, "lt,index not defined, use default value 0.\n");
		data->index = 0;
	}

	if (data->index > 1) {
		dev_info(&data->lt8912_client->dev, "lt,index(%d) is invalid, use max value 1.\n", data->index);
		data->index = 1;
	}

	return 0;
}

static struct regmap_config lt8912_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static irqreturn_t hpd_isr(int irq, void *data)
{
	struct lt8912_private *pdata = data;
	if (gpio_is_valid(pdata->hpd_gpio)) {
		int hpd = gpio_get_value(pdata->hpd_gpio);
		/* Fixme: do something while hdmi plugin */
		dev_info(&pdata->lt8912_client->dev, "hpd = %d\n", hpd);
	}
	return IRQ_HANDLED;
}

void lt8912_reset(struct lt8912_private *data)
{
	if (!data)
		return;

	if (data->public &&
		gpio_is_valid(data->public->reset_gpio) &&
		!data->public->is_reset)
	{
		data->public->is_reset = true;

		gpio_direction_output(data->public->reset_gpio, 1);
		mdelay(100);
		gpio_direction_output(data->public->reset_gpio, 0);
		mdelay(100);
		gpio_direction_output(data->public->reset_gpio, 1);
		mdelay(100);
	}
}

static void lt8912_power_on(struct lt8912_private *data)
{
	if (!data)
		return;

	if (data->public &&
		gpio_is_valid(data->public->pwren_gpio) &&
		!data->public->is_poweron)
	{
		data->public->is_poweron = true;
		gpio_direction_output(data->public->pwren_gpio, 1);
		mdelay(100);
	}
}

static void lt8912_power_off(struct lt8912_private *data)
{
	if (!data)
		return;

	if (data->public) {
		if (gpio_is_valid(data->public->reset_gpio))
			gpio_direction_output(data->public->reset_gpio, 0);

		if (gpio_is_valid(data->public->pwren_gpio))
			gpio_direction_output(data->public->pwren_gpio, 0);
	}
}

static void lt8912_init(struct lt8912_private *data)
{
	if (!data)
		return;

	pr_info("%s: init lt8912[%d] +\n", __func__, data->index);
	lt8912_power_on(data);
	lt8912_reset(data);

	if (digital_clock_enable(data))
		return;

	if (tx_analog(data))
		return;

	if (cbus_analog(data))
		return;

	if (hdmi_pll_analog(data))
		return;

	if (mipi_basic_set(data))
		return;

#ifdef MIPI_1080P
	if (mipi_dig_1080p(data))
		return;
#elif defined MIPI_720P
	if (mipi_dig_720p(data))
		return;
#elif defined MIPI_480P
	if (mipi_dig_480p(data))
		return;
#endif

	if (dds_config(data))
		return;

	if (mipi_rx_logic_res(data))
		return;

	pr_info("%s: init lt8912[%d] -\n", __func__, data->index);
}

int lt8912_on(int index)
{
	struct lt8912_private *data = g_data[index];
	if (!data) {
		pr_err("%s: g_data[%d] = NULL\n", __func__, index);
		return -1;
	}

	if (data->index != index) {
		pr_err("%s: invalid index: %d\n", __func__, index);
		return -1;
	}

	if (data->is_initialized) {
		pr_info("%s: lt8912[%d] is initialized, nothing to do !\n",
				__func__, index);
		return 0;
	}

#ifdef LT8912_ON_WORK
	schedule_delayed_work(&data->on_work, msecs_to_jiffies(0));
#else
	lt8912_init(data);
#endif
	return 0;
}

int lt8912_off(int index)
{
	struct lt8912_private *data = g_data[index];
	if (!data)
		return -1;

	if (data->index != index)
		return -1;

	lt8912_power_off(data);

	if (data->is_initialized)
		data->is_initialized = false;

	if (data->public && data->public->is_reset)
		data->public->is_reset = false;

	if (data->public && data->public->is_poweron)
		data->public->is_poweron = false;

	return 0;
}
#ifdef LT8912_ON_WORK
static void lt8912_on_work_fn(struct work_struct *work)
{
	struct lt8912_private *data = container_of((struct delayed_work *)work,
					struct lt8912_private, on_work);
	lt8912_init(data);
}
#endif
static int lt8912_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct lt8912_private *data;
	int ret;

	dev_dbg(&client->dev, "probing lt8912\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "lt8912 i2c check failed.\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct lt8912_private),
						GFP_KERNEL);

	data->lt8912_client = client;
	data->public = &g_lt_public;

	if (client->dev.of_node) {
		ret = lt8912_parse_dt(&client->dev, data);
		if (ret) {
			dev_err(&client->dev,
				"unable to parse device tree.(%d)\n", ret);
			goto out;
		}
	} else {
		dev_err(&client->dev, "device tree not found.\n");
		ret = -ENODEV;
		goto out;
	}

	dev_set_drvdata(&client->dev, data);

	if (gpio_is_valid(data->public->pwren_gpio) &&
			!data->public->is_pwren_gpio_requested) {
		ret = gpio_request(data->public->pwren_gpio, "lt8912_pwren_gpio");
		if (ret) {
			dev_err(&client->dev, "pwren_gpio request failed");
			goto out;
		}

		data->public->is_pwren_gpio_requested = true;
    }

	if (gpio_is_valid(data->hpd_gpio)) {
		data->hpd_irq = gpio_to_irq(data->hpd_gpio);
		if (data->hpd_irq < 0) {
			dev_err(&client->dev, "failed to get gpio irq\n");
			goto free_pwren_gpio;
		}

		ret = request_threaded_irq(data->hpd_irq, NULL, hpd_isr,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING
			| IRQF_ONESHOT, "lt8912-hpd-isr", data);
		if (ret < 0) {
			dev_err(&client->dev, "failed to request irq\n");
			data->hpd_irq = -1;
			goto free_pwren_gpio;
		}
    }

	if (gpio_is_valid(data->public->reset_gpio) &&
			!data->public->is_reset_gpio_requested) {
		ret = gpio_request(data->public->reset_gpio, "lt8912_reset_gpio");
		if (ret) {
			dev_err(&client->dev, "reset gpio request failed");
			goto out;
		}

		data->public->is_reset_gpio_requested = true;
	}

	data->regmap = devm_regmap_init_i2c(client, &lt8912_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "init regmap failed.(%ld)\n",
				PTR_ERR(data->regmap));
		ret = PTR_ERR(data->regmap);
		goto free_reset_gpio;
	}
#ifdef LT8912_ON_WORK
	INIT_DELAYED_WORK(&data->on_work, lt8912_on_work_fn);
#endif
	data->is_initialized = false;
	g_data[data->index] = data;
	return 0;

free_pwren_gpio:
	if (gpio_is_valid(data->public->pwren_gpio)) {
		gpio_free(data->public->pwren_gpio);
		data->public->pwren_gpio = -1;
		data->public->is_pwren_gpio_requested = false;
	}
free_reset_gpio:
	if (gpio_is_valid(data->public->reset_gpio)) {
		gpio_free(data->public->reset_gpio);
		data->public->reset_gpio = -1;
		data->public->is_reset_gpio_requested = false;
	}
out:
	return ret;
}

static int lt8912_i2c_remove(struct i2c_client *client)
{
	struct lt8912_private *data = dev_get_drvdata(&client->dev);

	if (!data)
		return -1;

	if (data->hpd_irq >= 0)
		free_irq(data->hpd_irq, data);

	if (data->public &&
		gpio_is_valid(data->public->reset_gpio))
	{
		gpio_free(data->public->reset_gpio);
		data->public->reset_gpio = -1;
		data->public->is_reset_gpio_requested = false;
	}

	if (data->public &&
		gpio_is_valid(data->public->pwren_gpio))
	{
		gpio_free(data->public->pwren_gpio);
		data->public->pwren_gpio = -1;
		data->public->is_pwren_gpio_requested = false;
	}

    if (gpio_is_valid(data->hpd_gpio))
		gpio_free(data->hpd_gpio);

	return 0;
}

static struct i2c_driver lt8912_i2c_driver = {
	.driver = {
		.name = "lt8912",
		.owner = THIS_MODULE,
		.of_match_table = of_rk_lt8912_match,
	},
	.probe = lt8912_i2c_probe,
	.remove = lt8912_i2c_remove,
	.id_table = lt8912_id,
};

static int __init lontium_i2c_test_init(void)
{
	return i2c_add_driver(&lt8912_i2c_driver);
}

static void __exit lontium_i2c_test_exit(void)
{
	i2c_del_driver(&lt8912_i2c_driver);
}

module_init(lontium_i2c_test_init);
module_exit(lontium_i2c_test_exit);

MODULE_DESCRIPTION("LT8912 Driver");
