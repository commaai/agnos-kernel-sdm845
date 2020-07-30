/*
 * Touch Screen driver for Samsung I2C Touch panels
 *
 * Copyright (C) 2020 comma.ai
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/crc-itu-t.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#define SS_I2C_NAME "samsung_i2c_touchpanel"

static irqreturn_t ss_ts_irq_handler(int irq, void *dev_id)
{
	printk("SAMSUNG PANEL IRQ HANDLING\n");
	return IRQ_HANDLED;
}

static int ss_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    printk("SAMSUNG PANEL probe\n");
	return 0;
}

static int ss_ts_remove(struct i2c_client *client)
{
    printk("SAMSUNG PANEL remove\n");
	return 0;
}

static const struct of_device_id ss_ts_dt_ids[] = {
	{ .compatible = SS_I2C_NAME },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ss_ts_dt_ids);

static const struct i2c_device_id ss_ts_id[] = {
	{ SS_I2C_NAME,	0 },
	{ /* sentinel */  }
};
MODULE_DEVICE_TABLE(i2c, ss_ts_id);

static struct i2c_driver ss_ts_driver = {
	.driver = {
		.name	= SS_I2C_NAME,
		.of_match_table = of_match_ptr(ss_ts_dt_ids),
	},
	.probe		= ss_ts_probe,
    .remove     = ss_ts_remove,
	.id_table	= ss_ts_id,
};
module_i2c_driver(ss_ts_driver);

MODULE_DESCRIPTION("Samsung Touchscreen Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Robbe Derks <robbe@comma.ai>");