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

/* Touchscreen parameters */
#define SS_MAX_FINGERS			10

/* Touchscreen registers */
#define SS_ONE_EVENT_CMD        0x60
#define SS_ONE_EVENT_SIZE       15


struct ss_ts_data {
	struct i2c_client *client;
	struct input_dev *input;

	struct gpio_desc *interrupt_gpio;
	struct gpio_desc *reset_gpio;
    struct gpio_desc *ta_gpio;
};

static int ss_read(struct ss_ts_data *ts, uint8_t command, uint8_t *data, uint8_t len)
{
    int err;
    err = i2c_smbus_read_i2c_block_data(ts->client, command, len, data);
    if(err){
        dev_err(&ts->client->dev, "%s: i2c_smbus_read_i2c_block_data err: %d\n", __func__, err);
        return err;
    }
}

static irqreturn_t ss_ts_irq_handler(int irq, void *dev_id)
{
    struct ss_ts_data *ts = dev_id;
    int err, i;

    // Keep handling packets until the IRQ pin has gone high again
    uint8_t event[SS_ONE_EVENT_SIZE];
    do {
        // Read event
        err = ss_read(ts, SS_ONE_EVENT_CMD, event, SS_ONE_EVENT_SIZE);
        if(err){
            dev_err(&ts->client->dev, "%s: Reading event failed: %d\n", __func__, err);
            return IRQ_HANDLED;
        }

        // Print
        printk("Event: ");
        for(i=0; i<SS_ONE_EVENT_SIZE; i++){
            printk("%d ", event[i]);
        }
        printk("\n");

		//ss_ts_handle_packet(ts);
	} while (ts->interrupt_gpio && !gpiod_get_value_cansleep(ts->interrupt_gpio));

	return IRQ_HANDLED;
}

static void ss_ts_reset(struct ss_ts_data *ts)
{
	if (ts->reset_gpio) {
		/* Get out of reset */
		usleep_range(1000, 2000);
		gpiod_set_value(ts->reset_gpio, 0);
		usleep_range(1000, 2000);
		gpiod_set_value(ts->reset_gpio, 1);
		msleep(100);
	}
}

static int ss_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ss_ts_data *ts;
	int error;

    printk("SAMSUNG PANEL probe\n");

    ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->interrupt_gpio = devm_gpiod_get(&client->dev, "interrupt", GPIOD_IN);
	if (IS_ERR(ts->interrupt_gpio)) {
		error = PTR_ERR(ts->interrupt_gpio);
		if (error != -EPROBE_DEFER)
			dev_err(&client->dev, "Failed to get interrupt GPIO: %d\n", error);
		return error;
	}

    ts->reset_gpio = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio)) {
		error = PTR_ERR(ts->reset_gpio);
		if (error != -EPROBE_DEFER)
			dev_err(&client->dev, "Failed to get reset GPIO: %d\n", error);
		return error;
	}

    ts->ta_gpio = devm_gpiod_get(&client->dev, "ta", GPIOD_IN);
	if (IS_ERR(ts->ta_gpio)) {
		error = PTR_ERR(ts->ta_gpio);
		if (error != -EPROBE_DEFER)
			dev_err(&client->dev, "Failed to get TA GPIO: %d\n", error);
		return error;
	}

	ss_ts_reset(ts);

	ts->input = devm_input_allocate_device(&client->dev);
	if (!ts->input) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input->name = "Samsung Touchscreen";
	ts->input->id.bustype = BUS_I2C;

    // TODO: Read params from IC
	// input_set_abs_params(input, ABS_MT_POSITION_X, 0, SIS_MAX_X, 0, 0);
	// input_set_abs_params(input, ABS_MT_POSITION_Y, 0, SIS_MAX_Y, 0, 0);
	// input_set_abs_params(input, ABS_MT_PRESSURE, 0, SIS_MAX_PRESSURE, 0, 0);
	// input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, SIS_AREA_LENGTH_LONGER, 0, 0);
	// input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, SIS_AREA_LENGTH_SHORT, 0, 0);

	error = input_mt_init_slots(ts->input, SS_MAX_FINGERS, INPUT_MT_DIRECT);
	if (error) {
		dev_err(&client->dev, "Failed to initialize MT slots: %d\n", error);
		return error;
	}

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, ss_ts_irq_handler,
					  IRQF_ONESHOT,
					  client->name, ts);
	if (error) {
		dev_err(&client->dev, "Failed to request IRQ: %d\n", error);
		return error;
	}

	error = input_register_device(ts->input);
	if (error) {
		dev_err(&client->dev,
			"Failed to register input device: %d\n", error);
		return error;
	}

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
