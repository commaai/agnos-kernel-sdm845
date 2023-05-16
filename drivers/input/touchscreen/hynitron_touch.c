/*
 * Touch Screen driver for Hynitron I2C Touch panels
 *
 * Copyright (C) 2022 comma.ai
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

// Reverse engineered

#include <linux/crc-itu-t.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <asm/unaligned.h>
#include <linux/init.h>

#define HYN_I2C_NAME "hynitron_i2c_touchpanel"

/* Touchscreen parameters */
#define HYN_MAX_FINGERS			            5
#define HYN_MAX_X    			            1080
#define HYN_MAX_Y	    		            2160
#define HYN_MAX_PRESSURE   		            255

#define MAX_I2C_TRANSFER_SIZE               32

struct hyn_ts_data {
	struct i2c_client *client;
	struct input_dev *input;

	struct gpio_desc *interrupt_gpio;
	struct gpio_desc *reset_gpio;

    int touch_id_active[HYN_MAX_FINGERS];
};

/* Event structures */
struct hyn_ts_coordinate_event {
	uint8_t id;
	uint16_t x;
    uint16_t y;
    uint16_t z;
    uint8_t unknown[3];
} __attribute__ ((packed));

static int hyn_read(struct hyn_ts_data *ts, uint8_t address, uint8_t *data, uint8_t len)
{
    int ret, chunk_len, i = 0;

    // Read in chunks
    while(len > 0) {
        chunk_len = len > MAX_I2C_TRANSFER_SIZE ? MAX_I2C_TRANSFER_SIZE : len;
        ret = i2c_smbus_read_i2c_block_data(ts->client, address, chunk_len, &data[i]);
        if(ret != chunk_len){
            dev_err(&ts->client->dev, "%s: i2c_smbus_read_i2c_block_data. returned bytes: %d expected: %d\n", __func__, ret, chunk_len);
            return -1;
        }
        i += chunk_len;
        len -= chunk_len;
    }
    return 0;
}


static int hyn_report_touch(struct hyn_ts_data *ts, int touch_id, int active, int x, int y, int z)
{
    input_mt_slot(ts->input, touch_id);
    if(active) {
        input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
        input_report_abs(ts->input, ABS_MT_POSITION_X, x);
        input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
        input_report_abs(ts->input, ABS_MT_PRESSURE, z);
    } else {
        input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
    }
    input_mt_report_pointer_emulation(ts->input, true);
	input_sync(ts->input);
    return 0;
}

static int hyn_handle_event(struct hyn_ts_data *ts, uint8_t *event)
{
    int num_events, touch_id, ret, i;
    int processed_touch_ids[HYN_MAX_FINGERS] = {0};
    struct hyn_ts_coordinate_event *event_coord;

    num_events = event[0] / sizeof(struct hyn_ts_coordinate_event);
    if(num_events > HYN_MAX_FINGERS) {
        dev_err(&ts->client->dev, "%s: num_events (%d) > HYN_MAX_FINGERS (%d)\n", __func__, num_events, HYN_MAX_FINGERS);
        return -1;
    }

    for(i = 0; i < num_events; i++) {
        event_coord = (struct hyn_ts_coordinate_event *) &event[8 + i * sizeof(struct hyn_ts_coordinate_event)];
        touch_id = event_coord->id & 0xF;
        if(touch_id >= HYN_MAX_FINGERS) {
            dev_err(&ts->client->dev, "%s: touch_id (%d) >= HYN_MAX_FINGERS (%d)\n", __func__, touch_id, HYN_MAX_FINGERS);
            return -1;
        }

        ret = hyn_report_touch(ts, touch_id, 1, event_coord->x, event_coord->y, event_coord->z);
        if(ret != 0) {
            dev_err(&ts->client->dev, "%s: hyn_report_touch failed\n", __func__);
            return -1;
        }

        processed_touch_ids[touch_id] = 1;
    }

    for(i = 0; i < HYN_MAX_FINGERS; i++) {
        if(!processed_touch_ids[i] && ts->touch_id_active[i]) {
            ret = hyn_report_touch(ts, i, 0, 0, 0, 0);
            if(ret != 0) {
                dev_err(&ts->client->dev, "%s: hyn_report_touch failed\n", __func__);
                return -1;
            }
        }

        ts->touch_id_active[i] = processed_touch_ids[i];
    }

    if (ret > 0){
        ret = 0;
    }
    return ret;
}


static irqreturn_t hyn_ts_irq_handler(int irq, void *dev_id)
{
    struct hyn_ts_data *ts = dev_id;
    int ret, retry_count = 0;

    // Keep handling packets until the IRQ pin has gone high again
    uint8_t event[255];

    do {
        // Read event
        while (true) {
            ret = hyn_read(ts, 0x00, &event[0], 1);
            if(ret < 0){
                dev_err(&ts->client->dev, "%s: reading num bytes failed: %d\n", __func__, ret);
                if(retry_count < 10){
                    retry_count++;
                    msleep(10);
                    continue;
                } else {
                    dev_err(&ts->client->dev, "%s: retry count reached\n", __func__);
                    return IRQ_HANDLED;
                }
            }
            break;
        }
        ret = hyn_read(ts, 0x01, &event[1], event[0]);
        if(ret < 0){
            dev_err(&ts->client->dev, "%s: reading event data failed: %d\n", __func__, ret);
            return IRQ_HANDLED;
        }

        // Handle
        ret = hyn_handle_event(ts, event);
        if(ret < 0){
            dev_err(&ts->client->dev, "%s: handling event failed: %d\n", __func__, ret);
            return IRQ_HANDLED;
        }
    } while (ts->interrupt_gpio && !gpiod_get_value_cansleep(ts->interrupt_gpio));

	return IRQ_HANDLED;
}

static void hyn_ts_reset(struct hyn_ts_data *ts)
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

static int hyn_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct hyn_ts_data *ts;
    struct firmware *fw;
	int error, boot_status;

    dev_info(&client->dev, "HYNITRON PANEL probe\n");

    // Check that we are running with a Mate10 Lite panel, otherwise abort
    if(strstr(saved_command_line, "mate10_lite") == NULL){
        dev_err(&client->dev, "Not running on a Mate10 Lite panel, aborting\n");
        return -ENODEV;
    }

    ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->interrupt_gpio = devm_gpiod_get(&client->dev, "interrupt", GPIOD_IN);
	if (IS_ERR(ts->interrupt_gpio)) {
		error = PTR_ERR(ts->interrupt_gpio);
		if (error != -EPROBE_DEFER)
			dev_err(&client->dev, "failed to get interrupt GPIO: %d\n", error);
		return error;
	}

    ts->reset_gpio = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio)) {
		error = PTR_ERR(ts->reset_gpio);
		if (error != -EPROBE_DEFER)
			dev_err(&client->dev, "failed to get reset GPIO: %d\n", error);
		return error;
	}

	ts->input = devm_input_allocate_device(&client->dev);
	if (!ts->input) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input->name = "Hynitron Touchscreen";
	ts->input->id.bustype = BUS_I2C;

    input_set_abs_params(ts->input, ABS_MT_TRACKING_ID, 0, HYN_MAX_FINGERS, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, HYN_MAX_X, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, HYN_MAX_Y, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, HYN_MAX_PRESSURE, 0, 0);

	error = input_mt_init_slots(ts->input, HYN_MAX_FINGERS, INPUT_MT_DIRECT);
	if (error) {
		dev_err(&client->dev, "failed to initialize MT slots: %d\n", error);
		return error;
	}

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, hyn_ts_irq_handler,
					  IRQF_ONESHOT,
					  client->name, ts);
	if (error) {
		dev_err(&client->dev, "failed to request IRQ: %d\n", error);
		return error;
	}

	error = input_register_device(ts->input);
	if (error) {
		dev_err(&client->dev, "failed to register input device: %d\n", error);
		return error;
	}

    // Handle first event without interrupt
    hyn_ts_irq_handler(client->irq, ts);

	return 0;
}

static int hyn_ts_remove(struct i2c_client *client)
{
    dev_info(&client->dev, "HYNITRON PANEL remove\n");
	return 0;
}

static const struct of_device_id hyn_ts_dt_ids[] = {
	{ .compatible = HYN_I2C_NAME },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hyn_ts_dt_ids);

static const struct i2c_device_id hyn_ts_id[] = {
	{ HYN_I2C_NAME,	0 },
	{ /* sentinel */  }
};
MODULE_DEVICE_TABLE(i2c, hyn_ts_id);

static struct i2c_driver hyn_ts_driver = {
	.driver = {
		.name	= HYN_I2C_NAME,
		.of_match_table = of_match_ptr(hyn_ts_dt_ids),
	},
	.probe		= hyn_ts_probe,
    .remove     = hyn_ts_remove,
	.id_table	= hyn_ts_id,
};
module_i2c_driver(hyn_ts_driver);

MODULE_DESCRIPTION("Hynitron Touchscreen Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Robbe Derks <robbe@comma.ai>");
