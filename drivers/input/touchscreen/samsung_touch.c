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

// Loosely based on the Huawei Mate 10 Pro kernel driver!

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
#define SS_MAX_FINGERS			            10
#define SS_MAX_HOVER			            1
#define SS_MAX_X    			            1080
#define SS_MAX_Y	    		            2160
#define SS_MAX_PRESSURE   		            255
#define SS_AREA_LENGTH_LONGER               100
#define SS_AREA_LENGTH_SHORTER	            100

/* Touchscreen registers */
#define SS_ONE_EVENT_CMD                    0x60
#define SS_ONE_EVENT_SIZE                   15
#define SS_BOOT_STATUS_CMD                  0x55
#define SS_ENTER_FW_MODE_CMD                0x57
#define SS_CHIP_ID_CMD                      0x52

/* Touchscreen events */
#define SS_EVENT_COORDINATE		            0
#define SS_EVENT_STATUS		                1
#define SS_EVENT_GESTURE		            2
#define SS_EVENT_EMPTY		                3
#define SS_EVENT_COORDINATE_WET		        6

/* Touchscreen event constants */
#define SS_EVENT_COORDINATE_ACTION_DOWN     1
#define SS_EVENT_COORDINATE_ACTION_MOVE     2
#define SS_EVENT_COORDINATE_ACTION_UP       3

/* Constants */
#define SS_STOP                             0
#define SS_CONTINUE                         1
#define SS_BOOT_STATUS_BOOT                 0x10
#define SS_BOOT_STATUS_APP                  0x20
#define SS_FIRMWARE                         "samsung_touch.img"
uint8_t expected_chip_id[3] =               {0xBA, 0xB6, 0x61};

struct ss_ts_data {
	struct i2c_client *client;
	struct input_dev *input;

	struct gpio_desc *interrupt_gpio;
	struct gpio_desc *reset_gpio;
    struct gpio_desc *ta_gpio;

    uint8_t chip_id[3];
};

/* Event structures */
struct sec_ts_event_coordinate {
	uint8_t eid:2;
	uint8_t tid:4;
	uint8_t tchsta:2;
	uint8_t x_11_4;
	uint8_t y_11_4;
	uint8_t y_3_0:4;
	uint8_t x_3_0:4;
	uint8_t major;
	uint8_t minor;
	uint8_t z:6;
	uint8_t ttype_3_2:2;
	uint8_t left_event:6;
	uint8_t ttype_1_0:2;
	uint8_t wx;
	uint8_t wy;
	uint8_t ewx;
	uint8_t ewy;
	uint8_t orient;
	uint8_t sgx;
	uint8_t sgy;
} __attribute__ ((packed));

static int ss_read(struct ss_ts_data *ts, uint8_t command, uint8_t *data, uint8_t len)
{
    int ret;
    ret = i2c_smbus_read_i2c_block_data(ts->client, command, len, data);
    if(ret != len){
        dev_err(&ts->client->dev, "%s: i2c_smbus_read_i2c_block_data. returned bytes: %d expected: %d\n", __func__, ret, len);
        return -1;
    }
    return 0;
}

static int ss_write(struct ss_ts_data *ts, uint8_t command, uint8_t *data, uint8_t len)
{
    int ret;
    ret = i2c_smbus_write_i2c_block_data(ts->client, command, len, data);
    if(ret != 0){
        dev_err(&ts->client->dev, "%s: i2c_smbus_write_i2c_block_data: %d\n", __func__, ret);
        return ret;
    }
    return 0;
}

static int ss_report_touch(struct ss_ts_data *ts, int touch_id, int action, int x, int y, int z, int type, int major, int minor, int ewx, int ewy)
{
    input_mt_slot(ts->input, touch_id);
    switch(action){
        case SS_EVENT_COORDINATE_ACTION_DOWN:
        case SS_EVENT_COORDINATE_ACTION_MOVE:
            input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
            input_report_abs(ts->input, ABS_MT_POSITION_X, x);
            input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
            input_report_abs(ts->input, ABS_MT_PRESSURE, z);
            input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, major);
            input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, minor);
            break;
        case SS_EVENT_COORDINATE_ACTION_UP:
            input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
            break;
    }
    input_mt_report_pointer_emulation(ts->input, true);
	input_sync(ts->input);
    return 0;
}

static int ss_handle_coordinate_event(struct ss_ts_data *ts, uint8_t *event)
{
    int touch_id, ret;
    struct sec_ts_event_coordinate *event_coord;

    event_coord = (struct sec_ts_event_coordinate *) event;
    touch_id = (event_coord->tid - 1);

    if(touch_id < SS_MAX_FINGERS + SS_MAX_HOVER && touch_id >= 0){
        dev_dbg(&ts->client->dev, "%s: coordinate event: ID: %d ACT: %d X: %d, Y: %d Z: %d Type: %d Major: %d Minor: %d WX: %d WY: %d EWX: %d EWY: %d\n",
            __func__,
            touch_id,
            event_coord->tchsta,
            (event_coord->x_11_4 << 4) | (event_coord->x_3_0),
            (event_coord->y_11_4 << 4) | (event_coord->y_3_0),
            (event_coord->z & 0x3F),
            (event_coord->ttype_3_2 << 2 | event_coord->ttype_1_0),
            event_coord->major,
            event_coord->minor,
            event_coord->ewx,
            event_coord->ewy
        );

        ret = ss_report_touch(
            ts,
            touch_id,
            event_coord->tchsta,
            (event_coord->x_11_4 << 4) | (event_coord->x_3_0),
            (event_coord->y_11_4 << 4) | (event_coord->y_3_0),
            (event_coord->z & 0x3F),
            (event_coord->ttype_3_2 << 2 | event_coord->ttype_1_0),
            event_coord->major,
            event_coord->minor,
            event_coord->ewx,
            event_coord->ewy
        );
        if(ret < 0){
            dev_err(&ts->client->dev, "%s: report touch failed: %d\n", __func__, ret);
        }

        // Maybe there's more???
        return SS_CONTINUE;
    }
    return SS_STOP;
}

static int ss_handle_one_event(struct ss_ts_data *ts, uint8_t *event)
{
    uint8_t event_id;
    int ret;

    event_id = event[0] & 0x3;
    switch(event_id){
        case SS_EVENT_COORDINATE:
            dev_dbg(&ts->client->dev, "%s: coordinate event\n", __func__);
            ret = ss_handle_coordinate_event(ts, event);
            if(ret < 0){
                dev_err(&ts->client->dev, "%s: coordinate event handler error: %d\n", __func__, ret);
                return SS_STOP;
            }
            return ret;
        case SS_EVENT_STATUS:
            dev_warn(&ts->client->dev, "%s: status event handler not implemented!\n", __func__);
            return SS_STOP;
        case SS_EVENT_GESTURE:
            dev_warn(&ts->client->dev, "%s: gesture event handler not implemented!\n", __func__);
            return SS_STOP;
        case SS_EVENT_EMPTY:
            dev_dbg(&ts->client->dev, "%s: empty event\n", __func__);
            return SS_STOP;
        case SS_EVENT_COORDINATE_WET:
            dev_warn(&ts->client->dev, "%s: wet coordinate event handler not implemented!\n", __func__);
            return SS_STOP;
    }
}

static irqreturn_t ss_ts_irq_handler(int irq, void *dev_id)
{
    struct ss_ts_data *ts = dev_id;
    int ret;

    // Keep handling packets until the IRQ pin has gone high again
    uint8_t event[SS_ONE_EVENT_SIZE];
    do {
        // Read event
        ret = ss_read(ts, SS_ONE_EVENT_CMD, event, SS_ONE_EVENT_SIZE);
        if(ret < 0){
            dev_err(&ts->client->dev, "%s: reading event failed: %d\n", __func__, ret);
            return IRQ_HANDLED;
        }

        // Handle
        ret = ss_handle_one_event(ts, event);
        if(ret < 0){
            dev_err(&ts->client->dev, "%s: handling event failed: %d\n", __func__, ret);
            return IRQ_HANDLED;
        }

	} while ((ts->interrupt_gpio && !gpiod_get_value_cansleep(ts->interrupt_gpio)) || (ret == SS_CONTINUE));

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

static int ss_get_boot_status(struct ss_ts_data *ts)
{
    uint8_t ret = 0;
    uint8_t result[1];
    ret = ss_read(ts, SS_BOOT_STATUS_CMD, result, 1);
    if(ret < 0){
        dev_err(&ts->client->dev, "%s: reading boot status: %d\n", __func__, ret);
        return ret;
    }

    return result[0];
}

static int ss_enter_boot_mode(struct ss_ts_data *ts)
{
    uint8_t ret = 0;
    uint8_t boot_status = 0;
    uint8_t firmware_password[2] = {0x55, 0xAC};
    boot_status = ss_get_boot_status(ts);
    if(boot_status < 0) return boot_status;

    if(boot_status != SS_BOOT_STATUS_BOOT){
        // Enter firmware mode
        ret = ss_write(ts, SS_ENTER_FW_MODE_CMD, firmware_password, sizeof(firmware_password));
        if(ret < 0){
            dev_err(&ts->client->dev, "%s: entering fw mode: %d\n", __func__, ret);
            return ret;
        }

        msleep(200);

        // Check that it worked
        boot_status = ss_get_boot_status(ts);
        if(boot_status != SS_BOOT_STATUS_BOOT) {
            dev_err(&ts->client->dev, "%s: didn't enter fw mode. Boot status: %d\n", __func__, boot_status);
            return -1;
        }
    }

    return 0;
}

static int ss_read_chip_id(struct ss_ts_data *ts)
{
    uint8_t ret = 0;
    ret = ss_read(ts, SS_CHIP_ID_CMD, ts->chip_id, 3);
    if(ret < 0){
        dev_err(&ts->client->dev, "%s: reading chip id: %d\n", __func__, ret);
        return ret;
    }

    return 0;
}

static int ss_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ss_ts_data *ts;
    struct firmware *firmware;
	int error, boot_status;

    dev_info(&client->dev, "SAMSUNG PANEL probe\n");

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

    ts->ta_gpio = devm_gpiod_get(&client->dev, "ta", GPIOD_IN);
	if (IS_ERR(ts->ta_gpio)) {
		error = PTR_ERR(ts->ta_gpio);
		if (error != -EPROBE_DEFER)
			dev_err(&client->dev, "failed to get TA GPIO: %d\n", error);
		return error;
	}

    // Reset panel
	ss_ts_reset(ts);

    // Check that the panel is running in app mode, otherwise flash
    boot_status = ss_get_boot_status(ts);
    if (boot_status != SS_BOOT_STATUS_APP) {
        dev_info(&client->dev, "Device not in app mode (current mode:). Attempting reflash\n");

        // Enter boot mode
        error = ss_enter_boot_mode(ts);
        if(error < 0) {
            return error;
        }

        // Check that the chip ID matches
        error = ss_read_chip_id(ts);
        if(error < 0) {
            return error;
        } else if(memcmp(ts->chip_id, expected_chip_id, sizeof(expected_chip_id))){
            dev_err(&client->dev, "Chip ID does not match expected: 0x%x%x%x\n", ts->chip_id[0], ts->chip_id[1], ts->chip_id[2]);
            return -1;
        }

        // Load firmware
        error = request_firmware(firmware, SS_FIRMWARE, &client->dev);
        if(error < 0){
            dev_err(&client->dev, "firmware request failed: %d\n", error);
            return error;
        }


    }

	ts->input = devm_input_allocate_device(&client->dev);
	if (!ts->input) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input->name = "Samsung Touchscreen";
	ts->input->id.bustype = BUS_I2C;

    input_set_abs_params(ts->input, ABS_MT_TRACKING_ID, 0, SS_MAX_FINGERS + SS_MAX_HOVER, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, SS_MAX_X, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, SS_MAX_Y, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, SS_MAX_PRESSURE, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, SS_AREA_LENGTH_LONGER, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MINOR, 0, SS_AREA_LENGTH_SHORTER, 0, 0);

	error = input_mt_init_slots(ts->input, SS_MAX_FINGERS, INPUT_MT_DIRECT);
	if (error) {
		dev_err(&client->dev, "failed to initialize MT slots: %d\n", error);
		return error;
	}

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, ss_ts_irq_handler,
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

	return 0;
}

static int ss_ts_remove(struct i2c_client *client)
{
    dev_info(&client->dev, "SAMSUNG PANEL remove\n");
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
