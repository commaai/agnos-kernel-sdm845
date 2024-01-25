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
#include <linux/firmware.h>
#include <asm/unaligned.h>
#include <linux/regulator/consumer.h>
#include <linux/init.h>

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
#define SS_FLASH_ERASE_CMD                  0xD8
#define SS_FLASH_WRITE_CMD                  0xD9
#define SS_PROTOCOL_ID_CMD                  0xDD
#define SS_SW_RESET_APP_CMD                 0x12
#define SS_SW_RESET_BOOT_CMD                0x42
#define SS_ENABLE_SENSING_CMD               0x10
#define SS_NVM_CMD                          0x85

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

/* Touchscreen ACKs */
#define SS_ACK_BOOT_COMPLETE                0x00

/* Constants */
#define SS_STOP                             0
#define SS_CONTINUE                         1
#define SS_BOOT_STATUS_BOOT                 0x10
#define SS_BOOT_STATUS_APP                  0x20
#define SS_FIRMWARE                         "samsung_touch.img"
#define SS_HEADER_SIGNATURE                 0x53494654
#define SS_CHUNK_SIGNATURE                  0x53434654
#define SS_PAGE_SIZE                        256
#define SS_PROTOCOL_APP                     0x1
#define SS_COLOR_CAL_OFFSET                 0x20
#define SS_COLOR_CAL_SIZE                   ((15*2) + 1) // 15 float16s + 1 checksum byte
#define SS_COLOR_CAL_MAGIC                  0xC0
uint8_t expected_chip_id[3] =               {0xBA, 0xB6, 0x61};

struct ss_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
    struct regulator *vdd;

	struct gpio_desc *interrupt_gpio;
	struct gpio_desc *reset_gpio;
    struct gpio_desc *ta_gpio;

    uint64_t touch_count;

    uint8_t chip_id[3];
};

/* Firmware header structures */
typedef struct {
	uint32_t signature;
	uint32_t version;
	uint32_t totalsize;
	uint32_t checksum;
	uint32_t img_ver;
	uint32_t img_date;
	uint32_t img_description;
	uint32_t fw_ver;
	uint32_t fw_date;
	uint32_t fw_description;
	uint32_t para_ver;
	uint32_t para_date;
	uint32_t para_description;
	uint32_t num_chunk;
	uint32_t reserved1;
	uint32_t reserved2;
} ss_ts_fw_header;

typedef struct {
	uint32_t signature;
	uint32_t addr;
	uint32_t size;
	uint32_t reserved;
} ss_ts_fw_chunk;

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

// This doesn't have the 32 byte limit
static int ss_write_bulk(struct ss_ts_data *ts, uint8_t command, uint8_t *data, uint16_t len)
{
    int ret;
    uint8_t buffer[len + 1];
    buffer[0] = command;
    memcpy(&buffer[1], data, len);

    ret = i2c_master_send(ts->client, buffer, len + 1);
    if(ret != (len + 1)){
        dev_err(&ts->client->dev, "%s: i2c_master_send: %d\n", __func__, ret);
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
            ts->touch_count++;
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
    int ret, retry_count = 0;

    // Keep handling packets until the IRQ pin has gone high again
    uint8_t event[SS_ONE_EVENT_SIZE];
    do {
        // Read event
        ret = ss_read(ts, SS_ONE_EVENT_CMD, event, SS_ONE_EVENT_SIZE);
        if(ret < 0){
            dev_err(&ts->client->dev, "%s: reading event failed: %d\n", __func__, ret);
            if(retry_count < 10){
                retry_count++;
                msleep(10);
                continue;
            } else {
                dev_err(&ts->client->dev, "%s: retry count reached\n", __func__);
                return IRQ_HANDLED;
            }
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
    int ret = 0;
    uint8_t result[1];
    ret = ss_read(ts, SS_BOOT_STATUS_CMD, result, 1);
    if(ret < 0){
        dev_err(&ts->client->dev, "%s: reading boot status: %d\n", __func__, ret);
        return ret;
    }

    return result[0];
}

static int ss_get_protocol_id(struct ss_ts_data *ts)
{
    int ret = 0;
    uint8_t result[1];
    ret = ss_read(ts, SS_PROTOCOL_ID_CMD, result, 1);
    if(ret < 0){
        dev_err(&ts->client->dev, "%s: reading protocol id: %d\n", __func__, ret);
        return ret;
    }

    return result[0];
}

static int ss_enter_boot_mode(struct ss_ts_data *ts)
{
    int ret = 0;
    int boot_status = 0;
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

static int ss_wait_for_ack(struct ss_ts_data *ts, uint8_t ack)
{
    int ret = 0;
    uint8_t event[8];
    uint32_t i;

    dev_info(&ts->client->dev, "%s: waiting for ack %d\n", __func__, ack);
    for(i = 0; i < 100; i++){
        // Read event
        ret = ss_read(ts, SS_ONE_EVENT_CMD, event, 8);
        if(ret < 0){
            dev_err(&ts->client->dev, "%s: reading event failed: %d\n", __func__, ret);
            return ret;
        }

        // Check if it's right
        if(((event[0] >> 2) & 0xF) == ack){
            dev_info(&ts->client->dev, "%s: ack received\n", __func__);
            return 0;
        }

        msleep(20);
    }
    dev_err(&ts->client->dev, "%s: timed out\n", __func__);
    return -ETIMEDOUT;
}

static int ss_exit_boot_mode(struct ss_ts_data *ts)
{
    int ret = 0;
    int boot_status = 0, protocol_id;
    uint8_t firmware_password[2] = {0x55, 0xAC};
    boot_status = ss_get_boot_status(ts);
    if(boot_status < 0) return boot_status;

    if(boot_status != SS_BOOT_STATUS_APP){
        // Software reset
        protocol_id = ss_get_protocol_id(ts);
        if(IS_ERR(protocol_id)){
            dev_err(&ts->client->dev, "%s: couldn't read protocol ID: %d\n", __func__, protocol_id);
            return protocol_id;
        }

        ret = ss_write(ts, (protocol_id == SS_PROTOCOL_APP) ? SS_SW_RESET_APP_CMD : SS_SW_RESET_BOOT_CMD, NULL, 0);
        if(ret < 0){
            dev_err(&ts->client->dev, "%s: couldn't perform SW reset: %d\n", __func__, ret);
            return ret;
        }

        msleep(100);


        // Wait for boot acknowledge
        ret = ss_wait_for_ack(ts, SS_ACK_BOOT_COMPLETE);
        if(ret != 0){
            dev_err(&ts->client->dev, "%s: SW reset failed: %d\n", __func__, ret);
            return ret;
        }

        // Check that it worked
        boot_status = ss_get_boot_status(ts);
        if(boot_status != SS_BOOT_STATUS_APP) {
            dev_err(&ts->client->dev, "%s: didn't exit fw mode. Boot status: %d\n", __func__, boot_status);
            return -1;
        }
    }

    return 0;
}

static int ss_read_chip_id(struct ss_ts_data *ts)
{
    int ret = 0;
    ret = ss_read(ts, SS_CHIP_ID_CMD, ts->chip_id, 3);
    if(ret < 0){
        dev_err(&ts->client->dev, "%s: reading chip id: %d\n", __func__, ret);
        return ret;
    }

    return 0;
}

static int ss_power_init(struct ss_ts_data *ts)
{
    int ret = 0;
    ts->vdd = devm_regulator_get(&ts->client->dev, "vdd");
    if(IS_ERR(ts->vdd)){
        ret = PTR_ERR(ts->vdd);
        if(ret == -EPROBE_DEFER){
            dev_info(&ts->client->dev, "%s: regulator vdd not ready. Deferring probe...\n", __func__);
        } else {
            dev_err(&ts->client->dev, "%s: getting regulator vdd: %d\n", __func__, ret);
        }
        goto err;
    }

    if (of_property_read_bool(&ts->client->dev.of_node, "invert-vdd")) {
        ret = regulator_disable(ts->vdd);
    } else {
        ret = regulator_enable(ts->vdd);
    }

    if(ret){
        dev_err(&ts->client->dev, "%s: enabling regulator vdd: %d\n", __func__, ret);
    }
err:
    return ret;
}

static int ss_erase_pages(struct ss_ts_data *ts, uint32_t start_page, uint32_t len)
{
    uint8_t data[4];
    data[0] = (uint8_t)((start_page >> 8) & 0xFF);
    data[1] = (uint8_t)(start_page & 0xFF);
    data[2] = (uint8_t)((len >> 8) & 0xFF);
    data[3] = (uint8_t)(len & 0xFF);

    dev_info(&ts->client->dev, "%s: erasing pages: 0x%x - 0x%x\n", __func__, start_page, start_page + len);
    return ss_write(ts, SS_FLASH_ERASE_CMD, data, sizeof(data));
}

static uint8_t ss_calculate_checksum(uint8_t *data, uint32_t len){
    uint8_t result = 0;
    uint32_t i;
    for(i = 0; i<len; i++){
        result += data[i];
    }
    return result;
}

static int ss_flash_memory(struct ss_ts_data *ts, uint32_t addr, uint8_t *data, uint32_t len)
{
    int ret = 0;
    uint32_t page_index, data_len, i;
    uint8_t buf[2 + SS_PAGE_SIZE + 1];

    uint32_t num_pages = (len / SS_PAGE_SIZE) + ((len % SS_PAGE_SIZE) != 0);
    uint32_t start_page = (addr / SS_PAGE_SIZE);
    if ((start_page * SS_PAGE_SIZE) != addr) {
        dev_err(&ts->client->dev, "%s: non-aligned flashing is not supported!\n", __func__);
        return -EINVAL;
    }

    ret = ss_erase_pages(ts, start_page, num_pages);
    if(IS_ERR(ret)){
        dev_err(&ts->client->dev, "%s: failed to erase pages: %d\n", __func__, ret);
        goto err;
    }
    msleep(100);

    dev_info(&ts->client->dev, "%s: flashing pages: 0x%x - 0x%x\n", __func__, start_page, start_page + num_pages);
    for(i = 0; i<num_pages; i++){
        page_index = start_page + i;

        // Set page index
        buf[0] = (uint8_t)((page_index >> 8) & 0xFF);
        buf[1] = (uint8_t)(page_index & 0xFF);

        // Append data (zero padded)
        memset(&buf[2], 0, SS_PAGE_SIZE);
        data_len = min(SS_PAGE_SIZE, (len - (i * SS_PAGE_SIZE)));
        memcpy(&buf[2], (data + (i * SS_PAGE_SIZE)), data_len);

        // Append checksum
        buf[SS_PAGE_SIZE + 2] = ss_calculate_checksum(buf, (SS_PAGE_SIZE + 2));

        // Write the page
        ret = ss_write_bulk(ts, SS_FLASH_WRITE_CMD, buf, sizeof(buf));
        if(IS_ERR(ret)){
            dev_err(&ts->client->dev, "%s: failed to write page %d: %d\n", __func__, page_index, ret);
            goto err;
        }
        msleep(5);
    }
    dev_info(&ts->client->dev, "%s: flashed pages: 0x%x - 0x%x\n", __func__, start_page, start_page + num_pages);

err:
    return ret;
}

static int ss_flash_firmware(struct ss_ts_data *ts, struct firmware *fw)
{
    int ret = 0;
    uint32_t i;
    uint8_t *data_ptr = (uint8_t *) fw->data;
    ss_ts_fw_header *header;
    ss_ts_fw_chunk *chunk_header;

    // Parse header
    header = (ss_ts_fw_header *) data_ptr;
    data_ptr += sizeof(ss_ts_fw_header);
    if(header->signature != SS_HEADER_SIGNATURE){
        dev_err(&ts->client->dev, "%s: header signature mismatch\n", __func__);
        return -ENODATA;
    }

    for (i = 0; i<header->num_chunk; i++){
        // Parse chunk
        chunk_header = (ss_ts_fw_chunk *) data_ptr;
        data_ptr += sizeof(ss_ts_fw_chunk);
        if(chunk_header->signature != SS_CHUNK_SIGNATURE){
            dev_err(&ts->client->dev, "%s: chunk %d signature mismatch\n", __func__, i);
            return -ENODATA;
        }

        // Flash chunk
        ret = ss_flash_memory(ts, chunk_header->addr, data_ptr, chunk_header->size);
        if(IS_ERR(ret)){
            return ret;
        }
        data_ptr += chunk_header->size;

        // TODO: check that flashed pages match
    }
}

static int ss_enable_sensing(struct ss_ts_data *ts)
{
    int ret = 0;
    ret = ss_write(ts, SS_ENABLE_SENSING_CMD, NULL, 0);
    if(ret < 0){
        dev_err(&ts->client->dev, "%s: enabling sensing failed: %d\n", __func__, ret);
        return ret;
    }
    dev_info(&ts->client->dev, "%s: enabled sensing\n", __func__);

    return 0;
}

static char ss_color_cal_calculate_checksum(char *dat)
{
    int i = 0;
    char res = SS_COLOR_CAL_MAGIC;
    for(i = 0; i < SS_COLOR_CAL_SIZE - 1; i++){
        res += dat[i];
    }
    return res;
}

static ssize_t ss_color_cal_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    struct ss_ts_data *ts = dev_get_drvdata(dev);
    char data[SS_COLOR_CAL_SIZE];
    char checksum;

    data[0] = SS_COLOR_CAL_OFFSET;
    data[1] = SS_COLOR_CAL_SIZE - 1;
    ret = ss_write(ts, SS_NVM_CMD, data, 2);
    if(ret < 0){
        dev_err(&ts->client->dev, "%s: writing NVM command failed: %d\n", __func__, ret);
        return ret;
    }

    msleep(50);

    ret = ss_read(ts, SS_NVM_CMD, data, SS_COLOR_CAL_SIZE);
    if(ret < 0){
        dev_err(&ts->client->dev, "%s: reading NVM command failed: %d\n", __func__, ret);
        return ret;
    }

    // Make sure checksum is correct
    checksum = ss_color_cal_calculate_checksum(data);
    if(checksum != data[SS_COLOR_CAL_SIZE - 1]){
        dev_err(&ts->client->dev, "%s: checksum 0x%x did not match expected 0x%x\n", __func__, data[SS_COLOR_CAL_SIZE - 1], checksum);
        return -ENODATA;
    }

    memcpy(buf, data, SS_COLOR_CAL_SIZE);
    return SS_COLOR_CAL_SIZE;
}

static ssize_t ss_color_cal_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int ret = 0, i, write_size, block_size;
    struct ss_ts_data *ts = dev_get_drvdata(dev);
    char data[SS_COLOR_CAL_SIZE + 2];

    if(size != SS_COLOR_CAL_SIZE){
        dev_err(&ts->client->dev, "%s: expected data of length %d, got length %d\n", __func__, SS_COLOR_CAL_SIZE, size);
        return -EINVAL;
    }

    if(ss_color_cal_calculate_checksum(buf) != buf[SS_COLOR_CAL_SIZE - 1]){
        dev_err(&ts->client->dev, "%s: wrong checksum value\n", __func__);
        return -EINVAL;
    }

    block_size = I2C_SMBUS_BLOCK_MAX - 2;
    for(i=0; i<=((SS_COLOR_CAL_SIZE - 1) / block_size); i++) {
        write_size = min(SS_COLOR_CAL_SIZE - (i * block_size), block_size);
        data[0] = SS_COLOR_CAL_OFFSET + (i * block_size);
        data[1] = write_size - 1;
        memcpy(&data[2], buf + (i * block_size), write_size);
        ret = ss_write(ts, SS_NVM_CMD, data, write_size + 2);
        if(ret < 0){
            dev_err(&ts->client->dev, "%s: writing NVM command failed: %d\n", __func__, ret);
            return ret;
        }
    }

    dev_info(&ts->client->dev, "%s: color cal succesfully written!\n", __func__);
    return size;
}

static DEVICE_ATTR(color_cal, 0664, ss_color_cal_read, ss_color_cal_store);

static ssize_t ss_touch_count_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct ss_ts_data *ts = dev_get_drvdata(dev);
    return sprintf(buf, "%llu\n", ts->touch_count);
}

static DEVICE_ATTR(touch_count, 0444, ss_touch_count_read, NULL);

static struct attribute *ss_attributes[] = {
    &dev_attr_color_cal.attr,
    &dev_attr_touch_count.attr,
    NULL
};

static const struct attribute_group ss_attr_group = {
    .attrs = ss_attributes,
};

static int ss_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct ss_ts_data *ts;
    struct firmware *fw;
	int error, boot_status;
    char test;

    dev_info(&client->dev, "SAMSUNG PANEL probe\n");

    // Check that we are running with a Samsung panel, otherwise abort
    if(strstr(saved_command_line, "ea8074") == NULL){
        dev_err(&client->dev, "Not running on a Samsung EA8074 panel, aborting\n");
        return -ENODEV;
    }

    ts = devm_kzalloc(&client->dev, sizeof(struct ss_ts_data), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

    // Init regulator
    error = ss_power_init(ts);
    if(IS_ERR(error)) {
        return error;
    }

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

    // Log if OEM
    if(ss_read(ts, 0, &test, 1) == 0) {
        if (test == 0xFF) {
            dev_err(&client->dev, "OEM touch detected\n");
        } else {
            dev_err(&client->dev, "clone touch detected\n");
        }
    }

    // Check that the panel is running in app mode, otherwise flash
    boot_status = ss_get_boot_status(ts);
    if (boot_status != SS_BOOT_STATUS_APP) {
        dev_info(&client->dev, "Device not in app mode (current status: 0x%x). Attempting reflash\n", boot_status);

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
        error = request_firmware(&fw, SS_FIRMWARE, &client->dev);
        if(error < 0){
            dev_err(&client->dev, "firmware request failed: %d\n", error);
            return error;
        }

        // Flash firmware
        error = ss_flash_firmware(ts, fw);
        if(error < 0){
            dev_err(&client->dev, "firmware flash failed: %d\n", error);
            return error;
        }

        // Exit boot mode
        error = ss_exit_boot_mode(ts);
        if(error < 0) {
            return error;
        }
    }

    // Enable sensing
    error = ss_enable_sensing(ts);
    if(error < 0) {
        return error;
    }

	ts->input = devm_input_allocate_device(&client->dev);
	if (!ts->input) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input->name = "Samsung Touchscreen";
	ts->input->id.bustype = BUS_I2C;
    ts->touch_count = 0;

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

    // Handle all events currently stored
    ss_ts_irq_handler(client->irq, ts);

    // Init sysfs
    error = sysfs_create_group(&client->dev.kobj, &ss_attr_group);
    if (error) {
        dev_err(&client->dev, "failed to create sysfs group: %d\n", error);
		return error;
    }

	return 0;
}

static int ss_ts_remove(struct i2c_client *client)
{
    dev_info(&client->dev, "SAMSUNG PANEL remove\n");
    sysfs_remove_group(&client->dev.kobj, &ss_attr_group);
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
