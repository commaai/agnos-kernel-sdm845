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
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <video/lt9611.h>
#include <linux/workqueue.h>

#define CFG_HPD_INTERRUPTS	BIT(0)
#define CFG_EDID_INTERRUPTS	BIT(1)
#define CFG_CEC_INTERRUPTS	BIT(2)
#define CFG_VID_CHK_INTERRUPTS	BIT(3)

#define EDID_SEG_SIZE	0x100
#define AUDIO_DATA_SIZE	32
#define DISPLAY_4K_VERTICAL_ACTIVE 0x870

typedef enum {
	MIPI_1LANE = 1,
	MIPI_2LANE = 2,
	MIPI_3LANE = 3,
	MIPI_4LANE = 0,
} mipi_lane_counts;


typedef enum {
	MIPI_1PORT = 0x00,
	MIPI_2PORT = 0x03,
} mipi_port_counts;

typedef enum {
	VIDEO_640x480_60HZ = 0,
	VIDEO_720x480_60HZ = 1,
	VIDEO_1280x720_60HZ = 2,
	VIDEO_1920x1080_30HZ = 3,
	VIDEO_1920x1080_60HZ = 4,
	VIDEO_3840x1080_60HZ = 5,
	VIDEO_3840x2160_30HZ = 6,
} video_format_id;


struct lt9611_reg_cfg {
	u8 reg;
	u8 val;
	int sleep_in_ms;
};

struct lt9611_video_cfg {
	u32 h_front_porch;
	u32 h_pulse_width;
	u32 h_back_porch;
	u32 h_active;
	u32 v_front_porch;
	u32 v_pulse_width;
	u32 v_back_porch;
	u32 v_active;
	bool h_polarity;
	bool v_polarity;
	u32 vic;
	u32 pclk_khz;
};

struct lt9611 {
	struct i2c_client *i2c_client;

	mipi_port_counts mipi_port_counts;
	mipi_lane_counts mipi_lane_counts;
	video_format_id	video_format_id;

	int irq;
	int irq_gpio;
	int reset_gpio;
	int pwr_enable_gpio;
	int pwr_sys5v0_gpio;
	int pwr_sys3v3_gpio;

	int pwr_sys1v8_gpio;
	int receiver_enable_gpio;

	u8 pcr_m;
	u8 edid_buf[EDID_SEG_SIZE];

	bool IsDisplayOn ;
	struct delayed_work on_work;


};

static struct lt9611 *this_lt9611 = NULL;
#define LT9611_PATTERN_TEST	(0)
static void LT9611_pattern(struct lt9611 *pdata);
static int lt9611_i2s_init(struct lt9611 *pdata);


static struct lt9611_video_cfg video_tab[] = {
	{ 8,   96,  40,	640,  33, 2,  10, 480,	0, 0, 0,  25000	},//video_640x480_60Hz
	{ 16,  62,  60,	720,  9,  6,  30, 480,	0, 0, 0,  27000	},//video_720x480_60Hz
	{ 110, 40, 220, 1280, 5,  5,  20, 720,	1, 1, 0,  74250	},//video_1280x720_60Hz
	{ 88,  44, 148, 1920, 4,  5,  36, 1080, 1, 1, 0,  74250	},//video_1920x1080_30Hz
	{ 88,  44, 148, 1920, 4,  5,  36, 1080, 1, 1, 16, 148500 },//video_1920x1080_60Hz
	{ 176, 88, 296, 3840, 4,  5,  36, 1080, 1, 1, 0,  297000 },//video_3840x1080_60Hz
	{ 176, 88, 296, 3840, 8,  10, 72, 2160, 1, 1, 0,  297000 },//video_3840x2160_30Hz
};

static int i2c_write_byte(struct i2c_client *client, u8 addr, u8 reg, u8 val)
{
	int rc = 0;
	struct i2c_msg msg;
	u8 buf[2] = { reg, val };

	if (!client) {
		printk("%s: Invalid params\n", __func__);
		return -EINVAL;
	}

	msg.addr = addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;

	if (i2c_transfer(client->adapter, &msg, 1) < 1) {
		printk("%s: i2c write failed\n", __func__);
		rc = -EIO;
	}

	return rc;
}

static int i2c_read(struct i2c_client *client, u8 addr, u8 reg, char *buf, u32 size)
{
	int rc = 0;
	struct i2c_msg msg[2];

	if (!client || !buf) {
		printk("%s: Invalid params\n", __func__);
		return -EINVAL;
	}

	msg[0].addr = addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;

	msg[1].addr = addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size;
	msg[1].buf = buf;

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		printk("%s: i2c read failed\n", __func__);
		rc = -EIO;
	}

	return rc;
}

static int lt9611_read(struct lt9611 *pdata, u8 reg, char *buf, u32 size)
{
	int ret = 0;

	ret = i2c_read(pdata->i2c_client, pdata->i2c_client->addr, reg, buf, size);
	if (ret) {
		printk("%s: read err: reg 0x%02x, size 0x%02x\n", __func__, reg, size);
	}

	return ret;
}

static int lt9611_write(struct lt9611 *pdata, u8 reg, u8 val)
{
	int ret = 0;

	ret = i2c_write_byte(pdata->i2c_client, pdata->i2c_client->addr, reg, val);
	if (ret) {
		printk("%s: wr err: reg 0x%02x, val 0x%02x\n", __func__, reg, val);
	}

	return ret;
}

static int lt9611_read_device_rev(struct lt9611 *pdata)
{
	int ret = 0;
	u8 rev = 0;

	lt9611_write(pdata, 0xff, 0x80);
	lt9611_write(pdata, 0xee, 0x01);

	ret = lt9611_read(pdata, 0x02, &rev, 1);
	if (!ret) {
		printk(KERN_INFO"%s: lt9611 version: 0x%02x\n", __func__, rev);
	}

	return ret;
}

static int lt9611_system_init(struct lt9611 *pdata)
{
	lt9611_write(pdata, 0xFF, 0x81);
	lt9611_write(pdata, 0x01, 0x18);//sel xtal clock
	lt9611_write(pdata, 0xFF, 0x82);
	lt9611_write(pdata, 0x51, 0x11);
	//Timer for Frequency meter
	lt9611_write(pdata, 0xFF, 0x82);
	lt9611_write(pdata, 0x1b, 0x69);//Timer 2
	lt9611_write(pdata, 0x1c, 0x78);
	lt9611_write(pdata, 0xcb, 0x69);//Timer 1
	lt9611_write(pdata, 0xcc, 0x78);

	/*power consumption for work*/
	lt9611_write(pdata, 0xff, 0x80);
	lt9611_write(pdata, 0x04, 0xf0);
	lt9611_write(pdata, 0x06, 0xf0);
	lt9611_write(pdata, 0x0a, 0x80);
	lt9611_write(pdata, 0x0b, 0x40);
	lt9611_write(pdata, 0x0d, 0xef);
	lt9611_write(pdata, 0x11, 0xfa);

	return 0;
}


static int lt9611_mipi_input_analog(struct lt9611 *pdata)
{
	//mipi mode
	lt9611_write(pdata, 0xff, 0x81);
	lt9611_write(pdata, 0x06, 0x20);   //port A rx current
	lt9611_write(pdata, 0x07, 0x3f);   //20180420 PortB EQ
	lt9611_write(pdata, 0x08, 0x3f);   //20180420 PortB EQ
	lt9611_write(pdata, 0x0a, 0xfe);   //port A ldo voltage set
	lt9611_write(pdata, 0x0b, 0xbf);   //enable port A lprx
	lt9611_write(pdata, 0x11, 0x20);   //port B rx current 
	lt9611_write(pdata, 0x12, 0x3f);   //20180420 PortB EQ
	lt9611_write(pdata, 0x13, 0x3f);   //20180420 PortB EQ
	lt9611_write(pdata, 0x15, 0xfe);   //port B ldo voltage set
	lt9611_write(pdata, 0x16, 0xbf);   //enable port B lprx

	lt9611_write(pdata, 0x1c, 0x03);   //PortA clk lane no-LP mode.
	lt9611_write(pdata, 0x20, 0x03);   //PortB clk lane no-LP mode.

	return 0;
}

static int lt9611_mipi_input_digital(struct lt9611 *pdata)
{
	u8 lanes;
	u8 ports;

	lanes = pdata->mipi_lane_counts;
	ports = pdata->mipi_port_counts;

	lt9611_write(pdata, 0xff, 0x82);
	lt9611_write(pdata, 0x4f, 0x80);   //[7] = Select ad_txpll_d_clk.
	lt9611_write(pdata, 0x50, 0x10);

	lt9611_write(pdata, 0xff, 0x83);
	lt9611_write(pdata, 0x00, lanes);
	lt9611_write(pdata, 0x02, 0x0a);    //settle

	lt9611_write(pdata, 0x06, 0x0a);    //settle
	lt9611_write(pdata, 0x0a, ports);

	return 0;
}

static int lt9611_mipi_video_setup(struct lt9611 *pdata)
{
	u32 h_total, h_act, hpw, hfp, hss;
	u32 v_total, v_act, vpw, vfp, vss;
	video_format_id video_id;
	struct lt9611_video_cfg *cfg;

	video_id = pdata->video_format_id;
	cfg = &video_tab[video_id];

	h_total = cfg->h_active + cfg->h_front_porch + cfg->h_pulse_width + cfg->h_back_porch;
	v_total = cfg->v_active + cfg->v_front_porch + cfg->v_pulse_width + cfg->v_back_porch;

	h_act = cfg->h_active;
	hpw = cfg->h_pulse_width;
	hfp = cfg->h_front_porch;
	hss = cfg->h_pulse_width + cfg->h_back_porch;

	v_act = cfg->v_active;
	vpw = cfg->v_pulse_width;
	vfp = cfg->v_front_porch;
	vss = cfg->v_pulse_width + cfg->v_back_porch;

	printk(KERN_INFO "%s: h_total=%d, h_active=%d, hfp=%d, hpw=%d, hbp=%d\n", __func__,
			h_total, cfg->h_active, cfg->h_front_porch,
			cfg->h_pulse_width, cfg->h_back_porch);

	printk(KERN_INFO "%s: v_total=%d, v_active=%d, vfp=%d, vpw=%d, vbp=%d\n", __func__,
			v_total, cfg->v_active, cfg->v_front_porch,
			cfg->v_pulse_width, cfg->v_back_porch);

	lt9611_write(pdata, 0xff, 0x83);

	lt9611_write(pdata, 0x0d, (u8)(v_total / 256));
	lt9611_write(pdata, 0x0e, (u8)(v_total % 256));

	lt9611_write(pdata, 0x0f, (u8)(v_act / 256));
	lt9611_write(pdata, 0x10, (u8)(v_act % 256));

	lt9611_write(pdata, 0x11, (u8)(h_total / 256));
	lt9611_write(pdata, 0x12, (u8)(h_total % 256));

	lt9611_write(pdata, 0x13, (u8)(h_act / 256));
	lt9611_write(pdata, 0x14, (u8)(h_act % 256));

	lt9611_write(pdata, 0x15, (u8)(vpw % 256));
	lt9611_write(pdata, 0x16, (u8)(hpw % 256));

	lt9611_write(pdata, 0x17, (u8)(vfp % 256));

	lt9611_write(pdata, 0x18, (u8)(vss % 256));

	lt9611_write(pdata, 0x19, (u8)(hfp % 256));

	lt9611_write(pdata, 0x1a, (u8)(hss / 256));
	lt9611_write(pdata, 0x1b, (u8)(hss % 256));

	return 0;
}


static int lt9611_pll_setup(struct lt9611 *pdata)
{
	u32 pclk;
	u8 pcr_m;
	u8 hdmi_post_div;
	u8 pll_lock_flag;
	u8 i;
	u8 format_id;
	int ret;
	struct lt9611_video_cfg *cfg;

	format_id = pdata->video_format_id;
	cfg = &video_tab[format_id];
	pclk = cfg->pclk_khz;

	printk(KERN_INFO "%s: set rx pll = %d\n", __func__, pclk);

	lt9611_write(pdata, 0xff, 0x81);
	lt9611_write(pdata, 0x23, 0x40);
	lt9611_write(pdata, 0x24, 0x64);
	lt9611_write(pdata, 0x25, 0x80); //pre-divider
	lt9611_write(pdata, 0x26, 0x55);
	lt9611_write(pdata, 0x2c, 0x37);
	lt9611_write(pdata, 0x2f, 0x01);
	lt9611_write(pdata, 0x26, 0x55);
	lt9611_write(pdata, 0x27, 0x66);
	lt9611_write(pdata, 0x28, 0x88);

	if (pclk > 150000) {
		lt9611_write(pdata, 0x2d, 0x88);
		hdmi_post_div = 0x01;
	} else if (pclk > 70000) {
		lt9611_write(pdata, 0x2d, 0x99);
		hdmi_post_div = 0x02;
	} else {
		lt9611_write(pdata, 0x2d, 0xaa);
		hdmi_post_div = 0x04;
	}

	pcr_m = (u8)((pclk * 5 * hdmi_post_div) / 27000);
	lt9611_write(pdata, 0xff, 0x83); //M up limit
	lt9611_write(pdata, 0x2d, 0x40); //M down limit
	lt9611_write(pdata, 0x31, 0x08);
	lt9611_write(pdata, 0x26, 0x80 | pcr_m);
	pdata->pcr_m = pcr_m;

	printk(KERN_INFO "%s: pcr_m = 0x%x\n", __func__, pcr_m);

	pclk = pclk / 2;
	lt9611_write(pdata, 0xff, 0x82);             //13.5M
	lt9611_write(pdata, 0xe3, pclk / 65536);
	pclk = pclk % 65536;
	lt9611_write(pdata, 0xe4, pclk / 256);
	lt9611_write(pdata, 0xe5, pclk % 256);

	lt9611_write(pdata, 0xde, 0x20);
	lt9611_write(pdata, 0xde, 0xe0);

	lt9611_write(pdata, 0xff, 0x80);
	lt9611_write(pdata, 0x11, 0x5a);                /* Pcr clk reset */
	lt9611_write(pdata, 0x11, 0xfa);
	lt9611_write(pdata, 0x18, 0xdc);                /* pll analog reset */
	lt9611_write(pdata, 0x18, 0xfc);
	lt9611_write(pdata, 0x16, 0xf1);
	lt9611_write(pdata, 0x16, 0xf3);

	/* pll lock status */
	for (i = 0; i < 6; i++) {
		lt9611_write(pdata, 0xff, 0x80);
		lt9611_write(pdata, 0x16, 0xe3);         /* pll lock logic reset */
		lt9611_write(pdata, 0x16, 0xf3);
		lt9611_write(pdata, 0xff, 0x82);
		msleep(5);
		ret = lt9611_read(pdata, 0x15, &pll_lock_flag, 1);
		if (pll_lock_flag & 0x80) {
			printk("%s: HDMI pll locked\n", __func__);
			break;
		} else {
			lt9611_write(pdata, 0xff, 0x80);
			lt9611_write(pdata, 0x11, 0x5a);                /* Pcr clk reset */
			lt9611_write(pdata, 0x11, 0xfa);
			lt9611_write(pdata, 0x18, 0xdc);                /* pll analog reset */
			lt9611_write(pdata, 0x18, 0xfc);
			lt9611_write(pdata, 0x16, 0xf1);                /* pll cal reset*/
			lt9611_write(pdata, 0x16, 0xf3);
			printk("%s: HDMI pll unlocked, reset pll\n", __func__);
		}
	}

	return 0;
}

static int lt9611_pcr_setup(struct lt9611 *pdata)
{
	video_format_id video_format_id = pdata->video_format_id;

	if (!pdata) {
		printk("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	lt9611_write(pdata, 0xff, 0x83);
	lt9611_write(pdata, 0x0b, 0x01);        //vsync mode
	lt9611_write(pdata, 0x0c, 0x10);        //=1/4 hact

	lt9611_write(pdata, 0x48, 0x00);        //de mode delay
	lt9611_write(pdata, 0x49, 0x81);        //=1/4 hact

	/* stage 1 */
	lt9611_write(pdata, 0x21, 0x4a); //bit[3:0] step[11:8]

	lt9611_write(pdata, 0x24, 0x31);        //bit[7:4]v/h/de mode; line for clk stb[11:8]
	lt9611_write(pdata, 0x25, 0x30);        //line for clk stb[7:0]

	lt9611_write(pdata, 0x2a, 0x01);        //clk stable in

	/* stage 2 */
	lt9611_write(pdata, 0x4a, 0x40);        //offset //0x10
	lt9611_write(pdata, 0x1d, 0x10);        //PCR de mode step setting.

	/* MK limit */
	//lt9611_write(pdata, 0x2d, 0x38);        //M up limit
	//lt9611_write(pdata, 0x31, 0x08);        //M down limit

	switch (video_format_id) {
	case VIDEO_3840x1080_60HZ:
	case VIDEO_3840x2160_30HZ:

		lt9611_write(pdata, 0x0b, 0x03);        //vsync mode
		lt9611_write(pdata, 0x0c, 0xd0);        //=1/4 hact

		lt9611_write(pdata, 0x48, 0x03);        //de mode delay
		lt9611_write(pdata, 0x49, 0xe0);        //

		lt9611_write(pdata, 0x24, 0x72);        //bit[7:4]v/h/de mode; line for clk stb[11:8]
		lt9611_write(pdata, 0x25, 0x00);        //line for clk stb[7:0]

		lt9611_write(pdata, 0x2a, 0x01);        //clk stable in

		lt9611_write(pdata, 0x4a, 0x10);        //offset
		lt9611_write(pdata, 0x1d, 0x10);        //PCR de mode step setting.
		break;
	case VIDEO_1920x1080_60HZ:
	case VIDEO_1920x1080_30HZ:
	case VIDEO_1280x720_60HZ:
	case VIDEO_720x480_60HZ:
	case VIDEO_640x480_60HZ:
		break;
	default:
		break;
	}

	return 0;
}

static int lt9611_pcr_start(struct lt9611 *pdata)
{
	u8 pcr_m;

	pcr_m = pdata->pcr_m;
	lt9611_write(pdata, 0xff, 0x83);
	lt9611_write(pdata, 0x26, pcr_m);
	printk(KERN_INFO "%s: pcr_m = 0x%x\n", __func__, pcr_m);

	lt9611_write(pdata, 0xff, 0x80);  //Pcr reset
	lt9611_write(pdata, 0x11, 0x5a);
	lt9611_write(pdata, 0x11, 0xfa);

	return 0;
}

static int lt9611_hdmi_tx_digital(struct lt9611 *pdata)
{
	struct lt9611_video_cfg *cfg;
	video_format_id video_id;
	u32 checksum, vic;

	video_id = pdata->video_format_id;

	printk(KERN_INFO "%s: video_id: %d\n", __func__, video_id);

	cfg = &video_tab[video_id];
	vic = cfg->vic;

	printk(KERN_INFO "%s: vic: %d\n", __func__, vic);
	checksum = 0x46 - vic;

	lt9611_write(pdata, 0xff, 0x84);
	lt9611_write(pdata, 0x43, checksum);
	lt9611_write(pdata, 0x44, 0x10);
	lt9611_write(pdata, 0x47, vic);

	lt9611_write(pdata, 0xff, 0x82);
	lt9611_write(pdata, 0xd6, 0x8c);
	lt9611_write(pdata, 0xd7, 0x04);

	return 0;
}

static int lt9611_hdmi_tx_phy(struct lt9611 *pdata)
{
	lt9611_write(pdata, 0xff, 0x81);
	lt9611_write(pdata, 0x30, 0x6a);
	lt9611_write(pdata, 0x31, 0x44);//DC: 0x44, AC:0x73
	lt9611_write(pdata, 0x32, 0x4a);
	lt9611_write(pdata, 0x33, 0x0b);
	lt9611_write(pdata, 0x34, 0x00);
	lt9611_write(pdata, 0x35, 0x00);
	lt9611_write(pdata, 0x36, 0x00);
	lt9611_write(pdata, 0x37, 0x44);
	lt9611_write(pdata, 0x3f, 0x0f);
	lt9611_write(pdata, 0x40, 0x90);
	lt9611_write(pdata, 0x41, 0x90);
	lt9611_write(pdata, 0x42, 0x90);
	lt9611_write(pdata, 0x43, 0x90);
	lt9611_write(pdata, 0x44, 0x0a);

	return 0;
}

static void lt9611_hdmi_output_enable(struct lt9611 *pdata)
{
	lt9611_write(pdata, 0xff, 0x81);
	lt9611_write(pdata, 0x30, 0xea);
}

static void lt9611_hdmi_output_disable(struct lt9611 *pdata)
{
	lt9611_write(pdata, 0xff, 0x81);
	lt9611_write(pdata, 0x30, 0x6a);
}

static int lt9611_read_edid(struct lt9611 *pdata)
{
	int ret = 0;
	u8 i, j, temp;

	memset(pdata->edid_buf, 0, EDID_SEG_SIZE);

	lt9611_write(pdata, 0xff, 0x85);
	lt9611_write(pdata, 0x03, 0xc9);
	lt9611_write(pdata, 0x04, 0xa0);/* 0xA0 is EDID device address */
	lt9611_write(pdata, 0x05, 0x00);/* 0x00 is EDID offset address */
	lt9611_write(pdata, 0x06, 0x20);/* length for read */
	lt9611_write(pdata, 0x14, 0x7f);

	for (i = 0; i < 8; i++) {
		lt9611_write(pdata, 0x05, i * 32); /* offset address */
		lt9611_write(pdata, 0x07, 0x36);
		lt9611_write(pdata, 0x07, 0x31);
		lt9611_write(pdata, 0x07, 0x37);
		mdelay(5);   /* wait 5ms before reading edid */

		lt9611_read(pdata, 0x40, &temp, 1);
		printk(KERN_INFO "%s: temp = 0x%02x\n", __func__, temp);
		if (temp & 0x02) {                  /*KEY_DDC_ACCS_DONE=1*/

			if (temp & 0x50) {           /* DDC No Ack or Abitration lost */
				printk(KERN_INFO "%s: read edid failed: no ack\n",__func__);
				ret = -EINVAL;
				goto end;
			} else {
				for (j = 0; j < 32; j++)
					lt9611_read(pdata, 0x83, &(pdata->edid_buf[i * 32 + j]), 1);
			}

		} else {
			printk(KERN_INFO "%s: read edid failed: access not done\n", __func__);
			ret = -EINVAL;
			goto end;
		}
	}

	printk(KERN_INFO "%s: read edid succeeded, checksum = 0x%x\n", __func__, pdata->edid_buf[255]);
end:
	lt9611_write(pdata, 0x03, 0xc2);
	lt9611_write(pdata, 0x07, 0x1f);
	return ret;
}

static void lt9611_dump_edid(struct lt9611 *pdata)
{
	int i,j;

	for (i = 0; i < 16; i++) {
		printk(KERN_INFO "----%02xH-to-%02xH---\n", i, i+15);
		for (j = 0; j < 16; j++) {
			printk(KERN_INFO "0x%02x ", pdata->edid_buf[i * 16 + j]);
		}
		printk(KERN_INFO "\n");
	}
}

static int lt9611_video_check_debug(struct lt9611 *pdata)
{
	u32 v_total, v_act, h_act_a, h_act_b, h_total_sysclk;
	u8 val;
	int ret = 0;

	/* top module video check */
	lt9611_write(pdata, 0xff, 0x82);

	/* v_act */
	ret = lt9611_read(pdata, 0x82, &val, 1);
	if (ret)
		goto end;

	v_act = val << 8;
	ret = lt9611_read(pdata, 0x83, &val, 1);
	if (ret)
		goto end;
	v_act = v_act + val;

	/* v_total */
	ret = lt9611_read(pdata, 0x6c, &val, 1);
	if (ret)
		goto end;
	v_total = val << 8;
	ret = lt9611_read(pdata, 0x6d, &val, 1);
	if (ret)
		goto end;
	v_total = v_total + val;

	/* h_total_sysclk */
	ret = lt9611_read(pdata, 0x86, &val, 1);
	if (ret)
		goto end;
	h_total_sysclk = val << 8;
	ret = lt9611_read(pdata, 0x87, &val, 1);
	if (ret)
		goto end;
	h_total_sysclk = h_total_sysclk + val;

	/* h_act_a */
	lt9611_write(pdata, 0xff, 0x83);
	ret = lt9611_read(pdata, 0x82, &val, 1);
	if (ret)
		goto end;
	h_act_a = val << 8;
	ret = lt9611_read(pdata, 0x83, &val, 1);
	if (ret)
		goto end;
	h_act_a = (h_act_a + val) / 3;

	/* h_act_b */
	lt9611_write(pdata, 0xff, 0x83);
	ret = lt9611_read(pdata, 0x86, &val, 1);
	if (ret)
		goto end;
	h_act_b = val << 8;
	ret = lt9611_read(pdata, 0x87, &val, 1);
	if (ret)
		goto end;
	h_act_b = (h_act_b + val) / 3;

	printk(KERN_INFO "%s: video check: h_act_a=%d, h_act_b=%d, v_act=%d, v_total=%d, h_total_sysclk=%d\n",
			__func__, h_act_a, h_act_b, v_act, v_total, h_total_sysclk);

	return 0;

end:
	printk(KERN_ERR "%s: read video check error\n", __func__);
	return ret;
}

static int lt9611_pcr_mk_debug(struct lt9611 *pdata)
{
	u8 m, k1, k2, k3;
	u8 reg_8397h;
	u8 i;

	for (i = 0; i < 30; i++) {
		lt9611_write(pdata, 0xff, 0x83);
		lt9611_read(pdata, 0x97, &reg_8397h, 1);
		lt9611_read(pdata, 0xb4, &m, 1);
		lt9611_read(pdata, 0xb5, &k1, 1);
		lt9611_read(pdata, 0xb6, &k2, 1);
		lt9611_read(pdata, 0xb7, &k3, 1);

		printk("%s: 0x8397 = 0x%x; pcr mk:0x%x 0x%x 0x%x 0x%x\n", __func__,
				reg_8397h, m, k1, k2, k3);
	}

	return 0;
}

static int lt9611_htotal_stable_debug(struct lt9611 *pdata)
{
	u32 h_total_sysclk;
	u8 val, i;
	int ret;

	lt9611_write(pdata, 0xff, 0x82);

	for (i = 0; i < 30; i++) {
		/* h_total_sysclk */
		ret = lt9611_read(pdata, 0x86, &val, 1);
		if (ret)
			goto end;
		h_total_sysclk = val << 8;
		ret = lt9611_read(pdata, 0x87, &val, 1);
		if (ret)
			goto end;
		h_total_sysclk = h_total_sysclk + val;

		printk("%s: h_total_sysclk = %d\n", __func__, h_total_sysclk);
	}

	return 0;
end:
	printk(KERN_ERR "%s: read htotal by system clock error\n", __func__);
	return ret;
}

static int lt9611_mipi_byte_clk_debug(struct lt9611 *pdata)
{
	u8 reg_val;
	u32 byte_clk;

	/* port A byte clk meter */
	lt9611_write(pdata, 0xff, 0x82);
	lt9611_write(pdata, 0xc7, 0x03);        /* port A */
	msleep(50);
	lt9611_read(pdata, 0xcd, &reg_val, 1);

	if ((reg_val & 0x60) == 0x60) {
		byte_clk = (reg_val & 0x0f) * 65536;
		lt9611_read(pdata, 0xce, &reg_val, 1);
		byte_clk = byte_clk + reg_val * 256;
		lt9611_read(pdata, 0xcf, &reg_val, 1);
		byte_clk = byte_clk + reg_val;

		printk("%s: port A byte clk = %d khz,\n", __func__, byte_clk);
	} else {
		printk("%s: port A byte clk unstable\n", __func__);
	}

	/* port B byte clk meter */
	lt9611_write(pdata, 0xff, 0x82);
	lt9611_write(pdata, 0xc7, 0x04); /* port B */
	msleep(50);
	lt9611_read(pdata, 0xcd, &reg_val, 1);

	if ((reg_val & 0x60) == 0x60) {
		byte_clk = (reg_val & 0x0f) * 65536;
		lt9611_read(pdata, 0xce, &reg_val, 1);
		byte_clk = byte_clk + reg_val * 256;
		lt9611_read(pdata, 0xcf, &reg_val, 1);
		byte_clk = byte_clk + reg_val;

		printk("%s: port B byte clk = %d khz,\n", __func__, byte_clk);
	} else {
		printk("%s: port B byte clk unstable\n", __func__);
	}

	return 0;
}


static void lt9611_reset(struct lt9611 *pdata)
{
	gpio_direction_output(pdata->reset_gpio, 1);
	msleep(100);
	gpio_direction_output(pdata->reset_gpio, 0);
	msleep(100);
	gpio_direction_output(pdata->reset_gpio, 1);
	msleep(100);
}

static int lt9611_parse_dt(struct device *dev, struct lt9611 *pdata)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	pdata->irq_gpio = of_get_named_gpio(np, "lt,irq-gpio", 0);
	if (!gpio_is_valid(pdata->irq_gpio)) {
		printk(KERN_ERR "%s: irq gpio not specified\n", __func__);
		return -EINVAL;
	}

	pdata->reset_gpio = of_get_named_gpio(np, "lt,reset-gpio", 0);
	if (!gpio_is_valid(pdata->reset_gpio)) {
		printk(KERN_ERR "%s: reset gpio not specified\n", __func__);
		return -EINVAL;
	}
/*
	pdata->receiver_enable_gpio = of_get_named_gpio(np, "lt,receiver-enable-gpio", 0);
	if (!gpio_is_valid(pdata->receiver_enable_gpio)) {
		printk(KERN_ERR "%s: receiver enable gpio not specified\n", __func__);
		return -EINVAL;
	}

	pdata->pwr_enable_gpio = of_get_named_gpio(np, "lt,pwr-enable-gpio", 0);
	if (!gpio_is_valid(pdata->pwr_enable_gpio)) {
		printk(KERN_ERR "%s: pwr enable gpio not specified\n", __func__);
		return -EINVAL;
	}
*/
	pdata->pwr_sys5v0_gpio = of_get_named_gpio(np, "lt,pwr-sys5v0-gpio", 0);
	if (!gpio_is_valid(pdata->pwr_sys5v0_gpio)) {
		printk(KERN_ERR "%s: pwr sys5v0 gpio not specified\n", __func__);
		return -EINVAL;
	}
/*
	pdata->pwr_sys3v3_gpio = of_get_named_gpio(np, "lt,pwr-sys3v3-gpio", 0);
	if (!gpio_is_valid(pdata->pwr_sys3v3_gpio)) {
		printk(KERN_ERR "%s: pwr sys3v3 gpio not specified\n", __func__);
		return -EINVAL;
	}
*/
	pdata->pwr_sys1v8_gpio = of_get_named_gpio(np, "lt,pwr-sys1v8-gpio", 0);
	if (!gpio_is_valid(pdata->pwr_sys1v8_gpio)) {
		printk(KERN_ERR "%s: pwr sys1v8 gpio not specified\n", __func__);
		return -EINVAL;
	}

	return ret;
}


static int lt9611_gpio_init(struct lt9611 *pdata)
{
	int ret = 0;

	ret = gpio_request(pdata->reset_gpio, "lt9611-reset-gpio");
	if (!ret) {
		printk(KERN_INFO "%s: reset gpio request success!\n", __func__);
		gpio_direction_output(pdata->reset_gpio, 1);
		msleep(10);
	}

	ret = gpio_request(pdata->irq_gpio, "lt9611-irq-gpio");
	if (!ret) {
		printk(KERN_INFO "%s: irq gpio request success!\n", __func__);
		gpio_direction_input(pdata->irq_gpio);
	}
/*
	ret = gpio_request(pdata->receiver_enable_gpio, "lt9611-receiver-enable-gpio");
	if (!ret) {
		printk(KERN_INFO "%s: receiver enable gpio request success!\n", __func__);
		gpio_direction_output(pdata->receiver_enable_gpio, 1);
	}

	ret = gpio_request(pdata->pwr_sys3v3_gpio, "lt9611-pwr-sys3v3-gpio");
	if (!ret) {
		printk(KERN_INFO "%s: pwr sys3v3 gpio request success!\n", __func__);
		gpio_direction_output(pdata->pwr_sys3v3_gpio, 1);
	}
*/
	ret = gpio_request(pdata->pwr_sys5v0_gpio, "lt9611-pwr-sys5v0-gpio");
	if (!ret) {
		printk(KERN_INFO "%s: pwr sys5v0 gpio request success!\n", __func__);
		gpio_direction_output(pdata->pwr_sys5v0_gpio, 1);
	}

	ret = gpio_request(pdata->pwr_sys1v8_gpio, "lt9611-pwr-sys1v8-gpio");
	if (!ret) {
		printk(KERN_INFO "%s: pwr sys1v8 gpio request success!\n", __func__);
		gpio_direction_output(pdata->pwr_sys1v8_gpio, 1);
		msleep(10);
	}
/*
	ret = gpio_request(pdata->pwr_enable_gpio, "lt9611-pwr-enable-gpio");
	if (!ret) {
		printk(KERN_INFO "%s: pwr enable gpio request success!\n", __func__);
		gpio_direction_output(pdata->pwr_enable_gpio, 1);
		msleep(10);
	}
*/
	return ret;
}

static int lt9611_gpio_deinit(struct lt9611 *pdata)
{
//	gpio_direction_output(pdata->receiver_enable_gpio, 0);
//	gpio_direction_output(pdata->pwr_enable_gpio, 0);
	gpio_direction_output(pdata->pwr_sys5v0_gpio, 0);
//	gpio_direction_output(pdata->pwr_sys3v3_gpio, 0);
//	gpio_direction_output(pdata->pwr_sys1v8_gpio, 0);
	gpio_direction_output(pdata->reset_gpio, 0);

	gpio_free(pdata->reset_gpio);
	gpio_free(pdata->irq_gpio);
	//gpio_free(pdata->receiver_enable_gpio);
//	gpio_free(pdata->pwr_enable_gpio);
	gpio_free(pdata->pwr_sys5v0_gpio);
//	gpio_free(pdata->pwr_sys3v3_gpio);
//	gpio_free(pdata->pwr_sys1v8_gpio);

	return 0;
}

static irqreturn_t lt9611_interrupt_thread(int irq, void *dev_id)
{


#if 0
	struct lt9611 *pdata = (struct lt9611*)dev_id;
	u8 irq_flag0, irq_flag3;

	printk(KERN_INFO "%s: enter\n", __func__);
	lt9611_write(pdata, 0xff, 0x82);
	lt9611_read(pdata, 0x0f, &irq_flag3, 1);
	lt9611_read(pdata, 0x0f, &irq_flag0, 1);  //0x0f -->0x0c

	/* hpd changed low */
	if (irq_flag3 & 0x80) {
		printk(KERN_INFO "%s:hdmi cable disconnected\n", __func__);

		lt9611_write(pdata, 0xff, 0x82); /* irq 3 clear flag */
		lt9611_write(pdata, 0x07, 0xbf);
		lt9611_write(pdata, 0x07, 0x3f);
	}

	/* hpd changed high */
	if (irq_flag3 & 0x40) {
		printk(KERN_INFO "%s:hdmi cable connected\n", __func__);
		lt9611_read_edid(pdata);

		lt9611_write(pdata, 0xff, 0x82); /* irq 3 clear flag */
		lt9611_write(pdata, 0x07, 0x7f);
		lt9611_write(pdata, 0x07, 0x3f);
	}

	/* video input changed */
	if (irq_flag0 & 0x01) {
		printk(KERN_INFO "%s:video input changed\n", __func__);
		lt9611_write(pdata, 0xff, 0x82); /* irq 0 clear flag */
		lt9611_write(pdata, 0x9e, 0xff);
		lt9611_write(pdata, 0x9e, 0xf7);
		lt9611_write(pdata, 0x04, 0xff);
		lt9611_write(pdata, 0x04, 0xfe);
	}
#endif
	return IRQ_HANDLED;
}

static int lt9611_enable_interrupts(struct lt9611 *pdata, int interrupts)
{
	u8 reg_val, init_reg_val;

	if (interrupts & CFG_VID_CHK_INTERRUPTS) {
		lt9611_write(pdata, 0xff, 0x82);
		lt9611_read(pdata, 0x00, &reg_val, 1);

		if (reg_val & 0x01) { //reg_val | 0x01???
			init_reg_val = reg_val & 0xfe;
			printk(KERN_INFO "%s:enabling video check interrupts\n", __func__);
			lt9611_write(pdata, 0x00, init_reg_val);
		}

		lt9611_write(pdata, 0x04, 0xff); /* clear */
		lt9611_write(pdata, 0x04, 0xfe);
	}

	if (interrupts & CFG_HPD_INTERRUPTS) {
		lt9611_write(pdata, 0xff, 0x82);
		lt9611_read(pdata, 0x03, &reg_val, 1);

		if (reg_val & 0xc0) { //reg_val | 0xc0???
			init_reg_val = reg_val & 0x3f;
			printk(KERN_INFO "%s: enabling hpd interrupts\n", __func__);
			lt9611_write(pdata, 0x03, init_reg_val);
		}

		lt9611_write(pdata, 0x07, 0xff); //clear
		lt9611_write(pdata, 0x07, 0x3f);
	}
	return 0;
}

static void lt9611_on_work_fn (struct work_struct *work)
{
	int ret ;

	ret = lt9611_gpio_init(this_lt9611);
	msleep(500);

	lt9611_reset(this_lt9611);

	ret = lt9611_read_device_rev(this_lt9611);
	lt9611_system_init(this_lt9611);
	lt9611_mipi_input_analog(this_lt9611);
	lt9611_mipi_input_digital(this_lt9611);
	lt9611_mipi_video_setup(this_lt9611);
	lt9611_pll_setup(this_lt9611);
	lt9611_pcr_setup(this_lt9611);
	lt9611_pcr_start(this_lt9611);
	lt9611_hdmi_tx_digital(this_lt9611);
	lt9611_hdmi_tx_phy(this_lt9611);

	mdelay(100);
	lt9611_hdmi_output_enable(this_lt9611);
	lt9611_i2s_init(this_lt9611);
}

static int lt9611_video_on(struct lt9611 *pdata)
{
	int ret = 0;

	lt9611_mipi_input_analog(pdata);
	lt9611_mipi_input_digital(pdata);
	lt9611_pll_setup(pdata);
	lt9611_mipi_video_setup(pdata);
	lt9611_pcr_setup(pdata);
	lt9611_hdmi_tx_digital(pdata);
	lt9611_hdmi_tx_phy(pdata);

	msleep(500);

	lt9611_video_check_debug(pdata);
	lt9611_hdmi_output_enable(pdata);

	return ret;
}

static int lt9611_video_off(struct lt9611 *pdata)
{
	lt9611_hdmi_output_disable(pdata);
	return 0;
}

int lt9611_video_enable(int enable)
{
	if (!this_lt9611) {
		return -EINVAL;
	}

	if (enable) {
		lt9611_video_on(this_lt9611);
	} else {
		lt9611_video_off(this_lt9611);
	}

	return 0;
}

static int lt9611_irq_init(struct lt9611 *pdata)
{
	int ret = 0;

	pdata->irq = gpio_to_irq(pdata->irq_gpio);
	ret = devm_request_threaded_irq(&pdata->i2c_client->dev, pdata->irq,
			NULL, lt9611_interrupt_thread,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"lt9611_irq", pdata);
	if (ret) {
		printk(KERN_ERR "%s: requset irq failed\n", __func__);
	}

	lt9611_enable_interrupts(pdata, CFG_HPD_INTERRUPTS);
	return ret;
}

static int lt9611_i2s_init(struct lt9611 *pdata)
{
	lt9611_write(pdata, 0xff, 0x82);
	lt9611_write(pdata, 0xd6, 0x8c);
	lt9611_write(pdata, 0xd7, 0x04);

	lt9611_write(pdata, 0xff, 0x84);
	lt9611_write(pdata, 0x06, 0x08);
	lt9611_write(pdata, 0x07, 0x10);

	lt9611_write(pdata, 0x34, 0xd4);

	return 0;
}

static ssize_t lt9611_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!this_lt9611) {
		return -EINVAL;
	}

	lt9611_video_check_debug(this_lt9611);
	lt9611_mipi_byte_clk_debug(this_lt9611);
	lt9611_pcr_mk_debug(this_lt9611);
	lt9611_htotal_stable_debug(this_lt9611);

	return 1;
}

static ssize_t lt9611_debug_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (!this_lt9611) {
		return count;
	}
//to be done
	if (!strcmp(buf, "on")) {
		lt9611_hdmi_output_enable(this_lt9611);
	} else if (!strcmp(buf, "off")) {
		lt9611_hdmi_output_disable(this_lt9611);
	} else if (!strcmp(buf, "edid")) {
		lt9611_read_edid(this_lt9611);
		lt9611_dump_edid(this_lt9611);
	} else if (!strcmp(buf, "reset")) {
		lt9611_mipi_input_analog(this_lt9611);
		lt9611_mipi_input_digital(this_lt9611);
		lt9611_mipi_video_setup(this_lt9611);
		lt9611_pll_setup(this_lt9611);
		lt9611_pcr_setup(this_lt9611);
		lt9611_pcr_start(this_lt9611);
		lt9611_hdmi_tx_digital(this_lt9611);
		lt9611_hdmi_tx_phy(this_lt9611);
		lt9611_i2s_init(this_lt9611);
	} else if (!strcmp(buf, "irq")) {
		lt9611_enable_interrupts(this_lt9611, CFG_HPD_INTERRUPTS);
	}

	return count;
}

static DEVICE_ATTR(lt9611_debug, 0600, lt9611_debug_show, lt9611_debug_store);

void lt9611_on(int index)
{
	if(!this_lt9611->IsDisplayOn)
	{
		schedule_delayed_work(&this_lt9611->on_work, msecs_to_jiffies(0));
		this_lt9611->IsDisplayOn = true;
	}
}

void lt9611_off(int index)
{
	if(this_lt9611->IsDisplayOn)
	{
		lt9611_gpio_deinit(this_lt9611);
		this_lt9611->IsDisplayOn = false;
	}
}

static int lt9611_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lt9611 *pdata;
	int ret = 0;
	uint16_t pReferredTiming;

	printk(KERN_ERR "%s: i2c addr = 0x%02x enter!\n", __func__, client->addr);
	if (!client->dev.of_node) {
		printk(KERN_ERR "%s invalid argument\n", __func__);
		return -EINVAL;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct lt9611), GFP_KERNEL);
	if (!pdata) {
		printk(KERN_ERR "%s: memory allocate failed.\n", __func__);
		return -ENOMEM;
	}

	pdata->i2c_client = client;
	this_lt9611 = pdata;
	i2c_set_clientdata(client, pdata);

	ret = lt9611_parse_dt(&client->dev, pdata);
	if (ret) {
		printk(KERN_ERR "%s: failed to parse device tree\n", __func__);
		goto err_dt_parse;
	}

	ret = lt9611_gpio_init(pdata);
	if (ret) {
		printk(KERN_ERR "%s: failed to configure GPIOs\n", __func__);
		goto err_dt_parse;
	}

	pdata->video_format_id = VIDEO_3840x2160_30HZ;
	pdata->mipi_lane_counts = MIPI_4LANE;
	pdata->mipi_port_counts = MIPI_2PORT;


	lt9611_reset(pdata);

	ret = lt9611_read_device_rev(pdata);
	if (ret) {
		printk(KERN_ERR "%s: failed to read chip version\n", __func__);
		goto err_i2c_prog;
	}

	if (LT9611_PATTERN_TEST == 1) {
		LT9611_pattern(pdata);
		return 0;
	}

	lt9611_system_init(pdata);
	lt9611_read_edid(pdata);

	pReferredTiming = pdata->edid_buf[0x3b] + ((pdata->edid_buf[0x3d] & 0xF0)<<4);
	if(pReferredTiming == DISPLAY_4K_VERTICAL_ACTIVE)
	{
		pdata->video_format_id = VIDEO_3840x2160_30HZ;
		pdata->mipi_port_counts = MIPI_2PORT;
		printk(KERN_INFO "%s lt9611 preferred display 3840*2160\n",__func__);
	}
	else
	{
		pdata->video_format_id = VIDEO_1920x1080_60HZ;
		pdata->mipi_port_counts = MIPI_1PORT;
		 printk(KERN_INFO "%s lt9611 preferred display 1920*1080\n",__func__);
	}
	pdata->mipi_lane_counts = MIPI_4LANE;

	lt9611_mipi_input_analog(pdata);
	lt9611_mipi_input_digital(pdata);
	lt9611_mipi_video_setup(pdata);
	lt9611_pll_setup(pdata);
	lt9611_pcr_setup(pdata);
	lt9611_pcr_start(pdata);
	lt9611_hdmi_tx_digital(pdata);
	lt9611_hdmi_tx_phy(pdata);

	msleep(100);
	lt9611_hdmi_output_enable(pdata);
	lt9611_i2s_init(pdata);

	INIT_DELAYED_WORK(&pdata->on_work, lt9611_on_work_fn);

	pdata->IsDisplayOn = true;

	ret = device_create_file(&client->dev, &dev_attr_lt9611_debug);
	if (ret) {
		printk(KERN_ERR "%s: create debug sysfs fail!\n", __func__);
		goto err_i2c_prog;
	}
	ret = lt9611_irq_init(pdata);
	if (ret) {
		goto err_irq;
	}

	return ret;

err_irq:
	device_remove_file(&client->dev, &dev_attr_lt9611_debug);
err_i2c_prog:
	lt9611_gpio_deinit(pdata);
err_dt_parse:
	devm_kfree(&client->dev, pdata);
	this_lt9611 = NULL;
	return ret;
}


static int lt9611_remove(struct i2c_client *client)
{
	struct lt9611 *pdata = (struct lt9611 *)i2c_get_clientdata(client);

	this_lt9611 = NULL;
	device_remove_file(&client->dev, &dev_attr_lt9611_debug);
	lt9611_gpio_deinit(pdata);
	i2c_set_clientdata(client, NULL);
	printk("%s: remove lt9611 driver successfully", __func__);
	return 0;
}


static struct i2c_device_id lt9611_id[] = {
	{ "lt9611", 0 },
	{}
};

static const struct of_device_id lt9611_match_table[] = {
	{ .compatible = "lontium,lt9611" },
	{}
};

MODULE_DEVICE_TABLE(of, lt9611_match_table);

static struct i2c_driver lt9611_driver = {
	.driver			= {
		.name		= "lt9611",
		.owner		= THIS_MODULE,
		.of_match_table = lt9611_match_table,
	},
	.probe			= lt9611_probe,
	.remove			= lt9611_remove,
	.id_table		= lt9611_id,
};

module_i2c_driver(lt9611_driver);

MODULE_AUTHOR("xhguo@lontium.com");
MODULE_DESCRIPTION("Lontium bridge IC LT9611 that convert mipi to hdmi)");
MODULE_LICENSE("GPL");

#define HDMI_VIC	(0x10)
#define HDMI_Y	(0x00)
//0x00:RGB, 0x01:YCbCr422 0x02:YCbCr 444

struct video_timing {
	u16 hfp;
	u16 hs;
	u16 hbp;
	u16 hact;
	u16 htotal;
	u16 vfp;
	u16 vs;
	u16 vbp;
	u16 vact;
	u16 vtotal;
	u32 pclk_khz;
};
#if 0
static struct video_timing video_640x480_60Hz   = { 8, 96,  40, 640,   800, 33,  2,  10, 480,   525,  25000};
static struct video_timing video_720x480_60Hz   = {16, 62,  60, 720,   858,  9,  6,  30, 480,   525,  27000};
static struct video_timing video_1280x720_60Hz  = {110,40, 220,1280,  1650,  5,  5,  20, 720,   750,  74250};
static struct video_timing video_1280x720_30Hz  = {110,40, 220,1280,  1650,  5,  5,  20, 720,   750,  74250};
static struct video_timing video_1920x1080_30Hz = {88, 44, 148,1920,  2200,  4,  5,  36, 1080, 1125,  74250};
static struct video_timing video_1920x1080_60Hz = {88, 44, 148,1920,  2200,  4,  5,  36, 1080, 1125, 148500};
static struct video_timing video_3840x1080_60Hz = {176,88, 296,3840,  4400,  4,  5,  36, 1080, 1125, 297000};
static struct video_timing video_3840x2160_30Hz = {176,88, 296,3840,  4400,  8,  10, 72, 2160, 2250, 297000};
static struct video_timing video_1024x600_60Hz  = {60,60, 100,1024,  1154,  2,  5, 10, 600, 617, 34000};
#else
static struct video_timing video_1920x1080_60Hz = {88, 44, 148,1920,  2200,  4,  5,  36, 1080, 1125, 148500};
#endif
static void LT9611_System_Init(struct lt9611 *pdata)
{
	lt9611_write(pdata,0xFF,0x82);
	lt9611_write(pdata,0x51,0x01);
	//Timer for Frequency meter
	lt9611_write(pdata,0xFF,0x82);
	lt9611_write(pdata,0x1b,0x69); //Timer 2
	lt9611_write(pdata,0x1c,0x78);
	lt9611_write(pdata,0xcb,0x69); //Timer 1
	lt9611_write(pdata,0xcc,0x78);
	/*power consumption for work*/
	lt9611_write(pdata,0xff,0x80);
	lt9611_write(pdata,0x04,0xf0);
	lt9611_write(pdata,0x06,0xf0);
	lt9611_write(pdata,0x0a,0x80);
	lt9611_write(pdata,0x0b,0x40);
	lt9611_write(pdata,0x0d,0xef);
	lt9611_write(pdata,0x11,0xfa);
}

static void LT9611_PLL(struct lt9611 *pdata, struct video_timing *video_format)
{
	u8 pll_lock_flag, i;
	u32 pclk;

	pclk = video_format->pclk_khz;

	lt9611_write(pdata,0xff,0x81);
	lt9611_write(pdata,0x23,0x40);
	lt9611_write(pdata,0x24,0x64);
	lt9611_write(pdata,0x25,0x80);
	lt9611_write(pdata,0x26,0x55);
	lt9611_write(pdata,0x2c,0x37);

	lt9611_write(pdata,0x2f,0x01);
	lt9611_write(pdata,0x26,0x55);
	lt9611_write(pdata,0x27,0x66);
	lt9611_write(pdata,0x28,0x88);
	if(pclk > 150000) {
		lt9611_write(pdata,0x2d,0x88);
	} else if (pclk > 70000) {
		lt9611_write(pdata,0x2d,0x99);
	} else {
		lt9611_write(pdata,0x2d,0xaa);
	}
	pclk = pclk / 2;
	lt9611_write(pdata,0xff,0x82);
	lt9611_write(pdata,0xe3,pclk / 65536);
	pclk = pclk % 65536;
	lt9611_write(pdata,0xe4,pclk / 256);
	lt9611_write(pdata,0xe5,pclk % 256);
	lt9611_write(pdata,0xde,0x20);
	lt9611_write(pdata,0xde,0xe0);
	lt9611_write(pdata,0xff,0x80);
	lt9611_write(pdata,0x11,0x5a);
	lt9611_write(pdata,0x11,0xfa);
	lt9611_write(pdata,0x18,0xdc);
	lt9611_write(pdata,0x18,0xfc);
	lt9611_write(pdata,0x16,0xf1);
	lt9611_write(pdata,0x16,0xf3);
	/* pll lock status */
	for(i = 0; i < 6 ; i++) {
		lt9611_write(pdata,0xff,0x80);
		lt9611_write(pdata,0x16,0xe3); /* pll lock logic reset */
		lt9611_write(pdata,0x16,0xf3);
		lt9611_write(pdata,0xff,0x82);
		lt9611_read(pdata, 0x15, &pll_lock_flag, 1);
		if (pll_lock_flag & 0x80) {
			break;
		} else {
			lt9611_write(pdata,0xff,0x80);
			lt9611_write(pdata,0x11,0x5a); /* Pcr clk reset */
			lt9611_write(pdata,0x11,0xfa);
			lt9611_write(pdata,0x18,0xdc); /* pll analog reset */
			lt9611_write(pdata,0x18,0xfc);
			lt9611_write(pdata,0x16,0xf1); /* pll cal reset*/
			lt9611_write(pdata,0x16,0xf3);
		}

	}
}


static void LT9611_HDMI_TX_Phy(struct lt9611 *pdata)
{
	lt9611_write(pdata,0xff,0x81);
	lt9611_write(pdata,0x30,0x6a);

	lt9611_write(pdata,0x31,0x73); //DC: 0x44, AC:0x73

	lt9611_write(pdata,0x32,0x4a);
	lt9611_write(pdata,0x33,0x0b);
	lt9611_write(pdata,0x34,0x00);
	lt9611_write(pdata,0x35,0x00);
	lt9611_write(pdata,0x36,0x00);
	lt9611_write(pdata,0x37,0x44);
	lt9611_write(pdata,0x3f,0x0f);
	lt9611_write(pdata,0x40,0xa0);
	lt9611_write(pdata,0x41,0xa0);
	lt9611_write(pdata,0x42,0xa0);
	lt9611_write(pdata,0x43,0xa0);
	lt9611_write(pdata,0x44,0x0a);
}

static void LT9611_HDMI_Out_Enable(struct lt9611 *pdata)
{
	lt9611_write(pdata,0xff,0x81);
	lt9611_write(pdata,0x23,0x40);

	lt9611_write(pdata,0xff,0x82);
	lt9611_write(pdata,0xde,0x20);
	lt9611_write(pdata,0xde,0xe0);

	lt9611_write(pdata,0xff,0x80);
	lt9611_write(pdata,0x18,0xdc); /* txpll sw rst */
	lt9611_write(pdata,0x18,0xfc);
	lt9611_write(pdata,0x16,0xf1); /* txpll calibration rest */
	lt9611_write(pdata,0x16,0xf3);

	lt9611_write(pdata,0x11,0x5a); //Pcr reset
	lt9611_write(pdata,0x11,0xfa);

	lt9611_write(pdata,0xff,0x81);
	lt9611_write(pdata,0x30,0xea);
}

static void LT9611_HDMI_TX_Digital(struct lt9611 *pdata)
{
	lt9611_write(pdata,0xff,0x84);
	lt9611_write(pdata,0x43,0x56-HDMI_VIC-((HDMI_Y<<5)+0x10));   //AVI_PB0
	lt9611_write(pdata,0x44,(HDMI_Y<<5)+0x10); //AVI_PB1
	lt9611_write(pdata,0x47,HDMI_VIC); //AVI_PB1
}

static void LT9611_pattern_gcm(struct lt9611 *pdata, struct video_timing *video_format)
{
	lt9611_write(pdata,0xff,0x82);
	lt9611_write(pdata,0xa3,(u8)((video_format->hs+video_format->hbp)/256));//de_delay
	lt9611_write(pdata,0xa4,(u8)((video_format->hs+video_format->hbp)%256));
	lt9611_write(pdata,0xa5,(u8)((video_format->vs+video_format->vbp)%256));//de_top
	lt9611_write(pdata,0xa6,(u8)(video_format->hact/256));
	lt9611_write(pdata,0xa7,(u8)(video_format->hact%256));//de_cnt
	lt9611_write(pdata,0xa8,(u8)(video_format->vact/256));
	lt9611_write(pdata,0xa9,(u8)(video_format->vact%256));//de_line
	lt9611_write(pdata,0xaa,(u8)(video_format->htotal/256));
	lt9611_write(pdata,0xab,(u8)(video_format->htotal%256));//htotal
	lt9611_write(pdata,0xac,(u8)(video_format->vtotal/256));
	lt9611_write(pdata,0xad,(u8)(video_format->vtotal%256));//vtotal
	lt9611_write(pdata,0xae,(u8)(video_format->hs/256));
	lt9611_write(pdata,0xaf,(u8)(video_format->hs%256));//hvsa
	lt9611_write(pdata,0xb0,(u8)(video_format->vs%256));//vsa
}

static void LT9611_pattern_pixel_clk(struct lt9611 *pdata, struct video_timing *video_format)
{
	u32 pclk;

	pclk = video_format->pclk_khz;

	lt9611_write(pdata,0xff,0x83);
	lt9611_write(pdata,0x2d,0x50);

	if (pclk == 297000) {
		lt9611_write(pdata,0x26,0xb6);
		lt9611_write(pdata,0x27,0xf0);
	} else if (pclk == 148500) {
		lt9611_write(pdata,0x26,0xb7);
	} else if (pclk == 74250) {
		lt9611_write(pdata,0x26,0x9c);
	}

	lt9611_write(pdata,0xff,0x80);
	lt9611_write(pdata,0x11,0x5a); //Pcr reset
	lt9611_write(pdata,0x11,0xfa);
}

static void LT9611_pattern_en(struct lt9611 *pdata)
{
	lt9611_write(pdata,0xff,0x82);
	lt9611_write(pdata,0x4f,0x80);
	lt9611_write(pdata,0x50,0x20);
}

static void LT9611_pattern(struct lt9611 *pdata)
{
	struct video_timing *video;

	video = &video_1920x1080_60Hz;
	//Video_Format = video_3840x2160_30Hz_vic;
	//video = &video_3840x2160_30Hz;
	//video = &video_1024x600_60Hz;
	LT9611_System_Init(pdata);
	LT9611_pattern_en(pdata);

	LT9611_PLL(pdata, video);
	LT9611_pattern_pixel_clk(pdata, video);

	LT9611_pattern_gcm(pdata, video);

	LT9611_HDMI_TX_Digital(pdata);
	LT9611_HDMI_TX_Phy(pdata);
	LT9611_HDMI_Out_Enable(pdata);
}
