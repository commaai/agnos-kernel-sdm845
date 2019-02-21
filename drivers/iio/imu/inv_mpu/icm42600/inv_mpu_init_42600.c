/*
 * Copyright (C) 2017-2018 InvenSense, Inc.
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
#define pr_fmt(fmt) "inv_mpu: " fmt
#include "../inv_mpu_iio.h"

static int inv_read_timebase(struct inv_mpu_state *st)
{
	st->eng_info[ENGINE_ACCEL].base_time = NSEC_PER_SEC;
	st->eng_info[ENGINE_ACCEL].base_time_1k = NSEC_PER_SEC;
	st->eng_info[ENGINE_ACCEL].base_time_vr = NSEC_PER_SEC;
	/* talor expansion to calculate base time unit */
	st->eng_info[ENGINE_GYRO].base_time = NSEC_PER_SEC;
	st->eng_info[ENGINE_GYRO].base_time_1k = NSEC_PER_SEC;
	st->eng_info[ENGINE_GYRO].base_time_vr = NSEC_PER_SEC;
	st->eng_info[ENGINE_I2C].base_time = NSEC_PER_SEC;
	st->eng_info[ENGINE_I2C].base_time_1k = NSEC_PER_SEC;
	st->eng_info[ENGINE_I2C].base_time_vr = NSEC_PER_SEC;

	st->eng_info[ENGINE_ACCEL].orig_rate = BASE_SAMPLE_RATE;
	st->eng_info[ENGINE_GYRO].orig_rate = BASE_SAMPLE_RATE;
	st->eng_info[ENGINE_I2C].orig_rate = BASE_SAMPLE_RATE;

	return 0;
}

int inv_set_gyro_sf(struct inv_mpu_state *st)
{
	int result;
	u8 data;

	result = inv_plat_read(st, REG_GYRO_CONFIG0, 1, &data);
	if (result)
		return result;
	data &= ~BIT_GYRO_FSR;
	data |= (3 - st->chip_config.fsr) << SHIFT_GYRO_FS_SEL;
	result = inv_plat_single_write(st, REG_GYRO_CONFIG0,
				   data);
	return result;
}

int inv_set_accel_sf(struct inv_mpu_state *st)
{
	int result;
	u8 data;

	result = inv_plat_read(st, REG_ACCEL_CONFIG0, 1, &data);
	if (result)
		return result;
	data &= ~BIT_ACCEL_FSR;
	data |= (3 - st->chip_config.accel_fs) << SHIFT_ACCEL_FS_SEL;
	result = inv_plat_single_write(st, REG_ACCEL_CONFIG0,
			   data);
	return result;
}

int inv_set_accel_intel(struct inv_mpu_state *st)
{
	int result = 0;
	int8_t val, accel_rate;

	result = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_4);
	if (result)
		return result;
	accel_rate = 1000 / st->eng_info[ENGINE_ACCEL].divider;
	if (accel_rate > 50)
		val = WOM_THRESHOLD / (accel_rate / 50);
	else
		val = WOM_THRESHOLD;
	result |= inv_plat_single_write(st, REG_ACCEL_WOM_X_THR, val);
	result |= inv_plat_single_write(st, REG_ACCEL_WOM_Y_THR, val);
	result |= inv_plat_single_write(st, REG_ACCEL_WOM_Z_THR, val);
	result |= inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_0);
	if (result)
		return result;

	return result;
}

int inv_config_apex_gestures(struct inv_mpu_state *st)
{
	int result = 0;
	int8_t rw;
	int8_t mount_matrix = 0;
	int8_t tilt_wait_time = 0x01;
	int8_t pedo_low_energy_amp_th = 0xA;
	int8_t pedo_amp_th_sel = 0x8;
	int8_t pedo_step_cnt_th_sel = 0x5;
	int8_t pedo_hi_enrgy_th_sel = 0x1;
	int8_t pedo_sb_timer_th_sel = 0x4;
	int8_t pedo_step_det_th_sel = 0x2;
	int8_t pedo_sensitivity_mode = 0x0;
	int8_t tap_tmax = 0x2;
	int8_t tap_tmin = 0x3;
	int8_t tap_tavg = 0x3;
	int8_t tap_min_jerk_thr = 0x11;
	int8_t tap_max_peak_tol = 0x1;
	int8_t sleep_time_out = 0x4;
	int8_t sleep_gesture_delay = 0x4;

	/* mount matrix */
	if (st->plat_data.orientation[1] && st->plat_data.orientation[3])
		mount_matrix |= 0x04; /* swap(x,y) */
	if (st->plat_data.orientation[0] < 0 || st->plat_data.orientation[1] < 0)
		mount_matrix |= 0x02; /* x = -x */
	if (st->plat_data.orientation[3] < 0 || st->plat_data.orientation[4] < 0)
		mount_matrix |= 0x01; /* y = -y */

	result = inv_plat_single_write(st, REG_SIGNAL_PATH_RESET,
		BIT_DMP_MEM_RESET_EN);
	if (result)
		return result;
	usleep_range(1000, 1001);

	result = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_4);
	if (result)
		return result;

	/* APEX_CONFIG1 */
	result = inv_plat_read(st, REG_APEX_CONFIG1, 1, &rw);
	if (result)
		goto restore_bank;
	rw &= 0x0f;
	rw |= (pedo_low_energy_amp_th << 4) & 0xf0;
	result = inv_plat_single_write(st, REG_APEX_CONFIG1, rw);
	if (result)
		goto restore_bank;
	/* APEX_CONFIG2 */
	rw = 0x00;
	rw |= (pedo_amp_th_sel << 4) & 0xf0;
	rw |= pedo_step_cnt_th_sel & 0x0f;
	result = inv_plat_single_write(st, REG_APEX_CONFIG2, rw);
	if (result)
		goto restore_bank;
	/* APEX_CONFIG2 */
	rw = 0x00;
	rw |= (pedo_step_det_th_sel << 5) & 0xe0;
	rw |= (pedo_sb_timer_th_sel << 2) & 0x1c;
	rw |= pedo_hi_enrgy_th_sel & 0x03;
	result = inv_plat_single_write(st, REG_APEX_CONFIG3, rw);
	if (result)
		goto restore_bank;
	/* APEX_CONFIG4 */
	result = inv_plat_read(st, REG_APEX_CONFIG4, 1, &rw);
	if (result)
		goto restore_bank;
	rw &= 0x07;
	rw |= (tilt_wait_time << 6) & 0xc0;
	rw |= (sleep_time_out << 3) & 0x38;
	result = inv_plat_single_write(st, REG_APEX_CONFIG4, rw);
	if (result)
		goto restore_bank;
	/* APEX_CONFIG5 */
	result = inv_plat_read(st, REG_APEX_CONFIG5, 1, &rw);
	if (result)
		goto restore_bank;
	rw &= 0xf8;
	rw |= mount_matrix & 0x07;
	result = inv_plat_single_write(st, REG_APEX_CONFIG5, rw);
	if (result)
		goto restore_bank;
	/* APEX_CONFIG6 */
	result = inv_plat_read(st, REG_APEX_CONFIG6, 1, &rw);
	if (result)
		goto restore_bank;
	rw &= 0xf8;
	rw |= sleep_gesture_delay & 0x07;
	result = inv_plat_single_write(st, REG_APEX_CONFIG6, rw);
	if (result)
		goto restore_bank;
	/* APEX_CONFIG7 */
	rw = 0x00;
	rw |= (tap_min_jerk_thr << 2) & 0xfc;
	rw |= tap_max_peak_tol & 0x03;
	result = inv_plat_single_write(st, REG_APEX_CONFIG7, rw);
	if (result)
		goto restore_bank;
	/* APEX_CONFIG8 */
	result = inv_plat_read(st, REG_APEX_CONFIG8, 1, &rw);
	if (result)
		goto restore_bank;
	rw &= 0x80;
	rw |= (tap_tmax << 5) & 0x60;
	rw |= (tap_tmin << 3) & 0x18;
	rw |= tap_tavg & 0x07;
	result = inv_plat_single_write(st, REG_APEX_CONFIG8, rw);
	if (result)
		goto restore_bank;
	/* APEX_CONFIG9 */
	result = inv_plat_read(st, REG_APEX_CONFIG9, 1, &rw);
	if (result)
		goto restore_bank;
	rw &= 0xfe;
	rw |= pedo_sensitivity_mode & 0x01;
	result = inv_plat_single_write(st, REG_APEX_CONFIG9, rw);

restore_bank:
	if (result)
		pr_err("failed to access apex config register\n");

	result |= inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_0);
	if (result)
		return result;
#ifdef NOT_SET_DMP_POWER_SAVE
	result = inv_plat_single_write(st, REG_APEX_CONFIG0,
		BIT_DMP_ODR_50HZ);
#else
	result = inv_plat_single_write(st, REG_APEX_CONFIG0,
		BIT_DMP_ODR_50HZ | BIT_DMP_POWER_SAVE_EN);
#endif

	return result;
}

static void inv_init_sensor_struct(struct inv_mpu_state *st)
{
	int i;

#ifdef SUPPORT_ACCEL_LPM
	for (i = 0; i < SENSOR_NUM_MAX; i++)
		st->sensor[i].rate = MPU_INIT_SENSOR_RATE_LPM;
#else
	for (i = 0; i < SENSOR_NUM_MAX; i++)
		st->sensor[i].rate = MPU_INIT_SENSOR_RATE_LNM;
#endif
	st->sensor[SENSOR_GYRO].rate = MPU_INIT_SENSOR_RATE_LNM;

	st->sensor[SENSOR_ACCEL].sample_size = BYTES_PER_SENSOR;
	st->sensor[SENSOR_TEMP].sample_size = BYTES_FOR_TEMP;
	st->sensor[SENSOR_GYRO].sample_size = BYTES_PER_SENSOR;

	st->sensor_l[SENSOR_L_SIXQ].base = SENSOR_GYRO;
	st->sensor_l[SENSOR_L_PEDQ].base = SENSOR_GYRO;

	st->sensor_l[SENSOR_L_SIXQ_WAKE].base = SENSOR_GYRO;
	st->sensor_l[SENSOR_L_PEDQ_WAKE].base = SENSOR_GYRO;

	st->sensor[SENSOR_ACCEL].a_en = true;
	st->sensor[SENSOR_GYRO].a_en = false;

	st->sensor[SENSOR_ACCEL].g_en = false;
	st->sensor[SENSOR_GYRO].g_en = true;

	st->sensor[SENSOR_ACCEL].c_en = false;
	st->sensor[SENSOR_GYRO].c_en = false;

	st->sensor[SENSOR_ACCEL].p_en = false;
	st->sensor[SENSOR_GYRO].p_en = false;

	st->sensor[SENSOR_ACCEL].engine_base = ENGINE_ACCEL;
	st->sensor[SENSOR_GYRO].engine_base = ENGINE_GYRO;

	st->sensor_l[SENSOR_L_ACCEL].base = SENSOR_ACCEL;
	st->sensor_l[SENSOR_L_GESTURE_ACCEL].base = SENSOR_ACCEL;
	st->sensor_l[SENSOR_L_GYRO].base = SENSOR_GYRO;
	st->sensor_l[SENSOR_L_GYRO_CAL].base = SENSOR_GYRO;
	st->sensor_l[SENSOR_L_EIS_GYRO].base = SENSOR_GYRO;

	st->sensor_l[SENSOR_L_ACCEL_WAKE].base = SENSOR_ACCEL;
	st->sensor_l[SENSOR_L_GYRO_WAKE].base = SENSOR_GYRO;

	st->sensor_l[SENSOR_L_GYRO_CAL_WAKE].base = SENSOR_GYRO;

	st->sensor_l[SENSOR_L_ACCEL].header = ACCEL_HDR;
	st->sensor_l[SENSOR_L_GESTURE_ACCEL].header = ACCEL_HDR;
	st->sensor_l[SENSOR_L_GYRO].header = GYRO_HDR;
	st->sensor_l[SENSOR_L_GYRO_CAL].header = GYRO_CALIB_HDR;

	st->sensor_l[SENSOR_L_EIS_GYRO].header = EIS_GYRO_HDR;
	st->sensor_l[SENSOR_L_SIXQ].header = SIXQUAT_HDR;
	st->sensor_l[SENSOR_L_THREEQ].header = LPQ_HDR;
	st->sensor_l[SENSOR_L_NINEQ].header = NINEQUAT_HDR;
	st->sensor_l[SENSOR_L_PEDQ].header = PEDQUAT_HDR;

	st->sensor_l[SENSOR_L_ACCEL_WAKE].header = ACCEL_WAKE_HDR;
	st->sensor_l[SENSOR_L_GYRO_WAKE].header = GYRO_WAKE_HDR;
	st->sensor_l[SENSOR_L_GYRO_CAL_WAKE].header = GYRO_CALIB_WAKE_HDR;
	st->sensor_l[SENSOR_L_MAG_WAKE].header = COMPASS_WAKE_HDR;
	st->sensor_l[SENSOR_L_MAG_CAL_WAKE].header = COMPASS_CALIB_WAKE_HDR;
	st->sensor_l[SENSOR_L_SIXQ_WAKE].header = SIXQUAT_WAKE_HDR;
	st->sensor_l[SENSOR_L_NINEQ_WAKE].header = NINEQUAT_WAKE_HDR;
	st->sensor_l[SENSOR_L_PEDQ_WAKE].header = PEDQUAT_WAKE_HDR;

	st->sensor_l[SENSOR_L_ACCEL].wake_on = false;
	st->sensor_l[SENSOR_L_GYRO].wake_on = false;
	st->sensor_l[SENSOR_L_GYRO_CAL].wake_on = false;
	st->sensor_l[SENSOR_L_MAG].wake_on = false;
	st->sensor_l[SENSOR_L_MAG_CAL].wake_on = false;
	st->sensor_l[SENSOR_L_EIS_GYRO].wake_on = false;
	st->sensor_l[SENSOR_L_SIXQ].wake_on = false;
	st->sensor_l[SENSOR_L_NINEQ].wake_on = false;
	st->sensor_l[SENSOR_L_PEDQ].wake_on = false;

	st->sensor_l[SENSOR_L_ACCEL_WAKE].wake_on = true;
	st->sensor_l[SENSOR_L_GYRO_WAKE].wake_on = true;
	st->sensor_l[SENSOR_L_GYRO_CAL_WAKE].wake_on = true;
	st->sensor_l[SENSOR_L_MAG_WAKE].wake_on = true;
	st->sensor_l[SENSOR_L_SIXQ_WAKE].wake_on = true;
	st->sensor_l[SENSOR_L_NINEQ_WAKE].wake_on = true;
	st->sensor_l[SENSOR_L_PEDQ_WAKE].wake_on = true;
}

static int inv_init_config(struct inv_mpu_state *st)
{
	int res, i;

	st->batch.overflow_on = 0;
	st->chip_config.fsr = MPU_INIT_GYRO_SCALE;
	st->chip_config.accel_fs = MPU_INIT_ACCEL_SCALE;
	st->ped.int_thresh = MPU_INIT_PED_INT_THRESH;
	st->ped.step_thresh = MPU_INIT_PED_STEP_THRESH;
	st->chip_config.low_power_gyro_on = 1;
	st->eis.count_precision = NSEC_PER_MSEC;
	st->firmware = 0;
	st->fifo_count_mode = BYTE_MODE;

	st->eng_info[ENGINE_GYRO].base_time = NSEC_PER_SEC;
	st->eng_info[ENGINE_ACCEL].base_time = NSEC_PER_SEC;

	inv_init_sensor_struct(st);
	res = inv_read_timebase(st);
	if (res)
		return res;

	res = inv_set_gyro_sf(st);
	if (res)
		return res;
	res = inv_set_accel_sf(st);
	if (res)
		return res;
	res =  inv_set_accel_intel(st);
	if (res)
		return res;
	res =  inv_config_apex_gestures(st);
	if (res)
		return res;
	for (i = 0; i < SENSOR_NUM_MAX; i++)
		st->sensor[i].ts = 0;

	for (i = 0; i < SENSOR_NUM_MAX; i++)
		st->sensor[i].previous_ts = 0;

	return res;
}

int inv_mpu_initialize(struct inv_mpu_state *st)
{
	u8 v;
	int result;

	/* verify whoami */
	result = inv_plat_read(st, REG_WHO_AM_I, 1, &v);
	if (result)
		return result;
	pr_info("whoami= %x\n", v);
	if (v == 0x00 || v == 0xff)
		return -ENODEV;

	/* reset to make sure previous state are not there */
	result = inv_plat_read(st, REG_CHIP_CONFIG_REG, 1, &v);
	if (result)
		return result;
	v |= BIT_SOFT_RESET;
	result = inv_plat_single_write(st, REG_CHIP_CONFIG_REG, v);
	if (result)
		return result;
	msleep(100);

#ifdef SUPPORT_RTC_MODE
	result |= inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_1);
	result |= inv_plat_single_write(st, REG_INTF_CONFIG5, BIT_PIN9_FUNC_CLKIN);
	result |= inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_0);
	if (result)
		return result;
#endif

	v = BIT_GYRO_AFSR_MODE_HFS | BIT_ACCEL_AFSR_MODE_HFS | BIT_CLK_SEL_PLL;
#ifdef SUPPORT_RTC_MODE
	v |= BIT_RTC_MODE;
#endif

	result = inv_plat_single_write(st, REG_INTF_CONFIG1, v);
	if (result)
		return result;

	/* enable chip timestamp */
	v = BIT_EN_DREG_FIFO_D2A |
		BIT_TMST_TO_REGS_EN |
		BIT_TMST_EN;
	result = inv_plat_single_write(st, REG_TMST_CONFIG, v);
	if (result)
		return result;

	result = inv_plat_read(st, REG_INTF_CONFIG0, 1, &v);
	if (result)
		return result;
	v |= st->i2c_dis;
	result = inv_plat_single_write(st, REG_INTF_CONFIG0, v);
	if (result)
		return result;

	v = (INT_POLARITY << SHIFT_INT1_POLARITY) |
		(INT_DRIVE_CIRCUIT << SHIFT_INT1_DRIVE_CIRCUIT) |
		(INT_MODE << SHIFT_INT1_MODE);
	result = inv_plat_single_write(st, REG_INT_CONFIG_REG, v);
	if (result)
		return result;

	result = inv_plat_single_write(st, REG_FIFO_CONFIG_REG,
		BIT_FIFO_MODE_BYPASS);
	if (result)
		return result;

	result = inv_plat_single_write(st, REG_INT_CONFIG1,
		BIT_INT_ASY_RST_DISABLE);
	if (result)
		return result;

	result = inv_plat_single_write(st, REG_PWR_MGMT_0, 0);
	if (result)
		return result;

	result = inv_init_config(st);
	if (result)
		return result;

	st->chip_config.lp_en_mode_off = 0;

	result = inv_set_power(st, false);

	pr_info("%s: initialize result is %d....\n", __func__, result);
	return 0;
}
