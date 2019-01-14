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

static int inv_get_actual_duration(int rate)
{
	int duration_ns;

	if (rate > 500)
		duration_ns = 1000000;
	else if (rate > 200)
		duration_ns = 2000000;
	else if (rate > 100)
		duration_ns = 5000000;
	else if (rate > 50)
		duration_ns = 10000000;
	else if (rate > 25)
		duration_ns = 20000000;
	else if (rate > 12)
		duration_ns = 40000000;
	else if (rate > 6)
		duration_ns = 80000000;
	else if (rate > 3)
		duration_ns = 160000000;
	else
		duration_ns = 320000000;

	return duration_ns;
}

static int inv_calc_engine_dur(struct inv_mpu_state *st,
				struct inv_engine_info *ei)
{
	if (!ei->running_rate)
		return -EINVAL;
	ei->dur = ei->base_time / ei->orig_rate;
	ei->dur *= ei->divider;

	return 0;
}

int inv_turn_off_fifo(struct inv_mpu_state *st)
{
	int res;

	res = inv_plat_single_write(st,
		REG_FIFO_CONFIG_REG, BIT_FIFO_MODE_BYPASS);
	if (res)
		return res;

	return 0;
}

static int inv_turn_on_fifo(struct inv_mpu_state *st)
{
	u8 int0_en, int1_en, fifo_en, smd;
	u8 int6_en;
	uint8_t burst_read[3];
	int r;

	r = inv_plat_single_write(st,
		REG_FIFO_CONFIG_REG, BIT_FIFO_MODE_BYPASS);
	if (r)
		return r;
	r = inv_plat_single_write(st, REG_FIFO_CONFIG1, 0);
	if (r)
		return r;
	r = inv_plat_read(st, REG_FIFO_BYTE_COUNT1, 2, burst_read);
	if (r)
		return r;
	r = inv_plat_read(st, REG_FIFO_DATA_REG, 3, burst_read);
	if (r)
		return r;
	r = inv_plat_single_write(st,
		REG_FIFO_CONFIG_REG, BIT_FIFO_MODE_STREAM);
	if (r)
		return r;
	int0_en = 0;
	int1_en = 0;
	smd = 0;
	if (inv_get_apex_enabled(st)) {
		/* for saving power when apex gesture is enabled */
		smd |= BIT_WOM_INT_MODE_AND |
			BIT_WOM_MODE_PREV |
			BIT_SMD_MODE_OLD;
	}
	if (st->gesture_only_on && (!st->batch.timeout)) {
		if (st->chip_config.stationary_detect_enable)
			st->gesture_int_count = STATIONARY_DELAY_THRESHOLD;
		else
			st->gesture_int_count = WOM_DELAY_THRESHOLD;
		r = inv_set_accel_intel(st);
		if (r)
			return r;
		int1_en |= BIT_INT_WOM_XYZ_INT1_EN;
		smd = BIT_WOM_INT_MODE_AND |
			BIT_WOM_MODE_PREV |
			BIT_SMD_MODE_OLD;
	}
	if (st->smd.on) {
		int1_en |= BIT_INT_SMD_INT1_EN;
		smd |= BIT_WOM_INT_MODE_AND |
			BIT_WOM_MODE_PREV |
			BIT_SMD_MODE_LONG;
	}
	if (st->sensor[SENSOR_GYRO].on ||
		st->sensor[SENSOR_ACCEL].on) {
		if (st->batch.timeout) {
			if (!st->batch.fifo_wm_th)
				int0_en = BIT_INT_UI_DRDY_INT1_EN;
			else
				int0_en = BIT_INT_FIFO_THS_INT1_EN |
				BIT_INT_FIFO_FULL_INT1_EN;
		} else {
			int0_en = BIT_INT_UI_DRDY_INT1_EN;
			if (st->chip_config.eis_enable)
				int0_en |= BIT_INT_UI_FSYNC_INT1_EN;
		}
	}
	fifo_en = 0;
	if (st->sensor[SENSOR_GYRO].on ||
		st->sensor[SENSOR_ACCEL].on) {
		fifo_en |= (BIT_FIFO_ACCEL_EN |
		BIT_FIFO_GYRO_EN |
		BIT_FIFO_WM_TH);
	}
	r = inv_plat_single_write(st, REG_FIFO_CONFIG1, fifo_en);
	if (r)
		return r;
	r = inv_plat_single_write(st, REG_INT_SOURCE0, int0_en);
	if (r)
		return r;
	r = inv_plat_single_write(st, REG_INT_SOURCE1, int1_en);
	if (r)
		return r;
	int6_en = 0;
	if (st->step_detector_l_on ||
		st->step_detector_wake_l_on ||
		st->step_counter_wake_l_on ||
		(st->step_counter_l_on && st->ped.int_mode))
		int6_en |= BIT_INT_STEP_DET_INT1_EN;
	if (st->chip_config.tilt_enable)
		int6_en |= BIT_INT_TILT_DET_INT1_EN;
	if (st->chip_config.tap_enable)
		int6_en |= BIT_INT_TAP_DET_INT1_EN;
	if (st->chip_config.pick_up_enable)
		int6_en |= BIT_INT_RAISE_DET_INT1_EN;
	/* enable apex gesture interrupt */
	if (int6_en) {
		r = inv_plat_single_write(st,
			REG_REG_BANK_SEL, BIT_BANK_SEL_4);
		if (r)
			return r;
		r = inv_plat_single_write(st, REG_INT_SOURCE6, int6_en);
		if (r)
			return r;
		r = inv_plat_single_write(st,
			REG_REG_BANK_SEL, BIT_BANK_SEL_0);
	}
	r = inv_plat_single_write(st, REG_SMD_CONFIG, smd);

	return r;
}

/*
 *  inv_reset_fifo() - Reset FIFO related registers.
 */
int inv_reset_fifo(struct inv_mpu_state *st, bool turn_off)
{
	int r, i;
	struct inv_timestamp_algo *ts_algo = &st->ts_algo;
	int accel_rate, gyro_rate;

	r = inv_turn_on_fifo(st);
	if (r)
		return r;

	ts_algo->last_run_time = get_time_ns();
	ts_algo->reset_ts = ts_algo->last_run_time;

	accel_rate = 1000 / st->eng_info[ENGINE_ACCEL].divider;
	gyro_rate = 1000 / st->eng_info[ENGINE_GYRO].divider;

	if (accel_rate >= 500)
		ts_algo->first_drop_samples[SENSOR_ACCEL] =
			FIRST_DROP_SAMPLES_ACC_500HZ;
	else if (accel_rate >= 200)
		ts_algo->first_drop_samples[SENSOR_ACCEL] =
			FIRST_DROP_SAMPLES_ACC_200HZ;
	else
		ts_algo->first_drop_samples[SENSOR_ACCEL] = 1;

	if (gyro_rate >= 500)
		ts_algo->first_drop_samples[SENSOR_GYRO] =
			FIRST_DROP_SAMPLES_GYR_500HZ;
	else if (gyro_rate >= 200)
		ts_algo->first_drop_samples[SENSOR_GYRO] =
			FIRST_DROP_SAMPLES_GYR_200HZ;
	else
		ts_algo->first_drop_samples[SENSOR_GYRO] = 1;

	st->last_temp_comp_time = ts_algo->last_run_time;
	st->left_over_size = 0;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		st->sensor[i].calib_flag = 0;
		st->sensor[i].sample_calib = 0;
		st->sensor[i].time_calib = ts_algo->last_run_time;
	}

	ts_algo->calib_counter = 0;

	return 0;
}

static int inv_turn_on_engine(struct inv_mpu_state *st)
{
	u8 v, w;
	int r;
	unsigned int wait_ms;
	int accel_rate, gyro_rate;

	accel_rate = 1000 / st->eng_info[ENGINE_ACCEL].divider;
	gyro_rate = 1000 / st->eng_info[ENGINE_GYRO].divider;
	r = inv_plat_read(st, REG_PWR_MGMT_0, 1, &v);
	if (r)
		return r;
	w = v & ~(BIT_GYRO_MODE_LNM|BIT_ACCEL_MODE_LNM);
	if (st->chip_config.gyro_enable) {
		/* gyro support low noise mode only */
		w |= BIT_GYRO_MODE_LNM;
	}
	if (st->chip_config.accel_enable ||
		inv_get_apex_enabled(st)) {
#ifdef SUPPORT_ACCEL_LPM
		if (accel_rate > ACC_LPM_MAX_RATE)
			w |= BIT_ACCEL_MODE_LNM;
		else
			w |= BIT_ACCEL_MODE_LPM;
#else
		w |= BIT_ACCEL_MODE_LNM;
#endif
	}
	r = inv_plat_single_write(st, REG_PWR_MGMT_0, w);
	if (r)
		return r;
	usleep_range(1000, 1001);
	wait_ms = 0;
	if (st->chip_config.gyro_enable && !(v & BIT_GYRO_MODE_LNM))
		wait_ms = INV_ICM42600_GYRO_START_TIME;
	if ((st->chip_config.accel_enable || inv_get_apex_enabled(st)) &&
		!(v & BIT_ACCEL_MODE_LNM)) {
		if (wait_ms < INV_ICM42600_ACCEL_START_TIME)
			wait_ms = INV_ICM42600_ACCEL_START_TIME;
	}
	if (wait_ms)
		msleep(wait_ms);

	if (st->chip_config.has_compass) {
		if (st->chip_config.compass_enable)
			r = st->slave_compass->resume(st);
		else
			r = st->slave_compass->suspend(st);
		if (r)
			return r;
	}

	return 0;
}

static int inv_setup_dmp_rate(struct inv_mpu_state *st)
{
	int i;

	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			st->cntl |= st->sensor[i].output;
			st->sensor[i].dur =
				st->eng_info[st->sensor[i].engine_base].dur;
			st->sensor[i].div = 1;
		}
	}

	return 0;
}

/*
 *  inv_set_lpf() - set low pass filer based on fifo rate.
 */
static int inv_set_lpf(struct inv_mpu_state *st)
{
	u8 ga_cfg0, g_cfg1, a_cfg1;
	int accel_rate, gyro_rate;
	int result = 0;

	ga_cfg0 = 0;
	a_cfg1 = BIT_ACC_UI_FILT_ODR_IND_3 | BIT_ACC_DEC2_M2_ORD_3;
	g_cfg1 = BIT_TEMP_FILT_BW_BYPASS | BIT_GYR_UI_FILT_ORD_IND_3 |
		BIT_GYR_DEC2_M2_ORD_3;
	accel_rate = 1000 / st->eng_info[ENGINE_ACCEL].divider;
	/* filter needs to be matched with HAL rate */
	if (accel_rate < 25) {
		if (st->sensor_l[SENSOR_L_ACCEL].on) {
			accel_rate =
				min(accel_rate,
				st->sensor_l[SENSOR_L_ACCEL].rate);
		}
		if (st->sensor_l[SENSOR_L_ACCEL_WAKE].on) {
			accel_rate =
				min(accel_rate,
				st->sensor_l[SENSOR_L_ACCEL_WAKE].rate);
		}
	}
	gyro_rate = 1000 / st->eng_info[ENGINE_GYRO].divider;
#ifdef SUPPORT_ACCEL_LPM
	if (accel_rate > ACC_LPM_MAX_RATE) {
		ga_cfg0 |= BIT_ACCEL_UI_LNM_BW_2_FIR;
	} else {
		if (inv_get_apex_enabled(st) &&
			!st->chip_config.accel_enable &&
			!st->gesture_only_on) {
			/* for saving power when apex gesture is enabled */
			ga_cfg0 |= BIT_ACCEL_UI_LPM_AVG_1;
			a_cfg1 |= BIT_ACC_AVG_FLT_RATE_8KHZ;
		} else if (accel_rate <= 6) {
			ga_cfg0 |= BIT_ACCEL_UI_LPM_AVG_128;
			a_cfg1 |= BIT_ACC_AVG_FLT_RATE_1KHZ;
		} else if (accel_rate <= 12) {
			ga_cfg0 |= BIT_ACCEL_UI_LPM_AVG_64;
			a_cfg1 |= BIT_ACC_AVG_FLT_RATE_1KHZ;
		} else if (accel_rate <= 25) {
			ga_cfg0 |= BIT_ACCEL_UI_LPM_AVG_32;
			a_cfg1 |= BIT_ACC_AVG_FLT_RATE_1KHZ;
		} else if (accel_rate <= 50) {
			ga_cfg0 |= BIT_ACCEL_UI_LPM_AVG_128;
			a_cfg1 |= BIT_ACC_AVG_FLT_RATE_8KHZ;
		} else if (accel_rate <= 100) {
			ga_cfg0 |= BIT_ACCEL_UI_LPM_AVG_64;
			a_cfg1 |= BIT_ACC_AVG_FLT_RATE_8KHZ;
		} else if (accel_rate <= 200) {
			ga_cfg0 |= BIT_ACCEL_UI_LPM_AVG_16;
			a_cfg1 |= BIT_ACC_AVG_FLT_RATE_8KHZ;
		} else if (accel_rate <= 500) {
			ga_cfg0 |= BIT_ACCEL_UI_LPM_AVG_4;
			a_cfg1 |= BIT_ACC_AVG_FLT_RATE_8KHZ;
		} else {
			ga_cfg0 |= BIT_ACCEL_UI_LPM_AVG_1;
			a_cfg1 |= BIT_ACC_AVG_FLT_RATE_8KHZ;
		}
	}
#else /* SUPPORT_ACCEL_LPM */
	ga_cfg0 |= BIT_ACCEL_UI_LNM_BW_2_FIR;
#endif
	ga_cfg0 |= BIT_GYRO_UI_LNM_BW_2_FIR;
	result = inv_plat_single_write(st, REG_GYRO_CONFIG1, g_cfg1);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_ACCEL_CONFIG1, a_cfg1);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_GYRO_ACCEL_CONFIG0, ga_cfg0);

	return result;
}

static int inv_set_div(struct inv_mpu_state *st, int a_d, int g_d)
{
	u8 databuf;
	int result;

	result = inv_plat_read(st, REG_ACCEL_CONFIG0, 1, &databuf);
	if (result)
		return result;
	databuf &= ~BIT_ACCEL_ODR;
	if ((1000 / (a_d + 1)) > 500)
		databuf |= BIT_ACCEL_ODR_1000;
	else if ((1000 / (a_d + 1)) > 200)
		databuf |= BIT_ACCEL_ODR_500;
	else if ((1000 / (a_d + 1)) > 100)
		databuf |= BIT_ACCEL_ODR_200;
	else if ((1000 / (a_d + 1)) > 50)
		databuf |= BIT_ACCEL_ODR_100;
	else if ((1000 / (a_d + 1)) > 25)
		databuf |= BIT_ACCEL_ODR_50;
	else if ((1000 / (a_d + 1)) > 12)
		databuf |= BIT_ACCEL_ODR_25;
	else if ((1000 / (a_d + 1)) > 6)
		databuf |= BIT_ACCEL_ODR_12;
	else {
#ifdef SUPPORT_ACCEL_LPM
		/* low power mode */
		if ((1000 / (a_d + 1)) > 3)
			databuf |= BIT_ACCEL_ODR_6;
		else
			databuf |= BIT_ACCEL_ODR_3;
#else
		/* low noise mode */
		databuf |= BIT_ACCEL_ODR_12;
#endif
	}
	result = inv_plat_single_write(st, REG_ACCEL_CONFIG0, databuf);
	if (result)
		return result;
	result = inv_plat_read(st, REG_GYRO_CONFIG0, 1, &databuf);
	if (result)
		return result;
	databuf &= ~BIT_GYRO_ODR;
	if ((1000 / (g_d + 1)) > 500)
		databuf |= BIT_GYRO_ODR_1000;
	else if ((1000 / (g_d + 1)) > 200)
		databuf |= BIT_GYRO_ODR_500;
	else if ((1000 / (g_d + 1)) > 100)
		databuf |= BIT_GYRO_ODR_200;
	else if ((1000 / (g_d + 1)) > 50)
		databuf |= BIT_GYRO_ODR_100;
	else if ((1000 / (g_d + 1)) > 25)
		databuf |= BIT_GYRO_ODR_50;
	else if ((1000 / (g_d + 1)) > 12)
		databuf |= BIT_GYRO_ODR_25;
	else
		databuf |= BIT_GYRO_ODR_12;
	result = inv_plat_single_write(st, REG_GYRO_CONFIG0, databuf);
	if (result)
		return result;

	return result;
}

static int inv_set_batch(struct inv_mpu_state *st)
{
	int res = 0;
	u32 w;
	u32 running_rate;

	if (st->sensor[SENSOR_ACCEL].on || st->sensor[SENSOR_GYRO].on)
		st->batch.pk_size = 16;
	else
		st->batch.pk_size = 0;
	if (st->sensor[SENSOR_GYRO].on && !st->sensor[SENSOR_ACCEL].on)
		running_rate = st->eng_info[ENGINE_GYRO].running_rate;
	else if (!st->sensor[SENSOR_GYRO].on && st->sensor[SENSOR_ACCEL].on)
		running_rate = st->eng_info[ENGINE_ACCEL].running_rate;
	else
		running_rate = st->eng_info[ENGINE_GYRO].running_rate;
	if (st->batch.timeout) {
		w = st->batch.timeout * running_rate
					* st->batch.pk_size / 1000;
		if (w > MAX_BATCH_FIFO_SIZE)
			w = MAX_BATCH_FIFO_SIZE;
	} else {
		w = 0;
	}
	st->batch.fifo_wm_th = w;
	pr_debug("running= %d, pksize=%d, to=%d w=%d\n",
		running_rate, st->batch.pk_size, st->batch.timeout, w);

	res = inv_plat_single_write(st, REG_FIFO_CONFIG2, w & 0xff);
	if (res)
		return res;
	res = inv_plat_single_write(st, REG_FIFO_CONFIG3, (w >> 8) & 0xff);

	return res;
}

static int inv_set_rate(struct inv_mpu_state *st)
{
	int g_d, a_d, result;

	result = inv_setup_dmp_rate(st);
	if (result)
		return result;

	g_d = st->eng_info[ENGINE_GYRO].divider - 1;
	a_d = st->eng_info[ENGINE_ACCEL].divider - 1;
	result = inv_set_div(st, a_d, g_d);
	if (result)
		return result;
	result = inv_set_lpf(st);
	if (result)
		return result;

	inv_set_batch(st);

	return result;
}

static int inv_enable_apex_gestures(struct inv_mpu_state *st)
{
	int result;
	u8 w, r;
	unsigned int wait_ms = 0;

	result = inv_plat_read(st, REG_APEX_CONFIG0, 1, &r);
	if (result)
		return result;

#ifdef NOT_SET_DMP_POWER_SAVE
	w = BIT_DMP_ODR_50HZ;
#else
	w = BIT_DMP_POWER_SAVE_EN | BIT_DMP_ODR_50HZ;
#endif
	if (st->step_detector_l_on ||
		st->step_detector_wake_l_on ||
		st->step_counter_l_on ||
		st->step_counter_wake_l_on)
		w |= BIT_PEDO_ENABLE;
	if (st->chip_config.tilt_enable)
		w |= BIT_TILT_ENABLE;
	if (st->chip_config.tap_enable)
		w |= BIT_TAP_ENABLE;
	if (st->chip_config.pick_up_enable)
		w |= BIT_RAISE_ENABLE;

	if (r != w) {
		if (!(r & BIT_PEDO_ENABLE) && (w & BIT_PEDO_ENABLE))
			wait_ms = 50;
		if (!(r & (BIT_PEDO_ENABLE | BIT_TILT_ENABLE | BIT_RAISE_ENABLE)) &&
				(w & (BIT_PEDO_ENABLE | BIT_TILT_ENABLE | BIT_RAISE_ENABLE))) {
			result = inv_plat_single_write(st, REG_SIGNAL_PATH_RESET, BIT_DMP_INIT_EN);
			if (result)
				return result;
			msleep(50);
			result = inv_plat_single_write(st, REG_APEX_CONFIG0, w);
			if (result)
				return result;
			if (wait_ms)
				msleep(wait_ms);
		} else if (!(w & (BIT_PEDO_ENABLE | BIT_TILT_ENABLE | BIT_RAISE_ENABLE))) {
			result = inv_plat_single_write(st, REG_APEX_CONFIG0, w);
			if (result)
				return result;
			result = inv_plat_single_write(st, REG_SIGNAL_PATH_RESET,
				BIT_DMP_MEM_RESET_EN);
			if (result)
				return result;
			usleep_range(1000, 1001);
		} else {
			result = inv_plat_single_write(st, REG_APEX_CONFIG0, w);
			if (result)
				return result;
			if (wait_ms)
				msleep(wait_ms);
		}

		if (!(r & BIT_PEDO_ENABLE) && (w & BIT_PEDO_ENABLE))
			st->apex_data.step_reset_last_val = true;
		else
			st->apex_data.step_reset_last_val = false;
	}

	return result;
}

static int inv_determine_engine(struct inv_mpu_state *st)
{
	int i;
	bool a_en, g_en;
	int accel_rate, gyro_rate;

	a_en = false;
	g_en = false;
	gyro_rate = MPU_INIT_SENSOR_RATE_LNM;
#ifdef SUPPORT_ACCEL_LPM
	accel_rate = MPU_INIT_SENSOR_RATE_LPM;
#else
	accel_rate = MPU_INIT_SENSOR_RATE_LNM;
#endif
	/*
	 * loop the streaming sensors to see which engine needs to be turned on
	 */
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			a_en |= st->sensor[i].a_en;
			g_en |= st->sensor[i].g_en;
		}
	}

	if (st->chip_config.eis_enable) {
		g_en = true;
		st->eis.frame_count = 0;
		st->eis.fsync_delay = 0;
		st->eis.gyro_counter = 0;
		st->eis.voting_count = 0;
		st->eis.voting_count_sub = 0;
		gyro_rate = BASE_SAMPLE_RATE;
	} else {
		st->eis.eis_triggered = false;
		st->eis.prev_state = false;
	}

	accel_rate = st->sensor[SENSOR_ACCEL].rate;
	gyro_rate  = max(gyro_rate, st->sensor[SENSOR_GYRO].rate);

	if (g_en)
		st->ts_algo.clock_base = ENGINE_GYRO;
	else
		st->ts_algo.clock_base = ENGINE_ACCEL;

	st->eng_info[ENGINE_GYRO].running_rate = gyro_rate;
	st->eng_info[ENGINE_ACCEL].running_rate = accel_rate;

	/* engine divider for pressure and compass is set later */
	if (st->chip_config.eis_enable) {
		st->eng_info[ENGINE_GYRO].divider = 1;
		st->eng_info[ENGINE_ACCEL].divider = 1;
		/* need to update rate and div for 1khz mode */
		for (i = 0 ; i < SENSOR_L_NUM_MAX ; i++) {
			if (st->sensor_l[i].on) {
				st->sensor_l[i].counter = 0;
				if (st->sensor_l[i].rate)
					st->sensor_l[i].div =
					    ((BASE_SAMPLE_RATE /
						 st->eng_info[ENGINE_GYRO].divider) /
						 st->sensor_l[i].rate);
				else
					st->sensor_l[i].div = 0xffff;
			}
		}
	} else {
		st->eng_info[ENGINE_GYRO].divider =
			inv_get_actual_duration(st->eng_info[ENGINE_GYRO].running_rate) / 1000000;
		st->eng_info[ENGINE_ACCEL].divider =
			inv_get_actual_duration(st->eng_info[ENGINE_ACCEL].running_rate) / 1000000;
	}

	for (i = 0 ; i < SENSOR_L_NUM_MAX ; i++)
		st->sensor_l[i].counter = 0;

	inv_calc_engine_dur(st, &st->eng_info[ENGINE_GYRO]);
	inv_calc_engine_dur(st, &st->eng_info[ENGINE_ACCEL]);

	pr_debug("gen: %d aen: %d grate: %d arate: %d\n",
				g_en, a_en, gyro_rate, accel_rate);

	st->chip_config.gyro_enable = g_en;
	st->chip_config.accel_enable = a_en;

	return 0;
}

/*
 *  set_inv_enable() - enable function.
 */
int set_inv_enable(struct iio_dev *indio_dev)
{
	int result;
	struct inv_mpu_state *st = iio_priv(indio_dev);
	u8 w;

	inv_stop_interrupt(st);
	inv_turn_off_fifo(st);
	inv_determine_engine(st);
	result = inv_set_rate(st);
	if (result) {
		pr_err("inv_set_rate error\n");
		return result;
	}
	/* enable chip timestamp */
	w = BIT_EN_DREG_FIFO_D2A |
		BIT_TMST_TO_REGS_EN |
		BIT_TMST_EN;
	result = inv_plat_single_write(st, REG_TMST_CONFIG, w);
	if (result)
		return result;

	w = BIT_GYRO_AFSR_MODE_HFS |
		BIT_ACCEL_AFSR_MODE_HFS |
		BIT_CLK_SEL_PLL;
	if (st->chip_config.accel_enable && st->chip_config.gyro_enable)
		w |= BIT_ACCEL_LP_CLK_SEL;
	result = inv_plat_single_write(st, REG_INTF_CONFIG1, w);
	if (result)
		return result;
	if (w & BIT_ACCEL_LP_CLK_SEL)
		msleep(st->eng_info[ENGINE_ACCEL].divider);
	result = inv_turn_on_engine(st);
	if (result) {
		pr_err("inv_turn_on_engine error\n");
		return result;
	}
	result = inv_enable_apex_gestures(st);
	if (result) {
		pr_err("inv_enable_apex_gestures error\n");
		return result;
	}
	result = inv_reset_fifo(st, false);
	if (result)
		return result;
	if ((!st->chip_config.gyro_enable) &&
		(!st->chip_config.accel_enable) &&
		(!inv_get_apex_enabled(st))) {
		inv_set_power(st, false);
		return 0;
	}

	return result;
}
/* dummy function for 20608D */
int inv_enable_pedometer_interrupt(struct inv_mpu_state *st, bool en)
{
	return 0;
}
int inv_dmp_read(struct inv_mpu_state *st, int off, int size, u8 *buf)
{
	return 0;
}
int inv_firmware_load(struct inv_mpu_state *st)
{
	return 0;
}
