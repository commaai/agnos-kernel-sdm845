/*
 * Copyright (C) 2018-2018 InvenSense, Inc.
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

/**
 * inv_get_apex_enabled() - Check if any APEX feature is enabled
 * @st: struct inv_mpu_state.
 *
 * Return: true when any is enabled, otherwise false.
 */
bool inv_get_apex_enabled(struct inv_mpu_state *st)
{
	if (st->step_detector_l_on ||
		st->step_detector_wake_l_on ||
		st->step_counter_l_on ||
		st->step_counter_wake_l_on)
		return true;
	if (st->chip_config.tilt_enable)
		return true;
	if (st->chip_config.tap_enable)
		return true;
	if (st->chip_config.pick_up_enable)
		return true;
	if (st->smd.on)
		return true;

	return false;
}

/**
 * inv_get_apex_odr() - Get min accel ODR according to enabled APEX feature
 * @st: struct inv_mpu_state.
 *
 * Return: min accel ODR in Hz
 */
int inv_get_apex_odr(struct inv_mpu_state *st)
{
	int odr_hz;

#ifdef SUPPORT_ACCEL_LPM
	odr_hz = MPU_INIT_SENSOR_RATE_LPM;
#else
	odr_hz = MPU_INIT_SENSOR_RATE_LNM;
#endif
	/* returns min accel rate for each algorithm */
	if (st->chip_config.tap_enable)
		odr_hz = 200;
	else if (st->step_detector_l_on ||
		st->step_detector_wake_l_on ||
		st->step_counter_l_on ||
		st->step_counter_wake_l_on ||
		st->chip_config.tilt_enable ||
		st->chip_config.pick_up_enable ||
		st->smd.on)
		odr_hz = 50;

	return odr_hz;
}
