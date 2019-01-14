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

static u8 reg_backup[128];
static u8 reg_backup_offset[9];
static u8 reg_backup_int6;

#define DEF_ST_ACCEL_RESULT_SHIFT   1
#define SELF_TEST_ODR               6   /* 1000Hz */
#define SELF_TEST_ODR_LP            7   /* 200Hz */
#define SELF_TEST_ACC_FS            3   /* +-2g */
#define SELF_TEST_GYR_FS            3   /* +-250dps */
#define SELF_TEST_ACC_BW_IND        BIT_ACCEL_UI_LNM_BW_10_IIR
#define SELF_TEST_ACC_LPM_AVG       BIT_ACCEL_UI_LPM_AVG_16
#define SELF_TEST_GYR_BW_IND        BIT_GYRO_UI_LNM_BW_10_IIR
#define SELF_TEST_GYR_LPM_AVG       BIT_GYRO_UI_LPM_AVG_16
#define SELF_TEST_PRECISION         1000
#define SELF_TEST_SAMPLE_NB         200
#define SELF_TEST_MIN_ACC_MG        225 /* mg */
#define SELF_TEST_MAX_ACC_MG        675 /* mg */
#define SELF_TEST_MIN_GYR_DPS       60  /* dps */
#define SELF_TEST_MAX_GYR_OFF_DPS   20  /* dps */
#define SELF_TEST_ACC_SHIFT_DELTA   500 /* = 0.5 */
#define SELF_TEST_GYR_SHIFT_DELTA   500 /* = 0.5 */
#define SELF_TEST_ACC_SCALE         (32768 / (16 >> SELF_TEST_ACC_FS) / 1000)
#define SELF_TEST_GYR_SCALE         (32768 / (2000 >> SELF_TEST_GYR_FS))
#define SELF_TEST_MIN_ACC \
	(SELF_TEST_MIN_ACC_MG * SELF_TEST_ACC_SCALE * SELF_TEST_PRECISION)
#define SELF_TEST_MAX_ACC \
	(SELF_TEST_MAX_ACC_MG * SELF_TEST_ACC_SCALE * SELF_TEST_PRECISION)
#define SELF_TEST_MIN_GYR \
	(SELF_TEST_MIN_GYR_DPS * SELF_TEST_GYR_SCALE * SELF_TEST_PRECISION)
#define SELF_TEST_MAX_GYR_OFFSET \
	(SELF_TEST_MAX_GYR_OFF_DPS * SELF_TEST_GYR_SCALE * SELF_TEST_PRECISION)

#define SELF_TEST_LN_MODE 0
#define SELF_TEST_LP_MODE 1
#define SELF_TEST_ACC_POWER_MODE	SELF_TEST_LP_MODE /* low power mode is default */
#define SELF_TEST_GYRO_POWER_MODE	SELF_TEST_LN_MODE /* low noise mode is default */

static const uint16_t SelfTestEquation[256] = {
	2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
	2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
	3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
	3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
	3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
	3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
	4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
	4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
	4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
	5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
	5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
	6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
	6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
	7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
	7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
	8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
	9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
	10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
	10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
	11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
	12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
	13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
	15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
	16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
	17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
	19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
	20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
	22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
	24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
	26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
	28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
	30903, 31212, 31524, 31839, 32157, 32479, 32804
};

static int inv_save_setting(struct inv_mpu_state *st)
{
	int result, i;

	/* bank 0 */
	for (i = 0; i <= REG_SELF_TEST_CONFIG; i++) {
		if ((i == REG_MEM_R_W) || (i == REG_FIFO_DATA_REG)) {
			reg_backup[i] = 0;
		} else {
			result = inv_plat_read(st, i, 1, &reg_backup[i]);
			if (result)
				return result;
		}
	}

	/* bank 4 */
	result = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_4);
	if (result)
		return result;
	for (i = REG_GOS_USER0 ; i <= REG_GOS_USER8; i++) {
		result = inv_plat_read(st, i, 1,
				&reg_backup_offset[i - REG_GOS_USER0]);
		if (result)
			goto restore_bank;
	}
	result = inv_plat_read(st, REG_INT_SOURCE6, 1, &reg_backup_int6);
	if (result)
		goto restore_bank;

restore_bank:
	result |= inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_0);
	return result;
}

static int inv_recover_setting(struct inv_mpu_state *st)
{
	int result, i;

	/* bank 0 */
	for (i = 0; i <= REG_SELF_TEST_CONFIG; i++) {
		if (i == REG_CHIP_CONFIG_REG ||
			(i >= REG_TEMP_DATA0_UI && i <= REG_SIGNAL_PATH_RESET))
			continue;
		result = inv_plat_single_write(st, i, reg_backup[i]);
		if (result)
			return result;
	}

	/* bank 4 */
	result = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_4);
	if (result)
		return result;
	for (i = REG_GOS_USER0 ; i <= REG_GOS_USER8; i++) {
		result = inv_plat_single_write(st, i,
				reg_backup_offset[i - REG_GOS_USER0]);
		if (result)
			goto restore_bank;
	}
	result = inv_plat_single_write(st, REG_INT_SOURCE6, reg_backup_int6);
	if (result)
		goto restore_bank;

restore_bank:
	result |= inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_0);
	return result;
}

static int inv_reset_offset_reg(struct inv_mpu_state *st)
{
	int result = 0;
	u8 databuf;

	result = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_4);
	if (result)
		return result;
	databuf = 0x00;
	result = inv_plat_single_write(st, REG_GOS_USER0, databuf);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_GOS_USER1, databuf);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_GOS_USER2, databuf);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_GOS_USER3, databuf);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_GOS_USER4, databuf);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_GOS_USER5, databuf);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_GOS_USER6, databuf);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_GOS_USER7, databuf);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_GOS_USER8, databuf);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_0);
	if (result)
		return result;
	return result;
}

static int  inv_init_selftest(struct inv_mpu_state *st,
	int sensor, int mode, int lp_mode)
{
	int result = 0;
	u8 databuf;

	/* Disable Interrupt */
	result = inv_plat_single_write(st, REG_INT_SOURCE0, 0);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_INT_SOURCE1, 0);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_4);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_INT_SOURCE6, 0);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_0);
	if (result)
		return result;
	/* reset offset registers */
	result = inv_reset_offset_reg(st);
	if (result)
		return result;
	/* stop FIFO */
	result = inv_plat_single_write(st, REG_FIFO_CONFIG_REG, 0);
	if (result)
		return result;
	result = inv_plat_single_write(st, REG_FIFO_CONFIG1, 0);
	if (result)
		return result;
	/* self-test mode set */
	databuf = 0;
	if (mode) {
		if (sensor == SENSOR_ACCEL) {
			databuf |= (BIT_TEST_AX_EN |
				BIT_TEST_AY_EN |
				BIT_TEST_AZ_EN);
			databuf |= BIT_SELF_TEST_REGULATOR_EN;
		} else if (sensor == SENSOR_GYRO) {
			databuf |= (BIT_TEST_GX_EN |
				BIT_TEST_GY_EN |
				BIT_TEST_GZ_EN);
		}
	}
	result = inv_plat_single_write(st, REG_SELF_TEST_CONFIG, databuf);
	if (result)
		return result;
	/* set rate */
	if (lp_mode) {
		result = inv_plat_single_write(st, REG_GYRO_CONFIG0,
			(SELF_TEST_GYR_FS << SHIFT_GYRO_FS_SEL) |
			(SELF_TEST_ODR_LP << SHIFT_ODR_CONF));
		if (result)
			return result;
		result = inv_plat_single_write(st, REG_ACCEL_CONFIG0,
			(SELF_TEST_ACC_FS << SHIFT_ACCEL_FS_SEL) |
			(SELF_TEST_ODR_LP << SHIFT_ODR_CONF));
		if (result)
			return result;
	} else {
		result = inv_plat_single_write(st, REG_GYRO_CONFIG0,
			(SELF_TEST_GYR_FS << SHIFT_GYRO_FS_SEL) |
			(SELF_TEST_ODR << SHIFT_ODR_CONF));
		if (result)
			return result;
		result = inv_plat_single_write(st, REG_ACCEL_CONFIG0,
			(SELF_TEST_ACC_FS << SHIFT_ACCEL_FS_SEL) |
			(SELF_TEST_ODR << SHIFT_ODR_CONF));
		if (result)
			return result;
	}
	/* set filter */
	databuf = 0;
	if (sensor == SENSOR_ACCEL) {
		if (lp_mode) {
			result = inv_plat_single_write(st, REG_ACCEL_CONFIG1, 0x05);
			if (result)
				return result;
			databuf |= SELF_TEST_ACC_LPM_AVG;
		} else {
			result = inv_plat_single_write(st, REG_ACCEL_CONFIG1, 0x10);
			if (result)
				return result;
			databuf |= SELF_TEST_ACC_BW_IND;
		}
	} else if (sensor == SENSOR_GYRO) {
		if (lp_mode) {
			result = inv_plat_single_write(st, REG_GYRO_CONFIG1, 0x02);
			if (result)
				return result;
			databuf |= SELF_TEST_GYR_LPM_AVG;
		} else {
			result = inv_plat_single_write(st, REG_GYRO_CONFIG1, 0x08);
			if (result)
				return result;
			databuf |= SELF_TEST_GYR_BW_IND;
		}
	}
	result = inv_plat_single_write(st, REG_GYRO_ACCEL_CONFIG0, databuf);
	if (result)
		return result;
	/* turn on sensors */
	databuf = 0;
	if (sensor == SENSOR_ACCEL) {
		if (lp_mode)
			databuf |= BIT_ACCEL_MODE_LPM;
		else
			databuf |= BIT_ACCEL_MODE_LNM;
	} else if (sensor == SENSOR_GYRO) {
		if (lp_mode)
			databuf |= BIT_GYRO_MODE_LPM;
		else
			databuf |= BIT_GYRO_MODE_LNM;
	}
	result = inv_plat_single_write(st, REG_PWR_MGMT_0, databuf);
	if (result)
		return result;
	msleep(200);
	return result;
}

static int inv_accel_calc_avg_with_samples(struct inv_mpu_state *st,
	int avg[3], int count, int lp_mode)
{
	int result = 0;
	int i, nAxis;
	u8 sensor_data[6];
	s32 sum[3] = { 0, 0, 0 };

	for (i = 0; i < count; i++) {
		result = inv_plat_read(st, REG_ACCEL_DATA_X0_UI,
			6, sensor_data);
		if (result)
			return result;
		/*	convert 8-bit to 16-bit */
		for (nAxis = 0; nAxis < 3; nAxis++) {
			sum[nAxis] +=
				(s16) be16_to_cpup((__be16 *) (&sensor_data[nAxis*2]));
		}
		if (lp_mode) {
			/* data register updated @200hz */
			usleep_range(5000, 5001);
		} else {
			/* data register updated @1khz */
			usleep_range(1000, 1001);
		}
	}
	for (nAxis = 0; nAxis < 3; nAxis++)
		avg[nAxis] = (int)(sum[nAxis] / count * SELF_TEST_PRECISION);
	return result;
}

static int inv_gyro_calc_avg_with_samples(struct inv_mpu_state *st,
	int avg[3], int count, int lp_mode)
{
	int result = 0;
	int i, nAxis;
	u8 sensor_data[6];
	s32 sum[3] = { 0, 0, 0 };

	for (i = 0; i < count; i++) {
		result = inv_plat_read(st, REG_GYRO_DATA_X0_UI, 6, sensor_data);
		if (result)
			return result;
		/*	convert 8-bit to 16-bit */
		for (nAxis = 0; nAxis < 3; nAxis++) {
			sum[nAxis] +=
				(s16) be16_to_cpup((__be16 *) (&sensor_data[nAxis*2]));
		}
		if (lp_mode) {
			/* data register updated @200hz */
			usleep_range(5000, 5001);
		} else {
			/* data register updated @1khz */
			usleep_range(1000, 1001);
		}
	}
	for (nAxis = 0; nAxis < 3; nAxis++)
		avg[nAxis] = (int)(sum[nAxis] / count * SELF_TEST_PRECISION);
	return result;
}

static uint16_t inv_accel_do_selftest(struct inv_mpu_state *st,
	int *accel_result, int lp_mode)
{
	u16 ret = 0;
	u8 reg_data = 0x0;
	int lsb[3] = {0}, lsb_st[3] = {0}, lsb_lp[3] = {0};
	int st_otp[3];
	u8 otp_st_data_acc[3];
	int pass = true;
	int otp_value_zero = false;
	int ratio;
	int i;
	int st_res;

	ret = inv_init_selftest(st, SENSOR_ACCEL, false, false);
	if (ret)
		return ret;
	ret = inv_accel_calc_avg_with_samples(st, lsb,
		SELF_TEST_SAMPLE_NB, false);
	if (ret)
		return ret;
	if (lp_mode) {
		ret = inv_init_selftest(st, SENSOR_ACCEL, false, true);
		if (ret)
			return ret;
		ret = inv_accel_calc_avg_with_samples(st, lsb_lp,
			SELF_TEST_SAMPLE_NB, true);
		if (ret)
			return ret;
		for (i = 0 ; i < 3 ; i++)
			accel_result[i] = lsb_lp[i];
	} else {
		for (i = 0 ; i < 3 ; i++)
			accel_result[i] = lsb[i];
	}
	ret = inv_init_selftest(st, SENSOR_ACCEL, true, false);
	if (ret)
		return ret;
	/* wait 20ms for oscillations to stabilize */
	msleep(20);
	ret = inv_accel_calc_avg_with_samples(st, lsb_st,
		SELF_TEST_SAMPLE_NB, false);
	if (ret)
		return ret;
	/* set selftest off */
	reg_data = 0x00;
	ret = inv_plat_single_write(st, REG_SELF_TEST_CONFIG, reg_data);
	if (ret)
		return ret;
	/* wait 20ms for oscillations to stabilize */
	msleep(20);
	/* read OTP */
	ret = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_2);
	if (ret)
		return ret;
	ret = inv_plat_read(st, REG_XA_ST_DATA, 3, otp_st_data_acc);
	if (ret)
		return ret;
	ret = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_0);
	if (ret)
		return ret;
	/* calculate ST results OTP based on the equation */
	for (i = 0; i < 3; i++) {
		if (otp_st_data_acc[i] != 0)
			st_otp[i] = SelfTestEquation[otp_st_data_acc[i] - 1];
		else
			otp_value_zero = true;
	}
	if (!otp_value_zero) {
		/* criteria a */
		for (i = 0; i < 3; i++) {
			st_res = lsb_st[i] - lsb[i];
			ratio = abs(st_res / st_otp[i] - SELF_TEST_PRECISION);
			if (ratio >= SELF_TEST_ACC_SHIFT_DELTA) {
				pr_debug("error accel[%d] : st_res = %d, st_otp = %d\n",
					i, st_res, st_otp[i]);
				pass = false;
			}
		}
	} else {
		/* criteria b */
		for (i = 0; i < 3; i++) {
			st_res = abs(lsb_st[i] - lsb[i]);
			if (st_res < SELF_TEST_MIN_ACC ||
				st_res > SELF_TEST_MAX_ACC) {
				pr_debug("error accel[%d] : st_res = %d, min = %d, max = %d\n",
					i, st_res,
					SELF_TEST_MIN_ACC,
					SELF_TEST_MAX_ACC);
				pass = false;
			}
		}
	}
	return !pass;
}

static uint16_t inv_gyro_do_selftest(struct inv_mpu_state *st,
	int *gyro_result, int lp_mode)
{
	u16 ret = 0;
	u8 reg_data = 0x0;
	int lsb[3] = {0}, lsb_st[3] = {0}, lsb_lp[3] = {0};
	u8 otp_st_data_gyro[3];
	int st_otp[3];
	int pass = true;
	int otp_value_zero = false;
	int i;
	int st_res, gyro_bias;

	ret = inv_init_selftest(st, SENSOR_GYRO, false, false);
	if (ret)
		return ret;
	ret = inv_gyro_calc_avg_with_samples(st, lsb,
		SELF_TEST_SAMPLE_NB, false);
	if (ret)
		return ret;
	if (lp_mode) {
		ret = inv_init_selftest(st, SENSOR_GYRO, false, true);
		if (ret)
			return ret;
		ret = inv_gyro_calc_avg_with_samples(st, lsb_lp,
			SELF_TEST_SAMPLE_NB, true);
		if (ret)
			return ret;
		for (i = 0 ; i < 3 ; i++)
			gyro_result[i] = lsb_lp[i];
	} else {
		for (i = 0 ; i < 3 ; i++)
			gyro_result[i] = lsb[i];
	}
	ret = inv_init_selftest(st, SENSOR_GYRO, true, false);
	if (ret)
		return ret;
	/* wait 20ms for oscillations to stabilize */
	msleep(20);
	/* Read 200 Samples with selftest on */
	ret = inv_gyro_calc_avg_with_samples(st, lsb_st,
		SELF_TEST_SAMPLE_NB, false);
	if (ret)
		return ret;
	/* set selftest off */
	reg_data = 0x00;
	ret = inv_plat_single_write(st, REG_SELF_TEST_CONFIG, reg_data);
	if (ret)
		return ret;
	/* wait 20ms for oscillations to stabilize */
	msleep(20);
	/* read OTP */
	ret = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_1);
	if (ret)
		return ret;
	ret = inv_plat_read(st, REG_XG_ST_DATA, 3, otp_st_data_gyro);
	if (ret)
		return ret;
	ret = inv_plat_single_write(st, REG_REG_BANK_SEL, BIT_BANK_SEL_0);
	if (ret)
		return ret;
	/* calculate ST_OTP */
	for (i = 0; i < 3; i++) {
		if (otp_st_data_gyro[i] != 0)
			st_otp[i] = SelfTestEquation[otp_st_data_gyro[i] - 1];
		else
			otp_value_zero = true;
	}
	if (!otp_value_zero) {
		/* criteria a */
		for (i = 0; i < 3; i++) {
			st_res = lsb_st[i] - lsb[i];
			if (st_res <= st_otp[i] * SELF_TEST_GYR_SHIFT_DELTA) {
				pr_debug("error gyro[%d] : st_res = %d, st_otp = %d\n",
					i, st_res, st_otp[i]);
				pass = false;
			}
		}
	} else {
		/* criteria b */
		for (i = 0; i < 3; i++) {
			st_res = abs(lsb_st[i] - lsb[i]);
			if (st_res < SELF_TEST_MIN_GYR) {
				pr_debug("error gyro[%d] : st_res = %d, min = %d\n",
					i, st_res, SELF_TEST_MIN_GYR);
				pass = false;
			}
		}
	}
	if (pass) {
		/* criteria c */
		for (i = 0; i < 3; i++) {
			gyro_bias = abs(lsb[i]);
			if (gyro_bias > SELF_TEST_MAX_GYR_OFFSET) {
				pr_debug("error gyro[%d] = %d, max = %d\n",
					i, gyro_bias, SELF_TEST_MAX_GYR_OFFSET);
				pass = false;
			}
		}
	}
	return !pass;
}

/*
 *  inv_hw_self_test() - main function to do hardware self test
 */
int inv_hw_self_test(struct inv_mpu_state *st)
{
	int result;
	int gyro_bias[3] = {0};
	int accel_bias[3] = {0};
	char accel_result, gyro_result;
	int i;

	accel_result = false;
	gyro_result = false;
	result = inv_save_setting(st);
	if (result)
		goto test_fail;
	if (result)
		goto test_fail;
	result = inv_accel_do_selftest(st, accel_bias, SELF_TEST_ACC_POWER_MODE);
	if (!result)
		accel_result = true;
	for (i = 0; i < 3; i++)
		st->accel_st_bias[i] = accel_bias[i] / SELF_TEST_PRECISION;
	result = inv_gyro_do_selftest(st, gyro_bias, SELF_TEST_GYRO_POWER_MODE);
	if (!result)
		gyro_result = true;
	for (i = 0; i < 3; i++)
		st->gyro_st_bias[i] = gyro_bias[i] / SELF_TEST_PRECISION;
test_fail:
	inv_recover_setting(st);
	return (accel_result << DEF_ST_ACCEL_RESULT_SHIFT) | gyro_result;
}
