/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/crc32.h>
#include <media/cam_sensor.h>

#include "cam_eeprom_core.h"
#include "cam_eeprom_soc.h"
#include "cam_debug_util.h"
/*for tof camera Begin*/
struct cam_eeprom_list_head cam_eeprom_list_head;
bool if_support_multi_camera = false;

void set_whether_support_multi_camera(bool support_state)
{
	if_support_multi_camera = support_state;
}

bool get_whether_support_multi_camera(void)
{
	return if_support_multi_camera;
}
int transmit_sensor_reg_setting_get(struct list_head *reg_settings
		,enum EEPROM_DATA_OP_T type
		,uint32_t mode)
{
	if(mode < EEPROM_MODE_DATA_NUM)
	{
		CAM_INFO(CAM_EEPROM,"set tof reg date from eeprom");
		if(type == EEPROM_INIT_DATA &&
				cam_eeprom_list_head.initial == 1) {
			reg_settings->prev
				= cam_eeprom_list_head.list_head_init.prev;
			reg_settings->next
				= cam_eeprom_list_head.list_head_init.next;
			cam_eeprom_list_head.list_head_init.next->prev
				= reg_settings;
			cam_eeprom_list_head.list_head_init.prev->next
				= reg_settings;
		} else if (type == EEPROM_CONFIG_DATA &&
				cam_eeprom_list_head.resolution[mode] == 1) {
			reg_settings->prev
				= cam_eeprom_list_head.list_head_config[mode].prev;
			reg_settings->next
				= cam_eeprom_list_head.list_head_config[mode].next;
			cam_eeprom_list_head.list_head_config[mode].next->prev
				= reg_settings;
			cam_eeprom_list_head.list_head_config[mode].prev->next
				= reg_settings;
		} else if(type == EEPROM_STREAMON_DATA && cam_eeprom_list_head.streamon == 1) {
			CAM_INFO(CAM_EEPROM,"streamon EEPROM data");
			reg_settings->prev
				= cam_eeprom_list_head.list_head_streamon.prev;
			reg_settings->next
				= cam_eeprom_list_head.list_head_streamon.next;
			cam_eeprom_list_head.list_head_streamon.next->prev
				= reg_settings;
			cam_eeprom_list_head.list_head_streamon.prev->next
				= reg_settings;
		} else if (type == EEPROM_STREAMOFF_DATA && cam_eeprom_list_head.streamoff == 1){
			CAM_INFO(CAM_EEPROM,"streamoff EEPROM data");
			reg_settings->prev
				= cam_eeprom_list_head.list_head_streamoff.prev;
			reg_settings->next
				= cam_eeprom_list_head.list_head_streamoff.next;
			cam_eeprom_list_head.list_head_streamoff.next->prev
				= reg_settings;
			cam_eeprom_list_head.list_head_streamoff.prev->next
				= reg_settings;
		} else return -1;
	} else return -1;
	return 0;
}

int transmit_sensor_reg_setting_ret(struct list_head *reg_settings
		,enum EEPROM_DATA_OP_T type
		,uint32_t mode)
{
	if(mode > EEPROM_MODE_DATA_NUM)
		return -1;
	if(type == EEPROM_INIT_DATA) {
		cam_eeprom_list_head.list_head_init.prev = reg_settings->prev;
		cam_eeprom_list_head.list_head_init.next = reg_settings->next;
		reg_settings->next->prev = &cam_eeprom_list_head.list_head_init;
		reg_settings->prev->next = &cam_eeprom_list_head.list_head_init;
		reg_settings->next = reg_settings;
		reg_settings->prev = reg_settings;
	} else if(type == EEPROM_CONFIG_DATA) {
			cam_eeprom_list_head.list_head_config[mode].prev
				= reg_settings->prev;
			cam_eeprom_list_head.list_head_config[mode].next
				= reg_settings->next;
			reg_settings->next->prev
				= &cam_eeprom_list_head.list_head_config[mode];
			reg_settings->prev->next
				= &cam_eeprom_list_head.list_head_config[mode];
			reg_settings->next = reg_settings;
			reg_settings->prev = reg_settings;
	}
	if(type == EEPROM_STREAMON_DATA){
		cam_eeprom_list_head.list_head_streamon.prev = reg_settings->prev;
		cam_eeprom_list_head.list_head_streamon.next = reg_settings->next;
		reg_settings->next->prev = &cam_eeprom_list_head.list_head_streamon;
		reg_settings->prev->next = &cam_eeprom_list_head.list_head_streamon;
		reg_settings->next = reg_settings;
		reg_settings->prev = reg_settings;
	}
	if(type == EEPROM_STREAMOFF_DATA){
		cam_eeprom_list_head.list_head_streamoff.prev = reg_settings->prev;
		cam_eeprom_list_head.list_head_streamoff.next = reg_settings->next;
		reg_settings->next->prev = &cam_eeprom_list_head.list_head_streamoff;
		reg_settings->prev->next = &cam_eeprom_list_head.list_head_streamoff;
		reg_settings->next = reg_settings;
		reg_settings->prev = reg_settings;
	}

//  show();
	return 0;
}

static int cam_eeprom_list_head_create(struct list_head *list_head)
{
	if(list_head == NULL)
		return -1;
	list_head->next = list_head;
	list_head->prev = list_head;
	return 0;
}

static struct i2c_settings_list * list_node_create(
		struct list_head *list_head
		,uint32_t data_num)
{
	struct i2c_settings_list *node
		= (struct i2c_settings_list *)kzalloc(
			sizeof(struct i2c_settings_list),GFP_KERNEL);
	if (node != NULL)
		list_add_tail(&(node->list),list_head);
	else
		return NULL;

	if(data_num == 1) {
		node->op_code = CAM_SENSOR_I2C_WRITE_BURST;
	} else if (data_num > 1) {
		node->op_code = CAM_SENSOR_I2C_WRITE_RANDOM;
	} else {
		CAM_ERR(CAM_EEPROM,"data_num error");
		return NULL;
	}
	node->i2c_settings.size = data_num;
	node->i2c_settings.reg_setting
		= (struct cam_sensor_i2c_reg_array *)kzalloc(
			sizeof(struct cam_sensor_i2c_reg_array)*data_num,GFP_KERNEL);
	if(node->i2c_settings.reg_setting == NULL) {
		return NULL;
	}
	node->i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	node->i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;

	return node;
}

static int create_node_for_seq(struct cam_eeprom_ctrl_t *e_ctrl
		,uint32_t reg_addr
		,uint32_t data_num)
{
	uint32_t n                = 2;
	uint32_t i                = 0;
	uint32_t temp             = 0;
	uint32_t reg_val          = 0;

	struct i2c_settings_list *node = kzalloc(
			sizeof(struct i2c_settings_list),GFP_KERNEL);
	if (node != NULL)
		list_add_tail(&(node->list),&cam_eeprom_list_head.list_head_init);
	else
		return -1;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,reg_addr + n,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	temp = reg_val;

	node->i2c_settings.reg_setting
		= (struct cam_sensor_i2c_reg_array *)kzalloc(
			sizeof(struct cam_sensor_i2c_reg_array)*data_num,GFP_KERNEL);
	if(node->i2c_settings.reg_setting == NULL){
		return -1;
	}

	for(i = 0;i < data_num; i++)
	{
		n += 2;
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,reg_addr + n,&reg_val
				,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
		node->i2c_settings.reg_setting[i].reg_data = reg_val;
		node->i2c_settings.reg_setting[i].reg_addr = temp + i;
	}
	node->i2c_settings.size = data_num;
	node->i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	node->i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	node->op_code = CAM_SENSOR_I2C_WRITE_SEQ;

	return 0;
}

static int cam_eeprom_list_init_setting_data(
		struct cam_eeprom_ctrl_t *e_ctrl,
		tl_dev_eeprom_pup *tof_eeprom)
{
	uint32_t  n                            = 0;
	int16_t   i                            = 0;
	uint32_t  reg_addr                     = EEPROM_INIT_MAP_ADDR;
	int       rc                           = 0;
	struct    i2c_settings_list *i2c_list;
	uint32_t  data_num,pup_size;
	uint32_t  reg_val,reg_array_num,addr_temp,n_temp;

	rc = cam_eeprom_list_head_create(
			&cam_eeprom_list_head.list_head_init);
	if(rc == -1){
		CAM_ERR(CAM_EEPROM,"list_head == NULL");
		return rc;
	}
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,TL_EEPROM_PUP_SIZE,&pup_size
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	if(pup_size <= 0){
		return -1;
	}
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,reg_addr + n,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	data_num = reg_val;
	if(data_num < 1){
		CAM_ERR(CAM_EEPROM,"have no powerup message");
		return -1;
	}
	rc = create_node_for_seq(e_ctrl,reg_addr,data_num);
	if(rc == -1){
		CAM_ERR(CAM_EEPROM,"SEQ == NULL");
		return -1;
	}

	n += (data_num + 2) * 2; //data_num reg_addr reg_data ...
	while(n < pup_size)
	{
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,reg_addr + n,&reg_val
				,CAMERA_SENSOR_I2C_TYPE_WORD
				,CAMERA_SENSOR_I2C_TYPE_WORD);
		data_num = reg_val;
		if(data_num < 0){
			CAM_ERR(CAM_EEPROM,"data_num 0");
			return -1;
		} else if(data_num > 1){
			n += 2;
			camera_io_dev_read(&(e_ctrl->io_master_info)
					,reg_addr + n,&reg_val
					,CAMERA_SENSOR_I2C_TYPE_WORD
					,CAMERA_SENSOR_I2C_TYPE_WORD);
			addr_temp = reg_val;
			n += 2;
			while(data_num > 0){
				i2c_list = list_node_create(
						&cam_eeprom_list_head.list_head_init,1);
				if(i2c_list == NULL)
					return -1;
				camera_io_dev_read(&(e_ctrl->io_master_info)
						,reg_addr + n,&reg_val
						,CAMERA_SENSOR_I2C_TYPE_WORD
						,CAMERA_SENSOR_I2C_TYPE_WORD);
				i2c_list->i2c_settings.reg_setting[0].reg_data
					= reg_val;
				i2c_list->i2c_settings.reg_setting[0].reg_addr
					= addr_temp;
				if(addr_temp++ == 0xC08E){
					tof_eeprom->gpo_out_stby_value = reg_val;
				}
				n += 2;
				data_num--;
			}
		} else {
			n_temp = n;
			reg_array_num = 1;
			while(1)
			{
				n_temp += 6;
				if(n_temp < pup_size){
				camera_io_dev_read(&(e_ctrl->io_master_info)
						,reg_addr + n_temp,&reg_val
						,CAMERA_SENSOR_I2C_TYPE_WORD
						,CAMERA_SENSOR_I2C_TYPE_WORD);
				if(reg_val != 1)
				{
					break;
				}
				reg_array_num++;
				} else
					break;
			}
			i2c_list = list_node_create(
					&cam_eeprom_list_head.list_head_init,reg_array_num);
			if(i2c_list == NULL)
				return -1;
			if(reg_array_num == 1){
				i2c_list->op_code = CAM_SENSOR_I2C_WRITE_RANDOM;
			}
			for(i = 0;i < reg_array_num;i++){
				n += 2;
				camera_io_dev_read(&(e_ctrl->io_master_info)
						,reg_addr + n,&reg_val
						,CAMERA_SENSOR_I2C_TYPE_WORD
						,CAMERA_SENSOR_I2C_TYPE_WORD);
				i2c_list->i2c_settings.reg_setting[i].reg_addr = reg_val;
				n += 2;
				camera_io_dev_read(&(e_ctrl->io_master_info)
						,reg_addr + n,&reg_val
						,CAMERA_SENSOR_I2C_TYPE_WORD
						,CAMERA_SENSOR_I2C_TYPE_WORD);
				i2c_list->i2c_settings.reg_setting[i].reg_data = reg_val;
				if(i2c_list->i2c_settings.reg_setting[i].reg_addr == 0xC08E){
					tof_eeprom->gpo_out_stby_value = reg_val;
				}
				n += 2;
			}
		}
	}
	cam_eeprom_list_head.initial = 1;
	return 0;
}

static int eeprom_create_list_node(struct cam_eeprom_ctrl_t * e_ctrl
		,struct cam_eeprom_config_common *eeprom_config_common
		,uint32_t array_size
		,uint32_t mode)
{
	uint32_t  i               = 0;
	uint32_t  j               = 0;
	uint32_t  temp            = 0;
	uint32_t  reg_val         = 0;
	uint32_t  size            = 0;
	uint32_t  index           = 0;
	struct    i2c_settings_list *i2c_list;

	for(i = 0; i < array_size; i++){
		if(eeprom_config_common[i].op_code == CAM_SENSOR_I2C_WRITE_BURST){
			for(j = 0;j < eeprom_config_common[i].data_size;j++){
				camera_io_dev_read(&(e_ctrl->io_master_info)
						,eeprom_config_common[i].reg_addr_from + (j * 2)
						,&reg_val
						,CAMERA_SENSOR_I2C_TYPE_WORD
						,CAMERA_SENSOR_I2C_TYPE_WORD);
				i2c_list = list_node_create(
						&cam_eeprom_list_head.list_head_config[mode],1);
				if(i2c_list == NULL)
					return -1;
				i2c_list->i2c_settings.reg_setting[0].reg_addr
					= eeprom_config_common[i].reg_addr_to + j;
				i2c_list->i2c_settings.reg_setting[0].reg_data = reg_val;
			}
		} else {
			temp = i;
			index = 0;
			while(eeprom_config_common[temp].op_code
					== CAM_SENSOR_I2C_WRITE_RANDOM){
				index += eeprom_config_common[temp].data_size;
				temp++;
			}
			size = temp - i;
			i2c_list = list_node_create(
					&cam_eeprom_list_head.list_head_config[mode]
					,index);
			if(i2c_list == NULL)
				return -1;
			if(index == 1)
				i2c_list->op_code = CAM_SENSOR_I2C_WRITE_RANDOM;
			for(j = 0;j < size;j++){
				i2c_list->i2c_settings.reg_setting[j].reg_addr
					= eeprom_config_common[i].reg_addr_to;
				camera_io_dev_read(&(e_ctrl->io_master_info)
						,eeprom_config_common[i].reg_addr_from
						,&reg_val
						,CAMERA_SENSOR_I2C_TYPE_WORD
						,CAMERA_SENSOR_I2C_TYPE_WORD);
				i2c_list->i2c_settings.reg_setting[j].reg_data
					= reg_val;
				i++;
			}
			i--;
		}
	}
	return 0;
}


static int read_eeprom_config_reg_addr(struct cam_eeprom_ctrl_t *e_ctrl
		,struct cam_eeprom_exp_reg_addr *reg_addr_array
		,uint32_t mode)
{
	uint32_t temp,i,reg_val;
	//exp
	//  addr_num
	uint32_t eeprom_mode_addr
		= EEPROM_MODE_DATA_TOP + EEPROM_MODE_DATA_SIZE * mode;
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_EXP_ADDR_NUM,&temp
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	reg_addr_array->exp_addr.long_num = ((temp & 0xFF00) >> 8);
	if(reg_addr_array->exp_addr.long_num > 15){
		CAM_ERR(CAM_EEPROM,"long_num > MAX");
		return -1;
	}
	reg_addr_array->exp_addr.short_num = ((temp & 0x00F0) >> 4);
	if(reg_addr_array->exp_addr.short_num > 4){
		CAM_ERR(CAM_EEPROM,"short_num > MAX");
		return -1;
	}
	reg_addr_array->exp_addr.lms_num = (temp & 0x000F);
	if(reg_addr_array->exp_addr.lms_num > 4){
		CAM_ERR(CAM_EEPROM,"lms_num > MAX");
		return -1;
	}
	//  addr
	for(i = 0;i < reg_addr_array->exp_addr.long_num; i++){
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,eeprom_mode_addr+TL_EEPROM_EXP_ADDR_LONG+(i * 2)
				,&reg_val
				,CAMERA_SENSOR_I2C_TYPE_WORD
				,CAMERA_SENSOR_I2C_TYPE_WORD);
		reg_addr_array->exp_addr.long_addr[i] = reg_val;
	}
	for(i = 0; i < reg_addr_array->exp_addr.short_num;i++){
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,eeprom_mode_addr+TL_EEPROM_EXP_ADDR_SHORT+(i * 2)
				,&reg_val
				,CAMERA_SENSOR_I2C_TYPE_WORD
				,CAMERA_SENSOR_I2C_TYPE_WORD);
		reg_addr_array->exp_addr.short_addr[i] = reg_val;
	}
	for(i = 0; i < reg_addr_array->exp_addr.lms_num;i++){
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,eeprom_mode_addr+TL_EEPROM_EXP_ADDR_LMS+(i * 2)
				,&reg_val
				,CAMERA_SENSOR_I2C_TYPE_WORD
				,CAMERA_SENSOR_I2C_TYPE_WORD);
		reg_addr_array->exp_addr.lms_addr[i] = reg_val;
	}

	//read_size2
	reg_addr_array->read_size2_addr = TL_AFE_READ_SIZE2_ADDR;
	//ccd_dummy_addr
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_DMMY_TRNS_NUM,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD
			,CAMERA_SENSOR_I2C_TYPE_WORD);
	reg_addr_array->ccd_dummy_addr.addr_num = reg_val;
	if(reg_val > 4){
		CAM_ERR(CAM_EEPROM,"addr_num > MAX");
		return -1;
	}
	for(i = 0;i < reg_addr_array->ccd_dummy_addr.addr_num;i++){
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,eeprom_mode_addr+TL_EEPROM_DMMY_TRNS_ADR+(i * 2)
				,&reg_val
				,CAMERA_SENSOR_I2C_TYPE_WORD
				,CAMERA_SENSOR_I2C_TYPE_WORD);
		reg_addr_array->ccd_dummy_addr.addr[i] = reg_val;
	}
	//start_v_addr
	reg_addr_array->start_v_addr = TL_AFE_CHKR_START_V_ADDR;

	//vd_ini_ofst_addr
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_VD_INI_OFST_ADR_NUM
			,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD
			,CAMERA_SENSOR_I2C_TYPE_WORD);
	reg_addr_array->vd_ini_ofst_addr.vd_ini_ofst_num = reg_val;
	if(reg_val > 4){
		CAM_ERR(CAM_EEPROM,"vd_ini_ofst_num > MAX");
		return -1;
	}
	for(i = 0;i < reg_addr_array->vd_ini_ofst_addr.vd_ini_ofst_num;i++){
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,eeprom_mode_addr+TL_EEPROM_VD_INIT_OFST_ADR+(i * 2)
				,&reg_val,CAMERA_SENSOR_I2C_TYPE_WORD
				,CAMERA_SENSOR_I2C_TYPE_WORD);
		reg_addr_array->vd_ini_ofst_addr.addr[i] = reg_val;
	}
	//vd_length_addr
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_VD_REG_ADR,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	reg_addr_array->vd_length_addr = reg_val;
	return 0;
}

static int read_eeprom_config_reg_data(struct cam_eeprom_ctrl_t *e_ctrl
		,struct cam_eeprom_exp_reg_data *reg_data_array
		,uint32_t mode)
{
	uint32_t addr_temp,i,reg_val,clk;
	uint32_t pls_num = 0;
	int32_t tmp,dmy_hd,exp_hd,hd;
	struct cam_eeprom_config_exp_data prm;
	uint32_t eeprom_mode_addr;
	uint32_t IdlePeri;

	eeprom_mode_addr
		= EEPROM_MODE_DATA_TOP + EEPROM_MODE_DATA_SIZE * mode;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_NUM_CLK_IN_HD,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD
			,CAMERA_SENSOR_I2C_TYPE_WORD);
	if(reg_val == 0){
		CAM_ERR(CAM_EEPROM,"num_clk_hd == 0");
		return -1;
	}
	//exp
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_EXP_MAX,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD
			,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.exp_max = reg_val;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_EXP_DEFAULT,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD
			,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.exp_val = reg_val;
	if(prm.exp_val > prm.exp_max){
		prm.exp_val = prm.exp_max;
	}
	reg_data_array->exp_long_data = prm.exp_val;
	reg_data_array->exp_short_data = prm.exp_val/4;
	reg_data_array->exp_lms_data = prm.exp_val-(prm.exp_val/4)-1;

	for(i = 0; i < 10;i++){
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,eeprom_mode_addr+TL_EEPROM_PLS_MOD_VAL+(i * 2)
				,&reg_val
				,CAMERA_SENSOR_I2C_TYPE_WORD
				,CAMERA_SENSOR_I2C_TYPE_WORD);
		pls_num += reg_val;
	}

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_TOF_SEQ_INI_OFST,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.tof_seq_ini_ofst = reg_val;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_LD_PLS_DUTY,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.ld_pls_duty = reg_val;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_NUM_CLK_IN_HD,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.num_clk_in_hd = reg_val;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_BETA_NUM,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.beta_num = reg_val;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_TOF_EMT_PERIOD_OFST,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.tof_emt_period_ofst = reg_val;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_VD_DURATION,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.vd_duration = reg_val;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_VD_INI_OFST,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.vd_ini_ofst = reg_val;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_OPT_AXIS_CENTER_H,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.num_hd_in_readout = reg_val;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_IDLE_PERI_NUM,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.idle = reg_val;
	if(prm.idle > 4){
		return -1;
	}
	for(i = 0;i < prm.idle;i++){
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,eeprom_mode_addr+TL_EEPROM_IDLE_PERI_ADR1+i*2,
				&reg_val
				,CAMERA_SENSOR_I2C_TYPE_WORD
				,CAMERA_SENSOR_I2C_TYPE_WORD);
		if(reg_val == 0){
			prm.idle = reg_val;
		} else {
			prm.Idle[i] = reg_val + 2;
		}
	}

	tmp = pls_num*prm.exp_val/40;
	if(pls_num*prm.exp_val%40 >= 20)
		tmp++;
	clk = (prm.tof_seq_ini_ofst+prm.exp_val*prm.ld_pls_duty-2)+tmp;
	hd = (clk/prm.num_clk_in_hd);
	if((clk%prm.num_clk_in_hd) > 0)
		hd++;
	exp_hd = ((hd+hd+hd)*prm.beta_num)+prm.tof_emt_period_ofst;
	dmy_hd = prm.vd_duration-prm.vd_ini_ofst
		-exp_hd-prm.num_hd_in_readout;
	if(dmy_hd < 0 || dmy_hd > 0xFFFF){
		CAM_ERR(CAM_EEPROM,"dmy_hd error");
		return -1;
	}
	if(prm.idle == 0)
		IdlePeri = 0;
	else{
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,prm.Idle[0],&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
		IdlePeri = reg_val;
	}
	prm.idle = (prm.idle == 0)?0:(prm.idle -2U);
#if 0
	reg_data_array->read_size2_data = exp_hd + TL_AFE_READ_SIZE_OFFSET;
	reg_data_array->ccd_dummy_data = dmy_hd;
	reg_data_array->start_v_data = exp_hd + TL_AFE_START_V_OFFSET;
	reg_data_array->vd_length_data = prm.vd_duration - 2;
#endif
	if(if_support_multi_camera == false){
		reg_data_array->read_size2_data
			= prm.vd_ini_ofst+exp_hd+TL_AFE_READ_SIZE_OFFSET+IdlePeri;
		reg_data_array->ccd_dummy_data = dmy_hd-IdlePeri;
		reg_data_array->start_v_data = exp_hd+TL_AFE_START_V_OFFSET;
		reg_data_array->vd_length_data = prm.vd_duration-2;
	} else {
		reg_data_array->read_size2_data
			= prm.vd_ini_ofst + exp_hd + TL_AFE_READ_SIZE_OFFSET + prm.idle;
		reg_data_array->ccd_dummy_data = prm.vd_duration + dmy_hd - prm.idle;
		reg_data_array->start_v_data = exp_hd + TL_AFE_START_V_OFFSET;
		reg_data_array->vd_length_data = (prm.vd_duration * 2) - 2;
	}
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_VD_INI_OFST_ADR_NUM,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.vd_ini_ofst_adr_num = reg_val;
	if((prm.vd_ini_ofst >= 2) && prm.vd_ini_ofst_adr_num > 0){
		addr_temp = prm.vd_ini_ofst - 1;
		reg_data_array->vd_ini_ofst_data = addr_temp;
	}

	return 0;
}

static int eeprom_create_exp_list_node(struct cam_eeprom_ctrl_t *e_ctrl
		,uint32_t mode)
{
	uint32_t data_num,i;
	uint32_t data_cnt = 0;
	int      rc = -1;
	struct i2c_settings_list *i2c_list;
	struct cam_eeprom_exp_reg_addr *reg_addr_array;
	struct cam_eeprom_exp_reg_data *reg_data_array;

	reg_addr_array = (struct cam_eeprom_exp_reg_addr *)kzalloc(
			sizeof(struct cam_eeprom_exp_reg_addr),GFP_KERNEL);
	if(reg_addr_array == NULL) return rc;
	reg_data_array = (struct cam_eeprom_exp_reg_data *)kzalloc(
			sizeof(struct cam_eeprom_exp_reg_data),GFP_KERNEL);
	if(reg_data_array == NULL)
	    goto free_addr_array;
	rc = read_eeprom_config_reg_addr(e_ctrl,reg_addr_array,mode);
	if(rc != 0)
		goto free_array;
	rc = read_eeprom_config_reg_data(e_ctrl,reg_data_array,mode);
	if(rc != 0)
		goto free_array;
	//create_node
	data_num = 1 + 1 + 1
		+ reg_addr_array->exp_addr.long_num
		+ reg_addr_array->exp_addr.short_num
		+ reg_addr_array->exp_addr.lms_num
		+ reg_addr_array->ccd_dummy_addr.addr_num
		+ reg_addr_array->vd_ini_ofst_addr.vd_ini_ofst_num;

	i2c_list = list_node_create(
			&cam_eeprom_list_head.list_head_config[mode],data_num);

    if(i2c_list) {
		for(i = 0;i < reg_addr_array->exp_addr.long_num;i++){
			i2c_list->i2c_settings.reg_setting[data_cnt].reg_addr
				= reg_addr_array->exp_addr.long_addr[i];
			i2c_list->i2c_settings.reg_setting[data_cnt].reg_data
				= reg_data_array->exp_long_data;
			data_cnt++;
		}
		for(i = 0;i < reg_addr_array->exp_addr.short_num;i++){
			i2c_list->i2c_settings.reg_setting[data_cnt].reg_addr
				= reg_addr_array->exp_addr.long_addr[i];
			i2c_list->i2c_settings.reg_setting[data_cnt].reg_data
				= reg_data_array->exp_short_data;
			data_cnt++;
		}
		for(i = 0;i < reg_addr_array->exp_addr.lms_num;i++){
			i2c_list->i2c_settings.reg_setting[data_cnt].reg_addr
				= reg_addr_array->exp_addr.long_addr[i];
			i2c_list->i2c_settings.reg_setting[data_cnt].reg_data
				= reg_data_array->exp_lms_data;
			data_cnt++;
		}

		i2c_list->i2c_settings.reg_setting[data_cnt].reg_addr
			= reg_addr_array->read_size2_addr;
		i2c_list->i2c_settings.reg_setting[data_cnt].reg_data
			= reg_data_array->read_size2_data;
		data_cnt++;

		for(i = 0;i < reg_addr_array->ccd_dummy_addr.addr_num;i++){
			i2c_list->i2c_settings.reg_setting[data_cnt].reg_addr
				= reg_addr_array->ccd_dummy_addr.addr[i];
			i2c_list->i2c_settings.reg_setting[data_cnt].reg_data
				= reg_data_array->ccd_dummy_data;
			data_cnt++;
		}

		i2c_list->i2c_settings.reg_setting[data_cnt].reg_addr
			= reg_addr_array->start_v_addr;
		i2c_list->i2c_settings.reg_setting[data_cnt].reg_data
			= reg_data_array->start_v_data;
		data_cnt++;

		for(i=0;i<reg_addr_array->vd_ini_ofst_addr.vd_ini_ofst_num;i++){
			i2c_list->i2c_settings.reg_setting[data_cnt].reg_addr
				= reg_addr_array->vd_ini_ofst_addr.addr[i];
			i2c_list->i2c_settings.reg_setting[data_cnt].reg_data
				= reg_data_array->vd_ini_ofst_data;
			data_cnt++;
		}

		i2c_list->i2c_settings.reg_setting[data_cnt].reg_addr
			= reg_addr_array->vd_length_addr;
		i2c_list->i2c_settings.reg_setting[data_cnt].reg_data
			= reg_data_array->vd_length_data;
		data_cnt++;
    } else
		rc = -1;

free_array:
	kfree(reg_data_array);
free_addr_array:
	kfree(reg_addr_array);
	return rc;
}

static int cam_eeprom_list_common_data_offload(
		struct cam_eeprom_ctrl_t *e_ctrl
		,uint32_t mode)
{
	uint32_t i             = 0;
	uint32_t rc            = 0;
	uint32_t reg_addr_from_array[EEPROM_CONFIG_DATA_MAX] = {
		TL_EEPROM_SHD_OFFSET,
		TL_EEPROM_SHD,
		TL_EEPROM_SHD_X0,
		TL_EEPROM_SHD_XPWR,
		TL_EEPROM_SHD_Y0,
		TL_EEPROM_SHD_YPWR,
		TL_EEPROM_DFCT_PIX_TH_TBL,
		TL_EEPROM_DFCT,
		TL_EEPROM_SHP_LOC,
		TL_EEPROM_SHD_LOC,
		TL_EEPROM_OUTPUT,
		TL_EEPROM_OUTPUT_SEL,
		TL_EEPROM_VC,
		TL_EEPROM_GRID3,
		TL_EEPROM_IR1,
		TL_EEPROM_IR_GMM,
		TL_EEPROM_IR_GMM_Y,
		TL_EEPROM_UPPRTH,
		TL_EEPROM_LWRTH,
		TL_EEPROM_START_V,
		TL_EEPROM_START_H,
		TL_EEPROM_SIZE_H,
		TL_EEPROM_UPPRERR_H,
		TL_EEPROM_UPPRERR_V,
		TL_EEPROM_LWRERR_H,
		TL_EEPROM_LWRERR_V,
		TL_EEPROM_DET_ENA,
	};
	uint32_t data_size_array[EEPROM_CONFIG_DATA_MAX] = {
		EEPROM_CONFIG_SHD_OFFSET_SIZE,
		EEPROM_CONFIG_SHD_SIZE,
		EEPROM_CONFIG_SHD_X0_SIZE,
		EEPROM_CONFIG_SHD_WPWR_SIZE,
		EEPROM_CONFIG_SHD_Y0_SIZE,
		EEPROM_CONFIG_SHD_YPWR_SIZE,
		EEPROM_CONFIG_DFCT_PIX_TH_TBL_SIZE,
		EEPROM_CONFIG_DFCT_SIZE,
		EEPROM_CONFIG_TIMING_MIPI_SHP_LOC_SIZE,
		EEPROM_CONFIG_TIMING_MIPI_SHD_LOC_SIZE,
		EEPROM_CONFIG_TIMING_MIPI_OUTPUT_SIZE,
		EEPROM_CONFIG_TIMING_MIPI_OUTPUT_SEL_SIZE,
		EEPROM_CONFIG_TIMING_MIPI_VC_SIZE,
		EEPROM_CONFIG_GRID3_SIZE_SIZE,
		EEPROM_CONFIG_IR_SIZE,
		EEPROM_CONFIG_IR_GMM_SIZE,
		EEPROM_CONFIG_IR_GMM_Y_SIZE,
		EEPROM_CONFIG_CHKR_UPPRTH_SIZE,
		EEPROM_CONFIG_CHKR_LWRTH_SIZE,
		EEPROM_CONFIG_CHKR_START_V_SIZE,
		EEPROM_CONFIG_CHKR_START_H_SIZE,
		EEPROM_CONFIG_CHKR_SIZE_H_SIZE,
		EEPROM_CONFIG_CHKR_UPRERR_H_SIZE,
		EEPROM_CONFIG_CHKR_UPRERR_V_SIZE,
		EEPROM_CONFIG_CHKR_LWRERR_H_SIZE,
		EEPROM_CONFIG_CHKR_LWRERR_V_SIZE,
		EEPROM_CONFIG_CHKR_DET_ENA_SIZE,
	};
	uint32_t reg_addr_to_array[EEPROM_CONFIG_DATA_MAX] = {
		TL_AFE_SHD_OFFSET_ADDR,
		TL_AFE_SHD_ADDR,
		TL_AFE_SHD_X0_ADDR,
		TL_AFE_SHD_XPWR_ADDR,
		TL_AFE_SHD_Y0_ADDR,
		TL_AFE_SHD_YPWR_ADDR,
		TL_AFE_DFCT_PIX_TH_TBL_ADDR,
		TL_AFE_DFCT_ADDR,
		TL_AFE_SHP_LOC_ADDR,
		TL_AFE_SHD_LOC_ADDR,
		TL_AFE_OUTPUT_ADDR,
		TL_AFE_OUTPUTSEL_ADDR,
		TL_AFE_VC_ADDR,
		TL_AFE_GRID3_ADDR,
		TL_AFE_IR_GAIN_GMM_ADDR,
		TL_AFE_IR_GMM_ADDR,
		TL_AFE_IR_GMM_Y_ADDR,
		TL_AFE_CHKR_UPPRTH_ADDR,
		TL_AFE_CHKR_LWRTH_ADDR,
		TL_AFE_CHKR_START_V_ADDR,
		TL_AFE_CHKR_START_H_ADDR,
		TL_AFE_CHKR_SIZE_H_ADDR,
		TL_AFE_CHKR_UPRERR_H_ADDR,
		TL_AFE_CHKR_UPRERR_V_ADDR,
		TL_AFE_CHKR_LWRERR_H_ADDR,
		TL_AFE_CHKR_LWRERR_V_ADDR,
		TL_AFE_CHKR_DET_ENA_ADDR,
	};

	struct cam_eeprom_config_common *eeprom_common_array
		= (struct cam_eeprom_config_common *)kzalloc(
		sizeof(struct cam_eeprom_config_common)*EEPROM_CONFIG_DATA_MAX
		,GFP_KERNEL);
	if(eeprom_common_array == NULL)
		return -1;
	rc = cam_eeprom_list_head_create(
			&cam_eeprom_list_head.list_head_config[mode]);
	if(rc == -1){
		CAM_ERR(CAM_EEPROM,"list_head == NULL");
		goto free_common;
	}
	for(i = 0;i < EEPROM_CONFIG_DATA_MAX; i++){
		eeprom_common_array[i].reg_addr_from = reg_addr_from_array[i];
		eeprom_common_array[i].data_size = data_size_array[i];
		eeprom_common_array[i].reg_addr_to = reg_addr_to_array[i];
		if(i == TL_AFE_SHP_LOC || i == TL_AFE_SHD_LOC
				|| i == TL_AFE_GRID3){
			eeprom_common_array[i].op_code
				= CAM_SENSOR_I2C_WRITE_RANDOM;
		} else {
			eeprom_common_array[i].op_code
				= CAM_SENSOR_I2C_WRITE_BURST;
		}
	}
	rc = eeprom_create_list_node(e_ctrl
			,eeprom_common_array,EEPROM_CONFIG_DATA_MAX,mode);
	if(rc != 0)
		return rc;
	return 0;
free_common:
	kfree(eeprom_common_array);
	return -1;
}

static int eeprom_create_ld_list_node(struct cam_eeprom_ctrl_t *e_ctrl
		,uint32_t mode)
{
	uint32_t temp = 0;
	uint32_t ld_setting = TL_AFE_LD_ENABLE;
	struct    i2c_settings_list *i2c_list;
	uint32_t reg_val;
	uint32_t eeprom_mode_addr
		= EEPROM_MODE_DATA_TOP + EEPROM_MODE_DATA_SIZE * mode;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_AFE_WDR,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	temp = reg_val;
	if((temp & 0x0001) == 1U){
		ld_setting |= 0x0001U;
	} else if (((temp & 0x0010U) >> 4U) == 1U){
		ld_setting |= 0x0002U;
	} else if (((temp & 0x0100U) >> 8U) == 1U){
		ld_setting |= 0x0004U;
	} else if (((temp & 0x1000U) >> 12U) == 1U){
		ld_setting |= 0x0008U;
	}
	i2c_list
		= list_node_create(
				&cam_eeprom_list_head.list_head_config[mode],1);
	if(i2c_list == NULL)
		return -1;
	i2c_list->i2c_settings.reg_setting[0].reg_addr
		= TL_AFE_LDPOSBLKOUTEN_ADDR;
	i2c_list->i2c_settings.reg_setting[0].reg_data
		= ld_setting;
	i2c_list =
		list_node_create(
				&cam_eeprom_list_head.list_head_config[mode],1);
	if(i2c_list == NULL)
		return -1;
	i2c_list->i2c_settings.reg_setting[0].reg_addr
		= TL_AFE_LDNEGBLKOUTEN_ADDR;
	i2c_list->i2c_settings.reg_setting[0].reg_data
		= ld_setting;
	return 0;
}


static int eeprom_create_mode_list_node(
		struct cam_eeprom_ctrl_t *e_ctrl
		,uint32_t mode)
{
	struct i2c_settings_list *i2c_list;
	i2c_list = list_node_create(
			&cam_eeprom_list_head.list_head_config[mode],2);
	if(i2c_list == NULL)
		return -1;
	i2c_list->i2c_settings.reg_setting[0].reg_addr
		= TL_AFE_MODE_ADDR;
	i2c_list->i2c_settings.reg_setting[0].reg_data
		= TL_AFE_MODE0_VAL + mode;
	i2c_list->i2c_settings.reg_setting[1].reg_addr =
		TL_AFE_OUTPUTSEL_ADDR;
	i2c_list->i2c_settings.reg_setting[1].reg_data
		= TL_EEPROM_LD_FLAG;
	return 0;
}

static int cam_eeprom_list_other(struct cam_eeprom_ctrl_t *e_ctrl)
{
	struct    i2c_settings_list *i2c_list;
	uint32_t rc = 0;
	rc = cam_eeprom_list_head_create(
			&cam_eeprom_list_head.list_head_config_other);
	i2c_list = list_node_create(
			&cam_eeprom_list_head.list_head_config_other,2);
	if(i2c_list == NULL)
		return -1;
	i2c_list->i2c_settings.reg_setting[0].reg_addr = 0x104;
	i2c_list->i2c_settings.reg_setting[0].reg_data = 0x2;
	i2c_list->i2c_settings.reg_setting[1].reg_addr = 0x104;
	i2c_list->i2c_settings.reg_setting[1].reg_data = 0x2;
	cam_eeprom_list_head.other = 1;
	return 0;
}

static int cam_eeprom_list_mode_data_offload(
		struct cam_eeprom_ctrl_t *e_ctrl
		,uint32_t mode)
{
	struct    i2c_settings_list *i2c_list;
	int16_t  i             = 0;
	int      rc            = 0;
	uint32_t  eeprom_mode_addr
		= EEPROM_MODE_DATA_TOP+ EEPROM_MODE_DATA_SIZE * mode;
	uint32_t reg_addr_from_array[EEPROM_CONFIG_DATA_MODE_MAX] = {
		eeprom_mode_addr + EEPROM_CONFIG_NLR_OFFSET_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_NLR_X0_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_NLR_XPWR_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_DEPTH0_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_DEPTH2_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_DEPTH3_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_RATE_ADJUST_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_ALIGN_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_READ_SIZE0_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_READ_SIZE3_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_ROI_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_GRID_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_RAWNR_XPWR_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_RAWNR_BLTBL_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_MED_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_SAT_TH_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_RAWNR_BKTNL_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_COR_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_CORB_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_CORF_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_DEPTH1_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_DEPTH_CTRL_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_PLS_MOD_CTRL_ADDR,
		eeprom_mode_addr + EEPROM_CONFIG_PLS_MOD_VAL_ADDR,
	};
	uint32_t data_size_array[EEPROM_CONFIG_DATA_MODE_MAX] = {
		EEPROM_NLR_OFFSET_SIZE,
		EEPROM_NLR_X0_SIZE,
		EEPROM_NLR_XPWR_SIZE,
		EEPROM_DEPTH0_SIZE,
		EEPROM_DEPTH2_SIZE,
		EEPROM_DEPTH3_SIZE,
		EEPROM_RATE_ADJUST_SIZE,
		EEPROM_ALIGN_SIZE,
		EEPROM_READ_SIZE0_SIZE,
		EEPROM_READ_SIZE3_SIZE,
		EEPROM_ROI_SIZE,
		EEPROM_GRID_SIZE,
		EEPROM_RAWNR_XPWR_SIZE,
		EEPROM_RAWNR_BLTBL_SIZE,
		EEPROM_MED_SIZE,
		EEPROM_SAT_TH_SIZE,
		EEPROM_RAWNR_BKTNL_SIZE,
		EEPROM_COR_SIZE,
		EEPROM_CORB_SIZE,
		EEPROM_CORF_SIZE,
		EEPROM_DEPTH1_SIZE,
		EEPROM_DEPTH_CTRL_SIZE,
		EEPROM_PLS_MOD_CTRL__SIZE,
		EEPROM_PLS_MOD_VAL_SIZE,
	};
	uint32_t reg_addr_to_array[EEPROM_CONFIG_DATA_MODE_MAX] = {
		TL_AFE_NLR_OFFSET_ADDR,
		TL_AFE_NLR_X0_ADDR,
		TL_AFE_NLR_XPWR_ADDR,
		TL_AFE_ZERO_OFFSET_ADDR,
		TL_AFE_DEPTH_SLOPE_ADDR,
		TL_AFE_DEPTH3_SLOPE_ADDR,
		TL_AFE_RATE_ADJUST_ADDR,
		TL_AFE_ALIGN_ADDR,
		TL_AFE_READ_SIZE0_ADDR,
		TL_AFE_READ_SIZE3_ADDR,
		TL_AFE_ROI_ADDR,
		TL_AFE_GRID_ADDR,
		TL_AFE_RAWNR_XPWR_ADDR,
		TL_AFE_RAWNR_BLTBL_ADDR,
		TL_AFE_RAWNR_MED_ADDR,
		TL_AFE_SAT_TH_ADDR,
		TL_AFE_RAWNR_BKTBL_ADDR,
		TL_AFE_CORING_ADDR,
		TL_AFE_CORB_ADDR,
		TL_AFE_CORF_ADDR,
		TL_AFE_DEPTH1_ADDR,
		TL_AFE_CONTROL_ADDR,
		TL_AFE_PLS_MOD_CTRL_ADDR,
		TL_AFE_PLS_MOD_VAL_ADDR,
	};
	struct cam_eeprom_config_common *eeprom_mode_array =
		(struct cam_eeprom_config_common *)kzalloc(
			sizeof(struct cam_eeprom_config_common)
			*EEPROM_CONFIG_DATA_MAX
			,GFP_KERNEL);
	if(eeprom_mode_array == NULL)
		return -1;
	i2c_list = list_node_create(
			&cam_eeprom_list_head.list_head_config[mode],2);
	if(i2c_list == NULL)
		goto free_mode;
	i2c_list->i2c_settings.reg_setting[0].reg_addr =
		TL_AFE_TAL_DETECTOR_EN_ADDR;
	i2c_list->i2c_settings.reg_setting[0].reg_data = 0x0;
	i2c_list->i2c_settings.reg_setting[1].reg_addr
		= TL_AFE_TAL_EN_ADDR;
	i2c_list->i2c_settings.reg_setting[1].reg_data = 0x0;

	for(i = 0;i < EEPROM_CONFIG_DATA_MODE_MAX; i++){
		eeprom_mode_array[i].reg_addr_from
			= reg_addr_from_array[i];
		eeprom_mode_array[i].data_size = data_size_array[i];
		eeprom_mode_array[i].reg_addr_to
			= reg_addr_to_array[i];
		if(i == TL_AFE_ZERO_OFFSET
				|| i == TL_AFE_DEPTH1
				|| i == TL_AFE_CONTROL
				|| i == TL_AFE_PLS_MOD_CTRL){
			eeprom_mode_array[i].op_code
				= CAM_SENSOR_I2C_WRITE_RANDOM;
		}
		else{
			eeprom_mode_array[i].op_code
				= CAM_SENSOR_I2C_WRITE_BURST;
		}
	}
	rc = eeprom_create_list_node(e_ctrl,eeprom_mode_array
			,EEPROM_CONFIG_DATA_MODE_MAX,mode);
	if(rc != 0)
		goto free_mode;
	rc = eeprom_create_exp_list_node(e_ctrl,mode);
	if(rc != 0)
		goto free_mode;
	rc = eeprom_create_ld_list_node(e_ctrl,mode);
	if(rc != 0)
		goto free_mode;
	rc = eeprom_create_mode_list_node(e_ctrl,mode);
	if(rc != 0)
		goto free_mode;

	cam_eeprom_list_head.resolution[mode] = 1;
	kfree(eeprom_mode_array);
	return 0;
free_mode:
	kfree(eeprom_mode_array);
	return -1;
}

int cam_eeprom_free_list_head(enum cam_eeprom_free cmd)
{
	uint32_t i;
	struct i2c_settings_list *i2c_list = NULL;
	switch(cmd){
	case LIST_HEAD_ALL:
		list_for_each_entry(i2c_list
				,&cam_eeprom_list_head.list_head_init,list){
			kfree(i2c_list->i2c_settings.reg_setting);
			kfree(i2c_list);
			i2c_list = NULL;
		}
		for(i = 0; i < EEPROM_MODE_DATA_NUM;i++)
		{
			list_for_each_entry(i2c_list,
					&cam_eeprom_list_head.list_head_config[i],list){
				kfree(i2c_list->i2c_settings.reg_setting);
				kfree(i2c_list);
				i2c_list = NULL;
			}
		}
		list_for_each_entry(i2c_list,
				&cam_eeprom_list_head.list_head_config_other,list){
			kfree(i2c_list->i2c_settings.reg_setting);
			kfree(i2c_list);
			i2c_list = NULL;
		}
		break;
	case LIST_HEAD_INITIAL:
		list_for_each_entry(i2c_list
				,&cam_eeprom_list_head.list_head_init,list){
			kfree(i2c_list->i2c_settings.reg_setting);
			kfree(i2c_list);
			i2c_list = NULL;
		}
		break;
	case LIST_HEAD_RESOLUTION:
		for(i = 0; i < EEPROM_MODE_DATA_NUM;i++)
		{
			list_for_each_entry(i2c_list,
					&cam_eeprom_list_head.list_head_config[i],list){
				kfree(i2c_list->i2c_settings.reg_setting);
				kfree(i2c_list);
				i2c_list = NULL;
			}
		}
		break;
	case LIST_HEAD_OTHER:
		list_for_each_entry(i2c_list,
				&cam_eeprom_list_head.list_head_config_other,list){
			kfree(i2c_list->i2c_settings.reg_setting);
			kfree(i2c_list);
			i2c_list = NULL;
		}
		break;
	default:
		CAM_ERR(CAM_EEPROM,"free list error/no this list");
		break;
	}
	return 0;
}

static int cam_eeprom_streamon_camera(struct cam_eeprom_ctrl_t *e_ctrl,
		uint16_t *gpo_out_stby_value,
		uint16_t *control_value)
{
	int i = 0;
	struct i2c_settings_list *i2c_list;
	*gpo_out_stby_value = (*gpo_out_stby_value & 0xFFBFU) | 0x0040U;

	i2c_list = list_node_create(
			&cam_eeprom_list_head.list_head_streamon,1);
	i2c_list->i2c_settings.reg_setting[0].reg_data
			= *gpo_out_stby_value;
	i2c_list->i2c_settings.reg_setting[0].reg_addr
			= 0xc08e;
	i2c_list->i2c_settings.delay = 3;
	i2c_list->op_code =  CAM_SENSOR_I2C_WRITE_RANDOM;

	if(if_support_multi_camera == true){
		i2c_list = list_node_create(
				&cam_eeprom_list_head.list_head_streamon,17);
	} else {
		i2c_list = list_node_create(
				&cam_eeprom_list_head.list_head_streamon,18);
	}

	*gpo_out_stby_value = (*gpo_out_stby_value & 0xFFFDU) | 0x0002U;
	i2c_list->i2c_settings.reg_setting[i].reg_data
		= *gpo_out_stby_value;
	i2c_list->i2c_settings.reg_setting[i++].reg_addr
		= 0xc08e;
	*control_value = (*control_value & 0xFFFEU) | 0x0001U;
	i2c_list->i2c_settings.reg_setting[i].reg_data
		= *control_value;
	i2c_list->i2c_settings.reg_setting[i++].reg_addr
		= 0xC300;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4C0;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x001C;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4C3;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x001C;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4D7;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0000;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4D5;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0002;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4DA;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0001;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4F0;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0000;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC427;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0003;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC427;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0001;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC427;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0000;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC426;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0030;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC426;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0010;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC426;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0000;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC423;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0080;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC431;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0080;

	if(if_support_multi_camera == true){
		i2c_list->i2c_settings.reg_setting[i].reg_data
			= 0xFFEB;
		i2c_list->i2c_settings.reg_setting[i++].reg_addr
			= 0x7C20;
	} else {
		i2c_list->i2c_settings.reg_setting[i].reg_data
			= 0x0007;
		i2c_list->i2c_settings.reg_setting[i++].reg_addr
			= 0x4001;
		i2c_list->i2c_settings.reg_setting[i].reg_data
			= 0x0004;
		i2c_list->i2c_settings.reg_setting[i++].reg_addr
			= 0x7C22;
	}
	cam_eeprom_list_head.streamon = 1;
	return 0;
}

static int cam_eeprom_streamoff_camera(struct cam_eeprom_ctrl_t *e_ctrl,
		uint16_t *control_value,
		uint16_t *gpo_out_stby_value)
{
	uint16_t i = 0;
	struct i2c_settings_list *i2c_list;

	if(if_support_multi_camera == true){
		i2c_list = list_node_create(
				&cam_eeprom_list_head.list_head_streamoff,15);
		i2c_list->i2c_settings.reg_setting[i].reg_data
			= 0xFFFB;
		i2c_list->i2c_settings.reg_setting[i++].reg_addr
			= 0x7C20;
	} else {
		i2c_list = list_node_create(
				&cam_eeprom_list_head.list_head_streamoff,14);
	}
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0x4001;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0004;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0x7C22;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0004;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC431;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0082;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC423;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0000;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC426;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0020;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC427;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0002;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4C0;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x003C;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4C3;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x003C;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4D5;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0003;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4DA;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0000;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4D7;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0001;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC4F0;
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= 0x0001;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC300;
		*control_value = (*control_value & 0xFFFE);
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= *control_value;
	i2c_list->i2c_settings.reg_setting[i].reg_addr
		= 0xC08E;
	*gpo_out_stby_value = (*gpo_out_stby_value & 0xFFFD);
	i2c_list->i2c_settings.reg_setting[i++].reg_data
		= *gpo_out_stby_value;
	i2c_list->i2c_settings.delay = 3;

	i2c_list = list_node_create(
			&cam_eeprom_list_head.list_head_streamoff,1);
	i2c_list->i2c_settings.reg_setting[0].reg_addr
			= 0xC08E;
	*gpo_out_stby_value = (*gpo_out_stby_value & 0xFFBF);
	i2c_list->i2c_settings.reg_setting[0].reg_data
		= *gpo_out_stby_value;
	i2c_list->op_code =  CAM_SENSOR_I2C_WRITE_RANDOM;
	cam_eeprom_list_head.streamoff = 1;
	return 0;
}
static int cam_eeprom_list_stream_on(struct cam_eeprom_ctrl_t *e_ctrl,
		uint16_t *gpo_out_stby_value,
		uint16_t *control_value)
{
	int ret;
	cam_eeprom_list_head_create(&cam_eeprom_list_head.list_head_streamon);
	ret = cam_eeprom_streamon_camera(e_ctrl,gpo_out_stby_value,control_value);
	return ret;
}

static int cam_eeprom_list_stream_off(struct cam_eeprom_ctrl_t *e_ctrl,
		uint16_t *control_value,
		uint16_t *gpo_out_stby_value)
{
	int ret;
	cam_eeprom_list_head_create(&cam_eeprom_list_head.list_head_streamoff);
	ret = cam_eeprom_streamoff_camera(e_ctrl,control_value,gpo_out_stby_value);
	return ret;
}

static void cam_eeprom_create_list(struct cam_eeprom_ctrl_t *e_ctrl,tl_dev_eeprom_pup *tof_eeprom)
{
	int i,ret;
//init data list
	ret = cam_eeprom_list_init_setting_data(e_ctrl,tof_eeprom);
	if(ret != 0){
		cam_eeprom_list_head.initial = 0;
		CAM_ERR(CAM_EEPROM,"error create initial list");
		cam_eeprom_free_list_head(LIST_HEAD_INITIAL);
	}
	for(i = 0;i < EEPROM_MODE_DATA_NUM; i++){
//common data list
		ret = cam_eeprom_list_common_data_offload(e_ctrl,i);
//mode data list
		if(ret == 0){
			ret = cam_eeprom_list_mode_data_offload(e_ctrl,i);
			if(ret != 0){
				cam_eeprom_list_head.resolution[i] = 0;
				CAM_ERR(CAM_EEPROM,"error create mode list");
				cam_eeprom_free_list_head(LIST_HEAD_RESOLUTION);
			}
		} else {
			cam_eeprom_list_head.resolution[i] = 0;
			CAM_ERR(CAM_EEPROM,"error create common list");
			cam_eeprom_free_list_head(LIST_HEAD_RESOLUTION);
		}
	}
	ret = cam_eeprom_list_other(e_ctrl);
	if(ret != 0){
		cam_eeprom_list_head.other = 0;
		CAM_ERR(CAM_EEPROM,"error create other list");
		cam_eeprom_free_list_head(LIST_HEAD_OTHER);
	}
	ret = cam_eeprom_list_stream_on(e_ctrl,&(tof_eeprom->gpo_out_stby_value),
			&tof_eeprom->control_value);
	if(ret != 0){
		cam_eeprom_list_head.streamon = 0;
		CAM_ERR(CAM_EEPROM,"error create stream on list");
		cam_eeprom_free_list_head(LIST_HEAD_STREAMON);
	}
	ret = cam_eeprom_list_stream_off(e_ctrl,
			&tof_eeprom->control_value,
			&tof_eeprom->gpo_out_stby_value);
	if(ret != 0){
		cam_eeprom_list_head.streamoff = 0;
		CAM_ERR(CAM_EEPROM,"error create stream off list");
		cam_eeprom_free_list_head(LIST_HEAD_STREAMOFF);
	}
}
/*for tof camera End*/

/**
 * cam_eeprom_read_memory() - read map data into buffer
 * @e_ctrl:     eeprom control struct
 * @block:      block to be read
 *
 * This function iterates through blocks stored in block->map, reads each
 * region and concatenate them into the pre-allocated block->mapdata
 */
static int cam_eeprom_read_memory(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_eeprom_memory_block_t *block)
{
	int                                rc = 0;
	int                                j;
	struct cam_sensor_i2c_reg_setting  i2c_reg_settings;
	struct cam_sensor_i2c_reg_array    i2c_reg_array;
	struct cam_eeprom_memory_map_t    *emap = block->map;
	struct cam_eeprom_soc_private     *eb_info;
	uint8_t                           *memptr = block->mapdata;

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "e_ctrl is NULL");
		return -EINVAL;
	}

	eb_info = (struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;

	for (j = 0; j < block->num_map; j++) {
		CAM_DBG(CAM_EEPROM, "slave-addr = 0x%X", emap[j].saddr);
		if (emap[j].saddr) {
			eb_info->i2c_info.slave_addr = emap[j].saddr;
			rc = cam_eeprom_update_i2c_info(e_ctrl,
				&eb_info->i2c_info);
			if (rc) {
				CAM_ERR(CAM_EEPROM,
					"failed: to update i2c info rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].page.valid_size) {
			i2c_reg_settings.addr_type = emap[j].page.addr_type;
			i2c_reg_settings.data_type = emap[j].page.data_type;
			i2c_reg_settings.size = 1;
			i2c_reg_array.reg_addr = emap[j].page.addr;
			i2c_reg_array.reg_data = emap[j].page.data;
			i2c_reg_array.delay = emap[j].page.delay;
			i2c_reg_settings.reg_setting = &i2c_reg_array;
			rc = camera_io_dev_write(&e_ctrl->io_master_info,
				&i2c_reg_settings);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "page write failed rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].pageen.valid_size) {
			i2c_reg_settings.addr_type = emap[j].pageen.addr_type;
			i2c_reg_settings.data_type = emap[j].pageen.data_type;
			i2c_reg_settings.size = 1;
			i2c_reg_array.reg_addr = emap[j].pageen.addr;
			i2c_reg_array.reg_data = emap[j].pageen.data;
			i2c_reg_array.delay = emap[j].pageen.delay;
			i2c_reg_settings.reg_setting = &i2c_reg_array;
			rc = camera_io_dev_write(&e_ctrl->io_master_info,
				&i2c_reg_settings);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "page enable failed rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].poll.valid_size) {
			rc = camera_io_dev_poll(&e_ctrl->io_master_info,
				emap[j].poll.addr, emap[j].poll.data,
				0, emap[j].poll.addr_type,
				emap[j].poll.data_type,
				emap[j].poll.delay);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "poll failed rc %d",
					rc);
				return rc;
			}
		}

		if (emap[j].mem.valid_size) {
			rc = camera_io_dev_read_seq(&e_ctrl->io_master_info,
				emap[j].mem.addr, memptr,
				emap[j].mem.addr_type,
				emap[j].mem.data_type,
				emap[j].mem.valid_size);
			if (rc) {
				CAM_ERR(CAM_EEPROM, "read failed rc %d",
					rc);
				return rc;
			}
			memptr += emap[j].mem.valid_size;
		}

		if (emap[j].pageen.valid_size) {
			i2c_reg_settings.addr_type = emap[j].pageen.addr_type;
			i2c_reg_settings.data_type = emap[j].pageen.data_type;
			i2c_reg_settings.size = 1;
			i2c_reg_array.reg_addr = emap[j].pageen.addr;
			i2c_reg_array.reg_data = 0;
			i2c_reg_array.delay = emap[j].pageen.delay;
			i2c_reg_settings.reg_setting = &i2c_reg_array;
			rc = camera_io_dev_write(&e_ctrl->io_master_info,
				&i2c_reg_settings);
			if (rc) {
				CAM_ERR(CAM_EEPROM,
					"page disable failed rc %d",
					rc);
				return rc;
			}
		}
	}
	return rc;
}

/**
 * cam_eeprom_power_up - Power up eeprom hardware
 * @e_ctrl:     ctrl structure
 * @power_info: power up/down info for eeprom
 *
 * Returns success or failure
 */
static int cam_eeprom_power_up(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_sensor_power_ctrl_t *power_info)
{
	int32_t                 rc = 0;
	struct cam_hw_soc_info *soc_info =
		&e_ctrl->soc_info;

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		&e_ctrl->soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_EEPROM,
			"failed to fill power up vreg params rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		&e_ctrl->soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_EEPROM,
			"failed to fill power down vreg params  rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed in eeprom power up rc %d", rc);
		return rc;
	}

	if (e_ctrl->io_master_info.master_type == CCI_MASTER) {
		rc = camera_io_init(&(e_ctrl->io_master_info));
		if (rc) {
			CAM_ERR(CAM_EEPROM, "cci_init failed");
			return -EINVAL;
		}
	}
	return rc;
}

/**
 * cam_eeprom_power_down - Power down eeprom hardware
 * @e_ctrl:    ctrl structure
 *
 * Returns success or failure
 */
static int cam_eeprom_power_down(struct cam_eeprom_ctrl_t *e_ctrl)
{
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info         *soc_info;
	struct cam_eeprom_soc_private  *soc_private;
	int                             rc = 0;

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "failed: e_ctrl %pK", e_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &e_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_EEPROM, "failed: power_info %pK", power_info);
		return -EINVAL;
	}
	rc = msm_camera_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "power down the core is failed:%d", rc);
		return rc;
	}

	if (e_ctrl->io_master_info.master_type == CCI_MASTER)
		camera_io_release(&(e_ctrl->io_master_info));

	return rc;
}

/**
 * cam_eeprom_match_id - match eeprom id
 * @e_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_eeprom_match_id(struct cam_eeprom_ctrl_t *e_ctrl)
{
	int                      rc;
	struct camera_io_master *client = &e_ctrl->io_master_info;
	uint8_t                  id[2];

	rc = cam_spi_query_id(client, 0, CAMERA_SENSOR_I2C_TYPE_WORD,
		&id[0], 2);
	if (rc)
		return rc;
	CAM_DBG(CAM_EEPROM, "read 0x%x 0x%x, check 0x%x 0x%x",
		id[0], id[1], client->spi_client->mfr_id0,
		client->spi_client->device_id0);
	if (id[0] != client->spi_client->mfr_id0
		|| id[1] != client->spi_client->device_id0)
		return -ENODEV;
	return 0;
}

/**
 * cam_eeprom_parse_read_memory_map - Parse memory map
 * @of_node:    device node
 * @e_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
int32_t cam_eeprom_parse_read_memory_map(struct device_node *of_node,
	struct cam_eeprom_ctrl_t *e_ctrl)
{
	int32_t                         rc = 0;
	struct cam_eeprom_soc_private  *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	if (!e_ctrl) {
		CAM_ERR(CAM_EEPROM, "failed: e_ctrl is NULL");
		return -EINVAL;
	}

	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	rc = cam_eeprom_parse_dt_memory_map(of_node, &e_ctrl->cal_data);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: eeprom dt parse rc %d", rc);
		return rc;
	}
	rc = cam_eeprom_power_up(e_ctrl, power_info);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "failed: eeprom power up rc %d", rc);
		goto data_mem_free;
	}

	e_ctrl->cam_eeprom_state = CAM_EEPROM_CONFIG;
	if (e_ctrl->eeprom_device_type == MSM_CAMERA_SPI_DEVICE) {
		rc = cam_eeprom_match_id(e_ctrl);
		if (rc) {
			CAM_DBG(CAM_EEPROM, "eeprom not matching %d", rc);
			goto power_down;
		}
	}
	rc = cam_eeprom_read_memory(e_ctrl, &e_ctrl->cal_data);
	if (rc) {
		CAM_ERR(CAM_EEPROM, "read_eeprom_memory failed");
		goto power_down;
	}

	rc = cam_eeprom_power_down(e_ctrl);
	if (rc)
		CAM_ERR(CAM_EEPROM, "failed: eeprom power down rc %d", rc);

	e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
	return rc;
power_down:
	cam_eeprom_power_down(e_ctrl);
data_mem_free:
	vfree(e_ctrl->cal_data.mapdata);
	vfree(e_ctrl->cal_data.map);
	e_ctrl->cal_data.num_data = 0;
	e_ctrl->cal_data.num_map = 0;
	e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
	return rc;
}

/**
 * cam_eeprom_get_dev_handle - get device handle
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_get_dev_handle(struct cam_eeprom_ctrl_t *e_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    eeprom_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (e_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_EEPROM, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&eeprom_acq_dev, (void __user *) cmd->handle,
		sizeof(eeprom_acq_dev))) {
		CAM_ERR(CAM_EEPROM,
			"EEPROM:ACQUIRE_DEV: copy from user failed");
		return -EFAULT;
	}

	bridge_params.session_hdl = eeprom_acq_dev.session_handle;
	bridge_params.ops = &e_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = e_ctrl;

	eeprom_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	e_ctrl->bridge_intf.device_hdl = eeprom_acq_dev.device_handle;
	e_ctrl->bridge_intf.session_hdl = eeprom_acq_dev.session_handle;

	CAM_DBG(CAM_EEPROM, "Device Handle: %d", eeprom_acq_dev.device_handle);
	if (copy_to_user((void __user *) cmd->handle, &eeprom_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_EEPROM, "EEPROM:ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

/**
 * cam_eeprom_update_slaveInfo - Update slave info
 * @e_ctrl:     ctrl structure
 * @cmd_buf:    command buffer
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_update_slaveInfo(struct cam_eeprom_ctrl_t *e_ctrl,
	void *cmd_buf)
{
	int32_t                         rc = 0;
	struct cam_eeprom_soc_private  *soc_private;
	struct cam_cmd_i2c_info        *cmd_i2c_info = NULL;

	soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	cmd_i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
	soc_private->i2c_info.slave_addr = cmd_i2c_info->slave_addr;
	soc_private->i2c_info.i2c_freq_mode = cmd_i2c_info->i2c_freq_mode;

	rc = cam_eeprom_update_i2c_info(e_ctrl,
		&soc_private->i2c_info);
	CAM_DBG(CAM_EEPROM, "Slave addr: 0x%x Freq Mode: %d",
		soc_private->i2c_info.slave_addr,
		soc_private->i2c_info.i2c_freq_mode);

	return rc;
}

/**
 * cam_eeprom_parse_memory_map - Parse memory map info
 * @data:             memory block data
 * @cmd_buf:          command buffer
 * @cmd_length:       command buffer length
 * @num_map:          memory map size
 * @cmd_length_bytes: command length processed in this function
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_parse_memory_map(
	struct cam_eeprom_memory_block_t *data,
	void *cmd_buf, int cmd_length, uint16_t *cmd_length_bytes,
	int *num_map)
{
	int32_t                            rc = 0;
	int32_t                            cnt = 0;
	int32_t                            processed_size = 0;
	uint8_t                            generic_op_code;
	struct cam_eeprom_memory_map_t    *map = data->map;
	struct common_header              *cmm_hdr =
		(struct common_header *)cmd_buf;
	uint16_t                           cmd_length_in_bytes = 0;
	struct cam_cmd_i2c_random_wr      *i2c_random_wr = NULL;
	struct cam_cmd_i2c_continuous_rd  *i2c_cont_rd = NULL;
	struct cam_cmd_conditional_wait   *i2c_poll = NULL;
	struct cam_cmd_unconditional_wait *i2c_uncond_wait = NULL;

	generic_op_code = cmm_hdr->third_byte;
	switch (cmm_hdr->cmd_type) {
	case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR:
		i2c_random_wr = (struct cam_cmd_i2c_random_wr *)cmd_buf;
		cmd_length_in_bytes   = sizeof(struct cam_cmd_i2c_random_wr) +
			((i2c_random_wr->header.count - 1) *
			sizeof(struct i2c_random_wr_payload));

		for (cnt = 0; cnt < (i2c_random_wr->header.count);
			cnt++) {
			map[*num_map + cnt].page.addr =
				i2c_random_wr->random_wr_payload[cnt].reg_addr;
			map[*num_map + cnt].page.addr_type =
				i2c_random_wr->header.addr_type;
			map[*num_map + cnt].page.data =
				i2c_random_wr->random_wr_payload[cnt].reg_data;
			map[*num_map + cnt].page.data_type =
				i2c_random_wr->header.data_type;
			map[*num_map + cnt].page.valid_size = 1;
		}

		*num_map += (i2c_random_wr->header.count - 1);
		cmd_buf += cmd_length_in_bytes / sizeof(int32_t);
		processed_size +=
			cmd_length_in_bytes;
		break;
	case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_RD:
		i2c_cont_rd = (struct cam_cmd_i2c_continuous_rd *)cmd_buf;
		cmd_length_in_bytes = sizeof(struct cam_cmd_i2c_continuous_rd);

		map[*num_map].mem.addr = i2c_cont_rd->reg_addr;
		map[*num_map].mem.addr_type = i2c_cont_rd->header.addr_type;
		map[*num_map].mem.data_type = i2c_cont_rd->header.data_type;
		map[*num_map].mem.valid_size =
			i2c_cont_rd->header.count;
		cmd_buf += cmd_length_in_bytes / sizeof(int32_t);
		processed_size +=
			cmd_length_in_bytes;
		data->num_data += map[*num_map].mem.valid_size;
		break;
	case CAMERA_SENSOR_CMD_TYPE_WAIT:
		if (generic_op_code ==
			CAMERA_SENSOR_WAIT_OP_HW_UCND ||
			generic_op_code ==
			CAMERA_SENSOR_WAIT_OP_SW_UCND) {
			i2c_uncond_wait =
				(struct cam_cmd_unconditional_wait *)cmd_buf;
			cmd_length_in_bytes =
				sizeof(struct cam_cmd_unconditional_wait);

			if (*num_map < 1) {
				CAM_ERR(CAM_EEPROM,
					"invalid map number, num_map=%d",
					*num_map);
				return -EINVAL;
			}

			/*
			 * Though delay is added all of them, but delay will
			 * be applicable to only one of them as only one of
			 * them will have valid_size set to >= 1.
			 */
			map[*num_map - 1].mem.delay = i2c_uncond_wait->delay;
			map[*num_map - 1].page.delay = i2c_uncond_wait->delay;
			map[*num_map - 1].pageen.delay = i2c_uncond_wait->delay;
		} else if (generic_op_code ==
			CAMERA_SENSOR_WAIT_OP_COND) {
			i2c_poll = (struct cam_cmd_conditional_wait *)cmd_buf;
			cmd_length_in_bytes =
				sizeof(struct cam_cmd_conditional_wait);

			map[*num_map].poll.addr = i2c_poll->reg_addr;
			map[*num_map].poll.addr_type = i2c_poll->addr_type;
			map[*num_map].poll.data = i2c_poll->reg_data;
			map[*num_map].poll.data_type = i2c_poll->data_type;
			map[*num_map].poll.delay = i2c_poll->timeout;
			map[*num_map].poll.valid_size = 1;
		}
		cmd_buf += cmd_length_in_bytes / sizeof(int32_t);
		processed_size +=
			cmd_length_in_bytes;
		break;
	default:
		break;
	}

	*cmd_length_bytes = processed_size;
	return rc;
}

/**
 * cam_eeprom_init_pkt_parser - Parse eeprom packet
 * @e_ctrl:       ctrl structure
 * @csl_packet:	  csl packet received
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_init_pkt_parser(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_packet *csl_packet)
{
	int32_t                         rc = 0;
	int                             i = 0;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uint32_t                       *offset = NULL;
	uint32_t                       *cmd_buf = NULL;
	uint64_t                        generic_pkt_addr;
	size_t                          pkt_len = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	uint32_t                        processed_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uint16_t                        cmd_length_in_bytes = 0;
	struct cam_cmd_i2c_info        *i2c_info = NULL;
	int                             num_map = -1;
	struct cam_eeprom_memory_map_t *map = NULL;
	struct cam_eeprom_soc_private  *soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	e_ctrl->cal_data.map = vzalloc((MSM_EEPROM_MEMORY_MAP_MAX_SIZE *
		MSM_EEPROM_MAX_MEM_MAP_CNT) *
		(sizeof(struct cam_eeprom_memory_map_t)));
	if (!e_ctrl->cal_data.map) {
		rc = -ENOMEM;
		CAM_ERR(CAM_EEPROM, "failed");
		return rc;
	}
	map = e_ctrl->cal_data.map;

	offset = (uint32_t *)&csl_packet->payload;
	offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);

	/* Loop through multiple command buffers */
	for (i = 0; i < csl_packet->num_cmd_buf; i++) {
		total_cmd_buf_in_bytes = cmd_desc[i].length;
		processed_cmd_buf_in_bytes = 0;
		if (!total_cmd_buf_in_bytes)
			continue;
		rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
			(uint64_t *)&generic_pkt_addr, &pkt_len);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "Failed to get cpu buf");
			return rc;
		}
		cmd_buf = (uint32_t *)generic_pkt_addr;
		if (!cmd_buf) {
			CAM_ERR(CAM_EEPROM, "invalid cmd buf");
			return -EINVAL;
		}
		cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
		/* Loop through multiple cmd formats in one cmd buffer */
		while (processed_cmd_buf_in_bytes < total_cmd_buf_in_bytes) {
			cmm_hdr = (struct common_header *)cmd_buf;
			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
				/* Configure the following map slave address */
				map[num_map + 1].saddr = i2c_info->slave_addr;
				rc = cam_eeprom_update_slaveInfo(e_ctrl,
					cmd_buf);
				cmd_length_in_bytes =
					sizeof(struct cam_cmd_i2c_info);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
					sizeof(uint32_t);
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				cmd_length_in_bytes = total_cmd_buf_in_bytes;
				rc = cam_sensor_update_power_settings(cmd_buf,
					cmd_length_in_bytes, power_info);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/
					sizeof(uint32_t);
				if (rc) {
					CAM_ERR(CAM_EEPROM, "Failed");
					return rc;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_I2C_RNDM_WR:
			case CAMERA_SENSOR_CMD_TYPE_I2C_CONT_RD:
			case CAMERA_SENSOR_CMD_TYPE_WAIT:
				num_map++;
				rc = cam_eeprom_parse_memory_map(
					&e_ctrl->cal_data, cmd_buf,
					total_cmd_buf_in_bytes,
					&cmd_length_in_bytes, &num_map);
				processed_cmd_buf_in_bytes +=
					cmd_length_in_bytes;
				cmd_buf += cmd_length_in_bytes/sizeof(uint32_t);
				break;
			default:
				break;
			}
		}
		e_ctrl->cal_data.num_map = num_map + 1;
	}
	return rc;
}

/**
 * cam_eeprom_get_cal_data - parse the userspace IO config and
 *                                        copy read data to share with userspace
 * @e_ctrl:     ctrl structure
 * @csl_packet: csl packet received
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_get_cal_data(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_packet *csl_packet)
{
	struct cam_buf_io_cfg *io_cfg;
	uint32_t              i = 0;
	int                   rc = 0;
	uint64_t              buf_addr;
	size_t                buf_size;
	uint8_t               *read_buffer;

	io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
		&csl_packet->payload +
		csl_packet->io_configs_offset);

	CAM_DBG(CAM_EEPROM, "number of IO configs: %d:",
		csl_packet->num_io_configs);

	for (i = 0; i < csl_packet->num_io_configs; i++) {
		CAM_DBG(CAM_EEPROM, "Direction: %d:", io_cfg->direction);
		if (io_cfg->direction == CAM_BUF_OUTPUT) {
			rc = cam_mem_get_cpu_buf(io_cfg->mem_handle[0],
				(uint64_t *)&buf_addr, &buf_size);
			CAM_DBG(CAM_EEPROM, "buf_addr : %pK, buf_size : %zu\n",
				(void *)buf_addr, buf_size);

			read_buffer = (uint8_t *)buf_addr;
			if (!read_buffer) {
				CAM_ERR(CAM_EEPROM,
					"invalid buffer to copy data");
				return -EINVAL;
			}
			read_buffer += io_cfg->offsets[0];

			if (buf_size < e_ctrl->cal_data.num_data) {
				CAM_ERR(CAM_EEPROM,
					"failed to copy, Invalid size");
				return -EINVAL;
			}

			CAM_DBG(CAM_EEPROM, "copy the data, len:%d",
				e_ctrl->cal_data.num_data);
			memcpy(read_buffer, e_ctrl->cal_data.mapdata,
					e_ctrl->cal_data.num_data);

		} else {
			CAM_ERR(CAM_EEPROM, "Invalid direction");
			rc = -EINVAL;
		}
	}
	return rc;
}

/**
 * cam_eeprom_pkt_parse - Parse csl packet
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int32_t cam_eeprom_pkt_parse(struct cam_eeprom_ctrl_t *e_ctrl, void *arg)
{
	int32_t                         rc = 0;
	/*for tof camera Begin*/
	tl_dev_eeprom_pup			   *tof_eeprom = NULL;
	/*for tof camera End*/
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	uint64_t                        generic_pkt_addr;
	size_t                          pkt_len;
	struct cam_packet              *csl_packet = NULL;
	struct cam_eeprom_soc_private  *soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	ioctl_ctrl = (struct cam_control *)arg;

	if (copy_from_user(&dev_config, (void __user *) ioctl_ctrl->handle,
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		(uint64_t *)&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_EEPROM,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	if (dev_config.offset > pkt_len) {
		CAM_ERR(CAM_EEPROM,
			"Offset is out of bound: off: %lld, %zu",
			dev_config.offset, pkt_len);
		return -EINVAL;
	}

	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + dev_config.offset);
	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_EEPROM_PACKET_OPCODE_INIT:
		if (e_ctrl->userspace_probe == false) {
			rc = cam_eeprom_parse_read_memory_map(
					e_ctrl->soc_info.dev->of_node, e_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_EEPROM, "Failed: rc : %d", rc);
				return rc;
			}
			rc = cam_eeprom_get_cal_data(e_ctrl, csl_packet);
			vfree(e_ctrl->cal_data.mapdata);
			vfree(e_ctrl->cal_data.map);
			e_ctrl->cal_data.num_data = 0;
			e_ctrl->cal_data.num_map = 0;
			CAM_DBG(CAM_EEPROM,
				"Returning the data using kernel probe");
			break;
		}
		rc = cam_eeprom_init_pkt_parser(e_ctrl, csl_packet);
		if (rc) {
			CAM_ERR(CAM_EEPROM,
				"Failed in parsing the pkt");
			return rc;
		}

		e_ctrl->cal_data.mapdata =
			vzalloc(e_ctrl->cal_data.num_data);
		if (!e_ctrl->cal_data.mapdata) {
			rc = -ENOMEM;
			CAM_ERR(CAM_EEPROM, "failed");
			goto error;
		}

		rc = cam_eeprom_power_up(e_ctrl,
			&soc_private->power_info);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "failed rc %d", rc);
			goto memdata_free;
		}

		e_ctrl->cam_eeprom_state = CAM_EEPROM_CONFIG;
		rc = cam_eeprom_read_memory(e_ctrl, &e_ctrl->cal_data);
		if (rc) {
			CAM_ERR(CAM_EEPROM,
				"read_eeprom_memory failed");
			goto power_down;
		}
		/*for tof camera Begin*/
		tof_eeprom = cam_eeprom_module_offload(e_ctrl,e_ctrl->cal_data.mapdata);
		if(tof_eeprom != NULL){
			cam_eeprom_create_list(e_ctrl,tof_eeprom);
		} else {
			CAM_ERR(CAM_EEPROM,"can`t create list");
		}
		//	show();
		/*for tof camera End*/

		rc = cam_eeprom_get_cal_data(e_ctrl, csl_packet);
		rc = cam_eeprom_power_down(e_ctrl);
		e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
		vfree(e_ctrl->cal_data.mapdata);
		vfree(e_ctrl->cal_data.map);
		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		e_ctrl->cal_data.num_data = 0;
		e_ctrl->cal_data.num_map = 0;
		break;
	default:
		break;
	}
	return rc;
power_down:
	cam_eeprom_power_down(e_ctrl);
memdata_free:
	vfree(e_ctrl->cal_data.mapdata);
error:
	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	vfree(e_ctrl->cal_data.map);
	e_ctrl->cal_data.num_data = 0;
	e_ctrl->cal_data.num_map = 0;
	e_ctrl->cam_eeprom_state = CAM_EEPROM_INIT;
	return rc;
}

void cam_eeprom_shutdown(struct cam_eeprom_ctrl_t *e_ctrl)
{
	int rc;
	struct cam_eeprom_soc_private  *soc_private =
		(struct cam_eeprom_soc_private *)e_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (e_ctrl->cam_eeprom_state == CAM_EEPROM_INIT)
		return;

	if (e_ctrl->cam_eeprom_state == CAM_EEPROM_CONFIG) {
		rc = cam_eeprom_power_down(e_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_EEPROM, "EEPROM Power down failed");
		e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
	}

	if (e_ctrl->cam_eeprom_state == CAM_EEPROM_ACQUIRE) {
		rc = cam_destroy_device_hdl(e_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_EEPROM, "destroying the device hdl");

		e_ctrl->bridge_intf.device_hdl = -1;
		e_ctrl->bridge_intf.link_hdl = -1;
		e_ctrl->bridge_intf.session_hdl = -1;

		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
	}

	e_ctrl->cam_eeprom_state = CAM_EEPROM_INIT;
}

/**
 * cam_eeprom_driver_cmd - Handle eeprom cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int32_t cam_eeprom_driver_cmd(struct cam_eeprom_ctrl_t *e_ctrl, void *arg)
{
	int                            rc = 0;
	struct cam_eeprom_query_cap_t  eeprom_cap = {0};
	struct cam_control            *cmd = (struct cam_control *)arg;

	if (!e_ctrl || !cmd) {
		CAM_ERR(CAM_EEPROM, "Invalid Arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_EEPROM, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	mutex_lock(&(e_ctrl->eeprom_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		eeprom_cap.slot_info = e_ctrl->soc_info.index;
		if (e_ctrl->userspace_probe == false)
			eeprom_cap.eeprom_kernel_probe = true;
		else
			eeprom_cap.eeprom_kernel_probe = false;

		if (copy_to_user((void __user *) cmd->handle,
			&eeprom_cap,
			sizeof(struct cam_eeprom_query_cap_t))) {
			CAM_ERR(CAM_EEPROM, "Failed Copy to User");
			return -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_EEPROM, "eeprom_cap: ID: %d", eeprom_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_eeprom_get_dev_handle(e_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "Failed to acquire dev");
			goto release_mutex;
		}
		e_ctrl->cam_eeprom_state = CAM_EEPROM_ACQUIRE;
		break;
	case CAM_RELEASE_DEV:
		if (e_ctrl->cam_eeprom_state != CAM_EEPROM_ACQUIRE) {
			rc = -EINVAL;
			CAM_WARN(CAM_EEPROM,
			"Not in right state to release : %d",
			e_ctrl->cam_eeprom_state);
			goto release_mutex;
		}

		if (e_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_EEPROM,
				"Invalid Handles: link hdl: %d device hdl: %d",
				e_ctrl->bridge_intf.device_hdl,
				e_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(e_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_EEPROM,
				"failed in destroying the device hdl");
		e_ctrl->bridge_intf.device_hdl = -1;
		e_ctrl->bridge_intf.link_hdl = -1;
		e_ctrl->bridge_intf.session_hdl = -1;
		e_ctrl->cam_eeprom_state = CAM_EEPROM_INIT;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_eeprom_pkt_parse(e_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_EEPROM, "Failed in eeprom pkt Parsing");
			goto release_mutex;
		}
		break;
	default:
		CAM_DBG(CAM_EEPROM, "invalid opcode");
		break;
	}

release_mutex:
	mutex_unlock(&(e_ctrl->eeprom_mutex));

	return rc;
}

