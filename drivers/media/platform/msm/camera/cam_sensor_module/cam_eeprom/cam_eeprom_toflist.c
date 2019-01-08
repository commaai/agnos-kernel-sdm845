#include "tl_dev_eeprom.h"

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
//	CAM_INFO(CAM_EEPROM,"get EEPROM data");
	if(mode < TL_E_MODE_MAX)
	{
		if(type == EEPROM_INIT_DATA &&
				cam_eeprom_list_head.initial == 1) {
			CAM_INFO(CAM_EEPROM,"initial EEPROM data");
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
			CAM_INFO(CAM_EEPROM,"mode[%d] EEPROM data",mode);
			reg_settings->prev
				= cam_eeprom_list_head.list_head_config[mode].prev;
			reg_settings->next
				= cam_eeprom_list_head.list_head_config[mode].next;
			cam_eeprom_list_head.list_head_config[mode].next->prev
				= reg_settings;
			cam_eeprom_list_head.list_head_config[mode].prev->next
				= reg_settings;
		} else if(type == EEPROM_STREAMON_DATA
				&& cam_eeprom_list_head.streamon == 1) {
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
	if(mode > TL_E_MODE_MAX)
		return -1;
	if(type == EEPROM_INIT_DATA &&
			cam_eeprom_list_head.initial == 1) {
		cam_eeprom_list_head.list_head_init.prev = reg_settings->prev;
		cam_eeprom_list_head.list_head_init.next = reg_settings->next;
		reg_settings->next->prev = &cam_eeprom_list_head.list_head_init;
		reg_settings->prev->next = &cam_eeprom_list_head.list_head_init;
		reg_settings->next = reg_settings;
		reg_settings->prev = reg_settings;
	} else if(type == EEPROM_CONFIG_DATA &&
				cam_eeprom_list_head.resolution[mode] == 1) {
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
	if(type == EEPROM_STREAMON_DATA &&
			cam_eeprom_list_head.streamon == 1) {
		cam_eeprom_list_head.list_head_streamon.prev = reg_settings->prev;
		cam_eeprom_list_head.list_head_streamon.next = reg_settings->next;
		reg_settings->next->prev = &cam_eeprom_list_head.list_head_streamon;
		reg_settings->prev->next = &cam_eeprom_list_head.list_head_streamon;
		reg_settings->next = reg_settings;
		reg_settings->prev = reg_settings;
	}
	if(type == EEPROM_STREAMOFF_DATA && cam_eeprom_list_head.streamoff == 1){
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
#if 0
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
#endif
#if 1
static int create_node_for_seq(uint16_t *pup_data
		,uint32_t data_num)
{
	uint32_t n                = 1;
	uint32_t i                = 0;
	uint32_t temp             = 0;

	struct i2c_settings_list *node = kzalloc(
			sizeof(struct i2c_settings_list),GFP_KERNEL);
	if (node != NULL)
		list_add_tail(&(node->list),&cam_eeprom_list_head.list_head_init);
	else
		return -1;

	temp = pup_data[n];

	node->i2c_settings.reg_setting
		= (struct cam_sensor_i2c_reg_array *)kzalloc(
			sizeof(struct cam_sensor_i2c_reg_array)*data_num,GFP_KERNEL);
	if(node->i2c_settings.reg_setting == NULL){
		return -1;
	}

	for(i = 0;i < data_num; i++){
		n += 1;
		node->i2c_settings.reg_setting[i].reg_data = pup_data[n];
		node->i2c_settings.reg_setting[i].reg_addr = temp + i;
	}
	node->i2c_settings.size = data_num;
	node->i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	node->i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	node->op_code = CAM_SENSOR_I2C_WRITE_SEQ;

	return 0;
}
#endif
#if 0
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
		if(data_num <= 0){
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
#endif
#if 1
static int cam_eeprom_list_init_setting_data(
		tl_dev_eeprom_pup *tof_eeprom)
{
	uint32_t  n                            = 0;
	int16_t   i                            = 0;
	int       rc                           = 0;
	struct    i2c_settings_list *i2c_list;
	uint32_t  data_num;
	uint16_t  pup_size = tof_eeprom->pup_size;
	uint32_t  reg_array_num,addr_temp,n_temp;
	uint16_t  *pup_data = tof_eeprom->pup_data;

	rc = cam_eeprom_list_head_create(
			&cam_eeprom_list_head.list_head_init);
	if(rc == -1){
		CAM_ERR(CAM_EEPROM,"list_head == NULL");
		return rc;
	}

	data_num = pup_data[n];
	if(data_num < 1){
		CAM_ERR(CAM_EEPROM,"have no powerup message");
		return -1;
	}
	rc = create_node_for_seq(pup_data,data_num);
	if(rc == -1){
		CAM_ERR(CAM_EEPROM,"SEQ == NULL");
		return -1;
	}

	n += data_num + 2; //data_num reg_addr reg_data ...
	while(n < pup_size)
	{
		data_num = pup_data[n];
		if(data_num <= 0){
			CAM_ERR(CAM_EEPROM,"data_num 0");
			return -1;
		} else if(data_num > 1){
			n += 1;
			addr_temp = pup_data[n];
			n += 1;
			while(data_num > 0){
				i2c_list = list_node_create(
						&cam_eeprom_list_head.list_head_init,1);
				if(i2c_list == NULL)
					return -1;
				i2c_list->i2c_settings.reg_setting[0].reg_data
					= pup_data[n];
				i2c_list->i2c_settings.reg_setting[0].reg_addr
					= addr_temp;
				if(addr_temp++ == 0xC08E){
					tof_eeprom->gpo_out_stby_value = pup_data[n];
				}
				n += 1;
				data_num--;
			}
		} else {
			n_temp = n;
			reg_array_num = 1;
			while(1)
			{
				n_temp += 3;
				if(n_temp < pup_size){
				if(pup_data[n_temp] != 1)
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
				n += 1;
				i2c_list->i2c_settings.reg_setting[i].reg_addr = pup_data[n];
				n += 1;
				i2c_list->i2c_settings.reg_setting[i].reg_data = pup_data[n];
				if(i2c_list->i2c_settings.reg_setting[i].reg_addr == 0xC08E){
					tof_eeprom->gpo_out_stby_value = pup_data[n];
				}
				n += 1;
			}
		}
	}
	cam_eeprom_list_head.initial = 1;
	return 0;
}
#endif
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
	uint16_t tof_mode_flag;
	//exp
	//  addr_num
	uint32_t eeprom_mode_addr
		= TL_EEPROM_MODE_TOP(mode);
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

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_TOF_MODE_FLAG
			,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD
			,CAMERA_SENSOR_I2C_TYPE_WORD);
	tof_mode_flag = reg_val;
	if((tof_mode_flag & TL_AFE_WDR) != 0){
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


static TL_ModeParam * cam_eeprom_tl_mode_param(uint32_t mode,
		uint16_t vd_ini_ofst_adr_num,uint16_t vd_ini_ofst,uint16_t idle_num,uint16_t idle)
{
	TL_ModeParam *modeparam = NULL;

	modeparam = (TL_ModeParam *)kzalloc(sizeof(TL_ModeParam),GFP_KERNEL);
	if(modeparam == NULL)
		return NULL;
	modeparam->mode = mode;
	if(vd_ini_ofst_adr_num == 0)
		modeparam->ini_ofst_delay = 0;
	else
		modeparam->ini_ofst_delay = vd_ini_ofst;
	if(idle_num == 0)
		modeparam->idle_delay = 0;
	else
		modeparam->idle_delay = idle;

	return modeparam;
}

static int read_eeprom_config_reg_data(struct cam_eeprom_ctrl_t *e_ctrl
		,struct cam_eeprom_exp_reg_data *reg_data_array
		,uint32_t mode)
{
	uint32_t addr_temp,i,reg_val,clk,afe_val;
	uint32_t pls_num = 0;
	int32_t tmp,dmy_hd,exp_hd,hd;
	struct cam_eeprom_config_exp_data prm;
	uint32_t eeprom_mode_addr;
	uint16_t IdlePeri,idle_delay,idle_num,ini_ofst_delay;
	int dummy;
	TL_ModeParam *mode_param = NULL;

	eeprom_mode_addr
		= TL_EEPROM_MODE_TOP(mode);

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
	idle_num = reg_val;
	if(reg_val > 4){
		return -1;
	}
	if(reg_val == 0){
		IdlePeri = 0;
	} else {
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,eeprom_mode_addr+TL_EEPROM_IDLE_PERI_ADR1,
				&reg_val
				,CAMERA_SENSOR_I2C_TYPE_WORD
				,CAMERA_SENSOR_I2C_TYPE_WORD);
		if(reg_val == 0){
			IdlePeri = 0;
		} else {
		camera_io_dev_read(&(e_ctrl->io_master_info)
				,reg_val,
				&afe_val
				,CAMERA_SENSOR_I2C_TYPE_WORD
				,CAMERA_SENSOR_I2C_TYPE_WORD);
			IdlePeri = afe_val + 2;
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
		addr_temp = prm.vd_ini_ofst;
	} else {
		mode_param = cam_eeprom_tl_mode_param(mode,
				prm.vd_ini_ofst_adr_num,
				prm.vd_ini_ofst,idle_num,
				prm.idle);

		idle_delay = mode_param->idle_delay;
		ini_ofst_delay = mode_param->ini_ofst_delay;

		prm.idle = (idle_delay == 0)?0:(idle_delay - 2U);
		reg_data_array->read_size2_data
			= prm.vd_ini_ofst + exp_hd + TL_AFE_READ_SIZE_OFFSET + idle_delay;
		reg_data_array->ccd_dummy_data = prm.vd_duration + dmy_hd - idle_delay;
		reg_data_array->start_v_data = exp_hd + TL_AFE_START_V_OFFSET;
		reg_data_array->vd_length_data = (prm.vd_duration * 2) - 2;
		addr_temp = ini_ofst_delay;
		dummy = prm.vd_duration - exp_hd + exp_hd +
			TL_AFE_READ_SIZE_OFFSET - ini_ofst_delay - idle_delay;
		CAM_ERR(CAM_EEPROM,"dummy = %d",dummy);
	}
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_VD_INI_OFST_ADR_NUM,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	prm.vd_ini_ofst_adr_num = reg_val;
#if 0
	if((prm.vd_ini_ofst >= 2) && prm.vd_ini_ofst_adr_num > 0){
		addr_temp = prm.vd_ini_ofst - 1;
		reg_data_array->vd_ini_ofst_data = addr_temp;
	}
#endif
	if(prm.vd_ini_ofst_adr_num > 0){
		if(addr_temp >= 2){
			addr_temp = addr_temp -1;
			reg_data_array->vd_ini_ofst_data = addr_temp;
		} else {
			reg_data_array->vd_ini_ofst_data = 0;
		}
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
		= TL_EEPROM_MODE_TOP(mode);

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,eeprom_mode_addr+TL_EEPROM_LD_FLAG,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	temp = reg_val;
	if((temp & 0x0001) == 1U){
		ld_setting |= 0x0001U;
	}
	if (((temp & 0x0010U) >> 4U) == 1U){
		ld_setting |= 0x0002U;
	}
	if (((temp & 0x0100U) >> 8U) == 1U){
		ld_setting |= 0x0004U;
	}
	if (((temp & 0x1000U) >> 12U) == 1U){
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
	return ld_setting;
}


static int eeprom_create_mode_list_node(
		struct cam_eeprom_ctrl_t *e_ctrl
		,uint32_t mode)
{
	uint16_t output_sel;
	uint32_t reg_val;
	struct i2c_settings_list *i2c_list;

	camera_io_dev_read(&(e_ctrl->io_master_info)
			,TL_EEPROM_OUTPUT_SEL,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	output_sel = reg_val;
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
		= output_sel;
	return 0;
}

static int eeprom_enable_tal_list(struct cam_eeprom_ctrl_t *e_ctrl
		,uint16_t tal_mode_flag,uint16_t ld_setting,uint16_t mode)
{
	struct    i2c_settings_list *i2c_list;
	uint16_t unData2 = ld_setting | (uint16_t)(ld_setting << 8U);

	if((tal_mode_flag & 0x0001U) == 0x0001U){
		/* LD rising edge TAL-valid */
	} else {
		/* LD rising edge TAL-invalid */
		unData2 &= 0xFFF0U;
	}
	if((tal_mode_flag & 0x0002U) == 0x0002U){
		/* LD falling edge TAL-valid */
	} else {
		/* LD falling edge TAL-invalid */
		unData2 &= 0xF0FFU;
	}
	if((tal_mode_flag & 0x0004U) == 0x0004U){
		/* SUB rising edge TAL-valid */
	} else {
		/* SUB rising edge TAL-invalid */
		unData2 &= 0x0FFFU;
	}
	if((tal_mode_flag & 0x0008U) == 0x0008U){
		/* SUB falling edge TAL-valid */
	} else {
		/* SUB falling edge TAL-invalid */
		unData2 &= 0xFF0FU;
	}
	i2c_list = list_node_create(
			&cam_eeprom_list_head.list_head_config[mode],2);
	if(i2c_list == NULL)
		return -1;
	i2c_list->i2c_settings.reg_setting[0].reg_addr = TL_AFE_TAL_EN_ADDR;
	i2c_list->i2c_settings.reg_setting[0].reg_data = unData2;
	i2c_list->i2c_settings.reg_setting[1].reg_addr = TL_AFE_TAL_DETECTOR_EN_ADDR;
	i2c_list->i2c_settings.reg_setting[1].reg_data = ld_setting;
	return 0;
}

static int cam_eeprom_list_mode_data_offload(
		struct cam_eeprom_ctrl_t *e_ctrl
		,uint32_t mode)
{
	struct    i2c_settings_list *i2c_list;
	int16_t  i             = 0;
	int      rc            = 0;
	uint32_t tal;
	uint16_t ld_setting;
	uint32_t  eeprom_mode_addr
		= TL_EEPROM_MODE_TOP(mode);
	uint32_t reg_addr_from_array[EEPROM_CONFIG_DATA_MODE_MAX] = {
		eeprom_mode_addr + TL_EEPROM_NLR_OFFSET,
		eeprom_mode_addr + TL_EEPROM_NLR_X0,
		eeprom_mode_addr + TL_EEPROM_NLR_XPWR,
		eeprom_mode_addr + TL_EEPROM_DEPTH0,
		eeprom_mode_addr + TL_EEPROM_DEPTH2,
		eeprom_mode_addr + TL_EEPROM_DEPTH3,
		eeprom_mode_addr + TL_EEPROM_RATE_ADJUST,
		eeprom_mode_addr + TL_EEPROM_ALIGN,
		eeprom_mode_addr + TL_EEPROM_READ_SIZE0,
		eeprom_mode_addr + TL_EEPROM_READ_SIZE3,
		eeprom_mode_addr + TL_EEPROM_ROI,
		eeprom_mode_addr + TL_EEPROM_GRID,
		eeprom_mode_addr + TL_EEPROM_RAWNR_XPWR,
		eeprom_mode_addr + TL_EEPROM_RAWNR_BL_TBL,
		eeprom_mode_addr + TL_EEPROM_RAWNR_MED,
		eeprom_mode_addr + TL_EEPROM_RAWNR_SAT_TH,
		eeprom_mode_addr + TL_EEPROM_RAWNR_BK_TBL,
		eeprom_mode_addr + TL_EEPROM_COR,
		eeprom_mode_addr + TL_EEPROM_CORB,
		eeprom_mode_addr + TL_EEPROM_CORF,
		eeprom_mode_addr + TL_EEPROM_DEPTH1,
		eeprom_mode_addr + TL_EEPROM_CONTROL,
		eeprom_mode_addr + TL_EEPROM_PLS_MOD_CTRL,
		eeprom_mode_addr + TL_EEPROM_PLS_MOD_VAL,
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
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,TL_EEPROM_TAL_MODE,&tal
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	if(tal != 0U){
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
	}
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
	ld_setting = eeprom_create_ld_list_node(e_ctrl,mode);
	rc = eeprom_create_mode_list_node(e_ctrl,mode);
	if(rc != 0)
		goto free_mode;
	if(tal != 0){
		rc = eeprom_enable_tal_list(e_ctrl,tal,ld_setting,mode);
		if(rc != 0)
			goto free_mode;
	}
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
		cam_eeprom_list_head.initial = 0;

		for(i = 0; i < TL_E_MODE_MAX;i++)
		{
			list_for_each_entry(i2c_list,
					&cam_eeprom_list_head.list_head_config[i],list){
				kfree(i2c_list->i2c_settings.reg_setting);
				kfree(i2c_list);
				i2c_list = NULL;
			}
			cam_eeprom_list_head.resolution[i] = 0;
		}

		list_for_each_entry(i2c_list
				,&cam_eeprom_list_head.list_head_streamon,list){
			kfree(i2c_list->i2c_settings.reg_setting);
			kfree(i2c_list);
			i2c_list = NULL;
		}
		cam_eeprom_list_head.streamon = 0;

		list_for_each_entry(i2c_list
				,&cam_eeprom_list_head.list_head_streamoff,list){
			kfree(i2c_list->i2c_settings.reg_setting);
			kfree(i2c_list);
			i2c_list = NULL;
		}
		cam_eeprom_list_head.streamoff = 0;
		break;
	case LIST_HEAD_INITIAL:
		list_for_each_entry(i2c_list
				,&cam_eeprom_list_head.list_head_init,list){
			kfree(i2c_list->i2c_settings.reg_setting);
			kfree(i2c_list);
			i2c_list = NULL;
		}
		cam_eeprom_list_head.initial = 0;
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
#if 1
	if(if_support_multi_camera == true){
		i2c_list = list_node_create(
				&cam_eeprom_list_head.list_head_streamon,18);
	} else {
		i2c_list = list_node_create(
				&cam_eeprom_list_head.list_head_streamon,19);
	}

	i2c_list->i2c_settings.reg_setting[i].reg_data
			= *gpo_out_stby_value;
	i2c_list->i2c_settings.reg_setting[i++].reg_addr
			= 0xc08e;
	i2c_list->i2c_settings.delay = 4;
#endif
#if 0
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
#endif
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

int cam_eeprom_create_list(struct cam_eeprom_ctrl_t *e_ctrl,tl_dev_eeprom_pup *tof_eeprom)
{
	int i,ret;
//init data list
	if(cam_eeprom_list_head.initial != 1) {
		ret = cam_eeprom_list_init_setting_data(tof_eeprom);
		if(ret != 0){
			cam_eeprom_list_head.initial = 0;
			CAM_ERR(CAM_EEPROM,"error create initial list");
			cam_eeprom_free_list_head(LIST_HEAD_INITIAL);
			return -1;
		}
		CAM_INFO(CAM_EEPROM,"create tof camera initial list success");
	} else {
		return -1;
	}
	for(i = 0;i < TL_E_MODE_MAX; i++){
//common data list
		if(cam_eeprom_list_head.resolution[i] != 1) {
			ret = cam_eeprom_list_common_data_offload(e_ctrl,i);
//mode data list
			if(ret == 0){
				ret = cam_eeprom_list_mode_data_offload(e_ctrl,i);
				if(ret != 0){
					cam_eeprom_list_head.resolution[i] = 0;
					CAM_ERR(CAM_EEPROM,"error create mode list");
					cam_eeprom_free_list_head(LIST_HEAD_ALL);
					return -1;
				}
			} else {
				cam_eeprom_list_head.resolution[i] = 0;
				CAM_ERR(CAM_EEPROM,"error create common list");
				cam_eeprom_free_list_head(LIST_HEAD_ALL);
				return -1;
			}
			CAM_INFO(CAM_EEPROM,"create tof camera mode[%d] list success",i);
		} else {
			return -1;
		}
	}
	if(cam_eeprom_list_head.streamon != 1) {
		ret = cam_eeprom_list_stream_on(e_ctrl,&(tof_eeprom->gpo_out_stby_value),
				&tof_eeprom->control_value);
		if(ret != 0){
			cam_eeprom_list_head.streamon = 0;
			CAM_ERR(CAM_EEPROM,"error create stream on list");
			cam_eeprom_free_list_head(LIST_HEAD_ALL);
			return -1;
		}
			CAM_INFO(CAM_EEPROM,"create tof camera stream on list success");
	} else {
		return -1;
	}
	if(cam_eeprom_list_head.streamoff != 1) {
		ret = cam_eeprom_list_stream_off(e_ctrl,
				&tof_eeprom->control_value,
				&tof_eeprom->gpo_out_stby_value);
		if(ret != 0){
			cam_eeprom_list_head.streamoff = 0;
			CAM_ERR(CAM_EEPROM,"error create stream off list");
			cam_eeprom_free_list_head(LIST_HEAD_ALL);
			return -1;
		}
		CAM_INFO(CAM_EEPROM,"create tof camera stream off list success");
	} else {
		return -1;
	}
	return 0;
}
/*for tof camera End*/
