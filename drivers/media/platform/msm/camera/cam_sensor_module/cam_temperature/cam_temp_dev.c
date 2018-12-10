#include <linux/module.h>
#include "cam_temp_core.h"

struct v4l2_subdev *tmp_sd = NULL;

static void show(struct list_head *list_head)
{
	uint32_t index = 0;
	uint32_t i = 0;
	struct i2c_settings_list *i2c_list;

	list_for_each_entry(i2c_list,list_head,list) {
		CAM_ERR(CAM_TEMPERATURE,"INIT:size = %#x,op_code = %#x",
				i2c_list->i2c_settings.size,i2c_list->op_code);
		for(i = 0;i < i2c_list->i2c_settings.size; i++){
			CAM_ERR(CAM_TEMPERATURE,
					"index = %#x,reg_addr = %#x,reg_data = %#x",
					index++,
					i2c_list->i2c_settings.reg_setting[i].reg_addr,
					i2c_list->i2c_settings.reg_setting[i].reg_data);
		}
	}
}


int init_temperature_setting(int cmd)
{
	struct cam_tmp_ctrl *t_ctrl =
		v4l2_get_subdevdata(tmp_sd);
//	struct i2c_settings_list *i2c_list;
	int32_t rc;
	int i;
	uint32_t reg_data;
	struct cam_sensor_i2c_reg_array tl_therm_init_data[3];
	struct cam_sensor_i2c_reg_setting therm_write;
#if 0
	list_for_each_entry(i2c_list,
		&(t_ctrl->init_settings.list_head), list) {
		rc = cam_sensor_i2c_modes_util(
			&(t_ctrl->io_master_info),
			i2c_list);
		if (rc < 0) {
			CAM_ERR(CAM_TEMPERATURE,
				"Failed to apply settings: %d",
				rc);
			return rc;
		}
	}
#endif
	if(cmd == 1){
		/* Therm low Register */
		tl_therm_init_data[0].reg_addr = 0x02U;
		tl_therm_init_data[0].reg_data = 0xE480U;
		tl_therm_init_data[0].delay = 0x00U;
		tl_therm_init_data[0].data_mask = 0x00U;
		/* Therm high Register */
		tl_therm_init_data[1].reg_addr = 0x03U;
		tl_therm_init_data[1].reg_data = 0x4B00U;
		tl_therm_init_data[1].delay = 0x00U;
		tl_therm_init_data[1].data_mask = 0x00U;
		 /* Configuration Register */
		tl_therm_init_data[2].reg_addr = 0x01U;
		tl_therm_init_data[2].reg_data = 0x78B0U;
		tl_therm_init_data[2].delay = 0x00U;
		tl_therm_init_data[2].data_mask = 0x00U;
	} else if (cmd == 0){
		/* Therm low Register */
		tl_therm_init_data[0].reg_addr = 0x02U;
		tl_therm_init_data[0].reg_data = 0x4B00U;
		tl_therm_init_data[0].delay = 0x00U;
		tl_therm_init_data[0].data_mask = 0x00U;
		/* Therm high Register */
		tl_therm_init_data[1].reg_addr = 0x03U;
		tl_therm_init_data[1].reg_data = 0x5000U;
		tl_therm_init_data[1].delay = 0x00U;
		tl_therm_init_data[1].data_mask = 0x00U;
		 /* Configuration Register */
		tl_therm_init_data[2].reg_addr = 0x01U;
		tl_therm_init_data[2].reg_data = 0x60a0U;
		tl_therm_init_data[2].delay = 0x00U;
		tl_therm_init_data[2].data_mask = 0x00U;
	}

    for(i = 0;i <3;i++) {
		therm_write.size = 1;
        therm_write.reg_setting = &tl_therm_init_data[i];
        therm_write.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
        therm_write.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
        therm_write.delay = 0;
        camera_io_dev_write_continuous(&(t_ctrl->io_master_info),&therm_write,1);
	}

	return 0;
}

static int32_t cam_sensor_therm_convert_temperature(uint32_t value)
{
	int32_t   sign, tmp;
	uint32_t  comp, t;

	//conversionsrt temperature
	sign = ((value & 0x8000U) == 0U) ? 1 : -1;
	comp = (sign == 1) ? value : ((uint32_t)~value + 1U);
	t = (comp & 0x7FE0U) >> 3U;
	tmp = sign * (((int32_t)t * 625 ) / 100);
	return tmp;
}

int read_tof_temperature(void)
{
	struct cam_tmp_ctrl *t_ctrl =
		v4l2_get_subdevdata(tmp_sd);
	uint32_t reg_val;
	uint16_t temperature = 0;

	camera_io_dev_read(&(t_ctrl->io_master_info)
			,0x00,&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
	temperature = cam_sensor_therm_convert_temperature(reg_val);
	return temperature;
}

static int cam_temperature_init_settings(struct list_head *list_head_init)
{
	//uint16_t data_num =3;
	struct i2c_settings_list *node = NULL;

#if 0
	node = (struct i2c_settings_list *)kzalloc(
			sizeof(struct i2c_settings_list),GFP_KERNEL);
	if (node != NULL)
		list_add_tail(&(node->list),list_head_init);
	else
		goto free_error;

	node->op_code =CAM_SENSOR_I2C_WRITE_BURST;
	node->i2c_settings.size = data_num;
	node->i2c_settings.reg_setting
		= (struct cam_sensor_i2c_reg_array *)kzalloc(
			sizeof(struct cam_sensor_i2c_reg_array)*data_num,GFP_KERNEL);
	if(node->i2c_settings.reg_setting == NULL){
		goto free_node;
	}
	node->i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	node->i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;

	node->i2c_settings.reg_setting[0].reg_addr = 0x02U;
	node->i2c_settings.reg_setting[0].reg_data = 0xE480U;
	node->i2c_settings.reg_setting[1].reg_addr = 0x03U;
	node->i2c_settings.reg_setting[1].reg_data = 0x4B00U;
	node->i2c_settings.reg_setting[2].reg_addr = 0x01U;
	node->i2c_settings.reg_setting[2].reg_data = 0x78B0U;
#endif

	node = (struct i2c_settings_list *)kzalloc(
			sizeof(struct i2c_settings_list),GFP_KERNEL);
	if (node != NULL)
		list_add_tail(&(node->list),list_head_init);
	else
		goto free_error;

	node->op_code =CAM_SENSOR_I2C_WRITE_BURST;
	node->i2c_settings.size = 1;
	node->i2c_settings.reg_setting
		= (struct cam_sensor_i2c_reg_array *)kzalloc(
			sizeof(struct cam_sensor_i2c_reg_array),GFP_KERNEL);
	if(node->i2c_settings.reg_setting == NULL){
		goto free_node;
	}
	node->i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	node->i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;

	node->i2c_settings.reg_setting[0].reg_addr = 0x02U;
	node->i2c_settings.reg_setting[0].reg_data = 0xE480U;

	node = (struct i2c_settings_list *)kzalloc(
			sizeof(struct i2c_settings_list),GFP_KERNEL);
	if (node != NULL)
		list_add_tail(&(node->list),list_head_init);
	else
		goto free_error;

	node->op_code =CAM_SENSOR_I2C_WRITE_BURST;
	node->i2c_settings.size = 1;
	node->i2c_settings.reg_setting
		= (struct cam_sensor_i2c_reg_array *)kzalloc(
			sizeof(struct cam_sensor_i2c_reg_array),GFP_KERNEL);
	if(node->i2c_settings.reg_setting == NULL){
		goto free_node;
	}
	node->i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	node->i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;

	node->i2c_settings.reg_setting[0].reg_addr = 0x03U;
	node->i2c_settings.reg_setting[0].reg_data = 0x4B00U;

	node = (struct i2c_settings_list *)kzalloc(
			sizeof(struct i2c_settings_list),GFP_KERNEL);
	if (node != NULL)
		list_add_tail(&(node->list),list_head_init);
	else
		goto free_error;

	node->op_code =CAM_SENSOR_I2C_WRITE_BURST;
	node->i2c_settings.size = 1;
	node->i2c_settings.reg_setting
		= (struct cam_sensor_i2c_reg_array *)kzalloc(
			sizeof(struct cam_sensor_i2c_reg_array),GFP_KERNEL);
	if(node->i2c_settings.reg_setting == NULL){
		goto free_node;
	}
	node->i2c_settings.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	node->i2c_settings.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;

	node->i2c_settings.reg_setting[0].reg_addr = 0x01U;
	node->i2c_settings.reg_setting[0].reg_data = 0x78B0U;
	return 0;
free_node:
	kfree(node);
free_error:
	return -1;
}

#if 1
static long cam_temperature_subdev_ioctl(struct v4l2_subdev *sd,
		unsigned int cmd,void *arg)
{
	int rc = 0;
	struct cam_tmp_ctrl *t_ctrl =
		v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_temperature_driver_cmd(t_ctrl, arg);
		break;
	default:
		CAM_ERR(CAM_TEMPERATURE, "Invalid ioctl cmd");
		CAM_ERR(CAM_TEMPERATURE,"%d",cmd);
		rc = -EINVAL;
		break;
	}
	return rc;
}

#ifdef CONFIG_COMPAT
static long cam_temperature_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_TEMPERATURE,
			"Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		cmd = VIDIOC_CAM_CONTROL;
		rc = cam_temperature_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc) {
			CAM_ERR(CAM_TEMPERATURE,
				"Failed in actuator subdev handling rc: %d",
				rc);
			return rc;
		}
	break;
	default:
		CAM_ERR(CAM_TEMPERATURE, "Invalid compat ioctl: %d", cmd);
		rc = -EINVAL;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_TEMPERATURE,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}
	return rc;
}
#endif

#endif
static struct v4l2_subdev_core_ops cam_temperature_subdev_core_ops = {
	.ioctl = cam_temperature_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_temperature_subdev_do_ioctl,
#endif
};

static struct v4l2_subdev_ops cam_temperature_subdev_ops = {
	.core = &cam_temperature_subdev_core_ops,
};
static int cam_temperature_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_tmp_ctrl *t_ctrl =
		v4l2_get_subdevdata(sd);

	if (!t_ctrl) {
		CAM_ERR(CAM_TEMPERATURE, "temperature ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&t_ctrl->t_mutex);
	cam_temperature_shutdown(t_ctrl);
	mutex_unlock(&t_ctrl->t_mutex);

	return 0;
}


static const struct v4l2_subdev_internal_ops cam_temperature_internal_ops = {
	.close = cam_temperature_subdev_close,
};


static int32_t cam_temperature_driver_platform_probe(
	struct platform_device *pdev)
{
	int rc = 0;
	struct cam_tmp_ctrl *t_ctrl = kzalloc(sizeof(struct cam_tmp_ctrl),GFP_KERNEL);

	t_ctrl->pdev = pdev;

	t_ctrl->io_master_info.master_type = CCI_MASTER;
	t_ctrl->io_master_info.cci_client
		= kzalloc(sizeof(struct cam_sensor_cci_client),GFP_KERNEL);
	if(!(t_ctrl->io_master_info.cci_client)){
		rc = -ENOMEM;
		goto free_ctrl;
	}
	t_ctrl->io_master_info.cci_client->cci_i2c_master = MASTER_0;
	t_ctrl->io_master_info.cci_client->sid = 0x90 >> 1;
	t_ctrl->io_master_info.cci_client->retries = 3;
	t_ctrl->io_master_info.cci_client->id_map = 0;
	t_ctrl->io_master_info.cci_client->i2c_freq_mode = I2C_FAST_MODE;

	t_ctrl->v4l2_dev_str.pdev = pdev;
	t_ctrl->v4l2_dev_str.internal_ops =&cam_temperature_internal_ops;
	t_ctrl->v4l2_dev_str.ops = &cam_temperature_subdev_ops;
	t_ctrl->v4l2_dev_str.name = CAMX_TEMPERATURE_DEV_NAME;
	t_ctrl->v4l2_dev_str.sd_flags =
		V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	t_ctrl->v4l2_dev_str.ent_function = CAM_TEMPERATURE_DEVICE_TYPE;
	t_ctrl->v4l2_dev_str.token = t_ctrl;

	INIT_LIST_HEAD(&(t_ctrl->init_settings.list_head));

	rc = cam_register_subdev(&(t_ctrl->v4l2_dev_str));
	if (rc) {
		CAM_ERR(CAM_TEMPERATURE, "Fail to create subdev with %d", rc);
		goto free_resource;
	}
	show(&(t_ctrl->init_settings.list_head));
	platform_set_drvdata(pdev, t_ctrl);
	v4l2_set_subdevdata(&t_ctrl->v4l2_dev_str.sd, t_ctrl);

	tmp_sd = &t_ctrl->v4l2_dev_str.sd;
	t_ctrl->io_master_info.cci_client->cci_subdev = cam_cci_get_subdev();
	return rc;
free_resource:
	kfree(t_ctrl->io_master_info.cci_client);
free_ctrl:
	kfree(t_ctrl);
	return rc;
}

static int32_t cam_temperature_platform_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id cam_temperature_driver_dt_match[] = {
	{.compatible = "tof-tmp"},
	{},
};

MODULE_DEVICE_TABLE(of, cam_temperature_driver_dt_match);

static struct platform_driver cam_temperature_platform_driver = {
	.probe = cam_temperature_driver_platform_probe,
	.driver = {
		.name = "tof_temperature",
		.owner = THIS_MODULE,
		.of_match_table = cam_temperature_driver_dt_match,
	},
	.remove = cam_temperature_platform_remove,
};

static int __init cam_temperature_init_module(void)
{
	int32_t rc = 0;
	rc = platform_driver_register(&cam_temperature_platform_driver);
	if (rc < 0)
	{
		CAM_ERR(CAM_TEMPERATURE,
				"platform_driver_register failed rc = %d", rc);
		return rc;
	}

	return rc;
}

static void __exit cam_temperature_exit_module(void)
{
	return;
}

module_init(cam_temperature_init_module);
module_exit(cam_temperature_exit_module);
MODULE_DESCRIPTION("CAM TEMPERATURE");
MODULE_LICENSE("GPL v2");
