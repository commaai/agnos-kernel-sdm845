#include <cam_sensor_io.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/irqreturn.h>
#include <linux/ion.h>
#include <linux/iommu.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <cam_cci_dev.h>
#include <cam_sensor_cmn_header.h>
#include <cam_subdev.h>
#include "cam_sensor_util.h"
#include "cam_soc_util.h"
#include "cam_debug_util.h"

#define CAMX_TEMPERATURE_DEV_NAME "cam-temperature-dev"

struct cam_tmp_ctrl {
	struct platform_device *pdev;
	struct cam_subdev v4l2_dev_str;
	struct mutex t_mutex;
	struct camera_io_master io_master_info;
	struct i2c_settings_array init_settings;
};


int init_temperature_setting(int cmd);
int read_tof_temperature(void);

extern int32_t cam_sensor_i2c_modes_util(
	struct camera_io_master *io_master_info,
	struct i2c_settings_list *i2c_list);

