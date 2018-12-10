#include "cam_temp_core.h"
int32_t cam_temperature_driver_cmd(struct cam_tmp_ctrl *t_ctrl,
	void *arg)
{
	struct cam_control *cmd = (struct cam_control *)arg;
	switch (cmd->op_code) {
	default:
		CAM_ERR(CAM_TEMPERATURE, "Invalid Opcode %d", cmd->op_code);
	}
	return 0;
}


void cam_temperature_shutdown(struct cam_tmp_ctrl *t_ctrl)
{
	return;
}
