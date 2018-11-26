/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
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
#ifndef _CAM_EEPROM_DEV_H_
#define _CAM_EEPROM_DEV_H_

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/cam_sensor.h>
#include <cam_sensor_i2c.h>
#include <cam_sensor_spi.h>
#include <cam_sensor_io.h>
#include <cam_cci_dev.h>
#include <cam_req_mgr_util.h>
#include <cam_req_mgr_interface.h>
#include <cam_mem_mgr.h>
#include <cam_subdev.h>
#include "cam_soc_util.h"
/*for tof camera Begin*/
#include "tl_dev_sensor_config.h"
/*for tof camera End*/

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

#define PROPERTY_MAXSIZE 32

#define MSM_EEPROM_MEMORY_MAP_MAX_SIZE         80
#define MSM_EEPROM_MAX_MEM_MAP_CNT             8
#define MSM_EEPROM_MEM_MAP_PROPERTIES_CNT      8

enum cam_eeprom_state {
	CAM_EEPROM_INIT,
	CAM_EEPROM_ACQUIRE,
	CAM_EEPROM_CONFIG,
};

/**
 * struct cam_eeprom_map_t - eeprom map
 * @data_type       :   Data type
 * @addr_type       :   Address type
 * @addr            :   Address
 * @data            :   data
 * @delay           :   Delay
 *
 */
struct cam_eeprom_map_t {
	uint32_t valid_size;
	uint32_t addr;
	uint32_t addr_type;
	uint32_t data;
	uint32_t data_type;
	uint32_t delay;
};

/**
 * struct cam_eeprom_memory_map_t - eeprom memory map types
 * @page            :   page memory
 * @pageen          :   pageen memory
 * @poll            :   poll memory
 * @mem             :   mem
 * @saddr           :   slave addr
 *
 */
struct cam_eeprom_memory_map_t {
	struct cam_eeprom_map_t page;
	struct cam_eeprom_map_t pageen;
	struct cam_eeprom_map_t poll;
	struct cam_eeprom_map_t mem;
	uint32_t saddr;
};

/**
 * struct cam_eeprom_memory_block_t - eeprom mem block info
 * @map             :   eeprom memory map
 * @num_map         :   number of map blocks
 * @mapdata         :   map data
 * @cmd_type        :   size of total mapdata
 *
 */
struct cam_eeprom_memory_block_t {
	struct cam_eeprom_memory_map_t *map;
	uint32_t num_map;
	uint8_t *mapdata;
	uint32_t num_data;
};

/**
 * struct cam_eeprom_cmm_t - camera multimodule
 * @cmm_support     :   cmm support flag
 * @cmm_compression :   cmm compression flag
 * @cmm_offset      :   cmm data start offset
 * @cmm_size        :   cmm data size
 *
 */
struct cam_eeprom_cmm_t {
	uint32_t cmm_support;
	uint32_t cmm_compression;
	uint32_t cmm_offset;
	uint32_t cmm_size;
};

/**
 * struct cam_eeprom_i2c_info_t - I2C info
 * @slave_addr      :   slave address
 * @i2c_freq_mode   :   i2c frequency mode
 *
 */
struct cam_eeprom_i2c_info_t {
	uint16_t slave_addr;
	uint8_t i2c_freq_mode;
};

/**
 * struct cam_eeprom_soc_private - eeprom soc private data structure
 * @eeprom_name     :   eeprom name
 * @i2c_info        :   i2c info structure
 * @power_info      :   eeprom power info
 * @cmm_data        :   cmm data
 *
 */
struct cam_eeprom_soc_private {
	const char *eeprom_name;
	struct cam_eeprom_i2c_info_t i2c_info;
	struct cam_sensor_power_ctrl_t power_info;
	struct cam_eeprom_cmm_t cmm_data;
};

/**
 * struct cam_eeprom_intf_params - bridge interface params
 * @device_hdl   : Device Handle
 * @session_hdl  : Session Handle
 * @ops          : KMD operations
 * @crm_cb       : Callback API pointers
 */
struct cam_eeprom_intf_params {
	int32_t device_hdl;
	int32_t session_hdl;
	int32_t link_hdl;
	struct cam_req_mgr_kmd_ops ops;
	struct cam_req_mgr_crm_cb *crm_cb;
};

/**
 * struct cam_cmd_conditional_wait - Conditional wait command
 * @pdev            :   platform device
 * @spi             :   spi device
 * @eeprom_mutex    :   eeprom mutex
 * @soc_info        :   eeprom soc related info
 * @io_master_info  :   Information about the communication master
 * @gpio_num_info   :   gpio info
 * @cci_i2c_master  :   I2C structure
 * @v4l2_dev_str    :   V4L2 device structure
 * @bridge_intf     :   bridge interface params
 * @cam_eeprom_state:   eeprom_device_state
 * @userspace_probe :   flag indicates userspace or kernel probe
 * @cal_data        :   Calibration data
 * @device_name     :   Device name
 *
 */
struct cam_eeprom_ctrl_t {
	struct platform_device *pdev;
	struct spi_device *spi;
	struct mutex eeprom_mutex;
	struct cam_hw_soc_info soc_info;
	struct camera_io_master io_master_info;
	struct msm_camera_gpio_num_info *gpio_num_info;
	enum cci_i2c_master_t cci_i2c_master;
	struct cam_subdev v4l2_dev_str;
	struct cam_eeprom_intf_params bridge_intf;
	enum msm_camera_device_type_t eeprom_device_type;
	enum cam_eeprom_state cam_eeprom_state;
	bool userspace_probe;
	struct cam_eeprom_memory_block_t cal_data;
	char device_name[20];
};

/*for tof camera Begin*/
struct cam_eeprom_exposure_addr {
	uint32_t long_addr[15];
	uint32_t long_num;
	uint32_t short_addr[4];
	uint32_t short_num;
	uint32_t lms_addr[4];
	uint32_t lms_num;
};

struct cam_eeprom_ccd_dummy_addr {
	uint32_t addr[4];
	uint32_t addr_num;
};

struct cam_eeprom_vd_ini_ofst_addr {
	uint32_t addr[4];
	uint32_t vd_ini_ofst_num;
};

struct cam_eeprom_exp_reg_addr{
	struct cam_eeprom_exposure_addr exp_addr;
	uint32_t read_size2_addr;
	struct cam_eeprom_ccd_dummy_addr ccd_dummy_addr;
	uint32_t start_v_addr;
	uint32_t vd_length_addr;
	struct cam_eeprom_vd_ini_ofst_addr vd_ini_ofst_addr;
};

struct cam_eeprom_exp_reg_data{
	uint32_t exp_long_data;
	uint32_t exp_short_data;
	uint32_t exp_lms_data;
	uint32_t read_size2_data;
	uint32_t ccd_dummy_data;
	uint32_t start_v_data;
	uint32_t vd_length_data;
	uint32_t vd_ini_ofst_data;
};

struct cam_eeprom_config_common{
	uint32_t reg_addr_from;
	uint32_t data_size;
	uint32_t reg_addr_to;
	uint32_t op_code;
};

enum EEPROM_CONFIG_DATA_LIST{
	EEPROM_CONFIG1_DATA_LIST,
	EEPROM_CONFIG2_DATA_LIST,
};

enum eeprom_config_common_data  {
		TL_AFE_SHD_OFFSET = 0,
		TL_AFE_SHD,
		TL_AFE_SHD_X0,
		TL_AFE_SHD_XPWR,
		TL_AFE_SHD_Y0,
		TL_AFE_SHD_YPWR,
		TL_AFE_DFCT_PIX_TH_TBL,
		TL_AFE_DFCT,
		TL_AFE_SHP_LOC,
		TL_AFE_SHD_LOC,
		TL_AFE_OUTPUT,
		TL_AFE_OUTPUTSELR,
		TL_AFE_VC,
		TL_AFE_GRID3,
		TL_AFE_IR_GAIN_GMM,
		TL_AFE_IR_GMM,
		TL_AFE_IR_GMM_Y,
		TL_AFE_CHKR_UPPRTH,
		TL_AFE_CHKR_LWRTH,
		TL_AFE_CHKR_START_V,
		TL_AFE_CHKR_START_H,
		TL_AFE_CHKR_SIZE_H,
		TL_AFE_CHKR_UPRERR_H,
		TL_AFE_CHKR_UPRERR_V,
		TL_AFE_CHKR_LWRERR_H,
		TL_AFE_CHKR_LWRERR_V,
		TL_AFE_CHKR_DET_ENA,
};
enum eeprom_config_mode_data  {
		TL_AFE_NLR_OFFSET = 0,
		TL_AFE_NLR_X0,
		TL_AFE_NLR_XPWR,
		TL_AFE_ZERO_OFFSET,
		TL_AFE_DEPTH_SLOPE,
		TL_AFE_DEPTH3_SLOPE,
		TL_AFE_RATE_ADJUST,
		TL_AFE_ALIGN,
		TL_AFE_READ_SIZE0,
		TL_AFE_READ_SIZE3,
		TL_AFE_ROI,
		TL_AFE_GRID,
		TL_AFE_RAWNR_XPWR,
		TL_AFE_RAWNR_BLTBL,
		TL_AFE_RAWNR_MED,
		TL_AFE_SAT_TH,
		TL_AFE_RAWNR_BKTBL,
		TL_AFE_CORING,
		TL_AFE_CORB,
		TL_AFE_CORF,
		TL_AFE_DEPTH1,
		TL_AFE_CONTROL,
		TL_AFE_PLS_MOD_CTRL,
		TL_AFE_PLS_MOD_VAL,
};

struct cam_eeprom_config_exp_data {
	uint16_t exp_max;
	uint16_t exp_val;
	uint16_t tof_seq_ini_ofst;
	uint16_t ld_pls_duty;
	uint16_t num_clk_in_hd;
	uint16_t beta_num;
	uint16_t tof_emt_period_ofst;
	uint16_t vd_duration;
	uint16_t vd_ini_ofst;
	uint16_t num_hd_in_readout;
	uint16_t vd_ini_ofst_adr_num;
	uint16_t idle;
	uint16_t Idle[4];
};

struct cam_eeprom_list_head {
	struct list_head list_head_init;
	int    initial;
	struct list_head list_head_config[EEPROM_MODE_DATA_NUM];
	int    resolution[EEPROM_MODE_DATA_NUM];
	struct list_head list_head_config_other;
	int    other;
	struct list_head list_head_streamon;
	int streamon;
	struct list_head list_head_streamoff;
	int streamoff;
	enum EEPROM_CONFIG_DATA_LIST list_chioce;
};

enum cam_eeprom_free {
	LIST_HEAD_ALL,
	LIST_HEAD_INITIAL,
	LIST_HEAD_RESOLUTION,
	LIST_HEAD_OTHER,
	LIST_HEAD_STREAMON,
	LIST_HEAD_STREAMOFF,
};
/*for tof camera End*/

int32_t cam_eeprom_update_i2c_info(struct cam_eeprom_ctrl_t *e_ctrl,
	struct cam_eeprom_i2c_info_t *i2c_info);

#endif /*_CAM_EEPROM_DEV_H_ */
