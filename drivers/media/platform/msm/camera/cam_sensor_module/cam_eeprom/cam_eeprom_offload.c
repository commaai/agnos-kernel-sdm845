#include "tl_dev_eeprom.h"

struct kobject *eeprom_kobj;
tl_dev_eeprom_pup *tof_eeprom;

ssize_t eeprom_show(struct kobject *kobj,
		struct kobj_attribute *attr,char *ubuf)
{
	memcpy(ubuf,&(tof_eeprom->eeprom),sizeof(tl_dev_eeprom));
	return sizeof(tl_dev_eeprom);
}

ssize_t mode_show(struct kobject *kobj,
		struct kobj_attribute *attr,char *ubuf)
{
	uint32_t mode,count;
	mode = get_op_mode();
	if(mode < TL_E_MODE_MAX)
	{
		switch(mode){
		case 0:
			count = sizeof("mode near");
			snprintf(ubuf,count,"%s","mode near");
			break;
		case 1:
			count = sizeof("mode far");
			snprintf(ubuf,count,"%s","mode far");
			break;
		default:
			count = sizeof("mode error");
			CAM_ERR(CAM_EEPROM,"error mode");
			snprintf(ubuf,count,"%s","mode error");
			break;
		}
	} else {
		count = sizeof("mode error");
		CAM_ERR(CAM_EEPROM,"mode > mode MAX");
		snprintf(ubuf,count,"%s","mode error");
	}
	return count;
}

ssize_t multi_show(struct kobject *kobj,
		struct kobj_attribute *attr,char *ubuf)
{
	uint32_t count;
	bool multi_camera;
	multi_camera = get_whether_support_multi_camera();
	if(multi_camera == false)
	{
		count = sizeof("not support multi camera ");
		snprintf(ubuf,count,"%s","not support multi camera");
	} else {
		count = sizeof("support multi camera ");
		snprintf(ubuf,count,"%s","support multi camera");
	}
	return count;
}


ssize_t temperature_show(struct kobject *kobj,
		struct kobj_attribute *attr,char *ubuf)
{
	CAM_ERR(CAM_EEPROM,"temperature show");
	return sizeof(tl_dev_eeprom);
}

ssize_t mode_store(struct kobject *kobj,
		struct kobj_attribute *attr,const char *ubuf,size_t count)
{
	switch(*ubuf){
	case 'n':
		set_op_mode(0);
		break;
	case 'N':
		set_op_mode(0);
		break;
	case 'f':
		set_op_mode(1);
		break;
	case 'F':
		set_op_mode(1);
		break;
	default:
		CAM_ERR(CAM_EEPROM,"nodeset mode error(correct value n/N f/F)");
		break;
	}
	return 1;
}

ssize_t multi_store(struct kobject *kobj,
		struct kobj_attribute *attr,const char *ubuf,size_t count)
{
	switch(*ubuf){
	case '0':
		set_whether_support_multi_camera(false);
		CAM_ERR(CAM_EEPROM," set multi camera false");
		break;
	case '1':
		set_whether_support_multi_camera(true);
		CAM_ERR(CAM_EEPROM," set multi camera true");
		break;
	default:
		CAM_ERR(CAM_EEPROM,"multi camera mode error(correct value 0/1)");
		break;
	}
	return 1;
}

struct kobj_attribute eeprom_kobj_attr_eeprom_bin
	=__ATTR(eeprom_bin,0664,eeprom_show,NULL);
struct kobj_attribute eeprom_kobj_attr_mode
	= __ATTR(mode,0664,mode_show,mode_store);
struct kobj_attribute eeprom_kobj_attr_temperature
	= __ATTR(temperature,0664,temperature_show,NULL);
struct kobj_attribute eeprom_kobj_attr_multi_state
	= __ATTR(multi_state,0664,multi_show,multi_store);

struct attribute *eeprom_attr[] = {
	&eeprom_kobj_attr_eeprom_bin.attr,
	NULL,
};

struct attribute *mode_attr[] = {
	&eeprom_kobj_attr_mode.attr,
	NULL,
};

struct attribute *temperature_attr[] = {
	&eeprom_kobj_attr_temperature.attr,
	NULL,
};

struct attribute *multi_attr[] = {
	&eeprom_kobj_attr_multi_state.attr,
	NULL,
};
struct attribute_group eeprom_attr_grp[] = {
	{
		.name = "eepromdata",
		.attrs = eeprom_attr,
	},
	{
		.name = "mode",
		.attrs = mode_attr,
	},
	{
		.name = "temperature",
		.attrs = temperature_attr,
	},
	{
		.name = "multicamera",
		.attrs = multi_attr,
	},
	{},
};

void tl_eeprom_create_node(void) {
	int ret,i;
	eeprom_kobj = kobject_create_and_add("TOFeeprom",NULL);
	if(eeprom_kobj == NULL){
		return;
	}
	for(i = 0; i < 4;i++){
	ret = sysfs_create_group(eeprom_kobj,&eeprom_attr_grp[i]);
	if(ret)
		kobject_put(eeprom_kobj);
	}
}

static void cam_eeprom_format_data_cnv_u16_c(uint16_t *p_in16,
		char *p_out8, uint16_t u16_num)
{
	uint16_t i, tmp;
	for(i=0; i<u16_num; i++){
		tmp = *(p_in16 + i);
		*(p_out8 + (i * 2U)     ) = (char)((tmp & 0xFF00U) >> 8U);
		*(p_out8 + (i * 2U) + 1U) = (char)((tmp & 0x00FFU));
   }

}

static void cam_eeprom_format_data_cnv_u16_i64(uint16_t *p_in16,
		int64_t *p_out64, uint16_t i64_num)
{
    uint16_t i, *p_tmp, num = (sizeof(int64_t) / sizeof(uint16_t));
    for(i=0; i<i64_num; i++){
        p_tmp = p_in16 + (num * i);
        /* convert int64 data */
        *(p_out64 + i)  = (int64_t)((((uint64_t)*p_tmp) << 48)
				| (((uint64_t)*(p_tmp + 1U)) << 32)
				| (((uint64_t)*(p_tmp + 2U)) << 16)
				| ((uint64_t)*(p_tmp + 3U)));
    }
}

static uint16_t tl_dev_eeprom_calc_xor(uint16_t *p_top,
		uint16_t num, uint16_t org_xor)
{
    uint16_t i;
    for(i=0; i<num; i++){
        org_xor ^= *((uint16_t *)p_top + i);
    }
    return org_xor;
}

static void tl_dev_eeprom_check_exp(tl_dev_rom_mode *p_mode)
{
    /* if not set maximum, set from ae parameter[far] */
    if(p_mode->exp.exp_max == 0U) {
        p_mode->exp.exp_max = p_mode->ae[2].lumine_num;
    }
    /* if not set default, set from ae parameter[near] */
    if(p_mode->exp.exp_default == 0U) {
        p_mode->exp.exp_default = p_mode->ae[0].lumine_num;
    }
}

static TL_E_RESULT tl_dev_eeprom_calc_chksum(
    uint16_t *p_cmn_mode, uint16_t *p_pup, uint16_t pup_size)
{
    uint16_t i, num, r_xor=0, *p_tmp;
	uint16_t align = (uint16_t)sizeof(uint16_t);

    if((p_cmn_mode == NULL) || (p_pup == NULL) || (pup_size == 0U)){
        return TL_E_ERR_PARAM;
    }

    /* calc camera type & info(without CHECK_SUM) */
    p_tmp = p_cmn_mode;
    num   = (TL_EEPROM_CHECK_SUM - TL_EEPROM_CMN_TOP) / align;
    r_xor = tl_dev_eeprom_calc_xor(p_tmp, num, r_xor);
    /* after CHECK_SUM data */
    p_tmp = p_cmn_mode + TL_EEPROM_OFFSET(TL_EEPROM_CHECK_SUM) + 1U;
    r_xor = tl_dev_eeprom_calc_xor(p_tmp, 1U, r_xor);
    /* calc common area remain */
    p_tmp = p_cmn_mode + TL_EEPROM_OFFSET(TL_EEPROM_PLANER_PRM);
    num   = (TL_EEPROM_CMN_AREA_SIZE - TL_EEPROM_PLANER_PRM) / align;
    r_xor = tl_dev_eeprom_calc_xor(p_tmp, num, r_xor);
    /* calc mode area */
    for(i=0; i<(uint16_t)TL_E_MODE_MAX; i++){
        p_tmp = p_cmn_mode + TL_EEPROM_OFFSET(TL_EEPROM_MODE_TOP(i));
        num   = TL_EEPROM_MODE_AREA_SIZE / align;
        r_xor = tl_dev_eeprom_calc_xor(p_tmp, num, r_xor);
    }
    /* calc Power-Up Sequence area */
    num   = pup_size / align;
    r_xor = tl_dev_eeprom_calc_xor(p_pup, num, r_xor);

    /* check CHECK_SUM */
    p_tmp = p_cmn_mode + TL_EEPROM_OFFSET(TL_EEPROM_CHECK_SUM);
    if(r_xor != *p_tmp){
        return TL_E_ERR_SYSTEM;
    }

    return TL_E_SUCCESS;
}
#if 1
static uint16_t tl_dev_eeprom_calc_fps(tl_dev_rom_exp_prm *p_exp_prm)
{
    uint32_t vd = 0;
	uint16_t fps = 0;
    uint16_t ifps = 0xFFFFU;
    uint32_t clk_width = ((uint32_t)p_exp_prm->clk_width_u << 16U)
		| ((uint32_t)p_exp_prm->clk_width_l);

    if(p_exp_prm->vd_duration <= 2U){
        return 0xFFFFU;
    }
    if((p_exp_prm->vd_duration % 3U) != 0U){
        return 0xFFFFU;
    }
    if(p_exp_prm->num_clk_in_hd == 0U){
        return 0xFFFFU;
    }
    if(clk_width == 0U){
        return 0xFFFFU;
    }

    vd = p_exp_prm->vd_duration * p_exp_prm->num_clk_in_hd *
		(clk_width / 1000000);
	if(clk_width % 1000000 > 500000)
		vd += p_exp_prm->vd_duration * p_exp_prm->num_clk_in_hd;
    if(vd <= 0){
        return 0xFFFFU;
    } else {
        fps = (1000000000 / vd) ;  /* convert nano sec to sec */
		if( (1000000000 % vd) >  vd / 2)
			ifps = ++fps;
        ifps = fps;
        if(ifps == 0U){
            return 0xFFFFU;
        }
    }

    return ifps;
}
#endif

void cam_eeprom_format_calibration_common_data_read(
		tl_dev_rom_common *p_eep, uint16_t *p_cmn)
{
    uint16_t i;
    /* cam_type */
    cam_eeprom_format_data_cnv_u16_c(p_cmn +
			TL_EEPROM_OFFSET(TL_EEPROM_MOD_NAME),
			p_eep->cam_type.mod_name,
			TL_EEPROM_ARY_SIZE(p_eep->cam_type.mod_name));
    cam_eeprom_format_data_cnv_u16_c(p_cmn +
			TL_EEPROM_OFFSET(TL_EEPROM_AFE_NAME),
			p_eep->cam_type.afe_name,
			TL_EEPROM_ARY_SIZE(p_eep->cam_type.afe_name));
    cam_eeprom_format_data_cnv_u16_c(p_cmn +
			TL_EEPROM_OFFSET(TL_EEPROM_SNS_NAME),
			p_eep->cam_type.sns_name,
			TL_EEPROM_ARY_SIZE(p_eep->cam_type.sns_name));
    cam_eeprom_format_data_cnv_u16_c(p_cmn +
			TL_EEPROM_OFFSET(TL_EEPROM_LNS_NAME),
			p_eep->cam_type.lns_name,
			TL_EEPROM_ARY_SIZE(p_eep->cam_type.lns_name));

    /* cam_info */
    p_eep->cam_info.sno_l
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_SNO_L);
    p_eep->cam_info.map_ver
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_MAP_VER);
    p_eep->cam_info.sno_u
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_SNO_U);
    p_eep->cam_info.ajust_date
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_AJUST_DATE);
    p_eep->cam_info.ajust_no
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_AJUST_NO);
    p_eep->cam_info.mod_type1
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_MOD_TYPE1);
    p_eep->cam_info.mod_type2
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_MOD_TYPE2);
    p_eep->cam_info.afe_ptn_id
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_AFE_PTN_ID);
    p_eep->cam_info.opt_axis_center_h
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_OPT_AXIS_CENTER_H);
    p_eep->cam_info.opt_axis_center_v
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_OPT_AXIS_CENTER_V);
    p_eep->cam_info.fov_h
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_FOV_H);
    p_eep->cam_info.fov_v
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_FOV_V);
    p_eep->cam_info.pixel_pitch
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_PIXEL_PITCH);
    p_eep->cam_info.focal_len
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_FOCAL_LEN);
    p_eep->cam_info.enable_mode
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_ENABLE_MODE);
    p_eep->cam_info.tal_mode
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_TAL_MODE);
    p_eep->cam_info.pup_size
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_PUP_SIZE);
    p_eep->cam_info.check_sum
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_CHECK_SUM);

    /* lens */
    cam_eeprom_format_data_cnv_u16_i64(p_cmn +
			TL_EEPROM_OFFSET(TL_EEPROM_PLANER_PRM),
			p_eep->lens.planer_prm,
			TL_EEPROM_ARY_SIZE(p_eep->lens.planer_prm));
    cam_eeprom_format_data_cnv_u16_i64(p_cmn +
			TL_EEPROM_OFFSET(TL_EEPROM_DISTORTION_PRM),
			p_eep->lens.distortion_prm,
			TL_EEPROM_ARY_SIZE(p_eep->lens.distortion_prm));
    /* shading */
//    memcpy((void*)p_eep->shading.offset,
//    (void*)(p_cmn + TL_EEPROM_OFFSET(TL_EEPROM_SHD_OFFSET)),
//    TL_EEPROM_ARY_SIZE(p_eep->shading.offset) * sizeof(uint16_t));
    p_eep->shading.shd
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_SHD);
    p_eep->shading.x0
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_SHD_X0);
//    memcpy((void*)p_eep->shading.xpwr,
//    (void*)(p_cmn + TL_EEPROM_OFFSET(TL_EEPROM_SHD_XPWR)),
//    TL_EEPROM_ARY_SIZE(p_eep->shading.xpwr) * sizeof(uint16_t));
    p_eep->shading.y0
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_SHD_Y0);
//    memcpy((void*)p_eep->shading.ypwr, (void*)(p_cmn +
//    TL_EEPROM_OFFSET(TL_EEPROM_SHD_YPWR)),
//    TL_EEPROM_ARY_SIZE(p_eep->shading.ypwr) * sizeof(uint16_t));
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->shading.offset);i++){
		p_eep->shading.offset[i] =
			TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_SHD_OFFSET + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->shading.xpwr);i++){
		p_eep->shading.xpwr[i] =
			TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_SHD_XPWR + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->shading.ypwr);i++){
		p_eep->shading.ypwr[i] =
			TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_SHD_YPWR + i * 2);
    }

    /* dfct */
//    memcpy((void*)p_eep->dfct.dfct_pix_th_tbl
//    , (void*)(p_cmn + TL_EEPROM_OFFSET(TL_EEPROM_DFCT_PIX_TH_TBL)),
//    TL_EEPROM_ARY_SIZE(p_eep->dfct.dfct_pix_th_tbl) * sizeof(uint16_t));
//    memcpy((void*)p_eep->dfct.dfct, (void*)(p_cmn +
//    TL_EEPROM_OFFSET(TL_EEPROM_DFCT)),
//    TL_EEPROM_ARY_SIZE(p_eep->dfct.dfct) * sizeof(uint16_t));
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->dfct.dfct_pix_th_tbl);i++){
		p_eep->dfct.dfct_pix_th_tbl[i]
			= TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_DFCT_PIX_TH_TBL + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->dfct.dfct);i++){
		p_eep->dfct.dfct[i]
			= TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_DFCT + i * 2);
    }

    /* timing_mipi */
    p_eep->timing_mipi.shp_loc
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_SHP_LOC);
    p_eep->timing_mipi.shd_loc
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_SHD_LOC);
    p_eep->timing_mipi.output
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_OUTPUT);
    p_eep->timing_mipi.output_sel
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_OUTPUT_SEL);
    p_eep->timing_mipi.vc
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_VC);

    /* cutout */
    p_eep->cutout.grid3
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_GRID3);

    /* ir */
    p_eep->ir.ir1
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_IR1);
//    memcpy((void*)p_eep->ir.ir_gmm,
//    (void*)(p_cmn + TL_EEPROM_OFFSET(TL_EEPROM_IR_GMM)),
//    TL_EEPROM_ARY_SIZE(p_eep->ir.ir_gmm)   * sizeof(uint16_t));
//    memcpy((void*)p_eep->ir.ir_gmm_y,
//    (void*)(p_cmn + TL_EEPROM_OFFSET(TL_EEPROM_IR_GMM_Y)),
//    TL_EEPROM_ARY_SIZE(p_eep->ir.ir_gmm_y) * sizeof(uint16_t));
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->ir.ir_gmm);i++){
		p_eep->ir.ir_gmm[i]
			= TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_IR_GMM + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->ir.ir_gmm_y);i++){
		p_eep->ir.ir_gmm_y[i]
			= TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_IR_GMM_Y + i * 2);
    }

    /* chkr */
    p_eep->chkr.upprth
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_UPPRTH);
    p_eep->chkr.lwrth
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_LWRTH);
    p_eep->chkr.start_v
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_START_V);
    p_eep->chkr.start_h
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_START_H);
    p_eep->chkr.size_h
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_SIZE_H);
    p_eep->chkr.upprerr_h
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_UPPRERR_H);
    p_eep->chkr.upprerr_v
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_UPPRERR_V);
    p_eep->chkr.lwrerr_h
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_LWRERR_H);
    p_eep->chkr.lwrerr_v
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_LWRERR_V);
    p_eep->chkr.det_ena
		= TL_EEPROM_OFST_VAL(p_cmn, TL_EEPROM_DET_ENA);

}

static TL_E_RESULT cam_eeprom_format_calibration_mode_data_read(
		tl_dev_eeprom_pup *tof_eeprom,
		tl_dev_rom_mode *p_eep,
		uint16_t *p_mode,uint16_t mode)
{
    uint16_t i, tmp;

    /* mode info */
    p_eep->info.tof_mode_flag
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_TOF_MODE_FLAG);
    p_eep->info.ld_flag
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_LD_FLAG);
    p_eep->info.range_near_limit
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_RANGE_NEAR_LIMIT);
    p_eep->info.range_far_limit
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_RANGE_FAR_LIMIT);
    p_eep->info.depth_unit
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_DEPTH_UNIT);
    /* exposure */
    p_eep->exp.exp_max
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_EXP_MAX);
    p_eep->exp.exp_default
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_EXP_DEFAULT);

    /* AE parameters */
    for(i=0; i<(uint16_t)TL_EEPROM_ARY_SIZE(p_eep->ae); i++){
        p_eep->ae[i].distance
			= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_AE_OFST(i, TL_EEPROM_AE_DISTANCE));
        p_eep->ae[i].lumine_num
			= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_AE_OFST(i, TL_EEPROM_AE_LUMINE_NUM));
        p_eep->ae[i].slope
			= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_AE_OFST(i, TL_EEPROM_AE_SLOPE));
        p_eep->ae[i].zofst
			= (int16_t)TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_AE_OFST(i, TL_EEPROM_AE_ZOFST));
    }

    /* AFE address of exposure */
    tmp = TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_EXP_ADDR_NUM);
    p_eep->exp_addr.long_num
		= (uint8_t)((tmp & 0xFF00U) >> 8U);
    p_eep->exp_addr.short_num
		= (uint8_t)((tmp & 0x00F0U) >> 4U);
    p_eep->exp_addr.lms_num
		= (uint8_t) (tmp & 0x000FU);
//    memcpy((void*)p_eep->exp_addr.long_addr,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_EXP_ADDR_LONG)),
//    TL_EEPROM_ARY_SIZE(p_eep->exp_addr.long_addr)  * sizeof(uint16_t));
//    memcpy((void*)p_eep->exp_addr.short_addr,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_EXP_ADDR_SHORT)),
//    TL_EEPROM_ARY_SIZE(p_eep->exp_addr.short_addr) * sizeof(uint16_t));
//    memcpy((void*)p_eep->exp_addr.lms_addr,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_EXP_ADDR_LMS)),
//    TL_EEPROM_ARY_SIZE(p_eep->exp_addr.lms_addr)   * sizeof(uint16_t));
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->exp_addr.long_addr);i++){
		p_eep->exp_addr.long_addr[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_EXP_ADDR_LONG + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->exp_addr.short_addr);i++){
		p_eep->exp_addr.short_addr[i] =
			TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_EXP_ADDR_SHORT + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->exp_addr.lms_addr);i++){
		p_eep->exp_addr.lms_addr[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_EXP_ADDR_LMS + i * 2);
    }

    if(p_eep->exp_addr.long_num >
			(uint8_t)TL_EEPROM_ARY_SIZE(p_eep->exp_addr.long_addr)){
        return TL_E_ERR_SYSTEM;
    }
    if(p_eep->exp_addr.short_num >
			(uint8_t)TL_EEPROM_ARY_SIZE(p_eep->exp_addr.short_addr)){
        return TL_E_ERR_SYSTEM;
    }
    if(p_eep->exp_addr.lms_num >
			(uint8_t)TL_EEPROM_ARY_SIZE(p_eep->exp_addr.lms_addr)){
        return TL_E_ERR_SYSTEM;
    }

    /* AFE address of dummy transfer */
    p_eep->ccd_addr.addr_num
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_DMMY_TRNS_NUM);
//    memcpy((void*)p_eep->ccd_addr.addr,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_DMMY_TRNS_ADR)),
//    TL_EEPROM_ARY_SIZE(p_eep->ccd_addr.addr) * sizeof(uint16_t));
    if(p_eep->ccd_addr.addr_num >
			(uint16_t)TL_EEPROM_ARY_SIZE(p_eep->ccd_addr.addr)){
        return TL_E_ERR_SYSTEM;
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->ccd_addr.addr);i++){
		p_eep->ccd_addr.addr[i] =
			TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_DMMY_TRNS_ADR + i * 2);
    }
    /* Parameters of exposure setting */
    p_eep->exp_prm.vd_ini_ofst
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_VD_INI_OFST);
    p_eep->exp_prm.vd_ini_ofst_adr_num
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_VD_INI_OFST_ADR_NUM);
//    memcpy((void*)p_eep->exp_prm.vd_init_ofst_adr,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_VD_INIT_OFST_ADR)),
//    TL_EEPROM_ARY_SIZE(p_eep->exp_prm.vd_init_ofst_adr) * sizeof(uint16_t));
    p_eep->exp_prm.ld_pls_duty
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_LD_PLS_DUTY);
    p_eep->exp_prm.vd_duration
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_VD_DURATION);
    p_eep->exp_prm.vd_reg_adr
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_VD_REG_ADR);
    p_eep->exp_prm.num_clk_in_hd
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_NUM_CLK_IN_HD);
    p_eep->exp_prm.beta_num
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_BETA_NUM);
    p_eep->exp_prm.num_hd_in_readout
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_NUM_HD_IN_READOUT);
    p_eep->exp_prm.clk_width_u
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_CLK_WIDTH_U);
    p_eep->exp_prm.clk_width_l
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_CLK_WIDTH_L);
    p_eep->exp_prm.tof_emt_period_ofst
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_TOF_EMT_PERIOD_OFST);
    p_eep->exp_prm.tof_seq_ini_ofst
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_TOF_SEQ_INI_OFST);
	p_eep->exp_prm.idle_peri_num
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_IDLE_PERI_NUM);
    //memcpy((void*)p_eep->exp_prm.idle_peri_adr, (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_IDLE_PERI_ADR1)), TL_EEPROM_ARY_SIZE(p_eep->exp_prm.idle_peri_adr) * sizeof(uint16_t));
    if(p_eep->exp_prm.vd_ini_ofst_adr_num >
			(uint16_t)TL_EEPROM_ARY_SIZE(p_eep->exp_prm.vd_init_ofst_adr)){
        return TL_E_ERR_SYSTEM;
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->exp_prm.vd_init_ofst_adr);i++){
		p_eep->exp_prm.vd_init_ofst_adr[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_VD_INIT_OFST_ADR + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->exp_prm.idle_peri_adr);i++){
		p_eep->exp_prm.idle_peri_adr[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_IDLE_PERI_ADR1 + i * 2);
    }

    /* nonlinear correction */
//    memcpy((void*)p_eep->nlr.offset,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_NLR_OFFSET)),
//    TL_EEPROM_ARY_SIZE(p_eep->nlr.offset) * sizeof(uint16_t));
    p_eep->nlr.x0
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_NLR_X0);
//    memcpy((void*)p_eep->nlr.xpwr,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_NLR_XPWR)),
//    TL_EEPROM_ARY_SIZE(p_eep->nlr.xpwr)   * sizeof(uint16_t));
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->nlr.offset);i++){
		p_eep->nlr.offset[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_NLR_OFFSET + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->nlr.xpwr);i++){
		p_eep->nlr.xpwr[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_NLR_XPWR + i * 2);
    }

    /* depth correction */
    p_eep->depth.depth0
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_DEPTH0);
    p_eep->depth.depth2
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_DEPTH2);
    p_eep->depth.depth3
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_DEPTH3);
    /* AFE timing control */
//    memcpy((void*)p_eep->timing.rate_adjust,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_RATE_ADJUST)),
//    TL_EEPROM_ARY_SIZE(p_eep->timing.rate_adjust) * sizeof(uint16_t));
//    memcpy((void*)p_eep->timing.align,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_ALIGN)),
//    TL_EEPROM_ARY_SIZE(p_eep->timing.align)       * sizeof(uint16_t));
//    memcpy((void*)p_eep->timing.read_size,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_READ_SIZE)),
//    TL_EEPROM_ARY_SIZE(p_eep->timing.read_size)   * sizeof(uint16_t));
//    memcpy((void*)p_eep->timing.roi,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_ROI)),
//    TL_EEPROM_ARY_SIZE(p_eep->timing.roi)         * sizeof(uint16_t));
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->timing.rate_adjust);i++){
		p_eep->timing.rate_adjust[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_RATE_ADJUST + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->timing.align);i++){
		p_eep->timing.align[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_ALIGN + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->timing.read_size);i++){
		p_eep->timing.read_size[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_READ_SIZE0 + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->timing.roi);i++){
		p_eep->timing.roi[i] =
			TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_ROI + i * 2);
    }

    /* grid conversion */
//    memcpy((void*)p_eep->grid.grid,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_GRID)),
//    TL_EEPROM_ARY_SIZE(p_eep->grid.grid) * sizeof(uint16_t));
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->grid.grid);i++){
		p_eep->grid.grid[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_GRID + i * 2);
    }

    /* RAW noise reduction */
//    memcpy((void*)p_eep->raw_nr.xpwr,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_RAWNR_XPWR)),
//    TL_EEPROM_ARY_SIZE(p_eep->raw_nr.xpwr)   * sizeof(uint16_t));
//    memcpy((void*)p_eep->raw_nr.bl_tbl,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_RAWNR_BL_TBL)),
//    TL_EEPROM_ARY_SIZE(p_eep->raw_nr.bl_tbl) * sizeof(uint16_t));
    p_eep->raw_nr.med
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_RAWNR_MED);
    p_eep->raw_nr.sat_th
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_RAWNR_SAT_TH);
//    memcpy((void*)p_eep->raw_nr.bk_tbl,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_RAWNR_BK_TBL)),
//    TL_EEPROM_ARY_SIZE(p_eep->raw_nr.bk_tbl) * sizeof(uint16_t));
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->raw_nr.xpwr);i++){
		p_eep->raw_nr.xpwr[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_RAWNR_XPWR + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->grid.grid);i++){
		p_eep->raw_nr.bl_tbl[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_RAWNR_BL_TBL + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->raw_nr.bk_tbl);i++){
		p_eep->raw_nr.bk_tbl[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_RAWNR_BK_TBL + i * 2);
    }

    /* coring */
//    memcpy((void*)p_eep->coring.cor,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_COR)),
//    TL_EEPROM_ARY_SIZE(p_eep->coring.cor)  * sizeof(uint16_t));
//    memcpy((void*)p_eep->coring.corb,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_CORB)),
//    TL_EEPROM_ARY_SIZE(p_eep->coring.corb) * sizeof(uint16_t));
//    memcpy((void*)p_eep->coring.corf,
//    (void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_CORF)),
//    TL_EEPROM_ARY_SIZE(p_eep->coring.corf) * sizeof(uint16_t));
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->coring.cor);i++){
		p_eep->coring.cor[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_COR + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->coring.corb);i++){
		p_eep->coring.corb[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_CORB + i * 2);
    }
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->coring.corf);i++){
		p_eep->coring.corf[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_CORF + i * 2);
    }

    /* depth control */
    p_eep->depth_ctrl.depth1
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_DEPTH1);
    p_eep->depth_ctrl.control
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_CONTROL);

    /* Duty modulation of TOF pulse */
    p_eep->pls_mod.control
		= TL_EEPROM_OFST_VAL(p_mode, TL_EEPROM_PLS_MOD_CTRL);

	tof_eeprom->control_value = p_eep->pls_mod.control;

    memcpy((void*)p_eep->pls_mod.val,
			(void*)(p_mode + TL_EEPROM_OFFSET(TL_EEPROM_PLS_MOD_VAL)),
			TL_EEPROM_ARY_SIZE(p_eep->pls_mod.val) * sizeof(uint16_t));
    for(i=0;i<TL_EEPROM_ARY_SIZE(p_eep->pls_mod.val);i++){
		p_eep->pls_mod.val[i]
			= TL_EEPROM_OFST_VAL(p_mode,TL_EEPROM_PLS_MOD_VAL + i * 2);
    }

    /* calc frame rate */
    p_eep->info.fps = tl_dev_eeprom_calc_fps(&p_eep->exp_prm);
    if(p_eep->info.fps == 0xFFFFU){
        return TL_E_ERR_SYSTEM;
    }

    /* check exposure(max, default) */
    tl_dev_eeprom_check_exp(p_eep);

    return TL_E_SUCCESS;
}


static TL_E_RESULT tl_dev_eeprom_check_map_version(uint16_t map_ver)
{
	uint8_t ver = (uint8_t)(map_ver >> 8U);
	if(ver != TL_EEPROM_SUPPORT_MAP_VER)
		return TL_E_ERR_NOT_SUPPORT;
	return TL_E_SUCCESS;
}

static TL_E_RESULT tl_dev_eeprom_check_module_type(uint16_t mod_type)
{
    uint8_t type = (uint8_t)(mod_type >> 12U);
    switch(type) {
	    case (uint8_t)TL_E_EEPROM_MOD_TYPE_V4T: /* this software supports only V4T */
			break;
	    case (uint8_t)TL_E_EEPROM_MOD_TYPE_V4:
	    default:
		    return TL_E_ERR_NOT_SUPPORT;
	  }
    return TL_E_SUCCESS;
}
#if 0
void show(uint16_t *p_cmn,uint32_t pup_size)
{
	int i;
	int size = (912 + 624 + 624 + pup_size)/2;
	for(i = 0; i< size;i++){
		CAM_ERR(CAM_EEPROM,
		"LOEE : reg_addr = %#x,reg_data = %#x",i * 2
		,TL_EEPROM_OFST_VAL(p_cmn,i * 2));
	}
}
#endif
tl_dev_eeprom_pup* cam_eeprom_module_offload(
		struct cam_eeprom_ctrl_t *e_ctrl,
		uint8_t *mapdata)
{
	uint32_t e_size,tmp_vc,pup_size;//,reg_val;
	int      i;
	uint16_t *p_cmn = (uint16_t *)mapdata;
	tl_dev_rom_common *p_eep = NULL;
	TL_E_RESULT tl_ret = TL_E_SUCCESS;

	if(e_ctrl->cal_data.num_data == 0){
		CAM_ERR(CAM_EEPROM,"failed,OTP/EEPROM empty");
		return NULL;
	}
	tof_eeprom = kzalloc(sizeof(tl_dev_eeprom_pup),GFP_KERNEL);
	if(tof_eeprom == NULL)
		return NULL;
	p_eep = &(tof_eeprom->eeprom.cmn);
	e_size = e_ctrl->cal_data.num_data/2;

	for(i = 0; i <(int)e_size; i++){
		*(p_cmn + i)
			= (uint16_t)((uint16_t)(*(p_cmn + i) & 0x00FFU) << 8U)
			| ((uint16_t)(*(p_cmn + i) & 0xFF00U) >> 8U);
	}
	//check version
	tl_ret = tl_dev_eeprom_check_map_version(
			TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_MAP_VER));
	if(tl_ret != TL_E_SUCCESS){
		CAM_ERR(CAM_EEPROM,"failed check map version = %#x",
				(TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_MAP_VER)));
//		goto free_kz;
	}
	//check module type
	tl_ret = tl_dev_eeprom_check_module_type(
			TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_MOD_TYPE1));
	if(tl_ret != TL_E_SUCCESS){
		CAM_ERR(CAM_EEPROM,
				"failed. check module type,tl_ret=%d",tl_ret);
		goto free_kz;
	}
	//check virtual
	tmp_vc = TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_VC);
	if(tmp_vc != TL_EEPROM_SUPPORT_VC){
		CAM_ERR(CAM_EEPROM,"failed. check virtual Channel");
		goto free_kz;
	}
	//check pup_size
	pup_size =TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_PUP_SIZE);
	if(pup_size == 0U){
		CAM_ERR(CAM_EEPROM,"failed. pup_size == 0U");
		goto free_kz;
	}
	tof_eeprom->pup_size = pup_size/2;
	//pup
/*
	memcpy(tof_eeprom->pup_data,p_cmn
	+ TL_EEPROM_OFFSET(TL_EEPROM_PUP_TOP),pup_size);
*/
/*
	for(i = 0; i < pup_size/2;i++){
	camera_io_dev_read(&(e_ctrl->io_master_info)
			,TL_EEPROM_PUP_TOP + (i*2),&reg_val
			,CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_WORD);
		tof_eeprom->pup_data[i] = reg_val;
	}
*/
#if 1
	for(i = 0; i < pup_size/2;i++){
		tof_eeprom->pup_data[i]
			= TL_EEPROM_OFST_VAL(p_cmn,TL_EEPROM_PUP_TOP + i * 2);
	}
#endif
	// show(p_cmn,pup_size);
 	 /* check CHECK_SUM value */
	tl_ret = tl_dev_eeprom_calc_chksum(
			p_cmn, tof_eeprom->pup_data, pup_size);
	if(tl_ret != TL_E_SUCCESS){
    	CAM_ERR(CAM_EEPROM,"failed. check CHECK_SUM value,tl_ret=%d",tl_ret);
		goto free_kz;
 	 }

  	cam_eeprom_format_calibration_common_data_read(p_eep,p_cmn);

 	for(i=0; i<(int)TL_E_MODE_MAX; i++){
  	    tl_ret = cam_eeprom_format_calibration_mode_data_read(
				tof_eeprom,
				&tof_eeprom->eeprom.mode[i],
				p_cmn + TL_EEPROM_OFFSET(TL_EEPROM_MODE_TOP(i)),i);
  	    if(tl_ret != TL_E_SUCCESS){
  	        CAM_ERR(CAM_EEPROM,"failed. read mode data ,tl_ret=%d",tl_ret);
		goto free_kz;
  	 }
	}
	for(i = 0; i <(int)e_size; i++){
		*(p_cmn + i)
			= (uint16_t)((uint16_t)(*(p_cmn + i) & 0x00FFU) << 8U)
			| ((uint16_t)(*(p_cmn + i) & 0xFF00U) >> 8U);
	}

	return tof_eeprom;
free_kz:
	kfree(tof_eeprom);
	return NULL;
}
