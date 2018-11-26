#ifndef H_TL_DEV_SENSOR_CONFIG
#define H_TL_DEV_SENSOR_CONFIG
/*-----------------------------------------*/
/* AFE setting address from EEPROM(mode)   */
/*-----------------------------------------*/
#define EEPROM_COMMON_DATA_TOP                   (0U)
#define EEPROM_MODE_DATA_SIZE                    (624U)
#define EEPROM_CMN_AREA_SIZE                     (912U)
#define EEPROM_MODE_DATA_NUM                     (2U)

#define EEPROM_MODE_DATA_TOP                     (EEPROM_COMMON_DATA_TOP+EEPROM_CMN_AREA_SIZE)
#define EEPROM_MODE_ADDR                         (EEPROM_MODE_DATA_TOP)
//pup_size
#define EEPROM_PUP_SIZE_ADDR                     (0x00AAU)
#define EEPROM_INIT_MAP_ADDR                     (0x0870U)

#define TL_AFE_READ_SIZE2_ADDR              (0xC3CCU)   /* when to start output */
#define TL_AFE_LDPOSBLKOUTEN_ADDR           (0xC084U)   /* LD positive edge setting */
#define TL_AFE_LDNEGBLKOUTEN_ADDR           (0xC085U)   /* LD negative edge setting */
#define TL_AFE_LD_ENABLE                    (0x0010U)   /* TAL enable setting data */

#define TL_AFE_TAL_EN_ADDR                  (0xC730U)   /* TAL enable setting address */
#define TL_AFE_TAL_DETECTOR_EN_ADDR         (0xC75EU)   /* TAL detector setting address */

#define TL_AFE_MODE_ADDR                    (0x4000U)   /* mode address */
#define TL_AFE_MODE0_VAL                    (0x0000U)   /* mode0 */
#define TL_AFE_MODE1_VAL                    (0x0001U)   /* mode1 */

#define TL_AFE_READ_SIZE_OFFSET     (7U)    /* offset of read_size2 */
#define TL_AFE_START_V_OFFSET       (750U)  /* offset of start_v */

#define TL_EEPROM_SUPPORT_VC        (0x0044U)   /* supported Virtual Channel */

#define TL_AFE_WDR                               (0x0002U)    /* WDR mode */
#define TL_EEPROM_OPT_AXIS_CENTER_H    (0x0090U)    /* optical axis center (horizonal) [pixel] */
/*---------------------*/
/* mode area mapping   */
/*---------------------*/
/* offset address from start address of mode0 area */
#define TL_EEPROM_TOF_MODE_FLAG        (0x0000U)    /* TOF mode */
#define TL_EEPROM_LD_FLAG              (0x0002U)    /* number of LD */
#define TL_EEPROM_RANGE_NEAR_LIMIT     (0x0004U)    /* near distance */
#define TL_EEPROM_RANGE_FAR_LIMIT      (0x0006U)    /* far distance */
#define TL_EEPROM_DEPTH_UNIT           (0x0008U)    /* unit of depth */

#define TL_EEPROM_EXP_MAX              (0x000CU)    /* exposure max value */
#define TL_EEPROM_EXP_DEFAULT          (0x000EU)    /* exposure default value */

#define TL_EEPROM_AE_DISTANCE          (0x0010U)    /* distance */
#define TL_EEPROM_AE_LUMINE_NUM        (0x0012U)    /* number of lumine */
#define TL_EEPROM_AE_SLOPE             (0x0014U)    /* slope correction */
#define TL_EEPROM_AE_ZOFST             (0x0016U)    /* zero point offset */

#define TL_EEPROM_EXP_ADDR_NUM         (0x0040U)    /* number of <long><short><lms> address */
#define TL_EEPROM_EXP_ADDR_LONG        (0x0042U)    /* address of long */
#define TL_EEPROM_EXP_ADDR_SHORT       (0x0060U)    /* address of short */
#define TL_EEPROM_EXP_ADDR_LMS         (0x0068U)    /* address of lms */

#define TL_EEPROM_DMMY_TRNS_NUM        (0x0070U)    /* number of dummy transfer address */
#define TL_EEPROM_DMMY_TRNS_ADR        (0x0072U)    /* address of dummy transfer */

#define TL_EEPROM_VD_INI_OFST          (0x007AU)    /* HD number from VD to exposure start */
#define TL_EEPROM_VD_INI_OFST_ADR_NUM  (0x007CU)    /* number of AFE address to set "VD_INI_OFST" */
#define TL_EEPROM_VD_INIT_OFST_ADR     (0x007EU)    /* address to set "VD_INI_OFST"  */
#define TL_EEPROM_LD_PLS_DUTY          (0x0086U)    /* TOF pulse duty ratio */
#define TL_EEPROM_VD_DURATION          (0x0088U)    /* VD length (HD number within 1 VD) ** this value is orignal VD - 2  */
#define TL_EEPROM_VD_REG_ADR           (0x008AU)    /* addres to set "VD_DURATION" */
#define TL_EEPROM_NUM_CLK_IN_HD        (0x008CU)    /* Number of clocks in 1 HD */
#define TL_EEPROM_BETA_NUM             (0x008EU)    /* Number of light emission and exposure */
#define TL_EEPROM_NUM_HD_IN_READOUT    (0x0090U)    /* Number of HDs in ReadOut */
#define TL_EEPROM_CLK_WIDTH_U          (0x0092U)    /* Reference clock(x1000000) : upper 2byte */
#define TL_EEPROM_CLK_WIDTH_L          (0x0094U)    /* Reference clock(x1000000) : lower 2byte */
#define TL_EEPROM_TOF_EMT_PERIOD_OFST  (0x0096U)    /* Number of HDs for adjusting the light emission exposure period */
#define TL_EEPROM_TOF_SEQ_INI_OFST     (0x0098U)    /* Offset from the positive edge of HD to the start of exposure */


#define TL_EEPROM_NLR_OFFSET           (0x00F0U)    /* nonlinear offset */
#define TL_EEPROM_NLR_X0               (0x0152U)    /* nonlinear setting */
#define TL_EEPROM_NLR_XPWR             (0x0154U)    /* nonlinear setting */

#define TL_EEPROM_DEPTH0               (0x0170U)    /* zero point offset */
#define TL_EEPROM_DEPTH2               (0x0172U)    /* slope correction of depth */
#define TL_EEPROM_DEPTH3               (0x0174U)    /* slope correction of depth */

#define TL_EEPROM_RATE_ADJUST          (0x0180U)    /* rate adjust */
#define TL_EEPROM_ALIGN                (0x0186U)    /* align */
#define TL_EEPROM_READ_SIZE0           (0x019CU)    /* read size */
#define TL_EEPROM_READ_SIZE3           (0x01a0U)
#define TL_EEPROM_ROI                  (0x01AAU)    /* ROI */

#define TL_EEPROM_GRID                 (0x01C0U)    /* grid conversion */

#define TL_EEPROM_RAWNR_XPWR           (0x01C6U)    /* Raw-NR setting */
#define TL_EEPROM_RAWNR_BL_TBL         (0x01CCU)    /* Raw-NR setting */
#define TL_EEPROM_RAWNR_MED            (0x01E6U)    /* Raw-NR setting */
#define TL_EEPROM_RAWNR_SAT_TH         (0x01E8U)    /* Raw-NR setting */
#define TL_EEPROM_RAWNR_BK_TBL         (0x01EAU)    /* Raw-NR setting */

#define TL_EEPROM_COR                  (0x0204U)    /* Raw-NR setting */
#define TL_EEPROM_CORB                 (0x020AU)    /* Raw-NR setting */
#define TL_EEPROM_CORF                 (0x0210U)    /* Raw-NR setting */

#define TL_EEPROM_DEPTH1               (0x0218U)    /* method of TOF */
#define TL_EEPROM_CONTROL              (0x021AU)    /* contorl of depth */

#define TL_EEPROM_PLS_MOD_CTRL         (0x021CU)    /* contorl of depth */
#define TL_EEPROM_PLS_MOD_VAL          (0x021EU)    /* contorl of depth */

/*  to reg common addr*/
#define EEPROM_CONFIG_DATA_MAX                   (27)

#define TL_AFE_SHD_OFFSET_ADDR                   (0xC600U)   /* depth shading offset */
#define TL_AFE_SHD_ADDR                          (0xC3C0U)
#define TL_AFE_SHD_X0_ADDR                       (0xC3C1U)
#define TL_AFE_SHD_XPWR_ADDR                     (0xC3C2U)
#define TL_AFE_SHD_Y0_ADDR                       (0xC3C6U)
#define TL_AFE_SHD_YPWR_ADDR                     (0xC3C7U)

#define TL_AFE_DFCT_PIX_TH_TBL_ADDR              (0xC318U)   /* dfct look up table value */
#define TL_AFE_DFCT_ADDR                         (0xC316U)   /* dfct setttng */

#define TL_AFE_SHP_LOC_ADDR                      (0xC044U)   /* SHP edge locations */
#define TL_AFE_SHD_LOC_ADDR                      (0xC046U)   /* SHD edge location */
#define TL_AFE_OUTPUT_ADDR                       (0xC3DAU)   /* AFE output setting */
#define TL_AFE_OUTPUTSEL_ADDR                    (0xC3DBU)   /* image output setting(default) */
#define TL_AFE_VC_ADDR                           (0xC3DCU)   /* MIPI virtual channel setting */

#define TL_AFE_GRID3_ADDR                        (0xC32AU)   /* cut out setting */

#define TL_AFE_IR_GAIN_GMM_ADDR                  (0xC371U)   /* IR gain & gamma setting */
#define TL_AFE_IR_GMM_ADDR                       (0xC372U)
#define TL_AFE_IR_GMM_Y_ADDR                     (0xC375U)

#define TL_AFE_CHKR_UPPRTH_ADDR                  (0xC30CU)   /* chkr setting */
#define	TL_AFE_CHKR_LWRTH_ADDR                   (0xc30dU)
#define TL_AFE_CHKR_START_V_ADDR                 (0xC30EU)   /* Vsync timing address */
#define	TL_AFE_CHKR_START_H_ADDR                 (0xc30fU)
#define	TL_AFE_CHKR_SIZE_H_ADDR                  (0xc310U)
#define	TL_AFE_CHKR_UPRERR_H_ADDR                (0xc311U)
#define	TL_AFE_CHKR_UPRERR_V_ADDR                (0xc312U)
#define	TL_AFE_CHKR_LWRERR_H_ADDR                (0xc313U)
#define	TL_AFE_CHKR_LWRERR_V_ADDR                (0xc314U)
#define	TL_AFE_CHKR_DET_ENA_ADDR                 (0xc315U)

//data_size
#define	EEPROM_CONFIG_SHD_OFFSET_SIZE              (192)
#define	EEPROM_CONFIG_SHD_SIZE                     (1)
#define	EEPROM_CONFIG_SHD_X0_SIZE                  (1)
#define	EEPROM_CONFIG_SHD_WPWR_SIZE                (4)
#define	EEPROM_CONFIG_SHD_Y0_SIZE                  (1)
#define	EEPROM_CONFIG_SHD_YPWR_SIZE                (3)

#define	EEPROM_CONFIG_DFCT_PIX_TH_TBL_SIZE         (12)
#define	EEPROM_CONFIG_DFCT_SIZE                    (2)

#define	EEPROM_CONFIG_TIMING_MIPI_SHP_LOC_SIZE     (1)
#define	EEPROM_CONFIG_TIMING_MIPI_SHD_LOC_SIZE     (1)
#define	EEPROM_CONFIG_TIMING_MIPI_OUTPUT_SIZE      (1)
#define	EEPROM_CONFIG_TIMING_MIPI_OUTPUT_SEL_SIZE  (1)
#define	EEPROM_CONFIG_TIMING_MIPI_VC_SIZE          (1)

#define	EEPROM_CONFIG_GRID3_SIZE_SIZE              (1)

#define	EEPROM_CONFIG_IR_SIZE                      (1)
#define	EEPROM_CONFIG_IR_GMM_SIZE                  (3)
#define	EEPROM_CONFIG_IR_GMM_Y_SIZE                (9)

#define	EEPROM_CONFIG_CHKR_UPPRTH_SIZE             (1)
#define	EEPROM_CONFIG_CHKR_LWRTH_SIZE              (1)
#define	EEPROM_CONFIG_CHKR_START_V_SIZE            (1)
#define	EEPROM_CONFIG_CHKR_START_H_SIZE            (1)
#define	EEPROM_CONFIG_CHKR_SIZE_H_SIZE             (1)
#define	EEPROM_CONFIG_CHKR_UPRERR_H_SIZE           (1)
#define	EEPROM_CONFIG_CHKR_UPRERR_V_SIZE           (1)
#define	EEPROM_CONFIG_CHKR_LWRERR_H_SIZE           (1)
#define	EEPROM_CONFIG_CHKR_LWRERR_V_SIZE           (1)
#define	EEPROM_CONFIG_CHKR_DET_ENA_SIZE            (1)

/*  from eeprom common data */
//shading
#define TL_EEPROM_SHD_OFFSET           (0x0150U)    /* shading setting */
#define TL_EEPROM_SHD                  (0x02D0U)    /* shading setting */
#define TL_EEPROM_SHD_X0               (0x02D2U)    /* shading setting */
#define TL_EEPROM_SHD_XPWR             (0x02D4U)    /* shading setting */
#define TL_EEPROM_SHD_Y0               (0x02DCU)    /* shading setting */
#define TL_EEPROM_SHD_YPWR             (0x02DEU)    /* shading setting */
//dfct
#define TL_EEPROM_DFCT_PIX_TH_TBL      (0x02F0U)    /* dfct look up table value */
#define TL_EEPROM_DFCT                 (0x0308U)    /* dfct setting */
//timing_mipi
#define TL_EEPROM_SHP_LOC              (0x030CU)    /* SHP edge locations */
#define TL_EEPROM_SHD_LOC              (0x030EU)    /* SHD edge location */
#define TL_EEPROM_OUTPUT               (0x0310U)    /* AFE output setting */
#define TL_EEPROM_OUTPUT_SEL           (0x0312U)    /* image output setting(default) */
#define TL_EEPROM_VC                   (0x0314U)    /* MIPI virtual channel setting */
//grid3
#define TL_EEPROM_GRID3                (0x0320U)    /* cut out */
//ir
#define TL_EEPROM_IR1                  (0x0322U)    /* IR gain */
#define TL_EEPROM_IR_GMM               (0x0324U)    /* IR gamma */
#define TL_EEPROM_IR_GMM_Y             (0x032AU)    /* IR gamma */
//chkr
#define TL_EEPROM_UPPRTH               (0x033CU)    /* houndstooth detection setting */
#define TL_EEPROM_LWRTH                (0x033EU)    /* houndstooth detection setting */
#define TL_EEPROM_START_V              (0x0340U)    /* houndstooth detection setting */
#define TL_EEPROM_START_H              (0x0342U)    /* houndstooth detection setting */
#define TL_EEPROM_SIZE_H               (0x0344U)    /* houndstooth detection setting */
#define TL_EEPROM_UPPRERR_H            (0x0346U)    /* houndstooth detection setting */
#define TL_EEPROM_UPPRERR_V            (0x0348U)    /* houndstooth detection setting */
#define TL_EEPROM_LWRERR_H             (0x034AU)    /* houndstooth detection setting */
#define TL_EEPROM_LWRERR_V             (0x034CU)    /* houndstooth detection setting */
#define TL_EEPROM_DET_ENA              (0x034EU)    /* houndstooth detection setting */

#define EEPROM_CONFIG_DATA_MODE_MAX                     (24)
/*  to reg mode addr*/
#define TL_AFE_NLR_OFFSET_ADDR              (0xC382U)   /* nlr data offset address */
#define TL_AFE_NLR_X0_ADDR                  (0xC3B3U)   /* nlr data x0 address */
#define TL_AFE_NLR_XPWR_ADDR                (0xC3B4U)   /* nlr data xpwr address */

#define TL_AFE_ZERO_OFFSET_ADDR             (0xC37EU)   /* zero point offset address */
#define TL_AFE_DEPTH_SLOPE_ADDR             (0xC380U)   /* depth slope address */
#define TL_AFE_DEPTH3_SLOPE_ADDR            (0xC381U)   /* depth3 slope address */

#define TL_AFE_RATE_ADJUST_ADDR             (0xC324U)   /* rate adjust */
#define TL_AFE_ALIGN_ADDR                   (0xC301U)   /* align */

#define TL_AFE_READ_SIZE0_ADDR              (0xC3CAU)   /* read size0 */
#define TL_AFE_READ_SIZE3_ADDR              (0xC3CDU)   /* read size3 */

#define TL_AFE_ROI_ADDR                     (0xC3D2U)   /* ROI */

#define TL_AFE_GRID_ADDR                    (0xC327U)   /* grid */

#define TL_AFE_RAWNR_XPWR_ADDR              (0xC347U)   /* RAW-NR xpwr */
#define TL_AFE_RAWNR_BLTBL_ADDR             (0xC338U)   /* RAW-NR bl_tbl */
#define TL_AFE_RAWNR_MED_ADDR               (0xC345U)   /* RAW-NR med */
#define	TL_AFE_SAT_TH_ADDR                  (0xC346U)
#define TL_AFE_RAWNR_BKTBL_ADDR             (0xC32BU)   /* RAW-NR bk_tbl */

#define TL_AFE_CORING_ADDR                  (0xC34AU)   /* coring address */
#define	TL_AFE_CORB_ADDR                    (0xc34dU)
#define	TL_AFE_CORF_ADDR                    (0xc350U)

#define TL_AFE_DEPTH1_ADDR                  (0xC37FU)   /* depth1 address */
#define TL_AFE_CONTROL_ADDR                 (0xC300U)   /* depth control address */

#define TL_AFE_PLS_MOD_CTRL_ADDR            (0x7D11U)   /* contorl of TOF pluse address */
#define TL_AFE_PLS_MOD_VAL_ADDR             (0xC710U)   /* contorl of TOF pluse address */


//data_size
#define EEPROM_NLR_OFFSET_SIZE                   (49)
#define EEPROM_NLR_X0_SIZE                       (1)
#define EEPROM_NLR_XPWR_SIZE                     (12)

#define EEPROM_DEPTH0_SIZE                       (1)
#define EEPROM_DEPTH2_SIZE                       (1)
#define EEPROM_DEPTH3_SIZE                       (1)

#define EEPROM_RATE_ADJUST_SIZE                  (3)
#define EEPROM_ALIGN_SIZE                        (11)

#define EEPROM_READ_SIZE0_SIZE                   (2)
#define EEPROM_READ_SIZE3_SIZE                   (5)

#define EEPROM_ROI_SIZE                          (8)

#define EEPROM_GRID_SIZE                         (3)

#define EEPROM_RAWNR_XPWR_SIZE                   (3)
#define EEPROM_RAWNR_BLTBL_SIZE                  (13)
#define EEPROM_MED_SIZE                          (1)
#define	EEPROM_SAT_TH_SIZE                       (1)
#define	EEPROM_RAWNR_BKTNL_SIZE                  (13)

#define	EEPROM_COR_SIZE                          (3)
#define	EEPROM_CORB_SIZE                         (3)
#define	EEPROM_CORF_SIZE                         (3)

#define	EEPROM_DEPTH1_SIZE                       (1)
#define	EEPROM_DEPTH_CTRL_SIZE                   (1)

#define	EEPROM_PLS_MOD_CTRL__SIZE                (1)
#define	EEPROM_PLS_MOD_VAL_SIZE                  (10)



/*  from eeprom mode data */
//shading
#define EEPROM_CONFIG_NLR_OFFSET_ADDR                   (TL_EEPROM_NLR_OFFSET)
#define EEPROM_CONFIG_NLR_X0_ADDR                       (TL_EEPROM_NLR_X0)
#define EEPROM_CONFIG_NLR_XPWR_ADDR                     (TL_EEPROM_NLR_XPWR)
#define EEPROM_CONFIG_DEPTH0_ADDR                       (TL_EEPROM_DEPTH0)
#define EEPROM_CONFIG_DEPTH2_ADDR                       (TL_EEPROM_DEPTH2)
#define EEPROM_CONFIG_DEPTH3_ADDR                       (TL_EEPROM_DEPTH3)

#define EEPROM_CONFIG_RATE_ADJUST_ADDR                  (TL_EEPROM_RATE_ADJUST)
#define EEPROM_CONFIG_ALIGN_ADDR                        (TL_EEPROM_ALIGN)
#define EEPROM_CONFIG_READ_SIZE0_ADDR                   (TL_EEPROM_READ_SIZE0)
#define EEPROM_CONFIG_READ_SIZE3_ADDR                   (TL_EEPROM_READ_SIZE3)

#define EEPROM_CONFIG_ROI_ADDR                          (TL_EEPROM_ROI)

#define EEPROM_CONFIG_GRID_ADDR                         (TL_EEPROM_GRID)

#define EEPROM_CONFIG_RAWNR_XPWR_ADDR                   (TL_EEPROM_RAWNR_XPWR)
#define EEPROM_CONFIG_RAWNR_BLTBL_ADDR                  (TL_EEPROM_RAWNR_BL_TBL)
#define EEPROM_CONFIG_MED_ADDR                          (TL_EEPROM_RAWNR_MED)
#define	EEPROM_CONFIG_SAT_TH_ADDR                       (TL_EEPROM_RAWNR_SAT_TH)
#define	EEPROM_CONFIG_RAWNR_BKTNL_ADDR                  (TL_EEPROM_RAWNR_BK_TBL)

#define	EEPROM_CONFIG_COR_ADDR                          (TL_EEPROM_COR)
#define	EEPROM_CONFIG_CORB_ADDR                         (TL_EEPROM_CORB)
#define	EEPROM_CONFIG_CORF_ADDR                         (TL_EEPROM_CORF)

#define	EEPROM_CONFIG_DEPTH1_ADDR                       (TL_EEPROM_DEPTH1)
#define	EEPROM_CONFIG_DEPTH_CTRL_ADDR                   (TL_EEPROM_CONTROL)

#define	EEPROM_CONFIG_PLS_MOD_CTRL_ADDR                 (TL_EEPROM_PLS_MOD_CTRL)
#define	EEPROM_CONFIG_PLS_MOD_VAL_ADDR                  (TL_EEPROM_PLS_MOD_VAL)

#endif//H_TL_DEV_SENSOR_CONFIG










