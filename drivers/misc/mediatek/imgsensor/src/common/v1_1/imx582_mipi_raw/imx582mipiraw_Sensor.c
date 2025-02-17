/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX582mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>
/* AIWORKS[20190722][start] stereo cam */
#include <linux/slab.h>
/* AIWORKS[20190722][end] stereo cam */

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "imx582mipiraw_Sensor.h"

#define IMX582_CAPTURE_48M         0
#define IMX582_PDAF_SWITCH         0
#define IMX582_OTP_SWTICH          1
#define AIWORKS_SUPPORT			   0

#define PFX "IMX582_camera_sensor"

#define DEVICE_VERSION_IMX582     "imx582"
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)
static DEFINE_SPINLOCK(imgsensor_drv_lock);

#if IMX582_PDAF_SWITCH
#define BYTE      unsigned char
extern BYTE imx582_LRC_data[384];
#endif
static int custom2_flag = 0;
extern BYTE imx582_QSC_data[2304];
/* AIWORKS[20190722][start] stereo cam */
#if AIWORKS_SUPPORT
#define EEPROM_WRITE_ID 0xA0
static struct semaphore aidualcam_thread_sem;
static int aidualcam_poweron;
static int aidualcam_thread_loop(void *arg);
extern void aidualcam_cal_init(void* p_data, int len);
#endif
/* AIWORKS[20190722][end] stereo cam */


static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX582_SENSOR_ID,

	.checksum_value = 0x4ded6277,  //0xffb1ec31,

	.pre = {
		/*setting for normal binning*/
		.pclk = 864000000,
		.linelength = 7872,
		.framelength = 3622,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 590400000,
		.max_framerate = 300,
	},
	#if IMX582_CAPTURE_48M
	.cap = {
		.pclk = 864000000,
		.linelength = 9184,
		.framelength = 6493,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8000,
		.grabwindow_height = 6000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 800000000,
		.max_framerate = 150,
	},
	#else
	.cap = {
		.pclk = 864000000,
		.linelength = 7872,
		.framelength = 3622,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 590400000,
		.max_framerate = 300,
	},
	#endif
	.normal_video = {
		/*setting for normal binning*/
		.pclk = 864000000,
		.linelength = 7872,
		.framelength = 3622,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 590400000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 864000000,
		.linelength = 2912,
		.framelength = 2472,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 480000000,
		.max_framerate = 1200,

	},
	.slim_video = {
		/*setting for normal binning*/
		.pclk = 864000000,
		.linelength = 7872,
		.framelength = 3622,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 590400000,
		.max_framerate = 300,
	},
	
	.custom1 = {		/*32M */
		/*setting for normal binning*/
		.pclk = 864000000,
		.linelength = 7872,
		.framelength = 3622,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 590400000,
		.max_framerate = 300,
	},
	.custom2 = {		/*48M@15fps*/
		.pclk = 864000000,
		.linelength = 9184,
		.framelength = 6493,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8000,
		.grabwindow_height = 6000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 800000000,
		.max_framerate = 150,
	},
	.custom3 = {		/*stero@34fps*/
		.pclk = 864000000,
		.linelength = 7872,
		.framelength = 1828,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2000,
		.grabwindow_height = 1500,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 600000000,
		.max_framerate = 600,
	},	
	
	.margin = 48,
	.min_shutter = 16,
	.min_gain = 64,
	.max_gain = 1024,
	.min_gain_iso = 100,
	.exp_step = 2,
	.gain_step = 1,
	.gain_type = 0,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.frame_time_delay_frame = 2,
	.ihdr_support = 0,	  /* 1, support; 0,not support */
	.ihdr_le_firstline = 0,  /* 1,le first ; 0, se first */
	.sensor_mode_num = 8,	  /* support sensor mode num */
	.cap_delay_frame = 2,
	.pre_delay_frame = 2,
	.video_delay_frame = 2,
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,
	
	.custom1_delay_frame = 2,	/*32M */
	.custom2_delay_frame = 2,	/*48M@15fps*/
	.custom3_delay_frame = 2,	/*stero@34fps*/
	
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, /* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0, /* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x34, 0x20, 0xff},
	.i2c_speed = 400, /* i2c read/write speed */
};


static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				/* mirrorflip information */
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview,
	*  Capture, Video,High Speed Video, Slim Video
	*/
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x3D0,					/* current shutter */
	.gain = 0x100,						/* current gain */
	.dummy_pixel = 0,					/* current dummypixel */
	.dummy_line = 0,					/* current dummyline */
	.current_fps = 300,  /* full size current fps : 24fps for PIP, 30fps for Normal or ZSD */
	/* auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker */
	.autoflicker_en = KAL_FALSE,
	/* test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output */
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,/* current scenario id */
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.hdr_mode = 0, /* HDR mODE : 0: disable HDR, 1:IHDR, 2:HDR, 9:ZHDR */
	.i2c_write_id = 0x34,

};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] = {
{ 8000,  6000,  0,  0,  8000,  6000,  4000,  3000,  0000,  0000,  4000,  3000,  0,  0,  4000,  3000},/*Preview*/
#if IMX582_CAPTURE_48M
{ 8000,  6000,  0,  0,  8000,  6000,  8000,  6000,  0000,  0000,  8000,  6000,  0,  0,  8000,  6000}, /* capture*/
#else
{ 8000,  6000,  0,  0,  8000,  6000,  4000,  3000,  0000,  0000,  4000,  3000,  0,  0,  4000,  3000},
#endif
{ 8000,  6000,  0,  0,  8000,  6000,  4000,  3000,  0000,  0000,  4000,  3000,  0,  0,  4000,  3000}, /* video */
{ 8000,  6000,  0,  0,  8000,  6000,  2000,  1500,  360,  390,  1280,  720,  0,  0,  1280,  720}, /*hs video*/
{ 8000,  6000,  0,  0,  8000,  6000,  4000,  3000,  0000,  0000,  4000,  3000,  0,  0,  4000,  3000}, /*slim video*/
{ 8000,  6000,  0,  0,  8000,  6000,  4000,  3000,  0000,  0000,  4000,  3000,  0,  0,  4000,  3000}, /*custom1*/
{ 8000,  6000,  0,  0,  8000,  6000,  8000,  6000,  0000,  0000,  8000,  6000,  0,  0,  8000,  6000}, /*custom2 48M@15fps*/
{ 8000,  6000,  0,  0,  8000,  6000,  2000,  1500,  0000,  0000,  2000,  1500,  0,  0,  2000,  1500}, /*custom3 stero@34fps*/
};


static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3] = {
	/* Preview mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0B40, 0x086C, 0x00, 0x12, 0x0E10, 0x0002, /*VC0:raw, VC1:Embedded data*/
	 0x00, 0x36, 0x100, 0x0001, 0x00, 0x00, 0x0000, 0x0000}, /*VC2:Y HIST(3HDR), VC3:AE HIST(3HDR)*/
	 //0x00, 0x33, 0x0E10, 0x0001, 0x00, 0x00, 0x0000, 0x0000}, /*VC4:Flicker(3HDR), VC5:no data*/
	/* Capture mode setting */
	{0x03, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x1680, 0x10D8, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x36, 0x100, 0x0001, 0x00, 0x00, 0x0000, 0x0000},
	/* Video mode setting */
	{0x02, 0x0a, 0x00, 0x08, 0x40, 0x00,
	 0x00, 0x2b, 0x0B40, 0x086C, 0x00, 0x00, 0x0000, 0x0000,
	 0x00, 0x36, 0x0FA0, 0x0001, 0x00, 0x00, 0x0000, 0x0000}
};
#if 0
/* If mirror flip */
static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 17,
	.i4OffsetY = 12,
	.i4PitchX = 8,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 2,
	.i4PosL = {{20, 13}, {18, 15}, {22, 17}, {24, 19},
		   {20, 21}, {18, 23}, {22, 25}, {24, 27} },
	.i4PosR = {{19, 13}, {17, 15}, {21, 17}, {23, 19},
		   {19, 21}, {17, 23}, {21, 25}, {23, 27} },
	.iMirrorFlip = 0,
	.i4BlockNumX = 496,
	.i4BlockNumY = 186,
	.i4Crop = {{0, 0}, {0, 0}, {0, 372}, {0, 0}, {0, 0},
		   {0, 0}, {0, 0}, {0,   0}, {0, 0}, {0, 0} }
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info_video = {
	.i4OffsetX = 17,
	.i4OffsetY = 12,
	.i4PitchX = 8,
	.i4PitchY = 16,
	.i4PairNum = 8,
	.i4SubBlkW = 8,
	.i4SubBlkH = 2,
	.i4PosL = {{20, 13}, {18, 15}, {22, 17}, {24, 19},
		   {20, 21}, {18, 23}, {22, 25}, {24, 27} },
	.i4PosR = {{19, 13}, {17, 15}, {21, 17}, {23, 19},
		   {19, 21}, {17, 23}, {21, 25}, {23, 27} },
	.iMirrorFlip = 0,
	.i4BlockNumX = 480,
	.i4BlockNumY = 134,
	.i4Crop = {{0, 0}, {0, 0}, {80, 420}, {0, 0}, {0, 0},
		   {0, 0}, {0, 0}, {0,   0}, {0, 0}, {0, 0} }
};
#endif

#define IMX582MIPI_MaxGainIndex (223)
kal_uint16 IMX582MIPI_sensorGainMapping[IMX582MIPI_MaxGainIndex][2] ={
	{72,114},
	{76,162},
	{80,205},
	{84,244},
	{88,279},
	{92,312},
	{96,341},
	{100,369},
	{104,394},
	{108,417},
	{112,439},
	{116,459},
	{120,478},
	{124,495},
	{128,512},
	{132,528},
	{136,542},
	{140,556},
	{144,569},
	{148,581},
	{152,593},
	{156,604},
	{160,614},
	{164,624},
	{168,634},
	{172,643},
	{176,652},
	{180,660},
	{184,668},
	{188,675},
	{192,683},
	{196,690},
	{200,696},
	{204,703},
	{208,709},
	{212,715},
	{216,721},
	{220,726},
	{224,731},
	{228,737},
	{232,742},
	{236,746},
	{240,751},
	{244,755},
	{248,760},
	{252,764},
	{256,768},
	{260,772},
	{264,776},
	{267,779},
	{272,783},
	{277,787},
	{280,790},
	{284,793},
	{287,796},
	{293,800},
	{297,803},
	{301,806},
	{303,808},
	{308,811},
	{312,814},
	{317,817},
	{320,819},
	{324,822},
	{328,824},
	{333,827},
	{336,829},
	{340,831},
	{343,833},
	{349,836},
	{352,838},
	{356,840},
	{360,842},
	{364,844},
	{368,846},
	{372,848},
	{377,850},
	{381,852},
	{383,853},
	{388,855},
	{392,857},
	{397,859},
	{400,860},
	{405,862},
	{407,863},
	{412,865},
	{415,866},
	{420,868},
	{423,869},
	{428,871},
	{431,872},
	{437,874},
	{440,875},
	{443,876},
	{449,878},
	{452,879},
	{455,880},
	{462,882},
	{465,883},
	{468,884},
	{471,885},
	{475,886},
	{478,887},
	{485,889},
	{489,890},
	{493,891},
	{496,892},
	{500,893},
	{504,894},
	{508,895},
	{512,896},
	{516,897},
	{520,898},
	{524,899},
	{529,900},
	{533,901},
	{537,902},
	{542,903},
	{546,904},
	{551,905},
	{555,906},
	{560,907},
	{565,908},
	{570,909},
	{575,910},
	{580,911},
	{585,912},
	{590,913},
	{596,914},
	{601,915},
	{607,916},
	{612,917},
	{618,918},
	{624,919},
	{630,920},
	{636,921},
	{643,922},
	{649,923},
	{655,924},
	{662,925},
	{669,926},
	{676,927},
	{683,928},
	{690,929},
	{697,930},
	{705,931},
	{712,932},
	{720,933},
	{728,934},
	{736,935},
	{745,936},
	{753,937},
	{762,938},
	{771,939},
	{780,940},
	{790,941},
	{799,942},
	{809,943},
	{819,944},
	{830,945},
	{840,946},
	{851,947},
	{862,948},
	{874,949},
	{886,950},
	{898,951},
	{910,952},
	{923,953},
	{936,954},
	{950,955},
	{964,956},
	{978,957},
	{993,958},
	{1008,959},
	{1024,960},
	{1040,961},
	{1057,962},
	{1074,963},
	{1092,964},
	{1111,965},
	{1130,966},
	{1150,967},
	{1170,968},
	{1192,969},
	{1214,970},
	{1237,971},
	{1260,972},
	{1285,973},
	{1311,974},
	{1337,975},
	{1365,976},
	{1394,977},
	{1425,978},
	{1456,979},
	{1489,980},
	{1524,981},
	{1560,982},
	{1598,983},
	{1638,984},
	{1680,985},
	{1725,986},
	{1771,987},
	{1820,988},
	{1872,989},
	{1928,990},
	{1986,991},
	{2048,992},
	{2114,993},
	{2185,994},
	{2260,995},
	{2341,996},
	{2427,997},
	{2521,998},
	{2621,999},
	{2731,1000},
	{2849,1001},
	{2979,1002},
	{3121,1003},
	{3277,1004},
	{3449,1005},
	{3641,1006},
	{3855,1007},
	{4096,1008},
};

static kal_uint32 ana_gain_table_8x[] = {
	100000,
	100196,
	100392,
	100589,
	100787,
	100986,
	101186,
	101386,
	101587,
	101789,
	101992,
	102196,
	102400,
	102605,
	102811,
	103018,
	103226,
	103434,
	103644,
	103854,
	104065,
	104277,
	104490,
	104703,
	104918,
	105133,
	105350,
	105567,
	105785,
	106004,
	106224,
	106445,
	106667,
	106889,
	107113,
	107338,
	107563,
	107789,
	108017,
	108245,
	108475,
	108705,
	108936,
	109168,
	109402,
	109636,
	109871,
	110108,
	110345,
	110583,
	110823,
	111063,
	111304,
	111547,
	111790,
	112035,
	112281,
	112527,
	112775,
	113024,
	113274,
	113525,
	113778,
	114031,
	114286,
	114541,
	114798,
	115056,
	115315,
	115576,
	115837,
	116100,
	116364,
	116629,
	116895,
	117162,
	117431,
	117701,
	117972,
	118245,
	118519,
	118794,
	119070,
	119347,
	119626,
	119906,
	120188,
	120471,
	120755,
	121040,
	121327,
	121615,
	121905,
	122196,
	122488,
	122782,
	123077,
	123373,
	123671,
	123971,
	124272,
	124574,
	124878,
	125183,
	125490,
	125799,
	126108,
	126420,
	126733,
	127047,
	127363,
	127681,
	128000,
	128321,
	128643,
	128967,
	129293,
	129620,
	129949,
	130280,
	130612,
	130946,
	131282,
	131620,
	131959,
	132300,
	132642,
	132987,
	133333,
	133681,
	134031,
	134383,
	134737,
	135092,
	135450,
	135809,
	136170,
	136533,
	136898,
	137265,
	137634,
	138005,
	138378,
	138753,
	139130,
	139510,
	139891,
	140274,
	140659,
	141047,
	141436,
	141828,
	142222,
	142618,
	143017,
	143417,
	143820,
	144225,
	144633,
	145042,
	145455,
	145869,
	146286,
	146705,
	147126,
	147550,
	147977,
	148406,
	148837,
	149271,
	149708,
	150147,
	150588,
	151032,
	151479,
	151929,
	152381,
	152836,
	153293,
	153754,
	154217,
	154683,
	155152,
	155623,
	156098,
	156575,
	157055,
	157538,
	158025,
	158514,
	159006,
	159502,
	160000,
	160502,
	161006,
	161514,
	162025,
	162540,
	163057,
	163578,
	164103,
	164630,
	165161,
	165696,
	166234,
	166775,
	167320,
	167869,
	168421,
	168977,
	169536,
	170100,
	170667,
	171237,
	171812,
	172391,
	172973,
	173559,
	174150,
	174744,
	175342,
	175945,
	176552,
	177163,
	177778,
	178397,
	179021,
	179649,
	180282,
	180919,
	181560,
	182206,
	182857,
	183513,
	184173,
	184838,
	185507,
	186182,
	186861,
	187546,
	188235,
	188930,
	189630,
	190335,
	191045,
	191760,
	192481,
	193208,
	193939,
	194677,
	195420,
	196169,
	196923,
	197683,
	198450,
	199222,
	200000,
	200784,
	201575,
	202372,
	203175,
	203984,
	204800,
	205622,
	206452,
	207287,
	208130,
	208980,
	209836,
	210700,
	211570,
	212448,
	213333,
	214226,
	215126,
	216034,
	216949,
	217872,
	218803,
	219742,
	220690,
	221645,
	222609,
	223581,
	224561,
	225551,
	226549,
	227556,
	228571,
	229596,
	230631,
	231674,
	232727,
	233790,
	234862,
	235945,
	237037,
	238140,
	239252,
	240376,
	241509,
	242654,
	243810,
	244976,
	246154,
	247343,
	248544,
	249756,
	250980,
	252217,
	253465,
	254726,
	256000,
	257286,
	258586,
	259898,
	261224,
	262564,
	263918,
	265285,
	266667,
	268063,
	269474,
	270899,
	272340,
	273797,
	275269,
	276757,
	278261,
	279781,
	281319,
	282873,
	284444,
	286034,
	287640,
	289266,
	290909,
	292571,
	294253,
	295954,
	297674,
	299415,
	301176,
	302959,
	304762,
	306587,
	308434,
	310303,
	312195,
	314110,
	316049,
	318012,
	320000,
	322013,
	324051,
	326115,
	328205,
	330323,
	332468,
	334641,
	336842,
	339073,
	341333,
	343624,
	345946,
	348299,
	350685,
	353103,
	355556,
	358042,
	360563,
	363121,
	365714,
	368345,
	371014,
	373723,
	376471,
	379259,
	382090,
	384962,
	387879,
	390840,
	393846,
	396899,
	400000,
	403150,
	406349,
	409600,
	412903,
	416260,
	419672,
	423140,
	426667,
	430252,
	433898,
	437607,
	441379,
	445217,
	449123,
	453097,
	457143,
	461261,
	465455,
	469725,
	474074,
	478505,
	483019,
	487619,
	492308,
	497087,
	501961,
	506931,
	512000,
	517172,
	522449,
	527835,
	533333,
	538947,
	544681,
	550538,
	556522,
	562637,
	568889,
	575281,
	581818,
	588506,
	595349,
	602353,
	609524,
	616867,
	624390,
	632099,
	640000,
	648101,
	656410,
	664935,
	673684,
	682667,
	691892,
	701370,
	711111,
	721127,
	731429,
	742029,
	752941,
	764179,
	775758,
	787692,
	800000,
};
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 2, imgsensor.i2c_write_id);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/ /* Add this func to set i2c speed by each sensor */
	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pusendcmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	/*kdSetI2CSpeed(imgsensor_info.i2c_speed);*/ /* Add this func to set i2c speed by each sensor */
	iReadRegI2C(pusendcmd, 2, (u8 *)&get_byte, 1, imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_INF("frame_length = %d, line_length = %d\n",
		imgsensor.frame_length,
		imgsensor.line_length);

	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
//	write_cmos_sensor_8(0x0342, imgsensor.line_length >> 8);
//	write_cmos_sensor_8(0x0343, imgsensor.line_length & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
}	/*	set_dummy  */

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{

	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_INF("framerate = %d, min framelength should enable %d\n", framerate, min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	if (frame_length >= imgsensor.min_frame_length)
		imgsensor.frame_length = frame_length;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;


	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
		/* Extend frame length */
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}
	} else {
		/* Extend frame length */
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);

}	/*	write_shutter  */

static void set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;

	 spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);
	/*Change frame time*/
	if (frame_length > 1)
		imgsensor.frame_length = frame_length;
/* */
	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		shutter = (imgsensor_info.max_frame_length - imgsensor_info.margin);

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
			write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter, imgsensor.frame_length);
}

/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	write_shutter(shutter);
}	/*	set_shutter */

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 i;

	for (i = 0; i < IMX582MIPI_MaxGainIndex; i++) {
		if(gain <= IMX582MIPI_sensorGainMapping[i][0]){
			break;
		}
	}
	if(gain != IMX582MIPI_sensorGainMapping[i][0])
		LOG_INF("Gain mapping don't correctly:%d %d \n", gain, IMX582MIPI_sensorGainMapping[i][0]);
	return IMX582MIPI_sensorGainMapping[i][1];

}


/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

    /* gain=1024;//for test */
    /* return; //for test */

	//if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
	if (gain < 72 || gain > 64 * BASEGAIN) {
	LOG_INF("Error gain setting");

	if (gain < 72)
		gain = 72;
	else if (gain > 64 * BASEGAIN)
		gain = 64 * BASEGAIN;
	}

	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor_8(0x0104, 0x01);
	write_cmos_sensor_8(0x0204, (reg_gain>>8) & 0xFF);
	write_cmos_sensor_8(0x0205, reg_gain & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	return gain;
}	/*	set_gain  */
/*
static void set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 itemp;

	LOG_INF("image_mirror = %d\n", image_mirror);
	itemp = read_cmos_sensor_8(0x0101);
	itemp &= ~0x03;

	switch (image_mirror) {

	case IMAGE_NORMAL:
	write_cmos_sensor_8(0x0101, itemp | 0x00);
	break;

	case IMAGE_V_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x02);
	break;

	case IMAGE_H_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x01);
	break;

	case IMAGE_HV_MIRROR:
	write_cmos_sensor_8(0x0101, itemp | 0x03);
	break;
	}
}
*/
/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#if 0
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}	/*	night_mode	*/
#endif

#define MULTI_WRITE 1

#if MULTI_WRITE
#define I2C_BUFFER_LEN 765	/* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3

#endif

static kal_uint16 imx582_table_write_cmos_sensor(kal_uint16 *para, kal_uint32 len)
{
	char puSendCmd[I2C_BUFFER_LEN];
	kal_uint32 tosend, IDX;
	kal_uint16 addr = 0, addr_last = 0, data;

	tosend = 0;
	IDX = 0;

	while (len > IDX) {
		addr = para[IDX];

		{
			puSendCmd[tosend++] = (char)(addr >> 8);
			puSendCmd[tosend++] = (char)(addr & 0xFF);
			data = para[IDX + 1];
			puSendCmd[tosend++] = (char)(data & 0xFF);
			IDX += 2;
			addr_last = addr;

		}
#if MULTI_WRITE
		/* Write when remain buffer size is less than 3 bytes or reach end of data */
		if ((I2C_BUFFER_LEN - tosend) < 3 || IDX == len || addr != addr_last) {
			iBurstWriteReg_multi(puSendCmd,
								tosend,
								imgsensor.i2c_write_id,
								3,
								imgsensor_info.i2c_speed);
			tosend = 0;
		}
#else
		iWriteRegI2C(puSendCmd, 3, imgsensor.i2c_write_id);
		tosend = 0;

#endif
	}
	return 0;
}

#if IMX582_PDAF_SWITCH
static void write_imx582_LRC_Data(void)
{
	kal_uint16 i;	

	for (i = 0; i < 192; i++) {
		write_cmos_sensor_8(0x7B10 + i, imx582_LRC_data[i]);
		//LOG_INF("imx582_LRC_data[%d] = 0x%x\n", i, imx582_LRC_data[i]);
	}
	for (i = 0; i < 192; i++) {
		write_cmos_sensor_8(0x7C00 + i, imx582_LRC_data[192 + i]);
		//LOG_INF("imx582_LRC_data[%d] = 0x%x\n", i + 192, imx582_LRC_data[i + 192]);
	}
}
#endif

static void write_imx582_QSC_Data(void)
{
	kal_uint16 i;	

	for (i = 0; i < 2304; i++) {
		write_cmos_sensor_8(0xC500 + i, imx582_QSC_data[i]);
		/* LOG_INF("imx582_qsc_data[%d] = 0x%x\n", i, imx582_qsc_data[i]); */
	}
}

kal_uint16 addr_data_pair_init_imx582[] = {
	0x0136, 0x18,
	0x0137, 0x00,
	0x3C7E, 0x01,
	0x3C7F, 0x04,
	0x6E28, 0x00,
	0x3C00, 0x10,
	0x3C01, 0x10,
	0x3C02, 0x10,
	0x3C03, 0x10,
	0x3C04, 0x10,
	0x3C05, 0x01,
	0x3C06, 0x00,
	0x3C07, 0x00,
	0x3C08, 0x03,
	0x3C09, 0xFF,
	0x3C0A, 0x01,
	0x3C0B, 0x00,
	0x3C0C, 0x00,
	0x3C0D, 0x03,
	0x3C0E, 0xFF,
	0x3C0F, 0x20,
	0x3F89, 0x01,
	0x5265, 0x13,
	0x5268, 0x05,
	0x526A, 0x27,
	0x526B, 0x23,
	0x5727, 0x77,
	0x5729, 0x05,
	0x572B, 0x0A,
	0x5737, 0x88,
	0x5739, 0x0F,
	0x573B, 0x0A,
	0x574B, 0x4D,
	0x574F, 0x0A,
	0x5757, 0xA8,
	0x5759, 0x14,
	0x575B, 0x0A,
	0x5765, 0x8C,
	0x5768, 0x00,
	0x5769, 0xE7,
	0x577D, 0x5A,
	0x577F, 0x00,
	0x5781, 0x0A,
	0x5783, 0x95,
	0x5787, 0xC2,
	0x5789, 0x06,
	0x578B, 0x0A,
	0x579B, 0x8A,
	0x579D, 0x00,
	0x579F, 0x0A,
	0x57A5, 0x95,
	0x57A9, 0x99,
	0x57AB, 0x01,
	0x57AF, 0xC9,
	0x57B3, 0x95,
	0x57BF, 0x00,
	0x57C1, 0x0A,
	0x57C9, 0x00,
	0x57CB, 0x0A,
	0x5801, 0x00,
	0x5803, 0x0A,
	0x5811, 0x00,
	0x5813, 0x0A,
	0x5B79, 0xB5,
	0x5B7B, 0x51,
	0x5C57, 0x54,
	0x5C59, 0xD9,
	0x5C5B, 0x90,
	0x5C5D, 0x18,
	0x5C5F, 0x2A,
	0x5C61, 0xAF,
	0x5C62, 0x01,
	0x5C63, 0xBA,
	0x5C65, 0x42,
	0x5C67, 0xC8,
	0x5C69, 0x4D,
	0x5C6B, 0xBC,
	0x5C6D, 0x44,
	0x5C6E, 0x01,
	0x5C6F, 0xE5,
	0x5C73, 0x34,
	0x5C74, 0x02,
	0x5C75, 0xBC,
	0x5C7F, 0x40,
	0x5C81, 0xA7,
	0x5C82, 0x01,
	0x5C83, 0xD8,
	0x5C85, 0xA6,
	0x5C8B, 0x6F,
	0x5C8C, 0x01,
	0x5C8D, 0xF7,
	0x5E2C, 0x03,
	0x5E2D, 0x03,
	0x6024, 0x05,
	0x6027, 0x05,
	0x6030, 0x05,
	0x6033, 0x05,
	0x603C, 0x05,
	0x603F, 0x05,
	0x6048, 0x05,
	0x604B, 0x05,
	0x606A, 0x08,
	0x606F, 0x08,
	0x6071, 0x08,
	0x6079, 0x06,
	0x607A, 0x07,
	0x607B, 0x07,
	0x607C, 0x06,
	0x607D, 0x0D,
	0x607E, 0x06,
	0x607F, 0x06,
	0x6080, 0x06,
	0x6081, 0x0E,
	0x6082, 0x06,
	0x6083, 0x06,
	0x6084, 0x0E,
	0x6085, 0x0B,
	0x6086, 0x0E,
	0x6087, 0x0E,
	0x6088, 0x0E,
	0x6089, 0x0B,
	0x608B, 0x0E,
	0x608C, 0x0E,
	0x6091, 0x06,
	0x6095, 0x06,
	0x6098, 0x06,
	0x6099, 0x06,
	0x609A, 0x06,
	0x609B, 0x06,
	0x609C, 0x06,
	0x609D, 0x06,
	0x609E, 0x06,
	0x609F, 0x06,
	0x60A0, 0x06,
	0x60B7, 0x32,
	0x60C1, 0x32,
	0x60C5, 0x32,
	0x60D5, 0x99,
	0x60DF, 0x99,
	0x60E3, 0x99,
	0x60F3, 0x99,
	0x60F7, 0xAD,
	0x60FB, 0xCE,
	0x60FD, 0x99,
	0x60FF, 0xCE,
	0x6103, 0xCE,
	0x6106, 0x16,
	0x6107, 0x15,
	0x6108, 0x0E,
	0x610A, 0x17,
	0x610B, 0x16,
	0x610C, 0x0F,
	0x610E, 0x18,
	0x610F, 0x16,
	0x6110, 0x0F,
	0x6112, 0x18,
	0x6113, 0x17,
	0x6114, 0x10,
	0x6116, 0x18,
	0x6117, 0x18,
	0x6118, 0x11,
	0x6147, 0x0D,
	0x6148, 0x0D,
	0x6149, 0x0D,
	0x614A, 0x0D,
	0x614B, 0x0D,
	0x614C, 0x0D,
	0x614D, 0x02,
	0x614E, 0x0D,
	0x614F, 0x0D,
	0x6150, 0x02,
	0x6151, 0x0D,
	0x6152, 0x02,
	0x6153, 0x02,
	0x6154, 0x02,
	0x6155, 0x02,
	0x6157, 0x21,
	0x6159, 0x0C,
	0x615B, 0x0C,
	0x615D, 0x0A,
	0x6161, 0x22,
	0x6163, 0x20,
	0x6165, 0x1F,
	0x6167, 0x1A,
	0x616B, 0x22,
	0x616D, 0x21,
	0x616F, 0x20,
	0x6171, 0x1B,
	0x6175, 0x23,
	0x6177, 0x22,
	0x6179, 0x21,
	0x617B, 0x1E,
	0x617F, 0x23,
	0x6181, 0x23,
	0x6183, 0x23,
	0x6185, 0x1E,
	0x6188, 0xB4,
	0x6189, 0x7E,
	0x618A, 0x96,
	0x618B, 0x50,
	0x618D, 0xD2,
	0x618E, 0xC8,
	0x618F, 0xB4,
	0x6190, 0x78,
	0x6192, 0xE6,
	0x6193, 0xCC,
	0x6194, 0xB4,
	0x6195, 0x82,
	0x6197, 0xE6,
	0x6198, 0xD2,
	0x6199, 0xD2,
	0x619A, 0xAA,
	0x619C, 0xE6,
	0x619D, 0xE8,
	0x619E, 0xDC,
	0x619F, 0xCD,
	0x61A1, 0x09,
	0x61A2, 0x0A,
	0x61A3, 0x0B,
	0x61A4, 0x0A,
	0x61A6, 0x08,
	0x61A7, 0x0A,
	0x61A8, 0x0B,
	0x61A9, 0x0A,
	0x61AB, 0x08,
	0x61AC, 0x0A,
	0x61AD, 0x0B,
	0x61AE, 0x0A,
	0x61B0, 0x08,
	0x61B1, 0x08,
	0x61B2, 0x0A,
	0x61B3, 0x09,
	0x61B5, 0x08,
	0x61B6, 0x08,
	0x61B7, 0x09,
	0x61B8, 0x09,
	0x61C4, 0x0F,
	0x61C5, 0x05,
	0x61C9, 0x14,
	0x61CA, 0x05,
	0x61CC, 0x0A,
	0x61CE, 0x14,
	0x61CF, 0x14,
	0x61D1, 0x23,
	0x621E, 0x50,
	0x621F, 0x3C,
	0x6220, 0x30,
	0x6221, 0x30,
	0x6223, 0x6E,
	0x6224, 0x5E,
	0x6225, 0x5C,
	0x6226, 0x5C,
	0x6229, 0x64,
	0x622A, 0x66,
	0x622B, 0x66,
	0x6E1D, 0x00,
	0x6E25, 0x00,
	0x6E38, 0x03,
	0x6E3B, 0x01,
	0x9004, 0x2C,
	0x9200, 0xF4,
	0x9201, 0xA7,
	0x9202, 0xF4,
	0x9203, 0xAA,
	0x9204, 0xF4,
	0x9205, 0xAD,
	0x9206, 0xF4,
	0x9207, 0xB0,
	0x9208, 0xF4,
	0x9209, 0xB3,
	0x920A, 0xB7,
	0x920B, 0x34,
	0x920C, 0xB7,
	0x920D, 0x36,
	0x920E, 0xB7,
	0x920F, 0x37,
	0x9210, 0xB7,
	0x9211, 0x38,
	0x9212, 0xB7,
	0x9213, 0x39,
	0x9214, 0xB7,
	0x9215, 0x3A,
	0x9216, 0xB7,
	0x9217, 0x3C,
	0x9218, 0xB7,
	0x9219, 0x3D,
	0x921A, 0xB7,
	0x921B, 0x3E,
	0x921C, 0xB7,
	0x921D, 0x3F,
	0x921E, 0x85,
	0x921F, 0x77,
	0x9226, 0x42,
	0x9227, 0x52,
	0x9228, 0x60,
	0x9229, 0xB9,
	0x922A, 0x60,
	0x922B, 0xBF,
	0x922C, 0x60,
	0x922D, 0xC5,
	0x922E, 0x60,
	0x922F, 0xCB,
	0x9230, 0x60,
	0x9231, 0xD1,
	0x9232, 0x60,
	0x9233, 0xD7,
	0x9234, 0x60,
	0x9235, 0xDD,
	0x9236, 0x60,
	0x9237, 0xE3,
	0x9238, 0x60,
	0x9239, 0xE9,
	0x923A, 0x60,
	0x923B, 0xEF,
	0x923C, 0x60,
	0x923D, 0xF5,
	0x923E, 0x60,
	0x923F, 0xF9,
	0x9240, 0x60,
	0x9241, 0xFD,
	0x9242, 0x61,
	0x9243, 0x01,
	0x9244, 0x61,
	0x9245, 0x05,
	0x924A, 0x61,
	0x924B, 0x6B,
	0x924C, 0x61,
	0x924D, 0x7F,
	0x924E, 0x61,
	0x924F, 0x92,
	0x9250, 0x61,
	0x9251, 0x9C,
	0x9252, 0x61,
	0x9253, 0xAB,
	0x9254, 0x61,
	0x9255, 0xC4,
	0x9256, 0x61,
	0x9257, 0xCE,
	0x9810, 0x14,
	0x9814, 0x14,
	0xA112, 0x68,
	0xA113, 0x56,
	0xA114, 0x2B,
	0xA115, 0x55,
	0xA116, 0x55,
	0xA117, 0x16,
	0xA119, 0x51,
	0xA11A, 0x34,
	0xA139, 0x4F,
	0xA13A, 0x48,
	0xA13B, 0x45,
	0xA13C, 0x02,
	0xA13F, 0x23,
	0xA140, 0x16,
	0xA141, 0x12,
	0xA142, 0x02,
	0xA596, 0x06,
	0xA597, 0x13,
	0xA598, 0x13,
	0xA779, 0x20,
	0xAC12, 0x01,
	0xAC13, 0x26,
	0xAC14, 0x01,
	0xAC15, 0x26,
	0xAC17, 0xC4,
	0xAF05, 0x48,
	0xB069, 0x02,
	0xC448, 0x01,
	0xC44B, 0x07,
	0xC44C, 0x0E,
	0xC44D, 0x50,
	0xC451, 0x00,
	0xC452, 0x01,
	0xC455, 0x00,
	0xE286, 0x31,
	0xE2A6, 0x32,
	0xE2C6, 0x33,
	0xEA4B, 0x00,
	0xEA4C, 0x00,
	0xEA4D, 0x00,
	0xEA4E, 0x00,
	0xF001, 0x10,
	0xF00D, 0x40,
	0xF031, 0x10,
	0xF03D, 0x40,
	0xF44B, 0x80,
	0xF44D, 0x06,
	0xF44E, 0x80,
	0xF450, 0x06,
	0xF451, 0x80,
	0xF453, 0x06,
	0xF454, 0x80,
	0xF456, 0x06,
	0xF457, 0x80,
	0xF459, 0x06,
	0xF478, 0x20,
	0xF47B, 0x20,
	0xF47E, 0x20,
	0xF481, 0x20,
	0xF484, 0x20,
	0x88D6, 0x60,
	0x9852, 0x00,
	0xAE09, 0xFF,
	0xAE0A, 0xFF,
	0xAE12, 0x58,
	0xAE13, 0x58,
	0xAE15, 0x10,
	0xAE16, 0x10,
	0xB071, 0x00,

};

kal_uint16 addr_data_pair_preview_imx582[] = {
	0x0112,  0x0A,
	0x0113,  0x0A,
	0x0114,  0x03,
	0x0342,  0x1E,
	0x0343,  0xC0,
	0x0340,  0x0E,
	0x0341,  0x26,
	0x0344,  0x00,
	0x0345,  0x00,
	0x0346,  0x00,
	0x0347,  0x00,
	0x0348,  0x1F,
	0x0349,  0x3F,
	0x034A,  0x17,
	0x034B,  0x6F,
	0x0900,  0x01,
	0x0901,  0x22,
	0x0902,  0x08,
	0x3246,  0x81,
	0x3247,  0x81,
	0x0401,  0x00,
	0x0404,  0x00,
	0x0405,  0x10,
	0x0408,  0x00,
	0x0409,  0x00,
	0x040A,  0x00,
	0x040B,  0x00,
	0x040C,  0x0F,
	0x040D,  0xA0,
	0x040E,  0x0B,
	0x040F,  0xB8,
	0x034C,  0x0F,
	0x034D,  0xA0,
	0x034E,  0x0B,
	0x034F,  0xB8,
	0x0301,  0x05,
	0x0303,  0x02,
	0x0305,  0x04,
	0x0306,  0x01,
	0x0307,  0x68,
	0x030B,  0x01,
	0x030D,  0x06,
	0x030E,  0x01,
	0x030F,  0x71,
	0x0310,  0x01,
	0x3620,  0x00,
	0x3621,  0x00,
	0x380C,  0x80,
	0x3C13,  0x00,
	0x3C14,  0x28,
	0x3C15,  0x28,
	0x3C16,  0x32,
	0x3C17,  0x46,
	0x3C18,  0x67,
	0x3C19,  0x8F,
	0x3C1A,  0x8F,
	0x3C1B,  0x99,
	0x3C1C,  0xAD,
	0x3C1D,  0xCE,
	0x3C1E,  0x8F,
	0x3C1F,  0x8F,
	0x3C20,  0x99,
	0x3C21,  0xAD,
	0x3C22,  0xCE,
	0x3C25,  0x22,
	0x3C26,  0x23,
	0x3C27,  0xE6,
	0x3C28,  0xE6,
	0x3C29,  0x08,
	0x3C2A,  0x0F,
	0x3C2B,  0x14,
	0x3F0C,  0x01,
	0x3F14,  0x00,
	0x3F80,  0x00,
	0x3F81,  0x00,
	0x3F82,  0x00,
	0x3F83,  0x00,
	0x3F8C,  0x07,
	0x3F8D,  0xD0,
	0x3FF4,  0x00,
	0x3FF5,  0x00,
	0x3FFC,  0x04,
	0x3FFD,  0xB0,
	0x0202,  0x0D,
	0x0203,  0xF6,
	0x0224,  0x01,
	0x0225,  0xF4,
	0x3FE0,  0x01,
	0x3FE1,  0xF4,
	0x0204,  0x00,
	0x0205,  0x70,
	0x0216,  0x00,
	0x0217,  0x70,
	0x0218,  0x01,
	0x0219,  0x00,
	0x020E,  0x01,
	0x020F,  0x00,
	0x0210,  0x01,
	0x0211,  0x00,
	0x0212,  0x01,
	0x0213,  0x00,
	0x0214,  0x01,
	0x0215,  0x00,
	0x3FE2,  0x00,
	0x3FE3,  0x70,
	0x3FE4,  0x01,
	0x3FE5,  0x00,
	0x3E20,  0x01,
	0x3E37,  0x01,
	
	0x38a3,0x02,  //0:16x12; 1:8x6; 2:Flex
	0x38ac,0x01,  //1:Flex win0 En; 0:Dis
	0x38ad,0x01,  //1:Flex win1 En; 0:Dis
	0x38ae,0x01,  //1:Flex win2 En; 0:Dis
	0x38af,0x01,  //1:Flex win3 En; 0:Dis
	0x38b0,0x01,  //1:Flex win4 En; 0:Dis
	0x38b1,0x01,  //1:Flex win5 En; 0:Dis
	0x38b2,0x01,  //1:Flex win6 En; 0:Dis
	0x38b3,0x00,  //1:Flex win7 En; 0:Dis
	0x38b4,0x05,  //Win0 Xtart H
	0x38B5,0xF0,  //Win0 Xtart L
	0x38B6,0x03,  //Win0 Ytart H
	0x38B7,0xFC,  //Win0 Ytart L
	0x38B8,0x09,  //Win0 Xend H
	0x38B9,0xB0,  //Win0 Xend L
	0x38BA,0x07,  //Win0 Yend H
	0x38BB,0xBC,  //Win0 Yend L
	0x38BC,0x00,  //Win1 Xtart H
	0x38BD,0x11,  //Win1 Xtart L
	0x38BE,0x00,  //Win1 Ytart H
	0x38BF,0x0C,  //Win1 Ytart L
	0x38C0,0x05,  //Win1 Xend H
	0x38C1,0xF0,  //Win1 Xend L
	0x38C2,0x05,  //Win1 Yend H
	0x38C3,0xDC,  //Win1 Yend L
	0x38C4,0x05,  //Win2 Xtart H
	0x38C5,0xF0,  //Win2 Xtart L
	0x38C6,0x00,  //Win2 Ytart H
	0x38C7,0x0C,  //Win2 Ytart L
	0x38C8,0x09,  //Win2 Xend H
	0x38C9,0xB0,  //Win2 Xend L
	0x38CA,0x03,  //Win2 Yend H
	0x38CB,0xFC,  //Win2 Yend L
	0x38CC,0x09,  //Win3 Xtart H
	0x38CD,0xB0,  //Win3 Xtart L
	0x38CE,0x00,  //Win3 Ytart H
	0x38CF,0x0C,  //Win3 Ytart L
	0x38D0,0x0F,  //Win3 Xend H
	0x38D1,0x8E,  //Win3 Xend L
	0x38D2,0x05,  //Win3 Yend H
	0x38D3,0xDC,  //Win3 Yend L
	0x38D4,0x00,  //Win4 Yend H
	0x38D5,0x11,  //Win4 Yend L
	0x38D6,0x05,  //Win4 Ytart H
	0x38D7,0xDC,  //Win4 Ytart L
	0x38D8,0x05,  //Win4 Xend H
	0x38D9,0xF0,  //Win4 Xend L
	0x38DA,0x0B,  //Win4 Yend H
	0x38DB,0xAB,  //Win4 Yend L
	0x38DC,0x05,  //Win5 Yend H
	0x38DD,0xF0,  //Win5 Yend L
	0x38DE,0x07,  //Win5 Ytart H
	0x38DF,0xBC,  //Win5 Ytart L
	0x38E0,0x09,  //Win5 Xend H
	0x38E1,0xB0,  //Win5 Xend L
	0x38E2,0x0B,  //Win5 Yend H
	0x38E3,0xAB,  //Win5 Yend L
	0x38E4,0x09,  //Win6 Yend H
	0x38E5,0xB0,  //Win6 Yend L
	0x38E6,0x05,  //Win6 Ytart H
	0x38E7,0xDC,  //Win6 Ytart L
	0x38E8,0x0F,  //Win6 Xend H
	0x38E9,0x8E,  //Win6 Xend L
	0x38EA,0x0B,  //Win6 Yend H
	0x38EB,0xAB,  //Win6 Yend L
};
#if IMX582_CAPTURE_48M
kal_uint16 addr_data_pair_capture_imx582[] = {
	
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0xE0,
	0x0340, 0x19,
	0x0341, 0x5D,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3246, 0x01,
	0x3247, 0x01,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x1F,
	0x040D, 0x40,
	0x040E, 0x17,
	0x040F, 0x70,
	0x034C, 0x1F,
	0x034D, 0x40,
	0x034E, 0x17,
	0x034F, 0x70,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x06,
	0x030E, 0x01,
	0x030F, 0xF4,
	0x0310, 0x01,
	0x3620, 0x01,
	0x3621, 0x01,
	0x380C, 0x80,
	0x3C13, 0x00,
	0x3C14, 0x28,
	0x3C15, 0x28,
	0x3C16, 0x32,
	0x3C17, 0x46,
	0x3C18, 0x67,
	0x3C19, 0x8F,
	0x3C1A, 0x8F,
	0x3C1B, 0x99,
	0x3C1C, 0xAD,
	0x3C1D, 0xCE,
	0x3C1E, 0x8F,
	0x3C1F, 0x8F,
	0x3C20, 0x99,
	0x3C21, 0xAD,
	0x3C22, 0xCE,
	0x3C25, 0x22,
	0x3C26, 0x23,
	0x3C27, 0xE6,
	0x3C28, 0xE6,
	0x3C29, 0x08,
	0x3C2A, 0x0F,
	0x3C2B, 0x14,
	0x3F0C, 0x00,
	0x3F14, 0x01,
	0x3F80, 0x04,
	0x3F81, 0xB0,
	0x3F82, 0x00,
	0x3F83, 0x00,
	0x3F8C, 0x03,
	0x3F8D, 0x5C,
	0x3FF4, 0x00,
	0x3FF5, 0x00,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
	0x0202, 0x19,
	0x0203, 0x2D,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x01,
	0x3E37, 0x01,
};
#endif
kal_uint16 addr_data_pair_custom1_imx582[] = {
	0x0112,	0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x39,
	0x0343, 0x70,
	0x0340, 0x17,
	0x0341, 0x2D,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x48,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x27,
	0x0220, 0x22,
	0x0222, 0x01,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3140, 0x00,
	0x3246, 0x01,
	0x3247, 0x01,
	0x3F15, 0x00,
	0x0401, 0x02,
	0x0404, 0x00,
	0x0405, 0x13,
	0x0408, 0x00,
	0x0409, 0x68,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x1E,
	0x040D, 0x70,
	0x040E, 0x16,
	0x040F, 0xDC,
	0x034C, 0x19,
	0x034D, 0xA0,
	0x034E, 0x13,
	0x034F, 0x40,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x35,
	0x030B, 0x01,
	0x030D, 0x0C,
	0x030E, 0x03,
	0x030F, 0x66,
	0x0310, 0x01,
	0x3620, 0x01,
	0x3621, 0x01,
	0x3F0C, 0x00,
	0x3F14, 0x01,
	0x3F80, 0x00,
	0x3F81, 0x00,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
	0x0202, 0x16,
	0x0203, 0xFD,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x00,
	0x0216, 0x00,
	0x0217, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x00,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
	0x3E20, 0x01,
	0x3E37, 0x00,
};

kal_uint16 addr_data_pair_custom2_imx582[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
	0x0342, 0x23,
	0x0343, 0xE0,
	0x0340, 0x19,
	0x0341, 0x5D,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x17,
	0x034B, 0x6F,
	0x0900, 0x00,
	0x0901, 0x11,
	0x0902, 0x0A,
	0x3246, 0x01,
	0x3247, 0x01,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x1F,
	0x040D, 0x40,
	0x040E, 0x17,
	0x040F, 0x70,
	0x034C, 0x1F,
	0x034D, 0x40,
	0x034E, 0x17,
	0x034F, 0x70,
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x06,
	0x030E, 0x01,
	0x030F, 0xF4,
	0x0310, 0x01,
	0x3620, 0x01,
	0x3621, 0x01,
	0x380C, 0x80,
	0x3C13, 0x00,
	0x3C14, 0x28,
	0x3C15, 0x28,
	0x3C16, 0x32,
	0x3C17, 0x46,
	0x3C18, 0x67,
	0x3C19, 0x8F,
	0x3C1A, 0x8F,
	0x3C1B, 0x99,
	0x3C1C, 0xAD,
	0x3C1D, 0xCE,
	0x3C1E, 0x8F,
	0x3C1F, 0x8F,
	0x3C20, 0x99,
	0x3C21, 0xAD,
	0x3C22, 0xCE,
	0x3C25, 0x22,
	0x3C26, 0x23,
	0x3C27, 0xE6,
	0x3C28, 0xE6,
	0x3C29, 0x08,
	0x3C2A, 0x0F,
	0x3C2B, 0x14,
	0x3F0C, 0x00,
	0x3F14, 0x01,
	0x3F80, 0x04,
	0x3F81, 0xB0,
	0x3F82, 0x00,
	0x3F83, 0x00,
	0x3F8C, 0x03,
	0x3F8D, 0x5C,
	0x3FF4, 0x00,
	0x3FF5, 0x00,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
	0x0202, 0x19,
	0x0203, 0x2D,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
		       
	0x3E20, 0x01,
		       
	0x3E37, 0x01,
};

kal_uint16 addr_data_pair_custom3_imx582[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
		       
	0x0342, 0x1E,
	0x0343, 0xC0,
		       
	0x0340, 0x07,
	0x0341, 0x24,
		       
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x06,
	0x0347, 0xA8,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x10,
	0x034B, 0xC7,
		       
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x08,
	0x3246, 0x81,
	0x3247, 0x81,
		       
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x02,
	0x0409, 0x50,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x0B,
	0x040D, 0x00,
	0x040E, 0x05,
	0x040F, 0x10,
		       
	0x034C, 0x0B,
	0x034D, 0x00,
	0x034E, 0x05,
	0x034F, 0x10,
		     
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x06,
	0x030E, 0x01,
	0x030F, 0x77,
	0x0310, 0x01,
		       
	0x3620, 0x00,
	0x3621, 0x00,
	0x380C, 0x80,
	0x3C13, 0x00,
	0x3C14, 0x28,
	0x3C15, 0x28,
	0x3C16, 0x32,
	0x3C17, 0x46,
	0x3C18, 0x67,
	0x3C19, 0x8F,
	0x3C1A, 0x8F,
	0x3C1B, 0x99,
	0x3C1C, 0xAD,
	0x3C1D, 0xCE,
	0x3C1E, 0x8F,
	0x3C1F, 0x8F,
	0x3C20, 0x99,
	0x3C21, 0xAD,
	0x3C22, 0xCE,
	0x3C25, 0x22,
	0x3C26, 0x23,
	0x3C27, 0xE6,
	0x3C28, 0xE6,
	0x3C29, 0x08,
	0x3C2A, 0x0F,
	0x3C2B, 0x14,
	0x3F0C, 0x01,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x00,
	0x3F82, 0x00,
	0x3F83, 0x00,
	0x3F8C, 0x07,
	0x3F8D, 0xD0,
	0x3FF4, 0x00,
	0x3FF5, 0x00,
	0x3FFC, 0x04,
	0x3FFD, 0xB0,
		       
	0x0202, 0x06,
	0x0203, 0xF4,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
		       
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
		      
	0x3E20, 0x01,
		       
	0x3E37, 0x01,
};

kal_uint16 addr_data_pair_hs_video_imx582[] = {
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x03,
		       
	0x0342, 0x0B,
	0x0343, 0x60,
		       
	0x0340, 0x04,
	0x0341, 0xD4,
		       
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x06,
	0x0347, 0x10,
	0x0348, 0x1F,
	0x0349, 0x3F,
	0x034A, 0x11,
	0x034B, 0x4F,
		       
	0x0900, 0x01,
	0x0901, 0x44,
	0x0902, 0x08,
	0x3246, 0x89,
	0x3247, 0x89,
		       
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x01,
	0x0409, 0x68,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x05,
	0x040D, 0x00,
	0x040E, 0x02,
	0x040F, 0xD0,
		       
	0x034C, 0x05,
	0x034D, 0x00,
	0x034E, 0x02,
	0x034F, 0xD0,
		       
	0x0301, 0x05,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x68,
	0x030B, 0x01,
	0x030D, 0x06,
	0x030E, 0x01,
	0x030F, 0x2c,
	0x0310, 0x01,
		       
	0x3620, 0x00,
	0x3621, 0x00,
	0x380C, 0x80,
	0x3C13, 0x00,
	0x3C14, 0x28,
	0x3C15, 0x28,
	0x3C16, 0x32,
	0x3C17, 0x46,
	0x3C18, 0x67,
	0x3C19, 0x8F,
	0x3C1A, 0x8F,
	0x3C1B, 0x99,
	0x3C1C, 0xAD,
	0x3C1D, 0xCE,
	0x3C1E, 0x8F,
	0x3C1F, 0x8F,
	0x3C20, 0x99,
	0x3C21, 0xAD,
	0x3C22, 0xCE,
	0x3C25, 0x22,
	0x3C26, 0x23,
	0x3C27, 0xE6,
	0x3C28, 0xE6,
	0x3C29, 0x08,
	0x3C2A, 0x0F,
	0x3C2B, 0x14,
	0x3F0C, 0x00,
	0x3F14, 0x00,
	0x3F80, 0x00,
	0x3F81, 0x00,
	0x3F82, 0x00,
	0x3F83, 0x00,
	0x3F8C, 0x00,
	0x3F8D, 0x00,
	0x3FF4, 0x00,
	0x3FF5, 0x4C,
	0x3FFC, 0x00,
	0x3FFD, 0x00,
		       
	0x0202, 0x04,
	0x0203, 0xA4,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x3FE0, 0x01,
	0x3FE1, 0xF4,
		       
	0x0204, 0x00,
	0x0205, 0x70,
	0x0216, 0x00,
	0x0217, 0x70,
	0x0218, 0x01,
	0x0219, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0210, 0x01,
	0x0211, 0x00,
	0x0212, 0x01,
	0x0213, 0x00,
	0x0214, 0x01,
	0x0215, 0x00,
	0x3FE2, 0x00,
	0x3FE3, 0x70,
	0x3FE4, 0x01,
	0x3FE5, 0x00,
		       
	0x3E20, 0x01,
		       
	0x3E37, 0x00,
};

static void sensor_init(void)
{
	LOG_INF("E\n");
	imx582_table_write_cmos_sensor(addr_data_pair_init_imx582,
							   sizeof(addr_data_pair_init_imx582) / sizeof(kal_uint16));
	LOG_INF("L\n");
} /*	sensor_init  */

#if 0	//IMX582_OTP_SWTICH
/*************************************************************************
* FUNCTION
*    read_eeprom_data
*
* DESCRIPTION
*    This function read data from EEprom.add by he .
* 
*************************************************************************/

static  kal_uint16 read_eeprom_data(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, EEPROM_WRITE_ID);

    return get_byte;
}

/*************************************************************************
* FUNCTION
*    load_imx582_awb
*
* DESCRIPTION
*    This function apply AWB otp data into sensor.add by he .
*
* 
*************************************************************************/
static void  cal_wb(void)
{
    kal_uint32 Unit_RG, Unit_BG, Unit_GG, Golden_RG, Golden_BG, Golden_GG, r_ratio, b_ratio;
    kal_uint32 R_Gain, B_Gain, G_Gain, G_Gain_R, G_Gain_B;
    kal_uint16 a1, b1, a2, b2, a3, b3, a4, b4;

    #define GAIN_DEFAULT 0x0100 
    
    Unit_RG   = ((read_eeprom_data(0x0022) << 8) | read_eeprom_data(0x0023));
    Unit_BG   = ((read_eeprom_data(0x0024) << 8) | read_eeprom_data(0x0025));
    Unit_GG   = ((read_eeprom_data(0x0026) << 8) | read_eeprom_data(0x0027));
    Golden_RG = ((read_eeprom_data(0x0028) << 8) | read_eeprom_data(0x0029));
    Golden_BG = ((read_eeprom_data(0x002A) << 8) | read_eeprom_data(0x002B));
    Golden_GG = ((read_eeprom_data(0x002C) << 8) | read_eeprom_data(0x002D));

    LOG_INF("Unit_RG = 0x%x, Unit_BG = 0x%x, Unit_GG = 0x%x\n", Unit_RG, Unit_BG, Unit_GG);
    LOG_INF("Golden_RG = 0x%x, Golden_BG = 0x%x, Golden_GG = 0x%x\n", Golden_RG, Golden_BG, Golden_GG);


    r_ratio = 512 * (Golden_RG) /(Unit_RG);
    b_ratio = 512 * (Golden_BG) /(Unit_BG);
//if(r_ratio == 0 || b_ratio == 0) return 0;

    if(r_ratio >= 512 )
    {
        if(b_ratio>=512) 
        {
            R_Gain = (unsigned short)(GAIN_DEFAULT * r_ratio / 512);
            G_Gain = GAIN_DEFAULT;    
            B_Gain = (unsigned short)(GAIN_DEFAULT * b_ratio / 512);
        }
        else
        {
            R_Gain = (unsigned short)(GAIN_DEFAULT * r_ratio / b_ratio);
            G_Gain = (unsigned short)(GAIN_DEFAULT * 512 / b_ratio);
            B_Gain = GAIN_DEFAULT;    
        }
    }
    else             
    {
        if(b_ratio >= 512)
        {
            R_Gain = GAIN_DEFAULT;    
            G_Gain =(unsigned short)(GAIN_DEFAULT * 512 / r_ratio);
            B_Gain =(unsigned short)(GAIN_DEFAULT *  b_ratio / r_ratio);

        } 
        else 
        {
            G_Gain_R = (unsigned short)(GAIN_DEFAULT * 512 / r_ratio );
            G_Gain_B = (unsigned short)(GAIN_DEFAULT * 512 / b_ratio );

            if(G_Gain_R >= G_Gain_B)
            {
                R_Gain = GAIN_DEFAULT;
                G_Gain = (unsigned short)(GAIN_DEFAULT * 512 / r_ratio );
                B_Gain = (unsigned short)(GAIN_DEFAULT * b_ratio / r_ratio);
            } 
            else
            {
                R_Gain = (unsigned short)(GAIN_DEFAULT * r_ratio / b_ratio );
                G_Gain = (unsigned short)(GAIN_DEFAULT * 512 / b_ratio );
                B_Gain = GAIN_DEFAULT;
            }
        }    
    }
    

    LOG_INF("R_Gain=0x%x, B_Gain=0x%x, G_Gain=0x%x\n", R_Gain, B_Gain, G_Gain);
    
    write_cmos_sensor_8(0x3130, 0x01);    //imx582 need del by he

    if(R_Gain>0x0100)
    {
        write_cmos_sensor_8(0x0210, (R_Gain & 0xff00)>>8);
        write_cmos_sensor_8(0x0211, (R_Gain & 0x00ff)); //R
    }

    if(G_Gain>0x0100)
    {
        write_cmos_sensor_8(0x020e, (G_Gain & 0xff00)>>8);
        write_cmos_sensor_8(0x020f, (G_Gain & 0x00ff));//GR

        write_cmos_sensor_8(0x0214, (G_Gain & 0xff00)>>8);
        write_cmos_sensor_8(0x0215, (G_Gain & 0x00ff));//GB
    }

    if(B_Gain>0x0100)
    {
        write_cmos_sensor_8(0x0212, (B_Gain & 0xff00)>>8);
        write_cmos_sensor_8(0x0213, (B_Gain & 0x00ff)); //B
    }


    a1 = read_cmos_sensor_8(0x0210); 
    b1 = read_cmos_sensor_8(0x0211);
    a2 = read_cmos_sensor_8(0x020e); 
    b2 = read_cmos_sensor_8(0x020f);
    a3 = read_cmos_sensor_8(0x0214); 
    b3 = read_cmos_sensor_8(0x0215);
    a4 = read_cmos_sensor_8(0x0212); 
    b4 = read_cmos_sensor_8(0x0213);

    LOG_INF("0x0210 = 0x%x, 0x0211 = 0x%x\n", a1, b1);
    LOG_INF("0x020e = 0x%x, 0x020f = 0x%x\n", a2, b2);
    LOG_INF("0x0214 = 0x%x, 0x0215 = 0x%x\n", a3, b3);
    LOG_INF("0x0212 = 0x%x, 0x0213 = 0x%x\n", a4, b4);

}

#endif


static void preview_setting(void)
{

	LOG_INF("E binning_normal_setting\n");
	imx582_table_write_cmos_sensor(addr_data_pair_preview_imx582,
		sizeof(addr_data_pair_preview_imx582) / sizeof(kal_uint16));

	LOG_INF("L\n");
}	/*	preview_setting  */

/* Pll Setting - VCO = 280Mhz */
static void capture_setting(kal_uint16 currefps, kal_bool stream_on)
{
	LOG_INF("E! currefps:%d\n", currefps);
	
	#if IMX582_CAPTURE_48M
	imx582_table_write_cmos_sensor(addr_data_pair_capture_imx582,
						   sizeof(addr_data_pair_capture_imx582) / sizeof(kal_uint16));
	#else
	preview_setting();
	#endif
	LOG_INF("L!\n");
}

static void normal_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();	/* Tower modify 20160214 */
}

static void hs_video_setting(void)
{
	LOG_INF("E\n");
	imx582_table_write_cmos_sensor(addr_data_pair_hs_video_imx582,
		sizeof(addr_data_pair_hs_video_imx582) / sizeof(kal_uint16));
}

static void slim_video_setting(void)
{
	LOG_INF("E\n");
	preview_setting();
}

static void custom1_setting(void)
{
	/* custom1 32M setting */
	LOG_INF("E\n");
#if 0
	imx582_table_write_cmos_sensor(addr_data_pair_custom1_imx582,
		sizeof(addr_data_pair_custom1_imx582) / sizeof(kal_uint16));
#else
	preview_setting();
#endif
}

static void custom2_setting(void)
{
	/* custom2 48M@15fps setting */
	LOG_INF("E\n");
	imx582_table_write_cmos_sensor(addr_data_pair_custom2_imx582,
						   sizeof(addr_data_pair_custom2_imx582) / sizeof(kal_uint16));
}

static void custom3_setting(void)
{
	/* custom3 stero@34fps setting */
	LOG_INF("E\n");
	imx582_table_write_cmos_sensor(addr_data_pair_custom3_imx582,
						   sizeof(addr_data_pair_custom3_imx582) / sizeof(kal_uint16));
}

static kal_uint32 return_lot_id_from_otp(void)
{
	kal_uint32 sensor_id = 0;

	LOG_INF("0x0016 0x%x 0x0017 0x%x\n",
		read_cmos_sensor_8(0x0016), read_cmos_sensor_8(0x0017));
	sensor_id = (read_cmos_sensor_8(0x0016) << 8) | (read_cmos_sensor_8(0x0017));

	if (sensor_id == IMX582_SENSOR_ID) {
		LOG_INF("This is IMX582\n");
		return sensor_id;
	}

	return 0;
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	*sensorID : return the sensor ID
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
    /* AIWORKS[20190722][start] stereo cam */
	#if AIWORKS_SUPPORT
	int sys_ret = 0;

	sys_ret = down_interruptible(&aidualcam_thread_sem);
	if (0 == sys_ret) {
		aidualcam_poweron = 1;
		up(&aidualcam_thread_sem);
	}
	printk("Imx582 get_imgsensor_id ID !\n");
	#endif
    /* AIWORKS[20190722][end] stereo cam */
	printk("Imx582 get_imgsensor_id ID !\n");
	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_lot_id_from_otp();
			printk("read_sensro_id sensor_id = 0x%x\n", *sensor_id);
			if (*sensor_id == imgsensor_info.sensor_id) {
				printk("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, *sensor_id);

			#if IMX582_OTP_SWTICH
				imx582_get_otp_data();
			#endif		

	            /* AIWORKS[20190722][start] stereo cam */
				#if AIWORKS_SUPPORT
				sys_ret = aidualcam_thread_loop(NULL);
				#endif
	            /* AIWORKS[20190722][end] stereo cam */

				return ERROR_NONE;
			}
			printk("Read sensor id fail, i2c_write_id: 0x%x  sensor_id: 0x%x\n",
				imgsensor.i2c_write_id, *sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}

	if (*sensor_id != imgsensor_info.sensor_id) {
		/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_lot_id_from_otp();
			if (sensor_id == imgsensor_info.sensor_id) {
		
			LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id, sensor_id);
			break;
		}
			LOG_INF("Read sensor id fail, id: 0x%x sensor_id=0x%x\n ", imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}

	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	/* initail sequence write in  */

	sensor_init();
	
#if IMX582_OTP_SWTICH
	load_imx582_awb();
#endif

#if IMX582_PDAF_SWITCH
	write_imx582_LRC_Data();
#endif
	
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_mode = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    /* AIWORKS [20190722][start] stereo cam */
	#if AIWORKS_SUPPORT
	int sys_ret;

	sys_ret = down_interruptible(&aidualcam_thread_sem);
	if(0 == sys_ret){
		aidualcam_poweron = 0;
		up(&aidualcam_thread_sem);
	}
	#endif
    /* AIWORKS [20190722][end] stereo cam */
	/*No Need to implement this function*/
	LOG_INF("E\n");
	custom2_flag = 0;
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E imgsensor.hdr_mode=%d\n", imgsensor.hdr_mode);

	LOG_INF("E preview normal\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();

	return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps, 1);

	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

		LOG_INF("E preview normal\n");
		spin_lock(&imgsensor_drv_lock);
		imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
		imgsensor.pclk = imgsensor_info.normal_video.pclk;
		imgsensor.line_length = imgsensor_info.normal_video.linelength;
		imgsensor.frame_length = imgsensor_info.normal_video.framelength;
		imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
		spin_unlock(&imgsensor_drv_lock);
		normal_video_setting();

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	/* imgsensor.video_mode = KAL_TRUE; */
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	/* imgsensor.current_fps = 300; */
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();

	return ERROR_NONE;
}	/*	slim_video	 */

static kal_uint32 custom1(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM1;
		imgsensor.pclk = imgsensor_info.custom1.pclk;
		imgsensor.line_length = imgsensor_info.custom1.linelength;
		imgsensor.frame_length = imgsensor_info.custom1.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	/*PIP24fps_capture_setting(imgsensor.current_fps,imgsensor.pdaf_mode); */
	custom1_setting();
	return ERROR_NONE;
}

static kal_uint32 custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
		imgsensor.pclk = imgsensor_info.custom2.pclk;
		imgsensor.line_length = imgsensor_info.custom2.linelength;
		imgsensor.frame_length = imgsensor_info.custom2.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);

	if(custom2_flag == 0)
	{
		write_imx582_QSC_Data();
		custom2_flag++;
	}

	custom2_setting();
	return ERROR_NONE;
}

static kal_uint32 custom3(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM3;
		imgsensor.pclk = imgsensor_info.custom3.pclk;
		imgsensor.line_length = imgsensor_info.custom3.linelength;
		imgsensor.frame_length = imgsensor_info.custom3.framelength;
		imgsensor.min_frame_length = imgsensor_info.custom3.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom3_setting();
	return ERROR_NONE;
}

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;

	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	
	sensor_resolution->SensorCustom1Width = imgsensor_info.custom1.grabwindow_width;
	sensor_resolution->SensorCustom1Height = imgsensor_info.custom1.grabwindow_height;
	sensor_resolution->SensorCustom2Width = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height = imgsensor_info.custom2.grabwindow_height;
	sensor_resolution->SensorCustom3Width = imgsensor_info.custom3.grabwindow_width;
	sensor_resolution->SensorCustom3Height = imgsensor_info.custom3.grabwindow_height;


	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; /* inverse with datasheet */
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom1DelayFrame = imgsensor_info.custom1_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->Custom3DelayFrame = imgsensor_info.custom3_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	/* The frame of setting sensor gain */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->FrameTimeDelayFrame = imgsensor_info.frame_time_delay_frame; /* The delay frame of setting frame length  */
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	#if IMX582_PDAF_SWITCH
	sensor_info->PDAF_Support = 2;
	#else
	sensor_info->PDAF_Support = 0;
	#endif
	sensor_info->HDR_Support = 0;	/*0: NO HDR, 1: iHDR, 2:mvHDR, 3:zHDR, 4:four-cell mVHDR*/

	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */

	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;

		switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			sensor_info->SensorGrabStartX = imgsensor_info.custom1.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom1.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.custom1.mipi_data_lp2hs_settle_dc;
				
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			sensor_info->SensorGrabStartX = imgsensor_info.custom3.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom3.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.custom3.mipi_data_lp2hs_settle_dc;
			break;
		default:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
				imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}

	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		custom1(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		custom2(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		custom3(image_window, sensor_config_data);
		break;
	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) /* enable auto flicker */
		imgsensor.autoflicker_en = KAL_TRUE;
	else /* Cancel Auto flick */
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d, hdr_mode = %d\n", scenario_id, framerate, imgsensor.hdr_mode);

		switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:

			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
			(frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if (framerate == 0)
				return ERROR_NONE;

			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 /
				imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ?
				(frame_length - imgsensor_info.normal_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:

			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ?
				(frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			
			set_dummy();
	    	break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 /
				imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ?
				(frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 /
				imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ?
				(frame_length - imgsensor_info.slim_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;			
		case MSDK_SCENARIO_ID_CUSTOM1:
			frame_length = imgsensor_info.custom1.pclk / framerate * 10 / 
				imgsensor_info.custom1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom1.framelength) ? 
				(frame_length - imgsensor_info.custom1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			frame_length = imgsensor_info.custom2.pclk / framerate * 10 / 
				imgsensor_info.custom2.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? 
				(frame_length - imgsensor_info.custom2.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			frame_length = imgsensor_info.custom3.pclk / framerate * 10 / 
				imgsensor_info.custom3.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.custom3.framelength) ? 
				(frame_length - imgsensor_info.custom3.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.custom3.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			break;
		
		default:  /* coding with  preview scenario by default */
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ?
				(frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario\n", scenario_id);
			break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		write_cmos_sensor(0x0600, 0x0002);
	} else {
		/* 0x5E00[8]: 1 enable,  0 disable */
		/* 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK */
		write_cmos_sensor(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static void hdr_write_tri_shutter(kal_uint16 le, kal_uint16 me, kal_uint16 se)
{
	kal_uint16 realtime_fps = 0;

	LOG_INF("E! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);
	spin_lock(&imgsensor_drv_lock);
	if (le > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = le + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	if (le < imgsensor_info.min_shutter)
		le = imgsensor_info.min_shutter;

	if (imgsensor.autoflicker_en) {
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else {
			write_cmos_sensor_8(0x0104, 0x01);
			write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8); /*FRM_LENGTH_LINES[15:8]*/
			write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF); /*FRM_LENGTH_LINES[7:0]*/
			write_cmos_sensor_8(0x0104, 0x00);
		}
	} else {
		write_cmos_sensor_8(0x0104, 0x01);
		write_cmos_sensor_8(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor_8(0x0341, imgsensor.frame_length & 0xFF);
		write_cmos_sensor_8(0x0104, 0x00);
	}

	write_cmos_sensor_8(0x0104, 0x01);
	/* Long exposure */
	write_cmos_sensor_8(0x0202, (le >> 8) & 0xFF);
	write_cmos_sensor_8(0x0203, le & 0xFF);
	/* Muddle exposure */
	write_cmos_sensor_8(0x3FE0, (me >> 8) & 0xFF); /*MID_COARSE_INTEG_TIME[15:8]*/
	write_cmos_sensor_8(0x3FE1, me & 0xFF); /*MID_COARSE_INTEG_TIME[7:0]*/
	/* Short exposure */
	write_cmos_sensor_8(0x0224, (se >> 8) & 0xFF);
	write_cmos_sensor_8(0x0225, se & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	LOG_INF("L! le:0x%x, me:0x%x, se:0x%x\n", le, me, se);

}

static void hdr_write_tri_gain(kal_uint16 lg, kal_uint16 mg, kal_uint16 sg)
{
	kal_uint16 reg_lg, reg_mg, reg_sg;

	if (lg < BASEGAIN || lg > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");

		if (lg < BASEGAIN)
			lg = BASEGAIN;
		else if (lg > 16 * BASEGAIN)
			lg = 16 * BASEGAIN;
	}

	reg_lg = gain2reg(lg);
	reg_mg = gain2reg(mg);
	reg_sg = gain2reg(sg);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_lg;
	spin_unlock(&imgsensor_drv_lock);
	write_cmos_sensor_8(0x0104, 0x01);
	/* Long Gian */
	write_cmos_sensor_8(0x0204, (reg_lg>>8) & 0xFF);
	write_cmos_sensor_8(0x0205, reg_lg & 0xFF);
	/* Middle Gian */
	write_cmos_sensor_8(0x3FE2, (reg_mg>>8) & 0xFF);
	write_cmos_sensor_8(0x3FE3, reg_mg & 0xFF);
	/* Short Gian */
	write_cmos_sensor_8(0x0216, (reg_sg>>8) & 0xFF);
	write_cmos_sensor_8(0x0217, reg_sg & 0xFF);
	write_cmos_sensor_8(0x0104, 0x00);

	if (lg > mg) {
		LOG_INF("long gain > medium gain\n");
		write_cmos_sensor_8(0xEB06, 0x00);
		write_cmos_sensor_8(0xEB08, 0x00);
		write_cmos_sensor_8(0xEB0A, 0x00);
		write_cmos_sensor_8(0xEB12, 0x00);
		write_cmos_sensor_8(0xEB14, 0x00);
		write_cmos_sensor_8(0xEB16, 0x00);

		write_cmos_sensor_8(0xEB07, 0x08);
		write_cmos_sensor_8(0xEB09, 0x08);
		write_cmos_sensor_8(0xEB0B, 0x08);
		write_cmos_sensor_8(0xEB13, 0x10);
		write_cmos_sensor_8(0xEB15, 0x10);
		write_cmos_sensor_8(0xEB17, 0x10);
	} else {
		LOG_INF("long gain <= medium gain\n");
		write_cmos_sensor_8(0xEB06, 0x00);
		write_cmos_sensor_8(0xEB08, 0x00);
		write_cmos_sensor_8(0xEB0A, 0x00);
		write_cmos_sensor_8(0xEB12, 0x01);
		write_cmos_sensor_8(0xEB14, 0x01);
		write_cmos_sensor_8(0xEB16, 0x01);

		write_cmos_sensor_8(0xEB07, 0xC8);
		write_cmos_sensor_8(0xEB09, 0xC8);
		write_cmos_sensor_8(0xEB0B, 0xC8);
		write_cmos_sensor_8(0xEB13, 0x2C);
		write_cmos_sensor_8(0xEB15, 0x2C);
		write_cmos_sensor_8(0xEB17, 0x2C);
	}

	LOG_INF("lg:0x%x, mg:0x%x, sg:0x%x, reg_lg:0x%x, reg_mg:0x%x, reg_sg:0x%x\n",
			lg, mg, sg, reg_lg, reg_mg, reg_sg);

}

static void imx582_set_lsc_reg_setting(kal_uint8 index, kal_uint16 *regDa, MUINT32 regNum)
{

	LOG_INF("E\n");

}

static void set_imx582_ATR(kal_uint16 LimitGain, kal_uint16 LtcRate, kal_uint16 PostGain)
{


}

static kal_uint32 imx582_awb_gain(struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	kal_uint32 tempGR = pSetSensorAWB->ABS_GAIN_GR >> 1;
	kal_uint32 tempR  = pSetSensorAWB->ABS_GAIN_R  >> 1;
	kal_uint32 tempB  = pSetSensorAWB->ABS_GAIN_B  >> 1;
	kal_uint32 tempGB = pSetSensorAWB->ABS_GAIN_GB >> 1;
	
	write_cmos_sensor_8(0x0B8E, tempGR >> 8);	//Gr_H
	write_cmos_sensor_8(0x0B8F, tempGR & 0xff);	//Gr_L
	
	write_cmos_sensor_8(0x0B90, tempR >> 8);	//R_H
	write_cmos_sensor_8(0x0B91, tempR & 0xff);	//R_L
	
	write_cmos_sensor_8(0x0B92, tempB >> 8);	//B_H
	write_cmos_sensor_8(0x0B93, tempB & 0xff);	//B_L
	
	write_cmos_sensor_8(0x0B94, tempGB >> 8);	//Gb_H
	write_cmos_sensor_8(0x0B95, tempGB & 0xff);	//Gb_L
	
	return ERROR_NONE;
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
		write_cmos_sensor_8(0x0100, 0x01);
	else
		write_cmos_sensor_8(0x0100, 0x00);

	mdelay(10);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;

	//struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB = (struct SET_SENSOR_AWB_GAIN *) feature_para;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data = (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	LOG_INF("feature_id = %d\n", feature_id);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(ana_gain_table_8x)/sizeof(char);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)ana_gain_table_8x,
			sizeof(ana_gain_table_8x)/sizeof(char));
		}
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		imx582_awb_gain(pSetSensorAWB);
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n",
			imgsensor.pclk, imgsensor.current_fps);
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter(*feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
	     /* night_mode((BOOL) *feature_data); */
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/* get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE */
		/* if EEPROM does not exist in camera module. */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL)*feature_data_16, *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
		(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_GET_PDAF_DATA:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_DATA\n");
		/* read_3P3_eeprom((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),
		 *(kal_uint32)(*(feature_data+2)));
		 */
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:/*lzl*/
		set_shutter_frame_length((UINT16)*feature_data, (UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: /* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_INF("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		/* LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data); */
		wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[3],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			//printk("hct-drv add wininfo=%d %d %d %d imgsensor_winsize_info[3]=%d %d %d %d\n", wininfo->w1_size, wininfo->h1_size, wininfo->w2_tg_size, wininfo->h2_tg_size, imgsensor_winsize_info[3].w1_size, imgsensor_winsize_info[3].h1_size, imgsensor_winsize_info[3].w2_tg_size, imgsensor_winsize_info[3].h2_tg_size);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[4],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[5],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[6],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[7],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)wininfo, (void *)&imgsensor_winsize_info[0],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	/*HDR CMD */
	case SENSOR_FEATURE_SET_HDR_ATR:
		LOG_INF("SENSOR_FEATURE_SET_HDR_ATR Limit_Gain=%d, LTC Rate=%d, Post_Gain=%d\n",
				(UINT16)*feature_data,
				(UINT16)*(feature_data + 1),
				(UINT16)*(feature_data + 2));
		set_imx582_ATR((UINT16)*feature_data,
					(UINT16)*(feature_data + 1),
					(UINT16)*(feature_data + 2));
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_INF("hdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.hdr_mode = (UINT8)*feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d, no support\n",
			(UINT16) *feature_data,	(UINT16) *(feature_data + 1));
		/*hdr_write_shutter((UINT16) *feature_data, (UINT16) *(feature_data + 1),
		*	(UINT16) *(feature_data + 2));
		*/
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_SHUTTER:
		LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_SHUTTER LE=%d, ME=%d, SE=%d\n",
				(UINT16) *feature_data,
				(UINT16) *(feature_data + 1),
				(UINT16) *(feature_data + 2));
		hdr_write_tri_shutter((UINT16)*feature_data,
							(UINT16)*(feature_data+1),
							(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_HDR_TRI_GAIN:
		LOG_INF("SENSOR_FEATURE_SET_HDR_TRI_GAIN LGain=%d, SGain=%d, MGain=%d\n",
				(UINT16) *feature_data,
				(UINT16) *(feature_data + 1),
				(UINT16) *(feature_data + 2));
		hdr_write_tri_gain((UINT16)*feature_data, (UINT16)*(feature_data+1), (UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n", (UINT16) *feature_data);
		pvcinfo = (struct SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				   sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
				   sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:		
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				   sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_SET_LSC_TBL:
		{
			kal_uint8 index = *(((kal_uint8 *)feature_para) + (*feature_para_len));

			imx582_set_lsc_reg_setting(index, feature_data_16, (*feature_para_len)/sizeof(UINT16));
		}
		break;
	case SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY:
		/*
		  * SENSOR_VHDR_MODE_NONE  = 0x0,
		  * SENSOR_VHDR_MODE_IVHDR = 0x01,
		  * SENSOR_VHDR_MODE_MVHDR = 0x02,
		  * SENSOR_VHDR_MODE_ZVHDR = 0x09
		  * SENSOR_VHDR_MODE_4CELL_MVHDR = 0x0A
		*/
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x2;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_CUSTOM1:
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_CUSTOM3:
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0x0;
			break;
		}
		LOG_INF("SENSOR_FEATURE_GET_SENSOR_HDR_CAPACITY scenarioId:%llu, HDR:%llu\n"
			, *feature_data, *(feature_data+1));
		break;
		/*END OF HDR CMD */
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		kal_uint32 rate;

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			rate = imgsensor_info.cap.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			rate = imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			rate = imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			rate = imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			rate = imgsensor_info.pre.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM1:
			rate = imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			rate = imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case MSDK_SCENARIO_ID_CUSTOM3:
			rate = imgsensor_info.custom3.mipi_pixel_rate;
			break;
		default:
			rate = 0;
			break;
		}
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
	}
	break;
#if 0
	case SENSOR_FEATURE_GET_PDAF_INFO:
		LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
			(UINT16) *feature_data);
		PDAFinfo =
		  (struct SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
				 sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo,
				(void *)&imgsensor_pd_info_video,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:

		default:
			break;
		}
	break;
#endif
		case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
			/* LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", *feature_data); */
			/* PDAF capacity enable or not, 2p8 only full size support PDAF */
			switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				/* video & capture use same setting */
			*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			case MSDK_SCENARIO_ID_CUSTOM3:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
				break;
			}
			break;
		case SENSOR_FEATURE_SET_PDAF:
			LOG_INF("PDAF mode :%d\n", *feature_data_16);
			imgsensor.pdaf_mode= *feature_data_16;
			break;
		case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
			LOG_INF("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
			streaming_control(KAL_FALSE);
			break;
		case SENSOR_FEATURE_SET_STREAMING_RESUME:
			LOG_INF("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n", *feature_data);
			if (*feature_data != 0)
				set_shutter(*feature_data);
			streaming_control(KAL_TRUE);
			break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		*(feature_data + 2) = imgsensor_info.exp_step;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.cap.pclk;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.normal_video.pclk;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.hs_video.pclk;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.custom1.pclk;
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.custom2.pclk;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.slim_video.pclk;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				    = imgsensor_info.pre.pclk;
		    break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.cap.framelength << 16)
				    + imgsensor_info.cap.linelength;
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.normal_video.framelength << 16)
				    + imgsensor_info.normal_video.linelength;
				break;
			case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.hs_video.framelength << 16)
				    + imgsensor_info.hs_video.linelength;
				break;
			case MSDK_SCENARIO_ID_SLIM_VIDEO:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.slim_video.framelength << 16)
				    + imgsensor_info.slim_video.linelength;
				break;
			case MSDK_SCENARIO_ID_CUSTOM1:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.custom1.framelength << 16)
				    + imgsensor_info.custom1.linelength;
				break;
			case MSDK_SCENARIO_ID_CUSTOM2:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.custom2.framelength << 16)
				    + imgsensor_info.custom2.linelength;
				break;
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			default:
				*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= (imgsensor_info.pre.framelength << 16)
				    + imgsensor_info.pre.linelength;
				break;
		}
		break;
	default:
		break;
	}

	return ERROR_NONE;
}	/*	feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 IMX582_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* AIWORKS[20190722][start] stereo cam */
	#if AIWORKS_SUPPORT
	sema_init(&aidualcam_thread_sem, 1);
	#endif
    /* AIWORKS [20190722][end] stereo cam */

	if (pfFunc != NULL)
		*pfFunc =  &sensor_func;
	return ERROR_NONE;
}	

/* AIWORKS[20190722][start] stereo cam */
#if AIWORKS_SUPPORT
static int aidualcam_thread_loop(void *arg) {
	//kal_uint8 ret1;
	kal_uint16 start_addr = 0x12a0;
	MUINT16 data_size = 1112+1;
	char pu_send_cmd[2] = {(char)(start_addr >> 8), (char)(start_addr & 0xFF) };
	int sys_ret;
	int i=0;
	u8 *pdatabuff=NULL;
	
	if (1) {
		pdatabuff=(u8 *)kmalloc(data_size, GFP_KERNEL);
		if (NULL == pdatabuff) {
			LOG_INF("[xapi]: No mem \n");
			return -1;
		}
		memset(pdatabuff, 0, data_size);

		sys_ret = down_interruptible(&aidualcam_thread_sem);
		if (0 == sys_ret) {
			if (aidualcam_poweron) {
				LOG_INF("sensor is on, to read 3d calibration data\n");
				iReadRegI2C(pu_send_cmd, 2, pdatabuff, data_size, EEPROM_WRITE_ID);
				LOG_INF("sensor is on, to read 3d calibration data end");
				for(i = 0;i<5;i++)
					LOG_INF("pdatabuff = %d,i = %d\n", pdatabuff[i], i);

				aidualcam_cal_init(pdatabuff, data_size);
			}
		up(&aidualcam_thread_sem);
		}
		kfree(pdatabuff);
	}
	return 0;
}
#endif
/* AIWORKS[20190722][end] stereo cam */
