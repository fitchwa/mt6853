/*****************************************************************************
 *
 * Filename:
 * ---------
 *     GC5024mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
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
#include <asm/atomic.h>
#include <linux/types.h>
#include "kd_camera_typedef.h"


//#include <asm/system.h>
//#include <linux/xlog.h>

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc032a_mipi_yuv.h"
#include <linux/hct_include/hct_project_all_config.h>

/****************************Modify Following Strings for Debug****************************/
#define PFX "GC032a_mipi_yuvr"
#define LOG_1 LOG_INF("GC032a, MAIN FAKE\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#define GC5024_MIPI_USE_OTP

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static kal_bool GC5024DuringTestPattern = KAL_FALSE;

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = GC032A_SENSOR_ID,        //record sensor id defined in Kd_imgsensor.h

    .checksum_value = 0x596769eb,        //checksum value for Camera Auto Test

    .pre = {
        .pclk = 45000000,                //record different mode's pclk
        .linelength = 1000,                //record different mode's linelength
        .framelength = 1984,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2592,        //record different mode's width of grabwindow
        .grabwindow_height = 1944,        //record different mode's height of grabwindow
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        /*     following for GetDefaultFramerateByScenario()    */
        .max_framerate = 100,
    },
    .cap = {
        .pclk = 45000000,
        .linelength = 1000,
        .framelength = 1984,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2592,
        .grabwindow_height = 1944,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 100,
    },
    .cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 45000000,
        .linelength = 1000,
        .framelength = 1984,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2592,
        .grabwindow_height = 1944,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 100,
    },
    .normal_video = {
          .pclk = 45000000,
        .linelength = 1000,
        .framelength = 1480,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2560,
        .grabwindow_height = 1440,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 100,
    },
    .hs_video = {
          .pclk = 45000000,
        .linelength = 1000,
        .framelength = 1480,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2560,
        .grabwindow_height = 1440,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 100,
    },
    .slim_video = {
        .pclk = 45000000,
        .linelength = 1000,
        .framelength = 1480,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2560,
        .grabwindow_height = 1440,
        .mipi_data_lp2hs_settle_dc = 85,
        .max_framerate = 100,
    },
    .margin = 0,            //sensor framelength & shutter margin
    .min_shutter = 1,        //min shutter
    .max_frame_length = 0x3fff,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 3,      //support sensor mode num

    .cap_delay_frame = 2,        //enter capture delay frame num
    .pre_delay_frame = 2,         //enter preview delay frame num
    .video_delay_frame = 2,        //enter video delay frame num
    .hs_video_delay_frame = 2,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 2,//enter slim video delay frame num

    .isp_driving_current = ISP_DRIVING_6MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
    .i2c_addr_table = {0x42, 0x43, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3ED,                    //current shutter
    .gain = 0x40,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x42,//record current sensor's i2c write id
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{
	{ 2592, 1944,	 0,    0, 2592, 1944, 2592,  1944, 0000, 0000, 2592,  1944, 	 0,    0, 2592,  1944}, // Preview
	{ 2592, 1944,	 0,    0, 2592, 1944, 2592,  1944, 0000, 0000, 2592,  1944, 	 0,    0, 2592,  1944}, // capture
	{ 2560, 1440,	 0,    0, 2560, 1440, 2560,  1440, 0000, 0000, 2560,  1440, 	 0,    0, 2560,  1440}, // video
	{ 2560, 1440,	 0,    0, 2560, 1440, 2560,  1440, 0000, 0000, 2560,  1440, 	 0,    0, 2560,  1440}, //hight speed video
	{ 2560, 1440,	 0,    0, 2560, 1440, 2560,  1440, 0000, 0000, 2560,  1440, 	 0,    0, 2560,  1440}};// slim video



static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[1] = {(char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 1, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;

}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
#if 1
		char pu_send_cmd[2] = {(char)(addr & 0xFF), (char)(para & 0xFF)};
		iWriteRegI2C(pu_send_cmd, 2, imgsensor.i2c_write_id);
#else
		iWriteReg((u16)addr, (u32)para, 2, imgsensor.i2c_write_id);
#endif

}

#ifdef GC5024_MIPI_USE_OTP

#define RG_TYPICAL    				0x0090
#define BG_TYPICAL				0x0095
#define INFO_ROM_START			0x08
#define INFO_WIDTH       			0x07
#define WB_ROM_START           		0x78
#define WB_WIDTH         			0x02

typedef struct otp_gc5024{
	kal_uint16 module_id;
	kal_uint16 lens_id;
	kal_uint16 vcm_id;
	kal_uint16 vcm_driver_id;
	kal_uint16 year;
	kal_uint16 month;
	kal_uint16 day;
	kal_uint16 rg_gain;
	kal_uint16 bg_gain;
	kal_uint16 wb_flag;// 0:Empty 1:Success 2:Invalid
}gc5024_otp;

static gc5024_otp gc5024_otp_info;

typedef enum{
	otp_close=0,
	otp_open,
}otp_state;

static kal_uint8 gc5024_read_otp(kal_uint8 addr)
{
	kal_uint8 value;

	write_cmos_sensor(0xfe,0x00);
	write_cmos_sensor(0xd5,addr);
	write_cmos_sensor(0xf3,0x20);
	value = read_cmos_sensor(0xd7);

	return value;
}

static void gc5024_gcore_read_otp_info(void)
{
	kal_uint8 flag;
	kal_uint8 index;
	kal_uint8 info_start_add,wb_start_add;
	memset(&gc5024_otp_info,0,sizeof(gc5024_otp));

	flag = gc5024_read_otp(0x00);
	LOG_INF("GC5024_OTP : flag = 0x%x\n",flag);

//INFO&WB
	for(index=0;index<2;index++)
	{
		switch((flag>>(4 + 2 * index))&0x03)
		{
		case 0x00:
			LOG_INF("GC5024_OTP_INFO group%d is Empty !!\n", index + 1);
			break;
		case 0x01:
			info_start_add = INFO_ROM_START + index * INFO_WIDTH*8;
			gc5024_otp_info.module_id = gc5024_read_otp(info_start_add);
			gc5024_otp_info.lens_id = gc5024_read_otp(info_start_add + 8*1);
			gc5024_otp_info.vcm_driver_id = gc5024_read_otp(info_start_add + 8*2);
			gc5024_otp_info.vcm_id = gc5024_read_otp(info_start_add + 8*3);
			gc5024_otp_info.year = gc5024_read_otp(info_start_add + 8*4);
			gc5024_otp_info.month = gc5024_read_otp(info_start_add + 8*5);
			gc5024_otp_info.day = gc5024_read_otp(info_start_add + 8*6);
			break;
		case 0x02:
		case 0x03:
			LOG_INF("GC5024_OTP_INFO group%d is Invalid !!\n", index + 1);
			break;
		default :
			break;
		}

		switch((flag>>(2 * index))&0x03)
		{
		case 0x00:
			LOG_INF("GC5024_OTP_WB group%d is Empty !!\n", index + 1);
			gc5024_otp_info.wb_flag = gc5024_otp_info.wb_flag|0x00;
			break;
		case 0x01:
			LOG_INF("GC5024_OTP_WB group%d is Valid !!\n", index + 1);
			wb_start_add = WB_ROM_START + index * WB_WIDTH*8;
			gc5024_otp_info.rg_gain = gc5024_read_otp(wb_start_add);
			gc5024_otp_info.bg_gain = gc5024_read_otp(wb_start_add + 1*8);

			if((0==gc5024_otp_info.rg_gain)||(0==gc5024_otp_info.bg_gain))
			{
				gc5024_otp_info.wb_flag = gc5024_otp_info.wb_flag|0x02;
				LOG_INF("GC5024_OTP_WB group%d is Error ,wb rg or bg = 0!!\n", index + 1);
			}
			else
			{
				gc5024_otp_info.wb_flag = gc5024_otp_info.wb_flag|0x01;
			}

			break;
		case 0x02:
		case 0x03:
			LOG_INF("GC5024_OTP_WB group%d is Invalid !!\n", index + 1);
			gc5024_otp_info.wb_flag = gc5024_otp_info.wb_flag|0x02;
			break;
		default :
			break;
		}
	}

	/*print otp information*/
	LOG_INF("GC5024_OTP_INFO:module_id=0x%x\n",gc5024_otp_info.module_id);
	LOG_INF("GC5024_OTP_INFO:lens_id=0x%x\n",gc5024_otp_info.lens_id);
	LOG_INF("GC5024_OTP_INFO:vcm_id=0x%x\n",gc5024_otp_info.vcm_id);
	LOG_INF("GC5024_OTP_INFO:vcm_driver_id=0x%x\n",gc5024_otp_info.vcm_driver_id);
	LOG_INF("GC5024_OTP_INFO:data=%d-%d-%d\n",gc5024_otp_info.year,gc5024_otp_info.month,gc5024_otp_info.day);
	LOG_INF("GC5024_OTP_WB:r/g=0x%x\n",gc5024_otp_info.rg_gain);
	LOG_INF("GC5024_OTP_WB:b/g=0x%x\n",gc5024_otp_info.bg_gain);

}

static void gc5024_gcore_update_awb(void)
{
	kal_uint16 r_gain_current = 0 , g_gain_current = 0 , b_gain_current = 0 , base_gain = 0;
	kal_uint16 r_gain = 128 , g_gain = 128 , b_gain = 128 ;
	kal_uint16 rg_typical,bg_typical;

	rg_typical=RG_TYPICAL;
	bg_typical=BG_TYPICAL;

	if(0x01==(gc5024_otp_info.wb_flag&0x01))
	{
		r_gain_current = 256 * rg_typical/gc5024_otp_info.rg_gain;
		b_gain_current = 256 * bg_typical/gc5024_otp_info.bg_gain;
		g_gain_current = 256;

		base_gain = (r_gain_current<b_gain_current) ? r_gain_current : b_gain_current;
		base_gain = (base_gain<g_gain_current) ? base_gain : g_gain_current;
		LOG_INF("GC5024_OTP_UPDATE_AWB:r_gain_current = 0x%x , b_gain_current = 0x%x , base_gain = 0x%x \n",r_gain_current,b_gain_current,base_gain);

		r_gain = 0x80 * r_gain_current / base_gain;
		g_gain = 0x80 * g_gain_current / base_gain;
		b_gain = 0x80 * b_gain_current / base_gain;
		LOG_INF("GC5024_OTP_UPDATE_AWB:r_gain = 0x%x , g_gain = 0x%x , b_gain = 0x%x \n",r_gain,g_gain,b_gain);

		/*TODO*/
		write_cmos_sensor(0xfe,0x00);
		write_cmos_sensor(0xb8,g_gain);
		write_cmos_sensor(0xb9,g_gain);
		write_cmos_sensor(0xba,r_gain);
		write_cmos_sensor(0xbb,r_gain);
		write_cmos_sensor(0xbc,b_gain);
		write_cmos_sensor(0xbd,b_gain);
		write_cmos_sensor(0xbe,g_gain);
		write_cmos_sensor(0xbf,g_gain);
	}

}

static void gc5024_gcore_update_otp(void)
{
	gc5024_gcore_update_awb();
}

static void gc5024_gcore_enable_otp(otp_state state)
{
	kal_uint8 otp_clk,otp_en;
	otp_clk = read_cmos_sensor(0xfa);
	otp_en= read_cmos_sensor(0xd4);
	if(state)
	{
		otp_clk = otp_clk | 0x10;
		otp_en = otp_en | 0x80;
		mdelay(5);
		write_cmos_sensor(0xfa,otp_clk);	// 0xfa[6]:OTP_CLK_en
		write_cmos_sensor(0xd4,otp_en);	// 0xd4[7]:OTP_en

		LOG_INF("GC5024_OTP: Enable OTP!\n");
	}
	else
	{
		otp_en = otp_en & 0x7f;
		otp_clk = otp_clk & 0xef;
		mdelay(5);
		write_cmos_sensor(0xd4,otp_en);
		write_cmos_sensor(0xfa,otp_clk);

		LOG_INF("GC5024_OTP: Disable OTP!\n");
	}

}

static void gc5024_gcore_identify_otp(void)
{
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xf8, 0x0e);
	write_cmos_sensor(0xf9, 0xae);
	write_cmos_sensor(0xfa, 0x84);
	write_cmos_sensor(0xfc, 0xae);
	gc5024_gcore_enable_otp(otp_open);
	gc5024_gcore_read_otp_info();
	gc5024_gcore_enable_otp(otp_close);
}

#endif


static void set_dummy(void)
{

//  end
}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
    return ((read_cmos_sensor(0xf0) << 8) | read_cmos_sensor(0xf1));
}

static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    //kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();
}    /*    set_max_framerate  */



/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
	{
	kal_uint16 realtime_fps = 0;

	 unsigned long flags;
	//    kal_uint16 realtime_fps = 0;
  	//  kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

			/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
			/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

			// OV Recommend Solution
			// if shutter bigger than frame_length, should extend frame length first
			spin_lock(&imgsensor_drv_lock);
			if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
				imgsensor.frame_length = shutter + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;

			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
			shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

			if (imgsensor.autoflicker_en) {
				realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
}
	// Update Shutter
	if(shutter > 16383) shutter = 16383;
	if(shutter < 1) shutter = 1;
	if(shutter <= 20)
  	{
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x32, 0x09);
		write_cmos_sensor(0xb0, 0x53);
  	}
  	else
	{
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x32, 0x49);
		write_cmos_sensor(0xb0, 0x4b);
  	}
	//Update Shutter
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x03, (shutter>>8) & 0x3F);
	write_cmos_sensor(0x04, shutter & 0xFF);

    LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    /*    set_shutter */


/*
static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0000;

    reg_gain = ((gain / BASEGAIN) << 4) + ((gain % BASEGAIN) * 16 / BASEGAIN);
    reg_gain = reg_gain & 0xFFFF;
    return (kal_uint16)reg_gain;
}*/

/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/

#define ANALOG_GAIN_1 64   // 1.000x
#define ANALOG_GAIN_2 88   // 1.375x
#define ANALOG_GAIN_3 121  // 1.891x
#define ANALOG_GAIN_4 168  // 2.625x
#define ANALOG_GAIN_5 239  // 3.734x
#define ANALOG_GAIN_6 336  // 5.250x
#define ANALOG_GAIN_7 481  // 7.516x

static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 iReg,temp;
	kal_uint16 reg_gain;
	reg_gain = gain>>1;	//zengyuan
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	iReg = gain;

	if(iReg < 0x40)
		iReg = 0x40;
	else if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_2))
	{
#if 0
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x21, 0x0f);
		write_cmos_sensor(0x29, 0x0f);
		write_cmos_sensor(0xe8, 0x03);
  		write_cmos_sensor(0xe9, 0x01);
  		write_cmos_sensor(0xea, 0x02);
  		write_cmos_sensor(0xeb, 0x02);
  		write_cmos_sensor(0xec, 0x03);
  		write_cmos_sensor(0xed, 0x01);
  		write_cmos_sensor(0xee, 0x02);
  		write_cmos_sensor(0xef, 0x02);
#endif
		//analog gain
		write_cmos_sensor(0xb6,  0x00);
		temp = iReg;
		write_cmos_sensor(0xb1, temp>>6);
		write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		LOG_INF("GC5024MIPI analogic gain 1x, GC5024MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_2<= iReg)&&(iReg < ANALOG_GAIN_3))
	{
#if 0
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x21, 0x0f);
		write_cmos_sensor(0x29, 0x0f);
		write_cmos_sensor(0xe8, 0x03);
		write_cmos_sensor(0xe9, 0x01);
 		write_cmos_sensor(0xea, 0x02);
 		write_cmos_sensor(0xeb, 0x02);
		write_cmos_sensor(0xec, 0x03);
  		write_cmos_sensor(0xed, 0x01);
  		write_cmos_sensor(0xee, 0x02);
  		write_cmos_sensor(0xef, 0x02);
#endif
		//analog gain
		write_cmos_sensor(0xb6,  0x01);//
		temp = 64*iReg/ANALOG_GAIN_2;
		write_cmos_sensor(0xb1, temp>>6);
		write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		LOG_INF("GC5024MIPI analogic gain 1.375x , GC5024MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_3<= iReg)&&(iReg < ANALOG_GAIN_4))
	{
#if 0
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x21, 0x0b);
		write_cmos_sensor(0x29, 0x1b);
		write_cmos_sensor(0xe8, 0x03);
 		write_cmos_sensor(0xe9, 0x01);
 		write_cmos_sensor(0xea, 0x02);
 		write_cmos_sensor(0xeb, 0x02);
 		write_cmos_sensor(0xec, 0x03);
 		write_cmos_sensor(0xed, 0x01);
 		write_cmos_sensor(0xee, 0x02);
 		write_cmos_sensor(0xef, 0x02);
#endif
		//analog gain
		write_cmos_sensor(0xb6,  0x02);//
		temp = 64*iReg/ANALOG_GAIN_3;
		write_cmos_sensor(0xb1, temp>>6);
		write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		LOG_INF("GC5024MIPI analogic gain 1.891x , GC5024MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_4<= iReg)&&(iReg < ANALOG_GAIN_5))
	{
#if 0
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x21, 0x0d);
		write_cmos_sensor(0x29, 0x1d);
		write_cmos_sensor(0xe8, 0x02);
		write_cmos_sensor(0xe9, 0x02);
		write_cmos_sensor(0xea, 0x02);
		write_cmos_sensor(0xeb, 0x02);
		write_cmos_sensor(0xec, 0x02);
		write_cmos_sensor(0xed, 0x02);
		write_cmos_sensor(0xee, 0x02);
		write_cmos_sensor(0xef, 0x02);
#endif
		//analog gain
		write_cmos_sensor(0xb6,  0x03);//
		temp = 64*iReg/ANALOG_GAIN_4;
		write_cmos_sensor(0xb1, temp>>6);
		write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		LOG_INF("GC5024MIPI analogic gain 2.625x , GC5024MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_5<= iReg)&&(iReg < ANALOG_GAIN_6))
	{
#if 0
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x21, 0x08);
		write_cmos_sensor(0x29, 0x38);
		write_cmos_sensor(0xe8, 0x03);
		write_cmos_sensor(0xe9, 0x03);
		write_cmos_sensor(0xea, 0x03);
		write_cmos_sensor(0xeb, 0x03);
		write_cmos_sensor(0xec, 0x03);
		write_cmos_sensor(0xed, 0x03);
		write_cmos_sensor(0xee, 0x03);
		write_cmos_sensor(0xef, 0x03);
#endif
		//analog gain
		write_cmos_sensor(0xb6,  0x04);//
		temp = 64*iReg/ANALOG_GAIN_5;
		write_cmos_sensor(0xb1, temp>>6);
		write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		LOG_INF("GC5024MIPI analogic gain 3.734x , GC5024MIPI add pregain = %d\n",temp);
	}
	else if((ANALOG_GAIN_6<= iReg)&&(iReg < ANALOG_GAIN_7))
	{
#if 0
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x21, 0x08);
		write_cmos_sensor(0x29, 0x38);
		write_cmos_sensor(0xe8, 0x03);
		write_cmos_sensor(0xe9, 0x03);
		write_cmos_sensor(0xea, 0x03);
		write_cmos_sensor(0xeb, 0x03);
		write_cmos_sensor(0xec, 0x03);
		write_cmos_sensor(0xed, 0x03);
		write_cmos_sensor(0xee, 0x03);
		write_cmos_sensor(0xef, 0x03);
#endif
		//analog gain
		write_cmos_sensor(0xb6,  0x05);//
		temp = 64*iReg/ANALOG_GAIN_6;
		write_cmos_sensor(0xb1, temp>>6);
		write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		LOG_INF("GC5024MIPI analogic gain 5.250x , GC5024MIPI add pregain = %d\n",temp);
	}
	else
	{
#if 0
		write_cmos_sensor(0xfe, 0x00);
		write_cmos_sensor(0x21, 0x08);
		write_cmos_sensor(0x29, 0x38);
		write_cmos_sensor(0xe8, 0x03);
		write_cmos_sensor(0xe9, 0x03);
		write_cmos_sensor(0xea, 0x03);
		write_cmos_sensor(0xeb, 0x03);
		write_cmos_sensor(0xec, 0x03);
		write_cmos_sensor(0xed, 0x03);
		write_cmos_sensor(0xee, 0x03);
		write_cmos_sensor(0xef, 0x03);
#endif
		//analog gain
		write_cmos_sensor(0xb6,  0x06);//
		temp = 64*iReg/ANALOG_GAIN_7;
		write_cmos_sensor(0xb1, temp>>6);
		write_cmos_sensor(0xb2, (temp<<2)&0xfc);
		LOG_INF("GC5024MIPI analogic gain 7.516x , GC5024MIPI add pregain = %d\n",temp);
	}
	return gain;

}    /*    set_gain  */

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);

}


/*
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

}
*/
/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}    /*    night_mode    */

static void sensor_init(void)
{
	LOG_INF("E");
#if 0
    /*SYS*/
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xf7, 0x01);
	write_cmos_sensor(0xf8, 0x0e);
	write_cmos_sensor(0xf9, 0xae);
	write_cmos_sensor(0xfa, 0x84);
	write_cmos_sensor(0xfc, 0xae);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x88, 0x03);
	write_cmos_sensor(0xe7, 0xc0);
	/*Analog*/
	write_cmos_sensor(0xfe, 0x00);
	write_cmos_sensor(0x03, 0x08);
	write_cmos_sensor(0x04, 0xca);
	write_cmos_sensor(0x05, 0x01);
	write_cmos_sensor(0x06, 0xf4);
	write_cmos_sensor(0x07, 0x00);
	write_cmos_sensor(0x08, 0x08);
	write_cmos_sensor(0x0a, 0x00);
	write_cmos_sensor(0x0c, 0x00);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x40);

	write_cmos_sensor(0x11, 0x31);
	write_cmos_sensor(0x12, 0x28);
	write_cmos_sensor(0x13, 0x10);
	write_cmos_sensor(0x17, MIRROR);
	write_cmos_sensor(0x18, 0x02);
	write_cmos_sensor(0x19, 0x0d);
	write_cmos_sensor(0x1a, PH_SWITCH);
	write_cmos_sensor(0x1b, 0x41);
	write_cmos_sensor(0x1c, 0x2b);
	write_cmos_sensor(0x21, 0x0f);
	write_cmos_sensor(0x24, 0xb0);
	write_cmos_sensor(0x29, 0x38);
	write_cmos_sensor(0x2d, 0x16);
	write_cmos_sensor(0x2f, 0x16);
	write_cmos_sensor(0x32, 0x49);

	write_cmos_sensor(0xcd, 0xaa);
	write_cmos_sensor(0xd0, 0xc2);
	write_cmos_sensor(0xd1, 0xc4);
	write_cmos_sensor(0xd2, 0xcb);
	write_cmos_sensor(0xd3, 0x73);
	write_cmos_sensor(0xd8, 0x18);
	write_cmos_sensor(0xdc, 0xba);
	write_cmos_sensor(0xe2, 0x20);
	write_cmos_sensor(0xe4, 0x78);
	write_cmos_sensor(0xe6, 0x08);

	/*ISP*/
	write_cmos_sensor(0x80, 0x50);
	write_cmos_sensor(0x8d, 0x07);
	write_cmos_sensor(0x90, 0x01);
	write_cmos_sensor(0x92, STARTY);
	write_cmos_sensor(0x94, STARTX);
	write_cmos_sensor(0x95, 0x07);
	write_cmos_sensor(0x96, 0x98);
	write_cmos_sensor(0x97, 0x0a);
	write_cmos_sensor(0x98, 0x20);

	/*Gain*/
	write_cmos_sensor(0x99, 0x01);
	write_cmos_sensor(0x9a, 0x02);
	write_cmos_sensor(0x9b, 0x03);
	write_cmos_sensor(0x9c, 0x04);
	write_cmos_sensor(0x9d, 0x0d);
	write_cmos_sensor(0x9e, 0x15);
	write_cmos_sensor(0x9f, 0x1d);
	write_cmos_sensor(0xb0, 0x4b);
	write_cmos_sensor(0xb1, 0x01);
	write_cmos_sensor(0xb2, 0x00);
	write_cmos_sensor(0xb6, 0x00);

	/*BLK*/
	write_cmos_sensor(0x40, 0x22);
	write_cmos_sensor(0x4e, 0x3c);
	write_cmos_sensor(0x4f, 0x00);
	write_cmos_sensor(0x60, 0x00);
	write_cmos_sensor(0x61, 0x80);
	write_cmos_sensor(0xfe, 0x02);
	write_cmos_sensor(0xa4, 0x30);
	write_cmos_sensor(0xa5, 0x00);

	/*Dark Sun*/
	write_cmos_sensor(0x40, 0x00); //96 travis20160527
	write_cmos_sensor(0x42, 0x0f);
	write_cmos_sensor(0x45, 0xca);
	write_cmos_sensor(0x47, 0xff);
	write_cmos_sensor(0x48, 0xc8);

	/*DD*/
	write_cmos_sensor(0x80, 0x98);
	write_cmos_sensor(0x81, 0x50);
	write_cmos_sensor(0x82, 0x60);
	write_cmos_sensor(0x84, 0x20);
	write_cmos_sensor(0x85, 0x10);
	write_cmos_sensor(0x86, 0x04);
	write_cmos_sensor(0x87, 0x20);
	write_cmos_sensor(0x88, 0x10);
	write_cmos_sensor(0x89, 0x04);

	/*Degrid*/
	write_cmos_sensor(0x8a, 0x0a);

	/*MIPI*/
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x01, 0x07);
	write_cmos_sensor(0x02, 0x34);
	write_cmos_sensor(0x03, 0x13);
	write_cmos_sensor(0x04, 0x04);
	write_cmos_sensor(0x05, 0x00);
	write_cmos_sensor(0x06, 0x80);
	write_cmos_sensor(0x11, 0x2b);
	write_cmos_sensor(0x12, 0xa8);
	write_cmos_sensor(0x13, 0x0c);
	write_cmos_sensor(0x15, 0x00);
	write_cmos_sensor(0x16, 0x09);
	write_cmos_sensor(0x18, 0x01);
	write_cmos_sensor(0x21, 0x10);
	write_cmos_sensor(0x22, 0x05);
	write_cmos_sensor(0x23, 0x30);
	write_cmos_sensor(0x24, 0x10);
	write_cmos_sensor(0x25, 0x14);
	write_cmos_sensor(0x26, 0x08);
	write_cmos_sensor(0x29, 0x05);
	write_cmos_sensor(0x2a, 0x0a);
	write_cmos_sensor(0x2b, 0x08);
	write_cmos_sensor(0x42, 0x20);
	write_cmos_sensor(0x43, 0x0a);
	write_cmos_sensor(0xfe, 0x00);
#endif
}    /*    sensor_init  */


static void preview_setting(void)
{
	LOG_INF("E!\n");
#if 0
	//2592x1944
	/*SYS*/
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x10, 0x00);//Stream off
	write_cmos_sensor(0xfe, 0x00);
	/*Analog*/
	write_cmos_sensor(0x0a, 0x00);
	write_cmos_sensor(0x0c, 0x00);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x40);
	/*ISP*/
	write_cmos_sensor(0x90, 0x01);
	write_cmos_sensor(0x95, 0x07);
	write_cmos_sensor(0x96, 0x98);
	write_cmos_sensor(0x97, 0x0a);
	write_cmos_sensor(0x98, 0x20);
	/*MIPI*/
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x12, 0xa8);
	write_cmos_sensor(0x13, 0x0c);
	write_cmos_sensor(0x42, 0x20);
	write_cmos_sensor(0x43, 0x0a);
	write_cmos_sensor(0x10, 0x91); //Stream on
	write_cmos_sensor(0xfe, 0x00);
#endif
}    /*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
#if 0
	//2592x1944
	/*SYS*/
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x10, 0x00);//Stream off
	write_cmos_sensor(0xfe, 0x00);
	/*Analog*/
	write_cmos_sensor(0x0a, 0x00);
	write_cmos_sensor(0x0c, 0x00);
	write_cmos_sensor(0x0d, 0x07);
	write_cmos_sensor(0x0e, 0xa8);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x40);
	/*ISP*/
	write_cmos_sensor(0x90, 0x01);
	write_cmos_sensor(0x95, 0x07);
	write_cmos_sensor(0x96, 0x98);
	write_cmos_sensor(0x97, 0x0a);
	write_cmos_sensor(0x98, 0x20);
	/*MIPI*/
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x12, 0xa8);
	write_cmos_sensor(0x13, 0x0c);
	write_cmos_sensor(0x42, 0x20);
	write_cmos_sensor(0x43, 0x0a);
	write_cmos_sensor(0x10, 0x91); //Stream on
	write_cmos_sensor(0xfe, 0x00);
#endif
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
#if 0
	//2560x1440
	/*SYS*/
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x10, 0x00); //Stream off
	write_cmos_sensor(0xfe, 0x00);
	/*Analog*/
	write_cmos_sensor(0x0a, 0xfc);
	write_cmos_sensor(0x0c, 0x10);
	write_cmos_sensor(0x0d, 0x05);
	write_cmos_sensor(0x0e, 0xb0);
	write_cmos_sensor(0x0f, 0x0a);
	write_cmos_sensor(0x10, 0x20);
	/*ISP*/
	write_cmos_sensor(0x90, 0x01);
	write_cmos_sensor(0x95, 0x05);
	write_cmos_sensor(0x96, 0xa0);
	write_cmos_sensor(0x97, 0x0a);
	write_cmos_sensor(0x98, 0x00);
	/*MIPI*/
	write_cmos_sensor(0xfe, 0x03);
	write_cmos_sensor(0x12, 0x80);
	write_cmos_sensor(0x13, 0x0c);
	write_cmos_sensor(0x42, 0x00);
	write_cmos_sensor(0x43, 0x0a);
	write_cmos_sensor(0x10, 0x91); //Stream on
	write_cmos_sensor(0xfe, 0x00);
#endif
}

static void hs_video_setting(void)
{
   	LOG_INF("E\n");
#if 0
	write_cmos_sensor(0xfe,0x03);
	write_cmos_sensor(0x10,0x91);
	write_cmos_sensor(0xfe,0x00);
#endif
}

static void slim_video_setting(void)
{
    LOG_INF("E\n");
#if 0
	write_cmos_sensor(0xfe,0x03);
	write_cmos_sensor(0x10,0x91);
	write_cmos_sensor(0xfe,0x00);
#endif
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);
#if 0
    if (enable) {
        write_cmos_sensor(0xfe, 0x00);
        write_cmos_sensor(0xb3, 0x40);
        write_cmos_sensor(0xb4, 0x40);
        write_cmos_sensor(0xb5, 0x40);
	write_cmos_sensor(0xb6, 0x00);
        write_cmos_sensor(0xb1, 0x01);
        write_cmos_sensor(0xb2, 0x00);
        write_cmos_sensor(0x03, 0x00);
        write_cmos_sensor(0x04, 0x00);
        write_cmos_sensor(0x8c, 0x11);
    } else {
        write_cmos_sensor(0xfe, 0x00);
        write_cmos_sensor(0x8c, 0x10);
    }

    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
#endif
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 3;

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
                return ERROR_NONE;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 3;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;

    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    /* GC5024 OTP */
#ifdef GC5024_MIPI_USE_OTP
    gc5024_gcore_identify_otp();
#endif

    /* initail sequence write in  */
    sensor_init();

    /*write registers from sram*/
#ifdef GC5024_MIPI_USE_OTP
    gc5024_gcore_update_otp();
#endif

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);
    GC5024DuringTestPattern = KAL_FALSE;

    return ERROR_NONE;
}    /*    open  */



/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");

    /*No Need to implement this function*/

    return ERROR_NONE;
}    /*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}    /*    preview   */

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
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
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
    return ERROR_NONE;
}    /* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);
    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
    return ERROR_NONE;
}    /*    slim_video     */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
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

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


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
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{//This Function not used after ROME
    LOG_INF("framerate = %d\n ", framerate);
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
        // Dynamic frame rate
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable) //enable auto flicker
        imgsensor.autoflicker_en = KAL_TRUE;
    else //Cancel Auto flick
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
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
        default:
            break;
    }

    return ERROR_NONE;
}



static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    //unsigned long long *feature_return_para=(unsigned long long *) feature_para;

   // struct SET_PD_BLOCK_INFO_T *PDAFinfo;
    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("zcy add feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
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
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
                     set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
                     get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func123 = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 GC032A_YUV_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func123;
    return ERROR_NONE;
}    /*    GC032a_YUV_SensorInit    */


/***********************************************************************/

#define GC032a_READ_ID								0x43
#define GC032a_WRITE_ID							  0x42


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

kal_uint16 GC032A_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
    char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};

	iWriteRegI2C(puSendCmd , 2, GC032a_WRITE_ID);

	return 0;
}


void GC032a_Sensor_Init(void)
{
	/*System*/
	GC032A_write_cmos_sensor(0xf3,0xff);
	GC032A_write_cmos_sensor(0xf5,0x06);
	GC032A_write_cmos_sensor(0xf7,0x01);
	GC032A_write_cmos_sensor(0xf8,0x03);
	GC032A_write_cmos_sensor(0xf9,0xce); 
	GC032A_write_cmos_sensor(0xfa,0x00);
	GC032A_write_cmos_sensor(0xfc,0x02);
	GC032A_write_cmos_sensor(0xfe,0x02);
	GC032A_write_cmos_sensor(0x81,0x03); 

	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0x77,0x64);
	GC032A_write_cmos_sensor(0x78,0x40);
	GC032A_write_cmos_sensor(0x79,0x60);
	/*ANALOG & CISCTL*/
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0x03,0x01);
	GC032A_write_cmos_sensor(0x04,0xce);
	GC032A_write_cmos_sensor(0x05,0x01);
	GC032A_write_cmos_sensor(0x06,0xad);
	GC032A_write_cmos_sensor(0x07,0x00);
	GC032A_write_cmos_sensor(0x08,0x10);
	GC032A_write_cmos_sensor(0x0a,0x00);
	GC032A_write_cmos_sensor(0x0c,0x00);
	GC032A_write_cmos_sensor(0x0d,0x01);
	GC032A_write_cmos_sensor(0x0e,0xe8);
	GC032A_write_cmos_sensor(0x0f,0x02);
	GC032A_write_cmos_sensor(0x10,0x88);
	GC032A_write_cmos_sensor(0x17,0x54);
	GC032A_write_cmos_sensor(0x19,0x08);
	GC032A_write_cmos_sensor(0x1a,0x0a);
	GC032A_write_cmos_sensor(0x1f,0x40);
	GC032A_write_cmos_sensor(0x20,0x30);
	GC032A_write_cmos_sensor(0x2e,0x80);
	GC032A_write_cmos_sensor(0x2f,0x2b);
	GC032A_write_cmos_sensor(0x30,0x1a);
	GC032A_write_cmos_sensor(0xfe,0x02);
	GC032A_write_cmos_sensor(0x03,0x02);
	GC032A_write_cmos_sensor(0x05,0xd7);
	GC032A_write_cmos_sensor(0x06,0x60);
	GC032A_write_cmos_sensor(0x08,0x80);
	GC032A_write_cmos_sensor(0x12,0x89);

	/*blk*/
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0x18,0x02);
	GC032A_write_cmos_sensor(0xfe,0x02);
	GC032A_write_cmos_sensor(0x40,0x22);
	GC032A_write_cmos_sensor(0x45,0x00);
	GC032A_write_cmos_sensor(0x46,0x00);
	GC032A_write_cmos_sensor(0x49,0x20);
	GC032A_write_cmos_sensor(0x4b,0x3c);
	GC032A_write_cmos_sensor(0x50,0x20);
	GC032A_write_cmos_sensor(0x42,0x10);

	/*isp*/
	GC032A_write_cmos_sensor(0xfe,0x01);
	GC032A_write_cmos_sensor(0x0a,0xc5);
	GC032A_write_cmos_sensor(0x45,0x00);
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0x40,0xff);
	GC032A_write_cmos_sensor(0x41,0x25);
	GC032A_write_cmos_sensor(0x42,0xcf);
	GC032A_write_cmos_sensor(0x43,0x10);
	GC032A_write_cmos_sensor(0x44,0x83);
	GC032A_write_cmos_sensor(0x46,0x22);
	GC032A_write_cmos_sensor(0x49,0x03);
	GC032A_write_cmos_sensor(0x52,0x02);
	GC032A_write_cmos_sensor(0x54,0x00);
	GC032A_write_cmos_sensor(0xfe,0x02);
	GC032A_write_cmos_sensor(0x22,0xf6);

	/*Shading*/
	GC032A_write_cmos_sensor(0xfe,0x01);	
	GC032A_write_cmos_sensor(0xc1,0x38);
	GC032A_write_cmos_sensor(0xc2,0x4c);
	GC032A_write_cmos_sensor(0xc3,0x00);
	GC032A_write_cmos_sensor(0xc4,0x32);
	GC032A_write_cmos_sensor(0xc5,0x24);
	GC032A_write_cmos_sensor(0xc6,0x16);
	GC032A_write_cmos_sensor(0xc7,0x08);
	GC032A_write_cmos_sensor(0xc8,0x08);
	GC032A_write_cmos_sensor(0xc9,0x00);
	GC032A_write_cmos_sensor(0xca,0x20);
	GC032A_write_cmos_sensor(0xdc,0x8a);
	GC032A_write_cmos_sensor(0xdd,0xa0);
	GC032A_write_cmos_sensor(0xde,0xa6);
	GC032A_write_cmos_sensor(0xdf,0x75);

	/*AWB*/
	GC032A_write_cmos_sensor(0xfe,0x01);
	GC032A_write_cmos_sensor(0x7c,0x09);
	GC032A_write_cmos_sensor(0x65,0x06);
	GC032A_write_cmos_sensor(0x7c,0x08);
	GC032A_write_cmos_sensor(0x56,0xf4); 
	GC032A_write_cmos_sensor(0x66,0x0f); 
	GC032A_write_cmos_sensor(0x67,0x84);
	GC032A_write_cmos_sensor(0x6b,0x80);
	GC032A_write_cmos_sensor(0x6d,0x12);
	GC032A_write_cmos_sensor(0x6e,0xb0); 
	GC032A_write_cmos_sensor(0x86,0x00);
	GC032A_write_cmos_sensor(0x87,0x00);
	GC032A_write_cmos_sensor(0x88,0x00);
	GC032A_write_cmos_sensor(0x89,0x00);
	GC032A_write_cmos_sensor(0x8a,0x00);
	GC032A_write_cmos_sensor(0x8b,0x00);
	GC032A_write_cmos_sensor(0x8c,0x00);
	GC032A_write_cmos_sensor(0x8d,0x00);
	GC032A_write_cmos_sensor(0x8e,0x00);
	GC032A_write_cmos_sensor(0x8f,0x00);
	GC032A_write_cmos_sensor(0x90,0x00);
	GC032A_write_cmos_sensor(0x91,0x00);
	GC032A_write_cmos_sensor(0x92,0xf4);
	GC032A_write_cmos_sensor(0x93,0xd5);
	GC032A_write_cmos_sensor(0x94,0x50);
	GC032A_write_cmos_sensor(0x95,0x0f);
	GC032A_write_cmos_sensor(0x96,0xf4);
	GC032A_write_cmos_sensor(0x97,0x2d);
	GC032A_write_cmos_sensor(0x98,0x0f);
	GC032A_write_cmos_sensor(0x99,0xa6);
	GC032A_write_cmos_sensor(0x9a,0x2d);
	GC032A_write_cmos_sensor(0x9b,0x0f);
	GC032A_write_cmos_sensor(0x9c,0x59);
	GC032A_write_cmos_sensor(0x9d,0x2d);
	GC032A_write_cmos_sensor(0x9e,0xaa);
	GC032A_write_cmos_sensor(0x9f,0x67);
	GC032A_write_cmos_sensor(0xa0,0x59);
	GC032A_write_cmos_sensor(0xa1,0x00);
	GC032A_write_cmos_sensor(0xa2,0x00);
	GC032A_write_cmos_sensor(0xa3,0x0a);
	GC032A_write_cmos_sensor(0xa4,0x00);
	GC032A_write_cmos_sensor(0xa5,0x00);
	GC032A_write_cmos_sensor(0xa6,0xd4);
	GC032A_write_cmos_sensor(0xa7,0x9f);
	GC032A_write_cmos_sensor(0xa8,0x55);
	GC032A_write_cmos_sensor(0xa9,0xd4);
	GC032A_write_cmos_sensor(0xaa,0x9f);
	GC032A_write_cmos_sensor(0xab,0xac);
	GC032A_write_cmos_sensor(0xac,0x9f);
	GC032A_write_cmos_sensor(0xad,0x55);
	GC032A_write_cmos_sensor(0xae,0xd4);
	GC032A_write_cmos_sensor(0xaf,0xac);
	GC032A_write_cmos_sensor(0xb0,0xd4);
	GC032A_write_cmos_sensor(0xb1,0xa3);
	GC032A_write_cmos_sensor(0xb2,0x55);
	GC032A_write_cmos_sensor(0xb3,0xd4);
	GC032A_write_cmos_sensor(0xb4,0xac);
	GC032A_write_cmos_sensor(0xb5,0x00);
	GC032A_write_cmos_sensor(0xb6,0x00);
	GC032A_write_cmos_sensor(0xb7,0x05);
	GC032A_write_cmos_sensor(0xb8,0xd6);
	GC032A_write_cmos_sensor(0xb9,0x8c);

	/*CC*/
	GC032A_write_cmos_sensor(0xfe,0x01);
	GC032A_write_cmos_sensor(0xd0,0x40);
	GC032A_write_cmos_sensor(0xd1,0xf8);
	GC032A_write_cmos_sensor(0xd2,0x00);
	GC032A_write_cmos_sensor(0xd3,0xfa);
	GC032A_write_cmos_sensor(0xd4,0x45);
	GC032A_write_cmos_sensor(0xd5,0x02);

	GC032A_write_cmos_sensor(0xd6,0x30);
	GC032A_write_cmos_sensor(0xd7,0xfa);
	GC032A_write_cmos_sensor(0xd8,0x08);
	GC032A_write_cmos_sensor(0xd9,0x08);
	GC032A_write_cmos_sensor(0xda,0x58);
	GC032A_write_cmos_sensor(0xdb,0x02);
	GC032A_write_cmos_sensor(0xfe,0x00);

	/*Gamma*/
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0xba,0x00);
	GC032A_write_cmos_sensor(0xbb,0x04);
	GC032A_write_cmos_sensor(0xbc,0x0a);
	GC032A_write_cmos_sensor(0xbd,0x0e);
	GC032A_write_cmos_sensor(0xbe,0x22);
	GC032A_write_cmos_sensor(0xbf,0x30);
	GC032A_write_cmos_sensor(0xc0,0x3d);
	GC032A_write_cmos_sensor(0xc1,0x4a);
	GC032A_write_cmos_sensor(0xc2,0x5d);
	GC032A_write_cmos_sensor(0xc3,0x6b);
	GC032A_write_cmos_sensor(0xc4,0x7a);
	GC032A_write_cmos_sensor(0xc5,0x85);
	GC032A_write_cmos_sensor(0xc6,0x90);
	GC032A_write_cmos_sensor(0xc7,0xa5);
	GC032A_write_cmos_sensor(0xc8,0xb5);
	GC032A_write_cmos_sensor(0xc9,0xc2);
	GC032A_write_cmos_sensor(0xca,0xcc);
	GC032A_write_cmos_sensor(0xcb,0xd5);
	GC032A_write_cmos_sensor(0xcc,0xde);
	GC032A_write_cmos_sensor(0xcd,0xea);
	GC032A_write_cmos_sensor(0xce,0xf5);
	GC032A_write_cmos_sensor(0xcf,0xff);

	/*Auto Gamma*/                      
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0x5a,0x08);
	GC032A_write_cmos_sensor(0x5b,0x0f);
	GC032A_write_cmos_sensor(0x5c,0x15);
	GC032A_write_cmos_sensor(0x5d,0x1c);
	GC032A_write_cmos_sensor(0x5e,0x28);
	GC032A_write_cmos_sensor(0x5f,0x36);
	GC032A_write_cmos_sensor(0x60,0x45);
	GC032A_write_cmos_sensor(0x61,0x51);
	GC032A_write_cmos_sensor(0x62,0x6a);
	GC032A_write_cmos_sensor(0x63,0x7d);
	GC032A_write_cmos_sensor(0x64,0x8d);
	GC032A_write_cmos_sensor(0x65,0x98);
	GC032A_write_cmos_sensor(0x66,0xa2);
	GC032A_write_cmos_sensor(0x67,0xb5);
	GC032A_write_cmos_sensor(0x68,0xc3);
	GC032A_write_cmos_sensor(0x69,0xcd);
	GC032A_write_cmos_sensor(0x6a,0xd4);
	GC032A_write_cmos_sensor(0x6b,0xdc);
	GC032A_write_cmos_sensor(0x6c,0xe3);
	GC032A_write_cmos_sensor(0x6d,0xf0);
	GC032A_write_cmos_sensor(0x6e,0xf9);
	GC032A_write_cmos_sensor(0x6f,0xff);

	/*Gain*/
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0x70,0x50);

	/*AEC*/
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0x4f,0x01);
	GC032A_write_cmos_sensor(0xfe,0x01);
	GC032A_write_cmos_sensor(0x0d,0x00);
	GC032A_write_cmos_sensor(0x12,0xa0);
	GC032A_write_cmos_sensor(0x13,0x3a);	
	GC032A_write_cmos_sensor(0x44,0x04);
	GC032A_write_cmos_sensor(0x1f,0x30);
	GC032A_write_cmos_sensor(0x20,0x40);	
	GC032A_write_cmos_sensor(0x26,0x9a);
	GC032A_write_cmos_sensor(0x3e,0x20);
	GC032A_write_cmos_sensor(0x3f,0x2d);
	GC032A_write_cmos_sensor(0x40,0x40);
	GC032A_write_cmos_sensor(0x41,0x5b);
	GC032A_write_cmos_sensor(0x42,0x82);
	GC032A_write_cmos_sensor(0x43,0xb7);
	GC032A_write_cmos_sensor(0x04,0x0a);
	GC032A_write_cmos_sensor(0x02,0x79);
	GC032A_write_cmos_sensor(0x03,0xc0);

	/*measure window*/
	GC032A_write_cmos_sensor(0xfe,0x01);
	GC032A_write_cmos_sensor(0xcc,0x08);
	GC032A_write_cmos_sensor(0xcd,0x08);
	GC032A_write_cmos_sensor(0xce,0xa4);
	GC032A_write_cmos_sensor(0xcf,0xec);

	/*DNDD*/
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0x81,0xb8);
	GC032A_write_cmos_sensor(0x82,0x12);
	GC032A_write_cmos_sensor(0x83,0x0a);
	GC032A_write_cmos_sensor(0x84,0x01);
	GC032A_write_cmos_sensor(0x86,0x50);
	GC032A_write_cmos_sensor(0x87,0x18);
	GC032A_write_cmos_sensor(0x88,0x10);
	GC032A_write_cmos_sensor(0x89,0x70);
	GC032A_write_cmos_sensor(0x8a,0x20);
	GC032A_write_cmos_sensor(0x8b,0x10);
	GC032A_write_cmos_sensor(0x8c,0x08);
	GC032A_write_cmos_sensor(0x8d,0x0a);

	/*Intpee*/
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0x8f,0xaa);
	GC032A_write_cmos_sensor(0x90,0x9c);
	GC032A_write_cmos_sensor(0x91,0x52);
	GC032A_write_cmos_sensor(0x92,0x03);
	GC032A_write_cmos_sensor(0x93,0x03);
	GC032A_write_cmos_sensor(0x94,0x08);
	GC032A_write_cmos_sensor(0x95,0x44);
	GC032A_write_cmos_sensor(0x97,0x00);
	GC032A_write_cmos_sensor(0x98,0x00);

	/*ASDE*/
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0xa1,0x30);
	GC032A_write_cmos_sensor(0xa2,0x41);
	GC032A_write_cmos_sensor(0xa4,0x30);
	GC032A_write_cmos_sensor(0xa5,0x20);
	GC032A_write_cmos_sensor(0xaa,0x30);
	GC032A_write_cmos_sensor(0xac,0x32);

	/*YCP*/
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0xd1,0x3c);
	GC032A_write_cmos_sensor(0xd2,0x3c);
	GC032A_write_cmos_sensor(0xd3,0x38);
	GC032A_write_cmos_sensor(0xd6,0xf4);
	GC032A_write_cmos_sensor(0xd7,0x1d);
	GC032A_write_cmos_sensor(0xdd,0x73);
	GC032A_write_cmos_sensor(0xde,0x84);

	/*Banding*/
	GC032A_write_cmos_sensor(0xfe,0x00);
	GC032A_write_cmos_sensor(0x05,0x01);
	GC032A_write_cmos_sensor(0x06,0xad);
	GC032A_write_cmos_sensor(0x07,0x00);
	GC032A_write_cmos_sensor(0x08,0x10);

	GC032A_write_cmos_sensor(0xfe,0x01);
	GC032A_write_cmos_sensor(0x25,0x00);
	GC032A_write_cmos_sensor(0x26,0x9a);

	GC032A_write_cmos_sensor(0x27,0x01);
	GC032A_write_cmos_sensor(0x28,0xce);
	GC032A_write_cmos_sensor(0x29,0x03);
	GC032A_write_cmos_sensor(0x2a,0x02);
	GC032A_write_cmos_sensor(0x2b,0x04);
	GC032A_write_cmos_sensor(0x2c,0x36);
	GC032A_write_cmos_sensor(0x2d,0x07);
	GC032A_write_cmos_sensor(0x2e,0xd2);
	GC032A_write_cmos_sensor(0x2f,0x0b);
	GC032A_write_cmos_sensor(0x30,0x6e);
	GC032A_write_cmos_sensor(0x31,0x0e);
	GC032A_write_cmos_sensor(0x32,0x70);
	GC032A_write_cmos_sensor(0x33,0x12);
	GC032A_write_cmos_sensor(0x34,0x0c);
	GC032A_write_cmos_sensor(0x3c,0x30);
	GC032A_write_cmos_sensor(0xfe,0x00);

}






kal_uint16 GC032a_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd = { (char)(addr & 0xFF) };

	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, GC032a_READ_ID);

    return get_byte;
}

UINT32 GC032aOpen(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	//SENSORDB("<Jet> Entry GC032aOpen!!!\r\n");

    for(i=0;i<3;i++)
		{
		sensor_id = ((GC032a_read_cmos_sensor(0xf0) << 8) | GC032a_read_cmos_sensor(0xf1));
		if(sensor_id == GC032A_SENSOR_ID)
		{
			printk("GC032amipi Read Sensor ID ok[open] = 0x%x\n", sensor_id);
			break;
		}
		}
	//  Read sensor ID to adjust I2C is OK?
	if(sensor_id != GC032A_SENSOR_ID)
	{
		printk("GC032amipi Read Sensor ID Fail[open] = 0x%x\n", sensor_id);
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	GC032a_Sensor_Init();

	return ERROR_NONE;
} /* GC032aOpen */

kal_uint32 GC032a_Read_Shutter(void)
{
    	kal_uint8 temp_reg1, temp_reg2,temp_reg3;
	kal_uint32 shutter;

	temp_reg1 = GC032a_read_cmos_sensor(0x04);
	temp_reg2 = GC032a_read_cmos_sensor(0x03);

	temp_reg3 = GC032a_read_cmos_sensor(0xef);

//printk("GC032a_Read_Shutter_1 0xef=%d\n",temp_reg3);
	shutter = (temp_reg1 & 0xFF) | (temp_reg2 << 8) | (temp_reg3 << 16);
printk("GC032a_Read_Shutter_1 shutter=%d\n",shutter);
	return shutter;
} /* GC032a_read_shutter */

UINT32 GC032a_GetSensorID(void)
{
    int  retry = 3;
	UINT32 sensorID;
    // check if sensor ID correct
    do {
        sensorID=((GC032a_read_cmos_sensor(0xf0)<< 8)|GC032a_read_cmos_sensor(0xf1));
        if (sensorID == GC032A_SENSOR_ID)
            break;
        printk("hct-drv-%s: Read Sensor ID Fail = 0x%04x\n", __func__, sensorID);
        retry--;
    } while (retry > 0);

    if (sensorID != GC032A_SENSOR_ID) {
        sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}

SENSOR_FUNCTION_STRUCT_1 SensorFuncGC032AYUV_1=
{
	GC032aOpen,
	GC032a_Read_Shutter,
	GC032a_GetSensorID,
};
