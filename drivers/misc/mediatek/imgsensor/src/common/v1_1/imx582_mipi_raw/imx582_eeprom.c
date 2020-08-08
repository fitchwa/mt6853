#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include "kd_camera_typedef.h"


#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"


#define PFX "imx582_otp"
#define LOG_INF(fmt, args...)   pr_debug(PFX "[%s] " fmt, __func__, ##args)

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

BYTE imx582_DCC_data[96]= {0};
BYTE imx582_LRC_data[384]= {0};
BYTE imx582_QSC_data[2304]= {0};
static BYTE imx582_AWB_data[12] = {0};

#define USHORT             unsigned short
#define BYTE               unsigned char

#define EEPROM_WRITE_ID         0xA0

#define OTP_DATA                imx582_eeprom_data
#define OTP_PLATFORM_CHOICE     0x0a     //bit0:awb, bit1:af, bit3:lsc
#define OTP_FLAG_ADDR           0x0000
#define AWB_ADDR                0x000C
#define AWB_CHECK_ADDR          0x001D
#define LSC_ADDR                0x0031
#define LSC_CHECK_ADDR          0x077D
#define AF_ADDR                 0x0021
#define AF_CHECK_ADDR           0x0027
#define LRC_ADDR				0x0781
#define LRC_CHECK_ADDR			0x0901
#define DCC_ADDR				0x0903
#define DCC_CHECK_ADDR			0x0963
#define QSC_ADDR				0x0965
#define QSC_CHECK_ADDR			0x1265
#define CHECKSUM_METHOD(x,addr)  \
( (x % 256) == read_cmos_sensor_8(addr) )

BYTE OTP_DATA[2048]= {0};

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,EEPROM_WRITE_ID);
    return get_byte;
}
static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
	char pusendcmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pusendcmd, 3, 0x34);
}

static void read_imx582_LRC(void)
{
	int size = 384;
	int addr = LRC_ADDR;
	int i, checksum = 0;

	LOG_INF("read imx582 LRC, size = %d\n", size);

	for(i = 0; i < size; i++)
	{
		imx582_LRC_data[i] = read_cmos_sensor_8(addr + i);
		checksum += imx582_LRC_data[i];
	}
	if(CHECKSUM_METHOD(checksum, LRC_CHECK_ADDR))
		LOG_INF("LRC Checksum Success");
	else
		LOG_INF("LRC Checksum Failed!!!");

}

static void read_imx582_QSC(void)
{
	int size = 2304;
	int addr = QSC_ADDR;
	int i, checksum = 0;

	LOG_INF("read imx582 QSC, size = %d\n", size);

	for(i = 0; i < size; i++)
	{
		imx582_QSC_data[i] = read_cmos_sensor_8(addr + i);
		checksum += imx582_QSC_data[i];
	}
	if(CHECKSUM_METHOD(checksum, QSC_CHECK_ADDR))
		LOG_INF("QSC Checksum Success");
	else
		LOG_INF("QSC Checksum Failed!!!");
}

//void read_imx582_DCC(kal_uint16 addr,BYTE* data, kal_uint32 size)
static void read_imx582_DCC(void)
{
	int i, checksum = 0;
	int addr = DCC_ADDR;
	int size = 96;

	LOG_INF("read imx582 DCC, size = %d\n", size);

	for(i = 0; i < size; i++)
	{
		imx582_DCC_data[i] = read_cmos_sensor_8(addr + i);
		checksum += imx582_DCC_data[i];
		//LOG_INF("imx582_DCC_data[%d] = 0x%x\n", i, imx582_DCC_data[i]);
	}
	if(CHECKSUM_METHOD(checksum, DCC_CHECK_ADDR))
		LOG_INF("DCC Checksum Success");
	else
		LOG_INF("DCC Checksum Failed!!!");

	//memcpy(data, imx582_DCC_data , size);
}

void imx582_get_otp_data(void)
{
	int i;
	kal_uint32 checksum = 0;

    if(0x01 != read_cmos_sensor_8(OTP_FLAG_ADDR))
	{
        LOG_INF("OTP DATA Invalid!!!\n");
        return;
    }

//0x010b00ff
	OTP_DATA[0] = 0xff;
	OTP_DATA[1] = 0x00;
	OTP_DATA[2] = 0x0b;
	OTP_DATA[3] = 0x01;

	OTP_DATA[4] = OTP_PLATFORM_CHOICE;

	//AF
    if(OTP_PLATFORM_CHOICE & 0x2)
    {
    	for(i = 0; i < 6; i++)
    	{
    		OTP_DATA[i + 6] = read_cmos_sensor_8(AF_ADDR + i);
			checksum += OTP_DATA[i + 6];
		}
    	if(CHECKSUM_METHOD(checksum, AF_CHECK_ADDR))
    	{
    		LOG_INF("AF Checksum OK\n");
			OTP_DATA[8] = OTP_DATA[7];
			OTP_DATA[9] = OTP_DATA[6];
			
			OTP_DATA[6] = OTP_DATA[11];
			OTP_DATA[7] = OTP_DATA[10];

            LOG_INF("AFInf = %d, AFMarco = %d\n", OTP_DATA[7] << 8 | OTP_DATA[6], OTP_DATA[9] << 8 | OTP_DATA[8]);
    	}
    	else
    	{
    		LOG_INF("AF Checksum Failed!!!\n");
    		OTP_DATA[4] = OTP_DATA[4] & (~0x02);
    	}
    }


//AWB Platform
    if(OTP_PLATFORM_CHOICE & 0x1)
    {
        //checksum = 0;

        for(i = 0; i < 8; i++)
        	OTP_DATA[i + 10] = read_cmos_sensor_8(AWB_ADDR + i);

		//for(i = 0; i < 12; i++)
		//	checksum += read_cmos_sensor_8(AWB_ADDR + i);

        if(CHECKSUM_METHOD(checksum, AWB_CHECK_ADDR))
        {
			LOG_INF("AWB OTP Checksum OK\n");
			LOG_INF("Unit_R = 0x%x, Unit_Gr = 0x%x, Unit_Gb = 0x%x, Unit_B = 0x%x\n", OTP_DATA[10], OTP_DATA[11], OTP_DATA[12], OTP_DATA[13]);
			LOG_INF("Golden_R = 0x%x, Golden_Gr = 0x%x, Golden_Gb = 0x%x, Golden_B = 0x%x\n", OTP_DATA[14], OTP_DATA[15], OTP_DATA[16], OTP_DATA[17]);
        }
        else
        {
			LOG_INF("AWB OTP Checksum Failed!!!\n");
			OTP_DATA[4] = OTP_DATA[4] & (~0x01);
        }
    }
	else	//AWB Sensor
	{
		checksum = 0;
		for(i = 0; i < 12; i++)
		{
			imx582_AWB_data[i] = read_cmos_sensor_8(AWB_ADDR + i);
			checksum += imx582_AWB_data[i];
		}

		if(CHECKSUM_METHOD(checksum, AWB_CHECK_ADDR))
			LOG_INF("Sensor AWB Checksum OK\n");
		else
			LOG_INF("Sensor AWB Checksum Failed!!!\n");
	}

	//LSC
    if(OTP_PLATFORM_CHOICE & 0x8)
    {
    	checksum = 0;
    	for(i = 0; i < 1868; i++)
    	{
    		OTP_DATA[20 + i] = read_cmos_sensor_8(LSC_ADDR + i);
    		checksum += OTP_DATA[20 + i];
			//LOG_INF("LSC data[%d] = 0x%x\n", i, OTP_DATA[20 + i]);
    	}

        if(CHECKSUM_METHOD(checksum, LSC_CHECK_ADDR))
    		LOG_INF("LSC Checksum OK\n");
    	else
    	{
    		LOG_INF("LSC Checksum Failed!!!\n");
    		OTP_DATA[4] = OTP_DATA[4] & (~0x08);
    	}
    }
	
	read_imx582_DCC();
	read_imx582_QSC();
	read_imx582_LRC();
}


void load_imx582_awb(void)
{
	kal_uint32 Unit_RG, Unit_BG, Unit_GG, Golden_RG, Golden_BG, Golden_GG;
	kal_uint32 R_Gain, B_Gain, G_Gain, G_Gain_R, G_Gain_B;
	kal_uint32 r_ratio, b_ratio;
	#define GAIN_DEFAULT 0x0100

	Unit_RG   = ((imx582_AWB_data[0] << 8) | imx582_AWB_data[1]);
	Unit_BG   = ((imx582_AWB_data[2] << 8) | imx582_AWB_data[3]);
	Unit_GG   = ((imx582_AWB_data[4] << 8) | imx582_AWB_data[5]);
	Golden_RG = ((imx582_AWB_data[6] << 8) | imx582_AWB_data[7]);
	Golden_BG = ((imx582_AWB_data[8] << 8) | imx582_AWB_data[9]);
	Golden_GG = ((imx582_AWB_data[10] << 8) | imx582_AWB_data[11]);

	LOG_INF("Unit_RG = 0x%x, Unit_BG = 0x%x, Unit_GG = 0x%x\n", Unit_RG, Unit_BG, Unit_GG);
	LOG_INF("Golden_RG = 0x%x, Golden_BG = 0x%x, Golden_GG = 0x%x\n", Golden_RG, Golden_BG, Golden_GG);

	r_ratio = 1024 * (Golden_RG) / (Unit_RG);
    b_ratio = 1024 * (Golden_BG) / (Unit_BG);

    if(r_ratio >= 1024 )
    {
        if(b_ratio>=1024) 
        {
            R_Gain = GAIN_DEFAULT * r_ratio / 1024;
            G_Gain = GAIN_DEFAULT;    
            B_Gain = GAIN_DEFAULT * b_ratio / 1024;
        }
        else
        {
            R_Gain = GAIN_DEFAULT * r_ratio / b_ratio;
            G_Gain = GAIN_DEFAULT * 1024 / b_ratio;
            B_Gain = GAIN_DEFAULT;    
        }
    }
    else             
    {
        if(b_ratio >= 1024)
        {
            R_Gain = GAIN_DEFAULT;    
            G_Gain = GAIN_DEFAULT * 1024 / r_ratio;
            B_Gain = GAIN_DEFAULT *  b_ratio / r_ratio;

        } 
        else 
        {
            G_Gain_R = GAIN_DEFAULT * 1024 / r_ratio;
            G_Gain_B = GAIN_DEFAULT * 1024 / b_ratio;

            if(G_Gain_R >= G_Gain_B)
            {
                R_Gain = GAIN_DEFAULT;
                G_Gain = GAIN_DEFAULT * 1024 / r_ratio;
                B_Gain = GAIN_DEFAULT * b_ratio / r_ratio;
            } 
            else
            {
                R_Gain = GAIN_DEFAULT * r_ratio / b_ratio;
                G_Gain = GAIN_DEFAULT * 1024 / b_ratio;
                B_Gain = GAIN_DEFAULT;
            }
        }    
    }

	LOG_INF("R_Gain=0x%x, B_Gain=0x%x, G_Gain=0x%x\n", R_Gain, B_Gain, G_Gain);
	
	write_cmos_sensor_8(0x3130, 0x01);	//imx582 need

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
}
