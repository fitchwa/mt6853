#include <linux/hct_include/hct_project_all_config.h>
#include "imx582_otp_cam_cal.h"
//read from imgsensor.c

extern u8 imx582_DCC_data[96];
extern u8 imx582_eeprom_data[2048];
unsigned int  imx582_selective_read_region(struct i2c_client *client, unsigned int addr,
	unsigned char *data, unsigned int size)
{
	if(addr == 2048)
	{
		memcpy((void *)data,(void *)&imx582_DCC_data[0],size);
	}
	else
	{
    	memcpy((void *)data,(void *)&imx582_eeprom_data[addr],size);
    }
	printk("imx582_selective_read_region addr:%d,size %d data read = 0x%x\n",addr,size, *data);
    return size;
}


