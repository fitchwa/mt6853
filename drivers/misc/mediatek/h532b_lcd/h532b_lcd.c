/*****************************************************************************
**      Copyright 2015 FLIR Systems - Commercial
**      Vision Systems.  All rights reserved.
**
**      Proprietary - PROPRIETARY - FLIR Systems Inc..

** The information contained in this source code is controlled under the Export
** Administration Regulations (EAR).  Diversion contrary to US law is prohibited.
** US Department of Commerce authorization may be required prior to export or
** transfer to certain destinations or foreign persons.

*****************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>

#include "h532b_lcd.h"

extern void mt_spi_enable_master_clk(struct spi_device *spidev);

/* Options */
#define LCD_TEST
#define FASTEST_FRAME

static unsigned char gbDispDirection = Direction_Normal;
static unsigned char gbDisplayData[255*40];
static unsigned short *g_pusPattern;
static bool gbDoDisplay = false;
static bool bLcdInit = false;

/* Global vars for initialization of device */
// Character device
static struct class *h532blcd_class;
static unsigned int h532blcd_major = 0;
// SPI device
struct spi_device gspiDev;
static struct regulator *h532blcd_ldo = NULL;
struct h532b_lcd_pin *gh532b_lcd_pin;
#define LCD_EL_ON		gpio_set_value(gh532b_lcd_pin->backlight_gpio, 1);
#define LCD_EL_OFF		gpio_set_value(gh532b_lcd_pin->backlight_gpio, 0);



void LCDSerialWrite(int dwType, unsigned char bData)
{
	int ret;

	switch(dwType) {
		case LCD_COMMAND :
			gpio_set_value(gh532b_lcd_pin->a0, LCD_COMMAND);
			break;
		case LCD_DATA :
			gpio_set_value(gh532b_lcd_pin->a0, LCD_DATA);
			break;
	}
	//printk("hct-drv add %s is start !\n",__func__);
	ret = spi_write(&gspiDev, &bData, 1);

	if(ret != 0)
		DBG_PRINTK("Cannot send spi message 0x%.2x", bData);
}

static int LCDDisplayThread(void *unused)
{
	int iBytePos=0, iLine=0;
	int i=0, j=0;
	unsigned short tmp;

	DBG_PRINTK(" + ");
	while(1) {
		gbDoDisplay = false;
		gh532b_lcd_pin->wi_flag = 0;
		wait_event_interruptible(gh532b_lcd_pin->wq, gh532b_lcd_pin->wi_flag);
		DBG_PRINTK("Writiing pattern\n");
		gbDoDisplay = true;
		
		/* Reset pattern */
		memset(g_pusPattern, 0x0, sizeof(*g_pusPattern));

		/* Fill pattern */
		for(iLine=0; iLine<LCD_LINE_END; iLine++) { // Total line is 40 lines
			iBytePos = 0;
			for(i=iLine*32; i<(iLine*32+31); i++) { // 32 Bytes per 1 Line
				for(j=7; j>=0; j--) {
					if(iBytePos < 253) {
						if((gbDisplayData[i]>>j) & 0x1) {
							if(gbDispDirection == Direction_Normal) {
								g_pusPattern[(iLine+1)*85 - (iBytePos/3)-1]
									|= usMask[(iBytePos%3)]; // Normal
							} else {
								g_pusPattern[(40-iLine-1)*85 + (iBytePos/3)]
									|= usMask_Inverse[(iBytePos%3)]; // Inverse
							}
						} else {
							if(gbDispDirection == Direction_Normal) {
								g_pusPattern[(iLine+1)*85 - (iBytePos/3)-1]
									&= ~usMask[(iBytePos%3)];	// Normal
							} else {
								g_pusPattern[(40-iLine-1)*85 + (iBytePos/3)]
									&= ~usMask_Inverse[(iBytePos%3)]; // Inverse
							}
						}
						iBytePos++;
					} else if((iBytePos>=253) && (iBytePos<255)) {
						if(gbDispDirection == Direction_Normal) {
							g_pusPattern[(iLine+1)*85 - (iBytePos/3)-1]
								&= ~usMask[(iBytePos%3)];	// Normal
						} else {
							g_pusPattern[(40-iLine-1)*85 + (iBytePos/3)]
								&= ~usMask_Inverse[(iBytePos%3)]; // Inverse
						}							
						iBytePos++;
					}
				}
			}
		}
		
		/* Display */
		LCDSerialWrite(LCD_COMMAND, LCD_EXT_SET_LOW); // Ext = 0
		
		LCDSerialWrite(LCD_COMMAND, LCD_CA_SET);	// Column Address Set
		LCDSerialWrite(LCD_DATA, 0x0);				// Start Column Address 0x0
		LCDSerialWrite(LCD_DATA, LCD_COLUMN_END-1);	// End Column Address 66
		LCDSerialWrite(LCD_COMMAND, LCD_LA_SET);	// Page Address Set
		LCDSerialWrite(LCD_DATA, 0x0);				// Start Line Address 0x0
		LCDSerialWrite(LCD_DATA, LCD_LINE_END-1);	// End Line Address 39
		
		LCDSerialWrite(LCD_COMMAND, LCD_RAM_WR);	// Memory Write

#ifndef FASTEST_FRAME
		for(i=0; i< (LCD_COLUMN_END*LCD_LINE_END); i++)
		{
			LCDSerialWrite(LCD_DATA, (unsigned char)((g_pusPattern[i]>>8)));
			LCDSerialWrite(LCD_DATA, (unsigned char)(g_pusPattern[i]));
		}
#else
		for(i=0; i< (LCD_COLUMN_END*LCD_LINE_END); i++)
		{
			tmp = g_pusPattern[i] >> 8;
			g_pusPattern[i] = g_pusPattern[i] << 8 | tmp;
		}
		gpio_set_value(gh532b_lcd_pin->a0, LCD_DATA);
		spi_write(&gspiDev, g_pusPattern, LCD_COLUMN_END * LCD_LINE_END * 2);
#endif

		LCDSerialWrite(LCD_COMMAND, LCD_EXT_SET_LOW); // Ext = 0			
	}
	
	DBG_PRINTK(" - ");
	return 0;
}

void LCDReset(void)
{
	DBG_PRINTK(" + ");
		gpio_set_value(gh532b_lcd_pin->reset_gpio, 0);
		udelay(100); // Waiting for Stabilizing the Power
		gpio_set_value(gh532b_lcd_pin->reset_gpio, 1);
		udelay(100);
}

void LCDInitialize(void)
{
	DBG_PRINTK(" + ");
	bLcdInit = false;



	
	LCDReset();
	printk("hct-drv. LCDInitialize. power_ldo. = %d. ", gpio_get_value(gh532b_lcd_pin->power_ldo));
	printk("hct-drv. LCDInitialize. reset_gpio. = %d. ", gpio_get_value(gh532b_lcd_pin->reset_gpio));
	printk("hct-drv. LCDInitialize. power_level_shift_ldo. = %d. ", gpio_get_value(gh532b_lcd_pin->power_level_shift_ldo));

printk("hct-drv. LCDInitialize. 11. = %d. ", gpio_get_value(gh532b_lcd_pin->a0));

	LCDSerialWrite( LCD_COMMAND, LCD_EXT_SET_LOW );	// Ext = 0
	
	LCDSerialWrite( LCD_COMMAND, LCD_DIS_CTRL );	// Display Control
	printk("hct-drv. LCDInitialize. 22. = %d. ", gpio_get_value(gh532b_lcd_pin->a0));
	
	LCDSerialWrite( LCD_DATA, 0x0 );				// CL=X1
	LCDSerialWrite( LCD_DATA, 0x9 );				// Duty=40
	LCDSerialWrite( LCD_DATA, 0x0 );				// FR Inverse-Set Value	
	printk("hct-drv. LCDInitialize. 33. = %d. ", gpio_get_value(gh532b_lcd_pin->a0));
	
	LCDSerialWrite( LCD_COMMAND, LCD_OSC_ON );		// OSC On
	
	LCDSerialWrite( LCD_COMMAND, LCD_SLP_OUT );		// Sleep Out

	// LCDSerialWrite( LCD_COMMAND, LCD_OSC_ON );		// OSC On

	LCDSerialWrite( LCD_COMMAND, LCD_PWR_CTRL );	// Power Control Set
	LCDSerialWrite( LCD_DATA, 0x08 );				// Booster Must Be On Set
	msleep(10);
	LCDSerialWrite( LCD_COMMAND, LCD_PWR_CTRL );	// Power Control Set
	LCDSerialWrite( LCD_DATA, 0x0B );				// Booster, Regulator, Follower On
	LCDSerialWrite( LCD_COMMAND, LCD_VOL_CTRL );	// Electronic Control
	LCDSerialWrite( LCD_DATA, 0x7 );				// Vop=14.0 V
	LCDSerialWrite( LCD_DATA, 0x2 );	
	// YGRYU 2010.07.13 to test background color	
	// LCDSerialWrite( LCD_DATA, 0x33 );				// Vop=8.2V
	// LCDSerialWrite( LCD_DATA, 0x1 );

	// LCDSerialWrite( LCD_COMMAND, LCD_DIS_CTRL );	// Display Control
	// LCDSerialWrite( LCD_DATA, 0x0 );				// CL=X1
	// LCDSerialWrite( LCD_DATA, 0x9 );				// Duty=40
	// LCDSerialWrite( LCD_DATA, 0x0 );				// FR Inverse-Set Value
	
	// LCDSerialWrite( LCD_COMMAND, LCD_DIS_NOR );		// Normal Display
	LCDSerialWrite( LCD_COMMAND, LCD_DIS_INV );		// Inverse Display
	
	LCDSerialWrite( LCD_COMMAND, LCD_COM_SCN );		// COM Scan Direction
	LCDSerialWrite( LCD_DATA, 0x0 );				// 0->79, 80->159
	
	LCDSerialWrite( LCD_COMMAND, LCD_DAT_SDR ); 	// Data Scan Direction
	LCDSerialWrite( LCD_DATA, 0x0 );				// Normal
	LCDSerialWrite( LCD_DATA, 0x0 );				// RGB Arrangement
	// LCDSerialWrite( LCD_DATA, 0x02 );				// 3B3PP
	LCDSerialWrite( LCD_DATA, 0x01 );				// 2B3PP
	
	LCDSerialWrite( LCD_COMMAND, LCD_LA_SET );		// Line Address Set
	LCDSerialWrite( LCD_DATA, 0x0 );				// Start Line = 0
	LCDSerialWrite( LCD_DATA, LCD_LINE_END );		// End Line = 39
	
	LCDSerialWrite( LCD_COMMAND, LCD_CA_SET );		// Column Address Set
	LCDSerialWrite( LCD_DATA, 0x0 );				// Start Column = 0
	LCDSerialWrite( LCD_COMMAND, LCD_COLUMN_END-1 );// End Column = 66
	
	LCDSerialWrite( LCD_COMMAND, LCE_EXT_SET_HIGH );// Ext = 1
	
	LCDSerialWrite( LCD_COMMAND, LCD_ANA_SET );		// Analog Circuit Set
	LCDSerialWrite( LCD_DATA, 0x0 );				// OSC Frequency = 0 (default)
	LCDSerialWrite( LCD_DATA, 0x01 );				// Booster Efficiency = 01 (default)
	LCDSerialWrite( LCD_DATA, 0x6 );				// Bias = 1/14
	
	LCDSerialWrite( LCD_COMMAND, LCD_SW_INT );		// Software Initial
	
	LCDSerialWrite( LCD_COMMAND, LCD_EXT_SET_LOW );	// Ext = 0

	LCDSerialWrite( LCD_COMMAND, LCD_DIS_ON );		// Display On

	msleep(1);
	
	bLcdInit = true;
	DBG_PRINTK(" - ");
}
//int hct_log_flag = 0;
void LCDDisplay( unsigned short *pusData )
{
#ifndef FASTEST_FRAME
	int i=0;
#endif
	 DBG_PRINTK(" + ");

	LCDSerialWrite( LCD_COMMAND, LCD_EXT_SET_LOW );	// Ext = 0
	LCDSerialWrite( LCD_COMMAND, LCD_CA_SET );		// Column Address Set
	LCDSerialWrite( LCD_DATA, 0x0 );				// Start Column Address 0x0
	LCDSerialWrite( LCD_DATA, LCD_COLUMN_END-1 );	// End Column Address 66
	LCDSerialWrite( LCD_COMMAND, LCD_LA_SET );		// Page Address Set
	LCDSerialWrite( LCD_DATA, 0x0 );				// Start Line Address 0x0
	LCDSerialWrite( LCD_DATA, LCD_LINE_END-1 );		// End Line Address 39
	
	LCDSerialWrite( LCD_COMMAND, LCD_RAM_WR );		// Memory Write

#ifndef FASTEST_FRAME
	for( i=0; i< (LCD_COLUMN_END*LCD_LINE_END); i++ ) {
		LCDSerialWrite( LCD_DATA, (unsigned char)((pusData[i]>>8)) );
		LCDSerialWrite( LCD_DATA, (unsigned char)(pusData[i]) );
	}
#else
	gpio_set_value(gh532b_lcd_pin->a0, LCD_DATA);
	//printk("hct-drv add %s is pusData start!\n",__func__);
	//hct_log_flag = 1;
	spi_write(&gspiDev, g_pusPattern, LCD_COLUMN_END * LCD_LINE_END * 2);
	//printk("hct-drv-%s g_pusPatternbuf = %p pusDatabuf = %p len = %d \n",__func__,g_pusPattern, pusData,LCD_COLUMN_END * LCD_LINE_END * 2);
	//hct_log_flag = 0;
	//printk("hct-drv add %s is pusData end!\n",__func__);
#endif

	LCDSerialWrite( LCD_COMMAND, LCD_EXT_SET_LOW );	// Ext = 0
	
	 DBG_PRINTK(" - ");
}

#ifdef LCD_TEST
static int LCDTestThread(void *unused)
{
	int i, j;

	while(1) {
		memset(g_pusPattern, 0x0, sizeof(*g_pusPattern));
		for( i=0; i<LCD_LINE_END; i++ ) {
			for( j=0; j<LCD_COLUMN_END; j++ ) {
				if( j < 20 ) {
					if(i>=20){
						g_pusPattern[i*LCD_COLUMN_END+j] = 0xFFDF;
					}
				} else if( j < 40 ) {
					if(i<20){
						g_pusPattern[i*LCD_COLUMN_END+j] = 0x0;
					}
				} else if( j < 60 ) {
					if(i<20){
						g_pusPattern[i*LCD_COLUMN_END+j] = 0xFFDF;
					}
				} else if( j < 80 ) {
					if(i<20){
						g_pusPattern[i*LCD_COLUMN_END+j] = 0x0;
					}
				} else if( j < 100 ) {
					if(i<20){
						g_pusPattern[i*LCD_COLUMN_END+j] = 0xFFDF;
					}
				} else if( j < 120 ) {
					if(i<20){
						g_pusPattern[i*LCD_COLUMN_END+j] = 0x0;
					}
				} else {
					g_pusPattern[i*LCD_COLUMN_END+j] = 0xFFDF;
				}
			}
		}
		LCDDisplay(g_pusPattern);

		msleep(1000);

		memset(g_pusPattern, 0x0, sizeof(*g_pusPattern));
		for( i=0; i<LCD_LINE_END; i++ ) {
			for( j=0; j<LCD_COLUMN_END; j++ ) {
				if( i < 10 ) {
					g_pusPattern[i*LCD_COLUMN_END+j] = 0xFFDF;
				} else if( i < 20 ) {
					g_pusPattern[i*LCD_COLUMN_END+j] = 0x0;
				} else if( i < 30 ) {
					g_pusPattern[i*LCD_COLUMN_END+j] = 0xFFDF;
				} else {
					g_pusPattern[i*LCD_COLUMN_END+j] = 0x0;
				}
			}
		}			
		LCDDisplay(g_pusPattern);

		msleep(1000);
	}
	return 0;

}
#endif

void LCD_PowerUp (void)
{
#ifdef LCD_TEST
	struct task_struct *thread;
#endif
	//printk("hct-drv add h532b %s is start !\n",__func__);
	DBG_PRINTK("called");
	// Initialize LCD Module
	LCDInitialize();

	memset(g_pusPattern, 0x00, sizeof(*g_pusPattern));
	LCDDisplay(g_pusPattern);

#ifdef LCD_TEST
	thread = kthread_run(LCDTestThread, NULL, "lcd_test_thread");
#endif
}

bool LCD_PowerDown (void)
{
	LCDSerialWrite( LCD_COMMAND, LCD_DIS_OFF );		// Display Off
	LCDSerialWrite( LCD_COMMAND, LCD_SLP_IN );		// Sleep In

	gpio_set_value(gh532b_lcd_pin->reset_gpio, 1);
	
	bLcdInit = false;
	DBG_PRINTK("called");
	
	return true;	
}

static int h532bch_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int h532bch_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long h532bch_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	pixData xdata;
	static int config = 0;
	static int inv_lcd = 0;

	DBG_PRINTK("called");

	switch (cmd) {
	case H532BLCD_SET_POWER:
		ret = copy_from_user( (void *)&config, (void *)arg, sizeof(config));
		if(config) {
			DBG_PRINTK("TURN ON H532BLCD_POWER");
			LCD_PowerUp();
		}
		else {
			DBG_PRINTK("TURN OFF H532BLCD_POWER");
			LCD_PowerDown();
		}
		break;
	case H532BLCD_SET_INV_SCREEN:
		ret = copy_from_user( (void *)&config, (void *)arg, sizeof(config));
		if(config){
			DBG_PRINTK("TURN ON H532BLCD_INV_SCREEN");
			inv_lcd = 1;
			gbDispDirection = Direction_Inverse;	// Inverse Display
		}
		else {
			DBG_PRINTK("TURN OFF H532BLCD_INV_SCREEN");
			inv_lcd = 0;
			gbDispDirection = Direction_Normal;		// Normal Display
		}
		break;
	case H532BLCD_SET_BACKLIGHT:
		ret = copy_from_user( (void *)&config, (void *)arg, sizeof(config));
		if(config) {
		}
		else {
		}
		break;
	case H532BLCD_GET_POWER:
		DBG_PRINTK("H532BLCD_GET_POWER");
		config = (bLcdInit == true) ? 1 : 0;
		ret = copy_to_user((int *)arg, &config, sizeof(config));
		break;
	case H532BLCD_GET_INV_SCREEN:
		DBG_PRINTK("H532BLCD_GET_INV_SCREEN");
		ret = copy_to_user((int *)arg, &inv_lcd, sizeof(inv_lcd));
		break;
	case H532BLCD_GET_BACKLIGHT:
		ret = copy_to_user((int *)arg, &config, sizeof(config));	
		break;
	case H532BLCD_SEND_DATA:
		DBG_PRINTK("H532BLCD_SEND_DATA");
		ret = copy_from_user( (void *)&xdata, (void *)arg, sizeof(pixData));

		while( !bLcdInit ) 	return -1;

		// 	LCD_COLUMN_END * LCD_LINE_END * sizeof(unsigned short));
		memcpy( &gbDisplayData[0], xdata.data,  sizeof(xdata.data));
		while(gh532b_lcd_pin->wi_flag) msleep(10);
		gh532b_lcd_pin->wi_flag = 1;
		wake_up(&gh532b_lcd_pin->wq);

		break;
	}
	
	return ret;
}

static long h532bch_compat_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	switch (_IOC_NR(cmd)) {
		case _IOC_NR(H532BLCD_SET_POWER):
		case _IOC_NR(H532BLCD_SET_INV_SCREEN):
		case _IOC_NR(H532BLCD_SET_BACKLIGHT):
		case _IOC_NR(H532BLCD_GET_POWER):
		case _IOC_NR(H532BLCD_GET_INV_SCREEN):
		case _IOC_NR(H532BLCD_GET_BACKLIGHT):
			/* Fix up pointer size (usually 4 -> 8 in 32-on-64-bit case */
			if (_IOC_SIZE(cmd) == sizeof(compat_uptr_t)) {
				cmd &= ~IOCSIZE_MASK;
				cmd |= sizeof(void *) << IOCSIZE_SHIFT;
			}
			break;
	}

	return h532bch_ioctl(fp, cmd, arg);

}




static int h532b_spi_probe(struct spi_device *spi)
{
	int res;
	int ret;
	struct task_struct *thread;
	struct device_node *node;
	unsigned int gpio[4];

	DBG_PRINTK("called");

	gh532b_lcd_pin = kzalloc(sizeof(*gh532b_lcd_pin), GFP_KERNEL);
	if (!gh532b_lcd_pin) {
		DBG_PRINTK("No memory Error!");
		return ENOMEM;
	}

	g_pusPattern = kzalloc(LCD_COLUMN_END * LCD_LINE_END * 2, GFP_KERNEL);
	if (!g_pusPattern) {
		DBG_PRINTK("No memory Error!");
		return ENOMEM;
	}

	node = spi->dev.of_node;
	if (!node) {
		return -ENODEV;
	}
	h532blcd_ldo = regulator_get(&spi->dev, "vslp");
	if (IS_ERR(h532blcd_ldo)){
		ret = PTR_ERR(h532blcd_ldo);
		printk("hct-drv %s: regulator_get h532blcd_ldo fail=%d!!\n", __func__, ret);
	}else{
		ret = regulator_set_voltage(h532blcd_ldo, 3000000, 3000000);
//		ret = mtk_regulator_set_voltage(&fp_mt6370_ldo, 2800000, 2800000);
		if (ret < 0) {
			printk("hct-drv %s: regulator_set_voltage h532blcd_ldo fail!!\n", __func__);
			return -1;
		}
		ret = regulator_enable(h532blcd_ldo);
//		ret = mtk_regulator_enable(&fp_mt6370_ldo, true);
		if (ret < 0) {
			printk("hct-drv %s: regulator_enable enable h532blcd_ldo fail!!\n", __func__);
			return -1;
		}
	
		printk("hct-drv %s: regulator_enable enable h532blcd_ldo success!!\n", __func__);
	}

	gpio[0]= of_get_named_gpio_flags(node, "hct-spi-lcm,gpio-spi-miso", 0, NULL);
	gpio[1]= of_get_named_gpio_flags(node, "hct-spi-lcm,gpio-rst-en", 0, NULL);
	gpio[2]= of_get_named_gpio_flags(node, "hct-spi-lcm,gpio-power-ldo-en", 0, NULL);
	gpio[3]= of_get_named_gpio_flags(node, "hct-spi-lcm,gpio-power-lvshift-en", 0, NULL);

	DBG_PRINTK("gpio[0] = %d. gpio[1] = %d. gpio[2] = %d. gpio[3] = %d. ", gpio[0], gpio[1], gpio[2], gpio[3]);



	gh532b_lcd_pin->a0 = gpio[0];
	gh532b_lcd_pin->reset_gpio = gpio[1];
	gh532b_lcd_pin->power_ldo = gpio[2];
	gh532b_lcd_pin->power_level_shift_ldo = gpio[3];

	if(gpio_request(gh532b_lcd_pin->a0, "lcd532_a0"))
		DBG_PRINTK("lcd532_a0 gpio request error");
	if(gpio_request(gh532b_lcd_pin->reset_gpio, "lcd532_reset"))
		DBG_PRINTK("lcd532_reset gpio request error");
	if(gpio_request(gh532b_lcd_pin->power_ldo, "lcd532_power_ldo"))
		DBG_PRINTK("lcd532_power_ldo gpio request error");
	if(gpio_request(gh532b_lcd_pin->power_level_shift_ldo, "lcd532_power_level_shift_ldo"))
		DBG_PRINTK("power_level_shift_ldo gpio request error");

	DBG_PRINTK("called-2");


	gpio_direction_output(gh532b_lcd_pin->a0, 1);
	gpio_direction_output(gh532b_lcd_pin->reset_gpio, 1);
	gpio_direction_output(gh532b_lcd_pin->power_ldo, 1);
	gpio_direction_output(gh532b_lcd_pin->power_level_shift_ldo, 1);

	DBG_PRINTK("called-3");

	printk("hct-drv. h532b_spi_probe. power_ldo. = %d. ", gpio_get_value(gh532b_lcd_pin->power_ldo));
	printk("hct-drv. h532b_spi_probe. power_level_shift_ldo. = %d. ", gpio_get_value(gh532b_lcd_pin->power_level_shift_ldo));

	gpio_set_value(gh532b_lcd_pin->power_ldo, 1);
	gpio_set_value(gh532b_lcd_pin->power_level_shift_ldo, 0);
	//gpio_set_value(gh532b_lcd_pin->power_level_shift_ldo, 1);
	printk("hct-drv. h532b_spi_probe. power_ldo. = %d. ", gpio_get_value(gh532b_lcd_pin->power_ldo));
	printk("hct-drv. h532b_spi_probe. power_level_shift_ldo. = %d. ", gpio_get_value(gh532b_lcd_pin->power_level_shift_ldo));




	init_waitqueue_head(&gh532b_lcd_pin->wq);
	gh532b_lcd_pin->wi_flag = 0;
	thread = kthread_run(LCDDisplayThread, NULL, "lcd_thread");

	memcpy(&gspiDev, spi, sizeof(struct spi_device));

	gspiDev.mode = SPI_MODE_0;
	gspiDev.bits_per_word = 8;
	gspiDev.max_speed_hz = 10*1000*1000;

	DBG_PRINTK("Cell data : %s, bus[%d], cs[%d], mod[%d], %dkhz, %dbit,", 
		gspiDev.modalias, gspiDev.master->bus_num, gspiDev.chip_select, 
		gspiDev.mode, gspiDev.max_speed_hz/1000, gspiDev.bits_per_word);
	
	res = spi_setup(&gspiDev);
	if(res != 0) {
		DBG_PRINTK("can't spi_setup");
	}


	mt_spi_enable_master_clk(&gspiDev);



	LCD_PowerUp();

	return res;
}





static int h532b_spi_remove(struct spi_device *spi)
{
	struct h532b_lcd_pin *lcd_data;

	DBG_PRINTK("called");

	lcd_data = dev_get_drvdata(&spi->dev);
	if (lcd_data == NULL)
		return -ENODEV;

	dev_set_drvdata(&spi->dev, NULL);

	kfree(lcd_data);
	kfree(gh532b_lcd_pin);
	kfree(g_pusPattern);

	return 0;
}

static const struct of_device_id brllcd_of_match[] = {
	{
		.compatible = "hims,h532b_lcd",
	},
	{},
};
MODULE_DEVICE_TABLE(of, brllcd_of_match);

// SPI driver
static struct spi_driver h532b_spi = {
	.driver = {
		.name = "h532b_spi1",
		.bus  = &spi_bus_type,
		.owner=	THIS_MODULE,
		.of_match_table = brllcd_of_match,
	},
	.probe = h532b_spi_probe,
	.remove = h532b_spi_remove,
};




// Character device driver
static const struct file_operations h532bch_file_ops = {
	.owner = THIS_MODULE,
	.open = h532bch_open,
	.release = h532bch_release,
	.unlocked_ioctl = h532bch_ioctl,
	.compat_ioctl = h532bch_compat_ioctl,
	.llseek = no_llseek,
};


static int __init h532b_lcd_init(void)
{
	int ret;
	struct device *dev = NULL;

	/* init CHDEV */
	h532blcd_major = register_chrdev(0, DEVICE_NAME, &h532bch_file_ops);
	if (h532blcd_major<0){
		DBG_PRINTK("h532b_lcd failed to register a major number");
		return h532blcd_major;
	}
	DBG_PRINTK("h532b_lcd : registered correctly with major number %d",
		h532blcd_major);

	h532blcd_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(h532blcd_class)) {
		DBG_PRINTK("Failed to create the device -- h532b_lcd");
		unregister_chrdev(h532blcd_major, DEVICE_NAME);
		ret = PTR_ERR(h532blcd_class);
		goto err;
	}
	DBG_PRINTK("h532b_lcd: device class created correctly");
	
	dev = device_create(h532blcd_class, NULL, MKDEV(h532blcd_major, 0), NULL,
		DEVICE_NAME);
	if( !dev ) {
		DBG_PRINTK("h532blcd_init: device_create fail");
		goto err;		
	}
	DBG_PRINTK("h532b_lcd: device created correctly");
	/* init CHDEV END*/


    printk(KERN_ERR "%s:\n", __func__);
    return spi_register_driver(&h532b_spi);


err:
	return ret;
}

static void __exit h532b_lcd_exit(void)
{
    printk(KERN_ERR "%s:\n", __func__);

	spi_unregister_driver(&h532b_spi);
	device_destroy(h532blcd_class, MKDEV(h532blcd_major, 0));
	class_unregister(h532blcd_class);
	class_destroy(h532blcd_class);
	unregister_chrdev(h532blcd_major, DEVICE_NAME);

	
}

late_initcall(h532b_lcd_init);
module_exit(h532b_lcd_exit);
MODULE_DESCRIPTION("h532b LCD Driver");
MODULE_LICENSE("GPL v2");
