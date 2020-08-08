/*
 * Copyright (C) 2015 MediaTek Inc.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/of.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include "flashlight.h"
#include "flashlight-dt.h"
#include "flashlight-core.h"


/* device tree should be defined in flashlight-dt.h */
#ifndef AW36428_DTNAME
#define AW36428_DTNAME "mediatek,flashlights_aw36428"
#endif
#ifndef AW36428_DTNAME_I2C
#define AW36428_DTNAME_I2C "mediatek,flashlights_aw36428_i2c"
#endif

#define AW36428_NAME "flashlights-aw36428"
/* define registers */
#define AW36428_REG_CHIP_ID (0x00)  /*chip id is 0x17*/
#define AW36428_REG_ENABLE           (0x01)
#define AW36428_REG_IVFM (0x02)
#define AW36428_MASK_ENABLE_LED1     (0x01)
#define AW36428_DISABLE              (0x00)
#define AW36428_ENABLE_LED1          (0x01)
#define AW36428_ENABLE_LED1_TORCH    (0x21)
#define AW36428_ENABLE_LED1_FLASH    (0x31)

#define AW36428_REG_FLASH_LEVEL_LED1 (0x03)
#define AW36428_REG_TORCH_LEVEL_LED1 (0x04)


#define AW36428_REG_TIMING_CONF      (0x08)
#define AW36428_TORCH_RAMP_TIME      (0x10)
#define AW36428_FLASH_TIMEOUT        (0x0A)

/* define channel, level */
#define AW36428_CHANNEL_NUM          1
#define AW36428_CHANNEL_CH1          0

#define AW36428_LEVEL_NUM            26
#define AW36428_LEVEL_TORCH          7

/* define mutex and work queue */
static DEFINE_MUTEX(aw36428_mutex);
static struct work_struct aw36428_work_ch1;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw36428_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t aw36428_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t aw36428_debug_show(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t aw36428_debug_store(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);
static DEVICE_ATTR(reg, 0660, aw36428_get_reg,  aw36428_set_reg);
static DEVICE_ATTR(debug, 0660, aw36428_debug_show,  aw36428_debug_store);
/* define usage count */
static int use_count;

/* define i2c */
static struct i2c_client *aw36428_i2c_client;

/* platform data */
struct aw36428_platform_data {
	u8 torch_pin_enable;         /* 1: TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;      /* 1: TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable; /* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;       /* 1: STROBE Input disabled */
	u8 vout_mode_enable;         /* 1: Voltage Out Mode enable */
};

/* aw36428 chip data */
struct aw36428_chip_data {
	struct i2c_client *client;
	struct aw36428_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};


/******************************************************************************
 * aw36428 operations
 *****************************************************************************/
static const unsigned char aw36428_torch_level[AW36428_LEVEL_NUM] = {
    0x06, 0x0F, 0x17, 0x1F, 0x27, 0x2F, 0xCD, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //0xCD <-> 0xEF

static const unsigned char aw36428_flash_level[AW36428_LEVEL_NUM] = {
	0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x10, 0x14, 0x19,
	0x1D, 0x21, 0x25, 0x2A, 0x2E, 0x32, 0x37, 0x3B, 0x3F, 0x43,
	0x48, 0x4C, 0x50, 0x54, 0x59, 0x5D};

static volatile unsigned char aw36428_reg_enable;
static volatile int aw36428_level_ch1 = -1;

static int aw36428_is_torch(int level)
{
	if (level >= AW36428_LEVEL_TORCH)
		return -1;

	return 0;
}

static int aw36428_verify_level(int level)
{
	if (level < 0)
		level = 0;
	else if (level >= AW36428_LEVEL_NUM)
		level = AW36428_LEVEL_NUM - 1;

	return level;
}

/* i2c wrapper function */
static int aw36428_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	struct aw36428_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		printk("failed writing at 0x%02x\n", reg);

	return ret;
}

static int aw36428_read_reg(struct i2c_client *client, u8 reg)
{
	int val;
	struct aw36428_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (val < 0)
		printk("failed read at 0x%02x\n", reg);

	return val;
}

/* flashlight enable function */
static int aw36428_enable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36428_REG_ENABLE;
	if (!aw36428_is_torch(aw36428_level_ch1)) {
		/* torch mode */
		aw36428_reg_enable |= AW36428_ENABLE_LED1_TORCH;
		printk("hct_add torch mode");
	} else {
		/* flash mode */
		aw36428_reg_enable |= AW36428_ENABLE_LED1_FLASH;
		printk("hct_add flash mode");
	}

	val = aw36428_reg_enable;

	return aw36428_write_reg(aw36428_i2c_client, reg, val);
}



/* flashlight disable function */
static int aw36428_disable_ch1(void)
{
	unsigned char reg, val;

	reg = AW36428_REG_ENABLE;
	aw36428_reg_enable &= (~AW36428_ENABLE_LED1_FLASH);
	val = aw36428_reg_enable;

	return aw36428_write_reg(aw36428_i2c_client, reg, val);
}

static int aw36428_enable(int channel)
{
	if (channel == AW36428_CHANNEL_CH1)
		aw36428_enable_ch1();
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}

static int aw36428_disable(int channel)
{
	if (channel == AW36428_CHANNEL_CH1)
		aw36428_disable_ch1();
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}

/* set flashlight level */
static int aw36428_set_level_ch1(int level)
{
	int ret;
	unsigned char reg, val;

	level = aw36428_verify_level(level);
	printk("hct_add [%s] level %d \n",__func__,level);
	/* set torch brightness level */
	reg = AW36428_REG_TORCH_LEVEL_LED1;
	val = aw36428_torch_level[level];
	ret = aw36428_write_reg(aw36428_i2c_client, reg, val);

	aw36428_level_ch1 = level;

	/* set flash brightness level */
	reg = AW36428_REG_FLASH_LEVEL_LED1;
	val = aw36428_flash_level[level];
	ret = aw36428_write_reg(aw36428_i2c_client, reg, val);

	return ret;
}



static int aw36428_set_level(int channel, int level)
{
	if (channel == AW36428_CHANNEL_CH1)
		aw36428_set_level_ch1(level);
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}

/* flashlight init */
int aw36428_init(void)
{
	int ret;
	unsigned char reg, val;

    msleep(2);

	/* clear enable register */
	reg = AW36428_REG_ENABLE;
	val = AW36428_DISABLE;
	ret = aw36428_write_reg(aw36428_i2c_client, reg, val);

	aw36428_reg_enable = val;

	/* set torch current ramp time and flash timeout */
	reg = AW36428_REG_TIMING_CONF;
	val = AW36428_TORCH_RAMP_TIME | AW36428_FLASH_TIMEOUT;
	ret = aw36428_write_reg(aw36428_i2c_client, reg, val);

	return ret;
}

/* flashlight uninit */
int aw36428_uninit(void)
{
	aw36428_disable(AW36428_CHANNEL_CH1);

	//reset all registers add by jiangkai
	aw36428_write_reg(aw36428_i2c_client, AW36428_REG_CHIP_ID, 0x55);
	msleep(5);
	
	return 0;
}


/******************************************************************************
 * Timer and work queue
 *****************************************************************************/
static struct hrtimer aw36428_timer_ch1;
static unsigned int aw36428_timeout_ms[AW36428_CHANNEL_NUM];

static void aw36428_work_disable_ch1(struct work_struct *data)
{
	printk("ht work queue callback\n");
	aw36428_disable_ch1();
}

static enum hrtimer_restart aw36428_timer_func_ch1(struct hrtimer *timer)
{
	schedule_work(&aw36428_work_ch1);
	return HRTIMER_NORESTART;
}


int aw36428_timer_start(int channel, ktime_t ktime)
{
	if (channel == AW36428_CHANNEL_CH1)
		hrtimer_start(&aw36428_timer_ch1, ktime, HRTIMER_MODE_REL);
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}

int aw36428_timer_cancel(int channel)
{
	if (channel == AW36428_CHANNEL_CH1)
		hrtimer_cancel(&aw36428_timer_ch1);
	else {
		printk("Error channel\n");
		return -1;
	}

	return 0;
}


/******************************************************************************
 * Flashlight operations
 *****************************************************************************/
static int aw36428_ioctl(unsigned int cmd, unsigned long arg)
{
	struct flashlight_dev_arg *fl_arg;
	int channel;
	ktime_t ktime;

	fl_arg = (struct flashlight_dev_arg *)arg;
	channel = fl_arg->channel;

	/* verify channel */
	if (channel < 0 || channel >= AW36428_CHANNEL_NUM) {
		printk("Failed with error channel\n");
		return -EINVAL;
	}

	switch (cmd) {
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		printk("FLASH_IOC_SET_TIME_OUT_TIME_MS(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36428_timeout_ms[channel] = fl_arg->arg;
		break;

	case FLASH_IOC_SET_DUTY:
		printk("FLASH_IOC_SET_DUTY(%d): %d\n",
				channel, (int)fl_arg->arg);
		aw36428_set_level(channel, fl_arg->arg);
		break;

	case FLASH_IOC_SET_ONOFF:
		printk("FLASH_IOC_SET_ONOFF(%d): %d\n",
				channel, (int)fl_arg->arg);
		if (fl_arg->arg == 1) {
			if (aw36428_timeout_ms[channel]) {
				ktime = ktime_set(aw36428_timeout_ms[channel] / 1000,
						(aw36428_timeout_ms[channel] % 1000) * 1000000);
				aw36428_timer_start(channel, ktime);
			}
			aw36428_enable(channel);
		} else {
			aw36428_disable(channel);
			aw36428_timer_cancel(channel);
		}
		break;

	default:
		printk("No such command and arg(%d): (%d, %d)\n",
				channel, _IOC_NR(cmd), (int)fl_arg->arg);
		return -ENOTTY;
	}

	return 0;
}

static int aw36428_open(void)
{
	/* Actual behavior move to set driver function since power saving issue */
	return 0;
}

static int aw36428_release(void)
{
	/* uninit chip and clear usage count */
	mutex_lock(&aw36428_mutex);
	use_count--;
	if (!use_count)
		aw36428_uninit();
	if (use_count < 0)
		use_count = 0;
	mutex_unlock(&aw36428_mutex);

	printk("Release: %d\n", use_count);

	return 0;
}

static int aw36428_set_driver(int set)
{
	/* init chip and set usage count */
	mutex_lock(&aw36428_mutex);
	if (!use_count)
		aw36428_init();
	use_count++;
	mutex_unlock(&aw36428_mutex);

	printk("Set driver: %d\n", use_count);

	return 0;
}

static ssize_t aw36428_strobe_store(struct flashlight_arg arg)
{
	aw36428_set_driver(1);
	aw36428_set_level(arg.ct, arg.level);
	aw36428_enable(arg.ct);
	msleep(arg.dur);
	aw36428_disable(arg.ct);
	aw36428_set_driver(0);

	return 0;
}

static struct flashlight_operations aw36428_ops = {
	aw36428_open,
	aw36428_release,
	aw36428_ioctl,
	aw36428_strobe_store,
	aw36428_set_driver
};


/******************************************************************************
 * I2C device and driver
 *****************************************************************************/
static int aw36428_chip_init(struct aw36428_chip_data *chip)
{
	/* NOTE: Chip initialication move to "set driver" operation for power saving issue.
	 * aw36428_init();
	 */

	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AW36428 Debug file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw36428_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	unsigned char i;
	ssize_t len = 0;
	for(i=0;i<=0x04;i++)
	{
		reg_val = aw36428_read_reg(aw36428_i2c_client,i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg0x%2X = 0x%2X \n", i,reg_val);
	}
    /*read reg 0x0B*/
	reg_val = aw36428_read_reg(aw36428_i2c_client,0x0B);
	len += snprintf(buf+len, PAGE_SIZE-len, "reg0x0B = 0x%2X \n",reg_val);
    /*read reg 0x0F*/
	reg_val = aw36428_read_reg(aw36428_i2c_client,0x0F);
	len += snprintf(buf+len, PAGE_SIZE-len, "reg0x0F = 0x%2X \n",reg_val);
    /*read reg 0x10*/
	reg_val = aw36428_read_reg(aw36428_i2c_client,0x10);
	len += snprintf(buf+len, PAGE_SIZE-len, "reg0x10 = 0x%2X \n",reg_val);
	len += snprintf(buf+len, PAGE_SIZE-len, "\r\n");
	return len;
}

static ssize_t aw36428_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		//i2c_write_reg(databuf[0],databuf[1]);
		aw36428_write_reg(aw36428_i2c_client,databuf[0],databuf[1]);
	}
	return len;
}



static int aw36428_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);
	err += device_create_file(dev, &dev_attr_debug);

	return err;
}

static int aw36428_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw36428_chip_data *chip;
	struct aw36428_platform_data *pdata = client->dev.platform_data;
	int err;

	printk("Probe start.\n");

	/* check i2c */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("Failed to check i2c functionality.\n");
		err = -ENODEV;
		goto err_out;
	}

	/* init chip private data */
	chip = kzalloc(sizeof(struct aw36428_chip_data), GFP_KERNEL);
	if (!chip) {
		err = -ENOMEM;
		goto err_out;
	}
	chip->client = client;

	/* init platform data */
	if (!pdata) {
		printk("Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct aw36428_platform_data), GFP_KERNEL);
		if (!pdata) {
			err = -ENOMEM;
			goto err_init_pdata;
		}
		chip->no_pdata = 1;
	}
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
	aw36428_i2c_client = client;

	/* init mutex and spinlock */
	mutex_init(&chip->lock);

	/* init work queue */
	INIT_WORK(&aw36428_work_ch1, aw36428_work_disable_ch1);

	/* init timer */
	hrtimer_init(&aw36428_timer_ch1, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw36428_timer_ch1.function = aw36428_timer_func_ch1;
	aw36428_timeout_ms[AW36428_CHANNEL_CH1] = 100;

	/* init chip hw */
	aw36428_chip_init(chip);

	/* register flashlight operations */
	if (flashlight_dev_register(AW36428_NAME, &aw36428_ops)) {
		printk("Failed to register flashlight device.\n");
		err = -EFAULT;
		goto err_free;
	}

	/* clear usage count */
	use_count = 0;

	aw36428_create_sysfs(client);

	printk("Probe done.\n");

	return 0;

err_free:
	kfree(chip->pdata);
err_init_pdata:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
err_out:
	return err;
}

int aw36428_open_by_cam3(void)
{
	aw36428_set_driver(1);
	aw36428_set_level(AW36428_CHANNEL_CH1, 6);

	aw36428_enable(AW36428_CHANNEL_CH1);

	return 0;
}


int aw36428_close_by_cam3(void)
{
	aw36428_disable(AW36428_CHANNEL_CH1);
	
	//aw36428_set_driver(0);
	aw36428_release();

	return 0;
}

static ssize_t aw36428_debug_show(struct device* cd,struct device_attribute *attr, char* buf)
{
	return sprintf(buf, "aw36428 use_count %s\n", use_count);

}

static ssize_t aw36428_debug_store(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{

	printk("aw36428_debug store.\n");
	if(!strncmp(buf, "on", 2))
	{
		printk("aw36428_debug contrl on. %s\n", buf);
		aw36428_open_by_cam3();
	}
	else if(!strncmp(buf, "off", 3))
	{
		printk("aw36428_debug contrl off. %s\n", buf);
		aw36428_close_by_cam3();	
	}
	else
	{
		printk("aw36428_debug contrl error: %s\n", buf);
	}

	return len;
}

static int aw36428_i2c_remove(struct i2c_client *client)
{
	struct aw36428_chip_data *chip = i2c_get_clientdata(client);

	printk("Remove start.\n");

	/* flush work queue */
	flush_work(&aw36428_work_ch1);

	/* unregister flashlight operations */
	flashlight_dev_unregister(AW36428_NAME);

	/* free resource */
	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);

	printk("Remove done.\n");

	return 0;
}

static const struct i2c_device_id aw36428_i2c_id[] = {
	{AW36428_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id aw36428_i2c_of_match[] = {
	{.compatible = AW36428_DTNAME_I2C},
	{},
};
#endif

static struct i2c_driver aw36428_i2c_driver = {
	.driver = {
		   .name = AW36428_NAME,
#ifdef CONFIG_OF
		   .of_match_table = aw36428_i2c_of_match,
#endif
		   },
	.probe = aw36428_i2c_probe,
	.remove = aw36428_i2c_remove,
	.id_table = aw36428_i2c_id,
};


/******************************************************************************
 * Platform device and driver
 *****************************************************************************/
static int aw36428_probe(struct platform_device *dev)
{
	printk("Probe start.\n");

	if (i2c_add_driver(&aw36428_i2c_driver)) {
		printk("Failed to add i2c driver.\n");
		return -1;
	}

	printk("Probe done.\n");

	return 0;
}

static int aw36428_remove(struct platform_device *dev)
{
	printk("Remove start.\n");

	i2c_del_driver(&aw36428_i2c_driver);

	printk("Remove done.\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw36428_of_match[] = {
	{.compatible = AW36428_DTNAME},
	{},
};
MODULE_DEVICE_TABLE(of, aw36428_of_match);
#else
static struct platform_device aw36428_platform_device[] = {
	{
		.name = AW36428_NAME,
		.id = 0,
		.dev = {}
	},
	{}
};
MODULE_DEVICE_TABLE(platform, aw36428_platform_device);
#endif

static struct platform_driver aw36428_platform_driver = {
	.probe = aw36428_probe,
	.remove = aw36428_remove,
	.driver = {
		.name = AW36428_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw36428_of_match,
#endif
	},
};

static int __init flashlight_aw36428_init(void)
{
	int ret;

	printk("flashlight_aw36428-Init start.\n");

#ifndef CONFIG_OF
	ret = platform_device_register(&aw36428_platform_device);
	if (ret) {
		printk("Failed to register platform device\n");
		return ret;
	}
#endif

	ret = platform_driver_register(&aw36428_platform_driver);
	if (ret) {
		printk("Failed to register platform driver\n");
		return ret;
	}

	printk("flashlight_aw36428 Init done.\n");

	return 0;
}

static void __exit flashlight_aw36428_exit(void)
{
	printk("flashlight_aw36428-Exit start.\n");

	platform_driver_unregister(&aw36428_platform_driver);

	printk("flashlight_aw36428 Exit done.\n");
}


module_init(flashlight_aw36428_init);
module_exit(flashlight_aw36428_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joseph <zhangzetao@awinic.com.cn>");
MODULE_DESCRIPTION("AW Flashlight AW36428 Driver");

