/*
 * Copyright (C) 2017 aidualcam Inc.
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
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
//#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/types.h>
#include "linux/input.h"

#define aidualcam_3D_CAL_LENGTH     4096

static u8 aidualcam_cal_data[aidualcam_3D_CAL_LENGTH];
static u32 aidualcam_cal_data_len = 0;

static ssize_t aidualcam_eepromcal_write(struct file *filp, const char __user *buf, size_t len, loff_t *off)
{
	char info[32];

	if (copy_from_user(info, buf, 32)) {
		pr_err("%s: copy failed", __func__);
		return -1;
	}
	printk("aidualcam Cal: write \n");
	return len;
}

static ssize_t aidualcam_eepromcal_read(struct file *filep, char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	printk("aidualcam Cal: read \n");
	return simple_read_from_buffer(ubuf, count, ppos, (void*)aidualcam_cal_data, 
	aidualcam_cal_data_len);
}


static const struct file_operations aidualcam_cal_fops = {
	.owner = THIS_MODULE,
	.write = aidualcam_eepromcal_write,
	.read = aidualcam_eepromcal_read,
};

static struct miscdevice aidualcam_cal_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "aidualcam_eeprom_cal",
	.fops  = &aidualcam_cal_fops,
};

void aidualcam_cal_init(void* p_data, int len)
{
	if (NULL != p_data && 0 != len && len < aidualcam_3D_CAL_LENGTH) {
		aidualcam_cal_data_len = len;
		memcpy((void*)aidualcam_cal_data,  p_data, len);
	}
}

static int __init aiworks_dualcam_cal_int(void)
{
	return misc_register(&aidualcam_cal_misc);
}

static void __exit aiworks_dualcam_cal_exit(void)
{
	misc_deregister(&aidualcam_cal_misc);
}

module_init(aiworks_dualcam_cal_int);
module_exit(aiworks_dualcam_cal_exit);

MODULE_AUTHOR("liaokesen@360os.com");
MODULE_LICENSE("GPL");
