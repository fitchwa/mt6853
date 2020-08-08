/* 
 * drivers/input/touchscreen/CST2XX.c
 *
 * hynitron TouchScreen driver.
 *
 * Copyright (c) 2015  hynitron
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *  1.0		    2015-10-12		    Tim
 *
 * note: only support mulititouch
 */

#include <linux/string.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>

#include "hyn_cst3xx_K1336A_fw_v2.h"
#include "hyn_cst3xx_K1336A_fw.h"



/**********************************
config macro define
*********************************/
//#define HYN_GESTURE
//#define HYN_UPDATE_FIRMWARE_ONLINE
//#define HYN_UPDATE_FIRMWARE_FORCE          //update firmware online force
//#define HYN_UPDATE_FIRMWARE_POEWRON_ENABLE
#define CONFIG_TP_ESD_PROTECT
#define ANDROID_TOOL_SURPORT
#define HYN_SYSFS_NODE_EN
#define ICS_SLOT_REPORT
#define REPORT_XY_SWAP
#define SLEEP_CLEAR_POINT
//#define HIGH_SPEED_IIC_TRANSFER
#define TRANSACTION_LENGTH_LIMITED
//#define TPD_HAVE_BUTTON
//#define GTP_HAVE_TOUCH_KEY

#define TPD_REVERT_XY				0
#define TPD_REVERT_X				0
#define TPD_REVERT_Y			    0

#define GTP_DEBUG_FUNC_ON  1

#define GTP_DEBUG_FUNC()               do{\
                                         if(GTP_DEBUG_FUNC_ON)\
                                         printk("<<GTP-FUNC>> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)

//#define GTP_RST_PORT    0
//#define GTP_INT_PORT    1


//#define HYN_UPDATE_FIRMWARE_TEST
#ifdef HYN_UPDATE_FIRMWARE_TEST
static struct delayed_work hyn_update_firmware_work;
static struct workqueue_struct *hyn_update_firmware_workqueue = NULL;
static unsigned update_test_count=0;
static unsigned update_test_fail=0;
static unsigned update_test_running_on=1;
#endif

#ifdef HYN_GESTURE
//static int hyn_gesture_flag = 0;
static int hyn_lcd_flag = 0;
//static struct input_dev *hyn_power_idev;
//static char hyn_gesture_c = 0;
static int tpd_halt=0;

#if 0
#define  HYN_TPD_GES_COUNT  14
static int tpd_keys_gesture[HYN_TPD_GES_COUNT] ={KEY_POWER, \
												TPD_GES_KEY_DOUBLECLICK,\
												TPD_GES_KEY_LEFT,\
												TPD_GES_KEY_RIGHT,\
												TPD_GES_KEY_UP,\
												TPD_GES_KEY_DOWN,\
												TPD_GES_KEY_O,\
												TPD_GES_KEY_W,  \
												TPD_GES_KEY_M,\
												TPD_GES_KEY_E,\
												TPD_GES_KEY_C,\
												TPD_GES_KEY_S,\
												TPD_GES_KEY_V,\
												TPD_GES_KEY_Z};
#endif

#endif 

#ifdef CONFIG_TP_ESD_PROTECT
#define SWITCH_ESD_OFF                  0
#define SWITCH_ESD_ON                   1
static struct workqueue_struct *cst3xx_esd_workqueue;
#endif

#ifdef ANDROID_TOOL_SURPORT
static  unsigned short g_unnormal_mode = 0;
static unsigned short g_cst3xx_tx = 10;
static unsigned short g_cst3xx_rx = 18;

static  unsigned short hyn_tp_test_flag=0;
static DECLARE_WAIT_QUEUE_HEAD(dbg_waiter);
static int tpd_flag = 0;

#ifdef HYN_SYSFS_NODE_EN
static struct mutex g_device_mutex;
static DEFINE_MUTEX(g_device_mutex);
static struct kobject *k_obj = NULL;
#endif


#endif

struct hynitron_fw_array {
	const char* name;
	const int project_id;
	unsigned char *fw;
} cst3xx_fw_grp[] = {
    { "hyn_cst3xx_K1336A_fw",      0x2843,  cst3_fw },	// default
	{ "hyn_cst3xx_K1336A_fw_v2",   0x0501,  cst3_fw_v2 } //ÐÂÄ£×é
    
};

static unsigned char  report_flag = 0;
static unsigned char  report_finger_flag = 0;
//static unsigned char  key_index = 0xFF;
static unsigned int   g_cst3xx_ic_version  = 0;
static unsigned int   g_cst3xx_ic_checksum = 0;
static unsigned int   g_cst3xx_ic_checkcode = 0;
static unsigned int   g_cst3xx_ic_project_id  = 0;
static unsigned int   g_cst3xx_ic_type  = 0;

static struct workqueue_struct *cst3xx_wq;
//static unsigned int  g_irq_pin = 924;
struct i2c_client *g_i2c_client = NULL;
struct input_dev  *g_input_dev = NULL;

static int cst3xx_firmware_info(struct i2c_client * client);

#ifdef 	HYN_UPDATE_FIRMWARE_ONLINE
static int cst3xx_boot_update_fw(struct i2c_client   *client,  unsigned char *pdata);
static int cst3xx_update_firmware(struct i2c_client * client, const unsigned char *pdata);
static int hyn_boot_update_fw(struct i2c_client * client);
static unsigned char *pcst3xx_update_firmware =(unsigned char *)cst3_fw ; //the updating firmware
#endif
#ifdef CONFIG_TP_ESD_PROTECT
void cst3xx_esd_switch(s32 on);
#endif

#ifdef ICS_SLOT_REPORT
#include <linux/input/mt.h> // Protocol B, for input_mt_slot(), input_mt_init_slot()
#endif

struct cst3xx_ts_platform_data{
	int irq_gpio;
	u32 irq_gpio_flags;
	int reset_gpio;
	u32 reset_gpio_flags;
	const char *product1_id;
	const char *product2_id;
	char match_product_id;  //0 no match; 1 match product1-id; 2 match product2-id
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
//erobbing add for check if overturn axis of x and y
	bool x_overturn;
	bool y_overturn;
//end
	bool have_touch_key;
	bool i2c_pull_up;
	bool enable_power_off;
	bool enable_slot_report;
	bool enable_esd;

};

struct cst3xx_ts_data{
    struct cst3xx_ts_platform_data *pdata;
    spinlock_t irq_lock;
    struct i2c_client *client;
    struct input_dev  *input_dev;
    struct hrtimer timer;
    struct work_struct  work;
    s32 irq_is_disable;
    s32 use_irq;
    u16 abs_x_max;
    u16 abs_y_max;
    u8  max_touch_num;
    u8  int_trigger_type;
    u8  work_mode;
    u8  enter_update;
 
    spinlock_t esd_lock;
    u8  esd_running;
    s32 clk_tick_cnt;

    u8  sensor_id;
    u16 ic_version;
	u16 fw_version;
	u16 config_version;
	u16 ic_config_version;
	u8	fw_updating;
    //struct ts_func_test_device ts_test_dev;

    bool power_on;
    struct mutex lock;
    struct regulator *avdd;
//    struct regulator *vcc_i2c;

#if defined(CONFIG_FB)
    struct notifier_block fb_notif;
    struct work_struct resume_work;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;

	dev_t			devt;//Erobbing add for distinguish open glove mode or not
	bool     in_glove_mode;

};
enum {
    POWER_OFF = 0,
    POWER_ON,
    POWER_SUSPEND,
    POWER_WAKE,
};
struct cst3xx_ts_data *g_ts=NULL;
#define MODE_SUSPEND   0
#define MODE_RESUME    1

#ifdef TPD_HAVE_BUTTON
#define TPD_KEY_COUNT	2
#define TPD_KEYS		{KEY_MENU,KEY_BACK}
/* {button_center_x, button_center_y, button_width, button_height*/
#define TPD_KEYS_DIM	{{180, 1360, 60, 60},{540, 1360, 60, 60}}
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#ifdef GTP_HAVE_TOUCH_KEY
#define TPD_KEY_COUNT	2
//#define TPD_KEYS		{KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS		{KEY_MENU,KEY_BACK}
const u16 touch_key_array[] = TPD_KEYS;
//#define GTP_MAX_KEY_NUM ( sizeof( touch_key_array )/sizeof( touch_key_array[0] ) )
#endif

#define INPUT_DEV_PHYS  			"input/ts"
#define TPD_DRIVER_NAME			    "hynitron-ts"
#define TPD_MAX_FINGERS			    5
#define TPD_MAX_X				    1080
#define TPD_MAX_Y				    2340
#define CST3XX_I2C_ADDR				0x1A
#define CST3XX_I2C_ADDR_BOOT		0x1A
#define I2C_BUS_NUMBER              1 //IIC bus num for mtk

#define printk(fmt,arg...)  printk(KERN_ERR" linc HYN:[LINE=%d]"fmt,__LINE__, ##arg)							
//#define printk(fmt,arg...)  printk(" linc HYN:[LINE=%d]"fmt,__LINE__, ##arg)

static int cst3xx_suspend(struct cst3xx_ts_data *ts);
static int cst3xx_resume(struct cst3xx_ts_data *ts);

#if defined(CONFIG_PM)
static int cst3xx_pm_suspend(struct device *dev);
static int cst3xx_pm_resume(struct device *dev);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void cst3xx_ts_early_suspend(struct early_suspend *h);
static void cst3xx_ts_late_resume(struct early_suspend *h);
#endif

void cst3xx_irq_enable(struct cst3xx_ts_data *ts);
void cst3xx_irq_disable(struct cst3xx_ts_data *ts);

#ifdef HYN_GESTURE
static u16 hyn_ges_wakeup_switch = 1;
static unsigned char hyn_gesture_c = 0;

#if 0
static char  hyn_ges_wakeup_flag = 0;
static unsigned char hyn_ges_cmd[48] = {0};
static unsigned int hyn_ges_num = 0;
static struct proc_dir_entry   *tp_proc_file = NULL;

static ssize_t proc_gesture_switch_write(struct file *flip,const char __user *buf, size_t count, loff_t *data)
{
	if (copy_from_user(&hyn_ges_wakeup_flag, buf, count))
		return -EFAULT;
	printk(" linc -liuhx --%s---%c\n",__func__,hyn_ges_wakeup_flag);
	switch(hyn_ges_wakeup_flag)
	{
		case '0':
			hyn_ges_wakeup_switch = 0;
			break;
		default:
			hyn_ges_wakeup_switch = 1;
			break;
	}
	return count;
}

static ssize_t proc_gesture_switch_read(struct file *flip, char __user *user_buf, size_t count, loff_t *ppos)
{
	int len = 0;

	printk(" linc --%s--\n",__func__);
	len = simple_read_from_buffer(user_buf, count, ppos, &hyn_ges_wakeup_flag, 1);

	return len;
}

static ssize_t proc_gesture_data_write(struct file *flip,const char __user *buffer, size_t count, loff_t *data)
{

	int i =0;

	if(count > 48)
		count =48;

	if (copy_from_user(hyn_ges_cmd, buffer, count))
		return -EFAULT;

	hyn_ges_num = 0;
	//memcpy(hyn_ges_cmd, buffer, count);
	for(i = 0; i < count; i++){
		if(buffer[i] != 0){
			//--hyn_ges_cmd[hyn_ges_num ++] = buffer[i];
			printk(" linc  hyn_ges_cmd[%d] :0x%x\n",i,hyn_ges_cmd[i]);
		}

	//DBG("buffer[%d] = %x\n",buffer[i]);
	hyn_ges_num = count;
	}
	
	return count;

}

static ssize_t proc_gesture_data_read(struct file *flip, char __user *user_buf, size_t count, loff_t *ppos)
{
	int len = 0;

	for (len = 0; len < hyn_ges_num; len++)
		printk(" linc buff[%d] = %c,", len, hyn_ges_cmd[len]);
	printk(" linc \n");
	return len;
}

static ssize_t proc_gesture_write(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	printk(" linc --%s--\n",__func__);

	sscanf(buffer, "%x", (unsigned int *)&hyn_gesture_c);

	return count;

}

static int proc_gesture_show(struct seq_file *m, void *v)
{
	printk(" linc --%s--\n",__func__);

	seq_printf(m, "%#x\n", hyn_gesture_c);
	return 0;

}

static int proc_gesture_open(struct inode *inode, struct file *file)
{
        printk(" linc --%s--\n",__func__);
	return single_open(file, proc_gesture_show, NULL);
}

static const struct file_operations proc_gesture_fops = {
	.owner		= THIS_MODULE,
	.open		= proc_gesture_open,
	.read		= seq_read,
	.write		= proc_gesture_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations proc_gesture_switch_fops = {
	.write	=	proc_gesture_switch_write,
	.read	=	proc_gesture_switch_read,
};
static const struct file_operations proc_gesture_data_fops = {
	.write	=	proc_gesture_data_write,
	.read	=	proc_gesture_data_read,
};

static int gesture_proc_init(void)
{

   int rc;

   tp_proc_file = proc_create("gesture", 0777, NULL,&proc_gesture_fops);
   if (tp_proc_file == NULL)
   {
      rc = -ENOMEM;
      goto err_del_gesture;
   }

   tp_proc_file = proc_create_data("gesture_switch", 0777, NULL,&proc_gesture_switch_fops,NULL);
   if (tp_proc_file == NULL)
   {
      rc = -ENOMEM;
      goto err_del_gesture_switch;
   }

   tp_proc_file = proc_create_data("gesture_data", 0777, NULL,&proc_gesture_data_fops,NULL);
   if (tp_proc_file == NULL)
   {
      rc = -ENOMEM;
      goto err_del_gesture_data;
   }

   return 0;

err_del_gesture_data:
   remove_proc_entry("gesture_data", NULL);
err_del_gesture_switch:
   remove_proc_entry("gesture_switch", NULL);
err_del_gesture:
   remove_proc_entry("gesture", NULL);
   return rc;
}

#endif
#endif

#ifdef HIGH_SPEED_IIC_TRANSFER
static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len) 
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags |= I2C_M_RD;
	msg.addr   = client->addr;
	msg.len    = len;
	msg.buf    = buf;

	while (retries < 2) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		retries++;
	}

	return ret;
}

/*******************************************************
Function:
    read data from register.
Input:
    buf: first two byte is register addr, then read data store into buf
    len: length of data that to read
Output:
    success: number of messages
    fail:	negative errno
*******************************************************/
static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len) 
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = buf;

	msgs[1].flags |= I2C_M_RD;
	msgs[1].addr   = client->addr;
	msgs[1].len    = len;
	msgs[1].buf    = buf;

	while (retries < 2) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)
			break;
		retries++;
	}

	return ret;
}

static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len) 
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = client->flags & I2C_M_TEN;
	msg.addr  = client->addr;
	msg.len   = len;
	msg.buf   = buf;

	while (retries < 2) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)
			break;
		retries++;
	}

	return ret;
}

#else
static int cst3xx_i2c_read(struct i2c_client *client, unsigned char *buf, int len) 
{
	int ret = -1;
	int retries = 0;

	while (retries < 2) {
		ret = i2c_master_recv(client, buf, len);
		if(ret<=0)
		    retries++;
        else
            break;
	}

	return ret;
}

static int cst3xx_i2c_write(struct i2c_client *client, unsigned char *buf, int len) 
{
	int ret = -1;
	int retries = 0;

	while (retries < 2) {
		ret = i2c_master_send(client, buf, len);
		if(ret<=0)
		    retries++;
        else
            break;
	}

	return ret;
}

static int cst3xx_i2c_read_register(struct i2c_client *client, unsigned char *buf, int len) 
{
	int ret = -1;

    ret = cst3xx_i2c_write(client, buf, 2);

    ret = cst3xx_i2c_read(client, buf, len);

    return ret;
}
#endif

static int hyn_cst3xx_power_ctrl(struct cst3xx_ts_data *ts, bool power_on)
{
	int ret = 0;
	if(ts == NULL){
		printk("ktf hyn_cst3xx_power_ctrl:ts is null");
		return -EINVAL;
	}
	if(!ts->avdd){
		ts->avdd = regulator_get(&ts->client->dev, "vtouch");
		if (IS_ERR(ts->avdd)) {
		 ret = PTR_ERR(ts->avdd);
		 ts->avdd=NULL;
		 dev_info(&ts->client->dev,"ktf Regulator get failed avdd ret=%d\n", ret);
		 return ret;
		}

		ret = regulator_set_voltage(ts->avdd,3000000,3000000);//fixd from 2.85v to 3.0v by liujianhui
        	if (ret) {
            		regulator_put(ts->avdd);
            		ts->avdd = NULL;
			dev_info(&ts->client->dev,"ktf Regulator set 2.8V failed avdd ret=%d\n", ret);
            		return ret;
        	}
	}
/*
	if(!ts->vcc_i2c){
		ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
		if (IS_ERR(ts->vcc_i2c)) {
			ret = PTR_ERR(ts->vcc_i2c);
			ts->vcc_i2c=NULL;
			dev_info(&ts->client->dev,"ktf Regulator get failed vcc_i2c ret=%d\n", ret);
			return ret;
		}

		ret = regulator_set_voltage(ts->vcc_i2c,1800000,1800000);
        if (ret) {
            regulator_put(ts->vcc_i2c);
            ts->vcc_i2c = NULL;
			dev_info(&ts->client->dev,"ktf Regulator set 1.8V failed vcc_i2c ret=%d\n", ret);
            return ret;
        }
	}
*/
	if (power_on) {
		printk("ktf power on E");
		ret = regulator_enable(ts->avdd);
		if (ret){
			printk("ktf can not enable avdd 2.8V: %d", ret);
			return ret;
		}
		mdelay(2);
/*
		ret = regulator_enable(ts->vcc_i2c);
		if (ret){
			printk("ktf can not enable vcc_i2c 1.8V: %d", ret);
			return ret;
		}
		msleep(2);
*/
    } else {
		printk("ktf power off E");
        ret = regulator_disable(ts->avdd);
        if (ret){
            printk("ktf can not disable avdd 2.8V: %d", ret);
            return ret;
        }
/*
        ret = regulator_disable(ts->vcc_i2c);
        if (ret){
            printk("ktf can not disable vcc_i2c 1.8V: %d", ret);
            return ret;
        }
*/
    }
	//printk("ktf hyn_cst3xx_power_ctrl done,power_on = %d,ret = %d\n",power_on,ret);
	return ret;
}

#ifdef CONFIG_TP_ESD_PROTECT
static void cst3xx_poweron_ic(unsigned int ms)
{
	int ret;

	ret = hyn_cst3xx_power_ctrl(g_ts, POWER_OFF); //
	if (ret){
		printk("%s:ctf power off failed",__func__);
	}
	g_ts->power_on = POWER_OFF;

	mdelay(ms);

	ret = hyn_cst3xx_power_ctrl(g_ts, POWER_ON); //
	if (ret){
		printk("%s:ctf power off failed",__func__);
	}
	g_ts->power_on = POWER_ON;
}
#endif
static void cst3xx_reset_ic(unsigned int ms)
{
#if 0
	unsigned char buf[4];
	buf[0] = 0xD1;
	buf[1] = 0x0E;
	cst3xx_i2c_write(g_i2c_client, buf, 2);
#else
	int ret=-1;
	ret=gpio_direction_output(g_ts->pdata->reset_gpio, 0);
	if (ret < 0) {
		printk("Failed to gpio_direction_input 0.\n");
	}
	mdelay(20);
	ret=gpio_direction_output(g_ts->pdata->reset_gpio,1);
	if (ret < 0) {
		printk("Failed to gpio_direction_input 1.\n");
	}
#endif
	mdelay(ms);
}
/*******************************************************
Function:
    test i2c communication
Input:
    client: i2c client
Output:

    success: big than 0
    fail:	negative
*******************************************************/
static int cst3xx_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int ret;
	unsigned char buf[4];

	buf[0] = 0xD1;
	buf[1] = 0x06;
	while (retry++ < 5) {
		ret = cst3xx_i2c_write(client, buf, 2);
		if (ret > 0)
			return ret;

		mdelay(2);
	}

    if(retry==5) printk("linc cst3xx hyn I2C TEST error.ret:%d;\n", ret);

	return ret;
}

static s32 irq_is_disable = 0;

void cst3xx_irq_enable(struct cst3xx_ts_data *ts)
{
	unsigned long irqflags = 0;

	//GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (irq_is_disable) {
		enable_irq(ts->client->irq);
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

void cst3xx_irq_disable(struct cst3xx_ts_data *ts)
{
	unsigned long irqflags;

	//GTP_DEBUG_FUNC();
	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!irq_is_disable) {
		irq_is_disable = 1;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

#ifdef HYN_GESTURE
void hyn_key_report(int key_value)
{
#if 1
	input_report_key(g_input_dev, key_value, 1);
	input_sync(g_input_dev);	
	input_report_key(g_input_dev, key_value, 0);
	input_sync(g_input_dev);
#else
	input_report_key(g_input_dev, KEY_POWER, 1);
	input_sync(g_input_dev);
	input_report_key(g_input_dev, KEY_POWER, 0);
	input_sync(g_input_dev);
#endif
}

#if 0
static ssize_t hyn_sysfs_tpgesture_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    ssize_t len=0;
    sprintf(&buf[len],"%s\n","tp gesture is on/off:");
    len += (strlen("tp gesture is on/off:")+1);
    if(hyn_ges_wakeup_switch == 1){
        sprintf(&buf[len],"%s\n","  on  ");
        len += (strlen("  on  ")+1);
    }else if(hyn_ges_wakeup_switch == 0){
        sprintf(&buf[len],"%s\n","  off  ");
        len += (strlen("  off  ")+1);
    }

    sprintf(&buf[len],"%s\n","tp gesture:");
    len += (strlen("tp gesture:")+1);
    sprintf(&buf[len],"%c\n",hyn_gesture_c);
    len += 2;
    return len;
}
static ssize_t hyn_sysfs_tpgesturet_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    char tmp_buf[16];

    if(copy_from_user(tmp_buf, buf, (count>16?16:count))){
        return -1;
    }
    if(buf[0] == '0'){
       	hyn_ges_wakeup_switch = 0;
        printk("linc cst3xx[HYN_GESTURE] hyn_sysfs_tpgesturet_store off.\n");
    }else if(buf[0] == '1'){
        hyn_ges_wakeup_switch = 1;
        printk("linc cst3xx[HYN_GESTURE] hyn_sysfs_tpgesturet_store on.\n");
    }

    return count;
}

static DEVICE_ATTR(tpgesture, S_IRUGO|S_IWUSR, hyn_sysfs_tpgesture_show, hyn_sysfs_tpgesturet_store);
static void hyn_request_power_idev(void)
{
    struct input_dev *idev;
    int rc = 0;
    idev = input_allocate_device();


    if(!idev){
        return;
    }
    hyn_power_idev = idev; 
    idev->name = "hyn_gesture";  
    idev->id.bustype = BUS_I2C;
    input_set_capability(idev,EV_KEY,KEY_POWER);
    input_set_capability(idev,EV_KEY,KEY_END);

    rc = input_register_device(idev);
    if(rc){
        input_free_device(idev);
        hyn_power_idev = NULL;
    }
}
static unsigned int hyn_gesture_init(void)
{
    int ret;
    struct kobject *hyn_debug_kobj;

    hyn_debug_kobj = kobject_create_and_add("hyn_gesture", NULL) ;
    if (hyn_debug_kobj == NULL)
    {
        printk("linc cst3xx%s: subsystem_register failed\n", __func__);
        return -ENOMEM;
    }
    ret = sysfs_create_file(hyn_debug_kobj, &dev_attr_tpgesture.attr);
    if (ret)
    {
        printk("linc cst3xx%s: sysfs_create_version_file failed\n", __func__);
        return ret;
    }
    hyn_request_power_idev();
    printk("linc cst3xx hyn_gesture_init success.\n");
    return 1;
}
#endif

#endif

#ifdef HYN_UPDATE_FIRMWARE_TEST
static void hyn_update_firmware_worker(struct work_struct *work)
{
	int ret = 0;

	if(update_test_count<5000){
		update_test_count++;
	}else{
		printk("[HYN4.1] hyn update firmware worker update_test_count: 5000.\n");
		return;
	}
	input_report_key(g_input_dev, 3, 1);
	input_sync(g_input_dev);
	input_report_key(g_input_dev, 3, 0);
	input_sync(g_input_dev);

	ret = cst3xx_update_firmware(g_i2c_client,pcst3xx_update_firmware);
    if (ret < 0) {
        printk ("[HYN 4.1] update cst328 fw thread failed.\n");
		update_test_fail++;
		update_test_running_on=0xff;
		cancel_delayed_work(&hyn_update_firmware_work);
		printk("[HYN 4.1] hyn update firmware worker update_test_fail done.\n");
		return;
    }
	printk("[HYN 4.1] hyn update firmware worker update_test_count: %u.\n",update_test_count);
	printk("[HYN 4.1] hyn update firmware worker update_test_fail: %u.\n",update_test_fail);

	queue_delayed_work(hyn_update_firmware_workqueue, &hyn_update_firmware_work, 2000);
    return;
}
#endif


static irqreturn_t cst3xx_ts_irq_handler(int irq, void *dev_id)
{
	struct cst3xx_ts_data *ts=NULL;
//	GTP_DEBUG_FUNC();
	ts=dev_id;
	if(!ts)
	{
		return IRQ_NONE;
	}

	if(ts->work_mode==MODE_SUSPEND)
	{
		return IRQ_HANDLED;
	}

#ifdef ANDROID_TOOL_SURPORT
		if(hyn_tp_test_flag==1){
			printk("tpd irq interrupt to dbg_waiter\n");
			tpd_flag  = 1;
			wake_up(&dbg_waiter);
			return IRQ_HANDLED;
		}
#endif

	queue_work(cst3xx_wq, &ts->work);
    cst3xx_irq_disable(ts);
	return IRQ_HANDLED;
}

static void cst3xx_touch_down(struct input_dev *input_dev,s32 id,s32 x,s32 y,s32 w)
{
	s32 temp_w = (w>>1);
#ifdef ICS_SLOT_REPORT
	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 1);
//	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_PRESSURE, temp_w);
#else
	input_report_key(input_dev, BTN_TOUCH, 1);
	input_report_abs(input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, temp_w);
	input_report_abs(input_dev, ABS_MT_PRESSURE, temp_w);
	input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(input_dev);
#endif
	#if 0
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
      {
          tpd_button(x, y, 1);
	}
    #endif
	//printk("cst3xx_touch_down  coordinate=X=%d,Y=%d,ID=%d\n",x,y,id);
}
static void cst3xx_touch_up(struct input_dev *input_dev, int id)
{
#ifdef ICS_SLOT_REPORT
    input_mt_slot(input_dev, id);
    //input_report_abs(input_dev, ABS_MT_TRACKING_ID, false);
    input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
#else
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_mt_sync(input_dev);
#endif

#if 0
	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()) {
	    tpd_button(0, 0, 0);
	}
#endif
	//printk("cst3xx_touch_up  ID=%d\n",id);

}

#ifdef ANDROID_TOOL_SURPORT   //debug tool support
#define CST3XX_PROC_DIR_NAME	"cst1xx_ts"
#define CST3XX_PROC_FILE_NAME	"cst1xx-update"
static struct proc_dir_entry *g_proc_dir, *g_update_file;
static int CMDIndex = 0;
#if 0
static struct file *cst3xx_open_fw_file(char *path)
{
	struct file * filp = NULL;
	int ret;

	filp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(filp)) {
        ret = PTR_ERR(filp);
        return NULL;
    }
    filp->f_op->llseek(filp, 0, 0);

    return filp;
}

static void cst3xx_close_fw_file(struct file * filp)
{
	 if(filp)
	 filp_close(filp,NULL);
}

static int cst3xx_read_fw_file(unsigned char *filename, unsigned char *pdata, int *plen)
{
	struct file *fp;
	int size;
	int length;
	int ret = -1;

	if((pdata == NULL) || (strlen(filename) == 0)) {
		printk("linc cst3xxfile name is null.\n");
		return ret;
	}
	fp = cst3xx_open_fw_file(filename);
	if(fp == NULL) {
        printk("linc cst3xxOpen bin file faild.path:%s.\n", filename);
		goto clean;
	}

	length = fp->f_op->llseek(fp, 0, SEEK_END);
	fp->f_op->llseek(fp, 0, 0);
	size = fp->f_op->read(fp, pdata, length, &fp->f_pos);
	if(size == length) {
    	ret = 0;
    	*plen = length;
	}

clean:
	cst3xx_close_fw_file(fp);
	return ret;
}

#else

static struct file *cst3xx_open_fw_file(char *path, mm_segment_t * old_fs_p)
{
	struct file * filp;
	int ret;

	*old_fs_p = get_fs();
//	set_fs(KERNEL_DS);
	filp = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(filp))
	{
        ret = PTR_ERR(filp);
        return NULL;
    }
    filp->f_op->llseek(filp, 0, 0);

    return filp;
}

static void cst3xx_close_fw_file(struct file * filp,mm_segment_t old_fs)
{
	//set_fs(old_fs);
	if(filp)
	    filp_close(filp,NULL);
}

static int cst3xx_read_fw_file(unsigned char *filename, unsigned char *pdata, int *plen)
{
	struct file *fp;
	int ret = -1;
	loff_t pos;
	off_t fsize;
	struct inode *inode;
	unsigned long magic;
	mm_segment_t old_fs;

	printk("cst3xx_read_fw_file enter.\n");

	if((pdata == NULL) || (strlen(filename) == 0)) {
		printk(" cst3xxfile name is null.\n");
		return ret;
	}
	fp = cst3xx_open_fw_file(filename,&old_fs);
	if(fp == NULL) {
        printk(" cst3xxOpen bin file faild.path:%s.\n", filename);
		goto clean;
	}

#if 0

	length = fp->f_op->llseek(fp, 0, SEEK_END);
	fp->f_op->llseek(fp, 0, 0);
	size = fp->f_op->read(fp, pdata, length, &fp->f_pos);
	if(size == length)
	{
    	ret = 0;
    	*plen = length;
	}
	else
	{
		printk("read bin file length fail****size:%d*******length:%d .\n", size,length);

	}

#else

	if (IS_ERR(fp)) {
		printk("error occured while opening file %s.\n", filename);
		return -EIO;
	}
	inode = fp->f_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	ret=vfs_read(fp, pdata, fsize, &pos);
	if(ret==fsize){
		printk("vfs_read success.ret:%d.\n",ret);
	}else{
		printk("vfs_read fail.ret:%d.\n",ret);
	}
	filp_close(fp, NULL);
	set_fs(old_fs);

	printk("vfs_read done.\n");

#endif

clean:
	cst3xx_close_fw_file(fp,old_fs);
	return ret;
}

#endif

static int cst3xx_apk_fw_dowmload(struct i2c_client *client,
		unsigned char *pdata, int length)
{

#ifdef 	HYN_UPDATE_FIRMWARE_ONLINE
{
		int ret=-1;
		pcst3xx_update_firmware=(unsigned char *)pdata;
		ret = cst3xx_update_firmware(g_i2c_client, pdata);
		return ret;
}
#endif

	return 0;
}

static ssize_t cst3xx_proc_read_foobar(struct file *page,char __user *user_buf, size_t count, loff_t *data)
{
	unsigned char buf[1024];
	int len = 0;
	int ret;

	printk(" linc cst3xx is entering cst3xx_proc_read_foobar.\n");

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif

	if (CMDIndex == 0) {
		buf[0] = 0xD1;
		buf[1] = 0x01;
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if (ret < 0)
			return -1;

		mdelay(10);

		buf[0] = 0xD1;
		buf[1] = 0xF4;//E8
		ret = cst3xx_i2c_read_register(g_i2c_client, buf, 28);//4   //4
		if (ret < 0)
			return -1;

		g_cst3xx_tx=buf[0];
		g_cst3xx_rx=buf[2];

		sprintf(buf, "  cst3xx_proc_read_foobar:g_cst3xx_tx:%d,g_cst3xx_rx:%d.\n",g_cst3xx_tx,g_cst3xx_rx);
        len = strlen(buf);
		ret = copy_to_user(user_buf,buf,len);

	}
	else if (CMDIndex == 1) {
		buf[0] = 0xD1;
		buf[1] = 0x01;
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if (ret < 0) return -1;

		mdelay(10);

		buf[0] = 0xD1;
		buf[1] = 0xF4;
		ret = cst3xx_i2c_read_register(g_i2c_client, buf, 28);
		if (ret < 0) return -1;

		g_cst3xx_tx=buf[0];
		g_cst3xx_rx=buf[2];

		printk("  cst3xx_proc_read_foobar:g_cst3xx_tx:%d,g_cst3xx_rx:%d.\n",g_cst3xx_tx,g_cst3xx_rx);

		len = 28;
		ret = copy_to_user(user_buf,buf,len);

		buf[0] = 0xD1;
		buf[1] = 0x09;
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
	}
	if((CMDIndex == 2 )|| (CMDIndex == 3)||(CMDIndex == 4))
	{
		unsigned short rx,tx;
		int data_len;

		rx = g_cst3xx_rx;
		tx = g_cst3xx_tx;
		data_len = rx*tx*2 + 4 + (tx+rx)*2 + rx + rx;

		tpd_flag = 0;
		if(CMDIndex == 2)  //read diff
		{
			buf[0] = 0xD1;
			buf[1] = 0x0D;
		}
		else  if(CMDIndex == 3)        //rawdata
		{
			buf[0] = 0xD1;
			buf[1] = 0x0A;
		}
		else if(CMDIndex == 4)          //factory test
		{
			buf[0] = 0xD1;
			buf[1] = 0x19;
			data_len = rx*tx*4 +(4 + tx + rx)*2;
		}

		hyn_tp_test_flag=1;
		ret = i2c_master_send(g_i2c_client, buf, 2);
		if (ret < 0) {
			printk("Write command raw/diff mode failed:%d.\n", ret);
			goto END;
		}

		g_unnormal_mode = 1;
		mdelay(14);

		wait_event(dbg_waiter, tpd_flag!=0);
		tpd_flag = 0;

	    printk(" cst3xx Read wait_event interrupt");
		if(CMDIndex == 4)
		{
			buf[0] = 0x12;
			buf[1] = 0x15;
		}
		else
		{
			buf[0] = 0x80;
			buf[1] = 0x01;
		}
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
		if (ret < 0) {
			printk("Write command(0x8001) failed.error:%d.\n", ret);
			goto END;
		}
		ret = cst3xx_i2c_read(g_i2c_client, &buf[2], data_len);
		if(ret < 0)
		{
			printk("Read raw/diff data failed.error:%d.\n", ret);
			goto END;
		}

		mdelay(2);

		buf[0] = 0xD1;
		buf[1] = 0x08;
		ret = cst3xx_i2c_write(g_i2c_client, buf, 2); 		
		if(ret < 0)
		{
			printk("Write command normal mode failed.error:%d.\n", ret);
			goto END;
		}

		if (4 == CMDIndex) {	//factory test
			buf[0] = 0xD1;
			buf[1] = 0x02;
			ret = cst3xx_i2c_write(g_i2c_client, buf, 2);
			if (ret < 0) {
				printk("Write command normal mode failed.error:%d.\n", ret);
				goto END;
			}
		}

		buf[0] = rx;
		buf[1] = tx;
    	ret = copy_to_user(user_buf,buf,data_len + 2);
    	len = data_len + 2;

		mdelay(2);
	}

END:

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif
	g_unnormal_mode = 0;
	CMDIndex = 0;
	hyn_tp_test_flag=0;
	return len;
}

static ssize_t cst3xx_proc_write_foobar(struct file *file, const char __user *buffer,size_t count, loff_t *data)
{
    unsigned char cmd[128];
    unsigned char *pdata = NULL;
	int len;
	int ret;
    int length = 24*1024;

	if (count > 128)
		len = 128;
	else
		len = count;

   printk(" linc cst3xx is entering cst3xx_proc_write_foobar .\n");

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif


	if (copy_from_user(cmd, buffer, len))  {
		printk("linc cst3xxcopy data from user space failed.\n");
		return -EFAULT;
	}

	 printk(" linc cmd:%d......%d.......len:%d\r\n", cmd[0], cmd[1], len);

	if (cmd[0] == 0) {
	    pdata = kzalloc(sizeof(char)*length, GFP_KERNEL);
	    if(pdata == NULL) {
	        printk("linc cst3xxzalloc GFP_KERNEL memory fail.\n");
	        return -ENOMEM;
	    }
		ret = cst3xx_read_fw_file(&cmd[1], pdata, &length);
	  	if(ret < 0) {
			if(pdata != NULL) {
				kfree(pdata);
				pdata = NULL;
			}
			return -EPERM;
	  	}

		ret = cst3xx_apk_fw_dowmload(g_i2c_client, pdata, length);
	  	if(ret < 0){
	        printk("linc cst3xxupdate firmware failed.\n");
			if(pdata != NULL) {
				kfree(pdata);
				pdata = NULL;
			}
	        return -EPERM;
		}
        mdelay(50);

		cst3xx_firmware_info(g_i2c_client);

		if(pdata != NULL) {
			kfree(pdata);
			pdata = NULL;
		}
	}
	else if (cmd[0] == 2) {
		//cst3xx_touch_release();
		CMDIndex = cmd[1];
	}
	else if (cmd[0] == 3) {
		CMDIndex = 0;
	}

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif

	return count;
}

static const struct file_operations proc_tool_debug_fops = {

	.owner		= THIS_MODULE,

	.read	    = cst3xx_proc_read_foobar,

	.write		= cst3xx_proc_write_foobar, 

};



static int  cst3xx_proc_fs_init(void)

{

	int ret;

	g_proc_dir = proc_mkdir(CST3XX_PROC_DIR_NAME, NULL);

	if (g_proc_dir == NULL) {

		ret = -ENOMEM;

		goto out;

	}

   g_update_file = proc_create(CST3XX_PROC_FILE_NAME, 0777, g_proc_dir,&proc_tool_debug_fops);

   if (g_update_file == NULL)

   {

      ret = -ENOMEM;

      goto no_foo;

   }
	return 0;

no_foo:

	remove_proc_entry(CST3XX_PROC_FILE_NAME, g_proc_dir);

out:

	return ret;

}
#ifdef HYN_SYSFS_NODE_EN

static ssize_t hyn_tpfwver_show(struct device *dev,	struct device_attribute *attr,char *buf)
{
	ssize_t num_read_chars = 0;
	u8 buf1[20];
	u8 retry;
	int ret=-1;
	unsigned int firmware_version,module_version,project_version,chip_type,checksum,ic_checkcode;

	mutex_lock(&g_device_mutex);
	memset((u8 *)buf1, 0, 20);

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif

	firmware_version=0;
	module_version=0;
	project_version=0;
	chip_type=0;
	checksum=0;
	ic_checkcode=0;
	retry=0;

START:
	mdelay(5);
	buf1[0] = 0xD1;
	buf1[1] = 0x01;
	ret = cst3xx_i2c_write(g_i2c_client, buf1, 2);
	if (ret < 0)
	{
	    num_read_chars = snprintf(buf, 50, "hyn_tpfwver_show:write debug info command fail.\n");
		printk("%s : ret = %d. hyn_tpfwver_show:write debug info command fail.\n",__func__,ret);
		goto err_return;
	}

	mdelay(5);

	buf1[0] = 0xD1;
	buf1[1] = 0xFC;
	ret = cst3xx_i2c_read_register(g_i2c_client, buf1, 20);
	if (ret < 0)
	{
		num_read_chars = snprintf(buf, 50, "hyn_tpfwver_show:Read version resgister fail.\n");
		printk("%s : ret = %d. hyn_tpfwver_show:Read version resgister fail.\n",__func__,ret);
		goto err_return;
	}

	chip_type = buf1[11];
	chip_type <<= 8;
	chip_type |= buf1[10];

	project_version = buf1[9];
	project_version <<= 8;
	project_version |= buf1[8];

	firmware_version = buf1[15];
	firmware_version <<= 8;
	firmware_version |= buf1[14];
	firmware_version <<= 8;
	firmware_version |= buf1[13];
	firmware_version <<= 8;
	firmware_version |= buf1[12];

	checksum = buf1[19];
	checksum <<= 8;
	checksum |= buf1[18];
	checksum <<= 8;
	checksum |= buf1[17];
	checksum <<= 8;
	checksum |= buf1[16];

	//0xCACA0000
	ic_checkcode = buf1[3];
	ic_checkcode <<= 8;
	ic_checkcode |= buf1[2];
	ic_checkcode <<= 8;
	ic_checkcode |= buf1[1];
	ic_checkcode <<= 8;
	ic_checkcode |= buf1[0];

	printk("linc cst3xx [cst3xx] the chip ic_checkcode:0x%x£¬retry:0x%x.\r\n",ic_checkcode,retry);

	if((ic_checkcode&0xffff0000)!=0xCACA0000){

		printk("linc cst3xx [cst3xx]  ic_checkcode read error .\r\n");
		goto err_return;
	}

	buf1[0] = 0xD1;
	buf1[1] = 0x09;
	ret = cst3xx_i2c_write(g_i2c_client, buf1, 2);
	if(ret < 0)
	{
		num_read_chars = snprintf(buf, 50, "hyn_tpfwver_show:write normal mode fail.\n");
		printk("%s : ret = %d. hyn_tpfwver_show:write normal mode fail.\n",__func__,ret);
		goto err_return;
	}

	num_read_chars = snprintf(buf, 240, "firmware_version: 0x%02X,module_version:0x%02X,project_version:0x%02X,chip_type:0x%02X,checksum:0x%02X,esd_count:0x%08X,work_mode:0x%01X.\n",
	firmware_version,module_version, project_version,chip_type,checksum,g_ts->clk_tick_cnt,g_ts->work_mode);

	printk("%s ---> TP info : %s.\n",__func__,buf);

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif

	mutex_unlock(&g_device_mutex);
	printk("%s : num_read_chars = %ld.\n",__func__,num_read_chars);
	return num_read_chars;

err_return:
	if(retry<5){
		retry++;
		goto START;
	}else{
		printk("%s : num_read_chars = %ld.\n",__func__,num_read_chars);
		buf1[0] = 0xD1;
		buf1[1] = 0x09;
		ret = cst3xx_i2c_write(g_i2c_client, buf1, 2);

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif
		mutex_unlock(&g_device_mutex);
		return -1;
	}

}

static ssize_t hyn_tpfwver_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	/*place holder for future use*/
	return -EPERM;
}


static ssize_t hyn_tprwreg_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	ssize_t num_read_chars = 0;
	u8 buf1[20];
	u8 retry;
	int ret=-1;
	u16 value_0,value_1,value_2,value_3,value_4;

	mutex_lock(&g_device_mutex);
	memset((u8 *)buf1, 0, 20);

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue != NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif

	retry=0;
	value_0=0;
	value_1=0;
	value_2=0;
	value_3=0;
	value_4=0;

START:
	mdelay(5);
	buf1[0] = 0xD1;
	buf1[1] = 0x01;
	ret = cst3xx_i2c_write(g_i2c_client, buf1, 2);
	if (ret < 0)
	{
	    num_read_chars = snprintf(buf, 50, "hyn_tprwreg_show:write debug info command fail.\n");
		printk("%s : ret = %d. hyn_tpfwver_show:write debug info command fail.\n",__func__,ret);
		goto err_return;
	}

	mdelay(5);

	buf1[0] = 0xD2;
	buf1[1] = 0x20;
	ret = cst3xx_i2c_read_register(g_i2c_client, buf1, 10);
	if (ret < 0)
	{
		num_read_chars = snprintf(buf, 50, "hyn_tprwreg_show:Read version resgister fail.\n");
		printk("%s : ret = %d. hyn_tprwreg_show:Read version resgister fail.\n",__func__,ret);
		goto err_return;
	}
	value_0 = buf1[1];
	value_0 <<= 8;
	value_0 |= buf1[0];

	value_1 = buf1[3];
	value_1 <<= 8;
	value_1 |= buf1[2];
	
	value_2 = buf1[5];
	value_2 <<= 8;
	value_2 |= buf1[4];
	
	value_3 = buf1[7];
	value_3 <<= 8;
	value_3 |= buf1[6];
	
	value_4 = buf1[9];
	value_4 <<= 8;
	value_4 |= buf1[8];
	

	num_read_chars = snprintf(buf, 240, "value_0: 0x%02X,value_1:0x%02X,value_2:0x%02X,value_3:0x%02X,value_4:0x%02X.\n",
	value_0,value_1, value_2,value_3,value_4);

	buf1[0] = 0xD1;
	buf1[1] = 0x09;
	ret = cst3xx_i2c_write(g_i2c_client, buf1, 2);
	if(ret < 0)
	{
		num_read_chars = snprintf(buf, 50, "hyn_tprwreg_show:write normal mode fail.\n");
		printk("%s : ret = %d. hyn_tprwreg_show:write normal mode fail.\n",__func__,ret);
		goto err_return;
	}


#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif

	mutex_unlock(&g_device_mutex);
	printk("%s : num_read_chars = %ld.\n",__func__,num_read_chars);
	return num_read_chars;

err_return:
	if(retry<5){
		retry++;
		goto START;
	}else{
		printk("%s : num_read_chars = %ld.\n",__func__,num_read_chars);
		buf1[0] = 0xD1;
		buf1[1] = 0x09;
		ret = cst3xx_i2c_write(g_i2c_client, buf1, 2);

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif
		mutex_unlock(&g_device_mutex);
		return -1;
	}

}

static ssize_t hyn_tprwreg_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg = 0;
	u16 regaddr = 0xff;
	u8 valbuf[10] = {0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if (num_read_chars != 2) {
		if (num_read_chars != 4) {
			printk("please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = kstrtoul(valbuf, 16, &wmreg);


	if (0 != retval) {
		printk("%s() - ERROR: The given input was: \"%s\"\n",__func__, buf);
		goto error_return;
	}

	if (2 == num_read_chars) {
		/*read register*/
		regaddr = valbuf[0]<<8;
		regaddr |= valbuf[1];

		if(regaddr==0x3838){   //88-ascll
			//disable_irq(touch_irq);
			cst3xx_irq_disable(g_ts);
		}else if(regaddr==0x3939){//99-ascll
			//enable_irq(touch_irq);
			cst3xx_irq_enable(g_ts);
		}else if(regaddr==0x3737){
			cst3xx_reset_ic(10);
		}


		if (cst3xx_i2c_read_register(g_i2c_client, valbuf,num_read_chars) < 0)
			printk("Could not read the register(0x%02x).\n",regaddr);
		else
			printk("the register(0x%02x) is 0x%02x\n",regaddr,valbuf[0] );
	} else {
		regaddr = valbuf[0]<<8;
		regaddr |= valbuf[1];
		if (cst3xx_i2c_read_register(g_i2c_client, valbuf, num_read_chars) < 0)
			printk("Could not write the register(0x%02x)\n",regaddr);
		else
			printk("Write 0x%02x into register(0x%02x) successful\n",regaddr, valbuf[0]);
	}

error_return:
	mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t hyn_fwupdate_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/*upgrade from *.i*/
static ssize_t hyn_fwupdate_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	printk("hyn_fwupdate_store enter.\n");
	mutex_lock(&g_device_mutex);
#ifdef HYN_UPDATE_FIRMWARE_ONLINE
	hyn_boot_update_fw(g_i2c_client);
#endif
	mutex_unlock(&g_device_mutex);
	return count;
}

static ssize_t hyn_fwupgradeapp_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	/*place holder for future use*/
	return -EPERM;
}


/*upgrade from app.bin*/
static ssize_t hyn_fwupgradeapp_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	char fwname[256];
	int ret;
	unsigned char *pdata = NULL;
	int length = 24*1024;
	//struct i2c_client *client = container_of(dev, struct i2c_client, dev);


	printk("hyn_fwupgradeapp_store enter.\n");

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "/mnt/%s", buf);
	fwname[count-1+8] = '\0';

	printk("fwname:%s.\n",fwname);
	pdata = kzalloc(sizeof(char)*length, GFP_KERNEL);
    if(pdata == NULL)
	{
        printk("hyn_fwupgradeapp_store GFP_KERNEL memory fail.\n");
        return -ENOMEM;
    }

	mutex_lock(&g_device_mutex);
	ret = cst3xx_read_fw_file(fwname, pdata, &length);
	if(ret < 0)
	{
		printk("cst2xx_read_fw_file fail.\n");
		if(pdata != NULL)
		{
			kfree(pdata);
			pdata = NULL;
		}
	}else{

		ret = cst3xx_apk_fw_dowmload(g_i2c_client, pdata, length);
		if(ret < 0)
		{
	        printk("cst2xx_apk_fw_dowmload failed.\n");
			if(pdata != NULL)
			{
				kfree(pdata);
				pdata = NULL;
			}
		}
	}

	mutex_unlock(&g_device_mutex);

	printk("hyn_fwupgradeapp_store exit.\n");

	return count;
}



/*sysfs */
/*get the fw version
*example:cat hyntpfwver
*/
static DEVICE_ATTR(hyntpfwver, S_IRUGO | S_IWUSR, hyn_tpfwver_show,
			hyn_tpfwver_store);

/*upgrade from *.i
*example: echo 1 > hynfwupdate
*/
static DEVICE_ATTR(hynfwupdate, S_IRUGO | S_IWUSR, hyn_fwupdate_show,
			hyn_fwupdate_store);

/*read and write register
*read example: echo 88 > hyntprwreg ---read register 0x88
*write example:echo 8807 > hyntprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(hyntprwreg, S_IRUGO | S_IWUSR, hyn_tprwreg_show,
			hyn_tprwreg_store);


/*upgrade from app.bin
*example:echo "*_app.bin" > hynfwupgradeapp
*/
static DEVICE_ATTR(hynfwupgradeapp, S_IRUGO | S_IWUSR, hyn_fwupgradeapp_show,
			hyn_fwupgradeapp_store);

/*add your attr in here*/
static struct attribute *hyn_attributes[] = {
	&dev_attr_hyntpfwver.attr,
	&dev_attr_hynfwupdate.attr,
	&dev_attr_hyntprwreg.attr,
	&dev_attr_hynfwupgradeapp.attr,
	NULL
};

static struct attribute_group hyn_attribute_group = {
	.attrs = hyn_attributes
};
/*create sysfs for debug*/

int hyn_create_sysfs(struct i2c_client *client)
{
	int err;
	g_i2c_client=client;
	if ((k_obj = kobject_create_and_add("hynitron_debug", NULL)) == NULL ) {
		printk("hynitron_debug sys node create error.\n");
    }
	err = sysfs_create_group(k_obj, &hyn_attribute_group);
	if (0 != err) {
		printk("%s() - ERROR: sysfs_create_group() failed.\n",__func__);
		sysfs_remove_group(k_obj, &hyn_attribute_group);
		return -EIO;
	} else {
		mutex_init(&g_device_mutex);
		printk("cst3xx:%s() - sysfs_create_group() succeeded.\n",__func__);
	}
	return err;
}

void hyn_release_sysfs(struct i2c_client *client)
{
	sysfs_remove_group(k_obj, &hyn_attribute_group);
	mutex_destroy(&g_device_mutex);
}
#endif


#define PROC_TP_VERSION "driver/tp_version"
static char mtk_tp_version[128] = {0};

static int tp_info_version_read(struct seq_file *m, void *v)
{
    struct input_dev *input_dev =g_input_dev;
   // struct i2c_client *client = g_i2c_client;
    unsigned int fwver = 0;

    mutex_lock(&input_dev->mutex);

	fwver=g_cst3xx_ic_version;

    if ((fwver == 0xFF) || (fwver == 0x00)){
              printk("get tp fw version fail!!\n");
    }else{

#ifdef HYN_UPDATE_FIRMWARE_TEST
	 sprintf(mtk_tp_version,"0x%02x,0x%02x,0x%02x",fwver,update_test_count,update_test_fail);
#else
	 sprintf(mtk_tp_version,"0x%02x",fwver);
#endif
    }

     seq_printf(m, "%s\n",mtk_tp_version);

     mutex_unlock(&input_dev->mutex);

   return 0;
};

static int proc_tp_version_open_cst3xx(struct inode *inode, struct file *file)
{
    return single_open(file, tp_info_version_read, NULL);
};

static  struct file_operations tp_version_proc_fops1 = {
    .owner = THIS_MODULE,
    .open  = proc_tp_version_open_cst3xx,
    .read  = seq_read,
};
////steven add end 20181024

#endif


#ifdef CONFIG_TP_ESD_PROTECT

static int esd_work_cycle = 1000;
static struct delayed_work esd_check_work;
static int esd_running;
static struct mutex esd_lock;
static void cst3xx_esd_check_func(struct work_struct *);
static unsigned int pre_counter = 0;
static unsigned int cur_counter = 0;
static char i2c_lock_flag = 0;

void cst3xx_init_esd_protect(void)
{
    esd_work_cycle = 1 * HZ;	/*HZ: clock ticks in 1 second generated by system*/
	printk(" linc Clock ticks for an esd cycle: %d", esd_work_cycle);
	INIT_DELAYED_WORK(&esd_check_work, cst3xx_esd_check_func);
	mutex_init(&esd_lock);

}

void cst3xx_esd_switch(s32 on)
{
	mutex_lock(&esd_lock);
	if (SWITCH_ESD_ON == on) {	/* switch on esd check */
		if (!esd_running) {
			esd_running = 1;
			g_ts->esd_running=1;
			pre_counter = 0;
			cur_counter = 0;
			printk(" linc Esd protector started!");
			queue_delayed_work(cst3xx_esd_workqueue, &esd_check_work, esd_work_cycle);
		}
	} else {		/* switch off esd check */
		if (esd_running) {
			esd_running = 0;
			g_ts->esd_running=0;
			printk(" linc Esd protector stopped!");
			cancel_delayed_work(&esd_check_work);
		}
	}
	mutex_unlock(&esd_lock);
}

static void cst3xx_esd_check_func(struct work_struct *work)
{

    int retry = 0;
	int ret;
	int check_sum;
	unsigned char buf[8];

	if (!esd_running) {
	printk(" linc Esd protector suspended!");
	return;
	}

	if((i2c_lock_flag != 0)||(hyn_tp_test_flag!=0))
		goto END;
	else
		i2c_lock_flag = 1;

	buf[0] = 0xD0;
	buf[1] = 0x40;

	while(retry++ < 5) {
		ret = cst3xx_i2c_read_register(g_i2c_client, buf, 6);
		if (ret > 0){
			check_sum = buf[0]+buf[1]+buf[2]+buf[3]+0xA5;
			if(check_sum != ( (buf[4]<<8)+ buf[5])){
		          printk("[HYN]cst3xx_esd_check_func read check_sum error. check_sum = 0x%x.\n", check_sum);
			}else{
				if(g_ts->work_mode==MODE_RESUME)
					break;
				else
					goto END;
			}
		}
		mdelay(4);
	}

    if((retry==6) || (ret<0)){
		goto hyn_esd_check_init;
    }

	cur_counter = buf[3]+(buf[2]<<8)+(buf[1]<<16)+(buf[0]<<24);
	g_ts->clk_tick_cnt=cur_counter;
	if(((cur_counter<pre_counter) || ((cur_counter-pre_counter)<20)) && (pre_counter>100))
	{
           printk("[HYN]cst3xx_esd_check_func cur_counter is :0x%x. pre_counter is:0x%x.------\n",cur_counter,pre_counter);
           goto hyn_esd_check_init;
	}
	goto END;

hyn_esd_check_init:
	printk("[HYN]cst3xx_esd_check_func reset .\n");
	if(g_ts->work_mode==MODE_RESUME){
		if(report_finger_flag==0){
			cst3xx_irq_disable(g_ts);
			cst3xx_poweron_ic(10);
			cst3xx_reset_ic(30);
			cst3xx_irq_enable(g_ts);
		}else{		
			report_finger_flag--;
		}
		cur_counter=0;
		pre_counter=0;
	}

END:
	i2c_lock_flag = 0;
	pre_counter=cur_counter;
	mutex_lock(&esd_lock);
	if (esd_running)
		queue_delayed_work(cst3xx_esd_workqueue, &esd_check_work, esd_work_cycle);
	else
		printk(" linc Esd protector suspended!");
	mutex_unlock(&esd_lock);
}

#endif


/*******************************************************
Function:
    get firmware version, ic type...
Input:
    client: i2c client
Output:
    success: 0
    fail:	-1
*******************************************************/
static int cst3xx_firmware_info(struct i2c_client * client)
{
	int ret;
	unsigned char buf[28];
//	unsigned short ic_type, project_id;

	buf[0] = 0xD1;
	buf[1] = 0x01;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;

	mdelay(10);

	buf[0] = 0xD1;
	buf[1] = 0xFC;
	ret = cst3xx_i2c_read_register(client, buf, 4);
	if (ret < 0) return -1;

	//0xCACA0000
	g_cst3xx_ic_checkcode = buf[3];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[2];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[1];
	g_cst3xx_ic_checkcode <<= 8;
	g_cst3xx_ic_checkcode |= buf[0];

	printk("linc cst3xx [cst3xx] the chip g_cst3xx_ic_checkcode:0x%x.\r\n",g_cst3xx_ic_checkcode);

	if((g_cst3xx_ic_checkcode&0xffff0000)!=0xCACA0000){

		printk("linc cst3xx [cst3xx] cst3xx_firmware_info read error .\r\n");
		return -1;
	}

	mdelay(10);
	buf[0] = 0xD2;
	buf[1] = 0x04;
	ret = cst3xx_i2c_read_register(client, buf, 4);
	if (ret < 0) return -1;
	g_cst3xx_ic_type = buf[3];
	g_cst3xx_ic_type <<= 8;
	g_cst3xx_ic_type |= buf[2];

	g_cst3xx_ic_project_id = buf[1];
	g_cst3xx_ic_project_id <<= 8;
	g_cst3xx_ic_project_id |= buf[0];

	printk("linc cst3xx [cst3xx] the chip ic g_cst3xx_ic_type :0x%x, g_cst3xx_ic_project_id:0x%x\r\n",
		g_cst3xx_ic_type, g_cst3xx_ic_project_id);


	mdelay(2);

	buf[0] = 0xD2;
	buf[1] = 0x08;
	ret = cst3xx_i2c_read_register(client, buf, 8);
	if (ret < 0) return -1;

	g_cst3xx_ic_version = buf[3];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[2];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[1];
	g_cst3xx_ic_version <<= 8;
	g_cst3xx_ic_version |= buf[0];

	g_cst3xx_ic_checksum = buf[7];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[6];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[5];
	g_cst3xx_ic_checksum <<= 8;
	g_cst3xx_ic_checksum |= buf[4];	

	printk("linc cst3xx [cst3xx] the chip ic version:0x%x, checksum:0x%x\r\n",
		g_cst3xx_ic_version, g_cst3xx_ic_checksum);

	if(g_cst3xx_ic_version==0xA5A5A5A5)
	{
		printk("linc cst3xx [cst3xx] the chip ic don't have firmware. \n");
		return -1;
	}

    buf[0] = 0xD1;
	buf[1] = 0x09;
	ret = cst3xx_i2c_write(client, buf, 2);
	if (ret < 0) return -1;
    mdelay(5);

	return 0;
}


#ifdef 	HYN_UPDATE_FIRMWARE_ONLINE

/*******************************************************
Function:
    read checksum in bootloader mode
Input:
    client: i2c client
    strict: check checksum value
Output:
    success: 0
    fail:	-1
*******************************************************/

#define CST3XX_BIN_SIZE    (24*1024 + 24)

static int cst3xx_check_checksum(struct i2c_client * client)
{
	int ret;
	int i;
	unsigned int  checksum;
	unsigned int  bin_checksum;
	unsigned char buf[4];
	const unsigned char *pData;

	for(i=0; i<5; i++)
	{
		buf[0] = 0xA0;
		buf[1] = 0x00;
		ret = cst3xx_i2c_read_register(client, buf, 1);
		if(ret < 0)
		{
			mdelay(2);
			continue;
		}

		if(buf[0]!=0)
			break;
		else
		mdelay(2);
	}
    mdelay(2);


    if(buf[0]==0x01)
	{
		buf[0] = 0xA0;
		buf[1] = 0x08;
		ret = cst3xx_i2c_read_register(client, buf, 4);

		if(ret < 0)	return -1;

		// read chip checksum
		checksum = buf[0] + (buf[1]<<8) + (buf[2]<<16) + (buf[3]<<24);

        pData=(unsigned char  *)pcst3xx_update_firmware +24*1024+16;   //7*1024 +512
		bin_checksum = pData[0] + (pData[1]<<8) + (pData[2]<<16) + (pData[3]<<24);

        printk(" linc hyn the updated ic checksum is :0x%x. the updating firmware checksum is:0x%x------\n", checksum, bin_checksum);

        if(checksum!=bin_checksum)
		{
			printk("linc cst3xx hyn check sum error.\n");		
			return -1;

		}

	}
	else
	{
		printk("linc cst3xx hyn No checksum.\n");
		return -1;
	}
	return 0;
}
static int cst3xx_into_program_mode(struct i2c_client * client)
{
	int ret;
	unsigned char buf[4];

	buf[0] = 0xA0;
	buf[1] = 0x01;
	buf[2] = 0xAA;	//set cmd to enter program mode
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0)  return -1;
	mdelay(2);

	buf[0] = 0xA0;
	buf[1] = 0x02;	//check whether into program mode
	ret = cst3xx_i2c_read_register(client, buf, 1);
	if (ret < 0)  return -1;

	if (buf[0] != 0x55) return -1;

	return 0;
}

static int cst3xx_exit_program_mode(struct i2c_client * client)
{
	int ret;
	unsigned char buf[3];

	buf[0] = 0xA0;
	buf[1] = 0x06;
	buf[2] = 0xEE;
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0)
		return -1;

	mdelay(10);	//wait for restart

	return 0;
}

static int cst3xx_erase_program_area(struct i2c_client * client)
{
	int ret;
	unsigned char buf[3];

	buf[0] = 0xA0;
	buf[1] = 0x02;
	buf[2] = 0x00;		//set cmd to erase main area
	ret = cst3xx_i2c_write(client, buf, 3);
	if (ret < 0) return -1;

	mdelay(5);

	buf[0] = 0xA0;
	buf[1] = 0x03;
	ret = cst3xx_i2c_read_register(client, buf, 1);
	if (ret < 0)  return -1;

	if (buf[0] != 0x55) return -1;

	return 0;
}

static int cst3xx_write_program_data(struct i2c_client * client,
		const unsigned char *pdata)
{
	int i, ret;
	unsigned char *i2c_buf;
	unsigned short eep_addr;
	int total_kbyte;
#ifdef TRANSACTION_LENGTH_LIMITED
	unsigned char temp_buf[8];
	unsigned short iic_addr;
	int  j;

#endif

	i2c_buf = kmalloc(sizeof(unsigned char)*(1024 + 2), GFP_KERNEL);
	if (i2c_buf == NULL)
		return -1;

	//make sure fwbin len is N*1K
	//total_kbyte = len / 1024;
	total_kbyte = 24;
	for (i=0; i<total_kbyte; i++) {
		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x14;
		eep_addr = i << 10;		//i * 1024
		i2c_buf[2] = eep_addr;
		i2c_buf[3] = eep_addr>>8;
		ret = cst3xx_i2c_write(client, i2c_buf, 4);
		if (ret < 0)
			goto error_out;

#ifdef TRANSACTION_LENGTH_LIMITED
		memcpy(i2c_buf, pdata + eep_addr, 1024);
		for(j=0; j<256; j++) {
			iic_addr = (j<<2);
    	temp_buf[0] = (iic_addr+0xA018)>>8;
    	temp_buf[1] = (iic_addr+0xA018)&0xFF;
		temp_buf[2] = i2c_buf[iic_addr+0];
		temp_buf[3] = i2c_buf[iic_addr+1];
		temp_buf[4] = i2c_buf[iic_addr+2];
		temp_buf[5] = i2c_buf[iic_addr+3];
    	ret = cst3xx_i2c_write(client, temp_buf, 6);
    		if (ret < 0)
    			goto error_out;
		}
#else

		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x18;
		memcpy(i2c_buf + 2, pdata + eep_addr, 1024);
		ret = cst3xx_i2c_write(client, i2c_buf, 1026);
		if (ret < 0)
			goto error_out;
#endif

		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x04;
		i2c_buf[2] = 0xEE;
		ret = cst3xx_i2c_write(client, i2c_buf, 3);
		if (ret < 0)
			goto error_out;

		mdelay(60);

		i2c_buf[0] = 0xA0;
		i2c_buf[1] = 0x05;
		ret = cst3xx_i2c_read_register(client, i2c_buf, 1);
		if (ret < 0)
			goto error_out;

		if (i2c_buf[0] != 0x55)
			goto error_out;

	}

	i2c_buf[0] = 0xA0;
	i2c_buf[1] = 0x03;
	i2c_buf[2] = 0x00;
	ret = cst3xx_i2c_write(client, i2c_buf, 3);
	if (ret < 0)
		goto error_out;

	mdelay(8);

	if (i2c_buf != NULL) {
		kfree(i2c_buf);
		i2c_buf = NULL;
	}

	return 0;

error_out:
	if (i2c_buf != NULL) {
		kfree(i2c_buf);
		i2c_buf = NULL;
	}
	return -1;
}

static int cst3xx_update_firmware(struct i2c_client * client, const unsigned char *pdata)
{
	int ret;
	int retry_timer= 0;
	int retry = 0;
	
	printk("linc cst3xx----------upgrade cst3xx begain------------\n");
	cst3xx_irq_disable(g_ts);
#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);
#endif

	mdelay(20);
START_FLOW:

	client->addr = CST3XX_I2C_ADDR;
	cst3xx_reset_ic(5+retry_timer);
	client->addr = CST3XX_I2C_ADDR_BOOT;
	ret = cst3xx_into_program_mode(client);
	if (ret < 0) {
		printk("linc cst3xx[cst3xx]into program mode failed.\n");
		retry_timer++;
		goto err_out;
	}
	ret = cst3xx_erase_program_area(client);
	if (ret<0) {
		printk("linc cst3xx[cst3xx]erase main area failed.\n");
		goto err_out;
	}
	ret = cst3xx_write_program_data(client, pdata);
	if (ret < 0) {
		printk("linc cst3xx[cst3xx]write program data into cstxxx failed.\n");
		goto err_out;
	}
    ret =cst3xx_check_checksum(client);
	if (ret < 0) {
		printk("linc cst3xx[cst3xx] after write program cst3xx_check_checksum failed.\n");
		goto err_out;
	}
	ret = cst3xx_exit_program_mode(client);
	if (ret < 0) {
		printk("linc cst3xx[cst3xx]exit program mode failed.\n");
		goto err_out;
	}
	client->addr = CST3XX_I2C_ADDR;
	cst3xx_reset_ic(30);
	cst3xx_irq_enable(g_ts);

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif

	printk("linc cst3xx hyn----------cst3xx_update_firmware  end------------\n");

	return 0;

err_out:
	if (retry < 40) {		
		mdelay(30);
		retry++;
		goto START_FLOW;
	}
	else {
		client->addr = CST3XX_I2C_ADDR;
		cst3xx_irq_enable(g_ts);
		return -1;
	}
}

static int cst3xx_update_judge( unsigned char *pdata, int strict)
{
	unsigned short ic_type, project_id;
	unsigned int fw_checksum, fw_version;
	const unsigned int *p;
	int i;
	unsigned char *pBuf;

	fw_checksum = 0x55;
	p = (const unsigned int *)pdata;
	for (i=0; i<(CST3XX_BIN_SIZE-4); i+=4) {
		fw_checksum += (*p);
		p++;
	}

	if (fw_checksum != (*p)) {
		printk("linc cst3xx[cst3xx]calculated checksum error:0x%x not equal 0x%x.\n", fw_checksum, *p);
		return -1;	//bad fw, so do not update
	}

	pBuf = &pdata[CST3XX_BIN_SIZE-16];

	project_id = pBuf[1];
	project_id <<= 8;
	project_id |= pBuf[0];

	ic_type = pBuf[3];
	ic_type <<= 8;
	ic_type |= pBuf[2];

	fw_version = pBuf[7];
	fw_version <<= 8;
	fw_version |= pBuf[6];
	fw_version <<= 8;
	fw_version |= pBuf[5];
	fw_version <<= 8;
	fw_version |= pBuf[4];

	fw_checksum = pBuf[11];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[10];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[9];
	fw_checksum <<= 8;
	fw_checksum |= pBuf[8];

	printk("linc cst3xx[cst3xx]the updating firmware:project_id:0x%04x,ic type:0x%04x,fw_version:0x%x,checksum:0x%x\n",
			project_id, ic_type, fw_version, fw_checksum);

#ifndef HYN_UPDATE_FIRMWARE_ONLINE
	printk("[cst3xx] HYN_UPDATE_FIRMWARE_ENABLE is not open.\n");
    return -1;
#endif

	if (strict > 0) {
		if (g_cst3xx_ic_checksum != fw_checksum){

#ifdef HYN_UPDATE_FIRMWARE_FORCE
    printk("[cst3xx] update firmware online force.\n");
    return 0;
#endif
			if (g_cst3xx_ic_version >fw_version){
				printk("[cst3xx]fw version(%d), ic version(%d).\n",fw_version, g_cst3xx_ic_version);
				return -1;
			}
		}else{
			printk("[cst3xx]fw checksum(0x%x), ic checksum(0x%x).\n",fw_checksum, g_cst3xx_ic_checksum);
			return -1;
		}

	}

	return 0;
}

static int cst3xx_boot_update_fw(struct i2c_client   *client,  unsigned char *pdata)
{
	int ret;
//	int retry = 0;
//	int flag = 0;

	/*while (retry++ < 3) {
		ret = cst3xx_firmware_info(client);
		if (ret == 0) {
			flag = 1;
			break;
		}
	}*/

	ret = cst3xx_update_judge(pdata, 1);
	if (ret < 0) {
		printk("linc cst3xx[cst3xx] no need to update firmware.\n");
		return 0;
	}


	ret = cst3xx_update_firmware(client, pdata);
	if (ret < 0){
		printk("linc cst3xx [cst3xx] update firmware failed.\n");
		return -1;
	}

    mdelay(50);

	ret = cst3xx_firmware_info(client);
	if (ret < 0) {
		printk("linc cst3xx [cst3xx] after update read version and checksum fail.\n");
		return -1;
	}


	return 0;
}
static int cst3xx_find_fw_idx( int project_id )
{
	int i = 0;
	if (0 != project_id) {
		for (i=0; i<ARRAY_SIZE(cst3xx_fw_grp); i++) {
			if (project_id == cst3xx_fw_grp[i].project_id){
				return i;
			}
		}
	}
	return -1;
}

static int hyn_boot_update_fw(struct i2c_client * client)
{
	unsigned char *ptr_fw;
	int ret;
	int retry=0;
	int proj_id=0;

	while (retry++ < 3) {
		ret = cst3xx_firmware_info(client);
		if (ret == 0) {
			break;
		}
	}

	proj_id=cst3xx_find_fw_idx(g_cst3xx_ic_project_id);
	printk("[cst3xx] cst3xx_firmware_info proj_id:%d.\r\n",proj_id);

	if(proj_id<0){
		printk("cst3xx: not find matched TP firmware,update default firmware !\n");
		return -1;
	}else{
		printk("cst3xx: find matched TP firmware(%s)!\n", cst3xx_fw_grp[proj_id].name);
		pcst3xx_update_firmware=cst3xx_fw_grp[proj_id].fw;

	}
	ptr_fw = pcst3xx_update_firmware;

	ret = cst3xx_boot_update_fw(client, ptr_fw);
    return ret;

}


#endif

static void cst3xx_release_all_finger(struct cst3xx_ts_data *ts)
{
	unsigned int finger_count = 0;
	mutex_lock(&ts->lock);
#ifdef ICS_SLOT_REPORT
	for(finger_count = 0; finger_count < 10; finger_count++)
	{
		input_mt_slot(ts->input_dev,finger_count);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
	}
	input_report_key(ts->input_dev,BTN_TOUCH, 0);
	input_report_key(ts->input_dev,BTN_TOOL_FINGER, 0);
#else
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ts->input_dev);
#endif
	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
	mutex_unlock(&ts->lock);
}

static void cst3xx_touch_report(struct work_struct *work)
{
	unsigned char buf[30];
	unsigned char i2c_buf[8];
	unsigned char key_status, key_id = 0, finger_id, sw;
	unsigned int  input_x = 0;
	unsigned int  input_y = 0;
	unsigned int  input_w = 0;
    unsigned char cnt_up, cnt_down;
	int   i, ret, idx;
	int cnt, i2c_len;
#if TPD_REVERT_XY
	unsigned int  input_xy_temp = 0;
#endif

	struct cst3xx_ts_data *ts=NULL;
#ifdef TRANSACTION_LENGTH_LIMITED
	int  len_1, len_2;
#endif
	ts=container_of(work,struct cst3xx_ts_data,work);

#ifdef CONFIG_TP_ESD_PROTECT
	if(i2c_lock_flag!=0) goto END;
	else i2c_lock_flag=1;
#endif

	//GTP_DEBUG_FUNC();

#ifdef HYN_GESTURE
    if((hyn_ges_wakeup_switch == 1)&&(tpd_halt == 1)){
        int tmp_c;
		int err;
      // unsigned char hyn_gesture_tmp = 0;
        buf[0] = 0xD0;
		buf[1] = 0x4C;
		err = cst3xx_i2c_read_register(g_i2c_client, buf, 1);
        if(err < 0)
        {
        	printk("linc cst3xxiic read gesture flag failed.\n");
        	goto OUT_PROCESS;
        }
		tmp_c=buf[0]&0x7F;

		printk(" linc [HYN_GESTURE] tmp_c =%d \n",tmp_c);
        if(1)
        {

			if(0 == hyn_lcd_flag){

                #if 1

				switch(tmp_c){
	                case 0x20:    //double
						 hyn_key_report(KEY_I);
						 hyn_gesture_c = (char)'*';
						 break;
	                case 0x3:     //left
	                     hyn_key_report(KEY_F);
						 hyn_gesture_c = (char)0xa1fb;
						 break;
	                case 0x1:     //right
	                     hyn_key_report(KEY_R);
						 hyn_gesture_c = (char)0xa1fa;
						 break;
	                case 0x2:     //up
	                     hyn_key_report(KEY_K);
						 hyn_gesture_c = (char)0xa1fc;
						 break;
					case 0x4:     //down
						 hyn_key_report(KEY_L);
						 hyn_gesture_c = (char)0xa1fd;
						 break;
	                case 0x5:     //O
	                     hyn_key_report(KEY_O);
						 hyn_gesture_c = (char)'O';
						 break;
	                case 0x0A:    //W
	                     hyn_key_report(KEY_W);
						 hyn_gesture_c = (char)'W';
						 break;
	                case 0x8: //M
	                case 0x09:
					case 0x0f:
			  	    case 0x10:
					case 0x15:
						 hyn_key_report(KEY_M);
						 hyn_gesture_c = (char)'M';
						 break;
	                case 0x07:            //E
	                     hyn_key_report(KEY_E);
						 hyn_gesture_c = (char)'E';
						 break;
	                case 0x6:
	                case 0x0e:      //C
	                     hyn_key_report(KEY_C); 
						 hyn_gesture_c = (char)'C';
						 break;
	                case 0x0C:            //S
	                     hyn_key_report(KEY_S);
						 hyn_gesture_c = (char)'S';
						 break;
	                case 0x0B:            //V
	                     hyn_key_report(KEY_V);
						 hyn_gesture_c = (char)'V';
						 break;
	                case 0x0D:            //Z
	                     hyn_key_report(KEY_Z);
						 hyn_gesture_c = (char)'Z';
				         break;
	                default:
	                     break;
					}
                #else
					switch(tmp_c){
	                case 0x20:    //double
						 hyn_gesture_tmp = (char)'*';
						 break;
	                case 0x3:     //left
						 hyn_gesture_tmp = 0xBB;
						 break;
	                case 0x1:     //right
						 hyn_gesture_tmp = 0xAA;
						 break;
	                case 0x2:     //up
						 hyn_gesture_tmp = 0xBA;
						 break;
		      		case 0x4:     //down
						 hyn_gesture_tmp =0xAB;
						 break;
	                case 0x5:     //O
	                case 0x14:
					case 0x16:

						 hyn_gesture_tmp = (char)'o';
						 break;
	                case 0x0A:    //W
						 hyn_gesture_tmp = (char)'w';
						 break;
	                case 0x8: //M
	                case 0x09:
					case 0x0f:
			  	    case 0x10:
					case 0x15:
						 hyn_gesture_tmp = (char)'m';
						 break;
	                case 0x07:            //E
	                case 0x11:
		      case 0x13:
						 hyn_gesture_tmp = 'e';
						 break;
	                case 0x6:       //C
	                case 0x0e:
						 hyn_gesture_tmp = (char)'c';
						 break;
	                case 0x0C:            //S
	                case 0x12:
						 hyn_gesture_tmp = (char)'s';
						 break;
	                case 0x0B:            //V
						 hyn_gesture_tmp = (char)'v';
						 break;
	                case 0x0D:            //Z
						 hyn_gesture_tmp = (char)'z';
				         break;
	                default:
	                     break;
						}


				printk(" linc [GSL_GESTURE] input report KEY_POWER.hyn_gesture_c = %c ,%d\n",hyn_gesture_c,hyn_gesture_c);
				for(i = 0; i < hyn_ges_num; i++){
						if(hyn_gesture_tmp == hyn_ges_cmd[i]){
			 				hyn_gesture_c = hyn_gesture_tmp;
							input_report_key(hyn_power_idev,KEY_POWER,1);
							input_sync(hyn_power_idev);
							input_report_key(hyn_power_idev,KEY_POWER,0);
							input_sync(hyn_power_idev);    
							}
					}
                #endif
			//hyn_lcd_flag = 1;

			}
            goto i2c_lock;
        }
    }
#endif


    key_status = 0;


	buf[0] = 0xD0;
	buf[1] = 0x00;
	ret = cst3xx_i2c_read_register(g_i2c_client, buf, 7);
	if(ret < 0) {
		printk(" linc iic read touch point data failed.\n");
		goto OUT_PROCESS;
	}

	if(buf[6] != 0xAB) {
		//printk(" linc data is not valid..\r\n");
		goto OUT_PROCESS;
	}

	if(buf[5] == 0x80) {
		key_status = buf[0];
		key_id = buf[1];
		goto KEY_PROCESS;
	}

	cnt = buf[5] & 0x7F;
	report_finger_flag=cnt;
	if(cnt > TPD_MAX_FINGERS) goto OUT_PROCESS;
	else if(cnt==0)     goto CLR_POINT;

	if(cnt == 0x01) {
		goto FINGER_PROCESS;
	}
	else {
		#ifdef TRANSACTION_LENGTH_LIMITED
		if((buf[5]&0x80) == 0x80) //key
		{
			i2c_len = (cnt - 1)*5 + 3;
			len_1   = i2c_len;
			for(idx=0; idx<i2c_len; idx+=6) {
			    i2c_buf[0] = 0xD0;
				i2c_buf[1] = 0x07+idx;

				if(len_1>=6) {
					len_2  = 6;
					len_1 -= 6;
				}
				else {
					len_2 = len_1;
					len_1 = 0;
				}

    			ret = cst3xx_i2c_read_register(g_i2c_client, i2c_buf, len_2);
    			if(ret < 0) goto OUT_PROCESS;

				for(i=0; i<len_2; i++) {
                   buf[5+idx+i] = i2c_buf[i];
				}
			}

			i2c_len   += 5;
			key_status = buf[i2c_len - 3];
			key_id     = buf[i2c_len - 2];
		}
		else {
			i2c_len = (cnt - 1)*5 + 1;
			len_1   = i2c_len;

			for(idx=0; idx<i2c_len; idx+=6) {
			    i2c_buf[0] = 0xD0;
				i2c_buf[1] = 0x07+idx;

				if(len_1>=6) {
					len_2  = 6;
					len_1 -= 6;
				}
				else {
					len_2 = len_1;
					len_1 = 0;
				}

    			ret = cst3xx_i2c_read_register(g_i2c_client, i2c_buf, len_2);
    			if (ret < 0) goto OUT_PROCESS;

				for(i=0; i<len_2; i++) {
                   buf[5+idx+i] = i2c_buf[i];
				}
			}
			i2c_len += 5;
		}
		#else
		if ((buf[5]&0x80) == 0x80) {
			buf[5] = 0xD0;
			buf[6] = 0x07;
			i2c_len = (cnt - 1)*5 + 3;
			ret = cst3xx_i2c_read_register(g_i2c_client, &buf[5], i2c_len);
			if (ret < 0)
				goto OUT_PROCESS;
			i2c_len += 5;
			key_status = buf[i2c_len - 3];
			key_id = buf[i2c_len - 2];
		}
		else {
			buf[5] = 0xD0;
			buf[6] = 0x07;
			i2c_len = (cnt - 1)*5 + 1;
			ret = cst3xx_i2c_read_register(g_i2c_client, &buf[5], i2c_len);
			if (ret < 0)
				goto OUT_PROCESS;
			i2c_len += 5;
		}
		#endif

		if (buf[i2c_len - 1] != 0xAB) {
			goto OUT_PROCESS;
		}
	}

    //both key and point
	if((cnt>0)&&(key_status&0x80))  {
        if(report_flag==0xA5) goto KEY_PROCESS;
	}

FINGER_PROCESS:

	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst3xx_i2c_write(g_i2c_client, i2c_buf, 3);
	if(ret < 0) {
		printk("linc cst3xx hyn send read touch info ending failed.\r\n"); 
		cst3xx_reset_ic(20);
	}

	idx = 0;
    cnt_up = 0;
    cnt_down = 0;
	for (i = 0; i < cnt; i++) {

		input_x = (unsigned int)((buf[idx + 1] << 4) | ((buf[idx + 3] >> 4) & 0x0F));
		input_y = (unsigned int)((buf[idx + 2] << 4) | (buf[idx + 3] & 0x0F));
		input_w = (unsigned int)(buf[idx + 4]);
		sw = (buf[idx] & 0x0F) >> 1;
		finger_id = (buf[idx] >> 4) & 0x0F;

	#if TPD_REVERT_XY
		input_xy_temp=input_x;
		input_x =input_y ;
		input_y = input_xy_temp;
	#endif

	#if TPD_REVERT_X
		input_x = TPD_MAX_X-input_x;
	#endif
	#if TPD_REVERT_Y
		input_y = TPD_MAX_Y-input_y;
	#endif
        #if 0
		input_x = TPD_MAX_X - input_x;
		input_y = TPD_MAX_Y - input_y;

	#endif

        //printk("linc cst3xxPoint x:%d, y:%d, id:%d, sw:%d. \n", input_x, input_y, finger_id, sw);

		if (sw == 0x03) {
			cst3xx_touch_down(ts->input_dev, finger_id, input_x, input_y, input_w);
            cnt_down++;
        }
		else {
            cnt_up++;
            #ifdef ICS_SLOT_REPORT
			cst3xx_touch_up(ts->input_dev, finger_id);
            #endif
        }
		idx += 5;
	}

	#ifdef ICS_SLOT_REPORT
	if((cnt_up>0) && (cnt_down==0))
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
	else
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
	#endif

    #ifndef ICS_SLOT_REPORT
    if((cnt_up>0) && (cnt_down==0))
        cst3xx_touch_up(ts->input_dev, 0);
    #endif

	if(cnt_down==0)  report_flag = 0;
	else report_flag = 0xCA;

    input_sync(ts->input_dev);
	goto END;

KEY_PROCESS:

	i2c_buf[0] = 0xD0;
	i2c_buf[1] = 0x00;
	i2c_buf[2] = 0xAB;
	ret = cst3xx_i2c_write(g_i2c_client, i2c_buf, 3);
	if (ret < 0) {
		printk("linc cst3xx hyn send read touch info ending failed.\r\n"); 
		cst3xx_reset_ic(20);
	}

    #ifdef GTP_HAVE_TOUCH_KEY
	if(key_status&0x80) {
		i = (key_id>>4)-1;
        if((key_status&0x7F)==0x03) {
			if((i==key_index)||(key_index==0xFF)) {
        		//cst3xx_touch_down(input_dev, 0, tpd_keys_dim_local[i][0], tpd_keys_dim_local[i][1], 50);
                input_report_key(ts->input_dev, touch_key_array[i], 1);
    			report_flag = 0xA5;
				key_index   = i;
			}
			else {
                input_report_key(ts->input_dev, touch_key_array[key_index], 0);
				key_index = 0xFF;
			}
		}
    	else {
			input_report_key(ts->input_dev, touch_key_array[i], 0);
            cst3xx_touch_up(ts->input_dev, 0);
			report_flag = 0;
			key_index = 0xFF;
    	}
	}
	#endif

    #ifdef TPD_HAVE_BUTTON
	if(key_status&0x80) {
		i = (key_id>>4)-1;
        if((key_status&0x7F)==0x03) {
			if((i==key_index)||(key_index==0xFF)) {
        		cst3xx_touch_down(ts->input_dev, 0, tpd_keys_dim_local[i][0], tpd_keys_dim_local[i][1], 50);
    			report_flag = 0xA5;
				key_index   = i;
			}
			else {

				key_index = 0xFF;
			}
		}
    	else {
            cst3xx_touch_up(ts->input_dev, 0);
			report_flag = 0;
			key_index = 0xFF;
    	}
	}

	#endif

	input_sync(ts->input_dev);
    goto END;

CLR_POINT:
#ifdef SLEEP_CLEAR_POINT
	cst3xx_release_all_finger(ts);
#endif

OUT_PROCESS:
	buf[0] = 0xD0;
	buf[1] = 0x00;
	buf[2] = 0xAB;
	ret = cst3xx_i2c_write(g_i2c_client, buf, 3);
	if (ret < 0) {
		printk(" linc send read touch info ending failed.\n");
		if((g_ts->work_mode==MODE_RESUME)&&(g_ts->esd_running==0))
		cst3xx_reset_ic(20);
	}

#ifdef HYN_GESTURE	
i2c_lock:
     key_status = 0;
	;
#endif

END:

#ifdef CONFIG_TP_ESD_PROTECT
	i2c_lock_flag = 0;
#endif

    cst3xx_irq_enable(ts);
	return;
}

#if   defined(CONFIG_FB)
static void cst3xx_ts_resume_work(struct work_struct *work)
{
    struct cst3xx_ts_data *ts_data = container_of(work,struct cst3xx_ts_data, resume_work);

    cst3xx_resume(ts_data);
}
/* frame buffer notifier block control the suspend/resume procedure */
struct notifier_block cst3xx_fb_notifier;
static int gtp_fb_notifier_callback(struct notifier_block *noti, unsigned long event, void *data)
{
	struct fb_event *ev_data = data;
	int *blank;
	struct cst3xx_ts_data *ts=NULL;
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts)
	{
		printk("cst3xx data pointor is null,no the device\n");
		return -ENODEV;
	}

    #ifndef FB_EARLY_EVENT_BLANK
        #error Need add FB_EARLY_EVENT_BLANK to fbmem.c
    #endif


	if (ev_data && ev_data->data && event == FB_EARLY_EVENT_BLANK ) {
		blank = ev_data->data;
        if (*blank == FB_BLANK_UNBLANK) {
			printk("Resume by fb notifier.");
			if (!work_pending (&ts->resume_work))
			{
				schedule_work (&ts->resume_work);
			}
        }
		else if (*blank == FB_BLANK_POWERDOWN) {
			printk("Suspend by fb notifier.");
			cancel_work_sync (&ts->resume_work);
			if(ts)
			{
				cst3xx_suspend(ts);
			}
			else
			{
				printk("cst3xx ts pointer is null, suspend fail\n");
			}
		}
    }
	else if (event == FB_R_EARLY_EVENT_BLANK)
	{
		if (!work_pending (&ts->resume_work))
		{
			schedule_work (&ts->resume_work);
		}
	}

	return 0;
}
#endif

#if defined(CONFIG_PM)

static int cst3xx_pm_suspend(struct device *dev)
{
	struct cst3xx_ts_data *ts=NULL;
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts);
	{
		printk("%s failed\n",__func__);
		return -1;
	}
    return cst3xx_suspend(ts);
}
static int cst3xx_pm_resume(struct device *dev)
{
	struct cst3xx_ts_data *ts=NULL;
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts)
	{
		return -1;
	}
	return cst3xx_resume(ts);
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops cst3xx_ts_pm_ops = {
	.suspend = cst3xx_pm_suspend,
	.resume = cst3xx_pm_resume,
};

#elif defined(CONFIG_HAS_EARLYSUSPEND)
/* earlysuspend module the suspend/resume procedure */
static void cst3xx_ts_early_suspend(struct early_suspend *h)
{
	struct cst3xx_ts_data *ts=NULL;
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts);
	{
		printk("%s failed\n",__func__);
		return;
	}
	cst3xx_suspend(ts);
}

static void cst3xx_ts_late_resume(struct early_suspend *h)
{
	struct cst3xx_ts_data *ts=NULL;
	ts=i2c_get_clientdata(g_i2c_client);
	if(!ts);
	{
		printk("%s failed\n",__func__);
		return;
	}

	cst3xx_resume(ts);
}
static struct early_suspend cst3xx_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = cst3xx_ts_early_suspend,
	.resume = cst3xx_ts_late_resume,
};
#endif

static s32 cst3xx_request_irq(struct cst3xx_ts_data *ts)
{
	s32 ret = -1;
	ret = request_irq(ts->client->irq, cst3xx_ts_irq_handler, IRQ_TYPE_EDGE_RISING, ts->client->name, ts);
	if (ret) {
		printk("Request IRQ failed!ERRNO:%d.", ret);
		gpio_direction_input(ts->pdata->irq_gpio);
		gpio_free(ts->pdata->irq_gpio);

		return -1;
	} else {
		cst3xx_irq_disable(ts);
		return 0;
	}
}
static s8 cst3xx_request_input_dev(struct cst3xx_ts_data *ts)
{
	s8 ret = -1;
//	u8 index = 0;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		printk("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
#ifdef ICS_SLOT_REPORT 
#if 1
    input_mt_init_slots(ts->input_dev, 10, INPUT_MT_DIRECT);
#else
    input_mt_init_slots(ts->input_dev, 10);
#endif
#else
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0,(5+1),0, 0);
#endif

    set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#ifdef HYN_GESTURE
	input_set_capability(ts->input_dev, EV_KEY, KEY_GES_REGULAR);
    input_set_capability(ts->input_dev, EV_KEY, KEY_GES_CUSTOM);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,    0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	//input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0,(5+1),0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TPD_MAX_X,0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TPD_MAX_Y,0, 0);

	ts->input_dev->name = TPD_DRIVER_NAME;
	ts->input_dev->phys = INPUT_DEV_PHYS;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk("Register %s input device failed", ts->input_dev->name);
		return -ENODEV;
	}

#if   defined(CONFIG_FB)
		INIT_WORK(&ts->resume_work,cst3xx_ts_resume_work);
		ts->fb_notif.notifier_call = gtp_fb_notifier_callback;
		fb_register_client(&ts->fb_notif);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
		ts->early_suspend.suspend=cst3xx_ts_early_suspend;
		ts->early_suspend.resume=cst3xx_ts_late_resume;
		register_early_suspend(&ts->early_suspend);
#endif

	return 0;
}

static s32 cst3xx_request_io_port(struct cst3xx_ts_data *ts)
{
	s32 ret = 0;

	if(!ts)
		return -ENODEV;

	ret = gpio_request(ts->pdata->irq_gpio, "HYN_INT_IRQ");
	if (ret < 0) {
		printk("Failed to request GPIO:%d, ERRNO:%d", ts->pdata->irq_gpio, ret);
		ret = -ENODEV;
		return ret;
	}
	ts->client->irq = gpio_to_irq(ts->pdata->irq_gpio);
	//g_irq_pin=ts->pdata->irq_gpio;

	ret = gpio_request(ts->pdata->reset_gpio, "HYN_RST_PORT");
	if (ret < 0) {
		printk("Failed to request GPIO:%d, ERRNO:%d", ts->pdata->reset_gpio, ret);
		ret = -ENODEV;
	}
	gpio_direction_output(ts->pdata->reset_gpio, 1);

	if (ret < 0) {
		gpio_free(ts->pdata->reset_gpio);
		gpio_free(ts->pdata->irq_gpio);
		printk("Failed to gpio_request INT and RST.");
	}

	return ret;
}

static int cst3xx_ts_get_dt_coords(struct device *dev, char *name,struct cst3xx_ts_platform_data *pdata)
{
	struct property *prop=NULL;
	struct device_node *np = dev->of_node;
	int rc;
	u32 coords[4];

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	rc = of_property_read_u32_array(np, name, coords,4);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "hynitron,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "hynitron,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}
static int cst3xx_parse_dt(struct device *dev,struct cst3xx_ts_platform_data *pdata)
{
	struct device_node *np;
	//struct property *prop;
	int ret = 0;
	//int i, read_cfg_num, tch_key_nu;
	//u32 temp_val,button_map[3];
	//u32 gesture_map[15];
	//char prop_name[24];

    if (!dev)
        return -ENODEV;

    np = dev->of_node;
	ret = cst3xx_ts_get_dt_coords(dev, "hynitron,panel-coords", pdata);
	if (ret && (ret != -EINVAL))
		return ret;

/*	ret = cst3xx_ts_get_dt_coords(dev, "hynitron,display-coords", pdata);
	if (ret)
		return ret;
*/
	pdata->reset_gpio= of_get_named_gpio_flags(np, "hynitron,rst-gpio",0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
	{
		printk("CST3XX Invalid GPIO, get reset-gpio failed\n");
		return pdata->reset_gpio;
	}
	pdata->irq_gpio = of_get_named_gpio_flags(np, "hynitron,irq-gpio",0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
	{
		printk("CST3XX Invalid GPIO, get irs-gpio failed\n");
		return pdata->irq_gpio;
	}
    if (!gpio_is_valid(pdata->irq_gpio) || !gpio_is_valid(pdata->reset_gpio) ) {
        printk("Invalid GPIO, irq-gpio:%d, rst-gpio:%d\n",
            pdata->irq_gpio, pdata->reset_gpio);
        return -EINVAL;
    }
    /*

//for reading touch key value from device tree
	pdata->have_touch_key=0;
	tch_key_nu=0;
	prop = of_find_property(np, "hynitron,button-map", NULL);
	if (prop) {
		tch_key_nu= prop->length / sizeof(temp_val);
		if (tch_key_nu > 3)
			return -EINVAL;
		pdata->button_map= devm_kzalloc(dev,sizeof(*pdata->button_map),GFP_KERNEL);
		if (!pdata->button_map)
			return -ENOMEM;
		pdata->button_map->map = devm_kzalloc(dev,sizeof(*pdata->button_map->map) *3, GFP_KERNEL);
		if (!pdata->button_map->map)
			return -ENOMEM;
		ret = of_property_read_u32_array(np,"hynitron,button-map", button_map,tch_key_nu);
		if (ret) {
			dev_err(dev, "Unable to read key codes\n");
			return ret;
		}
		for (i = 0; i < tch_key_nu; i++)
			pdata->button_map->map[i] = button_map[i];
		pdata->button_map->nbuttons = tch_key_nu;
		pdata->have_touch_key=1;
	}
	else
	{
		printk("GTP1x there is no touch key\n");
		//pdata->have_touch_key=0;
	}
//end
	prop = of_find_property(np, "hynitron,gesture-map", NULL);
	if (prop) {
		tch_key_nu = prop->length / sizeof(temp_val);
		if (tch_key_nu > 15)
			return -EINVAL;
		pdata->gesture_map= devm_kzalloc(dev,sizeof(*pdata->gesture_map),GFP_KERNEL);
		if (!pdata->gesture_map)
			return -ENOMEM;
		pdata->gesture_map->map = devm_kzalloc(dev,	sizeof(*pdata->gesture_map->map) *15, GFP_KERNEL);
		if (!pdata->gesture_map->map)
			return -ENOMEM;
		ret = of_property_read_u32_array(np,"hynitron,gesture-map", gesture_map,tch_key_nu);
		if (ret) {
			dev_err(dev, "Unable to read gesture codes\n");
			return ret;
		}
		for (i = 0; i < tch_key_nu; i++)
			pdata->gesture_map->map[i] = gesture_map[i];
		pdata->gesture_map->nbuttons = tch_key_nu;
	}

	prop = of_find_property(np, "hynitron,gesture-key", NULL);
	if (prop) {
		tch_key_nu = prop->length / sizeof(temp_val);
		if (tch_key_nu > 15)
			return -EINVAL;
		pdata->gesture_key= devm_kzalloc(dev,sizeof(*pdata->gesture_key),GFP_KERNEL);
		if (!pdata->gesture_key)
			return -ENOMEM;
		pdata->gesture_key->map = devm_kzalloc(dev,	sizeof(*pdata->gesture_key->map) *15, GFP_KERNEL);
		if (!pdata->gesture_key->map)
			return -ENOMEM;
		ret = of_property_read_u32_array(np,"hynitron,gesture-key", gesture_map,tch_key_nu);
		if (ret) {
			dev_err(dev, "Unable to read gesture codes\n");
			return ret;
		}
		for (i = 0; i < tch_key_nu; i++)
			pdata->gesture_key->map[i] = gesture_map[i];
		pdata->gesture_key->nbuttons = tch_key_nu;
	}
	dev_dbg(dev, "%d config data read from device tree.\n", read_cfg_num);
	*/

    return 0;

}

static int cst3xx_tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;

	struct cst3xx_ts_platform_data *pdata;
    struct cst3xx_ts_data *ts;

#ifdef GTP_HAVE_TOUCH_KEY
    s32 idx = 0;
#endif

    printk(" linc hyn is entering tpd_i2c_probe. \n");

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,sizeof(struct cst3xx_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev,"cst3xx Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}

		ret = cst3xx_parse_dt(&client->dev,pdata);
		if (ret)
			return ret;
	} else {
		pdata = client->dev.platform_data;
	}
	if (!pdata) {
		dev_err(&client->dev, "cst3xx invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk("I2C check functionality failed.");
        return -ENODEV;
    }
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
        printk("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }

	if(client->addr != CST3XX_I2C_ADDR)
	{
		client->addr = CST3XX_I2C_ADDR;
		printk("i2c_client_HYN->addr=0x%X.\n",client->addr);
	}

 	memset(ts, 0, sizeof(*ts));
	g_ts=ts;
	ts->power_on = POWER_OFF;
	ts->work_mode= MODE_RESUME;
	INIT_WORK(&ts->work, cst3xx_touch_report);
	ts->client = client;
    	ts->pdata = pdata;
	i2c_set_clientdata(client,ts);

	spin_lock_init(&ts->irq_lock);//(&irq_lock);
	mutex_init(&ts->lock);
	g_i2c_client = client;

	/*power on*/
	if(!ts->power_on){
		ret = hyn_cst3xx_power_ctrl(ts, POWER_ON);
		if (ret) {
			printk("ktf hyn_cst3xx power on failed\n");
			goto err_power_failed;
		}
		ts->power_on = POWER_ON;
	}

	ret = cst3xx_request_io_port(ts);
	if (ret < 0) {
		printk("cst3xx request IO port failed.");
		return ret;
	}
	mdelay(40); //20
	ret = cst3xx_i2c_test(client);
	if (ret < 0) {
		printk("linc cst3xx hyn i2c communication failed.\n");
		client->addr = CST3XX_I2C_ADDR_BOOT;
		printk("linc cst3xx hyn i2c communication test boot_addr=0x%X.\n",client->addr);
		ret = cst3xx_i2c_test(client);
		if(ret<0){
			printk("linc cst3xx hyn i2c communication test boot_addr failed.\n");
			goto i2c_test_failed;
		}else{		
#ifdef HYN_UPDATE_FIRMWARE_ONLINE	
			ret = cst3xx_update_firmware(client,pcst3xx_update_firmware);
			if(ret<0){
				printk("linc cst3xx hyn boot update null fail.\n");
				goto i2c_test_failed;	
			}
#endif
			client->addr = CST3XX_I2C_ADDR;
		}

	}

	ret = cst3xx_request_input_dev(ts);
	if (ret < 0) {
		printk("cst3xxrequest input dev failed");
	}

	g_input_dev=ts->input_dev;

	ret = cst3xx_request_irq(ts);
	if (ret < 0) {
		printk("cst3xx works in polling mode.");
	} else {
		printk("cst3xx works in interrupt mode.");
	}

#ifdef 	HYN_UPDATE_FIRMWARE_ONLINE
	ret = hyn_boot_update_fw(client);
	if(ret < 0){
		printk("linc cst3xx hyn_boot_update_fw failed.\n");
		return -1;
	}
#else
	ret = cst3xx_firmware_info(client);
	if(ret < 0){
		printk("linc cst3xx cst3xx_firmware_info failed.\n");
		return -1;
	}
#endif

#ifdef ANDROID_TOOL_SURPORT
	ret = cst3xx_proc_fs_init();
	if(ret < 0) {
		printk("linc cst3xx hyn create cst3xx proc fs failed.\n");
	}
#ifdef HYN_SYSFS_NODE_EN
    hyn_create_sysfs(client);
#endif
	memset(mtk_tp_version,0,128);
    proc_create(PROC_TP_VERSION, 0, NULL, &tp_version_proc_fops1);
#endif

#ifdef HYN_GESTURE
//	for(Hyn_key_set=0;Hyn_key_set<HYN_TPD_GES_COUNT;Hyn_key_set++)
//	input_set_capability(g_input_dev, EV_KEY,  tpd_keys_gesture[Hyn_key_set]);
	enable_irq_wake(client->irq);
#endif

#ifdef CONFIG_TP_ESD_PROTECT

	cst3xx_esd_workqueue = create_singlethread_workqueue("cst3xx_esd_workqueue");
	if (cst3xx_esd_workqueue == NULL)
		printk("linc cst3xxcreate cst2xx_esd_workqueue failed!");
	else{
		cst3xx_init_esd_protect();
		cst3xx_esd_switch(SWITCH_ESD_ON);
	}
#endif

#ifdef GTP_HAVE_TOUCH_KEY
    for (idx=0; idx<TPD_KEY_COUNT; idx++) {
        input_set_capability(g_input_dev, EV_KEY, touch_key_array[idx]);
    }
#endif


#ifdef HYN_GESTURE

#if 0

	retval = gesture_proc_init();
	if (retval)
	{
		printk("linc cst3xxCreate gesture_proc failed!\n.");
	}

    hyn_gesture_init();

#endif

input_set_capability(g_input_dev, EV_KEY, KEY_POWER);
input_set_capability(g_input_dev, EV_KEY, KEY_C);
input_set_capability(g_input_dev, EV_KEY, KEY_M);
input_set_capability(g_input_dev, EV_KEY, KEY_E);
input_set_capability(g_input_dev, EV_KEY, KEY_O);
input_set_capability(g_input_dev, EV_KEY, KEY_W);
input_set_capability(g_input_dev, EV_KEY, KEY_S);
input_set_capability(g_input_dev, EV_KEY, KEY_UP);
input_set_capability(g_input_dev, EV_KEY, KEY_LEFT);
input_set_capability(g_input_dev, EV_KEY, KEY_RIGHT);
input_set_capability(g_input_dev, EV_KEY, KEY_DOWN);
input_set_capability(g_input_dev, EV_KEY, KEY_U);

#endif

#ifdef HYN_UPDATE_FIRMWARE_TEST
	printk( "hyn_ts_probe () : queue hyn_update_firmware_workqueue\n");

	INIT_DELAYED_WORK(&hyn_update_firmware_work, hyn_update_firmware_worker);
	hyn_update_firmware_workqueue = create_singlethread_workqueue("hyn_update_firmware_workqueue");
	queue_delayed_work(hyn_update_firmware_workqueue, &hyn_update_firmware_work, 5000);
#endif


	cst3xx_reset_ic(30);

	cst3xx_irq_enable(ts);

	printk(" linc hyn is endding tpd_i2c_probe .\n");

	return 0;

err_power_failed:
    if (gpio_is_valid(ts->pdata->irq_gpio))
       gpio_free(ts->pdata->irq_gpio);

i2c_test_failed:
       gpio_free(ts->pdata->irq_gpio);
	hyn_cst3xx_power_ctrl(ts, POWER_OFF);
	return -1;

}

static int cst3xx_tpd_remove(struct i2c_client *client)
{
	struct cst3xx_ts_data *ts;
	printk(" linc cst3xx removed.\n");	
	ts=i2c_get_clientdata(client);
#ifdef HYN_GESTURE
	disable_irq_wake(client->irq);
#endif

#if defined(CONFIG_FB)
	fb_unregister_client(&ts->fb_notif);
#endif
	disable_irq(ts->pdata->irq_gpio);
	/* Power off */
	if(ts->power_on){
		hyn_cst3xx_power_ctrl(ts, POWER_OFF);//remove
		ts->power_on = POWER_OFF;
	}
	sysfs_remove_group(k_obj, &hyn_attribute_group);

    free_irq(ts->pdata->irq_gpio, ts->pdata);

	if (cst3xx_wq) {
		destroy_workqueue(cst3xx_wq);
		cst3xx_wq = NULL;
		}

	if (gpio_is_valid(ts->pdata->irq_gpio))
		gpio_free(ts->pdata->irq_gpio);
	if (gpio_is_valid(ts->pdata->reset_gpio))
		gpio_free(ts->pdata->reset_gpio);
	if (ts->client&& ts->client->irq)
		free_irq(ts->client->irq, ts->client);
	//Erobbing add for distinguish resistance TP or not

	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);

	return 0;
	}
/*
static int cst3xx_tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE_NAME);
	return 0;
}
*/

static const struct i2c_device_id cst3xx_tpd_id[] = {{TPD_DRIVER_NAME,0},{}};

//static unsigned short force[] = { 0, CST3XX_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END };
//static const unsigned short *const forces[] = { force, NULL };

/*
	static struct i2c_board_info __initdata cst3xx_i2c_tpd = {
		I2C_BOARD_INFO(TPD_DEVICE_NAME, CST3XX_I2C_ADDR)
	};
*/
/*
static void cst3xx_enter_sleep(struct i2c_client *client)
{
	int ret;
	int retry = 0;
	unsigned char buf[2];

    buf[0] = 0xD1;
	buf[1] = 0x05;
	while (retry++ < 5) {
		ret = cst3xx_i2c_write(client, buf, 2);
		if (ret > 0)
			return;
		mdelay(2);
	}

	return;
}*/

static int cst3xx_resume(struct cst3xx_ts_data *ts)
{
	int ret;
	if(!ts->power_on){
		ret = hyn_cst3xx_power_ctrl(ts, POWER_ON);//resume
		if (ret){
			printk("%s:ktf power on failed",__func__);
		}
		ts->power_on = POWER_ON;
	}
	else
	{
		return 0;
	}
	printk("ktf linc cst3xx wake up.\n");

#ifdef SLEEP_CLEAR_POINT
	cst3xx_release_all_finger(ts);
#endif

#ifdef HYN_GESTURE

    if(hyn_ges_wakeup_switch == 1){
        u8 buf[4];
        //close gesture detect
        buf[0] = 0xD0;
		buf[1] = 0x4C;
		buf[2] = 0x00;
		cst3xx_i2c_write(g_i2c_client, buf, 3);
        tpd_halt=0;
		hyn_lcd_flag = 1;

		printk(" linc tpd-gesture detect is closed .\n");
    }
	else
#endif

	// cst3xx_firmware_info(g_i2c_client); //no need to read firmware_info

#ifdef HYN_UPDATE_FIRMWARE_TEST
	if(update_test_running_on==0){
		queue_delayed_work(hyn_update_firmware_workqueue, &hyn_update_firmware_work, 3000);
		update_test_running_on=1;
	}
#endif

	cst3xx_reset_ic(30);

	//mdelay(20);

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_ON);
#endif

	cst3xx_irq_enable(ts);
	ts->work_mode=MODE_RESUME;

	printk(" linc cst3xx wake up done.\n");

	return 0;

}
static int cst3xx_suspend(struct cst3xx_ts_data *ts)
{
	int ret;
	if(ts->power_on == POWER_OFF)
	{
		return 0;
	}
	printk(" ktf linc cst3xx enter sleep.\n");
	ts->work_mode=MODE_SUSPEND;

#ifdef HYN_GESTURE
    if(hyn_ges_wakeup_switch == 1){

//		int err;
		u8 buf[4];
        hyn_lcd_flag = 0;
        mdelay(10);
		tpd_halt=1;

        buf[0] = 0xD0;
		buf[1] = 0x4C;
		buf[2] = 0x80;
		cst3xx_i2c_write(g_i2c_client, buf, 3);
		printk(" linc tpd-gesture detect is opened \n");

        mdelay(10);
        return -1;
    }
#endif

#ifdef CONFIG_TP_ESD_PROTECT
	if(cst3xx_esd_workqueue!=NULL)
	cst3xx_esd_switch(SWITCH_ESD_OFF);

#endif

#ifdef HYN_UPDATE_FIRMWARE_TEST
	if(update_test_running_on==1){
	cancel_delayed_work(&hyn_update_firmware_work);
	update_test_running_on=0;
	}
#endif

    cst3xx_irq_disable(ts);

    //cst3xx_enter_sleep(g_i2c_client);
	mdelay(30);
	gpio_direction_output(ts->pdata->reset_gpio, 0);

	if(ts->power_on){
		ret = hyn_cst3xx_power_ctrl(ts, POWER_OFF); //suspend
		if (ret){
			printk("%s:ktf power off failed",__func__);
		}
		ts->power_on = POWER_OFF;
	}

	printk(" linc cst3xx enter sleep done.\n");

	return 0;

}

static const struct of_device_id tpd_of_match[] = {
	{.compatible = "hynitron,cst1xx"},
	{},
};

MODULE_DEVICE_TABLE(of, tpd_of_match);

static struct i2c_driver cst3xx_ts_driver = {

  .driver = {
    		.name = TPD_DRIVER_NAME,		
#ifdef CONFIG_OF
		   .of_match_table = of_match_ptr(tpd_of_match),
#endif
#if !defined(CONFIG_FB) && defined(CONFIG_PM)
		   .pm = &cst3xx_ts_pm_ops,
#endif
  },
  .probe    = cst3xx_tpd_probe,
  .remove   = cst3xx_tpd_remove,
  .id_table = cst3xx_tpd_id,
//  .detect   = cst3xx_tpd_detect,
  //.address_list = (const unsigned short *)forces,
};
/* called when loaded into kernel */
static int __init cst3xx_ts_init(void)
{
	printk(" linc hyn is entering cst3xx_ts_init.\n");

	cst3xx_wq = create_singlethread_workqueue("cst3xx_wq");
	if (!cst3xx_wq) {
		printk("Creat workqueue failed.");
		return -ENOMEM;
	}
	return i2c_add_driver(&cst3xx_ts_driver);
}

/* should never be called */
static void __exit cst3xx_ts_exit(void)
{
	printk(" linc hyn is entering cst3xx_ts_exit.\n");
	i2c_del_driver(&cst3xx_ts_driver);
	if (cst3xx_wq) {
		destroy_workqueue(cst3xx_wq);
	}
}

late_initcall(cst3xx_ts_init);
module_exit(cst3xx_ts_exit);

