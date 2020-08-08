#include <linux/of.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>   //for DECLARE_TASKLET compile
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>  //add by zhuguanglei 
#include <linux/platform_device.h>
#include <linux/hct_include/hct_project_all_config.h>

#ifdef __HCT_CUSTOMKEY_F2_SUPPORT__
#if __HCT_CUSTOMKEY_F2_SUPPORT__

#if 0 //def  __HCT_CUSTOMKEY_CHANGE__
#undef KEY_F2
#define KEY_F2		KEY_CAMERA
#endif

#define CUSTOMKEY_F2_GPIO_NAME  "customkey_f2_gpio"           
#define CUSTOMKEY_F2_NUM 		KEY_F2

static int hct_customkey_f2_irqnum;

static unsigned int hct_customkey_f2_gpio_num;

//static unsigned int hct_customkey_state_pin;
//static struct pinctrl *hct_customkey_pinctrl1;
//static struct pinctrl_state *hct_customkey_pins_eint_int;
//static struct pinctrl_state *pins_default;

static void kpd_customkey_f2_handler(unsigned long data);
static DECLARE_TASKLET(kpd_customkey_f2_tasklet,kpd_customkey_f2_handler,0);

extern void customkey_handler(bool pressed, int key_value);					//In kpd.c achieve add by zhuguanglei
//extern int hct_parse_dts(struct device_node *node, const char *gpio_name, bool gpio_mode);
#if 0
static unsigned int hct_parse_dts1(struct device_node *node, const char *gpio_name, bool gpio_mode)
{
	unsigned int gpio_num = 0;
//	struct gpio_desc *desc;
//	int ret = 0;
		if (node)
	{
		if (of_property_read_u32_index(node, gpio_name, 1, &gpio_num)) {
            pr_err("f2 read_irq_registration fail~~~\n");
        }

//		gpio_num = of_get_named_gpio(node, gpio_name, 0);
		if (gpio_num < 0)
		{
			printk("%s: f2 of_get_named_gpio fail. \n", __func__);
			return -1;
		}
		else //获取GPIO成功
		{
			printk("%s: f2 of_get_named_gpio GPIO is %d.\n", __func__, gpio_num);



		}
	}
	else
	{
		printk("%s: f2  get gpio num fail. \n", __func__);
		
	}


return gpio_num;
}
#endif
static int hct_parse_dts(struct device_node *node, const char *gpio_name, bool gpio_mode)
{
	int gpio_num = 0;
	struct gpio_desc *desc;
	int ret = 0;
	unsigned int  debounce11;
	
	if (node)
	{
		gpio_num = of_get_named_gpio(node, gpio_name, 0);
		if (gpio_num < 0)
		{
			printk("%s: of_get_named_gpio fail. \n", __func__);
			return -1;
		}
		else //获取GPIO成功
		{
			printk("%s: of_get_named_gpio GPIO is %d.\n", __func__, gpio_num);
			desc = gpio_to_desc(gpio_num);
			if (!desc)
			{
				printk("%s: gpio_desc is null.\n", __func__);
				return -1;
			}
			else //获取描述成功
				printk("%s: gpio_desc is not null.\n", __func__);

			if (gpio_is_valid(gpio_num))
				printk("%s: gpio number %d is valid. \n", __func__ ,gpio_num);

			ret = gpio_request(gpio_num, gpio_name);
			if (ret)
			{
				printk("%s: gpio_request fail. \n", __func__);
				return -1;
			}
			else //gpio_request 成功
			{
				if (!gpio_mode) //中断模式
				{
					ret = gpio_direction_input(gpio_num);
					if (ret)
					{
						printk("%s: gpio_direction_input fail. \n", __func__);
						return -1;
					}
					of_property_read_u32_index(node, "debounce", 1, &debounce11);
	                gpio_set_debounce(gpio_num,debounce11*1000);
 	      	        printk("zcy add for gpio_num = (%d) debug debounce = (%d)\n",gpio_num,debounce11);
				}
				else //GPIO 模式
				{
					ret = gpio_direction_output(gpio_num, 1);
					if (ret)
					{
						printk("%s: gpio_direction_output failed. \n", __func__);
						return -1;
					}

					gpio_set_value(gpio_num, 0);
					printk("%s: gpio_get_value =%d. \n", __func__, gpio_get_value(gpio_num));
				}

				return gpio_num; //返回GPIO num
			}
		}
	}
	else
	{
		printk("%s: get gpio num fail. \n", __func__);
		return -1;
	}
}
static void kpd_customkey_f2_handler(unsigned long data)
{
	bool pressed;
	int customkey_f2_state = 0;

	printk("%s: f2 enter. \n", __func__);

	customkey_f2_state = gpio_get_value(hct_customkey_f2_gpio_num);//__gpio_get_value(hct_customkey_f2_gpio_num);
	if(customkey_f2_state < 0)
		return;
	printk("%s: f2_num = (%d) customkey_f2_state = (%d) gpio_get_value end. \n", __func__,hct_customkey_f2_gpio_num,customkey_f2_state);

	pressed = !customkey_f2_state;

    customkey_handler(pressed, CUSTOMKEY_F2_NUM);

	printk("%s: %s HW keycode(%u) using EINT =%d.\n", __func__, pressed ? "pressed" : "released", CUSTOMKEY_F2_NUM, customkey_f2_state);

	if (pressed)
	{
		printk("%s: key %s. \n", __func__, pressed ? "pressed" : "released");
		irq_set_irq_type(hct_customkey_f2_irqnum, IRQF_TRIGGER_RISING);
	}
	else
	{
		printk("%s: key %s. \n", __func__, pressed ? "pressed" : "released");
		irq_set_irq_type(hct_customkey_f2_irqnum, IRQF_TRIGGER_FALLING);
	}

	enable_irq(hct_customkey_f2_irqnum);
}

static irqreturn_t hct_customkey_f2_isr(int irqnum, void *data)
{
	printk("%s: enter.\n", __func__);

	disable_irq_nosync(hct_customkey_f2_irqnum);
	tasklet_schedule(&kpd_customkey_f2_tasklet);
	return IRQ_HANDLED;
}

/*
 初始化, 获取dts中相关的配置信息
 pdev 由 hct_intercom_plat_probe 传进来
 */
int hct_customkey_f2_init(struct platform_device *pdev)
{
	int retval;
	//int err = -EIO;
	struct device_node *node;

    printk("+++++++++++ %s: begin +++++++++++\n",__func__);

	/*get the node*/
	node = of_find_compatible_node(NULL, NULL, "mediatek,hct_customkey2");
	if (node) {
        printk("%s: node = (%d) not null.\n", __func__,node);

		hct_customkey_f2_irqnum = irq_of_parse_and_map(node, 0);
		printk("%s: f2  hct_customkey_f2_irqnum = %d\n", __func__, hct_customkey_f2_irqnum);
	}
	else {
		printk("%s: f2  cannot get the node: 'mediatek,hct_customkey'.\n", __func__);
		return -ENODEV;
	}
	



	//call key_parse_dts_and_get_pin_status to get gpio num 
	hct_customkey_f2_gpio_num = hct_parse_dts(node, CUSTOMKEY_F2_GPIO_NAME, false);


	if(hct_customkey_f2_gpio_num < 0)
	{   //获取GPIO num 失败
		printk("%s: hct_parse_dts get gpio fail.\n", __func__);
		return -ENODEV;	
	}
	else
	{	//打印出GPIO num的值
		printk("%s = %d. \n",CUSTOMKEY_F2_GPIO_NAME, hct_customkey_f2_gpio_num);
	}

	/*irq register*/
	retval = request_irq(hct_customkey_f2_irqnum, hct_customkey_f2_isr, IRQF_TRIGGER_FALLING, "customkeyf2", NULL);
	if (retval != 0) {
		printk("%s: request_irq fail, ret %d, irqnum %d.\n", __func__, retval, hct_customkey_f2_irqnum);
		return retval;
	}
	enable_irq(hct_customkey_f2_irqnum);

	printk("+++++++++++ hct_customkey_f2_init end +++++++++++");
	return retval;
}

#endif
#endif

