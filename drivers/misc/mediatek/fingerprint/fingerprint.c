#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/compat.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#ifdef CONFIG_OF
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#endif
#include <linux/gpio.h>
#include <linux/spi/spi.h>
//#include "mtk_spi.h"
#include "fingerprint.h"

#include <linux/hct_include/hct_project_all_config.h>


DECLARE_WAIT_QUEUE_HEAD(finger_init_waiter);

int hct_finger_probe_isok = 0;//add for hct finger jianrong

struct pinctrl *hct_finger_pinctrl;
struct pinctrl_state *hct_finger_reset_high,*hct_finger_reset_low,*hct_finger_spi0_mi_as_spi0_mi,*hct_finger_spi0_mi_as_gpio,
*hct_finger_spi0_mo_as_spi0_mo,*hct_finger_spi0_mo_as_gpio,*hct_finger_spi0_clk_as_spi0_clk,*hct_finger_spi0_clk_as_gpio,
*hct_finger_spi0_cs_as_spi0_cs,*hct_finger_spi0_cs_as_gpio,*hct_finger_eint_pull_down,*hct_finger_eint_pull_up,*hct_finger_eint_pull_dis;

int hct_finger_get_gpio_info(struct platform_device *pdev)
{
	struct device_node *node;
	int ret;
	node = of_find_compatible_node(NULL, NULL, "mediatek,hct_finger");
	printk("node.name <%s> full name <%s>\n",node->name,node->full_name);

	wake_up_interruptible(&finger_init_waiter);

	hct_finger_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(hct_finger_pinctrl)) {
		ret = PTR_ERR(hct_finger_pinctrl);
		dev_err(&pdev->dev, "hct_finger cannot find pinctrl and ret = (%d)\n",ret);
		return ret;
	}

	printk("[%s] hct_finger_pinctrl+++++++++++++++++\n",pdev->name);

	hct_finger_reset_high = pinctrl_lookup_state(hct_finger_pinctrl, "finger_reset_en1");
	if (IS_ERR(hct_finger_reset_high)) {
		ret = PTR_ERR(hct_finger_reset_high);
		dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_reset_high!\n");
		return ret;
	}
	hct_finger_reset_low = pinctrl_lookup_state(hct_finger_pinctrl, "finger_reset_en0");
	if (IS_ERR(hct_finger_reset_low)) {
		ret = PTR_ERR(hct_finger_reset_low);
		dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_reset_low!\n");
		return ret;
	}
	hct_finger_spi0_mi_as_spi0_mi = pinctrl_lookup_state(hct_finger_pinctrl, "finger_spi0_mi_as_spi0_mi");
	if (IS_ERR(hct_finger_spi0_mi_as_spi0_mi)) {
		ret = PTR_ERR(hct_finger_spi0_mi_as_spi0_mi);
		dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_spi0_mi_as_spi0_mi!\n");
		return ret;
	}
	hct_finger_spi0_mi_as_gpio = pinctrl_lookup_state(hct_finger_pinctrl, "finger_spi0_mi_as_gpio");
	if (IS_ERR(hct_finger_spi0_mi_as_gpio)) {
		ret = PTR_ERR(hct_finger_spi0_mi_as_gpio);
		dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_spi0_mi_as_gpio!\n");
		return ret;
	}
	hct_finger_spi0_mo_as_spi0_mo = pinctrl_lookup_state(hct_finger_pinctrl, "finger_spi0_mo_as_spi0_mo");
	if (IS_ERR(hct_finger_spi0_mo_as_spi0_mo)) {
		ret = PTR_ERR(hct_finger_spi0_mo_as_spi0_mo);
		dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_spi0_mo_as_spi0_mo!\n");
		return ret;
	}
	hct_finger_spi0_mo_as_gpio = pinctrl_lookup_state(hct_finger_pinctrl, "finger_spi0_mo_as_gpio");
	if (IS_ERR(hct_finger_spi0_mo_as_gpio)) {
		ret = PTR_ERR(hct_finger_spi0_mo_as_gpio);
		dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_spi0_mo_as_gpio!\n");
		return ret;
	}
	hct_finger_spi0_clk_as_spi0_clk = pinctrl_lookup_state(hct_finger_pinctrl, "finger_spi0_clk_as_spi0_clk");
	if (IS_ERR(hct_finger_spi0_clk_as_spi0_clk)) {
		ret = PTR_ERR(hct_finger_spi0_clk_as_spi0_clk);
		dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_spi0_clk_as_spi0_clk!\n");
		return ret;
	}
	hct_finger_spi0_clk_as_gpio = pinctrl_lookup_state(hct_finger_pinctrl, "finger_spi0_clk_as_gpio");
	if (IS_ERR(hct_finger_spi0_clk_as_gpio)) {
		ret = PTR_ERR(hct_finger_spi0_clk_as_gpio);
		dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_spi0_clk_as_gpio!\n");
		return ret;
	}
	hct_finger_spi0_cs_as_spi0_cs = pinctrl_lookup_state(hct_finger_pinctrl, "finger_spi0_cs_as_spi0_cs");
	if (IS_ERR(hct_finger_spi0_cs_as_spi0_cs)) {
		ret = PTR_ERR(hct_finger_spi0_cs_as_spi0_cs);
		dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_spi0_cs_as_spi0_cs!\n");
		return ret;
	}
	hct_finger_spi0_cs_as_gpio = pinctrl_lookup_state(hct_finger_pinctrl, "finger_spi0_cs_as_gpio");
	if (IS_ERR(hct_finger_spi0_cs_as_gpio)) {
		ret = PTR_ERR(hct_finger_spi0_cs_as_gpio);
		dev_err(&pdev->dev, " Cannot find hct_finger pinctrl hct_finger_spi0_cs_as_gpio!\n");
		return ret;
	}

    	hct_finger_eint_pull_down = pinctrl_lookup_state(hct_finger_pinctrl, "finger_eint_pull_down");
    	if (IS_ERR(hct_finger_eint_pull_down)) {
    		ret = PTR_ERR(hct_finger_eint_pull_down);
    		dev_err(&pdev->dev, " Cannot find fp pinctrl hct_finger_eint_pull_down!\n");
    		return ret;
    	}
    	hct_finger_eint_pull_up= pinctrl_lookup_state(hct_finger_pinctrl, "finger_eint_pull_up");
    	if (IS_ERR(hct_finger_eint_pull_up)) {
    		ret = PTR_ERR(hct_finger_eint_pull_up);
    		dev_err(&pdev->dev, " Cannot find fp pinctrl hct_finger_eint_pull_up!\n");
    		return ret;
    	}
    
    	hct_finger_eint_pull_dis = pinctrl_lookup_state(hct_finger_pinctrl, "finger_eint_pull_dis");
    	if (IS_ERR(hct_finger_eint_pull_dis)) {
    		ret = PTR_ERR(hct_finger_eint_pull_dis);
    		dev_err(&pdev->dev, " Cannot find fp pinctrl hct_finger_eint_pull_dis!\n");
    		return ret;
    	}

	printk("hct_finger get gpio info ok--------\n");
	return 0;
}

int hct_finger_set_reset(int cmd)
{

	if(IS_ERR(hct_finger_reset_low)||IS_ERR(hct_finger_reset_high))
	{
		 pr_err( "err: hct_finger_reset_low or hct_finger_reset_high is error!!!");
		 return -1;
	}	
	

	switch (cmd)
	{
	case 0 : 		
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_reset_low);
	break;
	case 1 : 		
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_reset_high);
	break;
	}
	return 0;
}

int hct_finger_set_irq(int cmd)
{
        if(IS_ERR(hct_finger_eint_pull_down)||IS_ERR(hct_finger_eint_pull_up)||IS_ERR(hct_finger_eint_pull_dis))
	{
		 pr_err( "err: hct_finger_int_as_gpio is error!!!!");
		 return -1;
	}	

	switch (cmd)
	{
	case 0 : 		
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_eint_pull_down);
	break;
	case 1 : 		
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_eint_pull_up);
	break;
	case 2 : 		
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_eint_pull_dis);
	break;
	}
	return 0;


}

/*********************blestech(start  2018-1-19)***********/

unsigned int hct_finger_get_irqnum(void){
    struct device_node *node = NULL;
    //MALOGF("start");
    node = of_find_compatible_node(NULL, NULL, "mediatek,hct_finger");
    return irq_of_parse_and_map(node, 0);
}

unsigned int hct_finger_get_irq_gpio(void){
    struct device_node *node = NULL;
    //MALOGF("start");
    node = of_find_compatible_node(NULL, NULL, "mediatek,hct_finger");
    return of_get_named_gpio(node, "int-gpio", 0);
}

unsigned int hct_finger_get_reset_gpio(void){
    struct device_node *node = NULL;
    //MALOGF("start");
    node = of_find_compatible_node(NULL, NULL, "mediatek,hct_finger");
    return of_get_named_gpio(node, "reset-gpio", 0);
}

int hct_finger_set_spi_mode(int cmd)
{

	if(IS_ERR(hct_finger_spi0_clk_as_gpio)||IS_ERR(hct_finger_spi0_cs_as_gpio)||IS_ERR(hct_finger_spi0_mi_as_gpio) \
		||IS_ERR(hct_finger_spi0_mo_as_gpio)||IS_ERR(hct_finger_spi0_clk_as_spi0_clk)||IS_ERR(hct_finger_spi0_cs_as_spi0_cs) \
		||IS_ERR(hct_finger_spi0_mi_as_spi0_mi)||IS_ERR(hct_finger_spi0_mo_as_spi0_mo))
	{
		 pr_err( "err: hct_finger_reset_low or hct_finger_reset_high is error!!!");
		 return -1;
	}	

	

	switch (cmd)
	{
	case 0 : 		
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_spi0_clk_as_gpio);
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_spi0_cs_as_gpio);
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_spi0_mi_as_gpio);
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_spi0_mo_as_gpio);
	break;
	case 1 : 		
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_spi0_clk_as_spi0_clk);
#if defined(CONFIG_FINGERPRINT_CHIPONE_REE)
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_spi0_cs_as_gpio);
#else
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_spi0_cs_as_spi0_cs);
#endif
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_spi0_mi_as_spi0_mi);
		pinctrl_select_state(hct_finger_pinctrl, hct_finger_spi0_mo_as_spi0_mo);
	break;
	}
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id hct_finger_match[] = {
	{ .compatible = "mediatek,hct_finger", },
	{}
};
MODULE_DEVICE_TABLE(of, hct_finger_match);
#endif

static struct platform_device *hct_finger_plat = NULL;


void hct_waite_for_finger_dts_paser(void)
{
    if(hct_finger_plat==NULL)
    {
        wait_event_interruptible_timeout(finger_init_waiter, hct_finger_plat != NULL, 3 * HZ);
    }
    else
	return;
	
}

int hct_get_max_finger_spi_cs_number(void)
{
    return MAX_FINGER_CHIP_CS_NUMBER;
}

int hct_finger_plat_probe(struct platform_device *pdev) {
	hct_finger_plat = pdev;
	printk("hct_finger_plat_probe entry\n");
	hct_finger_get_gpio_info(pdev);
	return 0;
}

int hct_finger_plat_remove(struct platform_device *pdev) {
	hct_finger_plat = NULL;
	return 0;
} 


#ifndef CONFIG_OF
static struct platform_device hct_finger_dev = {
	.name		  = "hct_finger",
	.id		  = -1,
};
#endif


static struct platform_driver hct_finger_pdrv = {
	.probe	  = hct_finger_plat_probe,
	.remove	 = hct_finger_plat_remove,
	.driver = {
		.name  = "hct_finger",
		.owner = THIS_MODULE,			
#ifdef CONFIG_OF
		.of_match_table = hct_finger_match,
#endif
	}
};


static int __init hct_finger_init(void)
{

#ifndef CONFIG_OF
    int retval=0;
    retval = platform_device_register(&hct_finger_dev);
    if (retval != 0){
        return retval;
    }
#endif
    if(platform_driver_register(&hct_finger_pdrv))
    {
    	printk("failed to register driver");
    	return -ENODEV;
    }
    
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit hct_finger_exit(void)
{
	platform_driver_unregister(&hct_finger_pdrv);
}


rootfs_initcall(hct_finger_init);
module_exit(hct_finger_exit);

MODULE_AUTHOR("Jay_zhou");
MODULE_DESCRIPTION("for hct fingerprint driver");
MODULE_LICENSE("GPL");
