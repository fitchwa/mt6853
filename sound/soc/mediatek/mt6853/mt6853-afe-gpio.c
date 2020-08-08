// SPDX-License-Identifier: GPL-2.0
/*
 *  mt6853-afe-gpio.c  --  Mediatek 6853 afe gpio ctrl
 *
 *  Copyright (c) 2018 MediaTek Inc.
 *  Author: Shane Chien <shane.chien@mediatek.com>
 */

#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>

#include "mt6853-afe-common.h"
#include "mt6853-afe-gpio.h"
#include <linux/hct_include/hct_project_all_config.h>
#ifdef __HCT_EXTAMP_GPIO_NUM__
	#if __HCT_EXTAMP_GPIO_NUM__
	#include <linux/of_gpio.h>
	#include <linux/device.h>
	#endif
#endif

struct pinctrl *aud_pinctrl;

enum mt6853_afe_gpio {
	MT6853_AFE_GPIO_DAT_MISO_OFF,
	MT6853_AFE_GPIO_DAT_MISO_ON,
	MT6853_AFE_GPIO_DAT_MOSI_OFF,
	MT6853_AFE_GPIO_DAT_MOSI_ON,
	MT6853_AFE_GPIO_DAT_MISO_CH34_OFF,
	MT6853_AFE_GPIO_DAT_MISO_CH34_ON,
	MT6853_AFE_GPIO_DAT_MOSI_CH34_OFF,
	MT6853_AFE_GPIO_DAT_MOSI_CH34_ON,
	MT6853_AFE_GPIO_I2S0_OFF,
	MT6853_AFE_GPIO_I2S0_ON,
	MT6853_AFE_GPIO_I2S1_OFF,
	MT6853_AFE_GPIO_I2S1_ON,
	MT6853_AFE_GPIO_I2S2_OFF,
	MT6853_AFE_GPIO_I2S2_ON,
	MT6853_AFE_GPIO_I2S3_OFF,
	MT6853_AFE_GPIO_I2S3_ON,
	MT6853_AFE_GPIO_I2S5_OFF,
	MT6853_AFE_GPIO_I2S5_ON,
	MT6853_AFE_GPIO_VOW_DAT_OFF,
	MT6853_AFE_GPIO_VOW_DAT_ON,
	MT6853_AFE_GPIO_VOW_CLK_OFF,
	MT6853_AFE_GPIO_VOW_CLK_ON,
	MT6853_AFE_GPIO_GPIO_NUM
};

struct audio_gpio_attr {
	const char *name;
	bool gpio_prepare;
	struct pinctrl_state *gpioctrl;
};

static struct audio_gpio_attr aud_gpios[MT6853_AFE_GPIO_GPIO_NUM] = {
	[MT6853_AFE_GPIO_DAT_MISO_OFF] = {"aud_dat_miso_off", false, NULL},
	[MT6853_AFE_GPIO_DAT_MISO_ON] = {"aud_dat_miso_on", false, NULL},
	[MT6853_AFE_GPIO_DAT_MOSI_OFF] = {"aud_dat_mosi_off", false, NULL},
	[MT6853_AFE_GPIO_DAT_MOSI_ON] = {"aud_dat_mosi_on", false, NULL},
	[MT6853_AFE_GPIO_I2S0_OFF] = {"aud_gpio_i2s0_off", false, NULL},
	[MT6853_AFE_GPIO_I2S0_ON] = {"aud_gpio_i2s0_on", false, NULL},
	[MT6853_AFE_GPIO_I2S1_OFF] = {"aud_gpio_i2s1_off", false, NULL},
	[MT6853_AFE_GPIO_I2S1_ON] = {"aud_gpio_i2s1_on", false, NULL},
	[MT6853_AFE_GPIO_I2S2_OFF] = {"aud_gpio_i2s2_off", false, NULL},
	[MT6853_AFE_GPIO_I2S2_ON] = {"aud_gpio_i2s2_on", false, NULL},
	[MT6853_AFE_GPIO_I2S3_OFF] = {"aud_gpio_i2s3_off", false, NULL},
	[MT6853_AFE_GPIO_I2S3_ON] = {"aud_gpio_i2s3_on", false, NULL},
	[MT6853_AFE_GPIO_I2S5_OFF] = {"aud_gpio_i2s5_off", false, NULL},
	[MT6853_AFE_GPIO_I2S5_ON] = {"aud_gpio_i2s5_on", false, NULL},
	[MT6853_AFE_GPIO_VOW_DAT_OFF] = {"vow_dat_miso_off", false, NULL},
	[MT6853_AFE_GPIO_VOW_DAT_ON] = {"vow_dat_miso_on", false, NULL},
	[MT6853_AFE_GPIO_VOW_CLK_OFF] = {"vow_clk_miso_off", false, NULL},
	[MT6853_AFE_GPIO_VOW_CLK_ON] = {"vow_clk_miso_on", false, NULL},
	[MT6853_AFE_GPIO_DAT_MISO_CH34_OFF] = {"aud_dat_miso_ch34_off",
					       false, NULL},
	[MT6853_AFE_GPIO_DAT_MISO_CH34_ON] = {"aud_dat_miso_ch34_on",
					      false, NULL},
	[MT6853_AFE_GPIO_DAT_MOSI_CH34_OFF] = {"aud_dat_mosi_ch34_off",
					       false, NULL},
	[MT6853_AFE_GPIO_DAT_MOSI_CH34_ON] = {"aud_dat_mosi_ch34_on",
					      false, NULL},
};

static DEFINE_MUTEX(gpio_request_mutex);

#ifdef __HCT_EXTAMP_GPIO_NUM__
	#if __HCT_EXTAMP_GPIO_NUM__
int g_gpio_extamp = 0;
static int hct_extamp_mode = 0;

static int hct_gpio_extamp(struct device *dev)
{
	int ret = 0;
	struct device_node *np = dev->of_node;
	pr_info("[%s] start..\n", __func__);
	if (np) {
		ret = of_get_named_gpio(np, "gpio-extamp", 0);
		if (ret < 0)
			pr_err("%s: get extamp GPIO failed (%d)", __func__, ret);
		else
			g_gpio_extamp = ret;
	}
	else
		return -1;
	if (gpio_is_valid(g_gpio_extamp))
			pr_info("gpio number %d is valid\n", g_gpio_extamp);
	if (g_gpio_extamp != 0) {
		ret = gpio_request(g_gpio_extamp, "gpio-extamp");
		if (ret) {
			pr_err("%s : extamp gpio_request failed\n", __func__);
			return -ENODEV;
		}
		pr_info("%s : extamp GPIO = %d\n", __func__, g_gpio_extamp);
		ret = gpio_direction_output(g_gpio_extamp, 1);
		if (ret) {
			pr_err("%s : extamp gpio_direction_output failed\n",  __func__);
			return -ENODEV;
		}
		gpio_set_value(g_gpio_extamp, 0);
		pr_info("%s : extamp gpio_get_value = %d\n", __func__, gpio_get_value(g_gpio_extamp));
	}
	pr_info("[%s] end..\n", __func__);
	return 0;
}

void hct_spk_enable_set(int bEnable)
{
	int i;
	mutex_lock(&gpio_request_mutex);
	if (bEnable == 1) {
		for (i = 0; i < hct_extamp_mode; i++) {
			gpio_set_value(g_gpio_extamp, 0);
			udelay(1);
			gpio_set_value(g_gpio_extamp, 1);
			udelay(1);
		}
	}else{
		gpio_set_value(g_gpio_extamp, 0);
	}
	mutex_unlock(&gpio_request_mutex);
}
	#endif
#endif

int mt6853_afe_gpio_init(struct mtk_base_afe *afe)
{
	int ret;
	int i = 0;

#ifdef __HCT_EXTAMP_GPIO_NUM__
	#if __HCT_EXTAMP_GPIO_NUM__
		hct_gpio_extamp(afe->dev);
	#endif
#endif

#ifdef __HCT_EXTAMP_HP_MODE__
	#if __HCT_EXTAMP_HP_MODE__
	hct_extamp_mode = __HCT_EXTAMP_HP_MODE__;
	#endif
#endif
	printk("hct-drv[%s]: extamp_mode =%d\n",__func__, hct_extamp_mode);
	aud_pinctrl = devm_pinctrl_get(afe->dev);
	if (IS_ERR(aud_pinctrl)) {
		ret = PTR_ERR(aud_pinctrl);
		dev_err(afe->dev, "%s(), ret %d, cannot get aud_pinctrl!\n",
			__func__, ret);
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(aud_gpios); i++) {
		aud_gpios[i].gpioctrl = pinctrl_lookup_state(aud_pinctrl,
							     aud_gpios[i].name);
		if (IS_ERR(aud_gpios[i].gpioctrl)) {
			ret = PTR_ERR(aud_gpios[i].gpioctrl);
			dev_err(afe->dev, "%s(), pinctrl_lookup_state %s fail, ret %d\n",
				__func__, aud_gpios[i].name, ret);
		} else {
			aud_gpios[i].gpio_prepare = true;
		}
	}

	/* gpio status init */
	mt6853_afe_gpio_request(afe, false, MT6853_DAI_ADDA, 0);
	mt6853_afe_gpio_request(afe, false, MT6853_DAI_ADDA, 1);

	return 0;
}

static int mt6853_afe_gpio_select(struct mtk_base_afe *afe,
				  enum mt6853_afe_gpio type)
{
	int ret = 0;

	if (type < 0 || type >= MT6853_AFE_GPIO_GPIO_NUM) {
		dev_err(afe->dev, "%s(), error, invaild gpio type %d\n",
			__func__, type);
		return -EINVAL;
	}

	if (!aud_gpios[type].gpio_prepare) {
		dev_warn(afe->dev, "%s(), error, gpio type %d not prepared\n",
			 __func__, type);
		return -EIO;
	}

	ret = pinctrl_select_state(aud_pinctrl,
				   aud_gpios[type].gpioctrl);
	if (ret) {
		dev_err(afe->dev, "%s(), error, can not set gpio type %d\n",
			__func__, type);
		AUDIO_AEE("can not set gpio type");
	}
	return ret;
}

static int mt6853_afe_gpio_adda_dl(struct mtk_base_afe *afe, bool enable)
{
	if (enable) {
		return mt6853_afe_gpio_select(afe,
					      MT6853_AFE_GPIO_DAT_MOSI_ON);
	} else {
		return mt6853_afe_gpio_select(afe,
					      MT6853_AFE_GPIO_DAT_MOSI_OFF);
	}
}

static int mt6853_afe_gpio_adda_ul(struct mtk_base_afe *afe, bool enable)
{
	if (enable) {
		return mt6853_afe_gpio_select(afe,
					      MT6853_AFE_GPIO_DAT_MISO_ON);
	} else {
		return mt6853_afe_gpio_select(afe,
					      MT6853_AFE_GPIO_DAT_MISO_OFF);
	}
}

static int mt6853_afe_gpio_adda_ch34_dl(struct mtk_base_afe *afe, bool enable)
{
	if (enable) {
		return mt6853_afe_gpio_select(afe,
			MT6853_AFE_GPIO_DAT_MOSI_CH34_ON);
	} else {
		return mt6853_afe_gpio_select(afe,
			MT6853_AFE_GPIO_DAT_MOSI_CH34_OFF);
	}
}

static int mt6853_afe_gpio_adda_ch34_ul(struct mtk_base_afe *afe, bool enable)
{
	if (enable) {
		return mt6853_afe_gpio_select(afe,
			MT6853_AFE_GPIO_DAT_MISO_CH34_ON);
	} else {
		return mt6853_afe_gpio_select(afe,
			MT6853_AFE_GPIO_DAT_MISO_CH34_OFF);
	}
}

int mt6853_afe_gpio_request(struct mtk_base_afe *afe, bool enable,
			    int dai, int uplink)
{
	mutex_lock(&gpio_request_mutex);
	switch (dai) {
	case MT6853_DAI_ADDA:
		if (uplink)
			mt6853_afe_gpio_adda_ul(afe, enable);
		else
			mt6853_afe_gpio_adda_dl(afe, enable);
		break;
	case MT6853_DAI_ADDA_CH34:
		if (uplink)
			mt6853_afe_gpio_adda_ch34_ul(afe, enable);
		else
			mt6853_afe_gpio_adda_ch34_dl(afe, enable);
		break;
	case MT6853_DAI_I2S_0:
		if (enable)
			mt6853_afe_gpio_select(afe, MT6853_AFE_GPIO_I2S0_ON);
		else
			mt6853_afe_gpio_select(afe, MT6853_AFE_GPIO_I2S0_OFF);
		break;
	case MT6853_DAI_I2S_1:
		if (enable)
			mt6853_afe_gpio_select(afe, MT6853_AFE_GPIO_I2S1_ON);
		else
			mt6853_afe_gpio_select(afe, MT6853_AFE_GPIO_I2S1_OFF);
		break;
	case MT6853_DAI_I2S_2:
		if (enable)
			mt6853_afe_gpio_select(afe, MT6853_AFE_GPIO_I2S2_ON);
		else
			mt6853_afe_gpio_select(afe, MT6853_AFE_GPIO_I2S2_OFF);
		break;
	case MT6853_DAI_I2S_3:
		if (enable)
			mt6853_afe_gpio_select(afe, MT6853_AFE_GPIO_I2S3_ON);
		else
			mt6853_afe_gpio_select(afe, MT6853_AFE_GPIO_I2S3_OFF);
		break;
	case MT6853_DAI_I2S_5:
		if (enable)
			mt6853_afe_gpio_select(afe, MT6853_AFE_GPIO_I2S5_ON);
		else
			mt6853_afe_gpio_select(afe, MT6853_AFE_GPIO_I2S5_OFF);
		break;
	case MT6853_DAI_VOW:
		if (enable) {
			mt6853_afe_gpio_select(afe,
					       MT6853_AFE_GPIO_VOW_CLK_ON);
			mt6853_afe_gpio_select(afe,
					       MT6853_AFE_GPIO_VOW_DAT_ON);
		} else {
			mt6853_afe_gpio_select(afe,
					       MT6853_AFE_GPIO_VOW_CLK_OFF);
			mt6853_afe_gpio_select(afe,
					       MT6853_AFE_GPIO_VOW_DAT_OFF);
		}
		break;
	default:
		mutex_unlock(&gpio_request_mutex);
		dev_warn(afe->dev, "%s(), invalid dai %d\n", __func__, dai);
		AUDIO_AEE("invalid dai");
		return -EINVAL;
	}
	mutex_unlock(&gpio_request_mutex);
	return 0;
}

