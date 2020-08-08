/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mtk_panel_ext.h"
#include "../mediatek/mtk_log.h"
#include "../mediatek/mtk_drm_graphics_base.h"
#endif

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif
#include "../i2c/lcm_i2c.h"
struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *powerdm_gpio;
	struct gpio_desc *powerdp_gpio;
	struct gpio_desc *bias_pos, *bias_neg;

	bool prepared;
	bool enabled;

	int error;
};

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;


static int lcm_panel_bias_regulator_init(void)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static int lcm_panel_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
#endif

static void lcm_panel_init(struct lcm *ctx)
{
	printk("hct-nt36675-%s.\n",__func__);
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}

	//printk("hct-drv- add %s reset_gpio = %d is off !!\n",__func__,ctx->reset_gpio);
	//gpiod_set_value(ctx->reset_gpio, 0);
	//msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	printk("hct-drv- add %s reset_gpio = %d is on !!\n",__func__,ctx->reset_gpio);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 0);
	printk("hct-drv- add %s reset_gpio = %d is off !!\n",__func__,ctx->reset_gpio);
	msleep(10);
	gpiod_set_value(ctx->reset_gpio, 1);
	printk("hct-drv- add %s reset_gpio = %d is on !!\n",__func__,ctx->reset_gpio);
	msleep(5);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);


lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0xB0, 0x00);

lcm_dcs_write_seq_static(ctx, 0xFF, 0x20);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0x06, 0x46);
lcm_dcs_write_seq_static(ctx, 0x07, 0x3C);
lcm_dcs_write_seq_static(ctx, 0x69, 0x92);

//lcm_dcs_write_seq_static(ctx, 0x89, 0x18);
//lcm_dcs_write_seq_static(ctx, 0x8A, 0x18);
lcm_dcs_write_seq_static(ctx, 0x95, 0xE5);
lcm_dcs_write_seq_static(ctx, 0x96, 0xE5);
lcm_dcs_write_seq_static(ctx, 0xF2, 0x54);
lcm_dcs_write_seq_static(ctx, 0xF4, 0x54);
lcm_dcs_write_seq_static(ctx, 0xF6, 0x54);
lcm_dcs_write_seq_static(ctx, 0xF8, 0x54);

//AUO modify
lcm_dcs_write_seq_static(ctx, 0xFF, 0x20);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
//R(+)
lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x39, 0x00, 0x4C, 0x00, 0x6B, 0x00, 0x86, 0x00, 0x9D, 0x00, 0xB2, 0x00, 0xC4, 0x00, 0xD5);
lcm_dcs_write_seq_static(ctx, 0xB1, 0x00, 0xE4, 0x01, 0x19, 0x01, 0x42, 0x01, 0x7F, 0x01, 0xAF, 0x01, 0xF7, 0x02, 0x32, 0x02, 0x33);
lcm_dcs_write_seq_static(ctx, 0xB2, 0x02, 0x6C, 0x02, 0xAE, 0x02, 0xD6, 0x03, 0x0A, 0x03, 0x2D, 0x03, 0x56, 0x03, 0x62, 0x03, 0x70);
lcm_dcs_write_seq_static(ctx, 0xB3, 0x03, 0x80, 0x03, 0x93, 0x03, 0xA8, 0x03, 0xB9, 0x03, 0xBE, 0x03, 0xE4, 0x00, 0x00);
//G(+)
lcm_dcs_write_seq_static(ctx, 0xB4, 0x00, 0x39, 0x00, 0x4C, 0x00, 0x6B, 0x00, 0x86, 0x00, 0x9D, 0x00, 0xB2, 0x00, 0xC4, 0x00, 0xD5);
lcm_dcs_write_seq_static(ctx, 0xB5, 0x00, 0xE4, 0x01, 0x19, 0x01, 0x42, 0x01, 0x7F, 0x01, 0xAF, 0x01, 0xF7, 0x02, 0x32, 0x02, 0x33);
lcm_dcs_write_seq_static(ctx, 0xB6, 0x02, 0x6C, 0x02, 0xAE, 0x02, 0xD6, 0x03, 0x0A, 0x03, 0x2D, 0x03, 0x56, 0x03, 0x62, 0x03, 0x70);
lcm_dcs_write_seq_static(ctx, 0xB7, 0x03, 0x80, 0x03, 0x93, 0x03, 0xA8, 0x03, 0xB9, 0x03, 0xBE, 0x03, 0xE4, 0x00, 0x00);
//B(+)
lcm_dcs_write_seq_static(ctx, 0xB8, 0x00, 0x39, 0x00, 0x4C, 0x00, 0x6B, 0x00, 0x86, 0x00, 0x9D, 0x00, 0xB2, 0x00, 0xC4, 0x00, 0xD5);
lcm_dcs_write_seq_static(ctx, 0xB9, 0x00, 0xE4, 0x01, 0x19, 0x01, 0x42, 0x01, 0x7F, 0x01, 0xAF, 0x01, 0xF7, 0x02, 0x32, 0x02, 0x33);
lcm_dcs_write_seq_static(ctx, 0xBA, 0x02, 0x6C, 0x02, 0xAE, 0x02, 0xD6, 0x03, 0x0A, 0x03, 0x2D, 0x03, 0x56, 0x03, 0x62, 0x03, 0x70);
lcm_dcs_write_seq_static(ctx, 0xBB, 0x03, 0x80, 0x03, 0x93, 0x03, 0xA8, 0x03, 0xB9, 0x03, 0xBE, 0x03, 0xE4, 0x00, 0x00);

lcm_dcs_write_seq_static(ctx, 0xFF, 0x21);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
//R(-)
lcm_dcs_write_seq_static(ctx, 0xB0, 0x00, 0x39, 0x00, 0x4C, 0x00, 0x6B, 0x00, 0x86, 0x00, 0x9D, 0x00, 0xB2, 0x00, 0xC4, 0x00, 0xD5);
lcm_dcs_write_seq_static(ctx, 0xB1, 0x00, 0xE4, 0x01, 0x19, 0x01, 0x42, 0x01, 0x7F, 0x01, 0xAF, 0x01, 0xF7, 0x02, 0x32, 0x02, 0x33);
lcm_dcs_write_seq_static(ctx, 0xB2, 0x02, 0x6C, 0x02, 0xAE, 0x02, 0xD6, 0x03, 0x0A, 0x03, 0x2D, 0x03, 0x56, 0x03, 0x62, 0x03, 0x70);
lcm_dcs_write_seq_static(ctx, 0xB3, 0x03, 0x80, 0x03, 0x93, 0x03, 0xA8, 0x03, 0xB9, 0x03, 0xBE, 0x03, 0xD6, 0x00, 0x00);
//G(-)
lcm_dcs_write_seq_static(ctx, 0xB4, 0x00, 0x39, 0x00, 0x4C, 0x00, 0x6B, 0x00, 0x86, 0x00, 0x9D, 0x00, 0xB2, 0x00, 0xC4, 0x00, 0xD5);
lcm_dcs_write_seq_static(ctx, 0xB5, 0x00, 0xE4, 0x01, 0x19, 0x01, 0x42, 0x01, 0x7F, 0x01, 0xAF, 0x01, 0xF7, 0x02, 0x32, 0x02, 0x33);
lcm_dcs_write_seq_static(ctx, 0xB6, 0x02, 0x6C, 0x02, 0xAE, 0x02, 0xD6, 0x03, 0x0A, 0x03, 0x2D, 0x03, 0x56, 0x03, 0x62, 0x03, 0x70);
lcm_dcs_write_seq_static(ctx, 0xB7, 0x03, 0x80, 0x03, 0x93, 0x03, 0xA8, 0x03, 0xB9, 0x03, 0xBE, 0x03, 0xD6, 0x00, 0x00);
//B(-)
lcm_dcs_write_seq_static(ctx, 0xB8, 0x00, 0x39, 0x00, 0x4C, 0x00, 0x6B, 0x00, 0x86, 0x00, 0x9D, 0x00, 0xB2, 0x00, 0xC4, 0x00, 0xD5);
lcm_dcs_write_seq_static(ctx, 0xB9, 0x00, 0xE4, 0x01, 0x19, 0x01, 0x42, 0x01, 0x7F, 0x01, 0xAF, 0x01, 0xF7, 0x02, 0x32, 0x02, 0x33);
lcm_dcs_write_seq_static(ctx, 0xBA, 0x02, 0x6C, 0x02, 0xAE, 0x02, 0xD6, 0x03, 0x0A, 0x03, 0x2D, 0x03, 0x56, 0x03, 0x62, 0x03, 0x70);
lcm_dcs_write_seq_static(ctx, 0xBB, 0x03, 0x80, 0x03, 0x93, 0x03, 0xA8, 0x03, 0xB9, 0x03, 0xBE, 0x03, 0xD6, 0x00, 0x00);

lcm_dcs_write_seq_static(ctx, 0xFF, 0x24);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0x03, 0x24);
lcm_dcs_write_seq_static(ctx, 0x04, 0x24);
lcm_dcs_write_seq_static(ctx, 0x05, 0x0F);
lcm_dcs_write_seq_static(ctx, 0x06, 0x13);
lcm_dcs_write_seq_static(ctx, 0x07, 0x15);
lcm_dcs_write_seq_static(ctx, 0x08, 0x17);
lcm_dcs_write_seq_static(ctx, 0x09, 0x26);
lcm_dcs_write_seq_static(ctx, 0x0A, 0x27);
lcm_dcs_write_seq_static(ctx, 0x0B, 0x28);
lcm_dcs_write_seq_static(ctx, 0x0C, 0x0B);
lcm_dcs_write_seq_static(ctx, 0x0D, 0x0C);
lcm_dcs_write_seq_static(ctx, 0x0E, 0x03);
lcm_dcs_write_seq_static(ctx, 0x0F, 0x03);
lcm_dcs_write_seq_static(ctx, 0x10, 0x04);
lcm_dcs_write_seq_static(ctx, 0x11, 0x04);
lcm_dcs_write_seq_static(ctx, 0x12, 0x22);
lcm_dcs_write_seq_static(ctx, 0x13, 0x01);
lcm_dcs_write_seq_static(ctx, 0x14, 0x1C);
lcm_dcs_write_seq_static(ctx, 0x15, 0x29);
lcm_dcs_write_seq_static(ctx, 0x16, 0x0F);
lcm_dcs_write_seq_static(ctx, 0x17, 0x0F);
lcm_dcs_write_seq_static(ctx, 0x1B, 0x24);
lcm_dcs_write_seq_static(ctx, 0x1C, 0x24);
lcm_dcs_write_seq_static(ctx, 0x1D, 0x0F);
lcm_dcs_write_seq_static(ctx, 0x1E, 0x13);
lcm_dcs_write_seq_static(ctx, 0x1F, 0x15);
lcm_dcs_write_seq_static(ctx, 0x20, 0x17);
lcm_dcs_write_seq_static(ctx, 0x21, 0x26);
lcm_dcs_write_seq_static(ctx, 0x22, 0x27);
lcm_dcs_write_seq_static(ctx, 0x23, 0x28);
lcm_dcs_write_seq_static(ctx, 0x24, 0x0B);
lcm_dcs_write_seq_static(ctx, 0x25, 0x0C);
lcm_dcs_write_seq_static(ctx, 0x26, 0x03);
lcm_dcs_write_seq_static(ctx, 0x27, 0x03);
lcm_dcs_write_seq_static(ctx, 0x28, 0x04);
lcm_dcs_write_seq_static(ctx, 0x29, 0x04);
lcm_dcs_write_seq_static(ctx, 0x2A, 0x22);
lcm_dcs_write_seq_static(ctx, 0x2B, 0x01);
lcm_dcs_write_seq_static(ctx, 0x2D, 0x1C);
lcm_dcs_write_seq_static(ctx, 0x2F, 0x29);
lcm_dcs_write_seq_static(ctx, 0x30, 0x0F);
lcm_dcs_write_seq_static(ctx, 0x31, 0x0F);
lcm_dcs_write_seq_static(ctx, 0x32, 0x44);
lcm_dcs_write_seq_static(ctx, 0x33, 0x03);
lcm_dcs_write_seq_static(ctx, 0x35, 0x40);
lcm_dcs_write_seq_static(ctx, 0x36, 0x47);
lcm_dcs_write_seq_static(ctx, 0x37, 0x22);
lcm_dcs_write_seq_static(ctx, 0x38, 0x24);
lcm_dcs_write_seq_static(ctx, 0x39, 0x04);
lcm_dcs_write_seq_static(ctx, 0x3A, 0x04);
lcm_dcs_write_seq_static(ctx, 0x45, 0x02);
lcm_dcs_write_seq_static(ctx, 0x46, 0x03);
lcm_dcs_write_seq_static(ctx, 0x47, 0x02);
lcm_dcs_write_seq_static(ctx, 0x48, 0x03);
lcm_dcs_write_seq_static(ctx, 0x49, 0x04);
lcm_dcs_write_seq_static(ctx, 0x4D, 0x02);
lcm_dcs_write_seq_static(ctx, 0x4E, 0x7C);
lcm_dcs_write_seq_static(ctx, 0x4F, 0x7C);
lcm_dcs_write_seq_static(ctx, 0x50, 0x02);
lcm_dcs_write_seq_static(ctx, 0x53, 0x7C);
lcm_dcs_write_seq_static(ctx, 0x79, 0x23);
lcm_dcs_write_seq_static(ctx, 0x7A, 0x82);
lcm_dcs_write_seq_static(ctx, 0x7B, 0x8F);
lcm_dcs_write_seq_static(ctx, 0x7D, 0x06);
lcm_dcs_write_seq_static(ctx, 0x80, 0x06);
lcm_dcs_write_seq_static(ctx, 0x81, 0x06);
lcm_dcs_write_seq_static(ctx, 0x82, 0x16);
lcm_dcs_write_seq_static(ctx, 0x83, 0x34);
lcm_dcs_write_seq_static(ctx, 0x84, 0x52);
lcm_dcs_write_seq_static(ctx, 0x85, 0x25);
lcm_dcs_write_seq_static(ctx, 0x86, 0x43);
lcm_dcs_write_seq_static(ctx, 0x87, 0x61);
lcm_dcs_write_seq_static(ctx, 0x90, 0x25);
lcm_dcs_write_seq_static(ctx, 0x91, 0x43);
lcm_dcs_write_seq_static(ctx, 0x92, 0x61);
lcm_dcs_write_seq_static(ctx, 0x93, 0x16);
lcm_dcs_write_seq_static(ctx, 0x94, 0x34);
lcm_dcs_write_seq_static(ctx, 0x95, 0x52);
lcm_dcs_write_seq_static(ctx, 0x9C, 0xF4);
lcm_dcs_write_seq_static(ctx, 0x9D, 0x01);
lcm_dcs_write_seq_static(ctx, 0xA0, 0x0F);
lcm_dcs_write_seq_static(ctx, 0xA2, 0x0F);
lcm_dcs_write_seq_static(ctx, 0xA3, 0x03);
lcm_dcs_write_seq_static(ctx, 0xA4, 0x06);
lcm_dcs_write_seq_static(ctx, 0xA5, 0x06);
lcm_dcs_write_seq_static(ctx, 0xAA, 0x0B);
lcm_dcs_write_seq_static(ctx, 0xAC, 0x0A);
lcm_dcs_write_seq_static(ctx, 0xAD, 0x02);
lcm_dcs_write_seq_static(ctx, 0xAE, 0x0C);
lcm_dcs_write_seq_static(ctx, 0xB0, 0x05);
lcm_dcs_write_seq_static(ctx, 0xB2, 0x04);
lcm_dcs_write_seq_static(ctx, 0xB3, 0x08);
lcm_dcs_write_seq_static(ctx, 0xB4, 0x06);
lcm_dcs_write_seq_static(ctx, 0xB6, 0x17);
lcm_dcs_write_seq_static(ctx, 0xB8, 0x16);
lcm_dcs_write_seq_static(ctx, 0xB9, 0x0E);
lcm_dcs_write_seq_static(ctx, 0xBA, 0x18);
lcm_dcs_write_seq_static(ctx, 0xBC, 0x11);
lcm_dcs_write_seq_static(ctx, 0xBE, 0x10);
lcm_dcs_write_seq_static(ctx, 0xBF, 0x14);
lcm_dcs_write_seq_static(ctx, 0xC0, 0x12);
lcm_dcs_write_seq_static(ctx, 0xC4, 0xC0);
lcm_dcs_write_seq_static(ctx, 0xCA, 0xC9);
lcm_dcs_write_seq_static(ctx, 0xCD, 0x30);
lcm_dcs_write_seq_static(ctx, 0xD2, 0x30);
lcm_dcs_write_seq_static(ctx, 0xD9, 0x80);
lcm_dcs_write_seq_static(ctx, 0xE5, 0x06);
lcm_dcs_write_seq_static(ctx, 0xE6, 0x80);
lcm_dcs_write_seq_static(ctx, 0xE9, 0x03);
lcm_dcs_write_seq_static(ctx, 0xED, 0xC0);

lcm_dcs_write_seq_static(ctx, 0xFF, 0x25);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0x58, 0x22);
lcm_dcs_write_seq_static(ctx, 0x59, 0x04);
lcm_dcs_write_seq_static(ctx, 0x5A, 0x04);
lcm_dcs_write_seq_static(ctx, 0x5B, 0x04);
lcm_dcs_write_seq_static(ctx, 0x5E, 0x23);
lcm_dcs_write_seq_static(ctx, 0x5F, 0xE8);
lcm_dcs_write_seq_static(ctx, 0x66, 0xD8);
lcm_dcs_write_seq_static(ctx, 0x67, 0x12);
lcm_dcs_write_seq_static(ctx, 0x68, 0x58);
lcm_dcs_write_seq_static(ctx, 0x69, 0x60);
lcm_dcs_write_seq_static(ctx, 0x6B, 0x00);
lcm_dcs_write_seq_static(ctx, 0x6C, 0x6D);
lcm_dcs_write_seq_static(ctx, 0x71, 0x6D);
lcm_dcs_write_seq_static(ctx, 0x77, 0x72);
lcm_dcs_write_seq_static(ctx, 0x7E, 0x2D);
lcm_dcs_write_seq_static(ctx, 0x84, 0x60);
lcm_dcs_write_seq_static(ctx, 0xC1, 0xA9);
lcm_dcs_write_seq_static(ctx, 0xC2, 0x52);
lcm_dcs_write_seq_static(ctx, 0xC3, 0x01);
lcm_dcs_write_seq_static(ctx, 0xEF, 0x00);
lcm_dcs_write_seq_static(ctx, 0xF0, 0x00);
lcm_dcs_write_seq_static(ctx, 0xF1, 0x04);

lcm_dcs_write_seq_static(ctx, 0xFF, 0x26);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0x00, 0x10);
lcm_dcs_write_seq_static(ctx, 0x01, 0x87);
lcm_dcs_write_seq_static(ctx, 0x03, 0x00);
lcm_dcs_write_seq_static(ctx, 0x04, 0x87);
lcm_dcs_write_seq_static(ctx, 0x06, 0x31);
lcm_dcs_write_seq_static(ctx, 0x08, 0x31);
lcm_dcs_write_seq_static(ctx, 0x74, 0xAF);
lcm_dcs_write_seq_static(ctx, 0x81, 0x0F);
lcm_dcs_write_seq_static(ctx, 0x83, 0x03);
lcm_dcs_write_seq_static(ctx, 0x84, 0x05);
lcm_dcs_write_seq_static(ctx, 0x85, 0x01);
lcm_dcs_write_seq_static(ctx, 0x86, 0x05);
lcm_dcs_write_seq_static(ctx, 0x87, 0x01);
lcm_dcs_write_seq_static(ctx, 0x88, 0x04);
lcm_dcs_write_seq_static(ctx, 0x8A, 0x1A);
lcm_dcs_write_seq_static(ctx, 0x8C, 0x54);
lcm_dcs_write_seq_static(ctx, 0x8D, 0x27);
lcm_dcs_write_seq_static(ctx, 0x8E, 0x63);
lcm_dcs_write_seq_static(ctx, 0x8F, 0x36);
lcm_dcs_write_seq_static(ctx, 0x90, 0x72);
lcm_dcs_write_seq_static(ctx, 0x91, 0x45);
lcm_dcs_write_seq_static(ctx, 0x9A, 0x80);
lcm_dcs_write_seq_static(ctx, 0x9B, 0x42);
lcm_dcs_write_seq_static(ctx, 0x9C, 0x00);
lcm_dcs_write_seq_static(ctx, 0x9D, 0x00);
lcm_dcs_write_seq_static(ctx, 0x9E, 0x00);

lcm_dcs_write_seq_static(ctx, 0xFF, 0x27);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0x00, 0x84);
lcm_dcs_write_seq_static(ctx, 0x01, 0xFC);
lcm_dcs_write_seq_static(ctx, 0x20, 0x80);
lcm_dcs_write_seq_static(ctx, 0x21, 0x9C);
lcm_dcs_write_seq_static(ctx, 0x25, 0x81);
lcm_dcs_write_seq_static(ctx, 0x26, 0xF5);
lcm_dcs_write_seq_static(ctx, 0x88, 0x03);
lcm_dcs_write_seq_static(ctx, 0x89, 0x01);
lcm_dcs_write_seq_static(ctx, 0x8A, 0x04);
lcm_dcs_write_seq_static(ctx, 0xF9, 0x00);

lcm_dcs_write_seq_static(ctx, 0xFF, 0x2A);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0x07, 0x50);
lcm_dcs_write_seq_static(ctx, 0x0A, 0x20);
lcm_dcs_write_seq_static(ctx, 0x0C, 0x13);
lcm_dcs_write_seq_static(ctx, 0x0D, 0x90);
lcm_dcs_write_seq_static(ctx, 0x11, 0x73);
lcm_dcs_write_seq_static(ctx, 0x15, 0x08);
lcm_dcs_write_seq_static(ctx, 0x16, 0x6F);
lcm_dcs_write_seq_static(ctx, 0x19, 0x07);
lcm_dcs_write_seq_static(ctx, 0x1A, 0xEB);
lcm_dcs_write_seq_static(ctx, 0x1B, 0x0A);
lcm_dcs_write_seq_static(ctx, 0x1D, 0x14);
lcm_dcs_write_seq_static(ctx, 0x1E, 0x84);
lcm_dcs_write_seq_static(ctx, 0x1F, 0x84);
lcm_dcs_write_seq_static(ctx, 0x20, 0x84);
lcm_dcs_write_seq_static(ctx, 0x45, 0xE6);
lcm_dcs_write_seq_static(ctx, 0x75, 0x99);
lcm_dcs_write_seq_static(ctx, 0xA5, 0x73);
lcm_dcs_write_seq_static(ctx, 0xD8, 0x40);

lcm_dcs_write_seq_static(ctx, 0xFF, 0x2C);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);
lcm_dcs_write_seq_static(ctx, 0xFB, 0x01);

lcm_dcs_write_seq_static(ctx, 0xFF, 0x10);

	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);//SLPOUT
	msleep(120);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00);//DSPON

	msleep(20);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	printk("hct-nt36675-%s.\n",__func__);
	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->prepared)
		return 0;
	printk("hct-nt36675-%s.\n",__func__);
	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
#else
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	printk("hct-drv- %s reset_pin = %d is off !\n",__func__,ctx->reset_gpio);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	msleep(50);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	printk("hct-drv- %s bias_neg_pin = %d is off !\n",__func__,ctx->bias_neg);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	msleep(20);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	printk("hct-drv- %s bias_pos_pin = %d is off !\n",__func__,ctx->bias_pos);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
#endif
	msleep(20);
	ctx->powerdm_gpio =
		devm_gpiod_get(ctx->dev, "powerdm", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->powerdm_gpio)) {
		dev_err(ctx->dev, "%s: cannot get powerdm_gpio %ld\n",
			__func__, PTR_ERR(ctx->powerdm_gpio));
		return PTR_ERR(ctx->powerdm_gpio);
	}
	printk("hct-drv- %s powerdm_pin = %d is off !\n",__func__,ctx->powerdm_gpio);
	gpiod_set_value(ctx->powerdm_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->powerdm_gpio);

	return 0;
}
extern int lcm_i2c_set_data(char type, const LCM_DATA_T2 *t2);
static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
	LCM_DATA_T2 i2c_data;
	printk("hct1-nt36675-%s.\n",__func__);
	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

	ctx->powerdm_gpio =
		devm_gpiod_get(ctx->dev, "powerdm", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->powerdm_gpio)) {
		dev_err(ctx->dev, "%s: cannot get powerdm_gpio %ld\n",
			__func__, PTR_ERR(ctx->powerdm_gpio));
		return PTR_ERR(ctx->powerdm_gpio);
	}
	printk("hct-drv- %s powerdm_pin = %d is on !\n",__func__,ctx->powerdm_gpio);
	gpiod_set_value(ctx->powerdm_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->powerdm_gpio);
	msleep(10);
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_enable();
#else
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	printk("hct-drv- %s bias_pos_pin = %d is on !\n",__func__,ctx->bias_pos);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);


	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	printk("hct-drv- %s bias_neg_pin = %d is on !\n",__func__,ctx->bias_neg);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
#endif

	i2c_data.cmd = 0x00;  //+5V
	i2c_data.data = 0x12; //5.8
	lcm_i2c_set_data(1, &i2c_data);
	i2c_data.cmd = 0x01;  //-5V
	i2c_data.data = 0x12; //-5.8
	lcm_i2c_set_data(1, &i2c_data);
	msleep(10);
	msleep(10);
	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_rst(panel);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	printk("hct-nt36675-%s.\n",__func__);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define HFP (60)
#define HSA (10)
#define HBP (60)
#define VFP (20)
#define VSA (4)
#define VBP (6)
#define VAC (2300)
#define HAC (1080)
static u32 fake_heigh = 2300;
static u32 fake_width = 1080;
static bool need_fake_resolution;

static struct drm_display_mode default_mode = {
	.clock = 169158,//169158,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,
	.hsync_end = HAC + HFP + HSA,
	.htotal = HAC + HFP + HSA + HBP,//1210
	.vdisplay = VAC,
	.vsync_start = VAC + VFP,
	.vsync_end = VAC + VFP + VSA,
	.vtotal = VAC + VFP + VSA + VBP,//2330
	.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);
	printk("hct-nt36675-%s.\n",__func__);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	
	struct lcm *ctx = panel_to_lcm(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	unsigned char id[3] = {0x00, 0x00, 0x00};
	ssize_t ret;
	printk("hct-nt36675-%s.\n",__func__);
	ret = mipi_dsi_dcs_read(dsi, 0x4, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	DDPINFO("ATA read data %x %x %x\n", data[0], data[1], data[2]);

	if (data[0] == id[0] &&
			data[1] == id[1] &&
			data[2] == id[2])
		return 1;

	DDPINFO("ATA expect read data is %x %x %x\n",
			id[0], id[1], id[2]);

	return 0;
}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51, 0xFF};
	printk("hct-nt36675-%s.\n",__func__);

	bl_tb0[1] = level;

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static int lcm_get_virtual_heigh(void)
{
	return VAC;
}

static int lcm_get_virtual_width(void)
{
	return HAC;
}

static struct mtk_panel_params ext_params = {
	.pll_clk = 510,
	.vfp_low_power = 500,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9C,
	},
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
	},
};

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static void change_drm_disp_mode_params(struct drm_display_mode *mode)
{
	printk("hct-nt36675-%s.\n",__func__);
	if (fake_heigh > 0 && fake_heigh < VAC) {
		mode->vdisplay = fake_heigh;
		mode->vsync_start = fake_heigh + VFP;
		mode->vsync_end = fake_heigh + VFP + VSA;
		mode->vtotal = fake_heigh + VFP + VSA + VBP;
	}
	if (fake_width > 0 && fake_width < HAC) {
		mode->hdisplay = fake_width;
		mode->hsync_start = fake_width + HFP;
		mode->hsync_end = fake_width + HFP + HSA;
		mode->htotal = fake_width + HFP + HSA + HBP;
	}
}

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;
	printk("hct-nt36675-%s.\n",__func__);
	if (need_fake_resolution)
		change_drm_disp_mode_params(&default_mode);
	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 64;
	panel->connector->display_info.height_mm = 129;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static void check_is_need_fake_resolution(struct device *dev)
{
	unsigned int ret = 0;
	printk("hct-nt36675-%s.\n",__func__);
	ret = of_property_read_u32(dev->of_node, "fake_heigh", &fake_heigh);
	if (ret)
		need_fake_resolution = false;
	ret = of_property_read_u32(dev->of_node, "fake_width", &fake_width);
	if (ret)
		need_fake_resolution = false;
	if (fake_heigh > 0 && fake_heigh < VAC)
		need_fake_resolution = true;
	if (fake_width > 0 && fake_width < HAC)
		need_fake_resolution = true;
}

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	printk("hct-nt36675-%s.\n",__func__);
	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->powerdm_gpio =
		devm_gpiod_get(ctx->dev, "powerdm", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->powerdm_gpio)) {
		dev_err(ctx->dev, "%s: cannot get powerdm-gpio %ld\n",
			__func__, PTR_ERR(ctx->powerdm_gpio));
		return PTR_ERR(ctx->powerdm_gpio);
	}
	devm_gpiod_put(dev, ctx->powerdm_gpio);

	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	check_is_need_fake_resolution(dev);
	pr_info("%s-\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
	printk("hct-nt36675-%s.\n",__func__);
	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "truly,nt36675,vdo", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-truly-nt36675-vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("truly nt36675 VDO LCD Panel Driver");
MODULE_LICENSE("GPL v2");

