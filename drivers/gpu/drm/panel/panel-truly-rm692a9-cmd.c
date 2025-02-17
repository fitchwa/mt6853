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
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	pr_info("%s\n", __func__);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(15 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(1 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(10 * 1000);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
/*
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
	lcm_dcs_write_seq_static(ctx, 0xC2, 0x08);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x51, 0xFF);
*/

	lcm_dcs_write_seq_static(ctx, 0xFE	,0x72);
	lcm_dcs_write_seq_static(ctx, 0x26	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x27	,0x0A);
	lcm_dcs_write_seq_static(ctx, 0xB0	,0x9);
	lcm_dcs_write_seq_static(ctx, 0xB1	,0x40);
	lcm_dcs_write_seq_static(ctx, 0xB2	,0x9);
	lcm_dcs_write_seq_static(ctx, 0xB3	,0x40);
	lcm_dcs_write_seq_static(ctx, 0x6B	,0xb1);
	lcm_dcs_write_seq_static(ctx, 0x4B	,0xB7);
	lcm_dcs_write_seq_static(ctx, 0xAF	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0xFE	,0x72);
	lcm_dcs_write_seq_static(ctx, 0x69	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0xFE	,0x72);
	lcm_dcs_write_seq_static(ctx, 0x4B	,0xb7);
	lcm_dcs_write_seq_static(ctx, 0x4C	,0x4E);
	lcm_dcs_write_seq_static(ctx, 0x4D	,0x85);
	lcm_dcs_write_seq_static(ctx, 0x52	,0x0F);
	lcm_dcs_write_seq_static(ctx, 0x77	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x78	,0x4);
	lcm_dcs_write_seq_static(ctx, 0x79	,0x6);
	lcm_dcs_write_seq_static(ctx, 0x7A	,0x7);
	lcm_dcs_write_seq_static(ctx, 0x7B	,0x0A);
	lcm_dcs_write_seq_static(ctx, 0x7C	,0x40);
	lcm_dcs_write_seq_static(ctx, 0x7D	,0xA0);
	lcm_dcs_write_seq_static(ctx, 0x7E	,0xAF);
	lcm_dcs_write_seq_static(ctx, 0x7F	,0x56);
	lcm_dcs_write_seq_static(ctx, 0x80	,0xBB);
	lcm_dcs_write_seq_static(ctx, 0x81	,0x14);
	lcm_dcs_write_seq_static(ctx, 0x82	,0x4B);
	lcm_dcs_write_seq_static(ctx, 0x83	,0x90);
	lcm_dcs_write_seq_static(ctx, 0x84	,0xC8);
	lcm_dcs_write_seq_static(ctx, 0x85	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x73	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x74	,0x55);
	lcm_dcs_write_seq_static(ctx, 0x75	,0xFA);
	lcm_dcs_write_seq_static(ctx, 0x76	,0x3F);
	lcm_dcs_write_seq_static(ctx, 0x8A	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x8B	,0x3);
	lcm_dcs_write_seq_static(ctx, 0x8C	,0x4);
	lcm_dcs_write_seq_static(ctx, 0x8D	,0x6);
	lcm_dcs_write_seq_static(ctx, 0x8E	,0x9);
	lcm_dcs_write_seq_static(ctx, 0x8F	,0x15);
	lcm_dcs_write_seq_static(ctx, 0x90	,0x29);
	lcm_dcs_write_seq_static(ctx, 0x91	,0x54);
	lcm_dcs_write_seq_static(ctx, 0x92	,0xB1);
	lcm_dcs_write_seq_static(ctx, 0x93	,0x18);
	lcm_dcs_write_seq_static(ctx, 0x94	,0x8C);
	lcm_dcs_write_seq_static(ctx, 0x95	,0x0D);
	lcm_dcs_write_seq_static(ctx, 0x96	,0xA0);
	lcm_dcs_write_seq_static(ctx, 0x97	,0x45);
	lcm_dcs_write_seq_static(ctx, 0x98	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x86	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x87	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x88	,0x94);
	lcm_dcs_write_seq_static(ctx, 0x89	,0x3E);
	lcm_dcs_write_seq_static(ctx, 0xFE	,0x74);
	lcm_dcs_write_seq_static(ctx, 0x0	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x1	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x2	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x3	,0x7);
	lcm_dcs_write_seq_static(ctx, 0x4	,0x0F);
	lcm_dcs_write_seq_static(ctx, 0x5	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x6	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x7	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x8	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x9	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x0A	,0x80);
	lcm_dcs_write_seq_static(ctx, 0x0B	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x0C	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x0D	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x0E	,0x60);
	lcm_dcs_write_seq_static(ctx, 0x0F	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x10	,0x1);
	lcm_dcs_write_seq_static(ctx, 0x11	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x12	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x13	,0x1);
	lcm_dcs_write_seq_static(ctx, 0x14	,0x60);
	lcm_dcs_write_seq_static(ctx, 0x15	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x16	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x17	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x18	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x19	,0x20);
	lcm_dcs_write_seq_static(ctx, 0x1A	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x1B	,0xF0);
	lcm_dcs_write_seq_static(ctx, 0x1C	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x1D	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x1E	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x1F	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x20	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x21	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x22	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x23	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x24	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x25	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x26	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x27	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x28	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x29	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x2A	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x2B	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x2D	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x2F	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x30	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x31	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x32	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x33	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x34	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x35	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x36	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x37	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x38	,0x60);
	lcm_dcs_write_seq_static(ctx, 0x39	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x3A	,0xF1);
	lcm_dcs_write_seq_static(ctx, 0x3B	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x3D	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x3F	,0xF1);
	lcm_dcs_write_seq_static(ctx, 0x40	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x41	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x42	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x43	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x44	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x45	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x46	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x47	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x48	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x49	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x4A	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x4B	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x4C	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x4D	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x4E	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x4F	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x50	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x51	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x52	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x53	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x54	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x55	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x56	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x58	,0x60);
	lcm_dcs_write_seq_static(ctx, 0x59	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x5A	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x5B	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x5C	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x5D	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x5E	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x5F	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x60	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x61	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x62	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x63	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x64	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x65	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x66	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x67	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x68	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x69	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x6A	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x6B	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x6C	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x6D	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x6E	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x6F	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x70	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x71	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x72	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x73	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0x74	,0xFF);
	lcm_dcs_write_seq_static(ctx, 0xfe	,0x0);
	lcm_dcs_write_seq_static(ctx, 0xc2	,0x8);
	lcm_dcs_write_seq_static(ctx, 0x35	,0x0);
	lcm_dcs_write_seq_static(ctx, 0x51	,0x0 ,0x0);

	lcm_dcs_write_seq_static(ctx, 0x11);
	msleep(200);
	lcm_dcs_write_seq_static(ctx, 0x29);

	msleep(100);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s\n", __func__);
	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		printk("hct--01-%s: \n",__func__);
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	pr_info("%s\n", __func__);
	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	msleep(50);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(150);

	ctx->error = 0;
	ctx->prepared = false;

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

#if 0//defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
//#else


	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(1000);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
#endif
	msleep(10);
	ctx->powerdp_gpio =
		devm_gpiod_get(ctx->dev, "powerdp", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->powerdp_gpio)) {
		dev_err(ctx->dev, "%s: cannot get powerdp_gpio %ld\n",
			__func__, PTR_ERR(ctx->powerdp_gpio));
		return PTR_ERR(ctx->powerdp_gpio);
	}
	printk("hct-drv- %s powerdp_pin = %d is off !\n",__func__,ctx->powerdp_gpio);
	gpiod_set_value(ctx->powerdp_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->powerdp_gpio);
	msleep(10);
	ctx->powerdm_gpio =
		devm_gpiod_get(ctx->dev, "powerdm", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->powerdm_gpio)) {
		dev_err(ctx->dev, "%s: cannot get powerdm_gpio %ld\n",
			__func__, PTR_ERR(ctx->powerdm_gpio));
		return PTR_ERR(ctx->powerdm_gpio);
	}
	printk("hct-drv- %s powerdm_pin = %d is off !\n",__func__,ctx->powerdm_gpio);
	gpiod_set_value(ctx->powerdm_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->powerdm_gpio);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	if (ctx->prepared)
		return 0;

#if 0//defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_enable();
//#else
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
#endif
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
	ctx->powerdp_gpio =
		devm_gpiod_get(ctx->dev, "powerdp", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->powerdp_gpio)) {
		dev_err(ctx->dev, "%s: cannot get powerdp_gpio %ld\n",
			__func__, PTR_ERR(ctx->powerdp_gpio));
		return PTR_ERR(ctx->powerdp_gpio);
	}
	printk("hct-drv- %s powerdp_pin = %d is on !\n",__func__,ctx->powerdp_gpio);
	gpiod_set_value(ctx->powerdp_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->powerdp_gpio);

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
	pr_info("%s\n", __func__);
	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		printk("hct--01-%s: \n",__func__ );
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 162539,
	.hdisplay = 1080,
	.hsync_start = 1080 + 26,
	.hsync_end = 1080 + 26 + 2,
	.htotal = 1080 + 26 + 2 + 36,
	.vdisplay = 2340,
	.vsync_start = 2340 + 16,
	.vsync_end = 2340 + 16 + 6,
	.vtotal = 2340 + 16 + 6 + 6,
	.vrefresh = 60,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

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
	char bl_tb0[] = {0x51, 0x07, 0xFF};
	pr_info("%s--level = %d\n", __func__, level);
//	bl_tb0[1] = level;

	if (level > 255)
		level = 255;

	level = level * 2047 / 255;
	bl_tb0[1] = ((level >> 8) & 0xf);
	bl_tb0[2] = (level & 0xff);

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}




static struct mtk_panel_params ext_params = {
	.pll_clk = 525,
	.vfp_low_power = 750,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x1c,
	},

};

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
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

static int lcm_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

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

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
	printk("hct--01-%s: \n",__func__);

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

	pr_info("%s-00\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "truly,rm692a9,cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-truly-rm692a9-cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("truly rm692a9 CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
