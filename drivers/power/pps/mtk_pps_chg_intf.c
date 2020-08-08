/*
 * Copyright (C) 2017 Richtek Technology Corp.
 *
 * Richtek PPS Charger Interface For MTK Charging Archeture
 *
 * Author: ShuFanLee <shufan_lee@richtek.com>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "pps_class.h"

#ifndef CONFIG_MTK_GAUGE_VERSION
#ifdef CONFIG_MTK_HAFG_20
#define CONFIG_MTK_GAUGE_VERSION 20
#else
#define CONFIG_MTK_GAUGE_VERSION 10
#endif /* CONFIG_MTK_HAFG_20 */
#endif /* CONFIG_MTK_GAUGE_VERSION */

#if (CONFIG_MTK_GAUGE_VERSION == 30)
#include <mt-plat/charger_class.h>
#include <mt-plat/mtk_battery.h>
#include <mt-plat/mtk_charger.h>
#include "mtk_charger_intf.h"
/*#include "pmic_bif.h"*/
#else
#include <mt-plat/charging.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#endif /* CONFIG_MTK_GAUGE_VERSION */

#define MTK_PPS_INTF_VERSION	"1.0.2_MTK"

struct mtk_pps_intf_info {
	struct device *dev;
	struct pps_device *pps_dev;
	struct pps_desc desc;
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	struct charger_device *pmy_chg;
	struct charger_device *pmy_ls;
	struct charger_consumer *chg_consumer;
#else
	CHARGING_CONTROL mtk_chg_ctrl;
	CHARGING_CONTROL mtk_ls_ctrl;
#endif /* CONFIG_MTK_GAUGE_VERSION */
};

/* ========================= */
/* pps class ops             */
/* ========================= */

static int mtk_pps_enable_power_path(struct pps_device *pps, bool en)
{
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);

#if (CONFIG_MTK_GAUGE_VERSION == 30)
	return charger_manager_enable_power_path(info->chg_consumer,
		MAIN_CHARGER, en);
#else
	return info->mtk_chg_ctrl(CHARGING_CMD_ENABLE_POWER_PATH, &en);
#endif /* CONFIG_MTK_GAUGE_VERSION */
}

static int mtk_pps_enable_charger(struct pps_device *pps, bool en)
{
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);

#if (CONFIG_MTK_GAUGE_VERSION == 30)
	return charger_dev_enable(info->pmy_chg, en);
#else
	return info->mtk_chg_ctrl(CHARGING_CMD_ENABLE, &en);
#endif /* CONFIG_MTK_GAUGE_VERSION */
}

static int mtk_pps_get_tchg(struct pps_device *pps, int *min, int *max)
{
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);

#if (CONFIG_MTK_GAUGE_VERSION == 30)
	return charger_dev_get_temperature(info->pmy_ls, min, max);
#else
	int ret = 0, temp[2] = {0, 0};

	ret = info->mtk_ls_ctrl(CHARGING_CMD_GET_CHARGER_TEMPERATURE,
		temp);
	*min = temp[0];
	*max = temp[1];

	return ret;
#endif /* CONFIG_MTK_GAUGE_VERSION */
}

static int mtk_pps_set_dirchg_ibusoc(struct pps_device *pps, u32 mA)
{
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);

#if (CONFIG_MTK_GAUGE_VERSION == 30)
	return charger_dev_set_direct_charging_ibusoc(info->pmy_ls,
		mA * 1000);
#else
	return info->mtk_ls_ctrl(CHARGING_CMD_SET_DC_VBUSOC, &mA);
#endif /* CONFIG_MTK_GAUGE_VERSION */
}

static int mtk_pps_enable_dirchg(struct pps_device *pps, bool en)
{
	/* Also inform switching charger in case it needs to be in HZ */
	int ret = 0;
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);

#if (CONFIG_MTK_GAUGE_VERSION == 30)
	ret = charger_dev_enable_direct_charging(info->pmy_chg, en);
	ret |= charger_dev_enable_direct_charging(info->pmy_ls, en);
#else
	ret = info->mtk_chg_ctrl(CHARGING_CMD_ENABLE_DIRECT_CHARGE, &en);
	ret |= info->mtk_ls_ctrl(CHARGING_CMD_ENABLE_DIRECT_CHARGE, &en);
#endif /* CONFIG_MTK_GAUGE_VERSION */
	return ret;
}

static int mtk_pps_enable_dirchg_chip(struct pps_device *pps, bool en)
{
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);

#if (CONFIG_MTK_GAUGE_VERSION == 30)
	return charger_dev_enable_chip(info->pmy_ls, en);
#else
	return info->mtk_ls_ctrl(CHARGING_CMD_ENABLE_CHIP, &en);
#endif /* CONFIG_MTK_GAUGE_VERSION */
}

static int mtk_pps_get_vbat(struct pps_device *pps, u32 *mV)
{
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	*mV = pmic_get_battery_voltage();
#else
	*mV = battery_meter_get_battery_voltage(KAL_TRUE);
#endif /* CONFIG_MTK_GAUGE_VERSION */
	return 0;
}

static int mtk_pps_get_ibat(struct pps_device *pps, s32 *mA)
{
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	*mA = battery_get_bat_current() / 10;
#else
	*mA = battery_meter_get_battery_current() / 10;
#endif /* CONFIG_MTK_GAUGE_VERSION */
	return 0;
}

static int mtk_pps_get_tbat(struct pps_device *pps, int *tbat)
{
	*tbat = battery_get_bat_temperature();
	return 0;
}

static int mtk_pps_get_vbus(struct pps_device *pps, u32 *mV)
{
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	*mV = battery_get_vbus(); //pmic_get_vbus();
#else
	*mV = battery_meter_get_charger_voltage();
#endif /* CONFIG_MTK_GAUGE_VERSION */
	return 0;
}

static int mtk_pps_get_ibus(struct pps_device *pps, u32 *mA)
{
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	int ret = 0;

	ret = charger_dev_get_ibus(info->pmy_ls, mA);
	*mA /= 1000;
	return ret;
#else
	return info->mtk_ls_ctrl(CHARGING_CMD_GET_IBUS, mA);
#endif /* CONFIG_MTK_GAUGE_VERSION */
}

static int mtk_pps_get_bif_vbat(struct pps_device *pps, u32 *vbat)
{
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	return pmic_get_bif_battery_voltage(vbat);
#else
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);

	return info->mtk_chg_ctrl(CHARGING_CMD_GET_BIF_VBAT, vbat);
#endif /* CONFIG_MTK_GAUGE_VERSION */
}

static int mtk_pps_get_bif_tbat(struct pps_device *pps, int *tbat)
{
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	return pmic_get_bif_battery_temperature(tbat);
#else
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);

	return info->mtk_chg_ctrl(CHARGING_CMD_GET_BIF_TBAT, tbat);
#endif /* CONFIG_MTK_GAUGE_VERSION */
}

static bool mtk_pps_is_bif_exist(struct pps_device *pps)
{
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	return pmic_is_bif_exist();
#else
	bool exist = false;
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);

	info->mtk_chg_ctrl(CHARGING_CMD_GET_BIF_IS_EXIST, &exist);
	return exist;
#endif /* CONFIG_MTK_GAUGE_VERSION */
}

static void mtk_pps_wake_up_charger(struct pps_device *pps)
{
#if (CONFIG_MTK_GAUGE_VERSION == 30)
	struct mtk_pps_intf_info *info = pps_get_drvdata(pps);

	_wake_up_charger(info->chg_consumer->cm);
#else
	wake_up_bat();
#endif /* CONFIG_MTK_GAUGE_VERSION */
}

static struct pps_chg_ops chg_ops = {
	.enable_power_path = mtk_pps_enable_power_path,
	.enable_charger = mtk_pps_enable_charger,
	.get_tchg = mtk_pps_get_tchg,
	.set_dirchg_ibusoc = mtk_pps_set_dirchg_ibusoc,
	.enable_dirchg_chip = mtk_pps_enable_dirchg_chip,
	.enable_dirchg = mtk_pps_enable_dirchg,
	.get_vbat = mtk_pps_get_vbat,
	.get_ibat = mtk_pps_get_ibat,
	.get_tbat = mtk_pps_get_tbat,
	.get_vbus = mtk_pps_get_vbus,
	.get_ibus = mtk_pps_get_ibus,
	.is_bif_exist = mtk_pps_is_bif_exist,
	.get_bif_vbat = mtk_pps_get_bif_vbat,
	.get_bif_tbat = mtk_pps_get_bif_tbat,
	.wake_up_charger = mtk_pps_wake_up_charger,
};

static int chg_pps_register(struct mtk_pps_intf_info *info)
{
	dev_info(info->dev, "%s\n", __func__);

	info->desc.name = "chg_pps";
	info->desc.type = PPS_TYPE_CHARGER;
	info->pps_dev = pps_device_register(info->dev, &info->desc, NULL,
		&chg_ops, NULL, info);
	if (IS_ERR_OR_NULL(info->pps_dev))
		return PTR_ERR(info->pps_dev);

	return 0;
}

static int mtk_pps_intf_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mtk_pps_intf_info *info;

	dev_info(&pdev->dev, "%s(%s)\n", __func__, MTK_PPS_INTF_VERSION);

	info = devm_kzalloc(&pdev->dev, sizeof(struct mtk_pps_intf_info),
		GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

#if (CONFIG_MTK_GAUGE_VERSION == 30)
	info->pmy_chg = get_charger_by_name("primary_chg");
	if (!info->pmy_chg) {
		dev_err(info->dev, "%s get primary chg fail\n", __func__);
		return -EINVAL;
	}
	info->chg_consumer = charger_manager_get_by_name(&pdev->dev,
		"charger_port1");
	if (!info->chg_consumer) {
		pr_err("%s get charger consumer device fail\n", __func__);
		return -ENODEV;
	}

	info->pmy_ls = get_charger_by_name("primary_load_switch");
	if (!info->pmy_ls) {
		dev_err(info->dev, "%s get primary ls fail\n", __func__);
		return -EINVAL;
	}
#else
	info->mtk_chg_ctrl = chr_control_interface;
	info->mtk_ls_ctrl = load_switch_control_interface;
#endif /* CONFIG_MTK_GAUGE_VERSION */

	ret = chg_pps_register(info);
	if (ret < 0) {
		dev_err(info->dev, "%s register pps fail(%d)\n", __func__,
			ret);
		goto err_pps_register;
	}

	dev_info(info->dev, "%s successfully\n", __func__);
	return 0;

err_pps_register:
	return ret;
}

static int mtk_pps_intf_remove(struct platform_device *pdev)
{
	struct mtk_pps_intf_info *info = platform_get_drvdata(pdev);

	if (info)
		pps_device_unregister(info->pps_dev);

	return 0;
}

static struct platform_device mtk_pps_intf_dev = {
	.name = "mtk_pps_intf_dev",
	.id = -1,
};

static struct platform_driver mtk_pps_intf_driver = {
	.probe = mtk_pps_intf_probe,
	.remove = mtk_pps_intf_remove,
	.driver = {
		.name = "mtk_pps_intf_dev",
		.owner = THIS_MODULE,
	},
};

static int __init mtk_pps_intf_init(void)
{
	platform_device_register(&mtk_pps_intf_dev);
	return platform_driver_register(&mtk_pps_intf_driver);
}

static void __exit mtk_pps_intf_exit(void)
{
	platform_driver_unregister(&mtk_pps_intf_driver);
	platform_device_unregister(&mtk_pps_intf_dev);
}

device_initcall_sync(mtk_pps_intf_init);
module_exit(mtk_pps_intf_exit);

MODULE_DESCRIPTION("PPS Interface for MTK Charging Algorithm");
MODULE_AUTHOR("ShuFanLee <shufan_lee@richtek.com>");
MODULE_VERSION(MTK_PPS_INTF_VERSION);
MODULE_LICENSE("GPL");

/*
 * Revision Note
 * 1.0.2
 * (1) Inform primary charger when enable direct charge
 * (2) Use charger manager API to control power path
 *
 * 1.0.1
 * (1) Add load switch interface for GM20
 *
 * 1.0.0
 * Initial Release
 */
