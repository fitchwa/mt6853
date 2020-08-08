/*
 * Copyright (C) 2017 Richtek Technology Corp.
 *
 * Richtek TypeC Port Control Interface Core Driver
 *
 * Author: Jeff <jeff_chang@richtek.com>
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
#include <linux/notifier.h>
#include <tcpm.h>

#include "pps_class.h"

#define GENERIC_PPS_VERSION "1.0.1_G"
#define GENERIC_PPS_CMD_RETRY_COUNT	2

struct generic_pps_info {
	struct device *dev;
	struct pps_device *pps;
};

static struct pps_desc generic_pps_desc = {
	.name = "ta_pps",
	.type = PPS_TYPE_TA,
};

static int generic_enable_direct_charge(
		struct pps_device *pps, bool en, int mv, int ma)
{
	int ret, cnt = 0;

	do {
		if (en == true)
			ret = tcpm_set_apdo_charging_policy(pps->tcpc,
				DPM_CHARGING_POLICY_PPS, mv, ma, NULL);
		else
			ret = tcpm_reset_pd_charging_policy(pps->tcpc, NULL);
		cnt++;
	} while (ret != TCPM_SUCCESS && cnt < GENERIC_PPS_CMD_RETRY_COUNT);
	return ret;
}

#if 0
static int generic_get_boundary_cap(struct pps_device *pps,
				int *minvolt, int *maxvolt, int *maxcurr)
{
	int ret, cnt = 0;
	struct tcpm_power_cap_val cap;

	do {
		ret = tcpm_inquire_pd_apdo_boundary(pps->tcpc, &cap);
		cnt++;
	} while (ret != TCPM_SUCCESS && cnt < GENERIC_PPS_CMD_RETRY_COUNT);

	if (ret == TCPM_SUCCESS) {
		*minvolt = cap.min_mv;
		*maxvolt = cap.max_mv;
		*maxcurr = cap.ma;
	}
	return ret;
}
#endif

static int generic_set_cap(struct pps_device *pps, int mv, int ma)
{
	int ret, cnt = 0;

	dev_dbg(&pps->dev, "%s(%d mV, %d mA)\r\n", __func__, mv, ma);

	if (!tcpm_inquire_pd_connected(pps->tcpc)) {
		dev_dbg(&pps->dev, "%s pd not connected\n", __func__);
		return -EINVAL;
	}

	do {
		ret = tcpm_dpm_pd_request(pps->tcpc, mv, ma, NULL);
		cnt++;
	} while (ret != TCPM_SUCCESS && cnt < GENERIC_PPS_CMD_RETRY_COUNT);

	dev_dbg(&pps->dev, "%s(ret = %d)\r\n", __func__, ret);
	return ret;
}


static int generic_get_current_cap(struct pps_device *pps, int *mv, int *ma)
{
	int ret, cnt = 0;
	struct pd_pps_status pps_status;

	dev_dbg(&pps->dev, "%s\r\n", __func__);
	do {
		ret = tcpm_dpm_pd_get_pps_status(pps->tcpc, NULL, &pps_status);
		cnt++;
	} while (ret != TCPM_SUCCESS && cnt < GENERIC_PPS_CMD_RETRY_COUNT);

	if (ret == TCPM_SUCCESS) {
		*mv = pps_status.output_mv;
		*ma = pps_status.output_ma;
	}
	dev_dbg(&pps->dev, "%s(%d): %d mV, %d mA\r\n", __func__, ret, *mv, *ma);

	return ret;
}

static int generic_get_temperature(struct pps_device *pps, int *temp)
{
	int ret, cnt = 0;
	struct pd_status status;

	do {
		ret = tcpm_dpm_pd_get_status(pps->tcpc, NULL, &status);
		cnt++;
	} while (ret != TCPM_SUCCESS && cnt < GENERIC_PPS_CMD_RETRY_COUNT);

	if (ret == TCPM_SUCCESS)
		*temp = status.internal_temp;

	return ret;
}

static int generic_get_status(
	struct pps_device *pps, struct pps_ta_status *ta_status)
{
	int ret, cnt = 0;
	struct pd_status status;

	do {
		ret = tcpm_dpm_pd_get_status(pps->tcpc, NULL, &status);
		cnt++;
	} while (ret != TCPM_SUCCESS && cnt < GENERIC_PPS_CMD_RETRY_COUNT);

	if (ret == TCPM_SUCCESS) {
		ta_status->temp = status.internal_temp;
		ta_status->present_input = status.present_input;
		ta_status->present_battery_input = status.present_battey_input;
		ta_status->ocp = status.event_flags & PD_STASUS_EVENT_OCP;
		ta_status->otp = status.event_flags & PD_STATUS_EVENT_OTP;
		ta_status->ovp = status.event_flags & PD_STATUS_EVENT_OVP;
		ta_status->temp_level = PD_STATUS_TEMP_PTF(status.temp_status);
	}
	return ret;
}

static int generic_send_hard_reset(struct pps_device *pps)
{
	int ret = 0, cnt = 0;

	do {
		ret = tcpm_dpm_pd_hard_reset(pps->tcpc, NULL);
		cnt++;
	} while (ret != TCPM_SUCCESS && cnt < GENERIC_PPS_CMD_RETRY_COUNT);

	return ret;
}

static struct pps_ta_ops generic_ta_ops = {
	.enable_direct_charge = generic_enable_direct_charge,
	.set_cap = generic_set_cap,
	.get_current_cap = generic_get_current_cap,
	.get_temperature = generic_get_temperature,
	.get_status = generic_get_status,
	.send_hardreset = generic_send_hard_reset,
};

static int generic_pps_probe(struct platform_device *pdev)
{
	struct generic_pps_info *info;

	dev_info(&pdev->dev, "Probing...");

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);

	info->dev = &pdev->dev;

	info->pps = pps_device_register(&pdev->dev, &generic_pps_desc,
		&generic_ta_ops, NULL, NULL, info);

	if (!info->pps) {
		dev_err(&pdev->dev, "register pps fail\n");
		return -EINVAL;
	}

	info->pps->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!info->pps->tcpc) {
		dev_err(&pdev->dev, "get tcpc type_c_port0 fail\n");
		goto err_tcpc;
	}

	dev_info(&pdev->dev, "probe OK\n");
	return 0;

err_tcpc:
	pps_device_unregister(info->pps);
	return -EINVAL;

}

static int generic_pps_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_device generic_pps_dev = {
	.name = "generic_pps_dev",
	.id = -1,
};

static struct platform_driver generic_pps_driver = {
	.probe = generic_pps_probe,
	.remove = generic_pps_remove,
	.driver = {
		.name = "generic_pps_dev",
		.owner = THIS_MODULE,
	},
};

static int __init generic_pps_init(void)
{

	platform_device_register(&generic_pps_dev);
	return platform_driver_register(&generic_pps_driver);
}

static void __exit generic_pps_exit(void)
{
	platform_driver_unregister(&generic_pps_driver);
}

late_initcall(generic_pps_init);
module_exit(generic_pps_exit);

MODULE_DESCRIPTION("Richtek TypeC Port Control Core");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com>");
MODULE_VERSION(GENERIC_PPS_VERSION);
MODULE_LICENSE("GPL");
