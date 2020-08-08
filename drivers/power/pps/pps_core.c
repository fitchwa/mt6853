/*
 * Copyright (C) 2017 Richtek Technology Corp.
 *
 * Richtek PPS Algotirhm & Interface
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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/notifier.h>
#include <tcpm.h>

#include <mt-plat/mtk_battery.h>

#include "pps_class.h"
#include "pps_core.h"

#define RT_PPS_ALGO_VERSION	"1.0.10_G"
#define MS_TO_NS(msec) ((msec) * 1000 * 1000)

struct rt_pps_algo_info {
	struct device *dev;
	struct pps_device *pps_dev;
	struct pps_desc desc;
	struct rt_pps_data pps_data;
	struct rt_dc_data dc_data;
	struct rt_dc_desc dc_desc;
};

/* if there's no property in dts, these values will by applied */
static struct rt_dc_desc default_dc_desc = {
	.polling_mode = true,
	.polling_interval = 500,
	.cv_limit = 5900,
	.bat_upper_bound = 4350,
	.pps_dc_start_battery_soc = 5,
	.cc_ss_init = 1500,
	.cc_init = 6000,
	.cc_init_bad_cable0 = 5000,
	.cc_init_bad_cable1 = 4000,
	.cc_init_bad_cable2 = 3000,
	.cc_init_r = 125,
	.cc_init_bad_cable0_r = 150,
	.cc_init_bad_cable1_r = 188,
	.cc_init_bad_cable2_r = 250,
	.cc_normal = 4000,
	.cc_max = 6500,
	.cc_end = 2000,
	.cc_step = 200,
	.cc_ss_step = 500,
	.cc_ss_step1 = 200,
	.cc_ss_step2 = 100,
	.cv_ss_step1 = 4000,
	.cv_ss_step2 = 4200,
	.cc_ss_blanking = 100,
	.cc_blanking = 1000,
	.charger_temp_max = 70,
	.ta_warn_level1_temp = 50,
	.ta_warn_level2_temp = 60,
	.ta_warn_level3_temp = 70,
	.ta_temp_max = 80,
	.vbus_ov_gap = 100,
	.fod_current = 2000,
	.r_vbat_min = 900,
	.r_sw_min = 20,
	.bat_temp_min = 0,
	.bat_warn_level1_temp = 40,
	.bat_warn_level2_temp = 45,
	.bat_warn_level3_temp = 50,
	.bat_temp_max = 55,
	.curr_reco_temp_region = 3,
	.ta_pps_cap_vmin = 3000,
	.ta_pps_cap_vmax = 5900,
	.ta_pps_cap_imin = 3000,
	.chg_time_max = 5400,
	.need_bif = false,
};

struct meas_r_info {
	u32 vbus;
	u32 vbat;
	u32 bifvbat;
	s32 ibat;
	u32 ita;
	u32 vta;
};

static inline int pps_get_cali_vbus(struct rt_pps_algo_info *info,
	u32 chg_cur, u32 *cali_vbus)
{
	int ret = 0;
	u32 vbat = 0;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;
	struct rt_pps_data *pps_data = &info->pps_data;
	int r_cali = dc_data->is_bif_exist ? dc_data->r_total :
		(dc_data->r_cable + dc_data->r_sw);

	ret = (dc_data->is_bif_exist ? rt_pps_get_bif_vbat :
		rt_pps_get_vbat)(pps_data->chg_pps, &vbat);
	if (ret < 0) {
		PPS_ERR("%s get vbat fail\n", __func__);
		return ret;
	}

	*cali_vbus = vbat + 50 + chg_cur * r_cali / 1000;
	if (*cali_vbus >= dc_desc->cv_limit)
		*cali_vbus = dc_desc->cv_limit;
	return 0;
}

static inline int pps_get_chg_cur_lmt(struct rt_pps_algo_info *info)
{
	struct rt_dc_data *dc_data = &info->dc_data;
	int chg_curr = (dc_data->chg_cur_lmt_by_r < dc_data->chg_cur_lmt) ?
			dc_data->chg_cur_lmt_by_r : dc_data->chg_cur_lmt;
	int curr_step = (chg_curr >= 6000) ? 1000 : 500;
	int ot_level = (dc_data->bat_ot_level > dc_data->ta_ot_level) ?
			dc_data->bat_ot_level : dc_data->ta_ot_level;

	chg_curr -= curr_step * ot_level;

	return chg_curr;
}

static int pps_check_bat_ot_level(struct rt_pps_algo_info *info)
{
	int ret = 0;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;

	switch (dc_data->bat_ot_level) {
	case 0:
		if (dc_data->tbat >=
		    dc_desc->bat_warn_level1_temp)
			dc_data->bat_ot_level = 1;
		break;
	case 1:
		if (dc_data->tbat <=
		    dc_desc->bat_warn_level1_temp -
		    dc_desc->curr_reco_temp_region)
			dc_data->bat_ot_level = 0;
		else if (dc_data->tbat >= dc_desc->bat_warn_level2_temp)
			dc_data->bat_ot_level = 2;
		break;
	case 2:
		if (dc_data->tbat <=
		    dc_desc->bat_warn_level2_temp -
		    dc_desc->curr_reco_temp_region)
			dc_data->bat_ot_level = 1;
		else if (dc_data->tbat >= dc_desc->bat_warn_level3_temp)
			dc_data->bat_ot_level = 3;
		break;
	case 3:
		if (dc_data->tbat <=
		    dc_desc->bat_warn_level3_temp -
		    dc_desc->curr_reco_temp_region)
			dc_data->bat_ot_level = 2;
		break;
	default:
		PPS_DBG("%s NO SUCH STATE\n", __func__);
		break;
	}
	PPS_INFO("%s: tbat(%d), bat_ot_level(%d)\n", __func__, dc_data->tbat,
		 dc_data->bat_ot_level);
	return ret;
}

static int pps_check_ta_ot_level(struct rt_pps_algo_info *info)
{
	int ret = 0;
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;

	switch (dc_data->ta_ot_level) {
	case 0:
		if (pps_data->ta_current_temp >=
		    dc_desc->ta_warn_level1_temp)
			dc_data->ta_ot_level = 1;
		break;
	case 1:
		if (pps_data->ta_current_temp <=
		    dc_desc->ta_warn_level1_temp -
		    dc_desc->curr_reco_temp_region)
			dc_data->ta_ot_level = 0;
		else if (pps_data->ta_current_temp >=
			 dc_desc->ta_warn_level2_temp)
			dc_data->ta_ot_level = 2;
		break;
	case 2:
		if (pps_data->ta_current_temp <=
		    dc_desc->ta_warn_level2_temp -
		    dc_desc->curr_reco_temp_region)
			dc_data->ta_ot_level = 1;
		else if (pps_data->ta_current_temp >=
			 dc_desc->ta_warn_level3_temp)
			dc_data->ta_ot_level = 3;
		break;
	case 3:
		if (pps_data->ta_current_temp <=
		    dc_desc->ta_warn_level3_temp -
		    dc_desc->curr_reco_temp_region)
			dc_data->ta_ot_level = 2;
		break;
	default:
		PPS_DBG("%s NO SUCH STATE\n", __func__);
		break;
	}
	PPS_INFO("%s: TA temp(%d), ta_ot_level(%d)\n", __func__,
		 pps_data->ta_current_temp, dc_data->ta_ot_level);
	return ret;
}

static int pps_enable_chg_dirchg(struct rt_pps_algo_info *info, bool en)
{
	int ret = 0;
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;
	int ibusoc = en ? ((dc_data->chg_cur_lmt_by_r < dc_data->chg_cur_lmt) ?
			dc_data->chg_cur_lmt_by_r : dc_data->chg_cur_lmt)
			: dc_desc->cc_normal;

	PPS_DBG("%s en = %d\n", __func__, en);

	ret = rt_pps_set_dirchg_ibusoc(pps_data->chg_pps, ibusoc);
	if (ret < 0)
		PPS_DBG("%s set dirchg_ibusoc fail(%d)\n", __func__, ret);

	ret = rt_pps_enable_dirchg(pps_data->chg_pps, en);
	if (ret < 0)
		PPS_DBG("%s en_dirchg fail(%d)\n", __func__, ret);

	return ret;
}

static int pps_enable_ta_dirchg(struct rt_pps_algo_info *info, bool en,
	int mV, int mA)
{
	int ret = 0;
	struct rt_pps_data *pps_data = &info->pps_data;

	PPS_DBG("%s en = %d\n", __func__, en);
	ret = rt_pps_enable_ta_direct_charge(pps_data->ta_pps, en, mV, mA);
	if (ret != 0)
		PPS_DBG("%s en ta dirchg fail(%d)\n", __func__, ret);

	return ret;
}

static bool pps_check_ta_status(struct rt_pps_algo_info *info)
{
	int ret = 0;
	struct pps_ta_status ta_status;
	struct rt_pps_data *pps_data = &info->pps_data;

	ret = rt_pps_get_ta_status(pps_data->ta_pps, &ta_status);
	if (ret != 0) {
		PPS_ERR("%s get ta status fail(%d)\n", __func__, ret);
		return false;
	}

	PPS_DBG("%s temp = %d, temperature level = %d\n", __func__,
		ta_status.temp, ta_status.temp_level);
	PPS_DBG("%s present_input = 0x%x, present_battey_input = 0x%x\n",
		__func__, ta_status.present_input,
		ta_status.present_battery_input);
	PPS_DBG("%s ocp = %d, otp = %d, ovp = %d\n", __func__,
		ta_status.ocp, ta_status.otp, ta_status.ovp);

	if (ta_status.ocp == 1 || ta_status.otp == 1 || ta_status.ovp == 1)
		return false;

	return true;
}

static int pps_end(struct rt_pps_algo_info *info, bool reset_ta, bool hrst)
{
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;

	PPS_DBG("%s reset_ta = %d, hardreset = %d\n", __func__,
		reset_ta, hrst);

	if (dc_data->state == DC_STOP) {
		PPS_DBG("%s already stop\n", __func__);
		return 0;
	}

	/* reset pps_data */
	pps_data->ta_current_volt = 0;
	pps_data->ta_current_cur = 0;
	pps_data->ta_current_temp = 0;

	/* reset dc data */
	dc_data->state = DC_STOP;

	if (reset_ta) {
		pps_data->ta_setting_volt = 5000;
		pps_data->ta_setting_cur = 3000;
		rt_pps_set_ta_cap(pps_data->ta_pps,
			pps_data->ta_setting_volt,
			pps_data->ta_setting_cur);
		tcpm_set_direct_charge_en(pps_data->ta_pps->tcpc, false);
		pps_enable_ta_dirchg(info, false, 0, 0);
	}

	pps_enable_chg_dirchg(info, false);
	rt_pps_enable_dirchg_chip(pps_data->chg_pps, false);

	if (reset_ta && hrst)
		rt_pps_send_ta_hard_reset(pps_data->ta_pps);

	rt_pps_wake_up_charger(pps_data->chg_pps);
	return 0;
}

static inline void pps_dc_wakeup_thread(struct rt_dc_data *dc_data)
{
	PPS_DBG("%s\n", __func__);

	/* wakeup thread */
	if (!wake_up_process(dc_data->thread))
		PPS_ERR("%s: wakeup thread fail\n", __func__);
}

static inline int pps_dc_start(struct rt_pps_algo_info *info)
{
	int ret = 0;
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;
	ktime_t ktime = ktime_set(0, MS_TO_NS(dc_desc->polling_interval));

	PPS_DBG("%s\n", __func__);

	if (atomic_read(&dc_data->is_dc_once)) {
		PPS_ERR("%s already run dc once\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	/* check bif */
	dc_data->is_bif_exist = rt_pps_is_bif_exist(pps_data->chg_pps);
	if (dc_desc->need_bif && !dc_data->is_bif_exist) {
		PPS_ERR("%s no bif\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	/* disable charger */
	ret = rt_pps_enable_charger(pps_data->chg_pps, false);
	if (ret < 0) {
		PPS_ERR("%s disable charger fail\n", __func__);
		goto out;
	}
	msleep(500); /* Wait for battery to recovery */

	/* enable load switch chip */
	ret = rt_pps_enable_dirchg_chip(pps_data->chg_pps, true);
	if (ret < 0) {
		PPS_ERR("%s en dirchg chip fail\n", __func__);
		goto out;
	}

	atomic_set(&dc_data->is_dc_once, 1);
	dc_data->chg_cur_lmt =
		(dc_desc->cc_init < pps_data->ta_boundary_max_cur) ?
		dc_desc->cc_init : pps_data->ta_boundary_max_cur;
	dc_data->chg_cur_lmt_by_r = dc_data->chg_cur_lmt;
	dc_data->state = DC_INIT;
	get_monotonic_boottime(&dc_data->start_time);
	hrtimer_start(&dc_data->timer, ktime, HRTIMER_MODE_REL);

out:
	return ret;
}

static int pps_dc_init(struct rt_pps_algo_info *info)
{
	int i, ret = 0;
	const int avg_times = 10;
	bool hrst = true;
	int ita_avg = 0, vta_avg = 0, vbus_avg = 0;
	s32 ibat = 0;
	u32 vbus = 0;
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;

	PPS_DBG("%s\n", __func__);

	tcpm_set_direct_charge_en(pps_data->ta_pps->tcpc, true);

	ret = rt_pps_get_vbat(pps_data->chg_pps, &dc_data->zcv);
	if (ret < 0) {
		PPS_ERR("%s get vbat fail\n", __func__);
		goto err;
	}
	ret = rt_pps_get_ibat(pps_data->chg_pps, &ibat);
	if (ret < 0) {
		PPS_ERR("%s get ibat fail\n", __func__);
		goto err;
	}
	PPS_DBG("%s zcv:%d, ibat:%d\n", __func__, dc_data->zcv, ibat);

	/* Foreign Object Detection (FOD) */
	ret = rt_pps_enable_power_path(pps_data->chg_pps, false);
	if (ret < 0) {
		PPS_ERR("%s disable power path fail\n", __func__);
		goto err;
	}

	/* before use pps, we should change policy first */
	ret = pps_enable_ta_dirchg(info, true, 5000, 3000);
	if (ret != 0) {
		PPS_ERR("%s enable ta direct charge fail\n", __func__);
		goto err_en_powerpath;
	}

	for (i = 0; i < avg_times; i++) {
		ret = rt_pps_get_ta_current_cap(pps_data->ta_pps,
			&pps_data->ta_current_volt,
			&pps_data->ta_current_cur);
		if (ret != 0) {
			PPS_ERR("%s get ta current cap fail\n", __func__);
			goto err_en_powerpath;
		}
		ret = rt_pps_get_vbus(pps_data->chg_pps, &vbus);
		if (ret < 0) {
			PPS_ERR("%s get vbus fail\n", __func__);
			goto err_en_powerpath;
		}
		ita_avg += pps_data->ta_current_cur;
		vta_avg += pps_data->ta_current_volt;
		vbus_avg += vbus;
	}
	ita_avg /= avg_times;
	vta_avg /= avg_times;
	vbus_avg /= avg_times;

	/* vbus calibration */
	if (vta_avg < vbus_avg)
		PPS_ERR("%s warning: ta volt < chg vbus\n", __func__);
	dc_data->vbus_cali = vta_avg - vbus_avg;

	PPS_DBG("%s avg(ita,vta,vbus,vcali):(%d, %d, %d, %d)\n", __func__,
		ita_avg, vta_avg, vbus_avg, dc_data->vbus_cali);

	if (ita_avg > dc_desc->fod_current) {
		hrst = false;
		PPS_DBG("%s foreign object detected\n", __func__);
		goto err_en_powerpath;
	}

	ret = rt_pps_enable_power_path(pps_data->chg_pps, true);
	if (ret < 0) {
		PPS_ERR("%s en power path fail\n", __func__);
		goto err_en_powerpath;
	}

	ret = rt_pps_set_ta_cap(pps_data->ta_pps, 4500, 2000);
	if (ret != 0) {
		PPS_ERR("%s set ta cap fail(%d)\n", __func__, ret);
		goto err;
	}

	/* wait for TA */
	msleep(dc_desc->cc_ss_blanking);

	ret = rt_pps_set_ta_cap(pps_data->ta_pps, 4000, 1500);
	if (ret != 0) {
		PPS_ERR("%s set ta cap fail(%d)\n", __func__, ret);
		goto err;
	}

	/* wait for TA */
	msleep(dc_desc->cc_ss_blanking);

	/* set TA to soft start setting before enable dirchg */
	pps_data->ta_setting_volt = dc_data->zcv + 50;
	pps_data->ta_setting_cur = dc_desc->cc_ss_init;
	ret = rt_pps_set_ta_cap(pps_data->ta_pps,
		pps_data->ta_setting_volt, pps_data->ta_setting_cur);
	if (ret != 0) {
		PPS_ERR("%s set pps ta cap fail(%d)\n", __func__, ret);
		goto err;
	}

	/* wait for TA */
	msleep(dc_desc->cc_ss_blanking);

	ret = pps_enable_chg_dirchg(info, true);
	if (ret < 0) {
		PPS_ERR("%s en chg dirchg fail\n", __func__);
		goto err;
	}

	dc_data->is_vbat_over_cv = false;
	dc_data->cv_lower_bound = dc_desc->bat_upper_bound - 15;
	dc_data->need_add_curr_cnt = 0;
	dc_data->bat_ot_level = 0;
	dc_data->ta_ot_level = 0;
	dc_data->state = DC_MEASURE_R;
	return 0;

err_en_powerpath:
	rt_pps_enable_power_path(pps_data->chg_pps, true);
err:
	return pps_end(info, true, hrst);
}

static int __pps_dc_get_r_info(struct rt_pps_algo_info *info, int ita,
	struct meas_r_info *r_info)
{
	int ret = 0;
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;

	pps_data->ta_setting_volt = dc_data->zcv + dc_data->vbus_cali +
		(dc_desc->cc_init_r + dc_desc->r_vbat_min +
		 dc_desc->r_sw_min) * ita / 1000;
	if (pps_data->ta_setting_volt < 3700)
		pps_data->ta_setting_volt = 3700;
	else if (pps_data->ta_setting_volt > 5700)
		pps_data->ta_setting_volt = 5700;
	pps_data->ta_setting_cur = ita;

	PPS_DBG("%s: ta_setting_volt = %d, ta_setting_cur = %d\n", __func__, pps_data->ta_setting_volt, pps_data->ta_setting_cur);

	ret = rt_pps_set_ta_cap(pps_data->ta_pps,
		pps_data->ta_setting_volt,
		pps_data->ta_setting_cur);
	if (ret != 0) {
		PPS_ERR("%s set ta cap fail(%d)\n", __func__, ret);
		goto err;
	}

	/* wait for TA */
	msleep(300);

	ret = rt_pps_get_vbus(pps_data->chg_pps, &r_info->vbus);
	if (ret < 0) {
		PPS_ERR("%s get vbus fail\n", __func__);
		goto err;
	}
	ret = rt_pps_get_vbat(pps_data->chg_pps, &r_info->vbat);
	if (ret < 0) {
		PPS_ERR("%s get vbat fail\n", __func__);
		goto err;
	}
	if (dc_data->is_bif_exist) {
		ret = rt_pps_get_bif_vbat(pps_data->chg_pps,
			&r_info->bifvbat);
		if (ret < 0) {
			PPS_ERR("%s get bif vbat fail\n", __func__);
			goto err;
		}
	}
	ret = rt_pps_get_ibat(pps_data->chg_pps, &r_info->ibat);
	if (ret < 0) {
		PPS_ERR("%s get ibat fail\n", __func__);
		goto err;
	}

	ret = rt_pps_get_ta_current_cap(pps_data->ta_pps, &r_info->vta,
		&r_info->ita);
	if (ret != 0) {
		PPS_DBG("%s get ta current cap fail(%d)\n", __func__, ret);
		goto err;
	}

	pps_data->ta_current_volt = r_info->vta;
	pps_data->ta_current_cur = r_info->ita;

err:
	return ret;
}

static int pps_dc_measure_r(struct rt_pps_algo_info *info)
{
	int ret = 0;
	bool hrst = true;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;
	struct meas_r_info r_info[2];
	int r_cable_ref[2] = {0};

	PPS_DBG("%s\n", __func__);

	/* measure 1 */
	ret = __pps_dc_get_r_info(info, dc_desc->cc_end - 500, &r_info[0]);
	if (ret < 0) {
		PPS_ERR("%s: meas 1 get r info fail(%d)\n", __func__, ret);
		return ret;
	}

	/* BIF VBat > CV_END */
	if (dc_data->is_bif_exist && r_info[0].bifvbat != 0 &&
		r_info[0].bifvbat >= dc_desc->bat_upper_bound) {
		PPS_DBG("%s mea1 vbif(%d) >= CV_END(%d), ita:%d, vta:%d\n",
			__func__, r_info[0].bifvbat,
			dc_desc->bat_upper_bound, r_info[0].ita,
			r_info[0].vta);
		hrst = false;
		goto err;
	}

	/* measure 2 */
	ret = __pps_dc_get_r_info(info, dc_desc->cc_end, &r_info[1]);
	if (ret < 0) {
		PPS_ERR("%s: meas 2 get r info fail(%d)\n", __func__, ret);
		goto err;
	}

	/* BIF VBat > CV_END */
	if (dc_data->is_bif_exist && r_info[1].bifvbat != 0 &&
		r_info[1].bifvbat >= dc_desc->bat_upper_bound) {
		PPS_DBG("%s mea2 vbif(%d) >= CV_END(%d), ita:%d, vta:%d\n",
			__func__, r_info[1].bifvbat,
			dc_desc->bat_upper_bound, r_info[1].ita,
			r_info[1].vta);
		hrst = false;
		goto err;
	}

	/* if TA does not support different current level */
	if (r_info[0].ita == r_info[1].ita) {
		PPS_ERR("%s ita1(%d) = ita2(%d)\n", __func__,
			r_info[0].ita, r_info[1].ita);
		return 0;
	}

	/* use bifvbat to calculate r_vbat */
	if (dc_data->is_bif_exist && r_info[0].bifvbat != 0 &&
		r_info[1].bifvbat != 0) {
		dc_data->r_vbat = abs((r_info[1].vbat - r_info[1].bifvbat)
			- (r_info[0].vbat - r_info[0].bifvbat)) * 1000 /
			abs(r_info[1].ibat - r_info[0].ibat);
	}
	if (dc_data->r_vbat < dc_desc->r_vbat_min)
		dc_data->r_vbat = dc_desc->r_vbat_min;

	dc_data->r_sw = abs((r_info[1].vbus - r_info[1].vbat) -
		(r_info[0].vbus - r_info[0].vbat)) * 1000 /
		abs(r_info[1].ita - r_info[0].ita);
	if (dc_data->r_sw < dc_desc->r_sw_min)
		dc_data->r_sw = dc_desc->r_sw_min;

	/* Use this one instead of relative caculation */
	dc_data->r_cable = abs(r_info[1].vta - dc_data->vbus_cali -
		r_info[1].vbus) * 1000 / abs(r_info[1].ita);

	r_cable_ref[0] = abs(r_info[0].vta - dc_data->vbus_cali -
		r_info[0].vbus) * 1000 / abs(r_info[0].ita);

	/* Relative caculation might have larger variation */
	r_cable_ref[1] = abs((r_info[1].vta - dc_data->vbus_cali -
		r_info[1].vbus) - (r_info[0].vta - dc_data->vbus_cali -
		r_info[0].vbus)) * 1000 / abs(r_info[1].ita -
		r_info[0].ita);

	dc_data->r_total = dc_data->r_vbat + dc_data->r_sw +
		dc_data->r_cable;

	PPS_DBG("%s ita:%d %d, vta:%d %d, zcv:%d\n", __func__,
		r_info[0].ita, r_info[1].ita, r_info[0].vta, r_info[1].vta,
		dc_data->zcv);

	PPS_DBG("%s vbus:%d %d, vbat:%d %d, bifvbat:%d %d, ibat:%d %d\n",
		__func__, r_info[0].vbus, r_info[1].vbus, r_info[0].vbat,
		r_info[1].vbat, r_info[0].bifvbat, r_info[1].bifvbat,
		r_info[0].ibat, r_info[1].ibat);

	PPS_DBG("%s r_sw:%d, r_vbat:%d, r_cable:%d,%d,%d, r_total:%d\n",
		__func__, dc_data->r_sw, dc_data->r_vbat, dc_data->r_cable,
		r_cable_ref[0], r_cable_ref[1], dc_data->r_total);

	if (dc_data->r_cable < dc_desc->cc_init_r)
		dc_data->chg_cur_lmt_by_r = dc_desc->cc_init;
	else if (dc_data->r_cable < dc_desc->cc_init_bad_cable0_r)
		dc_data->chg_cur_lmt_by_r = dc_desc->cc_init_bad_cable0;
	else if (dc_data->r_cable < dc_desc->cc_init_bad_cable1_r)
		dc_data->chg_cur_lmt_by_r = dc_desc->cc_init_bad_cable1;
	else if (dc_data->r_cable < dc_desc->cc_init_bad_cable2_r)
		dc_data->chg_cur_lmt_by_r = dc_desc->cc_init_bad_cable2;
	else {
		PPS_ERR("%s r_cable(%d) too high\n", __func__,
			dc_data->r_cable);
		hrst = false;
		goto err;
	}
	PPS_DBG("%s chg cur limited by r = %d\n", __func__,
		dc_data->chg_cur_lmt_by_r);

	if (pps_get_chg_cur_lmt(info) < dc_desc->cc_end) {
		PPS_DBG("%s chg cur lmt(%d) < CC_END(%d)\n", __func__,
			pps_get_chg_cur_lmt(info), dc_desc->cc_end);
		hrst = false;
		goto err;
	}

	dc_data->state = DC_SOFT_START;
	return 0;

err:
	return pps_end(info, true, hrst);
}

static int pps_dc_soft_start(struct rt_pps_algo_info *info)
{
	int ret = 0;
	bool hrst = true;
	u32 cali_vbus = 0, vbat = 0;
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;

	PPS_DBG("%s\n", __func__);

	ret = rt_pps_get_ta_current_cap(pps_data->ta_pps,
		&pps_data->ta_current_volt, &pps_data->ta_current_cur);
	if (ret != 0) {
		PPS_DBG("%s get ta current cap fail(%d)\n", __func__, ret);
		goto err;
	}

	ret = (dc_data->is_bif_exist ? rt_pps_get_bif_vbat :
		rt_pps_get_vbat)(pps_data->chg_pps, &vbat);
	if (ret < 0) {
		PPS_DBG("%s get vbat fail\n", __func__);
		goto err;
	}

	if (vbat >= dc_desc->bat_upper_bound) {
		if (pps_data->ta_current_cur >= dc_desc->cc_end) {
			/* set new ta volt & cur */
			pps_data->ta_setting_cur -= dc_desc->cc_ss_step;
			ret = pps_get_cali_vbus(info,
				pps_data->ta_setting_cur, &cali_vbus);
			if (ret < 0) {
				PPS_ERR("%s get cali vbus fail(%d)\n",
					__func__, ret);
				goto err;
			}
			pps_data->ta_setting_volt = cali_vbus;
			ret = rt_pps_set_ta_cap(pps_data->ta_pps,
				pps_data->ta_setting_volt,
				pps_data->ta_setting_cur);
			if (ret != 0) {
				PPS_ERR("%s set ta cap fail(%d)\n",
					__func__, ret);
				goto err;
			}

			/* wait for TA */
			msleep(dc_desc->cc_blanking);
			dc_data->state = DC_CC;
			goto out;
		}
		PPS_DBG("%s end (vbat = %d, ita = %d, cc_end = %d)\n",
			__func__, vbat, pps_data->ta_current_cur,
			dc_desc->cc_end);
		hrst = false;
		goto err;
	}

	if (pps_data->ta_current_cur >= pps_get_chg_cur_lmt(info) ||
		pps_data->ta_setting_cur >= pps_get_chg_cur_lmt(info)) {
		dc_data->state = DC_CC;
		msleep(dc_desc->cc_blanking); /* wait for TA */
		goto out;
	}

	if (vbat >= dc_desc->cv_ss_step2)
		pps_data->ta_setting_cur += dc_desc->cc_ss_step2;
	else if (vbat >= dc_desc->cv_ss_step1)
		pps_data->ta_setting_cur += dc_desc->cc_ss_step1;
	else
		pps_data->ta_setting_cur += dc_desc->cc_ss_step;
	if (pps_data->ta_setting_cur >= pps_get_chg_cur_lmt(info))
		pps_data->ta_setting_cur = pps_get_chg_cur_lmt(info);

	ret = pps_get_cali_vbus(info, pps_data->ta_setting_cur,
		&cali_vbus);
	if (ret < 0) {
		PPS_ERR("%s get cali vbus fail(%d)\n", __func__, ret);
		goto err;
	}

	pps_data->ta_setting_volt = cali_vbus;
	ret = rt_pps_set_ta_cap(pps_data->ta_pps,
		pps_data->ta_setting_volt, pps_data->ta_setting_cur);
	if (ret != 0) {
		PPS_DBG("%s set ta cap fail(%d)\n", __func__, ret);
		goto err;
	}

out:
	PPS_DBG("%s vbat: %d, vta,ita(c,s):(%d,%d,%d,%d)\n", __func__,
		vbat, pps_data->ta_current_volt, pps_data->ta_setting_volt,
		pps_data->ta_current_cur, pps_data->ta_setting_cur);
	return 0;

err:
	return pps_end(info, true, hrst);
}

static int pps_dc_cc(struct rt_pps_algo_info *info)
{
	int ret = 0;
	u32 vbat = 0, cali_vbus = 0;
	int tchg_min = 0, tchg_max = 0;
	bool hrst = true;
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;

	PPS_DBG("%s\n", __func__);

	ret = (dc_data->is_bif_exist ? rt_pps_get_bif_vbat :
		rt_pps_get_vbat)(pps_data->chg_pps, &vbat);
	if (ret < 0)
		PPS_DBG("%s get vbat fail\n", __func__);

	PPS_DBG("%s vbat = %d\n", __func__, vbat);

	ret = rt_pps_get_ta_current_cap(pps_data->ta_pps,
		&pps_data->ta_current_volt, &pps_data->ta_current_cur);
	if (ret != 0) {
		PPS_DBG("%s get_ta_current_cap fail(%d)\n", __func__, ret);
		goto err;
	}
	PPS_DBG("%s vta_cur:%d ita_cur:%d\n", __func__,
		pps_data->ta_current_volt, pps_data->ta_current_cur);

	/* finish direct charge */
	if (pps_data->ta_current_cur <= dc_desc->cc_end) {
		PPS_DBG("%s finish direct charge, cur = %dmA\n", __func__,
			pps_data->ta_current_cur);
		hrst = false;
		return pps_end(info, true, hrst);
	}

	/* dynamic calculus cv_lower_bound */
	if (dc_data->is_vbat_over_cv) {
		dc_data->cv_lower_bound = vbat - 5;
		dc_data->need_add_curr_cnt = 0;
		dc_data->is_vbat_over_cv = false;
	}

	ret = rt_pps_get_tchg(pps_data->chg_pps, &tchg_min, &tchg_max);
	if (ret < 0) {
		PPS_DBG("%s get tchr fail(%d)\n", __func__, ret);
		goto err;
	}

	/* Set vta,ita for next round */
	if (vbat >= dc_desc->bat_upper_bound ||
		tchg_max >= dc_desc->charger_temp_max) {
		pps_data->ta_setting_cur -= dc_desc->cc_step;
		dc_data->is_vbat_over_cv = true;
	} else if (vbat <= dc_data->cv_lower_bound) {
		if (dc_data->need_add_curr_cnt > 5)
			pps_data->ta_setting_cur += dc_desc->cc_step;
		else
			dc_data->need_add_curr_cnt++;
	}

	if (pps_data->ta_setting_cur > pps_get_chg_cur_lmt(info)) {
		PPS_DBG("%s ita setting over lmt:%d lmt:%d\n", __func__,
			pps_data->ta_setting_cur,
			pps_get_chg_cur_lmt(info));
		pps_data->ta_setting_cur = pps_get_chg_cur_lmt(info);
		if (pps_get_chg_cur_lmt(info) < dc_desc->cc_end) {
			PPS_DBG("%s chg cur lmt(%d) < CC_END(%d)\n", __func__,
				pps_get_chg_cur_lmt(info), dc_desc->cc_end);
			hrst = false;
			goto err;
		}
	}

	ret = pps_get_cali_vbus(info, pps_data->ta_setting_cur,
		&cali_vbus);
	if (ret < 0) {
		PPS_ERR("%s get cali vbus fail(%d)\n", __func__, ret);
		goto err;
	}

	pps_data->ta_setting_volt = cali_vbus;
	ret = rt_pps_set_ta_cap(pps_data->ta_pps,
		pps_data->ta_setting_volt, pps_data->ta_setting_cur);
	if (ret != 0) {
		PPS_DBG("%s set_ta_cap fail(%d)\n", __func__, ret);
		goto err;
	}

	PPS_DBG("%s vbat:%d, vta,ita(c,s):(%d,%d,%d,%d), r:%d, tchg:%d\n",
		__func__, vbat, pps_data->ta_current_volt,
		pps_data->ta_setting_volt, pps_data->ta_current_cur,
		pps_data->ta_setting_cur, dc_data->r_total, tchg_max);

	return 0;

err:
	return pps_end(info, true, hrst);
}

static bool pps_dc_safety_check(struct rt_pps_algo_info *info)
{
	int ret = 0;
	bool hrst = false;
	u32 ibus = 0, vbus = 0, vbat = 0, vbatov = 0, vbusov = 0;
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;

	PPS_DBG("%s\n", __func__);

	if (!pps_check_ta_status(info)) {
		PPS_ERR("%s check TA status fail\n", __func__);
		hrst = true;
		goto err;
	}

	/* vbusov */
	ret = rt_pps_get_vbus(pps_data->chg_pps, &vbus);
	if (ret < 0) {
		PPS_ERR("%s get vbus fail\n", __func__);
		goto err;
	}
	if (dc_data->state != DC_INIT && dc_data->state != DC_MEASURE_R) {
		vbusov = dc_desc->bat_upper_bound + 70 +
			110 * pps_get_chg_cur_lmt(info) / 100 *
			(dc_data->r_vbat + dc_data->r_sw) / 1000;
		if (vbus >= vbusov) {
			PPS_ERR("%s vbus(%d) >= vbusov(%d)\n", __func__,
				vbus, vbusov);
			goto err;
		}
	}

	/* ibusoc: charger's ADC current */
	ret = rt_pps_get_ibus(pps_data->chg_pps, &ibus);
	if (ret < 0) {
		PPS_DBG("%s get ibus fail(%d)\n", __func__, ret);
		goto err;
	}
	if (ibus >= dc_desc->cc_max) {
		PPS_DBG("%s ibus(%d) >= ccmax(%d)\n", __func__, ibus,
			dc_desc->cc_max);
		goto err;
	}
	PPS_DBG("%s ibus = %dmA\n", __func__, ibus);

	/* ibusoc: TA ADC current */
	rt_pps_get_ta_current_cap(pps_data->ta_pps,
		&pps_data->ta_current_volt, &pps_data->ta_current_cur);
	if (pps_data->ta_current_cur >= dc_desc->cc_max) {
		PPS_DBG("%s ita(%d) >= ccmax(%d)\n", __func__,
			pps_data->ta_current_cur, dc_desc->cc_max);
		hrst = true;
		goto err;
	}

	/* vbatov */
	ret = (dc_data->is_bif_exist ? rt_pps_get_bif_vbat :
		rt_pps_get_vbat)(pps_data->chg_pps, &vbat);
	if (ret < 0) {
		PPS_DBG("%s get vbat fail(%d)\n", __func__, ret);
		goto err;
	}
	if (dc_data->is_bif_exist)
		vbat += ibus * dc_data->r_vbat / 1000;
	if (dc_data->state != DC_INIT && dc_data->state != DC_MEASURE_R) {
		vbatov = dc_desc->bat_upper_bound + 50 +
			110 * pps_get_chg_cur_lmt(info) / 100 *
			dc_data->r_vbat / 1000;
		if (vbat >= vbatov) {
			PPS_DBG("%s vbat(%d) >= vbatovp(%d)\n", __func__,
				vbat, vbatov);
			goto err;
		}
	}

	/* battery temperature */
	ret = rt_pps_get_tbat(pps_data->chg_pps, &dc_data->tbat);
	if (ret < 0) {
		PPS_DBG("%s get tbat fail(%d)\n", __func__, ret);
		goto err;
	}
	PPS_DBG("%s tbat(%d)\n", __func__, dc_data->tbat);
	if (dc_data->tbat > dc_desc->bat_temp_max) {
		PPS_DBG("%s tbat(%d) > otp(%d)\n", __func__,
			dc_data->tbat, dc_desc->bat_temp_max);
		goto err;
	} else if (dc_data->tbat < dc_desc->bat_temp_min) {
		PPS_DBG("%s tbat(%d) < utp(%d)\n", __func__,
			dc_data->tbat, dc_desc->bat_temp_min);
		goto err;
	}
	pps_check_bat_ot_level(info);

	/* TA thermal */
	ret = rt_pps_get_ta_temperature(pps_data->ta_pps,
		&pps_data->ta_current_temp);
	if (ret != 0) {
		PPS_DBG("%s get ta temperature fail(%d)\n", __func__, ret);
		hrst = true;
		goto err;
	}
	if (pps_data->ta_current_temp >= dc_desc->ta_temp_max) {
		PPS_DBG("%s TA temp(%d) >= otp(%d)\n", __func__,
			pps_data->ta_current_temp, dc_desc->ta_temp_max);
		hrst = true;
		goto err;
	}
	pps_check_ta_ot_level(info);

	PPS_DBG("%s vbusov, vbatov = %d, %d\n", __func__, vbusov, vbatov);
	return true;

err:
	atomic_set(&dc_data->is_safety_err, 1);
	pps_end(info, true, hrst);
	return false;
}

static bool is_pps_rdy(struct rt_pps_algo_info *info)
{
	int ret = 0;
	uint8_t cap_idx = 0;
	bool is_pd_pe_rdy = false;
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;
	struct tcpm_power_cap_val *apdo_cap = &pps_data->apdo_cap;

	if (!pps_data->inited) {
		PPS_ERR("%s not init yet\n", __func__);
		return false;
	}

	if (pps_data->is_pps_once) {
		PPS_ERR("%s already check pps\n", __func__);
		return true;
	}

	if (!pps_data->is_pps_en_unlock) {
		PPS_ERR("%s pps is not unlock\n", __func__);
		return false;
	}

	if (tcpm_inquire_typec_attach_state(pps_data->ta_pps->tcpc)
		!= TYPEC_ATTACHED_SNK) {
		atomic_set(&dc_data->is_dc_once, 0);
		return false;
	}

	is_pd_pe_rdy = tcpm_inquire_pd_pe_ready(pps_data->ta_pps->tcpc);
	PPS_DBG("%s pe pd rdy = %d\n", __func__, is_pd_pe_rdy);
	if (!is_pd_pe_rdy)
		return false;

	/* select TA boundary */
	cap_idx = 0;
	pps_data->apdo_idx = -1;
	while (1) {
		ret = tcpm_inquire_pd_source_apdo(pps_data->ta_pps->tcpc,
			TCPM_POWER_CAP_APDO_TYPE_PPS, &cap_idx, apdo_cap);
		if (ret != TCPM_SUCCESS) {
			PPS_DBG("%s tcpm_inquire_pd_src_apdo fail(%d)\n",
				__func__, ret);
			break;
		}

		PPS_INFO("%s pps_boundary[%d], %d mv ~ %d mv, %d ma",
			__func__, cap_idx, apdo_cap->min_mv,
			apdo_cap->max_mv, apdo_cap->ma);

		if ((apdo_cap->min_mv <= dc_desc->ta_pps_cap_vmin) &&
			(apdo_cap->max_mv >= dc_desc->ta_pps_cap_vmax) &&
			(apdo_cap->ma >= dc_desc->ta_pps_cap_imin)) {
			pps_data->apdo_idx = cap_idx;
			pps_data->ta_boundary_min_volt = apdo_cap->min_mv;
			pps_data->ta_boundary_max_volt = apdo_cap->max_mv;
			pps_data->ta_boundary_max_cur = apdo_cap->ma;
			break;
		}
	}
	if (pps_data->apdo_idx == -1) {
		PPS_INFO("%s cannot find apdo for pps\n", __func__);
		return false;
	}

	pps_data->is_pps_once = true;
	return true;
}

static enum hrtimer_restart pps_dc_timer_callback(struct hrtimer *timer)
{
	struct rt_dc_data *dc_data = container_of(timer, struct rt_dc_data,
		timer);

	PPS_DBG("%s\n", __func__);
	pps_dc_wakeup_thread(dc_data);

	return HRTIMER_NORESTART;
}

static int pps_dc_thread_handler(void *data)
{
	struct timespec time;
	struct rt_pps_algo_info *info = data;
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;
	ktime_t ktime = ktime_set(0, MS_TO_NS(dc_desc->polling_interval));

	while (!kthread_should_stop()) {
		pm_stay_awake(info->dev);
		mutex_lock(&dc_data->lock);

		/* check charging time */
		get_monotonic_boottime(&dc_data->end_time);
		time = timespec_sub(dc_data->end_time,
			dc_data->start_time);
		if (time.tv_sec >= dc_desc->chg_time_max) {
			PPS_ERR("%s pps dc timeout(%d, %d)\n", __func__,
				(int)time.tv_sec, dc_desc->chg_time_max);
			pps_end(info, true, false);
		}

		switch (dc_data->state) {
		case DC_INIT:
			pps_dc_init(info);
			break;
		case DC_MEASURE_R:
			pps_dc_measure_r(info);
			break;
		case DC_SOFT_START:
			pps_dc_soft_start(info);
			break;
		case DC_CC:
			pps_dc_cc(info);
			break;
		case DC_STOP:
			PPS_DBG("%s DC_STOP\n", __func__);
			break;
		default:
			PPS_DBG("%s NO SUCH STATE\n", __func__);
			break;
		}

		if (dc_data->state != DC_STOP) {
			pps_dc_safety_check(info);
			if (dc_desc->polling_mode)
				hrtimer_start(&dc_data->timer, ktime,
					HRTIMER_MODE_REL);
		}
		mutex_unlock(&dc_data->lock);
		pm_relax(info->dev);
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}

	return 0;
}

static inline int __pps_plugout_reset(struct rt_pps_algo_info *info)
{
	int ret = 0;
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;

	PPS_DBG("%s\n", __func__);

	/* reset pps data */
	pps_data->is_pps_once = false;
	atomic_set(&dc_data->is_dc_once, 0);
	atomic_set(&dc_data->is_safety_err, 0);
	pps_data->hrst_cnt = 0;

	ret = pps_end(info, true, false);

	return ret;
}

static int pps_tcp_notifier_call(
	struct notifier_block *nb, unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct pps_device *pps_dev = pps_dev_get_by_name("algo_pps");
	struct rt_pps_algo_info *info = NULL;
	struct rt_dc_data *dc_data = NULL;

	if (!pps_dev)
		return NOTIFY_OK;

	info = pps_get_drvdata(pps_dev);
	if (!info)
		return NOTIFY_OK;
	dc_data = &info->dc_data;

	switch (event) {
	case TCP_NOTIFY_PD_STATE:
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			pr_info("%s pd detached\n", __func__);
			info->pps_data.is_pps_en_unlock = false;
			break;
		case PD_CONNECT_HARD_RESET:
			pr_info("%s pd hard reset, stop PPS\n", __func__);
			info->pps_data.is_pps_en_unlock = false;
			pps_end(info, false, false);
			info->pps_data.is_pps_once = false;
			atomic_set(&dc_data->is_dc_once, 0);
			atomic_set(&dc_data->is_safety_err, 0);
			break;
		case PD_CONNECT_PE_READY_SNK_APDO:
			if (info->pps_data.hrst_cnt < 5) {
				pr_info("%s PPS en unlock\n", __func__);
				info->pps_data.is_pps_en_unlock = true;
			}
			pr_info("%s pps hardreset count = %d\n", __func__,
				info->pps_data.hrst_cnt++);
			break;
		default:
			break;
		}
	default:
		break;
	}
	return NOTIFY_OK;
}

static int pps_chg_notifier_call(
	struct notifier_block *nb, unsigned long event, void *data)
{
	struct pps_device *pps_dev = pps_dev_get_by_name("algo_pps");
	struct rt_pps_algo_info *info = NULL;

	if (!pps_dev)
		return NOTIFY_OK;

	info = pps_get_drvdata(pps_dev);
	if (info->dc_desc.polling_mode)
		goto out;

	switch (event) {
	case CHG_NOTIFY_DC_WDT:
		pps_dc_wakeup_thread(&info->dc_data);
		break;
	default:
		break;
	}

out:
	return NOTIFY_OK;
}

/* =================================================================== */
/* PPS Algo OPS                                                        */
/* =================================================================== */

static int pps_init(struct pps_device *pps_dev)
{
	int ret = 0;
	struct rt_pps_algo_info *info = pps_get_drvdata(pps_dev);
	struct rt_pps_data *pps_data = &info->pps_data;

	PPS_INFO("%s\n", __func__);

	if (pps_data->inited) {
		PPS_INFO("%s already inited\n", __func__);
		return 0;
	}

	/* get ta & chg pps */
	pps_data->ta_pps = pps_dev_get_by_name("ta_pps");
	if (!pps_data->ta_pps) {
		PPS_ERR("%s get ta_pps fail\n", __func__);
		return -ENODEV;
	}

	pps_data->chg_pps = pps_dev_get_by_name("chg_pps");
	if (!pps_data->chg_pps) {
		PPS_ERR("%s get chg_pps fail\n", __func__);
		return -ENODEV;
	}

	/* register tcp notifier callback */
	pps_data->tcp_nb.notifier_call = pps_tcp_notifier_call;
	ret = register_tcp_dev_notifier(pps_data->ta_pps->tcpc,
		&pps_data->tcp_nb, TCP_NOTIFY_TYPE_USB);
	if (ret < 0) {
		PPS_ERR("%s register tcpc notifier fail\n", __func__);
		return ret;
	}

	/* register chg notifier callback */
	pps_data->chg_nb.notifier_call = pps_chg_notifier_call;
	ret = srcu_notifier_chain_register(
		&(pps_data->chg_pps->desc->evt_nh), &pps_data->chg_nb);
	if (ret < 0) {
		PPS_ERR("%s register chg notifier fail\n", __func__);
		return ret;
	}

	pps_data->inited = true;
	pps_data->hrst_cnt = 0;
	PPS_INFO("%s successfully\n", __func__);
	return 0;
}

static bool pps_is_dc_rdy(struct pps_device *pps_dev)
{
	int ret = 0;
	bool rdy = true;
	u32 soc = battery_get_soc();
	struct rt_pps_algo_info *info = pps_get_drvdata(pps_dev);
	struct rt_dc_data *dc_data = &info->dc_data;
	struct rt_dc_desc *dc_desc = &info->dc_desc;

	PPS_INFO("%s soc(%d)\n", __func__, soc);

	mutex_lock(&dc_data->lock);
	if (soc < dc_desc->pps_dc_start_battery_soc) {
		PPS_INFO("%s soc(%d) < pps_dc_start_battery_soc(%d)\n",
			 __func__, soc, dc_desc->pps_dc_start_battery_soc);
		rdy = false;
		goto out;
	}

	if (!is_pps_rdy(info)) {
		PPS_INFO("%s pps not rdy\n", __func__);
		rdy = false;
		goto out;
	}

	ret = pps_dc_start(info);
	if (ret < 0) {
		PPS_ERR("%s start dc fail\n", __func__);
		rdy = false;
		goto out;
	}

out:
	mutex_unlock(&dc_data->lock);
	return rdy;
}

static bool pps_is_dc_running(struct pps_device *pps_dev)
{
	struct rt_pps_algo_info *info = pps_get_drvdata(pps_dev);
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;
	bool running = false;

	if (!pps_data->inited)
		return false;

	mutex_lock(&dc_data->lock);
	running = (dc_data->state == DC_STOP) ? false : true;
	PPS_DBG("%s running = %d\n", __func__, running);
	mutex_unlock(&dc_data->lock);

	return running;
}

static int pps_plugout_reset(struct pps_device *pps_dev)
{
	int ret = 0;
	struct rt_pps_algo_info *info = pps_get_drvdata(pps_dev);
	struct rt_pps_data *pps_data = &info->pps_data;
	struct rt_dc_data *dc_data = &info->dc_data;

	/* no need to reset before inited */
	if (!pps_data->inited)
		return 0;

	PPS_DBG("%s\n", __func__);
	mutex_lock(&dc_data->lock);
	ret = __pps_plugout_reset(info);
	mutex_unlock(&dc_data->lock);

	return ret;
}

static int pps_stop_dc(struct pps_device *pps_dev)
{
	int ret = 0;
	struct rt_pps_algo_info *info = pps_get_drvdata(pps_dev);
	struct rt_dc_data *dc_data = &info->dc_data;

	mutex_lock(&dc_data->lock);
	ret = pps_end(info, true, false);
	mutex_unlock(&dc_data->lock);
	return ret;
}

static struct pps_algo_ops algo_ops = {
	.init = pps_init,
	.is_dc_rdy = pps_is_dc_rdy,
	.is_dc_running = pps_is_dc_running,
	.plugout_reset = pps_plugout_reset,
	.stop_dc = pps_stop_dc,
};

static int algo_pps_register(struct rt_pps_algo_info *info)
{
	dev_info(info->dev, "%s\n", __func__);

	info->desc.name = "algo_pps";
	info->desc.type = PPS_TYPE_ALGO;
	info->pps_dev = pps_device_register(info->dev, &info->desc, NULL,
		NULL, &algo_ops, info);
	if (IS_ERR_OR_NULL(info->pps_dev))
		return PTR_ERR(info->pps_dev);

	return 0;
}

static int pps_parse_dt(struct rt_pps_algo_info *info)
{
	struct rt_dc_desc *desc = &info->dc_desc;
	struct device_node *np = NULL;
	u32 tmp = 0;

	memcpy(desc, &default_dc_desc, sizeof(struct rt_dc_desc));

	np = of_find_node_by_name(NULL, "rt_pps");
	if (!np) {
		PPS_ERR("%s find node rt_pps fail\n", __func__);
		return -ENODEV;
	}

	desc->polling_mode = of_property_read_bool(np, "polling_mode");
	desc->need_bif = of_property_read_bool(np, "need_bif");

	if (of_property_read_u32(np, "polling_interval", &tmp) >= 0)
		desc->polling_interval = tmp;

	if (of_property_read_u32(np, "cv_limit", &tmp) >= 0)
		desc->cv_limit = tmp;

	if (of_property_read_u32(np, "bat_upper_bound", &tmp) >= 0)
		desc->bat_upper_bound = tmp;

	if (of_property_read_u32(np, "pps_dc_start_battery_soc", &tmp) >= 0)
		desc->pps_dc_start_battery_soc = tmp;

	if (of_property_read_u32(np, "cc_ss_init", &tmp) >= 0)
		desc->cc_ss_init = tmp;

	if (of_property_read_u32(np, "cc_init", &tmp) >= 0)
		desc->cc_init = tmp;

	if (of_property_read_u32(np, "cc_init_bad_cable1", &tmp) >= 0)
		desc->cc_init_bad_cable1 = tmp;

	if (of_property_read_u32(np, "cc_init_bad_cable2", &tmp) >= 0)
		desc->cc_init_bad_cable2 = tmp;

	if (of_property_read_u32(np, "cc_init_r", &tmp) >= 0)
		desc->cc_init_r = tmp;

	if (of_property_read_u32(np, "cc_init_bad_cable1_r", &tmp) >= 0)
		desc->cc_init_bad_cable1_r = tmp;

	if (of_property_read_u32(np, "cc_init_bad_cable2_r", &tmp) >= 0)
		desc->cc_init_bad_cable2_r = tmp;

	if (of_property_read_u32(np, "cc_normal", &tmp) >= 0)
		desc->cc_normal = tmp;

	if (of_property_read_u32(np, "cc_max", &tmp) >= 0)
		desc->cc_max = tmp;

	if (of_property_read_u32(np, "cc_end", &tmp) >= 0)
		desc->cc_end = tmp;

	if (of_property_read_u32(np, "cc_step", &tmp) >= 0)
		desc->cc_step = tmp;

	if (of_property_read_u32(np, "cc_ss_step", &tmp) >= 0)
		desc->cc_ss_step = tmp;

	if (of_property_read_u32(np, "cc_ss_step1", &tmp) >= 0)
		desc->cc_ss_step1 = tmp;

	if (of_property_read_u32(np, "cc_ss_step2", &tmp) >= 0)
		desc->cc_ss_step2 = tmp;

	if (of_property_read_u32(np, "cv_ss_step1", &tmp) >= 0)
		desc->cv_ss_step1 = tmp;

	if (of_property_read_u32(np, "cv_ss_step2", &tmp) >= 0)
		desc->cv_ss_step2 = tmp;

	if (of_property_read_u32(np, "cc_ss_blanking", &tmp) >= 0)
		desc->cc_ss_blanking = tmp;

	if (of_property_read_u32(np, "cc_blanking", &tmp) >= 0)
		desc->cc_blanking = tmp;

	if (of_property_read_u32(np, "charger_temp_max", &tmp) >= 0)
		desc->charger_temp_max = tmp;

	if (of_property_read_u32(np, "ta_warn_level1_temp", &tmp) >= 0)
		desc->ta_warn_level1_temp = tmp;

	if (of_property_read_u32(np, "ta_warn_level2_temp", &tmp) >= 0)
		desc->ta_warn_level2_temp = tmp;

	if (of_property_read_u32(np, "ta_warn_level3_temp", &tmp) >= 0)
		desc->ta_warn_level3_temp = tmp;

	if (of_property_read_u32(np, "ta_temp_max", &tmp) >= 0)
		desc->ta_temp_max = tmp;

	if (of_property_read_u32(np, "vbus_ov_gap", &tmp) >= 0)
		desc->vbus_ov_gap = tmp;

	if (of_property_read_u32(np, "fod_current", &tmp) >= 0)
		desc->fod_current = tmp;

	if (of_property_read_u32(np, "r_vbat_min", &tmp) >= 0)
		desc->r_vbat_min = tmp;

	if (of_property_read_u32(np, "r_sw_min", &tmp) >= 0)
		desc->r_sw_min = tmp;

	if (of_property_read_u32(np, "bat_temp_min", &tmp) >= 0)
		desc->bat_temp_min = tmp;

	if (of_property_read_u32(np, "bat_warn_level1_temp", &tmp) >= 0)
		desc->bat_warn_level1_temp = tmp;

	if (of_property_read_u32(np, "bat_warn_level2_temp", &tmp) >= 0)
		desc->bat_warn_level2_temp = tmp;

	if (of_property_read_u32(np, "bat_warn_level3_temp", &tmp) >= 0)
		desc->bat_warn_level3_temp = tmp;

	if (of_property_read_u32(np, "bat_temp_max", &tmp) >= 0)
		desc->bat_temp_max = tmp;

	if (of_property_read_u32(np, "curr_reco_temp_region", &tmp) >= 0)
		desc->curr_reco_temp_region = tmp;

	if (of_property_read_u32(np, "ta_pps_cap_vmin", &tmp) >= 0)
		desc->ta_pps_cap_vmin = tmp;

	if (of_property_read_u32(np, "ta_pps_cap_vmax", &tmp) >= 0)
		desc->ta_pps_cap_vmax = tmp;

	if (of_property_read_u32(np, "ta_pps_cap_imin", &tmp) >= 0)
		desc->ta_pps_cap_imin = tmp;

	if (of_property_read_u32(np, "chg_time_max", &tmp) >= 0)
		desc->chg_time_max = tmp;

	return 0;
}

static int rt_pps_algo_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct rt_pps_algo_info *info;
	struct rt_dc_data *dc_data;

	dev_info(&pdev->dev, "%s(%s)\n", __func__, RT_PPS_ALGO_VERSION);

	info = devm_kzalloc(&pdev->dev, sizeof(struct rt_pps_algo_info),
		GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	dc_data = &info->dc_data;
	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

	ret = pps_parse_dt(info);
	if (ret < 0)
		dev_err(info->dev, "%s parse dt fail, use default value\n",
			__func__);

	ret = algo_pps_register(info);
	if (ret < 0) {
		dev_err(info->dev, "%s register pps algo fail(%d)\n",
			__func__, ret);
		goto err_pps_register;
	}

	/* init dc thread & timer */
	mutex_init(&dc_data->lock);
	atomic_set(&dc_data->is_dc_once, 0);
	atomic_set(&dc_data->is_safety_err, 0);
	dc_data->state = DC_STOP;
	init_waitqueue_head(&dc_data->wait_queue);
	hrtimer_init(&dc_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dc_data->timer.function = pps_dc_timer_callback;
	dc_data->thread = kthread_create(pps_dc_thread_handler, info,
		"dc_thread");
	if (IS_ERR(dc_data->thread)) {
		PPS_ERR("%s run dc thread fail\n", __func__);
		ret = PTR_ERR(dc_data->thread);
		goto err_run_dc_thread;
	}

	device_init_wakeup(info->dev, true);
	dev_info(info->dev, "%s successfully\n", __func__);
	return 0;

err_run_dc_thread:
	pps_device_unregister(info->pps_dev);
err_pps_register:
	return ret;
}

static int rt_pps_algo_remove(struct platform_device *pdev)
{
	struct rt_pps_algo_info *info = platform_get_drvdata(pdev);
	struct rt_dc_data *dc_data;

	if (info) {
		dc_data = &info->dc_data;
		kthread_stop(dc_data->thread);
		mutex_destroy(&dc_data->lock);
		pps_device_unregister(info->pps_dev);
	}

	return 0;
}

static struct platform_device rt_pps_algo_dev = {
	.name = "rt_pps_algo_dev",
	.id = -1,
};

static struct platform_driver rt_pps_algo_driver = {
	.probe = rt_pps_algo_probe,
	.remove = rt_pps_algo_remove,
	.driver = {
		.name = "rt_pps_algo_dev",
		.owner = THIS_MODULE,
	},
};

static int __init rt_pps_algo_init(void)
{
	platform_device_register(&rt_pps_algo_dev);
	return platform_driver_register(&rt_pps_algo_driver);
}

static void __exit rt_pps_algo_exit(void)
{
	platform_driver_unregister(&rt_pps_algo_driver);
	platform_device_unregister(&rt_pps_algo_dev);
}
device_initcall_sync(rt_pps_algo_init);
module_exit(rt_pps_algo_exit);

MODULE_DESCRIPTION("PPS Algorithm");
MODULE_AUTHOR("ShuFanLee <shufan_lee@richtek.com>");
MODULE_VERSION(RT_PPS_ALGO_VERSION);
MODULE_LICENSE("GPL");

/*
 * Revision Note
 * 1.0.10
 * (1) Revise BAT/TA OTP flow
 * (2) Remove ibat_sign
 * (3) Wake up charger routine thread at the end of pps_end()
 *
 * 1.0.9
 * (1) Support 6A charging current
 * (2) Add TA OTP
 *
 * 1.0.8
 * (1) Add PPS DC retry
 * (2) Smooth charging current in CV mode
 * (3) Add TBAT OTP
 *
 * 1.0.7
 * (1) Modify flow of measure R
 *
 * 1.0.6
 * (1) merge with PD3.0
 *
 * 1.0.5
 * (1) Add wake lock
 *
 * 1.0.4
 * (1) Do not use dpm command in tcp notifer handler
 * (2) Refine measure_r function
 *
 * 1.0.3
 * (1) Modify the way to wake up polling thread
 * (2) Add polling interval in dc desc
 *
 * 1.0.2
 * (1) Add prefix for pps class API to rt_xxx
 *
 * 1.0.1
 * (1) Add algorithm pps
 *
 * 1.0.0
 * (1) Initial release
 *
 */
