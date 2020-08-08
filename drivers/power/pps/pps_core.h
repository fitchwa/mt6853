/*
 * Copyright (C) 2017 Richtek Technology Corp.
 *
 * Richtek PPS Definition
 *
 * Author: WS <weisin_lin@richtek.com>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_RT_PPS_H
#define __LINUX_RT_PPS_H

#include <linux/types.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>

#define PPS_DBG(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#define PPS_INFO(fmt, ...) pr_info(fmt, ##__VA_ARGS__)
#define PPS_ERR(fmt, ...) pr_err(fmt, ##__VA_ARGS__)

enum pps_dc_state {
	DC_INIT = 0,
	DC_MEASURE_R,
	DC_SOFT_START,
	DC_CC,
	DC_STOP,
};

struct rt_dc_desc {
	bool polling_mode;
	u32 polling_interval;		/* polling interval */
	u32 cv_limit;			/* vbus upper bound */
	u32 bat_upper_bound;		/* vbat upper bound */
	u32 pps_dc_start_battery_soc;	/* pps_dc start bat low bound */
	u32 cc_ss_init;			/* init charging current */
	u32 cc_init;			/* max charging current */
	u32 cc_init_bad_cable0;		/* charging current for bad cable0 */
	u32 cc_init_bad_cable1;		/* charging current for bad cable1 */
	u32 cc_init_bad_cable2;		/* charging current for bad cable2 */
	u32 cc_init_r;			/* good cable max impedance */
	u32 cc_init_bad_cable0_r;	/* bad cable1 max impedance */
	u32 cc_init_bad_cable1_r;	/* bad cable1 max impedance */
	u32 cc_init_bad_cable2_r;	/* bad cable2 max impedance */
	u32 cc_normal;			/* normal charging ibusoc threshold */
	u32 cc_max;			/* DC ibusoc threshold */
	u32 cc_end;			/* DC terminated charging current */
	u32 cc_step;			/* CC state charging current step */
	u32 cc_ss_step;			/* SS state charging current step */
	u32 cc_ss_step1;		/* SS state charging current step2 */
	u32 cc_ss_step2;		/* SS state charging current step3 */
	u32 cv_ss_step1;		/* vbat threshold for cc_ss_step2 */
	u32 cv_ss_step2;		/* vbat threshold for cc_ss_step3 */
	u32 cc_ss_blanking;		/* polling duration for init/SS state */
	u32 cc_blanking;		/* polling duration for CC state */
	int charger_temp_max;		/* max charger temperature */
	int ta_warn_level1_temp;
	int ta_warn_level2_temp;
	int ta_warn_level3_temp;
	int ta_temp_max;		/* max adapter temperature */
	u32 vbus_ov_gap;
	u32 fod_current;		/* FOD current threshold */
	u32 r_vbat_min;			/* min r_vbat */
	u32 r_sw_min;			/* min r_sw */
	int bat_temp_min;		/* min battery temperature */
	int bat_warn_level1_temp;
	int bat_warn_level2_temp;
	int bat_warn_level3_temp;
	int bat_temp_max;		/* max battery temperature */
	int curr_reco_temp_region;	/* the region of tbat cool down and increase curr */
	u32 ta_pps_cap_vmin;		/* min pps voltage capability */
	u32 ta_pps_cap_vmax;		/* max pps voltage capability */
	u32 ta_pps_cap_imin;		/* min current to enter dc */
	u32 chg_time_max;		/* max charging time */
	bool need_bif;			/* is bif needed */
};

struct rt_dc_data {
	wait_queue_head_t wait_queue;
	struct hrtimer timer;
	struct task_struct *thread;
	struct mutex lock;
	bool is_bif_exist;
	bool is_vbat_over_cv;
	atomic_t is_dc_once;
	atomic_t is_safety_err;

	/* calibration */
	int vbus_cali;
	int r_sw;
	int r_cable;
	int r_vbat;
	int r_total;
	int chg_cur_lmt;
	int chg_cur_lmt_by_r;
	int cv_lower_bound;
	int need_add_curr_cnt;

	int tbat;
	int zcv;
	int bat_ot_level;
	int ta_ot_level;

	struct timespec start_time;
	struct timespec end_time;

	enum pps_dc_state state;
};

struct rt_pps_data {
	struct pps_device *ta_pps;
	struct pps_device *chg_pps;
	struct notifier_block tcp_nb;
	struct notifier_block chg_nb;
	bool is_pps_once;

	uint8_t typec_state;
	uint8_t is_pps_en_unlock;

	int ta_boundary_min_volt;
	int ta_boundary_max_volt;
	int ta_boundary_max_cur;
	int ta_setting_volt;
	int ta_setting_cur;
	int ta_current_volt;
	int ta_current_cur;
	int ta_current_temp;
	int apdo_idx;
	struct tcpm_power_cap_val apdo_cap;
	uint8_t inited:1;

	int hrst_cnt;
};

#endif /* __LINUX_RT_PPS_H */
