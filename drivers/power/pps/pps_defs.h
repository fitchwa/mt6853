#ifndef __LINUX_RT_PPS_DEFS_H
#define __LINUX_RT_PPS_DEFS_H

#include <tcpm.h>
#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/hrtimer.h>

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

enum pps_chg_mode {
	CHG_MODE_UNKNOWN = 0,
	CHG_MODE_IDLE,
	CHG_MODE_TRICKLE,
	CHG_MODE_PRECHG,
	CHG_MODE_NORMAL,
	CHG_MODE_CV,
	CHG_MODE_CHGDONE,
	CHG_MODE_FAULT,
	CHG_MODE_DC_FAULT,
};

enum pps_chg_fault_evt {
	CHG_EVT_UNKNOWN = 0,
	CHG_EVT_VBUSOV,
	CHG_EVT_VBATOV,
	CHG_EVT_VSYSOV,
	CHG_EVT_VSYSUV,
	CHG_EVT_DC_VBATOV,
	CHG_EVT_DC_IBUSOC,
	CHG_EVT_DC_IBUSUC,
	CHG_EVT_DC_VBUSOV,
};

struct rt_dc_data {
	wait_queue_head_t wait_queue;
	struct hrtimer timer;
	struct task_struct *thread;
	bool wakeup_thread;
	bool polling_mode;
	bool is_dc_once;

	/* calibration */
	int vbus_cali;
	int r_vbat;
	int r_total;
	int r_cable;
	int r_cable_1;
	int r_cable_2;
	int r_sw;
	int chg_cur_lmt;
	int chg_cur_lmt_by_r;

	int vbat;
	int ibat;
	int tbat;

	struct timespec start_time;
	struct timespec end_time;

	enum pps_dc_state state;
	enum pps_chg_mode chg_mode;
	enum pps_chg_fault_evt chg_fault_evt;

	u32 cv_limit;			/* vbus upper bound */
	u32 bat_upper_bound;		/* vbat upper bound */
	u32 bat_lower_bound;		/* vbat low bound */
	u32 cc_ss_init;			/* init charging current */
	u32 cc_init;			/* max charging current */
	u32 cc_init_bad_cable1;		/* charging current for bad cable1 */
	u32 cc_init_bad_cable2;		/* charging current for bad cable2 */
	u32 cc_init_r;			/* good cable max impedance */
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
	int ta_temp_max;		/* max adapter temperature */
	u32 vbus_ov_gap;
	u32 fod_current;		/* FOD current threshold */
	u32 r_vbat_min;			/* min r_vbat */
	u32 r_sw_min;			/* min r_sw */
	int bat_temp_min;		/* min battery temperature */
	int bat_temp_max;		/* max battery temperature */
	u32 ta_pps_cap_vmin;		/* min pps voltage capability */
	u32 ta_pps_cap_vmax;		/* max pps voltage capability */
	u32 chg_time_max;		/* max charging time */
};

struct rt_pps_data {
	struct pps_device *ta_pps;
	struct pps_device *chg_pps;
	struct notifier_block tcp_nb;
	struct notifier_block chg_nb;
	struct rt_dc_data dc_data;
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
	struct tcpm_status_data status;
	struct tcpm_pps_status pps_status;
	uint8_t inited:1;
};

#endif /* __LINUX_RT_PPS_DEFS_H */
