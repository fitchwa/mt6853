/*
 * Copyright (C) 2017 Richtek Technology Corp.
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

#ifndef __LINUX_RT_PPS_CORE_H
#define __LINUX_RT_PPS_CORE_H

#include <linux/device.h>
#include <linux/notifier.h>

#define PPS_CHECK_TYPE(pps_dev, type) \
	do { \
		if (!pps_dev) \
			return -ENODEV; \
		if (pps_get_type(pps_dev) != type) { \
			pr_err("%s type error\n", __func__); \
			return -EINVAL; \
		} \
	} while (0) \

struct pps_ta_status {
	uint8_t temp;
	uint8_t present_input;
	uint8_t present_battery_input;
	uint8_t ocp:1;
	uint8_t otp:1;
	uint8_t ovp:1;
	uint8_t temp_level;
};

struct pps_device;

struct pps_ta_ops {
	int (*enable_direct_charge)(struct pps_device *pps,
					bool en, int mv, int ma);
	int (*set_cap)(struct pps_device *pps, int mv, int ma);
	int (*get_current_cap)(struct pps_device *pps, int *mv, int *ma);
	int (*get_temperature)(struct pps_device *pps, int *temp);
	int (*get_status)(struct pps_device *pps,
				struct pps_ta_status *ta_status);
	int (*send_hardreset)(struct pps_device *pps);
};

struct pps_chg_ops {
	int (*enable_power_path)(struct pps_device *pps, bool en);
	int (*enable_charger)(struct pps_device *pps, bool en);
	int (*enable_dirchg)(struct pps_device *pps, bool en);
	int (*enable_dirchg_vbatov)(struct pps_device *pps, bool en);
	int (*enable_dirchg_chip)(struct pps_device *pps, bool en);
	int (*set_dirchg_ibusoc)(struct pps_device *pps, u32 mA);
	int (*get_tchg)(struct pps_device *pps, int *min, int *max);
	int (*get_vbat)(struct pps_device *pps, u32 *mV);
	int (*get_ibat)(struct pps_device *pps, s32 *mA);
	int (*get_tbat)(struct pps_device *pps, int *tbat);
	int (*get_vbus)(struct pps_device *pps, u32 *mV);
	int (*get_ibus)(struct pps_device *pps, u32 *mA);
	bool (*is_bif_exist)(struct pps_device *pps);
	int (*get_bif_vbat)(struct pps_device *pps, u32 *mV);
	int (*get_bif_tbat)(struct pps_device *pps, int *tbat);
	void (*wake_up_charger)(struct pps_device *pps);
};

struct pps_algo_ops {
	int (*init)(struct pps_device *pps);
	bool (*is_dc_rdy)(struct pps_device *pps);
	bool (*is_dc_running)(struct pps_device *pps);
	int (*plugout_reset)(struct pps_device *pps);
	int (*stop_dc)(struct pps_device *pps);
};

enum {
	PPS_TYPE_TA = 0,
	PPS_TYPE_CHARGER,
	PPS_TYPE_ALGO,
};

enum pps_notify {
	CHG_NOTIFY_DC_WDT = 0,
};

struct pps_desc {
	const char *name;
	uint8_t type;
	struct srcu_notifier_head evt_nh;
};

struct pps_device {
	struct device dev;
	struct pps_ta_ops *ta_ops;
	struct pps_chg_ops *chg_ops;
	struct pps_algo_ops *algo_ops;
	struct pps_desc *desc;
	struct tcpc_device *tcpc;
	void *drv_data;
};

extern struct pps_device *pps_device_register(struct device *parent,
	struct pps_desc *pps_desc, struct pps_ta_ops *ta_ops,
	struct pps_chg_ops *chg_ops, struct pps_algo_ops *algo_ops,
	void *drv_data);
extern void pps_device_unregister(struct pps_device *pps);
extern struct pps_device *pps_dev_get_by_name(const char *name);
static inline int pps_get_type(struct pps_device *pps)
{
	return pps->desc->type;
}

static inline void *pps_get_drvdata(struct pps_device *pps)
{
	return pps->drv_data;
}

/* Richtek PPS TA interface */
extern int rt_pps_enable_ta_direct_charge(
		struct pps_device *pps, bool en, int mv, int ma);
extern int rt_pps_set_ta_cap(struct pps_device *pps, int mv, int ma);
extern int rt_pps_get_ta_current_cap(struct pps_device *pps, int *mv, int *ma);
extern int rt_pps_get_ta_temperature(struct pps_device *pps, int *temp);
extern int rt_pps_get_ta_status(
	struct pps_device *pps, struct pps_ta_status *ta_status);
extern int rt_pps_send_ta_hard_reset(struct pps_device *pps);

/* Richtek PPS charger interface */
extern int rt_pps_enable_power_path(struct pps_device *pps, bool en);
extern int rt_pps_enable_charger(struct pps_device *pps, bool en);
extern int rt_pps_enable_dirchg(struct pps_device *pps, bool en);
extern int rt_pps_enable_dirchg_vbatov(struct pps_device *pps, bool en);
extern int rt_pps_enable_dirchg_chip(struct pps_device *pps, bool en);
extern int rt_pps_set_dirchg_ibusoc(struct pps_device *pps, u32 mA);
extern int rt_pps_get_tchg(struct pps_device *pps, int *min, int *max);
extern int rt_pps_get_vbat(struct pps_device *pps, u32 *mV);
extern int rt_pps_get_ibat(struct pps_device *pps, s32 *mA);
extern int rt_pps_get_tbat(struct pps_device *pps, int *tbat);
extern int rt_pps_get_vbus(struct pps_device *pps, u32 *mV);
extern int rt_pps_get_ibus(struct pps_device *pps, u32 *mA);
extern bool rt_pps_is_bif_exist(struct pps_device *pps);
extern int rt_pps_get_bif_vbat(struct pps_device *pps, u32 *mV);
extern int rt_pps_get_bif_tbat(struct pps_device *pps, int *tbat);
extern int rt_pps_wake_up_charger(struct pps_device *pps);

/* Richtek PPS algorithm interface */
#ifdef CONFIG_RICHTEK_PPS_DC
extern int rt_pps_init(struct pps_device *pps);
extern bool rt_pps_is_dc_rdy(struct pps_device *pps);
extern bool rt_pps_is_dc_running(struct pps_device *pps);
extern int rt_pps_plugout_reset(struct pps_device *pps);
extern int rt_pps_start_dc(struct pps_device *pps);
extern int rt_pps_stop_dc(struct pps_device *pps);
#else
static inline int rt_pps_init(struct pps_device *pps)
{
	return -ENOTSUPP;
}

static inline bool rt_pps_is_dc_rdy(struct pps_device *pps)
{
	return false;
}

static inline bool rt_pps_is_dc_running(struct pps_device *pps)
{
	return false;
}

static inline int rt_pps_plugout_reset(struct pps_device *pps)
{
	return -ENOTSUPP;
}

static inline int rt_pps_stop_dc(struct pps_device *pps)
{
	return -ENOTSUPP;
}
#endif /* CONFIG_RICHTEK_PPS_DC */

#endif /* __LINUX_RT_PPS_CORE_H */
