/*
 * Copyright (C) 2017 Richtek Technology Corp.
 *
 * Richtek PPS Charger Interface
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

#include "pps_class.h"

int rt_pps_enable_power_path(struct pps_device *pps, bool en)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->enable_power_path)
		return -ENOTSUPP;
	return pps->chg_ops->enable_power_path(pps, en);
}
EXPORT_SYMBOL(rt_pps_enable_power_path);

int rt_pps_enable_charger(struct pps_device *pps, bool en)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->enable_charger)
		return -ENOTSUPP;
	return pps->chg_ops->enable_charger(pps, en);
}
EXPORT_SYMBOL(rt_pps_enable_charger);

int rt_pps_get_tchg(struct pps_device *pps, int *min, int *max)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->get_tchg)
		return -ENOTSUPP;
	return pps->chg_ops->get_tchg(pps, min, max);
}
EXPORT_SYMBOL(rt_pps_get_tchg);

int rt_pps_set_dirchg_ibusoc(struct pps_device *pps, u32 mA)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->set_dirchg_ibusoc)
		return -ENOTSUPP;
	return pps->chg_ops->set_dirchg_ibusoc(pps, mA);
}
EXPORT_SYMBOL(rt_pps_set_dirchg_ibusoc);

int rt_pps_enable_dirchg(struct pps_device *pps, bool en)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->enable_dirchg)
		return -ENOTSUPP;
	return pps->chg_ops->enable_dirchg(pps, en);
}
EXPORT_SYMBOL(rt_pps_enable_dirchg);

int rt_pps_enable_dirchg_chip(struct pps_device *pps, bool en)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->enable_dirchg_chip)
		return -ENOTSUPP;
	return pps->chg_ops->enable_dirchg_chip(pps, en);
}
EXPORT_SYMBOL(rt_pps_enable_dirchg_chip);

int rt_pps_enable_dirchg_vbatov(struct pps_device *pps, bool en)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->enable_dirchg_vbatov)
		return -ENOTSUPP;
	return pps->chg_ops->enable_dirchg_vbatov(pps, en);
}
EXPORT_SYMBOL(rt_pps_enable_dirchg_vbatov);

int rt_pps_get_vbat(struct pps_device *pps, u32 *mV)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->get_vbat)
		return -ENOTSUPP;
	return pps->chg_ops->get_vbat(pps, mV);
}
EXPORT_SYMBOL(rt_pps_get_vbat);

int rt_pps_get_ibat(struct pps_device *pps, s32 *mA)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->get_ibat)
		return -ENOTSUPP;
	return pps->chg_ops->get_ibat(pps, mA);
}
EXPORT_SYMBOL(rt_pps_get_ibat);

int rt_pps_get_tbat(struct pps_device *pps, int *tbat)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->get_tbat)
		return -ENOTSUPP;
	return pps->chg_ops->get_tbat(pps, tbat);
}
EXPORT_SYMBOL(rt_pps_get_tbat);

int rt_pps_get_vbus(struct pps_device *pps, u32 *mV)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->get_vbus)
		return -ENOTSUPP;
	return pps->chg_ops->get_vbus(pps, mV);
}
EXPORT_SYMBOL(rt_pps_get_vbus);

int rt_pps_get_ibus(struct pps_device *pps, u32 *mA)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->get_ibus)
		return -ENOTSUPP;
	return pps->chg_ops->get_ibus(pps, mA);
}
EXPORT_SYMBOL(rt_pps_get_ibus);

bool rt_pps_is_bif_exist(struct pps_device *pps)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->is_bif_exist)
		return -ENOTSUPP;
	return pps->chg_ops->is_bif_exist(pps);
}
EXPORT_SYMBOL(rt_pps_is_bif_exist);

int rt_pps_get_bif_vbat(struct pps_device *pps, u32 *mV)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->get_bif_vbat)
		return -ENOTSUPP;
	return pps->chg_ops->get_bif_vbat(pps, mV);
}
EXPORT_SYMBOL(rt_pps_get_bif_vbat);

int rt_pps_get_bif_tbat(struct pps_device *pps, int *tbat)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->get_bif_tbat)
		return -ENOTSUPP;
	return pps->chg_ops->get_bif_tbat(pps, tbat);
}
EXPORT_SYMBOL(rt_pps_get_bif_tbat);

int rt_pps_wake_up_charger(struct pps_device *pps)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_CHARGER);
	if (!pps->chg_ops->wake_up_charger)
		return -ENOTSUPP;
	pps->chg_ops->wake_up_charger(pps);
	return 0;
}
EXPORT_SYMBOL(rt_pps_wake_up_charger);
