/*
 * Copyright (C) 2017 Richtek Technology Corp.
 *
 * Richtek TypeC Port Control Interface Core Driver
 *
 * Author: TH <tsunghan_tsai@richtek.com>
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

int rt_pps_enable_ta_direct_charge(
		struct pps_device *pps, bool en, int mv, int ma)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_TA);
	return pps->ta_ops->enable_direct_charge(pps, en, mv, ma);
}
EXPORT_SYMBOL(rt_pps_enable_ta_direct_charge);

int rt_pps_set_ta_cap(struct pps_device *pps, int mv, int ma)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_TA);
	return pps->ta_ops->set_cap(pps, mv, ma);
}
EXPORT_SYMBOL(rt_pps_set_ta_cap);

int rt_pps_get_ta_current_cap(struct pps_device *pps, int *mv, int *ma)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_TA);
	return pps->ta_ops->get_current_cap(pps, mv, ma);
}
EXPORT_SYMBOL(rt_pps_get_ta_current_cap);

int rt_pps_get_ta_temperature(struct pps_device *pps, int *temp)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_TA);
	return pps->ta_ops->get_temperature(pps, temp);
}
EXPORT_SYMBOL(rt_pps_get_ta_temperature);

int rt_pps_get_ta_status(
	struct pps_device *pps, struct pps_ta_status *ta_status)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_TA);
	return pps->ta_ops->get_status(pps, ta_status);
}
EXPORT_SYMBOL(rt_pps_get_ta_status);

int rt_pps_send_ta_hard_reset(struct pps_device *pps)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_TA);
	return pps->ta_ops->send_hardreset(pps);
}
EXPORT_SYMBOL(rt_pps_send_ta_hard_reset);
