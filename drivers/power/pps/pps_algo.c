/*
 * Copyright (C) 2017 Richtek Technology Corp.
 *
 * Richtek PPS Algorithm Interface
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

int rt_pps_init(struct pps_device *pps)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_ALGO);
	if (!pps->algo_ops->init)
		return -ENOTSUPP;
	return pps->algo_ops->init(pps);
}
EXPORT_SYMBOL(rt_pps_init);

bool rt_pps_is_dc_rdy(struct pps_device *pps)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_ALGO);
	if (!pps->algo_ops->is_dc_rdy)
		return false;
	return pps->algo_ops->is_dc_rdy(pps);
}
EXPORT_SYMBOL(rt_pps_is_dc_rdy);

bool rt_pps_is_dc_running(struct pps_device *pps)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_ALGO);
	if (!pps->algo_ops->is_dc_running)
		return false;
	return pps->algo_ops->is_dc_running(pps);
}
EXPORT_SYMBOL(rt_pps_is_dc_running);

int rt_pps_plugout_reset(struct pps_device *pps)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_ALGO);
	if (!pps->algo_ops->plugout_reset)
		return -ENOTSUPP;
	return pps->algo_ops->plugout_reset(pps);
}
EXPORT_SYMBOL(rt_pps_plugout_reset);

int rt_pps_stop_dc(struct pps_device *pps)
{
	PPS_CHECK_TYPE(pps, PPS_TYPE_ALGO);
	if (!pps->algo_ops->stop_dc)
		return -ENOTSUPP;
	return pps->algo_ops->stop_dc(pps);
}
EXPORT_SYMBOL(rt_pps_stop_dc);
