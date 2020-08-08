/*
 * Copyright (C) 2016 Richtek Technology Corp.
 *
 * Richtek PPS Class Driver
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
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/list.h>

#include "pps_class.h"

#define PPS_CLASS_VERSION	"1.0.2_G"

#define to_pps_device(obj)	container_of(obj, struct pps_device, dev)

static struct class *pps_class;

static struct device_type pps_dev_type;

static ssize_t pps_show_property(struct device *dev,
				  struct device_attribute *attr, char *buf);
static ssize_t pps_store_property(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

#define PPS_DEVICE_ATTR(_name, _mode)					\
{									\
	.attr = { .name = #_name, .mode = _mode },			\
	.show = pps_show_property,					\
	.store = pps_store_property,					\
}

static struct device_attribute pps_device_attributes[] = {
	PPS_DEVICE_ATTR(name, S_IRUGO),
	PPS_DEVICE_ATTR(type, S_IRUGO),
};

enum {
	PPS_DESC_NAME,
	PPS_DESC_TYPE,
};

static struct attribute *__pps_attrs[ARRAY_SIZE(pps_device_attributes) + 1];
static struct attribute_group pps_attr_group = {
	.attrs = __pps_attrs,
};

static const struct attribute_group *pps_attr_groups[] = {
	&pps_attr_group,
	NULL,
};

static ssize_t pps_show_property(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct pps_device *pps = to_pps_device(dev);
	const ptrdiff_t offset = attr - pps_device_attributes;

	dev_info(dev, "%s\n", __func__);
	switch (offset) {
	case PPS_DESC_NAME:
		snprintf(buf+strlen(buf), PAGE_SIZE, "%s\n", pps->desc->name);
		break;
	case PPS_DESC_TYPE:
		switch (pps->desc->type) {
		case PPS_TYPE_TA:
			snprintf(buf+strlen(buf), PAGE_SIZE, "Ta\n");
			break;
		case PPS_TYPE_CHARGER:
			snprintf(buf+strlen(buf), PAGE_SIZE, "Charger\n");
			break;
		}
		break;
	default:
		break;
	}
	return strlen(buf);
}

static ssize_t pps_store_property(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	/*struct pps_device *pps = to_pps_device(dev);*/
	const ptrdiff_t offset = attr - pps_device_attributes;

	dev_info(dev, "%s\n", __func__);
	switch (offset) {
	default:
		break;
	}
	return count;
}

static void pps_device_release(struct device *dev)
{
	struct pps_device *pps = to_pps_device(dev);

	dev_info(dev, "device release\n");
	devm_kfree(dev, pps);
}

struct pps_device *pps_device_register(struct device *parent,
	struct pps_desc *pps_desc, struct pps_ta_ops *ta_ops,
	struct pps_chg_ops *chg_ops, struct pps_algo_ops *algo_ops,
	void *drv_data)
{
	struct pps_device *pps = NULL;
	int ret = 0;

	dev_info(parent, "register pps device (%s)\n", pps_desc->name);
	pps = devm_kzalloc(parent, sizeof(*pps), GFP_KERNEL);
	if (!pps)
		return ERR_PTR(-ENOMEM);

	pps->dev.class = pps_class;
	pps->dev.type = &pps_dev_type;
	pps->dev.parent = parent;
	pps->dev.release = pps_device_release;
	dev_set_drvdata(&pps->dev, pps);
	pps->drv_data = drv_data;
	dev_set_name(&pps->dev, pps_desc->name);
	pps->desc = pps_desc;
	pps->ta_ops = ta_ops;
	pps->chg_ops = chg_ops;
	pps->algo_ops = algo_ops;
	srcu_init_notifier_head(&pps_desc->evt_nh);

	ret = device_register(&pps->dev);
	if (ret) {
		kfree(pps);
		return ERR_PTR(ret);
	}

	dev_info(parent, "register pps device done\n");
	return pps;
}
EXPORT_SYMBOL(pps_device_register);

void pps_device_unregister(struct pps_device *pps)
{
	if (!pps)
		return;

	device_unregister(&pps->dev);
}
EXPORT_SYMBOL(pps_device_unregister);

static int pps_match_dev_by_name(struct device *dev, const void *data)
{
	const char *name = data;
	struct pps_device *pps = dev_get_drvdata(dev);

	return strcmp(pps->desc->name, name) == 0;
}

struct pps_device *pps_dev_get_by_name(const char *name)
{
	struct device *dev = class_find_device(pps_class,
			NULL, (const char *)name, pps_match_dev_by_name);
	return dev ? dev_get_drvdata(dev) : NULL;
}
EXPORT_SYMBOL(pps_dev_get_by_name);

static void pps_init_attrs(struct device_type *dev_type)
{
	int i;

	dev_type->groups = pps_attr_groups;
	for (i = 0; i < ARRAY_SIZE(pps_device_attributes); i++)
		__pps_attrs[i] = &pps_device_attributes[i].attr;
}

static int __init pps_class_init(void)
{
	pr_info("%s (%s)\n", __func__, PPS_CLASS_VERSION);

	pps_class = class_create(THIS_MODULE, "richtek_pps");
	if (IS_ERR(pps_class)) {
		pr_info("Unable to create pps class; errno = %ld\n",
		       PTR_ERR(pps_class));
		return PTR_ERR(pps_class);
	}
	pps_init_attrs(&pps_dev_type);
	pps_class->suspend = NULL;
	pps_class->resume = NULL;

	pr_info("PPS class init OK\n");
	return 0;
}

static void __exit pps_class_exit(void)
{
	class_destroy(pps_class);
	pr_info("PPS class un-init OK\n");
}

subsys_initcall(pps_class_init);
module_exit(pps_class_exit);

MODULE_DESCRIPTION("Richtek TypeC Port Control Core");
MODULE_AUTHOR("Jeff Chang <jeff_chang@richtek.com>");
MODULE_VERSION(PPS_CLASS_VERSION);
MODULE_LICENSE("GPL");

/*
 * Revision Note
 * 1.0.2
 * (1) Add inline function for algo pps API
 *
 * 1.0.1
 * (1) Modify API naming from pps_xxx to rt_pps_xxx
 *
 * 1.0.0
 * Initial release
 */
