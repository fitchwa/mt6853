# Copyright (C) Microarray
# MicroArray Fprint Driver Code
# MAKEFILE
# Date: 2016-10-12
# Version: v4.0.01 
# Author: guq
# Contact: guq@microarray.com.cn


subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include \
					-I$(srctree)/drivers/misc/mediatek/include/mt-plat

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
subdir-ccflags-y += -I$(srctree)/drivers/pinctrl/mediatek
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/include/
#ifeq ($(CONFIG_FINGERPRINT_MICROARRAY),y)
obj-$(CONFIG_FINGERPRINT_MICROARRAY)		+= mtk-settings.o madev.o
#else ifeq ($(CONFIG_FINGERPRINT_MICROARRAY_REE),y)
obj-$(CONFIG_FINGERPRINT_MICROARRAY_REE)		+= mtk-settings.o madev.o
#else ifeq ($(CONFIG_FINGERPRINT_MICROARRAY_A80T),y)
obj-$(CONFIG_FINGERPRINT_MICROARRAY_A80T)		+= mtk-settings.o madev.o
#else ifeq ($(CONFIG_FINGERPRINT_MICROARRAY_A80T_TEE),y)
obj-$(CONFIG_FINGERPRINT_MICROARRAY_A80T_TEE)		+= mtk-settings.o madev_tee.o
#else ifeq ($(CONFIG_FINGERPRINT_MICROARRAY_TEE),y)
obj-$(CONFIG_FINGERPRINT_MICROARRAY_TEE)	+= mtk-settings.o madev_tee.o
#endif

##ifeq ($(CONFIG_MTK_PLATFORM), "mt6739" "mt6771")
ifneq (,$(filter $(CONFIG_MTK_PLATFORM), "mt6739" "mt6765" "mt6761" "mt6779" "mt6757"))
ccflags-y += -DCONFIG_FINGERPRINT_MICROARRAY_FOR_MT6739
endif
