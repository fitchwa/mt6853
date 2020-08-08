MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/
subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ifeq ($(CONFIG_MTK_GAUGE_VERSION),30)
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/charger
subdir-ccflags-y += -Werror -I$(srctree)/drivers/power/supply/mediatek/misc
endif


obj-$(CONFIG_RICHTEK_PPS_DC)	+= pps_class.o pps_core.o pps_ta.o pps_chg.o \
					pps_algo.o mtk_pps_chg_intf.o pps_ta_impl.o
