ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include


#甁砵 TEE 平台，兼容头文件
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/core
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/tkcore/include/

obj-y += fpsensor_spi_tee.o

