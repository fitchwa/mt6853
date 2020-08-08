###
# Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
#

subdir-ccflags-y += -I$(srctree)/sound/soc/mediatek/common
EXTRA_CFLAGS += -Wall -Werror
MODFLAGS = -fno-pic
CFLAGS_MODULE = $(MODFLAGS)
AFLAGS_MODULE = $(MODFLAGS)

snd-soc-fsm-objs := fsm_core.o
snd-soc-fsm-objs += fs1603.o
snd-soc-fsm-objs += fsm_i2c.o

obj-$(CONFIG_SND_SOC_FS18XX) += snd-soc-fsm.o
