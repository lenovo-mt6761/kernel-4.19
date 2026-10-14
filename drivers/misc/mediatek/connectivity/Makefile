#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

# Connectivity combo driver
# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
    subdir-ccflags-y += -I$(srctree)/
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/clkbuf/src
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1/$(MTK_PLATFORM)
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ifeq ($(CONFIG_MTK_PMIC_CHIP_MT6359),y)
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6359
endif
ifeq ($(CONFIG_MTK_PMIC_NEW_ARCH),y)
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
endif
    subdir-ccflags-y += -I$(srctree)/drivers/mmc/core
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/$(MTK_PLATFORM)
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/eccci/
    subdir-ccflags-y += -I$(srctree)/drivers/clk/mediatek/
    subdir-ccflags-y += -I$(srctree)/drivers/pinctrl/mediatek/
    subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/power_throttling/

    # Do Nothing, move to standalone repo
    MODULE_NAME := connadp
    obj-$(CONFIG_MTK_COMBO) += $(MODULE_NAME).o

    $(MODULE_NAME)-objs += common/connectivity_build_in_adapter.o
    $(MODULE_NAME)-objs += common/wmt_build_in_adapter.o
    $(MODULE_NAME)-objs += power_throttling/adapter.o
    $(MODULE_NAME)-objs += power_throttling/core.o
    ifeq ($(CONFIG_CONN_PWR_DEBUG),y)
        $(MODULE_NAME)-objs += power_throttling/test.o
    endif

    ifeq ($(CONFIG_MTK_COMBO), y)
        ccflags-y += -D CFG_CONNADP_BUILD_IN
    endif

    # for gen4m options
    export CONFIG_MTK_COMBO_WIFI_HIF=axi
    export MTK_COMBO_CHIP=CONNAC
    export BT_PLATFORM=connac1x
    export WLAN_CHIP_ID=6765
    export MTK_ANDROID_WMT=y
    export MTK_ANDROID_EMI=y
    export WIFI_IP_SET=1
    export WIFI_ECO_VER=1
    export MTK_WLAN_SERVICE=yes
    # Do build-in for Makefile checking
    # export CONFIG_WLAN_DRV_BUILD_IN=y

    ifeq ($(CONFIG_WLAN_DRV_BUILD_IN),y)
        PATH_TO_WMT_DRV      = vendor/mediatek/kernel_modules/connectivity/common
        PATH_TO_WLAN_CHR_DRV = vendor/mediatek/kernel_modules/connectivity/wlan/adaptor
        PATH_TO_WLAN_DRV     = vendor/mediatek/kernel_modules/connectivity/wlan/core/gen4m

        ABS_PATH_TO_WMT_DRV      = $(srctree)/../$(PATH_TO_WMT_DRV)
        ABS_PATH_TO_WLAN_CHR_DRV = $(srctree)/../$(PATH_TO_WLAN_CHR_DRV)
        ABS_PATH_TO_WLAN_DRV     = $(srctree)/../$(PATH_TO_WLAN_DRV)

        # check wlan driver folder
        ifeq (,$(wildcard $(ABS_PATH_TO_WMT_DRV)))
            $(error $(ABS_PATH_TO_WMT_DRV) is not existed)
        endif
        ifeq (,$(wildcard $(ABS_PATH_TO_WLAN_CHR_DRV)))
            $(error $(ABS_PATH_TO_WLAN_CHR_DRV) is not existed)
        endif
        ifeq (,$(wildcard $(ABS_PATH_TO_WLAN_DRV)))
            $(error $(ABS_PATH_TO_WLAN_DRV) is not existed)
        endif

        # TB301FU (sjllls): NO vendor-tree symlinks here. The Teracube
        # kernel-tree Kbuild/Makefile files (common/, wlan/adaptor/,
        # wlan/core/gen4m/) already natively flip to obj-y when
        # CONFIG_WLAN_DRV_BUILD_IN=y. Creating the upstream wmt_drv/
        # wmt_chrdev_wifi/ wlan_drv_gen4m/ symlinks on top of that builds
        # every object twice and fails vmlinux link with duplicate symbols.
        # The Kbuild files below are descended into once via the plain
        # obj-$(CONFIG_xxx) lines further down.

        # Do build-in for xxx.c checking
        subdir-ccflags-y += -D MTK_WCN_REMOVE_KERNEL_MODULE
        subdir-ccflags-y += -D MTK_WCN_BUILT_IN_DRIVER
    endif

    obj-$(CONFIG_MTK_COMBO) += common/
    obj-$(CONFIG_MTK_COMBO_WIFI) += wlan/adaptor/
    # TB301FU (sjllls): gen4m is built-in via CONFIG_WLAN_DRV_BUILD_IN
    # (its own Makefile flips obj-y; CONFIG_MTK_COMBO_WIFI=y is required
    # so the HIF/HAL config selects the right flavor).
    obj-$(CONFIG_MTK_COMBO_CHIP_CONSYS_6765) += wlan/core/gen4m/
    obj-$(CONFIG_MTK_BTIF) += bt/mt66xx/wmt/
    obj-$(CONFIG_MTK_COMBO_GPS) += gps/
    obj-$(CONFIG_MTK_FMRADIO) += fmradio/
    obj-$(CONFIG_MTK_CONNFEM) += connfem/

# Otherwise we were called directly from the command line;
# invoke the kernel build system.
else
    KERNELDIR ?= /lib/modules/$(shell uname -r)/build
    PWD  := $(shell pwd)
default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
endif
