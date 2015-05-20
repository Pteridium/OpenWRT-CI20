#
# Copyright (C) 2009-2014 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk

ARCH:=mipsel
BOARD:=jzrisc
BOARDNAME:=JZ47XX
FEATURES:=targz usb ramdisk fpu audio nand ubifs
SUBTARGETS:=ci20
CPU_TYPE:=jzrisc

#KERNEL_PATCHVER:=4.0
KERNEL_PATCHVER:=3.18

include $(INCLUDE_DIR)/target.mk

define Target/Description
	Build firmware images for Ingenic JZ4780 based boards.
endef

$(eval $(call BuildTarget))
