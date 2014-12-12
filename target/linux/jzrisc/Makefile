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
FEATURES:=targz nand ubifs audio video usb gpio
SUBTARGETS:=ci20

KERNEL_PATCHVER:=3.18

CPU_TYPE:=mips32r2

include $(INCLUDE_DIR)/target.mk

define Target/Description
	Build firmware images for MIPS32r2 XBurst JZ47xx based boards.
endef

$(eval $(call BuildTarget))
