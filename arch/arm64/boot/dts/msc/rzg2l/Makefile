#
# Copyright (C) 2023 Avnet Embedded GmbH
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation version 2.
#
# This program is distributed "as is" WITHOUT ANY WARRANTY of any
# kind, whether express or implied; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#

dtb-$(CONFIG_ARCH_R9A07G044) += msc-sm2s-g2l-sc-92N0850E-module.dtb

dtb-$(CONFIG_ARCH_R9A07G044) += overlay-baseboard-ep1.dtb
dtb-$(CONFIG_ARCH_R9A07G044) += overlay-baseboard-vebo.dtb

#Enable support for device-tree overlays
DTC_FLAGS_msc-sm2s-g2l-sc-92N0850E-module += -@

DTC_FLAGS_overlay-baseboard-ep1 += -@
DTC_FLAGS_overlay-baseboard-vebo += -@

always          := $(dtb-y)
subdir-y        := $(dts-dirs)
clean-files     := *.dtb

