# SPDX-License-Identifier: GPL-2.0
#
# DMA Engine Helpers
#

obj-$(CONFIG_SH_DMAE_BASE) += shdma-base.o shdma-of.o

#
# DMA Controllers
#

shdma-y := shdmac.o
shdma-objs := $(shdma-y)
obj-$(CONFIG_SH_DMAE) += shdma.o

obj-$(CONFIG_RCAR_DMAC) += rcar-dmac.o
obj-$(CONFIG_RENESAS_USB_DMAC) += usb-dmac.o
obj-$(CONFIG_RZ_DMAC) += rz-dmac.o
obj-$(CONFIG_RZV2MA_DMAC) += rzv2ma-dmac.o
