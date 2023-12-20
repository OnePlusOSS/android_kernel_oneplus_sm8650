# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Linux Kernel SOC specific device drivers.
#

obj-$(CONFIG_ARCH_ACTIONS)	+= actions/
obj-y				+= apple/
obj-y				+= aspeed/
obj-$(CONFIG_ARCH_AT91)		+= atmel/
obj-y				+= bcm/
obj-$(CONFIG_SOC_CANAAN)	+= canaan/
obj-$(CONFIG_ARCH_DOVE)		+= dove/
obj-$(CONFIG_MACH_DOVE)		+= dove/
obj-y				+= fsl/
obj-y				+= fujitsu/
obj-$(CONFIG_ARCH_GEMINI)	+= gemini/
obj-y				+= imx/
obj-y				+= ixp4xx/
obj-$(CONFIG_SOC_XWAY)		+= lantiq/
obj-$(CONFIG_LITEX_SOC_CONTROLLER) += litex/
obj-y				+= mediatek/
obj-y				+= microchip/
obj-y				+= pxa/
obj-y				+= amlogic/
obj-y				+= qcom/
obj-y				+= renesas/
obj-y				+= rockchip/
obj-$(CONFIG_SOC_SAMSUNG)	+= samsung/
obj-$(CONFIG_SOC_SIFIVE)	+= sifive/
obj-y				+= sunxi/
obj-$(CONFIG_ARCH_TEGRA)	+= tegra/
obj-y				+= ti/
obj-$(CONFIG_ARCH_U8500)	+= ux500/
obj-$(CONFIG_PLAT_VERSATILE)	+= versatile/
obj-y				+= xilinx/

#ifdef OPLUS_FEATURE_MM_FEEDBACK
obj-$(CONFIG_OPLUS_FEATURE_MM_FEEDBACK)	+= oplus/multimedia/
#endif /* OPLUS_FEATURE_MM_FEEDBACK */

#ifdef OPLUS_FEATURE_TP_BASIC
obj-$(CONFIG_OPLUS_DEVICE_IFNO)  +=  oplus/device_info/
#endif /* OPLUS_FEATURE_TP_BASIC */

#ifdef OPLUS_FEATURE_BOOT
obj-y += oplus/boot/
#endif /* OPLUS_FEATURE_BOOT */

#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
obj-$(CONFIG_OPLUS_FEATURE_RECORD_MDMRST)     += oplus/mdmrst/
#endif /*OPLUS_FEATURE_MODEM_MINIDUMP*/

obj-$(CONFIG_OPLUS_FEATURE_QCOM_WATCHDOG) += oplus/boot/qcom_watchdog/
obj-y += oplus/dft/

obj-y     += oplus/storage/
