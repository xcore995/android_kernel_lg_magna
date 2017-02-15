include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_LGE_LUT_KCAL) +=  lge_lcd_kcal.o lge_mtk_lcdc_luts.o

# LGE_CHANGE_S: [2014-09-25] aiden.song@lge.com
obj-$(CONFIG_MACH_LGE) += devices_lge.o
# LGE_CHANGE_E: [2014-09-25] aiden.song@lge.com

obj-$(CONFIG_MACH_LGE) += lge_blocking_monitor.o

obj-$(CONFIG_LGE_BOOTLOADER_LOG) += lge_bootloader_log.o

# LGE_CHANGE_S
obj-$(CONFIG_LGE_HANDLE_PANIC) += lge_handle_panic.o
obj-$(CONFIG_LGE_HIDDEN_RESET) += lge_hidden_reset.o

obj-$(CONFIG_LGE_PM_CHARGING_SCENARIO) += lge_charging_scenario.o

obj-$(CONFIG_PRE_SELF_DIAGNOSIS) += lge_pre_selfd.o

# Unified Power Management User-space Export Node
obj-$(CONFIG_MACH_LGE) += lge_power_sysfs.o