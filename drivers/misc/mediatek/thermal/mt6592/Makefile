obj-  := dummy.o

obj-y += mtk_ts_cpu.o
obj-$(CONFIG_THERMAL) += mtk_ts_AP.o
obj-$(CONFIG_THERMAL) += mtk_ts_6333buck.o

obj-$(CONFIG_THERMAL) += thermal_src.o
thermal_src-y := src/thermal_platform_settings.o src/ts_pa_extra_api.o
