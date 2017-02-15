obj-  := dummy.o

obj-y += mtk_ts_cpu.o

ifeq ($(CONFIG_THERMAL),y)
    ifeq ($(CONFIG_MTK_ENABLE_MD2),y)
    	ifeq ($(CONFIG_MTK_MD2_SUPPORT),3)
	    	obj-y += mtk_ts_tdpa.o
	    endif
    endif

    ifeq ($(CONFIG_MTK_PCB_BATTERY_SENSOR), y)
        obj-y += mtk_ts_battery2.o
    endif
endif

obj-$(CONFIG_THERMAL) += thermal_src.o
thermal_src-y := src/thermal_platform_settings.o
