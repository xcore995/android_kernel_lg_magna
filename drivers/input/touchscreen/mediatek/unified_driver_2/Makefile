include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  lgtp_common_driver.o
obj-y	+=  lgtp_platform_api.o
obj-y	+=  lgtp_model_config.o

ifeq ($(CONFIG_JDI_INCELL), y)
obj-y 	+= S3320/
endif

ifeq ($(CONFIG_MTK_TD4191),y)
obj-y	+=  td4191/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= LU201X/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= FT6X36/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= DUMMY/
endif

ifeq ($(CONFIG_TOUCHSCREEN_SYNAPTICS_S3528), y)
obj-y 	+= S3528/
endif
ifeq ($(CONFIG_TOUCHSCREEN_SYNAPTICS_S7020), y)
obj-y 	+= S7020/
endif
