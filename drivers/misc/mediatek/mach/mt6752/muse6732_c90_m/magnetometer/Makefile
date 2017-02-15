
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_YAS532),y)
obj-y	+=  yamaha532/
endif

# LGE_CHANGE_S: [2014-09-15] boyoung1.kim@lge.com
# Comment: Add Magnetometer Sensor driver
ifeq ($(CONFIG_BOSCH_BMM050),y)
obj-y	+=  bmm050/
endif
# LGE_CHANGE_E: [2014-09-15] boyoung1.kim@lge.com
