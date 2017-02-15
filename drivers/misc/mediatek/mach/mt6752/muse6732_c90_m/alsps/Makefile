
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  cm36652-new/
endif

# LGE_CHANGE_S: [2014-10-20] boyoung1.kim@lge.com
# Comment: Add Proximity sensor driver
ifeq ($(CONFIG_MTK_APDS9130),y)
obj-y	+=  apds9130/
endif

ifeq ($(CONFIG_ROHM_RPR0521),y)
obj-y	+=  rpr0521/
endif
# LGE_CHANGE_E: [2014-10-20] boyoung1.kim@lge.com
