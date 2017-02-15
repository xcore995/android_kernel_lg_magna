include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors
#obj-y   +=  alsps.o
#obj-y   +=  alsps_factory.o
#obj-y   +=  aal_control.o

obj-$(CONFIG_MTK_AUTO_DETECT_ALSPS)	+=  alsps_device_check.o
# LGE_CHANGE_S: [2014-10-20] boyoung1.kim@lge.com
# Comment: Add Proximity sensor driver
ifeq ($(CONFIG_MTK_APDS9130),y)
obj-y	+=  apds9130/
endif

ifeq ($(CONFIG_ROHM_RPR0521),y)
obj-y	+=  rpr0521/
endif
obj-$(CONFIG_MTK_GP2AP002S00F)	+=  gp2ap002s00f/
obj-$(CONFIG_MTK_CM36283)	+=  cm36283/
obj-$(CONFIG_MTK_CM36283_NEW)	+=  cm36283-new/
obj-$(CONFIG_MTK_APM_16D)	+=  apm_16d/
obj-$(CONFIG_MTK_EPL2182)	+=  epl2182/
obj-$(CONFIG_MTK_EPL2182_NEW)	+=  EPL2182-new/
obj-$(CONFIG_MTK_STK3X1X)	+=  stk3x1x/
obj-$(CONFIG_MTK_CM36652_NEW)	+=  cm36652-new/
obj-$(CONFIG_MTK_APDS9930)	+=  apds9930/
obj-$(CONFIG_MTK_CM3232_NEW)	+=  cm3232-new/
obj-$(CONFIG_MTK_APDS9930_NEW)	+=  APDS9930-new/
obj-$(CONFIG_MTK_STK3X1X_NEW)	+=  stk3x1x-new/
obj-$(CONFIG_MTK_RPR410)	+=  rpr410/
obj-$(CONFIG_MTK_TMD2771)	+=  tmd2771/
obj-$(CONFIG_MTK_TMD2771_AUTO)	+=  tmd2771_auto/
