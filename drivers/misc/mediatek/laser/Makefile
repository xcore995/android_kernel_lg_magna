#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

#obj-y += sensordriver/sensordrive_lens.o

ifeq ($(CONFIG_ARCH_MT6592),y)
#UPDATE_LGE_S [seongjo.kim@lge.com] 2014/02/14, Add LaserSensor For LDAF
obj-$(CONFIG_MTK_LASER_SENSOR_SUPPORT)  +=  mt6592/vl6180/
#UPDATE_LGE_E [seongjo.kim@lge.com] 2014/02/14, Add LaserSensor For LDAF
endif
