
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_MPU6515A),y)
obj-y	+=  mpu6515/
endif

# LGE_CHANGE_S: [2014-09-03] boyoung1.kim@lge.com
# Comment: Add Accelerometer Sensor driver
ifeq ($(CONFIG_BOSCH_BMA255),y)
obj-y   +=  bma255/
endif
# LGE_CHANGE_E: [2014-09-03] boyoung1.kim@lge.com

ifeq ($(CONFIG_MTK_BMA255_SDO0),y)
obj-y	+=  bma255/
endif

ifeq ($(CONFIG_MTK_BMA255_SDO1),y)
obj-y	+=  bma255/
endif