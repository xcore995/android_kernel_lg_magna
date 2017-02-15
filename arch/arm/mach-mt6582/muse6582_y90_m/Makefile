
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
ifeq ($(CONFIG_MTK_CM36283),y)
obj-y	+=  alsps/cm36283/
endif

ifeq ($(CONFIG_ROHM_RPR0521),y)
obj-y	+=  alsps/rpr0521/
endif

ifeq ($(CONFIG_MTK_APDS9130),y)
obj-y   +=  alsps/apds9130/
endif

ifeq ($(CONFIG_BOSCH_BMA255),y)
obj-y   +=  accelerometer/bma255/
endif

ifeq ($(CONFIG_BOSCH_BMM050),y)
obj-y	+=  magnetometer/bmm050/
endif

ifeq ($(CONFIG_MTK_BMA050_NEW),y)
obj-y   +=  accelerometer/bma050-new/
endif

ifeq ($(CONFIG_MTK_MPU3000),y)
obj-y   +=  gyroscope/mpu3000/
endif

ifeq ($(CONFIG_MTK_BMM050_NEW),y)
obj-y	+=  magnetometer/bmm050-new/
endif
endif
ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y	+= core/
obj-y	+= dct/dct/
endif

#obj-y	+=  hdmi/
obj-y	+=  camera/camera/
obj-y	+=  vibrator/
obj-y	+=  leds/
obj-y	+=  keypad/
obj-y	+=  sound/
obj-y	+=  accdet/
obj-y	+=  partition/

