
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_APDS9930),y)
obj-y   +=  alsps/apds9930/
endif

ifeq ($(CONFIG_BOSCH_BMA255),y)
obj-y   +=  accelerometer/bma255/
endif

ifeq ($(CONFIG_MTK_K303B_ACC),y)
obj-y   +=  accelerometer/k303b-acc/
endif
ifeq ($(CONFIG_BOSCH_BMM050),y)
obj-y	+=  magnetometer/bmm050/
endif

ifeq ($(CONFIG_MTK_K303B),y)
obj-y   +=  magnetometer/k303b/
endif

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y	+= core/
obj-y	+= dct/dct/
endif

custom_kernel_flashlight := $(addsuffix /, $(shell echo $(CONFIG_CUSTOM_KERNEL_FLASHLIGHT)))

obj-y	+=  camera/camera/
# obj-y   +=  flashlight/constant_flashlight/
obj-y   +=  flashlight/$(custom_kernel_flashlight)
obj-y	+=  vibrator/
obj-y	+=  leds/
obj-y	+=  keypad/
obj-y	+=  sound/
obj-y	+=  accdet/

