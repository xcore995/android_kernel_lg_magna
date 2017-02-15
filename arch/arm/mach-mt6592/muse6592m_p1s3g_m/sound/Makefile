include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_MTK_SOUND), y)
	obj-y += amp_tpa2058d3/		#LGE_BSP : Use ext. amp
else
obj-y	:=  yusu_android_speaker.o
endif

