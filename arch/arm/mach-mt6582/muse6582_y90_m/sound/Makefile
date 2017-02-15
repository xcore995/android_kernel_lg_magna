include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_MTK_SOUND), y)
	#20140930 euna.jo@lge.com : Y90 BringUp - start
	#obj-y += amp_tpa2058d3/		#LGE_BSP : Use ext. amp
	obj-$(CONFIG_MTK_SOUND) += amp_tpa6136a2/
	#20140930 euna.jo@lge.com : Y90 BringUp - end
else
obj-y	:=  yusu_android_speaker.o
endif


