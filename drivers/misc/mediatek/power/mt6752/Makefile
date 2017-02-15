include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o pmic_auxadc.o pmic_chr_type_det.o tps6128x.o mt6311.o

ifeq ($(CONFIG_MTK_PMIC_DVT_SUPPORT),y)
     obj-$(CONFIG_MTK_PMIC)     += pmic_dvt.o
endif

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o 
# LGE_CHANGE_S: [2014-09-19] youngmin.kwon@lge.com #
# Comment: Support RT9536 linear charger ic #
ifeq ($(CONFIG_MTK_EXTERNAL_CHARGER_SUPPORT),y)
     obj-y += charging_hw_external_charger.o
     obj-$(CONFIG_MTK_EXTERNAL_CHARGER_RT9536) += rt9536.o
else
# LGE_CHANGE_E: [2014-09-19] youngmin.kwon@lge.com #
ifeq ($(CONFIG_MTK_BQ24261_SUPPORT),y)
     obj-$(CONFIG_MTK_SMART_BATTERY) += bq24261.o charging_hw_bq24261.o
else
     ifeq ($(CONFIG_MTK_SWCHR_SUPPORT),y)
          obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_mt_sw_charger.o                
     else
          ifeq ($(CONFIG_MTK_NCP1854_SUPPORT),y)
               obj-$(CONFIG_MTK_SMART_BATTERY) += ncp1854.o charging_hw_ncp1854.o             
          else
               ifeq ($(CONFIG_MTK_RT9536_SUPPORT),y)
                   obj-$(CONFIG_MTK_SMART_BATTERY) += rt9536/
               else               
                    ifeq ($(CONFIG_MTK_BQ24196_SUPPORT),y)
                        obj-$(CONFIG_MTK_SMART_BATTERY) += bq24196.o charging_hw_bq24196.o
                    else
                        obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_pmic.o
                    endif
               endif
          endif
      endif
endif
endif
