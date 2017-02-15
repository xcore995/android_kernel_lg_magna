#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <linux/kernel.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERR, PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

extern int g_lge_camera_flag; //8MP:2, 13MP:3 //LGE_UPDATE [youmi.jun@lge.com] 2015/03/24, Apply camera SBP

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
u32 pinSetIdx = 0;//default main sensor
int i=0; // LGE_UPDATE [yonghwan.lym@lge.com] 2016/02/23, rst pin change to low, before the power on sequence

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


u32 pinSet[2][8] = {
                    //for main sensor
                    {GPIO_CAMERA_CMRST_PIN,
                        GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,                   /* ON state */
                        GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_CMPDN_PIN,
                        GPIO_CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                    },
                    //for sub sensor
                    {GPIO_CAMERA_CMRST1_PIN,
                     GPIO_CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     GPIO_CAMERA_CMPDN1_PIN,
                        GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                    },
                   };

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }

    //power ON
    if (On) {
        PK_DBG("kdCISModulePowerOn -on:currSensorName=%s\n",currSensorName);
        PK_DBG("kdCISModulePowerOn -on:pinSetIdx=%d\n",pinSetIdx);

        PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power off/rst \n");
        if (mt_set_gpio_mode(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_CAMERA_1V2_CAM_DVDD_PIN_M_GPIO)) {PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
        if (mt_set_gpio_dir(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_DIR_OUT)) {PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
        if (mt_set_gpio_out(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_OUT_ZERO)) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

        if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K3L2XX_MIPI_RAW,currSensorName)))
        {
            if (g_lge_camera_flag == 2 /*8MP*/) return 0; //LGE_UPDATE [youmi.jun@lge.com] 2015/03/24, Apply camera SBP

            PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_S5K3L2XX_MIPI_RAW \n");
            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_S5K3L2XX_MIPI_RAW -init set pdn/rst \n");
                #if 0
                if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])) {PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)) {PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])) {PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                mdelay(2);
                #endif
                //RST pin
 // LGE_UPDATE_S [yonghwan.lym@lge.com] 2016/02/23, rst pin change to low, before the power on sequence
#if 0
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
#else
                for(i=0;i<2;i++)
                {
                    if(mt_set_gpio_mode(pinSet[i][IDX_PS_CMRST],pinSet[i][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                    if(mt_set_gpio_dir(pinSet[i][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                    if(mt_set_gpio_out(pinSet[i][IDX_PS_CMRST],pinSet[i][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                }
#endif
 // LGE_UPDATE_E [yonghwan.lym@lge.com] 2016/02/23, rst pin change to low, before the power on sequence
                mdelay(2);
            }

            if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital io power\n");
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

            if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power On \n");
            if (mt_set_gpio_mode(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_CAMERA_1V2_CAM_DVDD_PIN_M_GPIO)) {PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if (mt_set_gpio_dir(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_DIR_OUT)) {PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if (mt_set_gpio_out(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_OUT_ONE)) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(5);

            if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable af  power\n");
                 goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                //if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])) {PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                //mdelay(5);

                //RST pin
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
                mdelay(5);
            }
            //mdelay(50);
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8858_MIPI_RAW,currSensorName)))
        {
            if (g_lge_camera_flag == 3 /*13MP*/) return 0; //LGE_UPDATE [youmi.jun@lge.com] 2015/03/24, Apply camera SBP

            PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV8858_MIPI_RAW \n");
            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV8858_MIPI_RAW -init set pdn/rst \n");
                #if 0
                if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])) {PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)) {PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])) {PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                mdelay(2);
                #endif
                //RST pin
 // LGE_UPDATE_S [yonghwan.lym@lge.com] 2016/02/23, rst pin change to low, before the power on sequence
#if 0
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
#else
                for(i=0;i<2;i++)
                {
                    if(mt_set_gpio_mode(pinSet[i][IDX_PS_CMRST],pinSet[i][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                    if(mt_set_gpio_dir(pinSet[i][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                    if(mt_set_gpio_out(pinSet[i][IDX_PS_CMRST],pinSet[i][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                }
#endif
 // LGE_UPDATE_E [yonghwan.lym@lge.com] 2016/02/23, rst pin change to low, before the power on sequence
                mdelay(2);
            }

            if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

            if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital io power\n");
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power On \n");
            if (mt_set_gpio_mode(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_CAMERA_1V2_CAM_DVDD_PIN_M_GPIO)) {PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if (mt_set_gpio_dir(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_DIR_OUT)) {PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if (mt_set_gpio_out(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_OUT_ONE)) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(5);

            if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable af  power\n");
                 goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                //if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])) {PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                //mdelay(5);

                //RST pin
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
                mdelay(5);
            }
            //mdelay(50);
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5670_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV5670_MIPI_RAW \n");
            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV5670_MIPI_RAW -init set pdn/rst \n");
                #if 0
                if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])) {PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)) {PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])) {PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                mdelay(2);
                #endif
                //RST pin
 // LGE_UPDATE_S [yonghwan.lym@lge.com] 2016/02/23, rst pin change to low, before the power on sequence
#if 0
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
#else
                for(i=0;i<2;i++)
                {
                    if(mt_set_gpio_mode(pinSet[i][IDX_PS_CMRST],pinSet[i][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                    if(mt_set_gpio_dir(pinSet[i][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                    if(mt_set_gpio_out(pinSet[i][IDX_PS_CMRST],pinSet[i][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                }
#endif
 // LGE_UPDATE_E [yonghwan.lym@lge.com] 2016/02/23, rst pin change to low, before the power on sequence
                mdelay(2);
            }

            if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

            if (TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digitalio power\n");
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power On \n");
            if (mt_set_gpio_mode(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_CAMERA_1V2_CAM_DVDD_PIN_M_GPIO)) {PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if (mt_set_gpio_dir(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_DIR_OUT)) {PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if (mt_set_gpio_out(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_OUT_ONE)) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(1);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                //if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])) {PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                //mdelay(5);

                //RST pin
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
                mdelay(5);
            }
            //mdelay(50);
        }
    }
    else {//power OFF
        if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K3L2XX_MIPI_RAW,currSensorName)))
        {
            if (g_lge_camera_flag == 2 /*8MP*/) return 0; //LGE_UPDATE [youmi.jun@lge.com] 2015/03/24, Apply camera SBP

            PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_S5K3L2XX_MIPI_RAW \n");

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])) {PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)) {PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                mdelay(2);

                #if 0
                if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])) {PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)) {PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])) {PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
                mdelay(2);
                #endif
            }

            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power Off \n");
            if (mt_set_gpio_mode(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_CAMERA_1V2_CAM_DVDD_PIN_M_GPIO)) {PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if (mt_set_gpio_dir(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_DIR_OUT)) {PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if (mt_set_gpio_out(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_OUT_ZERO)) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

            if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }

            if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital io power\n");
                goto _kdCISModulePowerOn_exit_;
            }

            if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable af power\n");
                goto _kdCISModulePowerOn_exit_;
            }
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8858_MIPI_RAW,currSensorName)))
        {
            if (g_lge_camera_flag == 3 /*13MP*/) return 0; //LGE_UPDATE [youmi.jun@lge.com] 2015/03/24, Apply camera SBP

            PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_OV8858_MIPI_RAW \n");

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])) {PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)) {PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                mdelay(2);

                #if 0
                if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])) {PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)) {PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])) {PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
                mdelay(2);
                #endif
            }

            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power Off \n");
            if (mt_set_gpio_mode(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_CAMERA_1V2_CAM_DVDD_PIN_M_GPIO)) {PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if (mt_set_gpio_dir(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_DIR_OUT)) {PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if (mt_set_gpio_out(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_OUT_ZERO)) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

            if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital io power\n");
                goto _kdCISModulePowerOn_exit_;
            }

            if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }

            if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable af power\n");
                goto _kdCISModulePowerOn_exit_;
            }
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5670_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_OV5670_MIPI_RAW \n");

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])) {PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)) {PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                mdelay(2);
#if 0
                if (mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])) {PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if (mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)) {PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if (mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])) {PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
                mdelay(2);
#endif
            }

            if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital io power\n");
                goto _kdCISModulePowerOn_exit_;
            }

            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power Off \n");
            if (mt_set_gpio_mode(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_CAMERA_1V2_CAM_DVDD_PIN_M_GPIO)) {PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if (mt_set_gpio_dir(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_DIR_OUT)) {PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if (mt_set_gpio_out(GPIO_CAMERA_1V2_CAM_DVDD,GPIO_OUT_ZERO)) {PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

            if (TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
                goto _kdCISModulePowerOn_exit_;
            }

        }
    }

    return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);


//!--
//
