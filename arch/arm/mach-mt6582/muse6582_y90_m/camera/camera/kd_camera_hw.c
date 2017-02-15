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

#include <mach/board_lge.h>

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

extern hw_rev_type lge_get_board_revno(void);
static hw_rev_type rev;

extern void ISP_MCLK1_EN(bool En);
int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
#if 1
u32 pinSetIdx = 0;//default main sensor
int i=0; //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-04] rst pin change to low, before the power on sequence

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

rev=lge_get_board_revno();
PK_DBG("kdCISModulePowerOn HW_REV : %d\n",rev);

u32 pinSet[2][8] = {
                    //for main sensor
                    {GPIO_CAMERA_CMRST_PIN_MAIN,
                        GPIO_CAMERA_CMRST_PIN_MAIN_M_GPIO,   /* mode */
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

        if(rev <= HW_REV_A){
            if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
        }
//[LGE_UPDATE_S][yonghwan.lym@lge.com][2015-01-02] Multiple EEPROM Suport
        if (currSensorName && ((0 == strcmp(SENSOR_DRVNAME_OV8858_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_OV8858S_MIPI_RAW,currSensorName))))
//[LGE_UPDATE_E][yonghwan.lym@lge.com][2015-01-02] Multiple EEPROM Suport
        {
            PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV8858_MIPI_RAW \n");

           ISP_MCLK1_EN(0);
           ISP_MCLK1_EN(1);
           mdelay(5);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV8858_MIPI_RAW -init set pdn/rst \n");
        #if 0
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                mdelay(2);
        #endif
                //RST pin
//[LGE_UPDATE_S][yonghwan.lym@lge.com][2015-03-04] rst pin change to low, before the power on sequence
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
//[LGE_UPDATE_E][yonghwan.lym@lge.com][2015-03-04] rst pin change to low, before the power on sequence
                mdelay(2);
            }

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800/*VOL_2800*/,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)
#if 0
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 //return -EIO;
                 goto _kdCISModulePowerOn_exit_;
            }
#else
            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power On \n");


            if(rev <= HW_REV_A){
                if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            }
            else{
                if(TRUE != hwPowerOn(MT6323_POWER_LDO_VCAMD, VOL_1200/*VOL_2800*/,mode_name))
                {
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                    //return -EIO;
                    goto _kdCISModulePowerOn_exit_;
                }
            }
#endif
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)

#if 1    //S7 RevA
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 //return -EIO;
                 goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)
#endif
            //if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            //{
            //    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
            //    //return -EIO;
            //    goto _kdCISModulePowerOn_exit_;
            //}
            //mdelay(2);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                //mdelay(5);

                //RST pin
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
                mdelay(5);
            }
            mdelay(50);
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX132_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_IMX132_MIPI_RAW \n");

           ISP_MCLK1_EN(0);
           ISP_MCLK1_EN(1);
           mdelay(5);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_IMX132_MIPI_RAW -init set pdn/rst \n");
                #if 0
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                mdelay(2);
                #endif
                //RST pin
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
                mdelay(2);
            }

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            //mdelay(2);
#if 0
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 //return -EIO;
                 goto _kdCISModulePowerOn_exit_;
            }
#else
            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power On \n");
            if(rev <= HW_REV_A){
                if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            }
            else{
                if(TRUE != hwPowerOn(MT6323_POWER_LDO_VCAMD, VOL_1200/*VOL_2800*/,mode_name))
                {
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                    //return -EIO;
                    goto _kdCISModulePowerOn_exit_;
                }
            }
#endif
            //mdelay(2);

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800/*VOL_2800*/,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            //mdelay(2);

#if 1    //S7 RevA
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 //return -EIO;
                 goto _kdCISModulePowerOn_exit_;
            }
#endif
            //if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            //{
            //    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
            //    //return -EIO;
            //    goto _kdCISModulePowerOn_exit_;
            //}
            //mdelay(2);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                //mdelay(5);

                //RST pin
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
                mdelay(5);
            }
            mdelay(50);

        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5670_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV5670_MIPI_RAW \n");

           ISP_MCLK1_EN(0);
           ISP_MCLK1_EN(1);
           mdelay(5);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_OV5670_MIPI_RAW -init set pdn/rst \n");
                //RST pin
//[LGE_UPDATE_S][yonghwan.lym@lge.com][2015-03-04] rst pin change to low, before the power on sequence
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
//[LGE_UPDATE_E][yonghwan.lym@lge.com][2015-03-04] rst pin change to low, before the power on sequence
                mdelay(2);
            }

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800/*VOL_2800*/,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)

            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power On \n");
            if(rev <= HW_REV_A){
                if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            }
            else{
                if(TRUE != hwPowerOn(MT6323_POWER_LDO_VCAMD, VOL_1200/*VOL_2800*/,mode_name))
                {
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                    //return -EIO;
                    goto _kdCISModulePowerOn_exit_;
                }
            }
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)

#if 0    //S7 RevA
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 //return -EIO;
                 goto _kdCISModulePowerOn_exit_;
            }
#endif

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                //mdelay(5);

                //RST pin
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
                mdelay(5);
            }
            mdelay(50);
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI842_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_HI842_MIPI_RAW \n");

           ISP_MCLK1_EN(0);
           mdelay(5);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_HI842_MIPI_RAW -init set pdn/rst \n");
                //RST pin
                for(i=0;i<2;i++)
                {
                    if(mt_set_gpio_mode(pinSet[i][IDX_PS_CMRST],pinSet[i][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                    if(mt_set_gpio_dir(pinSet[i][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                    if(mt_set_gpio_out(pinSet[i][IDX_PS_CMRST],pinSet[i][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                }
                mdelay(2);
            }

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800/*VOL_2800*/,mode_name))
            { // VCAM_IO
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            { //VCAM_A
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

#if 0
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 //return -EIO;
                 goto _kdCISModulePowerOn_exit_;
            }
#else
            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power On \n");
            if(rev <= HW_REV_A){
                if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            }
            else{
                if(TRUE != hwPowerOn(MT6323_POWER_LDO_VCAMD, VOL_1200/*VOL_2800*/,mode_name))
                {
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                    //return -EIO;
                    goto _kdCISModulePowerOn_exit_;
                }
            }
#endif
            mdelay(1);

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            { // VCAM_AF
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 //return -EIO;
                 goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

           ISP_MCLK1_EN(1);
           mdelay(1);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                //mdelay(5);

                //RST pin
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
                mdelay(5);
            }
            mdelay(50);
        }
        else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI553_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_HI553_MIPI_RAW \n");

           ISP_MCLK1_EN(0);
           mdelay(5);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                PK_DBG("kdCISModulePowerOn get in---  SENSOR_DRVNAME_HI553_MIPI_RAW -init set pdn/rst \n");
                //RST pin
                for(i=0;i<2;i++)
                {
                    if(mt_set_gpio_mode(pinSet[i][IDX_PS_CMRST],pinSet[i][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                    if(mt_set_gpio_dir(pinSet[i][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                    if(mt_set_gpio_out(pinSet[i][IDX_PS_CMRST],pinSet[i][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                }
                mdelay(2);
            }

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800/*VOL_2800*/,mode_name))
            { // VCAM_IO
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            { //VCAM_A
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);

#if 0
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 //return -EIO;
                 goto _kdCISModulePowerOn_exit_;
            }
#else
            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power On \n");
            if(rev <= HW_REV_A){
                if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            }
            else{
                if(TRUE != hwPowerOn(MT6323_POWER_LDO_VCAMD, VOL_1200/*VOL_2800*/,mode_name))
                {
                    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                    //return -EIO;
                    goto _kdCISModulePowerOn_exit_;
                }
            }
#endif
            mdelay(1);

#if 0
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
            { // VCAM_AF
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 //return -EIO;
                 goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
#endif

           ISP_MCLK1_EN(1);
           mdelay(1);

            //PDN/STBY pin
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST])
            {
                //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
                //mdelay(5);

                //RST pin
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
                mdelay(5);
            }
            mdelay(50);
        }
    }
    else {//power OFF

#if 0 //TODO: depends on HW layout. Should be notified by SA.
        PK_DBG("Set GPIO 94 for power OFF\n");
        if (mt_set_gpio_pull_enable(GPIO_CAMERA_LDO_EN_PIN, GPIO_PULL_DISABLE)) {PK_DBG("[CAMERA SENSOR] Set GPIO94 PULL DISABLE ! \n"); }
        if(mt_set_gpio_mode(GPIO_CAMERA_LDO_EN_PIN, GPIO_CAMERA_LDO_EN_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAMERA_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAMERA_LDO_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
#endif
//[LGE_UPDATE_S][yonghwan.lym@lge.com][2015-01-02] Multiple EEPROM Suport
        if(currSensorName && ((0 == strcmp(SENSOR_DRVNAME_OV8858_MIPI_RAW,currSensorName)) || (0 == strcmp(SENSOR_DRVNAME_OV8858S_MIPI_RAW,currSensorName))))
//[LGE_UPDATE_E][yonghwan.lym@lge.com][2015-01-02] Multiple EEPROM Suport
        {
            PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_OV8858_MIPI_RAW \n");

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                mdelay(2);

        #if 0
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
                mdelay(2);
        #endif
            }

#if 0
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#else
            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power Off \n");

            if(rev <= HW_REV_A){
                if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            }
            else{
                if(TRUE != hwPowerDown(MT6323_POWER_LDO_VCAMD,mode_name)) {
                    PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                    //return -EIO;
                    goto _kdCISModulePowerOn_exit_;
                }
            }
#endif
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)
#if 1     //S7 RevA
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#endif
            //if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            //{
            //    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
            //    goto _kdCISModulePowerOn_exit_;
            //}

           mdelay(5);
           ISP_MCLK1_EN(0);

        }
        else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX132_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_IMX132_MIPI_RAW \n");

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                mdelay(2);

                #if 0
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
                mdelay(2);
                #endif
            }

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#if 0
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#else
            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power Off \n");

            if(rev <= HW_REV_A){
                if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            }
            else{
                if(TRUE != hwPowerDown(MT6323_POWER_LDO_VCAMD,mode_name)) {
                    PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                    //return -EIO;
                    goto _kdCISModulePowerOn_exit_;
                }
            }
#endif
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#if 1     //S7 RevA
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#endif
            //if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            //{
            //    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
            //    goto _kdCISModulePowerOn_exit_;
            //}

           mdelay(5);
           ISP_MCLK1_EN(0);

        }
        else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5670_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_OV5670_MIPI_RAW \n");

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                mdelay(2);
            }

            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)
            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power Off \n");

            if(rev <= HW_REV_A){
                if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            }
            else{
                if(TRUE != hwPowerDown(MT6323_POWER_LDO_VCAMD,mode_name)) {
                    PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                    //return -EIO;
                    goto _kdCISModulePowerOn_exit_;
                }
            }
            mdelay(1); //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-30] Add delay between VDD:D and Reset(OV8858 & OV5670)
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }

           mdelay(5);
           ISP_MCLK1_EN(0);

        }
        else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI842_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_HI842_MIPI_RAW \n");

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                mdelay(2);

        #if 0
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
                mdelay(2);
        #endif
            }

           ISP_MCLK1_EN(0);
           mdelay(1);

#if 0
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#else
            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power Off \n");

            if(rev <= HW_REV_A){
                if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            }
            else{
                if(TRUE != hwPowerDown(MT6323_POWER_LDO_VCAMD,mode_name)) {
                    PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                    //return -EIO;
                    goto _kdCISModulePowerOn_exit_;
                }
            }
#endif
            mdelay(1);
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
            { // VCAM_A
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
           { // VCAM_IO
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
#if 1     //S7 RevA
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#endif
        }
        else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI553_MIPI_RAW,currSensorName)))
        {
            PK_DBG("kdCISModulePower--off get in---SENSOR_DRVNAME_HI553_MIPI_RAW \n");

            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
                mdelay(2);

        #if 0
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
                mdelay(2);
        #endif
            }

           ISP_MCLK1_EN(0);
           mdelay(1);

#if 0
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#else
            PK_DBG("kdCISModulePowerOn 1.2V DVDD LDO power Off \n");

            if(rev <= HW_REV_A){
                if(mt_set_gpio_mode(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_CAMERA_1V2_MAIN_CAM_DVDD_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(GPIO_CAMERA_1V2_MAIN_CAM_DVDD,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            }
            else{
                if(TRUE != hwPowerDown(MT6323_POWER_LDO_VCAMD,mode_name)) {
                    PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                    //return -EIO;
                    goto _kdCISModulePowerOn_exit_;
                }
            }
#endif
            mdelay(1);
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
            { // VCAM_A
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
           { // VCAM_IO
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            mdelay(1);
#if 0     //S7 RevA
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
#endif
        }
}

    return 0;
_kdCISModulePowerOn_exit_:
    return -EIO;

    #endif
}

EXPORT_SYMBOL(kdCISModulePowerOn);


//!--
//
