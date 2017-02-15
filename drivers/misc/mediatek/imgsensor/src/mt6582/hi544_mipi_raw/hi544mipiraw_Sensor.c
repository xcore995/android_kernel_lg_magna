/*******************************************************************************************/
// schedule
//   getsensorid ok
//   open ok
//   setting(pv,cap,video) ok

/*******************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi544mipiraw_Sensor.h"
#include "hi544mipiraw_Camera_Sensor_para.h"
#include "hi544mipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(HI544mipiraw_drv_lock);

#define HI544_TEST_PATTERN_CHECKSUM (0xfc8d8b88)//do rotate will change this value

#define HI544_DEBUG
//#define HI544_DEBUG_SOFIA

#define I2C_BUFFER_LEN 254//for Burst mode
#define BLOCK_I2C_DATA_WRITE iBurstWriteReg
#ifdef HI544_DEBUG
	#define HI544DB(fmt, arg...) pr_debug(fmt, ##arg)
#else
	#define HI544DB(fmt, arg...)
#endif

#ifdef HI544_DEBUG_SOFIA
	#define HI544DBSOFIA(fmt, arg...) pr_debug(fmt, ##arg)
#else
	#define HI544DBSOFIA(fmt, arg...)
#endif

#define mDELAY(ms)  mdelay(ms)

kal_uint32 HI544_FeatureControl_PERIOD_PixelNum=HI544_PV_PERIOD_PIXEL_NUMS;
kal_uint32 HI544_FeatureControl_PERIOD_LineNum=HI544_PV_PERIOD_LINE_NUMS;

UINT16 VIDEO_MODE_TARGET_FPS = 30;
static BOOL ReEnteyCamera = KAL_FALSE;


MSDK_SENSOR_CONFIG_STRUCT HI544SensorConfigData;

kal_uint32 HI544_FAC_SENSOR_REG;

MSDK_SCENARIO_ID_ENUM HI544CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT HI544SensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT HI544SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static HI544_PARA_STRUCT HI544;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId) ;
//#define HI544_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, HI544MIPI_WRITE_ID)
// modify by yfx
//extern int iMultiWriteReg(u8 *pData, u16 lens);

#define HI544_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 2, HI544MIPI_WRITE_ID)

void HI544_write_cmos_sensor_burst(hi544_short_t* para, kal_uint32 len, kal_uint32 slave_addr)
{
	hi544_short_t* pPara = (hi544_short_t*) para;
	kal_uint8 puSendCmd[I2C_BUFFER_LEN]={0,};
	kal_uint32 tosend=0 , IDX=0;
	kal_uint16 addr, addr_next, data;
	if(pPara == NULL)
{
		HI544DBSOFIA("[HI544MIPI] ERROR!! pPara is Null!!\n");
		return;
	}
	while(IDX < len)
	{
		addr = pPara->address;
		if(tosend == 0)
		{
			puSendCmd[tosend++] = (kal_uint8)(addr >> 8) & 0xff;
			puSendCmd[tosend++] = (kal_uint8)(addr & 0xff);
			data = (pPara->data >> 8) & 0xff;
			puSendCmd[tosend++] = (kal_uint8)data;
			data = pPara->data & 0xff;
			puSendCmd[tosend++] = (kal_uint8)data;
			addr_next = addr + 2;
			IDX ++;
			pPara++;
		}
		else if (addr == addr_next)
		{
			data = (pPara->data >> 8) & 0xff;
			puSendCmd[tosend++] = (kal_uint8)data;
			data = pPara->data & 0xff;
			puSendCmd[tosend++] = (kal_uint8)data;
			addr_next = addr + 2;
			IDX ++;
			pPara++;
}

		else // to send out the data if the address not incremental.
		{
			BLOCK_I2C_DATA_WRITE(puSendCmd , tosend, slave_addr);
			tosend = 0;
		}
		if ((tosend >= (I2C_BUFFER_LEN-8)) || (IDX == len))
		{
			BLOCK_I2C_DATA_WRITE(puSendCmd , tosend, slave_addr);
			tosend = 0;
		}
	}
	return;
}
// end

kal_uint16 HI544_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,HI544MIPI_WRITE_ID);
    return get_byte;
}

#define Sleep(ms) mdelay(ms)

void HI544_write_shutter(kal_uint32 shutter)
{
	kal_uint32 min_framelength = HI544_PV_PERIOD_PIXEL_NUMS, max_shutter=0;
	kal_uint32 extra_lines = 0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

	HI544DBSOFIA("!!shutter=%d!!!!!\n", shutter);

	if(HI544.HI544AutoFlickerMode == KAL_TRUE)
	{

		if ( SENSOR_MODE_PREVIEW == HI544.sensorMode )  //(g_iHI544_Mode == HI544_MODE_PREVIEW)	//SXGA size output
		{
			line_length = HI544_PV_PERIOD_PIXEL_NUMS + HI544.DummyPixels;
			max_shutter = HI544_PV_PERIOD_LINE_NUMS + HI544.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == HI544.sensorMode ) //add for video_6M setting
		{
			line_length = HI544_VIDEO_PERIOD_PIXEL_NUMS + HI544.DummyPixels;
			max_shutter = HI544_VIDEO_PERIOD_LINE_NUMS + HI544.DummyLines ;
		}
		else
		{
			line_length = HI544_FULL_PERIOD_PIXEL_NUMS + HI544.DummyPixels;
			max_shutter = HI544_FULL_PERIOD_LINE_NUMS + HI544.DummyLines ;
		}

		switch(HI544CurrentScenarioId)
		{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				HI544DBSOFIA("AutoFlickerMode!!! MSDK_SCENARIO_ID_CAMERA_ZSD  0!!\n");
				min_framelength = max_shutter;// capture max_fps 24,no need calculate
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				if(VIDEO_MODE_TARGET_FPS==30)
				{
					min_framelength = (HI544.videoPclk*10000) /(HI544_VIDEO_PERIOD_PIXEL_NUMS + HI544.DummyPixels)/304*10 ;
				}
				else if(VIDEO_MODE_TARGET_FPS==15)
				{
					min_framelength = (HI544.videoPclk*10000) /(HI544_VIDEO_PERIOD_PIXEL_NUMS + HI544.DummyPixels)/148*10 ;
				}
				else
				{
					min_framelength = max_shutter;
				}
				break;
			default:
				min_framelength = (HI544.pvPclk*10000) /(HI544_PV_PERIOD_PIXEL_NUMS + HI544.DummyPixels)/296*10 ;
    			break;
		}

		HI544DBSOFIA("AutoFlickerMode!!! min_framelength for AutoFlickerMode = %d (0x%x)\n",min_framelength,min_framelength);
		HI544DBSOFIA("max framerate(10 base) autofilker = %d\n",(HI544.pvPclk*10000)*10 /line_length/min_framelength);

		if (shutter < 4)
			shutter = 4;

		if (shutter > (max_shutter-4) )
			extra_lines = shutter - max_shutter + 4;
		else
			extra_lines = 0;

		if ( SENSOR_MODE_PREVIEW == HI544.sensorMode )	//SXGA size output
		{
			frame_length = HI544_PV_PERIOD_LINE_NUMS+ HI544.DummyLines + extra_lines ;
		}
		else if(SENSOR_MODE_VIDEO == HI544.sensorMode)
		{
			frame_length = HI544_VIDEO_PERIOD_LINE_NUMS+ HI544.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			frame_length = HI544_FULL_PERIOD_LINE_NUMS + HI544.DummyLines + extra_lines ;
		}
		HI544DBSOFIA("frame_length 0= %d\n",frame_length);

		if (frame_length < min_framelength)
		{
			//shutter = min_framelength - 4;

			switch(HI544CurrentScenarioId)
			{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				extra_lines = min_framelength- (HI544_FULL_PERIOD_LINE_NUMS+ HI544.DummyLines);
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				extra_lines = min_framelength- (HI544_VIDEO_PERIOD_LINE_NUMS+ HI544.DummyLines);
				break;
			default:
				extra_lines = min_framelength- (HI544_PV_PERIOD_LINE_NUMS+ HI544.DummyLines);
    			break;
			}
			frame_length = min_framelength;
		}

		HI544DBSOFIA("frame_length 1= %d\n",frame_length);

		ASSERT(line_length < HI544_MAX_LINE_LENGTH);		//0xCCCC
		ASSERT(frame_length < HI544_MAX_FRAME_LENGTH); 	//0xFFFF
		
		spin_lock_irqsave(&HI544mipiraw_drv_lock,flags);
		HI544.maxExposureLines = frame_length - 4;
		HI544_FeatureControl_PERIOD_PixelNum = line_length;
		HI544_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&HI544mipiraw_drv_lock,flags);


		HI544_write_cmos_sensor(0x0046, 0x0100);
		//Set total frame length
		HI544_write_cmos_sensor(0x0006, frame_length);
//		HI544_write_cmos_sensor(0x0007, frame_length & 0xFF);

		//Set shutter (Coarse integration time, uint: lines.)
		HI544_write_cmos_sensor(0x0004, shutter);
		//HI544_write_cmos_sensor(0x0005, (shutter) & 0xFF);
		
		HI544_write_cmos_sensor(0x0046, 0x0000);

		HI544DBSOFIA("frame_length 2= %d\n",frame_length);
		HI544DB("framerate(10 base) = %d\n",(HI544.pvPclk*10000)*10 /line_length/frame_length);

		HI544DB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);

	}
	else
	{
		if ( SENSOR_MODE_PREVIEW == HI544.sensorMode )  //(g_iHI544_Mode == HI544_MODE_PREVIEW)	//SXGA size output
		{
			max_shutter = HI544_PV_PERIOD_LINE_NUMS + HI544.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == HI544.sensorMode ) //add for video_6M setting
		{
			max_shutter = HI544_VIDEO_PERIOD_LINE_NUMS + HI544.DummyLines ;
		}
		else
		{
			max_shutter = HI544_FULL_PERIOD_LINE_NUMS + HI544.DummyLines ;
		}

		if (shutter < 4)
			shutter = 4;

		if (shutter > (max_shutter-4) )
			extra_lines = shutter - max_shutter + 4;
		else
			extra_lines = 0;

		if ( SENSOR_MODE_PREVIEW == HI544.sensorMode )	//SXGA size output
		{
			line_length = HI544_PV_PERIOD_PIXEL_NUMS + HI544.DummyPixels;
			frame_length = HI544_PV_PERIOD_LINE_NUMS+ HI544.DummyLines + extra_lines ;
		}
		else if( SENSOR_MODE_VIDEO == HI544.sensorMode )
		{
			line_length = HI544_VIDEO_PERIOD_PIXEL_NUMS + HI544.DummyPixels;
			frame_length = HI544_VIDEO_PERIOD_LINE_NUMS + HI544.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			line_length = HI544_FULL_PERIOD_PIXEL_NUMS + HI544.DummyPixels;
			frame_length = HI544_FULL_PERIOD_LINE_NUMS + HI544.DummyLines + extra_lines ;
		}

		ASSERT(line_length < HI544_MAX_LINE_LENGTH);		//0xCCCC
		ASSERT(frame_length < HI544_MAX_FRAME_LENGTH); 	//0xFFFF

		//Set total frame length
		HI544_write_cmos_sensor(0x0046, 0x0100);		
		HI544_write_cmos_sensor(0x0006, frame_length);
		//HI544_write_cmos_sensor(0x0007, frame_length & 0xFF);
		HI544_write_cmos_sensor(0x0046, 0x0000);		

		spin_lock_irqsave(&HI544mipiraw_drv_lock,flags);
		HI544.maxExposureLines = frame_length -4;
		HI544_FeatureControl_PERIOD_PixelNum = line_length;
		HI544_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&HI544mipiraw_drv_lock,flags);


		//Set shutter (Coarse integration time, uint: lines.)
		HI544_write_cmos_sensor(0x0046, 0x0100);		
		HI544_write_cmos_sensor(0x0004, shutter);
		//HI544_write_cmos_sensor(0x0005, (shutter) & 0xFF);
		HI544_write_cmos_sensor(0x0046, 0x0000);		
		
		//HI544DB("framerate(10 base) = %d\n",(HI544.pvPclk*10000)*10 /line_length/frame_length);
		HI544DB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);
	}

}   /* write_HI544_shutter */

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 HI544Reg2Gain(const kal_uint8 iReg)
{
	kal_uint16 iGain = 64;

	iGain = 4 * iReg + 64;

	return iGain;
}

/*******************************************************************************
*
********************************************************************************/
static kal_uint16 HI544Gain2Reg(const kal_uint16 Gain)
{
	kal_uint16 iReg;
	kal_uint8 iBaseGain = 64;
	
	iReg = Gain / 4 - 16;
    return iReg;//HI544. sensorGlobalGain

}


void write_HI544_gain(kal_uint8 gain)
{
#if 1
	HI544_write_cmos_sensor(0x003a, (gain << 8));
#endif
	return;
}

/*************************************************************************
* FUNCTION
*    HI544_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
void HI544_SetGain(UINT16 iGain)
{
	unsigned long flags;
    kal_uint16 HI544GlobalGain=0;
	kal_uint16 DigitalGain = 0;

    
	// AG = (regvalue / 16) + 1
	if(iGain > 1024)  
	{
		iGain = 1024;
	}
	if(iGain < 64)  // gain的reg最大值是255
	{
		iGain = 64;
	}
		
	HI544GlobalGain = HI544Gain2Reg(iGain); 
	spin_lock(&HI544mipiraw_drv_lock);
	HI544.realGain = iGain;
	HI544.sensorGlobalGain =HI544GlobalGain;
	spin_unlock(&HI544mipiraw_drv_lock);

	HI544DB("[HI544_SetGain]HI544.sensorGlobalGain=0x%x,HI544.realGain=%d\n",HI544.sensorGlobalGain,HI544.realGain);

	HI544_write_cmos_sensor(0x0046, 0x0100);
	write_HI544_gain(HI544.sensorGlobalGain);	
	HI544_write_cmos_sensor(0x0046, 0x0000);

	
	return;	
}   /*  HI544_SetGain_SetGain  */


/*************************************************************************
* FUNCTION
*    read_HI544_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_HI544_gain(void)
{
	kal_uint16 read_gain_anolog=0;
	kal_uint16 HI544RealGain_anolog =0;
	kal_uint16 HI544RealGain =0;

	read_gain_anolog=HI544_read_cmos_sensor(0x003a);
	
	HI544RealGain_anolog = HI544Reg2Gain(read_gain_anolog);
	

	spin_lock(&HI544mipiraw_drv_lock);
	HI544.sensorGlobalGain = read_gain_anolog;
	HI544.realGain = HI544RealGain;
	spin_unlock(&HI544mipiraw_drv_lock);
	HI544DB("[read_HI544_gain]HI544RealGain_anolog=0x%x\n",HI544RealGain_anolog);

	return HI544.sensorGlobalGain;
}  /* read_HI544_gain */


void HI544_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=HI544SensorReg[i].Addr; i++)
    {
        HI544_write_cmos_sensor(HI544SensorReg[i].Addr, HI544SensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HI544SensorReg[i].Addr; i++)
    {
        HI544_write_cmos_sensor(HI544SensorReg[i].Addr, HI544SensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        HI544_write_cmos_sensor(HI544SensorCCT[i].Addr, HI544SensorCCT[i].Para);
    }
}


/*************************************************************************
* FUNCTION
*    HI544_sensor_to_camera_para
*
* DESCRIPTION
*    // update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void HI544_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=HI544SensorReg[i].Addr; i++)
    {
         temp_data = HI544_read_cmos_sensor(HI544SensorReg[i].Addr);
		 spin_lock(&HI544mipiraw_drv_lock);
		 HI544SensorReg[i].Para =temp_data;
		 spin_unlock(&HI544mipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=HI544SensorReg[i].Addr; i++)
    {
        temp_data = HI544_read_cmos_sensor(HI544SensorReg[i].Addr);
		spin_lock(&HI544mipiraw_drv_lock);
		HI544SensorReg[i].Para = temp_data;
		spin_unlock(&HI544mipiraw_drv_lock);
    }
}

/*************************************************************************
* FUNCTION
*    HI544_get_sensor_group_count
*
* DESCRIPTION
*    //
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  HI544_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void HI544_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
}
}

void HI544_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;

    switch (group_idx)
    {
        case PRE_GAIN:
           switch (item_idx)
          {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

            temp_para= HI544SensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/HI544.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= HI544_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= HI544_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }

                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=    111;  //MT9P017_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}



kal_bool HI544_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
//   kal_int16 temp_reg;
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
//             temp_para=(temp_gain * HI544.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

			 HI544DBSOFIA("HI544????????????????????? :\n ");
		  spin_lock(&HI544mipiraw_drv_lock);
          HI544SensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&HI544mipiraw_drv_lock);
          HI544_write_cmos_sensor(HI544SensorCCT[temp_addr].Addr,temp_para);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&HI544mipiraw_drv_lock);
                    HI544_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&HI544mipiraw_drv_lock);
                    break;
                case 1:
                    HI544_write_cmos_sensor(HI544_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}

static void HI544_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == HI544.sensorMode )	//SXGA size output
	{
		line_length = HI544_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI544_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== HI544.sensorMode )
	{
		line_length = HI544_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI544_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else//QSXGA size output
	{
		line_length = HI544_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = HI544_FULL_PERIOD_LINE_NUMS + iLines;
	}

	//if(HI544.maxExposureLines > frame_length -4 )
	//	return;

	//ASSERT(line_length < HI544_MAX_LINE_LENGTH);		//0xCCCC
	//ASSERT(frame_length < HI544_MAX_FRAME_LENGTH);	//0xFFFF

	//Set total frame length
	HI544_write_cmos_sensor(0x0006, frame_length);
	//HI544_write_cmos_sensor(0x0007, frame_length & 0xFF);

	spin_lock(&HI544mipiraw_drv_lock);
	HI544.maxExposureLines = frame_length -4;
	HI544_FeatureControl_PERIOD_PixelNum = line_length;
	HI544_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&HI544mipiraw_drv_lock);

	//Set total line length
	HI544_write_cmos_sensor(0x0008, line_length);
	//HI544_write_cmos_sensor(0x0009, line_length & 0xFF);

}   /*  HI544_SetDummy */

void HI544PreviewSetting(void)
{	
	HI544DB("HI544PreviewSetting_2lane_30fps:\n ");

	//////////////////////////////////////////////////////////////////////////
	//			Sensor        	 : Hi-545
	//      Set Name         : Preview
	//      Mode             : D-Binning ( = Scaler 1/2)
	//      Size             : 1296 * 972
	//	set file	 : v0.45
	//	Date		 : 20140724
	//////////////////////////////////////////////////////////////////////////


	HI544_write_cmos_sensor(0x0118, 0x0000); //sleep On

	//--- SREG ---//
	HI544_write_cmos_sensor(0x0B02, 0x0014); //140718
	HI544_write_cmos_sensor(0x0B16, 0x4A8B); //140718 sys_clk 1/2
	HI544_write_cmos_sensor(0x0B18, 0x0000); //140718
	HI544_write_cmos_sensor(0x0B1A, 0x1044); //140718
	HI544_write_cmos_sensor(0x004C, 0x0100); 
	HI544_write_cmos_sensor(0x000C, 0x0000); 

	//---< mipi time >---//
	HI544_write_cmos_sensor(0x0902, 0x4101); //mipi_value_clk_trail.  
	HI544_write_cmos_sensor(0x090A, 0x01E4); //mipi_vblank_delay_h.
	HI544_write_cmos_sensor(0x090C, 0x0005); //mipi_hblank_short_delay_h.
	HI544_write_cmos_sensor(0x090E, 0x0100); //mipi_hblank_long_delay_h.
	HI544_write_cmos_sensor(0x0910, 0x5D04); //05 mipi_LPX
	HI544_write_cmos_sensor(0x0912, 0x030f); //05 mipi_CLK_prepare
	HI544_write_cmos_sensor(0x0914, 0x0204); //02 mipi_clk_pre
	HI544_write_cmos_sensor(0x0916, 0x0707); //09 mipi_data_zero
	HI544_write_cmos_sensor(0x0918, 0x0f04); //0c mipi_clk_post

	//---< Pixel Array Address >------//
	HI544_write_cmos_sensor(0x0012, 0x00AA); //x_addr_start_hact_h. 170
	HI544_write_cmos_sensor(0x0018, 0x0ACB); //x_addr_end_hact_h. 2763 . 2763-170+1=2594
	HI544_write_cmos_sensor(0x0026, 0x0018); //y_addr_start_vact_h. 24
	HI544_write_cmos_sensor(0x002C, 0x07AF); //y_addr_end_vact_h.   1967. 1967-24+1=1944

	//---< Crop size : 2592x1944 >----//
	HI544_write_cmos_sensor(0x0128, 0x0002); // digital_crop_x_offset_l  
	HI544_write_cmos_sensor(0x012A, 0x0000); // digital_crop_y_offset_l  
	HI544_write_cmos_sensor(0x012C, 0x0A20); // digital_crop_image_width 
	HI544_write_cmos_sensor(0x012E, 0x0798); // digital_crop_image_height

	//---< FMT Size : 1296x972 >---------//
	HI544_write_cmos_sensor(0x0110, 0x0510); //X_output_size_h 
	HI544_write_cmos_sensor(0x0112, 0x03CC); //Y_output_size_h 

	//---< Frame/Line Length >-----//
	HI544_write_cmos_sensor(0x0006, 0x07e1); //frame_length_h 2017
	HI544_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI544_write_cmos_sensor(0x000A, 0x0DB0); 

	//---< ETC set >---------------//
	HI544_write_cmos_sensor(0x0000, 0x0100); //orientation. [0]:x-flip, [1]:y-flip.
	HI544_write_cmos_sensor(0x0700, 0xA0A8); //140718 scaler 1/2
	HI544_write_cmos_sensor(0x001E, 0x0101); //140718
	HI544_write_cmos_sensor(0x0032, 0x0101); //140718
	HI544_write_cmos_sensor(0x0A02, 0x0100); //140718 Fast sleep Enable
	HI544_write_cmos_sensor(0x0A04, 0x0133); //TEST PATTERN

	HI544_write_cmos_sensor(0x0118, 0x0100); //sleep Off 

	mDELAY(50);

	ReEnteyCamera = KAL_FALSE;

    HI544DB("HI544PreviewSetting_2lane exit :\n ");	
}


void HI544VideoSetting(void)
{

	HI544DB("HI544VideoSetting:\n ");
	//////////////////////////////////////////////////////////////////////////
	//			Sensor			 : Hi-545
	//		Set Name		 : FHD
	//		Mode			 : D-Binning ( = Scaler 3/4)
	//		Size			 : 1920 * 1080
	//	set file	 : v0.45
	//	Date		 : 20140724
	//////////////////////////////////////////////////////////////////////////
	
	
	HI544_write_cmos_sensor(0x0118, 0x0000); //sleep On
	
	//--- SREG ---//
	HI544_write_cmos_sensor(0x0B02, 0x0014); //140718
	HI544_write_cmos_sensor(0x0B16, 0x4A0B); 
	HI544_write_cmos_sensor(0x0B18, 0x0000); //140718
	HI544_write_cmos_sensor(0x0B1A, 0x1044); //140718
	HI544_write_cmos_sensor(0x004C, 0x0100); 
	HI544_write_cmos_sensor(0x000C, 0x0000);
	
	//---< mipi time >---//
	HI544_write_cmos_sensor(0x0902, 0x4101); 
	HI544_write_cmos_sensor(0x090A, 0x03E4); 
	HI544_write_cmos_sensor(0x090C, 0x0020); 
	HI544_write_cmos_sensor(0x090E, 0x0020); 
	HI544_write_cmos_sensor(0x0910, 0x5D07); 
	HI544_write_cmos_sensor(0x0912, 0x061e); 
	HI544_write_cmos_sensor(0x0914, 0x0407); 
	HI544_write_cmos_sensor(0x0916, 0x0b0a); 
	HI544_write_cmos_sensor(0x0918, 0x0e09); 
	
	//---< Pixel Array Address >------//
	HI544_write_cmos_sensor(0x0012, 0x00BA); //186
	HI544_write_cmos_sensor(0x0018, 0x0ABB); //2747. 2747-186+1=2562
	HI544_write_cmos_sensor(0x0026, 0x0114); //276
	HI544_write_cmos_sensor(0x002C, 0x06b3); //1715. 1715-276=1440
	
	//---< Crop size : 2560x1440 >----//
	HI544_write_cmos_sensor(0x0128, 0x0002); // digital_crop_x_offset_l
	HI544_write_cmos_sensor(0x012A, 0x0000); // digital_crop_y_offset_l
	HI544_write_cmos_sensor(0x012C, 0x0A00); // digital_crop_image_width
	HI544_write_cmos_sensor(0x012E, 0x05A0); // digital_crop_image_height
	
	//---< FMT Size : 1920x1080 >---------//
	HI544_write_cmos_sensor(0x0110, 0x0780); //X_output_size_h	   
	HI544_write_cmos_sensor(0x0112, 0x0438); //Y_output_size_h	
	
	//---< Frame/Line Length >-----//
	HI544_write_cmos_sensor(0x0006, 0x07C0); //frame_length_h 1984 @2560x1440@30.7fps
	HI544_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI544_write_cmos_sensor(0x000A, 0x0DB0); //line_length for binning 3504
	//---------------------------------------//
	
	//---< ETC set >---------------//
	HI544_write_cmos_sensor(0x0700, 0x5090); //140718
	HI544_write_cmos_sensor(0x001E, 0x0101); //140718
	HI544_write_cmos_sensor(0x0032, 0x0101); //140718
	HI544_write_cmos_sensor(0x0A02, 0x0100); //140718 Fast sleep Enable
	HI544_write_cmos_sensor(0x0A04, 0x0133); //isp_en. [9]s-gamma,[8]MIPI_en,[6]compresion10to8,[5]Scaler,[4]window,[3]DG,[2]LSC,[1]adpc,[0]tpg //TEST PATTERN
	
	HI544_write_cmos_sensor(0x0118, 0x0100); //sleep Off 

	
	mDELAY(50);

	ReEnteyCamera = KAL_FALSE;

	HI544DB("HI544VideoSetting_4:3 exit :\n ");
}


void HI544CaptureSetting(void)
{

    if(ReEnteyCamera == KAL_TRUE)
    {
		HI544DB("HI544CaptureSetting_2lane_SleepIn :\n ");
    }
	else
	{
		HI544DB("HI544CaptureSetting_2lane_streamOff :\n ");
	}

	HI544DB("HI544CaptureSetting_2lane_OB:\n ");
	

	//////////////////////////////////////////////////////////////////////////
	//			Sensor			 : Hi-545
	//		Set Name		 : Capture
	//		Mode			 : Normal
	//		Size			 : 2592 * 1944
	//	set file	 : v0.45
	//	Date		 : 20140724
	//////////////////////////////////////////////////////////////////////////


	HI544_write_cmos_sensor(0x0118, 0x0000); //sleep On

	//--- SREG ---//
	HI544_write_cmos_sensor(0x0B02, 0x0014); //140718
	HI544_write_cmos_sensor(0x0B16, 0x4A0B); 
	HI544_write_cmos_sensor(0x0B18, 0x0000); //140718
	HI544_write_cmos_sensor(0x0B1A, 0x1044); //140718
	HI544_write_cmos_sensor(0x004C, 0x0100); 
	HI544_write_cmos_sensor(0x000C, 0x0000); 

	//---< mipi time >---//
	HI544_write_cmos_sensor(0x0902, 0x4101); 
	HI544_write_cmos_sensor(0x090A, 0x03E4); 
	HI544_write_cmos_sensor(0x090C, 0x0020); 
	HI544_write_cmos_sensor(0x090E, 0x0020); 
	HI544_write_cmos_sensor(0x0910, 0x5D07); 
	HI544_write_cmos_sensor(0x0912, 0x061e); 
	HI544_write_cmos_sensor(0x0914, 0x0407); 
	HI544_write_cmos_sensor(0x0916, 0x0b0a); 
	HI544_write_cmos_sensor(0x0918, 0x0e09); 

	//---< Pixel Array Address >------//
	HI544_write_cmos_sensor(0x0012, 0x00AA); //x_addr_start_hact_h. 170
	HI544_write_cmos_sensor(0x0018, 0x0ACB); //x_addr_end_hact_h. 2763 . 2763-170+1=2594
	HI544_write_cmos_sensor(0x0026, 0x0018); //y_addr_start_vact_h. 24
	HI544_write_cmos_sensor(0x002C, 0x07AF); //y_addr_end_vact_h.	1967. 1967-24+1=1944

	//---< Crop size : 2592x1944 >----//
	HI544_write_cmos_sensor(0x0128, 0x0002); //digital_crop_x_offset_l	
	HI544_write_cmos_sensor(0x012A, 0x0000); //digital_crop_y_offset_l	
	HI544_write_cmos_sensor(0x012C, 0x0A20); //2592 digital_crop_image_width 
	HI544_write_cmos_sensor(0x012E, 0x0798); //1944 digital_crop_image_height 

	//---< FMT Size : 2592x1944 >---------//
	HI544_write_cmos_sensor(0x0110, 0x0A20); //X_output_size_h 
	HI544_write_cmos_sensor(0x0112, 0x0798); //Y_output_size_h 

	//---< Frame/Line Length >-----//
	HI544_write_cmos_sensor(0x0006, 0x07e1); //frame_length_h 2017
	HI544_write_cmos_sensor(0x0008, 0x0B40); //line_length_h 2880
	HI544_write_cmos_sensor(0x000A, 0x0DB0); 

	//---< ETC set >---------------//
	HI544_write_cmos_sensor(0x0000, 0x0100); //orientation. [0]:x-flip, [1]:y-flip.
	HI544_write_cmos_sensor(0x0700, 0x0590); //140718
	HI544_write_cmos_sensor(0x001E, 0x0101); //140718
	HI544_write_cmos_sensor(0x0032, 0x0101); //140718
	HI544_write_cmos_sensor(0x0A02, 0x0100); //140718 Fast sleep Enable
	HI544_write_cmos_sensor(0x0A04, 0x011B); //TEST PATTERN enable

	HI544_write_cmos_sensor(0x0118, 0x0100); //sleep Off 

	mDELAY(50);

	ReEnteyCamera = KAL_FALSE;
	
	HI544DB("HI544CaptureSetting_2lane exit :\n ");
}

static void HI544_Sensor_Init(void)
{
	//////////////////////////////////////////////////////////////////////////
	//<<<<<<<<<  Sensor Information  >>>>>>>>>>///////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//
	//	Sensor        	: Hi-544
	//
	//	Initial Ver.  	: v0.47 for LGE
	//	Initial Date		: 2014-10-15
	//
	//	Customs          : MTK for LGE
	//	AP or B/E        : MT6582
	//	
	//	Image size       : 2604x1956
	//	mclk/pclk        : 24mhz / 88Mhz
	//	MIPI speed(Mbps) : 880Mbps (each lane)
	//	MIPI						 : Non-continuous 
	//	Frame Length     : 1984
	//	V-Blank          : 546us
	//	Line Length	     : 2880
	//	H-Blank          : 386ns / 387ns	
	//	Max Fps          : 30fps (= Exp.time : 33ms )	
	//	Pixel order      : Blue 1st (=BGGR)
	//	X/Y-flip         : No-X/Y flip
	//	I2C Address      : 0x40(Write), 0x41(Read)
	//	AG               : x1
	//	DG               : x1
	//	BLC offset       : 64 code
	//
	//////////////////////////////////////////////////////////////////////////
	//<<<<<<<<<  Notice >>>>>>>>>/////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//
	//	Notice
	//	1) I2C Address & Data Type is 2Byte. 
	//	2) I2C Data construction that high byte is low addres, and low byte is high address. 
	//		 Initial register address must have used the even number. 
	//		 ex){Address, Data} = {Address(Even, 2byte), Data(2byte)} <==== Used Type
	//												= {Address(Even, 2byte(low address)), Data(1byte)} + {Address(Odd, 2byte(high address)), Data(1byte)} <== Not Used Type
	//												= HI544_write_cmos_sensor(0x0000, 0x0F03} => HI544_write_cmos_sensor(0x0000, 0x0F} + HI544_write_cmos_sensor(0x0001, 0x03}
	//	3) The Continuous Mode of MIPI 2 Lane set is HI544_write_cmos_sensor(0x0902, 0x4301}. And, 1lane set is HI544_write_cmos_sensor(0x0902, 0x0301}.
	//		 The Non-continuous Mode of MIPI 2 Lane set is HI544_write_cmos_sensor(0x0902, 0x4101}. And, 1lane set is HI544_write_cmos_sensor(0x0902, 0x0101}.	
	//	4) Analog Gain address is 0x003a. 
	//			ex)	0x0000 = x1, 0x7000 = x8, 0xf000 = x16
	//
	/////////////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////
	////////////// Hi-544 Initial //////////////////
	////////////////////////////////////////////////
	//-- Start --//
	kal_uint32 len = 0;
	len = sizeof(Sensor_Init_Reg) / sizeof(Sensor_Init_Reg[0]);
	HI544_write_cmos_sensor_burst(Sensor_Init_Reg, len, HI544MIPI_WRITE_ID);
}



UINT32 HI544Open(void)
{

    volatile signed int i;
    kal_uint16 sensor_id = 0;
    HI544DB("HI544Open enter :\n ");
    for(i=0;i<3;i++)
    {
        sensor_id = (HI544_read_cmos_sensor(0x0F17)<<8)|HI544_read_cmos_sensor(0x0F16);
        HI544DB("OHI544 READ ID :%x",sensor_id);
        if(sensor_id != HI544MIPI_SENSOR_ID)
        {
            return ERROR_SENSOR_CONNECT_FAIL;
        }
        else
        {
            break;
        }
    }
    spin_lock(&HI544mipiraw_drv_lock);
    HI544.sensorMode = SENSOR_MODE_INIT;
    HI544.HI544AutoFlickerMode = KAL_FALSE;
    HI544.HI544VideoMode = KAL_FALSE;
    spin_unlock(&HI544mipiraw_drv_lock);
    HI544_Sensor_Init();

    spin_lock(&HI544mipiraw_drv_lock);
    HI544.DummyLines= 0;
    HI544.DummyPixels= 0;
    HI544.pvPclk =  ( HI544_PV_CLK / 10000);
    HI544.videoPclk = ( HI544_VIDEO_CLK / 10000);
    HI544.capPclk = (HI544_CAP_CLK / 10000);

    HI544.shutter = 0x4EA;
    HI544.pvShutter = 0x4EA;
    HI544.maxExposureLines =HI544_PV_PERIOD_LINE_NUMS -4;

    HI544.ispBaseGain = BASEGAIN;//0x40
    HI544.sensorGlobalGain = 0x1f;//sensor gain read from 0x350a 0x350b; 0x1f as 3.875x
    HI544.pvGain = 0x1f;
    HI544.realGain = 0x1f;//ispBaseGain as 1x
    spin_unlock(&HI544mipiraw_drv_lock);


    HI544DB("HI544Open exit :\n ");

    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   HI544GetSensorID
*
* DESCRIPTION
*   This function get the sensor ID
*
* PARAMETERS
*   *sensorID : return the sensor ID
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI544GetSensorID(UINT32 *sensorID)
{
    int  retry = 3;

    HI544DB("HI544GetSensorID enter :\n ");

    // check if sensor ID correct
    do {
        *sensorID = (HI544_read_cmos_sensor(0x0F17)<<8)|HI544_read_cmos_sensor(0x0F16);
        if (*sensorID == HI544MIPI_SENSOR_ID)
            {
                HI544DB("Sensor ID = 0x%04x\n", *sensorID);
                break;
            }
        HI544DB("Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);
    if (*sensorID != HI544MIPI_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}
void HI544_SetShutter(kal_uint32 iShutter)
{
   spin_lock(&HI544mipiraw_drv_lock);
   HI544.shutter= iShutter;
   spin_unlock(&HI544mipiraw_drv_lock);

   HI544_write_shutter(iShutter);
   return;

}   /*  HI544_SetShutter   */


UINT32 HI544_read_shutter(void)
{

    kal_uint16 temp_reg1, temp_reg2;
    UINT32 shutter =0;
    temp_reg1 = HI544_read_cmos_sensor(0x0004);
    //temp_reg2 = HI544_read_cmos_sensor(0x0005);
    //read out register value and divide 16;
    shutter  = temp_reg1;

    return shutter;
}

/*************************************************************************
* FUNCTION
*   HI544_night_mode
*
* DESCRIPTION
*   This function night mode of HI544.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void HI544_NightMode(kal_bool bEnable)
{
}/*    HI544_NightMode */



/*************************************************************************
* FUNCTION
*   HI544Close
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI544Close(void)
{
    //  CISModulePowerOn(FALSE);
    //s_porting
    //  DRV_I2CClose(HI544hDrvI2C);
    //e_porting
    ReEnteyCamera = KAL_FALSE;
    return ERROR_NONE;
}    /* HI544Close() */

void HI544SetFlipMirror(kal_int32 imgMirror)
{
#if 1

    switch (imgMirror)
    {
        case IMAGE_NORMAL://IMAGE_NOMAL:
            HI544_write_cmos_sensor(0x0000, 0x0000);//Set normal
            break;
        case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
            HI544_write_cmos_sensor(0x0000, 0x0200);    //Set flip
            break;
        case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
            HI544_write_cmos_sensor(0x0000, 0x0100);//Set mirror
            break;
        case IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
            HI544_write_cmos_sensor(0x0000, 0x0300);    //Set mirror & flip
            break;
    }
#endif
}


/*************************************************************************
* FUNCTION
*   HI544Preview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 HI544Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    HI544DB("HI544Preview enter:");

    // preview size
    if(HI544.sensorMode == SENSOR_MODE_PREVIEW)
    {
        // do nothing
        // FOR CCT PREVIEW
    }
    else
    {
        //HI544DB("HI544Preview setting!!\n");
        HI544PreviewSetting();
    }

    spin_lock(&HI544mipiraw_drv_lock);
    HI544.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
    HI544.DummyPixels = 0;//define dummy pixels and lines
    HI544.DummyLines = 0 ;
    HI544_FeatureControl_PERIOD_PixelNum=HI544_PV_PERIOD_PIXEL_NUMS+ HI544.DummyPixels;
    HI544_FeatureControl_PERIOD_LineNum=HI544_PV_PERIOD_LINE_NUMS+HI544.DummyLines;
    spin_unlock(&HI544mipiraw_drv_lock);

    spin_lock(&HI544mipiraw_drv_lock);
    HI544.imgMirror = sensor_config_data->SensorImageMirror;
    //HI544.imgMirror =IMAGE_NORMAL; //by module layout
    spin_unlock(&HI544mipiraw_drv_lock);

    //HI544SetFlipMirror(sensor_config_data->SensorImageMirror);
    HI544SetFlipMirror(IMAGE_HV_MIRROR);


    HI544DBSOFIA("[HI544Preview]frame_len=%x\n", ((HI544_read_cmos_sensor(0x380e)<<8)+HI544_read_cmos_sensor(0x380f)));

 //   mDELAY(40);
    HI544DB("HI544Preview exit:\n");
    return ERROR_NONE;
}    /* HI544Preview() */



UINT32 HI544Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    HI544DB("HI544Video enter:");

    if(HI544.sensorMode == SENSOR_MODE_VIDEO)
    {
        // do nothing
    }
    else
    {
        HI544VideoSetting();

    }
    spin_lock(&HI544mipiraw_drv_lock);
    HI544.sensorMode = SENSOR_MODE_VIDEO;
    HI544_FeatureControl_PERIOD_PixelNum=HI544_VIDEO_PERIOD_PIXEL_NUMS+ HI544.DummyPixels;
    HI544_FeatureControl_PERIOD_LineNum=HI544_VIDEO_PERIOD_LINE_NUMS+HI544.DummyLines;
    spin_unlock(&HI544mipiraw_drv_lock);


    spin_lock(&HI544mipiraw_drv_lock);
    HI544.imgMirror = sensor_config_data->SensorImageMirror;
    //HI544.imgMirror =IMAGE_NORMAL; //by module layout
    spin_unlock(&HI544mipiraw_drv_lock);
    //HI544SetFlipMirror(sensor_config_data->SensorImageMirror);
    HI544SetFlipMirror(IMAGE_HV_MIRROR);

//    mDELAY(40);
    HI544DB("HI544Video exit:\n");
    return ERROR_NONE;
}


UINT32 HI544Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

     kal_uint32 shutter = HI544.shutter;
    kal_uint32 temp_data;
    //kal_uint32 pv_line_length , cap_line_length,

    if( SENSOR_MODE_CAPTURE== HI544.sensorMode)
    {
        HI544DB("HI544Capture BusrtShot!!!\n");
    }
    else
    {
        HI544DB("HI544Capture enter:\n");
#if 0
        //Record Preview shutter & gain
        shutter=HI544_read_shutter();
        temp_data =  read_HI544_gain();
        spin_lock(&HI544mipiraw_drv_lock);
        HI544.pvShutter =shutter;
        HI544.sensorGlobalGain = temp_data;
        HI544.pvGain =HI544.sensorGlobalGain;
        spin_unlock(&HI544mipiraw_drv_lock);

        HI544DB("[HI544Capture]HI544.shutter=%d, read_pv_shutter=%d, read_pv_gain = 0x%x\n",HI544.shutter, shutter,HI544.sensorGlobalGain);
#endif
        // Full size setting
        HI544CaptureSetting();
    //    mDELAY(20);

        spin_lock(&HI544mipiraw_drv_lock);
        HI544.sensorMode = SENSOR_MODE_CAPTURE;
        HI544.imgMirror = sensor_config_data->SensorImageMirror;
        //HI544.imgMirror =IMAGE_NORMAL; //by module layout
        HI544.DummyPixels = 0;//define dummy pixels and lines
        HI544.DummyLines = 0 ;
        HI544_FeatureControl_PERIOD_PixelNum = HI544_FULL_PERIOD_PIXEL_NUMS + HI544.DummyPixels;
        HI544_FeatureControl_PERIOD_LineNum = HI544_FULL_PERIOD_LINE_NUMS + HI544.DummyLines;
        spin_unlock(&HI544mipiraw_drv_lock);

        //HI544SetFlipMirror(sensor_config_data->SensorImageMirror);

        HI544SetFlipMirror(IMAGE_HV_MIRROR);

        HI544DB("HI544Capture exit:\n");
    }

    return ERROR_NONE;
}    /* HI544Capture() */

UINT32 HI544GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    HI544DB("HI544GetResolution!!\n");

    pSensorResolution->SensorPreviewWidth    = HI544_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight    = HI544_IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorFullWidth        = HI544_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight        = HI544_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorVideoWidth        = HI544_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = HI544_IMAGE_SENSOR_VIDEO_HEIGHT;
//    HI544DB("SensorPreviewWidth:  %d.\n", pSensorResolution->SensorPreviewWidth);
//    HI544DB("SensorPreviewHeight: %d.\n", pSensorResolution->SensorPreviewHeight);
//    HI544DB("SensorFullWidth:  %d.\n", pSensorResolution->SensorFullWidth);
//    HI544DB("SensorFullHeight: %d.\n", pSensorResolution->SensorFullHeight);
    return ERROR_NONE;
}   /* HI544GetResolution() */

UINT32 HI544GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

    pSensorInfo->SensorPreviewResolutionX= HI544_IMAGE_SENSOR_PV_WIDTH;
    pSensorInfo->SensorPreviewResolutionY= HI544_IMAGE_SENSOR_PV_HEIGHT;

    pSensorInfo->SensorFullResolutionX= HI544_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY= HI544_IMAGE_SENSOR_FULL_HEIGHT;

    spin_lock(&HI544mipiraw_drv_lock);
    HI544.imgMirror = pSensorConfigData->SensorImageMirror ;
    spin_unlock(&HI544mipiraw_drv_lock);

       pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_R;
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 4;  // from 2 to 3 for test shutter linearity
    pSensorInfo->PreviewDelayFrame = 4;  // from 442 to 666
    pSensorInfo->VideoDelayFrame = 2;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;//0;            /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0;//0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI544_PV_X_START;
            pSensorInfo->SensorGrabStartY = HI544_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
             pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI544_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = HI544_VIDEO_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
             pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = HI544_FULL_X_START;    //2*HI544_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = HI544_FULL_Y_START;    //2*HI544_IMAGE_SENSOR_PV_STARTY;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorGrabStartX = HI544_PV_X_START;
            pSensorInfo->SensorGrabStartY = HI544_PV_Y_START;
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
             pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = MIPI_DELAY_COUNT;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }
    memcpy(pSensorConfigData, &HI544SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
}   /* HI544GetInfo() */
UINT32 HI544Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
        spin_lock(&HI544mipiraw_drv_lock);
        HI544CurrentScenarioId = ScenarioId;
        spin_unlock(&HI544mipiraw_drv_lock);
        //HI544DB("ScenarioId=%d\n",ScenarioId);
        HI544DB("HI544CurrentScenarioId=%d\n",HI544CurrentScenarioId);

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            HI544Preview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            HI544Video(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            HI544Capture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* HI544Control() */


UINT32 HI544SetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    HI544DB("[HI544SetVideoMode] frame rate = %d\n", u2FrameRate);

    spin_lock(&HI544mipiraw_drv_lock);
    VIDEO_MODE_TARGET_FPS=u2FrameRate;
    spin_unlock(&HI544mipiraw_drv_lock);

    if(u2FrameRate==0)
    {
        HI544DB("Disable Video Mode or dynimac fps\n");
        return KAL_TRUE;
    }
    if(u2FrameRate >30 || u2FrameRate <5)
        HI544DB("error frame rate seting\n");
    if(HI544.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {
        if(HI544.HI544AutoFlickerMode == KAL_TRUE)
        {
            if (u2FrameRate==30)
                frameRate= 304;
            else if(u2FrameRate==15)
                frameRate= 148;//148;
            else
                frameRate=u2FrameRate*10;
            MIN_Frame_length = (HI544.videoPclk*10000)/(HI544_VIDEO_PERIOD_PIXEL_NUMS + HI544.DummyPixels)/frameRate*10;
        }
        else
        {
            if (u2FrameRate==30)
                MIN_Frame_length= HI544_VIDEO_PERIOD_LINE_NUMS;
            else
                MIN_Frame_length = (HI544.videoPclk*10000) /(HI544_VIDEO_PERIOD_PIXEL_NUMS + HI544.DummyPixels)/u2FrameRate;
        }
        if((MIN_Frame_length <=HI544_VIDEO_PERIOD_LINE_NUMS))
        {
            MIN_Frame_length = HI544_VIDEO_PERIOD_LINE_NUMS;
            HI544DB("[HI544SetVideoMode]current fps = %d\n", (HI544.videoPclk*10000)  /(HI544_VIDEO_PERIOD_PIXEL_NUMS)/HI544_VIDEO_PERIOD_LINE_NUMS);
        }
        HI544DB("[HI544SetVideoMode]current fps (10 base)= %d\n", (HI544.videoPclk*10000)*10/(HI544_VIDEO_PERIOD_PIXEL_NUMS + HI544.DummyPixels)/MIN_Frame_length);
        if(HI544.shutter + 4 > MIN_Frame_length)
                MIN_Frame_length = HI544.shutter + 4;
        extralines = MIN_Frame_length - HI544_VIDEO_PERIOD_LINE_NUMS;
        spin_lock(&HI544mipiraw_drv_lock);
        HI544.DummyPixels = 0;//define dummy pixels and lines
        HI544.DummyLines = extralines ;
        spin_unlock(&HI544mipiraw_drv_lock);
        HI544_SetDummy(HI544.DummyPixels,extralines);
    }
    else if(HI544.sensorMode == SENSOR_MODE_CAPTURE)
    {
        HI544DB("-------[HI544SetVideoMode]ZSD???---------\n");
        if(HI544.HI544AutoFlickerMode == KAL_TRUE)
        {
            if (u2FrameRate==15)
                frameRate= 148;
            else
                frameRate=u2FrameRate*10;

            MIN_Frame_length = (HI544.capPclk*10000) /(HI544_FULL_PERIOD_PIXEL_NUMS + HI544.DummyPixels)/frameRate*10;
        }
        else
            MIN_Frame_length = (HI544.capPclk*10000) /(HI544_FULL_PERIOD_PIXEL_NUMS + HI544.DummyPixels)/u2FrameRate;

        if((MIN_Frame_length <=HI544_FULL_PERIOD_LINE_NUMS))
        {
            MIN_Frame_length = HI544_FULL_PERIOD_LINE_NUMS;
            HI544DB("[HI544SetVideoMode]current fps = %d\n", (HI544.capPclk*10000) /(HI544_FULL_PERIOD_PIXEL_NUMS)/HI544_FULL_PERIOD_LINE_NUMS);

        }
        HI544DB("[HI544SetVideoMode]current fps (10 base)= %d\n", (HI544.capPclk*10000)*10/(HI544_FULL_PERIOD_PIXEL_NUMS + HI544.DummyPixels)/MIN_Frame_length);

        if(HI544.shutter + 4 > MIN_Frame_length)
                MIN_Frame_length = HI544.shutter + 4;
        extralines = MIN_Frame_length - HI544_FULL_PERIOD_LINE_NUMS;
        spin_lock(&HI544mipiraw_drv_lock);
        HI544.DummyPixels = 0;//define dummy pixels and lines
        HI544.DummyLines = extralines ;
        spin_unlock(&HI544mipiraw_drv_lock);

        HI544_SetDummy(HI544.DummyPixels,extralines);
    }
    HI544DB("[HI544SetVideoMode]MIN_Frame_length=%d,HI544.DummyLines=%d\n",MIN_Frame_length,HI544.DummyLines);

    return KAL_TRUE;
}

UINT32 HI544SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
    //return ERROR_NONE;

    //HI544DB("[HI544SetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
    if(bEnable) {   // enable auto flicker
        spin_lock(&HI544mipiraw_drv_lock);
        HI544.HI544AutoFlickerMode = KAL_TRUE;
        spin_unlock(&HI544mipiraw_drv_lock);
    } else {
        spin_lock(&HI544mipiraw_drv_lock);
        HI544.HI544AutoFlickerMode = KAL_FALSE;
        spin_unlock(&HI544mipiraw_drv_lock);
        HI544DB("Disable Auto flicker\n");
    }

    return ERROR_NONE;
}

UINT32 HI544SetTestPatternMode(kal_bool bEnable)
{
    HI544DB("[HI544SetTestPatternMode] Test pattern enable:%d\n", bEnable);
#if 1
    if(bEnable)
    {
        HI544_write_cmos_sensor(0x020a,0x0200);
    }
    else
    {
        HI544_write_cmos_sensor(0x020a,0x0000);

    }
#endif
    return ERROR_NONE;
}

UINT32 HI544MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
    kal_uint32 pclk;
    kal_int16 dummyLine;
    kal_uint16 lineLength,frameHeight;

    HI544DB("HI544MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
    switch (scenarioId) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pclk =  HI544_PV_CLK;
            lineLength = HI544_PV_PERIOD_PIXEL_NUMS;
            frameHeight = (10 * pclk)/frameRate/lineLength;
            dummyLine = frameHeight - HI544_PV_PERIOD_LINE_NUMS;
            if(dummyLine<0)
                dummyLine = 0;
            spin_lock(&HI544mipiraw_drv_lock);
            HI544.sensorMode = SENSOR_MODE_PREVIEW;
            spin_unlock(&HI544mipiraw_drv_lock);
            HI544_SetDummy(0, dummyLine);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pclk =  HI544_VIDEO_CLK;
            lineLength = HI544_VIDEO_PERIOD_PIXEL_NUMS;
            frameHeight = (10 * pclk)/frameRate/lineLength;
            dummyLine = frameHeight - HI544_VIDEO_PERIOD_LINE_NUMS;
            if(dummyLine<0)
                dummyLine = 0;
            spin_lock(&HI544mipiraw_drv_lock);
            HI544.sensorMode = SENSOR_MODE_VIDEO;
            spin_unlock(&HI544mipiraw_drv_lock);
            HI544_SetDummy(0, dummyLine);
            break;
             break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pclk = HI544_CAP_CLK;
            lineLength = HI544_FULL_PERIOD_PIXEL_NUMS;
            frameHeight = (10 * pclk)/frameRate/lineLength;
            dummyLine = frameHeight - HI544_FULL_PERIOD_LINE_NUMS;
            if(dummyLine<0)
                dummyLine = 0;
            spin_lock(&HI544mipiraw_drv_lock);
            HI544.sensorMode = SENSOR_MODE_CAPTURE;
            spin_unlock(&HI544mipiraw_drv_lock);
            HI544_SetDummy(0, dummyLine);
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
            break;
        default:
            break;
    }
    return ERROR_NONE;
}
UINT32 HI544MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
{
    switch (scenarioId) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
             *pframeRate = 300;
             break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
             *pframeRate = 240;  // modify by yfx for zsd cc
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
             *pframeRate = 300;
            break;
        default:
            break;
    }
    return ERROR_NONE;
}
UINT32 HI544FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT    *pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;
    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= HI544_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= HI544_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
                *pFeatureReturnPara16++= HI544_FeatureControl_PERIOD_PixelNum;
                *pFeatureReturnPara16= HI544_FeatureControl_PERIOD_LineNum;
                *pFeatureParaLen=4;
                break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            switch(HI544CurrentScenarioId)
            {
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                    *pFeatureReturnPara32 =  HI544_PV_CLK;
                    *pFeatureParaLen=4;
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *pFeatureReturnPara32 =  HI544_VIDEO_CLK;
                    *pFeatureParaLen=4;
                    break;
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                case MSDK_SCENARIO_ID_CAMERA_ZSD:
                    *pFeatureReturnPara32 = HI544_CAP_CLK;
                    *pFeatureParaLen=4;
                    break;
                default:
                    *pFeatureReturnPara32 =  HI544_CAP_CLK;
                    *pFeatureParaLen=4;
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            HI544_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            HI544_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            HI544_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            HI544_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = HI544_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
                spin_lock(&HI544mipiraw_drv_lock);
                HI544SensorCCT[i].Addr=*pFeatureData32++;
                HI544SensorCCT[i].Para=*pFeatureData32++;
                spin_unlock(&HI544mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HI544SensorCCT[i].Addr;
                *pFeatureData32++=HI544SensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
                spin_lock(&HI544mipiraw_drv_lock);
                HI544SensorReg[i].Addr=*pFeatureData32++;
                HI544SensorReg[i].Para=*pFeatureData32++;
                spin_unlock(&HI544mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=HI544SensorReg[i].Addr;
                *pFeatureData32++=HI544SensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=HI544MIPI_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, HI544SensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, HI544SensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &HI544SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            HI544_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            HI544_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=HI544_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            HI544_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            HI544_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            HI544_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_R;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            HI544SetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            HI544GetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            HI544SetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            HI544SetTestPatternMode((BOOL)*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            HI544MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            HI544MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing
            *pFeatureReturnPara32=HI544_TEST_PATTERN_CHECKSUM;
            *pFeatureParaLen=4;
        break;
         case SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:
             {
                 UINT32 *pHorFOV = (UINT32*)pFeatureReturnPara32[0];
                 UINT32 *pVerFOV = (UINT32*)pFeatureReturnPara32[1];

                 HI544DB("SENSOR_FEATURE_GET_SENSOR_VIEWANGLE\n");
                 *pHorFOV = (0x37)<<16 | (0x37); // 16:9 = 55, 4:3 = 55 // (59)<<16 | (55)
                 *pVerFOV = (0x31)<<16 | (0x31); // 16:9 = 49, 4:3 = 49 // (49)<<16 | (49)
                 *pFeatureParaLen = 8;
                 break;
             }
        default:
            break;
    }
    return ERROR_NONE;
}    /* HI544FeatureControl() */


SENSOR_FUNCTION_STRUCT    SensorFuncHI544=
{
    HI544Open,
    HI544GetInfo,
    HI544GetResolution,
    HI544FeatureControl,
    HI544Control,
    HI544Close
};

UINT32 HI544_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncHI544;

	//--- MIPI blank time --------------//
	//----------------------------------//

	//--- Pixel Array Addressing ------//
	//------------------------------//

	//--Crop size 2604x1956 ----///
	//------------------------------//

	//----< Image FMT Size >--------------------//
	//Image size 2604x1956
	//------------------------------------------//

	//----< Frame / Line Length >--------------//
	//---------------------------------------//

	//--- ETC set ----//
	//----------------//

	//--- AG / DG control ----------------//
	//AG

	//DG
	//----------------------------------//


	//-----< Exp.Time >------------------------//
	// Pclk_88Mhz @ Line_length_pclk : 2880 @Exp.Time 33.33ms

	//--- ISP enable Selection ---------------//
	//----------------------------------------//



	//-- END --//	


/*************************************************************************
* FUNCTION
*   HI544Open
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/




	






    return ERROR_NONE;

/*************************************************************************
* FUNCTION
*   HI544GetSensorID
*
* DESCRIPTION
*   This function get the sensor ID
*
* PARAMETERS
*   *sensorID : return the sensor ID
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/


    // check if sensor ID correct
//      mDELAY(1000);
//    } while (1);



/*************************************************************************
* FUNCTION
*   HI544_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of HI544 to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

//   if(HI544.shutter == iShutter)
//   		return;


 



/*************************************************************************
* FUNCTION
*   HI544_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/

	//temp_reg2 = HI544_read_cmos_sensor(0x0005);    
	//read out register value and divide 16;


/*************************************************************************
* FUNCTION
*   HI544_night_mode
*
* DESCRIPTION
*   This function night mode of HI544.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
}/*	HI544_NightMode */



/*************************************************************************
* FUNCTION
*   HI544Close
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
    //  CISModulePowerOn(FALSE);
    //s_porting
    //  DRV_I2CClose(HI544hDrvI2C);
    //e_porting

	


/*************************************************************************
* FUNCTION
*   HI544Preview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/


	// preview size
		// do nothing
		// FOR CCT PREVIEW
		//HI544DB("HI544Preview setting!!\n");
	

	//HI544.imgMirror =IMAGE_NORMAL; //by module layout
	
	//HI544SetFlipMirror(sensor_config_data->SensorImageMirror);
	


 //   mDELAY(40);





		// do nothing



	//HI544.imgMirror =IMAGE_NORMAL; //by module layout
	//HI544SetFlipMirror(sensor_config_data->SensorImageMirror);

//    mDELAY(40);



	//kal_uint32 pv_line_length , cap_line_length,

		//Record Preview shutter & gain

		// Full size setting
	//    mDELAY(20);

		//HI544.imgMirror =IMAGE_NORMAL; //by module layout

		//HI544SetFlipMirror(sensor_config_data->SensorImageMirror);






//    HI544DB("SensorPreviewWidth:  %d.\n", pSensorResolution->SensorPreviewWidth);
//    HI544DB("SensorPreviewHeight: %d.\n", pSensorResolution->SensorPreviewHeight);
//    HI544DB("SensorFullWidth:  %d.\n", pSensorResolution->SensorFullWidth);
//    HI544DB("SensorFullHeight: %d.\n", pSensorResolution->SensorFullHeight);





















		//HI544DB("ScenarioId=%d\n",ScenarioId);













		
		










	//return ERROR_NONE;

    //HI544DB("[HI544SetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);




		









            //HI544_isp_master_clock=*pFeatureData32;



            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.




    /* To Do : Check Sensor status here */


