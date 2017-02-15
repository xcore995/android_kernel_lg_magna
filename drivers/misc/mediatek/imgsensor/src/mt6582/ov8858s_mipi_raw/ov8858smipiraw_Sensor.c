/*******************************************************************************************/
     

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

#include "ov8858smipiraw_Sensor.h"
#include "ov8858smipiraw_Camera_Sensor_para.h"
#include "ov8858smipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(OV8858Smipiraw_drv_lock);

#define OV8858S_DEBUG
#ifdef OV8858S_DEBUG
	#define OV8858SDB(fmt, arg...) pr_debug(fmt, ##arg)
#else
	#define OV8858SDB(fmt, arg...)
#endif


kal_uint32 OV8858S_FeatureControl_PERIOD_PixelNum=OV8858S_PV_PERIOD_PIXEL_NUMS;
kal_uint32 OV8858S_FeatureControl_PERIOD_LineNum=OV8858S_PV_PERIOD_LINE_NUMS;

UINT16 OV8858S_VIDEO_MODE_TARGET_FPS = 30;

MSDK_SCENARIO_ID_ENUM OV8858SCurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
MSDK_SENSOR_CONFIG_STRUCT OV8858SSensorConfigData;
static OV8858S_PARA_STRUCT OV8858S;
kal_uint32 OV8858S_FAC_SENSOR_REG;


SENSOR_REG_STRUCT OV8858SSensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT OV8858SSensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;


//TODO~
#define OV8858S_TEST_PATTERN_CHECKSUM 0x47a75476
kal_bool OV8858S_During_testpattern = KAL_FALSE;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define OV8858S_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV8858S.write_id) //yanggy modify for I2c read >>>Do not modify this part when code merging,or I2C will not available
//Gionee yanggy add for CAM Performance Optimization begin
#ifdef ORIGINAL_VERSION 
#else
static struct OV8858S_MIPI_otp_struct s_otp_wb;
static struct OV8858S_MIPI_otp_struct s_otp_lenc;
static bool isNeedReadOtp = true;
#endif
//Gionee yanggy add for CAM Performance Optimization end
kal_uint16 OV8858S_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	iReadReg((u16) addr ,(u8*)&get_byte,OV8858S.write_id);   //yanggy modify for I2c read >>>Do not modify this part when code merging,or I2C will not available
    return get_byte;
}


//Gionee yanggy 2013-09-03 add for module compability begin
#ifdef GN_MTK_BSP_LENS_COMPABILITY
static unsigned int OV8858S_MIPI_Module_id=0;
static unsigned int OV8858S_MIPI_Module_Lens_id =0;
#endif

//OV8858S OTP ¡§?|¡§??code
#if GN_MTK_BSP_LENS_COMPABILITY //OV8858S otp start
struct OV8858S_MIPI_otp_struct
{
  kal_uint16 customer_id;
	kal_uint16 module_integrator_id;
	kal_uint16 lens_id;
	kal_uint16 rg_ratio;
	kal_uint16 bg_ratio;
	kal_uint16 light_rg;
	kal_uint16 light_bg;
	kal_uint16 user_data[5];
	kal_uint16 lenc[110];//63
};
//R/G and B/G ratio of typical camera module is defined here
kal_uint16 RG_Ratio_typical = 0x146; //RG_TYPICAL
kal_uint16 BG_Ratio_typical = 0x12a;//BG_TYPICAL

struct OV8858S_MIPI_otp_struct current_otp={0};

kal_uint16 OV8858S_mipi_check_otp_info(kal_uint16 index)
{
	kal_uint16 flag;

   	OV8858S_write_cmos_sensor(0x3d84,0xc0); 		
   	//partial mode otp write start address
   	OV8858S_write_cmos_sensor(0x3d88,0x70); 	
   	OV8858S_write_cmos_sensor(0x3d89,0x10); 	
   	//partial mode otp write end address
   	OV8858S_write_cmos_sensor(0x3d8a,0x70); 	
   	OV8858S_write_cmos_sensor(0x3d8b,0x10); 	
   	
   	//read otp into buffer
   	OV8858S_write_cmos_sensor(0x3d81,0x01); 
   	mdelay(10);
	
	  //select group
		flag =OV8858S_read_cmos_sensor(0x7010);
  /* 	
   	switch(index)
   	{
   		case 1:
   			flag =(flag>>6)&0x03;
   			break;
   		case 2:
   			flag =(flag>>4)&0x03;
   			break;
   		case 3:
   			flag =(flag>>2)&0x03;	
   			break;
   		default :
   				break;
   	}
   	*/

	if(index == 1)
		flag =(flag>>6)&0x03;
	else if(index == 2)
		flag =(flag>>4)&0x03;
	else if(index == 3)
		flag =(flag>>2)&0x03;	
	
   	//clear otp buffer
   	OV8858S_write_cmos_sensor(0x7010,0x00); 
   	
		if(flag ==0x00)
			{
				return 0;
			}	
			else if(flag & 0x02)
				{
					return 1;
				}
				else
					{
						return 2;
					}
	
}

kal_uint16 OV8858S_mipi_read_otp_info(kal_uint16 index, struct OV8858S_MIPI_otp_struct *otp)
{
    kal_uint16 i=0,temp=0;
    kal_uint16 start_addr=0,end_addr=0;
	  
    if(index == 1)
    	{
    		start_addr=0x7011;
    		end_addr  =0x7015;
    	}
   else  if(index == 2)
   	{
    		start_addr=0x7016;
    		end_addr  =0x701a;
    	}
   else  if(index == 3)
   	{
    		start_addr=0x701b;
    		end_addr  =0x701f;
    }
    
    OV8858S_write_cmos_sensor(0x3d84,0xc0);
   	//partial mode otp write start address
    OV8858S_write_cmos_sensor(0x3d88,(start_addr>>8)&0xff);
    OV8858S_write_cmos_sensor(0x3d89,start_addr & 0xff);
    //partial mode otp write end address
    OV8858S_write_cmos_sensor(0x3d8a,(end_addr>>8)&0xff);
    OV8858S_write_cmos_sensor(0x3d8b,end_addr & 0xff);
    
    //read otp into buffer
    OV8858S_write_cmos_sensor(0x3d81,0x01);
    
    mdelay(10);	
    otp->module_integrator_id = OV8858S_read_cmos_sensor(start_addr);
    otp->lens_id=OV8858S_read_cmos_sensor(start_addr+1);


		//clear otp buffer
		for(i=start_addr;i<end_addr;i++)
		{
			OV8858S_write_cmos_sensor(i,0x00);
		}
		
		return 0;
}


kal_uint16 OV8858S_mipi_check_otp_wb(kal_uint16 index)
{
   	kal_uint16 flag;

   	OV8858S_write_cmos_sensor(0x3d84,0xc0); 		
   	//partial mode otp write start address
   	OV8858S_write_cmos_sensor(0x3d88,0x70); 	
   	OV8858S_write_cmos_sensor(0x3d89,0x20); 	
   	//partial mode otp write end address
   	OV8858S_write_cmos_sensor(0x3d8a,0x70); 	
   	OV8858S_write_cmos_sensor(0x3d8b,0x20); 	
   	
   	//read otp into buffer
   	OV8858S_write_cmos_sensor(0x3d81,0x01); 
   	mdelay(10);
	
	  //select group
		flag =OV8858S_read_cmos_sensor(0x7020);
   	
   	switch(index)
   	{
   		case 1:
   			flag =(flag>>6)&0x03;
   			break;
   		case 2:
   			flag =(flag>>4)&0x03;
   			break;
   		case 3:
   			flag =(flag>>2)&0x03;	
   			break;
   		default :
   				break;
   	}
   	
   	//clear otp buffer
   	OV8858S_write_cmos_sensor(0x7020,0x00); 
   	
		if(flag ==0x00)
			{
				return 0;
			}	
			else if(flag & 0x02)
				{
					return 1;
				}
				else
					{
						return 2;
					}
}

//index:index of otp group.(0,1,2)
//return:	0.group index is empty.
//		1.group index has invalid data
//		2.group index has valid data

kal_uint16 OV8858S_mipi_check_otp_lenc(kal_uint16 index)
{
  	kal_uint16 flag;

   	OV8858S_write_cmos_sensor(0x3d84,0xc0); 		
   	//partial mode otp write start address
   	OV8858S_write_cmos_sensor(0x3d88,0x70); 	
   	OV8858S_write_cmos_sensor(0x3d89,0x3a); 	
   	//partial mode otp write end address
   	OV8858S_write_cmos_sensor(0x3d8a,0x70); 	
   	OV8858S_write_cmos_sensor(0x3d8b,0x3a); 	
   	
   	//read otp into buffer
   	OV8858S_write_cmos_sensor(0x3d81,0x01); 
   	mdelay(10);
	
	  //select group
		flag =OV8858S_read_cmos_sensor(0x703a);
   	switch(index)
   	{
   		case 1:
   			flag =(flag>>6)&0x03;
   			break;
   		case 2:
   			flag =(flag>>4)&0x03;
   			break;
   		case 3:
   			flag =(flag>>2)&0x03;	
   			break;
   		default :
   			OV8858SDB("[OV8858S_mipi_check_otp_lenc] index error\n");
   				break;
   	}
   	
   	//clear otp buffer
   	OV8858S_write_cmos_sensor(0x703a,0x00); 
   	
		if(flag ==0x00)
			{
				return 0;
			}	
			else if(flag & 0x02)
				{
					return 1;
				}
				else
					{
						return 2;
					}
}

//index:index of otp group.(0,1,2)
//return: 0
kal_uint16 OV8858S_mipi_read_otp_wb(kal_uint16 index, struct OV8858S_MIPI_otp_struct *otp)
{
    kal_uint16 i=0,temp=0;
    kal_uint16 start_addr=0,end_addr=0;
	  
    switch(index)
    {
    	case 1:
    		start_addr=0x7021;
    		end_addr  =0x7025;
    		break;
    	case 2:
    		start_addr=0x7026;
    		end_addr  =0x702a;
    		break;
    	case 3:
    		start_addr=0x702b;
    		end_addr  =0x702f;
    		break;
    	default:
    		OV8858SDB("[OV8858S_MIPI_read_otp_wb] index error\n");
    		break;
    }
    
    
    OV8858S_write_cmos_sensor(0x3d84,0xc0);
   	//partial mode otp write start address
    OV8858S_write_cmos_sensor(0x3d88,(start_addr>>8)&0xff);
    OV8858S_write_cmos_sensor(0x3d89,start_addr & 0xff);
    //partial mode otp write end address
    OV8858S_write_cmos_sensor(0x3d8a,(end_addr>>8)&0xff);
    OV8858S_write_cmos_sensor(0x3d8b,end_addr & 0xff);
    
    //read otp into buffer
    OV8858S_write_cmos_sensor(0x3d81,0x01);
    
    mdelay(10);	
    temp =OV8858S_read_cmos_sensor(start_addr +4);
    
    otp->rg_ratio=(OV8858S_read_cmos_sensor(start_addr)<<2)+((temp>>6)&0x03);
    otp->bg_ratio=(OV8858S_read_cmos_sensor(start_addr+1)<<2)+((temp>>4)&0x03);
 #if 1
	OV8858SDB("[wujh_0616][OV8858S_MIPI_read_otp_wb] rg_ratio=%x\n",otp->rg_ratio);
       OV8858SDB("[wujh_0616][OV8858S_MIPI_read_otp_wb] bg_ratio=%x\n",otp->bg_ratio);
 #endif

    otp->light_rg=0;
    otp->light_bg=0;
    
		//clear otp buffer
		for(i=start_addr;i<end_addr;i++)
		{
			OV8858S_write_cmos_sensor(i,0x00);
		}
		
		return 0;
}

kal_uint16 OV8858S_mipi_read_otp_lenc(kal_uint16 index,struct OV8858S_MIPI_otp_struct *otp)
{
	  kal_uint16 i=0,temp=0;
    kal_uint16 start_addr=0,end_addr=0;
	 
	 switch(index)
	 {
	 	case 1:
	 			start_addr =0x703b;
	 			end_addr=0x70a8;
	 			break;
	 	case 2:
	 		  start_addr =0x70a9;
	 			end_addr=0x7116;
	 			break;
	 	case 3:
	 			start_addr =0x7117;
	 			end_addr=0x7184;
	 			break;
	 	default :
	 			OV8858SDB("[OV8858S_mipi_read_otp_lenc error [wangc_test]]");
	 			break;
	  }
	  
	  OV8858S_write_cmos_sensor(0x3d84,0xc0);
	  //partial mode otp write start address
    OV8858S_write_cmos_sensor(0x3d88,(start_addr>>8)&0xff);
    OV8858S_write_cmos_sensor(0x3d89,start_addr & 0xff);
    //partial mode otp write end address
    OV8858S_write_cmos_sensor(0x3d8a,(end_addr>>8)&0xff);
    OV8858S_write_cmos_sensor(0x3d8b,end_addr & 0xff);
    
    //read otp into buffer
    OV8858S_write_cmos_sensor(0x3d81,0x01);
	  mdelay(10);	
	  
	  for(i=0;i<110;i++)
	  {
	  	otp->lenc[i] = OV8858S_read_cmos_sensor(start_addr+i);
	  }
  	
  	//clear otp buffer
  	for(i=start_addr;i<=end_addr;i++)
  	{
  		OV8858S_write_cmos_sensor(i,0x00);
  	}
	  
	return 1;
}

//R_gain: red gain of sensor AWB, 0x400 = 1
//G_gain: green gain of sensor AWB, 0x400 = 1
//B_gain: blue gain of sensor AWB, 0x400 = 1
//reutrn 0
void OV8858S_mipi_update_wb_gain(kal_uint32 R_gain, kal_uint32 G_gain, kal_uint32 B_gain)
{   
	if(R_gain > 0x400)
		{
			OV8858S_write_cmos_sensor(0x5032,R_gain >> 8);
			OV8858S_write_cmos_sensor(0x5033,(R_gain&0x00ff));
		}
	if(G_gain > 0x400)
		{
			OV8858S_write_cmos_sensor(0x5034,G_gain >> 8);
			OV8858S_write_cmos_sensor(0x5035,(G_gain&0x00ff));
		}
	if(B_gain >0x400)
		{
			OV8858S_write_cmos_sensor(0x5036,B_gain >> 8);
			OV8858S_write_cmos_sensor(0x5037,(B_gain&0x00ff));
		}
	OV8858SDB("[OV8858S_MIPI_update_wb_gain]R_gain[%x]G_gain[%x]B_gain[%x]\n",R_gain,G_gain,B_gain);
	OV8858SDB("[OV8858S_MIPI_update_wb_gain_Finished[wangc_test]]");
}

void OV8858S_mipi_update_lenc(struct OV8858S_MIPI_otp_struct *otp)
{
    kal_uint16 i=0, temp=0;
    
    temp=OV8858S_read_cmos_sensor(0x5000);
    temp =temp | 0x80;
  //  OV8858S_write_cmos_sensor(0x5800,temp);
	OV8858S_write_cmos_sensor(0x5000,temp);

    
    for(i = 0; i < 110; i++)
    {
       OV8858S_write_cmos_sensor(0x5800+i,otp->lenc[i]);
       OV8858SDB("[OV8858S_MIPI_update_lenc]otp->lenc[%d][%x]\n",i,otp->lenc[i]);
     }
    OV8858SDB("[OV8858S_MIPI_update_lenc_Finished]\n");

}

//call this function after OV5650 initialization
//return value:	0 update success
//				1 no 	OTP
kal_uint16 OV8858S_mipi_update_wb_register_from_otp(void)
{
	kal_uint16 temp=0, i=0, otp_index=0,rg=0,bg=0;
	kal_uint32 awb_rgb_gain[2]={0};
	kal_uint32 R_gain=0, B_gain=0, G_gain=0;
	OV8858SDB("OV8858S_MIPI_update_wb_register_from_otp_Start[wangc_test]\n");
	//check first wb OTP with valid OTP
//Gionee yanggy add for CAM Performance Optimization begin
#ifdef ORIGINAL_VERSION 
	for(i = 1; i <= 3; i++)
		{
			temp = OV8858S_mipi_check_otp_wb(i);
			if(temp == 2)
				{
					otp_index = i;
					break;
				}
		}
	if( i >4)//wangc modified
		{
		 	OV8858SDB("[OV8858S_mipi_update_wb_register_from_otp] OTP AWB ERROR\r\n");
			return 0;//wangc_test_OV8858S_mipi_update_wb_register_from_otp invalid
		}
	/*	
	if(!OV8858S_mipi_read_otp_wb(otp_index,&current_otp))//wangc added for checking read_otp_wb 
		{
			return 0;//wangc_test_OV8858S_mipi_update_wb_register_from_otp invalid
		}       
		*/

    OV8858S_mipi_read_otp_wb(otp_index,&current_otp);
#else
    if( true == isNeedReadOtp)
    {
        for(i = 1; i <= 3; i++)
        {
            temp = OV8858S_mipi_check_otp_wb(i);
            if(temp == 2)
            {
                otp_index = i;
                break;
            }
        }
        if( i >4)//wangc modified
        {
            OV8858SDB("[OV8858S_mipi_update_wb_register_from_otp] OTP AWB ERROR\r\n");
            return 0;//wangc_test_OV8858S_mipi_update_wb_register_from_otp invalid
        }
        OV8858S_mipi_read_otp_wb(otp_index,&current_otp);
        s_otp_wb = current_otp;
    }
    else 
    {
        current_otp = s_otp_wb;
    }
#endif
    //Gionee yanggy add for CAM Performance Optimization end	
	//calculate gain
	//0x400 = 1x gain
	if(current_otp.light_rg==0)
		{
		//if no light source information in OTP,light factor=1;
		rg=current_otp.rg_ratio;
	      }
	else
		{
		rg = current_otp.rg_ratio * (current_otp.light_rg +512) / 1024;   
		}

	if(current_otp.light_bg==0)
		{
		//if no light source information in OTP,light factor=1;
				bg=current_otp.bg_ratio;
	  }
	else
		{
        bg = current_otp.bg_ratio * (current_otp.light_bg +512) / 1024;  
		}
	//calculate G gain	
	awb_rgb_gain[0] =RG_Ratio_typical*1024/rg;
	awb_rgb_gain[1] =BG_Ratio_typical*1024/bg;
	
	if((awb_rgb_gain[0]<1024)||(awb_rgb_gain[1]<1024))
		{
			if(awb_rgb_gain[0]<awb_rgb_gain[1])
				{
					R_gain = 0x400;
					G_gain = R_gain * rg / RG_Ratio_typical;
					B_gain = G_gain * BG_Ratio_typical /bg;
				}
			else
				{
					B_gain = 0x400;
					G_gain = B_gain * bg / BG_Ratio_typical;
					R_gain = G_gain * RG_Ratio_typical /rg;
				}

		}
	else
		{
			G_gain = 0x400;
			R_gain = G_gain * RG_Ratio_typical / rg;
			B_gain = G_gain * BG_Ratio_typical /bg;
		}
	//write sensor wb gain to register
	OV8858SDB("[R_gain=%x,G_gain=%x,B_Gain=%x",R_gain,G_gain,B_gain);
	OV8858S_mipi_update_wb_gain(R_gain,G_gain,B_gain);
	OV8858SDB("[OV8858S_mipi_update_wb_register_from_otp_Finished[wangc_test]\n]");
    return 1;//wangc_test_OV8858S_mipi_update_wb_register_from_otp invalid
}

//call this function after ov5650 initialization
//return value:	0 update success
//				1 no otp

kal_uint16 OV8858S_mipi_update_lenc_register_from_otp(void)
{
	   kal_uint16 temp,i,otp_index;

    OV8858SDB("OV8858S_mipi_update_lenc_register_from_otp_Start[wangc_test]\n");
//Gionee yanggy add for CAM Performance Optimization begin
#ifdef ORIGINAL_VERSION
    for(i = 1; i<4;i++)
	{
		temp = OV8858S_mipi_check_otp_lenc(i);
		if(2 == temp)
		{
			otp_index = i;
			break;
		}
	}
	if(i == 4) //wangc modified
		{
		 	OV8858SDB("[OV8858S_mipi_update_lenc_register_from_otp]no valid wb OTP data!\r\n");
			return 0;//wangc_test_OV8858S_mipi_update_lenc_register_from_otp invalid
        }
    if(!OV8858S_mipi_read_otp_lenc(otp_index,&current_otp))
    {
        return 0;//wangc_test_OV8858S_mipi_update_lenc_register_from_otp invalid
    }
#else
    if(true == isNeedReadOtp)
    {
        for(i = 1; i<4;i++)
        {
            temp = OV8858S_mipi_check_otp_lenc(i);
            if(2 == temp)
            {
                otp_index = i;
                break;
            }
        }
        if(i == 4) //wangc modified
        {
            OV8858SDB("[OV8858S_mipi_update_lenc_register_from_otp]no valid wb OTP data!\r\n");
            return 0;//wangc_test_OV8858S_mipi_update_lenc_register_from_otp invalid
        }
        if(!OV8858S_mipi_read_otp_lenc(otp_index,&current_otp))
        {
            return 0;//wangc_test_OV8858S_mipi_update_lenc_register_from_otp invalid
        }
        s_otp_lenc =  current_otp;
        isNeedReadOtp = false;
    }
    else
    {
        current_otp = s_otp_lenc;
    }
#endif
    //Gionee yanggy add for CAM Performance Optimization end
	OV8858S_mipi_update_lenc(&current_otp);
	OV8858SDB("OV8858S_mipi_update_lenc_register_from_otp_Finished[wangc_test]\n");
	return 1;// wangc_test_OV8858S_mipi_update_lenc_register_from_otp success
}

#endif

/*****************************  OTP Feature  End**********************************/



//#endif //OV8858S otp end



//Gionee yanggy 2013-09-03 add for module compability end




void OV8858S_Init_Para(void)
{

	spin_lock(&OV8858Smipiraw_drv_lock);
	OV8858S.sensorMode = SENSOR_MODE_INIT;
	OV8858S.OV8858SAutoFlickerMode = KAL_FALSE;
	OV8858S.OV8858SVideoMode = KAL_FALSE;
	OV8858S.DummyLines= 0;
	OV8858S.DummyPixels= 0;
	OV8858S.pvPclk =  (7440);
	OV8858S.videoPclk = (14880);
	OV8858S.capPclk = (14880);

	OV8858S.shutter = 0x4C00;
	OV8858S.ispBaseGain = BASEGAIN;		//64
	OV8858S.sensorGlobalGain = 0x0200;  //512
	spin_unlock(&OV8858Smipiraw_drv_lock);
}

kal_uint32 GetOV8858SLineLength(void)
{
	kal_uint32 OV8858S_line_length = 0;
	if ( SENSOR_MODE_PREVIEW == OV8858S.sensorMode )  
	{
		OV8858S_line_length = OV8858S_PV_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels;
	}
	else if( SENSOR_MODE_VIDEO == OV8858S.sensorMode ) 
	{
		OV8858S_line_length = OV8858S_VIDEO_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels;
	}
	else
	{
		OV8858S_line_length = OV8858S_FULL_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels;
	}
	
#ifdef OV8858S_DEBUG
	OV8858SDB("[GetOV8858SLineLength]: OV8858S.sensorMode = %d, OV8858S_line_length =%d, OV8858S.DummyPixels = %d\n", OV8858S.sensorMode,OV8858S_line_length,OV8858S.DummyPixels);
#endif


    return OV8858S_line_length;

}


kal_uint32 GetOV8858SFrameLength(void)
{
	kal_uint32 OV8858S_frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == OV8858S.sensorMode )  
	{
		OV8858S_frame_length = OV8858S_PV_PERIOD_LINE_NUMS + OV8858S.DummyLines ;
	}
	else if( SENSOR_MODE_VIDEO == OV8858S.sensorMode ) 
	{
		OV8858S_frame_length = OV8858S_VIDEO_PERIOD_LINE_NUMS + OV8858S.DummyLines ;
	}
	else
	{
		OV8858S_frame_length = OV8858S_FULL_PERIOD_LINE_NUMS + OV8858S.DummyLines ;
	}

#ifdef OV8858S_DEBUG
		OV8858SDB("[GetOV8858SFrameLength]: OV8858S.sensorMode = %d, OV8858S_frame_length =%d, OV8858S.DummyLines = %d\n", OV8858S.sensorMode,OV8858S_frame_length,OV8858S.DummyLines);
#endif


	return OV8858S_frame_length;
}


kal_uint32 OV8858S_CalcExtra_For_ShutterMargin(kal_uint32 shutter_value,kal_uint32 shutterLimitation)
{
    kal_uint32 extra_lines = 0;

	
	if (shutter_value <4 ){
		shutter_value = 4;
	}

	
	if (shutter_value > shutterLimitation)
	{
		extra_lines = shutter_value - shutterLimitation;
    }
	else
		extra_lines = 0;

#ifdef OV8858S_DEBUG
			OV8858SDB("[OV8858S_CalcExtra_For_ShutterMargin]: shutter_value = %d, shutterLimitation =%d, extra_lines = %d\n", shutter_value,shutterLimitation,extra_lines);
#endif

    return extra_lines;

}


//TODO~
kal_uint32 OV8858S_CalcFrameLength_For_AutoFlicker(void)
{

    kal_uint32 AutoFlicker_min_framelength = 0;

	switch(OV8858SCurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			AutoFlicker_min_framelength = (OV8858S.capPclk*10000) /(OV8858S_FULL_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels)/OV8858S_AUTOFLICKER_OFFSET_30*10 ;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(OV8858S_VIDEO_MODE_TARGET_FPS==30)
			{
				AutoFlicker_min_framelength = (OV8858S.videoPclk*10000) /(OV8858S_VIDEO_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels)/OV8858S_AUTOFLICKER_OFFSET_30*10 ;
			}
			else if(OV8858S_VIDEO_MODE_TARGET_FPS==15)
			{
				AutoFlicker_min_framelength = (OV8858S.videoPclk*10000) /(OV8858S_VIDEO_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels)/OV8858S_AUTOFLICKER_OFFSET_15*10 ;
			}
			else
			{
				AutoFlicker_min_framelength = OV8858S_VIDEO_PERIOD_LINE_NUMS + OV8858S.DummyLines;
			}
			break;
			
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			AutoFlicker_min_framelength = (OV8858S.pvPclk*10000) /(OV8858S_PV_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels)/OV8858S_AUTOFLICKER_OFFSET_30*10 ;
			break;
	}

	#ifdef OV8858S_DEBUG 
	OV8858SDB("AutoFlicker_min_framelength =%d,OV8858SCurrentScenarioId =%d\n", AutoFlicker_min_framelength,OV8858SCurrentScenarioId);
	#endif

	return AutoFlicker_min_framelength;

}


void OV8858S_write_shutter(kal_uint32 shutter)
{
#if 0
	//kal_uint32 min_framelength = OV8858S_PV_PERIOD_PIXEL_NUMS;
	//the init code write as up line;
	//modify it as follow
	kal_uint32 min_framelength = OV8858S_PV_PERIOD_LINE_NUMS;
	kal_uint32 max_shutter=0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0, framerate=0;
	unsigned long flags;
//Gionee <wujh><2014-06-05> modify for CR01284824 <fix frame rate in 25fps>  begin
#ifdef ORIGINAL_VERSION
#else 
	kal_uint16 realtime_fp = 0;
#endif
//Gionee <wujh><2014-06-05> modify for CR01284824 <fix frame rate in 25fps>  end
	//TODO~
	kal_uint32 read_shutter_1 = 0;
	kal_uint32 read_shutter_2 = 0;
	kal_uint32 read_shutter_3 = 0;

	//TODO~
    if(shutter > 0x90f7)//500ms for capture SaturationGain
    {
    	#ifdef OV8858S_DEBUG
		OV8858SDB("[OV8858S_write_shutter] shutter > 0x90f7 [warn.] shutter=%x, \n", shutter);
		#endif
		shutter = 0x90f7;
    }
	
    line_length  = GetOV8858SLineLength();
	frame_length = GetOV8858SFrameLength();
	
	max_shutter  = frame_length-OV8858S_SHUTTER_MARGIN;

    frame_length = frame_length + OV8858S_CalcExtra_For_ShutterMargin(shutter,max_shutter);
	


	if(OV8858S.OV8858SAutoFlickerMode == KAL_TRUE)
	{
        min_framelength = OV8858S_CalcFrameLength_For_AutoFlicker();

        if(frame_length < min_framelength)
			frame_length = min_framelength;
#if 0
		if (OV8858S.sensorMode == SENSOR_MODE_PREVIEW)
		{
			framerate = (10 * OV8858S.pvPclk) / (frame_length * line_length);
			if(framerate>=280)
			{
				framerate=280;
				frame_length = (10 * OV8858S_PREVIEW_PCLK) / (framerate * line_length);
			}
			else if(framerate>=220&&framerate<280)
			{
				framerate=220;
				frame_length = (10 * OV8858S_PREVIEW_PCLK) / (framerate * line_length);
			}
			else if(framerate>=183&&framerate<220)
			{
				framerate=183;
				frame_length = (10 * OV8858S_PREVIEW_PCLK) / (framerate * line_length);
			}
			else if(framerate>=157&&framerate<183)
			{
				framerate=157;
				frame_length = (10 * OV8858S_PREVIEW_PCLK) / (framerate * line_length);
			}
			else if(framerate>=138&&framerate<157)
			{
				framerate=138;
				frame_length = (10 * OV8858S_PREVIEW_PCLK) / (framerate * line_length);
			}
			else if(framerate>=129&&framerate<138)
			{
				framerate=129;
				frame_length = (10 * OV8858S_PREVIEW_PCLK) / (framerate * line_length);
			}
			else if(framerate>=116&&framerate<129)
			{
				framerate=116;
				frame_length = (10 * OV8858S_PREVIEW_PCLK) / (framerate * line_length);
			}
			else if(framerate>=105&&framerate<116)
			{
				framerate=105;
				frame_length = (10 * OV8858S_PREVIEW_PCLK) / (framerate * line_length);
			}
			else if(framerate>=96&&framerate<105)
			{
				framerate=96;
				frame_length = (10 * OV8858S_PREVIEW_PCLK) / (framerate * line_length);
			}
			else if(framerate<96)
			{			 
				frame_length = (10 * OV8858S_PREVIEW_PCLK) / (framerate * line_length);
			}
		}
		else if(OV8858S.sensorMode == SENSOR_MODE_VIDEO)
		{
			framerate = (10 * OV8858S.videoPclk) / (frame_length * line_length);
			if(framerate>=280)
			{
				framerate=280;
				frame_length = (10 * OV8858S_VIDEO_PCLK) / (framerate * line_length);
			}
			else if(framerate>=220&&framerate<280)
			{
				framerate=220;
				frame_length = (10 * OV8858S_VIDEO_PCLK) / (framerate * line_length);
			}
			else if(framerate>=183&&framerate<220)
			{
				framerate=183;
				frame_length = (10 * OV8858S_VIDEO_PCLK) / (framerate * line_length);
			}
			else if(framerate>=157&&framerate<183)
			{
				framerate=157;
				frame_length = (10 * OV8858S_VIDEO_PCLK) / (framerate * line_length);
			}
			else if(framerate>=138&&framerate<157)
			{
				framerate=138;
				frame_length = (10 * OV8858S_VIDEO_PCLK) / (framerate * line_length);
			}
			else if(framerate>=129&&framerate<138)
			{
				framerate=129;
				frame_length = (10 * OV8858S_VIDEO_PCLK) / (framerate * line_length);
			}
			else if(framerate>=116&&framerate<129)
			{
				framerate=116;
				frame_length = (10 * OV8858S_VIDEO_PCLK) / (framerate * line_length);
			}
			else if(framerate>=105&&framerate<116)
			{
				framerate=105;
				frame_length = (10 * OV8858S_VIDEO_PCLK) / (framerate * line_length);
			}
			else if(framerate>=96&&framerate<105)
			{
				framerate=96;
				frame_length = (10 * OV8858S_VIDEO_PCLK) / (framerate * line_length);
			}
			else if(framerate<96)
			{			 
				frame_length = (10 * OV8858S_VIDEO_PCLK) / (framerate * line_length);
			}
		}
		else
#endif	
        if (OV8858S.sensorMode == SENSOR_MODE_CAPTURE)
		{
			framerate = (10 * OV8858S_CAPTURE_PCLK) / (frame_length * line_length);

			if(framerate>=280)
			{
				framerate=280;
				frame_length = (10 * OV8858S_CAPTURE_PCLK) / (framerate * line_length);
			}
			else if(framerate>=220&&framerate<280)
			{
				framerate=220;
				frame_length = (10 * OV8858S_CAPTURE_PCLK) / (framerate * line_length);
			}
			else if(framerate>=183&&framerate<220)
			{
				framerate=183;
				frame_length = (10 * OV8858S_CAPTURE_PCLK) / (framerate * line_length);
			}
			else if(framerate>=157&&framerate<183)
			{
				framerate=157;
				frame_length = (10 * OV8858S_CAPTURE_PCLK) / (framerate * line_length);
			}
			else if(framerate>=138&&framerate<157)
			{
				framerate=138;
				frame_length = (10 * OV8858S_CAPTURE_PCLK) / (framerate * line_length);
			}
			else if(framerate>=129&&framerate<138)
			{
				framerate=129;
				frame_length = (10 * OV8858S_CAPTURE_PCLK) / (framerate * line_length);
			}
			else if(framerate>=116&&framerate<129)
			{
				framerate=116;
				frame_length = (10 * OV8858S_CAPTURE_PCLK) / (framerate * line_length);
			}
			else if(framerate>=105&&framerate<116)
			{
				framerate=105;
				frame_length = (10 * OV8858S_CAPTURE_PCLK) / (framerate * line_length);
			}
			else if(framerate>=96&&framerate<105)
			{
				framerate=96;
				frame_length = (10 * OV8858S_CAPTURE_PCLK) / (framerate * line_length);
			}
			else if(framerate<96)
			{			 
				frame_length = (10 * OV8858S_CAPTURE_PCLK) / (framerate * line_length);
			}
		}
	
	}
	
	spin_lock_irqsave(&OV8858Smipiraw_drv_lock,flags);
	OV8858S_FeatureControl_PERIOD_PixelNum = line_length;
	OV8858S_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock_irqrestore(&OV8858Smipiraw_drv_lock,flags);

#if 0
//Gionee <wujh><2014-06-05> modify for CR01284824<fix frame rate at 25fps>  begin
#ifdef ORIGINAL_VERSION
#else 
	if (OV8858S.sensorMode==SENSOR_MODE_CAPTURE)
		{
		OV8858SDB("[wujh_test]Preview mode\n");
		  if(shutter< (OV8858S_CAPTURE_PCLK/(OV8858S_FULL_PERIOD_PIXEL_NUMS+50)/100))
            		{            
	
				OV8858SDB("[wujh_test]shutter<0x16f\n");
				realtime_fp=250;
	      			spin_lock_irqsave(&OV8858Smipiraw_drv_lock,flags);
             			frame_length = OV8858S_CAPTURE_PCLK *10 / (OV8858S_FULL_PERIOD_PIXEL_NUMS * realtime_fp);
	      			spin_unlock_irqrestore(&OV8858Smipiraw_drv_lock,flags);
               		OV8858SDB("[realtime_fp=250][height:%d]",frame_length);
             		}
		}
#endif
//Gionee <wujh><2014-06-05> modify for CR01284824 <fix frame rate at 25fps>  end
#endif

	//Set total frame length
	OV8858S_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
	OV8858S_write_cmos_sensor(0x380f, frame_length & 0xFF);
	
	//Set shutter 
	OV8858S_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
	OV8858S_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
	OV8858S_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	

	#ifdef OV8858S_DEBUG
	OV8858SDB("[OV8858S_write_shutter]OV8858S write shutter=%x, line_length=%x, frame_length=%x\n", shutter, line_length, frame_length);
	#endif
#else
    //kal_uint32 min_framelength = OV8858S_PV_PERIOD_PIXEL_NUMS;
    //the init code write as up line;
    //modify it as follow
    kal_uint32 min_framelength = OV8858S_PV_PERIOD_LINE_NUMS;
    kal_uint32 max_shutter=0;
    kal_uint32 line_length = 0;
    kal_uint32 frame_length = 0, framerate=0;
    unsigned long flags;
    
    line_length  = GetOV8858SLineLength();
    frame_length = GetOV8858SFrameLength();
    
    max_shutter  = frame_length-OV8858S_SHUTTER_MARGIN;
    
    frame_length = frame_length + OV8858S_CalcExtra_For_ShutterMargin(shutter,max_shutter);
    
    if(OV8858S.OV8858SAutoFlickerMode == KAL_TRUE)
    {
        min_framelength = OV8858S_CalcFrameLength_For_AutoFlicker();
        OV8858SDB("[OV8858S_write_shutter] min_framelength=%d, line_length=%d, frame_length=%d \n", min_framelength, line_length, frame_length);
        if(frame_length < min_framelength)
            frame_length = min_framelength;
    }
    
    spin_lock_irqsave(&OV8858Smipiraw_drv_lock,flags);
    OV8858S_FeatureControl_PERIOD_PixelNum = line_length;
    OV8858S_FeatureControl_PERIOD_LineNum = frame_length;
    spin_unlock_irqrestore(&OV8858Smipiraw_drv_lock,flags);
    
    //Set total frame length
    OV8858S_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
    OV8858S_write_cmos_sensor(0x380f, frame_length & 0xFF);
    
    //Set shutter 
    OV8858S_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
    OV8858S_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
    OV8858S_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0); 
    
    #ifdef OV8858S_DEBUG
    OV8858SDB("[OV8858S_write_shutter]OV8858S write shutter=%x, line_length=%x, frame_length=%x\n", shutter, line_length, frame_length);
    #endif
#endif
}


void OV8858S_SetShutter(kal_uint32 iShutter)
{

   spin_lock(&OV8858Smipiraw_drv_lock);
   OV8858S.shutter= iShutter;
   spin_unlock(&OV8858Smipiraw_drv_lock);

   OV8858S_write_shutter(iShutter);
   return;
}


UINT32 OV8858S_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	UINT32 shutter =0;
	temp_reg1 = OV8858S_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV8858S_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV8858S_read_cmos_sensor(0x3502);    // AEC[b7~b0]
	
	shutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);

	return shutter;
}

static kal_uint16 OV8858SReg2Gain(const kal_uint16 iReg)
{
    kal_uint16 iGain =0; 

	iGain = iReg*BASEGAIN/OV8858S_GAIN_BASE;
	return iGain;
}

static kal_uint16 OV8858SGain2Reg(const kal_uint32 iGain)
{
    kal_uint32 iReg = 0x0000;

	iReg = iGain * 2; //(iGain/BASEGAIN)*OV8858S_GAIN_BASE;

    return iReg;
}

void write_OV8858S_gain(kal_uint16 gain)
{
	//kal_uint16 read_gain=0;

	OV8858S_write_cmos_sensor(0x3508,(gain>>8));
	OV8858S_write_cmos_sensor(0x3509,(gain&0xff));

	//read_gain=(((OV8858S_read_cmos_sensor(0x3508)&0x1F) << 8) | OV8865_read_cmos_sensor(0x3509));
	//OV8858SDB("[OV8858S_SetGain]0x3508|0x3509=0x%x \n",read_gain);

	return;
}

void OV8858S_SetGain(UINT16 iGain)
{
	unsigned long flags;

	
	OV8858SDB("OV8858S_SetGain iGain = %d :\n ",iGain);

	spin_lock_irqsave(&OV8858Smipiraw_drv_lock,flags);
	OV8858S.realGain = iGain;
	OV8858S.sensorGlobalGain = OV8858SGain2Reg(iGain);
	spin_unlock_irqrestore(&OV8858Smipiraw_drv_lock,flags);
	write_OV8858S_gain(OV8858S.sensorGlobalGain);
	#ifdef OV8858S_DEBUG
	OV8858SDB(" [OV8858S_SetGain]OV8858S.sensorGlobalGain=0x%x,OV8858S.realGain =0x%x",OV8858S.sensorGlobalGain,OV8858S.realGain); 
	#endif
	//temperature test
	//OV8858S_write_cmos_sensor(0x4d12,0x01);
	//OV8858SDB("Temperature read_reg  0x4d13  =%x \n",OV8865_read_cmos_sensor(0x4d13));
}   

kal_uint16 read_OV8858S_gain(void)
{
	kal_uint16 read_gain=0;

	read_gain=(((OV8858S_read_cmos_sensor(0x3508)&0x1F) << 8) | OV8858S_read_cmos_sensor(0x3509));

	spin_lock(&OV8858Smipiraw_drv_lock);
	OV8858S.sensorGlobalGain = read_gain;
	OV8858S.realGain = OV8858SReg2Gain(OV8858S.sensorGlobalGain);
	spin_unlock(&OV8858Smipiraw_drv_lock);

	OV8858SDB("OV8858S.sensorGlobalGain=0x%x,OV8858S.realGain=%d\n",OV8858S.sensorGlobalGain,OV8858S.realGain);

	return OV8858S.sensorGlobalGain;
}  



static void OV8858S_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == OV8858S.sensorMode )
	{
		line_length = OV8858S_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8858S_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== OV8858S.sensorMode )
	{
		line_length = OV8858S_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8858S_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else
	{
		line_length = OV8858S_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8858S_FULL_PERIOD_LINE_NUMS + iLines;
	}

	spin_lock(&OV8858Smipiraw_drv_lock);
	OV8858S_FeatureControl_PERIOD_PixelNum = line_length;
	OV8858S_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&OV8858Smipiraw_drv_lock);

	//Set total frame length
	OV8858S_write_cmos_sensor(0x380e, (frame_length >> 8) & 0xFF);
	OV8858S_write_cmos_sensor(0x380f, frame_length & 0xFF);
	//Set total line length
	OV8858S_write_cmos_sensor(0x380c, (line_length >> 8) & 0xFF);
	OV8858S_write_cmos_sensor(0x380d, line_length & 0xFF);

	#ifdef OV8858S_DEBUG
	OV8858SDB(" [OV8858S_SetDummy]OV8858S.sensorMode = %d, line_length = %d,iPixels = %d, frame_length =%d, iLines = %d\n",OV8858S.sensorMode, line_length,iPixels, frame_length, iLines); 
	#endif

}   


#if 1
void OV8858S_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=OV8858SSensorReg[i].Addr; i++)
    {
        OV8858S_write_cmos_sensor(OV8858SSensorReg[i].Addr, OV8858SSensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV8858SSensorReg[i].Addr; i++)
    {
        OV8858S_write_cmos_sensor(OV8858SSensorReg[i].Addr, OV8858SSensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        OV8858S_write_cmos_sensor(OV8858SSensorCCT[i].Addr, OV8858SSensorCCT[i].Para);
    }
}

void OV8858S_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=OV8858SSensorReg[i].Addr; i++)
    {
         temp_data = OV8858S_read_cmos_sensor(OV8858SSensorReg[i].Addr);
		 spin_lock(&OV8858Smipiraw_drv_lock);
		 OV8858SSensorReg[i].Para =temp_data;
		 spin_unlock(&OV8858Smipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV8858SSensorReg[i].Addr; i++)
    {
        temp_data = OV8858S_read_cmos_sensor(OV8858SSensorReg[i].Addr);
		spin_lock(&OV8858Smipiraw_drv_lock);
		OV8858SSensorReg[i].Para = temp_data;
		spin_unlock(&OV8858Smipiraw_drv_lock);
    }
}

kal_int32  OV8858S_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void OV8858S_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
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

void OV8858S_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
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

            temp_para= OV8858SSensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/ov8865.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= OV8858S_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= OV8858S_MAX_ANALOG_GAIN * 1000;
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
                    info_ptr->ItemValue=    111;  
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



kal_bool OV8858S_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
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
//             temp_para=(temp_gain * ov8865.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

		  spin_lock(&OV8858Smipiraw_drv_lock);
          OV8858SSensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&OV8858Smipiraw_drv_lock);
          OV8858S_write_cmos_sensor(OV8858SSensorCCT[temp_addr].Addr,temp_para);

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
					spin_lock(&OV8858Smipiraw_drv_lock);
                    OV8858S_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&OV8858Smipiraw_drv_lock);
                    break;
                case 1:
                    OV8858S_write_cmos_sensor(OV8858S_FAC_SENSOR_REG,ItemValue);
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
#endif

void OV8858S_1224pSetting(void)
{

	/*   Preview of OV8858S setting                                 */
	/*   @@5.1.2.2 Raw 10bit 1632x1224 30fps 2lane 720M bps/lane   */
	/*   ;Pclk 72MHz                                               */
	/*   ;pixels per line=1928(0x788)                              */
	/*   ;lines per frame=1244(0x4dc)                              */
	OV8858S_write_cmos_sensor(0x0100, 0x00); //
	OV8858S_write_cmos_sensor(0x030e, 0x00); // ; pll2_rdiv
	OV8858S_write_cmos_sensor(0x030f, 0x09); // ; pll2_divsp
	OV8858S_write_cmos_sensor(0x0312, 0x01); // ; pll2_pre_div0, pll2_r_divdac
	OV8858S_write_cmos_sensor(0x3015, 0x01); //
	OV8858S_write_cmos_sensor(0x3501, 0x4d); // ; exposure M
	OV8858S_write_cmos_sensor(0x3502, 0x40); // ; exposure L
	OV8858S_write_cmos_sensor(0x3508, 0x04); // ; gain H
	OV8858S_write_cmos_sensor(0x3706, 0x35); //
	OV8858S_write_cmos_sensor(0x370a, 0x00); //
	OV8858S_write_cmos_sensor(0x370b, 0xb5); //
	OV8858S_write_cmos_sensor(0x3778, 0x1b); //
	OV8858S_write_cmos_sensor(0x3808, 0x06); // ; x output size H 1632 
	OV8858S_write_cmos_sensor(0x3809, 0x60); // ; x output size L
	OV8858S_write_cmos_sensor(0x380a, 0x04); // ; y output size H 1224
	OV8858S_write_cmos_sensor(0x380b, 0xc8); // ; y output size L
	OV8858S_write_cmos_sensor(0x380c, 0x07); // ; HTS H
	OV8858S_write_cmos_sensor(0x380d, 0x88); // ; HTS L
	OV8858S_write_cmos_sensor(0x380e, 0x04); // ; VTS H
	OV8858S_write_cmos_sensor(0x380f, 0xdc); // ; VTS L
	OV8858S_write_cmos_sensor(0x3814, 0x03); // ; x odd inc
	OV8858S_write_cmos_sensor(0x3821, 0x61); // ; mirror on, bin on
	OV8858S_write_cmos_sensor(0x382a, 0x03); // ; y odd inc
	OV8858S_write_cmos_sensor(0x3830, 0x08); //
	OV8858S_write_cmos_sensor(0x3836, 0x02); //
	OV8858S_write_cmos_sensor(0x3f0a, 0x80); //
	OV8858S_write_cmos_sensor(0x4001, 0x10); // ; total 128 black column
	OV8858S_write_cmos_sensor(0x4022, 0x04); // ; Anchor left end H
	OV8858S_write_cmos_sensor(0x4023, 0xb9); // ; Anchor left end L
	OV8858S_write_cmos_sensor(0x4024, 0x05); // ; Anchor right start H
	OV8858S_write_cmos_sensor(0x4025, 0x2a); // ; Anchor right start L
	OV8858S_write_cmos_sensor(0x4026, 0x05); // ; Anchor right end H
	OV8858S_write_cmos_sensor(0x4027, 0x2b); // ; Anchor right end L
	OV8858S_write_cmos_sensor(0x402b, 0x04); // ; top black line number
	OV8858S_write_cmos_sensor(0x402e, 0x08); // ; bottom black line start
	OV8858S_write_cmos_sensor(0x4500, 0x38); //
	OV8858S_write_cmos_sensor(0x4600, 0x00); //
	OV8858S_write_cmos_sensor(0x4601, 0xcb); //
	OV8858S_write_cmos_sensor(0x382d, 0x7f); //
	OV8858S_write_cmos_sensor(0x0100, 0x01); //
}

void OV8858SPreviewSetting(void)
{
	// ++++++++  @@5.1.1.2 Raw 10bit 1632x1224 30fps 4lane 672M bps/lane
	// ++++++++  ;;
	// ++++++++  ;; MIPI=672Mbps, SysClk=72Mhz,Dac Clock=360Mhz.
	// ++++++++  ;;
	// ++++++++  ;Pclk 72MHz
	// ++++++++  ;pixels per line=1928(0x788) 
	// ++++++++  ;lines per frame=1244(0x4dc)
		//OV8858S_write_cmos_sensor( 0x0100, 0x00);  //
		//OV8858S_write_cmos_sensor( 0x030f, 0x09);  // ; pll2_divsp
		//OV8858S_write_cmos_sensor( 0x3501, 0x4d);  // ; exposure M
		//OV8858S_write_cmos_sensor( 0x3502, 0x40);  // ; exposure L
		//OV8858S_write_cmos_sensor( 0x3508, 0x04);  // ; gain H
		//OV8858S_write_cmos_sensor( 0x3700, 0x18);  //
		//OV8858S_write_cmos_sensor( 0x3701, 0x0c);  //
		//OV8858S_write_cmos_sensor( 0x3702, 0x28);  //
		//OV8858S_write_cmos_sensor( 0x3703, 0x19);  //
		//OV8858S_write_cmos_sensor( 0x3704, 0x14);  //
		//OV8858S_write_cmos_sensor( 0x3706, 0x35);  //
		//OV8858S_write_cmos_sensor( 0x3707, 0x04);  //
		//OV8858S_write_cmos_sensor( 0x3708, 0x24);  //
		//OV8858S_write_cmos_sensor( 0x3709, 0x33);  //
		//OV8858S_write_cmos_sensor( 0x370a, 0x00);  //
		//OV8858S_write_cmos_sensor( 0x370b, 0xb5);  //
		//OV8858S_write_cmos_sensor( 0x370c, 0x04);  //
		//OV8858S_write_cmos_sensor( 0x3718, 0x12);  //
		//OV8858S_write_cmos_sensor( 0x3712, 0x42);  //
		//OV8858S_write_cmos_sensor( 0x371e, 0x19);  //
		//OV8858S_write_cmos_sensor( 0x371f, 0x40);  //
		//OV8858S_write_cmos_sensor( 0x3720, 0x05);  //
		//OV8858S_write_cmos_sensor( 0x3721, 0x05);  //
		//OV8858S_write_cmos_sensor( 0x3724, 0x06);  //
		//OV8858S_write_cmos_sensor( 0x3725, 0x01);  //
		//OV8858S_write_cmos_sensor( 0x3726, 0x06);  //
		//OV8858S_write_cmos_sensor( 0x3728, 0x05);  //
		//OV8858S_write_cmos_sensor( 0x3729, 0x02);  //
		//OV8858S_write_cmos_sensor( 0x372a, 0x03);  //
		//OV8858S_write_cmos_sensor( 0x372b, 0x53);  //
		//OV8858S_write_cmos_sensor( 0x372c, 0xa3);  //
		//OV8858S_write_cmos_sensor( 0x372d, 0x53);  //
		//OV8858S_write_cmos_sensor( 0x372e, 0x06);  //
		//OV8858S_write_cmos_sensor( 0x372f, 0x10);  //
		//OV8858S_write_cmos_sensor( 0x3730, 0x01);  //
		//OV8858S_write_cmos_sensor( 0x3731, 0x06);  //
		//OV8858S_write_cmos_sensor( 0x3732, 0x14);  //
		//OV8858S_write_cmos_sensor( 0x3736, 0x20);  //
		//OV8858S_write_cmos_sensor( 0x373a, 0x05);  //
		//OV8858S_write_cmos_sensor( 0x373b, 0x06);  //
		//OV8858S_write_cmos_sensor( 0x373c, 0x0a);  //
		//OV8858S_write_cmos_sensor( 0x373e, 0x03);  //
		//OV8858S_write_cmos_sensor( 0x375a, 0x06);  //
		//OV8858S_write_cmos_sensor( 0x375b, 0x13);  //
		//OV8858S_write_cmos_sensor( 0x375d, 0x02);  //
		//OV8858S_write_cmos_sensor( 0x375f, 0x14);  //
		//OV8858S_write_cmos_sensor( 0x3772, 0x23);  //
		//OV8858S_write_cmos_sensor( 0x3773, 0x02);  //
		//OV8858S_write_cmos_sensor( 0x3774, 0x16);  //
		//OV8858S_write_cmos_sensor( 0x3775, 0x12);  //
		//OV8858S_write_cmos_sensor( 0x3776, 0x04);  //
		//OV8858S_write_cmos_sensor( 0x3778, 0x1b);  //
		//OV8858S_write_cmos_sensor( 0x37a0, 0x44);  //
		//OV8858S_write_cmos_sensor( 0x37a1, 0x3d);  //
		//OV8858S_write_cmos_sensor( 0x37a2, 0x3d);  //
		//OV8858S_write_cmos_sensor( 0x37a7, 0x44);  //
		//OV8858S_write_cmos_sensor( 0x37a8, 0x4c);  //
		//OV8858S_write_cmos_sensor( 0x37a9, 0x4c);  //
		//OV8858S_write_cmos_sensor( 0x37aa, 0x44);  //
		//OV8858S_write_cmos_sensor( 0x37ab, 0x2e);  //
		//OV8858S_write_cmos_sensor( 0x37ac, 0x2e);  //
		//OV8858S_write_cmos_sensor( 0x37ad, 0x33);  //
		//OV8858S_write_cmos_sensor( 0x37ae, 0x0d);  //
		//OV8858S_write_cmos_sensor( 0x37af, 0x0d);  //
		//OV8858S_write_cmos_sensor( 0x37b3, 0x42);  //
		//OV8858S_write_cmos_sensor( 0x37b4, 0x42);  //
		//OV8858S_write_cmos_sensor( 0x37b5, 0x33);  //
		//OV8858S_write_cmos_sensor( 0x3808, 0x06);  // ; x output size H
		//OV8858S_write_cmos_sensor( 0x3809, 0x60);  // ; x output size L
		//OV8858S_write_cmos_sensor( 0x380a, 0x04);  // ; y output size H
		//OV8858S_write_cmos_sensor( 0x380b, 0xc8);  // ; y output size L
		//OV8858S_write_cmos_sensor( 0x380c, 0x07);  // ; HTS H
		//OV8858S_write_cmos_sensor( 0x380d, 0x88);  // ; HTS L
		//OV8858S_write_cmos_sensor( 0x380e, 0x04);  // ; VTS H
		//OV8858S_write_cmos_sensor( 0x380f, 0xdc);  // ; VTS L
		//OV8858S_write_cmos_sensor( 0x3814, 0x03);  // ; x odd inc
		//OV8858S_write_cmos_sensor( 0x3821, 0x61);  // ; mirror on, bin on
		//OV8858S_write_cmos_sensor( 0x382a, 0x03);  // ; y odd inc
		//OV8858S_write_cmos_sensor( 0x3830, 0x08);  //
		//OV8858S_write_cmos_sensor( 0x3836, 0x02);  //
		//OV8858S_write_cmos_sensor( 0x3845, 0x02);
		//OV8858S_write_cmos_sensor( 0x3f08, 0x08);  //
		//OV8858S_write_cmos_sensor( 0x3f0a, 0x80);  //
		//OV8858S_write_cmos_sensor( 0x4001, 0x10);  // ; total 128 black column
		//OV8858S_write_cmos_sensor( 0x4022, 0x04);  // ; Anchor left end H
		//OV8858S_write_cmos_sensor( 0x4023, 0xb9);  // ; Anchor left end L
		//OV8858S_write_cmos_sensor( 0x4024, 0x05);  // ; Anchor right start H
		//OV8858S_write_cmos_sensor( 0x4025, 0x2a);  // ; Anchor right start L
		//OV8858S_write_cmos_sensor( 0x4026, 0x05);  // ; Anchor right end H
		//OV8858S_write_cmos_sensor( 0x4027, 0x2b);  // ; Anchor right end L
		//OV8858S_write_cmos_sensor( 0x402b, 0x04);  // ; top black line number
		//OV8858S_write_cmos_sensor( 0x402e, 0x08);  // ; bottom black line start
		//OV8858S_write_cmos_sensor( 0x4500, 0x38);  //
		//OV8858S_write_cmos_sensor( 0x4600, 0x00);  //
		//OV8858S_write_cmos_sensor( 0x4601, 0xcb);  //
		//OV8858S_write_cmos_sensor( 0x382d, 0x7f);  //
		//OV8858S_write_cmos_sensor( 0x0100, 0x01);  // ; 

		//@@1632x1224_30fps_4Lane_PCLK=74.4M

OV8858S_write_cmos_sensor(0x0100, 0x00);
OV8858S_write_cmos_sensor(0x030e, 0x02);
OV8858S_write_cmos_sensor(0x0312, 0x03);
OV8858S_write_cmos_sensor(0x3015, 0x00);
OV8858S_write_cmos_sensor(0x3700, 0x18);
OV8858S_write_cmos_sensor(0x3701, 0x0c);
OV8858S_write_cmos_sensor(0x3702, 0x28);
OV8858S_write_cmos_sensor(0x3703, 0x19);
OV8858S_write_cmos_sensor(0x3704, 0x14);
OV8858S_write_cmos_sensor(0x3707, 0x04);
OV8858S_write_cmos_sensor(0x3708, 0x24);
OV8858S_write_cmos_sensor(0x3709, 0x33);
OV8858S_write_cmos_sensor(0x370c, 0x04);
OV8858S_write_cmos_sensor(0x3718, 0x12);
OV8858S_write_cmos_sensor(0x3712, 0x42);
OV8858S_write_cmos_sensor(0x371e, 0x19);
OV8858S_write_cmos_sensor(0x371f, 0x40);
OV8858S_write_cmos_sensor(0x3720, 0x05);
OV8858S_write_cmos_sensor(0x3721, 0x05);
OV8858S_write_cmos_sensor(0x3724, 0x06);
OV8858S_write_cmos_sensor(0x3725, 0x01);
OV8858S_write_cmos_sensor(0x3726, 0x06);
OV8858S_write_cmos_sensor(0x3728, 0x05);
OV8858S_write_cmos_sensor(0x3729, 0x02);
OV8858S_write_cmos_sensor(0x372a, 0x03);
OV8858S_write_cmos_sensor(0x372b, 0x53);
OV8858S_write_cmos_sensor(0x372c, 0xa3);
OV8858S_write_cmos_sensor(0x372d, 0x53);
OV8858S_write_cmos_sensor(0x372e, 0x06);
OV8858S_write_cmos_sensor(0x372f, 0x10);
OV8858S_write_cmos_sensor(0x3730, 0x01);
OV8858S_write_cmos_sensor(0x3731, 0x06);
OV8858S_write_cmos_sensor(0x3732, 0x14);
OV8858S_write_cmos_sensor(0x3736, 0x20);
OV8858S_write_cmos_sensor(0x373a, 0x05);
OV8858S_write_cmos_sensor(0x373b, 0x06);
OV8858S_write_cmos_sensor(0x373c, 0x0a);
OV8858S_write_cmos_sensor(0x373e, 0x03);
OV8858S_write_cmos_sensor(0x375a, 0x06);
OV8858S_write_cmos_sensor(0x375b, 0x13);
OV8858S_write_cmos_sensor(0x375d, 0x02);
OV8858S_write_cmos_sensor(0x375f, 0x14);
OV8858S_write_cmos_sensor(0x3772, 0x23);
OV8858S_write_cmos_sensor(0x3773, 0x02);
OV8858S_write_cmos_sensor(0x3774, 0x16);
OV8858S_write_cmos_sensor(0x3775, 0x12);
OV8858S_write_cmos_sensor(0x3776, 0x04);
OV8858S_write_cmos_sensor(0x3778, 0x17);
OV8858S_write_cmos_sensor(0x37a0, 0x44);
OV8858S_write_cmos_sensor(0x37a1, 0x3d);
OV8858S_write_cmos_sensor(0x37a2, 0x3d);
OV8858S_write_cmos_sensor(0x37a7, 0x44);
OV8858S_write_cmos_sensor(0x37a8, 0x4c);
OV8858S_write_cmos_sensor(0x37a9, 0x4c);
OV8858S_write_cmos_sensor(0x37aa, 0x44);
OV8858S_write_cmos_sensor(0x37ab, 0x2e);
OV8858S_write_cmos_sensor(0x37ac, 0x2e);
OV8858S_write_cmos_sensor(0x37ad, 0x33);
OV8858S_write_cmos_sensor(0x37ae, 0x0d);
OV8858S_write_cmos_sensor(0x37af, 0x0d);
OV8858S_write_cmos_sensor(0x37b3, 0x42);
OV8858S_write_cmos_sensor(0x37b4, 0x42);
OV8858S_write_cmos_sensor(0x37b5, 0x31);
OV8858S_write_cmos_sensor(0x3768, 0x22);
OV8858S_write_cmos_sensor(0x3769, 0x44);
OV8858S_write_cmos_sensor(0x376a, 0x44);
OV8858S_write_cmos_sensor(0x3808, 0x06);
OV8858S_write_cmos_sensor(0x3809, 0x60);
OV8858S_write_cmos_sensor(0x380a, 0x04);
OV8858S_write_cmos_sensor(0x380b, 0xc8);
OV8858S_write_cmos_sensor(0x380c, 0x07);
OV8858S_write_cmos_sensor(0x380d, 0x88);
OV8858S_write_cmos_sensor(0x380e, 0x05);
OV8858S_write_cmos_sensor(0x380f, 0x04);
OV8858S_write_cmos_sensor(0x3814, 0x03);
OV8858S_write_cmos_sensor(0x3821, 0x67);
OV8858S_write_cmos_sensor(0x382a, 0x03);
OV8858S_write_cmos_sensor(0x382b, 0x01);
OV8858S_write_cmos_sensor(0x3830, 0x08);
OV8858S_write_cmos_sensor(0x3836, 0x02);
OV8858S_write_cmos_sensor(0x3f08, 0x08);
OV8858S_write_cmos_sensor(0x4001, 0x10);
OV8858S_write_cmos_sensor(0x4020, 0x00);
OV8858S_write_cmos_sensor(0x4021, 0x04);
OV8858S_write_cmos_sensor(0x4022, 0x04);
OV8858S_write_cmos_sensor(0x4023, 0xb9);
OV8858S_write_cmos_sensor(0x4024, 0x05);
OV8858S_write_cmos_sensor(0x4025, 0x2a);
OV8858S_write_cmos_sensor(0x4026, 0x05);
OV8858S_write_cmos_sensor(0x4027, 0x2b);
OV8858S_write_cmos_sensor(0x402a, 0x04);
OV8858S_write_cmos_sensor(0x402b, 0x04);
OV8858S_write_cmos_sensor(0x402e, 0x08);
OV8858S_write_cmos_sensor(0x402f, 0x02);
OV8858S_write_cmos_sensor(0x4600, 0x00);
OV8858S_write_cmos_sensor(0x4601, 0xcb);
OV8858S_write_cmos_sensor(0x5901, 0x00);
OV8858S_write_cmos_sensor(0x382d, 0x7f);
OV8858S_write_cmos_sensor(0x0100, 0x01);

}

void OV8858SCaptureSetting(void)
{
	// +++++++	@@5.1.1.3 Raw 10bit 3264*2448 30fps 4lane 672M bps/lane
	// +++++++	;Pclk 144MHz
	// +++++++	;pixels per line=1940(0x794) 
	// +++++++	;lines per frame=2474(0x9aa)
	
		//OV8858S_write_cmos_sensor( 0x0100, 0x00);  //
		//OV8858S_write_cmos_sensor( 0x030f, 0x04);  // ; pll2_divsp
		//OV8858S_write_cmos_sensor( 0x3501, 0x9a);  // ; exposure M
		//OV8858S_write_cmos_sensor( 0x3502, 0x20);  // ; exposure L
		//OV8858S_write_cmos_sensor( 0x3508, 0x02);  // ; gain H
		//OV8858S_write_cmos_sensor( 0x3700, 0x30);  //
		//OV8858S_write_cmos_sensor( 0x3701, 0x18);  //
		//OV8858S_write_cmos_sensor( 0x3702, 0x50);  //
		//OV8858S_write_cmos_sensor( 0x3703, 0x32);  //
		//OV8858S_write_cmos_sensor( 0x3704, 0x28);  //
		//OV8858S_write_cmos_sensor( 0x3706, 0x6a);  //
		//OV8858S_write_cmos_sensor( 0x3707, 0x08);  //
		//OV8858S_write_cmos_sensor( 0x3708, 0x48);  //
		//OV8858S_write_cmos_sensor( 0x3709, 0x66);  //
		//OV8858S_write_cmos_sensor( 0x370a, 0x01);  //
		//OV8858S_write_cmos_sensor( 0x370b, 0x6a);  //
		//OV8858S_write_cmos_sensor( 0x370c, 0x07);  //
		//OV8858S_write_cmos_sensor( 0x3718, 0x14);  //
		//OV8858S_write_cmos_sensor( 0x3712, 0x44);  //
		//OV8858S_write_cmos_sensor( 0x371e, 0x31);  //
		//OV8858S_write_cmos_sensor( 0x371f, 0x7f);  //
		//OV8858S_write_cmos_sensor( 0x3720, 0x0a);  //
		//OV8858S_write_cmos_sensor( 0x3721, 0x0a);  //
		//OV8858S_write_cmos_sensor( 0x3724, 0x0c);  //
		//OV8858S_write_cmos_sensor( 0x3725, 0x02);  //
		//OV8858S_write_cmos_sensor( 0x3726, 0x0c);  //
		//OV8858S_write_cmos_sensor( 0x3728, 0x0a);  //
		//OV8858S_write_cmos_sensor( 0x3729, 0x03);  //
		//OV8858S_write_cmos_sensor( 0x372a, 0x06);  //
		//OV8858S_write_cmos_sensor( 0x372b, 0xa6);  //
		//OV8858S_write_cmos_sensor( 0x372c, 0xa6);  //
		//OV8858S_write_cmos_sensor( 0x372d, 0xa6);  //
		//OV8858S_write_cmos_sensor( 0x372e, 0x0c);  //
		//OV8858S_write_cmos_sensor( 0x372f, 0x20);  //
		//OV8858S_write_cmos_sensor( 0x3730, 0x02);  //
		//OV8858S_write_cmos_sensor( 0x3731, 0x0c);  //
		//OV8858S_write_cmos_sensor( 0x3732, 0x28);  //
		//OV8858S_write_cmos_sensor( 0x3736, 0x30);  //
		//OV8858S_write_cmos_sensor( 0x373a, 0x0a);  //
		//OV8858S_write_cmos_sensor( 0x373b, 0x0b);  //
		//OV8858S_write_cmos_sensor( 0x373c, 0x14);  //
		//OV8858S_write_cmos_sensor( 0x373e, 0x06);  //
		//OV8858S_write_cmos_sensor( 0x375a, 0x0c);  //
		//OV8858S_write_cmos_sensor( 0x375b, 0x26);  //
		//OV8858S_write_cmos_sensor( 0x375d, 0x04);  //
		//OV8858S_write_cmos_sensor( 0x375f, 0x28);  //
		//OV8858S_write_cmos_sensor( 0x3772, 0x46);  //
		//OV8858S_write_cmos_sensor( 0x3773, 0x04);  //
		//OV8858S_write_cmos_sensor( 0x3774, 0x2c);  //
		//OV8858S_write_cmos_sensor( 0x3775, 0x13);  //
		//OV8858S_write_cmos_sensor( 0x3776, 0x08);  //
		//OV8858S_write_cmos_sensor( 0x3778, 0x16);  //
		//OV8858S_write_cmos_sensor( 0x37a0, 0x88);  //
		//OV8858S_write_cmos_sensor( 0x37a1, 0x7a);  //
		//OV8858S_write_cmos_sensor( 0x37a2, 0x7a);  //
		//OV8858S_write_cmos_sensor( 0x37a7, 0x88);  //
		//OV8858S_write_cmos_sensor( 0x37a8, 0x98);  //
		//OV8858S_write_cmos_sensor( 0x37a9, 0x98);  //
		//OV8858S_write_cmos_sensor( 0x37aa, 0x88);  //
		//OV8858S_write_cmos_sensor( 0x37ab, 0x5c);  //
		//OV8858S_write_cmos_sensor( 0x37ac, 0x5c);  //
		//OV8858S_write_cmos_sensor( 0x37ad, 0x55);  //
		//OV8858S_write_cmos_sensor( 0x37ae, 0x19);  //
		//OV8858S_write_cmos_sensor( 0x37af, 0x19);  //
		//OV8858S_write_cmos_sensor( 0x37b3, 0x84);  //
		//OV8858S_write_cmos_sensor( 0x37b4, 0x84);  //
		//OV8858S_write_cmos_sensor( 0x37b5, 0x66);  //
		//OV8858S_write_cmos_sensor( 0x3808, 0x0c);  // ; x output size H
		//OV8858S_write_cmos_sensor( 0x3809, 0xc0);  // ; x output size L
		//OV8858S_write_cmos_sensor( 0x380a, 0x09);  // ; y output size H
		//OV8858S_write_cmos_sensor( 0x380b, 0x90);  // ; y output size L
		//OV8858S_write_cmos_sensor( 0x380c, 0x07);  // ; HTS H
		//OV8858S_write_cmos_sensor( 0x380d, 0x94);  // ; HTS L
		//OV8858S_write_cmos_sensor( 0x380e, 0x09);  // ; VTS H
		//OV8858S_write_cmos_sensor( 0x380f, 0xaa);  // ; VTS L
		//OV8858S_write_cmos_sensor( 0x3814, 0x01);  // ; x odd inc
		//OV8858S_write_cmos_sensor( 0x3821, 0x40);  // ; mirror on, bin off
		//OV8858S_write_cmos_sensor( 0x382a, 0x01);  // ; y odd inc
		//OV8858S_write_cmos_sensor( 0x3830, 0x06);  //
		//OV8858S_write_cmos_sensor( 0x3836, 0x01);  //
		//OV8858S_write_cmos_sensor( 0x3845, 0x00);
		//OV8858S_write_cmos_sensor( 0x3f08, 0x08);  //
		//OV8858S_write_cmos_sensor( 0x3f0a, 0x00);  //
		//OV8858S_write_cmos_sensor( 0x4001, 0x00);  // ; total 256 black column
		//OV8858S_write_cmos_sensor( 0x4022, 0x0b);  // ; Anchor left end H
		//OV8858S_write_cmos_sensor( 0x4023, 0xc3);  // ; Anchor left end L
		//OV8858S_write_cmos_sensor( 0x4024, 0x0c);  // ; Anchor right start H
		//OV8858S_write_cmos_sensor( 0x4025, 0x36);  // ; Anchor right start L
		//OV8858S_write_cmos_sensor( 0x4026, 0x0c);  // ; Anchor right end H
		//OV8858S_write_cmos_sensor( 0x4027, 0x37);  // ; Anchor right end L
		//OV8858S_write_cmos_sensor( 0x402b, 0x08);  // ; top black line number
		//OV8858S_write_cmos_sensor( 0x402e, 0x0c);  // ; bottom black line start
		//OV8858S_write_cmos_sensor( 0x4500, 0x58);  //
		//OV8858S_write_cmos_sensor( 0x4600, 0x01);  //
		//OV8858S_write_cmos_sensor( 0x4601, 0x97);  //
		//OV8858S_write_cmos_sensor( 0x382d, 0xff);  //
		//OV8858S_write_cmos_sensor( 0x0100, 0x01);  //
OV8858S_write_cmos_sensor(0x0100, 0x00);
OV8858S_write_cmos_sensor(0x030e, 0x00);
OV8858S_write_cmos_sensor(0x0312, 0x01);
OV8858S_write_cmos_sensor(0x3015, 0x01);
OV8858S_write_cmos_sensor(0x3700, 0x30);
OV8858S_write_cmos_sensor(0x3701, 0x18);
OV8858S_write_cmos_sensor(0x3702, 0x50);
OV8858S_write_cmos_sensor(0x3703, 0x32);
OV8858S_write_cmos_sensor(0x3704, 0x28);
OV8858S_write_cmos_sensor(0x3707, 0x08);
OV8858S_write_cmos_sensor(0x3708, 0x48);
OV8858S_write_cmos_sensor(0x3709, 0x66);
OV8858S_write_cmos_sensor(0x370c, 0x07);
OV8858S_write_cmos_sensor(0x3718, 0x14);
OV8858S_write_cmos_sensor(0x3712, 0x44);
OV8858S_write_cmos_sensor(0x371e, 0x31);
OV8858S_write_cmos_sensor(0x371f, 0x7f);
OV8858S_write_cmos_sensor(0x3720, 0x0a);
OV8858S_write_cmos_sensor(0x3721, 0x0a);
OV8858S_write_cmos_sensor(0x3724, 0x0c);
OV8858S_write_cmos_sensor(0x3725, 0x02);
OV8858S_write_cmos_sensor(0x3726, 0x0c);
OV8858S_write_cmos_sensor(0x3728, 0x0a);
OV8858S_write_cmos_sensor(0x3729, 0x03);
OV8858S_write_cmos_sensor(0x372a, 0x06);
OV8858S_write_cmos_sensor(0x372b, 0xa6);
OV8858S_write_cmos_sensor(0x372c, 0xa6);
OV8858S_write_cmos_sensor(0x372d, 0xa6);
OV8858S_write_cmos_sensor(0x372e, 0x0c);
OV8858S_write_cmos_sensor(0x372f, 0x20);
OV8858S_write_cmos_sensor(0x3730, 0x02);
OV8858S_write_cmos_sensor(0x3731, 0x0c);
OV8858S_write_cmos_sensor(0x3732, 0x28);
OV8858S_write_cmos_sensor(0x3736, 0x30);
OV8858S_write_cmos_sensor(0x373a, 0x0a);
OV8858S_write_cmos_sensor(0x373b, 0x0b);
OV8858S_write_cmos_sensor(0x373c, 0x14);
OV8858S_write_cmos_sensor(0x373e, 0x06);
OV8858S_write_cmos_sensor(0x375a, 0x0c);
OV8858S_write_cmos_sensor(0x375b, 0x26);
OV8858S_write_cmos_sensor(0x375d, 0x04);
OV8858S_write_cmos_sensor(0x375f, 0x28);
OV8858S_write_cmos_sensor(0x3772, 0x46);
OV8858S_write_cmos_sensor(0x3773, 0x04);
OV8858S_write_cmos_sensor(0x3774, 0x2c);
OV8858S_write_cmos_sensor(0x3775, 0x13);
OV8858S_write_cmos_sensor(0x3776, 0x08);
OV8858S_write_cmos_sensor(0x3778, 0x17);
OV8858S_write_cmos_sensor(0x37a0, 0x88);
OV8858S_write_cmos_sensor(0x37a1, 0x7a);
OV8858S_write_cmos_sensor(0x37a2, 0x7a);
OV8858S_write_cmos_sensor(0x37a7, 0x88);
OV8858S_write_cmos_sensor(0x37a8, 0x98);
OV8858S_write_cmos_sensor(0x37a9, 0x98);
OV8858S_write_cmos_sensor(0x37aa, 0x88);
OV8858S_write_cmos_sensor(0x37ab, 0x5c);
OV8858S_write_cmos_sensor(0x37ac, 0x5c);
OV8858S_write_cmos_sensor(0x37ad, 0x55);
OV8858S_write_cmos_sensor(0x37ae, 0x19);
OV8858S_write_cmos_sensor(0x37af, 0x19);
OV8858S_write_cmos_sensor(0x37b3, 0x84);
OV8858S_write_cmos_sensor(0x37b4, 0x84);
OV8858S_write_cmos_sensor(0x37b5, 0x60);
OV8858S_write_cmos_sensor(0x3768, 0x22);
OV8858S_write_cmos_sensor(0x3769, 0x44);
OV8858S_write_cmos_sensor(0x376a, 0x44);
OV8858S_write_cmos_sensor(0x3808, 0x0c);
OV8858S_write_cmos_sensor(0x3809, 0xc0);
OV8858S_write_cmos_sensor(0x380a, 0x09);
OV8858S_write_cmos_sensor(0x380b, 0x90);
OV8858S_write_cmos_sensor(0x380c, 0x07);
OV8858S_write_cmos_sensor(0x380d, 0x94);
OV8858S_write_cmos_sensor(0x380e, 0x09);
OV8858S_write_cmos_sensor(0x380f, 0xfc);
OV8858S_write_cmos_sensor(0x3814, 0x01);
OV8858S_write_cmos_sensor(0x3821, 0x46);
OV8858S_write_cmos_sensor(0x382a, 0x01);
OV8858S_write_cmos_sensor(0x382b, 0x01);
OV8858S_write_cmos_sensor(0x3830, 0x06);
OV8858S_write_cmos_sensor(0x3836, 0x01);
OV8858S_write_cmos_sensor(0x3f08, 0x10);
OV8858S_write_cmos_sensor(0x4001, 0x00);
OV8858S_write_cmos_sensor(0x4020, 0x00);
OV8858S_write_cmos_sensor(0x4021, 0x04);
OV8858S_write_cmos_sensor(0x4022, 0x0b);
OV8858S_write_cmos_sensor(0x4023, 0xc3);
OV8858S_write_cmos_sensor(0x4024, 0x0c);
OV8858S_write_cmos_sensor(0x4025, 0x36);
OV8858S_write_cmos_sensor(0x4026, 0x0c);
OV8858S_write_cmos_sensor(0x4027, 0x37);
OV8858S_write_cmos_sensor(0x402a, 0x04);
OV8858S_write_cmos_sensor(0x402b, 0x08);
OV8858S_write_cmos_sensor(0x402e, 0x0c);
OV8858S_write_cmos_sensor(0x402f, 0x02);
OV8858S_write_cmos_sensor(0x4600, 0x01);
OV8858S_write_cmos_sensor(0x4601, 0x97);
OV8858S_write_cmos_sensor(0x5901, 0x00);
OV8858S_write_cmos_sensor(0x382d, 0xff);
OV8858S_write_cmos_sensor(0x0100, 0x01);
}

void OV8858SVideoSetting(void)
{
	//	++++++++   @@5.1.1.4 Raw 10bit 3264*1836 30fps 4lane 672M bps/lane
	//	++++++++   ;Pclk 144MHz
	//	++++++++   ;pixels per line=2566(0xa06) 
	//	++++++++   ;lines per frame=1872(0x750)
		//OV8858S_write_cmos_sensor( 0x0100, 0x00);  //  
		//OV8858S_write_cmos_sensor( 0x030f, 0x04);  //   ; pll2_divsp
		//OV8858S_write_cmos_sensor( 0x3501, 0x74);  //   ; exposure M
		//OV8858S_write_cmos_sensor( 0x3502, 0x80);  //   ; exposure L
		//OV8858S_write_cmos_sensor( 0x3508, 0x02);  //   ; gain H
		//OV8858S_write_cmos_sensor( 0x3700, 0x30);  //  
		//OV8858S_write_cmos_sensor( 0x3701, 0x18);  //  
		//OV8858S_write_cmos_sensor( 0x3702, 0x50);  //  
		//OV8858S_write_cmos_sensor( 0x3703, 0x32);  //  
		//OV8858S_write_cmos_sensor( 0x3704, 0x28);  //  
		//OV8858S_write_cmos_sensor( 0x3706, 0x6a);  //  
		//OV8858S_write_cmos_sensor( 0x3707, 0x08);  //  
		//OV8858S_write_cmos_sensor( 0x3708, 0x48);  //  
		//OV8858S_write_cmos_sensor( 0x3709, 0x66);  //  
		//OV8858S_write_cmos_sensor( 0x370a, 0x01);  //  
		//OV8858S_write_cmos_sensor( 0x370b, 0x6a);  //  
		//OV8858S_write_cmos_sensor( 0x370c, 0x07);  //  
		//OV8858S_write_cmos_sensor( 0x3718, 0x14);  //  
		//OV8858S_write_cmos_sensor( 0x3712, 0x44);  //  
		//OV8858S_write_cmos_sensor( 0x371e, 0x31);  //  
		//OV8858S_write_cmos_sensor( 0x371f, 0x7f);  //  
		//OV8858S_write_cmos_sensor( 0x3720, 0x0a);  //  
		//OV8858S_write_cmos_sensor( 0x3721, 0x0a);  //  
		//OV8858S_write_cmos_sensor( 0x3724, 0x0c);  //  
		//OV8858S_write_cmos_sensor( 0x3725, 0x02);  //  
		//OV8858S_write_cmos_sensor( 0x3726, 0x0c);  //  
		//OV8858S_write_cmos_sensor( 0x3728, 0x0a);  //  
		//OV8858S_write_cmos_sensor( 0x3729, 0x03);  //  
		//OV8858S_write_cmos_sensor( 0x372a, 0x06);  //  
		//OV8858S_write_cmos_sensor( 0x372b, 0xa6);  //  
		//OV8858S_write_cmos_sensor( 0x372c, 0xa6);  //  
		//OV8858S_write_cmos_sensor( 0x372d, 0xa6);  //  
		//OV8858S_write_cmos_sensor( 0x372e, 0x0c);  //  
		//OV8858S_write_cmos_sensor( 0x372f, 0x20);  //  
		//OV8858S_write_cmos_sensor( 0x3730, 0x02);  //  
		//OV8858S_write_cmos_sensor( 0x3731, 0x0c);  //  
		//OV8858S_write_cmos_sensor( 0x3732, 0x28);  //  
		//OV8858S_write_cmos_sensor( 0x3736, 0x30);  //  
		//OV8858S_write_cmos_sensor( 0x373a, 0x0a);  //  
		//OV8858S_write_cmos_sensor( 0x373b, 0x0b);  //  
		//OV8858S_write_cmos_sensor( 0x373c, 0x14);  //  
		//OV8858S_write_cmos_sensor( 0x373e, 0x06);  //  
		//OV8858S_write_cmos_sensor( 0x375a, 0x0c);  //  
		//OV8858S_write_cmos_sensor( 0x375b, 0x26);  //  
		//OV8858S_write_cmos_sensor( 0x375d, 0x04);  //  
		//OV8858S_write_cmos_sensor( 0x375f, 0x28);  //  
		//OV8858S_write_cmos_sensor( 0x3772, 0x46);  //  
		//OV8858S_write_cmos_sensor( 0x3773, 0x04);  //  
		//OV8858S_write_cmos_sensor( 0x3774, 0x2c);  //  
		//OV8858S_write_cmos_sensor( 0x3775, 0x13);  //  
		//OV8858S_write_cmos_sensor( 0x3776, 0x08);  //  
		//OV8858S_write_cmos_sensor( 0x3778, 0x16);  //  
		//OV8858S_write_cmos_sensor( 0x37a0, 0x88);  //  
		//OV8858S_write_cmos_sensor( 0x37a1, 0x7a);  //  
		//OV8858S_write_cmos_sensor( 0x37a2, 0x7a);  //  
		//OV8858S_write_cmos_sensor( 0x37a7, 0x88);  //  
		//OV8858S_write_cmos_sensor( 0x37a8, 0x98);  //  
		//OV8858S_write_cmos_sensor( 0x37a9, 0x98);  //  
		//OV8858S_write_cmos_sensor( 0x37aa, 0x88);  //  
		//OV8858S_write_cmos_sensor( 0x37ab, 0x5c);  //  
		//OV8858S_write_cmos_sensor( 0x37ac, 0x5c);  //  
		//OV8858S_write_cmos_sensor( 0x37ad, 0x55);  //  
		//OV8858S_write_cmos_sensor( 0x37ae, 0x19);  //  
		//OV8858S_write_cmos_sensor( 0x37af, 0x19);  //  
		//OV8858S_write_cmos_sensor( 0x37b3, 0x84);  //  
		//OV8858S_write_cmos_sensor( 0x37b4, 0x84);  //  
		//OV8858S_write_cmos_sensor( 0x37b5, 0x66);  //  
		//OV8858S_write_cmos_sensor( 0x3808, 0x0c);  //   ; x output size H
		//OV8858S_write_cmos_sensor( 0x3809, 0xc0);  //   ; x output size L
		//OV8858S_write_cmos_sensor( 0x380a, 0x07);  //   ; y output size H
		//OV8858S_write_cmos_sensor( 0x380b, 0x2c);  //   ; y output size L
		//OV8858S_write_cmos_sensor( 0x380c, 0x0a);  //   ; HTS H
		//OV8858S_write_cmos_sensor( 0x380d, 0x06);  //   ; HTS L
		//OV8858S_write_cmos_sensor( 0x380e, 0x07);  //   ; VTS H
		//OV8858S_write_cmos_sensor( 0x380f, 0x50);  //   ; VTS L
		//OV8858S_write_cmos_sensor( 0x3814, 0x01);  //   ; x odd inc
		//OV8858S_write_cmos_sensor( 0x3821, 0x40);  //   ; mirror on, bin off
		//OV8858S_write_cmos_sensor( 0x382a, 0x01);  //   ; y odd inc
		//OV8858S_write_cmos_sensor( 0x3830, 0x06);  //  
		//OV8858S_write_cmos_sensor( 0x3836, 0x01);  //  
		//OV8858S_write_cmos_sensor( 0x3845, 0x00);
		//OV8858S_write_cmos_sensor( 0x3f08, 0x08);  //  
		//OV8858S_write_cmos_sensor( 0x3f0a, 0x00);  //  
		//OV8858S_write_cmos_sensor( 0x4001, 0x00);  //   ; total 256 black column
		//OV8858S_write_cmos_sensor( 0x4022, 0x0b);  //   ; Anchor left end H
		//OV8858S_write_cmos_sensor( 0x4023, 0xc3);  //   ; Anchor left end L
		//OV8858S_write_cmos_sensor( 0x4024, 0x0c);  //   ; Anchor right start H
		//OV8858S_write_cmos_sensor( 0x4025, 0x36);  //   ; Anchor right start L
		//OV8858S_write_cmos_sensor( 0x4026, 0x0c);  //   ; Anchor right end H
		//OV8858S_write_cmos_sensor( 0x4027, 0x37);  //   ; Anchor right end L
		//OV8858S_write_cmos_sensor( 0x402b, 0x08);  //   ; top black line number
		//OV8858S_write_cmos_sensor( 0x402e, 0x0c);  //   ; bottom black line start
		//OV8858S_write_cmos_sensor( 0x4500, 0x58);  //  
		//OV8858S_write_cmos_sensor( 0x4600, 0x01);  //  
		//OV8858S_write_cmos_sensor( 0x4601, 0x97);  //  
		//OV8858S_write_cmos_sensor( 0x382d, 0xff);  //  
		//OV8858S_write_cmos_sensor( 0x0100, 0x01);  //	
OV8858S_write_cmos_sensor(0x0100, 0x00);
OV8858S_write_cmos_sensor(0x030e, 0x00);
OV8858S_write_cmos_sensor(0x0312, 0x01);
OV8858S_write_cmos_sensor(0x3015, 0x01);
OV8858S_write_cmos_sensor(0x3700, 0x30);
OV8858S_write_cmos_sensor(0x3701, 0x18);
OV8858S_write_cmos_sensor(0x3702, 0x50);
OV8858S_write_cmos_sensor(0x3703, 0x32);
OV8858S_write_cmos_sensor(0x3704, 0x28);
OV8858S_write_cmos_sensor(0x3707, 0x08);
OV8858S_write_cmos_sensor(0x3708, 0x48);
OV8858S_write_cmos_sensor(0x3709, 0x66);
OV8858S_write_cmos_sensor(0x370c, 0x07);
OV8858S_write_cmos_sensor(0x3718, 0x14);
OV8858S_write_cmos_sensor(0x3712, 0x44);
OV8858S_write_cmos_sensor(0x371e, 0x31);
OV8858S_write_cmos_sensor(0x371f, 0x7f);
OV8858S_write_cmos_sensor(0x3720, 0x0a);
OV8858S_write_cmos_sensor(0x3721, 0x0a);
OV8858S_write_cmos_sensor(0x3724, 0x0c);
OV8858S_write_cmos_sensor(0x3725, 0x02);
OV8858S_write_cmos_sensor(0x3726, 0x0c);
OV8858S_write_cmos_sensor(0x3728, 0x0a);
OV8858S_write_cmos_sensor(0x3729, 0x03);
OV8858S_write_cmos_sensor(0x372a, 0x06);
OV8858S_write_cmos_sensor(0x372b, 0xa6);
OV8858S_write_cmos_sensor(0x372c, 0xa6);
OV8858S_write_cmos_sensor(0x372d, 0xa6);
OV8858S_write_cmos_sensor(0x372e, 0x0c);
OV8858S_write_cmos_sensor(0x372f, 0x20);
OV8858S_write_cmos_sensor(0x3730, 0x02);
OV8858S_write_cmos_sensor(0x3731, 0x0c);
OV8858S_write_cmos_sensor(0x3732, 0x28);
OV8858S_write_cmos_sensor(0x3736, 0x30);
OV8858S_write_cmos_sensor(0x373a, 0x0a);
OV8858S_write_cmos_sensor(0x373b, 0x0b);
OV8858S_write_cmos_sensor(0x373c, 0x14);
OV8858S_write_cmos_sensor(0x373e, 0x06);
OV8858S_write_cmos_sensor(0x375a, 0x0c);
OV8858S_write_cmos_sensor(0x375b, 0x26);
OV8858S_write_cmos_sensor(0x375d, 0x04);
OV8858S_write_cmos_sensor(0x375f, 0x28);
OV8858S_write_cmos_sensor(0x3772, 0x46);
OV8858S_write_cmos_sensor(0x3773, 0x04);
OV8858S_write_cmos_sensor(0x3774, 0x2c);
OV8858S_write_cmos_sensor(0x3775, 0x13);
OV8858S_write_cmos_sensor(0x3776, 0x08);
OV8858S_write_cmos_sensor(0x3778, 0x17);
OV8858S_write_cmos_sensor(0x37a0, 0x88);
OV8858S_write_cmos_sensor(0x37a1, 0x7a);
OV8858S_write_cmos_sensor(0x37a2, 0x7a);
OV8858S_write_cmos_sensor(0x37a7, 0x88);
OV8858S_write_cmos_sensor(0x37a8, 0x98);
OV8858S_write_cmos_sensor(0x37a9, 0x98);
OV8858S_write_cmos_sensor(0x37aa, 0x88);
OV8858S_write_cmos_sensor(0x37ab, 0x5c);
OV8858S_write_cmos_sensor(0x37ac, 0x5c);
OV8858S_write_cmos_sensor(0x37ad, 0x55);
OV8858S_write_cmos_sensor(0x37ae, 0x19);
OV8858S_write_cmos_sensor(0x37af, 0x19);
OV8858S_write_cmos_sensor(0x37b3, 0x84);
OV8858S_write_cmos_sensor(0x37b4, 0x84);
OV8858S_write_cmos_sensor(0x37b5, 0x60);
OV8858S_write_cmos_sensor(0x3768, 0x22);
OV8858S_write_cmos_sensor(0x3769, 0x44);
OV8858S_write_cmos_sensor(0x376a, 0x44);
OV8858S_write_cmos_sensor(0x3808, 0x0c);
OV8858S_write_cmos_sensor(0x3809, 0xc0);
OV8858S_write_cmos_sensor(0x380a, 0x07);
OV8858S_write_cmos_sensor(0x380b, 0x2c);
OV8858S_write_cmos_sensor(0x380c, 0x0a);
OV8858S_write_cmos_sensor(0x380d, 0x06);
OV8858S_write_cmos_sensor(0x380e, 0x07);
OV8858S_write_cmos_sensor(0x380f, 0x8c);
OV8858S_write_cmos_sensor(0x3814, 0x01);
OV8858S_write_cmos_sensor(0x3821, 0x46);
OV8858S_write_cmos_sensor(0x382a, 0x01);
OV8858S_write_cmos_sensor(0x382b, 0x01);
OV8858S_write_cmos_sensor(0x3830, 0x06);
OV8858S_write_cmos_sensor(0x3836, 0x01);
OV8858S_write_cmos_sensor(0x3f08, 0x10);
OV8858S_write_cmos_sensor(0x4001, 0x00);
OV8858S_write_cmos_sensor(0x4020, 0x00);
OV8858S_write_cmos_sensor(0x4021, 0x04);
OV8858S_write_cmos_sensor(0x4022, 0x0b);
OV8858S_write_cmos_sensor(0x4023, 0xc3);
OV8858S_write_cmos_sensor(0x4024, 0x0c);
OV8858S_write_cmos_sensor(0x4025, 0x36);
OV8858S_write_cmos_sensor(0x4026, 0x0c);
OV8858S_write_cmos_sensor(0x4027, 0x37);
OV8858S_write_cmos_sensor(0x402a, 0x04);
OV8858S_write_cmos_sensor(0x402b, 0x08);
OV8858S_write_cmos_sensor(0x402e, 0x0c);
OV8858S_write_cmos_sensor(0x402f, 0x02);
OV8858S_write_cmos_sensor(0x4600, 0x01);
OV8858S_write_cmos_sensor(0x4601, 0x97);
OV8858S_write_cmos_sensor(0x5901, 0x00);
OV8858S_write_cmos_sensor(0x382d, 0xff);
OV8858S_write_cmos_sensor(0x0100, 0x01);

}

static void OV8858S_Sensor_Init(void)
{
	// ++++++++ @@5.1.1.1 Initialization (Global Setting)
	// ++++++++ ;;
	// ++++++++ ;; MIPI=672Mbps, SysClk=72Mhz,Dac Clock=360Mhz.
	// ++++++++ ;;
	// ++++++++ ;;
	// ++++++++ ;; v00_01_00 (12/19/2013) : initial setting
	// ++++++++ ;;
	//OV8858S_write_cmos_sensor( 0x0103, 0x01);
	//OV8858S_write_cmos_sensor( 0x0100, 0x00);
	//OV8858S_write_cmos_sensor( 0x0100, 0x00);
	//OV8858S_write_cmos_sensor( 0x0100, 0x00);
	//OV8858S_write_cmos_sensor( 0x0100, 0x00);
	//OV8858S_write_cmos_sensor( 0x0302, 0x1c);
	//OV8858S_write_cmos_sensor( 0x0303, 0x00);
	//OV8858S_write_cmos_sensor( 0x0304, 0x03);
	//OV8858S_write_cmos_sensor( 0x030e, 0x00);
	//OV8858S_write_cmos_sensor( 0x030f, 0x09);
	//OV8858S_write_cmos_sensor( 0x0312, 0x01);
	//OV8858S_write_cmos_sensor( 0x031e, 0x0c);
	//OV8858S_write_cmos_sensor( 0x3600, 0x00);
	//OV8858S_write_cmos_sensor( 0x3601, 0x00);
	//OV8858S_write_cmos_sensor( 0x3602, 0x00);
	//OV8858S_write_cmos_sensor( 0x3603, 0x00);
	//OV8858S_write_cmos_sensor( 0x3604, 0x22);
	//OV8858S_write_cmos_sensor( 0x3605, 0x30);
	//OV8858S_write_cmos_sensor( 0x3606, 0x00);
	//OV8858S_write_cmos_sensor( 0x3607, 0x20);
	//OV8858S_write_cmos_sensor( 0x3608, 0x11);
	//OV8858S_write_cmos_sensor( 0x3609, 0x28);
	//OV8858S_write_cmos_sensor( 0x360a, 0x00);
	//OV8858S_write_cmos_sensor( 0x360b, 0x06);
	//OV8858S_write_cmos_sensor( 0x360c, 0xdc);
	//OV8858S_write_cmos_sensor( 0x360d, 0x40);
	//OV8858S_write_cmos_sensor( 0x360e, 0x0c);
	//OV8858S_write_cmos_sensor( 0x360f, 0x20);
	//OV8858S_write_cmos_sensor( 0x3610, 0x07);
	//OV8858S_write_cmos_sensor( 0x3611, 0x20);
	//OV8858S_write_cmos_sensor( 0x3612, 0x88);
	//OV8858S_write_cmos_sensor( 0x3613, 0x80);
	//OV8858S_write_cmos_sensor( 0x3614, 0x58);
	//OV8858S_write_cmos_sensor( 0x3615, 0x00);
	//OV8858S_write_cmos_sensor( 0x3616, 0x4a);
	//OV8858S_write_cmos_sensor( 0x3617, 0x90);
	//OV8858S_write_cmos_sensor( 0x3618, 0x56);
	//OV8858S_write_cmos_sensor( 0x3619, 0x70);
	//OV8858S_write_cmos_sensor( 0x361a, 0x99);
	//OV8858S_write_cmos_sensor( 0x361b, 0x00);
	//OV8858S_write_cmos_sensor( 0x361c, 0x07);
	//OV8858S_write_cmos_sensor( 0x361d, 0x00);
	//OV8858S_write_cmos_sensor( 0x361e, 0x00);
	//OV8858S_write_cmos_sensor( 0x361f, 0x00);
	//OV8858S_write_cmos_sensor( 0x3638, 0xff);
	//OV8858S_write_cmos_sensor( 0x3633, 0x0c);
	//OV8858S_write_cmos_sensor( 0x3634, 0x0c);
	//OV8858S_write_cmos_sensor( 0x3635, 0x0c);
	//OV8858S_write_cmos_sensor( 0x3636, 0x0c);
	//OV8858S_write_cmos_sensor( 0x3645, 0x13);
	//OV8858S_write_cmos_sensor( 0x3646, 0x83);
	//OV8858S_write_cmos_sensor( 0x364a, 0x07);
	//OV8858S_write_cmos_sensor( 0x3015, 0x01);
	//OV8858S_write_cmos_sensor( 0x3018, 0x72);
	//OV8858S_write_cmos_sensor( 0x3020, 0x93);
	//OV8858S_write_cmos_sensor( 0x3022, 0x01);
	//OV8858S_write_cmos_sensor( 0x3031, 0x0a);
	//OV8858S_write_cmos_sensor( 0x3034, 0x00);
	//OV8858S_write_cmos_sensor( 0x3106, 0x01);
	//OV8858S_write_cmos_sensor( 0x3305, 0xf1);
	//OV8858S_write_cmos_sensor( 0x3308, 0x00);
	//OV8858S_write_cmos_sensor( 0x3309, 0x28);
	//OV8858S_write_cmos_sensor( 0x330a, 0x00);
	//OV8858S_write_cmos_sensor( 0x330b, 0x20);
	//OV8858S_write_cmos_sensor( 0x330c, 0x00);
	//OV8858S_write_cmos_sensor( 0x330d, 0x00);
	//OV8858S_write_cmos_sensor( 0x330e, 0x00);
	//OV8858S_write_cmos_sensor( 0x330f, 0x40);
	//OV8858S_write_cmos_sensor( 0x3307, 0x04);
	//OV8858S_write_cmos_sensor( 0x3500, 0x00);
	//OV8858S_write_cmos_sensor( 0x3501, 0x4d);
	//OV8858S_write_cmos_sensor( 0x3502, 0x40);
	//OV8858S_write_cmos_sensor( 0x3503, 0x00);
	//OV8858S_write_cmos_sensor( 0x3505, 0x80);
	//OV8858S_write_cmos_sensor( 0x3508, 0x04);
	//OV8858S_write_cmos_sensor( 0x3509, 0x00);
	//OV8858S_write_cmos_sensor( 0x350c, 0x00);
	//OV8858S_write_cmos_sensor( 0x350d, 0x80);
	//OV8858S_write_cmos_sensor( 0x3510, 0x00);
	//OV8858S_write_cmos_sensor( 0x3511, 0x02);
	//OV8858S_write_cmos_sensor( 0x3512, 0x00);
	//OV8858S_write_cmos_sensor( 0x3700, 0x18);
	//OV8858S_write_cmos_sensor( 0x3701, 0x0c);
	//OV8858S_write_cmos_sensor( 0x3702, 0x28);
	//OV8858S_write_cmos_sensor( 0x3703, 0x19);
	//OV8858S_write_cmos_sensor( 0x3704, 0x14);
	//OV8858S_write_cmos_sensor( 0x3705, 0x00);
	//OV8858S_write_cmos_sensor( 0x3706, 0x35);
	//OV8858S_write_cmos_sensor( 0x3707, 0x04);
	//OV8858S_write_cmos_sensor( 0x3708, 0x24);
	//OV8858S_write_cmos_sensor( 0x3709, 0x33);
	//OV8858S_write_cmos_sensor( 0x370a, 0x00);
	//OV8858S_write_cmos_sensor( 0x370b, 0xb5);
	//OV8858S_write_cmos_sensor( 0x370c, 0x04);
	//OV8858S_write_cmos_sensor( 0x3718, 0x12);
	//OV8858S_write_cmos_sensor( 0x3719, 0x31);
	//OV8858S_write_cmos_sensor( 0x3712, 0x42);
	//OV8858S_write_cmos_sensor( 0x3714, 0x24);
	//OV8858S_write_cmos_sensor( 0x371e, 0x19);
	//OV8858S_write_cmos_sensor( 0x371f, 0x40);
	//OV8858S_write_cmos_sensor( 0x3720, 0x05);
	//OV8858S_write_cmos_sensor( 0x3721, 0x05);
	//OV8858S_write_cmos_sensor( 0x3724, 0x06);
	//OV8858S_write_cmos_sensor( 0x3725, 0x01);
	//OV8858S_write_cmos_sensor( 0x3726, 0x06);
	//OV8858S_write_cmos_sensor( 0x3728, 0x05);
	//OV8858S_write_cmos_sensor( 0x3729, 0x02);
	//OV8858S_write_cmos_sensor( 0x372a, 0x03);
	//OV8858S_write_cmos_sensor( 0x372b, 0x53);
	//OV8858S_write_cmos_sensor( 0x372c, 0xa3);
	//OV8858S_write_cmos_sensor( 0x372d, 0x53);
	//OV8858S_write_cmos_sensor( 0x372e, 0x06);
	//OV8858S_write_cmos_sensor( 0x372f, 0x10);
	//OV8858S_write_cmos_sensor( 0x3730, 0x01);
	//OV8858S_write_cmos_sensor( 0x3731, 0x06);
	//OV8858S_write_cmos_sensor( 0x3732, 0x14);
	//OV8858S_write_cmos_sensor( 0x3733, 0x10);
	//OV8858S_write_cmos_sensor( 0x3734, 0x40);
	//OV8858S_write_cmos_sensor( 0x3736, 0x20);
	//OV8858S_write_cmos_sensor( 0x373a, 0x05);
	//OV8858S_write_cmos_sensor( 0x373b, 0x06);
	//OV8858S_write_cmos_sensor( 0x373c, 0x0a);
	//OV8858S_write_cmos_sensor( 0x373e, 0x03);
	//OV8858S_write_cmos_sensor( 0x3755, 0x10);
	//OV8858S_write_cmos_sensor( 0x3758, 0x00);
	//OV8858S_write_cmos_sensor( 0x3759, 0x4c);
	//OV8858S_write_cmos_sensor( 0x375a, 0x06);
	//OV8858S_write_cmos_sensor( 0x375b, 0x13);
	//OV8858S_write_cmos_sensor( 0x375c, 0x20);
	//OV8858S_write_cmos_sensor( 0x375d, 0x02);
	//OV8858S_write_cmos_sensor( 0x375e, 0x00);
	//OV8858S_write_cmos_sensor( 0x375f, 0x14);
	//OV8858S_write_cmos_sensor( 0x3768, 0x22);
	//OV8858S_write_cmos_sensor( 0x3769, 0x44);
	//OV8858S_write_cmos_sensor( 0x376a, 0x44);
	//OV8858S_write_cmos_sensor( 0x3761, 0x00);
	//OV8858S_write_cmos_sensor( 0x3762, 0x00);
	//OV8858S_write_cmos_sensor( 0x3763, 0x00);
	//OV8858S_write_cmos_sensor( 0x3766, 0xff);
	//OV8858S_write_cmos_sensor( 0x376b, 0x00);
	//OV8858S_write_cmos_sensor( 0x3772, 0x23);
	//OV8858S_write_cmos_sensor( 0x3773, 0x02);
	//OV8858S_write_cmos_sensor( 0x3774, 0x16);
	//OV8858S_write_cmos_sensor( 0x3775, 0x12);
	//OV8858S_write_cmos_sensor( 0x3776, 0x04);
	//OV8858S_write_cmos_sensor( 0x3777, 0x00);
	//OV8858S_write_cmos_sensor( 0x3778, 0x1b);
	//OV8858S_write_cmos_sensor( 0x37a0, 0x44);
	//OV8858S_write_cmos_sensor( 0x37a1, 0x3d);
	//OV8858S_write_cmos_sensor( 0x37a2, 0x3d);
	//OV8858S_write_cmos_sensor( 0x37a3, 0x00);
	//OV8858S_write_cmos_sensor( 0x37a4, 0x00);
	//OV8858S_write_cmos_sensor( 0x37a5, 0x00);
	//OV8858S_write_cmos_sensor( 0x37a6, 0x00);
	//OV8858S_write_cmos_sensor( 0x37a7, 0x44);
	//OV8858S_write_cmos_sensor( 0x37a8, 0x4c);
	//OV8858S_write_cmos_sensor( 0x37a9, 0x4c);
	//OV8858S_write_cmos_sensor( 0x3760, 0x00);
	//OV8858S_write_cmos_sensor( 0x376f, 0x01);
	//OV8858S_write_cmos_sensor( 0x37aa, 0x44);
	//OV8858S_write_cmos_sensor( 0x37ab, 0x2e);
	//OV8858S_write_cmos_sensor( 0x37ac, 0x2e);
	//OV8858S_write_cmos_sensor( 0x37ad, 0x33);
	//OV8858S_write_cmos_sensor( 0x37ae, 0x0d);
	//OV8858S_write_cmos_sensor( 0x37af, 0x0d);
	//OV8858S_write_cmos_sensor( 0x37b0, 0x00);
	//OV8858S_write_cmos_sensor( 0x37b1, 0x00);
	//OV8858S_write_cmos_sensor( 0x37b2, 0x00);
	//OV8858S_write_cmos_sensor( 0x37b3, 0x42);
	//OV8858S_write_cmos_sensor( 0x37b4, 0x42);
	//OV8858S_write_cmos_sensor( 0x37b5, 0x33);
	//OV8858S_write_cmos_sensor( 0x37b6, 0x00);
	//OV8858S_write_cmos_sensor( 0x37b7, 0x00);
	//OV8858S_write_cmos_sensor( 0x37b8, 0x00);
	//OV8858S_write_cmos_sensor( 0x37b9, 0xff);
	//OV8858S_write_cmos_sensor( 0x3800, 0x00);
	//OV8858S_write_cmos_sensor( 0x3801, 0x0c);
	//OV8858S_write_cmos_sensor( 0x3802, 0x00);
	//OV8858S_write_cmos_sensor( 0x3803, 0x0c);
	//OV8858S_write_cmos_sensor( 0x3804, 0x0c);
	//OV8858S_write_cmos_sensor( 0x3805, 0xd3);
	//OV8858S_write_cmos_sensor( 0x3806, 0x09);
	//OV8858S_write_cmos_sensor( 0x3807, 0xa3);
	//OV8858S_write_cmos_sensor( 0x3808, 0x06);
	//OV8858S_write_cmos_sensor( 0x3809, 0x60);
	//OV8858S_write_cmos_sensor( 0x380a, 0x04);
	//OV8858S_write_cmos_sensor( 0x380b, 0xc8);
	//OV8858S_write_cmos_sensor( 0x380c, 0x07);
	//OV8858S_write_cmos_sensor( 0x380d, 0x88);
	//OV8858S_write_cmos_sensor( 0x380e, 0x04);
	//OV8858S_write_cmos_sensor( 0x380f, 0xdc);
	//OV8858S_write_cmos_sensor( 0x3810, 0x00);
	//OV8858S_write_cmos_sensor( 0x3811, 0x04);
	//OV8858S_write_cmos_sensor( 0x3813, 0x02);
	//OV8858S_write_cmos_sensor( 0x3814, 0x03);
	//OV8858S_write_cmos_sensor( 0x3815, 0x01);
	//OV8858S_write_cmos_sensor( 0x3820, 0x06);
	//OV8858S_write_cmos_sensor( 0x3821, 0x61);
	//OV8858S_write_cmos_sensor( 0x382a, 0x03);
	//OV8858S_write_cmos_sensor( 0x382b, 0x01);
	//OV8858S_write_cmos_sensor( 0x3830, 0x08);
	//OV8858S_write_cmos_sensor( 0x3836, 0x02);
	//OV8858S_write_cmos_sensor( 0x3837, 0x18);
	//OV8858S_write_cmos_sensor( 0x3841, 0xff);
	//OV8858S_write_cmos_sensor( 0x3846, 0x48);
	//OV8858S_write_cmos_sensor( 0x3d85, 0x16); // 0x14
	//OV8858S_write_cmos_sensor( 0x3d8c, 0x73);
	//OV8858S_write_cmos_sensor( 0x3d8d, 0xde);
	//OV8858S_write_cmos_sensor( 0x3f08, 0x08);
	//OV8858S_write_cmos_sensor( 0x3f0a, 0x80);
	//OV8858S_write_cmos_sensor( 0x4000, 0xf1);
	//OV8858S_write_cmos_sensor( 0x4001, 0x10);
	//OV8858S_write_cmos_sensor( 0x4005, 0x10);
	//OV8858S_write_cmos_sensor( 0x4002, 0x27);
	//OV8858S_write_cmos_sensor( 0x4006, 0x04);
	//OV8858S_write_cmos_sensor( 0x4007, 0x04);
	//OV8858S_write_cmos_sensor( 0x4009, 0x81);
	//OV8858S_write_cmos_sensor( 0x400b, 0x0c);
	//OV8858S_write_cmos_sensor( 0x401b, 0x00);
	//OV8858S_write_cmos_sensor( 0x401d, 0x00);
	//OV8858S_write_cmos_sensor( 0x4020, 0x00);
	//OV8858S_write_cmos_sensor( 0x4021, 0x04);
	//OV8858S_write_cmos_sensor( 0x4022, 0x04);
	//OV8858S_write_cmos_sensor( 0x4023, 0xb9);
	//OV8858S_write_cmos_sensor( 0x4024, 0x05);
	//OV8858S_write_cmos_sensor( 0x4025, 0x2a);
	//OV8858S_write_cmos_sensor( 0x4026, 0x05);
	//OV8858S_write_cmos_sensor( 0x4027, 0x2b);
	//OV8858S_write_cmos_sensor( 0x4028, 0x00);
	//OV8858S_write_cmos_sensor( 0x4029, 0x02);
	//OV8858S_write_cmos_sensor( 0x402a, 0x04);
	//OV8858S_write_cmos_sensor( 0x402b, 0x04);
	//OV8858S_write_cmos_sensor( 0x402c, 0x02);
	//OV8858S_write_cmos_sensor( 0x402d, 0x02);
	//OV8858S_write_cmos_sensor( 0x402e, 0x08);
	//OV8858S_write_cmos_sensor( 0x402f, 0x02);
	//OV8858S_write_cmos_sensor( 0x401f, 0x00);
	//OV8858S_write_cmos_sensor( 0x4034, 0x3f);
	//OV8858S_write_cmos_sensor( 0x403d, 0x04);
	//OV8858S_write_cmos_sensor( 0x4300, 0xff);
	//OV8858S_write_cmos_sensor( 0x4301, 0x00);
	//OV8858S_write_cmos_sensor( 0x4302, 0x0f);
	//OV8858S_write_cmos_sensor( 0x4316, 0x00);
	//OV8858S_write_cmos_sensor( 0x4500, 0x38);
	//OV8858S_write_cmos_sensor( 0x4503, 0x18);
	//OV8858S_write_cmos_sensor( 0x4600, 0x00);
	//OV8858S_write_cmos_sensor( 0x4601, 0xcb);
	//OV8858S_write_cmos_sensor( 0x481f, 0x32);
	//OV8858S_write_cmos_sensor( 0x4837, 0x17);
	//OV8858S_write_cmos_sensor( 0x4850, 0x10);
	//OV8858S_write_cmos_sensor( 0x4851, 0x32);
	//OV8858S_write_cmos_sensor( 0x4b00, 0x2a);
	//OV8858S_write_cmos_sensor( 0x4b0d, 0x00);
	//OV8858S_write_cmos_sensor( 0x4d00, 0x04);
	//OV8858S_write_cmos_sensor( 0x4d01, 0x18);
	//OV8858S_write_cmos_sensor( 0x4d02, 0xc3);
	//OV8858S_write_cmos_sensor( 0x4d03, 0xff);
	//OV8858S_write_cmos_sensor( 0x4d04, 0xff);
	//OV8858S_write_cmos_sensor( 0x4d05, 0xff);
	//OV8858S_write_cmos_sensor( 0x5000, 0x7e);
	//OV8858S_write_cmos_sensor( 0x5001, 0x01);
	//OV8858S_write_cmos_sensor( 0x5002, 0x08);
	//OV8858S_write_cmos_sensor( 0x5003, 0x20);
	//OV8858S_write_cmos_sensor( 0x5046, 0x12);
	//OV8858S_write_cmos_sensor( 0x5780, 0xfc);
	//OV8858S_write_cmos_sensor( 0x5784, 0x0c);
	//OV8858S_write_cmos_sensor( 0x5787, 0x40);
	//OV8858S_write_cmos_sensor( 0x5788, 0x08);
	//OV8858S_write_cmos_sensor( 0x578a, 0x02);
	//OV8858S_write_cmos_sensor( 0x578b, 0x01);
	//OV8858S_write_cmos_sensor( 0x578c, 0x01);
	//OV8858S_write_cmos_sensor( 0x578e, 0x02);
	//OV8858S_write_cmos_sensor( 0x578f, 0x01);
	//OV8858S_write_cmos_sensor( 0x5790, 0x01);
//2014-3-31
//nick email add
//fuction
//--which will auto decrease sensor shading gain when sensor gain > 4x. 
//which can improve low light performance.
    //OV8858S_write_cmos_sensor( 0x5871, 0x0d);
	//OV8858S_write_cmos_sensor( 0x5870, 0x18);
	//OV8858S_write_cmos_sensor( 0x586e, 0x10);
	//OV8858S_write_cmos_sensor( 0x586f, 0x08);
//2014-3-31 end
	//OV8858S_write_cmos_sensor( 0x5901, 0x00);
	//OV8858S_write_cmos_sensor( 0x5b00, 0x02);
	//OV8858S_write_cmos_sensor( 0x5b01, 0x10);
	//OV8858S_write_cmos_sensor( 0x5b02, 0x03);
	//OV8858S_write_cmos_sensor( 0x5b03, 0xcf);
	//OV8858S_write_cmos_sensor( 0x5b05, 0x6c);
	//OV8858S_write_cmos_sensor( 0x5e00, 0x00);
	//OV8858S_write_cmos_sensor( 0x5e01, 0x41);
	//OV8858S_write_cmos_sensor( 0x382d, 0x7f);
	//OV8858S_write_cmos_sensor( 0x4825, 0x3a);
	//OV8858S_write_cmos_sensor( 0x4826, 0x40);
	//OV8858S_write_cmos_sensor( 0x4808, 0x25);
	//OV8858S_write_cmos_sensor( 0x0100, 0x01);
	
	//nick moidify to R2A version 0901
	//@@ OV8858SR2A_Initial_1632x1224_30FPS_MIPI_2LANE(Binning)_672Mbps/lane

OV8858S_write_cmos_sensor(0x0100, 0x00);
OV8858S_write_cmos_sensor(0x0302, 0x1c);
OV8858S_write_cmos_sensor(0x0303, 0x00);
OV8858S_write_cmos_sensor(0x0304, 0x03);
OV8858S_write_cmos_sensor(0x030d, 0x1f);
OV8858S_write_cmos_sensor(0x030e, 0x02);
OV8858S_write_cmos_sensor(0x030f, 0x04);
OV8858S_write_cmos_sensor(0x0312, 0x03);
OV8858S_write_cmos_sensor(0x031e, 0x0c);
OV8858S_write_cmos_sensor(0x3007, 0x80);
OV8858S_write_cmos_sensor(0x3600, 0x00);
OV8858S_write_cmos_sensor(0x3601, 0x00);
OV8858S_write_cmos_sensor(0x3602, 0x00);
OV8858S_write_cmos_sensor(0x3603, 0x00);
OV8858S_write_cmos_sensor(0x3604, 0x22);
OV8858S_write_cmos_sensor(0x3605, 0x20);
OV8858S_write_cmos_sensor(0x3606, 0x00);
OV8858S_write_cmos_sensor(0x3607, 0x20);
OV8858S_write_cmos_sensor(0x3608, 0x11);
OV8858S_write_cmos_sensor(0x3609, 0x28);
OV8858S_write_cmos_sensor(0x360a, 0x00);
OV8858S_write_cmos_sensor(0x360b, 0x05);
OV8858S_write_cmos_sensor(0x360c, 0xd4);
OV8858S_write_cmos_sensor(0x360d, 0x40);
OV8858S_write_cmos_sensor(0x360e, 0x0c);
OV8858S_write_cmos_sensor(0x360f, 0x20);
OV8858S_write_cmos_sensor(0x3610, 0x07);
OV8858S_write_cmos_sensor(0x3611, 0x20);
OV8858S_write_cmos_sensor(0x3612, 0x88);
OV8858S_write_cmos_sensor(0x3613, 0x80);
OV8858S_write_cmos_sensor(0x3614, 0x58);
OV8858S_write_cmos_sensor(0x3615, 0x00);
OV8858S_write_cmos_sensor(0x3616, 0x4a);
OV8858S_write_cmos_sensor(0x3617, 0x40);
OV8858S_write_cmos_sensor(0x3618, 0x5a);
OV8858S_write_cmos_sensor(0x3619, 0x70);
OV8858S_write_cmos_sensor(0x361a, 0x99);
OV8858S_write_cmos_sensor(0x361b, 0x0a);
OV8858S_write_cmos_sensor(0x361c, 0x07);
OV8858S_write_cmos_sensor(0x361d, 0x00);
OV8858S_write_cmos_sensor(0x361e, 0x00);
OV8858S_write_cmos_sensor(0x361f, 0x00);
OV8858S_write_cmos_sensor(0x3638, 0xff);
OV8858S_write_cmos_sensor(0x3633, 0x0f);
OV8858S_write_cmos_sensor(0x3634, 0x0f);
OV8858S_write_cmos_sensor(0x3635, 0x0f);
OV8858S_write_cmos_sensor(0x3636, 0x12);
OV8858S_write_cmos_sensor(0x3645, 0x13);
OV8858S_write_cmos_sensor(0x3646, 0x83);
OV8858S_write_cmos_sensor(0x364a, 0x07);
OV8858S_write_cmos_sensor(0x3015, 0x00);
OV8858S_write_cmos_sensor(0x3018, 0x72);
OV8858S_write_cmos_sensor(0x3020, 0x93);
OV8858S_write_cmos_sensor(0x3022, 0x01);
OV8858S_write_cmos_sensor(0x3031, 0x0a);
OV8858S_write_cmos_sensor(0x3034, 0x00);
OV8858S_write_cmos_sensor(0x3106, 0x01);
OV8858S_write_cmos_sensor(0x3305, 0xf1);
OV8858S_write_cmos_sensor(0x3308, 0x00);
OV8858S_write_cmos_sensor(0x3309, 0x28);
OV8858S_write_cmos_sensor(0x330a, 0x00);
OV8858S_write_cmos_sensor(0x330b, 0x20);
OV8858S_write_cmos_sensor(0x330c, 0x00);
OV8858S_write_cmos_sensor(0x330d, 0x00);
OV8858S_write_cmos_sensor(0x330e, 0x00);
OV8858S_write_cmos_sensor(0x330f, 0x40);
OV8858S_write_cmos_sensor(0x3307, 0x04);
OV8858S_write_cmos_sensor(0x3500, 0x00);
OV8858S_write_cmos_sensor(0x3501, 0x4d);
OV8858S_write_cmos_sensor(0x3502, 0x40);
OV8858S_write_cmos_sensor(0x3503, 0x80);
OV8858S_write_cmos_sensor(0x3505, 0x80);
OV8858S_write_cmos_sensor(0x3508, 0x02);
OV8858S_write_cmos_sensor(0x3509, 0x00);
OV8858S_write_cmos_sensor(0x350c, 0x00);
OV8858S_write_cmos_sensor(0x350d, 0x80);
OV8858S_write_cmos_sensor(0x3510, 0x00);
OV8858S_write_cmos_sensor(0x3511, 0x02);
OV8858S_write_cmos_sensor(0x3512, 0x00);
OV8858S_write_cmos_sensor(0x3700, 0x18);
OV8858S_write_cmos_sensor(0x3701, 0x0c);
OV8858S_write_cmos_sensor(0x3702, 0x28);
OV8858S_write_cmos_sensor(0x3703, 0x19);
OV8858S_write_cmos_sensor(0x3704, 0x14);
OV8858S_write_cmos_sensor(0x3705, 0x00);
OV8858S_write_cmos_sensor(0x3706, 0x82);
OV8858S_write_cmos_sensor(0x3707, 0x04);
OV8858S_write_cmos_sensor(0x3708, 0x24);
OV8858S_write_cmos_sensor(0x3709, 0x33);
OV8858S_write_cmos_sensor(0x370a, 0x01);
OV8858S_write_cmos_sensor(0x370b, 0x82);
OV8858S_write_cmos_sensor(0x370c, 0x04);
OV8858S_write_cmos_sensor(0x3718, 0x12);
OV8858S_write_cmos_sensor(0x3719, 0x31);
OV8858S_write_cmos_sensor(0x3712, 0x42);
OV8858S_write_cmos_sensor(0x3714, 0x24);
OV8858S_write_cmos_sensor(0x371e, 0x19);
OV8858S_write_cmos_sensor(0x371f, 0x40);
OV8858S_write_cmos_sensor(0x3720, 0x05);
OV8858S_write_cmos_sensor(0x3721, 0x05);
OV8858S_write_cmos_sensor(0x3724, 0x06);
OV8858S_write_cmos_sensor(0x3725, 0x01);
OV8858S_write_cmos_sensor(0x3726, 0x06);
OV8858S_write_cmos_sensor(0x3728, 0x05);
OV8858S_write_cmos_sensor(0x3729, 0x02);
OV8858S_write_cmos_sensor(0x372a, 0x03);
OV8858S_write_cmos_sensor(0x372b, 0x53);
OV8858S_write_cmos_sensor(0x372c, 0xa3);
OV8858S_write_cmos_sensor(0x372d, 0x53);
OV8858S_write_cmos_sensor(0x372e, 0x06);
OV8858S_write_cmos_sensor(0x372f, 0x10);
OV8858S_write_cmos_sensor(0x3730, 0x01);
OV8858S_write_cmos_sensor(0x3731, 0x06);
OV8858S_write_cmos_sensor(0x3732, 0x14);
OV8858S_write_cmos_sensor(0x3733, 0x10);
OV8858S_write_cmos_sensor(0x3734, 0x40);
OV8858S_write_cmos_sensor(0x3736, 0x20);
OV8858S_write_cmos_sensor(0x373a, 0x05);
OV8858S_write_cmos_sensor(0x373b, 0x06);
OV8858S_write_cmos_sensor(0x373c, 0x0a);
OV8858S_write_cmos_sensor(0x373e, 0x03);
OV8858S_write_cmos_sensor(0x3750, 0x0a);
OV8858S_write_cmos_sensor(0x3751, 0x0e);
OV8858S_write_cmos_sensor(0x3755, 0x10);
OV8858S_write_cmos_sensor(0x3758, 0x00);
OV8858S_write_cmos_sensor(0x3759, 0x4c);
OV8858S_write_cmos_sensor(0x375a, 0x06);
OV8858S_write_cmos_sensor(0x375b, 0x13);
OV8858S_write_cmos_sensor(0x375c, 0x20);
OV8858S_write_cmos_sensor(0x375d, 0x02);
OV8858S_write_cmos_sensor(0x375e, 0x00);
OV8858S_write_cmos_sensor(0x375f, 0x14);
OV8858S_write_cmos_sensor(0x3768, 0xcc);
OV8858S_write_cmos_sensor(0x3769, 0x44);
OV8858S_write_cmos_sensor(0x376a, 0x44);
OV8858S_write_cmos_sensor(0x3761, 0x00);
OV8858S_write_cmos_sensor(0x3762, 0x00);
OV8858S_write_cmos_sensor(0x3763, 0x18);
OV8858S_write_cmos_sensor(0x3766, 0xff);
OV8858S_write_cmos_sensor(0x376b, 0x00);
OV8858S_write_cmos_sensor(0x3772, 0x23);
OV8858S_write_cmos_sensor(0x3773, 0x02);
OV8858S_write_cmos_sensor(0x3774, 0x16);
OV8858S_write_cmos_sensor(0x3775, 0x12);
OV8858S_write_cmos_sensor(0x3776, 0x04);
OV8858S_write_cmos_sensor(0x3777, 0x00);
OV8858S_write_cmos_sensor(0x3778, 0x17);
OV8858S_write_cmos_sensor(0x37a0, 0x44);
OV8858S_write_cmos_sensor(0x37a1, 0x3d);
OV8858S_write_cmos_sensor(0x37a2, 0x3d);
OV8858S_write_cmos_sensor(0x37a3, 0x00);
OV8858S_write_cmos_sensor(0x37a4, 0x00);
OV8858S_write_cmos_sensor(0x37a5, 0x00);
OV8858S_write_cmos_sensor(0x37a6, 0x00);
OV8858S_write_cmos_sensor(0x37a7, 0x44);
OV8858S_write_cmos_sensor(0x37a8, 0x4c);
OV8858S_write_cmos_sensor(0x37a9, 0x4c);
OV8858S_write_cmos_sensor(0x3760, 0x00);
OV8858S_write_cmos_sensor(0x376f, 0x01);
OV8858S_write_cmos_sensor(0x37aa, 0x44);
OV8858S_write_cmos_sensor(0x37ab, 0x2e);
OV8858S_write_cmos_sensor(0x37ac, 0x2e);
OV8858S_write_cmos_sensor(0x37ad, 0x33);
OV8858S_write_cmos_sensor(0x37ae, 0x0d);
OV8858S_write_cmos_sensor(0x37af, 0x0d);
OV8858S_write_cmos_sensor(0x37b0, 0x00);
OV8858S_write_cmos_sensor(0x37b1, 0x00);
OV8858S_write_cmos_sensor(0x37b2, 0x00);
OV8858S_write_cmos_sensor(0x37b3, 0x42);
OV8858S_write_cmos_sensor(0x37b4, 0x42);
OV8858S_write_cmos_sensor(0x37b5, 0x31);
OV8858S_write_cmos_sensor(0x37b6, 0x00);
OV8858S_write_cmos_sensor(0x37b7, 0x00);
OV8858S_write_cmos_sensor(0x37b8, 0x00);
OV8858S_write_cmos_sensor(0x37b9, 0xff);
OV8858S_write_cmos_sensor(0x3800, 0x00);
OV8858S_write_cmos_sensor(0x3801, 0x0c);
OV8858S_write_cmos_sensor(0x3802, 0x00);
OV8858S_write_cmos_sensor(0x3803, 0x0c);
OV8858S_write_cmos_sensor(0x3804, 0x0c);
OV8858S_write_cmos_sensor(0x3805, 0xd3);
OV8858S_write_cmos_sensor(0x3806, 0x09);
OV8858S_write_cmos_sensor(0x3807, 0xa3);
OV8858S_write_cmos_sensor(0x3808, 0x06);
OV8858S_write_cmos_sensor(0x3809, 0x60);
OV8858S_write_cmos_sensor(0x380a, 0x04);
OV8858S_write_cmos_sensor(0x380b, 0xc8);
OV8858S_write_cmos_sensor(0x380c, 0x07);
OV8858S_write_cmos_sensor(0x380d, 0x88);
OV8858S_write_cmos_sensor(0x380e, 0x04);
OV8858S_write_cmos_sensor(0x380f, 0xdc);
OV8858S_write_cmos_sensor(0x3810, 0x00);
OV8858S_write_cmos_sensor(0x3811, 0x04);
OV8858S_write_cmos_sensor(0x3813, 0x02);
OV8858S_write_cmos_sensor(0x3814, 0x03);
OV8858S_write_cmos_sensor(0x3815, 0x01);
OV8858S_write_cmos_sensor(0x3820, 0x00);
OV8858S_write_cmos_sensor(0x3821, 0x67);
OV8858S_write_cmos_sensor(0x382a, 0x03);
OV8858S_write_cmos_sensor(0x382b, 0x01);
OV8858S_write_cmos_sensor(0x3830, 0x08);
OV8858S_write_cmos_sensor(0x3836, 0x02);
OV8858S_write_cmos_sensor(0x3837, 0x18);
OV8858S_write_cmos_sensor(0x3841, 0xff);
OV8858S_write_cmos_sensor(0x3846, 0x48);
OV8858S_write_cmos_sensor(0x3d85, 0x16);
OV8858S_write_cmos_sensor(0x3d8c, 0x73);
OV8858S_write_cmos_sensor(0x3d8d, 0xde);
OV8858S_write_cmos_sensor(0x3f08, 0x08);
OV8858S_write_cmos_sensor(0x3f0a, 0x00);
OV8858S_write_cmos_sensor(0x4000, 0xf1);
OV8858S_write_cmos_sensor(0x4001, 0x10);
OV8858S_write_cmos_sensor(0x4005, 0x10);
OV8858S_write_cmos_sensor(0x4002, 0x27);
OV8858S_write_cmos_sensor(0x4009, 0x83);
OV8858S_write_cmos_sensor(0x400a, 0x01);
OV8858S_write_cmos_sensor(0x400b, 0x0c);
OV8858S_write_cmos_sensor(0x400d, 0x10);
OV8858S_write_cmos_sensor(0x4011, 0x20);
OV8858S_write_cmos_sensor(0x401b, 0x00);
OV8858S_write_cmos_sensor(0x401d, 0x00);
OV8858S_write_cmos_sensor(0x4020, 0x00);
OV8858S_write_cmos_sensor(0x4021, 0x04);
OV8858S_write_cmos_sensor(0x4022, 0x04);
OV8858S_write_cmos_sensor(0x4023, 0xb9);
OV8858S_write_cmos_sensor(0x4024, 0x05);
OV8858S_write_cmos_sensor(0x4025, 0x2a);
OV8858S_write_cmos_sensor(0x4026, 0x05);
OV8858S_write_cmos_sensor(0x4027, 0x2b);
OV8858S_write_cmos_sensor(0x4028, 0x00);
OV8858S_write_cmos_sensor(0x4029, 0x02);
OV8858S_write_cmos_sensor(0x402a, 0x04);
OV8858S_write_cmos_sensor(0x402b, 0x04);
OV8858S_write_cmos_sensor(0x402c, 0x02);
OV8858S_write_cmos_sensor(0x402d, 0x02);
OV8858S_write_cmos_sensor(0x402e, 0x08);
OV8858S_write_cmos_sensor(0x402f, 0x02);
OV8858S_write_cmos_sensor(0x401f, 0x00);
OV8858S_write_cmos_sensor(0x4034, 0x3f);
OV8858S_write_cmos_sensor(0x403d, 0x04);
OV8858S_write_cmos_sensor(0x403e, 0x08);
OV8858S_write_cmos_sensor(0x4040, 0x07);
OV8858S_write_cmos_sensor(0x4041, 0xc6);
OV8858S_write_cmos_sensor(0x4300, 0xff);
OV8858S_write_cmos_sensor(0x4202, 0x00);
OV8858S_write_cmos_sensor(0x4301, 0x00);
OV8858S_write_cmos_sensor(0x4302, 0x0f);
OV8858S_write_cmos_sensor(0x4316, 0x00);
OV8858S_write_cmos_sensor(0x4500, 0x58);
OV8858S_write_cmos_sensor(0x4503, 0x18);
OV8858S_write_cmos_sensor(0x4600, 0x00);
OV8858S_write_cmos_sensor(0x4601, 0xcb);
OV8858S_write_cmos_sensor(0x470b, 0x28);
OV8858S_write_cmos_sensor(0x481f, 0x32);
OV8858S_write_cmos_sensor(0x4837, 0x17);
OV8858S_write_cmos_sensor(0x4850, 0x10);
OV8858S_write_cmos_sensor(0x4851, 0x32);
OV8858S_write_cmos_sensor(0x4b00, 0x2a);
OV8858S_write_cmos_sensor(0x4b0d, 0x00);
OV8858S_write_cmos_sensor(0x4d00, 0x04);
OV8858S_write_cmos_sensor(0x4d01, 0x18);
OV8858S_write_cmos_sensor(0x4d02, 0xc3);
OV8858S_write_cmos_sensor(0x4d03, 0xff);
OV8858S_write_cmos_sensor(0x4d04, 0xff);
OV8858S_write_cmos_sensor(0x4d05, 0xff);
OV8858S_write_cmos_sensor(0x5000, 0x7e);
OV8858S_write_cmos_sensor(0x5001, 0x01);
OV8858S_write_cmos_sensor(0x5002, 0x08);
OV8858S_write_cmos_sensor(0x5003, 0x20);
OV8858S_write_cmos_sensor(0x5046, 0x12);
OV8858S_write_cmos_sensor(0x5780, 0x3e);
OV8858S_write_cmos_sensor(0x5781, 0x0f);
OV8858S_write_cmos_sensor(0x5782, 0x44);
OV8858S_write_cmos_sensor(0x5783, 0x02);
OV8858S_write_cmos_sensor(0x5784, 0x01);
OV8858S_write_cmos_sensor(0x5785, 0x00);
OV8858S_write_cmos_sensor(0x5786, 0x00);
OV8858S_write_cmos_sensor(0x5787, 0x04);
OV8858S_write_cmos_sensor(0x5788, 0x02);
OV8858S_write_cmos_sensor(0x5789, 0x0f);
OV8858S_write_cmos_sensor(0x578a, 0xfd);
OV8858S_write_cmos_sensor(0x578b, 0xf5);
OV8858S_write_cmos_sensor(0x578c, 0xf5);
OV8858S_write_cmos_sensor(0x578d, 0x03);
OV8858S_write_cmos_sensor(0x578e, 0x08);
OV8858S_write_cmos_sensor(0x578f, 0x0c);
OV8858S_write_cmos_sensor(0x5790, 0x08);
OV8858S_write_cmos_sensor(0x5791, 0x04);
OV8858S_write_cmos_sensor(0x5792, 0x00);
OV8858S_write_cmos_sensor(0x5793, 0x52);
OV8858S_write_cmos_sensor(0x5794, 0xa3);
OV8858S_write_cmos_sensor(0x5871, 0x0d);
OV8858S_write_cmos_sensor(0x5870, 0x18);
OV8858S_write_cmos_sensor(0x586e, 0x10);
OV8858S_write_cmos_sensor(0x586f, 0x08);
OV8858S_write_cmos_sensor(0x58f8, 0x3d);
OV8858S_write_cmos_sensor(0x5901, 0x00);
OV8858S_write_cmos_sensor(0x5b00, 0x02);
OV8858S_write_cmos_sensor(0x5b01, 0x10);
OV8858S_write_cmos_sensor(0x5b02, 0x03);
OV8858S_write_cmos_sensor(0x5b03, 0xcf);
OV8858S_write_cmos_sensor(0x5b05, 0x6c);
OV8858S_write_cmos_sensor(0x5e00, 0x00);
OV8858S_write_cmos_sensor(0x5e01, 0x41);
OV8858S_write_cmos_sensor(0x382d, 0x7f);
OV8858S_write_cmos_sensor(0x4825, 0x3a);
OV8858S_write_cmos_sensor(0x4826, 0x40);
OV8858S_write_cmos_sensor(0x4808, 0x25);
OV8858S_write_cmos_sensor(0x0100, 0x01);
	                                 
	
	//otp apply by o-film 2014-3-28
	#if GN_MTK_BSP_LENS_COMPABILITY 
	//#if 1  //OTP AWB Start
	OV8858S_write_cmos_sensor( 0x5002, 0x00);
	mdelay(2);
			OV8858S_mipi_update_wb_register_from_otp();
	//#endif //OTP AWB end
	//#if 1 //otp lens shading start
			OV8858S_mipi_update_lenc_register_from_otp();
	OV8858S_write_cmos_sensor( 0x5002, 0x08);
	mdelay(2);
	//#endif //otp lens shading end
	#endif


}


UINT32 OV8858SOpen(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;

	OV8858SDB("OV8858S Open enter :\n ");
	OV8858S_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mdelay(2);

	for(i=0;i<2;i++)
	{
		sensor_id = (OV8858S_read_cmos_sensor(0x300B)<<8)|OV8858S_read_cmos_sensor(0x300C);
		OV8858SDB("OV8858S READ ID :%x",sensor_id);
		if(sensor_id != OV8858_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}else
			break;
	}
	
	OV8858S_Sensor_Init();
	mdelay(40);
    OV8858S_Init_Para();
	
	#ifdef OV8858S_DEBUG
		OV8858SDB("[OV8858SOpen] enter and exit."); 
	#endif

    return ERROR_NONE;
}


//Gionee yanggy 2013-09-03 add for module compability begin
#ifdef GN_MTK_BSP_LENS_COMPABILITY
static int OV8858S_MIPI_GetModuleInfo(unsigned int *module_id, unsigned int *lens_id)
{
    kal_uint16 temp, i, otp_index,bank;
    kal_uint32 address;
		OV8858S_write_cmos_sensor( 0x5002, 0x00);
	mdelay(2);
    //check first wb OTP with valid OTP
    for(i = 1; i < 4; i++)
    {
        temp = OV8858S_mipi_check_otp_info(i);
        if(temp == 2)
        {
            otp_index = i;
            break;
        }
    }
    if( i == 4)
    {
        OV8858SDB("[OV8858S_mipi_otp_info error]\r\n");
        return -1;
    }

		if(OV8858S_mipi_read_otp_info(otp_index,&current_otp))
			{
				return 0;
			}

    *module_id = current_otp.module_integrator_id;
		*lens_id =current_otp.lens_id;

    printk("[OV8858S_MIPI_GetModuleInfo] the module id is : 0x%02x , the lens id is :0x%02x ~~~\n",*module_id,*lens_id );
    return 0;
		OV8858S_write_cmos_sensor( 0x5002, 0x08);
	mdelay(2);

}
#endif
//Gionee yanggy 2013-09-03 add for module compability end

// [LGE_UPDATE_S][yonghwan.lym@lge.com][2015-01-29] LGIT and Sunny module check for Y90
#if defined(TARGET_MT6582_Y90)
extern int module_flag; // 1:Sunny Module, 0:LGIT Module
#endif
// [LGE_UPDATE_E][yonghwan.lym@lge.com][2015-01-29] LGIT and Sunny module check for Y90

UINT32 OV8858SGetSensorID(UINT32 *sensorID)
{
// [LGE_UPDATE_S][yonghwan.lym@lge.com][2015-01-29] LGIT and Sunny module check for Y90
#if defined(TARGET_MT6582_Y90)
    if(module_flag != 1)
    {
        printk("Please check the flag and module. module_flag = %d ...(flag 1 => SUNNY, flag 0 => LGIT)\n",module_flag);
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
#endif
// [LGE_UPDATE_E][yonghwan.lym@lge.com][2015-01-29] LGIT and Sunny module check for Y90
    int  retry = 2, ret;
    int i;
    unsigned int CurrentSensorIDInList;
#if 0//defined(MUSE6582_Y90_L)

    u16 backup_slave_addr;

#endif 


//Gionee yanggy 2013-09-03 add for module compbility begin
#ifdef GN_MTK_BSP_LENS_COMPABILITY
    unsigned int module_id,lens_id;
#endif
//Gionee yanggy 2013-09-03 add for module compbility end

#if 0//defined(MUSE6582_Y90_L)

    CurrentSensorIDInList = *sensorID;

    OV8858SDB("Y90 init Sensor ID = 0x%04x(0x%04x)\n", *sensorID,CurrentSensorIDInList);
    *sensorID =0;
#else
    CurrentSensorIDInList =OV8858S_SENSOR_ID;
#endif  


	OV8858SDB("OV8858SGetSensorID enter :\n ");


#if 0//defined(MUSE6582_Y90_L)
    
        kal_uint16 sccb_writeid[3] = {0,};

        if(CurrentSensorIDInList == OV8858S_SENSOR_ID)
        {
            sccb_writeid[0] = OV8858SMIPI_WRITE_ID;
        }
        else if(CurrentSensorIDInList == OV8858SL_SENSOR_ID)
        {
            sccb_writeid[0] = OV8858SMIPI_WRITE_ID1;
        }
        else
        {
            sccb_writeid[0] = OV8858SMIPI_WRITE_ID;
            sccb_writeid[1] = OV8858SMIPI_WRITE_ID1;
            sccb_writeid[2] = OV8858SMIPI_WRITE_ID2;
        }
        backup_slave_addr = OV8858S.write_id;
        
#else
        const kal_uint16 sccb_writeid[] = {OV8858SMIPI_WRITE_ID};
#endif      
    ret = FALSE;

    for (i=0;i<(sizeof(sccb_writeid)/sizeof(sccb_writeid[0]));i++){
    do {
        OV8858S.write_id = sccb_writeid[i];
        OV8858S.read_id = (sccb_writeid[i]|0x01);
        *sensorID = (OV8858S_read_cmos_sensor(0x300B)<<8)|OV8858S_read_cmos_sensor(0x300C);
        if (*sensorID == OV8858_SENSOR_ID)
        	{
        		OV8858SDB("Sensor ID = 0x%04x\n", *sensorID);
                i = 3;
                ret = TRUE;
            	break;
        	}
        OV8858SDB("[OV8858SGetSensorID] Read Sensor ID Fail = 0x%04x ,i2c addr is:0x%02x====\n ", *sensorID,OV8858S.write_id);
        retry--;
    } while (retry > 0);
    }

    #if 0//defined(MUSE6582_Y90_L)

    if(ret == TRUE)
    {

        if(sccb_writeid[0] == 0x6c)
        {
        	*sensorID+=1;
        }
    }
    else
    {
        OV8858S.write_id = backup_slave_addr;
        OV8858S.read_id = (backup_slave_addr|0x01);
    }

    OV8858SDB("Y90 final Sensor ID = 0x%04x\n", *sensorID);
    
    #else
		if(ret == TRUE)
    {
        *sensorID+=1;        
        OV8858SDB("Y90 final Sunny Sensor ID = 0x%04x\n", *sensorID);
    }
    #endif    

#if 1    

    if (*sensorID != OV8858S_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
#else
    if (*sensorID != CurrentSensorIDInList) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
#endif



//Gionee yanggy 2013-09-03 add for module compbility begin
#ifdef GN_MTK_BSP_LENS_COMPABILITY
    OV8858S_Sensor_Init();
    OV8858S_MIPI_GetModuleInfo(&module_id, &lens_id);
    OV8858S_MIPI_Module_id = module_id;
    OV8858S_MIPI_Module_Lens_id = lens_id;
 
    if(0x01 != module_id)
    {
        *sensorID = 0xFFFFFFFF;
        OV8858SDB("[OV8858S_MIPIGetSensorID] the module_id :0x%02x is not correct !!!!\n",module_id);
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    else
    {
        *sensorID = OV8858S_SENSOR_ID;
        OV8858SDB("[OV8858S_MIPIGetSensorID] the module_id is :0x%02x ,the lens_id is :0x%02x ~~~\n",module_id ,lens_id );

    }
#endif
//Gionee yanggy 2013-09-03 add for module compbility end	

    return ERROR_NONE;
}

UINT32 OV8858SClose(void)
{
	#ifdef OV8858S_DEBUG
		OV8858SDB("[OV8858SClose]enter and exit.\n");
	#endif

    return ERROR_NONE;
}

#if 1
void OV8858SSetFlipMirror(kal_int32 imgMirror)
{
	kal_int16 mirror=0,flip=0;
	mirror= OV8858S_read_cmos_sensor(0x3820);
	flip  = OV8858S_read_cmos_sensor(0x3821);

    switch (imgMirror)
    {
        case IMAGE_H_MIRROR://IMAGE_NORMAL:
            OV8858S_write_cmos_sensor(0x3820, (mirror & (0xF9)));//Set normal
            OV8858S_write_cmos_sensor(0x3821, (flip & (0xF9)));	//Set normal
            break;
        case IMAGE_NORMAL://IMAGE_V_MIRROR:
            OV8858S_write_cmos_sensor(0x3820, (mirror & (0xF9)));//Set flip
            OV8858S_write_cmos_sensor(0x3821, (flip | (0x06)));	//Set flip
            break;
        case IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
            OV8858S_write_cmos_sensor(0x3820, (mirror |(0x06)));	//Set mirror
            OV8858S_write_cmos_sensor(0x3821, (flip & (0xF9)));	//Set mirror
            break;
        case IMAGE_V_MIRROR://IMAGE_HV_MIRROR:
            OV8858S_write_cmos_sensor(0x3820, (mirror |(0x06)));	//Set mirror & flip
            OV8858S_write_cmos_sensor(0x3821, (flip |(0x06)));	//Set mirror & flip
            break;
    }
}
#endif

kal_uint32 OV8858S_SET_FrameLength_ByVideoMode(UINT16 Video_TargetFps)
{

    UINT32 frameRate = 0;
	kal_uint32 MIN_FrameLength=0;
	
	if(OV8858S.OV8858SAutoFlickerMode == KAL_TRUE)
	{
		if (Video_TargetFps==30)
			frameRate= OV8858S_AUTOFLICKER_OFFSET_30;
		else if(Video_TargetFps==15)
			frameRate= OV8858S_AUTOFLICKER_OFFSET_15;
		else
			frameRate=Video_TargetFps*10;
	
		MIN_FrameLength = (OV8858S.videoPclk*10000)/(OV8858S_VIDEO_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels)/frameRate*10;
	}
	else
		MIN_FrameLength = (OV8858S.videoPclk*10000) /(OV8858S_VIDEO_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels)/Video_TargetFps;

     return MIN_FrameLength;


}



UINT32 OV8858SSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    OV8858SDB("[OV8858SSetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&OV8858Smipiraw_drv_lock);
	OV8858S_VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&OV8858Smipiraw_drv_lock);

	if(u2FrameRate==0)
	{
		OV8858SDB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    OV8858SDB("abmornal frame rate seting,pay attention~\n");

    if(OV8858S.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {

        MIN_Frame_length = OV8858S_SET_FrameLength_ByVideoMode(u2FrameRate);

		if((MIN_Frame_length <=OV8858S_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = OV8858S_VIDEO_PERIOD_LINE_NUMS;
			OV8858SDB("[OV8858SSetVideoMode]current fps = %d\n", (OV8858S.videoPclk*10000)  /(OV8858S_VIDEO_PERIOD_PIXEL_NUMS)/OV8858S_VIDEO_PERIOD_LINE_NUMS);
		}
		OV8858SDB("[OV8858SSetVideoMode]current fps (10 base)= %d\n", (OV8858S.videoPclk*10000)*10/(OV8858S_VIDEO_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels)/MIN_Frame_length);
		extralines = MIN_Frame_length - OV8858S_VIDEO_PERIOD_LINE_NUMS;
		
		spin_lock(&OV8858Smipiraw_drv_lock);
		OV8858S.DummyPixels = 0;//define dummy pixels and lines
		OV8858S.DummyLines = extralines ;
		spin_unlock(&OV8858Smipiraw_drv_lock);
		
		OV8858S_SetDummy(0, extralines);
    }
	
	OV8858SDB("[OV8858SSetVideoMode]MIN_Frame_length=%d,OV8858S.DummyLines=%d\n",MIN_Frame_length,OV8858S.DummyLines);

    return KAL_TRUE;
}


UINT32 OV8858SSetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	if(bEnable) {   
		spin_lock(&OV8858Smipiraw_drv_lock);
		OV8858S.OV8858SAutoFlickerMode = KAL_TRUE;
		spin_unlock(&OV8858Smipiraw_drv_lock);
        OV8858SDB("OV8858S Enable Auto flicker\n");
    } else {
    	spin_lock(&OV8858Smipiraw_drv_lock);
        OV8858S.OV8858SAutoFlickerMode = KAL_FALSE;
		spin_unlock(&OV8858Smipiraw_drv_lock);
        OV8858SDB("OV8858S Disable Auto flicker\n");
    }

    return ERROR_NONE;
}


UINT32 OV8858SSetTestPatternMode(kal_bool bEnable)
{
    OV8858SDB("[OV8858SSetTestPatternMode] Test pattern enable:%d\n", bEnable);
    if(bEnable == KAL_TRUE)
    {
        OV8858S_During_testpattern = KAL_TRUE;
		OV8858S_write_cmos_sensor(0x5E00,0x80);
    }
	else
	{
        OV8858S_During_testpattern = KAL_FALSE;
		OV8858S_write_cmos_sensor(0x5E00,0x00);
	}

    return ERROR_NONE;
}


/*************************************************************************
*
* DESCRIPTION:
* INTERFACE FUNCTION, FOR USER TO SET MAX  FRAMERATE;
* 
*************************************************************************/
UINT32 OV8858SMIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	OV8858SDB("OV8858SMIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = OV8858S_PREVIEW_PCLK;
			lineLength = OV8858S_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8858S_PV_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&OV8858Smipiraw_drv_lock);
			OV8858S.sensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock(&OV8858Smipiraw_drv_lock);
			OV8858S_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = OV8858S_VIDEO_PCLK;
			lineLength = OV8858S_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8858S_VIDEO_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&OV8858Smipiraw_drv_lock);
			OV8858S.sensorMode = SENSOR_MODE_VIDEO;
			spin_unlock(&OV8858Smipiraw_drv_lock);
			OV8858S_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = OV8858S_CAPTURE_PCLK;
			lineLength = OV8858S_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8858S_FULL_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&OV8858Smipiraw_drv_lock);
			OV8858S.sensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock(&OV8858Smipiraw_drv_lock);
			OV8858S_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;

}


UINT32 OV8858SMIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = OV8858S_MAX_FPS_PREVIEW;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = OV8858S_MAX_FPS_CAPTURE;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = OV8858S_MAX_FPS_CAPTURE;
			break;		
		default:
			break;
	}

	return ERROR_NONE;

}


void OV8858S_NightMode(kal_bool bEnable)
{
	
	#ifdef OV8858S_DEBUG
	OV8858SDB("[OV8858S_NightMode]enter and exit.\n");
	#endif
}



UINT32 OV8858SPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	OV8858SDB("OV8858SPreview enter:");

	OV8858SPreviewSetting();

	spin_lock(&OV8858Smipiraw_drv_lock);
	OV8858S.sensorMode = SENSOR_MODE_PREVIEW; 
	OV8858S.DummyPixels = 0;
	OV8858S.DummyLines = 0 ;
	OV8858S_FeatureControl_PERIOD_PixelNum=OV8858S_PV_PERIOD_PIXEL_NUMS+ OV8858S.DummyPixels;
	OV8858S_FeatureControl_PERIOD_LineNum=OV8858S_PV_PERIOD_LINE_NUMS+OV8858S.DummyLines;
	//TODO~
	//OV8858S.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&OV8858Smipiraw_drv_lock);
	
	//OV8858SSetFlipMirror(sensor_config_data->SensorImageMirror);
	OV8858SSetFlipMirror(IMAGE_HV_MIRROR);
	//TODO~
    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
	OV8858SDB("OV8858SPreview exit:\n");

	  
    return ERROR_NONE;
}


UINT32 OV8858SVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	OV8858SDB("OV8858SVideo enter:");

	OV8858SVideoSetting();

	spin_lock(&OV8858Smipiraw_drv_lock);
	OV8858S.sensorMode = SENSOR_MODE_VIDEO;
	OV8858S_FeatureControl_PERIOD_PixelNum=OV8858S_VIDEO_PERIOD_PIXEL_NUMS+ OV8858S.DummyPixels;
	OV8858S_FeatureControl_PERIOD_LineNum=OV8858S_VIDEO_PERIOD_LINE_NUMS+OV8858S.DummyLines;
	OV8858S.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&OV8858Smipiraw_drv_lock);
	
	//OV8865SetFlipMirror(sensor_config_data->SensorImageMirror);
	OV8858SSetFlipMirror(IMAGE_HV_MIRROR);

    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
	OV8858SDB("OV8865Video exit:\n");
    return ERROR_NONE;
}


UINT32 OV8858SCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	kal_uint32 shutter = OV8858S.shutter;
	kal_uint32 temp_data;


	OV8858SDB("OV8858SCapture enter:\n");

	OV8858SCaptureSetting();

	spin_lock(&OV8858Smipiraw_drv_lock);
	OV8858S.sensorMode = SENSOR_MODE_CAPTURE;
	//TODO~
	//OV8858S.imgMirror = sensor_config_data->SensorImageMirror;
	OV8858S.DummyPixels = 0;
	OV8858S.DummyLines = 0 ;
	OV8858S_FeatureControl_PERIOD_PixelNum = OV8858S_FULL_PERIOD_PIXEL_NUMS + OV8858S.DummyPixels;
	OV8858S_FeatureControl_PERIOD_LineNum = OV8858S_FULL_PERIOD_LINE_NUMS + OV8858S.DummyLines;
	spin_unlock(&OV8858Smipiraw_drv_lock);

	//OV8865SetFlipMirror(sensor_config_data->SensorImageMirror);
	OV8858SSetFlipMirror(IMAGE_HV_MIRROR);
    mdelay(40);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY

	#if 0
	if(OV8858S_During_testpattern == KAL_TRUE)
	{
		//TODO~
		//Test pattern
		OV8858S_write_cmos_sensor(0x5E00,0x80);
	}
	#endif
	OV8858SDB("OV8865Capture exit:\n");
    return ERROR_NONE;

}	

#if 0
#endif

UINT32 OV8858SGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    OV8858SDB("OV8858SGetResolution!!\n");

	pSensorResolution->SensorPreviewWidth	= OV8858S_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= OV8858S_IMAGE_SENSOR_PV_HEIGHT;
	
    pSensorResolution->SensorFullWidth		= OV8858S_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= OV8858S_IMAGE_SENSOR_FULL_HEIGHT;
	
    pSensorResolution->SensorVideoWidth		= OV8858S_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = OV8858S_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   

UINT32 OV8858SGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    OV8858SDB("OV8858SGetInfo enter!!\n");
	spin_lock(&OV8858Smipiraw_drv_lock);
	OV8858S.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&OV8858Smipiraw_drv_lock);

    pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
   
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

//Gionee yanggy modify for CAM Performance Optimization begin
#ifdef ORIGINAL_VERSION
    pSensorInfo->CaptureDelayFrame = 3;
    pSensorInfo->PreviewDelayFrame = 3;
#else
    pSensorInfo->CaptureDelayFrame = 1;
    pSensorInfo->PreviewDelayFrame = 3;//1; //[LGE_UPDATE][yonghwan.lym@lge.com][2015-03-27] occurs "purple noise" after flash capture
#endif
//Gionee yanggy modify for CAM Performance Optimization end
    pSensorInfo->VideoDelayFrame = 3;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;	    
    pSensorInfo->AESensorGainDelayFrame = 0;
    pSensorInfo->AEISPGainDelayFrame = 2;
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
	pSensorInfo->MIPIsensorType = MIPI_OPHY_CSI2;


    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858S_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV8858S_PV_Y_START;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858S_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = OV8858S_VIDEO_Y_START;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;//0,4,14,32,40
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858S_FULL_X_START;	
            pSensorInfo->SensorGrabStartY = OV8858S_FULL_Y_START;	

            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8858S_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV8858S_PV_Y_START;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }
	
//Gionee yanggy 2013-09-03 add for module compability begin
#ifdef GN_MTK_BSP_LENS_COMPABILITY
    pSensorInfo->Sensor_Lens_id = OV8858S_MIPI_Module_Lens_id;
    pSensorInfo->Sensor_Module_id = OV8858S_MIPI_Module_id;
#endif
//Gionee yanggy 2013-09-03 add for module compability end

    memcpy(pSensorConfigData, &OV8858SSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    OV8858SDB("OV8858SGetInfo exit!!\n");

    return ERROR_NONE;
}   /* OV8858SGetInfo() */



UINT32 OV8858SControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&OV8858Smipiraw_drv_lock);
		OV8858SCurrentScenarioId = ScenarioId;
		spin_unlock(&OV8858Smipiraw_drv_lock);
		
		OV8858SDB("[OV8858SControl]OV8858SCurrentScenarioId=%d\n",OV8858SCurrentScenarioId);

	switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            OV8858SPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			OV8858SVideo(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            OV8858SCapture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* OV8858SControl() */


UINT32 OV8858SFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= OV8858S_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= OV8858S_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= OV8858S_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= OV8858S_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(OV8858SCurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = OV8858S_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = OV8858S_VIDEO_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV8858S_CAPTURE_PCLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV8858S_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            OV8858S_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            OV8858S_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:  
           	OV8858S_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //OV8858S_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            OV8858S_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = OV8858S_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&OV8858Smipiraw_drv_lock);
                OV8858SSensorCCT[i].Addr=*pFeatureData32++;
                OV8858SSensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&OV8858Smipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV8858SSensorCCT[i].Addr;
                *pFeatureData32++=OV8858SSensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&OV8858Smipiraw_drv_lock);
                OV8858SSensorReg[i].Addr=*pFeatureData32++;
                OV8858SSensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&OV8858Smipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV8858SSensorReg[i].Addr;
                *pFeatureData32++=OV8858SSensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=OV8858S_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, OV8858SSensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, OV8858SSensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &OV8858SSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            OV8858S_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            OV8858S_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=OV8858S_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            OV8858S_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            OV8858S_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            OV8858S_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
			//TODO~
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
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
			OV8858SSetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            OV8858SGetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			//TODO~
			OV8858SSetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV8858SMIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			OV8858SMIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			//TODO~
			OV8858SSetTestPatternMode((BOOL)*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing 			
			*pFeatureReturnPara32=OV8858S_TEST_PATTERN_CHECKSUM; 		  
			*pFeatureParaLen=4; 							
		     break;
			 //[LGE_UPDATE_S][yonghwan.lym@lge.com][2014-12-24] add FOV value.
		case SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:
			 {
				 UINT32 *pHorFOV = (UINT32*)pFeatureReturnPara32[0];
				 UINT32 *pVerFOV = (UINT32*)pFeatureReturnPara32[1];

				 OV8858SDB("SENSOR_FEATURE_GET_SENSOR_VIEWANGLE\n");
				 *pHorFOV = (0x3F)<<16 | (0x3F); // 16:9 = 63, 4:3 = 63 // (63)<<16 | (63)
				 *pVerFOV = (0x31)<<16 | (0x31); // 16:9 = 49, 4:3 = 49 // (49)<<16 | (49)
				 *pFeatureParaLen = 8;
				 break;
			 }
			 //[LGE_UPDATE_E][yonghwan.lym@lge.com][2014-12-24] add FOV value.
        default:
            break;
    }
    return ERROR_NONE;
}	


SENSOR_FUNCTION_STRUCT	SensorFuncOV8858S=
{
    OV8858SOpen,
    OV8858SGetInfo,
    OV8858SGetResolution,
    OV8858SFeatureControl,
    OV8858SControl,
    OV8858SClose
};

UINT32 OV8858S_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncOV8858S;

    return ERROR_NONE;
}  


