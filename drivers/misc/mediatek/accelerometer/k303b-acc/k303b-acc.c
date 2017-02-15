/* K303B motion sensor driver
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "k303b-acc.h"
#include <linux/hwmsen_helper.h>

/* Use CALIBRATION_TO_FILE define for saving cal data to file      */
/* Without CALIBRATION_TO_FILE define, Cal data is saved to MISC2 */
#define CALIBRATION_TO_FILE 1
#if defined(CALIBRATION_TO_FILE)
#include <linux/syscalls.h>
#include <linux/fs.h>
#endif

#include "../../tc1_interface/pmt/lg_partition.h"

#if 0  /* LGE_BSP_COMMON LGE_CHANGE_S 140306 jongwoo82.lee : Calibration Value write */
#if defined(TARGET_S4)
#include "../../lg_interface/lg_partition.h"
#else
#include "../../tc1_interface/lg_partition.h" //for MT6582/MT6592
#endif
#endif  /* LGE_BSP_COMMON LGE_CHANGE_E 140306 jongwoo82.lee : Calibration Value write */

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/board_lge.h>
//#include <linux/time_log.h>

/*-------------------------MT6516&MT6573 define-------------------------------*/

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#define SW_CALIBRATION

#ifdef SW_CALIBRATION

struct K303Bacc {
	s32 x;
	s32 y;
	s32 z;
};
#endif //

/*----------------------------------------------------------------------------*/
/*		Self-Test Threshold : 16bit/4g		*/
/*----------------------------------------------------------------------------*/
#define K303B_SHAKING_DETECT_THRESHOLD 1597	/* 0.195g X 8192 LSB/g */

#define TESTLIMIT_XY            (2142)	/* 8192 LSB/g X 0.180g */
#define TESTLIMIT_Z_USL_LSB          (10240)	/* 8192 LSB + 8192 LSB/g X 0.250g */
#define TESTLIMIT_Z_LSL_LSB          (6144)	/* 8192 LSB - 8192 LSB/g X 0.250g */

#define HWST_LSL_LSB            (573)	/* 8192LSB * 0.07g */
#define HWST_USL_LSB            (12288)	/* 8192LSB * 1.5g */

#define CALIBRATION_DATA_AMOUNT 10

/*----------------------------------------------------------------------------*/
//#define DEBUG 1
//#define CONFIG_K303B_LOWPASS	/*apply low pass filter on output */
/*----------------------------------------------------------------------------*/
#define K303B_AXIS_X          0
#define K303B_AXIS_Y          1
#define K303B_AXIS_Z          2
#define K303B_AXES_NUM        3
#define K303B_DATA_LEN        6
#define K303B_DEV_NAME        "k303b-acc"

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id k303b_i2c_id[] =
    { {K303B_DEV_NAME, 0}, {} };
static struct i2c_board_info __initdata i2c_k303b =
    { I2C_BOARD_INFO("k303b-acc", 0x3A >> 1) };

/*----------------------------------------------------------------------------*/
static int k303b_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id);
static int k303b_i2c_remove(struct i2c_client *client);
static int k303b_suspend(struct i2c_client *client, pm_message_t msg);
static int k303b_resume(struct i2c_client *client);
#if defined(CALIBRATION_TO_FILE)
static int k303b_calibration_read(int *cal_read);
static int k303b_calibration_save(int *cal);
static int make_cal_data_file(void);
#endif

/*----------------------------------------------------------------------------*/
typedef enum {
	ADX_TRC_FILTER = 0x01,
	ADX_TRC_RAWDATA = 0x02,
	ADX_TRC_IOCTL = 0x04,
	ADX_TRC_CALI = 0X08,
	ADX_TRC_INFO = 0X10,
} ADX_TRC;

static DEFINE_MUTEX(k303b_mutex);

/*----------------------------------------------------------------------------*/
struct scale_factor {
	u8 whole;
	u8 fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][K303B_AXES_NUM];
	int sum[K303B_AXES_NUM];
	int num;
	int idx;
};

/*----------------------------------------------------------------------------*/
struct k303b_i2c_data {
	struct i2c_client *client;
	struct acc_hw *hw;
	struct hwmsen_convert cvt;
	atomic_t layout;

	/*misc */
	struct data_resolution reso;
	atomic_t trace;
	atomic_t suspend;
	//atomic_t selftest;
    atomic_t selftest_rslt;
	atomic_t filter;
	s16 cali_sw[K303B_AXES_NUM + 1];

	/*data */
	s8 offset[K303B_AXES_NUM + 1];	/*+1: for 4-byte alignment */
	s32 data[K303B_AXES_NUM + 1];

#if defined(CONFIG_K303B_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
	/*early suspend */
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
#endif

#ifdef SW_CALIBRATION
	atomic_t fast_calib_rslt;
#endif
};
static s16 cali_old[K303B_AXES_NUM + 1];
static bool cali_read_check = 0;

/*----------------------------------------------------------------------------*/
static struct i2c_driver k303b_i2c_driver = {
    .driver = {
//        .owner          = THIS_MODULE,
		   .name = K303B_DEV_NAME,
		   },
	.probe = k303b_i2c_probe,
	.remove = k303b_i2c_remove,
//	.detect				= k303b_i2c_detect,
//#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = k303b_suspend,
	.resume = k303b_resume,
//#endif
	.id_table = k303b_i2c_id,
	//.address_list= k303b_forces,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *k303b_i2c_client = NULL;
static struct platform_driver k303b_gsensor_driver;
static struct k303b_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
static GSENSOR_VECTOR3D gsensor_gain;

static int test_status = 0;
static int data_count = 0;
static int hiddenreg_read_count = 0;

/*----------------------------------------------------------------------------*/
#define SENSOR_TAG                  "[LGE_Accelerometer]"
//#define DEBUG 1

#ifdef DEBUG
#define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
#define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt, __FUNCTION__, ##args)
#define SENSOR_DBG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[D]""%s : "fmt, __FUNCTION__, ##args)
#else
#define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
#define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt, __FUNCTION__, ##args)
#define SENSOR_DBG(fmt, args...)    NULL
#endif

/*----------------------------------------------------------------------------*/
static struct data_resolution k303b_data_resolution[] = {
	/*8 combination by {FULL_RES,RANGE} */
	{{3, 9}, 256},	/*+/-2g  in 10-bit resolution:  3.9 mg/LSB */
	{{7, 8}, 128},	/*+/-4g  in 10-bit resolution:  7.8 mg/LSB */
	{{15, 6}, 64},	/*+/-8g  in 10-bit resolution: 15.6 mg/LSB */
	{{31, 2}, 32},	/*+/-16g in 10-bit resolution: 31.2 mg/LSB */
	{{3, 9}, 256},	/*+/-2g  in 10-bit resolution:  3.9 mg/LSB (full-resolution) */
	{{3, 9}, 256},	/*+/-4g  in 11-bit resolution:  3.9 mg/LSB (full-resolution) */
	{{3, 9}, 256},	/*+/-8g  in 12-bit resolution:  3.9 mg/LSB (full-resolution) */
	{{3, 9}, 256},	/*+/-16g in 13-bit resolution:  3.9 mg/LSB (full-resolution) */
};

/*----------------------------------------------------------------------------*/
static struct data_resolution k303b_offset_resolution = { {15, 6}, 64 };

#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER) //for auto detect
static int k303b_local_init(void);
static int k303b_local_remove(void);
static int k303b_init_flag = -1;//0<==>OK, -1<==>fail
static struct sensor_init_info gsensor_init_info = {
	.name = "K303B-ACC",
	.init = k303b_local_init,
	.uninit = k303b_local_remove,
};
#endif

#define BACKUP_HIDDENREG 2
#ifdef BACKUP_HIDDENREG
static u8 ReservedRegData[15] = {0};
static int do_recovery = 0;

#define RECOVERY_MONITOR_COUNT_MAX 20
#define RECOVERY_NORM_THRESHOLD 25 //less than 5m/s^2
static int recovery_monitor_started = 0;
static int recovery_monitor_counter = 0;
#endif


/*--------------------ADXL power control function-----------------------------*/
static void k303b_power(struct acc_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{
		SENSOR_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			SENSOR_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "K303B-acc"))
			{
				SENSOR_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "K303B-acc"))
			{
				SENSOR_ERR("power off fail!!\n");
			}
		}
	}
	power_on = on;
}



/*----------------------------------------- OK -----------------------------------*/
static int k303b_CheckDeviceID(struct i2c_client *client)
{
	SENSOR_FUN();

	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);
	databuf[0] = K303B_REG_DEVID;

	res = i2c_master_send(client, databuf, 0x1);

	if(res <= 0)
	{
		goto exit_K303B_CheckDeviceID;
	}

	udelay(500);

	databuf[0] = 0x0;
	res = i2c_master_recv(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_K303B_CheckDeviceID;
	}


	if(databuf[0]!=K303B_FIXED_DEVID)
	{
        SENSOR_ERR("Check ID error: id = 0x%x != K303B_FIXED_DEVID[ 0x%x ]\n", databuf[0], K303B_FIXED_DEVID);
		return K303B_ERR_IDENTIFICATION;
	}

	exit_K303B_CheckDeviceID:
	if (res <= 0)
	{
		return K303B_ERR_I2C;
	}

	return K303B_SUCCESS;
}

#ifdef BACKUP_HIDDENREG
static int k303b_StoreHiddenReg(struct i2c_client *client)
{
    u8 databuf_reg[2] = {0,0};
    int res = -1;
    int i=0;

    SENSOR_FUN();

    memset(databuf_reg, 0, sizeof(u8)*2);

   for(i=0;i<15;i++)
   	{
      if((res = hwmsen_read_block(client, 0x0 + i, databuf_reg, 1)))
	  	{
	        mdelay(5);
            if((res = hwmsen_read_block(client, 0x0 + i, databuf_reg, 1))){
            	SENSOR_ERR("Read Error Reg = 0X%x, data=%d\n", i, databuf_reg[0]);
            }
            else{
                SENSOR_LOG("Reg = 0X%x, data=%d\n",i, databuf_reg[0]);
                ReservedRegData[i]=databuf_reg[0];
            }
        }
      else
        {
            SENSOR_LOG("Reg = 0X%x, data=%d\n",i, databuf_reg[0]);
            ReservedRegData[i]=databuf_reg[0];
        }
	  databuf_reg[0] = 0;
	  databuf_reg[1] = 0;
    }

    return K303B_SUCCESS;
}

static int k303b_HiddenReg_Recovery(struct i2c_client *client)
{
	u8 databuf_reg[2] = {0,0};
	int res = -1;
	int i=0;

	SENSOR_FUN();
	do_recovery = 1;
	memset(databuf_reg, 0, sizeof(u8)*2);

   for(i=0;i<15;i++)
	{
	  if((res = hwmsen_read_block(client, 0x0 + i, databuf_reg, 1)))
		{
			mdelay(5);
			if((res = hwmsen_read_block(client, 0x0 + i, databuf_reg, 1))){
				SENSOR_ERR("Read Error Reg = 0X%x, data=%d\n", i, databuf_reg[0]);
			}
			else{
				
				databuf_reg[0] = ReservedRegData[i];
				if(res = hwmsen_write_block(client, 0x0 + i, databuf_reg, 1))
				{
					SENSOR_ERR("error: %d\n", res);
				}
			}
		}
	  else
		{
		  databuf_reg[0] = ReservedRegData[i];
		  if(res = hwmsen_write_block(client, 0x0 + i, databuf_reg, 1))
		  {
			  SENSOR_ERR("error: %d\n", res);
		  }
		}
	  databuf_reg[0] = 0;
	  databuf_reg[1] = 0;
	}
	
	return K303B_SUCCESS;
}

#endif


/*-------------------------------set CTRL_REG1_A(20h)------------------------*/
static int k303b_SetBWRate(struct i2c_client *client, u8 bwrate)
{

	SENSOR_DBG("\n");
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	databuf[0] = K303B_REG_BW_RATE;

	mutex_lock(&k303b_mutex);

	bwrate = ((ODR_ACC_MASK & bwrate) | K303B_ACC_ODR_ENABLE);

	databuf[0] = K303B_REG_BW_RATE;
	databuf[1] = bwrate | 0x80;//for HIGH_RESOLUTION

	res = i2c_master_send(client, databuf, 0x2);

	//SENSOR_LOG("send rate");

	if(res <=0)
	{
		goto EXIT_ERR;

	}
	mutex_unlock(&k303b_mutex);

	return K303B_SUCCESS;

EXIT_ERR:
	mutex_unlock(&k303b_mutex);
	return K303B_ERR_I2C;

}

/*-------------------------------set accel sensitivity ----------------------*/
static int k303b_SetDataResolution(struct k303b_i2c_data *obj,
				     u8 new_fs_range)
{
	SENSOR_DBG("\n");
	u8 dat, reso;

	switch (new_fs_range) {
	case K303B_ACC_FS_2G:
		obj->reso.sensitivity = SENSITIVITY_ACC_2G;
		break;

	case K303B_ACC_FS_4G:
		obj->reso.sensitivity = SENSITIVITY_ACC_4G;
		break;

	case K303B_ACC_FS_8G:
		obj->reso.sensitivity = SENSITIVITY_ACC_8G;
		break;

	default:
		obj->reso.sensitivity = SENSITIVITY_ACC_2G;
		SENSOR_LOG("invalid magnetometer fs range requested: %u\n", new_fs_range);
		return -EINVAL;
	}

	return 0;

}

/*-------------------------------set CTRL_REG4_A(23h)------------------------*/
static int k303b_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	SENSOR_DBG("\n");
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	databuf[0] = K303B_REG_DATA_FORMAT;

	res = i2c_master_send(client, databuf, 0x1);

	if(res <= 0)
	{
		SENSOR_ERR("k303b_reg_data_format Error! send Error!!\n");
		return K303B_ERR_I2C;
	}

	udelay(500);

	databuf[0] = 0x0;
	res = i2c_master_recv(client, databuf, 0x01);

	if(res <= 0)
	{
		SENSOR_ERR("k303b_reg_data_format Error! recv Error!!\n");
		return K303B_ERR_I2C;
	}

	dataformat = ((K303B_ACC_FS_MASK & dataformat) |
		      ((~K303B_ACC_FS_MASK) & databuf[0]));

	databuf[0] = K303B_REG_DATA_FORMAT;
	databuf[1] = dataformat | AAF_BW_100Hz;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		SENSOR_ERR("k303b_reg_data_format Error! send 2 Error!!\n");
		return K303B_ERR_I2C;
	}

	return k303b_SetDataResolution(obj, dataformat & K303B_ACC_FS_MASK);

}

/*-----------------------set device enable/disable---------------------------*/
static int k303b_SetPowerMode(struct i2c_client *client, bool enable)
{
	SENSOR_FUN();
	int res = 0;

	if(enable == sensor_power)
	{
		SENSOR_LOG("Sensor power status is newest!\n");
		return K303B_SUCCESS;
	}
	if(enable == TRUE)
	{
		res = k303b_SetBWRate(client, K303B_ACC_ODR100);
	}
	else
	{
		res = k303b_SetBWRate(client, K303B_ACC_ODR_OFF);
	}

	if(res < 0)
	{
		SENSOR_ERR("set power mode failed!\n");
		return K303B_ERR_I2C;
	}
    else
    {
        SENSOR_LOG("set power mode ok, value: %d\n",enable);
    }

	sensor_power = enable;
	test_status = sensor_power;
	return K303B_SUCCESS;

}

/*-------------------------------set CTRL_REG2_A(21h)------------------------*/
static int k303b_SetFilterLen(struct i2c_client *client, u8 new_bandwidth)
{
	SENSOR_DBG("\n");
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 10);

	databuf[0] = K303B_REG_DATA_FILTER;

	res = i2c_master_send(client, databuf, 0x1);

	if(res <= 0)
	{
		return K303B_ERR_I2C;
	}

	udelay(500);

	databuf[0] = 0x0;
	res = i2c_master_recv(client, databuf, 0x01);

	if(res <= 0)
	{
		return K303B_ERR_I2C;
	}

	new_bandwidth = ((K303B_ACC_FILTER_MASK & new_bandwidth) |
					((~K303B_ACC_FILTER_MASK) & databuf[0]));

	databuf[0] = K303B_REG_DATA_FILTER;
	//databuf[1] = new_bandwidth | 0x00;//[ODR/50]Hz
	//databuf[1] = new_bandwidth | 0x20;//[ODR/100]Hz
	databuf[1] = new_bandwidth | 0x40;//[ODR/9]Hz
	//databuf[1] = new_bandwidth | 0x60;//[ODR/400]Hz

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return K303B_ERR_I2C;
	}

	return 0;

}

/*-----------------------------get device info-------------------------------*/
static int k303b_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	SENSOR_FUN();
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}

	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "K303B Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int k303b_ReadOffset(struct i2c_client *client, s8 ofs[K303B_AXES_NUM])
{
	int err = 0;;

#ifdef SW_CALIBRATION
	ofs[0] = ofs[1] = ofs[2] = 0x0;
#else

	if((err = hwmsen_read_block(client, MC3210_REG_OFSX, ofs, K303B_AXES_NUM)))
	{
		SENSOR_ERR("error: %d\n", err);
	}
#endif

	return err;
}

/*----------------------------------------------------------------------------*/
static int k303b_ReadCalibrationEx(struct i2c_client *client, int act[K303B_AXES_NUM], int raw[K303B_AXES_NUM])
{
	/*raw: the raw calibration data; act: the actual calibration data */
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int mul;

#ifdef SW_CALIBRATION
	mul = 0;		//only SW Calibration, disable HW Calibration
#else

	if((err = k303b_ReadOffset(client, obj->offset)))
	{
		SENSOR_ERR("read offset fail, %d\n", err);
		return err;
	}

	mul = obj->reso.sensitivity / mc3210_offset_resolution.sensitivity;
#endif

	raw[K303B_AXIS_X] =
	    obj->offset[K303B_AXIS_X] * mul + obj->cali_sw[K303B_AXIS_X];
	raw[K303B_AXIS_Y] =
	    obj->offset[K303B_AXIS_Y] * mul + obj->cali_sw[K303B_AXIS_Y];
	raw[K303B_AXIS_Z] =
	    obj->offset[K303B_AXIS_Z] * mul + obj->cali_sw[K303B_AXIS_Z];

	act[obj->cvt.map[K303B_AXIS_X]] =
	    obj->cvt.sign[K303B_AXIS_X] * raw[K303B_AXIS_X];
	act[obj->cvt.map[K303B_AXIS_Y]] =
	    obj->cvt.sign[K303B_AXIS_Y] * raw[K303B_AXIS_Y];
	act[obj->cvt.map[K303B_AXIS_Z]] =
	    obj->cvt.sign[K303B_AXIS_Z] * raw[K303B_AXIS_Z];

	return 0;
}

/*----------------------------------------------------------------------------*/
static int k303b_ResetCalibration(struct i2c_client *client)
{
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	s8 ofs[K303B_AXES_NUM] = { 0x00, 0x00, 0x00 };
	int err = 0;

#ifdef SW_CALIBRATION

#else

		if(err = hwmsen_write_block(client, K303B_REG_OFSX, ofs, 4))
		{
			SENSOR_ERR("error: %d\n", err);
		}
#endif

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;
}

/*----------------------------------------------------------------------------*/
static int k303b_ReadCalibration(struct i2c_client *client, int dat[K303B_AXES_NUM])
{
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int mul;

#ifdef SW_CALIBRATION
	struct k303b_i2c_data *priv = i2c_get_clientdata(client);
	u8 addr = K303B_REG_DATAX0;
	u8 buf[K303B_DATA_LEN] = { 0 };
	int i;
	s16 data[K303B_AXES_NUM] = { 0x00, 0x00, 0x00 };

	  if(NULL == client)
	  {
	  	err = -EINVAL;
	  }
	  else if((err = hwmsen_read_block(client, addr, buf, 0x06)))
	  {
		SENSOR_ERR("error: %d\n", err);
          }
	  else
	  {
		data[K303B_AXIS_X] = (s16)((buf[K303B_AXIS_X*2]) |
		         (buf[K303B_AXIS_X*2+1] << 8));
		data[K303B_AXIS_Y] = (s16)((buf[K303B_AXIS_Y*2]) |
		         (buf[K303B_AXIS_Y*2+1] << 8));
		data[K303B_AXIS_Z] = (s16)((buf[K303B_AXIS_Z*2]) |
		         (buf[K303B_AXIS_Z*2+1] << 8));
	  }
       #endif

#ifdef SW_CALIBRATION
	mul = 0;		//only SW Calibration, disable HW Calibration
#else

	if ((err = k303b_ReadOffset(client, obj->offset))) {
		SENSOR_ERR("read offset fail, %d\n", err);
		return err;
	}

	mul = obj->reso.sensitivity / k303b_offset_resolution.sensitivity;
#endif

	dat[obj->cvt.map[K303B_AXIS_X]] =
	    obj->cvt.sign[K303B_AXIS_X] * (obj->offset[K303B_AXIS_X] * mul +
					     obj->cali_sw[K303B_AXIS_X]);
	dat[obj->cvt.map[K303B_AXIS_Y]] =
	    obj->cvt.sign[K303B_AXIS_Y] * (obj->offset[K303B_AXIS_Y] * mul +
					     obj->cali_sw[K303B_AXIS_Y]);
	dat[obj->cvt.map[K303B_AXIS_Z]] =
	    obj->cvt.sign[K303B_AXIS_Z] * (obj->offset[K303B_AXIS_Z] * mul +
					     obj->cali_sw[K303B_AXIS_Z]);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int k303b_WriteCalibration(struct i2c_client *client, int dat[K303B_AXES_NUM])
{
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[K303B_AXES_NUM], raw[K303B_AXES_NUM];
	int lsb = k303b_offset_resolution.sensitivity;
	int divisor = obj->reso.sensitivity / lsb;

#define K303B_AXIS_X          0
#define K303B_AXIS_Y          1
#define K303B_AXIS_Z          2

	/*offset will be updated in obj->offset */
	if((err = k303b_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{
		SENSOR_ERR("read offset fail, %d\n", err);
		return err;
	}

	SENSOR_LOG
	    ("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
	     raw[K303B_AXIS_X], raw[K303B_AXIS_Y], raw[K303B_AXIS_Z],
	     obj->offset[K303B_AXIS_X], obj->offset[K303B_AXIS_Y],
	     obj->offset[K303B_AXIS_Z], obj->cali_sw[K303B_AXIS_X],
	     obj->cali_sw[K303B_AXIS_Y], obj->cali_sw[K303B_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[K303B_AXIS_X] += dat[K303B_AXIS_X];
	cali[K303B_AXIS_Y] += dat[K303B_AXIS_Y];
	cali[K303B_AXIS_Z] += dat[K303B_AXIS_Z];

	SENSOR_LOG("UPDATE: (%+3d %+3d %+3d)\n",
		dat[K303B_AXIS_X], dat[K303B_AXIS_Y], dat[K303B_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[K303B_AXIS_X] = 
	    (s16)(obj->cvt.sign[K303B_AXIS_X] * (cali[obj->cvt.map[K303B_AXIS_X]]));
	obj->cali_sw[K303B_AXIS_Y] =
	    (s16)(obj->cvt.sign[K303B_AXIS_Y] * (cali[obj->cvt.map[K303B_AXIS_Y]]));
	obj->cali_sw[K303B_AXIS_Z] =
	    (s16)(obj->cvt.sign[K303B_AXIS_Z] * (cali[obj->cvt.map[K303B_AXIS_Z]]));

	SENSOR_LOG
	    ("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
	     obj->offset[K303B_AXIS_X] * divisor + obj->cali_sw[K303B_AXIS_X],
	     obj->offset[K303B_AXIS_Y] * divisor + obj->cali_sw[K303B_AXIS_Y],
	     obj->offset[K303B_AXIS_Z] * divisor + obj->cali_sw[K303B_AXIS_Z],
	     obj->offset[K303B_AXIS_X], obj->offset[K303B_AXIS_Y],
	     obj->offset[K303B_AXIS_Z], obj->cali_sw[K303B_AXIS_X],
	     obj->cali_sw[K303B_AXIS_Y], obj->cali_sw[K303B_AXIS_Z]);
#else
	obj->offset[K303B_AXIS_X] =
	    (s8) (obj->cvt.sign[K303B_AXIS_X] * (cali[obj->cvt.map[K303B_AXIS_X]]) / (divisor));
	obj->offset[K303B_AXIS_Y] =
	    (s8) (obj->cvt.sign[K303B_AXIS_Y] * (cali[obj->cvt.map[K303B_AXIS_Y]]) / (divisor));
	obj->offset[K303B_AXIS_Z] =
	    (s8) (obj->cvt.sign[K303B_AXIS_Z] * (cali[obj->cvt.map[K303B_AXIS_Z]]) / (divisor));

	/*convert software calibration using standard calibration */
	obj->cali_sw[K303B_AXIS_X] =
	    (s16)(obj->cvt.sign[K303B_AXIS_X] * (cali[obj->cvt.map[K303B_AXIS_X]]) % (divisor));
	obj->cali_sw[K303B_AXIS_Y] =
	    (s16)(obj->cvt.sign[K303B_AXIS_Y] * (cali[obj->cvt.map[K303B_AXIS_Y]]) % (divisor));
	obj->cali_sw[K303B_AXIS_Z] =
	    (s16)(obj->cvt.sign[K303B_AXIS_Z] * (cali[obj->cvt.map[K303B_AXIS_Z]]) % (divisor));

	SENSOR_LOG
	    ("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n",
	     obj->offset[K303B_AXIS_X] * divisor + obj->cali_sw[K303B_AXIS_X],
	     obj->offset[K303B_AXIS_Y] * divisor + obj->cali_sw[K303B_AXIS_Y],
	     obj->offset[K303B_AXIS_Z] * divisor + obj->cali_sw[K303B_AXIS_Z],
	     obj->offset[K303B_AXIS_X], obj->offset[K303B_AXIS_Y],
	     obj->offset[K303B_AXIS_Z], obj->cali_sw[K303B_AXIS_X],
	     obj->cali_sw[K303B_AXIS_Y], obj->cali_sw[K303B_AXIS_Z]);

	if((err = hwmsen_write_block(obj->client, MC3210_REG_OFSX, obj->offset, MC3210_AXES_NUM)))
	{
		SENSOR_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}

static int k303b_init_client(struct i2c_client *client, int reset_cali)
{
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

	SENSOR_FUN();

	// 1 check ID  ok
	res = k303b_CheckDeviceID(client);

	if(res != K303B_SUCCESS)
	{
	    SENSOR_ERR("Check ID error\n");
		return res;
	}
	// 2 POWER MODE  YES
	res = k303b_SetPowerMode(client, false);
	if(res != K303B_SUCCESS)
	{
	    SENSOR_ERR("set power error\n");
		return res;
	}
	// 3 RATE  ok
	res = k303b_SetBWRate(client, K303B_ACC_ODR_OFF);
	if(res != K303B_SUCCESS ) //0x2C->BW=100Hz
	{
	    SENSOR_ERR("set power error\n");
		return res;
	}
	// 4 RANGE  ok
	res = k303b_SetDataFormat(client, K303B_ACC_FS_4G);
	if(res != K303B_SUCCESS) //0x2C->BW=100Hz
	{
	    SENSOR_ERR("set data format error\n");
		return res;
	}
	// 5 GAIN  ?
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso.sensitivity;

	// 6 CutOff
	k303b_SetFilterLen(client, 0x00);

	// 6 eint diable
    /***
    res = K303B_SetIntEnable(client, 0x00);   //disable INT
    if(res != K303B_SUCCESS)
    {
        SENSOR_ERR("K303B_SetIntEnable error\n");
          return res;
    }
    ***/
	// 7 cali
	if(0 != reset_cali)
	{
		/*reset calibration only in power on */
		res = k303b_ResetCalibration(client);

		if(res != K303B_SUCCESS)
		{
			return res;
		}
	}
#ifdef CONFIG_K303B_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	//for Hidden Register issue
#ifdef BACKUP_HIDDENREG
	k303b_StoreHiddenReg(client);
#endif	

	return K303B_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int k303b_ReadData(struct i2c_client *client, s32 data[K303B_AXES_NUM])
{
	struct k303b_i2c_data *priv = i2c_get_clientdata(client);
	u8 addr = K303B_REG_DATAX0 | I2C_AUTO_INCREMENT;
	u8 buf[K303B_DATA_LEN] = { 0 };
	u8 reserved[2]= {0,0};
	int i = 0;
	int err = 0;
	int hiddenreg_error = 0;

	unsigned int norm = 0;
	int x = 0, y = 0, z = 0;
	int acc[K303B_DATA_LEN];
	//GSE_FUN();

//TIME_LOG_START();
	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if((err = hwmsen_read_block(client, addr, buf, 0x06)))
	{
		SENSOR_ERR("error: %d\n", err);
	}
	else
	{
	
	//for Hidden Register issue
#if (BACKUP_HIDDENREG == 1)
		hiddenreg_read_count++;
		if(hiddenreg_read_count > 100)
		{
			hiddenreg_read_count = 0;	
			SENSOR_LOG("Check HiddenRegister Start\n");
			for(i=1;i<5;i++)
			{
				if((err = hwmsen_read_block(client, 0x0 + i, reserved, 1)))
				{
					mdelay(5);
					if((err = hwmsen_read_block(client, 0x0 + i, reserved, 1)))
					{
						SENSOR_ERR("Read Error for HiddenReg : 0x%x, data=%d\n",i,reserved[0]);
					}
					else
					{
						if(reserved[0] != ReservedRegData[i])
						{
							hiddenreg_error = -1;
							break;
						}
					}
				SENSOR_LOG(" %02x ", reserved[0]);
				}
				else
				{
					if(reserved[0] != ReservedRegData[i])
					{
						hiddenreg_error = -1;
						break;
					}
				}
				reserved[0] = 0;
				reserved[1] = 0;
			}

			if(hiddenreg_error != 0)
			{
				hiddenreg_error = 0;
				k303b_HiddenReg_Recovery(client);
			}
		}
#endif

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
				SENSOR_LOG
			    ("transfer before [%08X %08X %08X] => [%08X %08X %08X]\n",
			     buf[K303B_AXIS_X * 2],
			     buf[K303B_AXIS_X * 2 + 1],
			     buf[K303B_AXIS_Y * 2],
			     buf[K303B_AXIS_Y * 2 + 1],
			     buf[K303B_AXIS_Z * 2],
			     buf[K303B_AXIS_Z * 2 + 1]);
		}

		data[K303B_AXIS_X] =
		    ((s32)((s16)((buf[K303B_AXIS_X * 2 + 1] << 8) | (buf[K303B_AXIS_X * 2]))));
		data[K303B_AXIS_Y] =
		    ((s32)((s16)((buf[K303B_AXIS_Y * 2 + 1] << 8) | (buf[K303B_AXIS_Y * 2]))));
		data[K303B_AXIS_Z] =
		    ((s32)((s16)((buf[K303B_AXIS_Z * 2 + 1] << 8) | (buf[K303B_AXIS_Z * 2]))));

		if (atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
				SENSOR_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n",
				data[K303B_AXIS_X], data[K303B_AXIS_Y],
				data[K303B_AXIS_Z], data[K303B_AXIS_X],
				data[K303B_AXIS_Y], data[K303B_AXIS_Z]);
		}
#ifdef CONFIG_K303B_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);
				if(priv->fir.num < firlen)
				{
					priv->fir.raw[priv->fir.num][K303B_AXIS_X] = data[K303B_AXIS_X];
					priv->fir.raw[priv->fir.num][K303B_AXIS_Y] = data[K303B_AXIS_Y];
					priv->fir.raw[priv->fir.num][K303B_AXIS_Z] = data[K303B_AXIS_Z];
					priv->fir.sum[K303B_AXIS_X] += data[K303B_AXIS_X];
					priv->fir.sum[K303B_AXIS_Y] += data[K303B_AXIS_Y];
					priv->fir.sum[K303B_AXIS_Z] += data[K303B_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
							SENSOR_LOG ("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n",
						     priv->fir.num, priv->fir.raw[priv->fir.num][K303B_AXIS_X],
						     priv->fir.raw[priv->fir.num][K303B_AXIS_Y],
						     priv->fir.raw[priv->fir.num][K303B_AXIS_Z],
						     priv->fir.sum[K303B_AXIS_X],
						     priv->fir.sum[K303B_AXIS_Y],
						     priv->fir.sum[K303B_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[K303B_AXIS_X] -=  priv->fir.raw[idx][K303B_AXIS_X];
					priv->fir.sum[K303B_AXIS_Y] -=  priv->fir.raw[idx][K303B_AXIS_Y];
					priv->fir.sum[K303B_AXIS_Z] -=  priv->fir.raw[idx][K303B_AXIS_Z];
					priv->fir.raw[idx][K303B_AXIS_X] = data[K303B_AXIS_X];
					priv->fir.raw[idx][K303B_AXIS_Y] = data[K303B_AXIS_Y];
					priv->fir.raw[idx][K303B_AXIS_Z] = data[K303B_AXIS_Z];
					priv->fir.sum[K303B_AXIS_X] += data[K303B_AXIS_X];
					priv->fir.sum[K303B_AXIS_Y] += data[K303B_AXIS_Y];
					priv->fir.sum[K303B_AXIS_Z] += data[K303B_AXIS_Z];
					priv->fir.idx++;
					data[K303B_AXIS_X] = priv->fir.sum[K303B_AXIS_X] / firlen;
					data[K303B_AXIS_Y] = priv->fir.sum[K303B_AXIS_Y] / firlen;
					data[K303B_AXIS_Z] = priv->fir.sum[K303B_AXIS_Z] / firlen;

					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
							SENSOR_LOG ("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n",
						     idx, priv->fir.raw[idx][K303B_AXIS_X],
						     priv->fir.raw[idx][K303B_AXIS_Y],
						     priv->fir.raw[idx][K303B_AXIS_Z],
						     priv->fir.sum[K303B_AXIS_X],
						     priv->fir.sum[K303B_AXIS_Y],
						     priv->fir.sum[K303B_AXIS_Z],
						     data[K303B_AXIS_X],
						     data[K303B_AXIS_Y],
						     data[K303B_AXIS_Z]);
					}
				}
			}
		}
#endif
	}
	
#if (BACKUP_HIDDENREG == 2)
		{
			/*remap coordinate*/
			acc[priv->cvt.map[K303B_AXIS_X]] = priv->cvt.sign[K303B_AXIS_X]*(data[K303B_AXIS_X]+priv->cali_sw[K303B_AXIS_X]);
			acc[priv->cvt.map[K303B_AXIS_Y]] = priv->cvt.sign[K303B_AXIS_Y]*(data[K303B_AXIS_Y]+priv->cali_sw[K303B_AXIS_Y]);
			acc[priv->cvt.map[K303B_AXIS_Z]] = priv->cvt.sign[K303B_AXIS_Z]*(data[K303B_AXIS_Z]+priv->cali_sw[K303B_AXIS_Z]);
			//Out put the mg
			acc[K303B_AXIS_X] = acc[K303B_AXIS_X] * GRAVITY_EARTH_1000 / priv->reso.sensitivity;
			acc[K303B_AXIS_Y] = acc[K303B_AXIS_Y] * GRAVITY_EARTH_1000 / priv->reso.sensitivity;
			acc[K303B_AXIS_Z] = acc[K303B_AXIS_Z] * GRAVITY_EARTH_1000 / priv->reso.sensitivity;
			x = acc[K303B_AXIS_X]/1000;
			y = acc[K303B_AXIS_Y]/1000;
			z = acc[K303B_AXIS_Z]/1000;
			
			norm = x*x + y*y + z*z;
			//SENSOR_LOG("**=========  %d %d %d  norm value : %d\n",x,y,z,norm);
			if(!recovery_monitor_started) {
				if(norm < RECOVERY_NORM_THRESHOLD) {
					SENSOR_DBG("[1] recovery monitor start\n");
					recovery_monitor_started = 1;
					recovery_monitor_counter = 0;
				}
			} else {
				if(norm >= RECOVERY_NORM_THRESHOLD) {
					SENSOR_DBG("[2-1] recovery monitor reset\n");
					recovery_monitor_started = 0;
					recovery_monitor_counter = 0;
				} else {
					//SENSOR_DBG("[2-2] counter++\n");
					recovery_monitor_counter++;
					if(recovery_monitor_counter > RECOVERY_MONITOR_COUNT_MAX) {
						SENSOR_LOG("[3] Call hiddenregister recovery function\n");
						k303b_HiddenReg_Recovery(client);
						recovery_monitor_started = 0;
						recovery_monitor_counter = 0;
					}
				}
			}
		}
#endif	
//TIME_LOG_END("k303b_ReadData: ");
	return err;
}

/*----------------------------------------------------------------------------*/
static int k303b_ReadRawData(struct i2c_client *client, char *buf)
{
	struct k303b_i2c_data *obj = (struct k303b_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}

	if((res = k303b_ReadData(client, obj->data)))
	{
		SENSOR_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "%04x %04x %04x", obj->data[K303B_AXIS_X],
			obj->data[K303B_AXIS_Y], obj->data[K303B_AXIS_Z]);

	}

	return 0;
}


/*----------------------------------------------------------------------------*/
static int k303b_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	int acc[K303B_DATA_LEN];
	int res = 0;
	struct k303b_i2c_data *obj = obj_i2c_data;
	client = obj->client;

	if(NULL == buf)
	{
		return -1;
	}

	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == FALSE)
	{
		res = k303b_SetPowerMode(client, true);
		if(res)
		{
			SENSOR_ERR("Power on k303b error %d!\n", res);
		}
		msleep(20);
	}

	if((res = k303b_ReadData(client, obj->data)))
	{
		SENSOR_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		obj->data[K303B_AXIS_X] += obj->cali_sw[K303B_AXIS_X];
		obj->data[K303B_AXIS_Y] += obj->cali_sw[K303B_AXIS_Y];
		obj->data[K303B_AXIS_Z] += obj->cali_sw[K303B_AXIS_Z];

		/*remap coordinate*/
		acc[obj->cvt.map[K303B_AXIS_X]] = obj->cvt.sign[K303B_AXIS_X]*obj->data[K303B_AXIS_X];
		acc[obj->cvt.map[K303B_AXIS_Y]] = obj->cvt.sign[K303B_AXIS_Y]*obj->data[K303B_AXIS_Y];
		acc[obj->cvt.map[K303B_AXIS_Z]] = obj->cvt.sign[K303B_AXIS_Z]*obj->data[K303B_AXIS_Z];

		//SENSOR_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[K303B_AXIS_X], acc[K303B_AXIS_Y], acc[K303B_AXIS_Z]);

		//Out put the mg
		acc[K303B_AXIS_X] = acc[K303B_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso.sensitivity;
		acc[K303B_AXIS_Y] = acc[K303B_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso.sensitivity;
		acc[K303B_AXIS_Z] = acc[K303B_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso.sensitivity;
		//SENSOR_LOG("map (%d %d %d) / sign (%d %d %d)\n",obj->cvt.map[K303B_AXIS_X],obj->cvt.map[K303B_AXIS_Y],obj->cvt.map[K303B_AXIS_Z],obj->cvt.sign[K303B_AXIS_X],obj->cvt.sign[K303B_AXIS_Y],obj->cvt.sign[K303B_AXIS_Z]);
        //SENSOR_LOG("K303B_ReadSensorData %d %d %d", acc[K303B_AXIS_X], acc[K303B_AXIS_Y], acc[K303B_AXIS_Z]);
		data_count++;
		if(data_count > 100000)
		{
			data_count = 0;
		}

		sprintf(buf, "%04x %04x %04x %04x", acc[K303B_AXIS_X], acc[K303B_AXIS_Y], acc[K303B_AXIS_Z], data_count);

        /* Add for accel sensor data error debugging : start */
        if((data_count%1000)==0)
            SENSOR_LOG("gsensor data: debug count = %d, x=%d, y=%d, z=%d!\n",data_count,acc[K303B_AXIS_X], acc[K303B_AXIS_Y], acc[K303B_AXIS_Z]);
        /* Add for accel sensor data error debugging : end */

		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			SENSOR_LOG("gsensor data: %s!\n", buf);
		}
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = k303b_i2c_client;
	char strbuf[K303B_BUFSIZE];
	if(NULL == client)
	{
		SENSOR_ERR("i2c client is null!!\n");
		return 0;
	}

	k303b_ReadChipInfo(client, strbuf, K303B_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = k303b_i2c_client;
	char strbuf[K303B_BUFSIZE];

	if(NULL == client)
	{
		SENSOR_ERR("i2c client is null!!\n");
		return 0;
	}
	k303b_ReadSensorData(client, strbuf, K303B_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct k303b_i2c_data *data = i2c_get_clientdata(k303b_i2c_client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = k303b_i2c_client;
	struct k303b_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			SENSOR_ERR( "HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			SENSOR_ERR( "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			SENSOR_ERR( "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		SENSOR_ERR( "invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
#if defined(CALIBRATION_TO_FILE)
static int k303b_calibration_read(int *cal_read)
{
   int fd;
   int i;
   int res;
   char *fname = "/persist-lg/sensor/sensor_cal_data.txt";
   mm_segment_t old_fs = get_fs();
   char temp_str[5];

   set_fs(KERNEL_DS);

   fd = sys_open(fname, O_RDONLY, 0);
	if(fd< 0){
         SENSOR_ERR("File Open Error \n");
	     sys_close(fd);
     	 return -EINVAL;
		}
   for(i=0;i<6;i++)
   {
   		memset(temp_str,0x00,sizeof(temp_str));
        res = sys_read(fd, temp_str, sizeof(temp_str));
		if(res<0){
   			 SENSOR_ERR("Read Error \n");
              sys_close(fd);
              return -EINVAL;
         	}
        sscanf(temp_str,"%d",&cal_read[i]);
        SENSOR_LOG("k303b_calibration_read : cal_read[%d]=%d\n",i,cal_read[i]);
   	}
   sys_close(fd);
   set_fs(old_fs);
   SENSOR_LOG("k303b_calibration_read Done.\n");
   cali_old[0] = cal_read[3];
   cali_old[1] = cal_read[4];
   cali_old[2] = cal_read[5];
   cali_read_check = 1;
   SENSOR_DBG("data : %d %d %d / %d %d %d\n",cal_read[0],cal_read[1],cal_read[2],cali_old[0],cali_old[1],cali_old[2]);

   return K303B_SUCCESS;
}

static int k303b_calibration_save(int *cal)
{
   int fd;
   int i;
   int res;
   char *fname = "/persist-lg/sensor/sensor_cal_data.txt";
   mm_segment_t old_fs = get_fs();
   char temp_str[5];

   set_fs(KERNEL_DS);

   fd = sys_open(fname,O_WRONLY|O_CREAT|S_IROTH, 0666);
	if(fd< 0){
           SENSOR_ERR("File Open Error (%d)\n",fd);
   		   sys_close(fd);
	       return -EINVAL;
		}
   for(i=0;i<6;i++)
   {
   		memset(temp_str,0x00,sizeof(temp_str));
   		sprintf(temp_str, "%d", cal[i]);
	   	res = sys_write(fd,temp_str, sizeof(temp_str));

   		if(res<0) {
   			SENSOR_ERR("Write Error \n");
   			sys_close(fd);
	 		return -EINVAL;
   		}
   	}
   sys_fsync(fd);
   sys_close(fd);

   sys_chmod(fname, 0664);
   set_fs(old_fs);

#if 0
   SENSOR_LOG("%s: Entering into calibration read for making sure data is written properly \n",__func__);
   k2dh_calibration_read(cal);
#endif
   SENSOR_LOG("k303b_calibration_save Done.\n");
   cali_old[0] = cal[3];
   cali_old[1] = cal[4];
   cali_old[2] = cal[5];
   cali_read_check = 1;
   SENSOR_DBG("data : %d %d %d / %d %d %d\n",cal[0],cal[1],cal[2],cal[3],cal[4],cal[5]);

   return K303B_SUCCESS;
}

/* Make gsensor cal file to save calibration data */
/* 1. If file exist, do nothing                  */
/* 2. If file does not exist, read cal data from misc2 (for L-OS upgrade model) */
static int make_cal_data_file(void)
{
   int fd;
   int i;
   int res;
   char *fname = "/persist-lg/sensor/sensor_cal_data.txt";
   mm_segment_t old_fs = get_fs();
   char temp_str[5];
   int cal_misc[6]={0,};

   set_fs(KERNEL_DS);

   fd = sys_open(fname, O_RDONLY, 0); /* Cal file exist check */

   if(fd==-2)
   SENSOR_LOG("Open Cal File Error. Need to make file (%d)\n",fd);

 	if(fd< 0){
	       fd = sys_open(fname,O_WRONLY|O_CREAT|S_IROTH, 0666);
	     if(fd< 0){
	       SENSOR_ERR("Open or Make Cal File Error (%d)\n",fd);
  		   sys_close(fd);
	       return -EINVAL;
		   }

	     if(LGE_FacReadAccelerometerCalibration((unsigned int*)cal_misc) == TRUE)
	     	{
		     SENSOR_LOG("Read Cal from misc Old x: %d, y: %d, z: %d\n",cal_misc[3],cal_misc[4],cal_misc[5]);
		     SENSOR_LOG("Read Cal from misc Now x: %d, y: %d, z: %d\n",cal_misc[0],cal_misc[1],cal_misc[2]);
	     	}
         else{
		     SENSOR_ERR("Read Cal from misc Error\n");
         	}

		   for(i=0;i<6;i++)
		   {
		   		memset(temp_str,0x00,sizeof(temp_str));
		   		sprintf(temp_str, "%d", cal_misc[i]);
			   	res = sys_write(fd,temp_str, sizeof(temp_str));

		   		if(res<0) {
		   			SENSOR_ERR("Write Cal Error \n");
		   			sys_close(fd);
			 		return -EINVAL;
		   		}
		   	}
         SENSOR_LOG("make_cal_data_file Done.\n");
		}
	else
         SENSOR_LOG("Sensor Cal File exist.\n");

   sys_fsync(fd);
   sys_close(fd);

   sys_chmod(fname, 0664);
   set_fs(old_fs);
   return K303B_SUCCESS;
}
#endif

#if 1 //motion sensor cal backup
static ssize_t show_cali_backup_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = k303b_i2c_client;
    struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	int offset_x,offset_y,offset_z;
	int cal_backup[6]={0,};
	int i;

#if defined(CALIBRATION_TO_FILE)
   if(cali_read_check == 1)
		SENSOR_LOG("previous offset_x: %d,  offset_y: %d,  offset_z: %d\n",cali_old[0],cali_old[1],cali_old[2]);
   else
        SENSOR_LOG("Fail to read previous gsensor cal value from Cal File\n");
#else
	if(LGE_FacReadAccelerometerCalibration((unsigned int*)cal_backup) == TRUE)
		SENSOR_LOG("Read from LGPServer gsensor previous offset_x: %d,  offset_y: %d,  offset_z: %d\n",cal_backup[3],cal_backup[4],cal_backup[5]);
	else
        SENSOR_LOG("Fail to read previous gsensor cal value from LGPserver\n");
#endif

	offset_x = obj->cali_sw[K303B_AXIS_X];
    offset_y = obj->cali_sw[K303B_AXIS_Y];
    offset_z = obj->cali_sw[K303B_AXIS_Z];

    SENSOR_LOG("Current offset_x: %d, offset_y: %d, offset_z: %d\n",offset_x,offset_y,offset_z);

	return snprintf(buf, PAGE_SIZE, "Old %d %d %d   Now %d %d %d \n", (unsigned int)cali_old[0], (unsigned int)cali_old[1], (unsigned int)cali_old[2], offset_x, offset_y, offset_z);
}
#endif

static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = k303b_i2c_client;
	struct k303b_i2c_data *obj;
	int err = 0;
	int len = 0, mul;
	int tmp[K303B_AXES_NUM];

	if(NULL == client)
	{
		SENSOR_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
    tmp[K303B_AXIS_X] = obj->cali_sw[K303B_AXIS_X];
    tmp[K303B_AXIS_Y] = obj->cali_sw[K303B_AXIS_Y];
    tmp[K303B_AXIS_Z] = obj->cali_sw[K303B_AXIS_Z];

    SENSOR_LOG("offset_x: %d, offset_y: %d, offset_z: %d\n",tmp[K303B_AXIS_X],tmp[K303B_AXIS_Y],tmp[K303B_AXIS_Z]);

    return snprintf(buf, PAGE_SIZE, "%d %d %d \n", tmp[K303B_AXIS_X], tmp[K303B_AXIS_Y], tmp[K303B_AXIS_Z]);

#if 0
	if((err = k303b_ReadOffset(client, obj->offset)))
	{
		return -EINVAL;
	}
	else if((err = k303b_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{
		mul = obj->reso.sensitivity/k303b_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
			obj->offset[K303B_AXIS_X], obj->offset[K303B_AXIS_Y], obj->offset[K303B_AXIS_Z],
			obj->offset[K303B_AXIS_X], obj->offset[K303B_AXIS_Y], obj->offset[K303B_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
			obj->cali_sw[K303B_AXIS_X], obj->cali_sw[K303B_AXIS_Y], obj->cali_sw[K303B_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
			obj->offset[K303B_AXIS_X]*mul + obj->cali_sw[K303B_AXIS_X],
			obj->offset[K303B_AXIS_Y]*mul + obj->cali_sw[K303B_AXIS_Y],
			obj->offset[K303B_AXIS_Z]*mul + obj->cali_sw[K303B_AXIS_Z],
			tmp[K303B_AXIS_X], tmp[K303B_AXIS_Y], tmp[K303B_AXIS_Z]);

		return len;
    }
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = k303b_i2c_client;
    struct k303b_i2c_data *obj = i2c_get_clientdata(client);	
	int err = 0;
	int x = 0, y = 0, z = 0;
	int dat[K303B_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if((err = k303b_ResetCalibration(client)))
		{
			SENSOR_ERR("reset offset err = %d\n", err);
		}
	}
	else if(3 == sscanf(buf, "%d %d %d", &x, &y, &z))
	{
        obj->cali_sw[K303B_AXIS_X] = (s16)x;
        obj->cali_sw[K303B_AXIS_Y] = (s16)y;
        obj->cali_sw[K303B_AXIS_Z] = (s16)z;
#if 0
		dat[K303B_AXIS_X] = x;
		dat[K303B_AXIS_Y] = y;
		dat[K303B_AXIS_Z] = z;
		if((err = k303b_WriteCalibration(client, dat)))
		{
			SENSOR_ERR("write calibration err = %d\n", err);
		}
#endif
	}
	else
	{
		SENSOR_ERR("invalid format\n");
	}

	return count;
}

#ifdef BACKUP_HIDDENREG
static ssize_t show_recovery_value(struct device_driver *ddri, char *buf)
{
	SENSOR_LOG("recovery value  : %d\n",do_recovery);
    return snprintf(buf, PAGE_SIZE, "%d\n",do_recovery);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_recovery_value(struct device_driver *ddri, const char *buf, size_t count)
{
	SENSOR_FUN();
	return count;
}

static ssize_t show_hidden_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = k303b_i2c_client;
    struct k303b_i2c_data *obj = i2c_get_clientdata(client);	
	 u8 databuf_reg[2] = {0,0};
	 u8 RegData[15] = {0,};
	 int res = -1;
	 int i=0;
	 SENSOR_FUN();
	
	 memset(databuf_reg, 0, sizeof(u8)*2);
	
	for(i=0;i<15;i++)
	 {
	   if((res = hwmsen_read_block(client, 0x0 + i, databuf_reg, 1)))
		 {
			 mdelay(5);
			 if((res = hwmsen_read_block(client, 0x0 + i, databuf_reg, 1))){
				 SENSOR_ERR("Read Error Reg = 0X%x, data=%d\n", i, databuf_reg[0]);
			 }
			 else{				 
				 RegData[i] = databuf_reg[0];
			 }
		 }
	   else
		 {
		 	RegData[i] = databuf_reg[0];
		 }
	   databuf_reg[0] = 0;
	   databuf_reg[1] = 0;
	 }

    return snprintf(buf, PAGE_SIZE, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",RegData[0],RegData[1],RegData[2],RegData[3],RegData[4],RegData[5],RegData[6],RegData[7],RegData[8],RegData[9],RegData[10],RegData[11],RegData[12],RegData[13],RegData[14]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_hidden_value(struct device_driver *ddri, const char *buf, size_t count)
{
	SENSOR_FUN();
	return count;
}
#endif
/*---------------------------------  NO --------------------------------------*/
static ssize_t show_self_value(struct device_driver *ddri, char *buf)
{
    int selftest = 1;  // fail

    struct i2c_client *client = k303b_i2c_client;
    struct k303b_i2c_data *obj = i2c_get_clientdata(client);

    if(atomic_read(&obj->selftest_rslt) == 1)  // selftest success
    {
		SENSOR_LOG("result success\n");
        selftest = 0;  // success
    }
    else
    {
		SENSOR_ERR("result fail\n");
        selftest = 1;  // fail
    }
	return sprintf(buf, "%d\n", selftest);
}
/*-------------------------------------  NO ---------------------------------------*/
static int k303b_get_selftest(struct i2c_client *client)
{
		int val, i;
		int en_state = 0;
		int err = 0;
		unsigned char x[8]={0};
		int NO_ST[3] = {0, 0, 0};
		int ST[3] = {0, 0, 0};
		s16 tmp = 0;
		u8 addr = K303B_REG_DATAX0 | I2C_AUTO_INCREMENT;
	
		// ODR setting
		x[0] = K303B_REG_BW_RATE;
		x[1] = 0xC7;  // 0x47
		i2c_master_send(client, x, 2);
	
		x[0] = K303B_REG_DATA_FORMAT;
		x[1] = 0x24;//0x04
		i2c_master_send(client, x, 2);
	
		x[0] = K303B_REG_DATA_FORMAT + 1;
		x[1] = 0x0;
		i2c_master_send(client, x, 2);
	
		x[0] = K303B_REG_DATA_FORMAT + 2;
		x[1] = 0x0;
		i2c_master_send(client, x, 2);
	
		mdelay(80);
	
		hwmsen_read_block(client, addr, x, 6);
	
		for(i = 0; i < 5; i++)
		{
			while(1)
			{
				// status_reg
				val = hwmsen_read_byte(client, 0x27, x);
				if(val < 0)
				{
					SENSOR_ERR("[SELFTEST] I2C fail point1\n");
					goto ST_EXIT;
				}
	
				if(x[0] & 0x08)  // x,y,z asix new data Available
				{
					break;
				}
			}
	
			hwmsen_read_block(client, addr, x, 6);
	
		   // 16 bit resolution 1LSB=0.061mg
			NO_ST[0] += (s16)(x[1] << 8 | x[0]);
			NO_ST[1] += (s16)(x[3] << 8 | x[2]);
			NO_ST[2] += (s16)(x[5] << 8 | x[4]);
	
		}
	
	
		NO_ST[0] /= 5;
		NO_ST[1] /= 5;
		NO_ST[2] /= 5;
	
		SENSOR_LOG("[SELFTEST] AVE_NO_ST : %d, %d, %d\n", NO_ST[0], NO_ST[1], NO_ST[2]);
	
		x[0] = K303B_REG_REG5;
		x[1] = 0x04;		// ST enable
		i2c_master_send(client, x, 2);
	
		mdelay(80);
	
		hwmsen_read_block(client, addr, x, 6);
	
		for(i = 0; i < 5; i++)
		{
			while(1)
			{
				val = hwmsen_read_byte(client, 0x27, x);
				if(val < 0)
				{
					SENSOR_ERR("[SELFTEST] I2C fail point2\n");
					goto ST_EXIT;
				}
	
				if(x[0] & 0x08)
				{
					break;
				}
			}
	
			hwmsen_read_block(client, addr, x, 6);
			
			// 16 bit resolution
			ST[0] += (s16)(x[1] << 8 | x[0]);
			ST[1] += (s16)(x[3] << 8 | x[2]);
			ST[2] += (s16)(x[5] << 8 | x[4]);
			 
		}
		ST[0] /= 5;
		ST[1] /= 5;
		ST[2] /= 5;    
	
		SENSOR_LOG("[SELFTEST] AVE_ST : %d, %d, %d\n", ST[0], ST[1], ST[2]);
	
		for(val = 0, i = 0; i < 3; i++)
		{
			// calculate differece between SelfTest value and zoro g offset in 10bit resolution
			ST[i] -= NO_ST[i];
			ST[i] = abs(ST[i]);
	
			// range compare of the self test
			if((HWST_LSL_LSB > ST[i]) || (ST[i] > HWST_USL_LSB))
			{
				SENSOR_ERR("[SELFTEST] ST[%d] : Out of range!! (%d)\n", i, ST[i]);
				val = -1;
			}
		}
	
		// check zero-g offset
		if(val >= 0)
		{
			for(val = 1, i = 0; i < 3; i++)
			{
				if(i < 2)
				{
					if(abs(NO_ST[i]) > TESTLIMIT_XY)  // X, Y axis flat check
					{
						SENSOR_ERR("[SELFTEST] NO_ST[%d] : Out of ZOffset!! (%d)\n", i, NO_ST[i]);
						val = -1;
					}
				}
				else
				{  // Z axis flat check
					if((abs(NO_ST[i]) > TESTLIMIT_Z_USL_LSB) || (abs(NO_ST[i]) < TESTLIMIT_Z_LSL_LSB))
					{
						SENSOR_ERR("[SELFTEST] NO_ST[%d] : Out of ZOffset!! (%d)\n", i, NO_ST[i]);
						val = -1;
					}
				}
			}
		}
	
		//if(atomic_read(&obj->trace) & K2DH_TRC_CALI)
		{
			if(val >= 0)
			{
				SENSOR_LOG("[SELFTEST] OK!! val : %d, (%d, %d, %d) ||| (%d, %d, %d)\n", val, ST[0], ST[1], ST[2], NO_ST[0], NO_ST[1], NO_ST[2]);
			}
			else
			{
				SENSOR_LOG("[SELFTEST] NG!! val : %d, (%d, %d, %d) ||| (%d, %d, %d)\n", val, ST[0], ST[1], ST[2], NO_ST[0], NO_ST[1], NO_ST[2]);
			}	
		}			 
	
	ST_EXIT:
#if 0
		x[0] = CTRL_REG1;
		x[1] = 0x00; // power down
		i2c_master_send(client, x, 1);
#endif
	
		x[0] = K303B_REG_REG5;
		x[1] = 0x00; // ST disable
		i2c_master_send(client, x, 2);
	
	
		k303b_init_client(client, 0);

    return val;    
}


static ssize_t store_self_value(struct device_driver *ddri, const char *buf, size_t count)
{
	unsigned long data = 0;
    int error = -1;
    struct i2c_client *client = k303b_i2c_client;
    struct k303b_i2c_data *obj = i2c_get_clientdata(client);

	if (error = strict_strtoul(buf, 10, &data))
    {
        return error;
    }
    SENSOR_LOG("Self test CMD value : %d\n", (int)data);
    if(data == 1)  // self test start command
    {
    	if(k303b_get_selftest(client) >= 0)
        {            
        	SENSOR_LOG("result : %d\n",1);
            atomic_set(&obj->selftest_rslt, 1);
	    }
	    else
	    {
        	SENSOR_LOG("result : %d\n",0);
            atomic_set(&obj->selftest_rslt, 0);
	    }          
    }
	else // wrong input
	{
        SENSOR_ERR("SelfTest FAIL\n");
        return -EINVAL;
	}
	return count;
}


/*----------------------------------------------------------------------------*/
static int k303b_do_calibration(void)
{
        struct K303Bacc acc_cal = {0, };
        struct K303Bacc acc_cal_pre = {0, };
        int sum[3] = {0, };
        int err = 0;
        int i;

        struct i2c_client *client = k303b_i2c_client;
        struct k303b_i2c_data *obj = i2c_get_clientdata(client);
      /* Sensor stable time after power on  */
       msleep ( 300 ); 

        err = k303b_ReadData(client, &acc_cal_pre);
        if(err < 0)
        {
            SENSOR_ERR("k303b_ReadData() failed\n");
            return err;
        }

        for(i = 0; i < CALIBRATION_DATA_AMOUNT; i++)
        {
            mdelay(20);
            err = k303b_ReadData(client, &acc_cal);
            if(err < 0)
            {
                SENSOR_ERR("k303b_ReadData() failed in the %dth loop\n", i);
                return err;
            }

            SENSOR_LOG("===============moved x=============== timeout = %d\n", i);
            SENSOR_LOG("(%d, %d, %d) (%d, %d, %d)\n", acc_cal_pre.x, acc_cal_pre.y,  acc_cal_pre.z, acc_cal.x,acc_cal.y,acc_cal.z );


            if((abs(acc_cal.x - acc_cal_pre.x) > K303B_SHAKING_DETECT_THRESHOLD)
                || (abs((acc_cal.y - acc_cal_pre.y)) > K303B_SHAKING_DETECT_THRESHOLD)
                || (abs((acc_cal.z - acc_cal_pre.z)) > K303B_SHAKING_DETECT_THRESHOLD))
            {
                SENSOR_ERR("===============shaking x===============\n");
                return K303B_ERR_STATUS;                
            }
            else
            {

                sum[K303B_AXIS_X] += acc_cal.x;
                sum[K303B_AXIS_Y] += acc_cal.y;
                sum[K303B_AXIS_Z] += acc_cal.z;

                acc_cal_pre.x = acc_cal.x;
                acc_cal_pre.y = acc_cal.y;
                acc_cal_pre.z = acc_cal.z;

            }

            SENSOR_LOG("calibration sum data (%d, %d, %d)\n", sum[K303B_AXIS_X], sum[K303B_AXIS_Y], sum[K303B_AXIS_Z]);            
            SENSOR_LOG("===============timeout_shaking: %d=============== \n",i);            
        }

        SENSOR_LOG("===============complete shaking x check===============\n");        

#if 1  /* LGE_BSP_COMMON LGE_CHANGE_S 140327 jongwoo82.lee : Calibration zero-g offset check */
        // check zero-g offset
        if((abs(sum[K303B_AXIS_X]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY) ||
            (abs(sum[K303B_AXIS_Y]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY) ||
            ((abs(sum[K303B_AXIS_Z]/CALIBRATION_DATA_AMOUNT) > TESTLIMIT_Z_USL_LSB) || (abs(sum[K303B_AXIS_Z]/CALIBRATION_DATA_AMOUNT) < TESTLIMIT_Z_LSL_LSB)))
        {
            SENSOR_ERR("Calibration zero-g offset check failed (%d, %d, %d)\n", sum[K303B_AXIS_X]/CALIBRATION_DATA_AMOUNT, 
                                                                    sum[K303B_AXIS_Y]/CALIBRATION_DATA_AMOUNT, sum[K303B_AXIS_Z]/CALIBRATION_DATA_AMOUNT);
             return K303B_ERR_SETUP_FAILURE;                            
        }        
#endif  /* LGE_BSP_COMMON LGE_CHANGE_E 140327 jongwoo82.lee : Calibration zero-g offset check */

	obj->cali_sw[K303B_AXIS_X] = -(s16)(sum[K303B_AXIS_X] / CALIBRATION_DATA_AMOUNT);	//K303B(16bit) 0+-154
	obj->cali_sw[K303B_AXIS_Y] = -(s16)(sum[K303B_AXIS_Y] / CALIBRATION_DATA_AMOUNT);	//K303B(16bit) 0+-154

        if(sum[K303B_AXIS_Z] < 0)
        {
		obj->cali_sw[K303B_AXIS_Z] = -(s16)(sum[K303B_AXIS_Z] / CALIBRATION_DATA_AMOUNT + 8192) ;	// K303B(16bit) 8192 +- 226
        }
        else
        {
		obj->cali_sw[K303B_AXIS_Z] = -(s16)(sum[K303B_AXIS_Z] / CALIBRATION_DATA_AMOUNT - 8192);	// K303B(16bit) 8192 +- 226
        }

        SENSOR_LOG("calibration data (%d, %d, %d)\n",obj->cali_sw[K303B_AXIS_X], obj->cali_sw[K303B_AXIS_Y], obj->cali_sw[K303B_AXIS_Z]);
        return K303B_SUCCESS;
}

#if 1  /* LGE_BSP_COMMON LGE_CHANGE_S 140127 jongwoo82.lee : Calibration for user Apps */
static int k303b_runCalibration(void)
{
	struct i2c_client *client = k303b_i2c_client;
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	int backup_x, backup_y, backup_z;
#if 0 //motion sensor cal backup
		int cali[3];
#else
		int cali[6];
#endif
	int res = K303B_ERR_STATUS;
	SENSOR_FUN();
	backup_x = obj->cali_sw[K303B_AXIS_X];
	backup_y = obj->cali_sw[K303B_AXIS_Y];
	backup_z = obj->cali_sw[K303B_AXIS_Z];
	SENSOR_LOG("backup_offset_x: %d, backup_offset_y: %d, backup_offset_z: %d\n",backup_x, backup_y, backup_z);

	if(sensor_power == FALSE)
	{
		res = k303b_SetPowerMode(client, true);
		if(res)
		{
			SENSOR_ERR("Power on k303b error %d!\n", res);
			return FALSE;
		}
	}
	res = k303b_do_calibration();
    if(res != K303B_SUCCESS)
    {
		//SENSOR_LOG("self calibration FAIL\n");
        if(res==K303B_ERR_SETUP_FAILURE)
			return K303B_ERR_SETUP_FAILURE;
		else
			return K303B_ERR_STATUS;
    }
	else
	{
#if 1 //motion sensor cal backup
		cali[3] = (int)backup_x;
		cali[4] = (int)backup_y;
		cali[5] = (int)backup_z;
#endif
		cali[0] = obj->cali_sw[K303B_AXIS_X];
		cali[1] = obj->cali_sw[K303B_AXIS_Y];
		cali[2] = obj->cali_sw[K303B_AXIS_Z];
#if defined(CALIBRATION_TO_FILE)
         k303b_calibration_save(cali);
#else
		 if(LGE_FacWriteAccelerometerCalibration((unsigned int*)cali) == TRUE)
		 {
		 	SENSOR_LOG("Calibration factory write END\n");
		 }
#endif
		SENSOR_LOG("self calibration Done\n");
	}	

    return K303B_SUCCESS;
}
#endif  /* LGE_BSP_COMMON LGE_CHANGE_E 140127 jongwoo82.lee : Calibration for user Apps */


/* LGE_BSP_COMMON LGE_CHANGE_S 140228 : Calibration for user Apps */
static ssize_t k303b_runCalibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = k303b_i2c_client;
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	res = k303b_runCalibration();
	if(res == K303B_SUCCESS)
	{
		SENSOR_LOG("self calibration K303B_SUCCESS\n");
        atomic_set(&obj->fast_calib_rslt, K303B_SUCCESS);
	}
	else if(res == K303B_ERR_SETUP_FAILURE)
	{
		SENSOR_LOG("self calibration K303B_ERR_SETUP_FAILURE\n");
		atomic_set(&obj->fast_calib_rslt, 2);  // cal fail(flat)
	}
	else
	{
		SENSOR_LOG("self calibration FAIL\n");
		atomic_set(&obj->fast_calib_rslt, 1);  // cal fail
	}
	return count;
}

static ssize_t k303b_runCalibration_show(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = k303b_i2c_client;
    struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	SENSOR_LOG("calibration result show: %d\n", atomic_read(&obj->fast_calib_rslt));
	return sprintf(buf, "%d\n", atomic_read(&obj->fast_calib_rslt));
}
/* LGE_BSP_COMMON LGE_CHANGE_E 140228 : Calibration for user Apps */

static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_K303B_LOWPASS
	struct i2c_client *client = k303b_i2c_client;
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		SENSOR_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			SENSOR_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][K303B_AXIS_X], obj->fir.raw[idx][K303B_AXIS_Y], obj->fir.raw[idx][K303B_AXIS_Z]);
		}

		SENSOR_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[K303B_AXIS_X], obj->fir.sum[K303B_AXIS_Y], obj->fir.sum[K303B_AXIS_Z]);
		SENSOR_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[K303B_AXIS_X]/len, obj->fir.sum[K303B_AXIS_Y]/len, obj->fir.sum[K303B_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_K303B_LOWPASS
	struct i2c_client *client = k303b_i2c_client;
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		SENSOR_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		SENSOR_ERR("exceeds maximum filter length\n");
	}
	else
	{
		atomic_set(&obj->firlen, firlen);
		if(0 == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct k303b_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		SENSOR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct k303b_i2c_data *obj = obj_i2c_data;
	unsigned int trace;
	if (obj == NULL)
	{
		SENSOR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		SENSOR_ERR("invalid content: '%s', length = %d\n", buf, count);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buffer)
{
	struct i2c_client *client = k303b_i2c_client;

	struct k303b_i2c_data *priv = i2c_get_clientdata(client);
	u8 addr = K303B_REG_CTL0 | I2C_AUTO_INCREMENT;
	u8 buf[8] = {0};
	int err = 0;
	ssize_t len = 0;

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if((err = hwmsen_read_block(client, addr, buf, 0x08)))
	{
		SENSOR_ERR("error: %d\n", err);
	}

	len += snprintf(buffer+len, PAGE_SIZE, "0x%04X ,¡\\t 0x%04X ,¡\\t 0x%04X , \t0x%04X,   \n  0x%04X ,¡\\t  0x%04X ,¡\\t0x%04X ,\t  0x%04X , \t  \n ", buf[0],	buf[1],	buf[2],	buf[3],	buf[4],	buf[5],	buf[6],	buf[7]);

	struct k303b_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		SENSOR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if(obj->hw)
	{
		len += snprintf(buffer+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;
}

static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	int relv = 0;
	if(sensor_power)
		relv = snprintf(buf, PAGE_SIZE, "1\n");
	else
		relv = snprintf(buf, PAGE_SIZE, "0\n");

	return relv;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(layout,     S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(cali, S_IWUSR|S_IRUGO|S_IWGRP, show_cali_value,          store_cali_value);
static DRIVER_ATTR(selftest,S_IWUSR|S_IRUGO|S_IWGRP, show_self_value ,      store_self_value );
static DRIVER_ATTR(firlen,S_IWUSR|S_IRUGO|S_IWGRP, show_firlen_value,       store_firlen_value);
static DRIVER_ATTR(trace, S_IWUSR|S_IRUGO|S_IWGRP, show_trace_value,        store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,          S_IRUGO, show_power_status_value,        NULL);
#ifdef BACKUP_HIDDENREG
static DRIVER_ATTR(hidden, S_IWUSR|S_IRUGO|S_IWGRP, show_hidden_value,          store_hidden_value);
static DRIVER_ATTR(recovery, S_IWUSR|S_IRUGO|S_IWGRP, show_recovery_value,          store_recovery_value);
#endif
#if 1  /* LGE_BSP_COMMON LGE_CHANGE_S 140127 jongwoo82.lee : Calibration for user Apps */
static DRIVER_ATTR(run_fast_calibration, S_IRUGO|S_IWUSR|S_IWGRP, k303b_runCalibration_show, k303b_runCalibration_store);
#endif  /* LGE_BSP_COMMON LGE_CHANGE_E 140127 jongwoo82.lee : Calibration for user Apps */
#if 1 //motion sensor cal backup
static DRIVER_ATTR(cali_backup, S_IWUSR|S_IRUGO|S_IWGRP, show_cali_backup_value, NULL);
#endif

/*----------------------------------------------------------------------------*/
static struct driver_attribute *k303b_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_layout,
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_selftest,     /*self test*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
#ifdef BACKUP_HIDDENREG
	&driver_attr_hidden,
	&driver_attr_recovery,
#endif
#if 1  /* LGE_BSP_COMMON LGE_CHANGE_S 140127 jongwoo82.lee : Calibration for user Apps */
	&driver_attr_run_fast_calibration,
#endif  /* LGE_BSP_COMMON LGE_CHANGE_E 140127 jongwoo82.lee : Calibration for user Apps */    
#if 1 //motion sensor cal backup
	&driver_attr_cali_backup,         /*show calibration backup data*/
#endif
};


/*----------------------------------------------------------------------------*/
static int k303b_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(k303b_attr_list)/sizeof(k303b_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, k303b_attr_list[idx])))
		{
			SENSOR_ERR("driver_create_file (%s) = %d\n", k303b_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int k303b_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(k303b_attr_list)/sizeof(k303b_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, k303b_attr_list[idx]);
	}

	return err;
}



/*----------------------------------------------------------------------------*/
int k303b_gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;
	struct k303b_i2c_data *priv = (struct k303b_i2c_data*)self;
	hwm_sensor_data* gsensor_data;
	char buff[K303B_BUFSIZE];

	//SENSOR_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:

			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				SENSOR_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				SENSOR_LOG("SENSOR_DELAY %d\n", value);
				if(value <= 5)
				{
					sample_delay = K303B_ACC_ODR100;
				}
				else if(value <= 10)
				{
					sample_delay = K303B_ACC_ODR100;
				}
				else
				{
					sample_delay = K303B_ACC_ODR50;
				}

				err = k303b_SetBWRate(priv->client, sample_delay);
				if(err != K303B_SUCCESS ) //0x2C->BW=100Hz
				{
					SENSOR_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{
				#if defined(CONFIG_K303B_LOWPASS)
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[K303B_AXIS_X] = 0;
					priv->fir.sum[K303B_AXIS_Y] = 0;
					priv->fir.sum[K303B_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
				#endif
				}
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				SENSOR_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				SENSOR_LOG("SENSOR_ENABLE %d\n",value);
				#if 1
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					SENSOR_LOG("Gsensor device have updated!, power: %d\n", sensor_power);
				}
				else
				{
					err = k303b_SetPowerMode( priv->client, !sensor_power);
				}
				#else
				if(value == 0)
				{
					err = k303b_SetPowerMode( priv->client, 0);
					SENSOR_LOG("Gsensor device false!\n");
				}
				else
				{
					err = k303b_SetPowerMode( priv->client, 1);
					SENSOR_LOG("Gsensor device true!\n");
				}
				#endif
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				SENSOR_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;
				err = k303b_ReadSensorData(priv->client, buff, K303B_BUFSIZE);
				if(!err)
				{
				   sscanf(buff, "%x %x %x", &gsensor_data->values[0],
					   &gsensor_data->values[1], &gsensor_data->values[2]);
				   gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				   gsensor_data->value_divide = 1000;
				}
			}
			break;
		default:
			SENSOR_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int k303b_open(struct inode *inode, struct file *file)
{
	file->private_data = k303b_i2c_client;

	if(file->private_data == NULL)
	{
		SENSOR_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static long k303b_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct k303b_i2c_data *obj = (struct k303b_i2c_data*)i2c_get_clientdata(client);
	char strbuf[K303B_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	long err = 0;
#if defined(CALIBRATION_TO_FILE)
	int cali[6] = {0};
#else
    int cali[3] = {0};
#endif
	uint32_t enable = 0;

	//SENSOR_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		SENSOR_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_SET_ENABLE:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}
			//SENSOR_LOG("GSENSOR_IOCTL_SET_ENABLE %d\n",data);
			if(copy_from_user(&enable, data, sizeof(enable)))
			{
				return -EFAULT;
			}
			else
			{
				if(enable == 1)
				{
					k303b_SetPowerMode(obj->client, 1);
				}
				else if(enable == 0)
				{
					k303b_SetPowerMode(obj->client, 0);
				}
			}
			break;

		case GSENSOR_IOCTL_INIT:
			k303b_init_client(client, 0);
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}

			k303b_ReadChipInfo(client, strbuf, K303B_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}

			k303b_ReadSensorData(client, strbuf, K303B_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}

			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}
			k303b_ReadRawData(client, strbuf);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}
			break;

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}
#if defined(CALIBRATION_TO_FILE)
				if(make_cal_data_file() == K303B_SUCCESS)
				{
	            err = k303b_calibration_read(cali);
				if(err!=0){
				    SENSOR_ERR("Read Cal Fail from file !!!\n");
					break;
				}
				else
					{
	                  sensor_data.x = cali[0];
	                  sensor_data.y = cali[1];
	                  sensor_data.z = cali[2];
					}
				}
#endif
			if(atomic_read(&obj->suspend))
			{
				SENSOR_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				SENSOR_LOG("GSENSOR_IOCTL_SET_CALI, Cal data : x=%d, y=%d, z=%d\n", sensor_data.x, sensor_data.y, sensor_data.z);
				if((sensor_data.x == 0) && (sensor_data.y == 0) && (sensor_data.z == 0))
				{  // temp defensive code for nvram_daemon ioctl
					break;
				}
				#if 0
				cali[K303B_AXIS_X] = sensor_data.x * obj->reso.sensitivity / GRAVITY_EARTH_1000;
				cali[K303B_AXIS_Y] = sensor_data.y * obj->reso.sensitivity / GRAVITY_EARTH_1000;
				cali[K303B_AXIS_Z] = sensor_data.z * obj->reso.sensitivity / GRAVITY_EARTH_1000;
				err = k303b_WriteCalibration(client, cali);
				#else
				 obj->cali_sw[K303B_AXIS_X] = (s16)sensor_data.x;
				 obj->cali_sw[K303B_AXIS_Y] = (s16)sensor_data.y;
				 obj->cali_sw[K303B_AXIS_Z] = (s16)sensor_data.z; 
				if(obj->cali_sw[K303B_AXIS_X]>1000|| obj->cali_sw[K303B_AXIS_X]<-1000||
				obj->cali_sw[K303B_AXIS_Y]>1000|| obj->cali_sw[K303B_AXIS_Y]<-1000||
				obj->cali_sw[K303B_AXIS_Z]>1000|| obj->cali_sw[K303B_AXIS_Z]<-1000){
					SENSOR_ERR("Unnormal Cal Data");
					obj->cali_sw[K303B_AXIS_X] = 0;
					obj->cali_sw[K303B_AXIS_Y] = 0;
					obj->cali_sw[K303B_AXIS_Z] = 0;
				}
				#endif
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = k303b_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			#if 0
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}
			if((err = k303b_ReadCalibration(client, cali)))
			{
				break;
			}

			sensor_data.x = cali[K303B_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso.sensitivity;
			sensor_data.y = cali[K303B_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso.sensitivity;
			sensor_data.z = cali[K303B_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso.sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}
			#else
            SENSOR_LOG("GSENSOR_IOCTL_GET_CALI\n");
			#endif
			break;


		default:
			SENSOR_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;

	}

	return err;
}


/*----------------------------------------------------------------------------*/
static int k303b_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static struct file_operations k303b_fops = {
//	.owner = THIS_MODULE,
	.open = k303b_open,
	.release = k303b_release,
	.unlocked_ioctl = k303b_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice k303b_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &k303b_fops,
};

/*----------------------------------------------------------------------------*/
//#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int k303b_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	SENSOR_FUN();

	if(msg.event == PM_EVENT_SUSPEND)
	{
		if(obj == NULL)
		{
			SENSOR_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);
		k303b_power(obj->hw, 0);
		SENSOR_LOG("k303b_suspend ok\n");
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int k303b_resume(struct i2c_client *client)
{
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	SENSOR_FUN();

	if(obj == NULL)
	{
		SENSOR_ERR("null pointer!!\n");
		return -EINVAL;
	}

	k303b_power(obj->hw, 1);

	atomic_set(&obj->suspend, 0);
	SENSOR_LOG("k303b_resume ok\n");

	return 0;
}
/*----------------------------------------------------------------------------*/
//#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void k303b_early_suspend(struct early_suspend *h)
{
	struct k303b_i2c_data *obj = container_of(h, struct k303b_i2c_data, early_drv);
	SENSOR_FUN();

	if(obj == NULL)
	{
		SENSOR_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1);
	k303b_power(obj->hw, 0);
	SENSOR_LOG("k303b_early_suspend ok\n");
}
/*----------------------------------------------------------------------------*/
static void k303b_late_resume(struct early_suspend *h)
{
	struct k303b_i2c_data *obj = container_of(h, struct k303b_i2c_data, early_drv);
	SENSOR_FUN();

	if(obj == NULL)
	{
		SENSOR_ERR("null pointer!!\n");
		return;
	}

	k303b_power(obj->hw, 1);
	atomic_set(&obj->suspend, 0);
	SENSOR_LOG("k303b_late_resume ok\n");
}


/*----------------------------------------------------------------------------*/
static int k303b_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct k303b_i2c_data *obj;
	struct hwmsen_object sobj;
	int err = 0;
	SENSOR_FUN();

		
#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
	if(k303b_init_flag==0)
	{
		SENSOR_LOG("Accelerometer Sensor already probed...just skip\n");
		return err;
	}
#endif
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct k303b_i2c_data));

	obj->hw = get_cust_acc_hw();

	if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		SENSOR_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);

#ifdef CONFIG_K303B_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}

	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}

#endif

	k303b_i2c_client = new_client;

	if((err = k303b_init_client(new_client, 1)))
	{
		goto exit_init_failed;
	}

	if((err = misc_register(&k303b_device)))
	{
		SENSOR_ERR("k303b_device register failed\n");
		goto exit_misc_device_register_failed;
	}

#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
	if(err = k303b_create_attr(&(gsensor_init_info.platform_diver_addr->driver)))
	{
		SENSOR_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
#else
	if((err = k303b_create_attr(&k303b_gsensor_driver.driver)))
	{
		SENSOR_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
#endif

	sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = k303b_gsensor_operate;
	if((err = hwmsen_attach(ID_ACCELEROMETER, &sobj)))
	{
		SENSOR_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = k303b_early_suspend,
	obj->early_drv.resume   = k303b_late_resume,
	register_early_suspend(&obj->early_drv);
#endif

	SENSOR_LOG("OK\n");
#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
	k303b_init_flag=0; 
#endif
	return 0;

	exit_create_attr_failed:
	misc_deregister(&k303b_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	SENSOR_ERR("err = %d\n",err);
#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
	k303b_init_flag=-1; 
#endif
	return err;
}

/*----------------------------------------------------------------------------*/
static int k303b_i2c_remove(struct i2c_client *client)
{
	int err = 0;

#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
	if(err = k303b_delete_attr(&(gsensor_init_info.platform_diver_addr->driver)))
	{
		SENSOR_ERR("k303b_delete_attr fail: %d\n", err);
	}
#else
	if((err = k303b_delete_attr(&k303b_gsensor_driver.driver)))
	{
		SENSOR_ERR("k303b_delete_attr fail: %d\n", err);
	}
#endif

	if((err = misc_deregister(&k303b_device)))
	{
		SENSOR_ERR("misc_deregister fail: %d\n", err);
	}

	if((err = hwmsen_detach(ID_ACCELEROMETER)))
	{
		SENSOR_ERR("hwmsen_detach fail: %d\n", err);
	}

	k303b_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int k303b_probe(struct platform_device *pdev)
{
	struct acc_hw *hw = get_cust_acc_hw();
	SENSOR_FUN();

	k303b_power(hw, 1);
	if(i2c_add_driver(&k303b_i2c_driver))
	{
		SENSOR_ERR("add driver error\n");
		return -1;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int k303b_remove(struct platform_device *pdev)
{
    struct acc_hw *hw = get_cust_acc_hw();
    SENSOR_FUN();

    k303b_power(hw, 0);
    i2c_del_driver(&k303b_i2c_driver);
    return 0;
}
#ifdef CONFIG_OF_DT
static const struct of_device_id m_acc_pl_of_match[] = {
	{ .compatible = "mediatek,gsensor1", },
	{},
};
#endif
/*----------------------------------------------------------------------------*/
static struct platform_driver k303b_gsensor_driver = {
	.probe      = k303b_probe,
	.remove     = k303b_remove,
	.driver     = {
	.name  = "gsensor",
#ifdef CONFIG_OF_DT
	.of_match_table = m_acc_pl_of_match,
#endif
	}
};

#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
static int  k303b_local_init(void)
{
    struct acc_hw *hw = get_cust_acc_hw();
   	SENSOR_FUN();

	if(i2c_add_driver(&k303b_i2c_driver))
	{
		SENSOR_ERR("add driver error\n");
		return -1;
	}
	if(-1 == k303b_init_flag)
	{
	   return -1;
	}

	return 0;
}

static int  k303b_local_remove(void)
{
    struct acc_hw *hw = get_cust_acc_hw();

    SENSOR_FUN();
    i2c_del_driver(&k303b_i2c_driver);
    return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int __init k303b_init(void)
{
	SENSOR_FUN();
	struct acc_hw *hw = get_cust_acc_hw();

	i2c_register_board_info(hw->i2c_num, &i2c_k303b, 1);
#if defined(CONFIG_MTK_AUTO_DETECT_ACCELEROMETER)
	hwmsen_gsensor_add(&gsensor_init_info);
#else
	if(platform_driver_register(&k303b_gsensor_driver))
	{
		SENSOR_ERR("Failed to register platform driver\n");
		return -ENODEV;
	}
#endif
	SENSOR_LOG("i2c_number = %d / k303b_init success\n\n", hw->i2c_num);
	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit k303b_exit(void)
{
	SENSOR_FUN();
	platform_driver_unregister(&k303b_gsensor_driver);
}

/*----------------------------------------------------------------------------*/
module_init(k303b_init);
module_exit(k303b_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("K303B-ACC I2C driver");
MODULE_AUTHOR("RUO.liang@mediatek.com");
