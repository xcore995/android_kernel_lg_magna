
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
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>


#include <cust_mag.h>
#include "k303b.h"
#include <linux/hwmsen_helper.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/board_lge.h>

/*----------------------------------------------------------------------------*/
#define K303B_AXIS_X          0
#define K303B_AXIS_Y          1
#define K303B_AXIS_Z          2
#define K303B_AXES_NUM        3
#define K303B_DATA_LEN        6
#define K303B_DEV_NAME        "k303b-mag"

static int k303b_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int k303b_i2c_remove(struct i2c_client *client);
//static int k303b_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int k303b_suspend(struct i2c_client *client, pm_message_t msg) ;
static int k303b_resume(struct i2c_client *client);
static void k303b_power(struct mag_hw *hw, unsigned int on);
static int k303b_SetPowerMode(struct i2c_client *client, bool enable);
static atomic_t dev_open_count;

/*----------------------------------------------------------------------------*/
#define SENSOR_TAG                  "[LGE_Magnetometer]"
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
#define MSE_VER(fmt, args...)   ((void)0)

static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;
static int Count = 0;

/*-------------------------MT6516&MT6573 define-------------------------------*/

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#define MSENSOR_IOCTL_SET_SIMUGYRO        	      _IOW(MSENSOR, 0x53, int)
#define MSENSOR_IOCTL_READ_SIMUGYRODATA		      _IOR(MSENSOR, 0x54, int)
#define MSENSOR_IOCTL_SET_ROTATION_VECTOR  	      _IOW(MSENSOR, 0x55, int)
#define MSENSOR_IOCTL_READ_ROTATION_VECTOR_DATA	  _IOR(MSENSOR, 0x56, int)


#define SW_CALIBRATION

/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_K303B 345
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define CONFIG_K303B_LOWPASS   /*apply low pass filter on output*/



/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  =     0x01,
    ADX_TRC_RAWDATA =     0x02,
    ADX_TRC_IOCTL   =     0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,

} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
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
struct _k303b_data {
    rwlock_t lock;
    int mode;
    int rate;
    volatile int updated;
} k303b_data;

/*----------------------------------------------------------------------------*/
struct k303b_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw;
    struct hwmsen_convert   cvt;

    /*misc*/
    struct data_resolution  reso;
    atomic_t                trace;
	atomic_t 				layout;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[K303B_AXES_NUM+1];

    /*data*/
    s8                      offset[K303B_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s32                     data[K303B_AXES_NUM+1];

#if defined(CONFIG_K303B_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};


/*----------------------------------------------------------------------------*/
static const struct i2c_device_id k303b_i2c_id[] = {{K303B_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_k303b={ I2C_BOARD_INFO("k303b-mag", K303B_I2C_ADDRESS>>1)};
/*the adapter id will be available in customization*/
//static unsigned short k303b_force[] = {0x00, K303B_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const k303b_forces[] = { k303b_force, NULL };
//static struct i2c_client_address_data k303b_addr_data = { .forces = k303b_forces,};
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static struct i2c_client *k303b_i2c_client = NULL;
static struct platform_driver k303b_msensor_driver;
static struct k303b_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
//static MSENSOR_VECTOR3D msensor_gain;
static char selftestRes[8]= {0};
/*----------------------------------------------------------------------------*/
struct k303b_data {
    rwlock_t datalock;
    rwlock_t ctrllock;
    int controldata[10];
    unsigned int debug;
    int yaw;
    int roll;
    int pitch;

    int simuGyrox;
    int simuGyroy;
    int simuGyroz;
    int RVx;
    int RVy;
    int RVz;

    int nmx;
    int nmy;
    int nmz;
    int nax;
    int nay;
    int naz;
    int mag_status;
}k303bmid_data;
/*----------------------------------------------------------------------------*/
#if defined(CONFIG_MTK_AUTO_DETECT_MAGNETOMETER) //for auto detect
static int k303b_local_init(void);
static int k303b_local_remove(void);
static int k303b_init_flag = -1;//0<==>OK, -1<==>fail
static struct sensor_init_info msensor_init_info = {
	.name = "K303B",
	.init = k303b_local_init,
	.uninit = k303b_local_remove,
};
#endif

/*----------------------------------------------------------------------------*/

static struct i2c_driver k303b_i2c_driver = {
    .driver = {
        .name           = K303B_DEV_NAME,
    },
	.probe      		= k303b_i2c_probe,
	.remove    			= k303b_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
    .suspend            = k303b_suspend,
    .resume             = k303b_resume,
#endif
	.id_table = k303b_i2c_id,
};
#define LSM_ALL_SENSORS ~0x0
int inline get_sensors_status(int mask)
{
	int state;
	read_lock(&k303bmid_data.ctrllock);
	state = (k303bmid_data.controldata[7] & mask);
	read_unlock(&k303bmid_data.ctrllock);
	return state;
}

#if 0
#define K303B_M_NEW_ARCH   //susport kk new msensor arch
#endif
#ifdef K303B_M_NEW_ARCH
#include "mag.h"

static int k303b_open_report_data(int en)
{
	return 0;
}
static int k303b_set_delay(u64 delay)
{
	//int value = (int)delay/1000/1000;
	int value = (int)delay;

    SENSOR_LOG("k303b_set_delay = %d\n", value);//debug

	if(value <= 20)
	{
 		value = 20;
	}
	k303bmid_data.controldata[0] = value;  // Loop Delay

	return 0;
}
static int k303b_enable(int en)
{
	k303b_SetPowerMode(k303b_i2c_client, en);
	read_lock(&k303bmid_data.ctrllock);
    //en = 1; //enable Liming need delete
	if(en == 1)	
	{
		k303bmid_data.controldata[7] |= SENSOR_MAGNETIC;
	}
	else
	{
		k303bmid_data.controldata[7] &= ~SENSOR_MAGNETIC;
	}
	wake_up(&open_wq);
	read_unlock(&k303bmid_data.ctrllock);

	SENSOR_LOG("msensor enable/disable ok!status = %d\n",get_sensors_status(SENSOR_MAGNETIC));//debug

	return 0;
}
static int k303b_o_open_report_data(int en)
{
	return 0;
}
static int k303b_o_set_delay(u64 delay)
{
    int mdelay = (int)delay/1000/1000;
    k303b_set_delay(mdelay);
    return 0;
}
static int k303b_o_enable(int en)
{
	k303b_SetPowerMode(k303b_i2c_client, en);
	read_lock(&k303bmid_data.ctrllock);
	if(en == 1)
	{
		k303bmid_data.controldata[7] |= SENSOR_ORIENTATION;
	}
	else
	{
		k303bmid_data.controldata[7] &= ~SENSOR_ORIENTATION;
	}
	wake_up(&open_wq);
	read_unlock(&k303bmid_data.ctrllock);

	SENSOR_LOG("osensor enable/disable ok!status = %d\n",get_sensors_status(SENSOR_ORIENTATION));//debug
	                // Do nothing
	return 0;
}

#endif

/*----------------------------------------------------------------------------*/
static struct data_resolution k303b_data_resolution[] = {
 /*8 combination by {FULL_RES,RANGE}*/
    {{ 3, 9}, 256},   /*+/-2g  in 10-bit resolution:  3.9 mg/LSB*/
    {{ 7, 8}, 128},   /*+/-4g  in 10-bit resolution:  7.8 mg/LSB*/
    {{15, 6},  64},   /*+/-8g  in 10-bit resolution: 15.6 mg/LSB*/
    {{31, 2},  32},   /*+/-16g in 10-bit resolution: 31.2 mg/LSB*/
    {{ 3, 9}, 256},   /*+/-2g  in 10-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 3, 9}, 256},   /*+/-4g  in 11-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 3, 9}, 256},   /*+/-8g  in 12-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 3, 9}, 256},   /*+/-16g in 13-bit resolution:  3.9 mg/LSB (full-resolution)*/
};
/*----------------------------------------------------------------------------*/
static struct data_resolution k303b_offset_resolution = {{15, 6}, 64};

/*--------------------ADXL power control function----------------------------------*/
static void k303b_power(struct mag_hw *hw, unsigned int on)
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
			if(!hwPowerOn(hw->power_id, hw->power_vol, "K303B-mag"))
			{
				SENSOR_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "K303B-mag"))
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
	u8 databuf[10];
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
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



/*---------------------------------------  YES  -------------------------------------*/
static int k303b_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];
	int res = 0;
	u8 addr = K303B_REG_POWER_CTL;
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	SENSOR_FUN(f);
	if(enable == sensor_power)
	{
		SENSOR_LOG("Sensor power status is newest!\n");
		return K303B_SUCCESS;
	}

	if(enable == TRUE)
	{
		databuf[0] = K303B_MAG_POWER_ON;
	}
	else
	{
		databuf[0] = K303B_MAG_POWER_OFF;
	}
	databuf[1] = databuf[0];
	databuf[0] = K303B_REG_POWER_CTL;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		SENSOR_ERR("set power mode failed!\n");
		return K303B_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		SENSOR_LOG("set power mode ok %d!\n", databuf[1]);
	}
    //SENSOR_LOG("k303b_SetPowerMode end!\n");
	sensor_power = enable;
	return 0;
}


/*-----------------------------------   ok  -----------------------------------------*/
static int k303b_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];
	int res = 0;

	SENSOR_DBG("\n");
	memset(databuf, 0, sizeof(u8)*10);

	bwrate = (ODR_MAG_MASK | bwrate);


	databuf[0] = K303B_REG_BW_RATE;
	databuf[1] = bwrate;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return K303B_ERR_I2C;
	}

	return K303B_SUCCESS;
}


/*----------------------------------  done  ------------------------------------------*/
static int k303b_SetDataResolution(struct k303b_i2c_data *obj, u8 new_fs_range)
{
	SENSOR_DBG("\n");
	int err;
	u8  dat, reso;

	switch (new_fs_range) {
	case K303B_MAG_FS_16G:
		obj->reso.sensitivity = SENSITIVITY_MAG_16G;
		break;
	default:
		obj->reso.sensitivity = SENSITIVITY_MAG_16G;
		SENSOR_ERR("invalid magnetometer fs range requested: %u\n", new_fs_range);
		return -1;
	}

	return 0;


}


/*---------------------------------  ok -------------------------------------------*/
static int k303b_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	int res = 0;

	SENSOR_DBG("\n");
	memset(databuf, 0, sizeof(u8)*10);

	dataformat = ((K303B_MAG_FS_MASK & dataformat));

	databuf[0] = K303B_REG_DATA_FORMAT;
	databuf[1] = dataformat;

	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		return K303B_ERR_I2C;
	}

	return k303b_SetDataResolution(obj, dataformat);

}





/*----------------------------------------------------------------------------*/
static int k303b_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
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

	// 2 POWER MODE  no
	res = k303b_SetPowerMode(client, false);
	if(res != K303B_SUCCESS)
	{
	    SENSOR_ERR("set power error\n");
		return res;
	}

	// 3 RATE  YES
	res = k303b_SetBWRate(client, K303B_MAG_ODR40);
	if(res != K303B_SUCCESS ) //0x2C->BW=100Hz
	{
	    SENSOR_ERR("set power error\n");
		return res;
	}

	// 4 RANGE  ok
	res = k303b_SetDataFormat(client, K303B_MAG_FS_16G);
	if(res != K303B_SUCCESS) //0x2C->BW=100Hz
	{
	    SENSOR_ERR("set data format error\n");
		return res;
	}



#ifdef CONFIG_K303B_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	return K303B_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int k303b_ReadData(struct i2c_client *client, s32 data[K303B_AXES_NUM])
{
	struct k303b_i2c_data *priv = i2c_get_clientdata(client);
	u8 addr = K303B_REG_DATAX0 | I2C_AUTO_INCREMENT;
	u8 buf[K303B_DATA_LEN] = {0};
	int err = 0;

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
		data[K303B_AXIS_X] = ((s32)( (s16)((buf[K303B_AXIS_X*2+1] << 8) | (buf[K303B_AXIS_X*2]))));
		data[K303B_AXIS_Y] = ((s32)( (s16)((buf[K303B_AXIS_Y*2+1] << 8) | (buf[K303B_AXIS_Y*2]))));
		data[K303B_AXIS_Z] = ((s32)( (s16)((buf[K303B_AXIS_Z*2+1] << 8) | (buf[K303B_AXIS_Z*2]))));
/*
		SENSOR_LOG("k303b_ReadData [%08X %08X %08X] => [%5d %5d %5d]\n", data[K303B_AXIS_X], data[K303B_AXIS_Y], data[K303B_AXIS_Z],
		                               data[K303B_AXIS_X], data[K303B_AXIS_Y], data[K303B_AXIS_Z]);*/
		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			SENSOR_ERR("[%08X %08X %08X] => [%5d %5d %5d]\n", data[K303B_AXIS_X], data[K303B_AXIS_Y], data[K303B_AXIS_Z],
		                               data[K303B_AXIS_X], data[K303B_AXIS_Y], data[K303B_AXIS_Z]);
		}

	}
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
	int mag[K303B_DATA_LEN];
	int res = 0;
	struct k303b_i2c_data *obj = obj_i2c_data; //(struct k303b_i2c_data*)i2c_get_clientdata(client);
	client = obj->client;
	//u8 databuf[20];
	//memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}
/*
	if(sensor_power == FALSE)
	{
		res = k303b_SetPowerMode(client, true);
		if(res)
		{
			SENSOR_ERR("Power on k303b error %d!\n", res);
		}
		msleep(20);
	}
*/
	if((res = k303b_ReadData(client, obj->data)))
	{
		SENSOR_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{

		/*remap coordinate*/

		mag[obj->cvt.map[K303B_AXIS_X]] = obj->cvt.sign[K303B_AXIS_X]*obj->data[K303B_AXIS_X] * obj->reso.sensitivity / 1000;
		mag[obj->cvt.map[K303B_AXIS_Y]] = obj->cvt.sign[K303B_AXIS_Y]*obj->data[K303B_AXIS_Y] * obj->reso.sensitivity / 1000;
		mag[obj->cvt.map[K303B_AXIS_Z]] = obj->cvt.sign[K303B_AXIS_Z]*obj->data[K303B_AXIS_Z] * obj->reso.sensitivity / 1000;

		/**
		mag[obj->cvt.map[K303B_AXIS_X]] = obj->cvt.sign[K303B_AXIS_X]*obj->data[K303B_AXIS_X];
		mag[obj->cvt.map[K303B_AXIS_Y]] = obj->cvt.sign[K303B_AXIS_Y]*obj->data[K303B_AXIS_Y];
		mag[obj->cvt.map[K303B_AXIS_Z]] = obj->cvt.sign[K303B_AXIS_Z]*obj->data[K303B_AXIS_Z];
		**/

		//SENSOR_LOG("map (%d %d %d) / sign (%d %d %d)\n",obj->cvt.map[K303B_AXIS_X],obj->cvt.map[K303B_AXIS_Y],obj->cvt.map[K303B_AXIS_Z],obj->cvt.sign[K303B_AXIS_X],obj->cvt.sign[K303B_AXIS_Y],obj->cvt.sign[K303B_AXIS_Z]);
		//SENSOR_LOG("Mapped msensor data: %d, %d, %d!\n", mag[K303B_AXIS_X], mag[K303B_AXIS_Y], mag[K303B_AXIS_Z]);

		if((Count%1000)==0)
        		SENSOR_DBG("data: %d %d %\nd", mag[K303B_AXIS_X], mag[K303B_AXIS_Y], mag[K303B_AXIS_Z]);

		sprintf(buf, "%04x %04x %04x", mag[K303B_AXIS_X], mag[K303B_AXIS_Y], mag[K303B_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			SENSOR_ERR("msensor data: %s!\n", buf);
		}
	}


	//SENSOR_LOG("io read  msensor data: %d, %d, %d!\n", mag[K303B_AXIS_X],mag[K303B_AXIS_Y],mag[K303B_AXIS_Z]);
	return 0;
}



/*----------------------------------------------------------------------------*/
static int k303b_InitSelfTest(struct i2c_client *client)
{


	return K303B_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int k303b_JudgeTestResult(struct i2c_client *client, s32 prv[K303B_AXES_NUM], s32 nxt[K303B_AXES_NUM])
{

    int res = 0;
    return res;
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
/*---------------------------------  NO -------------------------------------------*/
static ssize_t show_reginfo_value(struct device_driver *ddri, char *buffer)
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

			len += snprintf(buffer+len, PAGE_SIZE, "0x%04X , \t 0x%04X , \t 0x%04X, \t0x%04X ,	 \n  0x%04X , \t  0x%04X, \t0x%04X,  \t 0x%04X ,  \t  \n ",
							buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);


			return len;
}
/*--------------------------------  NO --------------------------------------------*/
static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct k303b_i2c_data *obj = obj_i2c_data;
	int tmp;

	if(NULL == obj)
	{
		SENSOR_ERR("i2c data obj is null!!\n");
		return 0;
	}


	if(1 == sscanf(buf, "%d", &tmp))
	{
		if(atomic_read(&obj->selftest) && !tmp)
		{
			/*enable -> disable*/
			k303b_init_client(obj->client, 0);
		}
		else if(!atomic_read(&obj->selftest) && tmp)
		{
			/*disable -> enable*/
			k303b_InitSelfTest(obj->client);
		}

		SENSOR_LOG("selftest: %d => %d\n", atomic_read(&obj->selftest), tmp);
		atomic_set(&obj->selftest, tmp);
	}
	else
	{
		SENSOR_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
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
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct k303b_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		SENSOR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
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


static ssize_t store_power_status_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int mode = 0;
	int res = 0;
	sscanf(buf, "%d", &mode);
	res = k303b_SetPowerMode(k303b_i2c_client, mode);
	return count;
}



/*----------------------------------------------------------------------------*/
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[K303B_BUFSIZE];
	sprintf(strbuf, "lsm303md");
	return sprintf(buf, "%s", strbuf);
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t show_debug_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct k303b_i2c_data *obj = i2c_get_clientdata(k303b_i2c_client);
	if(NULL == obj)
	{
		SENSOR_ERR("k303b_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", k303bmid_data.controldata[8]);
	return res;
}

static ssize_t store_debug_value(struct device_driver *ddri, const char *buf, size_t count)

{
	struct k303b_i2c_data *obj = i2c_get_clientdata(k303b_i2c_client);
	unsigned int trace;
	if(NULL == obj)
	{
		SENSOR_ERR("k303b_i2c_data is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		//atomic_set(&obj->trace, trace);
		k303bmid_data.controldata[8] = trace;
	}
	else
	{
		SENSOR_ERR("invalid content: '%s', length = %d\n", buf, count);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = k303b_i2c_client;
	struct k303b_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = k303b_i2c_client;
	struct k303b_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			SENSOR_ERR("HWMSEN_GET_CONVERT function ok!\r\n");
			data->hw->direction = layout;
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			SENSOR_ERR("invalid layout: %d, restore to %d, no changed \n", layout, data->hw->direction);

		}
		else
		{
			SENSOR_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
			data->hw->direction = 0;
		}
	}
	else
	{
		SENSOR_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}



/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      		  S_IRUGO, show_daemon_name, 		 NULL);
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(reginfo,              S_IRUGO, show_reginfo_value,       NULL);
static DRIVER_ATTR(debugon,              S_IWUSR | S_IRUGO, show_debug_value,       store_debug_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,    S_IWUSR | S_IRUGO, show_power_status_value,        store_power_status_value);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR,     show_layout_value, store_layout_value );


/*----------------------------------------------------------------------------*/
static struct driver_attribute *k303b_attr_list[] = {
	&driver_attr_daemon,
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_reginfo,         /*self test demo*/
	&driver_attr_debugon,		  /*self test demo*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_layout,
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
static int k303b_ReadCaliData(char *buf, int bufsize)
{
	if((!buf)||(bufsize<=80))
	{
		return -1;
	}

	SENSOR_FUN(f);

	read_lock(&k303bmid_data.datalock);
	sprintf(buf, "%d %d %d %d %d %d %d", k303bmid_data.nmx, k303bmid_data.nmy,
		k303bmid_data.nmz,k303bmid_data.nax,k303bmid_data.nay,k303bmid_data.naz,k303bmid_data.mag_status);
	read_unlock(&k303bmid_data.datalock);
	return 0;
}


/*----------------------------------------------------------------------------*/

static int k303b_ReadRotationVectorData(char *buf, int bufsize)
{
	//SENSOR_FUN(f);
	if((!buf)||(bufsize<=80))
	{
		return -1;
	}

	read_lock(&k303bmid_data.datalock);
	sprintf(buf, "%d %d %d %d", k303bmid_data.RVx, k303bmid_data.RVy,
		k303bmid_data.RVz, k303bmid_data.mag_status);
	read_unlock(&k303bmid_data.datalock);
	return 0;
}

static int k303b_ReadSimuGyroData(char *buf, int bufsize)
{
	//SENSOR_FUN(f);
	if((!buf)||(bufsize<=80))
	{
		return -1;
	}

	read_lock(&k303bmid_data.datalock);
	sprintf(buf, "%d %d %d %d", k303bmid_data.simuGyrox, k303bmid_data.simuGyroy,
		k303bmid_data.simuGyroz, k303bmid_data.mag_status);
	read_unlock(&k303bmid_data.datalock);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int k303b_ReadPostureData(char *buf, int bufsize)
{
	//SENSOR_FUN(f);
	if((!buf)||(bufsize<=80))
	{
		return -1;
	}

	read_lock(&k303bmid_data.datalock);
	sprintf(buf, "%d %d %d %d", k303bmid_data.yaw, k303bmid_data.pitch,
		k303bmid_data.roll, k303bmid_data.mag_status);
	read_unlock(&k303bmid_data.datalock);
	return 0;
}

/*----------------------------------------------------------------------------*/
int k303b_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay, delay_time;
	int status = 0;
	hwm_sensor_data* msensor_data;
	struct k303b_i2c_data *priv = (struct k303b_i2c_data*)self;
	char buff[K303B_BUFSIZE];

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
					sample_delay = K303B_MAG_ODR5;
				}
				else if(value <= 10)
				{
					sample_delay = K303B_MAG_ODR10;
				}
				else
				{
					sample_delay = K303B_MAG_ODR40;
				}

				err = k303b_SetBWRate(priv->client, sample_delay);
				if(err != K303B_SUCCESS ) //0x2C->BW=100Hz
				{
					SENSOR_ERR("Set delay parameter error!\n");
				}

				value = *(int *)buff_in;
				if(value <= 20)
				{
					value = 20;
				}
				
#ifdef K303B_M_NEW_ARCH
                k303b_set_delay(value);
#endif
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
				k303b_SetPowerMode( priv->client, value);
				read_lock(&k303bmid_data.ctrllock);
				if(value == 1)
				{
					k303bmid_data.controldata[7] |= SENSOR_MAGNETIC;
				}
				else
				{
					k303bmid_data.controldata[7] &= ~SENSOR_MAGNETIC;
				}
				wake_up(&open_wq);
				read_unlock(&k303bmid_data.ctrllock);
				if(!(k303bmid_data.controldata[7] & SENSOR_MAGNETIC) && (k303bmid_data.controldata[7] & SENSOR_ORIENTATION)) {
					k303b_SetPowerMode( priv->client, 1);
				}
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

				msensor_data = (hwm_sensor_data *)buff_out;
				read_lock(&k303bmid_data.datalock);
				msensor_data->values[0] = k303bmid_data.nmx;
				msensor_data->values[1] = k303bmid_data.nmy;
				msensor_data->values[2] = k303bmid_data.nmz;
				//status = k303bmid_data.mag_status;
				msensor_data->status = k303bmid_data.mag_status;

//				read_unlock(&k303bmid_data.datalock);

				msensor_data->values[0] = msensor_data->values[0] / 10;
				msensor_data->values[1] = msensor_data->values[1] / 10;
				msensor_data->values[2] = msensor_data->values[2] / 10;
				msensor_data->value_divide = 1;
			/*
				switch (status)
		        {
		            case 1: case 2:
		                msensor_data->status = SENSOR_STATUS_ACCURACY_HIGH;
		                break;
		            case 3:
		                msensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		                break;
		            case 4:
		                msensor_data->status = SENSOR_STATUS_ACCURACY_LOW;
		                break;
		            default:
		                msensor_data->status = SENSOR_STATUS_UNRELIABLE;
		                break;
		        }*/
			//	msensor_data->status = SENSOR_STATUS_ACCURACY_HIGH;
				read_unlock(&k303bmid_data.datalock);

				//SENSOR_LOG("get msensor data: %d, %d, %d, %d!\n", msensor_data->values[0],msensor_data->values[1], msensor_data->values[2], msensor_data->status);
			}
			break;
		default:
			SENSOR_ERR("msensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int k303b_orientation_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay, status=0;
	hwm_sensor_data* osensor_data=NULL;
	struct k303b_i2c_data *priv = (struct k303b_i2c_data*)self;


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
				if(value <= 20)
				{
					sample_delay = 20;
				}
#ifdef K303B_M_NEW_ARCH
                k303b_set_delay(sample_delay);
#endif
			}
			break;

		case SENSOR_ENABLE:
			SENSOR_LOG("k303b orientation SENSOR_ENABLE\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				SENSOR_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				SENSOR_LOG("SENSOR_ENABLE %d\n",value);
				k303b_SetPowerMode(k303b_i2c_client, value);
				read_lock(&k303bmid_data.ctrllock);
				if(value == 1)
				{
					k303bmid_data.controldata[7] |= SENSOR_ORIENTATION;
				}
				else
				{
					k303bmid_data.controldata[7] &= ~SENSOR_ORIENTATION;
				}
				wake_up(&open_wq);
				read_unlock(&k303bmid_data.ctrllock);
				if(!(k303bmid_data.controldata[7] & SENSOR_ORIENTATION) && (k303bmid_data.controldata[7] & SENSOR_MAGNETIC)) {
					k303b_SetPowerMode( k303b_i2c_client, 1);
				}
				// Do nothing
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
				osensor_data = (hwm_sensor_data *)buff_out;
				read_lock(&k303bmid_data.datalock);
				osensor_data->values[0] = k303bmid_data.yaw;
				osensor_data->values[1] = k303bmid_data.pitch;
				osensor_data->values[2] = k303bmid_data.roll;
				//status = k303bmid_data.mag_status;
				osensor_data->status = k303bmid_data.mag_status;
				read_unlock(&k303bmid_data.datalock);

				osensor_data->value_divide = 1;

				//SENSOR_LOG(" get osensor data: %d, %d, %d, %d!\n", osensor_data->values[0],osensor_data->values[1], osensor_data->values[2], osensor_data->status);
			}
/*
			switch (status)
	        {
	            case 1: case 2:
	                osensor_data->status = SENSOR_STATUS_ACCURACY_HIGH;
	                break;
	            case 3:
	                osensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
	                break;
	            case 4:
	                osensor_data->status = SENSOR_STATUS_ACCURACY_LOW;
	                break;
	            default:
	                osensor_data->status = SENSOR_STATUS_UNRELIABLE;
	                break;
	        }*/
			break;
		default:
			SENSOR_ERR("msensor operate function no this parameter %d!\n", command);
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
	atomic_inc(&dev_open_count);

	if(file->private_data == NULL)
	{
		SENSOR_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}


/*----------------------------------------------------------------------------*/
static int k303b_release(struct inode *inode, struct file *file)
{
	struct k303b_i2c_data *obj = i2c_get_clientdata(k303b_i2c_client);
	atomic_dec(&dev_open_count);
	file->private_data = NULL;
	if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		SENSOR_LOG("Release device node\n");
	}
	return 0;
}


static int k303b_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (get_sensors_status(LSM_ALL_SENSORS) != 0));
	return get_sensors_status(LSM_ALL_SENSORS);
}



/*----------------------------------------------------------------------------*/
//static int  k303b_ioctl(struct inode *inode, struct file *file, unsigned int cmd,unsigned long arg)//modified here
static long  k303b_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    void __user *argp = (void __user *)arg;
	int valuebuf[4];
	int YPRdata[4];
	int controlbuf[10];
	char strbuf[K303B_BUFSIZE];
	void __user *data;
	long retval=0;
	int mode=0;
	hwm_sensor_data* osensor_data;
	uint32_t enable;
	char buff[512];
	int status = 0; 				/* for OPEN/CLOSE_STATUS */
	short sensor_status;		/* for Orientation and Msensor status */
	struct k303b_i2c_data *priv = obj_i2c_data;

//	SENSOR_FUN(f);
	switch (cmd)
	{
		case MSENSOR_IOCTL_INIT:
			SENSOR_LOG("===========IOCTL_INIT=======\r\n");
			read_lock(&k303b_data.lock);
			mode =  k303b_data.mode;
			read_unlock(&k303b_data.lock);
			k303b_init_client(k303b_i2c_client, 0);
			break;

		case ECOMPASS_IOC_GET_OFLAG:
			sensor_status = get_sensors_status(SENSOR_ORIENTATION);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				SENSOR_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_MFLAG:
			sensor_status = get_sensors_status(SENSOR_MAGNETIC);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				SENSOR_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_OPEN_STATUS:
			SENSOR_LOG("=========GET__OPEN_STATUS=======\r\n");
			status =  k303b_GetOpenStatus();
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				SENSOR_ERR("copy_to_user failed.");
				return -EFAULT;
			}
			//SENSOR_LOG("===========GET__OPEN_STATU  DONE=======\r\n");
			break;

		case MSENSOR_IOCTL_SET_POSTURE:
			//SENSOR_LOG("===========SET_POSTURE=======\r\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}

			if(copy_from_user(&valuebuf, data, sizeof(valuebuf)))
			{
				retval = -EFAULT;
				goto err_out;
			}

			write_lock(&k303bmid_data.datalock);
			 k303bmid_data.yaw   = valuebuf[0];
			 k303bmid_data.pitch = valuebuf[1];
			 k303bmid_data.roll  = valuebuf[2];
			 k303bmid_data.mag_status = valuebuf[3];
			write_unlock(&k303bmid_data.datalock);
			/* read data count here for at%compass=2 */
			 Count++;
				 if(Count>1000000){
					 Count = 0;
				 }
			/* read data count here for at%compass=2 */

			//SENSOR_LOG("SET_POSTURE osensor data: %d, %d, %d!\n", k303bmid_data.yaw ,k303bmid_data.pitch ,k303bmid_data.roll);
			break;

		case MSENSOR_IOCTL_SET_SIMUGYRO:
			//SENSOR_LOG("===========SET_SIMUGYRO=======\r\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}

			if(copy_from_user(&valuebuf, data, sizeof(valuebuf)))
			{
				retval = -EFAULT;
				goto err_out;
			}

			write_lock(&k303bmid_data.datalock);
			 k303bmid_data.simuGyrox  = valuebuf[0];
			 k303bmid_data.simuGyroy  = valuebuf[1];
			 k303bmid_data.simuGyroz  = valuebuf[2];
			 k303bmid_data.mag_status = valuebuf[3];
			write_unlock(&k303bmid_data.datalock);

			//SENSOR_LOG("SET_SIMUGYRO osensor data: %d, %d, %d!\n", k303bmid_data.simuGyrox ,k303bmid_data.simuGyroy ,k303bmid_data.simuGyroz);
			break;

		case MSENSOR_IOCTL_SET_ROTATION_VECTOR:
			//SENSOR_LOG("===========SET_ROTATION_VECTOR=======\r\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}

			if(copy_from_user(&valuebuf, data, sizeof(valuebuf)))
			{
				retval = -EFAULT;
				goto err_out;
			}

			write_lock(&k303bmid_data.datalock);
			 k303bmid_data.RVx = valuebuf[0];
			 k303bmid_data.RVy = valuebuf[1];
			 k303bmid_data.RVz = valuebuf[2];
			 k303bmid_data.mag_status = valuebuf[3];
			write_unlock(&k303bmid_data.datalock);

			//SENSOR_LOG("SET_ROTATION_VECTOR rvsensor data: %d, %d, %d!\n", k303bmid_data.RVx,k303bmid_data.RVy,k303bmid_data.RVz);
			break;
        case MSENSOR_IOCTL_READ_SIMUGYRODATA:
			//SENSOR_LOG("===========IOCTL_READ_READ_SIMUGYRODATA=======\r\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}

			k303b_ReadSimuGyroData(strbuf, K303B_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				retval = -EFAULT;
				goto err_out;
			}
			break;
        case MSENSOR_IOCTL_READ_ROTATION_VECTOR_DATA:
			//SENSOR_LOG("===========IOCTL_READ_READ_ROTATION_VECTOR_DATA=======\r\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}

			k303b_ReadRotationVectorData(strbuf, K303B_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				retval = -EFAULT;
				goto err_out;
			}
			break;
		case  MSENSOR_IOCTL_SET_CALIDATA:
			//SENSOR_LOG("===========IOCTL_SET_YPRdata=======\r\n");
			data = (void __user *) arg;
			if (data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}
			if(copy_from_user(&YPRdata, data, sizeof(YPRdata)))
			{
				retval = -EFAULT;
				goto err_out;
			}

			write_lock(&k303bmid_data.datalock);
			 k303bmid_data.nmx   = YPRdata[0];
			 k303bmid_data.nmy = YPRdata[1];
			 k303bmid_data.nmz  = YPRdata[2];
			 k303bmid_data.mag_status = YPRdata[3];
			write_unlock(&k303bmid_data.datalock);
			//SENSOR_LOG("IOCTL_SET_YPRdata msensor data: %d, %d, %d!\n", k303bmid_data.nmx ,k303bmid_data.nmy ,k303bmid_data.nmz);
			break;

		case MSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}

			 k303b_ReadChipInfo(k303b_i2c_client, strbuf, K303B_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case MSENSOR_IOCTL_SENSOR_ENABLE:
			//SENSOR_LOG("===========IOCTL_SENSOR_ENABLE=======\r\n");

			data = (void __user *) arg;
			if (data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}
			if(copy_from_user(&enable, data, sizeof(enable)))
			{
				SENSOR_ERR("copy_from_user failed.");
				return -EFAULT;
			}
			else
			{
			    SENSOR_LOG( "MSENSOR_IOCTL_SENSOR_ENABLE enable = %d\n",enable);
				k303b_SetPowerMode(k303b_i2c_client, enable);
				read_lock(&k303bmid_data.ctrllock);
				if(enable == 1)
				{
					k303bmid_data.controldata[7] |= SENSOR_ORIENTATION;
				}
				else
				{
					Count = 0;
					k303bmid_data.controldata[7] &= ~SENSOR_ORIENTATION;
				}
				wake_up(&open_wq);
				read_unlock(&k303bmid_data.ctrllock);

			}

			break;

		case MSENSOR_IOCTL_READ_SENSORDATA:
			//SENSOR_LOG("===========IOCTL_READ_SENSORDATA=======\r\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}
			 k303b_ReadSensorData(k303b_i2c_client, strbuf, K303B_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				retval = -EFAULT;
				goto err_out;
			}

			break;

		case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
			//SENSOR_LOG("===========IOCTL_READ_FACTORY_SENSORDATA=======\r\n");
			data = (void __user *) arg;
			if (data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}

			osensor_data = (hwm_sensor_data *)buff;

			read_lock(&k303bmid_data.datalock);
			osensor_data->values[0] =  k303bmid_data.yaw;
			osensor_data->values[1] =  k303bmid_data.pitch;
			osensor_data->values[2] =  k303bmid_data.roll;
			//status =  k303bmid_data.mag_status;
			read_unlock(&k303bmid_data.datalock);

			osensor_data->value_divide = 1;

			switch (k303bmid_data.mag_status)
		    {
		            case 1: case 2:
		                osensor_data->status = SENSOR_STATUS_ACCURACY_HIGH;
		                break;
		            case 3:
		                osensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		                break;
		            case 4:
		                osensor_data->status = SENSOR_STATUS_ACCURACY_LOW;
		                break;
		            default:
		                osensor_data->status = SENSOR_STATUS_UNRELIABLE;
		                break;
		    }

            sprintf(buff, "%d/%d/%d/%d",Count,osensor_data->values[0], osensor_data->values[1],
				osensor_data->values[2]);
			if(copy_to_user(data, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}

			break;

		case MSENSOR_IOCTL_READ_POSTUREDATA:
			//SENSOR_LOG("===========IOCTL_READ_READ_POSTUREDATA=======\r\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				SENSOR_ERR("IO parameter pointer is NULL!\r\n");
				break;
			}

			k303b_ReadPostureData(strbuf, K303B_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case MSENSOR_IOCTL_READ_CALIDATA:
			//SENSOR_LOG("===========IOCTL_READ_READ_CALIDATA=======\r\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				break;
			}
			 k303b_ReadCaliData(strbuf, K303B_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case MSENSOR_IOCTL_READ_CONTROL :
			read_lock(&k303bmid_data.ctrllock);
			memcpy(controlbuf, &k303bmid_data.controldata[0], sizeof(controlbuf));

            read_unlock(&k303bmid_data.ctrllock);
			data = (void __user *) arg;
			if(data == NULL)
			{
				break;
			}
			if(copy_to_user(data, controlbuf, sizeof(controlbuf)))
			{
				retval = -EFAULT;
				goto err_out;
			}
			break;

		case MSENSOR_IOCTL_SET_CONTROL:
			//SENSOR_LOG("===========IOCTL_SET_CONTROL=======\r\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				break;
			}
			if(copy_from_user(controlbuf, data, sizeof(controlbuf)))
			{
				retval = -EFAULT;
				goto err_out;
			}
			write_lock(&k303bmid_data.ctrllock);
			memcpy(&k303bmid_data.controldata[0], controlbuf, sizeof(controlbuf));
			write_unlock(&k303bmid_data.ctrllock);
			break;

		case MSENSOR_IOCTL_SET_MODE:
			break;

		default:
			SENSOR_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			retval = -ENOIOCTLCMD;
			break;
		}

	err_out:
	return retval;
}

int k303b_simugyro_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay, status=0;
	hwm_sensor_data* gysensor_data=NULL;
	struct k303b_i2c_data *priv = (struct k303b_i2c_data*)self;

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
				if(value <= 20)
				{
					sample_delay = 20;
				}


				k303bmid_data.controldata[0] = sample_delay;  // Loop Delay
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
				SENSOR_LOG("simugyro enable value = %d\n",value);
				k303b_SetPowerMode(k303b_i2c_client, value);
				read_lock(&k303bmid_data.ctrllock);
				if(value == 1)
				{
					k303bmid_data.controldata[7] |= SENSOR_GYROSCOPE;
				}
				else
				{
					k303bmid_data.controldata[7] &= ~SENSOR_GYROSCOPE;
				}
				wake_up(&open_wq);
				read_unlock(&k303bmid_data.ctrllock);
			}
			break;

		case SENSOR_GET_DATA:
			//SENSOR_LOG("+++++++++++MSENSOR_GET_ORIENTATION_DATA");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				SENSOR_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gysensor_data = (hwm_sensor_data *)buff_out;
				read_lock(&k303bmid_data.datalock);
				gysensor_data->values[0] = k303bmid_data.simuGyrox;
				gysensor_data->values[1] = k303bmid_data.simuGyroy;
				gysensor_data->values[2] = k303bmid_data.simuGyroz;
				status = k303bmid_data.mag_status;
				gysensor_data->value_divide = 1000;
				read_unlock(&k303bmid_data.datalock);
				//SENSOR_LOG(" get gysensor data: %d, %d, %d, %d!\n", gysensor_data->values[0],gysensor_data->values[1], gysensor_data->values[2], gysensor_data->status);
			}

			switch (status)
	        {
	            case 1: case 2:
	                gysensor_data->status = SENSOR_STATUS_ACCURACY_HIGH;
	                break;
	            case 3:
	                gysensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
	                break;
	            case 4:
	                gysensor_data->status = SENSOR_STATUS_ACCURACY_LOW;
	                break;
	            default:
	                gysensor_data->status = SENSOR_STATUS_UNRELIABLE;
	                break;
	        }
			break;
		default:
			SENSOR_ERR("gysensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}


int k303b_rotation_vector_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay, status=0;
	hwm_sensor_data* rvsensor_data=NULL;
	struct k303b_i2c_data *priv = (struct k303b_i2c_data*)self;
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
				if(value <= 20)
				{
					sample_delay = 20;
				}


				k303bmid_data.controldata[0] = sample_delay;  // Loop Delay
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
				SENSOR_LOG("rotation vector enable value = %d\n",value);
				k303b_SetPowerMode(k303b_i2c_client, value);
				read_lock(&k303bmid_data.ctrllock);
				if(value == 1)
				{
					k303bmid_data.controldata[7] |= SENSOR_ROTATION_VECTOR;
				}
				else
				{
					k303bmid_data.controldata[7] &= ~SENSOR_ROTATION_VECTOR;
				}
				wake_up(&open_wq);
				read_unlock(&k303bmid_data.ctrllock);
				// Do nothing
			}
			break;

		case SENSOR_GET_DATA:
			//SENSOR_LOG("rotation vector SENSOR_GET_DATA\n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				SENSOR_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				rvsensor_data = (hwm_sensor_data *)buff_out;
				read_lock(&k303bmid_data.datalock);
				rvsensor_data->values[0] = k303bmid_data.RVx;
				rvsensor_data->values[1] = k303bmid_data.RVy;
				rvsensor_data->values[2] = k303bmid_data.RVz;
				status = k303bmid_data.mag_status;
				rvsensor_data->value_divide = 10000;
				read_unlock(&k303bmid_data.datalock);

				//SENSOR_LOG(" get rv sensor data: %d, %d, %d, %d!\n", rvsensor_data->values[0],rvsensor_data->values[1], rvsensor_data->values[2], rvsensor_data->status);
			}

			switch (status)
	        {
	            case 1: case 2:
	                rvsensor_data->status = SENSOR_STATUS_ACCURACY_HIGH;
	                break;
	            case 3:
	                rvsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
	                break;
	            case 4:
	                rvsensor_data->status = SENSOR_STATUS_ACCURACY_LOW;
	                break;
	            default:
	                rvsensor_data->status = SENSOR_STATUS_UNRELIABLE;
	                break;
	        }
			break;
		default:
			SENSOR_ERR("rvsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
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
	.name = "msensor",
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
		/**
		if((err = k303b_SetPowerMode(obj->client, false)))
		{
			SENSOR_ERR("write power control fail!!\n");
			return err;
		}
		***/
		k303b_power(obj->hw, 0);
		SENSOR_LOG("k303b_suspend ok\n");
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int k303b_resume(struct i2c_client *client)
{
	struct k303b_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	SENSOR_FUN();

	if(obj == NULL)
	{
		SENSOR_ERR("null pointer!!\n");
		return -EINVAL;
	}

	k303b_power(obj->hw, 1);
/***
	if((err = k303b_SetPowerMode(obj->client, true)))
	{
		SENSOR_ERR("write power control fail!!\n");
		return err;
	}

	***/
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
#if 1
	if(get_sensors_status(LSM_ALL_SENSORS) &&k303b_SetPowerMode(obj->client, false)!=0 )
	{
		SENSOR_ERR("write power control fail!!\n");
		return;
	}
#endif
	sensor_power = false;
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

#if 1
	if(get_sensors_status(LSM_ALL_SENSORS) && (k303b_SetPowerMode(obj->client, true))!=0)
	{
		SENSOR_ERR("write power control fail!!\n");
		return;
	}
#endif
    atomic_set(&obj->suspend, 0);
	SENSOR_LOG("k303b_late_resume ok\n");
}


static int k303b_m_getdata(int *x,int *y,int *z,int *state)
{
	int status = 0,ret = 0;

	read_lock(&k303bmid_data.datalock);
	*x = k303bmid_data.nmx / 10;
	*y = k303bmid_data.nmy / 10;
	*z = k303bmid_data.nmz / 10;
	status = k303bmid_data.mag_status;
	read_unlock(&k303bmid_data.datalock);

	switch (status)
    {
        case 1: case 2:
            status = SENSOR_STATUS_ACCURACY_HIGH;
            break;
        case 3:
            status = SENSOR_STATUS_ACCURACY_MEDIUM;
            break;
        case 4:
            status = SENSOR_STATUS_ACCURACY_LOW;
            break;
        default:
            status = SENSOR_STATUS_UNRELIABLE;
            break;
    }

	*state = status;
	return 0;
}
static int k303b_o_getdata(int *x,int *y,int *z,int *state)
{
	read_lock(&k303bmid_data.datalock);
	*x = k303bmid_data.yaw;
	*y = k303bmid_data.pitch;
	*z = k303bmid_data.roll;
	*state = k303bmid_data.mag_status;
	read_unlock(&k303bmid_data.datalock);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int k303b_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
		SENSOR_FUN();
		struct i2c_client *new_client;
		struct k303b_i2c_data *data;
		int err = 0;

#ifdef K303B_M_NEW_ARCH
        struct mag_drv_obj sobj_m, sobj_o;
        struct mag_control_path ctl={0};
        struct mag_data_path mag_data={0};
#else
        struct hwmsen_object sobj_m, sobj_o;
#endif
		struct hwmsen_object sobj_gy, sobj_rv;

#if defined(CONFIG_MTK_AUTO_DETECT_MAGNETOMETER)
	if(k303b_init_flag==0)
	{
		SENSOR_LOG("Magnetometer Sensor already probed...just skip\n");
		return err;
	}
#endif
		client->addr = K303B_I2C_ADDRESS>>1;

		if (!(data = kmalloc(sizeof(struct k303b_i2c_data), GFP_KERNEL)))
		{
			err = -ENOMEM;
			goto exit;
		}
		memset(data, 0, sizeof(struct k303b_i2c_data));

		data->hw = get_cust_mag_hw();
		if((err = hwmsen_get_convert(data->hw->direction, &data->cvt)))
		{
			SENSOR_ERR("invalid direction: %d\n", data->hw->direction);
			goto exit;
		}

		atomic_set(&data->layout, data->hw->direction);
		atomic_set(&data->trace, 0);
		init_waitqueue_head(&data_ready_wq);
		init_waitqueue_head(&open_wq);

		data->client = client;
		new_client = data->client;
		i2c_set_clientdata(new_client, data);

		k303b_i2c_client = new_client;
		obj_i2c_data = data;

		if((err = k303b_init_client(new_client, 1)))
		{
			goto exit_init_failed;
		}

		/* Register sysfs attribute */
/*#ifdef K303B_M_NEW_ARCH
        if((err = k303b_create_attr(&(k303b_init_info.platform_diver_addr->driver))))
#else*/
#if defined(CONFIG_MTK_AUTO_DETECT_MAGNETOMETER)
		if(err = k303b_create_attr(&(msensor_init_info.platform_diver_addr->driver)))
		{
			SENSOR_ERR("create attribute err = %d\n", err);
			goto exit_sysfs_create_group_failed;
		}
#else
		if((err = k303b_create_attr(&k303b_msensor_driver.driver)))
//#endif
		{
			SENSOR_ERR("create attribute err = %d\n", err);
			goto exit_sysfs_create_group_failed;
		}
#endif

		if((err = misc_register(&k303b_device)))
		{
			SENSOR_ERR("k303b_device register failed\n");
			goto exit_misc_device_register_failed;
		}

		sobj_m.self = data;
		sobj_m.polling = 1;
#ifdef K303B_M_NEW_ARCH
	sobj_m.mag_operate = k303b_operate;
    if((err = mag_attach(ID_M_V_MAGNETIC, &sobj_m)))
#else
	sobj_m.sensor_operate = k303b_operate;
	if(err = hwmsen_attach(ID_MAGNETIC, &sobj_m))
#endif
		{
			SENSOR_ERR("attach fail = %d\n", err);
			goto exit_kfree;
		}

		sobj_o.self = data;
		sobj_o.polling = 1;
#ifdef K303B_M_NEW_ARCH
        sobj_o.mag_operate = k303b_orientation_operate;
        if((err = mag_attach(ID_M_V_ORIENTATION, &sobj_o)))
#else
        sobj_o.sensor_operate = k303b_orientation_operate;
        if(err = hwmsen_attach(ID_ORIENTATION, &sobj_o))
#endif
		{
			SENSOR_ERR("attach fail = %d\n", err);
			goto exit_kfree;
		}
#if 0
		sobj_gy.self = data;
		sobj_gy.polling = 1;
		sobj_gy.sensor_operate = k303b_simugyro_operate;
		if((err = hwmsen_attach(ID_GYROSCOPE, &sobj_gy)))
		{
			SENSOR_ERR("attach fail = %d\n", err);
			goto exit_kfree;
		}

		sobj_rv.self = data;
		sobj_rv.polling = 1;
		sobj_rv.sensor_operate = k303b_rotation_vector_operate;
		if((err = hwmsen_attach(ID_ROTATION_VECTOR, &sobj_rv)))
		{
			SENSOR_ERR("attach fail = %d\n", err);
			goto exit_kfree;
		}
#endif

#ifdef K303B_M_NEW_ARCH
    #define ORIENTATION_ACCURACY_RATE       1

	ctl.m_enable = k303b_enable;
	ctl.m_set_delay  = k303b_set_delay;
	ctl.m_open_report_data = k303b_open_report_data;

    ctl.o_enable = k303b_o_enable;
	ctl.o_set_delay  = k303b_o_set_delay;
	ctl.o_open_report_data = k303b_o_open_report_data;
	ctl.is_report_input_direct = false;

	err = mag_register_control_path(&ctl);
	if(err)
	{
	 	MAG_ERR("register mag control path err\n");
		goto exit_kfree;
	}

	mag_data.div_m = 1;
	mag_data.div_o = ORIENTATION_ACCURACY_RATE;

	mag_data.get_data_m = k303b_m_getdata;
	mag_data.get_data_o = k303b_o_getdata;
//	mag_data.get_raw_data = NULL;//k303b_getraw;

	err = mag_register_data_path(&mag_data);
	if(err)
	{
	 	MAG_ERR("register data control path err\n");
		goto exit_kfree;
	}
#endif

#if CONFIG_HAS_EARLYSUSPEND
		data->early_drv.level	 = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
		data->early_drv.suspend  = k303b_early_suspend,
		data->early_drv.resume	 = k303b_late_resume,
		register_early_suspend(&data->early_drv);
#endif


#ifdef K303B_M_NEW_ARCH
        k303b_init_flag = 1;
#endif

		SENSOR_LOG("OK\n");
#if defined(CONFIG_MTK_AUTO_DETECT_MAGNETOMETER)
		k303b_init_flag=0; 
#endif
		return 0;

		exit_sysfs_create_group_failed:
		exit_init_failed:
		//i2c_detach_client(new_client);
		exit_misc_device_register_failed:
		exit_kfree:
		kfree(data);
		exit:
		SENSOR_ERR("err = %d\n", err);
#if defined(CONFIG_MTK_AUTO_DETECT_MAGNETOMETER)
		k303b_init_flag=-1; 
#endif
		return err;

}



/*----------------------------------------------------------------------------*/
static int k303b_i2c_remove(struct i2c_client *client)
{
	int err = 0;

/*#ifdef K303B_M_NEW_ARCH
	if((err = k303b_delete_attr(&(k303b_init_info.platform_diver_addr->driver))))
#else*/
#if defined(CONFIG_MTK_AUTO_DETECT_MAGNETOMETER)
	if(err = k303b_delete_attr(&(msensor_init_info.platform_diver_addr->driver)))
	{
		SENSOR_ERR("k303b_delete_attr fail: %d\n", err);
	}
#else
	if((err = k303b_delete_attr(&k303b_msensor_driver.driver)))
//#endif
	{
		SENSOR_ERR("k303b_delete_attr fail: %d\n", err);
	}
#endif

	if((err = misc_deregister(&k303b_device)))
	{
		SENSOR_ERR("misc_deregister fail: %d\n", err);
	}

	if((err = hwmsen_detach(ID_MAGNETIC)))
	{
		SENSOR_ERR("hwmsen_detach fail: %d\n", err);
	}
	if((err = hwmsen_detach(ID_ORIENTATION)))
	{
		SENSOR_ERR("hwmsen_detach fail: %d\n", err);
	}


	k303b_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}


#ifndef K303B_M_NEW_ARCH
/*----------------------------------------------------------------------------*/
static int k303b_probe(struct platform_device *pdev)
{
	struct mag_hw *hw = get_cust_mag_hw();
	SENSOR_FUN();

	k303b_power(hw, 1);
	rwlock_init(&k303bmid_data.ctrllock);
	rwlock_init(&k303bmid_data.datalock);
	rwlock_init(&k303b_data.lock);
	memset(&k303bmid_data.controldata[0], 0, sizeof(int)*10);

	atomic_set(&dev_open_count, 0);

	//k303b_force[0] = hw->i2c_num;//modified
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
    struct mag_hw *hw = get_cust_mag_hw();

	SENSOR_FUN();
	k303b_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&k303b_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver k303b_msensor_driver = {
	.probe      = k303b_probe,
	.remove     = k303b_remove,
	.driver     = {
	.name  = "msensor",
	}
};
#endif

#if defined(CONFIG_MTK_AUTO_DETECT_MAGNETOMETER)
static int  k303b_local_init(void)
{
   struct mag_hw *hw = get_cust_mag_hw();
 	SENSOR_FUN();
	k303b_power(hw, 1);
	rwlock_init(&k303bmid_data.ctrllock);
	rwlock_init(&k303bmid_data.datalock);
	rwlock_init(&k303b_data.lock);
	memset(&k303bmid_data.controldata[0], 0, sizeof(int)*10);

	atomic_set(&dev_open_count, 0);

	if(i2c_add_driver(&k303b_i2c_driver))
	{
		SENSOR_ERR("add driver error\n");
		return -1;
	}
	if(-1 == k303b_init_flag)
	{
	   SENSOR_ERR("k303b_local_init add driver error\n");
	   return -1;
	}

	return 0;
}

static int  k303b_local_remove(void)
{
    struct mag_hw *hw = get_cust_mag_hw();
    SENSOR_FUN();
	k303b_power(hw, 0);
	atomic_set(&dev_open_count, 0);
    i2c_del_driver(&k303b_i2c_driver);
    return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int __init k303b_init(void)
{
	SENSOR_FUN();
	struct mag_hw *hw = get_cust_mag_hw();
	
	i2c_register_board_info(hw->i2c_num, &i2c_k303b, 1);
/*#ifdef k303b_M_NEW_ARCH
	mag_driver_add(&k303b_init_info);
#else*/
#if defined(CONFIG_MTK_AUTO_DETECT_MAGNETOMETER)
	hwmsen_msensor_add(&msensor_init_info);
#else
    if(platform_driver_register(&k303b_msensor_driver))
	{
		SENSOR_ERR("failed to register driver");
		return -ENODEV;
	}
#endif
	SENSOR_LOG("init OK / i2c_number = %d\n",hw->i2c_num);
	return 0;
//#endif
}
/*----------------------------------------------------------------------------*/
static void __exit k303b_exit(void)
{
	SENSOR_FUN();
#ifndef K303B_M_NEW_ARCH
	platform_driver_unregister(&k303b_msensor_driver);
#endif
}

/*----------------------------------------------------------------------------*/
module_init(k303b_init);
module_exit(k303b_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Ruo Liang");
MODULE_DESCRIPTION("K303B MI-Sensor driver without DRDY");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.1");
