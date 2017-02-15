/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file lm3639.c
   brief This file contains all function implementations for the lm3639 in linux
   this source file refer to MT6572 platform
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/kernel.h>
#include <linux/delay.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/mt_pm_ldo.h>

#include <linux/platform_device.h>
#include <cust_acc.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>

#include <cust_leds_def.h> //MTK Delta Package2 porting for G3S & L80+

#include <linux/leds.h>
#include <mach/board_lge.h>


#define LCD_LED_MAX 0x7F
#define LCD_LED_MIN 0

#define DEFAULT_BRIGHTNESS 0x73 //for 20mA

#if defined(TARGET_MT6582_L80)//G3S, L80+ source sync with KK
#define LM3639_MIN_VALUE_SETTINGS 10 /* value leds_brightness_set*/
#elif defined(TARGET_MT6582_B2L)
#define LM3639_MIN_VALUE_SETTINGS 20 /* value leds_brightness_set*/
#else
#define LM3639_MIN_VALUE_SETTINGS 10 /* value leds_brightness_set*/
#endif
#define LM3639_MAX_VALUE_SETTINGS 255 /* value leds_brightness_set*/
#define MIN_MAX_SCALE(x) (((x)<LM3639_MIN_VALUE_SETTINGS) ? LM3639_MIN_VALUE_SETTINGS : (((x)>LM3639_MAX_VALUE_SETTINGS) ? LM3639_MAX_VALUE_SETTINGS:(x)))

#define BACKLIHGT_NAME "charge-pump"

#define Lm3639_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define LM3639_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

#define LM3639_DEV_NAME "charge-pump"

#define CPD_TAG                  "[ChargePump] "
#define CPD_FUN(f)               printk(CPD_TAG"%s\n", __FUNCTION__)
#define CPD_ERR(fmt, args...)    printk(CPD_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define CPD_LOG(fmt, args...)    printk(CPD_TAG fmt, ##args)

// I2C variable
static struct i2c_client *new_client = NULL;
static const struct i2c_device_id lm3639_i2c_id[] = {{LM3639_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_lm3639={ I2C_BOARD_INFO("charge-pump", 0x39)};

// Flash control
unsigned char strobe_ctrl;
unsigned char flash_ctrl=0; //[LGE_UPDATE][yonghwan.lym@lge.com][2014-07-12] flash_en register(0x0A) setting position change.
unsigned char lm3639_mapping_level;
// Gamma 2.2 Table
//G3S, L80+ source sync with KK (G3S already present, L80 was using else case)
#if defined(TARGET_MT6582_L80) || defined (TARGET_Y50)
unsigned char bright_arr[] = {
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, // 19
    11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 18, 18, // 39
    18, 19, 20, 20, 20, 21, 22, 22, 23, 23, 23, 23, 24, 24, 25, 25, 27, 28, 29, 30, // 59
    31, 31, 32, 33, 34, 34, 35, 36, 37, 38, 39, 40, 40, 41, 42, 42, 43, 44, 45, 45, // 79
    48, 50, 51, 51, 53, 55, 56, 56, 58, 60, 61, 63, 65, 65, 66, 68, 70, 71, 73, 73, // 99
    75, 76, 78, 78, 78, 81, 83, 87, 89, 91, 91, 94, 96, 98, 101, 102, 103, 105, 108, 110, // 119
    110, 112, 115, 117, 119, 122, 124, 127, 127, // 128
    };
#elif defined (TARGET_Y70)
unsigned char bright_arr[] = {
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, // 19
    11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 18, 18, // 39
    18, 19, 20, 20, 20, 21, 22, 22, 23, 23, 23, 23, 24, 24, 25, 25, 27, 28, 29, 30, // 59
    31, 31, 32, 33, 34, 34, 35, 36, 37, 38, 39, 40, 40, 41, 42, 42, 43, 44, 45, 45, // 79
    48, 50, 51, 51, 53, 55, 56, 56, 58, 60, 61, 63, 65, 65, 66, 68, 70, 71, 73, 73, // 99
    75, 76, 78, 78, 78, 81, 83, 87, 89, 91, 91, 94, 96, 98, 101, 102, 103, 105, 108, 110, // 119
    110, 112, 115, 117, 119, 122, 124, 127, 127, // 128
    };
#elif defined (TARGET_MT6582_Y90)
unsigned char bright_arr[] = {  // array index max 100, value under 255
#if 1 // 1230
	3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6,  // 19
	6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15,  // 39
	16, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34,  // 59
	35, 37, 39, 40, 42, 43, 45, 46, 49, 50, 52, 53, 55, 56, 58, 59, 60, 61, 62, 64,  //79
	68, 70, 72, 73, 74, 77, 80, 83, 86, 89, 91, 94, 96, 98, 100, 102, 104, 106, 108, 110, 113 //106, 108, 110,112, 114  // 100
#else
        10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 16, 16, 17,  // 19
        17, 18, 18, 19, 19, 20, 21, 22, 22, 23, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,  // 39
        33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 53,  // 59
        54, 56, 57, 59, 60, 62, 64, 65, 67, 69, 71, 73, 75, 77, 79, 80, 82, 84, 86, 88,  //79
        90, 91, 93, 94, 96, 97, 99, 100, 102, 103, 105, 106, 108, 109, 111, 112, 114, 115, 117, 119, 120 //106, 108, 110,112, 114  // 100
#endif
    };
#elif defined(TARGET_MT6582_B2L)
unsigned char bright_arr[] = {
	1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  4,  4,  4,  5,  6, //19
	6,  7,  7,  8,  8,  9,  9,  10, 10, 10, 11, 11, 12, 13, 14, 14, 15, 16, 16, 18,//39
	18, 18, 18, 20, 21, 22, 23, 24, 25, 26, 27, 29, 30, 31, 32, 33, 35, 36, 37, 37,//59
	38, 41, 42, 45, 45, 47, 48, 49, 51, 52, 54, 54, 57, 57, 59, 62, 64, 65, 67, 69,//79
	71, 73, 76, 80, 80, 82, 82, 86, 89, 91, 93, 93, 95, 97, 100, 102, 102, 104, 106, 114, 114 //100
	};
#else
unsigned char bright_arr[] = {
	10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12,
	13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 19, 19, 20, 21, 21, 22, 23, 24,
	24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 38, 39, 40, 41, 43, 44, 45,
	47, 48, 49, 51, 52, 54, 55, 57, 59, 60, 62, 64, 65, 67, 69, 71, 73, 74, 76, 78,
	80, 82, 84, 86, 89, 91, 93, 95, 97, 100, 102, 104, 106, 109, 111, 114, 116, 119, 121, 124, 127
	};
#endif
static unsigned char current_brightness = 0;
static unsigned char is_suspend = 0;
static bool lm3639_power_on = 1; //LGE_CHANGE: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.

struct semaphore lm3639_lock; //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.

/* generic */
#define LM3639_MAX_RETRY_I2C_XFER (100)
#define LM3639_I2C_WRITE_DELAY_TIME 1

typedef struct
{
    kal_bool       bat_exist;
    kal_bool       bat_full;
    kal_bool       bat_low;
    UINT32      bat_charging_state;
    UINT32      bat_vol;
    kal_bool     charger_exist;
    UINT32      pre_charging_current;
    UINT32      charging_current;
    INT32      charger_vol;
    UINT32       charger_protect_status;
    UINT32      ISENSE;
    UINT32      ICharging;
    INT32       temperature;
    UINT32      total_charging_time;
    UINT32      PRE_charging_time;
    UINT32      CC_charging_time;
    UINT32      TOPOFF_charging_time;
    UINT32      POSTFULL_charging_time;
    UINT32       charger_type;
    UINT32       PWR_SRC;
    UINT32       SOC;
    UINT32       ADC_BAT_SENSE;
    UINT32       ADC_I_SENSE;
} PMU_ChargerStruct;

extern PMU_ChargerStruct BMT_status;

/* i2c read routine for API*/
static char lm3639_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMA_USE_BASIC_I2C_FUNC
	s32 dummy;
	if (NULL == client)
		return -1;

	while (0 != len--) {
#ifdef BMA_SMBUS
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			CPD_ERR("i2c bus read error");
			return -1;
		}
		*data = (u8)(dummy & 0xff);
#else
		dummy = i2c_master_send(client, (char *)&reg_addr, 1);
		if (dummy < 0)
		{
            printk("send dummy is %d", dummy);
			return -1;
		}

		dummy = i2c_master_recv(client, (char *)data, 1);
		if (dummy < 0)
		{
            printk("recv dummy is %d", dummy);
			return -1;
		}
#endif
		reg_addr++;
		data++;
	}
	return 0;
#else
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < LM3639_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			mdelay(LM3639_I2C_WRITE_DELAY_TIME);
	}

	if (LM3639_MAX_RETRY_I2C_XFER <= retry) {
		CPD_ERR("I2C xfer error");
		return -EIO;
	}

	return 0;
#endif
}

/* i2c write routine for */
static char lm3639_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
#if !defined BMA_USE_BASIC_I2C_FUNC
	s32 dummy;

	#ifndef BMA_SMBUS
	u8 buffer[2];
	#endif

	if (NULL == client)
		return -1;

	while (0 != len--) {
		#if 1
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
		#else
		buffer[0] = reg_addr;
		buffer[1] = *data;
		dummy = i2c_master_send(client, (char *)buffer, 2);
		#endif

		reg_addr++;
		data++;
		if (dummy < 0) {
			return -1;
		}
	}

#else
	u8 buffer[2];
	int retry;
	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buffer,
		 },
	};

	while (0 != len--) {
		buffer[0] = reg_addr;
		buffer[1] = *data;
		for (retry = 0; retry < LM3639_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, msg,
						ARRAY_SIZE(msg)) > 0) {
				break;
			} else {
				mdelay(LM3639_I2C_WRITE_DELAY_TIME);
			}
		}
		if (LM3639_MAX_RETRY_I2C_XFER <= retry) {
			return -EIO;
		}
		reg_addr++;
		data++;
	}
#endif
	printk("\n [LM3639] lm3639_i2c_write \n");
	return 0;
}

static int lm3639_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	return lm3639_i2c_read(client,reg_addr,data,1);
}

static int lm3639_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	int ret_val = 0;
	int i = 0;

	ret_val = lm3639_i2c_write(client,reg_addr,data,1);

	for ( i = 0; i < 5; i++)
	{
		if (ret_val != 0)
			lm3639_i2c_write(client,reg_addr,data,1);
		else
			return ret_val;
	}
	return ret_val;
}

static int lm3639_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	return lm3639_i2c_read(client,reg_addr,data,len);
}

bool check_charger_pump_vendor()
{
    int err = 0;
    unsigned char data = 0;

    err = lm3639_smbus_read_byte(new_client,0x01,&data);

    if(err < 0)
        printk(KERN_ERR "%s read charge-pump vendor id fail\n", __func__);

    printk(KERN_INFO "%s vendor is 0x%x\n", __func__, data&0x03);

    if((data&0x03) == 0x03) //Richtek
        return FALSE;
    else
        return TRUE;
}
int current_level;
int chargepump_set_backlight_level(unsigned int level)
{
	unsigned char data = 0;
	unsigned char data1 = 0;

	unsigned int bright_per = 0;

	printk("\n[LM3639] chargepump_set_backlight_level  [%d]\n",level);

	if (level == 0)
	{
		if(is_suspend == 0)
		{
			printk( "[LCD] Backlight Off\n"); //add log
			printk( "[LM3639] backlight off\n");
			down_interruptible(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
			data1 = 0x00; //backlight2 brightness 0
			lm3639_smbus_write_byte(new_client, 0x05, &data1);
			lm3639_smbus_read_byte(new_client, 0x0A, &data1);
			data1 &= 0x66;
			lm3639_smbus_write_byte(new_client, 0x0A, &data1);
			//mt_set_gpio_out(GPIO_LCM_BL_EN,GPIO_OUT_ZERO);

			is_suspend = 1;//[LGE_UPDATE][woonghwan.lee@lge.com][2014-07-14] Move backlight suspend setting position into semaphore

			up(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
			//LGE_CHANGE_S: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
			if(flash_ctrl == 0){
				mt_set_gpio_out(GPIO_LCM_BL_EN,GPIO_OUT_ZERO); //[LGE_UPDATE][woonghwan.lee@lge.com][2014-07-14] Move backlight suspend setting position into semaphore
				lm3639_power_on = 0;
			}
			//LGE_CHANGE_E: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
			//is_suspend = 1;
		}
	}
	else
	{
		level = MIN_MAX_SCALE(level);
		printk(KERN_ERR "[LM3639] level = %d", level);

		//Gamma 2.2 Table adapted
		//G3S, L80+ source sync with KK
		#if defined(TARGET_MT6582_L80)
		bright_per = level / 2;
		#elif defined(TARGET_MT6582_B2L)
		bright_per = (level - (unsigned int)20) *(unsigned int)100 / (unsigned int)235;
		#else
#if 1 //defined(TARGET_S7) || defined (TARGET_Y50) || defined (TARGET_Y70) || defined (TARGET_Y90)
		bright_per = (level - (unsigned int)LM3639_MIN_VALUE_SETTINGS) *(unsigned int)100 / (unsigned int)(255-LM3639_MIN_VALUE_SETTINGS);
		//bright_per = level / 2;
#else
		bright_per = (level - (unsigned int)20) *(unsigned int)100 / (unsigned int)235;
#endif
		#endif
		data = bright_arr[bright_per];
		current_level = bright_per;
		printk("[Backlight] %s bright_per = %d, data = %d\n", __func__, bright_per, data);
		lm3639_mapping_level = data;
		if (is_suspend == 1)
		{
			//printk( "------	 backlight_level resume-----\n");
			//is_suspend = 0;
			mt_set_gpio_out(GPIO_LCM_BL_EN,GPIO_OUT_ONE);
			lm3639_power_on=1; //LGE_CHANGE: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
			mdelay(10);
			down_interruptible(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
			if(check_charger_pump_vendor() == FALSE)
			{
				#if defined(TARGET_MT6582_Y90) /* sangcheol.seo@lge.com 20141103 : brightness */
				//data1 = 0x51; // Y90 Richtek RT8542 Max current 8.5mA
				data1 = 0x73; // Y90 Richtek RT8542 Max current 12mA (0:5mA,1:8.5mA,2:12mA) // 2014-12-30
				#elif defined(TARGET_MT6582_B2L) || defined(TARGET_MT6582_L80) //G3S, L80+ source sync with KK
				data1 = 0x34;//0x37;
				#else
				data1 = 0x54;
				#endif
				lm3639_smbus_write_byte(new_client, 0x02, &data1);
				printk("[ChargePump]-Richtek\n");
			}
			else
			{
				#if defined(TARGET_MT6582_Y90)
				//data1 = 0x50; // Y90 LM3632 Max current 5mA // 2014-12-05
				data1 = 0x73; // Y90 LM3632  Max current 12mA (0:5mA,1:8.5mA,2:12mA) // 2014-12-30
				#elif defined(TARGET_MT6582_L80) //G3S, L80+ source sync with KK
				data1 = 0x34;// L80+ BLED. OVP 24V
				#else
				data1 = 0x54;// G3 Stylus BLED. OVP 32V
				#endif
				lm3639_smbus_write_byte(new_client, 0x02, &data1);
				printk("[LM3639]-TI\n");
			}

			lm3639_smbus_write_byte(new_client, 0x05, &data);
			printk("[LM3639]-backlight brightness Setting[reg0x05][value:0x%x]\n",data);

		#if 0 //CABC Check
		{
			unsigned char data2 = 0;

			lm3639_smbus_read_byte(new_client, 0x09, &data2);
			printk("[LM3639]-CABC PWM-read reg[reg0x09][value:0x%x]\n",data2);
			data2 |= 0x68;
			lm3639_smbus_write_byte(new_client, 0x09, &data2);
		}
		#endif

			lm3639_smbus_read_byte(new_client, 0x0A, &data1);
			printk("[LM3639]-backlight brightness Read[reg0x0A][value:0x%x]\n",data1);
			#if defined(TARGET_MT6582_L80) || defined (TARGET_MT6582_B2L)//For Dark Area at Right Bottom after LCD resume in L80, G3S
			data1 |= 0x19;
			#else
			data1 &= ~(0x19); // for reset BLED 1/2 EN, Backlight EN
			data1 |= 0x11; //Enable BLED1 , Backlight EN
			#endif
			//LGE_CHANGE_S: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
			if(flash_ctrl==0)
				data1 &= 0x99; //flashlight off
			//LGE_CHANGE_E: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.

			lm3639_smbus_write_byte(new_client, 0x0A, &data1);
			
			printk("[LM3639]-backlight brightness Write[reg0x0A][value:0x%x]\n",data1);
			printk("[LCD]-Backlight On \n");//add log
			is_suspend = 0; //[LGE_UPDATE][woonghwan.lee@lge.com][2014-07-14] Move backlight suspend setting position into semaphore
			up(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
		}

		if (level != 0)	//[LGE_UPDATE][woonghwan.lee@lge.com][2014-07-14] Move backlight suspend setting position into semaphore
		{
			down_interruptible(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
			//	mt_set_gpio_out(GPIO_LCM_BL_EN,GPIO_OUT_ONE);
			//		mdelay(10);
			{
				unsigned char read_data = 0;

				lm3639_smbus_read_byte(new_client, 0x02, &read_data);
	            printk("[LM3639]-is_suspend[0x%x]\n",read_data);
			}
			printk("[LM3639]-backlight Setting[reg0x05][value:0x%x]\n",data);
			lm3639_smbus_write_byte(new_client, 0x05, &data);
			up(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
		} //[LGE_UPDATE][woonghwan.lee@lge.com][2014-07-14] Move backlight suspend setting position into semaphore
	}
	return 0;
}

int chargepump_backlight_level_test(unsigned int level)
{
	unsigned char data = 0;

	data = 0x73; // Y90 LM3632 Max current 5mA // 2014-12-05
	lm3639_smbus_write_byte(new_client, 0x02, &data);
	data = level;
	lm3639_smbus_write_byte(new_client, 0x05, &data);
	data = 0x11;
	lm3639_smbus_write_byte(new_client, 0x0A, &data);

}

unsigned char get_lm3639_backlight_level(void)
{
	return lm3639_mapping_level;
}

static int lm3639_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	unsigned char data = 0x01;
	new_client = client;
	int err;
    CPD_FUN();
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CPD_LOG("i2c_check_functionality error\n");
		return -1;
	}

    sema_init(&lm3639_lock, 1); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.

	if (client == NULL)
		printk("%s client is NULL\n", __func__);
	else
	{
//		printk("%s %x %x %x\n", __func__, client->adapter, client->addr, client->flags);
	}
	return 0;
}

static int lm3639_remove(struct i2c_client *client)
{
    new_client = NULL;
	return 0;
}


static int lm3639_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	return 0;
}

static struct i2c_driver lm3639_i2c_driver = {
	.driver = {
//		.owner	= THIS_MODULE,
		.name	= LM3639_DEV_NAME,
	},
	.probe		= lm3639_probe,
	.remove		= lm3639_remove,
//	.detect		= lm3639_detect,
	.id_table	= lm3639_i2c_id,
//	.address_data = &lm3639250_i2c_addr_data,
};

static int lm3639_pd_probe(struct platform_device *pdev)
{
	mt_set_gpio_mode(GPIO_LCM_BL_EN, GPIO_LCM_BL_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCM_BL_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCM_BL_EN, GPIO_DIR_OUT);

	i2c_add_driver(&lm3639_i2c_driver);
	return 0;
}

static int lm3639_pd_remove(struct platform_device *pdev)
{
    CPD_FUN();
    i2c_del_driver(&lm3639_i2c_driver);
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3639_early_suspend(struct early_suspend *h)
{
	int err = 0;
	unsigned char data;
	down_interruptible(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
	data = 0x00; //backlight2 brightness 0
	err = lm3639_smbus_write_byte(new_client, 0x05, &data);

    err = lm3639_smbus_read_byte(new_client, 0x0A, &data);
    data &= 0x66;

	err = lm3639_smbus_write_byte(new_client, 0x0A, &data);
	up(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
	printk("\n[LM3639] lm3639_early_suspend  [%d]",data);
	//mt_set_gpio_out(GPIO_LCM_BL_EN,GPIO_OUT_ZERO);
}

//LGE_CHANGE_S: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
void lm3639_flash_ctrl_during_lcd_off()
{
    printk("lm3639_flash_ctrl_during_lcd_off, flash_ctrl:%d\n",flash_ctrl);
    int data1;

    if(flash_ctrl == 2)
    {
        mt_set_gpio_out(GPIO_LCM_BL_EN,GPIO_OUT_ONE);

        down_interruptible(&lm3639_lock);
        lm3639_smbus_read_byte(new_client, 0x06, &data1);
        data1 &= 0x0F;
        data1 |= 0x50;
        lm3639_smbus_write_byte(new_client, 0x06, &data1);

        data1 = 0x40 | 0x1f;
        lm3639_smbus_write_byte(new_client, 0x07, &data1);

        lm3639_smbus_read_byte(new_client, 0x09, &data1);
        data1 &= 0xF3;
        data1 |= (0x20 | 0x10);
        lm3639_smbus_write_byte(new_client, 0x09, &data1);

        lm3639_smbus_read_byte(new_client, 0x0A, &data1);
        data1 &= 0x99;
        data1 |= 0x62;
        lm3639_smbus_write_byte(new_client, 0x0A, &data1);
        up(&lm3639_lock);
    }
    else
    {
        mt_set_gpio_out(GPIO_LCM_BL_EN,GPIO_OUT_ZERO);
    }
}
//LGE_CHANGE_E: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.

void lm3639_flash_strobe_prepare(char OnOff,char ActiveHigh)
{
	int err = 0;

    down_interruptible(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.

    err = lm3639_smbus_read_byte(new_client, 0x09, &strobe_ctrl);

//[LGE_UPDATE_S][yonghwan.lym@lge.com][2014-07-12] flash_en register(0x0A) setting position change.
    strobe_ctrl &= 0xF3;
    flash_ctrl = OnOff;

	if(ActiveHigh)
	{
		strobe_ctrl |= 0x20;
	}
	else
	{
		strobe_ctrl &= 0xDF;
	}

	if(OnOff == 1)
	{
		CPD_LOG("Strobe mode On\n");
		strobe_ctrl |= 0x10;
    }
    else if(OnOff == 2)
    {
		CPD_LOG("Torch mode On\n");
		strobe_ctrl |= 0x10;
    }
	else
	{
		CPD_LOG("Flash Off\n");
		strobe_ctrl &= 0xEF;
	}
//[LGE_UPDATE_E][yonghwan.lym@lge.com][2014-07-12] flash_en register(0x0A) setting position change.
	err = lm3639_smbus_write_byte(new_client, 0x09, &strobe_ctrl);

    up(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
}

//strobe enable
void lm3639_flash_strobe_en()
{
//[LGE_UPDATE_S][yonghwan.lym@lge.com][2014-07-12] flash_en register(0x0A) setting position change.
    int err = 0;
    int flash_OnOff=0;

    //LGE_CHANGE_S: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
    if(lm3639_power_on == 0)
    {
        lm3639_flash_ctrl_during_lcd_off();
        return;
    }
    //LGE_CHANGE_E: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.

    down_interruptible(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
    err = lm3639_smbus_read_byte(new_client, 0x0A, &flash_OnOff);
    if(flash_ctrl == 1)
        flash_OnOff |= 0x66;
    else if(flash_ctrl == 2)
        flash_OnOff |= 0x62;
    else
        flash_OnOff &= 0x99;
    err = lm3639_smbus_write_byte(new_client, 0x0A, &flash_OnOff);
    up(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
//[LGE_UPDATE_E][yonghwan.lym@lge.com][2014-07-12] flash_en register(0x0A) setting position change.
}

//strobe level
void lm3639_flash_strobe_level(char level)
{
	int err = 0;
	unsigned char data1=0;
    unsigned char data2=0;
    unsigned char torch_level;
    unsigned char strobe_timeout = 0x1F;
    down_interruptible(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
#if 0 //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-10] Add Main Flash current(4 Step)
    if( level == 1)
    {
        torch_level = 0x20;
    }
    else
    {
        torch_level = 0x50;
    }

    err = lm3639_smbus_read_byte(new_client, 0x06, &data1);

	if(31 < level)
    {
		data1= torch_level | 0x0A;
        strobe_timeout = 0x0F;
    }
    else if(level < 0)
    {
		data1= torch_level ;
    }
    else
    {
		data1= torch_level | level;
    }
//[LGE_UPDATE_S][yonghwan.lym@lge.com][2014-06-10] Add Main Flash current(4 Step)
#else
    torch_level = 0x50;

    err = lm3639_smbus_read_byte(new_client, 0x06, &data1);

    strobe_timeout = 0x1F; //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-27] Flash Timing Tuning
    if(level < 0)
        data1= torch_level;
    else if(level == 1)
        data1= torch_level | 0x03;
    else if(level == 2)
        data1= torch_level | 0x05;
    else if(level == 3)
        data1= torch_level | 0x08;
    else if(level == 4)
        data1= torch_level | 0x0A;
    else
        data1= torch_level | level;

#endif
//[LGE_UPDATE_E][yonghwan.lym@lge.com][2014-06-10] Add Main Flash current(4 Step)
/*
    if(0)
    {
	    CPD_LOG("Batt temp=%d\n", BMT_status.temperature );

        torch_level = 0xF0 & data1;
        level = 0x0F & data1;
        torch_level = 0xF0 & (torch_level >> 2);
        level = 0x0F & (level >> 2);

        data1 = torch_level | level;
    }
*/
	CPD_LOG("Flash Level =0x%x\n", data1);
    err = lm3639_smbus_write_byte(new_client, 0x06, &data1);

    data2 = 0x40 | strobe_timeout;
    CPD_LOG("Storbe Timeout =0x%x\n", data2);
    err |= lm3639_smbus_write_byte(new_client, 0x07, &data2);
    up(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
}

static void lm3639_late_resume(struct early_suspend *h)
{
	int err = 0;
	unsigned char data1;

	mt_set_gpio_out(GPIO_LCM_BL_EN, GPIO_OUT_ONE);
	lm3639_power_on = 1; //LGE_CHANGE: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
	mdelay(50);
	down_interruptible(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
	err = lm3639_smbus_write_byte(new_client, 0x05, &current_brightness);

    err = lm3639_smbus_read_byte(new_client, 0x0A, &data1);
	data1 |= 0x19;//backlight enable
	//LGE_CHANGE_S: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
	if(flash_ctrl==0)
		data1 &= 0x99;
	//LGE_CHANGE_E: [2014-11-28] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.

	err = lm3639_smbus_write_byte(new_client, 0x0A, &data1);
	up(&lm3639_lock); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-08] Add semaphore for lcd and flash i2c communication.
	printk("\n [LM3639] lm3639_late_resume  [%d]",data1);
}

static struct early_suspend lm3639_early_suspend_desc = {
	.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend	= lm3639_early_suspend,
	.resume		= lm3639_late_resume,
};
#endif

static struct platform_driver lm3639_backlight_driver = {
	.probe      = lm3639_pd_probe,
	.remove     = lm3639_pd_remove,
	.driver     = {
	.name  = "charge-pump",
	.owner = THIS_MODULE,
	}
};

static int __init lm3639_init(void)
{
	CPD_FUN();
	//i2c number 1(0~2) control
#if 0 //sangcheol.seo@lge.com 20140925
	printk("[LM3639] lm3639_hw_rev = %d\n", lge_get_board_revno());
	if(lge_get_board_revno() < HW_REV_A){
		i2c_register_board_info(2, &i2c_lm3639, 1);
		printk("[LM3639] lm3639_i2c_master_addr = 2");
	}
	else{
		i2c_register_board_info(1, &i2c_lm3639, 1);
		printk("[LM3639] lm3639_i2c_master_addr = 1");
	}
#else
	//MTK Delta Package2 porting for G3S & L80+_Start

	/*i2c_register_board_info(1, &i2c_lm3639, 1);
	printk("[LM3639] lm3639_i2c_master_addr = 1");*/
	struct cust_leds_param* cus_hw = get_cust_leds_param();
	if (strcmp(cus_hw->name, lm3639_backlight_driver.driver.name)){
			CPD_ERR("lm3639 mismatch device_name(%s,%s)\n", cus_hw->name,lm3639_backlight_driver.driver.name);
			return -1;
		}
		CPD_LOG("lm3639(%s) I2C channel =%d\n", cus_hw->name, (int)cus_hw->args);

        i2c_register_board_info((int)cus_hw->args, &i2c_lm3639, 1);

	//MTK Delta Package2 porting for G3S & L80+_End
#endif
	#ifndef	CONFIG_MTK_LEDS
	register_early_suspend(&lm3639_early_suspend_desc);
	#endif

	if(platform_driver_register(&lm3639_backlight_driver))
	{
		CPD_ERR("failed to register driver");
		return -1;
	}

	return 0;
}

static void __exit lm3639_exit(void)
{
	platform_driver_unregister(&lm3639_backlight_driver);
}

EXPORT_SYMBOL(lm3639_flash_strobe_en);
EXPORT_SYMBOL(lm3639_flash_strobe_prepare);
EXPORT_SYMBOL(lm3639_flash_strobe_level);
MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("lm3639 driver");
MODULE_LICENSE("GPL");

module_init(lm3639_init);
module_exit(lm3639_exit);

