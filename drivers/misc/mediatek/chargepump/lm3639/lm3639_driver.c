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

#include <linux/leds.h>

#define LCD_LED_MAX 0x7F
#define LCD_LED_MIN 0

#define DEFAULT_BRIGHTNESS 0x73 //for 20mA
#define LM3639_MIN_VALUE_SETTINGS 20 /* value leds_brightness_set*/
#define LM3639_MAX_VALUE_SETTINGS 255 /* value leds_brightness_set*/
#define MIN_MAX_SCALE(x) (((x)<LM3639_MIN_VALUE_SETTINGS) ? LM3639_MIN_VALUE_SETTINGS : (((x)>LM3639_MAX_VALUE_SETTINGS) ? LM3639_MAX_VALUE_SETTINGS:(x)))

#define BACKLIHGT_NAME "charge-pump"

#define Lm3639_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define LM3639_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

#define LM3639_DEV_NAME        "charge-pump"

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
#define GPIO_LCD_BL_EN GPIO_LCM_BL_EN
#define GPIO_LCD_BL_EN_M_GPIO GPIO_LCM_BL_EN_M_GPIO
// Gamma 2.2 Table
unsigned char bright_arr[] = {  // array index max 100, value under 255
	3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 6, 6, 7, 7, 8, 8,  // 19
	9, 9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19,  // 39
	19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 34, 35, 36, 36,  // 59
	37, 38, 40, 42, 43, 44, 45, 47, 49, 51, 52, 54, 56, 58, 60, 61, 63, 65, 67, 69,  //79
	71, 72, 74, 76, 78, 80, 83, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 113  // 100
};


static unsigned char current_brightness = 0;
static unsigned char is_suspend = 0;
static bool lm3639_power_on = 1; //LGE_CHANGE: [2014-12-02] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.

struct semaphore lm3639_lock;

/* generic */
#define LM3639_MAX_RETRY_I2C_XFER (100)
#define LM3639_I2C_WRITE_DELAY_TIME 1
// [LGE_UPDATE_S] [dongyu.gwak@lge.com] [2012-12-27] current limit
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

int chargepump_set_backlight_level(unsigned int level)
{
	unsigned char data = 0;
	unsigned char data1 = 0;
	unsigned char data2 = 0;
	unsigned int bright_per = 0;

	printk("\n[LM3639] chargepump_set_backlight_level  [%d]\n",data1);

	if (level == 0)
	{
		if(is_suspend == 0)
		{
			printk( "[LM3639] backlight off\n");
			down_interruptible(&lm3639_lock);
			data1 = 0x00; //backlight2 brightness 0
			lm3639_smbus_write_byte(new_client, 0x05, &data1);
			lm3639_smbus_read_byte(new_client, 0x0A, &data1);
			data1 &= 0x66;

			lm3639_smbus_write_byte(new_client, 0x0A, &data1);
			is_suspend = 1;
			up(&lm3639_lock);
			//LGE_CHANGE_S: [2014-12-02] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
			printk( "[LM3639] flash_ctrl = %d\n", flash_ctrl);
			if( flash_ctrl == 0 ){
				mt_set_gpio_out(GPIO_LCD_BL_EN,GPIO_OUT_ZERO);
				lm3639_power_on = 0;
			}
			//LGE_CHANGE_E: [2014-12-02] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
			//is_suspend = 1;
		}
	}
	else
	{
		level = MIN_MAX_SCALE(level);
		printk(KERN_ERR "[LM3639] level = %d", level);

		//Gamma 2.2 Table adapted
		bright_per = (level - (unsigned int)20) *(unsigned int)100 / (unsigned int)235;
		data = bright_arr[bright_per];

		//printk("[Backlight] %s bright_per = %d, data = %d\n", __func__, bright_per, data);

		if (is_suspend == 1)
		{
			//printk( "------	 backlight_level resume-----\n");
			//is_suspend = 0;
			mt_set_gpio_out(GPIO_LCD_BL_EN,GPIO_OUT_ONE);
			lm3639_power_on = 1; //LGE_CHANGE: [2014-12-02] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
			mdelay(10);
			down_interruptible(&lm3639_lock);
			if(check_charger_pump_vendor() == FALSE)
			{
				data1 = 0x73;//0x37;
				lm3639_smbus_write_byte(new_client, 0x02, &data1);
				printk("[ChargePump]-Richtek\n");
			}
			else
			{
				data1 = 0x73;//0x57;
				lm3639_smbus_write_byte(new_client, 0x02, &data1);
				printk("[LM3639]-TI\n");
			}

			lm3639_smbus_write_byte(new_client, 0x05, &data);
			printk("[lm3639]-backlight brightness Setting[reg0x05][value:0x%x]\n",data);

		#if 1 //CABC Check
			lm3639_smbus_read_byte(new_client, 0x09, &data2);
			printk("[LM3639]-CABC PWM-read reg[reg0x09][value:0x%x]\n",data2);
			data2 |= 0x68;
			lm3639_smbus_write_byte(new_client, 0x09, &data2);
		#endif

            lm3639_smbus_read_byte(new_client, 0x0A, &data1);
			data1 &= ~(0x19); // for reset BLED 1/2 EN, Backlight EN
			data1 |= 0x11; //Enable BLED1 , Backlight EN
//[LGE_UPDATS][yonghwan.lym@lge.com][2014-11-28] flash
			if(flash_ctrl==0)
				data1 &= 0x99; // flashlight off
//[LGE_UPDATE][yonghwan.lym@lge.com][2014-11-28] flash

			lm3639_smbus_write_byte(new_client, 0x0A, &data1);
			is_suspend = 0; //[LGE_UPDATE][woonghwan.lee@lge.com][2014-07-14] Move backlight suspend setting position into semaphore
			up(&lm3639_lock);
		}

		if (level != 0)	//[LGE_UPDATE][woonghwan.lee@lge.com][2014-07-14] Move backlight suspend setting position into semaphore
		{
			down_interruptible(&lm3639_lock);
			{
				unsigned char read_data = 0;

				lm3639_smbus_read_byte(new_client, 0x02, &read_data);
				printk("[lm3639]-OVP[0x%x]\n",read_data);
			}

			printk("[lm3639]-backlight Seting[reg0x05][value:0x%x]\n",data);
			lm3639_smbus_write_byte(new_client, 0x05, &data);
			up(&lm3639_lock);
		}
	}
	return 0;
}

static int lm3639_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	new_client = client;

    CPD_FUN();
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CPD_LOG("i2c_check_functionality error\n");
		return -1;
	}

    sema_init(&lm3639_lock, 1);

	if (client == NULL)
		printk("%s client is NULL\n", __func__);
	else
	{
		printk("%s %p %x %x\n", __func__, client->adapter, client->addr, client->flags);
	}
	return 0;
}


static int lm3639_remove(struct i2c_client *client)
{
    new_client = NULL;
	return 0;
}


static int __attribute__ ((unused)) lm3639_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
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
	mt_set_gpio_mode(GPIO_LCD_BL_EN, GPIO_LCD_BL_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCD_BL_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCD_BL_EN, GPIO_DIR_OUT);

	i2c_add_driver(&lm3639_i2c_driver);
	return 0;
}

static int __attribute__ ((unused)) lm3639_pd_remove(struct platform_device *pdev)
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
	down_interruptible(&lm3639_lock);
	data = 0x00; //backlight2 brightness 0
	err = lm3639_smbus_write_byte(new_client, 0x05, &data);

    err = lm3639_smbus_read_byte(new_client, 0x0A, &data);
    data &= 0x66;

	err = lm3639_smbus_write_byte(new_client, 0x0A, &data);
	up(&lm3639_lock);
	printk("\n[LM3639] lm3639_early_suspend  [%d]",data);
	//mt_set_gpio_out(GPIO_LCD_BL_EN,GPIO_OUT_ZERO);
}

//LGE_CHANGE_S: [2014-12-02] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
void lm3639_flash_ctrl_during_lcd_off()
{
    printk("lm3639_flash_ctrl_during_lcd_off, flash_ctrl:%d\n",flash_ctrl);
    int data1;

    if(flash_ctrl == 2)
    {
        mt_set_gpio_out(GPIO_LCD_BL_EN,GPIO_OUT_ONE);

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
        mt_set_gpio_out(GPIO_LCD_BL_EN,GPIO_OUT_ZERO);
    }
}

void lm3639_flash_strobe_prepare(char OnOff,char ActiveHigh)
{
	int err = 0;

    down_interruptible(&lm3639_lock);

    err = lm3639_smbus_read_byte(new_client, 0x09, &strobe_ctrl);

    err = lm3639_smbus_read_byte(new_client, 0x0A, &flash_ctrl);
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

    up(&lm3639_lock);
}

//strobe enable
void lm3639_flash_strobe_en()
{
//[LGE_UPDATE_S][yonghwan.lym@lge.com][2014-07-12] flash_en register(0x0A) setting position change.
    int err = 0;
    int flash_OnOff=0;

    //LGE_CHANGE_S: [2014-12-02] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
    if(lm3639_power_on == 0)
    {
        lm3639_flash_ctrl_during_lcd_off();
        return;
    }
    //LGE_CHANGE_E: [2014-12-02] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.

    down_interruptible(&lm3639_lock);
    err = lm3639_smbus_read_byte(new_client, 0x0A, &flash_OnOff);
    if(flash_ctrl == 1)
        flash_OnOff |= 0x66;
    else if(flash_ctrl == 2)
        flash_OnOff |= 0x62;
    else
        flash_OnOff &= 0x99;
    err = lm3639_smbus_write_byte(new_client, 0x0A, &flash_OnOff);
    up(&lm3639_lock);
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
    down_interruptible(&lm3639_lock);
#if 0
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

#else
// LGE_UPDATE_S [youmi.jun@lge.com] Modify flash level (EAV62991901)
    torch_level = 0x30; // 225/2 = 112.5mA

    err = lm3639_smbus_read_byte(new_client, 0x06, &data1);

    strobe_timeout = 0x1F;  //LGE_UPDATE [youmi.jun@lge.com] 2014/12/29, Flash timing tuning
    if(level < 0)
        data1= torch_level;
    else if(level == 1)
        data1= torch_level | 0x02; // 281.25/2 = 140.625mA
    else if(level == 2)
        data1= torch_level | 0x04; // 468.75/2 = 234.375mA
    else if(level == 3)
        data1= torch_level | 0x06; // 656.25/2 = 328.125mA
    else if(level == 4)
        data1= torch_level | 0x08; // 843.75/2 = 421.875mA
    else
        data1= torch_level | level;
// LGE_UPDATE_E [youmi.jun@lge.com] Modify flash level
#endif

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
    up(&lm3639_lock);
}

static void lm3639_late_resume(struct early_suspend *h)
{
	int err = 0;
	unsigned char data1;

	mt_set_gpio_out(GPIO_LCD_BL_EN, GPIO_OUT_ONE);
	lm3639_power_on = 1; //LGE_CHANGE: [2014-12-02] yonghwan.lym@lge.com, lm3639 is disable, when push the power key.
	mdelay(50);
	down_interruptible(&lm3639_lock);
	err = lm3639_smbus_write_byte(new_client, 0x05, &current_brightness);

    err = lm3639_smbus_read_byte(new_client, 0x0A, &data1);
	data1 |= 0x19;//backlight enable
//[LGE_UPDATS][yonghwan.lym@lge.com][2014-11-28] flash
	if(flash_ctrl==0)
		data1 &= 0x99; // flashlight off
//[LGE_UPDATE][yonghwan.lym@lge.com][2014-11-28] flash

	err = lm3639_smbus_write_byte(new_client, 0x0A, &data1);
	up(&lm3639_lock);
	printk("\n [LM3639] lm3639_late_resume  [%d]",data1);
}

static struct early_suspend __attribute__ ((unused)) lm3639_early_suspend_desc = {
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

static struct platform_device mtk_backlight_dev = {
	.name = "charge-pump",
	.id   = -1,};

static int __init lm3639_init(void)
{
	CPD_FUN();
	if (platform_device_register(&mtk_backlight_dev))
	{
		CPD_ERR("failed to register device");
		return -1;
	}
	//i2c number 1(0~2) control
	i2c_register_board_info(3, &i2c_lm3639, 1);

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

