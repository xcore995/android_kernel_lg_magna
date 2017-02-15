/***************************************************************************
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
 *    File  	: mediatek\custom\common\kernel\alsps\apds9930.c
 *    Author(s)   :  Kang Jun Mo < junmo.kang@lge.com >
 *    Description :
 *
 ***************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/
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

#define MT6582

#ifdef MT6516
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_pll.h>
#endif

#ifdef MT6573
#include <mach/mt6573_devs.h>
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_pll.h>
#endif


#ifdef MT6582
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif


#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#endif

#ifdef MT6573
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6582
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "apds9930.h"

#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define APDS9930_DEV_NAME     "APDS9930"

#define APDS9930_ENABLE_REG 0x00
#define APDS9930_PTIME_REG  0x02
#define APDS9930_WTIME_REG  0x03
#define APDS9930_PILTL_REG  0x08
#define APDS9930_PILTH_REG  0x09
#define APDS9930_PIHTL_REG  0x0A
#define APDS9930_PIHTH_REG  0x0B
#define APDS9930_PERS_REG 0x0C
#define APDS9930_CONFIG_REG 0x0D
#define APDS9930_PPCOUNT_REG  0x0E
#define APDS9930_CONTROL_REG  0x0F
#define APDS9930_REV_REG  0x11
#define APDS9930_ID_REG   0x12
#define APDS9930_STATUS_REG 0x13
#define APDS9930_PDATAL_REG 0x18
#define APDS9930_PDATAH_REG 0x19

//add register for als : HN
#define APDS9930_ATIME_REG  0x01
#define APDS9930_AILTL_REG  0x04
#define APDS9930_AILTH_REG  0x05
#define APDS9930_AIHTL_REG  0x06
#define APDS9930_AIHTH_REG  0x07
#define APDS9930_CDATA0L_REG 0x14
#define APDS9930_CDATA0H_REG 0x15
#define APDS9930_CDATA1L_REG 0x16
#define APDS9930_CDATA1H_REG 0x17

#define CMD_BYTE  0x80
#define CMD_WORD  0xA0
#define CMD_SPECIAL 0xE0

#define CMD_CLR_PS_INT  0xE5
#define CMD_CLR_ALS_INT 0xE6
#define CMD_CLR_PS_ALS_INT  0xE7

#define APDS9930_PINT 0x20
#define APDS9930_PVALID 0x02

#define APDS9930_PSAT 0x40  /* PS saturation bit check */

/* Add for ALS */
#define APDS9930_AINT 0x10
#define APDS9930_PAINT 0x30
#define APDS9930_AVALID 0x01
#define APDS9930_PAVALID 0x03


/****************************************************************************
 * Macros
 ****************************************************************************/
#define SENSOR_TAG                  "[LGE_Proximity]"
//#define DEBUG 1
	
#ifdef DEBUG
#define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
#define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt, __FUNCTION__,##args)
#define SENSOR_DBG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[D]""%s : "fmt, __FUNCTION__,##args)
#else
#define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
#define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt, __FUNCTION__,##args)
#define SENSOR_DBG(fmt, args...)    NULL
#endif

/****************************************************************************
* Type Definitions
****************************************************************************/
typedef enum
{
	PS_NEAR = 0,
	PS_FAR = 1,
	PS_UNKNOWN = 2
} PS_STATUS;

/* Add for ALS */
typedef enum
{
	ALS_DARK = 0,
	ALS_BRIGHT = 1,
	ALS_UNKNOWN = 2
} ALS_STATUS;

struct apds9930_priv
{
	struct i2c_client *client;
	struct work_struct eint_work;

	unsigned int activate; /* 1 = activate, 0 = deactivate */
	unsigned int als_activate; /* 1 = activate, 0 = deactivate */

	/* variables to store APDS9930 register value - begin */
	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int ppcount;
	unsigned int control;
	/* variables to store APDS9930 register value - end */

	/* CAUTION : in case of strong sunlight, ps_state is not same as ps_th_status. */
	unsigned int ps_status; /* current status of poximity detection : 0 = near, 1 = far */
	unsigned int ps_th_status; /* current threshold status of poximity detection : 0 = near, 1 = far */

	/* threshold value to detect "near-to-far" event */
	unsigned int ps_th_near_low;
	unsigned int ps_th_near_high;

	/* threshold value to detect "far-to-near" event */
	unsigned int ps_th_far_low;
	unsigned int ps_th_far_high;

	unsigned int ps_cross_talk; /* a result value of calibration. it will be used to compensate threshold value. */

	/******* Add for ALS *******/

	unsigned int als_status; /* current status : 0 = normal, 1 = strong light detect */
	unsigned int als_th_status; /* current threshold status : 0 = normal, 1 = strong light detect */

	/* threshold value to detect "bright-to-dark" event */
	unsigned int als_th_dark_low;
	unsigned int als_th_dark_high;

	/* threshold value to detect "dark-to-bright" event */
	unsigned int als_th_bright_low;
	unsigned int als_th_bright_high;

	/******* Add for ALS *******/

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
	#endif
	
};


/****************************************************************************
* Variables
****************************************************************************/
static struct i2c_client *apds9930_i2c_client = NULL; /* for general file I/O service. will be init on apds9930_i2c_probe() */
static struct apds9930_priv *g_apds9930_ptr = NULL; /* for interrupt service call. will be init on apds9930_i2c_probe() */
static struct platform_driver apds9930_alsps_driver;

static int Target_Pdata = 180;//150; /* parameter for taget pdata setting */
static int NearToFar = 50; /* parameter for far detection */

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern void mt_eint_unmask ( unsigned int line );
extern void mt_eint_mask ( unsigned int line );
extern void mt_eint_set_polarity ( unsigned int eint_num, unsigned int pol );
extern void mt_eint_set_hw_debounce ( unsigned int eint_num, unsigned int ms );
extern unsigned int mt_eint_set_sens ( unsigned int eint_num, unsigned int sens );
//extern void mt_eint_registration ( unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void ( EINT_FUNC_PTR ) ( void ),
	//								   unsigned int is_auto_umask );
void mt_eint_registration(unsigned int eint_num, unsigned int flag,
              void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);


#ifdef MT6516
extern void MT6516_EINTIRQUnmask ( unsigned int line );
extern void MT6516_EINTIRQMask ( unsigned int line );
extern void MT6516_EINT_Set_Polarity ( kal_uint8 eintno, kal_bool ACT_Polarity );
extern void MT6516_EINT_Set_HW_Debounce ( kal_uint8 eintno, kal_uint32 ms );
extern kal_uint32 MT6516_EINT_Set_Sensitivity ( kal_uint8 eintno, kal_bool sens );
extern void MT6516_EINT_Registration ( kal_uint8 eintno, kal_bool Dbounce_En, kal_bool ACT_Polarity, void ( EINT_FUNC_PTR ) ( void ),
									   kal_bool auto_umask );
#endif

/****************************************************************************
* Local Function Prototypes
****************************************************************************/
void apds9930_eint_func ( void );


/****************************************************************************
* Local Functions
****************************************************************************/

/* Add for ALS */
void (*apds9930_lux_change_cb)(int);
/* only one callback is maintained. may change it to list of callbacks */
void apds9930_register_lux_change_callback(void(*callback)(int))
{
	apds9930_lux_change_cb = callback;
}

//==========================================================
// Platform(AP) dependent functions
//==========================================================
static void apds9930_setup_eint ( void )
{
	SENSOR_FUN ();

#if 0 //not use
	/* Configure GPIO settings for external interrupt pin  */
	mt_set_gpio_dir ( GPIO_ALS_EINT_PIN, GPIO_DIR_IN );
	mt_set_gpio_mode ( GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT );
	mt_set_gpio_pull_enable ( GPIO_ALS_EINT_PIN, TRUE );
	mt_set_gpio_pull_select ( GPIO_ALS_EINT_PIN, GPIO_PULL_UP );

	/* Configure external interrupt settings for external interrupt pin */
	mt_eint_set_sens ( CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE );
	mt_eint_set_polarity ( CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY );
	mt_eint_set_hw_debounce ( CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN );

	mt_eint_registration ( CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, apds9930_eint_func, 0);

	/* Mask external interrupt to avoid un-wanted interrupt. Unmask it after initialization of APDS9930 */
	mt_eint_mask ( CUST_EINT_ALS_NUM );
#else
	/* Configure GPIO settings for external interrupt pin  */
	mt_set_gpio_mode(GPIO_PROXIMITY_INT, GPIO_PROXIMITY_INT_M_EINT);
	mt_set_gpio_dir(GPIO_PROXIMITY_INT, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_PROXIMITY_INT, GPIO_PULL_DISABLE);

	/* Configure external interrupt settings for external interrupt pin */
	mt_eint_set_hw_debounce(CUST_EINT_PROXIMITY_NUM, CUST_EINT_PROXIMITY_DEBOUNCE_EN);
	mt_eint_registration(CUST_EINT_PROXIMITY_NUM, EINTF_TRIGGER_FALLING, apds9930_eint_func, 0);

	/* Mask external interrupt to avoid un-wanted interrupt. Unmask it after initialization of APDS9930 */
	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );
#endif	
}
extern unsigned int system_rev;

#if 0 //not use
static void apds9930_main_power ( struct alsps_hw *hw, unsigned int on )
{
	static unsigned int main_power_on = 0xFF;

	SENSOR_FUN ();

	if ( main_power_on != on )
	{
		if ( on )
		{
			if ( !hwPowerOn ( hw->power_id, hw->power_vol, "APDS9930" ) )
			{
				SENSOR_ERR ( "failed to power on ( VCAM_AF )\n" );
				goto EXIT_ERR;
			}

			SENSOR_LOG( "turned on the power ( VCAM_AF )" );
		}
		else
		{
			if ( !hwPowerDown ( hw->power_id, "APDS9930" ) )
			{
				SENSOR_ERR ( "failed to power down ( VCAM_AF )\n" );
				goto EXIT_ERR;
			}

			SENSOR_LOG( "turned off the power ( VCAM_AF )" );
		}

		main_power_on = on;
	}

	EXIT_ERR:
		return;
}

#endif

#if 0 //not use
static void apds9930_led_power ( unsigned int on )
{
	static unsigned int led_power_on = 0xFF;

	SENSOR_FUN ();

	if ( led_power_on != on )
	{
		if ( on )
		{
		#ifdef MT6575
			if ( !hwPowerOn ( MT65XX_POWER_LDO_VCAMA, VOL_2800, "APDS9930LEDA" ) )
				{
					SENSOR_ERR ( "failed to power on ( VCAMA )\n" );
		#else
		if ( !hwPowerOn ( MT65XX_POWER_LDO_VCAM_AF, VOL_3000, "APDS9930LEDA" ) )
				{
					SENSOR_ERR ( "failed to power on ( VCAM_AF )\n" );
		#endif
		
					goto EXIT_ERR;
				}
		#ifdef MT6575
			SENSOR_LOG("turned on the power ( VCAMA )");
		#else
			SENSOR_LOG("turned on the power ( VCAM_AF )");
		#endif
		
		}
		else
		{
		#ifdef MT6575
			if ( !hwPowerDown ( MT65XX_POWER_LDO_VCAMA, "APDS9930LEDA" ) )
				{
					SENSOR_ERR ( "failed to power down ( VCAMA )\n" );
		#else
		if ( !hwPowerDown ( MT65XX_POWER_LDO_VCAM_AF, "APDS9930LEDA" ) )
				{
					SENSOR_ERR ( "failed to power down ( VCAM_AF )\n" );
		#endif
					goto EXIT_ERR;
				}
		#ifdef MT6575
			SENSOR_LOG("turned off the power ( VCAMA )\n");
		#else
			SENSOR_LOG("turned off the power ( VCAM_AF )\n");
		#endif
		
		}

		led_power_on = on;
	}

	EXIT_ERR:
		return;
}
#endif
//==========================================================
// APDS9930 Register Read / Write Funtions
//==========================================================
static int apds9930_write_cmd ( struct i2c_client *client, u8 val )
{
	int res = 0;

	res = i2c_master_send ( client, &val, 1 );
	if ( res == 1 )
	{
		SENSOR_DBG ( "I2C write ( val=0x%02x )\n", val );
		return APDS9930_SUCCESS;
	}
	else
	{
		SENSOR_ERR ( "failed to write to APDS9930 ( err=%d, cmd=0x%02x )\n", res, val );
		return APDS9930_ERR_I2C;
	}
}

static int apds9930_write_byte ( struct i2c_client *client, u8 reg, u8 val )
{
	int res = 0;
	u8 pBuf[2] = { 0 };

	pBuf[0] = reg;
	pBuf[1] = val;

	res = i2c_master_send ( client, pBuf, 2 );
	if ( res == 2 )
	{
		SENSOR_DBG ( "I2C write ( reg=0x%02x, val=0x%02x )\n", reg, val );
		return APDS9930_SUCCESS;
	}
	else
	{
		SENSOR_ERR ( "failed to write to APDS9930 ( err=%d, reg=0x%02x, val=0x%02x )\n", res, reg, val );
		return APDS9930_ERR_I2C;
	}

}

static int apds9930_write_word ( struct i2c_client *client, u8 reg, u16 val )
{
	int res = 0;
	u8 pBuf[3] = { 0 };

	pBuf[0] = reg ;
	pBuf[1] = val & 0xFF ;
	pBuf[2] = ( val >> 8 ) & 0xFF ;

	res = i2c_master_send ( client, pBuf, 3 );
	if ( res == 3 )
	{
		SENSOR_DBG ( "I2C write ( reg=0x%02x, val=0x%04x )\n", reg, val );
		return APDS9930_SUCCESS;
	}
	else
	{
		SENSOR_ERR ( "failed to write to APDS9930 ( err=%d, reg=0x%02x, val=0x%04x )\n", res, reg, val );
		return APDS9930_ERR_I2C;
	}

}

static int apds9930_read_byte ( struct i2c_client *client, u8 reg, u8 *pVal )
{
	int res = 0;

	if ( pVal == NULL )
	{
		SENSOR_ERR ( "invalid input ( pVal=NULL )" );
		goto EXIT_ERR;
	}

	res = i2c_master_send ( client, &reg, 1 );
	if ( res != 1 )
	{
		SENSOR_ERR ( "apds9930_read_byte error i2c_master_send (1)....\n" );
		goto EXIT_ERR;
	}

	res = i2c_master_recv ( client, pVal, 1 );
	if ( res != 1 )
	{
		SENSOR_ERR ( "apds9930_read_byte error i2c_master_recv (2)....\n" );
		goto EXIT_ERR;
	}

	SENSOR_DBG ( "I2C read ( reg=0x%02x, val=0x%02x )\n", reg, *pVal );
	return APDS9930_SUCCESS;

	EXIT_ERR:
	SENSOR_ERR ( "failed to read from APDS9930 ( err=%d, reg=0x%02x )\n", res, reg );
	return APDS9930_ERR_I2C;
	
}

static int apds9930_read_word ( struct i2c_client *client, u8 reg, u16 *pVal )
{
	int res = 0;
	u8 pBuf[2] = { 0 };

	if ( pVal == NULL )
	{
		SENSOR_ERR ( "invalid input ( pVal=NULL )" );
		goto EXIT_ERR;
	}

	res = i2c_master_send ( client, &reg, 1 );
	if ( res != 1 )
	{
		goto EXIT_ERR;
	}

	res = i2c_master_recv ( client, pBuf, 2 );
	if ( res != 2 )
	{
		goto EXIT_ERR;
	}

	*pVal = ( ( u16 ) pBuf[1] << 8 ) | pBuf[0] ;

	SENSOR_DBG ( "I2C read ( reg=0x%02x, val=0x%04x )\n", reg, *pVal );
	return APDS9930_SUCCESS;

	EXIT_ERR:
	SENSOR_ERR ( "failed to read from APDS9930 ( err=%d, reg=0x%02x )\n", res, reg );
	return APDS9930_ERR_I2C;
	
}

//==========================================================
// APDS9930 Basic Read / Write Funtions
//==========================================================
static int apds9930_set_enable ( struct i2c_client *client, int enable )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_byte ( client, CMD_BYTE | APDS9930_ENABLE_REG, ( u8 ) enable );
	if ( res == APDS9930_SUCCESS )
	{
		obj->enable = enable;
	}

	return res;
}

static int apds9930_set_ptime ( struct i2c_client *client, int ptime )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_byte ( client, CMD_BYTE | APDS9930_PTIME_REG, ( u8 ) ptime );
	if ( res == APDS9930_SUCCESS )
	{
		obj->ptime = ptime;
	}

	return res;
}

static int apds9930_set_wtime ( struct i2c_client *client, int wtime )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_byte ( client, CMD_BYTE | APDS9930_WTIME_REG, ( u8 ) wtime );
	if ( res == APDS9930_SUCCESS )
	{
		obj->wtime = wtime;
	}

	return res;
}

static int apds9930_set_pilt ( struct i2c_client *client, int threshold )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_word ( client, CMD_WORD | APDS9930_PILTL_REG, ( u16 ) threshold );
	if ( res == APDS9930_SUCCESS )
	{
		obj->pilt = threshold;
	}

	return res;
}

static int apds9930_set_piht ( struct i2c_client *client, int threshold )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_word ( client, CMD_WORD | APDS9930_PIHTL_REG, ( u16 ) threshold );
	if ( res == APDS9930_SUCCESS )
	{
		obj->piht = threshold;
	}

	return res;
}

static int apds9930_set_pers ( struct i2c_client *client, int pers )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_byte ( client, CMD_BYTE | APDS9930_PERS_REG, ( u8 ) pers );
	if ( res == APDS9930_SUCCESS )
	{
		obj->pers = pers;
	}

	return res;
}

static int apds9930_set_config ( struct i2c_client *client, int config )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_byte ( client, CMD_BYTE | APDS9930_CONFIG_REG, ( u8 ) config );
	if ( res == APDS9930_SUCCESS )
	{
		obj->config = config;
	}

	return res;
}

static int apds9930_set_ppcount ( struct i2c_client *client, int ppcount )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_byte ( client, CMD_BYTE | APDS9930_PPCOUNT_REG, ( u8 ) ppcount );
	if ( res == APDS9930_SUCCESS )
	{
		obj->ppcount = ppcount;
	}

	return res;
}

static int apds9930_set_control ( struct i2c_client *client, int control )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_byte ( client, CMD_BYTE | APDS9930_CONTROL_REG, ( u8 ) control );
	if ( res == APDS9930_SUCCESS )
	{
		obj->control = control;
	}

	return res;
}

static int apds9930_get_status ( struct i2c_client *client, int *pData )
{
	int res = 0;

	res = apds9930_read_byte ( client, CMD_BYTE | APDS9930_STATUS_REG, ( u8 * ) pData );
	if ( res == APDS9930_SUCCESS )
	{
		SENSOR_LOG ( "STATUS=0x%02x\n", ( u8 ) * pData );
	}

	return res;
}

static int apds9930_get_pdata ( struct i2c_client *client, int *pData )
{
	int res = 0;

	res = apds9930_read_word ( client, CMD_WORD | APDS9930_PDATAL_REG, ( u16 * ) pData );
	if ( res == APDS9930_SUCCESS )
	{
		SENSOR_LOG ( "PDATA=0x%04x\n", ( u16 ) * pData );
	}

	return res;
}

//LGE_CHANGE_S 2013-06-28 chulho.park@lge.com add at%proximity=3
static int apds9930_get_deivceid( struct i2c_client *client, int *pData )
{
	int res = 0;

	res = apds9930_read_byte ( client, CMD_BYTE | APDS9930_ID_REG, ( u8* ) pData );
	if ( res == APDS9930_SUCCESS )
	{
		SENSOR_LOG ( "DEVICEID=0x%02x\n", ( u8 ) * pData );
	}

	return res;
}
//LGE_CHANGE_E 2013-06-28 chulho.park@lge.com add at%proximity=3


static int apds9930_clear_interrupt ( struct i2c_client *client, int cmd ) //edit : HN
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	if (cmd == 0)
	{
		res = apds9930_write_cmd ( client, ( u8 )CMD_CLR_PS_INT  );
	}
	else if (cmd == 1)
	{
		res = apds9930_write_cmd ( client, ( u8 )CMD_CLR_ALS_INT  );
	}
	else
	{
		res = apds9930_write_cmd ( client, ( u8 )CMD_CLR_PS_ALS_INT  );
	}
	if ( res == APDS9930_SUCCESS )
	{
		SENSOR_DBG ( "APDS9930 interrupte was cleared\n" );
	}

	return res;
}

/* Add for ALS */
static int apds9930_get_cdata ( struct i2c_client *client, int *cData )
{
	int res = 0;

	res = apds9930_read_word ( client, CMD_WORD | APDS9930_CDATA0L_REG, ( u16 * ) cData );
	if ( res == APDS9930_SUCCESS )
	{
		SENSOR_LOG ( "CDATA0=0x%04x\n", ( u16 ) * cData );
	}

	return res;
}

static int apds9930_set_ailt ( struct i2c_client *client, int threshold )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_word ( client, CMD_WORD | APDS9930_AILTL_REG, ( u16 ) threshold );
	if ( res == APDS9930_SUCCESS )
	{
		obj->ailt = threshold;
	}
	return res;
}

static int apds9930_set_aiht ( struct i2c_client *client, int threshold )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_word ( client, CMD_WORD | APDS9930_AIHTL_REG, ( u16 ) threshold );
	if ( res == APDS9930_SUCCESS )
	{
		obj->aiht = threshold;
	}
	return res;
}

static int apds9930_set_atime ( struct i2c_client *client, int atime )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = apds9930_write_byte ( client, CMD_BYTE | APDS9930_ATIME_REG, ( u8 ) atime );
	if ( res == APDS9930_SUCCESS )
	{
		obj->atime = atime;
	}
	return res;
}

//==========================================================
// APDS9930 Data Processign Funtions
//==========================================================
static int apds9930_decide_ps_state ( struct i2c_client *client, int pdata, int int_status )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int ps_status = obj->ps_status;

	if ( obj->ps_status == PS_FAR )
	{
	    /* Even saturation bit is set by apds9930, pdata is correctly update. So Do not check saturation bit */
		if ( ( pdata >= obj->ps_th_far_high ) && ( ( int_status & APDS9930_PSAT ) != APDS9930_PSAT ) )
		{
			ps_status = PS_NEAR;
			SENSOR_LOG ( "PS = NEAR\n" );
		}
		else
		{
			SENSOR_ERR ( "Unknown Event State\n" );
		}
	}
	else
	{
		if ( pdata <= obj->ps_th_near_low )
		{
			ps_status = PS_FAR;
			SENSOR_LOG ( "PS = FAR\n" );
		}
		else
		{
			SENSOR_ERR ( "Unknown Event State\n" );
		}
	}

	return ps_status;
	
}

static long apds9930_initialize ( struct i2c_client *client  )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
	int id = 0;
	SENSOR_FUN(f);
	res = apds9930_read_byte ( client, CMD_BYTE | APDS9930_ID_REG, ( u8 * ) &id );
	if ( res != APDS9930_SUCCESS )
	{
		SENSOR_ERR ( "failed to read Device ID and it means I2C error happened\n" );
		return res;
	}
	 
	SENSOR_LOG ( "APDS9930 Device ID = 0x%02x\n", ( u8 ) id );

	obj->activate = 0;
	obj->als_activate = 0;

	/* disable proximity */
	apds9930_set_enable ( client, 0 );

	/* initialize registers of proximity */
	apds9930_set_atime ( client, 0xED ); /*add for als : HN / 0x6d=400ms , 0xDB=100ms, 0xED=50ms als  integration time*/
	apds9930_set_ptime ( client, 0xFF ); /* 2.72ms Prox integration time */
	apds9930_set_wtime ( client, 0xDC ); /* 100ms Wait time / 0xED=50ms, 0xDC=100ms */
	//apds9930_set_pers ( client, 0x20|0x02 ); /* 2 consecutive Interrupt persistence */
	apds9930_set_pers ( client, 0x00|0x00 ); /* Force PS interrupt every PS conversion cycle to get the first interrupt */
	apds9930_set_config ( client, 0 );

#if defined(TARGET_S6)
    apds9930_set_ppcount ( client, 0x0c ); /* 10-Pulse for proximity */
	apds9930_set_control ( client, 0x24 ); /* 50mA, IR-diode, 2X PGAIN */
#else
	apds9930_set_ppcount ( client, 0x08 ); /* 8-Pulse for proximity */
	apds9930_set_control ( client, 0x24|0x00 ); /* 100mA, IR-diode, 2X PGAIN */ /* add 1X AGAIN for als */
#endif

	/* crosstalk value shall be set by LGP Server using I/O so init here to 150 */
    obj->ps_cross_talk = 150;

	/* initialize threshold value of PS */
	if ( obj->ps_cross_talk > 870 )
	{
		obj->ps_cross_talk = 870;
	}
	if ( obj->ps_cross_talk < 0 )
	{
		obj->ps_cross_talk = 0;
	}

	obj->ps_th_far_low = 0;
	obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;
	obj->ps_th_near_high = 1023;

	/* Add for ALS : initial temp value(by L70+) */
	obj->als_th_dark_low = 0;
	obj->als_th_dark_high = 15000;
	obj->als_th_bright_low = 9000;
	obj->als_th_bright_high = 65535;

	return res;

}

static long apds9930_enable ( struct i2c_client *client  )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
    hwm_sensor_data sensor_data;
	int res = 0;
	int status = 0;
	int pdata = 0;
	int cdata = 0;
	SENSOR_FUN(f);
	/* enable ADC but block interrupt : both ps and als */
	apds9930_set_enable ( client, 0x0D | 0x02 );

	mdelay ( 200 );//fix delay time : for als

	apds9930_get_status( client, &status);

	if ( ( status & APDS9930_PVALID ) == APDS9930_PVALID )
	{
		/* read sensor data */
		apds9930_get_pdata ( client, &pdata );

		/* decide current PS threshold state and set PS thershold to proper range */
		if ( pdata >= obj->ps_th_far_high )
		{
			obj->ps_th_status = PS_NEAR;
			apds9930_set_pilt ( client, obj->ps_th_near_low );
			apds9930_set_piht ( client, obj->ps_th_near_high );
			SENSOR_LOG ( "PS_TH=NEAR\n" );
		}
		else
		{
			obj->ps_th_status = PS_FAR;
			apds9930_set_pilt ( client, obj->ps_th_far_low );
			apds9930_set_piht ( client, obj->ps_th_far_high );
			SENSOR_LOG ( "PS_TH=FAR\n" );
		}

		/* decide current PS status */
		if ( ( pdata >= obj->ps_th_far_high ) && ( (status&0x40) != 0x40 ) )
		{
			obj->ps_status = PS_NEAR;
			SENSOR_LOG ( "Enable PS Status=NEAR\n" );
		}
		else
		{
			obj->ps_status = PS_FAR;
			SENSOR_LOG ( "Enable PS Status=FAR\n" );
		}

	}
	else
	{
		SENSOR_ERR("ADC value is invalid so set to PS_FAR\n");

		obj->ps_th_status = PS_FAR;
		obj->ps_status = PS_FAR;
		apds9930_set_pilt ( client, obj->ps_th_far_low );
		apds9930_set_piht ( client, obj->ps_th_far_high );
	}

	/* inform to upper layer ( hwmsen ) */
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	sensor_data.value_divide = 1;
	sensor_data.values[0] = obj->ps_status;
	if ( ( res = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ) ) )
	{
		SENSOR_ERR ( "failed to send inform ( err = %d )\n", res );
	}

	if ( ( status & APDS9930_AVALID ) == APDS9930_AVALID )
	{
		/* read sensor data */
		apds9930_get_cdata ( client, &cdata );

		/* decide current ALS threshold state and set ALS thershold to proper range */
		if ( cdata >= obj->als_th_dark_high )
		{
			obj->als_th_status = ALS_BRIGHT;
			apds9930_set_ailt ( client, obj->als_th_bright_low );
			apds9930_set_aiht ( client, obj->als_th_bright_high );
			SENSOR_LOG ( "ALS_TH=BRIGHT\n" );
		}
		else
		{
			obj->als_th_status = ALS_DARK;
			apds9930_set_ailt ( client, obj->als_th_dark_low );
			apds9930_set_aiht ( client, obj->als_th_dark_high );
			SENSOR_LOG ( "ALS_TH=DARK\n" );
		}

		/* decide current ALS status */
		if ( cdata >= obj->als_th_dark_high )
		{
			obj->als_status = ALS_BRIGHT;
			SENSOR_LOG ( "Enable ALS Status=BRIGHT\n" );
		}
		else
		{
			obj->als_status = ALS_DARK;
			SENSOR_LOG ( "Enable ALS Status=DARK\n" );
		}

	}
	else
	{
		SENSOR_ERR("ADC value is invalid so set to ALS_DARK\n");

		obj->als_th_status = ALS_DARK;
		obj->als_status = ALS_DARK;
		apds9930_set_ailt ( client, obj->als_th_dark_low );
		apds9930_set_aiht ( client, obj->als_th_dark_high );
	}

	/* enable APDS9930 */
	res = apds9930_set_enable ( client, 0x2D | 0x12 ); /* Add for ALS */
	if ( res == APDS9930_SUCCESS )
	{
		/* unmask external interrupt */
		mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );
		SENSOR_LOG ( "APDS9930 was enabled\n" );
		
	}
	else
	{
		SENSOR_ERR ( "failed to enable APDS9930\n" );
	}

	return res;
	
}

static long apds9930_disable ( struct i2c_client *client  )
{
	int res = 0;
	
	/* mask external interrupt */
	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );

	/* disable APDS9930 */
	res = apds9930_set_enable ( client, 0 );
	if ( res == APDS9930_SUCCESS )
	{
		SENSOR_LOG ( "APDS9930 was disabled\n" );
	}
	else
	{
		SENSOR_ERR ( "failed to disable APDS9930\n" );
	}

	return res;
	
}

void apds9930_swap(int *x, int *y)
{
     int temp = *x;
     *x = *y;
     *y = temp;
}

static int apds9930_do_calibration ( struct i2c_client *client, int *value )
{
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	unsigned int sum_of_pdata = 0;
	int temp_pdata[20] = {0};
	int temp_state[20] = {0};
	unsigned int i=0;
	unsigned int j=0;
	unsigned int ArySize = 20;
	unsigned int cal_check_flag = 0;
	unsigned int old_enable = 0;
	SENSOR_FUN(f);
	//apds9930_led_power ( 1 ); //not use
	old_enable = obj->enable;

RE_CALIBRATION:
	sum_of_pdata = 0;

	/* Enable PS and Mask interrupt */
	apds9930_set_enable ( client, 0x0D ); /* Add for ALS */

	mdelay ( 50 );

	/* Read pdata */
	for ( i = 0 ; i < 20 ; i++ )
	{
		apds9930_get_status ( client, &( temp_state[i] ) );
		apds9930_get_pdata ( client, &( temp_pdata[i] ) );
		mdelay ( 6 );
	}

	#if defined ( DEBUG )
	SENSOR_LOG ( "State Value = " );
	for ( i = 0 ; i < 20 ; i++ )
	{
		SENSOR_LOG ( "%d ", temp_state[i] );
	}
	SENSOR_LOG ( "\n" );
	SENSOR_LOG ( "Read Value = " );
	for ( i = 0 ; i < 20 ; i++ )
	{
		SENSOR_LOG ( "%d ", temp_pdata[i] );
	}
	SENSOR_LOG ( "\n" );
	#endif

	/* sort pdata */
	for ( i = 0 ; i < ArySize - 1 ; i++ )
	{
		for ( j = i + 1 ; j < ArySize ; j++ )
		{
			if ( temp_pdata[i] > temp_pdata[j] )
			{
				apds9930_swap ( temp_pdata+i, temp_pdata+j );
			}
		}
	}

	#if defined(DEBUG)
#if 1 /* branden.you@lge.com_20120924 */
	SENSOR_LOG ( "Read Value = " );
	for ( i = 0 ; i < 20 ; i++ )
	{
		SENSOR_LOG ( "%d ", temp_pdata[i] );
	}
	SENSOR_LOG ( "\n" );
#else
	SENSOR_DBG ( "Read Value = " );
	for ( i = 0 ; i < 20 ; i++ )
	{
		SENSOR_DBG ( "%d ", temp_pdata[i] );
	}
	SENSOR_DBG ( "\n" );
#endif
	#endif

	/* take ten middle data only */
	for ( i = 5 ; i < 15 ; i++ )
	{
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	/* calculate average */
	obj->ps_cross_talk = sum_of_pdata / 10;
	SENSOR_LOG ( "New calibrated cross talk = %d\n", obj->ps_cross_talk );

	/* check if average is acceptable */
	if ( obj->ps_cross_talk > 870 )
	{
		if ( cal_check_flag == 0 )
		{
			cal_check_flag = 1;
			goto RE_CALIBRATION;
		}
		else
		{
			SENSOR_ERR ( "failed to calibrate cross talk/n" );
			apds9930_set_enable ( client, 0x00 );
			apds9930_set_enable ( client, old_enable );
			*value = obj->ps_cross_talk;
			return -1;
		}
	}

	apds9930_set_enable ( client, 0x00 ); /* Power Off */
	apds9930_set_enable ( client, old_enable );
	//apds9930_led_power ( 0 );  //not use

	obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;

	/* we should store it to storage ( it should be free from factory reset ) but ATCI Demon will store it through LGP Demon */

	*value = obj->ps_cross_talk;
	SENSOR_LOG("calibration Finish\n");
	return 0;
}

//==========================================================
// APDS9930 General Control Funtions
//==========================================================
static long apds9930_activate ( struct i2c_client *client, int enable )
{
//	SENSOR_FUN ();
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	long res = 0;

	if ( obj->activate != enable )
	{
		if ( enable )
		{
			//apds9930_led_power ( 1 );  //not use
			SENSOR_LOG("enable\n");
			res = apds9930_enable ( client );
			if ( res == APDS9930_SUCCESS )
			{
				SENSOR_LOG ( "APDS9930 was enabled\n" );
			}
			else
			{
				SENSOR_ERR ( "failed to enable APDS9930\n" );
			}
		}
		else
		{
			SENSOR_LOG("disable\n");
			res = apds9930_disable ( client );
			if ( res == APDS9930_SUCCESS )
			{
				SENSOR_LOG ( "APDS9930 was disabled\n" );
				//apds9930_led_power ( 0 ); //not use
			}
			else
			{
				SENSOR_ERR ( "failed to disable APDS9930\n" );
			}
		}

		if ( res == APDS9930_SUCCESS )
		{
			obj->activate = enable;
		}
		
	}

	return res;
	
}

//==========================================================
// APDS9930 Interrupt Service Routines
//==========================================================
void apds9930_eint_func ( void )
{
	SENSOR_FUN ();
	struct apds9930_priv *obj = g_apds9930_ptr;
	if ( !obj )
	{
		return;
	}
	schedule_work ( &obj->eint_work );
}
static void apds9930_change_ps_threshold(struct i2c_client *client, int int_status)
{
	SENSOR_FUN ();
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	hwm_sensor_data sensor_data;
	int pdata = 0;
	int new_ps_status = 0;
	int err = 0;

	apds9930_set_pers ( client, 0x20|0x02 ); /* 2 consecutive Interrupt persistence */ /* Add for ALS */
	/*repeat this because of the first interrupt forced */

	/* read sensor data */
	apds9930_get_pdata ( client, &pdata );

	/* change threshold to avoid frequent interrupt */
	if ( obj->ps_th_status == PS_FAR )
	{
		if ( pdata >= obj->ps_th_far_high )
		{
			SENSOR_LOG ( "PS_TH = NEAR\n" );
			obj->ps_th_status = PS_NEAR;
			apds9930_set_pilt ( client, obj->ps_th_near_low );
			apds9930_set_piht ( client, obj->ps_th_near_high );
		}
	}
	else
	{
		if ( pdata <= obj->ps_th_near_low )
		{
			SENSOR_LOG ( "PS_TH = FAR\n" );
			obj->ps_th_status = PS_FAR;
			apds9930_set_pilt ( client, obj->ps_th_far_low );
			apds9930_set_piht ( client, obj->ps_th_far_high );
		}
	}
	
	/* make a decision if it is near or far */
	new_ps_status = apds9930_decide_ps_state( client, pdata, int_status );
	
	/* inform to upper layer ( hwmsen ), if status was changed */
	if ( new_ps_status != obj->ps_status )
	{
		obj->ps_status = new_ps_status;
		
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		sensor_data.value_divide = 1;
		sensor_data.values[0] = obj->ps_status;
		if ( ( err = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ) ) )
		{
			SENSOR_ERR ( "failed to send inform ( err = %d )\n", err );
		}
		SENSOR_LOG ( "inform to upper layer(to hwmsen driver)\n" );
	}
}

static void apds9930_change_als_threshold(struct i2c_client *client, int int_status)
{
	SENSOR_FUN ();
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	hwm_sensor_data sensor_data;
	int als_cdata0 = 0;
	int new_als_status = 0;
	int v;
	int err = 0;

	apds9930_set_pers ( client, 0x20|0x02 ); /* 2 consecutive Interrupt persistence */ /* Add for ALS */
	/*repeat this because of the first interrupt forced */

	/* read sensor data */
	apds9930_get_cdata ( client, &als_cdata0 );

	/*
	* check PS under sunlight
	* PS was previously in far-to-near condition
	*/
	v = 1024 * (256 - obj->atime);
	v = (v * 75) / 100;
	if ( (obj->ps_status == 0) && (als_cdata0 > v) )
	{
		/*
		* need to inform input event as there will be no interrupt
		* from the PS
		*/
		/* NEAR-to-FAR detection Process */
		SENSOR_LOG ( "(Enforced)PS_TH = FAR\n" );
		obj->ps_th_status = PS_FAR;
		apds9930_set_pilt ( client, obj->ps_th_far_low );
		apds9930_set_piht ( client, obj->ps_th_far_high );
	
		/* inform to upper layer ( hwmsen ), if status was changed */
		obj->ps_status = 1;/* near-to-far detected */
		
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		sensor_data.value_divide = 1;
		sensor_data.values[0] = obj->ps_status;
		if ( ( err = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ) ) )
		{
			SENSOR_ERR ( "failed to send inform ( err = %d )\n", err );
		}
		SENSOR_LOG ( "NEAR-to-FAR enforced. cdata = %d\n",als_cdata0 );
	}
	
	/* change threshold to avoid frequent interrupt */
	if ( obj->als_th_status == ALS_DARK )
	{
		if ( als_cdata0 >= obj->als_th_dark_high )
		{
			SENSOR_LOG ( "ALS_TH = BRIGHT\n" );
			obj->als_th_status = ALS_BRIGHT;
			apds9930_set_ailt ( client, obj->als_th_bright_low );
			apds9930_set_aiht ( client, obj->als_th_bright_high );
		}
	}
	else
	{
		if ( als_cdata0 <= obj->als_th_bright_low )
		{
			SENSOR_LOG ( "ALS_TH = DARK\n" );
			obj->als_th_status = ALS_DARK;
			apds9930_set_ailt ( client, obj->als_th_dark_low );
			apds9930_set_aiht ( client, obj->als_th_dark_high );
		}
	}
	
	/* make a decision if it is near or far */
	if ( obj->als_status == ALS_DARK )
	{
		if ( als_cdata0 >= obj->als_th_dark_high )
		{
			SENSOR_LOG ( "ALS = BRIGHT\n" );
			new_als_status = ALS_BRIGHT;
			if(apds9930_lux_change_cb)
				apds9930_lux_change_cb(1);
		}
		else
		{
			SENSOR_ERR ( "Unknown Event State\n" );
		}
	}
	else
	{
		if ( als_cdata0 <= obj->als_th_bright_low )
		{
			SENSOR_LOG ( "ALS = DARK\n" );
			new_als_status = ALS_DARK;
			if(apds9930_lux_change_cb)
				apds9930_lux_change_cb(0);
		}
		else
		{
			SENSOR_ERR ( "Unknown Event State\n" );
		}
	}

	#if 0
	/* inform to upper layer ( hwmsen ), if status was changed */
	if ( new_als_status != obj->als_status )
	{
		obj->als_status = new_als_status;
		
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		sensor_data.value_divide = 1;
		sensor_data.values[0] = obj->als_status;
		if ( ( err = hwmsen_get_interrupt_data ( ID_LIGHT, &sensor_data ) ) )//must check ??ID_LIGHT : HN
		{
			SENSOR_ERR ( "failed to send inform ( err = %d )\n", err );
		}
	}
	#endif
}

static void apds9930_eint_work ( struct work_struct *work )
{
	SENSOR_FUN ();
	struct apds9930_priv *obj = ( struct apds9930_priv * ) container_of ( work, struct apds9930_priv, eint_work );
	struct i2c_client *client = obj->client;
	hwm_sensor_data sensor_data;

	int err;
	int int_status = 0;

	/* read status register */
	apds9930_get_status ( client, &int_status );

	/* disable ADC first */
	apds9930_set_enable ( client, 0x01 );

	if ( ( int_status & APDS9930_PAINT ) == APDS9930_PAINT )
	{
		SENSOR_LOG ( "Both PS, ALS interrupt happened\n" );
		if ( ( int_status & APDS9930_PAVALID ) != ( APDS9930_PAVALID ) )
		{
			SENSOR_ERR("ADC value is not valid so just skip this interrupt");
			/* clear interrupt of proximity */
			apds9930_clear_interrupt ( client , 2 ); /* Add for ALS - add command for als clr */
			goto CLEAR_INTERRUPT;
		}
		apds9930_change_ps_threshold(client, int_status);
		apds9930_change_als_threshold(client, int_status);
		/* clear interrupt of proximity */
		apds9930_clear_interrupt ( client , 2 );
	}
	else if ( ( int_status & APDS9930_PINT ) == APDS9930_PINT )
	{
		SENSOR_LOG ( "PS interrupt happened\n" );
		/* move to here. only check pvalid, when ps interrupt */
		if ( ( int_status & APDS9930_PVALID ) != ( APDS9930_PVALID ) )
		{
			SENSOR_ERR("ADC value is not valid so just skip this interrupt");
			apds9930_clear_interrupt ( client , 0 );
			goto CLEAR_INTERRUPT;
		}
		apds9930_change_ps_threshold(client, int_status);
		apds9930_clear_interrupt ( client , 0 );
	}
	else if ( ( int_status & APDS9930_AINT ) == APDS9930_AINT )
	{
		SENSOR_LOG ( "ALS interrupt happened\n" );
		if ( ( int_status & APDS9930_AVALID ) != ( APDS9930_AVALID ) )
		{
			SENSOR_ERR("ADC value is not valid so just skip this interrupt");
			apds9930_clear_interrupt ( client , 1 );
			goto CLEAR_INTERRUPT;
		}
		apds9930_change_als_threshold(client, int_status);
		apds9930_clear_interrupt ( client , 1 );
	}

CLEAR_INTERRUPT:	
	/* unmask external interrupt */
   	mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );

	/* activate proximity */
	apds9930_set_enable ( client, 0x2D | 0x12 ); /* Add for ALS */
}

//==========================================================
// APDS9930 ADB Shell command function
//==========================================================
static ssize_t apds9930_show_cali_value ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );
	SENSOR_LOG("show calibration value : %d\n",data->ps_cross_talk);
	return sprintf ( buf, "%u\n", data->ps_cross_talk );
}

static ssize_t apds9930_store_cali_value ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	int ret;
	int data;
	SENSOR_FUN();
	ret = apds9930_do_calibration ( client, &data );

	return count;
}

static ssize_t apds9930_show_ptime ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->ptime );
}

static ssize_t apds9930_store_ptime ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9930_set_ptime ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9930_show_wtime ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->wtime );
}

static ssize_t apds9930_store_wtime ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9930_set_wtime ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9930_show_pilt ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->pilt );
}

static ssize_t apds9930_store_pilt ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9930_set_pilt ( client, val );

	if ( ret < 0 )
		return ret;

	obj->ps_th_near_low = val;

	return count;
}

static ssize_t apds9930_show_piht ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->piht );
}

static ssize_t apds9930_store_piht ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9930_set_piht ( client, val );

	if ( ret < 0 )
		return ret;

	obj->ps_th_far_high = val;

	return count;
}

static ssize_t apds9930_show_pers ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->pers );
}

static ssize_t apds9930_store_pers ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9930_set_pers ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9930_show_config ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->config );
}

static ssize_t apds9930_store_config ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9930_set_config ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9930_show_ppcount ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->ppcount );
}

static ssize_t apds9930_store_ppcount ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9930_set_ppcount ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9930_show_control ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->control );
}

static ssize_t apds9930_store_control ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9930_set_control ( client, val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9930_show_status ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = apds9930_i2c_client;
    int status = 0;

    apds9930_get_status ( client, &status );

    return sprintf ( buf, "0x%02x\n", status );
}

static ssize_t apds9930_show_pdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = apds9930_i2c_client;
    int data = 0;

    apds9930_get_pdata ( client, &data );

    return sprintf ( buf, "%d\n", data );
}

//LGE_CHANGE_S 2013-06-28 chulho.park@lge.com add at%proximity=3
static ssize_t apds9930_show_deviceid ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = apds9930_i2c_client;
    int data = 0;

    apds9930_get_deivceid ( client, &data );

    return sprintf ( buf, "%02x\n", ( u8 )data );
}
//LGE_CHANGE_E 2013-06-28 chulho.park@lge.com add at%proximity=3

static ssize_t apds9930_show_target_pdata ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );
	SENSOR_LOG("show Target_Pdata val : %d\n",Target_Pdata);
	return sprintf ( buf, "%d\n", Target_Pdata);
}

static ssize_t apds9930_store_target_pdata ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;
//	SENSOR_LOG("store Target_Pdata val : %d\n",val);
	Target_Pdata = val;
	obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;
/*
	ret = apds9930_set_piht ( client, obj->ps_th_far_high );
	ret = apds9930_set_pilt ( client, obj->ps_th_near_low );
*/
	/* change threshold to avoid frequent interrupt */
	if ( obj->ps_th_status == PS_NEAR )
	{
		SENSOR_LOG ( "Set PS_TH = NEAR\n" );
		ret = apds9930_set_pilt ( client, obj->ps_th_near_low );
		ret = apds9930_set_piht ( client, obj->ps_th_near_high );
	}
	else
	{
		SENSOR_LOG ( "Set PS_TH = FAR\n" );
		ret = apds9930_set_pilt ( client, obj->ps_th_far_low );
		ret = apds9930_set_piht ( client, obj->ps_th_far_high );
	}
	if ( ret < 0 )
		return ret;
	
	return count;
}

static ssize_t apds9930_show_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

    switch(data->activate) {
          case 0:
         	   return sprintf ( buf, "%s\n", "Proximity Disabled");
          case 1:
               return sprintf ( buf, "%s\n", "Proximity Enabled" );

           default:
               return sprintf ( buf, "%s\n", "Proximity Error" );
     	}
    
}

static ssize_t apds9930_store_enable ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret=0;

    switch(val) {
          case 0:
            ret = apds9930_activate ( client, 0 );
            break;
          case 1:
            ret = apds9930_activate ( client, 1 );
            break;

           default:
           	break;
     	}
      
	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t apds9930_show_als_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

    switch(data->als_activate) {
          case 0:
         	   return sprintf ( buf, "%s\n", "Light Sensor Disabled");
          case 1:
               return sprintf ( buf, "%s\n", "Light Sensor Enabled" );

           default:
               return sprintf ( buf, "%s\n", "Light Sensor Error" );
     	}
    
}

static ssize_t apds9930_store_als_enable ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret=0;

    switch(val) {
          case 0:
	    if( data->activate == 0 ) {
		ret = APDS9930_SUCCESS;
	    	data->als_activate = 0;
	    }
            else {
		ret = APDS9930_FAIL;
	    }
            break;
          case 1:
	    if( data->activate == 1 ) {
		ret = APDS9930_SUCCESS;
	        data->als_activate = 1;
	    }
	    else {
                ret = APDS9930_FAIL;
	    }
            break;

           default:
           	break;
     	}
      
	return count;
}

/* Add for ALS */
static ssize_t apds9930_show_ailt ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->ailt );
}

static ssize_t apds9930_store_ailt ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9930_set_ailt ( client, val );

	if ( ret < 0 )
		return ret;

	obj->als_th_bright_low = val;

	return count;
}

static ssize_t apds9930_show_aiht ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->aiht );
}

static ssize_t apds9930_store_aiht ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = apds9930_i2c_client;
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = apds9930_set_aiht ( client, val );

	if ( ret < 0 )
		return ret;

	obj->als_th_dark_high = val;

	return count;
}

static ssize_t apds9930_show_cdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = apds9930_i2c_client;
    int data = 0;

    apds9930_get_cdata ( client, &data );

    return sprintf ( buf, "%d\n", data );
}

static DRIVER_ATTR ( cali, S_IWUSR | S_IRUGO, apds9930_show_cali_value, apds9930_store_cali_value );
static DRIVER_ATTR ( ptime, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_ptime, apds9930_store_ptime );
static DRIVER_ATTR ( wtime, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_wtime, apds9930_store_wtime );
static DRIVER_ATTR ( pilt, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_pilt, apds9930_store_pilt );
static DRIVER_ATTR ( piht, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_piht, apds9930_store_piht );
static DRIVER_ATTR ( pers, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_pers, apds9930_store_pers );
static DRIVER_ATTR ( config, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_config, apds9930_store_config );
static DRIVER_ATTR ( ppcount, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_ppcount, apds9930_store_ppcount );
static DRIVER_ATTR ( control, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_control, apds9930_store_control );
static DRIVER_ATTR ( status, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_status, NULL );
static DRIVER_ATTR ( pdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_pdata, NULL );
//LGE_CHANGE_S 2013-06-28 chulho.park@lge.com add at%proximity=3
static DRIVER_ATTR ( deviceid, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_deviceid, NULL );  //chulho.park
//LGE_CHANGE_E 2013-06-28 chulho.park@lge.com add at%proximity=3
static DRIVER_ATTR ( target_pdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_target_pdata, apds9930_store_target_pdata );
static DRIVER_ATTR ( enable, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_enable, apds9930_store_enable );
static DRIVER_ATTR ( als_enable, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_als_enable, apds9930_store_als_enable );
static DRIVER_ATTR ( ailt, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_ailt, apds9930_store_ailt );
static DRIVER_ATTR ( aiht, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_aiht, apds9930_store_aiht );
static DRIVER_ATTR ( alsdata0, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, apds9930_show_cdata, NULL );


static struct driver_attribute *apds9930_attr_list[] = {
	&driver_attr_cali,		   /*show calibration data*/
	&driver_attr_ptime,
	&driver_attr_wtime,
	&driver_attr_pilt,
	&driver_attr_piht,
	&driver_attr_pers,
	&driver_attr_config,
	&driver_attr_ppcount,
	&driver_attr_control,
	&driver_attr_status,
	&driver_attr_pdata,
//LGE_CHANGE_S 2013-06-28 chulho.park@lge.com add at%proximity=3
	&driver_attr_deviceid,
//LGE_CHANGE_E 2013-06-28 chulho.park@lge.com add at%proximity=3
	&driver_attr_target_pdata,
	&driver_attr_enable,
	&driver_attr_als_enable,
	&driver_attr_ailt,
	&driver_attr_aiht,
	&driver_attr_alsdata0,
};

static int apds9930_create_attr ( struct device_driver *driver )
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( apds9930_attr_list ) / sizeof ( apds9930_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		if ( err = driver_create_file ( driver, apds9930_attr_list[idx] ) )
		{
			SENSOR_ERR ( "driver_create_file (%s) = %d\n", apds9930_attr_list[idx]->attr.name, err );
			break;
		}
	}

	return err;
}

static int apds9930_delete_attr ( struct device_driver *driver )
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( apds9930_attr_list ) / sizeof ( apds9930_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		driver_remove_file ( driver, apds9930_attr_list[idx] );
	}

	return err;
}

//==========================================================
// APDS9930 Service APIs ( based on File I/O )
//==========================================================
static int apds9930_open ( struct inode *inode, struct file *file )
{
	SENSOR_FUN ();
	file->private_data = apds9930_i2c_client;

	if ( !file->private_data )
	{
		SENSOR_ERR ( "Invalid input paramerter\n" );
		return -EINVAL;
	}

	return nonseekable_open ( inode, file );
}

static int apds9930_release ( struct inode *inode, struct file *file )
{
	SENSOR_FUN ();
	file->private_data = NULL;
	return 0;
}

static long apds9930_unlocked_ioctl ( struct file *file, unsigned int cmd, unsigned long arg )
{
	//SENSOR_FUN ();
	struct i2c_client *client = ( struct i2c_client * ) file->private_data;
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	long err = 0;
	void __user *ptr = ( void __user * ) arg;
	int dat;
	uint32_t enable;
	uint32_t crosstalk = 0;

	switch ( cmd )
	{
		case ALSPS_SET_PS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_SET_PS_MODE\n" );
			if ( copy_from_user ( &enable, ptr, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			if ( enable )
			{
				if ( ( err = apds9930_activate ( obj->client, 1 ) ) )
				{
					SENSOR_ERR ( "failed to activate APDS9930 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			else
			{
				if ( ( err = apds9930_activate ( obj->client, 0 ) ) )
				{
					SENSOR_ERR ( "failed to deactivate APDS9930 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			break;

		case ALSPS_GET_PS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_MODE\n" );
			enable = obj->activate;
			if ( copy_to_user ( ptr, &enable, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_DATA\n" );
			dat = obj->ps_status;
			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_RAW_DATA\n" );
			if ( err = apds9930_get_pdata ( obj->client, &dat ) )
			{
				goto err_out;
			}

			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
			
		case ALSPS_GET_CALI:
			SENSOR_LOG ( "CMD = ALSPS_GET_CALI\n" );
			err = apds9930_do_calibration ( obj->client, &dat );
			if ( err == 0 )
			{
				if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
				{
					err = -EFAULT;
					goto err_out;
				}
			}
			break;

		case ALSPS_SET_CALI:
            if ( copy_from_user ( &crosstalk, ptr, sizeof ( crosstalk ) ) )
            {
                err = -EFAULT;
                goto err_out;
            }
            SENSOR_LOG ( "CMD = ALSPS_SET_CALI crosstalk value : %d\n",crosstalk );
			if ( ( crosstalk == 0x0000FFFF ) || ( crosstalk == 0 ) )
			{
				obj->ps_cross_talk = 150;
			}
			else
			{
				obj->ps_cross_talk = crosstalk;
			}

			obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
		    obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;
            break;

//LGE_CHANGE_S 2013-06-28 chulho.park@lge.com add at%proximity=3
		case ALSPS_GET_DEVICEID:
            SENSOR_LOG ( "CMD = ALSPS_GET_DEVICEID\n" );
			if ( err = apds9930_get_deivceid ( obj->client, &dat ) )
			{
				goto err_out;
			}

			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
//LGE_CHANGE_E 2013-06-28 chulho.park@lge.com add at%proximity=3
		default:
			SENSOR_ERR ( "Invalid Command = 0x%04x\n", cmd );
			err = -ENOIOCTLCMD;
			break;
	}

	err_out : return err;
}

static struct file_operations apds9930_fops = { .owner = THIS_MODULE, .open = apds9930_open, .release = apds9930_release,
												.unlocked_ioctl = apds9930_unlocked_ioctl,  };

static struct miscdevice apds9930_device = { .minor = MISC_DYNAMIC_MINOR, .name = "als_ps", .fops = &apds9930_fops,  };

//==========================================================
// APDS9930 Service APIs ( based on hwmsen Interface )
//==========================================================
static int apds9930_ps_operate ( void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout )
{
	//SENSOR_FUN ();
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data = NULL;
	struct apds9930_priv *obj = ( struct apds9930_priv * ) self;

	switch ( command )
	{
		case SENSOR_DELAY:
			SENSOR_LOG ( "CMD = SENSOR_DELAY\n" );
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				value = *( int * ) buff_in;
				if ( value )
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Enable )\n" );
					if ( err = apds9930_activate ( obj->client, 1 ) )
					{
						SENSOR_ERR ( "failed to activate APDS9930 ( err = %d )\n", err );
						return -1;
					}
				}
				else
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Disable )\n" );
					if ( err = apds9930_activate ( obj->client, 0 ) )
					{
						SENSOR_ERR ( "failed to deactivate APDS9930 ( err = %d )\n", err );
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			SENSOR_LOG ( "CMD = SENSOR_GET_DATA\n" );
			if ( ( buff_out == NULL ) || ( size_out < sizeof ( hwm_sensor_data ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				/* sumeet.sahu@lge.com added for avoiding data abort */
				sensor_data = (hwm_sensor_data *)buff_out;
				sensor_data->values[0] = obj->ps_status;
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			SENSOR_ERR ( "Invalid Command = %d\n", command );
			err = -1;
			break;
	}

	return err;
}

//==========================================================
// APDS9930 Initialization related Routines
//==========================================================
static int apds9930_init_client ( struct i2c_client *client )
{
	SENSOR_FUN ();
	struct apds9930_priv *obj = i2c_get_clientdata ( client );
	int err = 0;

	err = apds9930_initialize ( client );
	if ( err != APDS9930_SUCCESS )
	{
		SENSOR_ERR ( "failed to init APDS9930\n" );
	}

	return err;
}

/****************************************************************************
* I2C BUS Related Functions
****************************************************************************/

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void apds9930_early_suspend ( struct early_suspend *h )
{
	SENSOR_FUN ();
}

static void apds9930_late_resume ( struct early_suspend *h )
{
	SENSOR_FUN ();
}
#endif

static int apds9930_i2c_probe ( struct i2c_client *client, const struct i2c_device_id *id )
{
	struct apds9930_priv *obj;
	struct hwmsen_object obj_ps;
	struct alsps_hw *hw = get_cust_alsps_hw ();
	int err = 0;

	SENSOR_FUN();
	if ( !( obj = kzalloc ( sizeof ( *obj ), GFP_KERNEL ) ) )
	{
		err = -ENOMEM;
		goto exit;
	}
	memset ( obj, 0, sizeof ( *obj ) );

	obj->client = client;
	i2c_set_clientdata ( client, obj );

	g_apds9930_ptr = obj;
	apds9930_i2c_client = client;

	INIT_WORK ( &obj->eint_work, apds9930_eint_work );

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend = apds9930_early_suspend,
	obj->early_drv.resume = apds9930_late_resume,
	register_early_suspend ( &obj->early_drv );
	#endif

	/* Initialize APDS9930 */ 
	if ( err = apds9930_init_client ( client ) )
	{
		SENSOR_ERR ( "failed to init APDS9930 ( err = %d )\n", err );
		goto exit_init_failed;
	}

	/* Register APDS9930 as a misc device for general I/O interface */
	if ( err = misc_register ( &apds9930_device ) )
	{
		SENSOR_ERR ( "failed to register misc device ( err = %d )\n", err );
		goto exit_misc_device_register_failed;
	}

	if ( err = apds9930_create_attr ( &apds9930_alsps_driver.driver ) )
	{
		SENSOR_ERR ( "create attribute err = %d\n", err );
		goto exit_create_attr_failed;
	}

	/* Register APDS9930 as a member device of hwmsen */
	obj_ps.self = obj;
	obj_ps.polling = hw->polling_mode_ps;
	obj_ps.sensor_operate = apds9930_ps_operate;
	if ( err = hwmsen_attach ( ID_PROXIMITY, &obj_ps ) )
	{
		SENSOR_ERR ( "failed to attach to hwmsen ( err = %d )\n", err );
		goto exit_create_attr_failed;
	}

	SENSOR_LOG("OK\n");
	return 0;

	exit_create_attr_failed:
	misc_deregister ( &apds9930_device );
	exit_misc_device_register_failed:
	exit_init_failed:
	unregister_early_suspend ( &obj->early_drv );
	kfree ( obj );
	exit:
	apds9930_i2c_client = NULL;
	SENSOR_ERR ( "Err = %d\n", err );
	return err;
}

static int apds9930_i2c_remove ( struct i2c_client *client )
{
	SENSOR_FUN ();
	int err;

	if ( err = apds9930_delete_attr ( &apds9930_alsps_driver.driver ) )
	{
		SENSOR_ERR ( "apds9930_delete_attr fail: %d\n", err );
	}

	if ( err = misc_deregister ( &apds9930_device ) )
	{
		SENSOR_ERR ( "failed to deregister misc driver : %d\n", err );
	}

	apds9930_i2c_client = NULL;
	i2c_unregister_device ( client );
	kfree ( i2c_get_clientdata ( client ) );

	return 0;
}

static int apds9930_i2c_suspend ( struct i2c_client *client, pm_message_t msg )
{	
	SENSOR_FUN();
	
	return 0;
}

static int apds9930_i2c_resume ( struct i2c_client *client )
{
	SENSOR_FUN();	
	
	return 0;
}

static int apds9930_i2c_detect ( struct i2c_client *client, struct i2c_board_info *info )
{
	SENSOR_FUN ();
	strcpy ( info->type, APDS9930_DEV_NAME );
	return 0;
}

static const struct i2c_device_id apds9930_i2c_id[] = { { APDS9930_DEV_NAME, 0 }, {} };

static struct i2c_driver apds9930_i2c_driver = { .probe = apds9930_i2c_probe, .remove = apds9930_i2c_remove, .suspend = apds9930_i2c_suspend,
												 .resume = apds9930_i2c_resume, .detect = apds9930_i2c_detect, .id_table = apds9930_i2c_id,
												 .driver = { .name = APDS9930_DEV_NAME, }, };

/****************************************************************************
* Linux Device Driver Related Functions
****************************************************************************/
static int apds9930_probe ( struct platform_device *pdev )
{
	struct alsps_hw *hw = get_cust_alsps_hw ();
	SENSOR_FUN();
	/* Configure external ( GPIO ) interrupt */
	apds9930_setup_eint ();

	/* Turn on the power for APDS9930 */
	//apds9930_main_power ( hw, 1 );
	//msleep ( 9 );
	
	/* Add APDS9930 as I2C driver */
	if ( i2c_add_driver ( &apds9930_i2c_driver ) )
	{
		SENSOR_ERR ( "failed to add i2c driver\n" );
		return -1;
	}
   
	return 0;
}

static int apds9930_remove ( struct platform_device *pdev )
{
	SENSOR_FUN ();
	struct alsps_hw *hw = get_cust_alsps_hw ();

	/* Turn off the power for APDS9930 */
	//apds9930_main_power ( hw, 0 );

	i2c_del_driver ( &apds9930_i2c_driver );

	return 0;
}
static struct i2c_board_info __initdata i2c_APDS9930 = { I2C_BOARD_INFO ( "APDS9930", 0x39 ) };

static struct platform_driver apds9930_alsps_driver = { .probe = apds9930_probe, .remove = apds9930_remove, .driver = { .name = "als_ps", } };

static int __init apds9930_init ( void )
{
	SENSOR_FUN();
	struct alsps_hw *hw = get_cust_alsps_hw ();
	i2c_register_board_info ( hw->i2c_num, &i2c_APDS9930, 1 );
	if ( platform_driver_register ( &apds9930_alsps_driver ) )
	{
		SENSOR_ERR ( "failed to register platform driver\n" );
		return -ENODEV;
	}
	SENSOR_LOG("apds9930_init success/ i2c_number = %d\n", hw->i2c_num);
	return 0;
}

static void __exit apds9930_exit ( void )
{
	SENSOR_FUN ();
	platform_driver_unregister ( &apds9930_alsps_driver );
}


module_init ( apds9930_init );
module_exit ( apds9930_exit );

MODULE_AUTHOR ( "Kang Jun Mo" );
MODULE_DESCRIPTION ( "apds9930 driver" );
MODULE_LICENSE ( "GPL" );

/* End Of File */

