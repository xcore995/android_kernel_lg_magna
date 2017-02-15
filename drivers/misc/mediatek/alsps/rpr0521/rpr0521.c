/******************************************************************************
 * MODULE       : rpr0521.c
 * FUNCTION     : Driver source for RPR-0521,
 *              : Proximity Sensor(PS) and Ambient Light Sensor(ALS) IC.
 * AUTHOR       : Seo Ji Won < jiwon.seo@lge.com >
 * MODIFY       : Masafumi Seike
 * PROGRAMMED   : Sensing solution Group, ROHM CO.,LTD.
 * MODIFICATION : Modified by ROHM, OCT/02/2014
 * REMARKS      : File : mediatek\custom\common\kernel\alsps\rpr0521.c
 * COPYRIGHT    : Copyright (C) 2012 ROHM CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *****************************************************************************/

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

#ifdef MT6582
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#ifdef MT6582
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <cust_gpio_usage.h>
#include <mach/eint.h>

#include "rpr0521.h"
#define RPR0521_I2CADDR    (0x38)


#define CALB_SYSTEM_WAIT     (50)
#define CALB_IC_WAIT         (10)
#define CALB_TIMES           (2)
#define CALB_BOX_TIMES       (20)
#define CALB_REMOVAL_TIME    (5)

#define SET_IC_DISABLE       (0)
#define SET_IC_ENABLE        (1)

#define MOCTL_EN_MASK (0xC0)

#if 1 //jwseo_temp : enable all in case of ps enable
#define MOCTL_ENALS_MASK (0x80)
#define MOCTL_ENPS_MASK (0x40)
#else
#define MOCTL_ENALS_MASK (0xC0)
#define MOCTL_ENPS_MASK (0xC0)
#endif

#define SET_PS_MEASURE_10MS  (0x01)
#define SET_PS_MEASURE_40MS  (0x02)
#define SET_ALS_400_PS_50MS  (0x08)
#define SET_ALS_100_PS_50MS  (0x05)
#define SET_ALS_100_PS_100MS (0x06)
#define ALSGAIN_X1X1         (0x0 << 2)
#define ALSGAIN_X64X64       (0xA << 2)
#define LEDCURRENT_50MA      (1)
#define LEDCURRENT_100MA     (2)
#define PS_THH_BOTH_OUTSIDE  (2 << 4)
#define MODE_PROXIMITY       (1)
#define INTR_PERSIST         (2)
#define INIT_PS_GAIN_X1      (0 << 4)
#define INIT_PS_GAIN_X2      (1 << 4)
#define PWRON_PS             (6)
#define PWRON_ALS            (7)

#define RPR0521_PINT        (1 << 7)

#define AMB_IRFLAG_TOO_HIGH (3 << 6)

#define PS_TH_VAL_MAX          (0x0FFF)
#define PS_TH_VAL_MIN          (0)

#define MOCTL_SELECT_PS     (1 << PWRON_PS)
#define MOCTL_SELECT_ALS    (1 << PWRON_ALS)
#define MOCTL_SELECT_ALL    (MOCTL_ENALS_MASK | MOCTL_ENPS_MASK)
#define POWERON_ALL_ENABLE  (3)

#define COEFFICIENT (4)
const signed long judge_coef[COEFFICIENT] = { 1130, 1424, 1751, 3015};
const signed long data0_coef[COEFFICIENT] = { 6803, 3667, 2054, 1520};
const signed long data1_coef[COEFFICIENT] = { 4714, 1941,  809,  504};

//#define ALS_DECIMAL_POINT (15)
#define ALSCALC_OFFSET    (1000)
#define ALSCALC_GAIN_MASK (0xF << 2)
#define ALS_MAX_VALUE     (43000)
/************ define register for IC ************/
/* RPR0521 REGSTER */
#define REG_MODECONTROL           (0x41)
#define REG_ALSPSCONTROL          (0x42)
#define REG_PSCONTRL              (0x43)
#define REG_PSDATA                (0x44)
#define REG_ALSDATA0              (0x46)
#define REG_ALSDATA1              (0x48)
#define REG_INTERRUPT             (0x4A)
#define REG_PSTH                  (0x4B)
#define REG_PSTL                  (0x4D)
#define REG_MANUFACT_ID           (0x92)

/* Initialization parameter */
//#define INIT_MODE_CONTROL         (SET_PS_MEASURE_40MS)
//#define INIT_MODE_CONTROL         (SET_ALS_400_PS_50MS)
#define INIT_MODE_CONTROL         (SET_ALS_100_PS_100MS)
#define INIT_ALSPSMODE_CONTROL    (LEDCURRENT_50MA | ALSGAIN_X1X1)//with als gain
#define INIT_PSMODE_CONTROL       (INIT_PS_GAIN_X2 | INTR_PERSIST)
#define INIT_INTERRUPT            (PS_THH_BOTH_OUTSIDE | MODE_PROXIMITY)

/* parameter of Gain threshold */
#define GAIN_THRESHOLD            (1000)

static int rpr0521_set_enable ( struct i2c_client *client, unsigned char enable );
static int rpr0521_set_ps_enable ( struct i2c_client *client, unsigned char enable );
static int rpr0521_set_als_enable ( struct i2c_client *client, unsigned char enable );
static int rpr0521_set_pilt ( struct i2c_client *client, unsigned short threshold );
static int rpr0521_set_piht ( struct i2c_client *client, unsigned short threshold );
static int rpr0521_set_control ( struct i2c_client *client, unsigned char control );
static int rpr0521_set_alsps_control ( struct i2c_client *client, unsigned char data );
static int rpr0521_set_ps_control ( struct i2c_client *client, unsigned char data );
static int rpr0521_set_ps_interrupt ( struct i2c_client *client, unsigned char data );
static int rpr0521_get_control ( struct i2c_client *client, unsigned char *pData );
static int rpr0521_get_ps_control ( struct i2c_client *client, unsigned char *pData );
static int rpr0521_get_status ( struct i2c_client *client, unsigned char *pData );
static int rpr0521_get_alsps_control ( struct i2c_client *client, unsigned char *pData );
static int rpr0521_get_pdata ( struct i2c_client *client, unsigned short *pData );
static int rpr0521_get_alsdata0 ( struct i2c_client *client, unsigned short *pData );
static int rpr0521_get_alsdata1 ( struct i2c_client *client, unsigned short *pData );
static int rpr0521_get_deivceid( struct i2c_client *client, unsigned char *pData );
static int rpr0521_clear_interrupt ( struct i2c_client *client );

static int rpr0521_decide_ps_state ( struct i2c_client *client, int pdata, int int_status );
static long rpr0521_initialize ( struct i2c_client *client  );
static long rpr0521_ps_enable ( struct i2c_client *client  );
static long rpr0521_als_enable ( struct i2c_client *client  );
static long rpr0521_ps_disable ( struct i2c_client *client  );
static long rpr0521_als_disable ( struct i2c_client *client  );
void rpr0521_swap(int *x, int *y);
static int rpr0521_do_calibration ( struct i2c_client *client, int *value );
static unsigned int rpr0521_calc_calibration ( struct i2c_client *client );

static long rpr0521_ps_activate ( struct i2c_client *client, int enable );
static long rpr0521_als_activate ( struct i2c_client *client, int enable );
void rpr0521_eint_func ( void );
static void rpr0521_eint_work ( struct work_struct *work );

static ssize_t rpr0521_show_cali_value ( struct device_driver *dev, char *buf );
static ssize_t rpr0521_store_cali_value ( struct device_driver *dev, char *buf, size_t count );
static ssize_t rpr0521_show_pilt ( struct device_driver *dev, char *buf );
static ssize_t rpr0521_store_pilt ( struct device_driver *dev, char *buf, size_t count );
static ssize_t rpr0521_show_piht ( struct device_driver *dev, char *buf );
static ssize_t rpr0521_store_piht ( struct device_driver *dev, char *buf, size_t count );
static ssize_t rpr0521_show_als_poll_delay(struct device_driver *dev, char *buf);
static ssize_t rpr0521_store_als_poll_delay(struct device_driver *dev, const char *buf, size_t count);

static int rpr0521_set_als_poll_delay(struct i2c_client *client, unsigned int val);
static int rpr0521_lux_calculation(struct i2c_client *client);
static int rpr0521_get_alsgain( struct i2c_client *client, unsigned char *raw_gain, unsigned char *gain);
static int rpr0521_set_alsgain( struct i2c_client *client, unsigned char raw_gain, signed long lux);

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define RPR0521_DEV_NAME     "RPR0521"

#define CMD_BYTE    0x80
#define CMD_WORD    0xA0
#define CMD_SPECIAL 0xE0

#define CMD_CLR_PS_INT  0xE5
#define CMD_CLR_ALS_INT 0xE6
#define CMD_CLR_PS_ALS_INT  0xE7
static DEFINE_MUTEX(rpr0521_access);

/****************************************************************************
 * Macros
 ****************************************************************************/
#ifdef CONFIG_MT_ENG_BUILD
#define APS_DEBUG
#endif
#define APS_TAG "[ALS/PS] "
#define APS_ERR(fmt, args...)    printk(KERN_ERR APS_TAG"[ERROR] %s() line=%d : "fmt, __FUNCTION__, __LINE__, ##args)

#if defined ( APS_DEBUG )
/* You need to select proper loglevel to see the log what you want. ( Currently, you can't see "KERN_INFO" level ) */
#define APS_FUN(f)  	   printk(KERN_ERR APS_TAG"%s()\n", __FUNCTION__)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#else
#define APS_FUN(f)  	   printk(KERN_INFO APS_TAG"%s()\n", __FUNCTION__)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#endif

#if defined(TARGET_MT6732_C90)
#define CONFIG_OF_DT
#endif

#ifdef CONFIG_OF_DT
static const struct of_device_id psensor_of_match[] = {
	{ .compatible = "mediatek,als_ps", },
	{},
};
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

struct rpr0521_priv
{
	struct i2c_client *client;
	struct work_struct eint_work;

	unsigned int activate; /* 1 = activate, 0 = deactivate */

	/* variables to store register value - begin */
	unsigned int enable;
	unsigned int pilt;
	unsigned int piht;
	unsigned int alsps_control;
	unsigned int ps_control;
	unsigned int control;
    unsigned int interrupt;
	/* variables to store register value - end */

	u16 als;
	u16 ps;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	unsigned int ps_status; /* current status of poximity detection : 0 = near, 1 = far */
	unsigned int ps_th_status; /* current threshold status of poximity detection : 0 = near, 1 = far */

	/* threshold value to detect "near-to-far" event */
	unsigned int ps_th_near_low;
	unsigned int ps_th_near_high;

	/* threshold value to detect "far-to-near" event */
	unsigned int ps_th_far_low;
	unsigned int ps_th_far_high;

	unsigned int ps_cross_talk; /* a result value of calibration. it will be used to compensate threshold value. */

	/* ALS parameters */
	unsigned int als_data;		/* to store ALS data */
	int als_lux_value;		/* to report the lux value */

	unsigned int als_poll_delay;	/* needed for light sensor polling : micro-second (us) */

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_drv;
	#endif
};


/****************************************************************************
* Variables
****************************************************************************/
static struct i2c_client *rpr0521_i2c_client = NULL; /* for general file I/O service. will be init on rpr0521_i2c_probe() */
static struct rpr0521_priv *g_rpr0521_ptr = NULL; /* for interrupt service call. will be init on rpr0521_i2c_probe() */
static struct platform_driver rpr0521_alsps_driver;

static int Target_Pdata = 26 /*16*/ /*12*/ /*40*/; /* parameter for taget pdata setting */
static int NearToFar = 10 /*4*/ /*4*/ /*20*/; /* parameter for far detection */

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern void mt_eint_unmask ( unsigned int line );
extern void mt_eint_mask ( unsigned int line );
extern void mt_eint_set_polarity ( unsigned int eint_num, unsigned int pol );
extern void mt_eint_set_hw_debounce ( unsigned int eint_num, unsigned int ms );
extern unsigned int mt_eint_set_sens ( unsigned int eint_num, unsigned int sens );
void mt_eint_registration(unsigned int eint_num, unsigned int flag,
              void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);

#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS) //for auto detect
static int rpr0521_local_init(void);
static int rpr0521_local_remove(void);
static int rpr0521_init_flag = -1;//0<==>OK, -1<==>fail
static struct sensor_init_info rpr0521_init_info = {
	.name = "RPR0521",
	.init = rpr0521_local_init,
	.uninit = rpr0521_local_remove,
};
#endif


/****************************************************************************
* Local Function Prototypes
****************************************************************************/
void rpr0521_eint_func ( void );

#if defined(TARGET_MT6732_C90)
#define GPIO_PROXIMITY_INT         GPIO_ALS_EINT_PIN
#define GPIO_PROXIMITY_INT_M_GPIO   GPIO_ALS_EINT_PIN_M_EINT
#define GPIO_PROXIMITY_INT_M_EINT   GPIO_PROXIMITY_INT_M_GPIO

#define CUST_EINT_PROXIMITY_NUM              CUST_EINT_ALS_NUM
#define CUST_EINT_PROXIMITY_DEBOUNCE_CN      0
#define CUST_EINT_PROXIMITY_TYPE							CUST_EINTF_TRIGGER_FALLING
#define CUST_EINT_PROXIMITY_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE
#endif

/****************************************************************************
* Local Functions
****************************************************************************/

//==========================================================
// Platform(AP) dependent functions
//==========================================================
static void rpr0521_setup_eint ( void )
{
	APS_FUN ();

	/* Configure GPIO settings for external interrupt pin  */

	mt_set_gpio_mode(GPIO_PROXIMITY_INT, GPIO_PROXIMITY_INT_M_EINT);
	mt_set_gpio_dir(GPIO_PROXIMITY_INT, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_PROXIMITY_INT, GPIO_PULL_DISABLE);

	/* Configure external interrupt settings for external interrupt pin */
	mt_eint_set_hw_debounce(CUST_EINT_PROXIMITY_NUM, CUST_EINT_PROXIMITY_DEBOUNCE_EN);
	mt_eint_registration(CUST_EINT_PROXIMITY_NUM, EINTF_TRIGGER_FALLING, rpr0521_eint_func, 0);

	/* Mask external interrupt to avoid un-wanted interrupt. Unmask it after initialization */
	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );
}


//==========================================================
// RPR0521 Register Read / Write Funtions
//==========================================================
static int rpr0521_write_cmd ( struct i2c_client *client, u8 val )
{
	int res = 0;

   mutex_lock(&rpr0521_access);

	res = i2c_master_send ( client, &val, 1 );
	if ( res == 1 )
	{
		APS_DBG ( "I2C write ( val=0x%02x )\n", val );
        mutex_unlock(&rpr0521_access);
		return RPR0521_SUCCESS;
	}
	else
	{
	 	APS_ERR ( "failed to write to RPR0521 ( err=%d, cmd=0x%02x )\n", res, val );
        mutex_unlock(&rpr0521_access);
 		return RPR0521_ERR_I2C;
	}
}

static int rpr0521_write_byte ( struct i2c_client *client, u8 reg, u8 val )
{
	int res = 0;
	u8 pBuf[2] = { 0 };


    mutex_lock(&rpr0521_access);

	pBuf[0] = reg;
	pBuf[1] = val;

	res = i2c_master_send ( client, pBuf, 2 );
	if ( res == 2 )
	{
		APS_DBG ( "I2C write ( reg=0x%02x, val=0x%02x )\n", reg, val );
        mutex_unlock(&rpr0521_access);
		return RPR0521_SUCCESS;
	}
	else
	{
		APS_ERR ( "failed to write to RPR0521 ( err=%d, reg=0x%02x, val=0x%02x )\n", res, reg, val );
        mutex_unlock(&rpr0521_access);
		return RPR0521_ERR_I2C;
	}

}

static int rpr0521_write_word ( struct i2c_client *client, u8 reg, u16 val )
{
	int res = 0;
	u8 pBuf[3] = { 0 };

    mutex_lock(&rpr0521_access);

	pBuf[0] = reg ;
	pBuf[1] = val & 0xFF ;
	pBuf[2] = ( val >> 8 ) & 0xFF ;

	res = i2c_master_send ( client, pBuf, 3 );
	if ( res == 3 )
	{
		APS_DBG ( "I2C write ( reg=0x%02x, val=0x%04x )\n", reg, val );
        mutex_unlock(&rpr0521_access);
		return RPR0521_SUCCESS;
	}
	else
	{
		APS_ERR ( "failed to write to RPR0521 ( err=%d, reg=0x%02x, val=0x%04x )\n", res, reg, val );
        mutex_unlock(&rpr0521_access);
		return RPR0521_ERR_I2C;
	}

}

static int rpr0521_read_byte ( struct i2c_client *client, u8 reg, u8 *pVal )
{
	int res = 0;

    mutex_lock(&rpr0521_access);

	if ( pVal == NULL )
	{
		APS_ERR ( "invalid input ( pVal=NULL )" );
		goto EXIT_ERR;
	}

	res = i2c_master_send ( client, &reg, 1 );
	if ( res != 1 )
	{
		APS_ERR ( "rpr0521_read_byte error i2c_master_send (1)....\n" );
		goto EXIT_ERR;
	}

	res = i2c_master_recv ( client, pVal, 1 );
	if ( res != 1 )
	{
		APS_ERR ( "rpr0521_read_byte error i2c_master_recv (2)....\n" );
		goto EXIT_ERR;
	}

	//APS_DBG ( "I2C read ( reg=0x%02x, val=0x%02x )\n", reg, *pVal );
    mutex_unlock(&rpr0521_access);
	return RPR0521_SUCCESS;

	EXIT_ERR:
	APS_ERR ( "failed to read from RPR0521 ( err=%d, reg=0x%02x )\n", res, reg );
    mutex_unlock(&rpr0521_access);
	return RPR0521_ERR_I2C;
}

static int rpr0521_read_word ( struct i2c_client *client, u8 reg, u16 *pVal )
{
	int res = 0;
	u8 pBuf[2] = { 0 };

    mutex_lock(&rpr0521_access);

	if ( pVal == NULL )
	{
		APS_ERR ( "invalid input ( pVal=NULL )" );
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

	//APS_DBG ( "I2C read ( reg=0x%02x, val=0x%04x )\n", reg, *pVal );
    mutex_unlock(&rpr0521_access);
	return RPR0521_SUCCESS;

	EXIT_ERR:
	APS_ERR ( "failed to read from RPR0521 ( err=%d, reg=0x%02x )\n", res, reg );
    mutex_unlock(&rpr0521_access);
	return RPR0521_ERR_I2C;
}

//==========================================================
// RPR0521 Basic Read / Write Funtions
//==========================================================
static int rpr0521_set_enable ( struct i2c_client *client, unsigned char enable )
{
    struct rpr0521_priv *obj = i2c_get_clientdata ( client );
    int data;
    int res;
    int set_enable = 0;

    if( enable == SET_IC_ENABLE ){
		set_enable = MOCTL_EN_MASK;
    }
    
    res = rpr0521_get_control(client, &data);
    if ( res == RPR0521_SUCCESS ) {
        data = (data & ~MOCTL_EN_MASK) | set_enable;
        res = rpr0521_write_byte ( client, REG_MODECONTROL, data );
        if ( res == RPR0521_SUCCESS ) {
			obj->enable_ps_sensor = (unsigned int)enable;
			obj->enable_als_sensor = (unsigned int)enable;
            //obj->enable = (unsigned int)enable;//TODO fix
        }
    }

    return res;
}

static int rpr0521_set_ps_enable ( struct i2c_client *client, unsigned char enable )
{
    struct rpr0521_priv *obj = i2c_get_clientdata ( client );
    int data;
    int res;
    int set_enable = 0;

    if( enable == SET_IC_ENABLE ){
		set_enable = MOCTL_ENPS_MASK;
    }
    
    res = rpr0521_get_control(client, &data);
    if ( res == RPR0521_SUCCESS ) {
        data = (data & ~MOCTL_ENPS_MASK) | set_enable;
        res = rpr0521_write_byte ( client, REG_MODECONTROL, data );
        if ( res == RPR0521_SUCCESS ) {
			obj->enable_ps_sensor = (unsigned int)enable;
            //obj->enable = (unsigned int)enable;//TODO fix
        }
    }

    return res;
}

static int rpr0521_set_als_enable ( struct i2c_client *client, unsigned char enable )
{
    struct rpr0521_priv *obj = i2c_get_clientdata ( client );
    int data;
    int res;
    int set_enable = 0;

    if( enable == SET_IC_ENABLE ){
		set_enable = MOCTL_ENALS_MASK;
    }
    
    res = rpr0521_get_control(client, &data);
    if ( res == RPR0521_SUCCESS ) {
        data = (data & ~MOCTL_ENALS_MASK) | set_enable;
        res = rpr0521_write_byte ( client, REG_MODECONTROL, data );
        if ( res == RPR0521_SUCCESS ) {
			obj->enable_als_sensor = (unsigned int)enable;
            //obj->enable = (unsigned int)enable;//TODO fix
        }
    }

    return res;
}

static int rpr0521_set_pilt ( struct i2c_client *client, unsigned short threshold )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = rpr0521_write_word ( client, REG_PSTL, ( u16 )threshold );
	if ( res == RPR0521_SUCCESS )
	{
		obj->pilt = (unsigned int)threshold;
	}

	return res;
}

static int rpr0521_set_piht ( struct i2c_client *client, unsigned short threshold )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	int res = 0;

	res = rpr0521_write_word ( client, REG_PSTH, ( u16 )threshold );
	if ( res == RPR0521_SUCCESS )
	{
		obj->piht = (unsigned int)threshold;
	}

	return res;
}


static int rpr0521_set_control ( struct i2c_client *client, unsigned char control )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	unsigned char data;
    int           res = 0;
    

    res = rpr0521_get_control(client, &data);
	if ( res == RPR0521_SUCCESS )
	{
	    control |= (data & MOCTL_EN_MASK);
    	res = rpr0521_write_byte ( client, REG_MODECONTROL, ( u8 )control );
    	if ( res == RPR0521_SUCCESS )
    	{
    	    //obj->control = (unsigned int)control;
    	    obj->control = (unsigned int)(control & ~MOCTL_EN_MASK) ;
    	}
	}

	return res;
}

static int rpr0521_set_alsps_control ( struct i2c_client *client, unsigned char data )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
    int res = 0;
    
	res = rpr0521_write_byte ( client, REG_ALSPSCONTROL, ( u8 )data );
	if ( res == RPR0521_SUCCESS )
	{
		obj->alsps_control = (unsigned int)data;
	}

	return res;
}

static int rpr0521_set_ps_control ( struct i2c_client *client, unsigned char data )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
    int res = 0;
    
	res = rpr0521_write_byte ( client, REG_PSCONTRL, ( u8 )data );
	if ( res == RPR0521_SUCCESS )
	{
		obj->ps_control = data;
	}

	return res;
}

static int rpr0521_set_ps_interrupt ( struct i2c_client *client, unsigned char data )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
    int res = 0;
    
	res = rpr0521_write_byte ( client, REG_INTERRUPT, ( u8 )data );
	if ( res == RPR0521_SUCCESS )
	{
		obj->interrupt = (unsigned int)data;
	}

	return res;
}

static int cal_running=0; 
static int rpr0521_get_control ( struct i2c_client *client, unsigned char *pData )
{
    struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
	int enable_status = 0;

	res = rpr0521_read_byte ( client, REG_MODECONTROL,( u8 * ) pData );

    #if 1 //recovery code
    if( ((*pData & 0x0F) != SET_ALS_100_PS_100MS) && (cal_running == 0))
    	{
    	  APS_LOG ("RPR0521 0x41 Reg Setting Changed to 0x%02x : set again\n",(*pData&0x0F));
    	  enable_status = (*pData & 0xF0);
    	  *pData = 0x00;
	    	   if(!(( obj->enable_ps_sensor << 6) && (enable_status & MOCTL_ENPS_MASK)))
	    	   	{
	    	   	   APS_LOG ("RPR0521 0x41 Reg PS_EN data Changed : set again\n");
	    	  	   *pData = (obj->enable_ps_sensor << 6);
	    	    	 if(!(( obj->enable_als_sensor << 7) && (enable_status & MOCTL_ENALS_MASK)))
	    	    	 	{
	    	   	         APS_LOG ("RPR0521 0x41 Reg ALS_EN data Changed : set again\n");
	    	  	         *pData =(*pData | (obj->enable_als_sensor << 7));
	    	    	 	}
	    	   	}
	    	    else if(!(( obj->enable_als_sensor << 7) && (enable_status & MOCTL_ENALS_MASK)))
	    	    	{
	    	   	   APS_LOG ("RPR0521 0x41 Reg ALS_EN data Changed : set again\n");
	    	  	   *pData = (obj->enable_als_sensor << 7);
	    	    	 if(!(( obj->enable_ps_sensor << 6) && (enable_status & MOCTL_ENPS_MASK)))
	    	    	 	{
	    	   	         APS_LOG ("RPR0521 0x41 Reg PS_EN data Changed : set again\n");
	    	  	         *pData =  (*pData| (obj->enable_ps_sensor << 6));
	    	    	 	}
	    	    	}
    	      *pData = ((*pData & 0xF0) | SET_ALS_100_PS_100MS) ;
    	    }
    #endif
	
	if ( res == RPR0521_SUCCESS )
	{
		APS_LOG ( "MODECONTROL=0x%02x\n", *pData );
	}

	return res;
}

static int rpr0521_get_alsps_control ( struct i2c_client *client, unsigned char *pData )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
    int res = 0;
    
	res = rpr0521_read_byte ( client, REG_ALSPSCONTROL,( u8 * ) pData );
	if ( res != RPR0521_SUCCESS )
	{
		//APS_LOG ( "ALSPSCONTRL=0x%02x\n", *pData );
		APS_LOG ( "ALSPSCONTRL read unsuccessful!!!\n" );
	}

	return res;
}

static int rpr0521_get_ps_control ( struct i2c_client *client, unsigned char *pData )
{
	int res = 0;

	res = rpr0521_read_byte ( client, REG_PSCONTRL,( u8 * ) pData );
	if ( res == RPR0521_SUCCESS )
	{
		APS_LOG ( "PSCONTRL=0x%02x\n", *pData );
	}

	return res;
}


static int rpr0521_get_status ( struct i2c_client *client, unsigned char *pData )
{
	int res = 0;

	res = rpr0521_read_byte ( client, REG_INTERRUPT, ( u8 * ) pData );
	if ( res == RPR0521_SUCCESS )
	{
		APS_LOG ( "INT_STATUS=0x%02x\n", *pData );
	}

	return res;
}

static int rpr0521_get_pdata ( struct i2c_client *client, unsigned short *pData )
{
	int res = 0;

	res = rpr0521_read_word ( client, REG_PSDATA, ( u16 * ) pData );
	if ( res == RPR0521_SUCCESS )
	{
		APS_DBG ( "PDATA=0x%04x\n", *pData );
	}

	return res;
}

static int rpr0521_get_alsdata0 ( struct i2c_client *client, unsigned short *pData )
{
	int res = 0;

	res = rpr0521_read_word ( client, REG_ALSDATA0, ( u16 * ) pData );
	if ( res != RPR0521_SUCCESS )
	{
		//APS_DBG ( "ALSDATA0=0x%04x\n", *pData );
		 APS_DBG ( "ALSDATA0 read unsuccessful!!!\n");
	}

	return res;
}

static int rpr0521_get_alsdata1 ( struct i2c_client *client, unsigned short *pData )
{
	int res = 0;

	res = rpr0521_read_word ( client, REG_ALSDATA1, ( u16 * ) pData );
	if ( res != RPR0521_SUCCESS )
	{
		//APS_DBG ( "ALSDATA1=0x%04x\n", *pData );
		APS_DBG ( "ALSDATA1 read unsuccessful!!!\n");
	}

	return res;
}

static int rpr0521_get_deivceid( struct i2c_client *client, unsigned char *pData )
{
	int res = 0;

	res = rpr0521_read_byte ( client, REG_MANUFACT_ID, ( u8 * )pData );
	if ( res == RPR0521_SUCCESS )
	{
		APS_DBG ( "DEVICEID=0x%02x\n", *pData );
	}

	return res;
}

static int rpr0521_clear_interrupt ( struct i2c_client *client )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	unsigned char first_data, second_data;
	int           res = 0;

	res = rpr0521_get_status ( client, &first_data );
	if ( res == RPR0521_SUCCESS ) {
        res = rpr0521_get_status ( client, &second_data );
        if ( res == RPR0521_SUCCESS ) {
            if((second_data & RPR0521_PINT) == 0) {
                APS_DBG ( "RPR0521 interrupt was cleared\n" );
            } else {
                res = RPR0521_FAIL;
                APS_DBG ( "RPR0521 interrupt cleare failed\n" );
            }
        }
	}

	return res;
}

//==========================================================
// APDS9130 Data Processign Funtions
//==========================================================
static int rpr0521_decide_ps_state ( struct i2c_client *client, int pdata, int int_status )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	int ps_status = obj->ps_status;

	if ( obj->ps_status == PS_FAR )
	{
	    /* Even saturation bit is set by rpr0521, pdata is correctly update. So Do not check saturation bit */
	    if ( ( pdata >= obj->ps_th_far_high ) && ( ( int_status & AMB_IRFLAG_TOO_HIGH ) != AMB_IRFLAG_TOO_HIGH ) )
		{
			ps_status = PS_NEAR;
			APS_LOG ( "PS = NEAR\n" );
		}
		else
		{
			APS_ERR ( "Unknown Event State\n" );
		}
	}
	else
	{
		if ( pdata <= obj->ps_th_near_low )
		{
			ps_status = PS_FAR;
			APS_LOG ( "PS = FAR\n" );
		}
		else
		{
			APS_ERR ( "Unknown Event State\n" );
		}
	}

	return ps_status;
}

static long rpr0521_initialize ( struct i2c_client *client  )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
	unsigned char id = 0;

    res = rpr0521_get_deivceid(client, &id);
	if ( res != RPR0521_SUCCESS )
	{
		APS_ERR ( "failed to read Device ID and it means I2C error happened\n");
		return res;
	}
	 
	APS_LOG ( "RPR0521 Device ID = 0x%02x\n", id );

	obj->enable_ps_sensor = 0;
	obj->enable_als_sensor = 0;
	obj->als_poll_delay = 100;	// default to 100ms

	/* disable proximity */
	rpr0521_set_enable( client, SET_IC_DISABLE );
    
	/* initialize registers of proximity */
	rpr0521_set_control ( client, INIT_MODE_CONTROL ); // fix for als
	rpr0521_set_alsps_control ( client, INIT_ALSPSMODE_CONTROL);//add als gain
	rpr0521_set_ps_control ( client, INIT_PSMODE_CONTROL);
    rpr0521_set_ps_interrupt( client, INIT_INTERRUPT);
	
	/* crosstalk value shall be set by LGP Server using I/O so init here to 150 */
    obj->ps_cross_talk = 10 /*5*/ /*100*/;

#if 0
	/* initialize threshold value of PS */
	if ( obj->ps_cross_talk > 150 )
	{
		obj->ps_cross_talk = 150;
	}
	if ( obj->ps_cross_talk < 0 )
	{
		obj->ps_cross_talk = 0;
	}
#endif
	obj->ps_th_far_low = 0;
	obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;
	obj->ps_th_near_high = 4095;
//	obj->ps_th_near_high = 1023;

	return res;

}

static long rpr0521_ps_enable ( struct i2c_client *client  )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	hwm_sensor_data sensor_data;
	int res    = 0;
	int status = 0;
	unsigned char psmctl_status = 0;
	int pdata  = 0;
    unsigned char old_power_state;

    old_power_state = (unsigned char)obj->enable_ps_sensor;
    
	/* enable RPR0521 */
	res = rpr0521_set_ps_enable ( client, SET_IC_ENABLE );
    if(res != RPR0521_SUCCESS) {
	    goto enable_error;
    }

	mdelay ( 100 );
    
	/* read sensor data */
	res = rpr0521_get_pdata ( client, (unsigned short*)&pdata );
    if(res != RPR0521_SUCCESS) {
	    goto enable_error;
    }

	/* decide current PS threshold state and set PS thershold to proper range */
	if ( pdata >= obj->ps_th_far_high )
	{
		obj->ps_th_status = PS_NEAR;
		obj->ps_status = PS_NEAR;
		rpr0521_set_pilt ( client, (unsigned short)obj->ps_th_near_low );
		rpr0521_set_piht ( client, (unsigned short)obj->ps_th_near_high );
	    APS_LOG ( "PS_TH=NEAR\n" );
	}
	else
	{
		obj->ps_th_status = PS_FAR;
		obj->ps_status = PS_FAR;
		rpr0521_set_pilt ( client, (unsigned short)obj->ps_th_far_low );
		rpr0521_set_piht ( client, (unsigned short)obj->ps_th_far_high );
		APS_LOG ( "PS_TH=FAR\n" );
	}

    /* interrupt clear */
    res = rpr0521_clear_interrupt(client);
    if(res != RPR0521_SUCCESS) {
	    goto enable_error;
    }
    
	/* inform to upper layer ( hwmsen ) */
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	sensor_data.value_divide = 1;
	sensor_data.values[0] = obj->ps_status;
	if ( ( res = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ) ) )
	{
		APS_ERR ( "failed to send inform ( err = %d )\n", res );
	    goto enable_error;
	}
    
	/* unmask external interrupt */
	mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );
	APS_LOG ( "RPR0521 was enabled\n" );

    return (RPR0521_SUCCESS);
    
enable_error :
    rpr0521_set_ps_enable(client, old_power_state);//I think check this..?
	APS_ERR ( "failed to enable RPR0521\n" );
    
    return (res);
}

static long rpr0521_ps_disable ( struct i2c_client *client  )
{
	int res = 0;

	/* mask external interrupt */
	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );

	/* disable RPR0521 */
	res = rpr0521_set_ps_enable ( client, SET_IC_DISABLE);
	if ( res == RPR0521_SUCCESS )
	{
		APS_LOG ( "RPR0521 was disabled\n" );
	}
	else
	{
		APS_ERR ( "failed to disable RPR0521\n" );
	}

	return res;
}

static long rpr0521_als_enable ( struct i2c_client *client  )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	hwm_sensor_data sensor_data;
	int res    = 0;
	int status = 0;
	unsigned char psmctl_status = 0;
	int pdata  = 0;
    unsigned char old_power_state;

    old_power_state = (unsigned char)obj->enable_als_sensor;
    
	/* enable RPR0521 */
	res = rpr0521_set_als_enable ( client, SET_IC_ENABLE);
    if(res != RPR0521_SUCCESS) {
	    goto enable_error;
    }

	//add als enable code

	APS_LOG ( "RPR0521 was enabled\n" );

    return (RPR0521_SUCCESS);
    
enable_error :
    rpr0521_set_als_enable(client, old_power_state);//check!
	APS_ERR ( "failed to enable RPR0521\n" );
    
    return (res);
}

static long rpr0521_als_disable ( struct i2c_client *client  )
{
	int res = 0;

	/* disable RPR0521 */
	res = rpr0521_set_als_enable ( client, SET_IC_DISABLE);
	if ( res == RPR0521_SUCCESS )
	{
		APS_LOG ( "RPR0521 was disabled\n" );
	}
	else
	{
		APS_ERR ( "failed to disable RPR0521\n" );
	}

	return res;
}

static int rpr0521_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	int ret;
	int atime_index=0;

	APS_FUN ();
	/* minimum 5ms */
	if (val < 3000)
		val = 3000;

	/* convert us => ms */
	obj->als_poll_delay = val / 1000;

	/* pre-defined delays from SensorManager
	 * SENSOR_DELAY_NORMAL : 200ms
	 * SENSOR_DELAY_UI : 66.667ms
	 * SENSOR_DELAY_GAME : 20ms
	 * SENSOR_DELAY_FASTEST : 0ms
	 * hwmsen changes the delay to 10ms, less than 10ms
	 */

	return 0;
}

/******************************************************************************
 * NAME       : rpr0521_lux_calculation
 * FUNCTION   : calculate illuminance data for RPR-0521
 * REMARKS    : ALS Gain must be x1 or x64 and ALS Measure time must be 100ms.
 *            : The value of gain0 and gain1 must be same.
 *            : Measeure time can use only normal mode.
 *            :
 *            : final_data format
 *            :+-----------------------------------+
 *            :| positive values : 21bit(Max value)|
 *            :+-----------------------------------+
 *            : Step is 1 Lux. Data is cut decimal point.
 *****************************************************************************/
int IsDataReading = 0; 
static int rpr0521_lux_calculation(struct i2c_client *client)
{
    /* TODO : calculation lux value with alsdata0 and alsdata1 
      you can use rpr0521_get_alsdata0 & rpr0521_get_alsdata1 function for read register*/
    unsigned long  base1, base2;
    unsigned long  cnt;
    signed long    keep_lux;
    int            lux;
    int            result;
    unsigned short data0, data1;
    struct rpr0521_priv *obj = i2c_get_clientdata ( client );
    signed long    moving_point;
    unsigned char  raw_gain, als_gain;
    
    #if 1 
    IsDataReading = 1;
    #endif
    
    if (ALSCALC_OFFSET == 0) {
        moving_point = 1;
    } else {
        moving_point = ALSCALC_OFFSET;
    }

#if 1
    /* get the data from IC */
    result = rpr0521_get_alsdata0(client, &data0);
    if(result == RPR0521_SUCCESS) {
        result = rpr0521_get_alsdata1(client, &data1);
        if(result ==  RPR0521_SUCCESS) {
            result = rpr0521_get_alsgain(client, &raw_gain, &als_gain);
        }
    }
    /* IF      D1/ D0 < 1.13  : case 0 */
    /* ELSE IF D1/ D0 < 1.424 : case 1 */
    /* ELSE IF D1/ D0 < 1.751 : case 2 */
    /* ELSE IF D1/ D0 < 3.015 : case 3 */
    /* ELSE                   : case 4 */
    if(result == RPR0521_SUCCESS) {
        base1 = data1 * moving_point;
        for(cnt=0;cnt < COEFFICIENT; cnt++) {
            base2 = data0 * judge_coef[cnt];
            if(base1 < base2) {
                break;
            }
        }

        if(cnt >= COEFFICIENT) {
            /* case 4 */
            lux = 0;
        } else {
            /* case 0,1,2,3 */
            /* lux = ((data0_conf * data0 - data1_conf * data1) / (1000 * gain)  */
            keep_lux = ((data0_coef[cnt] * data0) - (data1_coef[cnt] * data1));
            lux      = (int)(keep_lux / (moving_point * als_gain));
            if (lux < 0) {
                /* minimum process */
                lux = 0;
            } else if (lux > ALS_MAX_VALUE) {
                /* overflow process */
                lux = ALS_MAX_VALUE;
            }
        }
        obj->als_lux_value = lux;
        //APS_LOG ( "rpr0521_lux_calculation : keep_lux = %ld \n",keep_lux );
        APS_LOG ( "rpr0521_lux_calculation : lux_result = %d \n",lux );
        result = rpr0521_set_alsgain(client, raw_gain, lux);
    }
#else
    result = rpr0521_get_alsdata0(client, &data0);
    if(result == RPR0521_SUCCESS) {
        result = rpr0521_get_alsdata1(client, &data1);
    }
    if(result ==  RPR0521_SUCCESS) {
        base1 = data1 * ALSCALC_OFFSET;
        base2 = data0 * judge_coef[0];
        for(cnt=0;cnt < COEFFICIENT; cnt++) {
            if(base1 < base2) {
                break;
            } else {
                base2 = data0 * judge_coef[cnt];
            }
        }

        if(cnt >= COEFFICIENT) {
            lux = 0;
        }
		else {
            keep_lux = ((data0_coef[cnt] * data0) - (data1_coef[cnt] * data1));
            lux      = (int)(keep_lux)/ ALSCALC_OFFSET;
            if (lux < 0) {
                /* minimum process */
                lux = 0;
            } else if (lux > ALS_MAX_VALUE) {
                /* overflow process */
                lux = ALS_MAX_VALUE;
            }
        }
        obj->als_lux_value = lux;
 		//APS_LOG ( "rpr0521_lux_calculation : keep_lux = %ld \n",keep_lux );
    	APS_LOG ( "rpr0521_lux_calculation : lux_result = %d \n",lux );
    }
#endif
    
    #if 1 
    IsDataReading = 0;
    #endif
    
    return (result);

}

static int rpr0521_get_alsgain( struct i2c_client *client, unsigned char *raw_gain, unsigned char *gain)
{
    int result;
    unsigned char check_gain;
    unsigned char gain_value;
    
    result = rpr0521_get_alsps_control(client, raw_gain);
    if(result == RPR0521_SUCCESS) {
        check_gain = *raw_gain & ALSCALC_GAIN_MASK;
        if (check_gain == 0) {
            gain_value = 1;
        } else {
            gain_value = 64;
        }
        *gain = gain_value;
    }
    
    return (result);
}

static int rpr0521_set_alsgain( struct i2c_client *client, unsigned char raw_gain, signed long lux)
{
    int result;
    unsigned char set_gain;
    unsigned char check_value;
    unsigned char set_value;
    
    if (lux < GAIN_THRESHOLD) {
        set_gain = ALSGAIN_X64X64;
    } else {
        set_gain = ALSGAIN_X1X1;
    }
    
    check_value = raw_gain & ALSCALC_GAIN_MASK;
    if(check_value != set_gain) {
        /* make setting value */
        set_value = (raw_gain & ~ALSCALC_GAIN_MASK) | set_gain;
        result    = rpr0521_set_alsps_control(client, set_value);
    }
    
    return (result);
}

void rpr0521_swap(int *x, int *y)
{
     int temp = *x;
     *x = *y;
     *y = temp;
}

static int rpr0521_do_calibration ( struct i2c_client *client, int *value )
{
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
    unsigned int calib_data;
	unsigned int old_enable = 0;
    unsigned char old_modectl;
    int           result,i;
    
	//rpr0521_led_power ( 1 ); //not use
	cal_running = 1; 
	
	old_enable  = obj->enable_ps_sensor;
    old_modectl = obj->control;
    
    calib_data = 0;
    for (i=0; i < CALB_TIMES; i++) {
        calib_data = rpr0521_calc_calibration(client);
        if( calib_data <= 350 /*200*/ /*25*/ /*870*/ ) {
			APS_LOG("ps_cross_talk save : %d\n",calib_data);
            obj->ps_cross_talk = calib_data;
            break;
        }
    }
    /* set retuned value (COMMON setting, error and nomal process) */
	result = rpr0521_set_control ( client, old_modectl ); /* Returned state  to system */
    if(result != RPR0521_SUCCESS) {
       cal_running = 0;
       return (result);
    }
	result = rpr0521_set_ps_enable ( client, old_enable );
    if(result != RPR0521_SUCCESS) {
    	cal_running = 0;
        return (result);
    }
	*value = obj->ps_cross_talk;
    
    if(i >= CALB_TIMES){
		APS_ERR ( "failed to calibrate cross talk/n" );
    	cal_running = 0;
		return -1;
    } else {
    	//rpr0521_led_power ( 0 );  //not use
    	obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
    	obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;
    }

	cal_running = 0;

	/* we should store it to storage ( it should be free from factory reset ) but ATCI Demon will store it through LGP Demon */
	return 0;
}

static unsigned int rpr0521_calc_calibration ( struct i2c_client *client )
{
    unsigned int value;
	int temp_pdata[CALB_BOX_TIMES] = {0,};
	int temp_state[CALB_BOX_TIMES] = {0,};
	unsigned int i                 = 0;
	unsigned int j                 = 0;
    unsigned int sum_of_pdata      = 0;
    int result ;


	/* Enable PS and Mask interrupt */
	result = rpr0521_set_ps_enable ( client, SET_IC_DISABLE );
    if(result != RPR0521_SUCCESS) {
        return (result);
    }
	result = rpr0521_set_control ( client,INIT_MODE_CONTROL /* SET_PS_MEASURE_10MS */); 
    if(result != RPR0521_SUCCESS) {
        return (result);
    }
	result = rpr0521_set_ps_enable ( client, SET_IC_ENABLE );
    if(result != RPR0521_SUCCESS) {
        return (result);
    }

	mdelay ( CALB_SYSTEM_WAIT );

	/* Read pdata */
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		rpr0521_get_status ( client, (unsigned char *)&( temp_state[i] ) );
		rpr0521_get_pdata ( client, (unsigned short *)&( temp_pdata[i] ) );
		mdelay ( CALB_IC_WAIT );
	}

	#if defined ( APS_DEBUG )
	APS_LOG ( "State Value = " );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		APS_LOG ( "%d ", temp_state[i] );
	}
	APS_LOG ( "\n" );
	APS_LOG ( "Read Value = " );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		APS_LOG ( "%d ", temp_pdata[i] );
	}
	APS_LOG ( "\n" );
	#endif

	/* sort pdata */
	for ( i = 0 ; i < CALB_BOX_TIMES - 1 ; i++ )
	{
		for ( j = i + 1 ; j < CALB_BOX_TIMES ; j++ )
		{
			if ( temp_pdata[i] > temp_pdata[j] )
			{
				rpr0521_swap ( temp_pdata+i, temp_pdata+j );
			}
		}
	}

	#if defined(APS_DEBUG)
#if 1 /* branden.you@lge.com_20120924 */
	APS_LOG ( "Read Value = " );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		APS_LOG ( "%d ", temp_pdata[i] );
	}
	APS_LOG ( "\n" );
#else
	APS_DBG ( "Read Value = " );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		APS_DBG ( "%d ", temp_pdata[i] );
	}
	APS_DBG ( "\n" );
#endif
	#endif

	/* take ten middle data only */
	for ( i = CALB_REMOVAL_TIME ; i < (CALB_BOX_TIMES - CALB_REMOVAL_TIME) ; i++ )
	{
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	/* calculate average */
    value = sum_of_pdata / (CALB_BOX_TIMES - (CALB_REMOVAL_TIME * 2));
	APS_LOG ( "New calibrated cross talk = %d\n", value );

    return (value);
}

//==========================================================
// RPR0521 General Control Funtions
//==========================================================
static long rpr0521_ps_activate ( struct i2c_client *client, int enable )
{
	APS_FUN ();

	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	long res = 0;

	if ( obj->enable_ps_sensor != enable )
	{
		if ( enable )
		{
			//rpr0521_led_power ( 1 );  //not use

			res = rpr0521_ps_enable ( client );
			if ( res == RPR0521_SUCCESS )
			{
				APS_LOG ( "RPR0521 was enabled\n" );
			}
			else
			{
				APS_ERR ( "failed to enable RPR0521\n" );
			}
		}
		else
		{
			res = rpr0521_ps_disable ( client );
			if ( res == RPR0521_SUCCESS )
			{
				APS_LOG ( "RPR0521 was disabled\n" );
				//rpr0521_led_power ( 0 ); //not use
			}
			else
			{
				APS_ERR ( "failed to disable RPR0521\n" );
			}
		}

		if ( res == RPR0521_SUCCESS )
		{
			obj->enable_ps_sensor = enable;
		}
		
	}

	return res;
	
}

static long rpr0521_als_activate ( struct i2c_client *client, int enable )
{
	APS_FUN ();

	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	long res = 0;

	if ( obj->enable_als_sensor != enable )
	{
		if ( enable )
		{
			//rpr0521_led_power ( 1 );  //not use

			res = rpr0521_als_enable ( client );
			if ( res == RPR0521_SUCCESS )
			{
				APS_LOG ( "RPR0521 was enabled\n" );
			}
			else
			{
				APS_ERR ( "failed to enable RPR0521\n" );
			}
		}
		else
		{
			res = rpr0521_als_disable ( client );
			if ( res == RPR0521_SUCCESS )
			{
				APS_LOG ( "RPR0521 was disabled\n" );
				//rpr0521_led_power ( 0 ); //not use
			}
			else
			{
				APS_ERR ( "failed to disable RPR0521\n" );
			}
		}

		if ( res == RPR0521_SUCCESS )
		{
			obj->enable_als_sensor = enable;
		}
		
	}

	return res;
	
}

//==========================================================
// RPR0521 Interrupt Service Routines
//==========================================================
void rpr0521_eint_func ( void )
{
	APS_FUN ();
	struct rpr0521_priv *obj = g_rpr0521_ptr;
	if ( !obj )
	{
		return;
	}
	schedule_work ( &obj->eint_work );
}

static void rpr0521_eint_work ( struct work_struct *work )
{
    APS_FUN ();
    struct rpr0521_priv *obj = ( struct rpr0521_priv * ) container_of ( work, struct rpr0521_priv, eint_work );
    struct i2c_client *client = obj->client;
    hwm_sensor_data sensor_data;

    int err;

    int int_status = 0;
    unsigned short pdata = 0;
    unsigned char psmdctl_state;
    int new_ps_status = 0;
    unsigned short low_threshould;
    unsigned short high_threshould;
    

	APS_LOG ( "External interrupt happened\n" );

    /* read status register */
    err = rpr0521_get_status ( client, &int_status );
    if ( err != RPR0521_SUCCESS)
    {
        APS_ERR("ADC value is not valid so just skip this interrupt");
        goto CLEAR_INTERRUPT;
    }
    if(int_status & RPR0521_PINT) {
        /* Check the system state */
        if(obj->enable_ps_sensor == (SET_IC_ENABLE)) {
            APS_LOG ( "PS interrupt happened\n" );

            err = rpr0521_get_ps_control(client, &psmdctl_state);
            if ( (err != RPR0521_SUCCESS) || ((psmdctl_state & AMB_IRFLAG_TOO_HIGH) == AMB_IRFLAG_TOO_HIGH)) {
                APS_ERR("ADC value is not valid so just skip this interrupt or Can't access register of ps_control");
                goto CLEAR_INTERRUPT;
            }
            
            /* read sensor data */
            err = rpr0521_get_pdata ( client, &pdata );
            if ( err != RPR0521_SUCCESS) {
                APS_ERR("Can't access register of pdata.");
                goto CLEAR_INTERRUPT;
            }
            
            new_ps_status = rpr0521_decide_ps_state(client, pdata, psmdctl_state);
            /* inform to upper layer ( hwmsen ), if status was changed */
            if ( new_ps_status != obj->ps_status )
            {
                if (new_ps_status == PS_NEAR) {
                    low_threshould  = obj->ps_th_near_low;
                    high_threshould = PS_TH_VAL_MAX;
                } else {
                    low_threshould  = PS_TH_VAL_MIN;
                    high_threshould = obj->ps_th_far_high;
                }
                rpr0521_set_pilt ( client, low_threshould );
                rpr0521_set_piht ( client, high_threshould );
            
                obj->ps_status    = new_ps_status;
                obj->ps_th_status = new_ps_status;

                sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
                sensor_data.value_divide = 1;
                sensor_data.values[0] = obj->ps_status;
                if ( ( err = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ) ) )
                {
                    APS_ERR ( "failed to send inform ( err = %d )\n", err );
                }
            } else {
                /* non process */
            }
        } else {
            APS_LOG ( "system stop already.\n" );
            return;
        }
    } else {
        APS_LOG ( "function Called by other factor (not interrupt of IC).\n" );
    }

CLEAR_INTERRUPT:	

    /* clear interrupt of proximity */
    rpr0521_clear_interrupt ( client );

    /* unmask external interrupt */
    mt_eint_unmask ( CUST_EINT_PROXIMITY_NUM );

    /* activate proximity */
    rpr0521_set_ps_enable ( client, SET_IC_ENABLE );
    

}

//==========================================================
// RPR0521 ADB Shell command function
//==========================================================
static ssize_t rpr0521_show_cali_value ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%u\n", data->ps_cross_talk );
}

static ssize_t rpr0521_store_cali_value ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = rpr0521_i2c_client;
	int ret;
	int data;

	ret = rpr0521_do_calibration ( client, &data );

	return count;
}

static ssize_t rpr0521_show_pilt ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->pilt );
}

static ssize_t rpr0521_store_pilt ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = rpr0521_set_pilt ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_th_near_low = (unsigned int)val;

	return count;
}

static ssize_t rpr0521_show_piht ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->piht );
}

static ssize_t rpr0521_store_piht ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = rpr0521_set_piht ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_th_far_high = (unsigned int)val;

	return count;
}

static ssize_t rpr0521_show_alsps_control ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->alsps_control );
}

static ssize_t rpr0521_store_alsps_control  ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = rpr0521_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = rpr0521_set_alsps_control ( client, (unsigned char)val );
    if ( ret < 0 ) {
		return ret;
    }
    
	return count;
}

static ssize_t rpr0521_show_ps_control ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->ps_control  );
}

static ssize_t rpr0521_store_ps_control ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = rpr0521_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = rpr0521_set_ps_control( client, (unsigned char)val );
    if ( ret < 0 ) {
		return ret;
    }

	return count;
}

static ssize_t rpr0521_show_control ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "0x%02x\n", data->control );
}

static ssize_t rpr0521_store_control ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = rpr0521_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = rpr0521_set_control ( client, (unsigned char)val );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t rpr0521_show_status ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = rpr0521_i2c_client;
    int status = 0;

    rpr0521_get_status ( client, &status );

    return sprintf ( buf, "0x%02x\n", status );
}

static ssize_t rpr0521_show_pdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = rpr0521_i2c_client;
    unsigned short data = 0;

    rpr0521_get_pdata ( client, &data );

    return sprintf ( buf, "%d\n", data );
}

static ssize_t rpr0521_show_alsdata0 ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = rpr0521_i2c_client;
    unsigned short data = 0;

    rpr0521_get_alsdata0 ( client, &data );

    return sprintf ( buf, "%d\n", data );
}

static ssize_t rpr0521_show_alsdata1 ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = rpr0521_i2c_client;
    unsigned short data = 0;

    rpr0521_get_alsdata1 ( client, &data );

    return sprintf ( buf, "%d\n", data );
}

static ssize_t rpr0521_show_luxdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
    int data = 0;

	rpr0521_lux_calculation(obj->client);
	data = obj->als_lux_value;

    return sprintf ( buf, "%d\n", data );
}

static ssize_t rpr0521_show_deviceid ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = rpr0521_i2c_client;
    unsigned char data = 0;

    rpr0521_get_deivceid ( client, &data );

    return sprintf ( buf, "%02x\n", data );
}

static ssize_t rpr0521_show_target_pdata ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", Target_Pdata);
}

static ssize_t rpr0521_store_target_pdata ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	Target_Pdata = val;
	obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;

	ret = rpr0521_set_piht ( client, (unsigned short)obj->ps_th_far_high );
	ret = rpr0521_set_pilt ( client, (unsigned short)obj->ps_th_near_low );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t rpr0521_show_ps_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *data = i2c_get_clientdata ( client );

    switch(data->enable_ps_sensor) {//need for fix (enable_ps_sensor and enable_als_sensor)
          case 0:
         	   return sprintf ( buf, "%s\n", "Proximity Disabled");
          case 1:
               return sprintf ( buf, "%s\n", "Proximity Enabled" );

           default:
               return sprintf ( buf, "%s\n", "Proximity Error" );
     	}
    
}

static ssize_t rpr0521_store_ps_enable ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = rpr0521_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret=0;

    switch(val) {
          case 0:
            ret = rpr0521_ps_activate ( client, 0 );
            break;
          case 1:
            ret = rpr0521_ps_activate ( client, 1 );
            break;

           default:

           	break;
     	}
      

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t rpr0521_show_als_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *data = i2c_get_clientdata ( client );

    switch(data->enable_als_sensor) {//need for fix (enable_ps_sensor and enable_als_sensor)
          case 0:
         	   return sprintf ( buf, "%s\n", "Ambient Light Disabled");
          case 1:
               return sprintf ( buf, "%s\n", "Ambient Light Enabled" );

           default:
               return sprintf ( buf, "%s\n", "Ambient Light Error" );
     	}
    
}

static ssize_t rpr0521_store_als_enable ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = rpr0521_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret=0;

    switch(val) {
          case 0:
            ret = rpr0521_als_activate ( client, 0 );
            break;
          case 1:
            ret = rpr0521_als_activate ( client, 1 );
            break;

           default:

           	break;
     	}
      

	if ( ret < 0 )
		return ret;

	return count;
}

#if 0
static ssize_t rpr0521_show_als_poll_delay(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = rpr0521_i2c_client;
	struct rpr0521_priv *data = i2c_get_clientdata ( client );

	/* return in micro-second */
	return sprintf(buf, "%d\n", data->als_poll_delay * 1000);
}

static ssize_t rpr0521_store_als_poll_delay(struct device_driver *dev, const char *buf, size_t count)
{
#ifdef ALS_POLLING_ENABLED
	struct i2c_client *client = rpr0521_i2c_client;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	rpr0521_set_als_poll_delay(client, val);
#endif

	return count;
}
#endif
static DRIVER_ATTR ( cali, S_IWUSR | S_IRUGO, rpr0521_show_cali_value, rpr0521_store_cali_value );
static DRIVER_ATTR ( pilt, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_pilt, rpr0521_store_pilt );
static DRIVER_ATTR ( piht, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_piht, rpr0521_store_piht );
static DRIVER_ATTR ( alsps_control, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_alsps_control, rpr0521_store_alsps_control );
static DRIVER_ATTR ( ps_control , S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_ps_control , rpr0521_store_ps_control  );
static DRIVER_ATTR ( control, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_control, rpr0521_store_control );
static DRIVER_ATTR ( status, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_status, NULL );
static DRIVER_ATTR ( pdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_pdata, NULL );
static DRIVER_ATTR ( deviceid, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_deviceid, NULL );  
static DRIVER_ATTR ( target_pdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_target_pdata, rpr0521_store_target_pdata );
static DRIVER_ATTR ( enable, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_ps_enable, rpr0521_store_ps_enable );
static DRIVER_ATTR ( als_enable, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_als_enable, rpr0521_store_als_enable );
static DRIVER_ATTR ( alsdata0, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_alsdata0, NULL );
static DRIVER_ATTR ( alsdata1, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_alsdata1, NULL );
static DRIVER_ATTR ( luxdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_luxdata, NULL );
//static DRIVER_ATTR ( als_poll_delay, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, rpr0521_show_als_poll_delay, rpr0521_store_als_poll_delay);

static struct driver_attribute *rpr0521_attr_list[] = {
	&driver_attr_cali,		   /*show calibration data*/
	&driver_attr_pilt,
	&driver_attr_piht,
	&driver_attr_alsps_control,
	&driver_attr_ps_control,
	&driver_attr_control,
	&driver_attr_status,
	&driver_attr_pdata,
	&driver_attr_deviceid,
	&driver_attr_target_pdata,
	&driver_attr_enable,
	&driver_attr_als_enable,
	&driver_attr_alsdata0,
	&driver_attr_alsdata1,
	&driver_attr_luxdata,
//	&driver_attr_als_poll_delay,
};

static int rpr0521_create_attr ( struct device_driver *driver )
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( rpr0521_attr_list ) / sizeof ( rpr0521_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		if ( err = driver_create_file ( driver, rpr0521_attr_list[idx] ) )
		{
			APS_ERR ( "driver_create_file (%s) = %d\n", rpr0521_attr_list[idx]->attr.name, err );
			break;
		}
	}

	return err;
}

static int rpr0521_delete_attr ( struct device_driver *driver )
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( rpr0521_attr_list ) / sizeof ( rpr0521_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		driver_remove_file ( driver, rpr0521_attr_list[idx] );
	}

	return err;
}

//==========================================================
// RPR0521 Service APIs ( based on File I/O )
//==========================================================
static int rpr0521_open ( struct inode *inode, struct file *file )
{
	APS_FUN ();
	file->private_data = rpr0521_i2c_client;

	if ( !file->private_data )
	{
		APS_ERR ( "Invalid input paramerter\n" );
		return -EINVAL;
	}

	return nonseekable_open ( inode, file );
}

static int rpr0521_release ( struct inode *inode, struct file *file )
{
	APS_FUN ();
	file->private_data = NULL;
	return 0;
}

static long rpr0521_unlocked_ioctl ( struct file *file, unsigned int cmd, unsigned long arg )
{

	APS_FUN ();

	struct i2c_client *client = ( struct i2c_client * ) file->private_data;
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	long err = 0;
	void __user *ptr = ( void __user * ) arg;
	int dat;
	unsigned short data0=0;
	uint32_t enable;
	uint32_t crosstalk = 0;

	switch ( cmd )
	{
		case ALSPS_SET_PS_MODE:
			APS_LOG ( "CMD = ALSPS_SET_PS_MODE\n" );
			if ( copy_from_user ( &enable, ptr, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			if ( enable )
			{
				if ( ( err = rpr0521_ps_activate ( obj->client, 1 ) ) )
				{
					APS_ERR ( "failed to activate RPR0521 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			else
			{
				if ( ( err = rpr0521_ps_activate ( obj->client, 0 ) ) )
				{
					APS_ERR ( "failed to deactivate RPR0521 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			break;

		case ALSPS_GET_PS_MODE:
			APS_LOG ( "CMD = ALSPS_GET_PS_MODE\n" );
			enable = obj->enable_ps_sensor;
			if ( copy_to_user ( ptr, &enable, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			APS_LOG ( "CMD = ALSPS_GET_PS_DATA\n" );
			dat = obj->ps_status;
			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:
			APS_LOG ( "CMD = ALSPS_GET_PS_RAW_DATA\n" );
			if ( err = rpr0521_get_pdata ( obj->client, (unsigned short *)&dat ) )
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
			APS_LOG ( "CMD = ALSPS_GET_CALI\n" );
			err = rpr0521_do_calibration ( obj->client, &dat );
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
            APS_LOG ( "CMD = ALSPS_SET_CALI\n" );
            if ( copy_from_user ( &crosstalk, ptr, sizeof ( crosstalk ) ) )
            {
                err = -EFAULT;
                goto err_out;
            }

			if ( ( crosstalk == 0x0000FFFF ) || ( crosstalk == 0 ) )
			{

				obj->ps_cross_talk = 10 /*5*/ /*100*/;

			}
			else
			{
				obj->ps_cross_talk = crosstalk;
			}


			obj->ps_th_far_high = Target_Pdata + obj->ps_cross_talk;
		    obj->ps_th_near_low = obj->ps_th_far_high - NearToFar;
            break;

		case ALSPS_GET_DEVICEID:
            APS_LOG ( "CMD = ALSPS_GET_DEVICEID\n" );
			if ( err = rpr0521_get_deivceid ( obj->client, (unsigned char*) &dat ) )
			{
				goto err_out;
			}

			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
			
		case ALSPS_SET_ALS_MODE:
			APS_LOG ( "CMD = ALSPS_SET_ALS_MODE\n" );
			if ( copy_from_user ( &enable, ptr, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			if ( enable )
			{
				if ( ( err = rpr0521_als_activate ( obj->client, 1 ) ) )
				{
					APS_ERR ( "failed to activate RPR0521 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			else
			{
				if ( ( err = rpr0521_als_activate ( obj->client, 0 ) ) )
				{
					APS_ERR ( "failed to deactivate RPR0521 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			break;
		case ALSPS_GET_ALS_MODE:
			APS_LOG ( "CMD = ALSPS_GET_ALS_MODE\n" );
			enable = obj->enable_als_sensor;
			if ( copy_to_user ( ptr, &enable, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_ALS_DATA:
			//APS_LOG ( "CMD = ALSPS_GET_ALS_DATA\n" );
	    if(!IsDataReading) 
	    	{
			 #if 1
					rpr0521_lux_calculation(client);
					dat = obj->als_lux_value;
					if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
					{
						err = -EFAULT;
						goto err_out;
					}			
			 #else
	            rpr0521_get_alsdata0 ( client, &data0 );
				if ( copy_to_user ( ptr, &data0, sizeof ( data0 ) ) )
				{
					err = -EFAULT;
					goto err_out;
				}
			 #endif
	    	}
	     else
	     	{
			APS_LOG ( "CMD = ALSPS_GET_ALS_DATA : Overlaped. Skip. \n" );
	     	}
		 
			break;

		default:
			APS_ERR ( "Invalid Command = 0x%04x\n", cmd );
			err = -ENOIOCTLCMD;
			break;
	}

	err_out : return err;
}

static struct file_operations rpr0521_fops = { .owner = THIS_MODULE, .open = rpr0521_open, .release = rpr0521_release,
												.unlocked_ioctl = rpr0521_unlocked_ioctl,  };

static struct miscdevice rpr0521_device = { .minor = MISC_DYNAMIC_MINOR, .name = "als_ps", .fops = &rpr0521_fops,  };

//==========================================================
// RPR0521 Service APIs ( based on hwmsen Interface )
//==========================================================
static int rpr0521_ps_operate ( void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout )
{
	APS_FUN ();
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	struct rpr0521_priv *obj = ( struct rpr0521_priv * ) self;

	switch ( command )
	{
		case SENSOR_DELAY:
			APS_LOG ( "CMD = SENSOR_DELAY\n" );
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				APS_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				APS_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				value = *( int * ) buff_in;
				if ( value )
				{
					APS_LOG ( "CMD = SENSOR_ENABLE ( Enable )\n" );
					if ( err = rpr0521_ps_activate ( obj->client, 1 ) )
					{
						APS_ERR ( "failed to activate RPR0521 ( err = %d )\n", err );
						return -1;
					}
				}
				else
				{
					APS_LOG ( "CMD = SENSOR_ENABLE ( Disable )\n" );
					if ( err = rpr0521_ps_activate ( obj->client, 0 ) )
					{
						APS_ERR ( "failed to deactivate RPR0521 ( err = %d )\n", err );
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			//APS_LOG ( "CMD = SENSOR_GET_DATA\n" );
			if ( ( buff_out == NULL ) || ( size_out < sizeof ( hwm_sensor_data ) ) )
			{
				APS_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				sensor_data = (struct hwm_sensor_data *)buff_out;
				sensor_data->values[0] = (int)obj->ps_status;
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			APS_ERR ( "Invalid Command = %d\n", command );
			err = -1;
			break;
	}

	return err;
}

static int rpr0521_als_operate ( void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout )
{
	APS_FUN ();
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	unsigned short data0=0;

	struct rpr0521_priv *obj = ( struct rpr0521_priv * ) self;

	switch ( command )
	{
		case SENSOR_DELAY:
			APS_LOG ( "CMD = SENSOR_DELAY\n" );
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				APS_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else {
				/* ALS Integration Time setting as fast as polling rate */
				value = *(int *)buff_in;
				rpr0521_set_als_poll_delay(obj->client, value*1000);
			}
			break;

		case SENSOR_ENABLE:
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				APS_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				value = *( int * ) buff_in;
				if ( value )
				{
					APS_LOG ( "CMD = SENSOR_ENABLE ( Enable )\n" );
					if ( err = rpr0521_als_activate ( obj->client, 1 ) )
					{
						APS_ERR ( "failed to activate RPR0521 ( err = %d )\n", err );
						return -1;
					}
				}
				else
				{
					APS_LOG ( "CMD = SENSOR_ENABLE ( Disable )\n" );
					if ( err = rpr0521_als_activate ( obj->client, 0 ) )
					{
						APS_ERR ( "failed to deactivate RPR0521 ( err = %d )\n", err );
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			APS_LOG ( "CMD = SENSOR_GET_DATA\n" );
			if ( ( buff_out == NULL ) || ( size_out < sizeof ( hwm_sensor_data ) ) )
			{
				APS_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				#if 1
					rpr0521_lux_calculation(obj->client);
					APS_LOG("**********lux : %d -- update : %d\n",obj->als_lux_value, obj->als_lux_value);
					sensor_data->values[0] = (obj->als_lux_value);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				#else
			        rpr0521_get_alsdata0 ( obj->client, &data0 );
					APS_LOG("********** TEMP Cdata0 : %d \n",data0);
					sensor_data->values[0] = data0;
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				#endif
			}
			break;

		default:
			APS_ERR ( "Invalid Command = %d\n", command );
			err = -1;
			break;
	}

	return err;
}


//==========================================================
// RPR0521 Initialization related Routines
//==========================================================
static int rpr0521_init_client ( struct i2c_client *client )
{
	APS_FUN ();
	struct rpr0521_priv *obj = i2c_get_clientdata ( client );
	int err = 0;

	err = rpr0521_initialize ( client );
	if ( err != RPR0521_SUCCESS )
	{
		APS_ERR ( "failed to init RPR0521\n" );
	}

	return err;
}

/****************************************************************************
* I2C BUS Related Functions
****************************************************************************/

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void rpr0521_early_suspend ( struct early_suspend *h )
{
	APS_FUN ();
}

static void rpr0521_late_resume ( struct early_suspend *h )
{
	APS_FUN ();
}
#endif

static int rpr0521_i2c_probe ( struct i2c_client *client, const struct i2c_device_id *id )
{
	APS_FUN ();
	struct rpr0521_priv *obj;
	struct hwmsen_object obj_ps;
	struct hwmsen_object obj_als;
	struct alsps_hw *hw = rohm_get_cust_alsps_hw ();
	int err = 0;

#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
	if(rpr0521_init_flag==0)
		{
			APS_LOG("Proximity Sensor already probed...just skip\n");
			return err;
		}	
	rpr0521_setup_eint ();
#endif
	if ( !( obj = kzalloc ( sizeof ( *obj ), GFP_KERNEL ) ) )

	{
		err = -ENOMEM;
		goto exit;
	}
	memset ( obj, 0, sizeof ( *obj ) );

	obj->client = client;
	i2c_set_clientdata ( client, obj );

	g_rpr0521_ptr = obj;
	rpr0521_i2c_client = client;

	INIT_WORK ( &obj->eint_work, rpr0521_eint_work );

	#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend = rpr0521_early_suspend,
	obj->early_drv.resume = rpr0521_late_resume,
	register_early_suspend ( &obj->early_drv );
	#endif

	/* Initialize RPR0521 */ 

	if ( err = rpr0521_init_client ( client ) )
	{
		APS_ERR ( "failed to init RPR0521 ( err = %d )\n", err );
		goto exit_init_failed;
	}
	APS_LOG("init_client pass\n");

	/* Register RPR0521 as a misc device for general I/O interface */
	if ( err = misc_register ( &rpr0521_device ) )
	{
		APS_ERR ( "failed to register misc device ( err = %d )\n", err );
		goto exit_misc_device_register_failed;
	}
#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
	if(err = rpr0521_create_attr(&(rpr0521_init_info.platform_diver_addr->driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
#else
	if ( err = rpr0521_create_attr ( &rpr0521_alsps_driver.driver ) )
	{
		APS_ERR ( "create attribute err = %d\n", err );
		goto exit_create_attr_failed;
	}
#endif

	/* Register RPR0521 as a member device of hwmsen */
	obj_ps.self = obj;
	obj_ps.polling = hw->polling_mode_ps;
	obj_ps.sensor_operate = rpr0521_ps_operate;
	if ( err = hwmsen_attach ( ID_PROXIMITY, &obj_ps ) )
	{
		APS_ERR ( "failed to attach to hwmsen ( err = %d )\n", err );
		goto exit_hwsen_attach_failed;
	}

	obj_als.self = obj;
	obj_als.polling = hw->polling_mode_als;
	obj_als.sensor_operate = rpr0521_als_operate;
	if ( err = hwmsen_attach ( ID_LIGHT, &obj_als ) )
	{
		APS_ERR ( "failed to attach to hwmsen ( err = %d )\n", err );
		goto exit_hwsen_attach_failed;
	}
	APS_LOG("%s: OK\n",__func__);
#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
		rpr0521_init_flag=0; 
#endif

	return 0;

	exit_hwsen_attach_failed:
    rpr0521_delete_attr(&rpr0521_alsps_driver.driver);
	exit_create_attr_failed:
	misc_deregister ( &rpr0521_device );
	exit_misc_device_register_failed:
	exit_init_failed:
	unregister_early_suspend ( &obj->early_drv );
	kfree ( obj );
	exit:
	rpr0521_i2c_client = NULL;
	APS_ERR ( "Err = %d\n", err );
#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
		rpr0521_init_flag=-1; 
#endif
	return err;
}

static int rpr0521_i2c_remove ( struct i2c_client *client )
{
	APS_FUN ();
	int err;
#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
		if(err = rpr0521_delete_attr(&(rpr0521_init_info.platform_diver_addr->driver)))
		{
			APS_ERR("rpr0521_delete_attr fail: %d\n", err);
		}
#else
	if ( err = rpr0521_delete_attr ( &rpr0521_alsps_driver.driver ) )
	{
		APS_ERR ( "rpr0521_delete_attr fail: %d\n", err );
	}
#endif
	if ( err = misc_deregister ( &rpr0521_device ) )
	{
		APS_ERR ( "failed to deregister misc driver : %d\n", err );
	}

	rpr0521_i2c_client = NULL;
	i2c_unregister_device ( client );
	kfree ( i2c_get_clientdata ( client ) );

	return 0;
}

static int rpr0521_i2c_suspend ( struct i2c_client *client, pm_message_t msg )
{	
	APS_FUN();
	return 0;
}

static int rpr0521_i2c_resume ( struct i2c_client *client )
{
	APS_FUN();	
	return 0;
}

static int rpr0521_i2c_detect ( struct i2c_client *client, struct i2c_board_info *info )
{
	APS_FUN ();
	strcpy ( info->type, RPR0521_DEV_NAME );
	return 0;
}

static const struct i2c_device_id rpr0521_i2c_id[] = { { RPR0521_DEV_NAME, 0 }, {} };


static struct i2c_driver rpr0521_i2c_driver = { .probe = rpr0521_i2c_probe, .remove = rpr0521_i2c_remove, .suspend = rpr0521_i2c_suspend,
												 .resume = rpr0521_i2c_resume, .detect = rpr0521_i2c_detect, .id_table = rpr0521_i2c_id,
												 .driver = { .name = RPR0521_DEV_NAME, }, };


/****************************************************************************
* Linux Device Driver Related Functions
****************************************************************************/
static int rpr0521_probe ( struct platform_device *pdev )
{
	APS_FUN ();

	struct alsps_hw *hw = rohm_get_cust_alsps_hw ();

	/* Configure external ( GPIO ) interrupt */
	rpr0521_setup_eint ();

	/* Turn on the power */
	//rpr0521_main_power ( hw, 1 );
	//msleep ( 9 );

	/* Add RPR0521 as I2C driver */
	if ( i2c_add_driver ( &rpr0521_i2c_driver ) )
	{
		APS_ERR ( "failed to add i2c driver\n" );
		return -1;
	}

	return 0;
}

static int rpr0521_remove ( struct platform_device *pdev )
{
	APS_FUN ();
	struct alsps_hw *hw = rohm_get_cust_alsps_hw ();

	/* Turn off the power */
	//rpr0521_main_power ( hw, 0 );

	i2c_del_driver ( &rpr0521_i2c_driver );

	return 0;
}

static struct i2c_board_info __initdata i2c_RPR0521 = { I2C_BOARD_INFO ( "RPR0521", RPR0521_I2CADDR ) };

static struct platform_driver rpr0521_alsps_driver = {
	.probe = rpr0521_probe,
	.remove = rpr0521_remove,
	.driver = {
		.name = "als_ps",
#ifdef CONFIG_OF_DT
		.of_match_table = psensor_of_match,
#endif

	},
};

#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
static int  rpr0521_local_init(void)
{
   struct alsps_hw *hw = rohm_get_cust_alsps_hw();
   
	APS_FUN();

	if(i2c_add_driver(&rpr0521_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}
	if(-1 == rpr0521_init_flag)
	{
	   return -1;
	}

	return 0;
}

static int  rpr0521_local_remove(void)
{
    struct alsps_hw *hw = rohm_get_cust_alsps_hw();

    APS_FUN();
    i2c_del_driver(&rpr0521_i2c_driver);
    return 0;
}
#endif

static int __init rpr0521_init ( void )
{
	APS_FUN ();

#if defined(TARGET_MT6732_C90)
	i2c_register_board_info ( 1, &i2c_RPR0521, 1 );
#else
	i2c_register_board_info ( 2, &i2c_RPR0521, 1 );
#endif

#if defined(CONFIG_MTK_AUTO_DETECT_ALSPS)
	hwmsen_alsps_sensor_add(&rpr0521_init_info);
#else
	if ( platform_driver_register ( &rpr0521_alsps_driver ) )
	{
		APS_ERR ( "failed to register platform driver\n" );
		return -ENODEV;
	}
#endif
	return 0;
}

static void __exit rpr0521_exit ( void )
{
	APS_FUN ();
	platform_driver_unregister ( &rpr0521_alsps_driver );
}


module_init ( rpr0521_init );
module_exit ( rpr0521_exit );

MODULE_AUTHOR ( "Seo Ji Won" );
MODULE_DESCRIPTION ( "rpr0521 driver" );
MODULE_LICENSE ( "GPL" );

/* End Of File */

