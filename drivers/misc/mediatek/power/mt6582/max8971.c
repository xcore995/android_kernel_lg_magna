/*
 *  MAXIM MAX8971 Charger Driver
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include <mach/mt_gpio.h>
#include <mach/charging.h>
#include <cust_gpio_usage.h>
#include <cust_gpio_boot.h>

#include <mach/eint.h>
#include <cust_eint.h>
#include <cust_charging.h>
#include <mt_auxadc_sw.h>

#include <max8971.h>
#include <mach/board_lge.h>

extern kal_bool chargin_hw_init_done;

// define register map
#define MAX8971_REG_CHGINT      	0x0F

#define MAX8971_REG_CHGINT_MASK 	0x01

#define MAX8971_REG_CHG_STAT    	0x02

#define MAX8971_DCV_MASK        	0x80
#define MAX8971_DCV_SHIFT       	7
#define MAX8971_DCI_MASK        	0x40
#define MAX8971_DCI_SHIFT       	6
#define MAX8971_DCOVP_MASK      	0x20
#define MAX8971_DCOVP_SHIFT     	5
#define MAX8971_DCUVP_MASK      	0x10
#define MAX8971_DCUVP_SHIFT     	4
#define MAX8971_CHG_MASK        	0x08
#define MAX8971_CHG_SHIFT		3
#define MAX8971_BAT_MASK		0x04
#define MAX8971_BAT_SHIFT		2
#define MAX8971_THM_MASK		0x02
#define MAX8971_THM_SHIFT		1
#define MAX8971_PWRUP_OK_MASK		0x01
#define MAX8971_PWRUP_OK_SHIFT		0
#define MAX8971_I2CIN_MASK		0x01
#define MAX8971_I2CIN_SHIFT		0

#define MAX8971_REG_DETAILS1		0x03
#define MAX8971_DC_V_MASK		0x80
#define MAX8971_DC_V_SHIFT		7
#define MAX8971_DC_I_MASK		0x40
#define MAX8971_DC_I_SHIFT		6
#define MAX8971_DC_OVP_MASK		0x20
#define MAX8971_DC_OVP_SHIFT		5
#define MAX8971_DC_UVP_MASK		0x10
#define MAX8971_DC_UVP_SHIFT		4
#define MAX8971_THM_DTLS_MASK		0x07
#define MAX8971_THM_DTLS_SHIFT  	0

#define MAX8971_THM_DTLS_COLD		1       // charging suspended(temperature<T1)
#define MAX8971_THM_DTLS_COOL		2       // (T1<temperature<T2)
#define MAX8971_THM_DTLS_NORMAL		3       // (T2<temperature<T3)
#define MAX8971_THM_DTLS_WARM		4       // (T3<temperature<T4)
#define MAX8971_THM_DTLS_HOT		5       // charging suspended(temperature>T4)

#define MAX8971_REG_DETAILS2		0x04
#define MAX8971_BAT_DTLS_MASK		0x30
#define MAX8971_BAT_DTLS_SHIFT		4
#define MAX8971_CHG_DTLS_MASK		0x0F
#define MAX8971_CHG_DTLS_SHIFT		0

#define MAX8971_BAT_DTLS_BATDEAD        0   // VBAT<2.1V
#define MAX8971_BAT_DTLS_TIMER_FAULT    1   // The battery is taking longer than expected to charge
#define MAX8971_BAT_DTLS_BATOK          2   // VBAT is okay.
#define MAX8971_BAT_DTLS_GTBATOV        3   // VBAT > BATOV

#define MAX8971_CHG_DTLS_DEAD_BAT           0   // VBAT<2.1V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_PREQUAL            1   // VBAT<3.0V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_FAST_CHARGE_CC     2   // VBAT>3.0V, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_FAST_CHARGE_CV     3   // VBAT=VBATREG, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_TOP_OFF            4   // VBAT>=VBATREG, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_DONE               5   // VBAT>VBATREG, T>Ttopoff+16s, TJSHDN<TJ<TJREG
#define MAX8971_CHG_DTLS_TIMER_FAULT        6   // VBAT<VBATOV, TJ<TJSHDN
#define MAX8971_CHG_DTLS_TEMP_SUSPEND       7   // TEMP<T1 or TEMP>T4
#define MAX8971_CHG_DTLS_USB_SUSPEND        8   // charger is off, DC is invalid or chaarger is disabled(USBSUSPEND)
#define MAX8971_CHG_DTLS_THERMAL_LOOP_ACTIVE    9   // TJ > REGTEMP
#define MAX8971_CHG_DTLS_CHG_OFF            	10  // charger is off and TJ >TSHDN

#define MAX8971_REG_CHGCNTL1		0x05
#define MAX8971_DCMON_DIS_MASK		0x02
#define MAX8971_DCMON_DIS_SHIFT		1
#define MAX8971_USB_SUS_MASK		0x01
#define MAX8971_USB_SUS_SHIFT		0

#define MAX8971_REG_FCHGCRNT		0x06
#define MAX8971_CHGCC_MASK		0x1F
#define MAX8971_CHGCC_SHIFT		0
#define MAX8971_FCHGTIME_MASK		0xE0
#define MAX8971_FCHGTIME_SHIFT         5 // BEFORE 5 it disabled need Change? go to X3-board platform data

#define MAX8971_REG_DCCRNT		0x07
#define MAX8971_CHGRSTRT_MASK		0x40
#define MAX8971_CHGRSTRT_SHIFT		6
#define MAX8971_DCILMT_MASK		0x3F
#define MAX8971_DCILMT_SHIFT		0

#define MAX8971_REG_TOPOFF		0x08
#define MAX8971_TOPOFFTIME_MASK		0xE0
#define MAX8971_TOPOFFTIME_SHIFT	5
#define MAX8971_IFST2P8_MASK		0x10
#define MAX8971_IFST2P8_SHIFT		4
#define MAX8971_TOPOFFTSHLD_MASK	0x0C
#define MAX8971_TOPOFFTSHLD_SHIFT	2
#define MAX8971_CHGCV_MASK		0x03
#define MAX8971_CHGCV_SHIFT		0

#define MAX8971_REG_TEMPREG		0x09
#define MAX8971_REGTEMP_MASK		0xC0
#define MAX8971_REGTEMP_SHIFT		6
#define MAX8971_THM_CNFG_MASK		0x08
#define MAX8971_THM_CNFG_SHIFT		3
//#define MAX8971_THM_CNFG_SHIFT	5
#define MAX8971_SAFETYREG_MASK		0x01
#define MAX8971_SAFETYREG_SHIFT		0

#define MAX8971_REG_PROTCMD		0x0A
#define MAX8971_CHGPROT_MASK		0x0C
#define MAX8971_CHGPROT_SHIFT		2
#define MAX8971_CHGPROT_UNLOCKED	0x03

#define MAX_CHARGER_INT_COUNT 10

#define MAX8971_TOPOFF_MODE	0x04
#define MAX8971_DONE_MODE	0x05

#define MAX8971_IFST2P8_CURRENT		2300

#define MAX8971_MONITOR_MS	10000	/* 10 sec */
#define AUXADC_CHG_CUR_CHANNEL 13 /* AUX_XP */

struct max8971_chip {
	struct i2c_client		*client;
	struct power_supply		psy;
	struct delayed_work		dwork;
	struct max8971_platform_data	*pdata;
	int				gpio;
	int				irq;
	int				online;
	int				cable_type;
	int				status;
	int				current_now;
	int				current_max;
	int				voltage;
	bool				enable;

	/* workaround for charging error */
	struct delayed_work		monitor_work;

	/* for debugging */
	struct delayed_work		debug_work;
	unsigned int 			debug_time;

};

static struct max8971_chip *max8971_chg;
static struct delayed_work *max8971_work = NULL;
static struct workqueue_struct *max8971_wq = NULL;
void max8971_enable_intr(struct max8971_chip *chip);
void max8971_disable_intr(struct max8971_chip *chip);

static struct max8971_platform_data max8971_default = {
	/* register value */
	.int_mask = 0x86,	/* Interrupt : TOPOFF, DC_OVP, DC_UVP, CHG */
	.usb_sus = 0,		/* Charging : Enable */
	.dcmon_dis = 0,		/* AICL : Enable */
	.chgcc = 31,		/* Charging Current : 1550 mA */
	.fchgtime = 0,		/* Fast Charging Timer : Disable */
	.dcilmt = 60,		/* Input Limit : 1500 mA */
	.chgrstrt = 0,		/* Fast Charge Restart Threshold : -150 mV */
	.chgcv = 2,		/* Charging Termination Voltage : 4.35V */
	.topofftshld = 3,	/* Topoff Current Threshold : 200 mA */
	.ifst2p8 = 0,		/* Scales Maximum Fast Charge Current to 2.8A : Off */
	.topofftime = 7,	/* Topoff Timer : 70 min */
	.safetyreg = 0,		/* JEITA Safety Region Selection : Safety region 1 */
	.thm_cnfg = 1,		/* Thermistor Monitor : Disable */
	.regtemp = 3,		/* Die-Temperature Thernaml Regulation Loop Setpoint : Disabled */
	.cprot = 3,		/* Charger-Setting Protection : Unlock */
};

static int max8971_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0) {
		dev_err(&client->dev, "%s failed : %d\n", __func__, ret);
	}

	return ret;
}

static int max8971_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0) {
		dev_err(&client->dev, "%s failed : %d\n", __func__, ret);
	}

	return ret;
}

static int max8971_set_bits(struct i2c_client *client, u8 reg, u8 mask, u8 data)
{
	u8 value = 0;
	int ret;

	ret = max8971_read_reg(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "%s failed : %d\n", __func__, ret);
		goto out;
	}

	value &= ~mask;
	value |= data;
	ret = max8971_write_reg(client, reg, value);
out:
	return ret;
}

static int max8971_unlock_setting_protection(struct max8971_chip *chip)
{
	return max8971_write_reg(chip->client, MAX8971_REG_PROTCMD, 0x0C);
}

static int max8971_get_ifst2p8(struct max8971_chip *chip)
{
	int data = 0;

	data = max8971_read_reg(chip->client, MAX8971_REG_TOPOFF);
	if (data < 0) {
		dev_err(&chip->client->dev, "cannot read ifst2p8.\n");
		return chip->pdata->ifst2p8;
	}

	data = ((data & MAX8971_IFST2P8_MASK) >> MAX8971_IFST2P8_SHIFT);

	return data;
}

static int max8971_set_ifst2p8(struct max8971_chip *chip, int enable)
{
	int data = 0;
	int rc;

	if (enable) {
		chip->pdata->ifst2p8 = 1;
	} else {
		chip->pdata->ifst2p8 = 0;
	}

	data = ((chip->pdata->topofftime << MAX8971_TOPOFFTIME_SHIFT) |
		(chip->pdata->ifst2p8 << MAX8971_IFST2P8_SHIFT) |
		(chip->pdata->topofftshld << MAX8971_TOPOFFTSHLD_SHIFT) |
		(chip->pdata->chgcv << MAX8971_CHGCV_SHIFT));
	rc = max8971_write_reg(chip->client, MAX8971_REG_TOPOFF, data);
	return rc;
}

static int max8971_set_gsm_test_mode(struct max8971_chip *chip)
{
	int rc = 0;

	//unlock charger setting protection bits
	rc = max8971_unlock_setting_protection(chip);
	rc = max8971_set_ifst2p8(chip, 1);
	rc = max8971_write_reg(chip->client,MAX8971_REG_DCCRNT,0x3F);
	rc = max8971_write_reg(chip->client,MAX8971_REG_CHGCNTL1,0x02);
	rc = max8971_write_reg(chip->client,MAX8971_REG_FCHGCRNT,0x1F);

	return rc;
}

static int max8971_enable_charging(struct max8971_chip *chip, int enable)
{
	int rc = 0;
	int usb_sus = (enable ? 0 : 1);
	int dcmon_dis = (chip->pdata->dcmon_dis | usb_sus);
	int data;

	/* Disable AICL when factory mode */
	if (chip->pdata->ifst2p8)
		dcmon_dis = 1;

	data = ((dcmon_dis << MAX8971_DCMON_DIS_SHIFT) |
		(usb_sus << MAX8971_USB_SUS_SHIFT));
	rc = max8971_write_reg(chip->client, MAX8971_REG_CHGCNTL1, data);
	if (rc) {
		dev_err(&chip->client->dev, "cannot %s charging.\n", (enable ? "start" : "stop"));
		return rc;
	}

	if (enable) {
		chip->enable = true;
		chip->status = POWER_SUPPLY_STATUS_CHARGING;
		chip->cable_type = POWER_SUPPLY_TYPE_MAINS;

		/* workaround : monitor charging current & reset charging setting */
		if (!chip->pdata->ifst2p8)
			queue_delayed_work(max8971_wq, &chip->monitor_work, msecs_to_jiffies(MAX8971_MONITOR_MS));
	} else {
		chip->enable = false;
		chip->status = POWER_SUPPLY_STATUS_DISCHARGING;
		chip->cable_type = POWER_SUPPLY_TYPE_BATTERY;
	}

	dev_info(&chip->client->dev, "charging %s.\n", (enable ? "start" : "stop"));

	return rc;
}

static int max8971_get_status(struct max8971_chip *chip)
{
	int rc;
	int data;

	data = max8971_read_reg(chip->client, MAX8971_REG_DETAILS2);
	if (data < 0) {
		dev_err(&chip->client->dev, "cannot read charger details.\n");
		return -EIO;
	}

	data = data & 0x0F;
	dev_info(&chip->client->dev, "charging status : %d\n", data);

	switch (data) {
	case MAX8971_CHG_DTLS_FAST_CHARGE_CC:
	case MAX8971_CHG_DTLS_FAST_CHARGE_CV:
		rc = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case MAX8971_CHG_DTLS_TOP_OFF:
	case MAX8971_CHG_DTLS_DONE:
		rc = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		rc = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	}

	return rc;
}

static int max8971_get_chgcv(struct max8971_chip *chip)
{
	int data;
	int voltage = 4200;

	data = max8971_read_reg(chip->client, MAX8971_REG_TOPOFF);
	if (data < 0) {
		dev_err(&chip->client->dev, "cannot read voltage setting.\n");
		return -EIO;
	}

	data = (data & MAX8971_CHGCV_MASK);
	if (data & 0x1)
		voltage -= 100;
	if (data & 0x2)
		voltage += 150;

	return voltage;
}

static int max8971_set_chgcv(struct max8971_chip *chip, int vol)
{
	const int vol_to_reg[] = {
		4200,
		4100,
		4350,
		4150,
	};
	int rc = 0;
	int data = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(vol_to_reg); i++) {
		if (vol_to_reg[i] == vol)
			break;
	}
	if (i >= ARRAY_SIZE(vol_to_reg)) {
		dev_err(&chip->client->dev,
		"charging voltage %dmV is not supported\n", vol);
		return -EINVAL;
	}
	chip->pdata->chgcv = i;

	data = ((chip->pdata->topofftime << MAX8971_TOPOFFTIME_SHIFT) |
		(chip->pdata->ifst2p8 << MAX8971_IFST2P8_SHIFT) |
		(chip->pdata->topofftshld << MAX8971_TOPOFFTSHLD_SHIFT) |
		(chip->pdata->chgcv << MAX8971_CHGCV_SHIFT));
	rc = max8971_write_reg(chip->client, MAX8971_REG_TOPOFF, data);
	if (rc) {
		dev_err(&chip->client->dev,
		"failed to set charging voltage to %dmV\n", vol);
	}
	chip->voltage = vol;
	return rc;
}

static int max8971_get_dcilmt(struct max8971_chip *chip)
{
	int data;

	data = max8971_read_reg(chip->client, MAX8971_REG_DCCRNT);
	if (data < 0) {
		dev_err(&chip->client->dev, "cannot read dcilmt setting.\n");
		return -EIO;
	}

	data = ((data & MAX8971_DCILMT_MASK) >> MAX8971_DCILMT_SHIFT);
	data = DCILMT_TO_CURRENT(data);

	return data;
}

static int max8971_set_dcilmt(struct max8971_chip *chip, int cur)
{
	int rc = 0;
	int data;

	/* if current limit setting is over 1500mA, disable limit */
	if (cur > 1500) {
		rc = max8971_set_gsm_test_mode(chip);
		chip->current_max = MAX8971_IFST2P8_CURRENT;

		return rc;
	}

	/* If ifst2p8 is enabled, disable it */
	if (chip->pdata->ifst2p8) {
		rc = max8971_set_ifst2p8(chip, 0);
	}

	data = ((DCI_LIMIT(cur) << MAX8971_DCILMT_SHIFT) |
		(chip->pdata->chgrstrt << MAX8971_CHGRSTRT_SHIFT));
	rc = max8971_write_reg(chip->client, MAX8971_REG_DCCRNT, data);
	if (rc) {
		dev_err(&chip->client->dev,
		"failed to set dcilmt to %dmA\n", cur);
		return rc;
	}

	dev_dbg(&chip->client->dev, "set dcilmt to %dmA\n", cur);
	chip->current_max = cur;

	return rc;
}

static int max8971_get_chgcc(struct max8971_chip *chip)
{
	int data;

	data = max8971_read_reg(chip->client, MAX8971_REG_FCHGCRNT);
	if (data < 0) {
		dev_err(&chip->client->dev, "cannot read chgcc setting.\n");
		return -EIO;
	}

	data = ((data & MAX8971_CHGCC_MASK) >> MAX8971_CHGCC_SHIFT);
	data = CHGCC_TO_CURRENT(data);

	return data;
}

static int max8971_set_chgcc(struct max8971_chip *chip, int cur)
{
	int rc = 0;
	int data;

	/* max8971 support charging current up to 1550mA */
	if (cur > 1550) {
		cur = 1550;
	}

	data = ((FCHG_CURRENT(cur) << MAX8971_CHGCC_SHIFT) |
		(chip->pdata->fchgtime << MAX8971_FCHGTIME_SHIFT));
	rc = max8971_write_reg(chip->client, MAX8971_REG_FCHGCRNT, data);
	if (rc) {
		dev_err(&chip->client->dev,
			"failed to set chgcc to %dmA\n", cur);
		return rc;
	}

	dev_dbg(&chip->client->dev, "set chgcc to %dmA\n", cur);
	chip->current_now = cur;

	return rc;
}

static int max8971_reset_registers(struct max8971_chip *chip)
{
	int rc = 0;

	/* Unlock protection */
	rc = max8971_unlock_setting_protection(chip);
	/* Set Interrupt Mask */
	rc = max8971_write_reg(chip->client, MAX8971_REG_CHGINT_MASK, chip->pdata->int_mask);
	/* Set Charging Current */
	rc = max8971_set_chgcc(chip, chip->current_now);
	/* Set Charging Current Limit */
	rc = max8971_set_dcilmt(chip, chip->current_max);
	/* Set Charging Voltage */
	rc = max8971_set_chgcv(chip, chip->voltage);
	/* Start / Stop Charging */
	rc = max8971_enable_charging(chip, chip->enable);

	return rc;
}

static int max8971_get_current(struct max8971_chip *chip)
{
	int adc[4] = {0, 0, 0, 0};
	int rawvalue = 0;
	int adc_vol = 0;
	int chg_cur;
	int rc;

	rc = IMM_auxadc_GetOneChannelValue(AUXADC_CHG_CUR_CHANNEL, adc, &rawvalue);
	if (rc) {
		dev_err(&chip->client->dev, "cannot get charging current.\n");
		return -EIO;
	}

	/* Convert ADC in uV */
	adc_vol = (adc[0] * 100 + adc[1]) * 10000;

	/* Voltage Devide by 47K and 100K */
	chg_cur = (adc_vol * (100 + 47) / 100) / 1400;

	return chg_cur;
}

static int max8971_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int ret = 0;
	struct max8971_chip *chip = container_of(psy, struct max8971_chip, psy);

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = chip->online;
			break;
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = max8971_get_status(chip);
			if (val->intval < 0) {
				ret = val->intval;
				val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			}
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = max8971_get_chgcv(chip);
			if (val->intval < 0) {
				ret = val->intval;
				val->intval = 0;
			}
			break;
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			if (chip->pdata->ifst2p8) {
				val->intval = MAX8971_IFST2P8_CURRENT;
				break;
			}
			val->intval = max8971_get_dcilmt(chip);
			if (val->intval < 0) {
				ret = val->intval;
				val->intval = 0;
			}
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if (chip->pdata->ifst2p8) {
				val->intval = MAX8971_IFST2P8_CURRENT;
				break;
			}
			val->intval = max8971_get_current(chip);
			if (val->intval < 0) {
				ret = val->intval;
				val->intval = 0;
			}
			break;
		default:
			ret = -ENODEV;
			break;
	}
	return ret;
}

static int max8971_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct max8971_chip *chip = container_of(psy, struct max8971_chip, psy);
	int rc = 0;

	dev_dbg(&chip->client->dev, "%s: psp %d, val->intval %d\n",
					__func__, psp, val->intval);

	switch (psp) {
	/* Set charging voltage */
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = max8971_set_chgcv(chip, val->intval);
		break;
	/* Set charging current */
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = max8971_set_dcilmt(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		rc = max8971_set_chgcc(chip, val->intval);
		break;
	/* Enable/Disable charging */
	case POWER_SUPPLY_PROP_STATUS:
		switch(val->intval) {
		case POWER_SUPPLY_STATUS_CHARGING:
			rc = max8971_enable_charging(chip, 1);
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			rc = max8971_enable_charging(chip, 0);
			break;
		default:
			rc = -EINVAL;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_TYPE:
		chip->cable_type = val->intval;
		break;
	default:
		dev_err(&chip->client->dev, "%s: default = 0x%02x\n",
			__func__, chip->cable_type);
		rc = -EINVAL;
	}
	return rc;
}

static int max8971_property_is_writeable(struct power_supply *psy,
				     enum power_supply_property psp)
{
	int rc = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
		case POWER_SUPPLY_PROP_ONLINE:
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		case POWER_SUPPLY_PROP_CURRENT_MAX:
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			rc = 1;
			break;
		default:
			break;
	}

	return rc;
}

static enum power_supply_property max8971_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static char *max8971_supplied_to[] = {
	"battery",
};

static struct power_supply max8971_psy = {
	.name = "max8971",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties	= max8971_props,
	.num_properties = ARRAY_SIZE(max8971_props),
	.get_property	= max8971_get_property,
	.set_property	= max8971_set_property,
	.property_is_writeable = max8971_property_is_writeable,
	.supplied_to = max8971_supplied_to,
	.num_supplicants = ARRAY_SIZE(max8971_supplied_to),
};

static ssize_t max8971_charger_show_registsers(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max8971_chip *chip = dev_get_drvdata(dev);
	int cnt = 0;
	int reg;
	int data;

	for (reg = 0x01; reg <= 0x0A; reg++) {
		data = max8971_read_reg(chip->client, reg);
		if (data < 0) {
			cnt += sprintf(buf + cnt,"register 0x%x = read error\n", reg);
			continue;
		}
		cnt += sprintf(buf + cnt,"register 0x%x = 0x%x\n", reg, data);
	}

	reg = 0x0F;
	data = max8971_read_reg(chip->client, reg);
	if (data < 0) {
		cnt += sprintf(buf + cnt,"register 0x%x = read error\n", reg);
		return cnt;
	}
	cnt += sprintf(buf + cnt,"register 0x%x = 0x%x\n", reg, data);

	return cnt;
}

static ssize_t max8971_charger_store_registsers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max8971_chip *chip = dev_get_drvdata(dev);
	int reg, data;
	int rc;

	rc = sscanf(buf, "%d %d", &reg, &data);
	if (rc)
		return rc;

	rc = max8971_write_reg(chip->client, reg, data);
	if (rc)
		return rc;

	return count;
}
static DEVICE_ATTR(registers, S_IRUGO | S_IWUSR | S_IRGRP | S_IWGRP,
		   max8971_charger_show_registsers,
		   max8971_charger_store_registsers);

static ssize_t max8971_charger_show_debug_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max8971_chip *chip = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", chip->debug_time);
}

static ssize_t max8971_charger_store_debug_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max8971_chip *chip = dev_get_drvdata(dev);
	int enable;
	int rc;

	rc = sscanf(buf, "%d", &enable);
	if (rc)
		return rc;

	if (enable && enable < 500)
		enable = 500;

	chip->debug_time = enable;
	if (enable) {
		queue_delayed_work(max8971_wq, &chip->debug_work, msecs_to_jiffies(chip->debug_time));
	}

	return count;
}
static DEVICE_ATTR(debug_enable, S_IRUGO | S_IWUSR | S_IRGRP | S_IWGRP,
		   max8971_charger_show_debug_enable,
		   max8971_charger_store_debug_enable);

void max8971_enable_intr(struct max8971_chip *chip)
{
    mt_set_gpio_pull_select(chip->gpio, GPIO_PULL_UP);
    mt_eint_unmask(chip->irq);
}

void max8971_disable_intr(struct max8971_chip *chip)
{
    mt_set_gpio_pull_select(chip->gpio, GPIO_PULL_UP);
    mt_eint_mask(chip->irq);
}

static int max8971_charger_detail_irq(int irq, void *data, u8 *val)
{
	struct max8971_chip *chip = (struct max8971_chip *)data;
	int psy_changed = 0;

	switch (irq) {
	case MAX8971_IRQ_PWRUP_OK:
		dev_warn(&chip->client->dev, "Power Up OK Interrupt\n");
		if ((val[0] & MAX8971_DCUVP_MASK) == 0) {
			chip->online = 1;
			// check DCUVP_OK bit in CGH_STAT
			// Vbus is valid
			max8971_reset_registers(chip);
			psy_changed = 1;
		}
		break;

	case MAX8971_IRQ_THM:
		dev_dbg(&chip->client->dev,
			"Thermistor Interrupt: details-0x%x\n",
			(val[1] & MAX8971_THM_DTLS_MASK));
		break;

	case MAX8971_IRQ_BAT:
		dev_dbg(&chip->client->dev,
			"Battery Interrupt: details-0x%x\n",
			(val[2] & MAX8971_BAT_DTLS_MASK));

		switch ((val[2] & MAX8971_BAT_MASK)>>MAX8971_BAT_SHIFT) {
			case MAX8971_BAT_DTLS_BATDEAD:
			    break;
			case MAX8971_BAT_DTLS_TIMER_FAULT:
			    break;
			case MAX8971_BAT_DTLS_BATOK:
			    break;
			case MAX8971_BAT_DTLS_GTBATOV:
			    break;
			default:
			    break;
		}
		break;

	case MAX8971_IRQ_CHG:
		dev_info(&chip->client->dev,
			"Fast Charge Interrupt: details-0x%x\n",
			(val[2] & MAX8971_CHG_DTLS_MASK));

		switch (val[2] & MAX8971_CHG_DTLS_MASK) {
			case MAX8971_CHG_DTLS_DEAD_BAT:
				// insert event if a customer need to do something //
				break;
			case MAX8971_CHG_DTLS_PREQUAL:
				// insert event if a customer need to do something //
				break;
			case MAX8971_CHG_DTLS_FAST_CHARGE_CC:
				// insert event if a customer need to do something //
				break;
			case MAX8971_CHG_DTLS_FAST_CHARGE_CV:
				// insert event if a customer need to do something //
				break;
			case MAX8971_CHG_DTLS_TOP_OFF:
				// insert event if a customer need to do something //
				break;
			case MAX8971_CHG_DTLS_DONE:
				// insert event if a customer need to do something //
				// Charging done and charge off automatically
				break;
			case MAX8971_CHG_DTLS_TIMER_FAULT:
				// insert event if a customer need to do something //
				break;
			case MAX8971_CHG_DTLS_TEMP_SUSPEND:
				// insert event if a customer need to do something //
				break;
			case MAX8971_CHG_DTLS_USB_SUSPEND:
				// insert event if a customer need to do something //
				break;
			case MAX8971_CHG_DTLS_THERMAL_LOOP_ACTIVE:
				// insert event if a customer need to do something //
				break;
			case MAX8971_CHG_DTLS_CHG_OFF:
				// insert event if a customer need to do something //
				power_supply_changed(&chip->psy);
				break;
			default:
				break;
		}

	        break;

	case MAX8971_IRQ_DCUVP:
		if (val[1] & MAX8971_DC_UVP_MASK) {
			chip->online = 1;
			dev_warn(&chip->client->dev, "Cable plugged in.\n");
		} else {
			chip->online = 0;
			dev_warn(&chip->client->dev, "Cable plugged out.\n");
		}
		psy_changed = 1;
		break;

	case MAX8971_IRQ_DCOVP:
		if (val[1] & MAX8971_DC_OVP_MASK) {
			chip->online = 0;
			dev_warn(&chip->client->dev, "Over Voltage on VBUS.\n");
		} else {
			chip->online = 1;
			dev_warn(&chip->client->dev, "Recovered from Over Voltage.\n");
		}
		psy_changed = 1;
		break;

	case MAX8971_IRQ_TOPOFF:
		dev_dbg(&chip->client->dev,
			"Topoff Interrrupt Interrupt: details-0x%x\n",
			(val[1] & MAX8971_DC_I_MASK));
		break;

	case MAX8971_IRQ_AICL:
		if (chip->online) {
			if (val[1] & MAX8971_DC_V_MASK)
				dev_warn(&chip->client->dev, "AICL working.\n");
			else
				dev_warn(&chip->client->dev, "Recovered from AICL mode.\n");
		}
		break;
	}
	return psy_changed;
}

static irqreturn_t max8971_charger_wq(int irq, void *data)
{
	struct max8971_chip *chip = (struct max8971_chip *)data;
	struct i2c_client *client = chip->client;

	int psy_changed = 0;
	int irq_val, irq_mask, irq_name;
	u8 val[3];


	irq_val = max8971_read_reg(client, MAX8971_REG_CHGINT);
	irq_mask = max8971_read_reg(client, MAX8971_REG_CHGINT_MASK);

	val[0] = max8971_read_reg(client, MAX8971_REG_CHG_STAT);
	val[1] = max8971_read_reg(client, MAX8971_REG_DETAILS1);
	val[2] = max8971_read_reg(client, MAX8971_REG_DETAILS2);

	dev_warn(&client->dev, "irq_val = 0x%x\n", irq_val);
	dev_warn(&client->dev, "irq_mask = 0x%x\n", irq_mask);
	dev_warn(&client->dev, "stat = 0x%x\n", val[0]);
	dev_warn(&client->dev, "detail1 = 0x%x\n", val[1]);
	dev_warn(&client->dev, "detail2 = 0x%x\n", val[2]);

	for (irq_name = MAX8971_IRQ_PWRUP_OK; irq_name < MAX8971_NR_IRQS; irq_name++) {
		if ((irq_val & (0x01<<irq_name)) && !(irq_mask & (0x01<<irq_name))) {
			psy_changed |= max8971_charger_detail_irq(irq_name, chip, val);
		}
	}

	if (psy_changed)
		power_supply_changed(&chip->psy);

	return IRQ_HANDLED;
}

static void max8971_work_func(struct work_struct *work)
{
	struct max8971_chip *chip =
		container_of(to_delayed_work(work), struct max8971_chip, dwork);

	max8971_charger_wq(chip->irq, chip);
	max8971_enable_intr(chip);
}

static void max8971_eint_interrupt_handler(void)
{
	max8971_disable_intr(max8971_chg);
	queue_delayed_work(max8971_wq, max8971_work, 0);
}

void max8971_gpio_init(struct max8971_chip *chip)
{
	chip->gpio   = GPIO_EINT_CHG_STAT_PIN;
	chip->irq    = CUST_EINT_CHR_STAT_NUM;
	mt_set_gpio_mode(chip->gpio, GPIO_EINT_CHG_STAT_PIN_M_EINT);
	mt_set_gpio_dir(chip->gpio, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(chip->gpio, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(chip->gpio, GPIO_PULL_UP);
	mt_eint_registration(chip->irq, CUST_EINTF_TRIGGER_LOW, max8971_eint_interrupt_handler, 0);
	max8971_enable_intr(chip);
}

static void max8971_monitor_work(struct work_struct *work)
{
	struct max8971_chip *chip =
		container_of(to_delayed_work(work), struct max8971_chip, monitor_work);
	struct i2c_client *client = chip->client;
	int chg_current;
	int reg, data;

	/* do not check current when factory cable plugged */
	if (chip->pdata->ifst2p8)
		return;

	/* do not check current when charging is not enabled */
	if (!chip->enable)
		return;

	chg_current = max8971_get_current(chip);
	/* if chg_current is under 100mA, there is a problem to charge. */
	if (chg_current < 100) {
		dev_err(&client->dev, "invalid charging current.\n");

		/* dump registers */
		dev_err(&client->dev, "------- MAX8971 Registers -------\n");
		for (reg = 0x00; reg <= 0x0A; reg++) {
			data = max8971_read_reg(client, reg);
			dev_err(&client->dev, "Register : 0x%02X, Value :0x%02X\n", reg, data);
		}
		dev_err(&client->dev, "---------------------------------\n");

		/*reset max8971 setting */
		max8971_enable_charging(chip, 0);

		msleep(50);

		max8971_reset_registers(chip);
		max8971_enable_charging(chip, 1);

		dev_err(&client->dev, "complete reset max8971\n");
	}

	queue_delayed_work(max8971_wq, &chip->monitor_work, msecs_to_jiffies(MAX8971_MONITOR_MS));
}

static void max8971_debug_work(struct work_struct *work)
{
	struct max8971_chip *chip =
		container_of(to_delayed_work(work), struct max8971_chip, debug_work);
	struct i2c_client *client = chip->client;
	int reg, data;

	dev_info(&client->dev, "--------- MAX8971 Debug ---------\n");
	for (reg = 0x00; reg <= 0x0A; reg++) {
		data = max8971_read_reg(client, reg);
		dev_info(&client->dev, "Register : 0x%02X, Value :0x%02X\n", reg, data);
	}
	dev_info(&client->dev, "---------------------------------\n");

	if (chip->debug_time)
		queue_delayed_work(max8971_wq, &chip->debug_work, msecs_to_jiffies(chip->debug_time));
}

static int max8971_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct max8971_chip *chip;
	int ret;

	dev_info(&client->dev, "%s start.\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "i2c check failed\n");
		return -EIO;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->pdata = (struct max8971_platform_data*)client->dev.platform_data;
	if (!chip->pdata) {
		chip->pdata = &max8971_default;
	}

	i2c_set_clientdata(client, chip);
	max8971_chg = chip;

	chip->psy = max8971_psy;
	ret = power_supply_register(&client->dev, &chip->psy);
	if (ret) {
		dev_err(&client->dev, "power supply register failed : %d\n", ret);
		goto err_power_supply_register;
	}

	INIT_DELAYED_WORK(&chip->dwork, max8971_work_func);
	INIT_DELAYED_WORK(&chip->monitor_work, max8971_monitor_work);
	INIT_DELAYED_WORK(&chip->debug_work, max8971_debug_work);
	max8971_work = &chip->dwork;

	max8971_gpio_init(chip);

	/* init variables */
	chip->debug_time = 0;
	chip->online = 0;

	/* read settings */
	chip->pdata->ifst2p8 = max8971_get_ifst2p8(chip);

	chip->current_now = max8971_get_chgcc(chip);
	chip->current_max = max8971_get_dcilmt(chip);
	chip->voltage = max8971_get_chgcv(chip);
	ret = max8971_read_reg(chip->client, MAX8971_REG_CHG_STAT);
	if (ret >= 0) {
		chip->online = (ret & MAX8971_DCUVP_MASK) ? 0 : 1;
		chip->enable = chip->online;
		if (chip->online) {
			// Set IRQ MASK register
			max8971_write_reg(chip->client, MAX8971_REG_CHGINT_MASK, chip->pdata->int_mask);
			//max8971_set_reg(chip, 1);
		}
	} else {
		dev_err(&client->dev, "i2c reading err : %d\n", ret);
		goto err;
	}

	ret = device_create_file(&client->dev, &dev_attr_registers);
	if (ret < 0) {
		dev_err(&client->dev, "device_create_file error! (%d)\n", ret);
		return ret;
	}
	ret = device_create_file(&client->dev, &dev_attr_debug_enable);
	if (ret < 0) {
		dev_err(&client->dev, "device_create_file error! (%d)\n", ret);
		return ret;
	}
	dev_info(&client->dev, "%s finish.\n", __func__);

	if (chip->debug_time)
		queue_delayed_work(max8971_wq, &chip->debug_work, msecs_to_jiffies(chip->debug_time));

	chargin_hw_init_done = KAL_TRUE;

	return 0;

err:
	free_irq(client->irq, chip);

err_power_supply_register:
	i2c_set_clientdata(client, NULL);
	kfree(chip);

	return ret;
}

static int max8971_remove(struct i2c_client *client)
{
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id max8971_id[] = {
	{"max8971", 0},
	{},
};

static struct i2c_driver max8971_i2c_driver = {
	.probe		= max8971_probe,
	.remove		= max8971_remove,
	.driver		= {
		.name	= "max8971",
        .owner	= THIS_MODULE,
	},
	.id_table	= max8971_id,
};

static int __init max8971_init(void)
{
	int rc = 0;

	max8971_wq = create_singlethread_workqueue("max8971_wq");
	if (!max8971_wq) {
		printk("cannot create workqueue for max8971\n");
		return -ENOMEM;
	}

	rc = i2c_add_driver(&max8971_i2c_driver);
	if (rc) {
		printk("[MAX8971]failed to i2c register driver.(%d) \n", rc);
		return -ENODEV;
	}
	return 0;
}

static void __exit max8971_exit(void)
{
	i2c_del_driver(&max8971_i2c_driver);

	if (max8971_wq)
		destroy_workqueue(max8971_wq);
}

module_init(max8971_init);
module_exit(max8971_exit);

static struct i2c_board_info __initdata max8971_i2c_device = {
	I2C_BOARD_INFO ( "max8971", 0x35 ),
};

static int __init max8971_device_init(void)
{

//G3 Stylus and L80+
#if defined(TARGET_MT6582_B2L)
	if(lge_get_board_revno() >= HW_REV_C)
		return 0;
#endif

#if defined(TARGET_MT6582_L80)
    if(lge_get_board_revno() > HW_REV_A)
    return 0;
#endif
    i2c_register_board_info (MAX8971_BUSNUM, &max8971_i2c_device, 1);
	return 0;
}

static void __exit max8971_device_exit(void)
{
	return;
}
module_init(max8971_device_init);
module_exit(max8971_device_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Clark Kim <clark.kim@maxim-ic.com>");
MODULE_DESCRIPTION("Power supply driver for MAX8971");
MODULE_VERSION("3.3");
MODULE_ALIAS("platform:max8971-charger");
