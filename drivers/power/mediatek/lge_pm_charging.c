/*****************************************************************************
 *
 * Filename:
 * ---------
 *    lge_pm_charging.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   08 Apr 2014 07:47:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/kernel.h>
#include <mach/battery_common.h>
#include <mach/charging.h>
#include "cust_charging.h"
#include <mach/mt_boot.h>
#include <mach/battery_meter.h>
#ifdef CONFIG_MACH_LGE
#include <mach/board_lge.h>
#endif

// ============================================================ //
//define
// ============================================================ //
//cut off to full
#define POST_CHARGING_TIME	30 * 60 // 30mins

#ifndef CHARGING_FULL_VOLTAGE
#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
#define CHARGING_FULL_VOLTAGE	4300
#else
#define CHARGING_FULL_VOLTAGE	4150
#endif
#endif

#ifndef CHARGER_CV_VOLTAGE
#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
#define CHARGER_CV_VOLTAGE	BATTERY_VOLT_04_350000_V
#else
#define CHARGER_CV_VOLTAGE	BATTERY_VOLT_04_200000_V
#endif
#endif

// ============================================================ //
//global variable
// ============================================================ //
kal_uint32 g_bcct_flag=0;
kal_uint32 g_bcct_value=0;
CHR_CURRENT_ENUM g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
kal_uint32 g_usb_state = USB_UNCONFIGURED;
static bool usb_unlimited=false;

// ============================================================ //
// function prototype
// ============================================================ //

// ============================================================ //
//extern variable
// ============================================================ //
extern int g_platform_boot_mode;

// ============================================================ //
//extern function
// ============================================================ //

// ============================================================ //
void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\n");
#else
	if ((usb_state_value < USB_SUSPEND) || (usb_state_value > USB_CONFIGURED)) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Fail! Restore to default value\n");
		usb_state_value = USB_UNCONFIGURED;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Success! Set %d\n", usb_state_value);
		g_usb_state = usb_state_value;
	}
#endif
}

kal_uint32 get_charging_setting_current(void)
{
	return g_temp_CC_value;
}

bool get_usb_current_unlimited(void)
{
	if (BMT_status.charger_type == STANDARD_HOST || BMT_status.charger_type == CHARGING_HOST)
		return usb_unlimited;
	else
		return false;
}

void set_usb_current_unlimited(bool enable)
{
	usb_unlimited = enable;
}

void select_charging_current_bcct(void)
{
	if ((BMT_status.charger_type == STANDARD_HOST) ||
	(BMT_status.charger_type == NONSTANDARD_CHARGER)) {
		if (g_bcct_value < 100)
			g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 500)
			g_temp_input_CC_value = CHARGE_CURRENT_100_00_MA;
		else if (g_bcct_value < 800)
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		else if (g_bcct_value == 800)
			g_temp_input_CC_value = CHARGE_CURRENT_800_00_MA;
		else
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	} else if ((BMT_status.charger_type == STANDARD_CHARGER) ||
		(BMT_status.charger_type == CHARGING_HOST)) {
		g_temp_input_CC_value = CHARGE_CURRENT_2000_00_MA;

		//---------------------------------------------------
		//set IOCHARGE
		if (g_bcct_value < 550)
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (g_bcct_value < 650)
			g_temp_CC_value = CHARGE_CURRENT_550_00_MA;
		else if (g_bcct_value < 750)
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		else if (g_bcct_value < 850)
			g_temp_CC_value = CHARGE_CURRENT_750_00_MA;
		else if (g_bcct_value < 950)
			g_temp_CC_value = CHARGE_CURRENT_850_00_MA;
		else if (g_bcct_value < 1050)
			g_temp_CC_value = CHARGE_CURRENT_950_00_MA;
		else if (g_bcct_value < 1150)
			g_temp_CC_value = CHARGE_CURRENT_1050_00_MA;
		else if (g_bcct_value < 1250)
			g_temp_CC_value = CHARGE_CURRENT_1150_00_MA;
		else if (g_bcct_value == 1250)
			g_temp_CC_value = CHARGE_CURRENT_1250_00_MA;
		else
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		//---------------------------------------------------
	} else {
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
	}
}

static void pchr_turn_on_charging (void);
kal_uint32 set_bat_charging_current_limit(int current_limit)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] set_bat_charging_current_limit (%d)\n", current_limit);

	if (current_limit != -1) {
		g_bcct_flag=1;
		g_bcct_value = current_limit;

		if(current_limit < 70)
			g_temp_CC_value=CHARGE_CURRENT_0_00_MA;
		else if(current_limit < 200)
			g_temp_CC_value=CHARGE_CURRENT_70_00_MA;
		else if(current_limit < 300)
			g_temp_CC_value=CHARGE_CURRENT_200_00_MA;
		else if(current_limit < 400)
			g_temp_CC_value=CHARGE_CURRENT_300_00_MA;
		else if(current_limit < 450)
			g_temp_CC_value=CHARGE_CURRENT_400_00_MA;
		else if(current_limit < 550)
			g_temp_CC_value=CHARGE_CURRENT_450_00_MA;
		else if(current_limit < 650)
			g_temp_CC_value=CHARGE_CURRENT_550_00_MA;
		else if(current_limit < 700)
			g_temp_CC_value=CHARGE_CURRENT_650_00_MA;
		else if(current_limit < 800)
			g_temp_CC_value=CHARGE_CURRENT_700_00_MA;
		else if(current_limit < 900)
			g_temp_CC_value=CHARGE_CURRENT_800_00_MA;
		else if(current_limit < 1000)
			g_temp_CC_value=CHARGE_CURRENT_900_00_MA;
		else if(current_limit < 1100)
			g_temp_CC_value=CHARGE_CURRENT_1000_00_MA;
		else if(current_limit < 1200)
			g_temp_CC_value=CHARGE_CURRENT_1100_00_MA;
		else if(current_limit < 1300)
			g_temp_CC_value=CHARGE_CURRENT_1200_00_MA;
		else if(current_limit < 1400)
			g_temp_CC_value=CHARGE_CURRENT_1300_00_MA;
		else if(current_limit < 1500)
			g_temp_CC_value=CHARGE_CURRENT_1400_00_MA;
		else if(current_limit < 1600)
			g_temp_CC_value=CHARGE_CURRENT_1500_00_MA;
		else if(current_limit == 1600)
			g_temp_CC_value=CHARGE_CURRENT_1600_00_MA;
		else
			g_temp_CC_value=CHARGE_CURRENT_450_00_MA;
	} else {
		//change to default current setting
		g_bcct_flag=0;
	}

	//wake_up_bat();
	pchr_turn_on_charging();

	return g_bcct_flag;
}

void select_charging_current(void)
{
#ifdef CONFIG_LGE_CABLE_ID_DETECT
	/* Set MAX Charging current when factory cable connected */
	if (BMT_status.factory_cable) {
		g_temp_input_CC_value = CHARGE_CURRENT_2000_00_MA;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		return;
	}
#endif

	switch (BMT_status.charger_type) {
	case CHARGER_UNKNOWN:
		g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
		g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		break;
	case NONSTANDARD_CHARGER:
		g_temp_input_CC_value = NON_STD_AC_CHARGER_CURRENT;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		break;
	case STANDARD_CHARGER:
		g_temp_input_CC_value = AC_CHARGER_CURRENT;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		break;
	default:
		g_temp_input_CC_value = USB_CHARGER_CURRENT;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		break;
	}

#ifdef CONFIG_LGE_PM_CHARGING_SCENARIO
	/* Decrease charging current when battery is too hot */
	if (BMT_status.bat_charging_state == CHR_HOLD) {
		g_temp_CC_value = CHARGE_CURRENT_400_00_MA;
	}
#endif

#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	/* To avoid power-off in ATS test, increase charging current */
	if (BMT_status.usb_current_max_enabled) {
		if (g_temp_input_CC_value < CHARGE_CURRENT_700_00_MA) {
			g_temp_input_CC_value = CHARGE_CURRENT_700_00_MA;
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] USB current MAX Mode enabled.\n");
		}
	}
#endif

#ifdef CONFIG_LGE_PM_PSEUDO_BATTERY
	/* To avoid power-off in ATS test, increase charging current */
	if (BMT_status.pseudo_batt_enabled) {
		if (g_temp_input_CC_value < CHARGE_CURRENT_700_00_MA) {
			g_temp_input_CC_value = CHARGE_CURRENT_700_00_MA;
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] USB current MAX Mode enabled.\n");
		}
	}
#endif

}

static int recharging_check(void)
{
	if (BMT_status.bat_charging_state != CHR_BATFULL)
		return KAL_FALSE;

#ifdef SOC_BY_SW_FG
	if (BMT_status.SOC < 100)
		return KAL_TRUE;
#ifdef CONFIG_LGE_PM_SOC_SCALE
	/* to support soc over 100 */
	if (BMT_status.SOC == 100)
		return KAL_TRUE;
#endif
#endif
	if (BMT_status.voltage_now <= RECHARGING_VOLTAGE || BMT_status.bat_vol <= RECHARGING_VOLTAGE)
		return KAL_TRUE;

	return KAL_FALSE;
}

static int eoc_check(void)
{
	int data = BMT_status.bat_vol;
	int eoc = KAL_FALSE;

	if (!BMT_status.bat_exist)
		return KAL_FALSE;
#ifdef SOC_BY_SW_FG
	if (BMT_status.SOC < 100)
		return KAL_FALSE;
#ifdef CONFIG_LGE_PM_SOC_SCALE
	/* to support soc over 100 */
	if (BMT_status.SOC == 100)
		return KAL_FALSE;
#endif
#endif
	if (BMT_status.bat_vol < CHARGING_FULL_VOLTAGE)
		return KAL_FALSE;

	battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &data);
	eoc = data;

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] EOC = %d\n", eoc);
	if (eoc == KAL_TRUE)
		return KAL_TRUE;

	return KAL_FALSE;
}

static void pchr_turn_on_charging (void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;

	kal_uint32 charging_enable = KAL_TRUE;

	if (BMT_status.bat_charging_state == CHR_ERROR) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Charger Error, turn OFF charging\n");
		charging_enable = KAL_FALSE;
	} else if ((g_platform_boot_mode==META_BOOT) || (g_platform_boot_mode==ADVMETA_BOOT)) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] In meta or advanced meta mode, disable charging.\n");
		charging_enable = KAL_FALSE;
	} else if (BMT_status.bat_charging_state == CHR_BATFULL) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Full, turn OFF charging\n");
		charging_enable = KAL_FALSE;
	} else {
		/*HW initialization*/
		battery_charging_control(CHARGING_CMD_INIT, NULL);

		battery_xlog_printk(BAT_LOG_FULL, "charging_hw_init\n" );

		/* Set Charging Current */
		if (g_bcct_flag == 1) {
			select_charging_current_bcct();

			battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] select_charging_current_bcct\n");
		} else {
			select_charging_current();
		}

		if (g_temp_CC_value == CHARGE_CURRENT_0_00_MA || g_temp_input_CC_value == CHARGE_CURRENT_0_00_MA) {
			charging_enable = KAL_FALSE;

			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] charging current is set 0mA, turn off charging\n");
		} else {
			cv_voltage = CHARGER_CV_VOLTAGE;

			battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT, &g_temp_input_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CURRENT, &g_temp_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);
		}
	}

	/* enable/disable charging */
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] pchr_turn_on_charging(), enable=%d\n", charging_enable);
}

PMU_STATUS BAT_PreChargeModeAction(void)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Pre-CC mode charge, timer=%u on %u\n", BMT_status.PRE_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	if (BMT_status.UI_SOC == 100) {
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	} else if ( BMT_status.bat_vol > V_PRE2CC_THRES ) {
		BMT_status.bat_charging_state = CHR_CC;
	}

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] CC mode charge, timer=%u on %u !!\n", BMT_status.CC_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time += BAT_TASK_PERIOD;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	if (eoc_check() == KAL_TRUE) {
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	}

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryFullAction(void)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery full !!\n");

	BMT_status.bat_full = KAL_TRUE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

	if (recharging_check() == KAL_TRUE) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Re-charging !!\n");

		BMT_status.bat_in_recharging_state = KAL_TRUE;
		BMT_status.bat_full = KAL_FALSE;
		BMT_status.bat_charging_state = CHR_CC;
	}

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryHoldAction(void)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Hold mode !!\n");

	/* Enable charger */
	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryStatusFailAction(void)
{
	kal_uint32 charging_enable;

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BAD Battery status... Charging Stop !!\n");

	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}

void mt_battery_charging_algorithm()
{
	battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
	if (BMT_status.AtCmdChargingModeOff == KAL_TRUE) {
		if (BMT_status.bat_charging_state != CHR_ERROR) {
			BMT_status.bat_charging_state = CHR_ERROR;
			pchr_turn_on_charging();
		}

		battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
		return;
	}
#endif

#ifdef CONFIG_LGE_CABLE_ID_DETECT
	/* If Factory cable plugged, bypass charging algorithm */
	if (BMT_status.factory_cable) {
		if (BMT_status.bat_charging_state != CHR_CC) {
			BMT_status.bat_charging_state = CHR_CC;
			pchr_turn_on_charging();
		}

		battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
		return;
	}
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID
	if (BMT_status.bat_exist && lge_get_battery_id() == BATT_ID_UNKNOWN) {
		/* Invalid battery inserted. Stop charging */
		BMT_status.bat_charging_state = CHR_ERROR;
	}
#endif

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Charging State = 0x%x\n", BMT_status.bat_charging_state);
	switch (BMT_status.bat_charging_state) {
	case CHR_PRE :
		/* Default State */
		BAT_PreChargeModeAction();
		break;
	case CHR_CC :
		/* Normal Charging */
		BAT_ConstantCurrentModeAction();
		break;
	case CHR_BATFULL:
		/* End of Charging */
		BAT_BatteryFullAction();
		break;
	case CHR_HOLD:
		/* Current decreased by OTP */
		BAT_BatteryHoldAction();
		break;
	case CHR_ERROR:
		/* Charging Stop by OTP */
		BAT_BatteryStatusFailAction();
		break;
	default:
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Should not be in here. Check the code.\n");
		BMT_status.bat_charging_state = CHR_PRE;
		break;
	}

	battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
}

