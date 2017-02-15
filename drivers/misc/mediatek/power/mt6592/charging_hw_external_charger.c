/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_hw_external_charger.c
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
 * $Modtime:   11 Aug 2005 10:28:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/



#include <mach/charging.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
#include <cust_charging.h>
#include <linux/power_supply.h>
#ifdef CONFIG_LGE_CABLE_ID_DETECT
#include <mach/board_lge.h>
#endif

// ============================================================ //
// define
// ============================================================ //
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1

// ============================================================ //
// global variable
// ============================================================ //
kal_bool chargin_hw_init_done = KAL_FALSE;
#ifdef CONFIG_LGE_CABLE_ID_DETECT
kal_bool chargin_usb_type_factory = KAL_FALSE;
#endif


// ============================================================ //
// internal variable
// ============================================================ //
static char* support_charger[] = {
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_RT9536
	"rt9536",
#endif
#ifdef CONFIG_MTK_EXTERNAL_CHARGER_MAX8971
	"max8971",
#endif
};
static struct power_supply *power_supply = NULL;

static kal_bool charging_type_det_done = KAL_TRUE;
#ifdef CONFIG_LGE_PM
static kal_bool charging_type_det_retry = KAL_FALSE;
#endif
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;
static DEFINE_MUTEX(cable_detection_lock);


static kal_uint32 g_charging_enabled = 0;
static kal_uint32 g_charging_current = 0;
static kal_uint32 g_charging_current_limit = 0;
static kal_uint32 g_charging_voltage = 0;
static kal_uint32 g_charging_setting_chagned = 0;

// ============================================================ //
// extern variable
// ============================================================ //

// ============================================================ //
// extern function
// ============================================================ //
extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern void mt_power_off(void);

/* Internal APIs for PowerSupply */
static int is_property_support(enum power_supply_property prop)
{
	int support = 0;
	int i;

	if (!power_supply)
		return 0;

	if (!power_supply->get_property)
		return 0;

	for(i = 0; i < power_supply->num_properties; i++) {
		if (power_supply->properties[i] == prop) {
			support = 1;
			break;
		}
	}

	return support;
}

static int is_property_writeable(enum power_supply_property prop)
{
	if (!power_supply->set_property)
		return 0;

	if (!power_supply->property_is_writeable)
		return 0;

	return power_supply->property_is_writeable(power_supply, prop);
}

static int get_property(enum power_supply_property prop, int *data)
{
	union power_supply_propval val;
	int rc = 0;

	*(int*)data = STATUS_UNSUPPORTED;

	if(!is_property_support(prop))
		return 0;

	rc = power_supply->get_property(power_supply, prop, &val);
	if (rc) {
		battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] failed to get property %d\n", prop);
		*(int*)data = 0;
		return rc;
	}

	*(int*)data = val.intval;

	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] set property %d to %d\n", prop, val.intval);
	return rc;
}

static int set_property(enum power_supply_property prop, int data)
{
	union power_supply_propval val;
	int rc = 0;

	if (!is_property_writeable(prop))
		return 0;

	val.intval = data;
	rc = power_supply->set_property(power_supply, prop, &val);
	if (rc) {
		battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] failed to set property %d\n", prop);
	}
	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] set property %d to %d\n", prop, data);
	return rc;
}

/* Internal APIs for cable type detection */
static void hw_bc11_dump_register(void)
{
	kal_uint32 reg_val = 0;
	kal_uint32 reg_num = CHR_CON18;
	kal_uint32 i = 0;

	if (Enable_BATDRV_LOG != BAT_LOG_FULL)
		return;

	for(i=reg_num ; i<=CHR_CON19 ; i+=2)
	{
		reg_val = upmu_get_reg_value(i);
		battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] Chr Reg[0x%x]=0x%x\n", i, reg_val);
	}
}

static void hw_bc11_init(void)
{
	msleep(300);
	Charger_Detect_Init();

	//RG_BC11_BIAS_EN=1
	upmu_set_rg_bc11_bias_en(0x1);
	//RG_BC11_VSRC_EN[1:0]=00
	upmu_set_rg_bc11_vsrc_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//BC11_RST=1
	upmu_set_rg_bc11_rst(0x1);
	//BC11_BB_CTRL=1
	upmu_set_rg_bc11_bb_ctrl(0x1);

	//msleep(10);
	mdelay(50);

	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] hw_bc11_init()\n");
	hw_bc11_dump_register();
}

static U32 hw_bc11_DCD(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_IPU_EN[1.0] = 10
	upmu_set_rg_bc11_ipu_en(0x2);
	//RG_BC11_IPD_EN[1.0] = 01
	upmu_set_rg_bc11_ipd_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=01
	upmu_set_rg_bc11_vref_vth(0x1);
	//RG_BC11_CMP_EN[1.0] = 10
	upmu_set_rg_bc11_cmp_en(0x2);

	//msleep(20);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] hw_bc11_DCD()\n");
	hw_bc11_dump_register();

	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);

	return wChargerAvail;
}

static U32 hw_bc11_stepA1(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_IPU_EN[1.0] = 10
	upmu_set_rg_bc11_ipu_en(0x2);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 10
	upmu_set_rg_bc11_cmp_en(0x2);

	//msleep(80);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] hw_bc11_stepA1()\n");
	hw_bc11_dump_register();

	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);

	return  wChargerAvail;
}

static U32 hw_bc11_stepB1(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_IPU_EN[1.0] = 01
	upmu_set_rg_bc11_ipu_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);

	//msleep(80);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] hw_bc11_stepB1()\n");
	hw_bc11_dump_register();

	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);

	return  wChargerAvail;
}

static U32 hw_bc11_stepC1(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_IPU_EN[1.0] = 01
	upmu_set_rg_bc11_ipu_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);

	//msleep(80);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] hw_bc11_stepC1()\n");
	hw_bc11_dump_register();

	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);

	return  wChargerAvail;
}

static U32 hw_bc11_stepA2(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_VSRC_EN[1.0] = 10
	upmu_set_rg_bc11_vsrc_en(0x2);
	//RG_BC11_IPD_EN[1:0] = 01
	upmu_set_rg_bc11_ipd_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);

	//msleep(80);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] hw_bc11_stepA2()\n");
	hw_bc11_dump_register();

	//RG_BC11_VSRC_EN[1:0]=00
	upmu_set_rg_bc11_vsrc_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);

	return wChargerAvail;
}

static U32 hw_bc11_stepB2(void)
{
	U32 wChargerAvail = 0;

	//RG_BC11_IPU_EN[1:0]=10
	upmu_set_rg_bc11_ipu_en(0x2);
	//RG_BC11_VREF_VTH = [1:0]=10
	upmu_set_rg_bc11_vref_vth(0x1);
	//RG_BC11_CMP_EN[1.0] = 01
	upmu_set_rg_bc11_cmp_en(0x1);

	//msleep(80);
	mdelay(80);

	wChargerAvail = upmu_get_rgs_bc11_cmp_out();

	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] hw_bc11_stepB2()\n");
	hw_bc11_dump_register();

	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	upmu_set_rg_bc11_vref_vth(0x0);

	return wChargerAvail;
}

static void hw_bc11_done(void)
{
	//RG_BC11_VSRC_EN[1:0]=00
	upmu_set_rg_bc11_vsrc_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=0
	upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_IPU_EN[1.0] = 00
	upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_IPD_EN[1.0] = 00
	upmu_set_rg_bc11_ipd_en(0x0);
	//RG_BC11_BIAS_EN=0
	upmu_set_rg_bc11_bias_en(0x0);

	Charger_Detect_Release();

	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] hw_bc11_done()\n");
	hw_bc11_dump_register();
}

#ifdef CONFIG_LGE_CABLE_ID_DETECT
static kal_bool check_factory_cable(void)
{
	usb_cable_type cableId = NO_INIT_CABLE;

	cableId = get_usb_type();
	if (cableId == LT_CABLE_56K  ||
		cableId == LT_CABLE_130K ||
		cableId == LT_CABLE_910K) {
	    if (cableId == LT_CABLE_56K)
			g_AtCmdUsbId = 56;			//for at%usbiadc
	    else if(cableId == LT_CABLE_130K)
			g_AtCmdUsbId = 130;			//for at%usbiadc
	    else if(cableId == LT_CABLE_910K)
			g_AtCmdUsbId = 910;			//for at%usbiadc

		chargin_usb_type_factory = KAL_TRUE;

		battery_xlog_printk(BAT_LOG_CRTI,
			"[PowerSupply] factory cable detected. (%dk)\n", g_AtCmdUsbId);

	} else {
		g_AtCmdUsbId = 0;				//for at%usbiadc

		chargin_usb_type_factory = KAL_FALSE;

		battery_xlog_printk(BAT_LOG_CRTI,
			"[PowerSupply] normal cable detected.\n");

	}

	return chargin_usb_type_factory;
}
#endif

/* Charger Control Interface Handler */
static kal_uint32 charging_hw_init(void *data)
{
	static int hw_initialized = 0;

	if (hw_initialized)
		return STATUS_OK;

	upmu_set_rg_usbdl_set(0);       //force leave USBDL mode
	upmu_set_rg_usbdl_rst(1);	//force leave USBDL mode

	hw_initialized = 1;
	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] initialized.\n");

	return STATUS_OK;
}

static kal_uint32 charging_dump_register(void *data)
{
	/* nothing to do */
	return STATUS_OK;
}

static kal_uint32 charging_enable(void *data)
{
	int status;
	int enable = *(int*)(data);
	int rc = 0;

	if (enable == g_charging_enabled && !g_charging_setting_chagned)
		return STATUS_OK;

	/* Do not disable charging when battery disconnected */
	if (!enable && upmu_get_rgs_baton_undet())
		return STATUS_OK;

	if (enable) {
		status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	rc = set_property(POWER_SUPPLY_PROP_STATUS, status);
	if (rc) {
		battery_xlog_printk(BAT_LOG_CRTI,
			"[PowerSupply] failed to %s charging.(%d)\n",
			(enable ? "start" : "stop"), rc);
		return STATUS_UNSUPPORTED;
	}

	g_charging_enabled = enable;

	/* clear charging setting */
	if (!g_charging_enabled) {
		g_charging_current = 0;
		g_charging_current_limit = 0;
		g_charging_voltage = 0;
	}

	g_charging_setting_chagned = 0;

	battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] %s charging.\n",
				(g_charging_enabled ? "start" : "stop"));

	return STATUS_OK;
}

static kal_uint32 charging_set_cv_voltage(void *data)
{
	int voltage = *(int*)(data);
	int rc = 0;

	/* convert unit uV to mV */
	voltage = voltage / 1000;

	if (voltage == g_charging_voltage)
		return STATUS_OK;

	rc = set_property(POWER_SUPPLY_PROP_VOLTAGE_MAX, voltage);
	if (rc) {
		battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] Set CV Voltage failed.(%d)\n", rc);
		return STATUS_UNSUPPORTED;
	}

	g_charging_voltage = voltage;
	g_charging_setting_chagned = 1;

	battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] Set CV Voltage to %dmV\n", g_charging_voltage);

	return STATUS_OK;
}

static kal_uint32 charging_get_current(void *data)
{
	int cur;
	int rc = 0;

	if (!g_charging_enabled) {
		*(int*)data = 0;
		return STATUS_OK;
	}

	rc = get_property(POWER_SUPPLY_PROP_CURRENT_NOW, &cur);
	if (rc)
		*(int*)data = min(g_charging_current, g_charging_current_limit);
	else if (cur < 0)
		*(int*)data = min(g_charging_current, g_charging_current_limit);
	else
		*(int*)data = cur;

	/* match unit with CHR_CURRENT_ENUM */
	*(int*)data *= 100;

	return STATUS_OK;
}

static kal_uint32 charging_set_current(void *data)
{
	enum power_supply_property prop = POWER_SUPPLY_PROP_CURRENT_NOW;
	int cur = *(int*)(data);
	int rc = 0;

	/* convert unit to mA */
	cur = cur / 100;

	/* charging current & current limit is not separated */
	if (!is_property_writeable(POWER_SUPPLY_PROP_CURRENT_NOW)) {
		prop = POWER_SUPPLY_PROP_CURRENT_MAX;

		/* use current limit value here */
		if (cur > g_charging_current_limit) {
			cur = g_charging_current_limit;
		}
	}

	if (cur == g_charging_current)
		return STATUS_OK;

	rc = set_property(prop, cur);
	if (rc) {
		battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] Set Current failed.(%d)\n", rc);
		return STATUS_UNSUPPORTED;
	}

	g_charging_current = cur;
	g_charging_setting_chagned = 1;

	battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] Set Current to %dmA\n", g_charging_current);

	return STATUS_OK;
}

static kal_uint32 charging_set_input_current(void *data)
{
	int cur = *(int*)(data);
	int rc = 0;

	/* convert unit to mA */
	cur = cur / 100;

	if (cur == g_charging_current_limit)
		return STATUS_OK;

	rc = set_property(POWER_SUPPLY_PROP_CURRENT_MAX, cur);
	if (rc) {
		battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] Set Current Limit failed.(%d)\n", rc);
		return STATUS_UNSUPPORTED;
	}

	g_charging_current_limit = cur;
	g_charging_setting_chagned = 1;

	battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] Set Current Limit to %dmA\n", g_charging_current_limit);

	return STATUS_OK;
}

static kal_uint32 charging_get_charging_status(void *data)
{
	int voltage = *(int*)data;
	int status;
	int rc = 0;

	if (g_charger_type == CHARGER_UNKNOWN) {
		*(int*)data = 0;
		return STATUS_OK;
	}

	rc = get_property(POWER_SUPPLY_PROP_STATUS, &status);
	if (rc) {
		battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] failed to get charging status.(%d)\n", rc);
		return STATUS_UNSUPPORTED;
	}

	/* if eoc check is not supported in charger ic, check battery voltage instead */
	if (status == STATUS_UNSUPPORTED) {
		/* battery voltage is invalid range */
		if (voltage > 5000) {
			*(int*)data = 0;
			return STATUS_OK;
		}

		if (voltage > RECHARGING_VOLTAGE)
			*(int*)data = 1;
		else
			*(int*)data = 0;

		return STATUS_OK;
	}

	if (status == POWER_SUPPLY_STATUS_FULL) {
		*(int*)data = 1;
	} else {
		*(int*)data = 0;
	}
	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] End of Charging : %d\n", *(int*)data);

	return STATUS_OK;
}

static kal_uint32 charging_reset_watch_dog_timer(void *data)
{
	/* nothing to do */
	return STATUS_OK;
}

static kal_uint32 charging_set_hv_threshold(void *data)
{
	/* nothing to do */
	return STATUS_OK;
}

static kal_uint32 charging_get_hv_status(void *data)
{
	/* nothing to do */
	*(kal_bool*)data = KAL_FALSE;
	return STATUS_OK;
}

static kal_uint32 charging_get_battery_status(void *data)
{
	kal_bool status = KAL_TRUE;

	upmu_set_baton_tdet_en(1);
	upmu_set_rg_baton_en(1);
#ifdef CONFIG_LGE_PM
	if (upmu_get_rgs_baton_undet())
		*(kal_bool*)(data) = KAL_FALSE;
	else
		*(kal_bool*)(data) = KAL_TRUE;
#else
	*(kal_bool*)(data) = upmu_get_rgs_baton_undet();
#endif

	return STATUS_OK;
}

static kal_uint32 charging_get_charger_det_status(void *data)
{
	int online;
	int rc;

	if (upmu_get_rgs_chrdet() == 0) {
		*(kal_bool*)(data) = KAL_FALSE;
		g_charger_type = CHARGER_UNKNOWN;
#ifdef CONFIG_LGE_PM
		charging_type_det_retry = KAL_FALSE;
#endif
	} else {
		*(kal_bool*)(data) = KAL_TRUE;
		rc = get_property(POWER_SUPPLY_PROP_ONLINE, &online);
		if (rc) {
			/* error reading online status. use pmic value */
			battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] cannot read online.\n");
			return STATUS_OK;
		}

		if (!online) {
			/* OVP detected in external charger */
			*(kal_bool*)(data) = KAL_FALSE;
			g_charger_type = CHARGER_UNKNOWN;
		}
	}

	battery_xlog_printk(BAT_LOG_FULL, "[PowerSupply] g_charger_type = %d\n", g_charger_type);

	return STATUS_OK;
}

static kal_uint32 charging_get_charger_type(void *data)
{
	kal_uint32 status = STATUS_OK;
#if defined(CONFIG_POWER_EXT)
	*(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
	mutex_lock(&cable_detection_lock);

	if (g_charger_type != CHARGER_UNKNOWN && g_charger_type != WIRELESS_CHARGER)
	{
		*(CHARGER_TYPE*)(data) = g_charger_type;
		battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] return %d!\n", g_charger_type);

		mutex_unlock(&cable_detection_lock);

		return status;
	}

	charging_type_det_done = KAL_FALSE;

	/********* Step initial  ***************/
	hw_bc11_init();

	/********* Step DCD ***************/
	if(1 == hw_bc11_DCD())
	{
		/********* Step A1 ***************/
		if(1 == hw_bc11_stepA1())
		{
			/********* Step B1 ***************/
			if(1 == hw_bc11_stepB1())
			{
				*(CHARGER_TYPE*)(data) = NONSTANDARD_CHARGER;
				battery_xlog_printk(BAT_LOG_CRTI, "step B1 : Non STANDARD CHARGER!\n");
			}
			else
			{
				*(CHARGER_TYPE*)(data) = APPLE_2_1A_CHARGER;
				battery_xlog_printk(BAT_LOG_CRTI, "step B1 : Apple 2.1A CHARGER!\n");
			}
		}
		else
		{
			/********* Step C1 ***************/
			if(1 == hw_bc11_stepC1())
			{
				*(CHARGER_TYPE*)(data) = APPLE_1_0A_CHARGER;
				battery_xlog_printk(BAT_LOG_CRTI, "step C1 : Apple 1A CHARGER!\n");
			}
			else
			{
				*(CHARGER_TYPE*)(data) = APPLE_0_5A_CHARGER;
				battery_xlog_printk(BAT_LOG_CRTI, "step C1 : Apple 0.5A CHARGER!\n");
			}
		}
	}
	else
	{
		/********* Step A2 ***************/
		if(1 == hw_bc11_stepA2())
		{
			/********* Step B2 ***************/
			if(1 == hw_bc11_stepB2())
			{
				*(CHARGER_TYPE*)(data) = STANDARD_CHARGER;
				battery_xlog_printk(BAT_LOG_CRTI, "step B2 : STANDARD CHARGER!\n");
			}
			else
			{
				*(CHARGER_TYPE*)(data) = CHARGING_HOST;
				battery_xlog_printk(BAT_LOG_CRTI, "step B2 : Charging Host!\n");
			}
		}
		else
		{
			*(CHARGER_TYPE*)(data) = STANDARD_HOST;
			battery_xlog_printk(BAT_LOG_CRTI, "step A2 : Standard USB Host!\n");
		}
	}

	/********* Finally setting *******************************/
	hw_bc11_done();

#ifdef CONFIG_LGE_CABLE_ID_DETECT
	check_factory_cable();
#endif

#ifdef CONFIG_LGE_PM
	/* we do not support apple TA, so consider this as non-standard charger */
	if (*(CHARGER_TYPE*)(data) == APPLE_2_1A_CHARGER ||
		*(CHARGER_TYPE*)(data) == APPLE_1_0A_CHARGER ||
		*(CHARGER_TYPE*)(data) == APPLE_0_5A_CHARGER) {
		*(CHARGER_TYPE*)(data) = NONSTANDARD_CHARGER;
	}

	if (*(CHARGER_TYPE*)(data) == NONSTANDARD_CHARGER) {
		if (charging_type_det_retry == KAL_FALSE) {
			*(CHARGER_TYPE*)(data) = CHARGER_UNKNOWN;
			charging_type_det_retry = KAL_TRUE;
		}
#ifdef CONFIG_LGE_CABLE_ID_DETECT
		/* if cable is for factory, consider as usb cable */
		if (chargin_usb_type_factory == KAL_TRUE) {
			*(CHARGER_TYPE*)(data) = STANDARD_HOST;
			charging_type_det_retry = KAL_FALSE;
		}
#endif
	}
#endif

	charging_type_det_done = KAL_TRUE;

	g_charger_type = *(CHARGER_TYPE*)(data);
#endif

	mutex_unlock(&cable_detection_lock);

	return STATUS_OK;
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
	kal_uint32 status = STATUS_OK;

	if(slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool*)(data) = KAL_TRUE;
	else
	*(kal_bool*)(data) = KAL_FALSE;

	battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] slp_get_wake_reason=%d\n", slp_get_wake_reason());

	return status;
}

static kal_uint32 charging_set_platform_reset(void *data)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] charging_set_platform_reset\n");

	arch_reset(0,NULL);

	return STATUS_OK;
}

static kal_uint32 charging_get_platfrom_boot_mode(void *data)
{
	*(kal_uint32*)(data) = get_boot_mode();

	battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] get_boot_mode=%d\n", get_boot_mode());

	return STATUS_OK;
}

static kal_uint32 charging_set_power_off(void *data)
{
	kal_uint32 status = STATUS_OK;

	battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] charging_set_power_off\n");
	mt_power_off();

	return status;
}

static kal_uint32 charging_get_power_source(void *data)
{
	kal_uint32 status = STATUS_OK;

	*(kal_bool *)data = KAL_FALSE;

	return status;
}
static kal_uint32 charging_get_csdac_full_flag(void *data)
{
	kal_uint32 status = STATUS_OK;

	*(kal_bool *)data = KAL_FALSE;

	return status;
}
static kal_uint32 charging_set_ta_current_pattern(void *data)
{
	kal_uint32 status = STATUS_OK;

	return status;
}

static kal_uint32 charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

static kal_uint32 (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
{
	 charging_hw_init
	,charging_dump_register
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
	,charging_set_power_off
	,charging_get_power_source			// not support, empty function
	,charging_get_csdac_full_flag		// not support, empty function
	,charging_set_ta_current_pattern	// not support, empty function
	,charging_set_error_state			// not support, empty function
};

/*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION
 *		 This function is called to set the charger hw
 *
 * CALLS
 *
 * PARAMETERS
 *		None
 *
 * RETURNS
 *
 *
 * GLOBALS AFFECTED
 *	   None
 */
kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	kal_int32 status;
	int i;

	switch(cmd) {
	/* these commands does not need power_supply, so jump to do_cmd */
	case CHARGING_CMD_INIT:
	case CHARGING_CMD_RESET_WATCH_DOG_TIMER:
	case CHARGING_CMD_SET_HV_THRESHOLD:
	case CHARGING_CMD_GET_HV_STATUS:
	case CHARGING_CMD_GET_BATTERY_STATUS:
	case CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER:
	case CHARGING_CMD_SET_PLATFORM_RESET:
	case CHARGING_CMD_GET_PLATFORM_BOOT_MODE:
	case CHARGING_CMD_SET_POWER_OFF:
	case CHARGING_CMD_GET_POWER_SOURCE:
	case CHARGING_CMD_GET_CSDAC_FALL_FLAG:
	case CHARGING_CMD_SET_TA_CURRENT_PATTERN:
	case CHARGING_CMD_SET_ERROR_STATE:
		goto chr_control_interface_do_cmd;
		break;
	default:
		break;
	}

	if (!power_supply) {
		/* find charger */
		for (i = 0; i < ARRAY_SIZE(support_charger); i++) {
			power_supply = power_supply_get_by_name(support_charger[i]);
			if (power_supply)
				break;
		}

		/* if not found, cannot control charger */
		if (!power_supply) {
			battery_xlog_printk(BAT_LOG_CRTI, "[PowerSupply] failed to get power_supply.\n");
			return STATUS_UNSUPPORTED;
		}

		/* If power source is attached, assume that device is charing now */
		if (upmu_get_rgs_chrdet()) {
			g_charging_enabled = true;
		}
	}

chr_control_interface_do_cmd:
	status = charging_func[cmd](data);

	return status;
}

