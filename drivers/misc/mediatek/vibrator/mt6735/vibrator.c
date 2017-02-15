/******************************************************************************
 * mt6575_vibrator.c - MT6575 Android Linux Vibrator Device Driver
 *
 * Copyright 2009-2010 MediaTek Co.,Ltd.
 *
 * DESCRIPTION:
 *     This file provid the other drivers vibrator relative functions
 *
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <mach/mt_typedefs.h>
#include <cust_vibrator.h>
#include <mach/upmu_common.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <linux/slab.h>
#include <linux/of.h>


struct vibrator_hw *pvib_cust = NULL;

extern S32 pwrap_read(U32 adr, U32 *rdata);
extern S32 pwrap_write(U32 adr, U32 wdata);

void vibr_Enable_HW(void)
{
	pmic_set_register_value(PMIC_RG_VIBR_EN, 1);	/* [bit 1]: VIBR_EN,  1=enable */
}

void vibr_Disable_HW(void)
{
	pmic_set_register_value(PMIC_RG_VIBR_EN, 0);	/* [bit 1]: VIBR_EN,  1=enable */
}


/******************************************
* Set RG_VIBR_VOSEL	Output voltage select
*  hw->vib_vol:  Voltage selection
* 3'b000: 1.3V
* 3'b001: 1.5V
* 3'b010: 1.8V
* 3'b011: 2.0V
* 3'b100: 2.5V
* 3'b101: 2.8V
* 3'b110: 3.0V
* 3'b111: 3.3V
*******************************************/
struct vibrator_hw *get_cust_vibrator_dtsi(void)
{
	int ret;
	struct device_node *led_node = NULL;
	if (pvib_cust == NULL) {
		pvib_cust = (struct vibrator_hw *)kmalloc(sizeof(struct vibrator_hw), GFP_KERNEL);
		if (pvib_cust == NULL) {
			printk("get_cust_vibrator_dtsi kmalloc fail\n");
			goto out;
		}

		led_node = of_find_compatible_node(NULL, NULL, "mediatek,vibrator");
		if (!led_node) {
			printk("Cannot find vibrator node from dts\n");
			kfree(pvib_cust);
			pvib_cust = NULL;
			goto out;
		} else {
			ret = of_property_read_u32(led_node, "vib_timer", &(pvib_cust->vib_timer));
			if (!ret) {
				printk("The vibrator timer from dts is : %d\n",
				       pvib_cust->vib_timer);
			} else {
				pvib_cust->vib_timer = 25;
			}
#ifdef CUST_VIBR_LIMIT
			ret = of_property_read_u32(led_node, "vib_limit", &(pvib_cust->vib_limit));
			if (!ret) {
				printk("The vibrator limit from dts is : %d\n",
				       pvib_cust->vib_limit);
			} else {
				pvib_cust->vib_limit = 9;
			}
#endif

#ifdef CUST_VIBR_VOL
			ret = of_property_read_u32(led_node, "vib_vol", &(pvib_cust->vib_vol));
			if (!ret) {
				printk("The vibrator vol from dts is : %d\n", pvib_cust->vib_vol);
			} else {
				pvib_cust->vib_vol = 0x05;
			}
#endif
		}
	}

out:
	return pvib_cust;
}

void vibr_power_set(void)
{
#ifdef CUST_VIBR_VOL
	struct vibrator_hw *hw = get_cust_vibrator_dtsi();
	if (hw == NULL) {
		printk
		    ("[vibrator] Can not get the vibrator info from dts,get it from customization folder.\n");
		hw = get_cust_vibrator_hw();
	}
	printk("[vibrator]vibr_init: vibrator set voltage = %d\n", hw->vib_vol);
	pmic_set_register_value(PMIC_RG_VIBR_VOSEL, hw->vib_vol);
#endif
}

struct vibrator_hw *mt_get_cust_vibrator_hw(void)
{
	struct vibrator_hw *hw = get_cust_vibrator_dtsi();
	if (hw == NULL) {
		printk
		    ("[vibrator] Can not get the vibrator info from dts,get it from customization folder.\n");
		return get_cust_vibrator_hw();
	}
	return hw;
}
