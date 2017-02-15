/*
 * MAX8971 Charger Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MAX8971_CHARGER_H
#define __LINUX_MAX8971_CHARGER_H


#define FCHG_CURRENT(x) ((x-250)/50+5)  // 0 and from 250mA to 1550mA in 50mA steps.
#define CHGCC_TO_CURRENT(x) ((((x)-5)*50)+250)

enum {
    MAX8971_FCHGTIME_DISABLE,
    MAX8974_FCHGTIME_4HRS,
    MAX8974_FCHGTIME_5HRS,
    MAX8974_FCHGTIME_6HRS,
    MAX8974_FCHGTIME_7HRS,
    MAX8974_FCHGTIME_8HRS,
    MAX8974_FCHGTIME_9HRS,
    MAX8974_FCHGTIME_10HRS,
};

enum {
    MAX8971_TOPOFFTIME_0MIN,
    MAX8971_TOPOFFTIME_10MIN,
    MAX8971_TOPOFFTIME_20MIN,
    MAX8971_TOPOFFTIME_30MIN,
    MAX8971_TOPOFFTIME_40MIN,
    MAX8971_TOPOFFTIME_50MIN,
    MAX8971_TOPOFFTIME_60MIN,
    MAX8971_TOPOFFTIME_70MIN,
};

enum {
    MAX8971_TOPOFFTSHLD_50mA,
    MAX8971_TOPOFFTSHLD_100mA,
    MAX8971_TOPOFFTSHLD_150mA,
    MAX8971_TOPOFFTSHLD_200mA,
};

enum {
    MAX8971_CHGCV_4P20V,
    MAX8971_CHGCV_4P10V,
    MAX8971_CHGCV_4P35V,
};

enum {
    MAX8971_CHGRSTRT_150mV,
    MAX8971_CHGRSTRT_100mV,
};

#define DCI_LIMIT(x) ((x<=100) ? 0 : \
		(x>=250 && x<=1550) ? ((x-250)/25+10) : -EINVAL)
#define DCILMT_TO_CURRENT(x) (((x)<10)?(100):((((x)-10)*25)+250))

enum {
    MAX8971_REGTEMP_105degree,
    MAX8971_REGTEMP_90degree,
    MAX8971_REGTEMP_120degree,
    MAX8971_REGTEMP_DISABLE,
};

enum {
    MAX8971_THM_CNFG_CONTINUOUS,
    MAX8971_THM_CNFG_NOT_MONITOR,
};

enum {
    MAX8971_SAFETYREG_REGION1,
    MAX8971_SAFETYREG_REGION2,
};

#define MAX8971_CHGPROT_LOCKED      0x00
#define MAX8971_CHGPROT_UNLOCKED    0x03

/* IRQ definitions */
enum {
	MAX8971_IRQ_PWRUP_OK =0,
	MAX8971_IRQ_THM,
	MAX8971_IRQ_BAT,
	MAX8971_IRQ_CHG,
	MAX8971_IRQ_DCUVP,
	MAX8971_IRQ_DCOVP,
	MAX8971_IRQ_TOPOFF,
	MAX8971_IRQ_AICL,
	MAX8971_NR_IRQS,
};

struct max8971_platform_data {
	/* Default Register Setting */
	u8 int_mask;		/* Interrupt Mask */
	u8 usb_sus;		/* USB Suspend  */
	u8 dcmon_dis;		/* DC Monitor Disable */
	u8 chgcc;		/* Fast Charging Current Selection */
	u8 fchgtime;		/* Fast Charging Timer Duration */
	u8 dcilmt;		/* Input-Current Limit Selection */
	u8 chgrstrt;		/* Fast Charge Restart Threshold */
	u8 chgcv;		/* Charging Termination Voltage */
	u8 topofftshld;		/* Topoff Current Threshold */
	u8 ifst2p8;		/* Scales Maximum Fast Charge Current to 2.8A */
	u8 topofftime;		/* Topoff Timer */
	u8 safetyreg;		/* JEITA Safety Region Selection */
	u8 thm_cnfg;		/* Thermistor Monitor */
	u8 regtemp;		/* Die-Temperature Thernaml Regulation Loop Setpoint */
	u8 cprot;		/* Charger-Setting Protection */

	/* Board Setting */
	int gpio_irq;
};
#endif

