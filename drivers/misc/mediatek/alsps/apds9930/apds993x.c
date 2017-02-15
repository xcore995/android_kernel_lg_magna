/*
 * apds993x.c - Linux kernel modules for ambient light + proximity sensor
 *
 * Copyright (C) 2012 Lee Kai Koon <kai-koon.lee@avagotech.com>
 * Copyright (C) 2012 Avago Technologies
 * Copyright (C) 2013 LGE Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/sensors_io.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/eint.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <cust_eint_md1.h>

#include "apds993x.h"

/*
 * Global data
 */
static struct apds993x_data *pdev_data = NULL;
static struct platform_driver apds993x_alsps_driver;

/* global i2c_client to support ioctl */
static struct i2c_client *apds993x_i2c_client = NULL;
static struct workqueue_struct *apds993x_workqueue = NULL;

/*calibration*/
static int apds993x_cross_talk_val = 0;

static unsigned char apds993x_als_atime_tb[] = { 0xF6, 0xED, 0xDB };
static unsigned short apds993x_als_integration_tb[] = {2720, 5168, 10064};
static unsigned short apds993x_als_res_tb[] = { 10240, 19456, 37888 };
static unsigned char apds993x_als_again_tb[] = { 1, 8, 16, 120 };
static unsigned char apds993x_als_again_bit_tb[] = { 0x00, 0x01, 0x02, 0x03 };

static int apds993x_ga = APDS993X_GA;
static int apds993x_coe_b = APDS993X_COE_B;
static int apds993x_coe_c = APDS993X_COE_C;
static int apds993x_coe_d = APDS993X_COE_D;

#ifdef ALS_POLLING_ENABLED
static int apds993x_set_als_poll_delay(struct i2c_client *client, unsigned int val);
#endif

static void apds993x_setup_eint(void)
{
	APS_FUN();

	/* Configure GPIO settings for external interrupt pin  */
	mt_set_gpio_mode(GPIO_PROXIMITY_INT, GPIO_PROXIMITY_INT_M_EINT);
	mt_set_gpio_dir(GPIO_PROXIMITY_INT, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_PROXIMITY_INT, GPIO_PULL_DISABLE);

	/* Configure external interrupt settings for external interrupt pin */
	mt_eint_set_hw_debounce(CUST_EINT_PROXIMITY_NUM, CUST_EINT_PROXIMITY_DEBOUNCE_EN);
	mt_eint_registration(CUST_EINT_PROXIMITY_NUM, EINTF_TRIGGER_FALLING, apds993x_interrupt, 0);

	/* Mask external interrupt to avoid un-wanted interrupt. Unmask it after initialization of APDS9190 */
	mt_eint_mask ( CUST_EINT_PROXIMITY_NUM );
}

/*
 * Management functions
 */
static int apds993x_set_command(struct i2c_client *client, int command)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	int clearInt;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte(client, clearInt);
	mutex_unlock(&data->update_lock);

	return ret;
}

static int apds993x_set_enable(struct i2c_client *client, int enable)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_ENABLE_REG, enable);
	mutex_unlock(&data->update_lock);

	data->enable = enable;

	return ret;
}

static int apds993x_set_atime(struct i2c_client *client, int atime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_ATIME_REG, atime);
	mutex_unlock(&data->update_lock);

	data->atime = atime;

	return ret;
}

static int apds993x_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_PTIME_REG, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int apds993x_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_WTIME_REG, wtime);
	mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	return ret;
}

static int apds993x_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_AILTL_REG, threshold);
	mutex_unlock(&data->update_lock);

	data->ailt = threshold;

	return ret;
}

static int apds993x_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_AIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);

	data->aiht = threshold;

	return ret;
}

static int apds993x_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_PILTL_REG, threshold);
	mutex_unlock(&data->update_lock);

	data->pilt = threshold;

	return ret;
}

static int apds993x_set_piht(struct i2c_client *client, int threshold)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_PIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);

	data->piht = threshold;

	return ret;
}

static int apds993x_set_pers(struct i2c_client *client, int pers)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_PERS_REG, pers);
	mutex_unlock(&data->update_lock);

	data->pers = pers;

	return ret;
}

static int apds993x_set_config(struct i2c_client *client, int config)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_CONFIG_REG, config);
	mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds993x_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_PPCOUNT_REG, ppcount);
	mutex_unlock(&data->update_lock);

	data->ppcount = ppcount;

	return ret;
}

static int apds993x_set_control(struct i2c_client *client, int control)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client,
			CMD_BYTE|APDS993X_CONTROL_REG, control);
	mutex_unlock(&data->update_lock);

	data->control = control;

	return ret;
}

static int apds993x_get_deviceid(struct i2c_client *client, int *pData)
{
	int id = 0;

	if(pData == NULL) {
		pr_err("invalid input (pVal=NULL)\n");
		return -ENODEV;
	}

	id = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_ID_REG);
	if (id == 0x30) {
		pr_alert("%s: APDS9931\n", __func__);
		*pData = id;
	} else if (id == 0x39) {
		pr_alert("%s: APDS9930\n", __func__);
		*pData = id;
	} else {
		pr_alert("%s: Neither APDS9931 nor APDS9930\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int hwmsen_report_interrupt_data(int devID, int value)
{
	hwm_sensor_data sensor_data;
	int err;

	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	sensor_data.value_divide = 1;
	sensor_data.values[0] = value;

	err = hwmsen_get_interrupt_data(devID, &sensor_data);
	if(err) {
		APS_ERR("failed to send inform (err = %d)\n", err);
		return err;
	}

	return 0;
}

/*calibration*/
void apds993x_swap(int *x, int *y)
{
	int temp = *x;
	*x = *y;
	*y = temp;
}

static int apds993x_run_cross_talk_calibration(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	unsigned int sum_of_pdata = 0;
	unsigned int temp_pdata[20];
	unsigned int ArySize = 20;
	unsigned int cal_check_flag = 0;
	int i, j;
#if defined(APDS993x_SENSOR_DEBUG)
	int status;
	int rdata;
#endif
	pr_info("%s: START proximity sensor calibration\n", __func__);

RECALIBRATION:
	apds993x_set_enable(client, 0x0D);/* Enable PS and Wait */

#if defined(APDS993x_SENSOR_DEBUG)
	mutex_lock(&data->update_lock);
	status = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_STATUS_REG);
	rdata = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG);
	mutex_unlock(&data->update_lock);

	pr_info("%s: APDS993x_ENABLE_REG=%2d APDS993x_STATUS_REG=%2d\n",
			__func__, rdata, status);
#endif

	for (i = 0; i < 20; i++) {
		mdelay(6);
		mutex_lock(&data->update_lock);
		temp_pdata[i] = i2c_smbus_read_word_data(client,
				CMD_WORD|APDS993X_PDATAL_REG);
		mutex_unlock(&data->update_lock);
	}

	/* pdata sorting */
	for (i = 0; i < ArySize - 1; i++)
		for (j = i+1; j < ArySize; j++)
			if (temp_pdata[i] > temp_pdata[j])
				apds993x_swap(temp_pdata + i, temp_pdata + j);

	/* calculate the cross-talk using central 10 data */
	for (i = 5; i < 15; i++) {
		pr_info("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	data->cross_talk = sum_of_pdata/10;
	pr_info("%s: sum_of_pdata = %d   cross_talk = %d\n",
			__func__, sum_of_pdata, data->cross_talk);

	/*
	 * this value is used at Hidden Menu to check
	 * if the calibration is pass or fail
	 */
	data->avg_cross_talk = data->cross_talk;

	if (data->cross_talk > 720) {
		pr_warn("%s: invalid calibrated data\n", __func__);

		if (cal_check_flag == 0) {
			pr_info("%s: RECALIBRATION start\n", __func__);
			cal_check_flag = 1;
			goto RECALIBRATION;
		} else {
			pr_err("%s: CALIBRATION FAIL -> "
			       "cross_talk is set to DEFAULT\n", __func__);
			data->cross_talk = DEFAULT_CROSS_TALK;
			apds993x_set_enable(client, 0x00); /* Power Off */
			data->ps_cal_result = 0; /* 0:Fail, 1:Pass */
			return -EINVAL;
		}
	}

	data->ps_threshold = ADD_TO_CROSS_TALK + data->cross_talk;
	data->ps_hysteresis_threshold =
		data->ps_threshold - SUB_FROM_PS_THRESHOLD;

	apds993x_set_enable(client, 0x00); /* Power Off */
	data->ps_cal_result = 1;

	pr_info("%s: total_pdata = %d & cross_talk = %d\n",
			__func__, sum_of_pdata, data->cross_talk);
	pr_info("%s: FINISH proximity sensor calibration\n", __func__);

	/* Save the cross-talk to the non-volitile memory in the phone  */
	return data->cross_talk;
}

/* apply the Cross-talk value to threshold */
static void apds993x_set_ps_threshold_adding_cross_talk(
		struct i2c_client *client, int cal_data)
{
	struct apds993x_data *data = i2c_get_clientdata(client);

	if (cal_data > 770)
		cal_data = 770;
	if (cal_data < 0)
		cal_data = 0;

	if (cal_data == 0) {
		data->ps_threshold = apds993x_ps_detection_threshold;
		data->ps_hysteresis_threshold =
			data->ps_threshold - SUB_FROM_PS_THRESHOLD;
	} else {
		data->cross_talk = cal_data;
		data->ps_threshold = ADD_TO_CROSS_TALK + data->cross_talk;
		data->ps_hysteresis_threshold =
			data->ps_threshold - SUB_FROM_PS_THRESHOLD;
	}
	pr_info("%s: configurations are set\n", __func__);
}

static int LuxCalculation(struct i2c_client *client, int ch0data, int ch1data)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int luxValue=0;
	int IAC1=0;
	int IAC2=0;
	int IAC=0;

	if ((ch0data >= apds993x_als_res_tb[data->als_atime_index] ||
		ch1data >= apds993x_als_res_tb[data->als_atime_index]) &&
		data->als_reduce) {
		luxValue = 30*1000; // kk 19-Aug-2013, report only als_reduce enabled
		return luxValue;
	}

	/* re-adjust COE_B to avoid 2 decimal point */
	IAC1 = (ch0data - (apds993x_coe_b * ch1data) / 100);
	/* re-adjust COE_C and COE_D to void 2 decimal point */
	IAC2 = ((apds993x_coe_c * ch0data) / 100 -
			(apds993x_coe_d * ch1data) / 100);

	if (IAC1 > IAC2)
		IAC = IAC1;
	else if (IAC1 <= IAC2)
		IAC = IAC2;
	else
		IAC = 0;

	// kk 19-Aug-2013
	if (IAC1 < 0 && IAC2 < 0) {
		if (ch0data < (apds993x_als_res_tb[data->als_atime_index]/2))
			IAC = 0;	/* cdata and irdata saturated */
		else {
			luxValue = 30*1000;
			return -1;	// kk 19-Aug-2013 Don't report max lux, reduce gain may help
		}
	}

	if (data->als_reduce) {
		luxValue = ((IAC * apds993x_ga * APDS993X_DF) / 100) * 65 / 10 /
			((apds993x_als_integration_tb[data->als_atime_index] /
			  100) * apds993x_als_again_tb[data->als_again_index]);
	} else {
		luxValue = ((IAC * apds993x_ga * APDS993X_DF) /100) /
			((apds993x_als_integration_tb[data->als_atime_index] /
			  100) * apds993x_als_again_tb[data->als_again_index]);
	}

	return luxValue;
}

static void apds993x_change_ps_threshold(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int status;

	// kk 29-Aug-2013
	unsigned char psat_bit=0x00;

	status = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_STATUS_REG);

	psat_bit = (status&0x40);

	if (psat_bit == 0x40) {
		// triggered by strong light
		if (data->ps_detection != 0) {
			/* Bring it back to FAR */
			hwmsen_report_interrupt_data(ID_PROXIMITY, PS_FAR);

			i2c_smbus_write_word_data(client,
					CMD_WORD|APDS993X_PILTL_REG, 0);
			i2c_smbus_write_word_data(client,
					CMD_WORD|APDS993X_PIHTL_REG,
					data->ps_threshold);

			data->pilt = 0;
			data->piht = data->ps_threshold;

			pr_info("%s: near-to-far\n", __func__);
		}

		/* 0 = CMD_CLR_PS_INT */
		apds993x_set_command(client, 0);	/* in case it is called by polling */
		return;
	}

	data->ps_data = i2c_smbus_read_word_data(
			client, CMD_WORD|APDS993X_PDATAL_REG);

	if ((data->ps_data > data->pilt) && (data->ps_data >= data->piht)) {
		/* far-to-near detected */
		data->ps_detection = PS_NEAR;

		hwmsen_report_interrupt_data(ID_PROXIMITY, data->ps_detection);

		/* FAR-to-NEAR detection */
		i2c_smbus_write_word_data(client,
				CMD_WORD|APDS993X_PILTL_REG,
				data->ps_hysteresis_threshold);
		i2c_smbus_write_word_data(client,
				CMD_WORD|APDS993X_PIHTL_REG, 1023);

		data->pilt = data->ps_hysteresis_threshold;
		data->piht = 1023;

		pr_info("%s: far-to-near\n", __func__);
	} else if ((data->ps_data <= data->pilt) &&
			(data->ps_data < data->piht)) {
		/* near-to-far detected */
		data->ps_detection = PS_FAR;

		hwmsen_report_interrupt_data(ID_PROXIMITY, data->ps_detection);

		/* NEAR-to-FAR detection */
		i2c_smbus_write_word_data(client,
				CMD_WORD|APDS993X_PILTL_REG, 0);
		i2c_smbus_write_word_data(client,
				CMD_WORD|APDS993X_PIHTL_REG,
				data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		pr_info("%s: near-to-far\n", __func__);
	}
}

static void apds993x_change_als_threshold(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ch0data, ch1data, v;
	int luxValue = 0;
	int err;
	unsigned char change_again = 0;
	unsigned char control_data = 0;
	unsigned char lux_is_valid = 1;
	int adc_low_data;	// kk 19-Aug-2013

	// kk 21-Aug-2013
	int pdata, pilt, piht, enable, status;

	ch0data = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS993X_CH0DATAL_REG);
	ch1data = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS993X_CH1DATAL_REG);

	// kk 21-Aug-2013
	status = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_STATUS_REG);
	enable = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG);
	pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_PDATAL_REG);
	pilt = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_PILTL_REG);
	piht = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_PIHTL_REG);

	luxValue = LuxCalculation(client, ch0data, ch1data);

	if (luxValue >= 0) {
		luxValue = (luxValue < 30000) ? luxValue : 30000;
		data->als_prev_lux = luxValue;
	} else {
		/* don't report, the lux is invalid value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
		if (data->als_reduce) {
			lux_is_valid = 1;
			/* report anyway since this is the lowest gain */
			luxValue = 30000;
		}
	}

	// kk 21-Aug-2013
	pr_info("%s: lux=%d ch0data=%d ch1data=%d pdata=%d again=%d als_reduce=%d status=%d, pilt=%d, piht=%d\n",
			__func__,
			luxValue, ch0data, ch1data, pdata,
			apds993x_als_again_tb[data->als_again_index],
			data->als_reduce, status, pilt, piht);

	/*
	 *  check PS under sunlight
	 * PS was previously in far-to-near condition
	 */
	v = 1024 * (256 - apds993x_als_atime_tb[data->als_atime_index]);
	v = (v * 75) / 100;
	if ((data->ps_detection == PS_NEAR) && (ch0data > v)) {
		/*
		 * need to inform input event as there will be no interrupt
		 * from the PS
		 */
		/* NEAR-to-FAR detection */
		hwmsen_report_interrupt_data(ID_PROXIMITY, PS_FAR);

		i2c_smbus_write_word_data(client,
				CMD_WORD|APDS993X_PILTL_REG, 0);
		i2c_smbus_write_word_data(client,
				CMD_WORD|APDS993X_PIHTL_REG,
				data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		/* near-to-far detected */
		data->ps_detection = PS_FAR;

		pr_info("%s: FAR\n", __func__);
	}

	if (lux_is_valid) {
		/* report the lux level */
		hwmsen_report_interrupt_data(ID_LIGHT, luxValue);
	}

	data->als_data = ch0data;

	// kk 19-Aug-2013
	if (!data->als_reduce) {
		adc_low_data = (apds993x_als_res_tb[data->als_atime_index] * 10)/100;
	}
	else {
		adc_low_data = (apds993x_als_res_tb[data->als_atime_index] * 9)/100;
	}

	data->als_threshold_l = (data->als_data *
			(100 - APDS993X_ALS_THRESHOLD_HSYTERESIS)) / 100;
	data->als_threshold_h = (data->als_data *
			(100 + APDS993X_ALS_THRESHOLD_HSYTERESIS)) / 100;

	if (data->als_threshold_h >=
			apds993x_als_res_tb[data->als_atime_index]) {
		data->als_threshold_h =
			apds993x_als_res_tb[data->als_atime_index];
	}

	if (data->als_data >=
		((apds993x_als_res_tb[data->als_atime_index] * 90 ) / 100)) {
		/* lower AGAIN if possible */
		if (data->als_again_index != APDS993X_ALS_GAIN_1X) {
			data->als_again_index--;
			change_again = 1;
		} else {
			err = i2c_smbus_write_byte_data(client,
					CMD_BYTE|APDS993X_CONFIG_REG,
					APDS993X_ALS_REDUCE);
			if (err >= 0)
				data->als_reduce = 1;
		}
	} else if (data->als_data <= adc_low_data) {// kk 19-Aug-2013
		/* increase AGAIN if possible */
		if (data->als_reduce) {
			err = i2c_smbus_write_byte_data(client,
					CMD_BYTE|APDS993X_CONFIG_REG, 0);
			if (err >= 0)
				data->als_reduce = 0;
		} else if (data->als_again_index != APDS993X_ALS_GAIN_120X) {
			data->als_again_index++;
			change_again = 1;
		}
	}

	if (change_again) {
		control_data = i2c_smbus_read_byte_data(client,
				CMD_BYTE|APDS993X_CONTROL_REG);
		control_data = control_data & 0xFC;
		control_data = control_data |
			apds993x_als_again_bit_tb[data->als_again_index];
		i2c_smbus_write_byte_data(client,
				CMD_BYTE|APDS993X_CONTROL_REG, control_data);
	}

	i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_AILTL_REG, data->als_threshold_l);
	i2c_smbus_write_word_data(client,
			CMD_WORD|APDS993X_AIHTL_REG, data->als_threshold_h);

}

static void apds993x_reschedule_work(struct apds993x_data *data, unsigned long delay)
{
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	__cancel_delayed_work(&data->dwork);
	queue_delayed_work(apds993x_workqueue, &data->dwork, delay);
}


#ifdef ALS_POLLING_ENABLED
/* ALS polling routine */
static void apds993x_als_polling_work_handler(struct work_struct *work)
{
	struct apds993x_data *data = container_of(work,
			struct apds993x_data, als_dwork.work);
	struct i2c_client *client = data->client;
	int ch0data, ch1data, pdata, v;
	int luxValue = 0;
	int err;
	unsigned char change_again = 0;
	unsigned char control_data = 0;
	unsigned char lux_is_valid = 1;

	ch0data = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS993X_CH0DATAL_REG);
	ch1data = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS993X_CH1DATAL_REG);
	pdata = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS993X_PDATAL_REG);

	luxValue = LuxCalculation(client, ch0data, ch1data);

	if (luxValue >= 0) {
		luxValue = luxValue<30000 ? luxValue : 30000;
		data->als_prev_lux = luxValue;
	} else {
		/* don't report, this is invalid lux value */
		lux_is_valid = 0;
		luxValue = data->als_prev_lux;
		if (data->als_reduce) {
			lux_is_valid = 1;
			/* report anyway since this is the lowest gain */
			luxValue = 30000;
		}
	}

#if defined(APDS993x_SENSOR_DEBUG)
	pr_info("%s: lux=%d ch0data=%d ch1data=%d pdata=%d delay=%d again=%d "
		"als_reduce=%d)\n", __func__,
			luxValue, ch0data, ch1data, pdata,
			data->als_poll_delay,
			apds993x_als_again_tb[data->als_again_index],
			data->als_reduce);
#endif

	// kk 19-Aug-2013
	/*
	 * check PS under sunlight
	 * PS was previously in far-to-near condition
	 */
	v = (75 * (1024 * (256 - data->atime))) / 100;
	if ((data->ps_detection == PS_NEAR) && (ch0data > v)) {
		/*
		 * need to inform input event as there will be no interrupt
		 * from the PS
		 */
		/* NEAR-to-FAR detection */
		hwmsen_report_interrupt_data(ID_PROXIMITY, PS_FAR);

		i2c_smbus_write_word_data(client,
				CMD_WORD|APDS993X_PILTL_REG, 0);
		i2c_smbus_write_word_data(client,
				CMD_WORD|APDS993X_PIHTL_REG, data->ps_threshold);

		data->pilt = 0;
		data->piht = data->ps_threshold;

		data->ps_detection = PS_FAR;	/* near-to-far detected */

		pr_info("%s: FAR\n", __func__);
	}

	if (lux_is_valid) {
		/* to report the lux level */
		data->als_lux_value = luxValue;
	}

	data->als_data = ch0data;

	if (data->als_data >=
	    (apds993x_als_res_tb[data->als_atime_index]* 90) / 100) {
		/* lower AGAIN if possible */
		if (data->als_again_index != APDS993X_ALS_GAIN_1X) {
			data->als_again_index--;
			change_again = 1;
		} else {
			err = i2c_smbus_write_byte_data(client,
					CMD_BYTE|APDS993X_CONFIG_REG,
					APDS993X_ALS_REDUCE);
			if (err >= 0)
				data->als_reduce = 1;
		}
	} else if (data->als_data <=
		   (apds993x_als_res_tb[data->als_atime_index] * 10) / 100) {
		/* increase AGAIN if possible */
		if (data->als_reduce) {
			err = i2c_smbus_write_byte_data(client,
					CMD_BYTE|APDS993X_CONFIG_REG, 0);
			if (err >= 0)
				data->als_reduce = 0;
		} else if (data->als_again_index != APDS993X_ALS_GAIN_120X) {
			data->als_again_index++;
			change_again = 1;
		}
	}

	if (change_again) {
		control_data = i2c_smbus_read_byte_data(client,
				CMD_BYTE|APDS993X_CONTROL_REG);
		control_data = control_data & 0xFC;
		control_data = control_data |
			apds993x_als_again_bit_tb[data->als_again_index];
		i2c_smbus_write_byte_data(client,
				CMD_BYTE|APDS993X_CONTROL_REG, control_data);
	}

	/* restart timer */
	queue_delayed_work(apds993x_workqueue,
			&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
}
#endif /* ALS_POLLING_ENABLED */

/* PS interrupt routine */
static void apds993x_work_handler(struct work_struct *work)
{
	struct apds993x_data *data = container_of(work, struct apds993x_data, dwork.work);
	struct i2c_client *client=data->client;
	int status;
	int enable;

	status = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_STATUS_REG);
	enable = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG);

	/* disable 993x's ADC first */
	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG, 1);

	// kk 20-Aug-2013
	if ((status & enable & 0x30) == 0x30) {
		/* both PS and ALS are interrupted */
		apds993x_change_als_threshold(client);

		// kk 29-Aug-2013
		apds993x_change_ps_threshold(client);

		/* 2 = CMD_CLR_PS_ALS_INT */
		apds993x_set_command(client, 2);
	} else if ((status & enable & 0x20) == 0x20) {
	/* only PS is interrupted */
		// kk 29-Aug-2013
		apds993x_change_ps_threshold(client);

		/* 0 = CMD_CLR_PS_INT */
		apds993x_set_command(client, 0);
	} else if ((status & enable & 0x10) == 0x10) {
		/* only ALS is interrupted */
		apds993x_change_als_threshold(client);

		/* 1 = CMD_CLR_ALS_INT */
		apds993x_set_command(client, 1);
	} else {  // kk 20-Aug-2013, unknow state but clear interrupt^M
		apds993x_set_command(client, 0);
	}

	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG, data->enable);

	mt_eint_unmask(CUST_EINT_PROXIMITY_NUM);
}

/* assume this is ISR */
static void apds993x_interrupt(void)
{
	struct apds993x_data *data = pdev_data;

	if(!data) {
		return;
	}

	mt_eint_mask(CUST_EINT_PROXIMITY_NUM);

	apds993x_reschedule_work(data, 0);
}

/*
 * IOCTL support
 */
static int apds993x_enable_als_sensor(struct i2c_client *client, int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);

	pr_debug("%s: val=%d\n", __func__, val);

	if ((val != 0) && (val != 1)) {
		pr_err("%s: invalid value (val = %d)\n", __func__, val);
		return -EINVAL;
	}

	if (val == 1) {
		/* turn on light  sensor */
		if (data->enable_als_sensor == 0) {
			data->enable_als_sensor = 1;
			/* Power Off */
			apds993x_set_enable(client, 0);

#ifdef ALS_POLLING_ENABLED
			if (data->enable_ps_sensor) {
				/* Enable PS with interrupt */
				apds993x_set_enable(client, 0x27);
			} else {
				/* no interrupt*/
				apds993x_set_enable(client, 0x03);
			}
#else
			/*
			 *  force first ALS interrupt in order to
			 * get environment reading
			 */
			apds993x_set_ailt(client, 0xFFFF);
			apds993x_set_aiht(client, 0);

			if (data->enable_ps_sensor) {
				/* Enable PS & ALS with interrupt */
				apds993x_set_enable(client, 0x37);
			} else {
				/* only enable light sensor with interrupt*/
				apds993x_set_enable(client, 0x13);
			}

			/* unmask external interrupt */
			mt_eint_unmask(CUST_EINT_PROXIMITY_NUM);
#endif

#ifdef ALS_POLLING_ENABLED
			/*
			 * If work is already scheduled then subsequent
			 * schedules will not change the scheduled time
			 * that's why we have to cancel it first.
			 */
			__cancel_delayed_work(&data->als_dwork);
			flush_delayed_work(&data->als_dwork);
			queue_delayed_work(apds993x_workqueue, &data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
#endif
		}
	} else {
		/*
		 * turn off light sensor
		 * what if the p sensor is active?
		 */
		data->enable_als_sensor = 0;

		if (data->enable_ps_sensor) {
			/* Power Off */
			apds993x_set_enable(client,0);

			apds993x_set_piht(client, 0);
			apds993x_set_piht(client,
					apds993x_ps_detection_threshold);

			/* only enable prox sensor with interrupt */
			apds993x_set_enable(client, 0x27);
		} else {
			apds993x_set_enable(client, 0);
		}

#ifdef ALS_POLLING_ENABLED
		/*
		 * If work is already scheduled then subsequent schedules
		 * will not change the scheduled time that's why we have
		 * to cancel it first.
		 */
		__cancel_delayed_work(&data->als_dwork);
		flush_delayed_work(&data->als_dwork);
#endif
	}
	return 0;
}

#ifdef ALS_POLLING_ENABLED
static int apds993x_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret;
	int atime_index=0;

	pr_debug("%s: val=%d\n", __func__, val);

	/* minimum 5ms */
	if (val < 3000)
		val = 3000;

	/* convert us => ms */
	data->als_poll_delay = val / 1000;

	/* pre-defined delays from SensorManager
	 * SENSOR_DELAY_NORMAL : 200ms
	 * SENSOR_DELAY_UI : 66.667ms
	 * SENSOR_DELAY_GAME : 20ms
	 * SENSOR_DELAY_FASTEST : 0ms
	 * hwmsen changes the delay to 10ms, less than 10ms
	 */
	if (data->als_poll_delay >= 100)
		atime_index = APDS993X_ALS_RES_37888;
	else if (data->als_poll_delay >= 50)
		atime_index = APDS993X_ALS_RES_19456;
	else
		atime_index = APDS993X_ALS_RES_10240;

	ret = apds993x_set_atime(client, apds993x_als_atime_tb[atime_index]);
	if (ret >= 0) {
		data->als_atime_index = atime_index;
		pr_debug("poll delay %d, atime_index %d\n",
				data->als_poll_delay, data->als_atime_index);
	} else {
		return ret;
	}

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	__cancel_delayed_work(&data->als_dwork);
	flush_delayed_work(&data->als_dwork);
	queue_delayed_work(apds993x_workqueue,
			&data->als_dwork,
			msecs_to_jiffies(data->als_poll_delay));

	return 0;
}
#endif

static int apds993x_enable_ps_sensor(struct i2c_client *client, int val)
{
	struct apds993x_data *data = i2c_get_clientdata(client);

	pr_debug("%s: val=%d\n", __func__, val);

	if ((val != 0) && (val != 1)) {
		pr_err("%s: invalid value=%d\n", __func__, val);
		return -EINVAL;
	}

	if (val == 1) {
		/* turn on p sensor */
		if (data->enable_ps_sensor==0) {
			data->enable_ps_sensor= 1;

			/* Power Off */
			apds993x_set_enable(client,0);

			/* init threshold for proximity */
			apds993x_set_pilt(client, 0);
			apds993x_set_piht(client, apds993x_ps_detection_threshold);

			/*calirbation*/
			apds993x_set_ps_threshold_adding_cross_talk(client, data->cross_talk);
			if (data->enable_als_sensor==0) {
				/* only enable PS interrupt */
				apds993x_set_enable(client, 0x27);
			} else {
#ifdef ALS_POLLING_ENABLED
				/* enable PS interrupt */
				apds993x_set_enable(client, 0x27);
#else
				/* enable ALS and PS interrupt */
				apds993x_set_enable(client, 0x37);
#endif
			}

			apds993x_change_ps_threshold(client);

			/* unmask external interrupt */
			mt_eint_unmask(CUST_EINT_PROXIMITY_NUM);
		}
	} else {
		/*
		 * turn off p sensor - kk 25 Apr 2011
		 * we can't turn off the entire sensor,
		 * the light sensor may be needed by HAL
		 */
		data->enable_ps_sensor = 0;
		if (data->enable_als_sensor) {
#ifdef ALS_POLLING_ENABLED
			/* no ALS interrupt */
			apds993x_set_enable(client, 0x03);

			/*
			 * If work is already scheduled then subsequent
			 * schedules will not change the scheduled time
			 * that's why we have to cancel it first.
			 */
			__cancel_delayed_work(&data->als_dwork);
			flush_delayed_work(&data->als_dwork);
			/* 100ms */
			queue_delayed_work(apds993x_workqueue,
					&data->als_dwork,
					msecs_to_jiffies(data->als_poll_delay));

#else
			/* reconfigute light sensor setting */
			/* Power Off */
			apds993x_set_enable(client,0);
			/* Force ALS interrupt */
			apds993x_set_ailt(client, 0xFFFF);
			apds993x_set_aiht(client, 0);

			/* enable ALS interrupt */
			apds993x_set_enable(client, 0x13);
#endif
		} else {
			apds993x_set_enable(client, 0);
#ifdef ALS_POLLING_ENABLED
			/*
			 * If work is already scheduled then subsequent
			 * schedules will not change the scheduled time
			 * that's why we have to cancel it first.
			 */
			__cancel_delayed_work(&data->als_dwork);
			flush_delayed_work(&data->als_dwork);
#endif
		}
	}
	return 0;
}

static int apds993x_als_ps_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int apds993x_als_ps_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long apds993x_als_ps_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct apds993x_data *data;
	struct i2c_client *client;
	int enable, id;
	int cross_talk, ret = -1;

	if (arg == 0)
		return -EINVAL;

	if (apds993x_i2c_client == NULL) {
		pr_err("%s: i2c driver not installed\n", __func__);
		return -ENODEV;
	}

	client = apds993x_i2c_client;
	data = i2c_get_clientdata(apds993x_i2c_client);

	switch (cmd) {
	case APDS993X_IOCTL_PS_ENABLE:
		ret = copy_from_user(&enable,
				(void __user *)arg, sizeof(enable));
		if (ret) {
			pr_err("%s: ALSPS_SET_PS_MODE: copy_from_user failed\n", __func__);
			return -EFAULT;
		}

		ret = apds993x_enable_ps_sensor(client, enable);
		if (ret < 0)
			return ret;
		break;

	case APDS993X_IOCTL_PS_GET_ENABLE:
		ret = copy_to_user((void __user *)arg,
				&data->enable_ps_sensor,
				sizeof(data->enable_ps_sensor));
		if (ret) {
			pr_err("%s: ALSPS_GET_PS_MODE: copy_to_user failed\n", __func__);
			return -EFAULT;
		}
		break;

	case APDS993X_IOCTL_PS_GET_PDATA:
		data->ps_data =	i2c_smbus_read_word_data(client,
				CMD_WORD|APDS993X_PDATAL_REG);

		ret = copy_to_user((void __user *)arg,
				&data->ps_data, sizeof(data->ps_data));
		if (ret) {
			pr_err("%s: ALSPS_GET_PS_RAW_DATA: copy_to_user failed\n", __func__);
			return -EFAULT;
		}
		break;

	case APDS993X_IOCTL_GET_CALI:
		cross_talk = apds993x_run_cross_talk_calibration(client);
		if(cross_talk < 0) {
			pr_err("%s: ALSPS_IOCTL_GET_CALI: apds993x_run_cross_talk_calibration failed\n", __func__);
			break;
		}
		ret = copy_to_user((void __user *)arg,
				&cross_talk, sizeof(cross_talk));
		if(ret) {
			pr_err("%s: ALSPS_IOCTL_GET_CALI: copy_to_user failed\n", __func__);
			return -EFAULT;
		}
		break;

	case APDS993X_IOCTL_SET_CALI:
		ret = copy_from_user(&cross_talk,
				(void __user*)arg, sizeof(cross_talk));
		if(ret) {
			pr_err("%s: ALSPS_IOCTL_SET_CALI: copy_to_user failed\n", __func__);
			return -EFAULT;
		}

		apds993x_set_ps_threshold_adding_cross_talk(client, cross_talk);
		break;

	case APDS993X_IOCTL_ALS_ENABLE:
		ret = copy_from_user(&enable,
				(void __user *)arg, sizeof(enable));
		if (ret) {
			pr_err("%s: ALS_ENABLE: copy_from_user failed\n",
					__func__);
			return -EFAULT;
		}

		ret = apds993x_enable_als_sensor(client, enable);
		if (ret < 0)
			return ret;
		break;

	case APDS993X_IOCTL_ALS_GET_ENABLE:
		ret = copy_to_user((void __user *)arg,
				&data->enable_als_sensor,
				sizeof(data->enable_als_sensor));
		if (ret) {
			pr_err("%s: ALS_GET_ENABLE: copy_to_user failed\n",
					__func__);
			return -EFAULT;
		}
		break;

#ifdef CONFIG_MACH_LGE
	case ALSPS_GET_PS_DATA:
		ret = copy_to_user((void __user *)arg,
				&data->ps_detection,
				sizeof(data->ps_detection));
		if(ret) {
			pr_err("%s: ALSPS_GET_PS_DATA: copy_to_user failed\n", __func__);
			return -EFAULT;
		}
		break;

	case ALSPS_GET_DEVICEID:
		ret = apds993x_get_deviceid(client, &id);
		if(ret < 0) {
			pr_err("%s: ALSPS_GET_DEVICEID\n", __func__);
			return ret;
		}

		ret = copy_to_user((void __user *)arg, &id, sizeof(id));
		if(ret) {
			pr_err("%s: ALSPS_GET_DEVICEID: copy_to_user failed\n", __func__);
			return -EFAULT;
		}
		break;

	case ALSPS_GET_ALS_DATA:
		ret = copy_to_user((void __user *)arg,
				&data->als_lux_value,
				sizeof(data->als_lux_value));
		if(ret) {
			pr_err("%s: ALSPS_GET_ALS_DATA: copy_to_user failed\n", __func__);
			return -EFAULT;
		}
		break;

#else	// vendor's original code
#if defined(ALS_POLLING_ENABLED)
	case APDS993X_IOCTL_ALS_DELAY:
		ret = copy_from_user(&delay, (void __user *)arg, sizeof(delay));
		if (ret) {
			pr_err("%s: ALS_DELAY: copy_to_user failed\n",
					__func__);
			return -EFAULT;
		}

		ret = apds993x_set_als_poll_delay(client, delay);
		if (ret < 0)
			return ret;
		break;
#endif

	case APDS993X_IOCTL_ALS_GET_CH0DATA:
		data->als_data = i2c_smbus_read_word_data(client,
					CMD_WORD|APDS993X_CH0DATAL_REG);

		ret = copy_to_user((void __user *)arg,
				&data->als_data, sizeof(data->als_data));
		if (ret) {
			pr_err("%s: ALS_GET_CH0DATA: copy_to_user failed\n",
					__func__);
			return -EFAULT;
		}
		break;

	case APDS993X_IOCTL_ALS_GET_CH1DATA:
		data->als_data = i2c_smbus_read_word_data(client,
				CMD_WORD|APDS993X_CH1DATAL_REG);

		ret = copy_to_user((void __user *)arg,
				&data->als_data, sizeof(data->als_data));
		if (ret) {
			pr_err("%s: ALS_GET_CH1DATA: copy_to_user failed\n",
					__func__);
			return -EFAULT;
		}
		break;
#endif

	default:
		pr_warn("%s: unknown ioctl (%d)\n", __func__, cmd);
		break;
	}

    return 0;
}

/*
 * sysFS support
 */
static ssize_t apds993x_show_ch0data(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ch0data;

	mutex_lock(&data->update_lock);
	ch0data = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS993X_CH0DATAL_REG);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", ch0data);
}

static ssize_t apds993x_show_ch1data(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ch1data;

	mutex_lock(&data->update_lock);
	ch1data = i2c_smbus_read_word_data(client,
			CMD_WORD|APDS993X_CH1DATAL_REG);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", ch1data);
}

static ssize_t apds993x_show_pdata(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);
	int pdata;

	mutex_lock(&data->update_lock);
	pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS993X_PDATAL_REG);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", pdata);
}

/*calibration sysfs*/
static ssize_t apds993x_show_status(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);
	int status;
	int rdata;

	mutex_lock(&data->update_lock);
	status = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_STATUS_REG);
	rdata = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS993X_ENABLE_REG);
	mutex_unlock(&data->update_lock);

	pr_info("%s: APDS993x_ENABLE_REG=%2d APDS993x_STATUS_REG=%2d\n",
			__func__, rdata, status);

	return sprintf(buf, "%d\n", status);
}

static ssize_t apds993x_show_ps_run_calibration(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->avg_cross_talk);
}

static ssize_t apds993x_store_ps_run_calibration(struct device_driver *dev, const char *buf, size_t count)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);
	int ret = 0;

	/* start calibration */
	ret = apds993x_run_cross_talk_calibration(client);

	/* set threshold for near/far status */
	data->ps_threshold = data->cross_talk + ADD_TO_CROSS_TALK;
	data->ps_hysteresis_threshold =
		data->ps_threshold - SUB_FROM_PS_THRESHOLD;

	pr_info("%s: [piht][pilt][c_t] = [%d][%d][%d]\n", __func__,
			data->ps_threshold,
			data->ps_hysteresis_threshold,
			data->cross_talk);

	if (ret < 0)
		return ret;

	return count;
}

static ssize_t apds993x_show_ps_default_crosstalk(struct device_driver *dev, char *buf)
{
	return sprintf(buf, "%d\n", DEFAULT_CROSS_TALK);
}

static ssize_t apds993x_store_ps_default_crosstalk(struct device_driver *dev, const char *buf, size_t count)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);

	data->ps_threshold = DEFAULT_CROSS_TALK + ADD_TO_CROSS_TALK;
	data->ps_hysteresis_threshold =
		data->ps_threshold - SUB_FROM_PS_THRESHOLD;

	pr_info("%s: [piht][pilt][c_t] = [%d][%d][%d]\n", __func__,
			data->ps_threshold,
			data->ps_hysteresis_threshold,
			data->cross_talk);

	return count;
}

/* for Calibration result */
static ssize_t apds993x_show_ps_cal_result(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->ps_cal_result);
}
/*calibration sysfs end*/

#ifdef APDS993X_HAL_USE_SYS_ENABLE
static ssize_t apds993x_show_enable_ps_sensor(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds993x_store_enable_ps_sensor(struct device_driver *dev, const char *buf, size_t count)
{
	struct i2c_client *client = apds993x_i2c_client;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	pr_debug("%s: val=%ld\n", __func__, val);

	if (val != 0 && val != 1) {
		pr_err("%s: invalid value(%ld)\n", __func__, val);
		return -EINVAL;
	}

	apds993x_enable_ps_sensor(client, val);

	return count;
}

static ssize_t apds993x_show_enable_als_sensor(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->enable_als_sensor);
}

static ssize_t apds993x_store_enable_als_sensor(struct device_driver *dev, const char *buf, size_t count)
{
	struct i2c_client *client = apds993x_i2c_client;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	pr_debug("%s: val=%ld\n", __func__, val);

	if (val != 0 && val != 1) {
		pr_err("%s: invalid value(%ld)\n", __func__, val);
		return -EINVAL;
	}

	apds993x_enable_als_sensor(client, val);

	return count;
}


static ssize_t apds993x_show_als_poll_delay(struct device_driver *dev, char *buf)
{
	struct i2c_client *client = apds993x_i2c_client;
	struct apds993x_data *data = i2c_get_clientdata(client);

	/* return in micro-second */
	return sprintf(buf, "%d\n", data->als_poll_delay * 1000);
}

static ssize_t apds993x_store_als_poll_delay(struct device_driver *dev, const char *buf, size_t count)
{
#ifdef ALS_POLLING_ENABLED
	struct i2c_client *client = apds993x_i2c_client;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	apds993x_set_als_poll_delay(client, val);
#endif

	return count;
}

static DRIVER_ATTR(enable_ps_sensor, S_IWUSR | S_IWGRP | S_IRUGO,
		apds993x_show_enable_ps_sensor,
		apds993x_store_enable_ps_sensor);
static DRIVER_ATTR(enable_als_sensor, S_IWUSR | S_IWGRP | S_IRUGO,
		apds993x_show_enable_als_sensor,
		apds993x_store_enable_als_sensor);
static DRIVER_ATTR(als_poll_delay, S_IWUSR | S_IWGRP | S_IRUGO,
		apds993x_show_als_poll_delay,
		apds993x_store_als_poll_delay);
#endif

static DRIVER_ATTR(ch0data, S_IRUGO, apds993x_show_ch0data, NULL);
static DRIVER_ATTR(ch1data, S_IRUGO, apds993x_show_ch1data, NULL);
static DRIVER_ATTR(pdata, S_IRUGO, apds993x_show_pdata, NULL);
static DRIVER_ATTR(status, S_IRUSR | S_IRGRP, apds993x_show_status, NULL);
static DRIVER_ATTR(ps_run_calibration,  S_IWUSR | S_IWGRP | S_IRUGO,
				apds993x_show_ps_run_calibration, apds993x_store_ps_run_calibration);
static DRIVER_ATTR(ps_default_crosstalk, S_IRUGO | S_IWUSR | S_IWGRP,
				apds993x_show_ps_default_crosstalk,	apds993x_store_ps_default_crosstalk);
static DRIVER_ATTR(ps_cal_result, S_IRUGO, apds993x_show_ps_cal_result, NULL);

static struct driver_attribute *apds993x_attributes[] = {
	&driver_attr_ch0data,
	&driver_attr_ch1data,
	&driver_attr_pdata,
#ifdef APDS993X_HAL_USE_SYS_ENABLE
	&driver_attr_enable_ps_sensor,
	&driver_attr_enable_als_sensor,
	&driver_attr_als_poll_delay,
#endif
	/*calibration*/
	&driver_attr_status,
	&driver_attr_ps_run_calibration,
	&driver_attr_ps_default_crosstalk,
	&driver_attr_ps_cal_result,
};

static int apds993x_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(apds993x_attributes)/sizeof(apds993x_attributes[0]));

	if(driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++) {
		if(err = driver_create_file(driver, apds993x_attributes[idx])) {
			APS_ERR("driver_create_file (%s) = %d\n", apds993x_attributes[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int apds993x_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(apds993x_attributes)/sizeof(apds993x_attributes[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, apds993x_attributes[idx]);
    }

    return err;
}

static struct file_operations apds993x_als_ps_fops = {
	.owner = THIS_MODULE,
	.open = apds993x_als_ps_open,
	.release = apds993x_als_ps_release,
	.unlocked_ioctl = apds993x_als_ps_unlocked_ioctl,
};

static struct miscdevice apds993x_als_ps_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &apds993x_als_ps_fops,
};

/*
 * Initialization function
 */
static int apds993x_init_client(struct i2c_client *client)
{
	struct apds993x_data *data = i2c_get_clientdata(client);
	int err;
	int id;

	err = apds993x_set_enable(client, 0);
	if (err < 0)
		return err;

	err = apds993x_get_deviceid(client, &id);
	if(err < 0)
		return err;

	/* 100.64ms ALS integration time */
	err = apds993x_set_atime(client,
			apds993x_als_atime_tb[data->als_atime_index]);
	if (err < 0)
		return err;

	/* 2.72ms Prox integration time */
	err = apds993x_set_ptime(client, 0xFF);
	if (err < 0)
		return err;

	/* 2.72ms Wait time */
	err = apds993x_set_wtime(client, 0xFF);
	if (err < 0)
		return err;

	err = apds993x_set_ppcount(client, apds993x_ps_pulse_number);
	if (err < 0)
		return err;

	/* no long wait */
	err = apds993x_set_config(client, 0);
	if (err < 0)
		return err;

	err = apds993x_set_control(client,
			APDS993X_PDRVIE_100MA |
			APDS993X_PRX_IR_DIOD |
			apds993x_ps_pgain |
			apds993x_als_again_bit_tb[data->als_again_index]);

	if (err < 0)
		return err;

	/* init threshold for proximity */
	err = apds993x_set_pilt(client, 0);
	if (err < 0)
		return err;

	err = apds993x_set_piht(client, apds993x_ps_detection_threshold);
	if (err < 0)
		return err;

	/*calirbation*/
	apds993x_set_ps_threshold_adding_cross_talk(client, data->cross_talk);
	data->ps_detection = 0; /* initial value = far*/

	/* force first ALS interrupt to get the environment reading */
	err = apds993x_set_ailt(client, 0xFFFF);
	if (err < 0)
		return err;

	err = apds993x_set_aiht(client, 0);
	if (err < 0)
		return err;
	/* 2 consecutive Interrupt persistence */
	err = apds993x_set_pers(client, APDS993X_PPERS_2 | APDS993X_APERS_2);
	if (err < 0)
		return err;

	/* sensor is in disabled mode but all the configurations are preset */
	return 0;
}

#if !defined(CONFIG_HAS_EARLYSUSPEND)
static int apds993x_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int apds993x_i2c_resume(struct i2c_client *client)
{
	return 0;
}
#endif

static int apds993x_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	APS_FUN();
	strcpy(info->type, APDS993X_DEV_NAME);
	return 0;
}

static void apds993x_main_power(struct alsps_hw *hw, unsigned int on)
{
	static unsigned int main_power_on = 0xFF;

	APS_FUN ();

	if ( main_power_on != on )
	{
		if ( on )
		{
			if ( !hwPowerOn ( hw->power_id, hw->power_vol, "APDS993x" ) )
			{
				APS_ERR ( "failed to power on (APDS993x)\n" );
				goto EXIT_ERR;
			}
			APS_LOG("turned on the power (APDS993x)\n");
		}
		else
		{
			if ( !hwPowerDown ( hw->power_id, "APDS993x" ) )
			{
				APS_ERR ( "failed to power down (APDS993x)\n" );
				goto EXIT_ERR;
			}
			APS_LOG("turned off the power (APDS993x)");
		}

		main_power_on = on;
	}

	EXIT_ERR:
		return;
}

/*--------------------------------------------------------------------------------*/
int apds993x_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct apds993x_data *obj = (struct apds993x_data *)self;

	APS_FUN(f);
	switch (command) {
		case SENSOR_DELAY:
			APS_ERR("apds993x ps delay command!\n");
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				err = -EINVAL;
			}
			else {
				value = *(int *)buff_in;

				if(value) {
					if((err = apds993x_enable_ps_sensor(obj->client, 1))) {
						APS_ERR("enable ps fail: %d\n", err);

						return -1;
					}
					set_bit(APDS_BIT_PS, &obj->enable);
				}
				else {
					if((err = apds993x_enable_ps_sensor(obj->client, 0))) {
						APS_ERR("disable ps fail: %d\n", err);
						return -1;
					}
					clear_bit(APDS_BIT_PS, &obj->enable);
				}
			}
			break;
		case SENSOR_GET_DATA:
			APS_ERR("apds993x ps get data command!\n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data))) {
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else {
				sensor_data = (hwm_sensor_data *)buff_out;
				sensor_data->values[0] = obj->ps_detection;
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("proximity sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
		}

	return err;
}

int apds993x_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
							void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct apds993x_data *obj = (struct apds993x_data *)self;
	APS_FUN(f);

	switch (command)
	{
		case SENSOR_DELAY:
			APS_ERR("apds993x als delay command!\n");
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else {
				/* ALS Integration Time setting as fast as polling rate */
				value = *(int *)buff_in;
				apds993x_set_als_poll_delay(obj->client, value*1000);
			}
			break;
		case SENSOR_ENABLE:
			APS_ERR("apds993x als enable command!\n");
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else {
				value = *(int *)buff_in;
				if(value) {
					if((err = apds993x_enable_als_sensor(obj->client, 1))) {
						APS_ERR("enable als fail: %d\n", err);
						return -1;
					}
					set_bit(APDS_BIT_ALS, &obj->enable);
				}
				else {
					if((err = apds993x_enable_als_sensor(obj->client, 0))) {
						APS_ERR("disable als fail: %d\n", err);
						return -1;
					}
					clear_bit(APDS_BIT_ALS, &obj->enable);
				}
			}
			break;
		case SENSOR_GET_DATA:
			APS_ERR("apds993x als get data command!\n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data))) {
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else {
				sensor_data = (hwm_sensor_data *)buff_out;
				sensor_data->values[0] = obj->als_lux_value;
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
		}
	return err;
}

/*--------------------------------------------------------------------------------*/

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void apds993x_early_suspend(struct early_suspend *h)
{
	APS_FUN ();
}

static void apds993x_late_resume(struct early_suspend *h)
{
	APS_FUN ();
}
#endif

/*
 * I2C probing/exit functions
 */
static int apds993x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct hwmsen_object obj_ps, obj_als;
	struct apds993x_data *data;
	int err = 0;

	APS_FUN();
	pr_debug("%s\n", __func__);

	data = kzalloc(sizeof(struct apds993x_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		err = -ENOMEM;
		goto exit;
	}
	pdev_data = data;

	/* get custom file data struct */
	data->hw = get_cust_alsps_hw();

	data->client = client;
	apds993x_i2c_client = client;

	i2c_set_clientdata(client, data);

	data->enable = 0;	/* default mode is standard */
	data->ps_threshold = apds993x_ps_detection_threshold;
	data->ps_hysteresis_threshold = apds993x_ps_hsyteresis_threshold;
	data->ps_detection = 0;	/* default to no detection */
	data->enable_als_sensor = 0;	// default to 0
	data->enable_ps_sensor = 0;	// default to 0
	data->als_poll_delay = 100;	// default to 100ms
	data->als_atime_index = APDS993X_ALS_RES_37888;	// 100ms ATIME
	data->als_again_index = APDS993X_ALS_GAIN_8X; // 8x AGAIN
	data->als_reduce = 0;	// no ALS 6x reduction
	data->als_prev_lux = 0;

#if defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_drv.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	data->early_drv.suspend = apds993x_early_suspend,
	data->early_drv.resume = apds993x_late_resume,
	register_early_suspend(&data->early_drv);
#endif

	/* calibration */
	if (apds993x_cross_talk_val > 0 && apds993x_cross_talk_val < 1000) {
		data->cross_talk = apds993x_cross_talk_val;
	} else {
		/*
		 * default value: Get the cross-talk value from the memory.
		 * This value is saved during the cross-talk calibration
		 */
		data->cross_talk = DEFAULT_CROSS_TALK;
	}

	mutex_init(&data->update_lock);
	INIT_DELAYED_WORK(&data->dwork, apds993x_work_handler);

#ifdef ALS_POLLING_ENABLED
	INIT_DELAYED_WORK(&data->als_dwork, apds993x_als_polling_work_handler);
#endif

	/* Initialize the APDS993X chip */
	err = apds993x_init_client(client);
	if (err) {
		pr_err("%s: Failed to init apds993x\n", __func__);
		goto exit_kfree;
	}

	/* Register for mise sensor */
	err = misc_register(&apds993x_als_ps_device);
	if (err) {
		pr_err("%s: Unable to register ps ioctl: %d", __func__, err);
		goto exit_kfree;
	}

	err = apds993x_create_attr(&apds993x_alsps_driver.driver);
	if(err) {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
	}

	pr_info("%s: Support ver. %s enabled\n", __func__, DRIVER_VERSION);

	obj_ps.self = pdev_data;
	obj_ps.polling = data->hw->polling_mode_ps;
	obj_ps.sensor_operate = apds993x_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps))) {
		APS_ERR("attach fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	obj_als.self = pdev_data;
	obj_als.polling = data->hw->polling_mode_als;
	obj_als.sensor_operate = apds993x_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als))) {
		APS_ERR("attach fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	APS_ERR("%s: OK\n", __func__);
	return 0;

exit_sensor_obj_attach_fail:
exit_create_attr_failed:
	misc_deregister(&apds993x_als_ps_device);
exit_kfree:
	kfree(data);
	pdev_data = NULL;

exit:
	return err;
}

static int apds993x_i2c_remove(struct i2c_client *client)
{
	int err;
	struct alsps_hw *hw = get_cust_alsps_hw();

	if((err = apds993x_delete_attr(&apds993x_alsps_driver.driver))) {
		APS_ERR("apds993x_delete_attr fail: %d\n", err);
	}

	/* Power down the device */
	apds993x_main_power(hw, 0);

	if((err = misc_deregister(&apds993x_als_ps_device))) {
		APS_ERR("als misc_deregister fail: %d\n", err);
	}

	apds993x_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id apds993x_i2c_id[] = {
	{ APDS993X_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver apds993x_i2c_driver = {
	.probe = apds993x_i2c_probe,
	.remove = apds993x_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = apds993x_i2c_suspend,
	.resume = apds993x_i2c_resume,
#endif
	.detect = apds993x_i2c_detect,
	.id_table = apds993x_i2c_id,
	.driver = {
		.name = APDS993X_DEV_NAME,
	},
};

static struct i2c_board_info __initdata i2c_APDS993X = {
	I2C_BOARD_INFO(APDS993X_DEV_NAME, 0x39)
};

/*
 * Platform Init/probing/exit functions
 */
static int apds993x_probe(struct platform_device *pdev) {
	struct alsps_hw *hw = get_cust_alsps_hw();

	APS_FUN();

	/* Configure external ( GPIO ) interrupt */
	apds993x_setup_eint();

	/* Turn on the power for APDS993x */
	apds993x_main_power(hw, 1);

	/* Add APDS993x as I2C driver */
	if(i2c_add_driver(&apds993x_i2c_driver)) {
		APS_ERR ("failed to add i2c driver\n");
		return -1;
	}
	return 0;
}

static int apds993x_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	APS_FUN();
	/* Turn off the power for APDS993x */
	apds993x_main_power(hw, 0);

	i2c_del_driver(&apds993x_i2c_driver);

	return 0;
}

static struct platform_driver apds993x_alsps_driver = {
	.probe = apds993x_probe,
	.remove = apds993x_remove,
	.driver = {
		.name = "als_ps",
	}
};

static int __init apds993x_init(void)
{
#if 0 // temp for L80P
	struct alsps_hw *hw = get_cust_alsps_hw();

	APS_FUN();

	apds993x_workqueue = create_workqueue("proximity_als");
	if(!apds993x_workqueue) {
		pr_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	i2c_register_board_info(hw->i2c_num, &i2c_APDS993X, 1);
	if(platform_driver_register(&apds993x_alsps_driver)) {
		APS_ERR("failed to register platform driver\n");
		return -ENODEV;
	}
#endif
	return 0;

}

static void __exit apds993x_exit(void)
{
	APS_FUN();

	if(apds993x_workqueue)
		destroy_workqueue(apds993x_workqueue);

	platform_driver_unregister(&apds993x_alsps_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS993X ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
module_init(apds993x_init);
module_exit(apds993x_exit);
