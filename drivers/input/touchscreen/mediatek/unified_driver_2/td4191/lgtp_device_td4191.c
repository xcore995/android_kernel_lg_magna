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
 *    File  	: lgtp_device_s3320.c
 *    Author(s)   : D3 BSP Touch Team < d3-bsp-touch@lge.com >
 *    Description :
 *
 ***************************************************************************/
#define LGTP_MODULE "[TD4191]"

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/input/unified_driver_2/lgtp_common.h>

#include <linux/input/unified_driver_2/lgtp_common_driver.h>
#include <linux/input/unified_driver_2/lgtp_platform_api.h>
#include <linux/input/unified_driver_2/lgtp_device_td4191.h>
#include <mach/mt_wdt.h>
#include <mach/wd_api.h>
#include <linux/file.h>    
#include <linux/syscalls.h> /*for file access*/
#include <linux/uaccess.h>  /*for file access*/
#include "RefCode_F54.h"

extern int mtk_wdt_enable ( enum wk_wdt_en en );

/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
/* RMI4 spec from 511-000405-01 Rev.D
 * Function	Purpose				See page
 * $01		RMI Device Control		45
 * $1A		0-D capacitive button sensors	61
 * $05		Image Reporting			68
 * $07		Image Reporting			75
 * $08		BIST				82
 * $09		BIST				87
 * $11		2-D TouchPad sensors		93
 * $19		0-D capacitive button sensors	141
 * $30		GPIO/LEDs			148
 * $31		LEDs				162
 * $34		Flash Memory Management		163
 * $36		Auxiliary ADC			174
 * $54		Test Reporting			176
 */
struct synaptics_ts_f12_info {
   bool ctrl_reg_is_present[32];
   bool data_reg_is_present[16];
   u8 ctrl_reg_addr[32];
   u8 data_reg_addr[16];
};
u8 default_finger_amplitude[2] = {0, };    
static struct synaptics_ts_f12_info f12_info;
struct i2c_client *ds4_i2c_client;
static bool need_scan_pdt = true;
#define RMI_DEVICE_CONTROL			0x01
#define TOUCHPAD_SENSORS			0x12
#define FLASH_MEMORY_MANAGEMENT		0x34
#define VIDEO_CONTROL				0x38
#define CUSTOM_CONTROL				0x51
#define ANALOG_CONTROL				0x54
#define SENSOR_CONTROL				0x55

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
#define DEVICE_CONTROL_REG 			(ts->common_fc.dsc.control_base)
#define MANUFACTURER_ID_REG			(ts->common_fc.dsc.query_base)
#define CUSTOMER_FAMILY_REG			(ts->common_fc.dsc.query_base+2)
#define FW_REVISION_REG				(ts->common_fc.dsc.query_base+3)
#define PRODUCT_ID_REG				(ts->common_fc.dsc.query_base+11)
#define DEVICE_COMMAND_REG		    (ts->common_fc.dsc.command_base)

#define COMMON_PAGE					(ts->common_fc.function_page)
#define LPWG_PAGE					(ts->custom_fc.function_page)
#define FINGER_PAGE					(ts->finger_fc.function_page)
#define ANALOG_PAGE					(ts->analog_fc.function_page)
#define FLASH_PAGE					(ts->flash_fc.function_page)
#define VIDEO_PAGE                  (ts->video_fc.function_page)
#define SENSOR_PAGE					(ts->sensor_fc.function_page)

#define DEVICE_STATUS_REG			(ts->common_fc.dsc.data_base)
#define DEVICE_FAILURE_MASK		0x03

#define INTERRUPT_STATUS_REG		(ts->common_fc.dsc.data_base+1)
#define INTERRUPT_ENABLE_REG		(ts->common_fc.dsc.control_base+1)
/* Display */
#define DISPLAY_PAGE			0x01
#define DCS_CMD_DATA_REG		(ts->video_fc.dsc.command_base) /* 1-page */
#define DCS_CMD_SEND_REG		(ts->video_fc.dsc.command_base + 6) /* 1-page */
#define WRITE_DCS_VAL			0x01 /* 1-page */


/* TOUCHPAD_SENSORS */
#define FINGER_DATA_REG_START		(ts->finger_fc.dsc.data_base)
#define OBJECT_ATTENTION_REG		(ts->finger_fc.dsc.data_base+2)
#define FINGER_REPORT_REG			(ts->finger_fc.dsc.control_base+7)
#define OBJECT_REPORT_ENABLE_REG	(ts->finger_fc.dsc.control_base+9)
#define WAKEUP_GESTURE_ENABLE_REG	(ts->finger_fc.dsc.control_base+11)

#define FLASH_CONFIG_ID_REG			(ts->flash_fc.dsc.control_base)
#define FLASH_STATUS_REG			(ts->flash_fc.dsc.data_base+3)

#define LPWG_STATUS_REG				(ts->custom_fc.dsc.data_base)
#define LPWG_DATA_REG				(ts->custom_fc.dsc.data_base+1)
#define LPWG_TAPCOUNT_REG			(ts->custom_fc.dsc.control_base)
#define LPWG_MIN_INTERTAP_REG		(ts->custom_fc.dsc.control_base+1)
#define LPWG_MAX_INTERTAP_REG		(ts->custom_fc.dsc.control_base+2)
#define LPWG_TOUCH_SLOP_REG			(ts->custom_fc.dsc.control_base+3)
#define LPWG_TAP_DISTANCE_REG		(ts->custom_fc.dsc.control_base+4)
#define LPWG_TAP_THRESHOLD_REG      (ts->custom_fc.dsc.control_base+5)
#define LPWG_INTERRUPT_DELAY_REG	(ts->custom_fc.dsc.control_base+6)
#define LPWG_TAPCOUNT_REG2			(ts->custom_fc.dsc.control_base+7)
#define LPWG_MIN_INTERTAP_REG2		(ts->custom_fc.dsc.control_base+8)
#define LPWG_MAX_INTERTAP_REG2		(ts->custom_fc.dsc.control_base+9)
#define LPWG_TOUCH_SLOP_REG2		(ts->custom_fc.dsc.control_base+10)
#define LPWG_TAP_DISTANCE_REG2		(ts->custom_fc.dsc.control_base+11)
#define LPWG_TAP_THRESHOLD_REG2     (ts->custom_fc.dsc.control_base+12)
#define LPWG_INTERRUPT_DELAY_REG2	(ts->custom_fc.dsc.control_base+13)
#define LPWG_FAIL_INT_ENABLE_REG    (ts->custom_fc.dsc.control_base+15) 

#define LPWG_FAIL_REASON_REALTIME   (ts->custom_fc.dsc.data_base + 0x49)

#if defined(ENABLE_SWIPE_MODE)
#define SWIPE_COOR_START_X_LSB_REG			(ts->custom_fc.dsc.data_base + 74)
#define SWIPE_COOR_START_X_MSB_REG			(ts->custom_fc.dsc.data_base + 75)
#define SWIPE_COOR_START_Y_LSB_REG			(ts->custom_fc.dsc.data_base + 76)
#define SWIPE_COOR_START_Y_MSB_REG			(ts->custom_fc.dsc.data_base + 77)
#define SWIPE_COOR_END_X_LSB_REG			(ts->custom_fc.dsc.data_base + 78)
#define SWIPE_COOR_END_X_MSB_REG			(ts->custom_fc.dsc.data_base + 79)
#define SWIPE_COOR_END_Y_LSB_REG			(ts->custom_fc.dsc.data_base + 80)
#define SWIPE_COOR_END_Y_MSB_REG			(ts->custom_fc.dsc.data_base + 81)

#define SWIPE_FAIL_REASON_REG				(ts->custom_fc.dsc.data_base + 82)
#define SWIPE_TIME_LSB_REG					(ts->custom_fc.dsc.data_base + 83)
#define SWIPE_TIME_MSB_REG					(ts->custom_fc.dsc.data_base + 84)

#define SWIPE_ENABLE_REG					(ts->custom_fc.dsc.control_base + 17)
#define SWIPE_DISTANCE_REG					(ts->custom_fc.dsc.control_base + 18)
#define SWIPE_RATIO_THRESHOLD_REG			(ts->custom_fc.dsc.control_base + 19)
#define SWIPE_RATIO_CHECK_PERIOD_REG		(ts->custom_fc.dsc.control_base + 20)
#define SWIPE_RATIO_CHK_MIN_DISTANCE_REG	(ts->custom_fc.dsc.control_base + 21)
#define MIN_SWIPE_TIME_THRES_LSB_REG		(ts->custom_fc.dsc.control_base + 22)
#define MIN_SWIPE_TIME_THRES_MSB_REG		(ts->custom_fc.dsc.control_base + 23)
#define MAX_SWIPE_TIME_THRES_LSB_REG		(ts->custom_fc.dsc.control_base + 24)
#define MAX_SWIPE_TIME_THRES_MSB_REG		(ts->custom_fc.dsc.control_base + 25)
#endif

/* DCS Opcode */
#define DCS_OPCODE_CTRL     1
#define DCS_COMMAND_SEND    2

u8 LPWG_FAIL_REASON_ENABLE = 0;
#define MAX_NUM_OF_FAIL_REASON 2

#define MAX_PRESSURE			255
#define LPWG_BLOCK_SIZE			7
#define KNOCKON_DELAY			68	/* 700ms */

u32 LPWG_ACTIVE_AREA_X1		= 109;
u32 LPWG_ACTIVE_AREA_X2		= 610;
u32 LPWG_ACTIVE_AREA_Y1		= 17;
u32 LPWG_ACTIVE_AREA_Y2		= 518;

u32 DEFAULT_ACTIVE_AREA_X1		= 40;
u32 DEFAULT_ACTIVE_AREA_X2		= 680;
u32 DEFAULT_ACTIVE_AREA_Y1		= 40;
u32 DEFAULT_ACTIVE_AREA_Y2		= 1240;
/****************************************************************************
 * Macros
 ****************************************************************************/
/* Get user-finger-data from register.
 */
#define GET_X_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_Y_POSITION(_msb_reg, _lsb_reg) \
	(((u16)((_msb_reg << 8)  & 0xFF00)  | (u16)((_lsb_reg) & 0xFF)))
#define GET_WIDTH_MAJOR(_width_x, _width_y) \
	((_width_x - _width_y) > 0) ? _width_x : _width_y
#define GET_WIDTH_MINOR(_width_x, _width_y) \
	((_width_x - _width_y) > 0) ? _width_y : _width_x
#define GET_OBJECT_REPORT_INFO(_reg, _type) \
	(((_reg) & ((u8)(1 << (_type)))) >> (_type))
#define GET_ORIENTATION(_width_y, _width_x) \
	((_width_y - _width_x) > 0) ? 0 : 1
#define GET_PRESSURE(_pressure) \
		_pressure

#if defined(ENABLE_SWIPE_MODE)
#define GET_LOW_U8_FROM_U16(_u16_data) \
	((u8)((_u16_data) & 0x00FF))
#define GET_HIGH_U8_FROM_U16(_u16_data) \
	((u8)(((_u16_data) & 0xFF00) >> 8))
#endif

/****************************************************************************
* Type Definitions
****************************************************************************/

/****************************************************************************
* Variables
****************************************************************************/
#if defined ( TOUCH_MODEL_P1S3G )
static const char defaultFirmware[] = "synaptics/lions/PLG496-V1.08_PR188341-DS5.8.0.0.1152_50057088.img";
static const char defaultFirmwareRecovery[] = "synaptics/lions/PLG496-V1.08_PR188341-DS5.8.0.0.1152_50057088.bin";
static const char defaultPanelSpec[] = "synaptics/lions/p1s3g_limit.txt";
#define TPD_I2C_ADDRESS 0x20
#define TPD_I2C_ADDRESS_SE 0x2C
#else
#error "Model should be defined"
#endif

struct synaptics_ts_data *ts =NULL;
static struct synaptics_ts_exp_fhandler rmidev_fhandler;
int enable_rmi_dev = 0;
int fw_crack_update = 0;
int check_first_fw_update = 0;
int ime_stat = 0;
int is_update = 0;
int pen_support = 0;
int QCoverClose = 0;
int force_set = 0;
#if defined(ENABLE_SWIPE_MODE)
int get_swipe_mode = 1;
int wakeup_by_swipe = 0;
#endif

struct delayed_work 	work_ime_drumming;
/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
//mode:0 => write_log, mode:1 && buf => cat, mode:2 && buf => delta
extern int F54TestHandle(struct synaptics_ts_data *ts, int input, int mode, char *buf);
extern int lockscreen_stat;
extern unsigned char g_qem_check;
extern struct workqueue_struct* touch_wq;
extern int Touch_Quick_Cover_Closed;
/****************************************************************************
* Local Function Prototypes
****************************************************************************/

/****************************************************************************
* Device Specific Function Prototypes
****************************************************************************/
static int TD4191_Initialize(struct i2c_client *client);
static void TD4191_Reset(struct i2c_client *client);
static int TD4191_Connect(void);
static int TD4191_InitRegister(struct i2c_client *client);
static int TD4191_InterruptHandler(struct i2c_client *client,TouchReadData *pData);
static int TD4191_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo);
static int TD4191_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo);
static int TD4191_UpdateFirmware(struct i2c_client *client, char *pFilename);
static int TD4191_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting);
static int TD4191_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* pBuf, int bufSize, int* pDataLen);
static int TD4191_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue);
static void TD4191_ClearInterrupt(struct i2c_client *client);
static void TD4191_SetActivceArea();
/****************************************************************************
* Local Functions
****************************************************************************/
static int synaptics_ts_page_data_read(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data)
{
	int ret = 0;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	
	ret = Touch_I2C_Read(client, reg, data, size);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
	
}
 
static int synaptics_ts_page_data_write(struct i2c_client *client,
	 u8 page, u8 reg, int size, u8 *data)
{
	int ret = 0;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	
	ret = Touch_I2C_Write(client, reg, data, size);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;

}
static void synapitcs_change_ime_status(struct i2c_client *client, int ime_status)
{
	u8 udata[2] = {0, };
	u8 drumming_address = f12_info.ctrl_reg_addr[10];

	TOUCH_LOG("IME STATUS is [ %d ]!!!\n",ime_status);

	Touch_I2C_Read(client, drumming_address, udata, 2);

	if (ime_status) {
		TOUCH_LOG("IME on !!\n");
		udata[0] = 0x06;/*Drumming Acceleration Threshold*/
		udata[1] = 0x0a;/*Minimum Drumming Separation*/
		if (Touch_I2C_Write(client,
					drumming_address, udata, 2) < 0) {
			TOUCH_ERR("Touch i2c write fail !!\n");
		}
	} else {
		udata[0] = 0x0a; /*Drumming Acceleration Threshold*/
		udata[1] = 0x0a; /*Minimum Drumming Separation*/
		if (Touch_I2C_Write(client,
					drumming_address, udata, 2) < 0) {
			TOUCH_ERR("Touch i2c write fail !!\n");
		}
		TOUCH_LOG("IME Off\n");
	}
	TOUCH_LOG("Done !!\n");
	return;
}

static void clear_swipe(struct i2c_client *client)
{
	u8 status = 0;

	u8 buf_lsb = 0;
	u8 buf_msb = 0;
	u8 fail_reason = 0;
	synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_STATUS_REG, 1, &status);
	if(status & 0x4){
	Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, LPWG_PAGE);

	Touch_I2C_Read_Byte(client, SWIPE_FAIL_REASON_REG, &fail_reason);
	Touch_I2C_Read_Byte(client, SWIPE_COOR_START_X_LSB_REG, &buf_lsb);
	Touch_I2C_Read_Byte(client, SWIPE_COOR_START_X_MSB_REG, &buf_msb);

	Touch_I2C_Read_Byte(client, SWIPE_COOR_START_Y_LSB_REG, &buf_lsb);
	Touch_I2C_Read_Byte(client, SWIPE_COOR_START_Y_MSB_REG, &buf_msb);

	Touch_I2C_Read_Byte(client, SWIPE_COOR_END_X_LSB_REG, &buf_lsb);
	Touch_I2C_Read_Byte(client, SWIPE_COOR_END_X_MSB_REG, &buf_msb);

	Touch_I2C_Read_Byte(client, SWIPE_COOR_END_Y_LSB_REG, &buf_lsb);
	Touch_I2C_Read_Byte(client, SWIPE_COOR_END_Y_MSB_REG, &buf_msb);

	Touch_I2C_Read_Byte(client, SWIPE_TIME_LSB_REG, &buf_lsb);
	Touch_I2C_Read_Byte(client, SWIPE_TIME_MSB_REG, &buf_msb);

	Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	TOUCH_LOG("Clear SWIPE\n");
	}
}

static int set_doze_param()
{
    u8 buf_array[6] = {0};

	Touch_I2C_Read(ds4_i2c_client,f12_info.ctrl_reg_addr[27],buf_array,6);
	buf_array[2] = 0x0C;  /* False Activation Threshold */
	buf_array[3] = 0x0B;  /* Max Active Duration */
	buf_array[5] = 0x01;  /* Max Active Duration Timeout */
	Touch_I2C_Write(ds4_i2c_client, f12_info.ctrl_reg_addr[27],buf_array,6);
	return 0;
}
 
static int synaptics_ts_page_data_write_byte(struct i2c_client *client,
	 u8 page, u8 reg, u8 data)
{
	int ret = 0;

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	
	ret = Touch_I2C_Write_Byte(client, reg, data);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;

}

void get_f12_info(struct synaptics_ts_data *ts)
{
	int retval;
	struct synaptics_ts_f12_query_5 query_5;
	struct synaptics_ts_f12_query_8 query_8;
	struct synaptics_ts_f12_ctrl_23 ctrl_23;
	int i;
	u8 offset;

	if (!ts) {
		TOUCH_ERR("ts is null\n");
		return;
	}

	/* ctrl_reg_info setting */
	retval = Touch_I2C_Read(ts->client, (ts->finger_fc.dsc.query_base + 5),
			query_5.data,sizeof(query_5.data));

	if (retval < 0) {
		TOUCH_ERR("Failed to read from F12_2D_QUERY_05_Control_Presence register\n");
		return;
	}

	f12_info.ctrl_reg_is_present[0] = query_5.ctrl_00_is_present;
	f12_info.ctrl_reg_is_present[1] = query_5.ctrl_01_is_present;
	f12_info.ctrl_reg_is_present[2] = query_5.ctrl_02_is_present;
	f12_info.ctrl_reg_is_present[3] = query_5.ctrl_03_is_present;
	f12_info.ctrl_reg_is_present[4] = query_5.ctrl_04_is_present;
	f12_info.ctrl_reg_is_present[5] = query_5.ctrl_05_is_present;
	f12_info.ctrl_reg_is_present[6] = query_5.ctrl_06_is_present;
	f12_info.ctrl_reg_is_present[7] = query_5.ctrl_07_is_present;
	f12_info.ctrl_reg_is_present[8] = query_5.ctrl_08_is_present;
	f12_info.ctrl_reg_is_present[9] = query_5.ctrl_09_is_present;
	f12_info.ctrl_reg_is_present[10] = query_5.ctrl_10_is_present;
	f12_info.ctrl_reg_is_present[11] = query_5.ctrl_11_is_present;
	f12_info.ctrl_reg_is_present[12] = query_5.ctrl_12_is_present;
	f12_info.ctrl_reg_is_present[13] = query_5.ctrl_13_is_present;
	f12_info.ctrl_reg_is_present[14] = query_5.ctrl_14_is_present;
	f12_info.ctrl_reg_is_present[15] = query_5.ctrl_15_is_present;
	f12_info.ctrl_reg_is_present[16] = query_5.ctrl_16_is_present;
	f12_info.ctrl_reg_is_present[17] = query_5.ctrl_17_is_present;
	f12_info.ctrl_reg_is_present[18] = query_5.ctrl_18_is_present;
	f12_info.ctrl_reg_is_present[19] = query_5.ctrl_19_is_present;
	f12_info.ctrl_reg_is_present[20] = query_5.ctrl_20_is_present;
	f12_info.ctrl_reg_is_present[21] = query_5.ctrl_21_is_present;
	f12_info.ctrl_reg_is_present[22] = query_5.ctrl_22_is_present;
	f12_info.ctrl_reg_is_present[23] = query_5.ctrl_23_is_present;
	f12_info.ctrl_reg_is_present[24] = query_5.ctrl_24_is_present;
	f12_info.ctrl_reg_is_present[25] = query_5.ctrl_25_is_present;
	f12_info.ctrl_reg_is_present[26] = query_5.ctrl_26_is_present;
	f12_info.ctrl_reg_is_present[27] = query_5.ctrl_27_is_present;
	f12_info.ctrl_reg_is_present[28] = query_5.ctrl_28_is_present;
	f12_info.ctrl_reg_is_present[29] = query_5.ctrl_29_is_present;
	f12_info.ctrl_reg_is_present[30] = query_5.ctrl_30_is_present;
	f12_info.ctrl_reg_is_present[31] = query_5.ctrl_31_is_present;

	offset = 0;

	for (i = 0; i < 32; i++) {
		f12_info.ctrl_reg_addr[i] =
			ts->finger_fc.dsc.control_base + offset;

		if (f12_info.ctrl_reg_is_present[i])
			offset++;
	}

	/* data_reg_info setting */
	retval = Touch_I2C_Read(ts->client, (ts->finger_fc.dsc.query_base + 8), query_8.data,sizeof(query_8.data));

	if (retval < 0) {
		TOUCH_ERR("Failed to read from F12_2D_QUERY_08_Data_Presence register\n");
		return;
	}

	f12_info.data_reg_is_present[0] = query_8.data_00_is_present;
	f12_info.data_reg_is_present[1] = query_8.data_01_is_present;
	f12_info.data_reg_is_present[2] = query_8.data_02_is_present;
	f12_info.data_reg_is_present[3] = query_8.data_03_is_present;
	f12_info.data_reg_is_present[4] = query_8.data_04_is_present;
	f12_info.data_reg_is_present[5] = query_8.data_05_is_present;
	f12_info.data_reg_is_present[6] = query_8.data_06_is_present;
	f12_info.data_reg_is_present[7] = query_8.data_07_is_present;
	f12_info.data_reg_is_present[8] = query_8.data_08_is_present;
	f12_info.data_reg_is_present[9] = query_8.data_09_is_present;
	f12_info.data_reg_is_present[10] = query_8.data_10_is_present;
	f12_info.data_reg_is_present[11] = query_8.data_11_is_present;
	f12_info.data_reg_is_present[12] = query_8.data_12_is_present;
	f12_info.data_reg_is_present[13] = query_8.data_13_is_present;
	f12_info.data_reg_is_present[14] = query_8.data_14_is_present;
	f12_info.data_reg_is_present[15] = query_8.data_15_is_present;

	offset = 0;

	for (i = 0; i < 16; i++) {
		f12_info.data_reg_addr[i] =
			ts->finger_fc.dsc.data_base + offset;

		if (f12_info.data_reg_is_present[i])
			offset++;
	}

	/* print info */
	for (i = 0; i < 32; i++) {
		if (f12_info.ctrl_reg_is_present[i])
			TOUCH_LOG("f12_info.ctrl_reg_addr[%d]=0x%02X\n",
					i, f12_info.ctrl_reg_addr[i]);
	}

	for (i = 0; i < 16; i++) {
		if (f12_info.data_reg_is_present[i])
			TOUCH_LOG("f12_info.data_reg_addr[%d]=0x%02X\n",
					i, f12_info.data_reg_addr[i]);
	}
	Touch_I2C_Read(ts->client, f12_info.ctrl_reg_addr[23], ctrl_23.data, sizeof(ctrl_23.data));
	pen_support = GET_OBJECT_REPORT_INFO(ctrl_23.obj_type_enable, OBJECT_STYLUS_BIT);
	return;
}

static int read_page_description_table(struct i2c_client *client)
{
	int ret = 0;
	struct function_descriptor buffer;

	unsigned short address = 0;
	unsigned short page_num = 0;

	TOUCH_FUNC();

	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->custom_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->sensor_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->analog_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->flash_fc, 0x0, sizeof(struct ts_ic_function));
    memset(&ts->video_fc, 0x0, sizeof(struct ts_ic_function));

	for (page_num = 0; page_num < PAGE_MAX_NUM; page_num++)
	{
		ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, page_num);
		if( ret == TOUCH_FAIL ) {
			return TOUCH_FAIL;
		}

		for (address = DESCRIPTION_TABLE_START; address > 10;
				address -= sizeof(struct function_descriptor)) {
			ret = Touch_I2C_Read(client, address, (unsigned char *)&buffer, sizeof(buffer));
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}

			if (buffer.id == 0)
				break;

			switch (buffer.id) {
			case RMI_DEVICE_CONTROL:
				ts->common_fc.dsc = buffer;
				ts->common_fc.function_page = page_num;
				break;
			case TOUCHPAD_SENSORS:
				ts->finger_fc.dsc = buffer;
				ts->finger_fc.function_page = page_num;
				break;
			case CUSTOM_CONTROL:
				ts->custom_fc.dsc = buffer;
				ts->custom_fc.function_page = page_num;
				break;
			case SENSOR_CONTROL:
				ts->sensor_fc.dsc = buffer;
				ts->sensor_fc.function_page = page_num;
				break;
			case ANALOG_CONTROL:
				ts->analog_fc.dsc = buffer;
				ts->analog_fc.function_page = page_num;
				break;
			case FLASH_MEMORY_MANAGEMENT:
				ts->flash_fc.dsc = buffer;
				ts->flash_fc.function_page = page_num;
				break;
            case VIDEO_CONTROL:
				ts->video_fc.dsc = buffer;
				ts->video_fc.function_page = page_num;
				break;
			default:
				break;
			}
		}
	}

	ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, 0x00);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	if( ts->common_fc.dsc.id == 0 || ts->finger_fc.dsc.id == 0 || ts->analog_fc.dsc.id == 0 || ts->flash_fc.dsc.id == 0 ) {
		TOUCH_ERR("failed to read page description\n");
		return TOUCH_FAIL;
	}

	TOUCH_DBG("common[%dP:0x%02x] finger[%dP:0x%02x] lpwg[%dP:0x%02x] sensor[%dP:0x%02x]"
		"analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
		ts->common_fc.function_page, ts->common_fc.dsc.id,
		ts->finger_fc.function_page, ts->finger_fc.dsc.id,
		ts->custom_fc.function_page, ts->custom_fc.dsc.id,
		ts->sensor_fc.function_page, ts->sensor_fc.dsc.id,
		ts->analog_fc.function_page, ts->analog_fc.dsc.id,
		ts->flash_fc.function_page, ts->flash_fc.dsc.id);
    
    get_f12_info(ts);
    
	return TOUCH_SUCCESS;

}
/**
 * Fail Reason
 *
 *   Error Type            value
 * 1 Distance_Inter_Tap    (1U << 0)
 * 2 Distance TouchSlop    (1U << 1)
 * 3 Timeout Inter Tap     (1U << 2)
 * 4 Multi Finger          (1U << 3)
 * 5 Delay Time            (1U << 4)
 * 6 Palm State            (1U << 5)
 * 7 Active Area           (1U << 6)
 * 8 Tap Count             (1U << 7)
 */
static int lpwg_fail_interrupt_control(struct i2c_client *client, u16 value)
{
	u8 buffer[2] = {0};

	DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_FAIL_INT_ENABLE_REG,
					2, buffer), error);
	buffer[0] = (value >> 8) & 0xFF;
	buffer[1] = value & 0xFF;
	if (synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_FAIL_INT_ENABLE_REG,
				2, buffer) < 0) {
		TOUCH_LOG("LPWG_FAIL_INT_ENABLE_REG write error \n");
	} else {
		TOUCH_LOG("LPWG_FAIL_INT_ENABLE_REG = 0x%02x%02x\n", buffer[0], buffer[1]);
	}
	return 0;
error:
	TOUCH_ERR("%s, %d : LPWG fail interrupt control failed\n", __func__, __LINE__);
	return -EPERM;
}

static int swipe_control(struct i2c_client *client, int mode)
{
	int ret = 0;
	u8 buf = 0;

	buf = mode ? 0x01 : 0x00;

	ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, LPWG_PAGE);
	ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, buf);
//	ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, SWIPE_ENABLE_REG, buf);
	if(mode == 0)Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);
	if(ret) {
		TOUCH_ERR("I2C fail\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}
static int swipe_setParam(struct i2c_client *client)
{
	int ret = 0;

	ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, LPWG_PAGE);

	ret |= Touch_I2C_Write_Byte(client, SWIPE_DISTANCE_REG, 10);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_RATIO_CHK_MIN_DISTANCE_REG, 2);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_RATIO_THRESHOLD_REG, 200);
	ret |= Touch_I2C_Write_Byte(client, SWIPE_RATIO_CHECK_PERIOD_REG, 5);
	ret |= Touch_I2C_Write_Byte(client, MIN_SWIPE_TIME_THRES_LSB_REG, GET_LOW_U8_FROM_U16(0));
	ret |= Touch_I2C_Write_Byte(client, MIN_SWIPE_TIME_THRES_MSB_REG, GET_HIGH_U8_FROM_U16(0));
	ret |= Touch_I2C_Write_Byte(client, MAX_SWIPE_TIME_THRES_LSB_REG, GET_LOW_U8_FROM_U16(150));
	ret |= Touch_I2C_Write_Byte(client, MAX_SWIPE_TIME_THRES_MSB_REG, GET_HIGH_U8_FROM_U16(150));

	ret |= Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);

	if(ret) {
		TOUCH_ERR("I2C fail\n");
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
}

static void synaptics_ts_check_fail_reason(char* reason)
{
	int i = 0;

	for (i = 0; i < MAX_NUM_OF_FAIL_REASON; i++) {
		switch (reason[i]) {
			case FAIL_DISTANCE_INTER_TAP:
				TOUCH_LOG("LPWG FAIL REASON = FAIL_DISTANCE_INTER_TAP\n");
				break;
			case FAIL_DISTANCE_TOUCHSLOP:
				TOUCH_LOG("LPWG FAIL REASON = FAIL_DISTANCE_TOUCHSLOP\n");
				break;
			case FAIL_TIMEOUT_INTER_TAP:
				TOUCH_LOG("LPWG FAIL REASON = FAIL_TIMEOUT_INTER_TAP\n");
				break;
			case FAIL_MULTI_FINGER:
				TOUCH_LOG("LPWG FAIL REASON = FAIL_MULTI_FINGER\n");
				break;
			case FAIL_DELAY_TIME:
				TOUCH_LOG("LPWG FAIL REASON = FAIL_DELAY_TIME\n");
				break;
			case FAIL_PALM_STATE:
				TOUCH_LOG("LPWG FAIL REASON = FAIL_PALM_STATE\n");
				break;
			case FAIL_ACTIVE_AREA:
				TOUCH_LOG("LPWG FAIL REASON = FAIL_ACTIVE_AREA\n");
				break;
			case FAIL_TAP_COUNT:
				TOUCH_LOG("LPWG FAIL REASON = FAIL_TAP_COUNT\n");
				break;
			default:
				TOUCH_LOG("LPWG FAIL REASON = Unknown Fail Reason\n");
				break;
		}
	}
}

static int check_firmware_status(struct i2c_client *client)
{
	u8 device_status = 0;
	u8 flash_status = 0;
    TOUCH_FUNC();
	DO_SAFE(Touch_I2C_Read_Byte(client, FLASH_STATUS_REG, &flash_status),error);
	DO_SAFE(Touch_I2C_Read_Byte(client, DEVICE_STATUS_REG, &device_status),error);
	if ((device_status & (DEVICE_STATUS_FLASH_PROG|DEVICE_CRC_ERROR_MASK)) || (flash_status & 0xFF)) {
		TOUCH_ERR("FLASH_STATUS[0x%x] DEVICE_STATUS[0x%x]\n", (u32)flash_status, (u32)device_status);
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;
error:
    return TOUCH_FAIL;    
}

static int tci_control(struct i2c_client *client, int type, u8 value)
{
	int ret = 0;
	u8 buffer[3] = {0};

	switch (type)
	{
		case REPORT_MODE_CTRL:
			DO_SAFE(Touch_I2C_Read(client, INTERRUPT_ENABLE_REG, buffer, 1),error);
			
			DO_SAFE(Touch_I2C_Write_Byte(client, INTERRUPT_ENABLE_REG,
					value ? buffer[0] & ~INTERRUPT_MASK_ABS0 : buffer[0] | INTERRUPT_MASK_ABS0),error);
			DO_SAFE(Touch_I2C_Read(client, f12_info.ctrl_reg_addr[20], buffer, 3),error);
			buffer[2] = (buffer[2] & 0xfc) | (value ? 0x2 : 0x0);
			DO_SAFE(Touch_I2C_Write(client, f12_info.ctrl_reg_addr[20], buffer, 3),error);
			TOUCH_DBG("report mode: %d\n", value);
			break;
		case TCI_ENABLE_CTRL:
			DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer),error);
		
			buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
			ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TCI_ENABLE_CTRL2:
			DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer),error);
			buffer[0] = (buffer[0] & 0xfe) | (value & 0x1);
			ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TAP_COUNT_CTRL:
			DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer),error);
			buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
			ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TAP_COUNT_CTRL2:
			DO_SAFE(synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer),error);
			buffer[0] = ((value << 3) & 0xf8) | (buffer[0] & 0x7);
			ret = synaptics_ts_page_data_write(client, LPWG_PAGE, LPWG_TAPCOUNT_REG2, 1, buffer);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case MIN_INTERTAP_CTRL:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MIN_INTERTAP_REG, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case MIN_INTERTAP_CTRL2:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MIN_INTERTAP_REG2, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case MAX_INTERTAP_CTRL:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MAX_INTERTAP_REG, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case MAX_INTERTAP_CTRL2:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_MAX_INTERTAP_REG2, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TOUCH_SLOP_CTRL:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TOUCH_SLOP_REG, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TOUCH_SLOP_CTRL2:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TOUCH_SLOP_REG2, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TAP_DISTANCE_CTRL:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TAP_DISTANCE_REG, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case TAP_DISTANCE_CTRL2:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_TAP_DISTANCE_REG2, value);
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case INTERRUPT_DELAY_CTRL:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG,
					value ? (buffer[0] = (KNOCKON_DELAY << 1) | 0x1) : (buffer[0] = 0));
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;
		case INTERRUPT_DELAY_CTRL2:
			ret = synaptics_ts_page_data_write_byte(client, LPWG_PAGE, LPWG_INTERRUPT_DELAY_REG2,
					value ? (buffer[0] = (KNOCKON_DELAY << 1) | 0x1) : (buffer[0] = 0));
			if( ret < 0 ) {
				return TOUCH_FAIL;
			}
			break;

		default:
			break;
	}

	return TOUCH_SUCCESS;
error:
        TOUCH_ERR("%s, %d : tci control failed\n", __func__, __LINE__);
        return TOUCH_FAIL;
	
}

static int sleep_control(struct i2c_client *client, int mode, int recal)
{
	int ret = 0;
	u8 curr = 0;
	u8 next = 0;

	ret = Touch_I2C_Read(client, DEVICE_CONTROL_REG, &curr, 1);
	if( ret < 0 ) {
		return TOUCH_FAIL;
	}

	next = (curr & 0xFC) | (mode ? DEVICE_CONTROL_NORMAL_OP : DEVICE_CONTROL_SLEEP);
	ret = Touch_I2C_Write_Byte(client, DEVICE_CONTROL_REG, next);
	if( ret < 0 ) {
		return TOUCH_FAIL;
	}

	TOUCH_DBG("%s : curr = [%x] next[%x]\n", __func__, curr, next);

	return TOUCH_SUCCESS;
	
}

static int lpwg_control(struct i2c_client *client)
{
	int ret = 0;
	u8 buf = 0;

	TOUCH_FUNC();

	ret = Touch_I2C_Read_Byte(client, INTERRUPT_ENABLE_REG, &buf);
	if( ret < 0 ) {
		return TOUCH_FAIL;
	}
	
	ret = Touch_I2C_Write_Byte(client, INTERRUPT_ENABLE_REG, 0x00);
	if( ret < 0 ) {
		return TOUCH_FAIL;
	}

	switch (ts->currState)
	{
		case STATE_NORMAL:
			swipe_control(client, 0);
			tci_control(client, TCI_ENABLE_CTRL, 0);		// tci-1 disable
			tci_control(client, TCI_ENABLE_CTRL2, 0);		// tci-2 disable
			tci_control(client, REPORT_MODE_CTRL, 0);		// normal
			sleep_control(client, 1, 0);
			wakeup_by_swipe = 0;
			break;
		case STATE_KNOCK_ON_ONLY:					// Only TCI-1
		    TOUCH_ERR("STATE_KNOCK_ON_ONLY\n");
			sleep_control(client, 1, 0);
			tci_control(client, TCI_ENABLE_CTRL, 1);		// tci enabl
			tci_control(client, TAP_COUNT_CTRL, 2); 		// tap count = 2
			tci_control(client, MIN_INTERTAP_CTRL, 6);		// min inter_tap = 60ms
			tci_control(client, MAX_INTERTAP_CTRL, 70);		// max inter_tap = 700ms
			tci_control(client, TOUCH_SLOP_CTRL, 100);		// touch_slop = 10mm
			tci_control(client, TAP_DISTANCE_CTRL, 10);		// tap distance = 10mm
			tci_control(client, INTERRUPT_DELAY_CTRL, 0);	// interrupt delay = 0ms
			tci_control(client, TCI_ENABLE_CTRL2, 0);		// tci-2 disable
			tci_control(client, REPORT_MODE_CTRL, 1);		// wakeup_gesture_only
#if defined(ENABLE_SWIPE_MODE)
			if(ts->lpwgSetting.coverState || !get_swipe_mode){
					swipe_control(client, 0);
			} else {
				swipe_control(client, 1);
				swipe_setParam(client);
			}
#endif
			if(LPWG_FAIL_REASON_ENABLE)lpwg_fail_interrupt_control(client,0xFFFF);
			wakeup_by_swipe = 0;
			break;
		case STATE_KNOCK_ON_CODE:					// TCI-1 and TCI-2
			sleep_control(client, 1, 0);
			tci_control(client, TCI_ENABLE_CTRL, 1);		// tci-1 enable
			tci_control(client, TAP_COUNT_CTRL, 2); 		// tap count = 2
			tci_control(client, MIN_INTERTAP_CTRL, 6);		// min inter_tap = 60ms
			tci_control(client, MAX_INTERTAP_CTRL, 70);		// max inter_tap = 700ms
			tci_control(client, TOUCH_SLOP_CTRL, 100);		// touch_slop = 10mm
			tci_control(client, TAP_DISTANCE_CTRL, 7);		// tap distance = 7mm
			tci_control(client, INTERRUPT_DELAY_CTRL, (u8)ts->lpwgSetting.isFirstTwoTapSame);	// interrupt delay = 0ms

			tci_control(client, TCI_ENABLE_CTRL2, 1);		// tci-2 enable
			tci_control(client, TAP_COUNT_CTRL2, (u8)ts->lpwgSetting.tapCount); // tap count = "user_setting"
			tci_control(client, MIN_INTERTAP_CTRL2, 6);		// min inter_tap = 60ms
			tci_control(client, MAX_INTERTAP_CTRL2, 70);	// max inter_tap = 700ms
			tci_control(client, TOUCH_SLOP_CTRL2, 100);		// touch_slop = 10mm
			tci_control(client, TAP_DISTANCE_CTRL2, 255);	// tap distance = MAX
			tci_control(client, INTERRUPT_DELAY_CTRL2, 0);	// interrupt delay = 0ms

			tci_control(client, REPORT_MODE_CTRL, 1);		// wakeup_gesture_only
			if(LPWG_FAIL_REASON_ENABLE)lpwg_fail_interrupt_control(client,0xFFFF);
#if defined(ENABLE_SWIPE_MODE)
			if(ts->lpwgSetting.coverState || !get_swipe_mode) {
				swipe_control(client, 0);
			} else {
				swipe_control(client, 1);
				swipe_setParam(client);
			}
#endif
			wakeup_by_swipe = 0;
			break;
		case STATE_OFF:
			tci_control(client, TCI_ENABLE_CTRL, 0);		// tci-1 disable
			tci_control(client, TCI_ENABLE_CTRL2, 0);		// tci-2 disable
			tci_control(client, REPORT_MODE_CTRL, 1);
			sleep_control(client, 0, 0);
			wakeup_by_swipe = 0;
			break;
		default:
			TOUCH_ERR("Impossible Report Mode ( %d )\n", ts->currState);
			wakeup_by_swipe = 0;
			break;
	}

	ret = Touch_I2C_Write_Byte(client, INTERRUPT_ENABLE_REG, buf);
	if( ret < 0 ) {
		return TOUCH_FAIL;
	}

	return TOUCH_SUCCESS;	

}

static int get_object_count(struct i2c_client *client)
{
	u8 object_num = 0;
	u8 buf[2] = {0};
	u16 object_attention_data = 0;
	u16 i = 0;

	Touch_I2C_Read(client, OBJECT_ATTENTION_REG, buf, 2);

	object_attention_data = (((u16)((buf[1] << 8)  & 0xFF00)  | (u16)((buf[0]) & 0xFF)));

	for (i = 0; i < MAX_NUM_OF_FINGERS; i++) {
		if (object_attention_data & (0x1 << i)) {
			object_num = i + 1;
		}
	}

	return object_num;
}

/****************************************************************************
* Device Specific Functions
****************************************************************************/
int synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert)
{
	rmidev_fhandler.inserted = insert;

	if (insert)
		rmidev_fhandler.exp_fn = rmidev_fn;
	else
		rmidev_fhandler.exp_fn = NULL;

	return TOUCH_SUCCESS;
}

static ssize_t store_tci(struct i2c_client *client,
	const char *buf, size_t count)
{
	u32 type = 0, tci_num = 0, value = 0;

	sscanf(buf, "%d %d %d", &type, &tci_num, &value);

	tci_control(client, type, (u8)value);

	return count;
}

static ssize_t show_tci(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int err = 0;
	int i = 0;
	u8 buffer[7] = {0};

	err = Touch_I2C_Read(client,  f12_info.ctrl_reg_addr[20], buffer, 3);
	ret += sprintf(buf+ret, "report_mode [%s]\n",
		(buffer[2] & 0x3) == 0x2 ? "WAKEUP_ONLY" : "NORMAL");
	err = Touch_I2C_Read(client, f12_info.ctrl_reg_addr[27], buffer, 1);
	ret += sprintf(buf+ret, "wakeup_gesture [%d]\n\n", buffer[0]);

	for (i = 0; i < 2; i++) {
		synaptics_ts_page_data_read(client, LPWG_PAGE,
			LPWG_TAPCOUNT_REG + (i * LPWG_BLOCK_SIZE), 7, buffer);
		ret += sprintf(buf+ret, "TCI - %d\n", i+1);
		ret += sprintf(buf+ret, "TCI [%s]\n",
			(buffer[0] & 0x1) == 1 ? "enabled" : "disabled");
		ret += sprintf(buf+ret, "Tap Count [%d]\n",
			(buffer[0] & 0xf8) >> 3);
		ret += sprintf(buf+ret, "Min InterTap [%d]\n", buffer[1]);
		ret += sprintf(buf+ret, "Max InterTap [%d]\n", buffer[2]);
		ret += sprintf(buf+ret, "Touch Slop [%d]\n", buffer[3]);
		ret += sprintf(buf+ret, "Tap Distance [%d]\n", buffer[4]);
		ret += sprintf(buf+ret, "Interrupt Delay [%d]\n\n", buffer[6]);
	}

	return ret;
	
}

static ssize_t store_reg_ctrl(struct i2c_client *client,
	const char *buf, size_t count)
{
	u8 buffer[50] = {0};
	char command[6] = {0};
	int page = 0;
	int reg = 0;
	int offset = 0;
	int value = 0;

	sscanf(buf, "%s %x %x %d %x ", command, &page, &reg, &offset, &value);

	if (!strcmp(command, "write")) {
		synaptics_ts_page_data_read(client, page,
			reg, offset+1, buffer);
		buffer[offset] = (u8)value;
		synaptics_ts_page_data_write(client, page,
			reg, offset+1, buffer);
	} else if (!strcmp(command, "read")) {
		synaptics_ts_page_data_read(client, page,
			reg, offset+1, buffer);
		TOUCH_DBG("page[0x%02x] reg[0x%02x] offset[+%d] = 0x%x\n",
			page, reg, offset, buffer[offset]);
	} else {
		TOUCH_DBG("Usage\n");
		TOUCH_DBG("Write page reg offset value\n");
		TOUCH_DBG("Read page reg offset\n");
	}
	return count;
	
}

static ssize_t show_object_report(struct i2c_client *client, char *buf)
{
	u8 object_report_enable_reg;
	u8 temp[8];

	int ret = 0;
	int i;

	ret = Touch_I2C_Read(client, OBJECT_REPORT_ENABLE_REG, &object_report_enable_reg,sizeof(object_report_enable_reg));
	if( ret < 0 ) {
		return ret;
	}

	for (i = 0; i < 8; i++)
		temp[i] = (object_report_enable_reg >> i) & 0x01;

	ret = sprintf(buf,
		"\n======= read object_report_enable register =======\n");
	ret += sprintf(buf+ret,
		" Addr Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0 HEX\n");
	ret += sprintf(buf+ret,
		"--------------------------------------------------\n");
	ret += sprintf(buf+ret,
		" 0x%02X %4d %4d %4d %4d %4d %4d %4d %4d 0x%02X\n",
		OBJECT_REPORT_ENABLE_REG, temp[7], temp[6],
		temp[5], temp[4], temp[3], temp[2], temp[1], temp[0],
		object_report_enable_reg);
	ret += sprintf(buf+ret,
		"--------------------------------------------------\n");
	ret += sprintf(buf+ret,
		" Bit0  : [F]inger -> %7s\n", temp[0] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit1  : [S]tylus -> %7s\n", temp[1] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit2  : [P]alm -> %7s\n", temp[2] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit3  : [U]nclassified Object -> %7s\n",
		temp[3] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit4  : [H]overing Finger -> %7s\n",
		temp[4] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit5  : [G]loved Finger -> %7s\n",
		temp[5] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit6  : [N]arrow Object Swipe -> %7s\n",
		temp[6] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		" Bit7  : Hand[E]dge  -> %7s\n",
		temp[7] ? "Enable" : "Disable");
	ret += sprintf(buf+ret,
		"==================================================\n\n");
	
	return ret;
	
}

static ssize_t store_object_report(struct i2c_client *client,
	const char *buf, size_t count)
{
	int ret = 0;
	char select[16];
	u8 value = 2;
	int select_cnt;
	int i;
	u8 bit_select = 0;
	u8 object_report_enable_reg_old;
	u8 object_report_enable_reg_new;

	sscanf(buf, "%s %hhu", select, &value);

	if ((strlen(select) > 8) || (value > 1)) {
		TOUCH_DBG("<writing object_report guide>\n");
		TOUCH_DBG("echo [select] [value] > object_report\n");
		TOUCH_DBG("select: [F]inger, [S]tylus, [P]alm,"
			" [U]nclassified Object, [H]overing Finger,"
			" [G]loved Finger, [N]arrow Object Swipe,"
			" Hand[E]dge\n");
		TOUCH_DBG("select length: 1~8, value: 0~1\n");
		TOUCH_DBG("ex) echo F 1 > object_report        "
			" (enable [F]inger)\n");
		TOUCH_DBG("ex) echo s 1 > object_report        "
			" (enable [S]tylus)\n");
		TOUCH_DBG("ex) echo P 0 > object_report        "
			" (disable [P]alm)\n");
		TOUCH_DBG("ex) echo u 0 > object_report        "
			" (disable [U]nclassified Object)\n");
		TOUCH_DBG("ex) echo HgNe 1 > object_report     "
			" (enable [H]overing Finger, [G]loved Finger,"
			" [N]arrow Object Swipe, Hand[E]dge)\n");
		TOUCH_DBG("ex) echo eNGh 1 > object_report     "
			" (enable Hand[E]dge, [N]arrow Object Swipe,"
			" [G]loved Finger, [H]overing Finger)\n");
		TOUCH_DBG("ex) echo uPsF 0 > object_report     "
			" (disable [U]nclassified Object, [P]alm,"
			" [S]tylus, [F]inger)\n");
		TOUCH_DBG("ex) echo HguP 0 > object_report     "
			" (disable [H]overing Finger, [G]loved Finger,"
			" [U]nclassified Object, [P]alm)\n");
		TOUCH_DBG("ex) echo HFnuPSfe 1 > object_report "
			" (enable all object)\n");
		TOUCH_DBG("ex) echo enghupsf 0 > object_report "
			" (disbale all object)\n");
	} else {
		select_cnt = strlen(select);

		for (i = 0; i < select_cnt; i++) {
			switch ((char)(*(select + i))) {
			case 'F': case 'f': /* (F)inger */
				bit_select |= (0x01 << 0);
				break;
			case 'S': case 's': /* (S)tylus */
				bit_select |= (0x01 << 1);
				break;
			case 'P': case 'p': /* (P)alm */
				bit_select |= (0x01 << 2);
				break;
			case 'U': case 'u': /* (U)nclassified Object */
				bit_select |= (0x01 << 3);
				break;
			case 'H': case 'h': /* (H)overing Filter */
				bit_select |= (0x01 << 4);
				break;
			case 'G': case 'g': /* (G)loved Finger */
				bit_select |= (0x01 << 5);
				break;
			case 'N': case 'n': /* (N)arrow Ojbect Swipe */
				bit_select |= (0x01 << 6);
				break;
			case 'E': case 'e': /* Hand (E)dge */
				bit_select |= (0x01 << 7);
				break;
			default:
				break;
			}
		}

		ret = Touch_I2C_Read(client, OBJECT_REPORT_ENABLE_REG, &object_report_enable_reg_old, sizeof(object_report_enable_reg_old));
		if( ret < 0 ) {
			return count;
		}

		object_report_enable_reg_new = object_report_enable_reg_old;

		if (value > 0)
			object_report_enable_reg_new |= bit_select;
		else
			object_report_enable_reg_new &= ~(bit_select);

		ret = Touch_I2C_Write_Byte(client, OBJECT_REPORT_ENABLE_REG, object_report_enable_reg_new);
		if( ret < 0 ) {
			return count;
		}
		
	}

	return count;
	
}

static ssize_t show_use_rmi_dev(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%u\n", enable_rmi_dev);

	return ret;
}
void use_rmi_dev()
{
	int ret = 0;
	enable_rmi_dev = 1;
	TOUCH_DBG("enable_rmi_dev:%u\n", enable_rmi_dev);

	if (enable_rmi_dev && rmidev_fhandler.inserted) {
		if (!rmidev_fhandler.initialized) {
			ret = rmidev_fhandler.exp_fn->init(ts);
			if( ret < 0 ) {
				TOUCH_ERR("fail to enable_rmi_dev\n");
			}else{
				rmidev_fhandler.initialized = true;
				TOUCH_DBG("rmidev_fhandler.initialized\n");
			}
			
		}
	}
	else {
		rmidev_fhandler.exp_fn->remove(ts);
		rmidev_fhandler.initialized = false;
	}
}

static ssize_t store_use_rmi_dev(struct i2c_client *client, const char *buf, size_t count)
{
	int ret = 0;
	int value = 0;

	sscanf(buf, "%d", &value);

	if (value < 0 || value > 1) {
		TOUCH_DBG("Invalid enable_rmi_dev value:%d\n", value);
		return count;
	}

	enable_rmi_dev = value;
	TOUCH_DBG("enable_rmi_dev:%u\n", enable_rmi_dev);

	if (enable_rmi_dev && rmidev_fhandler.inserted) {
		if (!rmidev_fhandler.initialized) {
			ret = rmidev_fhandler.exp_fn->init(ts);
			if( ret < 0 ) {
				TOUCH_ERR("fail to enable_rmi_dev\n");
				return count;
			}
			
			rmidev_fhandler.initialized = true;
		}
	}
	else {
		rmidev_fhandler.exp_fn->remove(ts);
		rmidev_fhandler.initialized = false;
	}

	return count;
}

static int dcs_opcode_control(struct synaptics_ts_data *ts, int type, u8 value)
{
	struct i2c_client *client = ts->client;

	switch (type) {
	case DCS_OPCODE_CTRL:
		if (synaptics_ts_page_data_write_byte(client,
					ANALOG_PAGE, DCS_CMD_DATA_REG,
					value) < 0) {
			TOUCH_LOG("DCS_COMMAND_OPCODE_REG write error \n");
			goto error;
		} else {
			TOUCH_LOG("DCS_COMMAND_OPCODE_REG = 0x%x\n", value);
		}
		break;
	case DCS_COMMAND_SEND:
		if (synaptics_ts_page_data_write_byte(client,
					ANALOG_PAGE, DCS_CMD_SEND_REG,
					value) < 0) {
			TOUCH_LOG("DCS_COMMAND_SEND_REG write error \n");
			goto error;
		} else {
			TOUCH_LOG("DCS_COMMAND_SEND_REG\n");
		}
		break;
	default:
		break;
	}

	return 0;
error:
	TOUCH_ERR("%s, %d : DCS Opcode failed\n", __func__, __LINE__);
	return -EPERM;

}

static ssize_t store_opcode(struct i2c_client *client,
		const char *buf, size_t count)
{
	u32 value = 0;

	sscanf(buf, "%x", &value);

	dcs_opcode_control(ts, DCS_OPCODE_CTRL, (u8)value);
	dcs_opcode_control(ts, DCS_COMMAND_SEND, WRITE_DCS_VAL);

	return count;
}

static ssize_t show_opcode(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret += snprintf(buf+ret, PAGE_SIZE-ret, "Command opcode Register Address : [0x%02x%02x]\n", VIDEO_PAGE, DCS_CMD_DATA_REG);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "Command DCS Send Register Address : [0x%02x%02x]\n", VIDEO_PAGE, DCS_CMD_SEND_REG);
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "=====  Opcode Usage  =====\n");
	ret += snprintf(buf+ret, PAGE_SIZE-ret, "echo [value] > opcode\n");
	return ret;
}

static ssize_t show_reset(struct i2c_client *client, char *buf)
{
	int ret = 0;
	Touch_I2C_Write_Byte(client, DEVICE_COMMAND_REG, 0x01);
	
	TOUCH_LOG("SOFT_RESET end\n");
	return ret;
}

static ssize_t show_pen_support(struct i2c_client *client, char *buf)
{
	int ret = 0;
	
	TOUCH_LOG("Read Pen Support : %d\n", pen_support);/* 1: Support , 0: Not support, -1: Unknown */
	ret = snprintf(buf, PAGE_SIZE, "%d\n", pen_support);
	return ret;
}

static ssize_t show_lpwg_fail_reason(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%s : LPWG RTFR is %s\n", __func__,
			LPWG_FAIL_REASON_ENABLE ? "enabled" : "disabled");

	return ret;
}

static ssize_t store_lpwg_fail_reason(struct i2c_client *client,
		const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);

	LPWG_FAIL_REASON_ENABLE = value;
	TOUCH_LOG("%s : %s - LPWG RTFR.\n", __func__,
			LPWG_FAIL_REASON_ENABLE ? "Enable" : "Disable");

	return count;
}

static ssize_t show_rawdata(struct i2c_client *client, char *buf)
{
	int full_raw_lower_ret = 0;
	int full_raw_upper_ret = 0;
	int ret = 0;
	TouchDisableIrq();
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);
	if(need_scan_pdt){
		SCAN_PDT();
		need_scan_pdt = false;
	}
	full_raw_lower_ret = get_limit(F12_2DTxCount,
                    F12_2DRxCount,
                        *ts->client,
                        defaultPanelSpec,
                    "RT78FullRawCapUpperLimit",
                    RT78FullRawCapUpper);
	full_raw_upper_ret =  get_limit(F12_2DTxCount,
                    F12_2DRxCount,
                        *ts->client,
                        defaultPanelSpec,
                    "RT78FullRawCapLowerLimit",
                    RT78FullRawCapLower);
    if (full_raw_lower_ret < 0 || full_raw_upper_ret < 0) {   
		TOUCH_LOG("[%s] lower return = %d upper return = %d\n",
					__func__, full_raw_lower_ret, full_raw_upper_ret);
		TOUCH_LOG("[%s][FAIL] Can not check the limit of raw cap\n",
					__func__);
		ret = snprintf(buf+ret, PAGE_SIZE-ret, "Can not check the limit of raw cap\n");
	}else{
		TOUCH_LOG("[%s] lower return = %d upper return = %d\n",
					__func__, full_raw_lower_ret, full_raw_upper_ret);
		TOUCH_LOG("[%s][SUCCESS] Can check the limit\n",
					__func__);
		ret = F54Test('p', 1, buf);
	}
	if (ret == 0)
		ret += snprintf(buf + ret, PAGE_SIZE-ret, "ERROR: full_raw_cap failed.\n");
	set_fs(old_fs);
	TD4191_InitRegister(client);
	TouchEnableIrq();
	return ret;
}

static ssize_t store_rawdata(struct i2c_client *client, const char *buf, size_t count)
{
	TOUCH_FUNC();
	int full_raw_lower_ret = 0;
	int full_raw_upper_ret = 0;
	char rawdata_file_path[64] = {0,};
	sscanf(buf, "%s", rawdata_file_path);
	
	TouchDisableIrq();
	if(need_scan_pdt){
		SCAN_PDT();
		need_scan_pdt = false;
	}	
	full_raw_lower_ret = get_limit(F12_2DTxCount,
                    F12_2DRxCount,
                        *ts->client,
                        defaultPanelSpec,
                    "RT78FullRawCapUpperLimit",
                    RT78FullRawCapUpper);
	full_raw_upper_ret =  get_limit(F12_2DTxCount,
                    F12_2DRxCount,
                        *ts->client,
                        defaultPanelSpec,
                    "RT78FullRawCapLowerLimit",
                    RT78FullRawCapLower);
	if (full_raw_lower_ret < 0 || full_raw_upper_ret < 0) {   
		TOUCH_LOG("[%s] lower return = %d upper return = %d\n",
					__func__, full_raw_lower_ret, full_raw_upper_ret);
		TOUCH_LOG("[%s][FAIL] Can not check the limit of raw cap\n",
					__func__);
	}else{
		TOUCH_LOG("[%s] lower return = %d upper return = %d\n",
					__func__, full_raw_lower_ret, full_raw_upper_ret);
		TOUCH_LOG("[%s][SUCCESS] Can check the limit\n",
					__func__);
		F54Test('p', 2, rawdata_file_path);
	}
	TD4191_InitRegister(client);
	TouchEnableIrq();
	return count;
}
static ssize_t show_updated(struct i2c_client *client, char *buf)
{
	TOUCH_LOG("[%s]show_updated %d\n",__func__,check_first_fw_update);
	return sprintf ( buf, "%d\n", check_first_fw_update );
}

#if defined(ENABLE_SWIPE_MODE)
static ssize_t show_swipe_mode(struct i2c_client *client, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%u\n", get_swipe_mode);

	return ret;
}

static ssize_t store_swipe_mode(struct i2c_client *client,
	const char *buf, size_t count)
{
	int value = 0;

	sscanf(buf, "%d", &value);

	get_swipe_mode = value;

	return count;
}
#endif
static void change_ime_drumming_func(struct work_struct *work_ime_drumming)
{
	synapitcs_change_ime_status(ds4_i2c_client, ime_stat);
	TOUCH_LOG("[%s]change_ime_drumming_func %d\n",__func__,ime_stat);
	return;
}

static ssize_t show_ime_drumming_status(struct i2c_client *client, char *buf)
{
	int ret = 0;
	return ret;
}

static ssize_t store_ime_drumming_status(struct i2c_client *client,
		const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);
	TOUCH_LOG("[Touch] %s : IME value %d\n", __func__,value);
	switch (value) {
	case IME_OFF:
			ime_stat = 0;
			TOUCH_LOG("[Touch] %s : IME OFF\n", __func__);
			queue_delayed_work(touch_wq, &work_ime_drumming, msecs_to_jiffies(10));
		break;
	case IME_ON:
			ime_stat = 1;
			TOUCH_LOG("[Touch] %s : IME ON\n", __func__);
			queue_delayed_work(touch_wq, &work_ime_drumming, msecs_to_jiffies(10));
		break;
	case IME_SWYPE:
			ime_stat = 2;
			TOUCH_LOG("[Touch] %s : IME ON and swype ON\n", __func__);
			queue_delayed_work(touch_wq, &work_ime_drumming, msecs_to_jiffies(10));
		break;
	default:
		break;
	}

	return count;
}

static LGE_TOUCH_ATTR(check_updated, S_IRUGO | S_IWUSR, show_updated, NULL);
static LGE_TOUCH_ATTR(reset, S_IRUGO | S_IWUSR, show_reset, NULL);
static LGE_TOUCH_ATTR(tci, S_IRUGO | S_IWUSR, show_tci, store_tci);
static LGE_TOUCH_ATTR(reg_ctrl, S_IRUGO | S_IWUSR, NULL, store_reg_ctrl);
static LGE_TOUCH_ATTR(object_report, S_IRUGO | S_IWUSR, show_object_report, store_object_report);
static LGE_TOUCH_ATTR(enable_rmi_dev, S_IRUGO | S_IWUSR, show_use_rmi_dev, store_use_rmi_dev);
static LGE_TOUCH_ATTR(opcode, S_IRUGO | S_IWUSR, show_opcode, store_opcode);
static LGE_TOUCH_ATTR(pen_support, S_IRUGO | S_IWUSR, show_pen_support, NULL);
static LGE_TOUCH_ATTR(lpwg_fail_reason, S_IRUGO | S_IWUSR,show_lpwg_fail_reason, store_lpwg_fail_reason);
static LGE_TOUCH_ATTR(rawdata, S_IRUGO | S_IWUSR, show_rawdata, store_rawdata);
static LGE_TOUCH_ATTR(ime_status, S_IRUGO | S_IWUSR, show_ime_drumming_status, store_ime_drumming_status);
#if defined(ENABLE_SWIPE_MODE)
static LGE_TOUCH_ATTR(swipe_mode, S_IRUGO | S_IWUSR, show_swipe_mode, store_swipe_mode);
#endif

static struct attribute *TD4191_attribute_list[] = {
	&lge_touch_attr_ime_status.attr,
	&lge_touch_attr_check_updated.attr,
	&lge_touch_attr_reset.attr,
	&lge_touch_attr_tci.attr,
	&lge_touch_attr_reg_ctrl.attr,
	&lge_touch_attr_object_report.attr,
	&lge_touch_attr_enable_rmi_dev.attr,
	&lge_touch_attr_opcode.attr,
	&lge_touch_attr_pen_support.attr,
	&lge_touch_attr_lpwg_fail_reason.attr,
	&lge_touch_attr_rawdata.attr,
#if defined(ENABLE_SWIPE_MODE)
	&lge_touch_attr_swipe_mode.attr,
#endif
	NULL,
};

void lcd_gamma_reset()
{
	u8 buffer[50] = {0};
	TOUCH_LOG("lcd_gamma_reset start.\n");
	synaptics_ts_page_data_read(ds4_i2c_client, 1,
			352, 1, buffer);
	buffer[0] = (u8)38;
	synaptics_ts_page_data_write(ds4_i2c_client, 1,
			352, 1, buffer);

	synaptics_ts_page_data_read(ds4_i2c_client, 1,
			352, 6+1, buffer);
	buffer[6] = (u8)1;
	synaptics_ts_page_data_write(ds4_i2c_client, 1,
			352, 6+1, buffer);

	msleep(200);
}

void soft_reset()
{
	TOUCH_LOG("soft_reset\n");
	Touch_I2C_Write_Byte(ds4_i2c_client, DEVICE_COMMAND_REG, 0x01);
}
int synaptics_ts_fw_recovery(struct synaptics_ts_data* ts)
{
	char path[256];
	SynaScanPDT(ts);
	if(ts->ubootloader_mode) {
		memcpy(path, defaultFirmwareRecovery, sizeof(path));
		goto firmware;
	}
	TOUCH_LOG("Don't need to Recovery Firmware.\n");
	return TOUCH_SUCCESS;
firmware:
    mtk_wdt_enable ( WK_WDT_DIS );
	DO_SAFE(FirmwareRecovery(ts, path),error);
	soft_reset();
	msleep(50);
    mtk_wdt_enable ( WK_WDT_EN);
error:
	return TOUCH_FAIL;
}

void write_time_log(char *filename, char *data, int data_include)
{
	int fd = 0;
	char *fname = NULL;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;

	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string,
			sizeof(time_string),
			"%02d-%02d %02d:%02d:%02d.%03lu\n\n",
			my_date.tm_mon + 1,
			my_date.tm_mday,
			my_date.tm_hour,
			my_date.tm_min,
			my_date.tm_sec,
			(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);

	if (filename == NULL)
		fname = "/mnt/sdcard/touch_self_test.txt";
	else
		fname = filename;
	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);

	TOUCH_LOG("write open %s, fd : %d\n",
			(fd >= 0) ? "success" : "fail", fd);

	if (fd >= 0) {
		if (data_include && data != NULL)
			sys_write(fd, data, strlen(data));
		sys_write(fd, time_string, strlen(time_string));
		sys_close(fd);
	}
	set_fs(old_fs);
}


static int TD4191_Initialize(struct i2c_client *client)
{
	int ret = 0;
	TOUCH_FUNC();
	TD4191_Connect();
	/* IMPLEMENT : Device initialization at Booting */
	ts = devm_kzalloc(&client->dev, sizeof(struct synaptics_ts_data), GFP_KERNEL);
	if( ts == NULL ) {
		TOUCH_ERR("failed to allocate memory for device driver data\n");
		return TOUCH_FAIL;
	}

	ts->client = client;
    ds4_i2c_client = client;
    synaptics_ts_fw_recovery(ts);
	INIT_DELAYED_WORK(&work_ime_drumming, change_ime_drumming_func);

    ret = Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, 0);
    if( ret == TOUCH_FAIL ) {
        TOUCH_ERR("TD4191 was NOT detected\n");
        return TOUCH_FAIL;
    }
    /* read initial page description */
	ret = read_page_description_table(client);
	if( ret == TOUCH_FAIL ) {
		if( check_firmware_status(client) == TOUCH_FAIL ) {
			int cnt = 0;
			do {
				fw_crack_update = 1;
				TOUCH_ERR("TD4191 update call.fw_crack_update value is %d\n",fw_crack_update);
				ret = TD4191_UpdateFirmware(client, NULL);
				cnt++;
			} while(ret == TOUCH_FAIL && cnt < 3);

			if( ret == TOUCH_FAIL ) {
				devm_kfree(&client->dev, ts);
				return TOUCH_FAIL;
			}
		}
	}
	return TOUCH_SUCCESS;
	
}

static void TD4191_Reset(struct i2c_client *client)
{
	int reset_ctrl = RESET_NONE;
	switch (reset_ctrl) {
		case SOFT_RESET:
			TOUCH_LOG("SOFT_RESET start\n");
			DO_SAFE(Touch_I2C_Write_Byte(ts->client, DEVICE_COMMAND_REG, 0x01),
					error);
		//	msleep(ts->pdata->role->booting_delay);
			TOUCH_LOG("SOFT_RESET end\n");
			break;
		case PIN_RESET:
			TOUCH_LOG("PIN_RESET start\n");
		/*	gpio_direction_output(ts->pdata->reset_pin, 0);
			msleep(ts->pdata->role->reset_delay);
			gpio_direction_output(ts->pdata->reset_pin, 1);
			msleep(ts->pdata->role->booting_delay);*/
			break;
		default:
			break;
	}
	return TOUCH_SUCCESS;
error:
	TOUCH_LOG("error\n");
	return TOUCH_FAIL;
}


static int TD4191_Connect(void)
{
/*
	int ret = 0;
	u8 reg = 0;
	u8 data = 0;
	reg = PAGE_SELECT_REG;
    ret = touch_i2c_read_for_query( 0x20, &reg, 1, &data, 1);
	if( ret == TOUCH_FAIL ) {
        TOUCH_ERR("TD4191 was NOT detected\n");
    }else{
		TOUCH_ERR("TD4191 was detected\n");
	}
*/
    return TOUCH_SUCCESS;
}

static void TD4191_SetActivceArea()
{
	int i;
	u8 buffer[50] = {0};
	u8 doubleTap_area_reg_addr = f12_info.ctrl_reg_addr[18];
	TOUCH_LOG( "TD4191_SetActivceArea QCover status = %d\n" , QCoverClose);
	if(QCoverClose){
			for (i = 0; i < 2; i++) {
				synaptics_ts_page_data_read(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr, i + 1, buffer);
				if (i == 0){
					buffer[i] = LPWG_ACTIVE_AREA_X1;
				}else{
					buffer[i] = LPWG_ACTIVE_AREA_X1 >> 8;
				}synaptics_ts_page_data_write(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr,i + 1, buffer);
			}
			for (i = 2; i < 4; i++) {
				synaptics_ts_page_data_read(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr, i + 1, buffer);
				if (i == 2){
					buffer[i] = LPWG_ACTIVE_AREA_Y1;
				}else{
					buffer[i] = LPWG_ACTIVE_AREA_Y1 >> 8;
				}synaptics_ts_page_data_write(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr,i + 1, buffer);
			}
			for (i = 4; i < 6; i++) {
				synaptics_ts_page_data_read(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr, i + 1, buffer);
				if (i == 4){
					buffer[i] = LPWG_ACTIVE_AREA_X2;
				}else{
					buffer[i] = LPWG_ACTIVE_AREA_X2 >> 8;
				}synaptics_ts_page_data_write(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr,i + 1, buffer);
			}
			for (i = 6; i < 8; i++) {
				synaptics_ts_page_data_read(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr, i + 1, buffer);
				if (i == 6){
					buffer[i] = LPWG_ACTIVE_AREA_Y2;
				}else{
					buffer[i] = LPWG_ACTIVE_AREA_Y2 >> 8;
				}synaptics_ts_page_data_write(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr,i + 1, buffer);
			}
			sleep_control(ds4_i2c_client, 1, 0);
	} else {
			for (i = 0; i < 2; i++) {
				synaptics_ts_page_data_read(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr, i + 1, buffer);
				if (i == 0){
					buffer[i] = DEFAULT_ACTIVE_AREA_X1;
				}else{
					buffer[i] = DEFAULT_ACTIVE_AREA_X1 >> 8;
				}synaptics_ts_page_data_write(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr,i + 1, buffer);
			}
			for (i = 2; i < 4; i++) {
				synaptics_ts_page_data_read(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr, i + 1, buffer);
				if (i == 2){
					buffer[i] = DEFAULT_ACTIVE_AREA_Y1;
				}else{
					buffer[i] = DEFAULT_ACTIVE_AREA_Y1 >> 8;
				}synaptics_ts_page_data_write(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr,i + 1, buffer);
			}
			for (i = 4; i < 6; i++) {
				synaptics_ts_page_data_read(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr, i + 1, buffer);
				if (i == 4){
					buffer[i] = DEFAULT_ACTIVE_AREA_X2;
				}else{
					buffer[i] = DEFAULT_ACTIVE_AREA_X2 >> 8;
				}synaptics_ts_page_data_write(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr,i + 1, buffer);
			}
			for (i = 6; i < 8; i++) {
				synaptics_ts_page_data_read(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr, i + 1, buffer);
				if (i == 6){
					buffer[i] = DEFAULT_ACTIVE_AREA_Y2;
				}else{
					buffer[i] = DEFAULT_ACTIVE_AREA_Y2 >> 8;
				}synaptics_ts_page_data_write(ds4_i2c_client,COMMON_PAGE, doubleTap_area_reg_addr,i + 1, buffer);
			}
			sleep_control(ds4_i2c_client, 1, 0);
	}
}

static int TD4191_InitRegister(struct i2c_client *client)
{
	int ret = 0;
	u8 reg = 0;
    u8 buf = 0;
	u8 data[2] = {0};
    u8 motion_suppression_reg_addr;
    u8 buf_array[2] = {0};
    TOUCH_FUNC();
	reg = DEVICE_CONTROL_REG;
	data[0] = DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED;
	ret = Touch_I2C_Write(client, reg, data, 1);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	
	reg = INTERRUPT_ENABLE_REG;
	data[0] = 0;
	ret = Touch_I2C_Read(client, reg, data, 1);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	reg = INTERRUPT_ENABLE_REG;
	data[0] |= INTERRUPT_MASK_ABS0;
	ret = Touch_I2C_Write(client, reg, data, 1);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
    
    buf_array[0] = buf_array[1] = 0;
    motion_suppression_reg_addr = f12_info.ctrl_reg_addr[20];
    DO_SAFE(Touch_I2C_Write(client, motion_suppression_reg_addr, buf_array,2), error);
    DO_SAFE(Touch_I2C_Read(client, f12_info.ctrl_reg_addr[15],default_finger_amplitude,2), error);

    DO_SAFE(Touch_I2C_Read(client, f12_info.ctrl_reg_addr[22], &buf,1), error);
	buf_array[0] = buf & 0x03;
    if (buf_array[0] != 0x01) {
			/* PalmFilterMode bits[1:0] (01:Enable palm filter) */
			buf &= ~(0x02);
			buf |= 0x01;
			DO_SAFE(Touch_I2C_Write_Byte(client,f12_info.ctrl_reg_addr[22],buf), error);
	}
    
	reg = INTERRUPT_STATUS_REG;
	data[0] = 0;
	ret = Touch_I2C_Read(client, reg, data, 1);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	tci_control(client, TCI_ENABLE_CTRL, 0);		// tci-1 disable
	tci_control(client, TCI_ENABLE_CTRL2, 0);		// tci-2 disable
	tci_control(client, REPORT_MODE_CTRL, 0);		// normal
	sleep_control(client, 1, 0);
	ts->currState = STATE_NORMAL;

	return TOUCH_SUCCESS;
error:
	TOUCH_ERR("%s, %d : synaptics init failed\n", __func__, __LINE__);
	return TOUCH_FAIL;	
}

static int TD4191_InterruptHandler(struct i2c_client *client,TouchReadData *pData)
{
	
	u8  i = 0;
	u8 buf = 0;
	TouchFingerData *pFingerData = NULL;
	
	char reason[NUM_OF_EACH_FINGER_DATA_REG];
	u8 readFingerCnt = 0;
	u8 regDevStatus = 0;
	u8 regIntStatus = 0;
	u8 regFingerData[MAX_NUM_OF_FINGERS][NUM_OF_EACH_FINGER_DATA_REG];
	u8 buffer[12][4] = { {0} };

	pData->type = DATA_UNKNOWN;
	pData->count = 0;
	Touch_I2C_Read_Byte(client, DEVICE_STATUS_REG, &regDevStatus);
    
    DO_IF((regDevStatus & DEVICE_FAILURE_MASK)== DEVICE_FAILURE_MASK, error);
    
	Touch_I2C_Read_Byte(client, INTERRUPT_STATUS_REG, &regIntStatus);

	if (regIntStatus == INTERRUPT_MASK_NONE) {
		return TOUCH_SUCCESS;
	} else if (regIntStatus & INTERRUPT_MASK_ABS0 ) {
		readFingerCnt = get_object_count(client);

		if (readFingerCnt > 0) {
			Touch_I2C_Read(client, FINGER_DATA_REG_START,			
				(u8 *)regFingerData, (NUM_OF_EACH_FINGER_DATA_REG * readFingerCnt));

			for (i = 0; i < readFingerCnt; i++) {
				if (regFingerData[i][0] == F12_FINGER_STATUS
                    ||regFingerData[i][0] == F12_STYLUS_STATUS
                    ||regFingerData[i][0] == F12_PALM_STATUS
                    ||regFingerData[i][0] == F12_GLOVED_FINGER_STATUS) {
					pFingerData = &pData->fingerData[pData->count];
					pFingerData->id = i;
					pFingerData->type = regFingerData[i][REG_OBJECT];
					pFingerData->x = GET_X_POSITION( regFingerData[i][REG_X_MSB], regFingerData[i][REG_X_LSB] );
					pFingerData->y = GET_Y_POSITION( regFingerData[i][REG_Y_MSB], regFingerData[i][REG_Y_LSB] );
					pFingerData->width_major = GET_WIDTH_MAJOR( regFingerData[i][REG_WX], regFingerData[i][REG_WY] );
					pFingerData->width_minor = GET_WIDTH_MINOR( regFingerData[i][REG_WX], regFingerData[i][REG_WY] );
					pFingerData->orientation = GET_ORIENTATION( regFingerData[i][REG_WY], regFingerData[i][REG_WX] );
					pFingerData->pressure = GET_PRESSURE( regFingerData[i][REG_Z] );
					if(QCoverClose||Touch_Quick_Cover_Closed){
						if(pFingerData->x < LPWG_ACTIVE_AREA_X1 || pFingerData->x > LPWG_ACTIVE_AREA_X2
							|| pFingerData->y < LPWG_ACTIVE_AREA_Y1 || pFingerData->y > LPWG_ACTIVE_AREA_Y2 )
						{
							continue;
						}
					}
					pData->count++;
				}
			}
		}

		pData->type = DATA_FINGER;
	} else if (regIntStatus & INTERRUPT_MASK_CUSTOM) {
		u8 status = 0;
		synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_STATUS_REG, 1, &status);

		if (status & 0x1) { /* TCI-1 : Double-Tap */
			TOUCH_LOG( "Knock-on Detected\n" );
			pData->type = DATA_KNOCK_ON;
            synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_DATA_REG, 4*2, &buffer[0][0]);
		} else if (status & 0x2) {  /* TCI-2 : Multi-Tap */
			TOUCH_LOG( "Knock-code Detected\n" );
			pData->type = DATA_KNOCK_CODE;
			synaptics_ts_page_data_read(client, LPWG_PAGE, LPWG_DATA_REG, 4*ts->lpwgSetting.tapCount, &buffer[0][0]);

			for (i = 0; i < ts->lpwgSetting.tapCount; i++) {
				pData->knockData[i].x
					= GET_X_POSITION(buffer[i][1], buffer[i][0]);
				pData->knockData[i].y
					= GET_Y_POSITION(buffer[i][3], buffer[i][2]);
				TOUCH_DBG("LPWG data [%d, %d]\n",
					pData->knockData[i].x, pData->knockData[i].y);
			}
			pData->count = ts->lpwgSetting.tapCount;

		} else if(status & 0x4){ /* SWIPE */
			u16 swipe_start_x = 0;
			u16 swipe_start_y = 0;
			u16 swipe_end_x = 0;
			u16 swipe_end_y = 0;
			u16 swipe_time = 0;
			u8 buf_lsb = 0;
			u8 buf_msb = 0;
			u8 fail_reason = 0;

			if(lockscreen_stat) {
				wakeup_by_swipe = 1;
				TOUCH_ERR("swipe_control set 0 ( %d ) %d\n", __LINE__,lockscreen_stat);
				swipe_control(client, 0);
			}

			TOUCH_LOG( "Swipe mode Detected\n" );
			Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, LPWG_PAGE);

			Touch_I2C_Read_Byte(client, SWIPE_FAIL_REASON_REG, &fail_reason);
			TOUCH_LOG( "Swipe Fail reason : 0x%x\n", fail_reason );

			Touch_I2C_Read_Byte(client, SWIPE_COOR_START_X_LSB_REG, &buf_lsb);
			Touch_I2C_Read_Byte(client, SWIPE_COOR_START_X_MSB_REG, &buf_msb);
			swipe_start_x = (buf_msb << 8) | buf_lsb;

			Touch_I2C_Read_Byte(client, SWIPE_COOR_START_Y_LSB_REG, &buf_lsb);
			Touch_I2C_Read_Byte(client, SWIPE_COOR_START_Y_MSB_REG, &buf_msb);
			swipe_start_y = (buf_msb << 8) | buf_lsb;

			Touch_I2C_Read_Byte(client, SWIPE_COOR_END_X_LSB_REG, &buf_lsb);
			Touch_I2C_Read_Byte(client, SWIPE_COOR_END_X_MSB_REG, &buf_msb);
			swipe_end_x = (buf_msb << 8) | buf_lsb;

			Touch_I2C_Read_Byte(client, SWIPE_COOR_END_Y_LSB_REG, &buf_lsb);
			Touch_I2C_Read_Byte(client, SWIPE_COOR_END_Y_MSB_REG, &buf_msb);
			swipe_end_y = (buf_msb << 8) | buf_lsb;

			Touch_I2C_Read_Byte(client, SWIPE_TIME_LSB_REG, &buf_lsb);
			Touch_I2C_Read_Byte(client, SWIPE_TIME_MSB_REG, &buf_msb);
			swipe_time = (buf_msb << 8) | buf_lsb;

			Touch_I2C_Write_Byte(client, PAGE_SELECT_REG, DEFAULT_PAGE);

			TOUCH_LOG("LPWG Swipe Gesture: start(%4d,%4d) end(%4d,%4d)\n", swipe_start_x, swipe_start_y, swipe_end_x, swipe_end_y);
			
			pData->count = 1;
			pData->knockData[0].x = swipe_end_x;
			pData->knockData[0].y = swipe_end_y;
			pData->type = DATA_SWIPE;
		} else {
			if(LPWG_FAIL_REASON_ENABLE){
					TOUCH_LOG("LPWG Fail Detected\n");
				synaptics_ts_page_data_read(client,LPWG_PAGE, LPWG_FAIL_REASON_REALTIME,1, &buf);
				reason[0] = buf & 0x0F;
				reason[1] = (buf & 0xF0) >> 4;
				if (reason) {
					TOUCH_LOG("Fail-Reason TCI1 : [%d], TCI2 : [%d]\n", reason[0], reason[1]);
					synaptics_ts_check_fail_reason(reason);
					/*if ((dummy_buf = kzalloc(PAGE_SIZE, GFP_KERNEL)) !=  NULL) {
						F54Test('m', 2, dummy_buf);
						kfree(dummy_buf);
					}*/
				} else {
					TOUCH_LOG("LPWG Real-Time-Fail-Reason has problem - Unknown fail reason\n");
				}
			}else{
				TOUCH_ERR( "Unexpected LPWG Interrupt Status ( 0x%X )\n", status);
			}
		}

	} else {
		TOUCH_ERR( "Unexpected Interrupt Status ( 0x%X )\n", regIntStatus );
	}

	return TOUCH_SUCCESS;
error:
        TOUCH_ERR("%s, %d : get data failed\n", __func__, __LINE__);
        return TOUCH_FAIL;

}


//=================================================================
// module maker : ELK(0), Suntel(1), Tovis(2), Innotek(3), JDI(4), LGD(5)
// 
//
//
//
//=================================================================
static int TD4191_ReadIcFirmwareInfo(struct i2c_client *client, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	u8 readData[FW_VER_INFO_NUM] = {0};
	u8 fw_product_id[11] = {0};
	TOUCH_FUNC();

	ret = Touch_I2C_Read(client, FLASH_CONFIG_ID_REG, readData, FW_VER_INFO_NUM);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	ret = Touch_I2C_Read(client, PRODUCT_ID_REG, fw_product_id, 11);
	if((strncmp(fw_product_id,"PLG496",6))){
		check_first_fw_update = 1;
	}
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	
	TOUCH_LOG("Maker=%d, Key=%d, Supplier=%d\n", ( readData[0] >> 4 ) & 0xF, readData[0] & 0xF, ( readData[1] >> 4 ) & 0x0F);
	TOUCH_LOG("Panel=%d, Size =%d.%d [Inch]\n", readData[2] & 0xF, readData[1] & 0xF, ( readData[2] >> 4 ) & 0xF);
	TOUCH_LOG("Version=%d ( %s )\n", readData[3] & 0x7F, ( ( readData[3] & 0x80 ) == 0x80 ) ? "Official Release" : "Test Release");
	TOUCH_LOG("Product id=%s\n", fw_product_id);
	
	pFwInfo->moduleMakerID = ( readData[0] >> 4 ) & 0x0F;
	pFwInfo->moduleVersion = readData[2] & 0x0F;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = ( readData[3] >> 7 ) & 0x01;
	pFwInfo->version = readData[3] & 0x7F;

	return TOUCH_SUCCESS;
	
}

static int TD4191_GetBinFirmwareInfo(struct i2c_client *client, char *pFilename, TouchFirmwareInfo *pFwInfo)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u8 *pBin = NULL;
	char *pFwFilename = NULL;
	u8 *pReadData = NULL;

	TOUCH_FUNC();
	TOUCH_ERR("TD4191_GetBinFirmwareInfo pFilename:%s\n", pFilename);
	if( pFilename == NULL ) {
		pFwFilename = (char *)defaultFirmware;
	} else {
		pFwFilename = pFilename;
	}

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);
	
	/* Get firmware image buffer pointer from file */
	ret = request_firmware(&fw, pFwFilename, &client->dev);
	if( ret )
	{
		TOUCH_ERR("failed at request_firmware() ( error = %d )\n", ret);
		return TOUCH_FAIL;
	}

	pBin = (u8 *)(fw->data);
	pReadData = &pBin[0x11100];

	TOUCH_LOG("Maker=%d, Key=%d, Supplier=%d\n", ( pReadData[0] >> 4 ) & 0xF, pReadData[0] & 0xF, ( pReadData[1] >> 4 ) & 0x0F);
	TOUCH_LOG("Panel=%d, Size =%d.%d [Inch]\n", pReadData[2] & 0xF, pReadData[1] & 0xF, ( pReadData[2] >> 4 ) & 0xF);
	TOUCH_LOG("Version=%d ( %s )\n", pReadData[3] & 0x7F, ( ( pReadData[3] & 0x80 ) == 0x80 ) ? "Official Release" : "Test Release");
	
	pFwInfo->moduleMakerID = ( pReadData[0] >> 4 ) & 0x0F;
	pFwInfo->moduleVersion = pReadData[2] & 0x0F;
	pFwInfo->modelID = 0;
	pFwInfo->isOfficial = ( pReadData[3] >> 7 ) & 0x01;
	pFwInfo->version = pReadData[3] & 0x7F;

	/* Free firmware image buffer */
	release_firmware(fw);
	use_rmi_dev();
	return TOUCH_SUCCESS;
		
}


static int TD4191_UpdateFirmware(struct i2c_client *client, char *pFilename)
{
	int ret = 0;
	char *pFwFilename = NULL;
	u8 fw_product_id[11] = {0};
	
	TOUCH_FUNC();
    
	TOUCH_ERR("TD4191_UpdateFirmware pFilename:%s\n", pFilename);
	is_update = 1;
	if( pFilename == NULL) {
		pFwFilename = (char *)defaultFirmware;
        TOUCH_LOG("Firmware filename1 = %s\n", pFwFilename);
	} else {
		pFwFilename = pFilename;
	//    pFwFilename = (char *)defaultFirmware;
        TOUCH_LOG("Firmware filename2 = %s\n", pFwFilename);
	}
	
	Touch_I2C_Read(client, PRODUCT_ID_REG, fw_product_id, 11);

	TOUCH_LOG("Firmware filename = %s\n", pFwFilename);

	FirmwareUpgrade(ts, pFwFilename);

	/* read changed page description */
	ret = read_page_description_table(client);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}

	ret = check_firmware_status(client);
	if( ret == TOUCH_FAIL ) {
		return TOUCH_FAIL;
	}
	TOUCH_LOG("g_qem_check = %c\n", g_qem_check);
	if((strncmp(fw_product_id,"PLG496",6)!=0)&&(g_qem_check=='0')){
		TOUCH_LOG("Touch Firmware is QCT before update. so LCD reinit!!\n");
		extern void display_restart(void);
		display_restart();
	}
	is_update = 0;
	soft_reset();
	msleep(50);
	return TOUCH_SUCCESS;
	
}

static int TD4191_SetLpwgMode(struct i2c_client *client, TouchState newState, LpwgSetting  *pLpwgSetting)
{
	int ret = TOUCH_SUCCESS;
	
	TOUCH_FUNC();

	memcpy(&ts->lpwgSetting, pLpwgSetting, sizeof(LpwgSetting));

	if( (ts->currState != newState )|| force_set ) {
		ts->currState = newState;
		ret = lpwg_control(client);
		force_set = 0;
		if( ret == TOUCH_FAIL ) {
			TOUCH_LOG("failed to set lpwg mode in device\n");
		}
	}

	return ret;
}

static void TD4191_ClearInterrupt(struct i2c_client *client)
{
	int ret = 0;
	
	u8 regDevStatus = 0;
	u8 regIntStatus = 0;

	ret = Touch_I2C_Read_Byte(client, DEVICE_STATUS_REG, &regDevStatus);
	if( ret == TOUCH_FAIL ) {
		TOUCH_ERR("failed to read device status reg\n");
	}

	ret = Touch_I2C_Read_Byte(client, INTERRUPT_STATUS_REG, &regIntStatus);
	if( ret == TOUCH_FAIL ) {
		TOUCH_ERR("failed to read interrupt status reg\n");
	}

	if( ret == TOUCH_FAIL ) {
		TOUCH_ERR("failed to clear interrupt\n");
	}
    if(!lockscreen_stat&&ime_stat){
		queue_delayed_work(touch_wq, &work_ime_drumming, msecs_to_jiffies(100));
	}
	return;
	
}

static int TD4191_DoSelfDiagnosis(struct i2c_client *client, int* pRawStatus, int* pChannelStatus, char* buf, int bufSize, int* pDataLen)
{
	int ret = 0;
	int full_raw_cap = 0;
	int electrode_short = 0;
	int lower_img = 0;
	int upper_img = 0;
	int short_upper_img = 0;
	int short_lower_img = 0;
	char *temp_buf = NULL;
	int len = 0;

	TOUCH_FUNC();
    
	if (1) {
        temp_buf = kzalloc(100, GFP_KERNEL);
		if (!temp_buf) {
			TOUCH_LOG("%s Failed to allocate memory\n", __func__);
			return 0;
		}
		write_time_log(NULL, NULL, 0);
		msleep(30);
        
		if(need_scan_pdt){
			SCAN_PDT();
			need_scan_pdt = false;
		}
		
        lower_img = get_limit(F12_2DTxCount,
                    F12_2DRxCount,
                        *ts->client,
                        defaultPanelSpec,
                    "RT78FullRawCapUpperLimit",
                    RT78FullRawCapUpper);
        
        upper_img = get_limit(F12_2DTxCount,
                    F12_2DRxCount,
                        *ts->client,
                        defaultPanelSpec,
                    "RT78FullRawCapLowerLimit",
                    RT78FullRawCapLower);
        
        short_upper_img = get_limit(F12_2DTxCount,
                    F12_2DRxCount,
                        *ts->client,
                        defaultPanelSpec,
                    "RT78ElectodeShortUpperLimit",
                    RT78ElectrodeShortUpper);
        
        short_lower_img = get_limit(F12_2DTxCount,
                    F12_2DRxCount,
                        *ts->client,
                        defaultPanelSpec,
                    "RT78ElectodeShortLowerLimit",
                    RT78ElectrodeShortLower);
        if (lower_img < 0 || upper_img < 0) {
			TOUCH_LOG("[%s] lower return = %d upper return = %d\n",
					__func__, lower_img, upper_img);
			TOUCH_LOG("[%s][FAIL] Can not check the limit of raw cap\n",
					__func__);
			ret = snprintf(buf+ret, PAGE_SIZE-ret, "Can not check the limit of raw cap\n");
		} else {
			TOUCH_LOG("[%s] lower return = %d upper return = %d\n",
					__func__, lower_img, upper_img);
			TOUCH_LOG("[%s][SUCCESS] Can check the limit\n",
					__func__);
			full_raw_cap = F54Test('p', 0, buf);
		/*	if (ts->pdata->ref_chk_option[0]) {
				msleep(30);
				synaptics_get_cap_diff(ts);
			}*/
			msleep(30);
		}
        if (short_lower_img < 0 || short_upper_img < 0) {
			TOUCH_LOG("[%s] short lower return = %d short upper return = %d\n",
					__func__, short_lower_img, short_upper_img);
			TOUCH_LOG("[%s][FAIL] Can not check the limit of Electrode Short\n",
					__func__);
			ret = snprintf(buf+ret, PAGE_SIZE-ret, "Can not check the limit of Electrode Short\n");
		} else {
			TOUCH_LOG("[%s] short lower return = %d short upper return = %d\n",
					__func__, short_lower_img, short_upper_img);
			TOUCH_LOG("[%s][SUCCESS] Can check the limit\n",
					__func__);
			electrode_short = F54Test('q', 0, buf);
			msleep(30);
		}
        
        write_log(NULL, temp_buf);
		msleep(30);
		write_time_log(NULL, NULL, 0);
		msleep(20);
        ret = snprintf(buf,
				PAGE_SIZE,
				"========RESULT=======\n");
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Channel Status : %s",
				(electrode_short == 1) ? "Pass\n" : "Fail\n");
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"Raw Data : %s",
				(full_raw_cap > 0) ? "Pass\n" : "Fail\n");    
	}else {
		write_time_log(NULL, NULL, 0);
		ret += snprintf(buf+ret,
				PAGE_SIZE-ret,
				"state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	kfree(temp_buf);

	return TOUCH_SUCCESS;
}

static int TD4191_AccessRegister(struct i2c_client *client, int cmd, int reg, int *pValue)
{
	int ret = 0;
	
	switch( cmd )
	{
		case READ_IC_REG:
			ret = Touch_I2C_Read_Byte(client, (u8)reg, (u8 *)pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		case WRITE_IC_REG:
			ret = Touch_I2C_Write_Byte(client, (u8)reg, (u8)*pValue);
			if( ret == TOUCH_FAIL ) {
				return TOUCH_FAIL;
			}
			break;

		default:
			TOUCH_ERR("Invalid access command ( cmd = %d )\n", cmd);
			return TOUCH_FAIL;
			break;
	}

	return TOUCH_SUCCESS;

}
static void TD4191_NotifyHandler(struct i2c_client *client, TouchNotify notify, int data)
{
	switch( notify )
	{
		case NOTIFY_CALL:
			TOUCH_LOG("Call was notified ( data = %d )\n", data);
			break;

		case NOTIFY_Q_COVER:
			TOUCH_LOG("Quick Cover was notified ( data = %d )\n", data);
			QCoverClose = data;
			TD4191_SetActivceArea();
			clear_swipe(client);
			break;

		default:
			TOUCH_ERR("Invalid notification ( notify = %d )\n", notify);
			break;
	}

	return;

}
TouchDeviceSpecificFunction td4191_Func = {

	.Initialize = TD4191_Initialize,
	.Reset = TD4191_Reset,
	.Connect = TD4191_Connect,
	.InitRegister = TD4191_InitRegister,
	.InterruptHandler = TD4191_InterruptHandler,
	.ReadIcFirmwareInfo = TD4191_ReadIcFirmwareInfo,
	.GetBinFirmwareInfo = TD4191_GetBinFirmwareInfo,
	.UpdateFirmware = TD4191_UpdateFirmware,
	.SetLpwgMode = TD4191_SetLpwgMode,
	.DoSelfDiagnosis = TD4191_DoSelfDiagnosis,
	.AccessRegister = TD4191_AccessRegister,
	.device_attribute_list = TD4191_attribute_list,
	.ClearInterrupt = TD4191_ClearInterrupt,
	.NotifyHandler = TD4191_NotifyHandler,
};


/* End Of File */


