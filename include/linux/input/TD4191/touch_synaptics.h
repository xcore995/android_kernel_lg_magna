/* include/linux/lge_touch_core.h
 *
 * Copyright (C) 2012 LGE.
 *
 * Writer: yehan.ahn@lge.com, 	hyesung.shin@lge.com
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
 */
#include <linux/input/lgtp_common.h>

#ifndef LGE_TOUCH_SYNAPTICS_H
#define LGE_TOUCH_SYNAPTICS_H

#define NUM_OF_EACH_FINGER_DATA_REG		8
#define MAX_NUM_OF_FINGERS			10

#define DESCRIPTION_TABLE_START			0xe9
#define EXIST_OFFSET                            0xEE

#define PAGE_SELECT_REG				0xFF		/* Button exists Page 02 */
#define PAGE_MAX_NUM				5		/* number of page register */

#define FW_VER_INFO_NUM				4

#define F12_NO_OBJECT_STATUS		(0x00)
#define F12_FINGER_STATUS			(0x01)
#define F12_STYLUS_STATUS			(0x02)
#define F12_PALM_STATUS				(0x03)
#define F12_HOVERING_FINGER_STATUS	(0x05)
#define F12_GLOVED_FINGER_STATUS	(0x06)

#define S3621           0
#define S3528_A0        1
#define S3528_A1        2
#define S3528_A1_SUN    3
#define TD4191    4

#define F35_ERROR_CODE_OFFSET 0
#define F35_CHUNK_NUM_LSB_OFFSET 0
#define F35_CHUNK_NUM_MSB_OFFSET 1
#define F35_CHUNK_DATA_OFFSET 2
#define F35_CHUNK_COMMAND_OFFSET 18

#define F35_CHUNK_SIZE 16
#define F35_ERASE_ALL_WAIT_MS 2000
#define F35_RESET_WAIT_MS 250

enum {
	POWER_OFF = 0,
	POWER_ON,
	POWER_SLEEP,
	POWER_WAKE,
};

struct function_descriptor {
	u8 	query_base;
	u8 	command_base;
	u8 	control_base;
	u8 	data_base;
	u8 	int_source_count;
	u8 	id;
};

struct ts_ic_function {
	struct function_descriptor dsc;
	u8 	function_page;
};

struct finger_data {
	u8	finger_reg[MAX_NUM_OF_FINGERS][NUM_OF_EACH_FINGER_DATA_REG];
};

struct button_data {
	u16	key_code;
};

struct cur_touch_data {
	u8	device_status_reg;		/* DEVICE_STATUS_REG */
	u8	interrupt_status_reg;
	u8	button_data_reg;
	struct finger_data	finger;
	struct button_data	button;
};

struct synaptics_ts_fw_info {
	u8		fw_version[5];
	u8		fw_product_id[11];
	u8		fw_image_version[5];
	u8		fw_image_product_id[11];
	unsigned char	*fw_start;
	unsigned char   family;
	unsigned char   fw_revision;
	unsigned long	fw_size;
	u8		need_rewrite_firmware;
};

struct lpwg_control {
	u8		lpwg_mode;
	u8              screen;
	u8              sensor;
	u8              qcover;
	u8		double_tap_enable;
	u8 		password_enable;
	u8		signature_enable;
	u8		lpwg_is_enabled;
	atomic_t	is_suspend;
};

struct lpwg_password_data {
	u8		tap_count;
	u8		data_num;
	u8              double_tap_check;
	struct point 	data[MAX_POINT_SIZE_FOR_LPWG];
};

struct state_flag {
	u8		ts_noise_log_flag;
	u8		check_noise_menu;
};

struct palm_data {
	bool curr_palm_mask[MAX_NUM_OF_FINGERS];
	bool prev_palm_mask[MAX_NUM_OF_FINGERS];
	struct point palm_coordinate[MAX_NUM_OF_FINGERS];
	u8 curr_palm_num;
	u8 prev_palm_num;
	bool all_palm_released;
};

#define TOUCH_PWR_NUM	3
#define TRX_MAX 32

struct touch_firmware_module {
	char	fw_image[256];
	u8	need_upgrade;
};

struct touch_power_info {
	u8	type;			/* 0:none, 1 : gpio, 2: regulator */
	char	name[16];		/* if type == 1 : gpio active contition "low" or "high" */
						/* if type == 2 : supply name for regulator */
	int	value;			/* if type == 1 : gpio pin no. */
						/* if type == 2 : regulator voltage */
};

struct bouncing_filter_role {
	u32	enable;
};

struct grip_filter_role {
	u32	enable;
	u32	edge_region;
	u32	max_delta;
	u32	width_ratio;
};

struct accuracy_filter_role {
	u32	enable;
	u32	min_delta;
	u32	curr_ratio;
	u32	min_pressure;
};

struct jitter_filter_role {
	u32	enable;
	u32	curr_ratio;
};

struct quickcover_filter_role {
	u32 enable;
	u32 X1;
	u32 X2;
	u32 Y1;
	u32 Y2;
};

struct ghost_detection_enable_check {
	u8 ghost_detection_enable;
	u8 ta_noise_chk;
	u8 incoming_call_chk;
	u8 first_finger_chk;
	u8 pressure_zero_chk;
	u8 ta_debouncing_chk;
	u8 press_interval_chk;
	u8 diff_fingers_chk;
	u8 subtraction_finger_chk;
	u8 long_press_chk;
	u8 button_chk;
	u8 rebase_repetition_chk;
};

struct ghost_detection_role {
	struct ghost_detection_enable_check check_enable;
	u32 ghost_detection_chk_cnt;
	u32 jitter_value;
	u32 first_finger_time;
	u32 ta_debouncing_cnt;
	u32 ta_debouncing_finger_num;
	bool pressure_zero;
	u32 press_interval;
	u32 diff_finger_num;
	u32 subtraction_time;
	u32 subtraction_finger_cnt;
	bool force_continuous_mode;
	u32 long_press_chk_time;
	u32 long_press_cnt;
	u32 button_int_num;
	u32 button_duration;
	u32 rebase_since_init;
	u32 rebase_since_rebase;
};

struct crack_detection_role {
	u32 use_crack_mode; /* Yes = 1, No = 0 */
	u32 min_cap_value;
};



struct touch_operation_role {
	u32	protocol_type;
	u32	report_mode;
	u32	delta_pos_threshold;
	u32	booting_delay;
	u32	reset_delay;
	u32	wake_up_by_touch;
	u32	use_sleep_mode; // Yes = 1, No = 0
	u32     use_lpwg_all;
	u32     use_security_mode;
	u32     thermal_check;
	u32	use_hover_finger;
	u32	use_rmi_dev;
	u32	palm_ctrl_mode;
	u32	mini_os_finger_amplitude;
	u32 ub_i2c_addr;
	u32 i2c_addr;
	unsigned long	irqflags;
	struct bouncing_filter_role	*bouncing_filter;
	struct grip_filter_role		*grip_filter;
	struct accuracy_filter_role	*accuracy_filter;
	struct jitter_filter_role	*jitter_filter;
	struct quickcover_filter_role *quickcover_filter;
	struct ghost_detection_role	*ghost_detection;
	struct crack_detection_role     *crack_detection;
};

struct touch_platform_data {
	u32	int_pin;
	u32	reset_pin;
	u32 fw_version[FW_VER_INFO_NUM];
	struct touch_operation_role	*role;
	//struct touch_power_module	*pwr;
	struct touch_power_info		pwr[TOUCH_PWR_NUM];
	struct touch_firmware_module	*fw;
	struct touch_firmware_module	*fw_recovery;
	const char* inbuilt_fw_name;
	const char* inbuilt_recovery_fw_name;
	const char* inbuilt_fw_name_s3621;
	const char* inbuilt_fw_name_s3528_a1;
	const char* inbuilt_fw_name_s3528_a1_suntel;
	const char* panel_spec;
	int tx_cap[TRX_MAX];
	int rx_cap[TRX_MAX];
	int tx_ch_count;
	int rx_ch_count;
	u8	touch_count_num;
	int ref_chk_option[4];
};

struct touch_firmware_info {
	u8      ic_fw_identifier[31];   /* String */
	u8      ic_fw_version[11];      /* String */
	char    fw_path[256];           /* used for dynamic firmware upgrade */
	char    fw_path_s3528_a1[256];  /* used for dynamic firmware upgrade */
	char    fw_path_s3528_a1_suntel[256]; /* used for dynamic firmware upgrade */
	char    fw_path_s3621[256];     /* used for dynamic firmware upgrade */
	u8      fw_force_upgrade;       /* used for dynamic firmware upgrade */
	u8      fw_force_upgrade_cat;   /* used for dynamic firmware upgrade */
};

enum {
	CONTINUOUS_REPORT_MODE = 0,
	REDUCED_REPORT_MODE,
	DEFAULT_REPORT_MODE,
};
enum{
	PALM_REJECT_FW,
	PALM_REJECT_DRIVER,
	PALM_REPORT,
};
enum {
	IC_CTRL_BASELINE_REBASE=3,
	IC_CTRL_REPORT_MODE,
	IC_CTRL_THERMAL,
};

struct synaptics_ts_data {
	u8	is_probed;
	u8	is_init;
	struct lpwg_control	lpwg_ctrl;
	struct lpwg_password_data	pw_data;
//	struct regulator	*regulator_vdd;
//	struct regulator	*regulator_vio;
//	struct regulator *vdd_regulator[TOUCH_PWR_NUM];
	struct i2c_client	*client;
	struct mutex			thread_lock;
	struct ts_ic_function	common_fc;
	struct ts_ic_function	lpwg_fc;
	struct ts_ic_function	finger_fc;
	struct ts_ic_function	button_fc;
	struct ts_ic_function	analog_fc;
	struct ts_ic_function	sensor_fc;
	struct ts_ic_function	flash_fc;
	struct cur_touch_data	ts_data;
	struct synaptics_ts_fw_info	fw_info;
    struct touch_fw_info*	touch_fw_info;
	struct delayed_work	work_timer;
	struct delayed_work	diff_node_timer;  //test code
	struct delayed_work     work_palm;
	struct wake_lock	timer_wake_lock;
	struct touch_platform_data	*pdata;
	struct state_info	*state;
	E_TouchReportMode reportMode;
	LGTcLpwgSetting lpwgSetting;
	char *pFirmwareImage;
	u8	fw_flag;
	struct state_flag		ts_state_flag;
	unsigned int bad_sample;
	int h_err_cnt;
	int v_err_cnt;
	bool ubootloader_mode;
	struct palm_data                ts_palm_data;
};

struct synaptics_ts_exp_fn {
	int (*init)(struct synaptics_ts_data *ts);
	void (*remove)(struct synaptics_ts_data *ts);
	void (*reset)(struct synaptics_ts_data *ts);
	void (*reinit)(struct synaptics_ts_data *ts);
	void (*early_suspend)(struct synaptics_ts_data *ts);
	void (*suspend)(struct synaptics_ts_data *ts);
	void (*resume)(struct synaptics_ts_data *ts);
	void (*late_resume)(struct synaptics_ts_data *ts);
	void (*attn)(unsigned char intr_status_reg);
};

struct synaptics_ts_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query_6;
			struct {
				unsigned char ctrl_00_is_present:1;
				unsigned char ctrl_01_is_present:1;
				unsigned char ctrl_02_is_present:1;
				unsigned char ctrl_03_is_present:1;
				unsigned char ctrl_04_is_present:1;
				unsigned char ctrl_05_is_present:1;
				unsigned char ctrl_06_is_present:1;
				unsigned char ctrl_07_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_08_is_present:1;
				unsigned char ctrl_09_is_present:1;
				unsigned char ctrl_10_is_present:1;
				unsigned char ctrl_11_is_present:1;
				unsigned char ctrl_12_is_present:1;
				unsigned char ctrl_13_is_present:1;
				unsigned char ctrl_14_is_present:1;
				unsigned char ctrl_15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_16_is_present:1;
				unsigned char ctrl_17_is_present:1;
				unsigned char ctrl_18_is_present:1;
				unsigned char ctrl_19_is_present:1;
				unsigned char ctrl_20_is_present:1;
				unsigned char ctrl_21_is_present:1;
				unsigned char ctrl_22_is_present:1;
				unsigned char ctrl_23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl_24_is_present:1;
				unsigned char ctrl_25_is_present:1;
				unsigned char ctrl_26_is_present:1;
				unsigned char ctrl_27_is_present:1;
				unsigned char ctrl_28_is_present:1;
				unsigned char ctrl_29_is_present:1;
				unsigned char ctrl_30_is_present:1;
				unsigned char ctrl_31_is_present:1;
			} __packed;
		};
		unsigned char data[5];
	};
};

struct synaptics_ts_f12_query_8 {
	union {
		struct {
			unsigned char size_of_query_9;
			struct {
				unsigned char data_00_is_present:1;
				unsigned char data_01_is_present:1;
				unsigned char data_02_is_present:1;
				unsigned char data_03_is_present:1;
				unsigned char data_04_is_present:1;
				unsigned char data_05_is_present:1;
				unsigned char data_06_is_present:1;
				unsigned char data_07_is_present:1;
			} __packed;
			struct {
				unsigned char data_08_is_present:1;
				unsigned char data_09_is_present:1;
				unsigned char data_10_is_present:1;
				unsigned char data_11_is_present:1;
				unsigned char data_12_is_present:1;
				unsigned char data_13_is_present:1;
				unsigned char data_14_is_present:1;
				unsigned char data_15_is_present:1;
			} __packed;
		};
		unsigned char data[3];
	};
};

enum {
	CHARGERLOGO_MODE = 0,
	NORMAL_BOOT_MODE,
};

enum{
	TS_NOISE_LOG_DISABLE = 0,
	TS_NOISE_LOG_ENABLE,
};

enum{
	MENU_OUT = 0,
	MENU_ENTER,
};

enum {
	THERMAL_LOW = 0,
	THERMAL_HIGH,
};

extern struct workqueue_struct *touch_wq;
error_type synaptics_ts_init(struct i2c_client *client);

extern void SCAN_PDT(void);
int compare_fw_version(struct i2c_client *client, struct touch_fw_info *fw_info);
//mode:0 => write_log, mode:1 && buf => cat, mode:2 && buf => delta
extern int F54Test(int input, int mode, char *buf);
extern int GetImageReport(char *buf);

void synaptics_ts_prox_function(struct synaptics_ts_exp_fn *prox_fn, bool insert);
void synaptics_ts_rmidev_function(struct synaptics_ts_exp_fn *rmidev_fn, bool insert);

/* extern function */
extern int FirmwareUpgrade(struct synaptics_ts_data *ts, const char* fw_path);
extern int FirmwareRecovery(struct synaptics_ts_data *ts, const char *fw_path);
extern void SynaScanPDT(struct synaptics_ts_data *ts);
int synaptics_ts_page_data_read(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data);
error_type synaptics_ts_page_data_write(struct i2c_client *client,u8 page, u8 reg, int size, u8 *data);
error_type synaptics_ts_page_data_write_byte(struct i2c_client *client,u8 page, u8 reg, u8 data);

#endif
