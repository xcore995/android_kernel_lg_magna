/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifdef BUILD_LK
#include <string.h>
#include <mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <linux/string.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#include "lcm_drv.h"
#include <cust_gpio_usage.h>

#include "disp_drv_platform.h"
#include "dsi_drv.h"

#include <mach/mt_pwm.h>

#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
/* pixel */
#if 1
#define FRAME_WIDTH			(720)
#define FRAME_HEIGHT			(1280)
#else
#define FRAME_WIDTH			(540)
#define FRAME_HEIGHT			(960)
#endif

/* physical dimension */
#define PHYSICAL_WIDTH        (71)
#define PHYSICAL_HEIGHT         (126)

#define LCM_ID       (0xb9)
#define LCM_DSI_CMD_MODE		0

#define REGFLAG_DELAY 0xAB
#define REGFLAG_END_OF_TABLE 0xAA	/* END OF REGISTERS MARKER */

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define UDELAY(n)		(lcm_util.udelay(n))
#define MDELAY(n)		(lcm_util.mdelay(n))

#define GPIO_INCELL_DISP_TE_PWM         (GPIO90 | 0x80000000)
#define GPIO_INCELL_DISP_TE_M_GPIO      GPIO_MODE_00
#define GPIO_INCELL_DISP_TE_M_PWM       GPIO_MODE_02

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */

#define dsi_set_cmdq_V3(para_tbl, size, force_update)	\
	lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	\
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	\
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)	\
	cm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)	\
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)	\
	cm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)	\
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static unsigned int need_set_lcm_addr = 1;
extern int wakeup_by_swipe;
#define LGE_LPWG_SUPPORT

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

#if defined(CONFIG_LEDS_LM3632) || defined(CONFIG_BACKLIGHT_LM3632)
extern void chargepump_dsv_ctrl(int dsv_en);
#endif

static LCM_setting_table_V3 lcm_initialization_setting_V3[] = {
	/* {0x05,0x0001, 1, {0x0000}},   //sw reset */
	{0x05, 0x11, 1, {0x00} },
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 120, {} },

	{0x29, 0x90, 3, {0x00, 0x43, 0x01} },
	{0x29, 0x01, 3, {0x00, 0x65, 0x02} },	/* flip */
	{0x29, 0x92, 3, {0x00, 0x13, 0x01} },	/* vcom 20150226 0xA5 -> 0x92 */
	{0x29, 0x03, 3, {0x07, 0x6A, 0x01} },	/* gate bias */
	{0x29, 0x0A, 3, {0x06, 0xD7, 0x01} },	/* increase line data storage */

	{0x05, 0x29, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_initialization_sleep_in_V3[] = {

	{0x05, 0x28, 1, {0x00} },
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10 /*200 */ , {} },	/* MDELAY(120) */
	{0x05, 0x10, 1, {0x00} },
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 100 /* 200 */ , {} },	/* MDELAY(120) */
	/* {0x29,0x40, 3, {0x00,0x42,0x01}},                    //ULPS only data0 lane */
	/* {0x29,0xC3, 3, {0x00,0x44,0x01}}, //bit period changed */
	/* {0x29,0xD3, 3, {0x00,0x44,0x01}}, */
	/* {REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 200,{}}, */

	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_suspend_ULPS_for_data0_in_V3[] = {
	{0x29, 0x40, 3, {0x00, 0x42, 0x01} },	/* ULPS only data0 lane */
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_initialization_pre_sleep_out_V3[] = {
	{0x29, 0x00, 3, {0x00, 0x42, 0x01} },	/* ULPS only data0 lane */
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {} },
	{0x05, 0x11, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_initialization_post_sleep_out_V3[] = {
	{0x29, 0x90, 3, {0x00, 0x43, 0x01} },	/* hsRX */
	{0x29, 0x01, 3, {0x00, 0x65, 0x02} },	/* flip */
	{0x29, 0x92, 3, {0x00, 0x13, 0x01} },	/* vcom 20150226 A5 -> 92 */
	{0x29, 0x03, 3, {0x07, 0x6A, 0x01} },	/* gate bias */
	{0x29, 0x0A, 3, {0x06, 0xD7, 0x01} },	/* increase line data storage */

	{0x29, 0xC5, 3, {0x00, 0x17, 0x03} },	/* CKV control */
	{0x29, 0x01, 3, {0x00, 0x1E, 0x01} },	/* AGH_ADJ */

/* {0x29,0x01, 3, {0x00,0x35,0x00}}, //tearing effect */

	/* {0x29,0x00, 4, {0x00,0x42,0x01}}, //ULPS only data0 lane */
	{0x05, 0x29, 1, {0x00} },

	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};

static LCM_setting_table_V3 lcm_initialization_sleep_out_V3[] = {

	{0x29, 0x00, 3, {0x00, 0x42, 0x01} },	/* ULPS only data0 lane */
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {} },
	{0x05, 0x11, 1, {0x00} },
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 120, {} },
	/* {0x29,0x00, 3, {0x00,0x42,0x01}}, //ULPS only data0 lane */

	{0x29, 0x90, 3, {0x00, 0x43, 0x01} },
	{0x29, 0x01, 3, {0x00, 0x65, 0x02} },	/* flip */
	{0x29, 0x92, 3, {0x00, 0x13, 0x01} },	/* vcom 20150226 A5 -> 92 */
	{0x29, 0x03, 3, {0x07, 0x6A, 0x01} },	/* gate bias */
	{0x29, 0x0A, 3, {0x06, 0xD7, 0x01} },	/* increase line data storage */
	/* {0x29,0x00, 3, {0x00,0x42,0x01}}, //ULPS only data0 lane */

	/* {0x29,0x00, 4, {0x00,0x42,0x01}}, //ULPS only data0 lane */
	{0x05, 0x29, 1, {0x00} },

	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },

};

static LCM_setting_table_V3 lcm_lut_update_in_V3[] = {
	{0x05, 0x26, 1, {0x00} },
	{REGFLAG_END_OF_TABLE, 0x00, 1, {} },
};


static struct LCM_setting_table __attribute__ ((unused)) lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct pwm_spec_config pwm_setting = {
	.pwm_no = PWM2,
	.mode = PWM_MODE_OLD,
	.clk_src = PWM_CLK_OLD_MODE_32K,
	.pmic_pad = false,
	.PWM_MODE_OLD_REGS.IDLE_VALUE = IDLE_FALSE,
	.PWM_MODE_OLD_REGS.GUARD_VALUE = 0,	/* in old mode, this value is invalid */
	.PWM_MODE_OLD_REGS.GDURATION = 0,
	.PWM_MODE_OLD_REGS.WAVE_NUM = 0,	/* 0 == none stop until the PWM is disable */
	/* 135 : 60.24HZ, margin btw touch_en & pwm_falling_edge=740uS */
	.clk_div = CLK_DIV4,
	.PWM_MODE_OLD_REGS.DATA_WIDTH = 138,	/* 58.2fps //135 //60.2fps */
	.PWM_MODE_OLD_REGS.THRESH = 136
};

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
	LCM_PRINT("[LCD] push_table\n");
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy((void *)&lcm_util, (void *)util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	params->physical_width = PHYSICAL_WIDTH;
	params->physical_height = PHYSICAL_HEIGHT;

	params->dsi.mode = SYNC_EVENT_VDO_MODE;	/* BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE; */
	/* params->dsi.switch_mode = CMD_MODE; */
	/* params->dsi.switch_mode_enable = 0; */
	/* enable tearing-free */
	/* params->dbi.te_mode                               = LCM_DBI_TE_MODE_DISABLED; */
	/* params->dbi.te_edge_polarity              = LCM_POLARITY_RISING; */

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */
	/* Video mode setting */
	/* params->dsi.intermediat_buffer_num = 0; */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.null_packet_en = TRUE;
	params->dsi.cont_clock = TRUE;

	LCM_PRINT("[LCD] lcm_get_params\n");
	params->dsi.mixmode_enable = TRUE;
	params->dsi.pwm_fps = 60;
	params->dsi.mixmode_mipi_clock = 255;
	params->dsi.send_frame_enable = FALSE;
	params->dsi.ssc_range = 2;

	params->dsi.vertical_sync_active = 126;	/* 4; */
	params->dsi.vertical_backporch = 127;	/* mtk max */
	params->dsi.vertical_frontporch = 127;	/* 27; */
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	/* params->dsi.ulps_sw_enable = TRUE; // using HW ULPS */
	params->dsi.horizontal_sync_active = 4;	/* 4; //0318 4->5 */
	params->dsi.horizontal_backporch = 4;	/* 4; //32; //0318 4->5 */
	params->dsi.horizontal_frontporch = 80;	/* 78; //127; //78;//127; //32; //0318 78->82 */
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 255;	/* 34;// 208; */

	LCM_PRINT("[LCD] lcm_get_params [Set PWM mode : en(%d),fps(%d),clock(%d)]\n",
		  params->dsi.mixmode_enable, params->dsi.pwm_fps, params->dsi.mixmode_mipi_clock);

}

static void init_lcm_registers(void)
{
	dsi_set_cmdq_V3(lcm_initialization_setting_V3, sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);

	LCM_PRINT("[LCD] init_lcm_registers\n");
}

static void init_lcm_registers_sleep(void)
{
	unsigned int data_array[1];

/* MDELAY(10); */
	data_array[0] = 0x00280500;	/* Display Off */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);
	data_array[0] = 0x00100500;	/* enter sleep */
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);
	LCM_PRINT("[LCD] init_lcm_registers_sleep\n");
}


/* VCAMD 1.8v LDO enable */
static void ldo_1v8io_on(void)
{
#ifdef BUILD_UBOOT
#error "not implemeted"
#elif defined(BUILD_LK)
	upmu_set_rg_vcamd_vosel(3);	/* VGP2_SEL= 101 : 2.8V , 110 : 3.0V */
	upmu_set_rg_vcamd_en(1);
#else
		upmu_set_rg_vcamd_vosel(3);  // VGP2_SEL= 101 : 2.8V , 110 : 3.0V
		upmu_set_rg_vcamd_en(1);
		//hwPowerOn(MT6323_POWER_LDO_VCAMD, VOL_1800, "1V8_LCD_VIO_MTK_S");
#endif
}

/* VCAMD 1.8v LDO disable */
static void ldo_1v8io_off(void)
{
#ifdef BUILD_UBOOT
#error "not implemeted"
#elif defined(BUILD_LK)
	upmu_set_rg_vcamd_en(0);
#else
		upmu_set_rg_vcamd_en(0);
		//hwPowerDown(MT6323_POWER_LDO_VCAMD, "1V8_LCD_VIO_MTK_S");
#endif
}

/* VGP2 3.0v LDO enable */
static void ldo_3v0_on(void)
{
#ifdef BUILD_UBOOT
#error "not implemeted"
#elif defined(BUILD_LK)
	upmu_set_rg_vgp1_vosel(6);	/* VGP2_SEL= 101 : 2.8V , 110 : 3.0V */
	upmu_set_rg_vgp1_en(1);

#else
		upmu_set_rg_vgp1_vosel(6);  // VGP2_SEL= 101 : 2.8V , 110 : 3.0V
		upmu_set_rg_vgp1_en(1);
		//hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3000, "3V0_TOUCH_VDD");
#endif
}

/* VGP2 3.0v LDO disable */
static void ldo_3v0_off(void)
{
#ifdef BUILD_UBOOT
#error "not implemeted"
#elif defined(BUILD_LK)
	upmu_set_rg_vgp1_en(0);
#else
		upmu_set_rg_vgp1_en(0);
		//hwPowerDown(MT6323_POWER_LDO_VGP1, "3V0_TOUCH_VDD");
#endif
}

/*
DSV power +5V,-5v
*/
static void ldo_p5m5_dsv_5v5_on(void)
{
#if defined(CONFIG_LEDS_LM3632) || defined(CONFIG_BACKLIGHT_LM3632)
    chargepump_dsv_ctrl(TRUE);
#else
	mt_set_gpio_mode(GPIO_DSV_AVDD_EN, GPIO_DSV_AVDD_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_DSV_AVDD_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_DSV_AVDD_EN, GPIO_DIR_OUT);
	mt_set_gpio_mode(GPIO_DSV_AVEE_EN, GPIO_DSV_AVEE_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_DSV_AVEE_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_DSV_AVEE_EN, GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_DSV_AVEE_EN, GPIO_OUT_ONE);
	/* MDELAY(4); */
	mt_set_gpio_out(GPIO_DSV_AVDD_EN, GPIO_OUT_ONE);
#endif
}

static void ldo_p5m5_dsv_5v5_off(void)
{
#if defined(CONFIG_LEDS_LM3632) || defined(CONFIG_BACKLIGHT_LM3632)
    chargepump_dsv_ctrl(FALSE);
#else
	mt_set_gpio_mode(GPIO_DSV_AVDD_EN, GPIO_DSV_AVDD_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_DSV_AVDD_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_DSV_AVDD_EN, GPIO_DIR_OUT);
	mt_set_gpio_mode(GPIO_DSV_AVEE_EN, GPIO_DSV_AVEE_EN_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_DSV_AVEE_EN, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_DSV_AVEE_EN, GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_DSV_AVDD_EN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_DSV_AVEE_EN, GPIO_OUT_ZERO);
#endif
}


static void reset_lcd_module(unsigned char reset)
{
	mt_set_gpio_mode(GPIO_LCM_RST, GPIO_LCM_RST_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_LCM_RST, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);

	if (reset) {
		mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
		/* MDELAY(50); //0318 block */
	} else {
		mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
	}
}

void lcm_init_power(void)
{
	MDELAY(50);
//	ldo_3v0_on();
//	ldo_1v8io_on();
//	MDELAY(10);
	ldo_p5m5_dsv_5v5_on();
	MDELAY(50);

	reset_lcd_module(0);
	MDELAY(20);

	LCM_PRINT("[LCD] lcm_init_Power \n");
}

static void lcm_init(void)
{
	DSI_clk_HS_mode(1);
	MDELAY(120);
	init_lcm_registers();	/* SET EXTC ~ sleep out register */

	MDELAY(120);

	/* init_lcm_registers_added();     //Display On */
	need_set_lcm_addr = 1;
	LCM_PRINT("[SEOSCTEST] lcm_init \n");
	LCM_PRINT("[LCD] lcm_init \n");
}

static void set_enable_te_framesync(void)
{
	LCM_PRINT("=============mt_pmic_pwm2_test===============\n");
	mt_set_gpio_mode(GPIO_INCELL_DISP_TE_PWM, GPIO_INCELL_DISP_TE_M_PWM);

	LCM_PRINT("PWM: clk_div = %x, clk_src = %x, pwm_no = %x\n", pwm_setting.clk_div,
		  pwm_setting.clk_src, pwm_setting.pwm_no);
	pwm_set_spec_config(&pwm_setting);

}

void lcm_set_fps(int fps)
{
	unsigned int width = 32 * 1024 / (4 * fps) - 1;

	LCM_PRINT("DSI_set_fps_for_PWM fps (%d), widht (%d)\n", fps, width);

	pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = width;
	pwm_setting.PWM_MODE_OLD_REGS.THRESH = width / 2;
}

void lcm_set_pwm_for_mix(int enable)
{
	LCM_PARAMS params;

	lcm_get_params(&params);
	if (params.dsi.pwm_fps == 0) {
		LCM_PRINT("Please set PWM fps\n");
		return;
	}

	if (enable) {
		lcm_set_fps(params.dsi.pwm_fps);
		set_enable_te_framesync();
	} else {
		mt_pwm_disable(pwm_setting.pwm_no, pwm_setting.pmic_pad);
	}
	LCM_PRINT("[LCD] lcm_set_pwm (%d)\n", enable);

	return;
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	dsi_set_cmdq_V3(lcm_initialization_sleep_in_V3, sizeof(lcm_initialization_sleep_in_V3) / sizeof(LCM_setting_table_V3), 1);

	MDELAY(120);
	lcm_set_pwm_for_mix(0);
	MDELAY(16);

	DPI_DisableClk();
	DSI_Reset();
	DSI_clk_HS_mode(1);
	DSI_SetMode(0);
	/* MDELAY(1); */

    dsi_set_cmdq_V3(lcm_suspend_ULPS_for_data0_in_V3, sizeof(lcm_suspend_ULPS_for_data0_in_V3) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(200);
	LCM_PRINT("[LCD] lcm_suspend\n");
}


static void lcm_resume(void)
{
	unsigned int data_array[1];

	MDELAY(5);
if(wakeup_by_swipe == false){
	//reset_lcd_module(0);
	//MDELAY(10); //seosc
	ldo_p5m5_dsv_5v5_on();
	MDELAY(5);
	reset_lcd_module(0);
	MDELAY(1);
	reset_lcd_module(1);
	MDELAY(1);
	reset_lcd_module(0);
	MDELAY(50);
}else{
	LCM_PRINT("[LCD] lcm_resume wakeup_by_swipe\n");
}
	DSI_clk_HS_mode(1);
	MDELAY(120);

	dsi_set_cmdq_V3(lcm_initialization_pre_sleep_out_V3, sizeof(lcm_initialization_pre_sleep_out_V3) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(120);
	dsi_set_cmdq_V3(lcm_initialization_post_sleep_out_V3, sizeof(lcm_initialization_post_sleep_out_V3) / sizeof(LCM_setting_table_V3), 1);


	need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_resume\n");
}

static void lcm_resume_power(void)
{
	if(wakeup_by_swipe == false){
		//if(!is_update)mutex_lock(pMutexTouch);
		ldo_p5m5_dsv_5v5_off();
		ldo_1v8io_off();
		ldo_3v0_off();
		reset_lcd_module(1); //removed reset delay 50ms

		//chargepump_dsv_reset();//no need it for ESD

		MDELAY(10); //0318 100ms -> 10ms
		ldo_3v0_on();
		ldo_1v8io_on();
		//MDELAY(10);
		//ldo_p5m5_dsv_5v5_on(); // 0318 move to LP11
		MDELAY(50);
		//SET_RESET_PIN(1);
		//reset_lcd_module(0); //0318 move to LP11
		//MDELAY(150);
		LCM_PRINT("[LCD] lcm_resume_power \n");
	}else{
		LCM_PRINT("[LCD] lcm_resume_power wakeup_by_swipe\n");
	}
}

void lcm_shutdown_power(void)
{
	ldo_p5m5_dsv_5v5_off();
	MDELAY(5);
	ldo_1v8io_off();
	ldo_3v0_off();
	MDELAY(5);
	reset_lcd_module(1); //removed reset delay 50m

	LCM_PRINT("[LCD] lcm_shutdown_power \n");
}


void lcm_force_lut_update(void)
{
	dsi_set_cmdq_V3(lcm_lut_update_in_V3, sizeof(lcm_lut_update_in_V3) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(120);

	LCM_PRINT("[LCD] lut update for upgrading Touch FW \n");
}


static void lcm_esd_recover(void)
{
	lcm_suspend();
	lcm_resume();

	LCM_PRINT("[LCD] lcm_esd_recover\n");
}

static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	/* need update at the first time */
	if (need_set_lcm_addr) {
		data_array[0] = 0x00053902;
		data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
		data_array[2] = (x1_LSB);
		dsi_set_cmdq(data_array, 3, 1);

		data_array[0] = 0x00053902;
		data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
		data_array[2] = (y1_LSB);
		dsi_set_cmdq(data_array, 3, 1);
		need_set_lcm_addr = 0;
	}

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
	LCM_PRINT("[LCD] lcm_update\n");
}

static unsigned int lcm_compare_id(void)
{
#if 0
	unsigned int id = 0;
	unsigned char buffer[2];
	unsigned int array[16];
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(10);		/* Must over 6 ms */
	array[0] = 0x00043902;
	array[1] = 0x8983FFB9;	/* page enable */
	dsi_set_cmdq(array, 2, 1);
	MDELAY(10);
	array[0] = 0x00023700;	/* return byte number */
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0];
	LCM_PRINT("%s, id = 0x%08x\n", __func__, id);
	return (LCM_ID == id) ? 1 : 0;
#else
	LCM_PRINT("[SEOSCTEST] lcm_compare_id\n");
	return 1;
#endif
}

/* --------------------------------------------------------------------------- */
/* Get LCM Driver Hooks */
/* --------------------------------------------------------------------------- */
LCM_DRIVER lh570_hd720_dsi_vdo_lgd_phase3_drv = {
	.name = "lh570_hd720_dsi_vdo_lgd_phase3",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.init_power = lcm_init_power,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.resume_power = lcm_resume_power,
	.shutdown_power = lcm_shutdown_power,
/* .compare_id = lcm_compare_id, */
/* .update = lcm_update, */
#if (!defined(BUILD_UBOOT) && !defined(BUILD_LK))
/* .esd_recover = lcm_esd_recover, */
#endif
	.set_pwm_for_mix = lcm_set_pwm_for_mix,
};
