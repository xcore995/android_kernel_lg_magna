#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <string.h>
    #include <mt_gpio.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
    #include <mach/mt_pm_ldo.h>
    #include <mach/mt_gpio.h>

    #include <mach/upmu_common.h>

#endif
#include <cust_gpio_usage.h>

#include <cust_i2c.h>

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

/* -------------------------------------------------------------------------
 *  Local Constants
 * -------------------------------------------------------------------------*/
/* pixel */
#define FRAME_WIDTH             (720)
#define FRAME_HEIGHT            (1280)

/* physical dimension */
#define PHYSICAL_WIDTH          (62)
#define PHYSICAL_HEIGHT         (110)

static const unsigned int BL_MIN_LEVEL =20;
#define REGFLAG_DELAY                       0xFC
#define REGFLAG_END_OF_TABLE                0xFD

/* - LGE_LPWG_SUPPORT - define is knock on mode.
 * If you don't use knock on, please remove or annotate it */
#define LGE_LPWG_SUPPORT

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)        (lcm_util.set_reset_pin((v)))
#define MDELAY(n)               (lcm_util.mdelay(n))

/* Local Functions */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                       lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE                    0

#define GPIO_DW8768_ENP GPIO_LCD_BIAS_ENP_PIN
#define GPIO_DW8768_ENN GPIO_LCD_BIAS_ENN_PIN

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* Local Variables */
struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	/* Display off sequence */
	{0x28, 1, {0x00} },
	{REGFLAG_DELAY, 20, {} },
	/* Sleep Mode On */
	{0x10, 1, {0x00} },
	{REGFLAG_DELAY, 80, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

#ifndef LGE_LPWG_SUPPORT
static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x51, 1, {0xFF} },
	{0x53, 1, {0x2C} },
	{0x55, 1, {0x40} },
	{0xB0, 1, {0x04} },
	{0xC1, 3, {0x84, 0x61, 0x00} },
	{0xD6, 1, {0x01} },
	{0x36, 1, {0x00} },
	/*display on */
	{0x29, 0, {} },
	/* exit sleep */
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} },
};
#else
/* Version 2 & No Gamma  */
static struct LCM_setting_table lcm_initialization_setting[] = {
    {0x51,  1,  {0xFF}},
    {0x53,  1,  {0x0C}},
    {0x55,  1,  {0x01}},	//CABC ON(0x41 GUI mode, 0x42 still mode, 0x43 Movie mode) / CABC OFF(0x40)
    /* Test command */
    {0xB0,  1,  {0x04}},
    /* Test command set(1) */
    {0xB3,  7,  {0x14, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00}},
    /* Test command set(2) */
    {0xB4,  1,  {0x0C}},
    /* Test command set(3) */
    {0xB6,  2,  {0x3A, 0xB3}},
    /* Test command set(4) */
    {0xC1, 44,  {0x84, 0x61, 0x10, 0x52, 0x4A, 0x59, 0x94, 0x20, 0x03, 0x1C,
                 0xD9, 0xD8, 0x82, 0xCF, 0xB9, 0x07, 0x17, 0x6B, 0xD1, 0x80,
                 0x41, 0x82, 0x94, 0x52, 0x4A, 0x09, 0x10, 0x10, 0x10, 0x10,
                 0xA0, 0x40, 0x42, 0x20, 0x12, 0x10, 0x22, 0x00, 0x15, 0x00,
                 0x01, 0x00, 0x00, 0x00}},
    /* Test command set(5) */
    {0xC2,  9,  {0x31, 0xF5, 0x00, 0x00, 0x04, 0x00, 0x08, 0x00, 0x00}},
    /* Test command set(6) */
    {0xC4, 15,  {0x70, 0x00, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x00,
                 0x06, 0x16, 0x16, 0x16, 0x01}},
    /* Test command set(7) */
    {0xC6, 20,  {0x54, 0x10, 0x10, 0x04, 0x49, 0x01, 0x01, 0x01, 0x01, 0x01,
                 0x01, 0x45, 0x0D, 0x01, 0x01, 0x01, 0x01, 0x06, 0x13, 0x03}},
    /* Test command set(8) */
    {0xCB, 12,  {0xF0, 0xF7, 0xFF, 0x3F, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x3C, 0xCF}},
    /* Test command set(9) */
    {0xCC,  1,  {0x0D}},
    /* Test command set(10) */
    {0xD0,  5,  {0x10, 0x91, 0xBB, 0x12, 0x8E}},
    /* Test command set(11) */
    {0xD1,  5,  {0x25, 0x00, 0x19, 0x61, 0x06}},
    /* Test command set(12) */
    {0xD3, 27,  {0x0B, 0x37, 0x9F, 0xBD, 0xB7, 0x33, 0x33, 0x17, 0x00, 0x01,
                 0x00, 0xA0, 0xD8, 0xA0, 0x0D, 0x23, 0x23, 0x33, 0x3B, 0xF7,
                 0x72, 0x07, 0x3D, 0xBF, 0x99, 0x21, 0xFA}},
    /* Test command set(13) */
    {0xD5, 11,  {0x06, 0x00, 0x00, 0x01, 0x28, 0x01, 0x28, 0x00, 0x00, 0x00,
                 0x00}},
    /* Test command set(14) */
    {0xD9, 12,  {0x8A, 0x23, 0xAA, 0xAA, 0x0A, 0xE4, 0x00, 0x06, 0x07, 0x00,
                 0x00, 0x00}},
    /* Test command set(15) */
    {0xEC, 48,  {0x29, 0x00, 0x2C, 0x2B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x09, 0x00, 0x11,
                 0x0F, 0x00, 0x00, 0x00, 0x00, 0x50, 0x06, 0x00, 0x00, 0x00,
                 0x00, 0x05, 0xFC, 0x00, 0x00, 0x00, 0x03, 0x40, 0x02, 0x13,
                 0x02, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}},
    /* Test command set(16) */
    {0xED, 31,  {0x3B, 0x02, 0x02, 0x03, 0x21, 0x11, 0x00, 0x93, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x03, 0x1E, 0x21, 0x00, 0xDF, 0x00, 0xDD,
                 0x06, 0xF3, 0x3F, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00,
                 0x00}},
    /* Test command set(17) */
    {0xEE, 31,  {0x1D, 0x02, 0x02, 0x03, 0x00, 0x10, 0x00, 0x02, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x01, 0xC5, 0x01, 0xC4,
                 0x0E, 0x28, 0x23, 0x00, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00,
                 0x00}},
    /* Test command set(18) */
    {0xEF, 62,  {0x3B, 0x02, 0x02, 0x03, 0x21, 0x11, 0x00, 0x93, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x03, 0x1E, 0x21, 0x00, 0xDF, 0x00, 0xDD,
                 0x06, 0xF3, 0x3F, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x3B, 0x02, 0x02, 0x03, 0x21, 0x11, 0x00, 0x93, 0x00,
                 0xF0, 0x00, 0x00, 0x54, 0x00, 0x03, 0x1E, 0x21, 0x00, 0xDF,
                 0x00, 0xDD, 0x06, 0xF3, 0x3F, 0x00, 0x15, 0x00, 0x00, 0x00,
                 0x00, 0x00}},
    /* Gamma setting(1) */
    {0xC7, 29,  {0x00, 0x09, 0x0F, 0x18, 0x25, 0x33, 0x3D, 0x4C, 0x30, 0x38,
                 0x46, 0x57, 0x64, 0x69, 0x76, 0x00, 0x09, 0x0F, 0x18, 0x25,
                 0x33, 0x3D, 0x4C, 0x30, 0x38, 0x46, 0x57, 0x64, 0x69, 0x76}},
    /* Gamma setting(2) */
    {0xC8, 19,  {0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00}},
    /* Color Enhancement */
	{0xCA,	32,	{0x01, 0x70, 0x9A, 0x95, 0xA2, 0x98, 0x95, 0x8F, 0x3F, 0x3F,
				 0x80, 0x80, 0x08, 0x80, 0x08, 0x3F, 0x08, 0x90, 0x0C, 0x0C,
				 0x0A, 0x06, 0x04, 0x04, 0x00, 0xC8, 0x10, 0x10, 0x3F, 0x3F,
				 0x3F, 0x3F}},
    /* Back Light Control 1 (for CABC) */
    {0xB8, 6,  {0x07, 0x90, 0x1E, 0x00, 0x40, 0x32}},
    /* Back Light Control 2 (for CABC) */
    {0xB9, 6,  {0x07, 0x8C, 0x3C, 0x20, 0x2D, 0x87}},
    /* Back Light Control 3 (for CABC) */
    {0xBA, 6,  {0x07, 0x82, 0x3C, 0x10, 0x3C, 0xB4}},
    /* Back Light Control 4 (for CABC) */
    {0xCE, 24,  {0x7D, 0x40, 0x43, 0x49, 0x55, 0x62, 0x71, 0x82, 0x94, 0xA8,
                 0xB9, 0xCB, 0xDB, 0xE9, 0xF5, 0xFC, 0xFF, 0x02, 0x00, 0x04,
                 0x04, 0x44, 0x04, 0x01}},
    /* Test command */
    {0xD6, 1, {0x01}},
    /* set address mode */
    {0x36,  1,  {0x00}},
    /*display on*/
    {0x29,  0,  {}},
    /* exit sleep*/
    {0x11,  0,  {}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}},
};
#endif


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    /* Display off sequence */
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    /* Sleep Mode On */
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 80, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

/* LCM Driver Implementations */
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy((void*)&lcm_util, (void*)util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// physical size
    params->physical_width  = PHYSICAL_WIDTH;
    params->physical_height = PHYSICAL_HEIGHT;
	params->dsi.mode = SYNC_EVENT_VDO_MODE;
	params->dsi.switch_mode = CMD_MODE;
	params->dsi.switch_mode_enable = 0;

	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 1;
	params->dsi.vertical_backporch = 3;
	params->dsi.vertical_frontporch = 6;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 5;
	params->dsi.horizontal_backporch = 60;
	params->dsi.horizontal_frontporch = 140;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 234;
}
static void init_lcm_registers(void)
{
    unsigned int data_array[32];

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

    LCM_PRINT("[LCD] init_lcm_registers \n");
}

/* VCAMD 1.8v LDO enable */
static void ldo_1v8io_on(void)
{
#ifdef BUILD_UBOOT
    #error "not implemeted"
#elif defined(BUILD_LK)
    dprintf(0, "vgp3 on\n");
    MDELAY(1);
    mt6325_upmu_set_rg_vgp3_vosel(3);
    mt6325_upmu_set_rg_vgp3_en(1);
#else
    hwPowerOn(MT6325_POWER_LDO_VGP3, VOL_1800, "mc90ds_LCD");
#endif
}

/* VCAMD 1.8v LDO disable */
static void ldo_1v8io_off(void)
{
#ifdef BUILD_UBOOT
    #error "not implemeted"
#elif defined(BUILD_LK)
    mt6325_upmu_set_rg_vgp3_en(0);
#else
    mt6325_upmu_set_rg_vgp3_en(0);
#endif
}

/* VGP2 3.0v LDO enable */
static void ldo_3v0_on(void)
{
#ifdef BUILD_UBOOT
    #error "not implemeted"
#elif defined(BUILD_LK)
    mt6325_upmu_set_rg_vgp1_vosel(6);  // VGP1_SEL= 101 : 2.8V , 110 : 3.0V
    mt6325_upmu_set_rg_vgp1_cal(11);  // offset b1101: 0.1V
    mt6325_upmu_set_rg_vgp1_en(1);
#else
    hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_3000, "TOUCH");
#endif
}

/* VGP2 3.0v LDO disable */
static void ldo_3v0_off(void)
{
#ifdef BUILD_UBOOT
    #error "not implemeted"
#elif defined(BUILD_LK)
    mt6325_upmu_set_rg_vgp1_en(0);
#else
    mt6325_upmu_set_rg_vgp1_en(0);
#endif
}

static void ldo_p5m5_dsv_5v5_on(void)
{
    mt_set_gpio_mode(GPIO_DW8768_ENP, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_DW8768_ENP, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DW8768_ENP, GPIO_OUT_ONE);
    MDELAY(1);
    mt_set_gpio_mode(GPIO_DW8768_ENN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_DW8768_ENN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DW8768_ENN, GPIO_OUT_ONE);
}

static void ldo_p5m5_dsv_5v5_off(void)
{
    mt_set_gpio_mode(GPIO_DW8768_ENP, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_DW8768_ENP, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DW8768_ENP, GPIO_OUT_ZERO);
    mt_set_gpio_mode(GPIO_DW8768_ENN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_DW8768_ENN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DW8768_ENN, GPIO_OUT_ZERO);
}

static void touch_reset_pin (int mode)
{
    mt_set_gpio_mode(GPIO_TOUCH_RESET, GPIO_TOUCH_RESET_M_GPIO);
    mt_set_gpio_dir(GPIO_TOUCH_RESET, GPIO_DIR_OUT);

    if(mode == 1)
    {
        mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ONE);
    }
    else if(mode == 0)
    {
        mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ZERO);
    }
}

static void lcm_init(void)
{
#if defined(BUILD_LK)
    ldo_p5m5_dsv_5v5_off();
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
    touch_reset_pin(0);
    MDELAY(2);
#endif

    /* TP_VCI 3.0v on */
    ldo_3v0_on();

    MDELAY(10);

    ldo_1v8io_on();
    MDELAY(1);
    touch_reset_pin(1);

    MDELAY(10);

    ldo_p5m5_dsv_5v5_on();

    MDELAY(20);

    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    MDELAY(20);
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
    LCM_PRINT("[LCD] lcm_init reset low \n");
    MDELAY(2);

    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    LCM_PRINT("[LCD] lcm_init reset high \n");
    MDELAY(20);

    init_lcm_registers();	//SET EXTC ~ sleep out register

    LCM_PRINT("[LCD] lcm_init \n");
}

static void lcm_suspend(void)
{
    push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);

#ifdef LGE_LPWG_SUPPORT
    /* DSV No control */
#else
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
    MDELAY(20);

    ldo_p5m5_dsv_5v5_off();
    MDELAY(10);

    ldo_1v8io_off();
#endif

    LCM_PRINT("[LCD] lcm_suspend \n");
}

static void lcm_resume(void)
{
    #ifdef LGE_LPWG_SUPPORT
    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
    LCM_PRINT("[LCD] lcm_resume reset low \n");
    MDELAY(2);

    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    LCM_PRINT("[LCD] lcm_resume reset high \n");
    MDELAY(20);

    init_lcm_registers();	//SET EXTC ~ sleep out register
#else
    lcm_init();
#endif
    LCM_PRINT("[LCD] lcm_resume \n");
}

/* ------------------------------------------------------------------------
 *  Get LCM Driver Hooks
 * ------------------------------------------------------------------------*/
LCM_DRIVER r69338_hd720_dsi_vdo_jdi_dw8755a_drv=
{
    .name               = "r69338_hd720_dsi_vdo_jdi_dw8755a_drv",
    .set_util_funcs     = lcm_set_util_funcs,
    .get_params         = lcm_get_params,
    .init               = lcm_init,/*tianma init fun.*/
    .suspend            = lcm_suspend,
    .resume             = lcm_resume,
};
