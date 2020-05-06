#include <linux/kernel.h>
#include <asm/setup.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/module.h>
#include <mach/board_lge.h>

#include "mt_auxadc_sw.h"

static hw_rev_type lge_bd_rev;
EXPORT_SYMBOL(lge_bd_rev);

#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
static bool is_mac_os;
#endif

/* CAUTION: These strings are come from LK. */
char *rev_str[] = {
	"evb1",
	"evb2",
	"rev_a",
	"rev_b",
	"rev_c",
	"rev_d",
	"rev_e",
	"rev_f",
	"rev_g",
	"rev_h",
	"rev_i",
	"rev_10",
	"rev_11",
	"rev_12",
	"rev_13",
	"reserved",
};

static int __init board_revno_setup(char *rev_info)
{
	int i;
	lge_bd_rev = HW_REV_EVB1;
	for (i = 0; i < HW_REV_MAX; i++) {
		if (!strncmp(rev_info, rev_str[i], (strlen(rev_info) > strlen(rev_str[i])) ? strlen(rev_info) : strlen(rev_str[i]))) {
			lge_bd_rev = (hw_rev_type) i;
			/* it is defined externally in <asm/system_info.h> */
			// system_rev = lge_bd_rev;
			break;
		}
	}

	printk(KERN_ALERT "unified LK bootcmd lge.rev setup: %s\n", rev_str[lge_bd_rev]);
	return 1;
}
__setup("lge.rev=", board_revno_setup);

hw_rev_type lge_get_board_revno(void)
{
	return lge_bd_rev;
}


static usb_cable_type lge_cable;
EXPORT_SYMBOL(lge_cable);

/* CAUTION: These strings are come from LK. */
char *cable_str[] = {
	" "," "," "," ", " ", " ",
	"LT_56K",
	"LT_130K",
	"400MA",
	"DTC_500MA",
	"Abnormal_400MA",
	"LT_910K",
	"NO_INIT",
};

static int __init board_cable_setup(char *cable_info)
{
	int i;
	lge_cable = NO_INIT_CABLE;
	for (i = LT_CABLE_56K; i < NO_INIT_CABLE; i++) {
		if (!strncmp(cable_info, cable_str[i], (strlen(cable_info) > strlen(cable_str[i])) ? strlen(cable_info) : strlen(cable_str[i]))) {
			lge_cable = (usb_cable_type) i;
			/* it is defined externally in <asm/system_info.h> */
			// system_rev = lge_bd_rev;
			break;
		}
	}

	printk(KERN_ALERT "unified LK bootcmd lge.cable: %s\n", cable_str[lge_cable]);
	return 1;
}
__setup("lge.cable=", board_cable_setup);

usb_cable_type lge_get_board_cable(void)
{
	return lge_cable;
}

static enum lge_laf_mode_type lge_laf_mode = LGE_LAF_MODE_NORMAL;

int __init lge_laf_mode_init(char *s)
{
    if (strcmp(s, "") && strcmp(s, "MID"))
        lge_laf_mode = LGE_LAF_MODE_LAF;

    return 1;
}
__setup("androidboot.laf=", lge_laf_mode_init);

enum lge_laf_mode_type lge_get_laf_mode(void)
{
    return lge_laf_mode;
}

#ifdef CONFIG_LGE_CABLE_ID_DETECT
#if defined (TARGET_MT6582_Y70) || defined (TARGET_MT6582_Y90)
usb_cable_adc_type cable_100K[CABLE_MAX]    =   {
/*                  adc_min     adc_max     cable_type  */
/* _56K     */  {   55,         65,         LT_CABLE_56K},
/* _130K    */  {   86,         96,         LT_CABLE_130K},
/* _910K    */  {   131,        141,        LT_CABLE_910K},
/* _USER    */  {   143,        300,        USB_CABLE_400MA},
};
#elif defined(TARGET_MT6732_C90)
/* Comment: ADC value for C90 */
usb_cable_adc_type cable_100K[CABLE_MAX]    =	{
/*                  adc_min     adc_max     cable_type  */
/* _56K     */  {   500,         650,         LT_CABLE_56K},
/* _130K    */  {   800,         930,         LT_CABLE_130K},
/* _910K    */  {   1200,        1320,        LT_CABLE_910K},
/* _USER    */  {   1340,        1450,        USB_CABLE_400MA},
};
#else
usb_cable_adc_type cable_100K[CABLE_MAX]    =   {
/*                  adc_min     adc_max     cable_type  */
/* _56K     */  {   55,         65,         LT_CABLE_56K},
/* _130K    */  {   86,         106,        LT_CABLE_130K},
/* _910K    */  {   131,        141,        LT_CABLE_910K},
/* _USER    */  {   143,        300,        USB_CABLE_400MA},
};
#endif

int  g_usb_cable_type  = NO_INIT_CABLE;

int get_usb_type(void)
{
    int retry = 3;
    int data[4] = {0, 0, 0, 0};
    int rawvalue    = 0;
    int ret         = 0;
    int cable_enum  = 0;
    int adc_voltage   = 0;
    int usb_cable_type_num  = NO_INIT_CABLE;

    while (IMM_auxadc_GetOneChannelValue(AUXADC_USB_ID_CHANNEL, data, &rawvalue) != 0) {
        printk("[Kernel] Failed to read ADC Channel 0 [USB Cable]\n");
        retry--;
        if (!retry) {
            return usb_cable_type_num;
        }
    }
#if defined (TARGET_MT6732_C90)
    adc_voltage = g_AtCmdUsbAdc = data[0] * 1000 + data[1];	//g_AtCmdUsbAdc for at%usbiadc
    printk("[Kernel] USB_Cable ADC : %d\n", adc_voltage);
    printk("[Kernel] USB_Cable ADC RAW data : %d, ADC value : %d.%d\n", rawvalue, data[0],data[1]%1000);
#else
    adc_voltage = g_AtCmdUsbAdc = data[0] * 100 + data[1];	//g_AtCmdUsbAdc for at%usbiadc
    printk("[Kernel] USB_Cable ADC : %d\n", adc_voltage);
    printk("[Kernel] USB_Cable ADC RAW data : %d, ADC value : %d.%d%d\n", rawvalue, data[0],data[1]/10, data[1]%10);
#endif
    for (cable_enum = 0; cable_enum < CABLE_MAX; cable_enum++)
    {
        if ((cable_100K[cable_enum].adc_min <= adc_voltage) && (adc_voltage <= cable_100K[cable_enum].adc_max))
            break;
    }
    if (cable_enum == CABLE_MAX)
        usb_cable_type_num  = ABNORMAL_USB_CABLE_400MA;
    else
        usb_cable_type_num  = cable_100K[cable_enum].cable_type;

    return usb_cable_type_num;
}
#endif

/*  LGE_CHANGE : LGP-S18 add LCD LUT for Kcal
 *  [2014-02-28] kyeongdon.kim@lge.com  */
#if defined(CONFIG_LGE_LUT_KCAL)
int g_kcal_r = 255;
int g_kcal_g = 255;
int g_kcal_b = 255;

EXPORT_SYMBOL(g_kcal_r);
EXPORT_SYMBOL(g_kcal_g);
EXPORT_SYMBOL(g_kcal_b);

static int __init lcd_kcal(char *lcd_kcal)
{
        char valid_k = 0;

        sscanf(lcd_kcal, "%d|%d|%d|%c", &g_kcal_r, &g_kcal_g, &g_kcal_b, &valid_k);
        printk("lcd_kcal is %d|%d|%d|%c\n", g_kcal_r, g_kcal_g, g_kcal_b, valid_k);

        if(valid_k != 'K')
        {
            printk("kcal not calibrated yet : %d\n", valid_k);
            g_kcal_r = g_kcal_g = g_kcal_b = 255;
            printk("set to default : %d\n", g_kcal_r);
        }
        return 0;
}
__setup("lge.kcal=", lcd_kcal);
#endif

unsigned int touch_module;
EXPORT_SYMBOL(touch_module);

static int __init touch_module_check(char *touch)
{
	if(!strcmp(touch,"PRIMARY_MODULE")){
		touch_module = 0;
		printk("touch_module 0\n");
	}else if(!strcmp(touch,"SECONDARY_MODULE")){
		touch_module = 1;
		printk("touch_module 1\n");
	}else if(!strcmp(touch,"TERITARY_MODULE")){
		touch_module = 2;
	}else if(!strcmp(touch,"QUATENARY_MODULE")){
		touch_module = 3;
	}

	printk("kernel touch module check : %s\n", touch);
	return 0;
}
early_param("lge.touchModule", touch_module_check);

// LGE_CHANGE_S : 2014-01-15 jbpaul.lee@lge.com add QEM
unsigned char g_qem_check;
EXPORT_SYMBOL(g_qem_check);

static int __init qem_check(char *qem)
{
	g_qem_check = *qem;
	printk("kernel qem check : %c\n", g_qem_check);
	return 0;
}
early_param("qem", qem_check);
// LGE_CHANGE_E : 2014-01-15 jbpaul.lee@lge.com add QEM
// LGE_CHANGE_S : 2014-02-06 yunhang.heo@lge.com USB_ID
#if 1 // USB_ID_Detection

LGBmCableId g_lgbmBootUsbCableId = 0;
EXPORT_SYMBOL(g_lgbmBootUsbCableId);

static int __init usb_id(char *usbid)
{
	sscanf(usbid, "%u", &g_lgbmBootUsbCableId);
	printk("kernel g_lgbmBootUsbCableId : %d\n", g_lgbmBootUsbCableId);
	return 0;
}
early_param("usbid", usb_id);
#endif
// LGE_CHANGE_E : 2014-02-06 yunhang.heo@lge.com USB_ID

#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
void lgeusb_set_host_os(u16 w_length)
{
        switch (w_length) {
        case MAC_OS_TYPE:
                is_mac_os = true;
                break;
        case WIN_LINUX_TYPE:
                is_mac_os = false;
                break;
        default:
                break;
        }
}

bool lgeusb_get_host_os(void)
{
        return is_mac_os;
}
#endif

int get_pcb_revision_voltage(void)
{
    int data[4] = {0, 0, 0, 0};
    int rawvalue    = 0;
    int ret         = 0;
    int adc_voltage   = 0;

    ret = IMM_auxadc_GetOneChannelValue(AUXADC_PCB_REV_CHANNEL, data, &rawvalue);
    if (ret == 0)
    {
        adc_voltage = data[0] * 100 + data[1];
        printk("[Kernel] PCB_Revision ADC : %d\n", adc_voltage);
        printk("[Kernel] PCB_Revision ADC RAW data : %d, ADC value : %d.%d%d\n", rawvalue, data[0],data[1]/10, data[1]%10);
    }
    else
    {
        printk("[Kernel] Failed to read ADC Channel %d [PCB_REV]\n", AUXADC_PCB_REV_CHANNEL);
        return -1;
    }
    return adc_voltage;
}
EXPORT_SYMBOL(get_pcb_revision_voltage);

#ifdef CONFIG_LGE_PM_BATTERY_ID
char *battery_id_str[] = {
	"Unknown",
	"DS2704_N",
	"DS2704_L",
	"DS2704_C",
	"ISL6296_N",
	"ISL6296_L",
	"ISL6296_C",
	"RA4301_VC0",
	"RA4301_VC1",
	"RA4301_VC1",
	"SW3800_VC0",
	"SW3800_VC1",
	"SW3800_VC2",
};
BATT_ID battery_id = BATT_ID_UNKNOWN;

BATT_ID lge_get_battery_id(void)
{
	return battery_id;
}
EXPORT_SYMBOL(lge_get_battery_id);

char* lge_get_battery_id_str(void)
{
	if (battery_id < BATT_ID_MAX)
		return battery_id_str[battery_id];
	return battery_id_str[BATT_ID_UNKNOWN];
}
EXPORT_SYMBOL(lge_get_battery_id_str);

static int __init batt_info(char *batt_id)
{
	int i;

	for (i = 0; i < BATT_ID_MAX; i++) {
		if (strcmp(batt_id, battery_id_str[i]) == 0)
			break;
	}
	if (i < BATT_ID_MAX) {
		battery_id = i;
	}

	return 0;
}
early_param("lge.battid", batt_info);
#endif

/* get boot mode information from cmdline.
 * If any boot mode is not specified,
 * boot mode is normal type.
 */
static enum lge_boot_mode_type lge_boot_mode = LGE_BOOT_MODE_NORMAL;
static int __init lge_boot_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "chargerlogo"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
	else if (!strcmp(s, "qem_56k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_56K;
	else if (!strcmp(s, "qem_130k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_130K;
	else if (!strcmp(s, "qem_910k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_910K;
	else if (!strcmp(s, "pif_56k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_56K;
	else if (!strcmp(s, "pif_130k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_130K;
	else if (!strcmp(s, "pif_910k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_910K;
	/* LGE_UPDATE_S for MINIOS2.0 */
	else if (!strcmp(s, "miniOS"))
		lge_boot_mode = LGE_BOOT_MODE_MINIOS;
	pr_info("ANDROID BOOT MODE : %d %s\n", lge_boot_mode, s);
	/* LGE_UPDATE_E for MINIOS2.0 */

	return 1;
}
__setup("androidboot.mode=", lge_boot_mode_init);

enum lge_boot_mode_type lge_get_boot_mode(void)
{
	return lge_boot_mode;
}

int lge_get_factory_boot(void)
{
    int res;

    switch (lge_boot_mode) {
    case LGE_BOOT_MODE_QEM_56K:
    case LGE_BOOT_MODE_QEM_130K:
    case LGE_BOOT_MODE_QEM_910K:
    case LGE_BOOT_MODE_PIF_56K:
    case LGE_BOOT_MODE_PIF_130K:
    case LGE_BOOT_MODE_PIF_910K:
    case LGE_BOOT_MODE_MINIOS:
        res = 1;
        break;
    default :
        res = 0;
        break;
    }
    return res;
}

#if defined(CONFIG_LGE_KSWITCH)
static int kswitch_status;
static int atoi(const char *name)
{
	int val = 0;

	for (;; name++) {
		switch (*name) {
		case '0' ... '9':
			val = 10*val+(*name-'0');
			break;
		default:
			return val;
		}
	}
}

static int __init kswitch_setup(char *value)
{
	kswitch_status = atoi(value);

	if (kswitch_status < 0)
		kswitch_status = 0;

	printk(KERN_INFO "[KSwitch] %d \n", kswitch_status);
	return 1;
}
__setup("kswitch=", kswitch_setup);

int lge_get_kswitch_status(void)
{
    return kswitch_status;
}
#endif

