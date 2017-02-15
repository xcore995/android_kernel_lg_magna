#ifndef __ASM_ARCH_MTK_BOARD_LGE_H
#define __ASM_ARCH_MTK_BOARD_LGE_H


typedef enum {
    HW_REV_EVB1 = 0,
    HW_REV_EVB2,
    HW_REV_A,
    HW_REV_B,
    HW_REV_C,
    HW_REV_D,
    HW_REV_E,
    HW_REV_F,
    HW_REV_G,
    HW_REV_H,
    HW_REV_I,
    HW_REV_1_0,
    HW_REV_1_1,
    HW_REV_1_2,
    HW_REV_1_3,
    HW_REV_MAX
} hw_rev_type;

typedef enum {
    LT_CABLE_56K = 6,
    LT_CABLE_130K,
    USB_CABLE_400MA,
    USB_CABLE_DTC_500MA,
    ABNORMAL_USB_CABLE_400MA,
    LT_CABLE_910K,
    NO_INIT_CABLE,
} usb_cable_type;

hw_rev_type lge_get_board_revno(void);
usb_cable_type lge_get_board_cable(void);
/* LGE_CHANGE_S: LGP-S18 changseung.yu@lge.com [2014-3-18] : Add pcb revision voltage. */
#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
int lge_pre_self_diagnosis(char *drv_bus_code, int func_code, char *dev_code, char *drv_code, int errno);
int lge_pre_self_diagnosis_pass(char *dev_code);
#endif
#define AUXADC_PCB_REV_CHANNEL	1
extern int get_pcb_revision_voltage(void);
/* LGE_CHANGE_E */

#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
int lge_pre_self_diagnosis(char *drv_bus_code, int func_code, char *dev_code, char *drv_code, int errno);

struct pre_selfd_platform_data {
    int (*set_values) (int r, int g, int b);
    int (*get_values) (int *r, int *g, int *b);
};
#endif

enum lge_laf_mode_type {
    LGE_LAF_MODE_NORMAL = 0,
    LGE_LAF_MODE_LAF,
};

enum lge_laf_mode_type lge_get_laf_mode(void);

/* LGE_CHANGE_S: LGP-S18 inmok.kim@lge.com [2014-2-25] : detect cable type */
#ifdef CONFIG_LGE_CABLE_ID_DETECT
typedef struct {
    int adc_min;
    int adc_max;
    int cable_type;
} usb_cable_adc_type;

typedef enum {
    _56K    = 0,
    _130K   = 1,
    _910K   = 2,
    _USER   = 3,
    CABLE_MAX,
} usb_Cable_enum_type;

extern int IMM_auxadc_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
int get_usb_type(void);

#if defined(CONFIG_MACH_MT6752)
#define AUXADC_USB_ID_CHANNEL 12
#elif defined(CONFIG_MACH_MT6582) || defined (CONFIG_MACH_MT6592)
#define AUXADC_USB_ID_CHANNEL 0
#else
#error USB_ID ADC port is not defined!
#endif

/* LGE_CHANGE_S: LGP-S18 inmok.kim@lge.com [2014-3-12] : ATcommand charging complete */
extern int g_AtCmdChargeMode;//for at%charge
extern int g_AtCmdUsbId;     //for at%usbiadc
extern int g_AtCmdUsbAdc;    //for at%usbiadc
/* LGE_CHANGE_E */

#endif
/* LGE_CHANGE_E */
#if defined(CONFIG_LGE_HANDLE_PANIC) || defined(CONFIG_LGE_HIDDEN_RESET)
// RESERVED_ADDR_END    : 0x7FEF_FFFF
// HIDDEN_RESET         : 0x7FE3_F400
// CTX                  : 0x7FE3_F000
// CRASH_LOG            : 0x7FE3_E000
// RAM_CONSOLE          : 0x7FE0_0000
// RESERVED_MEM         : 0x7FE0_0000
#if defined(CONFIG_MACH_MT6752)
#define LGE_BSP_MEM_BASE_ADDR           0x40000000U
#else
#define LGE_BSP_MEM_BASE_ADDR			0x80000000
#endif

#if defined (TARGET_MT6582_Y50) /* ONE_BIN_MEMORY */
extern unsigned long LGE_BSP_MEM_MAX_PHY_ADDR;
#else
#define LGE_BSP_MEM_MAX_PHY_ADDR        (LGE_BSP_MEM_BASE_ADDR + (1024 * SZ_1M)) // 0x4000_0000
#endif

#define LGE_BSP_PRELOADER_MEM_SIZE      (1 * SZ_1M) // Preloader 1M (0x7FC0_0000 - 0x7FE0_0000)
#define LGE_BSP_RESERVED_MEM_SIZE       (1 * SZ_1M) // RESERVED 1M  (0x7FE0_0000 - 0x7FF0_0000)
#define LGE_BSP_LK_MEM_SIZE             (1 * SZ_1M) // LK 1M        (0x7FF0_0000 - 0x8000_0000)

#define LGE_BSP_RESERVED_MEM_PHY_ADDR   (LGE_BSP_MEM_MAX_PHY_ADDR - LGE_BSP_LK_MEM_SIZE - LGE_BSP_RESERVED_MEM_SIZE) // 0x7FE0_0000
#define LGE_BSP_RAM_CONSOLE_PHY_ADDR    (LGE_BSP_RESERVED_MEM_PHY_ADDR) // 0x7FE0_0000
#define LGE_BSP_RAM_CONSOLE_SIZE        (124 * 2 * SZ_1K) // 0x003_E000

#define LGE_BSP_CRASH_LOG_PHY_ADDR      (LGE_BSP_RAM_CONSOLE_PHY_ADDR + LGE_BSP_RAM_CONSOLE_SIZE) // 0x7FE3_E000
#define LGE_BSP_CRASH_LOG_SIZE          (4 * SZ_1K) // 0x0000_1000

#define LGE_CRASH_CTX_BUF_PHY_ADDR      (LGE_BSP_CRASH_LOG_PHY_ADDR + LGE_BSP_CRASH_LOG_SIZE) //0x7FE3_F000
#define LGE_CRASH_CTX_BUF_SIZE          (1 * SZ_1K) //0x0000_0400

#define LGE_BSP_HIDDEN_RESET_PHY_ADDR   (LGE_CRASH_CTX_BUF_PHY_ADDR + LGE_CRASH_CTX_BUF_SIZE) // 0x7FE4_F400
#define LGE_BSP_HIDDEN_RESET_SIZE       (LGE_BSP_RESERVED_MEM_SIZE - LGE_BSP_RAM_CONSOLE_SIZE - LGE_BSP_CRASH_LOG_SIZE - LGE_CRASH_CTX_BUF_SIZE)

// these are the standard values. used in lge_save_boot_reason(), lge_get_boot_reason()
// use these values if need in other files
#define LGE_BOOT_REASON_MAGIC_CODE          0x1234ABCD
// boot reason value
// prefix, postfix 1 bytes (0xff) so, 0xFFxxxxFF
#define LGE_BOOT_KERNEL_CRASH               0xFF0001FF
#define LGE_BOOT_HIDDEN_RESET_REBOOT        0xFF0002FF
#define LGE_BOOT_BNR_RECOVERY_MODE_REBOOT   0x77665555
/* LGE_CHANGE_S: [2012-11-16] duvallee.lee@lge.com : add magic number for normal power-off */
#define LGE_BOOT_NORMAL_POWER_OFF           0xFF0003FF
/* LGE_CHANGE_E: [2012-11-16] duvallee.lee@lge.com : add magic number for normal power-off */
/* LGE_CHANGE_S: [2014-04-08] deokhwan.jung@lge.com  : add magic number for dload change  */
#define LGE_BOOT_DLOAD_REBOOT               0x6C616664
#define LGE_BOOT_LAF_RESTART_REBOOT         0x6F656D52
/* LGE_CHANGE_E: [2014-04-08] deokhwan.jung@lge.com  : add magic number for dload change  */
#define LGE_BOOT_INIT_BOOT_REASON           0x00000000
//[LGE_UPDATE_S] DMS_SYSTEM dms-fota@lge.com
#define LGE_BOOT_FOTA_REBOOT                0x77665566
//[LGE_UPDATE_E] DMS_SYSTEM dms-fota@lge.com
#define LGE_BOOT_DMVERITY_CORRUPTED         0x77665567
#define LGE_BOOT_WALLPAPER_CORRUPTED        0x77665568

int lge_save_boot_reason(unsigned long reason, unsigned long extra1, unsigned long extra2);
unsigned long lge_get_boot_reason(unsigned long *pExtra1, unsigned long * pExtra2);
#endif

// LGE_CHANGE_S : 2013-03-25 branden.you@lge.com
#define FACTORY_PID 0x6000
#define LGE_FACTORY_CABLE_TYPE 1
#define MAX_IMEI_LEN 19
#define LGE_PIF_CABLE 2
#define LGE_130K_CABLE 3

#if 1 //defined ( LGE_BSP_LGBM )
typedef enum LGBmCableIdTag
{
	USB_CABLE_ID_NONE  = 0,
	USB_CABLE_ID_OPEN,
	USB_CABLE_ID_56K,
	USB_CABLE_ID_130K,
	USB_CABLE_ID_180K,
	USB_CABLE_ID_910K,
	USB_CABLE_ID_UNKNOWN,

	USB_CABLE_ID_MAX
}
LGBmCableId;
#else
typedef enum {
	DEVICE_NONE,   // 0
	DEVICE_OPEN_CABLE,   // 1
	DEVICE_FACTORY_UART_CABLE,   // 130K
	DEVICE_FACTORY_USB_CABLE,  // 56K
	DEVICE_FACTORY_DOWNLOAD_CABLE,  // 910K
	DEVIDE_MAX
} USB_ID_TYPE;

extern USB_ID_TYPE readUSB_ID_Value();
#endif

#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
#define MAC_OS_TYPE	0x02
#define WIN_LINUX_TYPE	0xFF
#endif

//int android_set_factory_mode(struct usb_composite_dev *cdev);
//void android_factory_desc(int enable, char usb_desc, struct usb_composite_dev *cdev);
// LGE_CHANGE_E : 2013-03-25 branden.you@lge.com

#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
void lgeusb_set_host_os(u16);
bool lgeusb_get_host_os(void);
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID
typedef enum {
	BATT_ID_UNKNOWN = 0,
	BATT_ID_DS2704_N,
	BATT_ID_DS2704_L,
	BATT_ID_DS2704_C,
	BATT_ID_ISL6296_N,
	BATT_ID_ISL6296_L,
	BATT_ID_ISL6296_C,
	BATT_ID_RA4301_VC0,
	BATT_ID_RA4301_VC1,
	BATT_ID_RA4301_VC2,
	BATT_ID_SW3800_VC0,
	BATT_ID_SW3800_VC1,
	BATT_ID_SW3800_VC2,
	BATT_ID_MAX,
} BATT_ID;

BATT_ID lge_get_battery_id(void);
char* lge_get_battery_id_str(void);
#endif

enum lge_boot_mode_type {
	LGE_BOOT_MODE_NORMAL = 0,
	LGE_BOOT_MODE_CHARGER,
	LGE_BOOT_MODE_CHARGERLOGO,
	LGE_BOOT_MODE_QEM_56K,
	LGE_BOOT_MODE_QEM_130K,
	LGE_BOOT_MODE_QEM_910K,
	LGE_BOOT_MODE_PIF_56K,
	LGE_BOOT_MODE_PIF_130K,
	LGE_BOOT_MODE_PIF_910K,
	LGE_BOOT_MODE_MINIOS    /* LGE_UPDATE for MINIOS2.0 */
};
#if defined(CONFIG_LGE_KSWITCH)
int lge_get_kswitch_status(void);
#endif

enum lge_boot_mode_type lge_get_boot_mode(void);
int lge_get_factory_boot(void);
#endif /* __ASM_ARCH_MTK_BOARD_LGE_H */
