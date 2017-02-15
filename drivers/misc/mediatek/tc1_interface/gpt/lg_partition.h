#ifndef __LG_PARTITION_H
#define __LG_PARTITION_H
/* DATA layout*/
#if 1	//Re-arrange of Factory Data Layout - joohyung.lee
#define LGE_FAC_DATA_VERSION_OFFSET	(2)
#define LGE_FAC_PID_OFFSET	(4)
#define LGE_FAC_BT_ADDR_OFFSET (6)
#define LGE_FAC_IMEI_MASTER_OFFSET	(8)
#define LGE_FAC_IMEI_NOT_MASTER_OFFSET (9)
#define LGE_FAC_SOFTWARE_VERSION_OFFSET (44)
#define LGE_FAC_IMEI_TRIPLE_OFFSET      (10)
#define LGE_FAC_IMEI_QUADRUPLE_OFFSET   (11)
#define LGE_FAC_NETWORK_CODE_LIST_NUM_OFFSET	(14)
#define LGE_FAC_SIM_LOCK_TYPE_OFFSET	(16)
#define LGE_FAC_FUSG_FLAG_OFFSET	(18)
#define LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_OFFSET	(26)
#define LGE_FAC_WIFI_MAC_ADDR_OFFSET	(28)
#define LGE_FAC_UNLOCK_CODE_OFFSET	(30)
#define LGE_FAC_NETWORK_CODE_OFFSET	(46)

#define LGE_FAC_IMEI_ENDMARK    (0xFFFFFFFF)

#define LGE_FAC_IMEI_0_OFFSET	LGE_FAC_IMEI_MASTER_OFFSET
#define LGE_FAC_IMEI_1_OFFSET 	LGE_FAC_IMEI_NOT_MASTER_OFFSET
#define LGE_FAC_IMEI_2_OFFSET	LGE_FAC_IMEI_TRIPLE_OFFSET
#define LGE_FAC_IMEI_3_OFFSET	LGE_FAC_IMEI_QUADRUPLE_OFFSET

// Below items are not used or not needed
//#define LGE_FAC_UNLOCK_FAIL_COUNT_OFFSET	(??)
//#define LGE_FAC_VERIFY_UNLOCK_CODE_OFFSET	(??)
//#define LGE_FAC_UNLOCK_CODE_VALIDNESS_OFFSET	(??)
//#define LGE_FAC_NETWORK_CODE_VALIDNESS_OFFSET	(??)
//#define LGE_FAC_INIT_SIM_LOCK_DATA_OFFSET	(??)

#else	//Re-arrange of Factory Data Layout - joohyung.lee
#define LGE_FAC_WIFI_MAC_ADDR_OFFSET	(1)
#define LGE_FAC_BT_ADDR_OFFSET (2)
#define LGE_FAC_IMEI_MASTER_OFFSET	(3)
#define LGE_FAC_IMEI_NOT_MASTER_OFFSET (4)
#define LGE_FAC_SIM_LOCK_TYPE_OFFSET	(5)
#define LGE_FAC_NETWORK_CODE_LIST_NUM_OFFSET	(6)
#define LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_OFFSET	(7)
#define LGE_FAC_UNLOCK_FAIL_COUNT_OFFSET	(8)
#define LGE_FAC_UNLOCK_CODE_OFFSET	(9)
#define LGE_FAC_VERIFY_UNLOCK_CODE_OFFSET	(10)
#define LGE_FAC_UNLOCK_CODE_VALIDNESS_OFFSET	(11)
#define LGE_FAC_NETWORK_CODE_OFFSET	(12)
#define LGE_FAC_NETWORK_CODE_VALIDNESS_OFFSET	(13)
#define LGE_FAC_INIT_SIM_LOCK_DATA_OFFSET	(14)
#define LGE_FAC_FUSG_FLAG_OFFSET	(15)
#define LGE_FAC_DATA_VERSION_OFFSET	(16)
#define LGE_FAC_PID_OFFSET	(17)
#define LGE_FAC_SOFTWARE_VERSION_OFFSET (18)
#define LGE_FAC_IMEI_TRIPLE_OFFSET      (19)
#define LGE_FAC_IMEI_QUADRUPLE_OFFSET   (20)

#define LGE_FAC_IMEI_ENDMARK    (0xFFFFFFFF)

#define LGE_FAC_IMEI_0_OFFSET	LGE_FAC_IMEI_MASTER_OFFSET
#define LGE_FAC_IMEI_1_OFFSET 	LGE_FAC_IMEI_NOT_MASTER_OFFSET
#define LGE_FAC_IMEI_2_OFFSET	LGE_FAC_IMEI_TRIPLE_OFFSET
#define LGE_FAC_IMEI_3_OFFSET	LGE_FAC_IMEI_QUADRUPLE_OFFSET
#endif	//Re-arrange of Factory Data Layout - joohyung.lee

/*data length*/
#define LGE_FAC_PID_PART_1_LEN 22
#define LGE_FAC_PID_PART_2_LEN 10
#define LGE_FAC_PID_LEN ( LGE_FAC_PID_PART_1_LEN + LGE_FAC_PID_PART_2_LEN ) /* decimal(22) + ASCII(10) */
#define LGE_FAC_DATA_VERSION_LEN 4
#define LGE_FAC_FUSG_FLAG	1
#define LGE_FAC_UNLOCK_FAIL_COUNT_LEN	1
#define LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_LEN	1
#define LGE_FAC_VERIFY_UNLOCK_CODE_LEN	1
#define LGE_FAC_NETWORK_CODE_LIST_NUM_LEN	2
#define LGE_FAC_SIM_LOCK_TYPE_LEN	1
#define LGE_FAC_BT_ADDR_LEN (0x6)   /* hexadecimal */
#define LGE_FAC_IMEI_LEN (15)   /* decimal */
#define LGE_FAC_WIFI_MAC_ADDR_LEN (6)
#define LGE_FAC_SUFFIX_STR_LEN (15)
#define LGE_FAC_NC_MCC_LEN 3
#define LGE_FAC_NC_MNC_LEN 3
#define LGE_FAC_NC_GID1_LEN 8
#define LGE_FAC_NC_GID2_LEN 8
#define LGE_FAC_NC_SUBSET_LEN 2
#define LGE_FAC_NETWORK_CODE_LEN ( LGE_FAC_NC_MCC_LEN + LGE_FAC_NC_MNC_LEN + LGE_FAC_NC_GID1_LEN + LGE_FAC_NC_GID2_LEN + LGE_FAC_NC_SUBSET_LEN )
#define LGE_FAC_SV_LEN	60
#define LGE_FAC_FUSG_FLAG_LEN 1
#define LGE_FAC_MAX_NETWORK_CODE_LIST_NUM (40)  /* This number may be increased in the future */
#define LGE_FAC_UNLOCK_CODE_LEN (16)
#define LGE_FAC_SLTYPE_VALID_MASK 0x1F
#define LGE_FAC_SLTYPE_MASK_NETWORK 0x01
#define LGE_FAC_SLTYPE_MASK_SERVICE_PROVIDER 0x02
#define LGE_FAC_SLTYPE_MASK_NETWORK_SUBSET 0x04
#define LGE_FAC_SLTYPE_MASK_COOPERATE 0x08
#define LGE_FAC_SLTYPE_MASK_LOCK_TO_SIM 0x10
#define LGE_FAC_SLTYPE_MASK_HARDLOCK 0x20
#define LGE_FAC_SLTYPE_MASK_RESERVED_1 0x40 /* T.B.D */
#define LGE_FAC_SLTYPE_MASK_RESERVED_2 0x80 /* T.B.D */
#define LGE_FAC_MAX_UNLOCK_CODE_VERIFY_FAIL_COUNT 3

#define EMMC_BLOCK_SIZE	512

#ifndef bool
#define bool 		unsigned char
#define true 1
#define false 0
#endif

#if 1  /* LGE_BSP_COMMON LGE_CHANGE_S 140306 jongwoo82.lee : Accelerometer Calibration Value */
#define LGE_FAC_ACCELEROMETER_CALIBRATION_IDX       52
#define LGE_FAC_ACCELEROMETER_CALIBRATION_SIZE sizeof(unsigned int)*6 /*3*/ //motion sensor cal backup
#endif  /* LGE_BSP_COMMON LGE_CHANGE_E 140306 jongwoo82.lee : Accelerometer Calibration Value */

#if 1  /* LGE_BSP_COMMON LGE_CHANGE_S 140306 jongwoo82.lee : Accelerometer Calibration Value */
bool LGE_FacReadAccelerometerCalibration(unsigned char *accel_cal_val);
bool LGE_FacWriteAccelerometerCalibration(unsigned char *accel_cal_val);
#endif  /* LGE_BSP_COMMON LGE_CHANGE_E 140306 jongwoo82.lee : Accelerometer Calibration Value */

/*  LGE_CHANGE : LGP-S18 add LCD LUT for Kcal, this code is temporal code regarding FTM
 *  [2014-02-28] kyeongdon.kim@lge.com  */
#if defined(CONFIG_LGE_LUT_KCAL)
#define LGE_FAC_LCD_CALIBRATION_IDX             34
#define LGE_FAC_LCD_CALIBRATION_SIZE            4
extern bool LGE_FacReadLCDCalibration(unsigned char *lcd_kcal);
extern bool LGE_FacWriteLCDCalibration(unsigned char *lcd_kcal);

#endif

/* sbp_interface */
#define LGE_ONE_BINARY_HWINFO_IDX 100
#define LGE_ONE_BINARY_HWINFO_SIZE 512

#define LGE_FAC_FIXED_SOFTWARE_VER_IDX 101 /* SWFV-A Index */
#define LGE_FAC_FIXED_SOFTWARE_VER_SIZE 50 /* SWFV-A Size */

#define LGE_FAC_IMEI_SVN_IDX 102 /* SBP SVN Index */
#define LGE_FAC_IMEI_SVN_SIZE 10 /* SBP SVN Size */

#define LGE_FAC_MAX_NTCODE_COUNT_SBP 16
/* sbp_interface */

#define LGE_FAC_NETWORK_CODE_IDX01          46 /* facNetworkCode */
#define LGE_FAC_NETWORK_CODE_SIZE01         sizeof(FactoryNetworkCode)*16

#define LGE_FAC_NETWORK_CODE_IDX02          47 /* facNetworkCode */
#define LGE_FAC_NETWORK_CODE_SIZE02         sizeof(FactoryNetworkCode)*16

#define LGE_FAC_NETWORK_CODE_IDX03          48    /* facNetworkCode */
#define LGE_FAC_NETWORK_CODE_SIZE03         sizeof(FactoryNetworkCode)*8

#define LGE_FAC_NETWORK_CODE_SIZE_SBP       sizeof(FactoryNetworkCode)*LGE_FAC_MAX_NTCODE_COUNT_SBP
/* sbp_interface */

typedef struct FactoryNetworkCodeTag
{
    unsigned char Mcc[LGE_FAC_NC_MCC_LEN];  /* Ex) { 2, 4, 5 } */
    unsigned char Mnc[LGE_FAC_NC_MNC_LEN];  /* Ex) { 4, 3, 0xF } */
    unsigned char Gid1[LGE_FAC_NC_GID1_LEN];    /* Ex) { 0xB, 2, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF } */
    unsigned char Gid2[LGE_FAC_NC_GID2_LEN];    /* Ex) { 8, 0xA, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF } */
    unsigned char Subset[LGE_FAC_NC_SUBSET_LEN];    /* Ex) { 6, 2 } */
    unsigned char dummy[8];
} FactoryNetworkCode;

typedef struct FactoryUnlockCodeTag
{
    unsigned char network[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char serviceProvider[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char networkSubset[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char cooperate[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char lockToSim[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char hardlock[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char reserved_1[LGE_FAC_UNLOCK_CODE_LEN];
    unsigned char reserved_2[LGE_FAC_UNLOCK_CODE_LEN];
} FactoryUnlockCode;

bool LGE_FacWriteWifiMacAddr(unsigned char *wifiMacAddr, bool needFlashProgram);
bool LGE_FacReadWifiMacAddr(unsigned char *wifiMacAddr);
bool LGE_FacWriteBtAddr(unsigned char *btAddr, bool needFlashProgram);
bool LGE_FacReadBtAddr(unsigned char *btAddr);
bool LGE_FacWriteImei(unsigned char imei_type, unsigned char *imei, bool needFlashProgram);
bool LGE_FacReadImei(unsigned char imei_type, unsigned char *imei);
bool LGE_FacWriteSimLockType(unsigned char simLockType, bool needFlashProgram);
bool LGE_FacReadSimLockType(unsigned char *simLockType);
bool LGE_FacWriteUnlockCodeVerifyFailCount(unsigned char failCount, bool needFlashProgram);
bool LGE_FacReadUnlockCodeVerifyFailCount(unsigned char *failCount);
//bool LGE_FacWriteUnlockFailCount(unsigned char simLockType, unsigned char failCount, bool needFlashProgram);
//bool LGE_FacReadUnlockFailCount(unsigned char simLockType, unsigned char *failCount);
bool LGE_FacWriteUnlockCode(FactoryUnlockCode * unlockCode, bool needFlashProgram);
bool LGE_FacVerifyUnlockCode(unsigned char simLockType, unsigned char *unlockCode, bool * isOk);
bool LGE_FacCheckUnlockCodeValidness(bool * isValid);
bool LGE_FacWriteNetworkCode(FactoryNetworkCode * networkCode, unsigned short networkCodeListNum, bool needFlashProgram);
bool LGE_FacReadNetworkCode(FactoryNetworkCode * networkCode, unsigned short networkCodeListNum);
bool LGE_FacWriteNetworkCodeListNum(unsigned short *networkCodeListNum, bool needFlashProgram);
bool LGE_FacReadNetworkCodeListNum(unsigned short *networkCodeListNum);
bool LGE_FacCheckNetworkCodeValidness(unsigned char simLockType, bool * isValid);
bool LGE_FacInitSimLockData(void);
bool LGE_FacReadFusgFlag(unsigned char *fusgFlag);
bool LGE_FacWriteFusgFlag(unsigned char fusgFlag, bool needFlashProgram);
bool LGE_FacReadDataVersion(unsigned char *dataVersion);
bool LGE_FacWriteDataVersion(unsigned char *dataVersion, bool needFlashProgram);
bool LGE_FacReadPid(unsigned char *pid);
bool LGE_FacWritePid(unsigned char *pid, bool needFlashProgram);
void LGE_FacGetSoftwareversion(bool isOriginalVersion, unsigned char *pVersion);
/* sbp_interface */
bool LGE_FacReadNetworkCode_SBP (FactoryNetworkCode *networkCode, unsigned short  networkCodeListNum, unsigned int length);
bool LGE_FacReadOneBinaryHWInfo (unsigned char *data);
bool LGE_FacWriteOneBinaryHWInfo (unsigned char *data, bool needFlashProgram);
bool LGE_FacReadSVN_SBP (unsigned char *svn);
/* sbp_interface */
int LGE_API_test(void);

/*#define TC1_GET_NAME(fname)	"LGE_"#fname
#define TC1_FAC_NAME(fname)	LGE_##fname
#define TC1_FAC_IMEI_LEN		LGE_FAC_IMEI_LEN
#define TC1_FAC_NETWORK_CODE_LEN	LGE_FAC_NETWORK_CODE_LEN
*/
#endif
