#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>       /* printk() */
#include <linux/slab.h>         /* kmalloc() */
#include <linux/fs.h>           /* everything... filp_open */
#include <linux/errno.h>        /* error codes */
#include <linux/types.h>        /* size_t */
#include <linux/proc_fs.h>      /*proc */
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <linux/aio.h>
#include <asm/uaccess.h>        /*set_fs get_fs mm_segment_t */
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/unistd.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>
//#include <linux/autoconf.h>
#include "lg_partition.h"

#define PART_NAME "/dev/misc2"
#define PRINT	printk
int g_init_write_size = 1;

bool _LGE_GENERIC_WRITE_FUN(unsigned char *buff, unsigned int offset, unsigned int length)
{

    int ret;
    struct file *filp;
    unsigned char *tmp;
    mm_segment_t curr_fs;
    filp = filp_open(PART_NAME, O_RDWR, 0666);
    if (IS_ERR(filp))
    {
        ret = PTR_ERR(filp);
        printk("Open MISC2 partition fail! errno=%d\n", ret);
        return -1;
    }
	printk("WRITE - Open MISC2 partition OK!\n");	
    if (g_init_write_size ==0)
    {
        struct mtd_info_user info;
        if (filp->f_op->unlocked_ioctl)
        {
            filp->f_op->unlocked_ioctl(filp, MEMGETINFO, (unsigned long) &info);
        } else if (filp->f_op->compat_ioctl)
        {
            filp->f_op->compat_ioctl(filp, MEMGETINFO, (unsigned long) &info);
        }
        if (info.writesize != EMMC_BLOCK_SIZE)
        {
            printk("write size error!info.writesize=%d,EMMC_BLOCK_SIZE=%d\n", info.writesize, EMMC_BLOCK_SIZE);
            g_init_write_size = 0;
            filp_close(filp, NULL);
            return false;
        } else
        {
            g_init_write_size = 1;
        }
    }

    filp->f_op->llseek(filp, offset * EMMC_BLOCK_SIZE, SEEK_SET);
    tmp = kzalloc(EMMC_BLOCK_SIZE, GFP_KERNEL);
    if (!tmp)
    {
        printk("malloc memory fail!\n");
        filp_close(filp, NULL);
        return false;
    }
    memset(tmp, 0x0, EMMC_BLOCK_SIZE);
    curr_fs = get_fs();
    set_fs(KERNEL_DS);
    memcpy(tmp, buff, length);
    ret = filp->f_op->write(filp, tmp, EMMC_BLOCK_SIZE, &(filp->f_pos));
    if (EMMC_BLOCK_SIZE != ret)
    {
        printk("write fail!errno=%d\n", ret);
        filp_close(filp, NULL);
        kfree(tmp);
        set_fs(curr_fs);
        return false;

    }
    set_fs(curr_fs);
    kfree(tmp);
    filp_close(filp, NULL);
    return true;
}

bool _LGE_GENERIC_READ_FUN(unsigned char *buff, unsigned int offset, unsigned int length)
{

    int ret;
    struct file *filp;
    unsigned char *tmp;
    mm_segment_t curr_fs;
    filp = filp_open(PART_NAME, O_RDWR, 0666);
    if (IS_ERR(filp))
    {
        ret = PTR_ERR(filp);
        printk("Open MISC2 partition fail! errno=%d\n", ret);
        return -1;
    }
	printk("READ - Open MISC2 partition OK!\n");
    if (g_init_write_size == 0)
    {
        struct mtd_info_user info;
        if (filp->f_op->unlocked_ioctl)
        {
            filp->f_op->unlocked_ioctl(filp, MEMGETINFO, (unsigned long) &info);
        } else if (filp->f_op->compat_ioctl)
        {
            filp->f_op->compat_ioctl(filp, MEMGETINFO, (unsigned long) &info);
        }
        if (info.writesize != EMMC_BLOCK_SIZE)
        {
            printk("write size error!info.writesize=%d,EMMC_BLOCK_SIZE=%d\n", info.writesize, EMMC_BLOCK_SIZE);
            g_init_write_size = 0;
            filp_close(filp, NULL);
            return false;
        } else
        {
            g_init_write_size = 1;
        }
    }

    filp->f_op->llseek(filp, offset * EMMC_BLOCK_SIZE, SEEK_SET);
    tmp = kzalloc(EMMC_BLOCK_SIZE, GFP_KERNEL);
    if (!tmp)
    {
        printk("malloc memory fail!\n");
        filp_close(filp, NULL);
        return false;
    }
    memset(tmp, 0x0, EMMC_BLOCK_SIZE);
    curr_fs = get_fs();
    set_fs(KERNEL_DS);

    ret = filp->f_op->read(filp, tmp, EMMC_BLOCK_SIZE, &(filp->f_pos));
    if (EMMC_BLOCK_SIZE != ret)
    {
        printk("read fail!errno=%d\n", ret);
        filp_close(filp, NULL);
        kfree(tmp);
        set_fs(curr_fs);
        return false;

    }
    memcpy(buff, tmp, length);

	{
		int i=0;
			
		for(i=0;i<length;i++) {
			printk("%x ",(unsigned int)&buff[i]);
		}
	}
	printk("\n");
	
    set_fs(curr_fs);
    kfree(tmp);
    filp_close(filp, NULL);
    return true;
}
bool LGE_FacReadAccelerometerCalibration(unsigned char *accel_cal_val)
{
    return _LGE_GENERIC_READ_FUN(accel_cal_val, LGE_FAC_ACCELEROMETER_CALIBRATION_IDX, LGE_FAC_ACCELEROMETER_CALIBRATION_SIZE);
}
bool LGE_FacWriteAccelerometerCalibration(unsigned char *accel_cal_val)
{
    return _LGE_GENERIC_WRITE_FUN(accel_cal_val, LGE_FAC_ACCELEROMETER_CALIBRATION_IDX, LGE_FAC_ACCELEROMETER_CALIBRATION_SIZE);
}
bool LGE_FacWriteWifiMacAddr(unsigned char *wifiMacAddr, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(wifiMacAddr, LGE_FAC_WIFI_MAC_ADDR_OFFSET, LGE_FAC_WIFI_MAC_ADDR_LEN);
}

bool LGE_FacReadWifiMacAddr(unsigned char *wifiMacAddr)
{
    return _LGE_GENERIC_READ_FUN(wifiMacAddr, LGE_FAC_WIFI_MAC_ADDR_OFFSET, LGE_FAC_WIFI_MAC_ADDR_LEN);
}

bool LGE_FacWriteBtAddr(unsigned char *btAddr, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(btAddr, LGE_FAC_BT_ADDR_OFFSET, LGE_FAC_BT_ADDR_LEN);
}

bool LGE_FacReadBtAddr(unsigned char *btAddr)
{
    return _LGE_GENERIC_READ_FUN(btAddr, LGE_FAC_BT_ADDR_OFFSET, LGE_FAC_BT_ADDR_LEN);
}

const unsigned int imei_mapping_table[4] = { LGE_FAC_IMEI_1_OFFSET, LGE_FAC_IMEI_0_OFFSET, LGE_FAC_IMEI_2_OFFSET, LGE_FAC_IMEI_3_OFFSET};

bool LGE_FacWriteImei(unsigned char imei_type, unsigned char *imei, bool needFlashProgram)
{
    if(imei_mapping_table[imei_type]==LGE_FAC_IMEI_ENDMARK) return false;

    return _LGE_GENERIC_WRITE_FUN(imei, imei_mapping_table[imei_type], LGE_FAC_IMEI_LEN);
}

bool LGE_FacReadImei(unsigned char imei_type, unsigned char *imei)
{
    if(imei_mapping_table[imei_type]==LGE_FAC_IMEI_ENDMARK) return false;

    return _LGE_GENERIC_READ_FUN(imei, imei_mapping_table[imei_type], LGE_FAC_IMEI_LEN);
}

bool LGE_FacWriteSimLockType(unsigned char simLockType, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(&simLockType, LGE_FAC_SIM_LOCK_TYPE_OFFSET, LGE_FAC_SIM_LOCK_TYPE_LEN);
}

bool LGE_FacReadSimLockType(unsigned char *simLockType)
{
    return _LGE_GENERIC_READ_FUN(simLockType, LGE_FAC_SIM_LOCK_TYPE_OFFSET, LGE_FAC_SIM_LOCK_TYPE_LEN);
}

bool LGE_FacWriteNetworkCodeListNum(unsigned short *networkCodeListNum, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN((unsigned char *)&networkCodeListNum, LGE_FAC_NETWORK_CODE_LIST_NUM_OFFSET, LGE_FAC_NETWORK_CODE_LIST_NUM_LEN);
}

bool LGE_FacReadNetworkCodeListNum(unsigned short *networkCodeListNum)
{
    return _LGE_GENERIC_READ_FUN((unsigned char *)networkCodeListNum, LGE_FAC_NETWORK_CODE_LIST_NUM_OFFSET, LGE_FAC_NETWORK_CODE_LIST_NUM_LEN);
}

bool LGE_FacWriteUnlockCodeVerifyFailCount(unsigned char failCount, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(&failCount, LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_LEN);
}

bool LGE_FacReadUnlockCodeVerifyFailCount(unsigned char *failCount)
{
    return _LGE_GENERIC_READ_FUN(failCount, LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_CODE_VERIFY_FAIL_COUNT_LEN);
}
#if 0	//Re-arrange of Factory Data Layout - joohyung.lee
bool LGE_FacWriteUnlockFailCount(unsigned char simLockType, unsigned char failCount, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(&failCount, LGE_FAC_UNLOCK_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_FAIL_COUNT_LEN);
}

bool LGE_FacReadUnlockFailCount(unsigned char simLockType, unsigned char *failCount)
{
    return _LGE_GENERIC_READ_FUN(failCount, LGE_FAC_UNLOCK_FAIL_COUNT_OFFSET, LGE_FAC_UNLOCK_FAIL_COUNT_LEN);

}
#endif	//Re-arrange of Factory Data Layout - joohyung.lee
bool LGE_FacWriteUnlockCode(FactoryUnlockCode * unlockCode, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN((unsigned char *)unlockCode, LGE_FAC_UNLOCK_CODE_OFFSET, LGE_FAC_UNLOCK_CODE_LEN);
}

bool LGE_FacVerifyUnlockCode(unsigned char simLockType, unsigned char *unlockCode, bool * isOk)
{
    *isOk = true;
    return true;
}

bool LGE_FacCheckUnlockCodeValidness(bool * isValid)
{
    *isValid = true;
    return true;
}

bool LGE_FacWriteNetworkCode(FactoryNetworkCode * networkCode, unsigned short networkCodeListNum, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN((unsigned char *)networkCode, LGE_FAC_NETWORK_CODE_OFFSET, LGE_FAC_NETWORK_CODE_LEN);
}

bool LGE_FacReadNetworkCode(FactoryNetworkCode * networkCode, unsigned short networkCodeListNum)
{
    return _LGE_GENERIC_READ_FUN((unsigned char *)networkCode, LGE_FAC_NETWORK_CODE_OFFSET, LGE_FAC_NETWORK_CODE_LEN);

}

bool LGE_FacCheckNetworkCodeValidness(unsigned char simLockType, bool * isValid)
{
    *isValid = true;
    return true;
}

bool LGE_FacInitSimLockData(void)
{
    return true;
}

bool LGE_FacReadFusgFlag(unsigned char *fusgFlag)
{
    return _LGE_GENERIC_READ_FUN(fusgFlag, LGE_FAC_FUSG_FLAG_OFFSET, LGE_FAC_FUSG_FLAG_LEN);
}

bool LGE_FacWriteFusgFlag(unsigned char fusgFlag, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(&fusgFlag, LGE_FAC_FUSG_FLAG_OFFSET, LGE_FAC_FUSG_FLAG_LEN);
}

bool LGE_FacReadDataVersion(unsigned char *dataVersion)
{
    return _LGE_GENERIC_READ_FUN(dataVersion, LGE_FAC_DATA_VERSION_OFFSET, LGE_FAC_DATA_VERSION_LEN);
}

bool LGE_FacWriteDataVersion(unsigned char *dataVersion, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(dataVersion, LGE_FAC_DATA_VERSION_OFFSET, LGE_FAC_DATA_VERSION_LEN);
}

bool LGE_FacReadPid(unsigned char *pid)
{
    return _LGE_GENERIC_READ_FUN(pid, LGE_FAC_PID_OFFSET, LGE_FAC_PID_LEN);
}

bool LGE_FacWritePid(unsigned char *pid, bool needFlashProgram)
{
    return _LGE_GENERIC_WRITE_FUN(pid, LGE_FAC_PID_OFFSET, LGE_FAC_PID_LEN);
}

void LGE_FacGetSoftwareversion(bool isOriginalVersion, unsigned char *pVersion)
{
    int index = 0;
    for (index = 0; index < LGE_FAC_SV_LEN; index++)
    {
        pVersion[index] = index;
    }
    return;
}

/* sbp_interface */
bool LGE_FacReadNetworkCode_SBP(FactoryNetworkCode *networkCode, unsigned short networkCodeListNum, unsigned int length)
{
	int i = 0;
	bool result;
	char *temp_buf = NULL;

	if (length != LGE_FAC_NETWORK_CODE_SIZE_SBP)
	{
		printk("jjm_debug : Mismatch FactoryNetworkCode Structures!\n");
		return 0;
	}

	temp_buf = (char *)kmalloc(LGE_FAC_NETWORK_CODE_SIZE_SBP, GFP_KERNEL);

	if (temp_buf == NULL)
	{
		printk("jjm_debug : LGE_FacReadNetworkCode2 : temp_buf alloc failed!\n");
		return 0;
	}

	if (( networkCode == 0) ||(networkCodeListNum > LGE_FAC_MAX_NTCODE_COUNT_SBP ))
	{
		printk("jjm_debug : networkCodeListNum is wrong! networkCodeList = %d\n", networkCodeListNum);
		return 0;
	}

	result = _LGE_GENERIC_READ_FUN(temp_buf, LGE_FAC_NETWORK_CODE_IDX01, (sizeof(FactoryNetworkCode) * networkCodeListNum));

	if( result != 1 )
	{
		printk("jjm_debug : misc2 Read Error\n");
		kfree(temp_buf);
		return 0;
	}

	for(i = 0; i < LGE_FAC_MAX_NTCODE_COUNT_SBP; i++)
	{
		memcpy(&networkCode[i], &temp_buf[sizeof(FactoryNetworkCode)*i], sizeof(FactoryNetworkCode));
	}

	kfree(temp_buf);

	return 1;
}

bool LGE_FacReadOneBinaryHWInfo(unsigned char *data)
{
    bool result = 0;

    result = _LGE_GENERIC_READ_FUN(data, LGE_ONE_BINARY_HWINFO_IDX, LGE_ONE_BINARY_HWINFO_SIZE);

    if( result == 1 )
    {
        printk("jjm_debug : LGE_FacReadOneBinaryHWInfo Success! : OneBinaryHWInfo=%s\n", data);
    }
	else
	{
		printk("jjm_debug : LGE_FacReadOneBinaryHWInfo Fail!");
	}

    return result;
}

bool LGE_FacWriteOneBinaryHWInfo(unsigned char *data, bool needFlashProgram)
{
    bool result = 0;

    result = _LGE_GENERIC_WRITE_FUN(data, LGE_ONE_BINARY_HWINFO_IDX, LGE_ONE_BINARY_HWINFO_SIZE);

    if( result == 1 )
    {
        printk("jjm_debug : LGE_FacWriteOneBinaryHWInfo Success! : OneBinaryHWInfo=%s\n", data);
    }
	else
	{
		printk("jjm_debug : LGE_FacWriteOneBinaryHWInfo Fail!");
	}

    return result;
}

bool LGE_FacReadSVN_SBP (unsigned char *svn)
{
	bool result = 0;

	result = _LGE_GENERIC_READ_FUN (svn, LGE_FAC_IMEI_SVN_IDX, LGE_FAC_IMEI_SVN_SIZE);

	if( result == 1 )
	{
		printk("jjm_debug : LGE_FacReadSVN_SBP Success! : svn=%s\n", svn);
	}
	else
	{
		printk("jjm_debug : LGE_FacReadSVN_SBP : misc2 Read Error\n");
		return 0;
	}

	return 1;
}
/* sbp_interface */

int LGE_API_test(void)
{
#if 0
    unsigned char buff[EMMC_BLOCK_SIZE];
    int index = 0;
    memset(buff, 0xa, EMMC_BLOCK_SIZE);
    for (index = 0; index < LGE_FAC_WIFI_MAC_ADDR_LEN; index++)
    {
        buff[index] = LGE_FAC_WIFI_MAC_ADDR_OFFSET+0xa;
    }
    LGE_FacWriteWifiMacAddr(buff, true);
    memset(buff, 0xb, EMMC_BLOCK_SIZE);
    LGE_FacReadWifiMacAddr(buff);
	PRINT("kernel wifi data:");
    for (index = 0; index < 100; index++)
    {
        PRINT(" 0x%x", buff[index]);
    }
    PRINT("\n");

    memset(buff, 0xc, EMMC_BLOCK_SIZE);
    for (index = 0; index < LGE_FAC_BT_ADDR_LEN; index++)
    {
        buff[index] = LGE_FAC_BT_ADDR_OFFSET+0xc;
    }
    LGE_FacWriteBtAddr(buff, true);
    memset(buff, 0xd, EMMC_BLOCK_SIZE);
    LGE_FacReadBtAddr(buff);
	PRINT("kernel BT data:");
    for (index = 0; index < 100; index++)
    {
        PRINT(" 0x%x", buff[index]);
    }
    PRINT("\n");
#endif
    return 0;
}

EXPORT_SYMBOL(LGE_FacWriteWifiMacAddr);
EXPORT_SYMBOL(LGE_FacReadWifiMacAddr);
EXPORT_SYMBOL(LGE_FacWriteBtAddr);
EXPORT_SYMBOL(LGE_FacReadBtAddr);
EXPORT_SYMBOL(LGE_FacWriteImei);
EXPORT_SYMBOL(LGE_FacReadImei);
EXPORT_SYMBOL(LGE_FacWriteSimLockType);
EXPORT_SYMBOL(LGE_FacReadSimLockType);
EXPORT_SYMBOL(LGE_FacWriteUnlockCodeVerifyFailCount);
EXPORT_SYMBOL(LGE_FacReadUnlockCodeVerifyFailCount);
//EXPORT_SYMBOL(LGE_FacWriteUnlockFailCount);
//EXPORT_SYMBOL(LGE_FacReadUnlockFailCount);
EXPORT_SYMBOL(LGE_FacWriteUnlockCode);
EXPORT_SYMBOL(LGE_FacVerifyUnlockCode);
EXPORT_SYMBOL(LGE_FacCheckUnlockCodeValidness);
EXPORT_SYMBOL(LGE_FacWriteNetworkCode);
EXPORT_SYMBOL(LGE_FacReadNetworkCode);
EXPORT_SYMBOL(LGE_FacWriteNetworkCodeListNum);
EXPORT_SYMBOL(LGE_FacReadNetworkCodeListNum);
EXPORT_SYMBOL(LGE_FacCheckNetworkCodeValidness);
EXPORT_SYMBOL(LGE_FacInitSimLockData);
EXPORT_SYMBOL(LGE_FacReadFusgFlag);
EXPORT_SYMBOL(LGE_FacWriteFusgFlag);
EXPORT_SYMBOL(LGE_FacReadDataVersion);
EXPORT_SYMBOL(LGE_FacWriteDataVersion);
EXPORT_SYMBOL(LGE_FacReadPid);
EXPORT_SYMBOL(LGE_FacWritePid);
EXPORT_SYMBOL(LGE_FacGetSoftwareversion);
/* sbp_interface */
EXPORT_SYMBOL(LGE_FacReadNetworkCode_SBP);
EXPORT_SYMBOL(LGE_FacReadOneBinaryHWInfo);
EXPORT_SYMBOL(LGE_FacWriteOneBinaryHWInfo);
/* sbp_interface */
EXPORT_SYMBOL(LGE_API_test);
EXPORT_SYMBOL(LGE_FacReadAccelerometerCalibration);
EXPORT_SYMBOL(LGE_FacWriteAccelerometerCalibration);

MODULE_AUTHOR("Kai Zhu@mediatek.com");
MODULE_DESCRIPTION("access partition API");
MODULE_LICENSE("GPL");
