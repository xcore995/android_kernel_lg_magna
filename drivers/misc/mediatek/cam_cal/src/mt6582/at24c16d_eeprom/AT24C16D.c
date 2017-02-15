/*
 * Driver for CAM_CAL
 *
 *
 */

#if 0
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "eeprom.h"
#include "eeprom_define.h"
#include "AT24C16D.h"
#include <asm/system.h>		/* for SMP */
#else
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
/* #include "kd_camera_hw.h" */
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "AT24C16D.h"
#endif
/* #include <asm/system.h>  */ /* for SMP */

/* #define CAM_CALGETDLT_DEBUG */
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALDB printk
#else
#define CAM_CALDB(x, ...)
#endif


static DEFINE_SPINLOCK(g_CAM_CALLock);	/* for SMP */
/* #define CAM_CAL_I2C_BUSNUM 1 */

/* for init.rc static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO("CAM_CAL_AT24C16D", 0xAA>>1)}; */

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1	/* seanlin111208 */
/*******************************************************************************
*
********************************************************************************/
/* for init.rc #define CAM_CAL_DRVNAME "CAM_CAL_AT24C16D" */
/* #define CAM_CAL_DRVNAME "CAM_CAL_DRV" */
/* #define CAM_CAL_I2C_GROUP_ID 0 */
/*******************************************************************************
*
********************************************************************************/
/* static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0xAA>>1)}; */
/*******************************************************************************
/* define LSC data  */
/********************************************************************************/
#define SampleNum 221
#define Read_NUMofEEPROM 2

static struct i2c_client *g_pstI2Cclient;

/* 81 is used for V4L driver */
/* static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0); */
/* static struct cdev * g_pCAM_CAL_CharDrv = NULL; */
/* static spinlock_t g_CAM_CALLock; */
/* spin_lock(&g_CAM_CALLock); */
/* spin_unlock(&g_CAM_CALLock); */

/* static struct class *CAM_CAL_class = NULL; */
/* static atomic_t g_CAM_CALatomic; */
/* static DEFINE_SPINLOCK(kdcam_cal_drv_lock); */
/* spin_lock(&kdcam_cal_drv_lock); */
/* spin_unlock(&kdcam_cal_drv_lock); */

static unsigned short EEPROM_Address[2] = { 0x0, 0x0 };

/*******************************************************************************
*
********************************************************************************/


/*******************************************************************************
*
********************************************************************************/
/* maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt65xx.c which is 8 bytes */
int iWriteCAM_CAL_AT24C16D(u16 a_u2Addr, u32 a_u4Bytes, u8 *puDataInBytes)
{
	int i4RetValue = 0;
	u32 u4Index = 0;
	char puSendCmd[8] = { (char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF),
		0, 0, 0, 0, 0, 0
	};
	if (a_u4Bytes + 2 > 8) {
		CAM_CALDB
		    ("[CAM_CAL] exceed I2c-mt65xx.c 8 bytes limitation (include address 2 Byte)\n");
		return -1;
	}

	for (u4Index = 0; u4Index < a_u4Bytes; u4Index += 1)
		puSendCmd[(u4Index + 2)] = puDataInBytes[u4Index];

	i4RetValue = i2c_master_send(g_pstI2Cclient, puSendCmd, (a_u4Bytes + 2));
	if (i4RetValue != (a_u4Bytes + 2)) {
		CAM_CALDB("[CAM_CAL] I2C write  failed!!\n");
		return -1;
	}
	mdelay(10);		/* for tWR singnal --> write data form buffer to memory. */

	/* CAM_CALDB("[CAM_CAL] iWriteCAM_CAL_AT24C16D done!!\n"); */
	return 0;
}


/* maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt65xx.c which is 8 bytes */
int iReadCAM_CAL_AT24C16D(u16 a_u2Addr, u32 ui4_length, u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[2] = { (char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF) };

	/* CAM_CALDB("[CAM_CAL] iReadCAM_CAL_AT24C16D!!\n"); */

	if (ui4_length > 8) {
		CAM_CALDB("[CAM_CAL] exceed I2c-mt65xx.c 8 bytes limitation\n");
		return -1;
	}
	spin_lock(&g_CAM_CALLock);	/* for SMP */
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
	spin_unlock(&g_CAM_CALLock);	/* for SMP */

	/* CAM_CALDB("[CAM_CAL] i2c_master_send\n"); */
	i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
	if (i4RetValue != 2) {
		CAM_CALDB("[CAM_CAL] I2C send read address failed!!\n");
		return -1;
	}
	/* CAM_CALDB("[CAM_CAL] i2c_master_recv\n"); */
	i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
	if (i4RetValue != ui4_length) {
		CAM_CALDB("[CAM_CAL] I2C read data failed!!\n");
		return -1;
	}
	spin_lock(&g_CAM_CALLock);	/* for SMP */
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & I2C_MASK_FLAG;
	spin_unlock(&g_CAM_CALLock);	/* for SMP */

	/* CAM_CALDB("[CAM_CAL] iReadCAM_CAL_AT24C16D done!!\n"); */
	return 0;
}


static int iWriteData_AT24C16D(unsigned int ui4_offset, unsigned int ui4_length, unsigned char *pinputdata)
{
	int i4RetValue = 0;
	int i4ResidueDataLength;
	u32 u4IncOffset = 0;
	u32 u4CurrentOffset;
	u8 *pBuff;

	CAM_CALDB("[CAM_CAL] iWriteData_AT24C16D\n");

	if (ui4_offset + ui4_length >= 0x2000) {
		CAM_CALDB("[CAM_CAL] Write Error!! S-24CS64A not supprt address >= 0x2000!!\n");
		return -1;
	}

	i4ResidueDataLength = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;

	CAM_CALDB("[CAM_CAL] iWriteData_AT24C16D u4CurrentOffset is %d\n", u4CurrentOffset);

	do {
		if (i4ResidueDataLength >= 6) {
			i4RetValue = iWriteCAM_CAL_AT24C16D((u16) u4CurrentOffset, 6, pBuff);
			if (i4RetValue != 0) {
				CAM_CALDB("[CAM_CAL] I2C iWriteData_AT24C16D failed!!\n");
				return -1;
			}
			u4IncOffset += 6;
			i4ResidueDataLength -= 6;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
		} else {
			i4RetValue =
			    iWriteCAM_CAL_AT24C16D((u16) u4CurrentOffset, i4ResidueDataLength, pBuff);
			if (i4RetValue != 0) {
				CAM_CALDB("[CAM_CAL] I2C iWriteData_AT24C16D failed!!\n");
				return -1;
			}
			u4IncOffset += 6;
			i4ResidueDataLength -= 6;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
			/* break; */
		}
	} while (i4ResidueDataLength > 0);
	CAM_CALDB("[CAM_CAL] iWriteData_AT24C16D done\n");

	return 0;
}

int iReadDataFromAT24C16D(unsigned int ui4_offset, unsigned int ui4_length,
			     unsigned char *pinputdata)
{
	char puSendCmd[2];	/* = {(char)(ui4_offset & 0xFF) }; */
	/* unsigned short SampleOffset = (unsigned short)((ui4_offset) & (0x0000FFFF)) ; */
	unsigned short SampleOffset = (unsigned short)((ui4_offset & 0xFFF00000) >> 20);
	/* unsigned short EEPROM_Address[2] = {0xA0,0xA0} ; */
	/* unsigned char address_offset = ((SampleNum *SampleOffset)
				+ EEPROM_Address_Offset+ ui4_length) / Boundary_Address; */
	short loop[2], loopCount;
	unsigned short SampleCount;
	u8 *pBuff;
	u32 u4IncOffset = 0;
	int i4RetValue = 0;

	pBuff = pinputdata;

	CAM_CALDB("[AT24C16D_eeprom] ui4_offset=%x ui4_offset(80)=%x ui4_offset(8)=%x\n",
		  ui4_offset, (unsigned short)((ui4_offset >> 8) & 0x0000FFFF), SampleOffset);

	/* ui4_offset = (char)( (ui4_offset>>8) & 0xFF); */

#if 0
	EEPROM_Address[0] =
	    ((0 < address_offset) ? (EEPROM_Address[0] | (address_offset - 1)) : EEPROM_Address[0]);
	EEPROM_Address[1] = ((EEPROM_Address[0] & 0xF0) | (address_offset));

	EEPROM_Address[0] = EEPROM_Address[0] << 1;
	EEPROM_Address[1] = EEPROM_Address[1] << 1;
#endif

	CAM_CALDB("[AT24C16D_eeprom] EEPROM_Address[0]=%x EEPROM_Address[1]=%x\n",
		  (EEPROM_Address[0]), (EEPROM_Address[1]));

	/* loop[0] = (Boundary_Address * address_offset) - ((SampleNum *SampleOffset) + EEPROM_Address_Offset); */
	loop[0] = ((ui4_length >> 4) << 4);

	loop[1] = ui4_length - loop[0];


	CAM_CALDB("[AT24C16D_eeprom] loop[0]=%d loop[1]=%d\n", (loop[0]), (loop[1]));

	puSendCmd[0] = (char)(((SampleOffset + u4IncOffset) >> 8) & 0xFF);
	puSendCmd[1] = (char)((SampleOffset + u4IncOffset) & 0xFF);

	for (loopCount = 0; loopCount < Read_NUMofEEPROM; loopCount++) {
		do {
			if (16 <= loop[loopCount]) {
				CAM_CALDB("[AT24C16D_eeprom]1 loopCount=%d ", loopCount);
				CAM_CALDB("loop[loopCount]=%d puSendCmd[0]=%x ", loop[loopCount], puSendCmd[0]);
				CAM_CALDB("puSendCmd[1]=%x, EEPROM(%x)\n", puSendCmd[1], EEPROM_Address[loopCount]);

				/* iReadRegI2C(puSendCmd , 2, (u8*)pBuff,16,EEPROM_Address[loopCount]); */
				//i4RetValue =iBurstReadRegI2C(puSendCmd, 2, (u8 *) pBuff, 16,EEPROM_Address[loopCount]);
				i4RetValue=iBurstReadRegI2C(&puSendCmd[1], 1, (u8*)pBuff, 16, EEPROM_Address[loopCount] | ((puSendCmd[0]&0x7)<<1));
				if (i4RetValue != 0) {
					CAM_CALDB
					    ("[AT24C16D_eeprom] I2C iReadData failed!!\n");
					return -1;
				}
				u4IncOffset += 16;
				loop[loopCount] -= 16;
				/* puSendCmd[0] = (char)( (ui4_offset+u4IncOffset) & 0xFF) ; */
				puSendCmd[0] = (char)(((SampleOffset + u4IncOffset) >> 8) & 0xFF);
				puSendCmd[1] = (char)((SampleOffset + u4IncOffset) & 0xFF);
				pBuff = pinputdata + u4IncOffset;
			} else if (0 < loop[loopCount]) {
				CAM_CALDB("[AT24C16D_eeprom]2 loopCount=%d ", loopCount);
				CAM_CALDB("loop[loopCount]=%d puSendCmd[0]=%x ", loop[loopCount], puSendCmd[0]);
				CAM_CALDB("puSendCmd[1]=%x\n", puSendCmd[1]);

				/* iReadRegI2C(puSendCmd , 2, (u8*)pBuff,loop[loopCount],EEPROM_Address[loopCount]); */
				//i4RetValue =iBurstReadRegI2C(puSendCmd, 2, (u8 *) pBuff, 16,EEPROM_Address[loopCount]);
				i4RetValue=iBurstReadRegI2C(&puSendCmd[1], 1, (u8*)pBuff, 16, EEPROM_Address[loopCount] | ((puSendCmd[0]&0x7)<<1));
				if (i4RetValue != 0) {
					CAM_CALDB
					    ("[AT24C16D_eeprom] I2C iReadData failed!!\n");
					return -1;
				}
				u4IncOffset += loop[loopCount];
				loop[loopCount] -= loop[loopCount];
				/* puSendCmd[0] = (char)( (ui4_offset+u4IncOffset) & 0xFF) ; */
				puSendCmd[0] = (char)(((SampleOffset + u4IncOffset) >> 8) & 0xFF);
				puSendCmd[1] = (char)((SampleOffset + u4IncOffset) & 0xFF);
				pBuff = pinputdata + u4IncOffset;
			}
		} while (loop[loopCount] > 0);
	}

	return 0;
}

/* int iReadData_AT24C16D(stCAM_CAL_INFO_STRUCT * st_pOutputBuffer) */
static int iReadData_AT24C16D(unsigned int ui4_offset, unsigned int ui4_length, unsigned char *pinputdata)
{
	int i4RetValue = 0;
	int i4ResidueDataLength;
	u32 u4IncOffset = 0;
	u32 u4CurrentOffset;
	u8 *pBuff;
/* CAM_CALDB("[S24EEPORM] iReadData\n" ); */

	if (ui4_offset + ui4_length >= 0x2000) {
		CAM_CALDB
		    ("[AT24C16D_eeprom] Read Error!! S-AT24C16D_eeprom not supprt address >= 0x2000!!\n");
		return -1;
	}

	i4ResidueDataLength = (int)ui4_length;
	u4CurrentOffset = ui4_offset;
	pBuff = pinputdata;
	do {
		if (i4ResidueDataLength >= 8) {
			i4RetValue = iReadCAM_CAL_AT24C16D((u16) u4CurrentOffset, 8, pBuff);
			if (i4RetValue != 0) {
				CAM_CALDB("[AT24C16D_eeprom] I2C iReadData failed!!\n");
				return -1;
			}
			u4IncOffset += 8;
			i4ResidueDataLength -= 8;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
		} else {
			i4RetValue =
			    iReadCAM_CAL_AT24C16D((u16) u4CurrentOffset, i4ResidueDataLength, pBuff);
			if (i4RetValue != 0) {
				CAM_CALDB("[AT24C16D_eeprom] I2C iReadData failed!!\n");
				return -1;
			}
			u4IncOffset += 8;
			i4ResidueDataLength -= 8;
			u4CurrentOffset = ui4_offset + u4IncOffset;
			pBuff = pinputdata + u4IncOffset;
			/* break; */
		}
	} while (i4ResidueDataLength > 0);
/* CAM_CALDB("[S24EEPORM] iReadData finial address is %d length is %d buffer address is 0x%x\n"
	,u4CurrentOffset, i4ResidueDataLength, pBuff); */
/* CAM_CALDB("[S24EEPORM] iReadData done\n" ); */
	return 0;
}


/*******************************************************************************
*
********************************************************************************/
#if defined(COMMON_EEPROM)
int AT24C16D_EEPROM_Ioctl(struct file *file, unsigned int a_u4Command, unsigned long a_u4Param)
#else				/* COMMON_CAM_CAL_DRV */
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode *a_pstInode,
			 struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(struct file *file, unsigned int a_u4Command, unsigned long a_u4Param)
#endif
#endif				/* COMMON_CAM_CAL_DRV */
{
	int i4RetValue = 0;
	u8 *pBuff = NULL;
	u8 *pWorkingBuff = NULL;
	stCAM_CAL_INFO_STRUCT *ptempbuf;
	u8 readTryagain = 0, test_retry = 0;

#ifdef CAM_CALGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif

	if (_IOC_NONE != _IOC_DIR(a_u4Command)) {
		pBuff = kmalloc(sizeof(stCAM_CAL_INFO_STRUCT), GFP_KERNEL);

		if (NULL == pBuff) {
			CAM_CALDB("[AT24C16D_eeprom] ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
			if (copy_from_user((u8 *) pBuff, (u8 *) a_u4Param,
				sizeof(stCAM_CAL_INFO_STRUCT))) {	/* get input structure address */
				kfree(pBuff);
				CAM_CALDB("[AT24C16D_eeprom] ioctl copy from user failed\n");
				return -EFAULT;
			}
		}
	}

	ptempbuf = (stCAM_CAL_INFO_STRUCT *) pBuff;
	pWorkingBuff = kmalloc(ptempbuf->u4Length, GFP_KERNEL);
	if (NULL == pWorkingBuff) {
		kfree(pBuff);
		CAM_CALDB("[AT24C16D_eeprom] ioctl allocate mem failed\n");
		return -ENOMEM;
	}
	CAM_CALDB("[AT24C16D_eeprom] init Working buffer address 0x%8x  command is 0x%8x\n",
		  (u32) pWorkingBuff, (u32) a_u4Command);


	if (copy_from_user((u8 *) pWorkingBuff, (u8 *) ptempbuf->pu1Params, ptempbuf->u4Length)) {
		kfree(pBuff);
		kfree(pWorkingBuff);
		CAM_CALDB("[AT24C16D_eeprom] ioctl copy from user failed\n");
		return -EFAULT;
	}

	switch (a_u4Command) {
	case CAM_CALIOC_S_WRITE:
		CAM_CALDB("[AT24C16D_eeprom] Write CMD\n");
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		i4RetValue = iWriteData_AT24C16D((u16) ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

		CAM_CALDB("Write data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif
		break;
	case CAM_CALIOC_G_READ:
		CAM_CALDB("[AT24C16D_eeprom] Read CMD\n");
#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv1);
#endif
		CAM_CALDB("[AT24C16D_eeprom] offset %x\n", ptempbuf->u4Offset);
		CAM_CALDB("[AT24C16D_eeprom] length %d\n", ptempbuf->u4Length);
		CAM_CALDB("[AT24C16D_eeprom] Before read Working buffer address 0x%8x\n",
			  (u32) pWorkingBuff);

		/* i4RetValue = iReadData_AT24C16D((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff); */
		CAM_CALDB("[AT24C16D_eeprom] After read Working buffer data  0x%4x\n",
			  *pWorkingBuff);
		CAM_CALDB("POPO [AT24C16D_eeprom] EEPROM_Address[0] = %d\n", EEPROM_Address[0]);
		CAM_CALDB("POPO [AT24C16D_eeprom] EEPROM_Address[1] = %d\n", EEPROM_Address[1]);		
		if ((ptempbuf->u4Offset & 0xFFFFFFFF) == 0xFFFA24C1) {
			EEPROM_Address[0] = 0x0;
			EEPROM_Address[1] = 0x0;
		} else if (((ptempbuf->u4Offset & 0x000FFFFF) == 0x000A24C1)
			   && (EEPROM_Address[0] == 0x0)) {
			*(u32 *) pWorkingBuff = (ptempbuf->u4Offset | 0x10000000);
			EEPROM_Address[0] = (ptempbuf->u4Offset & 0xffff0000) >> 20;
			EEPROM_Address[1] = (ptempbuf->u4Offset & 0xffff0000) >> 20;
		} else if (ptempbuf->u4Offset == 0xFFFFFFFF) {
			char puSendCmd[1] = { 0, };

			puSendCmd[0] = 0x7E;
			/* iReadRegI2C(puSendCmd , 1, pWorkingBuff,2, (0x53<<1) ); */
			CAM_CALDB("[AT24C16D_eeprom] Shading CheckSum MSB=> %x %x\n",
				  pWorkingBuff[0], pWorkingBuff[1]);
		} else {
			/* i4RetValue = iReadData_AT24C16D((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff); */

			/* i4RetValue =  iReadData_AT24C16DFromM24C08F((u16)ptempbuf->u4Offset, ptempbuf->u4Length,
				pWorkingBuff); */
			readTryagain = 3;
			test_retry = 2;
			while (0 < readTryagain) {
				/* i4RetValue =  iReadDataFromAT24C16D((u16)ptempbuf->u4Offset,
					ptempbuf->u4Length, pWorkingBuff); */
				/* i4RetValue =  iReadDataFromM24C08F((u16)ptempbuf->u4Offset,
					ptempbuf->u4Length, pWorkingBuff); */
				i4RetValue =
				    iReadDataFromAT24C16D(ptempbuf->u4Offset, ptempbuf->u4Length,
							     pWorkingBuff);
#if 0
				if (0 < test_retry) {
					CAM_CALDB
					    ("[AT24C16D_eeprom] Test error (%d) Read retry (%d)\n",
					     test_retry, i4RetValue);
					i4RetValue = -1;
					test_retry--;
				}
#endif
				CAM_CALDB("[AT24C16D_eeprom] error (%d) Read retry (%d)\n",
					  i4RetValue, readTryagain);
				if (i4RetValue != 0)
					readTryagain--;
				else
					readTryagain = 0;

			}
		}

#ifdef CAM_CALGETDLT_DEBUG
		do_gettimeofday(&ktv2);
		if (ktv2.tv_sec > ktv1.tv_sec)
			TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
		else
			TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

		CAM_CALDB("Read data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif

		break;
	default:
		CAM_CALDB("[AT24C16D_eeprom] No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	if (_IOC_READ & _IOC_DIR(a_u4Command)) {
		/* copy data to user space buffer, keep other input paremeter unchange. */
		CAM_CALDB("[AT24C16D_eeprom] to user length %d\n", ptempbuf->u4Length);
		CAM_CALDB("[AT24C16D_eeprom] to user  Working buffer address 0x%8x\n",
			  (u32) pWorkingBuff);
		if (copy_to_user
		    ((u8 __user *) ptempbuf->pu1Params, (u8 *) pWorkingBuff, ptempbuf->u4Length)) {
			kfree(pBuff);
			kfree(pWorkingBuff);
			CAM_CALDB("[S24CAM_CAL] ioctl copy to user failed\n");
			return -EFAULT;
		}
	}

	kfree(pBuff);
	kfree(pWorkingBuff);
	return i4RetValue;
}

#if 0

static u32 g_u4Opened;
/* #define */
/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
static int CAM_CAL_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	CAM_CALDB("[AT24C16D_eeprom] CAM_CAL_Open\n");
	spin_lock(&g_CAM_CALLock);
	if (g_u4Opened) {
		spin_unlock(&g_CAM_CALLock);
		return -EBUSY;
	}
	g_u4Opened = 1;
	atomic_set(&g_CAM_CALatomic, 0);

	spin_unlock(&g_CAM_CALLock);

#if 0
	if (TRUE != hwPowerOn(MT65XX_POWER_LDO_VCAMA, VOL_2800, "AT24C16D")) {
		CAM_CALDB("[S24CAM_CAL] Fail to enable analog gain\n");
		return -EIO;
	}
#endif
	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int CAM_CAL_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	spin_lock(&g_CAM_CALLock);

	g_u4Opened = 0;

	atomic_set(&g_CAM_CALatomic, 0);

	spin_unlock(&g_CAM_CALLock);

	return 0;
}

static const struct file_operations g_stCAM_CAL_fops = {
	.owner = THIS_MODULE,
	.open = CAM_CAL_Open,
	.release = CAM_CAL_Release,
	/* .ioctl = CAM_CAL_Ioctl */
#if defined(COMMON_CAM_CAL_DRV)
#else
	.unlocked_ioctl = CAM_CAL_Ioctl
#endif
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
static int RegisterCAM_CALCharDrv(void)
{
	struct device *CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
	if (alloc_chrdev_region(&g_CAM_CALdevno, 0, 1, CAM_CAL_DRVNAME)) {
		CAM_CALDB("[AT24C16D_eeprom] Allocate device no failed\n");

		return -EAGAIN;
	}
#else
	if (register_chrdev_region(g_CAM_CALdevno, 1, CAM_CAL_DRVNAME)) {
		CAM_CALDB("[AT24C16D_eeprom] Register device no failed\n");

		return -EAGAIN;
	}
#endif

	/* Allocate driver */
	g_pCAM_CAL_CharDrv = cdev_alloc();

	if (NULL == g_pCAM_CAL_CharDrv) {
		unregister_chrdev_region(g_CAM_CALdevno, 1);

		CAM_CALDB("[AT24C16D_eeprom] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}
	/* Attatch file operation. */
	cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

	g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1)) {
		CAM_CALDB("[AT24C16D_eeprom] Attatch file operation failed\n");

		unregister_chrdev_region(g_CAM_CALdevno, 1);

		return -EAGAIN;
	}

	CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
	if (IS_ERR(CAM_CAL_class)) {
		int ret = PTR_ERR(CAM_CAL_class);

		CAM_CALDB("Unable to create class, err = %d\n", ret);
		return ret;
	}
	CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

	return 0;
}

static void UnregisterCAM_CALCharDrv(void)
{
	/* Release char driver */
	cdev_del(g_pCAM_CAL_CharDrv);

	unregister_chrdev_region(g_CAM_CALdevno, 1);

	device_destroy(CAM_CAL_class, g_CAM_CALdevno);
	class_destroy(CAM_CAL_class);
}


/* //////////////////////////////////////////////////////////////////// */
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = { {CAM_CAL_DRVNAME, 0}, {} };

#if 0				/* test110314 Please use the same I2C Group ID as Sensor */
static unsigned short force[] = {
	CAM_CAL_I2C_GROUP_ID, AT24C16D_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END };
#else
/* static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, AT24C16D_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END}; */
#endif
/* static const unsigned short * const forces[] = { force, NULL }; */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */


static struct i2c_driver CAM_CAL_i2c_driver = {
	.probe = CAM_CAL_i2c_probe,
	.remove = CAM_CAL_i2c_remove,
/* .detect = CAM_CAL_i2c_detect, */
	.driver.name = CAM_CAL_DRVNAME,
	.id_table = CAM_CAL_i2c_id,
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, CAM_CAL_DRVNAME);
	return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;

	CAM_CALDB("[AT24C16D_eeprom] Attach I2C\n");
/* spin_lock_init(&g_CAM_CALLock); */

	/* get sensor i2c client */
	spin_lock(&g_CAM_CALLock);	/* for SMP */
	g_pstI2Cclient = client;
	g_pstI2Cclient->addr = AT24C16D_DEVICE_ID >> 1;
	spin_unlock(&g_CAM_CALLock);	/* for SMP */

	CAM_CALDB("[AT24C16D_eeprom] g_pstI2Cclient->addr = 0x%8x\n", g_pstI2Cclient->addr);
	/* Register char driver */
	i4RetValue = RegisterCAM_CALCharDrv();

	if (i4RetValue) {
		CAM_CALDB("[AT24C16D_eeprom] register char device failed!\n");
		return i4RetValue;
	}


	CAM_CALDB("[AT24C16D_eeprom] Attached!!\n");
	return 0;
}

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
	return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
	i2c_del_driver(&CAM_CAL_i2c_driver);
	return 0;
}

/* platform structure */
static struct platform_driver g_stCAM_CAL_Driver = {
	.probe = CAM_CAL_probe,
	.remove = CAM_CAL_remove,
	.driver = {
		   .name = CAM_CAL_DRVNAME,
		   .owner = THIS_MODULE,
		   }
};


static struct platform_device g_stCAM_CAL_Device = {
	.name = CAM_CAL_DRVNAME,
	.id = 0,
	.dev = {
		}
};

static int __init CAM_CAL_i2C_init(void)
{
	i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
	if (platform_driver_register(&g_stCAM_CAL_Driver)) {
		CAM_CALDB("failed to register AT24C16D_eeprom driver\n");
		return -ENODEV;
	}

	if (platform_device_register(&g_stCAM_CAL_Device)) {
		CAM_CALDB("failed to register AT24C16D_eeprom driver, 2nd time\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}
#endif

#if defined(COMMON_EEPROM)
EXPORT_SYMBOL(AT24C16D_EEPROM_Ioctl);
#else
module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);
#endif

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");
