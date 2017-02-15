/*
 * Driver for EEPROM
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
#include "GT24c32a.h"
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

#include "Common_eeprom.h"
/* #include <asm/system.h>  // for SMP */
#endif
/* #define EEPROMGETDLT_DEBUG */
#define EEPROM_DEBUG
#ifdef EEPROM_DEBUG
#define EEPROMDB printk
#else
#define EEPROMDB(x, ...)
#endif


static DEFINE_SPINLOCK(g_EEPROMLock);	/* for SMP */
#define EEPROM_I2C_BUSNUM 1
/* static struct i2c_board_info __initdata kd_eeprom_dev={ I2C_BOARD_INFO("CAM_CAL_DRV", 0xAA>>1)}; */

/*******************************************************************************
*
********************************************************************************/
#define EEPROM_ICS_REVISION 1	/* seanlin111208 */
/*******************************************************************************
*
********************************************************************************/
#define EEPROM_DRVNAME "CAM_CAL_DRV"
#define EEPROM_I2C_GROUP_ID 0

/*******************************************************************************
*
********************************************************************************/
/* #define FM50AF_EEPROM_I2C_ID 0x28 */
#define FM50AF_EEPROM_I2C_ID 0xA1


/*******************************************************************************
/* define LSC data for M24C08F EEPROM on L10 project */
/********************************************************************************/
#define SampleNum 221
#define Read_NUMofEEPROM 2
#define Boundary_Address 256
#define EEPROM_Address_Offset 0xC


/*******************************************************************************
*
********************************************************************************/
static struct i2c_client *g_pstI2Cclient;

/* 81 is used for V4L driver */
static dev_t g_EEPROMdevno = MKDEV(EEPROM_DEV_MAJOR_NUMBER, 0);
static struct cdev *g_pEEPROM_CharDrv;
/* static spinlock_t g_EEPROMLock; */
/* spin_lock(&g_EEPROMLock); */
/* spin_unlock(&g_EEPROMLock); */

static struct class *EEPROM_class;
static atomic_t g_EEPROMatomic;
/* static DEFINE_SPINLOCK(kdeeprom_drv_lock); */
/* spin_lock(&kdeeprom_drv_lock); */
/* spin_unlock(&kdeeprom_drv_lock); */



#if defined(GT24C32A_EEPROM)
extern int GT24C32A_EEPROM_Ioctl(struct file *file, unsigned int a_u4Command,
				 unsigned long a_u4Param);
#endif

#if defined(BRCB032GWZ_3_EEPROM)
extern int BRCB032GW_3_EEPROM_Ioctl(struct file *file, unsigned int a_u4Command,
				    unsigned long a_u4Param);
#endif

#if defined(BRCC064GWZ_3_EEPROM)
extern int BRCC064GWZ_3_EEPROM_Ioctl(struct file *file, unsigned int a_u4Command,
				     unsigned long a_u4Param);
#endif

#if defined(DW9716_EEPROM)
extern int DW9716_EEPROM_Ioctl(struct file *file, unsigned int a_u4Command,
			       unsigned long a_u4Param);
#endif

#if defined(AT24C16D_EEPROM)
extern int AT24C16D_EEPROM_Ioctl(struct file *file, unsigned int a_u4Command,
			       unsigned long a_u4Param);
#endif

/*******************************************************************************
*
********************************************************************************/

/*******************************************************************************
*
********************************************************************************/
/* maximun read length is limited at "I2C_FIFO_SIZE" in I2c-mt6516.c which is 8 bytes */


/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int EEPROM_Ioctl(struct inode *a_pstInode,
			struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
#else
static long EEPROM_Ioctl(struct file *file, unsigned int a_u4Command, unsigned long a_u4Param)
#endif
{
	int i4RetValue = 0;
	u8 *pBuff = NULL;
/* u8 * pWorkingBuff = NULL; */
	stCAM_CAL_INFO_STRUCT *ptempbuf;
/* ssize_t writeSize; */
/* u8 readTryagain=0; */

#ifdef EEPROMGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif

	EEPROMDB("[COMMON_EEPROM]1 In to IOCTL %x %x\n", _IOC_DIR(a_u4Command), _IOC_WRITE);
	EEPROMDB("[COMMON_EEPROM]2 In to IOCTL %x %x\n", CAM_CALIOC_G_READ, CAM_CALIOC_S_WRITE);

	if (_IOC_NONE != _IOC_DIR(a_u4Command)) {
		pBuff = kmalloc(sizeof(stCAM_CAL_INFO_STRUCT), GFP_KERNEL);

		if (NULL == pBuff) {
			EEPROMDB("[S24EEPROM] ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
			if (copy_from_user((u8 *) pBuff, (u8 *) a_u4Param,
				sizeof(stCAM_CAL_INFO_STRUCT))) {	/* get input structure address */
				kfree(pBuff);
				EEPROMDB("[S24EEPROM] ioctl copy from user failed\n");
				return -EFAULT;
			}
		}
	}

	ptempbuf = (stCAM_CAL_INFO_STRUCT *) pBuff;

	EEPROMDB("[COMMON_EEPROM] In to IOCTL %x (0x%08x)\n", ptempbuf->u4Offset,
		 (ptempbuf->u4Offset & 0x000FFFFF));

	if ((ptempbuf->u4Offset & 0x000FFFFF) == 0x00024C32) {
#if defined(GT24C32A_EEPROM)
		EEPROMDB("[GT24C32A] Jump to IOCTL\n");

		i4RetValue = GT24C32A_EEPROM_Ioctl(file, a_u4Command, a_u4Param);
#else
		EEPROMDB("[GT24C32A] Not defined in config\n");
#endif
	} else if ((ptempbuf->u4Offset & 0x000FFFFF) == 0x000BCB32) {
#if defined(BRCB032GWZ_3_EEPROM)
		EEPROMDB("[BRCB032GW] Jump to IOCTL\n");

		i4RetValue = BRCB032GW_3_EEPROM_Ioctl(file, a_u4Command, a_u4Param);
#else
		EEPROMDB("[BRCB032GW] Not defined in config\n");
#endif
	} else if ((ptempbuf->u4Offset & 0x000FFFFF) == 0x000BCC64) {
#if defined(BRCC064GWZ_3_EEPROM)
		EEPROMDB("[BRCC064GW] Jump to IOCTL\n");

		i4RetValue = BRCC064GWZ_3_EEPROM_Ioctl(file, a_u4Command, a_u4Param);
#else
		EEPROMDB("[BRCC064GW] Not defined in config\n");
#endif
	} else if ((ptempbuf->u4Offset & 0x000FFFFF) == 0x000d9716) {
#if defined(DW9716_EEPROM)
		EEPROMDB("[DW9716EEPROM] Jump to IOCTL\n");

		i4RetValue = DW9716_EEPROM_Ioctl(file, a_u4Command, a_u4Param);
#else
		EEPROMDB("[DW9716EEPROM] Not defined in config\n");
#endif
	} else if ((ptempbuf->u4Offset & 0x000FFFFF) == 0x000A24C1) {
#if defined(AT24C16D_EEPROM)
		EEPROMDB("[AT24C16D_EEPROM] Jump to IOCTL\n");

		i4RetValue = AT24C16D_EEPROM_Ioctl(file, a_u4Command, a_u4Param);
#else
		EEPROMDB("[AT24C16D_EEPROM] Not defined in config\n");
#endif
	} else {
		EEPROMDB("[COMMON_EEPROM] Masic number is wrong\n");
	}

	kfree(pBuff);

	return i4RetValue;
}

static u32 g_u4Opened;
/* #define */
/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
static int EEPROM_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	EEPROMDB("[COMMON_EEPROM] EEPROM_Open Client %p\n", g_pstI2Cclient);
	spin_lock(&g_EEPROMLock);
	if (g_u4Opened) {
		spin_unlock(&g_EEPROMLock);
		return -EBUSY;
	}
	g_u4Opened = 1;
	atomic_set(&g_EEPROMatomic, 0);

	spin_unlock(&g_EEPROMLock);

	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int EEPROM_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	spin_lock(&g_EEPROMLock);

	g_u4Opened = 0;

	atomic_set(&g_EEPROMatomic, 0);

	spin_unlock(&g_EEPROMLock);

	return 0;
}

static const struct file_operations g_stEEPROM_fops = {
	.owner = THIS_MODULE,
	.open = EEPROM_Open,
	.release = EEPROM_Release,
	/* .ioctl = EEPROM_Ioctl */
	.unlocked_ioctl = EEPROM_Ioctl
};

#define EEPROM_DYNAMIC_ALLOCATE_DEVNO 1
static int RegisterEEPROMCharDrv(void)
{
	struct device *EEPROM_device = NULL;

#if EEPROM_DYNAMIC_ALLOCATE_DEVNO
	if (alloc_chrdev_region(&g_EEPROMdevno, 0, 1, EEPROM_DRVNAME)) {
		EEPROMDB("[S24EEPROM] Allocate device no failed\n");

		return -EAGAIN;
	}
#else
	if (register_chrdev_region(g_EEPROMdevno, 1, EEPROM_DRVNAME)) {
		EEPROMDB("[COMMON_EEPROM] Register device no failed\n");

		return -EAGAIN;
	}
#endif

	/* Allocate driver */
	g_pEEPROM_CharDrv = cdev_alloc();

	if (NULL == g_pEEPROM_CharDrv) {
		unregister_chrdev_region(g_EEPROMdevno, 1);

		EEPROMDB("[COMMON_EEPROM] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}
	/* Attatch file operation. */
	cdev_init(g_pEEPROM_CharDrv, &g_stEEPROM_fops);

	g_pEEPROM_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pEEPROM_CharDrv, g_EEPROMdevno, 1)) {
		EEPROMDB("[COMMON_EEPROM] Attatch file operation failed\n");

		unregister_chrdev_region(g_EEPROMdevno, 1);

		return -EAGAIN;
	}

	EEPROM_class = class_create(THIS_MODULE, "EEPROMdrv");
	if (IS_ERR(EEPROM_class)) {
		int ret = PTR_ERR(EEPROM_class);

		EEPROMDB("Unable to create class, err = %d\n", ret);
		return ret;
	}
	EEPROM_device = device_create(EEPROM_class, NULL, g_EEPROMdevno, NULL, EEPROM_DRVNAME);

	return 0;
}

static void UnregisterEEPROMCharDrv(void)
{
	/* Release char driver */
	cdev_del(g_pEEPROM_CharDrv);

	unregister_chrdev_region(g_EEPROMdevno, 1);

	device_destroy(EEPROM_class, g_EEPROMdevno);
	class_destroy(EEPROM_class);
}

#if 0
/* //////////////////////////////////////////////////////////////////// */
#ifndef EEPROM_ICS_REVISION
static int EEPROM_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int EEPROM_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int EEPROM_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int EEPROM_i2c_remove(struct i2c_client *);

static const struct i2c_device_id EEPROM_i2c_id[] = { {EEPROM_DRVNAME, 0}, {} };

#if 0				/* test110314 Please use the same I2C Group ID as Sensor */
static unsigned short force[] = {
	EEPROM_I2C_GROUP_ID, S24CS64A_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END };
#else
/* static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, S24CS64A_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END}; */
#endif
/* static const unsigned short * const forces[] = { force, NULL }; */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */


static struct i2c_driver EEPROM_i2c_driver = {
	.probe = EEPROM_i2c_probe,
	.remove = EEPROM_i2c_remove,
/* .detect = EEPROM_i2c_detect, */
	.driver.name = EEPROM_DRVNAME,
	.id_table = EEPROM_i2c_id,
};

#ifndef EEPROM_ICS_REVISION
static int EEPROM_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, EEPROM_DRVNAME);
	return 0;
}
#endif
static int EEPROM_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;

	EEPROMDB("[S24EEPROM] Attach I2C\n");
/* spin_lock_init(&g_EEPROMLock); */

	/* get sensor i2c client */
	spin_lock(&g_EEPROMLock);	/* for SMP */
	g_pstI2Cclient = client;
	g_pstI2Cclient->addr = COMMON_DEVICE_ID >> 1;
	spin_unlock(&g_EEPROMLock);	/* for SMP */

	EEPROMDB("[COMMON_EEPROM] g_pstI2Cclient->addr = 0x%8x\n", g_pstI2Cclient->addr);
	/* Register char driver */
	i4RetValue = RegisterEEPROMCharDrv();

	if (i4RetValue) {
		EEPROMDB("[COMMON_EEPROM] register char device failed!\n");
		return i4RetValue;
	}


	EEPROMDB("[COMMON_EEPROM] Attached!!\n");
	return 0;
}

static int EEPROM_i2c_remove(struct i2c_client *client)
{
	return 0;
}
#endif

static int EEPROM_probe(struct platform_device *pdev)
{
	/* return i2c_add_driver(&EEPROM_i2c_driver); */
	return 0;
}

static int EEPROM_remove(struct platform_device *pdev)
{
	/* i2c_del_driver(&EEPROM_i2c_driver); */
	return 0;
}

/* platform structure */
static struct platform_driver g_stEEPROM_Driver = {
	.probe = EEPROM_probe,
	.remove = EEPROM_remove,
	.driver = {
		   .name = EEPROM_DRVNAME,
		   .owner = THIS_MODULE,
		   }
};


static struct platform_device g_stEEPROM_Device = {
	.name = EEPROM_DRVNAME,
	.id = 0,
	.dev = {
		}
};

static int __init EEPROM_i2C_init(void)
{
	/* i2c_register_board_info(EEPROM_I2C_BUSNUM, &kd_eeprom_dev, 1); */
	RegisterEEPROMCharDrv();

	if (platform_device_register(&g_stEEPROM_Device)) {
		EEPROMDB("failed to register S24EEPROM driver, 2nd time\n");
		return -ENODEV;
	}


	if (platform_driver_register(&g_stEEPROM_Driver)) {
		EEPROMDB("failed to register S24EEPROM driver\n");
		return -ENODEV;
	}



	return 0;
}

static void __exit EEPROM_i2C_exit(void)
{
	platform_driver_unregister(&g_stEEPROM_Driver);
}
module_init(EEPROM_i2C_init);
module_exit(EEPROM_i2C_exit);

MODULE_DESCRIPTION("EEPROM driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");
