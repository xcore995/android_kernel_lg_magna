/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Benoit Goby <benoit@android.com>
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
#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/printk.h>

/* Add for HW/SW connect */
#include <linux/musb/mtk_musb.h>
/* Add for HW/SW connect */

#include "gadget_chips.h"


#include "f_fs.c"
#include "f_audio_source.c"
#ifdef CONFIG_SND_RAWMIDI
#include "f_midi.c"
#endif
#include "f_mass_storage.c"
#include "f_adb.c"
#include "f_mtp.c"
#include "f_accessory.c"
#define USB_ETH_RNDIS y
#include "f_rndis.c"
#include "rndis.c"
#include "f_ecm.c"
#include "f_eem.c"
#include "u_ether.c"

#include "f_laf.c"
#ifdef CONFIG_LGE_USB_G_ANDROID
#include <mach/board_lge.h>
#include "f_charge_only.c"
#include <mach/mt_boot_common.h>
#endif
#ifdef CONFIG_MTK_C2K_SUPPORT
#include <mach/viatel_rawbulk.h>
int rawbulk_bind_config(struct usb_configuration *c, int transfer_id);
int rawbulk_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl);
#endif


MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by userspace */
#define VENDOR_ID		0x0BB4
#define PRODUCT_ID		0x0001

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
#include <mach/mt_boot_common.h>
#define KPOC_USB_FUNC "mtp"
#define KPOC_USB_VENDOR_ID 0x0E8D
#define KPOC_USB_PRODUCT_ID 0x2008
#ifdef CONFIG_LGE_USB_G_ANDROID
extern BOOTMODE g_boot_mode;
#endif
#endif

/* f_midi configuration */
#define MIDI_INPUT_PORTS    1
#define MIDI_OUTPUT_PORTS   1
#define MIDI_BUFFER_SIZE    512
#define MIDI_QUEUE_LENGTH   32

/* Default manufacturer and product string , overridden by userspace */
#ifndef CONFIG_LGE_USB_G_ANDROID
#define MANUFACTURER_STRING "MediaTek"
#define PRODUCT_STRING "MT65xx Android Phone"
#else
#define MANUFACTURER_STRING "LGE"
#define PRODUCT_STRING "LGE Android Phone"
#endif

//#define USB_LOG "USB"
#ifdef CONFIG_LGE_USB_G_ANDROID
enum
{
	USB_NORMAL = 0, //for user
	USB_FACTORY,     //for factory
	USB_FACTORY_0, //for debugging
	USB_FACTORY_1, //for debugging
} fusb_mode;
#endif

struct android_usb_function {
	char *name;
	void *config;
	/* set only when function's bind_config() is called. */
	bool bound;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *android_dev;
#else
	/* for android_dev.enabled_functions */
	struct list_head enabled_list;
#endif
	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);
	/* Optional: called when the function is added the list of
	 *		enabled functions */
	void (*enable)(struct android_usb_function *);
	/* Optional: called when it is removed */
	void (*disable)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *,
			   struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *,
			      struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

#ifdef CONFIG_LGE_USB_G_ANDROID
struct android_usb_function_holder {

	struct android_usb_function *f;

	/* for android_conf.enabled_functions */
	struct list_head enabled_list;
};

/**
* struct android_dev - represents android USB gadget device
* @name: device name.
* @functions: an array of all the supported USB function
*    drivers that this gadget support but not necessarily
*    added to one of the gadget configurations.
* @cdev: The internal composite device. Android gadget device
*    is a composite device, such that it can support configurations
*    with more than one function driver.
* @dev: The kernel device that represents this android device.
* @enabled: True if the android gadget is enabled, means all
*    the configurations were set and all function drivers were
*    bind and ready for USB enumeration.
* @disable_depth: Number of times the device was disabled, after
*    symmetrical number of enables the device willl be enabled.
*    Used for controlling ADB userspace disable/enable requests.
* @mutex: Internal mutex for protecting device member fields.
* @pdata: Platform data fetched from the kernel device platfrom data.
* @connected: True if got connect notification from the gadget UDC.
*    False if got disconnect notification from the gadget UDC.
* @sw_connected: Equal to 'connected' only after the connect
*    notification was handled by the android gadget work function.
* @suspended: True if got suspend notification from the gadget UDC.
*    False if got resume notification from the gadget UDC.
* @sw_suspended: Equal to 'suspended' only after the susped
*    notification was handled by the android gadget work function.
* @pm_qos: An attribute string that can be set by user space in order to
*    determine pm_qos policy. Set to 'high' for always demand pm_qos
*    when USB bus is connected and resumed. Set to 'low' for disable
*    any setting of pm_qos by this driver. Default = 'high'.
* @work: workqueue used for handling notifications from the gadget UDC.
* @configs: List of configurations currently configured into the device.
*    The android gadget supports more than one configuration. The host
*    may choose one configuration from the suggested.
* @configs_num: Number of configurations currently configured and existing
*    in the configs list.
* @list_item: This driver supports more than one android gadget device (for
*    example in order to support multiple USB cores), therefore this is
*    a item in a linked list of android devices.
*/
#endif

struct android_dev {
#ifdef CONFIG_LGE_USB_G_ANDROID
	const char *name;
#endif
	struct android_usb_function **functions;
#ifndef CONFIG_LGE_USB_G_ANDROID
	struct list_head enabled_functions;
#endif
	struct usb_composite_dev *cdev;
	struct device *dev;

	void (*setup_complete)(struct usb_ep *ep,
				struct usb_request *req);

	bool enabled;
	int disable_depth;
	struct mutex mutex;
	bool connected;
	bool sw_connected;
	struct work_struct work;
	char ffs_aliases[256];
#ifdef CONFIG_LGE_USB_G_ANDROID
	bool check_charge_only;
#endif
	int rezero_cmd;
#ifdef CONFIG_LGE_USB_G_ANDROID
	int lock;
#endif
#ifdef CONFIG_LGE_USB_G_ANDROID
	/* A list of struct android_configuration */
	struct list_head configs;
	int configs_num;

	/* A list node inside the android_dev_list */
	struct list_head list_item;
#endif
};

#ifdef CONFIG_LGE_USB_G_ANDROID
struct android_configuration {
	struct usb_configuration usb_config;

	/* A list of the functions supported by this config */
	struct list_head enabled_functions;

	/* A list node inside the struct android_dev.configs list */
	struct list_head list_item;
};
#endif
static struct class *android_class;
#ifndef CONFIG_LGE_USB_G_ANDROID
static struct android_dev *_android_dev;
#endif
#ifdef CONFIG_LGE_USB_G_ANDROID
static struct list_head android_dev_list;
static int android_dev_count;
#endif
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);
#ifndef CONFIG_LGE_USB_G_ANDROID
static int android_setup_config(struct usb_configuration *c, const struct usb_ctrlrequest *ctrl);
#endif
#ifdef CONFIG_LGE_USB_G_ANDROID
static struct android_dev *cdev_to_android_dev(struct usb_composite_dev *cdev);
static struct android_configuration *alloc_android_config
						(struct android_dev *dev);
static void free_android_config(struct android_dev *dev,
				struct android_configuration *conf);
#endif

/* string IDs are assigned dynamically */
#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

#ifdef CONFIG_LGE_USB_G_ANDROID
#define LGE_PIF_VID 0x1004
#define LGE_PIF_PID 0x6000
#define LGE_PIF_SN 0
#endif
static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];

#ifdef CONFIG_LGE_USB_G_ANDROID
#define CHARGE_ONLY_STRING_IDX		3
static char charge_only_string[256];
#endif
/* String Table */
static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
#ifdef CONFIG_LGE_USB_G_ANDROID
	[CHARGE_ONLY_STRING_IDX].s = charge_only_string,
#endif
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
#ifdef CONFIG_USB_MU3D_DRV
	.bcdUSB               = __constant_cpu_to_le16(0x0300),
#else
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
#endif
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};
#ifndef CONFIG_LGE_USB_G_ANDROID
static struct usb_configuration android_config_driver = {
	.label		= "android",
	.unbind		= android_unbind_config,
	.setup		= android_setup_config,
	.bConfigurationValue = 1,
#ifdef CONFIG_USBIF_COMPLIANCE
	// USBIF, do we need to set bus powered
	.bmAttributes	= USB_CONFIG_ATT_ONE,
	//.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
#else
	.bmAttributes	= USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
#endif
#ifdef CONFIG_USB_MU3D_DRV
	.MaxPower	= 192, /* Only consume 192ma for passing USB30CV Descriptor Test [USB3.0 devices]*/
#else
	.MaxPower	= 500, /* 500ma */
#endif
};
#endif

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
#if 0
	/* Add for HW/SW connect */
	char *hwdisconnected[2] = { "USB_STATE=HWDISCONNECTED", NULL };
//	char *hwconnected[2]    = { "USB_STATE=HWCONNECTED", NULL };
	/* Add for HW/SW connect */
#endif
#ifdef CONFIG_LGE_USB_G_ANDROID_AUTORUN
	char *rezero_event[2] = { "USB_STATE=REZEROCMD", NULL };
	char *showcdrom_event[2] = { "USB_STATE=SHOWCDROMCMD", NULL };
#endif
	char **uevent_envp = NULL;
#ifdef CONFIG_LGE_USB_G_ANDROID_AUTORUN
	char **uevent_envp_cdrom = NULL;
#endif
	unsigned long flags;
	/* Add for HW/SW connect */
	bool is_hwconnected = true;

	/* patch for ALPS00345130, if the disconnect followed by hw_disconnect, then the hw_disconnect
	will not notify the UsbDeviceManager due to that musb->g.speed == USB_SPEED_UNKNOWN*/
	if (!cdev){
		return ;
	}

	if (usb_cable_connected())
		is_hwconnected = true;
	else
		is_hwconnected = false;

	pr_notice("[USB]%s: is_hwconnected=%d \n", __func__, is_hwconnected);
	/* Add for HW/SW connect */

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config) {
		uevent_envp = configured;
	} else if (dev->connected != dev->sw_connected) {
		uevent_envp = dev->connected ? connected : disconnected;
	}

	dev->sw_connected = dev->connected;

#ifdef CONFIG_LGE_USB_G_ANDROID_AUTORUN
	if (dev->rezero_cmd == 1) {
		uevent_envp_cdrom = rezero_event;
		dev->rezero_cmd = 0;
	} else if (dev->rezero_cmd == 2) {
		uevent_envp_cdrom = showcdrom_event;
		dev->rezero_cmd = 0;
	}

	//ALPS01329459 - remove disconnect! use HW disconnect to replace disconnect
	if (uevent_envp == disconnected){
		uevent_envp = NULL;
	}
	//ALPS01329459
#endif
	spin_unlock_irqrestore(&cdev->lock, flags);

	if (uevent_envp) {
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp);
		pr_notice("[USB]%s: sent uevent %s\n", __func__, uevent_envp[0]);
	} else {
		pr_notice("[USB]%s: did not send uevent (%d %d %p)\n", __func__,
			 dev->connected, dev->sw_connected, cdev->config);
	}

	if (!is_hwconnected) {
#if 0
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, hwdisconnected);
		pr_notice("[USB]%s: sent uevent %s\n", __func__, hwdisconnected[0]);
#else
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, disconnected);
		pr_notice("[USB]%s: sent uevent %s\n", __func__, disconnected[0]);
#endif
	}
/* LGE_CHANGE_S : USB Autorun function.sangchul1.park@lge.com */
#ifdef CONFIG_LGE_USB_G_ANDROID_AUTORUN
	if (uevent_envp_cdrom) {
		kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE, uevent_envp_cdrom);
		pr_notice("[USB]%s: sent uevent %s\n", __func__, uevent_envp_cdrom[0]);
	} else {
		pr_notice("[USB]%s: did not send zero uevent\n", __func__);
	}
#endif
/* LGE_CHANGE_E : USB Autorun function. */
}

static void android_enable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_configuration *conf;
#endif

	if (WARN_ON(!dev->disable_depth))
		return;

	if (--dev->disable_depth == 0) {
#ifdef CONFIG_LGE_USB_G_ANDROID
		list_for_each_entry(conf, &dev->configs, list_item) {
				usb_add_config(cdev, &conf->usb_config,
					android_bind_config);
		}
#else
		usb_add_config(cdev, &android_config_driver,
					android_bind_config);
#endif
		usb_gadget_connect(cdev->gadget);
	}
}

static void android_disable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_configuration *conf;
#endif

#ifdef CONFIG_LGE_USB_G_ANDROID
	if (dev->lock == USB_FACTORY) {
		pr_info("%s: pif cable is plugged, not permitted\n", __func__);
		return;
	}
#endif
	if (dev->disable_depth++ == 0) {
		usb_gadget_disconnect(cdev->gadget);
		/* Cancel pending control requests */
		usb_ep_dequeue(cdev->gadget->ep0, cdev->req);
#ifdef CONFIG_LGE_USB_G_ANDROID
		list_for_each_entry(conf, &dev->configs, list_item)
			usb_remove_config(cdev, &conf->usb_config);
#else
		usb_remove_config(cdev, &android_config_driver);
#endif
	}
}

/*-------------------------------------------------------------------------*/
/* Supported functions initialization */
struct functionfs_config {
	bool opened;
	bool enabled;
	struct ffs_data *data;
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev;
#endif
};

static int ffs_function_init(struct android_usb_function *f,
			     struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct functionfs_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return functionfs_init();
}

static void ffs_function_cleanup(struct android_usb_function *f)
{
	functionfs_cleanup();
	kfree(f->config);
}

static void ffs_function_enable(struct android_usb_function *f)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = f->android_dev;
#else
	struct android_dev *dev = _android_dev;
#endif
	struct functionfs_config *config = f->config;

#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
	if (config->enabled)
		return;
#endif
	config->enabled = true;

	/* Disable the gadget until the function is ready */
	if (!config->opened)
		android_disable(dev);
}

static void ffs_function_disable(struct android_usb_function *f)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = f->android_dev;
#else
	struct android_dev *dev = _android_dev;
#endif
	struct functionfs_config *config = f->config;

#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
	if (!config->enabled)
		return;
#endif
	config->enabled = false;

	/* Balance the disable that was called in closed_callback */
	if (!config->opened)
		android_enable(dev);
}

static int ffs_function_bind_config(struct android_usb_function *f,
				    struct usb_configuration *c)
{
	struct functionfs_config *config = f->config;
	return functionfs_bind_config(c->cdev, c, config->data);
}

static ssize_t
ffs_aliases_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev;
#else
	struct android_dev *dev = _android_dev;
#endif
	int ret;
#ifdef CONFIG_LGE_USB_G_ANDROID
	dev = list_first_entry(&android_dev_list, struct android_dev,
					list_item);
#endif

	mutex_lock(&dev->mutex);
	ret = sprintf(buf, "%s\n", dev->ffs_aliases);
	mutex_unlock(&dev->mutex);

	return ret;
}

static ssize_t
ffs_aliases_store(struct device *pdev, struct device_attribute *attr,
					const char *buf, size_t size)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev;
#else
	struct android_dev *dev = _android_dev;
#endif
	char buff[256];

#ifdef CONFIG_LGE_USB_G_ANDROID
	dev = list_first_entry(&android_dev_list, struct android_dev,
					list_item);
#endif
	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	strlcpy(buff, buf, sizeof(buff));
	strlcpy(dev->ffs_aliases, strim(buff), sizeof(dev->ffs_aliases));

	mutex_unlock(&dev->mutex);

	return size;
}

static DEVICE_ATTR(aliases, S_IRUGO | S_IWUSR, ffs_aliases_show,
					       ffs_aliases_store);
static struct device_attribute *ffs_function_attributes[] = {
	&dev_attr_aliases,
	NULL
};

static struct android_usb_function ffs_function = {
	.name		= "ffs",
	.init		= ffs_function_init,
	.enable		= ffs_function_enable,
	.disable	= ffs_function_disable,
	.cleanup	= ffs_function_cleanup,
	.bind_config	= ffs_function_bind_config,
	.attributes	= ffs_function_attributes,
};

static int functionfs_ready_callback(struct ffs_data *ffs)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = ffs_function.android_dev;
#else
	struct android_dev *dev = _android_dev;
#endif
	struct functionfs_config *config = ffs_function.config;
	int ret = 0;

	if (dev) {
	mutex_lock(&dev->mutex);

	ret = functionfs_bind(ffs, dev->cdev);
		if (ret) {
			mutex_unlock(&dev->mutex);
			return ret;
		}
	} else {
		/* android ffs_func requires daemon to start only after enable*/
		pr_debug("start adbd only in ADB composition\n");
		return -ENODEV;
	}
	config->data = ffs;
	config->opened = true;
	/* Save dev in case the adb function will get disabled */
	config->dev = dev;

	if (config->enabled)
		android_enable(dev);

	mutex_unlock(&dev->mutex);

	return 0;
}

static void functionfs_closed_callback(struct ffs_data *ffs)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = ffs_function.android_dev;
#else
	struct android_dev *dev = _android_dev;
#endif
	struct functionfs_config *config = ffs_function.config;

	/*
	 * In case new composition is without ADB or ADB got disabled by the
	 * time ffs_daemon was stopped then use saved one
	 */
	if (!dev)
		dev = config->dev;

	/* fatal-error: It should never happen */
	if (!dev)
		pr_err("adb_closed_callback: config->dev is NULL");

	if (dev)
		mutex_lock(&dev->mutex);

	if (config->enabled && dev)
		android_disable(dev);

	config->dev = NULL;

	config->opened = false;
	config->data = NULL;

	functionfs_unbind(ffs);

	if (dev)
	mutex_unlock(&dev->mutex);
}

static void *functionfs_acquire_dev_callback(const char *dev_name)
{
	return 0;
}

static void functionfs_release_dev_callback(struct ffs_data *ffs_data)
{
}

struct adb_data {
	bool opened;
	bool enabled;
};

static int
adb_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct adb_data), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
	kfree(f->config);
}

static int
adb_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return adb_bind_config(c);
}

static void adb_android_function_enable(struct android_usb_function *f)
{
/* This patch cause WHQL fail */
#if 0
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = true;

	/* Disable the gadget until adbd is ready */
	if (!data->opened)
		android_disable(dev);
#endif
}

static void adb_android_function_disable(struct android_usb_function *f)
{
/* This patch cause WHQL fail */
#if 0
	struct android_dev *dev = _android_dev;
	struct adb_data *data = f->config;

	data->enabled = false;

	/* Balance the disable that was called in closed_callback */
	if (!data->opened)
		android_enable(dev);
#endif
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.enable		= adb_android_function_enable,
	.disable	= adb_android_function_disable,
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};

static void adb_ready_callback(void)
{
/* This patch cause WHQL fail */
#if 0
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	mutex_lock(&dev->mutex);

	data->opened = true;

	if (data->enabled)
		android_enable(dev);

	mutex_unlock(&dev->mutex);
#endif
}

static void adb_closed_callback(void)
{
/* This patch cause WHQL fail */
#if 0
	struct android_dev *dev = _android_dev;
	struct adb_data *data = adb_function.config;

	mutex_lock(&dev->mutex);

	data->opened = false;

	if (data->enabled)
		android_disable(dev);

	mutex_unlock(&dev->mutex);
#endif
}

#define MAX_ACM_INSTANCES 4
struct acm_function_config {
	int instances;
	int instances_on;
	struct usb_function *f_acm[MAX_ACM_INSTANCES];
	struct usb_function_instance *f_acm_inst[MAX_ACM_INSTANCES];
	int port_index[MAX_ACM_INSTANCES];
	int port_index_on[MAX_ACM_INSTANCES];
};

static int
acm_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	int i;
	int ret;
	struct acm_function_config *config;
#ifdef CONFIG_USBIF_COMPLIANCE
	/* FIXME, USB_IF workaround */
	return 0;
#endif

	config = kzalloc(sizeof(struct acm_function_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	f->config = config;

	for (i = 0; i < MAX_ACM_INSTANCES; i++) {
		config->f_acm_inst[i] = usb_get_function_instance("acm");
		if (IS_ERR(config->f_acm_inst[i])) {
			ret = PTR_ERR(config->f_acm_inst[i]);
			goto err_usb_get_function_instance;
		}
		config->f_acm[i] = usb_get_function(config->f_acm_inst[i]);
		if (IS_ERR(config->f_acm[i])) {
			ret = PTR_ERR(config->f_acm[i]);
			goto err_usb_get_function;
		}
	}
	return 0;
err_usb_get_function_instance:
	pr_err("Could not usb_get_function_instance() %d\n", i);
	while (i-- > 0) {
		usb_put_function(config->f_acm[i]);
err_usb_get_function:
		pr_err("Could not usb_get_function() %d\n", i);
		usb_put_function_instance(config->f_acm_inst[i]);
	}
	return ret;
}

static void acm_function_cleanup(struct android_usb_function *f)
{
	int i;
	struct acm_function_config *config = f->config;
#ifdef CONFIG_USBIF_COMPLIANCE
	/* FIXME, USB_IF workaround */
	return 0;
#endif

	for (i = 0; i < MAX_ACM_INSTANCES; i++) {
		usb_put_function(config->f_acm[i]);
		usb_put_function_instance(config->f_acm_inst[i]);
	}
	kfree(f->config);
	f->config = NULL;
}

static int
acm_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int i;
	int ret = 0;
	struct acm_function_config *config = f->config;

	/*1st:Modem, 2nd:Modem, 3rd:BT, 4th:MD logger*/
	for (i = 0; i < MAX_ACM_INSTANCES; i++) {
		if(config->port_index[i] != 0) {
			ret = usb_add_function(c, config->f_acm[i]);
			if (ret) {
				pr_err("Could not bind acm%u config\n", i);
				goto err_usb_add_function;
			}
			pr_info("%s Open /dev/ttyGS%d\n", __func__, i);
			config->port_index[i] = 0;
			config->port_index_on[i] = 1;
			config->instances = 0;
		}
	}

	config->instances_on = config->instances;
	for (i = 0; i < config->instances_on; i++) {
		ret = usb_add_function(c, config->f_acm[i]);
		if (ret) {
			pr_err("Could not bind acm%u config\n", i);
			goto err_usb_add_function;
		}
		pr_info("%s Open /dev/ttyGS%d\n", __func__, i);
	}

	return 0;

err_usb_add_function:
	while (i-- > 0)
		usb_remove_function(c, config->f_acm[i]);
	return ret;
}

static void acm_function_unbind_config(struct android_usb_function *f,
				       struct usb_configuration *c)
{

	//int i;
	//struct acm_function_config *config = f->config;

	/* REVISIT:
	 * list_del(&f->list) and f->unbind() are called at unbind_config().
	 * f->disable() is called at enable_store.
	 * So directly return this function.
	 * If does NOT return, f->list is deleted twice and cuased KE.
	 * ToDo:
	 * What does the original kernel code look likes?
	 */

	return;

/*
	if (config->instances_on != 0) {
		for (i = 0; i < config->instances_on; i++) {
			pr_notice("%s f_acm[%d]=%p\n", __func__, i, config->f_acm[i]);
			usb_remove_function(c, config->f_acm[i]);
		}
	} else {
		for (i = 0; i < MAX_ACM_INSTANCES; i++)
			pr_notice("%s port_index_on=%d\n", __func__, config->port_index_on[i]);
			if (config->port_index_on[i] != 0) {
				usb_remove_function(c, config->f_acm[i]);
			}
	}
*/
}

static ssize_t acm_instances_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->instances);
}

static ssize_t acm_instances_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	int value;

	sscanf(buf, "%d", &value);
	if (value > MAX_ACM_INSTANCES)
		value = MAX_ACM_INSTANCES;
	config->instances = value;
	return size;
}

static DEVICE_ATTR(instances, S_IRUGO | S_IWUSR, acm_instances_show,
						 acm_instances_store);

static ssize_t acm_port_index_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	return sprintf(buf, "%d,%d,%d,%d\n", config->port_index[0], config->port_index[1],
		config->port_index[2], config->port_index[3]);
}

static ssize_t acm_port_index_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct acm_function_config *config = f->config;
	int val[MAX_ACM_INSTANCES]={0};
	int num = 0;
	int tmp = 0;

	num = sscanf(buf, "%d,%d,%d,%d", &(val[0]), &(val[1]), &(val[2]), &(val[3]));

	pr_notice("%s [0]=%d,[1]=%d,[2]=%d,[3]=%d, num=%d\n", __func__, val[0], val[1], \
								val[2], val[3], num);

	/* Set all port_index as 0*/
	for (tmp = 0; tmp < MAX_ACM_INSTANCES; tmp ++)
		config->port_index[tmp] = 0;

	for (tmp = 0; tmp < num; tmp++) {
		int port = (val[tmp] > MAX_ACM_INSTANCES || val[tmp] < 1) ? 0 : val[tmp]-1;
		config->port_index[port] = 1;
	}

	return size;
}

static DEVICE_ATTR(port_index, S_IRUGO | S_IWUSR, acm_port_index_show,
						 acm_port_index_store);

static struct device_attribute *acm_function_attributes[] = {
	&dev_attr_instances,
	&dev_attr_port_index, /*Only open the specific port*/
	NULL
};

static struct android_usb_function acm_function = {
	.name		= "acm",
	.init		= acm_function_init,
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.unbind_config	= acm_function_unbind_config,
	.attributes	= acm_function_attributes,
};

#ifdef CONFIG_USB_F_LOOPBACK

#define MAX_LOOPBACK_INSTANCES 1

struct loopback_function_config {
	int port_num;
	struct usb_function *f_lp[MAX_LOOPBACK_INSTANCES];
	struct usb_function_instance *f_lp_inst[MAX_LOOPBACK_INSTANCES];
};

static int loopback_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	int i;
	int ret;
	struct loopback_function_config *config;

	config = kzalloc(sizeof(struct loopback_function_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	f->config = config;

	for (i = 0; i < MAX_LOOPBACK_INSTANCES; i++) {
		config->f_lp_inst[i] = usb_get_function_instance("loopback");
		if (IS_ERR(config->f_lp_inst[i])) {
			ret = PTR_ERR(config->f_lp_inst[i]);
			goto err_usb_get_function_instance;
		}
		config->f_lp[i] = usb_get_function(config->f_lp_inst[i]);
		if (IS_ERR(config->f_lp[i])) {
			ret = PTR_ERR(config->f_lp[i]);
			goto err_usb_get_function;
		}
	}
	return 0;
err_usb_get_function_instance:
	pr_err("Could not usb_get_function_instance() %d\n", i);
	while (i-- > 0) {
		usb_put_function(config->f_lp[i]);
err_usb_get_function:
		pr_err("Could not usb_get_function() %d\n", i);
		usb_put_function_instance(config->f_lp_inst[i]);
	}
	return ret;
}

static void loopback_function_cleanup(struct android_usb_function *f)
{
	int i;
	struct loopback_function_config *config = f->config;

	for (i = 0; i < MAX_LOOPBACK_INSTANCES; i++) {
		usb_put_function(config->f_lp[i]);
		usb_put_function_instance(config->f_lp_inst[i]);
	}
	kfree(f->config);
	f->config = NULL;
}

static int
loopback_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	int ret = 0;
	struct loopback_function_config *config = f->config;

	ret = usb_add_function(c, config->f_lp[config->port_num]);
	if (ret) {
		pr_err("Could not bind loopback%u config\n", config->port_num);
		goto err_usb_add_function;
	}
	pr_info("%s Open loopback\n", __func__);

	return 0;

err_usb_add_function:
	usb_remove_function(c, config->f_lp[config->port_num]);
	return ret;
}

static struct android_usb_function loopback_function = {
	.name		= "loopback",
	.init		= loopback_function_init,
	.cleanup	= loopback_function_cleanup,
	.bind_config	= loopback_function_bind_config,
};
#endif

#define MAX_SERIAL_INSTANCES 4

struct serial_function_config {
	int port_num;
	struct usb_function *f_ser[MAX_SERIAL_INSTANCES];
	struct usb_function_instance *f_ser_inst[MAX_SERIAL_INSTANCES];
};

static int serial_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	int i;
	int ret;
	struct serial_function_config *config;
#ifdef CONFIG_USBIF_COMPLIANCE
	/* FIXME, USB_IF workaround */
	return 0;
#endif

	config = kzalloc(sizeof(struct serial_function_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	f->config = config;

	for (i = 0; i < MAX_SERIAL_INSTANCES; i++) {
		config->f_ser_inst[i] = usb_get_function_instance("gser");
		if (IS_ERR(config->f_ser_inst[i])) {
			ret = PTR_ERR(config->f_ser_inst[i]);
			goto err_usb_get_function_instance;
		}
		config->f_ser[i] = usb_get_function(config->f_ser_inst[i]);
		if (IS_ERR(config->f_ser[i])) {
			ret = PTR_ERR(config->f_ser[i]);
			goto err_usb_get_function;
		}
	}
		return 0;
err_usb_get_function_instance:
	pr_err("Could not usb_get_function_instance() %d\n", i);
	while (i-- > 0) {
		usb_put_function(config->f_ser[i]);
err_usb_get_function:
		pr_err("Could not usb_get_function() %d\n", i);
		usb_put_function_instance(config->f_ser_inst[i]);
	}
	return ret;
}

static void serial_function_cleanup(struct android_usb_function *f)
{
	int i;
	struct serial_function_config *config = f->config;
#ifdef CONFIG_USBIF_COMPLIANCE
	/* FIXME, USB_IF workaround */
	return 0;
#endif

	for (i = 0; i < MAX_SERIAL_INSTANCES; i++) {
		usb_put_function(config->f_ser[i]);
		usb_put_function_instance(config->f_ser_inst[i]);
	}
	kfree(f->config);
	f->config = NULL;
}

static int
serial_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	int ret = 0;
	struct serial_function_config *config = f->config;

	ret = usb_add_function(c, config->f_ser[config->port_num]);
	if (ret) {
		pr_err("Could not bind ser%u config\n", config->port_num);
		goto err_usb_add_function;
	}
	pr_info("%s Open /dev/ttyGS%d\n", __func__, config->port_num);

	return 0;

err_usb_add_function:
	usb_remove_function(c, config->f_ser[config->port_num]);
	return ret;
}

static ssize_t serial_port_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct serial_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->port_num);
}

static ssize_t serial_port_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct serial_function_config *config = f->config;
	int value;

	sscanf(buf, "%d", &value);
	if (value > MAX_SERIAL_INSTANCES)
		value = MAX_SERIAL_INSTANCES;
	config->port_num = value;
	return size;
}

static DEVICE_ATTR(port, S_IRUGO | S_IWUSR, serial_port_show, serial_port_store);
static struct device_attribute *serial_function_attributes[] = { &dev_attr_port, NULL };

static struct android_usb_function serial_function = {
	.name		= "gser",
	.init		= serial_function_init,
	.cleanup	= serial_function_cleanup,
	.bind_config	= serial_function_bind_config,
	.attributes	= serial_function_attributes,
};

static int
mtp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int
mtp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int
ptp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	/* nothing to do - initialization is handled by mtp_function_init */
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	/* nothing to do - cleanup is handled by mtp_function_cleanup */
}

static int
ptp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
					struct usb_composite_dev *cdev,
					const struct usb_ctrlrequest *c)
{
	/* MTP MSFT OS Descriptor */
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev		*dev = f->android_dev;
	struct android_usb_function_holder *f_holder;
	struct android_configuration *conf;
#else
	struct android_dev		*dev = _android_dev;
	struct android_usb_function	*f_count;
#endif
	int	   functions_no=0;
	char   usb_function_string[32];
	char * buff = usb_function_string;

#ifdef CONFIG_LGE_USB_G_ANDROID
	list_for_each_entry(conf, &dev->configs, list_item) {
		if (buff != usb_function_string)
			*(buff-1) = ':';
		list_for_each_entry(f_holder, &conf->enabled_functions,
					enabled_list) {
			functions_no++;
			buff += snprintf(buff, PAGE_SIZE, "%s,",
					f_holder->f->name);
		}
	}
#else
	list_for_each_entry(f_count, &dev->enabled_functions, enabled_list)
	{
		functions_no++;
		buff += sprintf(buff, "%s,", f_count->name);
	}
#endif
	*(buff-1) = '\n';

	mtp_read_usb_functions(functions_no, usb_function_string);
	return mtp_ctrlrequest(cdev, c);
}

static int ptp_function_ctrlrequest(struct android_usb_function *f,
					struct usb_composite_dev *cdev,
					const struct usb_ctrlrequest *c)
{
	return ptp_ctrlrequest(cdev, c);
}


static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
};

/* PTP function is same as MTP with slightly different interface descriptor */
static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
	.ctrlrequest	= ptp_function_ctrlrequest, //ALPS01832160
};

struct ecm_function_config {
	u8      ethaddr[ETH_ALEN];
	struct eth_dev *dev;
};

static int ecm_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct ecm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void ecm_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
ecm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct eth_dev *dev;
	struct ecm_function_config *ecm = f->config;
#ifdef CONFIG_LGE_USB_G_ANDROID
    int i, len;
#endif

	if (!ecm) {
		pr_err("%s: ecm_pdata\n", __func__);
		return -1;
	}

#ifdef CONFIG_LGE_USB_G_ANDROID
   /*
    * generate ethadd by serial_string
    */
    memset(ecm->ethaddr, 0, ETH_ALEN);
    ecm->ethaddr[0] = 0x02;
    len = strlen(serial_string);
    for (i = 0; i < len; i++) {
        ecm->ethaddr[i % (ETH_ALEN - 1) + 1] ^= serial_string[i];
    }
#endif

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);

#ifdef CONFIG_LGE_USB_G_ANDROID
	if (ecm->ethaddr[0])
		dev = gether_setup_name(c->cdev->gadget, NULL, "usb");
	else
		dev = gether_setup_name(c->cdev->gadget, ecm->ethaddr, "usb");
#else
	dev = gether_setup_name(c->cdev->gadget, ecm->ethaddr, "rndis");
#endif
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}
	ecm->dev = dev;

#ifdef CONFIG_LGE_USB_G_ANDROID
	ret = ecm_bind_config(c, ecm->ethaddr, dev);
	if (ret) {
		pr_err("%s: ecm_bind_config failed\n", __func__);
		gether_cleanup(dev);
	}
	return ret;
#else
	return ecm_bind_config(c, ecm->ethaddr, ecm->dev);
#endif
}

static void ecm_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct ecm_function_config *ecm = f->config;
	gether_cleanup(ecm->dev);
}

static ssize_t ecm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);
}

static ssize_t ecm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&ecm->ethaddr[0], (int *)&ecm->ethaddr[1],
		    (int *)&ecm->ethaddr[2], (int *)&ecm->ethaddr[3],
		    (int *)&ecm->ethaddr[4], (int *)&ecm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ecm_ethaddr, S_IRUGO | S_IWUSR, ecm_ethaddr_show,
					       ecm_ethaddr_store);

static struct device_attribute *ecm_function_attributes[] = {
	&dev_attr_ecm_ethaddr,
	NULL
};

static struct android_usb_function ecm_function = {
	.name		= "ecm",
	.init		= ecm_function_init,
	.cleanup	= ecm_function_cleanup,
	.bind_config	= ecm_function_bind_config,
	.unbind_config	= ecm_function_unbind_config,
	.attributes	= ecm_function_attributes,
};

struct eem_function_config {
	u8      ethaddr[ETH_ALEN];
	char	manufacturer[256];
	struct eth_dev *dev;
};

static int
eem_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct eem_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void eem_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
eem_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int ret;
	struct eth_dev *dev;
	struct eem_function_config *eem = f->config;

	pr_notice("[USB]%s: \n", __func__);

	if (!eem) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		eem->ethaddr[0], eem->ethaddr[1], eem->ethaddr[2],
		eem->ethaddr[3], eem->ethaddr[4], eem->ethaddr[5]);

	dev = gether_setup_name(c->cdev->gadget, eem->ethaddr, "rndis");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}
	eem->dev = dev;

	return eem_bind_config(c, eem->dev);
}

static void eem_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct eem_function_config *eem = f->config;
	gether_cleanup(eem->dev);
}

static ssize_t eem_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct eem_function_config *eem = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		eem->ethaddr[0], eem->ethaddr[1], eem->ethaddr[2],
		eem->ethaddr[3], eem->ethaddr[4], eem->ethaddr[5]);
}

static ssize_t eem_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct eem_function_config *eem = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&eem->ethaddr[0], (int *)&eem->ethaddr[1],
		    (int *)&eem->ethaddr[2], (int *)&eem->ethaddr[3],
		    (int *)&eem->ethaddr[4], (int *)&eem->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(eem_ethaddr, S_IRUGO | S_IWUSR, eem_ethaddr_show,
					       eem_ethaddr_store);

static struct device_attribute *eem_function_attributes[] = {
	&dev_attr_eem_ethaddr,
	NULL
};

static struct android_usb_function eem_function = {
	.name		= "eem",
	.init		= eem_function_init,
	.cleanup	= eem_function_cleanup,
	.bind_config	= eem_function_bind_config,
	.unbind_config	= eem_function_unbind_config,
	.attributes	= eem_function_attributes,
};

struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	/* "Wireless" RNDIS; auto-detected by Windows */
	bool	wceis;
	struct eth_dev *dev;
};

static int
rndis_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int
rndis_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int ret;
	struct eth_dev *dev;
	struct rndis_function_config *rndis = f->config;

	pr_notice("[USB]%s: \n", __func__);

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	if (rndis->ethaddr[0])
		dev = gether_setup_name(c->cdev->gadget, NULL, "rndis");
	else
	dev = gether_setup_name(c->cdev->gadget, rndis->ethaddr, "rndis");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}
	rndis->dev = dev;

	if (rndis->wceis) {
		/* "Wireless" RNDIS; auto-detected by Windows */
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_bind_config_vendor(c, rndis->ethaddr, rndis->vendorID,
					   rndis->manufacturer, rndis->dev);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct rndis_function_config *rndis = f->config;
	gether_cleanup(rndis->dev);
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;
	if (sscanf(buf, "%s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;
	return sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return sprintf(buf, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
};


struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;
	int i;

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

#ifdef CONFIG_MTK_MULTI_STORAGE_SUPPORT
#define LUN_MULTI (1)
#else
#define LUN_MULTI (0)
#endif

#ifdef CONFIG_MTK_SHARED_SDCARD
#define LUN_SHARED_SD (-1)
#else
#define LUN_SHARED_SD (0)
#endif

#ifdef CONFIG_MTK_ICUSB_SUPPORT
#define LUN_ICUSB (1)
#else
#define LUN_ICUSB (0)
#endif

#define LUN_NUM LUN_MULTI + LUN_SHARED_SD + LUN_ICUSB + 1

	config->fsg.nluns = LUN_NUM;

	for(i = 0; i < config->fsg.nluns; i++) {
		config->fsg.luns[i].removable = 1;
		config->fsg.luns[i].nofua = 1;
	}

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	err = sysfs_create_link(&f->dev->kobj,
				&common->luns[0].dev.kobj,
				"lun");
	if (err) {
		kfree(config);
		return err;
	}

	/*
	 * "i" starts from "1", cuz dont want to change the naming of
	 * the original path of "lun0".
	 */
	for(i = 1; i < config->fsg.nluns; i++) {
		char string_lun[5]={0};

		sprintf(string_lun, "lun%d",i);

		err = sysfs_create_link(&f->dev->kobj,
				&common->luns[i].dev.kobj,
				string_lun);
		if (err) {
			kfree(config);
			return err;
		}
	}

	config->common = common;
	f->config = config;
	return 0;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return sprintf(buf, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

#ifdef CONFIG_MTK_BICR_SUPPORT

static ssize_t mass_storage_bicr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return sprintf(buf, "%d\n", config->common->bicr);
}

static ssize_t mass_storage_bicr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->bicr))
		return -EINVAL;
	if (sscanf(buf, "%d", &config->common->bicr) != 1)
		return -EINVAL;

	/* Set Lun[0] is a CDROM when enable bicr.*/
	if (!strcmp(buf, "1"))
		config->common->luns[0].cdrom = 1;
	else {
		/*Reset the value. Clean the cdrom's parameters*/
		config->common->luns[0].cdrom = 0;
		config->common->luns[0].blkbits = 0;
		config->common->luns[0].blksize = 0;
		config->common->luns[0].num_sectors = 0;
	}

	return size;
}

static DEVICE_ATTR(bicr, S_IRUGO | S_IWUSR,
					mass_storage_bicr_show,
					mass_storage_bicr_store);

#endif

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
#ifdef CONFIG_MTK_BICR_SUPPORT
	&dev_attr_bicr,
#endif
	NULL
};

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};

#ifdef CONFIG_LGE_USB_G_ANDROID_AUTORUN
struct cdrom_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static const char cdrom_storage_kthread_name[] = "kcdrom-storaged";
static const char cdrom_lun_format[] = "clun%d";

static int cdrom_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct cdrom_storage_function_config *config;
	struct fsg_common *common;
	int err;

	config = kzalloc(sizeof(struct cdrom_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	config->fsg.vendor_name = "LGE";
	config->fsg.product_name = "CDROM storage";

	config->fsg.nluns = 1;
	config->fsg.luns[0].removable = 1;
	config->fsg.luns[0].cdrom = 1;
	config->fsg.thread_name = cdrom_storage_kthread_name;
	config->fsg.lun_name_format = cdrom_lun_format;

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	err = sysfs_create_link(&f->dev->kobj,
				&common->luns[0].dev.kobj,
				"lun");
	if (err) {
		fsg_common_release(&common->ref);
		kfree(config);
		return err;
	}

	config->common = common;
	f->config = config;
	return 0;
}

static void cdrom_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int cdrom_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct cdrom_storage_function_config *config = f->config;
	return csg_bind_config(c->cdev, c, config->common);
}

static ssize_t cdrom_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct cdrom_storage_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->common->inquiry_string);
}

static ssize_t cdrom_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct cdrom_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (strlcpy(config->common->inquiry_string, buf,
				sizeof(config->common->inquiry_string)) == 0)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(cdrom_inquiry_string, S_IRUGO | S_IWUSR,
					cdrom_storage_inquiry_show,
					cdrom_storage_inquiry_store);

static struct device_attribute *cdrom_storage_function_attributes[] = {
	&dev_attr_cdrom_inquiry_string,
	NULL
};

static struct android_usb_function cdrom_storage_function = {
	.name		= "cdrom_storage",
	.init		= cdrom_storage_function_init,
	.cleanup	= cdrom_storage_function_cleanup,
	.bind_config	= cdrom_storage_function_bind_config,
	.attributes	= cdrom_storage_function_attributes,
};
#endif

#ifdef CONFIG_LGE_USB_G_ANDROID
/* charge only mode */
static int charge_only_function_init(struct android_usb_function *f, struct usb_composite_dev *cdev)
{
	return charge_only_setup();
}

static void charge_only_function_cleanup(struct android_usb_function *f)
{
	charge_only_cleanup();
}

static int charge_only_function_bind_config(struct android_usb_function *f, struct usb_configuration *c)
{
	return charge_only_bind_config(c);
}

static struct android_usb_function charge_only_function = {
	.name		= "charge_only",
	.init		= charge_only_function_init,
	.cleanup	= charge_only_function_cleanup,
	.bind_config	= charge_only_function_bind_config,
};
#endif

static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};

static int audio_source_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	struct audio_source_config *config;

	config = kzalloc(sizeof(struct audio_source_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	f->config = config;
	return 0;
}

static void audio_source_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
}

static int audio_source_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	return audio_source_bind_config(c, config);
}

static void audio_source_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	config->card = -1;
	config->device = -1;
}

static ssize_t audio_source_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;

	/* print PCM card and device numbers */
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO, audio_source_pcm_show, NULL);

static struct device_attribute *audio_source_function_attributes[] = {
	&dev_attr_pcm,
	NULL
};

static struct android_usb_function audio_source_function = {
	.name		= "audio_source",
	.init		= audio_source_function_init,
	.cleanup	= audio_source_function_cleanup,
	.bind_config	= audio_source_function_bind_config,
	.unbind_config	= audio_source_function_unbind_config,
	.attributes	= audio_source_function_attributes,
};

#ifndef CONFIG_MTK_ECCCI_C2K
#ifdef CONFIG_MTK_C2K_SUPPORT
static int rawbulk_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return 0;
}

static void rawbulk_function_cleanup(struct android_usb_function *f)
{
	;
}

static int rawbulk_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
    char *i = f->name + strlen("via_");
    if (!strncmp(i, "modem", 5))
        return rawbulk_bind_config(c, RAWBULK_TID_MODEM);
    else if (!strncmp(i, "ets", 3))
        return rawbulk_bind_config(c, RAWBULK_TID_ETS);
    else if (!strncmp(i, "atc", 3))
        return rawbulk_bind_config(c, RAWBULK_TID_AT);
    else if (!strncmp(i, "pcv", 3))
        return rawbulk_bind_config(c, RAWBULK_TID_PCV);
    else if (!strncmp(i, "gps", 3))
        return rawbulk_bind_config(c, RAWBULK_TID_GPS);
    return -EINVAL;
}

static int rawbulk_function_modem_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
    if ((c->bRequestType & USB_RECIP_MASK) == USB_RECIP_DEVICE &&
            (c->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
        struct rawbulk_function *fn = rawbulk_lookup_function(RAWBULK_TID_MODEM);
        return rawbulk_function_setup(&fn->function, c);
    }
    return -1;
}

static struct android_usb_function rawbulk_modem_function = {
	.name		= "via_modem",
	.init		= rawbulk_function_init,
	.cleanup	= rawbulk_function_cleanup,
	.bind_config	= rawbulk_function_bind_config,
	.ctrlrequest	= rawbulk_function_modem_ctrlrequest,
};

static struct android_usb_function rawbulk_ets_function = {
	.name		= "via_ets",
	.init		= rawbulk_function_init,
	.cleanup	= rawbulk_function_cleanup,
	.bind_config	= rawbulk_function_bind_config,
};

static struct android_usb_function rawbulk_atc_function = {
	.name		= "via_atc",
	.init		= rawbulk_function_init,
	.cleanup	= rawbulk_function_cleanup,
	.bind_config	= rawbulk_function_bind_config,
};

static struct android_usb_function rawbulk_pcv_function = {
	.name		= "via_pcv",
	.init		= rawbulk_function_init,
	.cleanup	= rawbulk_function_cleanup,
	.bind_config	= rawbulk_function_bind_config,
};

static struct android_usb_function rawbulk_gps_function = {
	.name		= "via_gps",
	.init		= rawbulk_function_init,
	.cleanup	= rawbulk_function_cleanup,
	.bind_config	= rawbulk_function_bind_config,
};
#endif
#endif

/*-------------------------------------------------------------------------*/
/* Supported functions initialization */

/* laf */
struct laf_data {
    bool opened;
    bool enabled;
};

static int
laf_function_init(struct android_usb_function *f,
        struct usb_composite_dev *cdev)
{
    f->config = kzalloc(sizeof(struct laf_data), GFP_KERNEL);
    if (!f->config)
        return -ENOMEM;

    return laf_setup();
}


static void laf_function_cleanup(struct android_usb_function *f)
{
    laf_cleanup();
    kfree(f->config);
}

static int
laf_function_bind_config(struct android_usb_function *f,
        struct usb_configuration *c)
{
    return laf_bind_config(c);
}

static void laf_android_function_enable(struct android_usb_function *f)
{
    struct laf_data *data = f->config;

    data->enabled = true;

    pr_err("laf_android_function_enable");
}

static void laf_android_function_disable(struct android_usb_function *f)
{
    struct laf_data *data = f->config;

    data->enabled = false;
}

static struct android_usb_function laf_function = {
    .name           = "laf",
    .enable         = laf_android_function_enable,
    .disable        = laf_android_function_disable,
    .init           = laf_function_init,
    .cleanup        = laf_function_cleanup,
    .bind_config    = laf_function_bind_config,
};

static void laf_ready_callback(void)
{
    struct laf_data *data = laf_function.config;

    data->opened = true;
}

static void laf_closed_callback(void)
{
    struct laf_data *data = laf_function.config;

    data->opened = false;
}



#ifdef CONFIG_SND_RAWMIDI
static int midi_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct midi_alsa_config *config;

	config = kzalloc(sizeof(struct midi_alsa_config), GFP_KERNEL);
	f->config = config;
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	return 0;
}

static void midi_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
}

static int midi_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct midi_alsa_config *config = f->config;

	return f_midi_bind_config(c, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			MIDI_INPUT_PORTS, MIDI_OUTPUT_PORTS, MIDI_BUFFER_SIZE,
			MIDI_QUEUE_LENGTH, config);
}

static ssize_t midi_alsa_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct midi_alsa_config *config = f->config;

	/* print ALSA card and device numbers */
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(alsa, S_IRUGO, midi_alsa_show, NULL);

static struct device_attribute *midi_function_attributes[] = {
	&dev_attr_alsa,
	NULL
};

static struct android_usb_function midi_function = {
	.name		= "midi",
	.init		= midi_function_init,
	.cleanup	= midi_function_cleanup,
	.bind_config	= midi_function_bind_config,
	.attributes	= midi_function_attributes,
};
#endif

static struct android_usb_function *supported_functions[] = {
	&ffs_function,
	&adb_function,
	&acm_function,
	&laf_function,
	&mtp_function,
	&ptp_function,
#ifndef CONFIG_USBIF_COMPLIANCE
	&ecm_function,
	&eem_function,
#endif
	&serial_function,
	&rndis_function,
	&mass_storage_function,
#ifdef CONFIG_LGE_USB_G_ANDROID
	&charge_only_function,
#endif
	&accessory_function,
	&audio_source_function,
#ifdef CONFIG_SND_RAWMIDI
	&midi_function,
#endif
#ifdef CONFIG_LGE_USB_G_ANDROID_AUTORUN
    &cdrom_storage_function,
#endif

#ifndef CONFIG_MTK_ECCCI_C2K
#ifdef CONFIG_MTK_C2K_SUPPORT
	&rawbulk_modem_function,
	&rawbulk_ets_function,
	&rawbulk_atc_function,
	&rawbulk_pcv_function,
	&rawbulk_gps_function,
#endif
#endif
#ifdef CONFIG_USB_F_LOOPBACK
	&loopback_function,
#endif
	NULL
};

static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = cdev_to_android_dev(cdev);
#else
	struct android_dev *dev = _android_dev;
#endif
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err;

#ifdef CONFIG_USBIF_COMPLIANCE
	int index = 1;
#else
	int index = 0;
#endif

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		/* Added for USB Develpment debug, more log for more debuging help */
		pr_notice("[USB]%s: f->dev_name = %s, f->name = %s\n", __func__, f->dev_name, f->name);
		/* Added for USB Develpment debug, more log for more debuging help */
#ifdef CONFIG_LGE_USB_G_ANDROID
		f->android_dev = NULL;
#endif
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
#ifdef CONFIG_LGE_USB_G_ANDROID
			f->dev = NULL;
#endif
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_out;
			} else {
				pr_notice("[USB]%s: init %s success!!\n", __func__, f->name);
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_out;
		}
	}
	return 0;

err_out:
	device_destroy(android_class, f->dev->devt);
err_create:
	kfree(f->dev_name);
	return err;
}

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		}

		if (f->cleanup)
			f->cleanup(f);
	}
}

static int
android_bind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_usb_function_holder *f_holder;
	struct android_configuration *conf =
		container_of(c, struct android_configuration, usb_config);
#else
	struct android_usb_function *f;
#endif
	int ret;

	/* Added for USB Develpment debug, more log for more debuging help */
	pr_notice("[USB]%s: ", __func__);
	/* Added for USB Develpment debug, more log for more debuging help */

#ifdef CONFIG_LGE_USB_G_ANDROID
	list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
		pr_notice("[USB]bind_config function '%s'/%p\n", f_holder->f->name, f_holder->f);
		ret = f_holder->f->bind_config(f_holder->f, c);
		if (ret) {
			pr_err("%s: %s failed\n", __func__, f_holder->f->name);
			while (!list_empty(&c->functions)) {
				struct usb_function		*f;

				f = list_first_entry(&c->functions,
					struct usb_function, list);
				if (f->config) {
					list_del(&f->list);
					if (f->unbind)
						f->unbind(c, f);
				}
			}
			if (c->unbind)
				c->unbind(c);
			return ret;
		}
		f_holder->f->bound = true;
#else
	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		pr_notice("[USB]bind_config function '%s'/%p\n", f->name, f);
		ret = f->bind_config(f, c);
		if (ret) {
			pr_err("%s: %s failed", __func__, f->name);
			return ret;
		}
#endif
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_usb_function_holder *f_holder;
	struct android_configuration *conf =
		container_of(c, struct android_configuration, usb_config);

	list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
		pr_notice("[USB]unbind_config function '%s'/%p\n", f_holder->f->name, f_holder->f);
		if (f_holder->f->bound && f_holder->f->unbind_config)
			f_holder->f->unbind_config(f_holder->f, c);
		f_holder->f->bound = false;
	}
#else
	struct android_usb_function *f;

	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		pr_notice("[USB]unbind_config function '%s'/%p\n", f->name, f);
		if (f->unbind_config)
			f->unbind_config(f, c);
	}
#endif
}

#ifdef CONFIG_LGE_USB_G_ANDROID
static int android_enable_function(struct android_dev *dev,
				   struct android_configuration *conf,
				   char *name)
#else
static int android_enable_function(struct android_dev *dev, char *name)
#endif
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_usb_function_holder *f_holder;
#endif
	while ((f = *functions++)) {

		/* Added for USB Develpment debug, more log for more debuging help */
		pr_notice("[USB]%s: name = %s, f->name=%s \n", __func__, name, f->name);
		/* Added for USB Develpment debug, more log for more debuging help */
		if (!strcmp(name, f->name)) {
#ifdef CONFIG_LGE_USB_G_ANDROID
			if (f->android_dev && f->android_dev != dev)
				pr_err("%s is enabled in other device\n",
					f->name);
			else {
				f_holder = kzalloc(sizeof(*f_holder),
						GFP_KERNEL);
				if (!f_holder) {
					pr_err("Failed to alloc f_holder\n");
					return -ENOMEM;
				}

				f->android_dev = dev;
				f_holder->f = f;
				list_add_tail(&f_holder->enabled_list,
					      &conf->enabled_functions);
				pr_notice("func:%s is enabled.\n", f->name);
				return 0;
			}
#else
			list_add_tail(&f->enabled_list,
						&dev->enabled_functions);
			return 0;
#endif
		}
	}
	return -EINVAL;
}

/*-------------------------------------------------------------------------*/
/* /sys/class/android_usb/android%d/ interface */

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_configuration *conf;
	struct android_usb_function_holder *f_holder;
#else
	struct android_usb_function *f;
#endif
	char *buff = buf;

	/* Added for USB Develpment debug, more log for more debuging help */
	pr_notice("[USB]%s: ", __func__);
	/* Added for USB Develpment debug, more log for more debuging help */

	mutex_lock(&dev->mutex);

#ifdef CONFIG_LGE_USB_G_ANDROID
	list_for_each_entry(conf, &dev->configs, list_item) {
		if (buff != buf)
			*(buff-1) = ':';
		list_for_each_entry(f_holder, &conf->enabled_functions,
					enabled_list)
			buff += snprintf(buff, PAGE_SIZE, "%s,",
					f_holder->f->name);
	}
#else
	list_for_each_entry(f, &dev->enabled_functions, enabled_list)
		buff += sprintf(buff, "%s,", f->name);
#endif

	mutex_unlock(&dev->mutex);

	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct list_head *curr_conf = &dev->configs;
	struct android_configuration *conf;
	char *conf_str;
	struct android_usb_function_holder *f_holder;
#endif
	char *name;
	char buf[256], *b;
	char aliases[256], *a;
	int err;
	int is_ffs;
	int ffs_enabled = 0;

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

#ifdef CONFIG_LGE_USB_G_ANDROID
	if (dev->lock == USB_FACTORY)
	{
		pr_info("%s: pif cable is plugged, not permitted\n", __func__);
		mutex_unlock(&dev->mutex);
		return -EPERM;
	}
#endif
#ifdef CONFIG_LGE_USB_G_ANDROID
	pr_notice("[USB]request function list: %s\n", buff);
	/* Clear previous enabled list */
	list_for_each_entry(conf, &dev->configs, list_item) {
		while (conf->enabled_functions.next !=
				&conf->enabled_functions) {
			f_holder = list_entry(conf->enabled_functions.next,
					typeof(*f_holder),
					enabled_list);
			f_holder->f->android_dev = NULL;
			list_del(&f_holder->enabled_list);
			kfree(f_holder);
		}
		INIT_LIST_HEAD(&conf->enabled_functions);
	}
#else
	INIT_LIST_HEAD(&dev->enabled_functions);
#endif

#if 0 //def CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT || get_boot_mode() == LOW_POWER_OFF_CHARGING_BOOT) {
		pr_notice("[USB]KPOC, func%s\n", KPOC_USB_FUNC);
		err = android_enable_function(dev, KPOC_USB_FUNC);
		if (err)
			pr_err("android_usb: Cannot enable '%s' (%d)",
					KPOC_USB_FUNC, err);
		mutex_unlock(&dev->mutex);
		return size;
	}
#endif

	/* Added for USB Develpment debug, more log for more debuging help */
	pr_notice("[USB]%s: \n", __func__);
	/* Added for USB Develpment debug, more log for more debuging help */

	strlcpy(buf, buff, sizeof(buf));
	b = strim(buf);

#ifdef CONFIG_LGE_USB_G_ANDROID
	dev->check_charge_only = false;
	if (!strcmp(b, "charge_only"))
		dev->check_charge_only = true;
#endif

	while (b) {
#ifdef CONFIG_LGE_USB_G_ANDROID
#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
		ffs_enabled = 0;
#endif
		conf_str = strsep(&b, ":");

		if (!conf_str)
			continue;

		/* Added for USB Develpment debug, more log for more debuging help */
		pr_notice("[USB]%s: name = %s \n", __func__, conf_str);
		/* Added for USB Develpment debug, more log for more debuging help */
#else
		name = strsep(&b, ",");

		/* Added for USB Develpment debug, more log for more debuging help */
		pr_notice("[USB]%s: name = %s \n", __func__, name);
		/* Added for USB Develpment debug, more log for more debuging help */

		if (!name)
			continue;
#endif

#ifdef CONFIG_LGE_USB_G_ANDROID
		/* If the next not equal to the head, take it */
		if (curr_conf->next != &dev->configs)
			conf = list_entry(curr_conf->next,
					  struct android_configuration,
					  list_item);
		else
			conf = alloc_android_config(dev);

		curr_conf = curr_conf->next;

		while (conf_str) {
			name = strsep(&conf_str, ",");
			is_ffs = 0;
			strlcpy(aliases, dev->ffs_aliases, sizeof(aliases));
			a = aliases;

			while (a) {
				char *alias = strsep(&a, ",");
				if (alias && !strcmp(name, alias)) {
					is_ffs = 1;
					break;
				}
			}

			if (is_ffs) {
				if (ffs_enabled)
					continue;
				err = android_enable_function(dev, conf, "ffs");
				if (err)
					pr_err("android_usb: Cannot enable ffs (%d)",
										err);
				else
					ffs_enabled = 1;
				continue;
			}

			err = android_enable_function(dev, conf, name);
			if (err)
				pr_err("android_usb: Cannot enable '%s' (%d)",
							name, err);
		}
#else
		is_ffs = 0;
		strlcpy(aliases, dev->ffs_aliases, sizeof(aliases));
		a = aliases;

		while (a) {
			char *alias = strsep(&a, ",");
			if (alias && !strcmp(name, alias)) {
				is_ffs = 1;
				break;
			}
		}

		if (is_ffs) {
			if (ffs_enabled)
				continue;
			err = android_enable_function(dev, "ffs");
			if (err)
				pr_err("android_usb: Cannot enable ffs (%d)",
									err);
			else
				ffs_enabled = 1;
			continue;
		}

		err = android_enable_function(dev, name);
		if (err)
			pr_err("android_usb: Cannot enable '%s' (%d)",
							   name, err);
#endif
	}

#ifdef CONFIG_LGE_USB_G_ANDROID
	/* Free uneeded configurations if exists */
	while (curr_conf->next != &dev->configs) {
		conf = list_entry(curr_conf->next,
				  struct android_configuration, list_item);
		free_android_config(dev, conf);
	}
#endif

	mutex_unlock(&dev->mutex);

	return size;
}

static ssize_t enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return sprintf(buf, "%d\n", dev->enabled);
}

static ssize_t enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_usb_function_holder *f_holder;
	struct android_configuration *conf;
#else
	struct android_usb_function *f;
#endif
	int enabled = 0;


	if (!cdev)
		return -ENODEV;

	mutex_lock(&dev->mutex);

#ifdef CONFIG_LGE_USB_G_ANDROID
	dev_info(dev->dev, "gadget enable(%s)\n", buff);
#endif

	sscanf(buff, "%d", &enabled);
#ifdef CONFIG_LGE_USB_G_ANDROID
	if (dev->lock == USB_FACTORY)
	{
        if (enabled && !dev->enabled) {
            android_enable(dev);
            dev->enabled = true;
            mutex_unlock(&dev->mutex);
            pr_info("%s: pif cable is plugged, enable\n", __func__);
            return size;
        }
//		if(musb->softconnect == 0)
//			musb->softconnect = 1;
		mutex_unlock(&dev->mutex);
		pr_info("%s: pif cable is plugged, not permitted\n", __func__);
		return -EPERM;
	}
#endif
	/* Added for USB Develpment debug, more log for more debuging help */
	pr_notice("[USB]%s: device_attr->attr.name: %s\n", __func__, attr->attr.name);
	/* Added for USB Develpment debug, more log for more debuging help */

	if (enabled && !dev->enabled) {
		/*
		 * Update values in composite driver's copy of
		 * device descriptor.
		 */
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT || get_boot_mode() == LOW_POWER_OFF_CHARGING_BOOT) {
			pr_notice("[USB]KPOC, vid:%d, pid:%d\n", KPOC_USB_VENDOR_ID, KPOC_USB_PRODUCT_ID);
			cdev->desc.idVendor = __constant_cpu_to_le16(KPOC_USB_VENDOR_ID);
			cdev->desc.idProduct = __constant_cpu_to_le16(KPOC_USB_PRODUCT_ID);
		}
#endif
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;

		/* special case for meta mode */
		if (serial_string[0] == 0x0) {
			cdev->desc.iSerialNumber = 0;
#ifdef CONFIG_LGE_USB_G_ANDROID
		} else if (dev->check_charge_only) {
			cdev->desc.iSerialNumber = 0;
			cdev->desc.iProduct =
					strings_dev[CHARGE_ONLY_STRING_IDX].id;
#endif
		} else {
			cdev->desc.iSerialNumber =
					strings_dev[STRING_SERIAL_IDX].id;
			cdev->desc.iProduct =
				strings_dev[STRING_PRODUCT_IDX].id;
		}

#ifdef CONFIG_LGE_USB_G_ANDROID
		list_for_each_entry(conf, &dev->configs, list_item)
			list_for_each_entry(f_holder, &conf->enabled_functions,
						enabled_list) {
				pr_notice("[USB]enable function '%s'/%p\n",f_holder->f->name, f_holder->f);
				if (f_holder->f->enable)
					f_holder->f->enable(f_holder->f);
			}
#else
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			pr_notice("[USB]enable function '%s'/%p\n", f->name, f);
			if (f->enable)
				f->enable(f);
		}
#endif
		android_enable(dev);
		dev->enabled = true;

		/* Added for USB Develpment debug, more log for more debuging help */
		pr_notice("[USB]%s: enable 0->1 case, device_desc.idVendor = 0x%x, device_desc.idProduct = 0x%x\n", __func__, device_desc.idVendor, device_desc.idProduct);
		/* Added for USB Develpment debug, more log for more debuging help */

	} else if (!enabled && dev->enabled) {

		/* Added for USB Develpment debug, more log for more debuging help */
		pr_notice("[USB]%s: enable 1->0 case, device_desc.idVendor = 0x%x, device_desc.idProduct = 0x%x\n", __func__, device_desc.idVendor, device_desc.idProduct);
		/* Added for USB Develpment debug, more log for more debuging help */

		android_disable(dev);
#ifdef CONFIG_LGE_USB_G_ANDROID
		list_for_each_entry(conf, &dev->configs, list_item)
			list_for_each_entry(f_holder, &conf->enabled_functions,
						enabled_list) {
			pr_notice("[USB]disable function '%s'/%p\n", f_holder->f->name, f_holder->f);
				if (f_holder->f->disable)
					f_holder->f->disable(f_holder->f);
			}
#else
		list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
			pr_notice("[USB]disable function '%s'/%p\n", f->name, f);
			if (f->disable) {
				f->disable(f);
			}
		}
#endif
		dev->enabled = false;
	} else {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
		/* Add for HW/SW connect */
		if (!usb_cable_connected()) {
			schedule_work(&dev->work);
			pr_notice("[USB]%s: enable 0->0 case - no usb cable", __func__);
		}
		/* Add for HW/SW connect */
	}
	//ALPS01329459, force schedule work to check if hwdisconnect, and replace with disconnect
	schedule_work(&dev->work);

	mutex_unlock(&dev->mutex);
	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return sprintf(buf, "%s\n", state);
}
#ifdef CONFIG_LGE_USB_G_ANDROID
static ssize_t factory_cable_show(struct device *pdev,
		struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	if (dev->lock >= USB_FACTORY)
		return snprintf(buf, PAGE_SIZE,"%04x\n", 0x6000);

	return snprintf(buf, PAGE_SIZE,"%04x\n", device_desc.idProduct);
}

static DEVICE_ATTR(factory_cable, S_IRUGO, factory_cable_show, NULL);

//LGE_CHANGE_S namgyu.kim 2014.02.10 for debugging mini OS  after loading adb driver
static ssize_t lock_show(struct device *pdev, struct device_attribute *attr,
		char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	return snprintf(buf, PAGE_SIZE, "%d\n", dev->lock);
}

static ssize_t lock_store(struct device *pdev, struct device_attribute *attr,
	const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	int enabled = 0;
	mutex_lock(&dev->mutex);
	sscanf(buff, "%d", &enabled);
	if (enabled)
	{
		dev->lock = USB_FACTORY_1;
		printk("\n[USB] lock_store USB_FACTORY_1\n");
	}
	else
	{
		dev->lock = USB_FACTORY_0;
		printk("\n[USB] lock_store USB_FACTORY_0\n");
	}
	mutex_unlock(&dev->mutex);

	return size;
}
//LGE_CHANGE_E namgyu.kim
static DEVICE_ATTR(lock, S_IRUGO | S_IWUSR, lock_show, lock_store);
#endif

#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
static ssize_t config_num_show(struct device *pdev, struct device_attribute *attr,
		char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	u8 config_num = 0;

	if (dev->cdev->config)
		config_num = dev->cdev->config->bConfigurationValue;

	return snprintf(buf, PAGE_SIZE, "%d\n", config_num);
}
#endif

#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	int value;							\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return sprintf(buf, "%s", buffer);				\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	if (size >= sizeof(buffer))					\
		return -EINVAL;						\
	if (sscanf(buf, "%s", buffer) == 1) {				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);


DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)
#ifdef CONFIG_LGE_USB_G_ANDROID
DESCRIPTOR_ATTR(iSerialNumber, "%d\n")
#endif

static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show,
						 functions_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, enable_show, enable_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
static DEVICE_ATTR(config_num, S_IRUGO, config_num_show, NULL);
#endif

static struct device_attribute *android_usb_attributes[] = {
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
#ifdef CONFIG_LGE_USB_G_ANDROID
	&dev_attr_iSerialNumber,
#endif
	&dev_attr_functions,
	&dev_attr_enable,
	&dev_attr_state,
#ifdef CONFIG_LGE_USB_G_ANDROID
	&dev_attr_factory_cable,
	&dev_attr_lock,
#endif
#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
	&dev_attr_config_num,
#endif
	NULL
};

#ifdef CONFIG_LGE_USB_G_ANDROID
static void android_factory_desc(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = cdev_to_android_dev(cdev);
	struct list_head *curr_conf = &dev->configs;
	struct android_configuration *conf;
	struct android_usb_function_holder *f_holder;
	char *conf_str;
	char *name, *b;
	int err;
	char factory_driver[256] = {"acm,gser"};
	char laf_driver[256] = {"acm,laf"};

	// update values in composite driver's copy of device descriptor

	device_desc.idVendor = LGE_PIF_VID;
	device_desc.idProduct = LGE_PIF_PID;
	device_desc.iSerialNumber = LGE_PIF_SN;
	device_desc.bDeviceClass = USB_CLASS_COMM;
	device_desc.bDeviceSubClass = 0x0;
	device_desc.bDeviceProtocol = 0x0;

	cdev->desc.idVendor = device_desc.idVendor;
	cdev->desc.idProduct = device_desc.idProduct;
	cdev->desc.iSerialNumber = device_desc.iSerialNumber;
	cdev->desc.bcdDevice = device_desc.bcdDevice;
	cdev->desc.bDeviceClass = device_desc.bDeviceClass;
	cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
	cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;

	/* Clear previous enabled list */
	list_for_each_entry(conf, &dev->configs, list_item) {
		while (conf->enabled_functions.next !=
				&conf->enabled_functions) {
			f_holder = list_entry(conf->enabled_functions.next,
					typeof(*f_holder),
					enabled_list);
			f_holder->f->android_dev = NULL;
			list_del(&f_holder->enabled_list);
			kfree(f_holder);
		}
		INIT_LIST_HEAD(&conf->enabled_functions);
	}

	if (lge_get_laf_mode()) // for laf
		b = strim(laf_driver);
	else  //only both modem port and com port for factory
		b = strim(factory_driver);

	while (b) {
		conf_str = strsep(&b, ":");
		if (conf_str) {
			/* If the next not equal to the head, take it */
			if (curr_conf->next != &dev->configs)
				conf = list_entry(curr_conf->next,
						struct android_configuration,
						list_item);
			else
				conf = alloc_android_config(dev);

			curr_conf = curr_conf->next;
		}

		while (conf_str) {
			name = strsep(&conf_str, ",");
			if (name) {
				err = android_enable_function(dev, conf, name);
				if (err)
					pr_err("android_usb: Cannot enable %s",
							name);
			}
		}
	}
}

static void android_set_factory_mode(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = cdev_to_android_dev(cdev);
	char usb_driver_desc = 0;

	// 1. usb_driver_desc = 0
	//     - loading usb drivers with meta_init.project.rc file because of  meta timming issue (meta mode)
	//     - loading usb drivers with init.usb.rc file
	// 2. usb_driver_desc = 1
	//     loading only both the modem port and the com port
	// 3. usb_driver_desc = 2
	//     loading pc_suite,adb drivers for adb debugging or LG UP factory upgrade mode download(for fboot test of sw Q team's request)
	if (g_boot_mode == META_BOOT)
		usb_driver_desc = 0;
	else if (lge_get_factory_boot() && g_boot_mode != RECOVERY_BOOT)
		usb_driver_desc = 1;
	else
		usb_driver_desc = 0;

	printk("\n[USB]android_set_factory_mode cable_type %d, usb_driver_desc %d\n", lge_get_board_cable(), usb_driver_desc);

	if (usb_driver_desc == 1) {
		dev->lock = USB_FACTORY; //initialize lock for loading adb driver
		android_factory_desc(cdev);
	}
	else
	{
		dev->lock = USB_NORMAL; //initialize lock for loading adb driver
	}
}
#endif
/*-------------------------------------------------------------------------*/
/* Composite driver */

static int android_bind_config(struct usb_configuration *c)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = cdev_to_android_dev(c->cdev);
#else
	struct android_dev *dev = _android_dev;
#endif
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = cdev_to_android_dev(c->cdev);
#else
	struct android_dev *dev = _android_dev;
#endif

	android_unbind_enabled_functions(dev, c);
}
#ifndef CONFIG_LGE_USB_G_ANDROID
static int android_setup_config(struct usb_configuration *c, const struct usb_ctrlrequest *ctrl)
{
	int handled = -EINVAL;
	const u8 recip = ctrl->bRequestType & USB_RECIP_MASK;

	pr_notice("%s bRequestType=%x, bRequest=%x, recip=%x\n", __func__, ctrl->bRequestType, ctrl->bRequest, recip);

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (ctrl->bRequest)
		{
			case USB_REQ_CLEAR_FEATURE:
				switch (recip)
				{
					case USB_RECIP_DEVICE:
						switch (ctrl->wValue)
						{
							case USB_DEVICE_U1_ENABLE:
								handled = 1;
								pr_notice("Clear Feature->U1 Enable\n");
							break;

							case USB_DEVICE_U2_ENABLE:
								handled = 1;
								pr_notice("Clear Feature->U2 Enable\n");
							break;

							default:
								handled = -EINVAL;
							break;
						}
						break;
					default:
						handled = -EINVAL;
					break;
				}
			break;

			case USB_REQ_SET_FEATURE:
				switch (recip)
				{
					case USB_RECIP_DEVICE:
						switch (ctrl->wValue)
						{
							case USB_DEVICE_U1_ENABLE:
								pr_notice("Set Feature->U1 Enable\n");
								handled = 1;
							break;
							case USB_DEVICE_U2_ENABLE:
								pr_notice("Set Feature->U2 Enable\n");
								handled = 1;
							break;
							default:
								handled = -EINVAL;
							break;
						}
					break;

					default:
						handled = -EINVAL;
					break;
				}
			break;

			default:
				handled = -EINVAL;
			break;
		}
	}

	return handled;
}
#endif
static int android_bind(struct usb_composite_dev *cdev)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev;
#else
	struct android_dev *dev = _android_dev;
#endif
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;

#ifdef CONFIG_LGE_USB_G_ANDROID
	/* Bind to the last android_dev that was probed */
	dev = list_entry(android_dev_list.prev, struct android_dev, list_item);
#endif
	dev->cdev = cdev;

	/*
	 * Start disconnected. Userspace will connect the gadget once
	 * it is done configuring the functions.
	 */
	usb_gadget_disconnect(gadget);

#ifdef CONFIG_LGE_USB_G_ANDROID
	/* Init the supported functions only once, on the first android_dev */
	if (android_dev_count == 1) {
		ret = android_init_functions(dev->functions, cdev);
		if (ret)
			return ret;
	}
#else
	ret = android_init_functions(dev->functions, cdev);
	if (ret)
		return ret;
#endif

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	/* Default strings - should be updated by userspace */
	strncpy(manufacturer_string, MANUFACTURER_STRING, sizeof(manufacturer_string) - 1);
	strncpy(product_string, PRODUCT_STRING, sizeof(product_string) - 1);
	strncpy(serial_string, "0123456789ABCDEF", sizeof(serial_string) - 1);

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

#ifdef CONFIG_LGE_USB_G_ANDROID
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[CHARGE_ONLY_STRING_IDX].id = id;
	strlcpy(charge_only_string, "USB Charge Only Interface",
			sizeof(charge_only_string) - 1);
#endif

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}
#ifdef CONFIG_USBIF_COMPLIANCE
	usb_gadget_clear_selfpowered(gadget);
#else
	usb_gadget_set_selfpowered(gadget);
#endif
#ifdef CONFIG_LGE_USB_G_ANDROID
	android_set_factory_mode(cdev);
#endif
/* LGE_CHANGE_E :fatory cable detection.*/
	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = cdev_to_android_dev(cdev);
#else
	struct android_dev *dev = _android_dev;
#endif
	manufacturer_string[0] = '\0';
	product_string[0] = '\0';
	serial_string[0] = '0';
	cancel_work_sync(&dev->work);
	android_cleanup_functions(dev->functions);
	return 0;
}

/* HACK: android needs to override setup for accessory to work */
static int (*composite_setup_func)(struct usb_gadget *gadget, const struct usb_ctrlrequest *c);
extern void composite_setup_complete(struct usb_ep *ep, struct usb_request *req);

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = cdev_to_android_dev(cdev);
#else
	struct android_dev *dev = _android_dev;
#endif
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_usb_function_holder *f_holder;
	struct android_configuration	*conf;
#endif
	int value = -EOPNOTSUPP;
	unsigned long flags;
	static DEFINE_RATELIMIT_STATE(ratelimit, 1 * HZ, 5);

	if (__ratelimit(&ratelimit))
		pr_notice("[USB][ratelimit]%s,\n", __func__);

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;

#ifdef CONFIG_LGE_USB_G_ANDROID
	list_for_each_entry(conf, &dev->configs, list_item)
		list_for_each_entry(f_holder,
				    &conf->enabled_functions,
				    enabled_list) {
			f = f_holder->f;
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
#ifndef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
				if (value >= 0)
					break;
#else
				if (value >= 0) {
					if (!strcmp(f->name, "mtp"))
						goto list_exit;
					else
						break;
				}
#endif
			}
		}
#ifdef CONFIG_LGE_USB_G_MULTIPLE_CONFIGURATION
list_exit:
#endif
#else
	list_for_each_entry(f, &dev->enabled_functions, enabled_list) {
		if (f->ctrlrequest) {
			value = f->ctrlrequest(f, cdev, c);
			if (value >= 0)
				break;
		}
	}
#endif

	/* Special case the accessory function.
	 * It needs to handle control requests before it is enabled.
	 */
	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

	if (value < 0)
		value = composite_setup_func(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		schedule_work(&dev->work);
	} else if (c->bRequest == USB_REQ_SET_CONFIGURATION &&
						cdev->config) {
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return value;
}

static void android_disconnect(struct usb_composite_dev *cdev)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = cdev_to_android_dev(cdev);
#else
	struct android_dev *dev = _android_dev;
#endif

	/* Added for USB Develpment debug, more log for more debuging help */
	pr_notice("[USB]%s: \n", __func__);
	/* Added for USB Develpment debug, more log for more debuging help */

	/* accessory HID support can be active while the
	   accessory function is not actually enabled,
	   so we need to inform it when we are disconnected.
	 */
	acc_disconnect();

	dev->connected = 0;
	schedule_work(&dev->work);

	/* Added for USB Develpment debug, more log for more debuging help */
	pr_notice("[USB]%s: dev->connected = %d \n", __func__, dev->connected);
	/* Added for USB Develpment debug, more log for more debuging help */
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.unbind		= android_usb_unbind,
	.disconnect	= android_disconnect,
#ifdef CONFIG_USB_MU3D_DRV
	.max_speed	= USB_SPEED_SUPER
#else
	.max_speed	= USB_SPEED_HIGH
#endif
};

static int android_create_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	int err;

	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, "android0");
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

#ifdef CONFIG_LGE_USB_G_ANDROID
static void android_destroy_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(dev->dev, attr);
	device_destroy(android_class, dev->dev->devt);
}

static struct android_dev *cdev_to_android_dev(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = NULL;

	/* Find the android dev from the list */
	list_for_each_entry(dev, &android_dev_list, list_item) {
		if (dev->cdev == cdev)
			break;
	}

	return dev;
}

static struct android_configuration *alloc_android_config
						(struct android_dev *dev)
{
	struct android_configuration *conf;

	conf = kzalloc(sizeof(*conf), GFP_KERNEL);
	if (!conf) {
		pr_err("%s(): Failed to alloc memory for android conf\n",
			__func__);
		return ERR_PTR(-ENOMEM);
	}

	dev->configs_num++;
	conf->usb_config.label = dev->name;
	conf->usb_config.unbind = android_unbind_config;
	conf->usb_config.bConfigurationValue = dev->configs_num;

	INIT_LIST_HEAD(&conf->enabled_functions);

	list_add_tail(&conf->list_item, &dev->configs);

	return conf;
}

static void free_android_config(struct android_dev *dev,
			     struct android_configuration *conf)
{
	list_del(&conf->list_item);
	dev->configs_num--;
	kfree(conf);
}
#endif

#ifdef CONFIG_USBIF_COMPLIANCE

#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>


static int andoid_usbif_driver_on = 0 ;


static int android_start(void)
{
	struct android_dev *dev;
	int err;

	pr_notice("android_start ===> \n");

	err = usb_composite_probe(&android_usb_driver);
	if (err) {
		pr_err("%s: failed to probe driver %d", __func__, err);
	}

	/* HACK: exchange composite's setup with ours */
	composite_setup_func = android_usb_driver.gadget_driver.setup;
	android_usb_driver.gadget_driver.setup = android_setup;

	pr_notice("android_start <=== \n");

	return err;
}

static int android_stop(void)
{
	pr_notice("android_stop ===> \n");

	usb_composite_unregister(&android_usb_driver);

	pr_notice("android_stop <=== \n");
}

static int andoid_usbif_proc_show(struct seq_file *seq, void *v)
{
	seq_printf(seq, "andoid_usbif_proc_show, andoid_usbif_driver_on is %d (on:1, off:0)\n", andoid_usbif_driver_on);
	return 0;
}

static int andoid_usbif_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, andoid_usbif_proc_show, inode->i_private);
}

static ssize_t andoid_usbif_proc_write(struct file *file, const char __user *buf, size_t length, loff_t *ppos)
{
	int ret ;
	char msg[32] ;
	int result;
	int status;
	struct device	*dev ;
	int		irq ;
	struct resource	*iomem;
	void __iomem	*base;
	struct musb *musb ;
	void __iomem	*ctrl_base;

	if (length >= sizeof(msg)) {
		pr_notice("andoid_usbif_proc_write length error, the error len is %d\n", (unsigned int)length);
		return -EINVAL;
	}
	if (copy_from_user(msg, buf, length))
		return -EFAULT;

	msg[length] = 0 ;

	pr_notice("andoid_usbif_proc_write: %s, current driver on/off: %d\n", msg, andoid_usbif_driver_on);

	if ((msg[0] == '1') && (andoid_usbif_driver_on == 0)){
		pr_notice("start usb android driver ===> \n");
		printk("start usb android driver ===> \n");
		android_start() ;
		andoid_usbif_driver_on = 1 ;
		pr_notice("start usb android driver <=== \n");
	}else if ((msg[0] == '0') && (andoid_usbif_driver_on == 1)){
		pr_notice("stop usb android driver ===> \n");
		printk("stop usb android driver ===> \n");
		andoid_usbif_driver_on = 0 ;
		android_stop() ;

		pr_notice("stop usb android driver <=== \n");
	}

	return length;
}

static const struct file_operations andoid_usbif_proc_fops = {
	.owner = THIS_MODULE,
	.open = andoid_usbif_proc_open,
	.write = andoid_usbif_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,

};

static int __init init(void)
{
	struct android_dev *dev;
	int err;
	struct proc_dir_entry *prEntry;

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		goto err_dev;
	}

	dev->disable_depth = 1;
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
	mutex_init(&dev->mutex);

	err = android_create_device(dev);
	if (err) {
		pr_err("%s: failed to create android device %d", __func__, err);
		goto err_create;
	}

	_android_dev = dev;

	prEntry = proc_create("android_usbif_init", 0666, NULL, &andoid_usbif_proc_fops);

	if (prEntry)
	{
		printk("create the android_usbif_init proc OK!\n") ;
	}else{
		printk("[ERROR] create the android_usbif_init proc FAIL\n") ;
	}

	// set android up at boot up
	android_start() ;
	andoid_usbif_driver_on = 1 ;

	return 0;

err_create:
	kfree(dev);
err_dev:
	class_destroy(android_class);
	return err;
}

late_initcall(init);



static void __exit cleanup(void)
{
	printk("[U3D] android cleanup ===> \n") ;
	class_destroy(android_class);
	kfree(_android_dev);
	_android_dev = NULL;
	printk("[U3D] android cleanup <=== \n") ;
}
module_exit(cleanup);

#else
static int __init init(void)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *android_dev;
	int err;

	INIT_LIST_HEAD(&android_dev_list);
	android_dev_count = 0;

	if (!android_class) {
	android_class = class_create(THIS_MODULE, "android_usb");

	if (IS_ERR(android_class))
		return PTR_ERR(android_class);
	}

	android_dev = kzalloc(sizeof(*android_dev), GFP_KERNEL);
	if (!android_dev) {
		pr_err("%s(): Failed to alloc memory for android_dev\n",
			__func__);
		err = -ENOMEM;
		goto err_alloc;
	}

	android_dev->disable_depth = 1;
	android_dev->functions = supported_functions;
	android_dev->configs_num = 0;
	INIT_LIST_HEAD(&android_dev->configs);
	INIT_WORK(&android_dev->work, android_work);
	mutex_init(&android_dev->mutex);

	list_add_tail(&android_dev->list_item, &android_dev_list);
	android_dev_count++;

	err = android_create_device(android_dev);
	if (err) {
		pr_err("%s(): android_create_device failed\n", __func__);
		goto err_dev;

	}

	err =  usb_composite_probe(&android_usb_driver);
	if (err) {
		pr_err("%s(): Failed to register android "
				 "composite driver\n", __func__);
		goto err_probe;
	}

	/* HACK: exchange composite's setup with ours */
	composite_setup_func = android_usb_driver.gadget_driver.setup;
	android_usb_driver.gadget_driver.setup = android_setup;

	return err;
err_probe:
	android_destroy_device(android_dev);
err_dev:
	list_del(&android_dev->list_item);
	android_dev_count--;
	kfree(android_dev);
err_alloc:
	if (list_empty(&android_dev_list)) {
		class_destroy(android_class);
		android_class = NULL;
	}
	return err;
#else
	struct android_dev *dev;
	int err;

	android_class = class_create(THIS_MODULE, "android_usb");
	if (IS_ERR(android_class))
		return PTR_ERR(android_class);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		err = -ENOMEM;
		goto err_dev;
	}

	dev->disable_depth = 1;
	dev->functions = supported_functions;
	INIT_LIST_HEAD(&dev->enabled_functions);
	INIT_WORK(&dev->work, android_work);
	mutex_init(&dev->mutex);

	err = android_create_device(dev);
	if (err) {
		pr_err("%s: failed to create android device %d", __func__, err);
		goto err_create;
	}

	_android_dev = dev;

	err = usb_composite_probe(&android_usb_driver);
	if (err) {
		pr_err("%s: failed to probe driver %d", __func__, err);
		goto err_probe;
	}

	/* HACK: exchange composite's setup with ours */
	composite_setup_func = android_usb_driver.gadget_driver.setup;
	android_usb_driver.gadget_driver.setup = android_setup;

	return 0;

err_probe:
	device_destroy(android_class, dev->dev->devt);
err_create:
	kfree(dev);
err_dev:
	class_destroy(android_class);
	return err;
#endif
}
late_initcall(init);

static void __exit cleanup(void)
{
#ifdef CONFIG_LGE_USB_G_ANDROID
	struct android_dev *dev = NULL;
	/* Find the android dev from the list */
	list_for_each_entry(dev, &android_dev_list, list_item) {
		if(!dev->cdev)
			break;
	}

	if (dev) {
		android_destroy_device(dev);
		list_del(&dev->list_item);
		android_dev_count--;
		kfree(dev);
	}

	if (list_empty(&android_dev_list)) {
		class_destroy(android_class);
		android_class = NULL;
		usb_composite_unregister(&android_usb_driver);
	}
#else
	usb_composite_unregister(&android_usb_driver);
	class_destroy(android_class);
	kfree(_android_dev);
	_android_dev = NULL;
#endif
}
module_exit(cleanup);
#endif
