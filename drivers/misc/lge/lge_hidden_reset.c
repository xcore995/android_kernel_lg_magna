/*
 * arch/arm/mach-msm/lge/lge_handle_panic.c
 *
 * Copyright (C) 2012 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/setup.h>
#include <linux/module.h>
#include <mach/mt_reg_base.h>
#include <mach/board_lge.h>


static void 		* lge_hidden_reset_buf = NULL;
static unsigned int lge_hidden_reset_buf_size;

static DEFINE_SPINLOCK(lge_hidden_reset_spin_lock);


static int gen_hidden_reset(const char *val, struct kernel_param *kp)
{
	panic("generate panic for hidden reset");

	return 0;
}
static int dummy_arg;
module_param_call(gen_hidden_reset, gen_hidden_reset, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);



int lge_save_boot_reason(unsigned long reason, unsigned long extra1, unsigned long extra2)
{
	//
	// DRAM : phy 0x0100 0000 ~ 0x0100 0000 + 16 kb
	//         vir 0xF700 0000 ~ 0xF700 0000 + 16 kb
	// reserved upper 32 byte
	//
	// see : preloader- platform.h
	//		physical
	//			#define LGE_RESERVED_BOOT_REASON_AREA   (0x00100000 + 0xF800 - 32)
	// see : mach/mt_reg_base.h
	//		virtual
	//			#define LGE_RAM_CONSOLE_MEM_VBASE 				0xFA000000
	//
#if 1
    volatile u32 *lge_boot_magic_number 	= (volatile u32*)(INTER_SRAM + 0xFC00 - 32);
#else
    volatile u32 *lge_boot_magic_number 	= (volatile u32*)(INTER_SRAM + 0xF800 - 32);
#endif
	volatile u32 *lge_boot_reason 		 	= lge_boot_magic_number + 1; // next 4 byte
	volatile u32 *lge_boot_reason_extra1 	= lge_boot_reason + 1; 			// next 4 byte
	volatile u32 *lge_boot_reason_extra2 	= lge_boot_reason_extra1 + 1; 	// next 4 byte

	unsigned long flags;

	spin_lock_irqsave(&lge_hidden_reset_spin_lock, flags);

	// save boot reason
	*lge_boot_magic_number		= LGE_BOOT_REASON_MAGIC_CODE;
	*lge_boot_reason			= reason;
	*lge_boot_reason_extra1		= extra1;
	*lge_boot_reason_extra2		= extra2;

	printk("%s lge_boot_magic_number = 0x%x, lge_boot_reason = 0x%x\n", __func__, *lge_boot_magic_number, *lge_boot_reason);

	spin_unlock_irqrestore(&lge_hidden_reset_spin_lock, flags);
    return 0;
}


unsigned long lge_get_boot_reason(unsigned long *pExtra1, unsigned long * pExtra2)
{
#ifdef CONFIG_ARCH_MT6582
	#if 1
	volatile u32 *lge_boot_magic_number = (volatile u32*)(INTER_SRAM + 0xFC00 - 32);
	#else
	volatile u32 *lge_boot_magic_number = (volatile u32*)(INTER_SRAM + 0xF800 - 32);
	#endif
#else
	volatile u32 *lge_boot_magic_number = (volatile u32*)(SYSRAM_BASE + 0xF800 - 32);
#endif
	volatile u32 *lge_boot_reason 		 = lge_boot_magic_number + 1; // next 4 byte
	volatile u32 *lge_boot_reason_extra1 = lge_boot_reason + 1; 			// next 4 byte
	volatile u32 *lge_boot_reason_extra2 = lge_boot_reason_extra1 + 1; 	// next 4 byte

	if (*lge_boot_magic_number == LGE_BOOT_REASON_MAGIC_CODE) {
		if (pExtra1 != NULL) {
			*pExtra1 = *lge_boot_reason_extra1;
		}
		if (pExtra2 != NULL) {
			*pExtra2 = *lge_boot_reason_extra2;
		}
		return *lge_boot_reason;
	} else {
		return 0;
	}
}

static int lge_save_crash_reason(struct notifier_block *this, unsigned long event,
		void *ptr)
{
	lge_save_boot_reason(LGE_BOOT_KERNEL_CRASH,0x0,0x0);

	// save framebuffer at lge_hidden_reset_buf
	// TODO

	return NOTIFY_DONE;
}

static struct notifier_block lge_hidden_reset_block = {
	.notifier_call  = lge_save_crash_reason,
};

static int __init lge_hidden_reset_early_init(void)
{
#if 0
	struct resource *res = pdev->resource;
	size_t start;
	size_t buffer_size;
	void *buffer;
	int ret = 0;

	if (res == NULL || pdev->num_resources != 1 ||
			!(res->flags & IORESOURCE_MEM)) {
		printk(KERN_ERR "lge_hidden_reset: invalid resource, %p %d flags "
				"%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}

	buffer_size 	= res->end - res->start + 1;
	start 			= res->start;
	printk(KERN_INFO "lge_hidden_reset: got buffer at %zx, size %zx\n",
			start, buffer_size);

	buffer = ioremap(res->start, buffer_size);
	if (buffer == NULL) {
		printk(KERN_ERR "lge_hidden_reset: failed to map memory\n");
		return -ENOMEM;
	}
	memset(buffer, 0, buffer_size);

	lge_hidden_reset_buf 		= buffer;
	lge_hidden_reset_buf_size	= buffer_size;
#endif
	/* Setup panic notifier */
	atomic_notifier_chain_register(&panic_notifier_list, &lge_hidden_reset_block);

	lge_save_boot_reason(LGE_BOOT_INIT_BOOT_REASON, 0, 0);
}
early_initcall(lge_hidden_reset_early_init);

static int __init lge_hidden_reset_probe(struct platform_device *pdev)
{
#if 1
	int ret = 0;

	return ret;
#else
	struct resource *res = pdev->resource;
	size_t start;
	size_t buffer_size;
	void *buffer;
	int ret = 0;

	if (res == NULL || pdev->num_resources != 1 ||
			!(res->flags & IORESOURCE_MEM)) {
		printk(KERN_ERR "lge_hidden_reset: invalid resource, %p %d flags "
				"%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}

	buffer_size 	= res->end - res->start + 1;
	start 			= res->start;
	printk(KERN_INFO "lge_hidden_reset: got buffer at %zx, size %zx\n",
			start, buffer_size);

	buffer = ioremap(res->start, buffer_size);
	if (buffer == NULL) {
		printk(KERN_ERR "lge_hidden_reset: failed to map memory\n");
		return -ENOMEM;
	}
	memset(buffer, 0, buffer_size);

	lge_hidden_reset_buf 		= buffer;
	lge_hidden_reset_buf_size	= buffer_size;

	/* Setup panic notifier */
	atomic_notifier_chain_register(&panic_notifier_list, &lge_hidden_reset_block);

	lge_save_boot_reason(LGE_BOOT_INIT_BOOT_REASON, 0, 0);

	return ret;
#endif
}

static int  lge_hidden_reset_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver lge_hidden_reset_driver __refdata = {
	.probe 	= lge_hidden_reset_probe,
	.remove	= lge_hidden_reset_remove,
	.driver	= {
		.name = "lge_hidden_reset",
		.owner = THIS_MODULE,
	},
};

static int __init lge_hidden_reset_init(void)
{
	return platform_driver_register(&lge_hidden_reset_driver);
}

static void __exit lge_hidden_reset_exit(void)
{
	platform_driver_unregister(&lge_hidden_reset_driver);
}

module_init(lge_hidden_reset_init);
module_exit(lge_hidden_reset_exit);

MODULE_DESCRIPTION("LGE hidden reset driver");
MODULE_AUTHOR("Jungsu Kim <jungsu06.kim@lge.com>");
MODULE_LICENSE("GPL");
