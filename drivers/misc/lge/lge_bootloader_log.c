/*
 * arch/arm/mach-msm/lge/lge_bootloader_log.c
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/io.h>
#include <asm/setup.h>
//#include <mach/board_lge.h>
#include <linux/init.h>

#define ATAG_LGE_BOOTLOG	0xc0e0c0e0

struct log_buffer {
	uint32_t    sig;
	uint32_t    start;
	uint32_t    size;
	uint32_t    tot_size;
	uint8_t     data[0];
};

u32 boot_logbuf_phys;
struct log_buffer *boot_logbuf_virt;

static int __init parse_tag_lge_bootlog(const struct tag *tag)
{

	boot_logbuf_phys = tag->u.revision.rev;

	printk(KERN_INFO"%s: find the bootloader log tag\n", __func__);
	printk(KERN_INFO"%s: bootloader log at %x\n", __func__, boot_logbuf_phys);

	return 0;
}

__tagtable(ATAG_LGE_BOOTLOG, parse_tag_lge_bootlog);

static int __init lge_bootlog_init(void)
{
	char *buffer;
	char *token;
	char *ct = "\n";

	boot_logbuf_virt = (struct log_buffer *)ioremap(boot_logbuf_phys, 128 * 1024);
	if (boot_logbuf_virt == NULL) {
		printk(KERN_INFO"%s: failed to map memory\n",__func__);
		return 0;
	}

	printk(KERN_INFO"%s: sig %x\n",__func__, boot_logbuf_virt->sig);
	printk(KERN_INFO"%s: start %d\n",__func__, boot_logbuf_virt->start);
	printk(KERN_INFO"%s: size %d\n",__func__, boot_logbuf_virt->size);
	printk(KERN_INFO"--------------------------------------------------------------\n");
	printk(KERN_INFO"below logs are got from bootloader \n");
	printk(KERN_INFO"--------------------------------------------------------------\n");
	printk(KERN_INFO"\n");
	buffer = (char *)boot_logbuf_virt->data;

	while (1) {
		token = strsep(&buffer, ct);
		if (!token) {
			printk(KERN_INFO"%s: token %p\n",__func__, token);
			break;
		}
		printk(KERN_INFO"%s\n", token);
	}
	printk(KERN_INFO"--------------------------------------------------------------\n");

	return 0;
}

static void __exit lge_bootlog_exit(void)
{
	return;
}

#if !(defined(CONFIG_OF) && defined(TARGET_MT6732_C90))
module_init(lge_bootlog_init);
module_exit(lge_bootlog_exit);
#endif

MODULE_DESCRIPTION("LGE bootloader log driver");
MODULE_AUTHOR("SungEun Kim <cleaneye.kim@lge.com>");
MODULE_LICENSE("GPL");
