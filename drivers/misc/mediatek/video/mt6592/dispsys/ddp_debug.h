#ifndef __DDP_DEBUG_H__
#define __DDP_DEBUG_H__

#include <linux/kernel.h>
#include <linux/mmprofile.h>
#include <linux/aee.h>
#include <linux/printk.h>
#include "ddp_drv.h"

extern struct DDP_MMP_Events_t {
	MMP_Event DDP;
	MMP_Event MutexParent;
	MMP_Event Mutex[6];
	MMP_Event BackupReg;
	MMP_Event DDP_IRQ;
	MMP_Event SCL_IRQ;
	MMP_Event ROT_IRQ;
	MMP_Event OVL_IRQ;
	MMP_Event WDMA0_IRQ;
	MMP_Event WDMA1_IRQ;
	MMP_Event RDMA0_IRQ;
	MMP_Event RDMA1_IRQ;
	MMP_Event COLOR_IRQ;
	MMP_Event BLS_IRQ;
	MMP_Event TDSHP_IRQ;
	MMP_Event CMDQ_IRQ;
	MMP_Event Mutex_IRQ;
	MMP_Event WAIT_INTR;
	MMP_Event Debug;
} DDP_MMP_Events;

extern unsigned int dbg_log;
extern unsigned int irq_log;
extern unsigned int irq_err_log;

extern unsigned int gUltraLevel;
extern unsigned int gEnableUltra;

extern unsigned char aal_debug_flag;
extern unsigned char pq_debug_flag;

extern unsigned int isAEEEnabled;

extern void mtkfb_dump_layer_info(void);
extern unsigned int gNeedToRecover;
extern int disp_intr_disable_and_clear(void);

extern void mtkfb_dump_layer_info(void);
extern unsigned int gNeedToRecover;
extern void disp_path_reset(void);
extern int disp_path_ovl_reset(void);

extern unsigned char data_rgb888_64x64[12288];
extern unsigned char data_rgb888_64x64_golden[12288];

#define DISP_IRQ(string, args...) \
do { \
	if (irq_log) \
		pr_warn("[DDP]"string, ##args); \
} while (0)
#define DISP_DBG(string, args...) \
do { \
	if (dbg_log) \
		pr_warn("[DDP]"string, ##args); \
} while (0)
#define DISP_MSG(string, args...) pr_warn("[DDP]"string, ##args)
#define DISP_ERR(string, args...) pr_err("[DDP]error:"string, ##args)
#define DDP_IRQ_ERR(string) \
do { \
	if (irq_err_log) \
		aee_kernel_warning_api(__FILE__, __LINE__, DB_OPT_DEFAULT, "DDP, "string, string); \
} while (0)

void ddp_debug_init(void);
void ddp_debug_exit(void);
int ddp_mem_test(void);
int ddp_mem_test2(void);
void ddp_enable_bls(int BLS_switch);
int ddp_dump_info(DISP_MODULE_ENUM module);
int ddp_clock_on(void);
int ddp_clock_off(void);

#endif /* __DDP_DEBUG_H__ */
