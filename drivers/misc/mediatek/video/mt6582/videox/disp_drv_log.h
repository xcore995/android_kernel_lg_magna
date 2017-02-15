#ifndef __DISP_DRV_LOG_H__
#define __DISP_DRV_LOG_H__

/* /for kernel */
#include <linux/xlog.h>

#define DISP_LOG_PRINT(level, sub_module, fmt, arg...) xlog_printk(level, "DISP/"sub_module, fmt, ##arg)

#define LOG_PRINT(level, module, fmt, arg...) xlog_printk(level, module, fmt, ##arg)

#endif				/* __DISP_DRV_LOG_H__ */
