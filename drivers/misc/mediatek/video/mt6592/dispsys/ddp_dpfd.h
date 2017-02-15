#ifndef __DDP_DPFD_H__
#define __DDP_DPFD_H__

#include <linux/printk.h>

#define DDPKBITBLIT_CHNL_COUNT  (32)

#define DDPK_CH_HDMI_0 (0)
#define DDPK_CH_HDMI_1 (1)
#define DDPK_CH_HDMI_2 (2)
#define DDPK_CH_HDMI_3 (3)
#define DDPK_CH_WFD_0 (4)

#define DDPK_PRINTF	pr_warn

#define DDPK_SHOWFUNCTION(b_valid_function)

#define DDPK_MSG(fmt, arg...) DDPK_PRINTF("[DDPK MSG]: %s(): "fmt, __func__, ##arg)

#define DDPK_INFO(fmt, arg...) \
do { \
	if (b_ddp_show_info) \
		DDPK_PRINTF("[DDPK INFO]: %s(): "fmt, __func__, ##arg); \
} while (0)

#define DDPK_WARNING(fmt, arg...) \
do { \
	if (b_ddp_show_warning) \
		DDPK_PRINTF("[DDPK W]: %s(): "fmt, __func__, ##arg); \
} while (0)

#define DDPK_ERROR(fmt, arg...) \
do { \
	if (b_ddp_show_error) \
		DDPK_PRINTF("[DDPK E]: %s(): %s@%d: "fmt, __func__, __FILE__, __LINE__, ##arg); \
} while (0)

#define DDPK_ISRINFO(fmt, arg...) \
do { \
	if (b_ddp_show_isrinfo) \
		DDPK_PRINTF("[DDPK ISR]: %s(): "fmt, __func__, ##arg); \
} while (0)

#define DDPK_POWERINFO(fmt, arg...) \
do { \
	if (b_ddp_show_powerinfo) \
		DDPK_PRINTF("[DDPK CG]: "fmt, ##arg); \
} while (0)

#define DDPK_ZOOMINFO(fmt, arg...) \
do { \
	if (b_ddp_show_zoominfo) \
		DDPK_PRINTF("[DDPK ZOOM]: "fmt, ##arg); \
} while (0)

#define DDPK_FPSINFO(fmt, arg...) \
do { \
	if (b_ddp_show_fpsinfo) \
		DDPK_PRINTF("[DDPK FPS]: "fmt, ##arg); \
} while (0)

#define DDPK_BBINFO(fmt, arg...) \
do { \
	if (b_ddp_show_bbinfo) \
		DDPK_PRINTF("[DDPK BB]: "fmt, ##arg); \
} while (0)

#define DDPK_ZSDINFO(fmt, arg...) \
do { \
	if (b_ddp_show_zsdinfo) \
		DDPK_PRINTF("[DDPK ZSD]: "fmt, ##arg); \
} while (0)

#define DDPK_DEBUG(fmt, arg...) DDPK_PRINTF("[DDPK DEBUG]: "fmt, ##arg)

extern unsigned long max_pfn;

enum DpColorFormat_K {
	eY800_K,
	eYUY2_K,
	eUYVY_K,
	eYVYU_K,
	eVYUY_K,
	eYUV_444_1P_K,		/* 5 */
	eYUV_422_I_K,
	eYUV_422_I_BLK_K,

	eNV21_K,
	eNV12_K,
	eNV12_BLK_K,		/* 10 */
	eNV12_BLK_FCM_K,
	eYUV_420_2P_ISP_BLK_K,
	eYUV_420_2P_VDO_BLK_K,
	eYUV_422_2P_K,
	eYUV_444_2P_K,		/* 15 */
	eYUV_420_2P_UYVY_K,
	eYUV_420_2P_VYUY_K,
	eYUV_420_2P_YUYV_K,
	eYUV_420_2P_YVYU_K,


	eYV16_K,		/* 20 */
	eYV12_K,
	eYV21_K,
	eYUV_422_3P_K,
	eYUV_444_3P_K,
	eYUV_420_3P_YVU_K,	/* 25 */
	eYUV_420_3P_K,

	eRGB565_K,
	eRGB888_K,
	eARGB8888_K,
	eABGR8888_K,		/* 30 */

	eRGBA8888_K,
	eBGRA8888_K,

	ePARGB8888_K,
	eXARGB8888_K,

	eCompactRaw1_K,		/* 35 */
	eMTKYUV_K
};

unsigned int ddp_bitblt_ioctl_wait_reequest(unsigned long ioctl_user_param);
unsigned int ddp_bitblt_ioctl_inform_done(unsigned long ioctl_user_param);

void ddpk_testfunc_1(unsigned long channel);
void ddpk_testfunc_2(unsigned long channel);

int DDPK_Bitblt(int channel);
int DDPK_Bitblt_Config(int channel, DdpkBitbltConfig *pParam);

#endif /* __DDP_DPFD_H__ */
