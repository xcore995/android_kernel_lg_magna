#ifndef __MTKFB_VSYNC_H__
#define __MTKFB_VSYNC_H__


#define MTKFB_VSYNC_DEVNAME "mtkfb_vsync"

#define MTKFB_VSYNC_IOCTL_MAGIC      'V'

typedef enum {
	MTKFB_VSYNC_SOURCE_LCM = 0,
	MTKFB_VSYNC_SOURCE_HDMI = 1,
} vsync_src;

extern void mtkfb_waitVsync(void);
extern void mtkfb_disable_non_fb_layer(void);

#if defined(CONFIG_MTK_HDMI_SUPPORT)
extern void hdmi_waitVsync(void);
#endif

#define MTKFB_VSYNC_IOCTL     _IOW(MTKFB_VSYNC_IOCTL_MAGIC, 1, vsync_src)

#endif				/* MTKFB_VSYNC_H */
