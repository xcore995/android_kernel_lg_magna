#ifndef MT_MMC_H__
#define MT_MMC_H__

#ifdef CONFIG_MTK_MMC
extern void msdc_clk_status(int *status);
#else
void msdc_clk_status(int *status) { *status = 0; }
#endif

#endif /* MT_MMC_H__ */
