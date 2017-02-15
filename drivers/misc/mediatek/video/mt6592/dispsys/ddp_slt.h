#ifndef DDP_SLT_H
#define DDP_SLT_H

int slt_test_stream1(void);
int slt_test_stream2(void);
int m4u_ddp_test(unsigned int srcPa, unsigned int dst, unsigned int dstPa);

/* source and golden */
extern unsigned char data_argb_64x64[16384];
extern unsigned char slt_data_rgb888_64x64_golden[12288];
extern unsigned char data_rgb888_64x64_golden2[12288];

#endif /*DDP_SLT_H*/
