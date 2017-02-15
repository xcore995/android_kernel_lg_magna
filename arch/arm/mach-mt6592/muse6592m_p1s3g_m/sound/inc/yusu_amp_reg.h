#ifndef _YUSU_AMP_REG_
#define _YUSU_AMP_REG_

//define registers
#define EAMP_REG_SUBSYSTEMCONTROL           0x00
#define EMPA_REG_INPUTCONTROL               0x01
#define EMPA_REG_LIMITER_CONTROL            0x02
#define EMPA_REG_SPEAKER_OUTPUT_CONTROL     0x03
#define EMPA_REG_HEADPHONE_OUTPUT_CONTROL   0x04
#define EMPA_REG_SPEAKER_VOLUME             0x05
#define EMPA_REG_HEADPHONE_LEFT_VOLUME      0x06
#define EMPA_REG_HEADPHONE_RIGHT_VOLUME     0x07


/*----------------(0) EAMP_REG_SUBSYSTEMCONTROL --------=----------*/
#define THERMAL_CONDITION_HIGH      0x01

#define SPEAKER_FAULT_HIGH          0x02

#define SPREAD_SPECTRUM_HIGH        0x04

#define SHUTDOWN_MODE_HIGH          0x08

#define SPK_BYPASSMODE_HIGH         0x10
/*====------------------------------------------------------------*/

/*----------------(1) EMPA_REG_INPUTCONTROL --------=----------*/
//INPUT MODE
#define CH1_DIFF_INPUT              0x20
#define CH1_SINGLE_INPUT            0x00
#define CH1_INPUT_MASK              0x20
#define CH1_INPUT_SHIFT             0x05

#define CH2_DIFF_INPUT              0x10
#define CH2_SINGLE_INPUT            0x00
#define CH2_INPUT_MASK              0x10
#define CH2_INPUT_SHIFT             0x04

//INPUT MODE GAIN
#define CH1_GAIN_LEVEL1             0x00
#define CH1_GAIN_LEVEL2             0x04
#define CH1_GAIN_LEVEL3             0x08
#define CH1_GAIN_LEVEL4             0x0c
#define CH1_GAIN_LEVEL_MASK         0x0c
#define CH1_GAIN_LEVEL_SHIFT        0x02

#define CH2_GAIN_LEVEL1             0x00
#define CH2_GAIN_LEVEL2             0x01
#define CH2_GAIN_LEVEL3             0x02
#define CH2_GAIN_LEVEL4             0x03
#define CH2_GAIN_LEVEL_MASK         0x03
#define CH2_GAIN_LEVEL_SHIFT        0x00

/*------------------------------------------------------------*/


/*----------------(2) EMPA_REG_LIMITER_CONTROL ------------------*/
//RELEASE AND ATTACK TIME REGISTER
#define ATK_TIME0               0x00
#define ATK_TIME1               0x01
#define ATK_TIME2               0x02
#define ATK_TIME3               0x03
#define ATK_TIME4               0x04
#define ATK_TIME5               0x05
#define ATK_TIME6               0x06
#define ATK_TIME7               0x07
#define ATK_TIME_MASK           0x07
#define ATK_TIME_SHIFT          0x00

#define REL_TIME0               0x08
#define REL_TIME1               0x08
#define REL_TIME2               0x10
#define REL_TIME3               0x18
#define REL_TIME4               0x20
#define REL_TIME5               0x28
#define REL_TIME6               0x30
#define REL_TIME7               0x38
#define REL_TIME8               0x40
#define REL_TIME9               0x48
#define REL_TIME10              0x50
#define REL_TIME11              0x58
#define REL_TIME12              0x60
#define REL_TIME13              0x68
#define REL_TIME14              0x70
#define REL_TIME15              0x78
#define REL_TIME_MASK           0x78
#define REL_TIME_SHIFT          0x03
/*------------------------------------------------------------*/


/*----------------(3) EMPA_REG_SPEAKER_OUTPUT_CONTROL ------------------*/
//SPEAKER MUX AND LIMITER REGISTER
#define SPK_LIMITER_LEVEL0          0x00
#define SPK_LIMITER_LEVEL1          0x01
#define SPK_LIMITER_LEVEL2          0x02
#define SPK_LIMITER_LEVEL3          0x03
#define SPK_LIMITER_LEVEL4          0x04
#define SPK_LIMITER_LEVEL5          0x05
#define SPK_LIMITER_LEVEL6          0x06
#define SPK_LIMITER_LEVEL7          0x07
#define SPK_LIMITER_LEVEL8          0x08
#define SPK_LIMITER_LEVEL9          0x09
#define SPK_LIMITER_LEVEL10         0x0a
#define SPK_LIMITER_LEVEL11         0x0b
#define SPK_LIMITER_LEVEL12         0x0c
#define SPK_LIMITER_LEVEL13         0x0d
#define SPK_LIMITER_LEVEL14         0x0e
#define SPK_LIMITER_LEVEL15         0x0f
#define SPK_LIMITER_LEVEL_MASK      0x0F
#define SPK_LIMITER_LEVEL_SHIFT     0x00

#define SPK_LIMITER_ENABLE          0x10

#define SPK_OUT_MUTE                0x00
#define SPK_OUT_IN1                 0x20
#define SPK_OUT_IN2                 0x40
#define SPK_OUT_IN1_IN2             0x60
#define SPK_OUT_MASK                0x60
#define SPK_OUT_SHIFT               0x05

/*------------------------------------------------------------*/


/*----------------(4) EMPA_REG_HEADPHONE_OUTPUT_CONTROL ------------------*/
//HEADPHONE MUX AND LIMITER REGISTER
//LIMITER LEVEL CONTROL FOR THE HPH AMP
#define HPH_LIMITER_LEVEL0          0x00
#define HPH_LIMITER_LEVEL1          0x01
#define HPH_LIMITER_LEVEL2          0x02
#define HPH_LIMITER_LEVEL3          0x03
#define HPH_LIMITER_LEVEL4          0x04
#define HPH_LIMITER_LEVEL5          0x05
#define HPH_LIMITER_LEVEL6          0x06
#define HPH_LIMITER_LEVEL7          0x07
#define HPH_LIMITER_LEVEL_MASK      0x07
#define HPH_LIMITER_LEVEL_SHIFT     0x00


#define HPH_LIMITER_ENABLE          0x10

#define HPH_OUT_MUTE                0x00
#define HPH_OUT_IN1                 0x20
#define HPH_OUT_IN2                 0x40
#define HPH_OUT_IN1_IN2             0x60
#define HPH_OUT_MASK                0x60
#define HPH_OUT_SHIFT               0x05

/*------------------------------------------------------------*/


/*----------------(5) EMPA_REG_SPEAKER_VOLUME ------------------*/
//SPK
#define SPK_ENABLE          0x20
#define SPK_DISABLE          0x00

#define SPK_VOLUME0         0x00
#define SPK_VOLUME1         0x01
#define SPK_VOLUME2         0x02
#define SPK_VOLUME3         0x03
#define SPK_VOLUME4         0x04
#define SPK_VOLUME5         0x05
#define SPK_VOLUME6         0x06
#define SPK_VOLUME7         0x07
#define SPK_VOLUME8         0x08
#define SPK_VOLUME9         0x09
#define SPK_VOLUME10        0x0a
#define SPK_VOLUME11        0x0b
#define SPK_VOLUME12        0x0c
#define SPK_VOLUME13        0x0d
#define SPK_VOLUME14        0x0e
#define SPK_VOLUME15        0x0f
#define SPK_VOLUME16        0x10
#define SPK_VOLUME17        0x11
#define SPK_VOLUME18        0x12
#define SPK_VOLUME19        0x13
#define SPK_VOLUME20        0x14
#define SPK_VOLUME21        0x15
#define SPK_VOLUME22        0x16
#define SPK_VOLUME23        0x17
#define SPK_VOLUME24        0x18
#define SPK_VOLUME25        0x19
#define SPK_VOLUME26        0x1a
#define SPK_VOLUME27        0x1b
#define SPK_VOLUME28        0x1c
#define SPK_VOLUME29        0x1d
#define SPK_VOLUME30        0x1e
#define SPK_VOLUME31        0x1f
#define SPK_VOLUME_MASK     0x1f
#define SPK_VOLUME_SHIFT    0x00


/*------------------------------------------------------------*/


/*----------------(6) EMPA_REG_HEADPHONE_LEFT_VOLUME ------------------*/
//HPH
#define HPH_TRACK           0x40

#define HPH_ENABLE          0x20

#define HPH_VOLUME0         0x00
#define HPH_VOLUME1         0x01
#define HPH_VOLUME2         0x02
#define HPH_VOLUME3         0x03
#define HPH_VOLUME4         0x04
#define HPH_VOLUME5         0x05
#define HPH_VOLUME6         0x06
#define HPH_VOLUME7         0x07
#define HPH_VOLUME8         0x08
#define HPH_VOLUME9         0x09
#define HPH_VOLUME10        0x0a
#define HPH_VOLUME11        0x0b
#define HPH_VOLUME12        0x0c
#define HPH_VOLUME13        0x0d
#define HPH_VOLUME14        0x0e
#define HPH_VOLUME15        0x0f
#define HPH_VOLUME16        0x10
#define HPH_VOLUME17        0x11
#define HPH_VOLUME18        0x12
#define HPH_VOLUME19        0x13
#define HPH_VOLUME20        0x14
#define HPH_VOLUME21        0x15
#define HPH_VOLUME22        0x16
#define HPH_VOLUME23        0x17
#define HPH_VOLUME24        0x18
#define HPH_VOLUME25        0x19
#define HPH_VOLUME26        0x1a
#define HPH_VOLUME27        0x1b
#define HPH_VOLUME28        0x1c
#define HPH_VOLUME29        0x1d
#define HPH_VOLUME30        0x1e
#define HPH_VOLUME31        0x1f
#define HPH_VOLUME_MASK     0x1f
#define HPH_VOLUME_SHIFT    0x00

/*------------------------------------------------------------*/
#endif
