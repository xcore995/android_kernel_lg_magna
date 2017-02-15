/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2009
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include "yusu_android_speaker.h"
#include "yusu_amp_reg.h"

#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/xlog.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/mt_gpio.h>
/*****************************************************************************
*                          DEBUG INFO
*****************************************************************************/

static bool eamp_log_on = true;

#define EAMP_PRINTK(fmt, arg...) \
    do { \
        if (eamp_log_on) xlog_printk(ANDROID_LOG_INFO,"EAMP", \
                    "[EAMP]: %s() "fmt"\n", __func__,##arg); \
    }while (0)


#define ON_OFF_STR(val) (val) ? "On" : "Off"
#define IN_MODE_STR(val) (val) ? "Diffential Input" : "Single Input"
#define SPK_RCV_SEL_STR(val) (val) ? "Receiver" : "Speaker"
#define REG_VALUE(name, data)   (((data) & name ## _MASK) >> name ## _SHIFT)

/*****************************************************************************
*                Select Speak/Headset input
*****************************************************************************/
// USING_EXTAMP_ALL_VOICE_BUFFER
//      the all device use voice buffer in call. Else keep origin design.
// USING_EXTAMP_HP
//      receiver : voice buffer, headphone & speaker : audio buffer
// USING_EXTAMP_TC1
//      receiver & speaker : voice buffer, headphone : audio buffer


//#define CONFIG_USING_EXTAMP_HP
#define CONFIG_USING_EXTAMP_TC1
#define CONFIG_USING_EXTAMP_ALL_VOICE_BUFFER


#ifdef CONFIG_USING_EXTAMP_HP
#define SPEAKER_CALL_IN_PORT        SPK_OUT_IN1
#define SPEAKER_NORMAL_IN_PORT      SPK_OUT_IN2
#else
#define SPEAKER_CALL_IN_PORT        SPK_OUT_IN2
#define SPEAKER_NORMAL_IN_PORT      SPK_OUT_IN2
#endif

#ifdef CONFIG_USING_EXTAMP_ALL_VOICE_BUFFER
#define HEADSET_CALL_IN_PORT        HPH_OUT_IN2
#define HEADSET_NORMAL_IN_PORT      HPH_OUT_IN1
#else
#define HEADSET_CALL_IN_PORT        HPH_OUT_IN1
#define HEADSET_NORMAL_IN_PORT      HPH_OUT_IN1
#endif


#define CH1_INPUT_MODE              CH1_SINGLE_INPUT
#define CH2_INPUT_MODE              CH2_DIFF_INPUT
#define NORMAL_INPUT_MODE           (CH1_INPUT_MODE | CH2_INPUT_MODE)

#define USE_ANALOG_SWITCH  1

/*****************************************************************************
*                For I2C defination
*****************************************************************************/
#define SOUND_I2C_CHANNEL  2

// device address
#define EAMP_SLAVE_ADDR_WRITE   0xE0
#define EAMP_SLAVE_ADDR_READ    0xE1
#define EAMP_I2C_CHANNEL        (1)        //I2C Channel 0
#define EAMP_I2C_DEVNAME "TPA2058D3"

//control point
#define AUDIO_CONTROL_POINT_NUM (5);

// I2C variable
static struct i2c_client *new_client = NULL;

/*LGE_CHANGE ew0804.kim add register setting api */
char proc_str[128];
int register_setting_mode = 0;
/*LGE_CHANGE ew0804.kim add register setting api */

// 20140627 let tpa2028d know current device for combo_path gain control - hyeonsang85.park@lge.com - start
#define DEVICE_OUT_EARPIECER 0
#define DEVICE_OUT_HEADSETR 2
#define DEVICE_OUT_SPEAKERR 4
#define DEVICE_OUT_SPEAKER_HEADSET_R 6
static u32 set_device = 0;
// 20140627 let tpa2028d know current device for combo_path gain control - hyeonsang85.park@lge.com - end

static const struct i2c_device_id eamp_i2c_id[] = {
    {EAMP_I2C_DEVNAME,0},
    {}
};

static struct i2c_board_info __initdata eamp_dev = {
    I2C_BOARD_INFO(EAMP_I2C_DEVNAME,(EAMP_SLAVE_ADDR_WRITE>>1))
};

//function declration
static int eamp_i2c_detect(struct i2c_client *client,
                                int kind, struct i2c_board_info *info);
static int eamp_i2c_probe(struct i2c_client *client,
                                const struct i2c_device_id *id);
static int eamp_i2c_remove(struct i2c_client *client);

//i2c driver
struct i2c_driver eamp_i2c_driver = {
    .probe = eamp_i2c_probe,
    .remove = eamp_i2c_remove,
    //.detect = eamp_i2c_detect,
    .driver = {
        .name =EAMP_I2C_DEVNAME,
    },
    .id_table = eamp_i2c_id,
    //.address_data = &addr_data,
};

// speaker, earpiece, headphone status and path;
static bool gsk_on = false;
static bool gep_on = false;
static bool ghp_on = false;
//volume and gain
static u8  gspvol = 0x1f;
static u8  ghplvol = 0x19;
static u8  ghprvol = 0x19;
#ifdef CONFIG_LGE_USE_SEPERATE_GAIN
static u8  gch1gain = 0x0;
static u8  gch2gain = 0x0;
#else
static u8  gchgain = 0x0;
#endif
//mode
static u32 gMode     = 0;
static u32 gPreMode  = 0;

//response time
static int const speaker_response_time = 6; //ms
static int const headphone_response_time = 12; //ms
#ifdef CONFIG_LGE_USE_SEPERATE_GAIN
//mask
typedef enum
{
    GAIN_MASK_HP      = 0x1,
    GAIN_MASK_SPEAKER = 0x2,
    GAIN_MASK_INPUT2  = 0x4,
    GAIN_MASK_INPUT1  = 0x8,
    GAIN_MASK_ALL     = (GAIN_MASK_HP     |
                         GAIN_MASK_SPEAKER |
                         GAIN_MASK_INPUT2  |
                         GAIN_MASK_INPUT1)
}gain_mask;
#endif
/* LGE_CHANGE_E */

//kernal to open speaker
static bool gsk_forceon = false;
static bool gsk_preon   = false;
static bool gep_preon   = false;

// volume table
static  int gCtrPointNum = AUDIO_CONTROL_POINT_NUM;
static  s8  gCtrPoint[] = {2,2,5,5,5};  // repesent 2bis, 2bits,5bits,5bits,5bits
static  s8  gCtrPoint_in1Gain[]= {0,6,12,20};
static  s8  gCtrPoint_in2Gain[]= {0,6,12,20};
static  s8  gCtrPoint_SpeakerVol[] = {
    -60 ,-50, -45, -42,
    -39 ,-36, -33, -30,
    -27 ,-24, -21, -20,
    -19 ,-18, -17, -16,
    -15 ,-14, -13, -12,
    -11 ,-10, -9, -8 ,
    -7  ,-6 , -5 , -4 ,
    -3  ,-2 , -1,  0};

static  s8 gCtrPoint_HeadPhoneLVol[]= {
    -60 ,-50, -45, -42,
    -39 ,-36, -33, -30,
    -27 ,-24, -21, -20,
    -19 ,-18, -17, -16,
    -15 ,-14, -13, -12,
    -11 ,-10, -9, -8 ,
    -7  ,-6 , -5 , -4 ,
    -3  ,-2 , -1,  0};

static  s8 gCtrPoint_HeadPhoneRVol[]= {
    -60 ,-50, -45, -42,
    -39 ,-36, -33, -30,
    -27 ,-24, -21, -20,
    -19 ,-18, -17, -16,
    -15 ,-14, -13, -12,
    -11 ,-10, -9, -8 ,
    -7  ,-6 , -5 , -4 ,
    -3  ,-2 , -1,  0};

static  s8 *gCtrPoint_table[5]={
    gCtrPoint_in1Gain,
    gCtrPoint_in2Gain,
    gCtrPoint_SpeakerVol,
    gCtrPoint_HeadPhoneLVol,
    gCtrPoint_HeadPhoneRVol};

// function implementation

//read one register
ssize_t static eamp_read_byte(u8 addr, u8 *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int     ret=0;

    if(!new_client)
    {
        EAMP_PRINTK("I2C client not initialized!!");
        return -1;
    }

    cmd_buf[0] = addr;
    ret = i2c_master_send(new_client, &cmd_buf[0], 1);
    if (ret < 0) {
        EAMP_PRINTK("read sends command error!!");
        return -1;
    }
    ret = i2c_master_recv(new_client, &readData, 1);
    if (ret < 0) {
        EAMP_PRINTK("reads recv data error!!");
        return -1;
    }
    *returnData = readData;
    EAMP_PRINTK("addr 0x%x data 0x%x",addr, readData);
    return 0;
}

//read one register
static u8    I2CRead(u8 addr)
{
    u8 regvalue;
    eamp_read_byte(addr,&regvalue);
    return regvalue;
}

// write register
static ssize_t    I2CWrite(u8 addr, u8 writeData)
{
    char    write_data[2] = {0};
    int    ret=0;

    if(!new_client)
    {
        EAMP_PRINTK("I2C client not initialized!!");
        return -1;
    }

    write_data[0] = addr;          // ex. 0x01
    write_data[1] = writeData;
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) {
        EAMP_PRINTK("write sends command error!!");
        return -1;
    }
    EAMP_PRINTK("addr 0x%x data 0x%x",addr,writeData);
    return 0;
}

//write register
static ssize_t    eamp_write_byte(u8 addr, u8 writeData)
{
    char    write_data[2] = {0};
    int    ret=0;

    if(!new_client)
    {
        EAMP_PRINTK("I2C client not initialized!!");
        return -1;
    }

    write_data[0] = addr;          // ex. 0x01
    write_data[1] = writeData;
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) {
        EAMP_PRINTK("write sends command error!!");
        return -1;
    }
    EAMP_PRINTK("addr 0x%x data 0x%x",addr,writeData);
    return 0;
}

//***********subsystem control Register, functions to control bits************
//speaker bypass mode
static ssize_t eamp_set_bypass_mode(bool enable)
{
    u8 temp_control_reg = 0;

    EAMP_PRINTK("enable=%d",enable);

    eamp_read_byte(EAMP_REG_SUBSYSTEMCONTROL,&temp_control_reg);

    if (enable == true)
    {
        eamp_write_byte(EAMP_REG_SUBSYSTEMCONTROL, temp_control_reg & ~SPK_BYPASSMODE_HIGH);
    }
    else
    {
        eamp_write_byte(EAMP_REG_SUBSYSTEMCONTROL, temp_control_reg | SPK_BYPASSMODE_HIGH);
    }
    return 0;
}

//Software Shutdown mode
static ssize_t eamp_set_sws_mode(bool deactivate)
{
    u8 temp_control_reg = 0;

    EAMP_PRINTK("deactivate=%d",deactivate);

    eamp_read_byte(EAMP_REG_SUBSYSTEMCONTROL,&temp_control_reg);
    if (deactivate == true)
    {
        //enable bypass
        eamp_write_byte(EAMP_REG_SUBSYSTEMCONTROL, temp_control_reg | SHUTDOWN_MODE_HIGH);
    }
    else
    {
        //turn off bypass
        eamp_write_byte(EAMP_REG_SUBSYSTEMCONTROL, temp_control_reg & ~ SHUTDOWN_MODE_HIGH);
    }
    return 0;
}

//input mode and volume control Register
static ssize_t eamp_clear_input_gain()
{
    u8 temp_input_reg = 0;

    EAMP_PRINTK("");

    //eamp_read_byte(EMPA_REG_INPUTCONTROL,&temp_input_reg);
    //temp_input_reg = (temp_input_reg >>4)<<4;
    eamp_write_byte(EMPA_REG_INPUTCONTROL, temp_input_reg);

    return 0;
}

// set input gain on channel 1 and 2
static ssize_t eamp_set_input_gain( u8 inGain)
{
    u8 temp_input_reg = 0;

    EAMP_PRINTK("inGain(0x%x)",inGain);

    eamp_read_byte(EMPA_REG_INPUTCONTROL,&temp_input_reg);
    temp_input_reg = (temp_input_reg >>4)<<4;
    eamp_write_byte(EMPA_REG_INPUTCONTROL, temp_input_reg | (inGain & 0xf));

    return 0;
}

#ifdef CONFIG_LGE_USE_SEPERATE_GAIN
// set input gain on channel 1
static ssize_t eamp_set_input1_gain( u8 inGain)
{
    u8 temp_input_reg = 0;

    EAMP_PRINTK("inGain(0x%x)",inGain);

    eamp_read_byte(EMPA_REG_INPUTCONTROL,&temp_input_reg);
    temp_input_reg = (temp_input_reg & 0xf3) | (inGain & 0x3)<<2 ;
    eamp_write_byte(EMPA_REG_INPUTCONTROL, temp_input_reg);

    return 0;
}

// set input gain on channel 2
static ssize_t eamp_set_input2_gain( u8 inGain)
{
    u8 temp_input_reg = 0;

    EAMP_PRINTK("inGain(0x%x)",inGain);

    eamp_read_byte(EMPA_REG_INPUTCONTROL,&temp_input_reg);
    temp_input_reg = ((temp_input_reg >>2)<<2) | (inGain & 0x3) ;
    eamp_write_byte(EMPA_REG_INPUTCONTROL, temp_input_reg);

    return 0;
}
#endif

//  set input mode on channel 1 and 2.  0 for single-end inputs, 1 for differential inputs.
static ssize_t eamp_set_input_mode( bool in1se, bool in2se)
{
    u8 temp_input_reg = 0;
    eamp_read_byte(EMPA_REG_INPUTCONTROL,&temp_input_reg);
    if(in1se)
    {

        temp_input_reg = temp_input_reg & ~CH1_DIFF_INPUT;
    }
    else
    {
        temp_input_reg =temp_input_reg | CH1_DIFF_INPUT;
    }

    if(in2se)
    {

        temp_input_reg =temp_input_reg & ~ CH2_DIFF_INPUT;
    }
    else
    {
        temp_input_reg =temp_input_reg | CH2_DIFF_INPUT;
    }
    return eamp_write_byte(EMPA_REG_INPUTCONTROL, temp_input_reg);
}

//Release and attack time
static ssize_t eamp_Release_attackTime_speed(u8 ATK_time, u8 REL_time)
{

    u8    write_data = (ATK_time & ATK_TIME7) | (REL_time & 0x0f)<<3 ;
    return eamp_write_byte(EMPA_REG_LIMITER_CONTROL,write_data);
}

//speaker mux and limiter
//set limiter level
static ssize_t eamp_set_speakerLimiter_level(u8 limitlev )
{
    u8 temp_speak_reg = 0;
    eamp_read_byte(EMPA_REG_SPEAKER_OUTPUT_CONTROL,&temp_speak_reg);

    temp_speak_reg = (temp_speak_reg >>4) <<4;
    temp_speak_reg  = temp_speak_reg | (limitlev & 0x0f);
    return eamp_write_byte(EMPA_REG_SPEAKER_OUTPUT_CONTROL,temp_speak_reg);
}

// speaker limiter enable. 1 enable, 0 disable.
static ssize_t eamp_speakerLimiter_enable(bool enable )
{
    u8 temp_speak_reg = 0;
    eamp_read_byte(EMPA_REG_SPEAKER_OUTPUT_CONTROL,&temp_speak_reg);

    if (enable == true)
    {
        //enable
        eamp_write_byte(EMPA_REG_SPEAKER_OUTPUT_CONTROL, temp_speak_reg | SPK_LIMITER_ENABLE);
    }
    else
    {
        //disable
        eamp_write_byte(EMPA_REG_SPEAKER_OUTPUT_CONTROL, temp_speak_reg & ~SPK_LIMITER_ENABLE );
    }
    return 0;
}

// control for speaker channel.
static ssize_t eamp_set_speakerOut(u8  speakerout )
{
    u8 temp_speak_reg = 0;
    eamp_read_byte(EMPA_REG_SPEAKER_OUTPUT_CONTROL,&temp_speak_reg);

    temp_speak_reg = (temp_speak_reg & 0x9f ) | ((speakerout & 0x03) << 5);
    eamp_write_byte(EMPA_REG_SPEAKER_OUTPUT_CONTROL, temp_speak_reg);
    return 0;
}

//Headphone mux and limiter
// set headphone limiter level
static ssize_t eamp_set_headPhoneLimiter_level(u8 limitlev )
{
    u8 temp_headphone_reg = 0;
    eamp_read_byte(EMPA_REG_HEADPHONE_OUTPUT_CONTROL,&temp_headphone_reg);

    temp_headphone_reg = (temp_headphone_reg >>3) <<3;
    temp_headphone_reg    = temp_headphone_reg | (limitlev & 0x07);
    return eamp_write_byte(EMPA_REG_HEADPHONE_OUTPUT_CONTROL,temp_headphone_reg);

}

// enable /disable headphone limiter
static ssize_t eamp_headPhoneLimiter_enable(bool enable )
{
    u8 temp_headphone_reg = 0;
    eamp_read_byte(EMPA_REG_HEADPHONE_OUTPUT_CONTROL,&temp_headphone_reg);

    if (enable == true)
    {
        //enable
        eamp_write_byte(EMPA_REG_HEADPHONE_OUTPUT_CONTROL, temp_headphone_reg | HPH_LIMITER_ENABLE);
    }
    else
    {
        //disable
        eamp_write_byte(EMPA_REG_HEADPHONE_OUTPUT_CONTROL, temp_headphone_reg & ~HPH_LIMITER_ENABLE);
    }
    return 0;
}

// control for headphone channel
static ssize_t eamp_set_headPhoneOut(u8  headphoneout )
{
    u8 temp_headphone_reg = 0;
    eamp_read_byte(EMPA_REG_HEADPHONE_OUTPUT_CONTROL,&temp_headphone_reg);

    temp_headphone_reg = (temp_headphone_reg & 0x9f ) | ((headphoneout & 0x03) << 5);
    return eamp_write_byte(EMPA_REG_HEADPHONE_OUTPUT_CONTROL, temp_headphone_reg);
}

//speaker volume
// openspeaker
static ssize_t eamp_set_speaker_Open(bool enable)
{
    u8 temp_spvol_reg = 0;

    EAMP_PRINTK("enable=%d",enable);

    eamp_read_byte(EMPA_REG_SPEAKER_VOLUME,&temp_spvol_reg);

    if (enable == true)
    {
        //enable
        eamp_write_byte(EMPA_REG_SPEAKER_VOLUME, temp_spvol_reg | SPK_ENABLE);
    }
    else
    {
        //disable
        eamp_write_byte(EMPA_REG_SPEAKER_VOLUME, temp_spvol_reg & ~SPK_ENABLE);
    }
    return 0;
}

//set  speaker volume
static ssize_t eamp_set_speaker_vol(u8    vol)
{
    u8 temp_spvol_reg = 0;

    EAMP_PRINTK("vol=0x%x",vol);

    eamp_read_byte(EMPA_REG_SPEAKER_VOLUME,&temp_spvol_reg);
    temp_spvol_reg = (temp_spvol_reg >> 5) << 5;
    temp_spvol_reg = temp_spvol_reg | (vol & 0x1f);
    eamp_write_byte(EMPA_REG_SPEAKER_VOLUME, temp_spvol_reg);
    return 0;
}

//Headphone left channel volume
//enable headphone left  channel
static ssize_t eamp_set_headPhoneL_open( bool  enable )
{
    u8 temp_hpvol_reg = 0;

    EAMP_PRINTK("enable=%d",enable);

    eamp_read_byte(EMPA_REG_HEADPHONE_LEFT_VOLUME,&temp_hpvol_reg);

    if (enable == true)
    {
        //enable
        eamp_write_byte(EMPA_REG_HEADPHONE_LEFT_VOLUME, temp_hpvol_reg | HPH_ENABLE);
    }
    else
    {
        //disable
        eamp_write_byte(EMPA_REG_HEADPHONE_LEFT_VOLUME, temp_hpvol_reg & ~HPH_ENABLE);
    }
    return 0;

}

//set  headphone  volume
static ssize_t eamp_set_headPhone_vol(u8 HP_vol)
{
    u8 temp_hpvol_reg = 0;

    EAMP_PRINTK("vol=0x%x",HP_vol);

    eamp_read_byte(EMPA_REG_HEADPHONE_LEFT_VOLUME,&temp_hpvol_reg);
    temp_hpvol_reg = (temp_hpvol_reg>>5)<<5;
    temp_hpvol_reg = temp_hpvol_reg | 0x40;
    temp_hpvol_reg = temp_hpvol_reg | (HP_vol & 0x1f);

    return eamp_write_byte(EMPA_REG_HEADPHONE_LEFT_VOLUME,temp_hpvol_reg);
}

//set  headphone left  volume
static ssize_t eamp_set_headPhone_lvol(u8 HPL_Vol)
{
    u8 temp_hpvol_reg = 0;

    EAMP_PRINTK("vol=0x%x",HPL_Vol);

    eamp_read_byte(EMPA_REG_HEADPHONE_LEFT_VOLUME,&temp_hpvol_reg);
    temp_hpvol_reg = (temp_hpvol_reg>>5)<<5;
    temp_hpvol_reg = temp_hpvol_reg & 0xbf;
    temp_hpvol_reg = temp_hpvol_reg | (HPL_Vol & 0x1f);

    return eamp_write_byte(EMPA_REG_HEADPHONE_LEFT_VOLUME,temp_hpvol_reg);
}

//Headphone right channel volume  register
//enable headphone Right  channel
static ssize_t eamp_set_headPhoneR_open( bool  enable )
{
    u8 temp_hpvol_reg = 0;

    EAMP_PRINTK("enable=%d",enable);

    eamp_read_byte(EMPA_REG_HEADPHONE_RIGHT_VOLUME,&temp_hpvol_reg);

    if (enable == true)
    {
        //enable
        eamp_write_byte(EMPA_REG_HEADPHONE_RIGHT_VOLUME, temp_hpvol_reg | HPH_ENABLE);
    }
    else
    {
        //disable
        eamp_write_byte(EMPA_REG_HEADPHONE_RIGHT_VOLUME, temp_hpvol_reg & ~HPH_ENABLE);
    }
    return 0;

}

//set  headphone right volume
static ssize_t eamp_set_headPhone_rvol(u8 HPR_Vol)
{
    u8 temp_hpvol_reg = 0;

    EAMP_PRINTK("vol=0x%x",HPR_Vol);

    eamp_read_byte(EMPA_REG_HEADPHONE_RIGHT_VOLUME,&temp_hpvol_reg);
    temp_hpvol_reg = (temp_hpvol_reg>>5)<<5;
    temp_hpvol_reg = temp_hpvol_reg | (HPR_Vol & 0x1f);

    return eamp_write_byte(EMPA_REG_HEADPHONE_RIGHT_VOLUME,temp_hpvol_reg);
}


//**********************************functions to control devices***********************************
// set registers to default value
static ssize_t eamp_resetRegister()
{
    int ret = 0;

    EAMP_PRINTK("");

    ret = I2CWrite(EAMP_REG_SUBSYSTEMCONTROL, SPK_BYPASSMODE_HIGH | SHUTDOWN_MODE_HIGH);
    if(ret < 0) return -1;
    ret = I2CWrite(EMPA_REG_INPUTCONTROL, 0x00);
    if(ret < 0) return -1;
    ret = I2CWrite(EMPA_REG_LIMITER_CONTROL, REL_TIME5 | ATK_TIME2);
    if(ret < 0) return -1;
    ret = I2CWrite(EMPA_REG_SPEAKER_OUTPUT_CONTROL, SPK_DISABLE | SPK_VOLUME15 );
    if(ret < 0) return -1;
    ret = I2CWrite(EMPA_REG_HEADPHONE_OUTPUT_CONTROL, 0x00);
    if(ret < 0) return -1;
    ret = I2CWrite(EMPA_REG_SPEAKER_VOLUME, SPK_VOLUME21 );
    if(ret < 0) return -1;
    ret = I2CWrite(EMPA_REG_HEADPHONE_LEFT_VOLUME, HPH_VOLUME31 | HPH_TRACK);
    if(ret < 0) return -1;
    ret = I2CWrite(EMPA_REG_HEADPHONE_RIGHT_VOLUME, HPH_VOLUME31);
    if(ret < 0) return -1;

    if (ret < 0) {
        return -1;
    }else {
    return 0;
}
}


static ssize_t eamp_openEarpiece()
{
    EAMP_PRINTK("");
#if USE_ANALOG_SWITCH
    mt_set_gpio_out(GPIO_AUDIO_SEL, GPIO_OUT_ONE);
#endif

    I2CWrite(EAMP_REG_SUBSYSTEMCONTROL, SPK_BYPASSMODE_HIGH);
    gep_on=true;
    return 0;
}

static ssize_t eamp_closeEarpiece()
{
    EAMP_PRINTK("");
#if USE_ANALOG_SWITCH
    mt_set_gpio_out(GPIO_AUDIO_SEL, GPIO_OUT_ZERO);
#endif

    I2CWrite(EAMP_REG_SUBSYSTEMCONTROL,0x00);
    gep_on=false;
    return 0;
}

static ssize_t eamp_openheadPhone()
{
    EAMP_PRINTK("");
    mt_set_gpio_out(GPIO_AUDIO_SEL, GPIO_OUT_ZERO);
    if(gep_on) // if earpiece is open ,not open bypass
    {
        I2CWrite(EAMP_REG_SUBSYSTEMCONTROL, SPK_BYPASSMODE_HIGH);
    }
    else
    {
        I2CWrite(EAMP_REG_SUBSYSTEMCONTROL,0x00);//turn open
    }
#ifdef CONFIG_LGE_USE_SEPERATE_GAIN
#ifdef CONFIG_USING_EXTAMP_ALL_VOICE_BUFFER
    if(gMode==2) //MODE_IN_CALL
    {
        I2CWrite(EMPA_REG_INPUTCONTROL, NORMAL_INPUT_MODE | gch1gain<<2 | gch2gain);
    }
    else
#endif
    {
        I2CWrite(EMPA_REG_INPUTCONTROL, NORMAL_INPUT_MODE  | gch1gain<<2 | gch2gain);
    }
#else
#ifdef CONFIG_USING_EXTAMP_ALL_VOICE_BUFFER
    if(gMode==2) //MODE_IN_CALL
    {
        I2CWrite(EMPA_REG_INPUTCONTROL, NORMAL_INPUT_MODE | gchgain);
    }
    else
#endif
    {
        I2CWrite(EMPA_REG_INPUTCONTROL, NORMAL_INPUT_MODE | gchgain);
    }
#endif
    I2CWrite( EMPA_REG_HEADPHONE_LEFT_VOLUME , HPH_TRACK | HPH_ENABLE);
    I2CWrite( EMPA_REG_HEADPHONE_RIGHT_VOLUME , HPH_ENABLE);
#ifdef CONFIG_USING_EXTAMP_ALL_VOICE_BUFFER
    if(gMode==2)
    {
        I2CWrite(EMPA_REG_HEADPHONE_OUTPUT_CONTROL, HEADSET_CALL_IN_PORT);
    }
    else
#endif
    {
        I2CWrite(EMPA_REG_HEADPHONE_OUTPUT_CONTROL, HEADSET_NORMAL_IN_PORT);
    }
    if(ghplvol == ghprvol)
    {
        if (gMode == 2) {
            msleep(270); // for pop-noise
        }
        I2CWrite(EMPA_REG_HEADPHONE_LEFT_VOLUME,( HPH_ENABLE | HPH_TRACK |ghplvol));
    }
    else
    {
        if (gMode == 2) {
            msleep(270); // for pop-noise
        }
        I2CWrite(EMPA_REG_HEADPHONE_LEFT_VOLUME,( HPH_ENABLE | ghplvol));
        I2CWrite(EMPA_REG_HEADPHONE_RIGHT_VOLUME,( HPH_ENABLE | ghprvol));

    }
    ghp_on = true;
    msleep(headphone_response_time);
    return 0;
}

static ssize_t eamp_closeheadPhone()
{
    EAMP_PRINTK("");
    I2CWrite(EMPA_REG_HEADPHONE_LEFT_VOLUME, HPH_ENABLE);
    I2CWrite(EMPA_REG_HEADPHONE_OUTPUT_CONTROL,0x00);
    I2CWrite(EMPA_REG_HEADPHONE_LEFT_VOLUME,0x00);
    I2CWrite(EMPA_REG_HEADPHONE_RIGHT_VOLUME,0x00);
    //I2CWrite(EMPA_REG_HEADPHONE_LEFT_VOLUME,0x20);
    //I2CWrite(EMPA_REG_HEADPHONE_RIGHT_VOLUME,0x20);
    if(!gsk_on)
    {
        I2CWrite(EAMP_REG_SUBSYSTEMCONTROL, SHUTDOWN_MODE_HIGH);
    }
    ghp_on = false;
    return 0;
}

static ssize_t eamp_openspeaker()
{
    EAMP_PRINTK("");
#if USE_ANALOG_SWITCH // LGE_CHANGE : need to check if below line can be removed.
    mt_set_gpio_out(GPIO_AUDIO_SEL, GPIO_OUT_ZERO);
#endif

#ifdef CONFIG_USING_EXTAMP_HP
    if(gMode == 2) //MODE_IN_CALL
    {
        I2CWrite(EAMP_REG_SUBSYSTEMCONTROL,0x00);//turn on subsystem
#ifdef CONFIG_LGE_USE_SEPERATE_GAIN
        I2CWrite(EMPA_REG_INPUTCONTROL,
                NORMAL_INPUT_MODE | gch1gain<<2 | gch2gain);
#else
        I2CWrite(EMPA_REG_INPUTCONTROL, NORMAL_INPUT_MODE | gchgain);
#endif
        I2CWrite(EMPA_REG_SPEAKER_VOLUME, SPK_ENABLE);
        //I2CWrite(EMPA_REG_LIMITER_CONTROL,0x00);
        I2CWrite(EMPA_REG_SPEAKER_OUTPUT_CONTROL, SPEAKER_CALL_IN_PORT);
        I2CWrite(EMPA_REG_SPEAKER_VOLUME, SPK_ENABLE | gspvol);

    }
    else
#endif
    {
        I2CWrite(EAMP_REG_SUBSYSTEMCONTROL,0x40);//turn on subsystem
#ifdef CONFIG_LGE_USE_SEPERATE_GAIN
        I2CWrite(EMPA_REG_INPUTCONTROL,
                NORMAL_INPUT_MODE | gch1gain | gch2gain<<2);
              
#else
         I2CWrite(EMPA_REG_INPUTCONTROL, NORMAL_INPUT_MODE | gchgain);
#endif
        I2CWrite(EMPA_REG_SPEAKER_VOLUME, SPK_ENABLE | SPK_VOLUME15);
        I2CWrite(EMPA_REG_LIMITER_CONTROL,0x00);
        I2CWrite(EMPA_REG_SPEAKER_OUTPUT_CONTROL,
                SPEAKER_NORMAL_IN_PORT | SPK_LIMITER_ENABLE | SPK_LIMITER_LEVEL8);
        I2CWrite(EMPA_REG_SPEAKER_VOLUME, SPK_ENABLE | gspvol);
    }
    gsk_on = true;

    msleep(speaker_response_time);
    return 0;
}

static ssize_t eamp_closespeaker()
{
    EAMP_PRINTK("");

    I2CWrite(EMPA_REG_SPEAKER_VOLUME, SPK_ENABLE);
    I2CWrite(EMPA_REG_SPEAKER_OUTPUT_CONTROL, 0x00);

    I2CWrite(EMPA_REG_SPEAKER_VOLUME, 0x00);//Disable SPK
    if(!ghp_on)
    {
        I2CWrite(EAMP_REG_SUBSYSTEMCONTROL, SHUTDOWN_MODE_HIGH);
    }
    gsk_on = false;
    return 0;
}

#ifdef CONFIG_LGE_USE_SEPERATE_GAIN
// gainvol is composed of 24 bit, the format is as bellows.
//XXXXXXXXXXXXXXXXXXXXXXXX
// bit[ 0-1]:in1 gain, bit[ 2-3]:in2 gain, bit[4-8]: speaker vol,
// bit[9-13]: headphone L volume,
//bit[14-18]: headphone R volume [resolved]
// bit[21-24] mask

static ssize_t eamp_changeGainVolume(unsigned long int param)
{
    u8 mask = param & 0xF;
    u32 gainvol = param & 0xFFFFFF;

/*LGE_CHANGE ew0804.kim add register setting api */
	if((register_setting_mode & 0xF) == mask){
		mask = register_setting_mode & 0xF;
		gainvol = register_setting_mode & 0xFFFFFF;
	}
/*LGE_CHANGE ew0804.kim add register setting api */

    EAMP_PRINTK("param(0x%x)",param);

    if(mask & GAIN_MASK_INPUT1)
    {
        EAMP_PRINTK("GAIN_MASK_INPUT1");
        u8 ch1 = (gainvol>>22) & 0x3;
        if(gch1gain != ch1)
        {
            gch1gain = ch1;
            eamp_set_input1_gain(gch1gain);
        }
    }
    if(mask & GAIN_MASK_INPUT2)
    {
        u8 ch2 = (gainvol>>20) & 0x3;
// [AudioBSP] to fix gain doesn't changed issue - hyeonsang85.park@lge.com - start
//        if(gch2gain != ch2)
//        {
            gch2gain = ch2;
            eamp_set_input2_gain(gch2gain);
//        }
// [AudioBSP] to fix gain doesn't changed issue - hyeonsang85.park@lge.com - end
    }
    if(mask & GAIN_MASK_SPEAKER)
    {
        u8 spk  = (gainvol>>15) & 0x1f;
        if(gspvol != spk)
        {
            gspvol    = spk;
            eamp_set_speaker_vol(gspvol);
        }
    }
    if(mask & GAIN_MASK_HP)
    {
        u8 hpl = (gainvol>>10) & 0x1f;
        if(ghplvol != hpl)
        {
            ghplvol = hpl;
            ghprvol = ghplvol; // hpl equal hpr
            eamp_set_headPhone_vol(ghplvol);
        }
    }
    return 0;
}
#else
// gainvol is composed of 24 bit, the format is as bellows.
//XXXXXXXXXXXXXXXXXXXXXXXX
// bit[ 0-1]:in1 gain, bit[ 2-3]:in2 gain, bit[4-8]: speaker vol,
// bit[9-13]: headphone L volume, bit[14-18]: headphone R volume

static ssize_t eamp_changeGainVolume(unsigned long int param)
{
    u32 gainvol = param & 0xFFFFFF;

    EAMP_PRINTK("param(%u)",param);

    gchgain = (gainvol>>20) & 0xf;
    gspvol  = (gainvol>>15) & 0x1f;
    ghplvol = (gainvol>>10) & 0x1f;
    ghprvol = (gainvol>>5)  & 0x1f;
    eamp_set_input_gain(gchgain);
    eamp_set_speaker_vol(gspvol);
    if(ghplvol == ghprvol)
    {
        eamp_set_headPhone_vol(ghplvol);
    }
    else
    {
        eamp_set_headPhone_lvol(ghplvol);
        eamp_set_headPhone_rvol(ghprvol);
    }

    return 0;
}
#endif

static ssize_t eamp_getGainVolume(void)
{
    u8 gain     = I2CRead(EMPA_REG_INPUTCONTROL) & 0xF ;
    u8 speakvol = I2CRead(EMPA_REG_SPEAKER_VOLUME) & 0x1F;
    u8 hplreg    = I2CRead(EMPA_REG_HEADPHONE_LEFT_VOLUME);
    u8 hplvol    = hplreg & 0x1F;
    u8 hprvol    = I2CRead(EMPA_REG_HEADPHONE_RIGHT_VOLUME) & 0x1F;

    EAMP_PRINTK("");

    if(  hplreg & 0x20 )
        return ( gain << 20 | speakvol << 15 |  \
        hplvol << 10 | hplvol << 5 );

    return ( gain << 20 | speakvol << 15 |    \
        hplvol << 10 | hprvol << 5 );

}

static ssize_t eamp_suspend()
{
    EAMP_PRINTK("");
    eamp_resetRegister();
    return 0;
}

static ssize_t eamp_resume()
{
    EAMP_PRINTK("");
    if(gsk_on)
    {
        eamp_openspeaker();
    }
    if(ghp_on)
    {
        eamp_openheadPhone();
    }
    if(gep_on)
    {
        eamp_openEarpiece();
    }
    return 0;
}

static ssize_t eamp_getRegister(unsigned int regName)
{
    EAMP_PRINTK("Regname=%u",regName);

    if(regName >7)
        return -1;
    return I2CRead(regName);
}

static ssize_t eamp_setRegister(unsigned long int param)
{
    AMP_Control * p = (AMP_Control*)param;

    EAMP_PRINTK("");

    if(p->param1 >7)
        return -1;

    return I2CWrite(p->param1,p->param2);
}


static ssize_t eamp_setMode(unsigned long int param)
{
    EAMP_PRINTK("mode(%u)",param);
    gMode = param;
    return 0;
}

// 20140627 let tpa2028d know current device for combo_path gain control - hyeonsang85.park@lge.com - start
static ssize_t eamp_setDevice(unsigned long int param)
{
    EAMP_PRINTK("set Device (%u)", param);
    set_device = param;
        if (set_device == DEVICE_OUT_SPEAKER_HEADSET_R) {
            I2CWrite(EMPA_REG_SPEAKER_VOLUME, register_setting_mode ? register_setting_mode:SPK_ENABLE | SPK_VOLUME31);
            EAMP_PRINTK("eamp_setDevice : set gain COMBO_PATH = 0x%x\n", SPK_VOLUME31);
        } else {
            I2CWrite(EMPA_REG_SPEAKER_VOLUME, register_setting_mode ? register_setting_mode:SPK_ENABLE | SPK_VOLUME31);
            EAMP_PRINTK("eamp_setDevice : set gain default = 0x%x\n", SPK_VOLUME31);
        }
    return 0;
}
// 20140627 let tpa2028d know current device for combo_path gain control - hyeonsang85.park@lge.com - end

static ssize_t eamp_getCtrlPointNum()
{
    EAMP_PRINTK("");
    return gCtrPointNum;
}

static ssize_t eamp_getCtrPointBits(unsigned long int param)
{
    EAMP_PRINTK("CtrPointBits(%u)",param);
    return gCtrPoint[param];
}

static ssize_t eamp_getCtrlPointTable(unsigned long int param)
{
    EAMP_PRINTK("CtrlPointTable(0x%x)",param);
    AMP_Control *ampCtl = (AMP_Control*)param;
    if(copy_to_user((void __user *)ampCtl->param2,(void *)gCtrPoint_table[ampCtl->param1], 1<<gCtrPoint[ampCtl->param1])){
        return -1;
    }
    return 0;
}

static int eamp_command( unsigned int  type, unsigned long args,unsigned int count)
{
    EAMP_PRINTK("type(%u)",type);
    switch(type)
    {
        case EAMP_SPEAKER_CLOSE:
        {
            eamp_closespeaker();
            break;
        }
        case EAMP_SPEAKER_OPEN:
        {
            eamp_openspeaker();
            break;
        }
        case EAMP_HEADPHONE_CLOSE:
        {
            eamp_closeheadPhone();
            break;
        }
        case EAMP_HEADPHONE_OPEN:
        {
            eamp_openheadPhone();
            break;
        }
        case EAMP_EARPIECE_OPEN:
        {
            eamp_openEarpiece();
            break;
        }
        case EAMP_EARPIECE_CLOSE:
        {
            eamp_closeEarpiece();
            break;
        }
        case EAMP_GETREGISTER_VALUE:
        {
            return eamp_getRegister(args);
            break;
        }
        case EAMP_GETAMP_GAIN:
        {
            return eamp_getGainVolume();
            break;
        }
        case EAMP_SETAMP_GAIN:
        {
            eamp_changeGainVolume(args);
            break;
        }
        case EAMP_SETREGISTER_VALUE:
        {
            eamp_setRegister(args);
            break;
        }
        case EAMP_GET_CTRP_NUM:
        {
            return eamp_getCtrlPointNum();
            break;
        }
        case EAMP_GET_CTRP_BITS:
        {
            return eamp_getCtrPointBits(args);
            break;
        }
        case EAMP_GET_CTRP_TABLE:
        {
            eamp_getCtrlPointTable(args);
            break;
        }
        case EAMP_SETMODE:
        {
            eamp_setMode(args);
            break;
        }
// 20140627 let tpa2028d know current device for combo_path gain control - hyeonsang85.park@lge.com - start
        case EAMP_SETDEVICE:
        {
            eamp_setDevice(args);
            break;
        }
// 20140627 let tpa2028d know current device for combo_path gain control - hyeonsang85.park@lge.com - end
        default:
        return 0;
    }
    return 0;
}

int Audio_eamp_command(unsigned int type, unsigned long args, unsigned int count)
{
    return eamp_command(type,args,count);
}

#if 0
static int eamp_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, EAMP_I2C_DEVNAME);
    return 0;
}
#endif

static int eamp_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    int ret = 0;
    EAMP_PRINTK("eamp_i2c_probe success");

    new_client = client;
    ret = eamp_resetRegister();
    EAMP_PRINTK("client=%x !!",client);

    if(ret < 0) {
        new_client = NULL;
        EAMP_PRINTK("[ERROR] maybe, chip is died by .. !!",client);
        return -1;
    }else {
    return 0;
}
}

static int eamp_i2c_remove(struct i2c_client *client)
{
    EAMP_PRINTK("");
    new_client = NULL;
    i2c_unregister_device(client);
    i2c_del_driver(&eamp_i2c_driver);
    return 0;
}

static void eamp_poweron(void)
{
    return;
}

static void eamp_powerdown(void)
{
    EAMP_PRINTK("");
    eamp_closeheadPhone();
    eamp_closespeaker();

    return;
}

static int eamp_init()
{
#if USE_ANALOG_SWITCH
{
    int result;
    result = mt_set_gpio_mode(GPIO_AUDIO_SEL, GPIO_AUDIO_SEL_M_GPIO);
    EAMP_PRINTK("AUDIO_SEL GPIO Status : mt_set_gpio_mode %d\n", result);
    result = mt_set_gpio_pull_enable(GPIO_AUDIO_SEL, GPIO_PULL_DISABLE);
    EAMP_PRINTK("AUDIO_SEL GPIO Status : mt_set_gpio_pull_enable %d\n", result);
    result = mt_set_gpio_dir(GPIO_AUDIO_SEL, GPIO_DIR_OUT);
    EAMP_PRINTK("AUDIO_SEL GPIO Status : mt_set_gpio_dir %d\n", result);
    result = mt_set_gpio_out(GPIO_AUDIO_SEL, GPIO_OUT_ONE);
    EAMP_PRINTK("AUDIO_SEL GPIO Status : mt_set_gpio_out %d, %d\n", result, mt_get_gpio_out(GPIO_AUDIO_SEL));
}
#endif
    eamp_poweron();
    return 0;
}

static int eamp_deinit()
{
    EAMP_PRINTK("");
    eamp_powerdown();
    return 0;
}

static int eamp_register()
{
    EAMP_PRINTK("");
    i2c_register_board_info(SOUND_I2C_CHANNEL,&eamp_dev,1);
    if (i2c_add_driver(&eamp_i2c_driver)){
        EAMP_PRINTK("fail to add device into i2c");
        return -1;
    }
    return 0;
}

/*LGE_CHANGE ew0804.kim add register setting api */
static int eamp_write_procmem(struct file *file, const char __user *buffer,
	unsigned long count, void *data)
{
	char *register_data;
	int addr;
	int value;
	int fail;

	register_data = (char*)data;
	copy_from_user(register_data, buffer, count);
	register_data[count] = '\0';
	if(register_data[count - 1] == '\n')
			register_data[count - 1] = '\0';

	sscanf(register_data, "%d %d", &addr, &value);
	EAMP_PRINTK("S register_data=%s %d %d\n", register_data, addr, value);
	if((addr >= EAMP_REG_SUBSYSTEMCONTROL) && (addr <= EMPA_REG_HEADPHONE_RIGHT_VOLUME)){
		if((EMPA_REG_SPEAKER_VOLUME | EMPA_REG_HEADPHONE_LEFT_VOLUME | EMPA_REG_HEADPHONE_RIGHT_VOLUME) & addr){
			if(value >= -28 && value <= 30)
				register_setting_mode = value;
			else{
				EAMP_PRINTK("S Wrong value\n");
				return count;
			}
		}
		fail |= I2CWrite(addr, value);
		EAMP_PRINTK("S register Write Done Gain=%d\n", I2CRead(addr));
	}
	else
		EAMP_PRINTK("S Wrong addr or value\n");

	return count;
}
/*LGE_CHANGE ew0804.kim add register setting api */


static int eamp_read_procmem(char *buf, char **start, off_t offset,
        int count , int *eof, void *data)
{
    int len = 0;
    u8 val;
    EAMP_PRINTK("S\n");

    len += sprintf(buf + len , "Audio Select GPIO (SPK/RCV) =%s\n",
                    SPK_RCV_SEL_STR(mt_get_gpio_out(GPIO_AUDIO_SEL)));

    eamp_read_byte(EAMP_REG_SUBSYSTEMCONTROL, &val);
    len += sprintf(buf + len , "Subsystem Control =0x%x\n", val);
    len += sprintf(buf + len ,
                "\tBYPASS %s, SWS %s, SSM_EN %s, SPK_Fault %s, Thermal %s\n",
                ON_OFF_STR(val & SPK_BYPASSMODE_HIGH),
                ON_OFF_STR(val & SHUTDOWN_MODE_HIGH),
                ON_OFF_STR(val & SPREAD_SPECTRUM_HIGH),
                ON_OFF_STR(val & SPEAKER_FAULT_HIGH),
                ON_OFF_STR(val & THERMAL_CONDITION_HIGH));

    eamp_read_byte(EMPA_REG_INPUTCONTROL, &val);
    len += sprintf(buf + len , "Input Mode/Gain Control =0x%x\n", val);
    len += sprintf(buf + len ,
                "\tIN1 - Mode : %s, Gain : %d\n",
                IN_MODE_STR(val & CH1_DIFF_INPUT),
                REG_VALUE(CH1_GAIN_LEVEL, val));
    len += sprintf(buf + len ,
                "\tIN2 - Mode : %s, Gain : %d\n",
                IN_MODE_STR(val & CH2_DIFF_INPUT),
                REG_VALUE(CH2_GAIN_LEVEL, val));


    eamp_read_byte(EMPA_REG_LIMITER_CONTROL, &val);
    len += sprintf(buf + len , "Release/Attack Time =0x%x\n", val);
    len += sprintf(buf + len ,
                "\tRelease Time : %d, Attack Time : %d\n",
                REG_VALUE(REL_TIME, val), REG_VALUE(ATK_TIME, val));


    eamp_read_byte(EMPA_REG_SPEAKER_OUTPUT_CONTROL, &val);
    len += sprintf(buf + len , "Speaker Control =0x%x\n", val);
    len += sprintf(buf + len ,
                "\tSPK Out : %d, SPK Limiter : %s, SPK Limiter Level : %d\n",
                REG_VALUE(SPK_OUT, val),
                ON_OFF_STR(val & SPK_LIMITER_ENABLE),
                REG_VALUE(SPK_LIMITER_LEVEL, val));

    eamp_read_byte(EMPA_REG_HEADPHONE_OUTPUT_CONTROL, &val);
    len += sprintf(buf + len , "Headphone Control =0x%x\n", val);
    len += sprintf(buf + len ,
                "\tHPH Out : %d, HPH Limiter : %s, HPH Limiter Level : %d\n",
                REG_VALUE(HPH_OUT, val),
                ON_OFF_STR(val & HPH_LIMITER_ENABLE),
                REG_VALUE(HPH_LIMITER_LEVEL, val));

    eamp_read_byte(EMPA_REG_SPEAKER_VOLUME, &val);
    len += sprintf(buf + len , "Speaker Volume =0x%x\n", val);
    len += sprintf(buf + len ,
                "\tEnable : %s, Volmume : %d\n",
                ON_OFF_STR(val & SPK_ENABLE),
                REG_VALUE(SPK_VOLUME, val));

    eamp_read_byte(EMPA_REG_HEADPHONE_LEFT_VOLUME, &val);
    len += sprintf(buf + len , "HPH Left Channel Volume =0x%x\n", val);
    len += sprintf(buf + len ,
                "\tTracking : %s, Enable : %s, Volmume : %d\n",
                ON_OFF_STR(val & HPH_TRACK),
                ON_OFF_STR(val & HPH_ENABLE),
                REG_VALUE(HPH_VOLUME, val));

    eamp_read_byte(EMPA_REG_HEADPHONE_RIGHT_VOLUME, &val);
    len += sprintf(buf + len , "HPH Right Channel Volume =0x%x\n", val);
    len += sprintf(buf + len ,
                "\tEnable : %s, Volmume : %d\n",
                ON_OFF_STR(val & HPH_ENABLE),
                REG_VALUE(HPH_VOLUME, val));

    EAMP_PRINTK("E\n");
    return len;
}


/*****************************************************************************
*                  F U N C T I O N        D E F I N I T I O N
******************************************************************************
*/
extern void Yusu_Sound_AMP_Switch(BOOL enable);

bool Speaker_Init(void)
{
    EAMP_PRINTK("");
    eamp_init();

	/*LGE_CHANGE ew0804.kim add register setting api */
	//create_proc_read_write_entry("audio_ext_amp", 0644, NULL,
		//eamp_write_procmem, eamp_read_procmem, proc_str);
	/*LGE_CHANGE ew0804.kim add register setting api */

    return true;
}

bool Speaker_Register(void)
{
    EAMP_PRINTK("");
    eamp_register();
    return true;
}

int ExternalAmp()
{
    return 1;
}

void Sound_SpeakerL_SetVolLevel(int level)
{
    EAMP_PRINTK("level=%d",level);
}

void Sound_SpeakerR_SetVolLevel(int level)
{
    EAMP_PRINTK("level=%d",level);
}

void Sound_Speaker_Turnon(int channel)
{
    EAMP_PRINTK("channel = %d",channel);
    eamp_command(EAMP_SPEAKER_OPEN,channel,1);
}

void Sound_Speaker_Turnoff(int channel)
{
    EAMP_PRINTK("channel = %d",channel);
    eamp_command(EAMP_SPEAKER_CLOSE,channel,1);
}

void Sound_Speaker_SetVolLevel(int level)
{

}

void Sound_Headset_Turnon(void)
{
    EAMP_PRINTK("");
}
void Sound_Headset_Turnoff(void)
{
    EAMP_PRINTK("");
}

//kernal use
void AudioAMPDevice_Suspend(void)
{
    EAMP_PRINTK("");
    eamp_suspend();
}

void AudioAMPDevice_Resume(void)
{
    EAMP_PRINTK("");
    eamp_resume();
}

// for AEE beep sound
void AudioAMPDevice_SpeakerLouderOpen(void)
{
    EAMP_PRINTK("");
    if(gsk_on && gMode != 2) //speaker on and not incall mode
        return;
    gsk_forceon = true;
    gPreMode = gMode;
    gsk_preon = gsk_on;
    gep_preon = gep_on;
    if(gsk_on)
    {
        eamp_closespeaker();
    }
    gMode = 0;
    eamp_openspeaker();
    return ;
}

// for AEE beep sound
void AudioAMPDevice_SpeakerLouderClose(void)
{
    EAMP_PRINTK("");
    if(gsk_forceon)
    {
        eamp_closespeaker();
        gMode = gPreMode;
        if(gep_preon)
        {
            eamp_openEarpiece();
        }
        else if(gsk_preon)
        {
            eamp_openspeaker();
        }
    }
    gsk_forceon = false;
}

// mute device when INIT_DL1_STREAM
void AudioAMPDevice_mute(void)
{
    if(ghp_on)
        eamp_closeheadPhone();
    if(gsk_on)
        eamp_closespeaker();
    // now not control earpiece.
}

bool Speaker_DeInit(void)
{
    eamp_deinit();
    return true;
}

#if defined(CONFIG_MTK_SOUND_EXT_AMP_DEBUG)

ssize_t Audio_eamp_I2C_Write(U8 addr, U8 writeData)
{
    return eamp_write_byte(addr, writeData);
}

ssize_t Audio_eamp_I2C_Read(U8 addr, U8 *returnData)
{
    return eamp_read_byte( addr, returnData);
}
#endif

static char *ExtFunArray[] =
{
    "InfoMATVAudioStart",
    "InfoMATVAudioStop",
    "End",
};

kal_int32 Sound_ExtFunction(const char* name, void* param, int param_size)
{
    int i = 0;
    int funNum = -1;

    //Search the supported function defined in ExtFunArray
    while(strcmp("End",ExtFunArray[i]) != 0 ) {        //while function not equal to "End"

        if (strcmp(name,ExtFunArray[i]) == 0 ) {        //When function name equal to table, break
            funNum = i;
            break;
        }
        i++;
    }

    switch (funNum) {
    case 0:            //InfoMATVAudioStart
        printk("InfoMATVAudioStart");
        break;

    case 1:            //InfoMATVAudioStop
        printk("InfoMATVAudioStop");
        break;

    default:
        break;
    }
    return 1;
}
