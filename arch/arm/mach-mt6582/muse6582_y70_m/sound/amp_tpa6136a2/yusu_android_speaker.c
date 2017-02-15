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
#include <asm/uaccess.h>
#include <linux/xlog.h>
#include <linux/i2c.h>
/* LGE_CHANGE_S : 20121031 mtk-sp-bsp@lge.com sound play volume change issue fix MTK patch ALPS00373024 */
#include <linux/delay.h>
/* LGE_CHANGE_E : 20121031 mtk-sp-bsp@lge.com sound play volume change issue fix MTK patch ALPS00373024 */
#include "yusu_android_speaker.h"
#include "cust_sound.h"
/* LGE_CHANGE_S : JB MP audio switch enable MTK released*/
#include <mach/mt_gpio.h>
/* LGE_CHANGE_E */


#include "cust_gpio_usage.h"



/*****************************************************************************
*                          DEBUG INFO
******************************************************************************
*/

static bool eamp_log_on = true;

#define EAMP_PRINTK(fmt, arg...) \
	do { \
		if (eamp_log_on) xlog_printk(ANDROID_LOG_INFO,"EAMP", "[EAMP]: %s() "fmt"\n", __func__,##arg); \
	}while (0)


/*****************************************************************************
*				For I2C defination
******************************************************************************
*/

//control point
#define AUDIO_CONTROL_POINT_NUM (5);

// speaker, earpiece, headphone status and path;
static bool ghp_on = false;
//volume and gain
static u8  ghplvol = 0x19;
static u8  ghprvol = 0x19;
/* LGE_CHANGE_S : JB MP 20120908 migration patch , dohwan.an@lge.com */
#if defined (CONFIG_MACH_LGE)
static u8  gch1gain = 0x0;
static u8  gch2gain = 0x0;
#else
static u8  gchgain = 0x0;
#endif
/* LGE_CHANGE_E */
//mode
static u32 gMode	 = 0;
static u32 gPreMode  = 0;

/* LGE_CHANGE_S : 20121031 mtk-sp-bsp@lge.com sound play volume change issue fix MTK patch ALPS00373024 */
//response time
static int const headphone_response_time = 12; //ms
/* LGE_CHANGE_E : 20121031 mtk-sp-bsp@lge.com sound play volume change issue fix MTK patch ALPS00373024 */
/* LGE_CHANGE_S : JB MP 20120908 migration patch , dohwan.an@lge.com */
#if defined (CONFIG_MACH_LGE)
//mask
typedef enum
{
    GAIN_MASK_HP	  = 0x1,
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
	EAMP_PRINTK("eamp_read_byte");

	return 0;
}


//write register
static ssize_t	eamp_write_byte(u8 addr, u8 writeData)
{
	EAMP_PRINTK("eamp_write_byte");

	return 0;
}

//*****************************subsystem control Register, functions to control bits*************************
//speaker bypass mode
static ssize_t eamp_set_bypass_mode(bool enable)
{
	EAMP_PRINTK("eamp_set_bypass_mode");

	return 0;
}

//Software Shutdown mode

static ssize_t eamp_set_sws_mode(bool deactivate)
{
	EAMP_PRINTK("eamp_set_sws_mode");

	return 0;
}

//input mode and volume control Register
static ssize_t eamp_clear_input_gain()
{
	EAMP_PRINTK("eamp_clear_input_gain");

	return 0;
}

// set input gain on channel 1 and 2
static ssize_t eamp_set_input_gain( u8 inGain)
{
	EAMP_PRINTK("eamp_set_input_gain");

	return 0;
}

/* LGE_CHANGE_S : JB MP 20120908 migration patch , dohwan.an@lge.com */
#if defined (CONFIG_MACH_LGE)
// set input gain on channel 1
static ssize_t eamp_set_input1_gain( u8 inGain)
{
	EAMP_PRINTK("eamp_set_input1_gain");

	return 0;
}

// set input gain on channel 2

static ssize_t eamp_set_input2_gain( u8 inGain)
{
	EAMP_PRINTK("eamp_set_input2_gain");

	return 0;
}
#endif
/* LGE_CHANGE_E */

//  set input mode on channel 1 and 2.  0 for single-end inputs, 1 for differential inputs.
static ssize_t eamp_set_input_mode( bool in1se, bool in2se)
{
	EAMP_PRINTK("eamp_set_input_mode");

	return 0;
}

//Release and attack time
static ssize_t eamp_Release_attackTime_speed(u8 ATK_time, u8 REL_time)
{
	EAMP_PRINTK("eamp_Release_attackTime_speed");

	return 0;
}

//speaker mux and limiter

//set limiter level
static ssize_t eamp_set_speakerLimiter_level(u8 limitlev )
{
	EAMP_PRINTK("eamp_set_speakerLimiter_level");

	return 0;
}

// speaker limiter enable. 1 enable, 0 disable.
static ssize_t eamp_speakerLimiter_enable(bool enable )
{
	EAMP_PRINTK("eamp_speakerLimiter_enable");

	return 0;
}


// control for speaker channel.
static ssize_t eamp_set_speakerOut(u8  speakerout )
{
	EAMP_PRINTK("eamp_set_speakerOut");

	return 0;
}

//Headphone mux and limiter
// set headphone limiter level
static ssize_t eamp_set_headPhoneLimiter_level(u8 limitlev )
{
	EAMP_PRINTK("eamp_set_headPhoneLimiter_level");

	return 0;
}

// enable /disable headphone limiter
static ssize_t eamp_headPhoneLimiter_enable(bool enable )
{
	EAMP_PRINTK("eamp_headPhoneLimiter_enable");

	return 0;
}

// control for headphone channel
static ssize_t eamp_set_headPhoneOut(u8  headphoneout )
{
	EAMP_PRINTK("eamp_set_headPhoneOut");

	return 0;
}

//speaker volume

// openspeaker
static ssize_t eamp_set_speaker_Open(bool enable)
{
	EAMP_PRINTK("eamp_set_speaker_Open enable=%d",enable);
	if (enable == true)
	{
		//enable
	}
	else
	{
		//disable
	}
	return 0;
}

//set  speaker volume
static ssize_t eamp_set_speaker_vol(u8	vol)
{
	EAMP_PRINTK("eamp_set_speaker_vol vol=0x%x",vol);
	return 0;
}

//Headphone left channel volume
//enable headphone left  channel
static ssize_t eamp_set_headPhoneL_open( bool  enable )
{
	EAMP_PRINTK("eamp_set_headPhoneL_open enable=%d",enable);

	if (enable == true)
	{
		//enable
	}
	else
	{
		//disable
	}
	return 0;

}

//set  headphone  volume
static ssize_t eamp_set_headPhone_vol(u8 HP_vol)
{
	EAMP_PRINTK("eamp_set_headPhone_vol vol=0x%x",HP_vol);
	return 0;
}

//set  headphone left  volume
static ssize_t eamp_set_headPhone_lvol(u8 HPL_Vol)
{
	EAMP_PRINTK("eamp_set_headPhone_lvol vol=0x%x",HPL_Vol);
	return 0;
}

//Headphone right channel volume  register


//enable headphone Right  channel
static ssize_t eamp_set_headPhoneR_open( bool  enable )
{
	EAMP_PRINTK("eamp_set_headPhoneR_open enable=%d",enable);
	if (enable == true)
	{
		//enable
	}
	else
	{
		//disable
	}
	return 0;

}

//set  headphone right volume
static ssize_t eamp_set_headPhone_rvol(u8 HPR_Vol)
{
	EAMP_PRINTK("eamp_set_headPhone_rvol vol=0x%x",HPR_Vol);
	return 0;
}


//**********************************functions to control devices***********************************

// set registers to default value
static ssize_t eamp_resetRegister()
{
	EAMP_PRINTK("eamp_resetRegister");
	return 0;
}

static ssize_t eamp_openEarpiece()
{
	EAMP_PRINTK("eamp_openEarpiece");

	return 0;
}

static ssize_t eamp_closeEarpiece()
{
	EAMP_PRINTK("eamp_closeEarpiece");

	return 0;
}

static ssize_t eamp_openheadPhone()
{
	EAMP_PRINTK("eamp_openheadPhone");

	mt_set_gpio_out(GPIO_AUDIO_SEL, GPIO_OUT_ONE);

	if(ghplvol == ghprvol)
	{
/* LGE_CHANGE_S : dohwan.an@lge.com , for reduce pop-noise, rev1.0, TD 260828 */
		if (gMode == 2) {
			msleep(270); // for pop-noise
		}
	}
	else
	{
/* LGE_CHANGE_S : dohwan.an@lge.com , for reduce pop-noise, rev1.0, TD 260828 */
		if (gMode == 2) {
			msleep(270); // for pop-noise
		}
	}
	ghp_on = true;

/* LGE_CHANGE_S : 20121031 mtk-sp-bsp@lge.com sound play volume change issue fix MTK patch ALPS00373024 */
    msleep(headphone_response_time);
/* LGE_CHANGE_E : 20121031 mtk-sp-bsp@lge.com sound play volume change issue fix MTK patch ALPS00373024 */

	return 0;
}

static ssize_t eamp_closeheadPhone()
{
	EAMP_PRINTK("eamp_closeheadPhone");
	
	mt_set_gpio_out(GPIO_AUDIO_SEL, GPIO_OUT_ZERO);

	ghp_on = false;

	return 0;
}

static ssize_t eamp_openspeaker()
{
	EAMP_PRINTK("eamp_openspeaker");

	return 0;
}

static ssize_t eamp_closespeaker()
{
	EAMP_PRINTK("eamp_closespeaker");

	return 0;
}

static ssize_t eamp_changeGainVolume(unsigned long int param)
{
	EAMP_PRINTK("eamp_changeGainVolume param(0x%x)",param);
	return 0;
}

static ssize_t eamp_getGainVolume(void)
{
	EAMP_PRINTK("eamp_getGainVolume");
	return 0;
}

static ssize_t eamp_suspend()
{
	EAMP_PRINTK("eamp_suspend");
	eamp_resetRegister();
	return 0;
}

static ssize_t eamp_resume()
{
	EAMP_PRINTK("eamp_resume");

	if(ghp_on)
	{
		eamp_openheadPhone();
	}

	return 0;
}

static ssize_t eamp_getRegister(unsigned int regName)
{
	EAMP_PRINTK("eamp_getRegister Regname=%u",regName);

	return 0;
}

static ssize_t eamp_setRegister(unsigned long int param)
{
	EAMP_PRINTK("eamp_setRegister");

	return 0;
}


static ssize_t eamp_setMode(unsigned long int param)
{
	EAMP_PRINTK("eamp_setMode mode(%u)",param);
	gMode = param;
	return 0;
}

static ssize_t eamp_getCtrlPointNum()
{
	EAMP_PRINTK("eamp_getCtrlPointNum");
	return gCtrPointNum;
}

static ssize_t eamp_getCtrPointBits(unsigned long int param)
{
	EAMP_PRINTK("eamp_getCtrPointBits CtrPointBits(%u)",param);
	return gCtrPoint[param];
}

static ssize_t eamp_getCtrlPointTable(unsigned long int param)
{
	EAMP_PRINTK("eamp_getCtrlPointTable CtrlPointTable(0x%x)",param);
	AMP_Control *ampCtl = (AMP_Control*)param;
	if(copy_to_user((void __user *)ampCtl->param2,(void *)gCtrPoint_table[ampCtl->param1], 1<<gCtrPoint[ampCtl->param1])){
		return -1;
	}
	return 0;
}

static int eamp_command( unsigned int  type, unsigned long args,unsigned int count)
{
	EAMP_PRINTK("eamp_command type(%u)",type);
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
		}
		default:
		return 0;
	}
	return 0;
}

int Audio_eamp_command(unsigned int type, unsigned long args, unsigned int count)
{
	EAMP_PRINTK("Audio_eamp_command");

	return eamp_command(type,args,count);
}

static void eamp_poweron(void)
{
	EAMP_PRINTK("eamp_poweron");

	return;
}

static void eamp_powerdown(void)
{
	EAMP_PRINTK("eamp_powerdown");

	return;
}

static int eamp_init()
{
	EAMP_PRINTK("eamp_init");

	eamp_poweron();

	mt_set_gpio_mode(GPIO_AUDIO_SEL, GPIO_AUDIO_SEL_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_AUDIO_SEL, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_AUDIO_SEL, GPIO_DIR_OUT);
	
	return 0;
}

static int eamp_deinit()
{
	EAMP_PRINTK("eamp_deinit");
	eamp_powerdown();
	return 0;
}

static int eamp_register()
{
	EAMP_PRINTK("eamp_register");
	return 0;
}

/*****************************************************************************
*                  F U N C T I O N        D E F I N I T I O N
******************************************************************************
*/
extern void Yusu_Sound_AMP_Switch(BOOL enable);

bool Speaker_Init(void)
{
	EAMP_PRINTK("Speaker_Init");

	eamp_init();

	return true;
}

bool Speaker_Register(void)
{
	EAMP_PRINTK("Speaker_Register");

	eamp_register();

	return true;
}

int ExternalAmp()
{
	EAMP_PRINTK("ExternalAmp");

	return 1;
}

void Sound_SpeakerL_SetVolLevel(int level)
{
	EAMP_PRINTK("Sound_SpeakerL_SetVolLevel level=%d",level);
}

void Sound_SpeakerR_SetVolLevel(int level)
{
	EAMP_PRINTK("Sound_SpeakerR_SetVolLevel level=%d",level);
}

void Sound_Speaker_Turnon(int channel)
{
	EAMP_PRINTK("Sound_Speaker_Turnon channel = %d",channel);	
}

void Sound_Speaker_Turnoff(int channel)
{
	EAMP_PRINTK("Sound_Speaker_Turnoff channel = %d",channel);
}

void Sound_Speaker_SetVolLevel(int level)
{
	EAMP_PRINTK("Sound_Speaker_SetVolLevel");
}

void Sound_Headset_Turnon(void)
{
	EAMP_PRINTK("Sound_Headset_Turnon");
}

void Sound_Headset_Turnoff(void)
{
	EAMP_PRINTK("Sound_Headset_Turnoff");
}

//kernal use
void AudioAMPDevice_Suspend(void)
{
	EAMP_PRINTK("AudioAMPDevice_Suspend");
	eamp_suspend();
}

void AudioAMPDevice_Resume(void)
{
	EAMP_PRINTK("AudioAMPDevice_Resume");
	eamp_resume();
}

// for AEE beep sound
void AudioAMPDevice_SpeakerLouderOpen(void)
{
	EAMP_PRINTK("AudioAMPDevice_SpeakerLouderOpen");
}

// for AEE beep sound
void AudioAMPDevice_SpeakerLouderClose(void)
{
	EAMP_PRINTK("AudioAMPDevice_SpeakerLouderClose");
}

// mute device when INIT_DL1_STREAM
void AudioAMPDevice_mute(void)
{
	EAMP_PRINTK("AudioAMPDevice_mute");

	if(ghp_on)
		eamp_closeheadPhone();
	// now not control earpiece.
}

bool Speaker_DeInit(void)
{
	EAMP_PRINTK("Speaker_DeInit");

	eamp_deinit();
	return true;
}


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
	while(strcmp("End",ExtFunArray[i]) != 0 ) {		//while function not equal to "End"

		if (strcmp(name,ExtFunArray[i]) == 0 ) {		//When function name equal to table, break
			funNum = i;
			break;
		}
		i++;
	}

	switch (funNum) {
	case 0:			//InfoMATVAudioStart
		printk("InfoMATVAudioStart");
		break;

	case 1:			//InfoMATVAudioStop
		printk("InfoMATVAudioStop");
		break;

	default:
		break;
	}
	return 1;
}

