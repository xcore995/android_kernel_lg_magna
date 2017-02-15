/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <generated/autoconf.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/smp.h>

#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/random.h>
//#include <asm/system.h>
//#include <string.h>
//extern int rand(void);
#define RAND_MAX 20000000
#define MT6328_DEW_BASE 0x018C
#define MT6332_DEW_BASE 0x80F6

//#include <processor.h>
//#include <BusMonitor.h> //for high pri test
//#include "ts_pmic_wrap.h"
#include "pwrap_hal.h"
//#include "../../pmic_wrap/pwrap_hal.h"
#include <mach/mt_pmic_wrap.h>

//#include "reg_pmic.h"
//#include "reg_pmic_wrap.h"
//#include "mt_pmic_wrap.h"
//#include "tc_pwrap_ldvt.h"
#include "register_rw_tbl.h"

extern   void __pwrap_soft_reset(void);
extern int pwrap_of_iomap(void);
extern void pwrap_of_iounmap(void);

#define SLV_6328
//#define SLV_6332

extern S32 _pwrap_reset_spislv( void );
//static S32 pwrap_wacs2_nochk( U32 write, U32 adr, U32 wdata, U32 *rdata );

static struct mt_pmic_wrap_driver *mt_wrp;
struct mt_pwrap_dvt_t{
	U32 irq_mode;
	spinlock_t   wacs0_lock;
	spinlock_t   wacs1_lock;
	spinlock_t   wacs3_lock;
	void	(*complete)(void *context);
	void	*context;
};
enum pwrap_irq_mode{
	NORMAL_TEST,
	WDT_TEST,
	INT_TEST,
};
enum pwrap_test_case{
	PWRAP_DVT_UNSUPPORTED = -1,
	INIT=0,
	ACCESS,
	STATUS_UPDATE=2,
	DUAL_IO,
	REG_RW=4,
	MUX_SWITCH,
	SOFT_RESET=6,
	HIGH_PRI,
	ENCRYPTION=8,
	WDT,
	INTERRUPT=10,
	CONCURRENCE,
	INT_UPDATE=12,
	High_test_case=13,
	issue_test_case=14,
	PWRAP_DVT_MAX,
};
static struct mt_pwrap_dvt_t mt_wrp_dvt_obj={
	.irq_mode = NORMAL_TEST,
	.wacs0_lock = __SPIN_LOCK_UNLOCKED(lock),
	.wacs1_lock = __SPIN_LOCK_UNLOCKED(lock),
	.wacs3_lock = __SPIN_LOCK_UNLOCKED(lock),
};

extern spinlock_t	wrp_lock; 
static struct mt_pwrap_dvt_t *mt_wrp_dvt = &mt_wrp_dvt_obj;
/*-----start-- global variable-------------------------------------------------*/
DECLARE_COMPLETION(pwrap_done);


//#define WRAP_ACCESS_TEST_REG MT6328_DEW_WRITE_TEST
#define ldvt_follow_up
#define DEBUG_LDVT

extern S32 	pwrap_write_nochk( U32  adr, U32  wdata );
extern S32 	pwrap_read_nochk( U32  adr, U32 *rdata );

U32 eint_in_cpu0=0;
U32 eint_in_cpu1=2;
/*-pwrap debug--------------------------------------------------------------------------*/
static inline void pwrap_dump_ap_register(void)
{
#if 1
	U32 i=0;
	PWRAPREG("dump pwrap register, base=0x%p\n",PMIC_WRAP_BASE);
	PWRAPREG("address     :   3 2 1 0    7 6 5 4    B A 9 8    F E D C \n");
	for(i=0;i<=0x248;i+=16)
	{
		PWRAPREG("offset 0x%.3x:0x%.8x 0x%.8x 0x%.8x 0x%.8x \n",i,
		WRAP_RD32(PMIC_WRAP_BASE+i+0),
		WRAP_RD32(PMIC_WRAP_BASE+i+4),
		WRAP_RD32(PMIC_WRAP_BASE+i+8),
		WRAP_RD32(PMIC_WRAP_BASE+i+12));
	}
	//PWRAPREG("elapse_time=%llx(ns)\n",elapse_time);
#endif
	return;
}
static inline void pwrap_dump_pmic_register(void)
{
#if 0
		U32 i=0;
		U32 reg_addr=0;
		U32 reg_value=0;
	
		PWRAPREG("dump dewrap register\n");
		for(i=0;i<=14;i++)
		{
			reg_addr=(MT6328_DEW_BASE+i*2);
			pwrap_wacs2_nochk(0,reg_addr,0,&reg_value);
			PWRAPREG("MT6328 0x%x=0x%x\n",reg_addr,reg_value);
		}
		for(i=0;i<=14;i++)
		{
			reg_addr=(MT6332_DEW_BASE+i*2);
			pwrap_wacs2_nochk(0,reg_addr,0,&reg_value);
			PWRAPREG("MT6332 0x%x=0x%x\n",reg_addr,reg_value);
		}
#endif
	return;
}
static inline void pwrap_dump_all_register(void)
{
	pwrap_dump_ap_register();
	pwrap_dump_pmic_register();
	return;
}
/*
static void dvt__pwrap_soft_reset()
{
	PWRAPLOG("start reset wrapper\n");
	PWRAP_SOFT_RESET;
	PWRAPLOG("the reset register =%x\n",WRAP_RD32(INFRA_GLOBALCON_RST0));
	PWRAPLOG("PMIC_WRAP_STAUPD_GRPEN =0x%x,it should be equal to 0xc\n",WRAP_RD32(PMIC_WRAP_STAUPD_GRPEN));
	//clear reset bit
	PWRAP_CLEAR_SOFT_RESET_BIT;
	return;
}
*/

/******************************************************************************
  wrapper timeout
 ******************************************************************************/
#define PWRAP_TIMEOUT
#ifdef PWRAP_TIMEOUT
//#include <mach/mt_gpt.h>
static U64 _pwrap_get_current_time(void)
{
	return sched_clock(); 
}
//U64 elapse_time=0;

static BOOL _pwrap_timeout_ns (U64 start_time_ns, U64 timeout_time_ns)
{
	U64 cur_time=0;
	U64 elapse_time=0;

	// get current tick
	cur_time = _pwrap_get_current_time();//ns

	//avoid timer over flow exiting in FPGA env
	if(cur_time < start_time_ns){
		PWRAPERR("@@@@Timer overflow! start%lld cur timer%lld\n",start_time_ns,cur_time);
		start_time_ns=cur_time;
		timeout_time_ns=255*1000; //255us
		PWRAPERR("@@@@reset timer! start%lld setting%lld\n",start_time_ns,timeout_time_ns);
	}
		
	elapse_time=cur_time-start_time_ns;

	// check if timeout
	if (timeout_time_ns <= elapse_time)
	{
		// timeout
		PWRAPERR("@@@@Timeout: elapse time%lld,start%lld setting timer%lld\n",
				elapse_time,start_time_ns,timeout_time_ns);
		return TRUE;
	}
	return FALSE;
}
static U64 _pwrap_time2ns (U64 time_us)
{
	return time_us*1000;
}

#else
static U64 _pwrap_get_current_time(void)
{
	return 0;
}
static BOOL _pwrap_timeout_ns (U64 start_time_ns, U64 elapse_time)//,U64 timeout_ns)
{
	return FALSE;
}
static U64 _pwrap_time2ns (U64 time_us)
{
	return 0;
}

#endif
//#####################################################################
//define macro and inline function (for do while loop)
//#####################################################################
typedef U32 (*loop_condition_fp)(U32);//define a function pointer

static inline U32 wait_for_fsm_idle(U32 x)
{
	return (GET_WACS0_FSM( x ) != WACS_FSM_IDLE );
}
static inline U32 wait_for_fsm_vldclr(U32 x)
{
	return (GET_WACS0_FSM( x ) != WACS_FSM_WFVLDCLR);
}
static inline U32 wait_for_sync(U32 x)
{
	return (GET_SYNC_IDLE0(x) != WACS_SYNC_IDLE);
}
static inline U32 wait_for_idle_and_sync(U32 x)
{
	return ((GET_WACS2_FSM(x) != WACS_FSM_IDLE) || (GET_SYNC_IDLE2(x) != WACS_SYNC_IDLE)) ;
}
static inline U32 wait_for_wrap_idle(U32 x)
{
	return ((GET_WRAP_FSM(x) != 0x0) || (GET_WRAP_CH_DLE_RESTCNT(x) != 0x0));
}
static inline U32 wait_for_wrap_state_idle(U32 x)
{
	return ( GET_WRAP_AG_DLE_RESTCNT( x ) != 0 ) ;
}
static inline U32 wait_for_man_idle_and_noreq(U32 x)
{
	return ( (GET_MAN_REQ(x) != MAN_FSM_NO_REQ ) || (GET_MAN_FSM(x) != MAN_FSM_IDLE) );
}
static inline U32 wait_for_man_vldclr(U32 x)
{
	return  (GET_MAN_FSM( x ) != MAN_FSM_WFVLDCLR) ;
}
static inline U32 wait_for_cipher_ready(U32 x)
{
	return (x!=3) ;
}
static inline U32 wait_for_stdupd_idle(U32 x)
{
	return ( GET_STAUPD_FSM(x) != 0x0) ;
}
/*
static inline U32 wait_for_state_ready_init(loop_condition_fp fp,U32 timeout_us,U32 wacs_register,U32 *read_reg)
{

	U64 start_time_ns=0, timeout_ns=0;
	U32 reg_rdata=0x0;
	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do
	{
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
		{
			PWRAPERR("wait_for_state_ready_init timeout when waiting for idle\n");
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);
	} while( fp(reg_rdata)); //IDLE State
	if(read_reg)
		*read_reg=reg_rdata;
	return 0;
}
*/

static inline U32 wait_for_state_ready_init(loop_condition_fp fp,U32 timeout_us,void *wacs_register,U32 *read_reg)
{

	U64 start_time_ns=0, timeout_ns=0;
	U32 reg_rdata=0x0;
	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do
	{
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
		{
			PWRAPERR("wait_for_state_ready_init timeout when waiting for idle\n");
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);
	} while( fp(reg_rdata)); //IDLE State
	if(read_reg)
		*read_reg=reg_rdata;
	return 0;
}

static inline U32 wait_for_state_idle_init(loop_condition_fp fp,U32 timeout_us,void *wacs_register,void *wacs_vldclr_register,U32 *read_reg)
{

	U64 start_time_ns=0, timeout_ns=0;
	U32 reg_rdata;
	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do
	{
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
		{
			PWRAPERR("wait_for_state_idle_init timeout when waiting for idle\n");
			pwrap_dump_ap_register();
			//pwrap_trace_wacs2();
			//BUG_ON(1);
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);
		//if last read command timeout,clear vldclr bit
		//read command state machine:FSM_REQ-->wfdle-->WFVLDCLR;write:FSM_REQ-->idle
		switch ( GET_WACS0_FSM( reg_rdata ) )
		{
			case WACS_FSM_WFVLDCLR:
				WRAP_WR32(wacs_vldclr_register , 1);
				PWRAPERR("WACS_FSM = PMIC_WRAP_WACS_VLDCLR\n");
				break;
			case WACS_FSM_WFDLE:
				PWRAPERR("WACS_FSM = WACS_FSM_WFDLE\n");
				break;
			case WACS_FSM_REQ:
				PWRAPERR("WACS_FSM = WACS_FSM_REQ\n");
				break;
			default:
				break;
		}
	}while( fp(reg_rdata)); //IDLE State
	if(read_reg)
		*read_reg=reg_rdata;
	return 0;
}


/*
static inline U32 wait_for_state_idle_init(loop_condition_fp fp,U32 timeout_us,U32 wacs_register,U32 wacs_vldclr_register,U32 *read_reg)
{

	U64 start_time_ns=0, timeout_ns=0;
	U32 reg_rdata;
	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do
	{
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
		{
			PWRAPERR("wait_for_state_idle_init timeout when waiting for idle\n");
			pwrap_dump_ap_register();
			//pwrap_trace_wacs2();
			//BUG_ON(1);
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);
		//if last read command timeout,clear vldclr bit
		//read command state machine:FSM_REQ-->wfdle-->WFVLDCLR;write:FSM_REQ-->idle
		switch ( GET_WACS0_FSM( reg_rdata ) )
		{
			case WACS_FSM_WFVLDCLR:
				WRAP_WR32(wacs_vldclr_register , 1);
				PWRAPERR("WACS_FSM = PMIC_WRAP_WACS_VLDCLR\n");
				break;
			case WACS_FSM_WFDLE:
				PWRAPERR("WACS_FSM = WACS_FSM_WFDLE\n");
				break;
			case WACS_FSM_REQ:
				PWRAPERR("WACS_FSM = WACS_FSM_REQ\n");
				break;
			default:
				break;
		}
	}while( fp(reg_rdata)); //IDLE State
	if(read_reg)
		*read_reg=reg_rdata;
	return 0;
}
*/

static inline U32 wait_for_state_idle(loop_condition_fp fp,U32 timeout_us,void *wacs_register,void *wacs_vldclr_register,U32 *read_reg)
{

	U64 start_time_ns=0, timeout_ns=0;
	U32 reg_rdata;
	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do
	{
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
		{
			PWRAPERR("wait_for_state_idle timeout when waiting for idle\n");
			pwrap_dump_ap_register();
			//pwrap_trace_wacs2();
			//BUG_ON(1);
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);
		if( GET_INIT_DONE0( reg_rdata ) != WACS_INIT_DONE)
		{
			PWRAPERR("initialization isn't finished \n");
			return E_PWR_NOT_INIT_DONE;
		}
		//if last read command timeout,clear vldclr bit
		//read command state machine:FSM_REQ-->wfdle-->WFVLDCLR;write:FSM_REQ-->idle
		switch ( GET_WACS0_FSM( reg_rdata ) )
		{
			case WACS_FSM_WFVLDCLR:
				WRAP_WR32(wacs_vldclr_register , 1);
				PWRAPERR("WACS_FSM = PMIC_WRAP_WACS_VLDCLR\n");
				break;
			case WACS_FSM_WFDLE:
				PWRAPERR("WACS_FSM = WACS_FSM_WFDLE\n");
				break;
			case WACS_FSM_REQ:
				PWRAPERR("WACS_FSM = WACS_FSM_REQ\n");
				break;
			default:
				break;
		}
	}while( fp(reg_rdata)); //IDLE State
	if(read_reg)
		*read_reg=reg_rdata;
	return 0;
}


/*
static inline U32 wait_for_state_idle(loop_condition_fp fp,U32 timeout_us,U32 wacs_register,U32 wacs_vldclr_register,U32 *read_reg)
{

	U64 start_time_ns=0, timeout_ns=0;
	U32 reg_rdata;
	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do
	{
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
		{
			PWRAPERR("wait_for_state_idle timeout when waiting for idle\n");
			pwrap_dump_ap_register();
			//pwrap_trace_wacs2();
			//BUG_ON(1);
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);
		if( GET_INIT_DONE0( reg_rdata ) != WACS_INIT_DONE)
		{
			PWRAPERR("initialization isn't finished \n");
			return E_PWR_NOT_INIT_DONE;
		}
		//if last read command timeout,clear vldclr bit
		//read command state machine:FSM_REQ-->wfdle-->WFVLDCLR;write:FSM_REQ-->idle
		switch ( GET_WACS0_FSM( reg_rdata ) )
		{
			case WACS_FSM_WFVLDCLR:
				WRAP_WR32(wacs_vldclr_register , 1);
				PWRAPERR("WACS_FSM = PMIC_WRAP_WACS_VLDCLR\n");
				break;
			case WACS_FSM_WFDLE:
				PWRAPERR("WACS_FSM = WACS_FSM_WFDLE\n");
				break;
			case WACS_FSM_REQ:
				PWRAPERR("WACS_FSM = WACS_FSM_REQ\n");
				break;
			default:
				break;
		}
	}while( fp(reg_rdata)); //IDLE State
	if(read_reg)
		*read_reg=reg_rdata;
	return 0;
}

*/
	
	static inline U32 wait_for_state_ready(loop_condition_fp fp,U32 timeout_us,void *wacs_register,U32 *read_reg)
	{
	
		U64 start_time_ns=0, timeout_ns=0;
		U32 reg_rdata;
		start_time_ns = _pwrap_get_current_time();
		timeout_ns = _pwrap_time2ns(timeout_us);
		do
		{
			if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
			{
				PWRAPERR("timeout when waiting for idle\n");
				pwrap_dump_ap_register();
				//pwrap_trace_wacs2();
				return E_PWR_WAIT_IDLE_TIMEOUT;
			}
			reg_rdata = WRAP_RD32(wacs_register);
	
			if( GET_INIT_DONE0( reg_rdata ) != WACS_INIT_DONE)
			{
				PWRAPERR("initialization isn't finished \n");
				return E_PWR_NOT_INIT_DONE;
			}
		} while( fp(reg_rdata)); //IDLE State
		if(read_reg)
			*read_reg=reg_rdata;
		return 0;
	}

/*
static inline U32 wait_for_state_ready(loop_condition_fp fp,U32 timeout_us,U32 wacs_register,U32 *read_reg)
{

	U64 start_time_ns=0, timeout_ns=0;
	U32 reg_rdata;
	start_time_ns = _pwrap_get_current_time();
	timeout_ns = _pwrap_time2ns(timeout_us);
	do
	{
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
		{
			PWRAPERR("timeout when waiting for idle\n");
			pwrap_dump_ap_register();
			//pwrap_trace_wacs2();
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
		reg_rdata = WRAP_RD32(wacs_register);

		if( GET_INIT_DONE0( reg_rdata ) != WACS_INIT_DONE)
		{
			PWRAPERR("initialization isn't finished \n");
			return E_PWR_NOT_INIT_DONE;
		}
	} while( fp(reg_rdata)); //IDLE State
	if(read_reg)
		*read_reg=reg_rdata;
	return 0;
}

*/
#if 0
static  void pwrap_delay_us(U32 us)
{
	udelay(us);
}
#endif
static inline void pwrap_complete(void *arg)
{
	complete(arg);
}
#if 0
static S32 pwrap_wacs2_nochk( U32 write, U32 adr, U32 wdata, U32 *rdata )
{
	U32 reg_rdata=0x0;
	U32 wacs_write=0x0;
	U32 wacs_adr=0x0;
	U32 wacs_cmd=0x0;
	U32 return_value=0x0;
	//PWRAPFUC();
	// Check argument validation
	if( (write & ~(0x1))    != 0)  return E_PWR_INVALID_RW;
	if( (adr   & ~(0xffff)) != 0)  return E_PWR_INVALID_ADDR;
	if( (wdata & ~(0xffff)) != 0)  return E_PWR_INVALID_WDAT;

	// Check IDLE
	return_value=wait_for_state_ready_init(wait_for_fsm_idle,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS2_RDATA,0);
	if(return_value!=0)
	{
		PWRAPERR("_pwrap_wacs2_nochk write command fail,return_value=%x\n", return_value);
		return return_value;
	}

	wacs_write  = write << 31;
	wacs_adr    = (adr >> 1) << 16;
	wacs_cmd = wacs_write | wacs_adr | wdata;
	WRAP_WR32(PMIC_WRAP_WACS2_CMD,wacs_cmd);

	if( write == 0 )
	{
		if (NULL == rdata)
			return E_PWR_INVALID_ARG;
		// wait for read data ready
		return_value=wait_for_state_ready_init(wait_for_fsm_vldclr,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS2_RDATA,&reg_rdata);
		if(return_value!=0)
		{
			PWRAPERR("_pwrap_wacs2_nochk read fail,return_value=%x\n", return_value);
			return return_value;
		}
		*rdata = GET_WACS0_RDATA( reg_rdata );
		WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR , 1);
	}
	return 0;
}
#endif

//--------------------------------------------------------
//    Function : pwrap_wacs0()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
S32 pwrap_wacs0( U32 write, U32 adr, U32 wdata, U32 *rdata )
{
	U32 reg_rdata=0;
	U32 wacs_write=0;
	U32 wacs_adr=0;
	U32 wacs_cmd=0;
	U32 return_value=0;
	unsigned long flags=0;
	//PWRAPFUC();
	//PWRAPLOG("wrapper access,write=%x,add=%x,wdata=%x,rdata=%x\n",write,adr,wdata,rdata);

	// Check argument validation
	if( (write & ~(0x1))    != 0)  return E_PWR_INVALID_RW;
	if( (adr   & ~(0xffff)) != 0)  return E_PWR_INVALID_ADDR;
	if( (wdata & ~(0xffff)) != 0)  return E_PWR_INVALID_WDAT;

	spin_lock_irqsave(&mt_wrp_dvt->wacs0_lock,flags);
	//spin_lock_irqsave(&wrp_lock,flags);
	// Check IDLE & INIT_DONE in advance
	return_value=wait_for_state_idle(wait_for_fsm_idle,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS0_RDATA,PMIC_WRAP_WACS0_VLDCLR,0);
	if(return_value!=0)
	{
		PWRAPERR("wait_for_fsm_idle fail,return_value=%d\n",return_value);
		goto FAIL;
	}

	wacs_write  = write << 31;
	wacs_adr    = (adr >> 1) << 16;
	wacs_cmd= wacs_write | wacs_adr | wdata;

	WRAP_WR32(PMIC_WRAP_WACS0_CMD,wacs_cmd);
	if( write == 0 )
	{
		if (NULL == rdata)
		{
			PWRAPERR("rdata is a NULL pointer\n");
			return_value= E_PWR_INVALID_ARG;
			goto FAIL;
		}
		return_value=wait_for_state_ready(wait_for_fsm_vldclr,TIMEOUT_READ,PMIC_WRAP_WACS0_RDATA,&reg_rdata);
		if(return_value!=0)
		{
			PWRAPERR("wait_for_fsm_vldclr fail,return_value=%d\n",return_value);
			return_value+=1;//E_PWR_NOT_INIT_DONE_READ or E_PWR_WAIT_IDLE_TIMEOUT_READ
			goto FAIL;
		}
		*rdata = GET_WACS0_RDATA( reg_rdata );
		WRAP_WR32(PMIC_WRAP_WACS0_VLDCLR , 1);
	}
FAIL:
	spin_unlock_irqrestore(&mt_wrp_dvt->wacs0_lock,flags);
	//spin_unlock_irqrestore(&wrp_lock,flags);
	return return_value;
}

//--------------------------------------------------------
//    Function : pwrap_wacs1()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
S32 pwrap_wacs1( U32  write, U32  adr, U32  wdata, U32 *rdata )
{
	U32 reg_rdata=0;
	U32 wacs_write=0;
	U32 wacs_adr=0;
	U32 wacs_cmd=0;
	U32 return_value=0;
	unsigned long flags=0;
	//PWRAPFUC();

	// Check argument validation
	if( (write & ~(0x1))    != 0)  return E_PWR_INVALID_RW;
	if( (adr   & ~(0xffff)) != 0)  return E_PWR_INVALID_ADDR;
	if( (wdata & ~(0xffff)) != 0)  return E_PWR_INVALID_WDAT;

	spin_lock_irqsave(&mt_wrp_dvt->wacs1_lock,flags);
	//spin_lock_irqsave(&wrp_lock,flags);
	// Check IDLE & INIT_DONE in advance
	return_value=wait_for_state_idle(wait_for_fsm_idle,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS1_RDATA,PMIC_WRAP_WACS1_VLDCLR,0);
	if(return_value!=0)
	{
		PWRAPERR("wait_for_fsm_idle fail,return_value=%d\n",return_value);
		goto FAIL;
	}
	// Argument process
	wacs_write  = write << 31;
	wacs_adr    = (adr >> 1) << 16;
	wacs_cmd= wacs_write | wacs_adr | wdata;
	//send command
	WRAP_WR32(PMIC_WRAP_WACS1_CMD,wacs_cmd);
	if( write == 0 )
	{
		if (NULL == rdata)
		{
			PWRAPERR("rdata is a NULL pointer\n");
			return_value= E_PWR_INVALID_ARG;
			goto FAIL;
		}
		return_value=wait_for_state_ready(wait_for_fsm_vldclr,TIMEOUT_READ,PMIC_WRAP_WACS1_RDATA,&reg_rdata);
		if(return_value!=0)
		{
			PWRAPERR("wait_for_fsm_vldclr fail,return_value=%d\n",return_value);
			return_value+=1;//E_PWR_NOT_INIT_DONE_READ or E_PWR_WAIT_IDLE_TIMEOUT_READ
			goto FAIL;
		}
		*rdata = GET_WACS0_RDATA( reg_rdata );
		WRAP_WR32(PMIC_WRAP_WACS1_VLDCLR , 1);
	}
FAIL:
	spin_unlock_irqrestore(&mt_wrp_dvt->wacs1_lock,flags);
	//spin_unlock_irqrestore(&wrp_lock,flags);
	return return_value;

}


S32 pwrap_wacs3( U32  write, U32  adr, U32  wdata, U32 *rdata )
{
	U32 reg_rdata=0;
	U32 wacs_write=0;
	U32 wacs_adr=0;
	U32 wacs_cmd=0;
	U32 return_value=0;
	unsigned long flags=0;
	//PWRAPFUC();

	// Check argument validation
	if( (write & ~(0x1))    != 0)  return E_PWR_INVALID_RW;
	if( (adr   & ~(0xffff)) != 0)  return E_PWR_INVALID_ADDR;
	if( (wdata & ~(0xffff)) != 0)  return E_PWR_INVALID_WDAT;

	spin_lock_irqsave(&mt_wrp_dvt->wacs3_lock,flags);
	//spin_lock_irqsave(&wrp_lock,flags);
	// Check IDLE & INIT_DONE in advance
	return_value=wait_for_state_idle(wait_for_fsm_idle,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS3_RDATA,PMIC_WRAP_WACS3_VLDCLR,0);
	if(return_value!=0)
	{
		PWRAPERR("wait_for_fsm_idle fail,return_value=%d\n",return_value);
		goto FAIL;
	}
	// Argument process
	wacs_write  = write << 31;
	wacs_adr    = (adr >> 1) << 16;
	wacs_cmd= wacs_write | wacs_adr | wdata;
	//send command
	WRAP_WR32(PMIC_WRAP_WACS3_CMD,wacs_cmd);
	if( write == 0 )
	{
		if (NULL == rdata)
		{
			PWRAPERR("rdata is a NULL pointer\n");
			return_value= E_PWR_INVALID_ARG;
			goto FAIL;
		}
		return_value=wait_for_state_ready(wait_for_fsm_vldclr,TIMEOUT_READ,PMIC_WRAP_WACS3_RDATA,&reg_rdata);
		if(return_value!=0)
		{
			PWRAPERR("wait_for_fsm_vldclr fail,return_value=%d\n",return_value);
			return_value+=1;//E_PWR_NOT_INIT_DONE_READ or E_PWR_WAIT_IDLE_TIMEOUT_READ
			goto FAIL;
		}
		*rdata = GET_WACS3_RDATA( reg_rdata );
		WRAP_WR32(PMIC_WRAP_WACS3_VLDCLR , 1);
	}
FAIL:
	spin_unlock_irqrestore(&mt_wrp_dvt->wacs3_lock,flags);
	//spin_unlock_irqrestore(&wrp_lock,flags);
	return return_value;

}


//    Function : _pwrap_switch_dio()
// Description :call it after pwrap_init, check init done
//   Parameter :
//      Return :
//--------------------------------------------------------
S32 _pwrap_switch_dio( U32 dio_en )
{

	 U32 arb_en_backup=0x0;
  U32 rdata=0x0;
  U32 return_value=0;

  //PWRAPFUC();
  arb_en_backup = WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);
  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , WACS2); // only WACS2
#ifdef SLV_6328  
	  pwrap_write_nochk(MT6328_DEW_DIO_EN, (dio_en));
#endif
#ifdef SLV_6332  
	  pwrap_write_nochk(MT6332_DEW_DIO_EN, (dio_en>>1));
#endif  

  // Check IDLE & INIT_DONE in advance
  return_value=wait_for_state_ready_init(wait_for_idle_and_sync,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS2_RDATA,0);
  if(return_value!=0)
  {
    PWRAPERR("_pwrap_init_dio fail,return_value=%x\n", return_value);
    return return_value;
  }
  //enable AP DIO mode
  WRAP_WR32(PMIC_WRAP_DIO_EN , dio_en);
  // Read Test
#ifdef SLV_6328  
	  pwrap_read_nochk(MT6328_DEW_READ_TEST, &rdata);
	  if( rdata != MT6328_DEFAULT_VALUE_READ_TEST )
	  {
		PWRAPERR("[Dio_mode][Read Test] fail,dio_en = %x, READ_TEST rdata=%x, exp=0x5aa5\n", dio_en, rdata);
		return E_PWR_READ_TEST_FAIL;
	  }
#endif
#ifdef SLV_6332  
	  pwrap_read_nochk(MT6332_DEW_READ_TEST, &rdata);
	  if( rdata != MT6332_DEFAULT_VALUE_READ_TEST )
	  {
		PWRAPERR("[Dio_mode][Read Test] fail,dio_en = %x, READ_TEST rdata=%x, exp=0xa55a\n", dio_en, rdata);
		return E_PWR_READ_TEST_FAIL;
	  }
#endif
  WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , arb_en_backup);
  return 0;
}
//--------------------------------------------------------
//    Function : DrvPWRAP_SwitchMux()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
S32 _pwrap_switch_mux( U32 mux_sel_new )
{
	U32 mux_sel_old=0;
	U32 rdata=0;
	U32 return_value=0;
	// return if no change is necessary
	mux_sel_old = WRAP_RD32(PMIC_WRAP_MUX_SEL);
	if( mux_sel_new == mux_sel_old )
		return;

	// disable OLD, wait OLD finish
	// switch MUX, then enable NEW
	if( mux_sel_new == 1 )
	{
		WRAP_WR32(PMIC_WRAP_WRAP_EN ,0);
		// Wait for WRAP to be in idle state, // and no remaining rdata to be received
		return_value=wait_for_state_ready_init(wait_for_wrap_idle,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WRAP_STA,0);
		if(return_value!=0)
			return return_value;
		WRAP_WR32(PMIC_WRAP_MUX_SEL , 1);
		WRAP_WR32(PMIC_WRAP_MAN_EN , 1);
	}
	else
	{
		WRAP_WR32(PMIC_WRAP_MAN_EN , 0);
		// Wait for WRAP to be in idle state, // and no remaining rdata to be received
		return_value=wait_for_state_ready_init(wait_for_man_idle_and_noreq,TIMEOUT_WAIT_IDLE,PMIC_WRAP_MAN_RDATA,0);
		if(return_value!=0)
			return return_value;

		WRAP_WR32(PMIC_WRAP_MUX_SEL , 0);
		WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);
	}

	return 0;
}



//--------------------------------------------------------
//    Function : _pwrap_enable_cipher()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
S32 _pwrap_enable_cipher( void )
{
	U32 arb_en_backup=0;
	U32 rdata=0;
	U32 cipher_ready=0;
	U32 return_value=0;
	U64 start_time_ns=0, timeout_ns=5000000;
	PWRAPFUC();
	arb_en_backup = WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN ,WACS2); // only WACS2


	//Make sure CIPHER engine is idle
#ifdef SLV_6328
	pwrap_write(MT6328_DEW_CIPHER_EN,   0x0);
#endif
#ifdef SLV_6332
	pwrap_write(MT6332_DEW_CIPHER_EN,   0x0);
#endif
	WRAP_WR32(PMIC_WRAP_CIPHER_EN   ,0);

	WRAP_WR32(PMIC_WRAP_CIPHER_MODE    , 0);
	WRAP_WR32(PMIC_WRAP_CIPHER_SWRST   , 1);
	WRAP_WR32(PMIC_WRAP_CIPHER_SWRST   , 0);
	WRAP_WR32(PMIC_WRAP_CIPHER_KEY_SEL , 1);
	WRAP_WR32(PMIC_WRAP_CIPHER_IV_SEL  , 2);
	WRAP_WR32(PMIC_WRAP_CIPHER_EN   , 1);

	//Config CIPHER @ MT6328
#ifdef SLV_6328
	pwrap_write(MT6328_DEW_CIPHER_SWRST,   0x1);
	pwrap_write(MT6328_DEW_CIPHER_SWRST,   0x0);
	pwrap_write(MT6328_DEW_CIPHER_KEY_SEL, 0x1);
	pwrap_write(MT6328_DEW_CIPHER_IV_SEL,  0x2);
	pwrap_write(MT6328_DEW_CIPHER_EN,   0x1);
#endif
	//Config CIPHER @ MT6332
#ifdef SLV_6328
	pwrap_write(MT6332_DEW_CIPHER_SWRST,   0x1);
	pwrap_write(MT6332_DEW_CIPHER_SWRST,   0x0);
	pwrap_write(MT6332_DEW_CIPHER_KEY_SEL, 0x1);
	pwrap_write(MT6332_DEW_CIPHER_IV_SEL,  0x2);
	pwrap_write(MT6332_DEW_CIPHER_EN,   0x1);
#endif
	//wait for cipher ready
	return_value=wait_for_state_ready_init(wait_for_cipher_ready,TIMEOUT_WAIT_IDLE,PMIC_WRAP_CIPHER_RDY,0);
	if(return_value!=0)
		return return_value;
#ifdef SLV_6328
	start_time_ns = _pwrap_get_current_time();
	do
	{
		pwrap_read(MT6328_DEW_CIPHER_RDY, &rdata);
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
		{
			PWRAPERR("timeout %lldms when waiting for ready\n",timeout_ns/1000000);
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
	} while( rdata != 0x1 ); //6328 cipher_ready
	pwrap_write(MT6328_DEW_CIPHER_MODE, 0x1);
#endif
	//PWRAPERR("wait %dns till ready\n",sched_clock()-start_time_ns);
#ifdef SLV_6332
	start_time_ns = _pwrap_get_current_time();
	do
	{
		pwrap_read(MT6332_DEW_CIPHER_RDY, &rdata);
		if (_pwrap_timeout_ns(start_time_ns, timeout_ns))
		{
			PWRAPERR("timeout %lldms when waiting for ready\n",timeout_ns/1000000);
			return E_PWR_WAIT_IDLE_TIMEOUT;
		}
	} while( rdata != 0x1 ); //6332 cipher_ready
	pwrap_write(MT6332_DEW_CIPHER_MODE, 0x1);
#endif
	//wait for wacs2 ready
	return_value=wait_for_state_ready_init(wait_for_idle_and_sync,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS2_RDATA,0);
	if(return_value!=0)
		return return_value;

	WRAP_WR32(PMIC_WRAP_CIPHER_MODE , 1);
#ifdef SLV_6328
	// Read Test
	pwrap_read(MT6328_DEW_READ_TEST,&rdata);
	if( rdata != MT6328_DEFAULT_VALUE_READ_TEST )
	{
		PWRAPERR("Enable Encryption [Read Test] fail, READ_TEST rdata=%x, exp=0x5aa5", rdata);
		return E_PWR_READ_TEST_FAIL;
	}
#endif
#ifdef SLV_6332
	pwrap_read(MT6332_DEW_READ_TEST,&rdata);
	if( rdata != MT6332_DEFAULT_VALUE_READ_TEST )
	{
		PWRAPERR("disable Encryption [Read Test] fail, EXT_READ_TEST rdata=%x, exp=0x5aa5", rdata);
		return E_PWR_READ_TEST_FAIL;
	}
#endif
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , arb_en_backup);
	return 0;
}



//--------------------------------------------------------
//    Function : _pwrap_disable_cipher()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
S32 _pwrap_disable_cipher( void )
{
	U32 arb_en_backup=0;
	U32 rdata=0;
	U32 return_value=0;
	PWRAPFUC();
	arb_en_backup = WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , WACS2); // only WACS2

	//[7:6]key_sel, [5:4]iv_sel, [3]swrst [2]load, [1]start, [0]mode
#ifdef SLV_6328
	pwrap_write(MT6328_DEW_CIPHER_MODE, 0x0);
#endif
#ifdef SLV_6332
	pwrap_write(MT6332_DEW_CIPHER_MODE, 0x0);
#endif
	//wait for wacs2 ready
	return_value=wait_for_state_ready_init(wait_for_idle_and_sync,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WACS2_RDATA,0);
	if(return_value!=0)
		return return_value;

	WRAP_WR32(PMIC_WRAP_CIPHER_MODE , 0);
	WRAP_WR32(PMIC_WRAP_CIPHER_EN , 0);
	WRAP_WR32(PMIC_WRAP_CIPHER_SWRST , 1);
	WRAP_WR32(PMIC_WRAP_CIPHER_SWRST , 0);

	// Read Test
#ifdef SLV_6328
	pwrap_read(MT6328_DEW_READ_TEST,&rdata);
	if( rdata != MT6328_DEFAULT_VALUE_READ_TEST )
	{
		PWRAPERR("disable Encryption [Read Test] fail, READ_TEST rdata=%x, exp=0x5aa5", rdata);
		return E_PWR_READ_TEST_FAIL;
	}
#endif 
#ifdef SLV_6332
	pwrap_read(MT6332_DEW_READ_TEST,&rdata);
	if( rdata != MT6332_DEFAULT_VALUE_READ_TEST )
	{
		PWRAPERR("disable Encryption [Read Test] fail, EXT_READ_TEST rdata=%x, exp=0x5aa5", rdata);
		return E_PWR_READ_TEST_FAIL;
	}
#endif
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , arb_en_backup);
	return 0;
}

//--------------------------------------------------------
//    Function : _pwrap_manual_mode()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
S32 _pwrap_manual_mode( U32  write, U32  op, U32  wdata, U32 *rdata )
{
	U32 reg_rdata=0;
	U32 man_write=0;
	U32 man_op=0;
	U32 man_cmd=0;
	U32 return_value=0;
	reg_rdata = WRAP_RD32(PMIC_WRAP_MAN_RDATA);
	if( GET_MAN_FSM( reg_rdata ) != 0) //IDLE State
		return E_PWR_NOT_IDLE_STATE;

	// check argument validation
	if( (write & ~(0x1))  != 0)  return E_PWR_INVALID_RW;
	if( (op    & ~(0x1f)) != 0)  return E_PWR_INVALID_OP_MANUAL;
	if( (wdata & ~(0xff)) != 0)  return E_PWR_INVALID_WDAT;

	man_write = write << 13;
	man_op    = op << 8;
	man_cmd = man_write | man_op | wdata;
	WRAP_WR32(PMIC_WRAP_MAN_CMD ,man_cmd);
	if( write == 0 )
	{
		//wait for wacs2 ready
		return_value=wait_for_state_ready_init(wait_for_man_vldclr,TIMEOUT_WAIT_IDLE,PMIC_WRAP_MAN_RDATA,&reg_rdata);
		if(return_value!=0)
			return return_value;

		*rdata = GET_MAN_RDATA( reg_rdata );
		WRAP_WR32(PMIC_WRAP_MAN_VLDCLR , 1);
	}
	return 0;
}

//--------------------------------------------------------
//    Function : _pwrap_manual_modeAccess()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------

int DrvPWRAP_Man( unsigned int write, unsigned int op, unsigned int wdata, int *rdata )
{
  int reg_rdata;
  int man_write;
  int man_op;

  reg_rdata = *PMIC_WRAP_MAN_RDATA;
  if( GET_MAN_FSM( reg_rdata ) != 0) //IDLE State
    return 0x10;

  // check argument validation
  if( (write & ~(0x1))  != 0)  return 0x11;
  if( (op    & ~(0x1f)) != 0)  return 0x12;
  if( (wdata & ~(0xff)) != 0)  return 0x13;

  man_write = write << 13;
  man_op    = op << 8;
  *PMIC_WRAP_MAN_CMD = man_write | man_op | wdata;

  if( write == 0 )
  {
    do
    {
      reg_rdata = *PMIC_WRAP_MAN_RDATA;
    } while( GET_MAN_FSM( reg_rdata ) != 6 ); //WFVLDCLR

    *rdata = GET_MAN_RDATA( reg_rdata );
    *PMIC_WRAP_MAN_VLDCLR = 1;
  }

  return 0;
}

void DrvPWRAP_SwitchMux( int mux_sel_new )
{
  int mux_sel_old;
  int rdata;

  // return if no change is necessary
  mux_sel_old = *PMIC_WRAP_MUX_SEL;
  if( mux_sel_new == mux_sel_old )
  {  return;  }

  // disable OLD, wait OLD finish
  // switch MUX, then enable NEW
  if( mux_sel_new == 1 )
  {
    *PMIC_WRAP_WRAP_EN = 0;
    do
    {
      // Wait for WRAP to be in idle state,
      // and no remaining rdata to be received
      rdata = *PMIC_WRAP_WRAP_STA;
    } while( (GET_WRAP_FSM(rdata) != 0x0) ||
             (GET_WRAP_CH_DLE_RESTCNT(rdata) != 0x0) );

    *PMIC_WRAP_MUX_SEL = 1;
    *PMIC_WRAP_MAN_EN = 1;
  }
  else
  {
    *PMIC_WRAP_MAN_EN = 0;
    do
    {
      // Wait for MAN to be in idle state,
      // and no remaining rdata to be received
      rdata = *PMIC_WRAP_MAN_RDATA;
    } while( (GET_MAN_REQ(rdata) != 0x0) ||
             (GET_MAN_FSM(rdata) != 0x0) );

    *PMIC_WRAP_MUX_SEL = 0;
    *PMIC_WRAP_WRAP_EN = 1;
  }

  return ;
}


int DrvPWRAP_ManAccess( unsigned int write, unsigned int adr, unsigned int wdata, int *rdata )
{
  unsigned int man_wdata;
  int man_rdata;

  // check argument validation
  if( (write & ~(0x1))    != 0)      return 0x10;
  if( (adr   & ~(0xffff)) != 0)      return 0x11;
  if( (wdata & ~(0xffff)) != 0)      return 0x12;
  if( (adr   &   0x8000)  == 0x8000) return 0x13;  // not support manual access to PMIC 2

  DrvPWRAP_SwitchMux(1);
  DrvPWRAP_Man(OP_WR,  OP_CSH,  0, &man_rdata);
  DrvPWRAP_Man(OP_WR,  OP_CSL,  0, &man_rdata);
  man_wdata = (adr >> 9) | (write << 6);
  DrvPWRAP_Man(OP_WR,  OP_OUTD, (man_wdata & 0xff), &man_rdata);
  man_wdata = adr >> 1;
  DrvPWRAP_Man(OP_WR,  OP_OUTD, (man_wdata & 0xff), &man_rdata);

  if( write == 1 )
  {
    man_wdata = wdata >> 8;
    DrvPWRAP_Man(OP_WR,  OP_OUTD, (man_wdata & 0xff), &man_rdata);
    man_wdata = wdata;
    DrvPWRAP_Man(OP_WR,  OP_OUTD, (man_wdata & 0xff), &man_rdata);
  }
  else
  {
    DrvPWRAP_Man(OP_WR,  OP_CK,  8, &man_rdata);
    DrvPWRAP_Man(OP_RD,  OP_IND, 0, &man_rdata);
    *rdata = GET_MAN_RDATA( man_rdata ) << 8;
    DrvPWRAP_Man(OP_RD,  OP_IND, 0, &man_rdata);
    *rdata |= GET_MAN_RDATA( man_rdata );
  }
  DrvPWRAP_Man(OP_WR,  OP_CSL,  0, &man_rdata);
  DrvPWRAP_Man(OP_WR,  OP_CSH,  0, &man_rdata);
  DrvPWRAP_Man(OP_WR,  OP_CK,  0, &man_rdata); // EXT_CK

  return 0;
}


S32 _pwrap_manual_modeAccess( U32  write, U32  adr, U32  wdata, U32 *rdata )
{
	U32  man_wdata=0;
	U32 man_rdata=0;

	// check argument validation
	if( (write & ~(0x1))    != 0)  return E_PWR_INVALID_RW;
	if( (adr   & ~(0xffff)) != 0)  return E_PWR_INVALID_ADDR;
	if( (wdata & ~(0xffff)) != 0)  return E_PWR_INVALID_WDAT;
	if( (adr   &   0x8000)  == 0x8000) return 0x13;  // not support manual access to PMIC 2

	_pwrap_switch_mux(1);
	_pwrap_manual_mode(OP_WR,  OP_CSH,  0, &man_rdata);
	_pwrap_manual_mode(OP_WR,  OP_CSL,  0, &man_rdata);
	man_wdata = (adr >> 9) | (write << 6);
	_pwrap_manual_mode(OP_WR,  OP_OUTD, (man_wdata & 0xff), &man_rdata);
	man_wdata = adr >> 1;
	_pwrap_manual_mode(OP_WR,  OP_OUTD, (man_wdata & 0xff), &man_rdata);
	if( write == 1 )
	{
		man_wdata = wdata>> 8;
		_pwrap_manual_mode(OP_WR,  OP_OUTD, (man_wdata & 0xff), &man_rdata);
		man_wdata = wdata;
		_pwrap_manual_mode(OP_WR,  OP_OUTD, (man_wdata & 0xff), &man_rdata);
	}
	else
	{
		_pwrap_manual_mode(OP_WR,  OP_CK, 8, &man_rdata);
		_pwrap_manual_mode(OP_RD,  OP_IND, 0, &man_rdata);
		*rdata = (GET_MAN_RDATA( man_rdata )<<8);
		_pwrap_manual_mode(OP_RD,  OP_IND, 0, &man_rdata);
		*rdata |= GET_MAN_RDATA( man_rdata );
	}
	_pwrap_manual_mode(OP_WR,  OP_CSL,  0, &man_rdata);
	_pwrap_manual_mode(OP_WR,  OP_CSH,  0, &man_rdata);
	_pwrap_manual_mode(OP_WR,  OP_CK,  0, &man_rdata);  //EXT_CK
	return 0;
}

//--------------------------------------------------------
//    Function : _pwrap_StaUpdTrig()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
#if 0
static S32 _pwrap_StaUpdTrig( S32 mode )
{
	U32 man_rdata=0;
	U32 reg_data=0;
	U32 return_value=0;

	//Wait for FSM to be IDLE
	return_value=wait_for_state_ready_init(wait_for_stdupd_idle,TIMEOUT_WAIT_IDLE,PMIC_WRAP_STAUPD_STA,0);
	if(return_value!=0)
		return return_value;

	//Trigger FSM
	WRAP_WR32(PMIC_WRAP_STAUPD_MAN_TRIG ,0x1);
	reg_data=WRAP_RD32(PMIC_WRAP_STAUPD_STA);
	//Check if FSM is in REQ
	if( GET_STAUPD_FSM(reg_data) != 0x2)
		return E_PWR_NOT_IDLE_STATE;

	// if mode==1, only return after new status is updated.
	if( mode == 1)
	{
		while( GET_STAUPD_FSM(reg_data) != 0x0); //IDLE State
	}

	return 0;
}
#endif

//--------------------------------------------------------
//    Function : _pwrap_AlignCRC()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
void _pwrap_AlignCRC( void )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 arb_en_backup=0;
	U32 staupd_prd_backup=0;
	U32 return_value=0;
	//Backup Configuration & Set New Ones
	arb_en_backup = WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , WACS2); // only WACS2
	staupd_prd_backup = WRAP_RD32(PMIC_WRAP_STAUPD_PRD);
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD , 0); //disable STAUPD

	// reset CRC
#ifdef SLV_6328
	pwrap_write(MT6328_DEW_CRC_SWRST,1);
#endif
#ifdef SLV_6332
	pwrap_write(MT6332_DEW_CRC_SWRST,1);
#endif
	WRAP_WR32(PMIC_WRAP_CRC_EN , 0);

	//Wait for FSM to be IDLE
	return_value=wait_for_state_ready_init(wait_for_wrap_state_idle,TIMEOUT_WAIT_IDLE,PMIC_WRAP_WRAP_STA,0);
	if(return_value!=0)
		return ;

	// Enable CRC
#ifdef SLV_6328
	pwrap_write(MT6328_DEW_CRC_SWRST, 0);
#endif
#ifdef SLV_6332
	pwrap_write(MT6332_DEW_CRC_SWRST, 0);
#endif
	WRAP_WR32(PMIC_WRAP_CRC_EN , 1);

	//restore Configuration
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD , staupd_prd_backup);
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , arb_en_backup);
}
//--------------wrap test API--------------------------------------------------------------------

//--------------------------------------------------------
//    Function : _pwrap_status_update_test()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
S32 _pwrap_status_update_test( int flag )
{
	U32 i, j;
	U32 rdata;
	PWRAPFUC();
if(1==flag) {   //disable interrupt of PMIC_WRAP
	//disable signature interrupt
	WRAP_WR32(PMIC_WRAP_INT_EN,0x0);
#ifdef SLV_6328
	pwrap_write(MT6328_DEW_WRITE_TEST, MT6328_WRITE_TEST_VALUE);
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN, 0x1);
#endif
#ifdef SLV_6332
	pwrap_write(MT6332_DEW_WRITE_TEST, MT6332_WRITE_TEST_VALUE);
    WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN, WRAP_RD32(PMIC_WRAP_STAUPD_GRPEN)|0x2);
#endif
	WRAP_WR32(PMIC_WRAP_SIG_ADR,(MT6332_DEW_WRITE_TEST<<16)|MT6328_DEW_WRITE_TEST);
  	WRAP_WR32(PMIC_WRAP_SIG_VALUE,(0xAA55<<16)|0xAA55);
	WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x3);

	mdelay(5000);//delay 5 seconds
	rdata=WRAP_RD32(PMIC_WRAP_SIG_ERRVAL);
#ifdef SLV_6328
	if( (rdata&0xFFFF) != (MT6328_WRITE_TEST_VALUE) )
	{
		PWRAPERR("[MT6328 only]_pwrap_status_update_test error,error code=%x, rdata=%x\n", 1, (rdata&0xFFFF));
		return 1;
	}
#endif
#ifdef SLV_6332
	if( (rdata>>16) != (MT6332_WRITE_TEST_VALUE) )
	{
		PWRAPERR("[MT6332 only]_pwrap_status_update_test error,error code=%x, rdata=%x\n", 1, (rdata>>16));
		return 1;
	}
#endif
	WRAP_WR32(PMIC_WRAP_SIG_VALUE,(MT6332_WRITE_TEST_VALUE<<16)|MT6328_WRITE_TEST_VALUE);//tha same as write test
	//clear sig_error interrupt flag bit
	WRAP_WR32(PMIC_WRAP_INT_CLR,1<<1);

	//enable signature interrupt
	WRAP_WR32(PMIC_WRAP_INT_EN,0x7ffffff9);
	WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x0);
	WRAP_WR32(PMIC_WRAP_SIG_ADR , (MT6332_DEW_CRC_VAL<<16 | MT6328_DEW_CRC_VAL));
}
else if(2==flag) { //enable interrupt of PMIC_WRAP
	WRAP_WR32(PMIC_WRAP_INT_EN,(3<<1)|WRAP_RD32(PMIC_WRAP_INT_EN));
#ifdef SLV_6328
	pwrap_write(MT6328_DEW_WRITE_TEST, MT6328_WRITE_TEST_VALUE);
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN, 0x1);
#endif
#ifdef SLV_6332
	pwrap_write(MT6332_DEW_WRITE_TEST, MT6332_WRITE_TEST_VALUE);
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN, WRAP_RD32(PMIC_WRAP_STAUPD_GRPEN)|0x2);
#endif
	WRAP_WR32(PMIC_WRAP_SIG_ADR,(MT6332_DEW_WRITE_TEST<<16)|MT6328_DEW_WRITE_TEST);
  	WRAP_WR32(PMIC_WRAP_SIG_VALUE,(0xAA55<<16)|0xAA55);
	WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x3);

	mdelay(3000);//delay 5 seconds

	//tigger interrupt of wrapper Now!!!
	PWRAPLOG("_pwrap_status_update_test PMIC_WRAP_INT_FLG=0x%x.\n",WRAP_RD32(PMIC_WRAP_INT_FLG));
	
	WRAP_WR32(PMIC_WRAP_SIG_VALUE,(MT6332_WRITE_TEST_VALUE<<16)|MT6328_WRITE_TEST_VALUE);//tha same as write test
	//clear sig_error interrupt flag bit
	WRAP_WR32(PMIC_WRAP_INT_CLR,1<<1);

	WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x0);
	WRAP_WR32(PMIC_WRAP_SIG_ADR , (MT6332_DEW_CRC_VAL<<16 | MT6328_DEW_CRC_VAL));
}else if(3==flag) { //disable STAUOPD_GRPEN
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN, 0x0);
#ifdef SLV_6328
	pwrap_write(MT6328_DEW_WRITE_TEST, MT6328_WRITE_TEST_VALUE);
#endif
#ifdef SLV_6332
	pwrap_write(MT6332_DEW_WRITE_TEST, MT6332_WRITE_TEST_VALUE);
#endif
	WRAP_WR32(PMIC_WRAP_SIG_ADR,(MT6332_DEW_WRITE_TEST<<16)|MT6328_DEW_WRITE_TEST);
	WRAP_WR32(PMIC_WRAP_SIG_VALUE,(0xAA55<<16)|0xAA55);
	WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x3);

	mdelay(5000);//delay 5 seconds

	//cannot tigger interrupt of wrapper Now!!!,because of update disable
	
	WRAP_WR32(PMIC_WRAP_SIG_VALUE,(MT6332_WRITE_TEST_VALUE<<16)|MT6328_WRITE_TEST_VALUE);//tha same as write test
	//clear sig_error interrupt flag bit
	WRAP_WR32(PMIC_WRAP_INT_CLR,1<<1);

	WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x0);
	WRAP_WR32(PMIC_WRAP_SIG_ADR , (MT6332_DEW_CRC_VAL<<16 | MT6328_DEW_CRC_VAL));
}
	return 0;
}
//--------------------------------------------------------
//    Function : _pwrap_wrap_access_test()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
static S32 _pwrap_wrap_access_test( void )
{
	U32 rdata=0;
	U32 res=0;
	U32 reg_value_backup=0;
	U32 return_value=0;
	PWRAPFUC();
	//###############################//###############################
	// Read/Write test using WACS0
	//###############################
	//clear sig_error interrupt test
	reg_value_backup=WRAP_RD32(PMIC_WRAP_INT_EN);
	WRAP_CLR_BIT(3<<1,PMIC_WRAP_INT_EN); //two PMICs
	PWRAPLOG("start test WACS0\n");
#ifdef SLV_6328
	return_value=pwrap_wacs0(0, MT6328_DEW_READ_TEST, 0, &rdata);
	if( rdata != MT6328_DEFAULT_VALUE_READ_TEST )
	{
		PWRAPERR("MT6328 read test error(using WACS0),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
	rdata=0;
	pwrap_wacs0(1, MT6328_DEW_WRITE_TEST, MT6328_WRITE_TEST_VALUE, &rdata);
	return_value=pwrap_wacs0(0, MT6328_DEW_WRITE_TEST, 0, &rdata);
	if( rdata != MT6328_WRITE_TEST_VALUE )
	{
		PWRAPERR("write test error(using WACS0),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
#endif

	rdata=0;
	//###############################//###############################
	// Read/Write test using WACS1
	//###############################
	
	PWRAPLOG("start test WACS1\n");
#ifdef SLV_6328
	return_value=pwrap_wacs1(0, MT6328_DEW_READ_TEST, 0, &rdata);
	if( rdata != MT6328_DEFAULT_VALUE_READ_TEST )
	{
		PWRAPERR("MT6328 read test error(using WACS1),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
	rdata=0;
	pwrap_wacs1(1, MT6328_DEW_WRITE_TEST, MT6328_WRITE_TEST_VALUE, &rdata);
	return_value=pwrap_wacs1(0, MT6328_DEW_WRITE_TEST, 0, &rdata);
	if( rdata != MT6328_WRITE_TEST_VALUE )
	{
		PWRAPERR("write test error(using WACS1),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}

#endif
	rdata=0;
	//###############################//###############################
	// Read/Write test using WACS2
	//###############################
	PWRAPLOG("start test WACS2\n");
#ifdef SLV_6328
		return_value=pwrap_wacs2(0, MT6328_DEW_READ_TEST, 0, &rdata);
		if( rdata != MT6328_DEFAULT_VALUE_READ_TEST )
		{
			PWRAPERR("MT6328 read test error(using WACS2),return_value=%x, rdata=%x\n", return_value, rdata);
			res+=1;
		}
		rdata=0;
		pwrap_wacs2(1, MT6328_DEW_WRITE_TEST, MT6328_WRITE_TEST_VALUE, &rdata);
		return_value=pwrap_wacs2(0, MT6328_DEW_WRITE_TEST, 0, &rdata);
		if( rdata != MT6328_WRITE_TEST_VALUE )
		{
			PWRAPERR("write test error(using WACS2),return_value=%x, rdata=%x\n", return_value, rdata);
			res+=1;
		}
#endif


		PWRAPLOG("start test WACS3\n");
#ifdef SLV_6328
			return_value=pwrap_wacs3(0, MT6328_DEW_READ_TEST, 0, &rdata);
			if( rdata != MT6328_DEFAULT_VALUE_READ_TEST )
			{
				PWRAPERR("MT6328 read test error(using WACS3),return_value=%x, rdata=%x\n", return_value, rdata);
				res+=1;
			}
			rdata=0;
			pwrap_wacs3(1, MT6328_DEW_WRITE_TEST, MT6328_WRITE_TEST_VALUE, &rdata);
			return_value=pwrap_wacs3(0, MT6328_DEW_WRITE_TEST, 0, &rdata);
			if( rdata != MT6328_WRITE_TEST_VALUE )
			{
				PWRAPERR("write test error(using WACS3),return_value=%x, rdata=%x\n", return_value, rdata);
				res+=1;
			}
#endif

	WRAP_WR32(PMIC_WRAP_INT_EN,reg_value_backup);
	return res;
}




//--------------------------------------------------------
//    Function : _pwrap_man_access_test()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------

static S32 _pwrap_man_access_test( void )
{
	U32 rdata=0;
	U32 res=0;
	U32 return_value=0;
	U32 reg_value_backup;
	U32 reg_value_backup_prd;
	PWRAPFUC();
	//_pwrap_switch_mux(1);
	//###############################//###############################
	// Read/Write test using manual mode
	//###############################
	reg_value_backup=WRAP_RD32(PMIC_WRAP_STAUPD_GRPEN);
	//WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,reg_value_backup & (~(0x1<<6)));
	reg_value_backup_prd=WRAP_RD32(PMIC_WRAP_STAUPD_PRD);
	//WRAP_WR32(PMIC_WRAP_STAUPD_PRD,1);
	
/*Manual mode only support MT6328, don't support MT6332, so only test MT6328*/
#ifdef SLV_6328
	return_value=DrvPWRAP_ManAccess(0, MT6328_DEW_READ_TEST, 0, &rdata);
	if( rdata != MT6328_DEFAULT_VALUE_READ_TEST )
	{
		/* TERR="Error: [ReadTest] fail, rdata=%x, exp=0x5aa5", rdata */
		PWRAPERR("read test error(using manual mode),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
	rdata=0;
	DrvPWRAP_ManAccess(1, WRAP_ACCESS_TEST_REG, MT6328_WRITE_TEST_VALUE, &rdata);
	return_value=DrvPWRAP_ManAccess(0, WRAP_ACCESS_TEST_REG, 0, &rdata);
	if( rdata != MT6328_WRITE_TEST_VALUE )
	{
		/* TERR="Error: [WriteTest] fail, rdata=%x, exp=0x1234", rdata*/
		PWRAPERR("[MT6328]write test error(using manual mode),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
	rdata=0;
#endif
	_pwrap_switch_mux(0);//wrap mode

	rdata=0;
	//MAN
	_pwrap_AlignCRC();
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,reg_value_backup );
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD,reg_value_backup_prd );

	return res;
}
//--------------------------------------------------------
//    Function : _pwrap_man_access_test()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
static U32 _pwrap_int_update_test( void )
{
	UINT32 ret=0;
	UINT32 res=0;
	U32 rdata=0;
	//step1: test one pmic INT 
#ifdef SLV_6328
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0x5);
	WRAP_WR32(PMIC_WRAP_EINT_STA0_ADR,MT6328_DEW_WRITE_TEST);
	pwrap_wacs2(1, MT6328_DEW_WRITE_TEST, 0x3, &rdata);
	pwrap_wacs2(0, MT6328_DEW_WRITE_TEST, 0, &rdata);
	//WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0x4);//only 6328
	mdelay(100);
	rdata = WRAP_RD32(PMIC_WRAP_EINT_STA);
	if(rdata == 0x3) {
		PWRAPLOG("[PMIC1]_pwrap_int_update_test test pass.\n");
		PWRAPLOG("[PWRAP]EINT_AP 0xF000B000=0x%x\n", *((volatile *)0xF000B000));
	}else {
		PWRAPLOG("[PMIC1]_pwrap_int_update_test test fail EINT_STA=0x%x\n", rdata);
		PWRAPLOG("[PWRAP]EINT_AP 0xF000B000=0x%x\n", *((volatile *)0xF000B000));
		res++;
	}
	//WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0x0);
	WRAP_WR32(PMIC_WRAP_EINT_CLR,0xf);
#endif
	mdelay(5000);
	//step2: test another  pmic INT 
#ifdef SLV_6332
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0xa);
	WRAP_WR32(PMIC_WRAP_EINT_STA1_ADR,MT6332_DEW_WRITE_TEST);
	pwrap_wacs2(1, MT6332_DEW_WRITE_TEST, 0x1, &rdata);
	pwrap_wacs2(0, MT6332_DEW_WRITE_TEST, 0, &rdata);
	PWRAPLOG("[PMIC2]_pwrap_int_update_test test  MT6332_DEW_WRITE_TEST=0x%x\n", rdata);
	//WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0xa);//only 6332
	mdelay(100);
	rdata = WRAP_RD32(PMIC_WRAP_EINT_STA);
	if(rdata == 0x4) {
		PWRAPLOG("[PMIC2]_pwrap_int_update_test test pass.\n");
		PWRAPLOG("[PWRAP]EINT_AP 0xF000B000=0x%x\n", *((volatile *)0xF000B000));
	}else {
		PWRAPLOG("[PMIC2]_pwrap_int_update_test test fail EINT_STA=0x%x\n", rdata);
		PWRAPLOG("[PWRAP]EINT_AP 0xF000B000=0x%x\n", *((volatile *)0xF000B000));
		res++;
	}
	//WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0x0);
	WRAP_WR32(PMIC_WRAP_EINT_CLR,0xf);
#endif
	mdelay(5000);
	//step3: test two pmic INT 
#if defined SLV_6332 && defined SLV_6328
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0xf);
	WRAP_WR32(PMIC_WRAP_EINT_STA0_ADR,MT6328_DEW_WRITE_TEST);
	WRAP_WR32(PMIC_WRAP_EINT_STA1_ADR,MT6332_DEW_WRITE_TEST);
	pwrap_wacs2(1, MT6328_DEW_WRITE_TEST, 0x3, &rdata);
	pwrap_wacs2(1, MT6332_DEW_WRITE_TEST, 0x1, &rdata);
	pwrap_wacs2(0, MT6328_DEW_WRITE_TEST, 0, &rdata);
	pwrap_wacs2(0, MT6332_DEW_WRITE_TEST, 0, &rdata);
	//WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0xC);//6328 & 6332
	mdelay(100);
	rdata = WRAP_RD32(PMIC_WRAP_EINT_STA);
	if(rdata == 0x7) {
		PWRAPLOG("[PMIC1_PMIC2]_pwrap_int_update_test test pass.\n");
	}else {
		PWRAPLOG("[PMIC1_PMIC2]_pwrap_int_update_test test fail EINT_STA=0x%x\n", rdata);
		res++;
	}
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0x0);
	WRAP_WR32(PMIC_WRAP_EINT_CLR,0xf);
#endif
	mdelay(5000);
#ifdef SLV_6328
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0x5);
	WRAP_WR32(PMIC_WRAP_EINT_STA0_ADR,MT6328_INT_STA);
#endif
#ifdef SLV_6332
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,WRAP_RD32(PMIC_WRAP_STAUPD_GRPEN)|0xa);
	WRAP_WR32(PMIC_WRAP_EINT_STA1_ADR,MT6332_INT_STA);
#endif
	return res;
}
static S32 tc_pwrap_int_update_test(  )
{
	int res=0;
	res=_pwrap_int_update_test();
	if(res==0) {
		PWRAPLOG("DVT_WRAP_int_updateT pass.\n");
	} else {
		PWRAPLOG("DVT_WRAP_int_update fail.res=%d\n",res);
	}
	//res=pwrap_init();
	if(res==0)
	{
		PWRAPLOG("wrap_init test pass.\n");
	}else {
		PWRAPLOG("wrap_init test fail.\n");
	}
	return res;
}

static S32 tc_wrap_init_test(  )
{
	UINT32 ret=0;
	UINT32 res=0;
	U32 regValue=0;
	mt_wrp_dvt->complete = pwrap_complete;
	mt_wrp_dvt->context = &pwrap_done;
	//CTP_GetRandom(&u4Random, 200);
	ret=pwrap_init();
	if(ret==0)
	{
		PWRAPLOG("wrap_init test pass.\n");
		ret=_pwrap_status_update_test(1);
		if(ret==0)
		{
			PWRAPLOG("_pwrap_status_update_test pass.\n");
		}
		else
		{
			PWRAPLOG("error:_pwrap_status_update_test fail.\n");
			res+=1;
		}
	}
	else
	{
		PWRAPLOG("error:wrap_init test fail.return_value=%d.\n",ret);
		res+=1;
	}
#ifdef DEBUG_LDVT
	regValue=WRAP_RD32(PMIC_WRAP_INT_FLG);
	PWRAPLOG("PMIC_WRAP_INT_FLG =%x\n",regValue);
	regValue=WRAP_RD32(PMIC_WRAP_WDT_FLG);
	PWRAPLOG("PMIC_WRAP_WDT_FLG =%x\n",regValue);
#endif

#ifdef DEBUG_CTP
	regValue=WRAP_RD32(PMIC_WRAP_INT_FLG);
	PWRAPLOG("PMIC_WRAP_INT_FLG =%x\n",regValue);
	regValue=WRAP_RD32(PMIC_WRAP_WDT_FLG);
	PWRAPLOG("PMIC_WRAP_WDT_FLG =%x\n",regValue);
#endif
	if (res != 0)
	{
		PWRAPLOG("error:DVT_WRAP_init_test fail.\n");

	}
	else
	{
		PWRAPLOG("DVT_WRAP_init_test pass.\n");

	}
	return res;
}
static S32 tc_wrap_access_test(  )
{
	int res=0;
	_pwrap_switch_dio(0);
	res=_pwrap_wrap_access_test();
	if(res==0) {
		PWRAPLOG("DVT_WRAP_accesS_TEST pass.\n");
	} else {
		PWRAPLOG("DVT_WRAP_access_TEST fail.res=%d\n",res);
		pwrap_dump_all_register();
	}
	return res;
}


static S32 tc_status_update_test(  )
{
	int res=0;

	res=0;
	res=_pwrap_status_update_test(3);
	if(res==0) {
		PWRAPLOG("DVT_WRAP_status_update_test pass.\n");
	} else {
		PWRAPLOG("DVT_WRAP_status_update_test fail.res=%d\n",res);
	}
	return res;
}

static S32 tc_dual_io_test(  )
{
	int res=0;
	U32 rdata=0;

	res=0;
	//###############################//###############################
	PWRAPLOG("enable dual io mode.\n");
	//enable dual io mode
	_pwrap_switch_dio(1);
	res=_pwrap_wrap_access_test();
	if(res==0)
		PWRAPLOG("_pwrap_wrap_access_test pass.\n");
	else
		PWRAPLOG("_pwrap_wrap_access_test fail.res=%d\n",res);

	//###############################//###############################
	//disable dual io mode
	_pwrap_switch_dio(0);
	PWRAPLOG("disable dual io mode.\n");
	res=_pwrap_wrap_access_test();
	if(res==0)
		PWRAPLOG("_pwrap_wrap_access_test pass.\n");
	else
		PWRAPLOG("_pwrap_wrap_access_test fail.res=%d\n",res);

	if(res==0) {
		PWRAPLOG("DVT_WRAP_dual_io_test pass.\n");
	} else {
		PWRAPLOG("DVT_WRAP_dual_io_test fail.res=%d\n",res);
	}
	return res;
}

U32 RegWriteValue[4] = {0, 0xFFFFFFFF, 0x55555555, 0xAAAAAAAA};



#if 0
static int pwrap_of_iomap(void)
{
	/*
	 * Map the address of the following register base:
	 * INFRACFG_AO, TOPCKGEN, SCP_CLK_CTRL, SCP_PMICWP2P
	 */

	struct device_node *infracfg_ao_node;
	struct device_node *topckgen_node;

	infracfg_ao_node =
		of_find_compatible_node(NULL, NULL, "mediatek,INFRACFG_AO");
	if (!infracfg_ao_node) {
		pr_warn("get INFRACFG_AO failed\n");
		return -ENODEV;
	}

	wrap_infracfg_ao_base = of_iomap(infracfg_ao_node, 0);
	if (!wrap_infracfg_ao_base) {
		pr_warn("INFRACFG_AO iomap failed\n");
		return -ENOMEM;
	}

	topckgen_node =
		of_find_compatible_node(NULL, NULL, "mediatek,CKSYS");
	if (!topckgen_node) {
		pr_warn("get TOPCKGEN failed\n");
		return -ENODEV;
	}

	wrap_topckgen_base = of_iomap(topckgen_node, 0);
	if (!wrap_topckgen_base) {
		pr_warn("TOPCKGEN iomap failed\n");
		return -ENOMEM;
	}
	return 0;
}
static  void pwrap_of_iounmap(void)
{
	iounmap(wrap_infracfg_ao_base);
	iounmap(wrap_topckgen_base);
}

#endif


static S32 tc_reg_rw_test(void)
{
	int res=0;
	U32 i,j;
	U32 pmic_wrap_reg_size=0;
	U32 PERI_PWRAP_BRIDGE_reg_size=0;
	//U32 DEW_reg_tbl_size=0;
	U32 regValue=0;
	U32 reg_data=0;

	U32 test_result=0;

#ifdef CONFIG_OF
	S32 sub_return=0;

	sub_return = pwrap_of_iomap();
	if (sub_return)
		return sub_return;
#endif

    pwrap_of_iomap();
	PWRAPFUC();
	__pwrap_soft_reset();
	pwrap_dump_all_register();
	pmic_wrap_reg_size=sizeof(pmic_wrap_reg_tbl)/sizeof(pmic_wrap_reg_tbl[0]);
	//DEW_reg_tbl_size=sizeof(DEW_reg_tbl)/sizeof(DEW_reg_tbl[0]);

	PWRAPLOG("pmic_wrap_reg_size=%d\n",pmic_wrap_reg_size);
	//PWRAPLOG("DEW_reg_tbl_size=%d\n",DEW_reg_tbl_size);

	PWRAPLOG("start test pmic_wrap_reg_tbl:default value test\n");
	for(i=0; i<pmic_wrap_reg_size; i++){
		//Only R/W or RO should do default value test
		//if(REG_TYP_WO!=pmic_wrap_reg_tbl[i][3])
		if(pmic_wrap_reg_tbl[i][3] != WO)
		{
			PWRAPLOG("Reg offset %.3x: Default %.8x,i=%d\n", pmic_wrap_reg_tbl[i][0], pmic_wrap_reg_tbl[i][1],i);
			if((*((volatile unsigned int *)(pmic_wrap_reg_tbl[i][0]+PMIC_WRAP_BASE))!=pmic_wrap_reg_tbl[i][1])) {
				PWRAPLOG("Reg offset %.3x Default %.8x,infact %.8x, Test failed!!\r\n", 
						pmic_wrap_reg_tbl[i][0],  
						pmic_wrap_reg_tbl[i][1],  
						(*((volatile unsigned int *)(pmic_wrap_reg_tbl[i][0]+PMIC_WRAP_BASE))));
				test_result++;
			}
		}
	}
	//#if 0
	PWRAPLOG("start test pmic_wrap_reg_tbl:R/W test\n");
	test_result=0;
	//test value:RegWriteValue[LCD_RegWriteValueSize] = {0, 0xFFFFFFFF, 0x55555555, 0xAAAAAAAA};
	for(i=0; i<pmic_wrap_reg_size; i++) 
	{
		if(pmic_wrap_reg_tbl[i][3]==RW) 
		{
			for(j=0; j<4; j++) 
			{
				*((volatile unsigned int *)(pmic_wrap_reg_tbl[i][0]+PMIC_WRAP_BASE)) = (RegWriteValue[j]&pmic_wrap_reg_tbl[i][2]);
				if(((*((volatile unsigned int *)(pmic_wrap_reg_tbl[i][0]+PMIC_WRAP_BASE))&pmic_wrap_reg_tbl[i][2])!=(RegWriteValue[j]&pmic_wrap_reg_tbl[i][2]))) {
					PWRAPLOG("Reg offset %x R/W test fail. write %x, read %x \r\n",
							(pmic_wrap_reg_tbl[i][0]),
							(RegWriteValue[j]&pmic_wrap_reg_tbl[i][2]),
							(*((volatile unsigned int *)(pmic_wrap_reg_tbl[i][0]+PMIC_WRAP_BASE)))&pmic_wrap_reg_tbl[i][2]);
					test_result++;
				}
			}
		}
	}
	//#endif

	if(test_result==0) {
		PWRAPLOG("DVT_WRAP_rw_test pass.\n");
	} else {
		PWRAPLOG("DVT_WRAP_rw_test fail.res=%d\n",test_result);
	}

	
#ifdef CONFIG_OF
		pwrap_of_iounmap();
#endif
    
	return test_result;
}

int wrap_mux_test =0;
static S32 tc_mux_switch_test(  )
{
	UINT32 res=0;

	res=0;
	PWRAPLOG("fwq tc_mux_switch_test +++++++++++++++++++++++\n" );
	wrap_mux_test=1;
	res=_pwrap_man_access_test();
	if(res==0)
	{
		PWRAPLOG("DVT_WRAP_mux_switch_test pass.\n");

	}
	else
	{
		PWRAPLOG("DVT_WRAP_mux_switch_test fail.res=%d\n",res);

	}
	PWRAPLOG("fwq tc_mux_switch_test ------------\n" );
	wrap_mux_test=0;
	return res;
}
#if 0
static S32 tc_reset_pattern_test(  )
{
	UINT32 res=0;

	res=_pwrap_reset_spislv;
	res=_pwrap_wrap_access_test();
	res=_pwrap_reset_spislv;
	res=_pwrap_WRITE_TEST_test();
	if(res==0) {
		PWRAPLOG("tc_reset_pattern_test WRITE_TEST reset default value pass.\n");
	} else {
		PWRAPLOG("tc_reset_pattern_test WRITE_TEST reset default value fail.res=%d\n",res);

	}
	res=_pwrap_wrap_access_test();
	if(res==0) {
		PWRAPLOG("tc_reset_pattern_test pass.\n");
	} else {
		PWRAPLOG("tc_reset_pattern_test fail.res=%d\n",res);

	}
	return res;
}
#endif

static S32 tc_soft_reset_test(  )
{
	UINT32 res=0;
	UINT32 regValue=0;

	res=0;
	//---do wrap init and wrap access test-----------------------------------
	res=pwrap_init();
	res=_pwrap_wrap_access_test();
	//---reset wrap-------------------------------------------------------------
	PWRAPLOG("the wrap access test should be fail after reset,before init\n");
	res=_pwrap_wrap_access_test();
	if(res==0)
		PWRAPLOG("_pwrap_wrap_access_test pass.\n");
	else
		PWRAPLOG("_pwrap_wrap_access_test fail.res=%d\n",res);
	PWRAPLOG("the wrap access test should be pass after reset and wrap init\n");

	//---do wrap init and wrap access test-----------------------------------
	res=pwrap_init();
	res=_pwrap_wrap_access_test();
	if(res==0)
		PWRAPLOG("_pwrap_wrap_access_test pass.\n");
	else
		PWRAPLOG("_pwrap_wrap_access_test fail.res=%d\n",res);
	if(res==0)
	{
		PWRAPLOG("DVT_WRAP_soft_reset_test pass.\n");

	}
	else
	{
		PWRAPLOG("DVT_WRAP_soft_reset_test fail.res=%d\n",res);

	}
	return res;
}

static S32 tc_high_pri_test(  )
{
	U32 res=0;
	U32 rdata=0;
	U64 pre_time=0;
	U64 post_timer=0;
	U64 enable_staupd_time=0;
	U64 disable_staupd_time=0;
	U64 GPT2_COUNT_value=0;

	res=0;
	//----enable status updata and do wacs0-------------------------------------
	PWRAPLOG("enable status updata and do wacs0,record the cycle\n");
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0x1);  //0x1:20us,for concurrence test,MP:0x5;  //100us
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0xff);
	//###############################
	// Read/Write test using WACS0
	//###############################
	//perfmon_start();//record time start,ldvt_follow_up
	//GPT2_COUNT_value=WRAP_RD32(APMCU_GPTIMER_BASE + 0x0028);
	pre_time=sched_clock();
	PWRAPLOG("GPT2_COUNT_value=%lld pre_time=%lld\n", GPT2_COUNT_value,pre_time);
#ifdef SLV_6328
	pwrap_wacs0(0, MT6328_DEW_READ_TEST, 0, &rdata);
	if( rdata != MT6328_DEFAULT_VALUE_READ_TEST )
	{
		PWRAPERR("[MT6328]read test error(using WACS0),error code=%x, rdata=%x\n", 1, rdata);
		res+=1;
	}
#endif

	//perfmon_end();
	post_timer=sched_clock();
	enable_staupd_time=post_timer-pre_time;
	PWRAPLOG("pre_time=%lld post_timer=%lld\n", pre_time,post_timer);
	PWRAPLOG("pwrap_wacs0 enable_staupd_time=%lld\n", enable_staupd_time);

	//----disable status updata and do wacs0-------------------------------------
	PWRAPLOG("disable status updata and do wacs0,record the cycle\n");
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0xF);  //0x1:20us,for concurrence test,MP:0x5;  //100us
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN,0x00);
	//###############################
	// Read/Write test using WACS0
	//###############################
	//perfmon_start();
	pre_time=sched_clock();
#ifdef SLV_6328
	pwrap_wacs0(0, MT6328_DEW_READ_TEST, 0, &rdata);
	if( rdata != MT6328_DEFAULT_VALUE_READ_TEST )
	{
		PWRAPERR("[MT6328]read test error(using WACS0),error code=%x, rdata=%x\n", 1, rdata);
		res+=1;
	}
#endif

	//perfmon_end();
	post_timer=sched_clock();
	disable_staupd_time=post_timer-pre_time;
	PWRAPLOG("pre_time=%lld post_timer=%lld\n", pre_time,post_timer);
	PWRAPLOG("pwrap_wacs0 disable_staupd_time=%lld\n", disable_staupd_time);
	if(disable_staupd_time<=enable_staupd_time)
	{
		PWRAPLOG("DVT_WRAP_high_pri_test pass.\n");

	}
	else
	{
		PWRAPLOG("DVT_WRAP_high_pri_test fail.res=%d\n",res);

	}
	return res;
}


static S32 tc_spi_encryption_test(  )
{

	int res=0;
	U32 reg_value_backup=0;
	int return_value;
	U32 rdata;
	// disable status update,to check the waveform on oscilloscope
	//WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0x0);  //0x0:disable
	//disable wdt int bit
	reg_value_backup=WRAP_RD32(PMIC_WRAP_INT_EN);
	WRAP_CLR_BIT(1<<0,PMIC_WRAP_INT_EN);
	//disable dio mode,single io wave
	_pwrap_switch_dio(0);
	//###############################
	// disable Encryption
	//###############################
	res = _pwrap_disable_cipher();//FPGA:set breakpoint here
	if( res != 0 ) {
		PWRAPERR("disable Encryption error,error code=%x, rdata=%x", 0x21, res);
		return -EINVAL;
	}
	//PMIC1 read
#ifdef SLV_6328
	//res=_pwrap_wrap_access_test();
	PWRAPLOG("start read PMIC1 using WACS2, please record the waveform\n");
	return_value=pwrap_read(MT6328_DEW_READ_TEST,&rdata);
	if( rdata != MT6328_DEFAULT_VALUE_READ_TEST ) {
		PWRAPERR("read test error(using WACS2),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
#endif
	//PMIC2
#ifdef SLV_6332
	PWRAPLOG("start read PMIC2 using WACS2, please record the waveform\n");
	return_value=pwrap_read(MT6332_DEW_READ_TEST,&rdata);
	if( rdata != MT6332_DEFAULT_VALUE_READ_TEST ) {
		PWRAPERR("PMIC2 read test error(using WACS2),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
#endif
	msleep(5000);
	//###############################
	// enable Encryption
	//###############################
	res = _pwrap_enable_cipher();
	if( res != 0 ) {
		PWRAPERR("Enable Encryption error,error code=%x, res=%x", 0x21, res);
		return -EINVAL;
	}
	//PMIC1 read
#ifdef SLV_6328
	//res=_pwrap_wrap_access_test();
	PWRAPLOG("start read PMIC1 using WACS2, please record the waveform\n");
	return_value=pwrap_read(MT6328_DEW_READ_TEST,&rdata);
	if( rdata != MT6328_DEFAULT_VALUE_READ_TEST ) {
		PWRAPERR("read test error(using WACS2),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
#endif
	//PMIC2
#ifdef SLV_6332
	PWRAPLOG("start read PMIC2 using WACS2, please record the waveform\n");
	return_value=pwrap_read(MT6332_DEW_READ_TEST,&rdata);
	if( rdata != MT6332_DEFAULT_VALUE_READ_TEST ) {
		PWRAPERR("PMIC2 read test error(using WACS2),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
#endif
	if(res==0) {
		PWRAPLOG("DVT_WRAP_spi_encryption_test pass.\n");
	} else {
		PWRAPLOG("DVT_WRAP_spi_encryption_test fail.res=%d\n",res);
	}
	//WRAP_WR32(PMIC_WRAP_STAUPD_PRD, reg_value_backup);  //0x0:disable
	return res;
}
//-------------------irq init start-------------------------------------
//CHOOSE_LISR=0:normal test;CHOOSE_LISR=1:watch dog test;
//CHOOSE_LISR=2:interrupt test
#define CHOOSE_LISR     1
#define NORMAL_TEST     1
#define WDT_TEST        2
//#define PERI_WDT_TEST   3
#define INT_TEST        4
//#define PERI_INT_TEST   5

U32 wrapper_lisr_count_cpu0=0;
U32 wrapper_lisr_count_cpu1=0;

static U32 int_test_bit=0;
static U32 wait_int_flag=0;
static U32 int_test_fail_count=0;

//global value for watch dog
static U32 wdt_test_bit=0;
static U32 wait_for_wdt_flag=0;
static U32 wdt_test_fail_count=0;

//global value for peri watch dog

#if 0
static S32 pwrap_lisr_normal_test(void)
{
	U32 reg_int_flg=0;
	U32 reg_wdt_flg=0;
	PWRAPFUC();
	pwrap_dump_ap_register();
	//#ifndef ldvt_follow_up
	if (raw_smp_processor_id() == 0)
	{
		wrapper_lisr_count_cpu0++;
	} else if (raw_smp_processor_id() == 1)
	{
		wrapper_lisr_count_cpu1++;
	}
	//#endif
	reg_int_flg=WRAP_RD32(PMIC_WRAP_INT_FLG);
	PWRAPLOG("PMIC_WRAP_INT_FLG=0x%x.\n",reg_int_flg);
	reg_wdt_flg=WRAP_RD32(PMIC_WRAP_WDT_FLG);
	PWRAPLOG("PMIC_WRAP_WDT_FLG=0x%x.\n",reg_wdt_flg);
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN, 0);//clear watch dog
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0xffffff);
	WRAP_WR32(PMIC_WRAP_INT_CLR, reg_int_flg);

}
#endif
#if 1
static S32 pwrap_lisr_for_wdt_test(void)
{
	U32 reg_int_flg=0;
	U32 reg_wdt_flg=0;
	PWRAPFUC();
	reg_int_flg=WRAP_RD32(PMIC_WRAP_INT_FLG);
	PWRAPLOG("PMIC_WRAP_INT_FLG=0x%x.\n",reg_int_flg);
	reg_wdt_flg=WRAP_RD32(PMIC_WRAP_WDT_FLG);
	PWRAPLOG("PMIC_WRAP_WDT_FLG=0x%x.\n",reg_wdt_flg);

	if((reg_int_flg & 0x1) !=0)
	{
		//dispatch_WDT();
		if((reg_wdt_flg & (1<<wdt_test_bit) )!= 0)
		{
			PWRAPLOG("watch dog test:recieve the right wdt.\n");
			wait_for_wdt_flag=1;
			//clear watch dog and interrupt
			WRAP_WR32(PMIC_WRAP_WDT_SRC_EN, 0);
			//WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0xffffff);
		}
		else
		{
			PWRAPLOG("fail watch dog test:recieve the wrong wdt.\n");
			wdt_test_fail_count++;
			//clear the unexpected watch dog and interrupt
			WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0 );
			WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,1<<wdt_test_bit);
		}
	}

	WRAP_WR32(PMIC_WRAP_INT_CLR, reg_int_flg);
	reg_int_flg=WRAP_RD32(PMIC_WRAP_INT_FLG);
	PWRAPLOG("PMIC_WRAP_INT_FLG=0x%x.\n",reg_int_flg);
	reg_wdt_flg=WRAP_RD32(PMIC_WRAP_WDT_FLG);
	PWRAPLOG("PMIC_WRAP_WDT_FLG=0x%x.\n",reg_wdt_flg);
}

static S32 pwrap_lisr_for_int_test(void)
{
	U32 reg_int_flg=0;
	U32 reg_wdt_flg=0;
	PWRAPFUC();
	reg_int_flg=WRAP_RD32(PMIC_WRAP_INT_FLG);
	PWRAPLOG("PMIC_WRAP_INT_FLG=0x%x.\n",reg_int_flg);
	reg_wdt_flg=WRAP_RD32(PMIC_WRAP_WDT_FLG);
	PWRAPLOG("PMIC_WRAP_WDT_FLG=0x%x.\n",reg_wdt_flg);

	//-------------------------interrupt test---------------
	PWRAPLOG("int_test_bit=0x%x.\n",int_test_bit);
	if((reg_int_flg & (1<<int_test_bit)) != 0)
	{
		PWRAPLOG(" int test:recieve the right pwrap interrupt.\n");
		wait_int_flag=1;
	}
	else
	{
		PWRAPLOG(" int test fail:recieve the wrong pwrap interrupt.\n");
		int_test_fail_count++;
	}
	WRAP_WR32(PMIC_WRAP_INT_CLR, reg_int_flg);
	reg_int_flg=WRAP_RD32(PMIC_WRAP_INT_FLG);
	PWRAPLOG("PMIC_WRAP_INT_FLG=0x%x.\n",reg_int_flg);
	reg_wdt_flg=WRAP_RD32(PMIC_WRAP_WDT_FLG);
	PWRAPLOG("PMIC_WRAP_WDT_FLG=0x%x.\n",reg_wdt_flg);

	//for int test bit[1]
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN, 0);

}
#endif
static irqreturn_t mt_pwrap_dvt_irq(int irqno, void *dev_id)
{
	unsigned long flags=0;
	PWRAPFUC();
	//spin_lock_irqsave(&mt_wrp->lock,flags);
	//*-----------------------------------------------------------------------
	switch(mt_wrp_dvt->irq_mode)
	{
		case NORMAL_TEST:
			//pwrap_lisr_normal_test();
			break;
		case WDT_TEST:
			PWRAPLOG("fwq wdt irq\n");
			pwrap_lisr_for_wdt_test();
			mt_wrp_dvt->complete(mt_wrp_dvt->context);
			break;
		case INT_TEST:
			PWRAPLOG("fwq int irq\n");
			pwrap_lisr_for_int_test();
			mt_wrp_dvt->complete(mt_wrp_dvt->context);
			break;
	}

	return IRQ_HANDLED;
}


//-------------------irq init end-------------------------------------------

//-------------------watch dog test start-------------------------------------
U32 watch_dog_test_reg=MT6328_DEW_WRITE_TEST;

//static U32 wrap_WDT_flg=0;


//#define ENABLE_INT_ON_CTP

static S32 _wdt_test_disable_other_int(void)
{
	//disable watch dog
	WRAP_WR32(PMIC_WRAP_INT_EN,0x1);
	return 0;
}

//[1]: HARB_WACS0_ALE: HARB to WACS0 ALE timeout monitor
//disable the corresponding bit in HIPRIO_ARB_EN,and send a WACS0 write command
static S32 _wdt_test_bit1( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wait_for_wdt_flag=0;
	wdt_test_bit=1;
	//disable corresponding bit in PMIC_WRAP_HIPRIO_ARB_EN
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,0x3f);
	WRAP_CLR_BIT(1<<wdt_test_bit,PMIC_WRAP_HIPRIO_ARB_EN);
	pwrap_wacs0(1, watch_dog_test_reg, 0x1234, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit1 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit1 fail.\n");
	}
	PWRAPLOG("_wdt_test_bit1 pass.\n");
	return 0;

}

//[2]: HARB_WACS0_ALE: HARB to WACS1 ALE timeout monitor
//disable the corresponding bit in HIPRIO_ARB_EN,and send a WACS1 write command
static S32 _wdt_test_bit2( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=2;
	wait_for_wdt_flag=0;
	//disable corresponding bit in PMIC_WRAP_HIPRIO_ARB_EN
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,0x3f);
	WRAP_CLR_BIT(1<<2,PMIC_WRAP_HIPRIO_ARB_EN);
	pwrap_wacs0(1, watch_dog_test_reg, 0x1234, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit2 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit2 fail.\n");
	}
	return 0;
}

//[4]: HARB_WACS2_ALE: HARB to WACS2 ALE timeout monitor
//disable the corresponding bit in HIPRIO_ARB_EN,and send a WACS3 write command
static S32 _wdt_test_bit4( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=4;
	wait_for_wdt_flag=0;
	//disable corresponding bit in PMIC_WRAP_HIPRIO_ARB_EN
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,0x1ff);
	WRAP_CLR_BIT(1<<5,PMIC_WRAP_HIPRIO_ARB_EN);
	//pwrap_write(watch_dog_test_reg, 0x1234);
	pwrap_wacs2(1, watch_dog_test_reg, 0x1234, &rdata);
	PWRAPLOG("_wdt_test_bit4 111111111111111111111.\n");
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	PWRAPLOG("_wdt_test_bit4 222222222222222222222.\n");
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit4 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit4 fail.\n");
	}
	return 0;
}

//[9]: HARB_WACS2_ALE: HARB to WACS3 ALE timeout monitor
//disable the corresponding bit in HIPRIO_ARB_EN,and send a WACS3 write command
static S32 _wdt_test_bit9( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=9;
	wait_for_wdt_flag=0;
	//disable corresponding bit in PMIC_WRAP_HIPRIO_ARB_EN
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,0x1ff);
	WRAP_CLR_BIT(1<<9,PMIC_WRAP_HIPRIO_ARB_EN);
	//pwrap_write(watch_dog_test_reg, 0x1234);
	pwrap_wacs3(1, watch_dog_test_reg, 0x1234, &rdata);
	
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit9 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit9 fail.\n");
	}
	return 0;
}


//[5]: HARB_ERC_ALE: HARB to ERC ALE timeout monitor
//disable the corresponding bit in HIPRIO_ARB_EN,do event test
#if 0
static S32 _wdt_test_bit5( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=6;
	wait_for_wdt_flag=0;
	//disable corresponding bit in PMIC_WRAP_HIPRIO_ARB_EN
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,0x1ff);
	WRAP_CLR_BIT(1<<wdt_test_bit,PMIC_WRAP_HIPRIO_ARB_EN);
	//similar to event  test case
	//WRAP_WR32(PMIC_WRAP_EVENT_STACLR , 0xffff);

	//res=pwrap_wacs0(1, DEW_EVENT_TEST, 0x1, &rdata);

	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit5 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit5 fail.\n");
	}
	return 0;
}
#endif

//[8]: HARB_STAUPD_ALE: HARB to STAUPD ALE timeout monitor
//disable the corresponding bit in HIPRIO_ARB_EN,and send a WACS1 write command
static S32 _wdt_test_bit8( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=8;
	wait_for_wdt_flag=0;
	//disable corresponding bit in PMIC_WRAP_HIPRIO_ARB_EN
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,0x1ff);
	//WRAP_CLR_BIT(1<<5,PMIC_WRAP_HIPRIO_ARB_EN);
	WRAP_CLR_BIT(1<<6,PMIC_WRAP_HIPRIO_ARB_EN);
	//similar to status updata test case
	//pwrap_wacs0(1, DEW_WRITE_TEST, 0x55AA, &rdata);
	//WRAP_WR32(PMIC_WRAP_SIG_ADR,DEW_WRITE_TEST);
	//WRAP_WR32(PMIC_WRAP_SIG_VALUE,0xAA55);
	//WRAP_WR32(PMIC_WRAP_STAUPD_MAN_TRIG,0x1);
	pwrap_wacs2(1, watch_dog_test_reg, 0x1234, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit8 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit8 fail.\n");
	}
	WRAP_WR32(PMIC_WRAP_SIG_VALUE,0x55AA);//tha same as write test
	return 0;
}


static S32 _wdt_test_bit6( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=6;
	wait_for_wdt_flag=0;
	//disable corresponding bit in PMIC_WRAP_HIPRIO_ARB_EN
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,0x1ff);
	WRAP_CLR_BIT(1<<8,PMIC_WRAP_HIPRIO_ARB_EN);
	//similar to status updata test case
	//pwrap_wacs0(1, DEW_WRITE_TEST, 0x55AA, &rdata);
	//WRAP_WR32(PMIC_WRAP_SIG_ADR,DEW_WRITE_TEST);
	//WRAP_WR32(PMIC_WRAP_SIG_VALUE,0xAA55);
	//WRAP_WR32(PMIC_WRAP_STAUPD_MAN_TRIG,0x1);
	pwrap_wacs1(1, watch_dog_test_reg, 0x1234, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit6 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit6 fail.\n");
	}
	WRAP_WR32(PMIC_WRAP_SIG_VALUE,0x55AA);//tha same as write test
	return 0;
}


//[7]: PWRAP_PERI_ALE: HARB to PWRAP_PERI_BRIDGE ALE timeout monitor
//disable the corresponding bit in HIPRIO_ARB_EN,and send a WACS3 write command
#if 0
static S32 _wdt_test_bit7( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=7;
	wait_for_wdt_flag=0;
	//disable corresponding bit in PMIC_WRAP_HIPRIO_ARB_EN
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,0x1ff);
	WRAP_CLR_BIT(1<<wdt_test_bit,PMIC_WRAP_HIPRIO_ARB_EN);

	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit7 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit7 fail.\n");
	}
	return 0;
}

//[8]: HARB_EINTBUF_ALE: HARB to EINTBUF ALE timeout monitor
//disable the corresponding bit in HIPRIO_ARB_EN,and send a eint interrupt
static S32 _wdt_test_bit8( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=8;
	wait_for_wdt_flag=0;
	//disable corresponding bit in PMIC_WRAP_HIPRIO_ARB_EN
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN,0x1ff);
	WRAP_CLR_BIT(1<<wdt_test_bit,PMIC_WRAP_HIPRIO_ARB_EN);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit8 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit8 fail.\n");
	}
	return 0;
}

//[9]: WRAP_HARB_ALE: WRAP to HARB ALE timeout monitor
//disable RRARB_EN[0],and do eint test
static S32 _wdt_test_bit9( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0,i=0;
	PWRAPFUC();
	wdt_test_bit=9;
	wait_for_wdt_flag=0;
	//Delay(1000);
	for (i=0;i<(300*20);i++);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	//WRAP_WR32(PMIC_WRAP_WRAP_EN ,1);//recover
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit9 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit9 fail.\n");
	}
	return 0;
}

//[10]: PWRAP_AG_ALE#1: PWRAP to AG#1 ALE timeout monitor
//disable RRARB_EN[1],and do keypad test
static S32 _wdt_test_bit10( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=10;
	wait_for_wdt_flag=0;
	//disable wrap_en
	//WRAP_CLR_BIT(1<<1 ,PMIC_WRAP_RRARB_EN);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit10 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit10 fail.\n");
	}
	return 0;
}

//[11]: PWRAP_AG_ALE#2: PWRAP to AG#2 ALE timeout monitor
//disable RRARB_EN[0],and do eint test
static S32 _wdt_test_bit11( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=11;
	wait_for_wdt_flag=0;
	//disable wrap_en
	//WRAP_CLR_BIT(1<<1 ,PMIC_WRAP_RRARB_EN);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit11 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit11 fail.\n");
	}
	return 0;
}

#endif
//[12]: wrap_HARB_ALE: WRAP to harb ALE timeout monitor
//  ,disable wrap_en and set a WACS0 read command
static S32 _wdt_test_bit12( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=12;
	wait_for_wdt_flag=0;

	//_pwrap_switch_mux(1);//manual mode
	//WRAP_WR32(PMIC_WRAP_MUX_SEL , 0);
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 0);//disble wrap
	//WRAP_WR32(PMIC_WRAP_MAN_EN , 1);//enable manual
	pwrap_wacs0(0, watch_dog_test_reg, 0, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/

	//_pwrap_switch_mux(0);// recover

	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit12 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit12 fail.\n");
	}
	return 0;

}

//[13]: MUX_WRAP_ALE: MUX to WRAP ALE timeout monitor
// set MUX to manual mode ,enable wrap_en and set a WACS0 read command
static S32 _wdt_test_bit13( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=13;
	wait_for_wdt_flag=0;

	_pwrap_switch_mux(1);//manual mode
	//WRAP_WR32(PMIC_WRAP_MUX_SEL , 0);
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	//WRAP_WR32(PMIC_WRAP_MAN_EN , 1);//enable manual
	pwrap_wacs0(0, watch_dog_test_reg, 0, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/

	_pwrap_switch_mux(0);// recover

	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit13 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit13 fail.\n");
	}
	return 0;

}

//[14]: MUX_MAN_ALE: MUX to MAN ALE timeout monitor
//MUX to MAN ALE:set MUX to wrap mode and set manual command
static S32 _wdt_test_bit14( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=14;
	wait_for_wdt_flag=0;
	_pwrap_switch_mux(0);//wrap mode
	//WRAP_WR32(PMIC_WRAP_WRAP_EN , 0);//enable wrap
	WRAP_WR32(PMIC_WRAP_MAN_EN , 1);//enable manual

	_pwrap_manual_mode(0,  OP_IND, 0, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit14 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit14 fail.\n");
	}
	_pwrap_switch_mux(1);//
	return 0;
}

//[16]: HARB_WACS0_DLE: HARB to WACS0 DLE timeout monitor
//HARB to WACS0 DLE:disable MUX,and send a read commnad with WACS0
static S32 _wdt_test_bit16( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=16;
	wait_for_wdt_flag=0;

	//disable other wdt bit
	//WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0);
	//WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,1<<wdt_test_bit);
	reg_rdata=WRAP_RD32(PMIC_WRAP_WDT_SRC_EN);
	PWRAPLOG("PMIC_WRAP_WDT_SRC_EN=%x.\n",reg_rdata);

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , 1<<2);//enable wrap
	reg_rdata=WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);
	PWRAPLOG("PMIC_WRAP_WDT_SRC_EN=%x.\n",reg_rdata);
	//set status update period to the max value,or disable status update
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0xF);

	_pwrap_switch_mux(1);//manual mode
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	//read command
	pwrap_wacs0(0, watch_dog_test_reg, 0, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit16 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit16 fail.\n");
	}
	_pwrap_switch_mux(0);//recover
	return 0;
}


//[17]: HARB_WACS1_DLE: HARB to WACS1 DLE timeout monitor
//HARB to WACS1 DLE:disable MUX,and send a read commnad with WACS1
static S32 _wdt_test_bit17( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=17;
	wait_for_wdt_flag=0;

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , 1<<8);//enable wrap
	reg_rdata=WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);
	PWRAPLOG("PMIC_WRAP_HIPRIO_ARB_EN=%x.\n",reg_rdata);
	//set status update period to the max value,or disable status update
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0x0);

	_pwrap_switch_mux(1);//manual mode
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	//read command
	pwrap_wacs1(0, watch_dog_test_reg, 0, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit17 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit17 fail.\n");
	}
	_pwrap_switch_mux(0);//recover
	return 0;
}
//[18]: HARB_WACS2_DLE: HARB to WACS2 DLE timeout monitor
//HARB to WACS2 DLE:disable MUX,and send a read commnad with WACS2
static S32 _wdt_test_bit18( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=18;
	wait_for_wdt_flag=0;

	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0);
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,1<<wdt_test_bit);
	reg_rdata=WRAP_RD32(PMIC_WRAP_WDT_SRC_EN);
	PWRAPLOG("PMIC_WRAP_HIPRIO_ARB_EN=%x.\n",reg_rdata);

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , 1<<5); //WACS2
	reg_rdata=WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);
	PWRAPLOG("PMIC_WRAP_HIPRIO_ARB_EN=%x.\n",reg_rdata);
	//set status update period to the max value,or disable status update
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0xF);

	reg_rdata=WRAP_RD32(PMIC_WRAP_STAUPD_PRD);
	PWRAPLOG("PMIC_WRAP_STAUPD_PRD=%x.\n",reg_rdata);

	_pwrap_switch_mux(1);//manual mode
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	//clear INT
	WRAP_WR32(PMIC_WRAP_INT_CLR, 0xFFFFFFFF);

	//read command
	pwrap_read(watch_dog_test_reg,&rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit18 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit18 fail.\n");
	}
	_pwrap_switch_mux(0);//recover
	return 0;
}

static S32 _wdt_test_bit21( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=21;
	wait_for_wdt_flag=0;

	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0);
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,1<<wdt_test_bit);
	reg_rdata=WRAP_RD32(PMIC_WRAP_WDT_SRC_EN);
	PWRAPLOG("PMIC_WRAP_HIPRIO_ARB_EN=%x.\n",reg_rdata);

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , 1<<9); //WACS3
	reg_rdata=WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);
	PWRAPLOG("PMIC_WRAP_HIPRIO_ARB_EN=%x.\n",reg_rdata);
	//set status update period to the max value,or disable status update
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0xF);

	reg_rdata=WRAP_RD32(PMIC_WRAP_STAUPD_PRD);
	PWRAPLOG("PMIC_WRAP_STAUPD_PRD=%x.\n",reg_rdata);

	_pwrap_switch_mux(1);//manual mode
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	//clear INT
	WRAP_WR32(PMIC_WRAP_INT_CLR, 0xFFFFFFFF);

	//read command
	pwrap_wacs3(0, watch_dog_test_reg, 0, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit21 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit21 fail.\n");
	}
	_pwrap_switch_mux(0);//recover
	return 0;
}

static S32 _wdt_test_bit27( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=27;
	wait_for_wdt_flag=0;

	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0);
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,1<<wdt_test_bit);
	reg_rdata=WRAP_RD32(PMIC_WRAP_WDT_SRC_EN);
	PWRAPLOG("PMIC_WRAP_HIPRIO_ARB_EN=%x.\n",reg_rdata);

	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , 1<<9); //WACS2
	reg_rdata=WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);
	PWRAPLOG("PMIC_WRAP_HIPRIO_ARB_EN=%x.\n",reg_rdata);
	//set status update period to the max value,or disable status update
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0xF);

	reg_rdata=WRAP_RD32(PMIC_WRAP_STAUPD_PRD);
	PWRAPLOG("PMIC_WRAP_STAUPD_PRD=%x.\n",reg_rdata);

	_pwrap_switch_mux(1);//manual mode
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	//clear INT
	WRAP_WR32(PMIC_WRAP_INT_CLR, 0xFFFFFFFF);

	//read command
	pwrap_read(watch_dog_test_reg,&rdata);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit27 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit27 fail.\n");
	}
	_pwrap_switch_mux(0);//recover
	return 0;
}


//[19]: HARB_ERC_DLE: HARB to ERC DLE timeout monitor
//HARB to staupda DLE:disable event,write de_wrap event test,then swith mux to manual mode ,enable wrap_en enable event
//similar to bit5
#if 0
static S32 _wdt_test_bit19( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=19;
	wait_for_wdt_flag=0;
	//disable event
	//WRAP_WR32(PMIC_WRAP_EVENT_IN_EN , 0);

	//do event test
	//WRAP_WR32(PMIC_WRAP_EVENT_STACLR , 0xffff);
	//res=pwrap_wacs0(1, DEW_EVENT_TEST, 0x1, &rdata);
	//disable mux
	_pwrap_switch_mux(1);//manual mode
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	//enable event
	//WRAP_WR32(PMIC_WRAP_EVENT_IN_EN , 1);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit19 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit19 fail.\n");
	}
	_pwrap_switch_mux(0);//recover
	return 0;
}

//[20]: HARB_STAUPD_DLE: HARB to STAUPD DLE timeout monitor
//  HARB to staupda DLE:disable MUX,then send a read commnad ,and do status update test
//similar to bit6
static S32 _wdt_test_bit20( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=20;
	wait_for_wdt_flag=0;
	_pwrap_switch_mux(1);//manual mode
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	//similar to status updata test case
	pwrap_wacs0(1, MT6328_DEW_WRITE_TEST, 0x55AA, &rdata);
	WRAP_WR32(PMIC_WRAP_SIG_ADR,MT6328_DEW_WRITE_TEST);
	WRAP_WR32(PMIC_WRAP_SIG_VALUE,0xAA55);
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit20 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit20 fail.\n");
	}
	_pwrap_switch_mux(0);//recover
	WRAP_WR32(PMIC_WRAP_SIG_VALUE,0x55AA);//tha same as write test

	return 0;
}
//[21]: HARB_RRARB_DLE: HARB to RRARB DLE timeout monitor HARB to RRARB DLE
//:disable MUX,do keypad test
static S32 _wdt_test_bit21( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	U32 reg_backup;
	PWRAPFUC();
	wdt_test_bit=21;
	wait_for_wdt_flag=0;
#ifdef ENABLE_KEYPAD_ON_LDVT
	//kepad command
	*((volatile kal_uint16 *)(KP_BASE + 0x1c)) = 0x1;
	kpd_init();
	initKpdTest();
#endif
	//WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0 );
	//WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,1<<wdt_test_bit);
	reg_backup=WRAP_RD32(PMIC_WRAP_HIPRIO_ARB_EN);
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , 0x80);//only enable keypad
	_pwrap_switch_mux(1);//manual mode
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	WRAP_WR32(0x10016020 , 0);//write keypad register,to send a keypad read request
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit21 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit21 fail.\n");
	}
	WRAP_WR32(PMIC_WRAP_HIPRIO_ARB_EN , reg_backup);

	_pwrap_switch_mux(0);//recover
	return 0;
}


//[22]: MUX_WRAP_DLE: MUX to WRAP DLE timeout monitor
//MUX to WRAP DLE:disable MUX,then send a read commnad ,and do WACS0
static S32 _wdt_test_bit22( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=22;
	wait_for_wdt_flag=0;
	//WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	pwrap_wacs1(0, watch_dog_test_reg, 0, &rdata);
	_pwrap_switch_mux(1);//manual mode
	WRAP_WR32(PMIC_WRAP_WRAP_EN , 1);//enable wrap
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit22 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit22 fail.\n");
	}
	_pwrap_switch_mux(0);//recover
	return 0;
}
#endif

//[23]: MUX_MAN_DLE: MUX to MAN DLE timeout monitor
//MUX to MAN DLE:disable MUX,then send a read commnad in manual mode
static S32 _wdt_test_bit23( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	U32 return_value=0;
	PWRAPFUC();
	wdt_test_bit=23;
	wait_for_wdt_flag=0;

	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0);
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,1<<wdt_test_bit);
	reg_rdata=WRAP_RD32(PMIC_WRAP_WDT_SRC_EN);
	PWRAPLOG("PMIC_WRAP_WDT_SRC_EN=%x.\n",reg_rdata);



	return_value=_pwrap_switch_mux(1);//manual mode
	PWRAPLOG("_pwrap_switch_mux return_value=%x.\n",return_value);

	WRAP_WR32(PMIC_WRAP_SI_CK_CON,0x6);
	reg_rdata=WRAP_RD32(PMIC_WRAP_SI_CK_CON);
	PWRAPLOG("PMIC_WRAP_SI_CK_CON=%x.\n",reg_rdata);

	return_value=_pwrap_manual_mode(0,  OP_IND, 0, &rdata);
	PWRAPLOG("_pwrap_manual_mode return_value=%x.\n",return_value);

	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit23 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit23 fail.\n");
	}
	_pwrap_switch_mux(0);//recover
	return 0;
}

//[24]: MSTCTL_SYNC_DLE: MSTCTL to SYNC DLE timeout monitor
//MSTCTL to SYNC  DLE:disable sync,then send a read commnad with wacs0
#if 0
static S32 _wdt_test_bit24( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=24;
	wait_for_wdt_flag=0;
	_pwrap_switch_mux(1);//manual mode
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit24 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit24 fail.\n");
	}
	_pwrap_switch_mux(0);//recover
	return 0;
}
#endif
//[25]: STAUPD_TRIG:
//set period=0
static S32 _wdt_test_bit25( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	wdt_test_bit=25;
	wait_for_wdt_flag=0;
	WRAP_WR32(PMIC_WRAP_STAUPD_PRD, 0x0);  //0x1:20us,for concurrence test,MP:0x5;  //100us
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit25 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit25 fail.\n");
	}
	return 0;
}

//[26]: PREADY: APB PREADY timeout monitor
//disable wrap_en and write wacs0 6 times

#if 1
static S32 _wdt_test_bit26( )
{
	U32 rdata=0;
	U32 wdata=0;
	U32 reg_rdata=0;
	U32 i=0;
	U32 wacs_write=0;
	U32 wacs_adr=0;
	U32 wacs_cmd=0;
	U32 return_value=0;
	UINT32 regValue=0;
	PWRAPFUC();
	wdt_test_bit=26;
	wait_for_wdt_flag=0;

	//__pwrap_soft_reset();

	regValue=WRAP_RD32(PMIC_WRAP_STAUPD_GRPEN);
	PWRAPLOG("PMIC_WRAP_STAUPD_GRPEN =%x,it should be equal to 0x1\n",regValue);

	//_wdt_test_disable_other_int();   
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN, 0);
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,1<<wdt_test_bit);

	WRAP_WR32(PMIC_WRAP_WDT_UNIT, 8);

	WRAP_WR32(PMIC_WRAP_WRAP_EN , 0);//disable wrap
	WRAP_WR32(PMIC_WRAP_WACS0_EN , 1);//enable wacs0

	//enable watch dog interrupt
	WRAP_WR32(PMIC_WRAP_INT_EN,(1<<0)|WRAP_RD32(PMIC_WRAP_INT_EN));

	for(i=0;i<10;i++)
	{
		wdata+=0x20;
		PWRAPLOG("before send %d command .\n",i);
		//pwrap_wacs0(1, watch_dog_test_reg, wdata, &rdata);
		WRAP_WR32(PMIC_WRAP_WACS0_CMD,0x80000000);
		PWRAPLOG("send %d command .\n",i);
		msleep(500);
	}
	wait_for_completion(&pwrap_done);
	mdelay(2000);/*completion invalid, need delay*/
	if(wait_for_wdt_flag==1) {
		PWRAPLOG("_wdt_test_bit26 pass.\n");
	}else {
		PWRAPLOG("_wdt_test_bit26 fail.\n");
	}
	return 0;
}
#endif

#define test_fail
static S32 tc_wdt_test( )
{

	UINT32 return_value=0;
	UINT32 result=0;
	#if 1
	UINT32 reg_data=0;
	mt_wrp_dvt->complete = pwrap_complete;
	mt_wrp_dvt->context = &pwrap_done;

	mt_wrp_dvt->irq_mode = WDT_TEST;
	//enable watch dog
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0xffffff);
	//WRAP_WR32(PERI_PWRAP_BRIDGE_WDT_SRC_EN , 0xffff);
#if 1

	return_value=pwrap_init();
	_wdt_test_disable_other_int();

	
	return_value=pwrap_init();
	_wdt_test_disable_other_int();
	mdelay(1000);
	return_value=_wdt_test_bit2();
	mdelay(1000);

	
	return_value=pwrap_init();
	_wdt_test_disable_other_int();
	mdelay(1000);
	return_value=_wdt_test_bit6();
	mdelay(1000);

	return_value=pwrap_init();
	_wdt_test_disable_other_int();
	mdelay(1000);
	return_value=_wdt_test_bit4();
	mdelay(1000);



 
	return_value=pwrap_init();
	_wdt_test_disable_other_int();   
	return_value=_wdt_test_bit9();
    mdelay(1000);

	return_value=pwrap_init();
	_wdt_test_disable_other_int();   
	return_value=_wdt_test_bit8();


	mdelay(1000);
	return_value=pwrap_init();
	return_value=_wdt_test_bit12(); //need to add timeout
	mdelay(1000);
	
	return_value=pwrap_init();
	_wdt_test_disable_other_int();
	mdelay(1000);
	return_value=_wdt_test_bit13();
	mdelay(1000);
	
	return_value=pwrap_init();
	_wdt_test_disable_other_int();
	mdelay(1000);

	reg_data=WRAP_RD32(PMIC_WRAP_INT_FLG);
	PWRAPLOG("wrap_int_flg=%x.\n",reg_data);
	reg_data=WRAP_RD32(PMIC_WRAP_WDT_FLG);
	PWRAPLOG("PMIC_WRAP_WDT_FLG=%x.\n",reg_data);

	return_value=_wdt_test_bit14();
	mdelay(1000);
	
	return_value=pwrap_init();
	_wdt_test_disable_other_int();

	//return_value=_wdt_test_bit15();
	//return_value=pwrap_init();
	//_wdt_test_disable_other_int();
	mdelay(1000);
	return_value=_wdt_test_bit16();//pass
	mdelay(1000);
	
	return_value=pwrap_init();
	_wdt_test_disable_other_int();
	mdelay(1000);
	return_value=_wdt_test_bit17();
	mdelay(1000);
	
	return_value=pwrap_init();
	_wdt_test_disable_other_int();

	mdelay(1000);
	return_value=_wdt_test_bit18();
	mdelay(1000);

	return_value=pwrap_init();
	_wdt_test_disable_other_int();
	return_value=_wdt_test_bit21();
	
	return_value=pwrap_init();
	_wdt_test_disable_other_int();
	return_value=_wdt_test_bit23(); //pass
	mdelay(1000);
	
	return_value=pwrap_init();
	_wdt_test_disable_other_int();
	mdelay(1000);
	//return_value=_wdt_test_bit24();
	//return_value=pwrap_init();
	//_wdt_test_disable_other_int();

	return_value=_wdt_test_bit25();
	mdelay(1000);
	
	return_value=pwrap_init();
	_wdt_test_disable_other_int();
	mdelay(1000);
	return_value=_wdt_test_bit26();
	mdelay(1000);
	
	return_value=pwrap_init();
#endif
	PWRAPLOG("wdt_test_fail_count=%d.\n",wdt_test_fail_count);
	if(result==0)
	{
		PWRAPLOG("DVT_WRAP_wdt_test pass.\n");
	}
	else
	{
		PWRAPLOG("DVT_WRAP_wdt_test fail.res=%d\n",result);
	}
	mt_wrp_dvt->irq_mode = NORMAL_TEST;
	#endif
	return result;
}
//-------------------watch dog test end-------------------------------------

//start----------------interrupt test ------------------------------------
U32 interrupt_test_reg=MT6328_DEW_WRITE_TEST;
//[1]:  SIG_ERR: Signature Checking failed.	set bit[0]=1 in cmd
static S32 _int_test_bit1( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	U32 return_value=0;
	U32 wacs_write=0;
	U32 wacs_adr=0;
	U32 wacs_cmd=0;
	U32 addr=WRAP_ACCESS_TEST_REG;
	PWRAPFUC();
	int_test_bit=1;
	wait_int_flag=0;
	WRAP_WR32(PMIC_WRAP_INT_EN,(3<<1)|WRAP_RD32(PMIC_WRAP_INT_EN));

#if 1 //sig_value mode
#ifdef SLV_6328
	pwrap_write(MT6328_DEW_WRITE_TEST, MT6328_WRITE_TEST_VALUE);
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN, 0x5);
#endif
#ifdef SLV_6332
	pwrap_write(MT6332_DEW_WRITE_TEST, MT6332_WRITE_TEST_VALUE);
	WRAP_WR32(PMIC_WRAP_STAUPD_GRPEN, WRAP_RD32(PMIC_WRAP_STAUPD_GRPEN)|0xa);
#endif
	WRAP_WR32(PMIC_WRAP_SIG_ADR,(MT6332_DEW_WRITE_TEST<<16)|MT6328_DEW_WRITE_TEST);
  	WRAP_WR32(PMIC_WRAP_SIG_VALUE,(0xAA55<<16)|0xAA55);
	WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x3);

	mdelay(1000);//delay 5 seconds

	//tigger interrupt of wrapper Now!!!
#endif
	PWRAPLOG("_int_test_bit1 PMIC_WRAP_INT_FLG=0x%x.\n",WRAP_RD32(PMIC_WRAP_INT_FLG));
	wait_for_completion(&pwrap_done);
	WRAP_WR32(PMIC_WRAP_INT_CLR,1<<1);
	WRAP_WR32(PMIC_WRAP_SIG_VALUE,(MT6332_WRITE_TEST_VALUE<<16)|MT6328_WRITE_TEST_VALUE);//tha same as write test
	//clear sig_error interrupt flag bit
	WRAP_WR32(PMIC_WRAP_INT_CLR,1<<1);

	WRAP_WR32(PMIC_WRAP_SIG_MODE, 0x0);
	WRAP_WR32(PMIC_WRAP_SIG_ADR , (MT6332_DEW_CRC_VAL<<16 | MT6328_DEW_CRC_VAL));
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit1 pass.\n");
	else
		PWRAPLOG("_int_test_bit1 fail.\n");
	return 0;
}


//[5]:  MAN_CMD_MISS: A MAN CMD is written while MAN is disabled.
//    disable man,send a manual command
static S32 _int_test_bit5( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	U32 return_value=0;
	PWRAPFUC();
	int_test_bit=5;
	wait_int_flag=0;
	WRAP_WR32(PMIC_WRAP_MAN_EN , 0);// disable man

	return_value=_pwrap_manual_mode(OP_WR,  OP_CSH,  0, &rdata);
	PWRAPLOG("return_value of _pwrap_manual_mode=%x.\n",return_value);

	wait_for_completion(&pwrap_done);
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit5 pass.\n");
	else
		PWRAPLOG("_int_test_bit5 fail.\n");
	return 0;
}

//[14]: WACS0_CMD_MISS: A WACS0 CMD is written while WACS0 is disabled.
//    disable man,send a wacs0 command
static S32 _int_test_bit14( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	int_test_bit=14;
	wait_int_flag=0;
	WRAP_WR32(PMIC_WRAP_WACS0_EN , 0);// disable man

	pwrap_wacs0(1, WRAP_ACCESS_TEST_REG, 0x55AA, &rdata);
	wait_for_completion(&pwrap_done);
	mdelay(1000);/*completion invalid, need delay*/
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit14 pass.\n");
	else
		PWRAPLOG("_int_test_bit14 fail.\n");
	return 0;
}

//[17]: WACS1_CMD_MISS: A WACS1 CMD is written while WACS1 is disabled.
//    disable man,send a wacs0 command
static S32 _int_test_bit17( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	U32 return_value=0;
	PWRAPFUC();
	int_test_bit=17;
	wait_int_flag=0;
	WRAP_WR32(PMIC_WRAP_WACS1_EN , 0);// disable man
    PWRAPLOG("fwq write wacs1 before.\n");
	pwrap_wacs1(1, WRAP_ACCESS_TEST_REG, 0x55AA, &rdata);

	pwrap_wacs1(1, MT6328_DEW_WRITE_TEST, MT6328_WRITE_TEST_VALUE, &rdata);
	return_value=pwrap_wacs1(0, MT6328_DEW_WRITE_TEST, 0, &rdata);
	if( rdata != MT6328_WRITE_TEST_VALUE )
	{
		PWRAPERR("write test error(using WACS1),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
	PWRAPERR("write test pass(using WACS1),return_value=%x, rdata=%x\n", return_value, rdata);
	PWRAPLOG("fwq write wacs1 end.\n");
	wait_for_completion(&pwrap_done);
	mdelay(1000);/*completion invalid, need delay*/
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit17 pass.\n");
	else
		PWRAPLOG("_int_test_bit17 fail.\n");
	return 0;
}

//[20]: WACS2_CMD_MISS: A WACS2 CMD is written while WACS2 is disabled.
//    disable man,send a wacs2 command
static S32 _int_test_bit20( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	int_test_bit=20;
	wait_int_flag=0;
	WRAP_WR32(PMIC_WRAP_WACS2_EN , 0);// disable man

	pwrap_write(WRAP_ACCESS_TEST_REG, 0x55AA);
	wait_for_completion(&pwrap_done);
	mdelay(1000);/*completion invalid, need delay*/
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit20 pass.\n");
	else
		PWRAPLOG("_int_test_bit20 fail.\n");
	return 0;
}

//[4]:  MAN_UNEXP_VLDCLR: MAN unexpected VLDCLR
//    send a manual write command,and clear valid big
static S32 _int_test_bit3( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	U32 return_value;
	PWRAPFUC();
	int_test_bit=3;
	wait_int_flag=0;
	_pwrap_switch_mux(1);
	return_value=_pwrap_manual_mode(OP_WR,  OP_CSH,  0, &rdata);
	PWRAPLOG("return_value of _pwrap_manual_mode=%x.\n",return_value);
	WRAP_WR32(PMIC_WRAP_MAN_VLDCLR , 1);
	wait_for_completion(&pwrap_done);
	mdelay(1000);/*completion invalid, need delay*/
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit3 pass.\n");
	else
		PWRAPLOG("_int_test_bit3 fail.\n");
	return 0;
}//[12]: WACS0_UNEXP_VLDCLR: WACS0 unexpected VLDCLR
//    send a wacs0 write command,and clear valid big
static S32 _int_test_bit12( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	int_test_bit=12;
	wait_int_flag=0;
	pwrap_wacs0(1, WRAP_ACCESS_TEST_REG, 0x55AA, &rdata);
	WRAP_WR32(PMIC_WRAP_WACS0_VLDCLR , 1);
	wait_for_completion(&pwrap_done);
	mdelay(1000);/*completion invalid, need delay*/
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit12 pass.\n");
	else
		PWRAPLOG("_int_test_bit12 fail.\n");
	return 0;
}//[15]: WACS1_UNEXP_VLDCLR: WACS1 unexpected VLDCLR
//    send a wacs1 write command,and clear valid big
static S32 _int_test_bit15( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	int_test_bit=15;
	wait_int_flag=0;
	pwrap_wacs1(1, WRAP_ACCESS_TEST_REG, 0x55AA, &rdata);
	WRAP_WR32(PMIC_WRAP_WACS1_VLDCLR , 1);

	wait_for_completion(&pwrap_done);
	mdelay(1000);/*completion invalid, need delay*/
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit15 pass.\n");
	else
		PWRAPLOG("_int_test_bit15 fail.\n");
	return 0;
}//[18]: WACS2_UNEXP_VLDCLR: WACS2 unexpected VLDCLR
//    send a wacs2 write command,and clear valid big
static S32 _int_test_bit18( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	int_test_bit=18;
	wait_int_flag=0;
	pwrap_write(WRAP_ACCESS_TEST_REG, 0x55AA);
	WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR , 1);

	wait_for_completion(&pwrap_done);
	mdelay(1000);/*completion invalid, need delay*/
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit18 pass.\n");
	else
		PWRAPLOG("_int_test_bit18 fail.\n");
	return 0;
}//[21]: PERI_WRAP_INT: PERI_PWRAP_BRIDGE interrupt is asserted.
//    send a wacs3 write command,and clear valid big

static S32 _int_test_bit27( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	int_test_bit=27;
	wait_int_flag=0;
	pwrap_write(WRAP_ACCESS_TEST_REG, 0x55AA);
	WRAP_WR32(PMIC_WRAP_WACS3_VLDCLR , 1);

	wait_for_completion(&pwrap_done);
	mdelay(1000);/*completion invalid, need delay*/
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit27 pass.\n");
	else
		PWRAPLOG("_int_test_bit27 fail.\n");
	return 0;
}

static S32 _int_test_bit29( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	U32 return_value=0;
	PWRAPFUC();
	int_test_bit=29;
	wait_int_flag=0;
	WRAP_WR32(PMIC_WRAP_WACS3_EN , 0);// disable man
    PWRAPLOG("fwq write wacs3 before.\n");
	pwrap_wacs3(1, WRAP_ACCESS_TEST_REG, 0x55AA, &rdata);

	pwrap_wacs3(1, MT6328_DEW_WRITE_TEST, MT6328_WRITE_TEST_VALUE, &rdata);
	return_value=pwrap_wacs3(0, MT6328_DEW_WRITE_TEST, 0, &rdata);
	if( rdata != MT6328_WRITE_TEST_VALUE )
	{
		PWRAPERR("write test error(using WACS3),return_value=%x, rdata=%x\n", return_value, rdata);
		res+=1;
	}
	PWRAPERR("write test pass(using WACS3),return_value=%x, rdata=%x\n", return_value, rdata);
	PWRAPLOG("fwq write wacs1 end.\n");
	wait_for_completion(&pwrap_done);
	mdelay(1000);/*completion invalid, need delay*/
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit29 pass.\n");
	else
		PWRAPLOG("_int_test_bit29 fail.\n");
	return 0;
}


#if 0
static S32 _int_test_bit21( )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 res=0;
	PWRAPFUC();
	int_test_bit=21;

	wait_int_flag=0;
	//pwrap_wacs3(1, WRAP_ACCESS_TEST_REG, 0x55AA, &rdata);
	//WRAP_WR32(PERI_PWRAP_BRIDGE_WACS3_VLDCLR , 1);

	wait_for_completion(&pwrap_done);
	mdelay(1000);/*completion invalid, need delay*/
	if(wait_int_flag==1)
		PWRAPLOG("_int_test_bit21 pass.\n");
	else
		PWRAPLOG("_int_test_bit21 fail.\n");
	return 0;
}
#endif
static S32 _int_test_disable_watch_dog(void)
{
	//disable watch dog
	WRAP_WR32(PMIC_WRAP_WDT_SRC_EN,0);
	return 0;
}


static S32 tc_int_test(  )
{
	UINT32 return_value=0;
	UINT32 test_this_case=0;
	mt_wrp_dvt->complete = pwrap_complete;
	mt_wrp_dvt->context = &pwrap_done;

	mt_wrp_dvt->irq_mode = INT_TEST;
	/*
	pwrap_init();
	_int_test_disable_watch_dog();
	//return_value=1;
	return_value+=_int_test_bit1();
	mdelay(1000);
	*/
	
	pwrap_init();
	_int_test_disable_watch_dog();
	return_value+=_int_test_bit5();
	mdelay(1000);
	
	
	pwrap_init();
	_int_test_disable_watch_dog();
	return_value+=_int_test_bit14();
	mdelay(1000);
	
	pwrap_init();
	_int_test_disable_watch_dog();
	return_value+=_int_test_bit17();
	mdelay(1000);
	
	pwrap_init();
	_int_test_disable_watch_dog();
	return_value+=_int_test_bit20();
	mdelay(1000);
	
	pwrap_init();
	_int_test_disable_watch_dog();
	return_value+=_int_test_bit3();
	mdelay(1000);
	
	pwrap_init();
	_int_test_disable_watch_dog();
	return_value+=_int_test_bit12();
	mdelay(1000);
	
	pwrap_init();
	_int_test_disable_watch_dog();
	return_value+=_int_test_bit15();
	mdelay(1000);
	
	pwrap_init();
	_int_test_disable_watch_dog();
	return_value+=_int_test_bit18();
	mdelay(1000);


	pwrap_init();
	_int_test_disable_watch_dog();
	return_value+=_int_test_bit27();
	mdelay(1000);

	pwrap_init();
	_int_test_disable_watch_dog();
	return_value+=_int_test_bit29();
	mdelay(1000);
	
	//pwrap_init();
	//_int_test_disable_watch_dog();

	//return_value+=_int_test_bit21();
	//pwrap_init();
	if(return_value==0)
	{
		PWRAPLOG("DVT_WRAP_int_test pass.\n");

	}
	else
	{
		PWRAPLOG("DVT_WRAP_int_test fail.res=%d\n",return_value);

	}
	mt_wrp_dvt->irq_mode = NORMAL_TEST;
	return return_value;
}
#define CLK_CFG_4_SET 0xF0000000
#if 0
static void pwrap_power_off(void)
{
	WRAP_SET_BIT(0xE0,CLK_CFG_4_SET);//SPI clock

}
#endif
#if 0
static S32 tc_clock_gating_test(  )
{
	UINT32 return_value=0;
	UINT32 test_this_case=0;
	PWRAPFUC();
	pwrap_power_off();//need to follow up
	return_value=_pwrap_wrap_access_test();
	if(return_value==0)
	{
		PWRAPLOG("tc_clock_gating_test pass.\n");
		PWRAPLOG("tc_clock_gating_test fail.res=%d\n",return_value);
	}
	return return_value;

}
#endif
volatile U32 index_wacs0=0;
volatile U32 index_wacs1=0;
volatile U32 index_wacs2=0;
U64 start_time_wacs0=0;
U64 start_time_wacs1=0;
U64 start_time_wacs2=0;
U64 end_time_wacs0=0;
U64 end_time_wacs1=0;
U64 end_time_wacs2=0;
U32 WACS0_TEST_REG=MT6328_DEW_WRITE_TEST;
U32 WACS1_TEST_REG=MT6332_DEW_WRITE_TEST;
///TODO:fix me
U32 WACS2_TEST_REG=0x0546; //SYSLDO_CON21   
//U32 WACS3_TEST_REG=MT6328_DEW_WRITE_TEST;
//U32 WACS4_TEST_REG=MT6328_DEW_WRITE_TEST;
#if 0
static void _throughput_wacs0_test( void )
{
	U32 i=0;
	U32 rdata=0;
	PWRAPFUC();
	start_time_wacs0=sched_clock();
	for(index_wacs0=0;index_wacs0<10000;index_wacs0++)
	{
		pwrap_wacs0(0, WACS0_TEST_REG, 0, &rdata);
	}
	end_time_wacs0=sched_clock();
	PWRAPLOG("_throughput_wacs0_test send 10000 read command:average time(ns)=%llx.\n",(end_time_wacs0-start_time_wacs0));
	PWRAPLOG("index_wacs0=%d index_wacs1=%d index_wacs2=%d\n",index_wacs0,index_wacs1,index_wacs2);
	PWRAPLOG("start_time_wacs0=%llx start_time_wacs1=%llx start_time_wacs2=%llx\n",start_time_wacs0,start_time_wacs1,start_time_wacs2);
	PWRAPLOG("end_time_wacs0=%llx end_time_wacs1=%llx end_time_wacs2=%llx\n",end_time_wacs0,end_time_wacs1,end_time_wacs2);
}
static void _throughput_wacs1_test( void )
{
	//U32 i=0;
	U32 rdata=0;
	PWRAPFUC();
	start_time_wacs1=sched_clock();
	for(index_wacs1=0;index_wacs1<10000;index_wacs1++)
	{
		pwrap_wacs1(0, WACS1_TEST_REG, 0, &rdata);
	}
	end_time_wacs1=sched_clock();
	PWRAPLOG("_throughput_wacs1_test send 10000 read command:average time(ns)=%llx.\n",(end_time_wacs1-start_time_wacs1));
	PWRAPLOG("index_wacs0=%d index_wacs1=%d index_wacs2=%d\n",index_wacs0,index_wacs1,index_wacs2);
	PWRAPLOG("start_time_wacs0=%llx start_time_wacs1=%llx start_time_wacs2=%llx\n",start_time_wacs0,start_time_wacs1,start_time_wacs2);
	PWRAPLOG("end_time_wacs0=%llx end_time_wacs1=%llx end_time_wacs2=%llx\n",end_time_wacs0,end_time_wacs1,end_time_wacs2);
}
#endif
#if 0
static void _throughput_wacs2_test( void )
{
	U32 i=0;
	U32 rdata=0;
	U32 return_value=0;
	PWRAPFUC();
	start_time_wacs2=sched_clock();
	for(index_wacs2=0;index_wacs2<10000;index_wacs2++)
	{
		return_value=pwrap_wacs2(0, WACS2_TEST_REG, 0, &rdata);
		//      if(return_value!=0)
		//        PWRAPLOG("return_value=%d.index_wacs2=%d\n",return_value,index_wacs2);
	}
	end_time_wacs2=sched_clock();
	PWRAPLOG("_throughput_wacs2_test send 10000 read command:average time(ns)=%llx.\n",(end_time_wacs2-start_time_wacs2));
	PWRAPLOG("index_wacs0=%d index_wacs1=%d index_wacs2=%d\n",index_wacs0,index_wacs1,index_wacs2);
	PWRAPLOG("start_time_wacs0=%llx start_time_wacs1=%llx start_time_wacs2=%llx\n",start_time_wacs0,start_time_wacs1,start_time_wacs2);
	PWRAPLOG("end_time_wacs0=%llx end_time_wacs1=%llx end_time_wacs2=%llx\n",end_time_wacs0,end_time_wacs1,end_time_wacs2);
}
#endif

//#ifdef PWRAP_CONCURRENCE_TEST

//###############################concurrence_test start#########################
//---define wacs direction flag:  read:WACS0_READ_WRITE_FLAG=0;write:WACS0_READ_WRITE_FLAG=0;
//#define RANDOM_TEST
//#define NORMAL_TEST
//#define stress_test_on_concurrence

//static U8 wacs0_send_write_cmd_done=0;
//static U8 wacs0_send_read_cmd_done=0;
//static U8 wacs0_read_write_flag=0;


//static U8 wacs1_send_write_cmd_done=0;
//static U8 wacs1_send_read_cmd_done=0;
//static U8 wacs1_read_write_flag=0;

//static U8 wacs2_send_write_cmd_done=0;
//static U8 wacs2_send_read_cmd_done=0;
//static U8 wacs2_read_write_flag=0;


static U16 wacs0_test_value=0x10;
static U16 wacs1_test_value=0x20;
static U16 wacs2_test_value=0x30;


U32 wacs_read_cmd_done=0;
//U32 test_count0=100000000;
//U32 test_count1=100000000;
U32 test_count0=0;
U32 test_count1=0;




//static U16 concurrence_fail_count_cpu0=0;
//static U16 concurrence_fail_count_cpu1=0;
//static U16 concurrence_pass_count_cpu0=0;
//static U16 concurrence_pass_count_cpu1=0;


U32 g_spm_pass_count0=0;
U32 g_spm_fail_count0=0;
U32 g_spm_pass_count1=0;
U32 g_spm_fail_count1=0;

U32 g_pwm_pass_count0=0;
U32 g_pwm_fail_count0=0;
U32 g_pwm_pass_count1=0;
U32 g_pwm_fail_count1=0;

U32 g_wacs0_pass_count0=0;
U32 g_wacs0_fail_count0=0;
U32 g_wacs0_pass_count1=0;
U32 g_wacs0_fail_count1=0;

U32 g_wacs1_pass_count0=0;
U32 g_wacs1_fail_count0=0;
U32 g_wacs1_pass_count1=0;
U32 g_wacs1_fail_count1=0;

U32 g_wacs2_pass_count0=0;
U32 g_wacs2_fail_count0=0;
U32 g_wacs2_pass_count1=0;
U32 g_wacs2_fail_count1=0;


U32 g_stress0_cpu0_count=0;
U32 g_stress1_cpu0_count=0;
U32 g_stress2_cpu0_count=0;
U32 g_stress3_cpu0_count=0;
U32 g_stress4_cpu0_count=0;
//U32 g_stress5_cpu0_count=0;
U32 g_stress0_cpu1_count=0;
U32 g_stress1_cpu1_count=0;
U32 g_stress2_cpu1_count=0;
U32 g_stress3_cpu1_count=0;
U32 g_stress4_cpu1_count=0;
U32 g_stress5_cpu1_count=0;

U32 g_stress0_cpu0_count0=0;
U32 g_stress1_cpu0_count0=0;
U32 g_stress0_cpu1_count0=0;

U32 g_stress0_cpu0_count1=0;
U32 g_stress1_cpu0_count1=0;
U32 g_stress0_cpu1_count1=0;

U32 g_stress2_cpu0_count1=0;
U32 g_stress3_cpu0_count1=0;

U32 g_random_count0=0;
U32 g_random_count1=0;
U32 g_wacs0_pass_cpu0=0;
U32 g_wacs0_pass_cpu1=0;
U32 g_wacs0_pass_cpu2=0;
U32 g_wacs0_pass_cpu3=0;

U32 g_wacs0_fail_cpu0=0;
U32 g_wacs0_fail_cpu1=0;
U32 g_wacs0_fail_cpu2=0;
U32 g_wacs0_fail_cpu3=0;

U32 g_wacs1_pass_cpu0=0;
U32 g_wacs1_pass_cpu1=0;
U32 g_wacs1_pass_cpu2=0;
U32 g_wacs1_pass_cpu3=0;

U32 g_wacs1_fail_cpu0=0;
U32 g_wacs1_fail_cpu1=0;
U32 g_wacs1_fail_cpu2=0;
U32 g_wacs1_fail_cpu3=0;

U32 g_wacs2_pass_cpu0=0;
U32 g_wacs2_pass_cpu1=0;
U32 g_wacs2_pass_cpu2=0;
U32 g_wacs2_pass_cpu3=0;

U32 g_wacs2_fail_cpu0=0;
U32 g_wacs2_fail_cpu1=0;
U32 g_wacs2_fail_cpu2=0;
U32 g_wacs2_fail_cpu3=0;

#if 0


//--------------------------------------------------------
//    Function : pwrap_wacs0()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
static S32 _concurrence_pwrap_wacs0( U32 write, U32 adr, U32 wdata, U32 *rdata,U32 read_cmd_done )
{
	U32 reg_rdata=0;
	U32 wacs_write=0;
	U32 wacs_adr=0;
	U32 wacs_cmd=0;
	//PWRAPFUC();
	if(read_cmd_done==0)
	{
		reg_rdata = WRAP_RD32(PMIC_WRAP_WACS0_RDATA);
		if( GET_INIT_DONE0( reg_rdata ) != 1)
		{
			PWRAPERR("initialization isn't finished when write data\n");
			return 1;
		}
		if( GET_WACS0_FSM( reg_rdata ) != WACS_FSM_IDLE) //IDLE State
		{
			PWRAPERR("WACS0 is not in IDLE state\n");
			return 2;
		}
		// check argument validation
		if( (write & ~(0x1))    != 0)  return 3;
		if( (adr   & ~(0xffff)) != 0)  return 4;
		if( (wdata & ~(0xffff)) != 0)  return 5;

		wacs_write  = write << 31;
		wacs_adr    = (adr >> 1) << 16;
		wacs_cmd= wacs_write | wacs_adr | wdata;
		WRAP_WR32(PMIC_WRAP_WACS0_CMD,wacs_cmd);
	}
	else
	{
		if( write == 0 )
		{
			do
			{
				reg_rdata = WRAP_RD32(PMIC_WRAP_WACS0_RDATA);
				if( GET_INIT_DONE0( reg_rdata ) != 1)
				{
					//wrapper may be reset when error happen,so need to check if init is done
					PWRAPERR("initialization isn't finished when read data\n");
					return 6;
				}
			} while( GET_WACS0_FSM( reg_rdata ) != WACS_FSM_WFVLDCLR ); //WFVLDCLR

			*rdata = GET_WACS0_RDATA( reg_rdata );
			WRAP_WR32(PMIC_WRAP_WACS0_VLDCLR , 1);
		}
	}
	return 0;
}
//--------------------------------------------------------
//    Function : pwrap_wacs1()
// Description :
//   Parameter :
//      Return :
//--------------------------------------------------------
static S32 _concurrence_pwrap_wacs1( U32  write, U32  adr, U32  wdata, U32 *rdata ,U32 read_cmd_done)
{
	U32 reg_rdata=0;
	U32 wacs_write=0;
	U32 wacs_adr=0;
	U32 wacs_cmd=0;
	if(read_cmd_done==0)
	{
		//PWRAPFUC();
		reg_rdata = WRAP_RD32(PMIC_WRAP_WACS1_RDATA);
		if( GET_INIT_DONE0( reg_rdata ) != 1)
		{
			PWRAPERR("initialization isn't finished when write data\n");
			return 1;
		}
		if( GET_WACS0_FSM( reg_rdata ) != WACS_FSM_IDLE) //IDLE State
		{
			PWRAPERR("WACS1 is not in IDLE state\n");
			return 2;
		}
		// check argument validation
		if( (write & ~(0x1))    != 0)  return 3;
		if( (adr   & ~(0xffff)) != 0)  return 4;
		if( (wdata & ~(0xffff)) != 0)  return 5;

		wacs_write  = write << 31;
		wacs_adr    = (adr >> 1) << 16;
		wacs_cmd= wacs_write | wacs_adr | wdata;

		WRAP_WR32(PMIC_WRAP_WACS1_CMD,wacs_cmd);
	}
	else
	{
		if( write == 0 )
		{
			do
			{
				reg_rdata = WRAP_RD32(PMIC_WRAP_WACS1_RDATA);
				if( GET_INIT_DONE0( reg_rdata ) != 1)
				{
					//wrapper may be reset when error happen,so need to check if init is done
					PWRAPERR("initialization isn't finished when read data\n");
					return 6;
				}
			} while( GET_WACS0_FSM( reg_rdata ) != WACS_FSM_WFVLDCLR ); //WFVLDCLR State

			*rdata = GET_WACS0_RDATA( reg_rdata );
			WRAP_WR32(PMIC_WRAP_WACS1_VLDCLR , 1);
		}
	}
	return 0;
}
//----wacs API implement for concurrence test-----------------------
static S32 _concurrence_pwrap_wacs2( U32  write, U32  adr, U32  wdata, U32 *rdata, U32 read_cmd_done )
{
	U32 reg_rdata=0;
	U32 wacs_write=0;
	U32 wacs_adr=0;
	U32 wacs_cmd=0;
	if(read_cmd_done==0)
	{
		//PWRAPFUC();
		reg_rdata = WRAP_RD32(PMIC_WRAP_WACS2_RDATA);
		if( GET_INIT_DONE0( reg_rdata ) != 1)
			return 1;
		if( GET_WACS0_FSM( reg_rdata ) != WACS_FSM_IDLE) //IDLE State
		{
			PWRAPERR("WACS2 is not in IDLE state\n");
			return 2;
		}

		// check argument validation
		if( (write & ~(0x1))    != 0)  return 3;
		if( (adr   & ~(0xffff)) != 0)  return 4;
		if( (wdata & ~(0xffff)) != 0)  return 5;

		wacs_write  = write << 31;
		wacs_adr    = (adr >> 1) << 16;
		wacs_cmd= wacs_write | wacs_adr | wdata;

		WRAP_WR32(PMIC_WRAP_WACS2_CMD,wacs_cmd);
	}
	else
	{
		if( write == 0 )
		{
			do
			{
				reg_rdata = WRAP_RD32(PMIC_WRAP_WACS2_RDATA);
				//if( GET_INIT_DONE0( reg_rdata ) != 1)
				//  return 3;
			} while( GET_WACS0_FSM( reg_rdata ) != WACS_FSM_WFVLDCLR ); //WFVLDCLR

			*rdata = GET_WACS0_RDATA( reg_rdata );
			WRAP_WR32(PMIC_WRAP_WACS2_VLDCLR , 1);
		}
	}
	return 0;
}

#endif

static void _concurrence_wacs0_test( void )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 rand_number=0;
	PWRAPFUC();
	while(1)
	{
		//msleep(5);
#ifdef RANDOM_TEST
		rand_number=(U32)random32();
		if((rand_number%2)==1)
			msleep(10);
		else
#endif
		{
			rdata = 0;
			pwrap_wacs0(1, WACS0_TEST_REG, wacs0_test_value, &rdata);
			//printk("write (using WACS0),value=%x\n", wacs0_test_value);
			//mdelay(1);
			pwrap_wacs0(0, WACS0_TEST_REG, wacs0_test_value, &rdata);
			//mdelay(1);
			//printk("read (using WACS0),rdata=%x\n", rdata);

			if( rdata != wacs0_test_value )
			{
				g_wacs0_fail_count0++;
				pwrap_dump_all_register();
				PWRAPERR("read test error(using WACS0),wacs0_test_value=%x, rdata=%x\n", wacs0_test_value, rdata);
				switch ( raw_smp_processor_id())
				{
					case 0:
						g_wacs0_fail_cpu0++;
						break;
					case 1:
						g_wacs0_fail_cpu1++;
						break;
					case 2:
						g_wacs0_fail_cpu2++;
						break;
					case 3:
						g_wacs0_fail_cpu3++;
						break;
					default:
						break;
				}
				//PWRAPERR("concurrence_fail_count_cpu2=%d", ++concurrence_fail_count_cpu0);
			}
			else
			{
				g_wacs0_pass_count0++;
				//PWRAPLOG("WACS0 concurrence_test pass,rdata=%x.\n",rdata);
				//PWRAPLOG("WACS0 concurrence_test pass,concurrence_pass_count_cpu0=%d\n",++concurrence_pass_count_cpu0);

				switch ( raw_smp_processor_id())
				{
					case 0:
						g_wacs0_pass_cpu0++;
						break;
					case 1:
						g_wacs0_pass_cpu1++;
						break;
					case 2:
						g_wacs0_pass_cpu2++;
						break;
					case 3:
						g_wacs0_pass_cpu3++;
						break;
					default:
						break;
				}
			}
			wacs0_test_value+=0x1;

		}
	}//end of while(1)
}
static void _concurrence_wacs1_test(void)
{
#if 1
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 rand_number=0;
	PWRAPFUC();
	while(1)
	{
		//msleep(10);
/*
#ifdef RANDOM_TEST
		rand_number=(U32)random32();
		if((rand_number%2)==1)
			msleep(10);
		else
#endif
*/
		{
			rdata = 0;
			pwrap_wacs1(1, WACS1_TEST_REG, wacs1_test_value, &rdata);
			//printk("write (using WACS1),value=%x\n", wacs1_test_value);
			//mdelay(1);
			pwrap_wacs1(0, WACS1_TEST_REG, wacs1_test_value, &rdata);
			//printk("read  (using WACS1),rdata=%x\n", rdata);
			//mdelay(1);
			if( rdata != wacs1_test_value )
			{
				g_wacs1_fail_count0++;
				pwrap_dump_all_register();
				PWRAPERR("read test error(using WACS1),wacs1_test_value=%x, rdata=%x\n", wacs1_test_value, rdata);
				switch ( raw_smp_processor_id())
				{
					case 0:
						g_wacs1_fail_cpu0++;
						break;
					case 1:
						g_wacs1_fail_cpu1++;
						break;
					case 2:
						g_wacs1_fail_cpu2++;
						break;
					case 3:
						g_wacs1_fail_cpu3++;
						break;
					default:
						break;
				}
				// PWRAPERR("concurrence_fail_count_cpu1=%d", ++concurrence_fail_count_cpu1);
			}
			else
			{
				g_wacs1_pass_count0++;
				switch ( raw_smp_processor_id())
				{
					case 0:
						g_wacs1_pass_cpu0++;
						break;
					case 1:
						g_wacs1_pass_cpu1++;
						break;
					case 2:
						g_wacs1_pass_cpu2++;
						break;
					case 3:
						g_wacs1_pass_cpu3++;
						break;
					default:
						break;
				}
			}
			wacs1_test_value+=0x3;
		}
	}//end of while(1)
#endif
}
static void _concurrence_wacs2_test( void )
{
	U32 rdata=0;
	U32 reg_rdata=0;
	U32 rand_number=0;
	PWRAPFUC();
	while(1)
	{
		//msleep(10);
#ifdef RANDOM_TEST
		rand_number=(U32)random32();
		if((rand_number%2)==1)
			msleep(10);
		else
#endif
		{

			rdata = 0;
			pwrap_write(WACS2_TEST_REG, wacs2_test_value);
			//printk("write (using WACS2),value=%x\n", wacs2_test_value);
			pwrap_read(WACS2_TEST_REG,  &rdata);
			if( rdata != wacs2_test_value )
			{
				g_wacs2_fail_count0++;
				pwrap_dump_all_register();
				switch ( raw_smp_processor_id())
				{
					case 0:
						g_wacs2_fail_cpu0++;
						break;
					case 1:
						g_wacs2_fail_cpu1++;
						break;
					case 2:
						g_wacs2_fail_cpu2++;
						break;
					case 3:
						g_wacs2_fail_cpu3++;
						break;
					default:
						break;
				}
				PWRAPERR("read test error(using WACS2),wacs2_test_value=%x, rdata=%x\n", wacs2_test_value, rdata);
			}
			else
			{
				g_wacs2_pass_count0++;
				switch ( raw_smp_processor_id())
				{
					case 0:
						g_wacs2_pass_cpu0++;
						break;
					case 1:
						g_wacs2_pass_cpu1++;
						break;
					case 2:
						g_wacs2_pass_cpu2++;
						break;
					case 3:
						g_wacs2_pass_cpu3++;
						break;
					default:
						break;
				}
			}////end of if( rdata != wacs2_test_value )
			wacs2_test_value+=0x2;
		}
	}//end of while(1)
}


U32 spm_task=0;
U32 spm_cpu_id=1;

struct task_struct *  wacs0_task=0;
U32 wacs0_cpu_id=1;
struct task_struct *  wacs1_task=0;
U32 wacs1_cpu_id=1;
struct task_struct *  wacs2_task=0;
U32 wacs2_cpu_id=1;


struct task_struct *  log0_task=0;
U32 log0_cpu_id=0;

struct task_struct * log1_task=0;
U32 log1_cpu_id=1;

U32 kthread_stress0_cpu0=0;
U32 stress0_cpu_id=0;

U32 kthread_stress1_cpu0=0;
U32 stress1_cpu_id=0;

U32 kthread_stress2_cpu0=0;
U32 stress2_cpu_id=0;

U32 kthread_stress3_cpu0=0;
U32 stress3_cpu_id=0;

U32 kthread_stress4_cpu0=0;
U32 stress4_cpu_id=0;

U32 kthread_stress0_cpu1=0;
U32 stress01_cpu_id=0;

U32 kthread_stress1_cpu1=0;
U32 kthread_stress2_cpu1=0;
U32 kthread_stress3_cpu1=0;
U32 kthread_stress4_cpu1=0;
U32 kthread_stress5_cpu1=0;


	static S32 _concurrence_log1(unsigned int spm)
	{
		PWRAPFUC();
		U32 i=0;
		//while(i<20)
		while(1)
		{
			//log---------------------------------------------------------------
			//if((test_count0%10000)==0)
			//if((i%100)==0)
			//{
			//PWRAPLOG("spm,pass count=%d,fail count=%d\n", g_spm_pass_count0, g_spm_fail_count0);
			PWRAPLOG("wacs0,pass count=%.10d,fail count=%d\n",g_wacs0_pass_count0,g_wacs0_fail_count0);
			PWRAPLOG("wacs1,pass count=%.10d,fail count=%d\n",g_wacs1_pass_count0,g_wacs1_fail_count0);
			PWRAPLOG("wacs2,pass count=%.10d,fail count=%d\n",g_wacs2_pass_count0,g_wacs2_fail_count0);
			PWRAPLOG("\n");
			//PWRAPLOG("g_stress0_cpu1_count=%d\n",g_stress0_cpu1_count);
#if 0
			PWRAPLOG("g_stress0_cpu0_count1=%d\n",g_stress0_cpu0_count1);
			PWRAPLOG("g_stress1_cpu0_count1=%d\n",g_stress1_cpu0_count1);
			PWRAPLOG("g_stress0_cpu1_count1=%d\n",g_stress0_cpu1_count1);
#endif

			//}
			//i++;

			msleep(2000);	
			schedule_timeout(2000*HZ);
		}

	}

#if 0
	static S32 _concurrence_stress0_cpu0(unsigned int stress)
	{
		PWRAPFUC();
		U32 i=0;
		U32 rand_number=0;

		while(1)
		{
			g_random_count0++;
			//rand_number=(U32)random32();
			if((rand_number%2)==1)
			{
				g_random_count1++;
				for(i=0;i<100000;i++)
				{
					//g_stress0_cpu0_count++;
					if (raw_smp_processor_id() == 0)
					{
						g_stress0_cpu0_count0++;
					} else if (raw_smp_processor_id() == 1)
					{
						g_stress0_cpu0_count1++;
					}
				}
			}
		}
	}
	static S32 _concurrence_stress1_cpu0(unsigned int stress)
	{
		PWRAPFUC();
		U32 rand_number=0;
		U32 i=0;
		//while(i<20)
		for(;;)
		{
			for(i=0;i<100000;i++)
			{
				//g_stress1_cpu0_count++;
				if (raw_smp_processor_id() == 0)
				{
					g_stress1_cpu0_count0++;
				} else if (raw_smp_processor_id() == 1)
				{
					g_stress1_cpu0_count1++;
				}
			}
		}
	}

	static S32 _concurrence_stress2_cpu0(unsigned int stress)
	{
		PWRAPFUC();
		U32 i=0;
		U32 rand_number=0;
		for(;;)
		{
			//rand_number=(U32)random32();
			if((rand_number%2)==1)
			{
				for(i=0;i<100000;i++)
					g_stress1_cpu0_count++;
			}
		}
	}

	static S32 _concurrence_stress3_cpu0(unsigned int stress)
	{
		PWRAPFUC();
		U32 i=0;
		U32 rand_number=0;
		for(;;)
		{
			//rand_number=(U32)random32();
			if((rand_number%2)==1)
			{
				for(i=0;i<100000;i++)
					g_stress3_cpu0_count++;
			}
		}
	}


	static S32 _concurrence_stress4_cpu0(unsigned int stress)
	{
		PWRAPFUC();
		U32 i=0;
		U32 rand_number=0;
		for(;;)
		{
			//rand_number=(U32)random32();
			if((rand_number%2)==1)
			{
				for(i=0;i<100000;i++)
					g_stress4_cpu0_count++;
			}
		}
	}

	#endif
#if 0
	static S32 _concurrence_stress0_cpu1(unsigned int stress)
	{
		PWRAPFUC();
		U32 i=0;
		U32 rand_number=0;
		for(;;)
		{
			//rand_number=(U32)random32();
			if((rand_number%2)==1)
			{
				for(i=0;i<100000;i++)
				{
					if (raw_smp_processor_id() == 0)
					{
						g_stress0_cpu1_count0++;
					} else if (raw_smp_processor_id() == 1)
					{
						g_stress0_cpu1_count1++;
					}
				}
			}
		}
	}

	#endif
#if 0
	static S32 _concurrence_stress1_cpu1(unsigned int stress)
	{
		PWRAPFUC();
		U32 i=0;
		U32 rand_number=0;
		for(;;)
		{
			//rand_number=(U32)random32();
			if((rand_number%2)==1)
			{
				for(i=0;i<100000;i++)
					g_stress1_cpu1_count++;
			}
		}
	}

	static S32 _concurrence_stress2_cpu1(unsigned int stress)
	{
		PWRAPFUC();
		U32 i=0;
		U32 rand_number=0;
		//while(i<20)
		for(;;)
		{
			//rand_number=(U32)random32();
			if((rand_number%2)==1)
			{
				for(i=0;i<100000;i++)
					g_stress2_cpu0_count++;
			}
		}
	}

	static S32 _concurrence_stress3_cpu1(unsigned int stress)
	{
		PWRAPFUC();
		U32 i=0;
		U32 rand_number=0;
		//while(i<20)
		for(;;)
		{
			//rand_number=(U32)random32();
			if((rand_number%2)==1)
			{
				for(i=0;i<100000;i++)
					g_stress3_cpu0_count++;
			}
		}
	}

	static S32 _concurrence_stress4_cpu1(unsigned int stress)
	{
		PWRAPFUC();
		U32 i=0;
		U32 rand_number=0;

		while(1)
		{
			//rand_number=(U32)random32();
			if((rand_number%2)==1)
			{
				for(i=0;i<100000;i++)
					g_stress4_cpu1_count++;
			}
		}
	}

	static S32 _concurrence_stress5_cpu1(unsigned int stress)
	{
		PWRAPFUC();
		U32 i=0;
		U32 rand_number=0;

		while(1)
		{
			//rand_number=(U32)random32();
			if((rand_number%2)==1)
			{
				for(i=0;i<100000;i++)
					g_stress5_cpu1_count++;
			}
		}
	}
#endif 
	//----wacs concurrence test start ------------------------------------------

	static S32 tc_concurrence_test(  )
	{
		UINT32 res=0;
		//U32 rdata=0;
//		U32 i=0;
		res=0;
		//struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 }; 
		///TODO:fix me

		PWRAPFUC();
		//  spm_task = kthread_create(_concurrence_spm_test_code,0,"spm_concurrence");
		//  if(IS_ERR(spm_task)){
		//    PWRAPERR("Unable to start kernelthread \n");
		//    res = -5;
		//  }
		//  //kthread_bind(spm_task, spm_cpu_id);
		//  wake_up_process(spm_task);

		PWRAPERR("fwq Utc_concurrence_test\n");

		wacs0_task = kthread_create(_concurrence_wacs0_test,0,"wacs0_concurrence");
		if(IS_ERR(wacs0_task)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		kthread_bind(wacs0_task, 0);
		wake_up_process(wacs0_task);

		wacs1_task = kthread_create(_concurrence_wacs1_test,0,"wacs1_concurrence");
		if(IS_ERR(wacs1_task)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		kthread_bind(wacs1_task, 1);
		wake_up_process(wacs1_task);

		wacs2_task = kthread_create(_concurrence_wacs2_test,0,"wacs2_concurrence");
		if(IS_ERR(wacs2_task)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		kthread_bind(wacs2_task, 2);
		wake_up_process(wacs2_task);


		//log0_task = kthread_create(_concurrence_log0,0,"log0_concurrence");
		//if(IS_ERR(log0_task)){
		//	PWRAPERR("Unable to start kernelthread \n");
		//	res = -5;
		//}
		//sched_setscheduler(log0_task, SCHED_FIFO, &param);
		//kthread_bind(log0_task, log0_cpu_id);
		//wake_up_process(log0_task);

		log1_task = kthread_create(_concurrence_log1,0,"log1_concurrence");
		if(IS_ERR(log1_task)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		//sched_setscheduler(log1_task, SCHED_FIFO, &param);
		kthread_bind(log1_task, log1_cpu_id);
		wake_up_process(log1_task);
#ifdef stress_test_on_concurrence
		//increase cpu load
		kthread_stress0_cpu0 = kthread_create(_concurrence_stress0_cpu0,0,"stress0_cpu0_concurrence");
		if(IS_ERR(kthread_stress0_cpu0)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		kthread_bind(kthread_stress0_cpu0, 0);
		wake_up_process(kthread_stress0_cpu0);

		kthread_stress1_cpu0 = kthread_create(_concurrence_stress1_cpu0,0,"stress0_cpu1_concurrence");
		if(IS_ERR(kthread_stress1_cpu0)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		kthread_bind(kthread_stress1_cpu0, 0);
		wake_up_process(kthread_stress1_cpu0);

		kthread_stress2_cpu0 = kthread_create(_concurrence_stress2_cpu0,0,"stress0_cpu1_concurrence");
		if(IS_ERR(kthread_stress2_cpu0)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		//kthread_bind(kthread_stress2_cpu0, 0);
		wake_up_process(kthread_stress2_cpu0);

		kthread_stress3_cpu0 = kthread_create(_concurrence_stress3_cpu0,0,"stress0_cpu1_concurrence");
		if(IS_ERR(kthread_stress3_cpu0)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		//kthread_bind(kthread_stress3_cpu0, 0);
		wake_up_process(kthread_stress3_cpu0);

		//kthread_stress4_cpu0 = kthread_create(_concurrence_stress4_cpu0,0,"stress0_cpu1_concurrence");
		if(IS_ERR(kthread_stress4_cpu0)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		//kthread_bind(kthread_stress4_cpu0, 1);
		//wake_up_process(kthread_stress4_cpu0);

		kthread_stress0_cpu1 = kthread_create(_concurrence_stress0_cpu1,0,"stress0_cpu1_concurrence");
		if(IS_ERR(kthread_stress0_cpu1)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		kthread_bind(kthread_stress0_cpu1, 1);
		wake_up_process(kthread_stress0_cpu1);

		kthread_stress1_cpu1 = kthread_create(_concurrence_stress1_cpu1,0,"stress0_cpu1_concurrence");
		if(IS_ERR(kthread_stress1_cpu1)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		//kthread_bind(kthread_stress1_cpu1, 1);
		wake_up_process(kthread_stress1_cpu1);

		kthread_stress2_cpu1 = kthread_create(_concurrence_stress2_cpu1,0,"stress0_cpu1_concurrence");
		if(IS_ERR(kthread_stress2_cpu1)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		//kthread_bind(kthread_stress2_cpu1, 0);
		wake_up_process(kthread_stress2_cpu1);

		kthread_stress3_cpu1 = kthread_create(_concurrence_stress3_cpu1,0,"stress0_cpu1_concurrence");
		if(IS_ERR(kthread_stress3_cpu1)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		//kthread_bind(kthread_stress3_cpu1, 1);
		wake_up_process(kthread_stress3_cpu1);

		kthread_stress4_cpu1 = kthread_create(_concurrence_stress4_cpu1,0,"stress0_cpu1_concurrence");
		if(IS_ERR(kthread_stress3_cpu1)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		//kthread_bind(kthread_stress4_cpu1, 1);
		wake_up_process(kthread_stress4_cpu1);

		kthread_stress5_cpu1 = kthread_create(_concurrence_stress5_cpu1,0,"stress0_cpu1_concurrence");
		if(IS_ERR(kthread_stress3_cpu1)){
			PWRAPERR("Unable to start kernelthread \n");
			res = -5;
		}
		//kthread_bind(kthread_stress5_cpu1, 1);
		wake_up_process(kthread_stress5_cpu1);

#endif //stress test
		if(res == 0)
		{
			//delay 8 hour
			/*
			U32 i,j;
			for(i=0;i<8;i++)
				for(j=0;j<60;j++)
					msleep(60000);
				*/

			msleep(6000);
			PWRAPLOG("fwq stop concurrent thread \n");

			//kthread_stop(spm_task);
			kthread_stop(wacs0_task);
			kthread_stop(wacs1_task);
			kthread_stop(wacs2_task);
			//kthread_stop(log0_task);
			kthread_stop(log1_task);
		}

		if(res==0)
		{
			U32 count = g_wacs0_fail_count0 + g_wacs0_fail_count0 + g_wacs0_fail_count0;
			if(count == 0){
				PWRAPLOG("tc_concurrence_test pass.\n");
			}else{
				PWRAPLOG("tc_concurrence_test failed %d.\n",count);
			} 
		}
		else
		{
			PWRAPLOG("tc_concurrence_test build environment fail.res=%d\n",res);
		}
		return res;
	}
	//-------------------concurrence_test end-------------------------------------

static void high_case(void)
{
        //tc_mux_switch_test();
        int i=0;
        for(i=0;i < 10000; i++)
		{
			tc_wrap_init_test();
			tc_wrap_access_test();
			tc_status_update_test();
			tc_dual_io_test();
			//tc_reg_rw_test();
			
			tc_soft_reset_test();
			tc_high_pri_test();
			tc_spi_encryption_test();
			//tc_pwrap_int_update_test();


			PWRAPLOG("high case run (%d) times\n",  i);

		}

}

extern	 inline void pwrap_dump_all_register(void);
	static S32 mt_pwrap_dvt(U32 nbr)
	{
		S32 ret = 0;
		switch(nbr){
			case INIT:
				tc_wrap_init_test();
				break;
			case ACCESS:
				tc_wrap_access_test();
				break;
			case STATUS_UPDATE:
				tc_status_update_test();
				break;
			case DUAL_IO:
				PWRAPLOG("fwq log\n" );
				tc_dual_io_test();
				pwrap_dump_all_register();
				break;
			case REG_RW:
				tc_reg_rw_test();
				break;
			case MUX_SWITCH:
				tc_mux_switch_test();
				break;
			case SOFT_RESET: //6
				tc_soft_reset_test();
				break;
			case HIGH_PRI:
				tc_high_pri_test();
				break;
			case ENCRYPTION:
				tc_spi_encryption_test();
				break;
			case WDT:
				tc_wdt_test();
				break;
			case INTERRUPT:
				tc_int_test();
				break;
			case CONCURRENCE:
				tc_concurrence_test();
				break;
			case INT_UPDATE: //12
				tc_pwrap_int_update_test();
				break;
			case High_test_case:
				high_case();
				break;
			default:
				PWRAPERR("unsupport(%d)\n", nbr);
				ret = -1;
				break;
		}

		return ret;
	}

	static ssize_t mt_pwrap_dvt_show(struct device_driver *driver, char *buf)
	{

		//printk(KERN_ERR "[WRAP]""driver need registered!!");
		return snprintf(buf, PAGE_SIZE, "%s\n","[WRAP]driver need registered!! ");
	}

	static ssize_t mt_pwrap_dvt_store(struct device_driver *driver, const char *buf,
			size_t count)
	{
		U32 nbr;
		if(1 == sscanf(buf, "%d", &nbr))
			mt_pwrap_dvt(nbr);
		return count;
	}

	DRIVER_ATTR(dvt, 0664, mt_pwrap_dvt_show, mt_pwrap_dvt_store);
#define VERSION     "LDVT"

	static int __init mt_pwrap_init_dvt(void)
	{
		S32 ret = 0; 
		PWRAPLOG("fwq HAL init dvt: version %s\n", VERSION);
		mt_wrp = get_mt_pmic_wrap_drv();	

		ret = driver_create_file(&mt_wrp->driver, &driver_attr_dvt);
		if (ret) {
			printk(KERN_ERR "[WRAP]""Fail to create mt_wrp dvt sysfs files");
		}

		//free_irq(MT_PMIC_WRAP_IRQ_ID, NULL);
		/*
		ret = request_irq(163+32, mt_pwrap_dvt_irq, IRQF_TRIGGER_HIGH, "pwrap_dvt",NULL);
		if (ret) {
			PWRAPERR("register IRQ failed (%d)\n", ret);
			return ret;
		}
*/
		return ret;
	}
	//arch_initcall(mt_pwrap_init);
	//device_initcall(mt_pwrap_init_dvt);
	module_init(mt_pwrap_init_dvt);
