#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/utsname.h>
#include <asm/uaccess.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/notifier.h>

#include <linux/platform_device.h>
//#include "mt_hotplug_strategy.h"
#include "cpufreq_governor.h"
#include "mt_cpufreq.h"

extern void hp_based_cpu_num(enum hps_base_type_e type, int num);

/*--------------DEFAULT SETTING-------------------*/

#define TARGET_CORE (3)
#define TARGET_FREQ (1040000)

/*-----------------------------------------------*/

int perfmgr_get_target_core(void)
{
	return TARGET_CORE;
}

int perfmgr_get_target_freq(void)
{
	return TARGET_FREQ;
}

void perfmgr_boost(int enable, int core, int freq)
{
	if (enable) {
		/* hps */
		hp_based_cpu_num(BASE_TOUCH_BOOST, core);
		mt_cpufreq_set_min_freq(MT_CPU_DVFS_LITTLE, freq);
	} else {
		/* hps */
		hp_based_cpu_num(BASE_TOUCH_BOOST, 1);
		mt_cpufreq_set_min_freq(MT_CPU_DVFS_LITTLE, 0);
	}
}

