#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/aee.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_spm_idle.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_power_gs.h>

#include <mach/mt_pmic_wrap.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>

#define gs_read(addr) (*(volatile u32 *)(addr))

struct proc_dir_entry *mt_power_gs_dir = NULL;

static kal_uint16 gs6328_pmic_read(kal_uint16 reg)
{
	kal_uint32 ret = 0;
	kal_uint32 reg_val = 0;

	ret = pmic_read_interface(reg, &reg_val, 0xFFFF, 0x0);

	return (kal_uint16)reg_val;
}

void mt_power_gs_diff_output(unsigned int val1, unsigned int val2)
{
	int i = 0;
	unsigned int diff = val1 ^ val2;

	while (diff != 0) {
		if ((diff % 2) != 0)
			pr_err("%d ", i);

		diff /= 2;
		i++;
	}

	pr_err("\n");
}

void mt_power_gs_compare(char *scenario, \
			 unsigned int *pmic_6328_gs, unsigned int pmic_6328_gs_len)
{
	unsigned int i, val1, val2;

	// 6328
	for (i = 0; i < pmic_6328_gs_len; i += 3) {
		aee_sram_printk("%d\n", i);
		val1 = gs6328_pmic_read(pmic_6328_gs[i]) & pmic_6328_gs[i + 1];
		val2 = pmic_6328_gs[i + 2] & pmic_6328_gs[i + 1];

		if (val1 != val2) {
			pr_err("%s - 6328 - 0x%x - 0x%x - 0x%x - 0x%x - ",
			       scenario, pmic_6328_gs[i], gs6328_pmic_read(pmic_6328_gs[i]),
			       pmic_6328_gs[i + 1], pmic_6328_gs[i + 2]);
			mt_power_gs_diff_output(val1, val2);
		}
	}

//	mt_power_gs_compare_pll();
}
EXPORT_SYMBOL(mt_power_gs_compare);

static void __exit mt_power_gs_exit(void)
{
	//return 0;
}

static int __init mt_power_gs_init(void)
{
	mt_power_gs_dir = proc_mkdir("mt_power_gs", NULL);

	if (!mt_power_gs_dir)
		pr_err("mkdir /proc/mt_power_gs failed\n");

	return 0;
}

module_init(mt_power_gs_init);
module_exit(mt_power_gs_exit);

MODULE_DESCRIPTION("MT Low Power Golden Setting");
