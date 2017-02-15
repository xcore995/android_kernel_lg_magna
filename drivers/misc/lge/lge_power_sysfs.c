/* Copyright (c) 2013-2014, LG Eletronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#endif

#include <mach/lge_power_sysfs.h>

#define MODULE_NAME "power-sysfs"

#define PWR_SYSFS_GROUPS_NUM 9
#define PWR_SYSFS_MANDATORY_MAX_NUM 5

/* Set sysfs node for non-using DT */
__weak struct power_sysfs_array lge_power_sysfs_path[] = {
	/* ADC */
	{"adc", "batt_therm", "/sys/class/power_supply/battery/temp"},
	{"adc", "thermal", "/sys/class/thermal/"},
	{"adc", "usb_id", "/sys/devices/platform/battery/LGBM_AtCmdUsbidadc"},
	{"adc", "xo_therm", "NULL"},
	/* Battery */
	{"battery", "capacity", "/sys/class/power_supply/battery/capacity"},
	{"battery", "health", "/sys/class/power_supply/battery/health"},
	{"battery", "present", "/sys/class/power_supply/battery/present"},
	{"battery", "pseudo_batt", "/sys/class/power_supply/battery/pseudo_batt"},
	{"battery", "temp", "/sys/class/power_supply/battery/temp"},
	{"battery", "valid_batt_id", "/sys/class/power_supply/battery/valid_batt_id"},
	{"battery", "voltage_now", "/sys/class/power_supply/battery/voltage_now"},
	{"battery", "status", "/sys/class/power_supply/battery/status"},
	/* Charger */
	{"charger", "ac_online", "/sys/class/power_supply/ac/online"},
	{"charger", "usb_online", "/sys/class/power_supply/usb/online"},
	{"charger", "present", "/sys/class/power_supply/ac/present"},
	{"charger", "time_out", "/sys/class/power_supply/battery/safety_timer"},
	{"charger", "wlc_online", "NULL"},
	/* LCD */
	{"lcd", "brightness", "/sys/class/leds/lcd-backlight/brightness"},
	{"lcd", "max_brightness", "/sys/class/leds/lcd-backlight/max_brightness"},
	/* Key LED */
	{"key_led", "red_brightness", "/sys/class/leds/red/brightness"},
	{"key_led", "green_brightness", "NULL"},
	{"key_led", "blue_brightness", "NULL"},
	{"key_led", "button_backlight1_brightness", "NULL"},
	/* CPU */
	/* GPU */
	/* Platform */
	{"platform", "pcb_rev", "NULL"},	/* should be defined */
	/* TestMode */
	{"testmode", "temp", "/sys/devices/platform/battery/LGBM_AtCmdBatmp"},
	{"testmode", "chcomp", "/sys/devices/platform/battery/LGBM_AtCmdChcomp"},
	{"testmode", "charge", "/sys/devices/platform/battery/LGBM_AtCmdCharge"},
	{"testmode", "chgmodeoff", "/sys/devices/platform/battery/LGBM_AtCmdChargingModeOff"},
	{"testmode", "battexist", "/sys/class/power_supply/battery/present"},
};

static const struct power_sysfs_array mandatory_paths[] = {
	/* Battery */
	{"battery", "voltage_now"},
	/* TestMode */
	{"testmode", "charge"},
	{"testmode", "chcomp"},
};

static const char *group_names[PWR_SYSFS_GROUPS_NUM] = {
	"adc",
	"battery",
	"charger",
	"lcd",
	"key_led",
	"cpu",
	"gpu",
	"platform",
	"testmode"
};

static struct power_sysfs_array *arr;
static int arr_cnt;

static int check_mandatory_path(int arr_num)
{
	int mandatory_num;
	int array_size = ARRAY_SIZE(mandatory_paths);

	for (mandatory_num = 0; mandatory_num < array_size; mandatory_num++) {
		if (!strcmp(arr[arr_num].group, mandatory_paths[mandatory_num].group))
			if (!strcmp(arr[arr_num].user_node, mandatory_paths[mandatory_num].user_node))
				return 1;
	}
	return 0;
}

#ifdef CONFIG_OF
static int power_sysfs_parse_dt(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int arr_num, ret, i;

	arr_cnt = of_property_count_strings(node, "sysfs,node") / 3;
	if (arr_cnt > 0)
		pr_info("%s : Total sysfs node is %d\n", __func__, arr_cnt);
	else {
		pr_err("%s : ERROR sysfs node isn't exist\n", __func__);
		return 0;
	}

	arr = kzalloc(arr_cnt * sizeof(struct power_sysfs_array),
			GFP_KERNEL);
	if (arr != NULL) {
		for (arr_num = 0, i = 0; arr_num < arr_cnt; arr_num++) {
			ret = of_property_read_string_index(node, "sysfs,node",
					i++, &arr[arr_num].group);
			if (ret) {
				pr_err("%s : ERROR get %ith group\n", __func__, arr_num);
				goto err_get_array;
			}
			ret = of_property_read_string_index(node, "sysfs,node",
					i++, &arr[arr_num].user_node);
			if (ret) {
				pr_err("%s : ERROR get %ith user_node\n", __func__, arr_num);
				goto err_get_array;
			}
			ret = of_property_read_string_index(node, "sysfs,node",
					i++, &arr[arr_num].kernel_node);
			if (ret) {
				pr_err("%s : ERROR get %ith kernel_node\n", __func__, arr_num);
				goto err_get_array;
			} else if (check_mandatory_path(arr_num)) {
				if (!strcmp(arr[arr_num].kernel_node, "NULL")) {
					pr_err("%s : ERROR get mandatory path %s\n", __func__,
							arr[arr_num].user_node);
					goto err_get_array;
				}
				pr_info("%s : %s path is mandatory \n", __func__,
						arr[arr_num].user_node);
			}
		}
	} else {
		pr_err("%s : ERROR get sysfs array\n", __func__);
		return -1;
	}
/* debug */
	for (arr_num = 0; arr_num < arr_cnt; arr_num++)
		pr_err("%s : get %dth node is %s, %s, %s\n", __func__, arr_num,
				arr[arr_num].group, arr[arr_num].user_node,
				arr[arr_num].kernel_node);

	return 0;

err_get_array:
	kzfree(arr);

	return -1;
}
#else
static int power_sysfs_parse_dt(struct platform_device *pdev)
{
	pr_err("%s : ERROR CONFIG_OF isn't set\n", __func__);
	return -1;
}
#endif
static int power_sysfs_make_path(void)
{
	int arr_num, group_num;
	struct proc_dir_entry *p, *temp_p;
	struct proc_dir_entry *groups_p[PWR_SYSFS_GROUPS_NUM];

	/* Set Power Sysfs root directory */
	p = proc_mkdir("lge_power", NULL);
	if (p == NULL) {
		pr_err("%s : ERROR make root sysfs \n", __func__);
		return -ENOMEM;
	}

	/* Set Power Sysfs group directory */
	if (groups_p != NULL) {
		for (group_num = 0; group_num < PWR_SYSFS_GROUPS_NUM; group_num++) {
			groups_p[group_num] = proc_mkdir(group_names[group_num], p);
			if (groups_p[group_num] == NULL) {
				pr_err("%s : ERROR make %s group \n", __func__,
						group_names[group_num]);
				return -ENOMEM;
			}
		}
	} else {
		pr_err("%s : ERROR make groups pointer \n", __func__);
		return -ENOMEM;
	}

	/* Set Power Sysfs Path */
	for (arr_num = 0; arr_num < arr_cnt; arr_num++) {
		for (group_num = 0; group_num < PWR_SYSFS_GROUPS_NUM; group_num++)
			if (!strcmp(arr[arr_num].group, group_names[group_num]))
				break;

		if (!strcmp(arr[arr_num].kernel_node, "NULL")) {
			pr_info("%s : %s user node didn't have kernel node \n",
					__func__, arr[arr_num].user_node);
			continue;
		} else {
			temp_p = proc_symlink(arr[arr_num].user_node, groups_p[group_num],
					arr[arr_num].kernel_node);
			if (temp_p == NULL) {
				pr_err("%s : ERROR make %ith sysfs path(%s, %s, %s)\n",
						__func__, arr_num, arr[arr_num].group,
						arr[arr_num].kernel_node, arr[arr_num].user_node);
				return -ENOMEM;
			}
		}

	}

	return 0;
}

static int read_sysfs_path(void)
{
	int arr_num;

	arr_cnt = ARRAY_SIZE(lge_power_sysfs_path);

	arr = kzalloc(arr_cnt * sizeof(struct power_sysfs_array),
			GFP_KERNEL);

	if (arr != NULL) {
		for (arr_num = 0; arr_num < arr_cnt; arr_num++) {
			arr[arr_num].group = lge_power_sysfs_path[arr_num].group;
			arr[arr_num].user_node = lge_power_sysfs_path[arr_num].user_node;
			arr[arr_num].kernel_node = lge_power_sysfs_path[arr_num].kernel_node;
			if (arr[arr_num].kernel_node == NULL) {
				pr_err("%s : ERROR get %ith kernel_node\n", __func__, arr_num);
				goto err_get_array;
			} else if (check_mandatory_path(arr_num)) {
				if (!strcmp(arr[arr_num].kernel_node, "NULL")) {
					pr_err("%s : ERROR get mandatory path %s\n", __func__,
							arr[arr_num].user_node);
					goto err_get_array;
				}
				pr_info("%s : %s path is mandatory \n", __func__,
						arr[arr_num].user_node);
			}
		}
	} else {
		pr_err("%s : ERROR get sysfs array\n", __func__);
		return -1;
	}

/* debug */
	for (arr_num = 0; arr_num < arr_cnt; arr_num++)
		pr_err("%s : get %dth node is %s, %s, %s\n", __func__, arr_num,
				arr[arr_num].group, arr[arr_num].user_node,
				arr[arr_num].kernel_node);
	return 0;

err_get_array:
	kzfree(arr);

	return -1;

}

static int power_sysfs_probe(struct platform_device *pdev)
{
	int ret;

	if (pdev->dev.of_node) {
		ret = power_sysfs_parse_dt(pdev);
		if (ret < 0) {
			pr_err("%s : ERROR Parsing DT\n", __func__);
			return ret;
		}
	} else {
		ret = read_sysfs_path();
		if (ret < 0) {
			pr_err("%s : ERROR Parsing data\n", __func__);
			return ret;
		}
	}

	ret = power_sysfs_make_path();
	if (ret != 0) {
		pr_err("%s : ERROR make sysfs path\n", __func__);
		return ret;
	}

	pr_info("%s : Success Power sysfs Init\n", __func__);

	return ret;
}

static int power_sysfs_remove(struct platform_device *pdev)
{
	if (arr != NULL)
		kzfree(arr);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id power_sysfs_match_table[] = {
	{ .compatible = "lge,power-sysfs" },
	{}
};
#endif

static struct platform_driver power_sysfs_driver = {
	.probe = power_sysfs_probe,
	.remove = power_sysfs_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = power_sysfs_match_table,
#endif
	},
};

static int __init power_sysfs_init(void)
{
	return platform_driver_register(&power_sysfs_driver);
}

static void power_sysfs_exit(void)
{
	platform_driver_unregister(&power_sysfs_driver);
}

#ifdef CONFIG_OF
static struct platform_device power_sysfs_platform_device = {
	.name   = "power-sysfs",
	.id = 0,
};

static int __init power_sysfs_device_init(void)
{
	pr_err("%s st\n", __func__);
	return platform_device_register(&power_sysfs_platform_device);
}

static void power_sysfs_device_exit(void)
{
	platform_device_unregister(&power_sysfs_platform_device);
}
#endif

late_initcall(power_sysfs_init);
module_exit(power_sysfs_exit);
#ifdef CONFIG_OF
late_initcall(power_sysfs_device_init);
module_exit(power_sysfs_device_exit);
#endif
MODULE_DESCRIPTION("LGE Power sysfs driver");
MODULE_LICENSE("GPL v2");
