/*
 * LGE charging scenario Header file.
 *
 * Copyright (C) 2013 LG Electronics
 * sangwoo <sangwoo2.park@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LGE_POWER_SYSFS_H_
#define __LGE_POWER_SYSFS_H_

struct power_sysfs_array {
	const char *group;
	const char *user_node;
	const char *kernel_node;
};

#endif
