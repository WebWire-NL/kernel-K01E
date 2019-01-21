/* arch/arm/mach-msm/include/mach/board_asustek.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2012, LGE Inc.
 * Copyright (c) 2014, ASUSTek Computer Inc.
 * Author: Paris Yeh <paris_yeh@asus.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_BOARD_ASUSTEK_H
#define __ASM_ARCH_MSM_BOARD_ASUSTEK_H

#ifdef CONFIG_ANDROID_PERSISTENT_RAM
#define ASUSTEK_PERSISTENT_RAM_SIZE	(SZ_1M)
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define ASUSTEK_RAM_CONSOLE_SIZE	(124 * SZ_1K * 2)
#endif

typedef enum {
	TP_TYPE_INVALID = -1,
	TP_TYPE_A = 0,
	TP_TYPE_B = 1,
	TP_TYPE_C = 2,
	TP_TYPE_D = 3,
	TP_TYPE_MAX
} tp_type;

typedef enum {
	LCD_TYPE_INVALID = -1,
	LCD_TYPE_A = 0,
	LCD_TYPE_B = 1,
	LCD_TYPE_C = 2,
	LCD_TYPE_D = 3,
	LCD_TYPE_MAX
} lcd_type;

typedef enum {
	HW_REV_INVALID = -1,
	HW_REV_A = 0,
	HW_REV_B = 1,
	HW_REV_C = 2,
	HW_REV_D = 3,
	HW_REV_MAX
} hw_rev;

typedef enum {
	TP_IC_TYPE_INVALID = -1,
	TP_IC_TYPE_A = 0,
	TP_IC_TYPE_B = 1,
	TP_IC_TYPE_MAX
} tp_ic_type;

struct asustek_pcbid_platform_data {
	const char *UUID;
	struct resource *resource_leopardcat;
	u32 nr_resource_leopardcat;
};

void __init asustek_reserve(void);

#ifdef CONFIG_ANDROID_PERSISTENT_RAM
void __init asustek_add_persistent_ram(void);
#else
static inline void __init asustek_add_persistent_ram(void)
{
	/* empty */
}
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
void __init asustek_add_ramconsole_devices(void);
#else
static inline void __init asustek_add_ramconsole_devices(void)
{
	/* empty */
}
#endif

void __init asustek_add_pcbid_devices(void);

tp_ic_type asustek_get_tp_type(void);

lcd_type asustek_get_lcd_type(void);

hw_rev asustek_get_hw_rev(void);
#endif /* __ASM_ARCH_MSM_BOARD_ASUSTEK_H */
