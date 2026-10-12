/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef _LCD_BIAS_LP6280S_H_
#define _LCD_BIAS_LP6280S_H_

#ifndef BUILD_LK
struct lp6280s_setting_table {
	unsigned char cmd;
	unsigned char data;
};

extern int lp6280s_read_byte(unsigned char cmd, unsigned char *returnData);
extern int lp6280s_write_byte(unsigned char cmd, unsigned char writeData);
#endif

#endif


