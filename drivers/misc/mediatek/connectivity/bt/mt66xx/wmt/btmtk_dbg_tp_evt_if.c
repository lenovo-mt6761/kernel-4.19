/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifdef CONFIG_WLAN_DRV_BUILD_IN
/* TB301FU (sjllls): with the BT driver built-in the bt_evt tracepoint is
 * compiled out (it corrupts vmlinux tracefs event registration); keep the
 * bt_dbg_tp_evt() interface as a no-op. */
#include <linux/printk.h>

void bt_dbg_tp_evt(unsigned int pkt_action,
		unsigned int parameter,
		unsigned int data_len,
		char *data)
{
	pr_debug("bt_dbg_tp_evt(%u,%u,%u) dropped: tracepoint disabled\n",
		pkt_action, parameter, data_len);
}
#else
#define CREATE_TRACE_POINTS
#include "btmtk_dbg_tp_evt.h"
#include "btmtk_dbg_tp_evt_if.h"

void bt_dbg_tp_evt(unsigned int pkt_action,
		unsigned int parameter,
		unsigned int data_len,
		char *data)
{
	//struct timespec64 kerneltime;
	//ktime_get_ts64(&kerneltime);
	trace_bt_evt(pkt_action, parameter, data_len, data);
}
#endif

