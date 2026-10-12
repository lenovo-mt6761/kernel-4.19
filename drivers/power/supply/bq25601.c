// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
#include <mt-plat/mtk_boot.h>
#include <mt-plat/upmu_common.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "bq25601.h"
#include "charger_class.h"
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#include <linux/hardware_info.h>

/**********************************************************
 *
 *   [I2C Slave Setting]
 *
 *********************************************************/

/*bq25601 REG04 VREG[7:3]*/
const unsigned int VREG_TABLE[] = {
	3856000, 3888000, 3920000, 3952000,
	3984000, 4016000, 4048000, 4080000,
	4112000, 4144000, 4176000, 4208000,
	4240000, 4272000, 4304000, 4352000, //4352000 is special value
	4368000, 4400000, 4432000, 4464000,
	4496000, 4528000, 4560000, 4592000,
	4624000

};

/*SGM41511 REG02 ICHG[5:0]*/
const unsigned int SGM41513_ICHG_TABLE[] = {
	      0,    5000,   10000,   15000,   20000,
	  25000,   30000,   35000,   40000,   50000,
	  60000,   70000,   80000,   90000,  100000,
	 110000,  130000,  150000,  170000,  190000,
	 210000,  230000,  250000,  270000,  300000,
	 330000,  360000,  390000,  420000,  450000,
	 480000,  510000,  540000,  600000,  660000,
	 720000,  780000,  840000,  900000,  960000,
	1020000, 1080000, 1140000, 1200000, 1260000,
	1320000, 1380000, 1440000, 1500000, 1620000,
	1740000, 1860000, 1980000, 2100000, 2220000,
	2340000, 2460000, 2580000, 2700000, 2820000,
	2940000, 3000000
};

/*SY6974 REG02 ICHG[5:0]*/
const unsigned int SY6974_ICHG_TABLE[] = {
	      0,   60000,  120000,  180000,  240000,
	 300000,  360000,  420000,  480000,  540000,
	 600000,  660000,  720000,  780000,  840000,
	 900000,  960000, 1020000, 1080000, 1140000,
	1200000, 1260000, 1320000, 1380000, 1440000,
	1500000, 1560000, 1620000, 1680000, 1740000,
	1800000, 1860000, 1920000, 1980000, 2040000,
	2100000, 2160000, 2220000, 2280000, 2340000,
	2400000, 2460000, 2520000, 2580000, 2640000,
	2700000, 2760000, 2820000, 2880000, 2940000,
	3000000
};

/*BQ25601 REG00 IINDPM[4:0]*/
const unsigned int INPUT_LIMIT_TABLE[] = {
	 100000,  200000,  300000,  400000,
	 500000,  600000,  700000,  800000,
	 900000, 1000000, 1100000, 1200000,
	1300000, 1400000, 1500000, 1600000,
	1700000, 1800000, 1900000, 2000000,
	2100000, 2200000, 2300000, 2400000,
	2500000, 2600000, 2700000, 2800000,
	2900000, 3000000, 3100000, 3200000
};

//BQ25601 REG06 VINDPM[3:0]
const unsigned int VINDPM_TABLE[] = {
	3900000, 4000000, 4100000, 4200000,
	4300000, 4400000, 4500000, 4600000,
	4700000, 4800000, 4900000, 5000000,
	5100000, 5200000, 5300000, 5400000
};

struct bq25601_info {
	struct charger_device *chg_dev;
	struct charger_properties chg_props;
	struct device *dev;
	const char *chg_dev_name;
	const char *eint_name;
	int irq;
	struct regulator_dev *otg_rdev;
	bool is_charging_enable;
};

DEFINE_MUTEX(g_input_current_mutex);
static struct i2c_client *new_client;
static const struct i2c_device_id bq25601_i2c_id[] = { {"bq25601", 0}, {} };

static int disable_charger = 0;
extern int factory_charger_disable;
int g_charging_status = POWER_SUPPLY_STATUS_UNKNOWN;
static int enable_charge = 1;
static int enable_supply = 0;
static int ship_mode  = 0;
struct bq25601_info *g_bq25601_info = NULL;

extern struct hardware_info current_chargerIC_info;

/**********************************************************
 *
 *   [Global Variable]
 *
 *********************************************************/
static DEFINE_MUTEX(bq25601_i2c_access);
static DEFINE_MUTEX(bq25601_access_lock);

static int g_sy6974_hw_exist = 0;

/**********************************************************
 *
 *   [I2C Function For Read/Write bq25601]
 *
 *********************************************************/

static int bq25601_read_byte(u8 cmd)
{
        s32 ret;

        mutex_lock(&bq25601_i2c_access);
        ret = i2c_smbus_read_byte_data(new_client, cmd);
        mutex_unlock(&bq25601_i2c_access);

        return ret;
}

static int bq25601_write_byte(u8 cmd, u8 val)
{
        s32 ret;

	mutex_lock(&bq25601_i2c_access);
        ret = i2c_smbus_write_byte_data(new_client, cmd, val);
	mutex_unlock(&bq25601_i2c_access);

        return ret;
}

/**********************************************************
 *
 *   [Read / Write Function]
 *
 *********************************************************/
static int bq25601_read_interface(u8 reg, u8 *val, u8 mask, u8 shift)
{
        int ret = 0;

        ret = bq25601_read_byte(reg);
        if (ret < 0)
                return ret;

        ret &= (mask << shift);
        *val = (ret >> shift);

        return 0;
}

static int bq25601_write_interface(u8 reg, u8 val, u8 mask, u8 shift)
{
        int ret = 0;
        u8 read_reg = 0;

        ret = bq25601_read_byte(reg);
        if (ret < 0)
                return ret;

        read_reg = ret;
        ret &= ~(mask << shift);
        ret |= (val << shift);

        return bq25601_write_byte(reg, ret);
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
/* CON0---------------------------------------------------- */
void bq25601_set_en_hiz(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_EN_HIZ_MASK),
				       (unsigned char) (CON0_EN_HIZ_SHIFT)
				      );
}

void bq25601_set_iinlim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON0),
				       (unsigned char) (val),
				       (unsigned char) (CON0_IINLIM_MASK),
				       (unsigned char) (CON0_IINLIM_SHIFT)
				      );
}

void bq25601_set_stat_ctrl(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON0),
				   (unsigned char) (val),
				   (unsigned char) (CON0_STAT_IMON_CTRL_MASK),
				   (unsigned char) (CON0_STAT_IMON_CTRL_SHIFT)
				   );
}

/* CON1---------------------------------------------------- */

void bq25601_set_reg_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON11),
				       (unsigned char) (val),
				       (unsigned char) (CON11_REG_RST_MASK),
				       (unsigned char) (CON11_REG_RST_SHIFT)
				      );
}

void bq25601_set_pfm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_PFM_MASK),
				       (unsigned char) (CON1_PFM_SHIFT)
				      );
}

void bq25601_set_wdt_rst(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_WDT_RST_MASK),
				       (unsigned char) (CON1_WDT_RST_SHIFT)
				      );
}

void bq25601_set_otg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_OTG_CONFIG_MASK),
				       (unsigned char) (CON1_OTG_CONFIG_SHIFT)
				      );
}

unsigned int bq25601_get_otg_config(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq25601_read_interface((unsigned char) (bq25601_CON1),
				     (&val),
				     (unsigned char) (CON1_OTG_CONFIG_MASK),
				     (unsigned char) (CON1_OTG_CONFIG_SHIFT)
				    );
	return val;
}

void bq25601_set_chg_config(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_CHG_CONFIG_MASK),
				       (unsigned char) (CON1_CHG_CONFIG_SHIFT)
				      );
}


void bq25601_set_sys_min(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_SYS_MIN_MASK),
				       (unsigned char) (CON1_SYS_MIN_SHIFT)
				      );
}

void bq25601_set_batlowv(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON1),
				       (unsigned char) (val),
				       (unsigned char) (CON1_MIN_VBAT_SEL_MASK),
				       (unsigned char) (CON1_MIN_VBAT_SEL_SHIFT)
				      );
}

/* CON2---------------------------------------------------- */
void bq25601_set_rdson(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_Q1_FULLON_MASK),
				       (unsigned char) (CON2_Q1_FULLON_SHIFT)
				      );
}

void bq25601_set_boost_lim(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_BOOST_LIM_MASK),
				       (unsigned char) (CON2_BOOST_LIM_SHIFT)
				      );
}

void bq25601_set_ichg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON2),
				       (unsigned char) (val),
				       (unsigned char) (CON2_ICHG_MASK),
				       (unsigned char) (CON2_ICHG_SHIFT)
				      );
}

static int bq25601_get_ichg(u8 *val)
{
        return bq25601_read_interface((u8) (bq25601_CON2),
                        (val),
                        (u8) (CON2_ICHG_MASK),
                        (u8) (CON2_ICHG_SHIFT)
                        );
}

/* CON3---------------------------------------------------- */

void bq25601_set_iprechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_IPRECHG_MASK),
				       (unsigned char) (CON3_IPRECHG_SHIFT)
				      );
}

void bq25601_set_iterm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON3),
				       (unsigned char) (val),
				       (unsigned char) (CON3_ITERM_MASK),
				       (unsigned char) (CON3_ITERM_SHIFT)
				      );
}

/* CON4---------------------------------------------------- */

void bq25601_set_vreg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VREG_MASK),
				       (unsigned char) (CON4_VREG_SHIFT)
				      );
}

void bq25601_set_topoff_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_TOPOFF_TIMER_MASK),
				       (unsigned char) (CON4_TOPOFF_TIMER_SHIFT)
				      );

}


void bq25601_set_vrechg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON4),
				       (unsigned char) (val),
				       (unsigned char) (CON4_VRECHG_MASK),
				       (unsigned char) (CON4_VRECHG_SHIFT)
				      );
}

/* CON5---------------------------------------------------- */

void bq25601_set_en_term(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TERM_MASK),
				       (unsigned char) (CON5_EN_TERM_SHIFT)
				      );
}



void bq25601_set_watchdog(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_WATCHDOG_MASK),
				       (unsigned char) (CON5_WATCHDOG_SHIFT)
				      );
}

void bq25601_set_en_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_EN_TIMER_MASK),
				       (unsigned char) (CON5_EN_TIMER_SHIFT)
				      );
}

void bq25601_set_chg_timer(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON5),
				       (unsigned char) (val),
				       (unsigned char) (CON5_CHG_TIMER_MASK),
				       (unsigned char) (CON5_CHG_TIMER_SHIFT)
				      );
}

/* CON6---------------------------------------------------- */

void bq25601_set_treg(unsigned int val)
{
#ifdef FIXME
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BOOSTV_MASK),
				       (unsigned char) (CON6_BOOSTV_SHIFT)
				      );
#endif
}

void bq25601_set_vindpm(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_VINDPM_MASK),
				       (unsigned char) (CON6_VINDPM_SHIFT)
				      );
}


void bq25601_set_ovp(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_OVP_MASK),
				       (unsigned char) (CON6_OVP_SHIFT)
				      );

}

void bq25601_set_boostv(unsigned int val)
{

	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON6),
				       (unsigned char) (val),
				       (unsigned char) (CON6_BOOSTV_MASK),
				       (unsigned char) (CON6_BOOSTV_SHIFT)
				      );
}



/* CON7---------------------------------------------------- */

void bq25601_set_tmr2x_en(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON7),
					(unsigned char) (val),
					(unsigned char) (CON7_TMR2X_EN_MASK),
					(unsigned char) (CON7_TMR2X_EN_SHIFT)
					);
}

void bq25601_set_batfet_disable(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON7),
				(unsigned char) (val),
				(unsigned char) (CON7_BATFET_Disable_MASK),
				(unsigned char) (CON7_BATFET_Disable_SHIFT)
				);
}


void bq25601_set_batfet_delay(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON7),
				       (unsigned char) (val),
				       (unsigned char) (CON7_BATFET_DLY_MASK),
				       (unsigned char) (CON7_BATFET_DLY_SHIFT)
				      );
}

void bq25601_set_batfet_reset_enable(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON7),
				(unsigned char) (val),
				(unsigned char) (CON7_BATFET_RST_EN_MASK),
				(unsigned char) (CON7_BATFET_RST_EN_SHIFT)
				);
}

static int bq25601_enable_shipping_mode(bool en)
{
	int ret;

	if (en) {
		bq25601_dbg("%s:%d", __func__, en);
		ret = bq25601_write_interface((unsigned char) (bq25601_CON7), 0x20, 0x20, 0);
		if (ret < 0) {
			bq25601_dbg("[%s] set batfet_dis failed:%d", __func__, ret);
			return ret;
		}
		ret = bq25601_write_interface((unsigned char) (bq25601_CON7), 0x08, 0x08, 0);
		if (ret < 0) {
			bq25601_dbg("[%s] set batfet_dly failed:%d", __func__, ret);
			return ret;
		}
	} else {
		/* do nothing, should never happen */
	}
	return 0;
}
/* CON8---------------------------------------------------- */

unsigned int bq25601_get_system_status(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq25601_read_interface((unsigned char) (bq25601_CON8),
				     (&val), (unsigned char) (0xFF),
				     (unsigned char) (0x0)
				    );
	return val;
}

unsigned int bq25601_get_vbus_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq25601_read_interface((unsigned char) (bq25601_CON8),
				     (&val),
				     (unsigned char) (CON8_VBUS_STAT_MASK),
				     (unsigned char) (CON8_VBUS_STAT_SHIFT)
				    );
	return val;
}

unsigned int bq25601_get_chrg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq25601_read_interface((unsigned char) (bq25601_CON8),
				     (&val),
				     (unsigned char) (CON8_CHRG_STAT_MASK),
				     (unsigned char) (CON8_CHRG_STAT_SHIFT)
				    );
	return val;
}

unsigned int bq25601_get_vsys_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq25601_read_interface((unsigned char) (bq25601_CON8),
				     (&val),
				     (unsigned char) (CON8_VSYS_STAT_MASK),
				     (unsigned char) (CON8_VSYS_STAT_SHIFT)
				    );
	return val;
}

unsigned int bq25601_get_pg_stat(void)
{
	unsigned int ret = 0;
	unsigned char val = 0;

	ret = bq25601_read_interface((unsigned char) (bq25601_CON8),
				     (&val),
				     (unsigned char) (CON8_PG_STAT_MASK),
				     (unsigned char) (CON8_PG_STAT_SHIFT)
				    );
	return val;
}

/*CON10----------------------------------------------------------*/

void bq25601_set_int_mask(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON10),
				       (unsigned char) (val),
				       (unsigned char) (CON10_INT_MASK_MASK),
				       (unsigned char) (CON10_INT_MASK_SHIFT)
				      );
}

/* CON15---------------------------------------------------- */
void bq25601_set_itrickchg(unsigned int val)
{
	unsigned int ret = 0;

	ret = bq25601_write_interface((unsigned char) (bq25601_CON15),
			(unsigned char) (val),
			(unsigned char) (CON15_ITRICKCHG_MASK),
			(unsigned char) (CON15_ITRICKCHG_SHIFT)
			);
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int bq25601_dump_register(struct charger_device *chg_dev)
{
#ifdef BQ25601_DEBUG
	int i = 0;
	u8 val[bq25601_REG_NUM] = {0};

	for (i = 0; i <bq25601_REG_NUM; i++) {
		val[i] = bq25601_read_byte(i);
		if (val[i] < 0) {
			bq25601_dbg("%s, bq25601_read_byte fail\n", __func__);
			return -EIO;
		}
	}

	bq25601_dbg("0:0x%x,1:0x%x,2:0x%x,3:0x%x,4:0x%x,5:0x%x,6:0x%x,7:0x%x,8:0x%x,9:0x%x,10:0x%x,11:0x%x,12:0x%x,13:0x%x,14:0x%x,15:0x%x\n",
			val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15]);
#endif
	return 0;
}


/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int bq25601_hw_component_detect(void)
{
	int ret;
	u8 val;

	ret = bq25601_read_interface(bq25601_CON11, &val, CON11_PN_MASK, CON11_PN_SHIFT);
	if (ret < 0) {
		bq25601_dbg("%s: bq25601_read_interface() fail\n", __func__);
		return ret;
	}

	if (val == SY6974_PART_NUM)
		g_sy6974_hw_exist = 1;

	bq25601_dbg("%s: sy6974_exist:%d, part num=0x%x\n",
			__func__, g_sy6974_hw_exist, val);

	return ret;
}

static int bq25601_get_charging_status(struct charger_device *chg_dev,
				       bool *is_done)
{
	unsigned int status = true;
	unsigned int ret_val;

	ret_val = bq25601_get_chrg_stat();

	if (ret_val == 0x3)
		*is_done = true;
	else
		*is_done = false;

	return status;
}

static void bq25601_update_charging_status(struct charger_device *chg_dev)
{
	struct power_supply *chg_psy = NULL;
	bool charging_status = false;
	static int pre_charging_status = POWER_SUPPLY_STATUS_UNKNOWN;

	bq25601_get_charging_status(chg_dev, &charging_status);

	if (charging_status)
		g_charging_status = POWER_SUPPLY_STATUS_FULL;
	else {
		if (g_bq25601_info) {
			if (g_bq25601_info->is_charging_enable)
				g_charging_status = POWER_SUPPLY_STATUS_CHARGING;
			else
				g_charging_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}

	bq25601_dbg("%s charging_status:%d pre_charging_status:%d\n", __func__, g_charging_status, pre_charging_status);
	if (g_charging_status != pre_charging_status) {
		chg_psy = power_supply_get_by_name("mtk_charger_type");
		if (chg_psy) {
			power_supply_changed(chg_psy);
		}
		pre_charging_status = g_charging_status;
	} else {
		bq25601_dbg("%s charging_status and pre_charging_status is same, ignore update\n", __func__);
	}
}

static int bq25601_enable_hz(struct charger_device *chg_dev, bool en)
{
	int ret = 0;

	bq25601_set_en_hiz(en);
	bq25601_dbg("%s hiz mode \n", en?"enable":"disable");

	return ret;
}
static int bq25601_enable(struct charger_device *chg_dev,
				   bool en)
{
	int status = 0;
	//static bool pre_charging_status;
	if (disable_charger == 1 || factory_charger_disable == 1 || enable_charge  == 0)
		en = 0;

	/*if (pre_charging_status != en) {*/
	g_bq25601_info->is_charging_enable = en;
	//pre_charging_status = en;
	bq25601_dbg("charging enable state : %d\n", en);

	if (en) {
		/* bq25601_write_interface(bq25601_CON3, 0x1, 0x1, 4); */
		/* enable charging */
		if(enable_supply == 0)
		{
			bq25601_set_en_hiz(0x0);
		}
		bq25601_set_chg_config(en);
	} else {
		/* bq25601_write_interface(bq25601_CON3, 0x0, 0x1, 4); */
		/* disable charging */
		bq25601_set_chg_config(en);
		bq25601_dbg("[charging_enable] under test mode: disable charging\n");
		if (factory_charger_disable == 1 || enable_supply == 1) {
			bq25601_set_en_hiz(0x1);
                }
	}
	/*} else {
		bq25601_dbg("[%s] pre-status and current status is same \n", __func__);
	}*/

	bq25601_update_charging_status(chg_dev);
	return status;
}

static int bq25601_get_charging_current(struct charger_device *chg_dev,
			       u32 *ichg)
{
        int ret = 0;
        u8 reg_val = 0;

        ret = bq25601_get_ichg(&reg_val);
        if (ret < 0)
                return ret;

	if (g_sy6974_hw_exist)
		*ichg = SY6974_ICHG_TABLE[reg_val];
	else
		*ichg = SGM41513_ICHG_TABLE[reg_val];
        bq25601_dbg("%s, reg_val:0x%x, ichg:%duA\n", __func__, reg_val, *ichg);

        return ret;
}

static int convert_to_reg(const u32 *table, int table_size, u32 raw_val, u8 *reg_val)
{
        int i;

        if(raw_val > table[table_size - 1]) {
                *reg_val = (table_size - 1);
                bq25601_dbg("raw_val = %d, out of [%d-%d], default use %d\n",
                                raw_val, table[0], table[table_size - 1], table[table_size - 1]);
                return -1;
        }

        if(raw_val < table[0]) {
                *reg_val = 0;
                bq25601_dbg("raw_val = %d, out of [%d-%d], default use %d\n",
                                raw_val, table[0], table[table_size - 1], table[0]);
                return -1;
        }

        for (i = 0; i < table_size; i++) {
                if (table[i] > raw_val) {
                        *reg_val = i-1;
                        return 0;
                }

                if (table[i] == raw_val) {
                        *reg_val = i;
                        return 0;
                }
        }

        bq25601_dbg("%s fail\n", __func__);
        return -1;
}

static int bq25601_set_charging_current(struct charger_device *chg_dev,
			       u32 ichg)
{
	int ret;
	u8 reg_val;

	if (g_sy6974_hw_exist) {
		ret = convert_to_reg(SY6974_ICHG_TABLE, ARRAY_SIZE(SY6974_ICHG_TABLE), ichg, &reg_val);
		if (ret < 0)
			return ret;
	} else {
                ret = convert_to_reg(SGM41513_ICHG_TABLE, ARRAY_SIZE(SGM41513_ICHG_TABLE), ichg, &reg_val);
                if (ret < 0)
                        return ret;
	}

	bq25601_dbg("%s, ichg:%duA, reg_val:0x%x\n", __func__, ichg, reg_val);
	bq25601_set_ichg(reg_val);

	return ret;
}

static int bq25601_get_min_charging_current(struct charger_device *chg_dev, u32 *min_ichg)
{
	if (g_sy6974_hw_exist)
		*min_ichg = SY6974_ICHG_TABLE[0];
	else
		*min_ichg = SGM41513_ICHG_TABLE[0];
        bq25601_dbg("%s, min_ichg:%duA\n", __func__, *min_ichg);

        return 0;
}

static int bq25601_get_input_current(struct charger_device *chg_dev,
				     u32 *aicr)
{
	int ret = 0;
	unsigned char val = 0;

	bq25601_read_interface(bq25601_CON0, &val, CON0_IINLIM_MASK,
			       CON0_IINLIM_SHIFT);
	ret = (int)val;
	*aicr = INPUT_LIMIT_TABLE[val];
	return ret;
}


static int bq25601_set_input_current(struct charger_device *chg_dev,
				     u32 aicr)
{
        int ret;
        u8 reg_val;

	ret = convert_to_reg(INPUT_LIMIT_TABLE, ARRAY_SIZE(INPUT_LIMIT_TABLE), aicr, &reg_val);
	if (ret < 0)
		return ret;

	bq25601_dbg("%s, aicr:%duA, reg_val:0x%x\n", __func__, aicr, reg_val);
	bq25601_set_iinlim(reg_val);
	return ret;
}

static int bq25601_get_min_input_current(struct charger_device *chg_dev, u32 *min_aicr)
{
        *min_aicr = INPUT_LIMIT_TABLE[0];
        bq25601_dbg("%s, min_aicr:%duA\n", __func__, *min_aicr);
        return 0;
}

static int bq25601_set_constant_voltage(struct charger_device *chg_dev,
				  u32 cv)
{
        int ret;
        u8 reg_val;

        ret = convert_to_reg(VREG_TABLE, ARRAY_SIZE(VREG_TABLE), cv, &reg_val);
        if (ret < 0)
                return ret;

        bq25601_dbg("%s, cv:%duV, reg_val:0x%x\n", __func__, cv, reg_val);
	bq25601_set_vreg(reg_val);

	return ret;
}

static int bq25601_reset_watch_dog_timer(struct charger_device
		*chg_dev)
{
	unsigned int status = true;

	bq25601_dbg("charging_reset_watch_dog_timer\n");

	bq25601_set_wdt_rst(0x1);	/* Kick watchdog */
	bq25601_set_watchdog(0x3);	/* WDT 160s */

	return status;
}

static int bq25601_set_mivr(struct charger_device *chg_dev,
				      u32 mivr)
{
	int ret;
	u8 reg_val;

	ret = convert_to_reg(VINDPM_TABLE, ARRAY_SIZE(VINDPM_TABLE), mivr, &reg_val);
	if (ret < 0)
		return ret;

	bq25601_dbg("%s, mivr = %duV, reg_val = 0x%x\n", __func__, mivr, reg_val);
	bq25601_set_vindpm(reg_val);

	return ret;
}

static int bq25601_enable_safetytimer(struct charger_device *chg_dev,
				      bool en)
{
	int status = 0;

	if (en)
		bq25601_set_en_timer(0x1);
	else
		bq25601_set_en_timer(0x0);
	return status;
}

static int bq25601_get_is_safetytimer_enable(struct charger_device
		*chg_dev, bool *en)
{
	unsigned char val = 0;

	bq25601_read_interface(bq25601_CON5, &val, CON5_EN_TIMER_MASK,
			       CON5_EN_TIMER_SHIFT);
	*en = (bool)val;
	return val;
}


static unsigned int charging_hw_init(void)
{
	unsigned int status = 0;

	bq25601_set_en_hiz(0x0);
	bq25601_set_stat_ctrl(0x3); /* Disable STAT Pin Function */
	bq25601_set_vindpm(0x4);	/* VIN DPM check 4.3V */
	bq25601_set_ovp(0x1);		/* vbus overvoltage protection, 6.5V */
	bq25601_set_wdt_rst(0x1);	/* Kick watchdog */
	bq25601_set_sys_min(0x5);	/* Minimum system voltage 3.5V */

	if (g_sy6974_hw_exist) {
		bq25601_set_iprechg(0x3);       /* Precharge current 240mA */
		bq25601_set_iterm(0x1);	    /* Termination current 120mA */
	} else {
		bq25601_set_iprechg(0xf);	/* Precharge current 240mA */
		bq25601_set_iterm(0xb);	    /* Termination current 140mA */
		bq25601_set_itrickchg(0x1); /* Trickle charge current 30mA */
	}

	bq25601_set_vreg(0x12);		/* VREG 4.43V */
	bq25601_set_pfm(0x0);		/* default enable pfm */
	bq25601_set_rdson(0x0);     /* close rdson */
	bq25601_set_batlowv(0x1);	/* BATLOWV 3.15V */
	bq25601_set_vrechg(0x0);	/* VRECHG 0.1V (4.108V) */
	bq25601_set_en_term(0x1);	/* Enable termination */
	bq25601_set_watchdog(0x0);	/* WDT disable */
	bq25601_set_en_timer(0x1);	/* Enable charge timer */
	bq25601_set_int_mask(0x0);	/* Disable fault interrupt */
	bq25601_set_batfet_reset_enable(0x0); /*disable batfet reset */
	bq25601_set_boost_lim(0x1);	/* Boost mode current limit to 1.2A */

	bq25601_dbg("%s: hw_init down!\n", __func__);
	return status;
}

static int bq25601_parse_dt(struct bq25601_info *info,
			    struct device *dev)
{
	struct device_node *np = dev->of_node;
	//int bq25601_en_pin = 0;

	bq25601_dbg("%s\n", __func__);
	if (!np) {
		bq25601_dbg("%s: no of node\n", __func__);
		return -ENODEV;
	}

	if (of_property_read_string(np, "charger_name",
				    &info->chg_dev_name) < 0) {
		info->chg_dev_name = "primary_chg";
		bq25601_dbg("%s: no charger name\n", __func__);
	}

	if (of_property_read_string(np, "alias_name",
				    &(info->chg_props.alias_name)) < 0) {
		info->chg_props.alias_name = "bq25601";
		bq25601_dbg("%s: no alias name\n", __func__);
	}

	sprintf(current_chargerIC_info.chip, info->chg_props.alias_name);
	sprintf(current_chargerIC_info.more, "charger");

	return 0;
}

static int bq25601_enable_vbus(struct regulator_dev *rdev)
{
	bq25601_dbg("%s\n", __func__);
	bq25601_set_chg_config(0);
	bq25601_set_otg_config(1);

	return 0;
}

static int bq25601_disable_vbus(struct regulator_dev *rdev)
{
	bq25601_dbg("%s\n", __func__);
	bq25601_set_otg_config(0);
	bq25601_set_chg_config(1);

	return 0;
}

static int bq25601_is_enabled_vbus(struct regulator_dev *rdev)
{
	bq25601_dbg("%s\n", __func__);
	return bq25601_get_otg_config();
}

static const struct regulator_ops bq25601_vbus_ops = {
	.enable = bq25601_enable_vbus,
	.disable = bq25601_disable_vbus,
	.is_enabled = bq25601_is_enabled_vbus,
};

static const struct regulator_desc bq25601_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &bq25601_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static struct charger_ops bq25601_chg_ops = {
	.enable_hz = bq25601_enable_hz,

        /* enable/disable charger */
        .enable = bq25601_enable,

	/* get/set charging current */
	.get_charging_current = bq25601_get_charging_current,
	.set_charging_current = bq25601_set_charging_current,
	.get_min_charging_current = bq25601_get_min_charging_current,

	/* set cv */
	.set_constant_voltage = bq25601_set_constant_voltage,

	/* set input_current */
	.get_input_current = bq25601_get_input_current,
	.set_input_current = bq25601_set_input_current,
	.get_min_input_current = bq25601_get_min_input_current,

	/* set mivr */
	.set_mivr = bq25601_set_mivr,

	.kick_wdt = bq25601_reset_watch_dog_timer,
	.is_charging_done = bq25601_get_charging_status,

	/* Safety timer */
	.enable_safety_timer = bq25601_enable_safetytimer,
	.is_safety_timer_enabled = bq25601_get_is_safetytimer_enable,

	.dump_registers = bq25601_dump_register,
};

static ssize_t disable_charger_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int temp;

	if (kstrtoint(buf, 10, &temp) == 0 && (temp == 0 || temp == 1))
		disable_charger = temp;
	else
		bq25601_dbg("disable_charger format error!\n");

	return count;
}

static ssize_t disable_charger_show(struct device* cd, struct device_attribute *attr, char* buf)
{
	return sprintf(buf, "%d\n", disable_charger);
}

static ssize_t dump_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0, count = 0;
	u8 reg = 0;

	count += snprintf(buf + count, PAGE_SIZE, "dump bq25601 reg:\n");
	for (i = 0; i <bq25601_REG_NUM; i++) {
		reg = bq25601_read_byte(i);
		count += snprintf(buf + count, PAGE_SIZE, "REG%02x  %02x\n", i, reg);
	}
	count += snprintf(buf + count, PAGE_SIZE, "\n");

	return count;
}

static ssize_t dump_reg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	bq25601_dbg("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		bq25601_dbg("[%s] buf is %s and size is %zu\n", __func__, buf, size);

		pvalue = (char *)buf;
		addr = strsep(&pvalue, " ");
		ret = kstrtou32(addr, 16, (unsigned int *)&reg_address);

		val = strsep(&pvalue, " ");
		ret = kstrtou32(val, 16, (unsigned int *)&reg_value);
		bq25601_dbg("write bq25601 reg 0x%x with value 0x%x !\n", (unsigned int)reg_address, reg_value);

		ret = bq25601_write_byte(reg_address, reg_value);
		if (ret < 0)
			bq25601_dbg("Failed: reg=%02X, ret=%d\n", (unsigned int)reg_address, ret);
	}

	return size;
}

static DEVICE_ATTR(disable_charger, 0664, disable_charger_show, disable_charger_store);
static DEVICE_ATTR(dump_reg, 0664, dump_reg_show, dump_reg_store);

static struct attribute *bq25601_sysfs_entries[] = {
    &dev_attr_disable_charger.attr,
    &dev_attr_dump_reg.attr,
    NULL,
};

static const struct attribute_group bq25601_sysfs_group = {
    .attrs = bq25601_sysfs_entries,
};

static const struct attribute_group *bq25601_attribute_group[] = {
	&bq25601_sysfs_group,
	NULL
};

/**********************************************************
 *
 *   [platform_driver API]
 *
 *********************************************************/
unsigned char g_reg_value_bq25601;
static ssize_t bq25601_access_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	bq25601_dbg("[%s] 0x%x\n", __func__, g_reg_value_bq25601);
	return sprintf(buf, "%u\n", g_reg_value_bq25601);
}

static ssize_t bq25601_access_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	int ret = 0;
	char *pvalue = NULL, *addr, *val;
	unsigned int reg_value = 0;
	unsigned int reg_address = 0;

	bq25601_dbg("[%s]\n", __func__);

	if (buf != NULL && size != 0) {
		bq25601_dbg("[%s] buf is %s and size is %zu\n", __func__, buf,
			size);

		pvalue = (char *)buf;
		if (size > 3) {
			addr = strsep(&pvalue, " ");
			ret = kstrtou32(addr, 16,
				(unsigned int *)&reg_address);
		} else
			ret = kstrtou32(pvalue, 16,
				(unsigned int *)&reg_address);

		if (size > 3) {
			val = strsep(&pvalue, " ");
			ret = kstrtou32(val, 16, (unsigned int *)&reg_value);
			bq25601_dbg(
			"[%s] write bq25601 reg 0x%x with value 0x%x !\n",
			__func__,
			(unsigned int) reg_address, reg_value);
			ret = bq25601_write_interface(reg_address,
				reg_value, 0xFF, 0x0);
		} else {
			ret = bq25601_read_interface(reg_address,
					     &g_reg_value_bq25601, 0xFF, 0x0);
			bq25601_dbg(
			"[%s] read bq25601 reg 0x%x with value 0x%x !\n",
			__func__,
			(unsigned int) reg_address, g_reg_value_bq25601);
			bq25601_dbg(
			"[%s] use \"cat bq25601_access\" to get value\n",
			__func__);
		}
	}
	return size;
}

static DEVICE_ATTR_RW(bq25601_access);

static int bq25601_user_space_probe(struct platform_device *dev)
{
	int ret_device_file = 0;

	bq25601_dbg("******** %s!! ********\n", __func__);

	ret_device_file = device_create_file(&(dev->dev),
					     &dev_attr_bq25601_access);

	return 0;
}

struct platform_device bq25601_user_space_device = {
	.name = "bq25601-user",
	.id = -1,
};

static struct platform_driver bq25601_user_space_driver = {
	.probe = bq25601_user_space_probe,
	.driver = {
		.name = "bq25601-user",
	},
};

static int bq25601_driver_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	static bool probe_done = false;
	struct bq25601_info *info = NULL;
	struct regulator_config config = { };

	bq25601_dbg("[%s]\n", __func__);

	if (probe_done) {
		bq25601_dbg("[%s] already probe done, ignore \n", __func__);
		return ret;
	}

	info = devm_kzalloc(&client->dev, sizeof(struct bq25601_info),
			    GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	new_client = client;
	info->dev = &client->dev;

	if (bq25601_hw_component_detect() < 0) {
		bq25601_dbg("bq25601_hw_component_detect() fail!\n");
		return -ENODEV;
	}

	ret = bq25601_parse_dt(info, &client->dev);
	if (ret < 0)
		return ret;

	charging_hw_init();

	/* Register charger device */
	info->chg_dev = charger_device_register(info->chg_dev_name,
						&client->dev, info,
						&bq25601_chg_ops,
						&info->chg_props);
	if (IS_ERR_OR_NULL(info->chg_dev)) {
		bq25601_dbg("%s: register charger device  failed\n", __func__);
		ret = PTR_ERR(info->chg_dev);
		return ret;
	}

	/* otg regulator */
	config.dev = info->dev;
	config.driver_data = info;
	config.of_node = of_find_node_by_name(NULL, "usb-otg-vbus");
	if (IS_ERR(config.of_node)) {
		ret = PTR_ERR(config.of_node);
		bq25601_dbg("%s: find usb-otg-vbus node failed (%d)\n", __func__, ret);
		return ret;
	}

	config.init_data = of_get_regulator_init_data(config.dev, config.of_node,
					&bq25601_otg_rdesc);
	if (IS_ERR(config.init_data)) {
		ret = PTR_ERR(config.init_data);
		bq25601_dbg("%s: of_get_regulator_init_data failed (%d)\n", __func__, ret);
		return ret;
	}

	info->otg_rdev = devm_regulator_register(info->dev,
					&bq25601_otg_rdesc, &config);
	if (IS_ERR(info->otg_rdev)) {
		ret = PTR_ERR(info->otg_rdev);
		bq25601_dbg("%s: register otg regulator failed (%d)\n", __func__, ret);
		return ret;
	}

	ret = sysfs_create_groups(&info->dev->kobj, bq25601_attribute_group);
	if(ret)
		return ret;

	bq25601_dump_register(info->chg_dev);

	/* bq25601 user space access interface */
	ret = platform_device_register(&bq25601_user_space_device);
	if (ret) {
		bq25601_dbg("****[%s] Unable to device register(%d)\n", __func__, ret);
		return ret;
	}

	ret = platform_driver_register(&bq25601_user_space_driver);
	if (ret) {
		bq25601_dbg("****[%s] Unable to register driver (%d)\n", __func__, ret);
		return ret;
	}

	bq25601_dbg("[%s] probe successfully \n", __func__);
	g_bq25601_info = info;
	probe_done = true;

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id bq25601_of_match[] = {
	{.compatible = "mediatek,bq25601"},
	{},
};
#else
static struct i2c_board_info i2c_bq25601 __initdata = {
	I2C_BOARD_INFO("bq25601", (bq25601_SLAVE_ADDR_WRITE >> 1))
};
#endif

static int set_charge_enable(const char *val, const struct kernel_param *kp)
{
	int recive;
	//struct power_supply *bat_psy;

	recive = param_set_int(val,kp);
	if(enable_charge)
	{
		bq25601_enable(g_bq25601_info->chg_dev, true);
	}
	else
	{
		bq25601_enable(g_bq25601_info->chg_dev, false);
	}
	bq25601_update_charging_status(g_bq25601_info->chg_dev);
	return 0;
}

static int set_supply_enable(const char *val, const struct kernel_param *kp)
{
	int recive;
	recive = param_set_int(val,kp);
	if(enable_supply)
	{
		bq25601_set_en_hiz(0x1);
	}
	else
	{
		bq25601_set_en_hiz(0x0);
	}
	return 0;
}

static int set_ship_mode_enable(const char *val, const struct kernel_param *kp)
{
	int recive;
	recive = param_set_int(val,kp);
	if(ship_mode)
	{
		bq25601_enable_shipping_mode(true);
	}
	else
	{
		bq25601_enable_shipping_mode(false);
	}
	return 0;
}

const struct kernel_param_ops enable_charge_ops =
{
	.set = set_charge_enable,
	.get = param_get_int,
};

const struct kernel_param_ops enable_supply_ops =
{
	.set = set_supply_enable,
	.get = param_get_int,
};

const struct kernel_param_ops ship_mode_ops =
{
	.set = set_ship_mode_enable,
	.get = param_get_int,
};

module_param_cb(enable_charge, &enable_charge_ops, &enable_charge, 0664);
module_param_cb(enable_supply, &enable_supply_ops, &enable_supply, 0664);
module_param_cb(ship_mode, &ship_mode_ops, &ship_mode, 0664);


static struct i2c_driver bq25601_driver = {
	.driver = {
		.name = "bq25601",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = bq25601_of_match,
#endif
	},
	.probe = bq25601_driver_probe,
	.id_table = bq25601_i2c_id,
};

static int __init bq25601_init(void)
{
	/* i2c registration using DTS instead of boardinfo*/
#ifdef CONFIG_OF
	bq25601_dbg("[%s] init start with i2c DTS", __func__);
#else
	bq25601_dbg("[%s] init start. ch=%d\n", __func__, bq25601_BUSNUM);
	i2c_register_board_info(bq25601_BUSNUM, &i2c_bq25601, 1);
#endif
	if (i2c_add_driver(&bq25601_driver) != 0) {
		bq25601_dbg(
			"[%s] failed to register bq25601 i2c driver.\n",
			__func__);
	} else {
		bq25601_dbg(
			"[%s] Success to register bq25601 i2c driver.\n",
			__func__);
	}

	return 0;
}

static void __exit bq25601_exit(void)
{
	i2c_del_driver(&bq25601_driver);
}
module_init(bq25601_init);
module_exit(bq25601_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C bq25601 Driver");
MODULE_AUTHOR("will cai <will.cai@mediatek.com>");
