// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
#include "lcm_define.h"

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#include <linux/gpio.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif
#endif

#ifndef MACH_FPGA
#include <lcm_pmic.h>
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID (0x98)

static const unsigned int BL_MIN_LEVEL = 20;
static struct LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)	(lcm_util.set_reset_pin((v)))
#define MDELAY(n)		(lcm_util.mdelay(n))
#define UDELAY(n)		(lcm_util.udelay(n))


#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	  lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define GPIO_OUT_LOW 0
#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0

#ifndef BUILD_LK
static unsigned int GPIO_LCD_RST;
static unsigned int GPIO_LCD_LED_EN;
static unsigned int GPIO_LCD_PWR_ENP;
static unsigned int GPIO_LCD_PWR_ENN;
static unsigned short s_last_backlight_level = 255; //default level, must align with lk

//Gesture flag
volatile bool ft8201m_gesture_flag = false;
static bool ft8201m_first_boot_flag = true;


#include "Backlight_I2C_map_hx.h"
#include "sgm37604a.h"
#include "lcd_bias_lp6280s.h"
#define LP6280S_REG_WRITE(add, data) lp6280s_write_byte(add, data)
#define LP6280S_REG_READ(add) lp6280s_read_byte(add)


void lcm_request_gpio_control(struct device *dev)
{

	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	GPIO_LCD_RST = of_get_named_gpio(dev->of_node, "gpio_lcd_rst", 0);
	gpio_request(GPIO_LCD_RST, "GPIO_LCD_RST");
	GPIO_LCD_LED_EN = of_get_named_gpio(dev->of_node, "gpio_lcd_led_en", 0);
	gpio_request(GPIO_LCD_LED_EN, "GPIO_LCD_LED_EN");
	GPIO_LCD_PWR_ENP = of_get_named_gpio(dev->of_node,
		"gpio_lcd_pwr_enp", 0);
	gpio_request(GPIO_LCD_PWR_ENP, "GPIO_LCD_PWR_ENP");
	GPIO_LCD_PWR_ENN = of_get_named_gpio(dev->of_node,
		"gpio_lcd_pwr_enn", 0);
	gpio_request(GPIO_LCD_PWR_ENN, "GPIO_LCD_PWR_ENN");

    pr_notice("[Kernel/LCM] %s get gpio success\n", __func__);
}

static int lcm_driver_probe(struct device *dev, void const *data)
{
	lcm_request_gpio_control(dev);

	return 0;
}

static const struct of_device_id lcm_platform_of_match[] = {
	{
		.compatible = "ft,ft8201",
		.data = 0,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, platform_of_match);

static int lcm_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		.name = "ft8201",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);
	if (platform_driver_register(&lcm_driver)) {
		pr_notice("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_drv_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}

late_initcall(lcm_drv_init);
module_exit(lcm_drv_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
#endif

/* ----------------------------------------------------------------- */
/* Local Constants */
/* ----------------------------------------------------------------- */
static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE      0
#define FRAME_WIDTH           (800)
#define FRAME_HEIGHT          (1280)
#define LCM_DENSITY           (213)

#define LCM_PHYSICAL_WIDTH    (107640)
#define LCM_PHYSICAL_HEIGHT	  (172224)

#define REGFLAG_DELAY		  0xFFFC
#define REGFLAG_UDELAY	      0xFFFB
#define REGFLAG_END_OF_TABLE  0xFFFD
#define REGFLAG_RESET_LOW     0xFFFE
#define REGFLAG_RESET_HIGH    0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* ----------------------------------------------------------------- */
/*  Local Variables */
/* ----------------------------------------------------------------- */

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
#ifdef BUILD_LK
	mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO, output);
#else
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);
#endif
}

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{0x00, 0, {0x00} },
	{0xF7, 4, {0x5A, 0xA5, 0x95, 0x27}}
};

static struct LCM_setting_table init_setting_vdo[] = {
#if 0
    //initial version 0x05
    {REGFLAG_DELAY,10,{}},
    {REGFLAG_DELAY,255,{}},
    //============CMD WR enable=============
    {0x00,1,{0x00}},
    {0xFF,3,{0x82,0x01,0x01}},

    {0x00,1,{0x80}},
    {0xFF,2,{0x82,0x01}},

    //======voltage set==========
    {0x00,1,{0x93}},//VGH_N 20V
    {0xC5,1,{0x84}},

    {0x00,1,{0x97}},
    {0xC5,1,{0x84}},

    {0x00,1,{0x9E}},//VGH Ratio 2AVDD-2AVEE
    {0xC5,1,{0x0A}},

    {0x00,1,{0x9A}},//VGL_N -12V 2AVEE-AVDD
    {0xC5,1,{0xC3}},

    {0x00,1,{0x9C}},//VGL_I -12V 2AVEE-AVDD
    {0xC5,1,{0xC3}},

    {0x00,1,{0xB6}},
    {0xC5,2,{0x75,0x75}},  //VGHO1_N_I 19V

    {0x00,1,{0xB8}},
    {0xC5,2,{0x39,0x39}}, //VGLO1_N_I -11V

    //{0x00,1,{0x00}},
    //{0xD8,2,{0xD2,0xD2}}, //GVDDP/N 5.6V  -5.6V

    {0x00,1,{0x00}},
    {0xD8,2,{0xD7,0xCD}}, //GVDDP/N 5.65  -5.55

    //{0x00,1,{0x00}},
    //{0xD9,3,{0x00,0x82,0x82}}, //VCOM(-1V)

    {0x00,1,{0x82}},
    {0xC5,1,{0x95}}, //LVD

    {0x00,1,{0x83}},
    {0xC5,1,{0x07}}, //LVD Enable


    //==========gamma==============
    //Analog Gamma 2.2 #1
    {0x00,1,{0x00}},
    {0xE1,16,{0x05,0x0A,0x15,0x22,0x2A,0x33,0x42,0x4E,0x4E,0x5A,0x5B,0x6E,0x96,0x85,0x85,0x79}},
    {0x00,1,{0x10}},
    {0xE1,8,{0x70,0x62,0x4F,0x43,0x38,0x25,0x0B,0x00}},

    {0x00,1,{0x00}},
    {0xE2,16,{0x05,0x0A,0x15,0x22,0x2A,0x33,0x42,0x4E,0x4E,0x5A,0x5B,0x6E,0x96,0x85,0x85,0x79}},
    {0x00,1,{0x10}},
    {0xE2,8,{0x70,0x62,0x4F,0x43,0x38,0x25,0x0B,0x00}},


    //reg_sd_sap
    {0x00,1,{0x80}},
    {0xA4,1,{0x8C}},

    //OSC Auto calibration
    {0x00,1,{0xA0}},
    {0xF3,1,{0x10}},

    //############################################################################################################
    //===FT8201AB===

    // B3A1~B3A4 砞﹚秆猂
    // B3A5=0x00 Source Resolution B3A5[7:5]
    //B3A6=0x13  [4]:reg_panel_sd_resol,1,{[1]:reg_panel_size,1,{[0]:reg_panel_zigzag
    {0x00,1,{0xA1}},
    {0xB3,2,{0x03,0x20}},   //X=800
    {0x00,1,{0xA3}},
    {0xB3,2,{0x05,0x00}},   //Y=1280
    {0x00,1,{0xA5}},
    {0xB3,2,{0x00,0x13}},

    // DMA Setting C1D0=0x30 (Single Chip) / 0xb0 (Multiple Drop)
    {0x00,1,{0xD0}},
    {0xC1,1,{0x30}},

    // ## GOA Timing ##

    //SKIP LVD Power-OFF0
    {0x00,1,{0x80}},
    {0xCB,7,{0x33,0x33,0x30,0x33,0x30,0x33,0x30}},
    {0x00,1,{0x87}},
    {0xCB,1,{0x33}},
    {0x00,1,{0x88}},
    {0xCB,8,{0x33,0x33,0x33,0x33,0x33,0x30,0x33,0x33}},
    {0x00,1,{0x90}},
    {0xCB,7,{0x30,0x33,0x33,0x33,0x30,0x30,0x33}},
    {0x00,1,{0x97}},
    {0xCB,1,{0x33}},

    //Power-OFF NORM BOE8"_M8
    {0x00,1,{0x98}},
    {0xCB,8,{0x00,0x00,0x00,0x04,0x00,0x05,0x00,0x00}},
    {0x00,1,{0xA0}},
    {0xCB,8,{0x04,0x04,0x04,0x04,0x00,0x00,0x04,0x08}},
    {0x00,1,{0xA8}},
    {0xCB,8,{0x00,0x07,0x04,0x00,0x00,0x00,0x00,0x00}},

    //Power-On BOE8"_M8
    {0x00,1,{0xB0}},
    {0xCB,7,{0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x00,1,{0xB7}},
    {0xCB,1,{0x00}},
    {0x00,1,{0xB8}},
    {0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x00,1,{0xC0}},
    {0xCB,7,{0x00,0xFF,0x00,0x00,0x00,0x00,0x00}},
    {0x00,1,{0xC7}},
    {0xCB,1,{0x00}},

    //U2D_BOE8"_M8_20201201
    {0x00,1,{0x80}},
    {0xCC,8,{0x29,0x29,0x08,0x08,0x20,0x20,0x2A,0x2A}},
    {0x00,1,{0x88}},
    {0xCC,8,{0x2B,0x2B,0x2C,0x2C,0x1A,0x1A,0x19,0x19}},
    {0x00,1,{0x90}},
    {0xCC,6,{0x18,0x18,0x17,0x17,0x00,0x00}},
    {0x00,1,{0x80}},
    {0xCD,8,{0x29,0x29,0x07,0x07,0x1F,0x1F,0x2A,0x2A}},
    {0x00,1,{0x88}},
    {0xCD,8,{0x2B,0x2B,0x2C,0x2C,0x16,0x16,0x15,0x15}},
    {0x00,1,{0x90}},
    {0xCD,6,{0x14,0x14,0x13,0x13,0x00,0x00}},

    //D2U
    {0x00,1,{0xA0}},
    {0xCC,8,{0x29,0x29,0x07,0x07,0x1F,0x1F,0x2A,0x2A}},
    {0x00,1,{0xA8}},
    {0xCC,8,{0x2B,0x2B,0x2C,0x2C,0x13,0x13,0x14,0x14}},
    {0x00,1,{0xB0}},
    {0xCC,6,{0x15,0x15,0x16,0x16,0x00,0x00}},
    {0x00,1,{0xA0}},
    {0xCD,8,{0x29,0x29,0x08,0x08,0x20,0x20,0x2A,0x2A}},
    {0x00,1,{0xA8}},
    {0xCD,8,{0x2B,0x2B,0x2C,0x2C,0x17,0x17,0x18,0x18}},
    {0x00,1,{0xB0}},
    {0xCD,6,{0x19,0x19,0x1A,0x1A,0x00,0x00}},


    //goa_vstx_shift_cnt_sel (20200812)
    {0x00,1,{0x81}},
    {0xC2,1,{0x40}},    //C281[6]=1

    //VST5   BOE 8" STV1
    {0x00,1,{0x90}},
    {0xC2,4,{0x88,0x02,0xBD,0xA0}},

    //VST6   BOE 8" STV3
    {0x00,1,{0x94}},
    {0xC2,4,{0x87,0x02,0xBD,0xA0}},

    //VEND5  BOE 8" STV2
    {0x00,1,{0xB0}},
    {0xC2,4,{0x13,0x02,0xBD,0xA0}},

    //VEND6  BOE 8" STV4
    {0x00,1,{0xB4}},
    {0xC2,4,{0x14,0x02,0xBD,0xA0}},

    //CLKC_BOE8" Gout_R side_CLK1357
    {0x00,1,{0x80}},
    {0xC3,8,{0x82,0x09,0x02,0xFF,0xA0,0x00,0x02,0x07}},
    {0x00,1,{0x88}},
    {0xC3,8,{0x00,0x0B,0x02,0xFF,0xA0,0x00,0x02,0x07}},
    {0x00,1,{0x90}},
    {0xC3,8,{0x86,0x0D,0x02,0xFF,0xA0,0x00,0x02,0x07}},
    {0x00,1,{0x98}},
    {0xC3,8,{0x84,0x0F,0x02,0xFF,0xA0,0x00,0x02,0x07}},

    //CLKD_BOE8" Gout_L side_CLK2468
    {0x00,1,{0xC0}},
    {0xCD,8,{0x81,0x0A,0x02,0xFF,0xA0,0x00,0x02,0x07}},
    {0x00,1,{0xC8}},
    {0xCD,8,{0x01,0x0C,0x02,0xFF,0xA0,0x00,0x02,0x07}},
    {0x00,1,{0xD0}},
    {0xCD,8,{0x85,0x0E,0x02,0xFF,0xA0,0x00,0x02,0x07}},
    {0x00,1,{0xD8}},
    {0xCD,8,{0x83,0x10,0x02,0xFF,0xA0,0x00,0x02,0x07}},

    //GOFF1 BOE8" GCL
    {0x00,1,{0xE0}},
    {0xC3,4,{0x26,0xA0,0x01,0x00}},

    //GOFF1 BOE8" GCH
    {0x00,1,{0xE4}},
    {0xC3,4,{0x26,0xA0,0x01,0x00}},

    //ECLK (20200812)
    {0x00,1,{0xF0}},
    {0xCC,4,{0x3D,0x88,0x88,0xC2,0x90}},

    //CLK EQ
    {0x00,1,{0xC8}},
    {0xC3,1,{0x0F}},

    //=============== ## TCON Timing START ##   ===============//
    {0x00,1,{0x80}},
    {0xC0,6,{0x00,0xD6,0x00,0xC1,0x00,0x10}},

    {0x00,1,{0x90}},
    {0xC0,6,{0x00,0xD6,0x00,0xC1,0x00,0x10}},

    {0x00,1,{0xA0}},
    {0xC0,6,{0x01,0xAC,0x00,0xC1,0x00,0x10}},

    {0x00,1,{0xB0}},
    {0xC0,5,{0x00,0xD6,0x00,0xC1,0x10}},

    {0x00,1,{0xA3}},
    {0xC1,3,{0x24,0x1A,0x04}},

    {0x00,1,{0x80}},
    {0xCE,1,{0x00}},

    {0x00,1,{0x90}},
    {0xCE,1,{0x00}},

    {0x00,1,{0xD0}},
    {0xCE,8,{0x01,0x00,0x1E,0x01,0x01,0x00,0xA1,0x00}},

    {0x00,1,{0xE0}},
    {0xCE,1,{0x00}},

    {0x00,1,{0xF0}},
    {0xCE,1,{0x00}},

    {0x00,1,{0xB0}},
    {0xCF,4,{0x00,0x00,0x4B,0x4F}},

    {0x00,1,{0xB5}},
    {0xCF,4,{0x02,0x02,0x7A,0x7E}},

    {0x00,1,{0xC0}},
    {0xCF,4,{0x04,0x04,0xA0,0xA4}},

    {0x00,1,{0xC5}},
    {0xCF,4,{0x00,0x05,0x08,0x00}},

    //=============== ## TCON Timing END ## ===============//


    //CLK Source delay  (20200812)
    {0x00,1,{0x90}},
    {0xC4,1,{0x88}},    //CLK Source delay

    //CLK allon
    {0x00,1,{0x92}},
    {0xC4,1,{0xC0}},

    //VGHO1 sink OFF (20210125)
    {0x00,1,{0xA8}},
    {0xC5,1,{0x09}},//sink mode select 9

    //VGLO1 sink ON
    {0x00,1,{0xCB}},
    {0xC5,1,{0x01}},

    //reg_goa_select_dg_rtn
    {0x00,1,{0xFD}},
    {0xCB,1,{0x82}},

    //pump VGH afd off (20210125)(ACmode off)
    {0x00,1,{0x9F}},
    {0xC5,1,{0x00}},

    //PUMP  VGH CLK all on disable
    {0x00,1,{0x91}},
    {0xC5,1,{0x4C}},

    //tcon_tp_term2_vb_ln
    {0x00,1,{0xD7}},
    {0xCE,1,{0x01}},

    //VGH CLK Line Rate normal:5(4line)
    {0x00,1,{0x94}},
    {0xC5,1,{0x45}},

    //tp term VGH pump off CLK gating L (20210125)
    {0x00,1,{0x98}},
    {0xC5,1,{0x54}},//VGH CLK Line Rate Idle 5(4line)

    //VGL CLK Line Rate normal:5(4line)
    {0x00,1,{0x9B}},
    {0xC5,1,{0x55}},

    //VGL CLK Line Rate Idle:5(4line)
    {0x00,1,{0x9D}},
    {0xC5,1,{0x55}},

    //Gamma Chop
    {0x00,1,{0x90}},
    {0xE9,1,{0x40}},

    //GVDD Enable
    {0x00,1,{0xA0}},
    {0xA5,1,{0x04}},

    //Gamma AP
    {0x00,1,{0x84}},
    {0xC5,1,{0xAA}},

    //Source sd_sap
    {0x00,1,{0x80}},
    {0xA4,1,{0xFC}},

    {0x00,1,{0x89}},
    {0xC4,1,{0x20}},

    {0x00,1,{0x9A}},
    {0xC4,1,{0xCA}},

    //VGH pump AFD sequence
    {0x00,1,{0x8C}},
    {0xCF,2,{0x40,0x40}},

    //en_Vcom can't be gating by LVD
    {0x00,1,{0x9A}},
    {0xF5,1,{0x35}},

    //Source pull low setting
    {0x00,1,{0xA2}},
    {0xF5,1,{0x1F}},

    //Vsync sync with mipi
    {0x00,1,{0xA0}},
    {0xC1,1,{0xE0}},

    //FIFO
    {0x1C,1,{0x00}},

    //tp_term_sync_vb
    {0x00,1,{0xC9}},
    {0xCE,1,{0x00}},
    //============CMD WR disable=============
    {0x00,1,{0x00}},
    {0xFF,3,{0x00,0x00,0x00}},

    {0x00,1,{0x80}},
    {0xFF,2,{0x00,0x00}},
#endif
    //----------------------LCD initial code End----------------------//
    {0x53, 1, {0x2C}},//控制背光开关
    {0x51, 1, {0xFF}},//设置背光亮度

    //SLPOUT and DISPON
    {0x11,0,{}},
    //delayms_pc(120)
    {REGFLAG_DELAY,120,{}},
    {0x29,0,{}},
    {0x35,1,{0x00}},
    //delayms_pc(300)
   // {REGFLAG_DELAY,255,{}}
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_UDELAY:
			UDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count,
				table[i].para_list, force_update);
		}
	}
}

static void lcm_set_util_funcs(const struct LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(struct LCM_UTIL_FUNCS));
}

static void lcm_get_params(struct LCM_PARAMS *params)
{
	memset(params, 0, sizeof(struct LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = LCM_PHYSICAL_WIDTH/1000;
	params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
	params->density = LCM_DENSITY;
	/* params->physical_width_um = LCM_PHYSICAL_WIDTH; */
	/* params->physical_height_um = LCM_PHYSICAL_HEIGHT; */

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
	/* lcm_dsi_mode = CMD_MODE; */
#else
	params->dsi.mode = BURST_VDO_MODE;
	/* params->dsi.switch_mode = CMD_MODE; */
	/* lcm_dsi_mode = SYNC_PULSE_VDO_MODE; */
#endif
	/* LCM_LOGI("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode); */
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 8;
	params->dsi.vertical_backporch = 8;
	params->dsi.vertical_frontporch = 193;
	/* disable dynamic frame rate */
	/* params->dsi.vertical_frontporch_for_low_power = 540; */
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 6;
	params->dsi.horizontal_backporch = 5;
	params->dsi.horizontal_frontporch = 6;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	params->dsi.ssc_range = 4;
	params->dsi.ssc_disable = 1;
	/* params->dsi.ssc_disable = 1; */
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 270;
#else
	params->dsi.PLL_CLOCK = 260;//modified by anhengxuan.boe,fps=60,20220707
#endif
	/* params->dsi.PLL_CK_CMD = 220; */
	/* params->dsi.PLL_CK_VDO = 255; */
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
	params->dsi.clk_lp_per_line_enable = 1;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	params->round_corner_en = 0;
	params->corner_pattern_width = 720;
	params->corner_pattern_height = 32;
#endif

	/* params->use_gpioID = 1; */
	/* params->gpioID_value = 0; */
}

static void lcm_init_power(void)
{

}

static void lcm_resume_power(void)
{

}

static void lcm_init(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(5);
	lcm_set_gpio_output(GPIO_LCD_PWR_ENP, GPIO_OUT_ONE);//avdd
	/* Set AVDD = 6.0V */
	LP6280S_REG_WRITE(0x00, 0x14);
	/* Set AVEE = -6.0V */
	LP6280S_REG_WRITE(0x01, 0x14);
	LP6280S_REG_WRITE(0x03, 0x43);
	MDELAY(7);
	lcm_set_gpio_output(GPIO_LCD_PWR_ENN, GPIO_OUT_ONE);//avee
	MDELAY(7);
	/* after 6ms reset HLH */
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(10);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ZERO);
	MDELAY(1);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(20);//OPARSLEY-2249--solve decrease wake up time,by anhengxuan.boe,20220811 

	push_table(NULL,
		init_setting_vdo,
		sizeof(init_setting_vdo) / sizeof(struct LCM_setting_table),
		1);
	lcm_set_gpio_output(GPIO_LCD_LED_EN, GPIO_OUT_ONE);
	MDELAY(5);
	//register
	sgm37604a_write_byte(0x11, 0x00);
	pr_notice("[Kernel/LCM] %s exit\n", __func__);
}

static void lcm_suspend(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	push_table(NULL,
		lcm_suspend_setting,
		sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
		1);
	MDELAY(10);

	lcm_set_gpio_output(GPIO_LCD_LED_EN, GPIO_OUT_ZERO);

	pr_notice("[Kernel/LCM] %s end\n", __func__);
}

static void lcm_suspend_power(void)
{
	if(!ft8201m_gesture_flag) {
	MDELAY(100);
	LP6280S_REG_WRITE(0x00, 0x00);
	LP6280S_REG_WRITE(0x01, 0x00);
	MDELAY(35);
	lcm_set_gpio_output(GPIO_LCD_PWR_ENP, GPIO_OUT_ZERO);
	lcm_set_gpio_output(GPIO_LCD_PWR_ENN, GPIO_OUT_ZERO);
	}
	pr_notice("[Kernel/LCM] %s end\n", __func__);
}

static void lcm_resume(void)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	lcm_init();
	sgm37604a_write_byte(0x1A, (backlight_i2c_map_hx[s_last_backlight_level] & 0x0F));
	sgm37604a_write_byte(0x19, (backlight_i2c_map_hx[s_last_backlight_level] >> 4));
	pr_info("lsy_kernel %s,ft8201m backlight: s_last_backlight_level = %d \n", __func__,s_last_backlight_level);
    pr_notice("[Kernel/LCM] %s end\n", __func__);
}


#if (LCM_DSI_CMD_MODE)

/* partial update restrictions:
 * 1. roi width must be 1080 (full lcm width)
 * 2. vertical start (y) must be multiple of 16
 * 3. vertical height (h) must be multiple of 16
 */
static void lcm_validate_roi(int *x, int *y, int *width, int *height)
{
	unsigned int y1 = *y;
	unsigned int y2 = *height + y1 - 1;
	unsigned int x1, w, h;

	x1 = 0;
	w = FRAME_WIDTH;

	y1 = round_down(y1, 16);
	h = y2 - y1 + 1;

	/* in some cases, roi maybe empty. Then we need to use minimu roi */
	if (h < 16)
		h = 16;

	h = round_up(h, 16);

	/* check height again */
	if (y1 >= FRAME_HEIGHT || y1 + h > FRAME_HEIGHT) {
		/* assign full screen roi */
		pr_notice("%s calc error, assign full roi: y=%d, h=%d\n",
			__func__, *y, *height);
		y1 = 0;
		h = FRAME_HEIGHT;
	}

	/*LCM_LOGD("lcm_validate_roi (%d,%d,%d,%d) to (%d,%d,%d,%d)\n",*/
	/*	*x, *y, *width, *height, x1, y1, w, h);*/

	*x = x1;
	*width = w;
	*y = y1;
	*height = h;
}
#endif

static unsigned int lcm_ata_check(unsigned char *buffer)
{
	return 1;
}

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{
	int sgm_level = 0;

	if (level > 255)
		level = 255;
       if(ft8201m_first_boot_flag){
                ft8201m_first_boot_flag=false;
                level = 128;
                s_last_backlight_level=128;
        }

	sgm_level = backlight_i2c_map_hx[level];
	pr_info("lsy_kernel %s,ft8201m backlight: level = %d sgm_level = %d\n", __func__, level, sgm_level);

        if( level== s_last_backlight_level){
                sgm37604a_write_byte(0x1A, (backlight_i2c_map_hx[s_last_backlight_level] & 0x0F));
                sgm37604a_write_byte(0x19, (backlight_i2c_map_hx[s_last_backlight_level] >> 4));
        }

	while (level != s_last_backlight_level) {
		if (level > s_last_backlight_level)
			s_last_backlight_level++;
		else if (level < s_last_backlight_level)
			s_last_backlight_level--;
	if(0!=s_last_backlight_level)
	{
		sgm37604a_write_byte(0x1A, (backlight_i2c_map_hx[s_last_backlight_level] & 0x0F));
		sgm37604a_write_byte(0x19, (backlight_i2c_map_hx[s_last_backlight_level] >> 4));
	}
	else
	{
		sgm37604a_write_byte(0x1A, (backlight_i2c_map_hx[1] & 0x0F));
		sgm37604a_write_byte(0x19, (backlight_i2c_map_hx[1] >> 4));
	}
	}
}

struct LCM_DRIVER ft8201m_wxga_vdo_incell_boe_lcm_drv = {
	.name = "ft8201m_wxga_vdo_incell_boe",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.ata_check = lcm_ata_check,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
#if (LCM_DSI_CMD_MODE)
	.validate_roi = lcm_validate_roi,
#endif
};

