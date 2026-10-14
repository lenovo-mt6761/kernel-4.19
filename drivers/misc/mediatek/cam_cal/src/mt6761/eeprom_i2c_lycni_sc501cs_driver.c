/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cam_cal_list.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#include "eeprom_i2c_lycni_sc501cs_driver.h"

static struct i2c_client *g_pstI2CclientG;
#define SC501CS_OTP_DEBUG  1
#define SC501CS_I2C_ID     0x6c /*0x20*/

#define OTP_SECTION1_START_ADDR 0x80EC  //modifiable;the first section with 1800 Bytes OTP;
#define OTP_SECTION1_END_ADDR 0x87F3   //modifiable;
#define OTP_SECTION2_START_ADDR 0x8800 //modifiable;the second section with 2036 Bytes OTP;
#define OTP_SECTION2_END_ADDR 0x8FEF    //modifiable;don't set over 0x8FF3

#define AWB_DATA_SIZE 8
#define VALID_OTP_FLAG 0x55

static u16 read_cmos_sensor(u16 addr)
{
	u16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *)&get_byte, 1, SC501CS_I2C_ID);

	return get_byte;
}

static void write_cmos_sensor8(u16 addr, u16 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8),
		(char)(addr & 0xFF), (char)(para & 0xFF)};

	iWriteRegI2C(pu_send_cmd, 3, SC501CS_I2C_ID);
}

static int sc501cs_sensor_otp_init(int section, u8 threshold,int i){
	int delay=0;
	u16 otp_start_add_h,otp_start_add_l,otp_end_add_h,otp_end_add_l;

	if(section == 1) {
		otp_start_add_h = (OTP_SECTION1_START_ADDR&0x0f00)>>8;
		otp_start_add_l = OTP_SECTION1_START_ADDR&0x00ff;
		otp_end_add_h   = (OTP_SECTION1_END_ADDR&0x0f00)>>8;
		otp_end_add_l   = OTP_SECTION1_END_ADDR&0x00ff;

		write_cmos_sensor8(0x36b0,0x4c);
		//write_cmos_sensor8(0x36b1,0xd8);
		write_cmos_sensor8(0x36b1,threshold);
		write_cmos_sensor8(0x36b2,0x01);
		write_cmos_sensor8(0x4408,otp_start_add_h);
		write_cmos_sensor8(0x4409,otp_start_add_l);
		write_cmos_sensor8(0x440a,otp_end_add_h);
		write_cmos_sensor8(0x440b,otp_end_add_l);
		write_cmos_sensor8(0x4401,0x1f);
		write_cmos_sensor8(0x4400,0x11);

		//msleep(5);
		while((read_cmos_sensor(0x4420)&0x01) == 0x01){
			delay++;
			if(delay == 1000) {
				pr_debug("1st section OTP is still busy for reading. R0x4420[0]!=0 \n");
				return 0;
			}
		}
	} else if(section == 2) {
		otp_start_add_h = (OTP_SECTION2_START_ADDR&0x0f00)>>8;
		otp_start_add_l = OTP_SECTION2_START_ADDR&0x00ff;
		otp_end_add_h   = (OTP_SECTION2_END_ADDR&0x0f00)>>8;
		otp_end_add_l   = OTP_SECTION2_END_ADDR&0x00ff;

		write_cmos_sensor8(0x4408,otp_start_add_h);
		write_cmos_sensor8(0x4409,otp_start_add_l);
		write_cmos_sensor8(0x440a,otp_end_add_h);
		write_cmos_sensor8(0x440b,otp_end_add_l);
		write_cmos_sensor8(0x4401,0x1e);
		write_cmos_sensor8(0x4400,0x11);

		while((read_cmos_sensor(0x4420)&0x01) == 0x01){
			delay++;
			if(delay == 1000) {
				pr_debug("2nd section OTP is still busy for reading. R0x4420[0]!=0 \n");
				return 0;
			}
		}
	} else {
		pr_debug("section input error \n");
		return 0;
	}
	#if 0
	otp_start_add_h = (OTP_SECTION1_START_ADDR&0x0f00)>>8;
	otp_start_add_l = OTP_SECTION1_START_ADDR&0x00ff;
	otp_end_add_h   = (OTP_SECTION1_END_ADDR&0x0f00)>>8;
	otp_end_add_l   = OTP_SECTION1_END_ADDR&0x00ff;
	write_cmos_sensor8(0x4424,0x03);
	write_cmos_sensor8(0x4408,0x00);
	write_cmos_sensor8(0x4409,0x00);
	write_cmos_sensor8(0x440a,0x07);
	write_cmos_sensor8(0x440b,0xff);
	write_cmos_sensor8(0x4402,0x02);
	write_cmos_sensor8(0x4403,0x0a);
	write_cmos_sensor8(0x4404,0x1c);
	write_cmos_sensor8(0x4405,0x24);
	write_cmos_sensor8(0x440c,0x2e);
	write_cmos_sensor8(0x440d,0x2e);
	write_cmos_sensor8(0x440e,0x22);
	write_cmos_sensor8(0x440f,0x39);
	write_cmos_sensor8(0x4401,0x1f);
	#endif
	return 1;
}

/*
static void SC501CS_otp_close(void)
{

	write_cmos_sensor(0xf3,0x00);
}
*/

// static u8 SC501CS_otp_read_group(u16 addr, u8 *data, u8 length)
// {
// 	u8 i = 0;
// 	for (i = 0; i < length; i++) {
// 		data[i] = read_cmos_sensor(addr + i);
// #if SC501CS_OTP_DEBUG
// 		pr_debug("addr = 0x%x, data = 0x%x\n", addr + i, data[i]);
// #endif
// 	}
// 	return 0;
// }



int sc501cs_iReadData(unsigned int ui4_offset,
	unsigned int ui4_length, unsigned char *pinputdata)
{
	//int i4RetValue = 0;
	pr_debug("sc501cslycniotp ui4_offset = 0x%x, ui4_length = %d \n", ui4_offset, ui4_length);
	if(ui4_offset == 0x811a || ui4_offset == 0x88a2){
	/*===========lsc flag==================================*/
	int threshold[3] ={ 0xd8, 0x98,0x58};
	int j;
	int i = 0;
	//int h = 0;
	pr_debug("into ui4_offset == 0x811a\n");
	sc501cs_sensor_otp_init(1, threshold[j], j);
	pr_debug("into ui4_offset == 0x811a sc501cs_sensor_otp_init\n");
			pinputdata[i] = read_cmos_sensor(ui4_offset);
			pr_debug(" pinputdata = 0x%x, read_cmos_sensor(h) = %x \n", pinputdata[i], read_cmos_sensor(ui4_offset));
	}else if (ui4_offset == 0x88a3|| ui4_offset == 0x811b)
    {
	/*===========lsc==================================*/
	int threshold[3] ={ 0xd8, 0x98,0x58};
	int j,checksum_addr;
	unsigned int check_sum = 0;
	int i = 0;
	int h = 0;
	for(j=0;j<3;j++)
    {
	if(ui4_offset == 0x811b){ //lsc group1
	    checksum_addr = 0x8873;

			sc501cs_sensor_otp_init(1, threshold[j], j);
			for(i = 0, h = ui4_offset; h <= 0x87f3; i++, h++)
			{
				pr_info("===========lyp=========sc501cslycni========= ");
				pinputdata[i] = read_cmos_sensor(h);
				check_sum += pinputdata[i];
				pr_info("sc501cslycni data: 0x%x, addr: 0x%x\n",pinputdata[i], h);
				pr_info("sc501cslycni_lsc_ check_sum1: 0x%x\n",check_sum);
			}
			sc501cs_sensor_otp_init(2,threshold[j],j);
			for(h = 0x8800; h <= 0x8872; i++, h++)
			{
				pinputdata[i] = read_cmos_sensor(h);
				check_sum += pinputdata[i];
				pr_info("sc501cslycni data: 0x%x, addr: 0x%x\n",pinputdata[i], h);
				pr_info("sc501cslycni_lsc_ check_sum2: 0x%x\n",check_sum);
			}
			check_sum =check_sum + VALID_OTP_FLAG ;
			if(read_cmos_sensor(checksum_addr) == (check_sum%0xFF+1))
			{
			printk("sc501cslycni: lsc read_cmos_sensor:0x%x checksum PASS\n ",read_cmos_sensor(checksum_addr));
			printk("sc501cslycni: lsc data:0x%x checksum PASS\n ",(check_sum%0xFF+1));
			break;
			}
			else
			{
			printk("sc501cslycni: lsc daread_cmos_sensor:0x%x checksum Fail\n", read_cmos_sensor(checksum_addr));
			printk("sc501cslycni: lsc data:0x%x checksum Fail\n", (check_sum%0xFF+1));
			continue;
			}
	}else if(ui4_offset == 0x88a3){ //lsc group2
	    checksum_addr = 0x8FEF;
		sc501cs_sensor_otp_init(2, threshold[j], j);
		for(i = 0, h = ui4_offset; h < ui4_length; i++, h++)
		{
			pinputdata[i] = read_cmos_sensor(h);
			check_sum += pinputdata[i];
			pr_info("sc501cslycni data: 0x%x, addr: 0x%x\n",pinputdata[i], h);
		}
		check_sum =check_sum + VALID_OTP_FLAG ;
		if(read_cmos_sensor(checksum_addr) == (check_sum%0xFF+1))
		{
		printk("sc501cslycni: lsc data checksum PASS\n ");
		 break;
		}
	    else
		{
		printk("sc501cslycni: lsc data checksum Fail\n ");
		 continue;
		}
	  }
	}
   }else{
/*=======================module_info=awb=af==================================================*/
	int threshold[3] ={ 0xd8, 0x98,0x58};
	int check_sum = 0;
	int checksum_addr =0;
	int i = 0;
	int h = 0;
	int j = 0;
	switch(ui4_offset)
	{
		case 0x80EC://module_info flag1
		checksum_addr = 0x8109;
		break;
		case 0x8873://module_info flag2
		checksum_addr = 0x8891;
		break;
		case 0x80ED://module_info group1
		checksum_addr = 0x8109;
		break;
		case 0x8874://module_info group2
		checksum_addr = 0x8891;
		break;
		case 0x810A://awb group1 flag1
		checksum_addr = 0x8113;
		break;
		case 0x810B://awb group1
		checksum_addr = 0x8113;
		break;
		case 0x8892://awb group2 flag2
		checksum_addr = 0x889B;
		break;
		case 0x8893://awb group2
		checksum_addr = 0x889B;
		break;
		case 0x8114://af group1 flag1
		checksum_addr = 0x8119;
		break;
		case 0x8115://af group1
		checksum_addr = 0x8119;
		break;
		case 0x889C://af group2 flag2
		checksum_addr = 0x88A1;
		case 0x889D://af group2
		checksum_addr = 0x88A1;
		break;
		default:
		pr_info("sc501cslycni:flag:0x%x\n",ui4_offset);

	}

	for(j=0;j<3;j++)
	{
		if(ui4_offset <= 0x8873){
			sc501cs_sensor_otp_init(1, threshold[j],j);
		}else{
			sc501cs_sensor_otp_init(2, threshold[j], j);
		}

		for(i = 0, h = ui4_offset; i < ui4_length; i++, h++)
		{
			pinputdata[i] = read_cmos_sensor(h);
			check_sum += pinputdata[i];
			pr_info("sc501cslycni_other_ check_sum1: 0x%x\n",check_sum);
			pr_info("sc501cslycni data: 0x%x, addr: 0x%x\n",pinputdata[i], h);
			pr_info("sc501cslycni_other_ check_sum2: 0x%x\n",check_sum);
			pr_info("sc501cslycni_read_cmos_sensor(checksum_addr): 0x%x\n",read_cmos_sensor(checksum_addr));
			pr_info("sc501cslycni_(check_sum_FF+1): 0x%x\n",(check_sum%0xFF+1));
		}
		check_sum =check_sum + VALID_OTP_FLAG ;
		//if(ui4_length >1)
		if((ui4_offset ==0x80ED && ui4_length >4) || ui4_offset ==0x8875 ||ui4_offset ==0x810B || ui4_offset ==0x8893|| ui4_offset ==0x8115 || ui4_offset ==0x889D)
		{
			if(read_cmos_sensor(checksum_addr) == (check_sum%0xFF+1))
			{
				if(ui4_offset ==0x80ED || ui4_offset ==0x8875){
				printk("sc501cslycni: addr:0x%x module_info data checksum PASS\n",ui4_offset);
				}
				else
				if(ui4_offset ==0x810B || ui4_offset ==0x8893){
				printk("sc501cslycni: addr:0x%x awb data checksum PASS\n",ui4_offset);
				}
				else
				if(ui4_offset ==0x8115 || ui4_offset ==0x889D){
				printk("sc501cslycni: addr:0x%x af data checksum PASS\n",ui4_offset);
				}
				else{
				printk("sc501cslycni: addr:0x%x data checksum FAIL\n",ui4_offset);
				}
				break;
			}
			else
			{
				if(ui4_offset ==0x80ED || ui4_offset ==0x8875){
				printk("sc501cslycni: addr:0x%x module_info data checksum FAIL\n",ui4_offset);
				}
				else
				if(ui4_offset ==0x810B || ui4_offset ==0x8893){
				printk("sc501cslycni: addr:0x%x awb data checksum FAIL\n",ui4_offset);
				}
				if(ui4_offset ==0x8115 || ui4_offset ==0x889D){
				printk("sc501cslycni: addr:0x%x af data checksum FAIL\n",ui4_offset);
				}
				else{
				printk("sc501cslycni: addr:0x%x data checksum FAIL\n",ui4_offset);
				}
				continue;
			}
		}
	}
   }

	// pr_debug("offset = 0x%x, length = %d \n", ui4_offset, ui4_length);
	// mdelay(10);

	// i4RetValue = SC501CS_otp_read_group(ui4_offset, pinputdata, ui4_length);
	// if (i4RetValue != 0) {
	// 	pr_debug("I2C iReadData failed!!\n");
	// 	return -1;
	// }

    // //SC501CS_otp_close();
	 return 0;
}

unsigned int sc501cs_lycni_read_region(struct i2c_client *client, unsigned int addr,
				unsigned char *data, unsigned int size)
{
	g_pstI2CclientG = client;
	if (sc501cs_iReadData(addr, size, data) == 0)
		return size;
	else
		return 0;
}

