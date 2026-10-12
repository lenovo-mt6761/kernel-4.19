// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Jitao Shi <jitao.shi@mediatek.com>
 */

#ifndef BUILD_LK
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/module.h>

#include "lcd_bias_lp6280s.h"

/* I2C Slave Setting */
#define lp6280s_SLAVE_ADDR_WRITE	0x7C

static struct i2c_client *new_client;
static const struct i2c_device_id lp6280s_i2c_id[] = { {"lp6280s", 0}, {} };

static int lp6280s_driver_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int lp6280s_driver_remove(struct i2c_client *client);

static const struct of_device_id lp6280s_id[] = {
	{.compatible = "lcd_biias,lp6280s"},
	{},
};

MODULE_DEVICE_TABLE(of, lp6280s_id);

static struct i2c_driver lp6280s_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lp6280s",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lp6280s_id),
#endif
	},
	.probe = lp6280s_driver_probe,
	.remove = lp6280s_driver_remove,
	.id_table = lp6280s_i2c_id,
};

static DEFINE_MUTEX(lp6280s_i2c_access);

/* I2C Function For Read/Write */
int lp6280s_read_byte(unsigned char cmd, unsigned char *returnData)
{
	char cmd_buf[2] = { 0x00, 0x00 };
	char readData = 0;
	int ret = 0;

	mutex_lock(&lp6280s_i2c_access);

	cmd_buf[0] = cmd;
	ret = i2c_master_send(new_client, &cmd_buf[0], 1);
	ret = i2c_master_recv(new_client, &cmd_buf[1], 1);
	if (ret < 0) {
		mutex_unlock(&lp6280s_i2c_access);
		return 0;
	}

	readData = cmd_buf[1];
	*returnData = readData;

	mutex_unlock(&lp6280s_i2c_access);

	return 1;
}

int lp6280s_write_byte(unsigned char cmd, unsigned char writeData)
{
	char write_data[2] = { 0 };
	int ret = 0;

	pr_notice("[KE/lp6280s] %s\n", __func__);

	mutex_lock(&lp6280s_i2c_access);

	write_data[0] = cmd;
	write_data[1] = writeData;

	ret = i2c_master_send(new_client, write_data, 2);
	if (ret < 0) {
		mutex_unlock(&lp6280s_i2c_access);
		pr_notice("[lp6280s] I2C write fail!!!\n");

		return 0;
	}

	mutex_unlock(&lp6280s_i2c_access);

	return 1;
}

static int lp6280s_driver_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int err = 0;

	pr_notice("[KE/lp6280s] name=%s addr=0x%x\n",
		client->name, client->addr);
	new_client = devm_kzalloc(&client->dev,
		sizeof(struct i2c_client), GFP_KERNEL);
	if (!new_client) {
		err = -ENOMEM;
		goto exit;
	}

	memset(new_client, 0, sizeof(struct i2c_client));
	new_client = client;

	return 0;

 exit:
	return err;

}

static int lp6280s_driver_remove(struct i2c_client *client)
{
	pr_notice("[KE/lp6280s] %s\n", __func__);

	new_client = NULL;
	i2c_unregister_device(client);

	return 0;
}

#define RT4801_BUSNUM 1

static int __init lp6280s_init(void)
{
	pr_notice("[KE/lp6280s] %s\n", __func__);

	if (i2c_add_driver(&lp6280s_driver) != 0)
		pr_notice("[KE/lp6280s] failed to register lp6280s i2c driver.\n");
	else
		pr_notice("[KE/lp6280s] Success to register lp6280s i2c driver.\n");

	return 0;
}

static void __exit lp6280s_exit(void)
{
	i2c_del_driver(&lp6280s_driver);
}

module_init(lp6280s_init);
module_exit(lp6280s_exit);

#endif

