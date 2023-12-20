/*
 * leds-aw210xx.c
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 *  Author: hushanping <hushanping@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include "leds-aw2023.h"

 struct aw2023_led {
  	struct i2c_client *client;
  	//struct aw2023_platform_data *pdata;
	int aw210xx_en_gpio;
  	struct mutex lock;
	/*pinctrl*/
  	struct pinctrl          *pinctrl;
  	struct pinctrl_state    *pin_set_high;
  	struct pinctrl_state    *pin_set_low;
};

struct aw2023_led *aw2023_data;


/*
static void aw_gpio_high(unsigned int gpio_num, bool gpio_status)
{
	gpio_request(gpio_num, "aw-gpio");
	gpio_direction_output(gpio_num, gpio_status);
}
*/
static void aw_write_byte(struct i2c_client *client, unsigned char reg, unsigned char val)
{
	int ret = -EINVAL;
	int retry_times = 0;
	do {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		retry_times ++;
		if(retry_times == 5)
			break;
	}while (ret < 0);
	pr_err("AW :aw_write_byte: reg[%02x] val[%02x] ok... ret=%d  retry_times=%d \n", reg, val, ret, retry_times);
	//return ret;
/*
	char txbuf[2] = {aw210xx_page_addr,aw210xx_cmd_page};
	struct i2c_msg msg[2] = {
		[0] = {
			.addr = client->addr,
			.flags= 0,
			.len = sizeof(txbuf),
			.buf = txbuf,
		},
	};
	i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
*/
}

static void aw_set_SLx(struct aw2023_led *aw210xx, unsigned int aw_current)
{
	unsigned int i;
	aw_write_byte(aw210xx->client, 0xF0, 0xC2);
	for(i=0;i<360;i++)
	{
		aw_write_byte(aw210xx->client, i, aw_current);
	}

}

static int aw_power_ctrl(struct aw2023_led *chip, bool en)
{
	int rc;

	if (IS_ERR_OR_NULL(chip->pinctrl) ||
	    IS_ERR_OR_NULL(chip->pin_set_high) ||
	    IS_ERR_OR_NULL(chip->pin_set_low)) {
		pr_err("AW : aw_power_ctrl error\n");
		return -ENODEV;
	}

	if (en)
		{
		rc = pinctrl_select_state(chip->pinctrl,chip->pin_set_high);
		pr_err("AW:set high:%d\n", rc);

		aw_write_byte(chip->client, 0xF0, 0xC0);
		aw_write_byte(chip->client,0x2F, 0xAE);
		mdelay(5);
		pr_err("AW : aw2023 reset.\n");
		/*AW20xxx_ChipSoft_EN*/
		aw_write_byte(chip->client, 0xF0, 0xC0);
		aw_write_byte(chip->client, 0x00, 0x91);
		mdelay(5);
		pr_err("AW :AW20xxx_ChipSoft_EN success .\n");
		/*aw20xxx_set_global_current*/
		aw_write_byte(chip->client, 0xF0, 0xC0);
		aw_write_byte(chip->client, 0x01, 0x19);
		aw_write_byte(chip->client, 0xF0, 0xC2);
		mdelay(5);
		pr_err("AW :aw20xxx_set_global_current success .\n");
		aw_set_SLx(chip, 0xFF);
		mdelay(5);
		pr_err("AW :aw_set_SLx success .\n");
	}else {
		rc = pinctrl_select_state(chip->pinctrl,chip->pin_set_low);
		pr_err("AW:set low:%d\n", rc);
	}

	return rc;
}

static int aw_power_init(struct aw2023_led *chip)
{
	int rc = 0;
	struct device_node *node = NULL;

	if (IS_ERR_OR_NULL(chip)) {
		pr_err("AW:aw_power_init error");
		return -ENODEV;
		}
	node = chip->client->dev.of_node;
	chip->pinctrl = devm_pinctrl_get(&(chip->client)->dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		pr_err("AW:get pinctrl fail\n");
		return -ENODEV;
	}

	chip->pin_set_high = pinctrl_lookup_state(chip->pinctrl, "led_default");
	if (IS_ERR_OR_NULL(chip->pin_set_high)) {
		pr_err("AW:get cp_int_default fail\n");
	}
	rc = pinctrl_select_state(chip->pinctrl, chip->pin_set_high);
	pr_err("AW:led_default set sucess %d\n", rc);

	chip->pin_set_high = pinctrl_lookup_state(chip->pinctrl, "led_en_active");
	if (IS_ERR_OR_NULL(chip->pin_set_high)) {
		pr_err("AW:get led_en_active fail\n");
	}

	chip->pin_set_low = pinctrl_lookup_state(chip->pinctrl, "led_en_sleep");
			if (IS_ERR_OR_NULL(chip->pin_set_low)) {
				pr_err("AW:get cp_en_sleep fail\n");
			}

	chip->aw210xx_en_gpio = of_get_named_gpio(node, "aw210xx_en_gpio", 0);
	if (chip->aw210xx_en_gpio < 0) {
		pr_err("AW :aw2023[LED]:led->aw210xx_en_gpio not specified\n");
		goto free_en_gpio;
	} else {
		if (gpio_is_valid(chip->aw210xx_en_gpio)) {
			rc = gpio_request(chip->aw210xx_en_gpio, "aw210xx_en_gpio");
			if (rc) {
				pr_err("AW :aw2023[LED]:unable to request gpio [%d] %d\n", chip->aw210xx_en_gpio, rc);
				return -ENODEV;
			}
			pr_err("AW :aw2023[led]:Enable the aw210xx_en_gpio, chip->aw210xx_en_gpio is %d\n", chip->aw210xx_en_gpio);
		}
	}

	return 0;

free_en_gpio:
	if (!gpio_is_valid(chip->aw210xx_en_gpio))
		gpio_free(chip->aw210xx_en_gpio);
	return rc;

}

static void aw20xxx_set_led_120_open(struct aw2023_led *aw210xx, unsigned int led_num)
{
	unsigned int i;
	aw_write_byte(aw210xx->client, 0xF0, 0xC1);
	for(i=0;i<360;i++)
	{
		aw_write_byte(aw210xx->client, i,led_num);
	}
}

static ssize_t aw2023_led_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t cnt)
{
	int val = 0;

	int rc = 0;
	struct aw2023_led *chip = aw2023_data;
	pr_err("AW :aw2023_led_ctrl_store enter .\n");

	sscanf(buf, "%d", &val);
	if(val){

		pr_err("AW :led ctrl true enter .\n");
		rc = aw_power_ctrl(chip, true);
			if (rc < 0) {
				pr_err("AW :aw_power_ctrl_true fail %d\n", rc);
				return rc;
				}
		aw20xxx_set_led_120_open(chip, 0x1E);
		mdelay(10);
		pr_err("AW :aw20xxx_set_led_120_open open .\n");
		return cnt;
	} else {

		pr_err("AW :led ctrl false enter .\n");
		rc = aw_power_ctrl(chip, false);
			if (rc < 0) {
				pr_err("AW :aw_power_ctrl_false fail %d\n", rc);
				return rc;
				}

		aw20xxx_set_led_120_open(chip, 0x00);
		pr_err("AW :aw20xxx_set_led_120_open close .\n");
		return cnt;
	}
	return 0;
}
static ssize_t aw2023_led_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	pr_err("AW :aw2023_led_ctrl_show \n");
	return snprintf(buf, PAGE_SIZE, "aw2023_led_ctrl_show (max:1) \n");
}

static DEVICE_ATTR(ledctrl, 0664, aw2023_led_ctrl_show,  aw2023_led_ctrl_store);

static struct attribute *aw2023_led_attributes[] = {
	&dev_attr_ledctrl.attr,
	NULL,
};

static struct attribute_group aw2023_led_attr_group = {
		.attrs = aw2023_led_attributes,
};


/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw210xx_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct aw2023_led *aw210xx;
	struct device_node *node;
	int num_leds = 1;
	int rc = 0;
	node = client->dev.of_node;
	pr_err("AW :aw210xx_i2c_probe  in \n");
	if (node == NULL)
		return -EINVAL;

	aw210xx = devm_kzalloc(&client->dev,
  			(sizeof(struct aw2023_led) * num_leds), GFP_KERNEL);
	if (!aw210xx)
		return -ENOMEM;
	aw2023_data = aw210xx;

	aw210xx->client = client;
	aw2023_data->client = client;
	rc = sysfs_create_group(&client->dev.kobj, &aw2023_led_attr_group);
	if (rc) {
		pr_err("AW : led sysfs error rc: %d\n", rc);
	}

	rc = aw_power_init(aw210xx);
		if (rc < 0) {
			pr_err("gpio init error, rc=%d\n", rc);
		}
	return 0;
}

static void aw210xx_i2c_remove(struct i2c_client *i2c)
{
	sysfs_remove_group(&i2c->dev.kobj, &aw2023_led_attr_group);
	pr_err("AW :aw210xx_i2c_remove \n");
}

static const struct i2c_device_id aw210xx_i2c_id[] = {
	{"aw2023_led", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw210xx_i2c_id);

static const struct of_device_id aw210xx_dt_match[] = {
	{.compatible = "awinic,aw2023_led",},
	{},
};

static struct i2c_driver aw210xx_i2c_driver = {
	.driver = {
		.name = "aw2023_led",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw210xx_dt_match),
		},
	.probe = aw210xx_i2c_probe,
	.remove = aw210xx_i2c_remove,
	.id_table = aw210xx_i2c_id,
};

static int __init aw210xx_i2c_init(void)
{
	int ret = 0;
	pr_err("AW :aw210xx_i2c_init  in \n");
	ret = i2c_add_driver(&aw210xx_i2c_driver);
	if (ret) {
		pr_err("AW :i2c_add_driver  error \n");
		return ret;
	}

	return 0;
}
module_init(aw210xx_i2c_init);

static void __exit aw210xx_i2c_exit(void)
{
	pr_err("AW :aw210xx_i2c_init  off \n");
	i2c_del_driver(&aw210xx_i2c_driver);
}
module_exit(aw210xx_i2c_exit);

MODULE_DESCRIPTION("AW210XX LED Driver");
MODULE_LICENSE("GPL v2");
