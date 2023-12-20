// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * leds-aw20198.c   aw20198 led module
 *
 * Copyright (c) 2020 Shanghai Awinic Technology Co., Ltd. All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation;either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <linux/firmware.h>
#include <linux/time.h>

#include "leds-aw20198.h"
#include "leds-aw20198_effect.h"

static u32 old_brightness = 0;

DEFINE_MUTEX(aw_led_lock);

static const u8 led_addr[121]= {
	94,100,93,99,92,98,91,90,108,
	109,116,110,117,111,118,112,119,113,115,
	114,151,149,155,148,144,145,152,146,153,
	147,154,150,132,133,131,137,130,136,129,
	135,128,134,127,126,162,163,170,164,171,
	165,172,166,173,167,169,168,24,25,23,
	29,22,28,21,27,20,26,19,18,36,
	37,44,38,45,39,46,40,47,41,43,
	42,60,61,59,65,58,64,57,63,56,
	62,55,54,72,73,80,74,81,75,82,
	76,83,77,79,78,0 ,1 ,8 ,2 ,9 ,
	3 ,10,4 ,11,5 ,7 ,6 ,96,97,95,101,94
};

/*******************************************************************************
 *
 * hardware enable/off
 *
 ******************************************************************************/

static int aw20198_hw_enable(struct aw_led_chip *chip)
{
	if(!gpio_get_value(chip->enable_gpio)) {
		gpio_set_value(chip->enable_gpio, 1);
		usleep_range(10000, 10500);
		LED_INFO("%s: set gpio %d high\n", __func__, chip->enable_gpio);
	}

	return 0;
}

static int aw20198_hw_disable(struct aw_led_chip *chip)
{
	if(gpio_get_value(chip->enable_gpio)) {
		gpio_set_value(chip->enable_gpio, 0);
		usleep_range(2000, 2500);
		LED_INFO("%s: set gpio %d low\n", __func__, chip->enable_gpio);
	}

	return 0;
}

/*******************************************************************************
 *
 * aw20198 i2c read/write
 *
 ******************************************************************************/
static int
aw20198_i2c_read(struct i2c_client *client, unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = 0;
	u8 cnt = 0;
	int value = 0;

	while (cnt < AW_I2C_READ_RETRIES) {
		value = i2c_smbus_read_byte_data(client, reg_addr);
		if (value < 0) {
			ret = value;
			pr_err("%s: i2c read cnt=%d, error=%d\n", __func__, cnt, ret);
		} else {
			*reg_data = value;
			ret = 0;
			break;
		}
		cnt++;
		usleep_range(2000, 2500);
	}

	return ret;
}

static int
aw20198_i2c_write(struct i2c_client *client, unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_WRITE_RETRIES) {
		ret = i2c_smbus_write_byte_data(client, reg_addr, reg_data);
		if (ret < 0)
			pr_err("%s: i2c write cnt=%d, error=%d\n", __func__, cnt, ret);
		else
			break;

		cnt++;
		usleep_range(2000, 2500);
	}

	return ret;
}

static int aw20198_i2c_write_bit(struct i2c_client *client,
				unsigned char reg_addr, unsigned int mask,
				unsigned char reg_data)
{
	unsigned char reg_val;

	aw20198_i2c_read(client, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	aw20198_i2c_write(client, reg_addr, reg_val);

	return 0;
}

static int aw20198_set_page(struct i2c_client *client, unsigned char reg_data)
{
	return aw20198_i2c_write(client, AWPAGEADDR, reg_data);
}

static int aw20198_block_write(struct aw_led_dev *aw20198, const char *buf, int count)
{
	return i2c_master_send(aw20198->client, buf, count);
}

/*******************************************************************************
 *
 * aw20198 led init
 *
 ******************************************************************************/

static int aw20198_led_init(struct aw_led_chip *chip)
{
	int i = 0;

	/* enter page0 */
	aw20198_set_page(chip->client, AWPAGE0);
	aw20198_i2c_write(chip->client, REG_GCR, 0x90);
	/* set global current */
	if (chip->light_sensor_state) {
		aw20198_i2c_write(chip->client, REG_GCCR, LED_GCCR_MAX_VAL);
	} else {
		aw20198_i2c_write(chip->client, REG_GCCR, LED_GCCR_MIN_VAL);
	}
	/* set constant current */
	aw20198_set_page(chip->client, AWPAGE2);
	for (i = 0; i <= AW20198_CFG_CNT_PAGE2 - 2; i++)
		aw20198_i2c_write(chip->client, REG_SL0 + i, chip->sl_current);

	LED_INFO("light sensor state %d\n", chip->light_sensor_state);
	return 0;
}
void aw20198_swrst(struct aw_led_dev *aw20198)
{
	/* select page */
	aw20198_set_page(aw20198->client, 0xc0);
	/* software reset  */
	aw20198_i2c_write(aw20198->client, 0x2f, 0xae);
	usleep_range(2000, 2500);
	aw20198_led_init(aw20198->chip);
	LED_INFO("software reset complete\n");
}

void aw20198_clear_deghost(struct i2c_client *client)
{
	aw20198_set_page(client, AWPAGE0);
	aw20198_i2c_write_bit(client, REG_DGCR, (~(1<<7)), (1<<7));
	LED_INFO("clear de-ghost");
}

/*******************************************************************************
 *
 * aw20198 brightness work
 *
 ******************************************************************************/
void aw20198_cc_mode(struct aw_led_dev *led)
{
	u8 reg_page1_pwm[AW20198_CFG_CNT_PAGE1];
	struct timespec64 begin, end;
	u32 timeval = 0;

	aw20198_set_page(led->client, AWPAGE1);

	if (led->cdev.brightness > led->cdev.max_brightness)
		led->cdev.brightness = led->cdev.max_brightness;

	/* set all led brightness */
	memset(reg_page1_pwm, led->cdev.brightness, sizeof(reg_page1_pwm));
	/* base address */
	reg_page1_pwm[0] = 0x00;

	ktime_get_real_ts64(&begin);
	aw20198_block_write(led, reg_page1_pwm, AW20198_CFG_CNT_PAGE1);
	ktime_get_real_ts64(&end);
	timeval = (timeval_to_ns(end) - timeval_to_ns(begin));

	if (led->cdev.brightness > 0) {
		/* set chip enable */
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	LED_INFO("brightness %d i2c timeval %d ns\n", led->cdev.brightness, timeval);
}

void aw20198_breathe_instance(struct aw_led_dev *led, struct aw_breath_t *bth)
{
	u8 reg_page1_pwm[AW20198_CFG_CNT_PAGE1];
	u32 cnt = 0;
	u8 value = 0;
	struct timespec64 begin, end;
	u64 timeval = 0;

	ktime_get_real_ts64(&begin);
	/* set chip enable */
	aw20198_set_page(led->client, AWPAGE0);
	aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);

	aw20198_set_page(led->client, AWPAGE3);
	reg_page1_pwm[0] = 0x0;
	if (bth->single_led) {
		memset(&reg_page1_pwm[1], 0, AW20198_CFG_CNT_PAGE3);
		reg_page1_pwm[32] = 0x4;  // 31 * 3 + 1 = 94
	} else {
		memset(&reg_page1_pwm[1], 0x15, AW20198_CFG_CNT_PAGE3);
	}
	aw20198_block_write(led, reg_page1_pwm, AW20198_CFG_CNT_PAGE3);

	aw20198_set_page(led->client, AWPAGE0);

	/* set pattern breath PWM */
	aw20198_i2c_write(led->client, 0x30, bth->max_pwm);
	aw20198_i2c_write(led->client, 0x31, bth->max_pwm);
	aw20198_i2c_write(led->client, 0x32, bth->max_pwm);
	aw20198_i2c_write(led->client, 0x33, bth->min_pwm);
	aw20198_i2c_write(led->client, 0x34, bth->min_pwm);
	aw20198_i2c_write(led->client, 0x35, bth->min_pwm);

	/* set T0 T1 T2 T3 */
	aw20198_i2c_write(led->client, 0x36, (bth->t0_rise << 4) | bth->t1_high);
	aw20198_i2c_write(led->client, 0x3a, (bth->t0_rise << 4) | bth->t1_high);
	aw20198_i2c_write(led->client, 0x3e, (bth->t0_rise << 4) | bth->t1_high);

	aw20198_i2c_write(led->client, 0x37, (bth->t2_fall << 4) | bth->t3_low);
	aw20198_i2c_write(led->client, 0x3b, (bth->t2_fall << 4) | bth->t3_low);
	aw20198_i2c_write(led->client, 0x3f, (bth->t2_fall << 4) | bth->t3_low);

	/* set pattern breath start/end phase */
	aw20198_i2c_write(led->client, 0x38, 0x00);
	aw20198_i2c_write(led->client, 0x3c, 0x00);
	aw20198_i2c_write(led->client, 0x40, 0x00);

	aw20198_i2c_write(led->client, 0x39, bth->loop_times);
	aw20198_i2c_write(led->client, 0x3d, bth->loop_times);
	aw20198_i2c_write(led->client, 0x41, bth->loop_times);

	/* set auto breath mode */
	aw20198_i2c_write(led->client, 0x42, 0x03);
	aw20198_i2c_write(led->client, 0x43, 0x03);
	aw20198_i2c_write(led->client, 0x44, 0x03);

	/* start breath */
	aw20198_i2c_write(led->client, 0x45, 0x77);

	while (!(value & (0x1 << 5)) && bth->loop_times && (cnt < 10000))
	{
		aw20198_i2c_read(led->client, 0x42, &value);
		msleep(1);
		cnt++;
	};

	ktime_get_real_ts64(&end);
	timeval = (timeval_to_ns(end) - timeval_to_ns(begin));

	LED_INFO("breathe, %x %x %x %x loop %d single_led %d timeval %ld ns cnt %d\n",
		bth->t0_rise, bth->t1_high, bth->t2_fall, bth->t3_low, bth->loop_times, bth->single_led,
		timeval, cnt);
}

void aw20198_breathe_clear(struct aw_led_dev *led)
{
	u8 reg_page1_pwm[AW20198_CFG_CNT_PAGE1];

	aw20198_set_page(led->client, AWPAGE3);
	memset(reg_page1_pwm, 0, AW20198_CFG_CNT_PAGE3);
	aw20198_block_write(led, reg_page1_pwm, AW20198_CFG_CNT_PAGE3);
	aw20198_set_page(led->client, AWPAGE0);
	aw20198_i2c_write(led->client, 0x42, 0x00);
	aw20198_i2c_write(led->client, 0x43, 0x00);
	aw20198_i2c_write(led->client, 0x44, 0x00);
	aw20198_i2c_write(led->client, 0x45, 0x00);
}

void aw20198_breathe_mode(struct aw_led_dev *led)
{
	u8 page1_pwm[AW20198_CFG_CNT_PAGE1];
	int i;

	for (i = 0; i <= 255; i += 5) {
		msleep(1);
		memset(page1_pwm, i, sizeof(page1_pwm));
		page1_pwm[0] = 0x00;
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_block_write(led, page1_pwm, AW20198_CFG_CNT_PAGE1);
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	msleep(20);

	for (i = 255; i >= 0; i--) {
		msleep(10);
		memset(page1_pwm, i, sizeof(page1_pwm));
		page1_pwm[0] = 0x00;
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_block_write(led, page1_pwm, AW20198_CFG_CNT_PAGE1);
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}
}
void aw20198_marquee_mode(struct aw_led_dev *led, u32 brightness)
{
	int i, j;

	for (j = 0; j < 5; j++) {
		msleep(2);
		for (i = 0; i < 60; i++) {
			msleep(2);
			aw20198_set_page(led->client, AWPAGE1);

			aw20198_i2c_write(led->client, led_addr[i], brightness);

			aw20198_i2c_write(led->client, led_addr[119 - i], brightness);

			/* set chip enable */
			aw20198_set_page(led->client, AWPAGE0);
			aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
		}

		for (i = 0; i < 60; i++) {
			msleep(2);
			aw20198_set_page(led->client, AWPAGE1);

			aw20198_i2c_write(led->client, led_addr[i], 0);

			aw20198_i2c_write(led->client, led_addr[119 - i], 0);

			/* set chip enable */
			aw20198_set_page(led->client, AWPAGE0);
			aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
		}
	}

	LED_INFO("marquee 5 times loop , brightness %d\n", brightness);
}

void aw20198_incall_mode(struct aw_led_dev *led, u32 brightness)
{
	int i, j;
	struct timespec64 begin, end, end1;
	u64 timeval = 0;
	u64 timeval1 = 0;
	u32 step = 0;

	ktime_get_real_ts64(&begin);
	if (brightness < 32) {
		LED_INFO("invailed brightness %d\n", brightness);
		return;
	}
	step = (brightness - 8) / 16;
	for (i = 0; i <= 60; i++) {
		aw20198_set_page(led->client, AWPAGE1);
		for (j = MIN(i, 16); j >= 0; j--) {
			aw20198_i2c_write(led->client, led_addr[i - j], brightness - (j * step));
		}

		for (j = MIN(i, 16); j >= 0; j--) {
			aw20198_i2c_write(led->client, led_addr[120 - i + j], brightness - (j * step));
		}
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	for (i = 0; i < 16; i++) {
		msleep(1);
		aw20198_set_page(led->client, AWPAGE1);
		for (j = 1; j <= 16 - i; j++) {
			aw20198_i2c_write(led->client, led_addr[60 - j], brightness - ((j + i) * step));
			aw20198_i2c_write(led->client, led_addr[60 + j], brightness - ((j + i) * step));
		}
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}
	ktime_get_real_ts64(&end);
	for (i = 0; i <= 60; i++) {
		msleep(1);
		aw20198_set_page(led->client, AWPAGE1);

		aw20198_i2c_write(led->client, led_addr[i], 0);
		aw20198_i2c_write(led->client, led_addr[120 - i], 0);

		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}
	msleep(200);
	ktime_get_real_ts64(&end1);
	timeval = (timeval_to_ns(end) - timeval_to_ns(begin));
	timeval1 = (timeval_to_ns(end1) - timeval_to_ns(end));
	LED_INFO("1 call, brightness %d, step %d, timeval %ld, timeval1 %ld\n", brightness, step, timeval, timeval1);
}

void aw20198_notify_mode(struct aw_led_dev *led, u32 brightness)
{
	struct aw_breath_t breath;
	u8 page1_pwm[AW20198_CFG_CNT_PAGE1];
	u32 max_brightness = brightness;
	int i;
	struct timespec64 begin, end, end1, end2;
	u64 timeval = 0;
	u64 timeval1 = 0;
	u64 timeval2 = 0;

	ktime_get_real_ts64(&begin);
	for (i = 0; i <= max_brightness; i += 5) {
		msleep(10);
		memset(page1_pwm, i, sizeof(page1_pwm));
		page1_pwm[0] = 0x00;
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_block_write(led, page1_pwm, AW20198_CFG_CNT_PAGE1);
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	msleep(20);

	for (i = max_brightness; i >= 0; i -= 5) {
		msleep(10);
		memset(page1_pwm, i, sizeof(page1_pwm));
		page1_pwm[0] = 0x00;
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_block_write(led, page1_pwm, AW20198_CFG_CNT_PAGE1);
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}
	ktime_get_real_ts64(&end1);
	msleep(200);
	for (i = 0; i < max_brightness; i += 5) {
		msleep(8);
		memset(page1_pwm, i, sizeof(page1_pwm));
		page1_pwm[0] = 0x00;
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_block_write(led, page1_pwm, AW20198_CFG_CNT_PAGE1);
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}
	ktime_get_real_ts64(&end2);
	msleep(50);
	for (i = 0; i < 60; i++) {
		msleep(8);
		aw20198_set_page(led->client, AWPAGE1);

		aw20198_i2c_write(led->client, led_addr[60 - i], 0);
		aw20198_i2c_write(led->client, led_addr[60 + i], 0);

		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}
	ktime_get_real_ts64(&end);
	timeval = (timeval_to_ns(end1) - timeval_to_ns(begin));
	timeval1 = (timeval_to_ns(end2) - timeval_to_ns(end1));
	timeval2 = (timeval_to_ns(end) - timeval_to_ns(end2));

	breath.t0_rise = 8;
	breath.t1_high =  0;
	breath.t2_fall = 8;
	breath.t3_low = 0;
	breath.loop_times = 0; /* 0 means forever */
	breath.max_pwm = 150;
	breath.min_pwm = 0;
	breath.single_led = 1;
	aw20198_breathe_instance(led, &breath);

	LED_INFO("brightness %d timeval %ld ns, timeval1 %ld ns, timeval2 %ld ns,finish\n",
		brightness, timeval, timeval1, timeval2);
}


void aw20198_charge_start(struct aw_led_dev *led)
{
	int i, j;
	struct timespec64 begin, end, end1;
	u64 timeval = 0;
	u64 timeval1 = 0;
	u32 step = 4;

	ktime_get_real_ts64(&begin);

	for (i = 0; i < 46; i++) {
		msleep(1);
		aw20198_set_page(led->client, AWPAGE1);
		if (i == 0) {
			aw20198_i2c_write(led->client, led_addr[0], 24);
			continue;
		}

		for (j = 0; j <= MIN(i, 6); j++) {
			aw20198_i2c_write(led->client, led_addr[i + j], 24 - j * step);
		}

		for (j = 0; j <= MIN(i, 6); j++) {
			aw20198_i2c_write(led->client, led_addr[120 - i - j], 24 - j * step);
		}

		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	ktime_get_real_ts64(&end);

	for (i = 46; i > 0; i--) {
		msleep(1);
		aw20198_set_page(led->client, AWPAGE1);

		for (j = 0; j <= 6; j++) {
			aw20198_i2c_write(led->client, led_addr[i + j], 24 - j * step);
		}

		for (j = 0; j <= 6; j++) {
			aw20198_i2c_write(led->client, led_addr[120 - i - j], 24 - j * step);
		}

		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	old_brightness = 0;
	ktime_get_real_ts64(&end1);
	timeval = (timeval_to_ns(end) - timeval_to_ns(begin));
	timeval1 = (timeval_to_ns(end1) - timeval_to_ns(end));
	LED_INFO("6 timeval %ld, timeval1 %ld\n", timeval, timeval1);
}

void aw20198_charge_going_delay(u32 brightness)
{
	switch (brightness) {
	case 1:
		msleep(80);
		break;
	case 2:
		msleep(30);
		break;
	case 3:
		msleep((20));
		break;
	case 4:
		msleep((15));
		break;
	case 5:
		msleep((10));
		break;
	case 6:
		msleep((10));
		break;
	case 7:
		msleep((5));
		break;
	case 8:
		msleep((5));
		break;
	case 9:
		msleep((5));
		break;
	case 10:
		msleep((5));
		break;
	default:
		break;
	}
}
int aw20198_charge_going(struct aw_led_dev *led, u32 brightness)
{
	u8 val[2];
	int i = 0;
	struct timespec64 begin, end;
	u64 timeval = 0;
	u32 led_len = 6 * brightness;
	u32 change_len = 24;
	u32 temp;

	ktime_get_real_ts64(&begin);

	if (brightness > 10) {
		old_brightness = 0;
		return 1;
	}

	if (brightness <= 1)
		change_len = 6;
	else if (brightness <= 3)
		change_len = 12;
	else
		change_len = 24;


	if (old_brightness != brightness) {
		if (old_brightness > brightness) {
			aw20198_swrst(led);
		}
		for (i = 0; i <= led_len; i++) {
			msleep(5);
			aw20198_set_page(led->client, AWPAGE1);

			if (i <= led_len - change_len || brightness == 10) {
				aw20198_i2c_write(led->client, led_addr[i], 24);
				aw20198_i2c_write(led->client, led_addr[120 -i], 24);
				aw20198_set_page(led->client, AWPAGE0);
				aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
				continue;
			}
			temp = 24 - (change_len + i- led_len) * (24 / change_len);
			aw20198_i2c_write(led->client, led_addr[i], temp);
			aw20198_i2c_write(led->client, led_addr[120 - i], temp);
			aw20198_set_page(led->client, AWPAGE0);
			aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
		}

		old_brightness = brightness;
	}

	for (i = 0; i <= led_len; i++) {
		if (i > 60) {
			break;
		}
		aw20198_charge_going_delay(brightness);
		aw20198_set_page(led->client, AWPAGE1);

		if (i == 0) {
			aw20198_i2c_write(led->client, led_addr[0], 72);
			continue;
		}
		if (i > 1) {
			aw20198_i2c_write(led->client, led_addr[i - 1], val[0]);
			aw20198_i2c_write(led->client, led_addr[120 - i + 1], val[1]);
		}

		aw20198_i2c_read(led->client, led_addr[i], &val[0]);
		aw20198_i2c_read(led->client, led_addr[120 - i], &val[1]);

		if (val[0] == 24) {
			aw20198_i2c_write(led->client, led_addr[i], 72);
			aw20198_i2c_write(led->client, led_addr[120 - i], 72);
		} else if (i < led_len - 6 && brightness > 2) {
			aw20198_i2c_write(led->client, led_addr[i], val[0] * 2);
			aw20198_i2c_write(led->client, led_addr[120 - i], val[1] * 2);
		} else if (i < led_len - 3 && brightness <= 2) {
			aw20198_i2c_write(led->client, led_addr[i], val[0] * 3);
			aw20198_i2c_write(led->client, led_addr[120 - i], val[1] * 3);
		} else {
			continue;
		}

		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	if (i > 1) {
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_i2c_write(led->client, led_addr[i - 1], val[0]);
		aw20198_i2c_write(led->client, led_addr[120 - i + 1], val[1]);

		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}


	ktime_get_real_ts64(&end);
	timeval = (timeval_to_ns(end) - timeval_to_ns(begin));
	LED_INFO("timeval %ld\n", timeval);
	return 0;
}

void aw20198_charge_done(struct aw_led_dev *led)
{
	int i;
	struct timespec64 begin, end;
	u64 timeval = 0;

	ktime_get_real_ts64(&begin);

	for (i = 60; i >= 0; i--) {
		msleep(2);
		aw20198_set_page(led->client, AWPAGE1);

		aw20198_i2c_write(led->client, led_addr[i], 0);
		aw20198_i2c_write(led->client, led_addr[119 - i], 0);

		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	ktime_get_real_ts64(&end);
	timeval = (timeval_to_ns(end) - timeval_to_ns(begin));
	LED_INFO("timeval %ld\n", timeval);
}

void aw20198_charge_mode(struct aw_led_dev *led, u32 brightness)
{
	if (brightness == 0) {
		LED_INFO("brightness is %ld\n", brightness);
		return;
	}
	switch (led->charge_stage) {
	case CHARGE_START:
		aw20198_charge_start(led);
		led->charge_stage = CHARGE_GOING;
		fallthrough;
	case CHARGE_GOING:
		if (!aw20198_charge_going(led, brightness)) {
			break;
		}
		led->charge_stage = CHARGE_DONE;
		clear_bit(AW_LED_CHARGE_MODE, &led->effect_bitmap);
		fallthrough;
	case CHARGE_DONE:
		aw20198_charge_done(led);
		led->charge_stage = CHARGE_MAX;
		break;
	default:
		break;
	}
}

void aw20198_poweron_mode_start(struct aw_led_dev *led)
{
	u8 page1_pwm[AW20198_CFG_CNT_PAGE1];
	u8 val[8];
	int i, k;
	u32 step = 0;
	struct timespec64 begin, end, end1, end2;
	u64 timeval = 0;
	u64 timeval1 = 0;
	u64 timeval2 = 0;

	ktime_get_real_ts64(&begin);
	for (i = 0; i <= 8; i++) {
		msleep(1);
		memset(page1_pwm, i, sizeof(page1_pwm));
		page1_pwm[0] = 0x00;
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_block_write(led, page1_pwm, AW20198_CFG_CNT_PAGE1);
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	ktime_get_real_ts64(&end1);

	for (i = 50; i < 170; i += 3) {
		msleep(10);

		aw20198_set_page(led->client, AWPAGE1);
		if (i > 50) {
			for (k = i; k < i + 3; k++)
				aw20198_i2c_write(led->client, led_addr[(k - 3) % 120], val[k % 3]);
		}
		for (k = i; k < i + 3; k++)
			aw20198_i2c_read(led->client, led_addr[k % 120], &val[k % 3]);

		for (k = i; k < i + 3; k++)
			aw20198_i2c_write(led->client, led_addr[k % 120], 88);

		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}
	ktime_get_real_ts64(&end2);

	for (i = 50; i < 290; i += 3) {
		msleep(5);
		step++;

		aw20198_set_page(led->client, AWPAGE1);

		for (k = i; k < i + 3; k++)
			aw20198_i2c_write(led->client, led_addr[(k - 3) % 120], val[k % 3]);

		for (k = i; k < i + 3; k++)
			aw20198_i2c_read(led->client, led_addr[k % 120], &val[k % 3]);

		memset(page1_pwm, 8 + step, sizeof(page1_pwm));
		page1_pwm[0] = 0x00;
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_block_write(led, page1_pwm, AW20198_CFG_CNT_PAGE1);

		for (k = i; k < i + 3; k++)
			aw20198_i2c_write(led->client, led_addr[k % 120], 88);

		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	for (i = 88; i >= 24; i--) {
		msleep(1);
		memset(page1_pwm, i, sizeof(page1_pwm));
		page1_pwm[0] = 0x00;
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_block_write(led, page1_pwm, AW20198_CFG_CNT_PAGE1);
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}

	ktime_get_real_ts64(&end);
	timeval = (timeval_to_ns(end1) - timeval_to_ns(begin));
	timeval1 = (timeval_to_ns(end2) - timeval_to_ns(end1));
	timeval2 = (timeval_to_ns(end) - timeval_to_ns(end2));
	LED_INFO("timeval %ld ns, timeval1 %ld ns, timeval2 %ld ns,finish\n", timeval, timeval1, timeval2);
}

void aw20198_poweron_mode_breath(struct aw_led_dev *led, u32 brightness)
{
	u8 page1_pwm[AW20198_CFG_CNT_PAGE1];
	u32 max_brightness = brightness;
	int i;
	struct timespec64 begin, end1, end2;
	u64 timeval = 0;
	u64 timeval1 = 0;

	ktime_get_real_ts64(&begin);
	for (i = 24; i <= max_brightness; i += 2) {
		msleep(1);
		memset(page1_pwm, i, sizeof(page1_pwm));
		page1_pwm[0] = 0x00;
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_block_write(led, page1_pwm, AW20198_CFG_CNT_PAGE1);
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}
	ktime_get_real_ts64(&end1);

	for (i = max_brightness; i >= 24; i -= 2) {
		msleep(1);
		memset(page1_pwm, i, sizeof(page1_pwm));
		page1_pwm[0] = 0x00;
		aw20198_set_page(led->client, AWPAGE1);
		aw20198_block_write(led, page1_pwm, AW20198_CFG_CNT_PAGE1);
		aw20198_set_page(led->client, AWPAGE0);
		aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	}
	ktime_get_real_ts64(&end2);
	timeval = (timeval_to_ns(end1) - timeval_to_ns(begin));
	timeval1 = (timeval_to_ns(end2) - timeval_to_ns(end1));

	LED_INFO("brightness %d timeval %ld ns, timeval1 %ld ns,finish\n",
		brightness, timeval, timeval1);
}

void aw20198_poweron_mode(struct aw_led_dev *led, u32 brightness)
{
	if (brightness == 0) {
		LED_INFO("brightness is %ld\n", brightness);
		return;
	}
	switch (led->poweron_stage) {
	case PWRON_START:
		aw20198_poweron_mode_start(led);
		led->poweron_stage = PWRON_BREATH;
		fallthrough;
	case PWRON_BREATH:
		aw20198_poweron_mode_breath(led, brightness);
		break;
	default:
		break;
	}
}
bool aw20198_swrst_flag(struct aw_led_dev *led)
{
	return !(test_bit(AW_LED_CHARGE_MODE, &led->effect_bitmap) && (led->charge_stage != CHARGE_START));
}

static void aw20198_brightness_work(struct work_struct *work)
{
	struct aw_led_dev *led = container_of(work, struct aw_led_dev, brightness_work);

	mutex_lock(&aw_led_lock);
	LED_INFO("%s brightness %d led->led_mode %d effect 0x%x\n", led->cdev.name, led->cdev.brightness,
		led->led_mode, led->effect_bitmap);

	if (led->cdev.brightness == 0) {
		aw20198_swrst(led);
		aw20198_clear_deghost(led->client);
		aw20198_hw_disable(led->chip);
		mutex_unlock(&aw_led_lock);
		return;
	} else {
		aw20198_hw_enable(led->chip);
		if (aw20198_swrst_flag(led)) {
			aw20198_swrst(led);
		}
	}

	if (test_bit(AW_LED_POWERON_MODE, &led->effect_bitmap)) {
		wake_up(&led->wq);
	} else if (test_bit(AW_LED_INCALL_MODE, &led->effect_bitmap)) {
		wake_up(&led->wq);
	} else if (test_bit(AW_LED_NOTIFY_MODE, &led->effect_bitmap)) {
		aw20198_notify_mode(led, led->cdev.brightness);
	} else if (test_bit(AW_LED_CHARGE_MODE, &led->effect_bitmap)) {
		wake_up(&led->wq);
	} else if (test_bit(AW_LED_CC_MODE, &led->effect_bitmap)) {
		aw20198_cc_mode(led);
	} else if (test_bit(AW_LED_BREATHE_MODE, &led->effect_bitmap)) {
		wake_up(&led->wq);
	} else if (test_bit(AW_LED_MARQUEE_MODE, &led->effect_bitmap)) {
		wake_up(&led->wq);
	}

	mutex_unlock(&aw_led_lock);
}

static void aw20198_set_brightness(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	if(strcmp(led->cdev.name, LED_CDEV_NAME)) {
		LED_INFO("led->cdev.name %s not match white", led->cdev.name);
		return;
	}

	led->cdev.brightness = brightness;

	if (!brightness)
		clear_bit(led->led_mode, &led->effect_bitmap);

	LED_INFO("tgid %d pid %d %s brightness %d led->led_mode %d\n", current->tgid, current->pid,
		led->cdev.name, led->cdev.brightness, led->led_mode);
	schedule_work(&led->brightness_work);
}

static void aw20198_rgbblink_cfg(struct aw_led_dev *led, unsigned int *databuf)
{
	/* enter page0 */
	aw20198_set_page(led->client, AWPAGE0);
	/* set PWMH0/PWML0 */
	led->max_rgbblink = (databuf[1] & 0x0000ff00) >> 8;
	aw20198_i2c_write(led->client, REG_PWMH0, led->max_rgbblink);
	led->min_rgbblink = (databuf[1] & 0x000000ff);
	aw20198_i2c_write(led->client, REG_PWML0, led->min_rgbblink);
	/* set rise/hold/fall/off time */
	led->time_rise_hold = (databuf[2] & 0x0000ff00) >> 8;
	aw20198_i2c_write(led->client, REG_PAT0T0, led->time_rise_hold);
	led->time_fall_off = (databuf[2] & 0x000000ff);
	aw20198_i2c_write(led->client, REG_PAT0T1, led->time_fall_off);
	/* set chip enable */
	aw20198_i2c_write_bit(led->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	/* set auto breath mode */
	aw20198_i2c_write_bit(led->client, REG_PAT0CFG, BIT_PATMD_MANUAL, BIT_PATMD_AUTO);
	/* enable auto breath */
	aw20198_i2c_write_bit(led->client, REG_PAT0CFG, BIT_PATEN_DISABLE, BIT_PATEN_ENABLE);
	/* run clear */
	aw20198_i2c_write_bit(led->client, REG_PATGO, BIT_RUN0_DISABLE, BIT_RUN0_DISABLE);
	/* run auto breath */
	aw20198_i2c_write_bit(led->client, REG_PATGO, BIT_RUN0_DISABLE, BIT_RUN0_ENABLE);
}


/*******************************************************************************
 *
 * sysfs attribute group: allrgbblink store
 *
 ******************************************************************************/

static ssize_t
allrgbblink_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *aw20198 = container_of(led_cdev, struct aw_led_dev, cdev);
	unsigned char reg_page3_pwm[AW20198_CFG_CNT_PAGE3];
	unsigned int databuf[3] = { 0, 0, 0 };

	/* enter page 3 */
	aw20198_set_page(aw20198->client, AWPAGE3);

	if (sscanf(buf, "%x %x %x", &databuf[0], &databuf[1], &databuf[2]) == 3) {
		/* set rgb blink value */
		aw20198->rgb_color = (databuf[0] & 0x000000ff);
		memset(reg_page3_pwm, aw20198->rgb_color, sizeof(reg_page3_pwm));
		/* base address */
		reg_page3_pwm[0] = 0x00;
		aw20198_block_write(aw20198, reg_page3_pwm, AW20198_CFG_CNT_PAGE3);
		/* blink parameter configuration */
		aw20198_rgbblink_cfg(aw20198, databuf);
	} else {
		dev_err(aw20198->dev,
			"%s: input data invalid!", __func__);
		return -EAGAIN;
	}
	LED_INFO("%s\n", __func__);
	return len;
}

/*******************************************************************************
 *
 * sysfs attribute group: onergbblink store
 *
 ******************************************************************************/

static ssize_t
onergbblink_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *aw20198 = container_of(led_cdev, struct aw_led_dev, cdev);
	unsigned int databuf[3] = { 0, 0, 0 };

	if (sscanf(buf, "%x %x %x", &databuf[0], &databuf[1], &databuf[2]) == 3) {
		/* enter page 3 */
		aw20198_set_page(aw20198->client, AWPAGE3);
		/* select rgb and color */
		aw20198->rgb_num = (databuf[0] & 0x0000ff00) >> 8;
		aw20198->rgb_color = (databuf[0] & 0x000000ff);
		if (aw20198->rgb_num <= AW20198_RGB_NUM) {
			aw20198_i2c_write(aw20198->client, aw20198->rgb_num, aw20198->rgb_color);
			/* blink parameter configuration */
			aw20198_rgbblink_cfg(aw20198, databuf);
		} else {
			dev_err(aw20198->dev, "%s: rgb number invalid!", __func__);
			return -EAGAIN;
		}
	} else {
		dev_err(aw20198->dev, "%s: input data invalid!", __func__);
		return -EAGAIN;
	}
	LED_INFO("%s\n", __func__);

	return len;
}

/*******************************************************************************
 *
 * sysfs attribute group: allrgbbrightness store
 *
 ******************************************************************************/

static ssize_t allrgbbrightness_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *aw20198 = container_of(led_cdev, struct aw_led_dev, cdev);

	int led_num = 0;
	unsigned char reg_page1_pwm[AW20198_CFG_CNT_PAGE1];
	unsigned int databuf[1] = { 0 };
	int ret = -1;

	ret = kstrtou32(buf, 0, &databuf[0]);
	if (ret < 0) {
		dev_err(aw20198->dev, "%s: input data invalid!", __func__);
		return ret;
	}

	/* enter page 1 */
	aw20198_set_page(aw20198->client, AWPAGE1);
	/* base address */
	reg_page1_pwm[0] = 0x00;

	for (led_num = 1; led_num < AW20198_CFG_CNT_PAGE1; led_num += 3) {
		reg_page1_pwm[led_num] = (databuf[0] & 0x00ff0000) >> 16;
		reg_page1_pwm[led_num + 1] = (databuf[0] & 0x0000ff00) >> 8;
		reg_page1_pwm[led_num + 2] = (databuf[0] & 0x000000ff);
	}
	/* set all pwm value */
	aw20198_block_write(aw20198, reg_page1_pwm, AW20198_CFG_CNT_PAGE1);
	/* set chip enable */
	aw20198_set_page(aw20198->client, AWPAGE0);
	aw20198_i2c_write_bit(aw20198->client, REG_GCR, BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
	LED_INFO("%s: databuf[0] = 0x%x\n", __func__, databuf[0]);
	return len;
}

/*******************************************************************************
 *
 * sysfs attribute group: onerrgbbrightness store
 *
 ******************************************************************************/

static ssize_t
onergbbrightness_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *aw20198 = container_of(led_cdev, struct aw_led_dev, cdev);

	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		/* enter page 1 */
		aw20198_set_page(aw20198->client, AWPAGE1);
		if (databuf[0] <= AW20198_LED_NUM) {
			aw20198->rgbbrightness = databuf[1];
			aw20198_i2c_write(aw20198->client, databuf[0], aw20198->rgbbrightness);
			/* enter page 0 */
			aw20198_set_page(aw20198->client, AWPAGE0);
			/* set chip enable */
			aw20198_i2c_write_bit(aw20198->client, REG_GCR,
						BIT_CHIPEN_DISABLE, BIT_CHIPEN_ENABLE);
		} else {
			dev_err(aw20198->dev, "%s: rgb number invalid!", __func__);
			return -EAGAIN;
		}
	} else {
		dev_err(aw20198->dev, "%s: input data invalid!", __func__);
		return -EAGAIN;
	}
	LED_INFO("%s: databuf[0] = 0x%x\n", __func__, databuf[0]);
	return len;
}

/*******************************************************************************
 *
 * sysfs attribute group: reg store/show
 *
 ******************************************************************************/

static ssize_t reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *aw20198 = container_of(led_cdev, struct aw_led_dev, cdev);

	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	/* enter page 0 */
	aw20198_set_page(aw20198->client, AWPAGE0);
	for (i = 0; i < AW20198_REG_PAGE0_MAX; i++) {
		if (!(aw20198_reg_page0_access[i] & REG_RD_ACCESS))
			continue;
		aw20198_i2c_read(aw20198->client, i, &reg_val);
		len +=
		snprintf(buf + len, PAGE_SIZE - len, "PAGE0 reg: 0x%02x = 0x%02x\n", i, reg_val);
	}

	return len;
}

static ssize_t
reg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *aw20198 = container_of(led_cdev, struct aw_led_dev, cdev);

	unsigned int databuf[3] = { 0, 0, 0 };

	if (sscanf(buf, "%x %x %x", &databuf[0], &databuf[1], &databuf[2]) == 3) {
		/* select page */
		aw20198_set_page(aw20198->client, databuf[0]);
		/* write value in address */
		aw20198_i2c_write(aw20198->client, databuf[1], databuf[2]);
		LED_INFO("write reg, databuf 0x%x 0x%x 0x%x\n", databuf[0], databuf[1], databuf[2]);
	} else {
		dev_err(aw20198->dev, "%s: input reg data format err\n", __func__);
	}

	return len;
}

/*******************************************************************************
 *
 * sysfs attribute group: soft reset store/show
 *
 ******************************************************************************/
static ssize_t
swrst_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *aw20198 = container_of(led_cdev, struct aw_led_dev, cdev);

	aw20198_swrst(aw20198);

	return len;
}

static ssize_t
hwen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *led = container_of(led_cdev, struct aw_led_dev, cdev);
	unsigned int databuf[1] = { 0 };
	int ret = -1;

	ret = kstrtou32(buf, 0, &databuf[0]);
	if (ret < 0) {
		dev_err(led->dev, "%s: input data invalid!", __func__);
		return ret;
	}

	if (databuf[0] == 1) {
		aw20198_hw_enable(led->chip);
		gpio_set_value_cansleep(led->chip->vcc_enable_gpio, 1);
		LED_INFO("%s: hw enable complete\n", __func__);
	} else {
		aw20198_hw_disable(led->chip);
		gpio_set_value_cansleep(led->chip->vcc_enable_gpio, 0);
		LED_INFO("%s: hw off complete\n", __func__);
	}

	return len;
}

static ssize_t
hwen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *led = container_of(led_cdev, struct aw_led_dev, cdev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "hwen = %d\n", gpio_get_value(led->chip->enable_gpio));
	len += snprintf(buf + len, PAGE_SIZE - len, "vcc hwen = %d\n", gpio_get_value(led->chip->vcc_enable_gpio));

	LED_INFO("led->cdev.name :%s\n", led->cdev.name);
	return len;
}

static ssize_t support_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *led = container_of(led_cdev, struct aw_led_dev, cdev);

	return snprintf(buf, PAGE_SIZE, "%s-%d-white-marquee\n",LED_SUPPORT_TYPE,led->cdev.max_brightness);
}

static ssize_t breathe_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *led = container_of(led_cdev, struct aw_led_dev, cdev);

	unsigned int databuf[4] = { 0 };

	if (sscanf(buf, "%x %x %x %x", &databuf[0], &databuf[1], &databuf[2], &databuf[3]) != 4) {
		LED_INFO("invaild arguments\n");
		return len;
	}

	mutex_lock(&aw_led_lock);
	aw20198_hw_enable(led->chip);
	aw20198_swrst(led);
	led->breath.t0_rise = databuf[0];
	led->breath.t1_high = databuf[1];
	led->breath.t2_fall = databuf[2];
	led->breath.t3_low = databuf[3];
	led->breath.max_pwm = 0xff;
	aw20198_breathe_mode(led);
	mutex_unlock(&aw_led_lock);

	return len;
}

static ssize_t marquee_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *led = container_of(led_cdev, struct aw_led_dev, cdev);
	unsigned int databuf[4] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) != 1) {
		LED_INFO("invaild arguments\n");
		return len;
	}

	mutex_lock(&aw_led_lock);
	aw20198_hw_enable(led->chip);
	aw20198_swrst(led);
	aw20198_marquee_mode(led, databuf[0]);
	mutex_unlock(&aw_led_lock);

	LED_INFO("start marquee %d\n", databuf[0]);

	return len;
}

int aw20198_self_detection(struct aw_led_dev *led, enum aw_led_detect type, bool *status)
{
	u8 data = 0;
	int ret = 0;
	int i;

	aw20198_set_page(led->client, AWPAGE0);

	if (type == OPEN_DETECT)
		aw20198_i2c_write_bit(led->client, REG_GCR, (~(0x11 << 1)), (0x11 << 1));
	else
		aw20198_i2c_write_bit(led->client, REG_GCR, (~(0x11 << 1)), (0x10 << 1));

	for (i = 0; i < 33; i++) {
		ret = aw20198_i2c_read(led->client, REG_OSR0 + i, &data);
		if (ret) {
			LED_ERR("i2c read, ret %d\n", ret);
			goto out;
		}
		if (data & 0x3f) {
			*status = true;
			ret = 0;
			goto out;
		}
	}

	*status = false;
out:
	aw20198_i2c_write_bit(led->client, REG_GCR, (~(0x11 << 1)), (0x0 << 1));

	LED_INFO("type %d status %d\n", type, *status);
	return ret;
}

static ssize_t detect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *led = container_of(led_cdev, struct aw_led_dev, cdev);
	int ret;
	int len = 0;
	bool open_status, short_status;

	mutex_lock(&aw_led_lock);
	aw20198_hw_enable(led->chip);
	aw20198_swrst(led);
	ret = aw20198_self_detection(led, OPEN_DETECT, &open_status);
	if (ret) {
		LED_INFO("open detect failed,ret %d\n", ret);
		goto out;
	}

	ret = aw20198_self_detection(led, SHORT_DETECT, &short_status);
	if (ret) {
		LED_INFO("short detect failed,ret %d\n", ret);
		goto out;
	}

	len += snprintf(buf, PAGE_SIZE, "%s-%s %s-%s\n",
		LED_OPEN_DET_TYPE,
		(open_status ?  LED_DET_FAILED : LED_DET_PASS),
		LED_SHORT_DET_TYPE,
		(short_status ?  LED_DET_FAILED : LED_DET_PASS));

out:
	mutex_unlock(&aw_led_lock);
	return len;
}

static ssize_t reg_page1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *aw20198 = container_of(led_cdev, struct aw_led_dev, cdev);

	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	mutex_lock(&aw_led_lock);
	aw20198_set_page(aw20198->client, AWPAGE1);
	for (i = 0; i < 120; i++) {
		aw20198_i2c_read(aw20198->client, led_addr[i], &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "PAGE1 reg: %03d = 0x%02x\n", led_addr[i], reg_val);
	}
	mutex_unlock(&aw_led_lock);

	return len;
}

static ssize_t light_sensor_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *led = container_of(led_cdev, struct aw_led_dev, cdev);

	u32 value;

	if (sscanf(buf, "%d", &value) != 1) {
		LED_INFO("invaild arguments\n");
		return len;
	}
	led->chip->light_sensor_state = value;

	if (led->chip->light_sensor_state) {
		aw20198_i2c_write(led->client, REG_GCCR, LED_GCCR_MAX_VAL);
	} else {
		aw20198_i2c_write(led->client, REG_GCCR, LED_GCCR_MIN_VAL);
	}

	LED_INFO("light_sensor value %d\n", value);

	return len;
}

static ssize_t light_sensor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw_led_dev *led = container_of(led_cdev, struct aw_led_dev, cdev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "light_sensor: %d\n", led->chip->light_sensor_state);

	return len;
}

static DEVICE_ATTR_RO(support);
static DEVICE_ATTR_WO(allrgbblink);
static DEVICE_ATTR_WO(onergbblink);
static DEVICE_ATTR_WO(allrgbbrightness);
static DEVICE_ATTR_WO(onergbbrightness);
static DEVICE_ATTR_RW(reg);
static DEVICE_ATTR_WO(swrst);
static DEVICE_ATTR_RW(hwen);
static DEVICE_ATTR_WO(breathe);
static DEVICE_ATTR_WO(marquee);
static DEVICE_ATTR_RO(detect);
static DEVICE_ATTR_RO(reg_page1);
static DEVICE_ATTR_RW(light_sensor);

static struct attribute *aw20198_led_attributes[] = {
	&dev_attr_support.attr,
	&dev_attr_reg.attr,
	&dev_attr_hwen.attr,
	&dev_attr_swrst.attr,
	&dev_attr_onergbbrightness.attr,
	&dev_attr_allrgbbrightness.attr,
	&dev_attr_onergbblink.attr,
	&dev_attr_allrgbblink.attr,
	&dev_attr_breathe.attr,
	&dev_attr_marquee.attr,
	&dev_attr_detect.attr,
	&dev_attr_reg_page1.attr,
	&dev_attr_light_sensor.attr,
	NULL
};

static struct attribute_group aw20198_attribute_group = {
	.attrs = aw20198_led_attributes
};


/*******************************************************************************
 *
 * read chip id
 *
 ******************************************************************************/

static int aw20198_read_chipid(struct aw_led_chip *chip)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg_val = 0;

	/* hardware enable */
	aw20198_hw_enable(chip);

	while (cnt < AW20198_READ_CHIPID_RETRIES) {
		ret = aw20198_i2c_read(chip->client, REG_RSTN, &reg_val);
		LED_INFO("AW20198 chip id is %0x addr 0x%x ret %d\n", reg_val, chip->client->addr, ret);
		if (reg_val == AW20198_CHIPID) {
			LED_INFO("read aw20198 chipid successful\n");
			goto out;
		}

		cnt++;
		usleep_range(1000, 1500);
	}

	dev_err(chip->dev, "read aw20198 id failed, err=%d\n", ret);
out:
	return ret;
}

static int aw20198_led_change_mode(struct aw_led_dev *led, enum AW_LED_MODE mode)
{
	if(strcmp(led->cdev.name, LED_CDEV_NAME)) {
		LED_INFO("led->cdev.name %s not match white", led->cdev.name);
		return -1;
	}

	switch(mode) {
		case AW_LED_BREATHE_MODE:
			led->breath.t0_rise = 6;
			led->breath.t1_high =  0;
			led->breath.t2_fall = 12;
			led->breath.t3_low = 0;
			led->breath.loop_times = 0; /* forever */
			led->breath.max_pwm = 0xff;
			led->breath.min_pwm = 0;
			led->breath.single_led = 0;
			break;
		default:
			break;
	}
	LED_INFO("led mode %d set", mode);
	led->led_mode = mode;
	set_bit(mode, &led->effect_bitmap);
	return 0;
}

static int	aw20198_led_cc_activate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	aw20198_led_change_mode(led, AW_LED_CC_MODE);

	LED_INFO("[%s]: cc_mode",led->cdev.name);
	return 0;
}

static void  aw20198_led_cc_deactivate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	clear_bit(AW_LED_CC_MODE, &led->effect_bitmap);

	LED_INFO("[%s]: cc_mode",led->cdev.name);
}

static int	aw20198_led_blink_activate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	aw20198_led_change_mode(led, AW_LED_BLINK_MODE);

	LED_INFO("%s:blink_mode",led->cdev.name);
	return 0;
}

static void  aw20198_led_blink_deactivate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	clear_bit(AW_LED_BLINK_MODE, &led->effect_bitmap);

	LED_INFO("[%s]: blink_mode",led->cdev.name);
}


static int aw20198_led_breath_activate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	aw20198_led_change_mode(led, AW_LED_BREATHE_MODE);

	LED_INFO("[%s]: breath_mode",led->cdev.name);
	return 0;
}

static void  aw20198_led_breath_deactivate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	clear_bit(AW_LED_BREATHE_MODE, &led->effect_bitmap);

	LED_INFO("[%s]: breath_mode",led->cdev.name);
}

static int aw20198_led_marquee_activate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	aw20198_led_change_mode(led, AW_LED_MARQUEE_MODE);

	LED_INFO("[%s]: marquee_mode",led->cdev.name);
	return 0;
}

static void  aw20198_led_marquee_deactivate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	clear_bit(AW_LED_MARQUEE_MODE, &led->effect_bitmap);

	LED_INFO("[%s]:marquee_mode ",led->cdev.name);
}

static int aw20198_led_call_activate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	aw20198_led_change_mode(led, AW_LED_INCALL_MODE);

	LED_INFO("[%s]: call_mode",led->cdev.name);
	return 0;
}
static void  aw20198_led_call_deactivate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	clear_bit(AW_LED_INCALL_MODE, &led->effect_bitmap);

	LED_INFO("[%s]: call_mode",led->cdev.name);
}

static int aw20198_led_notify_activate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	aw20198_led_change_mode(led, AW_LED_NOTIFY_MODE);

	LED_INFO("[%s]: notify_mode",led->cdev.name);
	return 0;
}
static void  aw20198_led_notify_deactivate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	clear_bit(AW_LED_NOTIFY_MODE, &led->effect_bitmap);

	LED_INFO("[%s]: notify_mode",led->cdev.name);
}

static int aw20198_led_charge_activate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	aw20198_led_change_mode(led, AW_LED_CHARGE_MODE);
	led->charge_stage = CHARGE_START;

	LED_INFO("[%s]: charge_mode stage %d",led->cdev.name, led->charge_stage);
	return 0;
}
static void  aw20198_led_charge_deactivate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	clear_bit(AW_LED_CHARGE_MODE, &led->effect_bitmap);
	led->charge_stage = CHARGE_START;
	LED_INFO("[%s]: charge_mode",led->cdev.name);
}

static int aw20198_led_poweron_activate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	aw20198_led_change_mode(led, AW_LED_POWERON_MODE);
	led->poweron_stage = PWRON_START;

	LED_INFO("[%s]: poweron_mode stage %d",led->cdev.name, led->poweron_stage);
	return 0;
}
static void  aw20198_led_poweron_deactivate(struct led_classdev *cdev)
{
	struct aw_led_dev *led = container_of(cdev, struct aw_led_dev, cdev);

	clear_bit(AW_LED_POWERON_MODE, &led->effect_bitmap);
	led->poweron_stage = PWRON_START;
	LED_INFO("[%s]: poweron_mode",led->cdev.name);
}

struct led_trigger aw20198_led_trigger[AW_LED_MAX] = {
	{
		.name = "cc_mode",
		.activate = aw20198_led_cc_activate,
		.deactivate = aw20198_led_cc_deactivate,
	},
	{
		.name = "blink_mode",
		.activate = aw20198_led_blink_activate,
		.deactivate = aw20198_led_blink_deactivate,
	},
	{
		.name = "breath_mode",
		.activate = aw20198_led_breath_activate,
		.deactivate = aw20198_led_breath_deactivate,
	},
	{
		.name = "marquee_mode",
		.activate = aw20198_led_marquee_activate,
		.deactivate = aw20198_led_marquee_deactivate,
	},
	{
		.name = "incall_mode",
		.activate = aw20198_led_call_activate,
		.deactivate = aw20198_led_call_deactivate,
	},
	{
		.name = "notify_mode",
		.activate = aw20198_led_notify_activate,
		.deactivate = aw20198_led_notify_deactivate,
	},
	{
		.name = "charge_mode",
		.activate = aw20198_led_charge_activate,
		.deactivate = aw20198_led_charge_deactivate,
	},
	{
		.name = "poweron_mode",
		.activate = aw20198_led_poweron_activate,
		.deactivate = aw20198_led_poweron_deactivate,
	},
};


/*******************************************************************************
 *
 * parse device tree
 *
 ******************************************************************************/

static int aw20198_parse_dts(struct aw_led_chip *chip, struct device_node *np)
{
	int ret = -1;
	int i = 0;
	struct device_node *child_node;
	struct aw_led_dev *led;

	chip->enable_gpio = of_get_named_gpio(np, "enable_gpio", 0);
	if(!gpio_is_valid(chip->enable_gpio)) {
		dev_err(chip->dev, " enable_gpio err, ret = %d\n",  chip->enable_gpio);
		return ret;
	}

	chip->vcc_enable_gpio = of_get_named_gpio(np, "vcc-enable_gpio", 0);
	if(!gpio_is_valid(chip->vcc_enable_gpio)) {
		dev_err(chip->dev, " vcc_enable_gpio err, ret = %d\n",  chip->vcc_enable_gpio);
		return ret;
	}

	ret = of_property_read_u32(np, "aw20198,imax", &chip->imax);
	if (ret < 0) {
		dev_err(chip->dev, "%s: parse imax err, ret = %d\n", __func__, ret);
		return ret;
	}
	LED_INFO("%s: led imax = 0x%x\n", __func__, chip->imax);

	ret = of_property_read_u32(np, "aw20198,sl_current", &chip->sl_current);
	if (ret < 0) {
		dev_err(chip->dev, "%s: parse sl_current err, ret = %d\n", __func__, ret);
		return ret;
	}
	LED_INFO("%s: led sl_current = 0x%x\n", __func__, chip->sl_current);

	chip->effect_bin = of_property_read_bool(np, "aw20198,effect-bin");
	if (chip->effect_bin)
		LED_INFO("%s: led effect use bin\n", __func__);

	chip->num_leds = of_get_available_child_count(np);
	if (chip->num_leds == 0) {
		dev_err(chip->dev, "No led child node defined\n");
		return -ENODEV;
	}

	if (chip->num_leds > LED_DEV_NUM_MAX) {
		dev_err(chip->dev, "can't support %d leds(max %d)\n",
				chip->num_leds, LED_DEV_NUM_MAX);
		return -EINVAL;
	}

	chip->leds = devm_kcalloc(chip->dev, chip->num_leds,
			sizeof(struct aw_led_dev), GFP_KERNEL);
	if (!chip->leds)
		return -ENOMEM;

	for_each_available_child_of_node(np, child_node) {
		led = &chip->leds[i++];
		led->id = i;
		led->chip = chip;
		led->client = chip->client;
		led->dev = chip->dev;
		led->breath.max_pwm = 0xff;

		ret = of_property_read_string(child_node, "aw210xx,name",
			&led->cdev.name);
		if (ret < 0) {
			dev_err(&led->client->dev,
				"Failure reading led name, rc = %d\n", ret);
			return ret;
		}
		ret = of_property_read_u32(child_node, "aw210xx,brightness",
			&led->cdev.brightness);
		if (ret < 0) {
			dev_err(&led->client->dev,
				"Failure reading brightness, rc = %d\n", ret);
			return ret;
		}

		ret = of_property_read_u32(child_node, "aw210xx,max-brightness",
			&led->cdev.max_brightness);
		if (ret < 0) {
			dev_err(&led->client->dev,
				"Failure reading max-brightness, rc = %d\n",
				ret);
			return ret;
		}
	}
	return 0;
}

bool aw20198_thread_wait_flag(struct aw_led_dev *led)
{
	return (test_bit(AW_LED_MARQUEE_MODE, &led->effect_bitmap) ||
			test_bit(AW_LED_CHARGE_MODE, &led->effect_bitmap) ||
			test_bit(AW_LED_INCALL_MODE, &led->effect_bitmap) ||
			test_bit(AW_LED_POWERON_MODE, &led->effect_bitmap) ||
			test_bit(AW_LED_BREATHE_MODE, &led->effect_bitmap));
}

static int aw20198_led_effect_thread(void *arg)
{
	int ret = 0;
	struct aw_led_dev *led = (struct aw_led_dev *)arg;

	while (!kthread_should_stop()) {

		LED_INFO("%s brightness %d, effect_bitmap 0x%x\n", led->cdev.name,
			led->cdev.brightness, led->effect_bitmap);
		if (!aw20198_thread_wait_flag(led) || !led->cdev.brightness) {
			LED_INFO("%s wait_event_interruptible\n", led->cdev.name);
			wait_event_interruptible(led->wq, aw20198_thread_wait_flag(led) && led->cdev.brightness);
		}

		mutex_lock(&aw_led_lock);

		if (test_bit(AW_LED_POWERON_MODE, &led->effect_bitmap)) {
			aw20198_poweron_mode(led, led->cdev.brightness);
		} else if (test_bit(AW_LED_INCALL_MODE, &led->effect_bitmap)) {
			aw20198_incall_mode(led, led->cdev.brightness);
		} else if (test_bit(AW_LED_CHARGE_MODE, &led->effect_bitmap)) {
			aw20198_charge_mode(led, led->cdev.brightness);
		} else if (test_bit(AW_LED_MARQUEE_MODE, &led->effect_bitmap)) {
			aw20198_marquee_mode(led, led->cdev.brightness);
		} else if (test_bit(AW_LED_POWERON_MODE, &led->effect_bitmap)) {
			aw20198_poweron_mode(led, led->cdev.brightness);
		} else if (test_bit(AW_LED_BREATHE_MODE, &led->effect_bitmap)) {
			aw20198_breathe_mode(led);
		}

		mutex_unlock(&aw_led_lock);
		msleep(5);
	}

	return ret;
}

static int aw20198_leds_register(struct aw_led_chip *chip)
{
	struct aw_led_dev *led;
	int i, j;
	int ret;

	for (i = 0; i < chip->num_leds; i++) {
		led = &chip->leds[i];

		INIT_WORK(&led->brightness_work, aw20198_brightness_work);
		led->cdev.brightness_set = aw20198_set_brightness;

		ret = devm_led_classdev_register(led->dev, &led->cdev);
		if (ret) {
			dev_err(led->dev, "%s: classdev register failed, ret = %d\n", __func__, ret);
			goto err_out;
		}

		ret = sysfs_create_group(&led->cdev.dev->kobj, &aw20198_attribute_group);
		if (ret) {
			dev_err(led->dev, "%s: sysfs creat group failed, ret = %d\n", __func__, ret);
			goto err_out;
		}

		LED_INFO("%s led register success\n", led->cdev.name);
	}

	return 0;
err_out:
	for (j = 0; j < i; j++) {
		sysfs_remove_group(&chip->leds[j].cdev.dev->kobj, &aw20198_attribute_group);
	}
	return ret;
}

int aw20198_leds_trigger_register(struct aw_led_chip *chip)
{
	int i, j;
	int ret;

	for (i = 0; i < ARRAY_SIZE(aw20198_led_trigger); i++) {
		ret = led_trigger_register(&aw20198_led_trigger[i]);
		if (ret < 0) {
			dev_err(chip->dev, "register %d trigger fail\n", i);
			goto err_out;
		}
	}

	return 0;
err_out:
	for (j = 0; j < i; j++) {
		led_trigger_unregister(&aw20198_led_trigger[i]);
	}
	return ret;
}

static int aw20198_leds_kthread_run(struct aw_led_chip *chip)
{
	struct aw_led_dev *led;
	int i;

	for (i = 0; i < chip->num_leds; i++) {
		led = &chip->leds[i];

		init_waitqueue_head(&led->wq);
		led->effect_task = kthread_run(aw20198_led_effect_thread, (void *)led,
			"aw_%s_led_effect", led->cdev.name);
		if (IS_ERR(led->effect_task)) {
			LED_INFO("%s led kthread_run failed\n", led->cdev.name);
			return -1;
		}
	}

	return 0;
}


int aw20198_gpio_init(struct aw_led_chip *chip)
{
	int ret = 0;


	ret = devm_gpio_request_one(&chip->client->dev,
					chip->enable_gpio,
					GPIOF_OUT_INIT_LOW,
					"aw20198_enable_gpio");
	if (ret) {
		dev_err(chip->dev, "%s: gpio request failed\n", __func__);
		return ret;
	}
	LED_INFO("%s: enable_gpio set high\n", __func__);

	ret = devm_gpio_request_one(&chip->client->dev,
					chip->vcc_enable_gpio,
					GPIOF_OUT_INIT_LOW,
					"aw20198_enable_gpio");
	if (ret) {
		dev_err(chip->dev, "%s: gpio request failed\n", __func__);
		return ret;
	}

	LED_INFO("vcc_enable_gpio set high\n");

	return ret;
}

/*******************************************************************************
 *
 * i2c driver probe
 *
 ******************************************************************************/

static int aw20198_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct aw_led_chip *chip;
	struct device_node *np = client->dev.of_node;
	int ret = -1;
	int i;

	LED_INFO("probe in\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: check i2c error\n", __func__);
		return -ENODEV;
	}

	chip = devm_kzalloc(&client->dev, sizeof(struct aw_led_chip), GFP_KERNEL);
	if (chip == NULL) {
		ret = -ENOMEM;
		goto err_devm_kzalloc;
	}

	chip->client = client;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

	/* parse device tree */
	if (np) {
		ret = aw20198_parse_dts(chip, np);
		if (ret) {
			dev_err(&client->dev, "%s: parse dts failed\n", __func__);
			goto err_parse_dts;
		} else {
			LED_INFO("%s: parse dts successful\n", __func__);
		}
	} else {
		dev_err(&client->dev, "%s: np is NULL\n", __func__);
		goto err_parse_dts;
	}

	ret = aw20198_gpio_init(chip);
	if (ret) {
		goto err_parse_dts;
	}

	/* read chip id */
	ret = aw20198_read_chipid(chip);
	if (ret < 0) {
		dev_err(&client->dev, "%s: read chipid error\n", __func__);
		aw20198_hw_disable(chip);
		goto err_parse_dts;
	}

	chip->light_sensor_state = 1;
	aw20198_led_init(chip);
	aw20198_clear_deghost(chip->client);
	aw20198_hw_disable(chip);

	ret = aw20198_leds_register(chip);
	if (ret) {
		LED_ERR("classdev register failed, ret = %d\n", ret);
		goto err_parse_dts;
	}

	ret = aw20198_leds_trigger_register(chip);
	if (ret) {
		LED_ERR("classdev trigger set failed, ret = %d\n", ret);
		goto err_led_register;
	}

	ret = aw20198_leds_kthread_run(chip);
	if (ret) {
		LED_ERR("thread run failed, ret = %d\n", ret);
		goto err_led_trigger;
	}

	LED_INFO("probe successful\n");
	return 0;

err_led_trigger:
	for (i = 0; i < ARRAY_SIZE(aw20198_led_trigger); i++) {
		led_trigger_unregister(&aw20198_led_trigger[i]);
	}
err_led_register:
	for (i = 0; i < chip->num_leds; i++) {
		sysfs_remove_group(&chip->leds[i].cdev.dev->kobj, &aw20198_attribute_group);
	}
err_parse_dts:
	devm_kfree(&client->dev, chip);
err_devm_kzalloc:
	return ret;
}

/*******************************************************************************
 *
 * i2c driver remove
 *
 ******************************************************************************/

static void aw20198_i2c_remove(struct i2c_client *client)
{
	struct aw_led_chip *aw20198 = i2c_get_clientdata(client);

	/*if (gpio_is_valid(aw20198->enable_gpio))
		devm_gpio_free(&client->dev, aw20198->enable_gpio);*/

	devm_kfree(&client->dev, aw20198);

}

static const struct i2c_device_id aw20198_i2c_id[] = {
	{AW20198_I2C_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, aw20198_i2c_id);

static const struct of_device_id aw20198_dt_match[] = {
	{.compatible = "awinic,aw20198_led"},
	{},
};

static struct i2c_driver aw20198_i2c_driver = {
	.driver = {
		.name = AW20198_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw20198_dt_match),
	},
	.probe = aw20198_i2c_probe,
	.remove = aw20198_i2c_remove,
	.id_table = aw20198_i2c_id,
};

static int __init aw20198_i2c_init(void)
{
	int ret = -1;

	ret = i2c_add_driver(&aw20198_i2c_driver);
	if (ret) {
		pr_err("add aw20198 driver failed\n");
		return ret;
	}
	LED_INFO("driver version %s\n", AW20198_DRIVER_VERSION);
	return 0;
}

module_init(aw20198_i2c_init);

static void __exit aw20198_i2c_exit(void)
{
	i2c_del_driver(&aw20198_i2c_driver);
}

module_exit(aw20198_i2c_exit);

MODULE_DESCRIPTION("AW20198 LED Driver");
MODULE_LICENSE("GPL v2");

