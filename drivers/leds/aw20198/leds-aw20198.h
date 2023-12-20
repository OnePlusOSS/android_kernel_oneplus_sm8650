/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __AW20198_H__
#define __AW20198_H__

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/types.h>

/*******************************************************************************
 *
 * struct
 *
 ******************************************************************************/
struct aw_breath_t {
	u32 t0_rise;
	u32 t1_high;
	u32 t2_fall;
	u32 t3_low;
	u32 single_led;
	u32 max_pwm;
	u32 min_pwm;
	u32 loop_times;
};

struct aw_led_dev {
	struct i2c_client *client;
	struct device *dev;
	struct aw_led_chip *chip;
	struct led_classdev cdev;
	struct work_struct brightness_work;
	struct aw_breath_t breath;
	unsigned long effect_bitmap;
	struct task_struct *effect_task;
	wait_queue_head_t wq;
	u32 id;
	u8 rgb_num;
	u8 time_rise_hold;
	u8 time_fall_off;
	u8 max_rgbblink;
	u8 min_rgbblink;
	u8 rgbbrightness;
	u32 designeffect;
	u32 rgb_color;
	u32 led_mode;
	u32 charge_stage;
	u32 poweron_stage;
};


struct aw_led_chip {
	struct i2c_client *client;
	struct device *dev;
	struct mutex lock;
	struct aw_led_dev *leds;
	u32 num_leds;
	int enable_gpio;
	int vcc_enable_gpio;
	u32 pwm_current;
	u32 sl_current;
	u32 imax;
	bool effect_bin;
	u32 light_sensor_state;

};


struct awcfgdata {
	unsigned char *cfg_data;
	unsigned int cfg_size;
};

enum aw_led_detect {
	OPEN_DETECT,
	SHORT_DETECT,
};

enum aw_led_charge_stage {
	CHARGE_START,
	CHARGE_GOING,
	CHARGE_DONE,
	CHARGE_MAX,
};

enum aw_led_poweron_stage {
	PWRON_START,
	PWRON_BREATH,
	PWRON_MAX,
};


/*******************************************************************************
 *
 * register list
 *
 ******************************************************************************/

#define REG_GCR			0x00
#define REG_GCCR		0x01
#define REG_DGCR		0x02
#define REG_OSR0		0x03
#define REG_OTCR		0x27
#define REG_SSCR		0x28
#define REG_PCCR		0x29
#define REG_UVCR		0x2A
#define REG_SRCR		0x2B
#define REG_RSTN		0x2F
#define REG_PWMH0		0x30
#define REG_PWMH1		0x31
#define REG_PWMH2		0x32
#define REG_PWML0		0x33
#define REG_PWML1		0x34
#define REG_PWML2		0x35
#define REG_PAT0T0		0x36
#define REG_PAT0T1		0x37
#define REG_PAT0T2		0x38
#define REG_PAT0T3		0x39
#define REG_PAT1T0		0x3A
#define REG_PAT1T1		0x3B
#define REG_PAT1T2		0x3C
#define REG_PAT1T3		0x3D
#define REG_PAT2T0		0x3E
#define REG_PAT2T1		0x3F
#define REG_PAT2T2		0x40
#define REG_PAT2T3		0x41
#define REG_PAT0CFG		0x42
#define REG_PAT1CFG		0x43
#define REG_PAT2CFG		0x44
#define REG_PATGO		0x45
#define REG_MIXCR		0x46
#define REG_PAGE		0xF0

#define REG_SL0			0x00

/*******************************************************************************
 *
 * register detail
 *
 ******************************************************************************/

#define AW20198_CHIPID		0x71
#define AWREG_SWRST		0xAE

#define BIT_CHIPEN_ENABLE	(1<<0)
#define BIT_CHIPEN_DISABLE	(~(1<<0))
#define BIT_SWITCH_ON_LED	(1<<3)
#define BIT_SWITCH_OFF_LED	(~(1<<3))
#define BIT_PATMD_AUTO		(1<<1)
#define BIT_PATMD_MANUAL	(~(1<<1))
#define BIT_PATEN_ENABLE	(1<<0)
#define BIT_PATEN_DISABLE	(~(1<<0))
#define BIT_RUN0_ENABLE		(1<<0)
#define BIT_RUN0_DISABLE	(~(1<<0))

/*******************************************************************************
 *
 * register page
 *
 ******************************************************************************/

#define AWPAGEADDR		0xF0
#define AWPAGE0			0xC0
#define AWPAGE1			0xC1
#define AWPAGE2			0xC2
#define AWPAGE3			0xC3
#define AWPAGE4			0xC4

#define AW20198_LED_NUM		0xC5
#define AW20198_RGB_NUM		0x41

#define AW20198_CFG_CNT_PAGE1	(0xC5 + 2)
#define AW20198_CFG_CNT_PAGE2	(0xC5 + 2)
#define AW20198_CFG_CNT_PAGE3	(0x41 + 2)
#define AW20198_CFG_CNT_PAGE4	(0xC5 + 2)

/*******************************************************************************
 *
 * register read/write access
 *
 ******************************************************************************/

#define REG_NONE_AEECSS		0
#define REG_RD_ACCESS		(1 << 0)
#define REG_WR_ACCESS		(1 << 1)
#define AW20198_REG_PAGE0_MAX	0xFF
#define AW20198_REG_PAGE1_MAX	0xFF
#define AW20198_REG_PAGE2_MAX	0xFF
#define AW20198_REG_PAGE3_MAX	0xFF

#define LED_NAME "led_aw20198"

#define LED_CDEV_NAME "white"


#define LED_INFO(fmt, args...) \
        pr_info("[%s][%s][%d]:" fmt, \
            LED_NAME, __func__, __LINE__, ##args)

#define LED_WARN(fmt, args...)\
        pr_warn("[%s][%s][%d]:" fmt, \
            LED_NAME, __func__, __LINE__, ##args)

#define LED_ERR(fmt, args...) \
        pr_err("[%s][%s][%d]:" fmt, \
            LED_NAME, __func__, __LINE__, ##args)

#define LED_SUPPORT_TYPE	"support"

#define LED_DEV_NUM_MAX		4
#define LED_OPEN_DET_TYPE	"open"
#define LED_SHORT_DET_TYPE	"short"
#define LED_DET_PASS		"pass"
#define LED_DET_FAILED		"failed"
#define MIN(x, y) ((x < y) ? x : y)

#define timeval_to_ns(time) (time.tv_sec * 1000000000 + time.tv_nsec)

#define AW20198_I2C_NAME		"aw20198_led"
#define AW20198_DRIVER_VERSION		"v0.2.0"
#define AW20198_READ_CHIPID_RETRIES	3
#define AW_I2C_READ_RETRIES		1
#define AW_I2C_WRITE_RETRIES		1
#define AW20198_EFFECT_CNT		5


#define LED_GCCR_MAX_VAL	0x12
#define LED_GCCR_MIN_VAL	0xB

enum AW_LED_MODE{
	AW_LED_CC_MODE = 0,
	AW_LED_BLINK_MODE,
	AW_LED_BREATHE_MODE,
	AW_LED_MARQUEE_MODE,
	AW_LED_INCALL_MODE,
	AW_LED_NOTIFY_MODE,
	AW_LED_CHARGE_MODE,
	AW_LED_POWERON_MODE,
	AW_LED_MAX,
};

#define LEDMODE_MAX_NUM		(4)

const unsigned char aw20198_reg_page0_access[AW20198_REG_PAGE0_MAX] = {
	[REG_GCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_GCCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_DGCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_OSR0]	= REG_RD_ACCESS,
	[REG_OTCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_SSCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PCCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_UVCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_SRCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_RSTN]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PWMH0]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PWMH1]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PWMH2]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PWML0]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PWML1]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PWML2]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT0T0]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT0T1]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT0T2]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT0T3]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT1T0]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT1T1]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT1T2]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT1T3]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT2T0]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT2T1]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT2T2]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT2T3]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT0CFG]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT1CFG]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAT2CFG]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PATGO]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_MIXCR]	= REG_RD_ACCESS | REG_WR_ACCESS,
	[REG_PAGE]	= REG_RD_ACCESS | REG_WR_ACCESS,
};

#endif
