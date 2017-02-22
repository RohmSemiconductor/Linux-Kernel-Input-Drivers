/*
 * kxg03: tri-axis accelerometer / tri-axis gyroscope
 *
 * Copyright(C) 2016 Kionix, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
#define QCOM_SENSORS
#ifdef QCOM_SENSORS
#include <linux/sensors.h>
#endif
#include "kxg03.h"
#include "kxg03_registers.h"

#define DEBUG_PRINT 1

#if DEBUG_PRINT
//#define __debug pr_debug
#define __debug printk
#else
/* no debug prints */
#define __debug(...)
#endif

/* define DEBUG_XYZ_DATA to see sensor data print */
//#define DEBUG_XYZ_DATA

#define KXG03_WHO_AM_I_VAL 0x24

/* input dev names */
#define KXG03_ACCEL_INPUT_DEV "kxg03-accel"
#define KXG03_GYRO_INPUT_DEV "kxg03-gyro"

#define MIN_POLL_RATE_MS 5
#define POLL_MS_25HZ 40
#define POLL_MS_50HZ 20
#define POLL_MS_100HZ 10
#define POLL_MS_200HZ 5

/* Number of times to wait self clearing bit (MAN_WAKE)*/
#define KXG03_MAN_WAKE_RETRY_COUNT	100
/* Timeout between retry */
#define KXG03_MAN_WAKE_RETRY_TIME_US (5*1000)
/* MaxTimeout between retry */
#define KXG03_MAN_WAKE_RETRY_MAXTIME_US (6*1000)

struct kxg03_data {
	struct i2c_client *client;
	struct kxg03_platform_data pdata;
	struct mutex mutex;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	struct pinctrl_state	*pin_sleep;

	/* accelerometer */
	struct input_dev *accel_input_dev;
	struct hrtimer accel_timer;
	int accel_wkp_flag;
	struct task_struct *accel_task;
	bool accel_delay_change;
	wait_queue_head_t accel_wq;
	bool accel_enabled;
	u16 accel_poll_rate;

	/* gyroscope */
	struct input_dev *gyro_input_dev;
	struct hrtimer gyro_timer;
	int gyro_wkp_flag;
	struct task_struct *gyro_task;
	bool gyro_delay_change;
	wait_queue_head_t gyro_wq;
	bool gyro_enabled;
	u16 gyro_poll_rate;
	int gyro_start_delay;

#ifdef QCOM_SENSORS
	struct sensors_classdev accel_cdev;
	struct sensors_classdev gyro_cdev;
#endif

	int irq1;
	int irq2;
	u8 accel_range;
	u8 gyro_range;
	u8 wai;

	bool use_poll_timer;
};

/* accelerometer delay to odr map table */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kxg03_accel_odr_table[] = {
	{ 2, KXG03_ACCEL_ODR_WAKE_ODRA_W_1600},
	{ 3, KXG03_ACCEL_ODR_WAKE_ODRA_W_800},
	{ 5, KXG03_ACCEL_ODR_WAKE_ODRA_W_400},
	{ 10, KXG03_ACCEL_ODR_WAKE_ODRA_W_200},
	{ 20, KXG03_ACCEL_ODR_WAKE_ODRA_W_100},
	{ 40, KXG03_ACCEL_ODR_WAKE_ODRA_W_50},
	{ 80, KXG03_ACCEL_ODR_WAKE_ODRA_W_25},
	{ 160, KXG03_ACCEL_ODR_WAKE_ODRA_W_12P5},
	{ 320, KXG03_ACCEL_ODR_WAKE_ODRA_W_6P25},
	{ 640, KXG03_ACCEL_ODR_WAKE_ODRA_W_3P125},
	{ 1280,	KXG03_ACCEL_ODR_WAKE_ODRA_W_1P563},
	{ 0, 	KXG03_ACCEL_ODR_WAKE_ODRA_W_0P781},};


/* gyro delay to odr map table */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kxg03_gyro_odr_table[] = {
	{ 2, KXG03_GYRO_ODR_WAKE_ODRG_W_1600},
	{ 3, KXG03_GYRO_ODR_WAKE_ODRG_W_800},
	{ 5, KXG03_GYRO_ODR_WAKE_ODRG_W_400},
	{ 10, KXG03_GYRO_ODR_WAKE_ODRG_W_200},
	{ 20, KXG03_GYRO_ODR_WAKE_ODRG_W_100},
	{ 40, KXG03_GYRO_ODR_WAKE_ODRG_W_50},
	{ 80, KXG03_GYRO_ODR_WAKE_ODRG_W_25},
	{ 160, KXG03_GYRO_ODR_WAKE_ODRG_W_12P5},
	{ 320, KXG03_GYRO_ODR_WAKE_ODRG_W_6P25},
	{ 640, KXG03_GYRO_ODR_WAKE_ODRG_W_3P125},
	{ 1280,	KXG03_GYRO_ODR_WAKE_ODRG_W_1P563},
	{ 0, 	KXG03_GYRO_ODR_WAKE_ODRG_W_0P781},};


#ifdef QCOM_SENSORS
/* Qualcomm sensors class defines*/

#define POLL_INTERVAL_MIN_MS MIN_POLL_RATE_MS
#define POLL_DEFAULT_INTERVAL_MS 200

static struct sensors_classdev accel_sensors_cdev = {
	.name = KXG03_ACCEL_INPUT_DEV,
	.vendor = "Kionix",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "78.48",	/* m/s^2 (8g) */
	.resolution = "0.002354",/* m/s^2 (0.00024g) */
	.sensor_power = "0.15",	/* typical value */
	.min_delay = POLL_INTERVAL_MIN_MS * 1000,	/* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = POLL_DEFAULT_INTERVAL_MS,	/* in millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev gyro_sensors_cdev = {
	.name = KXG03_GYRO_INPUT_DEV,
	.vendor = "Kionix",
	.version = 1,
	.handle = SENSORS_GYROSCOPE_HANDLE,
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "17.84",	/* rad/s (1024 deg/s) */
	.resolution = "0.000544267",	/* rad/s (0.0312 deg/s) */
	.sensor_power = "1.85",
	.min_delay = POLL_INTERVAL_MIN_MS * 1000,	/* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = POLL_DEFAULT_INTERVAL_MS,	/* in millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

static int kxg03_accel_poll_thread(void *data);
static int kxg03_gyro_poll_thread(void *data);
static int kxg03_accel_set_delay(struct kxg03_data *sdata, unsigned long delay);
static int kxg03_gyro_set_delay(struct kxg03_data *sdata, unsigned long delay);

static int kxg03_accel_read_xyz(struct kxg03_data *sdata, int *xyz);
static void kxg03_accel_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts);
static int kxg03_gyro_read_xyz(struct kxg03_data *sdata, int *xyz);
static void kxg03_gyro_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts);

/* common part */
static int kxg03_set_reg_bit(struct kxg03_data *sdata, u8 reg, u8 bits)
{
	s32 reg_val;

	reg_val = i2c_smbus_read_byte_data(sdata->client, reg);

	if (reg_val < 0)
		return reg_val;

	reg_val |= bits;

	reg_val = i2c_smbus_write_byte_data(sdata->client, reg, reg_val);

	if (reg_val < 0)
		return reg_val;

	return 0;
}

static int kxg03_reset_reg_bit(struct kxg03_data *sdata, u8 reg, u8 bits)
{
	s32 reg_val;

	reg_val = i2c_smbus_read_byte_data(sdata->client, reg);

	if (reg_val < 0)
		return reg_val;

	reg_val &= ~bits;

	reg_val  = i2c_smbus_write_byte_data(sdata->client, reg, reg_val);

	if (reg_val < 0)
		return reg_val;

	return 0;
}

/* power modes... */
static int kxg03_gyro_set_power_mode_w(struct kxg03_data *sdata, u8 lpmode)
{
	s32 lp_mode_new;
	int reg_val;

	reg_val = i2c_smbus_read_byte_data(sdata->client, KXG03_ACCEL_ODR_WAKE);
	if (reg_val < 0)
		return reg_val;
	/* clear old odr and set new */
	lp_mode_new = reg_val & ~KXG03_ACCEL_ODR_WAKE_LPMODE_W_MASK;
	lp_mode_new |= lpmode;

	/* reg_val used for err checking */
	reg_val = i2c_smbus_write_byte_data(sdata->client,
					KXG03_ACCEL_ODR_WAKE, lp_mode_new);
	if (reg_val < 0)
		return reg_val;

	return 0;
}

/* wake sleep */
static int kxg03_set_wake_sleep(struct kxg03_data *sdata, u8 wake_sleep)
{
	int err;
	int loop = KXG03_MAN_WAKE_RETRY_COUNT;

	err = i2c_smbus_write_byte_data(sdata->client,
		KXG03_WAKE_SLEEP_CTL1, wake_sleep);
	if (err < 0)
		return err;

	/* Force manual wake */
	while(loop) {
		err = i2c_smbus_read_byte_data(sdata->client, KXG03_WAKE_SLEEP_CTL1);
		if (err < 0) {
			__debug("%s error reading wake sleep ctl1= %d\n", __FUNCTION__, err);
			return err;
		}
				
		if(!(err & KXG03_WAKE_SLEEP_CTL1_MAN_WAKE)){
			__debug("%s waiting wake manual= %d\n", __FUNCTION__, err);
			loop--;
			usleep_range(KXG03_MAN_WAKE_RETRY_TIME_US, KXG03_MAN_WAKE_RETRY_MAXTIME_US);
		} else {
			loop = 0;
			__debug("%s wake manual done= %d\n", __FUNCTION__, err);
		}
	}
	/* Check wake mode */
	loop = KXG03_MAN_WAKE_RETRY_COUNT;
	while(loop) {		
		err = i2c_smbus_read_byte_data(sdata->client, KXG03_STATUS1);
		if (err < 0) {
			__debug("%s error reading status1 = %d\n", __FUNCTION__, err);
			return err;
		}
		if(!(err & KXG03_STATUS1_WAKE_SLEEP_WAKE_MODE)){
			__debug("%s waiting wake status mode= %d\n", __FUNCTION__, err);
			loop--;
			usleep_range(KXG03_MAN_WAKE_RETRY_TIME_US, KXG03_MAN_WAKE_RETRY_MAXTIME_US);
		} else {
			loop = 0;
			__debug("%s wake mode status done= %d\n", __FUNCTION__, err);
		}
	}
	return 0;
}

static int kxg03_set_acc_odr(struct kxg03_data *sdata, u8 odr, u8 odr_mask)
{
	s32 new_odr;
	int err;

	err = i2c_smbus_read_byte_data(sdata->client, KXG03_ACCEL_ODR_WAKE);
	if (err < 0)
		return err;
	/* clear old odr and set new */
	new_odr = err & ~odr_mask;
	new_odr |= odr;

	err = i2c_smbus_write_byte_data(sdata->client,
						KXG03_ACCEL_ODR_WAKE, new_odr);
	if (err < 0)
		return err;

	return 0;
}

static int kxg03_set_gyro_odr(struct kxg03_data *sdata, u8 odr, u8 odr_mask)
{
	int err;
	u8 new_odr;

	err = i2c_smbus_read_byte_data(sdata->client, KXG03_GYRO_ODR_WAKE);
	if (err < 0)
		return err;

	/* clear old odr and set new */
	new_odr = err & ~odr_mask;
	new_odr |= odr;
	err = i2c_smbus_write_byte_data(sdata->client,
						KXG03_GYRO_ODR_WAKE, new_odr);

	return err;
}

/* Accelerometer granges */
static int kxg03_accel_set_grange_w(struct kxg03_data *sdata, u8 new_range)
{
	s32 ctl_reg;

	if ((new_range != KXG03_ACCEL_CTL_ACC_FS_W_2G) &&
		(new_range != KXG03_ACCEL_CTL_ACC_FS_W_4G) &&
		(new_range != KXG03_ACCEL_CTL_ACC_FS_W_8G) &&
		(new_range != KXG03_ACCEL_CTL_ACC_FS_W_16G))
		return -EINVAL;

	ctl_reg = i2c_smbus_read_byte_data(sdata->client, KXG03_ACCEL_CTL);

	if (ctl_reg < 0)
		return ctl_reg;

	ctl_reg &= ~KXG03_ACCEL_CTL_ACC_FS_W_MASK;
	ctl_reg |= new_range;

	ctl_reg = i2c_smbus_write_byte_data(sdata->client, KXG03_ACCEL_CTL, ctl_reg);

	if (ctl_reg < 0)
		return ctl_reg;

	return 0;
}

/* Gyro ranges */
static int kxg03_gyro_set_range_w(struct kxg03_data *sdata, u8 new_range)
{
	s32 odr_reg;

	if ((new_range != KXG03_GYRO_ODR_WAKE_GYRO_FS_W_256) &&
		(new_range != KXG03_GYRO_ODR_WAKE_GYRO_FS_W_512) &&
		(new_range != KXG03_GYRO_ODR_WAKE_GYRO_FS_W_1024) &&
		(new_range != KXG03_GYRO_ODR_WAKE_GYRO_FS_W_2048))
		return -EINVAL;

	odr_reg = i2c_smbus_read_byte_data(sdata->client, KXG03_GYRO_ODR_WAKE);

	if (odr_reg < 0)
		return odr_reg;

	odr_reg &= ~KXG03_GYRO_ODR_WAKE_GYRO_FS_W_MASK;
	odr_reg |= new_range;

	odr_reg = i2c_smbus_write_byte_data(sdata->client, KXG03_GYRO_ODR_WAKE, odr_reg);

	if (odr_reg < 0)
		return odr_reg;

	return 0;
}

static int kxg03_set_sensor_init_values(struct kxg03_data *sdata)
{
	int err;

	/* low power settings is not set! */
	err = kxg03_accel_set_delay(sdata, sdata->accel_poll_rate);
	if (err < 0)
		return err;

	err = kxg03_gyro_set_delay(sdata, sdata->gyro_poll_rate);
	if (err < 0)
		return err;

	/* Wake mode used */
	err = kxg03_accel_set_grange_w(sdata, sdata->accel_range);
	if (err < 0)
		return err;

	/* Wake mode used */
	err = kxg03_gyro_set_range_w(sdata, sdata->gyro_range);
	if (err < 0)
		return err;

	/* Set low power mode off in wake state */
	err = kxg03_gyro_set_power_mode_w(sdata, KXG03_ACCEL_ODR_WAKE_LPMODE_W_DISABLED);
	if (err < 0)
		return err;

	/* Set wake/sleep mode, wake mode selected */
	err = kxg03_set_wake_sleep(sdata, KXG03_WAKE_SLEEP_CTL1_MAN_WAKE);
	if (err < 0)
		return err;

	return 0;
}

#define KXG03_SOFT_RESET_WAIT_MS 100

static int kxg03_soft_reset(struct kxg03_data *sdata)
{
	int err;

	err = i2c_smbus_write_byte_data(sdata->client, KXG03_CTL_REG_1,KXG03_CTL_REG_1_RST);
	if (err < 0) {
		__debug("%s - write RST failed %d\n", __FUNCTION__, err);
		return err;
	}
	/* Wait reset */
	msleep(KXG03_SOFT_RESET_WAIT_MS);
	
	/* Read por status */
	err = i2c_smbus_read_byte_data(sdata->client, KXG03_STATUS1);	
	if (err < 0) {
		__debug("%s - Read status1 failed %d\n", __FUNCTION__, err);
		return err;
	}

	if (err & KXG03_STATUS1_POR) {
		__debug("%s - soft reset succeed %d\n", __FUNCTION__, err);
		err = 0;
	} else {
		__debug("%s - soft reset failed %d\n", __FUNCTION__, err);
		return err;
	}

	return err;
}

static int kxg03_hw_detect(struct kxg03_data *sdata)
{
	int err;

	err = i2c_smbus_read_byte_data(sdata->client, KXG03_WHO_AM_I);
	if (err < 0)
		return err;

	__debug("%s whoami = %d\n", __FUNCTION__, err);

	if (err != KXG03_WHO_AM_I_VAL)
		return -ENODEV;

	sdata->wai = err;

	dev_info(&sdata->client->dev,
			"Kionix kxg03 detected\n");

	return 0;
}

static void kxg03_irq_report_data(struct kxg03_data *sdata, int status, ktime_t ts)
{
	int xyz[3];
	int err;

	if (status & KXG03_INT1_SRC1_INT1_DRDY_ACCTEMP) {
		err = kxg03_accel_read_xyz(sdata, xyz);

		if (err) {
			dev_err(&sdata->client->dev, "acc i2c read error\n");
		} else {
			kxg03_accel_report_xyz(sdata->accel_input_dev, xyz, ts);
		}
	}

	if (status & KXG03_INT1_SRC1_INT1_DRDY_GYRO) {

		if (sdata->gyro_start_delay > 0) {
			sdata->gyro_start_delay--;
			return;
		}

		err = kxg03_gyro_read_xyz(sdata, xyz);

		if (err) {
			dev_err(&sdata->client->dev, "gyro i2c read error\n");
		} else {
			kxg03_gyro_report_xyz(sdata->gyro_input_dev, xyz, ts);
		}
	}

	return;
}

static irqreturn_t kxg03_irq_handler(int irq, void *dev)
{
	struct kxg03_data *sdata = dev;
	ktime_t ts;
	int status;

	if (irq == sdata->irq1) {
		ts = ktime_get_boottime();
		status = i2c_smbus_read_byte_data(sdata->client, KXG03_INT1_SRC1);
	}
	else if (irq == sdata->irq2) {
		ts = ktime_get_boottime();
		status = i2c_smbus_read_byte_data(sdata->client, KXG03_INT2_SRC1);
	}
	else
		return IRQ_HANDLED;

	if (status < 0) {
		dev_err(&sdata->client->dev, "irq status read error\n");
		return IRQ_HANDLED;
	}

	//__debug("%s: INS1 status=0x%x\n", __FUNCTION__, status);

	kxg03_irq_report_data(sdata, status, ts);

	return IRQ_HANDLED;
}

/* Used interrupt bits */
#define KXG03_USED_INT_BITS (KXG03_INT_MASK1_DRDY_ACCTEMP | KXG03_INT_MASK1_DRDY_GYRO)

static int kxg03_enable_interrupt(struct kxg03_data *sdata, u8 bits, int irq)
{
	int err;
	s32 reg_val;

	if (!bits)
		return -EINVAL;

	reg_val = i2c_smbus_read_byte_data(sdata->client, KXG03_INT_MASK1);
	if (reg_val < 0)
		return reg_val;

	/* check if some of used int bit is already set */
	reg_val &= KXG03_USED_INT_BITS;

	__debug("%s - reg_val 0x%x kxg03 used int bits 0x%x \n", __FUNCTION__, reg_val, KXG03_USED_INT_BITS);

	/* enable new int bits */
	err = i2c_smbus_write_byte_data(sdata->client, KXG03_INT_MASK1, (reg_val | bits) );
	if (err < 0)
		return err;

	/* enable host interrupt if this is first int */
	if(!reg_val) {
		err = pinctrl_select_state(sdata->pinctrl, sdata->pin_default);
		if (err)
			return err;

		__debug("%s - enable irq %d\n", __FUNCTION__, irq);
		enable_irq(irq);
	}

	return 0;
}

static int kxg03_disable_interrupt(struct kxg03_data *sdata, u8 bits, int irq)
{
	int err;
	s32 reg_val;

	if (!bits)
		return -EINVAL;

	reg_val = i2c_smbus_read_byte_data(sdata->client, KXG03_INT_MASK1);
	if (reg_val < 0)
		return reg_val;

	reg_val &= KXG03_USED_INT_BITS;
	/* clear int bits */
	reg_val &= ~bits;

	err = i2c_smbus_write_byte_data(sdata->client, KXG03_INT_MASK1, reg_val);
	if (err < 0)
		return err;

	/* disable host interrupt if this is last int */
	if (!reg_val) {
		err = pinctrl_select_state(sdata->pinctrl, sdata->pin_sleep);
		if (err)
			return err;

		__debug("%s - disable irq %d\n", __FUNCTION__, irq);
		disable_irq(irq);
	}

	return 0;
}

/* accelerometer part */
static int kxg03_sensor_accel_enable(struct kxg03_data *sdata)
{
	int err;

	/* enable accel, clear stdby bit */
	err = kxg03_reset_reg_bit(sdata, KXG03_STDBY, KXG03_STDBY_ACC_STDBY_DISABLED);

	return err;
}

static int kxg03_sensor_accel_disable(struct kxg03_data *sdata)
{
	int err;

	/* disable accel, set stdby bit */
	err = kxg03_set_reg_bit(sdata, KXG03_STDBY, KXG03_STDBY_ACC_STDBY_DISABLED);

	return err;
}

static int kxg03_accel_enable(struct kxg03_data *sdata)
{
	int err;

	if (sdata->accel_enabled)
		return 0;

	err = kxg03_sensor_accel_enable(sdata);
	if (err < 0)
		return err;

	sdata->accel_enabled = true;

	/* Set accel data reportting */
	if (sdata->use_poll_timer) {
		hrtimer_start(&sdata->accel_timer, 
				ktime_set(sdata->accel_poll_rate / 1000, (sdata->accel_poll_rate % 1000) * 1000000),
				HRTIMER_MODE_REL);
	} else {
		err = kxg03_enable_interrupt(sdata, KXG03_INT_MASK1_DRDY_ACCTEMP, sdata->irq1);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kxg03_accel_disable(struct kxg03_data *sdata)
{
	int err;

	if (!sdata->accel_enabled)
		return 0;

	err = kxg03_sensor_accel_disable(sdata);
	if (err < 0)
		return err;

	/* disable data report */ 
	if (!sdata->use_poll_timer) {
		err = kxg03_disable_interrupt(sdata, KXG03_INT_MASK1_DRDY_ACCTEMP, sdata->irq1);
		if (err < 0)
			return err;
	}

	sdata->accel_enabled = false;

	return 0;
}

static int kxg03_accel_set_delay(struct kxg03_data *sdata, unsigned long delay)
{
	int err, i;

	if (delay < MIN_POLL_RATE_MS) {
 		delay = MIN_POLL_RATE_MS;
	}

	/* map delay to odr */
	for (i = 0; i < ARRAY_SIZE(kxg03_accel_odr_table); i++) {
		if (delay < kxg03_accel_odr_table[i].cutoff ||
			kxg03_accel_odr_table[i].cutoff == 0)
			break;
	}

	__debug("%s - delay = %lu cutoff = %u mask = 0x%x\n",
		__FUNCTION__, delay, kxg03_accel_odr_table[i].cutoff,
		kxg03_accel_odr_table[i].mask );

	if (sdata->accel_enabled) {
		/* toggle stdby - disable */
		err = kxg03_sensor_accel_disable(sdata);
		if (err < 0)
			return err;
	}

	err = kxg03_set_acc_odr(sdata, kxg03_accel_odr_table[i].mask, KXG03_ACCEL_ODR_WAKE_ODRA_W_MASK);

	if (sdata->accel_enabled) {
		/* toggle stdby - enabled */
		err = kxg03_sensor_accel_enable(sdata);
		if (err < 0)
			return err;
	}

	if (sdata->use_poll_timer && !err) {
		hrtimer_cancel(&sdata->accel_timer);
		sdata->accel_delay_change = true;
		sdata->accel_poll_rate = delay;
		if (sdata->accel_enabled)
			hrtimer_start(&sdata->accel_timer, 
				ktime_set(sdata->accel_poll_rate / 1000,
				(sdata->accel_poll_rate % 1000) * 1000000),
				HRTIMER_MODE_REL);
	}

	return err;
}

static void kxg03_accel_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts)
{
#ifdef DEBUG_XYZ_DATA
	__debug("acc - x,y,z,ts:, %d, %d, %d, %lld \n", xyz[0], xyz[1], xyz[2],
		ktime_to_ms(ts) );
#endif

	input_report_abs(dev, ABS_X, xyz[0]);
	input_report_abs(dev, ABS_Y, xyz[1]);
	input_report_abs(dev, ABS_Z, xyz[2]);

	input_event(dev, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(ts).tv_sec);
	input_event(dev, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(ts).tv_nsec);

	input_sync(dev);
}

static int kxg03_accel_read_xyz(struct kxg03_data *sdata, int *xyz)
{
	int len;
	s16 raw_xyz[3];

	len = i2c_smbus_read_i2c_block_data(sdata->client, KXG03_ACC_XOUT_L,
					6, (u8*)raw_xyz);
	if (len != 6)
		return -EIO;

	raw_xyz[0] = (s16) le16_to_cpu( raw_xyz[0] );
	raw_xyz[1] = (s16) le16_to_cpu( raw_xyz[1] );
	raw_xyz[2] = (s16) le16_to_cpu( raw_xyz[2] );

	xyz[0] = ((sdata->pdata.x_negate) ? (-raw_xyz[sdata->pdata.x_map])
		   : (raw_xyz[sdata->pdata.x_map]));
	xyz[1] = ((sdata->pdata.y_negate) ? (-raw_xyz[sdata->pdata.y_map])
		   : (raw_xyz[sdata->pdata.y_map]));
	xyz[2] = ((sdata->pdata.z_negate) ? (-raw_xyz[sdata->pdata.z_map])
		   : (raw_xyz[sdata->pdata.z_map]));

	return 0;
}

static int kxg03_accel_poll_thread(void *data)
{
	struct kxg03_data *sdata = data;

	while (1) {
		int err;
		int xyz[3];
		ktime_t ts;

		wait_event_interruptible(sdata->accel_wq,
			((sdata->accel_wkp_flag != 0) ||
				kthread_should_stop()));
		sdata->accel_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		if (sdata->accel_delay_change) {

#ifdef QCOM_SENSORS
			if (sdata->accel_poll_rate <= POLL_MS_100HZ) {
				set_wake_up_idle(true);
			}
			else {
				set_wake_up_idle(false);
			}
#endif
			sdata->accel_delay_change = false;
		}

		ts = ktime_get_boottime();
		err = kxg03_accel_read_xyz(sdata, xyz);
		if (err) {
			dev_err(&sdata->client->dev, "i2c read/write error\n");
		} else {
			kxg03_accel_report_xyz(sdata->accel_input_dev, xyz, ts);
		}
	}

	return 0;
}

static enum hrtimer_restart kxg03_accel_timer_handler(struct hrtimer *handle)
{
	struct kxg03_data *sdata = container_of(handle,
				struct kxg03_data, accel_timer);

	if (sdata->accel_enabled) {
		hrtimer_forward_now(&sdata->accel_timer, 
			ktime_set(sdata->accel_poll_rate / 1000, (sdata->accel_poll_rate % 1000) * 1000000));
		sdata->accel_wkp_flag = 1;
		wake_up_interruptible(&sdata->accel_wq);

		return HRTIMER_RESTART;
	} else {
		__debug("%s: no restart\n", __FUNCTION__);
		return HRTIMER_NORESTART;
	}	
}

static ssize_t kxg03_accel_sysfs_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kxg03_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->accel_enabled);
}

static ssize_t kxg03_accel_sysfs_set_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kxg03_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	if (value)
		err = kxg03_accel_enable(sdata);
	else
		err = kxg03_accel_disable(sdata);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t kxg03_accel_sysfs_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kxg03_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->accel_poll_rate);
}

static ssize_t kxg03_accel_sysfs_set_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kxg03_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	err = kxg03_accel_set_delay(sdata, value);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_accel_enable = __ATTR(enable,
		S_IRUGO | S_IWUSR,
		kxg03_accel_sysfs_get_enable,
		kxg03_accel_sysfs_set_enable);

static struct device_attribute dev_attr_accel_delay = __ATTR(delay,
		S_IRUGO | S_IWUSR,
		kxg03_accel_sysfs_get_delay,
		kxg03_accel_sysfs_set_delay);

static struct attribute *kxg03_accel_sysfs_attrs[] = {
	&dev_attr_accel_enable.attr,
	&dev_attr_accel_delay.attr,
	NULL
};

static struct attribute_group kxg03_accel_attribute_group = {
	.attrs = kxg03_accel_sysfs_attrs
};

static int kxg03_accel_input_dev_register(struct kxg03_data *sdata)
{
	int err;

	sdata->accel_input_dev = input_allocate_device();
	if (!sdata->accel_input_dev)
		return -ENOMEM;

	sdata->accel_input_dev->name = KXG03_ACCEL_INPUT_DEV;
	sdata->accel_input_dev->id.bustype = BUS_I2C;
	sdata->accel_input_dev->id.vendor  = sdata->wai;
	sdata->accel_input_dev->dev.parent = &sdata->client->dev;

	set_bit(EV_ABS, sdata->accel_input_dev->evbit);
	input_set_abs_params(sdata->accel_input_dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(sdata->accel_input_dev, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(sdata->accel_input_dev, ABS_Z, INT_MIN, INT_MAX, 0, 0);
	input_set_drvdata(sdata->accel_input_dev, sdata);

	err = input_register_device(sdata->accel_input_dev);

	return err;
}

static void kxg03_accel_input_dev_cleanup(struct kxg03_data *sdata)
{
	if (sdata->accel_input_dev) {
		input_unregister_device(sdata->accel_input_dev);
		sdata->accel_input_dev = NULL;
	}
}

/* Gyro part */

/* Gyro startup delay in samples */
#define KXG03_GYRO_STARTUP_DELAY_200HZ 85
#define KXG03_GYRO_STARTUP_DELAY_100HZ 40
#define KXG03_GYRO_STARTUP_DELAY_50HZ 20
#define KXG03_GYRO_STARTUP_DELAY_25HZ 10
#define KXG03_GYRO_STARTUP_DELAY_LOWRATEHZ 1

static void kxg03_gyro_startup_delay(struct kxg03_data *sdata)
{

	if (sdata->gyro_poll_rate <= POLL_MS_200HZ) {
		sdata->gyro_start_delay = KXG03_GYRO_STARTUP_DELAY_200HZ;
	} else if (sdata->gyro_poll_rate <= POLL_MS_100HZ) {
		sdata->gyro_start_delay =  KXG03_GYRO_STARTUP_DELAY_100HZ;
	} else if (sdata->gyro_poll_rate <= POLL_MS_50HZ) {
		sdata->gyro_start_delay = KXG03_GYRO_STARTUP_DELAY_50HZ;
	} else if (sdata->gyro_poll_rate <= POLL_MS_25HZ) {
		sdata->gyro_start_delay = KXG03_GYRO_STARTUP_DELAY_25HZ ;
	} else {
		sdata->gyro_start_delay =  KXG03_GYRO_STARTUP_DELAY_LOWRATEHZ;
	}

	__debug("%s - gyro_start_delay = %d \n",
		__FUNCTION__, sdata->gyro_start_delay);

	return;
}

#define KXG03_GYRO_RUN_WAIT_US (4*1000)
#define KXG03_GYRO_RUN_WAITMAX_US (6*1000)
#define KXG03_GYRO_RUN_TOTAL_WAIT_TIME_US (500*1000)

static int kxg03_sensor_gyro_enable(struct kxg03_data *sdata)
{
	int err;
	int max_wait_time = KXG03_GYRO_RUN_TOTAL_WAIT_TIME_US;

	/* enable gyro, clear stdby bit */
	err = kxg03_reset_reg_bit(sdata, KXG03_STDBY, KXG03_STDBY_GYRO_STDBY_W_DISABLED);
	if (err < 0)
		return err;

	/* wait gyro run */
	while (max_wait_time > 0) {

		usleep_range(KXG03_GYRO_RUN_WAIT_US, KXG03_GYRO_RUN_WAITMAX_US);
		max_wait_time -= KXG03_GYRO_RUN_WAITMAX_US;
		
		err =  i2c_smbus_read_byte_data(sdata->client, KXG03_STATUS1);

		if (err < 0)
			return err;

		if (!(err & KXG03_STATUS1_GYRO_RUN)) {
			/* running wait */
			err = -ETIME;
			__debug("%s gyro running wait time %d\n", __FUNCTION__, max_wait_time);
		}
		else {
			/* running ready */
			err = 0;
			__debug("%s gyro running ready \n", __FUNCTION__);
			break;
		}
	}

	return err;
}

static int kxg03_sensor_gyro_disable(struct kxg03_data *sdata)
{
	int err;

	/* disable gyro, set stdby bit */
	err = kxg03_set_reg_bit(sdata, KXG03_STDBY, KXG03_STDBY_GYRO_STDBY_W_DISABLED);

	return err;
}

static int kxg03_gyro_enable(struct kxg03_data *sdata)
{
	int err;

	if (sdata->gyro_enabled)
		return 0;

	err = kxg03_sensor_gyro_enable(sdata);
	if (err < 0)
		return err;

	sdata->gyro_enabled = true;

	kxg03_gyro_startup_delay(sdata);

	/* Set gyro data reporting */
	if (sdata->use_poll_timer) {
		hrtimer_start(&sdata->gyro_timer, 
				ktime_set(sdata->gyro_poll_rate / 1000, (sdata->gyro_poll_rate % 1000) * 1000000),
				HRTIMER_MODE_REL);
	} else {
		err = kxg03_enable_interrupt(sdata, KXG03_INT_MASK1_DRDY_GYRO, sdata->irq1);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kxg03_gyro_disable(struct kxg03_data *sdata)
{
	s32 err;

	if (!sdata->gyro_enabled)
		return 0;

	err = kxg03_sensor_gyro_disable(sdata);
	if (err < 0)
		return err;

	sdata->gyro_enabled = false;

	/* disable data report */ 
	if (!sdata->use_poll_timer) {
		err = kxg03_disable_interrupt(sdata, KXG03_INT_MASK1_DRDY_GYRO, sdata->irq1);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kxg03_gyro_set_delay(struct kxg03_data *sdata, unsigned long delay)
{
	int err, i;

	if (delay < MIN_POLL_RATE_MS) {
		delay = MIN_POLL_RATE_MS;
	}
	
	for (i = 0; i < ARRAY_SIZE(kxg03_gyro_odr_table); i++) {
		if (delay < kxg03_gyro_odr_table[i].cutoff ||
			kxg03_gyro_odr_table[i].cutoff == 0)
			break;
	}

	__debug("%s - delay = %lu cutoff = %u mask = 0x%x\n",
		__FUNCTION__, delay, kxg03_gyro_odr_table[i].cutoff,
		kxg03_gyro_odr_table[i].mask );

	if (sdata->gyro_enabled) {
		/* toggle stdby - disable */
		err = kxg03_sensor_gyro_disable(sdata);
		if (err < 0)
			return err;
	}

	err = kxg03_set_gyro_odr(sdata, kxg03_gyro_odr_table[i].mask, KXG03_GYRO_ODR_WAKE_ODRG_W_MASK);

	if (err < 0)
		return err;

	sdata->gyro_poll_rate = delay;

	if (sdata->gyro_enabled) {
		/* toggle stdby - enable */
		err = kxg03_sensor_gyro_enable(sdata);
		if (err < 0)
			return err;
		kxg03_gyro_startup_delay(sdata);
	}

	if (sdata->use_poll_timer && !err) {
		hrtimer_cancel(&sdata->gyro_timer);
		sdata->gyro_delay_change = true;
		if (sdata->gyro_enabled) {
			hrtimer_start(&sdata->gyro_timer, 
				ktime_set(sdata->gyro_poll_rate / 1000, (sdata->gyro_poll_rate % 1000) * 1000000),
				HRTIMER_MODE_REL);
		}
	}

	return err;
}

static void kxg03_gyro_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts)
{
	input_report_abs(dev, ABS_RX, xyz[0]);
	input_report_abs(dev, ABS_RY, xyz[1]);
	input_report_abs(dev, ABS_RZ, xyz[2]);

	input_event(dev, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(ts).tv_sec);
	input_event(dev, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(ts).tv_nsec);

	input_sync(dev);
}

static int kxg03_gyro_read_xyz(struct kxg03_data *sdata, int *xyz)
{
	int len;
	s16 raw_xyz[3] = { 0 };

	len = i2c_smbus_read_i2c_block_data(sdata->client, KXG03_GYRO_XOUT_L,
					6, (u8*)raw_xyz);
	if (len != 6)
		return -EIO;

	raw_xyz[0] = (s16) le16_to_cpu( raw_xyz[0] );
	raw_xyz[1] = (s16) le16_to_cpu( raw_xyz[1] );
	raw_xyz[2] = (s16) le16_to_cpu( raw_xyz[2] );

	xyz[0] = ((sdata->pdata.x_negate) ? (-raw_xyz[sdata->pdata.x_map])
			: (raw_xyz[sdata->pdata.x_map]));
	xyz[1] = ((sdata->pdata.y_negate) ? (-raw_xyz[sdata->pdata.y_map])
			: (raw_xyz[sdata->pdata.y_map]));
	xyz[2] = ((sdata->pdata.z_negate) ? (-raw_xyz[sdata->pdata.z_map])
			: (raw_xyz[sdata->pdata.z_map]));

	return 0;
}

static int kxg03_gyro_poll_thread(void *data)
{
	struct kxg03_data *sdata = data;

	while (1) {
		int err;
		int xyz[3];
		ktime_t ts;

		wait_event_interruptible(sdata->gyro_wq,
			((sdata->gyro_wkp_flag != 0) ||
				kthread_should_stop()));
		sdata->gyro_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		if (sdata->gyro_delay_change) {

#ifdef QCOM_SENSORS
			if (sdata->gyro_poll_rate <= POLL_MS_100HZ) {
				set_wake_up_idle(true);
			}
			else {
				set_wake_up_idle(false);
			}
#endif

			sdata->gyro_delay_change = false;
		}

		if (sdata->gyro_start_delay > 0) {
			sdata->gyro_start_delay--;
			continue;
		}

		ts = ktime_get_boottime();

		err = kxg03_gyro_read_xyz(sdata, xyz);
		if (err) {
			dev_err(&sdata->client->dev, "gyro i2c read/write error\n");
		} else {

#ifdef DEBUG_XYZ_DATA
		__debug("gyro report - x,y,z,ts:, %d, %d, %d, %lld \n", xyz[0], xyz[1], xyz[2],
					ktime_to_ms(ts) );
#endif
			kxg03_gyro_report_xyz(sdata->gyro_input_dev, xyz, ts);
		}
	}

	return 0;
}

static enum hrtimer_restart kxg03_gyro_timer_handler(struct hrtimer *handle)
{
	struct kxg03_data *sdata = container_of(handle,
				struct kxg03_data, gyro_timer);

	if (sdata->gyro_enabled) {
		hrtimer_forward_now(&sdata->gyro_timer, 
			ktime_set(sdata->gyro_poll_rate / 1000, (sdata->gyro_poll_rate % 1000) * 1000000));
		sdata->gyro_wkp_flag = 1;
		wake_up_interruptible(&sdata->gyro_wq);
		return HRTIMER_RESTART;
	} else {
		__debug("%s: no restart\n", __FUNCTION__);
		return HRTIMER_NORESTART;
	}
}

static ssize_t kxg03_gyro_sysfs_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kxg03_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->gyro_enabled);
}

static ssize_t kxg03_gyro_sysfs_set_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kxg03_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	if (value)
		err = kxg03_gyro_enable(sdata);
	else
		err = kxg03_gyro_disable(sdata);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t kxg03_gyro_sysfs_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kxg03_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->gyro_poll_rate);
}

static ssize_t kxg03_gyro_sysfs_set_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kxg03_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	err = kxg03_gyro_set_delay(sdata, value);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t kxg03_gyro_sysfs_reg_read(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned long value;
	struct kxg03_data *sdata = dev_get_drvdata(dev);
	u8 reg;
	s32 reg_val;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	reg = 0xff & value;

	mutex_lock(&sdata->mutex);
	reg_val = i2c_smbus_read_byte_data(sdata->client, reg);
	mutex_unlock(&sdata->mutex);

	if (reg_val < 0)
		__debug("kxg03 read reg 0x%x fail - %d \n", reg, reg_val);
	else
		__debug("kxg03 read reg 0x%x value 0x%x \n", reg, reg_val);

	return (reg_val < 0) ? reg_val : len;
}

static ssize_t kxg03_gyro_sysfs_reg_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kxg03_data *sdata = dev_get_drvdata(dev);
	u8 reg;
	u8 reg_val;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	/* first byte register */
	reg = 0xff & value;

	/* second byte value */
	reg_val = 0xff & (value >> 8);

	mutex_lock(&sdata->mutex);
	err = i2c_smbus_write_byte_data(sdata->client, reg, reg_val);
	mutex_unlock(&sdata->mutex);

	if (err < 0)
		__debug("kxg03 write reg 0x%x fail - %d \n", reg, err);
	else
		__debug("kxg03 write reg 0x%x value 0x%x \n", reg, reg_val);

	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_gyro_enable = __ATTR(enable,
		S_IRUGO | S_IWUSR,
		kxg03_gyro_sysfs_get_enable,
		kxg03_gyro_sysfs_set_enable);

static struct device_attribute dev_attr_gyro_delay = __ATTR(delay,
		S_IRUGO | S_IWUSR,
		kxg03_gyro_sysfs_get_delay,
		kxg03_gyro_sysfs_set_delay);

static struct device_attribute dev_attr_reg_read = __ATTR(reg_read,
		S_IWUSR,
		NULL,
		kxg03_gyro_sysfs_reg_read);

static struct device_attribute dev_attr_reg_write = __ATTR(reg_write,
		S_IWUSR,
		NULL,
		kxg03_gyro_sysfs_reg_write);

static struct attribute *kxg03_gyro_sysfs_attrs[] = {
	&dev_attr_gyro_enable.attr,
	&dev_attr_gyro_delay.attr,
	/* debug attributes */
	&dev_attr_reg_read.attr,
	&dev_attr_reg_write.attr,
	NULL
};

static struct attribute_group kxg03_gyro_attribute_group = {
	.attrs = kxg03_gyro_sysfs_attrs
};

static int kxg03_gyro_input_dev_register(struct kxg03_data *sdata)
{
	int err;

	sdata->gyro_input_dev = input_allocate_device();
	if (!sdata->gyro_input_dev)
		return -ENOMEM;

	sdata->gyro_input_dev->name = KXG03_GYRO_INPUT_DEV;
	sdata->gyro_input_dev->id.bustype = BUS_I2C;
	sdata->gyro_input_dev->id.vendor  = sdata->wai;
	sdata->gyro_input_dev->dev.parent = &sdata->client->dev;

	set_bit(EV_ABS, sdata->gyro_input_dev->evbit);
	input_set_abs_params(sdata->gyro_input_dev, ABS_RX, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(sdata->gyro_input_dev, ABS_RY, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(sdata->gyro_input_dev, ABS_RZ, INT_MIN, INT_MAX, 0, 0);
	input_set_drvdata(sdata->gyro_input_dev, sdata);

	err = input_register_device(sdata->gyro_input_dev);

	return err;
}

static void kxg03_gyro_input_dev_cleanup(struct kxg03_data *sdata)
{
	if(sdata->gyro_input_dev) {
		input_unregister_device(sdata->gyro_input_dev);
		sdata->gyro_input_dev = NULL;
	}
}


#ifdef QCOM_SENSORS
static int kxg03_accel_cdev_sensors_enable(struct sensors_classdev *sensors_cdev,
				    unsigned int enable)
{
	int err;
	struct kxg03_data *sdata = container_of(sensors_cdev,
						struct kxg03_data,
						accel_cdev);

	mutex_lock(&sdata->mutex);
	__debug("%s > enable = %d \n", __FUNCTION__, enable);
	if (enable)
		err = kxg03_accel_enable(sdata);
	else
		err = kxg03_accel_disable(sdata);
	mutex_unlock(&sdata->mutex);
	__debug("%s < enable = %d \n", __FUNCTION__, enable);

	return err;
}

static int  kxg03_accel_cdev_sensors_poll_delay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_ms)
{
	struct kxg03_data *sdata = container_of(sensors_cdev,
						struct kxg03_data,
						accel_cdev);
	int err;

	mutex_lock(&sdata->mutex);
	__debug("%s > delay_ms = %d \n", __FUNCTION__, delay_ms);
	err = kxg03_accel_set_delay(sdata, delay_ms);
	mutex_unlock(&sdata->mutex);
	__debug("%s < delay_ms = %d \n", __FUNCTION__, delay_ms);

	return err;
}

static int kxg03_gyro_cdev_sensors_enable(struct sensors_classdev *sensors_cdev,
				    unsigned int enable)
{
	struct kxg03_data *sdata = container_of(sensors_cdev,
						struct kxg03_data,
						gyro_cdev);
	int err;

	mutex_lock(&sdata->mutex);
	__debug("%s > enable = %d \n", __FUNCTION__, enable);
	if (enable)
		err = kxg03_gyro_enable(sdata);
	else
		err = kxg03_gyro_disable(sdata);
	mutex_unlock(&sdata->mutex);
	__debug("%s < enable = %d \n", __FUNCTION__, enable);

	return err;
}

static int  kxg03_gyro_cdev_sensors_poll_delay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_ms)
{
	struct kxg03_data *sdata = container_of(sensors_cdev,
						struct kxg03_data,
						gyro_cdev);
	int err;

	mutex_lock(&sdata->mutex);
	__debug("%s > delay_ms = %d \n", __FUNCTION__, delay_ms);
	err = kxg03_gyro_set_delay(sdata, delay_ms);
	__debug("%s < delay_ms = %d \n", __FUNCTION__, delay_ms);
	mutex_unlock(&sdata->mutex);

	return err;
}

static int kxg03_sensors_cdev_register(struct kxg03_data *sdata)
{
	int err;

	/* accel cdev */
	sdata->accel_cdev = accel_sensors_cdev;
	sdata->accel_cdev.sensors_enable = kxg03_accel_cdev_sensors_enable;
	sdata->accel_cdev.sensors_poll_delay = kxg03_accel_cdev_sensors_poll_delay;
	sdata->accel_cdev.sensors_calibrate = NULL;
	sdata->accel_cdev.sensors_write_cal_params = NULL;
	err = sensors_classdev_register(&sdata->accel_input_dev->dev, &sdata->accel_cdev);

	if (err) {
		return err;
	}

	/* gyro cdev */
	sdata->gyro_cdev = gyro_sensors_cdev;
	sdata->gyro_cdev.sensors_enable = kxg03_gyro_cdev_sensors_enable;
	sdata->gyro_cdev.sensors_poll_delay = kxg03_gyro_cdev_sensors_poll_delay;
	sdata->gyro_cdev.sensors_calibrate = NULL;
	sdata->gyro_cdev.sensors_write_cal_params = NULL;
	err = sensors_classdev_register(&sdata->gyro_input_dev->dev, &sdata->gyro_cdev);

	if (err) {
		sensors_classdev_unregister(&sdata->accel_cdev);
	}

	return err;
}

static void kxg03_sensors_cdev_unregister(struct kxg03_data *sdata)
{
	sensors_classdev_unregister(&sdata->gyro_cdev);
	sensors_classdev_unregister(&sdata->accel_cdev);
}
#endif

#ifdef CONFIG_OF
static u8 kxg03_map_value_to_grange(u8 grange_val)
{
	u8 grange_reg_val;

	switch (grange_val) {
		case 2: grange_reg_val = KXG03_ACC_RANGE_2G; break;
		case 4: grange_reg_val = KXG03_ACC_RANGE_4G; break;
		case 8: grange_reg_val = KXG03_ACC_RANGE_8G; break;
		case 16: grange_reg_val = KXG03_ACC_RANGE_16G; break;
		default: grange_reg_val = KXG03_ACC_RANGE_8G; break;
	}

	return grange_reg_val;
}

static u8 kxg03_map_value_to_gyro_range(u16 gyro_range_val)
{
	u8 gyro_range_reg_val;

	switch (gyro_range_val) {
		case 256: gyro_range_reg_val = KXG03_GYRO_RANGE_256; break;
		case 512: gyro_range_reg_val = KXG03_GYRO_RANGE_512; break;
		case 1024: gyro_range_reg_val = KXG03_GYRO_RANGE_1024; break;
		case 2048: gyro_range_reg_val = KXG03_GYRO_RANGE_2048; break;
		default: gyro_range_reg_val = KXG03_GYRO_RANGE_1024; break;
	}

	return gyro_range_reg_val;
}

static int kxg03_parse_dt(struct kxg03_data *sdata, struct device *dev)
{
	u32 temp_val;
	int err;

	/* mandatory dt parameters */
	err = of_property_read_u32(dev->of_node, "kionix,x-map", &temp_val);
	if (err && (err != -EINVAL)) {
		dev_err(dev, "Unable to read property x-map\n");
		return err;
	}
	sdata->pdata.x_map = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,y-map", &temp_val);
	if (err && (err != -EINVAL)) {
		dev_err(dev, "Unable to read property y-map\n");
		return err;
	}
	sdata->pdata.y_map = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,z-map", &temp_val);
	if (err && (err != -EINVAL)) {
		dev_err(dev, "Unable to read property z-map\n");
		return err;
	}
	sdata->pdata.z_map = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,x-negate", &temp_val);
	if (err && (err != -EINVAL)) {
		dev_err(dev, "Unable to read property x-negate\n");
		return err;
	}
	sdata->pdata.x_negate = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,y-negate", &temp_val);
	if (err && (err != -EINVAL)) {
		dev_err(dev, "Unable to read property y-negate\n");
		return err;
	}
	sdata->pdata.y_negate = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,z-negate", &temp_val);
	if (err && (err != -EINVAL)) {
		dev_err(dev, "Unable to read property z-negate\n");
		return err;
	}
	sdata->pdata.z_negate = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,g-range", &temp_val);
	if (err && (err != -EINVAL)) {
		dev_err(dev, "Unable to read property g-range\n");
		return err;
	}
	sdata->pdata.accel_range = kxg03_map_value_to_grange((u8) temp_val);

	err = of_property_read_u32(dev->of_node, "kionix,gyro-range", &temp_val);
	if (err && (err != -EINVAL)) {
		dev_err(dev, "Unable to read property gyro-range\n");
		return err;
	}

	sdata->pdata.gyro_range = kxg03_map_value_to_gyro_range((u16) temp_val);

	/* optional dt parameters i.e. use poll if interrupt not found */
	sdata->pdata.gpio_int1 = of_get_named_gpio_flags(dev->of_node,
				"kionix,gpio-int1", 0, NULL);

	sdata->pdata.gpio_int2 = of_get_named_gpio_flags(dev->of_node,
				"kionix,gpio-int2", 0, NULL);

	sdata->pdata.use_drdy_int = of_property_read_bool(dev->of_node, "kionix,use-drdy-int");

	__debug("kxg03 pdata.x_map %d\n", sdata->pdata.x_map);
	__debug("kxg03 pdata.y_map %d\n", sdata->pdata.y_map);
	__debug("kxg03 pdata.z_map %d\n", sdata->pdata.z_map);
	__debug("kxg03 pdata.x_negate %d\n", sdata->pdata.x_negate);
	__debug("kxg03 pdata.y_negate %d\n", sdata->pdata.y_negate);
	__debug("kxg03 pdata.z_negate %d\n", sdata->pdata.z_negate);
	__debug("kxg03 pdata.accel_range 0x%x\n", sdata->pdata.accel_range);
	__debug("kxg03 pdata.gyro_range 0x%x\n", sdata->pdata.gyro_range);
	__debug("kxg03 pdata.gpio_int1 %d\n", sdata->pdata.gpio_int1); 
	__debug("kxg03 pdata.gpio_int2 %d\n", sdata->pdata.gpio_int2);
	__debug("kxg03 pdata.use_drdy_int %d\n", sdata->pdata.use_drdy_int); 

	return 0;
}
#else
static int kxg03_parse_dt(struct kxg03_data *sdata, struct device *dev)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */

static int kxg03_pinctrl_init(struct kxg03_data *sdata)
{
	struct i2c_client *client = sdata->client;

	sdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(sdata->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(sdata->pinctrl);
	}

	sdata->pin_default = pinctrl_lookup_state(sdata->pinctrl,
			"kxg03_default");
	if (IS_ERR_OR_NULL(sdata->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(sdata->pin_default);
	}

	sdata->pin_sleep = pinctrl_lookup_state(sdata->pinctrl,
			"kxg03_sleep");
	if (IS_ERR_OR_NULL(sdata->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(sdata->pin_sleep);
	}

	return 0;
}

void kxg03_set_default_pdata(struct kxg03_data *sdata)
{
	/* Set default pdata */
	sdata->pdata.x_map = 0;
	sdata->pdata.y_map = 1;
	sdata->pdata.z_map = 2;
	sdata->pdata.x_negate = 0;
	sdata->pdata.y_negate = 0;
	sdata->pdata.z_negate = 0;
	sdata->pdata.accel_range = KXG03_ACC_RANGE_DEFAULT;
	sdata->pdata.gyro_range = KXG03_GYRO_RANGE_DEFAULT;
	sdata->pdata.gpio_int1 = 0; 
	sdata->pdata.gpio_int2 = 0;
	sdata->pdata.use_drdy_int = false;

	return;
}

/* probe */
static int kxg03_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct kxg03_data *sdata;
	int err;
	u8 int_ctl_val = 0;
   

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev,
			"missing required i2c functionality\n");
		return -ENODEV;
	}

	sdata = devm_kzalloc(&client->dev,
			sizeof(struct kxg03_data), GFP_KERNEL);
	if (!sdata) {
		dev_err(&client->dev, "no memory available\n");
		return -ENOMEM;
	}

	mutex_init(&sdata->mutex);
	mutex_lock(&sdata->mutex);

	sdata->client = client;
	i2c_set_clientdata(client, sdata);

	/* get driver platform data */
	if (client->dev.of_node) {
		err = kxg03_parse_dt(sdata, &client->dev);
		if (err) {
			dev_err(&client->dev,
				"Unable to parse dt data err=%d\n", err);
			err = -EINVAL;
			goto free_init;
		}
	} else if (client->dev.platform_data) {
			sdata->pdata =  *(struct kxg03_platform_data *)client->dev.platform_data;
	} else {
			dev_err(&client->dev,"No dt or pdata. Use default pdata.\n");
			kxg03_set_default_pdata(sdata);
	}

	err = kxg03_hw_detect(sdata);
	if (err) {
		dev_err(&client->dev, "sensor not recognized\n");
		goto free_init;
	}

	err = kxg03_soft_reset(sdata);
	if (err) {
		dev_err(&client->dev, "soft reset failed\n");
		goto free_init;
	}
	
	if (sdata->pdata.init) {
		err = sdata->pdata.init();
		if (err) {
			dev_err(&client->dev,
				"impossible to initialize the device\n");
			goto free_init;
		}
	}

	/* init accelerometer dev */
	err = kxg03_accel_input_dev_register(sdata);
	if (err < 0) {
		dev_err(&client->dev,
			"accel input register fail\n");
		goto free_src;
	}
	err = sysfs_create_group(&sdata->accel_input_dev->dev.kobj,
					&kxg03_accel_attribute_group);
	if (err) {
		dev_err(&client->dev,
			"accel sysfs create fail\n");
		goto free_input_accel;
	}

	/* init gyro dev */
	err = kxg03_gyro_input_dev_register(sdata);
	if (err < 0) {
		dev_err(&client->dev,
			"gyro input register fail\n");
		goto free_sysfs_accel;
	}

	err = sysfs_create_group(&sdata->gyro_input_dev->dev.kobj,
					&kxg03_gyro_attribute_group);
	if (err) {
		dev_err(&client->dev,
			"gyro sysfs create fail\n");
		goto free_input_gyro;
	}

	/* initialize pinctrl */
	if (!kxg03_pinctrl_init(sdata)) {
		err = pinctrl_select_state(sdata->pinctrl, sdata->pin_sleep);
		if (err) {
			dev_err(&client->dev, "Can't select pinctrl state\n");
			goto free_sysfs_gyro;
		}
	} else {
		dev_err(&client->dev, "Pinctrl init failed. Force timer polling.\n");
		sdata->pdata.use_drdy_int = false;
	}

	/* gpio to irq */
	if (sdata->pdata.use_drdy_int && gpio_is_valid(sdata->pdata.gpio_int1)) {
		err = gpio_request(sdata->pdata.gpio_int1, "kxg03_int1");
		if (err) {
			dev_err(&client->dev,
				"Unable to request interrupt gpio1 %d\n",
				sdata->pdata.gpio_int1);
			goto free_sysfs_gyro;
		}

		err = gpio_direction_input(sdata->pdata.gpio_int1);
		if (err) {
			dev_err(&client->dev,
				"Unable to set direction for gpio1 %d\n",
				sdata->pdata.gpio_int1);
			goto free_sysfs_gyro;
		}
		sdata->irq1 = gpio_to_irq(sdata->pdata.gpio_int1);
	}

	if (sdata->pdata.use_drdy_int && gpio_is_valid(sdata->pdata.gpio_int2)) {
		err = gpio_request(sdata->pdata.gpio_int2, "kxg03_int2");
		if (err) {
			dev_err(&client->dev,
				"Unable to request interrupt gpio2 %d\n",
				sdata->pdata.gpio_int2);
			goto free_sysfs_gyro;
		}

		err = gpio_direction_input(sdata->pdata.gpio_int2);
		if (err) {
			dev_err(&client->dev,
				"Unable to set direction for gpio2 %d\n",
				sdata->pdata.gpio_int2);
			goto free_sysfs_gyro;
		}
		sdata->irq2 = gpio_to_irq(sdata->pdata.gpio_int2);
	}

	/* Set interrupts */

	/* Driver code uses irq1 as default interrupt for both accel and gyro */
	/* to take irq2 in use update irq usage in enable/disable functions */
	if (sdata->irq1) {
		err = request_threaded_irq(sdata->irq1, NULL,
				kxg03_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"kxg03_irq", sdata);

		if (err) {
			dev_err(&client->dev, "unable to request irq1\n");
			goto free_sysfs_gyro;
		}

		disable_irq(sdata->irq1);

		/* Acc and gyro routed to pin 1 */
		err = i2c_smbus_write_byte_data(sdata->client,
						KXG03_INT_PIN1_SEL,
						KXG03_INT_PIN1_SEL_DRDY_ACCTEMP_P1 | KXG03_INT_PIN1_SEL_DRDY_GYRO_P1);

		if (err) {
			dev_err(&client->dev, "unable to set irq1 pin selection\n");
			goto free_irq1;
		}
        
		int_ctl_val = KXG03_INT_PIN_CTL_IEN1 | KXG03_INT_PIN_CTL_IEL1_LATCHED | KXG03_INT_PIN_CTL_IEA1_ACTIVE_HIGH;

		/* set irq1 active high, latched, push/pull */
		err = i2c_smbus_write_byte_data(sdata->client,
						KXG03_INT_PIN_CTL,
						int_ctl_val );

		if (err) {
			dev_err(&client->dev, "unable to set irq1 control\n");
			goto free_irq1;
		}
	}

	if (sdata->irq2) {
		err = request_threaded_irq(sdata->irq2, NULL,
				kxg03_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"kxg03_irq", sdata);

		if (err) {
			dev_err(&client->dev, "unable to request irq2\n");
			goto free_irq1;
		}

		disable_irq(sdata->irq2);
       
		/* Acc and gyro routed to pin 2 */
		err = i2c_smbus_write_byte_data(sdata->client,
						KXG03_INT_PIN2_SEL,
						KXG03_INT_PIN2_SEL_DRDY_ACCTEMP_P2 | KXG03_INT_PIN2_SEL_DRDY_GYRO_P2);

		if (err) {
			dev_err(&client->dev, "unable to set irq2 pin selection\n");
			goto free_irq2;
		}

		/* set irq2 active high, latched, push/pull */	
		int_ctl_val |= KXG03_INT_PIN_CTL_IEN2 | KXG03_INT_PIN_CTL_IEL2_LATCHED | KXG03_INT_PIN_CTL_IEA2_ACTIVE_HIGH;
	
		err = i2c_smbus_write_byte_data(sdata->client,
						KXG03_INT_PIN_CTL,
						int_ctl_val);

		if (err) {
			dev_err(&client->dev, "unable to set irq2 control\n");
			goto free_irq2;
		}
	}

	/* poll rate for accel and gyro */
	if(sdata->pdata.poll_rate_ms < MIN_POLL_RATE_MS)
		sdata->accel_poll_rate = MIN_POLL_RATE_MS;
	else
		sdata->accel_poll_rate = sdata->pdata.poll_rate_ms;
	sdata->accel_delay_change = true;

	if(sdata->pdata.poll_rate_ms < MIN_POLL_RATE_MS)
		sdata->gyro_poll_rate = MIN_POLL_RATE_MS;
	else
		sdata->gyro_poll_rate = sdata->pdata.poll_rate_ms;

	sdata->gyro_delay_change = true;

	/* grange for accel */
	if (sdata->pdata.accel_range != KXG03_ACC_RANGE_2G &&
		sdata->pdata.accel_range != KXG03_ACC_RANGE_4G &&
		sdata->pdata.accel_range != KXG03_ACC_RANGE_8G &&
		sdata->pdata.accel_range != KXG03_ACC_RANGE_16G)
		sdata->accel_range = KXG03_ACC_RANGE_DEFAULT;
	else
		sdata->accel_range = sdata->pdata.accel_range;
        
	/* range for gyro */
	if (sdata->pdata.gyro_range != KXG03_GYRO_RANGE_256 &&
		sdata->pdata.gyro_range != KXG03_GYRO_RANGE_512 &&
		sdata->pdata.gyro_range != KXG03_GYRO_RANGE_1024 &&
		sdata->pdata.gyro_range != KXG03_GYRO_RANGE_2048)
		sdata->gyro_range = KXG03_GYRO_RANGE_DEFAULT;
	else
		sdata->gyro_range = sdata->pdata.gyro_range;
        
	/* poll thread accel */
	hrtimer_init(&sdata->accel_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sdata->accel_timer.function = kxg03_accel_timer_handler;

	init_waitqueue_head(&sdata->accel_wq);
	sdata->accel_wkp_flag = 0;
	sdata->accel_task = kthread_run(kxg03_accel_poll_thread, sdata,
						"kionix_accel");
	/* poll thread gyro */
	hrtimer_init(&sdata->gyro_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sdata->gyro_timer.function = kxg03_gyro_timer_handler;

	init_waitqueue_head(&sdata->gyro_wq);
	sdata->gyro_wkp_flag = 0;
	sdata->gyro_task = kthread_run(kxg03_gyro_poll_thread, sdata,
						"kionix_gyro");

	/* set init values to sensor */
	err = kxg03_set_sensor_init_values(sdata);
	if (err < 0) {
		dev_err(&client->dev, "set init values fail\n");
		goto free_irq2;
	}

	/* data report using irq or timer*/
	if (sdata->pdata.use_drdy_int &&
	   (sdata->irq1 || sdata->irq2 ))
		sdata->use_poll_timer = false;
	else
		sdata->use_poll_timer = true;

#ifdef QCOM_SENSORS
	/* Register to sensors class */
	err = kxg03_sensors_cdev_register(sdata);
	if (err) {
		dev_err(&client->dev, "create class device file failed\n");
		err = -EINVAL;
		goto free_irq2;
	}
#endif

	mutex_unlock(&sdata->mutex);

	return 0;

free_irq2:
	if (sdata->accel_task) {
		kthread_stop(sdata->accel_task);
		sdata->accel_task = NULL;
	}

	if (sdata->gyro_task) {
		kthread_stop(sdata->gyro_task);
		sdata->gyro_task = NULL;
	}

	if (sdata->irq2)
		free_irq(sdata->irq2, sdata);

free_irq1:
	if (sdata->irq1)
		free_irq(sdata->irq1, sdata);
free_sysfs_gyro:
	sysfs_remove_group(&sdata->gyro_input_dev->dev.kobj,
			&kxg03_gyro_attribute_group);

free_input_gyro:
	kxg03_gyro_input_dev_cleanup(sdata);

free_sysfs_accel:
	sysfs_remove_group(&sdata->accel_input_dev->dev.kobj,
			&kxg03_accel_attribute_group);

free_input_accel:
	kxg03_accel_input_dev_cleanup(sdata);

free_src:
	if (sdata->pdata.release)
		sdata->pdata.release();
free_init:
	mutex_unlock(&sdata->mutex);

	return err;
}

static int kxg03_remove(struct i2c_client *client)
{
	struct kxg03_data *sdata = i2c_get_clientdata(client);
#ifdef QCOM_SENSORS
	kxg03_sensors_cdev_unregister(sdata);
#endif
	sysfs_remove_group(&sdata->accel_input_dev->dev.kobj,
		&kxg03_accel_attribute_group);
	sysfs_remove_group(&sdata->gyro_input_dev->dev.kobj,
			&kxg03_gyro_attribute_group);

	if (sdata->irq1){
		__debug("%s - free_irq\n", __FUNCTION__);
		free_irq(sdata->irq1, sdata);
		sdata->irq1 = 0;
	}
	if (sdata->irq2){
		__debug("%s - free_irq\n", __FUNCTION__);
		free_irq(sdata->irq2, sdata);
		sdata->irq2 = 0;
	}
	if (gpio_is_valid(sdata->pdata.gpio_int1)){
		__debug("%s - free_gpio\n", __FUNCTION__);
		gpio_free(sdata->pdata.gpio_int1);
	}
	if (gpio_is_valid(sdata->pdata.gpio_int2)){
		__debug("%s - free_gpio\n", __FUNCTION__);
		gpio_free(sdata->pdata.gpio_int2);
	}
	if (sdata->accel_task) {
		hrtimer_cancel(&sdata->accel_timer);
		kthread_stop(sdata->accel_task);
		sdata->accel_task = NULL;
	}
	if (sdata->gyro_task) {
		hrtimer_cancel(&sdata->gyro_timer);
		kthread_stop(sdata->gyro_task);
		sdata->gyro_task = NULL;
	}

	kxg03_accel_input_dev_cleanup(sdata);
	kxg03_gyro_input_dev_cleanup(sdata);

	if (sdata->pdata.release)
		sdata->pdata.release();

	return 0;
}

#ifdef CONFIG_PM
static int kxg03_suspend(struct device *dev)
{
	return 0;
}

static int kxg03_resume(struct device *dev)
{
	return 0;
}
#else
#define kxg03_suspend	NULL
#define kxg03_resume	NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int kxg03_runtime_suspend(struct device *dev)
{
	return 0;
}

static int kxg03_runtime_resume(struct device *dev)
{
	return 0;
}
#else
#define kxg03_runtime_suspend	NULL
#define kxg03_runtime_resume	NULL
#endif

static const struct i2c_device_id kxg03_id[] = {
			{ KXG03_DEV_NAME, 0 },
			{ },
		};

MODULE_DEVICE_TABLE(i2c, kxg03_id);

static const struct of_device_id kxg03_of_match[] = {
	{ .compatible = "kionix,kxg03", },
	{ },
};
MODULE_DEVICE_TABLE(of, kxg03_of_match);
static const struct dev_pm_ops kxg03_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(kxg03_suspend, kxg03_resume)
	SET_RUNTIME_PM_OPS(kxg03_runtime_suspend, kxg03_runtime_resume, NULL)
};

static struct i2c_driver kxg03_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name  = KXG03_DEV_NAME,
			.pm    = &kxg03_pm_ops,
			.of_match_table = kxg03_of_match,
		  },
	.probe = kxg03_probe,
	.remove = kxg03_remove,
	.id_table = kxg03_id,
};

module_i2c_driver(kxg03_driver);

MODULE_DESCRIPTION("kxg03 driver");
MODULE_AUTHOR("Kionix");
MODULE_LICENSE("GPL");

