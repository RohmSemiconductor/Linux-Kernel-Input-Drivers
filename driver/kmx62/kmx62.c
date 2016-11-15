/*
 * kmx62: tri-axis accelerometer / tri-axis magnetometer
 *
 * Copyright(C) 2015 Kionix, Inc.
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
#include "kmx62.h"
#include "kmx62_regs.h"

//#define DEBUG_XYZ_DATA

#define __debug printk
//#define __debug pr_debug

#define KMX62_WHO_AM_I_VAL18 0x18
#define KMX62_WHO_AM_I_VAL19 0x19

/* poll defines */
#define POLL_MS_100HZ 10
#define MIN_POLL_RATE_MS 5

/* input dev names */
#define KMX62_ACCEL_INPUT_DEV "kmx62-accel"
#define KMX62_MAG_INPUT_DEV "kmx62-mag"

/* sensor register map */
#define KMX62_INC3_POR 0x88

struct kmx62_data {
	struct i2c_client *client;
	struct kmx62_platform_data pdata;
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
	wait_queue_head_t	accel_wq;
	bool accel_enabled;
	u16 accel_poll_rate;

	/* magnetometer */
	struct input_dev *mag_input_dev;
	struct hrtimer mag_timer;
	int mag_wkp_flag;
	struct task_struct *mag_task;
	bool mag_delay_change;
	wait_queue_head_t	mag_wq;
	bool mag_enabled;
	u16 mag_poll_rate;

#ifdef QCOM_SENSORS
	struct sensors_classdev accel_cdev;
	struct sensors_classdev mag_cdev;
#endif

	int irq1;
	int irq2;

	u8 g_range;
	u8 wai;

	bool use_poll_timer;

};

/* delay to odr map table */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kmx62_accel_odr_table[] = {
	{ 2, KMX62_ODCNTL_OSA_1600},
	{ 3, KMX62_ODCNTL_OSA_800},
	{ 5, KMX62_ODCNTL_OSA_400},
	{ 10, KMX62_ODCNTL_OSA_200},
	{ 20, KMX62_ODCNTL_OSA_100},
	{ 40, KMX62_ODCNTL_OSA_50},
	{ 80, KMX62_ODCNTL_OSA_25},
	{ 160, KMX62_ODCNTL_OSA_12P5},
	{ 320, KMX62_ODCNTL_OSA_6P25},
	{ 640, KMX62_ODCNTL_OSA_3P125},
	{ 1280,	KMX62_ODCNTL_OSA_1P563},
	{ 0, 	KMX62_ODCNTL_OSA_0P781},};

static const struct {
	unsigned int cutoff;
	u8 mask;
} kmx62_mag_odr_table[] = {
	{ 2, KMX62_ODCNTL_OSM_1600},
	{ 3, KMX62_ODCNTL_OSM_800},
	{ 5, KMX62_ODCNTL_OSM_400},
	{ 10, KMX62_ODCNTL_OSM_200},
	{ 20, KMX62_ODCNTL_OSM_100},
	{ 40, KMX62_ODCNTL_OSM_50},
	{ 80, KMX62_ODCNTL_OSM_25},
	{ 160, KMX62_ODCNTL_OSM_12P5},
	{ 320, KMX62_ODCNTL_OSM_6P25},
	{ 640, KMX62_ODCNTL_OSM_3P125},
	{ 1280,	KMX62_ODCNTL_OSM_1P563},
	{ 0, 	KMX62_ODCNTL_OSM_0P781},};

#ifdef QCOM_SENSORS
/* Qualcomm sensors class defines*/

#define POLL_INTERVAL_MIN_MS MIN_POLL_RATE_MS
#define POLL_DEFAULT_INTERVAL_MS 200

static struct sensors_classdev accel_sensors_cdev = {
	.name = KMX62_ACCEL_INPUT_DEV,
	.vendor = "Kionix",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	// TODO - add dt grange mapping to sensors_classdev to resolution.
	// now fixed 8g is defined here.
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

static struct sensors_classdev mag_sensors_cdev = {
	.name = KMX62_MAG_INPUT_DEV,
	.vendor = "Kionix",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "1200",	/* uT */
	.resolution = "0.0366",	/* uT */
	.sensor_power = "0.295",	/* typical value */
	.min_delay = POLL_INTERVAL_MIN_MS * 1000,	/* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = POLL_DEFAULT_INTERVAL_MS,	/* in millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

static int kmx62_accel_poll_thread(void *data);
static int kmx62_mag_poll_thread(void *data);
static int kmx62_accel_set_delay(struct kmx62_data *sdata, unsigned long delay);
static int kmx62_mag_set_delay(struct kmx62_data *sdata, unsigned long delay);

static void kmx62_accel_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts);
static int kmx62_accel_read_xyz(struct kmx62_data *sdata, int *xyz);
static void kmx62_mag_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts);
static int kmx62_mag_read_xyz(struct kmx62_data *sdata, int *xyz);

/* common part */
/* Returns 0 on success, negative on error */
static int kmx62_change_reg_bits(struct kmx62_data *sdata, u8 reg, u8 bits, u8 mask)
{
	s32 reg_val;

	reg_val = i2c_smbus_read_byte_data(sdata->client, reg);
	if (reg_val < 0)
		return reg_val;	//error code

	reg_val &= ~mask;
	reg_val |= bits;
	return i2c_smbus_write_byte_data(sdata->client, reg, reg_val);
}

/* Returns 0 on success, negative on error */
inline static int kmx62_set_reg_bit(struct kmx62_data *sdata, u8 reg, u8 bits)
{
	return kmx62_change_reg_bits(sdata, reg, bits, 0);
}

/* Returns 0 on success, negative on error */
inline static int kmx62_clear_reg_bit(struct kmx62_data *sdata, u8 reg, u8 bits)
{
	return kmx62_change_reg_bits(sdata, reg, 0, bits);
}

static int kmx62_set_odr(struct kmx62_data *sdata, u8 odr, u8 odr_mask)
{
	s32 cntl2_reg;
	s32 cntl2_reg_sensors_disabled;
	int err;

	/* spec says both sensors should be in stand-by before update odr */

	/* 1. set sensor to stand-by */
	cntl2_reg = i2c_smbus_read_byte_data(sdata->client, KMX62_CNTL2);
	if (cntl2_reg < 0)
		return cntl2_reg;	//Error code

	cntl2_reg_sensors_disabled = (cntl2_reg & ~(KMX62_CNTL2_MAG_EN | KMX62_CNTL2_ACCEL_EN));
	if (cntl2_reg != cntl2_reg_sensors_disabled) {
		err = i2c_smbus_write_byte_data(sdata->client,
						KMX62_CNTL2,
						cntl2_reg_sensors_disabled
						);
		if (err < 0)
			return err;
	}

	/* 2. update new odr to sensor */
	err = kmx62_change_reg_bits(sdata, KMX62_ODCNTL, odr, odr_mask);
	if (err < 0)
		return err;

	/* 3. enable sensor if it was enabled */
	if (cntl2_reg & (KMX62_CNTL2_MAG_EN | KMX62_CNTL2_ACCEL_EN)) {
		err = i2c_smbus_write_byte_data(sdata->client,
						KMX62_CNTL2,
						cntl2_reg
						);
		if (err < 0)
			return err;
	}
	return 0;
}

/* Returns 0 on success, negative on error */
static int kmx62_accel_set_grange(struct kmx62_data *sdata, u8 new_range)
{
	if ((new_range != KMX62_CNTL2_GSEL_2G) &&
		(new_range != KMX62_CNTL2_GSEL_4G) &&
		(new_range != KMX62_CNTL2_GSEL_8G) &&
		(new_range != KMX62_CNTL2_GSEL_16G))
		return -EINVAL;

	return kmx62_change_reg_bits(sdata,
			KMX62_CNTL2, new_range, KMX62_CNTL2_GSEL_MASK);
}

/* Returns 0 on success, negative on error */
static int kmx62_accel_set_res(struct kmx62_data *sdata, u8 res)
{
	return kmx62_change_reg_bits(sdata, 
			KMX62_CNTL2, res, KMX62_CNTL2_RES_MASK);
}

/* Return true/1 on error, false/0 on success */
static bool kmx62_set_sensor_init_values(struct kmx62_data *sdata)
{
	bool err;

	err = false || kmx62_accel_set_delay(sdata, sdata->accel_poll_rate);
	err = err   || kmx62_mag_set_delay(sdata, sdata->mag_poll_rate);
	err = err   || kmx62_accel_set_grange(sdata, sdata->g_range);
	err = err   || kmx62_accel_set_res(sdata, KMX62_CNTL2_RES_MAX);

	return err;
}

/* Returns 0 on success, negative on error */
static int kmx62_soft_reset(struct kmx62_data *sdata)
{
	return i2c_smbus_write_byte_data(sdata->client, KMX62_CNTL1,
		KMX62_CNTL1_SRST_1);
}

/* Returns 0 on success, negative on error */
static int kmx62_hw_detect(struct kmx62_data *sdata)
{
	int waireg;

	waireg = i2c_smbus_read_byte_data(sdata->client, KMX62_WHO_AM_I);
	if (waireg < 0)
		return waireg;

	__debug("%s whoami = 0x%x\n", __FUNCTION__, waireg);

	switch (waireg){
		case KMX62_WHO_AM_I_VAL18:
			break;
		case KMX62_WHO_AM_I_VAL19:
			break;
		default:
			return -ENODEV;
	}
	dev_info(&sdata->client->dev, "Kionix KMX62 detected\n");
	sdata->wai = waireg;

	return 0;
}

static void kmx62_irq_report_data(struct kmx62_data *sdata, int status)
{
	int xyz[3];
	ktime_t ts;
	int err;

	ts = ktime_get_boottime();

	if (status & KMX62_INS1_DRDY_A_1) {
		err = kmx62_accel_read_xyz(sdata, xyz);

		if (err) {
			dev_err(&sdata->client->dev, "acc i2c read error\n");
		} else {
			kmx62_accel_report_xyz(sdata->accel_input_dev, xyz, ts);
		}
	}

	if (status & KMX62_INS1_DRDY_M_1) {
		err = kmx62_mag_read_xyz(sdata, xyz);

		if (err) {
			dev_err(&sdata->client->dev, "mag i2c read error\n");
		} else {
			kmx62_mag_report_xyz(sdata->mag_input_dev, xyz, ts);
		}
	}

	return;
}

static irqreturn_t kmx62_irq_handler(int irq, void *dev)
{
	struct kmx62_data *sdata = dev;
	int status;

	status = i2c_smbus_read_byte_data(sdata->client, KMX62_INS1);

	if (status < 0) {
		dev_err(&sdata->client->dev, "irq status read error\n");
		return IRQ_HANDLED;
	}

	if (status == 0) {
		//dev_err(&sdata->client->dev, "%s: INS1 status ZERO !\n", __FUNCTION__);
		return IRQ_HANDLED;
	}


	//__debug("%s: INS1 status=0x%x\n", __FUNCTION__, status);

	kmx62_irq_report_data(sdata, status);

	return IRQ_HANDLED;
}

/* bits == KMX62_INC1_DRDY_A1 | KMX62_INC1_DRDY_M1
   return 0 on success, negative on fail */
static int kmx62_enable_interrupt(struct kmx62_data *sdata, u8 bits, int irq)
{
	int err;
	s32 incx_reg;
	s32 incx_val;

	if (!bits)
		return -EINVAL;

	/* irq to interrupt report register */
	if (sdata->irq1 == irq)
		incx_reg = KMX62_INC1;
	else if(sdata->irq2 == irq)
		incx_reg = KMX62_INC2;
	else
		return -EINVAL;

        incx_val = i2c_smbus_read_byte_data(sdata->client, incx_reg);
	if (incx_val < 0)
		return incx_val;	//error code

	err = i2c_smbus_write_byte_data(sdata->client, incx_reg, (incx_val | bits) );
	if (err)
		return err;

	if (!incx_val) {
		/* enable irq if incx val was zero */
		err = pinctrl_select_state(sdata->pinctrl, sdata->pin_default);
		if (err)
			return err;

		__debug("%s - enable irq %d\n", __FUNCTION__, irq);
		enable_irq(irq);
	}

	return 0;
}

/* bits == KMX62_INC1_DRDY_A1 | KMX62_INC1_DRDY_M1
   return 0 on success, negative on fail */
static int kmx62_disable_interrupt(struct kmx62_data *sdata, u8 bits, int irq)
{
	int err;
	s32 incx_reg;
	s32 incx_val;

	if (!bits)
		return -EINVAL;

	/* map irq to interrupt report register */
	if (sdata->irq1 == irq)
		incx_reg = KMX62_INC1;
	else if(sdata->irq2 == irq)
		incx_reg = KMX62_INC2;
	else
		return -EINVAL;

        incx_val = i2c_smbus_read_byte_data(sdata->client, incx_reg);
	if (incx_val < 0)
		return incx_val;
	incx_val &= ~bits;

	err = i2c_smbus_write_byte_data(sdata->client, incx_reg, incx_val);
	if (err)
		return err;

	if (!incx_val) {
		/* disable irq if incx val goes to zero */
		err = pinctrl_select_state(sdata->pinctrl, sdata->pin_sleep);
		if (err)
			return err;

		__debug("%s - disable irq %d\n", __FUNCTION__, irq);
		disable_irq(irq);
	}
	return 0;
}

static inline ktime_t kmx62_pollrate2ktime(u16 rate){
	return ktime_set(rate / 1000, (rate % 1000) * 1000000);
};

/* accelerometer part */
static int kmx62_accel_enable(struct kmx62_data *sdata)
{
	s32 err;

	if (sdata->accel_enabled)
		return 0;

	/* enable accel */
	err = kmx62_set_reg_bit(sdata, KMX62_CNTL2, KMX62_CNTL2_ACCEL_EN);
	if (err < 0)
		return err;

	/* set accel data reportting */
	if (sdata->use_poll_timer) {
		ktime_t poll_interval;

		poll_interval = kmx62_pollrate2ktime(sdata->accel_poll_rate);
		hrtimer_start(&sdata->accel_timer, poll_interval, HRTIMER_MODE_REL);
	} else {
		// TODO - only irq2 used
		err = kmx62_enable_interrupt(sdata, KMX62_INC1_DRDY_A1, sdata->irq2);
		__debug("%s - err 2 %d\n", __FUNCTION__, err);
		if (err < 0)
			return err;
	}

	sdata->accel_enabled = true;

	return 0;
}

static int kmx62_accel_disable(struct kmx62_data *sdata)
{
	int err;

	if (!sdata->accel_enabled)
		return 0;

	/* disable accel */
	err = kmx62_clear_reg_bit(sdata, KMX62_CNTL2, KMX62_CNTL2_ACCEL_EN);
	if (err < 0)
		return err;

	/* disable data report */ 
	if (!sdata->use_poll_timer) {
		err = kmx62_disable_interrupt(sdata, KMX62_INC1_DRDY_A1, sdata->irq2);
		if (err < 0)
			return err;
	}

	sdata->accel_enabled = false;

	return 0;
}

static int kmx62_accel_set_delay(struct kmx62_data *sdata, unsigned long delay)
{
	int err, i;

	if (delay < MIN_POLL_RATE_MS) {
 		delay = MIN_POLL_RATE_MS;
	}

	/* map delay to odr */
	for (i = 0; i < ARRAY_SIZE(kmx62_accel_odr_table); i++) {
		if (delay < kmx62_accel_odr_table[i].cutoff ||
		    kmx62_accel_odr_table[i].cutoff == 0)
			break;
	}

	__debug("%s - delay = %lu cutoff = %u mask = 0x%x\n",
		__FUNCTION__, delay, kmx62_accel_odr_table[i].cutoff,
		kmx62_accel_odr_table[i].mask );

	err = kmx62_set_odr(sdata, kmx62_accel_odr_table[i].mask, KMX62_ODCNTL_OSA_MASK);

	if (sdata->use_poll_timer && !err) {
		hrtimer_cancel(&sdata->accel_timer);
		sdata->accel_delay_change = true;
		sdata->accel_poll_rate = delay;
		if (sdata->accel_enabled){
			ktime_t poll_interval;

			poll_interval = kmx62_pollrate2ktime(sdata->accel_poll_rate);
			hrtimer_start(&sdata->accel_timer, poll_interval, HRTIMER_MODE_REL);
		}
	}

	return err;
}

static void kmx62_accel_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts)
{
#ifdef DEBUG_XYZ_DATA
	__debug("acc - x,y,z,ts:, %d, %d, %d, %lld \n", xyz[0], xyz[1], xyz[2],
		ktime_to_ms(ts) );
#endif
	input_report_abs(dev, ABS_X, xyz[0]);
	input_report_abs(dev, ABS_Y, xyz[1]);
	input_report_abs(dev, ABS_Z, xyz[2]);
	input_event(dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(ts).tv_sec);
	input_event(dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(ts).tv_nsec);
	input_sync(dev);
}

static int kmx62_accel_read_xyz(struct kmx62_data *sdata, int *xyz)
{

	int err;
	u8 reg_xyz[6];
	s16 raw_xyz[3] = { 0 };

	err = i2c_smbus_read_i2c_block_data(sdata->client, KMX62_ACCEL_XOUT_L,
					6, reg_xyz);
	if (err != 6)
		return -EIO;

	raw_xyz[0] = ((s16) ((reg_xyz[1] << 8) | reg_xyz[0]));
	raw_xyz[1] = ((s16) ((reg_xyz[3] << 8) | reg_xyz[2]));
	raw_xyz[2] = ((s16) ((reg_xyz[5] << 8) | reg_xyz[4]));

	xyz[0] = ((sdata->pdata.x_negate) ? (-raw_xyz[sdata->pdata.x_map])
		   : (raw_xyz[sdata->pdata.x_map]));
	xyz[1] = ((sdata->pdata.y_negate) ? (-raw_xyz[sdata->pdata.y_map])
		   : (raw_xyz[sdata->pdata.y_map]));
	xyz[2] = ((sdata->pdata.z_negate) ? (-raw_xyz[sdata->pdata.z_map])
		   : (raw_xyz[sdata->pdata.z_map]));

	return 0;
}

static int kmx62_accel_poll_thread(void *data)
{
	struct kmx62_data *sdata = data;
	int err;
	int xyz[3];
	ktime_t ts;

	while (1) {
		wait_event_interruptible(sdata->accel_wq,
			((sdata->accel_wkp_flag != 0) ||
				kthread_should_stop()));
		sdata->accel_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		if (sdata->accel_delay_change) {
#ifdef QCOM_SENSORS
			if (sdata->accel_poll_rate <= POLL_MS_100HZ)
				set_wake_up_idle(true);
			else
				set_wake_up_idle(false);
#endif
			sdata->accel_delay_change = false;

		}

		ts = ktime_get_boottime();
		err = kmx62_accel_read_xyz(sdata, xyz);
		if (err) {
			dev_err(&sdata->client->dev, "i2c read/write error\n");
		} else {
			kmx62_accel_report_xyz(sdata->accel_input_dev, xyz, ts);
		}
	}

	return 0;
}

static enum hrtimer_restart kmx62_accel_timer_handler(struct hrtimer *handle)
{
	struct kmx62_data *sdata = container_of(handle,
				struct kmx62_data, accel_timer);

	if (sdata->accel_enabled) {
		ktime_t poll_interval;

		poll_interval = kmx62_pollrate2ktime(sdata->accel_poll_rate);
		hrtimer_forward_now(&sdata->accel_timer, poll_interval);
		sdata->accel_wkp_flag = 1;
		wake_up_interruptible(&sdata->accel_wq);

		return HRTIMER_RESTART;
	} else {
		__debug("%s: No restart\n", __FUNCTION__);
		return HRTIMER_NORESTART;
	}	
}

static ssize_t kmx62_accel_sysfs_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kmx62_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->accel_enabled);
}

static ssize_t kmx62_accel_sysfs_set_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kmx62_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	if (value)
		err = kmx62_accel_enable(sdata);
	else
		err = kmx62_accel_disable(sdata);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t kmx62_accel_sysfs_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kmx62_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->accel_poll_rate);
}

static ssize_t kmx62_accel_sysfs_set_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kmx62_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	err = kmx62_accel_set_delay(sdata, value);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t kmx62_accel_sysfs_reg_read(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	unsigned long value;
	struct kmx62_data *sdata;
	u8 reg;
	s32 reg_val;

	sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;
	reg = 0xff & value;

	mutex_lock(&sdata->mutex);
        reg_val = i2c_smbus_read_byte_data(sdata->client, reg);
	mutex_unlock(&sdata->mutex);

	if (reg_val < 0)
		__debug(" read - reg 0x%x fail - %d \n", reg, reg_val);
	else
		__debug(" read - reg 0x%x value 0x%x \n", reg, reg_val);

	return (reg_val < 0) ? reg_val : len;
}

static ssize_t kmx62_accel_sysfs_reg_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kmx62_data *sdata;
	u8 reg;
	u8 reg_val;

	sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;
	reg = 0xff & value;
        reg_val = 0xff & (value >> 8);

	mutex_lock(&sdata->mutex);
	err = i2c_smbus_write_byte_data(sdata->client, reg, reg_val);
	mutex_unlock(&sdata->mutex);

	if (err < 0)
		__debug(" write - reg 0x%x fail - %d \n", reg, err);
	else
		__debug(" write - reg 0x%x value 0x%x \n", reg, reg_val);

	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_accel_enable = __ATTR(enable,
		S_IRUGO | S_IWUSR,
		kmx62_accel_sysfs_get_enable,
		kmx62_accel_sysfs_set_enable);

static struct device_attribute dev_attr_accel_delay = __ATTR(delay,
		S_IRUGO | S_IWUSR,
		kmx62_accel_sysfs_get_delay,
		kmx62_accel_sysfs_set_delay);

static struct device_attribute dev_attr_reg_read = __ATTR(reg_read,
		S_IWUSR,
		NULL,
		kmx62_accel_sysfs_reg_read);

static struct device_attribute dev_attr_reg_write = __ATTR(reg_write,
		S_IWUSR,
		NULL,
		kmx62_accel_sysfs_reg_write);

static struct attribute *kmx62_accel_sysfs_attrs[] = {
	&dev_attr_accel_enable.attr,
	&dev_attr_accel_delay.attr,
	&dev_attr_reg_read.attr,
	&dev_attr_reg_write.attr,
	NULL
};

static struct attribute_group kmx62_accel_attribute_group = {
	.attrs = kmx62_accel_sysfs_attrs
};

static int kmx62_accel_input_dev_register(struct kmx62_data *sdata)
{
	int err;

	sdata->accel_input_dev = input_allocate_device();
	if (!sdata->accel_input_dev)
		return -ENOMEM;

	sdata->accel_input_dev->name       = KMX62_ACCEL_INPUT_DEV;
	sdata->accel_input_dev->id.bustype = BUS_I2C;
	sdata->accel_input_dev->id.vendor  = sdata->wai;
	sdata->accel_input_dev->dev.parent = &sdata->client->dev;

	set_bit(EV_ABS, sdata->accel_input_dev->evbit);
	input_set_abs_params(sdata->accel_input_dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(sdata->accel_input_dev, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(sdata->accel_input_dev, ABS_Z, INT_MIN, INT_MAX, 0, 0);
	input_set_drvdata(sdata->accel_input_dev, sdata);

	err = input_register_device(sdata->accel_input_dev);

	if (err){
		input_free_device(sdata->accel_input_dev);
		sdata->accel_input_dev = NULL;
		return err;
	}

	return 0;
}

static void kmx62_accel_input_dev_unregister(struct kmx62_data *sdata)
{
	input_unregister_device(sdata->accel_input_dev);
	sdata->accel_input_dev = NULL;
}

/* magnetometer part */
static int kmx62_mag_enable(struct kmx62_data *sdata)
{
	s32 err;

	if (sdata->mag_enabled)
		return 0;

	/* enable mag */
	err = kmx62_set_reg_bit(sdata, KMX62_CNTL2, KMX62_CNTL2_MAG_EN);
	if (err < 0)
		return err;

	/* set mag data reportting */
	if (sdata->use_poll_timer) {
		ktime_t poll_interval;

		poll_interval = kmx62_pollrate2ktime(sdata->mag_poll_rate);
		hrtimer_start(&sdata->mag_timer, poll_interval, HRTIMER_MODE_REL);
	} else {
		// TODO - only irq2 used
		err = kmx62_enable_interrupt(sdata, KMX62_INC1_DRDY_M1, sdata->irq2);
		if (err < 0)
			return err;
	}

	sdata->mag_enabled = true;

	return 0;
}

static int kmx62_mag_disable(struct kmx62_data *sdata)
{
	s32 err;

	if (!sdata->mag_enabled)
		return 0;

	/* disable mag */
	err = kmx62_clear_reg_bit(sdata, KMX62_CNTL2, KMX62_CNTL2_MAG_EN);
	if (err < 0)
		return err;

	/* disable mag data reportting */
	if (!sdata->use_poll_timer) {
		err = kmx62_disable_interrupt(sdata, KMX62_INC1_DRDY_M1, sdata->irq2 );
		if (err < 0)
			return err;
	}

	sdata->mag_enabled = false;

	return 0;
}

static int kmx62_mag_set_delay(struct kmx62_data *sdata, unsigned long delay)
{
	int err, i;

	if (delay < MIN_POLL_RATE_MS) {
 		delay = MIN_POLL_RATE_MS;
	}

	/* map delay to mag odr */
	for (i = 0; i < ARRAY_SIZE(kmx62_mag_odr_table); i++) {
		if (delay < kmx62_mag_odr_table[i].cutoff ||
		    kmx62_mag_odr_table[i].cutoff == 0)
			break;
	}

	__debug("%s - delay = %lu cutoff = %u mask = 0x%x\n",
		__FUNCTION__, delay, kmx62_mag_odr_table[i].cutoff,
		kmx62_mag_odr_table[i].mask );

	err = kmx62_set_odr(sdata, kmx62_mag_odr_table[i].mask, KMX62_ODCNTL_OSM_MASK);

	if (sdata->use_poll_timer && !err) {
		hrtimer_cancel(&sdata->mag_timer);
		sdata->mag_delay_change = true;
		sdata->mag_poll_rate = delay;
		if (sdata->mag_enabled){
			ktime_t poll_interval;

			poll_interval = kmx62_pollrate2ktime(sdata->mag_poll_rate);
			hrtimer_start(&sdata->mag_timer, poll_interval, HRTIMER_MODE_REL);
		}
	}

	return err;
}

static void kmx62_mag_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts)
{
#ifdef DEBUG_XYZ_DATA
	__debug("mag - x,y,z,ts:, %d, %d, %d, %lld \n", xyz[0], xyz[1], xyz[2],
		ktime_to_ms(ts) );
#endif

	input_report_abs(dev, ABS_X, xyz[0]);
	input_report_abs(dev, ABS_Y, xyz[1]);
	input_report_abs(dev, ABS_Z, xyz[2]);
	input_event(dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(ts).tv_sec);
	input_event(dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(ts).tv_nsec);
	input_sync(dev);
}

static int kmx62_mag_read_xyz(struct kmx62_data *sdata, int *xyz)
{

	int err;
	u8 reg_xyz[6];
	s16 raw_xyz[3] = { 0 };

	err = i2c_smbus_read_i2c_block_data(sdata->client, KMX62_MAG_XOUT_L,
					6, reg_xyz);
	if (err != 6)
		return -EIO;

	raw_xyz[0] = ((s16) ((reg_xyz[1] << 8) | reg_xyz[0]));
	raw_xyz[1] = ((s16) ((reg_xyz[3] << 8) | reg_xyz[2]));
	raw_xyz[2] = ((s16) ((reg_xyz[5] << 8) | reg_xyz[4]));

	xyz[0] = ((sdata->pdata.x_negate) ? (-raw_xyz[sdata->pdata.x_map])
		   : (raw_xyz[sdata->pdata.x_map]));
	xyz[1] = ((sdata->pdata.y_negate) ? (-raw_xyz[sdata->pdata.y_map])
		   : (raw_xyz[sdata->pdata.y_map]));
	xyz[2] = ((sdata->pdata.z_negate) ? (-raw_xyz[sdata->pdata.z_map])
		   : (raw_xyz[sdata->pdata.z_map]));

	return 0;
}

static int kmx62_mag_poll_thread(void *data)
{
	struct kmx62_data *sdata = data;
	int err;
	int xyz[3];
	ktime_t ts;

	while (1) {
		wait_event_interruptible(sdata->mag_wq,
			((sdata->mag_wkp_flag != 0) ||
				kthread_should_stop()));
		sdata->mag_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		if (sdata->mag_delay_change) {
#ifdef QCOM_SENSORS
			if (sdata->mag_poll_rate <= POLL_MS_100HZ)
				set_wake_up_idle(true);
			else
				set_wake_up_idle(false);
#endif
			sdata->mag_delay_change = false;

		}

		ts = ktime_get_boottime();
		err = kmx62_mag_read_xyz(sdata, xyz);

		if (err) {
			dev_err(&sdata->client->dev, "mag i2c read/write error\n");
		} else {
			kmx62_mag_report_xyz(sdata->mag_input_dev, xyz, ts);
		}
	}

	return 0;
}

static enum hrtimer_restart kmx62_mag_timer_handler(struct hrtimer *handle)
{
	struct kmx62_data *sdata = container_of(handle,
				struct kmx62_data, mag_timer);

	if (sdata->mag_enabled) {
		ktime_t poll_interval;

		poll_interval = kmx62_pollrate2ktime(sdata->mag_poll_rate);
		hrtimer_forward_now(&sdata->mag_timer, poll_interval);
		sdata->mag_wkp_flag = 1;
		wake_up_interruptible(&sdata->mag_wq);

		return HRTIMER_RESTART;
	} else {
		__debug("%s: No restart\n", __FUNCTION__);
		return HRTIMER_NORESTART;
	}	
}

static ssize_t kmx62_mag_sysfs_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kmx62_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->mag_enabled);
}

static ssize_t kmx62_mag_sysfs_set_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kmx62_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	if (value)
		err = kmx62_mag_enable(sdata);
	else
		err = kmx62_mag_disable(sdata);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t kmx62_mag_sysfs_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kmx62_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->mag_poll_rate);
}

static ssize_t kmx62_mag_sysfs_set_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kmx62_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	err = kmx62_mag_set_delay(sdata, value);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_mag_enable = __ATTR(enable,
		S_IRUGO | S_IWUSR,
		kmx62_mag_sysfs_get_enable,
		kmx62_mag_sysfs_set_enable);

static struct device_attribute dev_attr_mag_delay = __ATTR(delay,
		S_IRUGO | S_IWUSR,
		kmx62_mag_sysfs_get_delay,
		kmx62_mag_sysfs_set_delay);

static struct attribute *kmx62_mag_sysfs_attrs[] = {
	&dev_attr_mag_enable.attr,
	&dev_attr_mag_delay.attr,
	NULL
};

static struct attribute_group kmx62_mag_attribute_group = {
	.attrs = kmx62_mag_sysfs_attrs
};

static int kmx62_mag_input_dev_register(struct kmx62_data *sdata)
{
	int err;

	sdata->mag_input_dev = input_allocate_device();
	if (!sdata->mag_input_dev)
		return -ENOMEM;

	sdata->mag_input_dev->name       = KMX62_MAG_INPUT_DEV;
	sdata->mag_input_dev->id.bustype = BUS_I2C;
	sdata->mag_input_dev->id.vendor  = sdata->wai;
	sdata->mag_input_dev->dev.parent = &sdata->client->dev;

	set_bit(EV_ABS, sdata->mag_input_dev->evbit);
	input_set_abs_params(sdata->mag_input_dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(sdata->mag_input_dev, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(sdata->mag_input_dev, ABS_Z, INT_MIN, INT_MAX, 0, 0);
	input_set_drvdata(sdata->mag_input_dev, sdata);

	err = input_register_device(sdata->mag_input_dev);
	if (err){
		input_free_device(sdata->mag_input_dev);
		sdata->mag_input_dev = NULL;
		return err;
	}
	return 0;
}

static void kmx62_mag_input_dev_unregister(struct kmx62_data *sdata)
{
	input_unregister_device(sdata->mag_input_dev);
	sdata->mag_input_dev = NULL;
}


#ifdef QCOM_SENSORS
static int kmx62_accel_cdev_sensors_enable(struct sensors_classdev *sensors_cdev,
				    unsigned int enable)
{
	struct kmx62_data *sdata = container_of(sensors_cdev,
						struct kmx62_data,
						accel_cdev);
	int err;

	mutex_lock(&sdata->mutex);
	if (enable)
		err = kmx62_accel_enable(sdata);
	else
		err = kmx62_accel_disable(sdata);
	mutex_unlock(&sdata->mutex);

	return err;
}

static int  kmx62_accel_cdev_sensors_poll_delay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_ms)
{
	struct kmx62_data *sdata = container_of(sensors_cdev,
						struct kmx62_data,
						accel_cdev);
	int err;

	mutex_lock(&sdata->mutex);
	err = kmx62_accel_set_delay(sdata, delay_ms);
	mutex_unlock(&sdata->mutex);

	return err;
}

static int kmx62_mag_cdev_sensors_enable(struct sensors_classdev *sensors_cdev,
				    unsigned int enable)
{
	struct kmx62_data *sdata = container_of(sensors_cdev,
						struct kmx62_data,
						mag_cdev);
	int err;

	mutex_lock(&sdata->mutex);
	if (enable)
		err = kmx62_mag_enable(sdata);
	else
		err = kmx62_mag_disable(sdata);
	mutex_unlock(&sdata->mutex);

	return err;
}

static int  kmx62_mag_cdev_sensors_poll_delay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_ms)
{
	struct kmx62_data *sdata = container_of(sensors_cdev,
						struct kmx62_data,
						mag_cdev);
	int err;

	mutex_lock(&sdata->mutex);
	err = kmx62_mag_set_delay(sdata, delay_ms);
	mutex_unlock(&sdata->mutex);

	return err;
}

static int kmx62_sensors_cdev_register(struct kmx62_data *sdata)
{
	int err;

	/* accel cdev */
	sdata->accel_cdev = accel_sensors_cdev;
	sdata->accel_cdev.sensors_enable = kmx62_accel_cdev_sensors_enable;
	sdata->accel_cdev.sensors_poll_delay = kmx62_accel_cdev_sensors_poll_delay;
	sdata->accel_cdev.sensors_calibrate = NULL;
	sdata->accel_cdev.sensors_write_cal_params = NULL;
	err = sensors_classdev_register(&sdata->accel_input_dev->dev, &sdata->accel_cdev);


	if (err) {
		return err;
	}

	/* mag cdev */
	sdata->mag_cdev = mag_sensors_cdev;
	sdata->mag_cdev.sensors_enable = kmx62_mag_cdev_sensors_enable;
	sdata->mag_cdev.sensors_poll_delay = kmx62_mag_cdev_sensors_poll_delay;
	sdata->mag_cdev.sensors_calibrate = NULL;
	sdata->mag_cdev.sensors_write_cal_params = NULL;
	err = sensors_classdev_register(&sdata->mag_input_dev->dev, &sdata->mag_cdev);

	if (err) {
		sensors_classdev_unregister(&sdata->accel_cdev);
	}

	return err;
}

static void kmx62_sensors_cdev_unregister(struct kmx62_data *sdata)
{
	sensors_classdev_unregister(&sdata->mag_cdev);
	sensors_classdev_unregister(&sdata->accel_cdev);
}

#endif

#ifdef CONFIG_OF
static u8 kmx62_map_value_to_grange(u8 grange_val)
{
	u8 grange_reg_val;

	switch (grange_val) {
		case 2: grange_reg_val = KMX62_G_RANGE_2G; break;
		case 4: grange_reg_val = KMX62_G_RANGE_4G; break;
		case 8: grange_reg_val = KMX62_G_RANGE_8G; break;
		case 16: grange_reg_val = KMX62_G_RANGE_16G; break;
		default: grange_reg_val = KMX62_G_RANGE_8G; break;
	}

	return grange_reg_val;
}

//Return 0 on success, negative on error
static int kmx62_parse_dt(struct kmx62_data *sdata, struct device *dev)
{
	u32 temp_val;
	int err;

	/* mandatory dt parameters */
	err = of_property_read_u32(dev->of_node, "kionix,x-map", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property x-map\n");
		return err;
	}
	sdata->pdata.x_map = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,y-map", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property y-map\n");
		return err;
	}
	sdata->pdata.y_map = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,z-map", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property z-map\n");
		return err;
	}
	sdata->pdata.z_map = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,x-negate", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property x-negate\n");
		return err;
	}
	sdata->pdata.x_negate = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,y-negate", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property y-negate\n");
		return err;
	}
	sdata->pdata.y_negate = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,z-negate", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property z-negate\n");
		return err;
	}
	sdata->pdata.z_negate = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,g-range", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property g-range\n");
		return err;
	}
	sdata->pdata.g_range = kmx62_map_value_to_grange((u8) temp_val);

	/* optional dt parameters i.e. use poll if interrupt not found */
	sdata->pdata.gpio_int1 = of_get_named_gpio_flags(dev->of_node,
				"kionix,gpio-int1", 0, NULL);
	if (sdata->pdata.gpio_int1 < 0) {
		//Invalid gpio
		sdata->pdata.gpio_int1 = 0;
	}

	sdata->pdata.gpio_int2 = of_get_named_gpio_flags(dev->of_node,
				"kionix,gpio-int2", 0, NULL);
	if (sdata->pdata.gpio_int2 < 0) {
		//Invalid gpio
		sdata->pdata.gpio_int2 = 0;
	}

	sdata->pdata.use_drdy_int = of_property_read_bool(dev->of_node, "kionix,use-drdy-int");

#if 0
	__debug("sdata->pdata.x_map %d\n", sdata->pdata.x_map);
	__debug("sdata->pdata.y_map %d\n", sdata->pdata.y_map);
	__debug("sdata->pdata.z_map %d\n", sdata->pdata.z_map);
	__debug("sdata->pdata.x_negate %d\n", sdata->pdata.x_negate);
	__debug("sdata->pdata.y_negate %d\n", sdata->pdata.y_negate);
	__debug("sdata->pdata.z_negate %d\n", sdata->pdata.z_negate);
	__debug("sdata->pdata.gpio_int1 %d\n", sdata->pdata.gpio_int1); 
	__debug("sdata->pdata.gpio_int2 %d\n", sdata->pdata.gpio_int2);
	__debug("sdata->pdata.use_drdy_int %d\n", sdata->pdata.use_drdy_int); 
#endif
	return 0;
}
#else
static int kmx62_parse_dt(struct kmx62_data *sdata, struct device *dev)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */

static int kmx62_pinctrl_init(struct kmx62_data *sdata)
{
	struct i2c_client *client = sdata->client;

	sdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(sdata->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(sdata->pinctrl);
	}
	
	//Todo: Check how to automatically choose using pinctrl-0/1
	sdata->pin_default = pinctrl_lookup_state(sdata->pinctrl,
			"kmx62_int2_default");
	if (IS_ERR_OR_NULL(sdata->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(sdata->pin_default);
	}

	sdata->pin_sleep = pinctrl_lookup_state(sdata->pinctrl,
			"kmx62_int2_sleep");
	if (IS_ERR_OR_NULL(sdata->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(sdata->pin_sleep);
	}

	return 0;
}

/* probe */
static int kmx62_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{	
	struct kmx62_data *sdata;
	int err;
	u8 inc3_val = 0;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev,
			"no algorithm associated to the i2c bus\n");
		return -ENODEV;
	}

	sdata = devm_kzalloc(&client->dev,
			sizeof(struct kmx62_data), GFP_KERNEL);
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
		/* Get data from device tree http://lwn.net/Articles/448502/ */
		err = kmx62_parse_dt(sdata, &client->dev);
		if (err) {
			dev_err(&client->dev,
				"Unable to parse dt data err=%d\n", err);
			err = -EINVAL;
			goto free_init;
		}
	} else if (client->dev.platform_data) {
		/* The driver has been instantiated in the traditional static way and
		   device tree does not enter into the picture. http://lwn.net/Articles/448499/ */
		sdata->pdata =  *(struct kmx62_platform_data *)client->dev.platform_data;
	} else {
		dev_err(&client->dev,"platform data is NULL\n");
		//Todo: could set default values instead of exit
		err = -EINVAL;
		goto free_init;
	}


	err = kmx62_hw_detect(sdata);
	if (err) {
		dev_err(&client->dev, "sensor not recognized\n");
		goto free_init;
	}

	err = kmx62_soft_reset(sdata);
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
	err = kmx62_accel_input_dev_register(sdata);
	if (err < 0) {
		dev_err(&client->dev,
			"accel input register fail\n");
		goto free_src;
	}
	err = sysfs_create_group(&sdata->accel_input_dev->dev.kobj,
					&kmx62_accel_attribute_group);
	if (err) {
		dev_err(&client->dev,
			"accel sysfs create fail\n");
		goto free_input_accel;
	}

	/* init magnetometer dev */
	err = kmx62_mag_input_dev_register(sdata);
	if (err < 0) {
		dev_err(&client->dev,
			"mag input register fail\n");
		goto free_sysfs_accel;
	}

	err = sysfs_create_group(&sdata->mag_input_dev->dev.kobj,
					&kmx62_mag_attribute_group);
	if (err) {
		dev_err(&client->dev,
			"mag sysfs create fail\n");
		goto free_input_mag;
	}

	/* initialize pinctrl */
	err = kmx62_pinctrl_init(sdata); 
	if (!err) {
		//Init OK
		err = pinctrl_select_state(sdata->pinctrl, sdata->pin_sleep);
		if (err) {
			dev_err(&client->dev, "Can't select pinctrl state\n");
			goto free_sysfs_mag;
		}
	} else if (sdata->pdata.use_drdy_int) {
			dev_err(&client->dev, "Pinctrl init failed. Forcing poll-thread instead of drdy-int.\n");
			sdata->pdata.use_drdy_int = false;
	} else {
			//(use_drdy_int == false) => don't care if pinctrl failed
			dev_err(&client->dev, "Pinctrl init failed..\n");
	}

	/* gpio to irq */
	if (sdata->pdata.use_drdy_int && sdata->pdata.gpio_int1 && gpio_is_valid(sdata->pdata.gpio_int1)) {
		err = gpio_request(sdata->pdata.gpio_int1, "kmx62-int1");
		if (err) {
			dev_err(&client->dev,
				"Unable to request interrupt gpio1 %d\n",
				sdata->pdata.gpio_int1);
			goto free_sysfs_mag;
		}

		err = gpio_direction_input(sdata->pdata.gpio_int1);
		if (err) {
			dev_err(&client->dev,
				"Unable to set direction for gpio1 %d\n",
				sdata->pdata.gpio_int1);
			goto free_gpio1;
		}
		sdata->irq1 = gpio_to_irq(sdata->pdata.gpio_int1);
	}

	if (sdata->pdata.use_drdy_int && sdata->pdata.gpio_int2 && gpio_is_valid(sdata->pdata.gpio_int2)) {
		err = gpio_request(sdata->pdata.gpio_int2, "kmx62-int2");
		if (err) {
			dev_err(&client->dev,
				"Unable to request interrupt gpio2 %d\n",
				sdata->pdata.gpio_int2);
			goto free_gpio1;
		}

		err = gpio_direction_input(sdata->pdata.gpio_int2);
		if (err) {
			dev_err(&client->dev,
				"Unable to set direction for gpio2 %d\n",
				sdata->pdata.gpio_int2);
			goto free_gpio2;
		}
		sdata->irq2 = gpio_to_irq(sdata->pdata.gpio_int2);
	}

	/* setup irq handler */
	// TODO - select irq1 or irq2. now irq2 is used
	if (sdata->irq1) {
		err = request_threaded_irq(sdata->irq1, NULL,
				kmx62_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"kmx62_irq", sdata);

		if (err) {
			dev_err(&client->dev, "unable to request irq1\n");
			goto free_gpio2;
		}

		disable_irq(sdata->irq1);

		/* set irq1 active high, latched, push/pull */
		inc3_val |= KMX62_INC3_IEA1_1 | KMX62_INC3_IEL1_LATCHED | KMX62_INC3_IED1_0;
		err = i2c_smbus_write_byte_data(sdata->client,
						KMX62_INC3,
						inc3_val);

		if (err) {
			dev_err(&client->dev, "unable to set irq2 control\n");
			goto free_irq1;
		}
	}

	if (sdata->irq2) {
		err = request_threaded_irq(sdata->irq2, NULL,
				kmx62_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"kmx62_irq", sdata);

		if (err) {
			dev_err(&client->dev, "unable to request irq2\n");
			goto free_irq1;
		}

		disable_irq(sdata->irq2);

		/* set irq2 active high, latched, push/pull */
		inc3_val |= KMX62_INC3_IEA2_1 | KMX62_INC3_IEL2_LATCHED | KMX62_INC3_IED2_0;
		err = i2c_smbus_write_byte_data(sdata->client,
						KMX62_INC3,
						inc3_val);

		if (err) {
			dev_err(&client->dev, "unable to set irq2 control\n");
			goto free_irq2;
		}
	}

	/* poll rate for accel and mag */
	if (sdata->pdata.poll_rate_ms < MIN_POLL_RATE_MS)
		sdata->accel_poll_rate = MIN_POLL_RATE_MS;
	else
		sdata->accel_poll_rate = sdata->pdata.poll_rate_ms;
	sdata->accel_delay_change = true;

	if(sdata->pdata.poll_rate_ms < MIN_POLL_RATE_MS)
		sdata->mag_poll_rate = MIN_POLL_RATE_MS;
	else
		sdata->mag_poll_rate = sdata->pdata.poll_rate_ms;
	sdata->mag_delay_change = true;

	/* grange for accel */
	if (sdata->pdata.g_range != KMX62_G_RANGE_2G &&
		sdata->pdata.g_range != KMX62_G_RANGE_4G &&
		sdata->pdata.g_range != KMX62_G_RANGE_8G &&
		sdata->pdata.g_range != KMX62_G_RANGE_16G)
		sdata->g_range = KMX62_DEFAULT_G_RANGE;
	else
		sdata->g_range = sdata->pdata.g_range;


	/* poll thread accel */
	hrtimer_init(&sdata->accel_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sdata->accel_timer.function = kmx62_accel_timer_handler;

	init_waitqueue_head(&sdata->accel_wq);
	sdata->accel_wkp_flag = 0;
	sdata->accel_task = kthread_run(kmx62_accel_poll_thread, sdata,
						"kionix_accel");
	/* poll thread mag */
	hrtimer_init(&sdata->mag_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sdata->mag_timer.function = kmx62_mag_timer_handler;

	init_waitqueue_head(&sdata->mag_wq);
	sdata->mag_wkp_flag = 0;
	sdata->mag_task = kthread_run(kmx62_mag_poll_thread, sdata,
						"kionix_mag");

	/* set init values to sensor */
	err = kmx62_set_sensor_init_values(sdata);
	if (err) {
		dev_err(&client->dev, "set init values fail\n");
		goto free_poll_threads;
	}

	/* data report using irq or timer*/
	if (sdata->pdata.use_drdy_int &&
	   (sdata->irq1 || sdata->irq2 ))
		sdata->use_poll_timer = false;
	else
		sdata->use_poll_timer = true;

#ifdef QCOM_SENSORS
	/* Register to sensors class */
	err = kmx62_sensors_cdev_register(sdata);
	if (err) {
		dev_err(&client->dev, "create class device file failed\n");
		err = -EINVAL;
		goto free_poll_threads;
	}
#endif
	//To be used for next fail exit: goto free_cdev

	mutex_unlock(&sdata->mutex);

	return 0;

//free_cdev:
#ifdef QCOM_SENSORS
//	kmx62_sensors_cdev_unregister(sdata);
#endif
free_poll_threads:
	if (sdata->accel_task) {
		hrtimer_cancel(&sdata->accel_timer);
		kthread_stop(sdata->accel_task);
		sdata->accel_task = NULL;
		}
	if (sdata->mag_task) {
		hrtimer_cancel(&sdata->mag_timer);
		kthread_stop(sdata->mag_task);
		sdata->mag_task = NULL;
		}
free_irq2:
	if (sdata->irq2){
		__debug("%s - free_irq\n", __FUNCTION__);
		free_irq(sdata->irq2, sdata);
		sdata->irq2 = 0;
		}
free_irq1:
	if (sdata->irq1){
		__debug("%s - free_irq\n", __FUNCTION__);
		free_irq(sdata->irq1, sdata);
		sdata->irq1 = 0;
		}
free_gpio2:
	if (sdata->pdata.gpio_int2){
		__debug("%s - free_gpio\n", __FUNCTION__);
		gpio_free(sdata->pdata.gpio_int2);
		sdata->pdata.gpio_int2 = 0;
		}
free_gpio1:
	if (sdata->pdata.gpio_int1){
		__debug("%s - free_gpio\n", __FUNCTION__);
		gpio_free(sdata->pdata.gpio_int1);
		sdata->pdata.gpio_int1 = 0;
		}
free_sysfs_mag:
	sysfs_remove_group(&sdata->mag_input_dev->dev.kobj,
			&kmx62_mag_attribute_group);
free_input_mag:
	kmx62_mag_input_dev_unregister(sdata);
free_sysfs_accel:
	sysfs_remove_group(&sdata->accel_input_dev->dev.kobj,
			&kmx62_accel_attribute_group);
free_input_accel:
	kmx62_accel_input_dev_unregister(sdata);
free_src:
	if (sdata->pdata.release)
		sdata->pdata.release();
free_init:
	mutex_unlock(&sdata->mutex);

	return err;
}

static int kmx62_remove(struct i2c_client *client)
{
	struct kmx62_data *sdata = i2c_get_clientdata(client);
#ifdef QCOM_SENSORS
	kmx62_sensors_cdev_unregister(sdata);
#endif
	if (sdata->accel_task) {
		hrtimer_cancel(&sdata->accel_timer);
		kthread_stop(sdata->accel_task);
		sdata->accel_task = NULL;
		}
	if (sdata->mag_task) {
		hrtimer_cancel(&sdata->mag_timer);
		kthread_stop(sdata->mag_task);
		sdata->mag_task = NULL;
		}
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
	if (sdata->pdata.gpio_int1){
		__debug("%s - free_gpio\n", __FUNCTION__);
		gpio_free(sdata->pdata.gpio_int1);
		sdata->pdata.gpio_int1 = 0;
		}
	if (sdata->pdata.gpio_int2){
		__debug("%s - free_gpio\n", __FUNCTION__);
		gpio_free(sdata->pdata.gpio_int2);
		sdata->pdata.gpio_int2 = 0;
		}

	sysfs_remove_group(&sdata->mag_input_dev->dev.kobj,
			&kmx62_mag_attribute_group);
	kmx62_mag_input_dev_unregister(sdata);
	sysfs_remove_group(&sdata->accel_input_dev->dev.kobj,
		&kmx62_accel_attribute_group);
	kmx62_accel_input_dev_unregister(sdata);

	if (sdata->pdata.release)
		sdata->pdata.release();

	return 0;
}

#ifdef CONFIG_PM
static int kmx62_suspend(struct device *dev)
{
	return 0;
}

static int kmx62_resume(struct device *dev)
{
	return 0;
}
#else
#define kmx62_suspend	NULL
#define kmx62_resume	NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int kmx62_runtime_suspend(struct device *dev)
{
	return 0;
}

static int kmx62_runtime_resume(struct device *dev)
{
	return 0;
}
#else
#define kmx62_runtime_suspend	NULL
#define kmx62_runtime_resume	NULL
#endif

static const struct i2c_device_id kmx62_id[] = {
			{ KMX62_DEV_NAME, 0 },
			{ },
		};

MODULE_DEVICE_TABLE(i2c, kmx62_id);

static const struct of_device_id kmx62_of_match[] = {
	{ .compatible = "kionix,kmx62", },
	{ },
};
MODULE_DEVICE_TABLE(of, kmx62_of_match);

static const struct dev_pm_ops kmx62_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(kmx62_suspend, kmx62_resume)
	SET_RUNTIME_PM_OPS(kmx62_runtime_suspend, kmx62_runtime_resume, NULL)
};

static struct i2c_driver kmx62_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name  = KMX62_DEV_NAME,
			.pm    = &kmx62_pm_ops,
			.of_match_table = kmx62_of_match,
		  },
	.probe    = kmx62_probe,
	.remove   = kmx62_remove,
	.id_table = kmx62_id,
};

module_i2c_driver(kmx62_driver);

MODULE_DESCRIPTION("kmx62 driver");
MODULE_AUTHOR("Kionix");
MODULE_LICENSE("GPL");
