/*
 * kx126: tri-axis accelerometer
 *
 * Copyright (c) 2020 Rohm Semiconductor
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
#include <linux/workqueue.h>
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
#include "kx126.h"
#include "kx126_registers.h"

/* Enable debug functionality */
#define DEBUG_SYSFS_ATTRIB
#define DEBUG_I2C_FAILS
//#define DEBUG_XYZ_DATA
#define DEBUG_STEP_DATA
#define DEBUG_DT_PARAMS
#define DEBUG_STATE

#define __debug printk
//#define __debug pr_debug

/* input dev names */
#define KX126_INPUT_DEV_ACCEL_NAME "kx126-accel"
#define KX126_INPUT_DEV_STEPCNT_NAME "kx126-stepcnt"
#define KX126_INPUT_DEV_STEPDET_NAME "kx126-stepdet"

/* Driver state bits */
#define KX126_STATE_STANDBY 0
#define KX126_STATE_STRM BIT(0)
/* pedometer step counter */
#define KX126_STATE_STEPCNT BIT(1)
/* pedometer step detector */
#define KX126_STATE_STEPDET BIT(2)

struct kx126_data {
	struct i2c_client *client;
	struct kx126_platform_data pdata;
	struct mutex mutex;

	/* accelerometer xyz data */
	struct input_dev *accel_input_dev;
	struct hrtimer accel_timer;
	int accel_wkp_flag;
	struct task_struct *accel_task;
	/* flag to use timer data read instead of drdy at high odr */
	bool poll_fast_odr; 
	wait_queue_head_t accel_wq;
	u16 accel_poll_rate;
#ifdef QCOM_SENSORS
	struct sensors_classdev accel_cdev;
#endif

	/* step counter --> prefix stepcnt_ */
	struct input_dev *stepcnt_input_dev;
	/* total steps */
	u32 stepcnt_total_steps;
	/* step counter watermark */
	u16 stepcnt_stepwm;
#ifdef QCOM_SENSORS
	struct sensors_classdev stepcnt_cdev;
#endif

	/* step detector --> prefix stepdet_ */
	struct input_dev *stepdet_input_dev;
	u16 stepdet_count;
#ifdef QCOM_SENSORS
	struct sensors_classdev stepdet_cdev;
#endif

	/* step_work used to poll steps if interrupt not used */
	struct delayed_work step_work;

	/* interrupt */
	int irq1;
	/* active interrupts: high byte INC7, low byte INC4 */
	u16 int1_enabled;

	/* odr index to odr_table */
	u16 odr_index;

	/* sensor who am i*/
	u8 wai;

	/* driver state */
	u16 state;

	bool use_poll_timer;

};

/* Poll rate / ODR defines */
#define MIN_POLL_RATE_MS 5
#define MAX_POLL_RATE_MS 1000

/* NOTE; 800,1600,3200,6400,12800,25600 odr not in table */
/* delay to odr map table */
static const struct {
	u16 cutoff;
	u16 sample_interval;
	u8 mask;
} kx126_accel_odr_table[] = {
	{ 10, 5, KX126_ODCNTL_OSA_200},
	{ 20, 10, KX126_ODCNTL_OSA_100},
	{ 40, 20, KX126_ODCNTL_OSA_50},
	{ 80, 40, KX126_ODCNTL_OSA_25},
	{ 160, 80, KX126_ODCNTL_OSA_12P5},
	{ 320, 160, KX126_ODCNTL_OSA_6P25},
	{ 640, 320, KX126_ODCNTL_OSA_3P125},
	{ 1280,	640, KX126_ODCNTL_OSA_1P563},};

#define KX126_LAST_CUTOFF 1280

/* Step counter poll delay */
#define KX126_STEP_CNT_POLL_DELAY_MS 500

#ifdef QCOM_SENSORS
/* Qualcomm sensors class defines*/

#define POLL_INTERVAL_MIN_MS MIN_POLL_RATE_MS
#define POLL_DEFAULT_INTERVAL_MS 200

#define KX126_ACCEL_NAME "kx126-accel"
#define KX126_SENSOR_POWER "0.145"

/* 8g*9.806 m/s2 */
#define KX126_8G_RANGE "78.448"
/* 0.00024g*9.806 m/s2 */
#define KX126_8G_RESOLUTION "0.002353"
/* 4g*9.806 m/s2 */
#define KX126_4G_RANGE "39.224"
/* 0.00012g*9.806 m/s2 */
#define KX126_4G_RESOLUTION "0.001176"
/* 2g*9.806 m/s2 */
#define KX126_2G_RANGE "19.612"
/* 0.00006g*9.806 m/s2 */
#define KX126_2G_RESOLUTION "0.000588"

static struct sensors_classdev accel_sensors_cdev = {
	.name = KX126_ACCEL_NAME,
	.vendor = "Kionix",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = KX126_8G_RANGE,
	.resolution = KX126_8G_RESOLUTION,
	.sensor_power = KX126_SENSOR_POWER,	/* typical value milliAmps*/
	.min_delay = POLL_INTERVAL_MIN_MS * 1000,/* in microseconds */
	.max_delay = MAX_POLL_RATE_MS,/* in milliseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = POLL_DEFAULT_INTERVAL_MS,	/* in millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

/* step counter */
#define KX126_STEPCNT_NAME "kx126-stepcnt"

static struct sensors_classdev stepcnt_sensors_cdev = {
	.name = KX126_STEPCNT_NAME,
	.vendor = "Kionix",
	.version = 1,
	.handle = SENSOR_TYPE_STEP_COUNTER,
	.type = SENSOR_TYPE_STEP_COUNTER,
	.max_range = "1",
	.resolution = "1",
	.sensor_power = KX126_SENSOR_POWER,	/* typical value milliAmps*/
	.min_delay = 0,/* in microseconds */
	.max_delay = 0,/* in milliseconds */
//	.flags = ON_CHANGE;
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 0,	/* in millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

/* step detector */
#define KX126_STEPDET_NAME "kx126-stepdet"

static struct sensors_classdev stepdet_sensors_cdev = {
	.name = KX126_STEPDET_NAME,
	.vendor = "Kionix",
	.version = 1,
	.handle = SENSOR_TYPE_STEP_DETECTOR,
	.type = SENSOR_TYPE_STEP_DETECTOR,
	.max_range = "1",
	.resolution = "1",
	.sensor_power = KX126_SENSOR_POWER,	/* typical value milliAmps*/
	.min_delay = 0,/* in microseconds */
	.max_delay = 0,/* in milliseconds */
//	.flags = ON_CHANGE;
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 0,	/* in millisecond */
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

#endif

/* Sensor functions */
static int kx126_sensor_set_odr(struct kx126_data *sdata, u16 odr_index);
static int kx126_sensor_set_grange(struct kx126_data *sdata, u8 grange);
static int kx126_sensor_read_xyz(struct kx126_data *sdata, int *xyz);

/* Driver/Sensor feature update functions */
static int kx126_state_update(struct kx126_data *sdata, u16 state, bool enable);
static int kx126_state_enable(struct kx126_data *sdata, u16 state);
static int kx126_state_disable(struct kx126_data *sdata, u16 state);

/* Accelerometer data stream functions */
static int kx126_strm_enable(struct kx126_data *sdata);
static int kx126_strm_disable(struct kx126_data *sdata);
static int kx126_strm_set_delay(struct kx126_data *sdata, u16 delay);
static int kx126_strm_read_and_report_xyz(struct kx126_data *sdata, ktime_t ts);

/* step counter */
static int kx126_stepcnt_enable(struct kx126_data *sdata);
static int kx126_stepcnt_disable(struct kx126_data *sdata);
static int kx126_stepcnt_read_steps(struct kx126_data *sdata);
static int kx126_stepcnt_set_stepwm(struct kx126_data *sdata, u16 stepwm);
static int kx126_stepcnt_read_and_report_steps(struct kx126_data *sdata, ktime_t ts);

/* step detector */
static int kx126_stepdet_enable(struct kx126_data *sdata);
static int kx126_stepdet_disable(struct kx126_data *sdata);
static int kx126_stepdet_report_step(struct kx126_data *sdata, ktime_t ts);

/* Pedometer engine (step counter + detector) functions */
static int kx126_step_engine_enable(struct kx126_data *sdata, u16 state);
static int kx126_step_engine_disable(struct kx126_data *sdata, u16 state);

/* Step counter poll work. Used in poll mode. */
static void kx126_step_work_func(struct work_struct *work);

/* Timer poll thread */
static int kx126_data_strm_thread(void *data);

/* Input dev functions */
static int kx126_strm_input_dev_register(struct kx126_data *sdata);
static void kx126_strm_input_dev_cleanup(struct kx126_data *sdata);
static void kx126_input_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts);

static int kx126_stepcnt_input_dev_register(struct kx126_data *sdata);
static void kx126_stepcnt_input_dev_cleanup(struct kx126_data *sdata);
static void kx126_stepcnt_input_report_steps(struct input_dev *dev, u32 steps, ktime_t ts);

static void kx126_stepdet_input_report_step(struct input_dev *dev, u16 count, ktime_t ts);

/* Sensor register read/write functions */
static s32 kx126_reg_write_byte(struct kx126_data *sdata, u8 reg, u8 value)
{
	s32 err;

	err = i2c_smbus_write_byte_data(sdata->client, reg, value);

#ifdef DEBUG_I2C_FAILS
	if(err < 0) {
		__debug("%s fail (%d) : reg 0x%x, value 0x%x \n",
			__FUNCTION__, err, reg, value);
	}
#endif

	return err;
}
static s32 kx126_reg_read_byte(struct kx126_data *sdata, u8 reg)
{
	s32 reg_val;

	reg_val = i2c_smbus_read_byte_data(sdata->client, reg);

#ifdef DEBUG_I2C_FAILS
	if(reg_val < 0) {
		__debug("%s fail (%d) : reg 0x%x \n", __FUNCTION__,
			reg_val, reg);
	}
#endif

	return reg_val;
}

static s32 kx126_reg_read_block(struct kx126_data *sdata, u8 reg, u8 size, u8 *data)
{
	s32 read_size;
	int err;

	read_size = i2c_smbus_read_i2c_block_data(sdata->client, reg,
		size, data);

#ifdef DEBUG_I2C_FAILS
	if (read_size != size) {
		__debug("%s fail : reg 0x%x bytes %d != %d \n", __FUNCTION__,
			reg, size, read_size);
	}
#endif
	if (read_size != size)
		err = -EIO;
	else
		err = 0;

	return err;
}

static int kx126_reg_set_bit_pattern(struct kx126_data *sdata,
		u8 reg, u8 bit_pattern, u8 mask)
{
	s32 reg_val;

	reg_val = kx126_reg_read_byte(sdata, reg);

	if (reg_val < 0)
		return reg_val;

	reg_val &= ~mask;
	reg_val |= bit_pattern;

	reg_val = kx126_reg_write_byte(sdata, reg, reg_val);

	if (reg_val < 0)
		return reg_val;

	return 0;
}

static inline int kx126_reg_set_bit(struct kx126_data *sdata, u8 reg, u8 bits)
{
	return kx126_reg_set_bit_pattern(sdata, reg, bits, 0x00);
}

static inline int kx126_reg_reset_bit(struct kx126_data *sdata, u8 reg, u8 bits)
{
	return kx126_reg_set_bit_pattern(sdata, reg, 0x00, bits);
}

/* Sensor read/write functions */

/* Limits delay to min/max range and maps delay to odr index. */
static void kx126_map_delay_to_odr(u16 delay_in, u16 *delay_out, u16 *odr_index)
{

	u16 i;

	*delay_out = delay_in;

	/* min/max check delay_out */
	if (*delay_out < MIN_POLL_RATE_MS)
 		*delay_out = MIN_POLL_RATE_MS;
	else if(*delay_out > MAX_POLL_RATE_MS)
		*delay_out = MAX_POLL_RATE_MS;

	/* get odr index based on delay value */
	for (i = 0; i < ARRAY_SIZE(kx126_accel_odr_table); i++) {
		if (*delay_out < kx126_accel_odr_table[i].cutoff ||
			kx126_accel_odr_table[i].cutoff == KX126_LAST_CUTOFF) {
			break;
			}
	}

	/* odr index and poll rate */
	*odr_index = i;

	__debug("%s - delay ms = %u cutoff = %u mask = 0x%x, interval ms = %d\n",
		__FUNCTION__, *delay_out, kx126_accel_odr_table[i].cutoff,
		kx126_accel_odr_table[i].cutoff,
		kx126_accel_odr_table[i].sample_interval );


	return;
}

static int kx126_sensor_set_odr(struct kx126_data *sdata, u16 odr_index)
{
	int err;

	if ( odr_index > (ARRAY_SIZE(kx126_accel_odr_table) - 1) )
		return -EINVAL;

	err = kx126_reg_set_bit_pattern(sdata, KX126_ODCNTL, 
		kx126_accel_odr_table[odr_index].mask, KX126_ODCNTL_OSA_MASK);

	return err;
}

static int kx126_sensor_set_grange(struct kx126_data *sdata, u8 grange)
{
	int err;
	u8 grange_bits;

	if (grange == 2)
		grange_bits = KX126_CNTL1_GSEL_2G;
	else if (grange == 4)
		grange_bits = KX126_CNTL1_GSEL_4G;
	else if (grange == 8)
		grange_bits = KX126_CNTL1_GSEL_8G;
	else
		return -EINVAL;;

	err = kx126_reg_set_bit_pattern(sdata,
		KX126_CNTL1, grange_bits, KX126_CNTL1_GSEL_MASK);

	return err;
}

static int kx126_sensor_read_xyz(struct kx126_data *sdata, int *xyz)
{
	int err;
	s16 raw_xyz[3] = { 0 };

	err = kx126_reg_read_block(sdata, KX126_XOUT_L, 6, (u8*)raw_xyz);

	if (err)
		return err;

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

static int kx126_sensor_read_steps(struct kx126_data *sdata, u16 *new_steps)
{
	int err;

	err = kx126_reg_read_block(sdata, KX126_PED_STP_L, 2, (u8*)new_steps);

	if (err) {
		*new_steps = 0;
		return err;
	}

	*new_steps = (u16) le16_to_cpu( *new_steps ); 
	
	return 0;
}

static int kx126_sensor_set_stepwm(struct kx126_data *sdata, u16 wm)
{
	int err;
	u8 reg_val;

	reg_val = 0x00ff & wm;

	err = kx126_reg_write_byte(sdata, KX126_PED_STPWM_L, reg_val);
	if (err)
		return err;

	reg_val = 0x00ff & (wm >> 8);

	err = kx126_reg_write_byte(sdata, KX126_PED_STPWM_H, reg_val);

	return err;
}

static int kx126_stepcnt_read_and_report_steps(struct kx126_data *sdata, ktime_t ts)
{
	int err;

	err = kx126_stepcnt_read_steps(sdata);
	if (err)
		return err;

	kx126_stepcnt_input_report_steps(sdata->stepcnt_input_dev,
		sdata->stepcnt_total_steps, ts);

	return 0;
}

static int kx126_stepdet_report_step(struct kx126_data *sdata, ktime_t ts)
{
	sdata->stepdet_count++;
	kx126_stepdet_input_report_step(sdata->stepdet_input_dev,
		sdata->stepdet_count, ts);

	return 0;
}

static int kx126_sensor_ped_config_at_100hz(struct kx126_data *sdata)
{
	int err;

	__debug("%s \n", __FUNCTION__);

	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL1, 0x66);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL2, 0x2C);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL3, 0x17);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL4, 0x1F);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL5, 0x24);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL6, 0x13);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL7, 0x0b);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL8, 0x08);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL9, 0x19);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL10, 0x1c);

	return err;
}

#if 0
static int kx126_sensor_ped_config_at_50hz(struct kx126_data *sdata)
{
	int err;

	__debug("%s \n", __FUNCTION__);

	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL1, 0x46);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL2, 0x26);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL3, 0x17);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL4, 0x1F);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL5, 0x10);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL6, 0x0A);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL7, 0x0B);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL8, 0x05);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL9, 0x10);
	if (err)
		return err;
	err = kx126_reg_write_byte(sdata, KX126_PED_CNTL10, 0x0A);

	return err;
}
#endif

static int kx126_sensor_ped_config(struct kx126_data *sdata)
{
	int err;
	/* Configure sensor pedometer based on Kionix application note: */
	/* AN073 Getting Started with Pedometer */

	/* step a) sensor to standby is done on kx126_state_update */
	/* step b) see kx126_sensor_set_stepwm */
	/* step c) set in step h)*/
	/* step d) */
	err = kx126_reg_write_byte(sdata, KX126_LP_CNTL, 0x7B);
	if (err)
		return err;
	/* step e,f,g) see kx126_enable_int1_routing */
	/* step h) select Pedometer at 100Hz or 50Hz */
	err = kx126_sensor_ped_config_at_100hz(sdata);
	//err = kx126_sensor_ped_config_at_50hz(sdata);

	return err;
}

/* Writes initial register config to sensor. */
static int kx126_set_sensor_initial_config(struct kx126_data *sdata)
{
	int err;

	err = kx126_sensor_set_odr(sdata, sdata->odr_index);
	if (err < 0)
		return err;

	err = kx126_sensor_set_grange(sdata, sdata->pdata.g_range);
	if (err < 0)
		return err;

	/* config sensor int1 interrupt: active high and latched */
	err = kx126_reg_write_byte(sdata, KX126_INC1, KX126_INC1_IEA1|KX126_INC1_IEL1);

	return err;
}

#define KX126_SOFT_RESET_WAIT_TIME_MS 2
#define KX126_SOFT_RESET_TOTAL_WAIT_TIME_MS 100

static int kx126_sensor_soft_reset(struct kx126_data *sdata)
{
	int err;
	int max_wait_time = KX126_SOFT_RESET_TOTAL_WAIT_TIME_MS;

	/* reboot sensor */
	err = kx126_reg_write_byte(sdata, KX126_CNTL2, KX126_CNTL2_SRST);

	if (err < 0)
		return err;
	
	/* wait sensor reboot to finish */
	while (max_wait_time > 0) {

		mdelay(KX126_SOFT_RESET_WAIT_TIME_MS);
		max_wait_time -= KX126_SOFT_RESET_WAIT_TIME_MS;
		
		err = kx126_reg_read_byte(sdata, KX126_CNTL2);

		if (err < 0)
			return err;

		if (err & KX126_CNTL2_SRST) {
			/* reboot not ready */
			err = -ETIME;
		}
		else {
			/* reboot done */
			err = 0;
			break;
		}
	}

	return err;
}

static int kx126_hw_detect(struct kx126_data *sdata)
{
	int err;

	err = kx126_reg_read_byte(sdata, KX126_WHO_AM_I);
	if (err < 0)
		return err;

	__debug("%s WHO_AM_I = 0x%x\n", __FUNCTION__, err);

	if (err != KX126_WHO_AM_I_WAI_ID)
		return -ENODEV;

	sdata->wai = err;

	dev_info(&sdata->client->dev, "Kionix kx126 detected\n");

	return 0;
}

/* Interrupt handling */

#define KX126_INT1_ROUTING_MASK 0x0fff
#define KX126_INT_NONE 0

static int kx126_enable_int1_routing(struct kx126_data *sdata, u16 int1_bits)
{
	int err;
	u8 value;

	/* INC4 routing low byte */
	value = 0x00ff & int1_bits;
	err = kx126_reg_set_bit(sdata, KX126_INC4, value);
	if (err)
		return err;

	/* INC7 step counter routing high byte, first nibble */
	value = 0x000f & (int1_bits >> 8) ;
	err = kx126_reg_set_bit(sdata, KX126_INC7, value);
	if (err)
		return err;

	if (sdata->int1_enabled == KX126_INT_NONE) {
		__debug("%s - enable int1\n", __FUNCTION__);
		/* enable sensor int1 */
		err = kx126_reg_set_bit(sdata, KX126_INC1, KX126_INC1_IEN1);
		if (err)
			return err;

		/* enable host int */
		enable_irq(sdata->irq1);
	}

	sdata->int1_enabled |= KX126_INT1_ROUTING_MASK & int1_bits;

	__debug("%s - int1_enabled 0x%x \n", __FUNCTION__, sdata->int1_enabled);

	return 0;
}

static int kx126_disable_int1_routing(struct kx126_data *sdata, u16 int_bits)
{
	int err;
	u8 value;

	/* INC4 routing low byte */
	value = 0x00ff & int_bits;
	err = kx126_reg_reset_bit(sdata, KX126_INC4, value);
	if (err)
		return err;

	/* INC7 step counter routing hi byte first nibble */
	value = 0x000f & (int_bits >> 8) ;
	err = kx126_reg_reset_bit(sdata, KX126_INC7, value);
	if (err)
		return err;

	sdata->int1_enabled &= ~(KX126_INT1_ROUTING_MASK & int_bits);

	if (sdata->int1_enabled == KX126_INT_NONE) {
		__debug("%s - disable int1\n", __FUNCTION__);

		/* disable sensor int1 */
		err = kx126_reg_reset_bit(sdata, KX126_INC1, KX126_INC1_IEN1);
		if (err)
			return err;

		/* disable host int */
		disable_irq(sdata->irq1);
	}

	__debug("%s - int1_enabled 0x%x \n", __FUNCTION__, sdata->int1_enabled);

	return 0;
}

/* defines to get correct INS value from ins table */
#define INS1 0
#define INS2 1
#define INS3 2

static void kx126_ins_handler(struct kx126_data *sdata, u8 *ins, ktime_t ts)
{
	/* xyz data ins bits */
	if (ins[INS2] & KX126_INS2_DRDY) {
		kx126_strm_read_and_report_xyz(sdata, ts);
	}

	/* Sensor step inc interrupt is trigger for both */
	/* step counter and step detector events. */
	if (ins[INS2] & KX126_INS2_STPINCI) {
		__debug("%s - step INS2_STPINCI \n", __FUNCTION__);
		if (sdata->state & KX126_STATE_STEPDET)
			(void)kx126_stepdet_report_step(sdata, ts);
		if (sdata->state & KX126_STATE_STEPCNT)
			(void)kx126_stepcnt_read_and_report_steps(sdata, ts);
	}

	if (ins[INS1] & KX126_INS1_STPOVI) {
		__debug("%s - step INS1_STPOVI \n", __FUNCTION__);
	}

	if (ins[INS1] & KX126_INS1_STPWMI) {
		__debug("%s - step INS1_STPWMI \n", __FUNCTION__);

		(void)kx126_stepcnt_read_and_report_steps(sdata, ts);
		// NOTE int line stays active if STPWMI is not set again
		(void)kx126_stepcnt_set_stepwm(sdata, sdata->stepcnt_stepwm);
	}

	/* release int */
	(void)kx126_reg_read_byte(sdata, KX126_INT_REL);

	return;
}

static irqreturn_t kx126_irq_handler(int irq, void *dev)
{
	struct kx126_data *sdata = dev;
	int err;
	u8 ins[3];
	ktime_t ts = ktime_get_boottime();

	if (irq != sdata->irq1)
		return IRQ_HANDLED;

	err = kx126_reg_read_block(sdata, KX126_INS1, 3, (u8*)ins);
	if (err)
		return IRQ_HANDLED;
	
	kx126_ins_handler(sdata, ins, ts);

	return IRQ_HANDLED;
}

/* Driver state handle functions */

/* Toggles sensor PC1 bit and updates state */
static int kx126_state_update(struct kx126_data *sdata, u16 state, bool enable)
{
	int err = 0;

#ifdef DEBUG_STATE
	/* debug state on entry */
	{
	s32 reg_val;
	reg_val = kx126_reg_read_byte(sdata, KX126_CNTL1);

	__debug("%s > state,CNTL1(0x%x,0x%x) in:(0x%x, %d)\n", 
		__FUNCTION__, sdata->state, reg_val, state, enable);
	}
#endif

	/* set sensor to standby, PC1 bit to zero before changes */
	err = kx126_reg_reset_bit(sdata, KX126_CNTL1, KX126_CNTL1_PC1);

	if (err)
		goto exit_out;

	if (enable)
		err = kx126_state_enable(sdata, state);
	else
		err = kx126_state_disable(sdata, state);
	if (err)
		goto exit_out;

	/* apply changes set PC1 bit to one */
	err = kx126_reg_set_bit(sdata, KX126_CNTL1, KX126_CNTL1_PC1);

exit_out:
	if (err)
		__debug("%s - exit_out fails err = %d \n", __FUNCTION__, err);
	/* sensor to standby if no active features */
	if (sdata->state == KX126_STATE_STANDBY)
		err = kx126_reg_reset_bit(sdata, KX126_CNTL1, KX126_CNTL1_PC1);

#ifdef DEBUG_STATE
	/* debug state on exit */
	{
	s32 reg_val;
	reg_val = kx126_reg_read_byte(sdata, KX126_CNTL1);
	__debug("%s < state,CNTL1(0x%x,0x%x)\n", 
		__FUNCTION__, sdata->state, reg_val);
	}
#endif

	return err;
}

static int kx126_state_enable(struct kx126_data *sdata, u16 state)
{
	int err = 0;

	if (KX126_STATE_STRM == state)
		err = kx126_strm_enable(sdata);
	else if (KX126_STATE_STEPCNT == state)
		err = kx126_stepcnt_enable(sdata);
	else if (KX126_STATE_STEPDET == state)
		err = kx126_stepdet_enable(sdata);

	return err;
}

static int kx126_state_disable(struct kx126_data *sdata, u16 state)
{
	int err = 0;

	if (KX126_STATE_STRM == state)
		err = kx126_strm_disable(sdata);
	else if (KX126_STATE_STEPCNT == state)
		err = kx126_stepcnt_disable(sdata);
	else if (KX126_STATE_STEPDET == state)
		err = kx126_stepdet_disable(sdata);

	return err;
}

/* Senosr data stream functions */

static int kx126_strm_drdy_enable(struct kx126_data *sdata)
{
	int err;

	/* route drdy interrupt to to sensor int1 */
	err = kx126_enable_int1_routing(sdata, KX126_INC4_DRDYI1);
	if (err)
		return err;

	/* enable sensor drdy engine */
	err = kx126_reg_set_bit(sdata, KX126_CNTL1, KX126_CNTL1_DRDYE);
	if (err)
		return err;

	return 0;
}

static int kx126_strm_enable(struct kx126_data *sdata)
{
	int err;

	if (sdata->state & KX126_STATE_STRM)
		return 0;

	/* update data ODR */
	err = kx126_sensor_set_odr(sdata, sdata->odr_index);
	if (err)
		return err;

	/* check if need to use timer data read instead of drdy */
	if (!sdata->use_poll_timer && sdata->accel_poll_rate == MIN_POLL_RATE_MS)
		sdata->poll_fast_odr = true;
	else
		sdata->poll_fast_odr = false;

	/* set data reportting */
	if (sdata->use_poll_timer || sdata->poll_fast_odr) {
		ktime_t time;
		time = ktime_set(0,sdata->accel_poll_rate * NSEC_PER_MSEC);
		hrtimer_start(&sdata->accel_timer, time, HRTIMER_MODE_REL);
	} else {
		err = kx126_strm_drdy_enable(sdata);
		if (err)
			return err;
	}

	sdata->state |= KX126_STATE_STRM;

	return 0;
}

static int kx126_strm_drdy_disable(struct kx126_data *sdata)
{
	int err;

	/* route drdy interrupt to to sensor int1 */
	err = kx126_disable_int1_routing(sdata, KX126_INC4_DRDYI1);
	if (err)
		return err;

	/* disable sensor drdy engine */
	err = kx126_reg_reset_bit(sdata, KX126_CNTL1, KX126_CNTL1_DRDYE);
	if (err)
		return err;

	return 0;
}

static int kx126_strm_disable(struct kx126_data *sdata)
{
	if (!(sdata->state & KX126_STATE_STRM))
		return 0;

	/* disable data report */ 
	if (sdata->use_poll_timer || sdata->poll_fast_odr) {
		hrtimer_cancel(&sdata->accel_timer);
	} else {
		int err;
		err = kx126_strm_drdy_disable(sdata);
		if (err)
			return err;
	}

	sdata->state &= ~KX126_STATE_STRM;

	return 0;
}


static int kx126_strm_set_delay(struct kx126_data *sdata, u16 delay)
{
	int err;

	kx126_map_delay_to_odr(delay, &sdata->accel_poll_rate, &sdata->odr_index);

	if (sdata->state == KX126_STATE_STANDBY)
		return 0;

	/* sensor in operation state, toggle PC1 to change ODR */
	if (sdata->use_poll_timer || sdata->poll_fast_odr)
		hrtimer_cancel(&sdata->accel_timer);

	err = kx126_reg_reset_bit(sdata, KX126_CNTL1, KX126_CNTL1_PC1);

	if (err)
		goto exit_out;

	err = kx126_sensor_set_odr(sdata, sdata->odr_index);
	if (err)
		goto exit_out;

	/* check if need to update data read mode */
	if (!sdata->use_poll_timer) {

		/* data read mode change from drdy to poll_fast_odr */
		if (sdata->accel_poll_rate == MIN_POLL_RATE_MS &&
		    sdata->poll_fast_odr == false) {
			__debug("%s - change drdy to poll_fast_odr \n", __FUNCTION__);
			err = kx126_strm_drdy_disable(sdata);
			if (err)
				goto exit_out;
			sdata->poll_fast_odr = true;
		}

		/*  data read mode change from poll_fast_odr to drdy */
		if (sdata->accel_poll_rate != MIN_POLL_RATE_MS && 
		    sdata->poll_fast_odr == true ) {
			__debug("%s - change poll_fast_odr to drdy \n", __FUNCTION__);
			err = kx126_strm_drdy_enable(sdata);
			if (err)
				goto exit_out;
			sdata->poll_fast_odr = false;
		}

	} 

	/* start poll timer */
	if (sdata->use_poll_timer || sdata->poll_fast_odr) {
		ktime_t time;
		time = ktime_set(0,sdata->accel_poll_rate * NSEC_PER_MSEC);
		hrtimer_start(&sdata->accel_timer, time, HRTIMER_MODE_REL);
	}

	err = kx126_reg_set_bit(sdata, KX126_CNTL1, KX126_CNTL1_PC1);

	exit_out:

	return err;
}

static int kx126_strm_read_and_report_xyz(struct kx126_data *sdata, ktime_t ts)
{
	int err;
	int xyz[3];

	err = kx126_sensor_read_xyz(sdata, xyz);

	if (err)
		dev_err(&sdata->client->dev, "read_xyz fail\n");
	else
		kx126_input_report_xyz(sdata->accel_input_dev, xyz, ts);

	return 0;
}

/*** Pedometer functions ***/

static int kx126_stepcnt_read_steps(struct kx126_data *sdata)
{
	int err;
	u16 new_steps;

	err = kx126_sensor_read_steps(sdata, &new_steps);
 
	if (err)
		return err;

	sdata->stepcnt_total_steps += new_steps;

	__debug("%s - new_steps %d, stepcnt_total_steps %u\n", 
		__FUNCTION__, new_steps , sdata->stepcnt_total_steps);

	return 0;
}

static int kx126_stepcnt_set_stepwm(struct kx126_data *sdata, u16 stepwm)
{
	int err;
	ktime_t ts = ktime_get_boottime();

	sdata->stepcnt_stepwm = stepwm;

	if (sdata->state == KX126_STATE_STANDBY)
		return 0;

	/* sensor active, toggle PC1 to update step wm */
	err = kx126_reg_reset_bit(sdata, KX126_CNTL1, KX126_CNTL1_PC1);
	if (err)
		return err;

	/* report steps before updating new step wm */
	(void)kx126_stepcnt_read_and_report_steps(sdata, ts);

	/* step wm */
	err = kx126_sensor_set_stepwm(sdata, sdata->stepcnt_stepwm);
	if (err)
		return err;

	err = kx126_reg_set_bit(sdata, KX126_CNTL1, KX126_CNTL1_PC1);

	return err;
}

static int kx126_stepcnt_enable(struct kx126_data *sdata)
{
	if (sdata->state & KX126_STATE_STEPCNT)
		return 0;

	return kx126_step_engine_enable(sdata, KX126_STATE_STEPCNT);
}

static int kx126_stepcnt_disable(struct kx126_data *sdata)
{
	if (!(sdata->state & KX126_STATE_STEPCNT))
		return 0;

	return kx126_step_engine_disable(sdata, KX126_STATE_STEPCNT);
}

static int kx126_stepdet_enable(struct kx126_data *sdata)
{
	if (sdata->state & KX126_STATE_STEPDET)
		return 0;

	return kx126_step_engine_enable(sdata, KX126_STATE_STEPDET);
}

static int kx126_stepdet_disable(struct kx126_data *sdata)
{
	if (!(sdata->state & KX126_STATE_STEPDET))
		return 0;

	return kx126_step_engine_disable(sdata, KX126_STATE_STEPDET);
}

/* Sensor engines enabled when stepcounter active */
#define KX126_STEP_ENGINE_BITS (KX126_CNTL1_PDE|KX126_CNTL1_TDTE)

/* All step counter states */
#define KX126_STEP_STATES (KX126_STATE_STEPDET|KX126_STATE_STEPCNT)

/* Step engine interrupt bits */
#define KX126_STEP_INT1_BITS ((KX126_INC7_STPINCI1|KX126_INC7_STPOVI1|KX126_INC7_STPWMI1) << 8)

static int kx126_step_engine_enable(struct kx126_data *sdata, u16 step_state)
{
	bool is_first = !(KX126_STEP_STATES & sdata->state);

	__debug("%s - step_state(0x%x)\n", __FUNCTION__, step_state);

	/* clear sensor step counter if driver step counter not enabled */
	if (!(sdata->state & KX126_STATE_STEPCNT)) {
		u16 dummy_steps;
		(void)kx126_sensor_read_steps(sdata, &dummy_steps);
	}

	if (is_first) {
		int err;

		/* configure sensor pedometer */
		err = kx126_sensor_ped_config(sdata);
		if (err)
			return err;

		/* step watermark */
		err = kx126_sensor_set_stepwm(sdata, sdata->stepcnt_stepwm);
		if (err)
			return err;

		if (sdata->use_poll_timer) {
			/* step report using delayed work polling */
			unsigned long delay = msecs_to_jiffies(KX126_STEP_CNT_POLL_DELAY_MS);
			schedule_delayed_work(&sdata->step_work, delay);
		} else {
			/* route step interrupts to sensor int1 */
			err = kx126_enable_int1_routing(sdata, KX126_STEP_INT1_BITS);
			if (err)
				return err;
		}
		/* enable sensor pedometer engine */
		err = kx126_reg_set_bit(sdata, KX126_CNTL1, KX126_STEP_ENGINE_BITS);
		if (err)
			return err;
	} 

	sdata->state |= step_state;

	return 0;
}

static int kx126_step_engine_disable(struct kx126_data *sdata, u16 step_state)
{
	bool is_last = !((KX126_STEP_STATES & sdata->state) & ~step_state);

	__debug("%s - step_state(0x%x) \n", __FUNCTION__, step_state);

	/* disable pedometer engine if last pedometer feature */
	if (is_last) {
		int err;

		if (sdata->use_poll_timer) {
			cancel_delayed_work_sync(&sdata->step_work);
		} else {
			/* step report using delayed work */
			err = kx126_disable_int1_routing(sdata, KX126_STEP_INT1_BITS);
			if (err)
				return err;
		}

		err = kx126_reg_reset_bit(sdata, KX126_CNTL1, KX126_STEP_ENGINE_BITS);
		if (err)
			return err;
	}

	sdata->state &= ~step_state;

	return 0;
}

/* input dev functions */
static int kx126_strm_input_dev_register(struct kx126_data *sdata)
{
	int err;

	sdata->accel_input_dev = input_allocate_device();
	if (!sdata->accel_input_dev)
		return -ENOMEM;

	sdata->accel_input_dev->name = KX126_INPUT_DEV_ACCEL_NAME;
	sdata->accel_input_dev->id.bustype = BUS_I2C;
	sdata->accel_input_dev->id.vendor  = sdata->wai;

	sdata->accel_input_dev->dev.parent = &sdata->client->dev;

	set_bit(EV_ABS, sdata->accel_input_dev->evbit);
	input_set_abs_params(sdata->accel_input_dev, ABS_X, INT_MIN, INT_MAX,0,0);
	input_set_abs_params(sdata->accel_input_dev, ABS_Y, INT_MIN, INT_MAX,0,0);
	input_set_abs_params(sdata->accel_input_dev, ABS_Z, INT_MIN, INT_MAX,0,0);

	input_set_drvdata(sdata->accel_input_dev, sdata);

	err = input_register_device(sdata->accel_input_dev);

	if (err) {
		input_free_device(sdata->accel_input_dev);
		sdata->accel_input_dev = NULL;
		return err;
	}

	return 0;
}

static void kx126_strm_input_dev_cleanup(struct kx126_data *sdata)
{
	input_unregister_device(sdata->accel_input_dev);
}

static void kx126_input_report_xyz(struct input_dev *dev, int *xyz, ktime_t ts)
{
#ifdef DEBUG_XYZ_DATA
	__debug("x,y,z,ts:, %d, %d, %d, %lld \n", xyz[0], xyz[1], xyz[2],
		ktime_to_ms(ts) );
#endif
	input_report_abs(dev, ABS_X, xyz[0]);
	input_report_abs(dev, ABS_Y, xyz[1]);
	input_report_abs(dev, ABS_Z, xyz[2]);
	input_event(dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(ts).tv_sec);
	input_event(dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(ts).tv_nsec);
	input_sync(dev);

	return;
}

static int kx126_stepcnt_input_dev_register(struct kx126_data *sdata)
{
	int err;

	sdata->stepcnt_input_dev = input_allocate_device();
	if (!sdata->stepcnt_input_dev)
		return -ENOMEM;

	sdata->stepcnt_input_dev->name = KX126_INPUT_DEV_STEPCNT_NAME;
	sdata->stepcnt_input_dev->id.bustype = BUS_I2C;
	sdata->stepcnt_input_dev->id.vendor  = sdata->wai;

	sdata->stepcnt_input_dev->dev.parent = &sdata->client->dev;

	set_bit(EV_ABS, sdata->stepcnt_input_dev->evbit);
	input_set_abs_params(sdata->stepcnt_input_dev, ABS_MISC, INT_MIN, INT_MAX,0,0);
	input_set_drvdata(sdata->stepcnt_input_dev, sdata);

	err = input_register_device(sdata->stepcnt_input_dev);

	if (err) {
		input_free_device(sdata->stepcnt_input_dev);
		sdata->stepcnt_input_dev = NULL;
		return err;
	}

	return 0;
}

static void kx126_stepcnt_input_dev_cleanup(struct kx126_data *sdata)
{
	if (sdata->stepcnt_input_dev) {
		input_unregister_device(sdata->stepcnt_input_dev);
		sdata->stepcnt_input_dev = NULL;
	}
}

static void kx126_stepcnt_input_report_steps(struct input_dev *dev, u32 steps, ktime_t ts)
{
#ifdef DEBUG_STEP_DATA
	__debug("steps,ts: %u, %lld \n", steps, ktime_to_ms(ts) );
#endif
	input_report_abs(dev, ABS_MISC, steps);
	input_event(dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(ts).tv_sec);
	input_event(dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(ts).tv_nsec);
	input_sync(dev);

	return;
}

/* step detector input dev */

static int kx126_stepdet_input_dev_register(struct kx126_data *sdata)
{
	int err;

	sdata->stepdet_input_dev = input_allocate_device();
	if (!sdata->stepdet_input_dev)
		return -ENOMEM;

	sdata->stepdet_input_dev->name = KX126_INPUT_DEV_STEPDET_NAME;
	sdata->stepdet_input_dev->id.bustype = BUS_I2C;
	sdata->stepdet_input_dev->id.vendor  = sdata->wai;

	sdata->stepdet_input_dev->dev.parent = &sdata->client->dev;

	set_bit(EV_ABS, sdata->stepdet_input_dev->evbit);
	input_set_abs_params(sdata->stepdet_input_dev, ABS_MISC, INT_MIN, INT_MAX,0,0);
	input_set_drvdata(sdata->stepdet_input_dev, sdata);

	err = input_register_device(sdata->stepdet_input_dev);

	if (err) {
		input_free_device(sdata->stepdet_input_dev);
		sdata->stepdet_input_dev = NULL;
		return err;
	}

	return 0;
}

static void kx126_stepdet_input_dev_cleanup(struct kx126_data *sdata)
{
	if (sdata->stepdet_input_dev) {
		input_unregister_device(sdata->stepdet_input_dev);
		sdata->stepdet_input_dev = NULL;
	}
}

static void kx126_stepdet_input_report_step(struct input_dev *dev, u16 count, ktime_t ts)
{
#ifdef DEBUG_STEP_DATA
	__debug("stepdet inc: %lld \n", ktime_to_ms(ts) );
#endif
	input_report_abs(dev, ABS_MISC, count);
	input_event(dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(ts).tv_sec);
	input_event(dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(ts).tv_nsec);
	input_sync(dev);

	return;
}

static void kx126_step_work_func(struct work_struct *work)
{
	struct kx126_data *sdata = container_of((struct delayed_work *)work,
			struct kx126_data, step_work);
	unsigned long delay = msecs_to_jiffies(KX126_STEP_CNT_POLL_DELAY_MS);
	int ins2;

	ins2 = kx126_reg_read_byte(sdata, KX126_INS2);

	__debug("%s - ins2 0x%x \n", __FUNCTION__, ins2);

	if (ins2 > 0 && ins2 & KX126_INS2_STPINCI) {
		ktime_t ts = ktime_get_boottime();

		if (sdata->state & KX126_STATE_STEPDET)
			(void)kx126_stepdet_report_step(sdata, ts);

		if (sdata->state & KX126_STATE_STEPCNT)
			(void)kx126_stepcnt_read_and_report_steps(sdata, ts);
	}

	/* release int */
	(void)kx126_reg_read_byte(sdata, KX126_INT_REL);

	schedule_delayed_work(&sdata->step_work, delay);
}

/* timer handler */

static enum hrtimer_restart kx126_data_timer_handler(struct hrtimer *handle)
{
	struct kx126_data *sdata = container_of(handle,
				struct kx126_data, accel_timer);

	if (sdata->state & KX126_STATE_STRM) {
		ktime_t time;
		time = ktime_set(0,sdata->accel_poll_rate * NSEC_PER_MSEC);
		hrtimer_forward_now(&sdata->accel_timer, time);
		sdata->accel_wkp_flag = 1;
		wake_up_interruptible(&sdata->accel_wq);

		return HRTIMER_RESTART;
	} else {
		__debug("%s: no restart\n", __FUNCTION__);
		return HRTIMER_NORESTART;
	}
}

static int kx126_data_strm_thread(void *data)
{
	struct kx126_data *sdata = data;

	while (1) {
		wait_event_interruptible(sdata->accel_wq,
			((sdata->accel_wkp_flag != 0) ||
				kthread_should_stop()));
		sdata->accel_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		if (sdata->state & KX126_STATE_STRM) {
			ktime_t ts = ktime_get_boottime();
			kx126_strm_read_and_report_xyz(sdata, ts);
		}
	}

	return 0;
}

/* sysfs attributes */
static ssize_t kx126_sysfs_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kx126_data *sdata = dev_get_drvdata(dev);
	u8 enabled = sdata->state & KX126_STATE_STRM;

	return sprintf(buf, "%u\n", enabled);
}

static ssize_t kx126_sysfs_set_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kx126_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	/* use mutex and update data stream state */
	mutex_lock(&sdata->mutex);
	if (value)
		err = kx126_state_update(sdata, KX126_STATE_STRM, true);
	else
		err = kx126_state_update(sdata, KX126_STATE_STRM, false);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t kx126_sysfs_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kx126_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->accel_poll_rate);
}

static ssize_t kx126_sysfs_set_delay(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kx126_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	/* use mutex and update delay */
	mutex_lock(&sdata->mutex);
	err = kx126_strm_set_delay(sdata, (u16)value);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t kx126_sysfs_get_stepcnt_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kx126_data *sdata = dev_get_drvdata(dev);
	u8 enabled = sdata->state & KX126_STATE_STEPCNT;

	return sprintf(buf, "%u\n", enabled);
}

static ssize_t kx126_sysfs_set_stepcnt_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kx126_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	/* use mutex and update data pedometer state */
	mutex_lock(&sdata->mutex);
	if (value)
		err = kx126_state_update(sdata, KX126_STATE_STEPCNT, true);
	else
		err = kx126_state_update(sdata, KX126_STATE_STEPCNT, false);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t kx126_sysfs_get_stepcnt_steps(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kx126_data *sdata = dev_get_drvdata(dev);
	u32 steps;

	/* read steps */
	mutex_lock(&sdata->mutex);
	(void)kx126_stepcnt_read_steps(sdata);
	steps = sdata->stepcnt_total_steps;
	mutex_unlock(&sdata->mutex);

	return sprintf(buf, "%u\n", steps);
}

static ssize_t kx126_sysfs_get_stepcnt_wm(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kx126_data *sdata = dev_get_drvdata(dev);
	u16 stpwm = sdata->stepcnt_stepwm;

	return sprintf(buf, "%u\n", stpwm);
}

static ssize_t kx126_sysfs_set_stepcnt_wm(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kx126_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	/* use mutex and update step wm */
	mutex_lock(&sdata->mutex);
	err = kx126_stepcnt_set_stepwm(sdata, value);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static ssize_t kx126_sysfs_get_stepdet_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kx126_data *sdata = dev_get_drvdata(dev);
	u8 enabled = sdata->state & KX126_STATE_STEPDET;

	return sprintf(buf, "%u\n", enabled);
}

static ssize_t kx126_sysfs_set_stepdet_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kx126_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	/* use mutex and update pedometer state */
	mutex_lock(&sdata->mutex);
	if (value)
		err = kx126_state_update(sdata, KX126_STATE_STEPDET, true);
	else
		err = kx126_state_update(sdata, KX126_STATE_STEPDET, false);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}


#ifdef DEBUG_SYSFS_ATTRIB
static ssize_t kx126_sysfs_reg_read(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct kx126_data *sdata = dev_get_drvdata(dev);
	u8 reg;
	s32 reg_val;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	reg = 0xff & value;

	mutex_lock(&sdata->mutex);
	reg_val = kx126_reg_read_byte(sdata, reg);
	mutex_unlock(&sdata->mutex);

	if (reg_val < 0) {
		err = reg_val;
		__debug("%s reg 0x%x fail - %d \n", __FUNCTION__,reg, reg_val);
	} else {
		err = 0;
		__debug("%s reg 0x%x val 0x%x \n", __FUNCTION__,reg, reg_val);
	}

	return (err < 0) ? err : len;
}

static ssize_t kx126_sysfs_reg_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct kx126_data *sdata = dev_get_drvdata(dev);
	int err;
	unsigned long value;
	u8 reg;
	u8 reg_val;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	reg = 0xff & value;
	reg_val = 0xff & (value >> 8);

	mutex_lock(&sdata->mutex);
	err = kx126_reg_write_byte(sdata, reg, reg_val);
	mutex_unlock(&sdata->mutex);

	if (err < 0)
		__debug("%s reg 0x%x fail - %d \n", __FUNCTION__, reg, err);
	else
		__debug("%s reg 0x%x val 0x%x \n",__FUNCTION__, reg, reg_val);

	return (err < 0) ? err : len;
}

#endif

static struct device_attribute dev_attr_accel_enable = __ATTR(enable,
		S_IRUGO | S_IWUSR,
		kx126_sysfs_get_enable,
		kx126_sysfs_set_enable);

static struct device_attribute dev_attr_accel_delay = __ATTR(delay,
		S_IRUGO | S_IWUSR,
		kx126_sysfs_get_delay,
		kx126_sysfs_set_delay);

static struct device_attribute dev_attr_stepcnt_enable = __ATTR(stepcnt_enable,
		S_IRUGO | S_IWUSR,
		kx126_sysfs_get_stepcnt_enable,
		kx126_sysfs_set_stepcnt_enable);

static struct device_attribute dev_attr_stepcnt_steps = __ATTR(stepcnt_steps,
		S_IRUGO,
		kx126_sysfs_get_stepcnt_steps,
		NULL);

static struct device_attribute dev_attr_stepcnt_wm = __ATTR(stepcnt_wm,
		S_IRUGO | S_IWUSR,
		kx126_sysfs_get_stepcnt_wm,
		kx126_sysfs_set_stepcnt_wm);

static struct device_attribute dev_attr_stepdet_enable = __ATTR(stepdet_enable,
		S_IRUGO | S_IWUSR,
		kx126_sysfs_get_stepdet_enable,
		kx126_sysfs_set_stepdet_enable);

#ifdef DEBUG_SYSFS_ATTRIB
static struct device_attribute dev_attr_reg_read = __ATTR(reg_read,
		S_IWUSR,
		NULL,
		kx126_sysfs_reg_read);

static struct device_attribute dev_attr_reg_write = __ATTR(reg_write,
		S_IWUSR,
		NULL,
		kx126_sysfs_reg_write);

#endif

/* accel sysfs attrs */
static struct attribute *kx126_sysfs_attrs[] = {
	&dev_attr_accel_enable.attr,
	&dev_attr_accel_delay.attr,
#ifdef DEBUG_SYSFS_ATTRIB
	&dev_attr_reg_read.attr,
	&dev_attr_reg_write.attr,
#endif
	NULL
};

static struct attribute_group kx126_accel_attribute_group = {
	.attrs = kx126_sysfs_attrs
};

/* step detector sysfs attrs */
static struct attribute *kx126_stepdet_sysfs_attrs[] = {
	&dev_attr_stepdet_enable.attr,
	NULL
};

static struct attribute_group kx126_stepdet_attribute_group = {
	.attrs = kx126_stepdet_sysfs_attrs
};


/* step counter sysfs attrs */
static struct attribute *kx126_stepcnt_sysfs_attrs[] = {
	&dev_attr_stepcnt_enable.attr,
	&dev_attr_stepcnt_steps.attr,
	&dev_attr_stepcnt_wm.attr,
	&dev_attr_stepdet_enable.attr,
	NULL
};

static struct attribute_group kx126_stepcnt_attribute_group = {
	.attrs = kx126_stepcnt_sysfs_attrs
};

#ifdef QCOM_SENSORS
/* sensor class interface */
static int kx126_cdev_sensors_enable(struct sensors_classdev *sensors_cdev,
					unsigned int enable)
{
	struct kx126_data *sdata = container_of(sensors_cdev,
						struct kx126_data,
						accel_cdev);
	int err;

	__debug("%s - enable %d\n", __FUNCTION__, enable);

	mutex_lock(&sdata->mutex);
	err = kx126_state_update(sdata, KX126_STATE_STRM, enable);
	mutex_unlock(&sdata->mutex);

	return err;
}

static int kx126_cdev_sensors_poll_delay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_ms)
{
	struct kx126_data *sdata = container_of(sensors_cdev,
						struct kx126_data,
						accel_cdev);
	int err;

	__debug("%s - delay_ms %d\n", __FUNCTION__, delay_ms);

	mutex_lock(&sdata->mutex);
	err = kx126_strm_set_delay(sdata, delay_ms);
	mutex_unlock(&sdata->mutex);

	return err;
}

static int kx126_sensors_cdev_register(struct kx126_data *sdata)
{
	int err;

	/* accel sensor class cdev */
	sdata->accel_cdev = accel_sensors_cdev;
	sdata->accel_cdev.sensors_enable = kx126_cdev_sensors_enable;
	sdata->accel_cdev.sensors_poll_delay = kx126_cdev_sensors_poll_delay;
	sdata->accel_cdev.sensors_calibrate = NULL;
	sdata->accel_cdev.sensors_write_cal_params = NULL;

	/* sensor name */
	sdata->accel_cdev.name = KX126_ACCEL_NAME;

	/* grange and data resolution */
	if (sdata->pdata.g_range == KX126_ACC_RANGE_2G) {
		sdata->accel_cdev.max_range = KX126_2G_RANGE;
		sdata->accel_cdev.resolution = KX126_2G_RESOLUTION;
	}
	else if (sdata->pdata.g_range == KX126_ACC_RANGE_4G){
		sdata->accel_cdev.max_range = KX126_4G_RANGE;
		sdata->accel_cdev.resolution = KX126_4G_RESOLUTION;
	}
	else if (sdata->pdata.g_range == KX126_ACC_RANGE_8G){
		sdata->accel_cdev.max_range = KX126_8G_RANGE;
		sdata->accel_cdev.resolution = KX126_8G_RESOLUTION;
	}
	else {
		sdata->accel_cdev.max_range = KX126_4G_RANGE;
		sdata->accel_cdev.resolution = KX126_4G_RESOLUTION;
	}

	err = sensors_classdev_register(&sdata->accel_input_dev->dev,
		&sdata->accel_cdev);

	return err;
}

static void kx126_sensors_cdev_unregister(struct kx126_data *sdata)
{
	sensors_classdev_unregister(&sdata->accel_cdev);
}

/* step counter sensors cdev */
static int kx126_stepcnt_cdev_sensors_enable(struct sensors_classdev *sensors_cdev,
					unsigned int enable)
{
	struct kx126_data *sdata = container_of(sensors_cdev,
						struct kx126_data,
						stepcnt_cdev);
	int err;

	__debug("%s - enable %d\n", __FUNCTION__, enable);

	mutex_lock(&sdata->mutex);
	err = kx126_state_update(sdata, KX126_STATE_STEPCNT, enable);
	mutex_unlock(&sdata->mutex);

	return err;
}

static int kx126_stepcnt_sensors_cdev_register(struct kx126_data *sdata)
{
	int err;

	/* step counter sensor class cdev */
	sdata->stepcnt_cdev = stepcnt_sensors_cdev;
	sdata->stepcnt_cdev.sensors_enable = kx126_stepcnt_cdev_sensors_enable;
	sdata->stepcnt_cdev.sensors_poll_delay = NULL;
	sdata->stepcnt_cdev.sensors_calibrate = NULL;
	sdata->stepcnt_cdev.sensors_write_cal_params = NULL;

	err = sensors_classdev_register(&sdata->stepcnt_input_dev->dev,
		&sdata->stepcnt_cdev);

	return err;
}

static void kx126_stepcnt_sensors_cdev_unregister(struct kx126_data *sdata)
{
	sensors_classdev_unregister(&sdata->stepcnt_cdev);
}

/* step detector sensors cdev */
static int kx126_stepdet_cdev_sensors_enable(struct sensors_classdev *sensors_cdev,
					unsigned int enable)
{
	struct kx126_data *sdata = container_of(sensors_cdev,
						struct kx126_data,
						stepdet_cdev);
	int err;

	__debug("%s - enable %d\n", __FUNCTION__, enable);

	mutex_lock(&sdata->mutex);
	err = kx126_state_update(sdata, KX126_STATE_STEPDET, enable);
	mutex_unlock(&sdata->mutex);

	return err;
}

static int kx126_stepdet_sensors_cdev_register(struct kx126_data *sdata)
{
	int err;

	/* step detector sensor class cdev */
	sdata->stepdet_cdev = stepdet_sensors_cdev;
	sdata->stepdet_cdev.sensors_enable = kx126_stepdet_cdev_sensors_enable;
	sdata->stepdet_cdev.sensors_poll_delay = NULL;
	sdata->stepdet_cdev.sensors_calibrate = NULL;
	sdata->stepdet_cdev.sensors_write_cal_params = NULL;

	err = sensors_classdev_register(&sdata->stepdet_input_dev->dev,
		&sdata->stepdet_cdev);

	return err;
}

static void kx126_stepdet_sensors_cdev_unregister(struct kx126_data *sdata)
{
	sensors_classdev_unregister(&sdata->stepdet_cdev);
}
#endif

#ifdef CONFIG_OF
static int kx126_parse_dt(struct kx126_data *sdata, struct device *dev)
{
	int err;
	u32 temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,x-map", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property x-map. Use default 0.\n");
		temp_val = 0;
	}
	sdata->pdata.x_map = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,y-map", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property y-map. Use default 1.\n");
		temp_val = 1;
	}
	sdata->pdata.y_map = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,z-map", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property z-map. Use default 2.\n");
		temp_val = 2;
	}
	sdata->pdata.z_map = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,x-negate", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property x-negate. Use default 0.\n");
		temp_val = 0;
	}
	sdata->pdata.x_negate = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,y-negate", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property y-negate. Use default 0.\n");
		temp_val = 0;
	}
	sdata->pdata.y_negate = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,z-negate", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property z-negate. Use default 0.\n");
		temp_val = 0;
	}
	sdata->pdata.z_negate = (u8) temp_val;

	err = of_property_read_u32(dev->of_node, "kionix,g-range", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property g-range. Use default 8g\n");
		temp_val = 8;
	}
	sdata->pdata.g_range = temp_val;

	/* optional dt parameters i.e. use poll if gpio-int not found */
	sdata->pdata.gpio_int1 = of_get_named_gpio_flags(dev->of_node,
		"kionix,gpio-int1", 0, NULL);

	sdata->pdata.use_drdy_int = of_property_read_bool(dev->of_node,
		"kionix,use-drdy-int");

#ifdef DEBUG_DT_PARAMS
	__debug("sdata->pdata.x_map %d\n", sdata->pdata.x_map);
	__debug("sdata->pdata.y_map %d\n", sdata->pdata.y_map);
	__debug("sdata->pdata.z_map %d\n", sdata->pdata.z_map);
	__debug("sdata->pdata.x_negate %d\n", sdata->pdata.x_negate);
	__debug("sdata->pdata.y_negate %d\n", sdata->pdata.y_negate);
	__debug("sdata->pdata.z_negate %d\n", sdata->pdata.z_negate);
	__debug("sdata->pdata.g_range %d\n", sdata->pdata.g_range);
	__debug("sdata->pdata.gpio_int1 %d\n", sdata->pdata.gpio_int1); 
	__debug("sdata->pdata.use_drdy_int %d\n", sdata->pdata.use_drdy_int); 
#endif

	return 0;
}
#else
static int kx126_parse_dt(struct kx126_data *sdata, struct device *dev)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */

/* probe */
static int kx126_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{	
	struct kx126_data *sdata;
	int err;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev,
			"missing required i2c functionality\n");
		return -ENODEV;
	}

	sdata = devm_kzalloc(&client->dev,
			sizeof(struct kx126_data), GFP_KERNEL);
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
		err = kx126_parse_dt(sdata, &client->dev);
		if (err) {
			dev_err(&client->dev,
				"Unable to parse dt data err=%d\n", err);
			err = -EINVAL;
			goto free_init;
		}
	} else if (client->dev.platform_data) {
		sdata->pdata = *(struct kx126_platform_data *)client->dev.platform_data;
	} else {
		dev_err(&client->dev,"platform data is NULL\n");
		err = -EINVAL;
		goto free_init;
	}

	err = kx126_hw_detect(sdata);
	if (err) {
		dev_err(&client->dev, "sensor not recognized\n");
		goto free_init;
	}

	err = kx126_sensor_soft_reset(sdata);
	if (err) {
		dev_err(&client->dev, "soft reset failed\n");
		goto free_init;
	}

	sdata->state = KX126_STATE_STANDBY;

	if (sdata->pdata.init) {
		err = sdata->pdata.init();
		if (err) {
			dev_err(&client->dev, "pdata.init() call failed \n");
			goto free_init;
		}
	}

	/* init accelerometer input dev */
	err = kx126_strm_input_dev_register(sdata);
	if (err < 0) {
		dev_err(&client->dev,
			"accel input register fail\n");
		goto free_src;
	}
	err = sysfs_create_group(&sdata->accel_input_dev->dev.kobj,
					&kx126_accel_attribute_group);
	if (err) {
		dev_err(&client->dev,
			"accel sysfs create fail\n");
		goto free_input_accel;
	}
	/* init stepdet input dev */
	err = kx126_stepdet_input_dev_register(sdata);
	if (err < 0) {
		dev_err(&client->dev,
			"stepdet input register fail\n");
		goto free_sysfs_accel;
	}
	err = sysfs_create_group(&sdata->stepdet_input_dev->dev.kobj,
					&kx126_stepdet_attribute_group);
	if (err) {
		dev_err(&client->dev,
			"stepcnt sysfs create fail\n");
		goto free_input_stepdet;
	}

	/* init stepcnt input dev */
	err = kx126_stepcnt_input_dev_register(sdata);
	if (err < 0) {
		dev_err(&client->dev,
			"stepcnt input register fail\n");
		goto free_sysfs_stepdet;
	}
	err = sysfs_create_group(&sdata->stepcnt_input_dev->dev.kobj,
					&kx126_stepcnt_attribute_group);
	if (err) {
		dev_err(&client->dev,
			"stepcnt sysfs create fail\n");
		goto free_input_stepcnt;
	}

	/* gpio to irq */
	if (sdata->pdata.use_drdy_int && gpio_is_valid(sdata->pdata.gpio_int1)) {
		err = gpio_request(sdata->pdata.gpio_int1, "kx126_int1");
		if (err) {
			dev_err(&client->dev,
				"Unable to request interrupt gpio1 %d\n",
				sdata->pdata.gpio_int1);
			goto free_sysfs_stepcnt;
		}

		err = gpio_direction_input(sdata->pdata.gpio_int1);
		if (err) {
			dev_err(&client->dev,
				"Unable to set direction for gpio1 %d\n",
				sdata->pdata.gpio_int1);
			goto free_sysfs_stepcnt;
		}
		sdata->irq1 = gpio_to_irq(sdata->pdata.gpio_int1);
	}

	/* NOTE; by default all interrupts are routed to int1 */
	/* setup irq handler */
	if (sdata->irq1) {
		err = request_threaded_irq(sdata->irq1, NULL,
				kx126_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"kx126_irq", sdata);

		if (err) {
			dev_err(&client->dev, "unable to request irq1\n");
			goto free_gpio;
		}

		disable_irq(sdata->irq1);
	}

	/* poll timer */
	hrtimer_init(&sdata->accel_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sdata->accel_timer.function = kx126_data_timer_handler;

	/* data report thread */
	init_waitqueue_head(&sdata->accel_wq);
	sdata->accel_wkp_flag = 0;
	sdata->accel_task = kthread_run(kx126_data_strm_thread, sdata,
						"kionix_accel");

	/* data report using irq or timer*/
	if (sdata->pdata.use_drdy_int && sdata->irq1)
		sdata->use_poll_timer = false;
	else
		sdata->use_poll_timer = true;

	__debug("%s use_poll_timer %d \n",__FUNCTION__, sdata->use_poll_timer);

	/* step counter poll work */
	INIT_DELAYED_WORK(&sdata->step_work, kx126_step_work_func);

	/* default values for pedometer  */
	sdata->pdata.poll_rate_ms = 10;
	sdata->stepcnt_stepwm = 0xffff;

	/* set sensor initial reg values to sensor */
	kx126_map_delay_to_odr(sdata->pdata.poll_rate_ms,
		&sdata->accel_poll_rate, &sdata->odr_index);
	err = kx126_set_sensor_initial_config(sdata);

	if (err < 0) {
		dev_err(&client->dev, "set init values fail\n");
		goto free_sensor_init;
	}

#ifdef QCOM_SENSORS
	/* register accel to sensors class */
	err = kx126_sensors_cdev_register(sdata);
	if (err) {
		dev_err(&client->dev, "create sensors cdev file failed\n");
		err = -EINVAL;
		goto free_sensor_init;
	}

	/* register step detector to sensors class */
	err = kx126_stepdet_sensors_cdev_register(sdata);
	if (err) {
		dev_err(&client->dev, "create stepdet sensors cdev file failed\n");
		err = -EINVAL;
		goto free_accel_cdev;
	}

	/* register step counter to sensors class */
	err = kx126_stepcnt_sensors_cdev_register(sdata);
	if (err) {
		dev_err(&client->dev, "create stepcnt sensors cdev file failed\n");
		err = -EINVAL;
		goto free_stepdet_cdev;
	}
#endif

	mutex_unlock(&sdata->mutex);

	return 0;

#ifdef QCOM_SENSORS
free_stepdet_cdev:
	kx126_stepdet_sensors_cdev_unregister(sdata);
free_accel_cdev:
	kx126_sensors_cdev_unregister(sdata);
#endif
free_sensor_init:
	if (sdata->accel_task) {
		hrtimer_cancel(&sdata->accel_timer);
		kthread_stop(sdata->accel_task);
		sdata->accel_task = NULL;
	}
	if (sdata->irq1)
		free_irq(sdata->irq1, sdata);
free_gpio:
	if (gpio_is_valid(sdata->pdata.gpio_int1))
		gpio_free(sdata->pdata.gpio_int1);
free_sysfs_stepcnt:
	sysfs_remove_group(&sdata->stepcnt_input_dev->dev.kobj,
			&kx126_stepcnt_attribute_group);
free_input_stepcnt:
	kx126_stepcnt_input_dev_cleanup(sdata);
free_sysfs_stepdet:
	sysfs_remove_group(&sdata->stepdet_input_dev->dev.kobj,
			&kx126_stepcnt_attribute_group);
free_input_stepdet:
	kx126_stepdet_input_dev_cleanup(sdata);
free_sysfs_accel:
	sysfs_remove_group(&sdata->accel_input_dev->dev.kobj,
			&kx126_accel_attribute_group);
free_input_accel:
	kx126_strm_input_dev_cleanup(sdata);
free_src:
	if (sdata->pdata.release)
		sdata->pdata.release();
free_init:
	mutex_unlock(&sdata->mutex);

	return err;
}

static int kx126_remove(struct i2c_client *client)
{
	struct kx126_data *sdata = i2c_get_clientdata(client);

#ifdef QCOM_SENSORS
	kx126_stepcnt_sensors_cdev_unregister(sdata);
	kx126_stepdet_sensors_cdev_unregister(sdata);
	kx126_sensors_cdev_unregister(sdata);
#endif

	if (sdata->accel_task) {
		hrtimer_cancel(&sdata->accel_timer);
		kthread_stop(sdata->accel_task);
		sdata->accel_task = NULL;
	}

	if (sdata->irq1)
		free_irq(sdata->irq1, sdata);

	if (gpio_is_valid(sdata->pdata.gpio_int1)) {
		__debug("%s - free gpio_int1\n", __FUNCTION__);
		gpio_free(sdata->pdata.gpio_int1);
	}

	/* stepcnt input dev */
	sysfs_remove_group(&sdata->stepcnt_input_dev->dev.kobj,
		&kx126_stepcnt_attribute_group);

	kx126_stepcnt_input_dev_cleanup(sdata);

	/* stepdet input dev */
	sysfs_remove_group(&sdata->stepdet_input_dev->dev.kobj,
			&kx126_stepcnt_attribute_group);

	kx126_stepdet_input_dev_cleanup(sdata);

	/* accel input dev */
	sysfs_remove_group(&sdata->accel_input_dev->dev.kobj,
		&kx126_accel_attribute_group);

	kx126_strm_input_dev_cleanup(sdata);

	if (sdata->pdata.release)
		sdata->pdata.release();

	return 0;
}

#ifdef CONFIG_PM
static int kx126_suspend(struct device *dev)
{
	return 0;
}

static int kx126_resume(struct device *dev)
{
	return 0;
}
#else
#define kx126_suspend	NULL
#define kx126_resume	NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int kx126_runtime_suspend(struct device *dev)
{
	return 0;
}

static int kx126_runtime_resume(struct device *dev)
{
	return 0;
}
#else
#define kx126_runtime_suspend	NULL
#define kx126_runtime_resume	NULL
#endif

static const struct i2c_device_id kx126_id[] = {
	{ KX126_DEV_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kx126_id);

static const struct of_device_id kx126_of_match[] = {
	{ .compatible = "kionix,kx126", },
	{ },
};
MODULE_DEVICE_TABLE(of, kx126_of_match);

static const struct dev_pm_ops kx126_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(kx126_suspend, kx126_resume)
	SET_RUNTIME_PM_OPS(kx126_runtime_suspend, kx126_runtime_resume, NULL)
};

static struct i2c_driver kx126_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name  = KX126_DEV_NAME,
			.pm    = &kx126_pm_ops,
			.of_match_table = kx126_of_match,
		  },
	.probe    = kx126_probe,
	.remove   = kx126_remove,
	.id_table = kx126_id,
};

module_i2c_driver(kx126_driver);

MODULE_DESCRIPTION("kx126 driver");
MODULE_AUTHOR("Kionix");
MODULE_LICENSE("GPL");

