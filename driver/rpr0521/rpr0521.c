// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * rpr0521: Proximity sensor, Ambient light sensors and ir led -combo.
 *
 * Copyright(C) 2016 ROHM Semiconductor
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include "rpr0521.h"

// Choose __debug macro according to needs
#define __debug(...)
// #define __debug printk

#define RPR0521_DEFAULT_PS_DELAY 100
#define RPR0521_DEFAULT_ALS_DELAY 100
#define RPR0521_DEFAULT_PS_OFFSET 0

#define EVENT_TYPE EV_ABS
#define PROXIMITY_EVENT_TYPE ABS_DISTANCE
#define ALS_LUX_EVENT_TYPE ABS_MISC


// Main data store. Memory for it is allocated in _probe.
struct rpr0521_data {
	struct i2c_client *client;
	struct rpr0521_platform_data pdata;
	struct mutex mutex;
	struct regmap *regmap;

	// proximity
	struct input_dev *proximity_input_dev;
	struct hrtimer proximity_timer;
	int proximity_wkp_flag;
	struct task_struct *proximity_task;
	int proximity_delay_requested;
	int proximity_delay;
	wait_queue_head_t proximity_wq;
	bool proximity_enabled;
	int proximity_last_reported_value;

	// ALS
	struct input_dev *als_input_dev;
	struct hrtimer als_timer;
	int als_wkp_flag;
	struct task_struct *als_task;
	int als_delay_requested;
	int als_delay;
	wait_queue_head_t als_wq;
	bool als_enabled;
	int als_last_reported_value;

	int irq_handle;
	u8 part_id;
	bool use_poll_timer;
};

struct meas_time_map_t {
	int ps_time;
	u8 reg;
};

static struct meas_time_map_t meas_time_map_0[] = {
	{0, RPR0521_MODE_CONTROL_MEASUREMENT_TIME_OFF_OFF},
	{100, RPR0521_MODE_CONTROL_MEASUREMENT_TIME_OFF_100MS},
	{400, RPR0521_MODE_CONTROL_MEASUREMENT_TIME_OFF_400MS},
};

static struct meas_time_map_t meas_time_map_100[] = {
	{50, RPR0521_MODE_CONTROL_MEASUREMENT_TIME_100MS_50MS},
	{100, RPR0521_MODE_CONTROL_MEASUREMENT_TIME_100MS_100MS},
	{400, RPR0521_MODE_CONTROL_MEASUREMENT_TIME_100MS_400MS},

};

static struct meas_time_map_t meas_time_map_400[] = {
	{0, RPR0521_MODE_CONTROL_MEASUREMENT_TIME_400MS_OFF},
	{50, RPR0521_MODE_CONTROL_MEASUREMENT_TIME_400MS_50MS},
	{100, RPR0521_MODE_CONTROL_MEASUREMENT_TIME_400MS_100MS},
	{400, RPR0521_MODE_CONTROL_MEASUREMENT_TIME_400MS_400MS},	
};

// 	Distance table is 'calibrated' with post-it note, ruler and bare sensor.
//	Adjust according to HW design.
static int ps_distance_table[] = {
	0xffe, 0x640, // 0.5cm, 1.0cm
	0x2c0, 0x160,
	0x0fe, 0x9b,
	0x4b, 0x3b,
	0x30, 0x26, // 4.5cm, 5cm
	0x1f, 0x19,
	0x16, 0x11,
	0x0f, 0x0e,
	0x0d, 0x0c,
	0x0b, 0x0a, // 9.5, 10cm
	0x09, 0x08,
	0x03, 0x03,
	0x03, 0x02,
	0x02, 0x02,
	0x01, 0x01,
	0x01, 0x00 // 15.5cm, 16cm
};

// Regmap data
static const struct reg_default rpr0521_reg_default_values[] = {
	{0x40, 0x0a},
	{0x41, 0x00},
	{0x42, 0x02},
	{0x43, 0x01},
	{0x4a, 0x00},
	{0x4b, 0xff},
	{0x4c, 0x0F},
	{0x4d, 0x00},
	{0x4e, 0x00},
	{0x4f, 0xff},
	{0x50, 0xff},
	{0x51, 0x00},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x54, 0x00},
};

static bool rpr0521_is_reg_readable(struct device *dev, unsigned int reg)
{
	if (((0x40 <= reg) && (reg <= 0x54)) || (reg == 0x92))
		return true;
	else
		return false;
}

static bool rpr0521_is_reg_writeable(struct device *dev, unsigned int reg)
{
	if ((0x4A <= reg) && (reg <= 0x54))
		return true;
	else if ((0x40 <= reg) && (reg <= 0x43))
		return true;
	else
		return false;
}

static bool rpr0521_is_reg_volatile(struct device *dev, unsigned int reg)
{
	// Volatile register value can change without driver changing it.
	if ((0x43 <= reg) && (reg <= 0x4A))
		return true;
	else
		return false;
}

static bool rpr0521_is_reg_precious(struct device *dev, unsigned int reg)
{
	if (reg == 0x4A)
		return true;
	else
		return false;
}

static const struct regmap_config rpr0521_regmap_config = {
	.reg_bits = 8,
	.reg_stride = 1,
	.val_bits = 8,

	.volatile_reg = rpr0521_is_reg_volatile,  //Can't be cached.
	.readable_reg = rpr0521_is_reg_readable,
	.writeable_reg = rpr0521_is_reg_writeable,
	.precious_reg = rpr0521_is_reg_precious,  //Don't let others touch

	.max_register = 0x92,
	.reg_defaults = rpr0521_reg_default_values,
	.num_reg_defaults = ARRAY_SIZE(rpr0521_reg_default_values),

	.use_single_rw = 0,
	.cache_type = REGCACHE_RBTREE,
};

// Prototypes
inline static int rpr0521_set_clear_interrupt(struct rpr0521_data *sdata);
inline static int rpr0521_set_als_data0_gain(struct rpr0521_data *sdata,
						enum e_RPR0521_ALS_PS_CONTROL_ALS_DATA0_GAIN gainx);
inline static int rpr0521_set_als_data1_gain(struct rpr0521_data *sdata,
						enum e_RPR0521_ALS_PS_CONTROL_ALS_DATA1_GAIN gainx);
inline static int rpr0521_set_ps_gain(struct rpr0521_data *sdata,
						enum e_RPR0521_PS_CONTROL_PS_GAIN gainx);
inline static int rpr0521_set_measurement_time_reg(struct rpr0521_data *sdata,
						enum e_RPR0521_MODE_CONTROL_MEASUREMENT_TIME time);
static int rpr0521_set_measurement_time(struct rpr0521_data *sdata);
inline static int rpr0521_set_ps_int_sensitivity(struct rpr0521_data *sdata,
						enum e_RPR0521_PS_CONTROL_PERSISTENCE int_after_x);
inline static bool rpr0521_set_interrupt_threshold_ps_high(
						struct rpr0521_data *sdata, u16 threshold);
inline static bool rpr0521_set_interrupt_threshold_ps_low(
						struct rpr0521_data *sdata, u16 threshold);
inline static bool rpr0521_set_interrupt_threshold_als_high(
						struct rpr0521_data *sdata, u16 threshold);
inline static bool rpr0521_set_interrupt_threshold_als_low(
						struct rpr0521_data *sdata, u16 threshold);
inline static int rpr0521_set_interrupt_off(struct rpr0521_data *sdata);

inline static int
rpr0521_set_als_ps_measurement_enabled(struct rpr0521_data *sdata);
inline static int
rpr0521_set_als_measurement_enabled(struct rpr0521_data *sdata);
inline static int
rpr0521_set_ps_measurement_enabled(struct rpr0521_data *sdata);
inline static int
rpr0521_set_als_ps_measurement_disabled(struct rpr0521_data *sdata);
inline static int
rpr0521_set_als_measurement_disabled(struct rpr0521_data *sdata);
inline static int
rpr0521_set_ps_measurement_disabled(struct rpr0521_data *sdata);
inline static int rpr0521_soft_reset(struct rpr0521_data *sdata);
inline static bool
rpr0521_set_ps_offset(struct rpr0521_data *sdata, uint16_t offset);
inline static int
rpr0521_get_ps_offset(struct rpr0521_data *sdata, uint16_t *offset);
inline static int rpr0521_set_ps_operating_mode(struct rpr0521_data *sdata,
			enum e_RPR0521_MODE_CONTROL_PS_OPERATING_MODE ps_operating_mode);

static int rpr0521_proximity_enable_interrupt(struct rpr0521_data *sdata);
static int rpr0521_proximity_disable_interrupt(struct rpr0521_data *sdata);
static int rpr0521_als_enable_interrupt(struct rpr0521_data *sdata);
static int rpr0521_als_disable_interrupt(struct rpr0521_data *sdata);
int rpr0521_set_sensor_als_delay(struct rpr0521_data *sdata,
					unsigned int delay_max_ms);
int rpr0521_set_sensor_proximity_delay(struct rpr0521_data *sdata,
					unsigned int delay_max_ms);

// RPR0521 Register manipulation
static int rpr0521_set_default_values(struct rpr0521_data *sdata)
{
	int error;

	__debug("%s: Setting registers default values\n", __FUNCTION__);
	error = false || rpr0521_set_als_ps_measurement_disabled(sdata);
	error = error || rpr0521_set_als_data0_gain(sdata,
							RPR0521_ALS_PS_CONTROL_ALS_DATA0_GAIN_X1);
	error = error || rpr0521_set_als_data1_gain(sdata,
							RPR0521_ALS_PS_CONTROL_ALS_DATA1_GAIN_X1);
	error = error || rpr0521_set_ps_gain(sdata,
							RPR0521_PS_CONTROL_PS_GAIN_X1);

	if(!sdata->use_poll_timer) {
		// Use double measurement operating mode to get faster response on
		// proximity changes
		error = error || rpr0521_set_ps_operating_mode(
			sdata, RPR0521_MODE_CONTROL_PS_OPERATING_MODE_DOUBLE_MEASUREMENT);
		error = error || rpr0521_set_ps_int_sensitivity(
			sdata, RPR0521_PS_CONTROL_PERSISTENCE_CONSECUTIVE_2);
	} else {
		error = error || rpr0521_set_ps_int_sensitivity(sdata,
									RPR0521_PS_CONTROL_PERSISTENCE_DRDY);
	}

	return error;
}

inline static int rpr0521_set_als_data0_gain(struct rpr0521_data *sdata,
						enum e_RPR0521_ALS_PS_CONTROL_ALS_DATA0_GAIN gainx)
{
	if (sdata->als_enabled)
		__debug("%s: Measurement need to stop and restart after gain change\n", __FUNCTION__);

	return regmap_update_bits(sdata->regmap, RPR0521_ALS_PS_CONTROL,
					RPR0521_ALS_PS_CONTROL_ALS_DATA0_GAIN_MASK, gainx);
}

inline static int rpr0521_set_als_data1_gain(struct rpr0521_data *sdata,
						enum e_RPR0521_ALS_PS_CONTROL_ALS_DATA1_GAIN gainx)
{
	if (sdata->als_enabled)
		__debug("%s: Measurement need to stop and restart after gain change\n", __FUNCTION__);

	return regmap_update_bits(sdata->regmap, RPR0521_ALS_PS_CONTROL,
					RPR0521_ALS_PS_CONTROL_ALS_DATA1_GAIN_MASK, gainx);
}

inline static int rpr0521_set_ps_gain(struct rpr0521_data *sdata,
						enum e_RPR0521_PS_CONTROL_PS_GAIN gainx)
{
	return regmap_update_bits(sdata->regmap, RPR0521_PS_CONTROL,
					RPR0521_PS_CONTROL_PS_GAIN_MASK, gainx);
}

inline static int rpr0521_set_measurement_time_reg(struct rpr0521_data *sdata,
							enum e_RPR0521_MODE_CONTROL_MEASUREMENT_TIME time)
{
	return regmap_update_bits(sdata->regmap, RPR0521_MODE_CONTROL,
					RPR0521_MODE_CONTROL_MEASUREMENT_TIME_MASK, time);
}

inline static int rpr0521_get_meas_time_reg(int ps_time, u8 *time_reg,
								struct meas_time_map_t map[], int size)
{
	int i;

	for(i = 0; i < size; i++) {
		if(map[i].ps_time == ps_time) {
			*time_reg = map[i].reg;
			return 0;
		}
	}

	return -1;
}

static int rpr0521_set_measurement_time(struct rpr0521_data *sdata)
{
	u8 time_reg = 0;
	int err = 0;
	bool skip_change = false;

	if(sdata->proximity_delay_requested == 10) {
		time_reg = RPR0521_MODE_CONTROL_MEASUREMENT_TIME_OFF_10MS;
	} else if(sdata->proximity_delay_requested == 40) {
		time_reg = RPR0521_MODE_CONTROL_MEASUREMENT_TIME_OFF_40MS;
	} else if(sdata->proximity_delay_requested == 50 && 
				sdata->als_delay_requested == 0) {
		time_reg = RPR0521_MODE_CONTROL_MEASUREMENT_TIME_OFF_100MS;
	} else if(sdata->als_delay_requested == 50) {
		time_reg = RPR0521_MODE_CONTROL_MEASUREMENT_TIME_50MS_50MS;
	} else if(sdata->als_delay_requested == 0) {
		err = rpr0521_get_meas_time_reg(sdata->proximity_delay_requested,
				&time_reg, meas_time_map_0, ARRAY_SIZE(meas_time_map_0));
	} else if(sdata->als_delay_requested == 100) {
		err = rpr0521_get_meas_time_reg(sdata->proximity_delay_requested,
				&time_reg, meas_time_map_100, ARRAY_SIZE(meas_time_map_100));
	} else if(sdata->als_delay_requested == 400) {
		err = rpr0521_get_meas_time_reg(sdata->proximity_delay_requested,
				&time_reg, meas_time_map_400, ARRAY_SIZE(meas_time_map_400));
	} else {
		skip_change = true;
	}

	if(err < 0 || skip_change) {
		__debug("%s: Time combination not possible\n", __FUNCTION__);
		return -1;
	} else {
		__debug("%s: time reg: 0x%x\n", __FUNCTION__, time_reg);
		return rpr0521_set_measurement_time_reg(sdata, time_reg);
	}
}

inline static int rpr0521_set_ps_int_sensitivity(struct rpr0521_data *sdata,
						enum e_RPR0521_PS_CONTROL_PERSISTENCE int_after_x)
{
	return regmap_update_bits(sdata->regmap, RPR0521_PS_CONTROL,
						RPR0521_PS_CONTROL_PERSISTENCE_MASK, int_after_x);
}

inline bool rpr0521_is_ambient_ir_low(struct rpr0521_data *sdata)
{
	unsigned int ps_control_reg;
	unsigned int ambient_ir_flag;
	int err;

	err = regmap_read(sdata->regmap, RPR0521_PS_CONTROL, &ps_control_reg);
	if (err)
		return false;

	ambient_ir_flag = ps_control_reg && ~RPR0521_PS_CONTROL_AMBIENT_IR_FLAG_MASK;
	if (ambient_ir_flag != RPR0521_PS_CONTROL_AMBIENT_IR_FLAG_LOW)
		return false;

	return true;
}

inline static bool rpr0521_set_interrupt_threshold_ps_high(
						struct rpr0521_data *sdata, u16 threshold)
{
	int err;
	u8 threshold_raw[2];
	//__debug("%s: threshold_new: 0x%04x\n", __FUNCTION__, threshold);

	threshold_raw[0] = threshold & 0xff;
	threshold_raw[1] = (threshold >> 8) & 0x0f;

	err = false || regmap_write(sdata->regmap, RPR0521_PS_TH_LSBS, threshold_raw[0]);
	err = err || regmap_write(sdata->regmap, RPR0521_PS_TH_MSBS, threshold_raw[1]);
	return err;
}

inline static bool rpr0521_set_interrupt_threshold_ps_low(
						struct rpr0521_data *sdata, u16 threshold)
{
	int err;
	u8 threshold_raw[2];
	//__debug("%s: threshold_new: 0x%04x\n", __FUNCTION__, threshold);

	threshold_raw[0] = threshold & 0xff;
	threshold_raw[1] = (threshold >> 8) & 0x0f;

	err = false || regmap_write(sdata->regmap, RPR0521_PS_TL_LSBS, threshold_raw[0]);
	err = err || regmap_write(sdata->regmap, RPR0521_PS_TL_MSBS, threshold_raw[1]);

	return err;
}

inline static bool rpr0521_set_interrupt_threshold_als_high(
						struct rpr0521_data *sdata, u16 threshold)
{
	int err;
	u8 threshold_raw[2];
	//__debug("%s: threshold_new: 0x%04x\n", __FUNCTION__, threshold);

	threshold_raw[0] = threshold & 0xff;
	threshold_raw[1] = (threshold >> 8) & 0x0f;

	err = false || regmap_write(sdata->regmap, RPR0521_ALS_DATA0_TH_LSBS, threshold_raw[0]);
	err = err || regmap_write(sdata->regmap, RPR0521_ALS_DATA0_TH_MSBS, threshold_raw[1]);

	return err;
}

inline static bool rpr0521_set_interrupt_threshold_als_low(
						struct rpr0521_data *sdata, u16 threshold)
{
	int err;
	u8 threshold_raw[2];
	//__debug("%s: threshold_new: 0x%04x\n", __FUNCTION__, threshold);

	threshold_raw[0] = threshold & 0xff;
	threshold_raw[1] = (threshold >> 8) & 0x0f;

	err = false || regmap_write(sdata->regmap, RPR0521_ALS_DATA0_TL_LSBS, threshold_raw[0]);
	err = err || regmap_write(sdata->regmap, RPR0521_ALS_DATA0_TL_MSBS, threshold_raw[1]);

	return err;
}

inline static int rpr0521_set_interrupt_off(struct rpr0521_data *sdata)
{
	u8 interrupt_reg_new;
	interrupt_reg_new = (RPR0521_INTERRUPT_INT_MODE_PS_TH_OUTSIDE_DETECTION |
						 RPR0521_INTERRUPT_INT_ASSERT_STABLE |
						 RPR0521_INTERRUPT_INT_LATCH_ENABLED |
						 RPR0521_INTERRUPT_INT_TRIG_INACTIVE);
	// __debug("%s: interrupt_reg_new: 0x%02x\n", __FUNCTION__,
	// 		interrupt_reg_new);

	return regmap_write(sdata->regmap, RPR0521_INTERRUPT, interrupt_reg_new);
}

inline static int rpr0521_set_clear_interrupt(struct rpr0521_data *sdata)
{
	u8 system_control_reg_new;
	system_control_reg_new = (RPR0521_SYSTEM_CONTROL_SW_RESET_NOT_STARTED |
							  RPR0521_SYSTEM_CONTROL_INT_PIN_HI_Z);

	return regmap_write(sdata->regmap, RPR0521_SYSTEM_CONTROL, system_control_reg_new);
}

inline static int
rpr0521_set_als_ps_measurement_enabled(struct rpr0521_data *sdata)
{
	return regmap_update_bits(sdata->regmap, RPR0521_MODE_CONTROL,
							  (RPR0521_MODE_CONTROL_ALS_EN_MASK |
							  RPR0521_MODE_CONTROL_PS_EN_MASK),
							  (RPR0521_MODE_CONTROL_ALS_EN_TRUE |
							  RPR0521_MODE_CONTROL_PS_EN_TRUE));
}

inline static int
rpr0521_set_als_measurement_enabled(struct rpr0521_data *sdata)
{
	return regmap_update_bits(sdata->regmap, RPR0521_MODE_CONTROL,
							  (RPR0521_MODE_CONTROL_ALS_EN_MASK),
							  (RPR0521_MODE_CONTROL_ALS_EN_TRUE));
}

inline static int
rpr0521_set_ps_measurement_enabled(struct rpr0521_data *sdata)
{
	return regmap_update_bits(sdata->regmap, RPR0521_MODE_CONTROL,
							  (RPR0521_MODE_CONTROL_PS_EN_MASK),
							  (RPR0521_MODE_CONTROL_PS_EN_TRUE));
}

inline static int
rpr0521_set_als_ps_measurement_disabled(struct rpr0521_data *sdata)
{
	return regmap_update_bits(sdata->regmap, RPR0521_MODE_CONTROL,
							  (RPR0521_MODE_CONTROL_ALS_EN_MASK |
							  RPR0521_MODE_CONTROL_PS_EN_MASK),
							  (RPR0521_MODE_CONTROL_ALS_EN_FALSE |
							  RPR0521_MODE_CONTROL_PS_EN_FALSE));
}

inline static int
rpr0521_set_als_measurement_disabled(struct rpr0521_data *sdata)
{
	return regmap_update_bits(sdata->regmap, RPR0521_MODE_CONTROL,
							  (RPR0521_MODE_CONTROL_ALS_EN_MASK),
							  (RPR0521_MODE_CONTROL_ALS_EN_FALSE));
}

inline static int
rpr0521_set_ps_measurement_disabled(struct rpr0521_data *sdata)
{
	return regmap_update_bits(sdata->regmap, RPR0521_MODE_CONTROL,
							  (RPR0521_MODE_CONTROL_PS_EN_MASK),
							  (RPR0521_MODE_CONTROL_PS_EN_FALSE));
}

inline static int rpr0521_set_ps_operating_mode(struct rpr0521_data *sdata,
			enum e_RPR0521_MODE_CONTROL_PS_OPERATING_MODE ps_operating_mode)
{
	return regmap_update_bits(sdata->regmap, RPR0521_MODE_CONTROL,
							  RPR0521_MODE_CONTROL_PS_OPERATING_MODE_MASK,
							  ps_operating_mode);
}

inline static int rpr0521_soft_reset(struct rpr0521_data *sdata)
{
	return regmap_write(sdata->regmap, RPR0521_SYSTEM_CONTROL,
						(RPR0521_SYSTEM_CONTROL_SW_RESET_START |
						RPR0521_SYSTEM_CONTROL_INT_PIN_HI_Z));
}

static int rpr0521_proximity_enable(struct rpr0521_data *sdata)
{
	int err;

	if (sdata->proximity_enabled) {
		__debug("%s: Already enabled, skipping\n", __FUNCTION__);
		return 0;
	}

	err = rpr0521_set_ps_measurement_enabled(sdata);
	if (err) return err;

	if (sdata->use_poll_timer) {
		__debug("%s: Start timer\n", __FUNCTION__);
		hrtimer_start(&sdata->proximity_timer,
					  ktime_set(sdata->proximity_delay / 1000,
					  (sdata->proximity_delay % 1000) * 1000000),
					  HRTIMER_MODE_REL);
	} else {
		__debug("%s: Enabling interrupt\n", __FUNCTION__);
		err = rpr0521_proximity_enable_interrupt(sdata);
		if (err < 0) return err;
	}
	sdata->proximity_enabled = true;

	return 0;
}

static int rpr0521_proximity_disable(struct rpr0521_data *sdata)
{
	int err;

	if (!sdata->proximity_enabled) {
		__debug("%s: Already disabled, skipping\n", __FUNCTION__);
		return 0;
	}

	err = rpr0521_set_ps_measurement_disabled(sdata);
	if (err) return err;

	if (!sdata->use_poll_timer) {
		__debug("%s: Disabling interrupt\n", __FUNCTION__);
		err = rpr0521_proximity_disable_interrupt(sdata);
		if (err) return err;
	}
	sdata->proximity_enabled = false;

	return 0;
}

static int rpr0521_als_enable(struct rpr0521_data *sdata)
{
	int err;

	if (sdata->als_enabled) {
		__debug("%s: Already enabled, skipping\n", __FUNCTION__);
		return 0;
	}

	err = rpr0521_set_als_measurement_enabled(sdata);
	if (err) return err;

	if (sdata->use_poll_timer) {
		__debug("%s: Start timer\n", __FUNCTION__);
		hrtimer_start(&sdata->als_timer,
					  ktime_set(sdata->als_delay / 1000,
					  (sdata->als_delay % 1000) * 1000000),
					  HRTIMER_MODE_REL);
	} else {
		__debug("%s: Enabling interrupt\n", __FUNCTION__);
		err = rpr0521_als_enable_interrupt(sdata);
		if (err < 0) return err;
	}
	sdata->als_enabled = true;

	return 0;
}

static int rpr0521_als_disable(struct rpr0521_data *sdata)
{
	int err;

	if (!sdata->als_enabled) {
		__debug("%s: Already disabled, skipping\n", __FUNCTION__);
		return 0;
	}

	err = rpr0521_set_als_measurement_disabled(sdata);
	if (err) return err;

	if (!sdata->use_poll_timer) {
		__debug("%s: Disabling interrupt\n", __FUNCTION__);
		err = rpr0521_als_disable_interrupt(sdata);
		if (err < 0) return err;
	}
	sdata->als_enabled = false;

	return 0;
}

int rpr0521_set_sensor_als_delay(struct rpr0521_data *sdata,
								unsigned int delay_max_ms)
{
	static int als_valid_delays[] = {0, 50, 100, 400};
	int err;
	int i;

	for (i = 0; i < ARRAY_SIZE(als_valid_delays); i++) {
		if (delay_max_ms == als_valid_delays[i])
			break;
	}

	if(i == ARRAY_SIZE(als_valid_delays)) {
		__debug("%s: ALS time not possible\n", __FUNCTION__);
		return -1;
	}

	sdata->als_delay_requested = delay_max_ms;
	__debug("%s: Setting ALS %dms\n", __FUNCTION__, sdata->als_delay_requested);

	err = rpr0521_set_measurement_time(sdata);
	if(err) {
		sdata->als_delay_requested = sdata->als_delay;
		return err;
	}
	sdata->als_delay = sdata->als_delay_requested;
	if(sdata->use_poll_timer && sdata->als_enabled) {
		hrtimer_start(&sdata->als_timer,
			ktime_set(sdata->als_delay / 1000,
			(sdata->als_delay % 1000) * 1000000),
			HRTIMER_MODE_REL);
		__debug("%s: als_poll_rate %d, proximity_poll_rate %d\n",
			__FUNCTION__, sdata->als_delay, sdata->proximity_delay);
	}

	return 0;
}

int rpr0521_set_sensor_proximity_delay(struct rpr0521_data *sdata,
								unsigned int delay_max_ms)
{
	static int ps_valid_delays[] = {0, 10, 40, 50, 100, 400};
	int err;
	int i;

	for (i = 0; i < ARRAY_SIZE(ps_valid_delays); i++) {
		if (delay_max_ms == ps_valid_delays[i])
			break;
	}

	if(i == ARRAY_SIZE(ps_valid_delays)) {
		__debug("%s: proximity time not possible\n", __FUNCTION__);
		return -1;
	}

	sdata->proximity_delay_requested = delay_max_ms;
	__debug("%s: Setting Proximity %dms\n", __FUNCTION__,
				sdata->proximity_delay_requested);
	
	err = rpr0521_set_measurement_time(sdata);
	if(err) {
		sdata->proximity_delay_requested = sdata->proximity_delay;
		return err;
	}
	sdata->proximity_delay = sdata->proximity_delay_requested;
	if(sdata->use_poll_timer && sdata->proximity_enabled) {
		hrtimer_start(&sdata->proximity_timer,
			ktime_set(sdata->proximity_delay / 1000,
			(sdata->proximity_delay % 1000) * 1000000),
			HRTIMER_MODE_REL);
		__debug("%s: als_poll_rate %d, proximity_poll_rate %d\n",
			__FUNCTION__, sdata->als_delay, sdata->proximity_delay);
	}

	return 0;
}

inline static bool
rpr0521_set_ps_offset(struct rpr0521_data *sdata, uint16_t offset)
{
	int err;
	uint8_t lsb, msb;

	lsb = offset & 0xff;
	msb = (offset >> 8) & 0x03;  //2bit msb
	err = false || regmap_write(sdata->regmap, RPR0521_PS_OFFSET_LSBS, lsb);
	err = err || regmap_write(sdata->regmap, RPR0521_PS_OFFSET_MSBS, msb);

	if(err)
		return err;
	sdata->pdata.proximity_offset = offset;

	return 0;
}

inline static int
rpr0521_get_ps_offset(struct rpr0521_data *sdata, uint16_t *offset)
{
#define OFFSET_LEN 2
	uint8_t temp[OFFSET_LEN];
	int err;

	err = regmap_raw_read(sdata->regmap, RPR0521_PS_OFFSET_LSBS, temp, OFFSET_LEN);
	*offset = temp[0] + (temp[1] << 8);

	return err;
}

static bool
rpr0521_set_new_ps_thresholds(struct rpr0521_data *sdata, int si_result_i)
{
	bool err;
	u16 threshold_high, threshold_low;

	threshold_high = ps_distance_table[si_result_i];
	if (si_result_i == (ARRAY_SIZE(ps_distance_table) - 1))
		threshold_low = 0x0000;  //Min value
	else
		threshold_low = ps_distance_table[si_result_i + 1];

	err = false || rpr0521_set_interrupt_threshold_ps_high(sdata, threshold_high);
	err = err || rpr0521_set_interrupt_threshold_ps_low(sdata, threshold_low);
	//__debug("%s: SI[%d]*0.5cm, threshold low[%x], high[%x]\n", __FUNCTION__,
	// 			si_result_i, threshold_low, threshold_high);

	return err;
}

// Interrupt related
static int rpr0521_enable_interrupt_pin(struct rpr0521_data *sdata)
{
	__debug("%s: Enable irq %d\n", __FUNCTION__, sdata->irq_handle);
	enable_irq(sdata->irq_handle);

	return 0;
}

static int rpr0521_disable_interrupt_pin(struct rpr0521_data *sdata)
{
	__debug("%s: Disable irq %d\n", __FUNCTION__, sdata->irq_handle);
	disable_irq(sdata->irq_handle);
	return 0;
}

static int rpr0521_als_enable_interrupt(struct rpr0521_data *sdata)
{
	int err;

	err = regmap_update_bits(sdata->regmap, RPR0521_INTERRUPT,
							 RPR0521_INTERRUPT_INT_TRIG_BY_ALS_MASK |
							 RPR0521_INTERRUPT_INT_ASSERT_MASK |
							 RPR0521_INTERRUPT_INT_LATCH_MASK,
							 RPR0521_INTERRUPT_INT_TRIG_BY_ALS_ON |
							 RPR0521_INTERRUPT_INT_ASSERT_STABLE |
							 RPR0521_INTERRUPT_INT_LATCH_ENABLED);
	if (err) return err;

	//Cause ALS DRDY by crossed thresholds
	err = rpr0521_set_interrupt_threshold_als_low(sdata, 101);
	err = err || rpr0521_set_interrupt_threshold_als_high(sdata, 100);
	if (err) return err;

	if (!sdata->proximity_enabled) {
		//Trust that pin is not enabled if PS is not enabled.
		err = rpr0521_enable_interrupt_pin(sdata);
		if (err < 0) return err;
	}
	return 0;
}

static int rpr0521_als_disable_interrupt(struct rpr0521_data *sdata)
{
	int err;

	err = regmap_update_bits(sdata->regmap, RPR0521_INTERRUPT,
							 RPR0521_INTERRUPT_INT_TRIG_BY_ALS_MASK,
							 RPR0521_INTERRUPT_INT_TRIG_BY_ALS_OFF);
	if (err) return err;

	if (!sdata->proximity_enabled) {
		// Int pin power consumption might be 25uA if left on so clear it
		(void)rpr0521_set_clear_interrupt(sdata);  //ignore fails
		return rpr0521_disable_interrupt_pin(sdata);
	}

	return 0;
}

static int rpr0521_proximity_enable_interrupt(struct rpr0521_data *sdata)
{
	int err;

	err = regmap_update_bits(sdata->regmap, RPR0521_INTERRUPT,
							 RPR0521_INTERRUPT_INT_TRIG_BY_PS_MASK |
							 RPR0521_INTERRUPT_INT_MODE_MASK |
							 RPR0521_INTERRUPT_INT_ASSERT_MASK |
							 RPR0521_INTERRUPT_INT_LATCH_MASK,
							 RPR0521_INTERRUPT_INT_TRIG_BY_PS_ON |
							 RPR0521_INTERRUPT_INT_MODE_PS_TH_OUTSIDE_DETECTION
							 | RPR0521_INTERRUPT_INT_ASSERT_STABLE |
							 RPR0521_INTERRUPT_INT_LATCH_ENABLED);

	//Always interrupt after first measurement
	err = err || rpr0521_set_interrupt_threshold_ps_high(sdata, 0x0000);
	err = err || rpr0521_set_interrupt_threshold_ps_low(sdata, 0x0eee);
	if (err) return err;

	if (!sdata->als_enabled) {
		err = rpr0521_enable_interrupt_pin(sdata);
		if (err < 0) return err;
	}

	return 0;
}

static int rpr0521_proximity_disable_interrupt(struct rpr0521_data *sdata)
{
	int err;

	err = regmap_update_bits(sdata->regmap, RPR0521_INTERRUPT,
							 RPR0521_INTERRUPT_INT_TRIG_BY_PS_MASK,
							 RPR0521_INTERRUPT_INT_TRIG_BY_PS_OFF);
	if (err) return err;

	if (!sdata->als_enabled) {
		//Int pin power consumption might be 25uA if left on so clear it
		(void)rpr0521_set_clear_interrupt(sdata);  //ignore fails
		return rpr0521_disable_interrupt_pin(sdata);
	}

	return 0;
}

static irqreturn_t rpr0521_irq_handler(int irq, void *sdata_as_cookie)
{
	unsigned int status;
	int ps_int_status;
	int als_int_status;
	int err;
	struct rpr0521_data *sdata = sdata_as_cookie;

	err = regmap_read(sdata->regmap, RPR0521_INTERRUPT, &status);
	if (err) {
		dev_err(&sdata->client->dev, "irq status read error\n");
		return IRQ_HANDLED;
	}
	ps_int_status = status & RPR0521_INTERRUPT_PS_INT_STATUS_MASK;
	als_int_status = status & RPR0521_INTERRUPT_ALS_INT_STATUS_MASK;

	if ((ps_int_status == 0) && (als_int_status == 0)) {
		if ((!sdata->als_enabled) && (!sdata->proximity_enabled)) {
			dev_err(&sdata->client->dev,
					"%s: rpr0521 measurements already disabled, no need for irq handling!\n",
					__FUNCTION__);
			//Interrupt was not relevant anymore if it was from this device
			return IRQ_HANDLED;
		} else {
			dev_err(&sdata->client->dev, "%s: rpr0521_int_status are ZERO!\n",
					__FUNCTION__);
			//interrupt was not from this device
			return IRQ_NONE;
		}
	}

	if (ps_int_status) {
		sdata->proximity_wkp_flag = 1;
		wake_up_interruptible(&sdata->proximity_wq);
	}
	if (als_int_status) {
		sdata->als_wkp_flag = 1;
		wake_up_interruptible(&sdata->als_wq);
	}
	err = rpr0521_set_clear_interrupt(sdata);
	if (err) {
		//Another try, failing to clear might cause data stop.
		err = rpr0521_set_clear_interrupt(sdata);
		dev_err(&sdata->client->dev,
				"%s: interrupt clear via i2c failed! Retry result:[%d]\n",
				__FUNCTION__, err);
	}

	return IRQ_HANDLED;
}

// Timer related functions
static enum
hrtimer_restart rpr0521_proximity_timer_handler(struct hrtimer *handle)
{
	struct rpr0521_data *sdata = container_of(
								handle, struct rpr0521_data, proximity_timer);

	if (sdata->proximity_enabled) {
		hrtimer_forward_now(&sdata->proximity_timer,
							ktime_set(sdata->proximity_delay / 1000,
							(sdata->proximity_delay % 1000) * 1000000));
		sdata->proximity_wkp_flag = 1;
		wake_up_interruptible(&sdata->proximity_wq);
		return HRTIMER_RESTART;
	} else {
		__debug("%s: No restart\n", __FUNCTION__);
		return HRTIMER_NORESTART;
	}
}

static enum hrtimer_restart rpr0521_als_timer_handler(struct hrtimer *handle)
{
	struct rpr0521_data *sdata = container_of(
								handle, struct rpr0521_data, als_timer);

	if (sdata->als_enabled) {
		hrtimer_forward_now(&sdata->als_timer,
							ktime_set(sdata->als_delay / 1000,
							(sdata->als_delay % 1000) * 1000000));
		sdata->als_wkp_flag = 1;
		wake_up_interruptible(&sdata->als_wq);
		return HRTIMER_RESTART;
	} else {
		__debug("%s: No restart\n", __FUNCTION__);
		return HRTIMER_NORESTART;
	}
}

// Event generation
static void rpr0521_report_ps_si(struct rpr0521_data *sdata, int *si_result)
{
	if (si_result[0] == sdata->proximity_last_reported_value)
		return; //skip unchanged value event since ON_CHANGE_ONLY

	__debug("%s: Prox: 0x%03x\n", __FUNCTION__, si_result[0]);

	input_report_abs(sdata->proximity_input_dev, PROXIMITY_EVENT_TYPE,
					si_result[0]);
	input_sync(sdata->proximity_input_dev);
	sdata->proximity_last_reported_value = si_result[0];
}

static void rpr0521_report_als_si(struct rpr0521_data *sdata, int *si_result)
{
	if (si_result[0] == sdata->als_last_reported_value)
		return; //skip unchanged value event

	__debug("%s: Lux: 0x%03x\n", __FUNCTION__, si_result[0]);

	input_report_abs(sdata->als_input_dev, ALS_LUX_EVENT_TYPE, si_result[0]);
	input_sync(sdata->als_input_dev);
	sdata->als_last_reported_value = si_result[0];
}

static int rpr0521_read_result_ps(struct rpr0521_data *sdata, int *si_result)
{
#define READLEN_PS 2
	int i;
	int raw;
	u8 reg_results[READLEN_PS];
	int err;

	err = regmap_raw_read(sdata->regmap, RPR0521_PS_DATA_LSBS,
							reg_results, READLEN_PS);
	if (err)
		return -EIO;
	//__debug("%s: Raw:  0x%x, 0x%x\n", __FUNCTION__, reg_results[0], reg_results[1]);

	raw = ((int)((reg_results[1] << 8) | reg_results[0]));  //Proximity 12bit
	for (i = 0; i < (ARRAY_SIZE(ps_distance_table) - 1); i++) {
		if (raw > ps_distance_table[i]) break;
	}
	if (i != 0)
		si_result[0] = i - 1;  //i * resolution = distance cm
	else
		si_result[0] = 0;

	// __debug("%s: 12bit Prox: 0x%03x\n", __FUNCTION__, raw);

	return 0;
}

static void rpr0521_calculate_als_lux_si(struct rpr0521_data *sdata,
									uint16_t d0, uint16_t d1, int *si_result)
{
#define CUTPOINTS 4
#define JTMUL 1000
	// Values from one actual product
	//	const uint16_t d0ce[CUTPOINTS] = {2156, 1601, 1140, 694};
	//	const uint16_t d1ce[CUTPOINTS] = {0, 891,  476,  247};
	//	const uint16_t jt[CUTPOINTS] = {177,  1107,  1955, 2810};
	// Application example -values
	const uint16_t d0ce[CUTPOINTS] = {1682, 644, 756, 766};
	const uint16_t d1ce[CUTPOINTS] = {1877, 132, 243, 250};
	const uint16_t jt[CUTPOINTS] = {595, 1015, 1352, 3053};
	int d;
	int i;
	int millilux;

	if (d0 == 0) {
		si_result[0] = 0;  //0 lux
		return;
	}
	//Assume ALS0-gain == 1 and ALS1-gain == 1
	d = (d1 * JTMUL) / d0;
	millilux = 0;  //default for d >= jt[CUTPOINTS]
	for (i = 0; i < CUTPOINTS; i++) {
		if (d < jt[i]) {
			millilux = ((d0ce[i] * d0) - (d1ce[i] * d1));
			break;
		}
	}
	si_result[0] = millilux / JTMUL;  //result in lux
	return;
}

static inline void
rpr0521_scaledown_als_si(struct rpr0521_data *sdata,uint16_t *d0, uint16_t *d1)
{
//Refer to specification page 18. 50ms/50ms mode needs check+scaledown.
#define MSB_MASK 0x8000

	if (*d0 & MSB_MASK) *d0 = 0xffff;
	if (*d1 & MSB_MASK) *d1 = 0xffff;
	*d0 = *d0 >> 1;
	*d1 = *d1 >> 1;
	return;
}

static int rpr0521_read_result_als(struct rpr0521_data *sdata, int *si_result)
{
#define READLEN_ALS 4
	int err;
	uint16_t visible_data0;
	uint16_t ir_data1;
	u8 reg_results[READLEN_ALS];

	err = regmap_raw_read(sdata->regmap, RPR0521_ALS_DATA0_LSBS,
							reg_results, READLEN_ALS);
	if (err)
		return -EIO;
	// __debug("%s: ALS Raw:  0x%x,  0x%x, 0x%x, 0x%x\n", __FUNCTION__,
	// 		reg_results[0], reg_results[1], reg_results[2], reg_results[3]);

	visible_data0 = ((int)((reg_results[1] << 8) | reg_results[0]));  //Als0 12bit
	ir_data1 = ((int)((reg_results[3] << 8) | reg_results[2]));       //Als1 12bit

	if (sdata->als_delay == 50 && sdata->proximity_delay == 50)
		rpr0521_scaledown_als_si(sdata, &visible_data0, &ir_data1);
	rpr0521_calculate_als_lux_si(sdata, visible_data0, ir_data1, &si_result[0]);

	// __debug("%s: 12bit Raw Als0-visible: [0x%03x], Als1-IR: [0x%03x], Lux: [0x%03x]\n",
			// __FUNCTION__, visible_data0, ir_data1, si_result[0]);

	return 0;
}

static int rpr0521_proximity_read_thread(void *data)
{
	struct rpr0521_data *sdata = data;

	while (1) {
		int si_i[1];
		int err;

		wait_event_interruptible(sdata->proximity_wq,
								 ((sdata->proximity_wkp_flag != 0) ||
								 kthread_should_stop()));
		sdata->proximity_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		err = rpr0521_read_result_ps(sdata, si_i);
		if (err) {
			dev_err(&sdata->client->dev, "i2c read/write error\n");
		} else {
			if(!sdata->use_poll_timer) {
				//Don't check IR levels if using threshold interrupt mode
				rpr0521_report_ps_si(sdata, si_i);
				//Set new interrupt values according to measurement result.
				err = rpr0521_set_new_ps_thresholds(sdata, si_i[0]);
				if (err)
					dev_err(&sdata->client->dev, "error changing thresholds\n");
			} else {
				if (rpr0521_is_ambient_ir_low(sdata))
					rpr0521_report_ps_si(sdata, si_i);
				else
					__debug("%s: Didn't report, too much ambient light\n",
							__FUNCTION__);
			}
		}
	}

	return 0;
}

static int rpr0521_als_read_thread(void *data)
{
	struct rpr0521_data *sdata = data;

	while (1) {
		int err;
		int si_results[2];

		wait_event_interruptible(sdata->als_wq,
								 ((sdata->als_wkp_flag != 0) ||
								 kthread_should_stop()));
		sdata->als_wkp_flag = 0;

		if (kthread_should_stop())
			break;

		err = rpr0521_read_result_als(sdata, si_results);
		if (err) {
			dev_err(&sdata->client->dev, "i2c read/write error\n");
		} else {
			rpr0521_report_als_si(sdata, si_results);
		}
	}

	return 0;
}

// Input sysfs related
static ssize_t rpr0521_sysfs_proximity_get_enable(
				struct device *dev, struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", sdata->proximity_enabled);
}

static ssize_t rpr0521_sysfs_proximity_set_enable( struct device *dev,
					struct device_attribute *attr, const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct rpr0521_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	if (value)
		err = rpr0521_proximity_enable(sdata);
	else
		err = rpr0521_proximity_disable(sdata);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_proximity_enable = __ATTR(
	enable,
	0644,
	rpr0521_sysfs_proximity_get_enable,
	rpr0521_sysfs_proximity_set_enable);

static ssize_t rpr0521_sysfs_reg_read(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *sdata = dev_get_drvdata(dev);
	u8 reg;
	unsigned int reg_val;
	int err;
	int i;
	int len = 0;

	len += sprintf(buf + len, "0x%2X ", RPR0521_REGISTER_DUMP_START);
	for (i = RPR0521_REGISTER_DUMP_START; i <= RPR0521_REGISTER_DUMP_END; i++) {
		reg = 0xff & i;
		mutex_lock(&sdata->mutex);
		err = regmap_read(sdata->regmap, reg, &reg_val);
		mutex_unlock(&sdata->mutex);
		if (err) {
			__debug("RPR0521 read error 0x%x\n", reg);
			return err;
		} else {
			__debug("RPR0521 read - reg 0x%x value 0x%02x \n", reg, reg_val);
		}
		len += sprintf(buf + len, "%02X ", reg_val);
	}

	len += sprintf(buf + len, "\n");
	return len;
}

static ssize_t rpr0521_sysfs_reg_write(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct rpr0521_data *sdata = dev_get_drvdata(dev);
	u8 reg;
	u8 reg_val;
	char *reg_str = vmalloc(sizeof(char) * 4);

	memcpy(reg_str, buf, 4);
	if (kstrtoul(reg_str, 0, &value))
		return -EINVAL;
	reg = 0xff & value;

	memcpy(reg_str, buf + 5, 4);
	if (kstrtoul(reg_str, 0, &value))
		return -EINVAL;
	reg_val = 0xff & value;

	mutex_lock(&sdata->mutex);
	err = regmap_write(sdata->regmap, reg, reg_val);
	mutex_unlock(&sdata->mutex);
	if (err < 0)
		__debug("RPR0521 write - reg 0x%x fail - %d \n", reg, err);
	else
		__debug("RPR0521 write - reg 0x%x value 0x%x \n", reg, reg_val);

	vfree(reg_str);
	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_reg_read = __ATTR(
	reg_read,
	0644,
	rpr0521_sysfs_reg_read,
	NULL);

static struct device_attribute dev_attr_reg_write = __ATTR(
	reg_write,
	0644,
	NULL,
	rpr0521_sysfs_reg_write);

static ssize_t rpr0521_sysfs_proximity_get_delay(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", sdata->proximity_delay);
}

static ssize_t rpr0521_sysfs_proximity_set_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct rpr0521_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	err = rpr0521_set_sensor_proximity_delay(sdata, value);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_proximity_delay = __ATTR(
	proximity_delay,
	0644,
	rpr0521_sysfs_proximity_get_delay,
	rpr0521_sysfs_proximity_set_delay);

static ssize_t rpr0521_sysfs_proximity_get_offset(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", sdata->pdata.proximity_offset);
}

static ssize_t rpr0521_sysfs_proximity_set_offset(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct rpr0521_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	err = rpr0521_set_ps_offset(sdata, value);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_proximity_offset = __ATTR(
	proximity_offset,
	0644,
	rpr0521_sysfs_proximity_get_offset,
	rpr0521_sysfs_proximity_set_offset);

static struct attribute *rpr0521_proximity_sysfs_attrs[] = {
	&dev_attr_proximity_enable.attr,
	&dev_attr_proximity_delay.attr,
	&dev_attr_proximity_offset.attr,
	&dev_attr_reg_read.attr,
	&dev_attr_reg_write.attr,
	NULL};

static struct attribute_group rpr0521_proximity_attribute_group = {
	.attrs = rpr0521_proximity_sysfs_attrs};

static int rpr0521_proximity_input_dev_register(struct rpr0521_data *sdata)
{
	int err;

	sdata->proximity_input_dev = input_allocate_device();
	if (!sdata->proximity_input_dev)
		return -ENOMEM;

	sdata->proximity_input_dev->name = RPR0521_INPUT_DEV_PROXIMITY;
	sdata->proximity_input_dev->id.bustype = BUS_I2C;
	sdata->proximity_input_dev->id.vendor = sdata->part_id;
	sdata->proximity_input_dev->dev.parent = &sdata->client->dev;

	set_bit(EVENT_TYPE, sdata->proximity_input_dev->evbit);
	input_set_abs_params(sdata->proximity_input_dev, PROXIMITY_EVENT_TYPE, 0, 32, 0, 0);  //dev,axis,min,max,fuzz,flat
	input_set_drvdata(sdata->proximity_input_dev, sdata);

	err = input_register_device(sdata->proximity_input_dev);

	if (err) {
		input_free_device(sdata->proximity_input_dev);
		sdata->proximity_input_dev = NULL;
		return err;
	}
	return 0;
}

static void rpr0521_proximity_input_dev_unregister(struct rpr0521_data *sdata)
{
	input_unregister_device(sdata->proximity_input_dev);
	sdata->proximity_input_dev = NULL;
}

static ssize_t rpr0521_sysfs_als_get_enable(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", sdata->als_enabled);
}

static ssize_t rpr0521_sysfs_als_set_enable(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct rpr0521_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	if (value)
		err = rpr0521_als_enable(sdata);
	else
		err = rpr0521_als_disable(sdata);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_als_enable = __ATTR(
	enable,
	0644,
	rpr0521_sysfs_als_get_enable,
	rpr0521_sysfs_als_set_enable);

static ssize_t rpr0521_sysfs_als_get_delay(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct rpr0521_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", sdata->als_delay);
}

static ssize_t rpr0521_sysfs_als_set_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct rpr0521_data *sdata = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&sdata->mutex);
	err = rpr0521_set_sensor_als_delay(sdata, value);
	mutex_unlock(&sdata->mutex);

	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_als_delay = __ATTR(
	als_delay,
	0644,
	rpr0521_sysfs_als_get_delay,
	rpr0521_sysfs_als_set_delay);

static struct attribute *rpr0521_als_sysfs_attrs[] = {
	&dev_attr_als_enable.attr,
	&dev_attr_als_delay.attr,
	&dev_attr_reg_read.attr,
	&dev_attr_reg_write.attr,
	NULL};

static struct attribute_group rpr0521_als_attribute_group = {
	.attrs = rpr0521_als_sysfs_attrs};

static int rpr0521_als_input_dev_register(struct rpr0521_data *sdata)
{
	int err;

	sdata->als_input_dev = input_allocate_device();
	if (!sdata->als_input_dev)
		return -ENOMEM;

	sdata->als_input_dev->name = RPR0521_INPUT_DEV_ALS;
	sdata->als_input_dev->id.bustype = BUS_I2C;
	sdata->als_input_dev->id.vendor = sdata->part_id;
	sdata->als_input_dev->dev.parent = &sdata->client->dev;

	set_bit(EVENT_TYPE, sdata->als_input_dev->evbit);
	input_set_abs_params(sdata->als_input_dev, ALS_LUX_EVENT_TYPE, 0, 43000, 0, 0);  //0.001-43k lux from specification
	input_set_drvdata(sdata->als_input_dev, sdata);

	err = input_register_device(sdata->als_input_dev);

	if (err) {
		input_free_device(sdata->als_input_dev);
		sdata->als_input_dev = NULL;
		return err;
	}
	return 0;
}

static void rpr0521_als_input_dev_unregister(struct rpr0521_data *sdata)
{
	input_unregister_device(sdata->als_input_dev);
	sdata->als_input_dev = NULL;
}

// Device tree related
static int rpr0521_parse_dt(struct rpr0521_data *sdata, struct device *dev)
{
	u32 temp_val;
	int err;

	// Mandatory device tree parameters, add what is needed
	err = of_property_read_u32(dev->of_node, "rohm,proximity-delay", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property proximity-delay, using default value 100\n");
		temp_val = 100;
	}
	err = rpr0521_set_sensor_proximity_delay(sdata, temp_val);
	if (err)
		return err;

	err = of_property_read_u32(dev->of_node, "rohm,als-delay", &temp_val);
	if (err) {
		dev_err(dev, "Unable to read property als-delay, using default value 100\n");
		temp_val = 100;
	}
	err = rpr0521_set_sensor_als_delay(sdata, temp_val);
	if (err)
		return err;

	// optional dt parameters i.e. use poll if interrupt not found
	sdata->pdata.use_drdy_int = of_property_read_bool(dev->of_node,
													"rohm,use-drdy-int");

	err = of_property_read_u32(dev->of_node, "rohm,proximity-offset", &temp_val);
	if (err) {
		temp_val = 0;
	}
	sdata->pdata.proximity_offset = temp_val & 0xffff;

	return 0;
}

inline static int rpr0521_check_manufacturer_id(struct i2c_client *client)
{
	int result = i2c_smbus_read_byte_data(client, RPR0521_MANUFACT);

	if (result < 0)
		return result;

	if (result != RPR0521_MANUFACT_ID_E0H) {
		dev_err(&client->dev, "Manufacturer id doesn't match RPR0521\n");
		return -ENODEV;
	}

	return 0;
}

inline static int rpr0521_check_part_id(struct i2c_client *client,
										struct rpr0521_data *sdata)
{
	int result = i2c_smbus_read_byte_data(client, RPR0521_SYSTEM_CONTROL);

	if (result < 0)
		return result;

	if ((result & RPR0521_SYSTEM_CONTROL_PART_MASK) !=
		RPR0521_SYSTEM_CONTROL_PART_ID) {
		dev_err(&client->dev, "No RPR0521 sensor\n");
		return -ENODEV;
	}

	sdata->part_id = result & RPR0521_SYSTEM_CONTROL_PART_MASK;
	return 0;
}

inline static int rpr0521_setup_regmap(struct rpr0521_data *sdata)
{
	sdata->regmap = devm_regmap_init_i2c(sdata->client, &rpr0521_regmap_config);
	if (IS_ERR(sdata->regmap))
		return PTR_ERR(sdata->regmap);

	return regcache_sync(sdata->regmap);
}

inline static void
rpr0521_setup_proximity_timer_thread(struct rpr0521_data *sdata)
{
	hrtimer_init(&sdata->proximity_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sdata->proximity_timer.function = rpr0521_proximity_timer_handler;

	init_waitqueue_head(&sdata->proximity_wq);
	sdata->proximity_wkp_flag = 0;
	sdata->proximity_task = kthread_run(rpr0521_proximity_read_thread, sdata,
										"rohm_proximity");
}

inline static void rpr0521_setup_als_timer_thread(struct rpr0521_data *sdata)
{
	hrtimer_init(&sdata->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	sdata->als_timer.function = rpr0521_als_timer_handler;

	init_waitqueue_head(&sdata->als_wq);
	sdata->als_wkp_flag = 0;
	sdata->als_task = kthread_run(rpr0521_als_read_thread, sdata,
								  "rohm_als");
}

static void rpr0521_set_no_pdata(struct rpr0521_data *sdata)
{
	u32 temp_val;

	temp_val = RPR0521_DEFAULT_PS_DELAY;
	(void)rpr0521_set_sensor_proximity_delay(sdata, temp_val);
	sdata->proximity_delay = RPR0521_DEFAULT_POLL_DELAY_MS;

	temp_val = RPR0521_DEFAULT_ALS_DELAY;
	(void)rpr0521_set_sensor_als_delay(sdata, temp_val);
	sdata->als_delay = RPR0521_DEFAULT_POLL_DELAY_MS;

	sdata->pdata.use_drdy_int = false;

	temp_val = RPR0521_DEFAULT_PS_OFFSET;
	sdata->pdata.proximity_offset = temp_val & 0xffff;

	return;
}

inline static int 
rpr0521_get_dev_of_node(struct i2c_client *client, struct rpr0521_data *sdata)
{
	// get driver platform data
	if (client->dev.of_node) {
		// Get data from device tree http://lwn.net/Articles/448502/
		int err;

		err = rpr0521_parse_dt(sdata, &client->dev);
		if (err) {
			dev_err(&client->dev, "Unable to parse dt data err=%d\n", err);
			return -EINVAL;
		}
	} else if (client->dev.platform_data) {
		// The driver has been instantiated in the traditional static way and
		// device is not needed http://lwn.net/Articles/448499/
		sdata->pdata = *(struct rpr0521_platform_data *)client->dev.platform_data;
	} else {
		rpr0521_set_no_pdata(sdata);
		dev_err(&client->dev, "Device tree of_node and platform data are NULL. Using defaults, no int\n");
	}

	return 0;
}

inline static void rpr0521_free_irq(struct rpr0521_data *sdata)
{
	if (sdata->irq_handle > 0 && sdata->pdata.use_drdy_int) {
		__debug("%s: free_irq\n", __FUNCTION__);
		free_irq(sdata->irq_handle, sdata);
		sdata->irq_handle = 0;
	}
}

// I2C Driver (probe/remove/tables)
static int
rpr0521_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rpr0521_data *sdata;
	int err;

	if (!i2c_check_functionality(client->adapter,
			(I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_I2C_BLOCK))) {
		dev_err(&client->dev, "No algorithm associated to the i2c bus\n");
		return -ENODEV;
	}

	err = rpr0521_check_manufacturer_id(client);
	if (err < 0)
		return err;

	sdata = devm_kzalloc(&client->dev, sizeof(struct rpr0521_data), GFP_KERNEL);
	if (!sdata) {
		dev_err(&client->dev, "No memory available\n");
		return -ENOMEM;
	}

	err = rpr0521_check_part_id(client, sdata);
	if (err < 0)
		goto free_init;

	dev_info(&client->dev, "ROHM RPR0521 detected\n");

	mutex_init(&sdata->mutex);
	mutex_lock(&sdata->mutex);

	sdata->client = client;
	i2c_set_clientdata(client, sdata);

	if (rpr0521_setup_regmap(sdata))
		goto free_init;

	if(rpr0521_get_dev_of_node(client, sdata) == -EINVAL)
		goto free_init;

	// Reset sensor to make sure interrupt is not triggered before
	// assigning interrupt handler
	err = rpr0521_soft_reset(sdata);
	if (err) {
		dev_err(&client->dev, "Soft reset failed\n");
		goto free_init;
	}

	if (sdata->pdata.init) {
		err = sdata->pdata.init();
		if (err) {
			dev_err(&client->dev, "Failed to initialize the device\n");
			goto free_init;
		}
	}

	// input_proximity device init
	err = rpr0521_proximity_input_dev_register(sdata);
	if (err < 0) {
		dev_err(&client->dev, "Proximity input register fail\n");
		goto free_src;
	}

	// sysfs_proximity init
	err = sysfs_create_group(&sdata->proximity_input_dev->dev.kobj,
							 &rpr0521_proximity_attribute_group);
	if (err) {
		dev_err(&client->dev, "Proximity sysfs create fail\n");
		goto free_input_proximity;
	}

	// input_als device init
	err = rpr0521_als_input_dev_register(sdata);
	if (err < 0) {
		dev_err(&client->dev, "ALS input register fail\n");
		goto free_sysfs_proximity;
	}

	// sysfs_als init
	err = sysfs_create_group(&sdata->als_input_dev->dev.kobj,
							 &rpr0521_als_attribute_group);
	if (err) {
		dev_err(&client->dev, "ALS sysfs create fail\n");
		goto free_input_als;
	}

	// setup gpio-pin to be interrupt, attach handler and let it wait for
	// turning on
	sdata->irq_handle = client->irq;
	if (sdata->pdata.use_drdy_int && sdata->irq_handle > 0) {
		// http://lxr.free-electrons.com/source/include/linux/interrupt.h?v=3.10#L23
		err = request_threaded_irq(sdata->irq_handle, NULL,
				rpr0521_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"rpr0521_irq", sdata);
		if (err) {
			dev_err(&client->dev,
				"unable to request threaded irq, err=%d\n", err);
			goto free_sysfs_als;
		}

		disable_irq(sdata->irq_handle);

		// RPR0521 sensor interrupt will be enabled when measurement is
		// enabled.
		err = rpr0521_set_interrupt_off(sdata);
		if (err) {
			dev_err(&client->dev, "unable to write interrupt setup (off)\n");
		}
	}

	rpr0521_setup_proximity_timer_thread(sdata);
	rpr0521_setup_als_timer_thread(sdata);

	// data report using irq or timer
	if (sdata->pdata.use_drdy_int && sdata->irq_handle > 0)
		sdata->use_poll_timer = false;
	else {
		sdata->use_poll_timer = true;
		__debug("%s: using timer mode\n", __FUNCTION__);
	}

	err = rpr0521_set_default_values(sdata);
	if (err) {
		dev_err(&client->dev, "Set default values failed\n");
		goto free_irq;
	}

	mutex_unlock(&sdata->mutex);
	return 0;

	// Abort _probe goto labels:
	free_irq:
		rpr0521_free_irq(sdata);
	free_sysfs_als:
		sysfs_remove_group(&sdata->als_input_dev->dev.kobj,
						&rpr0521_als_attribute_group);
	free_input_als:
		rpr0521_als_input_dev_unregister(sdata);
	free_sysfs_proximity:
		sysfs_remove_group(&sdata->proximity_input_dev->dev.kobj,
						&rpr0521_proximity_attribute_group);
	free_input_proximity:
		rpr0521_proximity_input_dev_unregister(sdata);
	free_src:
		if (sdata->pdata.release)
			sdata->pdata.release();
	free_init:
		mutex_unlock(&sdata->mutex);
		return err;
}

static int rpr0521_remove(struct i2c_client *client)
{
	struct rpr0521_data *sdata;

	sdata = i2c_get_clientdata(client);
	if (sdata->als_task) {
		hrtimer_cancel(&sdata->als_timer);
		kthread_stop(sdata->als_task);
		sdata->als_task = NULL;
	}
	if (sdata->proximity_task) {
		hrtimer_cancel(&sdata->proximity_timer);
		kthread_stop(sdata->proximity_task);
		sdata->proximity_task = NULL;
	}
	rpr0521_free_irq(sdata);
	sysfs_remove_group(&sdata->als_input_dev->dev.kobj,
					   &rpr0521_als_attribute_group);
	rpr0521_als_input_dev_unregister(sdata);
	sysfs_remove_group(&sdata->proximity_input_dev->dev.kobj,
					   &rpr0521_proximity_attribute_group);
	rpr0521_proximity_input_dev_unregister(sdata);
	if (sdata->pdata.release)
		sdata->pdata.release();

	dev_info(&sdata->client->dev, "ROHM RPR0521 removed\n");
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rpr0521_suspend(struct device *dev)
{
	return 0;
}

static int rpr0521_resume(struct device *dev)
{
	return 0;
}
#else
#define rpr0521_suspend NULL
#define rpr0521_resume NULL
#endif

#ifdef CONFIG_PM
static int rpr0521_runtime_suspend(struct device *dev)
{
	return 0;
}

static int rpr0521_runtime_resume(struct device *dev)
{
	return 0;
}
#else
#define rpr0521_runtime_suspend NULL
#define rpr0521_runtime_resume NULL
#endif

// Device data
static const struct i2c_device_id rpr0521_id[] = {
	{RPR0521_DEV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, rpr0521_id);

static const struct of_device_id rpr0521_of_match[] = {
	{
		.compatible = "rohm,rpr0521",
	},
	{},
};
MODULE_DEVICE_TABLE(of, rpr0521_of_match);

static const struct dev_pm_ops rpr0521_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rpr0521_suspend, rpr0521_resume)
	SET_RUNTIME_PM_OPS(rpr0521_runtime_suspend, rpr0521_runtime_resume, NULL)
};

static struct i2c_driver rpr0521_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = RPR0521_DEV_NAME,
		.pm = &rpr0521_pm_ops,
		.of_match_table = rpr0521_of_match,
	},
	.probe = rpr0521_probe,
	.remove = rpr0521_remove,
	.id_table = rpr0521_id,
};

module_i2c_driver(rpr0521_driver);

MODULE_DESCRIPTION("rpr0521 driver");
MODULE_AUTHOR("Rohm Semiconductor");
MODULE_LICENSE("GPL");
