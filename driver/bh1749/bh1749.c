// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * bh1749.c: driver for ROHM bh1749 Color sensor
 *
 * Copyright (C) 2018 ROHM Semiconductors
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include "bh1749_registers.h"

#define BH1749_MIN_INTERVAL_MS 35
#define BH1749_MAX_INTERVAL_MS 50

#define BH1749_MEAS_TIME_MIN 35
#define BH1749_MEAS_TIME_MAX 240

#define DEFAULT_RGB_GAIN BH1749_MODE_CONTROL1_RGB_GAIN_1X
#define DEFAULT_IR_GAIN BH1749_MODE_CONTROL1_IR_GAIN_1X
#define DEFAULT_MEAS_TIME BH1749_MODE_CONTROL1_ODR_4P167
#define DEFAULT_THRESHOLD_HIGH 0xFFFF
#define DEFAULT_THRESHOLD_LOW 0x0

#define BH1749_NAME "bh1749"

/* uapi definitions */

#define INPUT_EVENT_TYPE EV_MSC
#define MEAS_TYPE_RGB_RED MSC_SERIAL
#define MEAS_TYPE_RGB_GREEN MSC_PULSELED
#define MEAS_TYPE_RGB_BLUE MSC_GESTURE
#define MEAS_TYPE_RGB_IR MSC_SCAN
#define MEAS_TYPE_RGB_GREEN2 MSC_TIMESTAMP

struct bh1749_priv;
static int __bh1749_enable_measurement(struct bh1749_priv *);
static int __bh1749_disable_measurement(struct bh1749_priv *);
static int __bh1749_restart_measurement(struct bh1749_priv *);

/* raw measurement data */
struct bh1749_measurement {
	u16 red;
	u16 green;
	u16 blue;
	u16 ir;
	u16 green2;
};

/* operating_mode definitions */
#define BH1749_MODE_INT_JUDGE0 0 /* = always interrupt */
#define BH1749_MODE_INT_JUDGE1 1
#define BH1749_MODE_INT_JUDGE4 4
#define BH1749_MODE_INT_JUDGE8 8
#define BH1749_MODE_TIMER 0xff

/* local configuration values for bh1749, written to sensor
 * registers upon measurement start
 */
struct bh1749_config {
	u16 int_thh;
	u16 int_thl;
	u8 int_src;
	u8 ir_gain;
	u8 rgb_gain;
	u8 meas_time;
	u8 operating_mode;
};

struct bh1749_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct bh1749_config config;
	struct workqueue_struct *work_queue;
	struct delayed_work delayed_work;
	int read_delay_ms;
	struct work_struct irqwork;
	int irq;
	struct mutex drv_main_lock;
	int measurement_enabled;
};

/* restart measurement if it was already enabled */
static int __bh1749_restart_measurement(struct bh1749_priv *dev_priv)
{
	int ret = __bh1749_disable_measurement(dev_priv);

	if (ret)
		return ret;

	return __bh1749_enable_measurement(dev_priv);
}

static int __bh1749_set_config_and_restart(struct bh1749_priv *dev_priv,
					   u8 *value_to_set,
					   u8 value)
{
	bool disabled = false;
	int ret = 0;

	if (dev_priv->measurement_enabled) {
		ret = __bh1749_disable_measurement(dev_priv);
		if (ret < 0)
			return ret;
		disabled = true;
	}

	*value_to_set = value;

	if (disabled)
		ret = __bh1749_enable_measurement(dev_priv);

	return ret;
}

static int
bh1749_read_data(struct i2c_client *client, struct bh1749_measurement *data)
{
	int result;
	u8 read_data[12] = {0};

	result = i2c_smbus_read_i2c_block_data(
		client, BH1749_RED_DATA_LSBS, sizeof(read_data), read_data);
	if (result < 0) {
		dev_err(&client->dev, "i2c_smbus_read_i2c_block_data error\n");
		return result;
	}

	data->red = read_data[0] | (read_data[1] << 8);
	data->green = read_data[2] | (read_data[3] << 8);
	data->blue = read_data[4] | (read_data[5] << 8);
	data->ir = read_data[8] | (read_data[9] << 8);
	data->green2 = read_data[10] | (read_data[11] << 8);

	return 0;
}

static void
bh1749_report_data(struct input_dev *dev, struct bh1749_measurement *data)
{
	pr_debug("bh1749 data - r:%d, g:%d, b:%d, g2:%d, i:%d\n",
			 (int)data->red, (int)data->green, (int)data->blue,
			 (int)data->green2, (int)data->ir);

	input_event(dev, INPUT_EVENT_TYPE, MEAS_TYPE_RGB_RED, data->red);
	input_event(dev, INPUT_EVENT_TYPE, MEAS_TYPE_RGB_GREEN, data->green);
	input_event(dev, INPUT_EVENT_TYPE, MEAS_TYPE_RGB_BLUE, data->blue);
	input_event(dev, INPUT_EVENT_TYPE, MEAS_TYPE_RGB_IR, data->ir);
	input_event(dev, INPUT_EVENT_TYPE, MEAS_TYPE_RGB_GREEN2, data->green2);
	input_sync(dev);
}

static int bh1749_from_char2int_src(char in_char, u8 *out)
{
	switch (in_char) {
	case 'r':
	case 'R':
		*out = BH1749_INTERRUPT_SOURCE_SELECT_RED;
		break;
	case 'g':
	case 'G':
		*out = BH1749_INTERRUPT_SOURCE_SELECT_GREEN;
		break;
	case 'b':
	case 'B':
		*out = BH1749_INTERRUPT_SOURCE_SELECT_BLUE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static char bh1749_from_int_src2char(u8 int_src)
{
	switch (int_src) {
	case BH1749_INTERRUPT_SOURCE_SELECT_RED:
		return 'r';
	case BH1749_INTERRUPT_SOURCE_SELECT_GREEN:
		return 'g';
	case BH1749_INTERRUPT_SOURCE_SELECT_BLUE:
		return 'b';
	default:
		return 'r';
	}
}

static int bh1749_is_legal_gain(u32 gain)
{
	switch (gain) {
	case 1:
	case 32:
		return 1;
	default:
		return 0;
	}
}

static int bh1749_is_legal_meas_time(u32 meas_time)
{
	switch (meas_time) {
	case 35:
	case 120:
	case 240:
		return 1;
	default:
		return 0;
	}
}

static void bh1749_parse_dt_ir_gain(
	struct bh1749_config *config,
	struct device_node *dev_node, struct device *dev)
{
	u32 value;
	int ret = of_property_read_u32(dev_node, "rohm,ir-gain", &value);

	if (ret < 0) {
		dev_info(dev, "Unable to read ir-gain value, using default\n");
		config->ir_gain = DEFAULT_IR_GAIN;
	} else {
		if (bh1749_is_legal_gain(value)) {
			config->ir_gain = value;
		} else {
			dev_err(dev, "Illegal ir-gain value, using default\n");
			config->ir_gain = DEFAULT_IR_GAIN;
		}
	}
}

static void bh1749_parse_dt_rgb_gain(
	struct bh1749_config *config,
	struct device_node *dev_node, struct device *dev)
{
	u32 value;
	int ret = of_property_read_u32(dev_node, "rohm,rgb-gain", &value);

	if (ret < 0) {
		dev_info(dev, "Unable to read rgb-gain value, using default\n");
		config->rgb_gain = DEFAULT_RGB_GAIN;
	} else {
		if (bh1749_is_legal_gain(value)) {
			config->rgb_gain = value;
		} else {
			dev_err(dev, "Illegal rgb-gain value, using default\n");
			config->rgb_gain = DEFAULT_RGB_GAIN;
		}
	}
}

static void bh1749_parse_dt_meas_time(
	struct bh1749_config *config,
	struct device_node *dev_node, struct device *dev)
{
	u32 value;
	int ret = of_property_read_u32(dev_node,
			"rohm,measurement-time", &value);

	if (ret < 0) {
		dev_info(dev,
			"Unable to read measurement-time value, using default\n");
		config->meas_time = DEFAULT_MEAS_TIME;
	} else {
		if (bh1749_is_legal_meas_time(value)) {
			config->meas_time = value;
		} else {
			dev_err(dev, "Illegal measurement-time value, using default\n");
			config->meas_time = DEFAULT_MEAS_TIME;
		}
	}
}

static int bh1749_parse_dt_int_judge(
	struct bh1749_config *config, struct device_node *dev_node)
{
	if (of_property_read_bool(
			dev_node, "rohm,int-active-after-every-measurement"))
		config->operating_mode = BH1749_MODE_INT_JUDGE0;
	else if (of_property_read_bool(dev_node,
			"rohm,int-judge-after-1-measurement"))
		config->operating_mode = BH1749_MODE_INT_JUDGE1;
	else if (of_property_read_bool(dev_node,
			"rohm,int-judge-after-4-measurement"))
		config->operating_mode = BH1749_MODE_INT_JUDGE4;
	else if (of_property_read_bool(dev_node,
			"rohm,int-judge-after-8-measurement"))
		config->operating_mode = BH1749_MODE_INT_JUDGE8;
	else {
		config->operating_mode = BH1749_MODE_TIMER;
		return 1;
	}

	return 0;
}

static void bh1749_parse_dt_threshold(
	struct bh1749_config *config,
	struct device_node *dev_node, struct device *dev)
{
	u32 value;
	int ret = of_property_read_u32(
		dev_node, "rohm,interrupt-threshold-high", &value);

	if (ret < 0) {
		dev_info(dev, "Unable to read interrupt-threshold-high value.\n");
		value = DEFAULT_THRESHOLD_HIGH;
	}
	config->int_thh = value;

	ret = of_property_read_u32(
		dev_node, "rohm,interrupt-threshold-low", &value);
	if (ret < 0) {
		dev_info(dev, "Unable to read interrupt-threshold-low value.\n");
		value = DEFAULT_THRESHOLD_LOW;
	}
	config->int_thl = value;
}

static void bh1749_parse_dt_int_src(
	struct bh1749_config *config,
	struct device_node *dev_node, struct device *dev)
{
	const char *src_char;
	int ret = of_property_read_string(
		dev_node, "rohm,interrupt-source", &src_char);

	if (ret < 0) {
		dev_info(dev, "Unable to read interrupt-source value\n");
	} else {
		u8 int_src;

		if (bh1749_from_char2int_src(src_char[0], &int_src))
			dev_info(dev, "Invalid interrupt-source value\n");
		else
			config->int_src = int_src;
	}
}

static int bh1749_parse_dev_tree(
	struct bh1749_priv *dev_priv, struct device *dev)
{
	struct bh1749_config *config = &dev_priv->config;
	struct device_node *dev_node = dev->of_node;

	bh1749_parse_dt_ir_gain(config, dev_node, dev);
	bh1749_parse_dt_rgb_gain(config, dev_node, dev);
	bh1749_parse_dt_meas_time(config, dev_node, dev);

	/* no interrupts used, using timer mode, skip rest of this fn */
	if (bh1749_parse_dt_int_judge(config, dev_node))
		return 0;

	bh1749_parse_dt_threshold(config, dev_node, dev);
	bh1749_parse_dt_int_src(config, dev_node, dev);

	return 0;
}

static bool bh1749_is_measurement_data_valid(struct i2c_client *client)
{
	u8 reg_val = i2c_smbus_read_byte_data(client, BH1749_MODE_CONTROL2);

	if (reg_val < 0) {
		dev_err(&client->dev, "i2c_smbus_read_byte_data failed\n");
		return false;
	}

	return (reg_val & BH1749_MODE_CONTROL2_VALID_YES) ? true : false;
}

static int bh1749_start_measurement(struct i2c_client *client)
{
	int ret = i2c_smbus_write_byte_data(
		client,
		BH1749_MODE_CONTROL2,
		BH1749_MODE_CONTROL2_RGB_MEASUREMENT_ACTIVE);

	if (ret < 0)
		dev_err(&client->dev, "%s failed\n", __func__);

	return ret;
}

static int bh1749_stop_measurement(struct i2c_client *client)
{
	int ret = i2c_smbus_write_byte_data(
		client,
		BH1749_MODE_CONTROL2,
		BH1749_MODE_CONTROL2_RGB_MEASUREMENT_INACTIVE);
	if (ret < 0)
		dev_err(&client->dev, "%s failed\n", __func__);

	return ret;
}

static int __bh1749_set_irq(struct bh1749_priv *dev_priv, bool enable)
{
	u8 data = dev_priv->config.int_src;

	if (enable)
		data |= BH1749_INTERRUPT_EN_ENABLE;

	return i2c_smbus_write_byte_data(
		dev_priv->client, BH1749_INTERRUPT, data);
}

static int __bh1749_enable_irq(struct bh1749_priv *dev_priv)
{
	return __bh1749_set_irq(dev_priv, true);
}

static int __bh1749_disable_irq(struct bh1749_priv *dev_priv)
{
	return __bh1749_set_irq(dev_priv, false);
}

static int bh1749_read_and_report_data(struct bh1749_priv *dev_priv)
{
	struct bh1749_measurement data;
	int ret = bh1749_read_data(dev_priv->client, &data);

	if (ret)
		dev_err(&dev_priv->client->dev, "Failed to read data from BH1749\n");
	else
		bh1749_report_data(dev_priv->input_dev, &data);

	return ret;
}

static void bh1749_work(struct work_struct *work)
{
	struct bh1749_priv *dev_priv = container_of(
		work, struct bh1749_priv, delayed_work.work);

	if (!bh1749_is_measurement_data_valid(dev_priv->client)) {
		queue_delayed_work(
			dev_priv->work_queue, &dev_priv->delayed_work,
			msecs_to_jiffies(dev_priv->read_delay_ms) / 10);
		return;
	}

	bh1749_read_and_report_data(dev_priv);

	if (bh1749_stop_measurement(dev_priv->client) ||
		bh1749_start_measurement(dev_priv->client)) {
		dev_err(&dev_priv->client->dev,
				"Failed to restart measurement on BH1749\n");
		return;
	}

	queue_delayed_work(
		dev_priv->work_queue, &dev_priv->delayed_work,
		msecs_to_jiffies(dev_priv->read_delay_ms));
}

static u8 bh1749_get_ir_gain_reg_value(
	struct i2c_client *client, struct bh1749_config *config)
{
	switch (config->ir_gain) {
	case 1:
		return BH1749_MODE_CONTROL1_IR_GAIN_1X;
	case 32:
		return BH1749_MODE_CONTROL1_IR_GAIN_32X;
	default:
		dev_err(&client->dev,
				"illegal ir_gain configuration, setting default\n");
		return DEFAULT_IR_GAIN;
	}
}

static u8 bh1749_get_rgb_gain_reg_value(
	struct i2c_client *client, struct bh1749_config *config)
{
	switch (config->rgb_gain) {
	case 1:
		return BH1749_MODE_CONTROL1_RGB_GAIN_1X;
	case 32:
		return BH1749_MODE_CONTROL1_RGB_GAIN_32X;
	default:
		dev_err(&client->dev,
				"illegal rgb_gain configuration, setting default\n");
		return DEFAULT_RGB_GAIN;
	}
}

static u8 bh1749_get_meas_time_reg_value(
	struct i2c_client *client, struct bh1749_config *config)
{
	switch (config->meas_time) {
	case 35:
		return BH1749_MODE_CONTROL1_ODR_28P6;
	case 120:
		return BH1749_MODE_CONTROL1_ODR_8P333;
	case 240:
		return BH1749_MODE_CONTROL1_ODR_4P167;
	default:
		dev_err(
			&client->dev,
			"illegal meas_time configuration, setting default\n");
		return DEFAULT_MEAS_TIME;
	}
}

static u8 bh1749_get_persitence_reg_value(u8 operating_mode)
{
	switch (operating_mode) {
	case BH1749_MODE_INT_JUDGE0:
		return BH1749_PERSISTENCE_MODE_STATUS_ACTIVE_AFTER_MEASUREMENT;
	case BH1749_MODE_INT_JUDGE1:
		return BH1749_PERSISTENCE_MODE_STATUS_UPDATE_AFTER_MEASUREMENT;
	case BH1749_MODE_INT_JUDGE4:
		return BH1749_PERSISTENCE_MODE_STATUS_UPDATE_AFTER_4_SAME;
	case BH1749_MODE_INT_JUDGE8:
		return BH1749_PERSISTENCE_MODE_STATUS_UPDATE_AFTER_8_SAME;
	default:
		return 0;
	}
}

/* Write registers MODE_CONTROL1, TH_HIGH, TH_LOW, INTERRUPT and PERSISTENCE */
static int
__bh1749_set_config(struct i2c_client *client, struct bh1749_config *config)
{
	int ret = 0;
	u8 mode_ctrl1_val = 0;
	u8 regs[6] = {0};

	/* BH1749_MODE_CONTROL1 */
	mode_ctrl1_val |= bh1749_get_ir_gain_reg_value(client, config);
	mode_ctrl1_val |= bh1749_get_rgb_gain_reg_value(client, config);
	mode_ctrl1_val |= bh1749_get_meas_time_reg_value(client, config);

	ret = i2c_smbus_write_byte_data(
		client, BH1749_MODE_CONTROL1, mode_ctrl1_val);
	if (ret) {
		dev_err(&client->dev, "i2c_smbus_write_byte_data failed\n");
		return ret;
	}

	regs[0] = config->int_src;
	regs[1] = bh1749_get_persitence_reg_value(config->operating_mode);
	regs[2] = config->int_thh & 0xff;
	regs[3] = config->int_thh >> 8;
	regs[4] = config->int_thl & 0xff;
	regs[5] = config->int_thl >> 8;

	pr_debug(
		"bh1749 conf mode_ctrl1_val: 0x%02x, r[0]: 0x%02x, r[1]: 0x%02x, r[2]: 0x%02x, r[3]: 0x%02x, r[4]: 0x%02x, r[5]: 0x%02x\n",
		mode_ctrl1_val,
		regs[0], regs[1], regs[2], regs[3], regs[4], regs[5]);

	ret = i2c_smbus_write_i2c_block_data(
		client, BH1749_INTERRUPT, sizeof(regs), regs);
	if (ret)
		dev_err(&client->dev, "i2c_smbus_write_i2c_block failed\n");

	return ret;
}

static int
__bh1749_enable_measurement_operating_mode(struct bh1749_priv *dev_priv)
{
	int ret;

	if (dev_priv->config.operating_mode == BH1749_MODE_TIMER) {
		ret = bh1749_start_measurement(dev_priv->client);
		if (ret)
			return ret;

		queue_delayed_work(
			dev_priv->work_queue, &dev_priv->delayed_work,
			msecs_to_jiffies(dev_priv->read_delay_ms));
	} else {
		enable_irq(dev_priv->irq);
		__bh1749_enable_irq(dev_priv);

		ret = bh1749_start_measurement(dev_priv->client);
		if (ret) {
			__bh1749_disable_irq(dev_priv);
			disable_irq(dev_priv->irq);
			return ret;
		}
	}

	return 0;
}

static void
__bh1749_enable_measurement_set_read_delay(struct bh1749_priv *dev_priv)
{
	if (dev_priv->config.meas_time == BH1749_MEAS_TIME_MIN)
		dev_priv->read_delay_ms = BH1749_MIN_INTERVAL_MS;
	else
		dev_priv->read_delay_ms = BH1749_MAX_INTERVAL_MS;
}

static int
__bh1749_enable_measurement(struct bh1749_priv *dev_priv)
{
	int ret;

	if (dev_priv->measurement_enabled)
		return 0;

	ret = __bh1749_set_config(dev_priv->client, &dev_priv->config);
	if (ret) {
		dev_err(&dev_priv->client->dev,
			"Failed to set configuration on BH1749\n");
		return ret;
	}

	__bh1749_enable_measurement_set_read_delay(dev_priv);

	ret = __bh1749_enable_measurement_operating_mode(dev_priv);
	if (ret < 0)
		return ret;

	dev_priv->measurement_enabled = 1;
	return 0;
}

static int __bh1749_disable_measurement(struct bh1749_priv *dev_priv)
{
	int ret = 0;

	if (!dev_priv->measurement_enabled)
		return 0;

	if (dev_priv->config.operating_mode == BH1749_MODE_TIMER) {
		cancel_delayed_work_sync(&dev_priv->delayed_work);
	} else {
		__bh1749_disable_irq(dev_priv);
		disable_irq(dev_priv->irq);
		flush_workqueue(dev_priv->work_queue);
	}

	ret = bh1749_stop_measurement(dev_priv->client);
	if (ret < 0)
		return ret;

	dev_priv->measurement_enabled = 0;
	return 0;
}

static ssize_t bh1749_enable_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", dev_priv->measurement_enabled);
}

static ssize_t bh1749_enable_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&dev_priv->drv_main_lock);
	if (value)
		err = __bh1749_enable_measurement(dev_priv);
	else
		err = __bh1749_disable_measurement(dev_priv);

	mutex_unlock(&dev_priv->drv_main_lock);
	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_enable = __ATTR(
	enable, 0644, bh1749_enable_show, bh1749_enable_store);

static ssize_t bh1749_meas_time_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", (dev_priv->config.meas_time));
}

static int
bh1749_unlock_mutex_and_return(struct bh1749_priv *dev_priv, int ret)
{
	mutex_unlock(&dev_priv->drv_main_lock);
	return ret;
}

static ssize_t bh1749_meas_time_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	int ret = len;
	unsigned long value;
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (!bh1749_is_legal_meas_time(value))
		return -EINVAL;

	mutex_lock(&dev_priv->drv_main_lock);

	if (dev_priv->config.meas_time == value)
		return bh1749_unlock_mutex_and_return(dev_priv, ret);

	ret = __bh1749_set_config_and_restart(dev_priv,
					    &(dev_priv->config.meas_time),
					    value);

	return bh1749_unlock_mutex_and_return(dev_priv, ret);
}

static struct device_attribute dev_attr_meas_time = __ATTR(
	measurement_time, 0644, bh1749_meas_time_show, bh1749_meas_time_store);

static ssize_t bh1749_rgb_gain_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", (dev_priv->config.rgb_gain));
}

/* Stores RGB gain value to internal configuration
 * If measurement is running when this function is called, measurement
 * is restarted by this function
 */
static ssize_t bh1749_rgb_gain_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	int ret = len;
	unsigned long value;
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (!bh1749_is_legal_gain(value))
		return -EINVAL;

	mutex_lock(&dev_priv->drv_main_lock);

	if (dev_priv->config.rgb_gain == value)
		return bh1749_unlock_mutex_and_return(dev_priv, ret);

	ret = __bh1749_set_config_and_restart(dev_priv,
					    &(dev_priv->config.rgb_gain),
					    value);

	return bh1749_unlock_mutex_and_return(dev_priv, ret);
}

static struct device_attribute dev_attr_rgb_gain = __ATTR(
	rgb_gain, 0644, bh1749_rgb_gain_show, bh1749_rgb_gain_store);

static ssize_t bh1749_irgain_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", (dev_priv->config.ir_gain));
}

/* Stores IR gain value to internal configuration
 * If measurement is running when this function is called, measurement
 *  is restarted by this function
 */
static ssize_t bh1749_irgain_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	int ret = len;
	unsigned long value;
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (!bh1749_is_legal_gain(value))
		return -EINVAL;

	mutex_lock(&dev_priv->drv_main_lock);

	if (dev_priv->config.ir_gain == value)
		return bh1749_unlock_mutex_and_return(dev_priv, ret);


	ret = __bh1749_set_config_and_restart(dev_priv,
					    &(dev_priv->config.ir_gain),
					    value);

	return bh1749_unlock_mutex_and_return(dev_priv, ret);
}

static struct device_attribute dev_attr_ir_gain = __ATTR(
	ir_gain, 0644, bh1749_irgain_show, bh1749_irgain_store);

static ssize_t bh1749_source_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%c\n",
		bh1749_from_int_src2char(dev_priv->config.int_src));
}

static ssize_t bh1749_source_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	int ret = len;
	u8 int_from_char;
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	if (bh1749_from_char2int_src(buf[0], &int_from_char))
		return -EINVAL;

	mutex_lock(&dev_priv->drv_main_lock);

	if (dev_priv->config.operating_mode == BH1749_MODE_TIMER) {
		ret = -EINVAL;
		return bh1749_unlock_mutex_and_return(dev_priv, ret);
	}

	if (dev_priv->config.int_src == int_from_char) {
		ret = len;
		return bh1749_unlock_mutex_and_return(dev_priv, ret);
	}

	ret = __bh1749_set_config_and_restart(dev_priv,
					    &(dev_priv->config.int_src),
					    int_from_char);

	return bh1749_unlock_mutex_and_return(dev_priv, ret);
}

static struct device_attribute dev_attr_source = __ATTR(
	int_source, 0644, bh1749_source_show, bh1749_source_store);

static ssize_t bh1749_judgement_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	return snprintf(
		buf, PAGE_SIZE, "%d\n", dev_priv->config.operating_mode);
}

/* Store interrupt judgement value to local variable, if measurement is
 * operational when this function is called, the measurement is restarted by
 * this function.
 *
 * Calling this function in timer mode (no interrupts used) returns error.
 */
static ssize_t bh1749_judgement_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	int ret = len;
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&dev_priv->drv_main_lock);

	if (dev_priv->config.operating_mode == BH1749_MODE_TIMER) {
		ret = -EINVAL;
		return bh1749_unlock_mutex_and_return(dev_priv, ret);
	}

	if (dev_priv->config.operating_mode == value) {
		ret = len;
		return bh1749_unlock_mutex_and_return(dev_priv, ret);
	}

	switch (value) {
	case BH1749_MODE_INT_JUDGE0:
	case BH1749_MODE_INT_JUDGE1:
	case BH1749_MODE_INT_JUDGE4:
	case BH1749_MODE_INT_JUDGE8:
		break;
	default:
		ret = -EINVAL;
		return bh1749_unlock_mutex_and_return(dev_priv, ret);
	}

	ret = __bh1749_set_config_and_restart(dev_priv,
					    &(dev_priv->config.operating_mode),
					    value);
	return bh1749_unlock_mutex_and_return(dev_priv, ret);
}

static struct device_attribute dev_attr_judgement = __ATTR(
	int_judgement, 0644, bh1749_judgement_show, bh1749_judgement_store);

static ssize_t bh1749_low_threshold_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", dev_priv->config.int_thl);
}

/* Stores low threshold value to local configuration. If the measurement is
 * operational when this function is called and the interrupt judgement is
 * used, the measurement is restarted
 */
static ssize_t bh1749_low_threshold_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	int ret = len;
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&dev_priv->drv_main_lock);

	if (dev_priv->config.int_thl == value)
		return bh1749_unlock_mutex_and_return(dev_priv, ret);

	dev_priv->config.int_thl = value;

	/* changing threshold causes measurement restart if judgement is used */
	if (dev_priv->config.operating_mode == BH1749_MODE_INT_JUDGE1 ||
		dev_priv->config.operating_mode == BH1749_MODE_INT_JUDGE4 ||
		dev_priv->config.operating_mode == BH1749_MODE_INT_JUDGE8) {
		if (__bh1749_restart_measurement(dev_priv)) {
			dev_err(dev, "Failed to restart measurement\n");
			ret = -EIO;
		}
	}

	return bh1749_unlock_mutex_and_return(dev_priv, ret);
}

static struct device_attribute dev_attr_threshold_low = __ATTR(
	int_threshold_low,
	0644,
	bh1749_low_threshold_show,
	bh1749_low_threshold_store);

static ssize_t bh1749_high_threshold_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", dev_priv->config.int_thh);
}

/* Stores high threshold value to local configuration. If the measurement is
 * operational when this function is called and the interrupt judgement is
 * used, the measurement is restarted
 */
static ssize_t bh1749_high_threshold_store(
	struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	int ret = len;
	struct bh1749_priv *dev_priv = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&dev_priv->drv_main_lock);

	if (dev_priv->config.int_thh == value)
		return bh1749_unlock_mutex_and_return(dev_priv, ret);

	dev_priv->config.int_thh = value;

	/* changing threshold causes measurement restart if judgement used */
	if (dev_priv->config.operating_mode == BH1749_MODE_INT_JUDGE1 ||
		dev_priv->config.operating_mode == BH1749_MODE_INT_JUDGE4 ||
		dev_priv->config.operating_mode == BH1749_MODE_INT_JUDGE8) {
		if (__bh1749_restart_measurement(dev_priv)) {
			dev_err(dev, "Failed to restart measurement\n");
			ret = -EIO;
		}
	}
	return bh1749_unlock_mutex_and_return(dev_priv, ret);
}

static struct device_attribute dev_attr_threshold_high = __ATTR(
	int_threshold_high,
	0644,
	bh1749_high_threshold_show,
	bh1749_high_threshold_store);

static struct attribute *bh1749_sysfs_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_threshold_high.attr,
	&dev_attr_threshold_low.attr,
	&dev_attr_judgement.attr,
	&dev_attr_source.attr,
	&dev_attr_ir_gain.attr,
	&dev_attr_rgb_gain.attr,
	&dev_attr_meas_time.attr,
	NULL};

static struct attribute_group bh1749_attribute_group = {
	.attrs = bh1749_sysfs_attrs};

static void bh1749_ack_irq(struct bh1749_priv *dev_priv)
{
	i2c_smbus_read_byte_data(dev_priv->client, BH1749_INTERRUPT);
}

static void bh1749_irq_work_read_and_ack_irq(struct work_struct *work)
{
	struct bh1749_priv *dev_priv = container_of(
		work, struct bh1749_priv, irqwork);

	bh1749_read_and_report_data(dev_priv);
	bh1749_ack_irq(dev_priv);
}

static irqreturn_t bh1749_irq_handler_schedule_work(int irq, void *data)
{
	struct bh1749_priv *dev_priv = data;

	queue_work(dev_priv->work_queue, &dev_priv->irqwork);
	return IRQ_HANDLED;
}

static int bh1749_check_manufacturer_id(struct i2c_client *client)
{
	int result = i2c_smbus_read_byte_data(client, BH1749_ID_REG);

	if (result < 0)
		return result;

	if (result != BH1749_ID_REG_MANUFACTURER_ID) {
		dev_err(&client->dev, "Manufacturer id doesn't match BH1749\n");
		return -ENODEV;
	}

	return 0;
}

static int bh1749_check_part_id(struct i2c_client *client)
{
	int result = i2c_smbus_read_byte_data(client, BH1749_SYSTEM_CONTROL);

	if (result < 0)
		return result;

	if ((result & BH1749_SYSTEM_CONTROL_PART_MASK) !=
		BH1749_SYSTEM_CONTROL_PART_ID) {
		dev_err(&client->dev, "no BH1749 sensor\n");
		return -ENODEV;
	}

	return 0;
}

static void bh1749_assign_input_dev_defaults(struct bh1749_priv *dev_priv)
{
	dev_priv->input_dev->name = BH1749_NAME;
	dev_priv->input_dev->id.bustype = BUS_I2C;
	dev_priv->input_dev->id.vendor = BH1749_ID_REG_MANUFACTURER_ID;

	set_bit(INPUT_EVENT_TYPE, dev_priv->input_dev->evbit);
	set_bit(MEAS_TYPE_RGB_RED, dev_priv->input_dev->mscbit);
	set_bit(MEAS_TYPE_RGB_GREEN, dev_priv->input_dev->mscbit);
	set_bit(MEAS_TYPE_RGB_BLUE, dev_priv->input_dev->mscbit);
	set_bit(MEAS_TYPE_RGB_IR, dev_priv->input_dev->mscbit);
	set_bit(MEAS_TYPE_RGB_GREEN2, dev_priv->input_dev->mscbit);

	input_set_drvdata(dev_priv->input_dev, dev_priv);
}

static bool bh1749_is_timer_mode(struct bh1749_priv *dev_priv)
{
	if (dev_priv->config.operating_mode != BH1749_MODE_TIMER)
		return false;

	return true;
}

static int
bh1749_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bh1749_priv *dev_priv;
	int err;

	if (!i2c_check_functionality(
			client->adapter,
			I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_BYTE_DATA |
				I2C_FUNC_SMBUS_READ_I2C_BLOCK |
				I2C_FUNC_SMBUS_WRITE_I2C_BLOCK))
		return -EOPNOTSUPP;

	err = bh1749_check_manufacturer_id(client);
	if (err < 0)
		return err;

	err = bh1749_check_part_id(client);
	if (err < 0)
		return err;

	dev_info(&client->dev, "ROHM bh1749 detected\n");

	dev_priv = devm_kzalloc(
		&client->dev, sizeof(struct bh1749_priv), GFP_KERNEL);
	if (!dev_priv)
		return -ENOMEM;

	i2c_set_clientdata(client, dev_priv);
	dev_priv->client = client;
	mutex_init(&dev_priv->drv_main_lock);

	err = bh1749_parse_dev_tree(dev_priv, &client->dev);
	if (err) {
		dev_err(&client->dev,
			"Unable to parse dt data error = %d\n", err);
		return -EINVAL;
	}

	dev_priv->input_dev = devm_input_allocate_device(&client->dev);
	if (!dev_priv->input_dev)
		return -ENOMEM;

	bh1749_assign_input_dev_defaults(dev_priv);

	dev_priv->work_queue = create_singlethread_workqueue("bh1749_queue");
	if (!dev_priv->work_queue) {
		dev_err(&client->dev, "can't create workqueue\n");
		return -ENOMEM;
	}

	if (bh1749_is_timer_mode(dev_priv)) {
		INIT_DELAYED_WORK(&dev_priv->delayed_work, bh1749_work);
	} else {
		unsigned long irq_flags;

		INIT_WORK(&dev_priv->irqwork, bh1749_irq_work_read_and_ack_irq);
		dev_priv->irq = client->irq;
		irq_flags = irq_get_trigger_type(client->irq);
		if (irq_flags == IRQF_TRIGGER_NONE)
			irq_flags = IRQF_TRIGGER_FALLING;
		irq_flags |= IRQF_ONESHOT;

		err = devm_request_threaded_irq(&client->dev, dev_priv->irq,
			NULL, bh1749_irq_handler_schedule_work,

			irq_flags, "bh1749_irq", dev_priv);

		if (err) {
			dev_err(&client->dev, "devm_request_threaded_irq failed\n");
			goto devm_request_threaded_irq_fail;
		}
		disable_irq(dev_priv->irq);
	}

	err = input_register_device(dev_priv->input_dev);
	if (err) {
		dev_err(&client->dev, "input_register_device failed\n");
		goto input_register_device_fail;
	}

	err = sysfs_create_group(
		&dev_priv->input_dev->dev.kobj, &bh1749_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs create fail\n");
		goto sysfs_create_group_fail;
	}
	return 0;

sysfs_create_group_fail:
input_register_device_fail:
devm_request_threaded_irq_fail:
	if (dev_priv->work_queue)
		destroy_workqueue(dev_priv->work_queue);

	return err;
}

static int bh1749_remove(struct i2c_client *client)
{
	struct bh1749_priv *dev_priv = i2c_get_clientdata(client);

	sysfs_remove_group(
		&dev_priv->input_dev->dev.kobj, &bh1749_attribute_group);

	mutex_lock(&dev_priv->drv_main_lock);
	__bh1749_disable_measurement(dev_priv);
	mutex_unlock(&dev_priv->drv_main_lock);

	if (dev_priv->work_queue)
		destroy_workqueue(dev_priv->work_queue);
	input_unregister_device(dev_priv->input_dev);
	dev_info(&client->dev, "ROHM bh1749 removed\n");

	return 0;
}

static const struct of_device_id bh1749_of_match[] = {
	{
		.compatible = "rohm,bh1749",
	},
	{},
};

static const struct i2c_device_id bh1749_id[] = {
	{BH1749_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, bh1749_id);

static struct i2c_driver bh1749_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = BH1749_NAME,
		.of_match_table = bh1749_of_match,
	},
	.probe = bh1749_probe,
	.remove = bh1749_remove,
	.id_table = bh1749_id,
};

module_i2c_driver(bh1749_driver);
MODULE_AUTHOR("Mikko Mutanen <mikko.mutanen@fi.rohmeurope.com>");
MODULE_AUTHOR("Assam Boudjelthia <assam.boudjelthia@fi.rohmeurope.com>");
MODULE_DESCRIPTION("ROHM Ambient Light Sensor Driver");
MODULE_LICENSE("GPL");
