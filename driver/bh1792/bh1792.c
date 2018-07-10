// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * bh1792.c: driver for ROHM bh1792 Optical sensor for heart rate monitoring
 *
 * Copyright(C) 2018 Rohm Semiconductor
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>

#include "bh1792glc_registers.h"
#include <linux/workqueue.h>

#define BH1792_NAME "bh1792"

#define FIFO_WATERMARK 32

#define INPUT_EVENT_TYPE        EV_MSC
#define INPUT_EVENT_LED         MSC_SERIAL
#define INPUT_EVENT_TIME_MSB    MSC_PULSELED
#define INPUT_EVENT_TIME_LSB    MSC_SCAN

#define LED_CURRENT_MAX 0x3F

/*
 * Three modes of operation are supported by the sensor:
 *	1. syncronized, synchronization is done once per second.
 *	Measurement results are stored in FIFO and interrupt is generated when
 *	watermark is reached. Only available with Green leds.
 *	2. non synchronized mode, after measurement started its running and if
 *	interrupt condition is met, interrupt is raised. This driver implements
 *	only IR in this mode.
 *	3. single-measurement mode. Measurement is done once, interrupt is
 *	raised when measurement is finished. Available with both IR and Green
 *	leds.
 */

struct bh1792_priv;

static uint sync_freq_tbl[] = { 32, 128, 64, 256, 1, 1024 };

/* raw measurement data */
struct bh1792_data {
	u16 data0;
	u16 data1;
};

/* opmodes */
#define BH1792_MODE_SYNCHRONIZED     0
#define BH1792_MODE_NON_SYNCHRONIZED 1
#define BH1792_MODE_SINGLE_GREEN     2
#define BH1792_MODE_SINGLE_IR        3

/* configuration values for bh1792, written to sensor
 * registers during measurement start
 */
struct bh1792_config {
	bool update;
	u8 opmode;
	u8 led1_current;
	u8 led2_current;
	u8 ir_thl;
	u8 ir_thh;
	u16 freq;
};

struct bh1792_priv {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *wq;
	struct bh1792_config config;
	struct mutex conf_lock;
	int gpio_int;
	int irq;

	wait_queue_head_t sync_wq;
	struct hrtimer sync_timer;
	struct task_struct *sync_task;
	bool sync_flag;
	bool sync_run;

	struct work_struct irqwork;
	bool enabled;
	bool second_sync_sent;
};

/*
 *Common helper functions to all modes
 */

static int
bh1792_read_data(struct i2c_client *client, u8 reg, struct bh1792_data data[],
		 int count)
{
	int i;
	u8 d[4] = { 0 };

	for (i = 0; i < count; i++) {
		int result =
		    i2c_smbus_read_i2c_block_data(client, reg, sizeof(d), d);
		if (result < 0) {
			dev_err(&client->dev,
				"i2c_smbus_read_i2c_block_data error\n");
			return result;
		}

		data[i].data0 = d[0] | (d[1] << 8);
		data[i].data1 = d[2] | (d[3] << 8);
	}

	return count;
}

/* reads fifo and discards data */
static int bh1792_discard_fifo(struct bh1792_priv *p)
{
	int r;
	struct bh1792_data data[FIFO_WATERMARK];

	r = bh1792_read_data(p->client, BH1792_FIFO_DATA0_L, data,
			     FIFO_WATERMARK);
	if (r < 0) {
		dev_err(&p->client->dev,
			"i2c_smbus_read_i2c_block_data error\n");
		return r;
	}
	return 0;
}

static void bh1792_report_data(struct input_dev *dev, struct timespec ts,
			       struct bh1792_data data[], int count)
{
	u64 current_time;
	int i;

	current_time = timespec_to_ns(&ts);

	input_event(dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_MSB,
		    current_time >> 32);
	input_event(dev, INPUT_EVENT_TYPE, INPUT_EVENT_TIME_LSB,
		    current_time & 0xFFFFFFFF);

	for (i = 0; i < count; i++) {
		input_event(dev, INPUT_EVENT_TYPE, INPUT_EVENT_LED,
			    (data[i].data0 << 16) | data[i].data1);

		pr_debug("bh1792 led off: 0x%04x on: 0x%04x\n",
			 (int)data[i].data0, (int)data[i].data1);
	}
	input_sync(dev);

}

static int parse_freq(u16 freq, u16 *out)
{
	switch (freq) {
	case 32:
		*out = BH1792_MEAS_CONTROL1_MSR_32HZ;
		break;
	case 128:
		*out = BH1792_MEAS_CONTROL1_MSR_128HZ;
		break;
	case 64:
		*out = BH1792_MEAS_CONTROL1_MSR_64HZ;
		break;
	case 256:
		*out = BH1792_MEAS_CONTROL1_MSR_256HZ;
		break;
	case 1024:
		*out = BH1792_MEAS_CONTROL1_MSR_1024HZ;
		break;
	default:
		*out = BH1792_MEAS_CONTROL1_MSR_32HZ;
		return -1;
	}
	return 0;
}

static void dt_u32_read(struct device *dev,
		   struct device_node *np,
		   char *propname,
		   u32 *value)
{
	u32 temp_val = 0;

	if (of_property_read_u32(np, propname, &temp_val)) {
		dev_info(dev,
			 "Unable to read property %s. Default is %d\n",
			 propname, *value);
	} else {
		*value = temp_val;
	}
}

static void dt_u8_read(struct device *dev,
		     struct device_node *np,
		     char *propname,
		     u8 *value)
{
	u32 tmp_value = *value;

	dt_u32_read(dev, np, propname, &tmp_value);
	*value = (u8)tmp_value;
}

/* update config value if found from dt, otherwise keep default */
static int bh1792_parse_dt(struct bh1792_priv *p, struct device *dev)
{
	u32 temp_val;
	struct device_node *np = dev->of_node;

	p->gpio_int = of_get_named_gpio_flags(np, "rohm,irq-gpio", 0, NULL);

	dt_u8_read(dev, np, "rohm,operation-mode", &p->config.opmode);
	dt_u8_read(dev, np, "rohm,led1-current", &p->config.led1_current);
	dt_u8_read(dev, np, "rohm,led2-current", &p->config.led2_current);

	temp_val = (p->config.ir_thh<<8)|(p->config.ir_thl);
	dt_u32_read(dev, np, "rohm,ir-threshold", &temp_val);
	p->config.ir_thl = 0xff & temp_val;
	p->config.ir_thh = temp_val >> 8;

	temp_val = p->config.freq;
	dt_u32_read(dev, np, "rohm,frequency", &temp_val);
	parse_freq(temp_val, &(p->config.freq));

	p->config.update = 1;

	return 0;
}

static int bh1792_sw_reset(struct i2c_client *client)
{
	int err = i2c_smbus_write_byte_data(client, BH1792_RESET,
					    BH1792_RESET_SWRESET);
	if (err < 0) {
		dev_err(&client->dev, "Failed to set sw reset on BH1792\n");
		return err;
	}
	return 0;
}

static int bh1792_start_measurement(struct i2c_client *client)
{
	int err = i2c_smbus_write_byte_data(client, BH1792_MEAS_START,
					    BH1792_MEAS_START_MEAS_ST);
	if (err < 0) {
		dev_err(&client->dev, "Failed to write MEAS_ST on BH1792\n");
		return err;
	}
	return 0;
}

static int bh1792_clear_interrupt(struct i2c_client *client)
{
	int err = i2c_smbus_read_byte_data(client, BH1792_INT_CLEAR);

	if (err < 0) {
		dev_err(&client->dev, "Failed to clear interrupts on BH1792\n");
		return err;
	}
	return 0;
}

/* writes configuration to bh1792 */
static int _update_config(struct bh1792_priv *p, bool sync)
{
	int ret;
	u8 regs[8] = { 0 };
	struct bh1792_config *c = &p->config;

	if (!c->update)
		return 0;

	/* BH1792_MEAS_CONTROL1 */
	regs[0] = BH1792_MEAS_CONTROL1_RDY;

	/* BH1792_MEAS_CONTROL2 */
	regs[1] = c->led1_current & BH1792_MEAS_CONTROL2_LED_CURRENT1_MASK;

	/* BH1792_MEAS_CONTROL3 */
	regs[2] = c->led2_current & BH1792_MEAS_CONTROL3_LED_CURRENT2_MASK;

	/* BH1792_MEAS_CONTROL4_L */
	regs[3] = c->ir_thl;

	/* BH1792_MEAS_CONTROL4_H */
	regs[4] = c->ir_thh;

	/* BH1792_MEAS_CONTROL5 */
	switch (c->opmode) {
	case BH1792_MODE_SYNCHRONIZED:
		regs[0] |= BH1792_MEAS_CONTROL1_SEL_ADC_GREEN;
		regs[0] |= c->freq;
		regs[5] = BH1792_MEAS_CONTROL5_INT_SEL_FIFO_WATERMARK;
		break;
	case BH1792_MODE_NON_SYNCHRONIZED:
		regs[0] |= BH1792_MEAS_CONTROL1_MSR_NON_SYNCH_MODE;
		regs[0] |= BH1792_MEAS_CONTROL1_SEL_ADC_IR;
		regs[5] = BH1792_MEAS_CONTROL5_INT_SEL_IR_THRESHOLD;
		break;
	case BH1792_MODE_SINGLE_GREEN:
		regs[0] |= BH1792_MEAS_CONTROL1_SEL_ADC_GREEN;
		regs[0] |= BH1792_MEAS_CONTROL1_MSR_SINGLE_MEAS_MODE;
		regs[5] = BH1792_MEAS_CONTROL5_INT_SEL_ON_COMPLETE;
		break;
	case BH1792_MODE_SINGLE_IR:
		regs[0] |= BH1792_MEAS_CONTROL1_MSR_SINGLE_MEAS_MODE;
		regs[0] |= BH1792_MEAS_CONTROL1_SEL_ADC_IR;
		regs[5] = BH1792_MEAS_CONTROL5_INT_SEL_ON_COMPLETE;
		break;
	}

	/* BH1792_MEAS_START */
	regs[6] = BH1792_MEAS_START_MEAS_ST;

	pr_debug
	    ("bh1792 conf r[0]: 0x%02x, r[1]: 0x%02x, r[2]: 0x%02x, r[3]: 0x%02x, r[4]: 0x%02x, r[5]: 0x%02x, r[6]: 0x%02x\n",
	     regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6]);

	/* BH1792_MEAS_SYNC */
	regs[7] = BH1792_MEAS_SYNC_MEAS_SYNC;
	if (sync)		/* write with sync bit */
		ret = i2c_smbus_write_i2c_block_data(p->client,
						     BH1792_MEAS_CONTROL1,
						     sizeof(regs), regs);
	else			/* write without sync bit */
		ret = i2c_smbus_write_i2c_block_data(p->client,
						     BH1792_MEAS_CONTROL1,
						     sizeof(regs) - 1, regs);
	if (ret) {
		dev_err(&p->client->dev, "i2c_smbus_write_i2c_block failed\n");
		return ret;
	}

	c->update = 0;

	return 0;
}

/* Synchronization thread */
static int bh1792_sync_thread(void *data)
{
	struct bh1792_priv *p = data;

	while (1) {
		wait_event_interruptible(p->sync_wq, p->sync_flag
					 || kthread_should_stop());

		p->sync_flag = 0;

		if (kthread_should_stop())
			break;

		if (i2c_smbus_write_byte_data(p->client,
					      BH1792_MEAS_SYNC,
					      BH1792_MEAS_SYNC_MEAS_SYNC)) {
			dev_err(&p->client->dev,
				"Failed to synch with BH1792\n");
		}

		/* first sync run */
		if (!p->second_sync_sent) {
			bh1792_discard_fifo(p);
			p->second_sync_sent = true;
		}

	}
	return 0;
}

static enum hrtimer_restart sync_timer_handler(struct hrtimer *handle)
{
	struct bh1792_priv *p =
	    container_of(handle, struct bh1792_priv, sync_timer);

	if (!p->sync_run)
		return HRTIMER_NORESTART;

	hrtimer_forward_now(&p->sync_timer, ktime_set(1, 0));
	p->sync_flag = 1;
	wake_up_interruptible(&p->sync_wq);
	return HRTIMER_RESTART;
}

static int _start_synchronized_measurement(struct bh1792_priv *p)
{
	int ret;

	p->second_sync_sent = 0;

	hrtimer_start(&p->sync_timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	enable_irq(p->irq);

	/* set configuration and send first sync */
	ret = _update_config(p, 1);
	if (ret) {
		dev_err(&p->client->dev, "updating config failed\n");
		disable_irq(p->irq);
		return ret;
	}

	p->sync_run = 1;

	return 0;
}

static int _start_measurement(struct bh1792_priv *p)
{
	int ret;

	bh1792_clear_interrupt(p->client);

	/* force writing of settings to hw */
	p->config.update = 1;

	if (p->config.opmode == BH1792_MODE_SYNCHRONIZED)
		return _start_synchronized_measurement(p);

	enable_irq(p->irq);

	ret = _update_config(p, 0);

	if (ret) {
		dev_err(&p->client->dev, "updating config failed\n");
		disable_irq(p->irq);
		return ret;
	}

	return 0;
}

static int _stop_measurement(struct bh1792_priv *p)
{
	p->sync_run = 0;
	disable_irq(p->irq);
	bh1792_sw_reset(p->client);
	/* ensure that irqwork is stopped */
	flush_workqueue(p->wq);

	if (p->config.opmode == BH1792_MODE_SYNCHRONIZED)
		hrtimer_cancel(&p->sync_timer);

	return 0;
}

static int _enable_measurement(struct bh1792_priv *p)
{
	int err;

	if (p->enabled) {
		/* in single modes new enable will trigger new measurement by
		 *  writing meas_st, reject re-enable in other modes
		 */
		if (p->config.opmode == BH1792_MODE_SINGLE_GREEN ||
		    p->config.opmode == BH1792_MODE_SINGLE_IR)
			bh1792_start_measurement(p->client);

		return 0;
	}

	err = _start_measurement(p);
	if (err)
		return err;
	p->enabled = true;
	return 0;
}

static int _disable_measurement(struct bh1792_priv *p)
{
	int err;

	if (!p->enabled)
		return 0;

	err = _stop_measurement(p);
	if (err)
		return err;
	p->enabled = false;
	return 0;
}

static void bh1792_irq_work(struct work_struct *work)
{
	struct bh1792_priv *p = container_of(work, struct bh1792_priv, irqwork);
	struct bh1792_data d[FIFO_WATERMARK] = { {0} };
	int count = FIFO_WATERMARK;
	u8 read_addr = BH1792_FIFO_DATA0_L;
	struct timespec ts;

	get_monotonic_boottime(&ts);

	switch (p->config.opmode) {
	case BH1792_MODE_SYNCHRONIZED:
		if (!p->second_sync_sent)
			return;
		break;
	case BH1792_MODE_SINGLE_GREEN:
		read_addr = BH1792_DATAOUT_LEDOFF_L;
		count = 1;
		break;
	case BH1792_MODE_NON_SYNCHRONIZED:
	case BH1792_MODE_SINGLE_IR:
		read_addr = BH1792_IRDATA_LEDOFF_L;
		count = 1;
		break;
	}

	if (bh1792_read_data(p->client, read_addr, d, count) < 0) {
		dev_err(&p->client->dev, "Failed to read data from BH1792\n");
		return;
	}

	/* according to datasheet v.002 one must read FIFO level here */
	if (p->config.opmode == BH1792_MODE_SYNCHRONIZED) {
		int fifo_lev = i2c_smbus_read_byte_data(p->client,
							BH1792_FIFO_LEV);

		if (fifo_lev < 0) {
			dev_err(&p->client->dev,
				"Failed to read fifo level from BH1792\n");
			return;
		}
		_update_config(p, 0);
	} else {
		bh1792_clear_interrupt(p->client);
	}

	bh1792_report_data(p->input_dev, ts, d, count);
}

static irqreturn_t bh1792_irq_handler(int irq, void *data)
{
	struct bh1792_priv *p = data;

	queue_work(p->wq, &p->irqwork);
	return IRQ_HANDLED;
}

static ssize_t
bh1792_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1792_priv *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", p->enabled);
}

static ssize_t
bh1792_enable_store(struct device *dev,
		    struct device_attribute *attr, const char *buf, size_t len)
{
	int err;
	unsigned long value;
	struct bh1792_priv *p = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&p->conf_lock);

	if (value)
		err = _enable_measurement(p);
	else
		err = _disable_measurement(p);

	mutex_unlock(&p->conf_lock);

	return (err < 0) ? err : len;
}

static struct device_attribute dev_attr_enable = __ATTR(enable,
							0644,
							bh1792_enable_show,
							bh1792_enable_store);

static ssize_t
bh1792_led1_current_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct bh1792_priv *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", p->config.led1_current);
}

static ssize_t
bh1792_store_current(struct bh1792_priv *p,
	      const char *buf,
	      size_t len,
	      u8 *curr,
	      u8 mask)
{
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (value > LED_CURRENT_MAX)
		return -EINVAL;

	mutex_lock(&p->conf_lock);

	*curr = value & mask;
	p->config.update = 1;

	if ((p->config.opmode == BH1792_MODE_SINGLE_GREEN) ||
	    (p->config.opmode == BH1792_MODE_SINGLE_IR))
		bh1792_sw_reset(p->client);

	if (p->enabled && (p->config.opmode != BH1792_MODE_SYNCHRONIZED))
		_update_config(p, 0);

	mutex_unlock(&p->conf_lock);

	return len;
}

static ssize_t
bh1792_led1_current_store(struct device *dev,
			  struct device_attribute *attr, const char *buf,
			  size_t len)
{
	struct bh1792_priv *p = dev_get_drvdata(dev);

	return bh1792_store_current(p, buf, len, &(p->config.led1_current),
			     BH1792_MEAS_CONTROL2_LED_CURRENT1_MASK);
}

static struct device_attribute dev_attr_led1_current =
	__ATTR(led1_current,
	       0644,
	       bh1792_led1_current_show,
	       bh1792_led1_current_store);

static ssize_t
bh1792_led2_current_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct bh1792_priv *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", p->config.led2_current);
}

static ssize_t
bh1792_led2_current_store(struct device *dev,
			  struct device_attribute *attr, const char *buf,
			  size_t len)
{
	struct bh1792_priv *p = dev_get_drvdata(dev);

	return bh1792_store_current(p, buf, len, &p->config.led2_current,
			     BH1792_MEAS_CONTROL3_LED_CURRENT2_MASK);
}

static struct device_attribute dev_attr_led2_current =
	__ATTR(led2_current,
	       0644,
	       bh1792_led2_current_show,
	       bh1792_led2_current_store);

static ssize_t
bh1792_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1792_priv *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", sync_freq_tbl[p->config.freq]);
}

static ssize_t
bh1792_freq_store(struct device *dev,
		  struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	struct bh1792_priv *p = dev_get_drvdata(dev);
	u16 freq;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&p->conf_lock);

	if (p->enabled && (p->config.opmode == BH1792_MODE_SYNCHRONIZED)) {
		dev_err(dev,
			"Forbidden to change frequency while measurement running\n");
		mutex_unlock(&p->conf_lock);
		return -EBUSY;
	}

	if (parse_freq(value, &freq) < 0) {
		len = -EINVAL;
	} else {
		p->config.freq = freq;
		p->config.update = 1;
	}

	mutex_unlock(&p->conf_lock);

	return len;
}

static struct device_attribute dev_attr_freq = __ATTR(freq,
						      0644,
						      bh1792_freq_show,
						      bh1792_freq_store);

static ssize_t
bh1792_opmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1792_priv *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", p->config.opmode);
}

static ssize_t
bh1792_opmode_store(struct device *dev,
		    struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	struct bh1792_priv *p = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	switch (value) {
	case BH1792_MODE_SYNCHRONIZED:
	case BH1792_MODE_NON_SYNCHRONIZED:
	case BH1792_MODE_SINGLE_GREEN:
	case BH1792_MODE_SINGLE_IR:
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&p->conf_lock);

	if (p->enabled) {
		dev_err(dev,
			"Forbidden to change mode while measurement running\n");
		mutex_unlock(&p->conf_lock);
		return -EINVAL;
	}

	p->config.opmode = value;

	mutex_unlock(&p->conf_lock);

	return len;
}

static struct device_attribute dev_attr_opmode = __ATTR(operating_mode,
							0644,
							bh1792_opmode_show,
							bh1792_opmode_store);

static ssize_t
bh1792_ir_th_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bh1792_priv *p = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			(p->config.ir_thh << 8) | p->config.ir_thl);
}

static ssize_t
bh1792_ir_th_store(struct device *dev,
		   struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long value;
	struct bh1792_priv *p = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&p->conf_lock);

	p->config.ir_thl = 0xff & value;
	p->config.ir_thh = (value >> 8) & 0xff;
	p->config.update = 1;

	if (p->enabled && (p->config.opmode == BH1792_MODE_NON_SYNCHRONIZED))
		_update_config(p, 0);
	mutex_unlock(&p->conf_lock);
	return len;
}

static struct device_attribute dev_attr_ir_th = __ATTR(threshold,
						       0644,
						       bh1792_ir_th_show,
						       bh1792_ir_th_store);
static struct attribute *bh1792_sysfs_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_led1_current.attr,
	&dev_attr_led2_current.attr,
	&dev_attr_freq.attr,
	&dev_attr_opmode.attr,
	&dev_attr_ir_th.attr,
	NULL
};

static struct attribute_group bh1792_attribute_group = {
	.attrs = bh1792_sysfs_attrs
};

static int
bh1792_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bh1792_priv *p;
	int err;
	unsigned long irq_flags;

	if (!i2c_check_functionality
	    (client->adapter,
	     I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_BYTE_DATA |
	     I2C_FUNC_SMBUS_READ_I2C_BLOCK | I2C_FUNC_SMBUS_WRITE_I2C_BLOCK))
		return -EOPNOTSUPP;

	/* Check manufacturer ID */
	err = i2c_smbus_read_byte_data(client, BH1792_MANUFACTURER_REG);
	if (err < 0)
		return err;
	if (err != BH1792_MANUFACTURER_REG_MANUFACTURER_ID) {
		dev_err(&client->dev, "Manufacturer id doesn't match BH1792\n");
		return -ENODEV;
	}

	/* Check part ID */
	err = i2c_smbus_read_byte_data(client, BH1792_PARTID_REG);
	if (err < 0)
		return err;
	if (err != BH1792_PARTID_REG_PART_ID) {
		dev_err(&client->dev, "no BH1792 sensor\n");
		return -ENODEV;
	}

	dev_info(&client->dev, "ROHM bh1792 detected\n");
	p = devm_kzalloc(&client->dev, sizeof(struct bh1792_priv), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	mutex_init(&p->conf_lock);
	i2c_set_clientdata(client, p);
	p->client = client;

	/* Set default values for config */
	p->config.opmode = 0;
	p->config.led1_current = 1;
	p->config.led2_current = 1;
	p->config.ir_thl = 0xff;
	p->config.ir_thh = 0xff;
	p->config.freq = BH1792_MEAS_CONTROL1_MSR_1024HZ;

	err = bh1792_parse_dt(p, &client->dev);
	if (err) {
		dev_err(&client->dev, "Unable to parse dt data err=%d\n", err);
		return -EINVAL;
	}

	p->input_dev = devm_input_allocate_device(&client->dev);
	if (!p->input_dev)
		return -ENOMEM;
	p->input_dev->name = BH1792_NAME;
	p->input_dev->id.bustype = BUS_I2C;
	p->input_dev->id.vendor = BH1792_MANUFACTURER_REG_MANUFACTURER_ID;
	__set_bit(INPUT_EVENT_TYPE, p->input_dev->evbit);
	__set_bit(INPUT_EVENT_LED, p->input_dev->mscbit);
	set_bit(INPUT_EVENT_TIME_MSB, p->input_dev->mscbit);
	set_bit(INPUT_EVENT_TIME_LSB, p->input_dev->mscbit);
	input_set_events_per_packet(p->input_dev, 128);
	input_set_drvdata(p->input_dev, p);
	p->wq = create_singlethread_workqueue("bh1792_queue");
	if (p->wq == NULL) {
		dev_err(&client->dev, "can't create workqueue\n");
		return -ENOMEM;
	}

	hrtimer_init(&p->sync_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->sync_timer.function = sync_timer_handler;
	init_waitqueue_head(&p->sync_wq);
	p->sync_flag = 0;
	p->sync_task = kthread_run(bh1792_sync_thread, p, "bh1792_sync");
	INIT_WORK(&p->irqwork, bh1792_irq_work);
	err = input_register_device(p->input_dev);
	if (err) {
		dev_err(&client->dev, "input_register_device failed\n");
		goto input_register_device_fail;
	}
	err = sysfs_create_group(&p->input_dev->dev.kobj,
				 &bh1792_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs create fail\n");
		goto sysfs_create_group_fail;
	}

	/* gpio to irq */
	if ((p->gpio_int >= 0) && gpio_is_valid(p->gpio_int)) {
		err = gpio_request(p->gpio_int, "bh1792_int");
		if (err) {
			dev_err(&client->dev,
				"Unable to request interrupt gpio %d\n",
				p->gpio_int);
			goto out;
		}

		err = gpio_direction_input(p->gpio_int);
		if (err) {
			dev_err(&client->dev,
				"Unable to set direction for gpio %d\n",
				p->gpio_int);
			goto out;
		}
		p->irq = gpio_to_irq(p->gpio_int);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	} else {
		p->irq = client->irq;
		irq_flags = irq_get_trigger_type(client->irq);
		if (irq_flags == IRQF_TRIGGER_NONE)
			irq_flags = IRQF_TRIGGER_FALLING;
		irq_flags |= IRQF_ONESHOT;
	}

	err = devm_request_threaded_irq(&client->dev, p->irq, NULL,
					bh1792_irq_handler, irq_flags,
					"bh1792_irq", p);
	if (err) {
		dev_err(&client->dev, "devm_request_threaded_irq failed\n");
		goto out;
	}
	disable_irq(p->irq);

	return 0;
out:
	sysfs_remove_group(&p->input_dev->dev.kobj, &bh1792_attribute_group);
sysfs_create_group_fail:
input_register_device_fail:
	if (p->wq)
		destroy_workqueue(p->wq);
	return err;
}

static int bh1792_remove(struct i2c_client *client)
{
	struct bh1792_priv *p = i2c_get_clientdata(client);

	_stop_measurement(p);
	kthread_stop(p->sync_task);
	if (p->wq)
		destroy_workqueue(p->wq);
	if (p->gpio_int >= 0)
		gpio_free(p->gpio_int);

	sysfs_remove_group(&p->input_dev->dev.kobj, &bh1792_attribute_group);
	return 0;
}

static const struct of_device_id bh1792_of_match[] = {
	{
	 .compatible = "rohm,bh1792",},
	{},
};

static const struct i2c_device_id bh1792_id[] = {

	{
	 BH1792_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, bh1792_id);
static struct i2c_driver bh1792_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = BH1792_NAME,
		   .of_match_table = bh1792_of_match,
		   },
	.probe = bh1792_probe,
	.remove = bh1792_remove,
	.id_table = bh1792_id,
};

module_i2c_driver(bh1792_driver);
MODULE_AUTHOR("Mikko Mutanen <mikko.mutanen@fi.rohmeurope.com>");
MODULE_DESCRIPTION("ROHM bh1792 Heart rate sensor driver");
MODULE_LICENSE("GPL");
