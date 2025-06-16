/*
 * Copyright (c) 2020 George Gkinis
 * Copyright (c) 2022 Jan Gnip
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT avia_hx711

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include "hx711.h"

LOG_MODULE_REGISTER(HX711, CONFIG_SENSOR_LOG_LEVEL);

/* Shared clock line mutex (for all instances) */
static struct k_mutex clock_mutex;

/* One-time init for mutex */
static void init_shared_clock_mutex(void)
{
	static bool initialized;
	if (!initialized) {
		k_mutex_init(&clock_mutex);
		initialized = true;
	}
}

/* Instance-specific init function */
static int hx711_init(const struct device *dev)
{
	const struct hx711_config *cfg = dev->config;
	struct hx711_data *data = dev->data;

	init_shared_clock_mutex();

	if (!gpio_is_ready_dt(&cfg->dout_gpio)) {
		LOG_ERR("DOUT GPIO not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->sck_gpio)) {
		LOG_ERR("SCK GPIO not ready");
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&cfg->dout_gpio, GPIO_INPUT) < 0) {
		LOG_ERR("Failed to configure DOUT pin");
		return -EIO;
	}

	/* Only configure clock as output if this is the first device */
	if (dev == DEVICE_DT_GET(DT_INST(0, avia_hx711))) {
		if (gpio_pin_configure_dt(&cfg->sck_gpio, GPIO_OUTPUT_LOW) < 0) {
			LOG_ERR("Failed to configure SCK pin");
			return -EIO;
		}
	}

#if DT_INST_NODE_HAS_PROP(0, rate_gpios)
	if (cfg->rate_gpio.port && gpio_is_ready_dt(&cfg->rate_gpio)) {
		if (gpio_pin_configure_dt(&cfg->rate_gpio, GPIO_OUTPUT_INACTIVE) < 0) {
			LOG_WRN("Failed to configure RATE pin");
		}
	}
#endif

	data->offset = 0;
	data->scale = 1.0f;
	data->gain = cfg->gain;
	data->mode = cfg->mode;
	data->power = HX711_POWER_ON;
	data->last_value = 0;
	data->last_time_read = 0;

#if defined(CONFIG_HX711_ENABLE_MEDIAN_FILTER) || defined(CONFIG_HX711_ENABLE_EMA_FILTER)
	data->reading_unfiltered = 0;
#endif

	LOG_INF("HX711 initialized: %s", dev->name);
	return 0;
}


static void hx711_gpio_callback(const struct device *port,
                                struct gpio_callback *cb,
                                uint32_t pins)
{
    struct hx711_data *data = CONTAINER_OF(cb, struct hx711_data, dout_gpio_cb);
    const struct hx711_config *cfg = data->dev->config;

    gpio_pin_interrupt_configure_dt(&cfg->dout_gpio, GPIO_INT_DISABLE);
    k_sem_give(&data->dout_sem);
}

/* Generate one clock cycle and return DOUT state */
static int hx711_cycle(const struct hx711_data *data)
{
    const struct hx711_config *cfg = data->dev->config;

#ifdef CONFIG_HX711_DISABLE_INTERRUPTS_WHILE_POLLING
    unsigned int key = irq_lock();
#endif

    gpio_pin_set_dt(&cfg->sck_gpio, 1);
    k_busy_wait(1);
    gpio_pin_set_dt(&cfg->sck_gpio, 0);

#ifdef CONFIG_HX711_DISABLE_INTERRUPTS_WHILE_POLLING
    irq_unlock(key);
#endif

    k_busy_wait(1);
    return gpio_pin_get_dt(&cfg->dout_gpio);
}

/* Read and decode 24-bit HX711 value */
static int hx711_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct hx711_data *data = dev->data;
    const struct hx711_config *cfg = dev->config;

    if (data->power != HX711_POWER_ON) {
        return -EACCES;
    }

    if (chan != SENSOR_CHAN_ALL) {
        return -ENOTSUP;
    }

    if (k_sem_take(&data->dout_sem, K_MSEC(CONFIG_HX711_SAMPLE_FETCH_TIMEOUT_MS)) != 0) {
        LOG_ERR("Weight data not ready within %d ms", CONFIG_HX711_SAMPLE_FETCH_TIMEOUT_MS);
        return -EIO;
    }

    k_mutex_lock(&clock_mutex, K_FOREVER);

    /* Read 24 bits */
    int32_t count = 0;
    for (int i = 0; i < 24; i++) {
        count <<= 1;
        if (hx711_cycle(data)) {
            count++;
        }
    }

    /* Set gain for next read */
    for (int i = 0; i < data->gain; i++) {
        hx711_cycle(data);
    }

    k_mutex_unlock(&clock_mutex);

    /* Sign conversion */
    count ^= 0x800000;
    data->reading = count;

#if defined(CONFIG_HX711_ENABLE_MEDIAN_FILTER) || defined(CONFIG_HX711_ENABLE_EMA_FILTER)
    k_mutex_lock(&data->filter_lock, K_FOREVER);
    data->reading_unfiltered = count;
#endif

#ifdef CONFIG_HX711_ENABLE_MEDIAN_FILTER
    data->reading = median_filter_update(&data->median_filter, data->reading);
#endif

#ifdef CONFIG_HX711_ENABLE_EMA_FILTER
    data->reading = ema_filter_update(&data->ema_filter, data->reading);
#endif

#if defined(CONFIG_HX711_ENABLE_MEDIAN_FILTER) || defined(CONFIG_HX711_ENABLE_EMA_FILTER)
    k_mutex_unlock(&data->filter_lock);
#endif

    /* Re-enable interrupt */
    int ret = gpio_pin_interrupt_configure_dt(&cfg->dout_gpio, GPIO_INT_EDGE_TO_INACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to set DOUT GPIO interrupt");
    }

    return ret;
}


/* Set gain via sensor attribute */
static int hx711_attr_set_gain(const struct device *dev, const struct sensor_value *val)
{
	struct hx711_data *data = dev->data;

	switch (val->val1) {
	case HX711_GAIN_128:
	case HX711_GAIN_64:
	case HX711_GAIN_32:
		data->gain = val->val1;
		break;
	default:
		return -ENOTSUP;
	}

	/* Apply immediately by fetching a new sample */
	return hx711_sample_fetch(dev, SENSOR_CHAN_ALL);
}

#if DT_INST_NODE_HAS_PROP(0, rate_gpios)
/* Set sampling rate (10 Hz or 80 Hz) if rate pin is defined */
static int hx711_attr_set_rate(const struct device *dev, const struct sensor_value *val)
{
	struct hx711_data *data = dev->data;
	const struct hx711_config *cfg = dev->config;

	switch (val->val1) {
	case HX711_RATE_10HZ:
	case HX711_RATE_80HZ:
		if (!cfg->rate_gpio.port) {
			LOG_ERR("RATE GPIO not configured");
			return -EINVAL;
		}
		data->rate = val->val1;
		return gpio_pin_set_dt(&cfg->rate_gpio, data->rate == HX711_RATE_80HZ);
	default:
		return -ENOTSUP;
	}
}
#endif

/* Set offset for weight conversion (and reset filters if active) */
static void hx711_attr_set_offset(struct hx711_data *data, const struct sensor_value *offset)
{
#if defined(CONFIG_HX711_ENABLE_MEDIAN_FILTER) || defined(CONFIG_HX711_ENABLE_EMA_FILTER)
	k_mutex_lock(&data->filter_lock, K_FOREVER);
#endif

#ifdef CONFIG_HX711_ENABLE_MEDIAN_FILTER
	median_filter_init(&data->median_filter, offset->val1);
#endif

#ifdef CONFIG_HX711_ENABLE_EMA_FILTER
	ema_filter_reset(&data->ema_filter, (double)offset->val1);
#endif

#if defined(CONFIG_HX711_ENABLE_MEDIAN_FILTER) || defined(CONFIG_HX711_ENABLE_EMA_FILTER)
	k_mutex_unlock(&data->filter_lock);
#endif

	data->offset = offset->val1;
}

/* Set slope (scale factor for conversion) */
static void hx711_attr_set_slope(struct hx711_data *data, const struct sensor_value *slope)
{
	data->slope.val1 = slope->val1;
	data->slope.val2 = slope->val2;
}

/* Generic attribute setter */
static int hx711_attr_set(const struct device *dev,
			  enum sensor_channel chan,
			  enum sensor_attribute attr,
			  const struct sensor_value *val)
{
	struct hx711_data *data = dev->data;
	int ret = 0;

	switch ((int)attr) {

#if DT_INST_NODE_HAS_PROP(0, rate_gpios)
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		ret = hx711_attr_set_rate(dev, val);
		if (ret == 0) {
			LOG_DBG("RATE set to %d Hz", data->rate);
		}
		break;
#endif

	case SENSOR_ATTR_OFFSET:
		hx711_attr_set_offset(data, val);
		LOG_DBG("OFFSET set to %d", data->offset);
		break;

	case HX711_ATTR_SLOPE:
		hx711_attr_set_slope(data, val);
		LOG_DBG("SLOPE set to %d.%d", data->slope.val1, data->slope.val2);
		break;

	case HX711_ATTR_GAIN:
		ret = hx711_attr_set_gain(dev, val);
		if (ret == 0) {
			LOG_DBG("GAIN set to %d", data->gain);
		}
		break;

	default:
		return -ENOTSUP;
	}

	return ret;
}

/* Get converted weight channel reading */
static int hx711_channel_get(const struct device *dev,
			     enum sensor_channel chan,
			     struct sensor_value *val)
{
	struct hx711_data *data = dev->data;

	switch ((int)chan) {
	case HX711_SENSOR_CHAN_WEIGHT: {
		double slope = sensor_value_to_double(&data->slope);
		double result = slope * (data->reading - data->offset);
		sensor_value_from_double(val, result);
		return 0;
	}
	default:
		return -ENOTSUP;
	}
}


static int hx711_init(const struct device *dev)
{
	struct hx711_data *data = dev->data;
	const struct hx711_config *cfg = dev->config;
	int ret;

	LOG_DBG("Initializing HX711: %s", dev->name);

	/* Initialize shared clock mutex (only once globally) */
	init_shared_clock_mutex();

	/* Configure SCK pin */
	ret = gpio_pin_configure_dt(&cfg->sck_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to configure SCK pin");
		return ret;
	}

	/* Configure RATE pin if available */
#if DT_INST_NODE_HAS_PROP(0, rate_gpios)
	if (cfg->rate_gpio.port) {
		ret = gpio_pin_configure_dt(&cfg->rate_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret != 0) {
			LOG_ERR("Failed to configure RATE pin");
			return ret;
		}
		ret = gpio_pin_set_dt(&cfg->rate_gpio, cfg->rate == HX711_RATE_80HZ);
		if (ret != 0) {
			LOG_ERR("Failed to set RATE pin");
			return ret;
		}
	}
#endif

	/* Configure DOUT pin */
	ret = gpio_pin_configure_dt(&cfg->dout_gpio, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Failed to configure DOUT pin");
		return ret;
	}

	/* Initialize filter (optional) */
#if defined(CONFIG_HX711_ENABLE_MEDIAN_FILTER) || defined(CONFIG_HX711_ENABLE_EMA_FILTER)
	k_mutex_init(&data->filter_lock);
#endif

#ifdef CONFIG_HX711_ENABLE_MEDIAN_FILTER
	median_filter_init(&data->median_filter, data->offset);
#endif

#ifdef CONFIG_HX711_ENABLE_EMA_FILTER
	ema_filter_init(&data->ema_filter,
	                CONFIG_HX711_EMA_FILTER_ALPHA_FACTOR,
	                (double)data->offset);
#endif

	/* Initialize semaphore and device pointer */
	k_sem_init(&data->dout_sem, 1, 1);
	data->dev = dev;

	/* Setup GPIO callback */
	gpio_init_callback(&data->dout_gpio_cb, hx711_gpio_callback,
	                   BIT(cfg->dout_gpio.pin));
	ret = gpio_add_callback(cfg->dout_gpio.port, &data->dout_gpio_cb);
	if (ret != 0) {
		LOG_ERR("Failed to add DOUT GPIO callback");
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->dout_gpio, GPIO_INT_EDGE_TO_INACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to enable DOUT interrupt");
		return ret;
	}

	return 0;
}


/**
 * @brief Zero the HX711 by calculating average offset.
 *
 * @param dev       Pointer to the HX711 device instance
 * @param readings  Number of readings to average for tare (recommended: 5–10)
 * @retval          The computed offset
 */
int avia_hx711_tare(const struct device *dev, uint8_t readings)
{
	struct hx711_data *data = dev->data;
	int32_t avg = 0;

	if (readings == 0) {
		readings = 1;
	}

	for (int i = 0; i < readings; i++) {
		if (hx711_sample_fetch(dev, SENSOR_CHAN_ALL) == 0) {
			avg += data->reading;
		}
	}

	LOG_DBG("Tare raw sum: %d", avg);
	avg = avg / readings;
	LOG_DBG("Tare average: %d", avg);

	data->offset = avg;

#if defined(CONFIG_HX711_ENABLE_MEDIAN_FILTER) || defined(CONFIG_HX711_ENABLE_EMA_FILTER)
	k_mutex_lock(&data->filter_lock, K_FOREVER);
#endif

#ifdef CONFIG_HX711_ENABLE_MEDIAN_FILTER
	median_filter_init(&data->median_filter, data->offset);
#endif

#ifdef CONFIG_HX711_ENABLE_EMA_FILTER
	ema_filter_reset(&data->ema_filter, (double)data->offset);
#endif

#if defined(CONFIG_HX711_ENABLE_MEDIAN_FILTER) || defined(CONFIG_HX711_ENABLE_EMA_FILTER)
	k_mutex_unlock(&data->filter_lock);
#endif

	return data->offset;
}


/* Calibrate slope based on known target weight */
struct sensor_value avia_hx711_calibrate(const struct device *dev,
                                         uint32_t target,
                                         uint8_t readings)
{
	struct hx711_data *data = dev->data;
	int32_t avg = 0;

	if (readings == 0) {
		readings = 1;
	}

	for (int i = 0; i < readings; i++) {
		hx711_sample_fetch(dev, SENSOR_CHAN_ALL);
		avg += data->reading;
	}

	avg /= readings;
	LOG_DBG("Average raw value: %d", avg);

	double slope = (double)target / (double)(avg - data->offset);
	sensor_value_from_double(&data->slope, slope);

	LOG_DBG("Calibrated slope: %d.%06d", data->slope.val1, data->slope.val2);
	return data->slope;
}

/* Power management function */
int avia_hx711_power(const struct device *dev, enum hx711_power pow)
{
	struct hx711_data *data = dev->data;
	const struct hx711_config *cfg = dev->config;

	data->power = pow;

	switch (pow) {
	case HX711_POWER_ON:
		gpio_pin_set_dt(&cfg->sck_gpio, 1);
		hx711_sample_fetch(dev, SENSOR_CHAN_ALL);
		return 0;
	case HX711_POWER_OFF:
		gpio_pin_set_dt(&cfg->sck_gpio, 0);
		return 0;
	default:
		return -ENOTSUP;
	}
}

#ifdef CONFIG_PM_DEVICE
/* Device power management callback */
static int hx711_pm_ctrl(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return avia_hx711_power(dev, HX711_POWER_ON);
	case PM_DEVICE_ACTION_TURN_OFF:
		return avia_hx711_power(dev, HX711_POWER_OFF);
	default:
		return -ENOTSUP;
	}
}
#endif

/* Zephyr sensor driver API interface */
static const struct sensor_driver_api hx711_api = {
	.sample_fetch = hx711_sample_fetch,
	.channel_get  = hx711_channel_get,
	.attr_set     = hx711_attr_set,
};

#ifdef CONFIG_PM_DEVICE
#define HX711_PM_DEFINE(inst) \
	PM_DEVICE_DT_DEFINE(DT_DRV_INST(inst), hx711_pm_ctrl);
#else
#define HX711_PM_DEFINE(inst)
#endif

/* Instance generation macro */
#define HX711_DEFINE(inst)                                                 \
	static struct hx711_data hx711_data_##inst;                        \
	static const struct hx711_config hx711_config_##inst = {          \
		.dout_gpio = GPIO_DT_SPEC_INST_GET(inst, dout_gpios),     \
		.sck_gpio  = GPIO_DT_SPEC_INST_GET(inst, sck_gpios),      \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rate_gpios),      \
			(.rate_gpio = GPIO_DT_SPEC_INST_GET(inst, rate_gpios),), ()) \
		.gain = DT_INST_PROP_OR(inst, gain, HX711_GAIN_128),      \
		.mode = DT_INST_PROP_OR(inst, mode, HX711_MODE_AVERAGE),  \
		.rate = DT_INST_PROP_OR(inst, rate, HX711_RATE_10HZ),     \
	};                                                                \
	HX711_PM_DEFINE(inst)                                             \
	DEVICE_DT_INST_DEFINE(inst,                                       \
		hx711_init,                                                   \
		PM_DEVICE_DT_GET(DT_DRV_INST(inst)),                         \
		&hx711_data_##inst, &hx711_config_##inst,                    \
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &hx711_api);

DT_INST_FOREACH_STATUS_OKAY(HX711_DEFINE)
