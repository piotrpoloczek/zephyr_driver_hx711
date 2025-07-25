/*
 * Copyright (c) 2020 George Gkinis
 * Copyright (c) 2022 Jan Gnip
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT avia_hx711

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/pm/device.h>

#include "hx711.h"

#define SAMPLE_FETCH_TIMEOUT_MS 600

LOG_MODULE_REGISTER(HX711, CONFIG_SENSOR_LOG_LEVEL);

static void hx711_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct hx711_data *data = CONTAINER_OF(cb, struct hx711_data, dout_gpio_cb);
	const struct hx711_config *cfg = data->dev->config;

	gpio_pin_interrupt_configure(data->dout_gpio, cfg->dout_pin, GPIO_INT_DISABLE);

	/* Signal thread that data is now ready */
	k_sem_give(&data->dout_sem);
}

/**
 * @brief Send a pulse on the SCK pin.
 *
 * @param data Pointer to the hx711 driver data structure
 *
 * @retval The value of the DOUT pin (HIGH or LOW)
 *
 */
static int hx711_cycle(struct hx711_data *data)
{
	const struct hx711_config *cfg = data->dev->config;

	/* SCK set HIGH */
	gpio_pin_set(data->sck_gpio, cfg->sck_pin, true);
	k_busy_wait(1);

	/* SCK set LOW */
	gpio_pin_set(data->sck_gpio, cfg->sck_pin, false);
	k_busy_wait(1);

	/* Return DOUT pin state */
	return gpio_pin_get(data->dout_gpio, cfg->dout_pin);
}

/**
 * @brief Read HX711 data. Also sets GAIN for the next cycle.
 *
 * @param dev Pointer to the hx711 device structure
 * @param chan Channel to fetch data for.
 *             Only HX711_SENSOR_CHAN_WEIGHT is available.
 *
 * @retval 0 on success,
 * @retval -EACCES error if module is not powered up.
 * @retval -EIO error if SAMPLE_FETCH_TIMEOUT_MS elapsed with no data available.
 *
 */
static int hx711_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret = 0;
	int interrupt_cfg_ret;
	uint32_t count = 0;
	int i;

	struct hx711_data *data = dev->data;
	const struct hx711_config *cfg = dev->config;

	if (data->power != HX711_POWER_ON)
	{
		return -EACCES;
	}

	if (k_sem_take(&data->dout_sem, K_MSEC(SAMPLE_FETCH_TIMEOUT_MS)))
	{
		LOG_ERR("Weight data not ready within %d ms", SAMPLE_FETCH_TIMEOUT_MS);
		ret = -EIO;
		goto exit;
	}

	/* Clock data out. Optionally disable interrupts */
#ifdef CONFIG_HX711_DISABLE_INTERRUPTS_WHILE_POLLING
	uint32_t key = irq_lock();
#endif
	for (i = 0; i < 24; i++)
	{
		count = count << 1;
		if (hx711_cycle(data))
		{
			count++;
		}
	}

	/* set GAIN for next read */
	for (i = 0; i < data->gain; i++)
	{
		hx711_cycle(data);
	}

#ifdef CONFIG_HX711_DISABLE_INTERRUPTS_WHILE_POLLING
	irq_unlock(key);
#endif

	// count ^= 0x800000;
	count = (int32_t)(count << 8) >> 8;


	data->reading = count;

	/* Apply filters if enabled */
	data->filtered_reading = data->reading;

#ifdef CONFIG_HX711_ENABLE_MEDIAN_FILTER
	data->filtered_reading = median_filter_update(&data->median_filter, data->filtered_reading);
#endif
#ifdef CONFIG_HX711_ENABLE_EMA_FILTER
	data->filtered_reading = ema_filter_update(&data->ema_filter, data->filtered_reading);
#endif
#ifdef CONFIG_HX711_ENABLE_SPIKE_FILTER
	if (!spike_filter_update(&data->spike_filter, data->filtered_reading, CONFIG_HX711_SPIKE_FILTER_THRESHOLD)) {
		data->spike_rejected = true;
		data->filtered_reading = data->last_good_filtered_reading;
	} else {
		data->spike_rejected = false;
		data->last_good_filtered_reading = data->filtered_reading;
	}
#endif

exit:
	interrupt_cfg_ret = gpio_pin_interrupt_configure(data->dout_gpio, cfg->dout_pin, GPIO_INT_EDGE_TO_INACTIVE);
	if (interrupt_cfg_ret != 0)
	{
		LOG_ERR("Failed to set dout GPIO interrupt");
		ret = interrupt_cfg_ret;
	}

	return ret;
}

/**
 * @brief Set HX711 gain.
 *
 * @param dev Pointer to the hx711 device structure
 * @param val sensor_value struct. Only val1 is used.
 *            valid values are :
 *                HX711_GAIN_128X (default),
 *                HX711_GAIN_32X,
 *                HX711_GAIN_64X,
 *
 * @retval 0 on success,
 * @retval -ENOTSUP error if an invalid GAIN is provided or
 *         -EACCES if 600000 usec's are elapsed with no data available.
 *
 */
static int hx711_attr_set_gain(const struct device *dev, const struct sensor_value *val)
{
	struct hx711_data *data = dev->data;

	switch (val->val1)
	{
	case HX711_GAIN_128X:
		data->gain = HX711_GAIN_128X;
		break;
	case HX711_GAIN_32X:
		data->gain = HX711_GAIN_32X;
		break;
	case HX711_GAIN_64X:
		data->gain = HX711_GAIN_64X;
		break;
	default:
		return -ENOTSUP;
	}
	return hx711_sample_fetch(dev, HX711_SENSOR_CHAN_WEIGHT);
}

/**
 * @brief Set HX711 rate.
 *
 * @param data Pointer to the hx711 driver data structure
 * @param val sensor_value struct. Only val1 is used.
 *            valid values are :
 *               HX711_RATE_10HZ (default),
 *               HX711_RATE_80HZ
 *
 * @retval 0 on success,
 * @retval -EINVAL error if it fails to get RATE device.
 * @retval -ENOTSUP error if an invalid rate value is passed.
 *
 */
static int hx711_attr_set_rate(struct hx711_data *data, const struct sensor_value *val)
{
	int ret;
	const struct hx711_config *cfg = data->dev->config;


	switch (val->val1)
	{
	case HX711_RATE_10HZ:
	case HX711_RATE_80HZ:
		if (data->rate_gpio == NULL)
		{
			LOG_ERR("Failed to get pointer to RATE device");
			return -EINVAL;
		}
		data->rate = val->val1;
		ret = gpio_pin_set(data->rate_gpio, cfg->rate_pin, data->rate);
		return ret;
	default:
		return -ENOTSUP;
	}
}

/**
 * @brief Set HX711 offset.
 *
 * @param data Pointer to the hx711 driver data structure
 * @param offset sensor_value struct. Offset used to calculate the real weight.
 *        Formula is :
 *          weight = slope * (reading - offset)
 *          Only val1 is used.
 *
 * @return void
 *
 */
static void hx711_attr_set_offset(struct hx711_data *data, const struct sensor_value *offset)
{
	data->offset = offset->val1;
}

/**
 * @brief Set HX711 slope.
 *
 * @param data Pointer to the hx711 driver data structure
 * @param slope sensor_value struct. Slope used to calculate the real weight.
 *        Formula is :
 *          weight = slope * (reading - offset)
 *
 * @return void
 *
 */
static void hx711_attr_set_slope(struct hx711_data *data, const struct sensor_value *slope)
{
	data->slope.val1 = slope->val1;
	data->slope.val2 = slope->val2;
}

/**
 * @brief Set HX711 attributes.
 *
 * @param dev Pointer to the hx711 device structure
 * @param chan Channel to set.
 *             Supported channels :
 *               HX711_SENSOR_CHAN_WEIGHT
 * @param attr Attribute to change.
 *             Supported attributes :
 *               SENSOR_ATTR_SAMPLING_FREQUENCY
 *               SENSOR_ATTR_OFFSET
 *               HX711_SENSOR_ATTR_SLOPE
 *               HX711_SENSOR_ATTR_GAIN
 * @param val   Value to set.
 * @retval 0 on success
 * @retval -ENOTSUP if an invalid attribute is given
 *
 */
static int hx711_attr_set(const struct device *dev, enum sensor_channel chan,
						  enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret = 0;
	int hx711_attr = (int)attr;
	struct hx711_data *data = dev->data;

	switch (hx711_attr)
	{
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		ret = hx711_attr_set_rate(data, val);
		if (ret == 0)
		{
			LOG_DBG("Attribute RATE set to %d\n", data->rate);
		}
		return ret;
	case SENSOR_ATTR_OFFSET:
		hx711_attr_set_offset(data, val);
		LOG_DBG("Attribute OFFSET set to %d\n", data->offset);
		return ret;
	case HX711_SENSOR_ATTR_SLOPE:
		hx711_attr_set_slope(data, val);
		LOG_DBG("Attribute SLOPE set to %d.%d\n", data->slope.val1, data->slope.val2);
		return ret;
	case HX711_SENSOR_ATTR_GAIN:
		ret = hx711_attr_set_gain(dev, val);
		if (ret == 0)
		{
			LOG_DBG("Attribute GAIN set to %d\n", data->gain);
		}
		return ret;
	default:
		return -ENOTSUP;
	}
}

/**
 * @brief Get HX711 reading.
 *
 * @param dev Pointer to the hx711 device structure
 * @param chan Channel to set.
 *             Supported channels :
 *               HX711_SENSOR_CHAN_WEIGHT
 *
 * @param val  Value to write weight value to.
 *        Formula is :
 *          weight = slope * (reading - offset)
 *
 * @retval 0 on success
 * @retval  -ENOTSUP if an invalid channel is given
 *
 */
static int hx711_channel_get(const struct device *dev, enum sensor_channel chan,
							 struct sensor_value *val)
{
	enum hx711_channel hx711_chan = (enum hx711_channel)chan;
	struct hx711_data *data = dev->data;

	switch (hx711_chan)
	{
	case HX711_SENSOR_CHAN_WEIGHT:
	{
		val->val1 = sensor_value_to_double(&data->slope) * (data->filtered_reading - data->offset);
		return 0;
	}
	default:
		return -ENOTSUP;
	}
}

/**
 * @brief Initialise HX711.
 *
 * @param dev Pointer to the hx711 device structure
 *
 * @retval 0 on success
 * @retval -EINVAL if an invalid argument is given
 *
 */
static int hx711_init(const struct device *dev)
{
	LOG_DBG("Initializing HX711");

	int ret = 0;
	struct hx711_data *data = dev->data;
	const struct hx711_config *cfg = dev->config;

	/* Check GPIO devices */
	if (!device_is_ready(cfg->sck_ctrl)) {
		LOG_ERR("SCK GPIO controller %s not ready", cfg->sck_ctrl->name);
		return -ENODEV;
	}
	if (!device_is_ready(cfg->dout_ctrl)) {
		LOG_ERR("DOUT GPIO controller %s not ready", cfg->dout_ctrl->name);
		return -ENODEV;
	}
// #if DT_NODE_HAS_PROP(DT_DRV_INST(0), rate_gpios)
// 	// if (!device_is_ready(cfg->rate_ctrl)) {
// 	// 	LOG_ERR("RATE GPIO controller %s not ready", cfg->rate_ctrl->name);
// 	// 	return -ENODEV;
// 	// }
// 	if (cfg->rate_ctrl != NULL) {
// 	if (!device_is_ready(cfg->rate_ctrl)) {
// 		LOG_ERR("RATE GPIO controller %s not ready", cfg->rate_ctrl->name);
// 		return -ENODEV;
// 	}
// }
// #endif
	if (cfg->rate_ctrl != NULL) {
		if (!device_is_ready(cfg->rate_ctrl)) {
			LOG_ERR("RATE GPIO controller %s not ready", cfg->rate_ctrl->name);
			return -ENODEV;
		}
	}

	LOG_DBG("SCK: %s pin %d", cfg->sck_ctrl->name, cfg->sck_pin);
	LOG_DBG("DOUT: %s pin %d", cfg->dout_ctrl->name, cfg->dout_pin);
#if DT_NODE_HAS_PROP(DT_DRV_INST(0), rate_gpios)
	LOG_DBG("RATE: %s pin %d", cfg->rate_ctrl->name, cfg->rate_pin);
#endif
	LOG_DBG("Gain: %d", data->gain);
	LOG_DBG("Offset: %d", data->offset);
	LOG_DBG("Slope: %d.%d", data->slope.val1, data->slope.val2);

	/* Configure SCK as output, LOW */
	data->sck_gpio = cfg->sck_ctrl;
	ret = gpio_pin_configure(data->sck_gpio, cfg->sck_pin,
							 GPIO_OUTPUT_INACTIVE | cfg->sck_flags);
	if (ret != 0) {
		LOG_ERR("Failed to configure SCK pin: %d", ret);
		return ret;
	}

	/* Configure RATE as output, LOW (if defined) */
// #if DT_NODE_HAS_PROP(DT_DRV_INST(0), rate_gpios)
// 	data->rate_gpio = cfg->rate_ctrl;
// 	ret = gpio_pin_configure(data->rate_gpio, cfg->rate_pin,
// 							 GPIO_OUTPUT_INACTIVE | cfg->rate_flags);
// 	if (ret != 0) {
// 		LOG_ERR("Failed to configure RATE pin: %d", ret);
// 		return ret;
// 	}
// 	ret = gpio_pin_set(data->rate_gpio, cfg->rate_pin, CONFIG_HX711_SAMPLING_RATE);
// 	if (ret != 0) {
// 		LOG_ERR("Failed to set RATE pin: %d", ret);
// 		return ret;
// 	}
// #endif

	if (cfg->rate_ctrl != NULL) {
		data->rate_gpio = cfg->rate_ctrl;
		ret = gpio_pin_configure(data->rate_gpio, cfg->rate_pin,
								GPIO_OUTPUT_INACTIVE | cfg->rate_flags);
		if (ret != 0) {
			LOG_ERR("Failed to configure RATE pin: %d", ret);
			return ret;
		}
		ret = gpio_pin_set(data->rate_gpio, cfg->rate_pin, CONFIG_HX711_SAMPLING_RATE);
		if (ret != 0) {
			LOG_ERR("Failed to set RATE pin: %d", ret);
			return ret;
		}
	}

	/* Configure DOUT as input */
	data->dout_gpio = cfg->dout_ctrl;
	ret = gpio_pin_configure(data->dout_gpio, cfg->dout_pin,
							 GPIO_INPUT | cfg->dout_flags);
	if (ret != 0) {
		LOG_ERR("Failed to configure DOUT pin: %d", ret);
		return ret;
	}

	/* Set up interrupt and callback */
	k_sem_init(&data->dout_sem, 1, 1);
	gpio_init_callback(&data->dout_gpio_cb, hx711_gpio_callback, BIT(cfg->dout_pin));

	ret = gpio_add_callback(data->dout_gpio, &data->dout_gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add GPIO callback: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure(data->dout_gpio, cfg->dout_pin,
									   GPIO_INT_EDGE_TO_INACTIVE);
	if (ret != 0) {
		LOG_ERR("Failed to configure DOUT interrupt: %d", ret);
		return ret;
	}

	data->dev = dev;

	/* Initialize filters if enabled */
#ifdef CONFIG_HX711_ENABLE_EMA_FILTER
	ema_filter_init(&data->ema_filter, CONFIG_HX711_EMA_FILTER_ALPHA_FACTOR, 0.0);
	LOG_DBG("EMA filter initialized with alpha: %d", CONFIG_HX711_EMA_FILTER_ALPHA_FACTOR);
#endif
#ifdef CONFIG_HX711_ENABLE_MEDIAN_FILTER
	median_filter_init(&data->median_filter, 0);
	LOG_DBG("Median filter initialized with window size: %d", CONFIG_HX711_MEDIAN_FILTER_WINDOW_SIZE);
#endif
#ifdef CONFIG_HX711_ENABLE_SPIKE_FILTER
	spike_filter_init(&data->spike_filter);
	LOG_DBG("Spike filter initialized with threshold: %d grams", CONFIG_HX711_SPIKE_FILTER_THRESHOLD);
#endif

	LOG_DBG("HX711 init complete for device %s", dev->name);
	return 0;
}



/**
 * @brief Zero the HX711.
 *
 * @param dev Pointer to the hx711 device structure
 * @param readings Number of readings to get average offset.
 *        5~10 readings should be enough, although more are allowed.
 * @retval The offset value
 *
 */
int avia_hx711_tare(const struct device *dev, uint8_t readings)
{
	int32_t avg = 0;
	struct hx711_data *data = dev->data;

	if (readings == 0)
	{
		readings = 1;
	}

	for (int i = 0; i < readings; i++)
	{
		hx711_sample_fetch(dev, HX711_SENSOR_CHAN_WEIGHT);
		LOG_DBG("[%s] Sample %d: %d", dev->name, i + 1, data->reading);
		avg += data->reading;
	}
	LOG_DBG("Average before division : %d", avg);
	avg = avg / readings;
	LOG_DBG("Average after division : %d", avg);
	data->offset = avg;

	return data->offset;
}

/**
 * @brief Callibrate the HX711.
 *
 * Given a target value of a known weight the slope gets calculated.
 * This is actually unit agnostic.
 * If the target weight is given in grams, lb, Kg or any other weight unit,
 * the slope will be calculated accordingly.
 *
 * @param dev Pointer to the hx711 device structure
 * @param target Target weight in grams.
 *        If target is represented in another unit (lb, oz, Kg) then the
 *        value returned by sensor_channel_get() will represent that unit.
 * @param readings Number of readings to take for calibration.
 *        5~10 readings should be enough, although more are allowed.
 * @retval The slope value
 *
 */
struct sensor_value avia_hx711_calibrate(const struct device *dev, uint32_t target,
										 uint8_t readings)
{
	int32_t avg = 0;
	struct hx711_data *data = dev->data;

	if (readings == 0)
	{
		readings = 1;
	}

	for (int i = 0; i < readings; i++)
	{
		hx711_sample_fetch(dev, HX711_SENSOR_CHAN_WEIGHT);
		avg += data->reading;
	}
	LOG_DBG("Average before division : %d", avg);
	avg = avg / readings;

	LOG_DBG("Average after division : %d", avg);
	double slope = (double)target / (double)(avg - data->offset);

	data->slope.val1 = (int)slope;
	data->slope.val2 = (slope - data->slope.val1) * 1e6;

	LOG_DBG("Slope set to : %d.%06d", data->slope.val1, data->slope.val2);

	return data->slope;
}

/**
 * @brief Set the HX711 power.
 *
 * @param dev Pointer to the hx711 device structure
 * @param power one of HX711_POWER_OFF, HX711_POWER_ON
 * @retval The current power state or ENOTSUP if an invalid value pow is given
 *
 */
int avia_hx711_power(const struct device *dev, enum hx711_power pow)
{
	int ret;
	struct hx711_data *data = dev->data;
	const struct hx711_config *cfg = data->dev->config;


	data->power = pow;
	switch (pow)
	{
	case HX711_POWER_ON:
		ret = gpio_pin_set(data->sck_gpio, cfg->sck_pin, data->power);
		/* Fetch a sample to set GAIN again.
		 * GAIN is set to 128 channel A after RESET
		 */
		hx711_sample_fetch(dev, HX711_SENSOR_CHAN_WEIGHT);
		return ret;
	case HX711_POWER_OFF:
		ret = gpio_pin_set(data->sck_gpio, cfg->sck_pin, data->power);
		return ret;
	default:
		return -ENOTSUP;
	}
}

#ifdef CONFIG_PM_DEVICE
/**
 * @brief Set the Device Power Management State.
 *
 * @param dev - The device structure.
 * @param action - power management state
 * @retval 0 on success
 * @retval -ENOTSUP if an unsupported action is given
 *
 */
int hx711_pm_ctrl(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;

	switch (action)
	{
	case PM_DEVICE_ACTION_RESUME:
		ret = avia_hx711_power(dev, HX711_POWER_ON);
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		ret = avia_hx711_power(dev, HX711_POWER_OFF);
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static const struct sensor_driver_api hx711_api = {
	.sample_fetch = hx711_sample_fetch,
	.channel_get = hx711_channel_get,
	.attr_set = hx711_attr_set,
};

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define HX711_DEFINE(inst)                                                       \
	static struct hx711_data hx711_data_##inst = {                               \
		.reading = 0,                                                            \
		.offset = CONFIG_HX711_OFFSET,                                           \
		.slope = {                                                               \
			.val1 = CONFIG_HX711_SLOPE_INTEGER,                                  \
			.val2 = CONFIG_HX711_SLOPE_DECIMAL,                                  \
		},                                                                       \
		.gain = CONFIG_HX711_GAIN,                                               \
		.rate = CONFIG_HX711_SAMPLING_RATE,                                      \
		.power = HX711_POWER_ON,                                                 \
		.last_good_filtered_reading = 0,                                         \
		.spike_rejected = false,                                                 \
	};                                                                           \
                                                                                 \
	static const struct hx711_config hx711_config_##inst = {                     \
		.dout_pin = DT_INST_GPIO_PIN(inst, dout_gpios),                          \
		.dout_ctrl = DEVICE_DT_GET(DT_GPIO_CTLR(DT_DRV_INST(inst), dout_gpios)), \
		.dout_flags = DT_INST_GPIO_FLAGS(inst, dout_gpios),                      \
                                                                                 \
		.sck_pin = DT_INST_GPIO_PIN(inst, sck_gpios),                            \
		.sck_ctrl = DEVICE_DT_GET(DT_GPIO_CTLR(DT_DRV_INST(inst), sck_gpios)),   \
		.sck_flags = DT_INST_GPIO_FLAGS(inst, sck_gpios),                        \
	};                                                                           \
                                                                                 \
	PM_DEVICE_DT_INST_DEFINE(inst, hx711_pm_ctrl);                               \
                                                                                 \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, hx711_init, PM_DEVICE_DT_INST_GET(inst),  \
								 &hx711_data_##inst, &hx711_config_##inst,       \
								 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,       \
								 &hx711_api);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(HX711_DEFINE)
