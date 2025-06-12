/* --- C implementation --- */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "hx711.h"

LOG_MODULE_REGISTER(hx711, CONFIG_SENSOR_LOG_LEVEL);

struct hx711_data {
	const struct device *dout_dev;
	gpio_pin_t dout_pin;
	const struct device *sck_dev;
	gpio_pin_t sck_pin;
	int32_t offset;
	float scale;
	uint8_t gain;
	int32_t raw_reading;
};

struct hx711_config {
	struct gpio_dt_spec dout_gpio;
	struct gpio_dt_spec sck_gpio;
};

static inline void hx711_power_down(const struct hx711_data *data) {
	gpio_pin_set(data->sck_dev, data->sck_pin, 1);
	k_busy_wait(60);
}

static inline void hx711_power_up(const struct hx711_data *data) {
	gpio_pin_set(data->sck_dev, data->sck_pin, 0);
	k_busy_wait(1);
}

static bool hx711_is_ready(const struct hx711_data *data) {
	return gpio_pin_get(data->dout_dev, data->dout_pin) == 0;
}

static int32_t hx711_shift_in(const struct hx711_data *data) {
	int32_t value = 0;
	for (int i = 0; i < 24; i++) {
		gpio_pin_set(data->sck_dev, data->sck_pin, 1);
		k_busy_wait(1);
		value = (value << 1) | gpio_pin_get(data->dout_dev, data->dout_pin);
		gpio_pin_set(data->sck_dev, data->sck_pin, 0);
		k_busy_wait(1);
	}
	for (int i = 0; i < data->gain; i++) {
		gpio_pin_set(data->sck_dev, data->sck_pin, 1);
		k_busy_wait(1);
		gpio_pin_set(data->sck_dev, data->sck_pin, 0);
		k_busy_wait(1);
	}
	if (value & 0x800000) {
		value |= ~0xFFFFFF;
	}
	return value;
}

static int hx711_sample_fetch(const struct device *dev, enum sensor_channel chan) {
	struct hx711_data *data = dev->data;
	int64_t timeout = k_uptime_get() + CONFIG_HX711_SAMPLE_FETCH_TIMEOUT_MS;
	while (!hx711_is_ready(data)) {
		if (k_uptime_get() > timeout) {
			return -EIO;
		}
		k_usleep(10);
	}
	int32_t raw = hx711_shift_in(data);
	data->raw_reading = raw;
	return 0;
}

static int hx711_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
	struct hx711_data *data = dev->data;
	float weight = (data->raw_reading - data->offset) * data->scale;
	val->val1 = (int32_t)weight;
	val->val2 = (int32_t)((weight - val->val1) * 1000000);
	return 0;
}

static const struct sensor_driver_api hx711_api = {
	.sample_fetch = hx711_sample_fetch,
	.channel_get = hx711_channel_get,
};

int32_t hx711_raw_read(const struct device *dev) {
	struct hx711_data *data = dev->data;
	return data->raw_reading;
}

void hx711_tare(const struct device *dev, uint8_t samples) {
	struct hx711_data *data = dev->data;
	int64_t sum = 0;
	for (int i = 0; i < samples; i++) {
		hx711_sample_fetch(dev, SENSOR_CHAN_ALL);
		sum += data->raw_reading;
		k_msleep(5);
	}
	data->offset = sum / samples;
}

void hx711_set_scale(const struct device *dev, float scale) {
	struct hx711_data *data = dev->data;
	data->scale = scale;
}

float hx711_get_units(const struct device *dev, uint8_t samples) {
	struct hx711_data *data = dev->data;
	int64_t sum = 0;
	for (int i = 0; i < samples; i++) {
		hx711_sample_fetch(dev, SENSOR_CHAN_ALL);
		sum += data->raw_reading;
		k_msleep(5);
	}
	float avg = sum / (float)samples;
	return (avg - data->offset) * data->scale;
}

static int hx711_init(const struct device *dev) {
	const struct hx711_config *cfg = dev->config;
	struct hx711_data *data = dev->data;

	if (!device_is_ready(cfg->dout_gpio.port) || !device_is_ready(cfg->sck_gpio.port)) {
		LOG_ERR("GPIO devices not ready");
		return -ENODEV;
	}

	data->dout_dev = cfg->dout_gpio.port;
	data->dout_pin = cfg->dout_gpio.pin;
	data->sck_dev = cfg->sck_gpio.port;
	data->sck_pin = cfg->sck_gpio.pin;
	data->gain = CONFIG_HX711_GAIN;
	data->scale = CONFIG_HX711_SCALE_INTEGER + CONFIG_HX711_SCALE_DECIMAL / 1000000.0f;
	data->offset = 0;

	gpio_pin_configure_dt(&cfg->dout_gpio, GPIO_INPUT);
	gpio_pin_configure_dt(&cfg->sck_gpio, GPIO_OUTPUT);
	gpio_pin_set(cfg->sck_gpio.port, cfg->sck_gpio.pin, 0);

	hx711_power_up(data);
	k_sleep(K_MSEC(100));
	return 0;
}

#define HX711_DEFINE(inst) \
	static struct hx711_data hx711_data_##inst; \
	static const struct hx711_config hx711_config_##inst = { \
		.dout_gpio = GPIO_DT_SPEC_INST_GET(inst, dout_gpios), \
		.sck_gpio = GPIO_DT_SPEC_INST_GET(inst, sck_gpios), \
	}; \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, \
		hx711_init, NULL, \
		&hx711_data_##inst, \
		&hx711_config_##inst, \
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
		&hx711_api);

DT_INST_FOREACH_STATUS_OKAY(HX711_DEFINE)