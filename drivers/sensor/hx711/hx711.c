/*
 * Zephyr-compatible HX711 driver based on Rob Tillaart's Arduino library
 * Supports multiple HX711 sensors sharing one clock line (SCK) and individual data lines (DOUT)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <stdlib.h>

#define HX711_MAX_SAMPLES 15

enum hx711_gain {
	HX711_GAIN_128 = 1,
	HX711_GAIN_32 = 2,
	HX711_GAIN_64 = 3,
};

enum hx711_mode {
	HX711_RAW_MODE,
	HX711_AVERAGE_MODE,
	HX711_MEDIAN_MODE,
	HX711_MEDAVG_MODE,
	HX711_RUNAVG_MODE,
};

struct hx711_data {
	const struct device *dev;
	struct gpio_dt_spec dout;
	const struct gpio_dt_spec *shared_sck;

	enum hx711_gain gain;
	enum hx711_mode mode;
	bool fast_mode;

	int32_t offset;
	float scale;
	int32_t reading;
	uint32_t last_time_read;
};

struct k_mutex hx711_sck_mutex;

static uint8_t hx711_shift_in(const struct gpio_dt_spec *dout,
                              const struct gpio_dt_spec *sck,
                              bool fast)
{
	uint8_t value = 0;
	for (int i = 0; i < 8; ++i) {
		gpio_pin_set_dt(sck, 1);
		if (fast) k_busy_wait(1);

		value <<= 1;
		if (gpio_pin_get_dt(dout)) {
			value |= 1;
		}

		gpio_pin_set_dt(sck, 0);
		if (fast) k_busy_wait(1);
	}
	return value;
}

int32_t hx711_read_raw(struct hx711_data *data)
{
	union {
		int32_t value;
		uint8_t b[4];
	} v = { .value = 0 };

	uint32_t timeout = k_uptime_get() + 1000;
	while (gpio_pin_get_dt(&data->dout) != 0) {
		if (k_uptime_get() > timeout) {
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}

	k_mutex_lock(&hx711_sck_mutex, K_FOREVER);

	v.b[2] = hx711_shift_in(&data->dout, data->shared_sck, data->fast_mode);
	v.b[1] = hx711_shift_in(&data->dout, data->shared_sck, data->fast_mode);
	v.b[0] = hx711_shift_in(&data->dout, data->shared_sck, data->fast_mode);

	uint8_t pulses = 1;
	switch (data->gain) {
	case HX711_GAIN_128: pulses = 1; break;
	case HX711_GAIN_32:  pulses = 2; break;
	case HX711_GAIN_64:  pulses = 3; break;
	}

	for (int i = 0; i < pulses; ++i) {
		gpio_pin_set_dt(data->shared_sck, 1);
		if (data->fast_mode) k_busy_wait(1);
		gpio_pin_set_dt(data->shared_sck, 0);
		if (data->fast_mode) k_busy_wait(1);
	}

	k_mutex_unlock(&hx711_sck_mutex);

	if (v.b[2] & 0x80) v.b[3] = 0xFF;

	data->last_time_read = k_uptime_get();
	return v.value;
}

static void _insert_sort(float *arr, uint8_t size)
{
	for (uint8_t i = 1; i < size; ++i) {
		float key = arr[i];
		int j = i - 1;
		while (j >= 0 && arr[j] > key) {
			arr[j + 1] = arr[j];
			--j;
		}
		arr[j + 1] = key;
	}
}

static float hx711_read_average(struct hx711_data *data, uint8_t times)
{
	if (times < 1) times = 1;
	float sum = 0;
	for (uint8_t i = 0; i < times; ++i) {
		sum += hx711_read_raw(data);
		k_yield();
	}
	return sum / times;
}

static float hx711_read_median(struct hx711_data *data, uint8_t times)
{
	if (times > HX711_MAX_SAMPLES) times = HX711_MAX_SAMPLES;
	if (times < 3) times = 3;

	float samples[HX711_MAX_SAMPLES];
	for (uint8_t i = 0; i < times; ++i) {
		samples[i] = hx711_read_raw(data);
		k_yield();
	}
	_insert_sort(samples, times);

	if (times & 1)
		return samples[times / 2];
	else
		return (samples[times / 2] + samples[times / 2 - 1]) / 2;
}

static float hx711_read_medavg(struct hx711_data *data, uint8_t times)
{
	if (times > HX711_MAX_SAMPLES) times = HX711_MAX_SAMPLES;
	if (times < 3) times = 3;

	float samples[HX711_MAX_SAMPLES];
	for (uint8_t i = 0; i < times; ++i) {
		samples[i] = hx711_read_raw(data);
		k_yield();
	}
	_insert_sort(samples, times);

	float sum = 0;
	uint8_t first = (times + 2) / 4;
	uint8_t last = times - first - 1;
	uint8_t count = 0;

	for (uint8_t i = first; i <= last; ++i) {
		sum += samples[i];
		++count;
	}
	return sum / count;
}

static float hx711_read_runavg(struct hx711_data *data, uint8_t times, float alpha)
{
	if (times < 1) times = 1;
	if (alpha < 0) alpha = 0;
	if (alpha > 1) alpha = 1;

	float val = hx711_read_raw(data);
	for (uint8_t i = 1; i < times; ++i) {
		val += alpha * (hx711_read_raw(data) - val);
		k_yield();
	}
	return val;
}

float hx711_get_value(struct hx711_data *data, uint8_t times)
{
	float raw;
	switch (data->mode) {
	case HX711_RAW_MODE:
		raw = hx711_read_raw(data);
		break;
	case HX711_RUNAVG_MODE:
		raw = hx711_read_runavg(data, times, 0.5f);
		break;
	case HX711_MEDAVG_MODE:
		raw = hx711_read_medavg(data, times);
		break;
	case HX711_MEDIAN_MODE:
		raw = hx711_read_median(data, times);
		break;
	case HX711_AVERAGE_MODE:
	default:
		raw = hx711_read_average(data, times);
		break;
	}
	return raw - data->offset;
}

float hx711_get_units(struct hx711_data *data, uint8_t times)
{
	return hx711_get_value(data, times) * data->scale;
}

bool hx711_set_gain(struct hx711_data *data, enum hx711_gain gain, bool force)
{
	if (!force && data->gain == gain) return true;
	if (gain == HX711_GAIN_128 || gain == HX711_GAIN_64 || gain == HX711_GAIN_32) {
		data->gain = gain;
		hx711_read_raw(data);
		return true;
	}
	return false;
}

enum hx711_gain hx711_get_gain(struct hx711_data *data)
{
	return data->gain;
}

void hx711_tare(struct hx711_data *data, uint8_t times)
{
	data->offset = hx711_read_average(data, times);
}

float hx711_get_tare(struct hx711_data *data)
{
	return -1.0f * data->offset * data->scale;
}

bool hx711_tare_set(struct hx711_data *data)
{
	return data->offset != 0;
}

bool hx711_set_scale(struct hx711_data *data, float scale)
{
	if (scale == 0) return false;
	data->scale = 1.0f / scale;
	return true;
}

float hx711_get_scale(struct hx711_data *data)
{
	return 1.0f / data->scale;
}

void hx711_set_offset(struct hx711_data *data, int32_t offset)
{
	data->offset = offset;
}

int32_t hx711_get_offset(struct hx711_data *data)
{
	return data->offset;
}

void hx711_calibrate_scale(struct hx711_data *data, float weight, uint8_t times)
{
	int32_t avg = hx711_read_average(data, times);
	data->scale = weight / (avg - data->offset);
}

void hx711_power_down(const struct hx711_data *data)
{
	gpio_pin_set_dt(data->shared_sck, 1);
	k_busy_wait(64);
}

void hx711_power_up(const struct hx711_data *data)
{
	gpio_pin_set_dt(data->shared_sck, 0);
}

uint32_t hx711_last_time_read(const struct hx711_data *data)
{
	return data->last_time_read;
}


#define DT_DRV_COMPAT my_hx711

static int hx711_init(const struct device *dev)
{
	struct hx711_data *data = dev->data;

	if (!gpio_is_ready_dt(&data->dout) || !gpio_is_ready_dt(data->shared_sck)) {
		return -ENODEV;
	}

	gpio_pin_configure_dt(&data->dout, GPIO_INPUT);
	gpio_pin_configure_dt(data->shared_sck, GPIO_OUTPUT_INACTIVE);

	k_mutex_init(&hx711_sck_mutex);

	return 0;
}

#define HX711_DEFINE(inst) \
	static struct hx711_data hx711_data_##inst = { \
		.dout = GPIO_DT_SPEC_INST_GET(inst, dout), \
		.shared_sck = &((const struct gpio_dt_spec){ GPIO_DT_SPEC_INST_GET(inst, sck) }), \
		.gain = HX711_GAIN_128, \
		.mode = HX711_AVERAGE_MODE, \
		.scale = 1.0f, \
	}; \
	DEVICE_DT_INST_DEFINE(inst, hx711_init, NULL, \
			      &hx711_data_##inst, NULL, \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(HX711_DEFINE)
