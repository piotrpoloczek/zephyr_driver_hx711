#ifndef ZEPHYR_DRIVERS_SENSOR_HX711_H_
#define ZEPHYR_DRIVERS_SENSOR_HX711_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_HX711_ENABLE_MEDIAN_FILTER)
#include "median_filter.h"
#endif

#if defined(CONFIG_HX711_ENABLE_EMA_FILTER)
#include "ema_filter.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Gain options */
#define HX711_GAIN_128 1
#define HX711_GAIN_32  2
#define HX711_GAIN_64  3

/* Rate options */
#define HX711_RATE_10HZ  0
#define HX711_RATE_80HZ  1

/* Power states */
enum hx711_power {
	HX711_POWER_OFF = 0,
	HX711_POWER_ON  = 1,
};

/* Mode (user-defined or app-specific) */
#define HX711_MODE_RAW      0
#define HX711_MODE_AVERAGE  1
#define HX711_MODE_HS_RAW   5

/* Custom attributes */
#define HX711_ATTR_OFFSET   (SENSOR_ATTR_PRIV_START + 0)
#define HX711_ATTR_SCALE    (SENSOR_ATTR_PRIV_START + 1)
#define HX711_ATTR_GAIN     (SENSOR_ATTR_PRIV_START + 2)
#define HX711_ATTR_MODE     (SENSOR_ATTR_PRIV_START + 3)
#define HX711_ATTR_SLOPE    (SENSOR_ATTR_PRIV_START + 4)

/* Custom channel (mass output) */
#define HX711_SENSOR_CHAN_WEIGHT SENSOR_CHAN_PRIV_START

struct hx711_data {
	const struct device *dev;

	int32_t reading;
	int32_t offset;
	float scale;

	struct sensor_value slope;
	uint8_t gain;
	uint8_t mode;
	uint8_t rate;
	uint8_t power;

	int32_t last_value;
	uint32_t last_time_read;

	struct gpio_callback dout_gpio_cb;
	struct k_sem dout_sem;

#if defined(CONFIG_HX711_ENABLE_MEDIAN_FILTER) || defined(CONFIG_HX711_ENABLE_EMA_FILTER)
	int32_t reading_unfiltered;
	struct k_mutex filter_lock;
#endif

#ifdef CONFIG_HX711_ENABLE_MEDIAN_FILTER
	struct median_filter median_filter;
#endif

#ifdef CONFIG_HX711_ENABLE_EMA_FILTER
	struct ema_filter ema_filter;
#endif
};

struct hx711_config {
	struct gpio_dt_spec dout_gpio;
	struct gpio_dt_spec sck_gpio;
#if DT_INST_NODE_HAS_PROP(0, rate_gpios)
	struct gpio_dt_spec rate_gpio;
#endif
	uint8_t gain;
	uint8_t mode;
	uint8_t rate;
};

/* API functions */
int avia_hx711_tare(const struct device *dev, uint8_t readings);
struct sensor_value avia_hx711_calibrate(const struct device *dev, uint32_t target, uint8_t readings);
int avia_hx711_power(const struct device *dev, enum hx711_power power);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_HX711_H_ */
