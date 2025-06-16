#pragma once
//
//  HX711.h (Zephyr port from Rob Tillaart's HX711 Arduino library)
//  PURPOSE: HX711 24-bit ADC driver for load cells (Zephyr RTOS, multi-sensor, shared clock)
//  AUTHOR:  Piotr Poloczek + OpenAI GPT
//  LICENSE: MIT-style or match original Apache 2.0
//

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define HX711_ZEPHYR_VERSION "0.1.0"

#ifdef __cplusplus
extern "C" {
#endif

// === Read modes ===
enum hx711_mode {
	HX711_MODE_AVERAGE = 0x00,
	HX711_MODE_MEDIAN  = 0x01,
	HX711_MODE_MEDAVG  = 0x02,
	HX711_MODE_RUNAVG  = 0x03,
	HX711_MODE_RAW     = 0x04,
};

// === Gain ===
enum hx711_gain {
	HX711_GAIN_128 = 128,
	HX711_GAIN_64  = 64,
	HX711_GAIN_32  = 32,
};

// === HX711 instance context ===
struct hx711_t {
	struct gpio_dt_spec dout;
	const struct gpio_dt_spec *sck_shared; // Pointer to shared clock pin (all instances use same)
	struct k_mutex *sck_mutex;             // Global mutex pointer (shared)
	
	enum hx711_gain gain;
	int32_t offset;
	float scale;
	uint32_t last_read_time;
	float unit_price;
	enum hx711_mode mode;

	bool fast_processor;
};

// === Public API ===
int hx711_init(struct hx711_t *dev,
	       const struct gpio_dt_spec *dout_spec,
	       const struct gpio_dt_spec *sck_shared_spec,
	       struct k_mutex *shared_mutex,
	       bool fast_processor);

bool hx711_is_ready(struct hx711_t *dev);
void hx711_wait_ready(struct hx711_t *dev, uint32_t delay_ms);
bool hx711_wait_ready_retry(struct hx711_t *dev, uint8_t retries, uint32_t delay_ms);
bool hx711_wait_ready_timeout(struct hx711_t *dev, uint32_t timeout_ms, uint32_t delay_ms);

float hx711_read(struct hx711_t *dev);
float hx711_read_average(struct hx711_t *dev, uint8_t times);
float hx711_read_median(struct hx711_t *dev, uint8_t times);
float hx711_read_medavg(struct hx711_t *dev, uint8_t times);
float hx711_read_runavg(struct hx711_t *dev, uint8_t times, float alpha);

void hx711_set_mode(struct hx711_t *dev, enum hx711_mode mode);
enum hx711_mode hx711_get_mode(struct hx711_t *dev);

float hx711_get_value(struct hx711_t *dev, uint8_t times);
float hx711_get_units(struct hx711_t *dev, uint8_t times);

bool hx711_set_gain(struct hx711_t *dev, enum hx711_gain gain, bool force);
enum hx711_gain hx711_get_gain(struct hx711_t *dev);

void hx711_tare(struct hx711_t *dev, uint8_t times);
float hx711_get_tare(struct hx711_t *dev);
bool hx711_tare_set(struct hx711_t *dev);

bool hx711_set_scale(struct hx711_t *dev, float scale);
float hx711_get_scale(struct hx711_t *dev);

void hx711_set_offset(struct hx711_t *dev, int32_t offset);
int32_t hx711_get_offset(struct hx711_t *dev);

void hx711_calibrate_scale(struct hx711_t *dev, float weight, uint8_t times);

void hx711_power_down(struct hx711_t *dev);
void hx711_power_up(struct hx711_t *dev);

uint32_t hx711_last_read(struct hx711_t *dev);

float hx711_get_price(struct hx711_t *dev, uint8_t times);
void  hx711_set_unit_price(struct hx711_t *dev, float price);
float hx711_get_unit_price(struct hx711_t *dev);

#ifdef __cplusplus
}
#endif