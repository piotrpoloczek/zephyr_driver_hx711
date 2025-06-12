#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#define HX711_MAX_SENSORS 8

struct hx711_sensor {
    const struct device *dout_gpio;
    gpio_pin_t dout_pin;
    int32_t offset;
    float scale;
    float last_valid;
};

struct hx711_shared {
    const struct device *sck_gpio;
    gpio_pin_t sck_pin;
    uint8_t num_sensors;
    struct hx711_sensor sensors[HX711_MAX_SENSORS];
};

int hx711_shared_init(struct hx711_shared *mgr, const char *sck_gpio_name, gpio_pin_t sck_pin);
int hx711_add_sensor(struct hx711_shared *mgr, uint8_t index, const char *dout_gpio_name, gpio_pin_t dout_pin);
bool hx711_is_ready(const struct hx711_shared *mgr, uint8_t index);
float hx711_read_units(struct hx711_shared *mgr, uint8_t index, uint8_t times);
void hx711_tare(struct hx711_shared *mgr, uint8_t index, uint8_t times);
void hx711_calibrate(struct hx711_shared *mgr, uint8_t index, float weight, uint8_t times);
int32_t hx711_raw_read(const struct device *dev);
void hx711_tare(const struct device *dev, uint8_t samples);
void hx711_set_scale(const struct device *dev, float scale);
float hx711_get_units(const struct device *dev, uint8_t samples);