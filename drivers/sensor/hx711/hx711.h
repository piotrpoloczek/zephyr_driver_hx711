#ifndef ZEPHYR_DRIVERS_SENSOR_HX711_H_
#define ZEPHYR_DRIVERS_SENSOR_HX711_H_

#include <zephyr/drivers/sensor.h>

/* Gain settings */
#define HX711_GAIN_128 1
#define HX711_GAIN_64  3
#define HX711_GAIN_32  2

/* Operating modes */
#define HX711_MODE_RAW      0
#define HX711_MODE_AVERAGE  1
#define HX711_MODE_MEDIAN   2
#define HX711_MODE_MEDAVG   3
#define HX711_MODE_RUNAVG   4

/* Attributes */
#define HX711_ATTR_OFFSET   (SENSOR_ATTR_PRIV_START + 0)
#define HX711_ATTR_SCALE    (SENSOR_ATTR_PRIV_START + 1)
#define HX711_ATTR_GAIN     (SENSOR_ATTR_PRIV_START + 2)
#define HX711_ATTR_MODE     (SENSOR_ATTR_PRIV_START + 3)
#define HX711_ATTR_TARE     (SENSOR_ATTR_PRIV_START + 4)

struct hx711_data {
    int32_t offset;
    float scale;
    uint8_t gain;
    uint8_t mode;
    int32_t last_value;
    uint32_t last_time_read;
};

struct hx711_config {
    struct gpio_dt_spec data_gpio;
    struct gpio_dt_spec clk_gpio;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_HX711_H_ */