#include "hx711.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(HX711, CONFIG_SENSOR_LOG_LEVEL);

static int hx711_init(const struct device *dev) {
    const struct hx711_config *cfg = dev->config;
    struct hx711_data *data = dev->data;

    if (!device_is_ready(cfg->data_gpio.port)) {
        LOG_ERR("Data GPIO device not ready");
        return -ENODEV;
    }

    if (!device_is_ready(cfg->clk_gpio.port)) {
        LOG_ERR("Clock GPIO device not ready");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&cfg->data_gpio, GPIO_INPUT);
    gpio_pin_configure_dt(&cfg->clk_gpio, GPIO_OUTPUT_LOW);

    data->offset = 0;
    data->scale = 1.0f;
    data->gain = cfg->gain;
    data->mode = cfg->mode;

    return 0;
}

static bool hx711_is_ready(const struct device *dev) {
    const struct hx711_config *cfg = dev->config;
    return gpio_pin_get_dt(&cfg->data_gpio) == 0;
}

static void hx711_wait_ready(const struct device *dev, uint32_t delay_ms) {
    while (!hx711_is_ready(dev)) {
        k_msleep(delay_ms);
    }
}

static uint8_t hx711_shift_in(const struct device *dev) {
    const struct hx711_config *cfg = dev->config;
    uint8_t value = 0;

    for (uint8_t mask = 0x80; mask > 0; mask >>= 1) {
        gpio_pin_set_dt(&cfg->clk_gpio, 1);
        k_busy_wait(1);
        if (gpio_pin_get_dt(&cfg->data_gpio)) {
            value |= mask;
        }
        gpio_pin_set_dt(&cfg->clk_gpio, 0);
        k_busy_wait(1);
    }
    return value;
}

static int32_t hx711_read_raw(const struct device *dev) {
    const struct hx711_config *cfg = dev->config;
    struct hx711_data *data = dev->data;
    union {
        int32_t value;
        uint8_t data[4];
    } result = {0};

    /* Wait until data is ready */
    while (gpio_pin_get_dt(&cfg->data_gpio) == 1) {
        k_yield();
    }

    /* Critical section - disable interrupts */
    unsigned int key = irq_lock();

    /* Read 24-bit data */
    result.data[2] = hx711_shift_in(dev);
    result.data[1] = hx711_shift_in(dev);
    result.data[0] = hx711_shift_in(dev);

    /* Apply gain setting */
    for (uint8_t i = 0; i < data->gain; i++) {
        gpio_pin_set_dt(&cfg->clk_gpio, 1);
        k_busy_wait(1);
        gpio_pin_set_dt(&cfg->clk_gpio, 0);
        k_busy_wait(1);
    }

    irq_unlock(key);

    /* Sign extend 24-bit to 32-bit */
    if (result.data[2] & 0x80) {
        result.data[3] = 0xFF;
    }

    data->last_time_read = k_uptime_get_32();
    return result.value;
}

static float hx711_read_average(const struct device *dev, uint8_t times) {
    float sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += hx711_read_raw(dev);
        k_yield();
    }
    return sum / times;
}

static int hx711_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct hx711_data *data = dev->data;

    if (chan != SENSOR_CHAN_ALL) {
        return -ENOTSUP;
    }

    switch (data->mode) {
        case HX711_MODE_RAW:
            data->last_value = hx711_read_raw(dev);
            break;
        case HX711_MODE_AVERAGE:
            data->last_value = hx711_read_average(dev, 5);
            break;
        /* Other modes can be added here */
        default:
            return -ENOTSUP;
    }
    return 0;
}

static int hx711_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    struct hx711_data *data = dev->data;

    if (chan == SENSOR_CHAN_MASS) {
        float units = (data->last_value - data->offset) * data->scale;
        val->val1 = (int32_t)units;
        val->val2 = (units - val->val1) * 1000000;
        return 0;
    }
    return -ENOTSUP;
}

static int hx711_attr_set(const struct device *dev, enum sensor_channel chan,
                          enum sensor_attribute attr, const struct sensor_value *val) {
    struct hx711_data *data = dev->data;

    switch (attr) {
        case HX711_ATTR_OFFSET:
            data->offset = val->val1;
            break;
        case HX711_ATTR_SCALE:
            data->scale = sensor_value_to_float(val);
            break;
        case HX711_ATTR_GAIN:
            data->gain = val->val1;
            break;
        case HX711_ATTR_MODE:
            data->mode = val->val1;
            break;
        case HX711_ATTR_TARE:
            data->offset = hx711_read_average(dev, 10);
            break;
        default:
            return -ENOTSUP;
    }
    return 0;
}

static const struct sensor_driver_api hx711_api = {
    .sample_fetch = hx711_sample_fetch,
    .channel_get = hx711_channel_get,
    .attr_set = hx711_attr_set,
};

#define HX711_DEFINE(inst) \
    static struct hx711_data hx711_data_##inst; \
    static const struct hx711_config hx711_config_##inst = { \
        .data_gpio = GPIO_DT_SPEC_INST_GET(inst, data_gpios), \
        .clk_gpio = GPIO_DT_SPEC_INST_GET(inst, clk_gpios), \
        .gain = DT_INST_PROP(inst, gain), \
        .mode = DT_INST_PROP(inst, mode), \
    }; \
    DEVICE_DT_INST_DEFINE(inst, hx711_init, NULL, \
                  &hx711_data_##inst, &hx711_config_##inst, \
                  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
                  &hx711_api);

DT_INST_FOREACH_STATUS_OKAY(HX711_DEFINE)