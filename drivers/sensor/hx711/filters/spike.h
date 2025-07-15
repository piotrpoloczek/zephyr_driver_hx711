#ifndef ZEPHYR_DRIVERS_SENSOR_HX711_SPIKE_FILTER_H_
#define ZEPHYR_DRIVERS_SENSOR_HX711_SPIKE_FILTER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	bool last_in_range;
	bool just_rejected_spike;
} spike_filter_state_t;

void spike_filter_init(spike_filter_state_t *filter);
bool spike_filter_update(spike_filter_state_t *filter, int32_t weight_grams, int32_t threshold_grams);

#endif /* ZEPHYR_DRIVERS_SENSOR_HX711_SPIKE_FILTER_H_ */ 