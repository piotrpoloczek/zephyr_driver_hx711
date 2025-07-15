#include "spike.h"
#include <stdlib.h>

void spike_filter_init(spike_filter_state_t *filter)
{
	filter->last_in_range = true;
	filter->just_rejected_spike = false;
}

bool spike_filter_update(spike_filter_state_t *filter, int32_t weight_grams, int32_t threshold_grams)
{
	bool is_outlier = abs(weight_grams) > threshold_grams;

	// Detect transition into spike
	if (is_outlier && filter->last_in_range) {
		filter->just_rejected_spike = true;
		filter->last_in_range = false;
		return false; // Reject the spike
	}

	// Update state
	filter->last_in_range = !is_outlier;
	filter->just_rejected_spike = false;

	return true; // Accept the reading
} 