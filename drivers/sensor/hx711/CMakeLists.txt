# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_HX711 hx711.c)

# Add filter sources
zephyr_library_sources_ifdef(CONFIG_HX711_ENABLE_EMA_FILTER filters/ema.c)
zephyr_library_sources_ifdef(CONFIG_HX711_ENABLE_MEDIAN_FILTER filters/median.c)
zephyr_library_sources_ifdef(CONFIG_HX711_ENABLE_SPIKE_FILTER filters/spike.c)
