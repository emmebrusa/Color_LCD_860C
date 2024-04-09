#pragma once

#include <stdint.h>

void battery_voltage_init(void);

#if defined(DISPLAY_860C) || defined(DISPLAY_860C_V12) || defined(DISPLAY_860C_V13)
uint16_t adc_light_sensor_get();
#elif defined(DISPLAY_850C) || defined(DISPLAY_850C_2021) || defined(SW102)
uint16_t battery_voltage_10x_get();
#endif