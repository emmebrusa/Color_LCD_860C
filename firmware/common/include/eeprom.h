/*
 * Bafang LCD 850C firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "lcd.h"
#include "state.h"
#include "screen.h"

// For compatible changes, just add new fields at the end of the table (they will be inited to 0xff for old eeprom images).
// For incompatible changes bump up EEPROM_MIN_COMPAT_VERSION and the user's EEPROM settings will be discarded.
#define EEPROM_MIN_COMPAT_VERSION 0x40
#define EEPROM_VERSION 0x40

typedef struct {
  graph_auto_max_min_t auto_max_min;
  int32_t max;
  int32_t min;
} Graph_eeprom;

typedef struct eeprom_data {
	uint8_t eeprom_version; // Used to detect changes in eeprom encoding, if != EEPROM_VERSION we will not use it

	uint8_t ui8_bit_data_1;
	uint8_t ui8_bit_data_2;
	uint8_t ui8_bit_data_3;

	uint8_t ui8_riding_mode;
	uint8_t ui8_assist_level;
	uint16_t ui16_wheel_perimeter;
	uint8_t ui8_wheel_max_speed;
//	uint8_t ui8_units_type;
	uint32_t ui32_wh_x10_offset;
	uint32_t ui32_wh_x10_100_percent;
	uint8_t ui8_battery_soc_enable;
	uint8_t ui8_target_max_battery_power_div25;
	uint8_t ui8_battery_max_current;
	uint8_t ui8_motor_max_current; // CHECK
	uint8_t ui8_motor_current_min_adc;
	//uint8_t ui8_field_weakening;
//	uint8_t ui8_ramp_up_amps_per_second_x10;
	uint16_t ui16_battery_low_voltage_cut_off_x10;
	//uint8_t ui8_motor_type;
	//uint8_t ui8_motor_current_control_mode;
	//uint8_t ui8_motor_assistance_startup_without_pedal_rotation;
	uint8_t ui8_assist_level_factor[4][ASSIST_LEVEL_NUMBER];
	uint8_t ui8_number_of_assist_levels;
	//uint8_t ui8_startup_motor_power_boost_feature_enabled;
	//uint8_t ui8_startup_motor_power_boost_always;
	//uint8_t ui8_startup_motor_power_boost_limit_power;
//	uint8_t ui8_startup_motor_power_boost_factor[ASSIST_LEVEL_NUMBER];
//	uint8_t ui8_startup_motor_power_boost_time;
//	uint8_t ui8_startup_motor_power_boost_fade_time;
	uint8_t ui8_optional_ADC_function;
	uint8_t ui8_motor_temperature_min_value_to_limit;
	uint8_t ui8_motor_temperature_max_value_to_limit;
	uint16_t ui16_battery_voltage_reset_wh_counter_x10;
	uint8_t ui8_lcd_power_off_time_minutes;
	uint8_t ui8_lcd_backlight_on_brightness;
	uint8_t ui8_lcd_backlight_off_brightness;
	uint16_t ui16_battery_pack_resistance_x1000;
	//uint8_t ui8_offroad_feature_enabled;
	//uint8_t ui8_offroad_enabled_on_startup;
	//uint8_t ui8_offroad_speed_limit;
	//uint8_t ui8_offroad_power_limit_enabled;
	//uint8_t ui8_offroad_power_limit_div25;
	uint32_t ui32_odometer_x10;
	//uint8_t ui8_walk_assist_feature_enabled;
	uint8_t ui8_walk_assist_level_factor[ASSIST_LEVEL_NUMBER];

	//uint8_t ui8_battery_soc_increment_decrement;
	//uint8_t ui8_buttons_up_down_invert;
	//uint8_t ui8_torque_sensor_calibration_feature_enabled;
	//uint8_t ui8_torque_sensor_calibration_pedal_ground;
	//uint16_t ui16_torque_sensor_calibration_table_left[8][2];
	//uint16_t ui16_torque_sensor_calibration_table_right[8][2];

	uint8_t field_selectors[NUM_CUSTOMIZABLE_FIELDS]; // this array is opaque to the app, but the screen layer uses it to store which field is being displayed (it is stored to EEPROM)
	uint8_t graphs_field_selectors[3]; // 3 screen main pages

	uint8_t x_axis_scale; // x axis scale
	uint8_t showNextScreenIndex;

	//uint8_t ui8_street_mode_function_enabled;
	//uint8_t ui8_street_mode_enabled;
	//uint8_t ui8_street_mode_enabled_on_startup;
	uint8_t ui8_street_mode_speed_limit;
	uint8_t ui8_street_mode_power_limit_div25;
	//uint8_t ui8_street_mode_throttle_enabled;
	//uint8_t ui8_street_mode_hotkey_enabled;

#ifndef SW102
  Graph_eeprom graph_eeprom[VARS_SIZE];
  uint8_t tripDistanceField_x_axis_scale_config;
  field_threshold_t wheelSpeedField_auto_thresholds;
  int32_t wheelSpeedField_config_error_threshold;
  int32_t wheelSpeedField_config_warn_threshold;
  uint8_t wheelSpeedField_x_axis_scale_config;
  field_threshold_t cadenceField_auto_thresholds;
  int32_t cadenceField_config_error_threshold;
  int32_t cadenceField_config_warn_threshold;
  uint8_t cadenceField_x_axis_scale_config;
  field_threshold_t humanPowerField_auto_thresholds;
  int32_t humanPowerField_config_error_threshold;
  int32_t humanPowerField_config_warn_threshold;
  uint8_t humanPowerField_x_axis_scale_config;
  field_threshold_t batteryPowerField_auto_thresholds;
  int32_t batteryPowerField_config_error_threshold;
  int32_t batteryPowerField_config_warn_threshold;
  uint8_t batteryPowerField_x_axis_scale_config;
  field_threshold_t batteryPowerUsageField_auto_thresholds;
  int32_t batteryPowerUsageField_config_error_threshold;
  int32_t batteryPowerUsageField_config_warn_threshold;
  uint8_t batteryPowerUsageField_x_axis_scale_config;
  field_threshold_t batteryVoltageField_auto_thresholds;
  int32_t batteryVoltageField_config_error_threshold;
  int32_t batteryVoltageField_config_warn_threshold;
  uint8_t batteryVoltageField_x_axis_scale_config;
  field_threshold_t batteryCurrentField_auto_thresholds;
  int32_t batteryCurrentField_config_error_threshold;
  int32_t batteryCurrentField_config_warn_threshold;
  uint8_t batteryCurrentField_x_axis_scale_config;
  field_threshold_t motorCurrentField_auto_thresholds;
  int32_t motorCurrentField_config_error_threshold;
  int32_t motorCurrentField_config_warn_threshold;
  uint8_t motorCurrentField_x_axis_scale_config;
  field_threshold_t batterySOCField_auto_thresholds;
  int32_t batterySOCField_config_error_threshold;
  int32_t batterySOCField_config_warn_threshold;
  uint8_t batterySOCField_x_axis_scale_config;
  field_threshold_t motorTempField_auto_thresholds;
  int32_t motorTempField_config_error_threshold;
  int32_t motorTempField_config_warn_threshold;
  uint8_t motorTempField_x_axis_scale_config;
  field_threshold_t motorErpsField_auto_thresholds;
  int32_t motorErpsField_config_error_threshold;
  int32_t motorErpsField_config_warn_threshold;
  uint8_t motorErpsField_x_axis_scale_config;
  field_threshold_t pwmDutyField_auto_thresholds;
  int32_t pwmDutyField_config_error_threshold;
  int32_t pwmDutyField_config_warn_threshold;
  uint8_t pwmDutyField_x_axis_scale_config;
  field_threshold_t motorFOCField_auto_thresholds;
  int32_t motorFOCField_config_error_threshold;
  int32_t motorFOCField_config_warn_threshold;
  uint8_t motorFOCField_x_axis_scale_config;
#endif

  //uint8_t ui8_pedal_cadence_fast_stop;
  uint8_t ui8_coast_brake_adc;
  //uint8_t ui8_adc_lights_current_offset;
  uint8_t ui8_throttle_virtual_step;
  //uint8_t ui8_torque_sensor_filter;
  uint8_t ui8_torque_sensor_adc_threshold;
  //uint8_t ui8_coast_brake_enable;
  
  uint8_t ui8_motor_acceleration_adjustment;
  uint8_t ui8_time_field_enable;
  uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100;
  uint8_t ui8_lights_configuration;
  uint16_t ui16_startup_boost_torque_factor;
  uint8_t ui8_startup_boost_cadence_step;
  uint16_t ui16_adc_pedal_torque_offset;
  uint16_t ui16_adc_pedal_torque_max;
  uint8_t ui8_weight_on_pedal;
  uint16_t ui16_adc_pedal_torque_calibration;
  
#ifndef SW102
  uint8_t  ui8_trip_a_auto_reset;
  uint16_t ui16_trip_a_auto_reset_hours;
  uint32_t ui32_trip_a_last_update_time;
#endif
  uint32_t ui32_trip_a_distance_x1000;
  uint32_t ui32_trip_a_time;
  uint16_t ui16_trip_a_max_speed_x10;

#ifndef SW102  
  uint8_t  ui8_trip_b_auto_reset;
  uint16_t ui16_trip_b_auto_reset_hours;
  uint32_t ui32_trip_b_last_update_time;
#endif
  uint32_t ui32_trip_b_distance_x1000;
  uint32_t ui32_trip_b_time;
  uint16_t ui16_trip_b_max_speed_x10;
  
  uint8_t ui8_motor_deceleration_adjustment;
  uint8_t ui8_screen_temperature;
  
#ifndef SW102  
  uint32_t ui32_wh_x10_trip_a_offset;
  uint32_t ui32_wh_x10_trip_b_offset;
#endif
  
// FIXME align to 32 bit value by end of structure and pack other fields
} eeprom_data_t;

void eeprom_init(void);
void eeprom_init_variables(void);
void eeprom_write_variables(void);
void eeprom_init_defaults(void);

// *************************************************************************** //
// EEPROM memory variables default values
#define DEFAULT_VALUE_RIDING_MODE	                                1
#define DEFAULT_VALUE_ASSIST_LEVEL                                  0
#define DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS                       9
#define DEFAULT_VALUE_WHEEL_PERIMETER                               2100 // 27.5'' wheel: 2100mm perimeter
#define DEFAULT_VALUE_WHEEL_MAX_SPEED                               50 // 50 km/h
#define DEFAULT_VALUE_UNITS_TYPE                                    0 // // 0=km/h, 1=miles
#ifndef SW102
#define DEFAULT_VALUE_WH_X10_TRIP_A_OFFSET							0
#define DEFAULT_VALUE_WH_X10_TRIP_B_OFFSET							0
#endif
#define DEFAULT_VALUE_WH_X10_OFFSET                                 0
#define DEFAULT_VALUE_HW_X10_100_PERCENT                            4000 // default to a battery of 400 Wh
#define DEAFULT_VALUE_SHOW_NUMERIC_BATTERY_SOC                      1 // // 0=none 1=SOC 2=volts
#define DEAFULT_VALUE_TIME_FIELD                                    1 // 1 i show clock
#define DEFAULT_VALUE_BATTERY_MAX_CURRENT                           16 // 16 amps
#define DEFAULT_VALUE_MOTOR_MAX_CURRENT                             16 // 16 amps NOT USED
#define DEFAULT_VALUE_CURRENT_MIN_ADC                               0 // 1 unit, 0.156 A
//#define DEFAULT_VALUE_RAMP_UP_AMPS_PER_SECOND_X10                   80 // 8.0 amps per second ramp up
#define DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER                      32 // 32 * 25 = 800, 0 is disabled
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10               420 // 52v battery, LVC = 42.0 (3.0 * 14)
#define DEFAULT_VALUE_BATTERY_PACK_RESISTANCE                       300 // 52v battery, 14S3P measured 300 milli ohms
//#define DEFAULT_VALUE_MOTOR_CURRENT_CONTROL_MODE                    0 // 0 power; 1 torque
#define DEFAULT_VALUE_MOTOR_TYPE                                    0 // 0 = 48V
#define DEFAULT_VALUE_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION       0 // 0 to keep this feature disable
#define DEFAULT_VALUE_ASSIST_WITH_ERROR								0

// default value for power assist
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_1                          25  // MAX 254
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_2                          50
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_3                          75
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_4                          100
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_5                          130
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_6                          160
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_7                          190
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_8                          220
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_9                          250

// default value for torque assist
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_1                         50	// MAX 254
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_2                         70
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_3                         90
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_4                         120
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_5                         140
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_6                         160
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_7                         190
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_8                         220
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_9                         250

// default value for cadence assist
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_1                        100	// MAX 254
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_2                        120
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_3                        130
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_4                        140
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_5                        160
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_6                        180
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_7                        200
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_8                        220
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_9                        250

// default value for eMTB assist
#define DEFAULT_VALUE_EMTB_ASSIST_LEVEL_1                           4	// MAX 20
#define DEFAULT_VALUE_EMTB_ASSIST_LEVEL_2                           6
#define DEFAULT_VALUE_EMTB_ASSIST_LEVEL_3                           8
#define DEFAULT_VALUE_EMTB_ASSIST_LEVEL_4                           10
#define DEFAULT_VALUE_EMTB_ASSIST_LEVEL_5                           12
#define DEFAULT_VALUE_EMTB_ASSIST_LEVEL_6                           14
#define DEFAULT_VALUE_EMTB_ASSIST_LEVEL_7                           16
#define DEFAULT_VALUE_EMTB_ASSIST_LEVEL_8                           18
#define DEFAULT_VALUE_EMTB_ASSIST_LEVEL_9                           20

#define DEFAULT_VALUE_WALK_ASSIST_FEATURE_ENABLED                   1
#define DEFAULT_VALUE_CRUISE_FEATURE_ENABLED                   		0
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_1                    35
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_2                    40
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_3                    45
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_4                    50
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_5                    55
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_6                    35
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_7                    40
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_8                    45
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_9                    50

#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED     1
#define DEFAULT_VALUE_STARTUP_BOOST_TORQUE_FACTOR					250
#define DEFAULT_VALUE_STARTUP_BOOST_CADENCE_STEP					25

#define DEFAULT_VALUE_STARTUP_ASSIST_FEATURE_ENABLED     			0

#define DEFAULT_VALUE_OPTIONAL_ADC_FUNCTION              			0 // 0=not used 1=temperature control 2=throttle control
#define DEFAULT_VALUE_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT             65 // 65 degrees celsius
#define DEFAULT_VALUE_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT             85 // 85 degrees celsius
#define DEFAULT_VALUE_SCREEN_TEMPERATURE							0 // 0=AUTO 1=CELSIUS 2=FARENHEIT		

#define DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10          584 // 52v battery, 58.4 volts at fully charged
#define DEFAULT_VALUE_LCD_POWER_OFF_TIME                            30 // 30 minutes, each unit 1 minute

#ifdef SW102
#define DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS                   100 // 8 = 40%
#define DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS                  20 // 20 = 100%
#else
#define DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS                   20 // 100 = 100%
#define DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS                  100
#endif

#define DEFAULT_VALUE_ODOMETER_X10                                  0
#define DEFAULT_VALUE_BUTTONS_UP_DOWN_INVERT                        0 // regular state
#define DEFAULT_VALUE_CONFIG_SHORTCUT_KEY_ENABLED	                0 
#define DEFAULT_VALUE_X_AXIS_SCALE                                  0 // 15m
#define DEFAULT_STREET_MODE_FUNCTION_ENABLE                         1 // enabled
#define DEFAULT_STREET_MODE_ENABLE_AT_STARTUP                       1 // enabled
#define DEFAULT_STREET_MODE_ENABLE                                  0 // disabled
#define DEFAULT_STREET_MODE_SPEED_LIMIT                             25 // 25 km/h
#define DEFAULT_STREET_MODE_POWER_LIMIT                             10 // 250W --> 250 / 25 = 10
#define DEFAULT_STREET_MODE_THROTTLE_ENABLE                         0 // disabled
#define DEFAULT_STREET_MODE_CRUISE_ENABLE                         	0 // disabled
#define DEFAULT_STREET_MODE_HOTKEY_ENABLE                           0 // disabled
#define DEFAULT_PEDAL_CADENCE_FAST_STOP_ENABLE                      0 // disabled
#define DEFAULT_COAST_BRAKE_ADC                                     30 // 15: tested by plpetrov user on 28.04.2020:
#define DEFAULT_VALUE_FIELD_WEAKENING_FEATURE_ENABLED               1 // 1 enabled
//#define DEFAULT_ADC_LIGHTS_CURRENT_OFFSET                           1
#define DEFAULT_THROTTLE_VIRTUAL_STEP                               5
//#define DEFAULT_TORQUE_SENSOR_FILTER                                20 // 20%
#define DEFAULT_TORQUE_SENSOR_ADC_THRESHOLD                         20
#ifndef SW102
#define DEFAULT_COAST_BRAKE_ENABLE                                  0 // disable
#else
#define DEFAULT_COAST_BRAKE_ENABLE                                  1 // enable
#endif
#define DEFAULT_VALUE_MOTOR_ACCELERATION_ADJUSTMENT					5
#define DEFAULT_VALUE_MOTOR_DECELERATION_ADJUSTMENT					5
#define DEFAULT_VALUE_PEDAL_TORQUE_ADC_STEP_x100					67
#define DEFAULT_LIGHTS_CONFIGURATION								0

#define DEFAULT_TORQUE_SENSOR_ADC_OFFSET							150
#define DEFAULT_TORQUE_SENSOR_ADC_MAX								300
#define DEFAULT_WEIGHT_ON_PEDAL_CALIBRATION							25
#define DEFAULT_TORQUE_SENSOR_ADC_CALIBRATION						250

#define DEFAULT_TORQUE_SENSOR_CALIBRATION_FEATURE_ENABLE            0 // disabled

#ifndef SW102
#define DEFAULT_VALUE_TRIP_AUTO_RESET_ENABLE                         0 // disable
#define DEFAULT_VALUE_TRIP_LAST_UPDATE                               0 // disable USED ?
#define DEFAULT_VALUE_TRIP_A_AUTO_RESET_HOURS                        24 // 1 day 
#define DEFAULT_VALUE_TRIP_B_AUTO_RESET_HOURS                        168 // 1 week = 7 * 24 = 168 hours
#endif
#define DEFAULT_VALUE_TRIP_DISTANCE                                  0
#define DEFAULT_VALUE_TRIP_TIME                                      0
#define DEFAULT_VALUE_TRIP_MAX_SPEED                                 0


#define DEFAULT_BIT_DATA_1 (DEFAULT_VALUE_UNITS_TYPE | \
(DEFAULT_VALUE_MOTOR_TYPE << 1) | \
(DEFAULT_COAST_BRAKE_ENABLE << 2) | \
(DEFAULT_VALUE_MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION << 3) | \
(DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED << 4) | \
(DEFAULT_PEDAL_CADENCE_FAST_STOP_ENABLE << 5) | \
(DEFAULT_VALUE_CRUISE_FEATURE_ENABLED << 6) | \
(DEFAULT_VALUE_WALK_ASSIST_FEATURE_ENABLED << 7))

#define DEFAULT_BIT_DATA_2 (DEFAULT_VALUE_BUTTONS_UP_DOWN_INVERT | \
(DEFAULT_TORQUE_SENSOR_CALIBRATION_FEATURE_ENABLE << 1) | \
(DEFAULT_VALUE_ASSIST_WITH_ERROR << 2) | \
(DEFAULT_STREET_MODE_FUNCTION_ENABLE << 3) | \
(DEFAULT_STREET_MODE_ENABLE << 4) | \
(DEFAULT_STREET_MODE_ENABLE_AT_STARTUP << 5) | \
(DEFAULT_STREET_MODE_THROTTLE_ENABLE << 6) | \
(DEFAULT_STREET_MODE_HOTKEY_ENABLE << 7))

#define DEFAULT_BIT_DATA_3	(DEFAULT_STREET_MODE_CRUISE_ENABLE | \
(DEFAULT_VALUE_CONFIG_SHORTCUT_KEY_ENABLED << 1) | \
(DEFAULT_VALUE_FIELD_WEAKENING_FEATURE_ENABLED << 2) | \
(DEFAULT_VALUE_STARTUP_ASSIST_FEATURE_ENABLED << 3))
// bit free for future use


// *************************************************************************** //

// Torque sensor value found experimentaly USED ?
// measuring with a cheap digital hook scale, we found that each torque sensor unit is equal to 0.556 Nm
// using the scale, was found that each 1kg was measured as 3 torque sensor units
// Force (Nm) = Kg * 9.18 * 0.17 (arm cranks size)
#define TORQUE_SENSOR_FORCE_SCALE_X1000 556

// *************************************************************************** //
// BATTERY

// ADC Battery voltage USED ?
// 0.344 per ADC_8bits step: 17.9V --> ADC_8bits = 52; 40V --> ADC_8bits = 116; this signal atenuated by the opamp 358
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 44
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X256 (ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 >> 1)
#define ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP 0.344

// ADC Battery current USED ?
// 1A per 5 steps of ADC_10bits
#define ADC_BATTERY_CURRENT_PER_ADC_STEP_X512 80
// *************************************************************************** //

#endif /* _EEPROM_H_ */
