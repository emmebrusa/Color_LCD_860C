#pragma once

#include <stdbool.h>
#include <stdint.h>

// Torque sensor calibration from main.h v20.1C.4 TSDZ2-OSF
#define ADC_TORQUE_SENSOR_CALIBRATION_OFFSET    	6
#define ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ			20
#define ADC_TORQUE_SENSOR_RANGE_TARGET	  			160
#define ADC_TORQUE_SENSOR_RANGE_TARGET_MIN 			133
#define PEDAL_TORQUE_PER_10_BIT_ADC_STEP_BASE_X100	34 // base adc step
#define WEIGHT_ON_PEDAL_FOR_STEP_CALIBRATION		24 // Kg
#define PERCENT_TORQUE_SENSOR_RANGE_WITH_WEIGHT		75 // % of torque sensor range target with weight
#define ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT		(uint16_t)((ADC_TORQUE_SENSOR_RANGE_TARGET*PERCENT_TORQUE_SENSOR_RANGE_WITH_WEIGHT)/100)

// SOC calculation
#define SOC_CALC_AUTO						0
#define SOC_CALC_WH							1
#define SOC_CALC_VOLTS						2

// optional ADC function
#define NOT_IN_USE							0
#define TEMPERATURE_CONTROL					1
#define THROTTLE_CONTROL					2

// throttle and cruise
#define DISABLED							0
#define PEDALING							1
// throttle only
#define WP_6KM_H_ONLY						2
#define WP_6KM_H_AND_PEDALING				3
#define UNCONDITIONAL						4
// cruise only
#define WITHOUT_PEDALING					2

#define SPEED_LIMIT_WITHOUT_PEDALING		6
#define SPEED_LIMIT_WITHOUT_PEDALING_x10	60
#define MAX_SPEED_WITHOUT_PEDALING_x10		70

// screen temperature
#define AUTO								0
#define CELSIUS								1
#define FARENHEIT							2

// temperature sensor type
#define LM35								0
#define TMP36								1

// brake input
#define BRAKE								0
#define TEMPERATURE							1

// password
#define LOGOUT								0
#define LOGIN								1
#define WAIT								2
#define CHANGE								3

// assist level number
#define ASSIST_LEVEL_NUMBER					9

typedef enum {
  MOTOR_INIT_GET_MOTOR_ALIVE,
  MOTOR_INIT_WAIT_MOTOR_ALIVE,
  MOTOR_INIT_GET_MOTOR_FIRMWARE_VERSION,
  MOTOR_INIT_WAIT_MOTOR_FIRMWARE_VERSION,
  MOTOR_INIT_GOT_MOTOR_FIRMWARE_VERSION,
  MOTOR_INIT_ERROR_GET_FIRMWARE_VERSION,
  MOTOR_INIT_RECEIVED_MOTOR_FIRMWARE_VERSION,
  MOTOR_INIT_ERROR_FIRMWARE_VERSION,
  MOTOR_INIT_SET_CONFIGURATIONS,
  MOTOR_INIT_WAIT_CONFIGURATIONS_OK,
  MOTOR_INIT_WAIT_GOT_CONFIGURATIONS_OK,
  MOTOR_INIT_ERROR_SET_CONFIGURATIONS,
  MOTOR_INIT_ERROR,
  MOTOR_INIT_READY,
  MOTOR_INIT_SIMULATING,
} motor_init_state_t;

typedef enum {
  MOTOR_INIT_CONFIG_SEND_CONFIG,
  MOTOR_INIT_CONFIG_GET_STATUS,
  MOTOR_INIT_CONFIG_CHECK_STATUS,
} motor_init_state_config_t;

typedef enum {
  MOTOR_INIT_STATUS_RESET = 0,
  MOTOR_INIT_STATUS_GOT_CONFIG = 1,
  MOTOR_INIT_STATUS_INIT_OK = 2,
} motor_init_status_t;

extern volatile motor_init_state_t g_motor_init_state;

typedef struct battery_energy_h_km_struct {
  uint32_t ui32_sum_x50;
  uint32_t ui32_value_x100;
  uint32_t ui32_value_x10;
} battery_energy_h_km_t;

typedef struct rt_vars_struct {
	uint16_t ui16_adc_battery_voltage;
	uint8_t ui8_battery_current_x5;
	uint16_t ui16_battery_power_loss;
	uint8_t ui8_motor_current_x5;
	uint8_t ui8_adc_throttle;
	uint8_t ui8_throttle;
	uint16_t ui16_adc_pedal_torque_sensor;
	uint8_t ui8_pedal_weight_with_offset; // used in firmware/SW102/src/sw102/ble_service.c
	uint8_t ui8_pedal_weight;
	uint16_t ui16_pedal_power_x10;
	uint8_t ui8_duty_cycle;
	uint8_t ui8_motor_efficiency;
	uint8_t ui8_error_states;
	uint16_t ui16_wheel_speed_x10;
	uint8_t ui8_pedal_cadence;
	uint16_t ui16_motor_speed_erps;
	uint8_t ui8_foc_angle;
	uint8_t ui8_motor_hall_sensors;
	uint8_t ui8_motor_temperature;
	//uint32_t ui32_wheel_speed_sensor_tick_counter;
	uint16_t ui16_battery_voltage_filtered_x10;
	uint16_t ui16_battery_current_filtered_x5;
	uint16_t ui16_motor_current_filtered_x5;
	uint16_t ui16_full_battery_power_filtered_x50;
	uint16_t ui16_battery_power_filtered;
	uint16_t ui16_pedal_power_filtered;
	uint8_t ui8_pedal_cadence_filtered;
	uint16_t ui16_battery_voltage_soc_x10;
	uint32_t ui32_wh_sum_x5;
	uint32_t ui32_wh_sum_counter;
	uint32_t ui32_wh_x10;
	//uint32_t ui32_wheel_speed_sensor_tick_counter_offset;

//#ifndef SW102
	//uint32_t ui32_wh_x10_total;
	//uint32_t ui32_wh_x10_total_offset;
	uint16_t ui16_battery_charge_cycles_x10;
#ifndef SW102
	//uint32_t ui32_wh_x10_trip_a;
	//uint32_t ui32_wh_x10_trip_b;
	uint32_t ui32_wh_x10_trip_a_offset;
	uint32_t ui32_wh_x10_trip_b_offset;
	
	uint16_t ui16_service_a_distance;
	uint16_t ui16_service_b_distance;
	//uint16_t ui16_service_b_time;
	//uint8_t ui8_service_a_distance_enable;
	//uint8_t ui8_service_b_distance_enable;
#endif

	uint8_t ui8_assist_level;
	uint8_t ui8_number_of_assist_levels;
	uint16_t ui16_wheel_perimeter;
	uint8_t ui8_wheel_max_speed;
	uint8_t ui8_units_type;
	uint32_t ui32_wh_x10_offset;
	uint32_t ui32_wh_x10_100_percent;
	uint16_t ui16_motor_power_limit;
	uint8_t ui8_target_max_battery_power_div25;
	uint8_t ui8_battery_max_current;
	uint8_t ui8_motor_max_current;
	uint8_t ui8_motor_current_min_adc;
	uint8_t ui8_field_weakening_feature_enabled;
	uint16_t ui16_battery_low_voltage_cut_off_x10;
	uint16_t ui16_battery_voltage_calibrate_percent_x10;
	uint16_t ui16_battery_voltage_reset_wh_counter_x10;
	uint16_t ui16_battery_pack_resistance_x1000;
	uint8_t ui8_motor_type;
	uint8_t ui8_motor_assistance_startup_without_pedal_rotation;
	uint8_t ui8_assist_level_factor[4][ASSIST_LEVEL_NUMBER];
	uint8_t ui8_walk_assist_feature_enabled;
	uint8_t ui8_throttle_feature_enabled;
	uint8_t ui8_cruise_feature_enabled;
	uint8_t ui8_walk_assist_level_factor[ASSIST_LEVEL_NUMBER];
	uint8_t ui8_startup_motor_power_boost_feature_enabled;
	uint8_t ui8_startup_boost_at_zero;
	uint8_t ui8_startup_assist_feature_enabled;
	uint8_t ui8_optional_ADC_function;
	uint8_t ui8_motor_temperature_min_value_to_limit;
	uint8_t ui8_motor_temperature_max_value_to_limit;
	uint8_t ui8_screen_temperature;
	uint8_t ui8_temperature_sensor_type;
	uint8_t ui8_lcd_backlight_on_brightness;
	uint8_t ui8_lcd_backlight_off_brightness;
	uint32_t ui32_odometer_x10;

	uint32_t ui32_trip_a_distance_x10;
	uint32_t ui32_trip_b_distance_x10;

	uint8_t ui8_lights;
	uint8_t ui8_braking;
	uint8_t ui8_walk_assist;
	uint8_t ui8_startup_assist;
	//uint8_t ui8_offroad_mode;

  uint8_t ui8_torque_sensor_calibration_feature_enabled;

  uint8_t ui8_street_mode_enabled;
  uint8_t ui8_street_mode_speed_limit;
  uint8_t ui8_street_mode_power_limit_div25;
  uint8_t ui8_street_mode_throttle_enabled;
  uint8_t ui8_street_mode_cruise_enabled;

  uint8_t ui8_pedal_cadence_fast_stop;
  uint8_t ui8_coast_brake_adc;
  uint16_t ui16_adc_battery_current;
  uint8_t ui8_throttle_virtual;
  uint8_t ui8_torque_sensor_adc_threshold;
  uint8_t ui8_coast_brake_enable;

  uint8_t ui8_motor_acceleration_adjustment;
  uint8_t ui8_motor_deceleration_adjustment;
  uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100;
  uint8_t ui8_pedal_torque_per_10_bit_ADC_step_adv_x100;
  uint8_t ui8_lights_configuration;
  uint8_t ui8_assist_whit_error_enabled;
  uint16_t ui16_startup_boost_torque_factor;
  uint8_t ui8_startup_boost_cadence_step;
  uint8_t ui8_riding_mode;
  uint16_t ui16_adc_pedal_torque_delta;
  uint16_t ui16_adc_pedal_torque_delta_boost;
  uint8_t ui8_adc_pedal_torque_offset_adj;
  uint8_t ui8_adc_pedal_torque_range_adj;
  uint8_t ui8_adc_pedal_torque_angle_adj_index;
  uint16_t ui16_adc_pedal_torque_offset;
  uint16_t ui16_adc_pedal_torque_max;
  uint8_t ui8_weight_on_pedal;
  uint16_t ui16_adc_pedal_torque_with_weight;
  uint8_t ui8_pedal_torque_ADC_step_calc_x100;
  uint8_t ui8_config_shortcut_key_enabled;
  uint8_t ui8_battery_soc_auto_reset;
  uint8_t ui8_smooth_start_enabled;
  uint8_t ui8_smooth_start_counter_set;
  uint8_t ui8_eMTB_based_on_power;
#ifndef SW102
  uint32_t ui32_RTC_total_seconds;
#endif
  battery_energy_h_km_t battery_energy_h_km;
} rt_vars_t;

/* Selector positions for customizable fields
 * 0 is the graph,
 * 1-4  are the boxes above the graph, mainscreen1 on 850C
 * 5-8  are the boxes above the graph, mainscreen2 on 850C
 * 9-12 are the boxes above the graph, mainscreen2 on 850C
 */
#ifdef SW102
#define NUM_CUSTOMIZABLE_FIELDS 6
#else
#define NUM_CUSTOMIZABLE_FIELDS 12
#endif

typedef struct ui_vars_struct {
	uint16_t ui16_adc_battery_voltage;
	uint8_t ui8_battery_current_x5;
	uint16_t ui16_battery_power_loss;
	uint8_t ui8_motor_current_x5;
	uint8_t ui8_adc_throttle;
	uint8_t ui8_throttle;
	uint16_t ui16_adc_pedal_torque_sensor;
	uint8_t ui8_pedal_weight_with_offset;  // used in firmware/SW102/src/sw102/ble_service.c
	uint8_t ui8_pedal_weight;
	uint8_t ui8_duty_cycle;
	uint8_t ui8_motor_efficiency;
	uint8_t ui8_error_states;
	uint16_t ui16_wheel_speed_x10;
	uint8_t ui8_pedal_cadence;
	uint16_t ui16_motor_speed_erps;
	uint8_t ui8_foc_angle;
	uint8_t ui8_motor_hall_sensors;
	uint8_t ui8_motor_temperature;
	uint32_t ui32_wheel_speed_sensor_tick_counter;
	uint32_t ui32_wheel_speed_sensor_tick_counter_offset;
	uint16_t ui16_battery_voltage_filtered_x10;
	uint16_t ui16_battery_current_filtered_x5;
	uint16_t ui16_motor_current_filtered_x5;
	uint16_t ui16_full_battery_power_filtered_x50;
	uint16_t ui16_battery_power;
	uint16_t ui16_pedal_torque_filtered;
	uint16_t ui16_pedal_power;
	uint8_t ui8_pedal_cadence_filtered;
	uint8_t ui8_battery_soc_percent_calculation;
	uint16_t ui16_battery_voltage_soc_x10;
	uint32_t ui32_wh_sum_x5;
	uint32_t ui32_wh_sum_counter;
	uint32_t ui32_wh_x10;

//#ifndef SW102
	uint32_t ui32_wh_x10_total;
	uint32_t ui32_wh_x10_total_offset;
	uint16_t ui16_battery_charge_cycles_x10;
#ifndef SW102
	uint32_t ui32_wh_x10_trip_a;
	uint32_t ui32_wh_x10_trip_b;
	uint32_t ui32_wh_x10_trip_a_offset;
	uint32_t ui32_wh_x10_trip_b_offset;
	
	uint16_t ui16_service_a_distance;
	uint16_t ui16_service_b_distance;
	//uint16_t ui16_service_b_time;
	uint8_t ui8_service_a_distance_enable;
	uint8_t ui8_service_b_distance_enable;
#endif
	
	uint8_t ui8_assist_level;
	uint8_t ui8_number_of_assist_levels;
	uint16_t ui16_wheel_perimeter;
	uint8_t ui8_wheel_max_speed;
	uint16_t ui16_wheel_max_speed_x10;
	uint8_t ui8_units_type;
	uint32_t ui32_wh_x10_offset;
	uint32_t ui32_wh_x10_100_percent;
	uint8_t ui8_battery_soc_enable;
	uint8_t ui8_time_field_enable;
	uint8_t ui8_motor_power_limit_div25;
	uint16_t ui16_motor_power_limit;
	uint8_t ui8_target_max_battery_power_div25;
	uint16_t ui16_target_max_battery_power;
	uint8_t ui8_battery_max_current;
	uint8_t ui8_motor_max_current;
	uint8_t ui8_motor_current_min_adc;
	uint8_t ui8_field_weakening_feature_enabled;
	uint16_t ui16_battery_low_voltage_cut_off_x10;
	uint16_t ui16_battery_voltage_calibrate_percent_x10;
	uint16_t ui16_battery_voltage_reset_wh_counter_x10;
	uint16_t ui16_battery_pack_resistance_x1000;
	uint16_t ui16_battery_pack_resistance_estimated_x1000;
	uint8_t ui8_motor_type;
	uint8_t ui8_motor_assistance_startup_without_pedal_rotation;
	uint8_t ui8_assist_level_factor[4][ASSIST_LEVEL_NUMBER];
	uint8_t ui8_walk_assist_feature_enabled;
	uint8_t ui8_throttle_feature_enabled;
	uint8_t ui8_cruise_feature_enabled;
	uint8_t ui8_walk_assist_level_factor[ASSIST_LEVEL_NUMBER];
	uint8_t ui8_startup_motor_power_boost_feature_enabled;
	uint8_t ui8_startup_boost_at_zero;
	uint8_t ui8_startup_assist_feature_enabled;
	uint8_t ui8_password_enabled;
	uint16_t ui16_entered_password;
	uint16_t ui16_saved_password;
	uint8_t ui8_wait_confirm_password;
	uint8_t ui8_password_first_time;
	uint8_t ui8_password_changed;
	uint8_t ui8_password_confirmed;
	uint8_t ui8_confirm_password;
	uint8_t ui8_reset_password;
	uint8_t ui8_confirm_default_reset;
	uint8_t ui8_optional_ADC_function;
	uint8_t ui8_motor_temperature_min_value_to_limit;
	uint8_t ui8_motor_temperature_max_value_to_limit;
	uint8_t ui8_screen_temperature;
	uint8_t ui8_temperature_sensor_type;
	uint8_t ui8_lcd_power_off_time_minutes;
	uint8_t ui8_lcd_backlight_on_brightness;
	uint8_t ui8_lcd_backlight_off_brightness;
	uint32_t ui32_odometer_x10;

#ifndef SW102
	uint8_t  ui8_trip_a_auto_reset;
	uint16_t ui16_trip_a_auto_reset_hours;
	uint32_t ui32_trip_a_last_update_time;
#endif
	uint32_t ui32_trip_a_distance_x10;
	uint32_t ui32_trip_a_time;
	uint16_t ui16_trip_a_avg_speed_x10;
	uint16_t ui16_trip_a_max_speed_x10;

#ifndef SW102
	uint8_t  ui8_trip_b_auto_reset;
	uint16_t ui16_trip_b_auto_reset_hours;
	uint32_t ui32_trip_b_last_update_time;
#endif
	uint32_t ui32_trip_b_distance_x10;
	uint32_t ui32_trip_b_time;
  	uint16_t ui16_trip_b_avg_speed_x10;
  	uint16_t ui16_trip_b_max_speed_x10;

	uint32_t battery_energy_km_value_x100;

	uint8_t ui8_lights;
	uint8_t ui8_braking;
	uint8_t ui8_walk_assist;
	uint8_t ui8_startup_assist;
	//uint8_t ui8_offroad_mode;
	uint8_t ui8_buttons_up_down_invert;
	uint8_t ui8_brake_input;

	uint8_t ui8_torque_sensor_calibration_feature_enabled;

	uint8_t field_selectors[NUM_CUSTOMIZABLE_FIELDS]; // this array is opaque to the app, but the screen layer uses it to store which field is being displayed (it is stored to EEPROM)
	uint8_t graphs_field_selectors[3]; // 3 screen main pages

	uint8_t ui8_street_mode_function_enabled;
	uint8_t ui8_street_mode_enabled;
	uint8_t ui8_street_mode_enabled_on_startup;
	uint8_t ui8_street_mode_speed_limit;
	uint8_t ui8_street_mode_power_limit_div25;
	uint16_t ui16_street_mode_power_limit;
	uint8_t ui8_street_mode_throttle_enabled;
	uint8_t ui8_street_mode_cruise_enabled;
	uint8_t ui8_street_mode_hotkey_enabled;
	
  uint8_t ui8_pedal_cadence_fast_stop;
  uint8_t ui8_coast_brake_adc;
  uint16_t ui16_adc_battery_current;
  uint8_t ui8_throttle_virtual;
  uint8_t ui8_throttle_virtual_step;
  uint8_t ui8_torque_sensor_adc_threshold;
  uint8_t ui8_coast_brake_enable;
  
  uint8_t ui8_motor_acceleration_adjustment;
  uint8_t ui8_motor_deceleration_adjustment;
  uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100;
  uint8_t ui8_pedal_torque_per_10_bit_ADC_step_adv_x100;
  uint8_t ui8_lights_configuration;
  uint8_t ui8_assist_whit_error_enabled;
  uint16_t ui16_startup_boost_torque_factor;
  uint8_t ui8_startup_boost_cadence_step;
  uint8_t ui8_riding_mode;
  uint16_t ui16_adc_pedal_torque_delta;
  uint16_t ui16_adc_pedal_torque_delta_boost;
  uint8_t ui8_adc_pedal_torque_offset_adj;
  uint8_t ui8_adc_pedal_torque_range_adj;
  uint8_t ui8_adc_pedal_torque_angle_adj_index;
  uint16_t ui16_adc_pedal_torque_offset;
  uint16_t ui16_adc_pedal_torque_max;
  uint8_t ui8_weight_on_pedal;
  uint16_t ui16_adc_pedal_torque_with_weight;
  uint8_t ui8_pedal_torque_ADC_step_calc_x100;
  uint8_t ui8_config_shortcut_key_enabled;
  uint8_t ui8_battery_soc_auto_reset;
  uint8_t ui8_smooth_start_enabled;
  uint8_t ui8_smooth_start_counter_set;
  uint8_t ui8_eMTB_based_on_power;
#if defined(DISPLAY_860C) || defined(DISPLAY_860C_V12) || defined(DISPLAY_860C_V13)
  uint8_t ui8_light_sensor_enabled;
  uint8_t ui8_light_sensor_sensitivity;
  uint8_t ui8_light_sensor_hysteresis;
#endif
  uint8_t ui8_startup_assist_level;
  uint8_t ui8_startup_ridimg_mode;
  uint32_t ui32_last_errors;
  uint8_t ui8_last_error[4];
  uint8_t ui8_history_errors_reset;
#ifndef SW102
  uint32_t ui32_time_since_error[4];
  uint32_t ui32_last_error_time[4];
  uint32_t ui32_seconds_at_shutdown;
  uint32_t ui32_RTC_total_seconds;
  uint8_t ui8_motor_efficiency_auto_thresholds;
  uint8_t ui8_motor_efficiency_error_threshold;
  uint8_t ui8_motor_efficiency_warn_threshold;
#endif
  uint8_t ui8_battery_overcurrent_delay;

} ui_vars_t;

ui_vars_t* get_ui_vars(void);
rt_vars_t* get_rt_vars(void);

extern rt_vars_t rt_vars; // FIXME - this shouldn't be exposed outside of state.c - but currently mid merge
extern ui_vars_t ui_vars;

extern volatile uint8_t ui8_g_motorVariablesStabilized;

typedef struct {
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
} tsdz2_firmware_version_t;

void rt_processing(void);
void rt_processing_stop(void);
void rt_processing_start(void);

/**
 * Called from the main thread every 100ms
 *
 */
void copy_rt_to_ui_vars(void);

/// must be called from main() idle loop
void automatic_power_off_management(void);

void lcd_power_off(uint8_t updateDistanceOdo); // provided by LCD

/// Set correct backlight brightness for current headlight state
void set_lcd_backlight();

void prepare_torque_sensor_calibration_table(void);

void reset_wh(void);

extern volatile uint8_t ui8_trip_started;
extern volatile uint8_t ui8_voltage_cut_off_flag;
extern volatile uint8_t ui8_voltage_shutdown_flag;
extern volatile uint8_t ui8_speed_limit_high_flag;
extern uint8_t ui8_g_battery_soc;
extern uint8_t ui8_g_screen_init_flag;

extern tsdz2_firmware_version_t g_tsdz2_firmware_version;

extern volatile motor_init_status_t ui8_g_motor_init_status;

// Battery voltage (readed on motor controller):
#define ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X10000 866

// Battery voltage (readed on LCD3):
// 30.0V --> 447 | 0.0671 volts per each ADC unit
// 40.0V --> 595 | 0.0672 volts per each ADC unit

// Possible values: 0, 1, 2, 3, 4, 5, 6
// 0 equal to no filtering and no delay, higher values will increase filtering but will also add bigger delay
#define BATTERY_VOLTAGE_FILTER_COEFFICIENT 3
#define BATTERY_CURRENT_FILTER_COEFFICIENT 2
#define MOTOR_CURRENT_FILTER_COEFFICIENT   2
#define PEDAL_POWER_FILTER_COEFFICIENT     3
#define PEDAL_CADENCE_FILTER_COEFFICIENT   3

