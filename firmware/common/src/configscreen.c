#include "screen.h"
#include "mainscreen.h"
#include "configscreen.h"
#include "eeprom.h"



static Field tripMenus[] =
{
#ifndef SW102
	FIELD_EDITABLE_ENUM("A auto reset", &ui_vars.ui8_trip_a_auto_reset, "disable", "enable"),
	FIELD_EDITABLE_UINT("A auto reset hours", &ui_vars.ui16_trip_a_auto_reset_hours, "hrs", 0, 999, .inc_step = 1),
	FIELD_EDITABLE_ENUM("B auto reset", &ui_vars.ui8_trip_b_auto_reset, "disable", "enable"),
	FIELD_EDITABLE_UINT("B auto reset hours", &ui_vars.ui16_trip_b_auto_reset_hours, "hrs", 0, 999, .inc_step = 1),
#endif
	FIELD_EDITABLE_ENUM(_S("Reset trip A", "Rst trip A"), &ui8_g_configuration_trip_a_reset, "no", "yes"),
	FIELD_EDITABLE_ENUM(_S("Reset trip B", "Rst trip B"), &ui8_g_configuration_trip_b_reset, "no", "yes"),
/*
#ifndef SW102
	FIELD_READONLY_UINT("Used Wh trip A", &ui_vars.ui32_wh_x10_trip_a, "whr", false, .div_digits = 1),
	FIELD_READONLY_UINT("Used Wh trip B", &ui_vars.ui32_wh_x10_trip_b, "whr", false, .div_digits = 1),
#endif
*/
	FIELD_END };

static Field wheelMenus[] =
{
	FIELD_EDITABLE_UINT("Max speed", &ui_vars.ui16_wheel_max_speed_x10, "kph", 1, 990, .div_digits = 1, .inc_step = 10, .hide_fraction = true),
	FIELD_EDITABLE_UINT(_S("Circumference", "Circumfere"), &ui_vars.ui16_wheel_perimeter, "mm", 750, 3000, .inc_step = 10),
	FIELD_END };

static Field batteryMenus[] =
{
	FIELD_EDITABLE_UINT(_S("Max current", "Max curren"), &ui_vars.ui8_battery_max_current, "amps", 1, 18),
	FIELD_EDITABLE_UINT(_S("Low cut-off", "Lo cut-off"), &ui_vars.ui16_battery_low_voltage_cut_off_x10, "volts", 160, 630, .div_digits = 1),
	FIELD_EDITABLE_UINT(_S("Voltage cal %", "Voltag cal"), &ui_vars.ui16_battery_voltage_calibrate_percent_x10, "volts", 950, 1050, .div_digits = 1),
	FIELD_EDITABLE_UINT(_S("Resistance", "Resistance"), &ui_vars.ui16_battery_pack_resistance_x1000, "mohm", 0, 1000),
	FIELD_READONLY_UINT(_S("Voltage est", "Voltag est"), &ui_vars.ui16_battery_voltage_soc_x10, "volts", false, .div_digits = 1),
	FIELD_READONLY_UINT(_S("Resistance est", "Resist est"), &ui_vars.ui16_battery_pack_resistance_estimated_x1000, "mohm", 0, 1000),
	FIELD_READONLY_UINT(_S("Power loss est", "Power loss"), &ui_vars.ui16_battery_power_loss, "watts", false, .div_digits = 0),
	FIELD_END };

static Field batterySOCMenus[] =
{
	FIELD_EDITABLE_ENUM("Text", &ui_vars.ui8_battery_soc_enable, "disable", "SOC %", "volts"),
	FIELD_EDITABLE_ENUM("Calculation", &ui_vars.ui8_battery_soc_percent_calculation, "auto", "Wh", "volts"),
	FIELD_EDITABLE_UINT(_S("Reset at voltage", "Reset at"), &ui_vars.ui16_battery_voltage_reset_wh_counter_x10, "volts", 160, 630, .div_digits = 1),
	FIELD_EDITABLE_UINT(_S("Battery total Wh", "Batt total"), &ui_vars.ui32_wh_x10_100_percent, "whr", 0, 29990, .div_digits = 1, .inc_step = 100),
	FIELD_EDITABLE_UINT("Used Wh", &ui_vars.ui32_wh_x10, "whr", 0, 99900, .div_digits = 1, .inc_step = 10, .onSetEditable = onSetConfigurationBatterySOCUsedWh),
	FIELD_EDITABLE_ENUM(_S("Manual reset", "Manual rst"), &ui8_g_configuration_battery_soc_reset, "no", "yes"),
	FIELD_EDITABLE_UINT(_S("Auto reset %", "Auto rst%"), &ui_vars.ui8_battery_soc_auto_reset, "", 0, 100),
#ifndef SW102
	FIELD_READONLY_UINT("Charge cycles", &ui_vars.ui16_battery_charge_cycles_x10, "", .div_digits = 1),
	//FIELD_READONLY_UINT("Used total Wh", &ui_vars.ui32_wh_x10_total, "", .div_digits = 1),
#endif
	FIELD_END };

static Field motorMenus[] =
{
	FIELD_EDITABLE_ENUM(_S("Motor voltage", "Motor volt"), &ui_vars.ui8_motor_type, "48V", "36V"),
	FIELD_EDITABLE_UINT(_S("Motor power max", "Power max"), &ui_vars.ui16_target_max_battery_power, "watts", 25, 2500, .div_digits = 0, .inc_step = 25, .hide_fraction = true),
	FIELD_EDITABLE_UINT(_S("Motor acceleration", "Motor acc"), &ui_vars.ui8_motor_acceleration_adjustment, "%", 0, 100, .div_digits = 0),
	FIELD_EDITABLE_UINT(_S("Motor deceleration", "Motor dec"), &ui_vars.ui8_motor_deceleration_adjustment, "%", 0, 100, .div_digits = 0),
	//FIELD_EDITABLE_UINT(_S("Min current ADC step", "MinADCcur"), &ui_vars.ui8_motor_current_min_adc, "amps", 0, 3), // 3 ADC steps = 0.48 amps
	//FIELD_EDITABLE_ENUM(_S("Motor fast stop", "Motor stop"), &ui_vars.ui8_pedal_cadence_fast_stop, "no", "yes"),
#ifndef SW102
	FIELD_EDITABLE_ENUM(_S("Field weakening", "Field weak"), &ui_vars.ui8_field_weakening_feature_enabled, "disable", "enable"),
#endif
	FIELD_END };

#ifdef SW102
static Field torqueSensorMenus[] =
{
	FIELD_EDITABLE_ENUM(_S("Assist w/o pedal", "A w/o ped"), &ui_vars.ui8_motor_assistance_startup_without_pedal_rotation, "disable", "enable"),
	FIELD_EDITABLE_UINT(_S("Torque ADC threshold", "Torque thr"), &ui_vars.ui8_torque_sensor_adc_threshold, "", 5, 100),
	//FIELD_EDITABLE_ENUM(_S("Coast brake", "Coast brk"), &ui_vars.ui8_coast_brake_enable, "disable", "enable"),
	FIELD_EDITABLE_UINT(_S("Coast brake ADC", "Coast ADC"), &ui_vars.ui8_coast_brake_adc, "", 5, 50),
	FIELD_END };
	
static Field torqueCalibrationMenus[] =
{
	FIELD_EDITABLE_ENUM(_S("Calibration", "Calibrat"), &ui_vars.ui8_torque_sensor_calibration_feature_enabled, "disable", "enable"),
	FIELD_EDITABLE_UINT(_S("Torque adc step", "ADC step"), &ui_vars.ui8_pedal_torque_per_10_bit_ADC_step_x100, "", 20, 120),      
	FIELD_EDITABLE_UINT(_S("Torque adc step adv", "ADC s adv"), &ui_vars.ui8_pedal_torque_per_10_bit_ADC_step_adv_x100, "", 20, 50),
	FIELD_EDITABLE_UINT(_S("Torque offset adj", "OffsetAdj"), &ui_vars.ui8_adc_pedal_torque_offset_adj, "", 0, 34),
	FIELD_EDITABLE_UINT(_S("Torque range adj", "RangeAdj"), &ui_vars.ui8_adc_pedal_torque_range_adj, "", 0, 40),
	FIELD_EDITABLE_UINT(_S("Torque angle adj", "AngleAdj"), &ui_vars.ui8_adc_pedal_torque_angle_adj_index, "", 0, 40),
	FIELD_EDITABLE_UINT(_S("Torque adc offset", "ADCoffset"), &ui_vars.ui16_adc_pedal_torque_offset, "", 0, 300),
	FIELD_EDITABLE_UINT(_S("Torque adc max", "ADC max"), &ui_vars.ui16_adc_pedal_torque_max, "", 0, 500),
	FIELD_EDITABLE_UINT(_S("Weight on pedal", "Weight"), &ui_vars.ui8_weight_on_pedal, "kg", 20, 80),
	FIELD_EDITABLE_UINT(_S("Torque adc on weight", "ADC weight"), &ui_vars.ui16_adc_pedal_torque_with_weight, "", 100, 500),
	FIELD_READONLY_UINT(_S("ADC torque step calc", "ADC step c"), &ui_vars.ui8_pedal_torque_ADC_step_calc_x100, ""),
	FIELD_EDITABLE_ENUM(_S("Default weight", "Set weight"), &ui8_g_configuration_set_default_weight, "no", "yes"),
	FIELD_END };
#else
static Field torqueSensorMenus[] =
{
	FIELD_EDITABLE_ENUM(_S("Assist w/o pedal", "A w/o ped"), &ui_vars.ui8_motor_assistance_startup_without_pedal_rotation, "disable", "enable"),
	FIELD_EDITABLE_UINT(_S("Torque ADC threshold", "Torque thr"), &ui_vars.ui8_torque_sensor_adc_threshold, "", 5, 100),
	FIELD_EDITABLE_ENUM(_S("Coast brake", "Coast brk"), &ui_vars.ui8_coast_brake_enable, "disable", "enable"),
	FIELD_EDITABLE_UINT(_S("Coast brake ADC", "Coast ADC"), &ui_vars.ui8_coast_brake_adc, "", 5, 50),
	FIELD_EDITABLE_ENUM(_S("Calibration", "Calibrat"), &ui_vars.ui8_torque_sensor_calibration_feature_enabled, "disable", "enable"),
	FIELD_EDITABLE_UINT(_S("Torque adc step", "ADC step"), &ui_vars.ui8_pedal_torque_per_10_bit_ADC_step_x100, "", 20, 120),
	FIELD_EDITABLE_UINT(_S("Torque adc step adv", "ADC s adv"), &ui_vars.ui8_pedal_torque_per_10_bit_ADC_step_adv_x100, "", 20, 50),
	FIELD_EDITABLE_UINT(_S("Torque offset adj", "OffsetAdj"), &ui_vars.ui8_adc_pedal_torque_offset_adj, "", 0, 34),
	FIELD_EDITABLE_UINT(_S("Torque range adj", "RangeAdj"), &ui_vars.ui8_adc_pedal_torque_range_adj, "", 0, 40),
	FIELD_EDITABLE_UINT(_S("Torque angle adj", "AngleAdj"), &ui_vars.ui8_adc_pedal_torque_angle_adj_index, "", 0, 40),
	FIELD_EDITABLE_UINT(_S("Torque adc offset", "ADCoffset"), &ui_vars.ui16_adc_pedal_torque_offset, "", 0, 300),
	FIELD_EDITABLE_UINT(_S("Torque adc max", "ADC max"), &ui_vars.ui16_adc_pedal_torque_max, "", 0, 500),
	FIELD_EDITABLE_UINT(_S("Weight on pedal", "Weight"), &ui_vars.ui8_weight_on_pedal, "kg", 20, 80),
	FIELD_EDITABLE_UINT(_S("Torque adc on weight", "ADC weight"), &ui_vars.ui16_adc_pedal_torque_with_weight, "", 100, 500),
	FIELD_READONLY_UINT(_S("ADC torque step calc", "ADC step c"), &ui_vars.ui8_pedal_torque_ADC_step_calc_x100, ""),
	FIELD_EDITABLE_ENUM(_S("Default weight", "Set weight"), &ui8_g_configuration_set_default_weight, "no", "yes"),
	FIELD_END };
#endif

static Field powerAssistMenus[] =
{
	FIELD_EDITABLE_UINT("Level 1", &ui_vars.ui8_assist_level_factor[0][0], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 2", &ui_vars.ui8_assist_level_factor[0][1], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 3", &ui_vars.ui8_assist_level_factor[0][2], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 4", &ui_vars.ui8_assist_level_factor[0][3], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 5", &ui_vars.ui8_assist_level_factor[0][4], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 6", &ui_vars.ui8_assist_level_factor[0][5], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 7", &ui_vars.ui8_assist_level_factor[0][6], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 8", &ui_vars.ui8_assist_level_factor[0][7], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 9", &ui_vars.ui8_assist_level_factor[0][8], "", 1, 254, .div_digits = 0),
	FIELD_END };
	
static Field torqueAssistMenus[] =
{
	FIELD_EDITABLE_UINT("Level 1", &ui_vars.ui8_assist_level_factor[1][0], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 2", &ui_vars.ui8_assist_level_factor[1][1], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 3", &ui_vars.ui8_assist_level_factor[1][2], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 4", &ui_vars.ui8_assist_level_factor[1][3], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 5", &ui_vars.ui8_assist_level_factor[1][4], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 6", &ui_vars.ui8_assist_level_factor[1][5], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 7", &ui_vars.ui8_assist_level_factor[1][6], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 8", &ui_vars.ui8_assist_level_factor[1][7], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 9", &ui_vars.ui8_assist_level_factor[1][8], "", 1, 254, .div_digits = 0),
	FIELD_END };
	
static Field cadenceAssistMenus[] =
{
	FIELD_EDITABLE_UINT("Level 1", &ui_vars.ui8_assist_level_factor[2][0], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 2", &ui_vars.ui8_assist_level_factor[2][1], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 3", &ui_vars.ui8_assist_level_factor[2][2], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 4", &ui_vars.ui8_assist_level_factor[2][3], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 5", &ui_vars.ui8_assist_level_factor[2][4], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 6", &ui_vars.ui8_assist_level_factor[2][5], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 7", &ui_vars.ui8_assist_level_factor[2][6], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 8", &ui_vars.ui8_assist_level_factor[2][7], "", 1, 254, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 9", &ui_vars.ui8_assist_level_factor[2][8], "", 1, 254, .div_digits = 0),
	FIELD_END };
	
static Field eMTBAssistMenus[] =
{
	FIELD_EDITABLE_UINT("Level 1", &ui_vars.ui8_assist_level_factor[3][0], "", 1, 20, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 2", &ui_vars.ui8_assist_level_factor[3][1], "", 1, 20, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 3", &ui_vars.ui8_assist_level_factor[3][2], "", 1, 20, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 4", &ui_vars.ui8_assist_level_factor[3][3], "", 1, 20, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 5", &ui_vars.ui8_assist_level_factor[3][4], "", 1, 20, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 6", &ui_vars.ui8_assist_level_factor[3][5], "", 1, 20, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 7", &ui_vars.ui8_assist_level_factor[3][6], "", 1, 20, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 8", &ui_vars.ui8_assist_level_factor[3][7], "", 1, 20, .div_digits = 0),
	FIELD_EDITABLE_UINT("Level 9", &ui_vars.ui8_assist_level_factor[3][8], "", 1, 20, .div_digits = 0),
	FIELD_END };

	static Field assistMenus[] =
{
	FIELD_EDITABLE_UINT(_S("Num assist levels", "Num Levels"), &ui_vars.ui8_number_of_assist_levels, "", 1, 9),
	FIELD_SCROLLABLE(_S("Power assist", "Power"), powerAssistMenus),
	FIELD_SCROLLABLE(_S("Torque assist", "Torque"), torqueAssistMenus),
	FIELD_SCROLLABLE(_S(" Cadence assist", "Cadence"), cadenceAssistMenus),
	FIELD_SCROLLABLE(_S("eMTB assist", "eMTB"), eMTBAssistMenus),
	FIELD_END };
	
static Field walkAssistMenus[] =
{
	FIELD_EDITABLE_ENUM("Feature", &ui_vars.ui8_walk_assist_feature_enabled, "disable", "enable"),
	FIELD_EDITABLE_UINT("Speed 1", &ui_vars.ui8_walk_assist_level_factor[0], "kph", 0, 60, .div_digits = 1, .inc_step = 1),
	FIELD_EDITABLE_UINT("Speed 2", &ui_vars.ui8_walk_assist_level_factor[1], "kph", 0, 60, .div_digits = 1, .inc_step = 1),
	FIELD_EDITABLE_UINT("Speed 3", &ui_vars.ui8_walk_assist_level_factor[2], "kph", 0, 60, .div_digits = 1, .inc_step = 1),
	FIELD_EDITABLE_UINT("Speed 4", &ui_vars.ui8_walk_assist_level_factor[3], "kph", 0, 60, .div_digits = 1, .inc_step = 1),
	FIELD_EDITABLE_UINT("Speed 5", &ui_vars.ui8_walk_assist_level_factor[4], "kph", 0, 60, .div_digits = 1, .inc_step = 1),
	FIELD_EDITABLE_UINT("Speed 6", &ui_vars.ui8_walk_assist_level_factor[5], "kph", 0, 60, .div_digits = 1, .inc_step = 1),
	FIELD_EDITABLE_UINT("Speed 7", &ui_vars.ui8_walk_assist_level_factor[6], "kph", 0, 60, .div_digits = 1, .inc_step = 1),
	FIELD_EDITABLE_UINT("Speed 8", &ui_vars.ui8_walk_assist_level_factor[7], "kph", 0, 60, .div_digits = 1, .inc_step = 1),
	FIELD_EDITABLE_UINT("Speed 9", &ui_vars.ui8_walk_assist_level_factor[8], "kph", 0, 60, .div_digits = 1, .inc_step = 1),
	FIELD_EDITABLE_ENUM(_S("Cruise feature", "Cruise fe"), &ui_vars.ui8_cruise_feature_enabled, "disable", "enable"),
	FIELD_END };

static Field startupPowerMenus[] =
{
	FIELD_EDITABLE_ENUM("Feature", &ui_vars.ui8_startup_motor_power_boost_feature_enabled, "disable", "enable"), // FIXME, share one array of disable/enable strings
	FIELD_EDITABLE_UINT(_S("Boost torque factor", "Boost fact"), &ui_vars.ui16_startup_boost_torque_factor, "%", 1, 500, .div_digits = 0),
	FIELD_EDITABLE_UINT(_S("Boost cadence step", "Boost step"), &ui_vars.ui8_startup_boost_cadence_step, "", 10, 50, .div_digits = 0),
	FIELD_EDITABLE_ENUM(_S("Boost at zero", "Boost zero"), &ui_vars.ui8_startup_boost_at_zero, "cadence", "speed"),
#ifndef SW102
	FIELD_EDITABLE_ENUM(_S("Startup assist", "Start ass"), &ui_vars.ui8_startup_assist_feature_enabled, "disable", "enable"),
#endif
	FIELD_END };

static Field motorTempMenus[] =
{
	FIELD_EDITABLE_ENUM("Feature", &ui_vars.ui8_optional_ADC_function, "disable", "temperature", "throttle"),
	FIELD_EDITABLE_UINT("Min limit", &ui_vars.ui8_motor_temperature_min_value_to_limit, "C", 0, 255),
	FIELD_EDITABLE_UINT("Max limit", &ui_vars.ui8_motor_temperature_max_value_to_limit, "C", 0, 255),
	FIELD_EDITABLE_ENUM("Units", &ui_vars.ui8_screen_temperature, "auto", "celsius", "farenheit"),
	//FIELD_EDITABLE_ENUM(_S("Sensor type", "Sens type"), &ui_vars.ui8_temperature_sensor_type, "LM35", "TMP36"),
	FIELD_END };

static Field streetModeMenus[] =
{
	//FIELD_EDITABLE_ENUM("Feature", &ui_vars.ui8_street_mode_function_enabled, "disable", "enable"),
	FIELD_EDITABLE_ENUM(_S("Enable Mode", "Enabl Mode"), &ui_vars.ui8_street_mode_enabled, "no", "yes"),
	FIELD_EDITABLE_ENUM(_S("Enable at startup", "Enabl stup"), &ui_vars.ui8_street_mode_enabled_on_startup, "no", "yes"),
	FIELD_EDITABLE_UINT(_S("Speed limit", "Speed limt"), &ui_vars.ui8_street_mode_speed_limit, "kph", 1, 99, .div_digits = 0, .inc_step = 1, .hide_fraction = true),
	FIELD_EDITABLE_UINT(_S("Motor power limit", "Power limt"), &ui_vars.ui16_street_mode_power_limit, "watts", 25, 2500, .div_digits = 0, .inc_step = 25, .hide_fraction = true),
	FIELD_EDITABLE_ENUM(_S("Throttle enable", "Throt enab"), &ui_vars.ui8_street_mode_throttle_enabled, "no", "yes", "legal"),
	FIELD_EDITABLE_ENUM(_S("Cruise enable", "Cruise ena"), &ui_vars.ui8_street_mode_cruise_enabled, "no", "yes", "legal"),
	FIELD_EDITABLE_ENUM(_S("Hotkey enable", "HotKy enab"), &ui_vars.ui8_street_mode_hotkey_enabled, "no", "yes"),
    FIELD_END };

static Field displayMenus[] =
{
#ifndef SW102
	FIELD_EDITABLE_ENUM("Clock field", &ui_vars.ui8_time_field_enable, "disable", "clock", "batt SOC %", "batt volts"),
	FIELD_EDITABLE_UINT("Clock hours", &ui8_g_configuration_clock_hours, "", 0, 23, .onSetEditable = onSetConfigurationClockHours),
	FIELD_EDITABLE_UINT("Clock minutes", &ui8_g_configuration_clock_minutes, "", 0, 59, .onSetEditable = onSetConfigurationClockMinutes),
	FIELD_EDITABLE_UINT("Brightness on", &ui_vars.ui8_lcd_backlight_on_brightness, "", 5, 100, .inc_step = 5, .onSetEditable = onSetConfigurationDisplayLcdBacklightOnBrightness),
	FIELD_EDITABLE_UINT("Brightness off", &ui_vars.ui8_lcd_backlight_off_brightness, "", 5, 100, .inc_step = 5, .onSetEditable = onSetConfigurationDisplayLcdBacklightOffBrightness),
	FIELD_EDITABLE_ENUM("Buttons invert", &ui_vars.ui8_buttons_up_down_invert, "default", "invert"),
	FIELD_EDITABLE_ENUM(_S("Config shortcut key", "Short key"), &ui_vars.ui8_config_shortcut_key_enabled, "no", "yes"),
#endif
	FIELD_EDITABLE_UINT(_S("Auto power off", "Auto p off"), &ui_vars.ui8_lcd_power_off_time_minutes, "mins", 0, 255),
	FIELD_EDITABLE_ENUM("Units", &ui_vars.ui8_units_type, "SI", "Imperial"),
#ifndef SW102
	FIELD_READONLY_ENUM("LCD type", &g_lcd_ic_type, "ILI9481", "ST7796", "unknown"),
#else
	FIELD_EDITABLE_ENUM(_S("Reset BLE connections", "Reset BLE"), &ui8_g_configuration_display_reset_bluetooth_peers, "no", "yes"),
#endif
	FIELD_EDITABLE_ENUM(_S("Reset to defaults", "Reset def"), &ui8_g_configuration_display_reset_to_defaults, "no", "yes"),
	FIELD_END };

	
static Field variousMenus[] =
{
	FIELD_EDITABLE_UINT(_S("Lights configuration", "Light conf"), &ui_vars.ui8_lights_configuration, "", 0, 8),
	FIELD_EDITABLE_ENUM(_S("Assist with error", "As with er"), &ui_vars.ui8_assist_whit_error_enabled, "no", "yes"),
    FIELD_EDITABLE_UINT(_S("Virtual throttle step", "V thr step"), &ui_vars.ui8_throttle_virtual_step, "", 1, 100),
    FIELD_EDITABLE_UINT("Odometer", &ui_vars.ui32_odometer_x10, "km", 0, UINT32_MAX, .div_digits = 1, .inc_step = 100, .onSetEditable = onSetConfigurationWheelOdometer),
#ifndef SW102
	FIELD_EDITABLE_ENUM("A service", &ui_vars.ui8_service_a_distance_enable, "disable", "enable"),
	FIELD_EDITABLE_UINT("A service distance", &ui_vars.ui16_service_a_distance, "km", 0, 10000, .div_digits = 0, .inc_step = 10, .onSetEditable = onSetConfigurationServiceDistance),
	FIELD_EDITABLE_ENUM("B service", &ui_vars.ui8_service_b_hours_enable, "disable", "enable"),
	FIELD_EDITABLE_UINT("B service hours", &ui_vars.ui16_service_b_hours, "hrs", 0, 10000, .div_digits = 0, .inc_step = 10, .onSetEditable = onSetConfigurationServiceHours),
#endif
	//FIELD_READONLY_UINT(_S("OSF motor     v20.1C", "Mot v20.1C"), &g_tsdz2_firmware_version.patch, "", false, .div_digits = 1),
	FIELD_END };

#ifndef SW102

static Field varSpeedMenus[] =
{
	FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsWheelSpeed].auto_max_min, "auto", "man", "semi"),
	FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsWheelSpeed].max, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsWheelSpeed].min, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsWheelSpeed].auto_thresholds, "disabled", "manual", "auto"),
	FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsWheelSpeed].config_error_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsWheelSpeed].config_warn_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varTripDistanceMenus[] =
{
	FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsTripDistance].auto_max_min, "yes", "no"),
	FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsTripDistance].max, "km", 0, INT32_MAX, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsTripDistance].min, "km", 0, INT32_MAX, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsTripDistance].auto_thresholds, "disabled", "manual", "auto"),
	FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsTripDistance].config_error_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsTripDistance].config_warn_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varCadenceMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsCadence].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsCadence].max, "", 0, 200, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsCadence].min, "", 0, 200, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsCadence].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsCadence].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsCadence].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varHumanPowerMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsHumanPower].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsHumanPower].max, "", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsHumanPower].min, "", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsHumanPower].auto_thresholds, "disabled", "manual"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsHumanPower].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsHumanPower].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varBatteryPowerMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsBatteryPower].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryPower].max, "", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryPower].min, "", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryPower].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsBatteryPower].config_error_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsBatteryPower].config_warn_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
	FIELD_END };

static Field varBatteryPowerUsageMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsBatteryPowerUsage].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryPowerUsage].max, "Wh/km", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryPowerUsage].min, "Wh/km", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryPowerUsage].auto_thresholds, "disabled", "manual"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsBatteryPowerUsage].config_error_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsBatteryPowerUsage].config_warn_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
	FIELD_END };

static Field varBatteryVoltageMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsBatteryVoltage].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryVoltage].max, "", 0, 1000, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryVoltage].min, "", 0, 1000, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryVoltage].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsBatteryVoltage].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsBatteryVoltage].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varBatteryCurrentMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsBatteryCurrent].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryCurrent].max, "", 0, 50, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryCurrent].min, "", 0, 50, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryCurrent].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsBatteryCurrent].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsBatteryCurrent].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varMotorCurrentMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsMotorCurrent].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorCurrent].max, "", 0, 50, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorCurrent].min, "", 0, 50, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorCurrent].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsMotorCurrent].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsMotorCurrent].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varBatterySOCMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsBatterySOC].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatterySOC].max, "", 0, 100, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatterySOC].min, "", 0, 100, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatterySOC].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsBatterySOC].config_error_threshold, "", 0, 200, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsBatterySOC].config_warn_threshold, "", 0, 200, .div_digits = 1, .inc_step = 1),
	FIELD_END };

static Field varMotorTempMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsMotorTemp].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorTemp].max, "C", 0, 200, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorTemp].min, "C", 0, 200, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorTemp].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsMotorTemp].config_error_threshold, "C", 0, 200, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsMotorTemp].config_warn_threshold, "C", 0, 200, .div_digits = 1, .inc_step = 1),
	FIELD_END };

static Field varMotorERPSMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsMotorERPS].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorERPS].max, "", 0, 2000, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorERPS].min, "", 0, 2000, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorERPS].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsMotorERPS].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsMotorERPS].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 1),
	FIELD_END };

static Field varMotorPWMMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsMotorPWM].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorPWM].max, "", 0, 255, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorPWM].min, "", 0, 255, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorPWM].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsMotorPWM].config_error_threshold, "", 0, 500, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsMotorPWM].config_warn_threshold, "", 0, 500, .div_digits = 1, .inc_step = 1),
	FIELD_END };

static Field varMotorFOCMenus[] =
{
    FIELD_EDITABLE_ENUM(_S("Graph auto max min", "G auto m n"), &g_graphVars[VarsMotorFOC].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorFOC].max, "", 0, 60, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorFOC].min, "", 0, 60, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorFOC].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT(_S("Max threshold", "Max thresh"), &g_vars[VarsMotorFOC].config_error_threshold, "", 0, 120, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT(_S("Min threshold", "Min thresh"), &g_vars[VarsMotorFOC].config_warn_threshold, "", 0, 120, .div_digits = 1, .inc_step = 1),
	FIELD_END };

static Field variablesMenus[] =
{
	FIELD_SCROLLABLE("Speed", varSpeedMenus),
	FIELD_SCROLLABLE(_S("Trip distance", "Trip dist"), varTripDistanceMenus),
	FIELD_SCROLLABLE("Cadence", varCadenceMenus),
	FIELD_SCROLLABLE(_S("human power", "human powr"), varHumanPowerMenus),
	FIELD_SCROLLABLE(_S("motor power", "motor powr"), varBatteryPowerMenus),
	FIELD_SCROLLABLE(_S("Watts/km", "Watts/km"), varBatteryPowerUsageMenus),
	FIELD_SCROLLABLE(_S("batt voltage", "bat volts"), varBatteryVoltageMenus),
	FIELD_SCROLLABLE(_S("batt current", "bat curren"), varBatteryCurrentMenus),
	FIELD_SCROLLABLE(_S("battery SOC", "bat SOC"), varBatterySOCMenus),
	//FIELD_SCROLLABLE(_S("motor current", "mot curren"), varMotorCurrentMenus),
	FIELD_SCROLLABLE(_S("motor temp", "mot temp"), varMotorTempMenus),
	FIELD_SCROLLABLE(_S("motor speed", "mot speed"), varMotorERPSMenus),
	FIELD_SCROLLABLE(_S("motor pwm", "mot pwm"), varMotorPWMMenus),
	FIELD_SCROLLABLE(_S("motor foc", "mot foc"), varMotorFOCMenus),
	FIELD_END };
#endif

static Field technicalMenus[] =
{
	FIELD_READONLY_UINT(_S("ADC battery current", "ADC bat cu"), &ui_vars.ui16_adc_battery_current, ""),
	FIELD_READONLY_UINT(_S("ADC throttle sensor", "ADC thrott"), &ui_vars.ui8_adc_throttle, ""),
	FIELD_READONLY_UINT(_S("Throttle sensor", "Throttle s"), &ui_vars.ui8_throttle, ""),
	FIELD_READONLY_UINT(_S("ADC torque sensor", "ADC torque"), &ui_vars.ui16_adc_pedal_torque_sensor, ""),
	FIELD_READONLY_UINT(_S("ADC torque delta", "ADC delta"), &ui_vars.ui16_adc_pedal_torque_delta, ""),
	FIELD_READONLY_UINT(_S("ADC torque boost", "ADC boost"), &ui_vars.ui16_adc_pedal_torque_delta_boost, ""),
	//FIELD_READONLY_UINT(_S("ADC torque step calc", "ADC step c"), &ui_vars.ui8_pedal_torque_ADC_step_calc_x100, ""),
	FIELD_READONLY_UINT(_S("Pedal cadence", "Cadence"), &ui_vars.ui8_pedal_cadence, "rpm"),
	FIELD_READONLY_UINT(_S("PWM duty-cycle", "PWM duty"), &ui_vars.ui8_duty_cycle, ""),
	FIELD_READONLY_UINT(_S("Motor speed", "Mot speed"), &ui_vars.ui16_motor_speed_erps, ""),
	FIELD_READONLY_UINT("Motor FOC", &ui_vars.ui8_foc_angle, ""),
	FIELD_READONLY_UINT(_S("Hall sensors", "Hall sens"), &ui_vars.ui8_motor_hall_sensors, ""),
	FIELD_END };

static Field topMenus[] =
{
	FIELD_SCROLLABLE("Trip memories", tripMenus),
	FIELD_SCROLLABLE("Wheel", wheelMenus),
	FIELD_SCROLLABLE("Battery", batteryMenus),
	FIELD_SCROLLABLE("SOC", batterySOCMenus),
	FIELD_SCROLLABLE(_S("Motor", "Motor"), motorMenus),
	FIELD_SCROLLABLE(_S("Torque sensor", "Torque sen"), torqueSensorMenus),
#ifdef SW102
	FIELD_SCROLLABLE(_S("Torque calibr", "Torque cal"), torqueCalibrationMenus),
#endif
	FIELD_SCROLLABLE(_S("Assist level", "Assist"), assistMenus),
	FIELD_SCROLLABLE(_S("Walk assist", "Walk"), walkAssistMenus),
	FIELD_SCROLLABLE(_S("Startup BOOST", "StartBOOST"), startupPowerMenus),
	FIELD_SCROLLABLE(_S("Motor temperature", "Motor temp"), motorTempMenus),
	FIELD_SCROLLABLE(_S("Street mode", "Street mod"), streetModeMenus),
#ifndef SW102
	FIELD_SCROLLABLE("Variables", variablesMenus),
#endif
	FIELD_SCROLLABLE("Various", variousMenus),
	FIELD_SCROLLABLE("Display", displayMenus),
	FIELD_SCROLLABLE("Technical", technicalMenus),
	FIELD_END };

static Field configRoot = FIELD_SCROLLABLE(_S("Configurations", "Config"), topMenus);

uint8_t ui8_g_configuration_display_reset_to_defaults = 0;
uint32_t ui32_g_configuration_wh_100_percent = 0;
uint8_t ui8_g_configuration_display_reset_bluetooth_peers = 0;
uint8_t ui8_g_configuration_trip_a_reset = 0;
uint8_t ui8_g_configuration_trip_b_reset = 0;
uint8_t ui8_g_configuration_battery_soc_reset = 0;
uint8_t ui8_g_configuration_set_default_weight = 0;

static void configScreenOnEnter() {
	// Set the font preference for this screen
	editable_label_font = &CONFIGURATIONS_TEXT_FONT;
	editable_value_font = &CONFIGURATIONS_TEXT_FONT;
	editable_units_font = &CONFIGURATIONS_TEXT_FONT;
}

static void configExit() {
  prepare_torque_sensor_calibration_table();

	// save the variables on EEPROM
	eeprom_write_variables();
	set_conversions(); // we just changed units

  update_battery_power_usage_label();

	// send the configurations to TSDZ2
  if (g_motor_init_state == MOTOR_INIT_READY)
    g_motor_init_state = MOTOR_INIT_SET_CONFIGURATIONS;
}

static void configPreUpdate() {
  set_conversions(); // while in the config menu we might change units at any time - keep the display looking correct
}

//
// Screens
//
Screen configScreen = {
    .onExit = configExit,
    .onEnter = configScreenOnEnter,
    .onPreUpdate = configPreUpdate,

.fields = {
		{ .color = ColorNormal, .field = &configRoot },
		{ .field = NULL } } };
