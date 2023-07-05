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
	FIELD_EDITABLE_ENUM("Reset trip A", &ui8_g_configuration_trip_a_reset, "no", "yes"),
	FIELD_EDITABLE_ENUM("Reset trip B", &ui8_g_configuration_trip_b_reset, "no", "yes"),
#else
	FIELD_EDITABLE_ENUM("Rst trip A", &ui8_g_configuration_trip_a_reset, "no", "yes"),
	FIELD_EDITABLE_ENUM("Rst trip B", &ui8_g_configuration_trip_b_reset, "no", "yes"),
#endif
/*
#ifndef SW102
	FIELD_READONLY_UINT("Used Wh trip A", &ui_vars.ui32_wh_x10_trip_a, "whr", false, .div_digits = 1),
	FIELD_READONLY_UINT("Used Wh trip B", &ui_vars.ui32_wh_x10_trip_b, "whr", false, .div_digits = 1),
#endif
*/
	FIELD_END };

static Field bikeMenus[] =
{
	FIELD_EDITABLE_UINT("Max speed", &ui_vars.ui8_wheel_max_speed, "kph", 1, 99, .div_digits = 0, .inc_step = 1, .hide_fraction = true),
#ifndef SW102
	FIELD_EDITABLE_UINT("Circumference", &ui_vars.ui16_wheel_perimeter, "mm", 750, 3000, .inc_step = 10),
	FIELD_EDITABLE_UINT("Max power limit", &ui_vars.ui16_motor_power_limit, "watts", 25, 1000, .div_digits = 0, .inc_step = 25, .hide_fraction = true),
	FIELD_EDITABLE_ENUM("Assist with error", &ui_vars.ui8_assist_whit_error_enabled, "no", "yes"),
	FIELD_EDITABLE_ENUM("Throttle", &ui_vars.ui8_throttle_feature_enabled, "disable", "pedaling", "6km/h only", "6km/h & ped", "unconditional"),
	FIELD_EDITABLE_ENUM("Cruise", &ui_vars.ui8_cruise_feature_enabled, "disable", "pedaling", "unconditional"),
	FIELD_EDITABLE_ENUM("Password enable", &ui_vars.ui8_password_enabled, "no", "yes"),
#else
	FIELD_EDITABLE_UINT("Circumfere", &ui_vars.ui16_wheel_perimeter, "mm", 750, 3000, .inc_step = 10),
	FIELD_EDITABLE_UINT("Power limt", &ui_vars.ui16_motor_power_limit, "watts", 25, 1000, .div_digits = 0, .inc_step = 25, .hide_fraction = true),
	FIELD_EDITABLE_ENUM("As with er", &ui_vars.ui8_assist_whit_error_enabled, "no", "yes"),
	FIELD_EDITABLE_ENUM("Throttle", &ui_vars.ui8_throttle_feature_enabled, "disable", "pedaling", "6km/h only", "6km/h&ped" "w/o pedal"),
	FIELD_EDITABLE_ENUM("Cruise", &ui_vars.ui8_cruise_feature_enabled, "disable", "pedaling", "w/o pedal"),
	FIELD_EDITABLE_ENUM("Psw enable", &ui_vars.ui8_password_enabled, "no", "yes"),
#endif
	FIELD_EDITABLE_UINT("Password", &ui_vars.ui16_entered_password, "", 1000, 9999),
	FIELD_EDITABLE_ENUM("Confirm", &ui_vars.ui8_confirm_password, "logout", "login", "wait", "change"),
	FIELD_EDITABLE_ENUM("Reset", &ui_vars.ui8_reset_password, "no", "yes"),
	//FIELD_EDITABLE_ENUM("Wait", &ui_vars.ui8_wait_confirm_password, "no", "yes"),
	//FIELD_EDITABLE_ENUM("Confirmed", &ui_vars.ui8_password_confirmed, "no", "yes"),
	//FIELD_EDITABLE_ENUM("First time", &ui_vars.ui8_password_first_time, "no", "yes"),
	//FIELD_EDITABLE_ENUM("Changed", &ui_vars.ui8_password_changed, "no", "yes"),
	//FIELD_EDITABLE_UINT("Password", &ui_vars.ui16_saved_password, "", 1000, 9999),
	FIELD_END };

static Field batteryMenus[] =
{
#ifndef SW102
	FIELD_EDITABLE_UINT("Max current", &ui_vars.ui8_battery_max_current, "amps", 1, 18),
	FIELD_EDITABLE_UINT("Low cut-off", &ui_vars.ui16_battery_low_voltage_cut_off_x10, "volts", 160, 630, .div_digits = 1),
	FIELD_EDITABLE_UINT("Voltage cal %", &ui_vars.ui16_battery_voltage_calibrate_percent_x10, "volts", 950, 1050, .div_digits = 1),
	FIELD_EDITABLE_UINT("Resistance", &ui_vars.ui16_battery_pack_resistance_x1000, "mohm", 0, 1000),
	FIELD_READONLY_UINT("Voltage est", &ui_vars.ui16_battery_voltage_soc_x10, "volts", false, .div_digits = 1),
	FIELD_READONLY_UINT("Resistance est", &ui_vars.ui16_battery_pack_resistance_estimated_x1000, "mohm", 0, 1000),
	FIELD_READONLY_UINT("Power loss est", &ui_vars.ui16_battery_power_loss, "watts", false, .div_digits = 0),
#else
	FIELD_EDITABLE_UINT("Max curren", &ui_vars.ui8_battery_max_current, "amps", 1, 18),
	FIELD_EDITABLE_UINT("Lo cut-off", &ui_vars.ui16_battery_low_voltage_cut_off_x10, "volts", 160, 630, .div_digits = 1),
	FIELD_EDITABLE_UINT("Voltag cal", &ui_vars.ui16_battery_voltage_calibrate_percent_x10, "volts", 950, 1050, .div_digits = 1),
	FIELD_EDITABLE_UINT("Resistance", &ui_vars.ui16_battery_pack_resistance_x1000, "mohm", 0, 1000),
	FIELD_READONLY_UINT("Voltag est", &ui_vars.ui16_battery_voltage_soc_x10, "volts", false, .div_digits = 1),
	FIELD_READONLY_UINT("Resist est", &ui_vars.ui16_battery_pack_resistance_estimated_x1000, "mohm", 0, 1000),
	FIELD_READONLY_UINT("Power loss", &ui_vars.ui16_battery_power_loss, "watts", false, .div_digits = 0),
#endif
	FIELD_END };

static Field batterySOCMenus[] =
{
#ifndef SW102
	FIELD_EDITABLE_ENUM("Text", &ui_vars.ui8_battery_soc_enable, "disable", "SOC %", "volts"),
	FIELD_EDITABLE_ENUM("Calculation", &ui_vars.ui8_battery_soc_percent_calculation, "auto", "Wh", "volts"),
	FIELD_EDITABLE_UINT("Reset at voltage", &ui_vars.ui16_battery_voltage_reset_wh_counter_x10, "volts", 160, 680, .div_digits = 1),
	FIELD_EDITABLE_UINT("Battery total Wh", &ui_vars.ui32_wh_x10_100_percent, "whr", 0, 29990, .div_digits = 1, .inc_step = 100),
	FIELD_EDITABLE_UINT("Used Wh", &ui_vars.ui32_wh_x10, "whr", 0, 99900, .div_digits = 1, .inc_step = 10, .onSetEditable = onSetConfigurationBatterySOCUsedWh),
	FIELD_EDITABLE_ENUM("Manual reset", &ui8_g_configuration_battery_soc_reset, "no", "yes"),
	FIELD_EDITABLE_UINT("Auto reset %", &ui_vars.ui8_battery_soc_auto_reset, "", 0, 100),
	FIELD_READONLY_UINT("Charge cycles", &ui_vars.ui16_battery_charge_cycles_x10, "", .div_digits = 1),
	//FIELD_READONLY_UINT("Used total Wh", &ui_vars.ui32_wh_x10_total, "", .div_digits = 1),
#else
	FIELD_EDITABLE_ENUM("Text", &ui_vars.ui8_battery_soc_enable, "disable", "SOC %", "volts"),
	FIELD_EDITABLE_ENUM("Calculation", &ui_vars.ui8_battery_soc_percent_calculation, "auto", "Wh", "volts"),
	FIELD_EDITABLE_UINT("Reset at", &ui_vars.ui16_battery_voltage_reset_wh_counter_x10, "volts", 160, 680, .div_digits = 1),
	FIELD_EDITABLE_UINT("Batt total", &ui_vars.ui32_wh_x10_100_percent, "whr", 0, 29990, .div_digits = 1, .inc_step = 100),
	FIELD_EDITABLE_UINT("Used Wh", &ui_vars.ui32_wh_x10, "whr", 0, 99900, .div_digits = 1, .inc_step = 10, .onSetEditable = onSetConfigurationBatterySOCUsedWh),
	FIELD_EDITABLE_ENUM("Manual rst", &ui8_g_configuration_battery_soc_reset, "no", "yes"),
	FIELD_EDITABLE_UINT("Auto rst%", &ui_vars.ui8_battery_soc_auto_reset, "", 0, 100),
#endif
	FIELD_END };

static Field motorMenus[] =
{
#ifndef SW102
	FIELD_EDITABLE_ENUM("Motor voltage", &ui_vars.ui8_motor_type, "48V", "36V"),
	FIELD_EDITABLE_UINT("Motor power max", &ui_vars.ui16_target_max_battery_power, "watts", 25, 1000, .div_digits = 0, .inc_step = 25, .hide_fraction = true),
	FIELD_EDITABLE_UINT("Motor acceleration", &ui_vars.ui8_motor_acceleration_adjustment, "%", 0, 100, .div_digits = 0),
	FIELD_EDITABLE_UINT("Motor deceleration", &ui_vars.ui8_motor_deceleration_adjustment, "%", 0, 100, .div_digits = 0),
	FIELD_EDITABLE_ENUM("Field weakening", &ui_vars.ui8_field_weakening_feature_enabled, "disable", "enable"),
#else
	FIELD_EDITABLE_ENUM("Motor volt", &ui_vars.ui8_motor_type, "48V", "36V"),
	FIELD_EDITABLE_UINT("Power max", &ui_vars.ui16_target_max_battery_power, "watts", 25, 1000, .div_digits = 0, .inc_step = 25, .hide_fraction = true),
	FIELD_EDITABLE_UINT("Motor acc", &ui_vars.ui8_motor_acceleration_adjustment, "%", 0, 100, .div_digits = 0),
	FIELD_EDITABLE_UINT("Motor dec", &ui_vars.ui8_motor_deceleration_adjustment, "%", 0, 100, .div_digits = 0),
#endif
	FIELD_END };

#ifndef SW102
static Field torqueSensorMenus[] =
{
	FIELD_EDITABLE_ENUM("Assist w/o pedal", &ui_vars.ui8_motor_assistance_startup_without_pedal_rotation, "disable", "enable"),
	FIELD_EDITABLE_UINT("Torque ADC threshold", &ui_vars.ui8_torque_sensor_adc_threshold, "", 5, 100),
	FIELD_EDITABLE_ENUM("Coast brake", &ui_vars.ui8_coast_brake_enable, "disable", "enable"),
	FIELD_EDITABLE_UINT("Coast brake ADC", &ui_vars.ui8_coast_brake_adc, "", 5, 50),
	FIELD_EDITABLE_ENUM("Calibration", &ui_vars.ui8_torque_sensor_calibration_feature_enabled, "disable", "enable"),
	FIELD_EDITABLE_UINT("Torque adc step", &ui_vars.ui8_pedal_torque_per_10_bit_ADC_step_x100, "", 20, 120),
	FIELD_EDITABLE_UINT("Torque adc step adv", &ui_vars.ui8_pedal_torque_per_10_bit_ADC_step_adv_x100, "", 20, 50),
	FIELD_EDITABLE_UINT("Torque offset adj", &ui_vars.ui8_adc_pedal_torque_offset_adj, "", 0, 34),
	FIELD_EDITABLE_UINT("Torque range adj", &ui_vars.ui8_adc_pedal_torque_range_adj, "", 0, 40),
	FIELD_EDITABLE_UINT("Torque angle adj", &ui_vars.ui8_adc_pedal_torque_angle_adj_index, "", 0, 40),
	FIELD_EDITABLE_UINT("Torque adc offset", &ui_vars.ui16_adc_pedal_torque_offset, "", 0, 300),
	FIELD_EDITABLE_UINT("Torque adc max", &ui_vars.ui16_adc_pedal_torque_max, "", 0, 500),
	FIELD_EDITABLE_UINT("Weight on pedal", &ui_vars.ui8_weight_on_pedal, "kg", 20, 80),
	FIELD_EDITABLE_UINT("Torque adc on weight", &ui_vars.ui16_adc_pedal_torque_with_weight, "", 100, 500),
	FIELD_READONLY_UINT("ADC torque step calc", &ui_vars.ui8_pedal_torque_ADC_step_calc_x100, ""),
	FIELD_EDITABLE_ENUM("Default weight", &ui8_g_configuration_set_default_weight, "no", "yes"),
	FIELD_END };
#else
static Field torqueSensorMenus[] =
{
	FIELD_EDITABLE_ENUM("A w/o ped", &ui_vars.ui8_motor_assistance_startup_without_pedal_rotation, "disable", "enable"),
	FIELD_EDITABLE_UINT("Torque thr", &ui_vars.ui8_torque_sensor_adc_threshold, "", 5, 100),
	//FIELD_EDITABLE_ENUM("Coast brk", &ui_vars.ui8_coast_brake_enable, "disable", "enable"),
	FIELD_EDITABLE_UINT("Coast ADC", &ui_vars.ui8_coast_brake_adc, "", 5, 50),
	FIELD_END };
	
static Field torqueCalibrationMenus[] =
{
	FIELD_EDITABLE_ENUM("Calibrat", &ui_vars.ui8_torque_sensor_calibration_feature_enabled, "disable", "enable"),
	FIELD_EDITABLE_UINT("ADC step", &ui_vars.ui8_pedal_torque_per_10_bit_ADC_step_x100, "", 20, 120),      
	FIELD_EDITABLE_UINT("ADC s adv", &ui_vars.ui8_pedal_torque_per_10_bit_ADC_step_adv_x100, "", 20, 50),
	FIELD_EDITABLE_UINT("OffsetAdj", &ui_vars.ui8_adc_pedal_torque_offset_adj, "", 0, 34),
	FIELD_EDITABLE_UINT("RangeAdj", &ui_vars.ui8_adc_pedal_torque_range_adj, "", 0, 40),
	FIELD_EDITABLE_UINT("AngleAdj", &ui_vars.ui8_adc_pedal_torque_angle_adj_index, "", 0, 40),
	FIELD_EDITABLE_UINT("ADCoffset", &ui_vars.ui16_adc_pedal_torque_offset, "", 0, 300),
	FIELD_EDITABLE_UINT("ADC max", &ui_vars.ui16_adc_pedal_torque_max, "", 0, 500),
	FIELD_EDITABLE_UINT("Weight", &ui_vars.ui8_weight_on_pedal, "kg", 20, 80),
	FIELD_EDITABLE_UINT("ADC weight", &ui_vars.ui16_adc_pedal_torque_with_weight, "", 100, 500),
	FIELD_READONLY_UINT("ADC step c", &ui_vars.ui8_pedal_torque_ADC_step_calc_x100, ""),
	FIELD_EDITABLE_ENUM("Set weight", &ui8_g_configuration_set_default_weight, "no", "yes"),
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
#ifndef SW102
	FIELD_EDITABLE_UINT("Num assist levels", &ui_vars.ui8_number_of_assist_levels, "", 1, 9),
	FIELD_SCROLLABLE("Power assist", powerAssistMenus),
	FIELD_SCROLLABLE("Torque assist", torqueAssistMenus),
	FIELD_SCROLLABLE(" Cadence assist", cadenceAssistMenus),
	FIELD_SCROLLABLE("eMTB assist", eMTBAssistMenus),
#else
	FIELD_EDITABLE_UINT("Num Levels", &ui_vars.ui8_number_of_assist_levels, "", 1, 9),
	FIELD_SCROLLABLE("Power", powerAssistMenus),
	FIELD_SCROLLABLE("Torque", torqueAssistMenus),
	FIELD_SCROLLABLE("Cadence", cadenceAssistMenus),
	FIELD_SCROLLABLE("eMTB", eMTBAssistMenus),
#endif
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
	FIELD_END };

static Field startupPowerMenus[] =
{
	FIELD_EDITABLE_ENUM("Feature", &ui_vars.ui8_startup_motor_power_boost_feature_enabled, "disable", "enable"), // FIXME, share one array of disable/enable strings
#ifndef SW102
	FIELD_EDITABLE_UINT("Boost torque factor", &ui_vars.ui16_startup_boost_torque_factor, "%", 1, 500, .div_digits = 0),
	FIELD_EDITABLE_UINT("Boost cadence step", &ui_vars.ui8_startup_boost_cadence_step, "", 10, 50, .div_digits = 0),
	FIELD_EDITABLE_ENUM("Boost at zero", &ui_vars.ui8_startup_boost_at_zero, "cadence", "speed"),
	FIELD_EDITABLE_ENUM("Startup assist", &ui_vars.ui8_startup_assist_feature_enabled, "disable", "enable"),
#else
	FIELD_EDITABLE_UINT("Boost fact", &ui_vars.ui16_startup_boost_torque_factor, "%", 1, 500, .div_digits = 0),
	FIELD_EDITABLE_UINT("Boost step", &ui_vars.ui8_startup_boost_cadence_step, "", 10, 50, .div_digits = 0),
	FIELD_EDITABLE_ENUM("Boost zero", &ui_vars.ui8_startup_boost_at_zero, "cadence", "speed"),
#endif
	FIELD_END };

static Field motorTempMenus[] =
{
	FIELD_EDITABLE_ENUM("Feature", &ui_vars.ui8_optional_ADC_function, "disable", "temperature", "throttle"),
	FIELD_EDITABLE_UINT("Min limit", &ui_vars.ui8_motor_temperature_min_value_to_limit, "C", 0, 255),
	FIELD_EDITABLE_UINT("Max limit", &ui_vars.ui8_motor_temperature_max_value_to_limit, "C", 0, 255),
	FIELD_EDITABLE_ENUM("Units", &ui_vars.ui8_screen_temperature, "auto", "celsius", "farenheit"),
#ifndef SW102
	FIELD_EDITABLE_ENUM("Sensor type", &ui_vars.ui8_temperature_sensor_type, "LM35", "TMP36"),
#else
	FIELD_EDITABLE_ENUM("Sens type", &ui_vars.ui8_temperature_sensor_type, "LM35", "TMP36"),
#endif
	FIELD_EDITABLE_ENUM("Brake", &ui_vars.ui8_brake_input, "brake", "temperature"),
	FIELD_END };

static Field streetModeMenus[] =
{
#ifndef SW102
	FIELD_EDITABLE_ENUM("Enable Mode", &ui_vars.ui8_street_mode_enabled, "no", "yes"),
	FIELD_EDITABLE_ENUM("Enable at startup", &ui_vars.ui8_street_mode_enabled_on_startup, "no", "yes"),
	FIELD_EDITABLE_UINT("Speed limit", &ui_vars.ui8_street_mode_speed_limit, "kph", 1, 99, .div_digits = 0, .inc_step = 1, .hide_fraction = true),
	FIELD_EDITABLE_UINT("Motor power limit", &ui_vars.ui16_street_mode_power_limit, "watts", 25, 1000, .div_digits = 0, .inc_step = 25, .hide_fraction = true),
	FIELD_EDITABLE_ENUM("Throttle", &ui_vars.ui8_street_mode_throttle_enabled, "disable", "pedaling", "6km/h only", "6km/h & ped", "unconditional"),
	FIELD_EDITABLE_ENUM("Cruise", &ui_vars.ui8_street_mode_cruise_enabled, "disable", "pedaling", "unconditional"),
	FIELD_EDITABLE_ENUM("Hotkey enable", &ui_vars.ui8_street_mode_hotkey_enabled, "no", "yes"),
#else
	FIELD_EDITABLE_ENUM("Enabl Mode", &ui_vars.ui8_street_mode_enabled, "no", "yes"),
	FIELD_EDITABLE_ENUM("Enabl stup", &ui_vars.ui8_street_mode_enabled_on_startup, "no", "yes"),
	FIELD_EDITABLE_UINT("Speed limt", &ui_vars.ui8_street_mode_speed_limit, "kph", 1, 99, .div_digits = 0, .inc_step = 1, .hide_fraction = true),
	FIELD_EDITABLE_UINT("Power limt", &ui_vars.ui16_street_mode_power_limit, "watts", 25, 1000, .div_digits = 0, .inc_step = 25, .hide_fraction = true),
	FIELD_EDITABLE_ENUM("Throttle", &ui_vars.ui8_street_mode_throttle_enabled, "disable", "pedaling", "6km/h only", "6km/h&ped", "w/o pedal"),
	FIELD_EDITABLE_ENUM("Cruise", &ui_vars.ui8_street_mode_cruise_enabled, "disable", "pedaling", "w/o pedal"),
	FIELD_EDITABLE_ENUM("HotKy enab", &ui_vars.ui8_street_mode_hotkey_enabled, "no", "yes"),
#endif
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
	FIELD_EDITABLE_ENUM("Config shortcut key", &ui_vars.ui8_config_shortcut_key_enabled, "no", "yes"),
	FIELD_EDITABLE_UINT("Auto power off", &ui_vars.ui8_lcd_power_off_time_minutes, "mins", 0, 255),
	FIELD_EDITABLE_ENUM("Units", &ui_vars.ui8_units_type, "SI", "Imperial"),
	FIELD_READONLY_ENUM("LCD type", &g_lcd_ic_type, "ILI9481", "ST7796", "unknown"),
	FIELD_EDITABLE_ENUM("Reset to defaults", &ui8_g_configuration_display_reset_to_defaults, "no", "yes"),
	FIELD_EDITABLE_ENUM("Confirm reset", &ui_vars.ui8_confirm_default_reset, "no", "yes"),
#else
	FIELD_EDITABLE_UINT("Auto p off", &ui_vars.ui8_lcd_power_off_time_minutes, "mins", 0, 255),
	FIELD_EDITABLE_ENUM("Units", &ui_vars.ui8_units_type, "SI", "Imperial"),
	FIELD_EDITABLE_ENUM("Reset BLE", &ui8_g_configuration_display_reset_bluetooth_peers, "no", "yes"),
	FIELD_EDITABLE_ENUM("Reset def", &ui8_g_configuration_display_reset_to_defaults, "no", "yes"),
	FIELD_EDITABLE_ENUM("Confirm", &ui_vars.ui8_confirm_default_reset, "no", "yes"),
#endif
	FIELD_END };

	
static Field variousMenus[] =
{
#ifndef SW102
	FIELD_EDITABLE_UINT("Lights configuration", &ui_vars.ui8_lights_configuration, "", 0, 8),
    FIELD_EDITABLE_UINT("Virtual throttle step", &ui_vars.ui8_throttle_virtual_step, "", 1, 100),
    FIELD_EDITABLE_UINT("Odometer", &ui_vars.ui32_odometer_x10, "km", 0, UINT32_MAX, .div_digits = 1, .inc_step = 100, .onSetEditable = onSetConfigurationWheelOdometer),
	FIELD_EDITABLE_ENUM("A service", &ui_vars.ui8_service_a_distance_enable, "disable", "enable"),
	FIELD_EDITABLE_UINT("A service distance", &ui_vars.ui32_service_a_distance, "km", 0, 10000, .div_digits = 0, .inc_step = 10, .onSetEditable = onSetConfigurationServiceDistance),
	FIELD_EDITABLE_ENUM("B service", &ui_vars.ui8_service_b_hours_enable, "disable", "enable"),
	FIELD_EDITABLE_UINT("B service hours", &ui_vars.ui32_service_b_hours, "hrs", 0, 10000, .div_digits = 0, .inc_step = 10, .onSetEditable = onSetConfigurationServiceHours),
#else
	FIELD_EDITABLE_UINT("Light conf", &ui_vars.ui8_lights_configuration, "", 0, 8),
    FIELD_EDITABLE_UINT("V thr step", &ui_vars.ui8_throttle_virtual_step, "", 1, 100),
    FIELD_EDITABLE_UINT("Odometer", &ui_vars.ui32_odometer_x10, "km", 0, UINT32_MAX, .div_digits = 1, .inc_step = 100, .onSetEditable = onSetConfigurationWheelOdometer),
#endif
	FIELD_END };

#ifndef SW102

static Field varSpeedMenus[] =
{
	FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsWheelSpeed].auto_max_min, "auto", "man", "semi"),
	FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsWheelSpeed].max, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsWheelSpeed].min, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsWheelSpeed].auto_thresholds, "disabled", "manual", "auto"),
	FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsWheelSpeed].config_error_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsWheelSpeed].config_warn_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varTripDistanceMenus[] =
{
	FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsTripDistance].auto_max_min, "yes", "no"),
	FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsTripDistance].max, "km", 0, INT32_MAX, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsTripDistance].min, "km", 0, INT32_MAX, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsTripDistance].auto_thresholds, "disabled", "manual", "auto"),
	FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsTripDistance].config_error_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsTripDistance].config_warn_threshold, "km", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varCadenceMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsCadence].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsCadence].max, "", 0, 200, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsCadence].min, "", 0, 200, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsCadence].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsCadence].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsCadence].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varHumanPowerMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsHumanPower].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsHumanPower].max, "", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsHumanPower].min, "", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsHumanPower].auto_thresholds, "disabled", "manual"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsHumanPower].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsHumanPower].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varBatteryPowerMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsBatteryPower].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryPower].max, "", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryPower].min, "", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryPower].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsBatteryPower].config_error_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsBatteryPower].config_warn_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
	FIELD_END };

static Field varBatteryPowerUsageMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsBatteryPowerUsage].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryPowerUsage].max, "Wh/km", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryPowerUsage].min, "Wh/km", 0, 5000, .inc_step = 10),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryPowerUsage].auto_thresholds, "disabled", "manual"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsBatteryPowerUsage].config_error_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsBatteryPowerUsage].config_warn_threshold, "", 0, 2000, .div_digits = 0, .inc_step = 10),
	FIELD_END };

static Field varBatteryVoltageMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsBatteryVoltage].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryVoltage].max, "", 0, 1000, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryVoltage].min, "", 0, 1000, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryVoltage].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsBatteryVoltage].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsBatteryVoltage].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varBatteryCurrentMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsBatteryCurrent].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatteryCurrent].max, "", 0, 50, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatteryCurrent].min, "", 0, 50, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatteryCurrent].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsBatteryCurrent].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsBatteryCurrent].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varMotorCurrentMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsMotorCurrent].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorCurrent].max, "", 0, 50, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorCurrent].min, "", 0, 50, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorCurrent].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsMotorCurrent].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsMotorCurrent].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 10),
	FIELD_END };

static Field varBatterySOCMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsBatterySOC].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsBatterySOC].max, "", 0, 100, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsBatterySOC].min, "", 0, 100, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsBatterySOC].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsBatterySOC].config_error_threshold, "", 0, 200, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsBatterySOC].config_warn_threshold, "", 0, 200, .div_digits = 1, .inc_step = 1),
	FIELD_END };

static Field varMotorTempMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsMotorTemp].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorTemp].max, "C", 0, 200, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorTemp].min, "C", 0, 200, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorTemp].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsMotorTemp].config_error_threshold, "C", 0, 200, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsMotorTemp].config_warn_threshold, "C", 0, 200, .div_digits = 1, .inc_step = 1),
	FIELD_END };

static Field varMotorERPSMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsMotorERPS].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorERPS].max, "", 0, 2000, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorERPS].min, "", 0, 2000, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorERPS].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsMotorERPS].config_error_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsMotorERPS].config_warn_threshold, "", 0, 2000, .div_digits = 1, .inc_step = 1),
	FIELD_END };

static Field varMotorPWMMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsMotorPWM].auto_max_min, "auto", "man", "semi"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorPWM].max, "", 0, 255, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorPWM].min, "", 0, 255, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorPWM].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsMotorPWM].config_error_threshold, "", 0, 500, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsMotorPWM].config_warn_threshold, "", 0, 500, .div_digits = 1, .inc_step = 1),
	FIELD_END };

static Field varMotorFOCMenus[] =
{
    FIELD_EDITABLE_ENUM("Graph auto max min", &g_graphVars[VarsMotorFOC].auto_max_min, "auto", "man"),
    FIELD_EDITABLE_UINT("Graph max", &g_graphVars[VarsMotorFOC].max, "", 0, 60, .inc_step = 1),
    FIELD_EDITABLE_UINT("Graph min", &g_graphVars[VarsMotorFOC].min, "", 0, 60, .inc_step = 1),
    FIELD_EDITABLE_ENUM("Thresholds", &g_vars[VarsMotorFOC].auto_thresholds, "disabled", "manual", "auto"),
    FIELD_EDITABLE_UINT("Max threshold", &g_vars[VarsMotorFOC].config_error_threshold, "", 0, 120, .div_digits = 1, .inc_step = 1),
    FIELD_EDITABLE_UINT("Min threshold", &g_vars[VarsMotorFOC].config_warn_threshold, "", 0, 120, .div_digits = 1, .inc_step = 1),
	FIELD_END };

static Field variablesMenus[] =
{
	FIELD_SCROLLABLE("Speed", varSpeedMenus),
	FIELD_SCROLLABLE("Trip distance", varTripDistanceMenus),
	FIELD_SCROLLABLE("Cadence", varCadenceMenus),
	FIELD_SCROLLABLE("human power", varHumanPowerMenus),
	FIELD_SCROLLABLE("motor power", varBatteryPowerMenus),
	FIELD_SCROLLABLE("Watts/km", varBatteryPowerUsageMenus),
	FIELD_SCROLLABLE("batt voltage", varBatteryVoltageMenus),
	FIELD_SCROLLABLE("batt current", varBatteryCurrentMenus),
	FIELD_SCROLLABLE("battery SOC", varBatterySOCMenus),
	FIELD_SCROLLABLE("motor current", varMotorCurrentMenus),
	FIELD_SCROLLABLE("motor temp", varMotorTempMenus),
	FIELD_SCROLLABLE("motor speed", varMotorERPSMenus),
	FIELD_SCROLLABLE("motor pwm", varMotorPWMMenus),
	FIELD_SCROLLABLE("motor foc", varMotorFOCMenus),
	FIELD_END };
#endif

static Field technicalMenus[] =
{
#ifndef SW102
	FIELD_READONLY_UINT("ADC battery current", &ui_vars.ui16_adc_battery_current, ""),
	FIELD_READONLY_UINT("ADC throttle sensor", &ui_vars.ui8_adc_throttle, ""),
	FIELD_READONLY_UINT("Throttle sensor", &ui_vars.ui8_throttle, ""),
	FIELD_READONLY_UINT("ADC torque sensor", &ui_vars.ui16_adc_pedal_torque_sensor, ""),
	FIELD_READONLY_UINT("ADC torque delta", &ui_vars.ui16_adc_pedal_torque_delta, ""),
	FIELD_READONLY_UINT("ADC torque boost", &ui_vars.ui16_adc_pedal_torque_delta_boost, ""),
	FIELD_READONLY_UINT("Pedal cadence", &ui_vars.ui8_pedal_cadence, "rpm"),
	FIELD_READONLY_UINT("PWM duty-cycle", &ui_vars.ui8_duty_cycle, ""),
	FIELD_READONLY_UINT("Motor speed", &ui_vars.ui16_motor_speed_erps, ""),
	FIELD_READONLY_UINT("Motor FOC", &ui_vars.ui8_foc_angle, ""),
	FIELD_READONLY_UINT("Hall sensors", &ui_vars.ui8_motor_hall_sensors, ""),
#else
	FIELD_READONLY_UINT("ADC bat cu", &ui_vars.ui16_adc_battery_current, ""),
	FIELD_READONLY_UINT("ADC thrott", &ui_vars.ui8_adc_throttle, ""),
	FIELD_READONLY_UINT("Throttle s", &ui_vars.ui8_throttle, ""),
	FIELD_READONLY_UINT("ADC torque", &ui_vars.ui16_adc_pedal_torque_sensor, ""),
	FIELD_READONLY_UINT("ADC delta", &ui_vars.ui16_adc_pedal_torque_delta, ""),
	FIELD_READONLY_UINT("ADC boost", &ui_vars.ui16_adc_pedal_torque_delta_boost, ""),
	FIELD_READONLY_UINT("Cadence", &ui_vars.ui8_pedal_cadence, "rpm"),
	FIELD_READONLY_UINT("PWM duty", &ui_vars.ui8_duty_cycle, ""),
	FIELD_READONLY_UINT("Mot speed", &ui_vars.ui16_motor_speed_erps, ""),
	FIELD_READONLY_UINT("Motor FOC", &ui_vars.ui8_foc_angle, ""),
	FIELD_READONLY_UINT("Hall sens", &ui_vars.ui8_motor_hall_sensors, ""),
#endif
	FIELD_END };

static Field topMenus[] =
{
	FIELD_SCROLLABLE("Trip memories", tripMenus),
	FIELD_SCROLLABLE("Bike", bikeMenus),
	FIELD_SCROLLABLE("Battery", batteryMenus),
	FIELD_SCROLLABLE("SOC", batterySOCMenus),
	FIELD_SCROLLABLE("Motor", motorMenus),
#ifndef SW102
	FIELD_SCROLLABLE("Torque sensor", torqueSensorMenus),
	FIELD_SCROLLABLE("Assist level", assistMenus),
	FIELD_SCROLLABLE("Walk assist", walkAssistMenus),
	FIELD_SCROLLABLE("Startup BOOST", startupPowerMenus),
	FIELD_SCROLLABLE("Motor temperature", motorTempMenus),
	FIELD_SCROLLABLE("Street mode", streetModeMenus),
	FIELD_SCROLLABLE("Variables", variablesMenus),
#else
	FIELD_SCROLLABLE("Torque sen", torqueSensorMenus),
	FIELD_SCROLLABLE("Torque cal", torqueCalibrationMenus),
	FIELD_SCROLLABLE("Assist", assistMenus),
	FIELD_SCROLLABLE("Walk", walkAssistMenus),
	FIELD_SCROLLABLE("StartBOOST", startupPowerMenus),
	FIELD_SCROLLABLE("Motor temp", motorTempMenus),
	FIELD_SCROLLABLE("Street mod", streetModeMenus),
#endif
	FIELD_SCROLLABLE("Various", variousMenus),
	FIELD_SCROLLABLE("Display", displayMenus),
	FIELD_SCROLLABLE("Technical", technicalMenus),
	FIELD_END };

#ifndef SW102
static Field configRoot = FIELD_SCROLLABLE("Configurations", topMenus);
#else
static Field configRoot = FIELD_SCROLLABLE("Config", topMenus);
#endif

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
