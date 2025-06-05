#include "eeprom.h"
#include <string.h> 

#include "main.h"
#include "config_tsdz8.h"
#include "ebike_app.h"
#include "xmc1_flash.h"


extern struct_configuration_variables m_configuration_variables ; 

/*
static const uint8_t ui8_default_array[EEPROM_BYTES_STORED] = 
{
  DEFAULT_VALUE_KEY,							// 0 + EEPROM_BASE_ADDRESS (Array index)
  BATTERY_CURRENT_MAX,							// 1 + EEPROM_BASE_ADDRESS    It was 13A for TSDZ2, reduced to 5 for test of TSDZ8
  BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0,			// 2 + EEPROM_BASE_ADDRESS
  BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1,			// 3 + EEPROM_BASE_ADDRESS
  WHEEL_PERIMETER_0,							// 4 + EEPROM_BASE_ADDRESS
  WHEEL_PERIMETER_1,							// 5 + EEPROM_BASE_ADDRESS
  WHEEL_MAX_SPEED,								// 6 + EEPROM_BASE_ADDRESS
//  MOTOR_TYPE,									// 7 + EEPROM_BASE_ADDRESS           // removed in a TSDZ2 update
//  AVAILABLE_FOR_FUTURE_USE,						// 8 + EEPROM_BASE_ADDRESS   // removed in a TSDZ2 update
  // for oem display
  TORQUE_SENSOR_ESTIMATED,						// 7 + EEPROM_BASE_ADDRESS     // added in a TSDZ2 update
  PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100,		// 8 + EEPROM_BASE_ADDRESS // added in a TSDZ2 update
  MOTOR_ASSISTANCE_WITHOUT_PEDAL_ROTATION,		// 9 + EEPROM_BASE_ADDRESS 0 or 1 ; 1 allows to get assist just pressing on the pedal
  ASSISTANCE_WITH_ERROR_ENABLED,				// 10 + EEPROM_BASE_ADDRESS
  BATTERY_SOC,									// 11 + EEPROM_BASE_ADDRESS    State of charge
  ENABLE_SET_PARAMETER_ON_STARTUP,				// 12 + EEPROM_BASE_ADDRESS
  ENABLE_STREET_MODE_ON_STARTUP,				// 13 + EEPROM_BASE_ADDRESS
  RIDING_MODE_ON_STARTUP,						// 14 + EEPROM_BASE_ADDRESS   ; see list of code here below    
  LIGHTS_CONFIGURATION_ON_STARTUP,				// 15 + EEPROM_BASE_ADDRESS
  STARTUP_BOOST_ON_STARTUP,						// 16 + EEPROM_BASE_ADDRESS
  ENABLE_AUTO_DATA_DISPLAY,						// 17 + EEPROM_BASE_ADDRESS
  SOC_PERCENT_CALC,								// 18 + EEPROM_BASE_ADDRESS
  TORQUE_SENSOR_ADV_ON_STARTUP					// 19 + EEPROM_BASE_ADDRESS
};
*/

// Fill m_configuration_variables with saved flash or with default
void m_configuration_init(void){
 // added by mstrens in order to fill m-configuration_variables with the variables from m_config before having eeprom functions
 // This has to be called AFTER that m_config has been filled
  // pointer to m_configuration_variables
  struct_configuration_variables *p_configuration_variables = &m_configuration_variables;
  // 32 bytes to use as buffer  
  uint32_t ui32_temp[8] = {0};  //reserve 32 bytes = 2 bloks of 16 bytes
  // flash address being used
  uint32_t * pAddress32 = (uint32_t *) ADDRESS_OF_M_CONFIGURATION_VARIABLES;
  // read version in flash
  uint32_t ui32_saved_key = XMC_FLASH_ReadWord(pAddress32);
  
  // when saved_key is not valid, fill m_configuration_variables with default from general settings
  if (ui32_saved_key != VERSION_OF_M_CONFIGURATION_VARIABLES) {
    m_configuration_variables.version = VERSION_OF_M_CONFIGURATION_VARIABLES;  
    m_configuration_variables.ui8_battery_current_max = m_config.battery_current_max;
    m_configuration_variables.ui16_battery_low_voltage_cut_off_x10 = m_config.battery_low_voltage_cut_off * 10;
    m_configuration_variables.ui16_wheel_perimeter = m_config.wheel_perimeter;
    m_configuration_variables.ui8_wheel_speed_max = m_config.wheel_max_speed; 
    //m_configuration_variables.ui8_motor_type = m_config.motor_type; // replaced by ui8_torque_sensor_estimated     
    //m_configuration_variables.ui8_available_for_future_use = 0; //replaced by ui8_pedal_torque_per_10_bit_ADC_step_est_x100
    // for oem display
    m_configuration_variables.ui8_torque_sensor_estimated = 0; // added by an updated
    m_configuration_variables.ui8_pedal_torque_per_10_bit_ADC_step_est_x100 = m_config.pedal_torque_per_10_bit_adc_step_x100; // added by an updated
    m_configuration_variables.ui8_assist_without_pedal_rotation_enabled = m_config.motor_assistance_without_pedal_rotation;
    m_configuration_variables.ui8_assist_with_error_enabled = ASSISTANCE_WITH_ERROR_ENABLED; // is always disabled; must be enabled with the display
    m_configuration_variables.ui8_battery_SOC_percentage_8b = 0; // mstrens : Should be calculated and stored in eeprom (not defined in m_config) 
    m_configuration_variables.ui8_set_parameter_enabled = m_config.enable_set_parameter_on_startup;
    m_configuration_variables.ui8_street_mode_enabled = m_config.enable_street_mode_on_startup;
    m_configuration_variables.ui8_riding_mode = m_config.riding_mode_on_startup;
    m_configuration_variables.ui8_lights_configuration = m_config.lights_configuration_on_startup;
    m_configuration_variables.ui8_startup_boost_enabled = m_config.startup_boost_on_startup;
    m_configuration_variables.ui8_auto_display_data_enabled = m_config.enable_auto_data_display;
    m_configuration_variables.ui8_soc_percent_calculation = m_config.soc_percent_calc;
    m_configuration_variables.ui8_torque_sensor_adv_enabled = m_config.torque_sensor_adv_on_startup;
    //copy config in ui32_temp[8] nad save ui32_temp into flash, so it can be changed and reused
    memcpy( ui32_temp, p_configuration_variables , (uint32_t) sizeof(m_configuration_variables) ); // dest, source, number)
    XMC_FLASH_WriteBlocks	(	pAddress32, ui32_temp, 2, false);	 // flash adr, source, number of blocks of 16 bytes, verify   
  }
  else { // when saved_key is valid, use flash data
    // read config (32 bytes) into ui32_temp and copy the right number of bytes into m_configuration_variable
    XMC_FLASH_ReadBlocks	(	pAddress32, ui32_temp , 2); // we read 2 X 16 bytes (24 are normally enough)
    memcpy( p_configuration_variables , ui32_temp , sizeof(m_configuration_variables) ); // dest , source, number
  }
}

void EEPROM_write()
{
  struct_configuration_variables *p_configuration_variables = &m_configuration_variables;  
  uint32_t * pAddress32 = (uint32_t *) ADDRESS_OF_M_CONFIGURATION_VARIABLES;
  uint32_t ui32_temp[8] = {0};  //reserve 32 bytes = 2 bloks of 16 bytes
  // copy the config in a buffer (32 bytes)
  memcpy( ui32_temp, p_configuration_variables , (uint32_t) sizeof(m_configuration_variables) ); // dest, source, number)
  XMC_FLASH_WriteBlocks	(	pAddress32, ui32_temp, 2, false);	 // flash adr, source, number of blocks of 16 bytes, verify   
}