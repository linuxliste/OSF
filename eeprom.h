#pragma once

// system
#define DEFAULT_VALUE_KEY     204
#define SET_TO_DEFAULT        0
#define READ_FROM_MEMORY      1
#define WRITE_TO_MEMORY       2



void m_configuration_init(void); // create by mstrens to fill m_configuration structure before having real eeprom functions
void EEPROM_init(void);
void EEPROM_write();