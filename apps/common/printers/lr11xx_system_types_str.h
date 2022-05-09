#ifndef LR11XX_SYSTEM_TYPES_STR_H
#define LR11XX_SYSTEM_TYPES_STR_H
#include "lr11xx_system_types.h"
#ifdef __cplusplus
extern "C" {
#endif
const char* lr11xx_system_chip_modes_to_str( const lr11xx_system_chip_modes_t value );
const char* lr11xx_system_reset_status_to_str( const lr11xx_system_reset_status_t value );
const char* lr11xx_system_command_status_to_str( const lr11xx_system_command_status_t value );
const char* lr11xx_system_lfclk_cfg_to_str( const lr11xx_system_lfclk_cfg_t value );
const char* lr11xx_system_reg_mode_to_str( const lr11xx_system_reg_mode_t value );
const char* lr11xx_system_infopage_id_to_str( const lr11xx_system_infopage_id_t value );
const char* lr11xx_system_standby_cfg_to_str( const lr11xx_system_standby_cfg_t value );
const char* lr11xx_system_tcxo_supply_voltage_to_str( const lr11xx_system_tcxo_supply_voltage_t value );
#ifdef __cplusplus
}
#endif
#endif  // LR11XX_SYSTEM_TYPES_STR_H
