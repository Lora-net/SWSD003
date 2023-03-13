#ifndef LR11XX_BOOTLOADER_TYPES_STR_H
#define LR11XX_BOOTLOADER_TYPES_STR_H
#include "lr11xx_bootloader_types.h"
#ifdef __cplusplus
extern "C" {
#endif
const char* lr11xx_bootloader_chip_modes_to_str( const lr11xx_bootloader_chip_modes_t value );
const char* lr11xx_bootloader_reset_status_to_str( const lr11xx_bootloader_reset_status_t value );
const char* lr11xx_bootloader_command_status_to_str( const lr11xx_bootloader_command_status_t value );
#ifdef __cplusplus
}
#endif
#endif  // LR11XX_BOOTLOADER_TYPES_STR_H
