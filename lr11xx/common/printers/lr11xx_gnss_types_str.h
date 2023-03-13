#ifndef LR11XX_GNSS_TYPES_STR_H
#define LR11XX_GNSS_TYPES_STR_H
#include "lr11xx_gnss_types.h"
#ifdef __cplusplus
extern "C" {
#endif
const char* lr11xx_gnss_constellation_to_str( const lr11xx_gnss_constellation_t value );
const char* lr11xx_gnss_search_mode_to_str( const lr11xx_gnss_search_mode_t value );
const char* lr11xx_gnss_destination_to_str( const lr11xx_gnss_destination_t value );
const char* lr11xx_gnss_message_host_status_to_str( const lr11xx_gnss_message_host_status_t value );
const char* lr11xx_gnss_message_dmc_opcode_to_str( const lr11xx_gnss_message_dmc_opcode_t value );
const char* lr11xx_gnss_scan_mode_to_str( const lr11xx_gnss_scan_mode_t value );
const char* lr11xx_gnss_error_code_to_str( const lr11xx_gnss_error_code_t value );
const char* lr11xx_gnss_freq_search_space_to_str( const lr11xx_gnss_freq_search_space_t value );
#ifdef __cplusplus
}
#endif
#endif  // LR11XX_GNSS_TYPES_STR_H
