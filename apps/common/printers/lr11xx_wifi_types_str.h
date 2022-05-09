#ifndef LR11XX_WIFI_TYPES_STR_H
#define LR11XX_WIFI_TYPES_STR_H
#include "lr11xx_wifi_types.h"
#ifdef __cplusplus
extern "C" {
#endif
const char* lr11xx_wifi_channel_to_str( const lr11xx_wifi_channel_t value );
const char* lr11xx_wifi_datarate_to_str( const lr11xx_wifi_datarate_t value );
const char* lr11xx_wifi_frame_type_to_str( const lr11xx_wifi_frame_type_t value );
const char* lr11xx_wifi_mac_origin_to_str( const lr11xx_wifi_mac_origin_t value );
const char* lr11xx_wifi_signal_type_scan_to_str( const lr11xx_wifi_signal_type_scan_t value );
const char* lr11xx_wifi_signal_type_result_to_str( const lr11xx_wifi_signal_type_result_t value );
const char* lr11xx_wifi_mode_to_str( const lr11xx_wifi_mode_t value );
const char* lr11xx_wifi_result_format_to_str( const lr11xx_wifi_result_format_t value );
#ifdef __cplusplus
}
#endif
#endif  // LR11XX_WIFI_TYPES_STR_H
