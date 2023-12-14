/*!
 * @file      lr11xx_wifi_types_str.h
 *
 * @brief     Printer helper functions for LR11xx Wi-Fi types
 *
 * @copyright
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
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
