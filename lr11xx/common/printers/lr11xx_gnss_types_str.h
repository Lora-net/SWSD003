/*!
 * @file      lr11xx_gnss_types_str.h
 *
 * @brief     Printer helper functions for LR11xx GNSS types
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
const char* lr11xx_gnss_fetch_time_option_to_str( const lr11xx_gnss_fetch_time_option_t value );
const char* lr11xx_gnss_read_time_status_to_str( const lr11xx_gnss_read_time_status_t value );
const char* lr11xx_gnss_week_number_rollover_status_to_str( const lr11xx_gnss_week_number_rollover_status_t value );
const char* lr11xx_gnss_demod_status_to_str( const lr11xx_gnss_demod_status_t value );
const char* lr11xx_gnss_doppler_solver_error_code_to_str( const lr11xx_gnss_doppler_solver_error_code_t value );
const char* lr11xx_gnss_almanac_status_to_str( const lr11xx_gnss_almanac_status_t value );
const char* lr11xx_gnss_sv_type_to_str( const lr11xx_gnss_sv_type_t value );
const char* lr11xx_gnss_scan_mode_launched_to_str( const lr11xx_gnss_scan_mode_launched_t value );
#ifdef __cplusplus
}
#endif
#endif  // LR11XX_GNSS_TYPES_STR_H
