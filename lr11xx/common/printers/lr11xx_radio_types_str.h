/*!
 * @file      lr11xx_radio_types_str.h
 *
 * @brief     Printer helper functions for LR11xx radio types
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
 
#ifndef LR11XX_RADIO_TYPES_STR_H
#define LR11XX_RADIO_TYPES_STR_H
#include "lr11xx_radio_types.h"
#ifdef __cplusplus
extern "C" {
#endif
const char* lr11xx_radio_pa_selection_to_str( const lr11xx_radio_pa_selection_t value );
const char* lr11xx_radio_gfsk_address_filtering_to_str( const lr11xx_radio_gfsk_address_filtering_t value );
const char* lr11xx_radio_fallback_modes_to_str( const lr11xx_radio_fallback_modes_t value );
const char* lr11xx_radio_ramp_time_to_str( const lr11xx_radio_ramp_time_t value );
const char* lr11xx_radio_lora_network_type_to_str( const lr11xx_radio_lora_network_type_t value );
const char* lr11xx_radio_lora_sf_to_str( const lr11xx_radio_lora_sf_t value );
const char* lr11xx_radio_lora_bw_to_str( const lr11xx_radio_lora_bw_t value );
const char* lr11xx_radio_lora_cr_to_str( const lr11xx_radio_lora_cr_t value );
const char* lr11xx_radio_intermediary_mode_to_str( const lr11xx_radio_intermediary_mode_t value );
const char* lr11xx_radio_gfsk_crc_type_to_str( const lr11xx_radio_gfsk_crc_type_t value );
const char* lr11xx_radio_gfsk_dc_free_to_str( const lr11xx_radio_gfsk_dc_free_t value );
const char* lr11xx_radio_gfsk_pkt_len_modes_to_str( const lr11xx_radio_gfsk_pkt_len_modes_t value );
const char* lr11xx_radio_gfsk_preamble_detector_to_str( const lr11xx_radio_gfsk_preamble_detector_t value );
const char* lr11xx_radio_lora_crc_to_str( const lr11xx_radio_lora_crc_t value );
const char* lr11xx_radio_lora_pkt_len_modes_to_str( const lr11xx_radio_lora_pkt_len_modes_t value );
const char* lr11xx_radio_lora_iq_to_str( const lr11xx_radio_lora_iq_t value );
const char* lr11xx_radio_pkt_type_to_str( const lr11xx_radio_pkt_type_t value );
const char* lr11xx_radio_pa_reg_supply_to_str( const lr11xx_radio_pa_reg_supply_t value );
const char* lr11xx_radio_rx_duty_cycle_mode_to_str( const lr11xx_radio_rx_duty_cycle_mode_t value );
const char* lr11xx_radio_gfsk_bw_to_str( const lr11xx_radio_gfsk_bw_t value );
const char* lr11xx_radio_cad_exit_mode_to_str( const lr11xx_radio_cad_exit_mode_t value );
const char* lr11xx_radio_gfsk_pulse_shape_to_str( const lr11xx_radio_gfsk_pulse_shape_t value );
const char* lr11xx_radio_bpsk_pulse_shape_to_str( const lr11xx_radio_bpsk_pulse_shape_t value );
const char* lr11xx_radio_lr_fhss_bitrate_to_str( const lr11xx_radio_lr_fhss_bitrate_t value );
const char* lr11xx_radio_lr_fhss_pulse_shape_to_str( const lr11xx_radio_lr_fhss_pulse_shape_t value );
#ifdef __cplusplus
}
#endif
#endif  // LR11XX_RADIO_TYPES_STR_H
