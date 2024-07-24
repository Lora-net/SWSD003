/*!
 * @file      sx126x_str.h
 *
 * @brief     Printer helper functions for SX126x types
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

#ifndef SX126X_STR_H
#define SX126X_STR_H
#include "sx126x.h"
#ifdef __cplusplus
extern "C" {
#endif
const char* sx126x_status_to_str( const sx126x_status_t value );
const char* sx126x_sleep_cfgs_to_str( const sx126x_sleep_cfgs_t value );
const char* sx126x_standby_cfgs_to_str( const sx126x_standby_cfgs_t value );
const char* sx126x_reg_mod_to_str( const sx126x_reg_mod_t value );
const char* sx126x_fallback_modes_to_str( const sx126x_fallback_modes_t value );
const char* sx126x_tcxo_ctrl_voltages_to_str( const sx126x_tcxo_ctrl_voltages_t value );
const char* sx126x_pkt_type_to_str( const sx126x_pkt_type_t value );
const char* sx126x_ramp_time_to_str( const sx126x_ramp_time_t value );
const char* sx126x_gfsk_pulse_shape_to_str( const sx126x_gfsk_pulse_shape_t value );
const char* sx126x_bpsk_pulse_shape_to_str( const sx126x_bpsk_pulse_shape_t value );
const char* sx126x_gfsk_bw_to_str( const sx126x_gfsk_bw_t value );
const char* sx126x_lora_sf_to_str( const sx126x_lora_sf_t value );
const char* sx126x_lora_bw_to_str( const sx126x_lora_bw_t value );
const char* sx126x_lora_cr_to_str( const sx126x_lora_cr_t value );
const char* sx126x_gfsk_preamble_detector_to_str( const sx126x_gfsk_preamble_detector_t value );
const char* sx126x_gfsk_address_filtering_to_str( const sx126x_gfsk_address_filtering_t value );
const char* sx126x_gfsk_pkt_len_modes_to_str( const sx126x_gfsk_pkt_len_modes_t value );
const char* sx126x_gfsk_crc_types_to_str( const sx126x_gfsk_crc_types_t value );
const char* sx126x_gfsk_dc_free_to_str( const sx126x_gfsk_dc_free_t value );
const char* sx126x_lora_pkt_len_modes_to_str( const sx126x_lora_pkt_len_modes_t value );
const char* sx126x_cad_symbs_to_str( const sx126x_cad_symbs_t value );
const char* sx126x_cad_exit_modes_to_str( const sx126x_cad_exit_modes_t value );
const char* sx126x_chip_modes_to_str( const sx126x_chip_modes_t value );
const char* sx126x_cmd_status_to_str( const sx126x_cmd_status_t value );
#ifdef __cplusplus
}
#endif
#endif  // SX126X_STR_H
