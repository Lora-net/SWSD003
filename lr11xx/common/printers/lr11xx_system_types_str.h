/*!
 * @file      lr11xx_system_types_str.h
 *
 * @brief     Printer helper functions for LR11xx system types
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
const char* lr11xx_system_version_type_to_str( const lr11xx_system_version_type_t value );
#ifdef __cplusplus
}
#endif
#endif  // LR11XX_SYSTEM_TYPES_STR_H
