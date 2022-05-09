/*!
 * @file      smtc_board.h
 *
 * @brief     Board specific package board API definition.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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
#ifndef SMTC_BOARD_H
#define SMTC_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_hal_trace.h"

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include "lr11xx_radio_types.h"
#include "lr11xx_system_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct smtc_board_pa_pwr_cfg_s
{
    int8_t                power;
    lr11xx_radio_pa_cfg_t pa_config;
} smtc_board_pa_pwr_cfg_t;

typedef struct
{
    bool                                has_tcxo;
    lr11xx_system_tcxo_supply_voltage_t supply;
    uint32_t                            timeout_ms;
} smtc_board_tcxo_cfg_t;

typedef struct
{
    lr11xx_system_lfclk_cfg_t lf_clk_cfg;
    bool                      wait_32k_ready;
} smtc_board_lf_clck_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Set up the shield to be ready to transmit
 */
void smtc_shield_handle_pre_tx( void );

/*!
 * @brief Set up the shield after a transmission
 */
void smtc_shield_handle_post_tx( void );

/*!
 * @brief Set up the shield to be ready to receive
 */
void smtc_shield_handle_pre_rx( void );

/*!
 * @brief Set up the shield after a reception
 */
void smtc_shield_handle_post_rx( void );

/*!
 * @brief Set up the shield to be ready to perform a GNSS scan
 */
void smtc_shield_handle_pre_gnss_scan( void );

/*!
 * @brief Set up the shield after a GNSS scan
 */
void smtc_shield_handle_post_gnss_scan( void );

/*!
 * @brief Set up the shield to be ready to perform a Wi-Fi scan
 */
void smtc_shield_handle_pre_wifi_scan( void );

/*!
 * @brief Set up the shield after a Wi-Fi scan
 */
void smtc_shield_handle_post_wifi_scan( void );

/*!
 * @brief Initializes HAL used Peripherals
 */
void smtc_board_init_periph( void );

/*!
 * @brief reinit the peripherals
 */
void smtc_board_reinit_periph( void );

/*!
 * @brief deinit the peripherals
 */
void smtc_board_deinit_periph( void );

/*!
 * @brief Get the power amplifier configuration given a RF frequency and output power
 *
 * @param [in] rf_freq_in_hz RF frequence in Hz
 * @param [in] expected_output_pwr_in_dbm TX output power in dBm
 *
 * @returns Pointer to a structure holding the expected configuration.
 * Can be NULL if no configuration found for given arguments.
 */
const smtc_board_pa_pwr_cfg_t* smtc_board_get_pa_pwr_cfg( const uint32_t rf_freq_in_hz,
                                                          int8_t         expected_output_pwr_in_dbm );

/*!
 * @brief Return a regulator configuration to use compatible with the shield
 *
 * @return A regulator configuration compatible with the shield
 */
lr11xx_system_reg_mode_t smtc_board_get_reg_mode( void );

/*!
 * @brief Get the TCXO configuration compatible with the shield
 *
 * @return TCXO configuration compatible with the shield
 */
smtc_board_tcxo_cfg_t smtc_board_get_tcxo_cfg( void );

/*!
 * @brief Get a LF clock configuration compatible with the shield
 *
 * @return LF clock configuration compatible with the shield
 */
smtc_board_lf_clck_cfg_t smtc_board_get_lf_clk_cfg( void );

/*!
 * @brief Return the RF switch configuration of the shield
 *
 * @return The RF switch configuration
 */
lr11xx_system_rfswitch_cfg_t smtc_board_get_rf_switch_cfg( void );

/*!
 * @brief Return the RSSI calibration table corresponding to a given RF frequency
 *
 * @param [in] freq_in_hz RF frequence in Hz
 *
 * @return The RSSI calibration table
 */
const lr11xx_radio_rssi_calibration_table_t* smtc_board_get_rssi_calibration_table( const uint32_t freq_in_hz );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_BOARD_H

/* --- EOF ------------------------------------------------------------------ */
