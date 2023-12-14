/*!
 * @file      smtc_shield_lr1120mb1ipddis.h
 *
 * @brief     Interface specific to LR1120MB1IPDDIS shield
 *
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
#ifndef SMTC_SHIELD_LR1120MB1IPDDIS_H
#define SMTC_SHIELD_LR1120MB1IPDDIS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "smtc_shield_lr11xx_types.h"
#include "smtc_shield_lr11x0_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define SMTC_SHIELD_LR1120MB1IPDDIS_INSTANTIATE                                               \
    {                                                                                         \
        .get_pa_pwr_cfg             = smtc_shield_lr1120mb1ipddis_get_pa_pwr_cfg,             \
        .get_rssi_calibration_table = smtc_shield_lr1120mb1ipddis_get_rssi_calibration_table, \
        .get_rf_switch_cfg          = smtc_shield_lr1120mb1ipddis_get_rf_switch_cfg,          \
        .get_reg_mode               = smtc_shield_lr1120mb1ipddis_get_reg_mode,               \
        .get_xosc_cfg               = smtc_shield_lr1120mb1ipddis_get_xosc_cfg,               \
        .get_lfclk_cfg              = smtc_shield_lr1120mb1ipddis_get_lfclk_cfg,              \
        .get_pinout                 = smtc_shield_lr1120mb1ipddis_get_pinout,                 \
        .get_rttof_recommended_rx_tx_delay_indicator =                                        \
            smtc_shield_lr11x0_common_rttof_recommended_rx_tx_delay_indicator,                \
    }

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @see smtc_shield_lr11xx_get_pa_pwr_cfg
 */
const smtc_shield_lr11xx_pa_pwr_cfg_t* smtc_shield_lr1120mb1ipddis_get_pa_pwr_cfg( const uint32_t rf_freq_in_hz,
                                                                                   int8_t expected_output_pwr_in_dbm );

/**
 * @see smtc_shield_lr11xx_get_rssi_calibration_table
 */
const lr11xx_radio_rssi_calibration_table_t* smtc_shield_lr1120mb1ipddis_get_rssi_calibration_table(
    const uint32_t rf_freq_in_hz );

/**
 * @see smtc_shield_lr11xx_get_rf_switch_cfg
 */
const lr11xx_system_rfswitch_cfg_t* smtc_shield_lr1120mb1ipddis_get_rf_switch_cfg( void );

/**
 * @see smtc_shield_lr11xx_get_reg_mode
 */
lr11xx_system_reg_mode_t smtc_shield_lr1120mb1ipddis_get_reg_mode( void );

/**
 * @see smtc_shield_lr11xx_get_xosc_cfg
 */
const smtc_shield_lr11xx_xosc_cfg_t* smtc_shield_lr1120mb1ipddis_get_xosc_cfg( void );

/**
 * @see smtc_shield_lr11xx_get_lfclk_cfg
 */
const smtc_shield_lr11xx_lfclk_cfg_t* smtc_shield_lr1120mb1ipddis_get_lfclk_cfg( void );

/**
 * @see smtc_shield_lr11xx_get_pinout
 */
const smtc_shield_lr11xx_pinout_t* smtc_shield_lr1120mb1ipddis_get_pinout( void );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_SHIELD_LR1120MB1IPDDIS_H
