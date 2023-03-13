/*!
 * @file      smtc_shield_sx1261mb1cas_cfg.h
 *
 * @brief     Interface specific to SX1261MB1CAS shield
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
#ifndef SMTC_SHIELD_SX1261MB1CAS_H
#define SMTC_SHIELD_SX1261MB1CAS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "smtc_shield_sx126x_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define SMTC_SHIELD_SX1261MB1CAS_INSTANTIATE                                                                          \
    {                                                                                                                 \
        .get_pa_pwr_cfg           = smtc_shield_sx1261mb1cas_get_pa_pwr_cfg,                                          \
        .is_dio2_set_as_rf_switch = smtc_shield_sx1261mb1cas_is_dio2_set_as_rf_switch,                                \
        .get_reg_mode = smtc_shield_sx1261mb1cas_get_reg_mode, .get_xosc_cfg = smtc_shield_sx1261mb1cas_get_xosc_cfg, \
        .get_pinout = smtc_shield_sx1261mb1cas_get_pinout,                                                            \
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
 * @see smtc_shield_sx126x_get_pa_pwr_cfg
 */
const smtc_shield_sx126x_pa_pwr_cfg_t* smtc_shield_sx1261mb1cas_get_pa_pwr_cfg( const uint32_t rf_freq_in_hz,
                                                                                int8_t expected_output_pwr_in_dbm );

/**
 * @see smtc_shield_sx126x_is_dio2_set_as_rf_switch
 */
bool smtc_shield_sx1261mb1cas_is_dio2_set_as_rf_switch( void );

/**
 * @see smtc_shield_sx126x_get_reg_mode
 */
sx126x_reg_mod_t smtc_shield_sx1261mb1cas_get_reg_mode( void );

/**
 * @see smtc_shield_sx126x_get_xosc_cfg
 */
const smtc_shield_sx126x_xosc_cfg_t* smtc_shield_sx1261mb1cas_get_xosc_cfg( void );

/**
 * @see smtc_shield_sx126x_get_pinout
 */
const smtc_shield_sx126x_pinout_t* smtc_shield_sx1261mb1cas_get_pinout( void );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_SHIELD_SX1261MB1CAS_H
