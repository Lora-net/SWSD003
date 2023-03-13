/*!
 * \file      smtc_shield_sx126x_types.h
 *
 * \brief     Interface for types common to all SX126x-based shields
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
#ifndef SMTC_SHIELD_SX126X_TYPES_H
#define SMTC_SHIELD_SX126X_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "sx126x.h"
#include "smtc_shield_pinout.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define SMTC_SHIELD_SX126X_FREQ_MIN 150000000
#define SMTC_SHIELD_SX126X_FREQ_MAX 960000000

#define SMTC_SHIELD_SX1261_MIN_PWR -17
#define SMTC_SHIELD_SX1261_MAX_PWR 15

#define SMTC_SHIELD_SX1262_MIN_PWR -9
#define SMTC_SHIELD_SX1262_MAX_PWR 22

#define SMTC_SHIELD_SX1268_MIN_PWR -9
#define SMTC_SHIELD_SX1268_MAX_PWR 22

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Power amplifier and output power configurations structure definition
 */
typedef struct smtc_shield_sx126x_pa_pwr_cfg_s
{
    int8_t                 power;
    sx126x_pa_cfg_params_t pa_config;
} smtc_shield_sx126x_pa_pwr_cfg_t;

/**
 * @brief External 32MHz oscillator configuration structure definition
 */
typedef struct smtc_shield_sx126x_xosc_cfg_s
{
    bool                        tcxo_is_radio_controlled;
    sx126x_tcxo_ctrl_voltages_t supply_voltage;
    uint32_t                    startup_time_in_tick;
} smtc_shield_sx126x_xosc_cfg_t;

/**
 * @brief Pinout structure definition
 */
typedef struct smtc_shield_sx126x_pinout_s
{
    smtc_shield_pinout_t busy;
    smtc_shield_pinout_t irq;
    smtc_shield_pinout_t nss;
    smtc_shield_pinout_t reset;
    smtc_shield_pinout_t antenna_sw;
    smtc_shield_pinout_t led_tx;
    smtc_shield_pinout_t led_rx;
} smtc_shield_sx126x_pinout_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // SMTC_SHIELD_SX126X_TYPES_H
