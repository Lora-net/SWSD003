/**
 * @file      smtc_shield_sx126x.h
 *
 * @brief     Semtech SX126x shield configuration interface
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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

#ifndef SMTC_SHIELD_SX126X_H
#define SMTC_SHIELD_SX126X_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stddef.h>
#include "smtc_shield_sx126x_types.h"

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

/**
 * @brief Function pointer to abstract PA and power configuration getter
 */
typedef const smtc_shield_sx126x_pa_pwr_cfg_t* ( *smtc_shield_sx126x_get_pa_pwr_cfg_f )(
    const uint32_t rf_freq_in_hz, int8_t expected_output_pwr_in_dbm );

/**
 * @brief Function pointer to abstract DIO2 configuration getter
 */
typedef bool ( *smtc_shield_sx126x_is_dio2_set_as_rf_switch_f )( void );

/**
 * @brief Function pointer to abstract regulator mode configuration getter
 */
typedef sx126x_reg_mod_t ( *smtc_shield_sx126x_get_reg_mode_f )( void );

/**
 * @brief Function pointer to abstract XOSC configuration getter
 */
typedef const smtc_shield_sx126x_xosc_cfg_t* ( *smtc_shield_sx126x_get_xosc_cfg_f )( void );

/**
 * @brief Function pointer to abstract pinout getter
 */
typedef const smtc_shield_sx126x_pinout_t* ( *smtc_shield_sx126x_get_pinout_f )( void );

/**
 * @brief SX126x shield function pointer structure definition
 */
typedef struct smtc_shield_sx126x_s
{
    smtc_shield_sx126x_get_pa_pwr_cfg_f           get_pa_pwr_cfg;
    smtc_shield_sx126x_is_dio2_set_as_rf_switch_f is_dio2_set_as_rf_switch;
    smtc_shield_sx126x_get_reg_mode_f             get_reg_mode;
    smtc_shield_sx126x_get_xosc_cfg_f             get_xosc_cfg;
    smtc_shield_sx126x_get_pinout_f               get_pinout;
} smtc_shield_sx126x_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Return the PA (Power Amplifier) configuration
 *
 * @param [in] shield  Pointer to a shield data structure
 * @param [in] rf_freq_in_hz  RF frequency in Hertz
 * @param [in] expected_output_pwr_in_dbm  Expected output power in dBm
 *
 * @return A pointer to a PA configuration corresponding to the input configuration
 */
static inline const smtc_shield_sx126x_pa_pwr_cfg_t* smtc_shield_sx126x_get_pa_pwr_cfg(
    smtc_shield_sx126x_t* shield, const uint32_t rf_freq_in_hz, int8_t expected_output_pwr_in_dbm )
{
    return shield->get_pa_pwr_cfg( rf_freq_in_hz, expected_output_pwr_in_dbm );
}

/**
 * @brief Return the status of DIO2
 *
 * @param [in] shield  Pointer to a shield data structure
 *
 * @return DIO2
 */
static inline bool smtc_shield_sx126x_is_dio2_set_as_rf_switch( smtc_shield_sx126x_t* shield )
{
    return shield->is_dio2_set_as_rf_switch( );
}

/**
 * @brief Return the regulator mode
 *
 * @param [in] shield  Pointer to a shield data structure
 *
 * @return Regulator mode
 */
static inline sx126x_reg_mod_t smtc_shield_sx126x_get_reg_mode( smtc_shield_sx126x_t* shield )
{
    return shield->get_reg_mode( );
}

/**
 * @brief Return the XOSC (eXternal OSCillator) configuration
 *
 * @param [in] shield  Pointer to a shield data structure
 *
 * @return XOSC configuration
 */
static inline const smtc_shield_sx126x_xosc_cfg_t* smtc_shield_sx126x_get_xosc_cfg( smtc_shield_sx126x_t* shield )
{
    return shield->get_xosc_cfg( );
}

/**
 * @brief Return the board pinout configuration
 *
 * @param [in] shield  Pointer to a shield data structure
 *
 * @return Pinout configuration
 */
static inline const smtc_shield_sx126x_pinout_t* smtc_shield_sx126x_get_pinout( smtc_shield_sx126x_t* shield )
{
    return shield->get_pinout( );
}

#ifdef __cplusplus
}
#endif

#endif  // SMTC_SHIELD_SX126X_H

/* --- EOF ------------------------------------------------------------------ */
