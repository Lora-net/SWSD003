/**
 * @file      smtc_shield_lr11xx.h
 *
 * @brief     Semtech LR11xx shield interface
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

#ifndef SMTC_SHIELD_LR11XX_H
#define SMTC_SHIELD_LR11XX_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stddef.h>
#include "smtc_shield_lr11xx_types.h"

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
typedef const smtc_shield_lr11xx_pa_pwr_cfg_t* ( *smtc_shield_lr11xx_get_pa_pwr_cfg_f )(
    const uint32_t rf_freq_in_hz, int8_t expected_output_pwr_in_dbm );

/**
 * @brief Function pointer to abstract RSSI calibration table getter
 */
typedef const lr11xx_radio_rssi_calibration_table_t* ( *smtc_shield_lr11xx_get_rssi_calibration_table_f )(
    const uint32_t rf_freq_in_hz );

/**
 * @brief Function pointer to abstract RF switch configuration getter
 */
typedef const lr11xx_system_rfswitch_cfg_t* ( *smtc_shield_lr11xx_get_rf_switch_cfg_f )( void );

/**
 * @brief Function pointer to abstract regulator mode configuration getter
 */
typedef lr11xx_system_reg_mode_t ( *smtc_shield_lr11xx_get_reg_mode_f )( void );

/**
 * @brief Function pointer to abstract XOSC configuration getter
 */
typedef const smtc_shield_lr11xx_xosc_cfg_t* ( *smtc_shield_lr11xx_get_xosc_cfg_f )( void );

/**
 * @brief Function pointer to abstract LF clock configuration getter
 */
typedef const smtc_shield_lr11xx_lfclk_cfg_t* ( *smtc_shield_lr11xx_get_lfclk_cfg_f )( void );

/**
 * @brief Function pointer to abstract pinout getter
 */
typedef const smtc_shield_lr11xx_pinout_t* ( *smtc_shield_lr11xx_get_pinout_f )( void );

/**
 * @brief Function pointer to abstract RTToF recommented Tx/Rx delay indicator getter
 */
typedef bool ( *smtc_shield_lr11xx_get_rttof_recommended_rx_tx_delay_indicator_f )( lr11xx_radio_lora_bw_t bw,
                                                                                    lr11xx_radio_lora_sf_t sf,
                                                                                    uint32_t* delay_indicator );

typedef struct smtc_shield_lr11xx_s
{
    smtc_shield_lr11xx_get_pa_pwr_cfg_f                              get_pa_pwr_cfg;
    smtc_shield_lr11xx_get_rssi_calibration_table_f                  get_rssi_calibration_table;
    smtc_shield_lr11xx_get_rf_switch_cfg_f                           get_rf_switch_cfg;
    smtc_shield_lr11xx_get_reg_mode_f                                get_reg_mode;
    smtc_shield_lr11xx_get_xosc_cfg_f                                get_xosc_cfg;
    smtc_shield_lr11xx_get_lfclk_cfg_f                               get_lfclk_cfg;
    smtc_shield_lr11xx_get_pinout_f                                  get_pinout;
    smtc_shield_lr11xx_get_rttof_recommended_rx_tx_delay_indicator_f get_rttof_recommended_rx_tx_delay_indicator;
} smtc_shield_lr11xx_t;

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
static inline const smtc_shield_lr11xx_pa_pwr_cfg_t* smtc_shield_lr11xx_get_pa_pwr_cfg(
    const smtc_shield_lr11xx_t* shield, const uint32_t rf_freq_in_hz, int8_t expected_output_pwr_in_dbm )
{
    return shield->get_pa_pwr_cfg( rf_freq_in_hz, expected_output_pwr_in_dbm );
}

/**
 * @brief Return the RSSI calibration table
 *
 * @param [in] shield  Pointer to a shield data structure
 * @param [in] rf_freq_in_hz  RF frequency in Hertz
 *
 * @return A pointer to a RSSI calibration table corresponding to the input configuration
 */
static inline const lr11xx_radio_rssi_calibration_table_t* smtc_shield_lr11xx_get_rssi_calibration_table(
    const smtc_shield_lr11xx_t* shield, const uint32_t rf_freq_in_hz )
{
    return shield->get_rssi_calibration_table( rf_freq_in_hz );
}

/**
 * @brief Return the RF switch configuration
 *
 * @param [in] shield  Pointer to a shield data structure
 *
 * @return RF switch configuration
 */
static inline const lr11xx_system_rfswitch_cfg_t* smtc_shield_lr11xx_get_rf_switch_cfg(
    const smtc_shield_lr11xx_t* shield )
{
    return shield->get_rf_switch_cfg( );
}

/**
 * @brief Return the regulator mode
 *
 * @param [in] shield  Pointer to a shield data structure
 *
 * @return Regulator mode
 */
static inline lr11xx_system_reg_mode_t smtc_shield_lr11xx_get_reg_mode( const smtc_shield_lr11xx_t* shield )
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
static inline const smtc_shield_lr11xx_xosc_cfg_t* smtc_shield_lr11xx_get_xosc_cfg( const smtc_shield_lr11xx_t* shield )
{
    return shield->get_xosc_cfg( );
}

/**
 * @brief Return the 32kHz LFCLOCK (Low-Frequency Clock) configuration
 *
 * @param [in] shield  Pointer to a shield data structure
 *
 * @return LFCLOCK configuration
 */
static inline const smtc_shield_lr11xx_lfclk_cfg_t* smtc_shield_lr11xx_get_lfclk_cfg(
    const smtc_shield_lr11xx_t* shield )
{
    return shield->get_lfclk_cfg( );
}

/**
 * @brief Return the board pinout configuration
 *
 * @param [in] shield  Pointer to a shield data structure
 *
 * @return Pinout configuration
 */
static inline const smtc_shield_lr11xx_pinout_t* smtc_shield_lr11xx_get_pinout( const smtc_shield_lr11xx_t* shield )
{
    return shield->get_pinout( );
}

/**
 * @brief Return the RTToF recommended Rx/Tx delay indicator
 *
 * @param [in] shield  Pointer to a shield data structure
 * @param [in] bw Bandwidth to get the recommended delay indicator for
 * @param [in] sf Spreading factor to get the recommended delay indicator for
 * @param [out] delay_indicator Recommended delay indicator corresponding to sf/bw. Only valid if the call returned
 * true
 *
 * @retval true The RTToF recommanded Rx/Tx delay indicator has been found and can be used
 * @retval false The RTToF recommanded Rx/Tx delay indicator has not been found and must not be used
 */
static inline bool smtc_shield_lr11xx_get_rttof_recommended_rx_tx_delay_indicator( const smtc_shield_lr11xx_t* shield,
                                                                                   lr11xx_radio_lora_bw_t      bw,
                                                                                   lr11xx_radio_lora_sf_t      sf,
                                                                                   uint32_t* delay_indicator )
{
    return shield->get_rttof_recommended_rx_tx_delay_indicator( bw, sf, delay_indicator );
}

#ifdef __cplusplus
}
#endif

#endif  // SMTC_SHIELD_LR11XX_H

/* --- EOF ------------------------------------------------------------------ */
