/*!
 * @file      smtc_shield_sx1261mb1cas_cfg.c
 *
 * @brief     Implementation specific to SX1261MB1CAS shield
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "smtc_shield_sx126x_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

const smtc_shield_sx126x_pa_pwr_cfg_t smtc_shield_sx1261mb1cas_pa_pwr_cfg_table[SMTC_SHIELD_SX1261_MAX_PWR - SMTC_SHIELD_SX1261_MIN_PWR + 1] = {
    { // Expected output power = -17dBm
        .power = -15,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -16dBm
        .power = -14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -15dBm
        .power = -15,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -14dBm
        .power = -11,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -13dBm
        .power = -13,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -12dBm
        .power = -9,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -11dBm
        .power = -8,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -10dBm
        .power = -7,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -9dBm
        .power = -8,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -8dBm
        .power = -7,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -7dBm
        .power = -6,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -6dBm
        .power = -4,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -5dBm
        .power = -3,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -4dBm
        .power = -2,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -3dBm
        .power = -1,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -2dBm
        .power = 1,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = -1dBm
        .power = 1,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 0dBm
        .power = 2,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 1dBm
        .power = 5,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 2dBm
        .power = 6,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 3dBm
        .power = 7,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 4dBm
        .power = 8,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 5dBm
        .power = 8,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 6dBm
        .power = 9,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 7dBm
        .power = 12,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 8dBm
        .power = 13,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x00,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 9dBm
        .power = 13,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 10dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x01,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 11dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x02,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 12dBm
        .power = 12,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x05,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 13dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x03,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 14dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x04,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
    { // Expected output power = 15dBm
        .power = 14,
        .pa_config = {
            .hp_max        = 0x00,
            .pa_duty_cycle = 0x05,
            .device_sel    = 0x01,
            .pa_lut        = 0x01,
        },
    },
};

/**
 * @brief XOSC configuration
 */
const smtc_shield_sx126x_xosc_cfg_t smtc_shield_sx1261mb1cas_xosc_cfg = {
    .tcxo_is_radio_controlled = true,
    .supply_voltage           = SX126X_TCXO_CTRL_3_0V,
    .startup_time_in_tick     = 300,
};

/**
 * @brief GPIO configuration
 */
const smtc_shield_sx126x_pinout_t smtc_shield_sx1261mb1cas_pinout = {
    .busy       = SMTC_SHIELD_PINOUT_D3,
    .irq        = SMTC_SHIELD_PINOUT_D5,
    .nss        = SMTC_SHIELD_PINOUT_D7,
    .reset      = SMTC_SHIELD_PINOUT_A0,
    .antenna_sw = SMTC_SHIELD_PINOUT_D8,
    .led_tx     = SMTC_SHIELD_PINOUT_A4,
    .led_rx     = SMTC_SHIELD_PINOUT_A5,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

const smtc_shield_sx126x_pa_pwr_cfg_t* smtc_shield_sx1261mb1cas_get_pa_pwr_cfg( const uint32_t rf_freq_in_hz,
                                                                                int8_t expected_output_pwr_in_dbm )
{
    if( ( SMTC_SHIELD_SX126X_FREQ_MIN <= rf_freq_in_hz ) && ( rf_freq_in_hz <= SMTC_SHIELD_SX126X_FREQ_MAX ) )
    {
        if( ( SMTC_SHIELD_SX1261_MIN_PWR <= expected_output_pwr_in_dbm ) &&
            ( expected_output_pwr_in_dbm <= SMTC_SHIELD_SX1261_MAX_PWR ) )
        {
            return &(
                smtc_shield_sx1261mb1cas_pa_pwr_cfg_table[expected_output_pwr_in_dbm - SMTC_SHIELD_SX1261_MIN_PWR] );
        }
    }

    return NULL;
}

bool smtc_shield_sx1261mb1cas_is_dio2_set_as_rf_switch( void )
{
    return true;
}

sx126x_reg_mod_t smtc_shield_sx1261mb1cas_get_reg_mode( void )
{
    return SX126X_REG_MODE_DCDC;
}

const smtc_shield_sx126x_xosc_cfg_t* smtc_shield_sx1261mb1cas_get_xosc_cfg( void )
{
    return &smtc_shield_sx1261mb1cas_xosc_cfg;
}

const smtc_shield_sx126x_pinout_t* smtc_shield_sx1261mb1cas_get_pinout( void )
{
    return &smtc_shield_sx1261mb1cas_pinout;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
