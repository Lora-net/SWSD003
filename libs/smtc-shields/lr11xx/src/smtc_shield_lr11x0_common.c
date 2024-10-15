/*!
 * @file      smtc_shield_lr11x0_common.c
 *
 * @brief     Implementation common to LR11x0 shield
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "smtc_shield_lr11xx_types.h"
#include "smtc_shield_lr11x0_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * @brief Recommanded for frequency from 430MHz to 510MHz
 */
static const uint32_t rttof_delay_indicator_table_below_600mhz[3][8] = {
    /* SF5,  SF6,   SF7,   SF8,   SF9,   SF10,  SF11,  SF12 */
    { 19107, 19122, 19113, 19114, 19113, 19114, 19036, 19024 },  // BW125
    { 20265, 20279, 20278, 20273, 20270, 20272, 20236, 20232 },  // BW250
    { 20166, 20249, 20288, 20309, 20318, 20312, 20295, 20298 },  // BW500
};

/*!
 * @brief Recommanded for frequency from 860MHz to 928MHz
 */
static const uint32_t rttof_delay_indicator_table_from_600mhz_to_2ghz[3][8] = {
    /* SF5,  SF6,   SF7,   SF8,   SF9,   SF10,  SF11,  SF12 */
    { 19115, 19113, 19121, 19127, 19141, 19178, 19036, 19024 },  // BW125
    { 20265, 20266, 20279, 20292, 20236, 20305, 20236, 20232 },  // BW250
    { 20154, 20268, 20298, 20319, 20323, 20314, 20295, 20298 },  // BW500
};

/*!
 * @brief Recommanded for 2.4G frequency
 */
static const uint32_t rttof_delay_indicator_table_above_2ghz[3][8] = {
    /* SF5,  SF6,   SF7,   SF8,   SF9,   SF10,  SF11,  SF12 */
    { 19118, 19123, 19120, 19124, 19121, 19119, 19036, 19024 },  // BW125
    { 20221, 20230, 20226, 20231, 20236, 20223, 20236, 20232 },  // BW250
    { 20143, 20230, 20252, 20284, 20305, 20288, 20295, 20298 },  // BW500
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

const smtc_shield_lr11xx_xosc_cfg_t smtc_shield_lr11x0_common_xosc_cfg = {
    .has_tcxo             = true,
    .supply               = LR11XX_SYSTEM_TCXO_CTRL_3_0V,
    .startup_time_in_tick = 300,
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

const smtc_shield_lr11xx_xosc_cfg_t* smtc_shield_lr11x0_common_get_xosc_cfg( void )
{
    return &smtc_shield_lr11x0_common_xosc_cfg;
}

bool smtc_shield_lr11x0_common_rttof_recommended_rx_tx_delay_indicator( uint32_t               rf_freq_in_hz,
                                                                        lr11xx_radio_lora_bw_t bw,
                                                                        lr11xx_radio_lora_sf_t sf,
                                                                        uint32_t*              delay_indicator )
{
    uint8_t row_index;
    uint8_t column_index;

    *delay_indicator = 0u;

    switch( bw )
    {
    case LR11XX_RADIO_LORA_BW_125:
        row_index = 0;
        break;
    case LR11XX_RADIO_LORA_BW_250:
        row_index = 1;
        break;
    case LR11XX_RADIO_LORA_BW_500:
        row_index = 2;
        break;
    default:
        return false;
    }

    switch( sf )
    {
    case LR11XX_RADIO_LORA_SF5:
        column_index = 0;
        break;
    case LR11XX_RADIO_LORA_SF6:
        column_index = 1;
        break;
    case LR11XX_RADIO_LORA_SF7:
        column_index = 2;
        break;
    case LR11XX_RADIO_LORA_SF8:
        column_index = 3;
        break;
    case LR11XX_RADIO_LORA_SF9:
        column_index = 4;
        break;
    case LR11XX_RADIO_LORA_SF10:
        column_index = 5;
        break;
    case LR11XX_RADIO_LORA_SF11:
        column_index = 6;
        break;
    case LR11XX_RADIO_LORA_SF12:
        column_index = 7;
        break;
    default:
        return false;
    }

    if( rf_freq_in_hz < 600000000 )
    {
        *delay_indicator = rttof_delay_indicator_table_below_600mhz[row_index][column_index];
    }
    else if( ( 600000000 <= rf_freq_in_hz ) && ( rf_freq_in_hz < 2000000000 ) )
    {
        *delay_indicator = rttof_delay_indicator_table_from_600mhz_to_2ghz[row_index][column_index];
    }
    else
    {
        *delay_indicator = rttof_delay_indicator_table_above_2ghz[row_index][column_index];
    }

    return true;
}

void smtc_shield_lr11x0_common_gnss_consumption_instantaneous_value(
    lr11xx_gnss_instantaneous_power_consumption_ua_t* instantaneous_power_consumption_ua )
{
    /* These value are for EVK board in DC DC mode with Xtal 32KHz and a TCXO 32MHz*/
    instantaneous_power_consumption_ua->board_voltage_mv            = 3300;
    instantaneous_power_consumption_ua->init_ua                     = 3150;
    instantaneous_power_consumption_ua->phase1_gps_capture_ua       = 11900;
    instantaneous_power_consumption_ua->phase1_gps_process_ua       = 3340;
    instantaneous_power_consumption_ua->multiscan_gps_capture_ua    = 10700;
    instantaneous_power_consumption_ua->multiscan_gps_process_ua    = 4180;
    instantaneous_power_consumption_ua->phase1_beidou_capture_ua    = 13500;
    instantaneous_power_consumption_ua->phase1_beidou_process_ua    = 3190;
    instantaneous_power_consumption_ua->multiscan_beidou_capture_ua = 12600;
    instantaneous_power_consumption_ua->multiscan_beidou_process_ua = 3430;
    instantaneous_power_consumption_ua->sleep_32k_ua                = 1210;
    instantaneous_power_consumption_ua->demod_sleep_32m_ua          = 2530;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
