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

bool smtc_shield_lr11x0_common_rttof_recommended_rx_tx_delay_indicator( lr11xx_radio_lora_bw_t bw,
                                                                        lr11xx_radio_lora_sf_t sf,
                                                                        uint32_t*              delay_indicator )
{
    bool found = true;

    *delay_indicator = 0u;

    if( bw == LR11XX_RADIO_LORA_BW_500 )
    {
        if( sf == LR11XX_RADIO_LORA_SF5 )
        {
            *delay_indicator = 20149u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF6 )
        {
            *delay_indicator = 20227u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF7 )
        {
            *delay_indicator = 20258u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF8 )
        {
            *delay_indicator = 20277u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF9 )
        {
            *delay_indicator = 20286u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF10 )
        {
            *delay_indicator = 20292u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF11 )
        {
            *delay_indicator = 20295u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF12 )
        {
            *delay_indicator = 20298u;
        }
        else
        {
            found = false;
        }
    }
    else if( bw == LR11XX_RADIO_LORA_BW_250 )
    {
        if( sf == LR11XX_RADIO_LORA_SF5 )
        {
            *delay_indicator = 20235u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF6 )
        {
            *delay_indicator = 20239u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF7 )
        {
            *delay_indicator = 20238u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF8 )
        {
            *delay_indicator = 20237u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF9 )
        {
            *delay_indicator = 20236u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF10 )
        {
            *delay_indicator = 20235u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF11 )
        {
            *delay_indicator = 20236u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF12 )
        {
            *delay_indicator = 20232u;
        }
        else
        {
            found = false;
        }
    }
    else if( bw == LR11XX_RADIO_LORA_BW_125 )
    {
        if( sf == LR11XX_RADIO_LORA_SF5 )
        {
            *delay_indicator = 19035u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF6 )
        {
            *delay_indicator = 19040u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF7 )
        {
            *delay_indicator = 19040u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF8 )
        {
            *delay_indicator = 19039u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF9 )
        {
            *delay_indicator = 19036u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF10 )
        {
            *delay_indicator = 19038u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF11 )
        {
            *delay_indicator = 19036u;
        }
        else if( sf == LR11XX_RADIO_LORA_SF12 )
        {
            *delay_indicator = 19024u;
        }
        else
        {
            found = false;
        }
    }
    else
    {
        found = false;
    }
    return found;
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
