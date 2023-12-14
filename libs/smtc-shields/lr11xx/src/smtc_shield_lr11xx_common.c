/*!
 * @file      smtc_shield_lr11xx_common.c
 *
 * @brief     Implementation common to LR11xx shield
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
#include "smtc_shield_pinout.h"
#include "smtc_shield_lr11xx_types.h"
#include "smtc_shield_lr11xx_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static const lr11xx_radio_rssi_calibration_table_t smtc_shield_lr11xx_common_rssi_calibration_table_below_600mhz = {
    .gain_offset = 0,
    .gain_tune   = { .g4     = 12,
                     .g5     = 12,
                     .g6     = 14,
                     .g7     = 0,
                     .g8     = 1,
                     .g9     = 3,
                     .g10    = 4,
                     .g11    = 4,
                     .g12    = 3,
                     .g13    = 6,
                     .g13hp1 = 6,
                     .g13hp2 = 6,
                     .g13hp3 = 6,
                     .g13hp4 = 6,
                     .g13hp5 = 6,
                     .g13hp6 = 6,
                     .g13hp7 = 6 },
};

static const lr11xx_radio_rssi_calibration_table_t
    smtc_shield_lr11xx_common_rssi_calibration_table_from_600mhz_to_2ghz = {
        .gain_offset = 0,
        .gain_tune   = { .g4     = 2,
                         .g5     = 2,
                         .g6     = 2,
                         .g7     = 3,
                         .g8     = 3,
                         .g9     = 4,
                         .g10    = 5,
                         .g11    = 4,
                         .g12    = 4,
                         .g13    = 6,
                         .g13hp1 = 5,
                         .g13hp2 = 5,
                         .g13hp3 = 6,
                         .g13hp4 = 6,
                         .g13hp5 = 6,
                         .g13hp6 = 7,
                         .g13hp7 = 6 },
    };

static const lr11xx_radio_rssi_calibration_table_t smtc_shield_lr11xx_common_rssi_calibration_table_above_2ghz = {
    .gain_offset = 2030,
    .gain_tune   = { .g4     = 6,
                     .g5     = 7,
                     .g6     = 6,
                     .g7     = 4,
                     .g8     = 3,
                     .g9     = 4,
                     .g10    = 14,
                     .g11    = 12,
                     .g12    = 14,
                     .g13    = 12,
                     .g13hp1 = 12,
                     .g13hp2 = 12,
                     .g13hp3 = 12,
                     .g13hp4 = 8,
                     .g13hp5 = 8,
                     .g13hp6 = 9,
                     .g13hp7 = 9 },
};

const lr11xx_system_rfswitch_cfg_t smtc_shield_lr11xx_common_rf_switch_cfg = {
    .enable  = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH | LR11XX_SYSTEM_RFSW2_HIGH,
    .standby = 0,
    .rx      = LR11XX_SYSTEM_RFSW0_HIGH,
    .tx      = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,
    .tx_hp   = LR11XX_SYSTEM_RFSW1_HIGH,
    .tx_hf   = 0,
    .gnss    = LR11XX_SYSTEM_RFSW2_HIGH,
    .wifi    = 0,
};

const smtc_shield_lr11xx_lfclk_cfg_t smtc_shield_lr11xx_common_lfclk_cfg = {
    .lf_clk_cfg     = LR11XX_SYSTEM_LFCLK_XTAL,
    .wait_32k_ready = true,
};

const smtc_shield_lr11xx_pinout_t smtc_shield_lr11xx_common_pinout = {
    .busy     = SMTC_SHIELD_PINOUT_D3,
    .irq      = SMTC_SHIELD_PINOUT_D5,
    .nss      = SMTC_SHIELD_PINOUT_D7,
    .reset    = SMTC_SHIELD_PINOUT_A0,
    .lna      = SMTC_SHIELD_PINOUT_A3,
    .led_tx   = SMTC_SHIELD_PINOUT_A4,
    .led_rx   = SMTC_SHIELD_PINOUT_A5,
    .led_scan = SMTC_SHIELD_PINOUT_D4,
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

const lr11xx_radio_rssi_calibration_table_t* smtc_shield_lr11xx_get_rssi_calibration_table(
    const uint32_t rf_freq_in_hz )
{
    if( rf_freq_in_hz < 600000000 )
    {
        return &smtc_shield_lr11xx_common_rssi_calibration_table_below_600mhz;
    }
    else if( ( 600000000 <= rf_freq_in_hz ) && ( rf_freq_in_hz <= 2000000000 ) )
    {
        return &smtc_shield_lr11xx_common_rssi_calibration_table_from_600mhz_to_2ghz;
    }
    else
    {
        return &smtc_shield_lr11xx_common_rssi_calibration_table_above_2ghz;
    }
}

const lr11xx_system_rfswitch_cfg_t* smtc_shield_lr11xx_common_get_rf_switch_cfg( void )
{
    return &smtc_shield_lr11xx_common_rf_switch_cfg;
}

lr11xx_system_reg_mode_t smtc_shield_lr11xx_common_get_reg_mode( void )
{
    return LR11XX_SYSTEM_REG_MODE_DCDC;
}

const smtc_shield_lr11xx_lfclk_cfg_t* smtc_shield_lr11xx_common_get_lfclk_cfg( void )
{
    return &smtc_shield_lr11xx_common_lfclk_cfg;
}

const smtc_shield_lr11xx_pinout_t* smtc_shield_lr11xx_common_get_pinout( void )
{
    return &smtc_shield_lr11xx_common_pinout;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
