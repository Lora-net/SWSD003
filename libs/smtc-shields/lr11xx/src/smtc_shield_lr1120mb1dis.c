/*!
 * @file      smtc_shield_lr1120mb1dis.c
 *
 * @brief     Implementation specific to LR1120MB1DIS shield
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
#include "smtc_shield_lr11xx_types.h"
#include "smtc_shield_lr1120mb1dxs_common.h"
#include "smtc_shield_lr11xx_common.h"
#include "smtc_shield_lr11x0_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

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

const smtc_shield_lr11xx_pa_pwr_cfg_t* smtc_shield_lr1120mb1dis_get_pa_pwr_cfg( const uint32_t rf_freq_in_hz,
                                                                                int8_t expected_output_pwr_in_dbm )
{
    return smtc_shield_lr1120mb1dxs_common_get_pa_pwr_cfg( rf_freq_in_hz, expected_output_pwr_in_dbm );
}

const lr11xx_radio_rssi_calibration_table_t* smtc_shield_lr1120mb1dis_get_rssi_calibration_table(
    const uint32_t rf_freq_in_hz )
{
    return smtc_shield_lr11xx_get_rssi_calibration_table( rf_freq_in_hz );
}

const lr11xx_system_rfswitch_cfg_t* smtc_shield_lr1120mb1dis_get_rf_switch_cfg( void )
{
    return smtc_shield_lr11xx_common_get_rf_switch_cfg( );
}

lr11xx_system_reg_mode_t smtc_shield_lr1120mb1dis_get_reg_mode( void )
{
    return smtc_shield_lr11xx_common_get_reg_mode( );
}

const smtc_shield_lr11xx_xosc_cfg_t* smtc_shield_lr1120mb1dis_get_xosc_cfg( void )
{
    return smtc_shield_lr11x0_common_get_xosc_cfg( );
}

const smtc_shield_lr11xx_lfclk_cfg_t* smtc_shield_lr1120mb1dis_get_lfclk_cfg( void )
{
    return smtc_shield_lr11xx_common_get_lfclk_cfg( );
}

const smtc_shield_lr11xx_pinout_t* smtc_shield_lr1120mb1dis_get_pinout( void )
{
    return smtc_shield_lr11xx_common_get_pinout( );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
