/*!
 * @file      smtc_shield_lr11x0mb1ipddis_common.c
 *
 * @brief     Implementation common to LR11x0MB1IPDDIS shields
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
#include "smtc_shield_lr1110mb1dxs_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief This macro is used to determine if the Power Amplifier configuration to apply is for FCC or ETSI
 */
#define SMTC_SHIELD_LR11XX_SUBGHZ_FREQ_FCC_ETSI_LIMIT_DEFAULT ( 900000000 )

#ifndef SMTC_SHIELD_LR11XX_SUBGHZ_FREQ_FCC_ETSI_LIMIT
#define SMTC_SHIELD_LR11XX_SUBGHZ_FREQ_FCC_ETSI_LIMIT SMTC_SHIELD_LR11XX_SUBGHZ_FREQ_FCC_ETSI_LIMIT_DEFAULT
#endif  // SMTC_SHIELD_LR11XX_SUBGHZ_FREQ_FCC_ETSI_LIMIT

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Power Amplifier configuration for ETSI compliance at +15dBm for sub-ghz operations
 */
const smtc_shield_lr11xx_pa_pwr_cfg_t smtc_shield_lr11x0mb1ipddis_common_etsi_15dbm_pa_pwr_cfg = {
    .power = 14,
    .pa_config = {
        .pa_sel        = LR11XX_RADIO_PA_SEL_LP,
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
        .pa_duty_cycle = 0x06,
        .pa_hp_sel     = 0x00,
    },
};

/**
 * @brief Power Amplifier configuration for FCC compliance at +22dBm for sub-ghz operations
 */
const smtc_shield_lr11xx_pa_pwr_cfg_t smtc_shield_lr11x0mb1ipddis_common_fcc_22dbm_pa_pwr_cfg = {
    .power = 22,
    .pa_config = {
        .pa_sel        = LR11XX_RADIO_PA_SEL_HP,
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_duty_cycle = 0x04,
        .pa_hp_sel     = 0x07,
    },
};

/**
 * @brief Power Amplifier configuration for FCC compliance at +17dBm for sub-ghz operations
 */
const smtc_shield_lr11xx_pa_pwr_cfg_t smtc_shield_lr11x0mb1ipddis_common_fcc_17dbm_pa_pwr_cfg = {
    .power = 22,
    .pa_config = {
        .pa_sel        = LR11XX_RADIO_PA_SEL_HP,
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_duty_cycle = 0x05,
        .pa_hp_sel     = 0x03,
    },
};

/**
 * @brief Power Amplifier configuration for FCC compliance at +15dBm for sub-ghz operations
 */
const smtc_shield_lr11xx_pa_pwr_cfg_t smtc_shield_lr11x0mb1ipddis_common_fcc_15dbm_pa_pwr_cfg = {
    .power = 22,
    .pa_config = {
        .pa_sel        = LR11XX_RADIO_PA_SEL_HP,
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_duty_cycle = 0x01,
        .pa_hp_sel     = 0x03,
    },
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

const smtc_shield_lr11xx_pa_pwr_cfg_t* smtc_shield_lr11x0mb1ipddis_common_get_pa_pwr_cfg(
    const uint32_t rf_freq_in_hz, int8_t expected_output_pwr_in_dbm )
{
    if( rf_freq_in_hz < SMTC_SHIELD_LR11XX_SUBGHZ_FREQ_FCC_ETSI_LIMIT )
    {
        // Searching for ETSI Power Amplifier configuration
        if( expected_output_pwr_in_dbm == 15 )
        {
            return &smtc_shield_lr11x0mb1ipddis_common_etsi_15dbm_pa_pwr_cfg;
        }
    }
    else
    {
        // Searching for FCC Power Amplifier configuration
        if( expected_output_pwr_in_dbm == 22 )
        {
            return &smtc_shield_lr11x0mb1ipddis_common_fcc_22dbm_pa_pwr_cfg;
        }
        else if( expected_output_pwr_in_dbm == 17 )
        {
            return &smtc_shield_lr11x0mb1ipddis_common_fcc_17dbm_pa_pwr_cfg;
        }
        else if( expected_output_pwr_in_dbm == 15 )
        {
            return &smtc_shield_lr11x0mb1ipddis_common_fcc_15dbm_pa_pwr_cfg;
        }
    }
    // If this point is reached, the configuration has not been found: we fall back to default values from
    // smtc_shield_lr1110mb1dxs_common
    return smtc_shield_lr1110mb1dxs_common_get_pa_pwr_cfg( rf_freq_in_hz, expected_output_pwr_in_dbm );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
