/*!
 * @file      smtc_shield_lr11x0mb1pis_common.c
 *
 * @brief     Implementation common to LR11X0MB1PIS shields
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief LR11x0MB1PIS power amplifier configuration that passes ARIB
 *
 * This configuration has been used to pass harmonic test for ARIB regulation.
 * It is not known to be power consumption optimized.
 */
static const smtc_shield_lr11xx_pa_pwr_cfg_t
    smtc_shield_lr11x0mb1pis_common_pa_pwr_cfg_6dbm_arib = { .power     = 9,
                                                             .pa_config = {
                                                                 .pa_sel        = LR11XX_RADIO_PA_SEL_LP,
                                                                 .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
                                                                 .pa_duty_cycle = 0x04,
                                                                 .pa_hp_sel     = 0x00,
                                                             } };

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

const smtc_shield_lr11xx_pa_pwr_cfg_t* smtc_shield_lr11x0mb1pis_common_get_pa_pwr_cfg(
    const uint32_t rf_freq_in_hz, int8_t expected_output_pwr_in_dbm )
{
    if( ( SMTC_SHIELD_LR11XX_SUBGHZ_FREQ_MIN <= rf_freq_in_hz ) &&
        ( rf_freq_in_hz <= SMTC_SHIELD_LR11XX_SUBGHZ_FREQ_MAX ) )
    {
        // If the expected output power correspond to the one used for ARIB test: return it
        if( expected_output_pwr_in_dbm == 6 )
        {
            return &smtc_shield_lr11x0mb1pis_common_pa_pwr_cfg_6dbm_arib;
        }
        // If the expected output power is inferior to the one for ARIB test, return the same configuration as the one
        // for MB1DxS shields. It has however not been tested for ARIB harmonic tests
        else if( expected_output_pwr_in_dbm < 6 )
        {
            return smtc_shield_lr1110mb1dxs_common_get_pa_pwr_cfg( rf_freq_in_hz, expected_output_pwr_in_dbm );
        }
        // If the expected output power is superior to the one for a ARIB test, return NULL as it may not be compatible
        // with the shield
        else
        {
            return NULL;
        }
    }

    return NULL;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
