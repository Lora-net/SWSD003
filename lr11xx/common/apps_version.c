/*!
 * @file      apps_version.c
 *
 * @brief     LR11xx version related functions shared by the examples
 *
 * @copyright
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

#include "apps_version.h"
#include "apps_common.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
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

/**
 * @brief Validate that version corresponds to LR1110
 *
 * @param [in] version Version structure as fetched from device
 *
 * @return true The version correspond to LR1110
 * @return false The version does not correspond to LR1110
 */
bool apps_version_assert_version_is_lr1110( const lr11xx_system_version_t* version );

/**
 * @brief Validate that the version corresponds to LR1120
 *
 * @param [in] version Version structure as fetched from device
 *
 * @return true The version correspond to LR1120
 * @return false The version does not correspond to LR1120
 */
bool apps_version_assert_version_is_lr1120( const lr11xx_system_version_t* version );

/**
 * @brief Validate that the version corresponds to geolocation capable device
 *
 * A geolocation capable is either a LR1110 or a LR1120.
 *
 * @param [in] version Version structure as fetched from device
 *
 * @return true The version correspond to a geolocation capable device
 * @return false The version does not correspond to a geolocation capable device
 */
bool apps_version_assert_version_is_geolocation_capable( const lr11xx_system_version_t* version );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

const apps_version_validator_t apps_version_validator_is_geolocation_capable =
    apps_version_assert_version_is_geolocation_capable;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

bool apps_version_fetch_check_print_version( const void* context, const apps_version_validator_t validator )
{
    lr11xx_system_version_t version = { 0 };
    apps_common_lr11xx_fetch_version( context, &version );
    const bool is_expected = validator( &version );
    apps_common_lr11xx_print_version( &version );
    return is_expected;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

bool apps_version_assert_version_is_lr1110( const lr11xx_system_version_t* version )
{
    return version->type == 0x01;
}

bool apps_version_assert_version_is_lr1120( const lr11xx_system_version_t* version )
{
    return version->type == 0x02;
}

bool apps_version_assert_version_is_geolocation_capable( const lr11xx_system_version_t* version )
{
    return apps_version_assert_version_is_lr1110( version ) || apps_version_assert_version_is_lr1120( version );
}

/* --- EOF ------------------------------------------------------------------ */
