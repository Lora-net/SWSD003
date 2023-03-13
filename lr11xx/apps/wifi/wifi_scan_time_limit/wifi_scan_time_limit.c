/*!
 * @file      wifi_scan_time_limit.c
 *
 * @brief     Wi-Fi scan time limit example for LR11xx chip
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
#include "main_wifi.h"
#include "wifi_scan_time_limit.h"
#include "lr11xx_wifi.h"
#include "smtc_hal_dbg_trace.h"
#include "apps_utilities.h"
#include "lr11xx_wifi_types_str.h"
#include "wifi_configuration_base.h"
#include "wifi_result_printers.h"

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

typedef struct
{
    wifi_configuration_scan_base_t base;
    lr11xx_wifi_signal_type_scan_t signal_type;
    lr11xx_wifi_mode_t             scan_mode;
    uint16_t                       timeout_per_channel;
    uint16_t                       timeout_per_scan;
    uint32_t                       max_number_of_scan;
} wifi_configuration_scan_time_limit_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static const wifi_configuration_scan_time_limit_t wifi_configuration = {
    .signal_type         = WIFI_SIGNAL_TYPE,
    .base.channel_mask   = WIFI_CHANNEL_MASK,
    .scan_mode           = WIFI_SCAN_MODE,
    .base.max_result     = WIFI_MAX_RESULTS,
    .timeout_per_channel = WIFI_TIMEOUT_PER_CHANNEL,
    .timeout_per_scan    = WIFI_TIMEOUT_PER_SCAN_TIME_LIMIT,
    .max_number_of_scan  = WIFI_MAX_NUMBER_OF_SCAN,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

const char* wifi_scan_time_limit_get_example_name( void )
{
    return ( const char* ) "Wi-Fi passive scan time limit";
}

void wifi_scan_time_limit_call_api_scan( const void* context )
{
    ASSERT_LR11XX_RC(
        lr11xx_wifi_scan_time_limit( context, wifi_configuration.signal_type, wifi_configuration.base.channel_mask,
                                     wifi_configuration.scan_mode, wifi_configuration.base.max_result,
                                     wifi_configuration.timeout_per_channel, wifi_configuration.timeout_per_scan ) );
}

void wifi_scan_time_limit_print_configuration( void )
{
    HAL_DBG_TRACE_PRINTF( "Wi-Fi example configuration:\n" );
    HAL_DBG_TRACE_PRINTF( "  -> channel mask: 0x%04X\n", wifi_configuration.base.channel_mask );
    HAL_DBG_TRACE_PRINTF( "  -> max result: %u\n", wifi_configuration.base.max_result );
    HAL_DBG_TRACE_PRINTF( "  -> scan mode: %s\n", lr11xx_wifi_mode_to_str( wifi_configuration.scan_mode ) );
    HAL_DBG_TRACE_PRINTF( "  -> signal type: %s\n",
                          lr11xx_wifi_signal_type_scan_to_str( wifi_configuration.signal_type ) );
    HAL_DBG_TRACE_PRINTF( "  -> timeout per channel: %u\n", wifi_configuration.timeout_per_channel );
    HAL_DBG_TRACE_PRINTF( "  -> timeout per scan: %u\n", wifi_configuration.timeout_per_scan );
    HAL_DBG_TRACE_PRINTF( "  -> max number of scan: %lu\n", wifi_configuration.max_number_of_scan );
    HAL_DBG_TRACE_PRINTF( "\n" );
}