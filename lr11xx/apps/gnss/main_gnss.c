/*!
 * @file      main_gnss.c
 *
 * @brief     GNSS scan example for LR11xx chip
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "apps_common.h"
#include "apps_version.h"
#include "lr11xx_system.h"
#include "main_gnss.h"
#include "lr11xx_gnss.h"
#include "apps_utilities.h"
#include "smtc_hal_dbg_trace.h"

#include "gnss_example_api.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define NAV_MAX_LENGTH ( 300 )

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK ( LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_hal_context_t* context;
static uint32_t              number_of_scan = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Handle the start of a new GNSS scan
 *
 * It also check that a new Wi-Fi scan can start
 *
 */
static void start_scan( );

/**
 * @brief Handle the printing of results
 *
 * It includes the printing of detected SVs and cumulative timings.
 *
 */
static void fetch_and_print_results( void );

/**
 * @brief Helper function that indicate if a new scan can be started or if the demo is terminated
 *
 * @return true New scan can start
 * @return false The demo is terminated
 */
static bool can_execute_next_scan( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    smtc_hal_mcu_init( );
    apps_common_shield_init( );
    gnss_uart_init();

    HAL_DBG_TRACE_INFO( "===== %s =====\n\n", gnss_get_example_name( ) );
    apps_common_print_sdk_driver_version( );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );

    const bool is_geolocation_capable =
        apps_version_fetch_check_print_version( ( void* ) context, apps_version_validator_is_geolocation_capable );
    if( is_geolocation_capable == false )
    {
        HAL_DBG_TRACE_ERROR( "Fatal error: the device is not Geolocation capable." );
        while( true )
        {
        }
    }

    ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );

    gnss_init( context );
    start_scan( );

    while( 1 )
    {
        apps_common_lr11xx_irq_process( context, IRQ_MASK );
    }
}

void start_scan( void )
{
    if( can_execute_next_scan( ) )
    {
        apps_common_lr11xx_handle_pre_gnss_scan( );
        gnss_call_scan( context );
    }
}

static void fetch_and_print_results( void )
{
    HAL_DBG_TRACE_PRINTF( "== Scan #%lu ==\n", number_of_scan );

    lr11xx_gnss_timings_t gnss_timing = { 0 };
    ASSERT_LR11XX_RC( lr11xx_gnss_get_timings( context, &gnss_timing ) );
    HAL_DBG_TRACE_PRINTF( "Timings:\n" );
    HAL_DBG_TRACE_PRINTF( "  - radio: %u ms\n", gnss_timing.radio_ms );
    HAL_DBG_TRACE_PRINTF( "  - computation: %u ms\n", gnss_timing.computation_ms );

    uint8_t                          n_sv_detected            = 0;
    lr11xx_gnss_detected_satellite_t sv_detected[GNSS_MAX_SV] = { 0 };
    ASSERT_LR11XX_RC( lr11xx_gnss_get_nb_detected_satellites( context, &n_sv_detected ) );
    if( n_sv_detected > GNSS_MAX_SV )
    {
        HAL_DBG_TRACE_ERROR( "More SV detected than configured (detected %u, max is %u)\n", n_sv_detected,
                             GNSS_MAX_SV );
        return;
    }

    ASSERT_LR11XX_RC( lr11xx_gnss_get_detected_satellites( context, n_sv_detected, sv_detected ) );

    uint16_t result_size         = 0;
    uint8_t  nav[NAV_MAX_LENGTH] = { 0 };
    ASSERT_LR11XX_RC( lr11xx_gnss_get_result_size( context, &result_size ) );
    if( result_size > NAV_MAX_LENGTH )
    {
        HAL_DBG_TRACE_ERROR( "Result size too long (size %u, max is %u)\n", result_size, NAV_MAX_LENGTH );
        return;
    }

    ASSERT_LR11XX_RC( lr11xx_gnss_read_results( context, nav, result_size ) );

    HAL_DBG_TRACE_INFO( "Detected %u SV(s):\n", n_sv_detected );
    for( uint8_t index_sv = 0; index_sv < n_sv_detected; index_sv++ )
    {
        const lr11xx_gnss_detected_satellite_t* local_sv = &sv_detected[index_sv];
        HAL_DBG_TRACE_INFO( "  - SV %u: CNR: %i, doppler: %i\n", local_sv->satellite_id, local_sv->cnr,
                            local_sv->doppler );
    }

    HAL_DBG_TRACE_ARRAY( "NAV message", nav, result_size );
}

bool can_execute_next_scan( void )
{
    const uint32_t max_n_scans = GNSS_MAX_SCANS;
    if( max_n_scans == 0 )
    {
        // If the maximum number of scan is set to 0, the example run indefinitely
        return true;
    }
    else
    {
        return ( number_of_scan < max_n_scans ) ? true : false;
    }
}

void on_gnss_scan_done( void )
{
    apps_common_lr11xx_handle_post_gnss_scan( );
    number_of_scan++;

    fetch_and_print_results( );

    start_scan( );
}
