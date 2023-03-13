/*!
 * @file      main_spectrum_display.c
 *
 * @brief     Spectrum-display example for SX126x chip
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

#include <stdlib.h>

#include "apps_common.h"
#include "apps_utilities.h"
#include "sx126x.h"
#include "main_spectrum_display.h"
#include "curve_plot.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Duration of the wait between setting to RX mode and valid instant RSSI value available
 *
 * Expressed in milliseconds
 *
 * @warning If switching from StandbyRC mode this delay is recommended to set to 30ms; if switching from StandbyXOSC,
 * 1ms.
 */
#define DELAY_BETWEEN_SET_RX_AND_VALID_RSSI_MS ( 1 )

/**
 * @brief Duration of the wait between each instant RSSI fetch. This is to make sure that the RSSI value is stable
 * before fetching
 */
#define DELAY_BETWEEN_EACH_INST_RSSI_FETCH_US ( 800 )

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

static sx126x_hal_context_t* context;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void spectrum_display_start( uint32_t freq_hz );
static void print_configuration( void );

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
    uart_init();

    HAL_DBG_TRACE_INFO( "===== SX126x Spectrum Display example =====\n\n" );
    apps_common_sx126x_print_version_info( );

    apps_common_shield_init( );
    context = apps_common_sx126x_get_context( );

    apps_common_sx126x_init( ( void* ) context );
    apps_common_sx126x_radio_init( ( void* ) context );

    print_configuration( );

    create_canvas( );

    while( 1 )
    {
        int16_t        result;
        static uint8_t freq_chan_index = 0;
        const uint32_t freq_hz         = FREQ_START_HZ + ( freq_chan_index * WIDTH_CHAN_HZ );

        /* Start Spectral scan */
        spectrum_display_start( freq_hz );

        LL_mDelay( DELAY_BETWEEN_EACH_INST_RSSI_FETCH_US );
        ASSERT_SX126X_RC( sx126x_get_rssi_inst( context, &result ) );
        plot_curve( ( freq_chan_index + 1 ), ( abs( result ) / RSSI_SCALE ) );

        /* Switch to next channel */
        ASSERT_SX126X_RC( sx126x_set_standby( context, SX126X_STANDBY_CFG_XOSC ) );
        apps_common_sx126x_handle_post_rx( );

        freq_chan_index++;
        if( freq_chan_index >= NB_CHAN )
        {
            freq_chan_index = 0;

            /* Pace the scan speed (1 sec min) */
            for( uint16_t i = 0; i < ( int ) ( PACE_S ? PACE_S : 1 ); i++ )
            {
                LL_mDelay( 1000 );
            }
        }
    }
}

void spectrum_display_start( uint32_t freq_hz )
{
    /* Set frequency */
    ASSERT_SX126X_RC( sx126x_set_rf_freq( context, freq_hz ) );

    /* Set Radio in Rx continuous mode */
    apps_common_sx126x_handle_pre_rx( );
    ASSERT_SX126X_RC( sx126x_set_rx_with_timeout_in_rtc_step( context, RX_CONTINUOUS ) );

    LL_mDelay( DELAY_BETWEEN_SET_RX_AND_VALID_RSSI_MS );
}

void print_configuration( void )
{
    HAL_DBG_TRACE_PRINTF( "\n" );
    HAL_DBG_TRACE_INFO( "Spectral Scan configuration:\n" );
    HAL_DBG_TRACE_INFO( "  - Number of channels need to scan: %d\n", NB_CHAN );
    HAL_DBG_TRACE_INFO( "  - Time delay between 2 scans: %d S\n", PACE_S );
    HAL_DBG_TRACE_INFO( "  - Start frequency: %.3f MHz\n", ( FREQ_START_HZ / 10E5 ) );
    HAL_DBG_TRACE_INFO( "  - Frequency step of scan channels: %.3f kHz\n", ( WIDTH_CHAN_HZ / 10E2 ) );
    HAL_DBG_TRACE_INFO( "Start Spectrum Display:\n" );
}
