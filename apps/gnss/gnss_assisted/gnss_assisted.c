/*!
 * @file      gnss_assisted.c
 *
 * @brief     GNSS assisted scan example for LR11xx chip
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

#include "smtc_hal_dbg_trace.h"
#include "smtc_board.h"
#include "board_options.h"
#include "apps_utilities.h"
#include "gnss_example_api.h"
#include "ring_buffer.h"
#include "lr11xx_gnss.h"
#include "main_gnss.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct
{
    lr11xx_gnss_solver_assistance_position_t assistance_position;
    lr11xx_gnss_scan_mode_t                  scan_mode;
    lr11xx_gnss_search_mode_t                effort_mode;
    uint8_t                                  input_parameters;
    uint8_t                                  max_sv;
} gnss_configuration_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

const gnss_configuration_t gnss_configuration = {
    .assistance_position = { .latitude  = GNSS_ASSISTANCE_POSITION_LATITUDE,
                             .longitude = GNSS_ASSISTANCE_POSITION_LONGITUDE },
    .scan_mode           = GNSS_SCAN_MODE,
    .effort_mode         = GNSS_EFFORT_MODE,
    .input_parameters    = GNSS_INPUT_PARAMETERS,
    .max_sv              = GNSS_MAX_SV,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint32_t gps_local_delay_s = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static uint32_t get_gps_delay_from_serial( void );
static uint32_t get_gps_time_now( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void gnss_init( const void* context )
{
    gps_local_delay_s = get_gps_delay_from_serial( );
    ASSERT_LR11XX_RC( lr11xx_gnss_set_scan_mode( context, gnss_configuration.scan_mode ) );
    ASSERT_LR11XX_RC( lr11xx_gnss_set_assistance_position( context, &gnss_configuration.assistance_position ) );
}

void gnss_call_scan( const void* context )
{
    ASSERT_LR11XX_RC( lr11xx_gnss_scan_assisted( context, get_gps_time_now( ), gnss_configuration.effort_mode,
                                                 gnss_configuration.input_parameters, gnss_configuration.max_sv ) );
}

const char* gnss_get_example_name( void ) { return ( const char* ) "LR11xx GNSS Assisted scan example"; }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint32_t get_gps_delay_from_serial( )
{
    const uint16_t max_receive  = 50;
    char           received[50] = { 0 };
    uint16_t       n_received   = 0;
    hal_uart_start_rx_it( HAL_PRINTF_UART_ID );
    while( true )
    {
        int data = 0;
        hal_mcu_disable_irq( );
        const bool has_char    = pop_from_interrupt_buffer( &data );
        const bool has_overrun = has_buffer_overrun( );
        hal_mcu_enable_irq( );

        if( has_overrun == true )
        {
            HAL_DBG_TRACE_ERROR( "Failed to get GPS time from UART. Buffer overrun\n" );
            reset_buffer_overrun_flag( );
            return 0;
        }

        if( has_char == true )
        {
            if( data != '\n' )
            {
                received[n_received++] = data;
            }
            else
            {
                received[n_received++] = 0;
                break;
            }
            if( n_received >= max_receive - 1 )
            {
                received[n_received++] = 0;
                break;
            }
        }
    }
    const uint32_t local_rtc_s = hal_rtc_get_time_s( );
    uint32_t       gps_time    = 0;
    sscanf( received, "%lu", &gps_time );

    const uint32_t gps_local_delay_s = gps_time - local_rtc_s;
    return gps_local_delay_s;
}

uint32_t get_gps_time_now( void ) { return hal_rtc_get_time_s( ) + gps_local_delay_s; }
