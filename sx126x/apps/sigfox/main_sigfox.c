/*!
 * @file      main_sigfox.c
 *
 * @brief     Sigfox PHY example for SX126X chip
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

#include <stdio.h>
#include <string.h>

#include "apps_common.h"
#include "sx126x.h"
#include "sx126x_hal.h"
#include "main_sigfox.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "smtc_dbpsk.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// Application payload = 0x01
const uint8_t sample0[] = { 0xaa, 0xaa, 0xa0, 0x8d, 0x01, 0x05, 0x98, 0xba, 0xdc, 0xfe, 0x01, 0x9a, 0x09, 0xfe, 0x04 };

#define SIGFOX_PAYLOAD_LENGTH 15

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

static void send_frame( void );

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
    uart_init( );

    HAL_DBG_TRACE_INFO( "===== SX126X Sigfox PHY example =====\n\n" );
    apps_common_sx126x_print_version_info( );

    apps_common_shield_init( );
    context = apps_common_sx126x_get_context( );

    apps_common_sx126x_init( ( void* ) context );
    apps_common_sx126x_radio_dbpsk_init( ( void* ) context, SIGFOX_PAYLOAD_LENGTH );

    sx126x_set_dio_irq_params( context, SX126X_IRQ_ALL, SX126X_IRQ_TX_DONE, SX126X_IRQ_NONE, SX126X_IRQ_NONE );
    sx126x_clear_irq_status( context, SX126X_IRQ_ALL );

    send_frame( );

    while( 1 )
    {
        apps_common_sx126x_irq_process( context );
    }
}

void on_tx_done( void )
{
    apps_common_sx126x_handle_post_tx( );

    smtc_hal_mcu_wait_ms( ( const uint32_t ) TX_TO_TX_DELAY_IN_MS );

    send_frame( );
}

void send_frame( void )
{
    uint8_t frame_buffer[smtc_dbpsk_get_pld_len_in_bytes( SIGFOX_PAYLOAD_LENGTH << 3 )];

    smtc_dbpsk_encode_buffer( sample0, SIGFOX_PAYLOAD_LENGTH << 3, frame_buffer );

    sx126x_write_buffer( context, 0, frame_buffer, smtc_dbpsk_get_pld_len_in_bytes( SIGFOX_PAYLOAD_LENGTH << 3 ) );

    apps_common_sx126x_handle_pre_tx( );

    sx126x_set_tx( context, 0 );
}
