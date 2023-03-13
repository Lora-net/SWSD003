/*!
 * @file      main_ping_pong.c
 *
 * @brief     Ping-pong example for Sx126x chip
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
#include "sx126x.h"
#include "sx126x_hal.h"
#include "main_ping_pong.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Size of ping-pong message prefix
 *
 * Expressed in bytes
 */
#define PING_PONG_PREFIX_SIZE 4

/**
 * @brief Threshold:number of exchanges before informing user on UART that the board pair is still not synchronized
 *
 * Expressed in number of packet exchanged
 */
#define SYNC_PACKET_THRESHOLD 64

/**
 * @brief Number of exchanges are stored in the payload of the packet exchanged during this PING-PONG communication
 *        this constant indicates where in the packet the two bytes used to count are located
 *
 * Expressed in bytes
 */
#define ITERATION_INDEX ( PING_PONG_PREFIX_SIZE + 1 )

/**
 * @brief Duration of the wait between each ping-pong activity, can be used to adjust ping-pong speed
 *
 * Expressed in milliseconds
 */
#define DELAY_PING_PONG_PACE_MS 200

/**
 * @brief Duration of the wait before packet transmission to assure reception status is ready on the other side
 *
 * Expressed in milliseconds
 */
#define DELAY_BEFORE_TX_MS 20
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

static uint8_t buffer_tx[PAYLOAD_LENGTH];
static bool    is_master = true;

const uint8_t ping_msg[PING_PONG_PREFIX_SIZE] = "PING";
const uint8_t pong_msg[PING_PONG_PREFIX_SIZE] = "PONG";

static uint16_t iteration       = 0;
static uint16_t packets_to_sync = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

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

    HAL_DBG_TRACE_INFO( "===== SX126x Ping Pong example =====\n\n" );
    apps_common_sx126x_print_version_info( );

    apps_common_shield_init( );
    context = apps_common_sx126x_get_context( );

    apps_common_sx126x_init( ( void* ) context );
    apps_common_sx126x_radio_init( ( void* ) context );

    sx126x_set_dio_irq_params(
        context, SX126X_IRQ_ALL,
        SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CRC_ERROR,
        SX126X_IRQ_NONE, SX126X_IRQ_NONE );

    sx126x_clear_irq_status( context, SX126X_IRQ_ALL );

    /* Intializes random number generator */
    srand( 10 );

    memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
    buffer_tx[PING_PONG_PREFIX_SIZE] = 0;
    buffer_tx[ITERATION_INDEX]       = ( uint8_t ) ( iteration );
    for( int i = PING_PONG_PREFIX_SIZE + 1 + 1; i < PAYLOAD_LENGTH; i++ )
    {
        buffer_tx[i] = i;
    }

    sx126x_write_buffer( context, 0, buffer_tx, PAYLOAD_LENGTH );

    apps_common_sx126x_handle_pre_tx( );
    sx126x_set_tx( context, 0 );

    while( 1 )
    {
        apps_common_sx126x_irq_process( context );
    }
}

void on_tx_done( void )
{
    apps_common_sx126x_handle_post_tx( );
    HAL_DBG_TRACE_INFO( "Sent message %s, iteration %d\n", buffer_tx, iteration );

    LL_mDelay( DELAY_PING_PONG_PACE_MS );

    apps_common_sx126x_handle_pre_rx( );
    sx126x_set_rx( context, get_time_on_air_in_ms( ) + RX_TIMEOUT_VALUE +
                                rand( ) % 500 );  // Random delay to avoid unexpected sync
}

void on_rx_done( void )
{
    uint8_t buffer_rx[PAYLOAD_LENGTH];
    uint8_t size;

    packets_to_sync = 0;

    apps_common_sx126x_handle_post_rx( );

    apps_common_sx126x_receive( context, buffer_rx, &size, PAYLOAD_LENGTH );
    iteration = buffer_rx[ITERATION_INDEX];

    iteration++;
    HAL_DBG_TRACE_INFO( "Received message %s, iteration %d\n", buffer_rx, iteration );

    if( is_master == true )
    {
        if( memcmp( buffer_rx, ping_msg, PING_PONG_PREFIX_SIZE ) == 0 )
        {
            is_master = false;
            memcpy( buffer_tx, pong_msg, PING_PONG_PREFIX_SIZE );
        }
        else if( memcmp( buffer_rx, pong_msg, PING_PONG_PREFIX_SIZE ) != 0 )
        {
            HAL_DBG_TRACE_ERROR( "Unexpected message - PONG expected\n" );
        }
    }
    else
    {
        if( memcmp( buffer_rx, ping_msg, PING_PONG_PREFIX_SIZE ) != 0 )
        {
            HAL_DBG_TRACE_ERROR( "Unexpected message\n" );

            is_master = true;
            memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
        }
    }

    LL_mDelay( DELAY_PING_PONG_PACE_MS + DELAY_BEFORE_TX_MS );
    buffer_tx[ITERATION_INDEX] = ( uint8_t ) ( iteration );

    sx126x_write_buffer( context, 0, buffer_tx, PAYLOAD_LENGTH );

    apps_common_sx126x_handle_pre_tx( );
    sx126x_set_tx( context, 0 );
}

void on_rx_timeout( void )
{
    packets_to_sync++;
    if( packets_to_sync > SYNC_PACKET_THRESHOLD )
    {
        HAL_DBG_TRACE_WARNING(
            "It looks like synchronisation is still not done, consider resetting one of the board\n" );
    }
    apps_common_sx126x_handle_post_rx( );

    is_master = true;
    iteration = 0;
    memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
    buffer_tx[ITERATION_INDEX] = ( uint8_t ) ( iteration );

    sx126x_write_buffer( context, 0, buffer_tx, PAYLOAD_LENGTH );

    apps_common_sx126x_handle_pre_tx( );
    sx126x_set_tx( context, 0 );
}

void on_rx_error( void )
{
    apps_common_sx126x_handle_post_rx( );

    is_master = true;
    iteration = 0;
    memcpy( buffer_tx, ping_msg, PING_PONG_PREFIX_SIZE );
    buffer_tx[ITERATION_INDEX] = ( uint8_t ) ( iteration );

    sx126x_write_buffer( context, 0, buffer_tx, PAYLOAD_LENGTH );

    apps_common_sx126x_handle_pre_tx( );
    sx126x_set_tx( context, 0 );
}
