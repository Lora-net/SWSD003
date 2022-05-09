/*!
 * @file      main_ping_pong.c
 *
 * @brief     Ping-pong example for LR11xx chip
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
#include "apps_utilities.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"
#include "lr11xx_system_types.h"
#include "lr11xx_regmem.h"
#include "smtc_board.h"
#include "smtc_hal.h"
#include "smtc_hal_tmr_list.h"
#include "main_ping_pong.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Size of ping-pong message prefix
 *
 * Expressed in bytes
 */
#define PING_PONG_PREFIX_SIZE 6

/**
 * @brief Duration of the wait before packet transmission to assure reception status is ready on the other side
 *
 * Expressed in milliseconds
 */
#define DELAY_BEFORE_TX_MS 20

/**
 * @brief Duration of the wait between each ping-pong activity, can be used to adjust ping-pong speed
 *
 * Expressed in milliseconds
 */
#define DELAY_PING_PONG_PACE_MS 200

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK                                                                          \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT | \
      LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define APP_PARTIAL_SLEEP true

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_hal_context_t* context;

static uint8_t buffer_tx[PAYLOAD_LENGTH];
static bool    is_master = true;

static const uint8_t ping_msg[PING_PONG_PREFIX_SIZE] = "M-PING";
static const uint8_t pong_msg[PING_PONG_PREFIX_SIZE] = "S-PONG";

static const uint8_t prefix_size = ( PING_PONG_PREFIX_SIZE <= PAYLOAD_LENGTH ? PING_PONG_PREFIX_SIZE : PAYLOAD_LENGTH );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Handle reception failure for ping-pong example
 */
static void ping_pong_reception_failure_handling( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    hal_mcu_disable_irq( );

    /* Init board */
    hal_mcu_init( );
    smtc_board_init_periph( );

    hal_mcu_enable_irq( );

    HAL_DBG_TRACE_INFO( "===== LR11xx Ping-Pong example =====\n\n" );

    /* Configure the partial low power mode */
    hal_mcu_partial_sleep_enable( APP_PARTIAL_SLEEP );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );

    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );

    apps_common_lr11xx_radio_init( ( void* ) context );

    ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );

    apps_common_lr11xx_enable_irq( );

    /* Intializes random number generator */
    srand( hal_mcu_get_tick( ) );

    memcpy( buffer_tx, ping_msg, prefix_size );
    for( int i = prefix_size; i < PAYLOAD_LENGTH; i++ )
    {
        buffer_tx[i] = i;
    }

    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer_tx, PAYLOAD_LENGTH ) );

    smtc_shield_handle_pre_tx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( context, 0 ) );

    while( 1 )
    {
#if( HAL_USE_WATCHDOG == HAL_FEATURE_ON )
        hal_watchdog_reload( );
#endif  // HAL_USE_WATCHDOG == HAL_FEATURE_ON

        apps_common_lr11xx_irq_process( context, IRQ_MASK );
    }
}

void on_tx_done( void )
{
    smtc_shield_handle_post_tx( );

    hal_mcu_set_sleep_for_ms( DELAY_PING_PONG_PACE_MS );

    smtc_shield_handle_pre_rx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( context,
                                           RX_TIMEOUT_VALUE + rand( ) % 500 ) );  // Random delay to avoid
                                                                                  // unwanted synchronization
}

void on_rx_done( void )
{
    uint8_t buffer_rx[PAYLOAD_LENGTH];
    uint8_t size;

    smtc_shield_handle_post_rx( );

    apps_common_lr11xx_receive( context, buffer_rx, &size );

    if( is_master == true )
    {
        if( memcmp( buffer_rx, ping_msg, prefix_size ) == 0 )
        {
            is_master = false;
            memcpy( buffer_tx, pong_msg, prefix_size );
        }
        else if( memcmp( buffer_rx, pong_msg, prefix_size ) != 0 )
        {
            HAL_DBG_TRACE_ERROR( "Unexpected message\n" );
        }
    }
    else
    {
        if( memcmp( buffer_rx, ping_msg, prefix_size ) != 0 )
        {
            HAL_DBG_TRACE_ERROR( "Unexpected message\n" );

            is_master = true;
            memcpy( buffer_tx, ping_msg, prefix_size );
        }
    }

    hal_mcu_set_sleep_for_ms( DELAY_PING_PONG_PACE_MS );

    hal_mcu_set_sleep_for_ms( DELAY_BEFORE_TX_MS );

    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer_tx, PAYLOAD_LENGTH ) );

    smtc_shield_handle_pre_tx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( context, 0 ) );
}

void on_rx_timeout( void )
{
    ping_pong_reception_failure_handling( );
}

void on_rx_crc_error( void )
{
    ping_pong_reception_failure_handling( );
}

void on_fsk_len_error( void )
{
    ping_pong_reception_failure_handling( );
}

static void ping_pong_reception_failure_handling( void )
{
    smtc_shield_handle_post_rx( );

    is_master = true;
    memcpy( buffer_tx, ping_msg, prefix_size );

    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer_tx, PAYLOAD_LENGTH ) );

    smtc_shield_handle_pre_tx( );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( context, 0 ) );
}