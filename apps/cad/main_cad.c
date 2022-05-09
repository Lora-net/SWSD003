/*!
 * @file      main_cad.c
 *
 * @brief     Channel Activity Detection (CAD) example for LR11xx chip
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

#include "apps_common.h"
#include "apps_utilities.h"
#include "lr11xx_radio.h"
#include "lr11xx_radio_types.h"
#include "lr11xx_system.h"
#include "lr11xx_regmem.h"
#include "smtc_board.h"
#include "smtc_hal.h"
#include "main_cad.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK                                                                                  \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT |         \
      LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_CAD_DONE | \
      LR11XX_SYSTEM_IRQ_CAD_DETECTED )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define APP_PARTIAL_SLEEP true

/**
 * @brief Delay time before triggering CAD mode
 */
#define DELAY_TIME_BEFORE_SET_TO_CAD_MS 1000

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_hal_context_t* context;

static lr11xx_radio_cad_params_t cad_params = {
    .cad_symb_nb     = CAD_SYMBOL_NUM,
    .cad_detect_peak = CAD_DETECT_PEAK,
    .cad_detect_min  = CAD_DETECT_MIN,
    .cad_exit_mode   = CAD_EXIT_MODE,
    .cad_timeout     = 0,
};

static uint8_t buffer[PAYLOAD_LENGTH];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Add a delay before setting to CAD mode
 *
 * @param [in] delay_ms Delay time before setting to CAD mode, changing the value to adjust CAD pace
 *
 */
static void start_cad_after_delay( uint16_t delay_ms );

/**
 * @brief Handle reception failure for CAD example
 */
static void cad_reception_failure_handling( void );

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

    HAL_DBG_TRACE_INFO( "===== LR11xx CAD example =====\n\n" );

    /* Configure the partial low power mode */
    hal_mcu_partial_sleep_enable( APP_PARTIAL_SLEEP );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );

    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );

    apps_common_lr11xx_radio_init( ( void* ) context );

    ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );

    apps_common_lr11xx_enable_irq( );

    if( cad_params.cad_exit_mode == LR11XX_RADIO_CAD_EXIT_MODE_RX )
    {
        cad_params.cad_timeout = lr11xx_radio_convert_time_in_ms_to_rtc_step( CAD_TIMOUT_MS );
    }
    else if( cad_params.cad_exit_mode == LR11XX_RADIO_CAD_EXIT_MODE_TX )
    {
        for( int i = 0; i < PAYLOAD_LENGTH; i++ )
        {
            buffer[i] = i;
        }
        ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer, PAYLOAD_LENGTH ) );
    }
    ASSERT_LR11XX_RC( lr11xx_radio_set_cad_params( context, &cad_params ) );

    start_cad_after_delay( DELAY_TIME_BEFORE_SET_TO_CAD_MS );

    while( 1 )
    {
#if( HAL_USE_WATCHDOG == HAL_FEATURE_ON )
        hal_watchdog_reload( );
#endif

        apps_common_lr11xx_irq_process( ( void* ) context, IRQ_MASK );
    }
}

void on_cad_done_detected( void )
{
    switch( cad_params.cad_exit_mode )
    {
    case LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC:
        HAL_DBG_TRACE_INFO( "Switch to StandBy mode\n" );
        start_cad_after_delay( DELAY_TIME_BEFORE_SET_TO_CAD_MS );
        break;
    case LR11XX_RADIO_CAD_EXIT_MODE_RX:
        HAL_DBG_TRACE_INFO( "Switch to RX mode\n" );
        smtc_shield_handle_pre_rx( );
        break;
    case LR11XX_RADIO_CAD_EXIT_MODE_TX:
        start_cad_after_delay( DELAY_TIME_BEFORE_SET_TO_CAD_MS );
        break;
    default:
        HAL_DBG_TRACE_ERROR( "Unknown CAD exit mode: 0x%02x\n", cad_params.cad_exit_mode );
        break;
    }
}

void on_cad_done_undetected( void )
{
    switch( cad_params.cad_exit_mode )
    {
    case LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC:
        HAL_DBG_TRACE_INFO( "Switch to StandBy mode\n" );
        start_cad_after_delay( DELAY_TIME_BEFORE_SET_TO_CAD_MS );
        break;
    case LR11XX_RADIO_CAD_EXIT_MODE_RX:
        start_cad_after_delay( DELAY_TIME_BEFORE_SET_TO_CAD_MS );
        break;
    case LR11XX_RADIO_CAD_EXIT_MODE_TX:
        HAL_DBG_TRACE_INFO( "Switch to TX mode\n" );
        smtc_shield_handle_pre_tx( );
        break;
    default:
        HAL_DBG_TRACE_ERROR( "Unknown CAD exit mode: 0x%02x\n", cad_params.cad_exit_mode );
        break;
    }
}

void on_tx_done( void )
{
    smtc_shield_handle_post_tx( );
    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer, PAYLOAD_LENGTH ) );
    start_cad_after_delay( DELAY_TIME_BEFORE_SET_TO_CAD_MS );
}

void on_rx_done( void )
{
    uint8_t size;

    smtc_shield_handle_post_rx( );
    apps_common_lr11xx_receive( ( void* ) context, buffer, &size );
    start_cad_after_delay( DELAY_TIME_BEFORE_SET_TO_CAD_MS );
}

void on_rx_timeout( void )
{
    cad_reception_failure_handling( );
}

void on_rx_crc_error( void )
{
    cad_reception_failure_handling( );
}

static void start_cad_after_delay( uint16_t delay_ms )
{
    hal_mcu_set_sleep_for_ms( delay_ms );
    ASSERT_LR11XX_RC( lr11xx_radio_set_cad( context ) );
}

static void cad_reception_failure_handling( void )
{
    smtc_shield_handle_post_rx( );
    start_cad_after_delay( DELAY_TIME_BEFORE_SET_TO_CAD_MS );
}
