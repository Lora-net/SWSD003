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
#include "main_cad.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "smtc_hal_mcu.h"

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

/**
 * @brief Look up table to get the best measured CAD detection rate
 *        regarding Spreading Factor and Bandwidth
 *
 * It takes advantage of the existing definitions of spreading factors and bandwidth as enum types:
 * BW 62,5KHz to BW 500Khz are defined 3 to 6, which means that substracting 3 to this value gives the index of
 * the line for this bandwidth in the following table. Likewise, spreading factor enum definitions are defined 5 to 12,
 * meaning substracting 5 to this value gives the index for this SF in this table.
 *
 */
static const uint8_t optimized_parameters[4][4][8] = {
    // SymbolNum = 2
    {
        // SF5, SF6, SF7, SF8, SF9, SF10, SF11, SF12,
        { 39, 45, 47, 53, 59, 61, 64, 63 },                                         //  BW_62500
        { 44, 51, 49, 55, 56, 60, 62, 68 },                                         // BW_125000
        { 48, 48, 50, 55, 55, 59, 61, 65 },                                         // BW_250000
        { 76, 80, 71, 77, 69, CAD_DETECT_PEAK, CAD_DETECT_PEAK, CAD_DETECT_PEAK },  // BW_500000
    },
    // SymbolNum = 4
    {
        // SF5, SF6, SF7, SF8, SF9, SF10, SF11, SF12,
        { 43, 45, 45, 50, 53, 57, 59, 63 },  //  BW_62500
        { 44, 46, 49, 53, 53, 55, 57, 62 },  // BW_125000
        { 45, 47, 47, 51, 51, 56, 59, 62 },  // BW_250000
        { 58, 66, 58, 65, 62, 55, 60, 57 },  // BW_500000
    },
    // SymbolNum = 8
    {
        // SF5, SF6, SF7, SF8, SF9, SF10, SF11, SF12,
        { 43, 44, 46, 48, 51, 53, 56, 59 },  //  BW_62500
        { 45, 43, 44, 50, 52, 55, 56, 61 },  // BW_125000
        { 42, 44, 45, 48, 50, 53, 55, 60 },  // BW_250000
        { 49, 52, 50, 59, 56, 57, 57, 60 },  // BW_500000
    },
    // SymbolNum = 16
    {
        // SF5, SF6, SF7, SF8, SF9, SF10, SF11, SF12,
        { 41, 44, 43, 46, 49, 52, 54, 60 },  //  BW_62500
        { 42, 42, 43, 48, 49, 53, 55, 59 },  // BW_125000
        { 41, 42, 43, 48, 48, 53, 54, 58 },  // BW_250000
        { 44, 47, 45, 53, 52, 53, 57, 62 },  // BW_500000
    },

};

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

static uint8_t  buffer[PAYLOAD_LENGTH];
static uint32_t iteration_number        = 0;
static uint32_t received_packet_counter = 0;
static uint32_t detection_counter       = 0;
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

/**
 * @brief Look up in table to get best known detPeak parameter for CAD according to SF, BW and number of symbols used
 * for detection
 *
 * @param [in] sf  spreading factor
 * @param [in] bw bandwidth
 * @param [in,out] cad_params structure containing CAD parameters to check symbolNum and update detPeak
 */
static void optimize_cad_detection_peak_parameter( lr11xx_radio_lora_sf_t sf, lr11xx_radio_lora_bw_t bw,
                                                   lr11xx_radio_cad_params_t* cad_params );

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
    uart_init( );

    HAL_DBG_TRACE_INFO( "===== LR11xx CAD example =====\n\n" );
    apps_common_print_sdk_driver_version( );
    HAL_DBG_TRACE_INFO( "CAD Exit Mode : %s\n", lr11xx_radio_cad_exit_mode_to_str( CAD_EXIT_MODE ) );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );
    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );
    apps_common_lr11xx_radio_init( ( void* ) context );

    ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );

    if( cad_params.cad_exit_mode == LR11XX_RADIO_CAD_EXIT_MODE_RX )
    {
        cad_params.cad_timeout = lr11xx_radio_convert_time_in_ms_to_rtc_step( CAD_TIMEOUT_MS );
    }
    else if( cad_params.cad_exit_mode == LR11XX_RADIO_CAD_EXIT_MODE_TX )
    {
        for( int i = 0; i < PAYLOAD_LENGTH; i++ )
        {
            buffer[i] = i;
        }
        ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer, PAYLOAD_LENGTH ) );
    }

    optimize_cad_detection_peak_parameter( LORA_SPREADING_FACTOR, LORA_BANDWIDTH, &cad_params );
    ASSERT_LR11XX_RC( lr11xx_radio_set_cad_params( context, &cad_params ) );

    start_cad_after_delay( DELAY_MS_BEFORE_CAD );

    while( 1 )
    {
        apps_common_lr11xx_irq_process( ( void* ) context, IRQ_MASK );
    }
}

void on_cad_done_detected( void )
{
    detection_counter++;
    HAL_DBG_TRACE_INFO( "Consecutive detection(s): %d\n", detection_counter );
    switch( cad_params.cad_exit_mode )
    {
    case LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC:
        HAL_DBG_TRACE_INFO( "Switch to StandBy mode\n" );
        start_cad_after_delay( DELAY_MS_BEFORE_CAD );
        break;
    case LR11XX_RADIO_CAD_EXIT_MODE_RX:
        HAL_DBG_TRACE_INFO( "Switch to RX mode\n" );
        apps_common_lr11xx_handle_pre_rx( );
        break;
    case LR11XX_RADIO_CAD_EXIT_MODE_TX:
        start_cad_after_delay( DELAY_MS_BEFORE_CAD );
        break;
    default:
        HAL_DBG_TRACE_ERROR( "Unknown CAD exit mode: 0x%02x\n", cad_params.cad_exit_mode );
        break;
    }
}

void on_cad_done_undetected( void )
{
    detection_counter = 0;
    switch( cad_params.cad_exit_mode )
    {
    case LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC:
        HAL_DBG_TRACE_INFO( "Switch to StandBy mode\n" );
        start_cad_after_delay( DELAY_MS_BEFORE_CAD );
        break;
    case LR11XX_RADIO_CAD_EXIT_MODE_RX:
        start_cad_after_delay( DELAY_MS_BEFORE_CAD );
        break;
    case LR11XX_RADIO_CAD_EXIT_MODE_TX:
        HAL_DBG_TRACE_INFO( "Switch to TX mode\n" );
        apps_common_lr11xx_handle_pre_tx( );
        break;
    default:
        HAL_DBG_TRACE_ERROR( "Unknown CAD exit mode: 0x%02x\n", cad_params.cad_exit_mode );
        break;
    }
}

void on_tx_done( void )
{
    apps_common_lr11xx_handle_post_tx( );
    buffer[0] = buffer[0] + 1;
    ASSERT_LR11XX_RC( lr11xx_regmem_write_buffer8( context, buffer, PAYLOAD_LENGTH ) );
    start_cad_after_delay( DELAY_MS_BEFORE_CAD );
}

void on_rx_done( void )
{
    uint8_t size;
    memset( buffer, 0, PAYLOAD_LENGTH );
    apps_common_lr11xx_handle_post_rx( );
    apps_common_lr11xx_receive( ( void* ) context, buffer, PAYLOAD_LENGTH, &size );
    HAL_DBG_TRACE_INFO( "Consecutive reception(s): %d\n", received_packet_counter );
    received_packet_counter++;
    HAL_DBG_TRACE_ARRAY( "Received packet: ", buffer, PAYLOAD_LENGTH );
    start_cad_after_delay( DELAY_MS_BEFORE_CAD );
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
    HAL_DBG_TRACE_PRINTF( "\nStart CAD, iteration %d\n", iteration_number++ );
    smtc_hal_mcu_wait_ms( ( const uint32_t ) delay_ms );
    ASSERT_LR11XX_RC( lr11xx_radio_set_cad( context ) );
}

static void cad_reception_failure_handling( void )
{
    apps_common_lr11xx_handle_post_rx( );
    start_cad_after_delay( DELAY_MS_BEFORE_CAD );
    received_packet_counter = 0;
}

#if USER_PROVIDED_CAD_PARAMETERS == False
static void optimize_cad_detection_peak_parameter( lr11xx_radio_lora_sf_t sf, lr11xx_radio_lora_bw_t bw,
                                                   lr11xx_radio_cad_params_t* cad_params )
{
    if( ( bw >= LR11XX_RADIO_LORA_BW_62 ) && ( bw <= LR11XX_RADIO_LORA_BW_500 ) )
    {
        switch( cad_params->cad_symb_nb )
        {
        case 2:
        {
            cad_params->cad_detect_peak = optimized_parameters[0][bw - 3][sf - 5];
            break;
        }
        case 4:
        {
            cad_params->cad_detect_peak = optimized_parameters[1][bw - 3][sf - 5];
            break;
        }
        case 8:
        {
            cad_params->cad_detect_peak = optimized_parameters[2][bw - 3][sf - 5];
            break;
        }
        case 16:
        {
            cad_params->cad_detect_peak = optimized_parameters[3][bw - 3][sf - 5];
            break;
        }
        default:
        {
            HAL_DBG_TRACE_WARNING(
                "CAD may not function properly, detection peak optimization not supported for SymbolNum = %d\n",
                cad_params->cad_symb_nb );
            break;
        }
        }
    }
    else
    {
        HAL_DBG_TRACE_WARNING(
            "CAD may not function properly, unsupported bandwidth for CAD parameter optimization: %s\n",
            lr11xx_radio_lora_bw_to_str( bw ) );
    }
}
#else
static void optimize_cad_detection_peak_parameter( lr11xx_radio_lora_sf_t sf, lr11xx_radio_lora_bw_t bw,
                                                   lr11xx_radio_cad_params_t* cad_params )
{
}
#endif
