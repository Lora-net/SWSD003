/*!
 * @file      main_ranging.c
 *
 * @brief     Ranging example for LR11xx chip
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
#include "apps_utilities.h"
#include "lr11xx_radio.h"
#include "lr11xx_ranging.h"
#include "lr11xx_system.h"
#include "main_ranging.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Ranging related IRQs enabled on the ranging manager device
 */
#define RANGING_MANAGER_IRQ_MASK ( LR11XX_SYSTEM_IRQ_RANGING_EXCH_VALID | LR11XX_SYSTEM_IRQ_RANGING_TIMEOUT )

/**
 * @brief Ranging IRQs enabled on the ranging subordinate device
 */
#define RANGING_SUBORDINATE_IRQ_MASK ( LR11XX_SYSTEM_IRQ_RANGING_REQ_DISCARDED | LR11XX_SYSTEM_IRQ_RANGING_RESP_DONE )

/**
 * @brief Number of ranging address bytes the subordinate has to check upon reception of a ranging request
 */
#define RANGING_SUBORDINATE_CHECK_LENGTH_BYTES ( 4 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Ranging result comprising distance and RSSI.
 */
typedef struct ranging_result_s
{
    int32_t distance_m;  ///< Distance obtained from ranging [m]
    int32_t rssi;        ///< RSSI corresponding to manager-side response reception [dBm]
} ranging_result_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Context
 */

static lr11xx_hal_context_t* context;

#if defined( RANGING_DEVICE_MODE ) && ( RANGING_DEVICE_MODE == RANGING_DEVICE_MODE_MANAGER )
static const bool is_manager = true;
#elif defined( RANGING_DEVICE_MODE ) && ( RANGING_DEVICE_MODE == RANGING_DEVICE_MODE_SUBORDINATE )
static const bool is_manager = false;
#else
#error Application must define RANGING_DEVICE_MODE
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Read out and process a single ranging result from the ranging manager.
 *
 * @param [in]  ranging_bw   Bandwidth used during ranging
 * @param [out] result       Ranging result
 *
 * @return lr11xx_status_t Operation result
 */
lr11xx_status_t get_ranging_result_single( lr11xx_radio_lora_bw_t ranging_bw, ranging_result_t* result );

/**
 * @brief Obtain the most accurate ranging result for the given ranging configuration.
 *
 * This function reads out the ranging result.
 *
 * @param [in]  ranging_bw   Bandwidth used during ranging
 * @param [out] result       Ranging result
 *
 * @return lr11xx_status_t Operation result
 *
 * @sa get_ranging_result_single
 */
lr11xx_status_t get_ranging_result( lr11xx_radio_lora_bw_t ranging_bw, ranging_result_t* result );

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

    HAL_DBG_TRACE_INFO( "===== LR11xx Ranging example =====\n\n" );
    apps_common_print_sdk_driver_version( );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );
    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );
    apps_common_lr11xx_radio_ranging_init( ( void* ) context );

    /* Configure common LoRa ranging parameters */
    ASSERT_LR11XX_RC( lr11xx_radio_set_lora_sync_timeout( ( void* ) context, 0u ) );
    uint32_t   ranging_rx_tx_delay = 0u;
    const bool get_delay_result    = lr11xx_ranging_get_recommended_rx_tx_delay_indicator(
        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, &ranging_rx_tx_delay );
    if( get_delay_result == false )
    {
        HAL_DBG_TRACE_ERROR( "Failed to get ranging delay indicator\n" );
    }
    ASSERT_LR11XX_RC( lr11xx_ranging_set_parameters( ( void* ) context, RESPONSE_SYMBOLS_COUNT ) );
    ASSERT_LR11XX_RC( lr11xx_ranging_set_rx_tx_delay_indicator( ( void* ) context, ranging_rx_tx_delay ) );

    if( is_manager == true )
    {
        HAL_DBG_TRACE_INFO( "===== Running in ranging manager mode =====\n\n" );

        /* Manager specific LoRa ranging parameter configuration */
        ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( ( void* ) context, RANGING_MANAGER_IRQ_MASK, 0 ) );
        ASSERT_LR11XX_RC( lr11xx_ranging_set_request_address( ( void* ) context, RANGING_ADDRESS ) );
        ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( ( void* ) context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_tx( ( void* ) context, MANAGER_TX_RX_TIMEOUT_MS ) );
    }
    else
    {
        HAL_DBG_TRACE_INFO( "===== Running in ranging subordinate mode =====\n\n" );

        /* Subordinate specific LoRa ranging parameter configuration */
        ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( ( void* ) context, RANGING_SUBORDINATE_IRQ_MASK, 0 ) );
        ASSERT_LR11XX_RC(
            lr11xx_ranging_set_address( ( void* ) context, RANGING_ADDRESS, RANGING_SUBORDINATE_CHECK_LENGTH_BYTES ) );
        ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( ( void* ) context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_rx( ( void* ) context, 0u ) );
    }

    while( 1 )
    {
        if( is_manager == true )
        {
            apps_common_lr11xx_irq_process( ( void* ) context, RANGING_MANAGER_IRQ_MASK );
        }
        else
        {
            apps_common_lr11xx_irq_process( ( void* ) context, RANGING_SUBORDINATE_IRQ_MASK );
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

lr11xx_status_t get_ranging_result_single( lr11xx_radio_lora_bw_t ranging_bw, ranging_result_t* result )
{
    lr11xx_status_t rc;
    uint8_t         buf[LR11XX_RANGING_RESULT_LENGTH];

    /* get distance */
    rc = lr11xx_ranging_get_raw_result( context, LR11XX_RANGING_RESULT_TYPE_RAW, buf );
    if( rc != LR11XX_STATUS_OK )
    {
        return rc;
    }
    result->distance_m = lr11xx_ranging_distance_raw_to_meter( ranging_bw, buf );

    /* get RSSI */
    rc = lr11xx_ranging_get_raw_result( context, LR11XX_RANGING_RESULT_TYPE_RSSI, buf );
    if( rc != LR11XX_STATUS_OK )
    {
        return rc;
    }
    result->rssi = lr11xx_ranging_rssi_raw_to_value( buf );

    return rc;
}

lr11xx_status_t get_ranging_result( lr11xx_radio_lora_bw_t ranging_bw, ranging_result_t* result )
{
    return get_ranging_result_single( ranging_bw, result );
}

void on_ranging_request_discarded( void )
{
    /* start new ranging reception */
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( ( void* ) context, 0u ) );
}

void on_ranging_response_done( void )
{
    /* start new ranging reception */
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( ( void* ) context, 0u ) );
}

void on_ranging_exchange_valid( void )
{
    ranging_result_t result = { 0 };
    ASSERT_LR11XX_RC( get_ranging_result( LORA_BANDWIDTH, &result ) );
    HAL_DBG_TRACE_INFO( "Ranging result: Distance: %dm, RSSI: %d \n", result.distance_m, result.rssi );

    /* start new ranging transmission */
    LL_mDelay( MANAGER_RANGING_SLEEP_PERIOD );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( ( void* ) context, MANAGER_TX_RX_TIMEOUT_MS ) );
}

void on_ranging_timeout( void )
{
    /* start new ranging transmission */
    LL_mDelay( MANAGER_RANGING_SLEEP_PERIOD );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( ( void* ) context, MANAGER_TX_RX_TIMEOUT_MS ) );
}

/* --- EOF ------------------------------------------------------------------ */
