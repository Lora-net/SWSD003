/*!
 * @file      main_rttof.c
 *
 * @brief     RTToF (Ranging) example for LR11xx chip
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
#include "lr11xx_rttof.h"
#include "lr11xx_system.h"
#include "main_rttof.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief RTToF related IRQs enabled on the RTToF manager device
 */
#define RTTOF_MANAGER_IRQ_MASK ( LR11XX_SYSTEM_IRQ_RTTOF_EXCH_VALID | LR11XX_SYSTEM_IRQ_RTTOF_TIMEOUT )

/**
 * @brief RTToF IRQs enabled on the RTToF subordinate device
 */
#define RTTOF_SUBORDINATE_IRQ_MASK ( LR11XX_SYSTEM_IRQ_RTTOF_REQ_DISCARDED | LR11XX_SYSTEM_IRQ_RTTOF_RESP_DONE )

/**
 * @brief Number of RTToF address bytes the subordinate has to check upon reception of a RTToF request
 */
#define RTTOF_SUBORDINATE_CHECK_LENGTH_BYTES ( 4 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief RTToF result comprising distance and RSSI.
 */
typedef struct rttof_result_s
{
    int32_t distance_m;  ///< Distance obtained from RTToF [m]
    int32_t rssi;        ///< RSSI corresponding to manager-side response reception [dBm]
} rttof_result_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Context
 */

static lr11xx_hal_context_t* context;

#if defined( RTTOF_DEVICE_MODE ) && ( RTTOF_DEVICE_MODE == RTTOF_DEVICE_MODE_MANAGER )
static const bool is_manager = true;
#elif defined( RTTOF_DEVICE_MODE ) && ( RTTOF_DEVICE_MODE == RTTOF_DEVICE_MODE_SUBORDINATE )
static const bool is_manager = false;
#else
#error Application must define RTTOF_DEVICE_MODE
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Read out and process a single RTToF result from the RTToF manager.
 *
 * @param [in]  rttof_bw   Bandwidth used during RTToF
 * @param [out] result       RTToF result
 *
 * @return lr11xx_status_t Operation result
 */
lr11xx_status_t get_rttof_result( lr11xx_radio_lora_bw_t rttof_bw, rttof_result_t* result );

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

    HAL_DBG_TRACE_INFO( "===== LR11xx RTToF example =====\n\n" );
    apps_common_print_sdk_driver_version( );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );
    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );
    apps_common_lr11xx_radio_rttof_init( ( void* ) context );

    /* Configure common LoRa RTToF parameters */
    ASSERT_LR11XX_RC( lr11xx_radio_set_lora_sync_timeout( ( void* ) context, 0u ) );
    ASSERT_LR11XX_RC( lr11xx_rttof_set_parameters( ( void* ) context, RESPONSE_SYMBOLS_COUNT ) );

    if( is_manager == true )
    {
        HAL_DBG_TRACE_INFO( "===== Running in RTToF manager mode =====\n\n" );

        /* Manager specific LoRa RTToF parameter configuration */
        ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( ( void* ) context, RTTOF_MANAGER_IRQ_MASK, 0 ) );
        ASSERT_LR11XX_RC( lr11xx_rttof_set_request_address( ( void* ) context, RTTOF_ADDRESS ) );
        ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( ( void* ) context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_tx( ( void* ) context, MANAGER_TX_RX_TIMEOUT_MS ) );
    }
    else
    {
        HAL_DBG_TRACE_INFO( "===== Running in RTToF subordinate mode =====\n\n" );

        /* Subordinate specific LoRa RTToF parameter configuration */
        ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( ( void* ) context, RTTOF_SUBORDINATE_IRQ_MASK, 0 ) );
        ASSERT_LR11XX_RC(
            lr11xx_rttof_set_address( ( void* ) context, RTTOF_ADDRESS, RTTOF_SUBORDINATE_CHECK_LENGTH_BYTES ) );
        ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( ( void* ) context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_rx( ( void* ) context, 0u ) );
    }

    while( 1 )
    {
        if( is_manager == true )
        {
            apps_common_lr11xx_irq_process( ( void* ) context, RTTOF_MANAGER_IRQ_MASK );
        }
        else
        {
            apps_common_lr11xx_irq_process( ( void* ) context, RTTOF_SUBORDINATE_IRQ_MASK );
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

lr11xx_status_t get_rttof_result( lr11xx_radio_lora_bw_t rttof_bw, rttof_result_t* result )
{
    lr11xx_status_t rc;
    uint8_t         buf[LR11XX_RTTOF_RESULT_LENGTH];

    /* get distance */
    rc = lr11xx_rttof_get_raw_result( context, LR11XX_RTTOF_RESULT_TYPE_RAW, buf );
    if( rc != LR11XX_STATUS_OK )
    {
        return rc;
    }
    result->distance_m = lr11xx_rttof_distance_raw_to_meter( rttof_bw, buf );

    /* get RSSI */
    rc = lr11xx_rttof_get_raw_result( context, LR11XX_RTTOF_RESULT_TYPE_RSSI, buf );
    if( rc != LR11XX_STATUS_OK )
    {
        return rc;
    }
    result->rssi = lr11xx_rttof_rssi_raw_to_value( buf );

    return rc;
}

void on_rttof_request_discarded( void )
{
    HAL_DBG_TRACE_WARNING( "RTTOF request discarded\n\n" );
    /* start new RTToF reception */
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( ( void* ) context, 0u ) );
}

void on_rttof_response_done( void )
{
    /* start new RTToF reception */
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx( ( void* ) context, 0u ) );
}

void on_rttof_exchange_valid( void )
{
    rttof_result_t result = { 0 };
    ASSERT_LR11XX_RC( get_rttof_result( LORA_BANDWIDTH, &result ) );
    HAL_DBG_TRACE_INFO( "RTToF result: Distance: %dm, RSSI: %d \n", result.distance_m, result.rssi );

    /* start new rttof transmission */
    smtc_hal_mcu_wait_ms( ( const uint32_t ) MANAGER_RTTOF_SLEEP_PERIOD );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( ( void* ) context, MANAGER_TX_RX_TIMEOUT_MS ) );
}

void on_rttof_timeout( void )
{
    HAL_DBG_TRACE_WARNING( "RTTOF request timeout\n\n" );
    /* start new rttof transmission */
    smtc_hal_mcu_wait_ms( ( const uint32_t ) MANAGER_RTTOF_SLEEP_PERIOD );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx( ( void* ) context, MANAGER_TX_RX_TIMEOUT_MS ) );
}

/* --- EOF ------------------------------------------------------------------ */
