/*!
 * @file      main_tx_lr_fhss.c
 *
 * @brief     TX LR-FHSS example for LR11xx chip
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
#include <stdlib.h>
#include <time.h>

#include "apps_common.h"
#include "apps_utilities.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"
#include "lr11xx_lr_fhss.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "main_tx_lr_fhss.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static const uint8_t lr_fhss_sync_word[LR_FHSS_SYNC_WORD_BYTES] = { 0x2C, 0x0F, 0x79, 0x95 };

static lr11xx_lr_fhss_params_t lr_fhss_params;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_hal_context_t* context;

static uint8_t buffer[PAYLOAD_LENGTH];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Configure a LR-FHSS payload and send it, element 0 of the payload is incremented by one at each call
 *
 * @param [in] params Parameter configuration structure of the LR-FHSS
 * @param [in] payload The payload to send. It is the responsibility of the caller to ensure that this references an
 * array containing at least length elements
 * @param [in] length The length of the payload
 */
static void build_frame_and_send( const lr11xx_lr_fhss_params_t* params, uint8_t* payload, uint16_t length );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    /* Init board */
    smtc_hal_mcu_init( );
    apps_common_shield_init( );
    uart_init( );
    srand( time( NULL ) );

    HAL_DBG_TRACE_INFO( "===== LR11xx TX LR-FHSS example =====\n\n" );
    apps_common_print_sdk_driver_version( );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );
    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );
    apps_common_lr11xx_radio_init( ( void* ) context );

    ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( context, LR11XX_SYSTEM_IRQ_TX_DONE, LR11XX_SYSTEM_IRQ_NONE ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );

    for( int i = 0; i < PAYLOAD_LENGTH; i++ )
    {
        buffer[i] = i;
    }

    /* Init parameters */
    lr_fhss_params.device_offset                  = 0;
    lr_fhss_params.lr_fhss_params.bw              = LR_FHSS_BANDWIDTH;
    lr_fhss_params.lr_fhss_params.cr              = LR_FHSS_CODING_RATE;
    lr_fhss_params.lr_fhss_params.enable_hopping  = LR_FHSS_ENABLE_HOPPING;
    lr_fhss_params.lr_fhss_params.grid            = LR_FHSS_GRID;
    lr_fhss_params.lr_fhss_params.header_count    = LR_FHSS_HEADER_COUNT;
    lr_fhss_params.lr_fhss_params.modulation_type = LR_FHSS_MODULATION_TYPE;
    lr_fhss_params.lr_fhss_params.sync_word       = lr_fhss_sync_word;

    build_frame_and_send( &lr_fhss_params, buffer, PAYLOAD_LENGTH );

    while( 1 )
    {
        apps_common_lr11xx_irq_process( context, LR11XX_SYSTEM_IRQ_TX_DONE );
    }
}

void on_tx_done( void )
{
    apps_common_lr11xx_handle_post_tx( );

    LL_mDelay( TX_TO_TX_DELAY_IN_MS );

    build_frame_and_send( &lr_fhss_params, buffer, PAYLOAD_LENGTH );
}

void build_frame_and_send( const lr11xx_lr_fhss_params_t* params, uint8_t* payload, uint16_t length )
{
    const uint16_t hop_seq_id = ( uint16_t )( rand( ) % lr11xx_lr_fhss_get_hop_sequence_count( params ) );

    lr11xx_lr_fhss_build_frame( ( void* ) context, &lr_fhss_params, hop_seq_id, payload, length );

    buffer[0]++;

    apps_common_lr11xx_handle_pre_tx( );
    lr11xx_radio_set_tx( ( void* ) context, 0 );
}