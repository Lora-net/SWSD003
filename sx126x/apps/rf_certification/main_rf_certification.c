/*!
 * @file      main_rf_certification.c
 *
 * @brief     RF certification example for SX126X chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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
#include "rf_certification_common.h"
#include "apps_utilities.h"
#include "sx126x.h"
#include "sx126x_str.h"
#include "sx126x_lr_fhss.h"
#include "main_rf_certification.h"
#include "smtc_dbpsk.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "stm32l4xx_ll_utils.h"
#include "smtc_hal_button.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

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

static sx126x_hal_context_t* sx126x_radio_context;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DECLARATIONS -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

int main( void )
{
    uint32_t rand_seed = 0;
    smtc_hal_mcu_init( );
    apps_common_shield_init( );
    init_user_btn( );
    uart_init( );

    apps_common_sx126x_print_version_info( );

    sx126x_radio_context = apps_common_sx126x_get_context( );
    rf_certification_init_radio_context( sx126x_radio_context );

    apps_common_sx126x_init( ( void* ) sx126x_radio_context );

    sx126x_set_dio_irq_params( sx126x_radio_context, SX126X_IRQ_ALL, IRQ_MASK, SX126X_IRQ_NONE, SX126X_IRQ_NONE );

    sx126x_clear_irq_status( sx126x_radio_context, SX126X_IRQ_ALL );
    sx126x_get_random_numbers( sx126x_radio_context, &rand_seed, 1 );

    srand( rand_seed );

#ifdef MODE_RX_ONLY
    while( true )
    {
        rf_certification_common_rx_infinite( );
    }
#elif defined( MODE_TX_RX )
#ifdef REGION_ETSI
#include "etsi.h"
    HAL_DBG_TRACE_INFO( "===== SX126x certification example region: - ETSI =====\n\n" );
    etsi_tests_fsm( );
#elif defined( REGION_FCC )
#include "fcc.h"
    HAL_DBG_TRACE_INFO( "===== SX126x certification example region: - FCC =====\n\n" );
    fcc_init_radio_context( sx126x_radio_context );
    fcc_tests_fsm( );
#elif defined( REGION_ARIB )
#include "arib.h"
    HAL_DBG_TRACE_INFO( "===== SX126x certification example region: - ARIB =====\n\n" );
    arib_tests_fsm( );
#endif
#endif

    HAL_DBG_TRACE_INFO( "===== SX126x certification example terminated =====\n\n" );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
