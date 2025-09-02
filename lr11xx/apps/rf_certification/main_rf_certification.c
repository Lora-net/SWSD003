/*!
 * @file      main_rf_certification.c
 *
 * @brief     RF certification example for LR11xx chip
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
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"
#include "main_rf_certification.h"
#include "smtc_dbpsk.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "smtc_hal_button.h"

#include "etsi.h"
#include "fcc.h"
#include "arib.h"

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

static lr11xx_hal_context_t* lr11xx_radio_context;

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

    apps_common_print_sdk_driver_version( );

    lr11xx_radio_context = apps_common_lr11xx_get_context( );
    rf_certification_init_radio_context( lr11xx_radio_context );

    apps_common_lr11xx_fetch_and_print_version( ( void* ) lr11xx_radio_context );
    apps_common_lr11xx_system_init( ( void* ) lr11xx_radio_context );

    lr11xx_system_set_dio_irq_params( lr11xx_radio_context, IRQ_MASK, 0 );
    lr11xx_system_clear_irq_status( lr11xx_radio_context, LR11XX_SYSTEM_IRQ_ALL_MASK );
    lr11xx_system_get_random_number( lr11xx_radio_context, &rand_seed );

    srand( rand_seed );

#ifdef MODE_RX_ONLY
    while( true )
    {
        rf_certification_common_rx_infinite( );
    }
#elif defined( MODE_TX_RX )
#ifdef REGION_ETSI
    HAL_DBG_TRACE_INFO( "===== LR11xx certification example region: - ETSI =====\n\n" );
    etsi_tests_fsm( );
#elif defined( REGION_FCC )
    HAL_DBG_TRACE_INFO( "===== LR11xx certification example region: - FCC =====\n\n" );
    fcc_init_radio_context( lr11xx_radio_context );
    fcc_tests_fsm( );
#elif defined( REGION_ARIB )
    HAL_DBG_TRACE_INFO( "===== LR11xx certification example region: - ARIB =====\n\n" );
    arib_tests_fsm( );
#endif
#endif
    HAL_DBG_TRACE_INFO( "===== LR11xx certification example terminated =====\n\n" );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
