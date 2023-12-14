/*!
 * @file      main_bluetooth_low_energy_beaconing_compatibility.c
 *
 * @brief     Bluetooth Low Energy(R) Beaconing Compatibility example for LR11xx chip
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

#include "apps_common.h"
#include "apps_utilities.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"
#include "main_bluetooth_low_energy_beaconing_compatibility.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK ( LR11XX_SYSTEM_IRQ_TX_DONE )

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

static lr11xx_hal_context_t* context;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Start Bluetooth Low Energy(R) beacon transmission.
 */
static void send_bluetooth_low_energy_beacon( void );

/**
 * @brief Execute PLL calibration steps specific to LR1110 transceiver
 */
static void initialize_lr1110_pll_for_bluetooth_low_energy( void );

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

    HAL_DBG_TRACE_INFO( "===== LR11x0 Bluetooth Low Energy(R) Beaconing Compatibility example =====\n\n" );

    apps_common_print_sdk_driver_version( );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( ( void* ) context );

    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );

    /* Check version - expect LR1110 or LR1120 */
    lr11xx_system_version_t version = { 0 };
    ASSERT_LR11XX_RC( lr11xx_system_get_version( context, &version ) );
    if( ( version.type != LR11XX_SYSTEM_VERSION_TYPE_LR1110 ) && ( version.type != LR11XX_SYSTEM_VERSION_TYPE_LR1120 ) )
    {
        HAL_DBG_TRACE_ERROR(
            "Bluetooth Low Energy(R) Beaconing Compatibility example application requires LR1110 or LR1120\n" );
        while( true )
        {
        }
    }

    // If a LR1110 transceiver is used, then proceed with a specific initialization
    if( version.type == LR11XX_SYSTEM_VERSION_TYPE_LR1110 )
    {
        initialize_lr1110_pll_for_bluetooth_low_energy( );
    }

    ASSERT_LR11XX_RC( lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );

    // power the HF PA from VREG
    const lr11xx_radio_pa_cfg_t pa_config = {
        .pa_sel        = LR11XX_RADIO_PA_SEL_HF,
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
        .pa_duty_cycle = 4u,
        .pa_hp_sel     = 0u,
    };
    ASSERT_LR11XX_RC( lr11xx_radio_set_pa_cfg( context, &pa_config ) );
    ASSERT_LR11XX_RC(
        lr11xx_radio_set_tx_params( context, BLUETOOTH_LOW_ENERGY_BEACON_TX_POWER, LR11XX_RADIO_RAMP_16_US ) );

    // Advertising Channel PDU (Eddystone-format Beacon)
    const uint8_t pdu_buf[30] = {
        0x02,                                     // Preamble (ADV_NONCONN_IND)
        28,                                       // Length
        0xa4, 0x63, 0xef, 0x8c, 0x89, 0xe6,       // ADV Address
        0x02, 0x01, 0x06,                         // Advertising Data AD0
        0x03, 0x03, 0xaa, 0xfe,                   // Advertising Data AD1
        0x0e,                                     // Advertising Data AD2 - AD Length
        0x16,                                     // Advertising Data AD2 - AD Type
        0xaa, 0xfe,                               // Advertising Data AD2 - Eddystone UUID
        0x10,                                     // Eddystone URL Frame - Frame Type
        0x09,                                     // Eddystone URL Frame - TX Power
        0x00,                                     // Eddystone URL Frame - URL Scheme
        's',  'e',  'm',  't',  'e',  'c',  'h',  // Eddystone URL Frame - Encoded URL
        0x07                                      // Eddystone URL Frame - Encoded URL (.com)
    };
    ASSERT_LR11XX_RC( lr11xx_radio_cfg_bluetooth_low_energy_beaconning_compatibility(
        context, BLUETOOTH_LOW_ENERGY_BEACON_CHANNEL, pdu_buf, sizeof( pdu_buf ) ) );

    send_bluetooth_low_energy_beacon( );

    while( 1 )
    {
        apps_common_lr11xx_irq_process( ( void* ) context, IRQ_MASK );
    }
}

void on_tx_done( void )
{
    apps_common_lr11xx_handle_post_tx( );
    send_bluetooth_low_energy_beacon( );
}

static void send_bluetooth_low_energy_beacon( )
{
    LL_mDelay( BLUETOOTH_LOW_ENERGY_BEACON_PERIOD_MS );
    lr11xx_radio_set_tx( ( void* ) context, 0 );
}

void initialize_lr1110_pll_for_bluetooth_low_energy( void )
{
    ASSERT_LR11XX_RC( lr11xx_radio_set_rf_freq( ( void* ) context, 2400000000 ) );
    ASSERT_LR11XX_RC( lr11xx_system_calibrate( context, 0x20 ) );
}
