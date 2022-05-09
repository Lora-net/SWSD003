/*!
 * @file      lr11x0mb1xys_board.c
 *
 * @brief     Target board LR11x0 MB1xyS shield board board driver implementation
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

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "smtc_hal.h"
#include "shield_pinout.h"
#include "lis2de12.h"
#include "leds.h"
#include "external_supply.h"
#include "smtc_board.h"
#include "lr11xx_hal_context.h"
#include "smtc_hal.h"
#include "lr11xx_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
static const lr11xx_radio_rssi_calibration_table_t rssi_calibration_table_below_600mhz = {
    .gain_offset = 0,
    .gain_tune   = { .g4     = 12,
                     .g5     = 12,
                     .g6     = 14,
                     .g7     = 0,
                     .g8     = 1,
                     .g9     = 3,
                     .g10    = 4,
                     .g11    = 4,
                     .g12    = 3,
                     .g13    = 6,
                     .g13hp1 = 6,
                     .g13hp2 = 6,
                     .g13hp3 = 6,
                     .g13hp4 = 6,
                     .g13hp5 = 6,
                     .g13hp6 = 6,
                     .g13hp7 = 6 },
};

static const lr11xx_radio_rssi_calibration_table_t rssi_calibration_table_from_600mhz_to_2ghz = {
    .gain_offset = 0,
    .gain_tune   = { .g4     = 2,
                     .g5     = 2,
                     .g6     = 2,
                     .g7     = 3,
                     .g8     = 3,
                     .g9     = 4,
                     .g10    = 5,
                     .g11    = 4,
                     .g12    = 4,
                     .g13    = 6,
                     .g13hp1 = 5,
                     .g13hp2 = 5,
                     .g13hp3 = 6,
                     .g13hp4 = 6,
                     .g13hp5 = 6,
                     .g13hp6 = 7,
                     .g13hp7 = 6 },
};

static const lr11xx_radio_rssi_calibration_table_t rssi_calibration_table_above_2ghz = {
    .gain_offset = 2030,
    .gain_tune   = { .g4     = 6,
                     .g5     = 7,
                     .g6     = 6,
                     .g7     = 4,
                     .g8     = 3,
                     .g9     = 4,
                     .g10    = 14,
                     .g11    = 12,
                     .g12    = 14,
                     .g13    = 12,
                     .g13hp1 = 12,
                     .g13hp2 = 12,
                     .g13hp3 = 12,
                     .g13hp4 = 8,
                     .g13hp5 = 8,
                     .g13hp6 = 9,
                     .g13hp7 = 9 },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief LR11XX EVK LED context
 */
typedef struct
{
    timer_event_t led_timer;         /*!< @brief Pulse timer */
    bool          timer_initialized; /*!< @brief True if the pulse timer has been initialized, false otherwise */
} smtc_board_led_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief LR11XX EVK LED context array
 */
static smtc_board_led_ctx_t smtc_board_leds[LR11XX_EVK_LED_COUNT] = {
    { .timer_initialized = false },
    { .timer_initialized = false },
    { .timer_initialized = false },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Turn on/off the requested LED(s)
 *
 * @param [in] led_mask Mask representing the list of the LEDs to turn on/off
 * @param [in] turn_on If true, the requested LEDs are turned on, else they are turned off
 */
static void smtc_board_led_set( uint32_t led_mask, bool turn_on );

/*!
 * @brief Get the status on/off of the requested LED(s)
 *
 * @param [in] led_mask Mask representing the list of the LEDs to turn on/off
 *
 * @return LED mask state
 */
static uint32_t smtc_board_led_get( uint32_t led_mask );

/*!
 * @brief Return the  mask for TX LED
 *
 * The mask is to be used for LED control functions like smtc_board_led_set
 *
 * @return TX LED mask
 */
static uint32_t smtc_board_get_led_tx_mask( void );

/*!
 * @brief Return the  mask for RX LED
 *
 * The mask is to be used for LED control functions like smtc_board_led_set
 *
 * @return RX LED mask
 */
static uint32_t smtc_board_get_led_rx_mask( void );

/*!
 * @brief Return the mask for SCAN LED
 *
 * The mask is to be used for LED control functions like smtc_board_led_set
 *
 * @return SCAN LED mask
 */
static uint32_t smtc_board_get_led_scan_mask( void );

/*!
 * @brief Pulse timer timeout callback
 *
 * @param context Context used to retrieve the index of the relevant LED.
 */
static void on_led_timer_event( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void smtc_board_init_periph( void )
{
    /* Init TX & RX Leds */
    leds_init( );

    /* External supplies */
    external_supply_init( LNA_SUPPLY_MASK );

    /* LIS2DE12 accelerometer */
    accelerometer_init( INT_1 );
}

void smtc_board_reinit_periph( void )
{
    leds_init( );

    /* External supplies */
    external_supply_init( LNA_SUPPLY_MASK );
}

void smtc_board_deinit_periph( void )
{
    leds_deinit( );

    /* Disable external supply */
    external_supply_deinit( LNA_SUPPLY_MASK );
}

void smtc_board_led_set( uint32_t led_mask, bool turn_on )
{
    /* If a pulse timer is running on one of the requested LEDs, it
     *  must be stopped to avoid conflicting with the requested LED state. */
    lr11xx_evk_led_t led = LR11XX_EVK_LED_TX;
    for( led = LR11XX_EVK_LED_TX; led < LR11XX_EVK_LED_COUNT; led++ )
    {
        if( led_mask & ( 1 << led ) )
        {
            if( ( smtc_board_leds[led].timer_initialized ) && ( timer_is_started( &smtc_board_leds[led].led_timer ) ) )
            {
                timer_stop( &smtc_board_leds[led].led_timer );
            }
        }
    }
    if( turn_on )
    {
        leds_on( led_mask );
    }
    else
    {
        leds_off( led_mask );
    }
}

uint32_t smtc_board_led_get( uint32_t led_mask )
{
    return leds_get_state( led_mask );
}

uint32_t smtc_board_get_led_tx_mask( void )
{
    return LED_TX_MASK;
}

uint32_t smtc_board_get_led_rx_mask( void )
{
    return LED_RX_MASK;
}

uint32_t smtc_board_get_led_scan_mask( void )
{
    return LED_SCAN_MASK;
}

void smtc_shield_handle_pre_tx( void )
{
    smtc_board_led_set( smtc_board_get_led_tx_mask( ), true );
}

void smtc_shield_handle_post_tx( void )
{
    smtc_board_led_set( smtc_board_get_led_tx_mask( ), false );
}

void smtc_shield_handle_pre_rx( void )
{
    smtc_board_led_set( smtc_board_get_led_rx_mask( ), true );
}

void smtc_shield_handle_post_rx( void )
{
    smtc_board_led_set( smtc_board_get_led_rx_mask( ), false );
}

void smtc_shield_handle_pre_gnss_scan( void )
{
    smtc_board_led_set( smtc_board_get_led_scan_mask( ), true );
    lna_on( );
};

void smtc_shield_handle_post_gnss_scan( void )
{
    smtc_board_led_set( smtc_board_get_led_scan_mask( ), false );
    lna_off( );
}

void smtc_shield_handle_pre_wifi_scan( void )
{
    smtc_board_led_set( smtc_board_get_led_scan_mask( ), true );
}

void smtc_shield_handle_post_wifi_scan( void )
{
    smtc_board_led_set( smtc_board_get_led_scan_mask( ), false );
}

lr11xx_system_reg_mode_t smtc_board_get_reg_mode( void )
{
    return LR11XX_SYSTEM_REG_MODE_DCDC;
}

smtc_board_tcxo_cfg_t smtc_board_get_tcxo_cfg( void )
{
    return ( smtc_board_tcxo_cfg_t ){
        .has_tcxo   = true,
        .supply     = LR11XX_SYSTEM_TCXO_CTRL_3_0V,
        .timeout_ms = 5,
    };
}

smtc_board_lf_clck_cfg_t smtc_board_get_lf_clk_cfg( void )
{
    return ( smtc_board_lf_clck_cfg_t ){
        .lf_clk_cfg     = LR11XX_SYSTEM_LFCLK_XTAL,
        .wait_32k_ready = true,
    };
}

lr11xx_system_rfswitch_cfg_t smtc_board_get_rf_switch_cfg( void )
{
    lr11xx_system_rfswitch_cfg_t rf_switch_setup = { 0 };

    rf_switch_setup.enable =
        LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH | LR11XX_SYSTEM_RFSW2_HIGH | LR11XX_SYSTEM_RFSW3_HIGH;
    rf_switch_setup.standby = 0;
    rf_switch_setup.tx      = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH;
    rf_switch_setup.tx_hp   = LR11XX_SYSTEM_RFSW1_HIGH;
    rf_switch_setup.rx      = LR11XX_SYSTEM_RFSW0_HIGH;
    rf_switch_setup.wifi    = LR11XX_SYSTEM_RFSW3_HIGH;
    rf_switch_setup.gnss    = LR11XX_SYSTEM_RFSW2_HIGH;
    return rf_switch_setup;
}

const lr11xx_radio_rssi_calibration_table_t* smtc_board_get_rssi_calibration_table( const uint32_t freq_in_hz )
{
    if( freq_in_hz < 600000000 )
    {
        return &rssi_calibration_table_below_600mhz;
    }
    else if( ( 600000000 <= freq_in_hz ) && ( freq_in_hz <= 2000000000 ) )
    {
        return &rssi_calibration_table_from_600mhz_to_2ghz;
    }
    else
    {
        return &rssi_calibration_table_above_2ghz;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void on_led_timer_event( void* context )
{
    lr11xx_evk_led_t led      = ( lr11xx_evk_led_t ) context;
    uint32_t         led_mask = 1 << led;
    leds_toggle( led_mask );
    timer_stop( &smtc_board_leds[led].led_timer );
}

/* --- EOF ------------------------------------------------------------------ */
