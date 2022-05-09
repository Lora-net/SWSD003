/*!
 * @file      leds.c
 *
 * @brief     Leds driver implementation.
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

#include "leds.h"
#include "smtc_hal.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void leds_init( void )
{
    hal_gpio_init_out( SMTC_LED_TX, 0 );
    hal_gpio_init_out( SMTC_LED_RX, 0 );
    hal_gpio_init_out( SMTC_LED_SCAN, 0 );
}

void leds_deinit( void )
{
    hal_gpio_deinit( SMTC_LED_TX );
    hal_gpio_deinit( SMTC_LED_RX );
    hal_gpio_deinit( SMTC_LED_SCAN );
}

void leds_on( uint8_t leds )
{
    if( leds & LED_TX_MASK )
    {
        /* LED TX */
        hal_gpio_set_value( SMTC_LED_TX, HAL_GPIO_SET );
    }
    if( leds & LED_RX_MASK )
    {
        /* LED RX */
        hal_gpio_set_value( SMTC_LED_RX, HAL_GPIO_SET );
    }
    if( leds & LED_SCAN_MASK )
    {
        /* LED SCAN */
        hal_gpio_set_value( SMTC_LED_SCAN, HAL_GPIO_SET );
    }
}

void leds_off( uint8_t leds )
{
    if( leds & LED_TX_MASK )
    {
        /* LED TX */
        hal_gpio_set_value( SMTC_LED_TX, HAL_GPIO_RESET );
    }
    if( leds & LED_RX_MASK )
    {
        /* LED RX */
        hal_gpio_set_value( SMTC_LED_RX, HAL_GPIO_RESET );
    }
    if( leds & LED_SCAN_MASK )
    {
        /* LED SCAN */
        hal_gpio_set_value( SMTC_LED_SCAN, HAL_GPIO_RESET );
    }
}

void leds_toggle( uint8_t leds )
{
    if( leds & LED_TX_MASK )
    {
        /* LED TX */
        hal_gpio_toggle( SMTC_LED_TX );
    }
    if( leds & LED_RX_MASK )
    {
        /* LED RX */
        hal_gpio_toggle( SMTC_LED_RX );
    }
    if( leds & LED_SCAN_MASK )
    {
        /* LED SCAN */
        hal_gpio_toggle( SMTC_LED_SCAN );
    }
}

uint32_t leds_get_state( uint8_t leds )
{
    uint32_t leds_state = 0;

    if( leds & LED_TX_MASK )
    {
        /* LED TX */
        leds_state += hal_gpio_get_value( SMTC_LED_TX ) << LR11XX_EVK_LED_TX;
    }
    if( leds & LED_RX_MASK )
    {
        /* LED RX */
        leds_state += hal_gpio_get_value( SMTC_LED_RX ) << LR11XX_EVK_LED_RX;
    }
    if( leds & LED_SCAN_MASK )
    {
        /* LED SCAN */
        leds_state += hal_gpio_get_value( SMTC_LED_SCAN ) << LR11XX_EVK_LED_SCAN;
    }

    return leds_state;
}

void leds_blink( uint8_t leds, uint32_t delay, uint8_t nb_blink, bool reset_leds )
{
    uint8_t i = 0;

    if( reset_leds == true )
    {
        leds_off( LED_ALL_MASK );
    }

    while( i < nb_blink )
    {
        i++;
        leds_on( leds );
        hal_mcu_delay_ms( delay / 2 );
        leds_off( leds );
        hal_mcu_delay_ms( delay / 2 );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
