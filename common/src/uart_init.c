/*!
 * @file      uart_init.c
 *
 * @brief     UART init helper functions implementation
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
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include "uart_init.h"
#include "stm32l4xx.h"
#include "smtc_hal_mcu_uart_stm32l4.h"

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

static smtc_hal_mcu_uart_inst_t inst_uart = NULL;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Base function to initialize UART peripheral
 *
 * @param callback_rx The callback called on RX byte reception. Can be NULL
 */
void uart_init_base( void ( *callback_rx )( uint8_t data ) );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void uart_init( void )
{
    uart_init_base( NULL );
}

void uart_init_with_rx_callback( void ( *callback_rx )( uint8_t data ) )
{
    uart_init_base( callback_rx );
}

void vprint( const char* fmt, va_list argp )
{
    char string[255];
    if( 0 < vsprintf( string, fmt, argp ) )  // build string
    {
        smtc_hal_mcu_uart_send( inst_uart, ( uint8_t* ) string, strlen( string ) );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void uart_init_base( void ( *callback_rx )( uint8_t data ) )
{
    const struct smtc_hal_mcu_uart_cfg_s cfg_uart = {
        .usart = USART2,
    };
    const smtc_hal_mcu_uart_cfg_app_t uart_cfg_app = {
        .baudrate    = 921600,
        .callback_rx = callback_rx,
    };
    smtc_hal_mcu_uart_init( ( const smtc_hal_mcu_uart_cfg_t ) &cfg_uart, &uart_cfg_app, &inst_uart );
}

/* --- EOF ------------------------------------------------------------------ */
