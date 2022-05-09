/*!
 * @file      smtc_hal_uart.c
 *
 * @brief     Board specific package UART API implementation.
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "stm32l4xx_hal.h"
#include "smtc_hal_gpio_pin_names.h"
#include "smtc_hal_uart.h"
#include "smtc_hal_mcu.h"
#include "smtc_board.h"

#ifdef USE_RING_BUFFER
#include "ring_buffer.h"
#endif

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

/*!
 * @brief UART structure
 */
typedef struct hal_uart_s
{
    USART_TypeDef*     interface;
    UART_HandleTypeDef handle;
    struct
    {
        hal_gpio_pin_names_t tx;
        hal_gpio_pin_names_t rx;
    } pins;
} hal_uart_t;

static hal_uart_t hal_uart[] = {
    [0] =
        {
            .interface = USART1,
            .handle    = { NULL },
            .pins =
                {
                    .tx = NC,
                    .rx = NC,
                },
        },
    [1] =
        {
            .interface = USART2,
            .handle    = { NULL },
            .pins =
                {
                    .tx = NC,
                    .rx = NC,
                },
        },
    [2] =
        {
            .interface = USART3,
            .handle    = { NULL },
            .pins =
                {
                    .tx = NC,
                    .rx = NC,
                },
        },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

uint8_t             uart_rx_done  = false;
static volatile int byte_received = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

void USART1_IRQHandler( void );
void USART2_IRQHandler( void );
void USART3_IRQHandler( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_uart_init( const uint32_t id, const hal_gpio_pin_names_t uart_tx, const hal_gpio_pin_names_t uart_rx )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_uart ) ) );
    uint32_t local_id = id - 1;

    hal_uart[local_id].handle.Instance                    = hal_uart[local_id].interface;
    hal_uart[local_id].handle.Init.BaudRate               = 921600;
    hal_uart[local_id].handle.Init.WordLength             = UART_WORDLENGTH_8B;
    hal_uart[local_id].handle.Init.StopBits               = UART_STOPBITS_1;
    hal_uart[local_id].handle.Init.Parity                 = UART_PARITY_NONE;
    hal_uart[local_id].handle.Init.Mode                   = UART_MODE_TX_RX;
    hal_uart[local_id].handle.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    hal_uart[local_id].handle.Init.OverSampling           = UART_OVERSAMPLING_16;
    hal_uart[local_id].handle.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    hal_uart[local_id].handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    hal_uart[local_id].pins.tx = uart_tx;
    hal_uart[local_id].pins.rx = uart_rx;

    if( HAL_UART_Init( &hal_uart[local_id].handle ) != HAL_OK )
    {
        mcu_panic( );
    }
    __HAL_UART_ENABLE( &hal_uart[local_id].handle );
}

void hal_uart_deinit( const uint32_t id )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_uart ) ) );
    uint32_t local_id = id - 1;

    HAL_UART_DeInit( &hal_uart[local_id].handle );
}

void hal_uart_tx( const uint32_t id, uint8_t* buff, uint16_t len )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_uart ) ) );
    uint32_t local_id = id - 1;

    HAL_UART_Transmit( &hal_uart[local_id].handle, ( uint8_t* ) buff, len, 0xffffff );
}

void hal_uart_rx( const uint32_t id, uint8_t* rx_buffer, uint16_t len )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_uart ) ) );
    uint32_t local_id = id - 1;

    HAL_UART_Receive_IT( &hal_uart[local_id].handle, rx_buffer, len );

    while( uart_rx_done != true )
        ;

    uart_rx_done = false;
}

void hal_uart_start_rx_it( const uint32_t id )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_uart ) ) );
    uint32_t                local_id = id - 1;
    const HAL_StatusTypeDef status = HAL_UART_Receive_IT( &hal_uart[local_id].handle, ( uint8_t* ) &byte_received, 1 );
    if( status != HAL_OK )
    {
        HAL_DBG_TRACE_ERROR( "Failed rx it\n" );
        while( true )
        {
        }
    }
}

void HAL_UART_MspInit( UART_HandleTypeDef* huart )
{
    if( huart->Instance == hal_uart[0].interface )
    {
        GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_uart[0].pins.tx & 0xF0 ) << 6 ) );
        GPIO_InitTypeDef gpio      = {
            .Mode      = GPIO_MODE_AF_PP,
            .Pull      = GPIO_NOPULL,
            .Speed     = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF7_USART1,
        };
        gpio.Pin = ( 1 << ( hal_uart[0].pins.tx & 0x0F ) ) | ( 1 << ( hal_uart[0].pins.rx & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &gpio );

        /* NVIC for USART1 */
        HAL_NVIC_SetPriority( USART1_IRQn, 0, 1 );
        HAL_NVIC_EnableIRQ( USART1_IRQn );

        __HAL_RCC_USART1_CLK_ENABLE( );
    }
    else if( huart->Instance == hal_uart[1].interface )
    {
        GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_uart[1].pins.tx & 0xF0 ) << 6 ) );
        GPIO_InitTypeDef gpio      = {
            .Mode      = GPIO_MODE_AF_PP,
            .Pull      = GPIO_NOPULL,
            .Speed     = GPIO_SPEED_HIGH,
            .Alternate = GPIO_AF7_USART2,
        };
        gpio.Pin = ( 1 << ( hal_uart[1].pins.tx & 0x0F ) ) | ( 1 << ( hal_uart[1].pins.rx & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &gpio );

        /* NVIC for USART2 */
        HAL_NVIC_SetPriority( USART2_IRQn, 0, 1 );
        HAL_NVIC_EnableIRQ( USART2_IRQn );

        __HAL_RCC_USART2_CLK_ENABLE( );
    }
    else if( huart->Instance == hal_uart[2].interface )
    {
        GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_uart[2].pins.tx & 0xF0 ) << 6 ) );
        GPIO_InitTypeDef gpio      = {
            .Mode      = GPIO_MODE_AF_PP,
            .Pull      = GPIO_NOPULL,
            .Speed     = GPIO_SPEED_HIGH,
            .Alternate = GPIO_AF7_USART3,
        };
        gpio.Pin = ( 1 << ( hal_uart[2].pins.tx & 0x0F ) ) | ( 1 << ( hal_uart[2].pins.rx & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &gpio );

        /* NVIC for USART3 */
        HAL_NVIC_SetPriority( USART3_IRQn, 0, 1 );
        HAL_NVIC_EnableIRQ( USART3_IRQn );

        __HAL_RCC_USART3_CLK_ENABLE( );
    }
    else
    {
        mcu_panic( );
    }
}

void HAL_UART_MspDeInit( UART_HandleTypeDef* huart )
{
    uint32_t local_id = 0;
    if( huart->Instance == hal_uart[0].interface )
    {
        __HAL_RCC_USART1_CLK_DISABLE( );
    }
    else if( huart->Instance == hal_uart[1].interface )
    {
        local_id = 1;
        __HAL_RCC_USART2_CLK_DISABLE( );
    }
    else if( huart->Instance == hal_uart[2].interface )
    {
        local_id = 2;
        __HAL_RCC_USART3_CLK_DISABLE( );
    }
    else
    {
        mcu_panic( );
    }

    HAL_GPIO_DeInit( ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_uart[local_id].pins.tx & 0xF0 ) << 6 ) ),
                     ( 1 << ( hal_uart[local_id].pins.tx & 0x0F ) ) | ( 1 << ( hal_uart[local_id].pins.rx & 0x0F ) ) );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/**
 * @brief  This function handles USART1 interrupt request.
 */
void USART1_IRQHandler( void ) { HAL_UART_IRQHandler( &hal_uart[0].handle ); }

/**
 * @brief  This function handles USART2 interrupt request.
 */
void USART2_IRQHandler( void ) { HAL_UART_IRQHandler( &hal_uart[1].handle ); }

/**
 * @brief  This function handles USART3 interrupt request.
 */
void USART3_IRQHandler( void ) { HAL_UART_IRQHandler( &hal_uart[2].handle ); }

/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback( UART_HandleTypeDef* UartHandle )
{
    uart_rx_done = true;
#ifdef USE_RING_BUFFER
    append_to_interrupt_buffer( ( int ) byte_received );
#endif  // USE_RING_BUFFER
    hal_uart_start_rx_it( HAL_PRINTF_UART_ID );
}

/* --- EOF ------------------------------------------------------------------ */
