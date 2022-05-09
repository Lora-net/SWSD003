/*!
 * \file      smtc_hal_spi.c
 *
 * \brief     SPI Hardware Abstraction Layer implementation
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

#include "smtc_hal.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_spi.h"
#include "shield_pinout.h"

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
 *  @brief SPI structure
 */
typedef struct hal_spi_s
{
    SPI_TypeDef*      interface;
    SPI_HandleTypeDef handle;
    struct
    {
        hal_gpio_pin_names_t mosi;
        hal_gpio_pin_names_t miso;
        hal_gpio_pin_names_t sclk;
    } pins;
} hal_spi_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static hal_spi_t hal_spi[] = {
    [0] =
        {
            .interface = SPI1,
            .handle    = { NULL },
            .pins =
                {
                    .mosi = NC,
                    .miso = NC,
                    .sclk = NC,
                },
        },
    [1] =
        {
            .interface = SPI2,
            .handle    = { NULL },
            .pins =
                {
                    .mosi = NC,
                    .miso = NC,
                    .sclk = NC,
                },
        },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_spi_init( const uint32_t id, const hal_gpio_pin_names_t mosi, const hal_gpio_pin_names_t miso,
                   const hal_gpio_pin_names_t sclk )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_spi ) ) );
    uint32_t local_id = id - 1;

    hal_spi[local_id].handle.Instance               = hal_spi[local_id].interface;
    hal_spi[local_id].handle.Init.Mode              = SPI_MODE_MASTER;
    hal_spi[local_id].handle.Init.Direction         = SPI_DIRECTION_2LINES;
    hal_spi[local_id].handle.Init.DataSize          = SPI_DATASIZE_8BIT;
    hal_spi[local_id].handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hal_spi[local_id].handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hal_spi[local_id].handle.Init.NSS               = SPI_NSS_SOFT;
    hal_spi[local_id].handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hal_spi[local_id].handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hal_spi[local_id].handle.Init.TIMode            = SPI_TIMODE_DISABLE;
    hal_spi[local_id].handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hal_spi[local_id].handle.Init.CRCPolynomial     = 7;

    hal_spi[local_id].pins.mosi = mosi;
    hal_spi[local_id].pins.miso = miso;
    hal_spi[local_id].pins.sclk = sclk;

    if( HAL_SPI_Init( &hal_spi[local_id].handle ) != HAL_OK )
    {
        mcu_panic( );
    }
    __HAL_SPI_ENABLE( &hal_spi[local_id].handle );
}

void hal_spi_deinit( const uint32_t id )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_spi ) ) );
    uint32_t local_id = id - 1;

    HAL_SPI_DeInit( &hal_spi[local_id].handle );
}

uint16_t hal_spi_in_out( const uint32_t id, const uint16_t out_data )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_spi ) ) );
    uint32_t local_id = id - 1;

    while( LL_SPI_IsActiveFlag_TXE( hal_spi[local_id].interface ) == 0 )
    {
    };
    LL_SPI_TransmitData8( hal_spi[local_id].interface, ( uint8_t ) ( out_data & 0xFF ) );

    while( LL_SPI_IsActiveFlag_RXNE( hal_spi[local_id].interface ) == 0 )
    {
    };
    return LL_SPI_ReceiveData8( hal_spi[local_id].interface );
}

void HAL_SPI_MspInit( SPI_HandleTypeDef* spiHandle )
{
    if( spiHandle->Instance == hal_spi[0].interface )
    {
        GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_spi[0].pins.mosi & 0xF0 ) << 6 ) );
        GPIO_InitTypeDef gpio      = {
                 .Mode      = GPIO_MODE_AF_PP,
                 .Pull      = GPIO_NOPULL,
                 .Speed     = GPIO_SPEED_FREQ_HIGH,
                 .Alternate = GPIO_AF5_SPI1,
        };
        gpio.Pin = ( 1 << ( hal_spi[0].pins.mosi & 0x0F ) ) | ( 1 << ( hal_spi[0].pins.miso & 0x0F ) ) |
                   ( 1 << ( hal_spi[0].pins.sclk & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &gpio );

        __HAL_RCC_SPI1_CLK_ENABLE( );
    }
    else if( spiHandle->Instance == hal_spi[1].interface )
    {
        GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_spi[1].pins.mosi & 0xF0 ) << 6 ) );
        GPIO_InitTypeDef gpio      = {
                 .Mode      = GPIO_MODE_AF_PP,
                 .Pull      = GPIO_NOPULL,
                 .Speed     = GPIO_SPEED_HIGH,
                 .Alternate = GPIO_AF5_SPI2,
        };
        gpio.Pin = ( 1 << ( hal_spi[1].pins.mosi & 0x0F ) ) | ( 1 << ( hal_spi[1].pins.miso & 0x0F ) ) |
                   ( 1 << ( hal_spi[1].pins.sclk & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &gpio );

        __HAL_RCC_SPI2_CLK_ENABLE( );
    }
    else
    {
        mcu_panic( );
    }
}

void HAL_SPI_MspDeInit( SPI_HandleTypeDef* spiHandle )
{
    uint32_t local_id = 0;
    if( spiHandle->Instance == hal_spi[0].interface )
    {
        __HAL_RCC_SPI1_CLK_DISABLE( );
        GPIO_TypeDef* gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_spi[0].pins.mosi & 0xF0 ) << 6 ) );

        GPIO_InitTypeDef gpio = {
            .Speed = GPIO_SPEED_LOW,
        };

        gpio.Mode = GPIO_MODE_ANALOG;
        gpio.Pull = GPIO_NOPULL;
        gpio.Pin  = ( 1 << ( hal_spi[0].pins.miso & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &gpio );

        // put mosi and sclk in input pull down mode
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_PULLDOWN;
        gpio.Pin  = ( 1 << ( hal_spi[0].pins.mosi & 0x0F ) ) | ( 1 << ( hal_spi[0].pins.sclk & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &gpio );
    }
    else if( spiHandle->Instance == hal_spi[1].interface )
    {
        local_id = 1;
        __HAL_RCC_SPI2_CLK_DISABLE( );
    }
    else
    {
        mcu_panic( );
    }
}

/* --- EOF ------------------------------------------------------------------ */
