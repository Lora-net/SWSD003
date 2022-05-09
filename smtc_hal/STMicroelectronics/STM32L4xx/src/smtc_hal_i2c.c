/*!
 * @file      smtc_hal_i2c.c
 *
 * @brief     Implements the i2c HAL functions
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
#include "stm32l4xx_hal.h"
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

typedef struct hal_i2c_s
{
    I2C_TypeDef*      interface;
    I2C_HandleTypeDef handle;
    struct
    {
        hal_gpio_pin_names_t sda;
        hal_gpio_pin_names_t scl;
    } pins;
} hal_i2c_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static hal_i2c_t hal_i2c[] = {
    [0] =
        {
            .interface = I2C1,
            .handle    = { NULL },
            .pins =
                {
                    .sda = NC,
                    .scl = NC,
                },
        },
    [1] =
        {
            .interface = I2C2,
            .handle    = { NULL },
            .pins =
                {
                    .sda = NC,
                    .scl = NC,
                },
        },
    [2] =
        {
            .interface = I2C3,
            .handle    = { NULL },
            .pins =
                {
                    .sda = NC,
                    .scl = NC,
                },
        },
};

static i2c_addr_size i2c_internal_addr_size;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Write data buffer to the I2C device
 *
 * @param [in] id               I2C interface id [1:N]
 * @param [in] deviceAddr       device address
 * @param [in] addr             data address
 * @param [in] buffer           data buffer to write
 * @param [in] size             number of data bytes to write
 *
 * @returns status [SMTC_HAL_SUCCESS, SMTC_HAL_FAILURE]
 */
static uint8_t i2c_write_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer,
                                 uint16_t size );

/*!
 * @brief Write data buffer to the I2C device
 *
 * @param [in] id               I2C interface id [1:N]
 * @param [in] deviceAddr       device address
 * @param [in] addr             data address
 * @param [in] buffer           data buffer to write
 * @param [in] size             number of data bytes to write
 *
 * @returns status [SMTC_HAL_SUCCESS, SMTC_HAL_FAILURE]
 */
static uint8_t i2c_read_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_i2c_init( const uint32_t id, const hal_gpio_pin_names_t sda, const hal_gpio_pin_names_t scl )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_i2c ) ) );
    uint32_t local_id = id - 1;

    hal_i2c[local_id].handle.Instance              = hal_i2c[local_id].interface;
    hal_i2c[local_id].handle.Init.Timing           = 0x10909CEC;
    hal_i2c[local_id].handle.Init.OwnAddress1      = 0;
    hal_i2c[local_id].handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hal_i2c[local_id].handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hal_i2c[local_id].handle.Init.OwnAddress2      = 0;
    hal_i2c[local_id].handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hal_i2c[local_id].handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hal_i2c[local_id].handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    hal_i2c[local_id].pins.sda = sda;
    hal_i2c[local_id].pins.scl = scl;

    if( HAL_I2C_Init( &hal_i2c[local_id].handle ) != HAL_OK )
    {
        mcu_panic( );
    }

    /**Configure Analogue filter
     */
    if( HAL_I2CEx_ConfigAnalogFilter( &hal_i2c[local_id].handle, I2C_ANALOGFILTER_ENABLE ) != HAL_OK )
    {
        mcu_panic( );
    }

    /**Configure Digital filter
     */
    if( HAL_I2CEx_ConfigDigitalFilter( &hal_i2c[local_id].handle, 0 ) != HAL_OK )
    {
        mcu_panic( );
    }
}

void hal_i2c_deinit( const uint32_t id )
{
    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_i2c ) ) );
    uint32_t local_id = id - 1;

    HAL_I2C_DeInit( &hal_i2c[local_id].handle );
}

void HAL_I2C_MspInit( I2C_HandleTypeDef* i2cHandle )
{
    if( i2cHandle->Instance == hal_i2c[0].interface )
    {
        GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_i2c[0].pins.sda & 0xF0 ) << 6 ) );
        GPIO_InitTypeDef gpio      = {
            .Mode  = GPIO_MODE_AF_OD,
            .Pull  = GPIO_PULLUP,  // Pull UP instead of no pull because pull resistors are not mounter on the shield
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF4_I2C1,
        };
        gpio.Pin = ( 1 << ( hal_i2c[0].pins.scl & 0x0F ) ) | ( 1 << ( hal_i2c[0].pins.sda & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &gpio );

        __HAL_RCC_I2C1_CLK_ENABLE( );
    }
    else if( i2cHandle->Instance == hal_i2c[1].interface )
    {
        GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_i2c[1].pins.sda & 0xF0 ) << 6 ) );
        GPIO_InitTypeDef gpio      = {
            .Mode      = GPIO_MODE_AF_OD,
            .Pull      = GPIO_NOPULL,
            .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF4_I2C2,
        };
        gpio.Pin = ( 1 << ( hal_i2c[1].pins.scl & 0x0F ) ) | ( 1 << ( hal_i2c[1].pins.sda & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &gpio );

        __HAL_RCC_I2C2_CLK_ENABLE( );
    }
    else if( i2cHandle->Instance == hal_i2c[2].interface )
    {
        GPIO_TypeDef*    gpio_port = ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_i2c[2].pins.sda & 0xF0 ) << 6 ) );
        GPIO_InitTypeDef gpio      = {
            .Mode      = GPIO_MODE_AF_OD,
            .Pull      = GPIO_NOPULL,
            .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF4_I2C3,
        };
        gpio.Pin = ( 1 << ( hal_i2c[2].pins.scl & 0x0F ) ) | ( 1 << ( hal_i2c[2].pins.sda & 0x0F ) );
        HAL_GPIO_Init( gpio_port, &gpio );

        __HAL_RCC_I2C3_CLK_ENABLE( );
    }
    else
    {
        mcu_panic( );
    }
}

void HAL_I2C_MspDeInit( I2C_HandleTypeDef* i2cHandle )
{
    uint32_t local_id = 0;
    if( i2cHandle->Instance == hal_i2c[0].interface )
    {
        local_id = 0;
        __HAL_RCC_I2C1_FORCE_RESET( );
        __HAL_RCC_I2C1_RELEASE_RESET( );
        __HAL_RCC_I2C1_CLK_DISABLE( );
    }
    else if( i2cHandle->Instance == hal_i2c[2].interface )
    {
        local_id = 2;
        __HAL_RCC_I2C3_FORCE_RESET( );
        __HAL_RCC_I2C3_RELEASE_RESET( );
        __HAL_RCC_I2C3_CLK_DISABLE( );
    }
    else
    {
        mcu_panic( );
    }

    HAL_GPIO_DeInit( ( GPIO_TypeDef* ) ( AHB2PERIPH_BASE + ( ( hal_i2c[local_id].pins.sda & 0xF0 ) << 6 ) ),
                     ( 1 << ( hal_i2c[local_id].pins.sda & 0x0F ) ) | ( 1 << ( hal_i2c[local_id].pins.scl & 0x0F ) ) );
}

uint8_t hal_i2c_write( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t data )
{
    if( i2c_write_buffer( id, device_addr, addr, &data, 1u ) == SMTC_HAL_FAILURE )
    {
        // if first attempt fails due to an IRQ, try a second time
        if( i2c_write_buffer( id, device_addr, addr, &data, 1u ) == SMTC_HAL_FAILURE )
        {
            return SMTC_HAL_FAILURE;
        }
        else
        {
            return SMTC_HAL_SUCCESS;
        }
    }
    else
    {
        return SMTC_HAL_SUCCESS;
    }
}

uint8_t hal_i2c_write_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size )
{
    if( i2c_write_buffer( id, device_addr, addr, buffer, size ) == SMTC_HAL_FAILURE )
    {
        // if first attempt fails due to an IRQ, try a second time
        if( i2c_write_buffer( id, device_addr, addr, buffer, size ) == SMTC_HAL_FAILURE )
        {
            return SMTC_HAL_FAILURE;
        }
        else
        {
            return SMTC_HAL_SUCCESS;
        }
    }
    else
    {
        return SMTC_HAL_SUCCESS;
    }
}

uint8_t hal_i2c_read( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* data )
{
    return ( i2c_read_buffer( id, device_addr, addr, data, 1 ) );
}

uint8_t hal_i2c_read_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size )
{
    return ( i2c_read_buffer( id, device_addr, addr, buffer, size ) );
}

void i2c_set_addr_size( i2c_addr_size addr_size ) { i2c_internal_addr_size = addr_size; }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint8_t i2c_write_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size )
{
    uint8_t  write_status = SMTC_HAL_FAILURE;
    uint16_t memAddSize   = 0u;

    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_i2c ) ) );
    uint32_t local_id = id - 1;

    if( i2c_internal_addr_size == I2C_ADDR_SIZE_8 )
    {
        memAddSize = I2C_MEMADD_SIZE_8BIT;
    }
    else
    {
        memAddSize = I2C_MEMADD_SIZE_16BIT;
    }
    if( HAL_I2C_Mem_Write( &hal_i2c[local_id].handle, device_addr, addr, memAddSize, buffer, size, 2000u ) == HAL_OK )
    {
        write_status = SMTC_HAL_SUCCESS;
    }
    return write_status;
}

static uint8_t i2c_read_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size )
{
    uint8_t  readStatus = SMTC_HAL_FAILURE;
    uint16_t memAddSize = 0u;

    assert_param( ( id > 0 ) && ( ( id - 1 ) < sizeof( hal_i2c ) ) );
    uint32_t local_id = id - 1;

    if( i2c_internal_addr_size == I2C_ADDR_SIZE_8 )
    {
        memAddSize = I2C_MEMADD_SIZE_8BIT;
    }
    else
    {
        memAddSize = I2C_MEMADD_SIZE_16BIT;
    }
    if( HAL_I2C_Mem_Read( &hal_i2c[local_id].handle, device_addr, addr, memAddSize, buffer, size, 2000 ) == HAL_OK )
    {
        readStatus = SMTC_HAL_SUCCESS;
    }

    return readStatus;
}
