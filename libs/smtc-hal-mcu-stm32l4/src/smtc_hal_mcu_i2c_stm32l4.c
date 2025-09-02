/*!
 * @file      smtc_hal_mcu_i2c_stm32l4.c
 *
 * @brief      Implementation of I2C module on top of STM32L4 Low Level drivers
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

#include "stm32l4xx.h"
#include "stm32l4xx_ll_i2c.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_system.h"
#include "smtc_hal_mcu_i2c.h"
#include "smtc_hal_mcu_i2c_stm32l4.h"
#include <stddef.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Maximum number of I2C instances
 */
#ifndef SMTC_HAL_MCU_I2C_STM32L4_N_INSTANCES_MAX
#define SMTC_HAL_MCU_I2C_STM32L4_N_INSTANCES_MAX 2
#endif

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

/**
 * @brief Structure defining a I2C instance
 */
struct smtc_hal_mcu_i2c_inst_s
{
    bool         is_cfged;
    I2C_TypeDef* i2c;
    uint8_t      current_nacks_in_a_row;
    uint8_t      max_nacks_in_a_row;
};

/**
 * @brief Array to store the I2C instances
 */
static struct smtc_hal_mcu_i2c_inst_s i2c_inst_array[SMTC_HAL_MCU_I2C_STM32L4_N_INSTANCES_MAX];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Get a pointer to the first free slot
 *
 * @retval Pointer to the first free slot - NULL if there is no space left
 */
static struct smtc_hal_mcu_i2c_inst_s* smtc_hal_mcu_i2c_stm32l4_get_free_slot( void );

/**
 * @brief Check if the instance given as parameter is genuine
 *
 * @param [in] inst I2C instance
 *
 * @retval true Instance is genuine
 * @retval false Instance is not genuine
 */
static bool smtc_hal_mcu_i2c_stm32l4_is_real_inst( smtc_hal_mcu_i2c_inst_t inst );

/**
 * @brief Check if the instance given as parameter is genuine
 *
 * @param [in] inst I2C instance
 *
 * @retval true Instance is genuine
 * @retval false Instance is not genuine
 */
static int smtc_hal_mcu_i2c_stm32l4_get_address_parameter( smtc_hal_mcu_i2c_address_format_t address_format );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_hal_mcu_status_t smtc_hal_mcu_i2c_init( smtc_hal_mcu_i2c_cfg_t cfg, smtc_hal_mcu_i2c_inst_t* inst )
{
    struct smtc_hal_mcu_i2c_inst_s* i2c_cfg_slot = smtc_hal_mcu_i2c_stm32l4_get_free_slot( );

    if( i2c_cfg_slot == NULL )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    i2c_cfg_slot->i2c = cfg->i2c;

    if( i2c_cfg_slot->i2c == I2C1 )
    {
        /* Peripheral clock enable */
        LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_I2C1 );
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOB );
        LL_RCC_SetI2CClockSource( LL_RCC_I2C1_CLKSOURCE_SYSCLK );

        /**I2C1 GPIO Configuration
        PB8   ------> I2C1_SCL
        PB9   ------> I2C1_SDA
        */
        LL_GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin        = LL_GPIO_PIN_8 | LL_GPIO_PIN_9,
            .Mode       = LL_GPIO_MODE_ALTERNATE,
            .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
            .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
            .Pull       = LL_GPIO_PULL_UP,
            .Alternate  = LL_GPIO_AF_4,
        };

        LL_GPIO_Init( GPIOB, &GPIO_InitStruct );
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    LL_I2C_InitTypeDef I2C_InitStruct = {
        .PeripheralMode  = LL_I2C_MODE_I2C,
        .Timing          = 0x00300F33,
        .AnalogFilter    = LL_I2C_ANALOGFILTER_ENABLE,
        .DigitalFilter   = 0,
        .OwnAddress1     = 0,
        .TypeAcknowledge = LL_I2C_ACK,
        .OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT,
    };

    if( LL_I2C_Init( i2c_cfg_slot->i2c, &I2C_InitStruct ) != SUCCESS )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    LL_I2C_Enable( i2c_cfg_slot->i2c );
    while( LL_I2C_IsEnabled( i2c_cfg_slot->i2c ) == 0 );

    i2c_cfg_slot->is_cfged               = true;
    i2c_cfg_slot->current_nacks_in_a_row = 0;
    i2c_cfg_slot->max_nacks_in_a_row     = cfg->max_nacks_in_a_row;

    *inst = i2c_cfg_slot;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_i2c_deinit( smtc_hal_mcu_i2c_inst_t* inst )
{
    smtc_hal_mcu_i2c_inst_t inst_local = *inst;

    if( smtc_hal_mcu_i2c_stm32l4_is_real_inst( inst_local ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst_local->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    if( inst_local->i2c == I2C1 )
    {
        if( LL_I2C_DeInit( inst_local->i2c ) != SUCCESS )
        {
            return SMTC_HAL_MCU_STATUS_ERROR;
        }

        LL_GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin        = LL_GPIO_PIN_8 | LL_GPIO_PIN_9,
            .Mode       = LL_GPIO_MODE_ANALOG,
            .Speed      = LL_GPIO_SPEED_FREQ_LOW,
            .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
            .Pull       = LL_GPIO_PULL_NO,
            .Alternate  = LL_GPIO_AF_0,
        };

        if( LL_GPIO_Init( GPIOB, &GPIO_InitStruct ) != SUCCESS )
        {
            return SMTC_HAL_MCU_STATUS_ERROR;
        }
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    inst_local->max_nacks_in_a_row     = 0;
    inst_local->current_nacks_in_a_row = 0;
    inst_local->is_cfged               = false;

    *inst = NULL;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_i2c_write_buffer( smtc_hal_mcu_i2c_inst_t           inst,
                                                     smtc_hal_mcu_i2c_address_format_t address_format,
                                                     const uint16_t device_addr, const uint8_t* data_out,
                                                     const uint16_t data_length )
{
    if( smtc_hal_mcu_i2c_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    uint8_t   i                  = 0;
    const int address_format_mcu = smtc_hal_mcu_i2c_stm32l4_get_address_parameter( address_format );

    LL_I2C_HandleTransfer( inst->i2c, device_addr, address_format_mcu, data_length, LL_I2C_MODE_AUTOEND,
                           LL_I2C_GENERATE_START_WRITE );

    /* Loop until STOP flag is raised  */
    while( !LL_I2C_IsActiveFlag_STOP( inst->i2c ) )
    {
        if( LL_I2C_IsActiveFlag_TXIS( inst->i2c ) )
        {
            LL_I2C_TransmitData8( inst->i2c, data_out[i++] );
        }
    }

    LL_I2C_ClearFlag_STOP( inst->i2c );

    if( LL_I2C_IsActiveFlag_NACK( inst->i2c ) )
    {
        LL_I2C_ClearFlag_NACK( inst->i2c );

        if( inst->max_nacks_in_a_row > 0 )
        {
            inst->current_nacks_in_a_row++;

            if( inst->current_nacks_in_a_row >= inst->max_nacks_in_a_row )
            {
                return SMTC_HAL_MCU_STATUS_ERROR;
            }
        }
    }
    else
    {
        inst->current_nacks_in_a_row = 0;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_i2c_read_buffer( smtc_hal_mcu_i2c_inst_t           inst,
                                                    smtc_hal_mcu_i2c_address_format_t address_format,
                                                    const uint16_t device_addr, uint8_t* data_in,
                                                    uint16_t const data_length )
{
    if( smtc_hal_mcu_i2c_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    uint8_t   i                  = 0;
    const int address_format_mcu = smtc_hal_mcu_i2c_stm32l4_get_address_parameter( address_format );

    LL_I2C_HandleTransfer( inst->i2c, device_addr, address_format_mcu, data_length, LL_I2C_MODE_AUTOEND,
                           LL_I2C_GENERATE_START_READ );

    /* Loop until STOP flag is raised  */
    while( !LL_I2C_IsActiveFlag_STOP( inst->i2c ) )
    {
        if( LL_I2C_IsActiveFlag_RXNE( inst->i2c ) )
        {
            data_in[i++] = LL_I2C_ReceiveData8( inst->i2c );
        }
    }

    LL_I2C_ClearFlag_STOP( inst->i2c );

    return SMTC_HAL_MCU_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static struct smtc_hal_mcu_i2c_inst_s* smtc_hal_mcu_i2c_stm32l4_get_free_slot( void )
{
    for( int i = 0; i < SMTC_HAL_MCU_I2C_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( i2c_inst_array[i].is_cfged == false )
        {
            return &i2c_inst_array[i];
        }
    }

    return NULL;
}

static bool smtc_hal_mcu_i2c_stm32l4_is_real_inst( smtc_hal_mcu_i2c_inst_t inst )
{
    for( int i = 0; i < SMTC_HAL_MCU_I2C_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( inst == &i2c_inst_array[i] )
        {
            return true;
        }
    }

    return false;
}

static int smtc_hal_mcu_i2c_stm32l4_get_address_parameter( smtc_hal_mcu_i2c_address_format_t address_format )
{
    if( address_format == SMTC_HAL_MCU_I2C_ADDRESS_FORMAT_7_BITS )
    {
        return LL_I2C_ADDRSLAVE_7BIT;
    }
    else if( address_format == SMTC_HAL_MCU_I2C_ADDRESS_FORMAT_10_BITS )
    {
        return LL_I2C_ADDRSLAVE_10BIT;
    }

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */
