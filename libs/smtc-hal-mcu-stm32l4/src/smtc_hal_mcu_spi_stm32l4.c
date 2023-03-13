/*!
 * @file      smtc_hal_mcu_spi_stm32l4.c
 *
 * @brief      Implementation of SPI module on top of STM32L4 Low Level drivers
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
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "smtc_hal_mcu_spi.h"
#include "smtc_hal_mcu_spi_stm32l4.h"
#include <stddef.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Maximum number of SPI instances
 */
#ifndef SMTC_HAL_MCU_SPI_STM32L4_N_INSTANCES_MAX
#define SMTC_HAL_MCU_SPI_STM32L4_N_INSTANCES_MAX 4
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
 * @brief Structure defining a SPI instance
 */
struct smtc_hal_mcu_spi_inst_s
{
    bool         is_cfged;
    SPI_TypeDef* spi;
};

/**
 * @brief Array to store the SPI instances
 */
static struct smtc_hal_mcu_spi_inst_s spi_inst_array[SMTC_HAL_MCU_SPI_STM32L4_N_INSTANCES_MAX];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Get a pointer to the first free slot
 *
 * @retval Pointer to the first free slot - NULL if there is no space left
 */
static struct smtc_hal_mcu_spi_inst_s* smtc_hal_mcu_spi_stm32l4_get_free_slot( void );

/**
 * @brief Check if the instance given as parameter is genuine
 *
 * @param [in] inst GPIO instance
 *
 * @retval true Instance is genuine
 * @retval false Instance is not genuine
 */
static bool smtc_hal_mcu_spi_stm32l4_is_real_inst( smtc_hal_mcu_spi_inst_t inst );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_hal_mcu_status_t smtc_hal_mcu_spi_init( smtc_hal_mcu_spi_cfg_t cfg, smtc_hal_mcu_spi_inst_t* inst )
{
    struct smtc_hal_mcu_spi_inst_s* spi_cfg_slot = smtc_hal_mcu_spi_stm32l4_get_free_slot( );

    if( spi_cfg_slot == NULL )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    spi_cfg_slot->spi = cfg->spi;

    if( spi_cfg_slot->spi == SPI1 )
    {
        /* Peripheral clock enable */
        LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_SPI1 );
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOA );

        /** SPI1 GPIO Configuration
        PA5   ------> SPI1_SCK
        PA6   ------> SPI1_MISO
        PA7   ------> SPI1_MOSI
        */
        LL_GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin        = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7,
            .Mode       = LL_GPIO_MODE_ALTERNATE,
            .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
            .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
            .Pull       = LL_GPIO_PULL_NO,
            .Alternate  = LL_GPIO_AF_5,
        };

        LL_GPIO_Init( GPIOA, &GPIO_InitStruct );
    }
    else if( spi_cfg_slot->spi == SPI3 )
    {
        /* Peripheral clock enable */
        LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_SPI3 );
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOC );

        /** SPI3 GPIO Configuration
        PC10   ------> SPI3_SCK
        PC11   ------> SPI3_MISO
        PC12   ------> SPI3_MOSI
        */
        LL_GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin        = LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12,
            .Mode       = LL_GPIO_MODE_ALTERNATE,
            .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
            .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
            .Pull       = LL_GPIO_PULL_NO,
            .Alternate  = LL_GPIO_AF_6,
        };

        LL_GPIO_Init( GPIOC, &GPIO_InitStruct );
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    LL_SPI_InitTypeDef SPI_InitStruct = {
        .BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV16,
        .TransferDirection = LL_SPI_FULL_DUPLEX,
        .Mode              = LL_SPI_MODE_MASTER,
        .DataWidth         = LL_SPI_DATAWIDTH_8BIT,
        .ClockPolarity     = LL_SPI_POLARITY_LOW,
        .ClockPhase        = LL_SPI_PHASE_1EDGE,
        .NSS               = LL_SPI_NSS_SOFT,
        .BitOrder          = LL_SPI_MSB_FIRST,
        .CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE,
        .CRCPoly           = 7,
    };

    if( LL_SPI_Init( spi_cfg_slot->spi, &SPI_InitStruct ) != SUCCESS )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    LL_SPI_SetRxFIFOThreshold( spi_cfg_slot->spi, LL_SPI_RX_FIFO_TH_QUARTER );
    LL_SPI_SetStandard( spi_cfg_slot->spi, LL_SPI_PROTOCOL_MOTOROLA );
    LL_SPI_DisableNSSPulseMgt( spi_cfg_slot->spi );

    LL_SPI_Enable( spi_cfg_slot->spi );
    while( LL_SPI_IsEnabled( spi_cfg_slot->spi ) == 0 )
        ;

    spi_cfg_slot->is_cfged = true;

    *inst = spi_cfg_slot;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_spi_deinit( smtc_hal_mcu_spi_inst_t* inst )
{
    smtc_hal_mcu_spi_inst_t inst_local = *inst;

    if( smtc_hal_mcu_spi_stm32l4_is_real_inst( inst_local ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst_local->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    if( inst_local->spi == SPI1 )
    {
        if( LL_SPI_DeInit( inst_local->spi ) != SUCCESS )
        {
            return SMTC_HAL_MCU_STATUS_ERROR;
        }

        LL_GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin        = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7,
            .Mode       = LL_GPIO_MODE_ANALOG,
            .Speed      = LL_GPIO_SPEED_FREQ_LOW,
            .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
            .Pull       = LL_GPIO_PULL_NO,
            .Alternate  = LL_GPIO_AF_0,
        };

        if( LL_GPIO_Init( GPIOA, &GPIO_InitStruct ) != SUCCESS )
        {
            return SMTC_HAL_MCU_STATUS_ERROR;
        }
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    inst_local->is_cfged = false;

    *inst = NULL;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_spi_rw_buffer( smtc_hal_mcu_spi_inst_t inst, const uint8_t* data_out,
                                                  uint8_t* data_in, uint16_t data_length )
{
    uint16_t rem_bytes_to_send    = data_length;
    uint16_t rem_bytes_to_receive = data_length;

    if( smtc_hal_mcu_spi_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    while( ( rem_bytes_to_send > 0 ) || ( rem_bytes_to_receive > 0 ) )
    {
        if( ( LL_SPI_GetTxFIFOLevel( inst->spi ) != LL_SPI_TX_FIFO_FULL ) && ( rem_bytes_to_send > 0 ) )
        {
            const uint8_t byte_to_transmit = ( data_out == NULL ) ? 0x00 : data_out[data_length - rem_bytes_to_send];

            LL_SPI_TransmitData8( inst->spi, byte_to_transmit );

            rem_bytes_to_send--;
        }

        if( ( LL_SPI_GetRxFIFOLevel( inst->spi ) != LL_SPI_RX_FIFO_EMPTY ) && ( rem_bytes_to_receive > 0 ) )
        {
            const uint8_t byte_received = LL_SPI_ReceiveData8( inst->spi );

            if( data_in != NULL )
            {
                data_in[data_length - rem_bytes_to_receive] = byte_received;
            }

            rem_bytes_to_receive--;
        }
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static struct smtc_hal_mcu_spi_inst_s* smtc_hal_mcu_spi_stm32l4_get_free_slot( void )
{
    for( int i = 0; i < SMTC_HAL_MCU_SPI_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( spi_inst_array[i].is_cfged == false )
        {
            return &spi_inst_array[i];
        }
    }

    return NULL;
}

static bool smtc_hal_mcu_spi_stm32l4_is_real_inst( smtc_hal_mcu_spi_inst_t inst )
{
    for( int i = 0; i < SMTC_HAL_MCU_SPI_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( inst == &spi_inst_array[i] )
        {
            return true;
        }
    }

    return false;
}

/* --- EOF ------------------------------------------------------------------ */
