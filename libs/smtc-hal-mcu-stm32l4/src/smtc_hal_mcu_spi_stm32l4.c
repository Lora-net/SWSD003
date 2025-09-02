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
#include "stm32l4xx_ll_dma.h"
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

typedef struct
{
    SPI_TypeDef* spi;
    DMA_TypeDef* dma;
    uint32_t     ahb_bus;
    uint32_t     dma_channel_irq_rx;
    uint32_t     dma_channel_irq_tx;
    uint32_t     dma_channel_rx;
    uint32_t     dma_channel_tx;
    uint32_t     periph_request;
} smtc_hal_mcu_spi_stm32l4_dma_periph_mapping_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Structure defining a SPI instance
 */
struct smtc_hal_mcu_spi_inst_s
{
    bool                                                 is_cfged;
    SPI_TypeDef*                                         spi;
    const smtc_hal_mcu_spi_stm32l4_dma_periph_mapping_t* dma_periph_mapping;
    void ( *DMAcallback_tx )( void );
    void ( *DMAcallback_rx )( void );
    void ( *DMAcallback_tx_error )( void );
    void ( *DMAcallback_rx_error )( void );
};

/**
 * @brief Array to store the SPI instances
 */
static struct smtc_hal_mcu_spi_inst_s spi_inst_array[SMTC_HAL_MCU_SPI_STM32L4_N_INSTANCES_MAX];

/**
 * @brief Static null value
 *
 * This null value has a static storage duration.
 * It ensures it always exists during DMA operation. It is used when DMA is used in a way that it has to send always the
 * same value.
 */
static const uint8_t dma_null_value = 0;

/**
 * @brief Static garbage value
 *
 * This value is used to provide a static storage that DMA uses when dumping value that should not be returned.
 */
static uint8_t garbage_value = 0;

/**
 * @brief Array of SPI/DMA peripheral configuration mapping
 */
static const smtc_hal_mcu_spi_stm32l4_dma_periph_mapping_t dma_periph_mappping[] = {
    {
        .spi                = SPI1,
        .dma                = DMA1,
        .ahb_bus            = LL_AHB1_GRP1_PERIPH_DMA1,
        .dma_channel_irq_rx = DMA1_Channel2_IRQn,
        .dma_channel_irq_tx = DMA1_Channel3_IRQn,
        .dma_channel_rx     = LL_DMA_CHANNEL_2,
        .dma_channel_tx     = LL_DMA_CHANNEL_3,
        .periph_request     = LL_DMA_REQUEST_1,
    },
    {
        .spi                = SPI2,
        .dma                = DMA1,
        .ahb_bus            = LL_AHB1_GRP1_PERIPH_DMA1,
        .dma_channel_irq_rx = DMA1_Channel4_IRQn,
        .dma_channel_irq_tx = DMA1_Channel5_IRQn,
        .dma_channel_rx     = LL_DMA_CHANNEL_4,
        .dma_channel_tx     = LL_DMA_CHANNEL_5,
        .periph_request     = LL_DMA_REQUEST_1,
    },
    {
        .spi                = SPI3,
        .dma                = DMA2,
        .ahb_bus            = LL_AHB1_GRP1_PERIPH_DMA2,
        .dma_channel_irq_rx = DMA2_Channel1_IRQn,
        .dma_channel_irq_tx = DMA2_Channel2_IRQn,
        .dma_channel_rx     = LL_DMA_CHANNEL_1,
        .dma_channel_tx     = LL_DMA_CHANNEL_2,
        .periph_request     = LL_DMA_REQUEST_3,
    }
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Get DMA peripheric configuration from SPI peripheral
 */
static const smtc_hal_mcu_spi_stm32l4_dma_periph_mapping_t* smtc_hal_mcu_spi_stm32l4_get_dma_peripheral_configuration(
    SPI_TypeDef* spi );

/**
 * @brief Get a pointer to the first free slot
 *
 * @retval Pointer to the first free slot - NULL if there is no space left
 */
static struct smtc_hal_mcu_spi_inst_s* smtc_hal_mcu_spi_stm32l4_get_free_slot( void );

/**
 * @brief Check if the instance given as parameter is genuine
 *
 * @param [in] inst SPI instance
 *
 * @retval true Instance is genuine
 * @retval false Instance is not genuine
 */
static bool smtc_hal_mcu_spi_stm32l4_is_real_inst( smtc_hal_mcu_spi_inst_t inst );

/**
 * @brief Factorize IRQ calls for DMA channel IRQ
 *
 * @param dma The DMA peripheral that triggered the IRQ
 * @param channel The channel from DMA peripheral that triggered the IRQ
 */
static void smtc_hal_mcu_spi_stm32l4_call_dma_callback_irq( DMA_TypeDef* dma, uint32_t channel );

/**
 * @brief Factorize the call that determine if DMA's channel TC flag is set
 *
 * @param channel The channel to check
 * @param dma The DMA peripheral to check
 * @return uint32_t
 */
static uint32_t smtc_hal_mcu_spi_stm32l4_is_active_flag_tc( uint32_t channel, DMA_TypeDef* dma );

/**
 * @brief Factorize the clear of Global Interrupt DMA flag
 *
 * @param channel The channel to clear
 * @param dma The DMA peripheral to clear
 */
static void smtc_hal_mcu_spi_stm32l4_clear_flag_gi( uint32_t channel, DMA_TypeDef* dma );

/**
 * @brief Factorize the call that determine if DMA's channel TE flag is set
 *
 * @param channel The channel to check
 * @param dma The DMA peripheral to check
 * @return uint32_t
 */
static uint32_t smtc_hal_mcu_spi_stm32l4_is_active_flag_te( uint32_t channel, DMA_TypeDef* dma );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_hal_mcu_status_t smtc_hal_mcu_spi_init( smtc_hal_mcu_spi_cfg_t cfg, const smtc_hal_mcu_spi_cfg_app_t* cfg_app,
                                             smtc_hal_mcu_spi_inst_t* inst )
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
    else if( spi_cfg_slot->spi == SPI2 )
    {
        /* Peripheral clock enable */
        LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_SPI2 );
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOB );

        /** SPI2 GPIO Configuration
        PB13   ------> SPI2_SCK
        PB14   ------> SPI2_MISO
        PB15   ------> SPI2_MOSI
        */
        LL_GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin        = LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
            .Mode       = LL_GPIO_MODE_ALTERNATE,
            .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
            .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
            .Pull       = LL_GPIO_PULL_NO,
            .Alternate  = LL_GPIO_AF_5,
        };

        LL_GPIO_Init( GPIOB, &GPIO_InitStruct );
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
        .Mode              = ( cfg_app->is_master ) ? LL_SPI_MODE_MASTER : LL_SPI_MODE_SLAVE,
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
    while( LL_SPI_IsEnabled( spi_cfg_slot->spi ) == 0 );

    spi_cfg_slot->is_cfged           = true;
    spi_cfg_slot->dma_periph_mapping = 0;

    *inst = spi_cfg_slot;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_spi_dma_init( const smtc_hal_mcu_spi_dma_cfg_t      cfg,
                                                 const smtc_hal_mcu_spi_dma_cfg_app_t* cfg_app,
                                                 smtc_hal_mcu_spi_inst_t*              inst )
{
    const smtc_hal_mcu_spi_stm32l4_dma_periph_mapping_t* dma_periph_configuration =
        smtc_hal_mcu_spi_stm32l4_get_dma_peripheral_configuration( cfg->spi );
    if( dma_periph_configuration != 0 )
    {
        smtc_hal_mcu_status_t   status;
        smtc_hal_mcu_spi_inst_t spi_cfg_slot = NULL;

        /* Setting up SPI for DMA usage */
        struct smtc_hal_mcu_spi_cfg_s cfg_spi = {
            .spi = dma_periph_configuration->spi,
        };
        const struct smtc_hal_mcu_spi_cfg_app_s cfg_spi_app = {
            .is_master = cfg_app->is_master,
        };

        if( SMTC_HAL_MCU_STATUS_OK != ( status = smtc_hal_mcu_spi_init( &cfg_spi, &cfg_spi_app, &spi_cfg_slot ) ) )
        {
            return status;
        }

        /* Update Callbacks for DMA */
        spi_cfg_slot->DMAcallback_tx       = cfg_app->callback_tx;
        spi_cfg_slot->DMAcallback_rx       = cfg_app->callback_rx;
        spi_cfg_slot->DMAcallback_tx_error = cfg_app->callback_error_tx;
        spi_cfg_slot->DMAcallback_rx_error = cfg_app->callback_error_rx;
        spi_cfg_slot->dma_periph_mapping   = dma_periph_configuration;

        LL_AHB1_GRP1_EnableClock( dma_periph_configuration->ahb_bus );

        NVIC_SetPriority( dma_periph_configuration->dma_channel_irq_rx, 0 );
        NVIC_EnableIRQ( dma_periph_configuration->dma_channel_irq_rx );
        NVIC_SetPriority( dma_periph_configuration->dma_channel_irq_tx, 0 );
        NVIC_EnableIRQ( dma_periph_configuration->dma_channel_irq_tx );

        LL_DMA_EnableIT_TC( dma_periph_configuration->dma, dma_periph_configuration->dma_channel_rx );
        LL_DMA_EnableIT_TE( dma_periph_configuration->dma, dma_periph_configuration->dma_channel_rx );
        LL_DMA_EnableIT_TC( dma_periph_configuration->dma, dma_periph_configuration->dma_channel_tx );
        LL_DMA_EnableIT_TE( dma_periph_configuration->dma, dma_periph_configuration->dma_channel_tx );

        *inst = spi_cfg_slot;
        return SMTC_HAL_MCU_STATUS_OK;
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }
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
    else if( inst_local->spi == SPI3 )
    {
        if( LL_SPI_DeInit( inst_local->spi ) != SUCCESS )
        {
            return SMTC_HAL_MCU_STATUS_ERROR;
        }

        LL_GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin        = LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12,
            .Mode       = LL_GPIO_MODE_ANALOG,
            .Speed      = LL_GPIO_SPEED_FREQ_LOW,
            .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
            .Pull       = LL_GPIO_PULL_NO,
            .Alternate  = LL_GPIO_AF_0,
        };

        if( LL_GPIO_Init( GPIOC, &GPIO_InitStruct ) != SUCCESS )
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

    if( data_length > 1 )
    {
        LL_SPI_SetRxFIFOThreshold( inst->spi, LL_SPI_RX_FIFO_TH_HALF );
    }
    else
    {
        LL_SPI_SetRxFIFOThreshold( inst->spi, LL_SPI_RX_FIFO_TH_QUARTER );
    }

    while( ( rem_bytes_to_send > 0 ) || ( rem_bytes_to_receive > 0 ) )
    {
        if( ( LL_SPI_IsActiveFlag_TXE( inst->spi ) == 1 ) && ( rem_bytes_to_send > 0 ) )
        {
            if( rem_bytes_to_send > 1 )  // Load a 2-byte long chunk in the TXFIFO
            {
                const uint16_t bytes_to_transmit = ( data_out == NULL )
                                                       ? 0x0000
                                                       : ( data_out[data_length - rem_bytes_to_send + 1] << 8 ) +
                                                             data_out[data_length - rem_bytes_to_send];
                LL_SPI_TransmitData16( inst->spi, bytes_to_transmit );
                rem_bytes_to_send -= 2;
            }
            else  // Load a 1-byte long chunk in the TXFIFO
            {
                const uint8_t byte_to_transmit =
                    ( data_out == NULL ) ? 0x00 : data_out[data_length - rem_bytes_to_send];
                LL_SPI_TransmitData8( inst->spi, byte_to_transmit );
                rem_bytes_to_send -= 1;
            }
        }

        if( ( LL_SPI_IsActiveFlag_RXNE( inst->spi ) == 1 ) && ( rem_bytes_to_receive > 0 ) )
        {
            if( rem_bytes_to_receive > 1 )
            {
                const uint16_t bytes_received = LL_SPI_ReceiveData16( inst->spi );

                if( data_in != NULL )
                {
                    data_in[data_length - rem_bytes_to_receive + 1] = ( uint8_t ) ( bytes_received >> 8 );
                    data_in[data_length - rem_bytes_to_receive]     = ( uint8_t ) ( bytes_received >> 0 );
                }

                rem_bytes_to_receive -= 2;

                if( rem_bytes_to_receive == 1 )
                {
                    LL_SPI_SetRxFIFOThreshold( inst->spi, LL_SPI_RX_FIFO_TH_QUARTER );
                }
            }
            else
            {
                const uint8_t byte_received = LL_SPI_ReceiveData8( inst->spi );

                if( data_in != NULL )
                {
                    data_in[data_length - rem_bytes_to_receive] = byte_received;
                }

                rem_bytes_to_receive -= 1;
            }
        }
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_spi_dma_send_receive( smtc_hal_mcu_spi_inst_t spi, const uint8_t* data_out,
                                                         uint8_t* data_in, unsigned int length )
{
    if( data_out == NULL )
    {
        // Only send null value
        LL_DMA_ConfigTransfer( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_tx,
                               LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
                                   LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_NOINCREMENT | LL_DMA_PDATAALIGN_BYTE |
                                   LL_DMA_MDATAALIGN_BYTE );
        LL_DMA_ConfigAddresses(
            spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_tx, ( uint32_t ) &dma_null_value,
            LL_SPI_DMA_GetRegAddr( spi->spi ),
            LL_DMA_GetDataTransferDirection( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_tx ) );
    }
    else
    {
        LL_DMA_ConfigTransfer( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_tx,
                               LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
                                   LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE |
                                   LL_DMA_MDATAALIGN_BYTE );
        LL_DMA_ConfigAddresses(
            spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_tx, ( uint32_t ) data_out,
            LL_SPI_DMA_GetRegAddr( spi->spi ),
            LL_DMA_GetDataTransferDirection( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_tx ) );
    }

    if( data_in == NULL )
    {
        // Received data shall be disregarded
        LL_DMA_ConfigTransfer( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_rx,
                               LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
                                   LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_NOINCREMENT | LL_DMA_PDATAALIGN_BYTE |
                                   LL_DMA_MDATAALIGN_BYTE );
        LL_DMA_ConfigAddresses(
            spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_rx, LL_SPI_DMA_GetRegAddr( spi->spi ),
            ( uint32_t ) &garbage_value,
            LL_DMA_GetDataTransferDirection( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_rx ) );
    }
    else
    {
        LL_DMA_ConfigTransfer( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_rx,
                               LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
                                   LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE |
                                   LL_DMA_MDATAALIGN_BYTE );
        LL_DMA_ConfigAddresses(
            spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_rx, LL_SPI_DMA_GetRegAddr( spi->spi ),
            ( uint32_t ) data_in,
            LL_DMA_GetDataTransferDirection( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_rx ) );
    }
    LL_DMA_SetDataLength( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_rx, length );
    LL_DMA_SetDataLength( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_tx, length );
    LL_DMA_SetPeriphRequest( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_rx,
                             spi->dma_periph_mapping->periph_request );
    LL_DMA_SetPeriphRequest( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_tx,
                             spi->dma_periph_mapping->periph_request );

    /* Configure SPI1 DMA transfer interrupts */
    /* Enable DMA RX Interrupt */
    LL_SPI_EnableDMAReq_RX( spi->spi );
    /* Enable DMA TX Interrupt */
    LL_SPI_EnableDMAReq_TX( spi->spi );
    /* Enable DMA Channel Tx */
    LL_DMA_EnableChannel( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_rx );
    LL_DMA_EnableChannel( spi->dma_periph_mapping->dma, spi->dma_periph_mapping->dma_channel_tx );
    return SMTC_HAL_MCU_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

const smtc_hal_mcu_spi_stm32l4_dma_periph_mapping_t* smtc_hal_mcu_spi_stm32l4_get_dma_peripheral_configuration(
    SPI_TypeDef* spi )
{
    for( uint32_t index = 0; index < sizeof( dma_periph_mappping ); index++ )
    {
        const smtc_hal_mcu_spi_stm32l4_dma_periph_mapping_t* local_config = &dma_periph_mappping[index];
        if( local_config->spi == spi )
        {
            return local_config;
        }
    }
    return 0;
}

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

#define CASE_IS_ACTIVE_FLAG_CHANNEL( ll_dma_name, channel_no, dma ) \
    case LL_DMA_CHANNEL_##channel_no:                               \
    {                                                               \
        return LL_DMA_##ll_dma_name##channel_no( dma );             \
        break;                                                      \
    }

uint32_t smtc_hal_mcu_spi_stm32l4_is_active_flag_tc( uint32_t channel, DMA_TypeDef* dma )
{
    switch( channel )
    {
        CASE_IS_ACTIVE_FLAG_CHANNEL( IsActiveFlag_TC, 1, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( IsActiveFlag_TC, 2, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( IsActiveFlag_TC, 3, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( IsActiveFlag_TC, 4, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( IsActiveFlag_TC, 5, dma )
    }

    return 0;
}

void smtc_hal_mcu_spi_stm32l4_clear_flag_gi( uint32_t channel, DMA_TypeDef* dma )
{
    switch( channel )
    {
        CASE_IS_ACTIVE_FLAG_CHANNEL( ClearFlag_GI, 1, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( ClearFlag_GI, 2, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( ClearFlag_GI, 3, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( ClearFlag_GI, 4, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( ClearFlag_GI, 5, dma )
    default:
        break;
    }
}

uint32_t smtc_hal_mcu_spi_stm32l4_is_active_flag_te( uint32_t channel, DMA_TypeDef* dma )
{
    switch( channel )
    {
        CASE_IS_ACTIVE_FLAG_CHANNEL( IsActiveFlag_TE, 1, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( IsActiveFlag_TE, 2, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( IsActiveFlag_TE, 3, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( IsActiveFlag_TE, 4, dma )
        CASE_IS_ACTIVE_FLAG_CHANNEL( IsActiveFlag_TE, 5, dma )
    }

    return 0;
}

void smtc_hal_mcu_spi_stm32l4_call_dma_callback_irq( DMA_TypeDef* dma, uint32_t channel )
{
    // Search for the spi instance that has interruption registered on this DMA and this channel
    for( int i = 0; i < SMTC_HAL_MCU_SPI_STM32L4_N_INSTANCES_MAX; i++ )
    {
        smtc_hal_mcu_spi_inst_t local_instance = &spi_inst_array[i];
        // Check it is enabled, and with DMA
        if( local_instance->is_cfged == true && local_instance->dma_periph_mapping != 0 )
        {
            // Check it is on this DMA peripheral
            if( local_instance->dma_periph_mapping->dma == dma )
            {
                // Check the DMA channel transmit complete flag is active
                if( smtc_hal_mcu_spi_stm32l4_is_active_flag_tc( channel, dma ) )
                {
                    // Clear the global interrupt flag
                    smtc_hal_mcu_spi_stm32l4_clear_flag_gi( channel, dma );

                    // Check for RX DMA channel
                    if( local_instance->dma_periph_mapping->dma_channel_rx == channel )
                    {
                        // If RX callback is set, call it
                        if( spi_inst_array[i].DMAcallback_rx != NULL )
                        {
                            spi_inst_array[i].DMAcallback_rx( );
                        }
                        break;
                    }
                    // Check for TX DMA channel
                    if( local_instance->dma_periph_mapping->dma_channel_tx == channel )
                    {
                        // If TX callback is set, call it
                        if( spi_inst_array[i].DMAcallback_tx != NULL )
                        {
                            spi_inst_array[i].DMAcallback_tx( );
                        }
                        break;
                    }
                }
            }

            // Check the transmit error flag is active
            if( smtc_hal_mcu_spi_stm32l4_is_active_flag_te( channel, dma ) )
            {
                // Check for RX DMA channel
                if( local_instance->dma_periph_mapping->dma_channel_rx == channel )
                {
                    // If RX error callback is set, call it
                    if( spi_inst_array[i].DMAcallback_rx_error != NULL )
                    {
                        spi_inst_array[i].DMAcallback_rx_error( );
                    }
                    break;
                }
                // Check for TX DMA channel
                if( local_instance->dma_periph_mapping->dma_channel_tx == channel )
                {
                    // If TX error callback is set, call it
                    if( spi_inst_array[i].DMAcallback_tx_error != NULL )
                    {
                        spi_inst_array[i].DMAcallback_tx_error( );
                    }
                    break;
                }
            }
        }
    }
}

#define DEFINE_DMA_CHANNEL_IRQ_HANDLER( dma, channel_no )                                   \
    void dma##_Channel##channel_no##_IRQHandler( void )                                     \
    {                                                                                       \
        smtc_hal_mcu_spi_stm32l4_call_dma_callback_irq( dma, LL_DMA_CHANNEL_##channel_no ); \
    }
DEFINE_DMA_CHANNEL_IRQ_HANDLER( DMA1, 2 )
DEFINE_DMA_CHANNEL_IRQ_HANDLER( DMA1, 3 )
DEFINE_DMA_CHANNEL_IRQ_HANDLER( DMA1, 4 )
DEFINE_DMA_CHANNEL_IRQ_HANDLER( DMA1, 5 )
DEFINE_DMA_CHANNEL_IRQ_HANDLER( DMA2, 1 )
DEFINE_DMA_CHANNEL_IRQ_HANDLER( DMA2, 2 )

/* --- EOF ------------------------------------------------------------------ */
