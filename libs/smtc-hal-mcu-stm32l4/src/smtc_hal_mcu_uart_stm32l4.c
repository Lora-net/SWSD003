/*!
 * @file      smtc_hal_mcu_uart_stm32l4.c
 *
 * @brief      Implementation of UART module on top of STM32L4 Low Level drivers
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
#include "smtc_hal_mcu_uart.h"
#include "smtc_hal_mcu_uart_stm32l4.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_dma.h"
#include <stddef.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Maximum number of UART instances
 */
#ifndef SMTC_HAL_MCU_UART_STM32L4_N_INSTANCES_MAX
#define SMTC_HAL_MCU_UART_STM32L4_N_INSTANCES_MAX 1
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Structure defining a UART instance
 */
struct smtc_hal_mcu_uart_inst_s
{
    bool           is_cfged;
    USART_TypeDef* usart;
    DMA_TypeDef*   dma;
    void ( *callback_rx )( uint8_t data );
    void ( *DMAcallback_rx )( void );
    void ( *DMAcallback_tx )( void );
    void ( *DMAcallback_rx_error )( void );
    void ( *DMAcallback_tx_error )( void );
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Array to store the UART instances
 */
static struct smtc_hal_mcu_uart_inst_s uart_inst_array[SMTC_HAL_MCU_UART_STM32L4_N_INSTANCES_MAX];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Get a pointer to the first free slot
 *
 * @retval Pointer to the first free slot - NULL if there is no space left
 */
static struct smtc_hal_mcu_uart_inst_s* smtc_hal_mcu_uart_stm32l4_get_free_slot( void );

/**
 * @brief Check if the instance given as parameter is genuine
 *
 * @param [in] inst UART instance
 *
 * @retval true Instance is genuine
 * @retval false Instance is not genuine
 */
static bool smtc_hal_mcu_uart_stm32l4_is_real_inst( smtc_hal_mcu_uart_inst_t inst );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_hal_mcu_status_t smtc_hal_mcu_uart_init( const smtc_hal_mcu_uart_cfg_t      cfg,
                                              const smtc_hal_mcu_uart_cfg_app_t* cfg_app,
                                              smtc_hal_mcu_uart_inst_t*          inst )
{
    struct smtc_hal_mcu_uart_inst_s* uart_cfg_slot = smtc_hal_mcu_uart_stm32l4_get_free_slot( );

    if( uart_cfg_slot == NULL )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    if( cfg->usart == USART2 )
    {
        uart_cfg_slot->usart = cfg->usart;

        LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_USART2 );

        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOA );

        LL_GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin        = LL_GPIO_PIN_2 | LL_GPIO_PIN_3,
            .Mode       = LL_GPIO_MODE_ALTERNATE,
            .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
            .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
            .Pull       = LL_GPIO_PULL_NO,
            .Alternate  = LL_GPIO_AF_7,
        };

        LL_GPIO_Init( GPIOA, &GPIO_InitStruct );

        NVIC_SetPriority( USART2_IRQn, 0 );
        NVIC_EnableIRQ( USART2_IRQn );
    }

    LL_USART_InitTypeDef USART_InitStruct = {
        .BaudRate            = cfg_app->baudrate,
        .DataWidth           = LL_USART_DATAWIDTH_8B,
        .StopBits            = LL_USART_STOPBITS_1,
        .Parity              = LL_USART_PARITY_NONE,
        .TransferDirection   = LL_USART_DIRECTION_TX_RX,
        .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
        .OverSampling        = LL_USART_OVERSAMPLING_16,
    };

    uart_cfg_slot->callback_rx = cfg_app->callback_rx;

    if( LL_USART_Init( uart_cfg_slot->usart, &USART_InitStruct ) == ERROR )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    LL_USART_ConfigAsyncMode( uart_cfg_slot->usart );

    LL_USART_Enable( uart_cfg_slot->usart );
    while( LL_USART_IsEnabled( uart_cfg_slot->usart ) == 0 )
    {
    }

    LL_USART_RequestRxDataFlush( uart_cfg_slot->usart );
    LL_USART_RequestTxDataFlush( uart_cfg_slot->usart );

    if( cfg_app->callback_rx != NULL )
    {
        LL_USART_EnableIT_RXNE( uart_cfg_slot->usart );
    }

    uart_cfg_slot->is_cfged = true;

    *inst = uart_cfg_slot;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_uart_dma_init( const smtc_hal_mcu_uart_dma_cfg_t      cfg,
                                                  const smtc_hal_mcu_uart_dma_cfg_app_t* cfg_app,
                                                  smtc_hal_mcu_uart_inst_t*              inst )
{
    /* DMA1 used for USART2 Transmission and Reception
     */
    if( cfg->dma == DMA1 )
    {
        smtc_hal_mcu_status_t    status;
        smtc_hal_mcu_uart_inst_t uart_cfg_slot = NULL;

        /* Setting up UART for DMA usage */
        struct smtc_hal_mcu_uart_cfg_s cfg_uart = {
            .usart = USART2,
        };
        const struct smtc_hal_mcu_uart_cfg_app_s cfg_uart_app = {
            .baudrate    = cfg_app->baudrate,
            .callback_rx = NULL,
        };

        if( SMTC_HAL_MCU_STATUS_OK != ( status = smtc_hal_mcu_uart_init( &cfg_uart, &cfg_uart_app, &uart_cfg_slot ) ) )
        {
            return status;
        }

        /* Update Callbacks for DMA */
        uart_cfg_slot->DMAcallback_rx       = cfg_app->callback_rx;
        uart_cfg_slot->DMAcallback_tx       = cfg_app->callback_tx;
        uart_cfg_slot->DMAcallback_rx_error = cfg_app->callback_error_rx;
        uart_cfg_slot->DMAcallback_tx_error = cfg_app->callback_error_tx;
        uart_cfg_slot->dma                  = cfg->dma;

        /* (1) Enable the clock of DMA1 */
        LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_DMA1 );

        /* (2) Configure NVIC for DMA transfer complete/error interrupts */
        NVIC_SetPriority( DMA1_Channel7_IRQn, 0 );
        NVIC_EnableIRQ( DMA1_Channel7_IRQn );
        NVIC_SetPriority( DMA1_Channel6_IRQn, 0 );
        NVIC_EnableIRQ( DMA1_Channel6_IRQn );

        /* (3) Configure the DMA functional parameters for transmission */
        LL_DMA_ConfigTransfer( DMA1, LL_DMA_CHANNEL_7,
                               LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
                                   LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE |
                                   LL_DMA_MDATAALIGN_BYTE );

        /* (4) Configure the DMA functional parameters for reception */
        LL_DMA_ConfigTransfer( DMA1, LL_DMA_CHANNEL_6,
                               LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_HIGH | LL_DMA_MODE_NORMAL |
                                   LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT | LL_DMA_PDATAALIGN_BYTE |
                                   LL_DMA_MDATAALIGN_BYTE );

        /* (5) Enable DMA transfer complete/error interrupts  */
        LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_7 );
        LL_DMA_EnableIT_TE( DMA1, LL_DMA_CHANNEL_7 );
        LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_6 );
        LL_DMA_EnableIT_TE( DMA1, LL_DMA_CHANNEL_6 );

        *inst = uart_cfg_slot;
        return SMTC_HAL_MCU_STATUS_OK;
    }
    return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
}

smtc_hal_mcu_status_t smtc_hal_mcu_uart_dma_send( smtc_hal_mcu_uart_inst_t inst, const uint8_t* aTxBuffer,
                                                  unsigned int data_length )
{
    LL_DMA_ConfigAddresses( inst->dma, LL_DMA_CHANNEL_7, ( uint32_t ) aTxBuffer,
                            LL_USART_DMA_GetRegAddr( inst->usart, LL_USART_DMA_REG_DATA_TRANSMIT ),
                            LL_DMA_GetDataTransferDirection( DMA1, LL_DMA_CHANNEL_7 ) );
    LL_DMA_SetDataLength( inst->dma, LL_DMA_CHANNEL_7, data_length );
    LL_DMA_SetPeriphRequest( inst->dma, LL_DMA_CHANNEL_7, LL_DMA_REQUEST_2 );
    /* Enable DMA TX Interrupt */
    LL_USART_EnableDMAReq_TX( inst->usart );
    /* Enable DMA Channel Tx */
    LL_DMA_EnableChannel( inst->dma, LL_DMA_CHANNEL_7 );
    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_uart_dma_receive( smtc_hal_mcu_uart_inst_t inst, uint8_t* aRxBuffer,
                                                     unsigned int data_length )
{
    LL_DMA_ConfigAddresses( inst->dma, LL_DMA_CHANNEL_6,
                            LL_USART_DMA_GetRegAddr( inst->usart, LL_USART_DMA_REG_DATA_RECEIVE ),
                            ( uint32_t ) aRxBuffer, LL_DMA_GetDataTransferDirection( inst->dma, LL_DMA_CHANNEL_6 ) );
    LL_DMA_SetDataLength( inst->dma, LL_DMA_CHANNEL_6, data_length );
    LL_DMA_SetPeriphRequest( inst->dma, LL_DMA_CHANNEL_6, LL_DMA_REQUEST_2 );
    /* Enable DMA RX Interrupt */
    LL_USART_EnableDMAReq_RX( inst->usart );
    /* Enable DMA Channel Rx */
    LL_DMA_EnableChannel( inst->dma, LL_DMA_CHANNEL_6 );
    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_uart_deinit( smtc_hal_mcu_uart_inst_t* inst )
{
    smtc_hal_mcu_uart_inst_t inst_local = *inst;

    if( smtc_hal_mcu_uart_stm32l4_is_real_inst( inst_local ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst_local->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    LL_USART_Disable( inst_local->usart );
    while( LL_USART_IsEnabled( inst_local->usart ) != 0 )
    {
    }

    if( LL_USART_DeInit( inst_local->usart ) == ERROR )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    if( inst_local->usart == USART2 )
    {
        LL_APB1_GRP1_DisableClock( LL_APB1_GRP1_PERIPH_USART2 );

        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOA );

        LL_GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin        = LL_GPIO_PIN_2 | LL_GPIO_PIN_3,
            .Mode       = LL_GPIO_MODE_ANALOG,
            .Speed      = LL_GPIO_SPEED_FREQ_LOW,
            .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
            .Pull       = LL_GPIO_PULL_NO,
            .Alternate  = LL_GPIO_AF_7,
        };

        LL_GPIO_Init( GPIOA, &GPIO_InitStruct );
    }

    inst_local->is_cfged = false;

    *inst = NULL;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_uart_send( smtc_hal_mcu_uart_inst_t inst, const uint8_t* buffer,
                                              unsigned int length )
{
    unsigned int data_remaining = length;

    if( length == 0 )
    {
        return SMTC_HAL_MCU_STATUS_OK;
    }

    while( data_remaining > 0 )
    {
        while( !LL_USART_IsActiveFlag_TXE( inst->usart ) )
        {
        }

        LL_USART_TransmitData8( inst->usart, buffer[length - data_remaining--] );
    }

    while( !LL_USART_IsActiveFlag_TC( inst->usart ) )
    {
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_uart_receive( smtc_hal_mcu_uart_inst_t inst, uint8_t* buffer, unsigned int length )
{
    for( unsigned int i = 0; i < length; i++ )
    {
        while( !LL_USART_IsActiveFlag_RXNE( inst->usart ) );
        buffer[i] = LL_USART_ReceiveData8( inst->usart );
    };

    return SMTC_HAL_MCU_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static struct smtc_hal_mcu_uart_inst_s* smtc_hal_mcu_uart_stm32l4_get_free_slot( void )
{
    for( int i = 0; i < SMTC_HAL_MCU_UART_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( uart_inst_array[i].is_cfged == false )
        {
            return &uart_inst_array[i];
        }
    }

    return NULL;
}

static bool smtc_hal_mcu_uart_stm32l4_is_real_inst( smtc_hal_mcu_uart_inst_t inst )
{
    for( int i = 0; i < SMTC_HAL_MCU_UART_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( inst == &uart_inst_array[i] )
        {
            return true;
        }
    }

    return false;
}

void USART2_IRQHandler( void )
{
    /* Check RXNE flag value in ISR register */
    if( LL_USART_IsActiveFlag_RXNE( USART2 ) && LL_USART_IsEnabledIT_RXNE( USART2 ) )
    {
        /* RXNE flag will be cleared by reading of RDR register (done in call) */
        const uint8_t data = LL_USART_ReceiveData8( USART2 );

        for( int i = 0; i < SMTC_HAL_MCU_UART_STM32L4_N_INSTANCES_MAX; i++ )
        {
            if( uart_inst_array[i].usart == USART2 )
            {
                if( uart_inst_array[i].callback_rx != NULL )
                {
                    uart_inst_array[i].callback_rx( data );
                }

                break;
            }
        }
    }
}
/**
 * @brief  This function handles DMA1 interrupt request.
 * @param  None
 * @retval None
 */
void DMA1_Channel7_IRQHandler( void )
{
    if( LL_DMA_IsActiveFlag_TC7( DMA1 ) )
    {
        LL_DMA_ClearFlag_GI7( DMA1 );
        /* Call function Transmission complete Callback */
        for( int i = 0; i < SMTC_HAL_MCU_UART_STM32L4_N_INSTANCES_MAX; i++ )
        {
            if( uart_inst_array[i].dma == DMA1 )
            {
                if( uart_inst_array[i].DMAcallback_tx != NULL )
                {
                    uart_inst_array[i].DMAcallback_tx( );
                }
                break;
            }
        }
        /* Disable DMA1 Tx Channel */
        LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_7 );
    }
    else if( LL_DMA_IsActiveFlag_TE7( DMA1 ) )
    {
        for( int i = 0; i < SMTC_HAL_MCU_UART_STM32L4_N_INSTANCES_MAX; i++ )
        {
            if( uart_inst_array[i].dma == DMA1 )
            {
                if( uart_inst_array[i].DMAcallback_tx_error != NULL )
                {
                    uart_inst_array[i].DMAcallback_tx_error( );
                }
                break;
            }
        }
    }
}
/**
 * @brief  This function handles DMA1 interrupt request.
 * @param  None
 * @retval None
 */
void DMA1_Channel6_IRQHandler( void )
{
    if( LL_DMA_IsActiveFlag_TC6( DMA1 ) )
    {
        LL_DMA_ClearFlag_GI6( DMA1 );
        // vprint( "Received through DMA\n" );
        LL_USART_ClearFlag_ORE( USART2 );
        /* Disable DMA1 Rx Channel */
        LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_6 );
        /* Call function Reception complete Callback */
        for( int i = 0; i < SMTC_HAL_MCU_UART_STM32L4_N_INSTANCES_MAX; i++ )
        {
            if( uart_inst_array[i].dma == DMA1 )
            {
                if( uart_inst_array[i].DMAcallback_rx != NULL )
                {
                    uart_inst_array[i].DMAcallback_rx( );
                }
                break;
            }
        }
    }
    else if( LL_DMA_IsActiveFlag_TE6( DMA1 ) )
    {
        for( int i = 0; i < SMTC_HAL_MCU_UART_STM32L4_N_INSTANCES_MAX; i++ )
        {
            if( uart_inst_array[i].dma == DMA1 )
            {
                if( uart_inst_array[i].DMAcallback_rx_error != NULL )
                {
                    uart_inst_array[i].DMAcallback_rx_error( );
                }
                break;
            }
        }
    }
}
/* --- EOF ------------------------------------------------------------------ */
