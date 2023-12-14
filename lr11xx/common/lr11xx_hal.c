/*!
 * \file      lr11xx_hal.c
 *
 * \brief     Implements the lr11xx radio HAL functions
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "lr11xx_hal.h"
#include "smtc_hal_mcu_spi.h"
#include "smtc_hal_mcu_gpio.h"

#include "lr11xx_hal_context.h"

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

/**
 * @brief Wait until radio busy pin returns to 0
 */
void lr11xx_hal_wait_on_busy( const void* radio );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;

    smtc_hal_mcu_gpio_set_state( lr11xx_context->reset.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    LL_mDelay( 1 );
    smtc_hal_mcu_gpio_set_state( lr11xx_context->reset.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;

    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    LL_mDelay( 1 );
    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* context )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;
    uint8_t                     command[4]     = { 0 };

    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, command, NULL, 4 );
    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );

    lr11xx_hal_wait_on_busy( lr11xx_context );

    return LR11XX_HAL_STATUS_OK;
}


lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
#if defined( USE_LR11XX_CRC_OVER_SPI )
    uint8_t cmd_crc = lr11xx_hal_compute_crc( 0xFF, command, command_length );
    cmd_crc         = lr11xx_hal_compute_crc( cmd_crc, data, data_length );
#endif

    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;

    lr11xx_hal_wait_on_busy( lr11xx_context );

    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, command, NULL, command_length );
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, data, NULL, data_length );
#if defined( USE_LR11XX_CRC_OVER_SPI )
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, &cmd_crc, NULL, 1 );
#endif
    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
#if defined( USE_LR11XX_CRC_OVER_SPI )
    const uint8_t cmd_crc = lr11xx_hal_compute_crc( 0xFF, command, command_length );
#endif

    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;
    const uint8_t               dummy_byte     = LR11XX_NOP;
    uint8_t                     dummy_byte_rx  = LR11XX_NOP;

    lr11xx_hal_wait_on_busy( lr11xx_context );

    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, command, NULL, command_length );
#if defined( USE_LR11XX_CRC_OVER_SPI )
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, &cmd_crc, NULL, 1 );
#endif
    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );

    lr11xx_hal_wait_on_busy( lr11xx_context );

    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, &dummy_byte, &dummy_byte_rx, 1 );
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, NULL, data, data_length );
#if defined( USE_LR11XX_CRC_OVER_SPI )
    uint8_t crc_rx;
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, &dummy_byte, &crc_rx, 1 );
#endif
    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );

#if defined( USE_LR11XX_CRC_OVER_SPI )
    uint8_t crc_computed = lr11xx_hal_compute_crc( 0xFF, &dummy, 1 );
    crc_computed         = lr11xx_hal_compute_crc( crc_computed, data, data_length );
    if( crc_rx != crc_computed )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }
#endif

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* radio, uint8_t* data, const uint16_t data_length )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) radio;

    lr11xx_hal_wait_on_busy( lr11xx_context );

    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, NULL, data, data_length );
#if defined( USE_LR11XX_CRC_OVER_SPI )
    uint8_t crc_rx;
    smtc_hal_mcu_spi_rw_buffer( lr11xx_context->spi.inst, NULL, &crc_rx, 1 );
#endif
    smtc_hal_mcu_gpio_set_state( lr11xx_context->nss.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );

#if defined( USE_LR11XX_CRC_OVER_SPI )
    // check crc value
    uint8_t crc_computed = lr11xx_hal_compute_crc( 0xFF, data, data_length );
    if( rx_crc != crc_computed )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }
#endif

    return LR11XX_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void lr11xx_hal_wait_on_busy( const void* radio )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) radio;
    smtc_hal_mcu_gpio_state_t   gpio_state;

    do
    {
        smtc_hal_mcu_gpio_get_state( lr11xx_context->busy.inst, &gpio_state );
    } while( gpio_state == SMTC_HAL_MCU_GPIO_STATE_HIGH );
}

/* --- EOF ------------------------------------------------------------------ */
