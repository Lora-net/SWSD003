/*!
 * @file      apps_common.h
 *
 * @brief     Common functions shared by the examples
 *
 * @copyright
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

#ifndef APPS_COMMON_H
#define APPS_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "apps_configuration.h"
#include "sx126x_hal_context.h"
#include "sx126x.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#define RX_CONTINUOUS 0xFFFFFF

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Interface to initialise and return the sx126x_hal_context_t object corresponding to the board
 *
 * @warning It is up to the caller to ensure the context pointer stays valid for the life duration of the
 * sx126x_hal_context_t created by this call
 *
 * @returns Pointer to the sx126x_hal_context_t object of the board
 */
sx126x_hal_context_t* apps_common_sx126x_get_context( );

/*!
 * @brief Initialize the system configuration of the transceiver
 */
void apps_common_shield_init( void );

/*!
 * @brief Initialize the system configuration of the transceiver
 *
 * @param [in] context  Pointer to the radio context
 */
void apps_common_sx126x_init( const sx126x_hal_context_t* context );

/*!
 * @brief Initialize the radio configuration of the transceiver
 *
 * @param [in] context  Pointer to the radio context
 */
void apps_common_sx126x_radio_init( const void* context );

/*!
 * @brief Initialize the radio configuration of the transceiver for dbpsk only
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] context  Length of the payload
 */
void apps_common_sx126x_radio_dbpsk_init( const void* context, const uint8_t payload_len );

/*!
 * @brief Interface to read bytes from rx buffer
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] buffer Pointer to a byte array to be filled with content from memory. Its size must be enough to contain
 * at least length bytes
 * @param [in] size Number of bytes to read from memory
 * @param [in] max_size Size of the buffer passed as parameter for reception
 */
void apps_common_sx126x_receive( const void* context, uint8_t* buffer, uint8_t* size, uint8_t max_size );

/*!
 * @brief Interface to sx126x interrupt processing routine
 *
 * @warning This function must be called from the main loop of project to dispense all the sx126x interrupt routine
 *
 * @param [in] context  Pointer to the radio context
 */
void apps_common_sx126x_irq_process( const void* context );

/*!
 * @brief Prints all RF parameters
 */
void apps_common_sx126x_print_config( void );

/**
 * @brief Print version information
 */
void apps_common_sx126x_print_version_info( void );

void apps_common_sx126x_handle_pre_tx( void );
void apps_common_sx126x_handle_post_tx( void );
void apps_common_sx126x_handle_pre_rx( void );
void apps_common_sx126x_handle_post_rx( void );

/*!
 * @brief Computes time on air, packet type agnostic
 */
uint32_t get_time_on_air_in_ms( void );

/*!
 * @brief A function to get the value for low data rate optimization setting
 *
 * @param [in] sf  LoRa Spreading Factor
 * @param [in] bw  LoRa Bandwidth
 */
inline static uint8_t apps_common_compute_lora_ldro( const sx126x_lora_sf_t sf, const sx126x_lora_bw_t bw )
{
    switch( bw )
    {
    case SX126X_LORA_BW_500:
        return 0;

    case SX126X_LORA_BW_250:
        if( sf == SX126X_LORA_SF12 )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_125:
        if( ( sf == SX126X_LORA_SF12 ) || ( sf == SX126X_LORA_SF11 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_062:
        if( ( sf == SX126X_LORA_SF12 ) || ( sf == SX126X_LORA_SF11 ) || ( sf == SX126X_LORA_SF10 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_041:
        if( ( sf == SX126X_LORA_SF12 ) || ( sf == SX126X_LORA_SF11 ) || ( sf == SX126X_LORA_SF10 ) ||
            ( sf == SX126X_LORA_SF9 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_031:
    case SX126X_LORA_BW_020:
    case SX126X_LORA_BW_015:
    case SX126X_LORA_BW_010:
    case SX126X_LORA_BW_007:
        return 1;

    default:
        return 0;
    }
}

#ifdef __cplusplus
}
#endif

#endif  // APPS_COMMON_H

/* --- EOF ------------------------------------------------------------------ */
