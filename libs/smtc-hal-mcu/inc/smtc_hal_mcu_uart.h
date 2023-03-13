/**
 * @file      smtc_hal_mcu_uart.h
 *
 * @brief Interface to the Universal Asynchronous Receiver Transmitter (UART)
 * peripheral
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

#ifndef SMTC_HAL_MCU_UART_H
#define SMTC_HAL_MCU_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "smtc_hal_mcu_status.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief UART instance structure definition
 *
 * @remark smtc_hal_mcu_uart_inst_s has to be defined in the implementation
 */
typedef struct smtc_hal_mcu_uart_inst_s* smtc_hal_mcu_uart_inst_t;

/**
 * @brief UART configuration structure definition
 *
 * @remark smtc_hal_mcu_uart_cfg_s has to be defined in the implementation
 */
typedef struct smtc_hal_mcu_uart_cfg_s* smtc_hal_mcu_uart_cfg_t;

/**
 * @brief UART application configuration structure
 */
typedef struct smtc_hal_mcu_uart_cfg_app_s
{
    uint32_t baudrate;
    void ( *callback_rx )( uint8_t data );
} smtc_hal_mcu_uart_cfg_app_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize a UART peripheral
 *
 * @param [in] cfg Implementation-level configuration structure
 * @param [in] cfg_app Application-level configuration structure
 * @param [out] inst Pointer to a UART instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The initialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the @p uart is not initialised
 */
smtc_hal_mcu_status_t smtc_hal_mcu_uart_init( const smtc_hal_mcu_uart_cfg_t      cfg,
                                              const smtc_hal_mcu_uart_cfg_app_t* cfg_app,
                                              smtc_hal_mcu_uart_inst_t*          inst );

/**
 * @brief Deinitialize the UART peripheral
 *
 * @param [in, out] inst Pointer to a UART instance - if call return with SMTC_HAL_MCU_STATUS_OK, @p inst is set to NULL
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The UART has been deinit successfully
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the @p uart is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_uart_deinit( smtc_hal_mcu_uart_inst_t* inst );

/**
 * @brief Send bytes over a UART peripheral
 *
 * @remark It is a blocking operation until all bytes are sent
 *
 * @param [in] uart UART instance
 * @param [in] buffer Pointer to input buffer. It is up to the caller to ensure @p buffer is at least @p length byte
 * long
 * @param [in] length Number of bytes to send
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The UART send operation terminated successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS The operation failed because at least one parameter is incorrect
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the @p uart is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_uart_send( smtc_hal_mcu_uart_inst_t uart, const uint8_t* buffer,
                                              unsigned int length );

/**
 * @brief Receive bytes over a UART peripheral
 *
 * @remark It is a blocking operation until all bytes are received
 *
 * @param [in] uart UART instance
 * @param [out] buffer Pointer to a buffer to be filled with received bytes. It is up to the caller to ensure this
 * buffer is at least @p length byte long
 * @param [in] length Number of bytes to receive
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The UART receive operation terminated successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS The operation failed because at least one parameter is incorrect
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the @p uart is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_uart_receive( smtc_hal_mcu_uart_inst_t uart, uint8_t* buffer, unsigned int length );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_MCU_UART_H

/* --- EOF ------------------------------------------------------------------ */
