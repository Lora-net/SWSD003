/**
 * @file      smtc_hal_mcu_i2c.h
 *
 * @brief
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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

#ifndef SMTC_HAL_MCU_I2C_H
#define SMTC_HAL_MCU_I2C_H

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
 * @brief I2C instance structure definition
 *
 * @remark smtc_hal_mcu_i2c_inst_s has to be defined in the application
 */
typedef struct smtc_hal_mcu_i2c_inst_s* smtc_hal_mcu_i2c_inst_t;

/**
 * @brief I2C configuration structure definition
 *
 * @remark smtc_hal_mcu_i2c_cfg_s has to be defined in the application
 */
typedef struct smtc_hal_mcu_i2c_cfg_s* smtc_hal_mcu_i2c_cfg_t;

/**
 * @brief I2C address format
 */
typedef enum smtc_hal_mcu_i2c_address_format_e
{
    SMTC_HAL_MCU_I2C_ADDRESS_FORMAT_7_BITS,
    SMTC_HAL_MCU_I2C_ADDRESS_FORMAT_10_BITS,
} smtc_hal_mcu_i2c_address_format_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize I2C peripheral
 *
 * @param [in] cfg I2C instance configuration
 * @param [out] inst Pointer to a I2C instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The initialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS \p cfg has incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the I2C is not initialised
 */
smtc_hal_mcu_status_t smtc_hal_mcu_i2c_init( smtc_hal_mcu_i2c_cfg_t cfg, smtc_hal_mcu_i2c_inst_t* inst );

/**
 * @brief Deinitialize I2C peripheral
 *
 * @param [in,out] inst Pointer to a I2C instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The I2C has been deinit successfully
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the \p inst is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_i2c_deinit( smtc_hal_mcu_i2c_inst_t* inst );

/**
 * @brief Send a buffer of bytes over a I2C peripheral
 *
 * @remark It is a blocking operation until all bytes are sent
 *
 * @param [in] inst I2C instance
 * @param [in] address_format Device slave address format
 * @param [in] device_addr Device slave address
 * @param [in] data_out Buffer containing bytes to be sent
 * @param [in] data_length Number of bytes to be exchanged over I2C peripheral
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The I2C write operation terminated successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS The operation failed because one parameter is incorrect
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the \p i2c is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_i2c_write_buffer( smtc_hal_mcu_i2c_inst_t           inst,
                                                     smtc_hal_mcu_i2c_address_format_t address_format,
                                                     const uint16_t device_addr, const uint8_t* data_out,
                                                     const uint16_t data_length );

/**
 * @brief Receive a buffer of bytes over a I2C peripheral
 *
 * @remark It is a blocking operation until all bytes are received
 *
 * @param [in] inst I2C instance
 * @param [in] address_format Device slave address format
 * @param [in] device_addr Device slave address
 * @param [out] data_in Buffer to store bytes received
 * @param [in] data_length Number of bytes to be exchanged over I2C peripheral
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The I2C read operation terminated successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS The operation failed because one parameter is incorrect
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the \p i2c is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_i2c_read_buffer( smtc_hal_mcu_i2c_inst_t           inst,
                                                    smtc_hal_mcu_i2c_address_format_t address_format,
                                                    const uint16_t device_addr, uint8_t* data_in,
                                                    uint16_t const data_length );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_MCU_I2C_H

/* --- EOF ------------------------------------------------------------------ */
