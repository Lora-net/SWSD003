/**
 * @file      smtc_hal_mcu_nvm.h
 *
 * @brief Interface to the Non-Volatile Memory (NVM) peripheral
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

#ifndef SMTC_HAL_MCU_NVM_H
#define SMTC_HAL_MCU_NVM_H

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
 * @brief NVM instance structure definition
 *
 * @remark smtc_hal_mcu_nvm_inst_s has to be defined in the application
 */
typedef struct smtc_hal_mcu_nvm_inst_s* smtc_hal_mcu_nvm_inst_t;

/**
 * @brief NVM configuration structure definition
 *
 * @remark smtc_hal_mcu_nvm_cfg_s has to be defined in the application
 */
typedef struct smtc_hal_mcu_nvm_cfg_s* smtc_hal_mcu_nvm_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize the NVM peripheral
 *
 * @param [in] cfg Configuration structure
 * @param [out] inst NVM instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The initialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the NVM peripheral is not initialised
 */
smtc_hal_mcu_status_t smtc_hal_mcu_nvm_init( const smtc_hal_mcu_nvm_cfg_s cfg, smtc_hal_mcu_nvm_inst_s* inst );

/**
 * @brief Deinitialize the NVM peripheral
 *
 * @param [in, out] inst NVM instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK NVM has been stopped successfully
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT Operation failed as the NVM is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR Operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_nvm_deinit( smtc_hal_mcu_nvm_inst_t* inst );

/**
 * @brief Write data to the NVM
 *
 * @param [in] inst NVM instance
 * @param [in] offset Offset in NVM
 * @param [out] buffer Buffer storing data to be written to the NVM
 * @param [in] length Length of data to be written
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Data succesfully written on the NVM
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the NVM peripheral is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_nvm_write( smtc_hal_mcu_nvm_inst_s inst, unsigned int offset, const uint8_t* buffer,
                                              unsigned int length );

/**
 * @brief Read data from the NVM
 *
 * @param [in] inst NVM instance
 * @param [in] offset Offset in NVM
 * @param [in] buffer Buffer to store data read from the NVM
 * @param [in] length Length of data to be read
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Data succesfully read from the NVM
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the NVM peripheral is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_nvm_read( smtc_hal_mcu_nvm_inst_s inst, unsigned int offset, uint8_t* buffer,
                                             unsigned int length );

/**
 * @brief Erase a section of the NVM
 *
 * @param [in] inst NVM instance
 * @param [in] offset Offset in NVM
 * @param [in] length Length of data to be erased
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Data successfully erased from the NVM
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the NVM peripheral is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_nvm_erase( smtc_hal_mcu_nvm_inst_s inst, unsigned int offset, unsigned int length );

/**
 * @brief Get the total size of the NVM
 *
 * @param [in] inst NVM instance
 * @param [out] size Size read from the NVM instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The NVM peripheral successfully produced the random data
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the NVM peripheral is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_nvm_get_total_size( smtc_hal_mcu_nvm_inst_s inst, unsigned int* size );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_MCU_NVM_H

/* --- EOF ------------------------------------------------------------------ */
