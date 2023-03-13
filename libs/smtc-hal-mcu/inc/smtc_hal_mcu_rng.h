/**
 * @file      smtc_hal_mcu_rng.h
 *
 * @brief Interface to the Random Number Generator (RNG) peripheral
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

#ifndef SMTC_HAL_MCU_RNG_H
#define SMTC_HAL_MCU_RNG_H

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
 * @brief RNG instance structure definition
 *
 * @remark smtc_hal_mcu_rng_inst_s has to be defined in the application
 */
typedef struct smtc_hal_mcu_rng_inst_s* smtc_hal_mcu_rng_inst_t;

/**
 * @brief RNG configuration structure definition
 *
 * @remark smtc_hal_mcu_rng_cfg_s has to be defined in the application
 */
typedef struct smtc_hal_mcu_rng_cfg_s* smtc_hal_mcu_rng_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize the RNG peripheral
 *
 * @param [in] cfg The configuration structure
 * @param [out] inst RNG instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The initialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the random number generator is not initialised
 */
smtc_hal_mcu_status_t smtc_hal_mcu_rng_init( const smtc_hal_mcu_rng_cfg_t cfg, smtc_hal_mcu_rng_inst_t* rng );

/**
 * @brief Deinitialize the RNG peripheral
 *
 * @param [in, out] inst RNG instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The random number generator has been stopped successfully
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the random number generator is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_rng_deinit( smtc_hal_mcu_rng_inst_t* inst );

/**
 * @brief Get random data from the RNG peripheral
 *
 * @param [in] inst RNG instance
 * @param [out] buffer Pointer to an output buffer
 * @param [in] length Number of random numbers to get
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The random number generator successfully produced the random data
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the random number generator is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_rng_get_bytes( smtc_hal_mcu_rng_inst_t inst, uint8_t* buffer, unsigned int length );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_MCU_RNG_H

/* --- EOF ------------------------------------------------------------------ */
