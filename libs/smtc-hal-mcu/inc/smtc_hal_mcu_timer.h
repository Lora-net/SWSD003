/**
 * @file      smtc_hal_mcu_timer.h
 *
 * @brief Interface to the timer peripheral
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

#ifndef SMTC_HAL_MCU_TIMER_H
#define SMTC_HAL_MCU_TIMER_H

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
 * @brief Implementation-level timer instance structure definition
 *
 * @remark smtc_hal_mcu_timer_inst_s has to be defined in the application
 */
typedef struct smtc_hal_mcu_timer_inst_s* smtc_hal_mcu_timer_inst_t;

/**
 * @brief Implementation-level timer configuration structure definition
 *
 * @remark smtc_hal_mcu_timer_cfg_s has to be defined in the application
 */
typedef struct smtc_hal_mcu_timer_cfg_s* smtc_hal_mcu_timer_cfg_t;

/**
 * @brief Application-level timer configuration structure
 */
typedef struct smtc_hal_mcu_timer_cfg_app_s
{
    void ( *expiry_func )( void );
} smtc_hal_mcu_timer_cfg_app_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize the timer peripheral
 *
 * @param [in] cfg Implementation-level configuration structure
 * @param [in] cfg_app Application-level configuration structure
 * @param [out] inst Timer instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Initialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the timer is not initialised
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timer_init( const smtc_hal_mcu_timer_cfg_t      cfg,
                                               const smtc_hal_mcu_timer_cfg_app_t* cfg_app,
                                               smtc_hal_mcu_timer_inst_t*          inst );

/**
 * @brief Deinitialize the timer peripheral
 *
 * @param [in, out] inst Timer instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The timer has been stopped successfully
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timer_deinit( smtc_hal_mcu_timer_inst_t* inst );

/**
 * @brief Start the timer
 *
 * @remark Attempting to start a timer that is already running is allowed. The timer status is reset and start counting
 * using the new parameters.
 *
 * @param [in] inst Timer instance
 * @param [in] timeout_in_ms Timeout duration, in milliseconds
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The timer successfully started
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timer_start( smtc_hal_mcu_timer_inst_t inst, uint32_t timeout_in_ms );

/**
 * @brief Stop the timer
 *
 * @param [in] inst Timer instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The timer successfully stopped
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timer_stop( smtc_hal_mcu_timer_inst_t inst );

/**
 * @brief Get time remaining before a timer next expires, in milliseconds
 *
 * @remark If the timer is not running, it returns zero.
 *
 * @param [in] inst Timer instance
 * @param [out] time_in_ms Remaining time
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Remaining time succesfully reported
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timer_get_remaining_time( smtc_hal_mcu_timer_inst_t inst, uint32_t* time_in_ms );

/**
 * @brief Get the maximum value of the timer
 *
 * @param [in] inst Timer instance
 * @param [out] value Maximum value of the timer
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Maximum value of the timer succesfully read
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timer_get_max_value( smtc_hal_mcu_timer_inst_t inst, uint32_t* value_in_ms );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_MCU_TIMER_H

/* --- EOF ------------------------------------------------------------------ */
