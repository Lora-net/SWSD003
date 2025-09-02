/**
 * @file      smtc_hal_mcu_timing.h
 *
 * @brief Interface to the timing utility peripheral
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

#ifndef SMTC_HAL_MCU_TIMING_H
#define SMTC_HAL_MCU_TIMING_H

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
 * @brief Implementation-level timing instance structure definition
 *
 * @remark smtc_hal_mcu_timing_inst_s has to be defined in the application
 */
typedef struct smtc_hal_mcu_timing_inst_s* smtc_hal_mcu_timing_inst_t;

/**
 * @brief Implementation-level timing configuration structure definition
 *
 * @remark smtc_hal_mcu_timing_cfg_s has to be defined in the application
 */
typedef struct smtc_hal_mcu_timing_cfg_s* smtc_hal_mcu_timing_cfg_t;

/**
 * @brief Enum type to specify order of size while getting time
 *
 */
typedef enum
{
    SMTC_HAL_MCU_TIME_UNIT_S,
    SMTC_HAL_MCU_TIME_UNIT_MS,
    SMTC_HAL_MCU_TIME_UNIT_100US
} smtc_hal_mcu_timing_unit_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize the timing peripheral
 *
 * @param [in] cfg Implementation-level configuration structure
 * @param [out] inst Timing instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Initialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the timer is not initialised
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timing_init( const smtc_hal_mcu_timing_cfg_t cfg, smtc_hal_mcu_timing_inst_t* inst );

/**
 * @brief Deinitialize the timing peripheral
 *
 * @param [in, out] inst Timing instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The timer has been deinit successfully
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timing_deinit( smtc_hal_mcu_timing_inst_t* inst );

/**
 * @brief Start the timing information gathering
 *
 * @param [in] inst Timing instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The timer successfully started
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timing_start( smtc_hal_mcu_timing_inst_t inst );

/**
 * @brief Stop the timing information gathering
 *
 * @param [in] inst Timing instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The timer successfully stopped
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timing_stop( smtc_hal_mcu_timing_inst_t inst );

/**
 * @brief Get the timing counter
 *
 * @param [in] inst Timing instance
 * @param [in] time_unit Order of size for timing counter
 * @param [out] time Timing counter
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Current time successfully reported
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timing_get_time( smtc_hal_mcu_timing_inst_t inst,
                                                    smtc_hal_mcu_timing_unit_t time_unit, uint32_t* time );

/**
 * @brief Get the time elapsed between a call to @ref smtc_hal_mcu_timing_start and @ref smtc_hal_mcu_timing_stop
 *
 * @param [in] inst Timing instance
 * @param [out] time_unit Order of size for timing counter
 * @param [out] time Time elapsed
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Elapsed time successfully reported
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timing_get_elapsed_time( smtc_hal_mcu_timing_inst_t inst,
                                                            smtc_hal_mcu_timing_unit_t time_unit, uint32_t* time );

/**
 * @brief Waits a number of milliseconds
 *
 * @remark Blocking call that produces a small delay so it should only be used for short delays.
 *
 * @param [in] inst Timing instance
 * @param [in] time_unit Order of size for timing counter
 * @param [in] time Time to wait
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Successfully waited the given duration
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the timer is not initialised
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_timing_wait( smtc_hal_mcu_timing_inst_t inst, smtc_hal_mcu_timing_unit_t time_unit,
                                                uint32_t time );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_MCU_TIMER_H

/* --- EOF ------------------------------------------------------------------ */
