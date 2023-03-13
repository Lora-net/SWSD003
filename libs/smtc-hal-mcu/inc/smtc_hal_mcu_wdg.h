/**
 * @file smtc_hal_mcu_wdg.h
 *
 * @brief Interface to the watchdog timer
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

#ifndef SMTC_HAL_MCU_WDG_H
#define SMTC_HAL_MCU_WDG_H

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
 * @brief Watchdog instance structure definition
 *
 * @remark smtc_hal_mcu_spi_inst_s has to be defined in the implementation
 */
typedef struct smtc_hal_mcu_wdg_inst_s* smtc_hal_mcu_wdg_inst_t;

/**
 * @brief Implementation-level watchdog configuration structure definition
 *
 * @remark smtc_hal_mcu_spi_cfg_s has to be defined in the implementation
 */
typedef struct smtc_hal_mcu_wdg_cfg_s* smtc_hal_mcu_wdg_cfg_t;

/**
 * @brief Application-level watchdog configuration structure definition
 */
typedef struct smtc_hal_mcu_wdg_cfg_app_s
{
    uint32_t timeout_ms;
} smtc_hal_mcu_wdg_cfg_app_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize and start the watchdog timer with the given configuration
 *
 * @param [in] cfg Implementation-level configuration structure
 * @param [in] cfg_app Application-level configuration structure
 * @param [out] inst Pointer to a watchdog instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The initialisation completed successfully and the watchdog is started
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS \p timeout_ms has incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the watchdog is not initialised or not started
 */
smtc_hal_mcu_status_t smtc_hal_mcu_wdg_init_and_start( smtc_hal_mcu_wdg_cfg_t            cfg,
                                                       const smtc_hal_mcu_wdg_cfg_app_t* cfg_app,
                                                       smtc_hal_mcu_wdg_inst_t*          inst );

/**
 * @brief Stop the watchdog timer
 *
 * @param [in] inst Watchdog instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The watchdog has been stopped successfully
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the watchdog is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_wdg_stop( smtc_hal_mcu_wdg_inst_t inst );

/**
 * @brief Reload the watchdog timer
 *
 * @param [in] inst Watchdog instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The watchdog has been reloaded successfully
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT The operation failed as the watchdog is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_wdg_reload( smtc_hal_mcu_wdg_inst_t inst );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_MCU_WDG_H

/* --- EOF ------------------------------------------------------------------ */
