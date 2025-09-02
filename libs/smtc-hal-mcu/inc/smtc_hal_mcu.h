/**
 * @file smtc_hal_mcu.h
 *
 * @brief Interface to the generic functions
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

#ifndef SMTC_HAL_MCU_H
#define SMTC_HAL_MCU_H

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Perform the initialisation sequence of the MCU
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The initialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_ERROR An error occurred and the initialisation failed
 */
smtc_hal_mcu_status_t smtc_hal_mcu_init( void );

/*!
 * @brief Reset MCU
 */
void smtc_hal_mcu_reset( void );

/*!
 * @brief Panic function to trap MCU issues
 */
void smtc_hal_mcu_panic( void );

/*!
 * @brief Disable all interrupts on MCU side
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The interrupt disabling completed successfully
 * @retval SMTC_HAL_MCU_STATUS_ERROR An error occurred and the interrupt disabling failed
 */
smtc_hal_mcu_status_t smtc_hal_mcu_disable_irq( void );

/*!
 * @brief Enable all interrupts on MCU side
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The interrupt enabling completed successfully
 * @retval SMTC_HAL_MCU_STATUS_ERROR An error occurred and the interrupt enabling failed
 */
smtc_hal_mcu_status_t smtc_hal_mcu_enable_irq( void );

/*!
 * @brief Sets the MCU in sleep mode with RAM retention.
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The initialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_ERROR An error occurred and the initialisation failed
 */
smtc_hal_mcu_status_t smtc_hal_mcu_set_sleep_with_retention( void );

/*!
 * @brief Blocking wait for delay microseconds
 *
 * @param [in] microseconds Delay to wait in microseconds
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The blocking wait terminated successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS The input parameter is incorrect
 */
smtc_hal_mcu_status_t smtc_hal_mcu_wait_us( const uint32_t microseconds );

/*!
 * @brief Blocking wait for delay milliseconds
 *
 * @param [in] milliseconds Delay to wait in milliseconds
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The blocking wait terminated successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS The input parameter is incorrect
 */
smtc_hal_mcu_status_t smtc_hal_mcu_wait_ms( const uint32_t milliseconds );

/**
 * @brief Get junction temperature in tenth of °C from MCU
 *
 * @param [out] temperature Pointer to the temperature value
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The tempature has been read and returned sucessfully
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_get_junction_temperature( int16_t* temperature );

/**
 * @brief Get MCU internal voltage
 *
 * @param [out] internal_vref Pointer to the internal voltage in mV value
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The internal voltage has been read and returned sucessfully
 * @retval SMTC_HAL_MCU_STATUS_ERROR The operation failed because another error occurred
 */
smtc_hal_mcu_status_t smtc_hal_mcu_get_internal_vref( int16_t* internal_vref );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_MCU_H

/* --- EOF ------------------------------------------------------------------ */
