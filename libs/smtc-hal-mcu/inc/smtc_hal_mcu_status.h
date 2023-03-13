/*!
 * @file      smtc_hal_mcu_status.h
 *
 * @brief     Status type definition for SMTC HAL MCU
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

#ifndef SMTC_HAL_MCU_STATUS_H
#define SMTC_HAL_MCU_STATUS_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

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
 * @brief SMTC HAL MCU status
 *
 * If an API returns with a status SMTC_HAL_MCU_STATUS_OK, it means the related action terminated successfully,
 * respecting all caller provided parameters.
 * If an API returns with a status different from SMTC_HAL_MCU_STATUS_OK, then it means that at least the corresponding
 * error occurred. But the same call may suffer additional error. For instance in the case where an action is called
 * with incorrect parameter, on a resource that has not been initialised, the API will return status
 * SMTC_HAL_MCU_STATUS_BAD_PARAMETERS or SMTC_HAL_MCU_STATUS_NOT_INIT (but not both at the same time).
 * The first erroneous status to be returned is implementation dependant.
 */
typedef enum smtc_hal_mcu_status_e
{
    SMTC_HAL_MCU_STATUS_OK,  //!< The related action has terminated successfully respecting caller provided parameters
    SMTC_HAL_MCU_STATUS_ERROR,  //!< The related action is incomplete, failed, or cannot respect caller provided
                                //!< parameters
    SMTC_HAL_MCU_STATUS_BAD_PARAMETERS,  //!< At least one of the provided argument is incorrect
    SMTC_HAL_MCU_STATUS_NOT_INIT,  //!< The related action is expected to act on an initialized resource, but this is
                                   //!< not initialized
} smtc_hal_mcu_status_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#endif  // SMTC_HAL_MCU_STATUS_H

/* --- EOF ------------------------------------------------------------------ */
