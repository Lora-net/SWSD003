/*!
 * @file      smtc_hal_arduino_mapping.h
 *
 * @brief     Arduino mapping definition
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

#ifndef SMTC_HAL_ARDUINO_MAPPING_H
#define SMTC_HAL_ARDUINO_MAPPING_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_hal_mcu_gpio.h"

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

typedef enum
{
    ARDUINO_CONNECTOR_D0 = 0,
    ARDUINO_CONNECTOR_D1,
    ARDUINO_CONNECTOR_D2,
    ARDUINO_CONNECTOR_D3,
    ARDUINO_CONNECTOR_D4,
    ARDUINO_CONNECTOR_D5,
    ARDUINO_CONNECTOR_D6,
    ARDUINO_CONNECTOR_D7,
    ARDUINO_CONNECTOR_D8,
    ARDUINO_CONNECTOR_D9,
    ARDUINO_CONNECTOR_D10,
    ARDUINO_CONNECTOR_D11,
    ARDUINO_CONNECTOR_D12,
    ARDUINO_CONNECTOR_D13,
    ARDUINO_CONNECTOR_D14,
    ARDUINO_CONNECTOR_D15,
    ARDUINO_CONNECTOR_A0,
    ARDUINO_CONNECTOR_A1,
    ARDUINO_CONNECTOR_A2,
    ARDUINO_CONNECTOR_A3,
    ARDUINO_CONNECTOR_A4,
    ARDUINO_CONNECTOR_A5,
} smtc_hal_arduino_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Get the GPIO configuration
 *
 * @param [in] gpio GPIO definition
 *
 * @return A pointer to the MCU-related GPIO configuration
 */
const smtc_hal_mcu_gpio_cfg_t smtc_hal_mcu_get_gpio_cfg( smtc_hal_arduino_t gpio );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_ARDUINO_MAPPING_H
