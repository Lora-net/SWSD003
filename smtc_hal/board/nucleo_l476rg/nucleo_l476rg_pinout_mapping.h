/*!
 * @file      nucleo_l476rg_pinout_mapping.h
 *
 * @brief     TBD
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

#ifndef NUCLEO_L476RG_PINOUT_MAPPING_H
#define NUCLEO_L476RG_PINOUT_MAPPING_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_hal_gpio_pin_names.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define ARDUINO_CONNECTOR_D0 PA_3
#define ARDUINO_CONNECTOR_D1 PA_2
#define ARDUINO_CONNECTOR_D2 PA_10
#define ARDUINO_CONNECTOR_D3 PB_3
#define ARDUINO_CONNECTOR_D4 PB_5
#define ARDUINO_CONNECTOR_D5 PB_4
#define ARDUINO_CONNECTOR_D6 PB_10
#define ARDUINO_CONNECTOR_D7 PA_8
#define ARDUINO_CONNECTOR_D8 PA_9
#define ARDUINO_CONNECTOR_D9 PC_7
#define ARDUINO_CONNECTOR_D10 PB_6
#define ARDUINO_CONNECTOR_D11 PA_7
#define ARDUINO_CONNECTOR_D12 PA_6
#define ARDUINO_CONNECTOR_D13 PA_5
#define ARDUINO_CONNECTOR_D14 PB_9
#define ARDUINO_CONNECTOR_D15 PB_8

#define ARDUINO_CONNECTOR_A0 PA_0
#define ARDUINO_CONNECTOR_A1 PA_1
#define ARDUINO_CONNECTOR_A2 PA_4
#define ARDUINO_CONNECTOR_A3 PB_0
#define ARDUINO_CONNECTOR_A4 PC_1
#define ARDUINO_CONNECTOR_A5 PC_0

#ifdef __cplusplus
}
#endif

#endif  // NUCLEO_L476RG_PINOUT_MAPPING_H

/* --- EOF ------------------------------------------------------------------ */
