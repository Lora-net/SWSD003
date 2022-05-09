/*!
 * \file      shield_pinout.h
 *
 * \brief     shield specific pinout
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
#ifndef SHIELD_PINOUT_H
#define SHIELD_PINOUT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "board_options.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/********************************************************************************/
/*                         Application     dependant                            */
/********************************************************************************/

// Radio specific pinout and peripherals
#define SMTC_RADIO_SPI_MOSI ARDUINO_CONNECTOR_D11
#define SMTC_RADIO_SPI_MISO ARDUINO_CONNECTOR_D12
#define SMTC_RADIO_SPI_SCLK ARDUINO_CONNECTOR_D13
#define SMTC_RADIO_NSS ARDUINO_CONNECTOR_D7
#define SMTC_RADIO_NRST ARDUINO_CONNECTOR_A0
#define SMTC_RADIO_DIOX ARDUINO_CONNECTOR_D5
#define SMTC_RADIO_BUSY ARDUINO_CONNECTOR_D3

#define RADIO_LNA_CTRL ARDUINO_CONNECTOR_A3

/* Sensors */
#define SMTC_I2C_SCL ARDUINO_CONNECTOR_D15
#define SMTC_I2C_SDA ARDUINO_CONNECTOR_D14

/* LED */
#define SMTC_LED_RX ARDUINO_CONNECTOR_A5
#define SMTC_LED_TX ARDUINO_CONNECTOR_A4
#define SMTC_LED_SCAN ARDUINO_CONNECTOR_D4

#define ACC_INT1 ARDUINO_CONNECTOR_D8

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // SHIELD_PINOUT_H
