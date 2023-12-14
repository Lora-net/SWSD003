/*!
 * @file      main_cad.h
 *
 * @brief     Channel Activity Detection (CAD) example for LR11xx chip
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

#ifndef MAIN_CAD_H
#define MAIN_CAD_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 *  @brief Defines the number of symbols used for the CAD detection
 *
 *  @warning A minimum of 2 symbols is recommended to perform a CAD
 */
#ifndef CAD_SYMBOL_NUM
#define CAD_SYMBOL_NUM 2
#endif

/*!
 *  @brief Define the sensitivity of the LoRa® modem when trying to correlate to actual LoRa® preamble
symbols. Threshold used to decide if there is a LoRa activity.
 */
#ifndef CAD_DETECT_PEAK
#define CAD_DETECT_PEAK 50
#endif

/*!
 *  @brief Minimum peak value, meant to filter out case with almost no signal or noise.
 */
#ifndef CAD_DETECT_MIN
#define CAD_DETECT_MIN 10
#endif

/*!
 *  @brief Defines the action to be performed after a CAD operation
 */
#ifndef CAD_EXIT_MODE
#define CAD_EXIT_MODE LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC
#endif

/*!
 *  @brief Timeout is only used when the CAD is performed with CAD_EXIT_MODE = LR11XX_RADIO_CAD_EXIT_MODE_RX or LR11XX_RADIO_CAD_EXIT_MODE_TX
 *  If CAD_EXIT_MODE = LR11XX_RADIO_CAD_EXIT_MODE_RX, check SetRx for Timout definition.
 *  If CAD_EXIT_MODE = LR11XX_RADIO_CAD_EXIT_MODE_TX, check SetTx for Timout definition.
 */
#ifndef CAD_TIMEOUT_MS
#define CAD_TIMEOUT_MS 1000
#endif

/*!
 *  @brief Force user parameters for CAD
 *  Set to true to keep the value set above for CAD parameters
 *  Otherwise they will be overwritten according to Spreading Factor
 *  to optimize detection.
 */
#ifndef USER_PROVIDED_CAD_PARAMETERS
#define USER_PROVIDED_CAD_PARAMETERS false
#endif

/*!
 *  @brief Delay between CAD detection
 *  In milliseconds, how long to wait after last action before
 *  starting a new CAD
 */
#ifndef DELAY_MS_BEFORE_CAD
#define DELAY_MS_BEFORE_CAD 900
#endif

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

#ifdef __cplusplus
}
#endif

#endif  // MAIN_CAD_H

/* --- EOF ------------------------------------------------------------------ */
