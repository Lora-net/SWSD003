/*!
 * @file      main_wifi.h
 *
 * @brief     Wi-Fi scan example for LR11xx chip
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

#ifndef MAIN_WIFI_H
#define MAIN_WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_wifi_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * COMMON WI-FI PARAMETERS
 */

/**
 * @brief Selection of the demo to run.
 *
 * It must be one of wifi_demo_to_run_t enumeration
 *
 */
#define WIFI_DEMO_TO_RUN ( WIFI_SCAN_TIME_LIMIT )

/**
 * @brief Wi-Fi channels to scan
 *
 * It is expressed as 16 bits mask of channels
 *
 */
#define WIFI_CHANNEL_MASK ( 0x0421 )

/**
 * @brief Maximum number of results to scan for
 */
#define WIFI_MAX_RESULTS ( 10 )

/**
 * @brief Maximum number of scan to execute
 *
 * If set to 0, the application execute scan indefinitely
 */
#define WIFI_MAX_NUMBER_OF_SCAN ( 0 )

/*
 * WI-FI SCAN PARAMETERS
 */

/**
 * @brief Wi-Fi signal type to scan for
 */
#define WIFI_SIGNAL_TYPE ( LR11XX_WIFI_TYPE_SCAN_B )

/**
 * @brief Wi-Fi scan mode to use
 */
#define WIFI_SCAN_MODE ( LR11XX_WIFI_SCAN_MODE_BEACON )

/**
 * @brief Wi-Fi result formats
 *
 * Must be one of lr11xx_wifi_result_format_t
 *
 * @warning The possible value depends on the value provided in WIFI_SCAN_MODE:
 * WIFI_RESULT_FORMAT can be:
 *   - LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE or LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL if WIFI_SCAN_MODE
 * is:
 *       - LR11XX_WIFI_SCAN_MODE_BEACON or
 *       - LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT
 *   - LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL if WIFI_SCAN_MODE is:
 *       - LR11XX_WIFI_SCAN_MODE_FULL_BEACON or
 *       - LR11XX_WIFI_SCAN_MODE_UNTIL_SSID
 */
#define WIFI_RESULT_FORMAT ( LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE )

/*
 * NO TIME LIMIT PARAMETERS
 */

/**
 * @brief Number of scan per channel to execute
 */
#define WIFI_NB_SCAN_PER_CHANNEL ( 10 )

/**
 * @brief Maximum duration of the scan process
 */
#define WIFI_TIMEOUT_PER_SCAN ( 90 )

/**
 * @brief Indicate if scan on one channel should stop on the first preamble search timeout detected
 */
#define WIFI_ABORT_ON_TIMEOUT ( true )

/*
 * TIME LIMIT PARAMETERS
 */

/**
 * @brief Maximum scan duration spent per channel
 */
#define WIFI_TIMEOUT_PER_CHANNEL ( 1000 )

/**
 * @brief Maximum duration of the scan process
 */
#define WIFI_TIMEOUT_PER_SCAN ( 90 )

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

#endif  // MAIN_WIFI_H

/* --- EOF ------------------------------------------------------------------ */
