/**
 * @file      main_ranging.h
 *
 * @brief     Ranging example for LR11xx chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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

#ifndef MAIN_RANGING_H
#define MAIN_RANGING_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "apps_common.h"
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Ranging device slave (subordinate) mode
 */
#define RANGING_DEVICE_MODE_SUBORDINATE 0

/**
 * @brief Ranging device manager (master) mode
 */
#define RANGING_DEVICE_MODE_MANAGER 1

/**
 * @brief Mode of operation
 */
#ifndef RANGING_DEVICE_MODE
#define RANGING_DEVICE_MODE RANGING_DEVICE_MODE_MANAGER
#endif

/**
 * @brief Subordinate ranging address
 *
 * Address used to identify the ranging subordinate device.
 * This address is configured as address of the subordinate device
 * and used as request address in the manager device.
 */
#ifndef RANGING_ADDRESS
#define RANGING_ADDRESS UINT32_C( 0x00000019 )
#endif

/**
 * @brief Number of symbols in ranging response
 */
#ifndef RESPONSE_SYMBOLS_COUNT
#define RESPONSE_SYMBOLS_COUNT UINT8_C( 15 )
#endif

/**
 * @brief Manager-side ranging timeout [ms]
 *
 * Timeout for receiving a response after sending
 * a ranging request.
 */
#ifndef MANAGER_TX_RX_TIMEOUT_MS
#define MANAGER_TX_RX_TIMEOUT_MS UINT32_C( 3276 )
#endif

/**
 * @brief Manager-side sleep period after ranging [ms]
 *
 */
#ifndef MANAGER_RANGING_SLEEP_PERIOD
#define MANAGER_RANGING_SLEEP_PERIOD UINT32_C( 1000 )
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

#endif  // MAIN_RANGING_H

/* --- EOF ------------------------------------------------------------------ */