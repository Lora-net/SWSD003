/*!
 * @file      wifi_result_printers.h
 *
 * @brief     Wi-Fi result printers
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

#ifndef WIFI_RESULT_PRINTERS_H
#define WIFI_RESULT_PRINTERS_H

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
 * @brief Fetch and print Wi-Fi scan results with format basic MAC/type/channel
 *
 * This is only valid for Wi-Fi scan and Wi-Fi scan time limit.
 * It is not valid for Wi-Fi scan country code and Wi-Fi scan country code time limit.
 *
 * @param context The radio context
 */
void wifi_fetch_and_print_scan_basic_mac_type_channel_results( const void* context );

/**
 * @brief Fetch and print Wi-Fi scan results with format basic complete
 *
 * This is only valid for Wi-Fi scan and Wi-Fi scan time limit.
 * It is not valid for Wi-Fi scan country code and Wi-Fi scan country code time limit.
 *
 * @param context The radio context
 */
void wifi_fetch_and_print_scan_basic_complete_results( const void* context );

/**
 * @brief Fetch and print Wi-Fi scan results with format extended complete
 *
 * This is only valid for Wi-Fi scan and Wi-Fi scan time limit.
 * It is not valid for Wi-Fi scan country code and Wi-Fi scan country code time limit.
 *
 * @param context The radio context
 */
void wifi_fetch_and_print_scan_extended_complete_results( const void* context );

/**
 * @brief Fetch and print Wi-Fi scan country code results
 *
 * This is only valid for Wi-Fi scan country code and Wi-Fi scan country code time limit.
 * It is not valid for Wi-Fi scan and Wi-Fi scan time limit.
 *
 * @param context The radio context
 */
void wifi_fetch_and_print_scan_country_code_results( const void* context );

#ifdef __cplusplus
}
#endif

#endif  // WIFI_RESULT_PRINTERS_H

/* --- EOF ------------------------------------------------------------------ */
