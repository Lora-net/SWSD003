/*!
 * @file      main_rf_certification.h
 *
 * @brief     RF certification example for LR11xx chip
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2025. All rights reserved.
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

#ifndef MAIN_RF_CERTIFICATION_H
#define MAIN_RF_CERTIFICATION_H

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

/**
 * @brief The certification region to be used
 */
#define REGION_ETSI
//  #define REGION_FCC
//   #define REGION_ARIB

#if ( defined( REGION_ETSI ) && ( defined( REGION_FCC ) || defined( REGION_ARIB ) ) ) || \
    ( defined( REGION_FCC ) && ( defined( REGION_ETSI ) || defined( REGION_ARIB ) ) ) || \
    ( defined( REGION_ARIB ) && ( defined( REGION_ETSI ) || defined( REGION_FCC ) ) ) || \
    ( !defined( REGION_ETSI ) && !defined( REGION_FCC ) && !defined( REGION_ARIB ) )
#error "Please define one territory at a time"
#endif

/**
 * @brief The radio mode to be used
 */
#define MODE_TX_RX
// #define MODE_RX_ONLY

#if ( defined( MODE_TX_RX ) && defined( MODE_RX_ONLY ) ) || ( !defined( MODE_TX_RX ) && !defined( MODE_RX_ONLY ) )
#error "Please define one radio mode at a time"
#endif

/**
 * @brief If defined, notifications will be sent to the receiver
 * @note This is used to notify a receiver radio of the parameters used in the transmission
 */
#define ENABLE_NOTIFICATIONS

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
 * --- PUBLIC FUNCTION PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif /* MAIN_RF_CERTIFICATION_H */

/* --- EOF ------------------------------------------------------------------ */
