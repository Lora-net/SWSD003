/*!
 * @file      apps_configuration.h
 *
 * @brief     Common configuration
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

#ifndef APPS_CONFIGURATION_H
#define APPS_CONFIGURATION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 * @brief General parameters
 */
#ifndef PACKET_TYPE
#define PACKET_TYPE SX126X_PKT_TYPE_LORA
#endif
#ifndef RF_FREQ_IN_HZ
#define RF_FREQ_IN_HZ 490000000
#endif
#ifndef TX_OUTPUT_POWER_DBM
#define TX_OUTPUT_POWER_DBM 14  // range [-17, +22]
#endif
#ifndef PA_RAMP_TIME
#define PA_RAMP_TIME SX126X_RAMP_40_US
#endif
#ifndef FALLBACK_MODE
#define FALLBACK_MODE SX126X_FALLBACK_STDBY_RC
#endif
#ifndef ENABLE_RX_BOOST_MODE
#define ENABLE_RX_BOOST_MODE false
#endif
#ifndef PAYLOAD_LENGTH
#define PAYLOAD_LENGTH 7
#endif

#define MAX_PAYLOAD_LENGTH 255


/*!
 * @brief Modulation parameters for LoRa packets
 */
#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR SX126X_LORA_SF7
#endif
#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH SX126X_LORA_BW_125
#endif
#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE SX126X_LORA_CR_4_5
#endif

/*!
 * @brief Packet parameters for LoRa packets
 */
#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH 8
#endif
#ifndef LORA_PKT_LEN_MODE
#define LORA_PKT_LEN_MODE SX126X_LORA_PKT_EXPLICIT
#endif
#ifndef LORA_IQ
#define LORA_IQ false
#endif
#ifndef LORA_CRC
#define LORA_CRC false
#endif

/*!
 * @brief LoRa sync word
 */
#ifndef LORA_SYNCWORD_PRIVATE_NTW
#define LORA_SYNCWORD_PRIVATE_NTW   0x12 // 0x12 Private Network
#endif
#ifndef LORA_SYNCWORD_PUBLIC_NTW
#define LORA_SYNCWORD_PUBLIC_NTW    0x34 // 0x34 Public Network
#endif
#ifndef LORA_SYNCWORD
#define LORA_SYNCWORD LORA_SYNCWORD_PRIVATE_NTW
#endif

/*!
 * @brief Modulation parameters for GFSK packets
 */
#ifndef FSK_FDEV
#define FSK_FDEV 25e3     // Hz
#endif
#ifndef FSK_BITRATE
#define FSK_BITRATE 50e3  // bps
#endif
#ifndef FSK_BANDWIDTH
#define FSK_BANDWIDTH SX126X_GFSK_BW_117300
#endif
#ifndef FSK_PULSE_SHAPE
#define FSK_PULSE_SHAPE SX126X_GFSK_PULSE_SHAPE_OFF
#endif

/*!
 * @brief Packet parameters for GFSK packets
 */
#ifndef FSK_PREAMBLE_LENGTH
#define FSK_PREAMBLE_LENGTH 32  // bits
#endif
#ifndef FSK_PREAMBLE_DETECTOR
#define FSK_PREAMBLE_DETECTOR SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS
#endif
#ifndef FSK_SYNCWORD_LENGTH
#define FSK_SYNCWORD_LENGTH 40  // bits
#endif
#ifndef FSK_ADDRESS_FILTERING
#define FSK_ADDRESS_FILTERING SX126X_GFSK_ADDRESS_FILTERING_DISABLE
#endif
#ifndef FSK_HEADER_TYPE
#define FSK_HEADER_TYPE SX126X_GFSK_PKT_VAR_LEN
#endif
#ifndef FSK_CRC_TYPE
#define FSK_CRC_TYPE SX126X_GFSK_CRC_1_BYTE_INV
#endif
#ifndef FSK_DC_FREE
#define FSK_DC_FREE SX126X_GFSK_DC_FREE_OFF
#endif

/*!
 * @brief GFSK sync word
 */
static const uint8_t gfsk_sync_word[8] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

/*!
 * @brief GFSK whitening seed
 */
#ifndef FSK_WHITENING_SEED
#define FSK_WHITENING_SEED 0x0123
#endif

/*!
 * @brief GFSK CRC seed
 */
#ifndef FSK_CRC_SEED
#define FSK_CRC_SEED 0x01234567
#endif

/*!
 * @brief GFSK CRC polynomial
 */
#ifndef FSK_CRC_POLYNOMIAL
#define FSK_CRC_POLYNOMIAL 0x01234567
#endif

/*!
 * @brief GFSK address filtering - node address
 */
#ifndef FSK_NODE_ADDRESS
#define FSK_NODE_ADDRESS 0x05
#endif

/*!
 * @brief GFSK address filtering - broadcast address
 */
#ifndef FSK_BROADCAST_ADDRESS
#define FSK_BROADCAST_ADDRESS 0xAB
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

#endif  // APPS_CONFIGURATION_H

/* --- EOF ------------------------------------------------------------------ */
