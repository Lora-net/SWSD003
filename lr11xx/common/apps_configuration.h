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
#include "lr11xx_radio_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 * @brief General parameters
 */
#ifndef PACKET_TYPE
#define PACKET_TYPE LR11XX_RADIO_PKT_TYPE_LORA
#endif
#ifndef RF_FREQ_IN_HZ
#define RF_FREQ_IN_HZ 490000000U
#endif
#ifndef TX_OUTPUT_POWER_DBM
#define TX_OUTPUT_POWER_DBM 13  // range [-17, +22] for sub-G, range [-18, 13] for 2.4G ( HF_PA )
#endif
#ifndef PA_RAMP_TIME
#define PA_RAMP_TIME LR11XX_RADIO_RAMP_48_US
#endif
#ifndef FALLBACK_MODE
#define FALLBACK_MODE LR11XX_RADIO_FALLBACK_STDBY_RC
#endif
#ifndef ENABLE_RX_BOOST_MODE
#define ENABLE_RX_BOOST_MODE false
#endif
#ifndef PAYLOAD_LENGTH
#define PAYLOAD_LENGTH 7
#endif

/*!
 * @brief Modulation parameters for LoRa packets
 */
#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR LR11XX_RADIO_LORA_SF7
#endif
#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH LR11XX_RADIO_LORA_BW_125
#endif
#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE LR11XX_RADIO_LORA_CR_4_5
#endif

/*!
 * @brief Packet parameters for LoRa packets
 */
#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH 8
#endif
#ifndef LORA_PKT_LEN_MODE
#define LORA_PKT_LEN_MODE LR11XX_RADIO_LORA_PKT_EXPLICIT
#endif
#ifndef LORA_IQ
#define LORA_IQ LR11XX_RADIO_LORA_IQ_INVERTED
#endif
#ifndef LORA_CRC
#define LORA_CRC LR11XX_RADIO_LORA_CRC_ON
#endif

/*!
 * @brief LoRa sync word
 */
#ifndef LORA_SYNCWORD
#define LORA_SYNCWORD 0x12  // 0x12 Private Network, 0x34 Public Network
#endif

/*!
 * @brief Modulation parameters for GFSK packets
 */
#ifndef FSK_FDEV
#define FSK_FDEV 25000U  // Hz
#endif
#ifndef FSK_BITRATE
#define FSK_BITRATE 50000U  // bps
#endif
#ifndef FSK_BANDWIDTH
#define FSK_BANDWIDTH LR11XX_RADIO_GFSK_BW_117300  // Make sure to follow the rule: (2 * FDEV + BITRATE) < BW
#endif
#ifndef FSK_PULSE_SHAPE
#define FSK_PULSE_SHAPE LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF
#endif

/*!
 * @brief Packet parameters for GFSK packets
 */
#ifndef FSK_PREAMBLE_LENGTH
#define FSK_PREAMBLE_LENGTH 32  // bits
#endif
#ifndef FSK_PREAMBLE_DETECTOR
#define FSK_PREAMBLE_DETECTOR LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS
#endif
#ifndef FSK_SYNCWORD_LENGTH
#define FSK_SYNCWORD_LENGTH 40  // bits
#endif
#ifndef FSK_ADDRESS_FILTERING
#define FSK_ADDRESS_FILTERING LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE
#endif
#ifndef FSK_HEADER_TYPE
#define FSK_HEADER_TYPE LR11XX_RADIO_GFSK_PKT_VAR_LEN
#endif
#ifndef FSK_CRC_TYPE
#define FSK_CRC_TYPE LR11XX_RADIO_GFSK_CRC_1_BYTE_INV
#endif
#ifndef FSK_DC_FREE
#define FSK_DC_FREE LR11XX_RADIO_GFSK_DC_FREE_OFF
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

/*!
 * @brief Sigfox radio configuration
 */
#ifndef SIGFOX_RC
#define SIGFOX_RC 1
#endif

#if( SIGFOX_RC == 1 )
#define SIGFOX_UPLINK_RF_FREQ_IN_HZ 868130000
#define BPSK_BITRATE_IN_BPS 100
#define SIGFOX_TX_OUTPUT_POWER_DBM 14
#define RAMP_UP_DELAY SIGFOX_DBPSK_RAMP_UP_TIME_100_BPS
#define RAMP_DOWN_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS
#elif( SIGFOX_RC == 2 )
#define SIGFOX_UPLINK_RF_FREQ_IN_HZ 902200000
#define BPSK_BITRATE_IN_BPS 600
#define SIGFOX_TX_OUTPUT_POWER_DBM 22
#define RAMP_UP_DELAY SIGFOX_DBPSK_RAMP_UP_TIME_600_BPS
#define RAMP_DOWN_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_600_BPS
#elif( SIGFOX_RC == 3 )
#define SIGFOX_UPLINK_RF_FREQ_IN_HZ 923200000
#define BPSK_BITRATE_IN_BPS 100
#define SIGFOX_TX_OUTPUT_POWER_DBM 14
#define RAMP_UP_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS
#define RAMP_DOWN_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS
#elif( SIGFOX_RC == 4 )
#define SIGFOX_UPLINK_RF_FREQ_IN_HZ 920800000
#define BPSK_BITRATE_IN_BPS 600
#define SIGFOX_TX_OUTPUT_POWER_DBM 22
#define RAMP_UP_DELAY SIGFOX_DBPSK_RAMP_UP_TIME_600_BPS
#define RAMP_DOWN_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_600_BPS
#elif( SIGFOX_RC == 5 )
#define SIGFOX_UPLINK_RF_FREQ_IN_HZ 923300000
#define BPSK_BITRATE_IN_BPS 100
#define SIGFOX_TX_OUTPUT_POWER_DBM 12
#define RAMP_UP_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS
#define RAMP_DOWN_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS
#elif( SIGFOX_RC == 6 )
#define SIGFOX_UPLINK_RF_FREQ_IN_HZ 865200000
#define BPSK_BITRATE_IN_BPS 100
#define SIGFOX_TX_OUTPUT_POWER_DBM 14
#define RAMP_UP_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS
#define RAMP_DOWN_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS
#elif( SIGFOX_RC == 7 )
#define SIGFOX_UPLINK_RF_FREQ_IN_HZ 868800000
#define BPSK_BITRATE_IN_BPS 100
#define SIGFOX_TX_OUTPUT_POWER_DBM 14
#define RAMP_UP_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS
#define RAMP_DOWN_DELAY SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS
#else
#error "Select a valid Radio Configuration"
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
