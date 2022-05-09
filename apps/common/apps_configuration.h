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
#define PACKET_TYPE LR11XX_RADIO_PKT_TYPE_LORA
#define RF_FREQ_IN_HZ 490000000U
#define TX_OUTPUT_POWER_DBM 14  // range [-17, +22] for sub-G, range [-18, 13] for 2.4G ( HF_PA )
#define PA_RAMP_TIME LR11XX_RADIO_RAMP_48_US
#define FALLBACK_MODE LR11XX_RADIO_FALLBACK_STDBY_RC
#define ENABLE_RX_BOOST_MODE false
#define PAYLOAD_LENGTH 7

/*!
 * @brief Modulation parameters for LoRa packets
 */
#define LORA_SPREADING_FACTOR LR11XX_RADIO_LORA_SF7
#define LORA_BANDWIDTH LR11XX_RADIO_LORA_BW_125
#define LORA_CODING_RATE LR11XX_RADIO_LORA_CR_4_5

/*!
 * @brief Packet parameters for LoRa packets
 */
#define LORA_PREAMBLE_LENGTH 8
#define LORA_PKT_LEN_MODE LR11XX_RADIO_LORA_PKT_EXPLICIT
#define LORA_IQ LR11XX_RADIO_LORA_IQ_INVERTED
#define LORA_CRC LR11XX_RADIO_LORA_CRC_ON

/*!
 * @brief LoRa sync word
 */
#define LORA_SYNCWORD 0x12  // 0x12 Private Network, 0x34 Public Network

/*!
 * @brief Modulation parameters for GFSK packets
 */
#define FSK_FDEV 25000U                            // Hz
#define FSK_BITRATE 50000U                         // bps
#define FSK_BANDWIDTH LR11XX_RADIO_GFSK_BW_117300  // Make sure to follow the rule: (2 * FDEV + BITRATE) < BW
#define FSK_PULSE_SHAPE LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF

/*!
 * @brief Packet parameters for GFSK packets
 */
#define FSK_PREAMBLE_LENGTH 32  // bits
#define FSK_PREAMBLE_DETECTOR LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS
#define FSK_SYNCWORD_LENGTH 40  // bits
#define FSK_ADDRESS_FILTERING LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE
#define FSK_HEADER_TYPE LR11XX_RADIO_GFSK_PKT_VAR_LEN
#define FSK_CRC_TYPE LR11XX_RADIO_GFSK_CRC_1_BYTE_INV
#define FSK_DC_FREE LR11XX_RADIO_GFSK_DC_FREE_OFF

/*!
 * @brief GFSK sync word
 */
static const uint8_t gfsk_sync_word[8] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

/*!
 * @brief GFSK whitening seed
 */
#define FSK_WHITENING_SEED 0x0123

/*!
 * @brief GFSK CRC seed
 */
#define FSK_CRC_SEED 0x01234567

/*!
 * @brief GFSK CRC polynomial
 */
#define FSK_CRC_POLYNOMIAL 0x01234567

/*!
 * @brief GFSK address filtering - node address
 */
#define FSK_NODE_ADDRESS 0x05

/*!
 * @brief GFSK address filtering - broadcast address
 */
#define FSK_BROADCAST_ADDRESS 0xAB

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
