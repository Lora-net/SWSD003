/*!
 * @file      rf_certification_commons.h
 *
 * @brief     Rf certification common functions for LR11xx chip
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

#ifndef RF_CERTIFICATION_COMMON_H
#define RF_CERTIFICATION_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "apps_utilities.h"
#include "lr11xx_types.h"
#include "lr11xx_radio_types.h"
#include "lr11xx_hal_context.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief The irq mask used by the transceiver
 */
#define IRQ_MASK                                                                          \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT | \
      LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR | LR11XX_SYSTEM_IRQ_CRC_ERROR )

/**
 * @brief Evaluates if the rx is completed, true if completed, false otherwise
 */
#define IS_RX_COMPLETED( ) ( rx_done == true || rx_timeout == true || rx_error == true || rx_fsk_len_error == true )

/**
 * @brief Evaluates if the tx has been completed, true if completed, false otherwise
 */
#define IS_TX_COMPLETED( ) ( tx_done == true || rx_timeout == true )

/**
 * @brief Sends lora-modulated packets intil the user button is pressed
 *
 * @param freq: frequency in hz
 * @param sf: spreading factor
 * @param bw: lora bandwidth
 * @param pwr: output power in dBm
 * @return The status of the operation
 */
#define LORA_STANDARD_SEND_UNTIL_BTN_PRESS( freq, sf, bw, pwr )                                                    \
    rf_certification_common_send_payloads_until_btn_press( freq, sf, bw, LR11XX_RADIO_PKT_TYPE_LORA, pwr, 0, 0, 0, \
                                                           LORA_PREAMBLE_LENGTH, false, DUTY_CYCLE_OFF_VAL,        \
                                                           MAX_PAYLOAD_LENGTH )

/**
 * @brief Sends lr-fhss-modulated packets until the user button is pressed
 *
 * @param freq: frequency in hz
 * @param pwr: output power in dBm
 * @return The status of the operation
 */
#define LR_FHSS_SEND_UNTIL_BTN_PRESS( freq, pwr )                                                                   \
    rf_certification_common_send_payloads_until_btn_press( freq, 0, 0, LR11XX_RADIO_PKT_TYPE_LR_FHSS, pwr, 0, 0, 0, \
                                                           LORA_PREAMBLE_LENGTH, false, DUTY_CYCLE_OFF_VAL,         \
                                                           LR_FHSS_PAYLOAD_LENGTH )

/**
 * @brief Sends gfsk-modulated packets until the user button is pressed
 *
 * @param freq: frequency in hz
 * @param pwr: output power in dBm
 * @return The status of the operation
 */
#define GFSK_STANDARD_SEND_UNTIL_BTN_PRESS( freq, pwr )                                                               \
    rf_certification_common_send_payloads_until_btn_press( freq, 0, LR11XX_RADIO_GFSK_BW_117300,                      \
                                                           LR11XX_RADIO_PKT_TYPE_GFSK, pwr, FSK_BITRATE, FSK_FDEV, 0, \
                                                           0, false, DUTY_CYCLE_OFF_VAL, MAX_PAYLOAD_LENGTH )
/**
 * @brief Sends dbpsk-modulated packets, with radio configuration taken from radio_configuration.h, until the user
 * button is pressed
 *
 * @return The status of the operation
 */
#define DBPSK_SEND_UNTIL_BTN_PRESS( freq, pwr, bitrate )                                                               \
    rf_certification_common_send_payloads_until_btn_press( freq, 0, 0, LR11XX_RADIO_PKT_TYPE_BPSK, pwr, bitrate, 0, 0, \
                                                           0, false, DUTY_CYCLE_OFF_VAL, SIGFOX_PAYLOAD_LENGTH )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/**
 * @brief Output powers in dBm
 */
#define EXPECTED_PWR_14_DBM 14
#define EXPECTED_PWR_15_DBM 15
#define EXPECTED_PWR_22_DBM 22
#define EXPECTED_PWR_10_DBM 10
#define EXPECTED_PWR_9_DBM 9

/**
 * @brief The listen-before-talk limit (module) in dBm
 */
#define LBT_RSSI_LIMIT_DBM_MOD 80

/**
 * @brief The radio rx buffer size
 */
#define RX_BUF_SIZE 255

/**
 * @brief The number of packets that will be sent in the error rate test
 */
#define TX_ERROR_RATE_PCKS 20

/**
 * @brief The sigfox payload length
 */
#define SIGFOX_PAYLOAD_LENGTH 15

/**
 * @brief The max notify string length
 */
#define NOTIFY_STRING_MAX_LEN 256

/**
 * @brief The time that the radio takes to go back to tx after going to standby in milliseconds
 */
#define RADIO_OFF_MS 6

/**
 * @brief The minimum payload length in symbols
 */
#define MIN_PAYLOAD_LENGTH 8

/**
 * @brief The minimum preamble length in symbols
 */
#define PREAMBLE_MIN_LENGTH 8

/**
 * @brief The lr-fhss payload length
 */
#define LR_FHSS_PAYLOAD_LENGTH 7

/**
 * @brief The string that can trigger the error test when received
 */
#define RX_TEST_TRIGGER_STRING ( ( const char* ) "RX_ERROR_TEST_START" )

/**
 * @brief The value that disables the duty cycle calculation
 */
#define DUTY_CYCLE_OFF_VAL 0

/**
 * @brief Duration of the wait between setting to RX mode and valid instant RSSI value available
 *
 * Expressed in milliseconds
 *
 * @warning If switching from StandbyRC mode this delay is recommended to set to 30ms; if switching from StandbyXOSC,
 * 1ms.
 */
#define DELAY_BETWEEN_SET_RX_AND_VALID_RSSI_MS ( 1 )

/**
 * @brief Duration of the wait between each instant RSSI fetch. This is to make sure that the RSSI value is stable
 * before fetching
 */
#define DELAY_BETWEEN_EACH_INST_RSSI_FETCH_MS ( 1 )

/**
 * @brief The rx timeout in milliseconds
 */
#define RX_TIMEOUT_VALUE 3000

/**
 * @brief The maximum payload length
 */
#define MAX_PAYLOAD_LENGTH 255

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Possible states that the rx fsm can be
 *
 * @enum rx_fsm_states_t
 */
typedef enum rx_fsm_state_e
{
    RX_FSM_STATES_RX_INFINITE,
    RX_FSM_STATES_PER
} rx_fsm_states_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Puts the radio in rx mode
 */
lr11xx_status_t rf_certification_common_rx_infinite( void );

/**
 * @brief Sends the receiver info about the current executed test
 *
 * @return The status of the operation
 */
lr11xx_status_t rf_certification_common_notify_receiver( uint8_t* payload, uint8_t len );

/**
 * @brief Inits the user button
 */
void init_user_btn( void );

/**
 * @brief Called upon button press
 *
 * @param context: software context
 */
void on_btn_pressed( void* context );

/**
 * @brief Sets the radio in rx until the user button is pressed
 *
 * @param freq_in_hz: Frequency in hertz
 *
 * @return The status of the operation
 */
lr11xx_status_t rf_certification_common_set_rx_until_btn_press( uint32_t freq_in_hz );

/*!
 * @brief TX modulated packets until the user button on the board is pressed.
 *
 * @param context: fointer to the radio context
 * @param freq_in_hz: frequency in hertz
 * @param spreading_factor: spreading factor
 * @param bandwidth: bandwidth (lora or gfsk)
 * @param packet_type: the packet type: lora, gfsk, bpsk or lr-fhss
 * @param expected_output_pwr_in_dbm: expected output power in dBm
 * @param bitrate: bitrate in bps (gfsk only)
 * @param fdev: frequency deviation (gfsk only)
 * @param preamble_len: preamble length in bytes
 * @param lbt: listen before talk, if true, radio will listen to the rssi before every transmission
 * @param duty_cycle: duty cycle (ton-toff) in % to reach
 * @param payload_length: payload length in bytes
 *
 * @return The status of the operation
 */
lr11xx_status_t rf_certification_common_send_payloads_until_btn_press(
    uint32_t freq_in_hz, const lr11xx_radio_lora_sf_t spreading_factor, const uint16_t bandwidth,
    const lr11xx_radio_pkt_type_t packet_type, int8_t expected_output_pwr_in_dbm, uint32_t bitrate, uint32_t fdev,
    uint32_t timeout, uint16_t preamble_len, bool lbt, float duty_cycle, uint8_t payload_length );

/*!
 * @brief Inits the radio context used in the rest of the certification
 *
 * @param context the radio context
 */
void rf_certification_init_radio_context( lr11xx_hal_context_t* context );

/**
 * @brief Puts the radio in infinite rx mode
 */
lr11xx_status_t rf_certification_common_rx_infinite( void );

#ifdef __cplusplus
}
#endif

#endif  // RF_CERTIFICATION_COMMON_H

/* --- EOF ------------------------------------------------------------------ */
