/*!
 * @file      arib.c
 *
 * @brief     ARIB example for LR11xx chip
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "rf_certification_common.h"
#include "rf_certification_radio_common.h"
#include "apps_utilities.h"
#include "apps_common.h"
#include "lr11xx_radio_types.h"
#include "arib.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Frequencies used in ARIB-regulated territories
 */
#define RF_FREQUENCY_920_6 920600000u  // Hz
#define RF_FREQUENCY_922_0 922000000u  // Hz
#define RF_FREQUENCY_923_4 923400000u  // Hz
#define RF_FREQUENCY_921_0 921000000u  // Hz

#define ARIB_SIGFOX_FREQ RF_FREQUENCY_922_0

/**
 * @brief Listens to the channel, then send a lr-fhss-modulated packet, then listens again, until the user button is
 * pressed
 *
 * @param freq: frequency in hz
 * @param pwr: output power in dBm
 * @return The status of the operation
 */
#define LR_FHSS_LBT_SEND_UNTIL_BTN_PRESS( freq, pwr )                                                               \
    rf_certification_common_send_payloads_until_btn_press( freq, 0, 0, LR11XX_RADIO_PKT_TYPE_LR_FHSS, pwr, 0, 0, 0, \
                                                           LORA_PREAMBLE_LENGTH, true, DUTY_CYCLE_OFF_VAL,          \
                                                           LR_FHSS_PAYLOAD_LENGTH )

/**
 * @brief Listens to the channel, then send a lora-modulated packet, then listens again, until the user button is
 * pressed
 *
 * @param freq: frequency in hz
 * @param sf: spreading factor
 * @param bw: lora bandwidth
 * @param pwr: output power in dBm
 * @return The status of the operation
 */
#define LORA_LBT_SEND_UNTIL_BTN_PRESS( freq, sf, bw, pwr )                                                         \
    rf_certification_common_send_payloads_until_btn_press( freq, sf, bw, LR11XX_RADIO_PKT_TYPE_LORA, pwr, 0, 0, 0, \
                                                           LORA_PREAMBLE_LENGTH, true, DUTY_CYCLE_OFF_VAL,         \
                                                           MIN_PAYLOAD_LENGTH )

/**
 * @brief Listens to the channel, then send a gfsk-modulated packet, then listens again, until the user button is
 * pressed
 *
 * @param freq: frequency in hz
 * @param pwr: output power in dBm
 * @return The status of the operation
 */
#define GFSK_LBT_SEND_UNTIL_BTN_PRESS( freq, pwr )                                                                    \
    rf_certification_common_send_payloads_until_btn_press( freq, 0, LR11XX_RADIO_GFSK_BW_117300,                      \
                                                           LR11XX_RADIO_PKT_TYPE_GFSK, pwr, FSK_BITRATE, FSK_FDEV, 0, \
                                                           0, true, DUTY_CYCLE_OFF_VAL, MIN_PAYLOAD_LENGTH )

/**
 * @brief Listens to the channel, then send a dbpsk-modulated packet, then listens again, until the user button is
 * pressed
 *
 * @return The status of the operation
 */
#define DBPSK_LBT_SEND_UNTIL_BTN_PRESS( freq, pwr, bitrate )                                                           \
    rf_certification_common_send_payloads_until_btn_press( freq, 0, 0, LR11XX_RADIO_PKT_TYPE_BPSK, pwr, bitrate, 0, 0, \
                                                           0, true, DUTY_CYCLE_OFF_VAL, SIGFOX_PAYLOAD_LENGTH )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DECLARATIONS -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION DEFINITIONS ---------------------------------------------
 */

void arib_tests_fsm( void )
{
    /**********************************************/
    /*                  LORA                      */
    /**********************************************/
    LORA_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_920_6, LR11XX_RADIO_LORA_SF7, LR11XX_RADIO_LORA_BW_125,
                                   EXPECTED_PWR_9_DBM );
    LORA_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_920_6, LR11XX_RADIO_LORA_SF12, LR11XX_RADIO_LORA_BW_125,
                                   EXPECTED_PWR_9_DBM );
    LORA_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_922_0, LR11XX_RADIO_LORA_SF7, LR11XX_RADIO_LORA_BW_125,
                                   EXPECTED_PWR_9_DBM );
    LORA_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_922_0, LR11XX_RADIO_LORA_SF12, LR11XX_RADIO_LORA_BW_125,
                                   EXPECTED_PWR_9_DBM );
    LORA_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_923_4, LR11XX_RADIO_LORA_SF7, LR11XX_RADIO_LORA_BW_125,
                                   EXPECTED_PWR_9_DBM );
    LORA_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_923_4, LR11XX_RADIO_LORA_SF12, LR11XX_RADIO_LORA_BW_125,
                                   EXPECTED_PWR_9_DBM );

    /**********************************************/
    /*                  GFSK                      */
    /**********************************************/
    GFSK_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_920_6, EXPECTED_PWR_9_DBM );
    GFSK_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_922_0, EXPECTED_PWR_9_DBM );
    GFSK_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_923_4, EXPECTED_PWR_9_DBM );

    /**********************************************/
    /*                  LR-FHSS                   */
    /**********************************************/
    LR_FHSS_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_920_6, EXPECTED_PWR_9_DBM );
    LR_FHSS_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_922_0, EXPECTED_PWR_9_DBM );
    LR_FHSS_LBT_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_923_4, EXPECTED_PWR_9_DBM );

    /**********************************************/
    /*                  SIGFOX                    */
    /**********************************************/
    DBPSK_LBT_SEND_UNTIL_BTN_PRESS( ARIB_SIGFOX_FREQ, EXPECTED_PWR_9_DBM, 100 );

    /**********************************************/
    /*                  RX EMISSIONS              */
    /**********************************************/
    rf_certification_common_set_rx_until_btn_press( RF_FREQUENCY_920_6 );
    rf_certification_common_set_rx_until_btn_press( RF_FREQUENCY_922_0 );
    rf_certification_common_set_rx_until_btn_press( RF_FREQUENCY_923_4 );
}

/* --- EOF ------------------------------------------------------------------ */
