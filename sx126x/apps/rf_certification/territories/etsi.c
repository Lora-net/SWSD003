/*!
 * @file      etsi.c
 *
 * @brief     ETSI example for SX126X chip
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "etsi.h"
#include "rf_certification_common.h"
#include "rf_certification_radio_common.h"
#include "apps_common.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_hal_mcu.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Frequencies used in ETSI-regulated territories
 */
#define RF_FREQUENCY_863_1 863100000u  // Hz
#define RF_FREQUENCY_866_5 866500000u  // Hz
#define RF_FREQUENCY_869_9 869900000u  // Hz
#define RF_FREQUENCY_868_1 868100000u  // Hz
#define RF_FREQUENCY_868_3 868300000u  // Hz
#define RF_FREQUENCY_868_5 868500000u  // Hz
#define RF_FREQUENCY_867_3 867300000u  // Hz

#define ETSI_SIGFOX_FREQUENCY 868130000u

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

/**
 * @brief Executes the PER (packet error rate) test
 */
static void exec_per_test( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION DEFINITIONS ---------------------------------------------
 */

void etsi_tests_fsm( void )
{
    /**********************************************/
    /*                  LORA                      */
    /**********************************************/
    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_863_1, SX126X_LORA_SF7, SX126X_LORA_BW_125, EXPECTED_PWR_14_DBM );
    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_863_1, SX126X_LORA_SF12, SX126X_LORA_BW_125, EXPECTED_PWR_14_DBM );

    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_3, SX126X_LORA_SF7, SX126X_LORA_BW_125, EXPECTED_PWR_14_DBM );
    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_3, SX126X_LORA_SF12, SX126X_LORA_BW_125, EXPECTED_PWR_14_DBM );

    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_5, SX126X_LORA_SF7, SX126X_LORA_BW_125, EXPECTED_PWR_14_DBM );
    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_5, SX126X_LORA_SF12, SX126X_LORA_BW_125, EXPECTED_PWR_14_DBM );

    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_3, SX126X_LORA_SF7, SX126X_LORA_BW_250, EXPECTED_PWR_14_DBM );

    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_866_5, SX126X_LORA_SF7, SX126X_LORA_BW_125, EXPECTED_PWR_14_DBM );
    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_866_5, SX126X_LORA_SF12, SX126X_LORA_BW_125, EXPECTED_PWR_14_DBM );

    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_869_9, SX126X_LORA_SF7, SX126X_LORA_BW_125, EXPECTED_PWR_14_DBM );
    LORA_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_869_9, SX126X_LORA_SF12, SX126X_LORA_BW_125, EXPECTED_PWR_14_DBM );

    /**********************************************/
    /*                  GFSK                      */
    /**********************************************/

    /*Same frequency plan as abovementioned LORA, with LoRaWAN-compliant FSK parameters
    (bw=117khz, bitrate=50kbps, fdev=25khz, syncword=0xc194c1, crc=crc-16-ccitt, gaussian filter=BT 1,0, dc free
    encoding= whitening encoding)*/
    rf_certification_common_radio_sx126x_set_gfsk_lorawan_compliant_params( );
    GFSK_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_863_1, EXPECTED_PWR_14_DBM );
    GFSK_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_866_5, EXPECTED_PWR_14_DBM );
    GFSK_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_1, EXPECTED_PWR_14_DBM );
    GFSK_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_3, EXPECTED_PWR_14_DBM );
    GFSK_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_5, EXPECTED_PWR_14_DBM );
    GFSK_STANDARD_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_869_9, EXPECTED_PWR_14_DBM );

    /**********************************************/
    /*                  LR-FHSS                   */
    /**********************************************/
    LR_FHSS_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_863_1, EXPECTED_PWR_14_DBM );
    LR_FHSS_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_866_5, EXPECTED_PWR_14_DBM );
    LR_FHSS_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_1, EXPECTED_PWR_14_DBM );
    LR_FHSS_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_3, EXPECTED_PWR_14_DBM );
    LR_FHSS_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_868_5, EXPECTED_PWR_14_DBM );
    LR_FHSS_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_869_9, EXPECTED_PWR_14_DBM );

    /**********************************************/
    /*                  BPSK                      */
    /**********************************************/
    DBPSK_SEND_UNTIL_BTN_PRESS( ETSI_SIGFOX_FREQUENCY, EXPECTED_PWR_14_DBM, 100 );

    /**********************************************/
    /* Execute Packet error rate test */
    /**********************************************/
    exec_per_test( );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static void exec_per_test( void )
{
    HAL_DBG_TRACE_INFO( "Started packet error rate tx test\n" );

    rf_certification_common_notify_receiver( ( uint8_t* ) RX_TEST_TRIGGER_STRING, strlen( RX_TEST_TRIGGER_STRING ) );
    smtc_hal_mcu_wait_ms( 500 );
    for( uint8_t tx_cnt = 0; tx_cnt < TX_ERROR_RATE_PCKS; tx_cnt++ )
    {
        HAL_DBG_TRACE_INFO( "Sending packet %d of %d\n", tx_cnt, ( int ) TX_ERROR_RATE_PCKS );
        rf_certification_common_notify_receiver( ( uint8_t* ) &tx_cnt, sizeof( tx_cnt ) );
        smtc_hal_mcu_wait_ms( RX_TIMEOUT_VALUE / 10 );
    }
}

/* --- EOF ------------------------------------------------------------------ */
