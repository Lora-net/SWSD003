/*!
 * @file      fcc.c
 *
 * @brief     FCC example for SX126X chip
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

#include "fcc.h"
#include "apps_common.h"
#include "rf_certification_common.h"
#include "rf_certification_radio_common.h"
#include "sx126x.h"
#include "sx126x_hal_context.h"
#include "sx126x_str.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Frequencies used in FCC-regulated territories
 */
#define RF_FREQUENCY_902_3 902300000u  // Hz
#define RF_FREQUENCY_903_0 903000000u  // Hz
#define RF_FREQUENCY_908_7 908700000u  // Hz
#define RF_FREQUENCY_909_4 909400000u  // Hz
#define RF_FREQUENCY_914_2 914200000u  // Hz
#define RF_FREQUENCY_914_9 914900000u  // Hz
#define RF_FREQUENCY_927_7 927700000u  // Hz

/**
 * @brief The valid LoRaWAN subbands in hybrid mode
 */
#define LORAWAN_SUBBAND_0 0  // 902.3 - 903.7 MHz
#define LORAWAN_SUBBAND_1 1  // 903.9 - 905.3 MHz
#define LORAWAN_SUBBAND_2 2  // 905.5 - 906.9 MHz
#define LORAWAN_SUBBAND_3 3  // 907.1 - 908.5 MHz
#define LORAWAN_SUBBAND_4 4  // 908.7 - 910.1 MHz
#define LORAWAN_SUBBAND_5 5  // 910.3 - 911.7 MHz
#define LORAWAN_SUBBAND_6 6  // 911.9 - 913.3 MHz
#define LORAWAN_SUBBAND_7 7  // 913.5 - 914.9 MHz

/**
 * @brief The LoRaWAN DR0 (SF10/125 kHz) payload length
 */
#define LORAWAN_DR0_PAYLOAD_LENGTH 24

/**
 * @brief The LoRaWAN DR1 (SF9/125 kHz) payload length
 */
#define LORAWAN_DR1_PAYLOAD_LENGTH 66

/**
 * @brief The LoRaWAN DR2 (SF8/125 kHz) payload length
 */
#define LORAWAN_DR2_PAYLOAD_LENGTH 138

/**
 * @brief The LoRaWAN DR3 (SF7/125 kHz) payload length
 */
#define LORAWAN_DR3_PAYLOAD_LENGTH 235

/**
 * @brief The LoRaWAN DR4 (SF8/500 kHz) payload length
 */
#define LORAWAN_DR4_PAYLOAD_LENGTH 235

/**
 * @brief The LoRaWAN DR8 (SF12/500 kHz) payload length
 */
#define LORAWAN_DR8_PAYLOAD_LENGTH 66

/**
 * @brief The LoRaWAN DR9 (SF11/500 kHz) payload length
 */
#define LORAWAN_DR9_PAYLOAD_LENGTH 142

/**
 * @brief The LoRaWAN DR10 (SF10/500 kHz) payload length
 */
#define LORAWAN_DR10_PAYLOAD_LENGTH 255

/**
 * @brief The LoRaWAN DR11 (SF9/500 kHz) payload length
 */
#define LORAWAN_DR11_PAYLOAD_LENGTH 255

/**
 * @brief The LoRaWAN DR12 (SF8/500 kHz) payload length
 */
#define LORAWAN_DR12_PAYLOAD_LENGTH 255

/**
 * @brief The LoRaWAN DR13 (SF7/500 kHz) payload length
 */
#define LORAWAN_DR13_PAYLOAD_LENGTH 255

/**
 * @brief The number of channels that will be swept during the FCC FHSS sweep time
 */
#define FCC_SWEEP_FHSS_CHANNELS_NUM ( 64 )

/**
 * @brief The number of channels that will be swept during the FCC hybrid sweep time
 */
#define FCC_SWEEP_HYBRID_CHANNELS_NUM ( 8 )

/**
 * @brief The width in hz of a FCC channel during sweep mode
 */
#define FCC_SWEEP_CHANNEL_WIDTH_HZ 200000

/**
 * @brief The minimum FCC duty cycle in %
 */
#define FCC_DUTY_CYCLE_MIN 98.5

/**
 * @brief Sends lora-modulated packets with the FCC minimum duty cycle until the user button is pressed
 *
 * @param freq: frequency in hz
 * @param sf: spreading factor
 * @param bw: lora bandwidth
 * @param pwr: output power in dBm
 * @return The status of the operation
 */
#define LORA_DUTY_CYCLE_SEND_UNTIL_BTN_PRESS( freq, sf, bw, pwr, payload_len )                                         \
    rf_certification_common_send_payloads_until_btn_press( freq, sf, bw, SX126X_PKT_TYPE_LORA, pwr, 0, 0, 0, 8, false, \
                                                           FCC_DUTY_CYCLE_MIN, payload_len )

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

extern bool                  tx_done;
extern bool                  rx_timeout;
extern bool                  btn_pressed;
static sx126x_hal_context_t* sx126x_radio_context;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DECLARATIONS -------------------------------------------
 */

/*!
 * @brief Performs the FCC fhss mode sweep test (all 64 125 kHz channels).
 *
 * @param sf: spreading factor
 * @param pwr: output power
 * @param payload_len: payload length
 */

static void fcc_sweep_test_fhss( sx126x_lora_sf_t sf, int8_t pwr, uint8_t payload_len );

/*!
 * @brief Performs the FCC hybrid mode sweep test (125 kHz 8 channel subband).
 *
 * @param subband: LoRaWAN subband
 * @param sf: spreading factor
 * @param pwr: output power
 * @param payload_len: payload length
 */
static void fcc_sweep_test_hybrid( uint8_t subband, sx126x_lora_sf_t sf, int8_t pwr, uint8_t payload_len );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION DEFINITIONS ---------------------------------------------
 */

void fcc_tests_fsm( void )
{
    /**********************************************/
    /*                  DTS MODE                  */
    /**********************************************/
    LORA_DUTY_CYCLE_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_903_0, SX126X_LORA_SF8, SX126X_LORA_BW_500, EXPECTED_PWR_22_DBM,
                                          LORAWAN_DR4_PAYLOAD_LENGTH );
    LORA_DUTY_CYCLE_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_909_4, SX126X_LORA_SF8, SX126X_LORA_BW_500, EXPECTED_PWR_22_DBM,
                                          LORAWAN_DR4_PAYLOAD_LENGTH );
    LORA_DUTY_CYCLE_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_914_2, SX126X_LORA_SF8, SX126X_LORA_BW_500, EXPECTED_PWR_22_DBM,
                                          LORAWAN_DR4_PAYLOAD_LENGTH );

    /**********************************************/
    /*                 HYBRID MODE                */
    /**********************************************/
    LORA_DUTY_CYCLE_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_902_3, SX126X_LORA_SF7, SX126X_LORA_BW_125, EXPECTED_PWR_22_DBM,
                                          LORAWAN_DR3_PAYLOAD_LENGTH );
    LORA_DUTY_CYCLE_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_902_3, SX126X_LORA_SF10, SX126X_LORA_BW_125, EXPECTED_PWR_22_DBM,
                                          LORAWAN_DR0_PAYLOAD_LENGTH );
    LORA_DUTY_CYCLE_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_908_7, SX126X_LORA_SF7, SX126X_LORA_BW_125, EXPECTED_PWR_22_DBM,
                                          LORAWAN_DR3_PAYLOAD_LENGTH );
    LORA_DUTY_CYCLE_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_908_7, SX126X_LORA_SF10, SX126X_LORA_BW_125, EXPECTED_PWR_22_DBM,
                                          LORAWAN_DR0_PAYLOAD_LENGTH );
    LORA_DUTY_CYCLE_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_914_9, SX126X_LORA_SF7, SX126X_LORA_BW_125, EXPECTED_PWR_22_DBM,
                                          LORAWAN_DR3_PAYLOAD_LENGTH );
    LORA_DUTY_CYCLE_SEND_UNTIL_BTN_PRESS( RF_FREQUENCY_914_9, SX126X_LORA_SF10, SX126X_LORA_BW_125, EXPECTED_PWR_22_DBM,
                                          LORAWAN_DR0_PAYLOAD_LENGTH );

    /**********************************************/
    /*                 FHSS SWEEP                 */
    /**********************************************/
    fcc_sweep_test_fhss( SX126X_LORA_SF10, EXPECTED_PWR_22_DBM, LORAWAN_DR0_PAYLOAD_LENGTH );
    fcc_sweep_test_fhss( SX126X_LORA_SF9, EXPECTED_PWR_22_DBM, LORAWAN_DR1_PAYLOAD_LENGTH );
    fcc_sweep_test_fhss( SX126X_LORA_SF8, EXPECTED_PWR_22_DBM, LORAWAN_DR2_PAYLOAD_LENGTH );
    fcc_sweep_test_fhss( SX126X_LORA_SF7, EXPECTED_PWR_22_DBM, LORAWAN_DR3_PAYLOAD_LENGTH );

    /**********************************************/
    /*                 HYBRID SWEEP               */
    /**********************************************/
    fcc_sweep_test_hybrid( LORAWAN_SUBBAND_0, SX126X_LORA_SF10, EXPECTED_PWR_22_DBM, LORAWAN_DR0_PAYLOAD_LENGTH );
    fcc_sweep_test_hybrid( LORAWAN_SUBBAND_0, SX126X_LORA_SF9, EXPECTED_PWR_22_DBM, LORAWAN_DR1_PAYLOAD_LENGTH );
    fcc_sweep_test_hybrid( LORAWAN_SUBBAND_0, SX126X_LORA_SF8, EXPECTED_PWR_22_DBM, LORAWAN_DR2_PAYLOAD_LENGTH );
    fcc_sweep_test_hybrid( LORAWAN_SUBBAND_0, SX126X_LORA_SF7, EXPECTED_PWR_22_DBM, LORAWAN_DR3_PAYLOAD_LENGTH );
}

void fcc_init_radio_context( sx126x_hal_context_t* context )
{
    sx126x_radio_context = context;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static void fcc_sweep_test_fhss( sx126x_lora_sf_t sf, int8_t pwr, uint8_t payload_len )
{
    uint32_t frequency = 0;
    uint8_t  i, j, temp = 0;

    uint8_t channel_index[FCC_SWEEP_FHSS_CHANNELS_NUM] = { 0 };
    uint8_t payload[MAX_PAYLOAD_LENGTH];

    // generate pseudo-random channel map for subband
    for( i = 0; i < FCC_SWEEP_FHSS_CHANNELS_NUM; i++ ) channel_index[i] = i;

    for( i = ( FCC_SWEEP_FHSS_CHANNELS_NUM - 1 ); i > 0; i-- )
    {
        j                = rand( ) % ( i + 1 );
        temp             = channel_index[i];
        channel_index[i] = channel_index[j];
        channel_index[j] = temp;
    }

    char notify_string[NOTIFY_STRING_MAX_LEN] = { 0x00 };
    sprintf( notify_string,
             "FHSS sweep test, start freq %u hz, %u channels with %u frequency step and spreading factor %s",
             RF_FREQUENCY_902_3, FCC_SWEEP_FHSS_CHANNELS_NUM, FCC_SWEEP_CHANNEL_WIDTH_HZ, sx126x_lora_sf_to_str( sf ) );
    HAL_DBG_TRACE_INFO( "%s\n", notify_string );
    rf_certification_common_notify_receiver( ( uint8_t* ) notify_string, ( uint8_t ) strlen( notify_string ) );

    while( !btn_pressed )
    {
        for( i = 0; i < FCC_SWEEP_FHSS_CHANNELS_NUM; i++ )
        {
            frequency = RF_FREQUENCY_902_3 + ( FCC_SWEEP_CHANNEL_WIDTH_HZ * channel_index[i] );

            rf_certification_common_radio_sx126x_config_working_params( frequency, sf, SX126X_LORA_BW_125,
                                                                        SX126X_PKT_TYPE_LORA, pwr, 0, 0, payload_len, 8,
                                                                        SX126X_FALLBACK_STDBY_XOSC );
            rf_certification_common_radio_sx126x_radio_init( sx126x_radio_context );
            sx126x_write_buffer( sx126x_radio_context, 0, payload, payload_len );
            apps_common_sx126x_handle_pre_tx( );
            sx126x_set_tx( sx126x_radio_context, 1000 );

            while( !IS_TX_COMPLETED( ) )
            {
                apps_common_sx126x_irq_process( sx126x_radio_context );
            }

            tx_done    = false;
            rx_timeout = false;
        }
    }

    btn_pressed = false;
}

static void fcc_sweep_test_hybrid( uint8_t subband, sx126x_lora_sf_t sf, int8_t pwr, uint8_t payload_len )
{
    uint32_t frequency = 0;
    uint8_t  i, j, temp = 0;

    uint8_t channel_index[FCC_SWEEP_HYBRID_CHANNELS_NUM] = { 0 };
    uint8_t payload[MAX_PAYLOAD_LENGTH];

    // generate pseudo-random channel map for subband
    for( i = 0; i < FCC_SWEEP_HYBRID_CHANNELS_NUM; i++ ) channel_index[i] = i;

    for( i = ( FCC_SWEEP_HYBRID_CHANNELS_NUM - 1 ); i > 0; i-- )
    {
        j                = rand( ) % ( i + 1 );
        temp             = channel_index[i];
        channel_index[i] = channel_index[j];
        channel_index[j] = temp;
    }

    char notify_string[NOTIFY_STRING_MAX_LEN] = { 0x00 };
    sprintf( notify_string,
             "Hybrid sweep test, start freq %lu hz, %u channels with %lu frequency step and spreading factor %s",
             ( unsigned long ) RF_FREQUENCY_902_3, ( uint8_t ) FCC_SWEEP_HYBRID_CHANNELS_NUM,
             ( unsigned long ) FCC_SWEEP_CHANNEL_WIDTH_HZ, sx126x_lora_sf_to_str( sf ) );
    HAL_DBG_TRACE_INFO( "%s\n", notify_string );
    rf_certification_common_notify_receiver( ( uint8_t* ) notify_string, ( uint8_t ) strlen( notify_string ) );

    while( !btn_pressed )
    {
        for( i = 0; i < FCC_SWEEP_HYBRID_CHANNELS_NUM; i++ )
        {
            frequency = RF_FREQUENCY_902_3 +
                        ( FCC_SWEEP_CHANNEL_WIDTH_HZ * ( subband * FCC_SWEEP_HYBRID_CHANNELS_NUM + channel_index[i] ) );

            rf_certification_common_radio_sx126x_config_working_params( frequency, sf, SX126X_LORA_BW_125,
                                                                        SX126X_PKT_TYPE_LORA, pwr, 0, 0, payload_len, 8,
                                                                        SX126X_FALLBACK_STDBY_XOSC );
            rf_certification_common_radio_sx126x_radio_init( sx126x_radio_context );
            sx126x_write_buffer( sx126x_radio_context, 0, payload, payload_len );
            apps_common_sx126x_handle_pre_tx( );
            sx126x_set_tx( sx126x_radio_context, 1000 );

            while( !IS_TX_COMPLETED( ) )
            {
                apps_common_sx126x_irq_process( sx126x_radio_context );
            }

            tx_done    = false;
            rx_timeout = false;
        }
    }

    btn_pressed = false;
}

/* --- EOF ------------------------------------------------------------------ */
