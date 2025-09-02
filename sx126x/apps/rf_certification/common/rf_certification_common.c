/*!
 * @file      rf_certification_common.c
 *
 * @brief     Rf certification common functions for SX126x chip
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

#include "rf_certification_common.h"
#include "rf_certification_radio_common.h"
#include "apps_common.h"
#include "apps_utilities.h"
#include "sx126x.h"
#include "sx126x_str.h"
#include "sx126x_lr_fhss.h"
#include "../main_rf_certification.h"
#include "smtc_dbpsk.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "smtc_hal_button.h"
#include "smtc_dbpsk.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

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
static sx126x_hal_context_t* sx126x_radio_context;
static uint8_t               nb_rx_error         = 0;
static uint8_t               nb_rx_timeout       = 0;
static uint8_t               nb_fsk_len_error    = 0;
static uint8_t               nb_ok               = 0;
static uint8_t               lost_pkts_num       = 0;
bool                         rx_timeout          = false;
static bool                  rx_error            = false;
static bool                  rx_fsk_len_error    = false;
static uint8_t               expected_rx_counter = 0;
bool                         tx_done             = false;
bool                         rx_done             = false;
static uint8_t               rx_size             = 0;
static uint8_t               rx_buf[RX_BUF_SIZE] = { 0X00 };
volatile bool                btn_pressed         = false;

static const uint8_t           lr_fhss_sync_word[LR_FHSS_SYNC_WORD_BYTES] = { 0x2C, 0x0F, 0x79, 0x95 };
static sx126x_lr_fhss_params_t lr_fhss_params;
static sx126x_lr_fhss_state_t  lr_fhss_state;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DECLARATIONS -------------------------------------------
 */

/*!
 * @brief Calculates the preamble length needed to reach the target duty cycle
 *
 * @param duty_cycle_target: the target duty cycle to reach
 * @param spreading_factor: spreading factor
 * @param bandwidth: bandwidth (lora or gfsk)
 * @param packet_type: the packet type: lora, gfsk
 * @param pck_len: the packet length in bytes
 *
 * @return the calculated preamble len in symbols
 */
static uint16_t calc_preamble_len_for_target_duty_cycle( float                  duty_cycle_target,
                                                         const sx126x_lora_sf_t spreading_factor,
                                                         const uint16_t bandwidth, const sx126x_pkt_type_t packet_type,
                                                         uint8_t pck_len );

/**
 * @brief Configure the radio to send or receive notififications
 *
 * @param length: payload length
 */
static void init_notify_mode( uint8_t length );

/*!
 * @brief Configure a LR-FHSS payload
 *
 * @param payload: the payload to send. It is the responsibility of the caller to ensure that this references an
 * array containing at least length elements
 * @param length: the length of the payload
 * @param freq_in_hz: frequency in hertz
 */
static void build_lrfhss_frame( uint8_t* payload, uint16_t length, uint32_t freq_in_hz );

/**
 * @brief Puts the radio in rx continuous for spectral scan rx
 */
static void spectral_scan_start( uint32_t freq_hz );

/*!
 * @brief Notifies the receiver about the current transmission that is going on.
 *
 * @param context: pointer to the radio context
 * @param freq_in_hz: frequency in hertz
 * @param spreading_factor: spreading factor
 * @param bandwidth: bandwidth (lora or gfsk)
 * @param packet_type: the packet type (lora or fsk)
 * @param expected_output_pwr_in_dbm: expected output power in dBm
 * @param bitrate: bitrate in bps (gfsk only)
 * @param fdev: frequency deviation (gfsk only)
 */
static void build_notify_string_and_send( uint32_t freq_in_hz, const sx126x_lora_sf_t spreading_factor,
                                          const uint16_t bandwidth, const sx126x_pkt_type_t packet_type,
                                          int8_t expected_output_pwr_in_dbm, uint32_t bitrate, uint32_t fdev );

/*!
 * @brief Fills a buffer with random data.
 *
 * @param [in] buffer  Pointer to the buffer to be filled
 * @param [in] length  Length to be filled in bytes
 */
static void fill_buf_with_random_data( uint8_t* buffer, uint16_t length );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION DEFINITIONS ---------------------------------------------
 */

sx126x_status_t rf_certification_common_send_payloads_until_btn_press(
    uint32_t freq_in_hz, const sx126x_lora_sf_t spreading_factor, const uint16_t bandwidth,
    const sx126x_pkt_type_t packet_type, int8_t expected_output_pwr_in_dbm, uint32_t bitrate, uint32_t fdev,
    uint32_t timeout, uint16_t preamble_len, bool lbt, float duty_cycle, uint8_t payload_length )
{
    switch( packet_type )
    {
    case SX126X_PKT_TYPE_LORA:
    case SX126X_PKT_TYPE_GFSK:
    {
        uint8_t payload[MAX_PAYLOAD_LENGTH];
        fill_buf_with_random_data( payload, payload_length );
        if( duty_cycle != DUTY_CYCLE_OFF_VAL )
        {
            preamble_len = calc_preamble_len_for_target_duty_cycle( duty_cycle, spreading_factor, bandwidth,
                                                                    packet_type, payload_length );
        }
        build_notify_string_and_send( freq_in_hz, spreading_factor, bandwidth, packet_type, expected_output_pwr_in_dbm,
                                      bitrate, fdev );
        rf_certification_common_radio_sx126x_config_working_params(
            freq_in_hz, spreading_factor, bandwidth, packet_type, expected_output_pwr_in_dbm, bitrate, fdev,
            payload_length, preamble_len, SX126X_FALLBACK_STDBY_XOSC );
        rf_certification_common_radio_sx126x_radio_init( sx126x_radio_context );
        break;
    }
    case SX126X_PKT_TYPE_LR_FHSS:
    {
        uint8_t payload[MAX_PAYLOAD_LENGTH];
        fill_buf_with_random_data( payload, payload_length );
        build_notify_string_and_send( freq_in_hz, spreading_factor, bandwidth, packet_type, expected_output_pwr_in_dbm,
                                      bitrate, fdev );
        rf_certification_common_radio_sx126x_config_working_params(
            freq_in_hz, spreading_factor, bandwidth, packet_type, expected_output_pwr_in_dbm, bitrate, fdev,
            payload_length, preamble_len, SX126X_FALLBACK_STDBY_XOSC );
        rf_certification_common_radio_sx126x_radio_init( sx126x_radio_context );
        build_lrfhss_frame( payload, payload_length, freq_in_hz );
        break;
    }
    case SX126X_PKT_TYPE_BPSK:
    {
        uint8_t       frame_buffer[smtc_dbpsk_get_pld_len_in_bytes( SIGFOX_PAYLOAD_LENGTH << 3 )];
        const uint8_t sample0[SIGFOX_PAYLOAD_LENGTH] = { 0xaa, 0xaa, 0xa0, 0x8d, 0x01, 0x05, 0x98, 0xba,
                                                         0xdc, 0xfe, 0x01, 0x9a, 0x09, 0xfe, 0x04 };
        build_notify_string_and_send( freq_in_hz, spreading_factor, bandwidth, packet_type, expected_output_pwr_in_dbm,
                                      bitrate, fdev );
        rf_certification_common_radio_sx126x_config_working_params(
            freq_in_hz, spreading_factor, bandwidth, packet_type, expected_output_pwr_in_dbm, bitrate, fdev,
            payload_length, preamble_len, SX126X_FALLBACK_STDBY_XOSC );

        rf_certification_common_radio_sx126x_radio_init( sx126x_radio_context );
        smtc_dbpsk_encode_buffer( sample0, payload_length << 3, frame_buffer );
        sx126x_write_buffer( sx126x_radio_context, 0, frame_buffer,
                             smtc_dbpsk_get_pld_len_in_bytes( payload_length << 3 ) );
        break;
    }
    default:
        return SX126X_STATUS_ERROR;
    }

    while( true )
    {
        if( lbt == true )
        {
            int16_t rssi = 0;
            spectral_scan_start( freq_in_hz );

            while( rssi > -LBT_RSSI_LIMIT_DBM_MOD )
            {
                sx126x_get_rssi_inst( sx126x_radio_context, &rssi );
                smtc_hal_mcu_wait_ms( DELAY_BETWEEN_EACH_INST_RSSI_FETCH_MS );
            }

            sx126x_set_standby( sx126x_radio_context, SX126X_STANDBY_CFG_XOSC );
            apps_common_sx126x_handle_post_rx( );
        }
        apps_common_sx126x_handle_pre_tx( );
        if( ( lbt == true ) && ( packet_type == SX126X_PKT_TYPE_BPSK ) )
        {
            rf_certification_common_radio_sx126x_radio_init( sx126x_radio_context );
        }
        sx126x_set_tx( sx126x_radio_context, timeout );
        while( !IS_TX_COMPLETED( ) )
        {
            apps_common_sx126x_irq_process( sx126x_radio_context );
        }
        tx_done    = false;
        rx_timeout = false;

        if( btn_pressed )
        {
            btn_pressed = false;
            break;
        }
    }

    return SX126X_STATUS_OK;
}

sx126x_status_t rf_certification_common_rx_infinite( void )
{
    static rx_fsm_states_t rx_fsm_states = RX_FSM_STATES_RX_INFINITE;
    HAL_DBG_TRACE_INFO( "Radio will undefinitely wait for incoming transmissions\n\n" );
    init_notify_mode( 0 );

    switch( rx_fsm_states )
    {
    case RX_FSM_STATES_RX_INFINITE:
        while( true )
        {
            apps_common_sx126x_handle_pre_rx( );
            sx126x_set_rx( sx126x_radio_context, 0 );
            while( rx_done != true )
            {
                apps_common_sx126x_irq_process( sx126x_radio_context );
            }

            rx_done = false;

            apps_common_sx126x_receive( sx126x_radio_context, rx_buf, &rx_size, RX_BUF_SIZE );
            HAL_DBG_TRACE_INFO( "Received: %s\n", ( char* ) rx_buf );

            if( strcmp( ( char* ) rx_buf, RX_TEST_TRIGGER_STRING ) == 0 )
            {
                rx_fsm_states = RX_FSM_STATES_PER;
                break;
            }
            memset( rx_buf, 0x00, RX_BUF_SIZE );
        }
        HAL_DBG_TRACE_INFO( "Started packet error rate calculation\n" );
        break;
    case RX_FSM_STATES_PER:
        while( expected_rx_counter < TX_ERROR_RATE_PCKS )
        {
            apps_common_sx126x_handle_pre_rx( );
            sx126x_set_rx( sx126x_radio_context, RX_TIMEOUT_VALUE );
            HAL_DBG_TRACE_INFO( "Radio started in rx mode with %d ms timeout, waiting for reception...",
                                RX_TIMEOUT_VALUE );

            while( !IS_RX_COMPLETED( ) )
            {
                apps_common_sx126x_irq_process( sx126x_radio_context );
            }

            if( rx_timeout == true )
            {
                HAL_DBG_TRACE_INFO( "packet with counter: %d not received within the maximum %d ms timeout\n",
                                    expected_rx_counter, RX_TIMEOUT_VALUE );
                rx_timeout = false;
                nb_rx_timeout++;
            }

            if( rx_error == true )
            {
                HAL_DBG_TRACE_INFO( "A CRC error occurred while receiving the packet\n" );
                rx_error = false;
                nb_rx_error++;
            }

            if( rx_fsk_len_error == true )
            {
                HAL_DBG_TRACE_INFO( "A FSK len error occurred while receiving the packet\n" );
                rx_fsk_len_error = false;
                nb_fsk_len_error++;
            }

            if( rx_done == true )
            {
                rx_done = false;
                apps_common_sx126x_receive( sx126x_radio_context, rx_buf, &rx_size, RX_BUF_SIZE );
                uint8_t rx_cnt = rx_buf[0];
                if( rx_cnt == expected_rx_counter )
                {
                    HAL_DBG_TRACE_INFO( "correctly received packet with counter: %d\n", expected_rx_counter );
                    nb_ok++;
                }
            }

            expected_rx_counter++;
            memset( rx_buf, 0x00, RX_BUF_SIZE );
        }

        HAL_DBG_TRACE_PRINTF( "Packet receive test completed, results:\n" );
        HAL_DBG_TRACE_PRINTF( "Expected messages: %d \n", ( int ) TX_ERROR_RATE_PCKS );
        HAL_DBG_TRACE_PRINTF( "Valid reception amount: %d \n", nb_ok );
        HAL_DBG_TRACE_PRINTF( "Timeout reception amount: %d \n", nb_rx_timeout );
        HAL_DBG_TRACE_PRINTF( "CRC Error reception amount: %d \n", nb_rx_error );
        nb_rx_error         = 0;
        nb_rx_timeout       = 0;
        nb_fsk_len_error    = 0;
        nb_ok               = 0;
        lost_pkts_num       = 0;
        expected_rx_counter = 0;
        rx_fsm_states       = RX_FSM_STATES_RX_INFINITE;
        break;
    default:
        break;
    }

    return SX126X_STATUS_OK;
}

sx126x_status_t rf_certification_common_notify_receiver( uint8_t* payload, uint8_t len )
{
#ifdef ENABLE_NOTIFICATIONS
    init_notify_mode( len );
    sx126x_write_buffer( sx126x_radio_context, 0, payload, len );
    apps_common_sx126x_handle_pre_tx( );
    HAL_DBG_TRACE_INFO( "Sending notification over the air: %s\n", payload );
    sx126x_set_tx( sx126x_radio_context, 0 );
    while( tx_done != true )
    {
        apps_common_sx126x_irq_process( sx126x_radio_context );
    }
    tx_done = false;
#endif

    return SX126X_STATUS_OK;
}

sx126x_status_t rf_certification_common_set_rx_until_btn_press( uint32_t freq_in_hz )
{
    char notify_string[NOTIFY_STRING_MAX_LEN];
    memset( notify_string, 0x00, NOTIFY_STRING_MAX_LEN );
    sprintf( notify_string, "RX UNTIL BUTTON PRESS AT FREQUENCY %lu HZ", ( unsigned long ) freq_in_hz );
    HAL_DBG_TRACE_INFO( "%s\n", notify_string );
    sx126x_set_rf_freq( sx126x_radio_context, freq_in_hz );
    apps_common_sx126x_handle_pre_rx( );
    sx126x_set_rx( sx126x_radio_context, 0 );

    while( !btn_pressed )
    {
    }

    apps_common_sx126x_handle_post_rx( );
    btn_pressed = false;
    return SX126X_STATUS_OK;
}

void rf_certification_init_radio_context( sx126x_hal_context_t* context )
{
    sx126x_radio_context = context;
}

void on_fhss_hop_done( void )
{
    sx126x_lr_fhss_handle_hop( ( void* ) sx126x_radio_context, &lr_fhss_params, &lr_fhss_state );
}

void on_tx_done( void )
{
    apps_common_sx126x_handle_post_tx( );
    tx_done = true;
}

void on_btn_pressed( void* context )
{
    btn_pressed = true;
}

void on_rx_done( void )
{
    apps_common_sx126x_handle_post_rx( );
    rx_done = true;
}

void init_user_btn( void )
{
    smtc_hal_button_init_user_btn( on_btn_pressed, sx126x_radio_context );
}

void on_rx_timeout( void )
{
    apps_common_sx126x_handle_post_tx( );
    apps_common_sx126x_handle_post_rx( );
    rx_timeout = true;
}

void on_rx_crc_error( void )
{
    rx_error = true;
}

void on_fsk_len_error( void )
{
    rx_fsk_len_error = true;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static void fill_buf_with_random_data( uint8_t* buffer, uint16_t length )
{
    for( uint16_t i = 0; i < length; i++ )
    {
        *buffer = ( rand( ) % 0xFF );
        buffer++;
    }
}

static void spectral_scan_start( uint32_t freq_hz )
{
    /* Set frequency */
    sx126x_set_rf_freq( sx126x_radio_context, freq_hz );

    /* Set Radio in Rx continuous mode */
    apps_common_sx126x_handle_pre_rx( );
    sx126x_set_rx_with_timeout_in_rtc_step( sx126x_radio_context, RX_CONTINUOUS );

    smtc_hal_mcu_wait_ms( DELAY_BETWEEN_SET_RX_AND_VALID_RSSI_MS );
}

static void build_lrfhss_frame( uint8_t* payload, uint16_t length, uint32_t freq_in_hz )
{
    lr_fhss_params.device_offset            = 0,
    lr_fhss_params.center_freq_in_pll_steps = sx126x_convert_freq_in_hz_to_pll_step( freq_in_hz ),
    lr_fhss_params.lr_fhss_params.bw = LR_FHSS_V1_BW_136719_HZ, lr_fhss_params.lr_fhss_params.cr = LR_FHSS_V1_CR_5_6,
    lr_fhss_params.lr_fhss_params.enable_hopping = true, lr_fhss_params.lr_fhss_params.grid = LR_FHSS_V1_GRID_3906_HZ,
    lr_fhss_params.lr_fhss_params.header_count    = 2,
    lr_fhss_params.lr_fhss_params.modulation_type = LR_FHSS_V1_MODULATION_TYPE_GMSK_488,
    lr_fhss_params.lr_fhss_params.sync_word       = lr_fhss_sync_word,

    sx126x_lr_fhss_init( sx126x_radio_context, &lr_fhss_params );

    const uint16_t hop_sequence_id = ( uint16_t ) rand( ) % sx126x_lr_fhss_get_hop_sequence_count( &lr_fhss_params );

    sx126x_lr_fhss_build_frame( ( void* ) sx126x_radio_context, &lr_fhss_params, &lr_fhss_state, hop_sequence_id,
                                payload, length, 0 );
}

static void build_notify_string_and_send( uint32_t freq_in_hz, const sx126x_lora_sf_t spreading_factor,
                                          const uint16_t bandwidth, const sx126x_pkt_type_t packet_type,
                                          int8_t expected_output_pwr_in_dbm, uint32_t bitrate, uint32_t fdev )
{
    char notify_string[NOTIFY_STRING_MAX_LEN] = { 0x00 };
    memset( notify_string, 0x00, NOTIFY_STRING_MAX_LEN );
    switch( packet_type )
    {
    case SX126X_PKT_TYPE_LORA:
        sprintf( notify_string, "LORA TX AT FREQUENCY %lu HZ, BANDWIDTH %s KHZ, SF %s, POWER %d DBM\n",
                 ( unsigned long ) freq_in_hz, sx126x_lora_bw_to_str( ( sx126x_lora_bw_t ) bandwidth ),
                 sx126x_lora_sf_to_str( spreading_factor ), expected_output_pwr_in_dbm );
        break;
    case SX126X_PKT_TYPE_GFSK:
        sprintf( notify_string, "GFSK TX AT FREQUENCY %lu HZ, BANDWIDTH %s KHZ, BITRATE %lu BPS, POWER %d DBM\n",
                 ( unsigned long ) freq_in_hz, sx126x_gfsk_bw_to_str( ( sx126x_gfsk_bw_t ) bandwidth ),
                 ( unsigned long ) bitrate, expected_output_pwr_in_dbm );
        break;
    case SX126X_PKT_TYPE_BPSK:
        sprintf( notify_string, "BPSK TX AT FREQUENCY %lu HZ, BITRATE %lu BPS, POWER %d DBM\n",
                 ( unsigned long ) freq_in_hz, ( unsigned long ) bitrate, expected_output_pwr_in_dbm );
        break;
    case SX126X_PKT_TYPE_LR_FHSS:
        sprintf( notify_string, "LR-FHSS TX AT FREQUENCY %lu HZ, BITRATE 488 BPS, POWER %d DBM\n",
                 ( unsigned long ) freq_in_hz, expected_output_pwr_in_dbm );
        break;
    default:
        break;
    }

    HAL_DBG_TRACE_INFO( "%s\n", notify_string );
    rf_certification_common_notify_receiver( ( uint8_t* ) notify_string, ( uint8_t ) strlen( notify_string ) );
}

static void init_notify_mode( uint8_t length )
{
#ifdef ENABLE_NOTIFICATIONS
#ifdef REGION_ETSI
    CONFIGURE_WORKING_PARAMS_NOTIFY_TX_RX_EU( length );
#elif defined( REGION_FCC )
    CONFIGURE_WORKING_PARAMS_NOTIFY_TX_RX_NA( length );
#elif defined( REGION_ARIB )
    CONFIGURE_WORKING_PARAMS_NOTIFY_TX_RX_JP( length );
#else
#error "Please define a supported certification region"
#endif
    rf_certification_common_radio_sx126x_radio_init( sx126x_radio_context );
#endif
}

static uint16_t calc_preamble_len_for_target_duty_cycle( float                  duty_cycle_target,
                                                         const sx126x_lora_sf_t spreading_factor,
                                                         const uint16_t bandwidth, const sx126x_pkt_type_t packet_type,
                                                         uint8_t pck_len )
{
    uint16_t preamble_len_in_symb = PREAMBLE_MIN_LENGTH;
    float    duty_cycle           = 0.0;
    uint8_t  period               = 0;  // 15 ms measured with this code to switch radio back to send
    uint32_t time_tmp             = 0;

    // just setting the relevant parameters useful to calculate the duty cycle
    rf_certification_common_radio_sx126x_config_working_params( 0, spreading_factor, bandwidth, packet_type, 0, 0, 0,
                                                                pck_len, preamble_len_in_symb,
                                                                SX126X_FALLBACK_STDBY_XOSC );

    while( duty_cycle < duty_cycle_target )
    {
        time_tmp = rf_certification_common_radio_sx126x_get_time_on_air_in_ms( );
        preamble_len_in_symb++;

        if( period == 0 )  // in this case achieve duty_cycle_target in function of time_tmp
        {
            duty_cycle = ( ( ( float ) ( time_tmp ) / ( float ) ( time_tmp + RADIO_OFF_MS ) ) * 100 );
        }
        else
        {
            duty_cycle = ( ( ( float ) ( time_tmp ) / ( float ) ( period + RADIO_OFF_MS ) ) * 100 );
        }
        rf_certification_common_radio_sx126x_config_working_params( 0, spreading_factor, bandwidth, packet_type, 0, 0,
                                                                    0, pck_len, preamble_len_in_symb,
                                                                    SX126X_FALLBACK_STDBY_XOSC );
    }

    return preamble_len_in_symb;
}

/* --- EOF ------------------------------------------------------------------ */
