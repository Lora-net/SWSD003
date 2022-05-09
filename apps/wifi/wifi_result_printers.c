/*!
 * @file      wifi_result_printers.c
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "lr11xx_wifi.h"
#include "smtc_hal_dbg_trace.h"
#include "lr11xx_wifi_types_str.h"
#include "wifi_result_printers.h"
#include "apps_utilities.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

void print_mac_address( const char* prefix, lr11xx_wifi_mac_address_t mac );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void wifi_fetch_and_print_scan_basic_mac_type_channel_results( const void* context )
{
    uint8_t n_results = 0;
    ASSERT_LR11XX_RC( lr11xx_wifi_get_nb_results( context, &n_results ) );

    for( uint8_t result_index = 0; result_index < n_results; result_index++ )
    {
        lr11xx_wifi_basic_mac_type_channel_result_t local_result = { 0 };
        ASSERT_LR11XX_RC( lr11xx_wifi_read_basic_mac_type_channel_results( context, result_index, 1, &local_result ) );

        lr11xx_wifi_mac_origin_t mac_origin    = LR11XX_WIFI_ORIGIN_BEACON_FIX_AP;
        lr11xx_wifi_channel_t    channel       = LR11XX_WIFI_NO_CHANNEL;
        bool                     rssi_validity = false;
        lr11xx_wifi_parse_channel_info( local_result.channel_info_byte, &channel, &rssi_validity, &mac_origin );

        HAL_DBG_TRACE_PRINTF( "Result %u/%u\n", result_index + 1, n_results );
        print_mac_address( "  -> MAC address: ", local_result.mac_address );
        HAL_DBG_TRACE_PRINTF( "  -> Channel: %s\n", lr11xx_wifi_channel_to_str( channel ) );
        HAL_DBG_TRACE_PRINTF( "  -> MAC origin: %s\n", ( rssi_validity ? "From gateway" : "From end device" ) );
        HAL_DBG_TRACE_PRINTF(
            "  -> Signal type: %s\n",
            lr11xx_wifi_signal_type_result_to_str(
                lr11xx_wifi_extract_signal_type_from_data_rate_info( local_result.data_rate_info_byte ) ) );
        HAL_DBG_TRACE_PRINTF( "\n" );
    }
}

void wifi_fetch_and_print_scan_basic_complete_results( const void* context )
{
    uint8_t n_results = 0;
    ASSERT_LR11XX_RC( lr11xx_wifi_get_nb_results( context, &n_results ) );

    for( uint8_t result_index = 0; result_index < n_results; result_index++ )
    {
        lr11xx_wifi_basic_complete_result_t local_result = { 0 };
        ASSERT_LR11XX_RC( lr11xx_wifi_read_basic_complete_results( context, result_index, 1, &local_result ) );

        lr11xx_wifi_mac_origin_t mac_origin    = LR11XX_WIFI_ORIGIN_BEACON_FIX_AP;
        lr11xx_wifi_channel_t    channel       = LR11XX_WIFI_NO_CHANNEL;
        bool                     rssi_validity = false;
        lr11xx_wifi_parse_channel_info( local_result.channel_info_byte, &channel, &rssi_validity, &mac_origin );

        lr11xx_wifi_frame_type_t     frame_type     = LR11XX_WIFI_FRAME_TYPE_MANAGEMENT;
        lr11xx_wifi_frame_sub_type_t frame_sub_type = 0;
        bool                         to_ds          = false;
        bool                         from_ds        = false;
        lr11xx_wifi_parse_frame_type_info( local_result.frame_type_info_byte, &frame_type, &frame_sub_type, &to_ds,
                                           &from_ds );

        HAL_DBG_TRACE_PRINTF( "Result %u/%u\n", result_index + 1, n_results );
        print_mac_address( "  -> MAC address: ", local_result.mac_address );
        HAL_DBG_TRACE_PRINTF( "  -> Channel: %s\n", lr11xx_wifi_channel_to_str( channel ) );
        HAL_DBG_TRACE_PRINTF( "  -> MAC origin: %s\n", ( rssi_validity ? "From gateway" : "From end device" ) );
        HAL_DBG_TRACE_PRINTF(
            "  -> Signal type: %s\n",
            lr11xx_wifi_signal_type_result_to_str(
                lr11xx_wifi_extract_signal_type_from_data_rate_info( local_result.data_rate_info_byte ) ) );
        HAL_DBG_TRACE_PRINTF( "  -> Frame type: %s\n", lr11xx_wifi_frame_type_to_str( frame_type ) );
        HAL_DBG_TRACE_PRINTF( "  -> Frame sub-type: 0x%02X\n", frame_sub_type );
        HAL_DBG_TRACE_PRINTF( "  -> FromDS/ToDS: %s / %s\n", ( ( from_ds == true ) ? "true" : "false" ),
                              ( ( to_ds == true ) ? "true" : "false" ) );
        HAL_DBG_TRACE_PRINTF( "  -> Phi Offset: %i\n", local_result.phi_offset );
        HAL_DBG_TRACE_PRINTF( "  -> Timestamp: %lu us\n", local_result.timestamp_us );
        HAL_DBG_TRACE_PRINTF( "  -> Beacon period: %u TU\n", local_result.beacon_period_tu );
        HAL_DBG_TRACE_PRINTF( "\n" );
    }
}

void wifi_fetch_and_print_scan_extended_complete_results( const void* context )
{
    uint8_t n_results = 0;
    ASSERT_LR11XX_RC( lr11xx_wifi_get_nb_results( context, &n_results ) );

    for( uint8_t result_index = 0; result_index < n_results; result_index++ )
    {
        lr11xx_wifi_extended_full_result_t local_result = { 0 };
        ASSERT_LR11XX_RC( lr11xx_wifi_read_extended_full_results( context, result_index, 1, &local_result ) );

        lr11xx_wifi_mac_origin_t mac_origin    = LR11XX_WIFI_ORIGIN_BEACON_FIX_AP;
        lr11xx_wifi_channel_t    channel       = LR11XX_WIFI_NO_CHANNEL;
        bool                     rssi_validity = false;
        lr11xx_wifi_parse_channel_info( local_result.channel_info_byte, &channel, &rssi_validity, &mac_origin );

        lr11xx_wifi_signal_type_result_t wifi_signal_type = { 0 };
        lr11xx_wifi_datarate_t           wifi_data_rate   = { 0 };
        lr11xx_wifi_parse_data_rate_info( local_result.data_rate_info_byte, &wifi_signal_type, &wifi_data_rate );

        HAL_DBG_TRACE_PRINTF( "Result %u/%u\n", result_index + 1, n_results );
        print_mac_address( "  -> MAC address 1: ", local_result.mac_address_1 );
        print_mac_address( "  -> MAC address 2: ", local_result.mac_address_2 );
        print_mac_address( "  -> MAC address 3: ", local_result.mac_address_3 );
        HAL_DBG_TRACE_PRINTF( "  -> Country code: %c%c\n", ( uint8_t ) ( local_result.country_code[0] ),
                              ( uint8_t ) ( local_result.country_code[1] ) );
        HAL_DBG_TRACE_PRINTF( "  -> Channel: %s\n", lr11xx_wifi_channel_to_str( channel ) );
        HAL_DBG_TRACE_PRINTF( "  -> Signal type: %s\n", lr11xx_wifi_signal_type_result_to_str( wifi_signal_type ) );
        HAL_DBG_TRACE_PRINTF( "  -> RSSI: %i dBm\n", local_result.rssi );
        HAL_DBG_TRACE_PRINTF( "  -> Rate index: 0x%02x\n", local_result.rate );
        HAL_DBG_TRACE_PRINTF( "  -> Service: 0x%04x\n", local_result.service );
        HAL_DBG_TRACE_PRINTF( "  -> Length: %u\n", local_result.length );
        HAL_DBG_TRACE_PRINTF( "  -> Frame control: 0x%04X\n", local_result.frame_control );
        HAL_DBG_TRACE_PRINTF( "  -> Data rate: %s\n", lr11xx_wifi_datarate_to_str( wifi_data_rate ) );
        HAL_DBG_TRACE_PRINTF( "  -> MAC origin: %s\n", ( rssi_validity ? "From gateway" : "From end device" ) );
        HAL_DBG_TRACE_PRINTF( "  -> Phi Offset: %i\n", local_result.phi_offset );
        HAL_DBG_TRACE_PRINTF( "  -> Timestamp: %lu us\n", local_result.timestamp_us );
        HAL_DBG_TRACE_PRINTF( "  -> Beacon period: %u TU\n", local_result.beacon_period_tu );
        HAL_DBG_TRACE_PRINTF( "  -> Sequence control: 0x%04x\n", local_result.seq_control );
        HAL_DBG_TRACE_PRINTF( "  -> IO regulation: 0x%02x\n", local_result.io_regulation );
        HAL_DBG_TRACE_PRINTF( "  -> Current channel: %s\n",
                              lr11xx_wifi_channel_to_str( local_result.current_channel ) );
        HAL_DBG_TRACE_PRINTF( "  -> FCS status:\n    - %s\n",
                              ( local_result.fcs_check_byte.is_fcs_checked ) ? "Is present" : "Is not present" );
        HAL_DBG_TRACE_PRINTF( "    - %s\n", ( local_result.fcs_check_byte.is_fcs_ok ) ? "Valid" : "Not valid" );

        HAL_DBG_TRACE_PRINTF( "\n" );
    }
}

void wifi_fetch_and_print_scan_country_code_results( const void* context )
{
    uint8_t n_results = 0;
    ASSERT_LR11XX_RC( lr11xx_wifi_get_nb_results( context, &n_results ) );

    for( uint8_t result_index = 0; result_index < n_results; result_index++ )
    {
        lr11xx_wifi_country_code_t local_result = { 0 };
        ASSERT_LR11XX_RC( lr11xx_wifi_read_country_code_results( context, result_index, 1, &local_result ) );

        lr11xx_wifi_mac_origin_t mac_origin    = LR11XX_WIFI_ORIGIN_BEACON_FIX_AP;
        lr11xx_wifi_channel_t    channel       = LR11XX_WIFI_NO_CHANNEL;
        bool                     rssi_validity = false;
        lr11xx_wifi_parse_channel_info( local_result.channel_info_byte, &channel, &rssi_validity, &mac_origin );

        HAL_DBG_TRACE_PRINTF( "Result %u/%u\n", result_index + 1, n_results );
        print_mac_address( "  -> MAC address: ", local_result.mac_address );
        HAL_DBG_TRACE_PRINTF( "  -> Country code: %c%c\n", local_result.country_code[0], local_result.country_code[1] );
        HAL_DBG_TRACE_PRINTF( "  -> Channel: %s\n", lr11xx_wifi_channel_to_str( channel ) );
        HAL_DBG_TRACE_PRINTF( "  -> MAC origin: %s\n", ( rssi_validity ? "From gateway" : "From end device" ) );
        HAL_DBG_TRACE_PRINTF( "\n" );
    }
}

void print_mac_address( const char* prefix, lr11xx_wifi_mac_address_t mac )
{
    HAL_DBG_TRACE_PRINTF( "%s%02x:%02x:%02x:%02x:%02x:%02x\n", prefix, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] );
}