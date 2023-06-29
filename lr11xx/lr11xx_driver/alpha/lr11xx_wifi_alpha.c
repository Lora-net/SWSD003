/*!
 * @file      lr11xx_wifi_alpha.c
 *
 * @brief     Alpha Wi-Fi driver implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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

#include "lr11xx_wifi_alpha.h"
#include "lr11xx_hal.h"

#ifndef MIN
#define MIN( a, b ) ( ( a > b ) ? b : a )
#endif  // MIN

#define LR11XX_WIFI_CONFIGURE_HARDWARE_DEBARKER_CMD_LENGTH ( 2 + 1 )

#define LR11XX_WIFI_READ_RESULT_CMD_LENGTH ( 5 )
#define LR11XX_WIFI_MAX_SIZE_PER_SPI( single_size ) \
    ( single_size * ( LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( single_size ) ) )
#define LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( single_size ) \
    ( MIN( ( LR11XX_WIFI_READ_RESULT_LIMIT ) / ( single_size ), LR11XX_WIFI_N_RESULTS_MAX_PER_CHUNK ) )
#define LR11XX_WIFI_BASIC_MISC_RESULT_SIZE ( 12 )
#define LR11XX_WIFI_BASIC_MAC_ONLY_RESULT_SIZE ( 6 )
#define LR11XX_WIFI_BASIC_MAC_RSSI_RESULT_SIZE ( 7 )
#define LR11XX_WIFI_EXTENDED_MISC_RESULT_SIZE ( 76 )
#define LR11XX_WIFI_EXTENDED_BEACON_PERIOD_RESULT_SIZE ( 40 )
#define LR11XX_WIFI_READ_RESULT_LIMIT ( 1020 )

#define LR11XX_WIFI_SEARCH_COUNTRY_CODE_TIME_LIMIT_CMD_LENGTH ( 9 )

typedef union
{
    lr11xx_wifi_basic_misc_result_t*             basic_misc;
    lr11xx_wifi_basic_mac_only_result_t*         basic_mac_only;
    lr11xx_wifi_basic_mac_rssi_result_t*         basic_mac_rssi;
    lr11xx_wifi_extended_misc_result_t*          extended_misc;
    lr11xx_wifi_extended_beacon_period_result_t* extended_beacon_period;
} lr11xx_wifi_alpha_result_interface_t;

typedef enum
{
    LR11XX_WIFI_SEARCH_COUNTRY_CODE_TIME_LIMIT_OC = 0x0303,
    LR11XX_WIFI_CONFIGURE_HARDWARE_DEBARKER_OC    = 0x0304,
    LR11XX_WIFI_READ_RESULT_OC                    = 0x0306,
} WifiOpCodesAlpha_t;

static uint16_t uint16_from_array( const uint8_t* array, const uint16_t index );

static uint64_t uint64_from_array( const uint8_t* array, const uint16_t index );

static void lr11xx_wifi_read_mac_address_from_buffer( const uint8_t* buffer, const uint16_t index_in_buffer,
                                                      lr11xx_wifi_mac_address_t mac_address );

static void generic_results_interpreter( const uint8_t n_result_to_parse, const uint8_t* buffer,
                                         lr11xx_wifi_alpha_result_interface_t    result_interface,
                                         const lr11xx_wifi_alpha_result_format_t format_code );

static void lr11xx_wifi_read_results_helper( const void* radio, const uint8_t start_index, const uint8_t n_elem,
                                             uint8_t* buffer, const lr11xx_wifi_alpha_result_format_t result_format );

static void fetch_and_aggregate_all_results( const void* radio, const uint8_t index_result_start,
                                             const uint8_t nb_results, const uint8_t nb_results_per_chunk_max,
                                             const lr11xx_wifi_alpha_result_format_t result_format_code,
                                             uint8_t*                                result_buffer,
                                             lr11xx_wifi_alpha_result_interface_t    result_structures );

static void interpret_basic_misc_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                     lr11xx_wifi_basic_misc_result_t* result );

static void interpret_basic_mac_only_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                         lr11xx_wifi_basic_mac_only_result_t* result );

static void interpret_basic_mac_rssi_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                         lr11xx_wifi_basic_mac_rssi_result_t* result );

static void interpret_extended_misc_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                        lr11xx_wifi_extended_misc_result_t* result );

static void interpret_extended_period_beacon_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                                 lr11xx_wifi_extended_beacon_period_result_t* result );

void fetch_and_aggregate_all_results( const void* radio, const uint8_t index_result_start, const uint8_t nb_results,
                                      const uint8_t                           nb_results_per_chunk_max,
                                      const lr11xx_wifi_alpha_result_format_t result_format_code,
                                      uint8_t* result_buffer, lr11xx_wifi_alpha_result_interface_t result_structures )
{
    uint8_t index_to_read     = index_result_start;
    uint8_t remaining_results = nb_results;

    while( remaining_results > 0 )
    {
        uint8_t results_to_read = MIN( remaining_results, nb_results_per_chunk_max );

        lr11xx_wifi_read_results_helper( radio, index_to_read, results_to_read, result_buffer, result_format_code );

        generic_results_interpreter( results_to_read, result_buffer, result_structures, result_format_code );

        index_to_read += results_to_read;
        remaining_results -= results_to_read;
    }
}

lr11xx_status_t lr11xx_wifi_cfg_hardware_debarker( const void* context, const bool enable_hardware_debarker )
{
    const uint8_t cbuffer[LR11XX_WIFI_CONFIGURE_HARDWARE_DEBARKER_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_WIFI_CONFIGURE_HARDWARE_DEBARKER_OC >> 8 ),
        ( uint8_t ) ( LR11XX_WIFI_CONFIGURE_HARDWARE_DEBARKER_OC >> 0 ),
        ( uint8_t ) ( ( enable_hardware_debarker == true ) ? 1 : 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_WIFI_CONFIGURE_HARDWARE_DEBARKER_CMD_LENGTH,
                                                 0, 0 );
}

void lr11xx_wifi_read_basic_misc_results( const void* radio, const uint8_t start_result_index, const uint8_t nb_results,
                                          lr11xx_wifi_basic_misc_result_t* results )
{
    uint8_t       result_buffer[LR11XX_WIFI_MAX_SIZE_PER_SPI( LR11XX_WIFI_BASIC_MISC_RESULT_SIZE )] = { 0 };
    const uint8_t nb_results_per_chunk_max =
        LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( LR11XX_WIFI_BASIC_MISC_RESULT_SIZE );

    lr11xx_wifi_alpha_result_interface_t result_interface = { 0 };
    result_interface.basic_misc                           = results;

    fetch_and_aggregate_all_results( radio, start_result_index, nb_results, nb_results_per_chunk_max,
                                     LR11XX_WIFI_RESULT_FORMAT_BASIC_MISC, result_buffer, result_interface );
}

void lr11xx_wifi_read_basic_mac_only_results( const void* radio, const uint8_t start_result_index,
                                              const uint8_t nb_results, lr11xx_wifi_basic_mac_only_result_t* results )
{
    uint8_t       result_buffer[LR11XX_WIFI_MAX_SIZE_PER_SPI( LR11XX_WIFI_BASIC_MAC_ONLY_RESULT_SIZE )] = { 0 };
    const uint8_t nb_results_per_chunk_max =
        LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( LR11XX_WIFI_BASIC_MAC_ONLY_RESULT_SIZE );

    lr11xx_wifi_alpha_result_interface_t result_interface = { 0 };
    result_interface.basic_mac_only                       = results;

    fetch_and_aggregate_all_results( radio, start_result_index, nb_results, nb_results_per_chunk_max,
                                     LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_ONLY, result_buffer, result_interface );
}

void lr11xx_wifi_read_basic_mac_rssi_results( const void* radio, const uint8_t start_result_index,
                                              const uint8_t nb_results, lr11xx_wifi_basic_mac_rssi_result_t* results )
{
    uint8_t       result_buffer[LR11XX_WIFI_MAX_SIZE_PER_SPI( LR11XX_WIFI_BASIC_MAC_RSSI_RESULT_SIZE )] = { 0 };
    const uint8_t nb_results_per_chunk_max =
        LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( LR11XX_WIFI_BASIC_MAC_RSSI_RESULT_SIZE );

    lr11xx_wifi_alpha_result_interface_t result_interface = { 0 };
    result_interface.basic_mac_rssi                       = results;

    fetch_and_aggregate_all_results( radio, start_result_index, nb_results, nb_results_per_chunk_max,
                                     LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_RSSI, result_buffer, result_interface );
}

void lr11xx_wifi_read_extended_misc_results( const void* radio, const uint8_t start_result_index,
                                             const uint8_t nb_results, lr11xx_wifi_extended_misc_result_t* results )
{
    uint8_t       result_buffer[LR11XX_WIFI_MAX_SIZE_PER_SPI( LR11XX_WIFI_EXTENDED_MISC_RESULT_SIZE )] = { 0 };
    const uint8_t nb_results_per_chunk_max =
        LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( LR11XX_WIFI_EXTENDED_MISC_RESULT_SIZE );

    lr11xx_wifi_alpha_result_interface_t result_interface = { 0 };
    result_interface.extended_misc                        = results;

    fetch_and_aggregate_all_results( radio, start_result_index, nb_results, nb_results_per_chunk_max,
                                     LR11XX_WIFI_RESULT_FORMAT_EXTENDED_MISC, result_buffer, result_interface );
}

void lr11xx_wifi_read_extended_beacon_period_results( const void* radio, const uint8_t start_result_index,
                                                      const uint8_t                                nb_results,
                                                      lr11xx_wifi_extended_beacon_period_result_t* results )
{
    uint8_t result_buffer[LR11XX_WIFI_MAX_SIZE_PER_SPI( LR11XX_WIFI_RESULT_FORMAT_EXTENDED_PERIOD_BEACON )] = { 0 };
    const uint8_t nb_results_per_chunk_max =
        LR11XX_WIFI_MAX_RESULT_PER_TRANSACTION( LR11XX_WIFI_RESULT_FORMAT_EXTENDED_PERIOD_BEACON );

    lr11xx_wifi_alpha_result_interface_t result_interface = { 0 };
    result_interface.extended_beacon_period               = results;

    fetch_and_aggregate_all_results( radio, start_result_index, nb_results, nb_results_per_chunk_max,
                                     LR11XX_WIFI_RESULT_FORMAT_EXTENDED_PERIOD_BEACON, result_buffer,
                                     result_interface );
}

uint8_t lr11xx_wifi_get_format_code( const lr11xx_wifi_alpha_result_format_t format )
{
    uint8_t format_code = 0x00;
    switch( format )
    {
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MISC:
    {
        format_code = 0x00;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_ONLY:
    {
        format_code = 0x02;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_RSSI:
    {
        format_code = 0x03;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_MISC:
    {
        format_code = 0x00;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_PERIOD_BEACON:
    {
        format_code = 0x02;
        break;
    }
    }
    return format_code;
}

uint8_t lr11xx_wifi_get_result_size_from_format( const lr11xx_wifi_alpha_result_format_t format )
{
    uint8_t result_size = 0;
    switch( format )
    {
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MISC:
    {
        result_size = LR11XX_WIFI_BASIC_MISC_RESULT_SIZE;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_ONLY:
    {
        result_size = LR11XX_WIFI_BASIC_MAC_ONLY_RESULT_SIZE;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_RSSI:
    {
        result_size = LR11XX_WIFI_BASIC_MAC_RSSI_RESULT_SIZE;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_MISC:
    {
        result_size = LR11XX_WIFI_EXTENDED_MISC_RESULT_SIZE;
        break;
    }
    case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_PERIOD_BEACON:
    {
        result_size = LR11XX_WIFI_EXTENDED_BEACON_PERIOD_RESULT_SIZE;
        break;
    }
    }
    return result_size;
}

void generic_results_interpreter( const uint8_t n_result_to_parse, const uint8_t* buffer,
                                  lr11xx_wifi_alpha_result_interface_t    result_interface,
                                  const lr11xx_wifi_alpha_result_format_t format_code )
{
    switch( format_code )
    {
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MISC:
    {
        interpret_basic_misc_result_from_buffer( n_result_to_parse, buffer, result_interface.basic_misc );
        break;
    }

    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_ONLY:
    {
        interpret_basic_mac_only_result_from_buffer( n_result_to_parse, buffer, result_interface.basic_mac_only );
        break;
    }

    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_RSSI:
    {
        interpret_basic_mac_rssi_result_from_buffer( n_result_to_parse, buffer, result_interface.basic_mac_rssi );
        break;
    }

    case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_MISC:
    {
        interpret_extended_misc_result_from_buffer( n_result_to_parse, buffer, result_interface.extended_misc );
        break;
    }

    case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_PERIOD_BEACON:
    {
        interpret_extended_period_beacon_result_from_buffer( n_result_to_parse, buffer,
                                                             result_interface.extended_beacon_period );
        break;
    }
    }
}

void interpret_basic_misc_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                              lr11xx_wifi_basic_misc_result_t* result )
{
    for( uint8_t result_index = 0; result_index < nb_results; result_index++ )
    {
        const uint16_t                   local_index_start = LR11XX_WIFI_BASIC_MISC_RESULT_SIZE * result_index;
        lr11xx_wifi_basic_misc_result_t* local_wifi_result = &result[result_index];
        local_wifi_result->data_rate_info_byte             = buffer[local_index_start + 0];
        local_wifi_result->channel_info_byte               = buffer[local_index_start + 1];
        local_wifi_result->rssi                            = buffer[local_index_start + 2];
        local_wifi_result->frame_type_info_byte            = buffer[local_index_start + 3];
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 4, local_wifi_result->mac_address );
        local_wifi_result->phi_offset = uint16_from_array( buffer, local_index_start + 10 );
    }
}

void interpret_basic_mac_only_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                  lr11xx_wifi_basic_mac_only_result_t* result )
{
    for( uint8_t result_index = 0; result_index < nb_results; result_index++ )
    {
        const uint16_t                       local_index_start = LR11XX_WIFI_BASIC_MAC_ONLY_RESULT_SIZE * result_index;
        lr11xx_wifi_basic_mac_only_result_t* local_wifi_result = &result[result_index];
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start, local_wifi_result->mac_address );
    }
}

void interpret_basic_mac_rssi_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                  lr11xx_wifi_basic_mac_rssi_result_t* result )
{
    for( uint8_t result_index = 0; result_index < nb_results; result_index++ )
    {
        const uint16_t                       local_index_start = LR11XX_WIFI_BASIC_MAC_RSSI_RESULT_SIZE * result_index;
        lr11xx_wifi_basic_mac_rssi_result_t* local_wifi_result = &result[result_index];
        local_wifi_result->rssi                                = buffer[local_index_start];
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 1, local_wifi_result->mac_address );
    }
}

void interpret_extended_misc_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                 lr11xx_wifi_extended_misc_result_t* result )
{
    for( uint8_t result_index = 0; result_index < nb_results; result_index++ )
    {
        const uint16_t                      local_index_start = LR11XX_WIFI_EXTENDED_MISC_RESULT_SIZE * result_index;
        lr11xx_wifi_extended_misc_result_t* local_wifi_result = &result[result_index];

        local_wifi_result->data_rate_info_byte = buffer[local_index_start + 0];
        local_wifi_result->channel_info_byte   = buffer[local_index_start + 1];
        local_wifi_result->rssi                = buffer[local_index_start + 2];
        local_wifi_result->rate                = buffer[local_index_start + 3];
        local_wifi_result->service             = uint16_from_array( buffer, local_index_start + 4 );
        local_wifi_result->length              = uint16_from_array( buffer, local_index_start + 6 );
        local_wifi_result->frame_control       = uint16_from_array( buffer, local_index_start + 8 );
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 10, local_wifi_result->mac_address_1 );
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 16, local_wifi_result->mac_address_2 );
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 22, local_wifi_result->mac_address_3 );
        local_wifi_result->timestamp_us = uint64_from_array( buffer, local_index_start + 28 );
        local_wifi_result->seq_control  = uint16_from_array( buffer, local_index_start + 36 );
        for( uint8_t ssid_index = 0; ssid_index < LR11XX_WIFI_RESULT_SSID_LENGTH; ssid_index++ )
        {
            local_wifi_result->ssid_bytes[ssid_index] = buffer[local_index_start + ssid_index + 38];
        }
        local_wifi_result->country_code                  = uint16_from_array( buffer, local_index_start + 70 );
        local_wifi_result->io_regulation                 = buffer[local_index_start + 72];
        local_wifi_result->fcs_check_byte.is_fcs_checked = ( ( buffer[local_index_start + 73] & 0x01 ) == 0x01 );
        local_wifi_result->fcs_check_byte.is_fcs_ok      = ( ( buffer[local_index_start + 73] & 0x02 ) == 0x02 );
        local_wifi_result->phi_offset                    = uint16_from_array( buffer, local_index_start + 74 );
    }
}

void interpret_extended_period_beacon_result_from_buffer( const uint8_t nb_results, const uint8_t* buffer,
                                                          lr11xx_wifi_extended_beacon_period_result_t* result )
{
    for( uint8_t result_index = 0; result_index < nb_results; result_index++ )
    {
        const uint16_t local_index_start = LR11XX_WIFI_EXTENDED_BEACON_PERIOD_RESULT_SIZE * result_index;
        lr11xx_wifi_extended_beacon_period_result_t* local_wifi_result = &result[result_index];

        local_wifi_result->data_rate_info_byte = buffer[local_index_start + 0];
        local_wifi_result->channel_info_byte   = buffer[local_index_start + 1];
        local_wifi_result->rssi                = buffer[local_index_start + 2];
        local_wifi_result->rate                = buffer[local_index_start + 3];
        local_wifi_result->service             = uint16_from_array( buffer, local_index_start + 4 );
        local_wifi_result->length              = uint16_from_array( buffer, local_index_start + 6 );
        local_wifi_result->frame_control       = uint16_from_array( buffer, local_index_start + 8 );
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 10, local_wifi_result->mac_address_1 );
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 16, local_wifi_result->mac_address_2 );
        lr11xx_wifi_read_mac_address_from_buffer( buffer, local_index_start + 22, local_wifi_result->mac_address_3 );
        local_wifi_result->timestamp_us     = uint64_from_array( buffer, local_index_start + 28 );
        local_wifi_result->beacon_period_tu = uint16_from_array( buffer, local_index_start + 36 );
        local_wifi_result->seq_control      = uint16_from_array( buffer, local_index_start + 38 );
    }
}

void lr11xx_wifi_read_mac_address_from_buffer( const uint8_t* buffer, const uint16_t index_in_buffer,
                                               lr11xx_wifi_mac_address_t mac_address )
{
    for( uint8_t field_mac_index = 0; field_mac_index < LR11XX_WIFI_MAC_ADDRESS_LENGTH; field_mac_index++ )
    {
        mac_address[field_mac_index] = buffer[index_in_buffer + field_mac_index];
    }
}

uint16_t uint16_from_array( const uint8_t* array, const uint16_t index )
{
    return ( uint16_t ) ( array[index] << 8 ) + ( ( uint16_t ) ( array[index + 1] ) );
}

uint64_t uint64_from_array( const uint8_t* array, const uint16_t index )
{
    return ( ( uint64_t ) ( array[index] ) << 56 ) + ( ( uint64_t ) ( array[index + 1] ) << 48 ) +
           ( ( uint64_t ) ( array[index + 2] ) << 40 ) + ( ( uint64_t ) ( array[index + 3] ) << 32 ) +
           ( ( uint64_t ) ( array[index + 4] ) << 24 ) + ( ( uint64_t ) ( array[index + 5] ) << 16 ) +
           ( ( uint64_t ) ( array[index + 6] ) << 8 ) + ( uint64_t ) ( array[index + 7] );
}

void lr11xx_wifi_read_results_helper( const void* radio, const uint8_t start_index, const uint8_t n_elem,
                                      uint8_t* buffer, const lr11xx_wifi_alpha_result_format_t result_format )
{
    const uint8_t  size_single_elem   = lr11xx_wifi_get_result_size_from_format( result_format );
    const uint8_t  result_format_code = lr11xx_wifi_get_format_code( result_format );
    const uint8_t  cbuffer[LR11XX_WIFI_READ_RESULT_CMD_LENGTH] = { ( uint8_t ) ( LR11XX_WIFI_READ_RESULT_OC >> 8 ),
                                                                  ( uint8_t ) ( LR11XX_WIFI_READ_RESULT_OC & 0x00FF ),
                                                                  start_index, n_elem, result_format_code };
    const uint16_t size_total                                  = n_elem * size_single_elem;
    lr11xx_hal_read( radio, cbuffer, LR11XX_WIFI_READ_RESULT_CMD_LENGTH, buffer, size_total );
}