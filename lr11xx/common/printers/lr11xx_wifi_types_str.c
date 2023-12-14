/*!
 * @file      lr11xx_wifi_types_str.c
 *
 * @brief     Printer helper functions for LR11xx Wi-Fi types
 *
 * @copyright
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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
 
#include "lr11xx_wifi_types_str.h"

const char* lr11xx_wifi_channel_to_str( const lr11xx_wifi_channel_t value )
{
    switch( value )
    {
    case LR11XX_WIFI_NO_CHANNEL:
    {
        return ( const char* ) "LR11XX_WIFI_NO_CHANNEL";
    }

    case LR11XX_WIFI_CHANNEL_1:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_1";
    }

    case LR11XX_WIFI_CHANNEL_2:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_2";
    }

    case LR11XX_WIFI_CHANNEL_3:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_3";
    }

    case LR11XX_WIFI_CHANNEL_4:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_4";
    }

    case LR11XX_WIFI_CHANNEL_5:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_5";
    }

    case LR11XX_WIFI_CHANNEL_6:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_6";
    }

    case LR11XX_WIFI_CHANNEL_7:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_7";
    }

    case LR11XX_WIFI_CHANNEL_8:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_8";
    }

    case LR11XX_WIFI_CHANNEL_9:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_9";
    }

    case LR11XX_WIFI_CHANNEL_10:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_10";
    }

    case LR11XX_WIFI_CHANNEL_11:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_11";
    }

    case LR11XX_WIFI_CHANNEL_12:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_12";
    }

    case LR11XX_WIFI_CHANNEL_13:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_13";
    }

    case LR11XX_WIFI_CHANNEL_14:
    {
        return ( const char* ) "LR11XX_WIFI_CHANNEL_14";
    }

    case LR11XX_WIFI_ALL_CHANNELS:
    {
        return ( const char* ) "LR11XX_WIFI_ALL_CHANNELS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_wifi_datarate_to_str( const lr11xx_wifi_datarate_t value )
{
    switch( value )
    {
    case LR11XX_WIFI_DATARATE_1_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_1_MBPS";
    }

    case LR11XX_WIFI_DATARATE_2_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_2_MBPS";
    }

    case LR11XX_WIFI_DATARATE_6_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_6_MBPS";
    }

    case LR11XX_WIFI_DATARATE_9_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_9_MBPS";
    }

    case LR11XX_WIFI_DATARATE_12_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_12_MBPS";
    }

    case LR11XX_WIFI_DATARATE_18_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_18_MBPS";
    }

    case LR11XX_WIFI_DATARATE_24_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_24_MBPS";
    }

    case LR11XX_WIFI_DATARATE_36_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_36_MBPS";
    }

    case LR11XX_WIFI_DATARATE_48_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_48_MBPS";
    }

    case LR11XX_WIFI_DATARATE_54_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_54_MBPS";
    }

    case LR11XX_WIFI_DATARATE_6_5_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_6_5_MBPS";
    }

    case LR11XX_WIFI_DATARATE_13_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_13_MBPS";
    }

    case LR11XX_WIFI_DATARATE_19_5_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_19_5_MBPS";
    }

    case LR11XX_WIFI_DATARATE_26_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_26_MBPS";
    }

    case LR11XX_WIFI_DATARATE_39_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_39_MBPS";
    }

    case LR11XX_WIFI_DATARATE_52_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_52_MBPS";
    }

    case LR11XX_WIFI_DATARATE_58_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_58_MBPS";
    }

    case LR11XX_WIFI_DATARATE_65_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_65_MBPS";
    }

    case LR11XX_WIFI_DATARATE_7_2_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_7_2_MBPS";
    }

    case LR11XX_WIFI_DATARATE_14_4_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_14_4_MBPS";
    }

    case LR11XX_WIFI_DATARATE_21_7_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_21_7_MBPS";
    }

    case LR11XX_WIFI_DATARATE_28_9_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_28_9_MBPS";
    }

    case LR11XX_WIFI_DATARATE_43_3_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_43_3_MBPS";
    }

    case LR11XX_WIFI_DATARATE_57_8_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_57_8_MBPS";
    }

    case LR11XX_WIFI_DATARATE_65_2_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_65_2_MBPS";
    }

    case LR11XX_WIFI_DATARATE_72_2_MBPS:
    {
        return ( const char* ) "LR11XX_WIFI_DATARATE_72_2_MBPS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_wifi_frame_type_to_str( const lr11xx_wifi_frame_type_t value )
{
    switch( value )
    {
    case LR11XX_WIFI_FRAME_TYPE_MANAGEMENT:
    {
        return ( const char* ) "LR11XX_WIFI_FRAME_TYPE_MANAGEMENT";
    }

    case LR11XX_WIFI_FRAME_TYPE_CONTROL:
    {
        return ( const char* ) "LR11XX_WIFI_FRAME_TYPE_CONTROL";
    }

    case LR11XX_WIFI_FRAME_TYPE_DATA:
    {
        return ( const char* ) "LR11XX_WIFI_FRAME_TYPE_DATA";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_wifi_mac_origin_to_str( const lr11xx_wifi_mac_origin_t value )
{
    switch( value )
    {
    case LR11XX_WIFI_ORIGIN_BEACON_FIX_AP:
    {
        return ( const char* ) "LR11XX_WIFI_ORIGIN_BEACON_FIX_AP";
    }

    case LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP:
    {
        return ( const char* ) "LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP";
    }

    case LR11XX_WIFI_ORIGIN_UNKNOWN:
    {
        return ( const char* ) "LR11XX_WIFI_ORIGIN_UNKNOWN";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_wifi_signal_type_scan_to_str( const lr11xx_wifi_signal_type_scan_t value )
{
    switch( value )
    {
    case LR11XX_WIFI_TYPE_SCAN_B:
    {
        return ( const char* ) "LR11XX_WIFI_TYPE_SCAN_B";
    }

    case LR11XX_WIFI_TYPE_SCAN_G:
    {
        return ( const char* ) "LR11XX_WIFI_TYPE_SCAN_G";
    }

    case LR11XX_WIFI_TYPE_SCAN_N:
    {
        return ( const char* ) "LR11XX_WIFI_TYPE_SCAN_N";
    }

    case LR11XX_WIFI_TYPE_SCAN_B_G_N:
    {
        return ( const char* ) "LR11XX_WIFI_TYPE_SCAN_B_G_N";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_wifi_signal_type_result_to_str( const lr11xx_wifi_signal_type_result_t value )
{
    switch( value )
    {
    case LR11XX_WIFI_TYPE_RESULT_B:
    {
        return ( const char* ) "LR11XX_WIFI_TYPE_RESULT_B";
    }

    case LR11XX_WIFI_TYPE_RESULT_G:
    {
        return ( const char* ) "LR11XX_WIFI_TYPE_RESULT_G";
    }

    case LR11XX_WIFI_TYPE_RESULT_N:
    {
        return ( const char* ) "LR11XX_WIFI_TYPE_RESULT_N";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_wifi_mode_to_str( const lr11xx_wifi_mode_t value )
{
    switch( value )
    {
    case LR11XX_WIFI_SCAN_MODE_BEACON:
    {
        return ( const char* ) "LR11XX_WIFI_SCAN_MODE_BEACON";
    }

    case LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT:
    {
        return ( const char* ) "LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT";
    }

    case LR11XX_WIFI_SCAN_MODE_FULL_BEACON:
    {
        return ( const char* ) "LR11XX_WIFI_SCAN_MODE_FULL_BEACON";
    }

    case LR11XX_WIFI_SCAN_MODE_UNTIL_SSID:
    {
        return ( const char* ) "LR11XX_WIFI_SCAN_MODE_UNTIL_SSID";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_wifi_result_format_to_str( const lr11xx_wifi_result_format_t value )
{
    switch( value )
    {
    case LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE:
    {
        return ( const char* ) "LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE";
    }

    case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL:
    {
        return ( const char* ) "LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL";
    }

    case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL:
    {
        return ( const char* ) "LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}
