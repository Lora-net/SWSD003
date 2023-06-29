/*!
 * @file      lr11xx_wifi_alpha.h
 *
 * @brief     The Alpha Wi-Fi driver API for LR11XX
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

#ifndef __LR11XX_WIFI_ALPHA_DRIVER_H__
#define __LR11XX_WIFI_ALPHA_DRIVER_H__

#include "lr11xx_regmem.h"
#include "lr11xx_wifi_types.h"
#include "lr11xx_wifi.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Basic misc result structure
 */
typedef struct
{
    lr11xx_wifi_datarate_info_byte_t   data_rate_info_byte;
    lr11xx_wifi_channel_info_byte_t    channel_info_byte;
    int8_t                             rssi;
    lr11xx_wifi_frame_type_info_byte_t frame_type_info_byte;
    lr11xx_wifi_mac_address_t          mac_address;
    int16_t                            phi_offset;
} lr11xx_wifi_basic_misc_result_t;

/*!
 * @brief Basic MAC only result structure
 */
typedef struct
{
    lr11xx_wifi_mac_address_t mac_address;
} lr11xx_wifi_basic_mac_only_result_t;

/*!
 * @brief Basic RSSI, MAC result structure
 */
typedef struct
{
    int8_t                    rssi;
    lr11xx_wifi_mac_address_t mac_address;
} lr11xx_wifi_basic_mac_rssi_result_t;

/*!
 * @brief Extended misc result structure
 */
typedef struct
{
    lr11xx_wifi_datarate_info_byte_t data_rate_info_byte;
    lr11xx_wifi_channel_info_byte_t  channel_info_byte;
    int8_t                           rssi;
    uint8_t                          rate;     //!< Rate index
    uint16_t                         service;  //!< Service value
    uint16_t                         length;   //!< Length of MPDU (in microseconds for WiFi B, bytes for
                                               //!< WiFi G)
    uint16_t                  frame_control;   //!< Frame Control structure
    lr11xx_wifi_mac_address_t mac_address_1;
    lr11xx_wifi_mac_address_t mac_address_2;
    lr11xx_wifi_mac_address_t mac_address_3;
    uint64_t                  timestamp_us;               //!< Indicate the up-time of the Access Point
                                                          //!< transmitting the Beacon [us]
    uint16_t seq_control;                                 //!< Sequence Control value
    uint8_t  ssid_bytes[LR11XX_WIFI_RESULT_SSID_LENGTH];  //!< Service Set
                                                          //!< IDentifier
    uint16_t                    country_code;             //!< Country Code
    uint8_t                     io_regulation;            //!< Input Output Regulation
    lr11xx_wifi_fcs_info_byte_t fcs_check_byte;           //!< Frame Check Sequence info
    int16_t                     phi_offset;
} lr11xx_wifi_extended_misc_result_t;

/*!
 * @brief Extended beacon period result structure
 *
 * The beacon period is expressed in TU (Time Unit). 1 TU is 1024 microseconds.
 */
typedef struct
{
    lr11xx_wifi_datarate_info_byte_t data_rate_info_byte;
    lr11xx_wifi_channel_info_byte_t  channel_info_byte;
    int8_t                           rssi;
    uint8_t                          rate;     //!< Rate index
    uint16_t                         service;  //!< Service value
    uint16_t                         length;   //!< Length of MPDU (in microseconds for WiFi B, bytes for
                                               //!< WiFi G)
    uint16_t                  frame_control;   //!< Frame Control structure
    lr11xx_wifi_mac_address_t mac_address_1;
    lr11xx_wifi_mac_address_t mac_address_2;
    lr11xx_wifi_mac_address_t mac_address_3;
    uint64_t                  timestamp_us;  //!< Indicate the up-time of the Access Point
                                             //!< transmitting the Beacon [us]
    uint16_t beacon_period_tu;
    uint16_t seq_control;  //!< Sequence Control value
} lr11xx_wifi_extended_beacon_period_result_t;

typedef enum
{
    LR11XX_WIFI_RESULT_FORMAT_BASIC_MISC,
    LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_ONLY,
    LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_RSSI,
    LR11XX_WIFI_RESULT_FORMAT_EXTENDED_MISC,
    LR11XX_WIFI_RESULT_FORMAT_EXTENDED_PERIOD_BEACON,
} lr11xx_wifi_alpha_result_format_t;

/*!
 * @brief WiFi capture mode
 */
typedef enum
{
    LR11XX_WIFI_SCAN_ALPHA_MODE_BEACON = 1,             //!< Only scan for Beacons and Probe Response Access Points' MAC
                                                        //!< addresses until Period Beacon field (Basic result)
    LR11XX_WIFI_SCAN_ALPHA_MODE_BEACON_AND_PACKET = 2,  //!< Scan for beacons Access
                                                        //!< Points' MAC addresses until Period Beacon field, and for
                                                        //!< Packets until third Mac Address field (Basic result)
    LR11XX_WIFI_SCAN_ALPHA_MODE_FULL_MAC =
        3,  //!< Scan up to the three MAC addresses from
            //!< Beacons and Probe Responses or until third MAC address for packet (Extended result but all fields
            //!< except common and the three MAC addresses will be null)
} lr11xx_wifi_alpha_mode_t;

/*!
 * @brief Enable/Disable usage of hardware de-barker
 *
 * Hardware de-barker is used by the chip only in Wi-Fi type B passive scan. Using it dramatically reduces the passive
 * scan time. It must be enabled for country code search operations. It is enabled by default.
 *
 * When country code search operation or LR11XX_WIFI_SCAN_MODE_FULL_BEACON acquisition mode is used, the
 * hardware debarker is silently enabled independently on previous call of this command. In that situation it is then
 * reset to its previous state.
 *
 * @warning If the hardware debarker is disabled and a scan with acquisition mode @ref LR11XX_WIFI_SCAN_MODE_FULL_BEACON
 * is attempted (this includes scan country code), then the hardware debarker is silently used. Note however that the
 * hardware debarker is not re-enabled, ie. next call to scan that is not @ref LR11XX_WIFI_SCAN_MODE_FULL_BEACON, then
 * the hardware debarker is not used.
 *
 * @param [in] context Chip implementation context
 * @param [in] enable_hardware_debarker Set to true to enable usage of hardware de-barker, false to disable
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_wifi_cfg_hardware_debarker( const void* context, const bool enable_hardware_debarker );

/*!
 * @brief Read basic misc results
 */
void lr11xx_wifi_read_basic_misc_results( const void* radio, const uint8_t start_result_index, const uint8_t nb_results,
                                          lr11xx_wifi_basic_misc_result_t* results );

/*!
 * @brief Read basic MAC only results
 */
void lr11xx_wifi_read_basic_mac_only_results( const void* radio, const uint8_t start_result_index,
                                              const uint8_t nb_results, lr11xx_wifi_basic_mac_only_result_t* results );

/*!
 * @brief Read basic MAC and RSSI results
 */
void lr11xx_wifi_read_basic_mac_rssi_results( const void* radio, const uint8_t start_result_index,
                                              const uint8_t nb_results, lr11xx_wifi_basic_mac_rssi_result_t* results );

/*!
 * @brief Read extended misc results
 */
void lr11xx_wifi_read_extended_misc_results( const void* radio, const uint8_t start_result_index,
                                             const uint8_t nb_results, lr11xx_wifi_extended_misc_result_t* results );

/*!
 * @brief Read extended with beacon period results
 */
void lr11xx_wifi_read_extended_beacon_period_results( const void* radio, const uint8_t start_result_index,
                                                      const uint8_t                                nb_results,
                                                      lr11xx_wifi_extended_beacon_period_result_t* results );

#ifdef __cplusplus
}
#endif

#endif  // __LR11XX_WIFI_ALPHA_DRIVER_H__