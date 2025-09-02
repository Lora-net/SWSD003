/*!
 * @file      rf_certification_radio_common.h
 *
 * @brief     RF certification common radio functions for SX126X chip
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

#ifndef RF_CERTIFICATION_RADIO_COMMON_H
#define RF_CERTIFICATION_RADIO_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/**
 * @brief Configure the correct parameters to receive notifications in ETSI territories (Europe)
 *
 * @param len: the length of the string to send (if it has to be received, set 0)
 * @return The status of the operation
 */
#define CONFIGURE_WORKING_PARAMS_NOTIFY_TX_RX_EU( len )                                                               \
    rf_certification_common_radio_sx126x_config_working_params( 867300000u, SX126X_LORA_SF7, SX126X_LORA_BW_125,      \
                                                                SX126X_PKT_TYPE_LORA, EXPECTED_PWR_14_DBM, 0, 0, len, \
                                                                LORA_PREAMBLE_LENGTH, SX126X_FALLBACK_STDBY_XOSC )

/**
 * @brief Configure the correct parameters to receive notifications in FCC territories (North america)
 *
 * @param len: the length of the string to send (if it has to be received, set 0)
 * @return The status of the operation
 */
#define CONFIGURE_WORKING_PARAMS_NOTIFY_TX_RX_NA( len )                                                               \
    rf_certification_common_radio_sx126x_config_working_params( 902300000u, SX126X_LORA_SF7, SX126X_LORA_BW_125,      \
                                                                SX126X_PKT_TYPE_LORA, EXPECTED_PWR_22_DBM, 0, 0, len, \
                                                                LORA_PREAMBLE_LENGTH, SX126X_FALLBACK_STDBY_XOSC )

/**
 * @brief Configure the correct parameters to receive notifications in ARIB territories (Japan)
 *
 * @param len: the length of the string to send (if it has to be received, 0 is set)
 * @return The status of the operation
 */
#define CONFIGURE_WORKING_PARAMS_NOTIFY_TX_RX_JP( len )                                                               \
    rf_certification_common_radio_sx126x_config_working_params( 921000000u, SX126X_LORA_SF7, SX126X_LORA_BW_125,      \
                                                                SX126X_PKT_TYPE_LORA, EXPECTED_PWR_10_DBM, 0, 0, len, \
                                                                LORA_PREAMBLE_LENGTH, SX126X_FALLBACK_STDBY_XOSC )

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
 * --- PUBLIC FUNCTION PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Calculates the current time on air based on the current params (lora or gfsk only)
 *
 * @return the calculated on air time in milliseconds
 */
uint32_t rf_certification_common_radio_sx126x_get_time_on_air_in_ms( void );

/*!
 * @brief Initialize the radio configuration of the transceiver
 *
 * @param [in] context  Pointer to the radio context
 */
void rf_certification_common_radio_sx126x_radio_init( const void* context );

/*!
 * @brief Initialize the radio configuration of the transceiver with the input parameters
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] freq_in_hz  Frequency in hertz
 * @param [in] spreading_factor  Spreading factor (lora only)
 * @param [in] bandwidth  Bandwidth (lora or gfsk)
 * @param [in] packet_type  The packet type (lora or gfsk)
 * @param [in] expected_output_pwr_in_dbm  Expected output power in dBm
 * @param [in] bitrate  Bitrate in bps (gfsk or bpsk)
 * @param [in] fdev  Frequency deviation (gfsk only)
 * @param [in] payload_length  Payload length in bytes
 * @param [in] preamble_len  Preamble length in bytes
 * @param [in] fallback_mode  TX-RX fallback mode
 */
void rf_certification_common_radio_sx126x_config_working_params(
    uint32_t freq_in_hz, const sx126x_lora_sf_t spreading_factor, const int bandwidth, const uint16_t packet_type,
    int8_t expected_output_pwr_in_dbm, uint32_t bitrate, uint32_t fdev, uint16_t payload_length, uint16_t preamble_len,
    sx126x_fallback_modes_t fallback_mode );

/*!
 * @brief Sets the gfsk parameters (crc, whitening, syncword) to be compliant with LoRaWAN specs.
 */
void rf_certification_common_radio_sx126x_set_gfsk_lorawan_compliant_params( void );

/*!
 * @brief Sets the syncword to be used in gfsk transmissions. Maximum syncword length supported by this function is 8
 * bytes.
 *
 * @param [in] syncword Buffer containing the syncword
 * @param [in] len_in_bits Meaningful length of the syncwork in bytes
 */
void rf_certification_common_radio_sx126x_set_gfsk_custom_syncword( uint8_t* syncword, uint8_t len_in_bits );

/*!
 * @brief Initialize the lora parameters of the radio. Call this before the radio init.
 *
 * @param [in] spreading_factor  Lora spreading factor.
 * @param [in] bandwidth  Lora bandwidth
 * @param [in] preamble_len  Lora preamble length
 */
void rf_certification_common_radio_sx126x_set_lora_parameters( const sx126x_lora_sf_t spreading_factor,
                                                               const sx126x_lora_bw_t bandwidth,
                                                               uint16_t               preamble_len );

/*!
 * @brief Sets the standby mode to be used.
 *
 * @param [in] mode  The standby mode to use.
 */
void rf_certification_common_radio_sx126x_set_standy_mode( sx126x_standby_cfgs_t mode );

/*!
 * @brief Initialize the gfsk parameters of the radio. Call this before the radio init.
 *
 * @param [in] bitrate  Bitrate in bit per second.
 * @param [in] bandwidth  Gfsk bandwidth
 * @param [in] fdev_hz  Frequency deviation in hertz
 */
void rf_certification_common_radio_sx126x_set_gfsk_parameters( const uint32_t bitrate, sx126x_gfsk_bw_t bandwidth,
                                                               const uint32_t fdev_hz );

/*!
 * @brief Initialize the common parameters of the radio. Call this before the radio init.
 *
 * @param [in] bitrate  Bitrate in bit per second.
 */
void rf_certification_common_radio_sx126x_set_bpsk_parameters( const uint16_t bitrate );

/*!
 * @brief Sets the PA ramp time
 *
 * @param [in] ramp_time  Power amplifier ramp time
 */
void rf_certification_common_radio_sx126x_set_pa_ramp_time( sx126x_ramp_time_t ramp_time );

/*!
 * @brief Initialize the common parameters of the radio. Call this before the radio init.
 *
 * @param [in] freq_in_hz  Frequency in hertz
 * @param [in] packet_type  Packet type
 * @param [in] expected_output_pwr_in_dbm  Expected output power in dbm
 * @param [in] payload_length  Length of the payload
 * @param [in] fallback_mode  TX-RX fallback mode
 */
void rf_certification_common_radio_sx126x_set_common_params( const uint32_t          freq_in_hz,
                                                             const sx126x_pkt_type_t packet_type,
                                                             const int8_t            expected_output_pwr_in_dbm,
                                                             const uint16_t          payload_length,
                                                             sx126x_fallback_modes_t fallback_mode );

#ifdef __cplusplus
}
#endif

#endif /* RF_CERTIFICATION_RADIO_COMMON_H */

/* --- EOF ------------------------------------------------------------------ */
