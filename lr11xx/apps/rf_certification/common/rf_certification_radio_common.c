/*!
 * @file      rf_certification_radio_common.c
 *
 * @brief     RF certification common radio functions for LR11xx chip
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
#include <stdint.h>

#include "apps_common.h"
#include "rf_certification_radio_common.h"
#include "apps_utilities.h"
#include "lr11xx_system.h"
#include "lr11xx_radio.h"
#include "../main_rf_certification.h"
#include "smtc_dbpsk.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_dbg_trace.h"
#include "uart_init.h"
#include "smtc_hal_button.h"

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

static lr11xx_radio_mod_params_lora_t lora_mod_params = {
    .sf   = LORA_SPREADING_FACTOR,
    .bw   = LORA_BANDWIDTH,
    .cr   = LORA_CODING_RATE,
    .ldro = 0  // Will be initialized in radio init
};

static lr11xx_radio_pkt_params_lora_t lora_pkt_params = {
    .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,
    .header_type          = LORA_PKT_LEN_MODE,
    .pld_len_in_bytes     = PAYLOAD_LENGTH,
    .crc                  = LORA_CRC,
    .iq                   = LORA_IQ,
};

static lr11xx_radio_mod_params_gfsk_t gfsk_mod_params = {
    .br_in_bps    = FSK_BITRATE,
    .pulse_shape  = FSK_PULSE_SHAPE,
    .bw_dsb_param = FSK_BANDWIDTH,
    .fdev_in_hz   = FSK_FDEV,
};

static lr11xx_radio_pkt_params_gfsk_t gfsk_pkt_params = {
    .preamble_len_in_bits  = FSK_PREAMBLE_LENGTH,
    .preamble_detector     = FSK_PREAMBLE_DETECTOR,
    .sync_word_len_in_bits = FSK_SYNCWORD_LENGTH,
    .address_filtering     = FSK_ADDRESS_FILTERING,
    .header_type           = FSK_HEADER_TYPE,
    .pld_len_in_bytes      = PAYLOAD_LENGTH,
    .crc_type              = FSK_CRC_TYPE,
    .dc_free               = FSK_DC_FREE,
};

static const lr11xx_radio_mod_params_bpsk_t bpsk_mod_params = {
    .br_in_bps   = BPSK_BITRATE_IN_BPS,
    .pulse_shape = LR11XX_RADIO_DBPSK_PULSE_SHAPE,
};

static lr11xx_radio_pkt_params_bpsk_t bpsk_pkt_params = {
    .pld_len_in_bytes = 0,  // Will be initialized in radio init
    .ramp_up_delay    = 0,
    .ramp_down_delay  = 0,
    .pld_len_in_bits  = 0,  // Will be initialized in radio init
};

static uint32_t                      rf_freq_in_hz                 = RF_FREQ_IN_HZ;
static lr11xx_radio_pkt_type_t       rf_packet_type                = PACKET_TYPE;
static int8_t                        tx_expected_output_pwr_in_dbm = TX_OUTPUT_POWER_DBM;
static uint16_t                      rf_payload_length             = PAYLOAD_LENGTH;
static uint16_t                      rf_bpsk_bitrate               = BPSK_BITRATE_IN_BPS;
static uint8_t                       lora_preamble_len             = LORA_PREAMBLE_LENGTH;
static lr11xx_radio_fallback_modes_t radio_fallback_mode           = LR11XX_RADIO_FALLBACK_STDBY_XOSC;
static bool                          use_custom_gfsk_syncword      = false;
static uint8_t                       gfsk_custom_syncword[LR11XX_RADIO_GFSK_SYNC_WORD_LENGTH];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DECLARATIONS -------------------------------------------
 */

/*!
 * @brief Print the current RF configuration on the debug interface
 */
static void print_current_rf_configuration( void );

/*!
 * @brief Print the current RF LoRa configuration on the debug interface
 */
static void print_current_rf_lora_configuration( void );

/*!
 * @brief Print the current RF GFSK configuration on the debug interface
 */
static void print_current_rf_gfsk_configuration( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION DEFINITIONS --------------------------------------------
 */

void rf_certification_common_radio_lr11xx_radio_init( const void* context )
{
    smtc_shield_lr11xx_t*                  shield = apps_common_lr11xx_get_shield( );
    const smtc_shield_lr11xx_pa_pwr_cfg_t* pa_pwr_cfg =
        smtc_shield_lr11xx_get_pa_pwr_cfg( shield, rf_freq_in_hz, tx_expected_output_pwr_in_dbm );

    if( pa_pwr_cfg == NULL )
    {
        HAL_DBG_TRACE_ERROR( "Invalid target frequency or power level\n" );
        while( true )
        {
        }
    }

    print_current_rf_configuration( );

    lr11xx_radio_set_pkt_type( context, rf_packet_type );
    lr11xx_radio_set_rf_freq( context, rf_freq_in_hz );
    lr11xx_radio_set_rssi_calibration( context,
                                       smtc_shield_lr11xx_get_rssi_calibration_table( shield, rf_freq_in_hz ) );
    lr11xx_radio_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) );
    lr11xx_radio_set_tx_params( context, pa_pwr_cfg->power, PA_RAMP_TIME );
    lr11xx_radio_set_rx_tx_fallback_mode( context, radio_fallback_mode );
    lr11xx_radio_cfg_rx_boosted( context, ENABLE_RX_BOOST_MODE );

    switch( rf_packet_type )
    {
    case LR11XX_RADIO_PKT_TYPE_LORA:
    {
        print_current_rf_lora_configuration( );

        lora_mod_params.ldro = apps_common_compute_lora_ldro( lora_mod_params.sf, lora_mod_params.bw );
        lr11xx_radio_set_lora_mod_params( context, &lora_mod_params );
        lr11xx_radio_set_lora_pkt_params( context, &lora_pkt_params );
        lr11xx_radio_set_lora_sync_word( context, LORA_SYNCWORD );

        break;
    }
    case LR11XX_RADIO_PKT_TYPE_GFSK:
    {
        print_current_rf_gfsk_configuration( );

        uint8_t* gfsk_sync_word_ptr = ( ( uint8_t* ) &gfsk_sync_word[0] );

        if( use_custom_gfsk_syncword == true )
        {
            gfsk_sync_word_ptr = &gfsk_custom_syncword[0];
        }

        lr11xx_radio_set_gfsk_mod_params( context, &gfsk_mod_params );
        lr11xx_radio_set_gfsk_pkt_params( context, &gfsk_pkt_params );
        lr11xx_radio_set_gfsk_sync_word( context, gfsk_sync_word_ptr );

        if( gfsk_pkt_params.dc_free != LR11XX_RADIO_GFSK_DC_FREE_OFF )
        {
            lr11xx_radio_set_gfsk_whitening_seed( context, FSK_WHITENING_SEED );
        }

        if( gfsk_pkt_params.crc_type != LR11XX_RADIO_GFSK_CRC_OFF )
        {
            lr11xx_radio_set_gfsk_crc_params( context, FSK_CRC_SEED, FSK_CRC_POLYNOMIAL );
        }

        if( FSK_ADDRESS_FILTERING != LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE )
        {
            lr11xx_radio_set_pkt_address( context, FSK_NODE_ADDRESS, FSK_BROADCAST_ADDRESS );
        }
        break;
    }
    case LR11XX_RADIO_PKT_TYPE_LR_FHSS:
    {
        const lr11xx_radio_mod_params_lr_fhss_t mod_lr_fhss = {
            .br_in_bps   = LR11XX_RADIO_LR_FHSS_BITRATE_488_BPS,
            .pulse_shape = LR11XX_RADIO_LR_FHSS_PULSE_SHAPE_BT_1,
        };

        lr11xx_radio_set_lr_fhss_mod_params( context, &mod_lr_fhss );
        break;
    }
    case LR11XX_RADIO_PKT_TYPE_BPSK:
    {
        bpsk_pkt_params.pld_len_in_bytes = smtc_dbpsk_get_pld_len_in_bytes( rf_payload_length << 3 );
        bpsk_pkt_params.pld_len_in_bits  = smtc_dbpsk_get_pld_len_in_bits( rf_payload_length << 3 );

        if( rf_bpsk_bitrate == 100 )
        {
            bpsk_pkt_params.ramp_up_delay   = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_100_BPS;
            bpsk_pkt_params.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS;
        }
        else if( rf_bpsk_bitrate == 600 )
        {
            bpsk_pkt_params.ramp_up_delay   = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_600_BPS;
            bpsk_pkt_params.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_600_BPS;
        }
        else
        {
            bpsk_pkt_params.ramp_up_delay   = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_DEFAULT;
            bpsk_pkt_params.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_DEFAULT;
        }

        ASSERT_LR11XX_RC( lr11xx_radio_set_bpsk_pkt_params( context, &bpsk_pkt_params ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_bpsk_mod_params( context, &bpsk_mod_params ) );

        break;
    }
    default:
        break;
    }
}

uint32_t rf_certification_common_radio_lr11xx_get_time_on_air_in_ms( void )
{
    switch( rf_packet_type )
    {
    case LR11XX_RADIO_PKT_TYPE_LORA:
    {
        return lr11xx_radio_get_lora_time_on_air_in_ms( &lora_pkt_params, &lora_mod_params );
    }
    case LR11XX_RADIO_PKT_TYPE_GFSK:
    {
        return lr11xx_radio_get_gfsk_time_on_air_in_ms( &gfsk_pkt_params, &gfsk_mod_params );
    }
    default:
    {
        return 0;
    }
    }
}

void rf_certification_common_radio_lr11xx_config_working_params(
    uint32_t freq_in_hz, const lr11xx_radio_lora_sf_t spreading_factor, const int bandwidth, const uint16_t packet_type,
    int8_t expected_output_pwr_in_dbm, uint32_t bitrate, uint32_t fdev, uint16_t payload_length, uint16_t preamble_len,
    lr11xx_radio_fallback_modes_t fallback_mode )
{
    rf_certification_common_radio_lr11xx_set_common_params( freq_in_hz, packet_type, expected_output_pwr_in_dbm,
                                                            payload_length, fallback_mode );
    switch( rf_packet_type )
    {
    case LR11XX_RADIO_PKT_TYPE_LORA:
        rf_certification_common_radio_lr11xx_set_lora_parameters( spreading_factor,
                                                                  ( lr11xx_radio_lora_bw_t ) bandwidth, preamble_len );
        break;
    case LR11XX_RADIO_PKT_TYPE_GFSK:
        rf_certification_common_radio_lr11xx_set_gfsk_parameters( bitrate, ( lr11xx_radio_gfsk_bw_t ) bandwidth, fdev );
        break;
    case LR11XX_RADIO_PKT_TYPE_BPSK:
        rf_certification_common_radio_lr11xx_set_bpsk_parameters( bitrate );
        break;
    case LR11XX_RADIO_PKT_TYPE_LR_FHSS:
        break;
    default:
        HAL_DBG_TRACE_ERROR( "Invalid packet type\n" );
        break;
    }
}

void rf_certification_common_radio_lr11xx_set_gfsk_lorawan_compliant_params( void )
{
    gfsk_pkt_params.dc_free  = LR11XX_RADIO_GFSK_DC_FREE_WHITENING;
    gfsk_pkt_params.crc_type = LR11XX_RADIO_GFSK_CRC_2_BYTES;

    gfsk_mod_params.pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1;

    uint8_t syncword[8] = { 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 };
    rf_certification_common_radio_lr11xx_set_gfsk_custom_syncword( syncword, 24 );
}

void rf_certification_common_radio_lr11xx_set_gfsk_custom_syncword( uint8_t* syncword, uint8_t len_in_bits )
{
    use_custom_gfsk_syncword              = true;
    gfsk_pkt_params.sync_word_len_in_bits = len_in_bits;
    memcpy( gfsk_custom_syncword, syncword, LR11XX_RADIO_GFSK_SYNC_WORD_LENGTH );
}

void rf_certification_common_radio_lr11xx_set_lora_parameters( const lr11xx_radio_lora_sf_t spreading_factor,
                                                               const lr11xx_radio_lora_bw_t bandwidth,
                                                               uint16_t                     preamble_len )
{
    lora_pkt_params.preamble_len_in_symb = preamble_len;
    lora_mod_params.sf                   = spreading_factor;
    lora_mod_params.bw                   = bandwidth;
    lora_pkt_params.pld_len_in_bytes     = rf_payload_length;
    lora_pkt_params.crc                  = LR11XX_RADIO_LORA_CRC_ON;
    lora_pkt_params.iq                   = LR11XX_RADIO_LORA_IQ_INVERTED;
}

void rf_certification_common_radio_lr11xx_set_gfsk_parameters( const uint32_t bitrate, lr11xx_radio_gfsk_bw_t bandwidth,
                                                               const uint32_t fdev_hz )
{
    gfsk_mod_params.br_in_bps        = bitrate;
    gfsk_mod_params.bw_dsb_param     = bandwidth;
    gfsk_mod_params.fdev_in_hz       = fdev_hz;
    gfsk_pkt_params.pld_len_in_bytes = rf_payload_length;
}

void rf_certification_common_radio_lr11xx_set_bpsk_parameters( const uint16_t bitrate )
{
    rf_bpsk_bitrate = bitrate;
}

void rf_certification_common_radio_lr11xx_set_common_params( const uint32_t                freq_in_hz,
                                                             const lr11xx_radio_pkt_type_t packet_type,
                                                             const int8_t                  expected_output_pwr_in_dbm,
                                                             const uint16_t                payload_length,
                                                             lr11xx_radio_fallback_modes_t fallback_mode )
{
    rf_freq_in_hz                 = freq_in_hz;
    rf_packet_type                = packet_type;
    tx_expected_output_pwr_in_dbm = expected_output_pwr_in_dbm;
    rf_payload_length             = payload_length;
    radio_fallback_mode           = fallback_mode;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static void print_current_rf_configuration( void )
{
    HAL_DBG_TRACE_INFO( "Common parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Packet type   = %s\n", lr11xx_radio_pkt_type_to_str( rf_packet_type ) );
    HAL_DBG_TRACE_INFO( "   RF frequency  = %u Hz\n", rf_freq_in_hz );
    HAL_DBG_TRACE_INFO( "   Output power  = %i dBm\n", tx_expected_output_pwr_in_dbm );
    HAL_DBG_TRACE_INFO( "   Fallback mode = %s\n", lr11xx_radio_fallback_modes_to_str( radio_fallback_mode ) );
    HAL_DBG_TRACE_INFO( ( ENABLE_RX_BOOST_MODE == true ) ? "   Rx boost activated\n" : "   Rx boost deactivated\n" );
    HAL_DBG_TRACE_PRINTF( "\n" );
}

static void print_current_rf_lora_configuration( void )
{
    HAL_DBG_TRACE_INFO( "LoRa modulation parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Spreading factor = %s\n", lr11xx_radio_lora_sf_to_str( lora_mod_params.sf ) );
    HAL_DBG_TRACE_INFO( "   Bandwidth        = %s\n", lr11xx_radio_lora_bw_to_str( lora_mod_params.bw ) );
    HAL_DBG_TRACE_INFO( "   Coding rate      = %s\n", lr11xx_radio_lora_cr_to_str( lora_mod_params.cr ) );
    HAL_DBG_TRACE_PRINTF( "\n" );

    HAL_DBG_TRACE_INFO( "LoRa packet parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Preamble length = %d symbol(s)\n", lora_preamble_len );
    HAL_DBG_TRACE_INFO( "   Header mode     = %s\n", lr11xx_radio_lora_pkt_len_modes_to_str( LORA_PKT_LEN_MODE ) );
    HAL_DBG_TRACE_INFO( "   Payload length  = %d byte(s)\n", lora_pkt_params.pld_len_in_bytes );
    HAL_DBG_TRACE_INFO( "   CRC mode        = %s\n", lr11xx_radio_lora_crc_to_str( LORA_CRC ) );
    HAL_DBG_TRACE_INFO( "   IQ              = %s\n", lr11xx_radio_lora_iq_to_str( LORA_IQ ) );
    HAL_DBG_TRACE_PRINTF( "\n" );

    HAL_DBG_TRACE_INFO( "LoRa syncword = 0x%02X\n", LORA_SYNCWORD );
    HAL_DBG_TRACE_PRINTF( "\n" );
}

static void print_current_rf_gfsk_configuration( void )
{
    HAL_DBG_TRACE_INFO( "GFSK modulation parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Bitrate             = %u bps\n", gfsk_mod_params.br_in_bps );
    HAL_DBG_TRACE_INFO( "   Pulse shape         = %s\n",
                        lr11xx_radio_gfsk_pulse_shape_to_str( gfsk_mod_params.pulse_shape ) );
    HAL_DBG_TRACE_INFO( "   Bandwidth           = %s\n", lr11xx_radio_gfsk_bw_to_str( gfsk_mod_params.bw_dsb_param ) );
    HAL_DBG_TRACE_INFO( "   Frequency deviation = %u Hz\n", gfsk_mod_params.fdev_in_hz );
    HAL_DBG_TRACE_PRINTF( "\n" );

    HAL_DBG_TRACE_INFO( "GFSK packet parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Preamble length   = %d bit(s)\n", FSK_PREAMBLE_LENGTH );
    HAL_DBG_TRACE_INFO( "   Preamble detector = %s\n",
                        lr11xx_radio_gfsk_preamble_detector_to_str( FSK_PREAMBLE_DETECTOR ) );
    HAL_DBG_TRACE_INFO( "   Syncword length   = %d bit(s)\n", gfsk_pkt_params.sync_word_len_in_bits );
    HAL_DBG_TRACE_INFO( "   Address filtering = %s\n",
                        lr11xx_radio_gfsk_address_filtering_to_str( FSK_ADDRESS_FILTERING ) );
    if( FSK_ADDRESS_FILTERING != LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE )
    {
        HAL_DBG_TRACE_INFO( "     (Node address      = 0x%02X)\n", FSK_NODE_ADDRESS );
        if( FSK_ADDRESS_FILTERING == LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES )
        {
            HAL_DBG_TRACE_INFO( "     (Broadcast address = 0x%02X)\n", FSK_BROADCAST_ADDRESS );
        }
    }
    HAL_DBG_TRACE_INFO( "   Header mode       = %s\n", lr11xx_radio_gfsk_pkt_len_modes_to_str( FSK_HEADER_TYPE ) );
    HAL_DBG_TRACE_INFO( "   Payload length    = %d byte(s)\n", gfsk_pkt_params.pld_len_in_bytes );
    HAL_DBG_TRACE_INFO( "   CRC mode          = %s\n", lr11xx_radio_gfsk_crc_type_to_str( gfsk_pkt_params.crc_type ) );
    if( gfsk_pkt_params.crc_type != LR11XX_RADIO_GFSK_CRC_OFF )
    {
        HAL_DBG_TRACE_INFO( "     (CRC seed       = 0x%08X)\n", FSK_CRC_SEED );
        HAL_DBG_TRACE_INFO( "     (CRC polynomial = 0x%08X)\n", FSK_CRC_POLYNOMIAL );
    }
    HAL_DBG_TRACE_INFO( "   DC free           = %s\n", lr11xx_radio_gfsk_dc_free_to_str( gfsk_pkt_params.dc_free ) );
    if( gfsk_pkt_params.dc_free != LR11XX_RADIO_GFSK_DC_FREE_OFF )
    {
        HAL_DBG_TRACE_INFO( "     (Whitening seed = 0x%04X)\n", FSK_WHITENING_SEED );
    }
    HAL_DBG_TRACE_PRINTF( "\n" );
}

/* --- EOF ------------------------------------------------------------------ */
