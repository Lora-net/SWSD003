/*!
 * @file      apps_common.c
 *
 * @brief     Common functions shared by the examples
 *
 * @copyright
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "apps_common.h"
#include "common_version.h"
#include "apps_utilities.h"
#include "sx126x_str.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_shield_pinout_mapping.h"
#include "smtc_shield_sx126x.h"

#include "smtc_shield_sx1261mb1bas.h"
#include "smtc_shield_sx1261mb1cas.h"
#include "smtc_shield_sx1261mb2bas.h"
#include "smtc_shield_sx1262mb1cas.h"
#include "smtc_shield_sx1262mb1cbs.h"
#include "smtc_shield_sx1262mb1das.h"
#include "smtc_shield_sx1262mb1pas.h"
#include "smtc_shield_sx1262mb2cas.h"
#include "smtc_shield_sx1268mb1gas.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#ifdef SX1261MB1BAS
smtc_shield_sx126x_t shield = SMTC_SHIELD_SX1261MB1BAS_INSTANTIATE;
#endif
#ifdef SX1261MB1CAS
smtc_shield_sx126x_t shield = SMTC_SHIELD_SX1261MB1CAS_INSTANTIATE;
#endif
#ifdef SX1261MB2BAS
smtc_shield_sx126x_t shield = SMTC_SHIELD_SX1261MB2BAS_INSTANTIATE;
#endif
#ifdef SX1262MB1CAS
smtc_shield_sx126x_t shield = SMTC_SHIELD_SX1262MB1CAS_INSTANTIATE;
#endif
#ifdef SX1262MB1CBS
smtc_shield_sx126x_t shield = SMTC_SHIELD_SX1262MB1CBS_INSTANTIATE;
#endif
#ifdef SX1262MB1DAS
smtc_shield_sx126x_t shield = SMTC_SHIELD_SX1262MB1DAS_INSTANTIATE;
#endif
#ifdef SX1262MB1PAS
smtc_shield_sx126x_t shield = SMTC_SHIELD_SX1262MB1PAS_INSTANTIATE;
#endif
#ifdef SX1262MB2CAS
smtc_shield_sx126x_t shield = SMTC_SHIELD_SX1262MB2CAS_INSTANTIATE;
#endif
#ifdef SX1268MB1GAS
smtc_shield_sx126x_t shield = SMTC_SHIELD_SX1268MB1GAS_INSTANTIATE;
#endif

static sx126x_mod_params_lora_t lora_mod_params = {
    .sf   = LORA_SPREADING_FACTOR,
    .bw   = LORA_BANDWIDTH,
    .cr   = LORA_CODING_RATE,
    .ldro = 0,  // Will be initialized during radio init
};

const sx126x_pkt_params_lora_t lora_pkt_params = {
    .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,
    .header_type          = LORA_PKT_LEN_MODE,
    .pld_len_in_bytes     = PAYLOAD_LENGTH,
    .crc_is_on            = LORA_CRC,
    .invert_iq_is_on      = LORA_IQ,
};

const sx126x_mod_params_gfsk_t gfsk_mod_params = {
    .br_in_bps    = FSK_BITRATE,
    .fdev_in_hz   = FSK_FDEV,
    .pulse_shape  = FSK_PULSE_SHAPE,
    .bw_dsb_param = FSK_BANDWIDTH,
};

const sx126x_pkt_params_gfsk_t gfsk_pkt_params = {
    .preamble_len_in_bits  = FSK_PREAMBLE_LENGTH,
    .preamble_detector     = FSK_PREAMBLE_DETECTOR,
    .sync_word_len_in_bits = FSK_SYNCWORD_LENGTH,
    .address_filtering     = FSK_ADDRESS_FILTERING,
    .header_type           = FSK_HEADER_TYPE,
    .pld_len_in_bytes      = PAYLOAD_LENGTH,
    .crc_type              = FSK_CRC_TYPE,
    .dc_free               = FSK_DC_FREE,
};
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static sx126x_hal_context_t context;

static volatile bool irq_fired = false;

static const smtc_shield_sx126x_pinout_t* shield_pinout = 0;

struct
{
    smtc_hal_mcu_gpio_cfg_t        cfg;
    smtc_hal_mcu_gpio_output_cfg_t cfg_output;
    smtc_hal_mcu_gpio_inst_t       inst;
} led_tx;

struct
{
    smtc_hal_mcu_gpio_cfg_t        cfg;
    smtc_hal_mcu_gpio_output_cfg_t cfg_output;
    smtc_hal_mcu_gpio_inst_t       inst;
} led_rx;

struct
{
    smtc_hal_mcu_gpio_cfg_t        cfg;
    smtc_hal_mcu_gpio_output_cfg_t cfg_output;
    smtc_hal_mcu_gpio_inst_t       inst;
} antenna_sw;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
void radio_on_dio_irq( void* context );
void on_tx_done( void ) __attribute__( ( weak ) );
void on_rx_done( void ) __attribute__( ( weak ) );
void on_preamble_detected( void ) __attribute__( ( weak ) );
void on_syncword_valid( void ) __attribute__( ( weak ) );
void on_header_valid( ) __attribute__( ( weak ) );
void on_header_error( void ) __attribute__( ( weak ) );
void on_crc_error( void ) __attribute__( ( weak ) );
void on_rx_timeout( void ) __attribute__( ( weak ) );
void on_rx_error( void ) __attribute__( ( weak ) );
void on_cad_done_undetected( void ) __attribute__( ( weak ) );
void on_cad_done_detected( void ) __attribute__( ( weak ) );
void on_fhss_hop_done( void ) __attribute__( ( weak ) );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

sx126x_hal_context_t* apps_common_sx126x_get_context( )
{
    context.busy.cfg                 = smtc_shield_pinout_mapping_get_gpio_cfg( SMTC_SHIELD_PINOUT_D3 );
    context.busy.cfg_input.pull_mode = SMTC_HAL_MCU_GPIO_PULL_MODE_NONE;
    context.busy.cfg_input.irq_mode  = SMTC_HAL_MCU_GPIO_IRQ_MODE_OFF;
    context.busy.cfg_input.callback  = NULL;

    context.irq.cfg                 = smtc_shield_pinout_mapping_get_gpio_cfg( SMTC_SHIELD_PINOUT_D5 );
    context.irq.cfg_input.pull_mode = SMTC_HAL_MCU_GPIO_PULL_MODE_NONE;
    context.irq.cfg_input.irq_mode  = SMTC_HAL_MCU_GPIO_IRQ_MODE_RISING;
    context.irq.cfg_input.callback  = radio_on_dio_irq;
    context.irq.cfg_input.context   = NULL;

    context.nss.cfg                      = smtc_shield_pinout_mapping_get_gpio_cfg( SMTC_SHIELD_PINOUT_D7 );
    context.nss.cfg_output.initial_state = SMTC_HAL_MCU_GPIO_STATE_HIGH;
    context.nss.cfg_output.mode          = SMTC_HAL_MCU_GPIO_OUTPUT_MODE_PUSH_PULL;

    context.reset.cfg                      = smtc_shield_pinout_mapping_get_gpio_cfg( SMTC_SHIELD_PINOUT_A0 );
    context.reset.cfg_output.initial_state = SMTC_HAL_MCU_GPIO_STATE_HIGH;
    context.reset.cfg_output.mode          = SMTC_HAL_MCU_GPIO_OUTPUT_MODE_PUSH_PULL;

    context.spi.cfg.spi = SPI1;

    smtc_hal_mcu_gpio_init_input( context.busy.cfg, &( context.busy.cfg_input ), &( context.busy.inst ) );
    smtc_hal_mcu_gpio_init_input( context.irq.cfg, &( context.irq.cfg_input ), &( context.irq.inst ) );
    smtc_hal_mcu_gpio_init_output( context.nss.cfg, &( context.nss.cfg_output ), &( context.nss.inst ) );
    smtc_hal_mcu_gpio_init_output( context.reset.cfg, &( context.reset.cfg_output ), &( context.reset.inst ) );

    smtc_hal_mcu_gpio_enable_irq( context.irq.inst );

    smtc_hal_mcu_spi_init( &( context.spi.cfg ), &( context.spi.inst ) );

    return &context;
}

void apps_common_shield_init( void )
{
    shield_pinout = smtc_shield_sx126x_get_pinout( &shield );

    if( shield_pinout->led_tx != SMTC_SHIELD_PINOUT_NONE )
    {
        led_tx.cfg                      = smtc_shield_pinout_mapping_get_gpio_cfg( shield_pinout->led_tx );
        led_tx.cfg_output.initial_state = SMTC_HAL_MCU_GPIO_STATE_LOW;
        led_tx.cfg_output.mode          = SMTC_HAL_MCU_GPIO_OUTPUT_MODE_PUSH_PULL;

        smtc_hal_mcu_gpio_init_output( led_tx.cfg, &( led_tx.cfg_output ), &( led_tx.inst ) );
    }

    if( shield_pinout->led_rx != SMTC_SHIELD_PINOUT_NONE )
    {
        led_rx.cfg                      = smtc_shield_pinout_mapping_get_gpio_cfg( shield_pinout->led_rx );
        led_rx.cfg_output.initial_state = SMTC_HAL_MCU_GPIO_STATE_LOW;
        led_rx.cfg_output.mode          = SMTC_HAL_MCU_GPIO_OUTPUT_MODE_PUSH_PULL;

        smtc_hal_mcu_gpio_init_output( led_rx.cfg, &( led_rx.cfg_output ), &( led_rx.inst ) );
    }

    if( shield_pinout->antenna_sw != SMTC_SHIELD_PINOUT_NONE )
    {
        antenna_sw.cfg                      = smtc_shield_pinout_mapping_get_gpio_cfg( shield_pinout->antenna_sw );
        antenna_sw.cfg_output.initial_state = SMTC_HAL_MCU_GPIO_STATE_HIGH;
        antenna_sw.cfg_output.mode          = SMTC_HAL_MCU_GPIO_OUTPUT_MODE_PUSH_PULL;

        smtc_hal_mcu_gpio_init_output( antenna_sw.cfg, &( antenna_sw.cfg_output ), &( antenna_sw.inst ) );
    }
}

void apps_common_sx126x_print_config( void )
{
    HAL_DBG_TRACE_INFO( "Common RF parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Packet type     = %s\n", sx126x_pkt_type_to_str( PACKET_TYPE ) );
    HAL_DBG_TRACE_INFO( "   RF Freq         = %d Hz\n", RF_FREQ_IN_HZ );
    HAL_DBG_TRACE_INFO( "   Output power    = %d dBm\n", TX_OUTPUT_POWER_DBM );
    HAL_DBG_TRACE_INFO( "   Fallback mode   = %s\n", sx126x_fallback_modes_to_str( FALLBACK_MODE ) );
    HAL_DBG_TRACE_INFO( "   Rx boost mode %sactivated\n", ( ENABLE_RX_BOOST_MODE ) ? "" : "de" );
    HAL_DBG_TRACE_INFO( "\n" );
    if( PACKET_TYPE == SX126X_PKT_TYPE_LORA )
    {
        HAL_DBG_TRACE_INFO( "LoRa modulation parameters:\n" );
        HAL_DBG_TRACE_INFO( "   Spreading Factor = %s\n", sx126x_lora_sf_to_str( LORA_SPREADING_FACTOR ) );
        HAL_DBG_TRACE_INFO( "   Bandwidth        = %s\n", sx126x_lora_bw_to_str( LORA_BANDWIDTH ) );
        HAL_DBG_TRACE_INFO( "   Coding rate      = %s\n", sx126x_lora_cr_to_str( LORA_CODING_RATE ) );
        HAL_DBG_TRACE_INFO( "\n" );
        HAL_DBG_TRACE_INFO( "LoRa packet parameters:\n" );
        HAL_DBG_TRACE_INFO( "   Preamble length  = %d symbol(s)\n", LORA_PREAMBLE_LENGTH );
        HAL_DBG_TRACE_INFO( "   Header mode      = %s\n", sx126x_lora_pkt_len_modes_to_str( LORA_PKT_LEN_MODE ) );
        HAL_DBG_TRACE_INFO( "   Payload length   = %d\n", PAYLOAD_LENGTH );
        HAL_DBG_TRACE_INFO( "\n" );
        HAL_DBG_TRACE_INFO( "LoRa sync word = 0x%02x\n", LORA_SYNCWORD );
    }
    if( PACKET_TYPE == SX126X_PKT_TYPE_GFSK )
    {
        HAL_DBG_TRACE_INFO( "FSK modulation parameters:\n" );
        HAL_DBG_TRACE_INFO( "    Freq Deviation    = %e Hz\n", FSK_FDEV );
        HAL_DBG_TRACE_INFO( "    Bitrate           = %e bps\n", FSK_BITRATE );
        HAL_DBG_TRACE_INFO( "    Pulse shape       = %s\n", sx126x_gfsk_pulse_shape_to_str( FSK_PULSE_SHAPE ) );
        HAL_DBG_TRACE_INFO( "    Bandwidth         = %s\n", sx126x_gfsk_bw_to_str( FSK_BANDWIDTH ) );
        HAL_DBG_TRACE_INFO( "FSK packet parameters:\n" );
        HAL_DBG_TRACE_INFO( "    Preamble length   = %d bits\n", FSK_PREAMBLE_LENGTH );
        HAL_DBG_TRACE_INFO( "    Preamble detector = %s\n",
                            sx126x_gfsk_preamble_detector_to_str( FSK_PREAMBLE_DETECTOR ) );
        HAL_DBG_TRACE_INFO( "    Syncword length   = %d bits\n", FSK_SYNCWORD_LENGTH );
        HAL_DBG_TRACE_INFO( "    Address filtering = %s\n",
                            sx126x_gfsk_address_filtering_to_str( FSK_ADDRESS_FILTERING ) );
        HAL_DBG_TRACE_INFO( "    Header type       = %s\n", sx126x_gfsk_pkt_len_modes_to_str( FSK_HEADER_TYPE ) );
        HAL_DBG_TRACE_INFO( "    Payload length    = %d\n", PAYLOAD_LENGTH );
        HAL_DBG_TRACE_INFO( "    CRC type          = %s\n", sx126x_gfsk_crc_types_to_str( FSK_CRC_TYPE ) );
        HAL_DBG_TRACE_INFO( "    DC free           = %s\n", sx126x_gfsk_dc_free_to_str( FSK_DC_FREE ) );
    }
}

void apps_common_sx126x_print_version_info( void )
{
    common_version_print( );
    HAL_DBG_TRACE_INFO( "\n" );
}

void apps_common_sx126x_init( const sx126x_hal_context_t* context )
{
    ASSERT_SX126X_RC( sx126x_reset( ( void* ) context ) );

    ASSERT_SX126X_RC( sx126x_init_retention_list( ( void* ) context ) );

    const sx126x_reg_mod_t reg_mode = smtc_shield_sx126x_get_reg_mode( &shield );
    ASSERT_SX126X_RC( sx126x_set_reg_mode( context, reg_mode ) );

    const bool dio2_is_set_as_rf_switch = smtc_shield_sx126x_is_dio2_set_as_rf_switch( &shield );
    ASSERT_SX126X_RC( sx126x_set_dio2_as_rf_sw_ctrl( context, dio2_is_set_as_rf_switch ) );

    const smtc_shield_sx126x_xosc_cfg_t* xosc_cfg = smtc_shield_sx126x_get_xosc_cfg( &shield );
    if( xosc_cfg->tcxo_is_radio_controlled == true )
    {
        ASSERT_SX126X_RC(
            sx126x_set_dio3_as_tcxo_ctrl( context, xosc_cfg->supply_voltage, xosc_cfg->startup_time_in_tick ); )
        ASSERT_SX126X_RC( sx126x_cal( context, SX126X_CAL_ALL ); )
    }
}

void apps_common_sx126x_radio_init( const void* context )
{
    const smtc_shield_sx126x_pa_pwr_cfg_t* pa_pwr_cfg =
        smtc_shield_sx126x_get_pa_pwr_cfg( &shield, RF_FREQ_IN_HZ, TX_OUTPUT_POWER_DBM );

    apps_common_sx126x_print_config( );

    if( pa_pwr_cfg == NULL )
    {
        HAL_DBG_TRACE_ERROR( "Invalid target frequency or power level\n" );
        while( true )
        {
        }
    }
    ASSERT_SX126X_RC( sx126x_set_standby( context, SX126X_STANDBY_CFG_RC ) );
    ASSERT_SX126X_RC( sx126x_set_pkt_type( context, PACKET_TYPE ) );
    ASSERT_SX126X_RC( sx126x_set_rf_freq( context, RF_FREQ_IN_HZ ) );

    ASSERT_SX126X_RC( sx126x_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) ) );
    ASSERT_SX126X_RC( sx126x_set_tx_params( context, pa_pwr_cfg->power, PA_RAMP_TIME ) );

    ASSERT_SX126X_RC( sx126x_set_rx_tx_fallback_mode( context, FALLBACK_MODE ) );
    ASSERT_SX126X_RC( sx126x_cfg_rx_boosted( context, ENABLE_RX_BOOST_MODE ) );

    if( PACKET_TYPE == SX126X_PKT_TYPE_LORA )
    {
        lora_mod_params.ldro = apps_common_compute_lora_ldro( LORA_SPREADING_FACTOR, LORA_BANDWIDTH );
        ASSERT_SX126X_RC( sx126x_set_lora_mod_params( context, &lora_mod_params ) );
        ASSERT_SX126X_RC( sx126x_set_lora_pkt_params( context, &lora_pkt_params ) );
        ASSERT_SX126X_RC( sx126x_set_lora_sync_word( context, LORA_SYNCWORD ) );
    }
    else if( PACKET_TYPE == SX126X_PKT_TYPE_GFSK )
    {
        ASSERT_SX126X_RC( sx126x_set_gfsk_mod_params( context, &gfsk_mod_params ) );
        ASSERT_SX126X_RC( sx126x_set_gfsk_pkt_params( context, &gfsk_pkt_params ) );
        ASSERT_SX126X_RC( sx126x_set_gfsk_sync_word( context, gfsk_sync_word, sizeof( gfsk_sync_word ) ) );

        if( FSK_DC_FREE != SX126X_GFSK_DC_FREE_OFF )
        {
            ASSERT_SX126X_RC( sx126x_set_gfsk_whitening_seed( context, FSK_WHITENING_SEED ) );
        }

        if( FSK_CRC_TYPE != SX126X_GFSK_CRC_OFF )
        {
            ASSERT_SX126X_RC( sx126x_set_gfsk_crc_seed( context, ( uint16_t ) FSK_CRC_SEED ) );
            ASSERT_SX126X_RC( sx126x_set_gfsk_crc_polynomial( context, ( uint16_t ) FSK_CRC_POLYNOMIAL ) );
        }
        if( FSK_ADDRESS_FILTERING != SX126X_GFSK_ADDRESS_FILTERING_DISABLE )
        {
            ASSERT_SX126X_RC( sx126x_set_gfsk_pkt_address( context, FSK_NODE_ADDRESS, FSK_BROADCAST_ADDRESS ) );
        }
    }
}

void apps_common_sx126x_receive( const void* context, uint8_t* buffer, uint8_t* size, uint8_t max_size )
{
    sx126x_rx_buffer_status_t rx_buffer_status;
    sx126x_pkt_status_lora_t  pkt_status_lora;
    sx126x_pkt_status_gfsk_t  pkt_status_gfsk;

    sx126x_get_rx_buffer_status( context, &rx_buffer_status );

    if( max_size < rx_buffer_status.pld_len_in_bytes )
    {
        HAL_DBG_TRACE_ERROR( "Received more bytes than expected (%d vs %d), reception in buffer cancelled.\n",
                             rx_buffer_status.pld_len_in_bytes, max_size );
        *size = 0;
    }
    else
    {
        sx126x_read_buffer( context, 0, buffer, rx_buffer_status.pld_len_in_bytes );
        *size = rx_buffer_status.pld_len_in_bytes;
    }

    HAL_DBG_TRACE_ARRAY( "Packet content", buffer, *size );

    HAL_DBG_TRACE_INFO( "Packet status:\n" );
    if( PACKET_TYPE == SX126X_PKT_TYPE_LORA )
    {
        sx126x_get_lora_pkt_status( context, &pkt_status_lora );
        HAL_DBG_TRACE_INFO( "  - RSSI packet = %i dBm\n", pkt_status_lora.rssi_pkt_in_dbm );
        HAL_DBG_TRACE_INFO( "  - Signal RSSI packet = %i dBm\n", pkt_status_lora.signal_rssi_pkt_in_dbm );
        HAL_DBG_TRACE_INFO( "  - SNR packet = %i dB\n", pkt_status_lora.snr_pkt_in_db );
    }
    else if( PACKET_TYPE == SX126X_PKT_TYPE_GFSK )
    {
        sx126x_get_gfsk_pkt_status( context, &pkt_status_gfsk );
        HAL_DBG_TRACE_INFO( "  - RSSI average = %i dBm\n", pkt_status_gfsk.rssi_avg );
        HAL_DBG_TRACE_INFO( "  - RSSI sync = %i dBm\n", pkt_status_gfsk.rssi_sync );
    }
}

void apps_common_sx126x_irq_process( const void* context )
{
    if( irq_fired == true )
    {
        irq_fired = false;

        sx126x_irq_mask_t irq_regs;
        sx126x_get_and_clear_irq_status( context, &irq_regs );

        if( ( irq_regs & SX126X_IRQ_TX_DONE ) == SX126X_IRQ_TX_DONE )
        {
            HAL_DBG_TRACE_INFO( "Tx done\n" );
            on_tx_done( );
        }

        if( ( irq_regs & SX126X_IRQ_RX_DONE ) == SX126X_IRQ_RX_DONE )
        {
            HAL_DBG_TRACE_INFO( "Rx done\n" );
            ASSERT_SX126X_RC( sx126x_handle_rx_done( context ) );
            if( PACKET_TYPE == SX126X_PKT_TYPE_GFSK )
            {
                sx126x_pkt_status_gfsk_t pkt_status;
                sx126x_get_gfsk_pkt_status( context, &pkt_status );

                if( pkt_status.rx_status.crc_error == true )
                {
                    HAL_DBG_TRACE_ERROR( "CRC error from packet status\n" );
                    // The CRC error call to on_crc_error is handled with SX126X_IRQ_CRC_ERROR
                }
                else if( pkt_status.rx_status.adrs_error == true )
                {
                    HAL_DBG_TRACE_ERROR( "Address error from packet status\n" );
                    on_rx_error( );
                }
                else if( pkt_status.rx_status.length_error == true )
                {
                    HAL_DBG_TRACE_ERROR( "Length error from packet status\n" );
                    on_rx_error( );
                }
                else
                {
                    on_rx_done( );
                }
            }
            else
            {
                on_rx_done( );
            }
        }

        if( ( irq_regs & SX126X_IRQ_PREAMBLE_DETECTED ) == SX126X_IRQ_PREAMBLE_DETECTED )
        {
            HAL_DBG_TRACE_INFO( "Preamble detected\n" );
            on_preamble_detected( );
        }

        if( ( irq_regs & SX126X_IRQ_SYNC_WORD_VALID ) == SX126X_IRQ_SYNC_WORD_VALID )
        {
            HAL_DBG_TRACE_INFO( "Syncword valid\n" );
            on_syncword_valid( );
        }

        if( ( irq_regs & SX126X_IRQ_HEADER_VALID ) == SX126X_IRQ_HEADER_VALID )
        {
            HAL_DBG_TRACE_INFO( "Header valid\n" );
            on_header_valid( );
        }

        if( ( irq_regs & SX126X_IRQ_HEADER_ERROR ) == SX126X_IRQ_HEADER_ERROR )
        {
            HAL_DBG_TRACE_ERROR( "Header error\n" );
            on_header_error( );
        }

        if( ( irq_regs & SX126X_IRQ_CRC_ERROR ) == SX126X_IRQ_CRC_ERROR )
        {
            HAL_DBG_TRACE_ERROR( "CRC error\n" );
            on_crc_error( );
        }

        if( ( irq_regs & SX126X_IRQ_CAD_DONE ) == SX126X_IRQ_CAD_DONE )
        {
            HAL_DBG_TRACE_INFO( "CAD done\n" );
            if( ( irq_regs & SX126X_IRQ_CAD_DETECTED ) == SX126X_IRQ_CAD_DETECTED )
            {
                HAL_DBG_TRACE_INFO( "Channel activity detected\n" );
                on_cad_done_detected( );
            }
            else
            {
                HAL_DBG_TRACE_INFO( "No channel activity detected\n" );
                on_cad_done_undetected( );
            }
        }

        if( ( irq_regs & SX126X_IRQ_TIMEOUT ) == SX126X_IRQ_TIMEOUT )
        {
            HAL_DBG_TRACE_WARNING( "Rx timeout\n" );
            on_rx_timeout( );
        }

        if( ( irq_regs & SX126X_IRQ_LR_FHSS_HOP ) == SX126X_IRQ_LR_FHSS_HOP )
        {
            HAL_DBG_TRACE_INFO( "FHSS hop done\n" );
            on_fhss_hop_done( );
        }
    }
}

void apps_common_sx126x_handle_pre_tx( void )
{
    if( shield_pinout->led_tx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_tx.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }
}

void apps_common_sx126x_handle_post_tx( void )
{
    if( shield_pinout->led_tx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_tx.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }
}

void apps_common_sx126x_handle_pre_rx( void )
{
    if( shield_pinout->led_rx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_rx.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }
}

void apps_common_sx126x_handle_post_rx( void )
{
    if( shield_pinout->led_rx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_rx.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }
}

uint32_t get_time_on_air_in_ms( void )
{
    switch( PACKET_TYPE )
    {
    case SX126X_PKT_TYPE_LORA:
    {
        return sx126x_get_lora_time_on_air_in_ms( &lora_pkt_params, &lora_mod_params );
    }
    case SX126X_PKT_TYPE_GFSK:
    {
        return sx126x_get_gfsk_time_on_air_in_ms( &gfsk_pkt_params, &gfsk_mod_params );
    }
    case SX126X_PKT_TYPE_LR_FHSS:
    default:
    {
        return 0;
    }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void radio_on_dio_irq( void* context )
{
    irq_fired = true;
}
void on_tx_done( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_rx_done( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_preamble_detected( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_syncword_valid( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_header_valid( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_header_error( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_crc_error( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_rx_timeout( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_rx_error( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_cad_done_undetected( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_cad_done_detected( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}
void on_fhss_hop_done( void )
{
    HAL_DBG_TRACE_INFO( "%s\n", __func__ );
}

/* --- EOF ------------------------------------------------------------------ */
