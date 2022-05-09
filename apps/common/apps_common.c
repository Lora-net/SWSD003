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
#include "smtc_board.h"
#include "board_options.h"
#include "apps_common.h"
#include "lr11xx_regmem.h"
#include "apps_utilities.h"
#include "shield_pinout.h"
#include "lr11xx_system.h"
#include "lr11xx_radio.h"
#include "smtc_hal.h"
#include "lr11xx_radio_types_str.h"

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

static lr11xx_hal_context_t context = {
    .nss    = SMTC_RADIO_NSS,
    .busy   = SMTC_RADIO_BUSY,
    .reset  = SMTC_RADIO_NRST,
    .spi_id = HAL_RADIO_SPI_ID,
};

static volatile bool irq_fired = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Print the common configuration on the debug interface
 */
void print_common_configuration( void );

/*!
 * @brief Print the LoRa configuration on the debug interface
 */
void print_lora_configuration( void );

/*!
 * @brief Print the GFSK configuration on the debug interface
 */
void print_gfsk_configuration( void );

void radio_on_dio_irq( void* context );
void on_tx_done( void ) __attribute__( ( weak ) );
void on_rx_done( void ) __attribute__( ( weak ) );
void on_rx_timeout( void ) __attribute__( ( weak ) );
void on_preamble_detected( void ) __attribute__( ( weak ) );
void on_syncword_header_valid( void ) __attribute__( ( weak ) );
void on_header_error( void ) __attribute__( ( weak ) );
void on_fsk_len_error( void ) __attribute__( ( weak ) );
void on_rx_crc_error( void ) __attribute__( ( weak ) );
void on_cad_done_undetected( void ) __attribute__( ( weak ) );
void on_cad_done_detected( void ) __attribute__( ( weak ) );
void on_wifi_scan_done( void ) __attribute__( ( weak ) );
void on_gnss_scan_done( void ) __attribute__( ( weak ) );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_hal_context_t* apps_common_lr11xx_get_context( )
{
    return &context;
}

void apps_common_lr11xx_system_init( const lr11xx_hal_context_t* context )
{
    ASSERT_LR11XX_RC( lr11xx_system_reset( ( void* ) context ) );

    // Configure the regulator
    const lr11xx_system_reg_mode_t regulator = smtc_board_get_reg_mode( );
    ASSERT_LR11XX_RC( lr11xx_system_set_reg_mode( ( void* ) context, regulator ) );

    const lr11xx_system_rfswitch_cfg_t rf_switch_setup = smtc_board_get_rf_switch_cfg( );
    ASSERT_LR11XX_RC( lr11xx_system_set_dio_as_rf_switch( context, &rf_switch_setup ) );

    // Configure the 32MHz TCXO if it is available on the board
    const smtc_board_tcxo_cfg_t tcxo_cfg = smtc_board_get_tcxo_cfg( );
    if( tcxo_cfg.has_tcxo == true )
    {
        const uint32_t timeout_rtc_step = lr11xx_radio_convert_time_in_ms_to_rtc_step( tcxo_cfg.timeout_ms );
        ASSERT_LR11XX_RC( lr11xx_system_set_tcxo_mode( context, tcxo_cfg.supply, timeout_rtc_step ) );
    }

    // Configure the Low Frequency Clock
    const smtc_board_lf_clck_cfg_t lf_clk_cfg = smtc_board_get_lf_clk_cfg( );
    ASSERT_LR11XX_RC( lr11xx_system_cfg_lfclk( context, lf_clk_cfg.lf_clk_cfg, lf_clk_cfg.wait_32k_ready ) );

    ASSERT_LR11XX_RC( lr11xx_system_clear_errors( context ) );
    ASSERT_LR11XX_RC( lr11xx_system_calibrate( context, 0x3F ) );

    uint16_t errors;
    ASSERT_LR11XX_RC( lr11xx_system_get_errors( context, &errors ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_errors( context ) );
    ASSERT_LR11XX_RC( lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK ) );
}

void apps_common_lr11xx_fetch_and_print_version( const lr11xx_hal_context_t* context )
{
    lr11xx_system_version_t version;

    ASSERT_LR11XX_RC( lr11xx_system_get_version( ( void* ) context, &version ) );

    HAL_DBG_TRACE_INFO( "LR11xx information:\n" );
    HAL_DBG_TRACE_INFO( "  - Firmware = 0x%04X\n", version.fw );
    HAL_DBG_TRACE_INFO( "  - Hardware = 0x%02X\n", version.hw );
    HAL_DBG_TRACE_INFO( "  - Type     = 0x%02X (0x01 for LR1110, 0x02 for LR1120)\n", version.type );
    HAL_DBG_TRACE_PRINTF( "\n" );
}

void apps_common_lr11xx_radio_init( const void* context )
{
    const smtc_board_pa_pwr_cfg_t* pa_pwr_cfg = smtc_board_get_pa_pwr_cfg( RF_FREQ_IN_HZ, TX_OUTPUT_POWER_DBM );

    if( pa_pwr_cfg == NULL )
    {
        HAL_DBG_TRACE_ERROR( "Invalid target frequency or power level\n" );
        while( true )
        {
        }
    }

    print_common_configuration( );

    ASSERT_LR11XX_RC( lr11xx_radio_set_pkt_type( context, PACKET_TYPE ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rf_freq( context, RF_FREQ_IN_HZ ) );
    ASSERT_LR11XX_RC(
        lr11xx_radio_set_rssi_calibration( context, smtc_board_get_rssi_calibration_table( RF_FREQ_IN_HZ ) ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx_params( context, pa_pwr_cfg->power, PA_RAMP_TIME ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx_tx_fallback_mode( context, FALLBACK_MODE ) );
    ASSERT_LR11XX_RC( lr11xx_radio_cfg_rx_boosted( context, ENABLE_RX_BOOST_MODE ) );

    if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_LORA )
    {
        print_lora_configuration( );

        const lr11xx_radio_mod_params_lora_t lora_mod_params = {
            .sf   = LORA_SPREADING_FACTOR,
            .bw   = LORA_BANDWIDTH,
            .cr   = LORA_CODING_RATE,
            .ldro = apps_common_compute_lora_ldro( LORA_SPREADING_FACTOR, LORA_BANDWIDTH ),
        };

        const lr11xx_radio_pkt_params_lora_t lora_pkt_params = {
            .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,
            .header_type          = LORA_PKT_LEN_MODE,
            .pld_len_in_bytes     = PAYLOAD_LENGTH,
            .crc                  = LORA_CRC,
            .iq                   = LORA_IQ,
        };

        ASSERT_LR11XX_RC( lr11xx_radio_set_lora_mod_params( context, &lora_mod_params ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_lora_pkt_params( context, &lora_pkt_params ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_lora_sync_word( context, LORA_SYNCWORD ) );
    }
    else if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
    {
        print_gfsk_configuration( );

        const lr11xx_radio_mod_params_gfsk_t gfsk_mod_params = {
            .br_in_bps    = FSK_BITRATE,
            .pulse_shape  = FSK_PULSE_SHAPE,
            .bw_dsb_param = FSK_BANDWIDTH,
            .fdev_in_hz   = FSK_FDEV,
        };

        const lr11xx_radio_pkt_params_gfsk_t gfsk_pkt_params = {
            .preamble_len_in_bits  = FSK_PREAMBLE_LENGTH,
            .preamble_detector     = FSK_PREAMBLE_DETECTOR,
            .sync_word_len_in_bits = FSK_SYNCWORD_LENGTH,
            .address_filtering     = FSK_ADDRESS_FILTERING,
            .header_type           = FSK_HEADER_TYPE,
            .pld_len_in_bytes      = PAYLOAD_LENGTH,
            .crc_type              = FSK_CRC_TYPE,
            .dc_free               = FSK_DC_FREE,
        };

        ASSERT_LR11XX_RC( lr11xx_radio_set_gfsk_mod_params( context, &gfsk_mod_params ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_gfsk_pkt_params( context, &gfsk_pkt_params ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_gfsk_sync_word( context, gfsk_sync_word ) );

        if( FSK_DC_FREE != LR11XX_RADIO_GFSK_DC_FREE_OFF )
        {
            ASSERT_LR11XX_RC( lr11xx_radio_set_gfsk_whitening_seed( context, FSK_WHITENING_SEED ) );
        }

        if( FSK_CRC_TYPE != LR11XX_RADIO_GFSK_CRC_OFF )
        {
            ASSERT_LR11XX_RC( lr11xx_radio_set_gfsk_crc_params( context, FSK_CRC_SEED, FSK_CRC_POLYNOMIAL ) );
        }

        if( FSK_ADDRESS_FILTERING != LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE )
        {
            ASSERT_LR11XX_RC( lr11xx_radio_set_pkt_address( context, FSK_NODE_ADDRESS, FSK_BROADCAST_ADDRESS ) );
        }
    }
}

void apps_common_lr11xx_receive( const void* context, uint8_t* buffer, uint8_t* size )
{
    lr11xx_radio_rx_buffer_status_t rx_buffer_status;
    lr11xx_radio_pkt_status_lora_t  pkt_status_lora;
    lr11xx_radio_pkt_status_gfsk_t  pkt_status_gfsk;

    lr11xx_radio_get_rx_buffer_status( context, &rx_buffer_status );
    lr11xx_regmem_read_buffer8( context, buffer, rx_buffer_status.buffer_start_pointer,
                                rx_buffer_status.pld_len_in_bytes );
    *size = rx_buffer_status.pld_len_in_bytes;

    HAL_DBG_TRACE_ARRAY( "Packet content", buffer, *size );

    HAL_DBG_TRACE_INFO( "Packet status:\n" );
    if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_LORA )
    {
        lr11xx_radio_get_lora_pkt_status( context, &pkt_status_lora );
        HAL_DBG_TRACE_INFO( "  - RSSI packet = %i dBm\n", pkt_status_lora.rssi_pkt_in_dbm );
        HAL_DBG_TRACE_INFO( "  - Signal RSSI packet = %i dBm\n", pkt_status_lora.signal_rssi_pkt_in_dbm );
        HAL_DBG_TRACE_INFO( "  - SNR packet = %i dB\n", pkt_status_lora.snr_pkt_in_db );
    }
    else if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
    {
        lr11xx_radio_get_gfsk_pkt_status( context, &pkt_status_gfsk );
        HAL_DBG_TRACE_INFO( "  - RSSI average = %i dBm\n", pkt_status_gfsk.rssi_avg_in_dbm );
        HAL_DBG_TRACE_INFO( "  - RSSI sync = %i dBm\n", pkt_status_gfsk.rssi_sync_in_dbm );
    }
}

void apps_common_lr11xx_enable_irq( void )
{
    hal_mcu_disable_irq( );

    /* Init chip Irq */
    static const hal_gpio_irq_t irq = {
        .pin      = SMTC_RADIO_DIOX,
        .callback = radio_on_dio_irq,
        .context  = NULL,
    };
    hal_gpio_irq_attach( &irq );

    hal_mcu_enable_irq( );
}

void apps_common_lr11xx_irq_process( const void* context, lr11xx_system_irq_mask_t irq_filter_mask )
{
    if( irq_fired == true )
    {
        hal_mcu_disable_irq( );
        irq_fired = false;
        hal_mcu_enable_irq( );

        lr11xx_system_irq_mask_t irq_regs;
        lr11xx_system_get_and_clear_irq_status( context, &irq_regs );

        HAL_DBG_TRACE_INFO( "Interrupt flags = 0x%08X\n", irq_regs );

        irq_regs &= irq_filter_mask;

        HAL_DBG_TRACE_INFO( "Interrupt flags (after filtering) = 0x%08X\n", irq_regs );

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_TX_DONE ) == LR11XX_SYSTEM_IRQ_TX_DONE )
        {
            HAL_DBG_TRACE_INFO( "Tx done\n" );
            on_tx_done( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED ) == LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED )
        {
            HAL_DBG_TRACE_INFO( "Preamble detected\n" );
            on_preamble_detected( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_HEADER_ERROR ) == LR11XX_SYSTEM_IRQ_HEADER_ERROR )
        {
            HAL_DBG_TRACE_ERROR( "Header error\n" );
            on_header_error( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID ) == LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID )
        {
            HAL_DBG_TRACE_INFO( "Syncword or header valid\n" );
            on_syncword_header_valid( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_RX_DONE ) == LR11XX_SYSTEM_IRQ_RX_DONE )
        {
            if( ( irq_regs & LR11XX_SYSTEM_IRQ_CRC_ERROR ) == LR11XX_SYSTEM_IRQ_CRC_ERROR )
            {
                HAL_DBG_TRACE_ERROR( "CRC error\n" );
                on_rx_crc_error( );
            }
            else if( ( irq_regs & LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR ) == LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR )
            {
                HAL_DBG_TRACE_ERROR( "FSK length error\n" );
                on_fsk_len_error( );
            }
            else
            {
                HAL_DBG_TRACE_INFO( "Rx done\n" );
                on_rx_done( );
            }
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_CAD_DONE ) == LR11XX_SYSTEM_IRQ_CAD_DONE )
        {
            HAL_DBG_TRACE_INFO( "CAD done\n" );
            if( ( irq_regs & LR11XX_SYSTEM_IRQ_CAD_DETECTED ) == LR11XX_SYSTEM_IRQ_CAD_DETECTED )
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

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_TIMEOUT ) == LR11XX_SYSTEM_IRQ_TIMEOUT )
        {
            HAL_DBG_TRACE_WARNING( "Rx timeout\n" );
            on_rx_timeout( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE ) == LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE )
        {
            HAL_DBG_TRACE_INFO( "Wi-Fi scan done\n" );
            on_wifi_scan_done( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE ) == LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE )
        {
            HAL_DBG_TRACE_INFO( "GNSS scan done\n" );
            on_gnss_scan_done( );
        }

        HAL_DBG_TRACE_PRINTF( "\n" );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void print_common_configuration( void )
{
    HAL_DBG_TRACE_INFO( "Common parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Packet type   = %s\n", lr11xx_radio_pkt_type_to_str( PACKET_TYPE ) );
    HAL_DBG_TRACE_INFO( "   RF frequency  = %u Hz\n", RF_FREQ_IN_HZ );
    HAL_DBG_TRACE_INFO( "   Output power  = %i dBm\n", TX_OUTPUT_POWER_DBM );
    HAL_DBG_TRACE_INFO( "   Fallback mode = %s\n", lr11xx_radio_fallback_modes_to_str( FALLBACK_MODE ) );
    HAL_DBG_TRACE_INFO( ( ENABLE_RX_BOOST_MODE == true ) ? "   Rx boost activated\n" : "   Rx boost deactivated\n" );
    HAL_DBG_TRACE_PRINTF( "\n" );
}

void print_lora_configuration( void )
{
    HAL_DBG_TRACE_INFO( "LoRa modulation parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Spreading factor = %s\n", lr11xx_radio_lora_sf_to_str( LORA_SPREADING_FACTOR ) );
    HAL_DBG_TRACE_INFO( "   Bandwidth        = %s\n", lr11xx_radio_lora_bw_to_str( LORA_BANDWIDTH ) );
    HAL_DBG_TRACE_INFO( "   Coding rate      = %s\n", lr11xx_radio_lora_cr_to_str( LORA_CODING_RATE ) );
    HAL_DBG_TRACE_PRINTF( "\n" );

    HAL_DBG_TRACE_INFO( "LoRa packet parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Preamble length = %d symbol(s)\n", LORA_PREAMBLE_LENGTH );
    HAL_DBG_TRACE_INFO( "   Header mode     = %s\n", lr11xx_radio_lora_pkt_len_modes_to_str( LORA_PKT_LEN_MODE ) );
    HAL_DBG_TRACE_INFO( "   Payload length  = %d byte(s)\n", PAYLOAD_LENGTH );
    HAL_DBG_TRACE_INFO( "   CRC mode        = %s\n", lr11xx_radio_lora_crc_to_str( LORA_CRC ) );
    HAL_DBG_TRACE_INFO( "   IQ              = %s\n", lr11xx_radio_lora_iq_to_str( LORA_IQ ) );
    HAL_DBG_TRACE_PRINTF( "\n" );

    HAL_DBG_TRACE_INFO( "LoRa syncword = 0x%02X\n", LORA_SYNCWORD );
    HAL_DBG_TRACE_PRINTF( "\n" );
}

void print_gfsk_configuration( void )
{
    HAL_DBG_TRACE_INFO( "GFSK modulation parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Bitrate             = %u bps\n", FSK_BITRATE );
    HAL_DBG_TRACE_INFO( "   Pulse shape         = %s\n", lr11xx_radio_gfsk_pulse_shape_to_str( FSK_PULSE_SHAPE ) );
    HAL_DBG_TRACE_INFO( "   Bandwidth           = %s\n", lr11xx_radio_gfsk_bw_to_str( FSK_BANDWIDTH ) );
    HAL_DBG_TRACE_INFO( "   Frequency deviation = %u Hz\n", FSK_FDEV );
    HAL_DBG_TRACE_PRINTF( "\n" );

    HAL_DBG_TRACE_INFO( "GFSK packet parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Preamble length   = %d bit(s)\n", FSK_PREAMBLE_LENGTH );
    HAL_DBG_TRACE_INFO( "   Preamble detector = %s\n",
                        lr11xx_radio_gfsk_preamble_detector_to_str( FSK_PREAMBLE_DETECTOR ) );
    HAL_DBG_TRACE_INFO( "   Syncword length   = %d bit(s)\n", FSK_SYNCWORD_LENGTH );
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
    HAL_DBG_TRACE_INFO( "   Payload length    = %d byte(s)\n", PAYLOAD_LENGTH );
    HAL_DBG_TRACE_INFO( "   CRC mode          = %s\n", lr11xx_radio_gfsk_crc_type_to_str( FSK_CRC_TYPE ) );
    if( FSK_CRC_TYPE != LR11XX_RADIO_GFSK_CRC_OFF )
    {
        HAL_DBG_TRACE_INFO( "     (CRC seed       = 0x%08X)\n", FSK_CRC_SEED );
        HAL_DBG_TRACE_INFO( "     (CRC polynomial = 0x%08X)\n", FSK_CRC_POLYNOMIAL );
    }
    HAL_DBG_TRACE_INFO( "   DC free           = %s\n", lr11xx_radio_gfsk_dc_free_to_str( FSK_DC_FREE ) );
    if( FSK_DC_FREE != LR11XX_RADIO_GFSK_DC_FREE_OFF )
    {
        HAL_DBG_TRACE_INFO( "     (Whitening seed = 0x%04X)\n", FSK_WHITENING_SEED );
    }
    HAL_DBG_TRACE_PRINTF( "\n" );
}

void radio_on_dio_irq( void* context )
{
    irq_fired = true;
}
void on_tx_done( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_rx_done( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_rx_timeout( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_preamble_detected( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_syncword_header_valid( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_header_error( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_fsk_len_error( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_rx_crc_error( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_cad_done_undetected( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_cad_done_detected( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_wifi_scan_done( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_gnss_scan_done( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}

/* --- EOF ------------------------------------------------------------------ */
