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
#include "common_version.h"
#include "apps_common.h"
#include "lr11xx_regmem.h"
#include "apps_utilities.h"
#include "lr11xx_system.h"
#include "lr11xx_radio.h"
#include "lr11xx_driver_version.h"
#include "lr11xx_rttof.h"
#include "smtc_hal_dbg_trace.h"
#include "smtc_shield_pinout_mapping.h"
#include "smtc_shield_lr11xx.h"

#include "smtc_shield_lr1110mb1dis.h"
#include "smtc_shield_lr1110mb1ipddis.h"
#include "smtc_shield_lr1110mb1djs.h"
#include "smtc_shield_lr1110mb1gis.h"
#include "smtc_shield_lr1110mb1pis.h"
#include "smtc_shield_lr1110mb1gjs.h"
#include "smtc_shield_lr1120mb1dis.h"
#include "smtc_shield_lr1120mb1ipddis.h"
#include "smtc_shield_lr1120mb1djs.h"
#include "smtc_shield_lr1120mb1gis.h"
#include "smtc_shield_lr1120mb1pis.h"
#include "smtc_shield_lr1120mb1gjs.h"
#include "smtc_shield_lr1121mb1dis.h"
#include "smtc_shield_lr1121mb1gis.h"

#include "smtc_dbpsk.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#ifdef LR1110MB1DIS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1110MB1DIS_INSTANTIATE;
#endif
#ifdef LR1110MB1IPDDIS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1110MB1IPDDIS_INSTANTIATE;
#endif
#ifdef LR1110MB1DJS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1110MB1DJS_INSTANTIATE;
#endif
#ifdef LR1110MB1GIS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1110MB1GIS_INSTANTIATE;
#endif
#ifdef LR1110MB1PIS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1110MB1PIS_INSTANTIATE;
#endif
#ifdef LR1110MB1GJS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1110MB1GJS_INSTANTIATE;
#endif
#ifdef LR1120MB1DIS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1120MB1DIS_INSTANTIATE;
#endif
#ifdef LR1120MB1IPDDIS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1120MB1IPDDIS_INSTANTIATE;
#endif
#ifdef LR1120MB1DJS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1120MB1DJS_INSTANTIATE;
#endif
#ifdef LR1120MB1GIS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1120MB1GIS_INSTANTIATE;
#endif
#ifdef LR1120MB1PIS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1120MB1PIS_INSTANTIATE;
#endif
#ifdef LR1120MB1GJS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1120MB1GJS_INSTANTIATE;
#endif
#ifdef LR1121MB1DIS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1121MB1DIS_INSTANTIATE;
#endif
#ifdef LR1121MB1GIS
smtc_shield_lr11xx_t shield = SMTC_SHIELD_LR1121MB1GIS_INSTANTIATE;
#endif

static lr11xx_radio_mod_params_lora_t lora_mod_params = {
    .sf   = LORA_SPREADING_FACTOR,
    .bw   = LORA_BANDWIDTH,
    .cr   = LORA_CODING_RATE,
    .ldro = 0  // Will be initialized in radio init
};

static const lr11xx_radio_pkt_params_lora_t lora_pkt_params = {
    .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,
    .header_type          = LORA_PKT_LEN_MODE,
    .pld_len_in_bytes     = PAYLOAD_LENGTH,
    .crc                  = LORA_CRC,
    .iq                   = LORA_IQ,
};

static const lr11xx_radio_mod_params_gfsk_t gfsk_mod_params = {
    .br_in_bps    = FSK_BITRATE,
    .pulse_shape  = FSK_PULSE_SHAPE,
    .bw_dsb_param = FSK_BANDWIDTH,
    .fdev_in_hz   = FSK_FDEV,
};

static const lr11xx_radio_pkt_params_gfsk_t gfsk_pkt_params = {
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_hal_context_t context;

static volatile bool irq_fired = false;

static const smtc_shield_lr11xx_pinout_t* shield_pinout = 0;

static struct
{
    smtc_hal_mcu_gpio_cfg_t        cfg;
    smtc_hal_mcu_gpio_output_cfg_t cfg_output;
    smtc_hal_mcu_gpio_inst_t       inst;
} led_tx;

static struct
{
    smtc_hal_mcu_gpio_cfg_t        cfg;
    smtc_hal_mcu_gpio_output_cfg_t cfg_output;
    smtc_hal_mcu_gpio_inst_t       inst;
} led_rx;

static struct
{
    smtc_hal_mcu_gpio_cfg_t        cfg;
    smtc_hal_mcu_gpio_output_cfg_t cfg_output;
    smtc_hal_mcu_gpio_inst_t       inst;
} led_scan;

static struct
{
    smtc_hal_mcu_gpio_cfg_t        cfg;
    smtc_hal_mcu_gpio_output_cfg_t cfg_output;
    smtc_hal_mcu_gpio_inst_t       inst;
} lna_ctrl;

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

/*!
 * @brief Print the LR11XX driver version
 */
static void print_driver_version( void );

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
void on_lora_rx_timestamp( void ) __attribute__( ( weak ) );
void on_rttof_request_valid( void ) __attribute__( ( weak ) );
void on_rttof_request_discarded( void ) __attribute__( ( weak ) );
void on_rttof_response_done( void ) __attribute__( ( weak ) );
void on_rttof_exchange_valid( void ) __attribute__( ( weak ) );
void on_rttof_timeout( void ) __attribute__( ( weak ) );

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
    shield_pinout = smtc_shield_lr11xx_get_pinout( &shield );

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

    if( shield_pinout->led_scan != SMTC_SHIELD_PINOUT_NONE )
    {
        led_scan.cfg                      = smtc_shield_pinout_mapping_get_gpio_cfg( shield_pinout->led_scan );
        led_scan.cfg_output.initial_state = SMTC_HAL_MCU_GPIO_STATE_LOW;
        led_scan.cfg_output.mode          = SMTC_HAL_MCU_GPIO_OUTPUT_MODE_PUSH_PULL;

        smtc_hal_mcu_gpio_init_output( led_scan.cfg, &( led_scan.cfg_output ), &( led_scan.inst ) );
    }

    if( shield_pinout->lna != SMTC_SHIELD_PINOUT_NONE )
    {
        lna_ctrl.cfg                      = smtc_shield_pinout_mapping_get_gpio_cfg( shield_pinout->lna );
        lna_ctrl.cfg_output.initial_state = SMTC_HAL_MCU_GPIO_STATE_LOW;
        lna_ctrl.cfg_output.mode          = SMTC_HAL_MCU_GPIO_OUTPUT_MODE_PUSH_PULL;

        smtc_hal_mcu_gpio_init_output( lna_ctrl.cfg, &( lna_ctrl.cfg_output ), &( lna_ctrl.inst ) );
    }
}

void apps_common_lr11xx_system_init( const lr11xx_hal_context_t* context )
{
    ASSERT_LR11XX_RC( lr11xx_system_reset( ( void* ) context ) );

    // Configure the regulator
    const lr11xx_system_reg_mode_t regulator = smtc_shield_lr11xx_get_reg_mode( &shield );
    ASSERT_LR11XX_RC( lr11xx_system_set_reg_mode( ( void* ) context, regulator ) );

    const lr11xx_system_rfswitch_cfg_t* rf_switch_setup = smtc_shield_lr11xx_get_rf_switch_cfg( &shield );
    ASSERT_LR11XX_RC( lr11xx_system_set_dio_as_rf_switch( context, rf_switch_setup ) );

    const smtc_shield_lr11xx_xosc_cfg_t* tcxo_cfg = smtc_shield_lr11xx_get_xosc_cfg( &shield );
    if( tcxo_cfg->has_tcxo == true )
    {
        ASSERT_LR11XX_RC( lr11xx_system_set_tcxo_mode( context, tcxo_cfg->supply, tcxo_cfg->startup_time_in_tick ) );
    }

    const smtc_shield_lr11xx_lfclk_cfg_t* lfclk_cfg = smtc_shield_lr11xx_get_lfclk_cfg( &shield );
    ASSERT_LR11XX_RC( lr11xx_system_cfg_lfclk( context, lfclk_cfg->lf_clk_cfg, lfclk_cfg->wait_32k_ready ) );

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

    apps_common_lr11xx_fetch_version( ( void* ) context, &version );
    apps_common_lr11xx_print_version( &version );
}

void apps_common_lr11xx_fetch_version( const lr11xx_hal_context_t* context, lr11xx_system_version_t* version )
{
    ASSERT_LR11XX_RC( lr11xx_system_get_version( ( void* ) context, version ) );
}

void apps_common_lr11xx_print_version( const lr11xx_system_version_t* version )
{
    HAL_DBG_TRACE_INFO( "LR11xx information:\n" );
    HAL_DBG_TRACE_INFO( "  - Firmware = 0x%04X\n", version->fw );
    HAL_DBG_TRACE_INFO( "  - Hardware = 0x%02X\n", version->hw );
    HAL_DBG_TRACE_INFO( "  - Type     = 0x%02X (0x01 for LR1110, 0x02 for LR1120, 0x03 for LR1121)\n", version->type );

    if( ( version->type == LR11XX_SYSTEM_VERSION_TYPE_LR1110 ) && ( version->fw != LR1110_LATEST_FW_VERSION ) )
    {
        HAL_DBG_TRACE_WARNING( "LR1110 is on version 0x%02x, but latest firmware version is 0x%02X\n", version->fw,
                               LR1110_LATEST_FW_VERSION );
    }
    if( ( version->type == LR11XX_SYSTEM_VERSION_TYPE_LR1120 ) && ( version->fw != LR1120_LATEST_FW_VERSION ) )
    {
        HAL_DBG_TRACE_WARNING( "LR1120 doesn't use latest firmware version which is 0x%02X\n",
                               LR1120_LATEST_FW_VERSION );
    }
    if( ( version->type == LR11XX_SYSTEM_VERSION_TYPE_LR1121 ) && ( version->fw != LR1121_LATEST_FW_VERSION ) )
    {
        HAL_DBG_TRACE_WARNING( "LR1121 doesn't use latest firmware version which is 0x%02X\n",
                               LR1121_LATEST_FW_VERSION );
    }

    HAL_DBG_TRACE_PRINTF( "\n" );
}

void apps_common_print_sdk_driver_version( void )
{
    common_version_print( );
    print_driver_version( );
    HAL_DBG_TRACE_INFO( "\n" );
}

void apps_common_lr11xx_radio_init( const void* context )
{
    const smtc_shield_lr11xx_pa_pwr_cfg_t* pa_pwr_cfg =
        smtc_shield_lr11xx_get_pa_pwr_cfg( &shield, RF_FREQ_IN_HZ, TX_OUTPUT_POWER_DBM );

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
    ASSERT_LR11XX_RC( lr11xx_radio_set_rssi_calibration(
        context, smtc_shield_lr11xx_get_rssi_calibration_table( &shield, RF_FREQ_IN_HZ ) ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx_params( context, pa_pwr_cfg->power, PA_RAMP_TIME ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx_tx_fallback_mode( context, FALLBACK_MODE ) );
    ASSERT_LR11XX_RC( lr11xx_radio_cfg_rx_boosted( context, ENABLE_RX_BOOST_MODE ) );

    if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_LORA )
    {
        print_lora_configuration( );

        lora_mod_params.ldro = apps_common_compute_lora_ldro( LORA_SPREADING_FACTOR, LORA_BANDWIDTH );
        ASSERT_LR11XX_RC( lr11xx_radio_set_lora_mod_params( context, &lora_mod_params ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_lora_pkt_params( context, &lora_pkt_params ) );
        ASSERT_LR11XX_RC( lr11xx_radio_set_lora_sync_word( context, LORA_SYNCWORD ) );
    }
    else if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
    {
        print_gfsk_configuration( );

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
    else if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_LR_FHSS )
    {
        const lr11xx_radio_mod_params_lr_fhss_t mod_lr_fhss = {
            .br_in_bps   = LR11XX_RADIO_LR_FHSS_BITRATE_488_BPS,
            .pulse_shape = LR11XX_RADIO_LR_FHSS_PULSE_SHAPE_BT_1,
        };

        ASSERT_LR11XX_RC( lr11xx_radio_set_lr_fhss_mod_params( context, &mod_lr_fhss ) );
    }
}

void apps_common_lr11xx_radio_dbpsk_init( const void* context, const uint8_t payload_len )
{
    const smtc_shield_lr11xx_pa_pwr_cfg_t* pa_pwr_cfg =
        smtc_shield_lr11xx_get_pa_pwr_cfg( &shield, SIGFOX_UPLINK_RF_FREQ_IN_HZ, SIGFOX_TX_OUTPUT_POWER_DBM );

    if( pa_pwr_cfg == NULL )
    {
        HAL_DBG_TRACE_ERROR( "Invalid target frequency or power level\n" );
        while( true )
        {
        }
    }

    HAL_DBG_TRACE_INFO( "Sigfox parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Packet type   = %s\n", lr11xx_radio_pkt_type_to_str( LR11XX_RADIO_PKT_TYPE_BPSK ) );
    HAL_DBG_TRACE_INFO( "   RF frequency  = %u Hz\n", SIGFOX_UPLINK_RF_FREQ_IN_HZ );
    HAL_DBG_TRACE_INFO( "   Output power  = %i dBm\n", SIGFOX_TX_OUTPUT_POWER_DBM );

    ASSERT_LR11XX_RC( lr11xx_radio_set_pkt_type( context, LR11XX_RADIO_PKT_TYPE_BPSK ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rf_freq( context, SIGFOX_UPLINK_RF_FREQ_IN_HZ ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rssi_calibration(
        context, smtc_shield_lr11xx_get_rssi_calibration_table( &shield, SIGFOX_UPLINK_RF_FREQ_IN_HZ ) ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) ) );

    ASSERT_LR11XX_RC( lr11xx_radio_set_bpsk_mod_params( context, &bpsk_mod_params ) );

    bpsk_pkt_params.pld_len_in_bytes = smtc_dbpsk_get_pld_len_in_bytes( payload_len << 3 );
    bpsk_pkt_params.pld_len_in_bits  = smtc_dbpsk_get_pld_len_in_bits( payload_len << 3 );

    if( BPSK_BITRATE_IN_BPS == 100 )
    {
        bpsk_pkt_params.ramp_up_delay   = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_100_BPS;
        bpsk_pkt_params.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS;
    }
    else if( BPSK_BITRATE_IN_BPS == 600 )
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
}

void apps_common_lr11xx_radio_rttof_init( const void* context )
{
    const smtc_shield_lr11xx_pa_pwr_cfg_t* pa_pwr_cfg =
        smtc_shield_lr11xx_get_pa_pwr_cfg( &shield, RF_FREQ_IN_HZ, TX_OUTPUT_POWER_DBM );

    if( pa_pwr_cfg == NULL )
    {
        HAL_DBG_TRACE_ERROR( "Invalid target frequency or power level\n" );
        while( true )
        {
        }
    }

    print_common_configuration( );

    ASSERT_LR11XX_RC( lr11xx_radio_set_pkt_type( context, LR11XX_RADIO_PKT_TYPE_RTTOF ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rf_freq( context, RF_FREQ_IN_HZ ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rssi_calibration(
        context, smtc_shield_lr11xx_get_rssi_calibration_table( &shield, RF_FREQ_IN_HZ ) ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_tx_params( context, pa_pwr_cfg->power, PA_RAMP_TIME ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rx_tx_fallback_mode( context, FALLBACK_MODE ) );
    ASSERT_LR11XX_RC( lr11xx_radio_cfg_rx_boosted( context, ENABLE_RX_BOOST_MODE ) );

    print_lora_configuration( );

    lora_mod_params.ldro = apps_common_compute_lora_ldro( LORA_SPREADING_FACTOR, LORA_BANDWIDTH );
    ASSERT_LR11XX_RC( lr11xx_radio_set_lora_mod_params( context, &lora_mod_params ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_lora_pkt_params( context, &lora_pkt_params ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_lora_sync_word( context, LORA_SYNCWORD ) );

    uint32_t   rttof_rx_tx_delay       = 0u;
    const bool get_rttof_delay_success = smtc_shield_lr11xx_get_rttof_recommended_rx_tx_delay_indicator(
        &shield, lora_mod_params.bw, lora_mod_params.sf, &rttof_rx_tx_delay );
    if( get_rttof_delay_success == false )
    {
        HAL_DBG_TRACE_ERROR( "Failed to get RTToF delay indicator\n" );
    }
    ASSERT_LR11XX_RC( lr11xx_rttof_set_rx_tx_delay_indicator( context, rttof_rx_tx_delay ) );
}

void apps_common_lr11xx_receive( const void* context, uint8_t* buffer, uint8_t buffer_length, uint8_t* size )
{
    lr11xx_radio_rx_buffer_status_t rx_buffer_status;
    lr11xx_radio_pkt_status_lora_t  pkt_status_lora;
    lr11xx_radio_pkt_status_gfsk_t  pkt_status_gfsk;

    lr11xx_radio_get_rx_buffer_status( context, &rx_buffer_status );
    *size = rx_buffer_status.pld_len_in_bytes;
    if( *size > buffer_length )
    {
        HAL_DBG_TRACE_ERROR( "Received payload (size: %d) is bigger than the buffer (size: %d)!\n", *size,
                             buffer_length );
        return;
    }
    lr11xx_regmem_read_buffer8( context, buffer, rx_buffer_status.buffer_start_pointer,
                                rx_buffer_status.pld_len_in_bytes );

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

void apps_common_lr11xx_irq_process( const void* context, lr11xx_system_irq_mask_t irq_filter_mask )
{
    if( irq_fired == true )
    {
        irq_fired = false;

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

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_LORA_RX_TIMESTAMP ) == LR11XX_SYSTEM_IRQ_LORA_RX_TIMESTAMP )
        {
            HAL_DBG_TRACE_INFO( "LoRa Rx timestamp\n" );
            on_lora_rx_timestamp( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_RTTOF_REQ_VALID ) == LR11XX_SYSTEM_IRQ_RTTOF_REQ_VALID )
        {
            HAL_DBG_TRACE_INFO( "RTToF request valid\n" );
            on_rttof_request_valid( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_RTTOF_REQ_DISCARDED ) == LR11XX_SYSTEM_IRQ_RTTOF_REQ_DISCARDED )
        {
            HAL_DBG_TRACE_INFO( "RTToF request discarded\n" );
            on_rttof_request_discarded( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_RTTOF_RESP_DONE ) == LR11XX_SYSTEM_IRQ_RTTOF_RESP_DONE )
        {
            HAL_DBG_TRACE_INFO( "RTToF response done\n" );
            on_rttof_response_done( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_RTTOF_EXCH_VALID ) == LR11XX_SYSTEM_IRQ_RTTOF_EXCH_VALID )
        {
            HAL_DBG_TRACE_INFO( "RTToF exchange valid\n" );
            on_rttof_exchange_valid( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_RTTOF_TIMEOUT ) == LR11XX_SYSTEM_IRQ_RTTOF_TIMEOUT )
        {
            HAL_DBG_TRACE_INFO( "RTToF timeout\n" );
            on_rttof_timeout( );
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

void apps_common_lr11xx_handle_pre_tx( void )
{
    if( shield_pinout->led_tx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_tx.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }
}

void apps_common_lr11xx_handle_post_tx( void )
{
    if( shield_pinout->led_tx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_tx.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }
}

void apps_common_lr11xx_handle_pre_rx( void )
{
    if( shield_pinout->led_rx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_rx.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }
}

void apps_common_lr11xx_handle_post_rx( void )
{
    if( shield_pinout->led_rx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_rx.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }
}

void apps_common_lr11xx_handle_pre_gnss_scan( void )
{
    if( shield_pinout->led_scan != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_scan.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }

    if( shield_pinout->lna != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( lna_ctrl.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }
}

void apps_common_lr11xx_handle_post_gnss_scan( void )
{
    if( shield_pinout->led_scan != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_scan.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }

    if( shield_pinout->lna != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( lna_ctrl.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }
}

void apps_common_lr11xx_handle_pre_wifi_scan( void )
{
    if( shield_pinout->led_scan != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_scan.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }
}

void apps_common_lr11xx_handle_post_wifi_scan( void )
{
    if( shield_pinout->led_scan != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_scan.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }
}

uint32_t get_time_on_air_in_ms( void )
{
    switch( PACKET_TYPE )
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

void print_driver_version( void )
{
    HAL_DBG_TRACE_INFO( "LR11XX driver version: %s\n", lr11xx_driver_version_get_version_string( ) );
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
void on_lora_rx_timestamp( void )
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
