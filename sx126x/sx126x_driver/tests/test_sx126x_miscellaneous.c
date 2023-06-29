/**
 * @file      test_sx126x_miscellaneous.c
 *
 * @brief     SX126x test cases for miscellaneous commands
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "unity.h"
#include "sx126x.h"
#include "mock_sx126x_hal.h"
#include "sx126x_toa.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

#define CEILING( x ) ( int ) ( x ) + ( 1 - ( int ) ( ( int ) ( ( x ) + 1 ) - ( x ) ) )

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

void*           radio;
sx126x_status_t status;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void setUp( void )
{
}

void tearDown( void )
{
}

TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR )
TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK )
void test_sx126x_reset( sx126x_status_t status_expected, sx126x_hal_status_t status_hal )
{
    sx126x_hal_reset_ExpectAndReturn( radio, status_hal );

    status = sx126x_reset( radio );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}

TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR )
TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK )
void test_sx126x_wakeup( sx126x_status_t status_expected, sx126x_hal_status_t status_hal )
{
    sx126x_hal_wakeup_ExpectAndReturn( radio, status_hal );

    status = sx126x_wakeup( radio );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}

void test_sx126x_get_device_errors( void )
{
    uint8_t cbuffer_expected[] = { 0x17, 0x00 };
    uint8_t rbuffer_expected[] = { 0x00, 0x00 };
    uint8_t response[]         = { 0x01, 0x55 };

    sx126x_errors_mask_t errors;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 2, 2,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response, 2 );

    status = sx126x_get_device_errors( radio, &errors );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_UINT8( SX126X_ERRORS_PA_RAMP | SX126X_ERRORS_PLL_LOCK | SX126X_ERRORS_IMG_CALIBRATION |
                                 SX126X_ERRORS_PLL_CALIBRATION | SX126X_ERRORS_RC64K_CALIBRATION,
                             errors );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 2, 2,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_device_errors( radio, &errors );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_clear_device_errors( void )
{
    uint8_t cbuffer_expected[] = { 0x07, 0x00, 0x00 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_clear_device_errors( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_ERROR );

    status = sx126x_clear_device_errors( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

TEST_VALUE( 0, SX126X_STATUS_ERROR, 0 )
TEST_VALUE( 3000, SX126X_STATUS_OK, SX126X_GFSK_BW_4800 )
TEST_VALUE( 4800, SX126X_STATUS_OK, SX126X_GFSK_BW_4800 )
TEST_VALUE( 12000, SX126X_STATUS_OK, SX126X_GFSK_BW_14600 )
TEST_VALUE( 467000, SX126X_STATUS_OK, SX126X_GFSK_BW_467000 )
TEST_VALUE( 600000, SX126X_STATUS_UNKNOWN_VALUE, 0 )
void test_sx126x_get_gfsk_bw_param( uint32_t bw, sx126x_status_t status_expected, uint8_t param_expected )
{
    uint8_t         param = 0;
    sx126x_status_t status;

    status = sx126x_get_gfsk_bw_param( bw, &param );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );

    if( status == SX126X_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( param_expected, param );
    }
}

void test_sx126x_get_lora_time_on_air_in_ms( void )
{
    uint32_t                 time_on_air_in_ms = 0;
    sx126x_pkt_params_lora_t pkt_p;
    sx126x_mod_params_lora_t mod_p;
    uint32_t                 size = sizeof( assets ) / sizeof( assets[0] );

    for( uint32_t i = 0; i < size; i++ )
    {
        pkt_p.preamble_len_in_symb = assets[i].pbl;
        pkt_p.header_type          = ( assets[i].impl == 1 ) ? SX126X_LORA_PKT_IMPLICIT : SX126X_LORA_PKT_EXPLICIT;
        pkt_p.pld_len_in_bytes     = assets[i].pld;
        pkt_p.crc_is_on            = ( assets[i].crc == 1 ) ? true : false;
        pkt_p.invert_iq_is_on      = false;

        switch( assets[i].bw )
        {
        case 7812:
            mod_p.bw = SX126X_LORA_BW_007;
            break;
        case 10417:
            mod_p.bw = SX126X_LORA_BW_010;
            break;
        case 15625:
            mod_p.bw = SX126X_LORA_BW_015;
            break;
        case 20833:
            mod_p.bw = SX126X_LORA_BW_020;
            break;
        case 31250:
            mod_p.bw = SX126X_LORA_BW_031;
            break;
        case 41667:
            mod_p.bw = SX126X_LORA_BW_041;
            break;
        case 62500:
            mod_p.bw = SX126X_LORA_BW_062;
            break;
        case 125000:
            mod_p.bw = SX126X_LORA_BW_125;
            break;
        case 250000:
            mod_p.bw = SX126X_LORA_BW_250;
            break;
        case 500000:
            mod_p.bw = SX126X_LORA_BW_500;
            break;
        }
        mod_p.sf   = ( sx126x_lora_sf_t )( assets[i].sf );
        mod_p.cr   = ( sx126x_lora_cr_t )( assets[i].cr );
        mod_p.ldro = assets[i].ldro;

        time_on_air_in_ms = sx126x_get_lora_time_on_air_in_ms( &pkt_p, &mod_p );

        uint32_t time_on_air_in_ms_expected = CEILING( assets[i].toa * 1000 );

        TEST_ASSERT_UINT32_WITHIN( 1, time_on_air_in_ms_expected, time_on_air_in_ms );
    }
}

// NOTE: Same value due to the rounding to ms.
TEST_VALUE( SX126X_GFSK_PKT_FIX_LEN, SX126X_GFSK_CRC_OFF, 42 )
TEST_VALUE( SX126X_GFSK_PKT_VAR_LEN, SX126X_GFSK_CRC_1_BYTE, 42 )
TEST_VALUE( SX126X_GFSK_PKT_FIX_LEN, SX126X_GFSK_CRC_2_BYTES, 42 )
TEST_VALUE( SX126X_GFSK_PKT_VAR_LEN, SX126X_GFSK_CRC_1_BYTE_INV, 42 )
TEST_VALUE( SX126X_GFSK_PKT_VAR_LEN, SX126X_GFSK_CRC_2_BYTES_INV, 43 )
TEST_VALUE( SX126X_GFSK_PKT_VAR_LEN, 255, 42 )
void test_sx126x_get_gfsk_time_on_air_in_ms( sx126x_gfsk_pkt_len_modes_t header_type, sx126x_gfsk_crc_types_t crc_type,
                                             uint32_t time_on_air_in_ms_expected )
{
    uint32_t                 time_on_air_in_ms = 0;
    sx126x_pkt_params_gfsk_t pkt_p             = {
        .preamble_len_in_bits  = 40,
        .preamble_detector     = SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
        .sync_word_len_in_bits = 24,
        .address_filtering     = SX126X_GFSK_ADDRESS_FILTERING_DISABLE,
        .header_type           = header_type,
        .pld_len_in_bytes      = 252,
        .crc_type              = crc_type,
        .dc_free               = SX126X_GFSK_DC_FREE_WHITENING,
    };
    sx126x_mod_params_gfsk_t mod_p = {
        .br_in_bps    = 50000,
        .fdev_in_hz   = 25000,
        .pulse_shape  = SX126X_GFSK_PULSE_SHAPE_BT_1,
        .bw_dsb_param = SX126X_GFSK_BW_117300,
    };

    time_on_air_in_ms = sx126x_get_gfsk_time_on_air_in_ms( &pkt_p, &mod_p );

    TEST_ASSERT_EQUAL_UINT32( time_on_air_in_ms_expected, time_on_air_in_ms );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
