/**
 * @file      test_sx126x_modulation_packet.c
 *
 * @brief     SX126x test cases for modulation and packet related commands
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

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

void test_sx126x_set_rf_freq( void )
{
    uint8_t cbuffer_expected_1[] = { 0x86, 0x36, 0x40, 0x00, 0x00 };
    uint8_t cbuffer_expected_2[] = { 0x86, 0x39, 0x30, 0x00, 0x00 };
    uint8_t cbuffer_expected_3[] = { 0x86, 0x09, 0x60, 0x00, 0x00 };

    uint32_t freq_in_hz = 868000000;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 5, 5, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    freq_in_hz = 868000000;
    status     = sx126x_set_rf_freq( radio, freq_in_hz );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 5, 5, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    freq_in_hz = 915000000;
    status     = sx126x_set_rf_freq( radio, freq_in_hz );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );

    /*
     * Case 3
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_3, 5, 5, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    freq_in_hz = 150000000;
    status     = sx126x_set_rf_freq( radio, freq_in_hz );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_pkt_type( void )
{
    uint8_t cbuffer_expected_1[] = { 0x8A, 0x01 };
    uint8_t cbuffer_expected_2[] = { 0x8A, 0x00 };

    sx126x_pkt_type_t packet_type;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    packet_type = SX126X_PKT_TYPE_LORA;
    status      = sx126x_set_pkt_type( radio, packet_type );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 2, 2, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    packet_type = SX126X_PKT_TYPE_GFSK;
    status      = sx126x_set_pkt_type( radio, packet_type );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_get_pkt_type( void )
{
    uint8_t cbuffer_expected[] = { 0x11, 0x00 };
    uint8_t rbuffer_expected[] = { 0x00 };
    uint8_t response_1[]       = { 0x00 };

    sx126x_pkt_type_t packet_type = 0x00;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 1, 1,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 1 );

    status = sx126x_get_pkt_type( radio, &packet_type );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_UINT8( SX126X_PKT_TYPE_GFSK, packet_type );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 1, 1,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_pkt_type( radio, &packet_type );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_tx_params( void )
{
    uint8_t cbuffer_expected_1[] = { 0x8E, 0xFB, 0x07 };
    uint8_t cbuffer_expected_2[] = { 0x8E, 0x0E, 0x02 };

    int8_t             pwr_in_dbm;
    sx126x_ramp_time_t ramp_time;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    pwr_in_dbm = -5;
    ramp_time  = SX126X_RAMP_3400_US;
    status     = sx126x_set_tx_params( radio, pwr_in_dbm, ramp_time );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    pwr_in_dbm = 14;
    ramp_time  = SX126X_RAMP_40_US;
    status     = sx126x_set_tx_params( radio, pwr_in_dbm, ramp_time );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_gfsk_mod_params( void )
{
    uint8_t cbuffer_expected_1_1[] = { 0x8B, 0x00, 0x1A, 0xAA, 0x0A, 0x11, 0x00, 0xCC, 0xCD };
    uint8_t cbuffer_expected_1_2[] = { 0x1D, 0x08, 0x89, 0x00 };
    uint8_t rbuffer_expected_1_2[] = { 0x00 };
    uint8_t cbuffer_expected_1_3[] = { 0x0D, 0x08, 0x89 };
    uint8_t cdata_expected_1_3[]   = { 0x04 };
    uint8_t cbuffer_expected_2[]   = { 0x8B, 0x00, 0x1A, 0xAA, 0x08, 0x09, 0x03, 0x33, 0x33 };

    uint8_t response_1_2[] = { 0x00 };

    sx126x_mod_params_gfsk_t mod_params;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 9, 9, NULL, 0, 0, SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 4, 4, rbuffer_expected_1_2, 1, 1,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1_2, 1 );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_3, 3, 3, cdata_expected_1_3, 1, 1,
                                               SX126X_HAL_STATUS_OK );

    mod_params.br_in_bps    = 150000;
    mod_params.bw_dsb_param = SX126X_GFSK_BW_373600;
    mod_params.fdev_in_hz   = 50000;
    mod_params.pulse_shape  = SX126X_GFSK_PULSE_SHAPE_BT_07;
    status                  = sx126x_set_gfsk_mod_params( radio, &mod_params );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 9, 9, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    mod_params.br_in_bps    = 150000;
    mod_params.bw_dsb_param = SX126X_GFSK_BW_467000;
    mod_params.fdev_in_hz   = 200000;
    mod_params.pulse_shape  = SX126X_GFSK_PULSE_SHAPE_BT_03;
    status                  = sx126x_set_gfsk_mod_params( radio, &mod_params );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_LORA_BW_125,
            SX126X_LORA_SF6, SX126X_LORA_CR_4_6, 0x00, 0x00, 0x04 )
TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_LORA_BW_500,
            SX126X_LORA_SF6, SX126X_LORA_CR_4_6, 0x00, 0xFF, 0xFB )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR,
            SX126X_LORA_BW_125, SX126X_LORA_SF6, SX126X_LORA_CR_4_6, 0x00, 0x00, 0x04 )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK,
            SX126X_LORA_BW_125, SX126X_LORA_SF6, SX126X_LORA_CR_4_6, 0x00, 0x00, 0x04 )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_LORA_BW_125, SX126X_LORA_SF6, SX126X_LORA_CR_4_6, 0x00, 0x00, 0x04 )
void test_sx126x_set_lora_mod_params( sx126x_status_t status_expected, sx126x_hal_status_t hal_status_1,
                                      sx126x_hal_status_t hal_status_2, sx126x_hal_status_t hal_status_3,
                                      sx126x_lora_bw_t bw, sx126x_lora_sf_t sf, sx126x_lora_cr_t cr, uint8_t ldro,
                                      uint8_t reg_init_value, uint8_t reg_expected_value )
{
    uint8_t cbuffer_expected_1_1[] = { 0x8B, sf, bw, cr, ldro };
    uint8_t cbuffer_expected_1_2[] = { 0x1D, 0x08, 0x89, 0x00 };
    uint8_t rbuffer_expected_1_2[] = { 0x00 };
    uint8_t cbuffer_expected_1_3[] = { 0x0D, 0x08, 0x89 };
    uint8_t cdata_expected_1_3[]   = { reg_expected_value };

    uint8_t response_1_2[] = { reg_init_value };

    sx126x_mod_params_lora_t mod_params;

    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 5, 5, NULL, 0, 0, hal_status_1 );
    if( hal_status_1 == SX126X_STATUS_OK )
    {
        sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 4, 4, rbuffer_expected_1_2, 1, 1,
                                                  hal_status_2 );
        sx126x_hal_read_ReturnArrayThruPtr_data( response_1_2, 1 );
        if( hal_status_2 == SX126X_STATUS_OK )
        {
            sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_3, 3, 3, cdata_expected_1_3, 1, 1,
                                                       hal_status_3 );
        }
    }

    mod_params.bw   = bw;
    mod_params.cr   = cr;
    mod_params.ldro = ldro;
    mod_params.sf   = sf;
    status          = sx126x_set_lora_mod_params( radio, &mod_params );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}

void test_sx126x_set_gfsk_pkt_params( void )
{
    uint8_t cbuffer_expected_1[] = { 0x8C, 0x01, 0x89, 0x05, 0x20, 0x02, 0x01, 0xAB, 0x06, 0x01 };
    uint8_t cbuffer_expected_2[] = { 0x8C, 0xAB, 0xCD, 0x04, 0x40, 0x01, 0x00, 0xEF, 0x01, 0x00 };

    sx126x_pkt_params_gfsk_t pkt_params;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 10, 10, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    pkt_params.address_filtering     = SX126X_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES;
    pkt_params.crc_type              = SX126X_GFSK_CRC_2_BYTES_INV;
    pkt_params.dc_free               = SX126X_GFSK_DC_FREE_WHITENING;
    pkt_params.header_type           = SX126X_GFSK_PKT_VAR_LEN;
    pkt_params.preamble_len_in_bits  = 0x0189;
    pkt_params.preamble_detector     = SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS;
    pkt_params.pld_len_in_bytes      = 0xAB;
    pkt_params.sync_word_len_in_bits = 0x20;
    status                           = sx126x_set_gfsk_pkt_params( radio, &pkt_params );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 10, 10, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    pkt_params.address_filtering     = SX126X_GFSK_ADDRESS_FILTERING_NODE_ADDRESS;
    pkt_params.crc_type              = SX126X_GFSK_CRC_OFF;
    pkt_params.dc_free               = SX126X_GFSK_DC_FREE_OFF;
    pkt_params.header_type           = SX126X_GFSK_PKT_FIX_LEN;
    pkt_params.preamble_len_in_bits  = 0xABCD;
    pkt_params.preamble_detector     = SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS;
    pkt_params.pld_len_in_bytes      = 0xEF;
    pkt_params.sync_word_len_in_bits = 0x40;
    status                           = sx126x_set_gfsk_pkt_params( radio, &pkt_params );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_lora_pkt_params( void )
{
    uint8_t cbuffer_expected_1_1[] = { 0x8C, 0xAB, 0xCD, 0x01, 0xA5, 0x01, 0x00 };
    uint8_t cbuffer_expected_1_2[] = { 0x1D, 0x07, 0x36, 0x00 };
    uint8_t rbuffer_expected_1_2[] = { 0x00 };
    uint8_t cbuffer_expected_1_3[] = { 0x0D, 0x07, 0x36 };
    uint8_t cdata_expected_1_3[]   = { 0x04 };

    uint8_t cbuffer_expected_2_1[] = { 0x8C, 0xAB, 0xCD, 0x01, 0xA5, 0x01, 0x01 };
    uint8_t cbuffer_expected_2_2[] = { 0x1D, 0x07, 0x36, 0x00 };
    uint8_t rbuffer_expected_2_2[] = { 0x00 };
    uint8_t cbuffer_expected_2_3[] = { 0x0D, 0x07, 0x36 };
    uint8_t cdata_expected_2_3[]   = { 0x00 };

    uint8_t cbuffer_expected_3[] = { 0x8C, 0xAB, 0xCD, 0x01, 0xA5, 0x00, 0x01 };

    uint8_t response_1_2[] = { 0x00 };
    uint8_t response_2_2[] = { 0x04 };

    sx126x_pkt_params_lora_t pkt_params;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 7, 7, NULL, 0, 0, SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 4, 4, rbuffer_expected_1_2, 1, 1,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1_2, 1 );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_3, 3, 3, cdata_expected_1_3, 1, 1,
                                               SX126X_HAL_STATUS_OK );

    pkt_params.crc_is_on            = true;
    pkt_params.header_type          = SX126X_LORA_PKT_IMPLICIT;
    pkt_params.invert_iq_is_on      = false;
    pkt_params.preamble_len_in_symb = 0xABCD;
    pkt_params.pld_len_in_bytes     = 0xA5;
    status                          = sx126x_set_lora_pkt_params( radio, &pkt_params );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2_1, 7, 7, NULL, 0, 0, SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2_2, 4, 4, rbuffer_expected_2_2, 1, 1,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_2_2, 1 );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2_3, 3, 3, cdata_expected_2_3, 1, 1,
                                               SX126X_HAL_STATUS_OK );

    pkt_params.crc_is_on            = true;
    pkt_params.header_type          = SX126X_LORA_PKT_IMPLICIT;
    pkt_params.invert_iq_is_on      = true;
    pkt_params.preamble_len_in_symb = 0xABCD;
    pkt_params.pld_len_in_bytes     = 0xA5;
    status                          = sx126x_set_lora_pkt_params( radio, &pkt_params );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2 bis
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2_1, 7, 7, NULL, 0, 0, SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2_2, 4, 4, rbuffer_expected_2_2, 1, 1,
                                              SX126X_HAL_STATUS_ERROR );

    pkt_params.crc_is_on            = true;
    pkt_params.header_type          = SX126X_LORA_PKT_IMPLICIT;
    pkt_params.invert_iq_is_on      = true;
    pkt_params.preamble_len_in_symb = 0xABCD;
    pkt_params.pld_len_in_bytes     = 0xA5;
    status                          = sx126x_set_lora_pkt_params( radio, &pkt_params );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );

    /*
     * Case 3
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_3, 7, 7, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    pkt_params.crc_is_on            = false;
    pkt_params.header_type          = SX126X_LORA_PKT_IMPLICIT;
    pkt_params.invert_iq_is_on      = true;
    pkt_params.preamble_len_in_symb = 0xABCD;
    pkt_params.pld_len_in_bytes     = 0xA5;
    status                          = sx126x_set_lora_pkt_params( radio, &pkt_params );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

TEST_VALUE( 0x12, 0x34, SX126X_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( 0x98, 0x76, SX126X_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( 0x98, 0x76, SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR )
void test_sx126x_set_gfsk_pkt_address( uint8_t node_address, uint8_t broadcast_address, sx126x_status_t status_expected,
                                       sx126x_hal_status_t status_hal )
{
    sx126x_status_t status;
    const uint8_t   cbuffer_expected[3] = { 0x0D, 0x06, 0xCD };
    const uint8_t   cdata_expected[2]   = { node_address, broadcast_address };

    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 3, 3, cdata_expected, 2, 2, status_hal );

    status = sx126x_set_gfsk_pkt_address( radio, node_address, broadcast_address );
    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}

void test_sx126x_set_cad_params( void )
{
    uint8_t cbuffer_expected_1[] = { 0x88, 0x03, 0x34, 0x12, 0x10, 0xAB, 0xCD, 0xEF };
    uint8_t cbuffer_expected_2[] = { 0x88, 0x02, 0xCD, 0xAB, 0x00, 0x23, 0x45, 0x67 };

    sx126x_cad_params_t cad_params;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 8, 8, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    cad_params.cad_detect_min  = 0x12;
    cad_params.cad_detect_peak = 0x34;
    cad_params.cad_exit_mode   = SX126X_CAD_LBT;
    cad_params.cad_symb_nb     = SX126X_CAD_08_SYMB;
    cad_params.cad_timeout     = 0x89ABCDEF;
    status                     = sx126x_set_cad_params( radio, &cad_params );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 8, 8, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    cad_params.cad_detect_min  = 0xAB;
    cad_params.cad_detect_peak = 0xCD;
    cad_params.cad_exit_mode   = SX126X_CAD_ONLY;
    cad_params.cad_symb_nb     = SX126X_CAD_04_SYMB;
    cad_params.cad_timeout     = 0x01234567;
    status                     = sx126x_set_cad_params( radio, &cad_params );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_buffer_base_addr( void )
{
    uint8_t cbuffer_expected_1[] = { 0x8F, 0xAB, 0xEF };
    uint8_t cbuffer_expected_2[] = { 0x8F, 0xCD, 0x01 };
    uint8_t tx_base_address;
    uint8_t rx_base_address;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    tx_base_address = 0xAB;
    rx_base_address = 0xEF;
    status          = sx126x_set_buffer_base_address( radio, tx_base_address, rx_base_address );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    tx_base_address = 0xCD;
    rx_base_address = 0x01;
    status          = sx126x_set_buffer_base_address( radio, tx_base_address, rx_base_address );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_lora_symb_nb_timeout( void )
{
    uint8_t cbuffer_expected_1_1[] = { 0xA0, 0x90 };
    uint8_t cbuffer_expected_1_2[] = { 0x0D, 0x07, 0x06 };
    uint8_t cdata_expected_1_2[]   = { 0x91 };
    uint8_t cbuffer_expected_2_1[] = { 0xA0, 0x40 };
    uint8_t cbuffer_expected_2_2[] = { 0x0D, 0x07, 0x06 };
    uint8_t cdata_expected_2_2[]   = { 0x41 };
    uint8_t cbuffer_expected_3[]   = { 0xA0, 0x02 };
    uint8_t cbuffer_expected_4_1[] = { 0xA0, 0x00 };
    uint8_t cbuffer_expected_4_2[] = { 0x0D, 0x07, 0x06 };
    uint8_t cdata_expected_4_2[]   = { 0x41 };

    uint8_t symb_num;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 3, 3, cdata_expected_1_2, 1, 1,
                                               SX126X_HAL_STATUS_OK );

    symb_num = 140;
    status   = sx126x_set_lora_symb_nb_timeout( radio, symb_num );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2_2, 3, 3, cdata_expected_2_2, 1, 1,
                                               SX126X_HAL_STATUS_OK );

    symb_num = 63;
    status   = sx126x_set_lora_symb_nb_timeout( radio, symb_num );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 3
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_3, 2, 2, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    symb_num = 1;
    status   = sx126x_set_lora_symb_nb_timeout( radio, symb_num );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );

    /*
     * Case 4
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_4_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    symb_num = 0;
    status   = sx126x_set_lora_symb_nb_timeout( radio, symb_num );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
