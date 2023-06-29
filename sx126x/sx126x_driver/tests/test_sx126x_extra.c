/**
 * @file      test_sx126x_extra.c
 *
 * @brief     SX126x test cases for operational modes related commands
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

void test_sx126x_cfg_rx_boosted( void )
{
    uint8_t cbuffer_expected_1[] = { 0x0D, 0x08, 0xAC };
    uint8_t cdata_expected_1[]   = { 0x96 };
    uint8_t cbuffer_expected_2[] = { 0x0D, 0x08, 0xAC };
    uint8_t cdata_expected_2[]   = { 0x94 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, cdata_expected_1, 1, 1,
                                               SX126X_HAL_STATUS_OK );

    status = sx126x_cfg_rx_boosted( radio, true );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Second case
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 1, 1,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_cfg_rx_boosted( radio, false );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_gfsk_sync_word( void )
{
    uint8_t cbuffer_expected_1[] = { 0x0D, 0x06, 0xC0 };
    uint8_t cdata_expected_1[]   = { 0x01, 0x12, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t cbuffer_expected_2[] = { 0x0D, 0x06, 0xC0 };
    uint8_t cdata_expected_2[]   = { 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0x00, 0x00 };

    uint8_t syncword_1[] = { 0x01, 0x12, 0x23 };
    uint8_t syncword_2[] = { 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD };
    uint8_t syncword_3[] = { 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xEF, 0x01, 0x23 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, cdata_expected_1, 8, 8,
                                               SX126X_HAL_STATUS_OK );

    status = sx126x_set_gfsk_sync_word( radio, syncword_1, 3 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 8, 8,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_gfsk_sync_word( radio, syncword_2, 6 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );

    /*
     * Case 3
     */
    status = sx126x_set_gfsk_sync_word( radio, syncword_2, 9 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_lora_sync_word( void )
{
    uint8_t cbuffer_expected_1_1[]   = { 0x1D, 0x07, 0x40, 0x00 };
    uint8_t rbuffer_expected_1_1[]   = { 0x00, 0x00 };
    uint8_t cbuffer_expected_1_2[]   = { 0x0D, 0x07, 0x40 };
    uint8_t cdata_expected_1_2[]     = { 0x35, 0x47 };
    uint8_t cdata_expected_1_2_bis[] = { 0x15, 0x27 };
    uint8_t cbuffer_expected_2[]     = { 0x0D, 0x07, 0x40 };
    uint8_t cdata_expected_2[]       = { 0x15, 0x27 };

    uint8_t response_1[] = { 0xA5, 0xB7 };

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 4, 4, rbuffer_expected_1_1, 2, 2,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 2 );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 3, 3, cdata_expected_1_2, 2, 2,
                                               SX126X_HAL_STATUS_OK );

    status = sx126x_set_lora_sync_word( radio, 0x34 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 1 - bis
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 4, 4, rbuffer_expected_1_1, 2, 2,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 2 );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 3, 3, cdata_expected_1_2_bis, 2, 2,
                                               SX126X_HAL_STATUS_OK );

    status = sx126x_set_lora_sync_word( radio, 0x12 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 4, 4, rbuffer_expected_1_1, 2, 2,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 2 );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 2, 2,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_lora_sync_word( radio, 0x12 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );

    /*
     * Case 3
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 4, 4, rbuffer_expected_1_1, 2, 2,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_lora_sync_word( radio, 0x12 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_gfsk_crc_seed( void )
{
    uint8_t cbuffer_expected_1[] = { 0x0D, 0x06, 0xBC };
    uint8_t cdata_expected_1[]   = { 0x12, 0x34 };
    uint8_t cbuffer_expected_2[] = { 0x0D, 0x06, 0xBC };
    uint8_t cdata_expected_2[]   = { 0xAB, 0xCD };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, cdata_expected_1, 2, 2,
                                               SX126X_HAL_STATUS_OK );

    status = sx126x_set_gfsk_crc_seed( radio, 0x1234 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 2, 2,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_gfsk_crc_seed( radio, 0xABCD );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_gfsk_crc_polynomial( void )
{
    uint8_t cbuffer_expected_1[] = { 0x0D, 0x06, 0xBE };
    uint8_t cdata_expected_1[]   = { 0x56, 0x78 };
    uint8_t cbuffer_expected_2[] = { 0x0D, 0x06, 0xBE };
    uint8_t cdata_expected_2[]   = { 0xCD, 0xEF };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, cdata_expected_1, 2, 2,
                                               SX126X_HAL_STATUS_OK );

    status = sx126x_set_gfsk_crc_polynomial( radio, 0x5678 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 2, 2,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_gfsk_crc_polynomial( radio, 0xCDEF );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_gfsk_whitening_seed( void )
{
    uint8_t cbuffer_expected_1[] = { 0x1D, 0x06, 0xB8, 0x00 };
    uint8_t rbuffer_expected_1[] = { 0x00 };
    uint8_t response_1[]         = { 0x45 };
    uint8_t cbuffer_expected_2[] = { 0x0D, 0x06, 0xB8 };
    uint8_t cdata_expected_2[]   = { 0x44 };
    uint8_t cbuffer_expected_3[] = { 0x0D, 0x06, 0xB9 };
    uint8_t cdata_expected_3[]   = { 0x23 };

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, rbuffer_expected_1, 1, 1,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 1 );

    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 1, 1,
                                               SX126X_HAL_STATUS_OK );

    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_3, 3, 3, cdata_expected_3, 1, 1,
                                               SX126X_HAL_STATUS_OK );

    status = sx126x_set_gfsk_whitening_seed( radio, 0x0023 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, rbuffer_expected_1, 1, 1,
                                              SX126X_HAL_STATUS_ERROR );
    ;

    status = sx126x_set_gfsk_whitening_seed( radio, 0x0023 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );

    /*
     * Case 3
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, rbuffer_expected_1, 1, 1,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 1 );

    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 1, 1,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_gfsk_whitening_seed( radio, 0x0023 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_config_tx_clamp( void )
{
    uint8_t cbuffer_expected_1_1[] = { 0x1D, 0x08, 0xD8, 0x00 };
    uint8_t rbuffer_expected_1_1[] = { 0x00 };
    uint8_t cbuffer_expected_1_2[] = { 0x0D, 0x08, 0xD8 };
    uint8_t cdata_expected_1_2[]   = { 0x1E };
    uint8_t cbuffer_expected_2[]   = { 0x0D, 0x07, 0x40 };
    uint8_t cdata_expected_2[]     = { 0x00 };

    uint8_t response_1[] = { 0x00 };

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 4, 4, rbuffer_expected_1_1, 1, 1,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 2 );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 3, 3, cdata_expected_1_2, 1, 1,
                                               SX126X_HAL_STATUS_OK );

    status = sx126x_cfg_tx_clamp( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 4, 4, rbuffer_expected_1_1, 1, 1,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_cfg_tx_clamp( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR )
void test_sx126x_stop_rtc( sx126x_status_t status_expected, sx126x_hal_status_t hal_status_1,
                           sx126x_hal_status_t hal_status_2, sx126x_hal_status_t hal_status_3 )
{
    uint8_t cbuffer_expected_1[] = { 0x0D, 0x09, 0x02 };
    uint8_t cdata_expected_1[]   = { 0x00 };
    uint8_t cbuffer_expected_2[] = { 0x1D, 0x09, 0x44, 0x00 };
    uint8_t rbuffer_expected_2[] = { 0x00 };
    uint8_t cbuffer_expected_3[] = { 0x0D, 0x09, 0x44 };
    uint8_t cdata_expected_3[]   = { 0x02 };

    uint8_t response_2[] = { 0x00 };

    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, cdata_expected_1, 1, 1,
                                               hal_status_1 );

    if( hal_status_1 == SX126X_HAL_STATUS_OK )
    {
        sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 4, 4, rbuffer_expected_2, 1, 1,
                                                  hal_status_2 );
        sx126x_hal_read_ReturnArrayThruPtr_data( response_2, 1 );

        if( hal_status_2 == SX126X_HAL_STATUS_OK )
        {
            sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_3, 3, 3, cdata_expected_3, 1, 1,
                                                       hal_status_3 );
        }
    }

    status = sx126x_stop_rtc( radio );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}

TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, 10 )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR, 15 )
void test_sx126x_set_ocp_value( sx126x_status_t status_expected, sx126x_hal_status_t hal_status, uint8_t ocp_value )
{
    uint8_t cbuffer_expected_1[] = { 0x0D, 0x08, 0xE7 };
    uint8_t cdata_expected_1[]   = { ocp_value };

    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, cdata_expected_1, 1, 1, hal_status );

    status = sx126x_set_ocp_value( radio, ocp_value );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}

TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, 0x08, 0x09 )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR, 0x06, 0x07 )
void test_sx126x_set_trimming_capacitor_values( sx126x_status_t status_expected, sx126x_hal_status_t hal_status,
                                                uint8_t xta, uint8_t xtb )
{
    uint8_t cbuffer_expected_1[] = { 0x0D, 0x09, 0x11 };
    uint8_t cdata_expected_1[]   = { xta, xtb };

    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, cdata_expected_1, 2, 2, hal_status );

    status = sx126x_set_trimming_capacitor_values( radio, xta, xtb );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}

TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR,
            SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_ERROR )
TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK,
            SX126X_HAL_STATUS_OK )
void test_sx126x_get_random_numbers( sx126x_status_t status_expected, sx126x_hal_status_t hal_status_1,
                                     sx126x_hal_status_t hal_status_2, sx126x_hal_status_t hal_status_3,
                                     sx126x_hal_status_t hal_status_4, sx126x_hal_status_t hal_status_5,
                                     sx126x_hal_status_t hal_status_6, sx126x_hal_status_t hal_status_7,
                                     sx126x_hal_status_t hal_status_8, sx126x_hal_status_t hal_status_9 )
{
    uint8_t cbuffer_expected_1[] = { 0x1D, 0x08, 0xE2, 0x00 };
    uint8_t rbuffer_expected_1[] = { 0x00 };
    uint8_t response_1[]         = { 0xFF };
    uint8_t cbuffer_expected_2[] = { 0x0D, 0x08, 0xE2 };
    uint8_t cdata_expected_2[]   = { 0xFE };
    uint8_t cbuffer_expected_3[] = { 0x1D, 0x08, 0xE5, 0x00 };
    uint8_t rbuffer_expected_3[] = { 0x00 };
    uint8_t response_3[]         = { 0xFF };
    uint8_t cbuffer_expected_4[] = { 0x0D, 0x08, 0xE5 };
    uint8_t cdata_expected_4[]   = { 0x7F };
    uint8_t cbuffer_expected_5[] = { 0x82, 0xFF, 0xFF, 0xFF };
    uint8_t cbuffer_expected_6[] = { 0x1D, 0x08, 0x19, 0x00 };
    uint8_t rbuffer_expected_6[] = { 0x00, 0x00, 0x00, 0x00 };
    uint8_t response_6[]         = { 0x01, 0x23, 0x45, 0x67 };
    uint8_t cbuffer_expected_7[] = { 0x80, 0x00 };
    uint8_t cbuffer_expected_8[] = { 0x0D, 0x08, 0xE2 };
    uint8_t cdata_expected_8[]   = { 0xFF };
    uint8_t cbuffer_expected_9[] = { 0x0D, 0x08, 0xE5 };
    uint8_t cdata_expected_9[]   = { 0xFF };

    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, rbuffer_expected_1, 1, 1,
                                              hal_status_1 );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 1 );

    if( hal_status_1 == SX126X_HAL_STATUS_OK )
    {
        sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 1, 1,
                                                   hal_status_2 );

        if( hal_status_2 == SX126X_HAL_STATUS_OK )
        {
            sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_3, 4, 4, rbuffer_expected_3, 1, 1,
                                                      hal_status_3 );
            sx126x_hal_read_ReturnArrayThruPtr_data( response_3, 1 );

            if( hal_status_3 == SX126X_HAL_STATUS_OK )
            {
                sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_4, 3, 3, cdata_expected_4, 1, 1,
                                                           hal_status_4 );

                if( hal_status_4 == SX126X_HAL_STATUS_OK )
                {
                    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_5, 4, 4, NULL, 0, 0,
                                                               hal_status_5 );

                    if( hal_status_5 == SX126X_HAL_STATUS_OK )
                    {
                        sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_6, 4, 4,
                                                                  rbuffer_expected_6, 4, 4, hal_status_6 );
                        sx126x_hal_read_ReturnArrayThruPtr_data( response_6, 4 );

                        if( hal_status_6 == SX126X_HAL_STATUS_OK )
                        {
                            sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_7, 2, 2, NULL, 0, 0,
                                                                       hal_status_7 );

                            if( hal_status_7 == SX126X_HAL_STATUS_OK )
                            {
                                sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_8, 3, 3,
                                                                           cdata_expected_8, 1, 1, hal_status_8 );

                                if( hal_status_8 == SX126X_HAL_STATUS_OK )
                                {
                                    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_9, 3, 3,
                                                                               cdata_expected_9, 1, 1, hal_status_9 );
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    uint32_t numbers[10] = { 0 };

    status = sx126x_get_random_numbers( radio, numbers, 1 );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );

    if( status == SX126X_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT32( 0x67452301, numbers[0] );
    }
}

TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, 0, 4, 0x0123, 0x4567, 0x89AB, 0xCDEF, 4,
            0x0123, 0x4567, 0x89AB, 0xCDEF )
TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, 2, 4, 0x0123, 0x89AB, 0x4567, 0xCDEF, 2,
            0x0123, 0x4567, 0x0000, 0x0000 )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR, 2, 4, 0x0123, 0x89AB, 0x4567, 0xCDEF, 2,
            0x0123, 0x4567, 0x0000, 0x0000 )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, 0, 1, 0xA123, 0x4567, 0x89AB, 0xCDEF, 4,
            0x0123, 0x4567, 0x89AB, 0xCDEF )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, 0, 2, 0xA123, 0xA567, 0x89AB, 0xCDEF, 3,
            0x0123, 0x4567, 0x89AB, 0xCDEF )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK, 0, 2, 0xA123, 0xA567, 0x89AB, 0xCDEF, 3,
            0x0123, 0x4567, 0x89AB, 0xCDEF )
void test_sx126x_add_registers_to_retention_list( sx126x_status_t status_expected, sx126x_hal_status_t hal_status_1,
                                                  sx126x_hal_status_t hal_status_2, uint8_t nb_of_register_new,
                                                  uint8_t nb_of_register_to_add, uint16_t register_address_to_add_1,
                                                  uint16_t register_address_to_add_2,
                                                  uint16_t register_address_to_add_3,
                                                  uint16_t register_address_to_add_4, uint8_t nb_of_registers,
                                                  uint16_t register_address_1, uint16_t register_address_2,
                                                  uint16_t register_address_3, uint16_t register_address_4 )
{
    uint8_t cbuffer_expected_1[4] = { 0x1D, 0x02, 0x9F, 0x00 };
    uint8_t rbuffer_expected_1[9] = { 0x00 };
    uint8_t response_1[9]         = { nb_of_registers,
                              ( uint8_t )( register_address_1 >> 8 ),
                              ( uint8_t )( register_address_1 >> 0 ),
                              ( uint8_t )( register_address_2 >> 8 ),
                              ( uint8_t )( register_address_2 >> 0 ),
                              ( uint8_t )( register_address_3 >> 8 ),
                              ( uint8_t )( register_address_3 >> 0 ),
                              ( uint8_t )( register_address_4 >> 8 ),
                              ( uint8_t )( register_address_4 >> 0 ) };
    uint8_t cbuffer_expected_2[]  = { 0x0D, 0x02, 0x9F };
    uint8_t cdata_expected_2[9]   = { nb_of_registers + nb_of_register_new,   ( uint8_t )( register_address_1 >> 8 ),
                                    ( uint8_t )( register_address_1 >> 0 ), ( uint8_t )( register_address_2 >> 8 ),
                                    ( uint8_t )( register_address_2 >> 0 ), ( uint8_t )( register_address_3 >> 8 ),
                                    ( uint8_t )( register_address_3 >> 0 ), ( uint8_t )( register_address_4 >> 8 ),
                                    ( uint8_t )( register_address_4 >> 0 ) };

    uint16_t register_to_add[4] = {
        register_address_to_add_1,
        register_address_to_add_2,
        register_address_to_add_3,
        register_address_to_add_4,
    };

    do
    {
        sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, rbuffer_expected_1, 9, 9,
                                                  hal_status_1 );
        sx126x_hal_read_IgnoreArg_data( );
        sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 9 );

        if( hal_status_1 != SX126X_HAL_STATUS_OK )
        {
            break;
        }

        if( nb_of_register_new == 0 )
        {
            break;
        }

        cdata_expected_2[5] = 0x89;
        cdata_expected_2[6] = 0xAB;
        cdata_expected_2[7] = 0xCD;
        cdata_expected_2[8] = 0xEF;

        sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 9, 9,
                                                   hal_status_2 );

        if( hal_status_2 != SX126X_HAL_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    status = sx126x_add_registers_to_retention_list( radio, register_to_add, nb_of_register_to_add );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}

TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK )
void test_sx126x_init_retention_list( sx126x_status_t status_expected, sx126x_hal_status_t hal_status_1,
                                      sx126x_hal_status_t hal_status_2 )
{
    uint8_t cbuffer_expected_1[4] = { 0x1D, 0x02, 0x9F, 0x00 };
    uint8_t rbuffer_expected_1[9] = { 0x00 };
    uint8_t response_1[9]         = { 0, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
    uint8_t cbuffer_expected_2[3] = { 0x0D, 0x02, 0x9F };
    uint8_t cdata_expected_2[9]   = { 3, 0x08, 0xAC, 0x08, 0x89, 0x07, 0x36, 0xCD, 0xEF };

    do
    {
        sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, rbuffer_expected_1, 9, 9,
                                                  hal_status_1 );
        sx126x_hal_read_IgnoreArg_data( );
        sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 9 );

        if( hal_status_1 != SX126X_HAL_STATUS_OK )
        {
            break;
        }

        sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 9, 9,
                                                   hal_status_2 );

        if( hal_status_2 != SX126X_HAL_STATUS_OK )
        {
            break;
        }

    } while( 0 );

    status = sx126x_init_retention_list( radio );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}

TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR )
void test_sx126x_get_lora_params_from_header( sx126x_status_t status_expected, sx126x_hal_status_t hal_status_1, sx126x_hal_status_t hal_status_2 )
{
    uint8_t cbuffer_expected_1[] = { 0x1D, 0x07, 0x49, 0x00 };
    uint8_t rbuffer_expected_1[] = { 0x00 };
    uint8_t response_1[]         = { 0x40 };
    uint8_t cbuffer_expected_2[] = { 0x1D, 0x07, 0x6B, 0x00 };
    uint8_t rbuffer_expected_2[] = { 0x00 };
    uint8_t response_2[]         = { 0xFF };

    sx126x_lora_cr_t cr;
    bool crc_is_on;

    do
    {
        sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, rbuffer_expected_1, 1, 1,
                                                  hal_status_1 );
        sx126x_hal_read_IgnoreArg_data( );
        sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 1 );

        if( hal_status_1 != SX126X_HAL_STATUS_OK )
        {
            break;
        }

        sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 4, 4, rbuffer_expected_2, 1, 1,
                                                  hal_status_2 );
        sx126x_hal_read_IgnoreArg_data( );
        sx126x_hal_read_ReturnArrayThruPtr_data( response_2, 1 );

        if( hal_status_2 != SX126X_HAL_STATUS_OK )
        {
            break;
        }
    } while(0);

    status = sx126x_get_lora_params_from_header( radio, &cr, &crc_is_on );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );

    if( status == SX126X_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( SX126X_LORA_CR_4_8, cr );
        TEST_ASSERT_EQUAL_UINT8( true, crc_is_on );
    }
}


TEST_VALUE( SX126X_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR, SX126X_HAL_STATUS_OK )
TEST_VALUE( SX126X_STATUS_ERROR, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_OK, SX126X_HAL_STATUS_ERROR )
void test_sx126x_handle_rx_done( sx126x_status_t status_expected, sx126x_hal_status_t hal_status_1,
                           sx126x_hal_status_t hal_status_2, sx126x_hal_status_t hal_status_3 )
{
    uint8_t cbuffer_expected_1[] = { 0x0D, 0x09, 0x02 };
    uint8_t cdata_expected_1[]   = { 0x00 };
    uint8_t cbuffer_expected_2[] = { 0x1D, 0x09, 0x44, 0x00 };
    uint8_t rbuffer_expected_2[] = { 0x00 };
    uint8_t cbuffer_expected_3[] = { 0x0D, 0x09, 0x44 };
    uint8_t cdata_expected_3[]   = { 0x02 };

    uint8_t response_2[] = { 0x00 };

    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, cdata_expected_1, 1, 1,
                                               hal_status_1 );

    if( hal_status_1 == SX126X_HAL_STATUS_OK )
    {
        sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 4, 4, rbuffer_expected_2, 1, 1,
                                                  hal_status_2 );
        sx126x_hal_read_ReturnArrayThruPtr_data( response_2, 1 );

        if( hal_status_2 == SX126X_HAL_STATUS_OK )
        {
            sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_3, 3, 3, cdata_expected_3, 1, 1,
                                                       hal_status_3 );
        }
    }

    status = sx126x_handle_rx_done( radio );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
