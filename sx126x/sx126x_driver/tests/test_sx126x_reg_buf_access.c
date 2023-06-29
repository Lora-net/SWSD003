/**
 * @file      test_sx126x_reg_buf_access.c
 *
 * @brief     SX126x test cases for register and buffer access related commands
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

void test_sx126x_write_register( void )
{
    uint8_t  cbuffer_expected_1[] = { 0x0D, 0x01, 0x12 };
    uint8_t  cdata_expected_1[]   = { 0x23 };
    uint8_t  cbuffer_expected_2[] = { 0x0D, 0xAB, 0xBC };
    uint8_t  cdata_expected_2[]   = { 0xDE, 0xF0 };
    uint16_t address_1            = 0x0112;
    uint16_t address_2            = 0xABBC;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, cdata_expected_1, 1, 1,
                                               SX126X_HAL_STATUS_OK );

    status = sx126x_write_register( radio, address_1, cdata_expected_1, 1 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, cdata_expected_2, 2, 2,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_write_register( radio, address_2, cdata_expected_2, 2 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_read_register( void )
{
    uint8_t  cbuffer_expected_1[]   = { 0x1D, 0x45, 0x67, 0x00 };
    uint8_t  rbuffer_expected_1[]   = { 0x00, 0x00, 0x00, 0x00 };
    uint8_t  cbuffer_expected_2[]   = { 0x1D, 0xAB, 0xCD, 0x00 };
    uint8_t  rbuffer_expected_2[]   = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t  response_1[]           = { 0x89, 0xAB, 0xCD, 0xEF };
    uint8_t  response_expected_1[4] = { 0x00 };
    uint8_t  response_expected_2[5] = { 0x00 };
    uint16_t address_1              = 0x4567;
    uint16_t address_2              = 0xABCD;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, rbuffer_expected_1, 4, 4,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 4 );

    status = sx126x_read_register( radio, address_1, response_expected_1, 4 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( response_1, response_expected_1, 4 );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 4, 4, rbuffer_expected_2, 5, 5,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_read_register( radio, address_2, response_expected_2, 5 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_write_buffer( void )
{
    uint8_t cbuffer_expected_1[] = { 0x0E, 0x01 };
    uint8_t cdata_expected_1[]   = { 0x12, 0x23, 0x34 };
    uint8_t cbuffer_expected_2[] = { 0x0E, 0xAB };
    uint8_t cdata_expected_2[]   = { 0xBC, 0xDE, 0xEF, 0xF0 };
    uint8_t offset_1             = 0x01;
    uint8_t offset_2             = 0xAB;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 2, 2, cdata_expected_1, 3, 3,
                                               SX126X_HAL_STATUS_OK );

    status = sx126x_write_buffer( radio, offset_1, cdata_expected_1, 3 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 2, 2, cdata_expected_2, 4, 4,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_write_buffer( radio, offset_2, cdata_expected_2, 4 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_read_buffer( void )
{
    uint8_t cbuffer_expected_1[] = { 0x1E, 0x45, 0x00 };
    uint8_t rbuffer_expected_1[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t cbuffer_expected_2[] = { 0x1E, 0xAB, 0x00 };
    uint8_t rbuffer_expected_2[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    uint8_t response_1[] = { 0x01, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x98, 0x9A, 0xAB, 0xBC };

    uint8_t  response_expected_1[9]  = { 0 };
    uint8_t  response_expected_2[10] = { 0 };
    uint16_t offset_1                = 0x45;
    uint16_t offset_2                = 0xAB;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, rbuffer_expected_1, 9, 9,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1, 9 );

    status = sx126x_read_buffer( radio, offset_1, response_expected_1, 9 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( response_1, response_expected_1, 9 );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, rbuffer_expected_2, 10, 10,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_read_buffer( radio, offset_2, response_expected_2, 10 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
