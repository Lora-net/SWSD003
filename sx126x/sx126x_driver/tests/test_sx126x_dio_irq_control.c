/**
 * @file      test_sx126x_dio_irq_control.c
 *
 * @brief     SX126x test cases for DIO and interrupt related commands
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

void test_sx126x_get_irq_status( void )
{
    uint8_t           cbuffer_expected[] = { 0x12, 0x00 };
    uint8_t           rbuffer_expected[] = { 0x00, 0x00 };
    uint8_t           response[]         = { 0x03, 0xFF };
    sx126x_irq_mask_t irq_status;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 2, 2,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response, 2 );

    status = sx126x_get_irq_status( radio, &irq_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_UINT16( 0x03FF, irq_status );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 2, 2,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_irq_status( radio, &irq_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_get_and_clear_irq_status( void )
{
    uint8_t cbuffer_expected_1_1[] = { 0x12, 0x00 };
    uint8_t rbuffer_expected_1_1[] = { 0x00, 0x00 };
    uint8_t response_1_1[]         = { 0x03, 0xFF };
    uint8_t response_5_1[]         = { 0x00, 0x00 };

    uint8_t           cbuffer_expected_1_2[] = { 0x02, 0x03, 0xFF };
    uint16_t          irq_mask;
    sx126x_irq_mask_t irq_status;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 2, 2, rbuffer_expected_1_1, 2, 2,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1_1, 2 );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_get_and_clear_irq_status( radio, &irq_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_UINT16( 0x03FF, irq_status );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 2, 2, rbuffer_expected_1_1, 2, 2,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1_1, 2 );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 3, 3, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_and_clear_irq_status( radio, &irq_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );

    /*
     * Case 3
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 2, 2, rbuffer_expected_1_1, 2, 2,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_and_clear_irq_status( radio, &irq_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );

    /*
     * Case 4
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 2, 2, rbuffer_expected_1_1, 2, 2,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_1_1, 2 );
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_2, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_get_and_clear_irq_status( radio, NULL );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 5
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1_1, 2, 2, rbuffer_expected_1_1, 2, 2,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response_5_1, 2 );

    status = sx126x_get_and_clear_irq_status( radio, &irq_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
}

void test_sx126x_set_dio2_as_rf_sw_ctrl( void )
{
    uint8_t cbuffer_expected_1[] = { 0x9D, 0x01 };
    uint8_t cbuffer_expected_2[] = { 0x9D, 0x00 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_dio2_as_rf_sw_ctrl( radio, true );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 2, 2, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_dio2_as_rf_sw_ctrl( radio, false );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_dio3_as_tcxo_ctrl( void )
{
    uint8_t                     cbuffer_expected_1[] = { 0x97, 0x04, 0x23, 0x45, 0x67 };
    uint8_t                     cbuffer_expected_2[] = { 0x97, 0x07, 0xAB, 0xCD, 0xEF };
    sx126x_tcxo_ctrl_voltages_t tcxo_ctrl_voltage;
    uint32_t                    timeout;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 5, 5, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    timeout           = 0x01234567;
    tcxo_ctrl_voltage = SX126X_TCXO_CTRL_2_4V;
    status            = sx126x_set_dio3_as_tcxo_ctrl( radio, tcxo_ctrl_voltage, timeout );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 5, 5, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    timeout           = 0x89ABCDEF;
    tcxo_ctrl_voltage = SX126X_TCXO_CTRL_3_3V;
    status            = sx126x_set_dio3_as_tcxo_ctrl( radio, tcxo_ctrl_voltage, timeout );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
