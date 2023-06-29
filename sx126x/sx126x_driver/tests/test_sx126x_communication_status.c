/**
 * @file      test_sx126x_communication_status.c
 *
 * @brief     SX126x test cases for communication status related commands
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

void test_sx126x_get_status( void )
{
    uint8_t cbuffer_expected[]     = { 0xC0 };
    uint8_t rbuffer_expected[]     = { 0x00 };
    uint8_t cbuffer_out_expected[] = { 0x3A };

    sx126x_chip_status_t chip_status;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 1, 1, rbuffer_expected, 1, 1,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( cbuffer_out_expected, 1 );

    status = sx126x_get_status( radio, &chip_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_UINT8( SX126X_CHIP_MODE_STBY_XOSC, chip_status.chip_mode );
    TEST_ASSERT_EQUAL_UINT8( SX126X_CMD_STATUS_CMD_EXEC_FAILURE, chip_status.cmd_status );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 1, 1, rbuffer_expected, 1, 1,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_status( radio, &chip_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_get_rx_buffer_status( void )
{
    uint8_t cbuffer_expected[] = { 0x13, 0x00 };
    uint8_t rbuffer_expected[] = { 0x00, 0x00 };
    uint8_t response[]         = { 0xAB, 0xCD };

    sx126x_rx_buffer_status_t rx_buffer_status;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 2, 2,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response, 2 );

    status = sx126x_get_rx_buffer_status( radio, &rx_buffer_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_UINT8( 0xAB, rx_buffer_status.pld_len_in_bytes );
    TEST_ASSERT_EQUAL_UINT8( 0xCD, rx_buffer_status.buffer_start_pointer );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 2, 2,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_rx_buffer_status( radio, &rx_buffer_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_get_gfsk_pkt_status( void )
{
    uint8_t cbuffer_expected[] = { 0x14, 0x00 };
    uint8_t rbuffer_expected[] = { 0x00, 0x00, 0x00 };
    uint8_t response[]         = { 0x56, 0x67, 0x78 };

    sx126x_pkt_status_gfsk_t pkt_status = { 0 };

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 3, 3,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response, 3 );

    status = sx126x_get_gfsk_pkt_status( radio, &pkt_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_FALSE( pkt_status.rx_status.pkt_sent );
    TEST_ASSERT_TRUE( pkt_status.rx_status.pkt_received );
    TEST_ASSERT_TRUE( pkt_status.rx_status.abort_error );
    TEST_ASSERT_FALSE( pkt_status.rx_status.length_error );
    TEST_ASSERT_TRUE( pkt_status.rx_status.crc_error );
    TEST_ASSERT_FALSE( pkt_status.rx_status.adrs_error );
    TEST_ASSERT_EQUAL_INT8( -52, pkt_status.rssi_sync );
    TEST_ASSERT_EQUAL_INT8( -60, pkt_status.rssi_avg );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 3, 3,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_gfsk_pkt_status( radio, &pkt_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_get_lora_pkt_status( void )
{
    uint8_t cbuffer_expected[] = { 0x14, 0x00 };
    uint8_t rbuffer_expected[] = { 0x00, 0x00, 0x00 };
    uint8_t response[]         = { 0x23, 0x34, 0x45 };

    sx126x_pkt_status_lora_t pkt_status;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 3, 3,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response, 3 );

    status = sx126x_get_lora_pkt_status( radio, &pkt_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_INT8( -18, pkt_status.rssi_pkt_in_dbm );
    TEST_ASSERT_EQUAL_INT8( 13, pkt_status.snr_pkt_in_db );
    TEST_ASSERT_EQUAL_INT8( -35, pkt_status.signal_rssi_pkt_in_dbm );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 3, 3,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_lora_pkt_status( radio, &pkt_status );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_get_rssi_inst( void )
{
    uint8_t cbuffer_expected[] = { 0x15, 0x00 };
    uint8_t rbuffer_expected[] = { 0x00 };
    uint8_t response[]         = { 0x78 };
    int16_t rssi_in_dbm;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 1, 1,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response, 1 );

    status = sx126x_get_rssi_inst( radio, &rssi_in_dbm );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_INT8( -60, rssi_in_dbm );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 1, 1,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_rssi_inst( radio, &rssi_in_dbm );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_get_gfsk_stats( void )
{
    uint8_t cbuffer_expected[] = { 0x10, 0x00 };
    uint8_t rbuffer_expected[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t response[]         = { 0xCD, 0xDE, 0xEF, 0xF0, 0x01, 0x12 };

    sx126x_stats_gfsk_t stats;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 6, 6,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response, 6 );

    status = sx126x_get_gfsk_stats( radio, &stats );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_UINT16( 0xCDDE, stats.nb_pkt_received );
    TEST_ASSERT_EQUAL_UINT16( 0xEFF0, stats.nb_pkt_crc_error );
    TEST_ASSERT_EQUAL_UINT16( 0x0112, stats.nb_pkt_len_error );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 6, 6,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_gfsk_stats( radio, &stats );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_get_lora_stats( void )
{
    uint8_t cbuffer_expected[] = { 0x10, 0x00 };
    uint8_t rbuffer_expected[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t response[]         = { 0x23, 0x34, 0x45, 0x56, 0x67, 0x78 };

    sx126x_stats_lora_t stats;

    /*
     * Case 1
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 6, 6,
                                              SX126X_HAL_STATUS_OK );
    sx126x_hal_read_ReturnArrayThruPtr_data( response, 6 );

    status = sx126x_get_lora_stats( radio, &stats );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
    TEST_ASSERT_EQUAL_UINT16( 0x2334, stats.nb_pkt_received );
    TEST_ASSERT_EQUAL_UINT16( 0x4556, stats.nb_pkt_crc_error );
    TEST_ASSERT_EQUAL_UINT16( 0x6778, stats.nb_pkt_header_error );

    /*
     * Case 2
     */
    sx126x_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 6, 6,
                                              SX126X_HAL_STATUS_ERROR );

    status = sx126x_get_lora_stats( radio, &stats );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_reset_stats( void )
{
    uint8_t length;
    uint8_t cbuffer_expected[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 7, 7, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_reset_stats( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 7, 7, NULL, 0, 0, SX126X_HAL_STATUS_ERROR );

    status = sx126x_reset_stats( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
