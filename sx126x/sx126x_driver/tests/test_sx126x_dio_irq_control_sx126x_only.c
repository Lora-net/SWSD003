/**
 * @file      test_sx126x_dio_irq_control_sx126x_only.c
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

void test_sx126x_set_dio_irq_params( void )
{
    uint8_t  cbuffer_expected_1[] = { 0x08, 0x43, 0xFF, 0x02, 0x49, 0x00, 0x92, 0x01, 0x24 };
    uint8_t  cbuffer_expected_2[] = { 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint16_t irq_mask;
    uint16_t dio1_mask;
    uint16_t dio2_mask;
    uint16_t dio3_mask;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 9, 9, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    irq_mask  = SX126X_IRQ_ALL;
    dio1_mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_SYNC_WORD_VALID | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT;
    dio2_mask = SX126X_IRQ_RX_DONE | SX126X_IRQ_HEADER_VALID | SX126X_IRQ_CAD_DONE;
    dio3_mask = SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_CAD_DETECTED;

    status = sx126x_set_dio_irq_params( radio, irq_mask, dio1_mask, dio2_mask, dio3_mask );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 9, 9, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    irq_mask  = SX126X_IRQ_NONE;
    dio1_mask = SX126X_IRQ_NONE;
    dio2_mask = SX126X_IRQ_NONE;
    dio3_mask = SX126X_IRQ_NONE;
    status    = sx126x_set_dio_irq_params( radio, irq_mask, dio1_mask, dio2_mask, dio3_mask );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_clear_irq_status( void )
{
    uint8_t  cbuffer_expected_1[] = { 0x02, 0x43, 0xFF };
    uint8_t  cbuffer_expected_2[] = { 0x02, 0x00, 0x00 };
    uint16_t irq_mask;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    irq_mask = SX126X_IRQ_ALL;
    status   = sx126x_clear_irq_status( radio, irq_mask );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    irq_mask = SX126X_IRQ_NONE;
    status   = sx126x_clear_irq_status( radio, irq_mask );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
