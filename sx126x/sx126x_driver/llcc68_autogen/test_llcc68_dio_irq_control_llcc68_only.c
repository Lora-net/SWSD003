/**
 * @file      test_llcc68_dio_irq_control_llcc68_only.c
 *
 * @brief     Llcc68 test cases for DIO and interrupt related commands
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
#include "llcc68.h"
#include "mock_llcc68_hal.h"

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
llcc68_status_t status;

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

void test_llcc68_set_dio_irq_params( void )
{
    uint8_t  cbuffer_expected_1[] = { 0x08, 0x03, 0xFF, 0x02, 0x49, 0x00, 0x92, 0x01, 0x24 };
    uint8_t  cbuffer_expected_2[] = { 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint16_t irq_mask;
    uint16_t dio1_mask;
    uint16_t dio2_mask;
    uint16_t dio3_mask;

    /*
     * Case 1
     */
    llcc68_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 9, 9, NULL, 0, 0, LLCC68_HAL_STATUS_OK );

    irq_mask  = LLCC68_IRQ_ALL;
    dio1_mask = LLCC68_IRQ_TX_DONE | LLCC68_IRQ_SYNC_WORD_VALID | LLCC68_IRQ_CRC_ERROR | LLCC68_IRQ_TIMEOUT;
    dio2_mask = LLCC68_IRQ_RX_DONE | LLCC68_IRQ_HEADER_VALID | LLCC68_IRQ_CAD_DONE;
    dio3_mask = LLCC68_IRQ_PREAMBLE_DETECTED | LLCC68_IRQ_HEADER_ERROR | LLCC68_IRQ_CAD_DETECTED;

    status = llcc68_set_dio_irq_params( radio, irq_mask, dio1_mask, dio2_mask, dio3_mask );

    TEST_ASSERT_EQUAL_UINT8( LLCC68_STATUS_OK, status );

    /*
     * Case 2
     */
    llcc68_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 9, 9, NULL, 0, 0,
                                               LLCC68_HAL_STATUS_ERROR );

    irq_mask  = LLCC68_IRQ_NONE;
    dio1_mask = LLCC68_IRQ_NONE;
    dio2_mask = LLCC68_IRQ_NONE;
    dio3_mask = LLCC68_IRQ_NONE;
    status    = llcc68_set_dio_irq_params( radio, irq_mask, dio1_mask, dio2_mask, dio3_mask );

    TEST_ASSERT_EQUAL_UINT8( LLCC68_STATUS_ERROR, status );
}

void test_llcc68_clear_irq_status( void )
{
    uint8_t  cbuffer_expected_1[] = { 0x02, 0x03, 0xFF };
    uint8_t  cbuffer_expected_2[] = { 0x02, 0x00, 0x00 };
    uint16_t irq_mask;

    /*
     * Case 1
     */
    llcc68_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, NULL, 0, 0, LLCC68_HAL_STATUS_OK );

    irq_mask = LLCC68_IRQ_ALL;
    status   = llcc68_clear_irq_status( radio, irq_mask );

    TEST_ASSERT_EQUAL_UINT8( LLCC68_STATUS_OK, status );

    /*
     * Case 2
     */
    llcc68_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, NULL, 0, 0,
                                               LLCC68_HAL_STATUS_ERROR );

    irq_mask = LLCC68_IRQ_NONE;
    status   = llcc68_clear_irq_status( radio, irq_mask );

    TEST_ASSERT_EQUAL_UINT8( LLCC68_STATUS_ERROR, status );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
