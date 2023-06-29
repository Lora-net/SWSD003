/**
 * @file      test_sx126x_operational_modes.c
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

#include "sx126x.h"
#include "unity.h"
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

void test_sx126x_set_sleep( void )
{
    uint8_t ccbuffer_expected_1[] = { 0x84, 0x04 };
    uint8_t ccbuffer_expected_2[] = { 0x84, 0x00 };

    sx126x_sleep_cfgs_t sleep_config;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, ccbuffer_expected_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    sleep_config = SX126X_SLEEP_CFG_WARM_START;
    status       = sx126x_set_sleep( radio, sleep_config );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, ccbuffer_expected_2, 2, 2, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    sleep_config = SX126X_SLEEP_CFG_COLD_START;
    status       = sx126x_set_sleep( radio, sleep_config );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_standby( void )
{
    uint8_t cbuffer_expected_1[] = { 0x80, 0x01 };
    uint8_t cbuffer_expected_2[] = { 0x80, 0x00 };

    sx126x_standby_cfg_t standby_config;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    standby_config = SX126X_STANDBY_CFG_XOSC;
    status         = sx126x_set_standby( radio, standby_config );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 2, 2, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    standby_config = SX126X_STANDBY_CFG_RC;
    status         = sx126x_set_standby( radio, standby_config );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_fs( void )
{
    uint8_t cbuffer_expected[] = { 0xC1 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 1, 1, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_fs( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 1, 1, NULL, 0, 0, SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_fs( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_tx( void )
{
    uint8_t cbuffer_expected_1[] = { 0x83, 0x00, 0x00, 0x40 };

    uint32_t timeout_1 = 1;
    uint32_t timeout_2 = 0x89ABCDEF;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_tx( radio, timeout_1 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    status = sx126x_set_tx( radio, timeout_2 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_UNKNOWN_VALUE, status );
}

void test_sx126x_set_tx_with_timeout_in_rtc_step( void )
{
    uint8_t cbuffer_expected_1[] = { 0x83, 0x12, 0x23, 0x34 };
    uint8_t cbuffer_expected_2[] = { 0x83, 0xAB, 0xCD, 0xEF };

    uint32_t timeout_1 = 0x01122334;
    uint32_t timeout_2 = 0x89ABCDEF;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_tx_with_timeout_in_rtc_step( radio, timeout_1 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 4, 4, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_tx_with_timeout_in_rtc_step( radio, timeout_2 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_rx( void )
{
    uint8_t cbuffer_expected_1[] = { 0x82, 0x00, 0xFA, 0x00 };

    uint32_t timeout_1 = 1000;
    uint32_t timeout_2 = 0x01122334;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_rx( radio, timeout_1 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    status = sx126x_set_rx( radio, timeout_2 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_UNKNOWN_VALUE, status );
}

void test_sx126x_set_rx_with_timeout_in_rtc_step( void )
{
    uint8_t cbuffer_expected_1[] = { 0x82, 0xBC, 0xCD, 0xDE };
    uint8_t cbuffer_expected_2[] = { 0x82, 0x12, 0x23, 0x34 };

    uint32_t timeout_1 = 0xABBCCDDE;
    uint32_t timeout_2 = 0x01122334;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 4, 4, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_rx_with_timeout_in_rtc_step( radio, timeout_1 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 4, 4, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_rx_with_timeout_in_rtc_step( radio, timeout_2 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_stop_tmr_on_pbl( void )
{
    uint8_t cbuffer_expected_1[] = { 0x9F, 0x01 };
    uint8_t cbuffer_expected_2[] = { 0x9F, 0x00 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_stop_timer_on_preamble( radio, true );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 2, 2, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_stop_timer_on_preamble( radio, false );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_rx_duty_cycle( void )
{
    uint8_t cbuffer_expected_1[] = { 0x94, 0x00, 0x00, 0x40, 0x00, 0xFA, 0x00 };
    uint8_t cbuffer_expected_2[] = { 0x94, 0x00, 0xFA, 0x00, 0x00, 0x00, 0x40 };

    uint32_t time_1 = 1;
    uint32_t time_2 = 1000;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 7, 7, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_rx_duty_cycle( radio, time_1, time_2 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 7, 7, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_rx_duty_cycle( radio, time_2, time_1 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( void )
{
    uint8_t cbuffer_expected_1[] = { 0x94, 0xBC, 0xCD, 0xDE, 0x12, 0x23, 0x34 };
    uint8_t cbuffer_expected_2[] = { 0x94, 0x12, 0x23, 0x34, 0xBC, 0xCD, 0xDE };

    uint32_t time_1 = 0xABBCCDDE;
    uint32_t time_2 = 0x01122334;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 7, 7, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( radio, time_1, time_2 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 7, 7, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( radio, time_2, time_1 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_cad( void )
{
    uint8_t cbuffer_expected[] = { 0xC5 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 1, 1, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_cad( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 1, 1, NULL, 0, 0, SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_cad( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_tx_cw( void )
{
    uint8_t buffer_expected[] = { 0xD1 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, buffer_expected, 1, 1, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_tx_cw( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, buffer_expected, 1, 1, NULL, 0, 0, SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_tx_cw( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_tx_cpbl( void )
{
    uint8_t cbuffer_expected[] = { 0xD2 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 1, 1, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_tx_infinite_preamble( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 1, 1, NULL, 0, 0, SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_tx_infinite_preamble( radio );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_reg_mode( void )
{
    uint8_t cbuffer_expected_1[] = { 0x96, 0x00 };
    uint8_t cbuffer_expected_2[] = { 0x96, 0x01 };

    sx126x_reg_mod_t reg_mod;

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    reg_mod = SX126X_REG_MODE_LDO;
    status  = sx126x_set_reg_mode( radio, reg_mod );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 2, 2, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    reg_mod = SX126X_REG_MODE_DCDC;
    status  = sx126x_set_reg_mode( radio, reg_mod );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_cal( void )
{
    uint8_t cbuffer_expected_1[] = { 0x89, 0x01 };
    uint8_t cbuffer_expected_2[] = { 0x89, 0x02 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_cal( radio, SX126X_CAL_RC64K );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 2, 2, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_cal( radio, SX126X_CAL_RC13M );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_cal_img_in_mhz( void )
{
    uint8_t cbuffer_expected_1[] = { 0x98, 0xE1, 0xE8 };
    uint8_t cbuffer_expected_2[] = { 0x98, 0xD7, 0xDA };
    uint8_t cbuffer_expected_3[] = { 0x98, 0xC2, 0xC5 };
    uint8_t cbuffer_expected_4[] = { 0x98, 0x75, 0x80 };
    uint8_t cbuffer_expected_5[] = { 0x98, 0x6B, 0x6E };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_cal_img_in_mhz( radio, 902, 928 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_cal_img_in_mhz( radio, 863, 870 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 3
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_3, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_cal_img_in_mhz( radio, 779, 787 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 4
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_4, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_cal_img_in_mhz( radio, 470, 510 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 5
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_5, 3, 3, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_cal_img_in_mhz( radio, 430, 440 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
}

void test_sx126x_set_pa_cfg( void )
{
    uint8_t cbuffer_expected_1[] = { 0x95, 0x01, 0x12, 0x23, 0x34 };
    uint8_t cbuffer_expected_2[] = { 0x95, 0xAB, 0xBC, 0xCD, 0xDE };

    sx126x_pa_cfg_params_t pa_cfg_1 = { .pa_duty_cycle = 0x01, .hp_max = 0x12, .device_sel = 0x23, .pa_lut = 0x34 };

    sx126x_pa_cfg_params_t pa_cfg_2 = { .pa_duty_cycle = 0xAB, .hp_max = 0xBC, .device_sel = 0xCD, .pa_lut = 0xDE };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 5, 5, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_pa_cfg( radio, &pa_cfg_1 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 5, 5, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_pa_cfg( radio, &pa_cfg_2 );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );
}

void test_sx126x_set_rx_tx_fallback_mode( void )
{
    uint8_t cbuffer_expected_1[] = { 0x93, 0x20 };
    uint8_t cbuffer_expected_2[] = { 0x93, 0x30 };
    uint8_t cbuffer_expected_3[] = { 0x93, 0x40 };

    /*
     * Case 1
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_1, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_rx_tx_fallback_mode( radio, SX126X_FALLBACK_STDBY_RC );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );

    /*
     * Case 2
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_2, 2, 2, NULL, 0, 0,
                                               SX126X_HAL_STATUS_ERROR );

    status = sx126x_set_rx_tx_fallback_mode( radio, SX126X_FALLBACK_STDBY_XOSC );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_ERROR, status );

    /*
     * Case 3
     */
    sx126x_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_3, 2, 2, NULL, 0, 0, SX126X_HAL_STATUS_OK );

    status = sx126x_set_rx_tx_fallback_mode( radio, SX126X_FALLBACK_FS );

    TEST_ASSERT_EQUAL_UINT8( SX126X_STATUS_OK, status );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
