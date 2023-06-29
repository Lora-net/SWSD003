/**
 * @file      test_lr11xx_radio_timings.c
 *
 * @brief     LR11XX test cases for timings commands
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
#include "lr11xx_radio_timings.h"
#include "lr11xx_radio.h"
#include "mock_lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

TEST_FILE( "lr11xx_regmem.c" )

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

TEST_VALUE( LR11XX_RADIO_LORA_SF5, LR11XX_RADIO_LORA_BW_500, 218 )
TEST_VALUE( LR11XX_RADIO_LORA_SF5, LR11XX_RADIO_LORA_BW_250, 361 )
TEST_VALUE( LR11XX_RADIO_LORA_SF5, LR11XX_RADIO_LORA_BW_125, 643 )
void test_lr11xx_radio_timings_get_delay_between_last_bit_sent_and_rx_done_in_us( lr11xx_radio_lora_sf_t sf,
                                                                                  lr11xx_radio_lora_bw_t bw,
                                                                                  uint32_t delay_in_us_expected )
{
    lr11xx_radio_mod_params_lora_t mod_params;
    lr11xx_radio_pkt_params_lora_t pkt_params;

    mod_params.bw   = bw;
    mod_params.cr   = 0;
    mod_params.ldro = 0;
    mod_params.sf   = sf;

    uint32_t delay = lr11xx_radio_timings_get_delay_between_last_bit_sent_and_rx_done_in_us( &mod_params );

    TEST_ASSERT_EQUAL_UINT32( delay_in_us_expected, delay );
}

TEST_VALUE( LR11XX_RADIO_RAMP_16_US, 16 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_32_US, 32 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_48_US, 48 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_64_US, 64 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_80_US, 80 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_96_US, 96 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_112_US, 112 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_128_US, 128 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_144_US, 144 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_160_US, 160 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_176_US, 176 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_192_US, 192 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_208_US, 208 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_240_US, 240 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_272_US, 272 + 111 )
TEST_VALUE( LR11XX_RADIO_RAMP_304_US, 304 + 111 )
TEST_VALUE( 0x10, 0 + 111 )
void test_lr11xx_radio_timings_get_delay_between_last_bit_sent_and_tx_done_in_us( lr11xx_radio_ramp_time_t ramp_time,
                                                                                  uint32_t delay_in_us_expected )
{
    uint32_t delay = lr11xx_radio_timings_get_delay_between_last_bit_sent_and_tx_done_in_us( ramp_time );

    TEST_ASSERT_EQUAL_UINT32( delay_in_us_expected, delay );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
