/**
 * @file      test_from_driver_generated_data.c
 *
 * @brief     test case for LR-FHSS MAC layer
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "lr_fhss_mac.h"

#include "lr_fhss_test_syncword.h"
#include "lr_fhss_test_syncwords.h"
#include "lr_fhss_test_params.h"
#include "lr_fhss_test_app_payload.h"
#include "lr_fhss_test_frame.h"
#include "lr_fhss_test_sizes.h"
#include "lr_fhss_test_digests.h"
#include "lr_fhss_test_hop.h"
#include "lr_fhss_test_hops.h"
#include "lr_fhss_test_hop_sequence_ids.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void setUp( void ) {}

void tearDown( void ) {}

#define COUNTOF( arr ) ( sizeof( arr ) / sizeof( arr[0] ) )

#define OVERWRITE_PROTECTION_COUNT ( 64 )
#define OPC                                                                                                           \
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

void test_driver_generated_data( void )
{
    uint8_t data_buffer[256 + OVERWRITE_PROTECTION_COUNT];

    for( int i = 0; i < COUNTOF( test_params ); ++i )
    {
        const lr_fhss_v1_params_t* params = &test_params[i];

        const uint8_t* test_app_payload        = test_app_payloads[i];
        int            test_app_payload_length = test_sizes[i][0];

        const uint8_t* test_frame        = test_frames[i];
        int            test_frame_length = test_sizes[i][1];

        const int16_t*          test_hop    = test_hops[i];
        const lr_fhss_digest_t* test_digest = &test_digests[i];

        const uint16_t test_hop_sequence_id = test_hop_sequence_ids[i];

        memset( data_buffer, 0, sizeof( data_buffer ) );
        uint16_t frame_length =
            lr_fhss_build_frame( params, test_hop_sequence_id, test_app_payload, test_app_payload_length, data_buffer );

        TEST_ASSERT_EQUAL( test_frame_length, frame_length );
        TEST_ASSERT_EQUAL_MEMORY( test_frame, data_buffer, frame_length );
        for( int i = 0; i < OVERWRITE_PROTECTION_COUNT; ++i )
        {
            TEST_ASSERT_EQUAL( 0, data_buffer[frame_length + i] );
        }

        lr_fhss_digest_t digest;
        lr_fhss_process_parameters( params, test_app_payload_length, &digest );

        TEST_ASSERT_EQUAL( frame_length, digest.nb_bytes );

        TEST_ASSERT_EQUAL( test_digest->nb_hops, digest.nb_hops );
        TEST_ASSERT_EQUAL( test_digest->nb_bytes, digest.nb_bytes );

        if( params->enable_hopping )
        {
            uint16_t             lfsr_state;
            lr_fhss_hop_params_t hop_params;
            lr_fhss_status_t status = lr_fhss_get_hop_params( params, &hop_params, &lfsr_state, test_hop_sequence_id );
            TEST_ASSERT_EQUAL( LR_FHSS_STATUS_OK, status );

            // Skip the hop frequencies inside the set [0, 4 - header_count):
            for( unsigned int i = 0; i < 4 - params->header_count; ++i )
            {
                lr_fhss_get_next_state( &lfsr_state, &hop_params );
            }

            for( unsigned int hop = 0; hop < digest.nb_hops; ++hop )
            {
                const int16_t hop_freq = lr_fhss_get_next_freq_in_grid( &lfsr_state, &hop_params, params );
                TEST_ASSERT_EQUAL( test_hop[hop], hop_freq );
            }
        }
    }
}
