/**
 * @file      test_lr_fhss_mac.c
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

#include <stdint.h>
#include <string.h>

#include "lr_fhss_mac.h"

#include "declarations.h"

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

void setUp( void )
{
}

void tearDown( void )
{
}

#define COUNTOF( arr ) ( sizeof( arr ) / sizeof( arr[0] ) )

// CRC-8 test as defined in LR-FHSS specification
void test_crc8( void )
{
    const char* test_string = "123456789";

    uint8_t crc8 = lr_fhss_header_crc8( ( const uint8_t* ) test_string, strlen( test_string ) );
    TEST_ASSERT_EQUAL( 0x20, crc8 );
}

// CRC-8 test as defined in LR-FHSS specification
void test_crc16( void )
{
    const char* test_string = "123456789";

    uint16_t crc16 = lr_fhss_payload_crc16( ( const uint8_t* ) test_string, strlen( test_string ) );
    TEST_ASSERT_EQUAL( 0x375d, crc16 );
}

#define OVERWRITE_PROTECTION_COUNT ( 64 )
#define OPC                                                                                                           \
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

// payload test as defined in LR-FHSS specification
void test_generated_payload_1( void )
{
    uint16_t hop_sequence_id = 16;
    uint8_t  physical_payload[256 + OVERWRITE_PROTECTION_COUNT];
    uint8_t  physical_payload_check[] = {
        0x24, 0x0d, 0x43, 0xfc, 0x93, 0xcb, 0x03, 0xde, 0x65, 0x45, 0xeb, 0x62, 0x30, 0x18, 0x49, 0x0f, 0x50, 0xfe,
        0x24, 0xe2, 0xc0, 0xf7, 0x99, 0x50, 0x6e, 0x5c, 0x89, 0x26, 0x12, 0x60, 0xc5, 0x37, 0x01, 0x38, 0xb0, 0x3d,
        0xe6, 0x54, 0x39, 0x87, 0x2a, 0x45, 0xc0, 0x9a, 0x89, 0xbf, 0x1e, 0x1f, 0x9e, 0x0c, 0xc7, 0xe3, 0x39, 0x92,
        0x34, 0x4b, 0xd0, 0x32, 0xe5, 0xec, 0xdc, 0xb3, 0x0e, 0xf1, 0xfa, 0xdc, 0x4c, 0x34, 0x50, 0x2f, 0x1b, 0x32,
        0x34, 0xe5, 0x1f, 0xea, 0x6b, 0x8a, 0x63, 0x92, 0x4b, 0x5d, 0xf2, 0x4a, 0x93, 0x15, 0x30, 0xd9, 0x64, 0xa9,
        0x20, 0x0c, 0x00, 0xe3, 0x45, 0x08, 0x7e, 0xa2, 0x7c, 0x32, 0xc8, 0x6c, 0x2a, 0xa8, 0xc0, 0xcb, 0x07, 0xbb,
        0x0d, 0xab, 0xa4, 0xf1, 0xc5, 0x69, 0x39, 0x05, 0x2f, 0xa0, 0xee, 0x4e, 0xa9, 0x97, 0x74, 0x42, 0x1c, 0x49,
        0xe6, 0xb1, 0xa6, 0xa7, 0x84, 0x89, 0xb3, 0xea, 0xe4, 0x19, 0x20, 0xc7, 0x6d, 0x82, 0x2b, 0x20, OPC
    };

    const int16_t hop_sequence_check[] = {
        29, 22, -11, 0, -4, -24, -18, -17, 13, -30, -7, 2, -5, 3, 25, 20, -10, -29, 7
    };

    lr_fhss_digest_t digest;

    const uint8_t payload[] = { 64,  0,   0,  1,   38, 0,  1,   0,   3,   95,  187, 99, 173, 97, 199,
                                237, 155, 73, 212, 34, 28, 247, 149, 131, 173, 233, 71, 16,  247 };

    const uint8_t sync_word[4] = { 0x2C, 0x0F, 0x79, 0x95 };

    const struct lr_fhss_v1_params_s params = {
        .sync_word       = sync_word,
        .modulation_type = LR_FHSS_V1_MODULATION_TYPE_GMSK_488,
        .cr              = LR_FHSS_V1_CR_1_3,
        .grid            = LR_FHSS_V1_GRID_25391_HZ,
        .bw              = LR_FHSS_V1_BW_1523438_HZ,
        .enable_hopping  = 1,
        .header_count    = 3,
    };

    lr_fhss_process_parameters( &params, COUNTOF( payload ), &digest );

    TEST_ASSERT_EQUAL( COUNTOF( physical_payload_check ) - OVERWRITE_PROTECTION_COUNT, digest.nb_bytes );

    uint16_t bytes_written =
        lr_fhss_build_frame( &params, hop_sequence_id, payload, COUNTOF( payload ), physical_payload );

    TEST_ASSERT_EQUAL( COUNTOF( physical_payload_check ) - OVERWRITE_PROTECTION_COUNT, bytes_written );
    TEST_ASSERT_EQUAL_MEMORY( physical_payload_check, physical_payload, COUNTOF( physical_payload_check ) );
    TEST_ASSERT_EQUAL( COUNTOF( hop_sequence_check ), digest.nb_hops );

    uint16_t             lfsr_state;
    lr_fhss_hop_params_t hop_params;
    lr_fhss_status_t     status = lr_fhss_get_hop_params( &params, &hop_params, &lfsr_state, hop_sequence_id );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_OK, status );

    // Skip the hop frequencies inside the set [0, 4 - header_count):
    for( unsigned int i = 0; i < 4 - params.header_count; ++i )
    {
        lr_fhss_get_next_state( &lfsr_state, &hop_params );
    }

    for( unsigned int hop = 0; hop < digest.nb_hops; ++hop )
    {
        const int16_t hop_freq = lr_fhss_get_next_freq_in_grid( &lfsr_state, &hop_params, &params );
        TEST_ASSERT_EQUAL( hop_sequence_check[hop], hop_freq );
    }
}

// payload test as defined in LR-FHSS specification
void test_generated_payload_2( void )
{
    uint16_t hop_sequence_id = 16;
    uint8_t  physical_payload[256 + OVERWRITE_PROTECTION_COUNT];
    uint8_t  physical_payload_check[] = { 0x07, 0xbd, 0x16, 0x5b, 0xc3, 0xcb, 0x03, 0xde, 0x65, 0x51, 0x2d, 0x32,
                                         0x83, 0xc8, 0x01, 0x63, 0x01, 0xb4, 0xd0, 0xf2, 0xc0, 0xf7, 0x99, 0x54,
                                         0xc3, 0x0c, 0x80, 0xc3, 0x13, 0xce, 0x03, 0x07, 0x33, 0x6e, 0x9c, 0x17,
                                         0x4e, 0x0d, 0x4a, 0x92, 0x2e, 0x0a, 0x5b, 0xab, 0xbe, 0xd2, 0xe3, 0x44,
                                         0x9c, 0x6b, 0x83, 0x88, 0x86, 0xa1, 0x00, 0x38, 0x52, 0x79, 0x19, 0x6c,
                                         0x02, 0x98, 0x73, 0xa2, 0x1d, 0x1a, 0x04, 0x8c, 0xbb, 0x24, 0x1e, 0x2d,
                                         0xc5, 0xfe, 0xc8, 0x81, 0xb6, 0x2d, 0x80, OPC };

    const int16_t hop_sequence_check[] = { -85, -22, 73, 60, -50, 91, 53, 2, -35, -86 };

    lr_fhss_digest_t digest;

    const uint8_t payload[] = { 64,  0, 0,   1,  38,  0,  15,  0,   3,  130, 153, 130, 127, 102, 133,
                                205, 9, 120, 66, 118, 67, 161, 202, 35, 185, 23,  123, 108, 154 };

    const uint8_t sync_word[4] = { 0x2C, 0x0F, 0x79, 0x95 };

    const struct lr_fhss_v1_params_s params = {
        .sync_word       = sync_word,
        .modulation_type = LR_FHSS_V1_MODULATION_TYPE_GMSK_488,
        .cr              = LR_FHSS_V1_CR_2_3,
        .grid            = LR_FHSS_V1_GRID_3906_HZ,
        .bw              = LR_FHSS_V1_BW_722656_HZ,
        .enable_hopping  = 1,
        .header_count    = 2,
    };

    lr_fhss_process_parameters( &params, COUNTOF( payload ), &digest );

    TEST_ASSERT_EQUAL( COUNTOF( physical_payload_check ) - OVERWRITE_PROTECTION_COUNT, digest.nb_bytes );

    uint16_t bytes_written =
        lr_fhss_build_frame( &params, hop_sequence_id, payload, COUNTOF( payload ), physical_payload );

    TEST_ASSERT_EQUAL( COUNTOF( physical_payload_check ) - OVERWRITE_PROTECTION_COUNT, bytes_written );
    TEST_ASSERT_EQUAL_MEMORY( physical_payload_check, physical_payload, COUNTOF( physical_payload_check ) );
    TEST_ASSERT_EQUAL( COUNTOF( hop_sequence_check ), digest.nb_hops );

    uint16_t             lfsr_state;
    lr_fhss_hop_params_t hop_params;
    lr_fhss_status_t     status = lr_fhss_get_hop_params( &params, &hop_params, &lfsr_state, hop_sequence_id );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_OK, status );

    // Skip the hop frequencies inside the set [0, 4 - header_count):
    for( unsigned int i = 0; i < 4 - params.header_count; ++i )
    {
        lr_fhss_get_next_state( &lfsr_state, &hop_params );
    }

    for( unsigned int hop = 0; hop < digest.nb_hops; ++hop )
    {
        const int16_t hop_freq = lr_fhss_get_next_freq_in_grid( &lfsr_state, &hop_params, &params );
        TEST_ASSERT_EQUAL( hop_sequence_check[hop], hop_freq );
    }
}

// Test number of sequences as defined in LR-FHSS specification v18 Table 18
void test_num_sequences( void )
{
    lr_fhss_status_t status;
    unsigned int     hop_sequence_count;

    lr_fhss_hop_params_t hop_params;
    uint16_t             initial_state;

    const uint8_t sync_word[4] = { 0x2C, 0x0F, 0x79, 0x95 };

    struct lr_fhss_v1_params_s params = {
        .sync_word       = sync_word,
        .modulation_type = LR_FHSS_V1_MODULATION_TYPE_GMSK_488,
        .cr              = LR_FHSS_V1_CR_1_3,
        .grid            = LR_FHSS_V1_GRID_25391_HZ,
        .bw              = LR_FHSS_V1_BW_1523438_HZ,
        .enable_hopping  = 1,
        .header_count    = 3,
    };

    hop_sequence_count = lr_fhss_get_hop_sequence_count( &params );
    TEST_ASSERT_EQUAL( 384, hop_sequence_count );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 383 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_OK, status );
    TEST_ASSERT_EQUAL( 56, initial_state );
    TEST_ASSERT_EQUAL( 60, hop_params.n_grid );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 384 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_ERROR, status );

    params.bw = LR_FHSS_V1_BW_722656_HZ;

    hop_sequence_count = lr_fhss_get_hop_sequence_count( &params );
    TEST_ASSERT_EQUAL( 384, hop_sequence_count );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 383 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_OK, status );
    TEST_ASSERT_EQUAL( 6, initial_state );
    TEST_ASSERT_EQUAL( 28, hop_params.n_grid );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 384 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_ERROR, status );

    params.bw   = LR_FHSS_V1_BW_39063_HZ;
    params.grid = LR_FHSS_V1_GRID_3906_HZ;

    hop_sequence_count = lr_fhss_get_hop_sequence_count( &params );
    TEST_ASSERT_EQUAL( 384, hop_sequence_count );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 383 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_OK, status );
    TEST_ASSERT_EQUAL( 6, initial_state );
    TEST_ASSERT_EQUAL( 10, hop_params.n_grid );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 384 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_ERROR, status );

    params.bw = LR_FHSS_V1_BW_136719_HZ;

    hop_sequence_count = lr_fhss_get_hop_sequence_count( &params );
    TEST_ASSERT_EQUAL( 384, hop_sequence_count );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 383 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_OK, status );
    TEST_ASSERT_EQUAL( 6, initial_state );
    TEST_ASSERT_EQUAL( 35, hop_params.n_grid );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 384 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_ERROR, status );

    params.bw = LR_FHSS_V1_BW_335938_HZ;

    hop_sequence_count = lr_fhss_get_hop_sequence_count( &params );
    TEST_ASSERT_EQUAL( 512, hop_sequence_count );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 511 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_OK, status );
    TEST_ASSERT_EQUAL( 6, initial_state );
    TEST_ASSERT_EQUAL( 86, hop_params.n_grid );

    params.bw = LR_FHSS_V1_BW_722656_HZ;

    hop_sequence_count = lr_fhss_get_hop_sequence_count( &params );
    TEST_ASSERT_EQUAL( 512, hop_sequence_count );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 511 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_OK, status );
    TEST_ASSERT_EQUAL( 6, initial_state );
    TEST_ASSERT_EQUAL( 185, hop_params.n_grid );

    params.bw = LR_FHSS_V1_BW_1523438_HZ;

    hop_sequence_count = lr_fhss_get_hop_sequence_count( &params );
    TEST_ASSERT_EQUAL( 512, hop_sequence_count );

    status = lr_fhss_get_hop_params( &params, &hop_params, &initial_state, 511 );
    TEST_ASSERT_EQUAL( LR_FHSS_STATUS_OK, status );
    TEST_ASSERT_EQUAL( 6, initial_state );
    TEST_ASSERT_EQUAL( 390, hop_params.n_grid );
    TEST_ASSERT_EQUAL( 511, hop_params.xoring_seed );
}
