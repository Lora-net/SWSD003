#include "lr11xx_lr_fhss.h"
#include "lr11xx_radio.h"
#include "unity.h"

#include <string.h>  // for memset

#include "mock_lr11xx_hal.h"

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

TEST_FILE( "lr11xx_regmem.c" )

void* context = 0;

void setUp( void )
{
}

void tearDown( void )
{
}

void test_lr11xx_lr_fhss_init_nominal( )
{
    uint8_t cbuffer_expected_set_pkt_type[]  = { 0x02, 0x0E, 0x04 };
    uint8_t cbuffer_expected_set_mod_param[] = { 0x02, 0x0F, 0x80, 0x01, 0xE8, 0x48, 0x0B };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_set_pkt_type, 3, 3, NULL, 0, 0,
                                               LR11XX_STATUS_OK );
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_set_mod_param, 7, 7, NULL, 0, 0,
                                               LR11XX_STATUS_OK );

    const lr11xx_status_t status = lr11xx_lr_fhss_init( context );
    TEST_ASSERT_EQUAL_INT( LR11XX_STATUS_OK, status );
}

void test_lr11xx_lr_fhss_init_pkt_fail( )
{
    uint8_t cbuffer_expected_set_pkt_type[] = { 0x02, 0x0E, 0x04 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_set_pkt_type, 3, 3, NULL, 0, 0,
                                               LR11XX_STATUS_ERROR );

    const lr11xx_status_t status = lr11xx_lr_fhss_init( context );
    TEST_ASSERT_EQUAL_INT( LR11XX_STATUS_ERROR, status );
}

void test_lr11xx_lr_fhss_init_mod_fail( )
{
    uint8_t cbuffer_expected_set_pkt_type[]  = { 0x02, 0x0E, 0x04 };
    uint8_t cbuffer_expected_set_mod_param[] = { 0x02, 0x0F, 0x80, 0x01, 0xE8, 0x48, 0x0B };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_set_pkt_type, 3, 3, NULL, 0, 0,
                                               LR11XX_STATUS_OK );
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_set_mod_param, 7, 7, NULL, 0, 0,
                                               LR11XX_STATUS_ERROR );

    const lr11xx_status_t status = lr11xx_lr_fhss_init( context );
    TEST_ASSERT_EQUAL_INT( LR11XX_STATUS_ERROR, status );
}

void test_lr11xx_lr_fhss_get_bit_delay_in_us( void )
{
    lr11xx_lr_fhss_params_t params = {
        .lr_fhss_params.cr           = LR_FHSS_V1_CR_5_6,
        .lr_fhss_params.header_count = 1,
    };

    uint16_t delay = lr11xx_lr_fhss_get_bit_delay_in_us( &params, 24 );
    TEST_ASSERT_EQUAL( delay, 5696 );

    params.lr_fhss_params.header_count = 2;
    delay                              = lr11xx_lr_fhss_get_bit_delay_in_us( &params, 24 );
    TEST_ASSERT_EQUAL( delay, 17984 );

    params.lr_fhss_params.header_count = 3;
    delay                              = lr11xx_lr_fhss_get_bit_delay_in_us( &params, 24 );
    TEST_ASSERT_EQUAL( delay, 13888 );

    params.lr_fhss_params.header_count = 4;
    delay                              = lr11xx_lr_fhss_get_bit_delay_in_us( &params, 24 );
    TEST_ASSERT_EQUAL( delay, 9792 );

    params.lr_fhss_params.cr = LR_FHSS_V1_CR_1_3;
    delay                    = lr11xx_lr_fhss_get_bit_delay_in_us( &params, 2 );
    TEST_ASSERT_EQUAL( delay, 3648 );

    params.lr_fhss_params.cr = LR_FHSS_V1_CR_2_3;
    delay                    = lr11xx_lr_fhss_get_bit_delay_in_us( &params, 3 );
    TEST_ASSERT_EQUAL( delay, 17984 );
}

void test_lr11xx_radio_lr_fhss_build_frame_nominal( void )
{
    const lr_fhss_v1_cr_t              coding_rate          = LR_FHSS_V1_CR_1_3;
    const lr_fhss_v1_modulation_type_t modulation_type      = LR_FHSS_V1_MODULATION_TYPE_GMSK_488;
    const lr_fhss_v1_grid_t            grid                 = LR_FHSS_V1_GRID_25391_HZ;
    const bool                         enable_hopping       = true;
    const lr_fhss_v1_bw_t              bandwidth            = LR_FHSS_V1_BW_1574219_HZ;
    const uint16_t                     hop_sequence_id      = 0x015F;
    const int8_t                       device_offset        = -5;
    const uint8_t                      payload[]            = { 0xFF, 0xAE };
    const uint8_t                      payload_length       = 2;
    const uint8_t                      syncword_expected[4] = { 0x01, 0x02, 0x03, 0x04 };

    const uint8_t cbuffer_syncword_expected[2]     = { 0x02, 0x2D };
    const uint8_t data_buffer_syncword_expected[4] = { 0x01, 0x02, 0x03, 0x04 };

    const uint8_t cbuffer_build_expected[11]    = { 0x02, 0x2C, 0x04, 0x03, 0x00, 0x00, 0x01, 0x09, 0x01, 0x5F, 0xFB };
    const uint8_t data_buffer_build_expected[2] = { 0xFF, 0xAE };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_syncword_expected, 2, 2,
                                               data_buffer_syncword_expected, 4, 4, LR11XX_STATUS_OK );
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_build_expected, 11, 11, data_buffer_build_expected,
                                               2, 2, LR11XX_STATUS_OK );

    const lr11xx_lr_fhss_params_t lr_fhss_param = {
        .lr_fhss_params = {
            .header_count    = 4,
            .cr              = coding_rate,
            .modulation_type = modulation_type,
            .grid            = grid,
            .enable_hopping  = enable_hopping,
            .bw              = bandwidth,
            .sync_word       = syncword_expected,
        },
        .device_offset         = device_offset,
    };
    const lr11xx_hal_status_t status =
        lr11xx_lr_fhss_build_frame( context, &lr_fhss_param, hop_sequence_id, payload, payload_length );

    TEST_ASSERT_EQUAL_INT( LR11XX_STATUS_OK, status );
}

void test_lr11xx_radio_lr_fhss_build_frame_fail( void )
{
    const lr_fhss_v1_cr_t              coding_rate          = LR_FHSS_V1_CR_1_3;
    const lr_fhss_v1_modulation_type_t modulation_type      = LR_FHSS_V1_MODULATION_TYPE_GMSK_488;
    const lr_fhss_v1_grid_t            grid                 = LR_FHSS_V1_GRID_25391_HZ;
    const bool                         enable_hopping       = true;
    const lr_fhss_v1_bw_t              bandwidth            = LR_FHSS_V1_BW_1574219_HZ;
    const uint16_t                     hop_sequence_id      = 0x015F;
    const int8_t                       device_offset        = -5;
    const uint8_t                      payload[]            = { 0xFF, 0xAE };
    const uint8_t                      payload_length       = 2;
    const uint8_t                      syncword_expected[4] = { 0x01, 0x02, 0x03, 0x04 };

    const uint8_t cbuffer_syncword_expected[2]     = { 0x02, 0x2D };
    const uint8_t data_buffer_syncword_expected[4] = { 0x01, 0x02, 0x03, 0x04 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_syncword_expected, 2, 2,
                                               data_buffer_syncword_expected, 4, 4, LR11XX_STATUS_ERROR );

    const lr11xx_lr_fhss_params_t lr_fhss_param = {
        .lr_fhss_params = {
            .cr              = coding_rate,
            .modulation_type = modulation_type,
            .grid            = grid,
            .enable_hopping  = enable_hopping,
            .bw              = bandwidth,
            .sync_word       = syncword_expected,
        },
        .device_offset         = device_offset,
    };
    const lr11xx_hal_status_t status =
        lr11xx_lr_fhss_build_frame( context, &lr_fhss_param, hop_sequence_id, payload, payload_length );

    TEST_ASSERT_EQUAL_INT( LR11XX_STATUS_ERROR, status );
}

void test_lr11xx_radio_lr_fhss_build_frame_syncword_fail( void )
{
    const lr_fhss_v1_cr_t              coding_rate     = LR_FHSS_V1_CR_1_3;
    const lr_fhss_v1_modulation_type_t modulation_type = LR_FHSS_V1_MODULATION_TYPE_GMSK_488;
    const lr_fhss_v1_grid_t            grid            = LR_FHSS_V1_GRID_25391_HZ;
    const bool                         enable_hopping  = true;
    const lr_fhss_v1_bw_t              bandwidth       = LR_FHSS_V1_BW_1574219_HZ;
    const uint16_t                     hop_sequence_id = 0x015F;
    const int8_t                       device_offset   = -5;
    const uint8_t                      payload[]       = { 0xFF, 0xAE };
    const uint8_t                      payload_length  = 2;

    const uint8_t syncword_expected[4] = { 0x01, 0x02, 0x03, 0x04 };

    const uint8_t cbuffer_syncword_expected[2]     = { 0x02, 0x2D };
    const uint8_t data_buffer_syncword_expected[4] = { 0x01, 0x02, 0x03, 0x04 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_syncword_expected, 2, 2,
                                               data_buffer_syncword_expected, 4, 4, LR11XX_STATUS_ERROR );

    const lr11xx_lr_fhss_params_t lr_fhss_param = {
        .lr_fhss_params = {
            .cr              = coding_rate,
            .modulation_type = modulation_type,
            .grid            = grid,
            .enable_hopping  = enable_hopping,
            .bw              = bandwidth,
            .sync_word       = syncword_expected,
        },
        .device_offset         = device_offset,
    };
    const lr11xx_hal_status_t status =
        lr11xx_lr_fhss_build_frame( context, &lr_fhss_param, hop_sequence_id, payload, payload_length );

    TEST_ASSERT_EQUAL_INT( LR11XX_STATUS_ERROR, status );
}
