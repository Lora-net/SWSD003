#include "lr11xx_lr_fhss_alpha.h"
#include "unity.h"

#include "mock_lr11xx_hal.h"

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

void* context = 0;

void setUp( void )
{
}

void tearDown( void )
{
}

void test_lr11xx_radio_cfg_lr_fhss( void )
{
    const lr11xx_lr_fhss_mode_t        lr_fhss_mode         = LR11XX_LR_FHSS_MODE_INTRA_PKT;
    const uint8_t                      packet_length        = 132;
    const uint8_t                      number_of_hops       = 155;
    const lr11xx_lr_fhss_frequency_cfg symbols_per_freq[2]  = { { .frequency = 868000000, .nb_symbols = 15 },
                                                               { .frequency = 868500000, .nb_symbols = 22 } };
    const uint8_t                      nb_symbols_per_freq  = 2;
    const uint8_t                      cbuffer_expected[17] = { 0x02, 0x21, 0x01, 0x84, 0x9B, 0x00, 0x0F, 0x33, 0xBC,
                                           0xA1, 0x00, 0x00, 0x16, 0x33, 0xc4, 0x42, 0x20 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 17, 17, NULL, 0, 0, LR11XX_STATUS_OK );
    const lr11xx_hal_status_t status = lr11xx_lr_fhss_cfg( context, lr_fhss_mode, packet_length, number_of_hops,
                                                           symbols_per_freq, nb_symbols_per_freq );

    TEST_ASSERT_EQUAL_INT( LR11XX_STATUS_OK, status );
}

void test_lr11xx_radio_lr_fhss_update_block_cfg( void )
{
    const uint8_t                      block_index          = 2;
    const lr11xx_lr_fhss_frequency_cfg symbols_per_freq     = { .frequency = 868000000, .nb_symbols = 15 };
    const uint8_t                      cbuffer_expected[17] = { 0x02, 0x2A, 0x02, 0x00, 0x0F, 0x33, 0xBC, 0xA1, 0x00 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 9, 9, NULL, 0, 0, LR11XX_STATUS_OK );
    const lr11xx_hal_status_t status = lr11xx_lr_fhss_update_block_cfg( context, block_index, &symbols_per_freq );

    TEST_ASSERT_EQUAL_INT( LR11XX_STATUS_OK, status );
}

#define EXPECTED_LENGTH_TO_READ ( 3 + 6 * 40 )
void test_lr11xx_radio_lr_fhss_read_table( void )
{
    uint8_t cbuffer_expected[]                           = { 0x02, 0x2F };
    uint8_t rbuffer_in_expected[EXPECTED_LENGTH_TO_READ] = { 0x00 };
    uint8_t rbuffer_out_faked[EXPECTED_LENGTH_TO_READ]   = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                                           0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x00 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected,
                                              EXPECTED_LENGTH_TO_READ, EXPECTED_LENGTH_TO_READ, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, EXPECTED_LENGTH_TO_READ );

    lr11xx_lr_fhss_mode_t        lr_fhss_mode                                    = 0;
    uint8_t                      packet_length                                   = 0;
    uint8_t                      number_of_hops                                  = 0;
    lr11xx_lr_fhss_frequency_cfg symbols_per_freq[LR11XX_LR_FHSS_MAX_READ_TABLE] = { 0x00 };

    lr11xx_lr_fhss_read_table( context, &lr_fhss_mode, &packet_length, &number_of_hops, symbols_per_freq );

    TEST_ASSERT_EQUAL_UINT8( lr_fhss_mode, 0x01 );
    TEST_ASSERT_EQUAL_UINT8( packet_length, 0x02 );
    TEST_ASSERT_EQUAL_UINT8( number_of_hops, 0x03 );
    TEST_ASSERT_EQUAL_UINT16( symbols_per_freq[0].nb_symbols, 0x0405 );
    TEST_ASSERT_EQUAL_UINT32( symbols_per_freq[0].frequency, 0x06070809 );
    TEST_ASSERT_EQUAL_UINT16( symbols_per_freq[1].nb_symbols, 0x0A0B );
    TEST_ASSERT_EQUAL_UINT32( symbols_per_freq[1].frequency, 0x0C0D0E0F );
}