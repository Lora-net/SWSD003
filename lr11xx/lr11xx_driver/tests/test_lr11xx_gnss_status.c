#include "lr11xx_gnss.h"
#include "lr11xx_regmem.h"
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

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_get_result_size_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    uint16_t              resultSize = 0x0000;
    const lr11xx_status_t status     = lr11xx_gnss_get_result_size( context, &resultSize );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_read_results_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t result_buffer[1] = { 0 };
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    const lr11xx_status_t status = lr11xx_gnss_read_results( context, result_buffer, 0 );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_almanac_update_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    const uint8_t expected_number_of_calls = 2;
    for( uint8_t index_call = 0; index_call < expected_number_of_calls; index_call++ )
    {
        lr11xx_hal_write_IgnoreAndReturn( hal_status );
    }
    lr11xx_hal_write_IgnoreAndReturn( hal_status );
    uint8_t               almanac_bytestream[2580];
    const lr11xx_status_t status = lr11xx_gnss_almanac_update( context, almanac_bytestream, 129 );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_get_almanac_age_for_satellite_status( lr11xx_status_t     status_expected,
                                                            lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    uint16_t                   almanac_age  = 0;
    lr11xx_gnss_satellite_id_t satellite_id = 1;
    const lr11xx_status_t status = lr11xx_gnss_get_almanac_age_for_satellite( context, satellite_id, &almanac_age );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_read_almanac_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    lr11xx_gnss_almanac_full_read_bytestream_t almanac_read = { 0 };
    const lr11xx_status_t                      status       = lr11xx_gnss_read_almanac( context, almanac_read );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_set_constellation_to_use_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_write_IgnoreAndReturn( hal_status );
    const lr11xx_status_t status = lr11xx_gnss_set_constellations_to_use( context, LR11XX_GNSS_GPS_MASK );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_read_used_constellations_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    lr11xx_gnss_constellation_mask_t constellation_in_use = 0;
    const lr11xx_status_t            status = lr11xx_gnss_read_used_constellations( context, &constellation_in_use );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_set_almanac_update_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_write_IgnoreAndReturn( hal_status );
    lr11xx_gnss_constellation_mask_t almanac_update_bit_mask = 0;
    const lr11xx_status_t            status = lr11xx_gnss_set_almanac_update( context, almanac_update_bit_mask );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_read_almanac_update_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    lr11xx_gnss_constellation_mask_t almanac_update_bit_mask = 0;
    const lr11xx_status_t            status = lr11xx_gnss_read_almanac_update( context, &almanac_update_bit_mask );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_read_firmware_version_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    lr11xx_gnss_version_t version = { 0 };
    const lr11xx_status_t status  = lr11xx_gnss_read_firmware_version( context, &version );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_read_supported_constellation_status( lr11xx_status_t     status_expected,
                                                           lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    lr11xx_gnss_constellation_mask_t supported_constellation = 0;
    const lr11xx_status_t status = lr11xx_gnss_read_supported_constellations( context, &supported_constellation );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_get_timings_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );

    lr11xx_gnss_timings_t gnss_timing = { 0 };
    const lr11xx_status_t status      = lr11xx_gnss_get_timings( context, &gnss_timing );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_set_scan_mode_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_write_IgnoreAndReturn( hal_status );

    lr11xx_gnss_scan_mode_t expected_scan_mode = LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS;
    const lr11xx_status_t   status             = lr11xx_gnss_set_scan_mode( context, expected_scan_mode );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_scan_autonomous_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_write_IgnoreAndReturn( hal_status );
    const lr11xx_status_t status = lr11xx_gnss_scan_autonomous( context, 100, LR11XX_GNSS_OPTION_DEFAULT, 222, 47 );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_scan_assisted_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_write_IgnoreAndReturn( hal_status );
    const lr11xx_status_t status = lr11xx_gnss_scan_assisted( context, 25100, LR11XX_GNSS_OPTION_DEFAULT, 1, 10 );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_set_assistance_position_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_write_IgnoreAndReturn( hal_status );
    lr11xx_gnss_solver_assistance_position_t assistance_position = { };
    const lr11xx_status_t status = lr11xx_gnss_set_assistance_position( context, &assistance_position );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_read_assistance_position_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_gnss_solver_assistance_position_t assistance_position;
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    const lr11xx_status_t status = lr11xx_gnss_read_assistance_position( context, &assistance_position );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_push_dmc_msg_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_write_IgnoreAndReturn( hal_status );
    uint8_t               dmc_message[1] = { 0 };
    const lr11xx_status_t status         = lr11xx_gnss_push_dmc_msg( context, dmc_message, 0 );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_push_solver_msg_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_write_IgnoreAndReturn( hal_status );
    uint8_t               payload[1] = { 1 };
    const lr11xx_status_t status     = lr11xx_gnss_push_solver_msg( context, payload, 1 );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_get_nb_detected_satellites_status( lr11xx_status_t     status_expected,
                                                         lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    uint8_t               nb_satellite_detected = 0;
    const lr11xx_status_t status = lr11xx_gnss_get_nb_detected_satellites( context, &nb_satellite_detected );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_get_detected_satellites_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    lr11xx_gnss_detected_satellite_t detected_satellites[4] = { 0 };
    const lr11xx_status_t            status = lr11xx_gnss_get_detected_satellites( context, 4, detected_satellites );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_get_context_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[2]    = { 0x04, 0x16 };
    uint8_t rbuffer_in_expected[9] = { 0x00 };
    uint8_t rbuffer_out_faked[9]   = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x45, 0x80 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 9, 9,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 9 );

    lr11xx_gnss_context_status_bytestream_t context_status_bytestream = { 0 };

    const lr11xx_status_t status = lr11xx_gnss_get_context_status( context, context_status_bytestream );

    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( rbuffer_out_faked, context_status_bytestream,
                                       LR11XX_GNSS_CONTEXT_STATUS_LENGTH );
    }

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, 0x02, 0x18 )
TEST_VALUE( LR11XX_STATUS_ERROR, 0x01, 0x18 )
TEST_VALUE( LR11XX_STATUS_ERROR, 0x02, 0x19 )
void test_lr11xx_gnss_parse_context_status( lr11xx_status_t status_expected, uint8_t destination, uint8_t opcode )
{
    const uint8_t context_status_bytestream[9] = { destination, opcode, 0x03, 0x04, 0x05, 0x06, 0x07, 0x45, 0x80 };

    lr11xx_gnss_context_status_t context_status = { 0 };

    const lr11xx_status_t status =
        lr11xx_gnss_parse_context_status_buffer( context_status_bytestream, &context_status );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );

    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( 0x03, context_status.firmware_version );
        TEST_ASSERT_EQUAL_UINT32( 0x07060504, context_status.global_almanac_crc );
        TEST_ASSERT_EQUAL_UINT8( LR11XX_GNSS_ERROR_ALMANAC_UPDATE_NOT_ALLOWED, context_status.error_code );
        TEST_ASSERT_FALSE( context_status.almanac_update_gps );
        TEST_ASSERT_TRUE( context_status.almanac_update_beidou );
        TEST_ASSERT_EQUAL_UINT8( LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_2_KHZ, context_status.freq_search_space );
    }
}