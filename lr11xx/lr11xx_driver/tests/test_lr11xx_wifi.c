#include "lr11xx_wifi.h"
#include "unity.h"

#include <string.h>  // for memset

#include "mock_lr11xx_hal.h"

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

#define EXPECTED_WIFI_SCAN_CBUFFER_SIZE ( 11 )
#define EXPECTED_WIFI_SCAN_TIME_LIMIT_CBUFFER_SIZE ( 11 )
#define EXPECTED_WIFI_COUNTRY_CODE_TIME_LIMIT_CBUFFER_SIZE ( 9 )
#define EXPECTED_WIFI_SEARCH_COUNTRY_CODE_CBUFFER_SIZE ( 9 )
#define EXPECTED_WIFI_READ_CUMULATIVE_CBUFFER_SIZE ( 16 )
#define EXPECTED_WIFI_GET_SIZE_COUNTRY_RESULT_CBUFFER_SIZE ( 2 )
#define EXPECTED_WIFI_READ_COUNTRY_RESULT_CBUFFER_SIZE ( 4 )
#define EXPECTED_WIFI_CONFIGURE_TIMESTAMP_AP_PHONE_CBUFFER_SIZE ( 6 )
#define WIFI_READ_VERSION_CBUFFER_SIZE ( 2 )
#define WIFI_READ_CUMULATIVE_RBUFFER_SIZE ( 2 )
#define WIFI_GET_COUNTRY_RESULT_SIZE_RBUFFER_SIZE ( 1 )

void* context = 0;

void setUp( void )
{
}

void tearDown( void )
{
}

uint16_t wifi_channel_value_to_mask( const lr11xx_wifi_channel_t channel_value )
{
    return 1 << ( ( uint8_t ) channel_value - 1 );
}

TEST_VALUE( 1 )
TEST_VALUE( 0 )
void test_lr11xx_wifi_Scan( uint8_t abort_if_timeout_u )
{
    const lr11xx_wifi_signal_type_scan_t signal_type  = LR11XX_WIFI_TYPE_SCAN_B;
    const lr11xx_wifi_channel_mask_t     channel_mask = wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_1 ) |
                                                    wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_6 ) |
                                                    wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_11 );
    const lr11xx_wifi_mode_t scan_mode           = LR11XX_WIFI_SCAN_MODE_BEACON;
    const uint8_t            nb_max_results      = 16;
    const uint8_t            nb_scan_per_channel = 5;
    const uint16_t           timeout_in_ms       = 101;
    const bool               abort_if_timeout    = ( abort_if_timeout_u == 0 ) ? false : true;

    const uint8_t cbuffer_expected[EXPECTED_WIFI_SCAN_CBUFFER_SIZE] = {
        0x03,
        0x00,
        ( uint8_t ) signal_type,
        ( uint8_t ) ( channel_mask >> 8 ),
        ( uint8_t ) channel_mask,
        ( uint8_t ) scan_mode,
        nb_max_results,
        nb_scan_per_channel,
        ( uint8_t ) ( timeout_in_ms >> 8 ),
        ( uint8_t ) timeout_in_ms,
        ( abort_if_timeout ) ? 1 : 0,
    };

    uint8_t rbuffer_in_expected[EXPECTED_WIFI_SCAN_CBUFFER_SIZE] = { 0 };
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, EXPECTED_WIFI_SCAN_CBUFFER_SIZE,
                                               EXPECTED_WIFI_SCAN_CBUFFER_SIZE, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_wifi_scan( context, signal_type, channel_mask, scan_mode, nb_max_results, nb_scan_per_channel, timeout_in_ms,
                      abort_if_timeout );
}

TEST_VALUE( LR11XX_WIFI_SCAN_MODE_BEACON )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_FULL_BEACON )
void test_alpha_lr11xx_wifi_ScanTimeLimit( lr11xx_wifi_mode_t scan_mode )
{
    const lr11xx_wifi_signal_type_scan_t signal_type  = LR11XX_WIFI_TYPE_SCAN_B;
    const lr11xx_wifi_channel_mask_t     channel_mask = wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_1 ) |
                                                    wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_6 ) |
                                                    wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_11 );
    const uint8_t  nb_max_results         = 16;
    const uint16_t timeout_per_channel_ms = 101;
    const uint16_t timeout_per_scan_ms    = 300;

    const uint8_t cbuffer_expected[EXPECTED_WIFI_SCAN_TIME_LIMIT_CBUFFER_SIZE] = {
        0x03,
        0x01,
        ( uint8_t ) signal_type,
        ( uint8_t ) ( channel_mask >> 8 ),
        ( uint8_t ) channel_mask,
        ( uint8_t ) scan_mode,
        nb_max_results,
        ( uint8_t ) ( timeout_per_channel_ms >> 8 ),
        ( uint8_t ) timeout_per_channel_ms,
        ( uint8_t ) ( timeout_per_scan_ms >> 8 ),
        ( uint8_t ) timeout_per_scan_ms,
    };

    uint8_t rbuffer_in_expected[EXPECTED_WIFI_SCAN_TIME_LIMIT_CBUFFER_SIZE] = { 0 };
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, EXPECTED_WIFI_SCAN_TIME_LIMIT_CBUFFER_SIZE,
                                               EXPECTED_WIFI_SCAN_TIME_LIMIT_CBUFFER_SIZE, NULL, 0, 0,
                                               LR11XX_HAL_STATUS_OK );

    lr11xx_wifi_scan_time_limit( context, signal_type, channel_mask, scan_mode, nb_max_results, timeout_per_channel_ms,
                                 timeout_per_scan_ms );
}

TEST_VALUE( 1 )
TEST_VALUE( 0 )
void test_lr11xx_wifi_SearchCountryCode( uint8_t abort_if_timeout_uint )
{
    const lr11xx_wifi_channel_mask_t channels_mask        = 0x03FFF;
    const uint16_t                   timeout_in_ms        = 110;
    const uint8_t                    nb_max_results       = 32;
    const uint8_t                    nb_scan_per_channels = 255;
    const bool                       abort_if_timeout     = ( abort_if_timeout_uint == 1 );

    const uint8_t expected_cbuffer[EXPECTED_WIFI_SEARCH_COUNTRY_CODE_CBUFFER_SIZE] = {
        0x03,
        0x02,
        ( uint8_t ) ( channels_mask >> 8 ),
        ( uint8_t ) ( channels_mask >> 0 ),
        nb_max_results,
        nb_scan_per_channels,
        ( uint8_t ) ( timeout_in_ms >> 8 ),
        ( uint8_t ) ( timeout_in_ms >> 0 ),
        ( uint8_t ) ( abort_if_timeout ? 1 : 0 )
    };

    uint8_t rbuffer_in_expected[EXPECTED_WIFI_SEARCH_COUNTRY_CODE_CBUFFER_SIZE] = { 0 };
    lr11xx_hal_write_ExpectWithArrayAndReturn(
        context, 0, expected_cbuffer, EXPECTED_WIFI_SEARCH_COUNTRY_CODE_CBUFFER_SIZE,
        EXPECTED_WIFI_SEARCH_COUNTRY_CODE_CBUFFER_SIZE, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_wifi_search_country_code( context, channels_mask, nb_max_results, nb_scan_per_channels, timeout_in_ms,
                                     abort_if_timeout );
}

void test_alpha_lr11xx_wifi_CountryCodeTimeLimit( void )
{
    const lr11xx_wifi_channel_mask_t channel_mask = wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_1 ) |
                                                    wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_6 ) |
                                                    wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_11 );
    const uint8_t  nb_max_results         = 20;
    const uint16_t timeout_per_channel_ms = 300;
    const uint16_t timeout_per_scan_ms    = 105;

    const uint8_t cbuffer_expected[EXPECTED_WIFI_COUNTRY_CODE_TIME_LIMIT_CBUFFER_SIZE] = {
        0x03, 0x03, 0x04, 0x21, 0x14, 0x01, 0x2C, 0x00, 0x69,
    };

    uint8_t rbuffer_in_expected[EXPECTED_WIFI_COUNTRY_CODE_TIME_LIMIT_CBUFFER_SIZE] = { 0 };
    lr11xx_hal_write_ExpectWithArrayAndReturn(
        context, 0, cbuffer_expected, EXPECTED_WIFI_COUNTRY_CODE_TIME_LIMIT_CBUFFER_SIZE,
        EXPECTED_WIFI_COUNTRY_CODE_TIME_LIMIT_CBUFFER_SIZE, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_wifi_search_country_code_time_limit( context, channel_mask, nb_max_results, timeout_per_channel_ms,
                                                timeout_per_scan_ms );
}

TEST_VALUE( 0x00 )
TEST_VALUE( 0x03 )
TEST_VALUE( 0x30 )
TEST_VALUE( 0x31 )
TEST_VALUE( 0xFF )
void test_lr11xx_wifi_GetSizeResults( uint8_t expected_result_size )
{
    uint8_t cbuffer_expected[]     = { 0x03, 0x05 };
    uint8_t rbuffer_in_expected[1] = { 0x00 };
    uint8_t rbuffer_out_faked[]    = { expected_result_size };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    /************************/
    /* Perform transactions */
    /************************/
    uint8_t nb_results = 0x00;
    lr11xx_wifi_get_nb_results( context, &nb_results );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8( expected_result_size, nb_results );
}

void test_lr11xx_wifi_ReadBasicCompleteResults( void )
{
    const uint16_t nb_results  = 3;
    const uint16_t result_size = nb_results * 22;

    const uint8_t cbuffer_expected[]      = { 0x03, 0x06, 0x00, nb_results, 0x01 };
    uint8_t       rbuffer_in_expected[66] = { 0x00 };

    const uint8_t                          macAddress_exp1[]    = { 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
    const uint8_t                          channel_info_mac1    = 0xAA;
    lr11xx_wifi_frame_type_info_byte_t     frame_type_info_mac1 = 0xB1;
    int16_t                                phi_offset_mac1      = 0xA1;
    uint64_t                               timestamp_us_mac1    = 251000;
    const lr11xx_wifi_signal_type_result_t wifi_type_mac1       = LR11XX_WIFI_TYPE_RESULT_B;
    const uint8_t                          wifi_rssi_mac1       = 0xF0;
    uint16_t                               period_beacon_mac1   = 0x4321;

    const uint8_t                          macAddress_exp2[]    = { 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13 };
    const uint8_t                          channel_info_mac2    = 0x0D;
    lr11xx_wifi_frame_type_info_byte_t     frame_type_info_mac2 = 0xB2;
    int16_t                                phi_offset_mac2      = 0xA2;
    uint64_t                               timestamp_us_mac2    = 252000;
    const lr11xx_wifi_signal_type_result_t wifi_type_mac2       = LR11XX_WIFI_TYPE_RESULT_G;
    const uint8_t                          wifi_rssi_mac2       = 0x42;
    uint16_t                               period_beacon_mac2   = 0x4322;

    const uint8_t                          macAddress_exp3[]    = { 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };
    const uint8_t                          channel_info_mac3    = 0x20;
    lr11xx_wifi_frame_type_info_byte_t     frame_type_info_mac3 = 0xB3;
    int16_t                                phi_offset_mac3      = 0xA3;
    uint64_t                               timestamp_us_mac3    = 253000;
    const lr11xx_wifi_signal_type_result_t wifi_type_mac3       = LR11XX_WIFI_TYPE_RESULT_N;
    const uint8_t                          wifi_rssi_mac3       = 0x55;
    uint16_t                               period_beacon_mac3   = 0x4323;

    uint8_t rbuffer_out_faked[] = {
        ( uint8_t ) wifi_type_mac1,
        channel_info_mac1,
        wifi_rssi_mac1,
        frame_type_info_mac1,
        macAddress_exp1[0],
        macAddress_exp1[1],
        macAddress_exp1[2],
        macAddress_exp1[3],
        macAddress_exp1[4],
        macAddress_exp1[5],
        ( ( uint8_t ) ( phi_offset_mac1 >> 8 ) ),
        ( ( uint8_t ) ( phi_offset_mac1 >> 0 ) ),
        ( ( uint8_t ) ( timestamp_us_mac1 >> 56 ) ),
        ( ( uint8_t ) ( timestamp_us_mac1 >> 48 ) ),
        ( ( uint8_t ) ( timestamp_us_mac1 >> 40 ) ),
        ( ( uint8_t ) ( timestamp_us_mac1 >> 32 ) ),
        ( ( uint8_t ) ( timestamp_us_mac1 >> 24 ) ),
        ( ( uint8_t ) ( timestamp_us_mac1 >> 16 ) ),
        ( ( uint8_t ) ( timestamp_us_mac1 >> 8 ) ),
        ( ( uint8_t ) ( timestamp_us_mac1 >> 0 ) ),
        ( ( uint8_t ) ( period_beacon_mac1 >> 8 ) ),
        ( ( uint8_t ) ( period_beacon_mac1 >> 0 ) ),

        ( uint8_t ) wifi_type_mac2,
        channel_info_mac2,
        wifi_rssi_mac2,
        frame_type_info_mac2,
        macAddress_exp2[0],
        macAddress_exp2[1],
        macAddress_exp2[2],
        macAddress_exp2[3],
        macAddress_exp2[4],
        macAddress_exp2[5],
        ( ( uint8_t ) ( phi_offset_mac2 >> 8 ) ),
        ( ( uint8_t ) ( phi_offset_mac2 >> 0 ) ),
        ( ( uint8_t ) ( timestamp_us_mac2 >> 56 ) ),
        ( ( uint8_t ) ( timestamp_us_mac2 >> 48 ) ),
        ( ( uint8_t ) ( timestamp_us_mac2 >> 40 ) ),
        ( ( uint8_t ) ( timestamp_us_mac2 >> 32 ) ),
        ( ( uint8_t ) ( timestamp_us_mac2 >> 24 ) ),
        ( ( uint8_t ) ( timestamp_us_mac2 >> 16 ) ),
        ( ( uint8_t ) ( timestamp_us_mac2 >> 8 ) ),
        ( ( uint8_t ) ( timestamp_us_mac2 >> 0 ) ),
        ( ( uint8_t ) ( period_beacon_mac2 >> 8 ) ),
        ( ( uint8_t ) ( period_beacon_mac2 >> 0 ) ),

        ( uint8_t ) wifi_type_mac3,
        channel_info_mac3,
        wifi_rssi_mac3,
        frame_type_info_mac3,
        macAddress_exp3[0],
        macAddress_exp3[1],
        macAddress_exp3[2],
        macAddress_exp3[3],
        macAddress_exp3[4],
        macAddress_exp3[5],
        ( ( uint8_t ) ( phi_offset_mac3 >> 8 ) ),
        ( ( uint8_t ) ( phi_offset_mac3 >> 0 ) ),
        ( ( uint8_t ) ( timestamp_us_mac3 >> 56 ) ),
        ( ( uint8_t ) ( timestamp_us_mac3 >> 48 ) ),
        ( ( uint8_t ) ( timestamp_us_mac3 >> 40 ) ),
        ( ( uint8_t ) ( timestamp_us_mac3 >> 32 ) ),
        ( ( uint8_t ) ( timestamp_us_mac3 >> 24 ) ),
        ( ( uint8_t ) ( timestamp_us_mac3 >> 16 ) ),
        ( ( uint8_t ) ( timestamp_us_mac3 >> 8 ) ),
        ( ( uint8_t ) ( timestamp_us_mac3 >> 0 ) ),
        ( ( uint8_t ) ( period_beacon_mac3 >> 8 ) ),
        ( ( uint8_t ) ( period_beacon_mac3 >> 0 ) ),
    };

    lr11xx_wifi_basic_complete_result_t results[3] = { 0 };

    /********************/
    /* Set expectations */
    /********************/

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 5, 5, rbuffer_in_expected, result_size,
                                              result_size, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, result_size );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_read_basic_complete_results( context, 0, nb_results, results );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8( wifi_type_mac1, results[0].data_rate_info_byte );
    TEST_ASSERT_EQUAL_UINT8( channel_info_mac1, results[0].channel_info_byte );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp1, results[0].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac1, results[0].rssi );
    TEST_ASSERT_EQUAL_UINT8( frame_type_info_mac1, results[0].frame_type_info_byte );
    TEST_ASSERT_EQUAL_UINT8( phi_offset_mac1, results[0].phi_offset );
    TEST_ASSERT_EQUAL_UINT8( timestamp_us_mac1, results[0].timestamp_us );
    TEST_ASSERT_EQUAL_UINT8( period_beacon_mac1, results[0].beacon_period_tu );

    TEST_ASSERT_EQUAL_UINT8( wifi_type_mac2, results[1].data_rate_info_byte );
    TEST_ASSERT_EQUAL_UINT8( channel_info_mac2, results[1].channel_info_byte );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp2, results[1].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac2, results[1].rssi );
    TEST_ASSERT_EQUAL_UINT8( frame_type_info_mac2, results[1].frame_type_info_byte );
    TEST_ASSERT_EQUAL_UINT8( phi_offset_mac2, results[1].phi_offset );
    TEST_ASSERT_EQUAL_UINT8( timestamp_us_mac2, results[1].timestamp_us );
    TEST_ASSERT_EQUAL_UINT8( period_beacon_mac2, results[1].beacon_period_tu );

    TEST_ASSERT_EQUAL_UINT8( wifi_type_mac3, results[2].data_rate_info_byte );
    TEST_ASSERT_EQUAL_UINT8( channel_info_mac3, results[2].channel_info_byte );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp3, results[2].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac3, results[2].rssi );
    TEST_ASSERT_EQUAL_UINT8( frame_type_info_mac3, results[2].frame_type_info_byte );
    TEST_ASSERT_EQUAL_UINT8( phi_offset_mac3, results[2].phi_offset );
    TEST_ASSERT_EQUAL_UINT8( timestamp_us_mac3, results[2].timestamp_us );
    TEST_ASSERT_EQUAL_UINT8( period_beacon_mac3, results[2].beacon_period_tu );
}

void test_lr11xx_wifi_ReadBasicMacTypeChannelRssiResults( void )
{
    const uint16_t nb_results  = 3;
    const uint16_t result_size = nb_results * 9;

    const uint8_t cbuffer_expected[]      = { 0x03, 0x06, 0x00, nb_results, 0x04 };
    uint8_t       rbuffer_in_expected[27] = { 0x00 };

    const uint8_t                          macAddress_exp1[] = { 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
    const uint8_t                          channel_info_mac1 = 0xAA;
    const lr11xx_wifi_signal_type_result_t wifi_type_mac1    = LR11XX_WIFI_TYPE_RESULT_B;
    const uint8_t                          wifi_rssi_mac1    = 0xF0;

    const uint8_t                          macAddress_exp2[] = { 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13 };
    const uint8_t                          channel_info_mac2 = 0x0D;
    const lr11xx_wifi_signal_type_result_t wifi_type_mac2    = LR11XX_WIFI_TYPE_RESULT_G;
    const uint8_t                          wifi_rssi_mac2    = 0x42;

    const uint8_t                          macAddress_exp3[] = { 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };
    const uint8_t                          channel_info_mac3 = 0x20;
    const lr11xx_wifi_signal_type_result_t wifi_type_mac3    = LR11XX_WIFI_TYPE_RESULT_N;
    const uint8_t                          wifi_rssi_mac3    = 0x55;

    uint8_t rbuffer_out_faked[] = {
        ( uint8_t ) wifi_type_mac1, channel_info_mac1,  wifi_rssi_mac1,     macAddress_exp1[0], macAddress_exp1[1],
        macAddress_exp1[2],         macAddress_exp1[3], macAddress_exp1[4], macAddress_exp1[5],

        ( uint8_t ) wifi_type_mac2, channel_info_mac2,  wifi_rssi_mac2,     macAddress_exp2[0], macAddress_exp2[1],
        macAddress_exp2[2],         macAddress_exp2[3], macAddress_exp2[4], macAddress_exp2[5],

        ( uint8_t ) wifi_type_mac3, channel_info_mac3,  wifi_rssi_mac3,     macAddress_exp3[0], macAddress_exp3[1],
        macAddress_exp3[2],         macAddress_exp3[3], macAddress_exp3[4], macAddress_exp3[5],
    };

    lr11xx_wifi_basic_mac_type_channel_result_t results[3] = { 0 };

    /********************/
    /* Set expectations */
    /********************/

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 5, 5, rbuffer_in_expected, result_size,
                                              result_size, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, result_size );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_read_basic_mac_type_channel_results( context, 0, nb_results, results );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8( wifi_type_mac1, results[0].data_rate_info_byte );
    TEST_ASSERT_EQUAL_UINT8( channel_info_mac1, results[0].channel_info_byte );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp1, results[0].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac1, results[0].rssi );

    TEST_ASSERT_EQUAL_UINT8( wifi_type_mac2, results[1].data_rate_info_byte );
    TEST_ASSERT_EQUAL_UINT8( channel_info_mac2, results[1].channel_info_byte );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp2, results[1].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac2, results[1].rssi );

    TEST_ASSERT_EQUAL_UINT8( wifi_type_mac3, results[2].data_rate_info_byte );
    TEST_ASSERT_EQUAL_UINT8( channel_info_mac3, results[2].channel_info_byte );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp3, results[2].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac3, results[2].rssi );
}

void test_lr11xx_wifi_ReadExtendedFullResultsWithManyResults( void )
{
    const uint16_t nb_results_all = 16;

    const uint16_t nb_results1               = 12;
    const uint8_t  cbuffer1_expected[]       = { 0x03, 0x06, 0x00, nb_results1, 0x01 };
    uint8_t        rbuffer1_in_expected[948] = { 0x00 };
    const uint16_t result_size1              = nb_results1 * 79;

    const uint16_t nb_results2               = 4;
    const uint8_t  cbuffer2_expected[]       = { 0x03, 0x06, 12, nb_results2, 0x01 };
    uint8_t        rbuffer2_in_expected[316] = { 0x00 };
    const uint16_t result_size2              = nb_results2 * 79;

    lr11xx_wifi_mac_address_t mac_addresses[16] = { 0 };
    for( uint8_t mac_index = 0; mac_index < 16; mac_index++ )
    {
        mac_addresses[mac_index][0] = ( mac_index + 1 );
        mac_addresses[mac_index][1] = ( mac_index + 1 ) * 2;
        mac_addresses[mac_index][2] = ( mac_index + 1 ) * 3;
        mac_addresses[mac_index][3] = ( mac_index + 1 ) * 4;
        mac_addresses[mac_index][4] = ( mac_index + 1 ) * 5;
        mac_addresses[mac_index][5] = ( mac_index + 1 ) * 6;
    }

    uint8_t rbuffer_out_faked1[948] = { 0 };
    for( uint8_t mac_index1 = 0; mac_index1 < nb_results1; mac_index1++ )
    {
        const uint16_t             local_rbuffer_index        = mac_index1 * 79 + 10;
        lr11xx_wifi_mac_address_t* local_expected_mac_address = &mac_addresses[mac_index1];
        rbuffer_out_faked1[local_rbuffer_index + 0]           = ( *local_expected_mac_address )[0];
        rbuffer_out_faked1[local_rbuffer_index + 1]           = ( *local_expected_mac_address )[1];
        rbuffer_out_faked1[local_rbuffer_index + 2]           = ( *local_expected_mac_address )[2];
        rbuffer_out_faked1[local_rbuffer_index + 3]           = ( *local_expected_mac_address )[3];
        rbuffer_out_faked1[local_rbuffer_index + 4]           = ( *local_expected_mac_address )[4];
        rbuffer_out_faked1[local_rbuffer_index + 5]           = ( *local_expected_mac_address )[5];
    }

    uint8_t rbuffer_out_faked2[316] = { 0 };
    for( uint8_t mac_index2 = 0; mac_index2 < nb_results2; mac_index2++ )
    {
        const uint16_t             local_rbuffer_index        = mac_index2 * 79 + 10;
        lr11xx_wifi_mac_address_t* local_expected_mac_address = &mac_addresses[mac_index2 + nb_results1];
        rbuffer_out_faked2[local_rbuffer_index + 0]           = ( *local_expected_mac_address )[0];
        rbuffer_out_faked2[local_rbuffer_index + 1]           = ( *local_expected_mac_address )[1];
        rbuffer_out_faked2[local_rbuffer_index + 2]           = ( *local_expected_mac_address )[2];
        rbuffer_out_faked2[local_rbuffer_index + 3]           = ( *local_expected_mac_address )[3];
        rbuffer_out_faked2[local_rbuffer_index + 4]           = ( *local_expected_mac_address )[4];
        rbuffer_out_faked2[local_rbuffer_index + 5]           = ( *local_expected_mac_address )[5];
    }

    lr11xx_wifi_extended_full_result_t results[16] = { 0 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer1_expected, 5, 5, rbuffer1_in_expected, result_size1,
                                              result_size1, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked1, result_size1 );
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer2_expected, 5, 5, rbuffer2_in_expected, result_size2,
                                              result_size2, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked2, result_size2 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_read_extended_full_results( context, 0, nb_results_all, results );

    /*****************/
    /* Check results */
    /*****************/
    for( uint8_t index_result = 0; index_result < nb_results_all; index_result++ )
    {
        const lr11xx_wifi_mac_address_t* local_expected_mac         = &mac_addresses[index_result];
        const lr11xx_wifi_mac_address_t* local_received_mac_address = &results[index_result].mac_address_1;
        TEST_ASSERT_EQUAL_UINT8_ARRAY( local_expected_mac, local_received_mac_address, 6 );
    }
}

void test_lr11xx_wifi_ResetCumulativeTimings( void )
{
    uint8_t cbuffer_expected[] = { 0x03, 0x07 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_reset_cumulative_timing( context );
}

TEST_VALUE( 100, 100, 100, 100 )
TEST_VALUE( 100, 0, 0, 0 )
TEST_VALUE( 0, 100, 0, 0 )
TEST_VALUE( 0, 0, 100, 0 )
TEST_VALUE( 0, 0, 0, 100 )
TEST_VALUE( 0, 0, 0, 0 )
void test_lr11xx_wifi_ReadCumulTim( int det, int corr, int capt, int dem )
{
    // const uint32_t det   = 100;
    // const uint32_t corr = 100;
    // const uint32_t capt     = 100;
    // const uint32_t dem   = 100;

    const uint16_t result_size        = EXPECTED_WIFI_READ_CUMULATIVE_CBUFFER_SIZE;
    uint8_t        cbuffer_expected[] = { 0x03, 0x08 };
    uint8_t        rbuffer_in_expected[EXPECTED_WIFI_READ_CUMULATIVE_CBUFFER_SIZE] = { 0 };
    uint8_t        rbuffer_out_faked[EXPECTED_WIFI_READ_CUMULATIVE_CBUFFER_SIZE]   = {
        ( uint8_t ) ( det >> 24 ),  ( uint8_t ) ( det >> 16 ),  ( uint8_t ) ( det >> 8 ),  ( uint8_t ) det,
        ( uint8_t ) ( corr >> 24 ), ( uint8_t ) ( corr >> 16 ), ( uint8_t ) ( corr >> 8 ), ( uint8_t ) corr,
        ( uint8_t ) ( capt >> 24 ), ( uint8_t ) ( capt >> 16 ), ( uint8_t ) ( capt >> 8 ), ( uint8_t ) capt,
        ( uint8_t ) ( dem >> 24 ),  ( uint8_t ) ( dem >> 16 ),  ( uint8_t ) ( dem >> 8 ),  ( uint8_t ) dem,
    };

    lr11xx_wifi_cumulative_timings_t timings = { 0 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, result_size,
                                              result_size, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, result_size );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_read_cumulative_timing( context, &timings );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT32( det, timings.rx_detection_us );
    TEST_ASSERT_EQUAL_UINT32( corr, timings.rx_correlation_us );
    TEST_ASSERT_EQUAL_UINT32( capt, timings.rx_capture_us );
    TEST_ASSERT_EQUAL_UINT32( dem, timings.demodulation_us );
}

void test_lr11xx_wifi_GetSizeCountryResults( )
{
    const uint8_t expected_nb_country_results = 10;

    uint8_t cbuffer_expected[EXPECTED_WIFI_GET_SIZE_COUNTRY_RESULT_CBUFFER_SIZE] = { 0x03, 0x09 };

    uint8_t rbuffer_in_expected[WIFI_GET_COUNTRY_RESULT_SIZE_RBUFFER_SIZE] = { 0 };
    uint8_t rbuffer_out_faked[WIFI_READ_CUMULATIVE_RBUFFER_SIZE]           = {
        ( uint8_t ) expected_nb_country_results,
    };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn(
        context, 0, cbuffer_expected, EXPECTED_WIFI_GET_SIZE_COUNTRY_RESULT_CBUFFER_SIZE,
        EXPECTED_WIFI_GET_SIZE_COUNTRY_RESULT_CBUFFER_SIZE, rbuffer_in_expected,
        WIFI_GET_COUNTRY_RESULT_SIZE_RBUFFER_SIZE, WIFI_GET_COUNTRY_RESULT_SIZE_RBUFFER_SIZE, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, WIFI_GET_COUNTRY_RESULT_SIZE_RBUFFER_SIZE );

    /************************/
    /* Perform transactions */
    /************************/
    uint8_t nb_country_code_results = 0;
    lr11xx_wifi_get_nb_country_code_results( context, &nb_country_code_results );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8( expected_nb_country_results, nb_country_code_results );
}

void assert_wifi_country_code( const lr11xx_wifi_country_code_t expected, const lr11xx_wifi_country_code_t actual )
{
    TEST_ASSERT_EQUAL_UINT8_ARRAY( expected.country_code, actual.country_code, 2 );
    TEST_ASSERT_EQUAL_UINT8( expected.io_regulation, actual.io_regulation );
    TEST_ASSERT_EQUAL_UINT8( expected.channel_info_byte, actual.channel_info_byte );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( expected.mac_address, actual.mac_address, 6 );
}

#define EXPECTED_NUMBER_OF_COUNTRY_CODE ( 2 )
#define EXPECTED_COUNTRY_RESULT_BUFFER_SIZE ( EXPECTED_NUMBER_OF_COUNTRY_CODE * 10 )
void test_lr11xx_wifi_readCountryCodeResult( )
{
    const uint8_t expected_cbuffer[EXPECTED_WIFI_READ_COUNTRY_RESULT_CBUFFER_SIZE] = { 0x03, 0x0A, 0x00, 0x02 };

    const uint8_t expected_response_size                                = EXPECTED_COUNTRY_RESULT_BUFFER_SIZE;
    uint8_t       expected_rbuffer[EXPECTED_COUNTRY_RESULT_BUFFER_SIZE] = { 0 };
    uint8_t       fake_response[EXPECTED_COUNTRY_RESULT_BUFFER_SIZE]    = { 0 };

    for( uint16_t index_fake = 0; index_fake < expected_response_size; index_fake++ )
    {
        fake_response[index_fake] = index_fake;
    }

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, expected_cbuffer,
                                              EXPECTED_WIFI_READ_COUNTRY_RESULT_CBUFFER_SIZE,
                                              EXPECTED_WIFI_READ_COUNTRY_RESULT_CBUFFER_SIZE, expected_rbuffer,
                                              expected_response_size, expected_response_size, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( fake_response, expected_response_size );

    lr11xx_wifi_country_code_t expected_country_code_result[EXPECTED_NUMBER_OF_COUNTRY_CODE] = { 0 };
    expected_country_code_result[0].country_code[0]                                          = 0x00;
    expected_country_code_result[0].country_code[1]                                          = 0x01;
    expected_country_code_result[0].io_regulation                                            = 0x02;
    expected_country_code_result[0].channel_info_byte                                        = 0x03;
    expected_country_code_result[0].mac_address[5]                                           = 0x04;
    expected_country_code_result[0].mac_address[4]                                           = 0x05;
    expected_country_code_result[0].mac_address[3]                                           = 0x06;
    expected_country_code_result[0].mac_address[2]                                           = 0x07;
    expected_country_code_result[0].mac_address[1]                                           = 0x08;
    expected_country_code_result[0].mac_address[0]                                           = 0x09;
    expected_country_code_result[1].country_code[0]                                          = 0x0A;
    expected_country_code_result[1].country_code[1]                                          = 0x0B;
    expected_country_code_result[1].io_regulation                                            = 0x0C;
    expected_country_code_result[1].channel_info_byte                                        = 0x0D;
    expected_country_code_result[1].mac_address[5]                                           = 0x0E;
    expected_country_code_result[1].mac_address[4]                                           = 0x0F;
    expected_country_code_result[1].mac_address[3]                                           = 0x10;
    expected_country_code_result[1].mac_address[2]                                           = 0x11;
    expected_country_code_result[1].mac_address[1]                                           = 0x12;
    expected_country_code_result[1].mac_address[0]                                           = 0x13;

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_country_code_t country_code[EXPECTED_NUMBER_OF_COUNTRY_CODE] = { 0 };
    lr11xx_wifi_read_country_code_results( context, 0, EXPECTED_NUMBER_OF_COUNTRY_CODE, country_code );

    for( uint8_t index_country_code = 0; index_country_code < EXPECTED_NUMBER_OF_COUNTRY_CODE; index_country_code++ )
    {
        assert_wifi_country_code( expected_country_code_result[index_country_code], country_code[index_country_code] );
    }
}

void test_lr11xx_wifi_cfg_timestamp_ap_phone( )
{
    const uint32_t timestamp_s                                                               = 86400;
    const uint8_t  cbuffer_expected[EXPECTED_WIFI_CONFIGURE_TIMESTAMP_AP_PHONE_CBUFFER_SIZE] = {
        0x03,
        0x0B,
        ( ( uint8_t ) ( timestamp_s >> 24 ) ),
        ( ( uint8_t ) ( timestamp_s >> 16 ) ),
        ( ( uint8_t ) ( timestamp_s >> 8 ) ),
        ( ( uint8_t ) ( timestamp_s >> 0 ) ),
    };
    lr11xx_hal_write_ExpectWithArrayAndReturn(
        context, 0, cbuffer_expected, EXPECTED_WIFI_CONFIGURE_TIMESTAMP_AP_PHONE_CBUFFER_SIZE,
        EXPECTED_WIFI_CONFIGURE_TIMESTAMP_AP_PHONE_CBUFFER_SIZE, NULL, 0, 0, LR11XX_HAL_STATUS_OK );
    lr11xx_wifi_cfg_timestamp_ap_phone( context, timestamp_s );
}

void test_lr11xx_wifiReadVersion( )
{
    const lr11xx_wifi_version_t expected_version                                 = { .minor = 0xAE, .major = 0x01 };
    const uint16_t              result_size                                      = WIFI_READ_CUMULATIVE_RBUFFER_SIZE;
    uint8_t                     cbuffer_expected[WIFI_READ_VERSION_CBUFFER_SIZE] = { 0x03, 0x20 };
    uint8_t                     rbuffer_in_expected[WIFI_READ_CUMULATIVE_RBUFFER_SIZE] = { 0 };
    uint8_t                     rbuffer_out_faked[WIFI_READ_CUMULATIVE_RBUFFER_SIZE]   = {
        ( uint8_t ) expected_version.major,
        ( uint8_t ) expected_version.minor,
    };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, WIFI_READ_VERSION_CBUFFER_SIZE,
                                              WIFI_READ_VERSION_CBUFFER_SIZE, rbuffer_in_expected, result_size,
                                              result_size, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, result_size );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_version_t version = { 0 };
    lr11xx_wifi_read_version( context, &version );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8( expected_version.major, version.major );
    TEST_ASSERT_EQUAL_UINT8( expected_version.minor, version.minor );
}

TEST_VALUE( LR11XX_WIFI_CHANNEL_1, 0, LR11XX_WIFI_ORIGIN_BEACON_FIX_AP )
TEST_VALUE( LR11XX_WIFI_CHANNEL_1, 0, LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP )
TEST_VALUE( LR11XX_WIFI_CHANNEL_1, 0, LR11XX_WIFI_ORIGIN_UNKNOWN )
TEST_VALUE( LR11XX_WIFI_ALL_CHANNELS, 0, LR11XX_WIFI_ORIGIN_BEACON_FIX_AP )
TEST_VALUE( LR11XX_WIFI_ALL_CHANNELS, 0, LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP )
TEST_VALUE( LR11XX_WIFI_ALL_CHANNELS, 0, LR11XX_WIFI_ORIGIN_UNKNOWN )
TEST_VALUE( LR11XX_WIFI_ALL_CHANNELS, 1, LR11XX_WIFI_ORIGIN_BEACON_FIX_AP )
TEST_VALUE( LR11XX_WIFI_ALL_CHANNELS, 1, LR11XX_WIFI_ORIGIN_BEACON_MOBILE_AP )
TEST_VALUE( LR11XX_WIFI_ALL_CHANNELS, 1, LR11XX_WIFI_ORIGIN_UNKNOWN )
void test_lr11xx_wifi_parse_channel_info( const lr11xx_wifi_channel_t    expected_channel,
                                          const _Bool                    expected_rssiValidity,
                                          const lr11xx_wifi_mac_origin_t expected_macOriginEstimation )
{
    // const lr11xx_wifi_channel_t    expected_channel      =
    // LR11XX_WIFI_CHANNEL_1; const bool expected_rssiValidity = false; const
    // lr11xx_wifi_mac_origin_t expected_macOriginEstimation =
    //     LR11XX_WIFI_ORIGIN_BEACON_FIX_AP;

    const lr11xx_wifi_channel_info_byte_t channel_info = ( ( uint8_t ) expected_channel ) +
                                                         ( ( ( uint8_t ) expected_macOriginEstimation ) << 4 ) +
                                                         ( ( ( uint8_t ) ( expected_rssiValidity ? 0 : 1 ) ) << 6 );

    lr11xx_wifi_channel_t    channel       = 0;
    bool                     rssi_validity = false;
    lr11xx_wifi_mac_origin_t mac_origin    = LR11XX_WIFI_ORIGIN_BEACON_FIX_AP;

    lr11xx_wifi_parse_channel_info( channel_info, &channel, &rssi_validity, &mac_origin );

    TEST_ASSERT_EQUAL_UINT8( expected_channel, channel );
    TEST_ASSERT_EQUAL_UINT8( expected_macOriginEstimation, mac_origin );
    TEST_ASSERT_EQUAL_UINT8( expected_rssiValidity, rssi_validity );
}

void test_lr11xx_wifi_parse_frame_type_info( void )
{
    const lr11xx_wifi_frame_type_t     expected_frameType    = LR11XX_WIFI_FRAME_TYPE_MANAGEMENT;
    const lr11xx_wifi_frame_sub_type_t expected_frameSubType = 0x3;
    const bool                         expected_toDS         = true;
    const bool                         expected_fromDS       = false;

    const lr11xx_wifi_frame_type_info_byte_t expected_infoByte =
        ( expected_fromDS ? 1 : 0 ) + ( ( expected_toDS ? 1 : 0 ) << 1 ) + ( expected_frameSubType << 2 ) +
        ( expected_frameType << 4 );

    lr11xx_wifi_frame_type_t     frame_type     = 0;
    lr11xx_wifi_frame_sub_type_t frame_sub_type = 0;
    bool                         to_ds          = false;
    bool                         from_ds        = false;

    lr11xx_wifi_parse_frame_type_info( expected_infoByte, &frame_type, &frame_sub_type, &to_ds, &from_ds );

    TEST_ASSERT_EQUAL_UINT8( expected_frameType, frame_type );
    TEST_ASSERT_EQUAL_UINT8( expected_frameSubType, frame_sub_type );
    TEST_ASSERT_EQUAL_UINT8( expected_toDS, to_ds );
    TEST_ASSERT_EQUAL_UINT8( expected_fromDS, from_ds );
}

void test_lr11xx_wifi_parse_data_rate_info( void )
{
    const lr11xx_wifi_signal_type_result_t expected_wifiSignalType = LR11XX_WIFI_TYPE_RESULT_N;
    const lr11xx_wifi_datarate_t           expected_wifiDataRate   = LR11XX_WIFI_DATARATE_72_2_MBPS;

    const lr11xx_wifi_datarate_info_byte_t expected_dataRateInfoByte =
        expected_wifiSignalType + ( expected_wifiDataRate << 2 );

    lr11xx_wifi_signal_type_result_t signal_type = LR11XX_WIFI_TYPE_RESULT_B;
    lr11xx_wifi_datarate_t           datarate    = LR11XX_WIFI_DATARATE_1_MBPS;

    lr11xx_wifi_parse_data_rate_info( expected_dataRateInfoByte, &signal_type, &datarate );

    TEST_ASSERT_EQUAL_UINT8( expected_wifiSignalType, signal_type );
    TEST_ASSERT_EQUAL_UINT8( expected_wifiDataRate, datarate );
}

void test_lr11xx_wifi_get_nb_results_max_per_chunk( void )
{
    TEST_ASSERT_EQUAL_UINT8( 32, lr11xx_wifi_get_nb_results_max_per_chunk( ) );
}

void test_lr11xx_wifi_is_well_formed_utf8_byte_sequence( )
{
    const uint8_t well_formed_sequence_length_1[1]        = { 0x05 };
    const uint8_t well_formed_sequence_length_2[2]        = { 0xC2, 0xBF };
    const uint8_t well_formed_sequence_length_3[3]        = { 0xE0, 0xA0, 0x85 };
    const uint8_t well_formed_sequence_length_3_other[3]  = { 0xE1, 0x81, 0x81 };
    const uint8_t well_formed_sequence_length_3_other2[3] = { 0xED, 0x81, 0x81 };
    const uint8_t well_formed_sequence_length_3_other3[3] = { 0xEE, 0x81, 0x81 };
    const uint8_t well_formed_sequence_length_4[4]        = { 0xF4, 0x80, 0x83, 0x92 };
    const uint8_t well_formed_sequence_length_4_other[4]  = { 0xF0, 0x91, 0x81, 0x81 };
    const uint8_t well_formed_sequence_length_4_other2[4] = { 0xF2, 0x81, 0x81, 0x81 };
    const uint8_t ill_formed_sequence_wrong_byte[3]       = { 0xE0, 0x9F, 0x80 };
    const uint8_t ill_formed_sequence_wrong_length[2]     = { 0x01, 0xC3 };

    bool result;

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( well_formed_sequence_length_1, 1 );
    TEST_ASSERT_TRUE( result );

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( well_formed_sequence_length_2, 2 );
    TEST_ASSERT_TRUE( result );

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( well_formed_sequence_length_3, 3 );
    TEST_ASSERT_TRUE( result );

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( well_formed_sequence_length_3_other, 3 );
    TEST_ASSERT_TRUE( result );

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( well_formed_sequence_length_3_other2, 3 );
    TEST_ASSERT_TRUE( result );

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( well_formed_sequence_length_3_other3, 3 );
    TEST_ASSERT_TRUE( result );

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( well_formed_sequence_length_4, 4 );
    TEST_ASSERT_TRUE( result );

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( well_formed_sequence_length_4_other, 4 );
    TEST_ASSERT_TRUE( result );

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( well_formed_sequence_length_4_other2, 4 );
    TEST_ASSERT_TRUE( result );

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( ill_formed_sequence_wrong_byte, 3 );
    TEST_ASSERT_FALSE( result );

    result = lr11xx_wifi_is_well_formed_utf8_byte_sequence( ill_formed_sequence_wrong_length, 2 );
    TEST_ASSERT_FALSE( result );
}

TEST_VALUE( LR11XX_WIFI_SCAN_MODE_BEACON, LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE, 1 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_BEACON, LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL, 1 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT, LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE, 1 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT, LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL, 1 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_FULL_BEACON, LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL, 1 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_UNTIL_SSID, LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL, 1 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_BEACON, LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL, 0 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_BEACON_AND_PKT, LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL, 0 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_FULL_BEACON, LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE, 0 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_UNTIL_SSID, LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE, 0 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_FULL_BEACON, LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL, 0 )
TEST_VALUE( LR11XX_WIFI_SCAN_MODE_UNTIL_SSID, LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL, 0 )
void test_lr11xx_wifi_are_scan_mode_result_format_compatible( lr11xx_wifi_mode_t          scan_mode,
                                                              lr11xx_wifi_result_format_t result_format,
                                                              _Bool                       expected_is_compatible )
{
    const bool is_compatible = lr11xx_wifi_are_scan_mode_result_format_compatible( scan_mode, result_format );
    TEST_ASSERT( expected_is_compatible == is_compatible );
}