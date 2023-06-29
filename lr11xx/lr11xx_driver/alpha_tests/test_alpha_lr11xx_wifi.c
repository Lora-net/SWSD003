#include "lr11xx_wifi.h"
#include "lr11xx_wifi_alpha.h"
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
#define EXPECTED_WIFI_SEARCH_COUNTRY_CODE_TIME_LIMIT_CBUFFER_SIZE ( 9 )
#define EXPECTED_WIFI_CONFIGURE_DEBARKER_CBUFFER_SIZE ( 3 )

void* radio = 0;

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
void test_lr11xx_wifi_ConfigureHardwareDebarker( uint8_t hardware_debarker_enabled_uint )
{
    const bool    hardware_debarker_enabled = ( hardware_debarker_enabled_uint == 1 );
    const uint8_t cbuffer_expected[EXPECTED_WIFI_CONFIGURE_DEBARKER_CBUFFER_SIZE] = {
        0x03,
        0x04,
        ( hardware_debarker_enabled ) ? 1 : 0,
    };
    lr11xx_hal_write_ExpectWithArrayAndReturn(
        radio, 0, cbuffer_expected, EXPECTED_WIFI_CONFIGURE_DEBARKER_CBUFFER_SIZE,
        EXPECTED_WIFI_CONFIGURE_DEBARKER_CBUFFER_SIZE, NULL, 0, 0, LR11XX_HAL_STATUS_OK );
    lr11xx_wifi_cfg_hardware_debarker( radio, hardware_debarker_enabled );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_wifi_ConfigureHardwareDebarker_status( lr11xx_status_t     status_expected,
                                                        lr11xx_hal_status_t hal_status )
{
    const bool hardware_debarker_enabled = true;
    lr11xx_hal_write_IgnoreAndReturn( hal_status );
    const lr11xx_status_t status = lr11xx_wifi_cfg_hardware_debarker( radio, hardware_debarker_enabled );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_WIFI_SCAN_ALPHA_MODE_BEACON )
TEST_VALUE( LR11XX_WIFI_SCAN_ALPHA_MODE_BEACON_AND_PACKET )
TEST_VALUE( LR11XX_WIFI_SCAN_ALPHA_MODE_FULL_MAC )
void test_alpha_lr11xx_wifi_Scan( lr11xx_wifi_alpha_mode_t scan_mode )
{
    const lr11xx_wifi_signal_type_scan_t signal_type  = LR11XX_WIFI_TYPE_SCAN_B;
    const lr11xx_wifi_channel_mask_t     channel_mask = wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_1 ) |
                                                    wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_6 ) |
                                                    wifi_channel_value_to_mask( LR11XX_WIFI_CHANNEL_11 );
    const uint8_t  nb_max_results      = 16;
    const uint8_t  nb_scan_per_channel = 5;
    const uint16_t timeout_in_ms       = 101;
    const bool     abort_if_timeout    = true;

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
    lr11xx_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, EXPECTED_WIFI_SCAN_CBUFFER_SIZE,
                                               EXPECTED_WIFI_SCAN_CBUFFER_SIZE, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_wifi_scan( radio, signal_type, channel_mask, scan_mode, nb_max_results, nb_scan_per_channel, timeout_in_ms,
                      abort_if_timeout );
}

TEST_VALUE( LR11XX_WIFI_SCAN_ALPHA_MODE_FULL_MAC )
void test_alpha_lr11xx_wifi_ScanTimeLimit( lr11xx_wifi_alpha_mode_t scan_mode )
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
    lr11xx_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, EXPECTED_WIFI_SCAN_TIME_LIMIT_CBUFFER_SIZE,
                                               EXPECTED_WIFI_SCAN_TIME_LIMIT_CBUFFER_SIZE, NULL, 0, 0,
                                               LR11XX_HAL_STATUS_OK );

    lr11xx_wifi_scan_time_limit( radio, signal_type, channel_mask, scan_mode, nb_max_results, timeout_per_channel_ms,
                                 timeout_per_scan_ms );
}

void test_lr11xx_wifi_ReadBasicMiscResults( void )
{
    const uint16_t nb_results  = 3;
    const uint16_t result_size = nb_results * 12;

    const uint8_t cbuffer_expected[]      = { 0x03, 0x06, 0x00, nb_results, 0x00 };
    uint8_t       rbuffer_in_expected[36] = { 0x00 };

    const uint8_t                            macAddress_exp1[]  = { 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
    const uint8_t                            channel_info_mac1  = 0xAA;
    const lr11xx_wifi_signal_type_result_t   wifi_type_mac1     = LR11XX_WIFI_TYPE_RESULT_B;
    const uint8_t                            wifi_rssi_mac1     = 0xF0;
    const lr11xx_wifi_frame_type_info_byte_t frame_control_mac1 = 0xE5;
    const uint16_t                           phi_offset_mac1    = 0x1548;

    const uint8_t                            macAddress_exp2[]  = { 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13 };
    const uint8_t                            channel_info_mac2  = 0x0D;
    const lr11xx_wifi_signal_type_result_t   wifi_type_mac2     = LR11XX_WIFI_TYPE_RESULT_G;
    const uint8_t                            wifi_rssi_mac2     = 0x42;
    const lr11xx_wifi_frame_type_info_byte_t frame_control_mac2 = 0xA9;
    const uint16_t                           phi_offset_mac2    = 0x8569;

    const uint8_t                            macAddress_exp3[]  = { 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };
    const uint8_t                            channel_info_mac3  = 0x20;
    const lr11xx_wifi_signal_type_result_t   wifi_type_mac3     = LR11XX_WIFI_TYPE_RESULT_N;
    const uint8_t                            wifi_rssi_mac3     = 0x55;
    const lr11xx_wifi_frame_type_info_byte_t frame_control_mac3 = 0x00;
    const uint16_t                           phi_offset_mac3    = 0xFF06;

    uint8_t rbuffer_out_faked[] = {
        ( uint8_t ) wifi_type_mac1, channel_info_mac1,  wifi_rssi_mac1,           frame_control_mac1,
        macAddress_exp1[0],         macAddress_exp1[1], macAddress_exp1[2],       macAddress_exp1[3],
        macAddress_exp1[4],         macAddress_exp1[5], ( phi_offset_mac1 >> 8 ), ( phi_offset_mac1 & 0x00FF ),

        ( uint8_t ) wifi_type_mac2, channel_info_mac2,  wifi_rssi_mac2,           frame_control_mac2,
        macAddress_exp2[0],         macAddress_exp2[1], macAddress_exp2[2],       macAddress_exp2[3],
        macAddress_exp2[4],         macAddress_exp2[5], ( phi_offset_mac2 >> 8 ), ( phi_offset_mac2 & 0x00FF ),

        ( uint8_t ) wifi_type_mac3, channel_info_mac3,  wifi_rssi_mac3,           frame_control_mac3,
        macAddress_exp3[0],         macAddress_exp3[1], macAddress_exp3[2],       macAddress_exp3[3],
        macAddress_exp3[4],         macAddress_exp3[5], ( phi_offset_mac3 >> 8 ), ( phi_offset_mac3 & 0x00FF ),
    };

    lr11xx_wifi_basic_misc_result_t results[3] = { 0 };

    /********************/
    /* Set expectations */
    /********************/

    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 5, 5, rbuffer_in_expected, result_size,
                                              result_size, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, result_size );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_read_basic_misc_results( radio, 0, nb_results, results );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8( wifi_type_mac1, results[0].data_rate_info_byte );
    TEST_ASSERT_EQUAL_UINT8( channel_info_mac1, results[0].channel_info_byte );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp1, results[0].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac1, results[0].rssi );
    TEST_ASSERT_EQUAL_UINT8( frame_control_mac1, results[0].frame_type_info_byte );
    TEST_ASSERT_EQUAL_UINT16( phi_offset_mac1, results[0].phi_offset );

    TEST_ASSERT_EQUAL_UINT8( wifi_type_mac2, results[1].data_rate_info_byte );
    TEST_ASSERT_EQUAL_UINT8( channel_info_mac2, results[1].channel_info_byte );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp2, results[1].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac2, results[1].rssi );
    TEST_ASSERT_EQUAL_UINT8( frame_control_mac2, results[1].frame_type_info_byte );
    TEST_ASSERT_EQUAL_UINT16( phi_offset_mac2, results[1].phi_offset );

    TEST_ASSERT_EQUAL_UINT8( wifi_type_mac3, results[2].data_rate_info_byte );
    TEST_ASSERT_EQUAL_UINT8( channel_info_mac3, results[2].channel_info_byte );
    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp3, results[2].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac3, results[2].rssi );
    TEST_ASSERT_EQUAL_UINT8( frame_control_mac3, results[2].frame_type_info_byte );
    TEST_ASSERT_EQUAL_UINT16( phi_offset_mac3, results[2].phi_offset );
}

void test_lr11xx_wifi_FullResultLoop( void )
{
    const uint16_t nb_results  = 13;
    const uint16_t result_size = nb_results * 76;

    const uint8_t cbuffer_expected[] = { 0x03, 0x06, 0x00, nb_results, 0x00 };
    uint8_t       rbuffer_in_expected[result_size];
    memset( rbuffer_in_expected, 0, result_size );

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 5, 5, rbuffer_in_expected, result_size,
                                              result_size, LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_extended_misc_result_t results[13];
    lr11xx_wifi_read_extended_misc_results( radio, 0, nb_results, results );
}

void test_lr11xx_wifi_FullResultDualLoop( void )
{
    const uint16_t nb_results  = 14;
    const uint16_t result_size = nb_results * 76;

    const uint8_t cbuffer_expected_max_size[] = { 0x03, 0x06, 0x00, 13, 0x00 };
    const uint8_t cbuffer_expected[]          = { 0x03, 0x06, 13, 1, 0x00 };
    uint8_t       rbuffer_in_expected[result_size];
    memset( rbuffer_in_expected, 0, result_size );

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_max_size, 5, 5, rbuffer_in_expected, 988, 988,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 5, 5, rbuffer_in_expected, 76, 76,
                                              LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_extended_misc_result_t results[14];
    lr11xx_wifi_read_extended_misc_results( radio, 0, nb_results, results );
}

void test_lr11xx_wifi_FullResultTripleLoop( void )
{
    const uint16_t nb_results  = 32;
    const uint16_t result_size = nb_results * 76;

    const uint8_t cbuffer_expected_max1[]      = { 0x03, 0x06, 0, 13, 0x00 };
    const uint8_t cbuffer_expected_max2[]      = { 0x03, 0x06, 13, 13, 0x00 };
    const uint8_t cbuffer_expected_remaining[] = { 0x03, 0x06, 26, 6, 0x00 };
    uint8_t       rbuffer_in_expected[result_size];
    memset( rbuffer_in_expected, 0, result_size );

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_max1, 5, 5, rbuffer_in_expected, 988, 988,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_max2, 5, 5, rbuffer_in_expected, 988, 988,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected_remaining, 5, 5, rbuffer_in_expected, 456, 456,
                                              LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_extended_misc_result_t results[32];
    lr11xx_wifi_read_extended_misc_results( radio, 0, nb_results, results );
}

void test_lr11xx_wifi_ReadBasicMacRssiResults( void )
{
    const uint16_t nb_results  = 3;
    const uint16_t result_size = nb_results * 7;

    const uint8_t cbuffer_expected[]      = { 0x03, 0x06, 0x00, nb_results, 0x03 };
    uint8_t       rbuffer_in_expected[21] = { 0x00 };

    const uint8_t macAddress_exp1[] = { 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
    const uint8_t wifi_rssi_mac1    = 0xF0;

    const uint8_t macAddress_exp2[] = { 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13 };
    const uint8_t wifi_rssi_mac2    = 0x42;

    const uint8_t macAddress_exp3[] = { 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };
    const uint8_t wifi_rssi_mac3    = 0x55;

    uint8_t rbuffer_out_faked[] = {
        wifi_rssi_mac1,     macAddress_exp1[0], macAddress_exp1[1], macAddress_exp1[2],
        macAddress_exp1[3], macAddress_exp1[4], macAddress_exp1[5],

        wifi_rssi_mac2,     macAddress_exp2[0], macAddress_exp2[1], macAddress_exp2[2],
        macAddress_exp2[3], macAddress_exp2[4], macAddress_exp2[5],

        wifi_rssi_mac3,     macAddress_exp3[0], macAddress_exp3[1], macAddress_exp3[2],
        macAddress_exp3[3], macAddress_exp3[4], macAddress_exp3[5],
    };

    lr11xx_wifi_basic_mac_rssi_result_t results[3] = { 0 };

    /********************/
    /* Set expectations */
    /********************/

    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 5, 5, rbuffer_in_expected, result_size,
                                              result_size, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, result_size );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_wifi_read_basic_mac_rssi_results( radio, 0, nb_results, results );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp1, results[0].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac1, results[0].rssi );

    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp2, results[1].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac2, results[1].rssi );

    TEST_ASSERT_EQUAL_UINT8_ARRAY( macAddress_exp3, results[2].mac_address, 6 );
    TEST_ASSERT_EQUAL_UINT8( wifi_rssi_mac3, results[2].rssi );
}