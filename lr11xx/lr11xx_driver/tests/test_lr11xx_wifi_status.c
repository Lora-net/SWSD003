#include "lr11xx_wifi.h"
#include "unity.h"

#include "mock_lr11xx_hal.h"

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

void* context;

void setUp( void )
{
}

void tearDown( void )
{
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_wifi_scan_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    const lr11xx_wifi_signal_type_scan_t signal_type         = LR11XX_WIFI_TYPE_SCAN_B;
    const lr11xx_wifi_channel_mask_t     channel_mask        = 0x0421;
    const lr11xx_wifi_mode_t             scan_mode           = LR11XX_WIFI_SCAN_MODE_BEACON;
    const uint8_t                        nb_max_results      = 16;
    const uint8_t                        nb_scan_per_channel = 5;
    const uint16_t                       timeout_in_ms       = 101;
    const bool                           abort_if_timeout    = true;

    lr11xx_hal_write_IgnoreAndReturn( hal_status );

    const lr11xx_status_t status = lr11xx_wifi_scan( context, signal_type, channel_mask, scan_mode, nb_max_results,
                                                     nb_scan_per_channel, timeout_in_ms, abort_if_timeout );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_wifi_scan_time_limit_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    const lr11xx_wifi_signal_type_scan_t signal_type               = LR11XX_WIFI_TYPE_SCAN_B;
    const lr11xx_wifi_channel_mask_t     channel_mask              = 0x0421;
    const lr11xx_wifi_mode_t             scan_mode                 = LR11XX_WIFI_SCAN_MODE_BEACON;
    const uint8_t                        nb_max_results            = 16;
    const uint16_t                       timeout_per_channel_in_ms = 500;
    const uint16_t                       timeout_per_scan_in_ms    = 110;

    lr11xx_hal_write_IgnoreAndReturn( hal_status );

    const lr11xx_status_t status =
        lr11xx_wifi_scan_time_limit( context, signal_type, channel_mask, scan_mode, nb_max_results,
                                     timeout_per_channel_in_ms, timeout_per_scan_in_ms );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_wifi_SearchCountryCode_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    const lr11xx_wifi_channel_mask_t channels_mask        = 0x03FFF;
    const uint16_t                   timeout_in_ms        = 110;
    const uint8_t                    nb_max_results       = 32;
    const uint8_t                    nb_scan_per_channels = 255;
    const bool                       abort_if_timeout     = true;

    lr11xx_hal_write_IgnoreAndReturn( hal_status );

    const lr11xx_status_t status = lr11xx_wifi_search_country_code(
        context, channels_mask, nb_max_results, nb_scan_per_channels, timeout_in_ms, abort_if_timeout );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_wifi_GetSizeResults_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    uint8_t               nb_results = 0x00;
    const lr11xx_status_t status     = lr11xx_wifi_get_nb_results( context, &nb_results );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_wifi_ReadBasicMacTypeChannelRssiResults_status( lr11xx_status_t     status_expected,
                                                                 lr11xx_hal_status_t hal_status )
{
    lr11xx_wifi_basic_mac_type_channel_result_t results = { 0 };
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    const lr11xx_status_t status = lr11xx_wifi_read_basic_mac_type_channel_results( context, 0, 1, &results );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_wifi_ResetCumulativeTimings_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_write_IgnoreAndReturn( hal_status );
    const lr11xx_status_t status = lr11xx_wifi_reset_cumulative_timing( context );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_wifi_ReadCumulTimings_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_wifi_cumulative_timings_t timings = { 0 };
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    const lr11xx_status_t status = lr11xx_wifi_read_cumulative_timing( context, &timings );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_wifi_GetSizeCountryResults_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    uint8_t               nb_country_code_results = 0;
    const lr11xx_status_t status = lr11xx_wifi_get_nb_country_code_results( context, &nb_country_code_results );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_wifi_readCountryCodeResult_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    lr11xx_wifi_country_code_t country_code = { 0 };
    const lr11xx_status_t      status       = lr11xx_wifi_read_country_code_results( context, 0, 1, &country_code );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_wifiReadVersion_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_hal_read_IgnoreAndReturn( hal_status );
    lr11xx_wifi_version_t version = { 0 };
    const lr11xx_status_t status  = lr11xx_wifi_read_version( context, &version );
    TEST_ASSERT_EQUAL_INT( status_expected, status );
}