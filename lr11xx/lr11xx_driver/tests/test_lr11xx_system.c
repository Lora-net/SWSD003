#include "unity.h"
#include "lr11xx_system.h"

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
void test_lr11xx_system_reset( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_status_t status;

    lr11xx_hal_reset_ExpectAndReturn( context, hal_status );

    status = lr11xx_system_reset( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_get_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_out_faked[] = { 0x05, 0x17, 0x01, 0x23, 0x45, 0x67 };

    const uint8_t rbuffer_in_expected[6] = { 0 };

    lr11xx_system_stat1_t stat1;
    lr11xx_system_stat2_t stat2;
    uint32_t              irq = 0;
    lr11xx_status_t       status;

    lr11xx_hal_direct_read_ExpectAndReturn( context, NULL, 6, hal_status );
    lr11xx_hal_direct_read_IgnoreArg_data( );
    lr11xx_hal_direct_read_ReturnArrayThruPtr_data( cbuffer_out_faked, 6 );

    status = lr11xx_system_get_status( context, &stat1, &stat2, &irq );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( LR11XX_SYSTEM_CMD_STATUS_OK, stat1.command_status );
        TEST_ASSERT_TRUE( stat1.is_interrupt_active );
        TEST_ASSERT_EQUAL_UINT8( LR11XX_SYSTEM_CHIP_MODE_FS, stat2.chip_mode );
        TEST_ASSERT_EQUAL_UINT8( LR11XX_SYSTEM_RESET_STATUS_ANALOG, stat2.reset_status );
        TEST_ASSERT_TRUE( stat2.is_running_from_flash );
        TEST_ASSERT_EQUAL_UINT32( 0x01234567, irq );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_get_irq_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t rbuffer_out_faked[] = { 0x00, 0x00, 0x23, 0x45, 0x67, 0x89 };

    uint32_t        irq = 0;
    lr11xx_status_t status;

    lr11xx_hal_direct_read_ExpectAndReturn( context, NULL, 6, hal_status );
    lr11xx_hal_direct_read_IgnoreArg_data( );
    lr11xx_hal_direct_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 6 );

    status = lr11xx_system_get_irq_status( context, &irq );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT32( 0x23456789, irq );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_get_version( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x01, 0x01 };
    uint8_t rbuffer_in_expected[4] = { 0x00 };
    uint8_t rbuffer_out_faked[]    = { 0x01, 0x23, 0x45, 0x67 };

    lr11xx_system_version_t version;
    lr11xx_status_t         status;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 4, 4,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 4 );

    status = lr11xx_system_get_version( context, &version );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( 0x01, version.hw );
        TEST_ASSERT_EQUAL_UINT16( 0x4567, version.fw );
        TEST_ASSERT_EQUAL_UINT8( 0x23, version.type );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_get_errors( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x01, 0x0D };
    uint8_t rbuffer_in_expected[2] = { 0x00 };
    uint8_t rbuffer_out_faked[]    = { 0x00, 0xA5 };

    lr11xx_system_errors_t errors     = 0x0000;
    lr11xx_system_errors_t errors_exp = LR11XX_SYSTEM_ERRORS_LF_RC_CALIB_MASK | LR11XX_SYSTEM_ERRORS_ADC_CALIB_MASK |
                                        LR11XX_SYSTEM_ERRORS_PLL_LOCK_MASK | LR11XX_SYSTEM_ERRORS_HF_XOSC_START_MASK;
    lr11xx_status_t status;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 2, 2,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 2 );

    status = lr11xx_system_get_errors( context, &errors );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT16( errors_exp, errors );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_clear_errors( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x0E };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, NULL, 0, 0, hal_status );

    status = lr11xx_system_clear_errors( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_calibrate( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x0F, 0x2A };

    lr11xx_status_t status;
    uint8_t         calib_param =
        LR11XX_SYSTEM_CALIB_HF_RC_MASK | LR11XX_SYSTEM_CALIB_ADC_MASK | LR11XX_SYSTEM_CALIB_PLL_TX_MASK;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    status = lr11xx_system_calibrate( context, calib_param );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_set_regmode( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x10, 0x01 };

    lr11xx_status_t          status;
    lr11xx_system_reg_mode_t reg_mode = LR11XX_SYSTEM_REG_MODE_DCDC;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    status = lr11xx_system_set_reg_mode( context, reg_mode );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_calibrate_image( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x11, 0x01, 0x23 };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 4, 4, NULL, 0, 0, hal_status );

    status = lr11xx_system_calibrate_image( context, 0x01, 0x23 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 900, 900, 0xE1, 0xE1 )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 899, 901, 0xE0, 0xE2 )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, 900, 900, 0xE1, 0xE1 )
void test_lr11xx_system_calibrate_image_in_mhz( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                                uint16_t freq1_in_mhz, uint16_t freq2_in_mhz, uint8_t freq1_expected,
                                                uint8_t freq2_expected )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x11, freq1_expected, freq2_expected };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 4, 4, NULL, 0, 0, hal_status );

    status = lr11xx_system_calibrate_image_in_mhz( context, freq1_in_mhz, freq2_in_mhz );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_set_dio_as_rf_switch( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x12, 0x1F, 0x04, 0x01, 0x02, 0x04, 0x08, 0x10, 0x09 };

    lr11xx_status_t              status;
    lr11xx_system_rfswitch_cfg_t rfSwitchConfig = {
        .enable = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH | LR11XX_SYSTEM_RFSW2_HIGH |
                  LR11XX_SYSTEM_RFSW3_HIGH | LR11XX_SYSTEM_RFSW4_HIGH,
        .standby = LR11XX_SYSTEM_RFSW2_HIGH,
        .rx      = LR11XX_SYSTEM_RFSW0_HIGH,
        .tx      = LR11XX_SYSTEM_RFSW1_HIGH,
        .tx_hp   = LR11XX_SYSTEM_RFSW2_HIGH,
        .tx_hf   = LR11XX_SYSTEM_RFSW3_HIGH,
        .gnss    = LR11XX_SYSTEM_RFSW4_HIGH,
        .wifi    = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW3_HIGH,
    };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 10, 10, NULL, 0, 0, hal_status );

    status = lr11xx_system_set_dio_as_rf_switch( context, &rfSwitchConfig );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_set_dio_irq_params( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x13, 0x00, 0x00, 0x00, 0x8C, 0x00, 0x58, 0x00, 0x00 };

    lr11xx_status_t status;
    uint32_t irqs_to_enable_dio1 = LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR;
    uint32_t irqs_to_enable_dio2 =
        LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE | LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE | LR11XX_SYSTEM_IRQ_CMD_ERROR;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 10, 10, NULL, 0, 0, hal_status );

    status = lr11xx_system_set_dio_irq_params( context, irqs_to_enable_dio1, irqs_to_enable_dio2 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_clear_irq( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x14, 0x00, 0x58, 0x00, 0x8C };

    lr11xx_status_t status;
    uint32_t irqs_to_clear = LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR |
                             LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE | LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE |
                             LR11XX_SYSTEM_IRQ_CMD_ERROR;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, NULL, 0, 0, hal_status );

    status = lr11xx_system_clear_irq_status( context, irqs_to_clear );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_get_and_clear_irq_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status_1,
                                                  lr11xx_hal_status_t hal_status_2 )
{
    uint8_t rbuffer_out_faked_1[] = { 0x00, 0x00, 0x23, 0x45, 0x67, 0x89 };
    uint8_t cbuffer_expected_2[]  = { 0x01, 0x14, 0x23, 0x45, 0x67, 0x89 };

    uint32_t        irq = 0;
    lr11xx_status_t status;

    lr11xx_hal_direct_read_ExpectAndReturn( context, NULL, 6, hal_status_1 );
    lr11xx_hal_direct_read_IgnoreArg_data( );
    lr11xx_hal_direct_read_ReturnArrayThruPtr_data( rbuffer_out_faked_1, 6 );

    if( hal_status_1 == LR11XX_HAL_STATUS_OK )
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_2, 6, 6, NULL, 0, 0, hal_status_2 );
    }

    status = lr11xx_system_get_and_clear_irq_status( context, &irq );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT32( 0x23456789, irq );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_config_lfclk( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x16, 0x05 };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    status = lr11xx_system_cfg_lfclk( context, LR11XX_SYSTEM_LFCLK_XTAL, true );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_set_tcxo_mode( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x17, 0x03, 0x23, 0x45, 0x67 };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, NULL, 0, 0, hal_status );

    status = lr11xx_system_set_tcxo_mode( context, LR11XX_SYSTEM_TCXO_CTRL_2_2V, 0x01234567 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 0, 0 )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, 1, 3 )
void test_lr11xx_system_reboot( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                uint8_t stay_in_bootloader, uint8_t expected_value )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x18, expected_value };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    status = lr11xx_system_reboot( context, ( stay_in_bootloader == 0 ) ? false : true );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_get_vbat( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x01, 0x19 };
    uint8_t rbuffer_in_expected[1] = { 0x00 };
    uint8_t rbuffer_out_faked[1]   = { 0x5A };

    lr11xx_status_t status;
    uint8_t         vbat     = 0x00;
    uint8_t         vbat_exp = 0x5A;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    status = lr11xx_system_get_vbat( context, &vbat );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( vbat_exp, vbat );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_get_temp( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x01, 0x1A };
    uint8_t rbuffer_in_expected[2] = { 0x00 };
    uint8_t rbuffer_out_faked[2]   = { 0x01, 0x23 };

    lr11xx_status_t status;
    uint16_t        temp     = 0x00;
    uint16_t        temp_exp = 0x0123;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 2, 2,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 2 );

    status = lr11xx_system_get_temp( context, &temp );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT16( temp_exp, temp );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_set_sleep( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x1B, 0x02, 0x01, 0x23, 0x45, 0x67 };

    lr11xx_status_t           status;
    lr11xx_system_sleep_cfg_t sleep_cfg;

    sleep_cfg.is_warm_start  = false;
    sleep_cfg.is_rtc_timeout = true;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 7, 7, NULL, 0, 0, hal_status );

    status = lr11xx_system_set_sleep( context, sleep_cfg, 0x01234567 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_set_standby( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x1C, 0x01 };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    status = lr11xx_system_set_standby( context, LR11XX_SYSTEM_STANDBY_CFG_XOSC );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_wakeup( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_status_t status;

    lr11xx_hal_wakeup_ExpectAndReturn( context, hal_status );

    status = lr11xx_system_wakeup( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_set_fs( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x1D };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, NULL, 0, 0, hal_status );

    status = lr11xx_system_set_fs( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_erase_infopage( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x21, 0x01 };

    lr11xx_status_t             status;
    lr11xx_system_infopage_id_t infopage_id = LR11XX_SYSTEM_INFOPAGE_1;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    status = lr11xx_system_erase_infopage( context, infopage_id );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_write_infopage( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x22, 0x01, 0x01, 0x23 };
    uint8_t cdata_expected[]   = { 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67 };

    lr11xx_status_t             status;
    uint32_t                    data[]   = { 0x89ABCDEF, 0x01234567 };
    lr11xx_system_infopage_id_t infopage = LR11XX_SYSTEM_INFOPAGE_1;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 5, 5, cdata_expected, 8, 8, hal_status );

    status = lr11xx_system_write_infopage( context, infopage, 0x0123, data, 2 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_read_infopage( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]      = { 0x01, 0x23, 0x01, 0xCD, 0xEF, 0x03 };
    uint8_t rbuffer_in_expected[12] = { 0x00 };
    uint8_t rbuffer_out_faked[12]   = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C };

    lr11xx_status_t             status;
    uint32_t                    data[3]     = { 0 };
    uint32_t                    data_exp[3] = { 0x01020304, 0x05060708, 0x090A0B0C };
    lr11xx_system_infopage_id_t infopage    = LR11XX_SYSTEM_INFOPAGE_1;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, rbuffer_in_expected, 12, 12,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 12 );

    status = lr11xx_system_read_infopage( context, infopage, 0xCDEF, data, 3 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT32_ARRAY( data_exp, data, 3 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_read_uid( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t             cbuffer_expected[]     = { 0x01, 0x25 };
    lr11xx_system_uid_t unique_id_expected     = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
    uint8_t             rbuffer_in_expected[8] = { 0x00 };
    uint8_t             rbuffer_out_faked[8]   = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 8, 8,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 8 );

    lr11xx_status_t     status;
    lr11xx_system_uid_t unique_id = { 0 };

    status = lr11xx_system_read_uid( context, unique_id );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( unique_id_expected, unique_id, 8 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_read_join_eui( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t             cbuffer_expected[]     = { 0x01, 0x26 };
    lr11xx_system_uid_t join_eui_expected      = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
    uint8_t             rbuffer_in_expected[8] = { 0x00 };
    uint8_t             rbuffer_out_faked[8]   = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 8, 8,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 8 );

    lr11xx_status_t          status;
    lr11xx_system_join_eui_t join_eui = { 0 };

    status = lr11xx_system_read_join_eui( context, join_eui );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( join_eui_expected, join_eui, 8 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_read_pin( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t             cbuffer_expected[]     = { 0x01, 0x27 };
    lr11xx_system_pin_t pin_expected           = { 0x01, 0x23, 0x45, 0x67 };
    uint8_t             rbuffer_in_expected[4] = { 0x00 };
    uint8_t             rbuffer_out_faked[4]   = { 0x01, 0x23, 0x45, 0x67 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 4, 4,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 4 );

    lr11xx_status_t     status;
    lr11xx_system_pin_t pin = { 0 };

    status = lr11xx_system_read_pin( context, pin );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( pin_expected, pin, 4 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_read_pin_custom_eui( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t             cbuffer_expected[]     = { 0x01, 0x27, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
                                                   0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10, 0x00 };
    lr11xx_system_pin_t pin_expected           = { 0x01, 0x23, 0x45, 0x67 };
    uint8_t             rbuffer_in_expected[4] = { 0x00 };
    uint8_t             rbuffer_out_faked[4]   = { 0x01, 0x23, 0x45, 0x67 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 19, 19, rbuffer_in_expected, 4, 4,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 4 );

    lr11xx_status_t          status;
    lr11xx_system_pin_t      pin        = { 0 };
    lr11xx_system_uid_t      device_eui = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
    lr11xx_system_join_eui_t join_eui   = { 0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10 };

    status = lr11xx_system_read_pin_custom_eui( context, device_eui, join_eui, 0x00, pin );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( pin_expected, pin, 4 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_system_get_random( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x01, 0x20 };
    uint8_t rbuffer_in_expected[4] = { 0x00 };
    uint8_t rbuffer_out_faked[4]   = { 0x01, 0x23, 0x45, 0x67 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 4, 4,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 4 );

    lr11xx_status_t status;
    uint32_t        random_number = 0;

    status = lr11xx_system_get_random_number( context, &random_number );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT32( 0x67452301, random_number );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 0x00 )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, 0x01 )
void test_lr11xx_system_enable_spi_crc( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                        uint8_t enable_crc_uint )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x28, enable_crc_uint };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    status = lr11xx_system_enable_spi_crc( context, ( enable_crc_uint == 0 ) ? false : true );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 0x00 )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, 0x01 )
void test_lr11xx_system_drive_dio_in_sleep_mode( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                                 uint8_t enable_drive_uint )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x2A, enable_drive_uint };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    status = lr11xx_system_drive_dio_in_sleep_mode( context, ( enable_drive_uint == 0 ) ? false : true );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

void test_lr11xx_system_clear_reset_status_info( void )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x00 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_status_t status;
    status = lr11xx_system_clear_reset_status_info( context );

    TEST_ASSERT_EQUAL_INT( LR11XX_STATUS_OK, status );
}