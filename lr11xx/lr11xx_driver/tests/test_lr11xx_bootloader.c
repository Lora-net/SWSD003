#include "unity.h"
#include "lr11xx_bootloader.h"

#include "mock_lr11xx_hal.h"

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

void* context;

#define FLASH_SIZE 140
#define WORD_NUM 64
#define FLASH_BYTES_PER_SPI ( WORD_NUM * sizeof( uint32_t ) )

void setUp( void )
{
}

void tearDown( void )
{
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_bootloader_get_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t cbuffer_out_faked[] = { 0x05, 0x17, 0x01, 0x23, 0x45, 0x67 };

    lr11xx_bootloader_stat1_t stat1;
    lr11xx_bootloader_stat2_t stat2;
    uint32_t                  irq = 0;
    lr11xx_status_t           status;

    lr11xx_hal_direct_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, hal_status );
    lr11xx_hal_direct_read_IgnoreArg_data( );
    lr11xx_hal_direct_read_ReturnArrayThruPtr_data( cbuffer_out_faked, 6 );

    status = lr11xx_bootloader_get_status( context, &stat1, &stat2, &irq );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( LR11XX_BOOTLOADER_CMD_STATUS_OK, stat1.command_status );
        TEST_ASSERT_TRUE( stat1.is_interrupt_active );
        TEST_ASSERT_EQUAL_UINT8( LR11XX_BOOTLOADER_CHIP_MODE_FS, stat2.chip_mode );
        TEST_ASSERT_EQUAL_UINT8( LR11XX_BOOTLOADER_RESET_STATUS_ANALOG, stat2.reset_status );
        TEST_ASSERT_TRUE( stat2.is_running_from_flash );
        TEST_ASSERT_EQUAL_UINT32( 0x01234567, irq );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_bootloader_clear_reset_status_info( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t         cbuffer_expected[] = { 0x01, 0x00 };
    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, NULL, 0, 0, hal_status );

    status = lr11xx_bootloader_clear_reset_status_info( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_get_version( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]    = { 0x01, 0x01 };
    uint8_t rbuffer_in_expected[] = { 0x00, 0x00, 0x00, 0x00 };
    uint8_t rbuffer_out_faked[]   = { 0x01, 0x23, 0x45, 0x67 };

    lr11xx_bootloader_version_t version = { 0 };
    lr11xx_status_t             status;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 4, 4,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 4 );

    status = lr11xx_bootloader_get_version( context, &version );

    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( 0x01, version.hw );
        TEST_ASSERT_EQUAL_UINT8( 0x23, version.type );
        TEST_ASSERT_EQUAL_UINT16( 0x4567, version.fw );
    }

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_erase_flash( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t         cbuffer_expected[] = { 0x80, 0x00 };
    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, NULL, 0, 0, hal_status );

    status = lr11xx_bootloader_erase_flash( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_write_flash_encrypted( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t        cbuffer_expected[] = { 0x80, 0x03, 0x89, 0xAB, 0xCD, 0xEF };
    const uint8_t  cdata_expected[]   = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78 };
    const uint32_t data[]             = { 0x01234567, 0x89ABCDEF, 0x12345678 };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, cdata_expected, 12, 12, hal_status );

    status = lr11xx_bootloader_write_flash_encrypted( context, 0x89ABCDEF, data, 3 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_write_flash_encrypted_all( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status_1,
                                            lr11xx_hal_status_t hal_status_2, lr11xx_hal_status_t hal_status_3 )
{
    uint8_t  cbuffer_1_exp[] = { 0x80, 0x03, 0x01, 0x23, 0x45, 0x67 };
    uint8_t  cdata_1_exp[FLASH_BYTES_PER_SPI];
    uint8_t  cbuffer_2_exp[] = { 0x80, 0x03, 0x01, 0x23, 0x46, 0x67 };
    uint8_t  cdata_2_exp[FLASH_BYTES_PER_SPI];
    uint8_t  cbuffer_3_exp[] = { 0x80, 0x03, 0x01, 0x23, 0x47, 0x67 };
    uint8_t  cdata_3_exp[( FLASH_SIZE % WORD_NUM ) * 4];
    uint32_t flash[FLASH_SIZE];

    lr11xx_status_t status;

    for( uint32_t index = 0; index < FLASH_SIZE; index++ )
    {
        flash[index] = index;
    }

    for( uint32_t i = 0; i < WORD_NUM; i++ )
    {
        cdata_1_exp[i * 4 + 0] = ( uint8_t ) ( flash[i] >> 24 );
        cdata_1_exp[i * 4 + 1] = ( uint8_t ) ( flash[i] >> 16 );
        cdata_1_exp[i * 4 + 2] = ( uint8_t ) ( flash[i] >> 8 );
        cdata_1_exp[i * 4 + 3] = ( uint8_t ) ( flash[i] >> 0 );
    }

    for( uint32_t i = 0; i < WORD_NUM; i++ )
    {
        cdata_2_exp[i * 4 + 0] = ( uint8_t ) ( flash[WORD_NUM + i] >> 24 );
        cdata_2_exp[i * 4 + 1] = ( uint8_t ) ( flash[WORD_NUM + i] >> 16 );
        cdata_2_exp[i * 4 + 2] = ( uint8_t ) ( flash[WORD_NUM + i] >> 8 );
        cdata_2_exp[i * 4 + 3] = ( uint8_t ) ( flash[WORD_NUM + i] >> 0 );
    }

    for( uint32_t i = 0; i < ( FLASH_SIZE % WORD_NUM ); i++ )
    {
        cdata_3_exp[i * 4 + 0] = ( uint8_t ) ( flash[2 * WORD_NUM + i] >> 24 );
        cdata_3_exp[i * 4 + 1] = ( uint8_t ) ( flash[2 * WORD_NUM + i] >> 16 );
        cdata_3_exp[i * 4 + 2] = ( uint8_t ) ( flash[2 * WORD_NUM + i] >> 8 );
        cdata_3_exp[i * 4 + 3] = ( uint8_t ) ( flash[2 * WORD_NUM + i] >> 0 );
    }

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_1_exp, 6, 6, cdata_1_exp, FLASH_BYTES_PER_SPI,
                                               FLASH_BYTES_PER_SPI, hal_status_1 );

    if( hal_status_1 == LR11XX_HAL_STATUS_OK )
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_2_exp, 6, 6, cdata_2_exp, FLASH_BYTES_PER_SPI,
                                                   FLASH_BYTES_PER_SPI, hal_status_2 );

        if( hal_status_2 == LR11XX_HAL_STATUS_OK )
        {
            lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_3_exp, 6, 6, cdata_3_exp,
                                                       ( FLASH_SIZE % WORD_NUM ) * 4, ( FLASH_SIZE % WORD_NUM ) * 4,
                                                       hal_status_3 );
        }
    }

    status = lr11xx_bootloader_write_flash_encrypted_full( context, 0x01234567, flash, FLASH_SIZE );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 0, 0 )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, 1, 3 )
void test_lr11xx_reboot( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status, uint8_t stay_in_bootloader,
                         uint8_t expected_value )
{
    uint8_t cbuffer_expected[] = { 0x80, 0x05, expected_value };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    status = lr11xx_bootloader_reboot( context, ( stay_in_bootloader == 0 ) ? false : true );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_bootloader_read_pin( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t                 cbuffer_expected[]     = { 0x80, 0x0B };
    lr11xx_bootloader_pin_t pin_expected           = { 0x01, 0x23, 0x45, 0x67 };
    uint8_t                 rbuffer_in_expected[4] = { 0x00 };
    uint8_t                 rbuffer_out_faked[4]   = { 0x01, 0x23, 0x45, 0x67 };

    lr11xx_status_t status;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 4, 4,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 4 );

    lr11xx_bootloader_pin_t pin = { 0 };

    status = lr11xx_bootloader_read_pin( context, pin );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( pin_expected, pin, 4 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_read_chip_eui( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x80, 0x0C };
    uint8_t chip_eui_expected[]    = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
    uint8_t rbuffer_in_expected[8] = { 0x00 };
    uint8_t rbuffer_out_faked[8]   = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 8, 8,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 8 );

    lr11xx_status_t              status;
    lr11xx_bootloader_chip_eui_t chip_eui = { 0 };

    status = lr11xx_bootloader_read_chip_eui( context, chip_eui );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( chip_eui_expected, chip_eui, 8 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_read_join_eui( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x80, 0x0D };
    uint8_t join_eui_expected[]    = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
    uint8_t rbuffer_in_expected[8] = { 0x00 };
    uint8_t rbuffer_out_faked[8]   = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 8, 8,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 8 );

    lr11xx_status_t              status;
    lr11xx_bootloader_join_eui_t join_eui = { 0 };

    status = lr11xx_bootloader_read_join_eui( context, join_eui );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( join_eui_expected, join_eui, 8 );
    }
}
