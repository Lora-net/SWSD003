#include "lr11xx_bootloader_alpha.h"
#include "unity.h"

#include <string.h>  // for memset

#include "mock_lr11xx_hal.h"

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

#define FLASH_SIZE 140
#define WORD_NUM 64
#define FLASH_BYTES_PER_SPI ( WORD_NUM * sizeof( uint32_t ) )

void* context = 0;

void setUp( void )
{
}

void tearDown( void )
{
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_get_hash( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]      = { 0x80, 0x04 };
    uint8_t rbuffer_in_expected[16] = { 0x00 };
    uint8_t rbuffer_out_faked[16]   = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                      0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10 };

    lr11xx_bootloader_hash_t hash = { 0x00 };
    lr11xx_status_t          status;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 16, 16,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 16 );

    status = lr11xx_bootloader_get_hash( context, hash );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( rbuffer_out_faked, hash, 16 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_erase_page( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t         cbuffer_expected[] = { 0x80, 0x01, 0x08 };
    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    status = lr11xx_bootloader_erase_page( context, 0x08 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_write_flash( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t        cbuffer_expected[] = { 0x80, 0x02, 0x89, 0xAB, 0xCD, 0xEF };
    const uint8_t  cdata_expected[]   = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78 };
    const uint32_t data[]             = { 0x01234567, 0x89ABCDEF, 0x12345678 };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, cdata_expected, 12, 12, hal_status );

    status = lr11xx_bootloader_write_flash( context, 0x89ABCDEF, data, 3 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_write_flash_full( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status_1,
                                   lr11xx_hal_status_t hal_status_2, lr11xx_hal_status_t hal_status_3 )
{
    uint8_t  cbuffer_1_exp[] = { 0x80, 0x02, 0x01, 0x23, 0x45, 0x67 };
    uint8_t  cdata_1_exp[FLASH_BYTES_PER_SPI];
    uint8_t  cbuffer_2_exp[] = { 0x80, 0x02, 0x01, 0x23, 0x46, 0x67 };
    uint8_t  cdata_2_exp[FLASH_BYTES_PER_SPI];
    uint8_t  cbuffer_3_exp[] = { 0x80, 0x02, 0x01, 0x23, 0x47, 0x67 };
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

    status = lr11xx_bootloader_write_flash_full( context, 0x01234567, flash, FLASH_SIZE );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}
