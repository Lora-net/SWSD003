#include "unity.h"
#include "lr11xx_regmem_alpha.h"

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
void test_lr11xx_regmem_write_auxreg32( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x03, 0x01, 0x23, 0x45, 0x67 };
    uint8_t cdata_expected[]   = { 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0xFE, 0xDC, 0xBA, 0x98 };

    uint32_t        address = 0x01234567;
    uint32_t        data[3] = { 0x89ABCDEF, 0x01234567, 0xFEDCBA98 };
    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, cdata_expected, 12, 12, hal_status );

    status = lr11xx_regmem_write_auxreg32( context, address, data, 3 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_regmem_read_auxreg32( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x01, 0x04, 0x12, 0x34, 0x56, 0x78, 0x02 };
    uint8_t rbuffer_in_expected[8] = { 0x00 };
    uint8_t rbuffer_out_faked[8]   = { 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C };

    uint32_t        address                = 0x12345678;
    uint32_t        buffer_out[2]          = { 0 };
    uint32_t        buffer_out_expected[2] = { 0x05060708, 0x090A0B0C };
    lr11xx_status_t status;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 7, 7, rbuffer_in_expected, 8, 8,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 8 );

    status = lr11xx_regmem_read_auxreg32( context, address, buffer_out, 2 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT32_ARRAY( buffer_out_expected, buffer_out, 2 );
    }
}
