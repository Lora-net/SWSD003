#include "unity.h"
#include "lr11xx_regmem.h"

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
void test_lr11xx_regmem_write_regmem32( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x05, 0x01, 0x23, 0x45, 0x67 };
    uint8_t cdata_expected[]   = { 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0xFE, 0xDC, 0xBA, 0x98 };

    uint32_t        address = 0x01234567;
    uint32_t        data[3] = { 0x89ABCDEF, 0x01234567, 0xFEDCBA98 };
    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, cdata_expected, 12, 12, hal_status );

    status = lr11xx_regmem_write_regmem32( context, address, data, 3 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, 1 )
TEST_VALUE( LR11XX_STATUS_OK, 64 )
TEST_VALUE( LR11XX_STATUS_ERROR, 65 )
void test_lr11xx_regmem_write_regmem32_length( lr11xx_status_t status_expected, uint8_t nb_of_words )
{
    uint32_t        address = 0x01234567;
    uint32_t        data[nb_of_words];
    lr11xx_status_t status;

    lr11xx_hal_write_IgnoreAndReturn( LR11XX_HAL_STATUS_OK );

    status = lr11xx_regmem_write_regmem32( context, address, data, nb_of_words );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_regmem_read_regmem32( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]      = { 0x01, 0x06, 0x01, 0x23, 0x45, 0x67, 0x03 };
    uint8_t rbuffer_in_expected[12] = { 0x00 };
    uint8_t rbuffer_out_faked[12]   = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C };

    uint32_t        address                = 0x01234567;
    uint32_t        buffer_out[3]          = { 0 };
    uint32_t        buffer_out_expected[3] = { 0x01020304, 0x05060708, 0x090A0B0C };
    lr11xx_status_t status;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 7, 7, rbuffer_in_expected, 12, 12,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 12 );

    status = lr11xx_regmem_read_regmem32( context, address, buffer_out, 3 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT32_ARRAY( buffer_out_expected, buffer_out, 3 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, 1 )
TEST_VALUE( LR11XX_STATUS_OK, 64 )
TEST_VALUE( LR11XX_STATUS_ERROR, 65 )
void test_lr11xx_regmem_read_regmem32_length( lr11xx_status_t status_expected, uint8_t nb_of_words )
{
    uint32_t        address = 0x01234567;
    uint32_t        buffer_out[nb_of_words];
    lr11xx_status_t status;

    lr11xx_hal_read_IgnoreAndReturn( LR11XX_HAL_STATUS_OK );

    status = lr11xx_regmem_read_regmem32( context, address, buffer_out, nb_of_words );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_regmem_write_mem8( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x07, 0x01, 0x23, 0x45, 0x67 };
    uint8_t cdata_expected[]   = { 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34 };

    uint32_t        address = 0x01234567;
    uint8_t         data[]  = { 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34 };
    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, cdata_expected, 8, 8, hal_status );

    status = lr11xx_regmem_write_mem8( context, address, data, 8 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_regmem_read_mem8( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[7]    = { 0x01, 0x08, 0x12, 0x34, 0x56, 0x78, 0x05 };
    uint8_t rbuffer_in_expected[7] = { 0x00 };
    uint8_t rbuffer_out_faked[5]   = { 0x01, 0x23, 0x45, 0x67, 0x89 };

    uint32_t        address      = 0x12345678;
    uint8_t         bufferOut[5] = { 0x00 };
    lr11xx_status_t status;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 7, 7, rbuffer_in_expected, 5, 5,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 5 );

    status = lr11xx_regmem_read_mem8( context, address, bufferOut, 5 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( rbuffer_out_faked, bufferOut, 5 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_regmem_write_buffer8( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x09 };
    uint8_t cdata_expected[]   = { 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12 };

    uint8_t         data[] = { 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12 };
    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, cdata_expected, 7, 7, hal_status );

    status = lr11xx_regmem_write_buffer8( context, data, 7 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_regmem_read_buffer8( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x01, 0x0A, 0xA5, 0x04 };
    uint8_t rbuffer_in_expected[4] = { 0x00 };
    uint8_t rbuffer_out_faked[]    = { 0x01, 0x23, 0x45, 0x67 };

    uint8_t         bufferOut[4] = { 0x00 };
    lr11xx_status_t status;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 4, 4, rbuffer_in_expected, 4, 4,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 4 );

    status = lr11xx_regmem_read_buffer8( context, bufferOut, 0xA5, 4 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY( rbuffer_out_faked, bufferOut, 4 );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_regmem_clear_rxbuffer( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x0B };

    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, NULL, 0, 0, hal_status );

    status = lr11xx_regmem_clear_rxbuffer( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_regmem_write_regmem32_mask( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x01, 0x0C, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78 };

    uint32_t        address = 0x01234567;
    uint32_t        mask    = 0x89ABCDEF;
    uint32_t        data    = 0x12345678;
    lr11xx_status_t status;

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 14, 14, NULL, 0, 0, hal_status );

    status = lr11xx_regmem_write_regmem32_mask( context, address, mask, data );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}
