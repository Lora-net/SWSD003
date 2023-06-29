#include "unity.h"
#include "lr11xx_crypto_engine.h"

#include "mock_lr11xx_hal.h"

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

#define FLASH_SIZE 140
#define WORD_NUM 64
#define FLASH_BYTES_PER_SPI ( WORD_NUM * sizeof( uint32_t ) )

void* context;

void setUp( void )
{
}

void tearDown( void )
{
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_crypto_select( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x05, 0x00, 0x00 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    lr11xx_status_t status = lr11xx_crypto_select( context, LR11XX_CRYPTO_ELEMENT_CRYPTO_ENGINE );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_set_key( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                 lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]    = { 0x05, 0x02, 0xAA, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                                      0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { status_crypto_expected };

    lr11xx_crypto_status_t status_crypto;
    uint8_t                key_id = 0xAA;
    uint8_t key[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 19, 19, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    lr11xx_status_t status = lr11xx_crypto_set_key( context, &status_crypto, key_id, key );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_derive_key( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                    lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]    = { 0x05, 0x03, 0xAA, 0xBB, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
                                      0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { status_crypto_expected };

    lr11xx_crypto_status_t status_crypto;
    uint8_t                src_key_id  = 0xAA;
    uint8_t                dest_key_id = 0xBB;
    lr11xx_crypto_nonce_t  nonce       = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                           0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 20, 20, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    lr11xx_status_t status = lr11xx_crypto_derive_key( context, &status_crypto, src_key_id, dest_key_id, nonce );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_process_join_accept_lorawan_1_0_x( lr11xx_status_t        status_expected,
                                                           lr11xx_hal_status_t    hal_status,
                                                           lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]    = { 0x05, 0x04, 0xAA, 0xBB, 0x00, 0x33, 0x00, 0x01, 0x02, 0x03, 0x04,
                                      0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
    uint8_t rbuffer_in_expected[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rbuffer_out_faked[]   = {
        status_crypto_expected,
        0x10,
        0x11,
        0x12,
        0x13,
        0x14,
        0x15,
        0x16,
        0x17,
        0x18,
        0x19,
        0x1A,
        0x1B,
        0x1C,
        0x1D,
        0x1E,
        0x1F,
    };

    uint8_t data_out_expected[] = { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                                    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };

    lr11xx_crypto_status_t          status_crypto;
    uint8_t                         dec_key_id      = 0xAA;
    uint8_t                         ver_key_id      = 0xBB;
    lr11xx_crypto_lorawan_version_t lorawan_version = LR11XX_CRYPTO_LORAWAN_VERSION_1_0_X;
    uint8_t                         header[]        = { 0x33 };
    uint8_t                         data_in[]       = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };
    uint8_t data_out[16] = { 0x00 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 22, 22, rbuffer_in_expected, 17, 17,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 17 );

    lr11xx_status_t status = lr11xx_crypto_process_join_accept( context, &status_crypto, dec_key_id, ver_key_id,
                                                                lorawan_version, header, data_in, 16, data_out );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
        if( status_crypto == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            TEST_ASSERT_EQUAL_UINT8_ARRAY( data_out_expected, data_out, 16 );
        }
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_process_join_accept_lorawan_1_1_x( lr11xx_status_t        status_expected,
                                                           lr11xx_hal_status_t    hal_status,
                                                           lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]    = { 0x05, 0x04, 0xAA, 0xBB, 0x01, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
                                      0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x00, 0x01, 0x02, 0x03, 0x04,
                                      0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
    uint8_t rbuffer_in_expected[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rbuffer_out_faked[]   = {
        status_crypto_expected,
        0x10,
        0x11,
        0x12,
        0x13,
        0x14,
        0x15,
        0x16,
        0x17,
        0x18,
        0x19,
        0x1A,
        0x1B,
        0x1C,
        0x1D,
        0x1E,
        0x1F,
    };

    uint8_t data_out_expected[] = { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                                    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };

    lr11xx_crypto_status_t          status_crypto;
    uint8_t                         dec_key_id      = 0xAA;
    uint8_t                         ver_key_id      = 0xBB;
    lr11xx_crypto_lorawan_version_t lorawan_version = LR11XX_CRYPTO_LORAWAN_VERSION_1_1_X;
    uint8_t header[]  = { 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E };
    uint8_t data_in[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };
    uint8_t data_out[16] = { 0x00 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 33, 33, rbuffer_in_expected, 17, 17,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 17 );

    lr11xx_status_t status = lr11xx_crypto_process_join_accept( context, &status_crypto, dec_key_id, ver_key_id,
                                                                lorawan_version, header, data_in, 16, data_out );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
        if( status_crypto == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            TEST_ASSERT_EQUAL_UINT8_ARRAY( data_out_expected, data_out, 16 );
        }
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_compute_aes_cmac( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                          lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t             cbuffer_expected[]    = { 0x05, 0x05, 0xAA, 0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F,
                                                  0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A };
    uint8_t             rbuffer_in_expected[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t             rbuffer_out_faked[]   = { status_crypto_expected, 0x07, 0x0A, 0x16, 0xB4 };
    lr11xx_crypto_mic_t mic_expected          = { 0x07, 0x0A, 0x16, 0xB4 };

    lr11xx_crypto_status_t status_crypto;
    lr11xx_crypto_mic_t    mic;
    uint8_t                key_id = 0xAA;
    uint8_t data[] = { 0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 19, 19, rbuffer_in_expected, 5, 5,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 5 );

    lr11xx_status_t status = lr11xx_crypto_compute_aes_cmac( context, &status_crypto, key_id, data, 16, mic );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
        if( status_crypto == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            TEST_ASSERT_EQUAL_UINT8_ARRAY( mic_expected, mic, 4 );
        }
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_verify_aes_cmac( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                         lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]    = { 0x05, 0x06, 0xAA, 0x07, 0x0A, 0x16, 0xB4, 0x6B, 0xC1, 0xBE, 0xE2, 0x2E,
                                      0x40, 0x9F, 0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { status_crypto_expected };

    lr11xx_crypto_status_t status_crypto;
    lr11xx_crypto_mic_t    mic    = { 0x07, 0x0A, 0x16, 0xB4 };
    uint8_t                key_id = 0xAA;
    uint8_t data[] = { 0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 23, 23, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    lr11xx_status_t status = lr11xx_crypto_verify_aes_cmac( context, &status_crypto, key_id, data, 16, mic );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_aes_encrypt_01( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                        lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]      = { 0x05, 0x07, 0xA5, 0x01, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F,
                                        0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A };
    uint8_t rbuffer_in_expected[17] = { 0x00 };
    uint8_t rbuffer_out_faked[]     = {
        status_crypto_expected,
        0x00,
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x08,
        0x09,
        0x0A,
        0x0B,
        0x0C,
        0x0D,
        0x0E,
        0x0F,
    };

    uint8_t result_expected[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

    lr11xx_crypto_status_t status_crypto;
    uint8_t                key_id = 0xA5;
    uint8_t data[] = { 0x01, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A };
    uint8_t result[16] = { 0x00 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 19, 19, rbuffer_in_expected, 17, 17,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 17 );

    lr11xx_status_t status = lr11xx_crypto_aes_encrypt_01( context, &status_crypto, key_id, data, 16, result );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
        if( status_crypto == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            TEST_ASSERT_EQUAL_UINT8_ARRAY( result_expected, result, 16 );
        }
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_aes_encrypt( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                     lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]      = { 0x05, 0x08, 0xA5, 0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F,
                                        0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A };
    uint8_t rbuffer_in_expected[17] = { 0x00 };
    uint8_t rbuffer_out_faked[]     = {
        status_crypto_expected,
        0x00,
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x08,
        0x09,
        0x0A,
        0x0B,
        0x0C,
        0x0D,
        0x0E,
        0x0F,
    };

    uint8_t result_expected[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

    lr11xx_crypto_status_t status_crypto;
    uint8_t                key_id = 0xA5;
    uint8_t data[] = { 0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A };
    uint8_t result[16] = { 0x00 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 19, 19, rbuffer_in_expected, 17, 17,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 17 );

    lr11xx_status_t status = lr11xx_crypto_aes_encrypt( context, &status_crypto, key_id, data, 16, result );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
        if( status_crypto == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            TEST_ASSERT_EQUAL_UINT8_ARRAY( result_expected, result, 16 );
        }
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_aes_decrypt( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                     lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]      = { 0x05, 0x09, 0xA5, 0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F,
                                        0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A };
    uint8_t rbuffer_in_expected[17] = { 0x00 };
    uint8_t rbuffer_out_faked[]     = {
        status_crypto_expected,
        0x00,
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x08,
        0x09,
        0x0A,
        0x0B,
        0x0C,
        0x0D,
        0x0E,
        0x0F,
    };

    uint8_t result_expected[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

    lr11xx_crypto_status_t status_crypto;
    uint8_t                key_id = 0xA5;
    uint8_t data[] = { 0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F, 0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93, 0x17, 0x2A };
    uint8_t result[16] = { 0x00 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 19, 19, rbuffer_in_expected, 17, 17,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 17 );

    lr11xx_status_t status = lr11xx_crypto_aes_decrypt( context, &status_crypto, key_id, data, 16, result );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
        if( status_crypto == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            TEST_ASSERT_EQUAL_UINT8_ARRAY( result_expected, result, 16 );
        }
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_store_to_flash( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                        lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]    = { 0x05, 0x0A };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { status_crypto_expected };

    lr11xx_crypto_status_t status_crypto;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    lr11xx_status_t status = lr11xx_crypto_store_to_flash( context, &status_crypto );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_restore_from_flash( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                            lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]    = { 0x05, 0x0B };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { status_crypto_expected };

    lr11xx_crypto_status_t status_crypto;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    lr11xx_status_t status = lr11xx_crypto_restore_from_flash( context, &status_crypto );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_set_parameter( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                       lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]    = { 0x05, 0x0D, 0x5A, 0x81, 0x82, 0x83, 0x84 };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { status_crypto_expected };

    lr11xx_crypto_status_t status_crypto;
    uint8_t                param_id  = 0x5A;
    lr11xx_crypto_param_t  parameter = { 0x81, 0x82, 0x83, 0x84 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 7, 7, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    lr11xx_status_t status = lr11xx_crypto_set_parameter( context, &status_crypto, param_id, parameter );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_CRYPTO_STATUS_SUCCESS )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_CRYPTO_STATUS_ERROR )
void test_lr11xx_crypto_get_parameter( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                       lr11xx_crypto_status_t status_crypto_expected )
{
    uint8_t cbuffer_expected[]    = { 0x05, 0x0E, 0x5A };
    uint8_t rbuffer_in_expected[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rbuffer_out_faked[]   = { status_crypto_expected, 0x81, 0x82, 0x83, 0x84 };

    uint8_t parameter_expected[] = { 0x81, 0x82, 0x83, 0x84 };

    lr11xx_crypto_status_t status_crypto;
    uint8_t                param_id = 0x5A;
    lr11xx_crypto_param_t  parameter;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, rbuffer_in_expected, 5, 5,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 5 );

    lr11xx_status_t status = lr11xx_crypto_get_parameter( context, &status_crypto, param_id, parameter );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( status_crypto_expected, status_crypto );
        if( status_crypto == LR11XX_CRYPTO_STATUS_SUCCESS )
        {
            TEST_ASSERT_EQUAL_UINT8_ARRAY( parameter_expected, parameter, 4 );
        }
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_crypto_check_encrypted_firmware_image_full( lr11xx_status_t     status_expected,
                                                             lr11xx_hal_status_t hal_status_1,
                                                             lr11xx_hal_status_t hal_status_2,
                                                             lr11xx_hal_status_t hal_status_3 )
{
    uint8_t  cbuffer_1_exp[] = { 0x05, 0x0F, 0x01, 0x23, 0x45, 0x67 };
    uint8_t  cdata_1_exp[FLASH_BYTES_PER_SPI];
    uint8_t  cbuffer_2_exp[] = { 0x05, 0x0F, 0x01, 0x23, 0x46, 0x67 };
    uint8_t  cdata_2_exp[FLASH_BYTES_PER_SPI];
    uint8_t  cbuffer_3_exp[] = { 0x05, 0x0F, 0x01, 0x23, 0x47, 0x67 };
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

    status = lr11xx_crypto_check_encrypted_firmware_image_full( context, 0x01234567, flash, FLASH_SIZE );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 0x01 )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, 0x00 )
void test_lr11xx_crypto_get_check_encrypted_firmware_image_result( lr11xx_status_t     status_expected,
                                                                   lr11xx_hal_status_t hal_status,
                                                                   uint8_t             check_result )
{
    uint8_t cbuffer_expected[]    = { 0x05, 0x10 };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { check_result };
    bool    result_expected       = ( check_result != 0 ) ? true : false;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    bool result;

    const lr11xx_status_t status = lr11xx_crypto_get_check_encrypted_firmware_image_result( context, &result );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( result_expected, result );
    }
}
