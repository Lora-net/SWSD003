#include "lr11xx_gnss_alpha.h"
#include "unity.h"

#include <string.h>  // for memset

#include "mock_lr11xx_hal.h"

TEST_FILE( "lr11xx_regmem.c" )

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

void* radio = 0;

void setUp( void )
{
}

void tearDown( void )
{
}

#define GET_ALMANAC_CRC_OFFSET ( 128 * 22 )
#define GET_ADDRRESS_SIZE_RBUFFER_SIZE ( 6 )
#define EXPECTED_SIZE_U32 ( 1 )
#define EXPECTED_SIZE_U8 ( EXPECTED_SIZE_U32 * 4 )
void test_lr11xx_gnss_get_almanac_crc( )
{
    const uint32_t address_expected                    = 0x55FF0AA0;
    const uint32_t expected_crc                        = 0x01234567;
    uint8_t        cbuffer_get_address_size_expected[] = { 0x04, 0x0F };
    const uint16_t fake_almanac_size                   = 1022;
    uint8_t        cbuffer_regmem_expected[]           = {
        0x01,
        0x06,
        ( uint8_t ) ( ( address_expected + GET_ALMANAC_CRC_OFFSET ) >> 24 ),
        ( uint8_t ) ( ( address_expected + GET_ALMANAC_CRC_OFFSET ) >> 16 ),
        ( uint8_t ) ( ( address_expected + GET_ALMANAC_CRC_OFFSET ) >> 8 ),
        ( uint8_t ) ( ( address_expected + GET_ALMANAC_CRC_OFFSET ) >> 0 ),
        EXPECTED_SIZE_U32,
    };
    uint8_t rbuffer_get_address_size_in_expected[GET_ADDRRESS_SIZE_RBUFFER_SIZE] = { 0x00 };
    uint8_t rbuffer_get_address_size_out_faked[GET_ADDRRESS_SIZE_RBUFFER_SIZE]   = {
        ( uint8_t ) ( address_expected >> 24 ), ( uint8_t ) ( address_expected >> 16 ),
        ( uint8_t ) ( address_expected >> 8 ),  ( uint8_t ) address_expected,
        ( uint8_t ) ( fake_almanac_size >> 8 ), ( uint8_t ) fake_almanac_size
    };
    uint8_t rbuffer_regmem_in_expected[EXPECTED_SIZE_U8] = { 0x00 };
    // Note: the implementation of the get_almanac_age_for_satellite calls _read_mem8 that returns a LSB first memory
    // reading. So on the SPI the data seems to be "LSB first" for the 2-bytes almanac age.
    uint8_t rbuffer_regmem_out_faked[EXPECTED_SIZE_U8] = { ( uint8_t ) ( expected_crc >> 24 ),
                                                           ( uint8_t ) ( expected_crc >> 16 ),
                                                           ( uint8_t ) ( expected_crc >> 8 ),
                                                           ( uint8_t ) expected_crc };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_get_address_size_expected, 2, 2,
                                              rbuffer_get_address_size_in_expected, GET_ADDRRESS_SIZE_RBUFFER_SIZE,
                                              GET_ADDRRESS_SIZE_RBUFFER_SIZE, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_get_address_size_out_faked, GET_ADDRRESS_SIZE_RBUFFER_SIZE );

    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_regmem_expected, 7, 7, rbuffer_regmem_in_expected,
                                              EXPECTED_SIZE_U8, EXPECTED_SIZE_U8, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_regmem_out_faked, EXPECTED_SIZE_U8 );

    /************************/
    /* Perform transactions */
    /************************/
    uint32_t almanac_crc = 0;
    lr11xx_gnss_get_almanac_crc( radio, &almanac_crc );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT16( expected_crc, almanac_crc );
}
#undef GET_ALMANAC_CRC_OFFSET
#undef GET_ADDRRESS_SIZE_RBUFFER_SIZE

#define GET_ADDRRESS_SIZE_RBUFFER_SIZE ( 6 )
void test_lr11xx_gnss_get_almanac_crc_failure( )
{
    uint8_t cbuffer_get_address_size_expected[]                                  = { 0x04, 0x0F };
    uint8_t rbuffer_get_address_size_in_expected[GET_ADDRRESS_SIZE_RBUFFER_SIZE] = { 0x00 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_get_address_size_expected, 2, 2,
                                              rbuffer_get_address_size_in_expected, GET_ADDRRESS_SIZE_RBUFFER_SIZE,
                                              GET_ADDRRESS_SIZE_RBUFFER_SIZE, LR11XX_HAL_STATUS_ERROR );

    /************************/
    /* Perform transactions */
    /************************/
    uint32_t                  almanac_crc = 0;
    const lr11xx_hal_status_t status      = lr11xx_gnss_get_almanac_crc( radio, &almanac_crc );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_INT( LR11XX_HAL_STATUS_ERROR, status );
}
#undef GET_ADDRRESS_SIZE_RBUFFER_SIZE

void test_lr11xx_gnss_get_result_size_first_scan( )
{
    const uint16_t expected_result_size   = 0x0052;
    uint8_t        cbuffer_expected[]     = { 0x04, 0x25 };
    uint8_t        rbuffer_in_expected[2] = { 0x00 };
    uint8_t        rbuffer_out_faked[]    = { ( expected_result_size >> 8 ) & 0x00FF, expected_result_size & 0x00FF };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 2, 2,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 2 );

    /************************/
    /* Perform transactions */
    /************************/
    uint16_t resultSize = 0x0000;
    lr11xx_gnss_get_first_scan_result_size( radio, &resultSize );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT16( expected_result_size, resultSize );
}

void test_lr11xx_gnss_get_result_size_intermediate_scan( )
{
    const uint16_t expected_result_size   = 0x0052;
    uint8_t        cbuffer_expected[]     = { 0x04, 0x29 };
    uint8_t        rbuffer_in_expected[2] = { 0x00 };
    uint8_t        rbuffer_out_faked[]    = { ( expected_result_size >> 8 ) & 0x00FF, expected_result_size & 0x00FF };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 2, 2,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 2 );

    /************************/
    /* Perform transactions */
    /************************/
    uint16_t resultSize = 0x0000;
    lr11xx_gnss_get_intermediate_scan_result_size( radio, &resultSize );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT16( expected_result_size, resultSize );
}

void test_lr11xx_gnss_read_results_first_scan( )
{
    uint8_t cbuffer_expected[]     = { 0x04, 0x26 };
    uint8_t rbuffer_in_expected[7] = { 0x00 };
    uint8_t rbuffer_out_faked[7]   = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };

    uint8_t result_buffer[7] = { 0x00 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 7, 7,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 7 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_read_first_scan_results( radio, result_buffer, 7 );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8_ARRAY( rbuffer_out_faked, result_buffer, 7 );
}

void test_lr11xx_gnss_read_results_intermediate_scan( )
{
    uint8_t cbuffer_expected[]     = { 0x04, 0x2A };
    uint8_t rbuffer_in_expected[7] = { 0x00 };
    uint8_t rbuffer_out_faked[7]   = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };

    uint8_t result_buffer[7] = { 0x00 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 7, 7,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 7 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_read_intermediate_scan_results( radio, result_buffer, 7 );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8_ARRAY( rbuffer_out_faked, result_buffer, 7 );
}

TEST_VALUE( 1.2 )
TEST_VALUE( 0 )
TEST_VALUE( -0.3 )
void test_lr11xx_gnss_set_xtal_error( float xtal_error_in_ppm )
{
    uint8_t cbuffer_expected[] = { 0x04, 0x12, 0x00, 0x00 };
    int16_t xtal_error         = ( int16_t ) ( xtal_error_in_ppm * 32768 / 40 );

    cbuffer_expected[2] = ( uint8_t ) ( xtal_error >> 8 );
    cbuffer_expected[3] = ( uint8_t ) ( xtal_error >> 0 );

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 4, 4, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_set_xtal_error( radio, xtal_error_in_ppm );
}

TEST_VALUE( 1200 )
TEST_VALUE( 0 )
TEST_VALUE( -3003 )
void test_lr11xx_gnss_read_xtal_error( int16_t xtal_error )
{
    uint8_t cbuffer_expected[]  = { 0x04, 0x13 };
    uint8_t rbuffer_expected[2] = { 0x00 };
    uint8_t rbuffer_fake[]      = {
        ( uint8_t ) ( xtal_error >> 8 ),
        ( uint8_t ) ( xtal_error >> 0 ),
    };

    float xtal_error_in_ppm;

    /********************/
    /* Set expectations */
    /********************/
    rbuffer_fake[0] = ( uint8_t ) ( xtal_error >> 8 );
    rbuffer_fake[1] = ( uint8_t ) ( xtal_error >> 0 );
    lr11xx_hal_read_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 2, 2, rbuffer_expected, 2, 2,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_fake, 2 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_read_xtal_error( radio, &xtal_error_in_ppm );

    /*****************/
    /* Check results */
    /*****************/
    float xtal_error_in_ppm_expected = ( float ) ( xtal_error ) *40 / 32768;
    TEST_ASSERT_EQUAL_FLOAT( xtal_error_in_ppm_expected, xtal_error_in_ppm );
}
