#include "lr11xx_gnss.h"
#include "lr11xx_regmem.h"
#include "unity.h"

#include "mock_lr11xx_hal.h"

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

void* context = 0;

void setUp( void )
{
}

void tearDown( void )
{
}

TEST_VALUE( LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_250_HZ )
TEST_VALUE( LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_500_HZ )
TEST_VALUE( LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_1_KHZ )
TEST_VALUE( LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_2_KHZ )
void test_lr11xx_gnss_set_freq_search_space( uint8_t freq_search_space )
{
    uint8_t cbuffer_expected[] = { 0x04, 0x04, freq_search_space };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_set_freq_search_space( context, freq_search_space );
}

TEST_VALUE( LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_250_HZ )
TEST_VALUE( LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_500_HZ )
TEST_VALUE( LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_1_KHZ )
TEST_VALUE( LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_2_KHZ )
void test_lr11xx_gnss_read_freq_search_space( uint8_t freq_search_space_expected )
{
    uint8_t cbuffer_expected[]    = { 0x04, 0x05 };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { freq_search_space_expected };

    lr11xx_gnss_freq_search_space_t freq_search_space = 0;

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_read_freq_search_space( context, &freq_search_space );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8( freq_search_space_expected, freq_search_space );
}

TEST_VALUE( 0x0003 )
TEST_VALUE( 0x0100 )
TEST_VALUE( 0x0103 )
TEST_VALUE( 0xFFFF )
TEST_VALUE( 0x0000 )
void test_lr11xx_gnss_get_result_size( uint16_t expected_result_size )
{
    uint8_t cbuffer_expected[]     = { 0x04, 0x0C };
    uint8_t rbuffer_in_expected[2] = { 0x00 };
    uint8_t rbuffer_out_faked[]    = { ( expected_result_size >> 8 ) & 0x00FF, expected_result_size & 0x00FF };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 2, 2,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 2 );

    /************************/
    /* Perform transactions */
    /************************/
    uint16_t resultSize = 0x0000;
    lr11xx_gnss_get_result_size( context, &resultSize );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT16( expected_result_size, resultSize );
}

void test_lr11xx_gnss_read_results( )
{
    uint8_t cbuffer_expected[]     = { 0x04, 0x0D };
    uint8_t rbuffer_in_expected[7] = { 0x00 };
    uint8_t rbuffer_out_faked[7]   = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };

    uint8_t result_buffer[7] = { 0x00 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 7, 7,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 7 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_read_results( context, result_buffer, 7 );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8_ARRAY( rbuffer_out_faked, result_buffer, 7 );
}

/*
 * This test validates that the GNSS Almanac update is done as 5 SPI bursts of 500 bytes then 1 SPI burst of 80 bytes
 */
void test_lr11xx_gnss_almanac_update( )
{
    uint8_t cbuffer_expected[] = { 0x04, 0x0E };

    uint8_t blocks[2580];

    for( uint16_t i = 0; i < 2580; i++ )
    {
        blocks[i] = ( uint8_t ) i;
    }

    const uint8_t  expected_number_of_calls = 5;
    const uint16_t expected_length_per_call = 500;
    for( uint8_t index_call = 0; index_call < expected_number_of_calls; index_call++ )
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn(
            context, 0, cbuffer_expected, 2, 2, blocks + ( index_call * expected_length_per_call ),
            expected_length_per_call, expected_length_per_call, LR11XX_HAL_STATUS_OK );
    }

    const uint16_t expected_remaining_length = 2580 - expected_number_of_calls * expected_length_per_call;
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, blocks + 2500,
                                               expected_remaining_length, expected_remaining_length,
                                               LR11XX_HAL_STATUS_OK );

    lr11xx_gnss_almanac_update( context, blocks, 129 );
}

#define EXPECTED_SINGLE_ALMANAC_READ_SIZE ( 2 )
#define GET_ADDRRESS_SIZE_RBUFFER_SIZE ( 6 )
void test_lr11xx_gnss_get_almanac_age_for_satellite( )
{
    const uint32_t                   address_expected                    = 0x55FF0AA0;
    const lr11xx_gnss_satellite_id_t expected_satellite_id               = 5;
    const uint8_t                    size_expected                       = 2;
    uint8_t                          cbuffer_get_address_size_expected[] = { 0x04, 0x0F };
    uint8_t                          cbuffer_regmem_expected[]           = {
                                           0x01,
                                           0x08,
                                           ( uint8_t ) ( ( address_expected + 22 * expected_satellite_id + 1 ) >> 24 ),
                                           ( uint8_t ) ( ( address_expected + 22 * expected_satellite_id + 1 ) >> 16 ),
                                           ( uint8_t ) ( ( address_expected + 22 * expected_satellite_id + 1 ) >> 8 ),
                                           ( uint8_t ) ( ( address_expected + 22 * expected_satellite_id + 1 ) >> 0 ),
                                           size_expected,
    };
    uint8_t rbuffer_get_address_size_in_expected[GET_ADDRRESS_SIZE_RBUFFER_SIZE] = { 0x00 };
    uint8_t rbuffer_get_address_size_out_faked[GET_ADDRRESS_SIZE_RBUFFER_SIZE]   = {
          address_expected >> 24,       address_expected >> 16, address_expected >> 8,
          ( uint8_t ) address_expected, size_expected >> 8,     ( uint8_t ) size_expected
    };
    uint8_t rbuffer_regmem_in_expected[EXPECTED_SINGLE_ALMANAC_READ_SIZE] = { 0x00, 0x00 };
    // Note: the implementation of the get_almanac_age_for_satellite calls _read_mem8 that returns a LSB first memory
    // reading. So on the SPI the data seems to be "LSB first" for the 2-bytes almanac age.
    uint8_t        rbuffer_regmem_out_faked[EXPECTED_SINGLE_ALMANAC_READ_SIZE] = { 0x55, 0xAF };
    const uint16_t expected_almanac_age                                        = 0xAF55;

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_get_address_size_expected, 2, 2,
                                              rbuffer_get_address_size_in_expected, GET_ADDRRESS_SIZE_RBUFFER_SIZE,
                                              GET_ADDRRESS_SIZE_RBUFFER_SIZE, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_get_address_size_out_faked, GET_ADDRRESS_SIZE_RBUFFER_SIZE );

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_regmem_expected, 7, 7, rbuffer_regmem_in_expected,
                                              EXPECTED_SINGLE_ALMANAC_READ_SIZE, EXPECTED_SINGLE_ALMANAC_READ_SIZE,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_regmem_out_faked, EXPECTED_SINGLE_ALMANAC_READ_SIZE );

    /************************/
    /* Perform transactions */
    /************************/
    uint16_t almanac_age = 0;
    lr11xx_gnss_get_almanac_age_for_satellite( context, expected_satellite_id, &almanac_age );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT16( expected_almanac_age, almanac_age );
}
#undef EXPECTED_SINGLE_ALMANAC_READ_SIZE
#undef GET_ADDRRESS_SIZE_RBUFFER_SIZE
#undef REGMEM_RBUFFER_SIZE

#define SIZE_GNSS_READ_ALMANAC ( 2820 )
#define GET_ADDRRESS_SIZE_RBUFFER_SIZE ( 6 )
#define UINT32_PER_BURST ( 47 )
void test_lr11xx_gnss_read_almanac( )
{
    const uint32_t address_expected                                                     = 0x55FF0AA0;
    const uint16_t size_expected                                                        = SIZE_GNSS_READ_ALMANAC;
    uint8_t        cbuffer_get_address_size_expected[]                                  = { 0x04, 0x0F };
    uint8_t        rbuffer_get_address_size_in_expected[GET_ADDRRESS_SIZE_RBUFFER_SIZE] = { 0x00 };
    uint8_t        rbuffer_get_address_size_out_faked[GET_ADDRRESS_SIZE_RBUFFER_SIZE]   = {
                 address_expected >> 24,       address_expected >> 16, address_expected >> 8,
                 ( uint8_t ) address_expected, size_expected >> 8,     ( uint8_t ) size_expected
    };

    lr11xx_gnss_almanac_full_read_bytestream_t expected_almanac = { 0 };
    for( uint16_t index_exp_almanac = 0; index_exp_almanac < SIZE_GNSS_READ_ALMANAC; index_exp_almanac++ )
    {
        expected_almanac[index_exp_almanac] = 2 * index_exp_almanac + 5;
    }

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_get_address_size_expected, 2, 2,
                                              rbuffer_get_address_size_in_expected, GET_ADDRRESS_SIZE_RBUFFER_SIZE,
                                              GET_ADDRRESS_SIZE_RBUFFER_SIZE, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_get_address_size_out_faked, GET_ADDRRESS_SIZE_RBUFFER_SIZE );

    const uint8_t  n_bursts        = ( SIZE_GNSS_READ_ALMANAC / 4 ) / UINT32_PER_BURST;
    const uint16_t uint8_per_burst = UINT32_PER_BURST * 4;

    uint8_t all_regmem_cbuffer[( SIZE_GNSS_READ_ALMANAC / 4 ) / UINT32_PER_BURST][7] = { 0 };

    uint8_t all_regmem_rbuffer_in[( SIZE_GNSS_READ_ALMANAC / 4 ) / UINT32_PER_BURST][UINT32_PER_BURST * 4] = { 0 };

    uint8_t all_regmem_rbuffer_out[( SIZE_GNSS_READ_ALMANAC / 4 ) / UINT32_PER_BURST][UINT32_PER_BURST * 4] = { 0 };

    for( uint8_t index_burst = 0; index_burst < n_bursts; index_burst++ )
    {
        const uint32_t local_read_regmem_address = address_expected + ( index_burst * uint8_per_burst );
        all_regmem_cbuffer[index_burst][0]       = 0x01;
        all_regmem_cbuffer[index_burst][1]       = 0x06;
        all_regmem_cbuffer[index_burst][2]       = local_read_regmem_address >> 24;
        all_regmem_cbuffer[index_burst][3]       = local_read_regmem_address >> 16;
        all_regmem_cbuffer[index_burst][4]       = local_read_regmem_address >> 8;
        all_regmem_cbuffer[index_burst][5]       = ( uint8_t ) local_read_regmem_address;
        all_regmem_cbuffer[index_burst][6]       = ( uint8_t ) UINT32_PER_BURST;

        // fill-in the expected rbuffer out
        for( uint8_t index_word_burst = 0; index_word_burst < UINT32_PER_BURST; index_word_burst++ )
        {
            const uint16_t local_index_rbuffer_out = index_word_burst * 4;

            const uint16_t local_index_expected_almanac = ( index_burst * uint8_per_burst ) + local_index_rbuffer_out;

            all_regmem_rbuffer_out[index_burst][local_index_rbuffer_out + 0] =
                expected_almanac[local_index_expected_almanac + 3];
            all_regmem_rbuffer_out[index_burst][local_index_rbuffer_out + 1] =
                expected_almanac[local_index_expected_almanac + 2];
            all_regmem_rbuffer_out[index_burst][local_index_rbuffer_out + 2] =
                expected_almanac[local_index_expected_almanac + 1];
            all_regmem_rbuffer_out[index_burst][local_index_rbuffer_out + 3] =
                expected_almanac[local_index_expected_almanac + 0];
        }
        lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, all_regmem_cbuffer[index_burst], 7, 7,
                                                  all_regmem_rbuffer_in[index_burst], uint8_per_burst, uint8_per_burst,
                                                  LR11XX_HAL_STATUS_OK );
        lr11xx_hal_read_ReturnArrayThruPtr_data( all_regmem_rbuffer_out[index_burst], uint8_per_burst );
    }

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_almanac_full_read_bytestream_t almanac_read = { 0 };
    lr11xx_gnss_read_almanac( context, almanac_read );

    /*****************/
    /* Check results */
    /*****************/
    for( uint16_t index_almanac = 0; index_almanac < SIZE_GNSS_READ_ALMANAC; index_almanac++ )
    {
        TEST_ASSERT_EQUAL_UINT8( expected_almanac[index_almanac], almanac_read[index_almanac] );
    }
}
#undef SIZE_GNSS_READ_ALMANAC
#undef GET_ADDRRESS_SIZE_RBUFFER_SIZE
#undef UINT32_PER_BURST

#define SIZE_GNSS_READ_ALMANAC ( 2820 )
#define GET_ADDRRESS_SIZE_RBUFFER_SIZE ( 6 )
void test_lr11xx_gnss_read_almanac_failure( )
{
    const uint16_t size_expected                                                        = SIZE_GNSS_READ_ALMANAC;
    uint8_t        cbuffer_get_address_size_expected[]                                  = { 0x04, 0x0F };
    uint8_t        rbuffer_get_address_size_in_expected[GET_ADDRRESS_SIZE_RBUFFER_SIZE] = { 0x00 };

    lr11xx_gnss_almanac_full_read_bytestream_t expected_almanac = { 0 };
    for( uint16_t index_exp_almanac = 0; index_exp_almanac < SIZE_GNSS_READ_ALMANAC; index_exp_almanac++ )
    {
        expected_almanac[index_exp_almanac] = 2 * index_exp_almanac + 5;
    }

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_get_address_size_expected, 2, 2,
                                              rbuffer_get_address_size_in_expected, GET_ADDRRESS_SIZE_RBUFFER_SIZE,
                                              GET_ADDRRESS_SIZE_RBUFFER_SIZE, LR11XX_HAL_STATUS_ERROR );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_almanac_full_read_bytestream_t almanac_read = { 0 };
    const lr11xx_hal_status_t                  status       = lr11xx_gnss_read_almanac( context, almanac_read );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_INT( LR11XX_HAL_STATUS_ERROR, status );
}
#undef SIZE_GNSS_READ_ALMANAC
#undef GET_ADDRRESS_SIZE_RBUFFER_SIZE

#define GET_ADDRRESS_SIZE_RBUFFER_SIZE ( 6 )
void test_fail_lr11xx_gnss_read_almanac( )
{
    uint8_t cbuffer_get_address_size_expected[]                                  = { 0x04, 0x0F };
    uint8_t rbuffer_get_address_size_in_expected[GET_ADDRRESS_SIZE_RBUFFER_SIZE] = { 0x00 };

    /********************/
    /* Set expectations */
    /********************/
    // Here, the fetch of the almanac address fails, so there must be no attempt to read the almanac
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_get_address_size_expected, 2, 2,
                                              rbuffer_get_address_size_in_expected, GET_ADDRRESS_SIZE_RBUFFER_SIZE,
                                              GET_ADDRRESS_SIZE_RBUFFER_SIZE, LR11XX_HAL_STATUS_ERROR );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_almanac_full_read_bytestream_t almanac_read = { 0 };
    const lr11xx_status_t                      status       = lr11xx_gnss_read_almanac( context, almanac_read );

    TEST_ASSERT_EQUAL_INT( LR11XX_HAL_STATUS_ERROR, status );
}
#undef GET_ADDRRESS_SIZE_RBUFFER_SIZE

void test_lr11xx_gnss_set_constellation_to_use( )
{
    uint8_t cbuffer_expected[] = { 0x04, 0x00, 0x01 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_set_constellations_to_use( context, LR11XX_GNSS_GPS_MASK );
}

TEST_VALUE( LR11XX_GNSS_GPS_MASK )
TEST_VALUE( LR11XX_GNSS_GPS_MASK | LR11XX_GNSS_BEIDOU_MASK )
void test_lr11xx_gnss_read_used_constellations( uint8_t constellation_in_use_expected )
{
    uint8_t cbuffer_expected[]    = { 0x04, 0x01 };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { constellation_in_use_expected };

    uint8_t constellation_in_use = 0;

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_read_used_constellations( context, &constellation_in_use );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT32( constellation_in_use_expected, constellation_in_use );
}

TEST_VALUE( LR11XX_GNSS_BEIDOU_MASK )
TEST_VALUE( LR11XX_GNSS_GPS_MASK | LR11XX_GNSS_BEIDOU_MASK )
void test_lr11xx_gnss_set_almanac_update( uint8_t almanac_update_bit_mask )
{
    uint8_t cbuffer_expected[] = { 0x04, 0x02, almanac_update_bit_mask };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_set_almanac_update( context, almanac_update_bit_mask );
}

TEST_VALUE( LR11XX_GNSS_GPS_MASK )
TEST_VALUE( LR11XX_GNSS_GPS_MASK | LR11XX_GNSS_BEIDOU_MASK )
void test_lr11xx_gnss_read_almanac_update( uint8_t almanac_update_bit_mask_expected )
{
    uint8_t cbuffer_expected[]    = { 0x04, 0x03 };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { almanac_update_bit_mask_expected };

    uint8_t almanac_update_bit_mask = 0;

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_read_almanac_update( context, &almanac_update_bit_mask );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT32( almanac_update_bit_mask_expected, almanac_update_bit_mask );
}

TEST_VALUE( 0xA5, 0x00 )
TEST_VALUE( 0xA5, 0xFF )
TEST_VALUE( 0x00, 0x00 )
void test_lr11xx_gnss_read_firmware_version( uint8_t firmware_version_expected, uint8_t almanac_version_expected )
{
    uint8_t cbuffer_expected[2]    = { 0x04, 0x06 };
    uint8_t rbuffer_in_expected[2] = { 0x00, 0x00 };
    uint8_t rbuffer_out_faked[2]   = { firmware_version_expected, almanac_version_expected };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 2, 2,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 2 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_version_t version = { 0 };
    lr11xx_gnss_read_firmware_version( context, &version );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8( firmware_version_expected, version.gnss_firmware );
    TEST_ASSERT_EQUAL_UINT8( almanac_version_expected, version.gnss_almanac );
}

TEST_VALUE( LR11XX_GNSS_GPS_MASK )
TEST_VALUE( LR11XX_GNSS_GPS_MASK | LR11XX_GNSS_BEIDOU_MASK )
void test_lr11xx_gnss_read_supported_constellation( uint8_t supported_constellation_expected )
{
    uint8_t cbuffer_expected[]    = { 0x04, 0x07 };
    uint8_t rbuffer_in_expected[] = { 0x00 };
    uint8_t rbuffer_out_faked[]   = { supported_constellation_expected };

    uint8_t supported_constellation = 0;

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_read_supported_constellations( context, &supported_constellation );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_UINT8( supported_constellation_expected, supported_constellation );
}

TEST_VALUE( 0, 0 )
TEST_VALUE( 1, 0 )
TEST_VALUE( 0, 1 )
TEST_VALUE( 1, 1 )
TEST_VALUE( 700, 256 )
TEST_VALUE( 256, 700 )
TEST_VALUE( 0x0001FFFF, 1 )
void test_lr11xx_gnss_get_timings( uint32_t timing_radio_expected, uint32_t timing_computation_expected )
{
    const uint8_t  cbuffer_expected[2]                   = { 0x04, 0x19 };
    uint8_t        rbuffer_expected[8]                   = { 0 };
    const uint32_t timing_computation_expected_spi_value = timing_computation_expected * 1000;
    const uint32_t timing_radio_expected_spi_value       = timing_radio_expected * 1000;
    uint8_t        rbuffer_fake[8]                       = {
                                     ( uint8_t ) ( timing_computation_expected_spi_value >> 24 ),
                                     ( uint8_t ) ( timing_computation_expected_spi_value >> 16 ),
                                     ( uint8_t ) ( timing_computation_expected_spi_value >> 8 ),
                                     ( uint8_t ) timing_computation_expected_spi_value,
                                     ( uint8_t ) ( timing_radio_expected_spi_value >> 24 ),
                                     ( uint8_t ) ( timing_radio_expected_spi_value >> 16 ),
                                     ( uint8_t ) ( timing_radio_expected_spi_value >> 8 ),
                                     ( uint8_t ) timing_radio_expected_spi_value,
    };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_expected, 8, 8,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_fake, 8 );

    lr11xx_gnss_timings_t gnss_timing = { 0 };

    lr11xx_gnss_get_timings( context, &gnss_timing );

    TEST_ASSERT_EQUAL_UINT32( timing_radio_expected, gnss_timing.radio_ms );
    TEST_ASSERT_EQUAL_UINT32( timing_computation_expected, gnss_timing.computation_ms );
}

void test_lr11xx_gnss_set_scan_mode( )
{
    const lr11xx_gnss_scan_mode_t expected_scan_mode  = LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS;
    const uint8_t                 cbuffer_expected[3] = { 0x04, 0x08, ( uint8_t ) ( expected_scan_mode ) };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_gnss_set_scan_mode( context, expected_scan_mode );
}

TEST_VALUE( 0xA5, 12, LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( 0xA5, 12, LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( 0x5A, 15, LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_scan_autonomous( uint8_t gnss_input_parameters, uint8_t nb_sat, lr11xx_status_t status_expected,
                                       lr11xx_hal_status_t hal_status_1, lr11xx_hal_status_t hal_status_2 )
{
    uint32_t      expected_date        = 1580145516;
    const uint8_t cbuffer_expected_1[] = { 0x01, 0x0C, 0x00, 0xF3, 0x00, 0x24, 0x00,
                                           0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10 };
    uint8_t       cbuffer_expected_2[] = { 0x04,
                                           0x09,
                                           ( uint8_t ) ( expected_date >> 24 ),
                                           ( uint8_t ) ( expected_date >> 16 ),
                                           ( uint8_t ) ( expected_date >> 8 ),
                                           ( uint8_t ) ( expected_date >> 0 ),
                                           0x00,
                                           gnss_input_parameters,
                                           nb_sat };

    do
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_1, 14, 14, NULL, 0, 0, hal_status_1 );
        if( hal_status_1 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }

        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_2, 9, 9, NULL, 0, 0, hal_status_2 );
        if( hal_status_2 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    const lr11xx_status_t status = lr11xx_gnss_scan_autonomous( context, expected_date, LR11XX_GNSS_OPTION_DEFAULT,
                                                                gnss_input_parameters, nb_sat );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( 23000, LR11XX_GNSS_OPTION_DEFAULT, 10, 12, LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( 23000, LR11XX_GNSS_OPTION_DEFAULT, 10, 12, LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR,
            LR11XX_HAL_STATUS_OK )
TEST_VALUE( 10, LR11XX_GNSS_OPTION_BEST_EFFORT, 12, 15, LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK,
            LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_gnss_scan_assisted( uint32_t nb_of_seconds, lr11xx_gnss_search_mode_t search_mode,
                                     uint8_t gnss_input_parameters, uint8_t nb_sat, lr11xx_status_t status_expected,
                                     lr11xx_hal_status_t hal_status_1, lr11xx_hal_status_t hal_status_2 )
{
    const uint8_t cbuffer_expected_1[] = { 0x01, 0x0C, 0x00, 0xF3, 0x00, 0x24, 0x00,
                                           0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10 };
    uint8_t       cbuffer_expected_2[] = { 0x04,
                                           0x0A,
                                           ( uint8_t ) ( nb_of_seconds >> 24 ),
                                           ( uint8_t ) ( nb_of_seconds >> 16 ),
                                           ( uint8_t ) ( nb_of_seconds >> 8 ),
                                           ( uint8_t ) ( nb_of_seconds >> 0 ),
                                           search_mode,
                                           gnss_input_parameters,
                                           nb_sat };

    lr11xx_gnss_date_t gnss_date = nb_of_seconds;

    do
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_1, 14, 14, NULL, 0, 0, hal_status_1 );
        if( hal_status_1 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }

        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_2, 9, 9, NULL, 0, 0, hal_status_2 );
        if( hal_status_2 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    const lr11xx_status_t status =
        lr11xx_gnss_scan_assisted( context, gnss_date, search_mode, gnss_input_parameters, nb_sat );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( -25.2, 26.3 )
TEST_VALUE( 35, -43 )
void test_lr11xx_gnss_set_assistance_position( float latitude_in_deg, float longitude_in_deg )
{
    uint8_t cbuffer_expected[] = { 0x04, 0x10, 0x00, 0x00, 0x00, 0x00 };

    int16_t                                  latitude            = latitude_in_deg * 2048 / 90;
    int16_t                                  longitude           = longitude_in_deg * 2048 / 180;
    lr11xx_gnss_solver_assistance_position_t assistance_position = { .latitude  = latitude_in_deg,
                                                                     .longitude = longitude_in_deg };

    cbuffer_expected[2] = ( uint8_t ) ( latitude >> 8 );
    cbuffer_expected[3] = ( uint8_t ) ( latitude >> 0 );
    cbuffer_expected[4] = ( uint8_t ) ( longitude >> 8 );
    cbuffer_expected[5] = ( uint8_t ) ( longitude >> 0 );

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_set_assistance_position( context, &assistance_position );
}

TEST_VALUE( 3000, 0 )
TEST_VALUE( 0, 4200 )
TEST_VALUE( -4000, 0 )
TEST_VALUE( 0, -1200 )
void test_lr11xx_gnss_read_assistance_position( int16_t latitude, int16_t longitude )
{
    uint8_t cbuffer_expected[]  = { 0x04, 0x11 };
    uint8_t rbuffer_expected[4] = { 0x00 };
    uint8_t rbuffer_fake[4];

    lr11xx_gnss_solver_assistance_position_t assistance_position;

    /********************/
    /* Set expectations */
    /********************/
    rbuffer_fake[0] = ( uint8_t ) ( latitude >> 8 );
    rbuffer_fake[1] = ( uint8_t ) ( latitude >> 0 );
    rbuffer_fake[2] = ( uint8_t ) ( longitude >> 8 );
    rbuffer_fake[3] = ( uint8_t ) ( longitude >> 0 );
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_expected, 4, 4,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_fake, 4 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_read_assistance_position( context, &assistance_position );

    /*****************/
    /* Check results */
    /*****************/
    float latitude_expected  = ( float ) ( latitude ) *90 / 2048;
    float longitude_expected = ( float ) ( longitude ) *180 / 2048;
    TEST_ASSERT_EQUAL_FLOAT( latitude_expected, assistance_position.latitude );
    TEST_ASSERT_EQUAL_FLOAT( longitude_expected, assistance_position.longitude );
}

void test_lr11xx_gnss_push_dmc_msg( )
{
    uint8_t cbuffer_expected[] = { 0x04, 0x15 };
    uint8_t cdata_expected[]   = { 0x01, 0x02, 0x03, 0x04, 0x05 };

    uint8_t dmc_message[] = { 0x01, 0x02, 0x03, 0x04, 0x05 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, cdata_expected, 5, 5,
                                               LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_push_dmc_msg( context, dmc_message, 5 );
}

void test_lr11xx_gnss_push_solver_msg( )
{
    uint8_t cbuffer_expected[] = { 0x04, 0x14 };
    uint8_t cdata_expected[]   = { 0x01, 0x02, 0x03, 0x04, 0x05 };

    uint8_t payload[] = { 0x01, 0x02, 0x03, 0x04, 0x05 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, cdata_expected, 5, 5,
                                               LR11XX_HAL_STATUS_OK );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_push_solver_msg( context, payload, 5 );
}

void test_lr11xx_gnss_get_nb_detected_satellites( )
{
    const uint8_t expected_nb_detected = 10;
    uint8_t       cbuffer_expected[]   = { 0x04, 0x17 };
    uint8_t       rbuffer_expected[1]  = { 0x00 };
    uint8_t       rbuffer_fake[]       = { expected_nb_detected };

    /********************/
    /* Set expectations */
    /********************/

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_expected, 1, 1,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_fake, 1 );

    /************************/
    /* Perform transactions */
    /************************/
    uint8_t nb_satellite_detected = 0;
    lr11xx_gnss_get_nb_detected_satellites( context, &nb_satellite_detected );

    TEST_ASSERT_EQUAL_UINT8( expected_nb_detected, nb_satellite_detected );
}

void test_lr11xx_gnss_get_detected_satellites( )
{
    const uint8_t nb_detected         = 2;
    uint8_t       cbuffer_expected[]  = { 0x04, 0x18 };
    uint8_t       rbuffer_expected[8] = { 0 };
    uint8_t       rbuffer_fake[]      = { 0x01, 0xFA, 0x80, 0x01, 0x08, 0x42, 0x00, 0x01 };

    /********************/
    /* Set expectations */
    /********************/

    const uint8_t svid_1    = 1;
    const int8_t  snr_1     = 25;
    const int16_t doppler_1 = -32767;
    const uint8_t svid_2    = 8;
    const int8_t  snr_2     = 97;
    const int16_t doppler_2 = 1;
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_expected, 8, 8,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_fake, 8 );

    /************************/
    /* Perform transactions */
    /************************/
    lr11xx_gnss_detected_satellite_t detected_satellites[4] = { 0 };
    lr11xx_gnss_get_detected_satellites( context, nb_detected, detected_satellites );

    TEST_ASSERT_EQUAL_UINT8( detected_satellites[0].satellite_id, svid_1 );
    TEST_ASSERT_EQUAL_UINT8( detected_satellites[0].cnr, snr_1 );
    TEST_ASSERT_EQUAL_UINT8( detected_satellites[0].doppler, doppler_1 );
    TEST_ASSERT_EQUAL_UINT8( detected_satellites[1].satellite_id, svid_2 );
    TEST_ASSERT_EQUAL_UINT8( detected_satellites[1].cnr, snr_2 );
    TEST_ASSERT_EQUAL_UINT8( detected_satellites[1].doppler, doppler_2 );
}

void test_lr11xx_gnss_read_gnss_rssi_test( void )
{
    int16_t rssi_gnss_dbm_expected = -120;
    uint8_t cbuffer_expected[4]    = { 0x02, 0x22, 0x09, 0x00 };
    uint8_t rbuffer_in_expected[2] = { 0x00 };
    uint8_t rbuffer_out_faked[2]   = { 0x00, 0x88 };

    /********************/
    /* Set expectations */
    /********************/
    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 4, 4, rbuffer_in_expected, 2, 2,
                                              LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 2 );

    /************************/
    /* Perform transactions */
    /************************/
    int8_t rssi_gnss_dbm = 0;
    lr11xx_gnss_read_gnss_rssi_test( context, &rssi_gnss_dbm );

    /*****************/
    /* Check results */
    /*****************/
    TEST_ASSERT_EQUAL_INT8( rssi_gnss_dbm_expected, rssi_gnss_dbm );
}

void test_lr11xx_gnss_get_nb_visible_satellites( void )
{
    uint8_t cbuffer_expected[11] = { 0x04, 0x1F, 0x01, 0x23, 0x45, 0x67, 0x01, 0x11, 0x01, 0x82, 0x01 };
    uint8_t rbuffer_out_faked[1] = { 0x0A };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 11, 11, NULL, 1, 1, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_IgnoreArg_data( );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    const lr11xx_gnss_date_t                       date                = 0x01234567;
    const lr11xx_gnss_solver_assistance_position_t assistance_position = {
        .latitude  = 12,
        .longitude = 34,
    };
    const lr11xx_gnss_constellation_t constellation = LR11XX_GNSS_BEIDOU_MASK;
    uint8_t                           nb_visible_sv;

    lr11xx_gnss_get_nb_visible_satellites( context, date, &assistance_position, constellation, &nb_visible_sv );

    TEST_ASSERT_EQUAL_UINT8( 0x0A, nb_visible_sv );
}

void test_lr11xx_gnss_get_visible_satellites( void )
{
    uint8_t cbuffer_expected[11]  = { 0x04, 0x20 };
    uint8_t rbuffer_out_faked[15] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                      0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, NULL, 15, 15, LR11XX_HAL_STATUS_OK );
    lr11xx_hal_read_IgnoreArg_data( );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 15 );

    const uint8_t                   nb_visible_sv = 3;
    lr11xx_gnss_visible_satellite_t result[3];

    lr11xx_gnss_get_visible_satellites( context, nb_visible_sv, &result );

    TEST_ASSERT_EQUAL_UINT8( 0x01, result[0].satellite_id );
    TEST_ASSERT_EQUAL_UINT16( 0x0203, result[0].doppler );
    TEST_ASSERT_EQUAL_UINT16( 0x0405, result[0].doppler_error );
    TEST_ASSERT_EQUAL_UINT8( 0x06, result[1].satellite_id );
    TEST_ASSERT_EQUAL_UINT16( 0x0708, result[1].doppler );
    TEST_ASSERT_EQUAL_UINT16( 0x090A, result[1].doppler_error );
    TEST_ASSERT_EQUAL_UINT8( 0x0B, result[2].satellite_id );
    TEST_ASSERT_EQUAL_UINT16( 0x0C0D, result[2].doppler );
    TEST_ASSERT_EQUAL_UINT16( 0x0E0F, result[2].doppler_error );
}

TEST_VALUE( 0x00, 0x01, LR11XX_GNSS_DESTINATION_HOST, LR11XX_STATUS_OK )
TEST_VALUE( 0x01, 0x01, LR11XX_GNSS_DESTINATION_SOLVER, LR11XX_STATUS_OK )
TEST_VALUE( 0x02, 0x01, LR11XX_GNSS_DESTINATION_DMC, LR11XX_STATUS_OK )
TEST_VALUE( 0x02, 0x00, LR11XX_GNSS_DESTINATION_DMC, LR11XX_STATUS_ERROR )
TEST_VALUE( 0x03, 0x00, LR11XX_GNSS_DESTINATION_DMC, LR11XX_STATUS_ERROR )
void test_lr11xx_gnss_get_result_destination( uint8_t destination_uint, uint16_t buffer_size,
                                              lr11xx_gnss_destination_t destination_expected,
                                              lr11xx_status_t           status_expected )
{
    const uint8_t             result[1] = { destination_uint };
    lr11xx_gnss_destination_t destination;

    const lr11xx_status_t status = lr11xx_gnss_get_result_destination( result, buffer_size, &destination );

    TEST_ASSERT_EQUAL_UINT8( status_expected, status );

    if( status == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( destination_expected, destination );
    }
}

void test_lr11xx_gnss_compute_almanac_age( )
{
    const uint16_t now                  = 16000;
    const uint16_t rollover             = 14000;
    const uint16_t almanac_date         = 500;
    const uint16_t expected_almanac_age = 1500;

    const uint16_t almanac_age = lr11xx_gnss_compute_almanac_age( almanac_date, rollover, now );

    TEST_ASSERT_EQUAL_UINT16( expected_almanac_age, almanac_age );
}
