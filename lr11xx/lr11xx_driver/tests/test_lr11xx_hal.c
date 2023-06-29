#include "unity.h"
#include "lr11xx_hal.h"

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

void test_lr11xx_hal_compute_crc( )
{
    uint8_t buffer[] = { 0x01, 0x28, 0x01 };

    uint8_t crc = lr11xx_hal_compute_crc( 0xFF, buffer, 3 );

    TEST_ASSERT_EQUAL_INT( 0x20, crc );
}
