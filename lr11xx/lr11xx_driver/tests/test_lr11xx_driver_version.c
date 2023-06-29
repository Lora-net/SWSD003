
#include "lr11xx_driver_version.h"
#include "unity.h"

void setUp( void )
{
}

void tearDown( void )
{
}

void test_driver_version( void )
{
    const char* expected_version = "v2.2.0";
    const char* version          = lr11xx_driver_version_get_version_string( );
    TEST_ASSERT_EQUAL_STRING( expected_version, version );
}