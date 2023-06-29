#include "lr11xx_radio.h"
#include "unity.h"
#include "sx126x_toa.h"

#include <string.h>  // for memset

#include "mock_lr11xx_hal.h"

#if defined( TEST_PP )
#define TEST_VALUE( ... ) TEST_CASE( __VA_ARGS__ )
#else
#define TEST_VALUE( ... )
#endif

TEST_FILE( "lr11xx_regmem.c" )

#define CEILING( x ) ( int ) ( x ) + ( 1 - ( int ) ( ( int ) ( ( x ) + 1 ) - ( x ) ) )

void* context = 0;

void setUp( void )
{
}

void tearDown( void )
{
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_reset_stats( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x00 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_reset_stats( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_get_stats_gfsk( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x02, 0x01 };
    uint8_t rbuffer_in_expected[6] = { 0 };
    uint8_t rbuffer_out_faked[]    = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 6, 6,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 6 );

    lr11xx_radio_stats_gfsk_t stats;
    lr11xx_status_t           status;

    status = lr11xx_radio_get_gfsk_stats( context, &stats );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT16( 0x0123, stats.nb_pkt_received );
        TEST_ASSERT_EQUAL_UINT16( 0x4567, stats.nb_pkt_crc_error );
        TEST_ASSERT_EQUAL_UINT16( 0x89AB, stats.nb_pkt_len_error );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_get_stats_lora( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x02, 0x01 };
    uint8_t rbuffer_in_expected[8] = { 0 };
    uint8_t rbuffer_out_faked[]    = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 8, 8,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 8 );

    lr11xx_radio_stats_lora_t stats;
    lr11xx_status_t           status;

    status = lr11xx_radio_get_lora_stats( context, &stats );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT16( 0x0123, stats.nb_pkt_received );
        TEST_ASSERT_EQUAL_UINT16( 0x4567, stats.nb_pkt_crc_error );
        TEST_ASSERT_EQUAL_UINT16( 0x89AB, stats.nb_pkt_header_error );
        TEST_ASSERT_EQUAL_UINT16( 0xCDEF, stats.nb_pkt_falsesync );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_get_packet_type( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x02, 0x02 };
    uint8_t rbuffer_in_expected[1] = { 0x00 };
    uint8_t rbuffer_out_faked[]    = { 0x01 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    lr11xx_radio_pkt_type_t pkt_type = LR11XX_RADIO_PKT_NONE;
    lr11xx_status_t         status;

    status = lr11xx_radio_get_pkt_type( context, &pkt_type );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( LR11XX_RADIO_PKT_TYPE_GFSK, pkt_type );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_get_rxbuffer_status( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x02, 0x03 };
    uint8_t rbuffer_in_expected[2] = { 0x00 };
    uint8_t rbuffer_out_faked[]    = { 0x01, 0x23 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 2, 2,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 2 );

    lr11xx_radio_rx_buffer_status_t rx_buffer_status;
    lr11xx_status_t                 status;

    status = lr11xx_radio_get_rx_buffer_status( context, &rx_buffer_status );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( 0x01, rx_buffer_status.pld_len_in_bytes );
        TEST_ASSERT_EQUAL_UINT8( 0x23, rx_buffer_status.buffer_start_pointer );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_get_packet_status_gfsk( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x02, 0x04 };
    uint8_t rbuffer_in_expected[4] = { 0x00 };
    uint8_t rbuffer_out_faked[]    = { 0xFA, 0xF0, 0x45, 0x2A };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 4, 4,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 4 );

    lr11xx_radio_pkt_status_gfsk_t pkt_status;
    lr11xx_status_t                status;

    status = lr11xx_radio_get_gfsk_pkt_status( context, &pkt_status );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_INT16( -125, pkt_status.rssi_sync_in_dbm );
        TEST_ASSERT_EQUAL_INT16( -120, pkt_status.rssi_avg_in_dbm );
        TEST_ASSERT_EQUAL_UINT8( 0x45, pkt_status.rx_len_in_bytes );
        TEST_ASSERT_TRUE( pkt_status.is_addr_err );
        TEST_ASSERT_FALSE( pkt_status.is_crc_err );
        TEST_ASSERT_TRUE( pkt_status.is_len_err );
        TEST_ASSERT_FALSE( pkt_status.is_abort_err );
        TEST_ASSERT_TRUE( pkt_status.is_received );
        TEST_ASSERT_FALSE( pkt_status.is_sent );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_get_packet_status_lora( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x02, 0x04 };
    uint8_t rbuffer_in_expected[3] = { 0x00 };
    uint8_t rbuffer_out_faked[]    = { 0xFA, 0xD6, 0xF0 };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 3, 3,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 3 );

    lr11xx_radio_pkt_status_lora_t pkt_status;
    lr11xx_status_t                status;

    status = lr11xx_radio_get_lora_pkt_status( context, &pkt_status );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_INT16( -125, pkt_status.rssi_pkt_in_dbm );
        TEST_ASSERT_EQUAL_INT8( -10, pkt_status.snr_pkt_in_db );
        TEST_ASSERT_EQUAL_INT16( -120, pkt_status.signal_rssi_pkt_in_dbm );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_get_rssi_inst( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[]     = { 0x02, 0x05 };
    uint8_t rbuffer_in_expected[1] = { 0x00 };
    uint8_t rbuffer_out_faked[]    = { 0xFA };

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    int8_t          rssi = 0;
    lr11xx_status_t status;

    status = lr11xx_radio_get_rssi_inst( context, &rssi );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( -125, rssi );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_gfsk_sync_word( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x06, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 10, 10, NULL, 0, 0, hal_status );

    uint8_t         gfsk_sync_word[] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
    lr11xx_status_t status;

    status = lr11xx_radio_set_gfsk_sync_word( context, gfsk_sync_word );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_gfsk_sync_word_default( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x06, 0x97, 0x23, 0x52, 0x25, 0x56, 0x53, 0x65, 0x64 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 10, 10, NULL, 0, 0, hal_status );

    lr11xx_status_t status;
    const uint8_t   gfsk_sync_word_default[LR11XX_RADIO_GFSK_SYNC_WORD_LENGTH] = LR11XX_RADIO_GFSK_SYNC_WORD_DEFAULT;

    status = lr11xx_radio_set_gfsk_sync_word( context, gfsk_sync_word_default );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_lora_sync_word( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x2B, 0x21 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_set_lora_sync_word( context, 0x21 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_lora_public_network( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x08, 0x01 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_set_lora_public_network( context, LR11XX_RADIO_LORA_NETWORK_PUBLIC );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_rx( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status_1,
                               lr11xx_hal_status_t hal_status_2 )
{
    const uint8_t cbuffer_expected_1[] = { 0x01, 0x0C, 0x00, 0xF3, 0x00, 0x54, 0x40,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cbuffer_expected_2[] = { 0x02, 0x09, 0x00, 0x80, 0x00 };

    do
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_1, 14, 14, NULL, 0, 0, hal_status_1 );
        if( hal_status_1 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }

        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_2, 5, 5, NULL, 0, 0, hal_status_2 );
        if( hal_status_2 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    const lr11xx_status_t status = lr11xx_radio_set_rx( context, 1000 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_rx_with_timeout_in_rtc_step( lr11xx_status_t     status_expected,
                                                        lr11xx_hal_status_t hal_status_1,
                                                        lr11xx_hal_status_t hal_status_2 )
{
    const uint8_t cbuffer_expected_1[] = { 0x01, 0x0C, 0x00, 0xF3, 0x00, 0x54, 0x40,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cbuffer_expected_2[] = { 0x02, 0x09, 0xAB, 0xCD, 0xEF };

    do
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_1, 14, 14, NULL, 0, 0, hal_status_1 );
        if( hal_status_1 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }

        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_2, 5, 5, NULL, 0, 0, hal_status_2 );
        if( hal_status_2 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    const lr11xx_status_t status = lr11xx_radio_set_rx_with_timeout_in_rtc_step( context, 0x89ABCDEF );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_tx( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status_1,
                               lr11xx_hal_status_t hal_status_2 )
{
    const uint8_t cbuffer_expected_1[] = { 0x01, 0x0C, 0x00, 0xF3, 0x00, 0x54, 0x40,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cbuffer_expected_2[] = { 0x02, 0x0A, 0x00, 0xC0, 0x00 };

    do
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_1, 14, 14, NULL, 0, 0, hal_status_1 );
        if( hal_status_1 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }

        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_2, 5, 5, NULL, 0, 0, hal_status_2 );
        if( hal_status_2 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    const lr11xx_status_t status = lr11xx_radio_set_tx( context, 1500 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_tx_with_timeout_in_rtc_step( lr11xx_status_t     status_expected,
                                                        lr11xx_hal_status_t hal_status_1,
                                                        lr11xx_hal_status_t hal_status_2 )
{
    const uint8_t cbuffer_expected_1[] = { 0x01, 0x0C, 0x00, 0xF3, 0x00, 0x54, 0x40,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cbuffer_expected_2[] = { 0x02, 0x0A, 0x56, 0x78, 0x9A };

    do
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_1, 14, 14, NULL, 0, 0, hal_status_1 );
        if( hal_status_1 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }

        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_2, 5, 5, NULL, 0, 0, hal_status_2 );
        if( hal_status_2 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    const lr11xx_status_t status = lr11xx_radio_set_tx_with_timeout_in_rtc_step( context, 0x3456789A );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_rf_frequency( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x0B, 0x34, 0x56, 0x78, 0x9A };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_set_rf_freq( context, 0x3456789A );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_auto_tx_rx( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x0C, 0x23, 0x45, 0x67, 0x02, 0xAB, 0xCD, 0xEF };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 9, 9, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_auto_tx_rx( context, 0x01234567, LR11XX_RADIO_MODE_STANDBY_XOSC, 0x89ABCDEF );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_cad_params( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x0D, 0x08, 0xA5, 0x5A, 0x10, 0x23, 0x45, 0x67 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 9, 9, NULL, 0, 0, hal_status );

    lr11xx_radio_cad_params_t cad_params = { .cad_symb_nb     = 8,
                                             .cad_detect_peak = 0xA5,
                                             .cad_detect_min  = 0x5A,
                                             .cad_exit_mode   = LR11XX_RADIO_CAD_EXIT_MODE_TX,
                                             .cad_timeout     = 0x01234567 };
    lr11xx_status_t           status;

    status = lr11xx_radio_set_cad_params( context, &cad_params );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_packet_type( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x0E, 0x01 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    lr11xx_radio_pkt_type_t pkt_type = LR11XX_RADIO_PKT_TYPE_GFSK;
    lr11xx_status_t         status;

    status = lr11xx_radio_set_pkt_type( context, pkt_type );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_modulation_param_gfsk( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x0F, 0x01, 0x23, 0x45, 0x67, 0x00, 0x0E, 0x89, 0xAB, 0xCD, 0xEF };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 12, 12, NULL, 0, 0, hal_status );

    lr11xx_status_t                status;
    lr11xx_radio_mod_params_gfsk_t modulation_param = {
        .br_in_bps    = 0x01234567,
        .pulse_shape  = LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF,
        .bw_dsb_param = LR11XX_RADIO_GFSK_BW_14600,
        .fdev_in_hz   = 0x89ABCDEF,
    };

    status = lr11xx_radio_set_gfsk_mod_params( context, &modulation_param );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_bpsk_mod_params( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x0F, 0x01, 0x23, 0x45, 0x67, 0x16 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 7, 7, NULL, 0, 0, hal_status );

    lr11xx_status_t                status;
    lr11xx_radio_mod_params_bpsk_t modulation_param = {
        .br_in_bps   = 0x01234567,
        .pulse_shape = LR11XX_RADIO_DBPSK_PULSE_SHAPE,
    };

    status = lr11xx_radio_set_bpsk_mod_params( context, &modulation_param );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_modulation_param_lora( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x0F, 0x0B, 0x04, 0x01, 0x01 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, NULL, 0, 0, hal_status );

    lr11xx_status_t                status;
    lr11xx_radio_mod_params_lora_t modulation_param = {
        .sf   = LR11XX_RADIO_LORA_SF11,
        .bw   = LR11XX_RADIO_LORA_BW_125,
        .cr   = LR11XX_RADIO_LORA_CR_4_5,
        .ldro = 1,
    };

    status = lr11xx_radio_set_lora_mod_params( context, &modulation_param );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_packet_param_gfsk( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x10, 0x45, 0x67, 0x05, 0x76, 0x00, 0x01, 0xA5, 0x04, 0x00 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 11, 11, NULL, 0, 0, hal_status );

    lr11xx_status_t                status;
    lr11xx_radio_pkt_params_gfsk_t packet_param = { .preamble_len_in_bits = 0x4567,
                                                    .preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS,
                                                    .sync_word_len_in_bits = 0x76,
                                                    .address_filtering = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE,
                                                    .header_type       = LR11XX_RADIO_GFSK_PKT_VAR_LEN,
                                                    .pld_len_in_bytes  = 0xA5,
                                                    .crc_type          = LR11XX_RADIO_GFSK_CRC_1_BYTE_INV,
                                                    .dc_free           = LR11XX_RADIO_GFSK_DC_FREE_OFF };

    status = lr11xx_radio_set_gfsk_pkt_params( context, &packet_param );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_packet_param_lora( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x10, 0x00, 0x08, 0x00, 0xFF, 0x01, 0x01 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 8, 8, NULL, 0, 0, hal_status );

    lr11xx_status_t                status;
    lr11xx_radio_pkt_params_lora_t packet_param = { .preamble_len_in_symb = 0x0008,
                                                    .header_type          = LR11XX_RADIO_LORA_PKT_EXPLICIT,
                                                    .pld_len_in_bytes     = 0xFF,
                                                    .crc                  = LR11XX_RADIO_LORA_CRC_ON,
                                                    .iq                   = LR11XX_RADIO_LORA_IQ_INVERTED };

    status = lr11xx_radio_set_lora_pkt_params( context, &packet_param );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_tx_params( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x11, 0x05, 0x0F };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 4, 4, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_set_tx_params( context, 0x05, LR11XX_RADIO_RAMP_304_US );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_packet_address( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x12, 0x45, 0x67 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 4, 4, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_set_pkt_address( context, 0x45, 0x67 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_rx_tx_fallback_mode( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x13, 0x02 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_set_rx_tx_fallback_mode( context, LR11XX_RADIO_FALLBACK_STDBY_XOSC );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_rx_duty_cycle( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x14, 0x00, 0x00, 0x20, 0x00, 0x0C, 0xCC, 0x01 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 9, 9, NULL, 0, 0, hal_status );

    lr11xx_status_t status;
    uint32_t        rx_period    = 1;
    uint32_t        sleep_period = 100;
    uint8_t         mode         = 0x01;

    status = lr11xx_radio_set_rx_duty_cycle( context, rx_period, sleep_period, mode );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_rx_duty_cycle_with_timings_in_rtc_step( lr11xx_status_t     status_expected,
                                                                   lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x14, 0x23, 0x45, 0x67, 0xAB, 0xCD, 0xEF, 0x01 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 9, 9, NULL, 0, 0, hal_status );

    lr11xx_status_t status;
    uint32_t        rx_period    = 0x01234567;
    uint32_t        sleep_period = 0x89ABCDEF;
    uint8_t         mode         = 0x01;

    status = lr11xx_radio_set_rx_duty_cycle_with_timings_in_rtc_step( context, rx_period, sleep_period, mode );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_pa_config( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x15, 0x01, 0x01, 0xAB, 0xCD };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 6, 6, NULL, 0, 0, hal_status );

    lr11xx_status_t       status;
    lr11xx_radio_pa_cfg_t paConfig = {
        .pa_sel        = LR11XX_RADIO_PA_SEL_HP,
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_duty_cycle = 0xAB,
        .pa_hp_sel     = 0xCD,
    };

    status = lr11xx_radio_set_pa_cfg( context, &paConfig );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_StopTimeoutOnPreamble( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x17, 0x01 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_stop_timeout_on_preamble( context, true );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_cad( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status_1,
                                lr11xx_hal_status_t hal_status_2 )
{
    const uint8_t cbuffer_expected_1[] = { 0x01, 0x0C, 0x00, 0xF3, 0x00, 0x54, 0x40,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cbuffer_expected_2[] = { 0x02, 0x18 };

    do
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_1, 14, 14, NULL, 0, 0, hal_status_1 );
        if( hal_status_1 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }

        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_2, 2, 2, NULL, 0, 0, hal_status_2 );
        if( hal_status_2 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    const lr11xx_status_t status = lr11xx_radio_set_cad( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_tx_cw( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x19 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_set_tx_cw( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_OK, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_tx_infinite_preamble( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status_1,
                                                 lr11xx_hal_status_t hal_status_2 )
{
    const uint8_t cbuffer_expected_1[] = { 0x01, 0x0C, 0x00, 0xF3, 0x00, 0x54, 0x40,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cbuffer_expected_2[] = { 0x02, 0x1A };

    do
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_1, 14, 14, NULL, 0, 0, hal_status_1 );
        if( hal_status_1 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }

        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected_2, 2, 2, NULL, 0, 0, hal_status_2 );
        if( hal_status_2 != LR11XX_HAL_STATUS_OK )
        {
            break;
        }
    } while( 0 );

    const lr11xx_status_t status = lr11xx_radio_set_tx_infinite_preamble( context );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 0x00 )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, 0xFF )
void test_lr11xx_radio_set_lora_sync_timeout( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                              uint16_t nb_symbol )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x1B, nb_symbol };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    const lr11xx_status_t status = lr11xx_radio_set_lora_sync_timeout( context, nb_symbol );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 0x0100, 0x08, 0x02 )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, 0x0100, 0x08, 0x02 )
void test_lr11xx_radio_set_lora_sync_timeout_higher_than_255( lr11xx_status_t     status_expected,
                                                              lr11xx_hal_status_t hal_status, uint16_t nb_symbol,
                                                              uint8_t mant, uint8_t exp )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x1B, mant << 3 | exp, 0x01 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 4, 4, NULL, 0, 0, hal_status );

    const lr11xx_status_t status = lr11xx_radio_set_lora_sync_timeout( context, nb_symbol );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

/* This test is implemented to check that the number of symbols computed from the [mantissa, exponent] duple generated
   by lr11xx_radio_convert_nb_symb_to_mant_exp is higher or equal to the requested number of symbol and the difference
   is within a range depending on the requested number of symbols.
*/
void test_lr11xx_radio_convert_nb_symb_to_mant_exp( )
{
    for( int i = 0; i <= 63488; i++ )
    {
        uint8_t mant;
        uint8_t exp;

        const uint16_t nb_symb = lr11xx_radio_convert_nb_symb_to_mant_exp( i, &mant, &exp );

        TEST_ASSERT_GREATER_OR_EQUAL_UINT( i, nb_symb );

        uint16_t max_diff = 0;
        while( ( i >> ( 6 + max_diff ) ) > 0 )
        {
            max_diff++;
        };
        TEST_ASSERT_LESS_OR_EQUAL_UINT( 2 << ( 2 * max_diff + 1 ), nb_symb - i );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, 0x01, 0x02, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, 0x01, 0x08, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, 0x20, 0x02, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, 0x01, 0x02, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_lora_sync_timeout_with_mantissa_exponent( lr11xx_status_t status_expected, uint8_t mantissa,
                                                                     uint8_t exponent, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x1B, mantissa << 3 | exponent, 0x01 };

    if( mantissa < 32 && exponent < 8 )
    {
        lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 4, 4, NULL, 0, 0, hal_status );
    }

    lr11xx_status_t status;

    status = lr11xx_radio_set_lora_sync_timeout_with_mantissa_exponent( context, mantissa, exponent );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_gfsk_crc_params( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x24, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 10, 10, NULL, 0, 0, hal_status );

    lr11xx_status_t status;
    uint32_t        seed       = 0x01234567;
    uint32_t        polynomial = 0x89ABCDEF;

    status = lr11xx_radio_set_gfsk_crc_params( context, seed, polynomial );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_gfsk_whitening_params( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x25, 0x01, 0x23 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 4, 4, NULL, 0, 0, hal_status );

    lr11xx_status_t status;
    uint32_t        whitening = 0x0123;

    status = lr11xx_radio_set_gfsk_whitening_seed( context, whitening );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 1 )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, 0 )
void test_lr11xx_radio_set_rx_boosted( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                       uint8_t enable_boost_mode_uint )
{
    const bool enable_boost_mode = ( enable_boost_mode_uint == 1 );

    uint8_t cbuffer_expected[] = { 0x02, 0x27, ( enable_boost_mode == true ) ? 0x01 : 0x00 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_cfg_rx_boosted( context, enable_boost_mode );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_set_rssi_calibration( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    lr11xx_radio_rssi_calibration_table_t rssi_cal_table = {
        .gain_tune.g4     = 7,
        .gain_tune.g5     = 6,
        .gain_tune.g6     = 5,
        .gain_tune.g7     = 4,
        .gain_tune.g8     = 3,
        .gain_tune.g9     = 2,
        .gain_tune.g10    = 1,
        .gain_tune.g11    = 0,
        .gain_tune.g12    = 7,
        .gain_tune.g13    = 6,
        .gain_tune.g13hp1 = 5,
        .gain_tune.g13hp2 = 4,
        .gain_tune.g13hp3 = 3,
        .gain_tune.g13hp4 = 2,
        .gain_tune.g13hp5 = 1,
        .gain_tune.g13hp6 = 0,
        .gain_tune.g13hp7 = 7,
        .gain_offset      = 0x1234,
    };

    uint8_t cbuffer_expected[] = { 0x02, 0x29, 0x67, 0x45, 0x23, 0x01, 0x67, 0x45, 0x23, 0x01, 0x07, 0x12, 0x34 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 13, 13, NULL, 0, 0, hal_status );

    lr11xx_status_t status;

    status = lr11xx_radio_set_rssi_calibration( context, &rssi_cal_table );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, 1, LR11XX_RADIO_GFSK_BW_4800 )
TEST_VALUE( LR11XX_STATUS_OK, 4800, LR11XX_RADIO_GFSK_BW_4800 )
TEST_VALUE( LR11XX_STATUS_OK, 4801, LR11XX_RADIO_GFSK_BW_5800 )
TEST_VALUE( LR11XX_STATUS_OK, 5800, LR11XX_RADIO_GFSK_BW_5800 )
TEST_VALUE( LR11XX_STATUS_OK, 5801, LR11XX_RADIO_GFSK_BW_7300 )
TEST_VALUE( LR11XX_STATUS_OK, 7300, LR11XX_RADIO_GFSK_BW_7300 )
TEST_VALUE( LR11XX_STATUS_OK, 7301, LR11XX_RADIO_GFSK_BW_9700 )
TEST_VALUE( LR11XX_STATUS_OK, 9700, LR11XX_RADIO_GFSK_BW_9700 )
TEST_VALUE( LR11XX_STATUS_OK, 9701, LR11XX_RADIO_GFSK_BW_11700 )
TEST_VALUE( LR11XX_STATUS_OK, 11700, LR11XX_RADIO_GFSK_BW_11700 )
TEST_VALUE( LR11XX_STATUS_OK, 11701, LR11XX_RADIO_GFSK_BW_14600 )
TEST_VALUE( LR11XX_STATUS_OK, 14600, LR11XX_RADIO_GFSK_BW_14600 )
TEST_VALUE( LR11XX_STATUS_OK, 14601, LR11XX_RADIO_GFSK_BW_19500 )
TEST_VALUE( LR11XX_STATUS_OK, 19500, LR11XX_RADIO_GFSK_BW_19500 )
TEST_VALUE( LR11XX_STATUS_OK, 19501, LR11XX_RADIO_GFSK_BW_23400 )
TEST_VALUE( LR11XX_STATUS_OK, 23400, LR11XX_RADIO_GFSK_BW_23400 )
TEST_VALUE( LR11XX_STATUS_OK, 23401, LR11XX_RADIO_GFSK_BW_29300 )
TEST_VALUE( LR11XX_STATUS_OK, 29300, LR11XX_RADIO_GFSK_BW_29300 )
TEST_VALUE( LR11XX_STATUS_OK, 29301, LR11XX_RADIO_GFSK_BW_39000 )
TEST_VALUE( LR11XX_STATUS_OK, 39000, LR11XX_RADIO_GFSK_BW_39000 )
TEST_VALUE( LR11XX_STATUS_OK, 39001, LR11XX_RADIO_GFSK_BW_46900 )
TEST_VALUE( LR11XX_STATUS_OK, 46900, LR11XX_RADIO_GFSK_BW_46900 )
TEST_VALUE( LR11XX_STATUS_OK, 46901, LR11XX_RADIO_GFSK_BW_58600 )
TEST_VALUE( LR11XX_STATUS_OK, 58600, LR11XX_RADIO_GFSK_BW_58600 )
TEST_VALUE( LR11XX_STATUS_OK, 58601, LR11XX_RADIO_GFSK_BW_78200 )
TEST_VALUE( LR11XX_STATUS_OK, 78200, LR11XX_RADIO_GFSK_BW_78200 )
TEST_VALUE( LR11XX_STATUS_OK, 78201, LR11XX_RADIO_GFSK_BW_93800 )
TEST_VALUE( LR11XX_STATUS_OK, 93800, LR11XX_RADIO_GFSK_BW_93800 )
TEST_VALUE( LR11XX_STATUS_OK, 93801, LR11XX_RADIO_GFSK_BW_117300 )
TEST_VALUE( LR11XX_STATUS_OK, 117300, LR11XX_RADIO_GFSK_BW_117300 )
TEST_VALUE( LR11XX_STATUS_OK, 117301, LR11XX_RADIO_GFSK_BW_156200 )
TEST_VALUE( LR11XX_STATUS_OK, 156200, LR11XX_RADIO_GFSK_BW_156200 )
TEST_VALUE( LR11XX_STATUS_OK, 156201, LR11XX_RADIO_GFSK_BW_187200 )
TEST_VALUE( LR11XX_STATUS_OK, 187200, LR11XX_RADIO_GFSK_BW_187200 )
TEST_VALUE( LR11XX_STATUS_OK, 187201, LR11XX_RADIO_GFSK_BW_234300 )
TEST_VALUE( LR11XX_STATUS_OK, 234300, LR11XX_RADIO_GFSK_BW_234300 )
TEST_VALUE( LR11XX_STATUS_OK, 234301, LR11XX_RADIO_GFSK_BW_312000 )
TEST_VALUE( LR11XX_STATUS_OK, 312000, LR11XX_RADIO_GFSK_BW_312000 )
TEST_VALUE( LR11XX_STATUS_OK, 312001, LR11XX_RADIO_GFSK_BW_373600 )
TEST_VALUE( LR11XX_STATUS_OK, 373600, LR11XX_RADIO_GFSK_BW_373600 )
TEST_VALUE( LR11XX_STATUS_OK, 373601, LR11XX_RADIO_GFSK_BW_467000 )
TEST_VALUE( LR11XX_STATUS_OK, 467000, LR11XX_RADIO_GFSK_BW_467000 )
TEST_VALUE( LR11XX_STATUS_ERROR, 468000, LR11XX_RADIO_GFSK_BW_467000 )
void test_lr11xx_radio_get_gfsk_rx_bandwidth( lr11xx_status_t status_expected, uint32_t bw_in_hz,
                                              lr11xx_radio_gfsk_bw_t bw_param_expected )
{
    lr11xx_radio_gfsk_bw_t bw_param;
    lr11xx_status_t        status;

    status = lr11xx_radio_get_gfsk_rx_bandwidth( bw_in_hz, &bw_param );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( ( uint8_t ) bw_param_expected, bw_param );
    }
}

TEST_VALUE( LR11XX_RADIO_LORA_BW_10, 10417 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_15, 15625 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_20, 20833 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_31, 31250 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_41, 41667 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_62, 62500 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_125, 125000 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_250, 250000 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_500, 500000 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_200, 203000 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_400, 406000 )
TEST_VALUE( LR11XX_RADIO_LORA_BW_800, 812000 )
void test_lr11xx_radio_get_lora_bandwidth_in_hz( lr11xx_radio_lora_bw_t bw_param, uint32_t bw_in_hz_expected )
{
    uint32_t bw_in_hz;

    bw_in_hz = lr11xx_radio_get_lora_bw_in_hz( bw_param );

    TEST_ASSERT_EQUAL_UINT32( bw_in_hz_expected, bw_in_hz );
}

void test_lr11xx_radio_get_lora_time_on_air_in_ms( void )
{
    uint32_t                       time_on_air_in_ms = 0;
    lr11xx_radio_pkt_params_lora_t pkt_p;
    lr11xx_radio_mod_params_lora_t mod_p;
    uint32_t                       size = sizeof( assets ) / sizeof( assets[0] );

    for( uint32_t i = 0; i < size; i++ )
    {
        pkt_p.preamble_len_in_symb = assets[i].pbl;
        pkt_p.header_type = ( assets[i].impl == 1 ) ? LR11XX_RADIO_LORA_PKT_IMPLICIT : LR11XX_RADIO_LORA_PKT_EXPLICIT;
        pkt_p.pld_len_in_bytes = assets[i].pld;
        pkt_p.crc              = ( assets[i].crc == 1 ) ? LR11XX_RADIO_LORA_CRC_ON : LR11XX_RADIO_LORA_CRC_OFF;
        pkt_p.iq               = LR11XX_RADIO_LORA_IQ_STANDARD;

        switch( assets[i].bw )
        {
        case 10417:
            mod_p.bw = LR11XX_RADIO_LORA_BW_10;
            break;
        case 15625:
            mod_p.bw = LR11XX_RADIO_LORA_BW_15;
            break;
        case 20833:
            mod_p.bw = LR11XX_RADIO_LORA_BW_20;
            break;
        case 31250:
            mod_p.bw = LR11XX_RADIO_LORA_BW_31;
            break;
        case 41667:
            mod_p.bw = LR11XX_RADIO_LORA_BW_41;
            break;
        case 62500:
            mod_p.bw = LR11XX_RADIO_LORA_BW_62;
            break;
        case 125000:
            mod_p.bw = LR11XX_RADIO_LORA_BW_125;
            break;
        case 250000:
            mod_p.bw = LR11XX_RADIO_LORA_BW_250;
            break;
        case 500000:
            mod_p.bw = LR11XX_RADIO_LORA_BW_500;
            break;
        case 203000:
            mod_p.bw = LR11XX_RADIO_LORA_BW_200;
            break;
        case 406000:
            mod_p.bw = LR11XX_RADIO_LORA_BW_400;
            break;
        case 812000:
            mod_p.bw = LR11XX_RADIO_LORA_BW_800;
            break;
        }
        mod_p.sf   = ( lr11xx_radio_lora_sf_t ) ( assets[i].sf );
        mod_p.cr   = ( lr11xx_radio_lora_cr_t ) ( assets[i].cr );
        mod_p.ldro = assets[i].ldro;

        time_on_air_in_ms = lr11xx_radio_get_lora_time_on_air_in_ms( &pkt_p, &mod_p );

        uint32_t time_on_air_in_ms_expected = CEILING( assets[i].toa * 1000 );

        TEST_ASSERT_UINT32_WITHIN( 1, time_on_air_in_ms_expected, time_on_air_in_ms );
    }
}

// NOTE: Same value due to the rounding to ms.
TEST_VALUE( LR11XX_RADIO_GFSK_PKT_FIX_LEN, LR11XX_RADIO_GFSK_CRC_OFF, 42 )
TEST_VALUE( LR11XX_RADIO_GFSK_PKT_VAR_LEN, LR11XX_RADIO_GFSK_CRC_1_BYTE, 42 )
TEST_VALUE( LR11XX_RADIO_GFSK_PKT_FIX_LEN, LR11XX_RADIO_GFSK_CRC_2_BYTES, 42 )
TEST_VALUE( LR11XX_RADIO_GFSK_PKT_VAR_LEN, LR11XX_RADIO_GFSK_CRC_1_BYTE_INV, 42 )
TEST_VALUE( LR11XX_RADIO_GFSK_PKT_VAR_LEN, LR11XX_RADIO_GFSK_CRC_2_BYTES_INV, 43 )
TEST_VALUE( LR11XX_RADIO_GFSK_PKT_VAR_LEN, 255, 42 )
void test_lr11xx_radio_get_gfsk_time_on_air_in_ms( lr11xx_radio_gfsk_pkt_len_modes_t header_type,
                                                   lr11xx_radio_gfsk_crc_type_t      crc_type,
                                                   uint32_t                          time_on_air_in_ms_expected )
{
    uint32_t                       time_on_air_in_ms = 0;
    lr11xx_radio_pkt_params_gfsk_t pkt_p             = {
                    .preamble_len_in_bits  = 40,
                    .preamble_detector     = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
                    .sync_word_len_in_bits = 24,
                    .address_filtering     = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE,
                    .header_type           = header_type,
                    .pld_len_in_bytes      = 252,
                    .crc_type              = crc_type,
                    .dc_free               = LR11XX_RADIO_GFSK_DC_FREE_WHITENING,
    };
    lr11xx_radio_mod_params_gfsk_t mod_p = {
        .br_in_bps    = 50000,
        .fdev_in_hz   = 25000,
        .pulse_shape  = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1,
        .bw_dsb_param = LR11XX_RADIO_GFSK_BW_117300,
    };

    time_on_air_in_ms = lr11xx_radio_get_gfsk_time_on_air_in_ms( &pkt_p, &mod_p );

    TEST_ASSERT_EQUAL_UINT32( time_on_air_in_ms_expected, time_on_air_in_ms );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_cfg_ble_beacon( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x2E, 0x25 };
    uint8_t dbuffer_expected[] = { 0x01, 0x23, 0x45 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, dbuffer_expected, 3, 3, hal_status );

    lr11xx_status_t status;
    uint8_t         payload[3] = { 0x01, 0x23, 0x45 };

    status = lr11xx_radio_cfg_ble_beacon( context, 37, payload, 3 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 0x01, 0x00, LR11XX_RADIO_LORA_CR_4_5 )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 0x12, 0x01, LR11XX_RADIO_LORA_CR_4_6 )
TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK, 0x14, 0x01, LR11XX_RADIO_LORA_CR_4_8 )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR, 0x01, 0x00, LR11XX_RADIO_LORA_CR_4_5 )
void test_lr11xx_radio_get_lora_rx_info( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status,
                                         uint8_t output, uint8_t is_crc_present_expected_uint,
                                         lr11xx_radio_lora_cr_t cr_expected )
{
    uint8_t cbuffer_expected[]     = { 0x02, 0x30 };
    uint8_t rbuffer_in_expected[1] = { 0 };
    uint8_t rbuffer_out_faked[]    = { output };

    bool is_crc_present_expected = ( is_crc_present_expected_uint != 0 ) ? true : false;

    lr11xx_hal_read_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 2, 2, rbuffer_in_expected, 1, 1,
                                              hal_status );
    lr11xx_hal_read_IgnoreArg_data( );
    lr11xx_hal_read_ReturnArrayThruPtr_data( rbuffer_out_faked, 1 );

    bool                   is_crc_present;
    lr11xx_radio_lora_cr_t cr;

    lr11xx_status_t status = lr11xx_radio_get_lora_rx_info( context, &is_crc_present, &cr );

    TEST_ASSERT_EQUAL_INT( status_expected, status );

    if( status_expected == LR11XX_STATUS_OK )
    {
        TEST_ASSERT_EQUAL_UINT8( is_crc_present_expected, is_crc_present );
        TEST_ASSERT_EQUAL_UINT8( cr_expected, cr );
    }
}

TEST_VALUE( LR11XX_STATUS_OK, LR11XX_HAL_STATUS_OK )
TEST_VALUE( LR11XX_STATUS_ERROR, LR11XX_HAL_STATUS_ERROR )
void test_lr11xx_radio_cfg_and_send_ble_beacon( lr11xx_status_t status_expected, lr11xx_hal_status_t hal_status )
{
    const uint8_t cbuffer_expected[] = { 0x02, 0x31, 0x25 };
    const uint8_t dbuffer_expected[] = { 0x01, 0x23, 0x45 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( context, 0, cbuffer_expected, 3, 3, dbuffer_expected, 3, 3, hal_status );

    lr11xx_status_t status;
    const uint8_t payload[3] = { 0x01, 0x23, 0x45 };

    status = lr11xx_radio_cfg_and_send_ble_beacon( context, 37, payload, 3 );

    TEST_ASSERT_EQUAL_INT( status_expected, status );
}
