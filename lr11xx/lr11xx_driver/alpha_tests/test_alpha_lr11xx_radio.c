#include "lr11xx_radio_alpha.h"
#include "unity.h"

#include <string.h>  // for memset

#include "mock_lr11xx_hal.h"

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

void test_lr11xx_radio_set_modulation_param_bpsk( void )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x0F, 0x23, 0x45, 0x67, 0x89, 0x0A };

    lr11xx_radio_modulation_param_bpsk_t modulation_param = {
        .bitrate     = 0x23456789,
        .pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_07,
    };

    lr11xx_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 7, 7, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_radio_set_modulation_param_bpsk( radio, &modulation_param );
}

void test_lr11xx_radio_set_modulation_param_gmsk( void )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x0F, 0x23, 0x45, 0x67, 0x89, 0x0A };

    lr11xx_radio_modulation_param_gmsk_t modulation_param = {
        .bitrate     = 0x23456789,
        .pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_07,
    };

    lr11xx_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 7, 7, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_radio_set_modulation_param_gmsk( radio, &modulation_param );
}

void test_lr11xx_radio_set_packet_param_bpsk( void )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x10, 0x5A, 0x13, 0x06, 0x04, 0xE1, 0xAB, 0xCD };

    lr11xx_radio_packet_param_bpsk_t packet_param = {
        .payload_length  = 0x5A,
        .ramp_up_delay   = LR11XX_RADIO_BPSK_RAMP_UP_DELAY_100BPS,
        .ramp_down_delay = LR11XX_RADIO_BPSK_RAMP_DOWN_DELAY_600BPS,
        .bit_num         = 0xABCD,
    };

    lr11xx_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 9, 9, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_radio_set_packet_param_bpsk( radio, &packet_param );
}

void test_lr11xx_radio_set_packet_param_gmsk( void )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x10, 0x45, 0x67, 0x05, 0x76, 0x00, 0x01, 0xA5, 0x04, 0x00 };

    lr11xx_radio_packet_param_gmsk_t packet_param = { .preamble_len_in_bits = 0x4567,
                                                      .preamble_detector =
                                                          LR11XX_RADIO_GMSK_PREAMBLE_DETECTOR_LENGTH_16BITS,
                                                      .sync_word_len_in_bits = 0x76,
                                                      .address_filtering = LR11XX_RADIO_GMSK_ADDRESS_FILTERING_DISABLE,
                                                      .header_type       = LR11XX_RADIO_GMSK_HEADER_TYPE_EXPLICIT,
                                                      .pld_len_in_bytes  = 0xA5,
                                                      .crc_type          = LR11XX_RADIO_GMSK_CRC_1BYTE_INV,
                                                      .dc_free           = LR11XX_RADIO_GMSK_DCFREE_OFF };

    lr11xx_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 11, 11, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_radio_set_packet_param_gmsk( radio, &packet_param );
}

void test_lr11xx_radio_set_rf_fe_test( void )
{
    uint8_t cbuffer_expected[] = { 0x02, 0x22, 0x05, 0x04 };

    lr11xx_hal_write_ExpectWithArrayAndReturn( radio, 0, cbuffer_expected, 4, 4, NULL, 0, 0, LR11XX_HAL_STATUS_OK );

    lr11xx_radio_set_rf_fe_test( radio, LR11XX_RF_FE_TEST_CASE_WIFI_CAPTURE_6_MHZ, 0x04 );
}
