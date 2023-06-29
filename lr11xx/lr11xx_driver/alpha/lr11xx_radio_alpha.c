#include "lr11xx_radio_alpha.h"
#include "lr11xx_hal.h"

#define LR11XX_RADIO_SET_MODULATION_PARAMS_BPSK_CMD_LENGTH ( 2 + 5 )
#define LR11XX_RADIO_SET_MODULATION_PARAMS_GMSK_CMD_LENGTH ( 2 + 5 )
#define LR11XX_RADIO_SET_PACKET_PARAM_BPSK_CMD_LENGTH ( 2 + 7 )
#define LR11XX_RADIO_SET_PACKET_PARAM_GMSK_CMD_LENGTH ( 2 + 9 )
#define LR11XX_RADIO_SET_RF_FE_TEST_CMD_LENGTH ( 2 + 2 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

enum
{
    LR11XX_RADIO_SET_MODULATION_PARAM_OC = 0x020F,
    LR11XX_RADIO_SET_PKT_PARAM_OC        = 0x0210,
    LR11XX_RADIO_SET_RF_FE_TEST_OC       = 0x0222,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr11xx_radio_set_modulation_param_bpsk( const void* radio, const lr11xx_radio_modulation_param_bpsk_t* mod_params )
{
    uint8_t cbuffer[LR11XX_RADIO_SET_MODULATION_PARAMS_BPSK_CMD_LENGTH];

    cbuffer[0] = ( uint8_t ) ( LR11XX_RADIO_SET_MODULATION_PARAM_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_RADIO_SET_MODULATION_PARAM_OC >> 0 );

    cbuffer[2] = ( uint8_t ) ( mod_params->bitrate >> 24 );
    cbuffer[3] = ( uint8_t ) ( mod_params->bitrate >> 16 );
    cbuffer[4] = ( uint8_t ) ( mod_params->bitrate >> 8 );
    cbuffer[5] = ( uint8_t ) ( mod_params->bitrate >> 0 );

    cbuffer[6] = ( uint8_t ) mod_params->pulse_shape;

    lr11xx_hal_write( radio, cbuffer, LR11XX_RADIO_SET_MODULATION_PARAMS_BPSK_CMD_LENGTH, 0, 0 );
}

void lr11xx_radio_set_modulation_param_gmsk( const void* radio, const lr11xx_radio_modulation_param_gmsk_t* mod_params )
{
    uint8_t cbuffer[LR11XX_RADIO_SET_MODULATION_PARAMS_GMSK_CMD_LENGTH];

    cbuffer[0] = ( uint8_t ) ( LR11XX_RADIO_SET_MODULATION_PARAM_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_RADIO_SET_MODULATION_PARAM_OC >> 0 );

    cbuffer[2] = ( uint8_t ) ( mod_params->bitrate >> 24 );
    cbuffer[3] = ( uint8_t ) ( mod_params->bitrate >> 16 );
    cbuffer[4] = ( uint8_t ) ( mod_params->bitrate >> 8 );
    cbuffer[5] = ( uint8_t ) ( mod_params->bitrate >> 0 );

    cbuffer[6] = ( uint8_t ) mod_params->pulse_shape;

    lr11xx_hal_write( radio, cbuffer, LR11XX_RADIO_SET_MODULATION_PARAMS_GMSK_CMD_LENGTH, 0, 0 );
}

void lr11xx_radio_set_packet_param_bpsk( const void* radio, const lr11xx_radio_packet_param_bpsk_t* pkt_params )
{
    uint8_t cbuffer[LR11XX_RADIO_SET_PACKET_PARAM_BPSK_CMD_LENGTH];

    cbuffer[0] = ( uint8_t ) ( LR11XX_RADIO_SET_PKT_PARAM_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_RADIO_SET_PKT_PARAM_OC >> 0 );

    cbuffer[2] = ( uint8_t ) pkt_params->payload_length;

    cbuffer[3] = ( uint8_t ) ( pkt_params->ramp_up_delay >> 8 );
    cbuffer[4] = ( uint8_t ) ( pkt_params->ramp_up_delay >> 0 );

    cbuffer[5] = ( uint8_t ) ( pkt_params->ramp_down_delay >> 8 );
    cbuffer[6] = ( uint8_t ) ( pkt_params->ramp_down_delay >> 0 );

    cbuffer[7] = ( uint8_t ) ( pkt_params->bit_num >> 8 );
    cbuffer[8] = ( uint8_t ) ( pkt_params->bit_num >> 0 );

    lr11xx_hal_write( radio, cbuffer, LR11XX_RADIO_SET_PACKET_PARAM_BPSK_CMD_LENGTH, 0, 0 );
}

void lr11xx_radio_set_packet_param_gmsk( const void* radio, const lr11xx_radio_packet_param_gmsk_t* pkt_params )
{
    uint8_t cbuffer[LR11XX_RADIO_SET_PACKET_PARAM_GMSK_CMD_LENGTH];

    cbuffer[0] = ( uint8_t ) ( LR11XX_RADIO_SET_PKT_PARAM_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_RADIO_SET_PKT_PARAM_OC >> 0 );

    cbuffer[2] = ( uint8_t ) ( pkt_params->preamble_len_in_bits >> 8 );
    cbuffer[3] = ( uint8_t ) ( pkt_params->preamble_len_in_bits >> 0 );

    cbuffer[4] = ( uint8_t ) ( pkt_params->preamble_detector );

    cbuffer[5] = pkt_params->sync_word_len_in_bits;

    cbuffer[6] = ( uint8_t ) ( pkt_params->address_filtering );

    cbuffer[7] = ( uint8_t ) ( pkt_params->header_type );

    cbuffer[8] = pkt_params->pld_len_in_bytes;

    cbuffer[9] = ( uint8_t ) ( pkt_params->crc_type );

    cbuffer[10] = ( uint8_t ) ( pkt_params->dc_free );

    lr11xx_hal_write( radio, cbuffer, LR11XX_RADIO_SET_PACKET_PARAM_GMSK_CMD_LENGTH, 0, 0 );
}

void lr11xx_radio_set_rf_fe_test( const void* radio, const lr11xx_radio_rf_fe_test_case_t test_case,
                                  uint8_t wifi_channel )
{
    const uint8_t cbuffer[LR11XX_RADIO_SET_RF_FE_TEST_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_RADIO_SET_RF_FE_TEST_OC >> 8 ),
        ( uint8_t ) ( LR11XX_RADIO_SET_RF_FE_TEST_OC >> 0 ),
        ( uint8_t ) test_case,
        wifi_channel,
    };

    lr11xx_hal_write( radio, cbuffer, LR11XX_RADIO_SET_RF_FE_TEST_CMD_LENGTH, 0, 0 );
}