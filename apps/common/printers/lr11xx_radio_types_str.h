#ifndef LR11XX_RADIO_TYPES_STR_H
#define LR11XX_RADIO_TYPES_STR_H
#include "lr11xx_radio_types.h"
#ifdef __cplusplus
extern "C" {
#endif
const char* lr11xx_radio_pa_selection_to_str( const lr11xx_radio_pa_selection_t value );
const char* lr11xx_radio_gfsk_address_filtering_to_str( const lr11xx_radio_gfsk_address_filtering_t value );
const char* lr11xx_radio_fallback_modes_to_str( const lr11xx_radio_fallback_modes_t value );
const char* lr11xx_radio_ramp_time_to_str( const lr11xx_radio_ramp_time_t value );
const char* lr11xx_radio_lora_network_type_to_str( const lr11xx_radio_lora_network_type_t value );
const char* lr11xx_radio_lora_sf_to_str( const lr11xx_radio_lora_sf_t value );
const char* lr11xx_radio_lora_bw_to_str( const lr11xx_radio_lora_bw_t value );
const char* lr11xx_radio_lora_cr_to_str( const lr11xx_radio_lora_cr_t value );
const char* lr11xx_radio_intermediary_mode_to_str( const lr11xx_radio_intermediary_mode_t value );
const char* lr11xx_radio_gfsk_crc_type_to_str( const lr11xx_radio_gfsk_crc_type_t value );
const char* lr11xx_radio_gfsk_dc_free_to_str( const lr11xx_radio_gfsk_dc_free_t value );
const char* lr11xx_radio_gfsk_pkt_len_modes_to_str( const lr11xx_radio_gfsk_pkt_len_modes_t value );
const char* lr11xx_radio_gfsk_preamble_detector_to_str( const lr11xx_radio_gfsk_preamble_detector_t value );
const char* lr11xx_radio_lora_crc_to_str( const lr11xx_radio_lora_crc_t value );
const char* lr11xx_radio_lora_pkt_len_modes_to_str( const lr11xx_radio_lora_pkt_len_modes_t value );
const char* lr11xx_radio_lora_iq_to_str( const lr11xx_radio_lora_iq_t value );
const char* lr11xx_radio_pkt_type_to_str( const lr11xx_radio_pkt_type_t value );
const char* lr11xx_radio_pa_reg_supply_to_str( const lr11xx_radio_pa_reg_supply_t value );
const char* lr11xx_radio_rx_duty_cycle_mode_to_str( const lr11xx_radio_rx_duty_cycle_mode_t value );
const char* lr11xx_radio_gfsk_bw_to_str( const lr11xx_radio_gfsk_bw_t value );
const char* lr11xx_radio_cad_exit_mode_to_str( const lr11xx_radio_cad_exit_mode_t value );
const char* lr11xx_radio_gfsk_pulse_shape_to_str( const lr11xx_radio_gfsk_pulse_shape_t value );
#ifdef __cplusplus
}
#endif
#endif  // LR11XX_RADIO_TYPES_STR_H
