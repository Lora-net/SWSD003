#ifndef SX126X_STR_H
#define SX126X_STR_H
#include "sx126x.h"
#ifdef __cplusplus
extern "C" {
#endif
const char *sx126x_status_to_str(const sx126x_status_t value);
const char *sx126x_sleep_cfgs_to_str(const sx126x_sleep_cfgs_t value);
const char *sx126x_standby_cfgs_to_str(const sx126x_standby_cfgs_t value);
const char *sx126x_reg_mod_to_str(const sx126x_reg_mod_t value);
const char *sx126x_fallback_modes_to_str(const sx126x_fallback_modes_t value);
const char *sx126x_tcxo_ctrl_voltages_to_str(const sx126x_tcxo_ctrl_voltages_t value);
const char *sx126x_pkt_type_to_str(const sx126x_pkt_type_t value);
const char *sx126x_ramp_time_to_str(const sx126x_ramp_time_t value);
const char *sx126x_gfsk_pulse_shape_to_str(const sx126x_gfsk_pulse_shape_t value);
const char *sx126x_gfsk_bw_to_str(const sx126x_gfsk_bw_t value);
const char *sx126x_lora_sf_to_str(const sx126x_lora_sf_t value);
const char *sx126x_lora_bw_to_str(const sx126x_lora_bw_t value);
const char *sx126x_lora_cr_to_str(const sx126x_lora_cr_t value);
const char *sx126x_gfsk_preamble_detector_to_str(const sx126x_gfsk_preamble_detector_t value);
const char *sx126x_gfsk_address_filtering_to_str(const sx126x_gfsk_address_filtering_t value);
const char *sx126x_gfsk_pkt_len_modes_to_str(const sx126x_gfsk_pkt_len_modes_t value);
const char *sx126x_gfsk_crc_types_to_str(const sx126x_gfsk_crc_types_t value);
const char *sx126x_gfsk_dc_free_to_str(const sx126x_gfsk_dc_free_t value);
const char *sx126x_lora_pkt_len_modes_to_str(const sx126x_lora_pkt_len_modes_t value);
const char *sx126x_cad_symbs_to_str(const sx126x_cad_symbs_t value);
const char *sx126x_cad_exit_modes_to_str(const sx126x_cad_exit_modes_t value);
const char *sx126x_chip_modes_to_str(const sx126x_chip_modes_t value);
const char *sx126x_cmd_status_to_str(const sx126x_cmd_status_t value);
#ifdef __cplusplus
}
#endif
#endif // SX126X_STR_H
