/*!
 * @file      sx126x_str.c
 *
 * @brief     Printer helper functions for SX126x types
 *
 * @copyright
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sx126x_str.h"

const char* sx126x_status_to_str( const sx126x_status_t value )
{
    switch( value )
    {
    case SX126X_STATUS_OK:
    {
        return ( const char* ) "SX126X_STATUS_OK";
    }

    case SX126X_STATUS_UNSUPPORTED_FEATURE:
    {
        return ( const char* ) "SX126X_STATUS_UNSUPPORTED_FEATURE";
    }

    case SX126X_STATUS_UNKNOWN_VALUE:
    {
        return ( const char* ) "SX126X_STATUS_UNKNOWN_VALUE";
    }

    case SX126X_STATUS_ERROR:
    {
        return ( const char* ) "SX126X_STATUS_ERROR";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_sleep_cfgs_to_str( const sx126x_sleep_cfgs_t value )
{
    switch( value )
    {
    case SX126X_SLEEP_CFG_COLD_START:
    {
        return ( const char* ) "SX126X_SLEEP_CFG_COLD_START";
    }

    case SX126X_SLEEP_CFG_WARM_START:
    {
        return ( const char* ) "SX126X_SLEEP_CFG_WARM_START";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_standby_cfgs_to_str( const sx126x_standby_cfgs_t value )
{
    switch( value )
    {
    case SX126X_STANDBY_CFG_RC:
    {
        return ( const char* ) "SX126X_STANDBY_CFG_RC";
    }

    case SX126X_STANDBY_CFG_XOSC:
    {
        return ( const char* ) "SX126X_STANDBY_CFG_XOSC";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_reg_mod_to_str( const sx126x_reg_mod_t value )
{
    switch( value )
    {
    case SX126X_REG_MODE_LDO:
    {
        return ( const char* ) "SX126X_REG_MODE_LDO";
    }

    case SX126X_REG_MODE_DCDC:
    {
        return ( const char* ) "SX126X_REG_MODE_DCDC";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_fallback_modes_to_str( const sx126x_fallback_modes_t value )
{
    switch( value )
    {
    case SX126X_FALLBACK_STDBY_RC:
    {
        return ( const char* ) "SX126X_FALLBACK_STDBY_RC";
    }

    case SX126X_FALLBACK_STDBY_XOSC:
    {
        return ( const char* ) "SX126X_FALLBACK_STDBY_XOSC";
    }

    case SX126X_FALLBACK_FS:
    {
        return ( const char* ) "SX126X_FALLBACK_FS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_tcxo_ctrl_voltages_to_str( const sx126x_tcxo_ctrl_voltages_t value )
{
    switch( value )
    {
    case SX126X_TCXO_CTRL_1_6V:
    {
        return ( const char* ) "SX126X_TCXO_CTRL_1_6V";
    }

    case SX126X_TCXO_CTRL_1_7V:
    {
        return ( const char* ) "SX126X_TCXO_CTRL_1_7V";
    }

    case SX126X_TCXO_CTRL_1_8V:
    {
        return ( const char* ) "SX126X_TCXO_CTRL_1_8V";
    }

    case SX126X_TCXO_CTRL_2_2V:
    {
        return ( const char* ) "SX126X_TCXO_CTRL_2_2V";
    }

    case SX126X_TCXO_CTRL_2_4V:
    {
        return ( const char* ) "SX126X_TCXO_CTRL_2_4V";
    }

    case SX126X_TCXO_CTRL_2_7V:
    {
        return ( const char* ) "SX126X_TCXO_CTRL_2_7V";
    }

    case SX126X_TCXO_CTRL_3_0V:
    {
        return ( const char* ) "SX126X_TCXO_CTRL_3_0V";
    }

    case SX126X_TCXO_CTRL_3_3V:
    {
        return ( const char* ) "SX126X_TCXO_CTRL_3_3V";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_pkt_type_to_str( const sx126x_pkt_type_t value )
{
    switch( value )
    {
    case SX126X_PKT_TYPE_GFSK:
    {
        return ( const char* ) "SX126X_PKT_TYPE_GFSK";
    }

    case SX126X_PKT_TYPE_LORA:
    {
        return ( const char* ) "SX126X_PKT_TYPE_LORA";
    }

    case SX126X_PKT_TYPE_BPSK:
    {
        return ( const char* ) "SX126X_PKT_TYPE_BPSK";
    }

    case SX126X_PKT_TYPE_LR_FHSS:
    {
        return ( const char* ) "SX126X_PKT_TYPE_LR_FHSS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_ramp_time_to_str( const sx126x_ramp_time_t value )
{
    switch( value )
    {
    case SX126X_RAMP_10_US:
    {
        return ( const char* ) "SX126X_RAMP_10_US";
    }

    case SX126X_RAMP_20_US:
    {
        return ( const char* ) "SX126X_RAMP_20_US";
    }

    case SX126X_RAMP_40_US:
    {
        return ( const char* ) "SX126X_RAMP_40_US";
    }

    case SX126X_RAMP_80_US:
    {
        return ( const char* ) "SX126X_RAMP_80_US";
    }

    case SX126X_RAMP_200_US:
    {
        return ( const char* ) "SX126X_RAMP_200_US";
    }

    case SX126X_RAMP_800_US:
    {
        return ( const char* ) "SX126X_RAMP_800_US";
    }

    case SX126X_RAMP_1700_US:
    {
        return ( const char* ) "SX126X_RAMP_1700_US";
    }

    case SX126X_RAMP_3400_US:
    {
        return ( const char* ) "SX126X_RAMP_3400_US";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_gfsk_pulse_shape_to_str( const sx126x_gfsk_pulse_shape_t value )
{
    switch( value )
    {
    case SX126X_GFSK_PULSE_SHAPE_OFF:
    {
        return ( const char* ) "SX126X_GFSK_PULSE_SHAPE_OFF";
    }

    case SX126X_GFSK_PULSE_SHAPE_BT_03:
    {
        return ( const char* ) "SX126X_GFSK_PULSE_SHAPE_BT_03";
    }

    case SX126X_GFSK_PULSE_SHAPE_BT_05:
    {
        return ( const char* ) "SX126X_GFSK_PULSE_SHAPE_BT_05";
    }

    case SX126X_GFSK_PULSE_SHAPE_BT_07:
    {
        return ( const char* ) "SX126X_GFSK_PULSE_SHAPE_BT_07";
    }

    case SX126X_GFSK_PULSE_SHAPE_BT_1:
    {
        return ( const char* ) "SX126X_GFSK_PULSE_SHAPE_BT_1";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_bpsk_pulse_shape_to_str( const sx126x_bpsk_pulse_shape_t value )
{
    switch( value )
    {
    case SX126X_DBPSK_PULSE_SHAPE:
    {
        return ( const char* ) "SX126X_DBPSK_PULSE_SHAPE";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_gfsk_bw_to_str( const sx126x_gfsk_bw_t value )
{
    switch( value )
    {
    case SX126X_GFSK_BW_4800:
    {
        return ( const char* ) "SX126X_GFSK_BW_4800";
    }

    case SX126X_GFSK_BW_5800:
    {
        return ( const char* ) "SX126X_GFSK_BW_5800";
    }

    case SX126X_GFSK_BW_7300:
    {
        return ( const char* ) "SX126X_GFSK_BW_7300";
    }

    case SX126X_GFSK_BW_9700:
    {
        return ( const char* ) "SX126X_GFSK_BW_9700";
    }

    case SX126X_GFSK_BW_11700:
    {
        return ( const char* ) "SX126X_GFSK_BW_11700";
    }

    case SX126X_GFSK_BW_14600:
    {
        return ( const char* ) "SX126X_GFSK_BW_14600";
    }

    case SX126X_GFSK_BW_19500:
    {
        return ( const char* ) "SX126X_GFSK_BW_19500";
    }

    case SX126X_GFSK_BW_23400:
    {
        return ( const char* ) "SX126X_GFSK_BW_23400";
    }

    case SX126X_GFSK_BW_29300:
    {
        return ( const char* ) "SX126X_GFSK_BW_29300";
    }

    case SX126X_GFSK_BW_39000:
    {
        return ( const char* ) "SX126X_GFSK_BW_39000";
    }

    case SX126X_GFSK_BW_46900:
    {
        return ( const char* ) "SX126X_GFSK_BW_46900";
    }

    case SX126X_GFSK_BW_58600:
    {
        return ( const char* ) "SX126X_GFSK_BW_58600";
    }

    case SX126X_GFSK_BW_78200:
    {
        return ( const char* ) "SX126X_GFSK_BW_78200";
    }

    case SX126X_GFSK_BW_93800:
    {
        return ( const char* ) "SX126X_GFSK_BW_93800";
    }

    case SX126X_GFSK_BW_117300:
    {
        return ( const char* ) "SX126X_GFSK_BW_117300";
    }

    case SX126X_GFSK_BW_156200:
    {
        return ( const char* ) "SX126X_GFSK_BW_156200";
    }

    case SX126X_GFSK_BW_187200:
    {
        return ( const char* ) "SX126X_GFSK_BW_187200";
    }

    case SX126X_GFSK_BW_234300:
    {
        return ( const char* ) "SX126X_GFSK_BW_234300";
    }

    case SX126X_GFSK_BW_312000:
    {
        return ( const char* ) "SX126X_GFSK_BW_312000";
    }

    case SX126X_GFSK_BW_373600:
    {
        return ( const char* ) "SX126X_GFSK_BW_373600";
    }

    case SX126X_GFSK_BW_467000:
    {
        return ( const char* ) "SX126X_GFSK_BW_467000";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_lora_sf_to_str( const sx126x_lora_sf_t value )
{
    switch( value )
    {
    case SX126X_LORA_SF5:
    {
        return ( const char* ) "SX126X_LORA_SF5";
    }

    case SX126X_LORA_SF6:
    {
        return ( const char* ) "SX126X_LORA_SF6";
    }

    case SX126X_LORA_SF7:
    {
        return ( const char* ) "SX126X_LORA_SF7";
    }

    case SX126X_LORA_SF8:
    {
        return ( const char* ) "SX126X_LORA_SF8";
    }

    case SX126X_LORA_SF9:
    {
        return ( const char* ) "SX126X_LORA_SF9";
    }

    case SX126X_LORA_SF10:
    {
        return ( const char* ) "SX126X_LORA_SF10";
    }

    case SX126X_LORA_SF11:
    {
        return ( const char* ) "SX126X_LORA_SF11";
    }

    case SX126X_LORA_SF12:
    {
        return ( const char* ) "SX126X_LORA_SF12";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_lora_bw_to_str( const sx126x_lora_bw_t value )
{
    switch( value )
    {
    case SX126X_LORA_BW_500:
    {
        return ( const char* ) "SX126X_LORA_BW_500";
    }

    case SX126X_LORA_BW_250:
    {
        return ( const char* ) "SX126X_LORA_BW_250";
    }

    case SX126X_LORA_BW_125:
    {
        return ( const char* ) "SX126X_LORA_BW_125";
    }

    case SX126X_LORA_BW_062:
    {
        return ( const char* ) "SX126X_LORA_BW_062";
    }

    case SX126X_LORA_BW_041:
    {
        return ( const char* ) "SX126X_LORA_BW_041";
    }

    case SX126X_LORA_BW_031:
    {
        return ( const char* ) "SX126X_LORA_BW_031";
    }

    case SX126X_LORA_BW_020:
    {
        return ( const char* ) "SX126X_LORA_BW_020";
    }

    case SX126X_LORA_BW_015:
    {
        return ( const char* ) "SX126X_LORA_BW_015";
    }

    case SX126X_LORA_BW_010:
    {
        return ( const char* ) "SX126X_LORA_BW_010";
    }

    case SX126X_LORA_BW_007:
    {
        return ( const char* ) "SX126X_LORA_BW_007";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_lora_cr_to_str( const sx126x_lora_cr_t value )
{
    switch( value )
    {
    case SX126X_LORA_CR_4_5:
    {
        return ( const char* ) "SX126X_LORA_CR_4_5";
    }

    case SX126X_LORA_CR_4_6:
    {
        return ( const char* ) "SX126X_LORA_CR_4_6";
    }

    case SX126X_LORA_CR_4_7:
    {
        return ( const char* ) "SX126X_LORA_CR_4_7";
    }

    case SX126X_LORA_CR_4_8:
    {
        return ( const char* ) "SX126X_LORA_CR_4_8";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_gfsk_preamble_detector_to_str( const sx126x_gfsk_preamble_detector_t value )
{
    switch( value )
    {
    case SX126X_GFSK_PREAMBLE_DETECTOR_OFF:
    {
        return ( const char* ) "SX126X_GFSK_PREAMBLE_DETECTOR_OFF";
    }

    case SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS:
    {
        return ( const char* ) "SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS";
    }

    case SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS:
    {
        return ( const char* ) "SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS";
    }

    case SX126X_GFSK_PREAMBLE_DETECTOR_MIN_24BITS:
    {
        return ( const char* ) "SX126X_GFSK_PREAMBLE_DETECTOR_MIN_24BITS";
    }

    case SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS:
    {
        return ( const char* ) "SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_gfsk_address_filtering_to_str( const sx126x_gfsk_address_filtering_t value )
{
    switch( value )
    {
    case SX126X_GFSK_ADDRESS_FILTERING_DISABLE:
    {
        return ( const char* ) "SX126X_GFSK_ADDRESS_FILTERING_DISABLE";
    }

    case SX126X_GFSK_ADDRESS_FILTERING_NODE_ADDRESS:
    {
        return ( const char* ) "SX126X_GFSK_ADDRESS_FILTERING_NODE_ADDRESS";
    }

    case SX126X_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES:
    {
        return ( const char* ) "SX126X_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_gfsk_pkt_len_modes_to_str( const sx126x_gfsk_pkt_len_modes_t value )
{
    switch( value )
    {
    case SX126X_GFSK_PKT_FIX_LEN:
    {
        return ( const char* ) "SX126X_GFSK_PKT_FIX_LEN";
    }

    case SX126X_GFSK_PKT_VAR_LEN:
    {
        return ( const char* ) "SX126X_GFSK_PKT_VAR_LEN";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_gfsk_crc_types_to_str( const sx126x_gfsk_crc_types_t value )
{
    switch( value )
    {
    case SX126X_GFSK_CRC_OFF:
    {
        return ( const char* ) "SX126X_GFSK_CRC_OFF";
    }

    case SX126X_GFSK_CRC_1_BYTE:
    {
        return ( const char* ) "SX126X_GFSK_CRC_1_BYTE";
    }

    case SX126X_GFSK_CRC_2_BYTES:
    {
        return ( const char* ) "SX126X_GFSK_CRC_2_BYTES";
    }

    case SX126X_GFSK_CRC_1_BYTE_INV:
    {
        return ( const char* ) "SX126X_GFSK_CRC_1_BYTE_INV";
    }

    case SX126X_GFSK_CRC_2_BYTES_INV:
    {
        return ( const char* ) "SX126X_GFSK_CRC_2_BYTES_INV";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_gfsk_dc_free_to_str( const sx126x_gfsk_dc_free_t value )
{
    switch( value )
    {
    case SX126X_GFSK_DC_FREE_OFF:
    {
        return ( const char* ) "SX126X_GFSK_DC_FREE_OFF";
    }

    case SX126X_GFSK_DC_FREE_WHITENING:
    {
        return ( const char* ) "SX126X_GFSK_DC_FREE_WHITENING";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_lora_pkt_len_modes_to_str( const sx126x_lora_pkt_len_modes_t value )
{
    switch( value )
    {
    case SX126X_LORA_PKT_EXPLICIT:
    {
        return ( const char* ) "SX126X_LORA_PKT_EXPLICIT";
    }

    case SX126X_LORA_PKT_IMPLICIT:
    {
        return ( const char* ) "SX126X_LORA_PKT_IMPLICIT";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_cad_symbs_to_str( const sx126x_cad_symbs_t value )
{
    switch( value )
    {
    case SX126X_CAD_01_SYMB:
    {
        return ( const char* ) "SX126X_CAD_01_SYMB";
    }

    case SX126X_CAD_02_SYMB:
    {
        return ( const char* ) "SX126X_CAD_02_SYMB";
    }

    case SX126X_CAD_04_SYMB:
    {
        return ( const char* ) "SX126X_CAD_04_SYMB";
    }

    case SX126X_CAD_08_SYMB:
    {
        return ( const char* ) "SX126X_CAD_08_SYMB";
    }

    case SX126X_CAD_16_SYMB:
    {
        return ( const char* ) "SX126X_CAD_16_SYMB";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_cad_exit_modes_to_str( const sx126x_cad_exit_modes_t value )
{
    switch( value )
    {
    case SX126X_CAD_ONLY:
    {
        return ( const char* ) "SX126X_CAD_ONLY";
    }

    case SX126X_CAD_RX:
    {
        return ( const char* ) "SX126X_CAD_RX";
    }

    case SX126X_CAD_LBT:
    {
        return ( const char* ) "SX126X_CAD_LBT";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_chip_modes_to_str( const sx126x_chip_modes_t value )
{
    switch( value )
    {
    case SX126X_CHIP_MODE_UNUSED:
    {
        return ( const char* ) "SX126X_CHIP_MODE_UNUSED";
    }

    case SX126X_CHIP_MODE_RFU:
    {
        return ( const char* ) "SX126X_CHIP_MODE_RFU";
    }

    case SX126X_CHIP_MODE_STBY_RC:
    {
        return ( const char* ) "SX126X_CHIP_MODE_STBY_RC";
    }

    case SX126X_CHIP_MODE_STBY_XOSC:
    {
        return ( const char* ) "SX126X_CHIP_MODE_STBY_XOSC";
    }

    case SX126X_CHIP_MODE_FS:
    {
        return ( const char* ) "SX126X_CHIP_MODE_FS";
    }

    case SX126X_CHIP_MODE_RX:
    {
        return ( const char* ) "SX126X_CHIP_MODE_RX";
    }

    case SX126X_CHIP_MODE_TX:
    {
        return ( const char* ) "SX126X_CHIP_MODE_TX";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* sx126x_cmd_status_to_str( const sx126x_cmd_status_t value )
{
    switch( value )
    {
    case SX126X_CMD_STATUS_RESERVED:
    {
        return ( const char* ) "SX126X_CMD_STATUS_RESERVED";
    }

    case SX126X_CMD_STATUS_RFU:
    {
        return ( const char* ) "SX126X_CMD_STATUS_RFU";
    }

    case SX126X_CMD_STATUS_DATA_AVAILABLE:
    {
        return ( const char* ) "SX126X_CMD_STATUS_DATA_AVAILABLE";
    }

    case SX126X_CMD_STATUS_CMD_TIMEOUT:
    {
        return ( const char* ) "SX126X_CMD_STATUS_CMD_TIMEOUT";
    }

    case SX126X_CMD_STATUS_CMD_PROCESS_ERROR:
    {
        return ( const char* ) "SX126X_CMD_STATUS_CMD_PROCESS_ERROR";
    }

    case SX126X_CMD_STATUS_CMD_EXEC_FAILURE:
    {
        return ( const char* ) "SX126X_CMD_STATUS_CMD_EXEC_FAILURE";
    }

    case SX126X_CMD_STATUS_CMD_TX_DONE:
    {
        return ( const char* ) "SX126X_CMD_STATUS_CMD_TX_DONE";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}
