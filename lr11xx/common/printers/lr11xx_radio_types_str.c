/*!
 * @file      lr11xx_radio_types_str.c
 *
 * @brief     Printer helper functions for LR11xx radio types
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
 
#include "lr11xx_radio_types_str.h"

const char* lr11xx_radio_pa_selection_to_str( const lr11xx_radio_pa_selection_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_PA_SEL_LP:
    {
        return ( const char* ) "LR11XX_RADIO_PA_SEL_LP";
    }

    case LR11XX_RADIO_PA_SEL_HP:
    {
        return ( const char* ) "LR11XX_RADIO_PA_SEL_HP";
    }

    case LR11XX_RADIO_PA_SEL_HF:
    {
        return ( const char* ) "LR11XX_RADIO_PA_SEL_HF";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_gfsk_address_filtering_to_str( const lr11xx_radio_gfsk_address_filtering_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE";
    }

    case LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_ADDRESS:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_ADDRESS";
    }

    case LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_fallback_modes_to_str( const lr11xx_radio_fallback_modes_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_FALLBACK_STDBY_RC:
    {
        return ( const char* ) "LR11XX_RADIO_FALLBACK_STDBY_RC";
    }

    case LR11XX_RADIO_FALLBACK_STDBY_XOSC:
    {
        return ( const char* ) "LR11XX_RADIO_FALLBACK_STDBY_XOSC";
    }

    case LR11XX_RADIO_FALLBACK_FS:
    {
        return ( const char* ) "LR11XX_RADIO_FALLBACK_FS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_ramp_time_to_str( const lr11xx_radio_ramp_time_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_RAMP_16_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_16_US";
    }

    case LR11XX_RADIO_RAMP_32_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_32_US";
    }

    case LR11XX_RADIO_RAMP_48_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_48_US";
    }

    case LR11XX_RADIO_RAMP_64_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_64_US";
    }

    case LR11XX_RADIO_RAMP_80_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_80_US";
    }

    case LR11XX_RADIO_RAMP_96_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_96_US";
    }

    case LR11XX_RADIO_RAMP_112_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_112_US";
    }

    case LR11XX_RADIO_RAMP_128_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_128_US";
    }

    case LR11XX_RADIO_RAMP_144_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_144_US";
    }

    case LR11XX_RADIO_RAMP_160_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_160_US";
    }

    case LR11XX_RADIO_RAMP_176_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_176_US";
    }

    case LR11XX_RADIO_RAMP_192_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_192_US";
    }

    case LR11XX_RADIO_RAMP_208_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_208_US";
    }

    case LR11XX_RADIO_RAMP_240_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_240_US";
    }

    case LR11XX_RADIO_RAMP_272_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_272_US";
    }

    case LR11XX_RADIO_RAMP_304_US:
    {
        return ( const char* ) "LR11XX_RADIO_RAMP_304_US";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_lora_network_type_to_str( const lr11xx_radio_lora_network_type_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_LORA_NETWORK_PRIVATE:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_NETWORK_PRIVATE";
    }

    case LR11XX_RADIO_LORA_NETWORK_PUBLIC:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_NETWORK_PUBLIC";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_lora_sf_to_str( const lr11xx_radio_lora_sf_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_LORA_SF5:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_SF5";
    }

    case LR11XX_RADIO_LORA_SF6:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_SF6";
    }

    case LR11XX_RADIO_LORA_SF7:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_SF7";
    }

    case LR11XX_RADIO_LORA_SF8:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_SF8";
    }

    case LR11XX_RADIO_LORA_SF9:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_SF9";
    }

    case LR11XX_RADIO_LORA_SF10:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_SF10";
    }

    case LR11XX_RADIO_LORA_SF11:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_SF11";
    }

    case LR11XX_RADIO_LORA_SF12:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_SF12";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_lora_bw_to_str( const lr11xx_radio_lora_bw_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_LORA_BW_10:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_10";
    }

    case LR11XX_RADIO_LORA_BW_15:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_15";
    }

    case LR11XX_RADIO_LORA_BW_20:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_20";
    }

    case LR11XX_RADIO_LORA_BW_31:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_31";
    }

    case LR11XX_RADIO_LORA_BW_41:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_41";
    }

    case LR11XX_RADIO_LORA_BW_62:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_62";
    }

    case LR11XX_RADIO_LORA_BW_125:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_125";
    }

    case LR11XX_RADIO_LORA_BW_250:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_250";
    }

    case LR11XX_RADIO_LORA_BW_500:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_500";
    }

    case LR11XX_RADIO_LORA_BW_200:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_200";
    }

    case LR11XX_RADIO_LORA_BW_400:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_400";
    }

    case LR11XX_RADIO_LORA_BW_800:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_BW_800";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_lora_cr_to_str( const lr11xx_radio_lora_cr_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_LORA_NO_CR:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_NO_CR";
    }

    case LR11XX_RADIO_LORA_CR_4_5:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_CR_4_5";
    }

    case LR11XX_RADIO_LORA_CR_4_6:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_CR_4_6";
    }

    case LR11XX_RADIO_LORA_CR_4_7:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_CR_4_7";
    }

    case LR11XX_RADIO_LORA_CR_4_8:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_CR_4_8";
    }

    case LR11XX_RADIO_LORA_CR_LI_4_5:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_CR_LI_4_5";
    }

    case LR11XX_RADIO_LORA_CR_LI_4_6:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_CR_LI_4_6";
    }

    case LR11XX_RADIO_LORA_CR_LI_4_8:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_CR_LI_4_8";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_intermediary_mode_to_str( const lr11xx_radio_intermediary_mode_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_MODE_SLEEP:
    {
        return ( const char* ) "LR11XX_RADIO_MODE_SLEEP";
    }

    case LR11XX_RADIO_MODE_STANDBY_RC:
    {
        return ( const char* ) "LR11XX_RADIO_MODE_STANDBY_RC";
    }

    case LR11XX_RADIO_MODE_STANDBY_XOSC:
    {
        return ( const char* ) "LR11XX_RADIO_MODE_STANDBY_XOSC";
    }

    case LR11XX_RADIO_MODE_FS:
    {
        return ( const char* ) "LR11XX_RADIO_MODE_FS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_gfsk_crc_type_to_str( const lr11xx_radio_gfsk_crc_type_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_GFSK_CRC_OFF:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_CRC_OFF";
    }

    case LR11XX_RADIO_GFSK_CRC_1_BYTE:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_CRC_1_BYTE";
    }

    case LR11XX_RADIO_GFSK_CRC_2_BYTES:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_CRC_2_BYTES";
    }

    case LR11XX_RADIO_GFSK_CRC_1_BYTE_INV:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_CRC_1_BYTE_INV";
    }

    case LR11XX_RADIO_GFSK_CRC_2_BYTES_INV:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_CRC_2_BYTES_INV";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_gfsk_dc_free_to_str( const lr11xx_radio_gfsk_dc_free_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_GFSK_DC_FREE_OFF:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_DC_FREE_OFF";
    }

    case LR11XX_RADIO_GFSK_DC_FREE_WHITENING:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_DC_FREE_WHITENING";
    }

    case LR11XX_RADIO_GFSK_DC_FREE_WHITENING_SX128X_COMP:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_DC_FREE_WHITENING_SX128X_COMP";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_gfsk_pkt_len_modes_to_str( const lr11xx_radio_gfsk_pkt_len_modes_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_GFSK_PKT_FIX_LEN:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PKT_FIX_LEN";
    }

    case LR11XX_RADIO_GFSK_PKT_VAR_LEN:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PKT_VAR_LEN";
    }

    case LR11XX_RADIO_GFSK_PKT_VAR_LEN_SX128X_COMP:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PKT_VAR_LEN_SX128X_COMP";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_gfsk_preamble_detector_to_str( const lr11xx_radio_gfsk_preamble_detector_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_OFF:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_OFF";
    }

    case LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS";
    }

    case LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS";
    }

    case LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_24BITS:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_24BITS";
    }

    case LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_32BITS:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_32BITS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_lora_crc_to_str( const lr11xx_radio_lora_crc_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_LORA_CRC_OFF:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_CRC_OFF";
    }

    case LR11XX_RADIO_LORA_CRC_ON:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_CRC_ON";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_lora_pkt_len_modes_to_str( const lr11xx_radio_lora_pkt_len_modes_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_LORA_PKT_EXPLICIT:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_PKT_EXPLICIT";
    }

    case LR11XX_RADIO_LORA_PKT_IMPLICIT:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_PKT_IMPLICIT";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_lora_iq_to_str( const lr11xx_radio_lora_iq_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_LORA_IQ_STANDARD:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_IQ_STANDARD";
    }

    case LR11XX_RADIO_LORA_IQ_INVERTED:
    {
        return ( const char* ) "LR11XX_RADIO_LORA_IQ_INVERTED";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_pkt_type_to_str( const lr11xx_radio_pkt_type_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_PKT_NONE:
    {
        return ( const char* ) "LR11XX_RADIO_PKT_NONE";
    }

    case LR11XX_RADIO_PKT_TYPE_GFSK:
    {
        return ( const char* ) "LR11XX_RADIO_PKT_TYPE_GFSK";
    }

    case LR11XX_RADIO_PKT_TYPE_LORA:
    {
        return ( const char* ) "LR11XX_RADIO_PKT_TYPE_LORA";
    }

    case LR11XX_RADIO_PKT_TYPE_BPSK:
    {
        return ( const char* ) "LR11XX_RADIO_PKT_TYPE_BPSK";
    }

    case LR11XX_RADIO_PKT_TYPE_LR_FHSS:
    {
        return ( const char* ) "LR11XX_RADIO_PKT_TYPE_LR_FHSS";
    }

    case LR11XX_RADIO_PKT_TYPE_RTTOF:
    {
        return ( const char* ) "LR11XX_RADIO_PKT_TYPE_RTTOF";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_pa_reg_supply_to_str( const lr11xx_radio_pa_reg_supply_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_PA_REG_SUPPLY_VREG:
    {
        return ( const char* ) "LR11XX_RADIO_PA_REG_SUPPLY_VREG";
    }

    case LR11XX_RADIO_PA_REG_SUPPLY_VBAT:
    {
        return ( const char* ) "LR11XX_RADIO_PA_REG_SUPPLY_VBAT";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_rx_duty_cycle_mode_to_str( const lr11xx_radio_rx_duty_cycle_mode_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_RX_DUTY_CYCLE_MODE_RX:
    {
        return ( const char* ) "LR11XX_RADIO_RX_DUTY_CYCLE_MODE_RX";
    }

    case LR11XX_RADIO_RX_DUTY_CYCLE_MODE_CAD:
    {
        return ( const char* ) "LR11XX_RADIO_RX_DUTY_CYCLE_MODE_CAD";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_gfsk_bw_to_str( const lr11xx_radio_gfsk_bw_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_GFSK_BW_4800:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_4800";
    }

    case LR11XX_RADIO_GFSK_BW_5800:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_5800";
    }

    case LR11XX_RADIO_GFSK_BW_7300:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_7300";
    }

    case LR11XX_RADIO_GFSK_BW_9700:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_9700";
    }

    case LR11XX_RADIO_GFSK_BW_11700:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_11700";
    }

    case LR11XX_RADIO_GFSK_BW_14600:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_14600";
    }

    case LR11XX_RADIO_GFSK_BW_19500:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_19500";
    }

    case LR11XX_RADIO_GFSK_BW_23400:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_23400";
    }

    case LR11XX_RADIO_GFSK_BW_29300:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_29300";
    }

    case LR11XX_RADIO_GFSK_BW_39000:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_39000";
    }

    case LR11XX_RADIO_GFSK_BW_46900:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_46900";
    }

    case LR11XX_RADIO_GFSK_BW_58600:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_58600";
    }

    case LR11XX_RADIO_GFSK_BW_78200:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_78200";
    }

    case LR11XX_RADIO_GFSK_BW_93800:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_93800";
    }

    case LR11XX_RADIO_GFSK_BW_117300:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_117300";
    }

    case LR11XX_RADIO_GFSK_BW_156200:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_156200";
    }

    case LR11XX_RADIO_GFSK_BW_187200:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_187200";
    }

    case LR11XX_RADIO_GFSK_BW_234300:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_234300";
    }

    case LR11XX_RADIO_GFSK_BW_312000:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_312000";
    }

    case LR11XX_RADIO_GFSK_BW_373600:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_373600";
    }

    case LR11XX_RADIO_GFSK_BW_467000:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_BW_467000";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_cad_exit_mode_to_str( const lr11xx_radio_cad_exit_mode_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC:
    {
        return ( const char* ) "LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC";
    }

    case LR11XX_RADIO_CAD_EXIT_MODE_RX:
    {
        return ( const char* ) "LR11XX_RADIO_CAD_EXIT_MODE_RX";
    }

    case LR11XX_RADIO_CAD_EXIT_MODE_TX:
    {
        return ( const char* ) "LR11XX_RADIO_CAD_EXIT_MODE_TX";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_gfsk_pulse_shape_to_str( const lr11xx_radio_gfsk_pulse_shape_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PULSE_SHAPE_OFF";
    }

    case LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_03:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_03";
    }

    case LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_05:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_05";
    }

    case LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_07:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_07";
    }

    case LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1:
    {
        return ( const char* ) "LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_bpsk_pulse_shape_to_str( const lr11xx_radio_bpsk_pulse_shape_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_DBPSK_PULSE_SHAPE:
    {
        return ( const char* ) "LR11XX_RADIO_DBPSK_PULSE_SHAPE";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_lr_fhss_bitrate_to_str( const lr11xx_radio_lr_fhss_bitrate_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_LR_FHSS_BITRATE_488_BPS:
    {
        return ( const char* ) "LR11XX_RADIO_LR_FHSS_BITRATE_488_BPS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_radio_lr_fhss_pulse_shape_to_str( const lr11xx_radio_lr_fhss_pulse_shape_t value )
{
    switch( value )
    {
    case LR11XX_RADIO_LR_FHSS_PULSE_SHAPE_BT_1:
    {
        return ( const char* ) "LR11XX_RADIO_LR_FHSS_PULSE_SHAPE_BT_1";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}
