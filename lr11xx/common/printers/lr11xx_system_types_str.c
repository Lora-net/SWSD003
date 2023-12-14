/*!
 * @file      lr11xx_system_types_str.c
 *
 * @brief     Printer helper functions for LR11xx system types
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
 
#include "lr11xx_system_types_str.h"

const char* lr11xx_system_chip_modes_to_str( const lr11xx_system_chip_modes_t value )
{
    switch( value )
    {
    case LR11XX_SYSTEM_CHIP_MODE_SLEEP:
    {
        return ( const char* ) "LR11XX_SYSTEM_CHIP_MODE_SLEEP";
    }

    case LR11XX_SYSTEM_CHIP_MODE_STBY_RC:
    {
        return ( const char* ) "LR11XX_SYSTEM_CHIP_MODE_STBY_RC";
    }

    case LR11XX_SYSTEM_CHIP_MODE_STBY_XOSC:
    {
        return ( const char* ) "LR11XX_SYSTEM_CHIP_MODE_STBY_XOSC";
    }

    case LR11XX_SYSTEM_CHIP_MODE_FS:
    {
        return ( const char* ) "LR11XX_SYSTEM_CHIP_MODE_FS";
    }

    case LR11XX_SYSTEM_CHIP_MODE_RX:
    {
        return ( const char* ) "LR11XX_SYSTEM_CHIP_MODE_RX";
    }

    case LR11XX_SYSTEM_CHIP_MODE_TX:
    {
        return ( const char* ) "LR11XX_SYSTEM_CHIP_MODE_TX";
    }

    case LR11XX_SYSTEM_CHIP_MODE_LOC:
    {
        return ( const char* ) "LR11XX_SYSTEM_CHIP_MODE_LOC";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_system_reset_status_to_str( const lr11xx_system_reset_status_t value )
{
    switch( value )
    {
    case LR11XX_SYSTEM_RESET_STATUS_CLEARED:
    {
        return ( const char* ) "LR11XX_SYSTEM_RESET_STATUS_CLEARED";
    }

    case LR11XX_SYSTEM_RESET_STATUS_ANALOG:
    {
        return ( const char* ) "LR11XX_SYSTEM_RESET_STATUS_ANALOG";
    }

    case LR11XX_SYSTEM_RESET_STATUS_EXTERNAL:
    {
        return ( const char* ) "LR11XX_SYSTEM_RESET_STATUS_EXTERNAL";
    }

    case LR11XX_SYSTEM_RESET_STATUS_SYSTEM:
    {
        return ( const char* ) "LR11XX_SYSTEM_RESET_STATUS_SYSTEM";
    }

    case LR11XX_SYSTEM_RESET_STATUS_WATCHDOG:
    {
        return ( const char* ) "LR11XX_SYSTEM_RESET_STATUS_WATCHDOG";
    }

    case LR11XX_SYSTEM_RESET_STATUS_IOCD_RESTART:
    {
        return ( const char* ) "LR11XX_SYSTEM_RESET_STATUS_IOCD_RESTART";
    }

    case LR11XX_SYSTEM_RESET_STATUS_RTC_RESTART:
    {
        return ( const char* ) "LR11XX_SYSTEM_RESET_STATUS_RTC_RESTART";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_system_command_status_to_str( const lr11xx_system_command_status_t value )
{
    switch( value )
    {
    case LR11XX_SYSTEM_CMD_STATUS_FAIL:
    {
        return ( const char* ) "LR11XX_SYSTEM_CMD_STATUS_FAIL";
    }

    case LR11XX_SYSTEM_CMD_STATUS_PERR:
    {
        return ( const char* ) "LR11XX_SYSTEM_CMD_STATUS_PERR";
    }

    case LR11XX_SYSTEM_CMD_STATUS_OK:
    {
        return ( const char* ) "LR11XX_SYSTEM_CMD_STATUS_OK";
    }

    case LR11XX_SYSTEM_CMD_STATUS_DATA:
    {
        return ( const char* ) "LR11XX_SYSTEM_CMD_STATUS_DATA";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_system_lfclk_cfg_to_str( const lr11xx_system_lfclk_cfg_t value )
{
    switch( value )
    {
    case LR11XX_SYSTEM_LFCLK_RC:
    {
        return ( const char* ) "LR11XX_SYSTEM_LFCLK_RC";
    }

    case LR11XX_SYSTEM_LFCLK_XTAL:
    {
        return ( const char* ) "LR11XX_SYSTEM_LFCLK_XTAL";
    }

    case LR11XX_SYSTEM_LFCLK_EXT:
    {
        return ( const char* ) "LR11XX_SYSTEM_LFCLK_EXT";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_system_reg_mode_to_str( const lr11xx_system_reg_mode_t value )
{
    switch( value )
    {
    case LR11XX_SYSTEM_REG_MODE_LDO:
    {
        return ( const char* ) "LR11XX_SYSTEM_REG_MODE_LDO";
    }

    case LR11XX_SYSTEM_REG_MODE_DCDC:
    {
        return ( const char* ) "LR11XX_SYSTEM_REG_MODE_DCDC";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_system_infopage_id_to_str( const lr11xx_system_infopage_id_t value )
{
    switch( value )
    {
    case LR11XX_SYSTEM_INFOPAGE_0:
    {
        return ( const char* ) "LR11XX_SYSTEM_INFOPAGE_0";
    }

    case LR11XX_SYSTEM_INFOPAGE_1:
    {
        return ( const char* ) "LR11XX_SYSTEM_INFOPAGE_1";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_system_standby_cfg_to_str( const lr11xx_system_standby_cfg_t value )
{
    switch( value )
    {
    case LR11XX_SYSTEM_STANDBY_CFG_RC:
    {
        return ( const char* ) "LR11XX_SYSTEM_STANDBY_CFG_RC";
    }

    case LR11XX_SYSTEM_STANDBY_CFG_XOSC:
    {
        return ( const char* ) "LR11XX_SYSTEM_STANDBY_CFG_XOSC";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_system_tcxo_supply_voltage_to_str( const lr11xx_system_tcxo_supply_voltage_t value )
{
    switch( value )
    {
    case LR11XX_SYSTEM_TCXO_CTRL_1_6V:
    {
        return ( const char* ) "LR11XX_SYSTEM_TCXO_CTRL_1_6V";
    }

    case LR11XX_SYSTEM_TCXO_CTRL_1_7V:
    {
        return ( const char* ) "LR11XX_SYSTEM_TCXO_CTRL_1_7V";
    }

    case LR11XX_SYSTEM_TCXO_CTRL_1_8V:
    {
        return ( const char* ) "LR11XX_SYSTEM_TCXO_CTRL_1_8V";
    }

    case LR11XX_SYSTEM_TCXO_CTRL_2_2V:
    {
        return ( const char* ) "LR11XX_SYSTEM_TCXO_CTRL_2_2V";
    }

    case LR11XX_SYSTEM_TCXO_CTRL_2_4V:
    {
        return ( const char* ) "LR11XX_SYSTEM_TCXO_CTRL_2_4V";
    }

    case LR11XX_SYSTEM_TCXO_CTRL_2_7V:
    {
        return ( const char* ) "LR11XX_SYSTEM_TCXO_CTRL_2_7V";
    }

    case LR11XX_SYSTEM_TCXO_CTRL_3_0V:
    {
        return ( const char* ) "LR11XX_SYSTEM_TCXO_CTRL_3_0V";
    }

    case LR11XX_SYSTEM_TCXO_CTRL_3_3V:
    {
        return ( const char* ) "LR11XX_SYSTEM_TCXO_CTRL_3_3V";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_system_version_type_to_str( const lr11xx_system_version_type_t value )
{
    switch( value )
    {
    case LR11XX_SYSTEM_VERSION_TYPE_LR1110:
    {
        return ( const char* ) "LR11XX_SYSTEM_VERSION_TYPE_LR1110";
    }

    case LR11XX_SYSTEM_VERSION_TYPE_LR1120:
    {
        return ( const char* ) "LR11XX_SYSTEM_VERSION_TYPE_LR1120";
    }

    case LR11XX_SYSTEM_VERSION_TYPE_LR1121:
    {
        return ( const char* ) "LR11XX_SYSTEM_VERSION_TYPE_LR1121";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}
