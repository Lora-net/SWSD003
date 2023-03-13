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
