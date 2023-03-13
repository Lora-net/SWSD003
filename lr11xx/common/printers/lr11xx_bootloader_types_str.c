#include "lr11xx_bootloader_types_str.h"

const char* lr11xx_bootloader_chip_modes_to_str( const lr11xx_bootloader_chip_modes_t value )
{
    switch( value )
    {
    case LR11XX_BOOTLOADER_CHIP_MODE_SLEEP:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CHIP_MODE_SLEEP";
    }

    case LR11XX_BOOTLOADER_CHIP_MODE_STBY_RC:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CHIP_MODE_STBY_RC";
    }

    case LR11XX_BOOTLOADER_CHIP_MODE_STBY_XOSC:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CHIP_MODE_STBY_XOSC";
    }

    case LR11XX_BOOTLOADER_CHIP_MODE_FS:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CHIP_MODE_FS";
    }

    case LR11XX_BOOTLOADER_CHIP_MODE_RX:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CHIP_MODE_RX";
    }

    case LR11XX_BOOTLOADER_CHIP_MODE_TX:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CHIP_MODE_TX";
    }

    case LR11XX_BOOTLOADER_CHIP_MODE_LOC:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CHIP_MODE_LOC";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_bootloader_reset_status_to_str( const lr11xx_bootloader_reset_status_t value )
{
    switch( value )
    {
    case LR11XX_BOOTLOADER_RESET_STATUS_CLEARED:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_RESET_STATUS_CLEARED";
    }

    case LR11XX_BOOTLOADER_RESET_STATUS_ANALOG:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_RESET_STATUS_ANALOG";
    }

    case LR11XX_BOOTLOADER_RESET_STATUS_EXTERNAL:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_RESET_STATUS_EXTERNAL";
    }

    case LR11XX_BOOTLOADER_RESET_STATUS_SYSTEM:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_RESET_STATUS_SYSTEM";
    }

    case LR11XX_BOOTLOADER_RESET_STATUS_WATCHDOG:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_RESET_STATUS_WATCHDOG";
    }

    case LR11XX_BOOTLOADER_RESET_STATUS_IOCD_RESTART:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_RESET_STATUS_IOCD_RESTART";
    }

    case LR11XX_BOOTLOADER_RESET_STATUS_RTC_RESTART:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_RESET_STATUS_RTC_RESTART";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_bootloader_command_status_to_str( const lr11xx_bootloader_command_status_t value )
{
    switch( value )
    {
    case LR11XX_BOOTLOADER_CMD_STATUS_FAIL:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CMD_STATUS_FAIL";
    }

    case LR11XX_BOOTLOADER_CMD_STATUS_PERR:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CMD_STATUS_PERR";
    }

    case LR11XX_BOOTLOADER_CMD_STATUS_OK:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CMD_STATUS_OK";
    }

    case LR11XX_BOOTLOADER_CMD_STATUS_DATA:
    {
        return ( const char* ) "LR11XX_BOOTLOADER_CMD_STATUS_DATA";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}
