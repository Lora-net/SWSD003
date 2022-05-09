#include "lr11xx_gnss_types_str.h"

const char* lr11xx_gnss_constellation_to_str( const lr11xx_gnss_constellation_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_GPS_MASK:
    {
        return ( const char* ) "LR11XX_GNSS_GPS_MASK";
    }

    case LR11XX_GNSS_BEIDOU_MASK:
    {
        return ( const char* ) "LR11XX_GNSS_BEIDOU_MASK";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_search_mode_to_str( const lr11xx_gnss_search_mode_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_OPTION_DEFAULT:
    {
        return ( const char* ) "LR11XX_GNSS_OPTION_DEFAULT";
    }

    case LR11XX_GNSS_OPTION_BEST_EFFORT:
    {
        return ( const char* ) "LR11XX_GNSS_OPTION_BEST_EFFORT";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_destination_to_str( const lr11xx_gnss_destination_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_DESTINATION_HOST:
    {
        return ( const char* ) "LR11XX_GNSS_DESTINATION_HOST";
    }

    case LR11XX_GNSS_DESTINATION_SOLVER:
    {
        return ( const char* ) "LR11XX_GNSS_DESTINATION_SOLVER";
    }

    case LR11XX_GNSS_DESTINATION_DMC:
    {
        return ( const char* ) "LR11XX_GNSS_DESTINATION_DMC";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_message_host_status_to_str( const lr11xx_gnss_message_host_status_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_HOST_OK:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_OK";
    }

    case LR11XX_GNSS_HOST_UNEXPECTED_CMD:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_UNEXPECTED_CMD";
    }

    case LR11XX_GNSS_HOST_UNIMPLEMENTED_CMD:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_UNIMPLEMENTED_CMD";
    }

    case LR11XX_GNSS_HOST_INVALID_PARAMETERS:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_INVALID_PARAMETERS";
    }

    case LR11XX_GNSS_HOST_MESSAGE_SANITY_CHECK_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_MESSAGE_SANITY_CHECK_ERROR";
    }

    case LR11XX_GNSS_HOST_IQ_CAPTURE_FAILS:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_IQ_CAPTURE_FAILS";
    }

    case LR11XX_GNSS_HOST_NO_TIME:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_NO_TIME";
    }

    case LR11XX_GNSS_HOST_NO_SATELLITE_DETECTED:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_NO_SATELLITE_DETECTED";
    }

    case LR11XX_GNSS_HOST_ALMANAC_IN_FLASH_TOO_OLD:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_IN_FLASH_TOO_OLD";
    }

    case LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_CRC_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_CRC_ERROR";
    }

    case LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_FLASH_INTEGRITY_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_FLASH_INTEGRITY_ERROR";
    }

    case LR11XX_GNSS_HOST_ALMANAC_UPDATE_NOT_ALLOWED:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_UPDATE_NOT_ALLOWED";
    }

    case LR11XX_GNSS_HOST_ALMANAC_CRC_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_CRC_ERROR";
    }

    case LR11XX_GNSS_HOST_ALMANAC_VERSION_NOT_SUPPORTED:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_VERSION_NOT_SUPPORTED";
    }

    case LR11XX_GNSS_HOST_NOT_ENOUGH_SV_DETECTED_TO_BUILD_A_NAV_MESSAGE:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_NOT_ENOUGH_SV_DETECTED_TO_BUILD_A_NAV_MESSAGE";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_message_dmc_opcode_to_str( const lr11xx_gnss_message_dmc_opcode_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_DMC_STATUS:
    {
        return ( const char* ) "LR11XX_GNSS_DMC_STATUS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_scan_mode_to_str( const lr11xx_gnss_scan_mode_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_SCAN_MODE_0_SINGLE_SCAN_LEGACY:
    {
        return ( const char* ) "LR11XX_GNSS_SCAN_MODE_0_SINGLE_SCAN_LEGACY";
    }

    case LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS:
    {
        return ( const char* ) "LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_error_code_to_str( const lr11xx_gnss_error_code_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_NO_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_NO_ERROR";
    }

    case LR11XX_GNSS_ERROR_ALMANAC_TOO_OLD:
    {
        return ( const char* ) "LR11XX_GNSS_ERROR_ALMANAC_TOO_OLD";
    }

    case LR11XX_GNSS_ERROR_UPDATE_CRC_MISMATCH:
    {
        return ( const char* ) "LR11XX_GNSS_ERROR_UPDATE_CRC_MISMATCH";
    }

    case LR11XX_GNSS_ERROR_UPDATE_FLASH_MEMORY_INTEGRITY:
    {
        return ( const char* ) "LR11XX_GNSS_ERROR_UPDATE_FLASH_MEMORY_INTEGRITY";
    }

    case LR11XX_GNSS_ERROR_ALMANAC_UPDATE_NOT_ALLOWED:
    {
        return ( const char* ) "LR11XX_GNSS_ERROR_ALMANAC_UPDATE_NOT_ALLOWED";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_freq_search_space_to_str( const lr11xx_gnss_freq_search_space_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_250_HZ:
    {
        return ( const char* ) "LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_250_HZ";
    }

    case LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_500_HZ:
    {
        return ( const char* ) "LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_500_HZ";
    }

    case LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_1_KHZ:
    {
        return ( const char* ) "LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_1_KHZ";
    }

    case LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_2_KHZ:
    {
        return ( const char* ) "LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_2_KHZ";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}
