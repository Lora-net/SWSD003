#include "lr11xx_types_str.h"

const char* lr11xx_status_to_str( const lr11xx_status_t value )
{
    switch( value )
    {
    case LR11XX_STATUS_OK:
    {
        return ( const char* ) "LR11XX_STATUS_OK";
    }

    case LR11XX_STATUS_ERROR:
    {
        return ( const char* ) "LR11XX_STATUS_ERROR";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}
