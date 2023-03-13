#include "lr_fhss_v1_base_types_str.h"

const char* lr_fhss_v1_modulation_type_to_str( const lr_fhss_v1_modulation_type_t value )
{
    switch( value )
    {
    case LR_FHSS_V1_MODULATION_TYPE_GMSK_488:
    {
        return ( const char* ) "LR_FHSS_V1_MODULATION_TYPE_GMSK_488";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr_fhss_v1_cr_to_str( const lr_fhss_v1_cr_t value )
{
    switch( value )
    {
    case LR_FHSS_V1_CR_5_6:
    {
        return ( const char* ) "LR_FHSS_V1_CR_5_6";
    }

    case LR_FHSS_V1_CR_2_3:
    {
        return ( const char* ) "LR_FHSS_V1_CR_2_3";
    }

    case LR_FHSS_V1_CR_1_2:
    {
        return ( const char* ) "LR_FHSS_V1_CR_1_2";
    }

    case LR_FHSS_V1_CR_1_3:
    {
        return ( const char* ) "LR_FHSS_V1_CR_1_3";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr_fhss_v1_grid_to_str( const lr_fhss_v1_grid_t value )
{
    switch( value )
    {
    case LR_FHSS_V1_GRID_25391_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_GRID_25391_HZ";
    }

    case LR_FHSS_V1_GRID_3906_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_GRID_3906_HZ";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr_fhss_v1_bw_to_str( const lr_fhss_v1_bw_t value )
{
    switch( value )
    {
    case LR_FHSS_V1_BW_39063_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_BW_39063_HZ";
    }

    case LR_FHSS_V1_BW_85938_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_BW_85938_HZ";
    }

    case LR_FHSS_V1_BW_136719_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_BW_136719_HZ";
    }

    case LR_FHSS_V1_BW_183594_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_BW_183594_HZ";
    }

    case LR_FHSS_V1_BW_335938_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_BW_335938_HZ";
    }

    case LR_FHSS_V1_BW_386719_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_BW_386719_HZ";
    }

    case LR_FHSS_V1_BW_722656_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_BW_722656_HZ";
    }

    case LR_FHSS_V1_BW_773438_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_BW_773438_HZ";
    }

    case LR_FHSS_V1_BW_1523438_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_BW_1523438_HZ";
    }

    case LR_FHSS_V1_BW_1574219_HZ:
    {
        return ( const char* ) "LR_FHSS_V1_BW_1574219_HZ";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}
