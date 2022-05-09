#include "lr11xx_crypto_engine_types_str.h"

const char* lr11xx_crypto_element_to_str( const lr11xx_crypto_element_t value )
{
    switch( value )
    {
    case LR11XX_CRYPTO_ELEMENT_CRYPTO_ENGINE:
    {
        return ( const char* ) "LR11XX_CRYPTO_ELEMENT_CRYPTO_ENGINE";
    }

    case LR11XX_CRYPTO_ELEMENT_SECURE_ELEMENT:
    {
        return ( const char* ) "LR11XX_CRYPTO_ELEMENT_SECURE_ELEMENT";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_crypto_status_to_str( const lr11xx_crypto_status_t value )
{
    switch( value )
    {
    case LR11XX_CRYPTO_STATUS_SUCCESS:
    {
        return ( const char* ) "LR11XX_CRYPTO_STATUS_SUCCESS";
    }

    case LR11XX_CRYPTO_STATUS_ERROR_FAIL_CMAC:
    {
        return ( const char* ) "LR11XX_CRYPTO_STATUS_ERROR_FAIL_CMAC";
    }

    case LR11XX_CRYPTO_STATUS_ERROR_INVALID_KEY_ID:
    {
        return ( const char* ) "LR11XX_CRYPTO_STATUS_ERROR_INVALID_KEY_ID";
    }

    case LR11XX_CRYPTO_STATUS_ERROR_BUFFER_SIZE:
    {
        return ( const char* ) "LR11XX_CRYPTO_STATUS_ERROR_BUFFER_SIZE";
    }

    case LR11XX_CRYPTO_STATUS_ERROR:
    {
        return ( const char* ) "LR11XX_CRYPTO_STATUS_ERROR";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_crypto_lorawan_version_to_str( const lr11xx_crypto_lorawan_version_t value )
{
    switch( value )
    {
    case LR11XX_CRYPTO_LORAWAN_VERSION_1_0_X:
    {
        return ( const char* ) "LR11XX_CRYPTO_LORAWAN_VERSION_1_0_X";
    }

    case LR11XX_CRYPTO_LORAWAN_VERSION_1_1_X:
    {
        return ( const char* ) "LR11XX_CRYPTO_LORAWAN_VERSION_1_1_X";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_crypto_keys_idx_to_str( const lr11xx_crypto_keys_idx_t value )
{
    switch( value )
    {
    case LR11XX_CRYPTO_KEYS_IDX_MOTHER_KEY:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_MOTHER_KEY";
    }

    case LR11XX_CRYPTO_KEYS_IDX_NWK_KEY:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_NWK_KEY";
    }

    case LR11XX_CRYPTO_KEYS_IDX_APP_KEY:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_APP_KEY";
    }

    case LR11XX_CRYPTO_KEYS_IDX_J_S_ENC_KEY:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_J_S_ENC_KEY";
    }

    case LR11XX_CRYPTO_KEYS_IDX_J_S_INT_KEY:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_J_S_INT_KEY";
    }

    case LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_0:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_0";
    }

    case LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_1:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_1";
    }

    case LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_2:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_2";
    }

    case LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_3:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_3";
    }

    case LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_4:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_4";
    }

    case LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_5:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_GP_KE_KEY_5";
    }

    case LR11XX_CRYPTO_KEYS_IDX_APP_S_KEY:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_APP_S_KEY";
    }

    case LR11XX_CRYPTO_KEYS_IDX_F_NWK_S_INT_KEY:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_F_NWK_S_INT_KEY";
    }

    case LR11XX_CRYPTO_KEYS_IDX_S_NWK_S_INT_KEY:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_S_NWK_S_INT_KEY";
    }

    case LR11XX_CRYPTO_KEYS_IDX_NWK_S_ENC_KEY:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_NWK_S_ENC_KEY";
    }

    case LR11XX_CRYPTO_KEYS_IDX_RFU_0:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_RFU_0";
    }

    case LR11XX_CRYPTO_KEYS_IDX_RFU_1:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_RFU_1";
    }

    case LR11XX_CRYPTO_KEYS_IDX_MC_APP_S_KEY_0:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_MC_APP_S_KEY_0";
    }

    case LR11XX_CRYPTO_KEYS_IDX_MC_APP_S_KEY_1:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_MC_APP_S_KEY_1";
    }

    case LR11XX_CRYPTO_KEYS_IDX_MC_APP_S_KEY_2:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_MC_APP_S_KEY_2";
    }

    case LR11XX_CRYPTO_KEYS_IDX_MC_APP_S_KEY_3:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_MC_APP_S_KEY_3";
    }

    case LR11XX_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_0:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_0";
    }

    case LR11XX_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_1:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_1";
    }

    case LR11XX_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_2:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_2";
    }

    case LR11XX_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_3:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_MC_NWK_S_KEY_3";
    }

    case LR11XX_CRYPTO_KEYS_IDX_GP0:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_GP0";
    }

    case LR11XX_CRYPTO_KEYS_IDX_GP1:
    {
        return ( const char* ) "LR11XX_CRYPTO_KEYS_IDX_GP1";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}
