/*!
 * @file      lr11xx_crypto_engine_types_str.c
 *
 * @brief     Printer helper functions for LR11xx crypto engine types
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
