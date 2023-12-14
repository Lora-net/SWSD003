/*!
 * @file      lr11xx_lr_fhss_types_str.c
 *
 * @brief     Printer helper functions for LR11xx LRFHSS types
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
 
#include "lr11xx_lr_fhss_types_str.h"

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
