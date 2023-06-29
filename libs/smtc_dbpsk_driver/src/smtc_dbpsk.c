/**
 * \file
 *
 * \brief Generic DBPSK support library implementation
 */

/**
 * \file
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_dbpsk.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION DEFINITIONS ---------------------------------------------
 */

void smtc_dbpsk_encode_buffer( const uint8_t* data_in, int bpsk_pld_len_in_bits, uint8_t* data_out )
{
    uint8_t in_byte;
    uint8_t out_byte;

    int data_in_bytecount = bpsk_pld_len_in_bits >> 3;
    in_byte               = *data_in++;

    uint8_t current = 0;

    // Process full bytes
    while( --data_in_bytecount >= 0 )
    {
        for( int i = 0; i < 8; ++i )
        {
            out_byte = ( out_byte << 1 ) | current;
            if( ( in_byte & 0x80 ) == 0 )
            {
                current = current ^ 0x01;
            }
            in_byte <<= 1;
        }
        in_byte     = *data_in++;
        *data_out++ = out_byte;
    }

    // Process remaining bits
    for( int i = 0; i < ( bpsk_pld_len_in_bits & 7 ); ++i )
    {
        out_byte = ( out_byte << 1 ) | current;
        if( ( in_byte & 0x80 ) == 0 )
        {
            current = current ^ 0x01;
        }
        in_byte <<= 1;
    }

    // Process last data bit
    out_byte = ( out_byte << 1 ) | current;
    if( ( bpsk_pld_len_in_bits & 7 ) == 7 )
    {
        *data_out++ = out_byte;
    }

    // Add duplicate bit and store
    out_byte  = ( out_byte << 1 ) | current;
    *data_out = out_byte << ( 7 - ( ( bpsk_pld_len_in_bits + 1 ) & 7 ) );
}

/* --- EOF ------------------------------------------------------------------ */
