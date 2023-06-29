/**
 * \file
 *
 * \brief Generic DBPSK support library header
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

#ifndef SMTC_DBPSK_H
#define SMTC_DBPSK_H

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION PROTOTYPES ----------------------------------------------
 */

/*!
 * \brief Perform differential encoding for DBPSK modulation.
 *
 * \param [in]  data_in              Buffer with input data
 * \param [in]  bpsk_pld_len_in_bits Length of the input BPSK frame, in bits
 * \param [out] data_out             Buffer for output data (can optionally be the same as data_in, but must have space
 * for bpsk_pld_len_in_bits + 2 bits)
 */
void smtc_dbpsk_encode_buffer( const uint8_t* data_in, int bpsk_pld_len_in_bits, uint8_t* data_out );

/*!
 * \brief Given the length of a BPSK frame, in bits, calculate the space necessary to hold the frame after differential
 * encoding, in bits.
 *
 * \param [in]  bpsk_pld_len_in_bits Length of a BPSK frame, in bits
 * \returns                          Space required for DBPSK frame, after addition of start/stop bits [bits]
 */
static inline int smtc_dbpsk_get_pld_len_in_bits( int bpsk_pld_len_in_bits )
{
    // Hold the last bit one extra bit-time
    return bpsk_pld_len_in_bits + 2;
}

/*!
 * \brief Given the length of a BPSK frame, in bits, calculate the space necessary to hold the frame after differential
 * encoding, in bytes.
 *
 * \param [in]  bpsk_pld_len_in_bits Length of a BPSK frame, in bits
 * \returns                          Space required for DBPSK frame, after addition of start/stop bits [bytes]
 */
static inline int smtc_dbpsk_get_pld_len_in_bytes( int bpsk_pld_len_in_bits )
{
    return ( smtc_dbpsk_get_pld_len_in_bits( bpsk_pld_len_in_bits ) + 7 ) >> 3;
}

#ifdef __cplusplus
}
#endif

#endif  // SMTC_DBPSK_H

/* --- EOF ------------------------------------------------------------------ */
