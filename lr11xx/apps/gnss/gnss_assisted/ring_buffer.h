/*!
 * @file      ring_buffer.h
 *
 * @brief     Ring buffer implementation
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

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * @brief Enable ring buffer
 *
 * Enabling ring buffer allows UART to push data
 */
void ring_buffer_enable( void );

/**
 * @brief Disable ring buffer
 *
 * Disabling ring buffer discard the next UART attempts to push data
 */
void ring_buffer_disable( void );

/**
 * @brief Indicate if the ring buffer overran since last reset of the buffer overrun flag
 *
 * Buffer overrun occurs when an attempt to push more data than the size of the buffer is made.
 *
 * @return true : A buffer overrun occurred
 * @return false : No buffer overrun occurred
 */
bool has_buffer_overrun( );

/**
 * @brief Reset the buffer overrun flag
 */
void reset_buffer_overrun_flag( );

/**
 * @brief Push new data to the ring buffer
 *
 * @param _char The new data to push
 *
 * @return true The push is successful
 * @return false The push failed because there is no more room available in the buffer, or the ring buffer is disabled
 */
bool append_to_interrupt_buffer( int _char );

/**
 * @brief Pop data from the buffer
 *
 * If there is a data to pop, the data is copied to the location provided in argument, then an internal index is
 * incremented. So the data is not physically removed from the buffer, it is made unreachable. The data will eventually
 * be erased next time a push operation occurs on the memory slot the data is.
 *
 * @param _char Pointer to the location where the data to push should be copied
 *
 * @return true The operation is successful and _char points to a value that is a copy of the data pop'd
 * @return false The operation failed as there is no data to pop and _char should not be used
 */
bool pop_from_interrupt_buffer( int* _char );

#ifdef __cplusplus
}
#endif

#endif  // RING_BUFFER_H