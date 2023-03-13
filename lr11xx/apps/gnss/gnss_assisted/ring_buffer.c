/*!
 * @file      ring_buffer.c
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

#include <stdint.h>
#include <stdbool.h>

#define MAX_INTERRUPT_FILLED_BUFFER_LENGTH ( 512 )
volatile static int      interrupt_filled_buffer[MAX_INTERRUPT_FILLED_BUFFER_LENGTH] = { };
volatile static uint16_t index_head                                                  = 0;
volatile static uint16_t index_tail                                                  = 0;
volatile static bool     buffer_overrun                                              = false;
static bool              is_enabled                                                  = false;

void ring_buffer_enable( void )
{
    is_enabled = true;
}

void ring_buffer_disable( void )
{
    is_enabled = false;
}

bool has_buffer_overrun( )
{
    return buffer_overrun;
}

void reset_buffer_overrun_flag( )
{
    buffer_overrun = false;
}

bool append_to_interrupt_buffer( int _char )
{
    if( is_enabled == false )
    {
        // If the ring buffer is disabled, attempt to push data is discarded
        return false;
    }
    const uint16_t next_head_index = ( index_head + 1 ) % MAX_INTERRUPT_FILLED_BUFFER_LENGTH;
    if( next_head_index != index_tail )
    {
        index_head                          = next_head_index;
        interrupt_filled_buffer[index_head] = _char;
        return true;
    }
    else
    {
        buffer_overrun = true;
        return false;
    }
}

bool pop_from_interrupt_buffer( int* _char )
{
    if( index_head != index_tail )
    {
        index_tail = ( index_tail + 1 ) % MAX_INTERRUPT_FILLED_BUFFER_LENGTH;
        *_char     = interrupt_filled_buffer[index_tail];
        return true;
    }
    else
    {
        return false;
    }
}