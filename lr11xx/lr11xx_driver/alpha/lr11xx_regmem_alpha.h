/*!
 * @file      lr11xx_regmem_alpha.h
 *
 * @brief     Register/memory driver definition for LR11XX
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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

#ifndef LR11XX_REGMEM_ALPHA_H
#define LR11XX_REGMEM_ALPHA_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr11xx_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Write words into auxiliary memory space of LR11XX.
 *
 * A word is 32-bit long. The writing operations write contiguously in auxiliary memory, starting at the address
 * provided.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The auxiliary memory address to start writing operation
 * @param [in] data The buffer of words to write into memory. Its size must be enough to contain length words.
 * @param [in] length Number of words to write into memory
 *
 * @returns Operation status
 *
 * @see lr11xx_regmem_read_auxreg32
 */
lr11xx_status_t lr11xx_regmem_write_auxreg32( const void* context, const uint32_t address, const uint32_t* buffer,
                                              const uint8_t length );

/*!
 * @brief Read words into auxiliary memory space of LR11XX.
 *
 * A word is 32-bit long. The reading operations read contiguously from auxiliary memory, starting at the address
 * provided.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The auxiliary memory address to start reading operation
 * @param [in] length Number of words to read from memory
 * @param [out] buffer Pointer to a words array to be filled with content from memory. Its size must be enough to
 * contain at least length words.
 *
 * @returns Operation status
 *
 * @see lr11xx_regmem_write_auxreg32
 */
lr11xx_status_t lr11xx_regmem_read_auxreg32( const void* context, const uint32_t address, uint32_t* buffer,
                                             const uint8_t length );

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_REGMEM_ALPHA_H

/* --- EOF ------------------------------------------------------------------ */
