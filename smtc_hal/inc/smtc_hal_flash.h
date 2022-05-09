/*!
 * @file      smtc_hal_flash.h
 *
 * @brief     Board specific package FLASH API definition.
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

#ifndef SMTC_HAL_FLASH_H
#define SMTC_HAL_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>

#include "smtc_hal.h"

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
 * @brief Initializes the FLASH module and find the first empty page.
 *
 * @returns status [SMTC_HAL_SUCCESS, SMTC_HAL_FAILURE]
 */
smtc_hal_status_t hal_flash_init( void );

/*!
 * @brief Erase a given nb page to the FLASH at the specified address.
 *
 * @param [in] addr FLASH address to start the erase
 * @param [in] nb_page the number of page to erase.
 * @returns status [SMTC_HAL_SUCCESS, SMTC_HAL_FAILURE]
 */
smtc_hal_status_t hal_flash_erase_page( uint32_t addr, uint8_t nb_page );

/*!
 * @brief Force erasing of a given nb page to the FLASH at the specified address.
 *
 * @param [in] addr FLASH address to start the erase
 * @param [in] nb_page the number of page to erase.
 * @returns status [SMTC_HAL_SUCCESS, SMTC_HAL_FAILURE]
 */
smtc_hal_status_t hal_flash_force_erase_page( uint32_t addr, uint8_t nb_page );

/*!
 * @brief Writes the given buffer to the FLASH at the specified address.
 *
 * @param [in] addr FLASH address to write to
 * @param [in] buffer Pointer to the buffer to be written.
 * @param [in] size Size of the buffer to be written.
 * @returns status [Real_size_written, SMTC_HAL_FAILURE]
 */
smtc_hal_status_t hal_flash_write_buffer( uint32_t addr, const uint8_t* buffer, uint32_t size );

/*!
 * @brief Reads the FLASH at the specified address to the given buffer.
 *
 * @param [in] addr FLASH address to read from
 * @param [out] buffer Pointer to the buffer to be written with read data.
 * @param [in] size Size of the buffer to be read.
 * @returns status [SMTC_HAL_SUCCESS, SMTC_HAL_FAILURE]
 */
void hal_flash_read_buffer( uint32_t addr, uint8_t* buffer, uint32_t size );

/*!
 * @brief Reads the FLASH at the specified address to the given buffer.
 *
 * @returns User flash start address.
 */
uint32_t hal_flash_get_user_start_addr( void );

/*!
 * @brief Set the FLASH user start addr.
 *
 * @param [in] addr User flash start address.
 */
void hal_flash_set_user_start_addr( uint32_t addr );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_FLASH_H

/* --- EOF ------------------------------------------------------------------ */
