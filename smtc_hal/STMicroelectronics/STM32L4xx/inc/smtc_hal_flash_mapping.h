/*!
 * \file      smtc_hal_flash.c
 *
 * \brief     FLASH Hardware Abstraction Layer implementation
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

#ifndef SMTC_HAL_FLASH_MAPPING_H
#define SMTC_HAL_FLASH_MAPPING_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

#define ADDR_FLASH_PAGE_SIZE ( ( uint32_t ) 0x00000800 ) /* Size of Page = 2 KBytes */

#define FLASH_BYTE_EMPTY_CONTENT ( ( uint8_t ) 0xFF )
#define FLASH_PAGE_EMPTY_CONTENT ( ( uint64_t ) 0xFFFFFFFFFFFFFFFF )

#define FLASH_USER_START_PAGE ( 1 ) /* Start nb page of user Flash area */

#define FLASH_END_ADDR_OF_PAGE( page ) \
    ( ADDR_FLASH_PAGE( page ) + ADDR_FLASH_PAGE_SIZE - 1 )                    /* Last memory address of a flash page */
#define FLASH_USER_END_ADDR ( FLASH_END_ADDR_OF_PAGE( FLASH_USER_END_PAGE ) ) /* End @ of user Flash area */
#define FLASH_USER_END_PAGE ( 506 )                                           /* End nb page of user Flash area */

#define FLASH_USER_INTERNAL_LOG_CTX_START_PAGE ( FLASH_USER_END_PAGE )

#define FLASH_USER_INTERNAL_LOG_CTX_START_ADDR ADDR_FLASH_PAGE( FLASH_USER_INTERNAL_LOG_CTX_START_PAGE )
#define FLASH_USER_INTERNAL_LOG_CTX_END_ADDR ( FLASH_END_ADDR_OF_PAGE( FLASH_USER_INTERNAL_LOG_CTX_START_PAGE ) )

/* Base address of the Flash s */
#define ADDR_FLASH_PAGE_0 ( ( uint32_t ) 0x08000000 ) /* Base @ of Page 0, 2 KBytes */
#define ADDR_FLASH_PAGE( page ) ( ADDR_FLASH_PAGE_0 + ( page ) *ADDR_FLASH_PAGE_SIZE )

#endif  // SMTC_HAL_FLASH_MAPPING_H

/* --- EOF ------------------------------------------------------------------ */
