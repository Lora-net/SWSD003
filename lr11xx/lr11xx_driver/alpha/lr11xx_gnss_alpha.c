/*!
 * @file      lr11xx_gnss_alpha.c
 *
 * @brief     GNSS scan driver implementation for LR11XX - alpha
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_gnss_alpha.h"
#include "lr11xx_regmem.h"
#include "lr11xx_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR11XX_GNSS_SCAN_GET_RES_SIZE_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_SCAN_READ_RES_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_ALMANAC_READ_CMD_LENGTH ( 2 )
#define LR11XX_GNSS_SET_XTAL_ERROR_CMD_LENGTH ( 2 + 2 )
#define LR11XX_GNSS_READ_XTAL_ERROR_CMD_LENGTH ( 2 )

#define LR11XX_GNSS_ALMANAC_READ_RBUFFER_LENGTH ( 6 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    LR11XX_GNSS_ALMANAC_READ_OC            = 0x040F,  //!< Read all almanacs
    LR11XX_GNSS_SET_XTAL_ERROR_OC          = 0x0412,  //!< Set the xtal accuracy
    LR11XX_GNSS_READ_XTAL_ERROR_OC         = 0x0413,  //!< Read the xtal accuracy
    LR11XX_GNSS_GET_RES_SIZE_FIRST_SCAN_OC = 0x0425,  //!< get the size of the output payload for debug first scan
    LR11XX_GNSS_READ_RES_FIRST_SCAN_OC     = 0x0426,  //!< read the byte stream for debug first scan
    LR11XX_GNSS_GET_RES_SIZE_INTERMEDIATE_SCAN_OC =
        0x0429,  //!< get the size of the output payload for debug intermediate scan
    LR11XX_GNSS_READ_RES_INTERMEDIATE_SCAN_OC = 0x042A,  //!< read the byte stream for debug intermediate
} lr11xx_gnssOpCodeAlpha_t;

typedef enum
{
    LR11XX_GNSS_DEBUG_SCAN_TYPE_FIRST,
    LR11XX_GNSS_DEBUG_SCAN_TYPE_INTERMEDIATE,
} lr11xx_gnss_debug_scan_type;

/*!
 * @brief Helper function that returns the get result size opcode corresponding to the debug scan type
 *
 * @param [in] debug_scan_type The debug scan type
 * @param [out] get_res_size_oc The opcode for get result size operation
 *
 * @retval LR11XX_STATUS_OK The operation succeeded and get_res_size_oc can be read
 * @retval LR11XX_STATUS_ERROR The operation failed and get_res_size_oc must not be used
 */
static lr11xx_status_t lr11xx_gnss_get_get_res_size_oc( lr11xx_gnss_debug_scan_type debug_scan_type,
                                                        lr11xx_gnssOpCodeAlpha_t*   get_res_size_oc );

/*!
 * @brief Helper function that returns the read result opcode corresponding to the debug scan type
 *
 * @param [in] debug_scan_type The debug scan type
 * @param [out] read_res_oc The opcode for read result operation
 *
 * @retval LR11XX_STATUS_OK The operation succeeded and read_res_oc can be read
 * @retval LR11XX_STATUS_ERROR The operation failed and read_res_oc must not be used
 */
static lr11xx_status_t lr11xx_gnss_get_read_res_oc( lr11xx_gnss_debug_scan_type debug_scan_type,
                                                    lr11xx_gnssOpCodeAlpha_t*   read_res_oc );

/*!
 * @brief Helper function that calls get res size operation on the radio
 *
 * @param [in] context Chip implementation context
 * @param [in] debug_scan_type The debug scan type
 * @param [out] result_size The result size
 *
 * @returns Operation status
 */
static lr11xx_status_t lr11xx_gnss_get_res_size_base( const void* context, lr11xx_gnss_debug_scan_type debug_scan_type,
                                                      uint16_t* result_size );

/*!
 * @brief Helper function that calls read GNSS results on the radio
 *
 * The GNSS results are pushed into a buffer directly. This buffer is provided by the application using the driver. It
 * MUST be long enough to contains at least result_buffer_size bytes.
 *
 * @warning No check is done on result_buffer size. If this application provided buffer is too small, there will be a
 * buffer overflow bug!
 *
 * @param [in] context Chip implementation context
 * @param [in] debug_scan_type The debug scan type
 * @param [out] result_buffer Application provided buffer to be filled with result
 * @param [in] result_buffer_size The number of bytes to read from the LR11XX
 *
 * @returns Operation status
 */
static lr11xx_status_t lr11xx_gnss_read_res_base( const void* context, lr11xx_gnss_debug_scan_type debug_scan_type,
                                                  uint8_t* result_buffer, uint16_t result_buffer_size );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Get the almanac base address and size
 *
 * @param [in]  context Chip implementation context
 * @param [out] address Start address of the almanac in memory
 * @param [out] size    Size of the almanac in byte
 *
 * @returns Operation status
 */
static lr11xx_status_t lr11xx_gnss_get_almanac_address_size( const void* context, uint32_t* address, uint16_t* size );

/*!
 * @brief Helper function that convert an array of uint8_t into a uint32_t single value
 *
 * @warning It is up to the caller to ensure that value points to an array of at least sizeof(uint32_t) elements.
 *
 * @param [in] value Array of uint8_t to be translated into a uint32_t
 *
 * @returns 32-bit value
 */
static uint32_t lr11xx_gnss_uint8_to_uint32( uint8_t value[4] );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_gnss_get_first_scan_result_size( const void* context, uint16_t* result_size )
{
    return lr11xx_gnss_get_res_size_base( context, LR11XX_GNSS_DEBUG_SCAN_TYPE_FIRST, result_size );
}

lr11xx_status_t lr11xx_gnss_read_first_scan_results( const void* context, uint8_t* result_buffer,
                                                     const uint16_t result_buffer_size )
{
    return lr11xx_gnss_read_res_base( context, LR11XX_GNSS_DEBUG_SCAN_TYPE_FIRST, result_buffer, result_buffer_size );
}

lr11xx_status_t lr11xx_gnss_get_intermediate_scan_result_size( const void* context, uint16_t* result_size )
{
    return lr11xx_gnss_get_res_size_base( context, LR11XX_GNSS_DEBUG_SCAN_TYPE_INTERMEDIATE, result_size );
}

lr11xx_status_t lr11xx_gnss_read_intermediate_scan_results( const void* context, uint8_t* result_buffer,
                                                            const uint16_t result_buffer_size )
{
    return lr11xx_gnss_read_res_base( context, LR11XX_GNSS_DEBUG_SCAN_TYPE_INTERMEDIATE, result_buffer,
                                      result_buffer_size );
}

lr11xx_status_t lr11xx_gnss_get_get_res_size_oc( lr11xx_gnss_debug_scan_type debug_scan_type,
                                                 lr11xx_gnssOpCodeAlpha_t*   get_res_size_oc )
{
    lr11xx_status_t command_status = LR11XX_STATUS_ERROR;
    switch( debug_scan_type )
    {
    case LR11XX_GNSS_DEBUG_SCAN_TYPE_FIRST:
    {
        *get_res_size_oc = LR11XX_GNSS_GET_RES_SIZE_FIRST_SCAN_OC;
        command_status   = LR11XX_STATUS_OK;
        break;
    }
    case LR11XX_GNSS_DEBUG_SCAN_TYPE_INTERMEDIATE:
    {
        *get_res_size_oc = LR11XX_GNSS_GET_RES_SIZE_INTERMEDIATE_SCAN_OC;
        command_status   = LR11XX_STATUS_OK;
        break;
    }
    default:
    {
        command_status = LR11XX_STATUS_ERROR;
    }
    }
    return command_status;
}

lr11xx_status_t lr11xx_gnss_get_read_res_oc( lr11xx_gnss_debug_scan_type debug_scan_type,
                                             lr11xx_gnssOpCodeAlpha_t*   read_res_oc )
{
    lr11xx_status_t command_status = LR11XX_STATUS_ERROR;
    switch( debug_scan_type )
    {
    case LR11XX_GNSS_DEBUG_SCAN_TYPE_FIRST:
    {
        *read_res_oc   = LR11XX_GNSS_READ_RES_FIRST_SCAN_OC;
        command_status = LR11XX_STATUS_OK;
        break;
    }
    case LR11XX_GNSS_DEBUG_SCAN_TYPE_INTERMEDIATE:
    {
        *read_res_oc   = LR11XX_GNSS_READ_RES_INTERMEDIATE_SCAN_OC;
        command_status = LR11XX_STATUS_OK;
        break;
    }
    default:
    {
        command_status = LR11XX_STATUS_ERROR;
    }
    }
    return command_status;
}

lr11xx_status_t lr11xx_gnss_get_almanac_crc( const void* context, uint32_t* almanac_crc )
{
    uint32_t              almanac_base_address = 0;
    uint16_t              almanac_size         = 0;
    const lr11xx_status_t status_get_almanac_address_size =
        lr11xx_gnss_get_almanac_address_size( context, &almanac_base_address, &almanac_size );

    if( status_get_almanac_address_size != LR11XX_STATUS_OK )
    {
        return status_get_almanac_address_size;
    }

    const uint16_t offset_almanac = 128 * LR11XX_GNSS_SINGLE_ALMANAC_READ_SIZE;

    const lr11xx_status_t status_read_mem =
        lr11xx_regmem_read_regmem32( context, almanac_base_address + offset_almanac, almanac_crc, 1 );
    return status_read_mem;
}

lr11xx_status_t lr11xx_gnss_set_xtal_error( const void* context, const float xtal_error_in_ppm )
{
    const uint8_t cbuffer[LR11XX_GNSS_SET_XTAL_ERROR_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_GNSS_SET_XTAL_ERROR_OC >> 8 ),
        ( uint8_t ) ( LR11XX_GNSS_SET_XTAL_ERROR_OC >> 0 ),
        ( uint8_t ) ( ( int16_t ) ( ( xtal_error_in_ppm * 32768 ) / 40 ) >> 8 ),
        ( uint8_t ) ( ( int16_t ) ( ( xtal_error_in_ppm * 32768 ) / 40 ) >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_GNSS_SET_XTAL_ERROR_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_gnss_read_xtal_error( const void* context, float* xtal_error_in_ppm )
{
    uint8_t       xtal_error_buffer[2] = { 0x00 };
    int16_t       xtal_error_temp;
    const uint8_t cbuffer[LR11XX_GNSS_READ_XTAL_ERROR_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_GNSS_READ_XTAL_ERROR_OC >> 8 ),
        ( uint8_t ) ( LR11XX_GNSS_READ_XTAL_ERROR_OC >> 0 ),
    };

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_READ_XTAL_ERROR_CMD_LENGTH,
                                                            xtal_error_buffer, sizeof( xtal_error_buffer ) );

    xtal_error_temp    = ( ( ( uint16_t ) xtal_error_buffer[0] << 8 ) + xtal_error_buffer[1] );
    *xtal_error_in_ppm = ( ( float ) ( xtal_error_temp ) *40 ) / 32768;
    return ( lr11xx_status_t ) hal_status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

lr11xx_status_t lr11xx_gnss_get_almanac_address_size( const void* context, uint32_t* address, uint16_t* size )
{
    const uint8_t cbuffer[LR11XX_GNSS_ALMANAC_READ_CMD_LENGTH] = {
        ( uint8_t ) ( LR11XX_GNSS_ALMANAC_READ_OC >> 8 ),
        ( uint8_t ) ( LR11XX_GNSS_ALMANAC_READ_OC >> 0 ),
    };
    uint8_t rbuffer[LR11XX_GNSS_ALMANAC_READ_RBUFFER_LENGTH] = { 0 };

    const lr11xx_hal_status_t hal_status = lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_ALMANAC_READ_CMD_LENGTH,
                                                            rbuffer, LR11XX_GNSS_ALMANAC_READ_RBUFFER_LENGTH );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *address = lr11xx_gnss_uint8_to_uint32( &rbuffer[0] );
        *size    = ( ( ( uint16_t ) rbuffer[4] ) << 8 ) | ( ( uint16_t ) rbuffer[5] );
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_get_res_size_base( const void* context, lr11xx_gnss_debug_scan_type debug_scan_type,
                                               uint16_t* result_size )
{
    uint16_t              get_res_oc             = 0;
    const lr11xx_status_t get_res_size_oc_status = lr11xx_gnss_get_get_res_size_oc( debug_scan_type, &get_res_oc );
    if( get_res_size_oc_status != LR11XX_STATUS_OK )
    {
        return get_res_size_oc_status;
    }

    const uint8_t cbuffer[LR11XX_GNSS_SCAN_GET_RES_SIZE_CMD_LENGTH] = {
        ( uint8_t ) ( get_res_oc >> 8 ),
        ( uint8_t ) ( get_res_oc >> 0 ),
    };
    uint8_t rbuffer[sizeof( uint16_t )] = { 0 };

    const lr11xx_hal_status_t hal_status =
        lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_SCAN_GET_RES_SIZE_CMD_LENGTH, rbuffer, sizeof( uint16_t ) );

    if( hal_status == LR11XX_HAL_STATUS_OK )
    {
        *result_size = ( ( uint16_t ) rbuffer[0] << 8 ) + ( ( uint16_t ) rbuffer[1] );
    }

    return ( lr11xx_status_t ) hal_status;
}

lr11xx_status_t lr11xx_gnss_read_res_base( const void* context, lr11xx_gnss_debug_scan_type debug_scan_type,
                                           uint8_t* result_buffer, uint16_t result_buffer_size )
{
    uint16_t              read_res_oc            = 0;
    const lr11xx_status_t get_res_size_oc_status = lr11xx_gnss_get_read_res_oc( debug_scan_type, &read_res_oc );
    if( get_res_size_oc_status != LR11XX_STATUS_OK )
    {
        return get_res_size_oc_status;
    }
    const uint8_t cbuffer[LR11XX_GNSS_SCAN_READ_RES_CMD_LENGTH] = {
        ( uint8_t ) ( read_res_oc >> 8 ),
        ( uint8_t ) ( read_res_oc >> 0 ),
    };

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_GNSS_SCAN_READ_RES_CMD_LENGTH, result_buffer,
                                                result_buffer_size );
}

uint32_t lr11xx_gnss_uint8_to_uint32( uint8_t* value )
{
    return ( ( ( uint32_t ) value[0] ) << 24 ) + ( ( ( uint32_t ) value[1] ) << 16 ) +
           ( ( ( uint32_t ) value[2] ) << 8 ) + ( ( ( uint32_t ) value[3] ) << 0 );
}

/* --- EOF ------------------------------------------------------------------ */
