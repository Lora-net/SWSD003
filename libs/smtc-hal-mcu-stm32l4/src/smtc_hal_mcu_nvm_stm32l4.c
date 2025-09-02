/*!
 * @file      smtc_hal_mcu_nvm_stm32l4.c
 *
 * @brief      Implementation of NVM module on top of STM32L4 HAL drivers
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
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

#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal_flash.h"
#include "smtc_hal_mcu_nvm.h"
#include "smtc_hal_mcu_nvm_stm32l4.h"
#include "stm32l476xx.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Evaluates the current status and forces the calling function to return in case the status is !=
 * NVM_MANAGER_OK
 *
 * @param status the status to be evaluated
 *
 * @return If the status is != SMTC_HAL_MCU_STATUS_OK, returns it
 */

#define _SMTC_HAL_MCU_NVM_RETURN_ON_ERR( status )          \
    do                                                     \
    {                                                      \
        const smtc_hal_mcu_status_t local_status = status; \
        if( local_status != SMTC_HAL_MCU_STATUS_OK )       \
        {                                                  \
            return local_status;                           \
        }                                                  \
    } while( 0 );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#ifndef SMTC_HAL_MCU_NVM_STM32L4_ARRAY_SIZE
#define SMTC_HAL_MCU_NVM_STM32L4_ARRAY_SIZE 16
#endif  // SMTC_HAL_MCU_NVM_STM32L4_ARRAY_SIZE

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static struct smtc_hal_mcu_nvm_inst_s nvm_inst_array[SMTC_HAL_MCU_NVM_STM32L4_ARRAY_SIZE];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Checks the specified offset is within the boundaries of the user flash memories
 *
 * @param [in] inst NVM instance
 * @param offset the NVM write/read id to check
 *
 * @retval true if specified offset is within the boundaries
 * @retval false if specified offset is NOT within the boundaries
 */
static bool smtc_hal_mcu_nvm_smt32l4_is_offset_within_bounds( smtc_hal_mcu_nvm_inst_t inst, uint32_t offset );

/**
 * @brief  Writes a doubleword in flash
 *
 * @param doubleword doubleword to be written
 * @param address address to write the doubleword to
 *
 * @retval The result of the operation
 */
static smtc_hal_mcu_status_t smtc_hal_mcu_nvm_stm32l4_write_doubleword( uint64_t doubleword, uint32_t address );

/**
 * @brief  Erases the specified pages in flash
 *
 * @param  bank_number bank of flash memory that the pages lays on
 * @param  first_page first page number
 * @param  pages_number number of pages to be erased
 *
 * @retval The result of the operation
 */
static smtc_hal_mcu_status_t smtc_hal_mcu_nvm_stm32l4_erase_pages( uint32_t bank_number, uint32_t first_page,
                                                                   uint32_t pages_number );

/**
 * @brief Unlocks the NVM and makes it available for further operations
 *
 * @retval SMTC_HAL_MCU_STATUS_OK NVM has been stopped successfully
 * @retval SMTC_HAL_MCU_STATUS_ERROR Operation failed because another error occurred
 */
static smtc_hal_mcu_status_t smtc_hal_mcu_nvm_stm32l4_unlock( void );

/**
 * @brief Locks the NVM and makes it unavailable for further operations
 *
 * @retval SMTC_HAL_MCU_STATUS_OK NVM has been stopped successfully
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT Operation failed as the NVM is not initialised
 * @retval SMTC_HAL_MCU_STATUS_ERROR Operation failed because another error occurred
 */
static smtc_hal_mcu_status_t smtc_hal_mcu_nvm_stm32l4_lock( void );

/**
 * @brief  Gets the bank of a given address
 *
 * @param  addr address of the FLASH Memory
 *
 * @retval The bank of a given address
 */
static uint32_t smtc_hal_mcu_nvm_stm32l4_get_bank_by_address( uint32_t addr );

/**
 * @brief  Gets the page of a given address
 *
 * @param  addr Address of the FLASH Memory
 *
 * @retval The page of a given address
 */
static uint32_t smtc_hal_mcu_nvm_stm32l4_get_page_by_address( uint32_t addr );

/**
 * @brief  Returns a 64bits unsigned integer from a buffer
 *
 * @param  buf buffer containing the data
 * @param  len length of the data to be written, max is sizeof(uint64_t)
 *
 * @retval The 64bits value representing the data. If data length is less than 64 bits, the rest of the data is filled
 * with 0s
 */
static uint64_t smtc_hal_mcu_nvm_stm32l4_create_64_bits_data( const uint8_t* buf, uint8_t len );

/**
 * @brief  Checks if the instance is a valid nvm instance
 *
 * @param [in] inst NVM instance
 *
 * @retval true if valid,
 * @retval false if not valid
 */
static bool smtc_hal_mcu_nvm_stm32l4_is_valid_instance( const smtc_hal_mcu_nvm_inst_t inst );

/**
 * @brief Get a pointer to the first free slot
 *
 * @retval Pointer to the first free slot - NULL if there is no space left
 */
static smtc_hal_mcu_nvm_inst_t smtc_hal_mcu_nvm_stm32l4_get_free_slot( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_hal_mcu_status_t smtc_hal_mcu_nvm_init( uint32_t start_address, uint32_t end_address,
                                             smtc_hal_mcu_nvm_inst_t* inst )
{
    smtc_hal_mcu_nvm_inst_t allocated_inst = smtc_hal_mcu_nvm_stm32l4_get_free_slot( );

    if( allocated_inst == NULL )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    if( ( start_address < FLASH_BASE ) || ( end_address > FLASH_END ) || ( start_address >= end_address ) )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    // Check that this allocation will not overlap existing instance
    for( uint8_t index = 0; index < SMTC_HAL_MCU_NVM_STM32L4_ARRAY_SIZE; index++ )
    {
        if( ( ( nvm_inst_array[index].start_address >= start_address ) &&
              ( start_address <= nvm_inst_array[index].end_address ) ) ||
            ( ( nvm_inst_array[index].start_address >= end_address ) &&
              ( end_address <= nvm_inst_array[index].end_address ) ) )
        {
            // Asked start or address within exising instance boundaries
            return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
        }
    }

    allocated_inst->start_address = start_address;
    allocated_inst->end_address   = end_address;
    allocated_inst->is_cfged      = true;

    *inst = allocated_inst;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_nvm_deinit( smtc_hal_mcu_nvm_inst_t* inst )
{
    (*inst)->is_cfged = false;
    *inst = NULL;
    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_nvm_write( smtc_hal_mcu_nvm_inst_t inst, uint32_t offset, const uint8_t* buffer,
                                              uint32_t length )
{
    smtc_hal_mcu_status_t status;

    if( smtc_hal_mcu_nvm_stm32l4_is_valid_instance( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }
    if( inst->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }
    if( length == 0 )
    {
        return SMTC_HAL_MCU_STATUS_OK;
    }

    uint32_t buf_index = 0;
    uint32_t address   = offset + inst->start_address;

    if( ( !smtc_hal_mcu_nvm_smt32l4_is_offset_within_bounds( inst, offset ) ) ||
        ( !smtc_hal_mcu_nvm_smt32l4_is_offset_within_bounds( inst, offset + length ) ) )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    status = smtc_hal_mcu_nvm_stm32l4_unlock( );
    _SMTC_HAL_MCU_NVM_RETURN_ON_ERR( status );

    while( buf_index < length )
    {
        uint8_t data64_meaningful_len =
            ( ( length - buf_index ) >= sizeof( uint64_t ) ) ? sizeof( uint64_t ) : ( length - buf_index );

        uint64_t data_to_be_written =
            smtc_hal_mcu_nvm_stm32l4_create_64_bits_data( buffer + buf_index, data64_meaningful_len );

        status = smtc_hal_mcu_nvm_stm32l4_write_doubleword( data_to_be_written, address );
        _SMTC_HAL_MCU_NVM_RETURN_ON_ERR( status );

        buf_index += data64_meaningful_len;
        address += sizeof( uint64_t );
    }

    status = smtc_hal_mcu_nvm_stm32l4_lock( );
    _SMTC_HAL_MCU_NVM_RETURN_ON_ERR( status );

    return status;
}

smtc_hal_mcu_status_t smtc_hal_mcu_nvm_read( smtc_hal_mcu_nvm_inst_t inst, uint32_t offset, uint8_t* buffer,
                                             uint32_t length )
{
    if( length == 0 )
    {
        return SMTC_HAL_MCU_STATUS_OK;
    }

    if( smtc_hal_mcu_nvm_stm32l4_is_valid_instance( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( ( !smtc_hal_mcu_nvm_smt32l4_is_offset_within_bounds( inst, offset ) ) ||
        ( !smtc_hal_mcu_nvm_smt32l4_is_offset_within_bounds( inst, offset + length ) ) )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    const uint32_t start_address = offset + inst->start_address;
    for( uint32_t address = start_address; ( address < ( start_address + length ) ) && address < inst->end_address;
         address += 1 )
    {
        *buffer = *( ( uint8_t* ) address );
        buffer++;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_nvm_erase( smtc_hal_mcu_nvm_inst_t inst, uint32_t offset, uint32_t length )
{
    if( smtc_hal_mcu_nvm_stm32l4_is_valid_instance( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    uint32_t address      = offset + inst->start_address;
    uint32_t bank_number  = smtc_hal_mcu_nvm_stm32l4_get_bank_by_address( address );
    uint32_t first_page   = smtc_hal_mcu_nvm_stm32l4_get_page_by_address( address );
    uint32_t pages_number = smtc_hal_mcu_nvm_stm32l4_get_page_by_address( address + length ) - first_page + 1;

    return smtc_hal_mcu_nvm_stm32l4_erase_pages( bank_number, first_page, pages_number );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool smtc_hal_mcu_nvm_stm32l4_is_valid_instance( const smtc_hal_mcu_nvm_inst_t inst )
{
    for( int i = 0; i < SMTC_HAL_MCU_NVM_STM32L4_ARRAY_SIZE; i++ )
    {
        if( inst == &nvm_inst_array[i] )
        {
            return true;
        }
    }
    return false;
}

static smtc_hal_mcu_status_t smtc_hal_mcu_nvm_stm32l4_unlock( void )
{
    if( HAL_FLASH_Unlock( ) != HAL_OK )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

static smtc_hal_mcu_status_t smtc_hal_mcu_nvm_stm32l4_lock( void )
{
    if( HAL_FLASH_Lock( ) != HAL_OK )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

static bool smtc_hal_mcu_nvm_smt32l4_is_offset_within_bounds( smtc_hal_mcu_nvm_inst_t inst, uint32_t offset )
{
    return ( ( offset + inst->start_address ) <= inst->end_address );
}

static smtc_hal_mcu_status_t smtc_hal_mcu_nvm_stm32l4_write_doubleword( uint64_t doubleword, uint32_t address )
{
    smtc_hal_mcu_status_t status = SMTC_HAL_MCU_STATUS_OK;

    if( HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD, address, doubleword ) != HAL_OK )
    {
        status = SMTC_HAL_MCU_STATUS_ERROR;
    }

    return status;
}

static smtc_hal_mcu_status_t smtc_hal_mcu_nvm_stm32l4_erase_pages( uint32_t bank_number, uint32_t first_page,
                                                                   uint32_t pages_number )
{
    smtc_hal_mcu_status_t status = SMTC_HAL_MCU_STATUS_OK;
    uint32_t              page_error;

    FLASH_EraseInitTypeDef EraseInitStruct;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks     = bank_number;
    EraseInitStruct.Page      = first_page;
    EraseInitStruct.NbPages   = pages_number;

    status = smtc_hal_mcu_nvm_stm32l4_unlock( );
    _SMTC_HAL_MCU_NVM_RETURN_ON_ERR( status );

    __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_OPTVERR );

    if( HAL_FLASHEx_Erase( &EraseInitStruct, &page_error ) != HAL_OK )
    {
        status = SMTC_HAL_MCU_STATUS_ERROR;
        smtc_hal_mcu_nvm_stm32l4_lock( );
        return status;
    }

    status = smtc_hal_mcu_nvm_stm32l4_lock( );
    _SMTC_HAL_MCU_NVM_RETURN_ON_ERR( status );

    return status;
}

static uint64_t smtc_hal_mcu_nvm_stm32l4_create_64_bits_data( const uint8_t* buf, uint8_t len )
{
    uint64_t data64 = 0;
    if( len > sizeof( uint64_t ) )
    {
        return 0;
    }

    for( uint8_t i = 0; i < len; i++ )
    {
        const uint64_t temp = ( ( uint64_t ) *buf << ( i * 8 ) );
        data64 |= temp;
        buf++;
    }

    return data64;
}

static uint32_t smtc_hal_mcu_nvm_stm32l4_get_bank_by_address( uint32_t addr )
{
    uint32_t bank = 0;

    if( READ_BIT( SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE ) == 0 )
    {
        /* No Bank swap */
        if( addr < ( FLASH_BASE + FLASH_BANK_SIZE ) )
        {
            bank = FLASH_BANK_1;
        }
        else
        {
            bank = FLASH_BANK_2;
        }
    }
    else
    {
        /* Bank swap */
        if( addr < ( FLASH_BASE + FLASH_BANK_SIZE ) )
        {
            bank = FLASH_BANK_2;
        }
        else
        {
            bank = FLASH_BANK_1;
        }
    }

    return bank;
}

static uint32_t smtc_hal_mcu_nvm_stm32l4_get_page_by_address( uint32_t addr )
{
    uint32_t page = 0;

    if( addr < ( FLASH_BASE + FLASH_BANK_SIZE ) )
    {
        /* Bank 1 */
        page = ( addr - FLASH_BASE ) / FLASH_PAGE_SIZE;
    }
    else
    {
        /* Bank 2 */
        page = ( addr - ( FLASH_BASE + FLASH_BANK_SIZE ) ) / FLASH_PAGE_SIZE;
    }

    return page;
}

static smtc_hal_mcu_nvm_inst_t smtc_hal_mcu_nvm_stm32l4_get_free_slot( void )
{
    for( int i = 0; i < SMTC_HAL_MCU_NVM_STM32L4_ARRAY_SIZE; i++ )
    {
        if( nvm_inst_array[i].is_cfged == false )
        {
            return &nvm_inst_array[i];
        }
    }

    return NULL;
}

/* --- EOF ------------------------------------------------------------------ */
