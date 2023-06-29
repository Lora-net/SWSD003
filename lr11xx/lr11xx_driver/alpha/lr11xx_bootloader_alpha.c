#include "lr11xx_bootloader_alpha.h"
#include "lr11xx_hal.h"

#define LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 ( 64 )
#define LR11XX_FLASH_DATA_MAX_LENGTH_UINT8 ( LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 * 4 )

#define LR11XX_BL_CMD_NO_PARAM_LENGTH 2
#define LR11XX_BL_GET_HASH_CMD_LENGTH ( LR11XX_BL_CMD_NO_PARAM_LENGTH )
#define LR11XX_BL_WRITE_FLASH_CMD_LENGTH ( LR11XX_BL_CMD_NO_PARAM_LENGTH + 4 )
#define LR11XX_BL_ERASE_PAGE_CMD_LENGTH ( LR11XX_BL_CMD_NO_PARAM_LENGTH + 1 )

enum
{
    LR11XX_BL_ERASE_PAGE_OC  = 0x8001,
    LR11XX_BL_WRITE_FLASH_OC = 0x8002,
    LR11XX_BL_GET_HASH_OC    = 0x8004,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

uint32_t min( uint32_t a, uint32_t b )
{
    uint32_t min = a;

    if( a > b )
    {
        min = b;
    }

    return min;
}

/*!
 * @brief Helper function to fill cbuffer with opcode and offset
 *
 * Typically used in write flash functions.
 *
 * @warning It is up to the caller to ensure the size of cbuffer is big enough to contain all information!
 */
static void lr11xx_bootloader_fill_cbuffer_opcode_offset_flash( uint8_t* cbuffer, uint16_t opcode, uint32_t offset );

/*!
 * @brief Helper function to fill cdata with data
 *
 * Typically used in write flash functions.
 *
 * @warning It is up to the caller to ensure the size of cdata is big enough to contain all data!
 */
static void lr11xx_bootloader_fill_cdata_flash( uint8_t* cdata, const uint32_t* data, uint8_t data_length );

/*!
 * @brief Helper function to fill cbuffer and cdata with information to write flash
 *
 * Typically used in write flash functions. Internally calls lr11xx_bootloader_fill_cbuffer_opcode_offset_flash and
 * lr11xx_bootloader_fill_cdata_flash.
 *
 * @warning It is up to the caller to ensure the sizes of cbuffer and cdata are big enough to contain their respective
 * information!
 */
static void lr11xx_bootloader_fill_cbuffer_cdata_flash( uint8_t* cbuffer, uint8_t* cdata, uint16_t opcode,
                                                        uint32_t offset, const uint32_t* data, uint8_t data_length );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_bootloader_get_hash( const void* context, lr11xx_bootloader_hash_t hash )
{
    uint8_t cbuffer[LR11XX_BL_GET_HASH_CMD_LENGTH];

    cbuffer[0] = ( uint8_t ) ( LR11XX_BL_GET_HASH_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_BL_GET_HASH_OC >> 0 );

    return ( lr11xx_status_t ) lr11xx_hal_read( context, cbuffer, LR11XX_BL_GET_HASH_CMD_LENGTH, hash,
                                                LR11XX_BL_HASH_LENGTH );
}

lr11xx_status_t lr11xx_bootloader_erase_page( const void* context, const uint8_t page_number )
{
    uint8_t cbuffer[LR11XX_BL_ERASE_PAGE_CMD_LENGTH];

    cbuffer[0] = ( uint8_t ) ( LR11XX_BL_ERASE_PAGE_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_BL_ERASE_PAGE_OC >> 0 );

    cbuffer[2] = page_number;

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_BL_ERASE_PAGE_CMD_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_bootloader_write_flash( const void* context, const uint32_t offset, const uint32_t* data,
                                               uint8_t length )
{
    uint8_t cbuffer[LR11XX_BL_WRITE_FLASH_CMD_LENGTH];
    uint8_t cdata[256];

    lr11xx_bootloader_fill_cbuffer_cdata_flash( cbuffer, cdata, LR11XX_BL_WRITE_FLASH_OC, offset, data, length );

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_BL_WRITE_FLASH_CMD_LENGTH, cdata,
                                                 length * sizeof( uint32_t ) );
}

lr11xx_status_t lr11xx_bootloader_write_flash_full( const void* context, const uint32_t offset, const uint32_t* buffer,
                                                    const uint32_t length )
{
    lr11xx_status_t status           = LR11XX_STATUS_OK;
    uint32_t        remaining_length = length;
    uint32_t        local_offset     = offset;
    uint32_t        loop             = 0;

    while( ( remaining_length != 0 ) && ( status == LR11XX_STATUS_OK ) )
    {
        status = ( lr11xx_status_t ) lr11xx_bootloader_write_flash(
            context, local_offset, buffer + loop * LR11XX_FLASH_DATA_MAX_LENGTH_UINT32,
            min( remaining_length, LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 ) );

        local_offset += LR11XX_FLASH_DATA_MAX_LENGTH_UINT8;
        remaining_length = ( remaining_length < LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 )
                               ? 0
                               : ( remaining_length - LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 );

        loop++;
    }

    return status;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void lr11xx_bootloader_fill_cbuffer_opcode_offset_flash( uint8_t* cbuffer, uint16_t opcode, uint32_t offset )
{
    cbuffer[0] = ( uint8_t ) ( opcode >> 8 );
    cbuffer[1] = ( uint8_t ) ( opcode >> 0 );

    cbuffer[2] = ( uint8_t ) ( offset >> 24 );
    cbuffer[3] = ( uint8_t ) ( offset >> 16 );
    cbuffer[4] = ( uint8_t ) ( offset >> 8 );
    cbuffer[5] = ( uint8_t ) ( offset >> 0 );
}

void lr11xx_bootloader_fill_cdata_flash( uint8_t* cdata, const uint32_t* data, uint8_t data_length )
{
    for( uint8_t index = 0; index < data_length; index++ )
    {
        uint8_t* cdata_local = &cdata[index * sizeof( uint32_t )];

        cdata_local[0] = ( uint8_t ) ( data[index] >> 24 );
        cdata_local[1] = ( uint8_t ) ( data[index] >> 16 );
        cdata_local[2] = ( uint8_t ) ( data[index] >> 8 );
        cdata_local[3] = ( uint8_t ) ( data[index] >> 0 );
    }
}

void lr11xx_bootloader_fill_cbuffer_cdata_flash( uint8_t* cbuffer, uint8_t* cdata, uint16_t opcode, uint32_t offset,
                                                 const uint32_t* data, uint8_t data_length )
{
    lr11xx_bootloader_fill_cbuffer_opcode_offset_flash( cbuffer, opcode, offset );
    lr11xx_bootloader_fill_cdata_flash( cdata, data, data_length );
}

/* --- EOF ------------------------------------------------------------------ */
