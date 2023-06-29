#include "lr11xx_lr_fhss_alpha.h"

#define LR11XX_LR_FHSS_CFG_LR_FHSS_MAX_LENGTH ( 2 + 3 + 6 * 16 )  // 107
#define LR11XX_LR_FHSS_UPDATE_CFG_LENGTH ( 2 + 7 )
#define LR11XX_LR_FHSS_READ_TABLE_CMD_LENGTH ( 2 )
#define LR11XX_LR_FHSS_READ_TABLE_RBUFFER_LENGTH ( 3 + ( 6 * LR11XX_LR_FHSS_MAX_READ_TABLE ) )

enum
{
    LR11XX_LR_FHSS_CFG_LR_FHSS_OC = 0x0221,
    LR11XX_LR_FHSS_UPDATE_CFG_OC  = 0x022A,
    LR11XX_LR_FHSS_READ_TABLE_OC  = 0x022F,
};

lr11xx_status_t lr11xx_lr_fhss_cfg( const void* context, lr11xx_lr_fhss_mode_t lr_fhss_mode, uint8_t packet_length,
                                    uint8_t number_of_hops, const lr11xx_lr_fhss_frequency_cfg* symbols_per_freq,
                                    uint8_t nb_symbols_per_freq )
{
    uint8_t cbuffer[LR11XX_LR_FHSS_CFG_LR_FHSS_MAX_LENGTH] = { 0 };

    cbuffer[0] = ( uint8_t ) ( LR11XX_LR_FHSS_CFG_LR_FHSS_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_LR_FHSS_CFG_LR_FHSS_OC >> 0 );

    cbuffer[2] = ( uint8_t ) lr_fhss_mode;
    cbuffer[3] = packet_length;
    cbuffer[4] = number_of_hops;

    for( uint8_t hop_index = 0; hop_index < nb_symbols_per_freq; hop_index++ )
    {
        const uint8_t                       index_in_cbuffer = 5 + hop_index * 6;
        const lr11xx_lr_fhss_frequency_cfg* symbol_freq      = &symbols_per_freq[hop_index];

        cbuffer[index_in_cbuffer + 0] = ( uint8_t ) ( symbol_freq->nb_symbols >> 8 );
        cbuffer[index_in_cbuffer + 1] = ( uint8_t ) symbol_freq->nb_symbols;
        cbuffer[index_in_cbuffer + 2] = ( uint8_t ) ( symbol_freq->frequency >> 24 );
        cbuffer[index_in_cbuffer + 3] = ( uint8_t ) ( symbol_freq->frequency >> 16 );
        cbuffer[index_in_cbuffer + 4] = ( uint8_t ) ( symbol_freq->frequency >> 8 );
        cbuffer[index_in_cbuffer + 5] = ( uint8_t ) symbol_freq->frequency;
    }

    const uint16_t cbuffer_length = 6 * nb_symbols_per_freq + 5;
    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, cbuffer_length, 0, 0 );
}

lr11xx_status_t lr11xx_lr_fhss_update_block_cfg( const void* context, uint8_t block_id,
                                                 const lr11xx_lr_fhss_frequency_cfg* symbol_frequency )
{
    uint8_t cbuffer[LR11XX_LR_FHSS_UPDATE_CFG_LENGTH] = { 0 };

    cbuffer[0] = ( uint8_t ) ( LR11XX_LR_FHSS_UPDATE_CFG_OC >> 8 );
    cbuffer[1] = ( uint8_t ) ( LR11XX_LR_FHSS_UPDATE_CFG_OC >> 0 );

    cbuffer[2] = block_id;

    cbuffer[3] = ( uint8_t ) ( symbol_frequency->nb_symbols >> 8 );
    cbuffer[4] = ( uint8_t ) symbol_frequency->nb_symbols;
    cbuffer[5] = ( uint8_t ) ( symbol_frequency->frequency >> 24 );
    cbuffer[6] = ( uint8_t ) ( symbol_frequency->frequency >> 16 );
    cbuffer[7] = ( uint8_t ) ( symbol_frequency->frequency >> 8 );
    cbuffer[8] = ( uint8_t ) symbol_frequency->frequency;

    return ( lr11xx_status_t ) lr11xx_hal_write( context, cbuffer, LR11XX_LR_FHSS_UPDATE_CFG_LENGTH, 0, 0 );
}

lr11xx_status_t lr11xx_lr_fhss_read_table( const void* radio, lr11xx_lr_fhss_mode_t* lr_fhss_mode,
                                           uint8_t* packet_length, uint8_t* number_of_hops,
                                           lr11xx_lr_fhss_frequency_cfg* symbols_per_freq )
{
    const uint8_t cbuffer[LR11XX_LR_FHSS_READ_TABLE_CMD_LENGTH] = { ( uint8_t ) ( LR11XX_LR_FHSS_READ_TABLE_OC >> 8 ),
                                                                    ( uint8_t ) ( LR11XX_LR_FHSS_READ_TABLE_OC >> 0 ) };
    uint8_t       rbuffer[LR11XX_LR_FHSS_READ_TABLE_RBUFFER_LENGTH] = { 0x00 };

    const lr11xx_status_t status = lr11xx_hal_read( radio, cbuffer, LR11XX_LR_FHSS_READ_TABLE_CMD_LENGTH, rbuffer,
                                                    LR11XX_LR_FHSS_READ_TABLE_RBUFFER_LENGTH );

    if( status == LR11XX_STATUS_OK )
    {
        *lr_fhss_mode   = ( lr11xx_lr_fhss_mode_t ) rbuffer[0];
        *packet_length  = rbuffer[1];
        *number_of_hops = rbuffer[2];
        for( uint8_t index = 0; index < LR11XX_LR_FHSS_MAX_READ_TABLE; index++ )
        {
            const uint8_t  index_in_rbuffer = 3 + index * 6;
            const uint16_t nb_symbols =
                ( ( uint16_t ) rbuffer[index_in_rbuffer] << 8 ) + ( ( uint16_t ) rbuffer[index_in_rbuffer + 1] );
            const uint32_t frequency = ( ( uint32_t ) rbuffer[index_in_rbuffer + 2] << 24 ) +
                                       ( ( uint32_t ) rbuffer[index_in_rbuffer + 3] << 16 ) +
                                       ( ( uint32_t ) rbuffer[index_in_rbuffer + 4] << 8 ) +
                                       ( ( uint32_t ) rbuffer[index_in_rbuffer + 5] << 0 );
            symbols_per_freq[index].nb_symbols = nb_symbols;
            symbols_per_freq[index].frequency  = frequency;
        }
    }
    return status;
}