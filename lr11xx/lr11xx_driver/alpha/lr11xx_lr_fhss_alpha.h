#ifndef __LR11XX_LR_FHSS_ALPHA_H__
#define __LR11XX_LR_FHSS_ALPHA_H__

#include "lr11xx_lr_fhss_types.h"
#include "lr11xx_types.h"

/*!
 * @brief Number of hopping configuration read with lr11xx_radio_lr_fhss_read_table
 */
#define LR11XX_LR_FHSS_MAX_READ_TABLE ( 40 )

/*!
 * @brief LR_FHSS mode configuration
 */
typedef enum
{
    LR11XX_LR_FHSS_MODE_DISABLE         = 0x00,  //!< LR_FHSS is disabled
    LR11XX_LR_FHSS_MODE_INTRA_PKT       = 0x01,  //!< LR_FHSS enabled for intra packet only
    LR11XX_LR_FHSS_MODE_INTER_PKT       = 0x02,  //!< LR_FHSS enabled for inter packet only
    LR11XX_LR_FHSS_MODE_INTRA_INTER_PKT = 0x03,  //!< LR_FHSS enabled for intra and inter packet
} lr11xx_lr_fhss_mode_t;

/*!
 * @brief Configuration for one LR_FHSS frequency
 */
typedef struct
{
    uint32_t frequency;   //!< Frequency [Hz]
    uint16_t nb_symbols;  //!< Number of symbols to send on the frequency
} lr11xx_lr_fhss_frequency_cfg;

/*!
 * @brief Configure the LR_FHSS hopping pattern
 *
 * LR_FHSS is only available for TX operations.
 *
 * @param [in] context Chip implementation context
 * @param [in] lr_fhss_mode Indicate the LR_FHSS mode to use
 * @param [in] packet_length Number of bytes to send
 * @param [in] number_of_hops The number of frequency hopping to execute. Value must be in the range [1, 255]
 * @param [in] symbols_per_freq Pointer to a list of frequencies to iterate and the corresponding number of symbols to
 * send. Warning: it is the responsibility of the caller to ensure that the location pointed to by symbols_per_freq
 * contains at least nb_symbols_per_freq elements
 * @param [in] nb_symbols_per_freq Number of elements to write from symbols_per_freq array. Maximal value is 16
 */
lr11xx_status_t lr11xx_lr_fhss_cfg( const void* context, lr11xx_lr_fhss_mode_t lr_fhss_mode, uint8_t packet_length,
                                    uint8_t number_of_hops, const lr11xx_lr_fhss_frequency_cfg* symbols_per_freq,
                                    uint8_t nb_symbols_per_freq );

/*!
 * @brief Update the LR_FHSS hopping table
 *
 * @param [in] context Chip implementation context
 * @param [in] block_id Index of the block to update
 * @param [in] symbol_frequency The configuration to set for hopping block
 */
lr11xx_status_t lr11xx_lr_fhss_update_block_cfg( const void* context, uint8_t block_id,
                                                 const lr11xx_lr_fhss_frequency_cfg* symbol_frequency );

/*!
 * @brief Read the configuration of LR_FHSS.
 *
 * @param [in] radio Radio abstraction
 *
 * @param [out] lr_fhss_mode LR_FHSS mode configured
 *
 * @param [out] packet_length Configured number of bytes to send
 *
 * @param [out] number_of_hops Number of frequency hopping configured
 *
 * @param [out] symbols_per_freq Array to be populated by the call. Returns the configured hopping table. It is up to
 * the caller to ensure the array is long enough to contain at least LR11XX_LR_FHSS_MAX_READ_TABLE elements
 */
lr11xx_status_t lr11xx_lr_fhss_read_table( const void* radio, lr11xx_lr_fhss_mode_t* lr_fhss_mode,
                                           uint8_t* packet_length, uint8_t* number_of_hops,
                                           lr11xx_lr_fhss_frequency_cfg* symbols_per_freq );

#endif  // __LR11XX_LR_FHSS_ALPHA_H__