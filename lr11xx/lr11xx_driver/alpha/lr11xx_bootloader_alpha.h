#ifndef LR11XX_BOOTLOADER_ALPHA_H
#define LR11XX_BOOTLOADER_ALPHA_H

#include <stdint.h>
#include "lr11xx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Length in bytes of a hash value
 */
#define LR11XX_BL_HASH_LENGTH 0x10

typedef uint8_t lr11xx_bootloader_hash_t[LR11XX_BL_HASH_LENGTH];

/*!
 * @brief Get calculated hash of flash content.
 *
 * This method should be used to get the hash of flash content.
 *
 * @param [in] context Chip implementation context
 *
 * @param [out] hash Pointer to the hash array to be populated with hash value
 */
lr11xx_status_t lr11xx_bootloader_get_hash( const void* context, lr11xx_bootloader_hash_t hash );

/*!
 * @brief Erase the specified page in the flash memory
 *
 * @param [in] context Chip implementation context
 *
 * @param [in] page_number The index of the page to erase
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_erase_page( const void* context, const uint8_t page_number );

/*!
 * @brief Write data in program flash memory of the chip
 *
 * This function shall be used when updating the flash content of the LR11XX.
 * The flash payload to transfer shall be represented as an array of words (ie 4 bytes values).
 *
 * @param [in] context Chip implementation context
 *
 * @param [in] offset The offset from start register of flash
 *
 * @param [in] buffer A pointer to the buffer holding the content of flash to transfert. Its size in words must be at
 * least length
 *
 * @param [in] length Number of words (i.e. 4 bytes) in the buffer to transfer
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_write_flash( const void* context, const uint32_t offset, const uint32_t* buffer,
                                               const uint8_t length );

/*!
 * @brief Write data in program flash memory of the chip
 *
 * This function shall be used when updating the flash content of the LR11XX.
 * The flash payload to transfer shall be represented as an array of words (i.e. 4 bytes values).
 *
 * @param [in] context Chip implementation context
 *
 * @param [in] offset The offset from start register of flash
 *
 * @param [in] buffer A pointer to the buffer holding the content of flash to transfert. Its size in words must be at
 * least length
 *
 * @param [in] length Number of words (i.e. 4 bytes) in the buffer to transfer
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_bootloader_write_flash_full( const void* context, const uint32_t offset, const uint32_t* buffer,
                                                    const uint32_t length );

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_BOOTLOADER_ALPHA_H
