#ifndef __LR11XX_GNSS_ALPHA_H__
#define __LR11XX_GNSS_ALPHA_H__

#include <stdint.h>
#include "lr11xx_types.h"
#include "lr11xx_gnss_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Get the size of results for first scan
 *
 * This method returns the size in bytes of the results available in LR11XX result buffer.
 * It is a debug command for NAV 2 test. It is to be used for gnss scan modes
 * LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS.
 *
 * @param [in] context Chip implementation context
 * @param [out] result_size Result size
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_first_scan_result_size( const void* context, uint16_t* result_size );

/*!
 * @brief Read first scan GNSS results
 *
 * The GNSS results are pushed into a buffer directly. This buffer is provided by the application using the driver. It
 * MUST be long enough to contains at least result_buffer_size bytes.
 * It is a debug command for NAV 2 test. It is to be used for gnss scan mode
 * LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS.
 *
 * @warning No check is done on result_buffer size. If this application provided buffer is too small, there will be a
 * buffer overflow bug!
 *
 * @param [in] context Chip implementation context
 * @param [out] result_buffer Application provided buffer to be filled with result
 * @param [in] result_buffer_size The number of bytes to read from the LR11XX
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_first_scan_results( const void* context, uint8_t* result_buffer,
                                                     const uint16_t result_buffer_size );

/*!
 * @brief Get the size of results for intermediate scan
 *
 * This method returns the size in bytes of the results available in LR11XX result buffer.
 * It is a debug command for NAV 2 test. It is to be used for gnss scan modes
 * LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS, and LR11XX_GNSS_SCAN_MODE_4_SINGLE_SCAN_AND_17_FAST_SCANS.
 *
 * @param [in] context Chip implementation context
 * @param [out] result_size Result size
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_intermediate_scan_result_size( const void* context, uint16_t* result_size );

/*!
 * @brief Read first scan GNSS results
 *
 * The GNSS results are pushed into a buffer directly. This buffer is provided by the application using the driver. It
 * MUST be long enough to contains at least result_buffer_size bytes.
 * It is a debug command for NAV 2 test. It is to be used for gnss scan modes
 * LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS, and LR11XX_GNSS_SCAN_MODE_4_SINGLE_SCAN_AND_17_FAST_SCANS.
 *
 * @warning No check is done on result_buffer size. If this application provided buffer is too small, there will be a
 * buffer overflow bug!
 *
 * @param [in] context Chip implementation context
 * @param [out] result_buffer Application provided buffer to be filled with result
 * @param [in] result_buffer_size The number of bytes to read from the LR11XX
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_intermediate_scan_results( const void* context, uint8_t* result_buffer,
                                                            const uint16_t result_buffer_size );

/*!
 * @brief Get almanac CRC
 *
 * @param [in] context Chip implementation context
 * @param [out] almanac_crc Almanac CRC
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_get_almanac_crc( const void* context, uint32_t* almanac_crc );

/*!
 * @brief Function to set the crystal error.
 *
 * @param [in] context Chip implementation context
 * @param [in] xtal_error_in_ppm value in +/-40ppm
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_set_xtal_error( const void* context, const float xtal_error_in_ppm );

/*!
 * @brief Function to read the crystal error.
 *
 * @param [in] context Chip implementation context
 * @param [out] xtal_error_in_ppm value returned between +/-30ppm
 *
 * @returns Operation status
 */
lr11xx_status_t lr11xx_gnss_read_xtal_error( const void* context, float* xtal_error_in_ppm );

#ifdef __cplusplus
}
#endif

#endif  // __LR11XX_GNSS_ALPHA_H__
