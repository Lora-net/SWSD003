/*!
 * @file      lr11xx_gnss_types_str.c
 *
 * @brief     Printer helper functions for LR11xx GNSS types
 *
 * @copyright
 * The Clear BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
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
 
#include "lr11xx_gnss_types_str.h"

const char* lr11xx_gnss_constellation_to_str( const lr11xx_gnss_constellation_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_GPS_MASK:
    {
        return ( const char* ) "LR11XX_GNSS_GPS_MASK";
    }

    case LR11XX_GNSS_BEIDOU_MASK:
    {
        return ( const char* ) "LR11XX_GNSS_BEIDOU_MASK";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_search_mode_to_str( const lr11xx_gnss_search_mode_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_OPTION_LOW_EFFORT:
    {
        return ( const char* ) "LR11XX_GNSS_OPTION_LOW_EFFORT";
    }

    case LR11XX_GNSS_OPTION_MID_EFFORT:
    {
        return ( const char* ) "LR11XX_GNSS_OPTION_MID_EFFORT";
    }

    case LR11XX_GNSS_OPTION_HIGH_EFFORT:
    {
        return ( const char* ) "LR11XX_GNSS_OPTION_HIGH_EFFORT";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_destination_to_str( const lr11xx_gnss_destination_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_DESTINATION_HOST:
    {
        return ( const char* ) "LR11XX_GNSS_DESTINATION_HOST";
    }

    case LR11XX_GNSS_DESTINATION_SOLVER:
    {
        return ( const char* ) "LR11XX_GNSS_DESTINATION_SOLVER";
    }

    case LR11XX_GNSS_DESTINATION_DMC:
    {
        return ( const char* ) "LR11XX_GNSS_DESTINATION_DMC";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_message_host_status_to_str( const lr11xx_gnss_message_host_status_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_HOST_OK:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_OK";
    }

    case LR11XX_GNSS_HOST_UNEXPECTED_CMD:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_UNEXPECTED_CMD";
    }

    case LR11XX_GNSS_HOST_UNIMPLEMENTED_CMD:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_UNIMPLEMENTED_CMD";
    }

    case LR11XX_GNSS_HOST_INVALID_PARAMETERS:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_INVALID_PARAMETERS";
    }

    case LR11XX_GNSS_HOST_MESSAGE_SANITY_CHECK_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_MESSAGE_SANITY_CHECK_ERROR";
    }

    case LR11XX_GNSS_HOST_IQ_CAPTURE_FAILS:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_IQ_CAPTURE_FAILS";
    }

    case LR11XX_GNSS_HOST_NO_TIME:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_NO_TIME";
    }

    case LR11XX_GNSS_HOST_NO_SATELLITE_DETECTED:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_NO_SATELLITE_DETECTED";
    }

    case LR11XX_GNSS_HOST_ALMANAC_IN_FLASH_TOO_OLD:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_IN_FLASH_TOO_OLD";
    }

    case LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_CRC_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_CRC_ERROR";
    }

    case LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_FLASH_INTEGRITY_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_UPDATE_FAILS_FLASH_INTEGRITY_ERROR";
    }

    case LR11XX_GNSS_HOST_ALMANAC_UPDATE_NOT_ALLOWED:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_UPDATE_NOT_ALLOWED";
    }

    case LR11XX_GNSS_HOST_ALMANAC_CRC_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_CRC_ERROR";
    }

    case LR11XX_GNSS_HOST_ALMANAC_VERSION_NOT_SUPPORTED:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_ALMANAC_VERSION_NOT_SUPPORTED";
    }

    case LR11XX_GNSS_HOST_NOT_ENOUGH_SV_DETECTED_TO_BUILD_A_NAV_MESSAGE:
    {
        return ( const char* ) "LR11XX_GNSS_HOST_NOT_ENOUGH_SV_DETECTED_TO_BUILD_A_NAV_MESSAGE";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_message_dmc_opcode_to_str( const lr11xx_gnss_message_dmc_opcode_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_DMC_STATUS:
    {
        return ( const char* ) "LR11XX_GNSS_DMC_STATUS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_scan_mode_to_str( const lr11xx_gnss_scan_mode_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_SCAN_MODE_0_SINGLE_SCAN_LEGACY:
    {
        return ( const char* ) "LR11XX_GNSS_SCAN_MODE_0_SINGLE_SCAN_LEGACY";
    }

    case LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS:
    {
        return ( const char* ) "LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_error_code_to_str( const lr11xx_gnss_error_code_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_NO_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_NO_ERROR";
    }

    case LR11XX_GNSS_ERROR_ALMANAC_TOO_OLD:
    {
        return ( const char* ) "LR11XX_GNSS_ERROR_ALMANAC_TOO_OLD";
    }

    case LR11XX_GNSS_ERROR_UPDATE_CRC_MISMATCH:
    {
        return ( const char* ) "LR11XX_GNSS_ERROR_UPDATE_CRC_MISMATCH";
    }

    case LR11XX_GNSS_ERROR_UPDATE_FLASH_MEMORY_INTEGRITY:
    {
        return ( const char* ) "LR11XX_GNSS_ERROR_UPDATE_FLASH_MEMORY_INTEGRITY";
    }

    case LR11XX_GNSS_ERROR_ALMANAC_UPDATE_NOT_ALLOWED:
    {
        return ( const char* ) "LR11XX_GNSS_ERROR_ALMANAC_UPDATE_NOT_ALLOWED";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_freq_search_space_to_str( const lr11xx_gnss_freq_search_space_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_250_HZ:
    {
        return ( const char* ) "LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_250_HZ";
    }

    case LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_500_HZ:
    {
        return ( const char* ) "LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_500_HZ";
    }

    case LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_1_KHZ:
    {
        return ( const char* ) "LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_1_KHZ";
    }

    case LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_2_KHZ:
    {
        return ( const char* ) "LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_2_KHZ";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_fetch_time_option_to_str( const lr11xx_gnss_fetch_time_option_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_SEARCH_TOW:
    {
        return ( const char* ) "LR11XX_GNSS_SEARCH_TOW";
    }

    case LR11XX_GNSS_SEARCH_TOW_WN:
    {
        return ( const char* ) "LR11XX_GNSS_SEARCH_TOW_WN";
    }

    case LR11XX_GNSS_SEARCH_TOW_WN_ROLLOVER:
    {
        return ( const char* ) "LR11XX_GNSS_SEARCH_TOW_WN_ROLLOVER";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_read_time_status_to_str( const lr11xx_gnss_read_time_status_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_READ_TIME_STATUS_NO_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_READ_TIME_STATUS_NO_ERROR";
    }

    case LR11XX_GNSS_READ_TIME_STATUS_32K_STOPPED:
    {
        return ( const char* ) "LR11XX_GNSS_READ_TIME_STATUS_32K_STOPPED";
    }

    case LR11XX_GNSS_READ_TIME_STATUS_WN_TOW_NOT_SET:
    {
        return ( const char* ) "LR11XX_GNSS_READ_TIME_STATUS_WN_TOW_NOT_SET";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_week_number_rollover_status_to_str( const lr11xx_gnss_week_number_rollover_status_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_WN_ROLLOVER_ROLLOVER_NEVER_SET:
    {
        return ( const char* ) "LR11XX_GNSS_WN_ROLLOVER_ROLLOVER_NEVER_SET";
    }

    case LR11XX_GNSS_WN_ROLLOVER_ROLLOVER_SET_BY_SCAN:
    {
        return ( const char* ) "LR11XX_GNSS_WN_ROLLOVER_ROLLOVER_SET_BY_SCAN";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_demod_status_to_str( const lr11xx_gnss_demod_status_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_NO_DEMOD_BDS_ALMANAC_SV31_43:
    {
        return ( const char* ) "LR11XX_GNSS_NO_DEMOD_BDS_ALMANAC_SV31_43";
    }

    case LR11XX_GNSS_SV_SELECTED_FOR_DEMOD_LOST:
    {
        return ( const char* ) "LR11XX_GNSS_SV_SELECTED_FOR_DEMOD_LOST";
    }

    case LR11XX_GNSS_ALMANAC_DEMOD_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_ALMANAC_DEMOD_ERROR";
    }

    case LR11XX_GNSS_WAKE_UP_AFTER_PREAMBLE:
    {
        return ( const char* ) "LR11XX_GNSS_WAKE_UP_AFTER_PREAMBLE";
    }

    case LR11XX_GNSS_20MS_REAL_TIME_FAILURE:
    {
        return ( const char* ) "LR11XX_GNSS_20MS_REAL_TIME_FAILURE";
    }

    case LR11XX_GNSS_WAKE_UP_SYNC_FAILURE:
    {
        return ( const char* ) "LR11XX_GNSS_WAKE_UP_SYNC_FAILURE";
    }

    case LR11XX_GNSS_WEEK_NUMBER_NOT_VALIDATED:
    {
        return ( const char* ) "LR11XX_GNSS_WEEK_NUMBER_NOT_VALIDATED";
    }

    case LR11XX_GNSS_NO_ACTIVATED_SAT_IN_SV_LIST:
    {
        return ( const char* ) "LR11XX_GNSS_NO_ACTIVATED_SAT_IN_SV_LIST";
    }

    case LR11XX_GNSS_SLEEP_TIME_TOO_LONG:
    {
        return ( const char* ) "LR11XX_GNSS_SLEEP_TIME_TOO_LONG";
    }

    case LR11XX_GNSS_WRONG_TIME_OF_WEEK_DEMOD:
    {
        return ( const char* ) "LR11XX_GNSS_WRONG_TIME_OF_WEEK_DEMOD";
    }

    case LR11XX_GNSS_PREAMBLE_NOT_VALIDATED:
    {
        return ( const char* ) "LR11XX_GNSS_PREAMBLE_NOT_VALIDATED";
    }

    case LR11XX_GNSS_DEMOD_DISABLE:
    {
        return ( const char* ) "LR11XX_GNSS_DEMOD_DISABLE";
    }

    case LR11XX_GNSS_DEMOD_EXTRACTION_FAILURE:
    {
        return ( const char* ) "LR11XX_GNSS_DEMOD_EXTRACTION_FAILURE";
    }

    case LR11XX_GNSS_NO_BIT_CHANGE_FOUND_DURING_START_DEMOD:
    {
        return ( const char* ) "LR11XX_GNSS_NO_BIT_CHANGE_FOUND_DURING_START_DEMOD";
    }

    case LR11XX_GNSS_NO_BIT_CHANGE_FOUND_DURING_MULTISCAN:
    {
        return ( const char* ) "LR11XX_GNSS_NO_BIT_CHANGE_FOUND_DURING_MULTISCAN";
    }

    case LR11XX_GNSS_NO_SAT_FOUND:
    {
        return ( const char* ) "LR11XX_GNSS_NO_SAT_FOUND";
    }

    case LR11XX_GNSS_WORD_SYNC_LOST:
    {
        return ( const char* ) "LR11XX_GNSS_WORD_SYNC_LOST";
    }

    case LR11XX_GNSS_NOT_ENOUGH_PARITY_CHECK_FOUND:
    {
        return ( const char* ) "LR11XX_GNSS_NOT_ENOUGH_PARITY_CHECK_FOUND";
    }

    case LR11XX_GNSS_TOO_MANY_PARITY_CHECK_FOUND:
    {
        return ( const char* ) "LR11XX_GNSS_TOO_MANY_PARITY_CHECK_FOUND";
    }

    case LR11XX_GNSS_NO_PARITY_CHECK_FOUND:
    {
        return ( const char* ) "LR11XX_GNSS_NO_PARITY_CHECK_FOUND";
    }

    case LR11XX_GNSS_WORD_SYNC_SEARCH_NOT_STARTED:
    {
        return ( const char* ) "LR11XX_GNSS_WORD_SYNC_SEARCH_NOT_STARTED";
    }

    case LR11XX_GNSS_WORD_SYNC_POTENTIALLY_FOUND:
    {
        return ( const char* ) "LR11XX_GNSS_WORD_SYNC_POTENTIALLY_FOUND";
    }

    case LR11XX_GNSS_WORD_SYNC_FOUND:
    {
        return ( const char* ) "LR11XX_GNSS_WORD_SYNC_FOUND";
    }

    case LR11XX_GNSS_TIME_OF_WEEK_FOUND:
    {
        return ( const char* ) "LR11XX_GNSS_TIME_OF_WEEK_FOUND";
    }

    case LR11XX_GNSS_WEEK_NUMBER_FOUND:
    {
        return ( const char* ) "LR11XX_GNSS_WEEK_NUMBER_FOUND";
    }

    case LR11XX_GNSS_ALMANAC_FOUND_BUT_NO_SAVED:
    {
        return ( const char* ) "LR11XX_GNSS_ALMANAC_FOUND_BUT_NO_SAVED";
    }

    case LR11XX_GNSS_HALF_ALMANAC_FOUND_AND_SAVED:
    {
        return ( const char* ) "LR11XX_GNSS_HALF_ALMANAC_FOUND_AND_SAVED";
    }

    case LR11XX_GNSS_ALMANAC_FOUND_AND_SAVED:
    {
        return ( const char* ) "LR11XX_GNSS_ALMANAC_FOUND_AND_SAVED";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_doppler_solver_error_code_to_str( const lr11xx_gnss_doppler_solver_error_code_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_DOPPLER_SOLVER_NO_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_DOPPLER_SOLVER_NO_ERROR";
    }

    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_RESIDUE_HIGH:
    {
        return ( const char* ) "LR11XX_GNSS_DOPPLER_SOLVER_ERROR_RESIDUE_HIGH";
    }

    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_CONVERGED:
    {
        return ( const char* ) "LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_CONVERGED";
    }

    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_ENOUGH_SV:
    {
        return ( const char* ) "LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_ENOUGH_SV";
    }

    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_ILL_MATRIX:
    {
        return ( const char* ) "LR11XX_GNSS_DOPPLER_SOLVER_ERROR_ILL_MATRIX";
    }

    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_TIME_ERROR:
    {
        return ( const char* ) "LR11XX_GNSS_DOPPLER_SOLVER_ERROR_TIME_ERROR";
    }

    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_PARTIAL_ALMANAC_TOO_OLD:
    {
        return ( const char* ) "LR11XX_GNSS_DOPPLER_SOLVER_ERROR_PARTIAL_ALMANAC_TOO_OLD";
    }

    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_CONSISTENT_WITH_HISTORY:
    {
        return ( const char* ) "LR11XX_GNSS_DOPPLER_SOLVER_ERROR_NOT_CONSISTENT_WITH_HISTORY";
    }

    case LR11XX_GNSS_DOPPLER_SOLVER_ERROR_ALL_ALMANAC_TOO_OLD:
    {
        return ( const char* ) "LR11XX_GNSS_DOPPLER_SOLVER_ERROR_ALL_ALMANAC_TOO_OLD";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_almanac_status_to_str( const lr11xx_gnss_almanac_status_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_INTERNAL_ACCURACY_TOO_LOW:
    {
        return ( const char* ) "LR11XX_GNSS_INTERNAL_ACCURACY_TOO_LOW";
    }

    case LR11XX_GNSS_NO_TIME_SET:
    {
        return ( const char* ) "LR11XX_GNSS_NO_TIME_SET";
    }

    case LR11XX_GNSS_IMPOSSIBLE_TO_FIND_NEXT_TIME:
    {
        return ( const char* ) "LR11XX_GNSS_IMPOSSIBLE_TO_FIND_NEXT_TIME";
    }

    case LR11XX_GNSS_NO_PAGE_ID_KNOWN:
    {
        return ( const char* ) "LR11XX_GNSS_NO_PAGE_ID_KNOWN";
    }

    case LR11XX_GNSS_NO_SAT_TO_UPDATE:
    {
        return ( const char* ) "LR11XX_GNSS_NO_SAT_TO_UPDATE";
    }

    case LR11XX_GNSS_AT_LEAST_ONE_SAT_MUST_BE_UPDATED:
    {
        return ( const char* ) "LR11XX_GNSS_AT_LEAST_ONE_SAT_MUST_BE_UPDATED";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_sv_type_to_str( const lr11xx_gnss_sv_type_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_MEO_SAT:
    {
        return ( const char* ) "LR11XX_GNSS_MEO_SAT";
    }

    case LR11XX_GNSS_IGSO_SAT:
    {
        return ( const char* ) "LR11XX_GNSS_IGSO_SAT";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}

const char* lr11xx_gnss_scan_mode_launched_to_str( const lr11xx_gnss_scan_mode_launched_t value )
{
    switch( value )
    {
    case LR11XX_GNSS_LAST_SCAN_MODE_ASSISTED:
    {
        return ( const char* ) "LR11XX_GNSS_LAST_SCAN_MODE_ASSISTED";
    }

    case LR11XX_GNSS_LAST_SCAN_MODE_AUTONOMOUS_NO_TIME_NO_AP:
    {
        return ( const char* ) "LR11XX_GNSS_LAST_SCAN_MODE_AUTONOMOUS_NO_TIME_NO_AP";
    }

    case LR11XX_GNSS_LAST_SCAN_MODE_AUTONOMOUS_NO_AP:
    {
        return ( const char* ) "LR11XX_GNSS_LAST_SCAN_MODE_AUTONOMOUS_NO_AP";
    }

    case LR11XX_GNSS_LAST_SCAN_FETCH_TIME_OR_DOPPLER_SOLVER:
    {
        return ( const char* ) "LR11XX_GNSS_LAST_SCAN_FETCH_TIME_OR_DOPPLER_SOLVER";
    }

    case LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE:
    {
        return ( const char* ) "LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE";
    }

    case LR11XX_GNSS_LAST_SCAN_KEEP_SYNC:
    {
        return ( const char* ) "LR11XX_GNSS_LAST_SCAN_KEEP_SYNC";
    }

    case LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE_1_CONSTELLATION:
    {
        return ( const char* ) "LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE_1_CONSTELLATION";
    }

    case LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE_2_CONSTELLATIONS:
    {
        return ( const char* ) "LR11XX_GNSS_LAST_SCAN_ALMANAC_UPDATE_2_CONSTELLATIONS";
    }

    default:
    {
        return ( const char* ) "Unknown";
    }
    }
}
