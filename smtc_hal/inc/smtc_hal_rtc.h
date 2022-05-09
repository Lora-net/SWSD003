/*!
 * @file      smtc_hal_rtc.h
 *
 * @brief     Board specific package RTC API definition.
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
#ifndef SMTC_HAL_RTC_H
#define SMTC_HAL_RTC_H

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief Temperature coefficient of the clock source
 */
#define RTC_TEMP_COEFFICIENT ( -0.035 )

/*!
 * @brief Temperature coefficient deviation of the clock source
 */
#define RTC_TEMP_DEV_COEFFICIENT ( 0.0035 )

/*!
 * @brief Turnover temperature of the clock source
 */
#define RTC_TEMP_TURNOVER ( 25.0 )

/*!
 * @brief Turnover temperature deviation of the clock source
 */
#define RTC_TEMP_DEV_TURNOVER ( 5.0 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 *  @brief Initializes the MCU RTC peripheral
 */
void hal_rtc_init( void );

/*!
 * @brief Returns the current RTC time in seconds
 *
 * @remark Used for scheduling autonomous retransmissions (i.e: NbTrans),
 *         transmitting MAC answers, basically any delay without accurate time
 *         constraints. It is also used to measure the time spent inside the
 *         LoRaWAN process for the integrated failsafe.
 *
 * @returns rtc_time_s Current RTC time in seconds
 */
uint32_t hal_rtc_get_time_s( void );

/*!
 * @brief Returns the current RTC time in milliseconds
 *
 * @remark Used to timestamp radio events (i.e: end of TX), will also be used
 * for ClassB
 *
 * @returns rtc_time_ms Current RTC time in milliseconds wraps every 49 days
 */
uint32_t hal_rtc_get_time_ms( void );

/*!
 * @brief Waits delay milliseconds by polling RTC
 *
 * @param [in] milliseconds Delay in ms
 */
void hal_rtc_delay_in_ms( const uint32_t milliseconds );

/*!
 * @brief Sets the rtc wakeup timer for seconds parameter. The RTC will generate an IRQ
 * to wakeup the MCU.
 *
 * @param [in] seconds Number of seconds before wakeup
 */
void hal_rtc_wakeup_timer_set_s( const int32_t seconds );

/*!
 * @brief Sets the rtc wakeup timer for milliseconds parameter. The RTC will generate
 * an IRQ to wakeup the MCU.
 *
 * @param [in] milliseconds Number of seconds before wakeup
 */
void hal_rtc_wakeup_timer_set_ms( const int32_t milliseconds );

/*!
 * @brief Stops the Timer
 */
void hal_rtc_wakeup_timer_stop( void );

/*!
 * @brief Set the RTC time reference in ticks
 *
 * @returns time_ref_in_ticks RTC time reference in ticks
 */
uint32_t hal_rtc_set_time_ref_in_ticks( void );

/*!
 * @brief Get the RTC time reference in ticks
 *
 * @returns time_ref_in_ticks RTC time reference in ticks
 */
uint32_t hal_rtc_get_time_ref_in_ticks( void );

/*!
 * @brief Get the RTC timer elapsed time since the last Alarm was set
 *
 * @returns RTC Elapsed time since the last alarm in ticks.
 */
uint32_t hal_rtc_get_timer_elapsed_value( void );

/*!
 * @brief Get the RTC timer value
 *
 * @returns RTC Timer value
 */
uint32_t hal_rtc_get_timer_value( void );

/*!
 * @brief Converts time in ms to time in ticks
 *
 * @param [in] milliseconds Time in milliseconds
 * @returns milliseconds Time in timer ticks
 */
uint32_t hal_rtc_ms_2_tick( const uint32_t milliseconds );

/*!
 * @brief Converts time in ticks to time in ms
 *
 * @param [in] tick Time in timer ticks
 * @returns tick Time in milliseconds
 */
uint32_t hal_rtc_tick_2_ms( const uint32_t tick );

/*!
 * @brief returns the wake up time in ticks
 *
 * @returns wake up time in ticks
 */
uint32_t hal_rtc_get_minimum_timeout( void );

/*!
 * @brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * @param [in] period Time period to compensate in milliseconds
 * @param [in] temperature Current temperature
 *
 * @returns Compensated time period
 */
uint32_t hal_rtc_temp_compensation( uint32_t period, float temperature );

/*!
 * @brief Stops the Alarm
 */
void hal_rtc_stop_alarm( void );

/*!
 * @brief Starts wake up alarm
 *
 * @remark  Alarm in RtcTimerContext.Time + timeout
 *
 * @param [in] timeout Timeout value in ticks
 */
void hal_rtc_start_alarm( uint32_t timeout );

/*!
 * Return true if the wake up timer irq has been triggered
 *
 * \retval true if wut irq happended
 */
bool hal_rtc_has_wut_irq_happened( void );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_RTC_H

/* --- EOF ------------------------------------------------------------------ */
