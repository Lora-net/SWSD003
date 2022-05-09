/*!
 * @file      smtc_hal_tmr_list.h
 *
 * @brief     Board specific package HW timer API definition.
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

#ifndef SMTC_HAL_TMR_LIST_H
#define SMTC_HAL_TMR_LIST_H

#ifdef __cplusplus
extern "C" {
#endif
/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stddef.h>
#include <stdint.h>   
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Timer object description
 */
typedef struct timer_event_s
{
    uint32_t timestamp;                   //! Current timer value
    uint32_t reload_value;                //! Timer delay value
    bool     is_started;                  //! Is the timer currently running
    bool     is_next_2_expire;            //! Is the next timer to expire
    void ( *callback )( void* context );  //! Timer IRQ callback function
    void*                 context;        //! User defined data object pointer to pass back
    struct timer_event_s* next;           //! Pointer to the next Timer object.
} timer_event_t;

/*!
 * @brief Timer time variable definition
 */
#ifndef timer_time_t
typedef uint32_t timer_time_t;
#define TIMERTIME_T_MAX ( ( uint32_t ) ~0 )
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initializes the timer object
 *
 * @remark TimerSetValue function must be called before starting the timer.
 *         this function initializes timestamp and reload value at 0.
 *
 * @param [in] obj Structure containing the timer object parameters
 * @param [in] callback Function callback called at the end of the timeout
 */
void timer_init( timer_event_t* obj, void ( *callback )( void* context ) );

/*!
 * @brief Sets a user defined object pointer
 *
 * @param [in] obj Structure containing the timer object parameters
 * @param [in] context User defined data object pointer to pass back
 *                     on IRQ handler callback
 */
void timer_set_context( timer_event_t* obj, void* context );

/*!
 * @brief Timer IRQ event handler
 */
void timer_irq_handler( void );

/*!
 * @brief Check if a timer is running
 *
 * @returns status  returns if a timer is running [true: yes,
 *                                                false: no]
 */
bool is_timer_running( void );

/*!
 * @brief Starts and adds the timer object to the list of timer events
 *
 * @param [in] obj Structure containing the timer object parameters
 */
void timer_start( timer_event_t* obj );

/*!
 * @brief Checks if the provided timer is running
 *
 * @param [in] obj Structure containing the timer object parameters
 *
 * @returns status  returns the timer activity status [true: Started,
 *                                                    false: Stopped]
 */
bool timer_is_started( timer_event_t* obj );

/*!
 * @brief Stops and removes the timer object from the list of timer events
 *
 * @param [in] obj Structure containing the timer object parameters
 */
void timer_stop( timer_event_t* obj );

/*!
 * @brief Resets the timer object
 *
 * @param [in] obj Structure containing the timer object parameters
 */
void timer_reset( timer_event_t* obj );

/*!
 * @brief Set timer new timeout value
 *
 * @param [in] obj   Structure containing the timer object parameters
 * @param [in] value New timer timeout value
 */
void timer_set_value( timer_event_t* obj, uint32_t value );

/*!
 * @brief Return the Time elapsed since a fix moment in Time
 *
 * @remark TimerGetElapsedTime will return 0 for argument 0.
 *
 * @param [in] past         fix moment in Time
 * @returns time             returns elapsed time
 */
timer_time_t timer_get_elapsed_time( timer_time_t past );

/*!
 * @brief Computes the temperature compensation for a period of time on a
 *        specific temperature.
 *
 * @param [in] period Time period to compensate
 * @param [in] temperature Current temperature
 *
 * @returns Compensated time period
 */
timer_time_t timer_temp_compensation( timer_time_t period, float temperature );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_TMR_LIST_H

/* --- EOF ------------------------------------------------------------------ */
