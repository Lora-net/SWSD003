/*!
 * @file      smtc_hal_tmr_list.c
 *
 * @brief     Timer list API implementation.
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "stm32l4xx_hal.h"
#include "smtc_hal_mcu.h"
#include "smtc_hal_tmr_list.h"
#include "smtc_hal_rtc.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * Safely execute call back
 */
#define execute_callback( _callback_, context ) \
    do                                          \
    {                                           \
        if( _callback_ == NULL )                \
        {                                       \
            while( 1 )                          \
                ;                               \
        }                                       \
        else                                    \
        {                                       \
            _callback_( context );              \
        }                                       \
    } while( 0 );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Timers list head pointer
 */
static timer_event_t* timer_list_head = NULL;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Adds or replace the head timer of the list.
 *
 * @remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * @param [in]  obj Timer object to be become the new head
 * @param [in]  remainingTime Remaining time of the previous head to be replaced
 */
static void timer_insert_new_head_timer( timer_event_t* obj );

/*!
 * @brief Adds a timer to the list.
 *
 * @remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * @param [in]  obj Timer object to be added to the list
 * @param [in]  remainingTime Remaining time of the running head after which the object may be added
 */
static void timer_insert_timer( timer_event_t* obj );

/*!
 * @brief Sets a timeout with the duration "timestamp"
 *
 * @param [in] timestamp Delay duration
 */
static void timer_set_timeout( timer_event_t* obj );

/*!
 * @brief Check if the Object to be added is not already in the list
 *
 * @param [in] timestamp Delay duration
 * @returns true (the object is already in the list) or false
 */
static bool timer_exists( timer_event_t* obj );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void timer_init( timer_event_t* obj, void ( *callback )( void* context ) )
{
    obj->timestamp        = 0;
    obj->reload_value     = 0;
    obj->is_started       = false;
    obj->is_next_2_expire = false;
    obj->callback         = callback;
    obj->context          = NULL;
    obj->next             = NULL;
}

void timer_set_context( timer_event_t* obj, void* context ) { obj->context = context; }

void timer_start( timer_event_t* obj )
{
    uint32_t elapsed_time = 0;

    CRITICAL_SECTION_BEGIN( );

    if( ( obj == NULL ) || ( timer_exists( obj ) == true ) )
    {
        CRITICAL_SECTION_END( );
        return;
    }

    obj->timestamp        = obj->reload_value;
    obj->is_started       = true;
    obj->is_next_2_expire = false;

    if( timer_list_head == NULL )
    {
        hal_rtc_set_time_ref_in_ticks( );
        /* Inserts a timer at time now + obj->timestamp */
        timer_insert_new_head_timer( obj );
    }
    else
    {
        elapsed_time = hal_rtc_get_timer_elapsed_value( );
        obj->timestamp += elapsed_time;

        if( obj->timestamp < timer_list_head->timestamp )
        {
            timer_insert_new_head_timer( obj );
        }
        else
        {
            timer_insert_timer( obj );
        }
    }
    CRITICAL_SECTION_END( );
}

bool is_timer_running( void )
{
    if( timer_list_head == NULL )
    {
        return false;
    }
    else
    {
        return true;
    }
}

static void timer_insert_timer( timer_event_t* obj )
{
    timer_event_t* cur  = timer_list_head;
    timer_event_t* next = timer_list_head->next;

    while( cur->next != NULL )
    {
        if( obj->timestamp > next->timestamp )
        {
            cur  = next;
            next = next->next;
        }
        else
        {
            cur->next = obj;
            obj->next = next;
            return;
        }
    }
    cur->next = obj;
    obj->next = NULL;
}

static void timer_insert_new_head_timer( timer_event_t* obj )
{
    timer_event_t* cur = timer_list_head;

    if( cur != NULL )
    {
        cur->is_next_2_expire = false;
    }

    obj->next       = cur;
    timer_list_head = obj;
    timer_set_timeout( timer_list_head );
}

bool timer_is_started( timer_event_t* obj ) { return obj->is_started; }

void timer_irq_handler( void )
{
    timer_event_t* cur;
    timer_event_t* next;

    uint32_t old           = hal_rtc_get_time_ref_in_ticks( );
    uint32_t now           = hal_rtc_set_time_ref_in_ticks( );
    uint32_t delta_context = now - old;  // intentional wrap around

    /* Update timeStamp based upon new Time Reference
       because delta context should never exceed 2^32 */
    if( timer_list_head != NULL )
    {
        for( cur = timer_list_head; cur->next != NULL; cur = cur->next )
        {
            next = cur->next;
            if( next->timestamp > delta_context )
            {
                next->timestamp -= delta_context;
            }
            else
            {
                next->timestamp = 0;
            }
        }
    }

    /* Execute immediately the alarm callback */
    if( timer_list_head != NULL )
    {
        cur             = timer_list_head;
        timer_list_head = timer_list_head->next;
        cur->is_started = false;
        execute_callback( cur->callback, cur->context );
    }

    /* Remove all the expired object from the list */
    while( ( timer_list_head != NULL ) && ( timer_list_head->timestamp < hal_rtc_get_timer_elapsed_value( ) ) )
    {
        cur             = timer_list_head;
        timer_list_head = timer_list_head->next;
        cur->is_started = false;
        execute_callback( cur->callback, cur->context );
    }

    /* Start the next timer_list_head if it exists AND NOT running */
    if( ( timer_list_head != NULL ) && ( timer_list_head->is_next_2_expire == false ) )
    {
        timer_set_timeout( timer_list_head );
    }
}

void timer_stop( timer_event_t* obj )
{
    CRITICAL_SECTION_BEGIN( );

    timer_event_t* prev = timer_list_head;
    timer_event_t* cur  = timer_list_head;

    /* List is empty or the obj to stop does not exist */
    if( ( timer_list_head == NULL ) || ( obj == NULL ) )
    {
        CRITICAL_SECTION_END( );
        return;
    }

    obj->is_started = false;

    if( timer_list_head == obj )  // Stop the Head
    {
        if( timer_list_head->is_next_2_expire == true )  // The head is already running
        {
            timer_list_head->is_next_2_expire = false;
            if( timer_list_head->next != NULL )
            {
                timer_list_head = timer_list_head->next;
                timer_set_timeout( timer_list_head );
            }
            else
            {
                hal_rtc_stop_alarm( );
                timer_list_head = NULL;
            }
        }
        else  // Stop the head before it is started
        {
            if( timer_list_head->next != NULL )
            {
                timer_list_head = timer_list_head->next;
            }
            else
            {
                timer_list_head = NULL;
            }
        }
    }
    else  // Stop an object within the list
    {
        while( cur != NULL )
        {
            if( cur == obj )
            {
                if( cur->next != NULL )
                {
                    cur        = cur->next;
                    prev->next = cur;
                }
                else
                {
                    cur        = NULL;
                    prev->next = cur;
                }
                break;
            }
            else
            {
                prev = cur;
                cur  = cur->next;
            }
        }
    }
    CRITICAL_SECTION_END( );
}

static bool timer_exists( timer_event_t* obj )
{
    timer_event_t* cur = timer_list_head;

    while( cur != NULL )
    {
        if( cur == obj )
        {
            return true;
        }
        cur = cur->next;
    }
    return false;
}

void timer_reset( timer_event_t* obj )
{
    timer_stop( obj );
    timer_start( obj );
}

void timer_set_value( timer_event_t* obj, uint32_t value )
{
    uint32_t min_value = 0;
    uint32_t ticks     = hal_rtc_ms_2_tick( value );

    timer_stop( obj );

    min_value = hal_rtc_get_minimum_timeout( );

    if( ticks < min_value )
    {
        ticks = min_value;
    }

    obj->timestamp    = ticks;
    obj->reload_value = ticks;
}

timer_time_t timer_get_elapsed_time( timer_time_t past )
{
    if( past == 0 )
    {
        return 0;
    }
    uint32_t nowInTicks  = hal_rtc_get_timer_value( );
    uint32_t pastInTicks = hal_rtc_ms_2_tick( past );

    /* Intentional wrap around. Works Ok if tick duration below 1ms */
    return hal_rtc_tick_2_ms( nowInTicks - pastInTicks );
}

static void timer_set_timeout( timer_event_t* obj )
{
    int32_t min_ticks     = hal_rtc_get_minimum_timeout( );
    obj->is_next_2_expire = true;

    /* In case deadline too soon */
    if( obj->timestamp < ( hal_rtc_get_timer_elapsed_value( ) + min_ticks ) )
    {
        obj->timestamp = hal_rtc_get_timer_elapsed_value( ) + min_ticks;
    }
    hal_rtc_start_alarm( obj->timestamp );
}

timer_time_t timer_temp_compensation( timer_time_t period, float temperature )
{
    return hal_rtc_temp_compensation( period, temperature );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
