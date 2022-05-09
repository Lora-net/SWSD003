/*!
 * @file      smtc_hal_rtc.c
 *
 * \brief     RTC Hardware Abstraction Layer implementation
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

#include <time.h>
#include <math.h>

#include "smtc_hal.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_ll_rtc.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * Calculates ceiling( X / N )
 */
#define DIVC( X, N ) ( ( ( X ) + ( N ) -1 ) / ( N ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/* MCU Wake Up Time */
#define MIN_ALARM_DELAY_IN_TICKS 3U  // in ticks

/* sub-second number of bits */
#define N_PREDIV_S 10U

/* Synchronous prediv */
#define PREDIV_S ( ( 1U << N_PREDIV_S ) - 1U )

/* Asynchronous prediv */
#define PREDIV_A ( ( 1U << ( 15U - N_PREDIV_S ) ) - 1U )

/* Sub-second mask definition */
#define ALARM_SUBSECOND_MASK ( N_PREDIV_S << RTC_ALRMASSR_MASKSS_Pos )

/* RTC Time base in us */
#define USEC_NUMBER 1000000U
#define MSEC_NUMBER ( USEC_NUMBER / 1000 )

#define COMMON_FACTOR 3U
#define CONV_NUMER ( MSEC_NUMBER >> COMMON_FACTOR )
#define CONV_DENOM ( 1U << ( N_PREDIV_S - COMMON_FACTOR ) )

/*!
 * @brief Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR ( ( uint32_t ) 366U )
#define DAYS_IN_YEAR ( ( uint32_t ) 365U )
#define SECONDS_IN_1DAY ( ( uint32_t ) 86400U )
#define SECONDS_IN_1HOUR ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE ( ( uint32_t ) 60U )
#define MINUTES_IN_1HOUR ( ( uint32_t ) 60U )
#define HOURS_IN_1DAY ( ( uint32_t ) 24U )

/*!
 * @brief Correction factors
 */
#define DAYS_IN_MONTH_CORRECTION_NORM ( ( uint32_t ) 0x99AAA0 )
#define DAYS_IN_MONTH_CORRECTION_LEAP ( ( uint32_t ) 0x445550 )

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief RTC timer context
 */
typedef struct
{
    uint32_t        time_ref_in_ticks;  // Reference time
    RTC_TimeTypeDef calendar_time;      // Reference time in calendar format
    RTC_DateTypeDef calendar_date;      // Reference date in calendar format
} rtc_context_t;

/*!
 * @brief RTC structure
 */
typedef struct hal_rtc_s
{
    RTC_HandleTypeDef handle;
    /*!
     * Keep the value of the RTC timer when the RTC alarm is set
     * Set with the \ref hal_rtc_set_context function
     * Value is kept as a Reference to calculate alarm
     */
    rtc_context_t context;
} hal_rtc_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

hal_rtc_t hal_rtc;

static volatile bool wut_timer_irq_happened = false;

/*!
 * @brief RTC Alarm
 */
static RTC_AlarmTypeDef rtc_alarm;

/*!
 * @brief Number of days in each month on a normal year
 */
static const uint8_t days_in_month[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * @brief Number of days in each month on a leap year
 */
static const uint8_t days_in_month_leap_year[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Converts time in ms to time in wake up timer ticks
 * Assuming WUCKSEL[2:0] = 000: RTCCLK/16 clock is selected
 *
 * @param [in] milliseconds Time in milliseconds
 * @returns ticks Time in wake up timer ticks
 */
static uint32_t hal_rtc_ms_2_wakeup_timer_tick( const uint32_t milliseconds );

/*!
 * @brief Converts time in s to time in wake up timer ticks
 * When RTCCLK = 32768 Hz and ck_spre (Synchronous prescaler output clock) is
 * adjusted to 1 Hz
 *
 * @param [in] seconds Time in seconds
 * @returns ticks Time in wake up timer ticks
 */
static uint32_t hal_rtc_s_2_wakeup_timer_tick( const uint32_t seconds );

/*!
 * @brief Get the elapsed time in seconds and milliseconds since RTC initialization
 *
 * @param [out] milliseconds Number of milliseconds elapsed since RTC
 *                           initialization
 * @returns seconds           Number of seconds elapsed since RTC initialization
 */
static uint32_t hal_rtc_get_calendar_time( uint16_t* milliseconds );

/*!
 * @brief Get current full resolution RTC timestamp in ticks
 *
 * @returns timestamp_in_ticks Current timestamp in ticks
 */
static uint64_t rtc_get_timestamp_in_ticks( RTC_DateTypeDef* date, RTC_TimeTypeDef* time );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_rtc_init( void )
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    hal_rtc.handle.Instance            = RTC;
    hal_rtc.handle.Init.HourFormat     = RTC_HOURFORMAT_24;
    hal_rtc.handle.Init.AsynchPrediv   = PREDIV_A;
    hal_rtc.handle.Init.SynchPrediv    = PREDIV_S;
    hal_rtc.handle.Init.OutPut         = RTC_OUTPUT_DISABLE;
    hal_rtc.handle.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
    hal_rtc.handle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hal_rtc.handle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

    if( HAL_RTC_Init( &hal_rtc.handle ) != HAL_OK )
    {
        mcu_panic( );
    }

    /* Initialize RTC counter to 0 */
    date.Year    = 0;
    date.Month   = RTC_MONTH_JANUARY;
    date.Date    = 1;
    date.WeekDay = RTC_WEEKDAY_MONDAY;
    HAL_RTC_SetDate( &hal_rtc.handle, &date, RTC_FORMAT_BIN );

    /*at 0:0:0*/
    time.Hours          = 0;
    time.Minutes        = 0;
    time.Seconds        = 0;
    time.SubSeconds     = 0;
    time.TimeFormat     = 0;
    time.StoreOperation = RTC_DAYLIGHTSAVING_NONE;
    time.DayLightSaving = RTC_STOREOPERATION_RESET;
    HAL_RTC_SetTime( &hal_rtc.handle, &time, RTC_FORMAT_BIN );

    /* Enable Direct Read of the calendar registers (not through Shadow
       registers) */
    HAL_RTCEx_EnableBypassShadow( &hal_rtc.handle );

    HAL_NVIC_SetPriority( RTC_Alarm_IRQn, 1, 0 );
    HAL_NVIC_EnableIRQ( RTC_Alarm_IRQn );

    /* Init alarm. */
    HAL_RTC_DeactivateAlarm( &hal_rtc.handle, RTC_ALARM_A );

    hal_rtc_set_time_ref_in_ticks( );
}

uint32_t hal_rtc_get_time_s( void )
{
    uint16_t milliseconds = 0;
    return hal_rtc_get_calendar_time( &milliseconds );
}

uint32_t hal_rtc_get_time_ms( void )
{
    uint32_t seconds      = 0;
    uint16_t milliseconds = 0;

    seconds = hal_rtc_get_calendar_time( &milliseconds );

    return seconds * 1000 + milliseconds;
}

void hal_rtc_stop_alarm( void )
{
    /* Disable the Alarm A interrupt */
    HAL_RTC_DeactivateAlarm( &hal_rtc.handle, RTC_ALARM_A );

    /* Clear RTC Alarm Flag */
    __HAL_RTC_ALARM_CLEAR_FLAG( &hal_rtc.handle, RTC_FLAG_ALRAF );

    /* Clear the EXTI's line Flag for RTC Alarm */
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG( );
}

/*!
 * @brief Sets the alarm
 *
 * @remark The alarm is set at now (read in this function) + timeout
 *
 * @param [in] timeout Duration of the Timer ticks
 */
void hal_rtc_start_alarm( uint32_t timeout )
{
    uint16_t        rtc_alarm_sub_seconds = 0;
    uint16_t        rtc_alarm_seconds     = 0;
    uint16_t        rtc_alarm_minutes     = 0;
    uint16_t        rtc_alarm_hours       = 0;
    uint16_t        rtc_alarm_days        = 0;
    RTC_TimeTypeDef time                  = hal_rtc.context.calendar_time;  // rtc_timer_context.calendar_time;
    RTC_DateTypeDef date                  = hal_rtc.context.calendar_date;

    hal_rtc_stop_alarm( );

    /*reverse counter */
    rtc_alarm_sub_seconds = PREDIV_S - time.SubSeconds;
    rtc_alarm_sub_seconds += ( timeout & PREDIV_S );
    /* convert timeout  to seconds */
    timeout >>= N_PREDIV_S;

    /* Convert microsecs to RTC format and add to 'Now' */
    rtc_alarm_days = date.Date;
    while( timeout >= SECONDS_IN_1DAY )
    {
        timeout -= SECONDS_IN_1DAY;
        rtc_alarm_days++;
    }

    /* Calc hours */
    rtc_alarm_hours = time.Hours;
    while( timeout >= SECONDS_IN_1HOUR )
    {
        timeout -= SECONDS_IN_1HOUR;
        rtc_alarm_hours++;
    }

    /* Calc minutes */
    rtc_alarm_minutes = time.Minutes;
    while( timeout >= SECONDS_IN_1MINUTE )
    {
        timeout -= SECONDS_IN_1MINUTE;
        rtc_alarm_minutes++;
    }

    /* Calc seconds */
    rtc_alarm_seconds = time.Seconds + timeout;

    /***** Correct for modulo*********/
    while( rtc_alarm_sub_seconds >= ( PREDIV_S + 1 ) )
    {
        rtc_alarm_sub_seconds -= ( PREDIV_S + 1 );
        rtc_alarm_seconds++;
    }

    while( rtc_alarm_seconds >= SECONDS_IN_1MINUTE )
    {
        rtc_alarm_seconds -= SECONDS_IN_1MINUTE;
        rtc_alarm_minutes++;
    }

    while( rtc_alarm_minutes >= MINUTES_IN_1HOUR )
    {
        rtc_alarm_minutes -= MINUTES_IN_1HOUR;
        rtc_alarm_hours++;
    }

    while( rtc_alarm_hours >= HOURS_IN_1DAY )
    {
        rtc_alarm_hours -= HOURS_IN_1DAY;
        rtc_alarm_days++;
    }

    if( date.Year % 4 == 0 )
    {
        if( rtc_alarm_days > days_in_month_leap_year[date.Month - 1] )
        {
            rtc_alarm_days = rtc_alarm_days % days_in_month_leap_year[date.Month - 1];
        }
    }
    else
    {
        if( rtc_alarm_days > days_in_month[date.Month - 1] )
        {
            rtc_alarm_days = rtc_alarm_days % days_in_month[date.Month - 1];
        }
    }

    /* Set RTC_AlarmStructure with calculated values */
    rtc_alarm.AlarmTime.SubSeconds     = PREDIV_S - rtc_alarm_sub_seconds;
    rtc_alarm.AlarmSubSecondMask       = ALARM_SUBSECOND_MASK;
    rtc_alarm.AlarmTime.Seconds        = rtc_alarm_seconds;
    rtc_alarm.AlarmTime.Minutes        = rtc_alarm_minutes;
    rtc_alarm.AlarmTime.Hours          = rtc_alarm_hours;
    rtc_alarm.AlarmDateWeekDay         = ( uint8_t ) rtc_alarm_days;
    rtc_alarm.AlarmTime.TimeFormat     = time.TimeFormat;
    rtc_alarm.AlarmDateWeekDaySel      = RTC_ALARMDATEWEEKDAYSEL_DATE;
    rtc_alarm.AlarmMask                = RTC_ALARMMASK_NONE;
    rtc_alarm.Alarm                    = RTC_ALARM_A;
    rtc_alarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    rtc_alarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;

    /* Set RTC_Alarm */
    HAL_RTC_SetAlarm_IT( &hal_rtc.handle, &rtc_alarm, RTC_FORMAT_BIN );
}

uint32_t hal_rtc_get_timer_value( void )
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    uint32_t timestamp_value = ( uint32_t ) rtc_get_timestamp_in_ticks( &date, &time );

    return ( timestamp_value );
}

uint32_t hal_rtc_get_timer_elapsed_value( void )
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    uint32_t timestamp_value = ( uint32_t ) rtc_get_timestamp_in_ticks( &date, &time );

    return ( ( uint32_t )( timestamp_value - hal_rtc.context.time_ref_in_ticks ) );
}

void hal_rtc_delay_in_ms( const uint32_t milliseconds )
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    uint64_t delay_in_ticks     = 0;
    uint64_t ref_delay_in_ticks = rtc_get_timestamp_in_ticks( &date, &time );

    delay_in_ticks = hal_rtc_ms_2_tick( milliseconds );

    /* Wait delay ms */
    while( ( ( rtc_get_timestamp_in_ticks( &date, &time ) - ref_delay_in_ticks ) ) < delay_in_ticks )
    {
        __NOP( );
    }
}

void hal_rtc_wakeup_timer_set_s( const int32_t seconds )
{
    uint32_t delay_s_2_tick = hal_rtc_s_2_wakeup_timer_tick( seconds );

    HAL_RTCEx_SetWakeUpTimer_IT( &hal_rtc.handle, delay_s_2_tick, RTC_WAKEUPCLOCK_CK_SPRE_16BITS );
}

void hal_rtc_wakeup_timer_set_ms( const int32_t milliseconds )
{
    uint32_t delay_ms_2_tick = hal_rtc_ms_2_wakeup_timer_tick( milliseconds );

    HAL_RTCEx_DeactivateWakeUpTimer( &hal_rtc.handle );
    // reset irq status
    wut_timer_irq_happened = false;
    HAL_RTCEx_SetWakeUpTimer_IT( &hal_rtc.handle, delay_ms_2_tick, RTC_WAKEUPCLOCK_RTCCLK_DIV16 );
}
void hal_rtc_wakeup_timer_stop( void ) { HAL_RTCEx_DeactivateWakeUpTimer( &hal_rtc.handle ); }

bool hal_rtc_has_wut_irq_happened( void ) { return wut_timer_irq_happened; }

uint32_t hal_rtc_set_time_ref_in_ticks( void )
{
    hal_rtc.context.time_ref_in_ticks =
        ( uint32_t ) rtc_get_timestamp_in_ticks( &hal_rtc.context.calendar_date, &hal_rtc.context.calendar_time );
    return hal_rtc.context.time_ref_in_ticks;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint32_t hal_rtc_get_time_ref_in_ticks( void ) { return hal_rtc.context.time_ref_in_ticks; }

uint32_t hal_rtc_ms_2_tick( const uint32_t milliseconds )
{
    return ( uint32_t )( ( ( ( uint64_t ) milliseconds ) * CONV_DENOM ) / CONV_NUMER );
}

uint32_t hal_rtc_tick_2_ms( const uint32_t tick )
{
    uint32_t seconds    = tick >> N_PREDIV_S;
    uint32_t local_tick = tick & PREDIV_S;

    return ( uint32_t )( ( seconds * 1000 ) + ( ( local_tick * 1000 ) >> N_PREDIV_S ) );
}

static uint32_t hal_rtc_ms_2_wakeup_timer_tick( const uint32_t milliseconds )
{
    uint32_t nb_tick = 0;
    /* Compute is done for LSE @ 32.768kHz
       Assuming that RTC_WAKEUPCLOCK_RTCCLK_DIV16 is used => tick is 488.281µs */
    nb_tick = milliseconds * 2 + ( ( 6 * milliseconds ) >> 7 );
    return nb_tick;
}

static uint32_t hal_rtc_s_2_wakeup_timer_tick( const uint32_t seconds )
{
    uint32_t nb_tick = 0;
    /* Compute is done for LSE @ 32.768kHz
       Assuming that RTC_WAKEUPCLOCK_CK_SPRE_16BITS is used => tick is 1s */
    nb_tick = seconds;
    return nb_tick;
}

static uint32_t hal_rtc_get_calendar_time( uint16_t* milliseconds )
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    uint32_t        ticks;

    uint64_t timestamp_in_ticks = rtc_get_timestamp_in_ticks( &date, &time );

    uint32_t seconds = ( uint32_t )( timestamp_in_ticks >> N_PREDIV_S );

    ticks = ( uint32_t ) timestamp_in_ticks & PREDIV_S;

    *milliseconds = hal_rtc_tick_2_ms( ticks );

    return seconds;
}

/*!
 * @brief RTC IRQ Handler of the RTC Alarm
 */
void RTC_Alarm_IRQHandler( void )
{
    RTC_HandleTypeDef* hrtc = &hal_rtc.handle;

    /* Clear the EXTI's line Flag for RTC Alarm */
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG( );

    /* Gets the AlarmA interrupt source enable status */
    if( __HAL_RTC_ALARM_GET_IT_SOURCE( hrtc, RTC_IT_ALRA ) != RESET )
    {
        /* Gets the pending status of the AlarmA interrupt */
        if( __HAL_RTC_ALARM_GET_FLAG( hrtc, RTC_FLAG_ALRAF ) != RESET )
        {
            /* Clear the AlarmA interrupt pending bit */
            __HAL_RTC_ALARM_CLEAR_FLAG( hrtc, RTC_FLAG_ALRAF );
            /* AlarmA callback */
            HAL_RTC_AlarmAEventCallback( hrtc );
        }
    }
}

/*!
 * @brief  Alarm A callback.
 *
 * @param [in] hrtc RTC handle
 */
void HAL_RTC_AlarmAEventCallback( RTC_HandleTypeDef* hrtc ) { timer_irq_handler( ); }

static uint64_t rtc_get_timestamp_in_ticks( RTC_DateTypeDef* date, RTC_TimeTypeDef* time )
{
    uint64_t timestamp_in_ticks = 0;
    uint32_t correction;
    uint32_t seconds;

    /* Make sure it is correct due to asynchronous nature of RTC */
    volatile uint32_t ssr;

    do
    {
        ssr = RTC->SSR;
        HAL_RTC_GetDate( &hal_rtc.handle, date, RTC_FORMAT_BIN );
        HAL_RTC_GetTime( &hal_rtc.handle, time, RTC_FORMAT_BIN );
    } while( ssr != RTC->SSR );

    /* Calculate amount of elapsed days since 01/01/2000 */
    seconds = DIVC( ( DAYS_IN_YEAR * 3 + DAYS_IN_LEAP_YEAR ) * date->Year, 4 );

    correction = ( ( date->Year % 4 ) == 0 ) ? DAYS_IN_MONTH_CORRECTION_LEAP : DAYS_IN_MONTH_CORRECTION_NORM;

    seconds +=
        ( DIVC( ( date->Month - 1 ) * ( 30 + 31 ), 2 ) - ( ( ( correction >> ( ( date->Month - 1 ) * 2 ) ) & 0x03 ) ) );

    seconds += ( date->Date - 1 );

    /* Convert from days to seconds */
    seconds *= SECONDS_IN_1DAY;

    seconds += ( ( uint32_t ) time->Seconds + ( ( uint32_t ) time->Minutes * SECONDS_IN_1MINUTE ) +
                 ( ( uint32_t ) time->Hours * SECONDS_IN_1HOUR ) );

    timestamp_in_ticks = ( ( ( uint64_t ) seconds ) << N_PREDIV_S ) + ( PREDIV_S - time->SubSeconds );

    return timestamp_in_ticks;
}

void RTC_WKUP_IRQHandler( void )
{
    HAL_RTCEx_WakeUpTimerIRQHandler( &hal_rtc.handle );
    wut_timer_irq_happened = true;
}

void HAL_RTC_MspInit( RTC_HandleTypeDef* rtc_handle )
{
    __HAL_RCC_RTC_ENABLE( );
    HAL_NVIC_SetPriority( RTC_WKUP_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( RTC_WKUP_IRQn );
}

void HAL_RTC_MspDeInit( RTC_HandleTypeDef* rtc_handle )
{
    __HAL_RCC_RTC_DISABLE( );
    HAL_NVIC_DisableIRQ( RTC_WKUP_IRQn );
}

uint32_t hal_rtc_get_minimum_timeout( void ) { return ( MIN_ALARM_DELAY_IN_TICKS ); }

uint32_t hal_rtc_temp_compensation( uint32_t period, float temperature )
{
    float k       = RTC_TEMP_COEFFICIENT;
    float k_dev   = RTC_TEMP_DEV_COEFFICIENT;
    float t       = RTC_TEMP_TURNOVER;
    float t_dev   = RTC_TEMP_DEV_TURNOVER;
    float interim = 0.0;
    float ppm     = 0.0;

    if( k < ( float ) 0.0 )
    {
        ppm = ( k - k_dev );
    }
    else
    {
        ppm = ( k + k_dev );
    }
    interim = ( temperature - ( t - t_dev ) );
    ppm *= interim * interim;

    /* Calculate the drift in time */
    interim = ( ( float ) period * ppm ) / ( ( float ) 1e6 );
    /* Calculate the resulting time period */
    interim += period;
    interim = floor( interim );

    if( interim < ( float ) 0.0 )
    {
        interim = ( float ) period;
    }

    /* Calculate the resulting period */
    return ( uint32_t ) interim;
}
