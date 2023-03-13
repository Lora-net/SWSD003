/*!
 * @file      curve_plot.c
 *
 * @brief     Curve_plot functions used in Spectrum_display example for LR11xx chip
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

#include <stdlib.h>
#include <stdint.h>

#include "curve_plot.h"
#include "../main_spectrum_display.h"
#include "smtc_hal_dbg_trace.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * @brief Curve position offset from home position of the canvas
 */
#define COLUMN_OFFSET 5
#define ROW_OFFSET 1
#define X_AXIS_WIDTH 2

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

const static uint8_t rssi_level_num = ( ( RSSI_TOP_LEVEL_DBM - RSSI_BOTTOM_LEVEL_DBM ) / RSSI_SCALE + 1 );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void plot_y_axis( void );
static void plot_x_axis( void );
static void set_cursor_on_canvas( uint16_t row, uint16_t column );
static void move_cursor( int16_t x, int16_t y );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void create_canvas( void )
{
    for( uint8_t i = 1; i < rssi_level_num + ROW_OFFSET + X_AXIS_WIDTH; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "\n" );
    }
    HAL_DBG_TRACE_PRINTF( "\033[%dA", ( rssi_level_num + ROW_OFFSET + X_AXIS_WIDTH - 1 ) );  // Cursor up

    /* Set the home of canvas */
    HAL_DBG_TRACE_PRINTF( "\033[s" );  // Save cursor of home
}

void plot_curve( uint8_t column, uint8_t row )
{
    static uint16_t curve_levels[NB_CHAN] = { 0 };

    curve_levels[column - 1] = row;  // Save rssi level
    if( column == 1 )                // First column
    {
        /* Clear canvas, plot x-axis, y-axis */
        HAL_DBG_TRACE_PRINTF( "\033[u" );   // Unsave cursor, go back to home of canvas
        HAL_DBG_TRACE_PRINTF( "\033[J" );   // Erase down
        HAL_DBG_TRACE_PRINTF( "\033[7l" );  // Disable line wrap
        HAL_DBG_TRACE_PRINTF( "\033[0m" );  // Reset print color
        plot_y_axis( );
        plot_x_axis( );
        HAL_DBG_TRACE_PRINTF( "\033[0;32m" );  // Set green color for the curve
    }
    else if( column <= NB_CHAN )  // Other column
    {
        /* Improve curve continuity */
        if( row > curve_levels[column - 2] )  // Go down
        {
            for( uint8_t i = 1; row > curve_levels[column - 2] + i; i++ )
            {
                set_cursor_on_canvas( ( curve_levels[column - 2] + i ), ( ( column - 1 ) * 2 + COLUMN_OFFSET ) );
                HAL_DBG_TRACE_PRINTF( "|" );
            }
            set_cursor_on_canvas( row, ( ( column - 1 ) * 2 + COLUMN_OFFSET ) );
            HAL_DBG_TRACE_PRINTF( "|" );
        }
        else if( row < curve_levels[column - 2] )  // Go up
        {
            for( uint8_t i = 0; row < curve_levels[column - 2] - i; i++ )
            {
                set_cursor_on_canvas( ( curve_levels[column - 2] - i ), ( ( column - 1 ) * 2 + COLUMN_OFFSET ) );
                HAL_DBG_TRACE_PRINTF( "|" );
            }
        }
        else
        {
            set_cursor_on_canvas( row, ( ( column - 1 ) * 2 + COLUMN_OFFSET ) );
            HAL_DBG_TRACE_PRINTF( "." );
        }
    }

    set_cursor_on_canvas( row, ( column * 2 - 1 + COLUMN_OFFSET ) );
    HAL_DBG_TRACE_PRINTF( "_" );  // Plot symbol representing RSSI level
}

void plot_y_axis( void )
{
    set_cursor_on_canvas( 1, COLUMN_OFFSET );  // Go y-axis column
    HAL_DBG_TRACE_PRINTF( "^" );
    for( uint8_t i = 1; i <= rssi_level_num; i++ )
    {
        set_cursor_on_canvas( ( i + ROW_OFFSET ), 1 );
        HAL_DBG_TRACE_PRINTF( "%*d", 4, ( -1 * ( i - 1 ) * RSSI_SCALE ) );  // Print RSSI scales
        set_cursor_on_canvas( ( i + ROW_OFFSET ), COLUMN_OFFSET );
        HAL_DBG_TRACE_PRINTF( "|" );
    }
    set_cursor_on_canvas( ( rssi_level_num + ROW_OFFSET + 1 ), 1 );  // Go unit position
    HAL_DBG_TRACE_PRINTF( "/dBm" );
}

void plot_x_axis( void )
{
    set_cursor_on_canvas( ( rssi_level_num + ROW_OFFSET + 1 ), COLUMN_OFFSET );  // Go x-axis line
    HAL_DBG_TRACE_PRINTF( "x" );
    for( uint8_t i = 0; i < NB_CHAN; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "--" );
    }
    HAL_DBG_TRACE_PRINTF( ">" );

    set_cursor_on_canvas( ( rssi_level_num + ROW_OFFSET + 2 ), ( COLUMN_OFFSET + 1 ) );  // Go x-axis scale line
    HAL_DBG_TRACE_PRINTF( "%.0f --> %.0f MHz", ( FREQ_START_HZ / 10E5 ),
                          ( ( FREQ_START_HZ + ( WIDTH_CHAN_HZ * NB_CHAN ) ) / 10E5 ) );
}

void set_cursor_on_canvas( uint16_t row, uint16_t column )
{
    HAL_DBG_TRACE_PRINTF( "\033[u" );                     // Unsave cursor, go back to home of canvas
    move_cursor( ( column - 1 ), ( -1 * ( row - 1 ) ) );  // Default row direction is downward
}

void move_cursor( int16_t x, int16_t y )
{
    if( x > 0 )  // Move forward
    {
        HAL_DBG_TRACE_PRINTF( "\033[%dC", x );
    }
    else if( x < 0 )  // Move backward
    {
        HAL_DBG_TRACE_PRINTF( "\033[%dD", abs( x ) );
    }

    if( y > 0 )  // Move up
    {
        HAL_DBG_TRACE_PRINTF( "\033[%dA", y );
    }
    else if( y < 0 )  // Move down
    {
        HAL_DBG_TRACE_PRINTF( "\033[%dB", abs( y ) );
    }
}
