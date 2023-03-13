/*!
 * @file      smtc_hal_mcu_timer_stm32l4.c
 *
 * @brief      Implementation of timer module on top of STM32L4 Low Level drivers
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

#include "stm32l4xx.h"
#include "smtc_hal_mcu_timer.h"
#include "smtc_hal_mcu_timer_stm32l4.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_lptim.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Maximum number of timer instances
 */
#ifndef SMTC_HAL_MCU_TIMER_STM32L4_N_INSTANCES_MAX
#define SMTC_HAL_MCU_TIMER_STM32L4_N_INSTANCES_MAX 1
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Structure defining a timer instance
 */
struct smtc_hal_mcu_timer_inst_s
{
    bool           is_cfged;
    LPTIM_TypeDef* tim;
    uint32_t       max_value;
    void ( *callback_expiry )( void );
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Array to store the timer instances
 */
static struct smtc_hal_mcu_timer_inst_s tim_inst_array[SMTC_HAL_MCU_TIMER_STM32L4_N_INSTANCES_MAX];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Check if a timer is configured
 *
 * @param [in] cfg timer configuration
 *
 * @retval true Timer configured
 * @retval false Timer not configured
 */
static bool smtc_hal_mcu_timer_stm32l4_is_configured( smtc_hal_mcu_timer_cfg_t cfg );

/**
 * @brief Get a pointer to the first free slot
 *
 * @retval Pointer to the first free slot - NULL if there is no space left
 */
static struct smtc_hal_mcu_timer_inst_s* smtc_hal_mcu_timer_stm32l4_get_free_slot( void );

/**
 * @brief Check if the instance given as parameter is genuine
 *
 * @param [in] inst Timer instance
 *
 * @retval true Instance is genuine
 * @retval false Instance is not genuine
 */
static bool smtc_hal_mcu_timer_stm32l4_is_real_inst( smtc_hal_mcu_timer_inst_t inst );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_hal_mcu_status_t smtc_hal_mcu_timer_init( const smtc_hal_mcu_timer_cfg_t      cfg,
                                               const smtc_hal_mcu_timer_cfg_app_t* cfg_app,
                                               smtc_hal_mcu_timer_inst_t*          inst )
{
    if( smtc_hal_mcu_timer_stm32l4_is_configured( cfg ) == true )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    struct smtc_hal_mcu_timer_inst_s* tim_cfg_slot = smtc_hal_mcu_timer_stm32l4_get_free_slot( );

    if( tim_cfg_slot == NULL )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    tim_cfg_slot->is_cfged        = false;
    tim_cfg_slot->tim             = cfg->tim;
    tim_cfg_slot->max_value       = 0xFFFF;
    tim_cfg_slot->callback_expiry = cfg_app->expiry_func;

    const LL_LPTIM_InitTypeDef LPTIM_InitStruct = {
        .ClockSource = LL_LPTIM_CLK_SOURCE_INTERNAL,
        .Prescaler   = LL_LPTIM_PRESCALER_DIV32,
        .Waveform    = LL_LPTIM_OUTPUT_WAVEFORM_PWM,
        .Polarity    = LL_LPTIM_OUTPUT_POLARITY_REGULAR,
    };

    if( tim_cfg_slot->tim == LPTIM1 )
    {
        LL_RCC_SetLPTIMClockSource( LL_RCC_LPTIM1_CLKSOURCE_LSI );

        NVIC_SetPriority( LPTIM1_IRQn, 0 );
        NVIC_EnableIRQ( LPTIM1_IRQn );

        LL_APB1_GRP1_EnableClock( LL_APB1_GRP1_PERIPH_LPTIM1 );
        while( LL_APB1_GRP1_IsEnabledClock( LL_APB1_GRP1_PERIPH_LPTIM1 ) != 1 )
        {
        }
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( LL_LPTIM_Init( tim_cfg_slot->tim, &LPTIM_InitStruct ) != SUCCESS )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    LL_LPTIM_SetCounterMode( tim_cfg_slot->tim, LL_LPTIM_COUNTER_MODE_INTERNAL );

    tim_cfg_slot->is_cfged = true;

    *inst = tim_cfg_slot;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_timer_start( smtc_hal_mcu_timer_inst_t inst, uint32_t timeout_in_ms )
{
    if( smtc_hal_mcu_timer_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    if( ( inst->tim == LPTIM1 ) || ( inst->tim == LPTIM2 ) )
    {
        LL_LPTIM_EnableIT_ARRM( inst->tim );

        LL_LPTIM_Enable( inst->tim );
        while( LL_LPTIM_IsEnabled( inst->tim ) != 1 )
        {
        }

        LL_LPTIM_ClearFlag_ARROK( inst->tim );
        LL_LPTIM_SetAutoReload( inst->tim, timeout_in_ms );
        while( LL_LPTIM_IsActiveFlag_ARROK( inst->tim ) != 1 )
        {
        }
        LL_LPTIM_ClearFlag_ARROK( inst->tim );

        LL_LPTIM_StartCounter( inst->tim, LL_LPTIM_OPERATING_MODE_ONESHOT );
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_timer_stop( smtc_hal_mcu_timer_inst_t inst )
{
    if( smtc_hal_mcu_timer_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    if( ( inst->tim == LPTIM1 ) || ( inst->tim == LPTIM2 ) )
    {
        LL_LPTIM_Disable( inst->tim );
        while( LL_LPTIM_IsEnabled( inst->tim ) != 0 )
        {
        }

        LL_LPTIM_DisableIT_ARRM( inst->tim );
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_timer_get_remaining_time( smtc_hal_mcu_timer_inst_t inst, uint32_t* value_in_ms )
{
    if( smtc_hal_mcu_timer_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    if( ( inst->tim == LPTIM1 ) || ( inst->tim == LPTIM2 ) )
    {
        uint32_t val[2];
        int      i = 0;

        val[i++ % 2] = LL_LPTIM_GetCounter( inst->tim );

        do
        {
            val[i++ % 2] = LL_LPTIM_GetCounter( inst->tim );

            if( i == 10 )
            {
                return SMTC_HAL_MCU_STATUS_ERROR;
            }
        } while( val[0] != val[1] );

        *value_in_ms = val[0];
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_timer_get_max_value( smtc_hal_mcu_timer_inst_t inst, uint32_t* value_in_ms )
{
    if( smtc_hal_mcu_timer_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    *value_in_ms = inst->max_value;

    return SMTC_HAL_MCU_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool smtc_hal_mcu_timer_stm32l4_is_configured( smtc_hal_mcu_timer_cfg_t cfg )
{
    for( int i = 0; i < SMTC_HAL_MCU_TIMER_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( tim_inst_array[i].is_cfged == true )
        {
            if( tim_inst_array[i].tim == cfg->tim )
            {
                return true;
            }
        }
    }

    return false;
}

static struct smtc_hal_mcu_timer_inst_s* smtc_hal_mcu_timer_stm32l4_get_free_slot( void )
{
    for( int i = 0; i < SMTC_HAL_MCU_TIMER_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( tim_inst_array[i].is_cfged == false )
        {
            return &tim_inst_array[i];
        }
    }

    return NULL;
}

static bool smtc_hal_mcu_timer_stm32l4_is_real_inst( smtc_hal_mcu_timer_inst_t inst )
{
    for( int i = 0; i < SMTC_HAL_MCU_TIMER_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( inst == &tim_inst_array[i] )
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief  This function handles LPTIM1 interrupts.
 */
void LPTIM1_IRQHandler( void )
{
    /* Check whether Autoreload match interrupt is pending */
    if( LL_LPTIM_IsActiveFlag_ARRM( LPTIM1 ) == 1 )
    {
        /* Clear the Autoreload match interrupt flag */
        LL_LPTIM_ClearFLAG_ARRM( LPTIM1 );

        for( int i = 0; i < SMTC_HAL_MCU_TIMER_STM32L4_N_INSTANCES_MAX; i++ )
        {
            if( tim_inst_array[i].tim == LPTIM1 )
            {
                if( tim_inst_array[i].callback_expiry != NULL )
                {
                    tim_inst_array[i].callback_expiry( );
                    return;
                }
            }
        }
    }
}

/* --- EOF ------------------------------------------------------------------ */
