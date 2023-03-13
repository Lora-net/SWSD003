/*!
 * @file      smtc_hal_mcu_rng_stm32l4.c
 *
 * @brief      Implementation of RNG (Random Number Generator) module on top of STM32L4 Low Level drivers
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
#include "smtc_hal_mcu_rng.h"
#include "smtc_hal_mcu_rng_stm32l4.h"
#include "stm32l4xx_ll_rng.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Maximum number of RNG instances
 */
#ifndef SMTC_HAL_MCU_RNG_STM32L4_N_INSTANCES_MAX
#define SMTC_HAL_MCU_RNG_STM32L4_N_INSTANCES_MAX 1
#endif

/**
 * @brief Get the minimum out of two integers
 */
#define MIN( a, b ) ( ( a < b ) ? a : b )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * @brief Structure defining a RNG instance
 */
struct smtc_hal_mcu_rng_inst_s
{
    bool         is_cfged;
    RNG_TypeDef* rng;
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Array to store the RNG instances
 */
static struct smtc_hal_mcu_rng_inst_s rng_inst_array[SMTC_HAL_MCU_RNG_STM32L4_N_INSTANCES_MAX];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Get a pointer to the first free slot
 *
 * @retval Pointer to the first free slot - NULL if there is no space left
 */
static struct smtc_hal_mcu_rng_inst_s* smtc_hal_mcu_rng_stm32l4_get_free_slot( void );

/**
 * @brief Check if the instance given as parameter is genuine
 *
 * @param [in] inst RNG instance
 *
 * @retval true Instance is genuine
 * @retval false Instance is not genuine
 */
static bool smtc_hal_mcu_rng_stm32l4_is_real_inst( smtc_hal_mcu_rng_inst_t inst );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_hal_mcu_status_t smtc_hal_mcu_rng_init( const smtc_hal_mcu_rng_cfg_t cfg, smtc_hal_mcu_rng_inst_t* inst )
{
    struct smtc_hal_mcu_rng_inst_s* rng_cfg_slot = smtc_hal_mcu_rng_stm32l4_get_free_slot( );

    if( rng_cfg_slot == NULL )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    if( cfg->rng == RNG )
    {
        rng_cfg_slot->rng = cfg->rng;

        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_RNG );

        LL_RCC_PLLSAI1_ConfigDomain_48M( LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 24, LL_RCC_PLLSAI1Q_DIV_2 );
        LL_RCC_PLLSAI1_Enable( );
        LL_RCC_PLLSAI1_EnableDomain_48M( );
        while( LL_RCC_PLLSAI1_IsReady( ) != 1 )
        {
        };

        LL_RCC_SetRNGClockSource( LL_RCC_RNG_CLKSOURCE_PLLSAI1 );

        LL_RNG_Enable( rng_cfg_slot->rng );
        while( LL_RNG_IsEnabled( rng_cfg_slot->rng ) != 1 )
        {
        };

        rng_cfg_slot->is_cfged = true;

        *inst = rng_cfg_slot;

        return SMTC_HAL_MCU_STATUS_OK;
    }

    return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
}

smtc_hal_mcu_status_t smtc_hal_mcu_rng_deinit( smtc_hal_mcu_rng_inst_t* inst )
{
    smtc_hal_mcu_rng_inst_t inst_local = *inst;

    if( smtc_hal_mcu_rng_stm32l4_is_real_inst( inst_local ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst_local->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    LL_RNG_Disable( inst_local->rng );

    LL_RCC_SetRNGClockSource( LL_RCC_RNG_CLKSOURCE_NONE );

    LL_RCC_PLLSAI1_DisableDomain_48M( );
    LL_RCC_PLLSAI1_Disable( );
    LL_RCC_PLLSAI1_ConfigDomain_48M( LL_RCC_PLLSOURCE_NONE, LL_RCC_PLLM_DIV_1, 24, LL_RCC_PLLSAI1Q_DIV_2 );

    LL_AHB2_GRP1_DisableClock( LL_AHB2_GRP1_PERIPH_RNG );

    inst_local->is_cfged = false;
    *inst                = NULL;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_rng_get_bytes( smtc_hal_mcu_rng_inst_t inst, uint8_t* buffer, unsigned int length )
{
    unsigned int remaining_length = length;

    if( smtc_hal_mcu_rng_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    while( remaining_length > 0 )
    {
        while( !LL_RNG_IsActiveFlag_DRDY( inst->rng ) )
        {
        }

        if( ( LL_RNG_IsActiveFlag_CECS( inst->rng ) ) || ( LL_RNG_IsActiveFlag_SECS( inst->rng ) ) )
        {
            return SMTC_HAL_MCU_STATUS_ERROR;
        }

        const uint32_t     rng_32 = LL_RNG_ReadRandData32( inst->rng );
        const unsigned int tmp    = MIN( remaining_length, 4 );

        for( int i = 0; i < tmp; i++ )
        {
            buffer[length - remaining_length--] = ( uint8_t ) ( rng_32 >> ( 8 * i ) );
        }
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static struct smtc_hal_mcu_rng_inst_s* smtc_hal_mcu_rng_stm32l4_get_free_slot( void )
{
    for( int i = 0; i < SMTC_HAL_MCU_RNG_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( rng_inst_array[i].is_cfged == false )
        {
            return &rng_inst_array[i];
        }
    }

    return NULL;
}

static bool smtc_hal_mcu_rng_stm32l4_is_real_inst( smtc_hal_mcu_rng_inst_t inst )
{
    for( int i = 0; i < SMTC_HAL_MCU_RNG_STM32L4_N_INSTANCES_MAX; i++ )
    {
        if( inst == &rng_inst_array[i] )
        {
            return true;
        }
    }

    return false;
}

/* --- EOF ------------------------------------------------------------------ */
