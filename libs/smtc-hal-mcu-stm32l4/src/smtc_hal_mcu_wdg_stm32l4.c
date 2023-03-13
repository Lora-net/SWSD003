/*!
 * @file      smtc_hal_mcu_wdog_stm32l4.c
 *
 * @brief      Implementation of watchdog module on top of STM32L4 Low Level drivers
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
#include "smtc_hal_mcu_wdg.h"
#include "smtc_hal_mcu_wdg_stm32l4.h"
#include "stm32l4xx_ll_iwdg.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/**
 * @brief Array storing the maximum timeout per pre-scaler value
 */
uint32_t smtc_hal_mcu_wdg_max_timeout_per_pr[7][2] = {
    { LL_IWDG_PRESCALER_4, 512 },     { LL_IWDG_PRESCALER_8, 1024 },  { LL_IWDG_PRESCALER_16, 2048 },
    { LL_IWDG_PRESCALER_32, 4096 },   { LL_IWDG_PRESCALER_64, 8192 }, { LL_IWDG_PRESCALER_128, 16384 },
    { LL_IWDG_PRESCALER_256, 32768 },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Structure defining a watchdog instance
 */
struct smtc_hal_mcu_wdg_inst_s
{
    bool          is_cfged;
    IWDG_TypeDef* iwdg;
};

/**
 * @brief Watchdog instance
 */
static struct smtc_hal_mcu_wdg_inst_s wdg_inst = {
    .is_cfged = false,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Check if the instance given as parameter is genuine
 *
 * @param [in] inst watchdog instance
 *
 * @retval true Instance is genuine
 * @retval false Instance is not genuine
 */
static bool smtc_hal_mcu_wdg_stm32l4_is_real_inst( smtc_hal_mcu_wdg_inst_t inst );

/**
 * @brief Select the prescaler and reload values for a given timeout value in millisecond
 *
 * @param [in] timeout_in_ms Timeout in millisecond
 * @param [out] pr Pointer to the prescaler value
 * @param [out] rl Pointer to the reload value
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The paramaters are available
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS The timeout is too high to be configured
 */
static smtc_hal_mcu_status_t smtc_hal_mcu_wdg_stm32l4_get_pr_rl( uint32_t timeout_in_ms, uint8_t* pr, uint16_t* rl );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_hal_mcu_status_t smtc_hal_mcu_wdg_init_and_start( smtc_hal_mcu_wdg_cfg_t            cfg,
                                                       const smtc_hal_mcu_wdg_cfg_app_t* cfg_app,
                                                       smtc_hal_mcu_wdg_inst_t*          inst )
{
    uint8_t  prescaler;
    uint16_t reload_counter;

    if( wdg_inst.is_cfged == true )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    const smtc_hal_mcu_status_t status =
        smtc_hal_mcu_wdg_stm32l4_get_pr_rl( cfg_app->timeout_ms, &prescaler, &reload_counter );

    if( status != SMTC_HAL_MCU_STATUS_OK )
    {
        return status;
    }

    wdg_inst.iwdg = cfg->iwdg;

    LL_RCC_LSI_Enable( );
    while( LL_RCC_LSI_IsReady( ) != 1 )
    {
    }

    LL_IWDG_Enable( wdg_inst.iwdg );
    LL_IWDG_EnableWriteAccess( wdg_inst.iwdg );
    LL_IWDG_SetPrescaler( wdg_inst.iwdg, prescaler );
    LL_IWDG_SetReloadCounter( wdg_inst.iwdg, reload_counter );
    while( LL_IWDG_IsReady( wdg_inst.iwdg ) != 1 )
    {
    }
    LL_IWDG_ReloadCounter( wdg_inst.iwdg );

    wdg_inst.is_cfged = true;

    *inst = &wdg_inst;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_wdg_stop( smtc_hal_mcu_wdg_inst_t inst ) { return SMTC_HAL_MCU_STATUS_ERROR; }

smtc_hal_mcu_status_t smtc_hal_mcu_wdg_reload( smtc_hal_mcu_wdg_inst_t inst )
{
    if( smtc_hal_mcu_wdg_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    LL_IWDG_ReloadCounter( inst->iwdg );

    return SMTC_HAL_MCU_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool smtc_hal_mcu_wdg_stm32l4_is_real_inst( smtc_hal_mcu_wdg_inst_t inst )
{
    if( inst == &wdg_inst )
    {
        return true;
    }

    return false;
}

static smtc_hal_mcu_status_t smtc_hal_mcu_wdg_stm32l4_get_pr_rl( uint32_t timeout_in_ms, uint8_t* pr, uint16_t* rl )
{
    for( int i = 0; i < 7; i++ )
    {
        if( timeout_in_ms < smtc_hal_mcu_wdg_max_timeout_per_pr[i][1] )
        {
            *pr = smtc_hal_mcu_wdg_max_timeout_per_pr[i][0];
            *rl = ( timeout_in_ms << 3 ) >> *pr - 1;

            return SMTC_HAL_MCU_STATUS_OK;
        }
    }

    return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
}

/* --- EOF ------------------------------------------------------------------ */
