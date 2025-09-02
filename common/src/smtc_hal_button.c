/*!
 * @file      smtc_hal_button.c
 *
 * @brief     User button implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2024. All rights reserved.
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
#include "smtc_hal_button.h"
#include "smtc_hal_mcu_gpio.h"
#include "smtc_hal_mcu_gpio_stm32l4.h"
#include "stm32l4xx_ll_gpio.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

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

static struct
{
    smtc_hal_mcu_gpio_cfg_t       cfg;
    smtc_hal_mcu_gpio_input_cfg_t cfg_input;
    smtc_hal_mcu_gpio_inst_t      inst;
} usr_btn;

static const struct smtc_hal_mcu_gpio_cfg_s usr_btn_cfg = { .port = GPIOC, .pin = LL_GPIO_PIN_13 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DECLARATIONS -------------------------------------------
 */

/*!
 * @brief Returns the user button configuration
 */
static const smtc_hal_mcu_gpio_cfg_t smtc_hal_button_get_user_btn_cfg( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION DEFINITIONS ---------------------------------------------
 */

void smtc_hal_button_init_user_btn( void ( *callback )( void* ), void* context )
{
    usr_btn.cfg                 = smtc_hal_button_get_user_btn_cfg( );
    usr_btn.cfg_input.pull_mode = SMTC_HAL_MCU_GPIO_PULL_MODE_NONE;
    usr_btn.cfg_input.irq_mode  = SMTC_HAL_MCU_GPIO_IRQ_MODE_RISING;
    usr_btn.cfg_input.callback  = callback;
    usr_btn.cfg_input.context   = context;
    smtc_hal_mcu_gpio_inst_t instance;
    smtc_hal_mcu_gpio_init_input( usr_btn.cfg, &usr_btn.cfg_input, &instance );
    smtc_hal_mcu_gpio_enable_irq( instance );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static const smtc_hal_mcu_gpio_cfg_t smtc_hal_button_get_user_btn_cfg( void )
{
    return ( const smtc_hal_mcu_gpio_cfg_t ) &usr_btn_cfg;
}

/* --- EOF ------------------------------------------------------------------ */
