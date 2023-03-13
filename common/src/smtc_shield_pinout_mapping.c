/*!
 * @file      smtc_shield_pinout_mapping.c
 *
 * @brief     Arduino mapping implementation
 *
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>
#include "smtc_shield_pinout.h"
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

const struct smtc_hal_mcu_gpio_cfg_s arduino_mapping[22] = {
    { .port = GPIOA, .pin = LL_GPIO_PIN_3 },  { .port = GPIOA, .pin = LL_GPIO_PIN_2 },
    { .port = GPIOA, .pin = LL_GPIO_PIN_10 }, { .port = GPIOB, .pin = LL_GPIO_PIN_3 },
    { .port = GPIOB, .pin = LL_GPIO_PIN_5 },  { .port = GPIOB, .pin = LL_GPIO_PIN_4 },
    { .port = GPIOB, .pin = LL_GPIO_PIN_10 }, { .port = GPIOA, .pin = LL_GPIO_PIN_8 },
    { .port = GPIOA, .pin = LL_GPIO_PIN_9 },  { .port = GPIOC, .pin = LL_GPIO_PIN_7 },
    { .port = GPIOB, .pin = LL_GPIO_PIN_6 },  { .port = GPIOA, .pin = LL_GPIO_PIN_7 },
    { .port = GPIOA, .pin = LL_GPIO_PIN_6 },  { .port = GPIOA, .pin = LL_GPIO_PIN_5 },
    { .port = GPIOB, .pin = LL_GPIO_PIN_9 },  { .port = GPIOB, .pin = LL_GPIO_PIN_8 },
    { .port = GPIOA, .pin = LL_GPIO_PIN_0 },  { .port = GPIOA, .pin = LL_GPIO_PIN_1 },
    { .port = GPIOA, .pin = LL_GPIO_PIN_4 },  { .port = GPIOB, .pin = LL_GPIO_PIN_0 },
    { .port = GPIOC, .pin = LL_GPIO_PIN_1 },  { .port = GPIOC, .pin = LL_GPIO_PIN_0 },
};

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

const smtc_hal_mcu_gpio_cfg_t smtc_shield_pinout_mapping_get_gpio_cfg( smtc_shield_pinout_t gpio )
{
    return ( const smtc_hal_mcu_gpio_cfg_t ) &arduino_mapping[( int ) gpio];
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
