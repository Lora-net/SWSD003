/*!
 * \file      lr11xx_hal_context.h
 *
 * \brief     Declaration of LR11XX HAL context
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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
#ifndef LR11XX_HAL_CONTEXT_H
#define LR11XX_HAL_CONTEXT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_hal_mcu_spi.h"
#include "smtc_hal_mcu_spi_stm32l4.h"
#include "smtc_hal_mcu_gpio.h"
#include "smtc_hal_mcu_gpio_stm32l4.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_spi.h"

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

typedef struct
{
    struct
    {
        struct smtc_hal_mcu_spi_cfg_s cfg;
        smtc_hal_mcu_spi_inst_t       inst;
    } spi;
    struct
    {
        smtc_hal_mcu_gpio_cfg_t        cfg;
        smtc_hal_mcu_gpio_output_cfg_t cfg_output;
        smtc_hal_mcu_gpio_inst_t       inst;
    } nss;
    struct
    {
        smtc_hal_mcu_gpio_cfg_t        cfg;
        smtc_hal_mcu_gpio_output_cfg_t cfg_output;
        smtc_hal_mcu_gpio_inst_t       inst;
    } reset;
    struct
    {
        smtc_hal_mcu_gpio_cfg_t       cfg;
        smtc_hal_mcu_gpio_input_cfg_t cfg_input;
        smtc_hal_mcu_gpio_inst_t      inst;
    } irq;
    struct
    {
        smtc_hal_mcu_gpio_cfg_t       cfg;
        smtc_hal_mcu_gpio_input_cfg_t cfg_input;
        smtc_hal_mcu_gpio_inst_t      inst;
    } busy;
} lr11xx_hal_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_HAL_CONTEXT_H