/**
 * @file      smtc_hal_mcu_gpio.h
 *
 * @brief
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

#ifndef SMTC_HAL_MCU_GPIO_H
#define SMTC_HAL_MCU_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "smtc_hal_mcu_status.h"

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

/**
 * @brief GPIO states
 */
typedef enum smtc_hal_mcu_gpio_state_e
{
    SMTC_HAL_MCU_GPIO_STATE_LOW = 0U,
    SMTC_HAL_MCU_GPIO_STATE_HIGH,
} smtc_hal_mcu_gpio_state_t;

/**
 * @brief GPIO output modes
 */
typedef enum smtc_hal_mcu_gpio_output_mode_e
{
    SMTC_HAL_MCU_GPIO_OUTPUT_MODE_OPEN_DRAIN = 0U,
    SMTC_HAL_MCU_GPIO_OUTPUT_MODE_OPEN_DRAIN_WITH_PULL_UP,
    SMTC_HAL_MCU_GPIO_OUTPUT_MODE_PUSH_PULL,
} smtc_hal_mcu_gpio_output_mode_t;

/**
 * @brief GPIO pull modes
 */
typedef enum smtc_hal_mcu_gpio_pull_mode_e
{
    SMTC_HAL_MCU_GPIO_PULL_MODE_NONE = 0U,
    SMTC_HAL_MCU_GPIO_PULL_MODE_UP,
    SMTC_HAL_MCU_GPIO_PULL_MODE_DOWN,
} smtc_hal_mcu_gpio_pull_mode_t;

/**
 * @brief GPIO interrupt modes
 */
typedef enum smtc_hal_mcu_gpio_irq_mode_e
{
    SMTC_HAL_MCU_GPIO_IRQ_MODE_OFF = 0U,
    SMTC_HAL_MCU_GPIO_IRQ_MODE_RISING,
    SMTC_HAL_MCU_GPIO_IRQ_MODE_FALLING,
    SMTC_HAL_MCU_GPIO_IRQ_MODE_RISING_FALLING,
} smtc_hal_mcu_gpio_irq_mode_t;

/**
 * @brief Implementation-level GPIO instance structure definition
 *
 * @remark smtc_hal_mcu_gpio_inst_s has to be defined in the implementation
 */
typedef struct smtc_hal_mcu_gpio_inst_s* smtc_hal_mcu_gpio_inst_t;

/**
 * @brief Implementation-level GPIO configuration structure definition
 *
 * @remark smtc_hal_mcu_gpio_cfg_s has to be defined in the implementation
 */
typedef struct smtc_hal_mcu_gpio_cfg_s* smtc_hal_mcu_gpio_cfg_t;

/**
 * @brief GPIO output configuration structure
 */
typedef struct smtc_hal_mcu_gpio_output_cfg_s
{
    smtc_hal_mcu_gpio_state_t       initial_state;
    smtc_hal_mcu_gpio_output_mode_t mode;
} smtc_hal_mcu_gpio_output_cfg_t;

/**
 * @brief GPIO input configuration structure
 */
typedef struct smtc_hal_mcu_gpio_input_cfg_s
{
    smtc_hal_mcu_gpio_pull_mode_t pull_mode;
    smtc_hal_mcu_gpio_irq_mode_t  irq_mode;
    void ( *callback )( void* context );
    void* context;
} smtc_hal_mcu_gpio_input_cfg_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * @brief Initialize a GPIO as output
 *
 * @param [in] cfg Implementation-level configuration structure
 * @param [in] output_cfg Application-level configuration structure
 * @param [out] inst Pointer to a GPIO instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The initialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the GPIO is not initialised
 */
smtc_hal_mcu_status_t smtc_hal_mcu_gpio_init_output( smtc_hal_mcu_gpio_cfg_t               cfg,
                                                     const smtc_hal_mcu_gpio_output_cfg_t* output_cfg,
                                                     smtc_hal_mcu_gpio_inst_t*             inst );

/**
 * @brief Initialize a GPIO as input
 *
 * @param [in] cfg Implementation-level configuration structure
 * @param [in] input_cfg Application-level configuration structure
 * @param [out] inst Pointer to a GPIO instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The initialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the GPIO is not initialised
 */
smtc_hal_mcu_status_t smtc_hal_mcu_gpio_init_input( smtc_hal_mcu_gpio_cfg_t              cfg,
                                                    const smtc_hal_mcu_gpio_input_cfg_t* input_cfg,
                                                    smtc_hal_mcu_gpio_inst_t*            inst );

/**
 * @brief De-initialize a GPIO
 *
 * @param [out] inst Pointer to a GPIO instance - if calls return with SMTC_HAL_MCU_STATUS_OK, \p inst is set to NULL
 *
 * @retval SMTC_HAL_MCU_STATUS_OK The deinitialisation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the GPIO is not deinitialized
 */
smtc_hal_mcu_status_t smtc_hal_mcu_gpio_deinit( smtc_hal_mcu_gpio_inst_t* inst );

/**
 * @brief Set the state of a GPIO
 *
 * @param [in] inst GPIO instance
 * @param [in] state State to be applied to \p inst
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Operation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT \p inst is not initialized
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the GPIO state cannot be set
 */
smtc_hal_mcu_status_t smtc_hal_mcu_gpio_set_state( smtc_hal_mcu_gpio_inst_t inst, smtc_hal_mcu_gpio_state_t state );

/**
 * @brief Get the state of a GPIO
 *
 * @param [in] inst GPIO instance
 * @param [out] state State read from \p inst
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Operation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT \p inst is not initialized
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the GPIO state cannot be read
 */
smtc_hal_mcu_status_t smtc_hal_mcu_gpio_get_state( smtc_hal_mcu_gpio_inst_t inst, smtc_hal_mcu_gpio_state_t* state );

/**
 * @brief Enable an interrupt on a GPIO
 *
 * @param [in] inst GPIO instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Operation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT \p inst is not initialized as input
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the interrupt cannot be enabled
 */
smtc_hal_mcu_status_t smtc_hal_mcu_gpio_enable_irq( smtc_hal_mcu_gpio_inst_t inst );

/**
 * @brief Disable an interrupt on a GPIO
 *
 * @param [in] inst GPIO instance
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Operation completed successfully
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 * @retval SMTC_HAL_MCU_STATUS_NOT_INIT \p inst is not initialized as input
 * @retval SMTC_HAL_MCU_STATUS_ERROR Another error occurred and the interrupt cannot be disabled
 */
smtc_hal_mcu_status_t smtc_hal_mcu_gpio_disable_irq( smtc_hal_mcu_gpio_inst_t inst );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_MCU_GPIO_H

/* --- EOF ------------------------------------------------------------------ */
