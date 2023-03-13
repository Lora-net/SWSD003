/*!
 * @file      smtc_hal_mcu_gpio_stm32l4.c
 *
 * @brief      Implementation of GPIO module on top of STM32L4 Low Level drivers
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
#include "smtc_hal_mcu_gpio.h"
#include "smtc_hal_mcu_gpio_stm32l4.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_system.h"
#include <stddef.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Maximum number of GPIO configured as input with interruption
 */
#ifndef SMTC_HAL_MCU_GPIO_STM32L4_ARRAY_SIZE
#define SMTC_HAL_MCU_GPIO_STM32L4_ARRAY_SIZE 16
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct
{
    uint32_t syscfg_exti_port;
    uint32_t syscfg_exti_line;
    uint32_t exti_line;
    int      irq_number;
} smtc_hal_mcu_gpio_irq_exti_cfg_t;

typedef struct smtc_hal_mcu_gpio_irq_cfg_s
{
    bool                             is_irq_enabled;
    smtc_hal_mcu_gpio_input_cfg_t    input_cfg;
    smtc_hal_mcu_gpio_irq_exti_cfg_t exti_cfg;
} smtc_hal_mcu_gpio_irq_cfg_t;

struct smtc_hal_mcu_gpio_inst_s
{
    bool                        is_cfged;
    GPIO_TypeDef*               port;
    uint32_t                    pin;
    bool                        is_irq_cfged;
    smtc_hal_mcu_gpio_irq_cfg_t irq_cfg;
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/**
 * @brief Array to store the configuration of initialized GPIO with IRQ
 */
static struct smtc_hal_mcu_gpio_inst_s gpio_inst_array[SMTC_HAL_MCU_GPIO_STM32L4_ARRAY_SIZE];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Check if a GPIO is configured
 *
 * @param [in] cfg GPIO configuration
 *
 * @retval true GPIO configured
 * @retval false GPIO not configured
 */
static bool smtc_hal_mcu_gpio_stm32l4_is_configured( smtc_hal_mcu_gpio_cfg_t cfg );

/**
 * @brief Get a pointer to the first free slot
 *
 * @retval Pointer to the first free slot - NULL if there is no space left
 */
static struct smtc_hal_mcu_gpio_inst_s* smtc_hal_mcu_gpio_stm32l4_get_free_slot( void );

/**
 * @brief Check if the instance given as parameter is genuine
 *
 * @param [in] inst GPIO instance
 *
 * @retval true Instance is genuine
 * @retval false Instance is not genuine
 */
static bool smtc_hal_mcu_gpio_stm32l4_is_real_inst( smtc_hal_mcu_gpio_inst_t inst );

/**
 * @brief Enable peripheral clock for a given GPIO port
 *
 * @param [in] port GPIO port
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Clock successfully enabled
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 */
static smtc_hal_mcu_status_t smtc_hal_mcu_gpio_stm32l4_enable_clock( GPIO_TypeDef* port );

/**
 * @brief Disable peripheral clock for a given GPIO port
 *
 * @param [in] port GPIO port
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Clock successfully disabled
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 */
static smtc_hal_mcu_status_t smtc_hal_mcu_gpio_stm32l4_disable_clock( GPIO_TypeDef* port );

/**
 * @brief Enable peripheral clock for a given GPIO port
 *
 * @param [in] port GPIO port
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Clock successfully enabled
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 */
static bool smtc_hal_mcu_gpio_stm32l4_is_clock_enabled( GPIO_TypeDef* port );

/**
 * @brief Get the EXTI configuration for the given instance
 *
 * @param [in] inst GPIO instance
 * @param [out] exti_cfg EXTI configuration
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Successfully obtained the EXTI configuration
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 */
static smtc_hal_mcu_status_t smtc_hal_mcu_gpio_stm32l4_get_exti_cfg( smtc_hal_mcu_gpio_inst_t          inst,
                                                                     smtc_hal_mcu_gpio_irq_exti_cfg_t* exti_cfg );

/**
 * @brief Get the trigger mode for a given HAL MCU GPIO interrupt mode
 *
 * @param [in] mode HAL MCU GPIO interrupt mode
 * @param [out] trigger STM32L4xx trigger
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Successfully obtained trigger mode
 * @retval SMTC_HAL_MCU_STATUS_BAD_PARAMETERS At least one parameter has an incorrect value
 */
static smtc_hal_mcu_status_t smtc_hal_mcu_gpio_stm32l4_get_trigger( smtc_hal_mcu_gpio_irq_mode_t mode,
                                                                    uint32_t*                    trigger );

/**
 * @brief Wrapper that calls the callback function registered on pin \ref pin
 *
 * @param [in] pin GPIO pin
 *
 * @retval SMTC_HAL_MCU_STATUS_OK Successfully called a callback
 * @retval SMTC_HAL_MCU_STATUS_ERROR Did not call any callbacks
 */
static smtc_hal_mcu_status_t smtc_hal_mcu_gpio_stm32l4_call_exti_callback( uint32_t pin );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

smtc_hal_mcu_status_t smtc_hal_mcu_gpio_init_output( smtc_hal_mcu_gpio_cfg_t               cfg,
                                                     const smtc_hal_mcu_gpio_output_cfg_t* output_cfg,
                                                     smtc_hal_mcu_gpio_inst_t*             inst )
{
    smtc_hal_mcu_status_t status = SMTC_HAL_MCU_STATUS_ERROR;

    if( smtc_hal_mcu_gpio_stm32l4_is_configured( cfg ) == true )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    struct smtc_hal_mcu_gpio_inst_s* gpio_cfg_slot = smtc_hal_mcu_gpio_stm32l4_get_free_slot( );

    if( gpio_cfg_slot == NULL )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    gpio_cfg_slot->is_cfged     = false;
    gpio_cfg_slot->is_irq_cfged = false;
    gpio_cfg_slot->port         = cfg->port;
    gpio_cfg_slot->pin          = cfg->pin;

    LL_GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin        = gpio_cfg_slot->pin,
        .Mode       = LL_GPIO_MODE_OUTPUT,
        .Speed      = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull       = LL_GPIO_PULL_NO,
    };

    status = smtc_hal_mcu_gpio_stm32l4_enable_clock( gpio_cfg_slot->port );
    if( status != SMTC_HAL_MCU_STATUS_OK )
    {
        return status;
    }

    status = smtc_hal_mcu_gpio_set_state( gpio_cfg_slot, output_cfg->initial_state );
    if( status != SMTC_HAL_MCU_STATUS_OK )
    {
        return status;
    }

    if( LL_GPIO_Init( gpio_cfg_slot->port, &GPIO_InitStruct ) != SUCCESS )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    gpio_cfg_slot->is_cfged = true;

    *inst = gpio_cfg_slot;

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_gpio_init_input( smtc_hal_mcu_gpio_cfg_t              cfg,
                                                    const smtc_hal_mcu_gpio_input_cfg_t* input_cfg,
                                                    smtc_hal_mcu_gpio_inst_t*            inst )
{
    smtc_hal_mcu_status_t status = SMTC_HAL_MCU_STATUS_ERROR;

    if( smtc_hal_mcu_gpio_stm32l4_is_configured( cfg ) == true )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    struct smtc_hal_mcu_gpio_inst_s* gpio_cfg_slot = smtc_hal_mcu_gpio_stm32l4_get_free_slot( );

    if( gpio_cfg_slot == NULL )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    gpio_cfg_slot->is_cfged     = false;
    gpio_cfg_slot->is_irq_cfged = false;
    gpio_cfg_slot->port         = cfg->port;
    gpio_cfg_slot->pin          = cfg->pin;

    LL_GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin        = gpio_cfg_slot->pin,
        .Mode       = LL_GPIO_MODE_INPUT,
        .Speed      = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
        .Pull       = LL_GPIO_PULL_NO,
    };

    status = smtc_hal_mcu_gpio_stm32l4_enable_clock( gpio_cfg_slot->port );
    if( status != SMTC_HAL_MCU_STATUS_OK )
    {
        return status;
    }

    if( LL_GPIO_Init( gpio_cfg_slot->port, &GPIO_InitStruct ) != SUCCESS )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    if( input_cfg->irq_mode != SMTC_HAL_MCU_GPIO_IRQ_MODE_OFF )
    {
        uint32_t trigger;

        smtc_hal_mcu_gpio_irq_exti_cfg_t exti_cfg;

        status = smtc_hal_mcu_gpio_stm32l4_get_exti_cfg( gpio_cfg_slot, &exti_cfg );
        if( status != SMTC_HAL_MCU_STATUS_OK )
        {
            return status;
        }

        status = smtc_hal_mcu_gpio_stm32l4_get_trigger( input_cfg->irq_mode, &trigger );
        if( status != SMTC_HAL_MCU_STATUS_OK )
        {
            return status;
        }

        gpio_cfg_slot->irq_cfg.input_cfg      = *input_cfg;
        gpio_cfg_slot->irq_cfg.is_irq_enabled = false;
        gpio_cfg_slot->irq_cfg.exti_cfg       = exti_cfg;

        LL_EXTI_InitTypeDef EXTI_InitStruct = {
            .Line_0_31   = gpio_cfg_slot->irq_cfg.exti_cfg.exti_line,
            .Line_32_63  = LL_EXTI_LINE_NONE,
            .LineCommand = ENABLE,
            .Mode        = LL_EXTI_MODE_IT,
            .Trigger     = trigger,
        };

        LL_EXTI_Init( &EXTI_InitStruct );

        gpio_cfg_slot->is_irq_cfged = true;
    }

    gpio_cfg_slot->is_cfged = true;

    *inst = gpio_cfg_slot;

    return status;
}

smtc_hal_mcu_status_t smtc_hal_mcu_gpio_deinit( smtc_hal_mcu_gpio_inst_t* inst )
{
    smtc_hal_mcu_gpio_inst_t inst_local = *inst;

    if( smtc_hal_mcu_gpio_stm32l4_is_real_inst( inst_local ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst_local->is_cfged == false )
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    const bool is_clock_enabled = smtc_hal_mcu_gpio_stm32l4_is_clock_enabled( inst_local->port );

    smtc_hal_mcu_gpio_stm32l4_enable_clock( inst_local->port );

    LL_GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin        = inst_local->pin,
        .Mode       = LL_GPIO_MODE_ANALOG,
        .Speed      = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
        .Pull       = LL_GPIO_PULL_NO,
    };

    if( LL_GPIO_Init( inst_local->port, &GPIO_InitStruct ) != SUCCESS )
    {
        return SMTC_HAL_MCU_STATUS_ERROR;
    }

    inst_local->is_cfged = false;

    if( inst_local->is_irq_cfged == true )
    {
        if( inst_local->irq_cfg.is_irq_enabled == true )
        {
            smtc_hal_mcu_gpio_disable_irq( inst_local );

            LL_EXTI_InitTypeDef EXTI_InitStruct;

            LL_EXTI_StructInit( &EXTI_InitStruct );

            EXTI_InitStruct.Line_0_31 = inst_local->irq_cfg.exti_cfg.exti_line;

            LL_EXTI_Init( &EXTI_InitStruct );
        }

        inst_local->is_irq_cfged = false;
    }

    *inst = NULL;

    if( is_clock_enabled == false )
    {
        smtc_hal_mcu_gpio_stm32l4_disable_clock( inst_local->port );
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_gpio_set_state( smtc_hal_mcu_gpio_inst_t inst, smtc_hal_mcu_gpio_state_t state )
{
    if( smtc_hal_mcu_gpio_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    switch( state )
    {
    case SMTC_HAL_MCU_GPIO_STATE_HIGH:
    {
        LL_GPIO_SetOutputPin( inst->port, inst->pin );
        break;
    }
    case SMTC_HAL_MCU_GPIO_STATE_LOW:
    {
        LL_GPIO_ResetOutputPin( inst->port, inst->pin );
        break;
    }
    default:
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_gpio_get_state( smtc_hal_mcu_gpio_inst_t inst, smtc_hal_mcu_gpio_state_t* state )
{
    if( smtc_hal_mcu_gpio_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    const uint32_t pin_mode = LL_GPIO_GetPinMode( inst->port, inst->pin );

    if( pin_mode == LL_GPIO_MODE_OUTPUT )
    {
        *state = ( LL_GPIO_IsOutputPinSet( inst->port, inst->pin ) == 1 ) ? SMTC_HAL_MCU_GPIO_STATE_HIGH
                                                                          : SMTC_HAL_MCU_GPIO_STATE_LOW;
    }
    else if( pin_mode == LL_GPIO_MODE_INPUT )
    {
        *state = ( LL_GPIO_IsInputPinSet( inst->port, inst->pin ) == 1 ) ? SMTC_HAL_MCU_GPIO_STATE_HIGH
                                                                         : SMTC_HAL_MCU_GPIO_STATE_LOW;
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_NOT_INIT;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

smtc_hal_mcu_status_t smtc_hal_mcu_gpio_enable_irq( smtc_hal_mcu_gpio_inst_t inst )
{
    if( smtc_hal_mcu_gpio_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_irq_cfged == true )
    {
        if( inst->irq_cfg.is_irq_enabled == false )
        {
            LL_SYSCFG_SetEXTISource( inst->irq_cfg.exti_cfg.syscfg_exti_port, inst->irq_cfg.exti_cfg.syscfg_exti_line );
            NVIC_EnableIRQ( inst->irq_cfg.exti_cfg.irq_number );
            NVIC_SetPriority( inst->irq_cfg.exti_cfg.irq_number, 0 );

            inst->irq_cfg.is_irq_enabled = true;

            return SMTC_HAL_MCU_STATUS_OK;
        }
    }

    return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
}

smtc_hal_mcu_status_t smtc_hal_mcu_gpio_disable_irq( smtc_hal_mcu_gpio_inst_t inst )
{
    if( smtc_hal_mcu_gpio_stm32l4_is_real_inst( inst ) == false )
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->is_irq_cfged == true )
    {
        if( inst->irq_cfg.is_irq_enabled == true )
        {
            NVIC_DisableIRQ( inst->irq_cfg.exti_cfg.irq_number );

            inst->irq_cfg.is_irq_enabled = false;
        }

        return SMTC_HAL_MCU_STATUS_OK;
    }

    return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool smtc_hal_mcu_gpio_stm32l4_is_configured( smtc_hal_mcu_gpio_cfg_t cfg )
{
    for( int i = 0; i < SMTC_HAL_MCU_GPIO_STM32L4_ARRAY_SIZE; i++ )
    {
        if( gpio_inst_array[i].is_cfged == true )
        {
            if( ( gpio_inst_array[i].port == cfg->port ) && ( gpio_inst_array[i].pin == cfg->pin ) )
            {
                return true;
            }
        }
    }

    return false;
}

static struct smtc_hal_mcu_gpio_inst_s* smtc_hal_mcu_gpio_stm32l4_get_free_slot( void )
{
    for( int i = 0; i < SMTC_HAL_MCU_GPIO_STM32L4_ARRAY_SIZE; i++ )
    {
        if( gpio_inst_array[i].is_cfged == false )
        {
            return &gpio_inst_array[i];
        }
    }

    return NULL;
}

static bool smtc_hal_mcu_gpio_stm32l4_is_real_inst( smtc_hal_mcu_gpio_inst_t inst )
{
    for( int i = 0; i < SMTC_HAL_MCU_GPIO_STM32L4_ARRAY_SIZE; i++ )
    {
        if( inst == &gpio_inst_array[i] )
        {
            return true;
        }
    }

    return false;
}

static smtc_hal_mcu_status_t smtc_hal_mcu_gpio_stm32l4_enable_clock( GPIO_TypeDef* port )
{
    if( port == GPIOA )
    {
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOA );
    }
    else if( port == GPIOB )
    {
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOB );
    }
    else if( port == GPIOC )
    {
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOC );
    }
    else if( port == GPIOD )
    {
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOD );
    }
#if defined( GPIOE )
    else if( port == GPIOE )
    {
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOE );
    }
#endif
#if defined( GPIOF )
    else if( port == GPIOF )
    {
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOF );
    }
#endif
#if defined( GPIOG )
    else if( port == GPIOG )
    {
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOG );
    }
#endif
#if defined( GPIOH )
    else if( port == GPIOH )
    {
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOH );
    }
#endif
#if defined( GPIOI )
    else if( port == GPIOI )
    {
        LL_AHB2_GRP1_EnableClock( LL_AHB2_GRP1_PERIPH_GPIOI );
    }
#endif
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

static smtc_hal_mcu_status_t smtc_hal_mcu_gpio_stm32l4_disable_clock( GPIO_TypeDef* port )
{
    if( port == GPIOA )
    {
        LL_AHB2_GRP1_DisableClock( LL_AHB2_GRP1_PERIPH_GPIOA );
    }
    else if( port == GPIOB )
    {
        LL_AHB2_GRP1_DisableClock( LL_AHB2_GRP1_PERIPH_GPIOB );
    }
    else if( port == GPIOC )
    {
        LL_AHB2_GRP1_DisableClock( LL_AHB2_GRP1_PERIPH_GPIOC );
    }
    else if( port == GPIOD )
    {
        LL_AHB2_GRP1_DisableClock( LL_AHB2_GRP1_PERIPH_GPIOD );
    }
#if defined( GPIOE )
    else if( port == GPIOE )
    {
        LL_AHB2_GRP1_DisableClock( LL_AHB2_GRP1_PERIPH_GPIOE );
    }
#endif
#if defined( GPIOF )
    else if( port == GPIOF )
    {
        LL_AHB2_GRP1_DisableClock( LL_AHB2_GRP1_PERIPH_GPIOF );
    }
#endif
#if defined( GPIOG )
    else if( port == GPIOG )
    {
        LL_AHB2_GRP1_DisableClock( LL_AHB2_GRP1_PERIPH_GPIOG );
    }
#endif
#if defined( GPIOH )
    else if( port == GPIOH )
    {
        LL_AHB2_GRP1_DisableClock( LL_AHB2_GRP1_PERIPH_GPIOH );
    }
#endif
#if defined( GPIOI )
    else if( port == GPIOI )
    {
        LL_AHB2_GRP1_DisableClock( LL_AHB2_GRP1_PERIPH_GPIOI );
    }
#endif
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

static bool smtc_hal_mcu_gpio_stm32l4_is_clock_enabled( GPIO_TypeDef* port )
{
    uint32_t is_enabled_int;

    if( port == GPIOA )
    {
        is_enabled_int = LL_AHB2_GRP1_IsEnabledClock( LL_AHB2_GRP1_PERIPH_GPIOA );
    }
    else if( port == GPIOB )
    {
        is_enabled_int = LL_AHB2_GRP1_IsEnabledClock( LL_AHB2_GRP1_PERIPH_GPIOB );
    }
    else if( port == GPIOC )
    {
        is_enabled_int = LL_AHB2_GRP1_IsEnabledClock( LL_AHB2_GRP1_PERIPH_GPIOC );
    }
    else if( port == GPIOD )
    {
        is_enabled_int = LL_AHB2_GRP1_IsEnabledClock( LL_AHB2_GRP1_PERIPH_GPIOD );
    }
#if defined( GPIOE )
    else if( port == GPIOE )
    {
        is_enabled_int = LL_AHB2_GRP1_IsEnabledClock( LL_AHB2_GRP1_PERIPH_GPIOE );
    }
#endif
#if defined( GPIOF )
    else if( port == GPIOF )
    {
        is_enabled_int = LL_AHB2_GRP1_IsEnabledClock( LL_AHB2_GRP1_PERIPH_GPIOF );
    }
#endif
#if defined( GPIOG )
    else if( port == GPIOG )
    {
        is_enabled_int = LL_AHB2_GRP1_IsEnabledClock( LL_AHB2_GRP1_PERIPH_GPIOG );
    }
#endif
#if defined( GPIOH )
    else if( port == GPIOH )
    {
        is_enabled_int = LL_AHB2_GRP1_IsEnabledClock( LL_AHB2_GRP1_PERIPH_GPIOH );
    }
#endif
#if defined( GPIOI )
    else if( port == GPIOI )
    {
        is_enabled_int = LL_AHB2_GRP1_IsEnabledClock( LL_AHB2_GRP1_PERIPH_GPIOI );
    }
#endif
    else
    {
        return false;
    }

    return ( is_enabled_int != 0 ) ? true : false;
}

static smtc_hal_mcu_status_t smtc_hal_mcu_gpio_stm32l4_get_exti_cfg( smtc_hal_mcu_gpio_inst_t          inst,
                                                                     smtc_hal_mcu_gpio_irq_exti_cfg_t* exti_cfg )
{
    if( inst->pin == LL_GPIO_PIN_0 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_0;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE0;
        exti_cfg->irq_number       = EXTI0_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_1 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_1;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE1;
        exti_cfg->irq_number       = EXTI1_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_2 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_2;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE2;
        exti_cfg->irq_number       = EXTI2_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_3 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_3;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE3;
        exti_cfg->irq_number       = EXTI3_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_4 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_4;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE4;
        exti_cfg->irq_number       = EXTI4_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_5 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_5;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE5;
        exti_cfg->irq_number       = EXTI9_5_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_6 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_6;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE6;
        exti_cfg->irq_number       = EXTI9_5_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_7 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_7;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE7;
        exti_cfg->irq_number       = EXTI9_5_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_8 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_8;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE8;
        exti_cfg->irq_number       = EXTI9_5_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_9 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_9;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE9;
        exti_cfg->irq_number       = EXTI9_5_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_10 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_10;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE10;
        exti_cfg->irq_number       = EXTI15_10_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_11 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_11;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE11;
        exti_cfg->irq_number       = EXTI15_10_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_12 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_12;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE12;
        exti_cfg->irq_number       = EXTI15_10_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_13 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_13;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE13;
        exti_cfg->irq_number       = EXTI15_10_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_14 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_14;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE14;
        exti_cfg->irq_number       = EXTI15_10_IRQn;
    }
    else if( inst->pin == LL_GPIO_PIN_15 )
    {
        exti_cfg->exti_line        = LL_EXTI_LINE_15;
        exti_cfg->syscfg_exti_line = LL_SYSCFG_EXTI_LINE15;
        exti_cfg->irq_number       = EXTI15_10_IRQn;
    }
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    if( inst->port == GPIOA )
    {
        exti_cfg->syscfg_exti_port = LL_SYSCFG_EXTI_PORTA;
    }
    else if( inst->port == GPIOB )
    {
        exti_cfg->syscfg_exti_port = LL_SYSCFG_EXTI_PORTB;
    }
    else if( inst->port == GPIOC )
    {
        exti_cfg->syscfg_exti_port = LL_SYSCFG_EXTI_PORTC;
    }
    else if( inst->port == GPIOD )
    {
        exti_cfg->syscfg_exti_port = LL_SYSCFG_EXTI_PORTD;
    }
#if defined( GPIOE )
    else if( inst->port == GPIOE )
    {
        exti_cfg->syscfg_exti_port = LL_SYSCFG_EXTI_PORTE;
    }
#endif
#if defined( GPIOF )
    else if( inst->port == GPIOF )
    {
        exti_cfg->syscfg_exti_port = LL_SYSCFG_EXTI_PORTF;
    }
#endif
#if defined( GPIOG )
    else if( inst->port == GPIOG )
    {
        exti_cfg->syscfg_exti_port = LL_SYSCFG_EXTI_PORTG;
    }
#endif
#if defined( GPIOH )
    else if( inst->port == GPIOH )
    {
        exti_cfg->syscfg_exti_port = LL_SYSCFG_EXTI_PORTH;
    }
#endif
#if defined( GPIOI )
    else if( inst->port == GPIOI )
    {
        exti_cfg->syscfg_exti_port = LL_SYSCFG_EXTI_PORTI;
    }
#endif
    else
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

static smtc_hal_mcu_status_t smtc_hal_mcu_gpio_stm32l4_get_trigger( smtc_hal_mcu_gpio_irq_mode_t mode,
                                                                    uint32_t*                    trigger )
{
    switch( mode )
    {
    case SMTC_HAL_MCU_GPIO_IRQ_MODE_OFF:
    {
        *trigger = LL_EXTI_TRIGGER_NONE;
        break;
    }
    case SMTC_HAL_MCU_GPIO_IRQ_MODE_RISING:
    {
        *trigger = LL_EXTI_TRIGGER_RISING;
        break;
    }
    case SMTC_HAL_MCU_GPIO_IRQ_MODE_FALLING:
    {
        *trigger = LL_EXTI_TRIGGER_FALLING;
        break;
    }
    case SMTC_HAL_MCU_GPIO_IRQ_MODE_RISING_FALLING:
    {
        *trigger = LL_EXTI_TRIGGER_RISING_FALLING;
        break;
    }
    default:
    {
        return SMTC_HAL_MCU_STATUS_BAD_PARAMETERS;
    }
    }

    return SMTC_HAL_MCU_STATUS_OK;
}

static smtc_hal_mcu_status_t smtc_hal_mcu_gpio_stm32l4_call_exti_callback( uint32_t pin )
{
    for( uint32_t i = 0; i < SMTC_HAL_MCU_GPIO_STM32L4_ARRAY_SIZE; i++ )
    {
        if( pin == gpio_inst_array[i].pin )
        {
            if( gpio_inst_array[i].irq_cfg.is_irq_enabled == true )
            {
                if( gpio_inst_array[i].irq_cfg.input_cfg.callback != NULL )
                {
                    gpio_inst_array[i].irq_cfg.input_cfg.callback( gpio_inst_array[i].irq_cfg.input_cfg.context );
                    return SMTC_HAL_MCU_STATUS_OK;
                }
            }
        }
    }
    return SMTC_HAL_MCU_STATUS_ERROR;
}

/**
 * @brief This function handles EXTI line0 interrupt.
 */
void EXTI0_IRQHandler( void )
{
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_0 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_0 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_0 );
    }
}

/**
 * @brief This function handles EXTI line1 interrupt.
 */
void EXTI1_IRQHandler( void )
{
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_1 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_1 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_1 );
    }
}

/**
 * @brief This function handles EXTI line2 interrupt.
 */
void EXTI2_IRQHandler( void )
{
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_2 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_2 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_2 );
    }
}

/**
 * @brief This function handles EXTI line3 interrupt.
 */
void EXTI3_IRQHandler( void )
{
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_3 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_3 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_3 );
    }
}

/**
 * @brief This function handles EXTI line4 interrupt.
 */
void EXTI4_IRQHandler( void )
{
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_4 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_4 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_4 );
    }
}

/**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler( void )
{
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_5 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_5 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_5 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_6 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_6 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_6 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_7 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_7 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_7 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_8 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_8 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_8 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_9 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_9 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_9 );
    }
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler( void )
{
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_10 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_10 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_10 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_11 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_11 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_11 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_12 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_12 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_12 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_13 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_13 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_13 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_14 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_14 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_14 );
    }
    if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_15 ) != RESET )
    {
        LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_15 );
        smtc_hal_mcu_gpio_stm32l4_call_exti_callback( LL_GPIO_PIN_15 );
    }
}

/* --- EOF ------------------------------------------------------------------ */
