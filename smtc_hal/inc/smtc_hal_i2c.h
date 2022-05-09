/*!
 * @file      smtc_hal_i2c.h
 *
 * @brief     Board specific package I2C API definition.
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

#ifndef SMTC_HAL_I2C_H
#define SMTC_HAL_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

#include "smtc_hal.h"

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


/*!
 * @brief I2C peripheral ID
 */
typedef enum
{
    I2C_1,
    I2C_2,
} i2c_id_t;

/*!
 * @brief Operation Mode for the I2C
 */
typedef enum
{
    MODE_I2C = 0,
    MODE_SMBUS_DEVICE,
    MODE_SMBUS_HOST
} i2c_mode;

/*!
 * @brief I2C signal duty cycle
 */
typedef enum
{
    I2C_DUTY_CYCLE_2 = 0,
    I2C_DUTY_CYCLE_16_9
} i_2c_duty_cycle;

/*!
 * @brief I2C select if the acknowledge in after the 7th or 10th bit
 */
typedef enum
{
    I2C_ACK_ADD_7_BIT = 0,
    I2C_ACK_ADD_10_BIT
} i2c_ack_addr_mode;

/*!
 * @brief Internal device address size
 */
typedef enum
{
    I2C_ADDR_SIZE_8 = 0,
    I2C_ADDR_SIZE_16,
} i2c_addr_size;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initializes the I2C object and MCU peripheral
 *
 * @param [in] id   I2C interface id [1:N]
 * @param [in] sda  I2C SDA pin name to be used
 * @param [in] scl  I2C SCL pin name to be used
 */
void hal_i2c_init( const uint32_t id, const hal_gpio_pin_names_t sda, const hal_gpio_pin_names_t scl );

/*!
 * @brief DeInitializes the I2C object and MCU peripheral
 *
 * @param [in] id   I2C interface id [1:N]
 */
void hal_i2c_deinit( const uint32_t id );

/*!
 * @brief Write data to the I2C device
 *
 * @param [in] id           I2C interface id [1:N]
 * @param [in] device_addr  device address
 * @param [in] addr         data address
 * @param [in] data         data to write
 */
uint8_t hal_i2c_write( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t data );

/*!
 * @brief Write data buffer to the I2C device
 *
 * @param [in] id   I2C interface   id [1:N]
 * @param [in] device_addr          device address
 * @param [in] addr                 data address
 * @param [in] buffer               data buffer to write
 * @param [in] size                 number of bytes to write
 */
uint8_t hal_i2c_write_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size );

/*!
 * @brief Read data from the I2C device
 *
 * @param [in] id   I2C interface   id [1:N]
 * @param [in] device_addr          device address
 * @param [in] addr                 data address
 * @param [out] data                data to read
 */
uint8_t hal_i2c_read( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* data );

/*!
 * @brief Read data buffer from the I2C device
 *
 * @param [in] id   I2C interface   id [1:N]
 * @param [in] device_addr          device address
 * @param [in] addr                 data address
 * @param [out] buffer              data buffer to read
 * @param [in] size                 number of data bytes to read
 */
uint8_t hal_i2c_read_buffer( const uint32_t id, uint8_t device_addr, uint16_t addr, uint8_t* buffer, uint16_t size );

/*!
 * @brief Sets the internal device address size
 *
 * @param [in] addr_size Internal address size
 */
void hal_i2c_set_addr_size( i2c_addr_size addr_size );

#ifdef __cplusplus
}
#endif

#endif  // SMTC_HAL_I2C_H
