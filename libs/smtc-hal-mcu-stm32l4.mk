# --- The Clear BSD License ---
# Copyright Semtech Corporation 2022. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Semtech corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
# THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
# NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

C_SOURCES += \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lptim.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pwr.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/src/smtc_hal_mcu_spi_stm32l4.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/src/smtc_hal_mcu_gpio_stm32l4.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/src/smtc_hal_mcu_rng_stm32l4.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/src/smtc_hal_mcu_uart_stm32l4.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/src/smtc_hal_mcu_timer_stm32l4.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/src/smtc_hal_mcu_stm32l4.c \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/system_stm32l4xx.c

ASM_SOURCES +=  \
$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/startup_stm32l476xx.s

C_INCLUDES +=  \
-I$(TOP_DIR)/libs/smtc-hal-mcu/inc \
-I$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/inc \
-I$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/CMSIS/Device/ST/STM32L4xx/Include \
-I$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/CMSIS/Core/Include \
-I$(TOP_DIR)/libs/smtc-hal-mcu-stm32l4/third_party/STM32CubeL4/Drivers/STM32L4xx_HAL_Driver/Inc \

CPU = -mcpu=cortex-m4

FPU = -mfpu=fpv4-sp-d16

FLOAT-ABI = -mfloat-abi=hard

MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

C_DEFS +=  \
-DUSE_FULL_LL_DRIVER \
-DSTM32L476xx \

LDSCRIPT = $(TOP_DIR)/toolchain/gcc/stm32l476rgtx_flash.ld
