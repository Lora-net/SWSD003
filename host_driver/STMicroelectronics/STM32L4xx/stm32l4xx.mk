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

######################################
# source
######################################

# C sources

C_SOURCES +=  \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rng.c\
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_lptim.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_spi.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_wwdg.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_iwdg.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.c \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.c\
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/system_stm32l4xx.c

ASM_SOURCES +=  \
$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/CMSIS/Device/ST/STM32L4xx/Source/Templates/gcc/startup_stm32l476xx.s

#######################################
# CFLAGS
#######################################

CPU = -mcpu=cortex-m4

FPU = -mfpu=fpv4-sp-d16

FLOAT-ABI = -mfloat-abi=hard

MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

C_DEFS +=  \
-DUSE_FULL_LL_DRIVER \
-DUSE_HAL_DRIVER

C_INCLUDES +=  \
-I$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Inc\
-I$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy\
-I$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/CMSIS/Include \
-I$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/Drivers/CMSIS/Device/ST/STM32L4xx/Include \
-I$(TOP_DIR)/host_driver/STMicroelectronics/STM32L4xx/hal_config

# Determine the linker script to use based on TARGET_MCU
ifeq ($(TARGET_MCU),STM32L476xx)
LDSCRIPT = $(TOP_DIR)/host_driver/STMicroelectronics/gcc/stm32l476rgtx_flash.ld
else
$(error Invalid target, must be STM32L476xx or please modify makefile to add right .ld file)
endif
