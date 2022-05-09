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

# Determine TARGET_MCU based on TARGET_BOARD
ifeq ($(TARGET_BOARD), NUCLEO_L476RG)
TARGET_MCU ?= STM32L476xx
else
$(error Invalid target board, please select a supported target board)
endif

# Determine the MCU mk file to include based on TARGET_MCU
ifeq ($(TARGET_MCU),STM32L476xx)
SMTC_HAL_MAKEFILE = $(TOP_DIR)/smtc_hal/STMicroelectronics/STM32L4xx/smtc_hal_l4.mk
else
$(error Invalid target MCU, please select a supported target MCU)
endif

include $(SMTC_HAL_MAKEFILE)

# include the target MCU mk file
TARGET_MAKEFILE = $(TOP_DIR)/host_driver/target.mk
include $(TARGET_MAKEFILE)

# At this point both TARGET_BOARD and TARGET_MCU are good. So we had their value to the defined preprocessor tokens

C_DEFS +=  \
-D$(TARGET_BOARD) \
-D$(TARGET_MCU)

C_INCLUDES +=  \
-I$(TOP_DIR)/smtc_hal/inc/ \
-I$(TOP_DIR)/smtc_hal/board/

# Add the macro debug trace definition
ifeq ($(APP_TRACE),yes)
C_DEFS += \
    -DHAL_DBG_TRACE=1
endif

ifeq ($(APP_TRACE),no)
C_DEFS += \
    -DHAL_DBG_TRACE=0
endif