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

RADIO_SHIELD ?= LR1110MB1DIS

C_DEFS += -DLR11XX_DISABLE_WARNINGS
C_DEFS += -DLR11XX_DISABLE_HIGH_ACP_WORKAROUND

ifeq ($(RADIO_SHIELD), LR1110MB1DIS)
C_DEFS += -DLR1110MB1DIS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1110mb1dis.c
else ifeq ($(RADIO_SHIELD), LR1110MB1IPDDIS)
C_DEFS += -DLR1110MB1IPDDIS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1110mb1ipddis.c
else ifeq ($(RADIO_SHIELD), LR1110MB1DJS)
C_DEFS += -DLR1110MB1DJS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1110mb1djs.c
else ifeq ($(RADIO_SHIELD), LR1110MB1GIS)
C_DEFS += -DLR1110MB1GIS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1110mb1gis.c
else ifeq ($(RADIO_SHIELD), LR1110MB1PIS)
C_DEFS += -DLR1110MB1PIS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1110mb1pis.c
else ifeq ($(RADIO_SHIELD), LR1110MB1GJS)
C_DEFS += -DLR1110MB1GJS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1110mb1gjs.c
else ifeq ($(RADIO_SHIELD), LR1120MB1DIS)
C_DEFS += -DLR1120MB1DIS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1120mb1dis.c
else ifeq ($(RADIO_SHIELD), LR1120MB1IPDDIS)
C_DEFS += -DLR1120MB1IPDDIS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1120mb1ipddis.c
else ifeq ($(RADIO_SHIELD), LR1120MB1DJS)
C_DEFS += -DLR1120MB1DJS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1120mb1djs.c
else ifeq ($(RADIO_SHIELD), LR1120MB1GIS)
C_DEFS += -DLR1120MB1GIS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1120mb1gis.c
else ifeq ($(RADIO_SHIELD), LR1120MB1PIS)
C_DEFS += -DLR1120MB1PIS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1120mb1pis.c
else ifeq ($(RADIO_SHIELD), LR1120MB1GJS)
C_DEFS += -DLR1120MB1GJS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1120mb1gjs.c
else ifeq ($(RADIO_SHIELD), LR1121MB1DIS)
C_DEFS += -DLR1121MB1DIS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1121mb1dis.c
else ifeq ($(RADIO_SHIELD), LR1121MB1GIS)
C_DEFS += -DLR1121MB1GIS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1121mb1gis.c
else
$(error Unknown radio shield, please select a supported one.)
endif

C_SOURCES +=  \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr11xx_common.c \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr11x0_common.c \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr11x1_common.c \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1110mb1dxs_common.c \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr11x0mb1ipddis_common.c \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr11x0mb1pis_common.c \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1110mb1gxs_common.c \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1120mb1dxs_common.c \
$(TOP_DIR)/libs/smtc-shields/lr11xx/src/smtc_shield_lr1120mb1gxs_common.c \

C_INCLUDES +=  \
-I$(TOP_DIR)/libs/smtc-shields/lr11xx/inc \
-I$(TOP_DIR)/libs/smtc-shields/common/inc \

PRINTERS_MAKEFILE = $(TOP_DIR)/lr11xx/common/printers/printers.mk
TOOLCHAIN_MAKEFILE = $(TOP_DIR)/toolchain/gcc/toolchain.mk
SMTC_HAL_MCU_MAKEFILE = $(TOP_DIR)/libs/smtc-hal-mcu-stm32l4.mk

include $(TOOLCHAIN_MAKEFILE)
include $(PRINTERS_MAKEFILE)
include $(SMTC_HAL_MCU_MAKEFILE)

C_SOURCES +=  \
$(TOP_DIR)/lr11xx/common/apps_common.c \
$(TOP_DIR)/lr11xx/common/lr11xx_hal.c \
$(TOP_DIR)/lr11xx/common/apps_version.c \
$(TOP_DIR)/common/src/smtc_hal_dbg_trace.c \
$(TOP_DIR)/common/src/common_version.c \
$(TOP_DIR)/common/src/smtc_shield_pinout_mapping.c \
$(TOP_DIR)/common/src/uart_init.c \

C_INCLUDES +=  \
-I$(TOP_DIR)/lr11xx/common \
-I$(TOP_DIR)/common/inc

ifneq (,$(findstring LR1110,$(RADIO_SHIELD)))
TRX_IS_GEOLOCATION_CAPABLE = true
else ifneq (,$(findstring LR1120,$(RADIO_SHIELD)))
TRX_IS_GEOLOCATION_CAPABLE = true
else
TRX_IS_GEOLOCATION_CAPABLE = false
endif

C_SOURCES +=  \
$(TOP_DIR)/lr11xx/lr11xx_driver/src/lr11xx_system.c \
$(TOP_DIR)/lr11xx/lr11xx_driver/src/lr11xx_radio.c \
$(TOP_DIR)/lr11xx/lr11xx_driver/src/lr11xx_regmem.c \
$(TOP_DIR)/lr11xx/lr11xx_driver/src/lr11xx_wifi.c \
$(TOP_DIR)/lr11xx/lr11xx_driver/src/lr11xx_gnss.c \
$(TOP_DIR)/lr11xx/lr11xx_driver/src/lr11xx_rttof.c \
$(TOP_DIR)/lr11xx/lr11xx_driver/src/lr11xx_driver_version.c \

C_INCLUDES +=  \
-I$(TOP_DIR)/lr11xx/lr11xx_driver/src \
-I$(TOP_DIR)/libs/smtc_dbpsk_driver/src/ \
