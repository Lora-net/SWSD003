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

TARGET_BOARD ?= NUCLEO_L476RG
RADIO_SHIELD ?= SX1261MB2BAS
CUSTOM_XTAL_TRIM ?= no

ifeq ($(CUSTOM_XTAL_TRIM),yes)
C_DEFS += \
    -DCUSTOM_XTAL_TRIM
else ifeq ($(CUSTOM_XTAL_TRIM),no)
else
    $(error Invalid xtal trimming option)
endif

ifeq ($(RADIO_SHIELD), SX1261MB1BAS)
C_DEFS += -DSX1261MB1BAS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/sx126x/src/smtc_shield_sx1261mb1bas.c
else ifeq ($(RADIO_SHIELD), SX1261MB1CAS)
C_DEFS += -DSX1261MB1CAS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/sx126x/src/smtc_shield_sx1261mb1cas.c
else ifeq ($(RADIO_SHIELD), SX1261MB2BAS)
C_DEFS += -DSX1261MB2BAS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/sx126x/src/smtc_shield_sx1261mb2bas.c
else ifeq ($(RADIO_SHIELD), SX1262MB1CAS)
C_DEFS += -DSX1262MB1CAS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/sx126x/src/smtc_shield_sx1262mb1cas.c
else ifeq ($(RADIO_SHIELD), SX1262MB1CBS)
C_DEFS += -DSX1262MB1CBS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/sx126x/src/smtc_shield_sx1262mb1cbs.c
else ifeq ($(RADIO_SHIELD), SX1262MB1DAS)
C_DEFS += -DSX1262MB1DAS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/sx126x/src/smtc_shield_sx1262mb1das.c
else ifeq ($(RADIO_SHIELD), SX1262MB1PAS)
C_DEFS += -DSX1262MB1PAS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/sx126x/src/smtc_shield_sx1262mb1pas.c
else ifeq ($(RADIO_SHIELD), SX1262MB2CAS)
C_DEFS += -DSX1262MB2CAS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/sx126x/src/smtc_shield_sx1262mb2cas.c
else ifeq ($(RADIO_SHIELD), SX1268MB1GAS)
C_DEFS += -DSX1268MB1GAS
C_SOURCES += \
$(TOP_DIR)/libs/smtc-shields/sx126x/src/smtc_shield_sx1268mb1gas.c
else
$(error Unknown radio shield, please select a supported one.)
endif

C_INCLUDES +=  \
-I$(TOP_DIR)/libs/smtc-shields/sx126x/inc \
-I$(TOP_DIR)/libs/smtc-shields/common/inc \

PRINTERS_MAKEFILE = $(TOP_DIR)/sx126x/common/printers/printers.mk
TOOLCHAIN_MAKEFILE = $(TOP_DIR)/toolchain/gcc/toolchain.mk
SMTC_HAL_MCU_MAKEFILE = $(TOP_DIR)/libs/smtc-hal-mcu-stm32l4.mk

include $(TOOLCHAIN_MAKEFILE)
include $(PRINTERS_MAKEFILE)
include $(SMTC_HAL_MCU_MAKEFILE)

C_SOURCES +=  \
$(TOP_DIR)/sx126x/common/apps_common.c \
$(TOP_DIR)/sx126x/common/sx126x_hal.c \
$(TOP_DIR)/common/src/smtc_hal_dbg_trace.c \
$(TOP_DIR)/common/src/common_version.c \
$(TOP_DIR)/common/src/smtc_shield_pinout_mapping.c \
$(TOP_DIR)/common/src/uart_init.c \

C_INCLUDES +=  \
-I$(TOP_DIR)/sx126x/common \
-I$(TOP_DIR)/common/inc

C_SOURCES +=  \
$(TOP_DIR)/sx126x/sx126x_driver/src/sx126x.c \
$(TOP_DIR)/sx126x/sx126x_driver/src/sx126x_lr_fhss.c \
$(TOP_DIR)/sx126x/sx126x_driver/src/lr_fhss_driver/src/lr_fhss_mac.c

C_INCLUDES +=  \
-I$(TOP_DIR)/sx126x/sx126x_driver/src \
-I$(TOP_DIR)/sx126x/sx126x_driver/src/lr_fhss_driver/src
