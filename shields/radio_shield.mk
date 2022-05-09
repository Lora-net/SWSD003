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

define SUPPORTED_SHIELD_BODY
Shield $(RADIO_SHIELD) is not supported.

The supported shields are:
  * LR1110: LR1110MB1DIS / LR1110MB1DJS / LR1110MB1GIS / LR1110MB1GJS
  * LR1120: LR1120MB1DIS / LR1120MB1DJS / LR1120MB1GIS / LR1120MB1GJS

endef

C_SOURCES += $(TOP_DIR)/shields/common/lr11x0mb1xys_board.c

ifeq ($(RADIO_SHIELD), LR1110MB1DIS)
C_SOURCES += \
$(TOP_DIR)/shields/LR1110/LR1110MB1DxS/lr1110mb1dxs_pa_pwr_cfg.c
else ifeq ($(RADIO_SHIELD), LR1110MB1DJS)
C_SOURCES += \
$(TOP_DIR)/shields/LR1110/LR1110MB1DxS/lr1110mb1dxs_pa_pwr_cfg.c
else ifeq ($(RADIO_SHIELD), LR1110MB1GIS)
C_SOURCES += \
$(TOP_DIR)/shields/LR1110/LR1110MB1GxS/lr1110mb1gxs_pa_pwr_cfg.c
else ifeq ($(RADIO_SHIELD), LR1110MB1GJS)
C_SOURCES += \
$(TOP_DIR)/shields/LR1110/LR1110MB1GxS/lr1110mb1gxs_pa_pwr_cfg.c
else ifeq ($(RADIO_SHIELD), LR1120MB1DIS)
C_SOURCES += \
$(TOP_DIR)/shields/LR1120/LR1120MB1DxS/lr1120mb1dxs_pa_pwr_cfg.c
else ifeq ($(RADIO_SHIELD), LR1120MB1DJS)
C_SOURCES += \
$(TOP_DIR)/shields/LR1120/LR1120MB1DxS/lr1120mb1dxs_pa_pwr_cfg.c
else ifeq ($(RADIO_SHIELD), LR1120MB1GIS)
C_SOURCES += \
$(TOP_DIR)/shields/LR1120/LR1120MB1GxS/lr1120mb1gxs_pa_pwr_cfg.c
else ifeq ($(RADIO_SHIELD), LR1120MB1GJS)
C_SOURCES += \
$(TOP_DIR)/shields/LR1120/LR1120MB1GxS/lr1120mb1gxs_pa_pwr_cfg.c
else
$(error $(SUPPORTED_SHIELD_BODY))
endif

C_SOURCES +=  \
$(TOP_DIR)/shields/peripherals/Leds/leds.c\
$(TOP_DIR)/shields/peripherals/lis2de12/lis2de12.c\
$(TOP_DIR)/shields/peripherals/external_supply/external_supply.c\

C_INCLUDES +=  \
-I$(TOP_DIR)/shields/interface/ \
-I$(TOP_DIR)/shields/peripherals/lis2de12/ \
-I$(TOP_DIR)/shields/peripherals/Leds/ \
-I$(TOP_DIR)/shields/peripherals/external_supply/ \

RADIO_MAKEFILE = $(TOP_DIR)/radio/lr11xx.mk

include $(RADIO_MAKEFILE)