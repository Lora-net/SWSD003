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
# target
######################################
TOP_DIR = ../../../..

APP = per
APP_TRACE ?= yes

PROJECTS_COMMON_MAKEFILE = $(TOP_DIR)/sx126x/common/apps_common.mk

######################################
# building variables
######################################
# debug build?
DEBUG ?= 1
# optimization
OPT ?= -O0

#######################################
# paths
#######################################

# Build path
BUILD_DIR = ./build
OUT_DIR = ./out

######################################
# source
######################################

# C sources

# This file is built differenty because it depends on RECEIVER macro
# Files that does not depends to the RECEIVER macro should be added to C_SOURCES
# variable.
C_MAIN_SOURCE = ../main_$(APP).c

# Initialise empty C_DEFS
C_DEFS =

#######################################
# include
#######################################

include $(PROJECTS_COMMON_MAKEFILE)

#######################################
# build the application
#######################################

.PHONY: all target transmitter_target receiver_target

target: per

all: transmitter_target receiver_target
transmitter_target: $(OUT_DIR)/$(APP)_transmitter.bin
receiver_target: $(OUT_DIR)/$(APP)_receiver.bin
per: $(BUILD_DIR)/$(APP).bin

.DEFAULT_GOAL:= target

## For the main application
# list of objects
COMMON_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.c $(sort $(dir $(C_MAIN_SOURCE)))

# list of ASM program objects
COMMON_OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

.PRECIOUS: $(COMMON_OBJECTS)

OBJECTS_per_transmitter = $(BUILD_DIR)/$(APP)_transmitter.o
OBJECTS_per_receiver = $(BUILD_DIR)/$(APP)_receiver.o
OBJECTS_per = $(BUILD_DIR)/$(APP).o

$(OBJECTS_per_transmitter):  CFLAGS+= -DRECEIVER=0
$(OBJECTS_per_receiver):  CFLAGS+= -DRECEIVER=1
$(OBJECTS_per):  CFLAGS+=

$(BUILD_DIR)/per.o: $(C_MAIN_SOURCE) Makefile $(PLATFORM_MAKEFILE) | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/$(APP)_%.o: $(C_MAIN_SOURCE) Makefile $(PLATFORM_MAKEFILE) | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c Makefile $(PLATFORM_MAKEFILE) | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile $(PLATFORM_MAKEFILE) | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.elf: $(BUILD_DIR)/%.o $(COMMON_OBJECTS)  Makefile | $(BUILD_DIR)
	$(CC) $< $(COMMON_OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(OUT_DIR)/%.bin: $(BUILD_DIR)/%.bin | $(OUT_DIR)
	cp $^ $@

# This empty recipe avoid the rebuild attempt of included dependency files (*.d)
%.d: ;

$(BUILD_DIR):
	mkdir $@

$(OUT_DIR):
	mkdir $@


print-%  : ; @echo $* = $($*)

#######################################
# clean up
#######################################
clean:
	rm -fR $(BUILD_DIR)

clean_all:
	rm -fR $(BUILD_DIR)
	rm -fR $(OUT_DIR)
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
