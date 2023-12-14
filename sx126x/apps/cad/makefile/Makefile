# --- The Clear BSD License ---
# Copyright Semtech Corporation 2021. All rights reserved.
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

APP = cad
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
# Output path
OUT_DIR = ./out

######################################
# source
######################################

# C sources

# This is the main source file, it is build differently from the others
# because it is the only one where the CAD_EXIT_MODE macro is used.
# If any further files need touse this macro, they shall be also part of
# this list
# Files that does not depends to the CAD_EXIT_MODE macro should be added 
# to C_SOURCES variable.

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

.PHONY: all target cad_only cad_then_receive cad_then_transmit

target: cad

all: cad_only cad_then_receive cad_then_transmit

cad: $(BUILD_DIR)/$(APP).bin
cad_only: $(OUT_DIR)/$(APP)_only.bin
cad_then_receive: $(OUT_DIR)/$(APP)_then_receive.bin
cad_then_transmit: $(OUT_DIR)/$(APP)_then_transmit.bin

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

OBJECTS_cad= $(BUILD_DIR)/$(APP).o
OBJECTS_cad_then_receive = $(BUILD_DIR)/$(APP)_then_receive.o
OBJECTS_cad_then_transmit = $(BUILD_DIR)/$(APP)_then_transmit.o
OBJECTS_cad_then_only = $(BUILD_DIR)/$(APP)_only.o

$(OBJECTS_cad_then_receive): CFLAGS+= -DCAD_EXIT_MODE=0x01
$(OBJECTS_cad_then_transmit): CFLAGS+= -DCAD_EXIT_MODE=0x10
$(OBJECTS_cad_then_only): CFLAGS+= -DCAD_EXIT_MODE=0x00
$(OBJECTS_cad): CFLAGS+=

# Recipe to build main sources
$(info )
$(BUILD_DIR)/$(APP)%.o: $(C_MAIN_SOURCE) Makefile $(PLATFORM_MAKEFILE) | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/cad.o: $(C_MAIN_SOURCE) Makefile $(PLATFORM_MAKEFILE) | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# Recipe to build common files to all configurations targets
$(BUILD_DIR)/%.o: %.c Makefile $(PLATFORM_MAKEFILE) | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# Recipe to build assembly files
$(BUILD_DIR)/%.o: %.s Makefile $(PLATFORM_MAKEFILE) | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

# Recipe for linking
$(BUILD_DIR)/%.elf: $(BUILD_DIR)/%.o $(COMMON_OBJECTS) Makefile | $(BUILD_DIR)
	$(CC) $< $(COMMON_OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(OUT_DIR)/%.bin: $(BUILD_DIR)/%.bin | $(OUT_DIR)
	cp $^ $@

$(BUILD_DIR):
	mkdir $@

$(OUT_DIR):
	mkdir $@

# This empty recipe avoid the rebuild attempt of included dependency files (*.d)
%.d: ;

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
