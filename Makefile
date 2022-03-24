################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Top-level application make file.
#
################################################################################
# \copyright
# Copyright 2018-2019 Cypress Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Name of application (used to derive name of final linked file).
APPNAME=afr-example-mqtt-ml-gesture-classification

CY_AFR_BUILD=aws_demos

# Root location of AFR directory
CY_AFR_ROOT=../../..

# Build artifact location
CY_BUILD_RELATIVE_LOCATION=$(CY_AFR_ROOT)/build/cy
CY_BUILD_LOCATION=$(abspath $(CY_BUILD_RELATIVE_LOCATION))


################################################################################
# Basic Configuration
################################################################################

# Target board/hardware
TARGET=CY8CKIT_064S0S2_4343W

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC 7.2.1, provided with ModusToolbox IDE
# ARM     -- ARM Compiler (must be installed separately)
# IAR     -- IAR Compiler (must be installed separately)
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug   -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
CONFIG=Debug

# If set to "true" or "1", display full command-lines when building.
VERBOSE=

# Neural Network Configuration. Options include
#
# float    -- floating point for the input data and weights
# int8x8   -- 8-bit fixed-point for the input data and weights
# int16x8  -- 16-bit fixed-point for the input data and 8-bit for weights
# int16x16 -- 16-bit fixed-point for the input data and weights
NN_TYPE=float

# Model Name to be loaded to the firmware
NN_MODEL_NAME="MAGIC_WAND"

# Folder name containing the model and regression data
NN_MODEL_FOLDER=./mtb_ml_gen

# Shield used to gather IMU data
#
# CY_028_TFT_SHIELD    -- Using the 028-TFT shield
SHIELD_DATA_COLLECTION=CY_028_TFT_SHIELD

################################################################################
# Feature Configuration
################################################################################
# Enable or disable BLE module
BLE_SUPPORTED=1

# Set to 1 to add OTA defines, sources, and libraries (must be used with MCUBoot)
# NOTE: Extra code must be called from your app to initialize AFR OTA Agent
OTA_SUPPORT=0

# This platform always uses EXTERNAL_FLASH
OTA_USE_EXTERNAL_FLASH:=1

# Define CY_TEST_APP_VERSION_IN_TAR here to test application version
#        in TAR archive at start of OTA image download.
# NOTE: This requires that the version numbers here and in the header file match.
# NOTE: This will create compile warnings such as
#		'warning: "APP_VERSION_MAJOR" redefined'
#
# CY_TEST_APP_VERSION_IN_TAR=1
#
# APP_VERSION_MAJOR:=1
# APP_VERSION_MINOR:=0
# APP_VERSION_BUILD:=0

# CY_TFM_PSA_SUPPORTED feature cannot be disabled on this platform.
CY_TFM_PSA_SUPPORTED=1

# Using new Bootloader with SWAP / STATUS
CY_MCUBOOT_SWAP_USING_STATUS=1

################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS=

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=$(NN_MODEL_FOLDER)/mtb_ml_regression_data $(NN_MODEL_FOLDER)/mtb_ml_models source

# Add additional defines to the build process (without a leading -D).
DEFINES=MODEL_NAME=$(NN_MODEL_NAME)

# Depending which Neural Network Type, add a specific DEFINE and COMPONENT
ifeq (float, $(NN_TYPE))
DEFINES+=CY_ML_FLOATING_POINT_fltxflt_NN=1
COMPONENTS+=ML_FLOAT32
endif
ifeq (int16x16, $(NN_TYPE))
DEFINES+=CY_ML_FIXED_POINT_16_IN=1 CY_ML_FIXED_POINT_16_NN=1 
COMPONENTS+=ML_INT16x16
endif
ifeq (int16x8, $(NN_TYPE))
DEFINES+=CY_ML_FIXED_POINT_16_IN=1 CY_ML_FIXED_POINT_8_NN=1 
COMPONENTS+=ML_INT16x8
endif
ifeq (int8x8, $(NN_TYPE))
DEFINES+=CY_ML_FIXED_POINT_8_IN=1 CY_ML_FIXED_POINT_8_NN=1
COMPONENTS+=ML_INT8x8
endif

# Depending which shield is used for data collection, add specific DEFINE
ifeq (CY_028_TFT_SHIELD, $(SHIELD_DATA_COLLECTION))
DEFINES+=CY_BMI_160_IMU=1
endif

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
# TODO remove this once whd is update to new version
INCLUDES+=$(CY_AFR_ROOT)/vendors/cypress/MTB/libraries/wifi-host-driver/WiFi_Host_Driver/resources/nvram/TARGET_CY8CKIT_064B0S2_4343W

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT=hardfp

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CFLAGS=

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
LDFLAGS=

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
LINKER_SCRIPT=

# Set the right linker script based on toolchain chosen
ifeq ($(TOOLCHAIN), GCC_ARM)
LINKER_FILENAME=cyb06xxa_cm4_dual.ld
endif
ifeq ($(TOOLCHAIN), ARM)
$(error The code example supports only GCC_ARM toolchain)
endif
ifeq ($(TOOLCHAIN), IAR)
$(error The code example supports only GCC_ARM toolchain)
endif

# Custom pre-build commands to run.
PREBUILD=cp ./custom_linker_scripts/${LINKER_FILENAME} $(CY_AFR_ROOT)/vendors/cypress/boards/$(TARGET)/aws_demos/application_code/cy_code/COMPONENT_CM4/TOOLCHAIN_$(TOOLCHAIN)/${LINKER_FILENAME};
PREBUILD+=rm -rf $(CY_AFR_ROOT)/vendors/cypress/boards/$(TARGET)/aws_demos/application_code/main.c;
PREBUILD+=cp ./source/arm_math.h $(CY_AFR_ROOT)/vendors/cypress/MTB/psoc6/psoc6pdl/cmsis/include/arm_math.h

################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the "base" library. It provides the core makefile build
# infrastructure.
CY_BASELIB_PATH=$(CY_AFR_ROOT)/vendors/cypress/MTB/psoc6/psoc6make

# Absolute path to the compiler's "bin" directory.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# IDE provided compiler by default).
CY_COMPILER_PATH=

# Include aFR configuration make file
include $(CY_AFR_ROOT)/projects/cypress/make_support/afr.mk

################################################################################
# Tools path
################################################################################

# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

$(info Tools Directory: $(CY_TOOLS_DIR))
include $(CY_TOOLS_DIR)/make/start.mk
