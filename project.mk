###############################################################################
 #
 # Copyright (C) 2024 Analog Devices, Inc.
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
 #
 ##############################################################################
# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analogdevicesinc.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!
# This file can be used to set build configuration
# variables.  These variables are defined in a file called 
# "Makefile" that is located next to this one.

# For instructions on how to use this system, see
# https://analog-devices-msdk.github.io/msdk/USERGUIDE/#build-system

# **********************************************************

# Add your config here!
# Add your config here!
ifneq ($(BOARD),FTHR_Apps_P1)
$(error ERR_NOTSUPPORTED: This project requires an SD card slot and is only supported for the MAX32655FTHR)
endif

# Enable SDHC library
LIB_SDHC = 1
# Use FatFS version R0.15
FATFS_VERSION = ff15

# Enable CLI library
LIB_CLI = 1

# Enable Cordio library
LIB_CORDIO = 1

# Cordio library options
INIT_PERIPHERAL = 1
INIT_CENTRAL = 0

# TRACE option
# Set to 0 to disable
# Set to 1 to enable serial port trace messages
# Set to 2 to enable verbose messages
TRACE = 1

# Add services directory to build
IPATH += services
VPATH += services

# SET advertising name
ADV_NAME?=DATS
PROJ_CFLAGS += -DADV_NAME=\"$(ADV_NAME)\"


### CONFIGURE security
# /*! TRUE to initiate security upon connection*/
PROJ_CFLAGS += -DINIT_SECURITY=FALSE