############################################################################
# Tools/spresense/copter_app/Makefile
#
# SPDX-License-Identifier: GPL-3.0-or-later
############################################################################

include $(APPDIR)/Make.defs
include $(SDKDIR)/Make.defs

MODULE = $(CONFIG_SPRESENSE_M1_COPTER_LINK)
CXXEXT = .cpp

M1_REPO_ROOT := $(abspath $(SDKDIR)/../../..)
M1_COPTER_APP_DIR := $(M1_REPO_ROOT)/Tools/spresense/copter_app
SPRESENSE_HAL_DIR := $(M1_REPO_ROOT)/libraries/AP_HAL_Spresense

VPATH += :$(SPRESENSE_HAL_DIR)

CXXSRCS = HAL_Spresense_Class.cpp
CXXSRCS += RCOutput.cpp
CXXSRCS += SafeBringup.cpp
CXXSRCS += Scheduler.cpp
CXXSRCS += Semaphores.cpp
CXXSRCS += Storage.cpp
CXXSRCS += UARTDriver.cpp
CXXSRCS += system.cpp

CXXFLAGS += ${INCDIR_PREFIX}$(M1_REPO_ROOT)/libraries
CXXFLAGS += ${DEFINE_PREFIX}CONFIG_HAL_BOARD=HAL_BOARD_SPRESENSE
CXXFLAGS += ${DEFINE_PREFIX}AP_SCRIPTING_ENABLED=0
CXXFLAGS += ${DEFINE_PREFIX}AP_SIM_ENABLED=0
CXXFLAGS += ${DEFINE_PREFIX}__AP_LINE__=__LINE__
CXXFLAGS += -include $(M1_COPTER_APP_DIR)/nuttx_compat.h
CXXFLAGS += -std=gnu++17
CXXFLAGS += -fno-exceptions
CXXFLAGS += -fno-rtti
CXXFLAGS += -fno-threadsafe-statics
CXXFLAGS += -Wno-expansion-to-defined

include $(APPDIR)/Application.mk
