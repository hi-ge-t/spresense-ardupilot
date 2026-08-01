############################################################################
# Tools/spresense/m1_gcs_app/Makefile
#
# SPDX-License-Identifier: GPL-3.0-or-later
############################################################################

include $(APPDIR)/Make.defs
include $(SDKDIR)/Make.defs

PROGNAME  = $(CONFIG_SPRESENSE_M1_GCS_PROGNAME)
PRIORITY  = $(CONFIG_SPRESENSE_M1_GCS_PRIORITY)
STACKSIZE = $(CONFIG_SPRESENSE_M1_GCS_STACKSIZE)
MODULE    = $(CONFIG_SPRESENSE_M1_GCS)

M1_REPO_ROOT := $(abspath $(SDKDIR)/../../..)

CSRCS = m1_gcs_protocol.c
CSRCS += m1_gcs_memory_layout.c
CSRCS += m1_gcs_memory_rodata.c
CSRCS += m1_gcs_runtime.c
CSRCS += m1_gcs_boot.c
MAINSRC = m1_gcs_main.c

CFLAGS += ${INCDIR_PREFIX}$(M1_REPO_ROOT)/build/spresense-m1-generated
CFLAGS += -Wno-address-of-packed-member

include $(APPDIR)/Application.mk
