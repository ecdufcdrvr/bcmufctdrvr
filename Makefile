#
#  BSD LICENSE
#
#  Copyright (c) 2018 Broadcom.  All Rights Reserved.
#  Copyright (c) Intel Corporation. All Rights Reserved.
#
#  The term "Broadcom" refers to Broadcom Inc. and/or its subsidiaries.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in
#      the documentation and/or other materials provided with the
#      distribution.
#    * Neither the name of Intel Corporation nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

ifneq ($(V),1)
Q ?= @
endif
S ?= $(notdir $(CURDIR))

CONFIG_PREFIX=$(CURDIR)/build/
CONFIG_DEBUG=n

OCS_USPACE_COMMON	= ./src
OCS_USPACE_DRV		= ./src
OCS_FC_COMMON		= ./src
OCS_COMMON		= ./src
OCS_HAL_COMMON		= ./src
OCS_COMMON_INI_STUB	= ./src

CFLAGS += -g -Wall -Werror -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Wmissing-declarations
CFLAGS += -march=native -fPIC -fstack-protector -fno-common
CFLAGS += -Wformat -Wformat-security
CFLAGS += -Wno-pointer-sign -Wstrict-prototypes -Wold-style-definition -std=gnu99
# Disable clang warning: passing an object that undergoes default argument promotion to 'va_start' has undefined behavior [-Wvarargs]
CFLAGS += -Wno-varargs
CFLAGS += -D_GNU_SOURCE -DOCS_USPACE -DOCS_USPACE_SPDK -DOCS_USPACE_SPDK_UPSTREAM

CFLAGS += -I$(OCS_USPACE_COMMON) -I$(OCS_FC_COMMON) -I$(OCS_COMMON) -I$(OCS_HAL_COMMON) -I$(OCS_COMMON_INI_STUB) -I$(OCS_USPACE_DRV)
CFLAGS += -I$(CURDIR)/src -I$(CURDIR)/include -I$(DPDK_DIR)/include -I$(SPDK_DIR)/include -I$(SPDK_DIR)/lib/env_dpdk -I$(SPDK_DIR)/lib/nvmf

SPDK_SRCS = \
	src/ocs_spdk.c \
	src/ocs_ocsu.c \
	src/ocs_driver.c \
	src/ocs_spdk_nvmet.c \
	src/spdk_nvmf_xport.c \
	src/ocs_impl_spdk.c \
	src/ocs_tgt_stub.c 

SPDK_COMMON_C_SRCS = \
	$(OCS_USPACE_COMMON)/ocs_os.c \
	$(OCS_USPACE_COMMON)/crc32.c \
	$(OCS_USPACE_COMMON)/dslab.c \
	$(OCS_USPACE_COMMON)/ocsu_ioctl.c \
	$(OCS_USPACE_COMMON)/ocs_socket.c \
	$(OCS_FC_COMMON)/ocs_mgmt.c \
	$(OCS_FC_COMMON)/ocs_debug.c \
	$(OCS_FC_COMMON)/ocs_domain.c \
	$(OCS_FC_COMMON)/ocs_sport.c \
	$(OCS_FC_COMMON)/ocs_node.c \
	$(OCS_FC_COMMON)/ocs_unsol.c \
	$(OCS_FC_COMMON)/ocs_io.c \
	$(OCS_FC_COMMON)/ocs_els.c \
	$(OCS_FC_COMMON)/ocs_scsi.c \
	$(OCS_FC_COMMON)/ocs_fabric.c \
	$(OCS_FC_COMMON)/ocs_device.c \
	$(OCS_FC_COMMON)/ocs_ddump.c \
	$(OCS_FC_COMMON)/ocs_xport.c \
	$(OCS_FC_COMMON)/ocs_elxu.c \
	$(OCS_FC_COMMON)/ocs_femul.c \
	$(OCS_FC_COMMON)/spv.c \
	$(OCS_FC_COMMON)/ocs_auth.c \
	$(OCS_HAL_COMMON)/sli4.c \
	$(OCS_HAL_COMMON)/sli4_fc.c \
	$(OCS_HAL_COMMON)/ocs_sm.c \
	$(OCS_HAL_COMMON)/ocs_hal.c \
	$(OCS_HAL_COMMON)/ocs_hal_queues.c \
	$(OCS_HAL_COMMON)/ocs_hal_rqpair.c \
	$(OCS_HAL_COMMON)/ocs_hal_workaround.c \
	$(OCS_COMMON)/ocs_textbuf.c \
	$(OCS_COMMON)/ocs_dif.c \
	$(OCS_COMMON)/t10crc16.c \
	$(OCS_COMMON)/ocs_list.c \
	$(OCS_COMMON)/ocs_ramlog.c \
	$(OCS_COMMON)/ocs_mgmt_common.c \
	$(OCS_COMMON)/ocs_ddumplib.c \
	$(OCS_COMMON)/ocs_cbuf.c \
	$(OCS_COMMON)/ocs_array.c \
	$(OCS_COMMON)/ocs_pool.c \
	$(OCS_COMMON)/ocs_ras.c \
	$(OCS_COMMON)/ocs_gendump.c \
	$(OCS_COMMON)/ocs_recovery.c \
	$(OCS_COMMON)/ocs_compat.c \
	$(OCS_COMMON)/ocs_ioctl.c \
	$(OCS_COMMON)/ocs_ioctl_fc.c \
	$(OCS_COMMON)/ocs_recovery.c \
	$(OCS_COMMON_INI_STUB)/ocs_ini_stub.c

SPDK_COMMON_H_SRCS = $(OCS_USPACE_COMMON)/ocs_os.h \
	$(OCS_USPACE_COMMON)/ocsu_ioctl.h \
	$(OCS_USPACE_COMMON)/dslab.h \
	$(OCS_USPACE_COMMON)/ocs_params.h \
	$(OCS_USPACE_COMMON)/crc32.h \
	$(OCS_USPACE_COMMON)/ocs_socket.h \
	$(OCS_COMMON)/ocs_drv_fc.h \
	$(OCS_COMMON)/ocs_ramlog.h \
	$(OCS_COMMON)/ocs_textbuf.h \
	$(OCS_COMMON)/ocs_list.h \
	$(OCS_COMMON)/ocs_pool.h \
	$(OCS_COMMON)/ocs_array.h \
	$(OCS_COMMON)/ocs_cbuf.h \
	$(OCS_COMMON)/ocs_common_shared.h \
	$(OCS_COMMON)/ocs_mgmt.h \
	$(OCS_COMMON)/ocs_scsi.h \
	$(OCS_COMMON)/ocs_ddump.h \
	$(OCS_COMMON)/ocs_scsi_force_free_stub.h \
	$(OCS_COMMON)/ocs_stats.h \
	$(OCS_COMMON)/ocs_ioctl.h \
	$(OCS_COMMON)/ocs_elxu.h \
	$(OCS_COMMON)/ocs_pm.h \
	$(OCS_COMMON)/ocs_gendump.h \
	$(OCS_COMMON)/ocs_recovery.h \
	$(OCS_COMMON)/ocs_vpd.h \
	$(OCS_COMMON)/scsi_cmds.h \
	$(OCS_COMMON)/ocs_dif.h \
	$(OCS_COMMON)/ocs_compat.h \
	$(OCS_COMMON)/ocs_ras.h \
	$(OCS_COMMON)/t10crc16.h \
	$(OCS_COMMON)/ocs_ioctl_xport.h \
	$(OCS_USPACE_DRV)/ocs_uspace.h \
	$(OCS_HAL_COMMON)/ocs_hal.h \
	$(OCS_HAL_COMMON)/sli4.h \
	$(OCS_HAL_COMMON)/sli4_fc.h \
	$(OCS_HAL_COMMON)/ocs_fcp.h \
	$(OCS_HAL_COMMON)/ocs_hal_config.h \
	$(OCS_HAL_COMMON)/ocs_hal_workaround.h \
	$(OCS_HAL_COMMON)/ocs_hal_queues.h \
	$(OCS_FC_COMMON)/ocs_debug.h \
	$(OCS_FC_COMMON)/ocs_common.h \
	$(OCS_FC_COMMON)/spv.h \
	$(OCS_FC_COMMON)/ocs_auth.h \
	$(OCS_FC_COMMON)/ocs_io.h \
	$(OCS_FC_COMMON)/ocs_node.h \
	$(OCS_FC_COMMON)/ocs_unsol.h \
	$(OCS_FC_COMMON)/ocs_xport.h \
	$(OCS_FC_COMMON)/ocs_domain.h \
	$(OCS_FC_COMMON)/ocs_sport.h \
	$(OCS_FC_COMMON)/ocs_els.h \
	$(OCS_FC_COMMON)/ocs_device.h \
	$(OCS_FC_COMMON)/ocs_fc_config.h \
	$(OCS_FC_COMMON)/ocs_fabric.h \
	$(OCS_FC_COMMON)/ocs_scsi_fc.h \
	$(OCS_FC_COMMON)/ocs_nvme_stub.h \
	$(OCS_FC_COMMON)/ocs_femul.h \
	$(OCS_HAL_COMMON)/ocs_sm.h \
	$(OCS_COMMON_INI_STUB)/ocs_scsi_ini.h

C_SRCS =  $(SPDK_SRCS)
C_SRCS += $(SPDK_COMMON_C_SRCS)

LIB := $(CONFIG_PREFIX)libufc.a

DEPFLAGS = -MMD -MP -MF $*.d.tmp

OBJS = $(C_SRCS:.c=.o)

.PHONY: all clean include

all: $(CONFIG_PREFIX) $(LIB) include
	@:

clean:
	$(Q)rm -rf *.a *.o *.d *.d.tmp *.gcno *.gcda $(LIB) $(CONFIG_PREFIX)

$(CONFIG_PREFIX):
	mkdir -p $(CONFIG_PREFIX)

$(LIB): $(OBJS)
	$(Q)echo "  LIB $(notdir $@)"; \
	rm -f $@; \
	ar crDs $@ $(OBJS)

%.o: %.c %.d $(MAKEFILE_LIST)
	$(Q)echo "  CC $S/$@"; \
	cc -o $@ $(DEPFLAGS) $(CFLAGS) -c $< && \
	mv -f $*.d.tmp $*.d && touch -c $@

%.d: ;

include:
	cp include/* $(CONFIG_PREFIX)

.PRECIOUS: $(OBJS)

-include $(OBJS:.o=.d)

