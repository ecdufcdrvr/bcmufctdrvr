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

CFLAGS += -g -Wall -Werror -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Wmissing-declarations
CFLAGS += -march=native -fPIC -fstack-protector -fno-common
CFLAGS += -Wformat -Wformat-security
CFLAGS += -Wno-pointer-sign -Wstrict-prototypes -Wold-style-definition -std=gnu99
# Disable clang warning: passing an object that undergoes default argument promotion to 'va_start' has undefined behavior [-Wvarargs]
CFLAGS += -Wno-varargs
CFLAGS += -D_GNU_SOURCE -DOCS_USPACE_SPDK -DOCS_NVME_FC

CFLAGS += -I$(DPDK_DIR)/include -I$(SPDK_DIR)/include -I$(SPDK_DIR)/lib/env_dpdk -I$(SPDK_DIR)/lib/nvmf

C_SRCS  =  \
	fc_subsystem.c \
	ocs_spdk.c \
	ocs_driver.c \
	ocs_ocsu.c \
	ocs_os.c \
	crc32.c \
	dslab.c \
	ocs_debug.c \
	ocs_domain.c \
	ocs_sport.c \
	ocs_node.c \
	ocs_unsol.c \
	ocs_io.c \
	ocs_els.c \
	ocs_scsi.c \
	ocs_fabric.c \
	ocs_device.c \
	ocs_ddump.c \
	ocs_xport.c \
	ocs_elxu.c \
	ocs_femul.c \
	spv.c \
	sli4.c \
	sli4_fc.c \
	ocs_sm.c \
	ocs_hal.c \
	ocs_hal_queues.c \
	ocs_hal_rqpair.c \
	ocs_hal_workaround.c \
	ocs_ini_stub.c \
	ocs_textbuf.c \
	ocs_dif.c \
	t10crc16.c \
	ocs_list.c \
	ocs_ramlog.c \
	ocs_mgmt_common.c \
	ocs_ddumplib.c \
	ocs_cbuf.c \
	ocs_array.c \
	ocs_pool.c \
	ocs_spdk_nvmet.c \
	spdk_nvmf_xport.c \
	ocs_tgt_stub.c

LIB := $(CONFIG_PREFIX)libufc.a

DEPFLAGS = -MMD -MP -MF $*.d.tmp

OBJS = $(C_SRCS:.c=.o)

.PHONY: all clean

all: $(CONFIG_PREFIX) $(LIB)
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

.PRECIOUS: $(OBJS)

-include $(OBJS:.o=.d)

