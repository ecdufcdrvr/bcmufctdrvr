#
# BSD LICENSE
#
# Copyright (C) 2024 Broadcom. All Rights Reserved.
# The term “Broadcom” refers to Broadcom Inc. and/or its subsidiaries.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
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
CFLAGS += -Wno-pointer-sign -Wstrict-prototypes -Wno-cast-function-type -Wno-address-of-packed-member
# Disable clang warning: passing an object that undergoes default argument promotion to 'va_start' has undefined behavior [-Wvarargs]
CFLAGS += -Wno-varargs
CFLAGS += -D_GNU_SOURCE -DOCS_USPACE -DOCS_USPACE_SPDK -DOCS_USPACE_SPDK_UPSTREAM -DENABLE_DMABUF_SPDK

CFLAGS += -I$(CURDIR)/src -I$(CURDIR)/include -I$(DPDK_DIR)/include -I$(SPDK_DIR)/include -I$(SPDK_DIR)/lib/env_dpdk -I$(SPDK_DIR)/lib/nvmf

SPDK_SRCS = \
	src/spdk_probe.c \
	src/ocs_ocsu.c \
	src/ocs_driver.c \
	src/ocs_spdk_nvmet.c \
	src/spdk_nvmf_xport.c \
	src/ocs_impl_spdk.c \
	src/ocs_stub.c 

SPDK_COMMON_C_SRCS = \
	src/ocs_os.c \
	src/crc32.c \
	src/dslab.c \
	src/ocsu_ioctl.c \
	src/ocs_socket.c \
	src/ocs_mgmt.c \
	src/ocs_debug.c \
	src/ocs_domain.c \
	src/ocs_sport.c \
	src/ocs_node.c \
	src/ocs_unsol.c \
	src/ocs_io.c \
	src/ocs_els.c \
	src/ocs_scsi.c \
	src/ocs_fabric.c \
	src/ocs_device.c \
	src/ocs_ddump.c \
	src/ocs_xport.c \
	src/ocs_elxu.c \
	src/ocs_femul.c \
	src/spv.c \
	src/ocs_auth.c \
	src/sli4.c \
	src/sli4_fc.c \
	src/ocs_sm.c \
	src/ocs_hal.c \
	src/ocs_hal_queues.c \
	src/ocs_hal_rqpair.c \
	src/ocs_hal_workaround.c \
	src/ocs_textbuf.c \
	src/ocs_dif.c \
	src/t10crc16.c \
	src/ocs_list.c \
	src/ocs_ramlog.c \
	src/ocs_mgmt_common.c \
	src/ocs_ddumplib.c \
	src/ocs_cbuf.c \
	src/ocs_array.c \
	src/ocs_pool.c \
	src/ocs_ras.c \
	src/ocs_gendump.c \
	src/ocs_recovery.c \
	src/ocs_compat.c \
	src/ocs_ioctl.c \
	src/ocs_ioctl_fc.c \
	src/ocs_recovery.c \
	src/ocs_ini_stub.c

C_SRCS  = $(SPDK_SRCS)
C_SRCS += $(SPDK_COMMON_C_SRCS)

LIB := $(CONFIG_PREFIX)libufc.a

DEPFLAGS = -MMD -MP -MF $*.d.tmp

OBJS = $(C_SRCS:.c=.o)
TEXT_D_FILES = $(C_SRCS:.c=.d)

.PHONY: all clean include

all: $(CONFIG_PREFIX) $(LIB) include
	@:

clean:
	$(Q)rm -rf *.a *.o *.d *.d.tmp *.gcno *.gcda $(LIB) $(CONFIG_PREFIX) $(OBJS) $(TEXT_D_FILES)

$(CONFIG_PREFIX):
	mkdir -p $(CONFIG_PREFIX)

$(LIB): $(OBJS)
	$(Q)echo "  LIB $(notdir $@)"; \
	rm -f $@; \
	ar rcs $@ $(OBJS)

%.o: %.c %.d $(MAKEFILE_LIST)
	$(Q)echo "  CC $S/$@"; \
	cc -o $@ $(DEPFLAGS) $(CFLAGS) -c $< && \
	mv -f $*.d.tmp $*.d && touch -c $@

%.d: ;

include:
	cp include/* $(CONFIG_PREFIX)

.PRECIOUS: $(OBJS)

-include $(OBJS:.o=.d)

