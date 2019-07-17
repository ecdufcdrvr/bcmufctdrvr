#
#  BSD LICENSE
#
#  Copyright (c) 2018 Broadcom.  All Rights Reserved.
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

SPDK_ROOT_DIR := $(abspath $(CURDIR)/..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

DEFINES         = -DOCS_USPACE_SPDK -DOCS_NVME_FC

CFLAGS += $(ENV_CFLAGS) -I$(SPDK_ROOT_DIR)/lib/scsi -I$(SPDK_ROOT_DIR)/lib/nvmf
CFLAGS += $(DPDK_INC) $(DEFINES)

C_SRCS     =  \
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
	ocs_spdk_conf.c \
	task.c \
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
	ocs_tgt_spdk.c 

LIBNAME = fc
include $(SPDK_ROOT_DIR)/mk/spdk.lib.mk

# Disable clang warning: passing an object that undergoes default argument promotion to 'va_start' has undefined behavior [-Wvarargs]
CFLAGS += -Wno-varargs
