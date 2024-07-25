/*
 * BSD LICENSE
 *
 * Copyright (C) 2024 Broadcom. All Rights Reserved.
 * The term “Broadcom” refers to Broadcom Inc. and/or its subsidiaries.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 *
 */

#include "ocs_os.h"

#if !defined(__OCS_RAS_H__)
#define __OCS_RAS_H__

#define OCS_RAS_BUFFER_COUNT_MAX		16

/*
 * RAS diagnostic log memory consists of #n equal sized buffers. Each of these buffers
 * can be multiples of 4KB, range is 4K to 64KB. Value of #n (number of memory chunks)
 * is 1 - 16. So logical buffer can be 4K to 1MB, however OCS limits the range to
 * 128KB - 1MB.
 */
#define OCS_RAS_BUFFER_UNIT_MIN			(16 * 1024)
#define OCS_RAS_BUFFER_UNIT_MAX			(64 * 1024)
#define OCS_RAS_LOGICAL_BUFFER_SIZE_MIN		(128 * 1024)
#define OCS_RAS_LOGICAL_BUFFER_SIZE_MAX		(1024 * 1024)

#define OCS_RAS_FW_LOG_LEVEL_MIN		0
#define OCS_RAS_FW_LOG_LEVEL_MAX		4

#define OCS_RAS_DMA_ADDR_LOW_MASK		0xFFFFFFFF
#define OCS_RAS_DMA_ADDR_HI_SHIFT		32

#define OCS_RAS_MOD_INIT_DONE			1

typedef struct ocs_diag_log_buf_s {
	char *addr;
	uint32_t offset;
	uint32_t length;
} ocs_diag_log_buf_t;

typedef struct ocs_ras_adapter_desc_s {
	ocs_list_link_t link;
	uint32_t pci_busdev;
	ocs_ref_t refcnt;
	struct pci_dev *pdev;
	uint32_t buf_unit_size;
	uint32_t log_buf_size;
	uint32_t log_buf_count;
	uint32_t fw_log_level;
	bool lwpd_allocated;
	ocs_dma_t lwpd_buffer; /* Log Write Position Data */
	ocs_dma_t buffers[OCS_RAS_BUFFER_COUNT_MAX];
} ocs_ras_adapter_desc_t;

typedef struct ocs_ras_subsystem_s {
	bool init_done;
	uint32_t log_buf_size;
	uint32_t fw_log_level;
	uint32_t num_adapters;
	ocs_lock_t lock;
	ocs_list_t adapter_list;
} ocs_ras_subsystem_t;

extern void ocs_ras_init(uint32_t fw_log_level, uint32_t log_buf_size);
extern int ocs_ras_allocate_resc(ocs_os_handle_t *ocs_os);
extern int ocs_ras_free_resc(ocs_os_handle_t *ocs_os);
extern uintptr_t ocs_ras_get_lwpd_dma_addr(ocs_os_handle_t *ocs_os);
extern uint32_t * ocs_ras_get_dma_addr_list(ocs_os_handle_t *ocs_os, int32_t *buf_count);
extern int ocs_ras_copy_logs(ocs_t *ocs, void *user_buf, int32_t user_buf_size);

#endif // __OCS_RAS_H__
