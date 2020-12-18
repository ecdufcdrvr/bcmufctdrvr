/*
 * Copyright (C) 2020 Broadcom. All Rights Reserved.
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
#include "ocs.h"
#include "ocs_ras.h"

static ocs_ras_subsystem_t ocs_ras_subsystem_g;

static uint32_t
ocs_ras_get_sli_dev_pci_info(ocs_t *ocs)
{
	uint8_t pci_bus;
	uint8_t pci_dev;
	uint8_t pci_func;

	/* Gather PCI bus info to group together all SLI ports of a SLI device */
	ocs_get_bus_dev_func(ocs, &pci_bus, &pci_dev, &pci_func);
	return (uint32_t)pci_bus << 8 | (uint32_t)pci_dev;
}

static ocs_ras_adapter_desc_t *
ocs_ras_lookup_adapter(ocs_t *ocs)
{
	uint32_t pci_busdev;
	ocs_ras_adapter_desc_t *adapter_desc;

	/* Gather PCI bus info to bind together all SLI ports of a SLI device together */
	pci_busdev = ocs_ras_get_sli_dev_pci_info(ocs);

	ocs_lock(&ocs_ras_subsystem_g.lock);
	ocs_list_foreach(&ocs_ras_subsystem_g.adapter_list, adapter_desc) {
		if (adapter_desc->pci_busdev == pci_busdev) {
			/* Found the adapter to which this SLI port belongs */
			ocs_unlock(&ocs_ras_subsystem_g.lock);
			return adapter_desc;
		}
	}

	ocs_unlock(&ocs_ras_subsystem_g.lock);
	return NULL;
}

static bool
ocs_ras_get_virt_buf_info(ocs_t *ocs, ocs_ras_adapter_desc_t *adapter_desc, int32_t *active_idx, int32_t *offset)
{
	uint32_t *ptr = (uint32_t *)adapter_desc->lwpd_buffer.virt;
	uint32_t loffset;
	uint32_t wrap_seq;
	uint32_t buf_sz;
	bool logs_wrapped;

	loffset = ocs_be32toh(ptr[0]);
	wrap_seq = ocs_be32toh(ptr[1]);
	buf_sz = (adapter_desc->log_buf_size / adapter_desc->log_buf_count);

	*active_idx = loffset / buf_sz;
	*offset = loffset % buf_sz;
	if (wrap_seq)
		logs_wrapped = true;
	else
		logs_wrapped = false;

	return logs_wrapped;
}

static ocs_diag_log_buf_t *
ocs_ras_get_virtual_buf(ocs_t *ocs, int32_t *buf_count)
{
	ocs_diag_log_buf_t *buf_desc_tbl;
	ocs_ras_adapter_desc_t *adapter_desc;
	int32_t active_idx;
	int32_t start_idx;
	int32_t start_offset;
	uint32_t valid_log_sz;
	int32_t offset;
	bool logs_wrapped;
	int32_t tbl_size;
	int32_t i;

	adapter_desc = ocs_ras_lookup_adapter(ocs);
	if (!adapter_desc)
		return NULL;

	logs_wrapped = ocs_ras_get_virt_buf_info(ocs, adapter_desc, &active_idx, &offset);

	if (!active_idx && !offset && !logs_wrapped) {
		/* Log buffer is empty */
		*buf_count = 0;
		return NULL;
	}

	/* 4 scenarios to determine buffer descriptor table - a) logs wrapper or not. b) offset is at zero or not */
	if (logs_wrapped) {
		tbl_size = adapter_desc->log_buf_count;
		valid_log_sz = adapter_desc->log_buf_size;
		start_idx = active_idx;
		start_offset = offset;
		ocs_log_debug(ocs, "FW logs_wrapped; start_idx: 0x%x, start_offset: 0x%x\n", start_idx, start_offset);
	} else {
		tbl_size = active_idx;
		valid_log_sz = active_idx * adapter_desc->buf_unit_size + offset;
		start_idx = 0;
		start_offset = 0;
	}

	if (offset)
		tbl_size++;

	buf_desc_tbl = ocs_vmalloc(ocs, tbl_size * sizeof(ocs_diag_log_buf_t), OCS_M_ZERO);
	if (!buf_desc_tbl)
		return NULL;

	for (i = 0; i < tbl_size; i++) {
		buf_desc_tbl[i].addr = (char *)adapter_desc->buffers[(start_idx + i) % adapter_desc->log_buf_count].virt;
		buf_desc_tbl[i].offset = start_offset;

		if (start_offset) {
			buf_desc_tbl[i].length = adapter_desc->buf_unit_size - start_offset;
			start_offset = 0;
		} else {
			buf_desc_tbl[i].length =
				valid_log_sz > adapter_desc->buf_unit_size ?
				adapter_desc->buf_unit_size : valid_log_sz;
		}

		valid_log_sz -= buf_desc_tbl[i].length;
		if (!valid_log_sz)
			break;
	}

	if (valid_log_sz)
		ocs_log_err(ocs, "[RAS] BUG: residual log size %d\n", valid_log_sz);

	*buf_count = tbl_size;
	return buf_desc_tbl;
}

uintptr_t
ocs_ras_get_lwpd_dma_addr(ocs_os_handle_t *ocs_os)
{
	ocs_t *ocs = (ocs_t *)ocs_os;
	ocs_ras_adapter_desc_t *adapter_desc;

	adapter_desc = ocs_ras_lookup_adapter(ocs);
	if (!adapter_desc)
		return (uintptr_t)NULL;

	return adapter_desc->lwpd_buffer.phys;
}

uint32_t *
ocs_ras_get_dma_addr_list(ocs_os_handle_t *ocs_os, int32_t *buf_count)
{
	int32_t i;
	int32_t tbl_size;
	uint32_t *addr_tbl;
	ocs_t *ocs = (ocs_t *)ocs_os;
	ocs_ras_adapter_desc_t *adapter_desc;

	*buf_count = 0;
	adapter_desc = ocs_ras_lookup_adapter(ocs);
	if (!adapter_desc)
		return NULL;

	tbl_size = adapter_desc->log_buf_count;
	if (!tbl_size)
		return NULL;

	addr_tbl = ocs_vmalloc(ocs, tbl_size * 8, OCS_M_ZERO);
	if (!addr_tbl)
		return NULL;

	for (i = 0; i < tbl_size; i++) {
		addr_tbl[i * 2] = (uint32_t)(adapter_desc->buffers[i].phys & OCS_RAS_DMA_ADDR_LOW_MASK);
		addr_tbl[i * 2 + 1] = (uint32_t)(adapter_desc->buffers[i].phys >> OCS_RAS_DMA_ADDR_HI_SHIFT);
	}

	*buf_count = tbl_size;
	return addr_tbl;
}

static int
ocs_ras_gather_logs(void *log_buf, int32_t user_buf_len, ocs_diag_log_buf_t *buf_desc_tbl, int32_t buf_count)
{
	int32_t i;
	int32_t bytes_copied = 0;
	char *ptr = (char *)log_buf;

	for (i = 0; i < buf_count; i++) {
		if (bytes_copied + (int32_t)buf_desc_tbl[i].length > user_buf_len) {
			bytes_copied = -EINVAL;
			break;
		}

		ocs_memcpy(ptr + bytes_copied, buf_desc_tbl[i].addr + buf_desc_tbl[i].offset, buf_desc_tbl[i].length);
		bytes_copied += buf_desc_tbl[i].length;
	}

	return bytes_copied;
}

int
ocs_ras_copy_logs(ocs_t *ocs, void *user_buf, int32_t user_buf_len)
{
	ocs_diag_log_buf_t *buf_desc_tbl;
	int32_t buf_count;
	int rc;

	if (!user_buf)
		return -EINVAL;

	buf_desc_tbl = ocs_ras_get_virtual_buf(ocs, &buf_count);
	if (!buf_desc_tbl || !buf_count)
		return -EIO;

	rc = ocs_ras_gather_logs(user_buf, user_buf_len, buf_desc_tbl, buf_count);

	ocs_vfree(ocs, buf_desc_tbl, sizeof(ocs_diag_log_buf_t) * buf_count);
	return rc;
}

static int32_t
ocs_ras_alloc_dma_buffers(ocs_t *ocs, ocs_ras_adapter_desc_t *adapter_desc)
{
	int32_t i;
	int32_t rc;
	int32_t buf_count;

	rc = ocs_dma_alloc(ocs, &adapter_desc->lwpd_buffer, 64, 8);
	if (rc)
		return rc;

	adapter_desc->lwpd_allocated = true;

	buf_count =  adapter_desc->log_buf_size / adapter_desc->buf_unit_size;
	for (i = 0; i < buf_count; i++) {
		rc = ocs_dma_alloc(ocs, &adapter_desc->buffers[i], adapter_desc->buf_unit_size, 4096);
		if (rc)
			return rc;

		adapter_desc->log_buf_count++;
	}

	return 0;
}

static void
ocs_ras_free_dma_buffers(ocs_ras_adapter_desc_t *adapter_desc)
{
	uint32_t i;

	if (adapter_desc->lwpd_allocated) {
		ocs_dma_free_unbound(&adapter_desc->lwpd_buffer);
		adapter_desc->lwpd_allocated = false;
	}

	for (i = 0; i < adapter_desc->log_buf_count; i++) {
		ocs_dma_free_unbound(&adapter_desc->buffers[i]);
	}

	adapter_desc->log_buf_count = 0;
}

static void
ocs_ras_free_adapter_descriptor(ocs_ras_adapter_desc_t *adapter_desc)
{
	ocs_lock(&ocs_ras_subsystem_g.lock);
	ocs_list_remove(&ocs_ras_subsystem_g.adapter_list, adapter_desc);
	ocs_unlock(&ocs_ras_subsystem_g.lock);

	ocs_ras_free_dma_buffers(adapter_desc);
	ocs_free(NULL, adapter_desc, sizeof(*adapter_desc));
}

static void
ocs_ras_put_adapter_descriptor(void *cb_arg)
{
	ocs_ras_free_adapter_descriptor((ocs_ras_adapter_desc_t *)cb_arg);
}


extern int ocs_hal_stop_fw_diagnostic_logging(ocs_hal_t *hal);

int
ocs_ras_free_resc(ocs_os_handle_t *ocs_os)
{
	ocs_t *ocs = (ocs_t *)ocs_os;
	ocs_ras_adapter_desc_t *adapter_desc;
	int rc = 0;

	if (ocs_ras_subsystem_g.init_done != OCS_RAS_MOD_INIT_DONE) {
		ocs_log_debug(ocs, "[RAS] FW logging subsystem is not initialized\n");
		return -EPERM;
	}

	adapter_desc = ocs_ras_lookup_adapter(ocs);
	if (!adapter_desc) {
		ocs_log_debug(ocs, "[RAS] no resources to free\n");
		return -ENODEV;
	}

	if (ocs_ref_read_count(&adapter_desc->refcnt) == 1) {
		/* This is the last SLI port for this adapter, disable firmware diagnostic logging */
		rc = ocs_hal_stop_fw_diagnostic_logging(&ocs->hal);
	}

	ocs_ref_put(&adapter_desc->refcnt);
	return rc;
}

int32_t
ocs_ras_allocate_resc(ocs_os_handle_t *ocs_os)
{
	ocs_t *ocs = (ocs_t *)ocs_os;
	ocs_ras_adapter_desc_t *adapter_desc;

	if (!ocs_ras_subsystem_g.log_buf_size) {
		/* Nothing to do, RAS diagnostic logging is disabled */
		return -EOPNOTSUPP;
	}

	adapter_desc = ocs_ras_lookup_adapter(ocs);
	if (adapter_desc) {
		ocs_ref_get(&adapter_desc->refcnt);
		return -EEXIST;
	}

	adapter_desc = ocs_malloc(ocs, sizeof(*adapter_desc), OCS_M_ZERO | OCS_M_NOWAIT);
	if (!adapter_desc)
		return -ENOMEM;

	adapter_desc->pci_busdev = ocs_ras_get_sli_dev_pci_info(ocs);
	ocs_ref_init(&adapter_desc->refcnt, ocs_ras_put_adapter_descriptor, adapter_desc);
	adapter_desc->log_buf_size = ocs_ras_subsystem_g.log_buf_size;

	if (adapter_desc->log_buf_size / OCS_RAS_BUFFER_UNIT_MIN <=
	    OCS_RAS_BUFFER_COUNT_MAX) {
		adapter_desc->buf_unit_size = OCS_RAS_BUFFER_UNIT_MIN;
	} else {
		adapter_desc->buf_unit_size = OCS_RAS_BUFFER_UNIT_MAX;
	}

	adapter_desc->log_buf_count = 0;
	adapter_desc->fw_log_level = ocs_ras_subsystem_g.fw_log_level;

	if (ocs_ras_alloc_dma_buffers(ocs, adapter_desc)) {
		ocs_log_err(ocs, "[RAS] unable to allocate resources\n");
		goto err_log_buffer;
	}

	ocs_lock(&ocs_ras_subsystem_g.lock);
	ocs_list_add_tail(&ocs_ras_subsystem_g.adapter_list, adapter_desc);
	ocs_unlock(&ocs_ras_subsystem_g.lock);

	return 0;

err_log_buffer:
	ocs_ras_free_adapter_descriptor(adapter_desc);
	return -ENOMEM;
}

void
ocs_ras_init(uint32_t fw_log_level, uint32_t log_buf_size)
{
	if (ocs_ras_subsystem_g.init_done == OCS_RAS_MOD_INIT_DONE) {
		ocs_log_debug(NULL, "[RAS] subsystem already initialized\n");
		return;
	}

	ocs_ras_subsystem_g.log_buf_size = log_buf_size;
	ocs_ras_subsystem_g.fw_log_level = fw_log_level;
	ocs_ras_subsystem_g.num_adapters = 0;
	ocs_lock_init(NULL, &ocs_ras_subsystem_g.lock, "ras diag logging lock");
	ocs_list_init(&ocs_ras_subsystem_g.adapter_list, ocs_ras_adapter_desc_t, link);
	ocs_ras_subsystem_g.init_done = OCS_RAS_MOD_INIT_DONE;
}
