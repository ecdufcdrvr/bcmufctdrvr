/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Inc. and/or its subsidiaries.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *    * Neither the name of Intel Corporation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ocs_IMPL_H__
#define __ocs_IMPL_H__

#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <rte_config.h>
#include <rte_malloc.h>
#include <rte_atomic.h>
#include <rte_cycles.h>
#include <rte_version.h>
#include <rte_bus_pci.h>
#include <stdbool.h>

#include "spdk/env.h"
#include "ocs_pci.h"
#include "env_internal.h"

/**
 * \file
 *
 * This file describes the functions required to integrate
 * the userspace OCS driver for a specific implementation.  This
 * implementation is specific for DPDK.  Users would revise it as
 * necessary for their own particular environment if not using it
 * within the SPDK framework.
 */

/**
 * Allocate a pinned, physically contiguous memory buffer with the
 * given size and alignment.
 */
static inline void *
ocs_spdk_zmalloc(const char *tag, size_t size, unsigned align, uint64_t *phys_addr)
{
	void *buf_ptr = NULL;
	if (tag) {
		buf_ptr = spdk_memzone_reserve_aligned(tag, size, SPDK_ENV_SOCKET_ID_ANY, 0, align);
		if (buf_ptr) {
			*phys_addr = spdk_vtophys(buf_ptr, &size);
		}
	}
	else {
		ocs_log_err(NULL, "ocs_zmalloc() call without a tag!\n");
	}
	return buf_ptr;
}

/**
 * Free a memory buffer previously allocated with ocsu_zmalloc.
 */
#define ocs_spdk_free(mz_name)		spdk_memzone_free(mz_name)

/**
 * Return the physical address for the specified virtual address.
 */
#define ocs_vtophys(buf)		spdk_vtophys(buf, NULL)

/**
 * Delay us.
 */
#define ocs_delay_us(us)		rte_delay_us(us)

/**
 * Assert a condition and panic/abort as desired.  Failures of these
 *  assertions indicate catastrophic failures within the driver.
 */
#define ocs_spdk_assert(check)		assert(check)

/**
 * Log or print a message from the driver.
 */
#define ocs_spdk_printf(chan, fmt, args...) printf(fmt, ##args)

/**
 *
 */
#define ocs_pcicfg_read32(handle, var, offset)  spdk_pci_device_cfg_read32(handle, var, offset)
#define ocs_pcicfg_write32(handle, var, offset) spdk_pci_device_cfg_write32(handle, var, offset)

#define SPDK_OCS_PCI_DEVICE(DEVICE_ID) RTE_PCI_DEVICE(SPDK_PCI_VID_OCS, DEVICE_ID)

#define OCSU_DRIVER_NAME_UIO_GENERIC		"uio_pci_generic"
#define OCSU_DRIVER_NAME_VFIO			"vfio_pci"

struct ocs_pci_enum_ctx {
	int (*user_enum_cb)(void *enum_ctx, struct spdk_pci_device *pci_dev);
	void *user_enum_ctx;
};

static inline bool
ocs_pci_device_match_id(uint16_t vendor_id, uint16_t device_id)
{
	if (vendor_id != SPDK_PCI_VID_OCS) {
		return false;
	}

	switch (device_id) {
	case PCI_DEVICE_ID_OCS_LANCERG5:
	case PCI_DEVICE_ID_OCS_LANCERG6:
		return true;
	}

	return false;
}

static int
ocs_pci_enum_cb(void *enum_ctx, struct spdk_pci_device *pci_dev)
{
	struct ocs_pci_enum_ctx *ctx = enum_ctx;
	uint16_t vendor_id = spdk_pci_device_get_vendor_id(pci_dev);
	uint16_t device_id = spdk_pci_device_get_device_id(pci_dev);

	if (!ocs_pci_device_match_id(vendor_id, device_id)) {
		return 0;
	}

	return ctx->user_enum_cb(ctx->user_enum_ctx, pci_dev);
}

static inline int
ocs_pci_enumerate(int (*enum_cb)(void *enum_ctx, struct spdk_pci_device *pci_dev), void *enum_ctx)
{
	struct ocs_pci_enum_ctx ocs_enum_ctx;
	struct spdk_pci_driver *pci_driver;

	ocs_enum_ctx.user_enum_cb = enum_cb;
	ocs_enum_ctx.user_enum_ctx = enum_ctx;

	pci_driver = spdk_pci_get_driver(OCSU_DRIVER_NAME_UIO_GENERIC);
	if (pci_driver)
		spdk_pci_enumerate(pci_driver, ocs_pci_enum_cb, &ocs_enum_ctx);

	return 0;
}


typedef pthread_mutex_t ocs_mutex_t;

#define ocs_mutex_lock pthread_mutex_lock
#define ocs_mutex_unlock pthread_mutex_unlock
#define OCS_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER

#endif /* __OCS_IMPL_H__ */
