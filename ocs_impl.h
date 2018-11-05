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

#ifdef SPDK_CONFIG_PCIACCESS
#include <pciaccess.h>
#else
#include <rte_pci.h>
#endif

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
			*phys_addr = spdk_vtophys(buf_ptr);
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
#define ocs_vtophys(buf)		spdk_vtophys(buf)

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

#ifdef SPDK_CONFIG_PCIACCESS

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

	ocs_enum_ctx.user_enum_cb = enum_cb;
	ocs_enum_ctx.user_enum_ctx = enum_ctx;

	return spdk_pci_enumerate(ocs_pci_enum_cb, &ocs_enum_ctx);
}

#else /* !SPDK_CONFIG_PCIACCESS */

#define SPDK_OCS_PCI_DEVICE(DEVICE_ID) RTE_PCI_DEVICE(SPDK_PCI_VID_OCS, DEVICE_ID)

/* TODO: avoid duplicating the device ID list */
static struct rte_pci_id ocs_driver_id[] = {
	{SPDK_OCS_PCI_DEVICE(PCI_DEVICE_ID_OCS_LANCERG5)},
	{SPDK_OCS_PCI_DEVICE(PCI_DEVICE_ID_OCS_LANCERG6)},
	{ .vendor_id = 0, /* sentinel */ },
};

/*
 * TODO: eliminate this global if possible (does rte_pci_driver have a context field for this?)
 *
 * This should be protected by the ocs driver lock, since ocs_probe() holds the lock
 *  the whole time, but we shouldn't have to depend on that.
 */
static struct ocs_pci_enum_ctx g_ocs_pci_enum_ctx;

static int
ocs_driver_init(struct rte_pci_driver *dr, struct rte_pci_device *rte_dev)
{
	/*
	 * These are actually the same type internally.
	 * TODO: refactor this so it's inside pci.c
	 */
	struct spdk_pci_device *pci_dev = (struct spdk_pci_device *)rte_dev;

	return g_ocs_pci_enum_ctx.user_enum_cb(g_ocs_pci_enum_ctx.user_enum_ctx, pci_dev);
}

static struct rte_pci_driver ocs_rte_driver = {
#if RTE_VERSION >= RTE_VERSION_NUM(16, 11, 0, 0)
	.driver = {
		.name = "ocs_driver"
	},
	.probe = ocs_driver_init,
#else
	.name = "ocs_driver",
	.devinit = ocs_driver_init,
#endif
	.id_table = ocs_driver_id,
	.drv_flags = RTE_PCI_DRV_NEED_MAPPING,
};

static inline int
ocs_pci_enumerate(int (*enum_cb)(void *enum_ctx, struct spdk_pci_device *pci_dev), void *enum_ctx)
{
	int rc;

	g_ocs_pci_enum_ctx.user_enum_cb = enum_cb;
	g_ocs_pci_enum_ctx.user_enum_ctx = enum_ctx;

	rte_pci_register(&ocs_rte_driver);
	rc = rte_bus_probe();
	rte_pci_unregister(&ocs_rte_driver);

	return rc;
}

#endif /* !SPDK_CONFIG_PCIACCESS */

typedef pthread_mutex_t ocs_mutex_t;

#define ocs_mutex_lock pthread_mutex_lock
#define ocs_mutex_unlock pthread_mutex_unlock
#define OCS_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER

#endif /* __OCS_IMPL_H__ */
