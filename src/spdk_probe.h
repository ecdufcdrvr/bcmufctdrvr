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

/** \file
 * I/OAT DMA engine driver public interface
 */

#ifndef OCS_SPDK_H
#define OCS_SPDK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>
#include <stdbool.h>

#include "spdk/env.h"
#include "spdk/pci_ids.h"

#define SPDK_PCI_VID_OCS		0x10df
#define PCI_DEVICE_ID_OCS_LANCERG5	0xe200
#define PCI_DEVICE_ID_OCS_LANCERG6	0xe300
#define PCI_DEVICE_ID_OCS_LANCERG7	0xf400
#define PCI_DEVICE_ID_OCS_LANCERG7PLUS	0xf500
#define PCI_DEVICE_ID_OCS_LANCERG7PLUS_S 0xf600

#define PCI_MAX_BAR	6

struct spdk_ocs_memref {
	void *vaddr;
	uint64_t paddr;
	size_t size;

	/* memory region name. */
	char name[64];
};

struct spdk_ocs_get_pci_config {
	uint16_t vendor;
	uint16_t device;
	uint32_t bar_count;
	ocsu_memref_t bars[PCI_MAX_BAR];
	uint8_t bus;
	uint8_t dev;
	uint8_t func;
	uint8_t num_msix;
	uint8_t numa_node;
};

struct spdk_ocs {
	uint32_t instance_index;
	void *pdev;
	ocsu_memref_t bars[PCI_MAX_BAR];
	uint32_t bar_count;
	const char *desc;
	uint16_t pci_vendor;
	uint16_t pci_device;
	char businfo[16];
	uint8_t if_type;
	struct spdk_ocs_memref bmbx;

	/* tailq entry for attached_chans */
	TAILQ_ENTRY(spdk_ocs) tailq;
};

struct ocs_pci_enum_ctx {
	int (*user_enum_cb)(void *enum_ctx, struct spdk_pci_device *pci_dev);
	void *user_enum_ctx;
};

/**
 * Callback for struct spdk_ocs_probe() enumeration.
 *
 * \param cb_ctx User-specified opaque value corresponding to cb_ctx from struct spdk_ocs_probe().
 * \param pci_dev PCI device that is being probed.
 *
 * \return true to attach to this device.
 */
typedef bool (*spdk_ocs_probe_cb)(void *cb_ctx, struct spdk_pci_device *pci_dev);

/**
 * Callback for struct spdk_ocs_probe() to report a device that has been attached to the userspace I/OAT driver.
 *
 * \param cb_ctx User-specified opaque value corresponding to cb_ctx from struct spdk_ocs_probe().
 * \param pci_dev PCI device that was attached to the driver.
 * \param ocs I/OAT channel that was attached to the driver.
 */
typedef void (*spdk_ocs_attach_cb)(void *cb_ctx, struct spdk_pci_device *pci_dev,
		struct spdk_ocs *ocs);

/**
 * \brief Enumerate the I/OAT devices attached to the system and attach the userspace I/OAT driver
 * to them if desired.
 *
 * \param cb_ctx Opaque value which will be passed back in cb_ctx parameter of the callbacks.
 * \param probe_cb will be called once per I/OAT device found in the system.
 * \param attach_cb will be called for devices for which probe_cb returned true once the I/OAT
 * controller has been attached to the userspace driver.
 *
 * If called more than once, only devices that are not already attached to the SPDK I/OAT driver
 * will be reported.
 *
 * To stop using the the controller and release its associated resources,
 * call \ref struct spdk_ocs_detach with the ocs_channel instance returned by this function.
 */
extern int spdk_ocs_probe(void *cb_ctx, spdk_ocs_probe_cb probe_cb, spdk_ocs_attach_cb attach_cb);

/**
 * Detaches specified device returned by \ref struct spdk_ocs_probe() from the I/OAT driver.
 *
 * \param ocs I/OAT channel to detach from the driver.
 */
int spdk_ocs_detach(struct spdk_ocs *ocs);

struct spdk_ocs * spdk_ocs_attach(void *device);

struct spdk_ocs * spdk_ocs_get_object(struct spdk_pci_device *dev);
int spdk_ocs_get_pci_config(struct spdk_pci_device *dev, struct spdk_ocs_get_pci_config *pci_config);
int ocs_pci_enumerate(int (*enum_cb)(void *enum_ctx, struct spdk_pci_device *pci_dev), void *enum_ctx);

/**
 * Get the DMA engine capabilities.
 *
 * \param chan I/OAT channel to query.
 *
 * \return A combination of flags from \ref struct spdk_ocs_dma_capability_flags.
 */
#ifdef __cplusplus
}
#endif

#endif
