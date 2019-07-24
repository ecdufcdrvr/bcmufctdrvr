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

/**
 * Opaque handle for a single I/OAT channel returned by \ref struct spdk_ocs_t_probe().
 */
#define PCI_MAX_BAR	6

struct spdk_ocs_t;

typedef struct {
	void *vaddr;
	uint64_t paddr;
	size_t size;

	/* memory region name. */
	char name[64];
} spdk_ocs_memref_t;

struct spdk_ocs_get_pci_config_t {
	uint16_t vendor;
	uint16_t device;
	uint32_t bar_count;
	spdk_ocs_memref_t bars[PCI_MAX_BAR];
	uint8_t bus;
	uint8_t dev;
	uint8_t func;
	uint8_t num_msix;
	uint8_t numa_node;
};

/**
 * Signature for callback function invoked when a request is completed.
 *
 * \param arg User-specified opaque value corresponding to cb_arg from the request submission.
 */

/**
 * Callback for struct spdk_ocs_t_probe() enumeration.
 *
 * \param cb_ctx User-specified opaque value corresponding to cb_ctx from struct spdk_ocs_t_probe().
 * \param pci_dev PCI device that is being probed.
 *
 * \return true to attach to this device.
 */
typedef bool (*spdk_ocs_probe_cb)(void *cb_ctx, struct spdk_pci_device *pci_dev);

/**
 * Callback for struct spdk_ocs_t_probe() to report a device that has been attached to the userspace I/OAT driver.
 *
 * \param cb_ctx User-specified opaque value corresponding to cb_ctx from struct spdk_ocs_t_probe().
 * \param pci_dev PCI device that was attached to the driver.
 * \param ocs I/OAT channel that was attached to the driver.
 */
typedef void (*spdk_ocs_attach_cb)(void *cb_ctx, struct spdk_pci_device *pci_dev,
		struct spdk_ocs_t *ocs);

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
 * call \ref struct spdk_ocs_t_detach with the ocs_channel instance returned by this function.
 */
extern int spdk_ocs_probe(void *cb_ctx, spdk_ocs_probe_cb probe_cb, spdk_ocs_attach_cb attach_cb);

/**
 * Detaches specified device returned by \ref struct spdk_ocs_t_probe() from the I/OAT driver.
 *
 * \param ocs I/OAT channel to detach from the driver.
 */
int spdk_ocs_detach(struct spdk_ocs_t *ocs);

struct spdk_ocs_t * spdk_ocs_attach(void *device);

struct spdk_ocs_t * spdk_ocs_get_object(struct spdk_pci_device *dev);
int spdk_ocs_get_pci_config(struct spdk_pci_device *dev, struct spdk_ocs_get_pci_config_t *pci_config);

/**
 * Get the DMA engine capabilities.
 *
 * \param chan I/OAT channel to query.
 *
 * \return A combination of flags from \ref struct spdk_ocs_t_dma_capability_flags.
 */
#ifdef __cplusplus
}
#endif

#endif
