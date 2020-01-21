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

#include "ocs_internal.h"
#include "ocs_pci.h"
#include "ocs.h"
#include "ocs_ocsu.h"
#include "fc.h"
#include "ocs_spdk.h"
#include "ocs_impl.h"
#include "nvmf_fc.h"

struct spdk_ocs_t *spdk_ocs_devices[MAX_OCS_DEVICES];

struct ocs_driver {
	ocs_mutex_t	lock;
	TAILQ_HEAD(, spdk_ocs_t)	attached_chans;
};

static struct ocs_driver g_ocs_driver = {
	.lock = OCS_MUTEX_INITIALIZER,
	.attached_chans = TAILQ_HEAD_INITIALIZER(g_ocs_driver.attached_chans),
};

struct spdk_ocs_t *spdk_ocs_attach(void *device);
static uint32_t spdk_ocs_instance;
void ocs_spdk_exit(void);
void ocs_pci_set_bus_master(struct spdk_ocs_t *, bool );

void spdk_fc_shutdown(void)
{
	printf("Shutting down all OCS devices\n");
	ocs_spdk_exit();
}

static int
ocs_map_pci_bar(struct spdk_ocs_t *ocs)
{
	int bar, rc = 0;
	void *addr;
	uint64_t phys_addr, size;

	/* Map all memory BARs */
	for (bar = 0; bar < PCI_MAX_BAR; bar++) {
		size = 0;
		rc = spdk_pci_device_map_bar(ocs->pdev, bar, &addr, &phys_addr, &size);
		if (rc) {
			printf("Failed to map bar: %d\n", bar);
			rc = 0;
		}

		if (size) {
			ocs->bars[bar].paddr = phys_addr;
			ocs->bars[bar].vaddr = addr;
			ocs->bars[bar].size  = size;
			ocs_spdk_printf(ocs, "%s:bar[%d] mapped\n", __func__, bar);
		}

		ocs->bar_count++;
	}

	return rc;
}

static int
ocs_unmap_pci_bar(struct spdk_ocs_t *ocs)
{
	int i,rc = 0;

	for (i = 0; i < PCI_MAX_BAR; i++) {
		if (ocs->bars[i].vaddr) {
			spdk_pci_device_unmap_bar(ocs->pdev, i, ocs->bars[i].vaddr);
		}
	}

	return rc;
}

/* PCI Bus master enable or disable in config space*/

void ocs_pci_set_bus_master(struct spdk_ocs_t *ocs, bool enable)
{
	uint32_t pci_reg;

	ocs_pcicfg_read32(ocs->pdev, &pci_reg, 0x4);

	if (enable) {
		pci_reg |= BIT_2;
	} else {
		pci_reg &= ~BIT_2;
	}

	ocs_pcicfg_write32(ocs->pdev, pci_reg, 0x4);

}

struct spdk_ocs_t *
spdk_ocs_attach(void *device)
{
	struct spdk_ocs_t *ocs;
	struct ocs_driver *driver = &g_ocs_driver;
	uint32_t sli_intf;
	uint64_t phys;

	ocs_mutex_lock(&driver->lock);

	ocs = calloc(1, sizeof(struct spdk_ocs_t));
	if (ocs == NULL) {
		ocs_spdk_printf(NULL,"%s: Failed to create ocs structure\n",
			__func__);
		goto error1;
	}
	ocs->instance_index = spdk_ocs_instance;
	ocs->pdev = device;
	ocs->pci_vendor = spdk_pci_device_get_vendor_id(device);
	ocs->pci_device = spdk_pci_device_get_device_id(device);
        snprintf(ocs->businfo, sizeof(ocs->businfo), "%02X:%02X:%02X",
                spdk_pci_device_get_bus(device), spdk_pci_device_get_dev(device),
                spdk_pci_device_get_func(device));

	if (ocs_map_pci_bar(ocs) != 0) {
		goto error2;
	}

	ocs_pcicfg_read32(ocs->pdev, &sli_intf, 0x58);
	ocs->if_type = (sli_intf >> 12) & 0xf;

	/* Enable PCI Bus master bit in PCI configspace*/
	ocs_pci_set_bus_master(ocs, true);

        /* Allocate memory for the bootstrap mailbox
 	 *
 	 * The bootstrap mailbox is equivalent to a MQ with a single 256 byte
 	 * entry, a CQ with a single 16 byte entry, and no event queue.
 	 * Alignment must be 16 bytes as the low order address bits in the
 	 * address register are also control status.
 	*/
        snprintf(ocs->bmbx.name, sizeof(ocs->bmbx.name), "ocs_attach_%d", ocs->instance_index);
        ocs->bmbx.size = SLI4_BMBX_SIZE + 32 + 16;
        ocs->bmbx.vaddr = ocs_spdk_zmalloc(ocs->bmbx.name, ocs->bmbx.size, 64, &phys);
        if (ocs->bmbx.vaddr == NULL) {
                ocs_spdk_printf(ocs, "Error: %s dma_alloc_cohereent failed for bmbx\n", __func__);
		goto error3;
        }
	ocs->bmbx.paddr = phys;

	spdk_ocs_devices[spdk_ocs_instance ++] = ocs;
	TAILQ_INSERT_TAIL(&g_ocs_driver.attached_chans, ocs, tailq);
	ocs_mutex_unlock(&driver->lock);
	return ocs;

error3:
	ocs_pci_set_bus_master(ocs, false);
error2:
	ocs_unmap_pci_bar(ocs);
	free(ocs);
error1:
	ocs_mutex_unlock(&driver->lock);
	return NULL;
}

struct ocs_enum_ctx {
	spdk_ocs_probe_cb probe_cb;
	spdk_ocs_attach_cb attach_cb;
	void *cb_ctx;
};

/* This function must only be called while holding g_ocs_driver.lock */
static int
ocs_enum_cb(void *ctx, struct spdk_pci_device *pci_dev)
{
	struct ocs_enum_ctx *enum_ctx = ctx;
	struct spdk_ocs_t 	     *ocs      = NULL;

	/* Verify that this device is not already attached */
	TAILQ_FOREACH(ocs, &g_ocs_driver.attached_chans, tailq) {
		/*
		 * NOTE: This assumes that the PCI abstraction layer will use the same device handle
		 *  across enumerations; we could compare by BDF instead if this is not true.
		 */
		if (pci_dev == ocs->pdev) {
			return 0;
		}
	}

	if ((enum_ctx->probe_cb) && (enum_ctx->probe_cb(enum_ctx->cb_ctx, pci_dev))) {

		ocs = spdk_ocs_attach(pci_dev);
		if (ocs == NULL) {
			ocs_spdk_printf(NULL, "spdk_ocs_attach() failed\n");
			return -1;
		}

		enum_ctx->attach_cb(enum_ctx->cb_ctx, pci_dev, ocs);
	}

	return 0;
}

int
spdk_ocs_probe(void *cb_ctx, spdk_ocs_probe_cb probe_cb, spdk_ocs_attach_cb attach_cb)
{
	int rc;
	struct ocs_enum_ctx enum_ctx;

	enum_ctx.probe_cb = probe_cb;
	enum_ctx.attach_cb = attach_cb;
	enum_ctx.cb_ctx = cb_ctx;

	rc = ocs_pci_enumerate(ocs_enum_cb, &enum_ctx);
	return rc;
}

int
spdk_ocs_detach(struct spdk_ocs_t *ocs)
{
	struct ocs_driver	*driver = &g_ocs_driver;

	/* Clear Bus master enable bit from config space */
	ocs_pci_set_bus_master(ocs, false);

	/* ocs should be in the free list (not registered to a thread)
	 * when calling ocs_detach().
	 */
	ocs_mutex_lock(&driver->lock);
	TAILQ_REMOVE(&driver->attached_chans, ocs, tailq);
	ocs_mutex_unlock(&driver->lock);
	ocs_unmap_pci_bar(ocs);
	spdk_pci_device_detach(ocs->pdev);
	free(ocs);

	return 0;
}

struct spdk_ocs_t *
spdk_ocs_get_object(struct spdk_pci_device *dev)
{
        int i;
        struct spdk_ocs_t *ocs = NULL;
        for (i = 0; i<MAX_OCS_DEVICES; i++) {
                if ((dev != NULL) && (spdk_ocs_devices[i]->pdev == dev)) {
                        ocs = spdk_ocs_devices[i];
                        break;
                }
        }

        return ocs;
}

int spdk_ocs_get_pci_config(struct spdk_pci_device *dev, struct spdk_ocs_get_pci_config_t *pci_config)
{

        struct spdk_ocs_t *ocs = spdk_ocs_get_object(dev);
        if (ocs == NULL) {
                printf("ERR:%s: Failed to find OCS device object\n",__func__);
                return -1;
        }

        pci_config->vendor	= spdk_pci_device_get_vendor_id(dev);
        pci_config->device	= spdk_pci_device_get_device_id(dev);
        pci_config->bus		= spdk_pci_device_get_bus(dev);
        pci_config->dev		= spdk_pci_device_get_dev(dev);
        pci_config->func	= spdk_pci_device_get_func(dev);

        pci_config->bar_count 	= ocs->bar_count;
        memcpy(pci_config->bars, ocs->bars, sizeof(pci_config->bars));

	return 0;
}
