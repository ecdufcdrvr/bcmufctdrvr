/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
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

/**
 * @file
 * Common OCS userspace driver API
 *
 */

#include "ocs.h"
#include "ocs_ocsu.h"
#include "ocs_driver.h"
#include "ocs_params.h"
#include "ocs_impl.h"

#include "ocs_spdk.h"
#include "spdk/env.h"
#include "spdk/string.h"
#include "spdk/event.h"
#include "spdk/io_channel.h"
#include "fc.h"

#include "spdk_nvmf_xport.h"

#if !defined(PCI_DEVICE)
#define PCI_DEVICE(vendor, device) (((uint32_t) vendor << 16) | (device))
#endif


#ifndef PCI_PRI_FMT /* This is defined by rte_pci.h when SPDK_CONFIG_PCIACCESS is not set */
#define PCI_PRI_FMT		"%04x:%02x:%02x.%1u"
#endif

#define SYSFS_PCI_DEVICES	"/sys/bus/pci/devices"
#define SPDK_PCI_PATH_MAX	256
#define MRQ_TOPOLOGY    "eq cq mq cq rq:filter=2 cq wq eq cq rq:filter=0 cq wq %d(eq cq rq:filter=1:rqpolicy=3 cq wq)"

struct ocs_spdk_device
{
	struct spdk_pci_device  *spdk_pci_dev;
	struct spdk_ocs_t	*ocs;
	TAILQ_ENTRY(ocs_spdk_device) tailq;
};

struct ocs_spdk_fc_poller
{
	struct spdk_poller *spdk_poller;
	uint32_t lcore;
};

static TAILQ_HEAD(, ocs_spdk_device)g_devices;

static struct ocs_spdk_fc_poller g_fc_port_poller[MAX_OCS_DEVICES][OCS_HAL_MAX_NUM_EQ];

static uint32_t g_fc_lcore[RTE_MAX_LCORE];

ocs_t* ocsu_device_init(struct spdk_pci_device *pci_dev);

static int
ocs_is_uio_driver_loaded(struct spdk_pci_device *dev)
{
	char linkname[SPDK_PCI_PATH_MAX];
	char driver[SPDK_PCI_PATH_MAX];
	ssize_t driver_len;
	char *driver_begin;

	snprintf(linkname, sizeof(linkname),
		SYSFS_PCI_DEVICES "/" PCI_PRI_FMT "/driver",
		spdk_pci_device_get_domain(dev), spdk_pci_device_get_bus(dev),
		spdk_pci_device_get_dev(dev), spdk_pci_device_get_func(dev));

	driver_len = readlink(linkname, driver, sizeof(driver));

	if (driver_len < 0 || driver_len >= SPDK_PCI_PATH_MAX) {
		return 1;
	}

	driver[driver_len] = '\0'; /* readlink() doesn't null terminate, so we have to */

	driver_begin = strrchr(driver, '/');
	if (driver_begin) {
		/* Advance to the character after the slash */
		driver_begin++;
	} else {
		/* This shouldn't normally happen - driver should be a relative path with slashes */
		driver_begin = driver;
	}

	return (strncmp(driver_begin, "uio_", 4) != 0 &&
		strcmp(driver_begin, "vfio-pci") != 0);
}

/**
 * @brief Initialize OCS userspace driver framework
 *
 * Called by the application to initialize the OCS driver framework.  Each of
 * the available /dev/ocs_* device special files is opened.  The device BARS
 * are mapped into the process' address space.  The OCS driver API calls are
 * made to complete initialization and bring up each device.
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static bool
probe_cb(void *cb_ctx, struct spdk_pci_device *pci_dev)
{
	ocs_log_debug(NULL, " Found matching device at %d:%d:%d "
		"vendor:0x%04x device:0x%04x\n",
		spdk_pci_device_get_bus(pci_dev), spdk_pci_device_get_dev(pci_dev),
		spdk_pci_device_get_func(pci_dev),
		spdk_pci_device_get_vendor_id(pci_dev), spdk_pci_device_get_device_id(pci_dev));

	if (ocs_is_uio_driver_loaded(pci_dev)) {
		ocs_log_err(NULL, "Device has no UIO driver loaded, skipping...\n");
		return false;
	}

	return true;
}

static void
attach_cb(void *cb_ctx, struct spdk_pci_device *pci_dev, struct spdk_ocs_t *spdk_ocs)
{
	struct ocs_spdk_device *dev;

	dev = calloc(1, sizeof(*dev));
	if (dev == NULL) {
		ocs_log_err(NULL, "Failed to allocate device struct\n");
		return;
	}

	dev->spdk_pci_dev = pci_dev;
	dev->ocs	  = spdk_ocs;
	TAILQ_INSERT_TAIL(&g_devices, dev, tailq);
}


/* Dont assign master lcore and nvmf lcores. */
static uint32_t
ocs_alloc_lcore(void)
{
	return spdk_env_get_last_core();

#if 0 // TODO: NEW SPDK - FIX the lcore mask skip (if needed still)
	uint32_t least_assgined_lcore = 0;
	uint32_t least_assigned = 0, i = 0;

	RTE_LCORE_FOREACH_SLAVE(i) {
		/* If this is nvmf lcore skip */
		if ((g_nvmf_tgt.opts.lcore_mask >> i) & 0x1) {
			continue;
		}

		if (!least_assigned || (g_fc_lcore[i] < least_assigned)) {
			least_assigned = g_fc_lcore[i];
			least_assgined_lcore = i;
		}
	}

	if (!least_assgined_lcore) {
		/* This means no lcore availble. */
		return UINT32_MAX;
	}

	g_fc_lcore[least_assgined_lcore]++;

	return least_assgined_lcore;
#endif
}

static void
ocs_release_lcore(uint32_t lcore)
{
	g_fc_lcore[lcore]--;
}

int
ocs_spdk_fc_poller(void *arg)
{
	hal_eq_t *hal_eq = arg;
	ocs_hal_t *hal	= hal_eq->hal;

	ocs_hal_process(hal, hal_eq->instance, OCS_OS_MAX_ISR_TIME_MSEC);

	return 0;
}

static void
_ocs_poller_stop(void *arg1, void *arg2)
{
	struct ocs_spdk_fc_poller *fc_poller = arg1;

	spdk_poller_unregister(&fc_poller->spdk_poller);
	ocs_release_lcore(fc_poller->lcore);
	sem_post((sem_t *) arg2);
}

static void
ocs_poller_stop(struct ocs_spdk_fc_poller *fc_poller)
{
	struct spdk_event *event = NULL;
	sem_t sem;

	sem_init(&sem, 1, 0);
	event = spdk_event_allocate(fc_poller->lcore, _ocs_poller_stop,
				    (void *)fc_poller, (void *)&sem);
	if (event) {
		spdk_event_call(event);
		sem_wait(&sem);
	}
	sem_destroy(&sem);
}

void
ocs_spdk_poller_stop(ocs_t *ocs)
{
	if (ocs != NULL) {
		uint32_t i, index = ocs_instance(ocs);
		uint32_t pollers_required = ocs->hal.config.n_eq - (ocs->num_cores + 1);

		ocs_log_debug(ocs, "Destroying poller threads on port : %d\n", index);
		for (i = 0; i < pollers_required; i++) {
			ocs_poller_stop(&g_fc_port_poller[index][i]);
		}
	}
}

static struct spdk_thread *ocs_rsvd_thread = NULL;

struct spdk_thread *
ocs_get_rsvd_thread(void)
{
	return ocs_rsvd_thread;
}

static void
ocs_delay_poller_start(void *arg1, void *arg2)
{
	struct ocs_spdk_fc_poller *poller = arg1;
	poller->spdk_poller = spdk_poller_register(ocs_spdk_fc_poller, arg2, 0);
	if (ocs_rsvd_thread == NULL &&
	    spdk_env_get_current_core() == spdk_env_get_last_core()) {
		/* save last thread (reserved for SCSI) */
		ocs_rsvd_thread = spdk_get_thread();
	}
}

static int
ocs_create_pollers(ocs_t *ocs)
{
	uint32_t index = 0, i, lcore_id;
	uint32_t pollers_required = ocs->hal.config.n_eq - (ocs->num_cores + 1);
	struct spdk_event *event = NULL;

	for (i = 0; i < pollers_required; i++) {
		lcore_id = ocs_alloc_lcore();
		if (lcore_id == UINT32_MAX) {
			return -1;
		}

		ocs_log_debug(ocs, "Starting polling thread on Port: %d core: %d\n",
			ocs_instance(ocs), lcore_id);

		g_fc_port_poller[ocs_instance(ocs)][index].lcore = lcore_id;
		event = spdk_event_allocate(lcore_id, ocs_delay_poller_start,
	 				    (void *)&g_fc_port_poller[ocs_instance(ocs)][index],
					    ocs->hal.hal_eq[index]);
		spdk_event_call(event);

		index++;

		ocs->lcore_mask |= (1 << lcore_id);
	}
	return 0;
}

ocs_t*
ocsu_device_init(struct spdk_pci_device *pci_dev)
{
	ocs_t *ocs;
	int32_t rc = -1, num_interrupts;
	uint32_t num_cores = 0;
	const char *desc = "Unknown adapter";
	struct spdk_ocs_get_pci_config_t pciconfig;
	struct spdk_fc_hba_port *hba_port = NULL;

	num_cores = spdk_env_get_core_count();
	if (num_cores > OCS_NVME_FC_MAX_IO_QUEUES) {
		ocs_log_err(NULL, "%s: Cant configure more cores than %d for FC\n",
			 __func__, OCS_NVME_FC_MAX_IO_QUEUES);
		return NULL;
	}

	ocs = ocs_device_alloc(0);
	if (ocs == NULL) {
		ocs_log_err(NULL, "ocsu_device_init failed\n");
		return NULL;
	}
	ocs->num_cores = num_cores;

	spdk_ocs_get_pci_config(pci_dev, &pciconfig);

	ocs->pci_vendor = pciconfig.vendor;
	ocs->pci_device = pciconfig.device;

	switch (PCI_DEVICE(ocs->pci_vendor, ocs->pci_device)) {
	    case PCI_DEVICE(PCI_VENDOR_EMULEX, PCI_PRODUCT_EMULEX_OCE16001):
		    desc = "Emulex LightPulse G5 16Gb FC Adapter";
		    break;
	    case PCI_DEVICE(PCI_VENDOR_EMULEX, PCI_PRODUCT_EMULEX_LPE31004):
		    desc = "Emulex LightPulse G6 16Gb FC Adapter";
		    break;
	    default:
	    {
		    ocs_log_err(NULL, "Unsupported device found.");
		    ocs_free(ocs, ocs, sizeof(*ocs));
		    return NULL;
	    }
	}

	ocs->desc	= desc;
	ocs->ocsu_spdk	= pci_dev;
	ocs->ocs_os.pagesize 	= sysconf(_SC_PAGE_SIZE);
	ocs->ocs_os.bus		= pciconfig.bus;
	ocs->ocs_os.dev		= pciconfig.dev;
	ocs->ocs_os.func	= pciconfig.func;
	ocs->ocs_os.bar_count	= pciconfig.bar_count;
	memcpy(&ocs->ocs_os.bars, pciconfig.bars, sizeof(ocs->ocs_os.bars));
	sprintf(ocs->businfo, "%02x:%02x.%x", ocs->ocs_os.bus, ocs->ocs_os.dev,
		ocs->ocs_os.func);

	ocs_log_info(ocs, "%s: %s driver version %s bar count: %d \n", DRV_NAME,
		ocs->desc, DRV_VERSION, pciconfig.bar_count);

	/* initialize DMA buffer allocation */
	ocs_dma_init(ocs);

	/* Initialize per ocs queue topology */
	hba_port = ocs_spdk_tgt_find_hba_port(ocs->instance_index);
	if (hba_port) {
		ocs->enable_ini = hba_port->initiator;
		ocs->enable_tgt = hba_port->target;
		ocs_snprintf(ocs->queue_topology, sizeof(ocs->queue_topology),
			MRQ_TOPOLOGY, num_cores);
	} else {
		// use default
		ocs->enable_ini = initiator;
		ocs->enable_tgt = target;
		ocs_snprintf(ocs->queue_topology, sizeof(ocs->queue_topology),
			MRQ_TOPOLOGY, num_cores);
	}

	// For now always enable.
	ocs->enable_nvme_tgt = TRUE;

	num_interrupts = ocs_device_interrupts_required(ocs);
	if (num_interrupts < 0) {
		ocs_log_err(ocs, "%s: ocs_device_interrupts_required() failed\n", __func__);
		goto error1;
	}

	ocs_log_info(ocs, "Found Adapter Port: %d\n",
		ocs->hal.sli.physical_port);
	ocs_log_info(ocs, "Description  : %.64s \n",
		strlen(ocs->hal.sli.config.description) ?
		ocs->hal.sli.config.description : "N/A");
	ocs_log_info(ocs, "Model        : %.32s \n",
		strlen(ocs->hal.sli.config.model) ?
		ocs->hal.sli.config.model : "N/A");
	ocs_log_info(ocs, "SN           : %.32s \n",
		strlen(ocs->hal.sli.config.serial_number) ?
		ocs->hal.sli.config.serial_number : "N/A");

	ocs_log_info(ocs, "WWPN         : %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
		ocs->hal.sli.config.wwpn[0],
		ocs->hal.sli.config.wwpn[1],
		ocs->hal.sli.config.wwpn[2],
		ocs->hal.sli.config.wwpn[3],
		ocs->hal.sli.config.wwpn[4],
		ocs->hal.sli.config.wwpn[5],
		ocs->hal.sli.config.wwpn[6],
		ocs->hal.sli.config.wwpn[7]);

	ocs_log_info(ocs, "WWNN         : %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n\n",
		ocs->hal.sli.config.wwnn[0],
		ocs->hal.sli.config.wwnn[1],
		ocs->hal.sli.config.wwnn[2],
		ocs->hal.sli.config.wwnn[3],
		ocs->hal.sli.config.wwnn[4],
		ocs->hal.sli.config.wwnn[5],
		ocs->hal.sli.config.wwnn[6],
		ocs->hal.sli.config.wwnn[7]);

	rc = ocs_device_attach(ocs);
	if (rc) {
		ocs_log_err(ocs, "%s: ocs_device_attach failed: %d\n", __func__, rc);
		goto error2;
	}
	return ocs;

error2:
	ocs_dma_teardown(ocs);
error1:
	ocs_device_free(ocs);
	return NULL;
}

int
ocsu_init(void)
{
	ocs_t *ocs;
	int32_t rc = -1;
	struct ocs_spdk_device *ocs_spdk_device;

	TAILQ_INIT(&g_devices);

	if (spdk_ocs_probe(NULL, probe_cb, attach_cb) != 0) {
		fprintf(stderr, "ocs_probe() failed\n");
		return rc;
	}

	rc = ocs_device_init();
	if (rc) {
		return rc;
	}

	/* Open every ocsu_spdk_pci_devices[] */
	TAILQ_FOREACH(ocs_spdk_device, &g_devices, tailq) {
		if (!ocs_spdk_device->spdk_pci_dev) {
			continue;
		}

		ocs = ocsu_device_init(ocs_spdk_device->spdk_pci_dev);
		if (ocs == NULL) {
			ocs_log_err(ocs, "%s: ocsu_device_init failed\n", __func__);
			return -1;
		}
	}
	return rc;
}

bool
ocsu_device_add(struct spdk_pci_addr *pci_addr)
{
	uint32_t i;
	ocs_t *ocs;
	struct ocs_spdk_device 	*dev;
	struct spdk_ocs_t 	*spdk_ocs;
	struct spdk_pci_device 	*pci_dev = NULL;
	uint16_t 		vendor_id, device_id;

	if (!pci_addr) {
		ocs_log_err(NULL, "%s: Invalid pci_addr\n", __func__)
		goto error1;
	}

	// Make sure the device doesnt exist
	for_each_ocs(i, ocs) {
		if (!ocs)
			continue;

		if ((ocs->ocs_os.bus == pci_addr->bus) &&
		    (ocs->ocs_os.dev == pci_addr->dev) &&
		    (ocs->ocs_os.func == pci_addr->func))
		{
			break;
		}
	}

	if (ocs) {
		ocs_log_err(ocs, "%s: Already exists.\n", __func__)
		goto error1;
	}

	/* TODO: No equivalent call in 18.04. For now disable this. */
	// pci_dev = spdk_pci_get_device(pci_addr);
	if (!pci_dev) {
		ocs_log_err(ocs, "%s: PCI device not found.\n", __func__)
		goto error1;
	}

	vendor_id = spdk_pci_device_get_vendor_id(pci_dev);
	device_id = spdk_pci_device_get_device_id(pci_dev);

	if (!ocs_pci_device_match_id(vendor_id, device_id)) {
		ocs_log_err(ocs, "%s: Unknown PCI device.\n", __func__)
		goto error1;
	}

	spdk_ocs = spdk_ocs_attach(pci_dev);
	if (spdk_ocs == NULL) {
		ocs_log_err(ocs, "%s: spdk_ocs failed to attach.\n", __func__)
		goto error1;
	}

	if (!probe_cb(NULL, pci_dev)) {
		ocs_log_err(ocs, "%s: probe failed.\n", __func__)
		goto error2;
	}

	attach_cb(NULL, pci_dev, spdk_ocs);

	ocs = ocsu_device_init(pci_dev);
	if (!ocs) {
		ocs_log_err(ocs, "%s: ocsu_device_init failed. \n", __func__)
		goto error3;
	}

	return true;

error3:
	TAILQ_FOREACH(dev, &g_devices, tailq) {
		if (dev->spdk_pci_dev == pci_dev) {
			TAILQ_REMOVE(&g_devices, dev, tailq);
			free(dev);
			break;
		}
	}
error2:
	spdk_ocs_detach(spdk_ocs);
error1:
	return false;
}

bool
ocsu_device_remove(struct spdk_pci_addr *pci_addr)
{
	uint32_t i;
	ocs_t *ocs;
	struct ocs_spdk_device *dev;


	if (!pci_addr) {
		return false;
	}

	// Find the device that need to unplugged
	for_each_ocs(i, ocs) {
		if (!ocs)
			continue;

		if ((ocs->ocs_os.bus == pci_addr->bus) &&
		    (ocs->ocs_os.dev == pci_addr->dev) &&
		    (ocs->ocs_os.func == pci_addr->func))
		{
			break;
		}
	}

	if (!ocs) {
		ocs_log_err(ocs, "%s: PCI device not found.\n", __func__)
		return false;
	}

	ocs_device_detach(ocs);

	TAILQ_FOREACH(dev, &g_devices, tailq) {
		if (dev->spdk_pci_dev == ocs->ocsu_spdk) {
			TAILQ_REMOVE(&g_devices, dev, tailq);
			if (dev->ocs) {
				spdk_ocs_detach(dev->ocs);
			}
			free(dev);
			break;
		}
	}

	ocs_device_free(ocs);
	return true;
}

void
ocs_spdk_start_pollers(void)
{
	uint32_t i;
	int rc = 0;
	ocs_t *ocs;

	for_each_ocs(i, ocs) {
		if (!ocs)
			continue;
		rc = ocs_create_pollers(ocs);
		if (rc) {
			ocs_log_err(ocs, "%d: unable to start pollers\n", ocs->instance_index);
		}
	}
}

void
ocs_spdk_exit(void)
{
	uint32_t i;
	struct ocs_spdk_device *dev;
	ocs_t *ocs;

	for_each_ocs(i, ocs) {
		if (!ocs)
			continue;

		ocs_device_detach(ocs);
	}

	while (!TAILQ_EMPTY(&g_devices)) {
		dev = TAILQ_FIRST(&g_devices);
		TAILQ_REMOVE(&g_devices, dev, tailq);
		if (dev->ocs) {
			spdk_ocs_detach(dev->ocs);
		}
		free(dev);
	}
	/* Perform cleanup of user space DMA and device free in this second
	 * loop, as some back ends tie references to DMA objects to one of the
	 * device objects, and the previous loop might result in a use after
	 * free error.
	 */
	for_each_ocs(i, ocs) {
		if (!ocs)
			continue;
		ocs_device_free(ocs);
	}

	ocs_device_shutdown_complete();
}

/**
 * @brief Process pending events
 *
 * The devices for 'ocs' are processed. May be used in a polled environment
 *
 * @param ocs pointer to device instance
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocsu_process_events(ocs_t *ocs)
{
	uint32_t i;
	uint32_t eq_count = ocs_hal_get_num_eq(&ocs->hal);

	for (i = 0; i < eq_count; i++) {
		ocs_hal_process(&ocs->hal, i, OCS_OS_MAX_ISR_TIME_MSEC);
	}

	return 0;
}
