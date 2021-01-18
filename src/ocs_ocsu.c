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
#include "ocs_impl_spdk.h"
#include "ocs_recovery.h"

#include "ocs_spdk.h"
#include "spdk/env.h"
#include "spdk/string.h"
#include "spdk/event.h"
#include "spdk/thread.h"
#include "fc.h"

#include "spdk_nvmf_xport.h"

#if !defined(PCI_DEVICE)
#define PCI_DEVICE(vendor, device) (((uint32_t) vendor << 16) | (device))
#endif

#define MRQ_TOPOLOGY	"eq cq mq cq rq:filter=2 cq wq eq:nvmeq=1 cq rq:filter=0 cq wq %d(eq:nvmeq=1 cq rq:filter=1:rqpolicy=3 cq wq)"

#ifndef PCI_PRI_FMT /* This is defined by rte_pci.h when SPDK_CONFIG_PCIACCESS is not set */
#define PCI_PRI_FMT		"%04x:%02x:%02x.%1u"
#endif

#define SYSFS_PCI_DEVICES	"/sys/bus/pci/devices"
#define SPDK_PCI_PATH_MAX	256

struct ocs_spdk_device {
	struct spdk_pci_device  *spdk_pci_dev;
	struct spdk_ocs_t	*ocs;
	TAILQ_ENTRY(ocs_spdk_device) tailq;
};

static TAILQ_HEAD(, ocs_spdk_device)g_devices;

/* This will be set by ocsu_init() */
uint32_t ocs_spdk_master_core = 0;
struct spdk_thread *g_spdk_master_thread;
ocs_thread_t ocsu_shutdown_thr;

uint16_t
ocs_sriov_get_max_nr_vfs(ocs_t *ocs)
{
	return 0;
}

int32_t
ocs_sriov_enable_vfs(ocs_t *ocs)
{
	return -1;
}

extern spdk_nvmf_transport_destroy_done_cb g_transport_destroy_done_cb_fn;
extern void *g_transport_destroy_done_cb_arg;

int32_t
ocsu_spdk_shutdown_thread(ocs_thread_t *mythread)
{
	ocsu_exit();

	if (g_transport_destroy_done_cb_fn) {
		ocs_log_info(NULL, "Notify transport destroy done\n");
		spdk_thread_send_msg(g_spdk_master_thread, g_transport_destroy_done_cb_fn,
				g_transport_destroy_done_cb_arg);
	}
	return 0;
}

#define OCS_SPDK_WORKER_EVENT_POLL_TIME_USEC 100
int32_t
ocsu_spdk_worker_thread(ocs_thread_t *mythread)
{
	ocs_t *ocs = ocs_thread_get_arg(mythread);
	ocs_mqueue_t *msg_q = &ocs->tgt_ocs.worker_msg_q;
	ocs_spdk_worker_q_msg_t *msg;
	int32_t done = false;
	static int32_t poll_events = false;

	ocs_log_debug(ocs, "%s started\n", mythread->name);
	while (!done) {
		msg = ocs_mqueue_get(msg_q, OCS_SPDK_WORKER_EVENT_POLL_TIME_USEC);
		if (ocs_thread_terminate_requested(mythread)) {
			break;
		}

		if (!msg && poll_events)
			ocsu_process_events(ocs);

		if (!msg)
			continue;

		switch (msg->msg) {
		case OCS_SPDK_WORKER_START_EVENT_PROCESS:
			poll_events = true;
			break;
		case OCS_SPDK_WORKER_STOP_EVENT_PROCESS:
			poll_events = false;
			break;

		case OCS_SPDK_WORKER_THREAD_EXIT:
			done = true;
			break;
		case OCS_SPDK_WORKER_NODE_POST_EVENT: 
			msg->func(msg->func_arg);
			break;
		}

		if (msg->sync)
			ocs_sem_v(&msg->sync_sem);
		else
			ocs_free(ocs, msg, sizeof(*msg));
	}

	ocs_log_debug(ocs, "%s terminated\n", mythread->name);

	return 0;
}

int
ocs_send_msg_to_worker(ocs_t *ocs, ocs_spdk_worker_msg_t m, bool sync,
		       ocs_spdk_worker_func func, void *func_arg)
{
	ocs_mqueue_t *msg_q = &ocs->tgt_ocs.worker_msg_q;
	ocs_spdk_worker_q_msg_t *msg;
	int rc = 0;
	
	if (!sync && !func) {
		return -1;
	}

	msg = ocs_malloc(ocs, sizeof(*msg), OCS_M_ZERO);
	if (!msg) {
		ocs_log_err(ocs, "NOMEM failure\n");
		return -1;
	}

	msg->msg = m;
	if (sync) {
		msg->sync = sync;
		ocs_sem_init(&msg->sync_sem, 0, "sync_sem_%d", m);
	} else {
        	msg->func = func;
        	msg->func_arg = func_arg;
	}

	ocs_mqueue_put(msg_q, msg);

	if (sync) {
		rc = ocs_sem_p(&msg->sync_sem, 10000000);
		if (rc) {
			ocs_log_err(ocs, "sema wait timed out\n");
		}

		ocs_free(ocs, msg, sizeof(*msg));
	}

	return rc;
}

inline static int
ocs_device_create_worker_thread(ocs_t *ocs)
{
	ocs_mqueue_init(ocs, &ocs->tgt_ocs.worker_msg_q);
	return ocs_thread_create(ocs, &ocs->tgt_ocs.worker_thr, ocsu_spdk_worker_thread,
			"ocsu_spdk_worker", ocs, OCS_THREAD_RUN);

}

void
ocs_device_delete_worker_thread(ocs_t *ocs)
{
	if (ocs_send_msg_to_worker(ocs, OCS_SPDK_WORKER_THREAD_EXIT,
			true, NULL, NULL)) {
		ocs_log_err(ocs, "Failed to terminate worker thread\n");
	}
	
	ocs_mqueue_free(&ocs->tgt_ocs.worker_msg_q);

}

static int
ocs_is_vfio_driver_loaded(struct spdk_pci_device *dev)
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

	return (strcmp(driver_begin, "vfio-pci") != 0);
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
	ocs_log_debug(NULL, "Found matching device at %d:%d:%d "
		"vendor:0x%04x device:0x%04x\n",
		spdk_pci_device_get_bus(pci_dev), spdk_pci_device_get_dev(pci_dev),
		spdk_pci_device_get_func(pci_dev),
		spdk_pci_device_get_vendor_id(pci_dev), spdk_pci_device_get_device_id(pci_dev));

	if (ocs_is_vfio_driver_loaded(pci_dev)) {
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

int
ocs_spdk_poller_stop(ocs_t *ocs)
{
	int rc;
	rc = ocs_send_msg_to_worker(ocs, OCS_SPDK_WORKER_STOP_EVENT_PROCESS, 
				true, NULL, NULL);

	ocs_log_debug(ocs, "SPDK Event processing %s on Port: %d\n",
		      rc ? "failed to disable" : "disabled", ocs_instance(ocs)); 

	return rc;
}

int
ocs_spdk_poller_start(ocs_t *ocs)
{
	int rc;
	rc = ocs_send_msg_to_worker(ocs, OCS_SPDK_WORKER_START_EVENT_PROCESS, 
				true, NULL, NULL);

	ocs_log_debug(ocs, "SPDK Event processing %s on Port: %d\n",
		      rc ? "failed to enable" : "enabled", ocs_instance(ocs)); 

	return rc;
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

		rc = ocs_spdk_poller_start(ocs);
		if (rc) {
			ocs_log_err(ocs, "Unable to start worker poller on port %d\n",
				    ocs->instance_index);
		}
	}
}

static ocs_t*
ocsu_device_init(struct spdk_pci_device *pci_dev)
{
	ocs_t *ocs;
	int32_t rc = -1, num_interrupts;
	uint32_t num_cores = 0;
	const char *desc = "Unknown adapter";
	struct spdk_ocs_get_pci_config_t pciconfig;

	num_cores = spdk_env_get_core_count();
	if (num_cores > OCS_NVME_FC_MAX_IO_QUEUES) {
		ocs_log_err(NULL, "Cant configure more cores than %d for FC\n",
				OCS_NVME_FC_MAX_IO_QUEUES);
		return NULL;
	}

	ocs = ocs_device_alloc(0);
	if (ocs == NULL) {
		ocs_log_err(NULL, "ocsu_device_init failed\n");
		return NULL;
	}

	ocs->ocs_os.num_cores = num_cores;

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
		case PCI_DEVICE(PCI_VENDOR_EMULEX, PCI_PRODUCT_EMULEX_LANCER_G7_FC):
			desc = "Emulex LightPulse G7 32Gb FC Adapter";
			break;
		default:
		{
			ocs_log_err(NULL, "Unsupported device found.");
			ocs_free(ocs, ocs, sizeof(*ocs));
			return NULL;
		}
	}

	ocs->desc	= desc;
	ocs->ocs_os.spdk_pdev	= pci_dev;
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
	ocs->enable_ini = initiator;
	ocs->enable_tgt = target;
	ocs_snprintf(ocs->ocs_os.queue_topology, sizeof(ocs->ocs_os.queue_topology),
		     MRQ_TOPOLOGY, num_cores);
	hal_global.queue_topology_string = ocs->ocs_os.queue_topology;

	num_interrupts = ocs_device_interrupts_required(ocs);
	if (num_interrupts < 0) {
		ocs_log_err(ocs, "ocs_device_interrupts_required() failed\n");
		goto error1;
	}

	ocs_log_info(ocs, "Found Adapter Port: %d\n",
		ocs->hal.sli.physical_port);
	ocs_log_info(ocs, "Description  : %.64s \n",
		strlen(ocs->hal.sli.config.modeldesc) ?
		ocs->hal.sli.config.modeldesc : "N/A");

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
		ocs_log_err(ocs, "ocs_device_attach failed: %d\n", rc);
		goto error2;
	}

	rc = ocs_thread_create(ocs, &ocs->drv_ocs.ioctl_thr, ocs_ioctl_server,
			"ocs_ioctl_thr", ocs, OCS_THREAD_RUN);
	if (rc) {
		ocs_log_err(ocs, "ocs ioctl thread create failed! rc=%d\n", rc);
		goto error3;
	}

	rc = ocs_device_create_worker_thread(ocs);
	if (rc) {
		ocs_log_err(ocs, "ocsu worker thread create failed\n");
		goto error4;
	}

	return ocs;

error4:
	ocs_thread_terminate(&ocs->drv_ocs.ioctl_thr);
error3:
	ocs_device_detach(ocs);
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
	int32_t rc;
	struct ocs_spdk_device *ocs_spdk_device;

	/* Save master core value */
	ocs_spdk_master_core = spdk_env_get_current_core();
	g_spdk_master_thread = spdk_get_thread();

	TAILQ_INIT(&g_devices);

	if (spdk_ocs_probe(NULL, probe_cb, attach_cb) != 0) {
		fprintf(stderr, "ocs_probe() failed\n");
		return -1;
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
			ocs_log_err(ocs, "ocsu_device_init failed\n");
			return -1;
		}
	}

	rc  = ocs_fw_err_recovery_setup();

	return rc;
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
		ocs_log_err(ocs, "PCI device not found.\n");
		return false;
	}

	ocs_device_detach(ocs);

	TAILQ_FOREACH(dev, &g_devices, tailq) {
		if (dev->spdk_pci_dev == ocs->ocs_os.spdk_pdev) {
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
ocsu_shutdown(void)
{
	int rc;

	ocs_log_info(NULL, "OCSU initiate shutdown process\n");
	rc = ocs_thread_create(NULL, &ocsu_shutdown_thr,
			ocsu_spdk_shutdown_thread,
			"ocsu_spdk_shdutdown", NULL, OCS_THREAD_RUN);
	if (rc) {
		ocs_log_err(NULL, "Failed to initiate shutdown process\n");
	}
}

void
ocsu_exit(void)
{
	uint32_t i;
	struct ocs_spdk_device *dev;
	ocs_t *ocs;

	for_each_ocs(i, ocs) {
		if (!ocs)
			continue;

		ocs_device_detach(ocs);
		ocs_nvme_hw_port_free(ocs);
	}

	ocs_fw_err_recovery_stop();	

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
		if (!ocs->hal.hal_eq[i]->nvmeq) {
			ocs_hal_process(&ocs->hal, i, OCS_OS_MAX_ISR_TIME_MSEC);
		}
	}

	return 0;
}

int32_t
ocs_start_event_processing(ocs_os_t *ocs_os)
{
	int rc;
	ocs_t *ocs = (ocs_t *) ocs_os;

	rc = ocs_spdk_poller_start(ocs);
	if (rc) {
		ocs_log_err(ocs, "Unable to start worker poller on port %d\n",
			    ocs->instance_index);
		return rc;
	}

	rc = ocs_nvme_tgt_new_device(ocs);
	if (rc) {
		ocs_log_err(ocs, "Restart after recovery failed. nvme_hw_port_create failed \n");
	}

	return rc;
}

void
ocs_stop_event_processing(ocs_os_t *ocs_os)
{
	int rc;

	ocs_t *ocs = (ocs_t *)ocs_os;
	rc = ocs_spdk_poller_stop(ocs);
	if (rc) {
		ocs_log_err(ocs, "Unable to STOP event processing\n");
	}

	return;
}

static struct spdk_pci_id ocs_pci_driver_id[] = {
	{ .class_id = SPDK_PCI_CLASS_ANY_ID,
	 .vendor_id = SPDK_PCI_VID_OCS,
	 .device_id = PCI_DEVICE_ID_OCS_LANCERG5,
	 .subvendor_id = SPDK_PCI_ANY_ID,
	 .subdevice_id = SPDK_PCI_ANY_ID,
	},
	{ .class_id = SPDK_PCI_CLASS_ANY_ID,
	 .vendor_id = SPDK_PCI_VID_OCS,
	 .device_id = PCI_DEVICE_ID_OCS_LANCERG6,
	 .subvendor_id = SPDK_PCI_ANY_ID,
	 .subdevice_id = SPDK_PCI_ANY_ID,
	},
	{ .class_id = SPDK_PCI_CLASS_ANY_ID,
	 .vendor_id = SPDK_PCI_VID_OCS,
	 .device_id = PCI_DEVICE_ID_OCS_LANCERG7,
	 .subvendor_id = SPDK_PCI_ANY_ID,
	 .subdevice_id = SPDK_PCI_ANY_ID,
	},
};

SPDK_PCI_DRIVER_REGISTER(vfio_pci, ocs_pci_driver_id,
			SPDK_PCI_DRIVER_NEED_MAPPING | SPDK_PCI_DRIVER_WC_ACTIVATE);

