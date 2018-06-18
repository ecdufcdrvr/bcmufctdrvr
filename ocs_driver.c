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

#include "ocs.h"
#include "ocs_elxu.h"
#include "ocs_driver.h"
#include "ocs_vpd.h"
#include "ocs_params.h"
#include "ocs_gendump.h"
#include "ocs_ocsu.h"

#include "fc.h"

#include "spdk/log.h"

static ocs_t *ocs_devices[MAX_OCS_DEVICES];
static uint32_t num_ocs_devices;
static void ocs_driver_print_module_params(void);

/* PARAM_LIST: generate parameter declarations with initializers */
#define P(type, name, value, desc)	type name = value;
PARAM_LIST
#undef P

/* ocs_ini_get_initiators - Get the initiator port pointers (ocs_t's) from the FC driver */
/* Returns number nodes available. If number is larger than value passed in for node_size */
/* this function should be called again with enough nodes for the returned node size */
uint32_t ocsu_ini_get_initiators(ocs_t *ini_ocs_port[],  /* array of pointers to ocs_t's */
	uint32_t max_ports) /* max. number of ocs_t ptrs. that can be returned in ini_ocs_port[] */
{
	uint32_t i;

	SPDK_NOTICELOG(">> ocsu_ini_get_initiators() CALLED\n");
	if (max_ports < num_ocs_devices)
		return num_ocs_devices;

	for (i = 0; i < num_ocs_devices; i++)
		ini_ocs_port[i] = ocs_devices[i];

	return i;
}

int32_t
ocs_device_init(void)
{
	int32_t rc;

	/* driver-wide init for target-server */
	if (target) {
		rc = ocs_scsi_tgt_driver_init();
		if (rc) {
			return rc;
		}
	}

	/* driver-wide init for initiator-client */
	if (initiator) {
		rc = ocs_scsi_ini_driver_init();
		if (rc) {
			return rc;
		}
	}

	/* Print module Parameters */
	ocs_driver_print_module_params();

	return 0;
}

/*
 * @brief allocate ocs device
 *
 * @param nid Numa node ID
 *
 * @return returns pointer to allocated OCS structure
 */
void*
ocs_device_alloc(uint32_t nid)
{
	ocs_t *ocs = NULL;
	uint32_t i;

	ocs = ocs_malloc(NULL, sizeof(*ocs), OCS_M_NOWAIT | OCS_M_ZERO);
	if (ocs == NULL) {
		ocs_log_err(ocs, "%s: memory allocation failed for ocs_t\n", __func__);
		return NULL;
	}

	for (i = 0; i < ARRAY_SIZE(ocs_devices); i++) {
		if (ocs_devices[i] == NULL) {
			ocs->instance_index = i;
			ocs_devices[ocs->instance_index] = ocs;
			break;
		}
	}

	if (i == ARRAY_SIZE(ocs_devices)) {
		ocs_log_err(NULL, "%s failed\n", __func__);
		ocs_free(ocs, ocs, sizeof(*ocs));
		ocs = NULL;
		return ocs;
	}

	/* If enabled, initailize a RAM logging buffer */
	if (logdest & 2) {
		ocs->ramlog = ocs_ramlog_init(ocs, ramlog_size / OCS_RAMLOG_DEFAULT_BUFFERS,
			OCS_RAMLOG_DEFAULT_BUFFERS);
		/* If NULL was returned, then we'll simply skip using the ramlog but */
		/* set logdest to 1 to ensure that we at least get default logging.  */
		if (ocs->ramlog == NULL) {
			logdest = 1;
		}
	}

	/* initialize a saved ddump */
	if (ddump_saved_size) {
		if (ocs_textbuf_alloc(ocs, &ocs->ddump_saved, ddump_saved_size)) {
			ocs_log_err(ocs, "%s: failed to allocate memory for saved ddump\n", __func__);
		}
	}
	ocs_list_init(&ocs->domain_list, ocs_domain_t, link);
//TODO:		ocs->ocs_os.interrupt_cb = ocs_device_interrupt_handler;

#if defined(ENABLE_LOCK_DEBUG)
	ocs_lock_init(NULL, &ocs->ocs_os.locklist_lock, "locklist_lock[%d]", ocs->instance_index);
	ocs_list_init(&ocs->ocs_os.locklist, ocs_lock_t, link);
#endif

	ocs_device_lock_init(ocs);
	ocs->drv_ocs.attached = FALSE;

	/* Save userspace mask value */
	ocs->drv_ocs.uspace_thread_per_port = (uspacemask & (1U << 0)) == 0;
	ocs->drv_ocs.uspace_threads_block = (uspacemask & (1U << 1)) == 0;
	num_ocs_devices++;
	return ocs;
}

/**
 * @brief free ocs device
 *
 * @param ocs pointer to ocs structure
 *
 * @return none
 */

void
ocs_device_free(ocs_t *ocs)
{
	if (ocs != NULL) {
		ocs_textbuf_free(ocs, &ocs->ddump_saved);
		ocs_ramlog_free(ocs, ocs->ramlog);
		num_ocs_devices--;
		ocs_devices[ocs->instance_index] = NULL;
		ocs_device_lock_free(ocs);

#if defined(ENABLE_LOCK_DEBUG)
		{
			ocs_os_t *os = &ocs->ocs_os;
			ocs_lock_t *l;
			uint32_t count = 0;
			ocs_list_foreach(&os->locklist, l) {
				ocs_log_test(os, "%s: %s not freed\n", __func__, l->name);
				count++;
			}
			if (count == 0) {
				ocs_log_debug(ocs, "%s: all lock objects freed\n", __func__);
			}
		}
#endif

		ocs_free(ocs, ocs, sizeof(*ocs));
	}
}

/**
 * @brief return the number of interrupts required per HBA
 *
 * @param ocs pointer to ocs structure
 *
 * @return the number of interrupts.
 */
int32_t
ocs_device_interrupts_required(ocs_t *ocs)
{
	int32_t rc;

	rc = ocs_hal_setup(&ocs->hal, ocs, SLI4_PORT_TYPE_FC);
	if (rc) {
		return -1;
	}
	return ocs_hal_qtop_eq_count(&ocs->hal);
}

/**
 * @brief Initialize resources when pci devices attach
 *
 * @param ocs pointer to ocs structure
 *
 * @return 0 for success, a negative error code value for failure.
 */

int32_t
ocs_device_attach(ocs_t *ocs)
{
	uint32_t rc = 0;

	if (ocs->drv_ocs.attached) {
		ocs_log_warn(ocs, "%s: Device is already attached\n", __func__);
		rc = -1;
	} else {
		ocs_snprintf(ocs->display_name, sizeof(ocs->display_name), "[%s%d] ", "fc", ocs->instance_index);
		ocs->ctrlmask = ctrlmask;
		ocs->topology = topology;
		ocs->speed = speed;
		ocs->ethernet_license = ethernet_license;
		ocs->num_scsi_ios = num_scsi_ios;
		ocs->enable_hlm = enable_hlm;
		ocs->hlm_group_size = hlm_group_size;
		ocs->hal_war_version = hal_war_version;
		ocs->explicit_buffer_list = explicit_buffer_list;
		ocs->auto_xfer_rdy_size = auto_xfer_rdy_size;
		ocs->esoc = esoc;
		ocs->logmask = logmask;
		ocs->num_vports = num_vports;
		ocs->external_loopback = external_loopback;
		ocs->target_io_timer_sec = target_io_timer;
		ocs->driver_version = DRV_VERSION;
		ocs->hal_bounce = hal_bounce;
		ocs->rq_threads = rq_threads;
		ocs->filter_def = filter_def;
		ocs->max_isr_time_msec = OCS_OS_MAX_ISR_TIME_MSEC;
		ocs->model = ocs_pci_model(ocs->pci_vendor, ocs->pci_device);
		ocs->fw_version = (const char *)ocs_hal_get_ptr(&ocs->hal, OCS_HAL_FW_REV);
		ocs->hal.watchdog_timeout = watchdog_timeout;
		ocs->hal.sliport_healthcheck = sliport_healthcheck;

		ocs->tgt_rscn_delay_msec = 0;
		ocs->tgt_rscn_period_msec = 0;

		/* Allocate transport object and bring online */
		ocs->xport = ocs_xport_alloc(ocs);
		if (ocs->xport == NULL) {
			ocs_log_err(ocs, "%s: failed to allocate transport object\n", __func__);
			rc = -1;
		} else if (ocs_xport_attach(ocs->xport) != 0) {
			ocs_log_err(ocs, "%s: failed to attach transport object\n", __func__);
			rc = -1;
		} else if (ocs_xport_initialize(ocs->xport) != 0) {
			ocs_log_err(ocs, "%s: failed to initialize transport object\n", __func__);
			rc = -1;
		} else {
			if ((holdoff_link_online == 2) && ((!ocs->enable_tgt) && (ocs->enable_ini))) {
				holdoff_link_online = 0;
			}

			if (holdoff_link_online == 0 && !ocs->dont_linkup) {
				if (ocs_xport_control(ocs->xport, OCS_XPORT_PORT_ONLINE) != 0) {
					/*
					 * Only log a message.  Don't return the error,
					 * so that the driver will still attach to this port.
					*/
					ocs_log_err(ocs, "%s: failed to bring port online\n", __func__);
				}
			}

		}

		if (rc == 0) {
			ocs->drv_ocs.attached = TRUE;
		} else {
			ocs_xport_free(ocs->xport);
			ocs->xport = NULL;
		}
	}


	return rc;
}

/**
 * @brief free resources when pci device detach
 *
 * @param ocs pointer to ocs structure
 *
 * @return 0 for success, a negative error code value for failure.
 */

int32_t
ocs_device_detach(ocs_t *ocs)
{
	int32_t rc = 0;

	if (ocs != NULL) {

		if (!ocs->drv_ocs.attached) {
			ocs_log_warn(ocs, "%s: Device is not attached\n", __func__);
			return -1;
		}

		ocs_spdk_poller_stop(ocs);

		rc = ocs_xport_control(ocs->xport, OCS_XPORT_SHUTDOWN);
		if (rc) {
			ocs_log_err(ocs, "%s: Transport Shutdown timed out\n", __func__);
		}

		if (ocs_xport_detach(ocs->xport) != 0) {
			ocs_log_err(ocs, "%s: Transport detach failed\n", __func__);
		}

		ocs_xport_free(ocs->xport);
		ocs->xport = NULL;
		ocs->drv_ocs.attached = FALSE;
	}

	return 0;
}

/**
 * @brief get driver wide property value
 *
 * Return valued driver wide property.   All property values using this interface
 * are returned as null terminated text.
 *
 * @param prop_name property name
 * @param buffer pointer to buffer
 * @param buffer_len length of buffer in bytes
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int32_t
ocs_get_property(const char *prop_name, char *buffer, uint32_t buffer_len)
{
#if 0
} else if (ocs_strcmp(prop_name, "initiator") == 0) {
	snprintf(buffer, buffer_len, "%d", initiator);
} else if (ocs_strcmp(prop_name, "target") == 0) {
	snprintf(buffer, buffer_len, "%d", target);
} else if (ocs_strcmp(prop_name, "wwn_bump") == 0) {
	snprintf(buffer, buffer_len, "%s", wwn_bump);
} else if (ocs_strcmp(prop_name, "dif_separate") == 0) {
	snprintf(buffer, buffer_len, "%d", dif_separate);
} else if (ocs_strcmp(prop_name, "ncpu") == 0) {
	ocs_snprintf(buffer, buffer_len, "%ld", sysconf(_SC_NPROCESSORS_ONLN));
} else if (ocs_strcmp(prop_name, "thread_cmds") == 0) {
	ocs_snprintf(buffer, buffer_len, "%d", thread_cmds);
} else if (ocs_strcmp(prop_name, "external_dif") == 0) {
	ocs_strncpy(buffer, external_dif, buffer_len);
} else if (strcmp(prop_name, "esoc") == 0) {
	ocs_snprintf(buffer, buffer_len, "%d", esoc);
} else if (strcmp(prop_name, "auto_xfer_rdy_xri_cnt") == 0) {
	ocs_snprintf(buffer, buffer_len, "%d", auto_xfer_rdy_xri_cnt);
} else {
	ocs_log_test(NULL, "Error: %s: unknown property %s\n", __func__, prop_name);
	snprintf(buffer, buffer_len, "%s", "");
	return -1;
}
#endif
	return 0;
}

/**
 * @brief return OCS instance display name
 *
 * returns const char string of OCS instance display name
 *
 * @param os pointer to OS (ocs_t) object
 *
 * @return return display name
 */

const char* ocs_display_name(void *os)
{
	ocs_t *ocs = os;
	ocs_assert(ocs, "[?]");
	return ocs->display_name;
}

/**
 * @brief return pointer to ocs structure given instance index
 *
 * A pointer to an ocs structure is returned given an instance index.
 *
 * @param index index to ocs_devices array
 *
 * @return ocs pointer
 */

ocs_t* ocs_get_instance(uint32_t index)
{
	if (index < ARRAY_SIZE(ocs_devices)) {
		return ocs_devices[index];
	}
	return NULL;
}

/**
 * @brief Return instance index of an opaque ocs structure
 *
 * Returns the ocs instance index
 *
 * @param os pointer to ocs instance
 *
 * @return pointer to ocs instance index
 */
uint32_t
ocs_instance(void *os)
{
	ocs_t *ocs = os;

	return ocs != NULL ? ocs->instance_index : UINT32_MAX;
}


/**
 * @brief Perform driver wide shutdown complete actions
 *
 * This function is called shutdown for all devices has completed
 *
 * @return none
 */
void
ocs_device_shutdown_complete(void)
{
	if (initiator) {
		ocs_scsi_ini_driver_exit();
	}
	if (target) {
		ocs_scsi_tgt_driver_exit();
	}
}

/**
 * @brief Log module parameters
 *
 * A list of module paramters is gerneated to the driver log
 *
 * @return none
 */
static void
ocs_driver_print_module_params(void)
{
	ocs_log_info(NULL, "%s: driver version %s\n", DRV_NAME, DRV_VERSION);

	ocs_log_info(NULL, "  Module parameters:\n");
	ocs_log_info(NULL, "  Initiator = %d\n",		initiator);
	ocs_log_info(NULL, "  Target = %d\n",			target);
	ocs_log_info(NULL, "  ctrlmask = 0x%04x\n",		ctrlmask);
	ocs_log_info(NULL, "  loglevel = %d\n",			loglevel);
	ocs_log_info(NULL, "  logmask = 0x%04x\n",		logmask);
	ocs_log_info(NULL, "  ramlog_size = %d\n",		ramlog_size);
	ocs_log_info(NULL, "  rq_threads = %d\n",		rq_threads);
	ocs_log_info(NULL, "  wwn_bump = %s\n",			wwn_bump);
	ocs_log_info(NULL, "  topology = %d\n",			topology);
	ocs_log_info(NULL, "  speed = %d\n",			speed);
	ocs_log_info(NULL, "  holdoff_link_online = %d\n",	holdoff_link_online);
	ocs_log_info(NULL, "  enable_hlm = %d\n",		enable_hlm);
	ocs_log_info(NULL, "  hlm_group_size = %d\n",		hlm_group_size);
	ocs_log_info(NULL, "  ethernet_license = %d\n",		ethernet_license);
	ocs_log_info(NULL, "  num_scsi_ios = %d\n",		num_scsi_ios);
	ocs_log_info(NULL, "  dif_separate = %d\n",		dif_separate);
	ocs_log_info(NULL, "  auto_xfer_rdy_size = %d\n",	auto_xfer_rdy_size);
	ocs_log_info(NULL, "  esoc = %d\n",			esoc);
	ocs_log_info(NULL, "  auto_xfer_rdy_xri_cnt = %d\n",	auto_xfer_rdy_xri_cnt);
	ocs_log_info(NULL, "  filter_ref = %s\n",		filter_def);
	ocs_log_info(NULL, "  explicit_buffer_list = %d\n",	explicit_buffer_list);
	ocs_log_info(NULL, "  num_vports = %d\n",		num_vports);
	ocs_log_info(NULL, "  external_loopback = %d\n",	external_loopback);
	ocs_log_info(NULL, "  target_io_timer = %d\n",		target_io_timer);
	ocs_log_info(NULL, "  ddump_saved_size = %d\n",		ddump_saved_size);
	ocs_log_info(NULL, "  hal_war_version = %s\n",		hal_war_version);

#if defined(OCS_INCLUDE_RAMD)
	ocs_log_info(NULL, "  p_type = %d\n",			p_type);
	ocs_log_info(NULL, "  ramdisc_size = %s\n",		ramdisc_size);
	ocs_log_info(NULL, "  ramdisc_blocksize = %d\n",	ramdisc_blocksize);
	ocs_log_info(NULL, "  num_luns = %d\n",			num_luns);
	ocs_log_info(NULL, "  global_ramdisc = %d\n",		global_ramdisc);
	ocs_log_info(NULL, "  stub_res6_rel6 = %d\n",		stub_res6_rel6);
	ocs_log_info(NULL, "  ramd_threading = %d\n",		ramd_threading);
	ocs_log_info(NULL, "  thread_cmds = %d\n",		thread_cmds);
	ocs_log_info(NULL, "  external_dif = %s\n",		external_dif);
#endif
}

