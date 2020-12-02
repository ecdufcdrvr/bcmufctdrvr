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
#include "ocs_recovery.h"
#include "ocs_mgmt.h"
#include "ocs_impl_spdk.h"

#include "fc.h"

#include "spdk/log.h"

static ocs_t *ocs_devices[MAX_OCS_DEVICES];
static uint32_t num_ocs_devices;
static void ocs_driver_print_module_params(void);

int ocs_mgmt_driver_exec(char *parent, char *action, void *arg_in, uint32_t arg_in_length,
		void *arg_out, uint32_t arg_out_length, void *driver);

static ocs_mgmt_functions_t driver_mgmt_functions = {
	.exec_handler = ocs_mgmt_driver_exec
};

/* PARAM_LIST: generate parameter declarations with initializers */
#define P(type, name, value, desc)	type name = value;
PARAM_LIST
#undef P

uint16_t
ocs_sriov_get_nr_vfs(ocs_t *ocs)
{
	return 0;
}

bool
ocs_sriov_config_required(ocs_t *ocs)
{
	return false;
}

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
	ocs_thread_init();

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
		ocs_log_err(ocs, "memory allocation failed for ocs_t\n");
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
		ocs_log_err(NULL, "ocs devices are exceeded\n");
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
			ocs_log_err(ocs, "failed to allocate memory for saved ddump\n");
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
	ocs->mgmt_functions = &driver_mgmt_functions;

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
				ocs_log_test(os, "%s not freed\n", l->name);
				count++;
			}
			if (count == 0) {
				ocs_log_debug(ocs, "all lock objects freed\n");
			}
		}
#endif
		/* Delete worker thread */
		ocs_device_delete_worker_thread(ocs);

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

	ocs->enable_ini = initiator;
	ocs->enable_tgt = target;
	ocs->ini_fc_types = initiator_flags;
	ocs->tgt_fc_types = target_flags;

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
		ocs_log_warn(ocs, "Device is already attached\n");
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
		ocs->esoc = esoc;
		ocs->logmask = logmask;
		ocs->num_vports = num_vports;
		ocs->external_loopback = external_loopback;
		ocs->target_io_timer_sec = target_io_timer;
		ocs->driver_version = DRV_VERSION;
		ocs->hal_bounce = hal_bounce;
		ocs->rq_threads = rq_threads;
		ocs->rr_quanta = rr_quanta;
		ocs->filter_def = filter_def;
		ocs->max_isr_time_msec = OCS_OS_MAX_ISR_TIME_MSEC;
		ocs->model = ocs_pci_model(ocs->pci_vendor, ocs->pci_device);
		ocs->fw_version = (const char *)ocs_hal_get_ptr(&ocs->hal, OCS_HAL_FW_REV);
		ocs->sliport_pause_errors = sliport_pause_errors;
		ocs->enable_bbcr = enable_bbcr;
		ocs->enable_dpp = 0;
		ocs->enable_poll_mode = 1;
		ocs->enable_dual_dump = enable_dual_dump;

		ocs->enable_ini = initiator;
		ocs->enable_tgt = target;
		ocs->ini_fc_types = initiator_flags;
		ocs->tgt_fc_types = target_flags;

		ocs->tgt_rscn_delay_msec = 0;
		ocs->tgt_rscn_period_msec = 0;

		if (fw_dump_type != OCS_FW_DUMP_TYPE_DUMPTOHOST &&
				fw_dump_type != OCS_FW_DUMP_TYPE_FLASH) {
			char *dump_selection = "dump to flash";

			ocs_log_info(ocs, "Module param FW dump type is invalid. "
					"Using default setting: %s\n", dump_selection);
			fw_dump_type = OCS_FW_DUMP_TYPE_FLASH;
		}

		ocs_fw_dump_init(ocs, fw_dump_type);

		/* Allocate transport object and bring online */
		ocs->xport = ocs_xport_alloc(ocs);
		if (ocs->xport == NULL) {
			ocs_log_err(ocs, "failed to allocate transport object\n");
			rc = -1;
		} else if (ocs_xport_attach(ocs->xport) != 0) {
			ocs_log_err(ocs, "failed to attach transport object\n");
			rc = -1;
		} else if (ocs_xport_initialize(ocs->xport) != 0) {
			ocs_log_err(ocs, "failed to initialize transport object\n");
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
					ocs_log_err(ocs, "failed to bring port online\n");
				}
			}

		}
		ocs->desc = ocs->hal.sli.config.modeldesc;
		ocs_log_info(ocs, "adapter model description: %s\n", ocs->hal.sli.config.modeldesc);

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
		kill(ocs->drv_ocs.ioctl_thr.tid, SIGRTMIN);
		ocs_thread_join(&ocs->drv_ocs.ioctl_thr);

		if (!ocs->drv_ocs.attached) {
			ocs_log_warn(ocs, "Device is not attached\n");
			return -1;
		}

		ocs->drv_ocs.shutting_down = TRUE;

		rc = ocs_xport_control(ocs->xport, OCS_XPORT_SHUTDOWN);
		if (rc) {
			ocs_log_err(ocs, "Transport Shutdown timed out\n");
		}

		ocs_stop_event_processing(&ocs->ocs_os);

		if (ocs_xport_detach(ocs->xport) != 0) {
			ocs_log_err(ocs, "Transport detach failed\n");
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
	int32_t rc = -1;

	snprintf(buffer, buffer_len, "%s", "");

	if (ocs_strcmp(prop_name, "enable_nsler") == 0) {
		snprintf(buffer, buffer_len, "%d", enable_nsler);
		rc = 0;
		goto exit;
	}

exit:
	return rc;
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
	ocs_log_info(NULL, "  Initiator_flags = %x\n",		initiator_flags);
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
	ocs_log_info(NULL, "  tow_feature = %d\n",              tow_feature);
	ocs_log_info(NULL, "  tow_xri_cnt = %d\n",              tow_xri_cnt);
	ocs_log_info(NULL, "  tow_io_size = %d\n",              tow_io_size);
	ocs_log_info(NULL, "  esoc = %d\n",			esoc);
	ocs_log_info(NULL, "  rr_quanta = %d\n",		rr_quanta);
	ocs_log_info(NULL, "  filter_ref = %s\n",		filter_def);
	ocs_log_info(NULL, "  explicit_buffer_list = %d\n",	explicit_buffer_list);
	ocs_log_info(NULL, "  num_vports = %d\n",		num_vports);
	ocs_log_info(NULL, "  external_loopback = %d\n",	external_loopback);
	ocs_log_info(NULL, "  target_io_timer = %d\n",		target_io_timer);
	ocs_log_info(NULL, "  ddump_saved_size = %d\n",		ddump_saved_size);
	ocs_log_info(NULL, "  hal_war_version = %s\n",		hal_war_version);
	ocs_log_info(NULL, "  enable_nsler = %d\n",		enable_nsler);
	ocs_log_info(NULL, "  fw_dump_type = %d\n",             fw_dump_type);

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


int ocs_mgmt_driver_exec(char *parent, char *action_name, void *arg_in,
		uint32_t arg_in_length, void *arg_out, uint32_t arg_out_length, void *object)
{
	char qualifier[80];
	int retval = -1;
	ocs_t *ocs = (ocs_t *)object;


	ocs_snprintf(qualifier, sizeof(qualifier), "%s/driver", parent);

	if (ocs_strncmp(action_name, qualifier, strlen(qualifier)) == 0) {
		char *unqualified_name = action_name + strlen(qualifier) +1;

		/* See if it's a value I can supply */
		if (ocs_strcmp(unqualified_name, "gendump") == 0) {
			bool force;
			retval = ocs_copy_from_user(&force, arg_in, sizeof(force));
			if (retval) {
				ocs_log_err(ocs, "Gendump read user failed\n");
				return -EFAULT;
			}
			retval = ocs_gendump(ocs, OCS_FW_CHIP_LEVEL_DUMP, force);
		}

                if (ocs_strcmp(unqualified_name, "dump_to_host") == 0) {
                        retval = ocs_dump_to_host(ocs, arg_out, arg_out_length, OCS_FW_CHIP_LEVEL_DUMP);
                }

                if (ocs_strcmp(unqualified_name, "fdb_dump_to_host") == 0) {
                        retval = ocs_dump_to_host(ocs, arg_out, arg_out_length, OCS_FW_FUNC_DESC_DUMP);
                }

	}

	return retval;
}
