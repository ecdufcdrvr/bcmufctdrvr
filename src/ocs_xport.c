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

/**
 * @file
 * FC transport API
 *
 */

#include "ocs.h"
#include "ocs_compat.h"
#include "ocs_nvme_stub.h"

static void ocs_xport_link_stats_cb(int32_t status, uint32_t num_counters, ocs_hal_link_stat_counts_t *counters, void *arg);
static void ocs_xport_linkup_stats_cb(int32_t status, uint32_t num_counters, ocs_hal_link_stat_counts_t *counters, void *arg);
static void ocs_xport_host_stats_cb(int32_t status, uint32_t num_counters, ocs_hal_host_stat_counts_t *counters, void *arg);
static void ocs_xport_async_link_stats_cb(int32_t status, uint32_t num_counters, ocs_hal_link_stat_counts_t *counters, void *arg);
static void ocs_xport_async_host_stats_cb(int32_t status, uint32_t num_counters, ocs_hal_host_stat_counts_t *counters, void *arg);

/**
 * @brief Post node event callback argument.
 */
typedef struct {
	ocs_sem_t sem;
	ocs_atomic_t refcnt;
	ocs_node_t *node;
	ocs_sm_event_t evt;
	void *context;
} ocs_xport_post_node_event_t;

#define OCS_XPORT_STATS_TIMER_MS 3 * 1000

/**
 * @brief Allocate a transport object.
 *
 * @par Description
 * A transport object is allocated, and associated with a device instance.
 *
 * @param ocs Pointer to device instance.
 *
 * @return Returns the pointer to the allocated transport object, or NULL if failed.
 */
ocs_xport_t *
ocs_xport_alloc(ocs_t *ocs)
{
	ocs_xport_t *xport;

	ocs_assert(ocs, NULL);
	xport = ocs_malloc(ocs, sizeof(*xport), OCS_M_ZERO);
	if (xport != NULL) {
		xport->ocs = ocs;
	}
	return xport;
}

/**
 * @brief Create the RQ threads and the circular buffers used to pass sequences.
 *
 * @par Description
 * Creates the circular buffers and the servicing threads for RQ processing.
 *
 * @param xport Pointer to transport object
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
void
ocs_xport_rq_threads_teardown(ocs_xport_t *xport)
{
	ocs_t *ocs = xport->ocs;
	uint32_t i;

	if (xport->num_rq_threads == 0 ||
	    xport->rq_thread_info == NULL) {
		return;
	}

	/* Abort any threads */
	for (i = 0; i < xport->num_rq_threads; i++) {
		if (xport->rq_thread_info[i].thread_started) {
			ocs_thread_terminate(&xport->rq_thread_info[i].thread);
			/* wait for the thread to exit */
			ocs_log_debug(ocs, "wait for thread %d to exit\n", i);
			while (xport->rq_thread_info[i].thread_started) {
				ocs_delay_usec(10000);
			}
			ocs_log_debug(ocs, "thread %d to exited\n", i);
		}
		if (xport->rq_thread_info[i].seq_cbuf != NULL) {
			ocs_cbuf_free(xport->rq_thread_info[i].seq_cbuf);
			xport->rq_thread_info[i].seq_cbuf = NULL;
		}
	}
}

/**
 * @brief Create the RQ threads and the circular buffers used to pass sequences.
 *
 * @par Description
 * Creates the circular buffers and the servicing threads for RQ processing.
 *
 * @param xport Pointer to transport object.
 * @param num_rq_threads Number of RQ processing threads that the
 * driver creates.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
ocs_xport_rq_threads_create(ocs_xport_t *xport, uint32_t num_rq_threads)
{
	ocs_t *ocs = xport->ocs;
	int32_t rc = 0;
	uint32_t i;

	xport->num_rq_threads = num_rq_threads;
	ocs_log_debug(ocs, "number of RQ threads %d\n", num_rq_threads);
	if (num_rq_threads == 0) {
		return 0;
	}

	/* Allocate the space for the thread objects */
	xport->rq_thread_info = ocs_malloc(ocs, sizeof(ocs_xport_rq_thread_info_t) * num_rq_threads, OCS_M_ZERO);
	if (xport->rq_thread_info == NULL) {
		ocs_log_err(ocs, "memory allocation failure\n");
		return -1;
	}

	/* Create the circular buffers and threads. */
	for (i = 0; i < num_rq_threads; i++) {
		xport->rq_thread_info[i].ocs = ocs;
		xport->rq_thread_info[i].seq_cbuf = ocs_cbuf_alloc(ocs, OCS_RQTHREADS_MAX_SEQ_CBUF);
		if (xport->rq_thread_info[i].seq_cbuf == NULL) {
			goto ocs_xport_rq_threads_create_error;
		}

		ocs_snprintf(xport->rq_thread_info[i].thread_name,
			     sizeof(xport->rq_thread_info[i].thread_name),
			     "ocs_unsol_rq:%d:%d", ocs->instance_index, i);
		rc = ocs_thread_create(ocs, &xport->rq_thread_info[i].thread, ocs_unsol_rq_thread,
				       xport->rq_thread_info[i].thread_name,
				       &xport->rq_thread_info[i], OCS_THREAD_RUN);
		if (rc) {
			ocs_log_err(ocs, "ocs_thread_create failed: %d\n", rc);
			goto ocs_xport_rq_threads_create_error;
		}
		xport->rq_thread_info[i].thread_started = TRUE;
	}
	return 0;

ocs_xport_rq_threads_create_error:
	ocs_xport_rq_threads_teardown(xport);
	return -1;
}

/**
 * @brief Do as much allocation as possible, but do not initialization the device.
 *
 * @par Description
 * Performs the functions required to get a device ready to run.
 *
 * @param xport Pointer to transport object.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
ocs_xport_attach(ocs_xport_t *xport)
{
	ocs_t *ocs = xport->ocs;
	int32_t rc;
	uint32_t max_sgl;
	uint32_t n_sgl;
	uint32_t i;

	/* booleans used for cleanup if initialization fails */
	uint8_t node_pool_created = FALSE;

	ocs_list_init(&ocs->domain_list, ocs_domain_t, link);

	for (i = 0; i < SLI4_MAX_FCFI; i++) {
		xport->fcfi[i].hold_frames = 1;
		ocs_lock_init(ocs, &xport->fcfi[i].pend_frames_lock, OCS_LOCK_ORDER_IGNORE, "xport pend_frames[%d]", i);
		ocs_list_init(&xport->fcfi[i].pend_frames, ocs_hal_sequence_t, link);
	}

	rc = ocs_hal_set_ptr(&ocs->hal, OCS_HAL_WAR_VERSION, ocs->hal_war_version);
	if (rc) {
		ocs_log_test(ocs, "can't set OCS_HAL_WAR_VERSION\n");
		return -1;
	}

	rc = ocs_hal_setup(&ocs->hal, ocs, SLI4_PORT_TYPE_FC);
	if (rc) {
		ocs_log_err(ocs, "%s: Can't setup hardware\n", ocs->desc);
		return -1;
	} else if (ocs->ctrlmask & OCS_CTRLMASK_CRASH_RESET) {
		ocs_log_debug(ocs, "stopping after ocs_hal_setup\n");
		return -1;
	}

	/* Validate FW dump type after HAL setup */
	if (!ocs_fw_dump_type_validate(ocs)) {
		ocs_log_err(ocs, "FW dump type validate failed\n");
		return -1;
	}

	/* Validate SRIOV config after HAL setup */
	if (!ocs_sriov_config_validate(ocs)) {
		ocs_log_err(ocs, "Invalid SRIOV config in Virtfn\n");
		return -1;
	}

	ocs->hal.sliport_healthcheck = ocs->sliport_healthcheck;
	ocs->hal.disable_fec = ocs->disable_fec;
	ocs->hal.enable_fw_ag_rsp = ocs->enable_fw_ag_rsp;
	ocs->hal.sliport_pause_errors = ocs->sliport_pause_errors;

	ocs_hal_set(&ocs->hal, OCS_HAL_BOUNCE, ocs->hal_bounce);
	ocs_log_debug(ocs, "HAL bounce: %d\n", ocs->hal_bounce);

	ocs_hal_set(&ocs->hal, OCS_HAL_RR_QUANTA, ocs->rr_quanta);
	ocs_hal_set_ptr(&ocs->hal, OCS_HAL_FILTER_DEF, (void*) ocs->filter_def);
	ocs_hal_set(&ocs->hal, OCS_HAL_ENABLE_DUAL_DUMP, ocs->enable_dual_dump);

	ocs_hal_get(&ocs->hal, OCS_HAL_MAX_SGL, &max_sgl);
	n_sgl = MIN(OCS_FC_MAX_SGL, max_sgl);

	/* EVT: For chained SGL testing */
	if (ocs->ctrlmask & OCS_CTRLMASK_TEST_CHAINED_SGLS) {
		n_sgl = 8;
	}

	/* Note: number of SGLs must be set for ocs_node_create_pool */
	if (ocs_hal_set(&ocs->hal, OCS_HAL_N_SGL, n_sgl) != OCS_HAL_RTN_SUCCESS) {
		ocs_log_err(ocs, "%s: Can't set number of SGLs\n", ocs->desc);
		return -1;
	} else {
		ocs_log_debug(ocs, "%s: Configured for %d SGLs\n", ocs->desc, n_sgl);
	}

	/* Get updated MAX number of SGL */
	if (ocs_hal_get(&ocs->hal, OCS_HAL_N_SGL, &n_sgl)) {
		ocs_log_err(ocs, "failed to get number of SGLs\n");
		return -1;
	}

#if !defined(OCS_USPACE)
	if (ocs_percpu_counter_init(&xport->fc_stats.stats.fcp_stats.input_requests, 0, GFP_KERNEL) ||
	    ocs_percpu_counter_init(&xport->fc_stats.stats.fcp_stats.input_bytes, 0, GFP_KERNEL) ||
	    ocs_percpu_counter_init(&xport->fc_stats.stats.fcp_stats.output_requests, 0, GFP_KERNEL) ||
	    ocs_percpu_counter_init(&xport->fc_stats.stats.fcp_stats.output_bytes, 0, GFP_KERNEL) ||
	    ocs_percpu_counter_init(&xport->fc_stats.stats.fcp_stats.control_requests, 0, GFP_KERNEL))
		goto ocs_xport_attach_cleanup;
#endif

	/*
	 * Suppress response feature must satisfy the following 3 conditions.
	 * Module parameter cfg_suppress_rsp must be set (default 1)
	 * In SLI4-Parameters,
	 * Extended Inline Buffers (XIB) must be supported.
	 * Suppress Response UI Not Supported (SRIUNS) must not be supported.
	 * Must be validated after reading SLI4-parameters.
	 */
	if (ocs->cfg_suppress_rsp && sli_get_xib_capable(&ocs->hal.sli) &&
	    !sli_get_suppress_rsp_not_supported(&ocs->hal.sli)) {
		ocs_hal_set(&ocs->hal, OCS_HAL_SUPPRESS_RSP_CAPABLE, TRUE);
	} else {
		ocs_hal_set(&ocs->hal, OCS_HAL_SUPPRESS_RSP_CAPABLE, FALSE);
	}

	rc = ocs_node_create_pool(ocs, ocs->max_remote_nodes);
	if (rc) {
		ocs_log_err(ocs, "Can't allocate node pool\n");
		goto ocs_xport_attach_cleanup;
	} else {
		node_pool_created = TRUE;
	}

	/* EVT: if testing chained SGLs allocate OCS_FC_MAX_SGL SGE's in the IO */
	xport->io_pool = ocs_io_pool_create(ocs, ocs->num_scsi_ios,
		(ocs->ctrlmask & OCS_CTRLMASK_TEST_CHAINED_SGLS) ? OCS_FC_MAX_SGL : n_sgl, false);
	if (xport->io_pool == NULL) {
		ocs_log_err(ocs, "Can't allocate IO pool\n");
		goto ocs_xport_attach_cleanup;
	}

	xport->els_io_pool = ocs_io_pool_create(ocs, ocs->max_remote_nodes * 4, 0, true);
	if (xport->els_io_pool == NULL) {
		ocs_log_err(ocs, "Can't allocate ELS IO pool\n");
		goto ocs_xport_attach_cleanup;
	}
	/*
	 * setup the RQ processing threads
	 */
	if (ocs_xport_rq_threads_create(xport, ocs->rq_threads) != 0) {
		ocs_log_err(ocs, "failure creating RQ threads\n");
		goto ocs_xport_attach_cleanup;
	}

	return 0;

ocs_xport_attach_cleanup:
#if !defined(OCS_USPACE)
	percpu_counter_destroy(&xport->fc_stats.stats.fcp_stats.input_requests);
	percpu_counter_destroy(&xport->fc_stats.stats.fcp_stats.input_bytes);
	percpu_counter_destroy(&xport->fc_stats.stats.fcp_stats.output_requests);
	percpu_counter_destroy(&xport->fc_stats.stats.fcp_stats.output_bytes);
	percpu_counter_destroy(&xport->fc_stats.stats.fcp_stats.control_requests);
#endif

	ocs_io_pool_free(xport->io_pool);
	ocs_io_pool_free(xport->els_io_pool);

	if (node_pool_created) {
		ocs_node_free_pool(ocs);
	}

	return -1;
}

static int32_t
ocs_nsler_setup(ocs_t *ocs)
{
	char prop_buf[32];
	int32_t enable_nsler;

	ocs_hal_set(&ocs->hal, OCS_HAL_NSLER_CAPABLE, false);
	if (ocs_get_property("enable_nsler", prop_buf, sizeof(prop_buf)))
		return 0;

	enable_nsler = ocs_strtoul(prop_buf, 0, 0);
	if (!enable_nsler)
		return 0;

	if (ocs->enable_hlm) {
		ocs_log_err(ocs, "NSLER and HLM both can't be supported\n");
		return -1;
	}

	if (!sli_feature_iaab_enabled(&ocs->hal.sli) || !sli_feature_iaar_enabled(&ocs->hal.sli)) {
		ocs_log_err(ocs, "NSLER protocol recommended to use IAAB and IAAR features\n");
		return -1;
	}

	if (!sli_get_nsler_capable(&ocs->hal.sli)) {
		ocs_log_err(ocs, "SLER is not supported\n");
	} else {
		ocs_hal_set(&ocs->hal, OCS_HAL_NSLER_CAPABLE, true);
	}

	return 0;
}

static int32_t
ocs_tow_io_size_valid(int32_t tow_feature, int32_t tow_io_size)
{
	int32_t rc = TRUE;

	if (tow_feature & OCS_TOW_FEATURE_AXR) {
		if (tow_io_size < OCS_TOW_AXR_IO_SIZE_MIN || tow_io_size > OCS_TOW_IO_SIZE_MAX)
			rc = FALSE;
	}

	if (rc && (tow_feature & OCS_TOW_FEATURE_TFB)) {
		if (tow_io_size % 512 || tow_io_size > OCS_TOW_IO_SIZE_MAX)
			rc = FALSE;
	}

	return rc;
}

/**
 * @brief Setup AXR/FB optimized write parameters.
 *        Must be called after sli_setup.
 *
 * @par Description
 * @param ocs Pointer to OCS context
 *
 * @return Returns 0 on success or a non-zero value on failure.
 */
static int32_t
ocs_tow_setup(ocs_t *ocs)
{
	uint32_t agxf;
	uint32_t tow;
	char prop_buf[32];
	int32_t rc = -1;
	uint32_t ramdisc_blocksize = 512;
	uint8_t p_type = 0;
	int32_t tow_feature = 0;
	uint32_t tow_xri_cnt = 0;
	int32_t tow_io_size = 0;
	uint32_t tow_xris_max = 0;

	if (ocs_get_property("tow_feature", prop_buf, sizeof(prop_buf)) == 0)
		tow_feature = ocs_strtoul(prop_buf, 0, 0);

	if (!tow_feature)
		return 0;

	if (!(tow_feature & OCS_TOW_FEATURE_AXR) && !(tow_feature & OCS_TOW_FEATURE_TFB))
		return rc;

	if (ocs_get_property("tow_io_size", prop_buf, sizeof(prop_buf)) == 0)
		tow_io_size = ocs_strtoul(prop_buf, 0, 0);

	if (ocs_get_property("tow_xri_cnt", prop_buf, sizeof(prop_buf)) == 0)
		tow_xri_cnt = ocs_strtoul(prop_buf, 0, 0);

	if (!ocs_tow_io_size_valid(tow_feature, tow_io_size)) {
		ocs_log_err(ocs, "Invalid target optimized write io size\n");
		return rc;
	}

	ocs_hal_get(&ocs->hal, OCS_HAL_AUTO_XFER_RDY_CAPABLE, &agxf);
	ocs_hal_get(&ocs->hal, OCS_HAL_TOW_CAPABLE, &tow);

	if ((tow_feature & OCS_TOW_FEATURE_AXR) && !agxf && !tow) {
		ocs_log_err(ocs, "Auto Xfer Rdy feature is unsupported\n");
		return rc;
	}

	if ((tow_feature & OCS_TOW_FEATURE_TFB) && !tow) {
		ocs_log_err(ocs, "First burst feature is unsupported\n");
		return rc;
	}

	if (ocs_hal_set(&ocs->hal, OCS_HAL_TOW_FEATURE, tow_feature)) {
		ocs_log_err(ocs, "HAL set tow feature failed\n");
		return rc;
	}

	if (ocs_hal_set(&ocs->hal, OCS_HAL_TOW_IO_SIZE, tow_io_size)) {
		ocs_log_err(ocs, "HAL set tow io size failed\n");
		return rc;
	}

	if (tow) {
		ocs_hal_get(&ocs->hal, OCS_HAL_TOW_XRIS_MAX, &tow_xris_max);
		if (tow_xri_cnt > tow_xris_max) {
			ocs_log_err(ocs, "Invalid TOW xri count! Max. supported %d\n", tow_xris_max);
			return rc;
		}
	}

	if (ocs_hal_set(&ocs->hal, OCS_HAL_TOW_XRI_CNT, tow_xri_cnt)) {
		ocs_log_err(ocs, "HAL set tow xri cnt failed\n");
		return rc;
	}

	/*
	 * Determine if we are doing protection in the backend. We are looking
	 * at the modules parameters here. The backend cannot allow a format
	 * command to change the protection mode when using this feature,
	 * otherwise the firmware will not do the proper thing.
	 */
	if (ocs_get_property("p_type", prop_buf, sizeof(prop_buf)) == 0) {
		p_type = ocs_strtoul(prop_buf, 0, 0);
	}

	if (ocs_get_property("ramdisc_blocksize", prop_buf, sizeof(prop_buf)) == 0) {
		ramdisc_blocksize = ocs_strtoul(prop_buf, 0, 0);
	}

	if (ocs_get_property("external_dif", prop_buf, sizeof(prop_buf)) == 0) {
		if (ocs_strlen(prop_buf)) {
			if (p_type == 0) {
				p_type = 1;
			}
		}
	}

	if (p_type != 0) {
		if (ocs_hal_set(&ocs->hal, OCS_HAL_TOW_T10_ENABLE, TRUE)) {
			ocs_log_err(ocs, "HAL set tow t10 failed\n");
			return rc;
		}

		if (ocs_hal_set(&ocs->hal, OCS_HAL_TOW_BLK_SIZE, ramdisc_blocksize)) {
			ocs_log_err(ocs, "HAL set tow block size failed\n");
			return rc;
		}

		if (ocs_hal_set(&ocs->hal, OCS_HAL_TOW_P_TYPE, p_type)) {
			ocs_log_err(ocs, "HAL set tow p_type failed\n");
			return rc;
		}

		if (ocs_hal_set(&ocs->hal, OCS_HAL_TOW_REF_TAG_LBA, TRUE)) {
			ocs_log_err(ocs, "HAL set tow tag lba failed\n");
			return rc;
		}

		if (ocs_hal_set(&ocs->hal, OCS_HAL_TOW_APP_TAG_VALID, FALSE)) {
			ocs_log_err(ocs, "HAL set tow app tag valid\n");
			return rc;
		}
	}

	if (tow_feature && ocs->esoc)
		ocs_hal_set(&ocs->hal, OCS_HAL_ESOC, TRUE);

	return 0;
}
/**
 * @brief Initialize link topology config
 *
 * @par Description
 * Topology can be fetched from mod-param or Persistet Topology(PT).
 *  a. Mod-param value is used when the value is 1(P2P) or 2(LOOP).
 *  a. PT is used if mod-param is not provided( i.e, default value of AUTO)
 * Also, if mod-param is used, update PT.
 *
 * @param ocs Pointer to ocs
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */

static int
ocs_topology_setup(ocs_t *ocs)
{
	uint32_t topology;

	if (ocs->topology == OCS_HAL_TOPOLOGY_AUTO) {
		topology = sli_get_config_persistent_topology(&ocs->hal.sli);
	}  else {
		topology = ocs->topology;
		/* ignore failure here. link will come-up either in auto mode
		 * if PT is not supported or last saved PT value */
		ocs_hal_set_persistent_topology(&ocs->hal, topology, OCS_CMD_POLL);
	}

	return ocs_hal_set(&ocs->hal, OCS_HAL_TOPOLOGY, topology);
}

int32_t
ocs_xport_init_backends(ocs_t *ocs)
{
	uint8_t ini_device_set = FALSE;
	uint8_t tgt_device_set = FALSE;
	uint8_t nvme_ini_device_set = FALSE;
	uint8_t nvme_tgt_device_set = FALSE;
	uint8_t tgt_common_set = FALSE;

	if (ocs->enable_tgt) {
		if (ocs_tgt_scsi_enabled(ocs)) {
			if (ocs_scsi_tgt_new_device(ocs)) {
				ocs_log_err(ocs, "failed to initialize scsi target\n");
				goto cleanup;
			}
			tgt_device_set = TRUE;
		}

		if (ocs_tgt_nvme_enabled(ocs)) {
			if (ocs_nvme_tgt_new_device(ocs)) {
				ocs_log_err(ocs, "failed to initialize nvme target\n");
				goto cleanup;
			}
			nvme_tgt_device_set = TRUE;
		}

		if (tgt_device_set || nvme_tgt_device_set) {
			ocs_tgt_common_new_device(ocs);
			tgt_common_set = TRUE;
		}
	}

	/*
	 * Create Scsi_Host in all modes as this is required for scsi host sysfs interface.
	 */
	if (ocs_scsi_ini_new_device(ocs)) {
		ocs_log_err(ocs, "failed to initialize scsi initiator\n");
		goto cleanup;
	}
	ini_device_set = TRUE;

	if (ocs->enable_ini) {
		if (ocs_ini_nvme_enabled(ocs)) {
			if (ocs_nvme_ini_new_device(ocs)) {
				ocs_log_err(ocs, "failed to initialize nvme initiator\n");
				goto cleanup;
			}
			nvme_ini_device_set = TRUE;
		}
	}
	return 0;
cleanup:
	if (ini_device_set)
		ocs_scsi_ini_del_device(ocs);

	if (tgt_common_set)
		ocs_tgt_common_del_device(ocs);

	if (tgt_device_set)
		ocs_scsi_tgt_del_device(ocs);

	if (nvme_tgt_device_set)
		ocs_nvme_tgt_del_device(ocs);

	if (nvme_ini_device_set)
		ocs_nvme_ini_del_device(ocs);

	return -1;
}

void
ocs_xport_cleanup_backends(ocs_t *ocs)
{
	/* free resources associated with target-server and initiator-client */
	if (ocs_tgt_nvme_enabled(ocs))
		ocs_nvme_tgt_del_device(ocs);

	if (ocs_ini_nvme_enabled(ocs))
		ocs_nvme_ini_del_device(ocs);

	if (ocs_tgt_scsi_enabled(ocs))
		ocs_scsi_tgt_del_device(ocs);

	if (ocs_tgt_nvme_enabled(ocs) || ocs_tgt_scsi_enabled(ocs))
		ocs_tgt_common_del_device(ocs);

	ocs_scsi_ini_del_device(ocs);
}


/**
 * @brief Initializes the device.
 *
 * @par Description
 * Performs the functions required to make a device functional.
 *
 * @param xport Pointer to transport object.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
ocs_xport_initialize(ocs_xport_t *xport)
{
	ocs_t *ocs = xport->ocs;
	int32_t rc;
	uint32_t i;
	uint32_t hlm;
	uint32_t dpp;
	uint32_t dif_capable;
	uint8_t dif_separate = 0;
	char prop_buf[32];
	int32_t prop_val;

	/* booleans used for cleanup if initialization fails */
	uint8_t hal_initialized = FALSE;

	if (ocs_get_property("enable_rq_no_buff_warns", prop_buf, sizeof(prop_buf)) == 0) {
		prop_val = ocs_strtoul(prop_buf, 0, 0);
		if (ocs_hal_set(&ocs->hal, OCS_HAL_ENABLE_RQ_NO_BUFF_WARNS, prop_val)) {
			ocs_log_err(ocs, "HAL set failed for RQ no buffer warns\n");
			return -1;
		}
	}

	if (ocs->enable_hlm) {
		ocs_hal_get(&ocs->hal, OCS_HAL_HIGH_LOGIN_MODE, &hlm);
		if (!hlm) {
			ocs->enable_hlm = FALSE;
			ocs_log_err(ocs, "Cannot enable high login mode for this port\n");
		} else {
                        ocs_log_debug(ocs, "High login mode is enabled\n");
			if (ocs_hal_set(&ocs->hal, OCS_HAL_HIGH_LOGIN_MODE, TRUE)) {
				ocs_log_err(ocs, "%s: Can't set high login mode\n", ocs->desc);
				return -1;
			}
		}
	}

	if (ocs->enable_dpp) {
		ocs_hal_get(&ocs->hal, OCS_HAL_DPP_MODE, &dpp);
		if (!dpp) {
			ocs->enable_dpp = FALSE;
			ocs_log_warn(ocs, "DPP mode not supported for this port \n");
		} else {
			if (ocs_hal_set(&ocs->hal, OCS_HAL_DPP_MODE, TRUE)) {
				ocs_log_err(ocs, "%s: enable DPP mode failed\n", ocs->desc);
				return -1;
			}
			ocs_log_debug(ocs, "DPP mode is enabled\n");
		}
	}

	if (ocs->enable_poll_mode) {
		if (ocs_hal_set(&ocs->hal, OCS_HAL_POLL_MODE, TRUE)) {
			ocs_log_err(ocs, "%s: enable polling mode failed\n", ocs->desc);
			return -1;
		}
		ocs_log_info(ocs, "Polling mode is enabled\n");
	}

	if (ocs_nsler_setup(ocs)) {
		ocs_log_err(ocs, "NVMe SLER setup failed\n");
		return OCS_HAL_RTN_ERROR;
	}

	/* Setup AXR/FB optimized write parameters, must be called after sli_setup */
	if (ocs_tow_setup(ocs)) {
		ocs_log_err(ocs, "Target optimized write setup failed\n");
		return OCS_HAL_RTN_ERROR;
	}

	if (ocs->explicit_buffer_list) {
		// Are pre-registered SGL's required?
		ocs_hal_get(&ocs->hal, OCS_HAL_PREREGISTER_SGL, &i);
		if (i == TRUE) {
			ocs_log_err(ocs, "Explicit Buffer List not supported on this device, not enabled\n");
		} else {
			ocs_hal_set(&ocs->hal, OCS_HAL_PREREGISTER_SGL, FALSE);
		}
	}

	/* Setup persistent topology based on topology mod-param value */
	rc = ocs_topology_setup(ocs);
	if (rc) {
		ocs_log_err(ocs, "%s: Can't set the toplogy\n", ocs->desc);
		return -1;
	}

	ocs_hal_set(&ocs->hal, OCS_HAL_RQ_DEFAULT_BUFFER_SIZE, OCS_FC_RQ_SIZE_DEFAULT);

	if (ocs_hal_set(&ocs->hal, OCS_HAL_LINK_SPEED, ocs->speed) != OCS_HAL_RTN_SUCCESS) {
		ocs_log_err(ocs, "%s: Can't set the link speed\n", ocs->desc);
		return -1;
	}

	if (ocs_hal_set(&ocs->hal, OCS_HAL_ETH_LICENSE, ocs->ethernet_license) != OCS_HAL_RTN_SUCCESS) {
		ocs_log_err(ocs, "%s: Can't set the ethernet license\n", ocs->desc);
		return -1;
	}

	/* currently only lancer support setting the CRC seed value */
	if (ocs->hal.sli.asic_type == SLI4_ASIC_TYPE_LANCER) {
		if (ocs_hal_set(&ocs->hal, OCS_HAL_DIF_SEED, OCS_FC_DIF_SEED) != OCS_HAL_RTN_SUCCESS) {
			ocs_log_err(ocs, "%s: Can't set the DIF seed\n", ocs->desc);
			return -1;
		}
	}

	/* Set the Dif mode */
	if (0 == ocs_hal_get(&ocs->hal, OCS_HAL_DIF_CAPABLE, &dif_capable)) {
		if (dif_capable) {
			if (ocs_get_property("dif_separate", prop_buf, sizeof(prop_buf)) == 0) {
				dif_separate = ocs_strtoul(prop_buf, 0, 0);
			}

			if ((rc = ocs_hal_set(&ocs->hal, OCS_HAL_DIF_MODE,
			      (dif_separate == 0 ? OCS_HAL_DIF_MODE_INLINE : OCS_HAL_DIF_MODE_SEPARATE)))) {
				ocs_log_err(ocs, "Requested DIF MODE not supported\n");
			}
		}
	}

	if (ocs->target_io_timer_sec) {
		ocs_log_debug(ocs, "setting target io timer=%d\n", ocs->target_io_timer_sec);
		ocs_hal_set(&ocs->hal, OCS_HAL_EMULATE_TARGET_WQE_TIMEOUT, TRUE);
	}

	ocs_hal_callback(&ocs->hal, OCS_HAL_CB_DOMAIN, ocs_domain_cb, ocs);
	ocs_hal_callback(&ocs->hal, OCS_HAL_CB_REMOTE_NODE, ocs_remote_node_cb, ocs);
	ocs_hal_callback(&ocs->hal, OCS_HAL_CB_UNSOLICITED, ocs_unsolicited_cb, ocs);
	ocs_hal_callback(&ocs->hal, OCS_HAL_CB_PORT, ocs_port_cb, ocs);
	ocs_hal_callback(&ocs->hal, OCS_HAL_CB_XPORT, ocs_xport_cb, ocs);

	ocs->fw_version = (const char*) ocs_hal_get_ptr(&ocs->hal, OCS_HAL_FW_REV);

	/* Initialize vport list */
	ocs_list_init(&xport->vport_list, ocs_vport_spec_t, link);
	ocs_lock_init(ocs, &xport->io_pending_lock, OCS_LOCK_ORDER_IO_PENDING, "io_pending_lock[%d]", ocs->instance_index);
	ocs_list_init(&xport->io_pending_list, ocs_io_t, io_pending_link);

	ocs_atomic_init(&xport->io_active_count, 0);
	ocs_atomic_init(&xport->io_total_alloc, 0);
	ocs_atomic_init(&xport->io_total_free, 0);

#if defined(OCS_USPACE)
	ocs_atomic_init(&xport->fc_stats.stats.fcp_stats.input_requests, 0);
	ocs_atomic_init(&xport->fc_stats.stats.fcp_stats.input_bytes, 0);
	ocs_atomic_init(&xport->fc_stats.stats.fcp_stats.output_requests, 0);
	ocs_atomic_init(&xport->fc_stats.stats.fcp_stats.output_bytes, 0);
	ocs_atomic_init(&xport->fc_stats.stats.fcp_stats.control_requests, 0);
#endif
	ocs_atomic_init(&xport->io_pending_count, 0);
	ocs_atomic_init(&xport->io_total_pending, 0);
	ocs_atomic_init(&xport->io_alloc_failed_count, 0);
	ocs_atomic_init(&xport->io_pending_recursing, 0);

	ocs_lock_init(ocs, &xport->fc_stats_lock, OCS_LOCK_ORDER_FC_STATS, "fc_stats_lock[%d]", ocs->instance_index);
	ocs_lock_init(ocs, &ocs->hal.watchdog_lock, OCS_LOCK_ORDER_IGNORE, "watchdog_lock[%d]", ocs_instance(ocs));

	rc = ocs_hal_init(&ocs->hal);
	if (rc) {
		ocs_log_err(ocs, "ocs_hal_init failure\n");
		goto ocs_xport_init_cleanup;
	} else {
		hal_initialized = TRUE;
	}

	if (ocs_xport_init_backends(ocs)) {
		ocs_log_err(ocs, "ocs_init_backends failed\n");
		goto ocs_xport_init_cleanup;
	}

	/* Add vports */
	if (ocs->num_vports != 0) {
		uint32_t max_vports;
		ocs_hal_get(&ocs->hal, OCS_HAL_MAX_VPORTS, &max_vports);

		if (ocs->num_vports <= max_vports) {
			ocs_log_debug(ocs, "Provisioning %d vports\n", ocs->num_vports);
			for (i = 0; i < ocs->num_vports; i++) {
				ocs_vport_create_spec(ocs, 0, 0, UINT32_MAX, ocs->enable_ini, ocs->ini_fc_types, ocs->enable_tgt, ocs->tgt_fc_types, NULL, NULL);
			}
		} else {
			ocs_log_err(ocs, "failed to create vports, num_vports range should be (1-%d)\n", max_vports);
			goto ocs_xport_init_cleanup;
		}
	}
	return 0;

ocs_xport_init_cleanup:
	if (hal_initialized) {
		/* ocs_hal_teardown can only execute after ocs_hal_init */
		ocs_hal_teardown(&ocs->hal);
	}

	return -1;
}

/**
 * @brief Detaches the transport from the device.
 *
 * @par Description
 * Performs the functions required to shut down a device.
 *
 * @param xport Pointer to transport object.
 *
 * @return Returns 0 on success or a non-zero value on failure.
 */
int32_t
ocs_xport_detach(ocs_xport_t *xport)
{
	ocs_t *ocs = xport->ocs;

	/* Free up any saved virtual ports */
	ocs_vport_del_all(ocs);

	ocs_xport_cleanup_backends(ocs);

	/*Shutdown FC Statistics timer*/
	ocs_del_timer(&xport->fc_stats_timer);

	ocs_hal_teardown(&ocs->hal);
	return 0;
}

/**
 * @brief domain list empty callback
 *
 * @par Description
 * Function is invoked when the device domain list goes empty. By convention
 * @c arg points to an ocs_sem_t instance, that is incremented.
 *
 * @param ocs Pointer to device object.
 * @param arg Pointer to semaphore instance.
 *
 * @return None.
 */
void
ocs_xport_domain_list_empty_cb(ocs_t *ocs, void *arg)
{
	ocs_sem_t *sem = arg;

	ocs_assert(ocs);
	ocs_assert(sem);

	ocs_sem_v(sem);
}

/**
 * @brief post node event callback
 *
 * @par Description
 * This function is called from the mailbox completion interrupt context to post an
 * event to a node object. By doing this in the interrupt context, it has
 * the benefit of only posting events in the interrupt context, deferring the need to
 * create a per event node lock.
 *
 * @param hal Pointer to HAL structure.
 * @param status Completion status for mailbox command.
 * @param mqe Mailbox queue completion entry.
 * @param arg Callback argument.
 *
 * @return Returns 0 on success, a negative error code value on failure.
 */

static int32_t
ocs_xport_post_node_event_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	ocs_xport_post_node_event_t *payload = arg;

	if (payload != NULL) {
		ocs_node_post_event(payload->node, payload->evt, payload->context);
		ocs_sem_v(&payload->sem);
		if (ocs_atomic_sub_and_test(&payload->refcnt, 1))
			ocs_free(hal->os, payload, sizeof(*payload));
	}

	return 0;
}

/**
 * @brief post vport shutdown event callback
 *
 * @par Description
 * This function is called from the mailbox completion interrupt context.
 * By doing this in the interrupt context, it has
 * the benefit of only posting events in the interrupt context.
 *
 * @param hal Pointer to HAL structure.
 * @param status Completion status for mailbox command.
 * @param mqe Mailbox queue completion entry.
 * @param arg Callback argument.
 *
 */
static int32_t
ocs_xport_post_vport_shutdown_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	ocs_t *ocs = hal->os;
	ocs_vport_args_t *vport_info = arg;
	ocs_domain_t *domain;
	
	if (vport_info != NULL) {
		domain = ocs_domain_get_instance(ocs, vport_info->domain_index);

		ocs_sport_vport_del(ocs, domain, vport_info->wwpn,
				vport_info->wwnn, vport_info->delete_vport);
		ocs_free(ocs, vport_info, sizeof(*vport_info));
	}	

	return 0;
}

/*
 * @brief Perform transport attach function.
 *
 * @par Description
 * Perform the attach function, which for the FC transport makes a HAL call
 * to bring up the link.
 *
 * @param xport pointer to transport object.
 * @param cmd command to execute.
 *
 * ocs_xport_control(ocs_xport_t *xport, OCS_XPORT_PORT_ONLINE)
 * ocs_xport_control(ocs_xport_t *xport, OCS_XPORT_PORT_OFFLINE)
 * ocs_xport_control(ocs_xport_t *xport, OCS_XPORT_PORT_SHUTDOWN)
 * ocs_xport_control(ocs_xport_t *xport, OCS_XPORT_POST_NODE_EVENT, ocs_node_t *node, ocs_sm_event_t, void *context)
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */

int32_t
ocs_xport_control(ocs_xport_t *xport, ocs_xport_ctrl_e cmd, ...)
{
	int32_t rc = 0;
	ocs_t *ocs = NULL;
	va_list argp;

	ocs_assert(xport, -1);
	ocs_assert(xport->ocs, -1);
	ocs = xport->ocs;

	switch (cmd) {
	case OCS_XPORT_PORT_ONLINE: {
		/* Bring the port on-line */
		rc = ocs_hal_port_control(&ocs->hal, OCS_HAL_PORT_INIT, 0, NULL, NULL);
		if (rc) {
			ocs_log_err(ocs, "%s: Can't init port\n", ocs->desc);
		} else {
			xport->configured_link_state = cmd;
		}
		break;
	}

	case OCS_XPORT_PORT_OFFLINE: {
		rc = ocs_hal_port_control(&ocs->hal, OCS_HAL_PORT_SHUTDOWN, 0, NULL, NULL);
		if (rc) {
			ocs_log_err(ocs, "port shutdown failed\n");
		} else {
			xport->configured_link_state = cmd;
		}
		break;
	}

	case OCS_XPORT_SHUTDOWN: {
		uint32_t reset_required;

		/* if a PHYSDEV reset was performed (e.g. hal dump), will affect
		 * all PCI functions; orderly shutdown won't work, just force free
		 */
		// TODO: need to poll this regularly...
		if (ocs_hal_get(&ocs->hal, OCS_HAL_RESET_REQUIRED, &reset_required) != OCS_HAL_RTN_SUCCESS) {
			reset_required = 0;
		}

		if (reset_required) {
			uint32_t err1 = sli_reg_read(&ocs->hal.sli, SLI4_REG_SLIPORT_ERROR1);
			uint32_t err2 = sli_reg_read(&ocs->hal.sli, SLI4_REG_SLIPORT_ERROR2);

			if (err1 == SLI4_SLIPORT_ERROR1_WDT_TIMEOUT &&
			    err2 == SLI4_SLIPORT_ERROR2_WDT_TIMEOUT &&
			    ocs->drv_ocs.shutting_down) {
				ocs_log_warn(ocs, "WDT timeout error. Perform hard shutdown\n");
				break;
			}
		} else if (ocs_hal_port_control(&ocs->hal, OCS_HAL_PORT_SHUTDOWN, 0, NULL, NULL)) {
			ocs_log_warn(ocs, "Port shutdown failed. Perform hard shutdown \n");
		}

		if (ocs_register_domain_list_empty_cb(ocs)) {
			ocs_log_debug(ocs, "waiting for domain to free\n");
			rc = ocs_drain_shutdown_events(ocs, &ocs->domain_list_empty_cb_sem);
			if (rc) {
				ocs_log_warn(ocs, "Domain shutdown timed out after %d sec; " \
						"Perform hard shutdown\n",
						(OCS_FC_DOMAIN_SHUTDOWN_TIMEOUT_USEC/1000000));
				ocs_log_info(ocs, "Print driver dump\n");
				ocs_ddump(ocs, NULL, OCS_DDUMP_FLAGS_DEFAULT, -1);
				ocs_hal_assert(0);
			}
			ocs_unregister_domain_list_empty_cb(ocs);
		}

		break;
	}

	/*
	 * POST_NODE_EVENT:  post an event to a node object
	 *
	 * This transport function is used to post an event to a node object. It does
	 * this by submitting a NOP mailbox command to defer execution to the
	 * interrupt context (thereby enforcing the serialized execution of event posting
	 * to the node state machine instances)
	 *
	 * A counting semaphore is used to make the call synchronous (we wait until
	 * the callback increments the semaphore before returning (or times out)
	 */
	case OCS_XPORT_POST_NODE_EVENT: {
		ocs_node_t *node;
		ocs_sm_event_t evt;
		void *context;
		ocs_xport_post_node_event_t *payload = NULL;
		ocs_t *ocs;
		ocs_hal_t *hal;

		/* Retrieve arguments */
		va_start(argp, cmd);
		node = va_arg(argp, ocs_node_t*);
		evt = va_arg(argp, ocs_sm_event_t);
		context = va_arg(argp, void *);
		va_end(argp);

		ocs_assert(node, -1);
		ocs_assert(node->ocs, -1);

		ocs = node->ocs;
		hal = &ocs->hal;

		/* if node's state machine is disabled, don't bother continuing */
		if (!node->sm.current_state) {
			ocs_log_test(ocs, "node %p state machine disabled\n", node);
			return -1;
		}

		payload = ocs_malloc(ocs, sizeof(*payload), OCS_M_ZERO);
		if (payload == NULL) {
			ocs_log_err(ocs, "failed to allocate memory\n");
			return -1;
		}

		/* Setup payload */
		ocs_sem_init(&payload->sem, 0, "xport_post_node_Event");
		ocs_atomic_init(&payload->refcnt, 2); /* one for self and one for callback */
		payload->node = node;
		payload->evt = evt;
		payload->context = context;

		if (ocs_hal_async_call(hal, ocs_xport_post_node_event_cb, payload)) {
			ocs_log_test(ocs, "ocs_hal_async_call failed\n");
			ocs_free(ocs, payload, sizeof(*payload));
			rc = -1;
			break;
		}

		/* Wait for completion */
		if (ocs_sem_p(&payload->sem, OCS_SEM_FOREVER)) {
			ocs_log_test(ocs, "POST_NODE_EVENT: sem wait failed\n");
			rc = -1;
		}

		if (ocs_atomic_sub_and_test(&payload->refcnt, 1))
			ocs_free(ocs, payload, sizeof(*payload));

		break;
	}
	case OCS_XPORT_POST_VPORT_SHUTDOWN: {
		ocs_vport_args_t *vport_args;

		vport_args = ocs_malloc(ocs, sizeof(ocs_vport_args_t), OCS_M_ZERO);
		if (!vport_args) {
			ocs_log_test(ocs, "failed to allocate memory\n");
			rc = -1;
			break;
		}
		/* Retrieve arguments */
		va_start(argp, cmd);
		vport_args->wwpn = va_arg(argp, uint64_t);
		vport_args->wwnn = va_arg(argp, uint64_t);
		vport_args->domain_index = va_arg(argp, uint32_t);
		vport_args->delete_vport = va_arg(argp, int);
		va_end(argp);

		if (ocs_hal_async_call(&ocs->hal, ocs_xport_post_vport_shutdown_cb, vport_args)) {
			ocs_log_err(ocs, "ocs_hal_async_call failed\n");
			ocs_free(ocs, vport_args, sizeof(*vport_args));
			rc = -1;
		}

		break;
	}
	case OCS_XPORT_POST_NODE_SHUTDOWN: {
		ocs_node_t *node;
		ocs_sm_event_t evt;
		void *context;
		ocs_t *ocs;

		/* Retrieve arguments */
		va_start(argp, cmd);
		node = va_arg(argp, ocs_node_t*);
		evt = va_arg(argp, ocs_sm_event_t);
		context = va_arg(argp, void *);
		va_end(argp);

		ocs_assert(node, -1);
		ocs_assert(node->ocs, -1);

		ocs = node->ocs;

		/* if node's state machine is disabled, don't bother continuing */
		if (!node->sm.current_state) {
			ocs_log_test(ocs, "node %p state machine disabled\n", node);
			return -1;
		}

		ocs_node_post_event(node, evt, context);
		break;
	}

	/*
	 * Set wwnn for the port.  This will be used instead of the default provided by FW.
	 */
	case OCS_XPORT_WWNN_SET: {
		uint64_t wwnn;

		/* Retrieve arguments */
		va_start(argp, cmd);
		wwnn = va_arg(argp, uint64_t);
		va_end(argp);

		ocs_log_debug(ocs, "WWNN %016" PRIx64 "\n", wwnn);
		xport->req_wwnn = wwnn;
		break;
	}

	/*
	 * Set wwpn for the port.  This will be used instead of the default provided by FW.
	 */
	case OCS_XPORT_WWPN_SET: {
		uint64_t wwpn;

		/* Retrieve arguments */
		va_start(argp, cmd);
		wwpn = va_arg(argp, uint64_t);
		va_end(argp);

		ocs_log_debug(ocs, "WWPN %016" PRIx64 "\n", wwpn);
		xport->req_wwpn = wwpn;
		break;
	}

	default:
		break;
	}

	return rc;
}

/**
 * @brief Return status on a link.
 *
 * @par Description
 * Returns status information about a link.
 *
 * @param xport Pointer to transport object.
 * @param cmd Command to execute.
 * @param result Pointer to result value.
 *
 * ocs_xport_status(ocs_xport_t *xport, OCS_XPORT_PORT_STATUS)
 * ocs_xport_status(ocs_xport_t *xport, OCS_XPORT_LINK_SPEED, ocs_xport_stats_t *result)
 *	return link speed in MB/sec
 * ocs_xport_status(ocs_xport_t *xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, ocs_xport_stats_t *result)
 *	[in] *result is speed to check in MB/s
 *	returns 1 if supported, 0 if not
 * ocs_xport_status(ocs_xport_t *xport, OCS_XPORT_LINK_STATISTICS, ocs_xport_stats_t *result)
 *	return link/host port stats
 * ocs_xport_status(ocs_xport_t *xport, OCS_XPORT_LINK_STAT_RESET, ocs_xport_stats_t *result)
 *	resets link/host stats
 *
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */

int32_t
ocs_xport_status(ocs_xport_t *xport, ocs_xport_status_e cmd, ocs_xport_stats_t *result)
{
	uint32_t rc = 0;
	ocs_t *ocs = NULL;
	ocs_xport_stats_t value;
	ocs_hal_rtn_e hal_rc;

	ocs_assert(xport, -1);
	ocs_assert(xport->ocs, -1);

	ocs = xport->ocs;

	switch (cmd) {
	case OCS_XPORT_CONFIG_PORT_STATUS:
		ocs_assert(result, -1);
		if (xport->configured_link_state == 0) {
			/* Initial state is offline. configured_link_state is    */
			/* set to online explicitly when port is brought online. */
			xport->configured_link_state = OCS_XPORT_PORT_OFFLINE;
		}
		result->value = xport->configured_link_state;
		break;

	case OCS_XPORT_PORT_STATUS:
		ocs_assert(result, -1);
		/* Determine port status based on link speed. */
		hal_rc = ocs_hal_get(&(ocs->hal), OCS_HAL_LINK_SPEED, &value.value);
		if (hal_rc == OCS_HAL_RTN_SUCCESS) {
			if (value.value == 0) {
				result->value = 0;
			} else {
				result->value = 1;
			}
			rc = 0;
		} else {
			rc = -1;
		}
		break;

	case OCS_XPORT_LINK_SPEED: {
		uint32_t speed;

		ocs_assert(result, -1);
		result->value = 0;

		rc = ocs_hal_get(&ocs->hal, OCS_HAL_LINK_SPEED, &speed);
		if (rc == 0) {
			result->value = speed;
		}
		break;
	}

	case OCS_XPORT_LOGICAL_LINK_SPEED: {
		uint32_t speed;

		ocs_assert(result, -1);
		result->value = 0;

		rc = ocs_hal_get(&ocs->hal, OCS_HAL_LOGICAL_LINK_SPEED, &speed);
		if (rc == 0)
			result->value = speed;

		break;
	}

	case OCS_XPORT_IS_SUPPORTED_LINK_SPEED: {
		uint32_t speed;
		uint32_t link_module_type;
		uint32_t aggregate_link_speed;

		ocs_assert(result, -1);
		speed = result->value;

		/*
		 * Look for aggregate link speed to handle the trunked links.
		 * The link module property is immaterial in this case.
		 */
		if (!ocs_hal_get(&ocs->hal, OCS_HAL_AGGREGATE_LINK_SPEED, &aggregate_link_speed)) {
			if (aggregate_link_speed) {
				rc = ((speed == aggregate_link_speed) ? true : false);
				break;
			}
		}

		rc = ocs_hal_get(&ocs->hal, OCS_HAL_LINK_MODULE_TYPE, &link_module_type);
		if (rc == 0) {
			switch(speed) {
			case 1000:	rc = (link_module_type & OCS_HAL_LINK_MODULE_TYPE_1GB) != 0; break;
			case 2000:	rc = (link_module_type & OCS_HAL_LINK_MODULE_TYPE_2GB) != 0; break;
			case 4000:	rc = (link_module_type & OCS_HAL_LINK_MODULE_TYPE_4GB) != 0; break;
			case 8000:	rc = (link_module_type & OCS_HAL_LINK_MODULE_TYPE_8GB) != 0; break;
			case 10000:	rc = (link_module_type & OCS_HAL_LINK_MODULE_TYPE_10GB) != 0; break;
			case 16000:	rc = (link_module_type & OCS_HAL_LINK_MODULE_TYPE_16GB) != 0; break;
			case 32000:	rc = (link_module_type & OCS_HAL_LINK_MODULE_TYPE_32GB) != 0; break;
			case 64000:	rc = (link_module_type & OCS_HAL_LINK_MODULE_TYPE_64GB) != 0; break;
			case 128000:	rc = (link_module_type & OCS_HAL_LINK_MODULE_TYPE_128GB) != 0; break;
			case 256000:	rc = (link_module_type & OCS_HAL_LINK_MODULE_TYPE_256GB) != 0; break;
			default:	rc = 0; break;
			}
		} else {
			rc = 0;
		}
		break;
	}
	case OCS_XPORT_LINK_STATISTICS:
		ocs_lock(&ocs->xport->fc_stats_lock);
			ocs_memcpy((void *)result, &ocs->xport->fc_stats_sysfs, sizeof(ocs_xport_stats_t));
		ocs_unlock(&ocs->xport->fc_stats_lock);
		break;
	case OCS_XPORT_LINK_STAT_RESET: {
		/* Create a semaphore to synchronize the stat reset process. */
		ocs_sem_init(&(result->stats.semaphore), 0, "fc_stats_reset");

		/* Reset LINK stats retrived after LIP */
		ocs_memset(&ocs->xport->fc_stats.linkup_stats, 0,
				sizeof(ocs->xport->fc_stats.linkup_stats));

		/* First reset the link stats */
		rc = ocs_hal_get_link_stats(&ocs->hal, 0, 1, 1, ocs_xport_link_stats_cb, result);
		if (rc) {
			ocs_log_err(ocs, "Failed to reset link statistics\n");
			break;
		}

		/* Wait for semaphore to be signaled when the command completes */
		// TODO:  Should there be a timeout on this?  If so, how long?
		if (ocs_sem_p(&(result->stats.semaphore), OCS_SEM_FOREVER) != 0) {
			/* Undefined failure */
			ocs_log_test(ocs, "ocs_sem_p failed\n");
			rc = -ENXIO;
			break;
		}

		/* Next reset the host stats */
		rc = ocs_hal_get_host_stats(&ocs->hal, 1, ocs_xport_host_stats_cb, result);
		if (rc) {
			ocs_log_err(ocs, "Failed to reset host statistics\n");
			break;
		}

		/* Wait for semaphore to be signaled when the command completes */
		// TODO:  Should there be a timeout on this?  If so, how long?
		if (ocs_sem_p(&(result->stats.semaphore), OCS_SEM_FOREVER) != 0) {
			/* Undefined failure */
			ocs_log_test(ocs, "ocs_sem_p failed\n");
			rc = -ENXIO;
			break;
		}
		break;
	}
	case OCS_XPORT_IS_QUIESCED:
		ocs_device_lock(ocs);
			result->value = ocs_list_empty(&ocs->domain_list);
		ocs_device_unlock(ocs);
		break;
	default:
		rc = -1;
		break;
	}

	return rc;
}

void
ocs_xport_get_linkup_stats(ocs_t *ocs)
{
	ocs_xport_stats_t *result = &ocs->xport->fc_stats;

	/* First reset the link stats */
	ocs_hal_get_link_stats(&ocs->hal, 0, 0, 0, ocs_xport_linkup_stats_cb, result);
}

static void
ocs_xport_link_stats_cb(int32_t status, uint32_t num_counters, ocs_hal_link_stat_counts_t *counters, void *arg)
{
	ocs_xport_stats_t *result = arg;

	result->stats.link_stats.link_failure_error_count = counters[OCS_HAL_LINK_STAT_LINK_FAILURE_COUNT].counter;
	result->stats.link_stats.loss_of_sync_error_count = counters[OCS_HAL_LINK_STAT_LOSS_OF_SYNC_COUNT].counter;
	result->stats.link_stats.primitive_sequence_error_count = counters[OCS_HAL_LINK_STAT_PRIMITIVE_SEQ_COUNT].counter;
	result->stats.link_stats.invalid_transmission_word_error_count = counters[OCS_HAL_LINK_STAT_INVALID_XMIT_WORD_COUNT].counter;
	result->stats.link_stats.crc_error_count = counters[OCS_HAL_LINK_STAT_CRC_COUNT].counter;

	ocs_sem_v(&(result->stats.semaphore));
}

static void
ocs_xport_linkup_stats_cb(int32_t status, uint32_t num_counters, ocs_hal_link_stat_counts_t *counters, void *arg)
{
	ocs_xport_stats_t *result = arg;

	result->linkup_stats.link_stats.link_failure_error_count = counters[OCS_HAL_LINK_STAT_LINK_FAILURE_COUNT].counter;
	result->linkup_stats.link_stats.loss_of_sync_error_count = counters[OCS_HAL_LINK_STAT_LOSS_OF_SYNC_COUNT].counter;
	result->linkup_stats.link_stats.primitive_sequence_error_count = counters[OCS_HAL_LINK_STAT_PRIMITIVE_SEQ_COUNT].counter;
	result->linkup_stats.link_stats.invalid_transmission_word_error_count = counters[OCS_HAL_LINK_STAT_INVALID_XMIT_WORD_COUNT].counter;
	result->linkup_stats.link_stats.crc_error_count = counters[OCS_HAL_LINK_STAT_CRC_COUNT].counter;
	result->linkup_stat_req = 1;
}

static void
ocs_xport_host_stats_cb(int32_t status, uint32_t num_counters, ocs_hal_host_stat_counts_t *counters, void *arg)
{
	ocs_xport_stats_t *result = arg;

	result->stats.host_stats.transmit_kbyte_count = counters[OCS_HAL_HOST_STAT_TX_KBYTE_COUNT].counter;
	result->stats.host_stats.receive_kbyte_count = counters[OCS_HAL_HOST_STAT_RX_KBYTE_COUNT].counter;
	result->stats.host_stats.transmit_frame_count = counters[OCS_HAL_HOST_STAT_TX_FRAME_COUNT].counter;
	result->stats.host_stats.receive_frame_count = counters[OCS_HAL_HOST_STAT_RX_FRAME_COUNT].counter;

	ocs_sem_v(&(result->stats.semaphore));
}

static void
ocs_xport_async_link_stats_cb(int32_t status, uint32_t num_counters, ocs_hal_link_stat_counts_t *counters, void *arg)
{
	ocs_xport_t *xport = (ocs_xport_t *)arg;
	ocs_xport_stats_t *result;

	ocs_assert(xport);
	result = &xport->fc_stats;

	ocs_lock(&xport->fc_stats_lock);
	result->stats.link_stats.link_failure_error_count = counters[OCS_HAL_LINK_STAT_LINK_FAILURE_COUNT].counter;
	result->stats.link_stats.loss_of_sync_error_count = counters[OCS_HAL_LINK_STAT_LOSS_OF_SYNC_COUNT].counter;
	result->stats.link_stats.primitive_sequence_error_count = counters[OCS_HAL_LINK_STAT_PRIMITIVE_SEQ_COUNT].counter;
	if (result->linkup_stat_req) {
		if (result->linkup_stats.link_stats.invalid_transmission_word_error_count)
			result->stats.link_stats.invalid_transmission_word_error_count =
					counters[OCS_HAL_LINK_STAT_INVALID_XMIT_WORD_COUNT].counter -
					result->linkup_stats.link_stats.invalid_transmission_word_error_count;
		else
			result->stats.link_stats.invalid_transmission_word_error_count =
					counters[OCS_HAL_LINK_STAT_INVALID_XMIT_WORD_COUNT].counter;
	}

	/* Notify user if there is a change in LRR counter compared to previous value */
	if (result->stats.link_stats.lrr_count_local != counters[OCS_HAL_LINK_STAT_LRR_COUNT_LOCAL].counter) {
		ocs_log_warn(xport->ocs, "Received LRR local counter %s on Port: %d (previous value: 0x%x "
			"current value: 0x%x)\n", (counters[OCS_HAL_LINK_STAT_LRR_COUNT_LOCAL].counter >
			result->stats.link_stats.lrr_count_local) ? "Incremented" : "Decremented",
			xport->ocs->hal.sli.physical_port, result->stats.link_stats.lrr_count_local,
			counters[OCS_HAL_LINK_STAT_LRR_COUNT_LOCAL].counter);
	}

	/* Notify user if there is a change in LR counter compared to previous value */
	if (result->stats.link_stats.lr_count_remote != counters[OCS_HAL_LINK_STAT_LR_COUNT_REMOTE].counter) {
		ocs_log_warn(xport->ocs, "Received LR remote counter %s on Port %d (previous value: 0x%x "
			"current value: 0x%x)\n", (counters[OCS_HAL_LINK_STAT_LR_COUNT_REMOTE].counter >
			result->stats.link_stats.lr_count_remote) ? "Incremented" : "Decremented",
			xport->ocs->hal.sli.physical_port, result->stats.link_stats.lr_count_remote,
			counters[OCS_HAL_LINK_STAT_LR_COUNT_REMOTE].counter);
	}

	result->stats.link_stats.lrr_count_local = counters[OCS_HAL_LINK_STAT_LRR_COUNT_LOCAL].counter;
	result->stats.link_stats.lr_count_remote = counters[OCS_HAL_LINK_STAT_LR_COUNT_REMOTE].counter;
	result->stats.link_stats.crc_error_count = counters[OCS_HAL_LINK_STAT_CRC_COUNT].counter;
	ocs_unlock(&xport->fc_stats_lock);
}

const char *OCS_HOST_STAT_NAME[] = {
	"TX_KBYTE_COUNT",
	"RX_KBYTE_COUNT",
	"TX_FRAME_COUNT",
	"RX_FRAME_COUNT"
};

/**
 * @brief copy the statistics to the cache
 *
 * @par Description
 * Copies the statistics value returned by firmware to the driver's
 * cache. If the stats have rolled over, update the wrap_counter in
 * the upper 32-bits and copy the new stats value to lower 32 bits.
 *
 * @param old_stat: The driver's copy of statistics maintained as 64 bits.
 *                Upper 32 bits maintain the wrap_counter, i.e., number of times
 *                the counter has rolled over the 32 bit boundary.
 *
 * @param new_stat: The 32 statistics returned by the firmware.
 *
 * @param stat_type: The type of stats being updated.
 *
 * @return void
 */

static void
ocs_wrap_and_copy_stats(uint64_t *old_stat, uint32_t new_stat, uint32_t stat_type)
{
	uint32_t *ptr_low = (uint32_t *)old_stat;

	/* if stats have wrappped over, increment the wrap_counter */
	if (*ptr_low > new_stat) {
		*old_stat = *old_stat + OCS_WRAP_INCR;
	}
	/* update the stats */
	*ptr_low = new_stat;
}

static void
ocs_xport_async_host_stats_cb(int32_t status, uint32_t num_counters, ocs_hal_host_stat_counts_t *counters, void *arg)
{
	ocs_xport_t *xport = (ocs_xport_t *)arg;
	ocs_xport_stats_t *result;

	ocs_assert(xport);
	result = &xport->fc_stats;

	ocs_lock(&xport->fc_stats_lock);
	ocs_wrap_and_copy_stats(&result->stats.host_stats.transmit_kbyte_count,
				counters[OCS_HAL_HOST_STAT_TX_KBYTE_COUNT].counter, OCS_HAL_HOST_STAT_TX_KBYTE_COUNT);
	ocs_wrap_and_copy_stats(&result->stats.host_stats.receive_kbyte_count,
				counters[OCS_HAL_HOST_STAT_RX_KBYTE_COUNT].counter, OCS_HAL_HOST_STAT_RX_KBYTE_COUNT);
	ocs_wrap_and_copy_stats(&result->stats.host_stats.transmit_frame_count,
				counters[OCS_HAL_HOST_STAT_TX_FRAME_COUNT].counter, OCS_HAL_HOST_STAT_TX_FRAME_COUNT);
	ocs_wrap_and_copy_stats(&result->stats.host_stats.receive_frame_count,
				counters[OCS_HAL_HOST_STAT_RX_FRAME_COUNT].counter, OCS_HAL_HOST_STAT_RX_FRAME_COUNT);
	result->stats.host_stats.transmit_sequence_count= counters[OCS_HAL_HOST_STAT_TX_SEQ_COUNT].counter;
	result->stats.host_stats.receive_sequence_count= counters[OCS_HAL_HOST_STAT_RX_SEQ_COUNT].counter;
	result->stats.host_stats.total_exchanges_originator= counters[OCS_HAL_HOST_STAT_TOTAL_EXCH_ORIG].counter;
	result->stats.host_stats.total_exchanges_responder= counters[OCS_HAL_HOST_STAT_TOTAL_EXCH_RESP].counter;
	result->stats.host_stats.receive_p_bsy_count= counters[OCS_HAL_HOSY_STAT_RX_P_BSY_COUNT].counter;
	result->stats.host_stats.receive_f_bsy_count= counters[OCS_HAL_HOST_STAT_RX_F_BSY_COUNT].counter;
	ocs_unlock(&xport->fc_stats_lock);
}


void
ocs_xport_setup_stats_timer(ocs_t *ocs)
{
	if (!ocs->xport) {
		ocs_log_err(ocs, "xport object is NULL, can't schedule the fc stats timer\n");
		return;
	}

#if !defined(OCS_USPACE_RAMD)
	if (!ocs_scsi_host_ml_registered(ocs)) {
		ocs_log_err(ocs, "scsi host ml not registered, no fc stats timer\n");
		return;
	};
#endif

	ocs_setup_timer(ocs->hal.os, &ocs->xport->fc_stats_timer,
			ocs_xport_stats_timer_fn, ocs,
			OCS_XPORT_STATS_TIMER_MS, true);
}

/**
 * @brief Get FC link and host Statistics periodically
 *
 * @param hal Hardware context.
 *
 * @return NONE.
 */
void
ocs_xport_stats_timer_fn(void *arg)
{
	ocs_t *ocs = (ocs_t *)arg;
	ocs_xport_stats_t *fc_stats_sysfs, *fc_stats;

	if (!ocs) {
		ocs_log_err(ocs, "failed to locate OCS device\n");
		return;
	}

	if (!ocs->xport) {
		ocs_log_err(ocs, "xport object is NULL, can't schedule the fc stats timer\n");
		return;
	}

	fc_stats_sysfs = &ocs->xport->fc_stats_sysfs;
	fc_stats = &ocs->xport->fc_stats;

	/* Copy the cache'd stats */
	ocs_lock(&ocs->xport->fc_stats_lock);
	ocs_memcpy(fc_stats_sysfs, fc_stats, sizeof(ocs_xport_stats_t));

	fc_stats_sysfs->stats.host_stats.transmit_kbyte_count = fc_stats->stats.host_stats.transmit_kbyte_count;
	fc_stats_sysfs->stats.host_stats.receive_kbyte_count = fc_stats->stats.host_stats.receive_kbyte_count;
	fc_stats_sysfs->stats.host_stats.transmit_frame_count = fc_stats->stats.host_stats.transmit_frame_count;
	fc_stats_sysfs->stats.host_stats.receive_frame_count = fc_stats->stats.host_stats.receive_frame_count;
	ocs_unlock(&ocs->xport->fc_stats_lock);

	ocs_hal_get_link_stats(&ocs->hal, 1, 0, 0, ocs_xport_async_link_stats_cb, ocs->xport);
	ocs_hal_get_host_stats(&ocs->hal, 0, ocs_xport_async_host_stats_cb, ocs->xport);

	ocs_mod_timer(&ocs->xport->fc_stats_timer, OCS_XPORT_STATS_TIMER_MS);
}

/**
 * @brief Free a transport object.
 *
 * @par Description
 * The transport object is freed.
 *
 * @param xport Pointer to transport object.
 *
 * @return None.
 */

void
ocs_xport_free(ocs_xport_t *xport)
{
	ocs_t *ocs;
	uint32_t i;

	if (xport) {
#if !defined(OCS_USPACE)
		percpu_counter_destroy(&xport->fc_stats.stats.fcp_stats.input_requests);
		percpu_counter_destroy(&xport->fc_stats.stats.fcp_stats.input_bytes);
		percpu_counter_destroy(&xport->fc_stats.stats.fcp_stats.output_requests);
		percpu_counter_destroy(&xport->fc_stats.stats.fcp_stats.output_bytes);
		percpu_counter_destroy(&xport->fc_stats.stats.fcp_stats.control_requests);
#endif

		ocs = xport->ocs;
		ocs_io_pool_free(xport->io_pool);
		ocs_io_pool_free(xport->els_io_pool);
		ocs_node_free_pool(ocs);

		ocs_lock_free(&xport->io_pending_lock);

		for (i = 0; i < SLI4_MAX_FCFI; i++) {
			ocs_lock_free(&xport->fcfi[i].pend_frames_lock);
		}

		ocs_xport_rq_threads_teardown(xport);
		ocs_lock_free(&xport->fc_stats_lock);
		ocs_free(ocs, xport, sizeof(*xport));
	}
}

/**
 * @brief callback function for xport from HAL.
 *
 * @par Description
 * Handling XPORT event
 *
 * @param OCS context.
 *
 * @return status.
 */
int32_t
ocs_xport_cb(void *arg, ocs_hal_xport_event_e event)
{
	ocs_t *ocs = arg;

	if (!ocs) {
		ocs_log_err(ocs, "Invalid argument\n");
		return -1;
	}

	switch (event) {
	case OCS_HAL_XPORT_FC_SPEED_UPDATE:
		/* Updated supported FC speeds */
		ocs_scsi_host_update_supported_speed(ocs);
		break;
	default:
		ocs_log_err(ocs, "Unsupported XPORT event type\n");
		return -1;
	}

	return 0;
}
