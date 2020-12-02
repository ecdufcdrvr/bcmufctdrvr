/*
 * Copyright (C) 2020 Broadcom. All Rights Reserved.
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
 * Defines and implements the Hardware Abstraction Layer (HAL).
 * All interaction with the hardware is performed through the HAL, which abstracts
 * the details of the underlying SLI-4 implementation.
 */

/**
 * @defgroup devInitShutdown Device Initialization and Shutdown
 * @defgroup domain Domain Functions
 * @defgroup port Port Functions
 * @defgroup node Remote Node Functions
 * @defgroup io IO Functions
 * @defgroup interrupt Interrupt handling
 * @defgroup os OS Required Functions
 */

#include "ocs.h"
#include "ocs_os.h"
#include "ocs_hal.h"
#include "ocs_hal_queues.h"
#include "ocs_recovery.h"
#include "ocs_ras.h"
#include "ocs_compat.h"

#define OCS_HAL_READ_FCF_SIZE			4096
#define OCS_HAL_DEFAULT_AUTO_XFER_RDY_IOS	256
#define OCS_HAL_WQ_TIMER_PERIOD_MS		500

/* Values used for setting the optimized write parameters */
#define OCS_HAL_TOW_BLK_SIZE_DEFAULT		0	/* 512 bytes */
#define OCS_HAL_TOW_REF_TAG_LBA_DEFAULT		TRUE
#define OCS_HAL_TOW_APP_TAG_VALID_DEFAULT	FALSE
#define OCS_HAL_TOW_APP_TAG_VALUE_DEFAULT	0

/**
 * By design, driver elects not to receive completion CQE for REQUEUE_XRI WQE.
 * However, FW will still generate a CQE if it encounters an error while processing
 * REQUEUE_XRI request. In order to handle the completion CQE with error status,
 * driver pre-allocates & sets aside a completion context for REQUEUE_XRI WQE.
 */
#define OCS_HAL_REQUE_XRI_REGTAG			65534

/* max command and response buffer lengths -- arbitrary at the moment */
#define OCS_HAL_DMTF_CLP_CMD_MAX	256
#define OCS_HAL_DMTF_CLP_RSP_MAX	256

/* HAL global data */
ocs_hal_global_t hal_global;

static void ocs_hal_queue_hash_add(ocs_queue_hash_t *, uint16_t, uint16_t);
static void ocs_hal_adjust_wqs(ocs_hal_t *hal);
static uint32_t ocs_hal_get_num_chutes(ocs_hal_t *hal);
static int32_t ocs_hal_cb_link(void *, void *);
static int32_t ocs_hal_cb_fip(void *, void *);
static int32_t ocs_hal_command_process(ocs_hal_t *, int32_t, uint8_t *, size_t);
static int32_t ocs_hal_mq_process(ocs_hal_t *, int32_t, sli4_queue_t *);
static int32_t ocs_hal_cb_read_fcf(ocs_hal_t *, int32_t, uint8_t *, void *);
static int32_t ocs_hal_cb_node_attach(ocs_hal_t *, int32_t, uint8_t *, void *);
static int32_t ocs_hal_cb_node_free(ocs_hal_t *, int32_t, uint8_t *, void *);
static int32_t ocs_hal_cb_unreg_rpi_all(ocs_hal_t *, int32_t, uint8_t *, void *);
static ocs_hal_rtn_e ocs_hal_setup_io(ocs_hal_t *);
static ocs_hal_rtn_e ocs_hal_init_io(ocs_hal_t *);
static int32_t ocs_hal_command_cancel(ocs_hal_t *);
static void ocs_hal_io_quarantine(ocs_hal_t *hal, hal_wq_t *wq, ocs_hal_io_t *io);
static void ocs_hal_io_restore_sgl(ocs_hal_t *, ocs_hal_io_t *);
static int32_t ocs_hal_io_ini_sge(ocs_hal_t *, ocs_hal_io_t *, ocs_dma_t *, uint32_t, ocs_dma_t *);
static ocs_hal_rtn_e ocs_hal_firmware_write_lancer(ocs_hal_t *hal, ocs_dma_t *dma, uint32_t size, uint32_t offset, int last, ocs_hal_fw_cb_t cb, void *arg);
static int32_t ocs_hal_cb_fw_write(ocs_hal_t *, int32_t, uint8_t *, void  *);
static int32_t ocs_hal_cb_sfp(ocs_hal_t *, int32_t, uint8_t *, void  *);
static int32_t ocs_hal_cb_temp(ocs_hal_t *, int32_t, uint8_t *, void  *);
static int32_t ocs_hal_cb_link_stat(ocs_hal_t *, int32_t, uint8_t *, void  *);
static int32_t ocs_hal_cb_host_stat(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg);
static void ocs_hal_dmtf_clp_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg);
static int32_t ocs_hal_clp_resp_get_value(ocs_hal_t *hal, const char *keyword, char *value, uint32_t value_len, const char *resp, uint32_t resp_len);
typedef void (*ocs_hal_dmtf_clp_cb_t)(ocs_hal_t *hal, int32_t status, uint32_t result_len, void *arg);
static ocs_hal_rtn_e ocs_hal_exec_dmtf_clp_cmd(ocs_hal_t *hal, ocs_dma_t *dma_cmd, ocs_dma_t *dma_resp, uint32_t opts, ocs_hal_dmtf_clp_cb_t cb, void *arg);
static void ocs_hal_linkcfg_dmtf_clp_cb(ocs_hal_t *hal, int32_t status, uint32_t result_len, void *arg);

static int32_t __ocs_read_topology_cb(ocs_hal_t *, int32_t, uint8_t *, void *);
static int32_t __ocs_read_fec_cb(ocs_hal_t *, int32_t, uint8_t *, void *);
static ocs_hal_rtn_e ocs_hal_get_linkcfg(ocs_hal_t *, uint32_t, ocs_hal_port_control_cb_t, void *);
static ocs_hal_rtn_e ocs_hal_get_linkcfg_lancer(ocs_hal_t *, uint32_t, ocs_hal_port_control_cb_t, void *);
static ocs_hal_rtn_e ocs_hal_get_linkcfg_skyhawk(ocs_hal_t *, uint32_t, ocs_hal_port_control_cb_t, void *);
static ocs_hal_rtn_e ocs_hal_set_linkcfg(ocs_hal_t *, ocs_hal_linkcfg_e, uint32_t, ocs_hal_port_control_cb_t, void *);
static ocs_hal_rtn_e ocs_hal_set_linkcfg_lancer(ocs_hal_t *, ocs_hal_linkcfg_e, uint32_t, ocs_hal_port_control_cb_t, void *);
static ocs_hal_rtn_e ocs_hal_set_linkcfg_skyhawk(ocs_hal_t *, ocs_hal_linkcfg_e, uint32_t, ocs_hal_port_control_cb_t, void *);
static void ocs_hal_init_linkcfg_cb(int32_t status, uintptr_t value, void *arg);
static ocs_hal_rtn_e ocs_hal_set_eth_license(ocs_hal_t *hal, uint32_t license);
static ocs_hal_rtn_e ocs_hal_set_dif_seed(ocs_hal_t *hal);
static ocs_hal_rtn_e ocs_hal_set_dif_mode(ocs_hal_t *hal);
static void ocs_hal_io_free_internal(void *arg);
static void ocs_hal_io_free_port_owned(void *arg);
static ocs_hal_rtn_e ocs_hal_config_axr_t10pi(ocs_hal_t *hal);
static ocs_hal_rtn_e ocs_hal_config_tow_t10pi(ocs_hal_t *hal);
static ocs_hal_rtn_e ocs_hal_config_set_fdt_xfer_hint(ocs_hal_t *hal, uint32_t fdt_xfer_hint);
static void ocs_hal_wq_process_abort(void *arg, uint8_t *cqe, int32_t status);
static int32_t ocs_hal_config_mrq(ocs_hal_t *hal, uint8_t, uint16_t, uint16_t);
static ocs_hal_rtn_e ocs_hal_config_watchdog_timer(ocs_hal_t *hal);
static void ocs_watchdog_timer(void *arg);
static ocs_hal_rtn_e ocs_hal_config_sli_port_health_check(ocs_hal_t *hal, uint8_t query, uint8_t enable);
static ocs_hal_rtn_e ocs_hal_config_sliport_pause(ocs_hal_t *hal, uint8_t enable);
static ocs_hal_rtn_e ocs_hal_config_fec(ocs_hal_t *hal, uint8_t enable);
static int32_t ocs_hal_cb_host_stat(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg);
static void __ocs_hal_read_lmt_cb(ocs_hal_t *, int32_t, uint8_t *, void *);
static ocs_hal_rtn_e ocs_hal_config_fw_ag_rsp(ocs_hal_t *hal, uint8_t enable);
static ocs_hal_rtn_e ocs_hal_exec_mbx_command_generic_results(ocs_hal_t *hal, uint8_t *req_rsp_buf,
							      bool handle_sli4_rsp, int timeout_usecs);

/* HAL domain database operations */
static int32_t ocs_hal_domain_add(ocs_hal_t *, ocs_domain_t *);
static int32_t ocs_hal_domain_del(ocs_hal_t *, ocs_domain_t *);


/* Port state machine */
static void *__ocs_hal_port_alloc_init(ocs_sm_ctx_t *, ocs_sm_event_t, void *);
static void *__ocs_hal_port_alloc_read_sparm64(ocs_sm_ctx_t *, ocs_sm_event_t, void *);
static void *__ocs_hal_port_alloc_init_vpi(ocs_sm_ctx_t *, ocs_sm_event_t, void *);
static void *__ocs_hal_port_done(ocs_sm_ctx_t *, ocs_sm_event_t, void *);
static void *__ocs_hal_port_free_unreg_vpi(ocs_sm_ctx_t *, ocs_sm_event_t, void *);

/* Domain state machine */
static void *__ocs_hal_domain_init(ocs_sm_ctx_t *, ocs_sm_event_t, void *);
static void *__ocs_hal_domain_alloc_reg_fcfi(ocs_sm_ctx_t *, ocs_sm_event_t, void *);
static void * __ocs_hal_domain_alloc_init_vfi(ocs_sm_ctx_t *, ocs_sm_event_t, void *);
static void *__ocs_hal_domain_free_unreg_vfi(ocs_sm_ctx_t *, ocs_sm_event_t, void *);
static void *__ocs_hal_domain_free_unreg_fcfi(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data);
static int32_t __ocs_hal_domain_cb(ocs_hal_t *, int32_t, uint8_t *, void *);
static int32_t __ocs_hal_port_cb(ocs_hal_t *, int32_t, uint8_t *, void *);
static int32_t __ocs_hal_port_realloc_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg);

/* BZ 161832 */
static void ocs_hal_check_sec_hio_list(ocs_hal_t *hal);

/* WQE timeouts */
static void target_wqe_timer_cb(void *arg);
static void shutdown_target_wqe_timer(ocs_hal_t *hal);

/* Adapter firmware diagnostic logging (RAS) */
static ocs_hal_rtn_e ocs_hal_config_fw_diagnostic_logging(ocs_hal_t *hal, int32_t log_size, int32_t log_level);
static ocs_hal_rtn_e ocs_hal_teardown_fw_diagnostic_logging(ocs_hal_t *hal);
static ocs_hal_rtn_e ocs_hal_config_set_dual_dump(ocs_hal_t *hal);

static void
ocs_hal_log_sli4_rsp_hdr(ocs_hal_t *hal, sli4_res_hdr_t *hdr)
{
	ocs_log_err(hal->os, "SLI4_RSP: cmd_opcode: %#x, cmd_subsystem: %#x, status: %#x, addl_status: %#x\n",
		    hdr->opcode, hdr->subsystem, hdr->status, hdr->additional_status);
	ocs_log_err(hal->os, "          %08X %08X %08X %08X\n", ((uint32_t *)hdr)[0],
		    ((uint32_t *)hdr)[1], ((uint32_t *)hdr)[2], ((uint32_t *)hdr)[3]);
}

static ocs_hal_rtn_e
ocs_hal_init_tow_esoc(ocs_hal_t *hal, uint32_t tow_feature)
{
	uint32_t ramdisc_blocksize = 512;
	uint8_t buf[SLI4_BMBX_SIZE];
	int32_t len = 0;
	char prop_buf[32];
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;

	if (ocs_get_property("ramdisc_blocksize", prop_buf, sizeof(prop_buf)) == 0)
		ramdisc_blocksize = ocs_strtoul(prop_buf, 0, 0);

	if (sli_get_auto_xfer_rdy_capable(&hal->sli)) {
		len = sli_cmd_config_auto_xfer_rdy_hp(&hal->sli, buf, SLI4_BMBX_SIZE,
						      hal->config.tow_io_size, ramdisc_blocksize);
	} else if (sli_get_tow_capable(&hal->sli)) {
		len = sli_cmd_config_optimized_write_hp(&hal->sli, buf, SLI4_BMBX_SIZE,
							hal->config.tow_io_size,
							hal->config.tow_xri_cnt,
							ramdisc_blocksize, tow_feature);
	}

	if (len <= 0) {
		ocs_log_err(hal->os, "Failed to populate TOW ESOC command\n");
		return rc;
	}

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		return OCS_HAL_RTN_ERROR;

	return rc;
}

static ocs_hal_rtn_e
ocs_hal_init_tow(ocs_hal_t *hal, uint32_t tow_feature)
{
	uint8_t buf[SLI4_BMBX_SIZE];
	int32_t len = 0;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;

	if (sli_get_auto_xfer_rdy_capable(&hal->sli)) {
		len = sli_cmd_config_auto_xfer_rdy(&hal->sli, buf, SLI4_BMBX_SIZE, hal->config.tow_io_size);
	} else if (sli_get_tow_capable(&hal->sli)) {
		len = sli_cmd_config_optimized_write(&hal->sli, buf, SLI4_BMBX_SIZE,
						     hal->config.tow_io_size,
						     hal->config.tow_xri_cnt,
						     tow_feature);
	}

	if (len <= 0) {
		ocs_log_err(hal->os, "Failed to populate TOW comamnd\n");
		return rc;
	}

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		return OCS_HAL_RTN_ERROR;

	return rc;
}

static ocs_hal_rtn_e
ocs_hal_init_tow_t10pi(ocs_hal_t *hal)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;

	if (sli_get_auto_xfer_rdy_capable(&hal->sli)) {
		rc = ocs_hal_config_axr_t10pi(hal);
		if (rc != OCS_HAL_RTN_SUCCESS)
			ocs_log_err(hal->os, "Failed to config AXR T10 PI\n");
	} else if (sli_get_tow_capable(&hal->sli)) {
		rc = ocs_hal_config_tow_t10pi(hal);
		if (rc != OCS_HAL_RTN_SUCCESS)
			ocs_log_err(hal->os, "Failed to config tow T10 PI\n");
	}

	return rc;
}

/**
 * @brief Initialize TOW feature for AXR/FB
 */
static ocs_hal_rtn_e
ocs_hal_init_tow_feature(ocs_hal_t *hal)
{
	uint32_t tow_feature = 0;
	int32_t rc = OCS_HAL_RTN_ERROR;

	hal->tow_enabled = FALSE;

	ocs_hal_get(hal, OCS_HAL_TOW_FEATURE, &tow_feature);
	if (!tow_feature)
		return OCS_HAL_RTN_SUCCESS;

	if (hal->config.esoc)
		rc = ocs_hal_init_tow_esoc(hal, tow_feature);
	else
		rc = ocs_hal_init_tow(hal, tow_feature);

	if (rc) {
		ocs_log_err(hal->os, "Failed to enable TOW feature\n");
		return rc;
	}

	if (hal->config.tow_t10_enable) {
		rc = ocs_hal_init_tow_t10pi(hal);
		if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_log_err(hal->os, "Failed to config optimized write T10 PI\n");
			return rc;
		}
	}

	hal->tow_enabled = TRUE;

	/*
	 * In TOW first burst, when port-owned XRI pool is empty,
	 * the first burst data frames will be redirected to RQ pair payload buffer.
	 * To support this, set the default RQ data buffer size as 2K.
	 */
	ocs_hal_set(hal, OCS_HAL_RQ_DEFAULT_BUFFER_SIZE, OCS_HAL_RQ_SIZE_PAYLOAD);

	ocs_log_info(hal->os, "TOW feature is successfully enabled\n");
	return rc;
}

static inline void
ocs_hal_add_io_timed_wqe(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	if (hal->config.emulate_tgt_wqe_timeout && io->tgt_wqe_timeout) {
		/*
		 * Active WQE list currently only used for
		 * target WQE timeouts.
		 */
		ocs_lock(&hal->io_timed_wqe_lock);
			if (OCS_HAL_ELS_REQ == io->type) {
				ocs_list_add_head(&hal->io_timed_wqe, io);
			} else {
				ocs_list_add_tail(&hal->io_timed_wqe, io);
			}
			io->submit_ticks = ocs_get_os_ticks();
		ocs_unlock(&hal->io_timed_wqe_lock);
	}
}

static inline void
ocs_hal_remove_io_timed_wqe(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	if (hal->config.emulate_tgt_wqe_timeout) {
		/*
		 * If target wqe timeouts are enabled,
		 * remove from active wqe list.
		 */
		ocs_lock(&hal->io_timed_wqe_lock);
			if (ocs_list_on_list(&io->wqe_link) && !ocs_list_empty(&hal->io_timed_wqe))
				ocs_list_remove(&hal->io_timed_wqe, io);
		ocs_unlock(&hal->io_timed_wqe_lock);
	}
}

static bool
ocs_hal_io_can_post_abts(int32_t status, uint32_t ext_status)
{
	if (SLI4_FC_WCQE_STATUS_LOCAL_REJECT == status) {
		switch (ext_status) {
			case SLI4_FC_LOCAL_REJECT_LINK_DOWN:
			case SLI4_FC_LOCAL_REJECT_RPI_SUSPENDED:
				/*
				 * WQE failed because link went down. Depending on the timing, FW will complete
				 * all pending WQEs with extended status as either RPI_SUSPENDED or LINK_DOWN.
				 */
				return FALSE;
			default:
				break;
		}
	}

	return TRUE;
}

/**
 * @brief Determine the number of chutes on the device.
 *
 * @par Description
 * Some devices require queue resources allocated per protocol processor
 * (chute). This function returns the number of chutes on this device.
 *
 * @param hal Hardware context allocated by the caller.
 *
 * @return Returns the number of chutes on the device for protocol.
 */
static uint32_t
ocs_hal_get_num_chutes(ocs_hal_t *hal)
{
	uint32_t num_chutes = 1;

	if (sli_get_is_dual_ulp_capable(&hal->sli) &&
	    sli_get_is_ulp_enabled(&hal->sli, 0) &&
	    sli_get_is_ulp_enabled(&hal->sli, 1)) {
		num_chutes = 2;
	}
	return num_chutes;
}

static ocs_hal_rtn_e
ocs_hal_link_event_init(ocs_hal_t *hal)
{
	if (hal == NULL) {
		ocs_log_err(NULL, "bad parameter hal=%p\n", hal);
		return OCS_HAL_RTN_ERROR;
	}

	hal->link.status = SLI_LINK_STATUS_MAX;
	hal->link.topology = SLI_LINK_TOPO_NONE;
	hal->link.medium = SLI_LINK_MEDIUM_MAX;
	hal->link.speed = 0;
	hal->link.logical_link_speed = 0;
	hal->link.loop_map = NULL;
	hal->link.fc_id = UINT32_MAX;

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup devInitShutdown
 * @brief Read the max dump size for this port
 *
 * @par Description
 * Queries the FW for the maximum dump size
 *
 * @param hal Hardware context allocated by the caller.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static ocs_hal_rtn_e
ocs_hal_read_max_dump_size(ocs_hal_t *hal)
{
	uint8_t	buf[SLI4_BMBX_SIZE];
	int	rc;

	/* lancer only */
	if ((SLI4_IF_TYPE_LANCER_FC_ETH != sli_get_if_type(&hal->sli)) &&
	    (SLI4_IF_TYPE_LANCER_G7 != sli_get_if_type(&hal->sli))) {
		ocs_log_debug(hal->os, "Function only supported for I/F type 2/6\n");
		return OCS_HAL_RTN_ERROR;
	}

	/*
	 * Make sure the FW is new enough to support this command. If the FW
	 * is too old, the FW will UE.
	 */
	if (hal->workaround.disable_dump_loc) {
		ocs_log_test(hal->os, "FW version is too old for this feature\n");
		return OCS_HAL_RTN_ERROR;
	}

	sli_cmd_common_set_dump_location(&hal->sli, buf, SLI4_BMBX_SIZE, 1, 0, NULL, 0);
	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (OCS_HAL_RTN_SUCCESS == rc) {
		sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)buf;
		sli4_res_common_set_dump_location_t *sli4_rsp =
			(sli4_res_common_set_dump_location_t *)mbox_rsp->payload.embed;

		if ((0 == mbox_rsp->hdr.status) && (0 == sli4_rsp->hdr.status)) {
			hal->dump_size = sli4_rsp->buffer_length;
			ocs_log_debug(hal->os, "Dump size: %x\n", sli4_rsp->buffer_length);
		} else {
			rc = OCS_HAL_RTN_ERROR;
		}
	}

	return rc;
}

static void
ocs_hal_sli_queue_free(ocs_hal_t *hal)
{
	uint32_t i;

	for (i = 0; i < OCS_HAL_MAX_NUM_WQ && hal->wq[i]; i++) {
		ocs_free(hal->os, hal->wq[i], sizeof(sli4_queue_t));
		hal->wq[i] = NULL;
	}

	for (i = 0; i < OCS_HAL_MAX_NUM_RQ && hal->rq[i]; i++) {
		ocs_free(hal->os, hal->rq[i], sizeof(sli4_queue_t));
		hal->rq[i] = NULL;
	}

	for (i = 0; i < OCS_HAL_MAX_NUM_MQ && hal->mq[i]; i++) {
		ocs_free(hal->os, hal->mq[i], sizeof(sli4_queue_t));
		hal->mq[i] = NULL;
	}

	for (i = 0; i < OCS_HAL_MAX_NUM_CQ && hal->cq[i]; i++) {
		ocs_free(hal->os, hal->cq[i], sizeof(sli4_queue_t));
		hal->cq[i] = NULL;
	}

	for (i = 0; i < OCS_HAL_MAX_NUM_EQ && hal->eq[i]; i++) {
		ocs_free(hal->os, hal->eq[i], sizeof(sli4_queue_t));
		hal->eq[i] = NULL;
	}
}

static ocs_hal_rtn_e
ocs_hal_sli_queue_alloc(ocs_hal_t *hal)
{
	uint32_t i;

	for (i = 0; i < OCS_HAL_MAX_NUM_WQ; i++) {
		hal->wq[i] = ocs_malloc(hal->os, sizeof(sli4_queue_t), OCS_M_ZERO);
		if (!hal->wq[i]) {
			ocs_log_err(hal->os, "Failed to allocate memory for SLI WQ[%d]\n", i);
			goto free_sli_queues;
		}
	}

	for (i = 0; i < OCS_HAL_MAX_NUM_RQ; i++) {
		hal->rq[i] = ocs_malloc(hal->os, sizeof(sli4_queue_t), OCS_M_ZERO);
		if (!hal->rq[i]) {
			ocs_log_err(hal->os, "Failed to allocate memory for SLI RQ[%d]\n", i);
			goto free_sli_queues;
		}
	}

	for (i = 0; i < OCS_HAL_MAX_NUM_MQ; i++) {
		hal->mq[i] = ocs_malloc(hal->os, sizeof(sli4_queue_t), OCS_M_ZERO);
		if (!hal->mq[i]) {
			ocs_log_err(hal->os, "Failed to allocate memory for SLI MQ[%d]\n", i);
			goto free_sli_queues;
		}
	}

	for (i = 0; i < OCS_HAL_MAX_NUM_CQ; i++) {
		hal->cq[i] = ocs_malloc(hal->os, sizeof(sli4_queue_t), OCS_M_ZERO);
		if (!hal->cq[i]) {
			ocs_log_err(hal->os, "Failed to allocate memory for SLI CQ[%d]\n", i);
			goto free_sli_queues;
		}
	}

	for (i = 0; i < OCS_HAL_MAX_NUM_EQ; i++) {
		hal->eq[i] = ocs_malloc(hal->os, sizeof(sli4_queue_t), OCS_M_ZERO);
		if (!hal->eq[i]) {
			ocs_log_err(hal->os, "Failed to allocate memory for SLI EQ[%d]\n", i);
			goto free_sli_queues;
		}
	}

	return OCS_HAL_RTN_SUCCESS;

free_sli_queues:
	ocs_hal_sli_queue_free(hal);
	return OCS_HAL_RTN_NO_MEMORY;
}

static void
ocs_hal_queue_hash_free(ocs_hal_t *hal)
{
	if (hal->cq_hash) {
		ocs_free(hal->os, hal->cq_hash, OCS_HAL_Q_HASH_SIZE * sizeof(ocs_queue_hash_t));
		hal->cq_hash = NULL;
	}

	if (hal->rq_hash) {
		ocs_free(hal->os, hal->rq_hash, OCS_HAL_Q_HASH_SIZE * sizeof(ocs_queue_hash_t));
		hal->rq_hash = NULL;
	}

	if (hal->wq_hash) {
		ocs_free(hal->os, hal->wq_hash, OCS_HAL_Q_HASH_SIZE * sizeof(ocs_queue_hash_t));
		hal->wq_hash = NULL;
	}
}

static ocs_hal_rtn_e
ocs_hal_queue_hash_alloc(ocs_hal_t *hal)
{
	hal->cq_hash = ocs_malloc(hal->os, OCS_HAL_Q_HASH_SIZE * sizeof(ocs_queue_hash_t), OCS_M_ZERO);
	if (!hal->cq_hash) {
		ocs_log_err(hal->os, "CQ hash allocation failed\n");
		goto free_queue_hash;
	}

	hal->rq_hash = ocs_malloc(hal->os, OCS_HAL_Q_HASH_SIZE * sizeof(ocs_queue_hash_t), OCS_M_ZERO);
	if (!hal->rq_hash) {
		ocs_log_err(hal->os, "RQ hash allocation failed\n");
		goto free_queue_hash;
	}

	hal->wq_hash = ocs_malloc(hal->os, OCS_HAL_Q_HASH_SIZE * sizeof(ocs_queue_hash_t), OCS_M_ZERO);
	if (!hal->wq_hash) {
		ocs_log_err(hal->os, "WQ hash allocation failed\n");
		goto free_queue_hash;
	}

	return OCS_HAL_RTN_SUCCESS;

free_queue_hash:
	ocs_hal_queue_hash_free(hal);
	ocs_hal_sli_queue_free(hal);
	return OCS_HAL_RTN_NO_MEMORY;
}

/**
 * @ingroup devInitShutdown
 * @brief Set up the Hardware Abstraction Layer module.
 *
 * @par Description
 * Calls set up to configure the hardware.
 *
 * @param hal Hardware context allocated by the caller.
 * @param os Device abstraction.
 * @param port_type Protocol type of port, such as FC and NIC.
 *
 * @todo Why is port_type a parameter?
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_setup(ocs_hal_t *hal, ocs_os_handle_t os, sli4_port_type_e port_type)
{
	uint32_t i;

	if (hal == NULL) {
		ocs_log_err(os, "bad parameter(s) hal=%p\n", hal);
		return OCS_HAL_RTN_ERROR;
	}

	if (hal->hal_setup_called) {
		/* Setup run-time workarounds.
		 * Call for each setup, to allow for hal_war_version
		 */
		ocs_hal_workaround_setup(hal);
		return OCS_HAL_RTN_SUCCESS;
	}

	/*
	 * ocs_hal_init() relies on NULL pointers indicating that a structure
	 * needs allocation. If a structure is non-NULL, ocs_hal_init() won't
	 * free/realloc that memory
	 */
	ocs_memset(hal, 0, sizeof(ocs_hal_t));

	hal->hal_setup_called = TRUE;

	hal->os = os;

	ocs_sem_init(&hal->bmbx_sem, 1, "hal_bmbx_sem");

	ocs_lock_init(hal->os, &hal->io_lock, "HAL_io_lock[%d]", ocs_instance(hal->os));
	ocs_lock_init(hal->os, &hal->io_timed_wqe_lock, "HAL_io_timed_wqe_lock[%d]", ocs_instance(hal->os));
	ocs_lock_init(hal->os, &hal->cmd_lock, "HAL_cmd_lock[%d]", ocs_instance(hal->os));
	ocs_list_init(&hal->cmd_head, ocs_command_ctx_t, link);
	ocs_list_init(&hal->cmd_pending, ocs_command_ctx_t, link);
	hal->cmd_head_count = 0;

	ocs_atomic_init(&hal->io_alloc_failed_count, 0);

	hal->config.speed = FC_LINK_SPEED_AUTO_32_16_8;
	hal->config.dif_seed = 0;
	hal->config.tow_blksize_chip = OCS_HAL_TOW_BLK_SIZE_DEFAULT;
	hal->config.tow_ref_tag_lba = OCS_HAL_TOW_REF_TAG_LBA_DEFAULT;
	hal->config.tow_app_tag_valid = OCS_HAL_TOW_APP_TAG_VALID_DEFAULT;
	hal->config.tow_app_tag_value = OCS_HAL_TOW_APP_TAG_VALUE_DEFAULT;

	/* Allocate storage for SLI queue objects */
	if (ocs_hal_sli_queue_alloc(hal)) {
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* Allocate memory for queue hashes */
	if (ocs_hal_queue_hash_alloc(hal)) {
		return OCS_HAL_RTN_NO_MEMORY;
	}

	if (sli_setup(&hal->sli, hal->os, port_type)) {
		ocs_log_err(hal->os, "SLI setup failed\n");
		return OCS_HAL_RTN_ERROR;
	}

	ocs_memset(hal->domains, 0, sizeof(hal->domains));

	ocs_memset(hal->fcf_index_fcfi, 0, sizeof(hal->fcf_index_fcfi));

	ocs_hal_link_event_init(hal);

	sli_callback(&hal->sli, SLI4_CB_LINK, ocs_hal_cb_link, hal);
	sli_callback(&hal->sli, SLI4_CB_FIP, ocs_hal_cb_fip, hal);
	sli_callback(&hal->sli, SLI4_CB_ERR, ocs_hal_reset_pending, hal);

	/*
	 * Set all the queue sizes to the maximum allowed. These values may
	 * be changes later by the adjust and workaround functions.
	 */
	for (i = 0; i < ARRAY_SIZE(hal->num_qentries); i++) {
		hal->num_qentries[i] = sli_get_max_qentries(&hal->sli, i);
	}

	/*
	 * The RQ assignment for RQ pair mode.
	 */
	hal->config.rq_default_buffer_size = OCS_HAL_RQ_SIZE_PAYLOAD;
	hal->config.n_io = sli_get_max_rsrc(&hal->sli, SLI_RSRC_FCOE_XRI);

	/* by default, enable initiator-only auto-ABTS emulation */
	hal->config.i_only_aab = TRUE;

	/* Setup run-time workarounds */
	ocs_hal_workaround_setup(hal);

	/* HAL_WORKAROUND_OVERRIDE_FCFI_IN_SRB */
	if (hal->workaround.override_fcfi) {
		hal->first_domain_idx = -1;
	}

	/* Must be done after the workaround setup */
	if ((SLI4_IF_TYPE_LANCER_FC_ETH == sli_get_if_type(&hal->sli)) ||
	    (SLI4_IF_TYPE_LANCER_G7 == sli_get_if_type(&hal->sli))) {
		(void)ocs_hal_read_max_dump_size(hal);
	}

	/* calculate the number of WQs required. */
	ocs_hal_adjust_wqs(hal);

	/* Set the default dif mode */
	if (!sli_is_dif_inline_capable(&hal->sli)) {
		ocs_log_test(hal->os, "not inline capable, setting DIF mode to seperate\n");
		hal->config.dif_mode = OCS_HAL_DIF_MODE_SEPARATE;
	}

	/* Workaround: BZ 161832 */
	if (hal->workaround.use_dif_sec_xri) {
		ocs_list_init(&hal->sec_hio_wait_list, ocs_hal_io_t, link);
	}

	/*
	 * Figure out the starting and max ULP to spread the WQs across the
	 * ULPs.
	 */
	if (sli_get_is_dual_ulp_capable(&hal->sli)) {
		if (sli_get_is_ulp_enabled(&hal->sli, 0) &&
		    sli_get_is_ulp_enabled(&hal->sli, 1)) {
			hal->ulp_start = 0;
			hal->ulp_max   = 1;
		} else if (sli_get_is_ulp_enabled(&hal->sli, 0)) {
			hal->ulp_start = 0;
			hal->ulp_max   = 0;
		} else {
			hal->ulp_start = 1;
			hal->ulp_max   = 1;
		}
	} else {
		if (sli_get_is_ulp_enabled(&hal->sli, 0)) {
			hal->ulp_start = 0;
			hal->ulp_max   = 0;
		} else {
			hal->ulp_start = 1;
			hal->ulp_max   = 1;
		}
	}
	ocs_log_debug(hal->os, "ulp_start %d, ulp_max %d\n",
		hal->ulp_start, hal->ulp_max);
	hal->config.queue_topology = hal_global.queue_topology_string;

	hal->qtop = ocs_hal_qtop_parse(hal, hal->config.queue_topology);
	if (hal->qtop == NULL) {
		ocs_log_crit(hal->os, "Queue topology string is invalid\n");
		return OCS_HAL_RTN_ERROR;
	}

	hal->config.n_eq = hal->qtop->entry_counts[QTOP_EQ];
	/* One addl. CQ is reserved for ELS IO */
	hal->config.n_cq = hal->qtop->entry_counts[QTOP_CQ] + 1;
	hal->config.n_rq = hal->qtop->entry_counts[QTOP_RQ];
	/* One addl. WQ is reserved for ELS IO */
	hal->config.n_wq = hal->qtop->entry_counts[QTOP_WQ] + 1;
	hal->config.n_mq = hal->qtop->entry_counts[QTOP_MQ];

	/* Verify qtop configuration against driver supported configuration */
#if 0
	/* TODO - Verify how to sanity check num rqs */
	if (hal->config.n_rq > OCE_HAL_MAX_NUM_MRQ_PAIRS) {
		ocs_log_crit(hal->os, "Max supported MRQ pairs = %d\n",
			     OCE_HAL_MAX_NUM_MRQ_PAIRS);
		return OCS_HAL_RTN_ERROR;
	}
#endif

	if (hal->config.n_eq > OCS_HAL_MAX_NUM_EQ) {
		ocs_log_crit(hal->os, "Max supported EQs = %d\n",
			     OCS_HAL_MAX_NUM_EQ);
		return OCS_HAL_RTN_ERROR;
	}
	
	if (hal->config.n_cq > OCS_HAL_MAX_NUM_CQ) {
		ocs_log_crit(hal->os, "Max supported CQs = %d\n",
			     OCS_HAL_MAX_NUM_CQ);
		return OCS_HAL_RTN_ERROR;
	}

	if (hal->config.n_wq > OCS_HAL_MAX_NUM_WQ) {
		ocs_log_crit(hal->os, "Max supported WQs = %d\n",
			     OCS_HAL_MAX_NUM_WQ);
		return OCS_HAL_RTN_ERROR;
	}

	if (hal->config.n_mq > OCS_HAL_MAX_NUM_MQ) {
		ocs_log_crit(hal->os, "Max supported MQs = %d\n",
			     OCS_HAL_MAX_NUM_MQ);
		return OCS_HAL_RTN_ERROR;
	}

	return OCS_HAL_RTN_SUCCESS;
}

static ocs_hal_rtn_e
ocs_hal_init_sli(ocs_hal_t *hal)
{
	sli4_features_t features;

	features.dword = sli_config_get_features(&hal->sli);

	/* If MRQ is not required, make sure we dont request the feature */
	if (hal->config.n_rq == 1)
		features.flag.mrqp = false;

	features.flag.hlm = sli_get_hlm(&hal->sli);
	features.flag.rxseq = false;
	features.flag.rxri  = false;

	if (sli_get_if_type(&hal->sli) == SLI4_IF_TYPE_LANCER_G7) {
		uint32_t tow_feature;

		/*
		 * Prism A0 limitation:
		 * Enable either TOW or VMID, but not both.
		 */
		ocs_hal_get(hal, OCS_HAL_TOW_FEATURE, &tow_feature);
		if (tow_feature) {
			features.flag.ashdr = false;
			ocs_log_info(hal->os, "Disable SLI ASHDR feature\n");
		}
	}

	sli_config_set_features(&hal->sli, features.dword);

	return sli_init(&hal->sli);
}

/**
 * @ingroup devInitShutdown
 * @brief Allocate memory structures to prepare for the device operation.
 *
 * @par Description
 * Allocates memory structures needed by the device and prepares the device
 * for operation.
 * @n @n @b Note: This function may be called more than once (for example, at
 * initialization and then after a reset), but the size of the internal resources
 * may not be changed without tearing down the HAL (ocs_hal_teardown()).
 *
 * @param hal Hardware context allocated by the caller.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_init(ocs_hal_t *hal)
{
	ocs_hal_rtn_e	rc;
	uint32_t	i = 0;
	uint8_t		buf[SLI4_BMBX_SIZE];
	uint32_t	max_rpi;
	int		rem_count;
	uint32_t	q_count = 0;

	/*
	 * Make sure the command lists are empty. If this is start-of-day,
	 * they'll be empty since they were just initialized in ocs_hal_setup.
	 * If we've just gone through a reset, the command and command pending
	 * lists should have been cleaned up as part of the reset (ocs_hal_reset()).
	 */
	ocs_lock(&hal->cmd_lock);
		if (!ocs_list_empty(&hal->cmd_head)) {
			ocs_log_test(hal->os, "command found on cmd list\n");
			ocs_unlock(&hal->cmd_lock);
			return OCS_HAL_RTN_ERROR;
		}
		if (!ocs_list_empty(&hal->cmd_pending)) {
			ocs_log_test(hal->os, "command found on pending list\n");
			ocs_unlock(&hal->cmd_lock);
			return OCS_HAL_RTN_ERROR;
		}
	ocs_unlock(&hal->cmd_lock);

	/* Free RQ buffers if prevously allocated */
	ocs_hal_rx_free(hal);

	/*
	 * The IO queues must be initialized here for the reset case. The
	 * ocs_hal_init_io() function will re-add the IOs to the free list.
	 * The cmd_head list should be OK since we free all entries in
	 * ocs_hal_command_cancel() that is called in the ocs_hal_reset().
	 */

	/* If we are in this function due to a reset, there may be stale items
	 * on lists that need to be removed.  Clean them up.
	 */
	rem_count=0;
	if (ocs_list_valid(&hal->io_wait_free)) {
		while ((!ocs_list_empty(&hal->io_wait_free))) {
			rem_count++;
			ocs_list_remove_head(&hal->io_wait_free);
		}
		if (rem_count > 0)
			ocs_log_debug(hal->os, "removed %d items from io_wait_free list\n", rem_count);
	}
	rem_count=0;
	if (ocs_list_valid(&hal->io_inuse)) {
		while ((!ocs_list_empty(&hal->io_inuse))) {
			rem_count++;
			ocs_list_remove_head(&hal->io_inuse);
		}
		if (rem_count > 0)
			ocs_log_debug(hal->os, "removed %d items from io_inuse list\n", rem_count);
	}
	rem_count=0;
	if (ocs_list_valid(&hal->io_free)) {
		while ((!ocs_list_empty(&hal->io_free))) {
			rem_count++;
			ocs_list_remove_head(&hal->io_free);
		}
		if (rem_count > 0)
			ocs_log_debug(hal->os, "removed %d items from io_free list\n", rem_count);
	}
	if (ocs_list_valid(&hal->io_port_owned)) {
		while ((!ocs_list_empty(&hal->io_port_owned))) {
			ocs_list_remove_head(&hal->io_port_owned);
		}
	}
	ocs_list_init(&hal->io_inuse, ocs_hal_io_t, link);
	ocs_list_init(&hal->io_free, ocs_hal_io_t, link);
	ocs_list_init(&hal->io_port_owned, ocs_hal_io_t, link);
	ocs_list_init(&hal->io_wait_free, ocs_hal_io_t, link);
	ocs_list_init(&hal->io_timed_wqe, ocs_hal_io_t, wqe_link);
	ocs_list_init(&hal->io_port_dnrx, ocs_hal_io_t, dnrx_link);

	rc = ocs_hal_init_sli(hal);
	if (rc) {
		ocs_log_err(hal->os, "Failed to initialize SLI\n");
		return OCS_HAL_RTN_ERROR;
	}

	/* FW host logging feature is only supported for Lancer G6 or Prism */
	if (hal->sli.asic_type == SLI4_ASIC_TYPE_LANCERG6 ||
	    hal->sli.asic_type == SLI4_ASIC_TYPE_LANCERG7) {
		if (ocs_hal_config_fw_diagnostic_logging(hal, hal_global.fw_diag_log_size,
							 hal_global.fw_diag_log_level)) {
			ocs_log_err(hal->os, "Failed to enable RAS firmware logging\n");
		}
	}

	/*
	 * Configure dual dump feature based on user selection.
	 */
	rc = ocs_hal_config_set_dual_dump(hal);
	if (rc && rc != OCS_HAL_RTN_INVALID_ARG && rc != OCS_HAL_RTN_NOT_SUPPORTED) {
		ocs_log_err(hal->os, "Failed to configure dual dump feature: %d\n", rc);
	}

	/*
	 * Enable the target optimized write feature if requested.
	 */
	rc = ocs_hal_init_tow_feature(hal);
	if (rc) {
		ocs_log_err(hal->os, "Failed to initialize optimized write feature: %d\n", rc);
		return rc;
	}

	if(hal->sliport_healthcheck) {
		rc = ocs_hal_config_sli_port_health_check(hal, 0, 1);
		if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_log_err(hal->os, "Enabling Sliport Health check failed: %d\n", rc);
			return rc;
		}
	}

	if ((sli_get_if_type(&hal->sli) == SLI4_IF_TYPE_LANCER_FC_ETH) ||
	    (hal->sli.if_type == SLI4_IF_TYPE_LANCER_G7)) {
		rc = ocs_hal_config_fec(hal, !hal->disable_fec);
		if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_log_err(hal->os, "Config FEC failed (rc: %d)\n", rc);
			return rc;
		}
	}

	if (sli_get_if_type(&hal->sli) == SLI4_IF_TYPE_LANCER_FC_ETH) {
		rc = ocs_hal_config_fw_ag_rsp(hal, hal->enable_fw_ag_rsp);
		if (rc != OCS_HAL_RTN_SUCCESS &&  rc != OCS_HAL_RTN_NOT_SUPPORTED) {
			ocs_log_err(hal->os, "Setting FW Auto-good response config failed: %d\n", rc);
		}
	}

	if (hal->sliport_pause_errors && (ocs_strcmp(hal->sliport_pause_errors, "disabled")) &&
			(SLI4_IF_TYPE_LANCER_FC_ETH == sli_get_if_type(&hal->sli))) {
		rc = ocs_hal_config_sliport_pause(hal, 1);
		if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_log_err(hal->os, "Sliport pause state enable failed (rc: %d)\n", rc);
			return rc;
		}
	}

	/*
	 * Set FDT transfer hint, only works on Lancer
	 */
	if ((hal->sli.if_type == SLI4_IF_TYPE_LANCER_FC_ETH) && (OCS_HAL_FDT_XFER_HINT != 0)) {
		/*
		 * Non-fatal error. In particular, we can disregard failure to set OCS_HAL_FDT_XFER_HINT on
		 * devices with legacy firmware that do not support OCS_HAL_FDT_XFER_HINT feature.
		 */
		ocs_hal_config_set_fdt_xfer_hint(hal, OCS_HAL_FDT_XFER_HINT);
	}

#if !defined(OCSU_FC_WORKLOAD)
	/*
	 * Set dump to host buffers every time when hal is reset
	 */
	rc = ocs_hal_set_dump_to_host_buffers(hal);
	if (rc) {
		ocs_log_err(hal->os, "Set FW dump location failed (rc: %d)\n", rc);
		return rc;
	}
#endif

	/*
	 * Verify that we have not exceeded any queue sizes
	 */
	q_count = MIN(sli_get_max_queue(&hal->sli, SLI_QTYPE_EQ),
					OCS_HAL_MAX_NUM_EQ);
	if (hal->config.n_eq > q_count) {
		ocs_log_err(hal->os, "requested %d EQ but %d allowed\n",
			    hal->config.n_eq, q_count);
		return OCS_HAL_RTN_ERROR;
	}

	q_count = MIN(sli_get_max_queue(&hal->sli, SLI_QTYPE_CQ),
					OCS_HAL_MAX_NUM_CQ);
	if (hal->config.n_cq > q_count) {
		ocs_log_err(hal->os, "requested %d CQ but %d allowed\n",
			    hal->config.n_cq, q_count);
		return OCS_HAL_RTN_ERROR;
	}

	q_count = MIN(sli_get_max_queue(&hal->sli, SLI_QTYPE_MQ),
					OCS_HAL_MAX_NUM_MQ);
	if (hal->config.n_mq > q_count) {
		ocs_log_err(hal->os, "requested %d MQ but %d allowed\n",
			    hal->config.n_mq, q_count);
		return OCS_HAL_RTN_ERROR;
	}

	q_count = MIN(sli_get_max_queue(&hal->sli, SLI_QTYPE_RQ),
					OCS_HAL_MAX_NUM_RQ);
	if (hal->config.n_rq > q_count) {
		ocs_log_err(hal->os, "requested %d RQ but %d allowed\n",
			    hal->config.n_rq, q_count);
		return OCS_HAL_RTN_ERROR;
	}

	q_count = MIN(sli_get_max_queue(&hal->sli, SLI_QTYPE_WQ),
					OCS_HAL_MAX_NUM_WQ);
	if (hal->config.n_wq > q_count) {
		ocs_log_err(hal->os, "requested %d WQ but %d allowed\n",
			    hal->config.n_wq, q_count);
		return OCS_HAL_RTN_ERROR;
	}

	/* zero the hashes */
	ocs_memset(hal->cq_hash, 0, OCS_HAL_Q_HASH_SIZE * sizeof(ocs_queue_hash_t));
	ocs_log_debug(hal->os, "Max CQs %d, hash size = %d\n", OCS_HAL_MAX_NUM_CQ, OCS_HAL_Q_HASH_SIZE);

	ocs_memset(hal->rq_hash, 0, OCS_HAL_Q_HASH_SIZE * sizeof(ocs_queue_hash_t));
	ocs_log_debug(hal->os, "Max RQs %d, hash size = %d\n", OCS_HAL_MAX_NUM_RQ, OCS_HAL_Q_HASH_SIZE);

	ocs_memset(hal->wq_hash, 0, OCS_HAL_Q_HASH_SIZE * sizeof(ocs_queue_hash_t));
	ocs_log_debug(hal->os, "Max WQs %d, hash size = %d\n", OCS_HAL_MAX_NUM_WQ, OCS_HAL_Q_HASH_SIZE);

	rc = ocs_hal_init_queues(hal, hal->qtop);
	if (rc != OCS_HAL_RTN_SUCCESS) {
		return rc;
	}

	max_rpi = sli_get_max_rsrc(&hal->sli, SLI_RSRC_FCOE_RPI);
	i = sli_fc_get_rpi_requirements(&hal->sli, max_rpi);
	if (i) {
		ocs_dma_t payload_memory;

		rc = OCS_HAL_RTN_ERROR;

		if (hal->rnode_mem.size) {
			ocs_dma_free(hal->os, &hal->rnode_mem);
		}

		if (ocs_dma_alloc(hal->os, &hal->rnode_mem, i, 4096)) {
			ocs_log_err(hal->os, "remote node memory allocation failed\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}

		payload_memory.size = 0;
		if (sli_cmd_fcoe_post_hdr_templates(&hal->sli, buf, SLI4_BMBX_SIZE,
					&hal->rnode_mem, UINT16_MAX, &payload_memory)) {
			rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);

			if (payload_memory.size) {
				/* The command was non-embedded - need to free the dma buffer */
				ocs_dma_free(hal->os, &payload_memory);
			}
		}

		if (rc || ((sli4_mbox_command_header_t *)buf)->status) {
			ocs_log_err(hal->os, "header template registration failed\n");
			return OCS_HAL_RTN_ERROR;
		}
	}

	/* Allocate and post RQ buffers */
	rc = ocs_hal_rx_allocate(hal);
	if (rc) {
		ocs_log_err(hal->os, "rx_allocate failed (rc: %d)\n", rc);
		return rc;
	}

	/* Populate hal->seq_free_list */
	if (hal->seq_pool == NULL) {
		uint32_t count = 0;
		uint32_t i;

		/* Sum up the total number of RQ entries, to use to allocate the sequence object pool */
		for (i = 0; i < hal->hal_rq_count; i++) {
			count += hal->hal_rq[i]->entry_count;
		}

		hal->seq_pool = ocs_array_alloc(hal->os, sizeof(ocs_hal_sequence_t), count);
		if (hal->seq_pool == NULL) {
			ocs_log_err(hal->os, "malloc seq_pool failed\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}
	}

	if (ocs_hal_rx_post(hal))
		ocs_log_err(hal->os, "WARNING - error posting RQ buffers\n");

	/* Allocate rpi_ref if not previously allocated */
	if (hal->rpi_ref == NULL) {
		hal->rpi_ref = ocs_malloc(hal->os, max_rpi * sizeof(*hal->rpi_ref), OCS_M_ZERO);
		if (hal->rpi_ref == NULL) {
			ocs_log_err(hal->os, "rpi_ref allocation failure (%d)\n", i);
			return OCS_HAL_RTN_NO_MEMORY;
		}
	}

	for (i = 0; i < max_rpi; i ++) {
		ocs_atomic_init(&hal->rpi_ref[i].rpi_count, 0);
		ocs_atomic_init(&hal->rpi_ref[i].rpi_attached, 0);
	}

	ocs_memset(hal->domains, 0, sizeof(hal->domains));

	/* HAL_WORKAROUND_OVERRIDE_FCFI_IN_SRB */
	if (hal->workaround.override_fcfi) {
		hal->first_domain_idx = -1;
	}

	ocs_memset(hal->fcf_index_fcfi, 0, sizeof(hal->fcf_index_fcfi));

	/* Register a FCFI to allow unsolicited frames to be routed to the driver */
	if (sli_get_medium(&hal->sli) == SLI_LINK_MEDIUM_FC) {

		if (hal->hal_mrq_used) {
			ocs_log_info(hal->os, "using REG_FCFI MRQ\n");

			rc = ocs_hal_config_mrq(hal, SLI4_CMD_REG_FCFI_SET_FCFI_MODE, 0, 0);
			if (rc != OCS_HAL_RTN_SUCCESS) {
				ocs_log_err(hal->os, "REG_FCFI_MRQ FCFI registration failed: %d\n", rc);
				return rc;
			}

			rc = ocs_hal_config_mrq(hal, SLI4_CMD_REG_FCFI_SET_MRQ_MODE, 0, 0);
			if (rc != OCS_HAL_RTN_SUCCESS) {
				ocs_log_err(hal->os, "REG_FCFI_MRQ MRQ registration failed: %d\n", rc);
				return rc;
			}
		} else {
			sli4_cmd_rq_cfg_t rq_cfg[SLI4_CMD_REG_FCFI_NUM_RQ_CFG];

			ocs_log_info(hal->os, "using REG_FCFI standard\n");

			/* Set the filter match/mask values from hal's filter_def values */
			for (i = 0; i < SLI4_CMD_REG_FCFI_NUM_RQ_CFG; i++) {
				rq_cfg[i].rq_id = 0xffff;
				rq_cfg[i].r_ctl_mask =	(uint8_t)  hal->config.filter_def[i];
				rq_cfg[i].r_ctl_match = (uint8_t) (hal->config.filter_def[i] >> 8);
				rq_cfg[i].type_mask =	(uint8_t) (hal->config.filter_def[i] >> 16);
				rq_cfg[i].type_match =	(uint8_t) (hal->config.filter_def[i] >> 24);
			}

			/*
			 * Update the rq_id's of the FCF configuration (don't update more than the number
			 * of rq_cfg elements)
			 */
			for (i = 0; i < OCS_MIN(hal->hal_rq_count, SLI4_CMD_REG_FCFI_NUM_RQ_CFG); i++) {
				hal_rq_t *rq = hal->hal_rq[i];
				uint32_t j;
				for (j = 0; j < SLI4_CMD_REG_FCFI_NUM_RQ_CFG; j++) {
					uint32_t mask = (rq->filter_mask != 0) ? rq->filter_mask : 1;
					if (mask & (1U << j)) {
						rq_cfg[j].rq_id = rq->hdr->id;
						ocs_log_info(hal->os, "REG_FCFI: filter[%d] %08X -> RQ[%d] id=%d\n",
							j, hal->config.filter_def[j], i, rq->hdr->id);
					}
				}
			}

			sli_cmd_reg_fcfi(&hal->sli, buf, SLI4_BMBX_SIZE, 0, rq_cfg, 0);
			rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
			if (rc || ((sli4_mbox_command_header_t *)buf)->status)
				return OCS_HAL_RTN_ERROR;

			hal->fcf_indicator = ((sli4_cmd_reg_fcfi_t *)buf)->fcfi;
		}
	}

	/*
	 * Allocate the WQ request tag pool, if not previously allocated (the request tag value is 16 bits,
	 * thus the pool allocation size of 64k)
	 */
	rc = ocs_hal_reqtag_init(hal);
	if (rc) {
		ocs_log_err(hal->os, "ocs_pool_alloc hal_wq_callback_t failed: %d\n", rc);
		return rc;
	}

	rc = ocs_hal_setup_io(hal);
	if (rc) {
		ocs_log_err(hal->os, "IO allocation failure (rc: %d)\n", rc);
		return rc;
	}

	rc = ocs_hal_init_io(hal);
	if (rc) {
		ocs_log_err(hal->os, "IO initialization failure (rc: %d)\n", rc);
		return rc;
	}

	ocs_queue_history_init(hal->os, &hal->q_hist);

	/* get hw link config; polling, so callback will be called immediately */
	hal->linkcfg = OCS_HAL_LINKCFG_NA;
	ocs_hal_get_linkcfg(hal, OCS_CMD_POLL, ocs_hal_init_linkcfg_cb, hal);

	/* if lancer ethernet, ethernet ports need to be enabled */
	if ((hal->sli.if_type == SLI4_IF_TYPE_LANCER_FC_ETH) &&
	    (sli_get_medium(&hal->sli) == SLI_LINK_MEDIUM_ETHERNET)) {
		if (ocs_hal_set_eth_license(hal, hal->eth_license)) {
			/* log warning but continue */
			ocs_log_err(hal->os, "Failed to set ethernet license\n");
		}
	}

	/* Set the DIF seed - only for lancer right now */
	if (SLI4_IF_TYPE_LANCER_FC_ETH == sli_get_if_type(&hal->sli) &&
	    ocs_hal_set_dif_seed(hal) != OCS_HAL_RTN_SUCCESS) {
		ocs_log_err(hal->os, "Failed to set DIF seed value (rc: %d)\n", rc);
		return rc;
	}

	/* Set the DIF mode - skyhawk only */
	if (SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(&hal->sli) &&
	    sli_get_dif_capable(&hal->sli)) {
		rc = ocs_hal_set_dif_mode(hal);
		if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_log_err(hal->os, "Failed to set DIF mode value (rc: %d)\n", rc);
			return rc;
		}
	}

	/*
	 * Allocate a HAL IO for Send frame WQ for SCSI dev
	 */
	ocs_assert(hal->hal_sfwq[OCS_HAL_WQ_SFQ_SCSI], OCS_HAL_RTN_ERROR);
	hal->hal_sfwq[OCS_HAL_WQ_SFQ_SCSI]->send_frame_io = ocs_hal_io_alloc(hal);
	if (!hal->hal_sfwq[OCS_HAL_WQ_SFQ_SCSI]->send_frame_io) {
		ocs_log_err(hal->os, "Failed to allocate XRI for Send frame WQ\n");
		return OCS_HAL_RTN_ERROR;
	}

	/*
	 * Arming the EQ allows (e.g.) interrupts when CQ completions write EQ entries
	 */
	if (!hal->config.poll_mode) {
		for (i = 0; i < hal->eq_count; i++) {
			sli_queue_arm(&hal->sli, hal->eq[i], TRUE);
		}
	}

	/*
	 * Initialize RQ hash
	 */
	for (i = 0; i < hal->rq_count; i++) {
		ocs_hal_queue_hash_add(hal->rq_hash, hal->rq[i]->id, i);
	}

	/*
	 * Initialize WQ hash
	 */
	for (i = 0; i < hal->wq_count; i++) {
		ocs_hal_queue_hash_add(hal->wq_hash, hal->wq[i]->id, i);
	}

	/*
	 * Arming the CQ allows (e.g.) MQ completions to write CQ entries
	 */
	for (i = 0; i < hal->cq_count; i++) {
		ocs_hal_queue_hash_add(hal->cq_hash, hal->cq[i]->id, i);
		sli_queue_arm(&hal->sli, hal->cq[i], TRUE);
	}

	/* record the fact that the queues are functional */
	hal->state = OCS_HAL_STATE_ACTIVE;

	/* Note: Must be after the IOs are setup and the state is active */
	if (ocs_hal_rqpair_init(hal))
		ocs_log_err(hal->os, "WARNING - error initializing RQ pair\n");

	/* finally kick off periodic timer to check for timed out target WQEs */
	if (hal->config.emulate_tgt_wqe_timeout) {
		hal->active_wqe_timer_shutdown = FALSE;
		ocs_log_info(hal->os, "start wqe timer\n");
		ocs_setup_timer(hal->os, &hal->wqe_timer, target_wqe_timer_cb, hal,
				OCS_HAL_WQ_TIMER_PERIOD_MS, false);
	}

	/* Initialize send frame frame sequence id */
	ocs_atomic_init(&hal->send_frame_seq_id, 0);

	/* Initilize inbound pending frames counter*/
	ocs_atomic_init(&hal->pend_frames_count, 0);
	if (hal->watchdog_timeout) {
		hal->watchdog_timer_disable = false;
		ocs_setup_timer(hal->os, &hal->watchdog_timer, ocs_watchdog_timer,
				hal, OCS_HAL_WATCHDOG_TIMER_DEFER_MSEC, false);
	}

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @brief Configure Multi-RQ
 *
 * @param hal       Hardware context allocated by the caller.
 * @param mode      1 to set MRQ filters and 0 to set FCFI index
 * @param vlanid    valid in mode 0
 * @param fcf_index valid in mode 0
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
ocs_hal_config_mrq(ocs_hal_t *hal, uint8_t mode, uint16_t vlanid, uint16_t fcf_index)
{
	uint8_t buf[SLI4_BMBX_SIZE];
	hal_rq_t *rq;
	sli4_cmd_reg_fcfi_mrq_t *rsp = NULL;
	uint32_t i, j;
	sli4_cmd_rq_cfg_t rq_filter[SLI4_CMD_REG_FCFI_NUM_RQ_CFG];
	int32_t rq_filter_cnt = 0;
	int32_t rc;

	if (mode == SLI4_CMD_REG_FCFI_SET_FCFI_MODE) {
		goto issue_cmd;
	}

	/* Set the filter match/mask values from hal's filter_def values */
	for (i = 0; i < SLI4_CMD_REG_FCFI_NUM_RQ_CFG; i++) {
		rq_filter[i].rq_id = 0xffff;
		rq_filter[i].is_mrq = FALSE;
		rq_filter[i].r_ctl_mask  = (uint8_t)  hal->config.filter_def[i];
		rq_filter[i].r_ctl_match = (uint8_t) (hal->config.filter_def[i] >> 8);
		rq_filter[i].type_mask   = (uint8_t) (hal->config.filter_def[i] >> 16);
		rq_filter[i].type_match  = (uint8_t) (hal->config.filter_def[i] >> 24);
		rq_filter[i].protocol_valid = 0;
		rq_filter[i].protocol = 0;
	}

	/* Accumulate counts for each filter type used, build rq_ids[] list */
	for (i = 0; i < hal->hal_rq_count; i++) {
		rq = hal->hal_rq[i];
		for (j = 0; j < SLI4_CMD_REG_FCFI_NUM_RQ_CFG; j++) {
			if (rq->filter_mask & (1U << j)) {
				if (rq_filter[j].rq_id != 0xffff) {
					/* Already used. Bailout ifts not RQset case */
					if (!rq->is_mrq || (rq_filter[j].rq_id != rq->base_mrq_id)) {
						ocs_log_err(hal->os, "Wrong queue topology\n");
						return OCS_HAL_RTN_ERROR;
					}
					continue;
				} else {
					if (rq->is_mrq) {
						rq_filter[j].rq_id  = rq->base_mrq_id;
						rq_filter[j].filtermask = rq->filter_mask;
						rq_filter[j].is_mrq = TRUE;
						rq_filter[j].mrq_set_count = rq->mrq_set_count;
						rq_filter[j].mrq_set_num = rq->mrq_set_num;
						rq_filter[j].mrq_policy = rq->policy;

						/* Protocol check is mandatory; this is Prism asic expectation */
						rq_filter[j].protocol_valid = TRUE;
						if (rq->policy == REG_FCFI_RQ_SELECTION_POLICY_FC_NVME)
							rq_filter[j].protocol = REG_FCFI_RQ_CMD_PROTOCOL_TYPE_NVME;
						else
							rq_filter[j].protocol = REG_FCFI_RQ_CMD_PROTOCOL_TYPE_SCSI;
					} else {
						rq_filter[j].rq_id = rq->hdr->id;
					}

					ocs_log_info(hal->os, "REG_FCFI: filter[%d] %08X -> RQ[%d] id=%d mrq=%s\n",
						     j, hal->config.filter_def[j], i, rq->hdr->id,
						     rq->is_mrq ? "true" : "false");
					rq_filter_cnt++;
				}
			}
		}
	}

issue_cmd:
	/* Invoke REG_FCFI_MRQ */
	rc = sli_cmd_reg_fcfi_mrq(&hal->sli,
				 buf,					/* buf */
				 SLI4_BMBX_SIZE,			/* size */
				 mode,					/* mode 1 */
				 fcf_index,				/* fcf_index */
				 vlanid,				/* vlan_id */
				 rq_filter,				/* RQ filter */
				 rq_filter_cnt);			/* Number of RQ filters */
	if (rc == 0) {
		ocs_log_err(hal->os, "sli_cmd_reg_fcfi_mrq() failed: %d\n", rc);
		return OCS_HAL_RTN_ERROR;
	}

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		return OCS_HAL_RTN_ERROR;

	if (mode == SLI4_CMD_REG_FCFI_SET_FCFI_MODE) {
		rsp = (sli4_cmd_reg_fcfi_mrq_t *)buf;
		hal->fcf_indicator = rsp->fcfi;
	}

	return rc;
}

/**
 * @brief Callback function for getting linkcfg during HAL initialization.
 *
 * @param status Status of the linkcfg get operation.
 * @param value Link configuration enum to which the link configuration is set.
 * @param arg Callback argument (ocs_hal_t *).
 *
 * @return None.
 */
static void
ocs_hal_init_linkcfg_cb(int32_t status, uintptr_t value, void *arg)
{
	ocs_hal_t *hal = (ocs_hal_t *)arg;

	if (status == 0) {
		hal->linkcfg = (ocs_hal_linkcfg_e)value;
	} else {
		hal->linkcfg = OCS_HAL_LINKCFG_NA;
	}

	ocs_log_debug(hal->os, "linkcfg=%d\n", hal->linkcfg);
}

/**
 * @ingroup devInitShutdown
 * @brief Tear down the Hardware Abstraction Layer module.
 *
 * @par Description
 * Frees memory structures needed by the device, and shuts down the device. Does
 * not free the HAL context memory (which is done by the caller).
 *
 * @param hal Hardware context allocated by the caller.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_teardown(ocs_hal_t *hal)
{
	uint32_t	i = 0;
	uint32_t	iters = 10;/*XXX*/
	uint32_t	max_rpi;
	uint32_t destroy_queues;
	uint32_t free_memory;
	int32_t rc;

	if (!hal) {
		ocs_log_err(NULL, "bad parameter(s) hal=%p\n", hal);
		return OCS_HAL_RTN_ERROR;
	}

	destroy_queues = (hal->state == OCS_HAL_STATE_ACTIVE);
	free_memory = (hal->state != OCS_HAL_STATE_UNINITIALIZED);

	/* shutdown target wqe timer */
	shutdown_target_wqe_timer(hal);

	/* Cancel watchdog timer if enabled */
	if (hal->watchdog_timeout) {
		hal->watchdog_timer_disable = true;
		rc = ocs_hal_config_watchdog_timer(hal);
		if (rc)
			ocs_del_timer(&hal->watchdog_timer);
	}

	/* Cancel Sliport Healthcheck */
	if (hal->sliport_healthcheck) {
		hal->sliport_healthcheck = 0;
		ocs_hal_config_sli_port_health_check(hal, 0, 0);
	}

	if (hal->state != OCS_HAL_STATE_QUEUES_ALLOCATED) {
		hal->state = OCS_HAL_STATE_TEARDOWN_IN_PROGRESS;

		ocs_hal_flush(hal);

		/* If there are outstanding commands, wait for them to complete */
		while (!ocs_list_empty(&hal->cmd_head) && iters) {
			ocs_delay_usec(10000);
			ocs_hal_flush(hal);
			iters--;
		}

		if (ocs_list_empty(&hal->cmd_head)) {
			ocs_log_debug(hal->os, "All commands completed on MQ queue\n");
		} else {
			ocs_log_debug(hal->os, "Some commands still pending on MQ queue\n");
		}

		/* Cancel any remaining commands */
		ocs_hal_command_cancel(hal);
	} else {
		hal->state = OCS_HAL_STATE_TEARDOWN_IN_PROGRESS;
	}

#if !defined(OCSU_FC_WORKLOAD)
	/* Clear the dump_to_host buffers */
	ocs_hal_free_dump_to_host_buffers(hal);
#endif

	ocs_lock_free(&hal->cmd_lock);

	/* Free unregistered RPI if workaround is in force */
	if (hal->workaround.use_unregistered_rpi) {
		if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_RPI, hal->workaround.unregistered_rid))
			ocs_log_err(hal->os, "FCOE_RPI (%d) free failed\n", hal->workaround.unregistered_rid);
	}

	max_rpi = sli_get_max_rsrc(&hal->sli, SLI_RSRC_FCOE_RPI);
	if (hal->rpi_ref) {
		for (i = 0; i < max_rpi; i++) {
			if (ocs_atomic_read(&hal->rpi_ref[i].rpi_count)) {
				ocs_log_debug(hal->os, "non-zero ref[%d]=%d\n", i,
					ocs_atomic_read(&hal->rpi_ref[i].rpi_count));
			}
		}
		ocs_free(hal->os, hal->rpi_ref, max_rpi * sizeof(*hal->rpi_ref));
		hal->rpi_ref = NULL;
	}

	ocs_dma_free(hal->os, &hal->rnode_mem);

	if (hal->io) {
		for (i = 0; i < hal->config.n_io; i++) {
			if (!hal->io[i]) {
				continue;
			}

			if (hal->io[i]->sgl && hal->io[i]->sgl->virt) {
				if (hal->io[i]->is_port_owned) {
					ocs_lock_free(&hal->io[i]->tow_lock);
				}
				ocs_dma_free(hal->os, hal->io[i]->sgl);
			}

			if (hal->xfer_rdy && hal->xfer_rdy[i].virt) {
				ocs_dma_free(hal->os, &hal->xfer_rdy[i]);
			}

			ocs_lock_free(&hal->io[i]->lock);
			ocs_free(hal->os, hal->io[i], sizeof(ocs_hal_io_t));
			hal->io[i] = NULL;
		}

		ocs_free(hal->os, hal->wqe_buffs, hal->config.n_io * hal->sli.config.wqe_size);
		hal->wqe_buffs = NULL;
		ocs_free(hal->os, hal->io, hal->config.n_io * sizeof(ocs_hal_io_t *));
		hal->io = NULL;
	}

	if (hal->xfer_rdy) {
		ocs_free(hal->os, hal->xfer_rdy, hal->config.n_io * sizeof(ocs_dma_t));
		hal->xfer_rdy = NULL;
	}

	ocs_dma_free(hal->os, &hal->chip_dump_sges);
	ocs_dma_free(hal->os, &hal->func_dump_sges);
	ocs_dma_free(hal->os, &hal->loop_map);

	ocs_lock_free(&hal->io_timed_wqe_lock);
	ocs_lock_free(&hal->io_lock);

	for (i = 0; i < hal->wq_count; i++) {
		sli_queue_free(&hal->sli, hal->wq[i], destroy_queues, free_memory);
	}


	for (i = 0; i < hal->rq_count; i++) {
		sli_queue_free(&hal->sli, hal->rq[i], destroy_queues, free_memory);
	}

	for (i = 0; i < hal->mq_count; i++) {
		sli_queue_free(&hal->sli, hal->mq[i], destroy_queues, free_memory);
	}

	for (i = 0; i < hal->cq_count; i++) {
		sli_queue_free(&hal->sli, hal->cq[i], destroy_queues, free_memory);
	}

	for (i = 0; i < hal->eq_count; i++) {
		sli_queue_free(&hal->sli, hal->eq[i], destroy_queues, free_memory);
	}

	ocs_hal_qtop_free(hal->qtop);

	/* Free rq buffers */
	ocs_hal_rx_free(hal);

	hal_queue_teardown(hal);

	ocs_hal_rqpair_teardown(hal);

	ocs_hal_teardown_fw_diagnostic_logging(hal);

	if (sli_teardown(&hal->sli)) {
		ocs_log_err(hal->os, "SLI teardown failed\n");
	}

	/* Free the queue hashes */
	ocs_hal_queue_hash_free(hal);

	/* Free the SLI queue objects */
	ocs_hal_sli_queue_free(hal);

	ocs_queue_history_free(&hal->q_hist);

	/* record the fact that the queues are non-functional */
	hal->state = OCS_HAL_STATE_UNINITIALIZED;

	/* free sequence free pool */
	ocs_array_free(hal->seq_pool);
	hal->seq_pool = NULL;

	/* free hal_wq_callback pool */
	ocs_pool_free(hal->wq_reqtag_pool);

	/* Mark HAL setup as not having been called */
	hal->hal_setup_called = FALSE;

	return OCS_HAL_RTN_SUCCESS;
}

static ocs_hal_rtn_e
ocs_hal_sli_reset(ocs_hal_t *hal, ocs_hal_reset_e reset, ocs_hal_state_e prev_state)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	switch(reset) {
	case OCS_HAL_RESET_FUNCTION:
		ocs_log_debug(hal->os, "issuing function level reset\n");
		if (sli_reset(&hal->sli)) {
			ocs_log_err(hal->os, "sli_reset failed\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_RESET_FIRMWARE:
		ocs_log_debug(hal->os, "issuing firmware reset\n");
		if (sli_fw_reset(&hal->sli)) {
			ocs_log_err(hal->os, "sli_soft_reset failed\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		/*
		 * Because the FW reset leaves the FW in a non-running state,
		 * follow that with a regular reset.
		 */
		ocs_log_debug(hal->os, "issuing function level reset\n");
		if (sli_reset(&hal->sli)) {
			ocs_log_err(hal->os, "sli_reset failed\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	default:
		ocs_log_test(hal->os, "unknown reset type - no reset performed\n");
		hal->state = prev_state;
		rc = OCS_HAL_RTN_INVALID_ARG;
		break;
	}

	return rc;
}

ocs_hal_rtn_e
ocs_hal_port_migration(ocs_hal_t *hal)
{
	ocs_hal_rtn_e	rc = OCS_HAL_RTN_SUCCESS;

	ocs_log_debug(hal->os, "issuing Port migration\n");
	if (sli_port_migration(&hal->sli)) {
		ocs_log_err(hal->os, "sli_port_migration failed\n");
		rc = OCS_HAL_RTN_ERROR;
	}

	return rc;
}
ocs_hal_rtn_e
ocs_hal_reset(ocs_hal_t *hal, ocs_hal_reset_e reset)
{
	uint32_t	i, iters;
	ocs_hal_rtn_e	rc = OCS_HAL_RTN_SUCCESS;
	ocs_hal_state_e	prev_state = hal->state;
	uint32_t	destroy_queues, free_memory;
	ocs_domain_t	*domain;
	ocs_t		*ocs = (ocs_t *)hal->os;

	if (hal->state != OCS_HAL_STATE_ACTIVE)
		ocs_log_test(hal->os, "HAL state %d is not active\n", hal->state);

	destroy_queues = (hal->state == OCS_HAL_STATE_ACTIVE);
	free_memory = (hal->state != OCS_HAL_STATE_UNINITIALIZED);
	hal->state = OCS_HAL_STATE_RESET_IN_PROGRESS;

	/* If the prev_state is already reset/teardown in progress, don't continue further */
	if (prev_state == OCS_HAL_STATE_RESET_IN_PROGRESS ||
			prev_state == OCS_HAL_STATE_TEARDOWN_IN_PROGRESS) {
		return ocs_hal_sli_reset(hal, reset, prev_state);
	}

	/* shutdown target wqe timer */
	shutdown_target_wqe_timer(hal);

	/* Cancel watchdog timer if enabled */
	if (hal->watchdog_timeout) {
		hal->watchdog_timer_disable = true;
		rc = ocs_hal_config_watchdog_timer(hal);
		if (rc)
			ocs_del_timer(&hal->watchdog_timer);
	}

	if (prev_state != OCS_HAL_STATE_UNINITIALIZED) {
		ocs_hal_flush(hal);

		/*
		 * If an mailbox command requiring a DMA is outstanding (i.e. SFP/DDM),
		 * then the FW will UE when the reset is issued. So attempt to complete
		 * all mailbox commands.
		 */
		iters = 10;
		while (!ocs_list_empty(&hal->cmd_head) && iters) {
			ocs_delay_usec(10000);
			ocs_hal_flush(hal);
			iters--;
		}

		if (ocs_list_empty(&hal->cmd_head)) {
			ocs_log_debug(hal->os, "All commands completed on MQ queue\n");
		} else {
			ocs_log_debug(hal->os, "Some commands still pending on MQ queue\n");
		}
	}

	/* Reset the chip */
	rc = ocs_hal_sli_reset(hal, reset, prev_state);
	if (rc == OCS_HAL_RTN_INVALID_ARG) {
		return OCS_HAL_RTN_ERROR;
	}

	/* Not safe to walk command/io lists unless they've been initialized */
	if (prev_state != OCS_HAL_STATE_UNINITIALIZED) {
		ocs_hal_command_cancel(hal);

		/* Try to clean up the io_inuse list */
		ocs_hal_io_cancel(hal);

		ocs_memset(hal->domains, 0, sizeof(hal->domains));
		ocs_memset(hal->fcf_index_fcfi, 0, sizeof(hal->fcf_index_fcfi));

		ocs_hal_link_event_init(hal);

		/**
		 * The IO lists must be empty, but remove any that didn't get cleaned up.
		 * Clean up the timed wqe list, the free list, and the wait free list.
		 */
		ocs_lock(&hal->io_timed_wqe_lock);
			while (!ocs_list_empty(&hal->io_timed_wqe)) {
				ocs_list_remove_head(&hal->io_timed_wqe);
			}
		ocs_unlock(&hal->io_timed_wqe_lock);

		ocs_lock(&hal->io_lock);
			while (!ocs_list_empty(&hal->io_free)) {
				ocs_list_remove_head(&hal->io_free);
			}

			while (!ocs_list_empty(&hal->io_wait_free)) {
				ocs_list_remove_head(&hal->io_wait_free);
			}
		ocs_unlock(&hal->io_lock);
	}

	/*
	 * IO inuse has been cleaned-up; now call domain force free
	 * before queue reset and RX buffer free.
	 */
	while ((domain = ocs_list_get_head(&(ocs->domain_list))) != NULL) {
		ocs_log_debug(ocs, "free domain %p\n", domain);
		ocs_domain_force_free(domain);
	}

	if (prev_state != OCS_HAL_STATE_UNINITIALIZED) {
		for (i = 0; i < hal->wq_count; i++) {
			sli_queue_free(&hal->sli, hal->wq[i], destroy_queues, free_memory);
		}

		for (i = 0; i < hal->rq_count; i++) {
			sli_queue_free(&hal->sli, hal->rq[i], destroy_queues, free_memory);
		}

		for (i = 0; i < hal->hal_rq_count; i++) {
			hal_rq_t *rq = hal->hal_rq[i];
			if (rq->rq_tracker != NULL) {
				uint32_t j;

				for (j = 0; j < rq->entry_count; j++) {
					rq->rq_tracker[j] = NULL;
				}
			}
		}

		for (i = 0; i < hal->mq_count; i++) {
			sli_queue_free(&hal->sli, hal->mq[i], destroy_queues, free_memory);
		}

		for (i = 0; i < hal->cq_count; i++) {
			sli_queue_free(&hal->sli, hal->cq[i], destroy_queues, free_memory);
		}

		for (i = 0; i < hal->eq_count; i++) {
			sli_queue_free(&hal->sli, hal->eq[i], destroy_queues, free_memory);
		}

		/* Free rq buffers */
		ocs_hal_rx_free(hal);

		/* Teardown the HAL queue topology */
		hal_queue_teardown(hal);

		/* Reset the request tag pool, the HAL IO request tags are reassigned in ocs_hal_setup_io() */
		ocs_hal_reqtag_reset(hal);
	} else {

		/* Free rq buffers */
		ocs_hal_rx_free(hal);
	}

	/*
	 * Re-apply the run-time workarounds after clearing the SLI config
	 * fields in sli_reset.
	 */
	ocs_hal_workaround_setup(hal);
	return rc;
}

int32_t
ocs_hal_get_num_eq(ocs_hal_t *hal)
{
	return hal->eq_count;
}

static int32_t
ocs_hal_get_fw_timed_out(ocs_hal_t *hal)
{
	/*
	 * The error values below are taken from LOWLEVEL_SET_WATCHDOG_TIMER_rev1.pdf.
	 * No further explanation is given in the document.
	 */
	return (sli_reg_read(&hal->sli, SLI4_REG_SLIPORT_ERROR1) == 0x2 &&
		sli_reg_read(&hal->sli, SLI4_REG_SLIPORT_ERROR2) == 0x10);
}

ocs_hal_rtn_e
ocs_hal_get(ocs_hal_t *hal, ocs_hal_property_e prop, uint32_t *value)
{
	ocs_hal_rtn_e		rc = OCS_HAL_RTN_SUCCESS;
	int32_t			tmp;

	if (!value) {
		return OCS_HAL_RTN_ERROR;
	}

	*value = 0;

	switch (prop) {
	case OCS_HAL_N_IO:
		*value = hal->config.n_io;
		break;
	case OCS_HAL_N_SGL:
		*value = (hal->config.n_sgl - SLI4_SGE_MAX_RESERVED);
		break;
	case OCS_HAL_MAX_IO:
		*value = sli_get_max_rsrc(&hal->sli, SLI_RSRC_FCOE_XRI);
		break;
	case OCS_HAL_MAX_NODES:
		*value = sli_get_max_rsrc(&hal->sli, SLI_RSRC_FCOE_RPI);
		break;
	case OCS_HAL_MAX_RQ_ENTRIES:
		*value = hal->num_qentries[SLI_QTYPE_RQ];
		break;
	case OCS_HAL_RQ_DEFAULT_BUFFER_SIZE:
		*value = hal->config.rq_default_buffer_size;
		break;
	case OCS_HAL_AUTO_XFER_RDY_CAPABLE:
		*value = sli_get_auto_xfer_rdy_capable(&hal->sli);
		break;
	case OCS_HAL_TOW_CAPABLE:
		*value = sli_get_tow_capable(&hal->sli);
		break;
	case OCS_HAL_TOW_XRIS_MAX:
		*value = sli_get_tow_xris_max(&hal->sli);
		break;
	case OCS_HAL_TOW_BLK_SIZE:
		switch (hal->config.tow_blksize_chip) {
		case 0:
			*value = 512;
			break;
		case 1:
			*value = 1024;
			break;
		case 2:
			*value = 2048;
			break;
		case 3:
			*value = 4096;
			break;
		case 4:
			*value = 520;
			break;
		default:
			*value = 0;
			rc = OCS_HAL_RTN_ERROR;
			break;
		}
		break;
	case OCS_HAL_TOW_T10_ENABLE:
		*value = hal->config.tow_t10_enable;
		break;
	case OCS_HAL_TOW_P_TYPE:
		*value = hal->config.tow_p_type;
		break;
	case OCS_HAL_TOW_REF_TAG_LBA:
		*value = hal->config.tow_ref_tag_lba;
		break;
	case OCS_HAL_TOW_APP_TAG_VALID:
		*value = hal->config.tow_app_tag_valid;
		break;
	case OCS_HAL_TOW_APP_TAG_VALUE:
		*value = hal->config.tow_app_tag_value;
		break;
	case OCS_HAL_TOW_FEATURE:
		*value = hal->config.tow_feature;
		break;
	case OCS_HAL_TOW_XRI_CNT:
		*value = hal->config.tow_xri_cnt;
		break;
	case OCS_HAL_TOW_IO_SIZE:
		*value = hal->config.tow_io_size;
		break;
	case OCS_HAL_MAX_SGE:
		*value = sli_get_max_sge(&hal->sli);
		break;
	case OCS_HAL_MAX_SGL:
		*value = sli_get_max_sgl(&hal->sli);
		break;
	case OCS_HAL_TOPOLOGY:
		/*
		 * Infer link.status based on link.speed.
		 * Report OCS_HAL_TOPOLOGY_NONE if the link is down.
		 */
		if (hal->link.speed == 0) {
			*value = OCS_HAL_TOPOLOGY_NONE;
			break;
		}
		switch (hal->link.topology) {
		case SLI_LINK_TOPO_NPORT:
			*value = OCS_HAL_TOPOLOGY_NPORT;
			break;
		case SLI_LINK_TOPO_LOOP:
			*value = OCS_HAL_TOPOLOGY_LOOP;
			break;
		case SLI_LINK_TOPO_NONE:
			*value = OCS_HAL_TOPOLOGY_NONE;
			break;
		default:
			ocs_log_test(hal->os, "unsupported topology %#x\n", hal->link.topology);
			rc = OCS_HAL_RTN_ERROR;
			break;
		}
		break;
	case OCS_HAL_CONFIG_TOPOLOGY:
		*value = hal->config.topology;
		break;
	case OCS_HAL_LINK_SPEED:
		*value = hal->link.speed;
		break;
	case OCS_HAL_LOGICAL_LINK_SPEED:
		*value = hal->link.logical_link_speed;
		break;
	case OCS_HAL_AGGREGATE_LINK_SPEED:
		*value = hal->link.aggregate_link_speed;
		break;
	case OCS_HAL_LINK_CONFIG_SPEED:
		switch (hal->config.speed) {
		case FC_LINK_SPEED_10G:
			*value = 10000;
			break;
		case FC_LINK_SPEED_AUTO_32_16_8:
			*value = 0;
			break;
		case FC_LINK_SPEED_1G:
			*value = 1000;
			break;
		case FC_LINK_SPEED_2G:
			*value = 2000;
			break;
		case FC_LINK_SPEED_4G:
			*value = 4000;
			break;
		case FC_LINK_SPEED_8G:
			*value = 8000;
			break;
		case FC_LINK_SPEED_16G:
			*value = 16000;
			break;
		case FC_LINK_SPEED_32G:
			*value = 32000;
			break;
		case FC_LINK_SPEED_64G:
			*value = 64000;
			break;
		case FC_LINK_SPEED_128G:
			*value = 128000;
			break;
		case FC_LINK_SPEED_256G:
			*value = 256000;
			break;
		default:
			ocs_log_test(hal->os, "unsupported speed %#x\n", hal->config.speed);
			rc = OCS_HAL_RTN_ERROR;
			break;
		}
		break;
	case OCS_HAL_IF_TYPE:
		*value = sli_get_if_type(&hal->sli);
		break;
	case OCS_HAL_SLI_REV:
		*value = sli_get_sli_rev(&hal->sli);
		break;
	case OCS_HAL_SLI_FAMILY:
		*value = sli_get_sli_family(&hal->sli);
		break;
	case OCS_HAL_DIF_CAPABLE:
		*value = sli_get_dif_capable(&hal->sli);
		break;
	case OCS_HAL_DIF_SEED:
		*value = hal->config.dif_seed;
		break;
	case OCS_HAL_DIF_MODE:
		*value = hal->config.dif_mode;
		break;
	case OCS_HAL_DIF_MULTI_SEPARATE:
		/* Lancer supports multiple DIF separates */
		if (hal->sli.if_type == SLI4_IF_TYPE_LANCER_FC_ETH) {
			*value = TRUE;
		} else {
			*value = FALSE;
		}
		break;
	case OCS_HAL_DUMP_MAX_SIZE:
		*value = hal->dump_size;
		break;
	case OCS_HAL_DUMP_READY:
		*value = sli_dump_is_ready(&hal->sli);
		break;
	case OCS_HAL_DUMP_PRESENT:
	{
		ocs_t *ocs = (ocs_t *)hal->os;

		/**
		 * When auto recovery is enabled, SLIPort flag will be cleared by the
		 * set_dump_location mbox cmd in the hal reset path. So, we have to
		 * rely on OCS dump_state flag to validate the presence of FW dumps.
		 *
		 * Check for FDD, and return if a dump exists on this PCI func.
		 * This can even return the presence of CLD on PCI func 0 since we
		 * really don't care about the type of dump that exists in OCS buffers.
		 */
		*value = (ocs_fw_dump_state_check(ocs, OCS_FW_FUNC_DUMP_STATE_VALID) ||
			  sli_dump_is_present(&hal->sli));

		/* If FW dump is present, set the appropriate fw_dump_type */
		if (*value) {
			*value = ocs->fw_dump.type;
		}
		break;
	}
	case OCS_HAL_RESET_REQUIRED:
		tmp = sli_reset_required(&hal->sli);
		if(tmp < 0) {
			rc = OCS_HAL_RTN_ERROR;
		} else {
			*value = tmp;
		}
		break;
	case OCS_HAL_FW_ERROR:
		*value = sli_fw_error_status(&hal->sli);
		break;
	case OCS_HAL_FW_READY:
		*value = sli_fw_ready(&hal->sli);
		break;
	case OCS_HAL_FW_TIMED_OUT:
		*value = ocs_hal_get_fw_timed_out(hal);
		break;
	case OCS_HAL_HIGH_LOGIN_MODE:
		*value = sli_get_hlm_capable(&hal->sli);
		break;
	case OCS_HAL_DPP_MODE:
		*value = sli_get_dpp_capable(&hal->sli);
		break;
	case OCS_HAL_PREREGISTER_SGL:
		*value = sli_get_sgl_preregister_required(&hal->sli);
		break;
	case OCS_HAL_HW_REV1:
		*value = sli_get_hw_revision(&hal->sli, 0);
		break;
	case OCS_HAL_HW_REV2:
		*value = sli_get_hw_revision(&hal->sli, 1);
		break;
	case OCS_HAL_HW_REV3:
		*value = sli_get_hw_revision(&hal->sli, 2);
		break;
	case OCS_HAL_LINKCFG:
		*value = hal->linkcfg;
		break;
	case OCS_HAL_ETH_LICENSE:
		*value = hal->eth_license;
		break;
	case OCS_HAL_LINK_MODULE_TYPE:
		*value = sli_get_link_module_type(&hal->sli);
		break;
	case OCS_HAL_NUM_CHUTES:
		*value = ocs_hal_get_num_chutes(hal);
		break;
	case OCS_HAL_DISABLE_AR_TGT_DIF:
		*value = hal->workaround.disable_ar_tgt_dif;
		break;
	case OCS_HAL_EMULATE_I_ONLY_AAB:
		*value = hal->config.i_only_aab;
		break;
	case OCS_HAL_EMULATE_TARGET_WQE_TIMEOUT:
		*value = hal->config.emulate_tgt_wqe_timeout;
		break;
	case OCS_HAL_VPD_LEN:
		*value = sli_get_vpd_len(&hal->sli);
		break;
	case OCS_HAL_SGL_CHAINING_CAPABLE:
		*value = sli_get_is_sgl_chaining_capable(&hal->sli) || hal->workaround.sglc_misreported;
		break;
	case OCS_HAL_SGL_CHAINING_ALLOWED:
		/*
		 * SGL Chaining is allowed in the following cases:
		 *   1. Lancer with host SGL Lists
		 *   2. Skyhawk with pre-registered SGL Lists
		 */
		*value = FALSE;
		if ((sli_get_is_sgl_chaining_capable(&hal->sli) || hal->workaround.sglc_misreported) &&
		    !sli_get_sgl_preregister(&hal->sli) &&
		    SLI4_IF_TYPE_LANCER_FC_ETH  == sli_get_if_type(&hal->sli)) {
			*value = TRUE;
		}

		if ((sli_get_is_sgl_chaining_capable(&hal->sli) || hal->workaround.sglc_misreported) &&
		    sli_get_sgl_preregister(&hal->sli) &&
		    ((SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(&hal->sli)) ||
		     (SLI4_IF_TYPE_BE3_SKH_VF == sli_get_if_type(&hal->sli)))) {
			*value = TRUE;
		}
		break;
	case OCS_HAL_SGL_CHAINING_HOST_ALLOCATED:
		/* Only lancer supports host allocated SGL Chaining buffers. */
		*value = ((sli_get_is_sgl_chaining_capable(&hal->sli) || hal->workaround.sglc_misreported) &&
			  (SLI4_IF_TYPE_LANCER_FC_ETH  == sli_get_if_type(&hal->sli)));
		break;
	case OCS_HAL_SEND_FRAME_CAPABLE:
		*value = ((sli_get_if_type(&hal->sli) == SLI4_IF_TYPE_LANCER_FC_ETH) ||
			  (sli_get_if_type(&hal->sli) == SLI4_IF_TYPE_LANCER_G7));
		break;
	case OCS_HAL_RR_QUANTA:
		*value = hal->config.rr_quanta;
		break;
	case OCS_HAL_MAX_VPORTS:
		*value = sli_get_max_rsrc(&hal->sli, SLI_RSRC_FCOE_VPI);
		break;
	case OCS_HAL_PORTNUM:
		*value = sli_get_portnum(&hal->sli);
		break;
	case OCS_HAL_SUPPRESS_RSP_CAPABLE:
		*value = hal->config.suppress_rsp_capable;
		break;
	case OCS_HAL_POLL_MODE:
		*value = hal->config.poll_mode;
		break;
	case OCS_HAL_NSLER_CAPABLE:
		*value = hal->config.nsler;
		break;
	case OCS_HAL_ENABLE_DUAL_DUMP:
		*value = hal->config.enable_dual_dump;
		break;
	default:
		ocs_log_test(hal->os, "unsupported property %#x\n", prop);
		rc = OCS_HAL_RTN_ERROR;
	}

	return rc;
}

void *
ocs_hal_get_ptr(ocs_hal_t *hal, ocs_hal_property_e prop)
{
	void	*rc = NULL;

	switch (prop) {
	case OCS_HAL_WWN_NODE:
		rc = sli_get_wwn_node(&hal->sli);
		break;
	case OCS_HAL_WWN_PORT:
		rc = sli_get_wwn_port(&hal->sli);
		break;
	case OCS_HAL_VPD:
		/* make sure VPD length is non-zero */
		if (sli_get_vpd_len(&hal->sli)) {
			rc = sli_get_vpd(&hal->sli);
		}
		break;
	case OCS_HAL_FW_REV:
		rc = sli_get_fw_name(&hal->sli, 0);
		break;
	case OCS_HAL_FW_REV2:
		rc = sli_get_fw_name(&hal->sli, 1);
		break;
	case OCS_HAL_IPL:
		rc = sli_get_ipl_name(&hal->sli);
		break;
	case OCS_HAL_BIOS_VERSION_STRING:
		rc = sli_get_bios_version_string(&hal->sli);
		break;
	default:
		ocs_log_test(hal->os, "unsupported property %#x\n", prop);
	}

	return rc;
}



ocs_hal_rtn_e
ocs_hal_set(ocs_hal_t *hal, ocs_hal_property_e prop, uint32_t value)
{
	ocs_hal_rtn_e		rc = OCS_HAL_RTN_SUCCESS;

	switch (prop) {
	case OCS_HAL_N_IO:
		if (value > sli_get_max_rsrc(&hal->sli, SLI_RSRC_FCOE_XRI) ||
		    value == 0) {
			ocs_log_test(hal->os, "IO value out of range %d vs %d\n", value,
				     sli_get_max_rsrc(&hal->sli, SLI_RSRC_FCOE_XRI));
			rc = OCS_HAL_RTN_ERROR;
		} else {
			hal->config.n_io = value;
		}
		break;
	case OCS_HAL_N_SGL:
		value += SLI4_SGE_MAX_RESERVED;
		if (value > sli_get_max_sgl(&hal->sli)) {
			ocs_log_test(hal->os, "SGL value out of range %d vs %d\n",
				     value, sli_get_max_sgl(&hal->sli));
			rc = OCS_HAL_RTN_ERROR;
		} else {
			hal->config.n_sgl = value;
		}
		break;
	case OCS_HAL_TOPOLOGY:
		if ((sli_get_medium(&hal->sli) != SLI_LINK_MEDIUM_FC) &&
				(value != OCS_HAL_TOPOLOGY_AUTO)) {
			ocs_log_err(hal->os, "Unsupported topology=%#x medium=%#x\n",
				    value, sli_get_medium(&hal->sli));
			rc = OCS_HAL_RTN_ERROR;
			break;
		}

		switch (value) {
		case OCS_HAL_TOPOLOGY_AUTO:
			if (sli_get_medium(&hal->sli) == SLI_LINK_MEDIUM_FC) {
				sli_set_topology(&hal->sli, SLI4_READ_CFG_TOPO_FC);
			} else {
				sli_set_topology(&hal->sli, SLI4_READ_CFG_TOPO_FCOE);
			}
			break;
		case OCS_HAL_TOPOLOGY_NPORT:
			sli_set_topology(&hal->sli, SLI4_READ_CFG_TOPO_FC_DA);
			break;
		case OCS_HAL_TOPOLOGY_LOOP:
			sli_set_topology(&hal->sli, SLI4_READ_CFG_TOPO_FC_AL);
			break;
		default:
			ocs_log_test(hal->os, "unsupported topology %#x\n", value);
			rc = OCS_HAL_RTN_ERROR;
		}
		hal->config.topology = value;
		break;
	case OCS_HAL_LINK_SPEED:
		if (sli_get_medium(&hal->sli) != SLI_LINK_MEDIUM_FC) {
			switch (value) {
			case 0:		/* Auto-speed negotiation */
			case 10000:	/* FCoE speed */
				hal->config.speed = FC_LINK_SPEED_10G;
				break;
			default:
				ocs_log_test(hal->os, "unsupported speed=%#x medium=%#x\n",
					     value, sli_get_medium(&hal->sli));
				rc = OCS_HAL_RTN_ERROR;
			}
			break;
		}

		switch (value) {
		case 0:		/* Auto-speed negotiation */
			hal->config.speed = FC_LINK_SPEED_AUTO_32_16_8;
			break;
		case 1000:	/* FC speeds */
			hal->config.speed = FC_LINK_SPEED_1G;
			break;
		case 2000:
			hal->config.speed = FC_LINK_SPEED_2G;
			break;
		case 4000:
			hal->config.speed = FC_LINK_SPEED_4G;
			break;
		case 8000:
			hal->config.speed = FC_LINK_SPEED_8G;
			break;
		case 16000:
			hal->config.speed = FC_LINK_SPEED_16G;
			break;
		case 32000:
			hal->config.speed = FC_LINK_SPEED_32G;
			break;
		case 64000:
			hal->config.speed = FC_LINK_SPEED_64G;
			break;
		case 128000:
			hal->config.speed = FC_LINK_SPEED_128G;
			break;
		case 256000:
			hal->config.speed = FC_LINK_SPEED_256G;
			break;
		default:
			ocs_log_test(hal->os, "unsupported speed %d\n", value);
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_DIF_SEED:
		/* Set the DIF seed - only for lancer right now */
		if (SLI4_IF_TYPE_LANCER_FC_ETH != sli_get_if_type(&hal->sli)) {
			ocs_log_test(hal->os, "DIF seed not supported for this device\n");
			rc = OCS_HAL_RTN_ERROR;
		} else {
			hal->config.dif_seed = value;
		}
		break;
	case OCS_HAL_DIF_MODE:
		switch (value) {
		case OCS_HAL_DIF_MODE_INLINE:
			/*
			 *  Make sure we support inline DIF.
			 *
			 * Note: Having both bits clear means that we have old
			 *       FW that doesn't set the bits.
			 */
			if (sli_is_dif_inline_capable(&hal->sli)) {
				hal->config.dif_mode = value;
			} else {
				ocs_log_test(hal->os, "chip does not support DIF inline\n");
				rc = OCS_HAL_RTN_ERROR;
			}
			break;
		case OCS_HAL_DIF_MODE_SEPARATE:
			/* Make sure we support DIF separates. */
			if (sli_is_dif_separate_capable(&hal->sli)) {
				hal->config.dif_mode = value;
			} else {
				ocs_log_test(hal->os, "chip does not support DIF separate\n");
				rc = OCS_HAL_RTN_ERROR;
			}
		}
		break;
	case OCS_HAL_RQ_PROCESS_LIMIT: {
		hal_rq_t *rq;
		uint32_t i;

		/* For each hal_rq object, set its parent CQ limit value */
		for (i = 0; i < hal->hal_rq_count; i++) {
			rq = hal->hal_rq[i];
			hal->cq[rq->cq->instance]->proc_limit = value;
		}
		break;
	}
	case OCS_HAL_RQ_DEFAULT_BUFFER_SIZE:
		hal->config.rq_default_buffer_size = value;
		break;
	case OCS_HAL_TOW_IO_SIZE:
		hal->config.tow_io_size = value;
		break;
	case OCS_HAL_TOW_XRI_CNT:
		hal->config.tow_xri_cnt = value;
		break;
	case OCS_HAL_TOW_FEATURE:
		hal->config.tow_feature = value;
		break;
	case OCS_HAL_TOW_BLK_SIZE:
		switch (value) {
		case 512:
			hal->config.tow_blksize_chip = 0;
			break;
		case 1024:
			hal->config.tow_blksize_chip = 1;
			break;
		case 2048:
			hal->config.tow_blksize_chip = 2;
			break;
		case 4096:
			hal->config.tow_blksize_chip = 3;
			break;
		case 520:
			hal->config.tow_blksize_chip = 4;
			break;
		default:
			ocs_log_err(hal->os, "Invalid block size %d\n", value);
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_TOW_T10_ENABLE:
		hal->config.tow_t10_enable = value;
		break;
	case OCS_HAL_TOW_P_TYPE:
		hal->config.tow_p_type = value;
		break;
	case OCS_HAL_TOW_REF_TAG_LBA:
		hal->config.tow_ref_tag_lba = value;
		break;
	case OCS_HAL_TOW_APP_TAG_VALID:
		hal->config.tow_app_tag_valid = value;
		break;
	case OCS_HAL_TOW_APP_TAG_VALUE:
		hal->config.tow_app_tag_value = value;
		break;
	case OCS_HAL_ESOC:
		hal->config.esoc = value;
		break;
	case OCS_HAL_HIGH_LOGIN_MODE:
		rc = sli_set_hlm(&hal->sli, value);
		break;
	case OCS_HAL_DPP_MODE:
		rc = sli_set_dpp(&hal->sli, value);
		break;
	case OCS_HAL_PREREGISTER_SGL:
		rc = sli_set_sgl_preregister(&hal->sli, value);
		break;
	case OCS_HAL_ETH_LICENSE:
		hal->eth_license = value;
		break;
	case OCS_HAL_EMULATE_I_ONLY_AAB:
		hal->config.i_only_aab = value;
		break;
	case OCS_HAL_EMULATE_TARGET_WQE_TIMEOUT:
		hal->config.emulate_tgt_wqe_timeout = value;
		break;
	case OCS_HAL_BOUNCE:
		hal->config.bounce = value;
		break;
	case OCS_HAL_RR_QUANTA:
		hal->config.rr_quanta = value;
		break;
	case OCS_HAL_SUPPRESS_RSP_CAPABLE:
		hal->config.suppress_rsp_capable = value;
		break;
	case OCS_HAL_POLL_MODE:
		hal->config.poll_mode = value;
		break;
	case OCS_HAL_NSLER_CAPABLE:
		hal->config.nsler = value;
		break;
	case OCS_HAL_ENABLE_DUAL_DUMP:
		hal->config.enable_dual_dump = value;
		break;
	default:
		ocs_log_test(hal->os, "unsupported property %#x\n", prop);
		rc = OCS_HAL_RTN_ERROR;
	}

	return rc;
}


ocs_hal_rtn_e
ocs_hal_set_ptr(ocs_hal_t *hal, ocs_hal_property_e prop, void *value)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	switch (prop) {
	case OCS_HAL_WAR_VERSION:
		hal->hal_war_version = value;
		break;
	case OCS_HAL_FILTER_DEF: {
		char *p = value;
		uint32_t idx = 0;

		for (idx = 0; idx < ARRAY_SIZE(hal->config.filter_def); idx++) {
			hal->config.filter_def[idx] = 0;
		}

		for (idx = 0; (idx < ARRAY_SIZE(hal->config.filter_def)) && (p != NULL) && *p; ) {
			hal->config.filter_def[idx++] = ocs_strtoul(p, 0, 0);
			p = ocs_strchr(p, ',');
			if (p != NULL) {
				p++;
			}
		}

		break;
	}
	default:
		ocs_log_test(hal->os, "unsupported property %#x\n", prop);
		rc = OCS_HAL_RTN_ERROR;
		break;
	}
	return rc;
}

ocs_hal_rtn_e
ocs_hal_dump_type2(ocs_hal_t *hal, uint16_t region_id, void *buff, size_t req_size)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];
	ocs_dma_t dma_buf;

	if (hal == NULL) {
		ocs_log_err(NULL, "Failed to access HAL\n");
		return OCS_HAL_RTN_ERROR;
	}

	if (ocs_dma_alloc(hal->os, &dma_buf, req_size, OCS_MIN_DMA_ALIGNMENT)) {
		ocs_log_err(hal->os, "Failed to alloc DMA buffer\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	ocs_memset(dma_buf.virt, 0, req_size);

	if (sli_cmd_dump_type2(&hal->sli, buf, SLI4_BMBX_SIZE, region_id, &dma_buf)) {
		rc = ocs_hal_exec_mbx_command_generic_results(hal, buf, false, 15*1000*1000);
	} else {
		ocs_log_err(hal->os, "sli_cmd_dump_type2 failed\n");
		rc = OCS_HAL_RTN_ERROR;
	}

	if (!rc)
		ocs_memcpy(buff, dma_buf.virt, req_size);

	ocs_dma_free(hal->os, &dma_buf);
	return rc;
}

ocs_hal_rtn_e
ocs_hal_update_cfg(ocs_hal_t *hal, uint16_t region_id, void *buff, size_t req_size)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	sli4_cmd_update_cfg_t *mbox_cmd;
	ocs_dma_t dma_buf;

	if (hal == NULL) {
		ocs_log_err(NULL, "Failed to access HAL\n");
		return OCS_HAL_RTN_ERROR;
	}

	mbox_cmd = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (!mbox_cmd) {
		ocs_log_err(hal->os, "Failed to MBOX memory\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	if (ocs_dma_alloc(hal->os, &dma_buf, req_size, OCS_MIN_DMA_ALIGNMENT)) {
		ocs_log_err(hal->os, "Failed to alloc DMA buffer\n");
		ocs_free(hal->os, mbox_cmd, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	ocs_memcpy(dma_buf.virt, buff, req_size);

	if (sli_cmd_update_cfg(&hal->sli, mbox_cmd, SLI4_BMBX_SIZE, region_id, &dma_buf)) {
		rc = ocs_hal_exec_mbx_command_generic_results(hal, (uint8_t *)mbox_cmd, false, 15*1000*1000);
	} else {
		ocs_log_err(hal->os, "sli_cmd_update_cfg failed\n");
		rc = OCS_HAL_RTN_ERROR;
		goto exit_update_cfg;
	}

	if (mbox_cmd->hdr.status) {
		ocs_log_err(hal->os, "Update CFG MBOX command failed; response code: %d\n", mbox_cmd->response_info);
	}

exit_update_cfg:
	ocs_free(hal->os, mbox_cmd, SLI4_BMBX_SIZE);
	ocs_dma_free(hal->os, &dma_buf);
	return rc;
}

/**
 * @ingroup interrupt
 * @brief Check for the events associated with the interrupt vector.
 *
 * @param hal Hardware context.
 * @param vector Zero-based interrupt vector number.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
ocs_hal_event_check(ocs_hal_t *hal, uint32_t vector)
{
	int32_t rc = 0;

	if (!hal) {
		ocs_log_err(NULL, "HAL context is NULL!!!\n");
		return -1;
	}

	if (vector > hal->eq_count) {
		ocs_log_err(hal->os, "vector %d, max %d\n", vector, hal->eq_count);
		return -1;
	}

	/*
	 * The caller should disable interrupts if they wish to prevent us
	 * from processing during a shutdown. The following states are defined:
	 *   OCS_HAL_STATE_UNINITIALIZED - No queues allocated
	 *   OCS_HAL_STATE_QUEUES_ALLOCATED - The state after a chip reset,
	 *                                    queues are cleared.
	 *   OCS_HAL_STATE_ACTIVE - Chip and queues are operational
	 *   OCS_HAL_STATE_RESET_IN_PROGRESS - reset, we still want completions
	 *   OCS_HAL_STATE_TEARDOWN_IN_PROGRESS - We still want mailbox
	 *                                        completions.
	 */
	if (hal->state != OCS_HAL_STATE_UNINITIALIZED) {
		rc = sli_queue_is_empty(&hal->sli, hal->eq[vector]);

		/* Re-arm queue if there are no entries */
		if (rc != 0 && !hal->config.poll_mode) {
			sli_queue_arm(&hal->sli, hal->eq[vector], TRUE);
		}
	}
	return rc;
}

void
ocs_hal_unsol_process_bounce(void *arg)
{
	ocs_hal_sequence_t *seq = arg;
	ocs_hal_t *hal = seq->hal;

	ocs_hal_assert(hal != NULL);
	ocs_hal_assert(hal->callback.unsolicited != NULL);

	hal->callback.unsolicited(hal->args.unsolicited, seq);
}

int32_t
ocs_hal_process(ocs_hal_t *hal, uint32_t vector, uint32_t max_isr_time_msec)
{
	hal_eq_t *eq;
	int32_t rc = 0;

	CPUTRACE("");

	/*
	 * The caller should disable interrupts if they wish to prevent us
	 * from processing during a shutdown. The following states are defined:
	 *   OCS_HAL_STATE_UNINITIALIZED - No queues allocated
	 *   OCS_HAL_STATE_QUEUES_ALLOCATED - The state after a chip reset,
	 *                                    queues are cleared.
	 *   OCS_HAL_STATE_ACTIVE - Chip and queues are operational
	 *   OCS_HAL_STATE_RESET_IN_PROGRESS - reset, we still want completions
	 *   OCS_HAL_STATE_TEARDOWN_IN_PROGRESS - We still want mailbox
	 *                                        completions.
	 */
	if (hal->state == OCS_HAL_STATE_UNINITIALIZED) {
		return 0;
	}

	/* Get pointer to hal_eq_t */
	eq = hal->hal_eq[vector];
	if (eq == NULL) {
		return 0;
	}

	OCS_STAT(eq->use_count++);

	rc = ocs_hal_eq_process(hal, eq, max_isr_time_msec);

	eq->queue->last_processed_time = ocs_ticks_to_ms(ocs_get_os_ticks());
	return rc;
}

/**
 * @ingroup interrupt
 * @brief Process events associated with an EQ.
 *
 * @par Description
 * Loop termination:
 * @n @n Without a mechanism to terminate the completion processing loop, it
 * is possible under some workload conditions for the loop to never terminate
 * (or at least take longer than the OS is happy to have an interrupt handler
 * or kernel thread context hold a CPU without yielding).
 * @n @n The approach taken here is to periodically check how much time
 * we have been in this
 * processing loop, and if we exceed a predetermined time (multiple seconds), the
 * loop is terminated, and ocs_hal_process() returns.
 *
 * @param hal Hardware context.
 * @param eq Pointer to HAL EQ object.
 * @param max_isr_time_msec Maximum time in msec to stay in this function.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
ocs_hal_eq_process(ocs_hal_t *hal, hal_eq_t *eq, uint32_t max_isr_time_msec)
{
	uint8_t		eqe[sizeof(sli4_eqe_t)];
	uint32_t	done = FALSE;
	uint64_t        tstart = 0;
	uint64_t        telapsed;

	if (ocs_eq_process_watch_yield())
		tstart = ocs_get_os_ticks();

	CPUTRACE("");

	while (!done && !sli_queue_read(&hal->sli, eq->queue, eqe)) {
		uint16_t	cq_id = 0;
		int32_t		rc;

		rc = sli_eq_parse(&hal->sli, eqe, &cq_id);
		if (unlikely(rc)) {
			if (rc > 0) {
				uint32_t i;

				/*
				 * Received a sentinel EQE indicating the EQ is full.
				 * Process all CQs
				 */
				for (i = 0; i < hal->cq_count; i++) {
					ocs_hal_cq_process(hal, hal->hal_cq[i]);
				}
				continue;
			} else {
				return rc;
			}
		} else {
			int32_t index = ocs_hal_queue_hash_find(hal->cq_hash, cq_id);
			if (likely(index >= 0)) {
				// ocs_log_err(hal->os, "Recevied CQ Event %d : %d\n", cq_id, index);
				ocs_hal_cq_process(hal, hal->hal_cq[index]);
			} else {
				ocs_log_err(hal->os, "bad CQ_ID %#06x\n", cq_id);
			}
		}

		if (eq->queue->n_posted > (eq->queue->posted_limit)) {
			sli_queue_arm(&hal->sli, eq->queue, FALSE);
		}

		if (ocs_eq_process_watch_yield()) {
			telapsed = ocs_get_os_ticks() - tstart;
			if (ocs_ticks_to_ms(telapsed) >= max_isr_time_msec)
				done = TRUE;
		}
	}

	if (hal->config.poll_mode) {
		if (eq->queue->n_posted)
			sli_queue_eq_arm(&hal->sli, eq->queue, FALSE);
	} else
		sli_queue_eq_arm(&hal->sli, eq->queue, TRUE);

	return 0;
}

static ocs_hal_rtn_e
ocs_hal_bmbx_command(ocs_hal_t *hal, uint8_t *cmd)
{
	sli4_mbox_command_header_t *hdr = NULL;
	void *bmbx = hal->sli.bmbx.virt;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	int32_t mcqe_status;

	ocs_memset(bmbx, 0, SLI4_BMBX_SIZE);
	ocs_memcpy(bmbx, cmd, SLI4_BMBX_SIZE);

	mcqe_status = sli_bmbx_command(&hal->sli);
	if (mcqe_status < 0) {
		ocs_log_err(hal->os, "BMBX submission failed\n");
		return rc;
	}

	ocs_memcpy(cmd, bmbx, SLI4_BMBX_SIZE);
	hdr = (sli4_mbox_command_header_t *)cmd;
	if (!hdr->status) {
		rc = OCS_HAL_RTN_SUCCESS;
	} else {
		ocs_log_err(hal->os, "BMBX failed; bmbx_hdr: %p, bmbx_cmd: %#x, bmbx_status: %#x\n",
				hdr, hdr->command, hdr->status);
		if (mcqe_status == SLI4_MGMT_STATUS_FEATURE_NOT_SUPPORTED) {
			rc = OCS_HAL_RTN_NOT_SUPPORTED;
		}
	}

	return rc;
}

/**
 * @brief Submit queued (pending) mbx commands.
 *
 * @par Description
 * Submit queued mailbox commands.
 * --- Assumes that hal->cmd_lock is held ---
 *
 * @param hal Hardware context.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
static int32_t
ocs_hal_cmd_submit_pending(ocs_hal_t *hal)
{
	ocs_command_ctx_t *ctx;
	int32_t rc = -1;

	/* Submit MQE only if there's room */
	while (hal->cmd_head_count < (hal->mq[0]->length - 1U)) {
		ctx = ocs_list_remove_head(&hal->cmd_pending);
		if (ctx == NULL) {
			break;
		}

		ocs_list_add_tail(&hal->cmd_head, ctx);
		hal->cmd_head_count++;

		rc = sli_queue_write(&hal->sli, hal->mq[0], ctx->buf);
		ocs_assert((rc >= 0), -1);
	}

	return 0;
}

uint8_t
ocs_hal_reset_pending(ocs_hal_t *hal)
{
	ocs_t *ocs = (ocs_t *)hal->os;
	uint8_t reset_level = OCS_RESET_LEVEL_NONE;
	uint32_t err1 = sli_reg_read(&hal->sli, SLI4_REG_SLIPORT_ERROR1);
	uint32_t err2 = sli_reg_read(&hal->sli, SLI4_REG_SLIPORT_ERROR2);

	if (err1 == 0x2 && err2 == 0x10)
		ocs_log_crit(hal->os, "FW heartbeat expired after %d seconds\n", hal->watchdog_timeout);

	if (sli_fw_error_status(&hal->sli) > 0) {
		ocs_log_crit(hal->os, "Chip is in an error state - reset needed\n");
		ocs_log_crit(hal->os, "status=%#x error1=%#x error2=%#x\n",
			sli_reg_read(&hal->sli, SLI4_REG_SLIPORT_STATUS), err1, err2);

		/* Reset the chip */
		reset_level = OCS_RESET_LEVEL_CHIP;
	} else if (ocs->fw_dump.recover_func) {
		ocs_log_crit(hal->os, "Func level dump generated! Port level reset needed\n");

		/* Reset the function */
		reset_level = OCS_RESET_LEVEL_PORT;
	}

	return reset_level;
}

/**
 * @ingroup io
 * @brief Issue a SLI command.
 *
 * @par Description
 * Send a mailbox command to the hardware, and either wait for a completion
 * (OCS_CMD_POLL) or get an optional asynchronous completion (OCS_CMD_NOWAIT).
 *
 * @param hal Hardware context.
 * @param cmd Buffer containing a formatted command and results.
 * @param opts Command options:
 *  - OCS_CMD_POLL - Command executes synchronously and busy-waits for the completion.
 *  - OCS_CMD_NOWAIT - Command executes asynchronously. Uses callback.
 * @param cb Function callback used for asynchronous mode. May be NULL.
 * @n Prototype is <tt>(*cb)(void *arg, uint8_t *cmd)</tt>.
 * @n @n @b Note: If the
 * callback function pointer is NULL, the results of the command are silently
 * discarded, allowing this pointer to exist solely on the stack.
 * @param arg Argument passed to an asynchronous callback.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_command(ocs_hal_t *hal, uint8_t *cmd, uint32_t opts, void *cb, void *arg)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;

	/* Check if the chip is in an error state (UE'd) before proceeding */
	if (ocs_hal_reset_pending(hal))
		return OCS_HAL_RTN_ERROR;

	if (OCS_CMD_POLL == opts) {
		if (hal->mq[0]->length && !sli_queue_is_empty(&hal->sli, hal->mq[0])) {
			/*
			 * Can't issue Boot-strap mailbox command with other
			 * mail-queue commands pending as this interaction is
			 * undefined
			 */
			ocs_log_err(hal->os, "MQ is not empty\n");
		} else {
			if (ocs_sem_p(&hal->bmbx_sem, OCS_SEM_FOREVER)) {
				ocs_log_err(hal->os, "ocs_sem_p failed\n");
				rc = OCS_HAL_RTN_ERROR;
				goto log_err;
			} else {
				rc = ocs_hal_bmbx_command(hal, cmd);
				ocs_sem_v(&hal->bmbx_sem);
			}
		}
	} else if (OCS_CMD_NOWAIT == opts) {
		ocs_command_ctx_t *ctx = NULL;

		ctx = ocs_malloc(hal->os, sizeof(*ctx), OCS_M_ZERO | OCS_M_NOWAIT);
		if (!ctx) {
			ocs_log_err(hal->os, "Can't allocate the command context\n");
			rc = OCS_HAL_RTN_NO_RESOURCES;
			goto log_err;
		}

		if (OCS_HAL_STATE_ACTIVE != hal->state) {
			ocs_log_err(hal->os, "HAL state (%d) is not active\n", hal->state);
			ocs_free(hal->os, ctx, sizeof(*ctx));
			goto log_err;
		}

		if (cb) {
			ctx->cb = cb;
			ctx->arg = arg;
		}

		ctx->buf = cmd;
		ctx->ctx = hal;

		ocs_lock(&hal->cmd_lock);
			/* Add to pending list */
			ocs_list_add_tail(&hal->cmd_pending, ctx);

			/* Submit as much of the pending list as we can */
			rc = ocs_hal_cmd_submit_pending(hal);
			if (rc) {
				ocs_log_err(hal->os, "MQE submission failed\n");
				ocs_free(hal->os, ctx, sizeof(*ctx));
			}
		ocs_unlock(&hal->cmd_lock);
	}

log_err:
	if (rc)
		ocs_log_err(hal->os, "MBOX command: %#x failed, rc: %d\n",
			    ((sli4_mbox_command_header_t *)cmd)->command, rc);

	return rc;
}

/**
 * @ingroup devInitShutdown
 * @brief Register a callback for the given event.
 *
 * @param hal Hardware context.
 * @param which Event of interest.
 * @param func Function to call when the event occurs.
 * @param arg Argument passed to the callback function.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_callback(ocs_hal_t *hal, ocs_hal_callback_e which, void *func, void *arg)
{

	if (!hal || !func || (which >= OCS_HAL_CB_MAX)) {
		ocs_log_err(NULL, "bad parameter hal=%p which=%#x func=%p\n", hal, which, func);
		return OCS_HAL_RTN_ERROR;
	}

	switch (which) {
	case OCS_HAL_CB_DOMAIN:
		hal->callback.domain = func;
		hal->args.domain = arg;
		break;
	case OCS_HAL_CB_PORT:
		hal->callback.port = func;
		hal->args.port = arg;
		break;
	case OCS_HAL_CB_UNSOLICITED:
		hal->callback.unsolicited = func;
		hal->args.unsolicited = arg;
		break;
	case OCS_HAL_CB_REMOTE_NODE:
		hal->callback.rnode = func;
		hal->args.rnode = arg;
		break;
	case OCS_HAL_CB_BOUNCE:
		hal->callback.bounce = func;
		hal->args.bounce = arg;
		break;
	case OCS_HAL_CB_XPORT:
		hal->callback.xport = func;
		hal->args.xport = arg;
		break;
	default:
		ocs_log_test(hal->os, "unknown callback %#x\n", which);
		return OCS_HAL_RTN_ERROR;
	}

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup port
 * @brief Allocate a port object.
 *
 * @par Description
 * This function allocates a VPI object for the port and stores it in the
 * indicator field of the port object.
 *
 * @param hal Hardware context.
 * @param sport SLI port object used to connect to the domain.
 * @param domain Domain object associated with this port (may be NULL).
 * @param wwpn Port's WWPN in big-endian order, or NULL to use default.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_port_alloc(ocs_hal_t *hal, ocs_sli_port_t *sport, ocs_domain_t *domain, uint8_t *wwpn)
{
	uint8_t	*cmd = NULL;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint32_t index;

	sport->indicator = UINT32_MAX;
	sport->hal = hal;
	sport->ctx.app = sport;
	sport->sm_free_req_pending = 0;

	/* Check if the chip is in an error state (UE'd) before proceeding */
	if (ocs_hal_reset_pending(hal))
		return OCS_HAL_RTN_ERROR;

	if (wwpn) {
		ocs_memcpy(&sport->sli_wwpn, wwpn, sizeof(sport->sli_wwpn));
	}

	if (sli_resource_alloc(&hal->sli, SLI_RSRC_FCOE_VPI, &sport->indicator, &index)) {
		ocs_log_err(hal->os, "FCOE_VPI allocation failure\n");
		return OCS_HAL_RTN_ERROR;
	}

	if (domain != NULL) {
		ocs_sm_function_t	next = NULL;

		cmd = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
		if (!cmd) {
			ocs_log_err(hal->os, "command memory allocation failed\n");
			rc = OCS_HAL_RTN_NO_MEMORY;
			goto ocs_hal_port_alloc_out;
		}

		/* If the WWPN is NULL, fetch the default WWPN and WWNN before initializing the VPI */
		if (!wwpn) {
			next = __ocs_hal_port_alloc_read_sparm64;
		} else {
			next = __ocs_hal_port_alloc_init_vpi;
		}

		ocs_sm_transition(&sport->ctx, next, cmd);
	} else if (!wwpn) {
		/* This is a convention for the HAL, not SLI */
		ocs_log_test(hal->os, "need WWPN for the physical port\n");
		rc = OCS_HAL_RTN_ERROR;
	} else {
		/* domain NULL and wwpn non-NULL */
		ocs_sm_transition(&sport->ctx, __ocs_hal_port_alloc_init, NULL);
	}

ocs_hal_port_alloc_out:
	if (rc != OCS_HAL_RTN_SUCCESS) {
		ocs_free(hal->os, cmd, SLI4_BMBX_SIZE);
		if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_VPI, sport->indicator))
			ocs_log_err(hal->os, "FCOE_VPI (%d) free failed, addr=%#x\n", sport->indicator, sport->fc_id);
	}

	return rc;
}

/**
 * @ingroup port
 * @brief Attach a physical/virtual SLI port to a domain.
 *
 * @par Description
 * This function registers a previously-allocated VPI with the
 * device.
 *
 * @param hal Hardware context.
 * @param sport Pointer to the SLI port object.
 * @param fc_id Fibre Channel ID to associate with this port.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success, or an error code on failure.
 */
ocs_hal_rtn_e
ocs_hal_port_attach(ocs_hal_t *hal, ocs_sli_port_t *sport, uint32_t fc_id)
{
	uint8_t	*buf = NULL;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	if (!hal || !sport) {
		ocs_log_err(hal ? hal->os : NULL, "bad parameter(s) hal=%p sport=%p\n", hal, sport);
		return OCS_HAL_RTN_ERROR;
	}

	/* Check if the chip is in an error state (UE'd) before proceeding */
	if (ocs_hal_reset_pending(hal))
		return OCS_HAL_RTN_ERROR;

	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (!buf) {
		ocs_log_err(hal->os, "no buffer for command\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	sport->fc_id = fc_id;
	ocs_sm_post_event(&sport->ctx, OCS_EVT_HAL_PORT_REQ_ATTACH, buf);
	return rc;
}

/**
 * @brief Called when the port control command completes.
 *
 * @par Description
 * We only need to free the mailbox command buffer.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_cb_port_control(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @ingroup port
 * @brief Control a port (initialize, shutdown, or set link configuration).
 *
 * @par Description
 * This function controls a port depending on the @c ctrl parameter:
 * - @b OCS_HAL_PORT_INIT -
 * Issues the CONFIG_LINK and INIT_LINK commands for the specified port.
 * The HAL generates an OCS_HAL_DOMAIN_FOUND event when the link comes up.
 * .
 * - @b OCS_HAL_PORT_SHUTDOWN -
 * Issues the DOWN_LINK command for the specified port.
 * The HAL generates an OCS_HAL_DOMAIN_LOST event when the link is down.
 * .
 * - @b OCS_HAL_PORT_SET_LINK_CONFIG -
 * Sets the link configuration.
 *
 * @param hal Hardware context.
 * @param ctrl Specifies the operation:
 * - OCS_HAL_PORT_INIT
 * - OCS_HAL_PORT_SHUTDOWN
 * - OCS_HAL_PORT_SET_LINK_CONFIG
 *
 * @param value Operation-specific value.
 * - OCS_HAL_PORT_INIT - Selective reset AL_PA
 * - OCS_HAL_PORT_SHUTDOWN - N/A
 * - OCS_HAL_PORT_SET_LINK_CONFIG - An enum #ocs_hal_linkcfg_e value.
 *
 * @param cb Callback function to invoke the following operation.
 * - OCS_HAL_PORT_INIT/OCS_HAL_PORT_SHUTDOWN - NULL (link events
 * are handled by the OCS_HAL_CB_DOMAIN callbacks).
 * - OCS_HAL_PORT_SET_LINK_CONFIG - Invoked after linkcfg mailbox command
 * completes.
 *
 * @param arg Callback argument invoked after the command completes.
 * - OCS_HAL_PORT_INIT/OCS_HAL_PORT_SHUTDOWN - NULL (link events
 * are handled by the OCS_HAL_CB_DOMAIN callbacks).
 * - OCS_HAL_PORT_SET_LINK_CONFIG - Invoked after linkcfg mailbox command
 * completes.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_port_control(ocs_hal_t *hal, ocs_hal_port_e ctrl, uintptr_t value, ocs_hal_port_control_cb_t cb, void *arg)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;

	switch (ctrl) {
	case OCS_HAL_PORT_INIT:
	{
		uint8_t	*init_link;
		uint32_t speed = 0;
		uint8_t reset_alpa = 0;

		if (SLI_LINK_MEDIUM_FC == sli_get_medium(&hal->sli)) {
			uint8_t	*cfg_link;

			cfg_link = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
			if (cfg_link == NULL) {
				ocs_log_err(hal->os, "no buffer for command\n");
				return OCS_HAL_RTN_NO_MEMORY;
			}

			sli_cmd_config_link(&hal->sli, cfg_link, SLI4_BMBX_SIZE);
			rc = ocs_hal_command(hal, cfg_link, OCS_CMD_NOWAIT, ocs_hal_cb_port_control, NULL);
			if (rc) {
				ocs_free(hal->os, cfg_link, SLI4_BMBX_SIZE);
				break;
			}

			speed = hal->config.speed;
			reset_alpa = (uint8_t)(value & 0xff);
		} else {
			speed = FC_LINK_SPEED_10G;
		}

		/* Allocate a new buffer for the init_link command */
		init_link = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
		if (init_link == NULL) {
			ocs_log_err(hal->os, "no buffer for command\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}

		/*
		 * Bring link up, unless FW version is not supported
		 */
		if (hal->workaround.fw_version_too_low) {
			if ((SLI4_IF_TYPE_LANCER_FC_ETH == hal->sli.if_type) ||
			    (SLI4_IF_TYPE_LANCER_G7 == hal->sli.if_type)) {
				ocs_log_err(hal->os, "Cannot bring up link.  Please update firmware to %s or later (current version is %s)\n",
					OCS_FW_VER_STR(OCS_MIN_FW_VER_LANCER), (char *) sli_get_fw_name(&hal->sli,0));
			} else {
				ocs_log_err(hal->os, "Cannot bring up link.  Please update firmware to %s or later (current version is %s)\n",
					OCS_FW_VER_STR(OCS_MIN_FW_VER_SKYHAWK), (char *) sli_get_fw_name(&hal->sli, 0));
			}

			ocs_free(hal->os, init_link, SLI4_BMBX_SIZE);
			return OCS_HAL_RTN_ERROR;
		}

		rc = OCS_HAL_RTN_ERROR;
		if (sli_cmd_init_link(&hal->sli, init_link, SLI4_BMBX_SIZE, speed, reset_alpa)) {
			rc = ocs_hal_command(hal, init_link, OCS_CMD_NOWAIT, ocs_hal_cb_port_control, NULL);
		}

		/* Free buffer on error, since no callback is coming */
		if (rc) {
			ocs_free(hal->os, init_link, SLI4_BMBX_SIZE);
			ocs_log_err(hal->os, "INIT_LINK failed\n");
		}

		break;
	}
	case OCS_HAL_PORT_SHUTDOWN:
	{
		uint8_t	*down_link;

		down_link = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
		if (down_link == NULL) {
			ocs_log_err(hal->os, "no buffer for command\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}

		sli_cmd_down_link(&hal->sli, down_link, SLI4_BMBX_SIZE);
		rc = ocs_hal_command(hal, down_link, OCS_CMD_NOWAIT, ocs_hal_cb_port_control, NULL);
		/* Free buffer on error, since no callback is coming */
		if (rc)
			ocs_free(hal->os, down_link, SLI4_BMBX_SIZE);

		break;
	}
	case OCS_HAL_PORT_SET_LINK_CONFIG:
		rc = ocs_hal_set_linkcfg(hal, (ocs_hal_linkcfg_e)value, OCS_CMD_NOWAIT, cb, arg);
		break;
	default:
		ocs_log_test(hal->os, "unhandled control %#x\n", ctrl);
		break;
	}

	return rc;
}


/**
 * @ingroup port
 * @brief Free port resources.
 *
 * @par Description
 * Issue the UNREG_VPI command to free the assigned VPI context.
 *
 * @param hal Hardware context.
 * @param sport SLI port object used to connect to the domain.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_port_free(ocs_hal_t *hal, ocs_sli_port_t *sport)
{
	ocs_hal_rtn_e	rc = OCS_HAL_RTN_SUCCESS;

	if (!hal || !sport) {
		ocs_log_err(hal ? hal->os : NULL, "bad parameter(s) hal=%p sport=%p\n", hal, sport);
		return OCS_HAL_RTN_ERROR;
	}

	ocs_sm_post_event(&sport->ctx, OCS_EVT_HAL_PORT_REQ_FREE, NULL);
	return rc;
}

/**
 * @ingroup domain
 * @brief Allocate a fabric domain object.
 *
 * @par Description
 * This function starts a series of commands needed to connect to the domain, including
 *   - REG_FCFI
 *   - INIT_VFI
 *   - READ_SPARMS
 *   .
 * @b Note: Not all SLI interface types use all of the above commands.
 * @n @n Upon successful allocation, the HAL generates a OCS_HAL_DOMAIN_ALLOC_OK
 * event. On failure, it generates a OCS_HAL_DOMAIN_ALLOC_FAIL event.
 *
 * @param hal Hardware context.
 * @param domain Pointer to the domain object.
 * @param fcf FCF index.
 * @param vlan VLAN ID.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_domain_alloc(ocs_hal_t *hal, ocs_domain_t *domain, uint32_t fcf, uint32_t vlan)
{
	uint8_t		*cmd = NULL;
	uint32_t	index;

	if (!hal || !domain || !domain->sport) {
		ocs_log_err(NULL, "bad parameter(s) hal=%p domain=%p sport=%p\n",
				hal, domain, domain ? domain->sport : NULL);
		return OCS_HAL_RTN_ERROR;
	}

	/* Check if the chip is in an error state (UE'd) before proceeding */
	if (ocs_hal_reset_pending(hal))
		return OCS_HAL_RTN_ERROR;

	cmd = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (!cmd) {
		ocs_log_err(hal->os, "command memory allocation failed\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	// allocate memory for the service parameters
	if (ocs_dma_alloc(hal->os, &domain->dma, 112, 4)) {
		ocs_log_err(hal->os, "Failed to allocate DMA memory\n");
		ocs_free(hal->os, cmd, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	domain->hal = hal;
	domain->sm.app = domain;
	domain->fcf = fcf;
	domain->fcf_indicator = UINT32_MAX;
	domain->vlan_id = vlan;
	domain->indicator = UINT32_MAX;

	if (sli_resource_alloc(&hal->sli, SLI_RSRC_FCOE_VFI, &domain->indicator, &index)) {
		ocs_log_err(hal->os, "FCOE_VFI allocation failure\n");

		ocs_free(hal->os, cmd, SLI4_BMBX_SIZE);
		ocs_dma_free(hal->os, &domain->dma);

		return OCS_HAL_RTN_ERROR;
	}

	ocs_sm_transition(&domain->sm, __ocs_hal_domain_init, cmd);
	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup domain
 * @brief Attach a SLI port to a domain.
 *
 * @param hal Hardware context.
 * @param domain Pointer to the domain object.
 * @param fc_id Fibre Channel ID to associate with this port.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_domain_attach(ocs_hal_t *hal, ocs_domain_t *domain, uint32_t fc_id)
{
	uint8_t	*buf = NULL;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	if (!hal || !domain) {
		ocs_log_err(hal ? hal->os : NULL, "bad parameter(s) hal=%p domain=%p\n", hal, domain);
		return OCS_HAL_RTN_ERROR;
	}

	/* Check if the chip is in an error state (UE'd) before proceeding */
	if (ocs_hal_reset_pending(hal))
		return OCS_HAL_RTN_ERROR;

	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (!buf) {
		ocs_log_err(hal->os, "no buffer for command\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	domain->sport->fc_id = fc_id;
	ocs_sm_post_event(&domain->sm, OCS_EVT_HAL_DOMAIN_REQ_ATTACH, buf);
	return rc;
}

/**
 * @ingroup domain
 * @brief Free a fabric domain object.
 *
 * @par Description
 * Free both the driver and SLI port resources associated with the domain.
 *
 * @param hal Hardware context.
 * @param domain Pointer to the domain object.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_domain_free(ocs_hal_t *hal, ocs_domain_t *domain)
{
	ocs_hal_rtn_e	rc = OCS_HAL_RTN_SUCCESS;

	if (!hal || !domain) {
		ocs_log_err(hal ? hal->os : NULL, "bad parameter(s) hal=%p domain=%p\n", hal, domain);
		return OCS_HAL_RTN_ERROR;
	}

	ocs_sm_post_event(&domain->sm, OCS_EVT_HAL_DOMAIN_REQ_FREE, NULL);
	return rc;
}

/**
 * @ingroup domain
 * @brief Free a fabric domain object.
 *
 * @par Description
 * Free the driver resources associated with the domain. The difference between
 * this call and ocs_hal_domain_free() is that this call assumes resources no longer
 * exist on the SLI port, due to a reset or after some error conditions.
 *
 * @param hal Hardware context.
 * @param domain Pointer to the domain object.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_domain_force_free(ocs_hal_t *hal, ocs_domain_t *domain)
{
	if (!hal || !domain) {
		ocs_log_err(NULL, "bad parameter(s) hal=%p domain=%p\n", hal, domain);
		return OCS_HAL_RTN_ERROR;
	}

	ocs_dma_free(hal->os, &domain->dma);
	if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_VFI, domain->indicator))
		ocs_log_err(hal->os, "FCOE_VFI (%d) free failed\n", domain->indicator);

	hal->domains[domain->fcf_indicator] = NULL;
	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup node
 * @brief Allocate a remote node object.
 *
 * @param hal Hardware context.
 * @param rnode Allocated remote node object to initialize.
 * @param fc_addr FC address of the remote node.
 * @param sport SLI port used to connect to remote node.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_node_alloc(ocs_hal_t *hal, ocs_remote_node_t *rnode, uint32_t fc_addr,
		ocs_sli_port_t *sport)
{
	/* Check for invalid indicator */
	if (UINT32_MAX != rnode->indicator) {
		ocs_log_err(hal->os, "FCOE_RPI allocation failure addr=%#x rpi=%#x\n",
				fc_addr, rnode->indicator);
		return OCS_HAL_RTN_ERROR;
	}

	/* Check if the chip is in an error state (UE'd) before proceeding */
	if (ocs_hal_reset_pending(hal))
		return OCS_HAL_RTN_ERROR;

	// NULL SLI port indicates an unallocated remote node
	rnode->sport = NULL;

	if (sli_resource_alloc(&hal->sli, SLI_RSRC_FCOE_RPI, &rnode->indicator, &rnode->index)) {
		ocs_log_err(hal->os, "FCOE_RPI allocation failure addr=%#x\n",
				fc_addr);
		return OCS_HAL_RTN_ERROR;
	}

	rnode->fc_id = fc_addr;
	rnode->sport = sport;

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup node
 * @brief Update a remote node object with the remote port's service parameters.
 *
 * @param hal Hardware context.
 * @param rnode Allocated remote node object to initialize.
 * @param sparms DMA buffer containing the remote port's service parameters.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_node_attach(ocs_hal_t *hal, ocs_remote_node_t *rnode, ocs_dma_t *sparms)
{
	ocs_hal_rtn_e	rc = OCS_HAL_RTN_ERROR;
	uint8_t		*buf = NULL;
	uint8_t		hlm = FALSE;
	uint32_t	count = 0;

	if (!hal || !rnode || !sparms) {
		ocs_log_err(NULL, "bad parameter(s) hal=%p rnode=%p sparms=%p\n", hal, rnode, sparms);
		return OCS_HAL_RTN_ERROR;
	}

	/* Check if the chip is in an error state (UE'd) before proceeding */
	if (ocs_hal_reset_pending(hal))
		return OCS_HAL_RTN_ERROR;

	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (!buf) {
		ocs_log_err(hal->os, "no buffer for command\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/*
	 * If the attach count is non-zero, this RPI has already been registered.
	 * Otherwise, register the RPI
	 */
	if (rnode->index == UINT32_MAX) {
		ocs_log_err(NULL, "bad parameter rnode->index invalid\n");
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_ERROR;
	}
	count = ocs_atomic_add_return(&hal->rpi_ref[rnode->index].rpi_count, 1);
	if (count) {
		/*
		 * Can't attach multiple FC_ID's to a node unless High Login
		 * Mode is enabled
		 */
		if (sli_get_hlm(&hal->sli) == FALSE) {
			ocs_log_test(hal->os, "attach to already attached node HLM=%d count=%d\n",
				     sli_get_hlm(&hal->sli), count);
			rc = OCS_HAL_RTN_SUCCESS;
		} else {
			rnode->node_group = TRUE;
			rnode->attached = ocs_atomic_read(&hal->rpi_ref[rnode->index].rpi_attached);
			rc = rnode->attached  ? OCS_HAL_RTN_SUCCESS_SYNC : OCS_HAL_RTN_SUCCESS;
		}
	} else {
		rnode->node_group = FALSE;
		hlm = sli_get_hlm(&hal->sli);

		ocs_display_sparams("", "reg rpi", 0, NULL, sparms->virt);
		sli_cmd_reg_rpi(&hal->sli, buf, SLI4_BMBX_SIZE, rnode->fc_id,
				rnode->indicator, rnode->sport->indicator, sparms, false,
				(hal->tow_enabled && hal->config.tow_t10_enable), hlm, 0);
		rc = ocs_hal_command(hal, buf, OCS_CMD_NOWAIT, ocs_hal_cb_node_attach, rnode);
	}

	if (count || rc) {
		if (rc < OCS_HAL_RTN_SUCCESS) {
			ocs_atomic_sub_return(&hal->rpi_ref[rnode->index].rpi_count, 1);
			ocs_log_err(hal->os, "%s error\n", count ? "HLM" : "REG_RPI");
		}
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
	}

	return rc;
}

static int32_t
ocs_hal_cb_reg_rpi_update(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	ocs_remote_node_t *rnode = arg;
	sli4_mbox_command_header_t *hdr = (sli4_mbox_command_header_t *)mqe;
	ocs_sm_event_t evt = OCS_EVT_LAST;

	if (status || hdr->status)
		evt = OCS_EVT_NODE_RPI_UPDATE_FAIL;
	else
		evt = OCS_EVT_NODE_RPI_UPDATE_OK;

	if (rnode)
		ocs_node_post_event((ocs_node_t *)rnode->node, evt, NULL);
	else
		ocs_log_err(hal->os, "reg_rpi_update cb is not handled\n");

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @ingroup node
 * @brief Update REG_RPI with new parameters
 *
 * @param hal Hardware context.
 * @param rnode Remote node object to free.
 * @param nsler NVMe SLER capability to update
 *
 * @return Return 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_reg_rpi_update(ocs_hal_t *hal, ocs_remote_node_t *rnode, int32_t nsler)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	uint8_t *buf = NULL;

	if (!hal || !rnode || !rnode->attached) {
		ocs_log_err(NULL, "bad parameter(s) hal=%p rnode=%p rnode attached=%d\n",
			    hal, rnode, (rnode) ? rnode->attached : 0);
		return rc;
	}

	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_NOWAIT);
	if (!buf) {
		ocs_log_err(hal->os, "Failed to alloc mbox memory\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	sli_cmd_reg_rpi(&hal->sli, buf, SLI4_BMBX_SIZE, 0,
			rnode->indicator, 0, NULL, true,
			0, 0, nsler);
	rc = ocs_hal_command(hal, buf, OCS_CMD_NOWAIT, ocs_hal_cb_reg_rpi_update, rnode);
	if (rc) {
		ocs_log_err(hal->os, "Failed to submit reg_rpi update\n");
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
	}

	return rc;
}

/**
 * @ingroup node
 * @brief Free a remote node resource.
 *
 * @param hal Hardware context.
 * @param rnode Remote node object to free.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_node_free_resources(ocs_hal_t *hal, ocs_remote_node_t *rnode)
{
	ocs_hal_rtn_e	rc = OCS_HAL_RTN_SUCCESS;

	if (!hal || !rnode) {
		ocs_log_err(NULL, "bad parameter(s) hal=%p rnode=%p\n", hal, rnode);
		return OCS_HAL_RTN_ERROR;
	}

	if (rnode->sport) {
		if (!rnode->attached) {
			if (rnode->indicator != UINT32_MAX) {
				if (ocs_atomic_read(&hal->rpi_ref[rnode->index].rpi_count) > 0)
					return rc;

				if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_RPI, rnode->indicator)) {
					ocs_log_err(hal->os, "FCOE_RPI (%d) free failed, addr=%#x\n",
						    rnode->indicator, rnode->fc_id);
					rc = OCS_HAL_RTN_ERROR;
				} else {
					rnode->node_group = FALSE;
					rnode->indicator = UINT32_MAX;
					rnode->index = UINT32_MAX;
					rnode->free_group = FALSE;
				}
			}
		} else {
			ocs_log_err(hal->os, "Error: rnode is still attached\n");
			rc = OCS_HAL_RTN_ERROR;
		}
	}

	return rc;
}

/**
 * @ingroup node
 * @brief Free a remote node object.
 *
 * @param hal Hardware context.
 * @param rnode Remote node object to free.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_node_detach(ocs_hal_t *hal, ocs_remote_node_t *rnode)
{
	uint8_t	*buf = NULL;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS_SYNC;
	uint32_t index = UINT32_MAX;
	uint32_t indicator;
	ocs_node_t *node;

	if (!hal || !rnode) {
		ocs_log_err(NULL, "bad parameter(s) hal=%p rnode=%p\n", hal, rnode);
		return OCS_HAL_RTN_ERROR;
	}

	index = rnode->index;

	node = (ocs_node_t *)rnode->node;
	if (!node) {
		ocs_log_err(hal->os, "bad parameter node=%p\n", node);
		return OCS_HAL_RTN_ERROR;
	}

	if (rnode->sport) {
		uint32_t count = 0;
		uint32_t fc_id;

		if (!rnode->attached)
			return OCS_HAL_RTN_SUCCESS_SYNC;

		buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
		if (!buf) {
			ocs_log_err(hal->os, "no buffer for command\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}

		indicator = rnode->indicator;
		count = ocs_atomic_sub_return(&hal->rpi_ref[index].rpi_count, 1);

		if (count <= 1) {
			/* There are no other references to this RPI; so unregister it and free the resource */
			fc_id = UINT32_MAX;
			rnode->node_group = FALSE;
			rnode->free_group = TRUE;

			ocs_atomic_set(&hal->rpi_ref[index].rpi_attached, 0);
		} else {
			if (sli_get_hlm(&hal->sli) == FALSE)
				ocs_log_err(hal->os, "Invalid count with HLM disabled, count=%d\n", count);

			fc_id = rnode->fc_id & 0x00ffffff;
			rnode->free_group = FALSE;

			/*
			 * In HLM, RPI will be free'd only when the last initiator's unreg_rpi gets completed.
			 * Mark the other rnode indicators as invalid, except the last rnode's indicator.
			 */
			rnode->indicator = UINT32_MAX;
		}

		if (!node->unreg_rpi && !node->sport->unreg_rpi_all) {
			node->unreg_rpi = TRUE;
			rc = OCS_HAL_RTN_SUCCESS;

			if (sli_cmd_unreg_rpi(&hal->sli, buf, SLI4_BMBX_SIZE, indicator,
					      SLI_RSRC_FCOE_RPI, fc_id)) {
				if (hal->state != OCS_HAL_STATE_ACTIVE)
					ocs_hal_cb_node_free(hal, OCS_HAL_RTN_SUCCESS, buf, rnode);
				else
					rc = ocs_hal_command(hal, buf, OCS_CMD_NOWAIT, ocs_hal_cb_node_free, rnode);
			}

			if (rc)
				ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
		} else {
			/* We already called unreg_rpi_all, skipping unreg_rpi */
			ocs_log_info(hal->os, "UNREG_RPI skipped\n");
			ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
			rc = OCS_HAL_RTN_SUCCESS;
		}
	}

	return rc;
}

/**
 * @ingroup node
 * @brief Free all remote node objects.
 *
 * @param hal Hardware context.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_unreg_rpi_all(ocs_hal_t *hal, ocs_sport_t *sport)
{
	uint8_t	*buf = NULL;
	ocs_hal_rtn_e	rc = OCS_HAL_RTN_SUCCESS;

	if (!hal) {
		ocs_log_err(NULL, "bad parameter hal=%p\n", hal);
		return OCS_HAL_RTN_ERROR;
	}

	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (!buf) {
		ocs_log_err(hal->os, "no buffer for command\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	if (sli_cmd_unreg_rpi(&hal->sli, buf, SLI4_BMBX_SIZE, sport->indicator,
				SLI_RSRC_FCOE_VPI, UINT32_MAX)) {
		if (hal->state != OCS_HAL_STATE_ACTIVE)
			ocs_hal_cb_unreg_rpi_all(hal, OCS_HAL_RTN_SUCCESS, buf, sport);
		else
			rc = ocs_hal_command(hal, buf, OCS_CMD_NOWAIT, ocs_hal_cb_unreg_rpi_all, sport);
	}

	if (rc)
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);

	return rc;
}

ocs_hal_rtn_e
ocs_hal_node_group_alloc(ocs_hal_t *hal, ocs_remote_node_group_t *ngroup)
{
	if (!hal || !ngroup) {
		ocs_log_err(NULL, "bad parameter hal=%p ngroup=%p\n", hal, ngroup);
		return OCS_HAL_RTN_ERROR;
	}

	if (sli_resource_alloc(&hal->sli, SLI_RSRC_FCOE_RPI, &ngroup->indicator, &ngroup->index)) {
		ocs_log_err(hal->os, "FCOE_RPI allocation failure addr=%#x\n", ngroup->indicator);
		return OCS_HAL_RTN_ERROR;
	}

	return OCS_HAL_RTN_SUCCESS;
}

ocs_hal_rtn_e
ocs_hal_node_group_attach(ocs_hal_t *hal, ocs_remote_node_group_t *ngroup, ocs_remote_node_t *rnode)
{
	if (!hal || !ngroup || !rnode) {
		ocs_log_err(NULL, "bad parameter hal=%p ngroup=%p rnode=%p\n", hal, ngroup, rnode);
		return OCS_HAL_RTN_ERROR;
	}

	if (rnode->attached) {
		ocs_log_err(hal->os, "node already attached RPI=%#x addr=%#x\n", rnode->indicator, rnode->fc_id);
		return OCS_HAL_RTN_ERROR;
	}

	if ((rnode->indicator != UINT32_MAX) && (ocs_atomic_read(&hal->rpi_ref[rnode->index].rpi_count) == 0)) {
		if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_RPI, rnode->indicator)) {
			ocs_log_err(hal->os, "FCOE_RPI (%d) free failed, addr=%#x\n", rnode->indicator, rnode->fc_id);
			return OCS_HAL_RTN_ERROR;
		}
	}

	rnode->indicator = ngroup->indicator;
	rnode->index = ngroup->index;

	return OCS_HAL_RTN_SUCCESS;
}

ocs_hal_rtn_e
ocs_hal_node_group_free(ocs_hal_t *hal, ocs_remote_node_group_t *ngroup)
{
	if (!hal || !ngroup) {
		ocs_log_err(NULL, "bad parameter hal=%p ngroup=%p\n", hal, ngroup);
		return OCS_HAL_RTN_ERROR;
	}

	ocs_assert(ocs_atomic_read(&hal->rpi_ref[ngroup->index].rpi_count) == 0, -1);

	if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_RPI, ngroup->indicator)) {
		ocs_log_err(hal->os, "FCOE_RPI (%d) free failed\n", ngroup->indicator);
		return OCS_HAL_RTN_ERROR;
	}

	ngroup->indicator = UINT32_MAX;
	ngroup->index = UINT32_MAX;

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @brief Initialize IO fields on each free call.
 *
 * @n @b Note: This is done on each free call (as opposed to each
 * alloc call) because port-owned XRIs are not
 * allocated with ocs_hal_io_alloc() but are freed with this
 * function.
 *
 * @param io Pointer to HAL IO.
 */
static inline void
ocs_hal_init_free_io(ocs_hal_io_t *io)
{
	/*
	 * Set io->done to NULL, to avoid any callbacks, should
	 * a completion be received for one of these IOs
	 */
	io->done = NULL;
	io->abort_done = NULL;
	io->abort_issued = FALSE;
	io->port_owned_abort_count = 0;
	io->rnode = NULL;
	io->type = 0xFFFF;
	io->wq = NULL;
	io->ul_io = NULL;
	io->tgt_wqe_timeout = 0;
}

/**
 * @ingroup io
 * @brief Lockless allocate a HAL IO object.
 *
 * @par Description
 * Assume that hal->ocs_lock is held. This function is only used if
 * use_dif_sec_xri workaround is being used.
 *
 * @param hal Hardware context.
 *
 * @return Returns a pointer to an object on success, or NULL on failure.
 */
static inline ocs_hal_io_t *
_ocs_hal_io_alloc(ocs_hal_t *hal)
{
	ocs_hal_io_t	*io = NULL;

	if (NULL != (io = ocs_list_remove_head(&hal->io_free))) {
		ocs_list_add_tail(&hal->io_inuse, io);
		io->state = OCS_HAL_IO_STATE_INUSE;
		io->quarantine = FALSE;
		io->quarantine_first_phase = TRUE;
		io->abort_reqtag = UINT32_MAX;
		ocs_ref_init(&io->ref, ocs_hal_io_free_internal, io);
	} else {
		ocs_atomic_add_return(&hal->io_alloc_failed_count, 1);
	}

	return io;
}
/**
 * @ingroup io
 * @brief Allocate a HAL IO object.
 *
 * @par Description
 * @n @b Note: This function applies to non-port owned XRIs
 * only.
 *
 * @param hal Hardware context.
 *
 * @return Returns a pointer to an object on success, or NULL on failure.
 */
ocs_hal_io_t *
ocs_hal_io_alloc(ocs_hal_t *hal)
{
	ocs_hal_io_t	*io = NULL;

	ocs_lock(&hal->io_lock);
		io = _ocs_hal_io_alloc(hal);
	ocs_unlock(&hal->io_lock);

	return io;
}

/**
 * @ingroup io
 * @brief Allocate/Activate a port owned HAL IO object.
 *
 * @par Description
 * This function is called by the transport layer when an XRI is
 * allocated by the SLI-Port. This will "activate" the HAL IO
 * associated with the XRI received from the SLI-Port to mirror
 * the state of the XRI.
 * @n @n @b Note: This function applies to port owned XRIs only.
 *
 * @param hal Hardware context.
 * @param io Pointer HAL IO to activate/allocate.
 *
 * @return Returns a pointer to an object on success, or NULL on failure.
 */
ocs_hal_io_t *
ocs_hal_io_activate_port_owned(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	if (ocs_ref_read_count(&io->ref) > 0) {
		ocs_log_err(hal->os, "Bad parameter: refcount > 0\n");
		return NULL;
	}

	if (io->wq != NULL) {
		ocs_log_err(hal->os, "xri=%#x already in use\n", io->indicator);
		return NULL;
	}

	ocs_ref_init(&io->ref, ocs_hal_io_free_port_owned, io);
	io->xbusy = TRUE;

	return io;
}

/**
 * @ingroup io
 * @brief When an IO is freed, depending on the exchange busy flag, and other
 * workarounds, move it to the correct list.
 *
 * @par Description
 * @n @b Note: Assumes that the hal->io_lock is held and the item has been removed
 * from the busy or wait_free list.
 *
 * @param hal Hardware context.
 * @param io Pointer to the IO object to move.
 */
static void
ocs_hal_io_free_move_correct_list(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	if (io->xbusy) {
		/* add to wait_free list and wait for XRI_ABORTED CQEs to clean up */
		ocs_list_add_tail(&hal->io_wait_free, io);
		io->state = OCS_HAL_IO_STATE_WAIT_FREE;
	} else {
		/* IO not busy, add to free list */
		ocs_list_add_tail(&hal->io_free, io);
		io->state = OCS_HAL_IO_STATE_FREE;
	}

	/* BZ 161832 workaround */
	if (hal->workaround.use_dif_sec_xri) {
		ocs_hal_check_sec_hio_list(hal);
	}
}

/**
 * @ingroup io
 * @brief Free a HAL IO object. Perform cleanup common to
 * port and host-owned IOs.
 *
 * @param hal Hardware context.
 * @param io Pointer to the HAL IO object.
 */
static inline void
ocs_hal_io_free_common(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	/* initialize IO fields */
	ocs_hal_init_free_io(io);

	/* Restore default SGL */
	ocs_hal_io_restore_sgl(hal, io);
}

/**
 * @ingroup io
 * @brief Free a HAL IO object associated with a port-owned XRI.
 *
 * @param arg Pointer to the HAL IO object.
 */
static void
ocs_hal_io_free_port_owned(void *arg)
{
	ocs_hal_io_t *io = (ocs_hal_io_t *)arg;
	ocs_hal_t *hal = io->hal;

	/*
	 * If the dnrx bit is set, then add it to the list of XRIs waiting for buffers.
	 */
	if (io->tow_dnrx) {
		ocs_lock(&hal->io_lock);
			/* take a reference count because we still own the IO until the buffer is posted */
			ocs_ref_init(&io->ref, ocs_hal_io_free_port_owned, io);
			ocs_list_add_tail(&hal->io_port_dnrx, io);
		ocs_unlock(&hal->io_lock);
	}

	/* perform common cleanup */
	ocs_hal_io_free_common(hal, io);
}

/**
 * @ingroup io
 * @brief Free a previously-allocated HAL IO object. Called when
 * IO refcount goes to zero (host-owned IOs only).
 *
 * @param arg Pointer to the HAL IO object.
 */
static void
ocs_hal_io_free_internal(void *arg)
{
	ocs_hal_io_t *io = (ocs_hal_io_t *)arg;
	ocs_hal_t *hal = io->hal;

	/* perform common cleanup */
	ocs_hal_io_free_common(hal, io);

	ocs_lock(&hal->io_lock);
		/* remove from in-use list */
		ocs_list_remove(&hal->io_inuse, io);
		ocs_hal_io_free_move_correct_list(hal, io);
	ocs_unlock(&hal->io_lock);
}

/**
 * @ingroup io
 * @brief Free a previously-allocated HAL IO object.
 *
 * @par Description
 * @n @b Note: This function applies to port and host owned XRIs.
 *
 * @param hal Hardware context.
 * @param io Pointer to the HAL IO object.
 *
 * @return Returns a non-zero value if HAL IO was freed, 0 if references
 * on the IO still exist, or a negative value if an error occurred.
 */
int32_t
ocs_hal_io_free(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	/* just put refcount */
	if (ocs_ref_read_count(&io->ref) <= 0) {
		ocs_log_err(hal->os, "Bad parameter: refcount <= 0 io=%p xri=%#x tag=%#x\n",
			    io, io->indicator, io->reqtag);
		ocs_hal_assert(0);
		return -1;
	}

	return ocs_ref_put(&io->ref); /* ocs_ref_get(): ocs_hal_io_alloc() */
}

/**
 * @ingroup io
 * @brief Check if given HAL IO is in-use
 *
 * @par Description
 * This function returns TRUE if the given HAL IO has been
 * allocated and is in-use, and FALSE otherwise. It applies to
 * port and host owned XRIs.
 *
 * @param hal Hardware context.
 * @param io Pointer to the HAL IO object.
 *
 * @return TRUE if an IO is in use, or FALSE otherwise.
 */
uint8_t
ocs_hal_io_inuse(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	return (ocs_ref_read_count(&io->ref) > 0);
}

/**
 * @brief Write a HAL IO to a work queue.
 *
 * @par Description
 * A HAL IO is written to a work queue.
 *
 * @param wq Pointer to work queue.
 * @param wqe Pointer to WQ entry.
 *
 * @n @b Note: Assumes the SLI-4 queue lock is held.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
static int32_t
_hal_wq_write(hal_wq_t *wq, ocs_hal_wqe_t *wqe)
{
	int32_t rc;
	int32_t queue_rc;

	/* Every so often, set the wqec bit to generate comsummed completions */
	if (wq->wqec_count) {
		wq->wqec_count--;
	}
	if (wq->wqec_count == 0) {
		sli4_generic_wqe_t *genwqe = (void*)wqe->wqebuf;
		genwqe->wqec = 1;
		wq->wqec_count = wq->wqec_set_count;
	}

	/* Decrement WQ free count */
	wq->free_count--;

	queue_rc = _sli_queue_write(&wq->hal->sli, wq->queue, wqe->wqebuf);

	if (queue_rc < 0) {
		rc = -1;
	} else {
		rc = 0;
		ocs_queue_history_wq(&wq->hal->q_hist, (void *) wqe->wqebuf, wq->queue->id, queue_rc);
	}

	return rc;
}

/**
 * @brief Write a HAL IO to a work queue.
 *
 * @par Description
 * A HAL IO is written to a work queue.
 *
 * @param wq Pointer to work queue.
 * @param wqe Pointer to WQE entry.
 *
 * @n @b Note: Takes the SLI-4 queue lock.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
int32_t
hal_wq_write(hal_wq_t *wq, ocs_hal_wqe_t *wqe)
{
	int32_t rc = 0;

	sli_queue_lock(wq->queue);
		if ( ! ocs_list_empty(&wq->pending_list)) {
			ocs_list_add_tail(&wq->pending_list, wqe);
			OCS_STAT(wq->wq_pending_count++;)
			while ((wq->free_count > 0) && ((wqe = ocs_list_remove_head(&wq->pending_list)) != NULL)) {
				rc = _hal_wq_write(wq, wqe);
				if (rc < 0) {
					break;
				}
				if (wqe->abort_wqe_submit_needed) {
					wqe->abort_wqe_submit_needed = 0;
					sli_abort_wqe(&wq->hal->sli, wqe->wqebuf, wq->hal->sli.config.wqe_size, SLI_ABORT_XRI,
							wqe->send_abts, wqe->id, 0, wqe->abort_reqtag, SLI4_CQ_DEFAULT );
					ocs_list_add_tail(&wq->pending_list, wqe);
					OCS_STAT(wq->wq_pending_count++;)
				}
			}
		} else {
			if (wq->free_count > 0) {
				rc = _hal_wq_write(wq, wqe);
			} else {
				ocs_list_add_tail(&wq->pending_list, wqe);
				OCS_STAT(wq->wq_pending_count++;)
			}
		}

	sli_queue_unlock(wq->queue);

	return rc;

}

/**
 * @brief Update free count and submit any pending HAL IOs
 *
 * @par Description
 * The WQ free count is updated, and any pending HAL IOs are submitted that
 * will fit in the queue.
 *
 * @param wq Pointer to work queue.
 * @param update_free_count Value added to WQs free count.
 *
 * @return None.
 */
static void
hal_wq_submit_pending(hal_wq_t *wq, uint32_t update_free_count)
{
	ocs_hal_wqe_t *wqe;

	sli_queue_lock(wq->queue);

		/* Update free count with value passed in */
		wq->free_count += update_free_count;

		while ((wq->free_count > 0) && ((wqe = ocs_list_remove_head(&wq->pending_list)) != NULL)) {
			_hal_wq_write(wq, wqe);

			if (wqe->abort_wqe_submit_needed) {
				wqe->abort_wqe_submit_needed = 0;
				sli_abort_wqe(&wq->hal->sli, wqe->wqebuf, wq->hal->sli.config.wqe_size, SLI_ABORT_XRI,
						wqe->send_abts, wqe->id, 0, wqe->abort_reqtag, SLI4_CQ_DEFAULT);
				ocs_list_add_tail(&wq->pending_list, wqe);
				OCS_STAT(wq->wq_pending_count++;)
			}
		}

	sli_queue_unlock(wq->queue);
}

/**
 * @brief Check to see if there are any BZ 161832 workaround waiting IOs
 *
 * @par Description
 * Checks hal->sec_hio_wait_list, if an IO is waiting for a HAL IO, then try
 * to allocate a secondary HAL io, and dispatch it.
 *
 * @n @b Note: hal->io_lock MUST be taken when called.
 *
 * @param hal pointer to HAL object
 *
 * @return none
 */
static void
ocs_hal_check_sec_hio_list(ocs_hal_t *hal)
{
	ocs_hal_io_t *io;
	ocs_hal_io_t *sec_io;
	int rc = 0;

	while (!ocs_list_empty(&hal->sec_hio_wait_list)) {
		uint16_t flags;

		sec_io = _ocs_hal_io_alloc(hal);
		if (sec_io == NULL) {
			break;
		}

		io = ocs_list_remove_head(&hal->sec_hio_wait_list);
		ocs_list_add_tail(&hal->io_inuse, io);
		io->state = OCS_HAL_IO_STATE_INUSE;
		io->sec_hio = sec_io;

		/* mark secondary XRI for second and subsequent data phase as quarantine */
		if (io->xbusy) {
			sec_io->quarantine = TRUE;
		}

		flags = io->sec_iparam.fcp_tgt.flags;
		if (io->xbusy) {
			flags |= SLI4_IO_CONTINUATION;
		} else {
			flags &= ~SLI4_IO_CONTINUATION;
		}

		io->tgt_wqe_timeout = io->sec_iparam.fcp_tgt.timeout;

		/* Complete (continue) TRECV IO */
		if (io->xbusy) {
			if (sli_fcp_cont_treceive64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, &io->def_sgl,
				io->first_data_sge,
				io->sec_iparam.fcp_tgt.offset, io->sec_len, io->indicator, io->sec_hio->indicator,
				io->reqtag, SLI4_CQ_DEFAULT,
				io->sec_iparam.fcp_tgt.ox_id, io->rnode->indicator, io->rnode,
				flags, io->sec_iparam.fcp_tgt.dif_oper, io->sec_iparam.fcp_tgt.blk_size,
				io->sec_iparam.fcp_tgt.cs_ctl, io->sec_iparam.fcp_tgt.app_id,
				io->sec_iparam.fcp_tgt.wqe_timer)) {
					ocs_log_test(hal->os, "TRECEIVE WQE error\n");
					break;
			}
		} else {
			if (sli_fcp_treceive64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, &io->def_sgl,
				io->first_data_sge,
				io->sec_iparam.fcp_tgt.offset, io->sec_len, io->indicator,
				io->reqtag, SLI4_CQ_DEFAULT,
				io->sec_iparam.fcp_tgt.ox_id, io->rnode->indicator, io->rnode,
				flags, io->sec_iparam.fcp_tgt.dif_oper, io->sec_iparam.fcp_tgt.blk_size,
				io->sec_iparam.fcp_tgt.cs_ctl, io->sec_iparam.fcp_tgt.app_id,
				io->sec_iparam.fcp_tgt.wqe_timer)) {
					ocs_log_test(hal->os, "TRECEIVE WQE error\n");
					break;
			}
		}

		if (io->wq == NULL) {
			io->wq = ocs_hal_queue_next_wq(hal, io);
			ocs_hal_assert(io->wq != NULL);
		}
		io->xbusy = TRUE;

		/*
		 * Add IO to active io wqe list before submitting, in case the
		 * wcqe processing preempts this thread.
		 */
		ocs_hal_add_io_timed_wqe(hal, io);
		rc = hal_wq_write(io->wq, &io->wqe);
		if (rc >= 0) {
			/* non-negative return is success */
			rc = 0;
		} else {
			/* failed to write wqe, remove from active wqe list */
			ocs_log_err(hal->os, "sli_queue_write failed: %d\n", rc);
			io->xbusy = FALSE;
			ocs_hal_remove_io_timed_wqe(hal, io);
		}
	}
}

/**
 * @ingroup io
 * @brief Send a Single Request/Response Sequence (SRRS).
 *
 * @par Description
 * This routine supports communication sequences consisting of a single
 * request and single response between two endpoints. Examples include:
 *  - Sending an ELS request.
 *  - Sending an ELS response - To send an ELS reponse, the caller must provide
 * the OX_ID from the received request.
 *  - Sending a FC Common Transport (FC-CT) request - To send a FC-CT request,
 * the caller must provide the R_CTL, TYPE, and DF_CTL
 * values to place in the FC frame header.
 *  .
 * @n @b Note: The caller is expected to provide both send and receive
 * buffers for requests. In the case of sending a response, no receive buffer
 * is necessary and the caller may pass in a NULL pointer.
 *
 * @param hal Hardware context.
 * @param type Type of sequence (ELS request/response, FC-CT).
 * @param io Previously-allocated HAL IO object.
 * @param send DMA memory holding data to send (for example, ELS request, BLS response).
 * @param len Length, in bytes, of data to send.
 * @param receive Optional DMA memory to hold a response.
 * @param rnode Destination of data (that is, a remote node).
 * @param iparam IO parameters (ELS response and FC-CT).
 * @param cb Function call upon completion of sending the data (may be NULL).
 * @param arg Argument to pass to IO completion function.
 *
 * @return Returns 0 on success, or a non-zero on failure.
 */
ocs_hal_rtn_e
ocs_hal_srrs_send(ocs_hal_t *hal, ocs_hal_io_type_e type, ocs_hal_io_t *io,
		  ocs_dma_t *send, uint32_t len, ocs_dma_t *receive,
		  ocs_remote_node_t *rnode, ocs_hal_io_param_t *iparam,
		  ocs_hal_srrs_cb_t cb, void *arg)
{
	sli4_sge_t	*sge = NULL;
	ocs_hal_rtn_e	rc = OCS_HAL_RTN_SUCCESS;
	uint16_t	local_flags = 0;
	bool		select_els_wq = true;

	if (!hal || !io || !rnode || !iparam) {
		ocs_log_err(NULL, "bad parm hal=%p io=%p send=%p receive=%p rnode=%p iparam=%p\n",
			    hal, io, send, receive, rnode, iparam);
		return OCS_HAL_RTN_ERROR;
	}

	if (hal->state != OCS_HAL_STATE_ACTIVE) {
		ocs_log_test(hal->os, "cannot send SRRS, HAL state=%d\n", hal->state);
		return OCS_HAL_RTN_ERROR;
	}

	if (ocs_hal_is_xri_port_owned(hal, io->indicator)) {
		/* We must set the XC bit for port owned XRIs */
		local_flags |= SLI4_IO_CONTINUATION;
	}
	io->rnode = rnode;
	io->type  = type;
	io->done = cb;
	io->arg  = arg;

	sge = io->sgl->virt;

	// clear both SGE
	ocs_memset(io->sgl->virt, 0, 2 * sizeof(sli4_sge_t));

	if (send) {
		sge[0].buffer_address_high = ocs_addr32_hi(send->phys);
		sge[0].buffer_address_low  = ocs_addr32_lo(send->phys);
		sge[0].sge_type = SLI4_SGE_TYPE_DATA;
		sge[0].buffer_length = len;
	}

	if ((OCS_HAL_ELS_REQ == type) || (OCS_HAL_FC_CT == type)) {
		sge[1].buffer_address_high = ocs_addr32_hi(receive->phys);
		sge[1].buffer_address_low  = ocs_addr32_lo(receive->phys);
		sge[1].sge_type = SLI4_SGE_TYPE_DATA;
		sge[1].buffer_length = receive->size;
		sge[1].last = TRUE;
	} else {
		sge[0].last = TRUE;
	}

	switch (type) {
	case OCS_HAL_ELS_REQ:
		if (!iparam->els.timeout)
			io->tgt_wqe_timeout = 2 * (hal->sli.config.r_a_tov) + 5;
		else
			io->tgt_wqe_timeout = iparam->els.timeout + 5;

		if ((!send) || sli_els_request64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, io->sgl,
						      *((uint8_t *)(send->virt)), /* req_type */
						      len, receive->size,
						      iparam->els.timeout, io->indicator, io->reqtag, SLI4_CQ_DEFAULT, rnode)) {
			ocs_log_err(hal->os, "REQ WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_ELS_RSP:
		if ((!send) || sli_xmit_els_rsp64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, send, len,
					   io->indicator, io->reqtag, SLI4_CQ_DEFAULT,
					   iparam->els.ox_id,
						       rnode, local_flags, UINT32_MAX)) {
			ocs_log_err(hal->os, "RSP WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_ELS_RSP_SID:
		if ((!send) || sli_xmit_els_rsp64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, send, len,
					   io->indicator, io->reqtag, SLI4_CQ_DEFAULT,
					   iparam->els_sid.ox_id,
						       rnode, local_flags, iparam->els_sid.s_id)) {
			ocs_log_err(hal->os, "RSP (SID) WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_FC_CT:
		/* Pick regular WQ for CT requests to avoid deadlock */
		select_els_wq = false;
		if ((!send) || sli_gen_request64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, io->sgl, len,
					  receive->size, iparam->fc_ct.timeout, io->indicator,
					  io->reqtag, SLI4_CQ_DEFAULT, rnode, iparam->fc_ct.r_ctl,
					  iparam->fc_ct.type, iparam->fc_ct.df_ctl)) {
			ocs_log_err(hal->os, "GEN WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_FC_CT_RSP:
		/* Pick regular WQ for CT responses to avoid deadlock */
		select_els_wq = false;
		if ((!send) || sli_xmit_sequence64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, io->sgl, len,
					  iparam->fc_ct_rsp.timeout, iparam->fc_ct_rsp.ox_id, io->indicator,
					  io->reqtag, rnode, iparam->fc_ct_rsp.r_ctl,
					  iparam->fc_ct_rsp.type, iparam->fc_ct_rsp.df_ctl)) {
			ocs_log_err(hal->os, "XMIT SEQ WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_BLS_ACC:
	case OCS_HAL_BLS_RJT:
	{
		sli_bls_payload_t	bls;

		if (OCS_HAL_BLS_ACC == type) {
			bls.type = SLI_BLS_ACC;
			ocs_memcpy(&bls.u.acc, iparam->bls.payload, sizeof(bls.u.acc));
		} else {
			bls.type = SLI_BLS_RJT;
			ocs_memcpy(&bls.u.rjt, iparam->bls.payload, sizeof(bls.u.rjt));
		}

		bls.ox_id = iparam->bls.ox_id;
		bls.rx_id = iparam->bls.rx_id;

		if (sli_xmit_bls_rsp64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, &bls,
					   io->indicator, io->reqtag,
					   SLI4_CQ_DEFAULT,
					   rnode, UINT32_MAX)) {
			ocs_log_err(hal->os, "XMIT_BLS_RSP64 WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}

		/* Pick regular WQ for backend BLS responses to avoid race conditions */
		select_els_wq = false;
		break;
	}
	case OCS_HAL_BLS_ACC_SID:
	{
		sli_bls_payload_t	bls;

		bls.type = SLI_BLS_ACC;
		ocs_memcpy(&bls.u.acc, iparam->bls_sid.payload, sizeof(bls.u.acc));

		bls.ox_id = iparam->bls_sid.ox_id;
		bls.rx_id = iparam->bls_sid.rx_id;

		if (sli_xmit_bls_rsp64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, &bls,
					   io->indicator, io->reqtag,
					   SLI4_CQ_DEFAULT,
					   rnode, iparam->bls_sid.s_id)) {
			ocs_log_err(hal->os, "XMIT_BLS_RSP64 WQE SID error\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	}
	case OCS_HAL_BCAST:
		if ((!send) || sli_xmit_bcast64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, send, len,
					iparam->bcast.timeout, io->indicator, io->reqtag,
					SLI4_CQ_DEFAULT, rnode,
					iparam->bcast.r_ctl, iparam->bcast.type, iparam->bcast.df_ctl)) {
			ocs_log_err(hal->os, "XMIT_BCAST64 WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	default:
		ocs_log_err(hal->os, "bad SRRS type %#x\n", type);
		rc = OCS_HAL_RTN_ERROR;
	}

	if (OCS_HAL_RTN_SUCCESS == rc) {
		if (io->wq == NULL) {
			if (select_els_wq)
				io->wq = hal->hal_els_wq;
			else
				io->wq = ocs_hal_queue_next_wq(hal, io);
			ocs_hal_assert(io->wq != NULL);
		}
		io->xbusy = TRUE;

		/*
		 * Add IO to active io wqe list before submitting, in case the
		 * wcqe processing preempts this thread.
		 */
		OCS_STAT(io->wq->use_count++);
		ocs_hal_add_io_timed_wqe(hal, io);
		rc = hal_wq_write(io->wq, &io->wqe);
		if (rc >= 0) {
			/* non-negative return is success */
			rc = 0;
		} else {
			/* failed to write wqe, remove from active wqe list */
			ocs_log_err(hal->os, "sli_queue_write failed: %d\n", rc);
			io->xbusy = FALSE;
			ocs_hal_remove_io_timed_wqe(hal, io);
		}
	}

	return rc;
}

/**
 * @ingroup io
 * @brief Send a read, write, or response IO.
 *
 * @par Description
 * This routine supports sending a higher-level IO (for example, FCP) between two endpoints
 * as a target or initiator. Examples include:
 *  - Sending read data and good response (target).
 *  - Sending a response (target with no data or after receiving write data).
 *  .
 * This routine assumes all IOs use the SGL associated with the HAL IO. Prior to
 * calling this routine, the data should be loaded using ocs_hal_io_add_sge().
 *
 * @param hal Hardware context.
 * @param type Type of IO (target read, target response, and so on).
 * @param io Previously-allocated HAL IO object.
 * @param len Length, in bytes, of data to send.
 * @param iparam IO parameters.
 * @param rnode Destination of data (that is, a remote node).
 * @param cb Function call upon completion of sending data (may be NULL).
 * @param arg Argument to pass to IO completion function.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 *
 * @todo
 *  - Support specifiying relative offset.
 *  - Use a WQ other than 0.
 */
ocs_hal_rtn_e
ocs_hal_io_send(ocs_hal_t *hal, ocs_hal_io_type_e type, ocs_hal_io_t *io,
		uint32_t len, ocs_hal_io_param_t *iparam, ocs_remote_node_t *rnode,
		void *cb, void *arg)
{
	ocs_hal_rtn_e	rc = OCS_HAL_RTN_SUCCESS;
	uint32_t	rpi;
	uint8_t		send_wqe = TRUE;

	CPUTRACE("");

	if (!hal || !io || !rnode || !iparam) {
		ocs_log_err(NULL, "bad parm hal=%p io=%p iparam=%p rnode=%p\n", hal, io, iparam, rnode);
		return OCS_HAL_RTN_ERROR;
	}

	if (hal->state != OCS_HAL_STATE_ACTIVE) {
		ocs_log_err(hal->os, "Cannot send IO (xri=%#x), HAL state=%d\n",
			    io->indicator, hal->state);
		return OCS_HAL_RTN_ERROR;
	}

	rpi = rnode->indicator;

	if (hal->workaround.use_unregistered_rpi && (rpi == UINT32_MAX)) {
		rpi = hal->workaround.unregistered_rid;
		ocs_log_test(hal->os, "using unregistered RPI: %d\n", rpi);
	}

	/*
	 * Save state needed during later stages
	 */
	io->rnode = rnode;
	io->type  = type;
	io->done  = cb;
	io->arg   = arg;

	/*
	 * Format the work queue entry used to send the IO
	 */
	switch (type) {
	case OCS_HAL_IO_INITIATOR_READ:
		/*
		 * If use_dif_quarantine workaround is in effect, and dif_separates then mark the
		 * initiator read IO for quarantine
		 */
		if (hal->workaround.use_dif_quarantine && (hal->config.dif_mode == OCS_HAL_DIF_MODE_SEPARATE) &&
		    (iparam->fcp_tgt.dif_oper != OCS_HAL_DIF_OPER_DISABLED)) {
			io->quarantine = TRUE;
		}

		ocs_hal_io_ini_sge(hal, io, iparam->fcp_ini.cmnd, iparam->fcp_ini.cmnd_size,
				iparam->fcp_ini.rsp);

		if (sli_fcp_iread64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, &io->def_sgl, io->first_data_sge, len,
					io->indicator, io->reqtag, SLI4_CQ_DEFAULT, rpi, rnode,
					iparam->fcp_ini.dif_oper, iparam->fcp_ini.blk_size,
					iparam->fcp_ini.timeout)) {
			ocs_log_err(hal->os, "IREAD WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_IO_INITIATOR_WRITE:
		ocs_hal_io_ini_sge(hal, io, iparam->fcp_ini.cmnd, iparam->fcp_ini.cmnd_size,
				iparam->fcp_ini.rsp);

		if (sli_fcp_iwrite64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, &io->def_sgl, io->first_data_sge,
					 len, iparam->fcp_ini.first_burst,
					 io->indicator, io->reqtag,
					SLI4_CQ_DEFAULT, rpi, rnode,
					iparam->fcp_ini.dif_oper, iparam->fcp_ini.blk_size,
					iparam->fcp_ini.timeout)) {
			ocs_log_err(hal->os, "IWRITE WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_IO_INITIATOR_NODATA:
		ocs_hal_io_ini_sge(hal, io, iparam->fcp_ini.cmnd, iparam->fcp_ini.cmnd_size,
				iparam->fcp_ini.rsp);

		if (sli_fcp_icmnd64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, &io->def_sgl,
					io->indicator, io->reqtag, SLI4_CQ_DEFAULT,
					rpi, rnode, iparam->fcp_ini.timeout)) {
			ocs_log_err(hal->os, "ICMND WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}
		break;
	case OCS_HAL_IO_TARGET_WRITE: {
		uint16_t flags = iparam->fcp_tgt.flags;
		fcp_xfer_rdy_iu_t *xfer = io->xfer_rdy.virt;

		/*
		 * Fill in the XFER_RDY for IF_TYPE 0 devices
		 */
		*((uint32_t *)xfer->fcp_data_ro) = ocs_htobe32(iparam->fcp_tgt.offset);
		*((uint32_t *)xfer->fcp_burst_len) = ocs_htobe32(len);
		*((uint32_t *)xfer->rsvd) = 0;

		if (io->xbusy) {
			flags |= SLI4_IO_CONTINUATION;
		} else {
			flags &= ~SLI4_IO_CONTINUATION;
		}

		io->tgt_wqe_timeout = iparam->fcp_tgt.timeout;

		/*
		 * If use_dif_quarantine workaround is in effect, and this is a DIF enabled IO
		 * then mark the target write IO for quarantine
		 */
		if (hal->workaround.use_dif_quarantine && (hal->config.dif_mode == OCS_HAL_DIF_MODE_SEPARATE) &&
		    (iparam->fcp_tgt.dif_oper != OCS_HAL_DIF_OPER_DISABLED)) {
			io->quarantine = TRUE;
		}

		/*
		 * BZ 161832 Workaround:
		 * Check for use_dif_sec_xri workaround.  Note, even though the first dataphase
		 * doesn't really need a secondary XRI, we allocate one anyway, as this avoids the
		 * potential for deadlock where all XRI's are allocated as primaries to IOs that
		 * are on hal->sec_hio_wait_list.   If this secondary XRI is not for the first
		 * data phase, it is marked for quarantine.
		 */
		if (hal->workaround.use_dif_sec_xri && (iparam->fcp_tgt.dif_oper != OCS_HAL_DIF_OPER_DISABLED)) {

			/*
			 * If we have allocated a chained SGL for skyhawk, then
			 * we can re-use this for the sec_hio.
			 */
			if (io->ovfl_io != NULL) {
				io->sec_hio = io->ovfl_io;
				io->sec_hio->quarantine = TRUE;
			} else {
				io->sec_hio = ocs_hal_io_alloc(hal);
			}
			if (io->sec_hio == NULL) {
				/* Failed to allocate, so save full request context and put
				 * this IO on the wait list
				 */
				io->sec_iparam = *iparam;
				io->sec_len = len;
				ocs_lock(&hal->io_lock);
					ocs_list_remove(&hal->io_inuse, io);
					ocs_list_add_tail(&hal->sec_hio_wait_list, io);
					io->state = OCS_HAL_IO_STATE_WAIT_SEC_HIO;
					hal->sec_hio_wait_count++;
				ocs_unlock(&hal->io_lock);
				send_wqe = FALSE;
				/* Done */
				break;
			}
			/* We quarantine the secondary IO if this is the second or subsequent data phase */
			if (io->xbusy) {
				io->sec_hio->quarantine = TRUE;
			}
		}

		/*
		 * If not the first data phase, and io->sec_hio has been allocated, then issue
		 * FCP_CONT_TRECEIVE64 WQE, otherwise use the usual FCP_TRECEIVE64 WQE
		 */
		if (io->xbusy && (io->sec_hio != NULL)) {
			if (sli_fcp_cont_treceive64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, &io->def_sgl, io->first_data_sge,
						   iparam->fcp_tgt.offset, len, io->indicator, io->sec_hio->indicator,
						   io->reqtag, SLI4_CQ_DEFAULT,
						   iparam->fcp_tgt.ox_id, rpi, rnode,
						   flags, iparam->fcp_tgt.dif_oper, iparam->fcp_tgt.blk_size,
						   iparam->fcp_tgt.cs_ctl, iparam->fcp_tgt.app_id,
						   iparam->fcp_tgt.wqe_timer)) {
				ocs_log_err(hal->os, "TRECEIVE WQE error\n");
				rc = OCS_HAL_RTN_ERROR;
			}
		} else {
			if (sli_fcp_treceive64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, &io->def_sgl, io->first_data_sge,
						   iparam->fcp_tgt.offset, len, io->indicator, io->reqtag,
						   SLI4_CQ_DEFAULT,
						   iparam->fcp_tgt.ox_id, rpi, rnode,
						   flags, iparam->fcp_tgt.dif_oper, iparam->fcp_tgt.blk_size,
						   iparam->fcp_tgt.cs_ctl, iparam->fcp_tgt.app_id,
						   iparam->fcp_tgt.wqe_timer)) {
				ocs_log_err(hal->os, "TRECEIVE WQE error\n");
				rc = OCS_HAL_RTN_ERROR;
			}
		}
		break;
	}
	case OCS_HAL_IO_TARGET_READ: {
		uint16_t flags = iparam->fcp_tgt.flags;
		uint32_t suppress_rsp = false;

		if (io->xbusy) {
			flags |= SLI4_IO_CONTINUATION;
		} else {
			flags &= ~SLI4_IO_CONTINUATION;
		}

		/*
		 * Suppress the response when following conditions are met.
		 * 1. HAL suppress resp is capable.
		 * 2. Remote node is negotiated for suppress resp.
		 * 3. Auto response is enabled.
		 */
		ocs_hal_get(hal, OCS_HAL_SUPPRESS_RSP_CAPABLE, &suppress_rsp);
		if ((flags & SLI4_IO_AUTO_GOOD_RESPONSE) && suppress_rsp && ocs_node_suppress_resp(rnode))
			flags |= SLI4_IO_SUPPRESS_RESPONSE;

		io->tgt_wqe_timeout = iparam->fcp_tgt.timeout;
		if (sli_fcp_tsend64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size, &io->def_sgl, io->first_data_sge,
					iparam->fcp_tgt.offset, len, io->indicator, io->reqtag,
					SLI4_CQ_DEFAULT,
					iparam->fcp_tgt.ox_id, rpi, rnode,
					flags,
					iparam->fcp_tgt.dif_oper,
					iparam->fcp_tgt.blk_size,
					iparam->fcp_tgt.cs_ctl,
					iparam->fcp_tgt.app_id)) {
			ocs_log_err(hal->os, "TSEND WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		} else if (hal->workaround.retain_tsend_io_length) {
			io->length = len;
		}
		break;
	}
	case OCS_HAL_IO_TARGET_RSP: {
		uint16_t flags = iparam->fcp_tgt.flags;

		if (io->xbusy) {
			flags |= SLI4_IO_CONTINUATION;
		} else {
			flags &= ~SLI4_IO_CONTINUATION;
		}

		/* Attach buffer for port_owned XRI */
		if (hal->tow_enabled && io->is_port_owned) {
			if ((io->tow_dnrx = ocs_hal_rqpair_tow_xri_buffer_attach(hal, io, TRUE)))
				flags |= SLI4_IO_DNRX;
		}

		io->tgt_wqe_timeout = iparam->fcp_tgt.timeout;
		if (sli_fcp_trsp64_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size,
				       &io->def_sgl,
				       len,
				       io->indicator, io->reqtag,
				       SLI4_CQ_DEFAULT,
				       iparam->fcp_tgt.ox_id,
				       rpi, rnode,
				       flags, iparam->fcp_tgt.cs_ctl,
				       io->is_port_owned,
				       iparam->fcp_tgt.app_id)) {
			ocs_log_err(hal->os, "TRSP WQE error\n");
			rc = OCS_HAL_RTN_ERROR;
		}

		break;
	}
	default:
		ocs_log_err(hal->os, "unsupported IO type %#x\n", type);
		rc = OCS_HAL_RTN_ERROR;
	}

	if (send_wqe && (OCS_HAL_RTN_SUCCESS == rc)) {
		if (io->wq == NULL) {
			io->wq = ocs_hal_queue_next_wq(hal, io);
			ocs_hal_assert(io->wq != NULL);
		}

		io->xbusy = TRUE;

		/*
		 * Add IO to active io wqe list before submitting, in case the
		 * wcqe processing preempts this thread.
		 */
		OCS_STAT(hal->tcmd_wq_submit[io->wq->instance]++);
		OCS_STAT(io->wq->use_count++);
		ocs_hal_add_io_timed_wqe(hal, io);
		rc = hal_wq_write(io->wq, &io->wqe);
		if (rc >= 0) {
			/* non-negative return is success */
			rc = 0;
		} else {
			/* failed to write wqe, remove from active wqe list */
			ocs_log_err(hal->os, "sli_queue_write failed: %d\n", rc);
			io->xbusy = FALSE;
			ocs_hal_remove_io_timed_wqe(hal, io);
		}
	}

	return rc;
}

/**
 * @brief Send a raw frame
 *
 * @par Description
 * Using the SEND_FRAME_WQE, a frame consisting of header and payload is sent.
 *
 * @param hal Pointer to HAL object.
 * @param hdr Pointer to a little endian formatted FC header.
 * @param sof Value to use as the frame SOF.
 * @param eof Value to use as the frame EOF.
 * @param payload Pointer to payload DMA buffer.
 * @param ctx Pointer to caller provided send frame context.
 * @param callback Callback function.
 * @param arg Callback function argument.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
ocs_hal_rtn_e
ocs_hal_send_frame(ocs_hal_t *hal, fc_header_le_t *hdr, uint8_t sof,
		   uint8_t eof, ocs_dma_t *payload, ocs_hal_send_frame_context_t *ctx,
		   void (*callback)(void *arg, uint8_t *cqe, int32_t status), void *arg)
{
	int32_t rc;
	ocs_hal_wqe_t *wqe;
	uint32_t xri;
	hal_wq_t *wq;

	wqe = &ctx->wqe;

	/* populate the callback object */
	ctx->hal = hal;

	/* Fetch and populate request tag */
	ctx->wqcb = ocs_hal_reqtag_alloc(hal, callback, arg);
	if (ctx->wqcb == NULL) {
		ocs_log_err(hal->os, "can't allocate request tag\n");
		return OCS_HAL_RTN_NO_RESOURCES;
	}

	wq = hal->hal_sfwq[OCS_HAL_WQ_SFQ_SCSI];
	if (!wq) {
		ocs_log_err(hal->os, "Send frame WQ is not allocated\n");
		rc = OCS_HAL_RTN_ERROR;
		goto exit;
	}

	/* Set XRI and RX_ID in the header based on the WQ and send_frame_io that are being used */
	xri = wq->send_frame_io->indicator;

	/* Build the send frame WQE */
	sli_send_frame_wqe(&hal->sli, wqe->wqebuf,
			   hal->sli.config.wqe_size, sof, eof,
			   (uint32_t*) hdr, payload, payload->len,
			   OCS_HAL_SEND_FRAME_TIMEOUT, xri,
			   ctx->wqcb->instance_index);

	/* Write to WQ */
	rc = hal_wq_write(wq, wqe);
	if (rc) {
		ocs_log_err(hal->os, "hal_wq_write failed: %d\n", rc);
		rc = OCS_HAL_RTN_ERROR;
		goto exit;
	}

	OCS_STAT(wq->use_count++);
	return OCS_HAL_RTN_SUCCESS;

exit:
	ocs_hal_reqtag_free(hal, ctx->wqcb);
	ctx->wqcb = NULL;
	return rc;
}

ocs_hal_rtn_e
ocs_hal_io_register_sgl(ocs_hal_t *hal, ocs_hal_io_t *io, ocs_dma_t *sgl, uint32_t sgl_count)
{
	if (sli_get_sgl_preregister(&hal->sli)) {
		ocs_log_err(hal->os, "can't use temporary SGL with pre-registered SGLs\n");
		return OCS_HAL_RTN_ERROR;
	}
	io->ovfl_sgl = sgl;
	io->ovfl_sgl_count = sgl_count;
	io->ovfl_io = NULL;

	return OCS_HAL_RTN_SUCCESS;
}

static void
ocs_hal_io_restore_sgl(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	/* Restore the default */
	io->sgl = &io->def_sgl;
	io->sgl_count = io->def_sgl_count;

	/*
	 * For skyhawk, we need to free the IO allocated for the chained
	 * SGL. For all devices, clear the overflow fields on the IO.
	 *
	 * Note: For DIF IOs, we may be using the same XRI for the sec_hio and
	 *       the chained SGLs. If so, then we clear the ovfl_io field
	 *       when the sec_hio is freed.
	 */
	if (io->ovfl_io != NULL) {
		ocs_hal_io_free(hal, io->ovfl_io);
		io->ovfl_io = NULL;
	}

	/* Clear the overflow SGL */
	io->ovfl_sgl = NULL;
	io->ovfl_sgl_count = 0;
	io->ovfl_lsp = NULL;
}

/**
 * @ingroup io
 * @brief Initialize the scatter gather list entries of an IO.
 *
 * @param hal Hardware context.
 * @param io Previously-allocated HAL IO object.
 * @param type Type of IO (target read, target response, and so on).
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_io_init_sges(ocs_hal_t *hal, ocs_hal_io_t *io, ocs_hal_io_type_e type)
{
	sli4_sge_t	*data = NULL;
	uint32_t	i = 0;
	uint32_t	skips = 0;

	if (!hal || !io) {
		ocs_log_err(hal ? hal->os : NULL, "bad parameter hal=%p io=%p\n", hal, io);
		return OCS_HAL_RTN_ERROR;
	}

	// Clear / reset the scatter-gather list
	io->sgl = &io->def_sgl;
	io->sgl_count = io->def_sgl_count;
	io->first_data_sge = 0;

	ocs_memset(io->sgl->virt, 0, 2 * sizeof(sli4_sge_t));
	io->n_sge = 0;
	io->sge_offset = 0;

	io->type = type;

	data = io->sgl->virt;

	/*
	 * Some IO types have underlying hardware requirements on the order
	 * of SGEs. Process all special entries here.
	 */
	switch (type) {
	case OCS_HAL_IO_INITIATOR_READ:
	case OCS_HAL_IO_INITIATOR_WRITE:
	case OCS_HAL_IO_INITIATOR_NODATA:
		/*
		 * No skips, 2 special for initiator I/Os
		 * The addresses and length are written later
		 */
		// setup command pointer
		data->sge_type = SLI4_SGE_TYPE_DATA;
		data++;

		// setup response pointer
		data->sge_type = SLI4_SGE_TYPE_DATA;

		if (OCS_HAL_IO_INITIATOR_NODATA == type) {
			data->last = TRUE;
		}
		data++;

		io->n_sge = 2;
		break;
	case OCS_HAL_IO_TARGET_WRITE:
#define OCS_TARGET_WRITE_SKIPS	2
		skips = OCS_TARGET_WRITE_SKIPS;

		/* populate host resident XFER_RDY buffer */
		data->sge_type = SLI4_SGE_TYPE_DATA;
		data->buffer_address_high = ocs_addr32_hi(io->xfer_rdy.phys);
		data->buffer_address_low  = ocs_addr32_lo(io->xfer_rdy.phys);
		data->buffer_length = io->xfer_rdy.size;
		data++;

		skips--;

		io->n_sge = 1;
		break;
	case OCS_HAL_IO_TARGET_READ:
		/*
		 * For FCP_TSEND64, the first 2 entries are SKIP SGE's
		 */
#define OCS_TARGET_READ_SKIPS	2
		skips = OCS_TARGET_READ_SKIPS;
		break;
	case OCS_HAL_IO_TARGET_RSP:
		/*
		 * No skips, etc. for FCP_TRSP64
		 */
		break;
	default:
		ocs_log_err(hal->os, "unsupported IO type %#x\n", type);
		return OCS_HAL_RTN_ERROR;
	}

	/*
	 * Write skip entries
	 */
	for (i = 0; i < skips; i++) {
		data->sge_type = SLI4_SGE_TYPE_SKIP;
		data++;
	}

	io->n_sge += skips;

	/*
	 * Set last
	 */
	data->last = TRUE;

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup io
 * @brief Add a T10 PI seed scatter gather list entry.
 *
 * @param hal Hardware context.
 * @param io Previously-allocated HAL IO object.
 * @param dif_info Pointer to T10 DIF fields, or NULL if no DIF.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_io_add_seed_sge(ocs_hal_t *hal, ocs_hal_io_t *io, ocs_hal_dif_info_t *dif_info)
{
	sli4_sge_t	*data = NULL;
	sli4_diseed_sge_t *dif_seed;

	/* If no dif_info, or dif_oper is disabled, then just return success */
	if ((dif_info == NULL) || (dif_info->dif_oper == OCS_HAL_DIF_OPER_DISABLED)) {
		return OCS_HAL_RTN_SUCCESS;
	}

	if (!hal || !io) {
		ocs_log_err(hal ? hal->os : NULL, "bad parameter hal=%p io=%p dif_info=%p\n", hal, io, dif_info);
		return OCS_HAL_RTN_ERROR;
	}

	data = io->sgl->virt;
	data += io->n_sge;

	/* If we are doing T10 DIF add the DIF Seed SGE */
	ocs_memset(data, 0, sizeof(sli4_diseed_sge_t));
	dif_seed = (sli4_diseed_sge_t *)data;
	dif_seed->ref_tag_cmp = dif_info->ref_tag_cmp;
	dif_seed->ref_tag_repl = dif_info->ref_tag_repl;
	dif_seed->app_tag_repl = dif_info->app_tag_repl;
	dif_seed->repl_app_tag = dif_info->repl_app_tag;
	if (SLI4_IF_TYPE_LANCER_FC_ETH != hal->sli.if_type) {
		dif_seed->atrt = dif_info->disable_app_ref_ffff;
		dif_seed->at = dif_info->disable_app_ffff;
	}
	dif_seed->sge_type = SLI4_SGE_TYPE_DISEED;
	/* Workaround for SKH (BZ157233) */
	if (((io->type == OCS_HAL_IO_TARGET_WRITE) || (io->type == OCS_HAL_IO_INITIATOR_READ)) &&
		(SLI4_IF_TYPE_LANCER_FC_ETH != hal->sli.if_type) && dif_info->dif_separate) {
		dif_seed->sge_type = SLI4_SGE_TYPE_SKIP;
	}

	dif_seed->app_tag_cmp = dif_info->app_tag_cmp;
	dif_seed->dif_blk_size = dif_info->blk_size;
	dif_seed->auto_incr_ref_tag = dif_info->auto_incr_ref_tag;
	dif_seed->check_app_tag = dif_info->check_app_tag;
	dif_seed->check_ref_tag = dif_info->check_ref_tag;
	dif_seed->check_crc = dif_info->check_guard;
	dif_seed->new_ref_tag = dif_info->repl_ref_tag;

	switch(dif_info->dif_oper) {
	case OCS_HAL_SGE_DIF_OP_IN_NODIF_OUT_CRC:
		dif_seed->dif_op_rx = SLI4_SGE_DIF_OP_IN_NODIF_OUT_CRC;
		dif_seed->dif_op_tx = SLI4_SGE_DIF_OP_IN_NODIF_OUT_CRC;
		break;
	case OCS_HAL_SGE_DIF_OP_IN_CRC_OUT_NODIF:
		dif_seed->dif_op_rx = SLI4_SGE_DIF_OP_IN_CRC_OUT_NODIF;
		dif_seed->dif_op_tx = SLI4_SGE_DIF_OP_IN_CRC_OUT_NODIF;
		break;
	case OCS_HAL_SGE_DIF_OP_IN_NODIF_OUT_CHKSUM:
		dif_seed->dif_op_rx = SLI4_SGE_DIF_OP_IN_NODIF_OUT_CHKSUM;
		dif_seed->dif_op_tx = SLI4_SGE_DIF_OP_IN_NODIF_OUT_CHKSUM;
		break;
	case OCS_HAL_SGE_DIF_OP_IN_CHKSUM_OUT_NODIF:
		dif_seed->dif_op_rx = SLI4_SGE_DIF_OP_IN_CHKSUM_OUT_NODIF;
		dif_seed->dif_op_tx = SLI4_SGE_DIF_OP_IN_CHKSUM_OUT_NODIF;
		break;
	case OCS_HAL_SGE_DIF_OP_IN_CRC_OUT_CRC:
		dif_seed->dif_op_rx = SLI4_SGE_DIF_OP_IN_CRC_OUT_CRC;
		dif_seed->dif_op_tx = SLI4_SGE_DIF_OP_IN_CRC_OUT_CRC;
		break;
	case OCS_HAL_SGE_DIF_OP_IN_CHKSUM_OUT_CHKSUM:
		dif_seed->dif_op_rx = SLI4_SGE_DIF_OP_IN_CHKSUM_OUT_CHKSUM;
		dif_seed->dif_op_tx = SLI4_SGE_DIF_OP_IN_CHKSUM_OUT_CHKSUM;
		break;
	case OCS_HAL_SGE_DIF_OP_IN_CRC_OUT_CHKSUM:
		dif_seed->dif_op_rx = SLI4_SGE_DIF_OP_IN_CRC_OUT_CHKSUM;
		dif_seed->dif_op_tx = SLI4_SGE_DIF_OP_IN_CRC_OUT_CHKSUM;
		break;
	case OCS_HAL_SGE_DIF_OP_IN_CHKSUM_OUT_CRC:
		dif_seed->dif_op_rx = SLI4_SGE_DIF_OP_IN_CHKSUM_OUT_CRC;
		dif_seed->dif_op_tx = SLI4_SGE_DIF_OP_IN_CHKSUM_OUT_CRC;
		break;
	case OCS_HAL_SGE_DIF_OP_IN_RAW_OUT_RAW:
		dif_seed->dif_op_rx = SLI4_SGE_DIF_OP_IN_RAW_OUT_RAW;
		dif_seed->dif_op_tx = SLI4_SGE_DIF_OP_IN_RAW_OUT_RAW;
		break;
	default:
		ocs_log_err(hal->os, "unsupported DIF operation %#x\n", dif_info->dif_oper);
		return OCS_HAL_RTN_ERROR;
	}

	/*
	 * Set last, clear previous last
	 */
	data->last = TRUE;
	if (io->n_sge) {
		data[-1].last = FALSE;
	}

	io->n_sge++;

	return OCS_HAL_RTN_SUCCESS;
}

static ocs_hal_rtn_e
ocs_hal_io_overflow_sgl(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	sli4_lsp_sge_t *lsp;

	/* fail if we're already pointing to the overflow SGL */
	if (io->sgl == io->ovfl_sgl) {
		return OCS_HAL_RTN_ERROR;
	}

	/*
	 * For skyhawk, we can use another SGL to extend the SGL list. The
	 * Chained entry must not be in the first 4 entries.
	 *
	 * Note: For DIF enabled IOs, we will use the ovfl_io for the sec_hio.
	 */
	if (sli_get_sgl_preregister(&hal->sli) &&
	    io->def_sgl_count > 4 &&
	    io->ovfl_io == NULL &&
	    ((SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(&hal->sli)) ||
	     (SLI4_IF_TYPE_BE3_SKH_VF == sli_get_if_type(&hal->sli)))) {
		io->ovfl_io = ocs_hal_io_alloc(hal);
		if (io->ovfl_io != NULL) {
			/*
			 * Note: We can't call ocs_hal_io_register_sgl() here
			 * because it checks that SGLs are not pre-registered
			 * and for shyhawk, preregistered SGLs are required.
			 */
			io->ovfl_sgl = &io->ovfl_io->def_sgl;
			io->ovfl_sgl_count = io->ovfl_io->def_sgl_count;
		}
	}

	/* fail if we don't have an overflow SGL registered */
	if (io->ovfl_io == NULL || io->ovfl_sgl == NULL)
		return OCS_HAL_RTN_ERROR;

	/*
	 * Overflow, we need to put a link SGE in the last location of the current SGL, after
	 * copying the the last SGE to the overflow SGL
	 */

	((sli4_sge_t*)io->ovfl_sgl->virt)[0] = ((sli4_sge_t*)io->sgl->virt)[io->n_sge - 1];

	lsp = &((sli4_lsp_sge_t*)io->sgl->virt)[io->n_sge - 1];
	ocs_memset(lsp, 0, sizeof(*lsp));

	if ((SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(&hal->sli)) ||
	    (SLI4_IF_TYPE_BE3_SKH_VF == sli_get_if_type(&hal->sli))) {
		sli_skh_chain_sge_build(&hal->sli,
					(sli4_sge_t*)lsp,
					io->ovfl_io->indicator,
					0, /* frag_num */
					0); /* offset */
	} else {
		lsp->buffer_address_high = ocs_addr32_hi(io->ovfl_sgl->phys);
		lsp->buffer_address_low  = ocs_addr32_lo(io->ovfl_sgl->phys);
		lsp->sge_type = SLI4_SGE_TYPE_LSP;
		lsp->last = 0;
		io->ovfl_lsp = lsp;
		io->ovfl_lsp->segment_length = sizeof(sli4_sge_t);
	}

	/* Update the current SGL pointer, and n_sgl */
	io->sgl = io->ovfl_sgl;
	io->sgl_count = io->ovfl_sgl_count;
	io->n_sge = 1;

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup io
 * @brief Add a scatter gather list entry to an IO.
 *
 * @param hal Hardware context.
 * @param io Previously-allocated HAL IO object.
 * @param addr Physical address.
 * @param length Length of memory pointed to by @c addr.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_io_add_sge(ocs_hal_t *hal, ocs_hal_io_t *io, uintptr_t addr, uint32_t length)
{
	sli4_sge_t	*data = NULL;

	if (!hal || !io || !addr || !length) {
		ocs_log_err(hal ? hal->os : NULL, "bad parameter hal=%p io=%p addr=%lx length=%u\n",
			    hal, io, addr, length);
		return OCS_HAL_RTN_ERROR;
	}

	if ((length != 0) && (io->n_sge + 1) > io->sgl_count) {
		if (ocs_hal_io_overflow_sgl(hal, io) != OCS_HAL_RTN_SUCCESS) {
			ocs_log_err(hal->os, "SGL full (%d)\n", io->n_sge);
			return OCS_HAL_RTN_ERROR;
		}
	}

	if (length > sli_get_max_sge(&hal->sli)) {
		ocs_log_err(hal->os, "length of SGE %d bigger than allowed %d\n", length, sli_get_max_sge(&hal->sli));
		return OCS_HAL_RTN_ERROR;
	}

	data = io->sgl->virt;
	data += io->n_sge;

	data->sge_type = SLI4_SGE_TYPE_DATA;
	data->buffer_address_high = ocs_addr32_hi(addr);
	data->buffer_address_low  = ocs_addr32_lo(addr);
	data->buffer_length = length;
	data->data_offset = io->sge_offset;
	/*
	 * Always assume this is the last entry and mark as such.
	 * If this is not the first entry unset the "last SGE"
	 * indication for the previous entry
	 */
	data->last = TRUE;
	if (io->n_sge) {
		data[-1].last = FALSE;
	}

	/* Set first_data_bde if not previously set */
	if (io->first_data_sge == 0) {
		io->first_data_sge = io->n_sge;
	}

	io->sge_offset += length;
	io->n_sge++;

	/* Update the linked segment length (only executed after overflow has begun) */
	if (io->ovfl_lsp != NULL) {
		io->ovfl_lsp->segment_length = io->n_sge * sizeof(sli4_sge_t);
	}

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup io
 * @brief Add a T10 DIF scatter gather list entry to an IO.
 *
 * @param hal Hardware context.
 * @param io Previously-allocated HAL IO object.
 * @param addr DIF physical address.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_io_add_dif_sge(ocs_hal_t *hal, ocs_hal_io_t *io, uintptr_t addr)
{
	sli4_dif_sge_t	*data = NULL;

	if (!hal || !io || !addr) {
		ocs_log_err(hal ? hal->os : NULL, "bad parameter hal=%p io=%p addr=%lx\n", hal, io, addr);
		return OCS_HAL_RTN_ERROR;
	}

	if ((io->n_sge + 1) > hal->config.n_sgl) {
		if (ocs_hal_io_overflow_sgl(hal, io) != OCS_HAL_RTN_ERROR) {
			ocs_log_err(hal->os, "SGL full (%d)\n", io->n_sge);
			return OCS_HAL_RTN_ERROR;
		}
	}

	data = io->sgl->virt;
	data += io->n_sge;

	data->sge_type = SLI4_SGE_TYPE_DIF;
	/* Workaround for SKH (BZ157233) */
	if (((io->type == OCS_HAL_IO_TARGET_WRITE) || (io->type == OCS_HAL_IO_INITIATOR_READ)) &&
		(SLI4_IF_TYPE_LANCER_FC_ETH != hal->sli.if_type)) {
		data->sge_type = SLI4_SGE_TYPE_SKIP;
	}

	data->buffer_address_high = ocs_addr32_hi(addr);
	data->buffer_address_low  = ocs_addr32_lo(addr);

	/*
	 * Always assume this is the last entry and mark as such.
	 * If this is not the first entry unset the "last SGE"
	 * indication for the previous entry
	 */
	data->last = TRUE;
	if (io->n_sge) {
		data[-1].last = FALSE;
	}

	io->n_sge++;

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup io
 * @brief Abort all previously-started IO's.
 *
 * @param hal Hardware context.
 *
 * @return Returns None.
 */
void
ocs_hal_io_abort_all(ocs_hal_t *hal)
{
	ocs_hal_io_t *io_to_abort	= NULL;
	ocs_hal_io_t *next_io		= NULL;

	ocs_list_foreach_safe(&hal->io_inuse, io_to_abort, next_io) {
		ocs_hal_io_abort(hal, io_to_abort, TRUE, NULL, NULL);
	}
}

/**
 * @ingroup io
 * @brief Abort a previously-started IO.
 *
 * @param hal Hardware context.
 * @param io_to_abort The HAL IO to abort.
 * @param send_abts Boolean to have the hardware automatically generate an ABTS.
 * @param cb Function call upon completion of the abort (may be NULL).
 * @param arg Argument to pass to abort completion function.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_io_abort(ocs_hal_t *hal, ocs_hal_io_t *io_to_abort, uint32_t send_abts, void *cb, void *arg)
{
	hal_wq_callback_t *wqcb = NULL;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	if (!hal || !io_to_abort) {
		ocs_log_err(hal ? hal->os : NULL, "Bad parameter(s): hal=%p, io=%p\n", hal, io_to_abort);
		return OCS_HAL_RTN_ERROR;
	}

	if (OCS_HAL_STATE_ACTIVE != hal->state) {
		ocs_log_err(hal->os, "Cannot send IO abort, HAL state=%d\n", hal->state);
		return OCS_HAL_RTN_ERROR;
	}

	/* Take a reference on the IO being aborted */
	if (!ocs_ref_get_unless_zero(&io_to_abort->ref)) {
		ocs_log_err(hal->os, "IO is not active, io=%p xri=%#x tag=%#x\n",
			io_to_abort, io_to_abort->indicator, io_to_abort->reqtag);
		return OCS_HAL_RTN_IO_NOT_ACTIVE;
	}

	/* Take the HAL IO lock to protect the abort_issued flag */
	ocs_lock(&io_to_abort->lock);

	/* Check if the IO was already (being) aborted */
	if (io_to_abort->abort_issued) {
		ocs_log_info(hal->os, "IO is already being aborted, io=%p xri=%#x tag=%#x\n",
				io_to_abort, io_to_abort->indicator, io_to_abort->reqtag);
		rc = OCS_HAL_RTN_IO_ABORT_IN_PROGRESS;
		goto exit_hal_io_abort;
	}

	/* Non-port owned XRI's must have a valid WQ reference */
	if (!ocs_hal_io_port_owned(io_to_abort) && (io_to_abort->wq == NULL)) {
		ocs_log_err(hal->os, "IO is not active on WQ (%p), io=%p xri=%#x tag=%#x port_owned=%d\n",
				io_to_abort->wq, io_to_abort, io_to_abort->indicator,
				io_to_abort->reqtag, io_to_abort->is_port_owned);
		rc = OCS_HAL_RTN_IO_NOT_ACTIVE;
		goto exit_hal_io_abort;
	}

	/* Allocate a request tag for the abort portion of this IO */
	wqcb = ocs_hal_reqtag_alloc(hal, ocs_hal_wq_process_abort, io_to_abort);
	if (!wqcb) {
		ocs_log_err(hal->os, "Can't allocate the abort request tag, io=%p xri=%#x tag=%#x\n",
				io_to_abort, io_to_abort->indicator, io_to_abort->reqtag);
		rc = OCS_HAL_RTN_NO_RESOURCES;
		goto exit_hal_io_abort;
	}

	io_to_abort->abort_reqtag = wqcb->instance_index;
	io_to_abort->abort_issued = TRUE;
	io_to_abort->abort_done = cb;
	io_to_abort->abort_arg = arg;

	/*
	 * If the wqe is on the pending list, then set this wqe to
	 * be aborted when the IO's wqe is removed from the list.
	 */
	if (io_to_abort->wq) {
		sli_queue_lock(io_to_abort->wq->queue);
		if (ocs_list_on_list(&io_to_abort->wqe.link)) {
			io_to_abort->wqe.abort_wqe_submit_needed = 1;
			io_to_abort->wqe.send_abts = send_abts;
			io_to_abort->wqe.id = io_to_abort->indicator;
			io_to_abort->wqe.abort_reqtag = io_to_abort->abort_reqtag;
			sli_queue_unlock(io_to_abort->wq->queue);
			rc = OCS_HAL_RTN_SUCCESS;
			goto exit_hal_io_abort;
		}
		sli_queue_unlock(io_to_abort->wq->queue);
	}

	if (sli_abort_wqe(&hal->sli, io_to_abort->wqe.wqebuf, hal->sli.config.wqe_size, SLI_ABORT_XRI,
			send_abts, io_to_abort->indicator, 0, io_to_abort->abort_reqtag, SLI4_CQ_DEFAULT)) {
		ocs_log_err(hal->os, "ABORT WQE unknown error, io=%p xri=%#x tag=%#x\n",
			io_to_abort, io_to_abort->indicator, io_to_abort->reqtag);
		rc = OCS_HAL_RTN_ERROR;
	}

	if (OCS_HAL_RTN_SUCCESS == rc) {
		if (!io_to_abort->wq) {
			io_to_abort->wq = ocs_hal_queue_next_wq(hal, io_to_abort);
			ocs_hal_assert(io_to_abort->wq);
		}

		/*
		 * We can't abort an ABORT_WQE. So, skip adding this to the timed wqe list.
		 *
		 * ABORT_WQE does not actually utilize an XRI on the port. Therefore, keep
		 * xbusy as-is to track the exchange's state, not the ABORT_WQE's state.
		 */
		rc = hal_wq_write(io_to_abort->wq, &io_to_abort->wqe);
		if (rc >= 0) {
			/* Non-negative return value is success */
			rc = OCS_HAL_RTN_SUCCESS;
		} else {
			ocs_log_err(hal->os, "ABORT WQE submit error, io=%p xri=%#x tag=%#x\n",
				io_to_abort, io_to_abort->indicator, io_to_abort->reqtag);
			rc = OCS_HAL_RTN_ERROR;
		}
	}

	if (OCS_HAL_RTN_SUCCESS != rc) {
		io_to_abort->abort_issued = FALSE;
		io_to_abort->abort_reqtag = UINT32_MAX;
		ocs_hal_reqtag_free(hal, wqcb);
	}

exit_hal_io_abort:
	ocs_unlock(&io_to_abort->lock);
	if (OCS_HAL_RTN_SUCCESS != rc)
		ocs_ref_put(&io_to_abort->ref); /* ocs_ref_get(): same function */

	return rc;
}

/**
 * @ingroup io
 * @brief Return the OX_ID/RX_ID of the IO.
 *
 * @param hal Hardware context.
 * @param io HAL IO object.
 *
 * @return Returns X_ID on success, or -1 on failure.
 */
int32_t
ocs_hal_io_get_xid(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	if (!hal || !io) {
		ocs_log_err(hal ? hal->os : NULL, "bad parameter hal=%p io=%p\n", hal, io);
		return -1;
	}

	return io->indicator;
}


typedef struct ocs_hal_fw_write_cb_arg {
	ocs_hal_fw_cb_t cb;
	void *arg;
} ocs_hal_fw_write_cb_arg_t;

typedef struct ocs_hal_sfp_cb_arg {
	ocs_hal_sfp_cb_t cb;
	void *arg;
	ocs_dma_t payload;
} ocs_hal_sfp_cb_arg_t;

typedef struct ocs_hal_temp_cb_arg {
	ocs_hal_temp_cb_t cb;
	void *arg;
} ocs_hal_temp_cb_arg_t;

typedef struct ocs_hal_link_stat_cb_arg {
	ocs_hal_link_stat_cb_t cb;
	void *arg;
} ocs_hal_link_stat_cb_arg_t;

typedef struct ocs_hal_host_stat_cb_arg {
	ocs_hal_host_stat_cb_t cb;
	void *arg;
} ocs_hal_host_stat_cb_arg_t;

typedef struct ocs_hal_parity_stat_cb_arg {
	ocs_hal_parity_stat_cb_t cb;
	void *arg;
} ocs_hal_parity_stat_cb_arg_t;

typedef struct ocs_hal_dump_get_cb_arg {
	ocs_hal_dump_get_cb_t cb;
	void *arg;
	void *mbox_cmd;
} ocs_hal_dump_get_cb_arg_t;

typedef struct ocs_hal_dump_clear_cb_arg {
	ocs_hal_dump_clear_cb_t cb;
	void *arg;
	void *mbox_cmd;
} ocs_hal_dump_clear_cb_arg_t;

/**
 * @brief Write a portion of a firmware image to the device.
 *
 * @par Description
 * Calls the correct firmware write function based on the device type.
 *
 * @param hal Hardware context.
 * @param dma DMA structure containing the firmware image chunk.
 * @param size Size of the firmware image chunk.
 * @param offset Offset, in bytes, from the beginning of the firmware image.
 * @param last True if this is the last chunk of the image.
 * Causes the image to be committed to flash.
 * @param cb Pointer to a callback function that is called when the command completes.
 * The callback function prototype is
 * <tt>void cb(int32_t status, uint32_t bytes_written, void *arg)</tt>.
 * @param arg Pointer to be passed to the callback function.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_firmware_write(ocs_hal_t *hal, ocs_dma_t *dma, uint32_t size, uint32_t offset, int last, ocs_hal_fw_cb_t cb, void *arg)
{
	if ((hal->sli.if_type == SLI4_IF_TYPE_LANCER_FC_ETH) ||
	    (hal->sli.if_type == SLI4_IF_TYPE_LANCER_G7)) {
		return ocs_hal_firmware_write_lancer(hal, dma, size, offset, last, cb, arg);
	} else {
		// TODO:  Write firmware_write for BE3/Skyhawk
		return -1;
	}
}

/**
 * @brief Write a portion of a firmware image to the Emulex XE201 ASIC (Lancer).
 *
 * @par Description
 * Creates a SLI_CONFIG mailbox command, fills it with the correct values to write a
 * firmware image chunk, and then sends the command with ocs_hal_command(). On completion,
 * the callback function ocs_hal_fw_write_cb() gets called to free the mailbox
 * and to signal the caller that the write has completed.
 *
 * @param hal Hardware context.
 * @param dma DMA structure containing the firmware image chunk.
 * @param size Size of the firmware image chunk.
 * @param offset Offset, in bytes, from the beginning of the firmware image.
 * @param last True if this is the last chunk of the image. Causes the image to be committed to flash.
 * @param cb Pointer to a callback function that is called when the command completes.
 * The callback function prototype is
 * <tt>void cb(int32_t status, uint32_t bytes_written, void *arg)</tt>.
 * @param arg Pointer to be passed to the callback function.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_firmware_write_lancer(ocs_hal_t *hal, ocs_dma_t *dma, uint32_t size, uint32_t offset, int last, ocs_hal_fw_cb_t cb, void *arg)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	uint8_t *mbxdata;
	ocs_hal_fw_write_cb_arg_t *cb_arg;
	int noc=0;	// No Commit bit - set to 1 for testing

	if ((SLI4_IF_TYPE_LANCER_FC_ETH != sli_get_if_type(&hal->sli)) &&
	     (SLI4_IF_TYPE_LANCER_G7 != sli_get_if_type(&hal->sli))) {
		ocs_log_test(hal->os, "Function only supported for I/F type 2/6\n");
		return OCS_HAL_RTN_ERROR;
	}

	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_fw_write_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;

	/* Send the HAL command */
	sli_cmd_common_write_object(&hal->sli, mbxdata, SLI4_BMBX_SIZE, noc, last, size, offset, "/prg/", dma);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_cb_fw_write, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

/**
 * @brief Called when the WRITE OBJECT command completes.
 *
 * @par Description
 * Get the number of bytes actually written out of the response, free the mailbox
 * that was malloc'd by ocs_hal_firmware_write(),
 * then call the callback and pass the status and bytes written.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 * The callback function prototype is <tt>void cb(int32_t status, uint32_t bytes_written)</tt>.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_cb_fw_write(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_write_object_t *wr_obj_rsp = (sli4_res_common_write_object_t *)mbox_rsp->payload.embed;
	ocs_hal_fw_write_cb_arg_t *cb_arg = arg;
	uint32_t bytes_written;
	uint32_t change_status;

	if (wr_obj_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &wr_obj_rsp->hdr);

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_cb_fw_write_done;

	bytes_written = wr_obj_rsp->actual_write_length;
	change_status = wr_obj_rsp->change_status;

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (wr_obj_rsp->hdr.status)
			status = wr_obj_rsp->hdr.status;
	}

	cb_arg->cb(status, wr_obj_rsp->hdr.additional_status, bytes_written, change_status, cb_arg->arg);

ocs_hal_cb_fw_write_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @brief Called when the READ_TRANSCEIVER_DATA command completes.
 *
 * @par Description
 * Get the number of bytes read out of the response, free the mailbox that was malloc'd
 * by ocs_hal_get_sfp(), then call the callback and pass the status and bytes written.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 * The callback function prototype is
 * <tt>void cb(int32_t status, uint32_t bytes_written, uint32_t *data, void *arg)</tt>.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_cb_sfp(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_read_transceiver_data_t *sli4_rsp = NULL;
	ocs_hal_sfp_cb_arg_t *cb_arg = arg;
	ocs_dma_t *payload = NULL;
	uint32_t bytes_written = 0;

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_cb_sfp_done;

	payload = &cb_arg->payload;
	sli4_rsp = (sli4_res_common_read_transceiver_data_t *)payload->virt;
	if (sli4_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &sli4_rsp->hdr);

	bytes_written = sli4_rsp->hdr.response_length;

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (sli4_rsp->hdr.status)
			status = sli4_rsp->hdr.status;
	}

	cb_arg->cb(hal->os, status, bytes_written, sli4_rsp->page_data, cb_arg->arg);

ocs_hal_cb_sfp_done:
	if (cb_arg) {
		ocs_dma_free(hal->os, &cb_arg->payload);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
	}

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @ingroup io
 * @brief Function to retrieve the SFP information.
 *
 * @param hal Hardware context.
 * @param page The page of SFP data to retrieve (0xa0 or 0xa2).
 * @param cb Function call upon completion of sending the data (may be NULL).
 * @param arg Argument to pass to IO completion function.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS, OCS_HAL_RTN_ERROR, or OCS_HAL_RTN_NO_MEMORY.
 */
ocs_hal_rtn_e
ocs_hal_get_sfp(ocs_hal_t *hal, uint16_t page, ocs_hal_sfp_cb_t cb, void *arg)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	ocs_hal_sfp_cb_arg_t *cb_arg;
	uint8_t *mbxdata;

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* cb_arg holds the data that will be passed to the callback on completion */
	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_sfp_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;

	/* payload holds the non-embedded portion */
	if (ocs_dma_alloc(hal->os, &cb_arg->payload, sizeof(sli4_res_common_read_transceiver_data_t),
			  OCS_MIN_DMA_ALIGNMENT)) {
		ocs_log_err(hal->os, "Failed to allocate DMA buffer\n");
		ocs_free(hal->os, cb_arg, sizeof(ocs_hal_sfp_cb_arg_t));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* Send the HAL command */
	sli_cmd_common_read_transceiver_data(&hal->sli, mbxdata, SLI4_BMBX_SIZE, page, &cb_arg->payload);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_cb_sfp, cb_arg);
	if (rc) {
		ocs_dma_free(hal->os, &cb_arg->payload);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

/**
 * @brief Function to retrieve the temperature information.
 *
 * @param hal Hardware context.
 * @param cb Function call upon completion of sending the data (may be NULL).
 * @param arg Argument to pass to IO completion function.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS, OCS_HAL_RTN_ERROR, or OCS_HAL_RTN_NO_MEMORY.
 */
ocs_hal_rtn_e
ocs_hal_get_temperature(ocs_hal_t *hal, ocs_hal_temp_cb_t cb, void *arg)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	ocs_hal_temp_cb_arg_t *cb_arg;
	uint8_t *mbxdata;

	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_temp_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;

	/* Send the HAL command */
	sli_cmd_dump_type4(&hal->sli, mbxdata, SLI4_BMBX_SIZE, SLI4_WKI_TAG_SAT_TEM);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_cb_temp, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

/**
 * @brief Called when the DUMP command completes.
 *
 * @par Description
 * Get the temperature data out of the response, free the mailbox that was malloc'd
 * by ocs_hal_get_temperature(), then call the callback and pass the status and data.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 * The callback function prototype is defined by ocs_hal_temp_cb_t.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_cb_temp(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_dump4_t *mbox_rsp = (sli4_cmd_dump4_t *)mqe;
	ocs_hal_temp_cb_arg_t *cb_arg = arg;
	uint32_t curr_temp = mbox_rsp->resp_data[0]; /* word 5 */
	uint32_t crit_temp_thrshld = mbox_rsp->resp_data[1]; /* word 6*/
	uint32_t warn_temp_thrshld = mbox_rsp->resp_data[2]; /* word 7 */
	uint32_t norm_temp_thrshld = mbox_rsp->resp_data[3]; /* word 8 */
	uint32_t fan_off_thrshld = mbox_rsp->resp_data[4];   /* word 9 */
	uint32_t fan_on_thrshld = mbox_rsp->resp_data[5];    /* word 10 */

	ocs_log_debug(hal->os, "CB status %08x, curr_temp=%d crit_temp_thr=%d "\
		      "warn_temp_thr %d norm_temp_thr %d foff_thr %d fon_thr %d\n",
		      status, curr_temp, crit_temp_thrshld, warn_temp_thrshld,
		      norm_temp_thrshld, fan_off_thrshld, fan_on_thrshld);

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_cb_temp_done;

	if ((0 == status) && mbox_rsp->hdr.status)
		status = mbox_rsp->hdr.status;

	cb_arg->cb(status, curr_temp, crit_temp_thrshld, warn_temp_thrshld,
		norm_temp_thrshld, fan_off_thrshld, fan_on_thrshld, cb_arg->arg);

ocs_hal_cb_temp_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @brief Function to retrieve the link statistics.
 *
 * @param hal Hardware context.
 * @param req_ext_counters If TRUE, then the extended counters will be requested.
 * @param clear_overflow_flags If TRUE, then overflow flags will be cleared.
 * @param clear_all_counters If TRUE, the counters will be cleared.
 * @param cb Function call upon completion of sending the data (may be NULL).
 * @param arg Argument to pass to IO completion function.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS, OCS_HAL_RTN_ERROR, or OCS_HAL_RTN_NO_MEMORY.
 */
ocs_hal_rtn_e
ocs_hal_get_link_stats(ocs_hal_t *hal,
		       uint8_t req_ext_counters,
		       uint8_t clear_overflow_flags,
		       uint8_t clear_all_counters,
		       ocs_hal_link_stat_cb_t cb,
		       void *arg)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	ocs_hal_link_stat_cb_arg_t *cb_arg;
	uint8_t *mbxdata;

	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_link_stat_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;

	/* Send the HAL command */
	sli_cmd_read_link_stats(&hal->sli, mbxdata, SLI4_BMBX_SIZE, req_ext_counters,
				clear_overflow_flags, clear_all_counters);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_cb_link_stat, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

/**
 * @brief Called when the READ_LINK_STAT command completes.
 *
 * @par Description
 * Get the counters out of the response, free the mailbox that was malloc'd
 * by ocs_hal_get_link_stats(), then call the callback and pass the status and data.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 * The callback function prototype is defined by ocs_hal_link_stat_cb_t.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_cb_link_stat(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_read_link_stats_t *mbox_rsp = (sli4_cmd_read_link_stats_t *)mqe;
	ocs_hal_link_stat_cb_arg_t *cb_arg = arg;
	ocs_hal_link_stat_counts_t counts[OCS_HAL_LINK_STAT_MAX];
	uint32_t num_counters = (mbox_rsp->gec ? 20 : 13);

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_cb_link_stat_done;

	ocs_memset(counts, 0, sizeof(ocs_hal_link_stat_counts_t) * OCS_HAL_LINK_STAT_MAX);

	counts[OCS_HAL_LINK_STAT_LINK_FAILURE_COUNT].overflow = mbox_rsp->w02of;
	counts[OCS_HAL_LINK_STAT_LOSS_OF_SYNC_COUNT].overflow = mbox_rsp->w03of;
	counts[OCS_HAL_LINK_STAT_LOSS_OF_SIGNAL_COUNT].overflow = mbox_rsp->w04of;
	counts[OCS_HAL_LINK_STAT_PRIMITIVE_SEQ_COUNT].overflow = mbox_rsp->w05of;
	counts[OCS_HAL_LINK_STAT_INVALID_XMIT_WORD_COUNT].overflow = mbox_rsp->w06of;
	counts[OCS_HAL_LINK_STAT_CRC_COUNT].overflow = mbox_rsp->w07of;
	counts[OCS_HAL_LINK_STAT_PRIMITIVE_SEQ_TIMEOUT_COUNT].overflow = mbox_rsp->w08of;
	counts[OCS_HAL_LINK_STAT_ELASTIC_BUFFER_OVERRUN_COUNT].overflow = mbox_rsp->w09of;
	counts[OCS_HAL_LINK_STAT_ARB_TIMEOUT_COUNT].overflow = mbox_rsp->w10of;
	counts[OCS_HAL_LINK_STAT_ADVERTISED_RCV_B2B_CREDIT].overflow = mbox_rsp->w11of;
	counts[OCS_HAL_LINK_STAT_CURR_RCV_B2B_CREDIT].overflow = mbox_rsp->w12of;
	counts[OCS_HAL_LINK_STAT_ADVERTISED_XMIT_B2B_CREDIT].overflow = mbox_rsp->w13of;
	counts[OCS_HAL_LINK_STAT_CURR_XMIT_B2B_CREDIT].overflow = mbox_rsp->w14of;
	counts[OCS_HAL_LINK_STAT_RCV_EOFA_COUNT].overflow = mbox_rsp->w15of;
	counts[OCS_HAL_LINK_STAT_RCV_EOFDTI_COUNT].overflow = mbox_rsp->w16of;
	counts[OCS_HAL_LINK_STAT_RCV_EOFNI_COUNT].overflow = mbox_rsp->w17of;
	counts[OCS_HAL_LINK_STAT_RCV_SOFF_COUNT].overflow = mbox_rsp->w18of;
	counts[OCS_HAL_LINK_STAT_RCV_DROPPED_NO_AER_COUNT].overflow = mbox_rsp->w19of;
	counts[OCS_HAL_LINK_STAT_RCV_DROPPED_NO_RPI_COUNT].overflow = mbox_rsp->w20of;
	counts[OCS_HAL_LINK_STAT_RCV_DROPPED_NO_XRI_COUNT].overflow = mbox_rsp->w21of;

	counts[OCS_HAL_LINK_STAT_LINK_FAILURE_COUNT].counter = mbox_rsp->link_failure_error_count;
	counts[OCS_HAL_LINK_STAT_LOSS_OF_SYNC_COUNT].counter = mbox_rsp->loss_of_sync_error_count;
	counts[OCS_HAL_LINK_STAT_LOSS_OF_SIGNAL_COUNT].counter = mbox_rsp->loss_of_signal_error_count;
	counts[OCS_HAL_LINK_STAT_PRIMITIVE_SEQ_COUNT].counter = mbox_rsp->primitive_sequence_error_count;
	counts[OCS_HAL_LINK_STAT_INVALID_XMIT_WORD_COUNT].counter = mbox_rsp->invalid_transmission_word_error_count;
	counts[OCS_HAL_LINK_STAT_CRC_COUNT].counter = mbox_rsp->crc_error_count;
	counts[OCS_HAL_LINK_STAT_PRIMITIVE_SEQ_TIMEOUT_COUNT].counter = mbox_rsp->primitive_sequence_event_timeout_count;
	counts[OCS_HAL_LINK_STAT_ELASTIC_BUFFER_OVERRUN_COUNT].counter = mbox_rsp->elastic_buffer_overrun_error_count;
	counts[OCS_HAL_LINK_STAT_ARB_TIMEOUT_COUNT].counter = mbox_rsp->arbitration_fc_al_timout_count;
	counts[OCS_HAL_LINK_STAT_ADVERTISED_RCV_B2B_CREDIT].counter = mbox_rsp->advertised_receive_bufftor_to_buffer_credit;
	counts[OCS_HAL_LINK_STAT_CURR_RCV_B2B_CREDIT].counter = mbox_rsp->current_receive_buffer_to_buffer_credit;
	counts[OCS_HAL_LINK_STAT_ADVERTISED_XMIT_B2B_CREDIT].counter = mbox_rsp->advertised_transmit_buffer_to_buffer_credit;
	counts[OCS_HAL_LINK_STAT_CURR_XMIT_B2B_CREDIT].counter = mbox_rsp->current_transmit_buffer_to_buffer_credit;
	counts[OCS_HAL_LINK_STAT_RCV_EOFA_COUNT].counter = mbox_rsp->received_eofa_count;
	counts[OCS_HAL_LINK_STAT_RCV_EOFDTI_COUNT].counter = mbox_rsp->received_eofdti_count;
	counts[OCS_HAL_LINK_STAT_RCV_EOFNI_COUNT].counter = mbox_rsp->received_eofni_count;
	counts[OCS_HAL_LINK_STAT_RCV_SOFF_COUNT].counter = mbox_rsp->received_soff_count;
	counts[OCS_HAL_LINK_STAT_RCV_DROPPED_NO_AER_COUNT].counter = mbox_rsp->received_dropped_no_aer_count;
	counts[OCS_HAL_LINK_STAT_RCV_DROPPED_NO_RPI_COUNT].counter = mbox_rsp->received_dropped_no_available_rpi_resources_count;
	counts[OCS_HAL_LINK_STAT_RCV_DROPPED_NO_XRI_COUNT].counter = mbox_rsp->received_dropped_no_available_xri_resources_count;
	counts[OCS_HAL_LINK_STAT_LRR_COUNT_LOCAL].counter = mbox_rsp->lrr_count_local;
	counts[OCS_HAL_LINK_STAT_LR_COUNT_REMOTE].counter = mbox_rsp->lr_count_remote;

	if ((0 == status) && mbox_rsp->hdr.status)
		status = mbox_rsp->hdr.status;

	cb_arg->cb(status, num_counters, counts, cb_arg->arg);

ocs_hal_cb_link_stat_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @brief Function to retrieve the link and host statistics.
 *
 * @param hal Hardware context.
 * @param cc clear counters, if TRUE all counters will be cleared.
 * @param cb Function call upon completion of receiving the data.
 * @param arg Argument to pass to pointer fc hosts statistics structure.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS, OCS_HAL_RTN_ERROR, or OCS_HAL_RTN_NO_MEMORY.
 */
ocs_hal_rtn_e
ocs_hal_get_host_stats(ocs_hal_t *hal, uint8_t cc, ocs_hal_host_stat_cb_t cb, void *arg)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	ocs_hal_host_stat_cb_arg_t *cb_arg;
	uint8_t *mbxdata;

	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_host_stat_cb_arg_t),
			    OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;

	/* Send the HAL command to get the host stats */
	sli_cmd_read_status(&hal->sli, mbxdata, SLI4_BMBX_SIZE, cc);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_cb_host_stat, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

/**
 * @brief Called when the READ_STATUS command completes.
 *
 * @par Description
 * Get the counters out of the response, free the mailbox that was malloc'd
 * by ocs_hal_get_host_stats(), then call the callback and pass
 * the status and data.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 * The callback function prototype is defined by
 * ocs_hal_host_stat_cb_t.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_cb_host_stat(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_read_status_t *mbox_rsp = (sli4_cmd_read_status_t *)mqe;
	ocs_hal_host_stat_cb_arg_t *cb_arg = arg;
	ocs_hal_host_stat_counts_t counts[OCS_HAL_HOST_STAT_MAX];
	uint32_t num_counters = OCS_HAL_HOST_STAT_MAX;

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_cb_host_stat_done;

	ocs_memset(counts, 0, sizeof(ocs_hal_host_stat_counts_t) * OCS_HAL_HOST_STAT_MAX);

	counts[OCS_HAL_HOST_STAT_TX_KBYTE_COUNT].counter = mbox_rsp->transmit_kbyte_count;
	counts[OCS_HAL_HOST_STAT_RX_KBYTE_COUNT].counter = mbox_rsp->receive_kbyte_count;
	counts[OCS_HAL_HOST_STAT_TX_FRAME_COUNT].counter = mbox_rsp->transmit_frame_count;
	counts[OCS_HAL_HOST_STAT_RX_FRAME_COUNT].counter = mbox_rsp->receive_frame_count;
	counts[OCS_HAL_HOST_STAT_TX_SEQ_COUNT].counter = mbox_rsp->transmit_sequence_count;
	counts[OCS_HAL_HOST_STAT_RX_SEQ_COUNT].counter = mbox_rsp->receive_sequence_count;
	counts[OCS_HAL_HOST_STAT_TOTAL_EXCH_ORIG].counter = mbox_rsp->total_exchanges_originator;
	counts[OCS_HAL_HOST_STAT_TOTAL_EXCH_RESP].counter = mbox_rsp->total_exchanges_responder;
	counts[OCS_HAL_HOSY_STAT_RX_P_BSY_COUNT].counter = mbox_rsp->receive_p_bsy_count;
	counts[OCS_HAL_HOST_STAT_RX_F_BSY_COUNT].counter = mbox_rsp->receive_f_bsy_count;
	counts[OCS_HAL_HOST_STAT_DROP_FRM_DUE_TO_NO_RQ_BUF_COUNT].counter = mbox_rsp->dropped_frames_due_to_no_rq_buffer_count;
	counts[OCS_HAL_HOST_STAT_EMPTY_RQ_TIMEOUT_COUNT].counter = mbox_rsp->empty_rq_timeout_count;
	counts[OCS_HAL_HOST_STAT_DROP_FRM_DUE_TO_NO_XRI_COUNT].counter = mbox_rsp->dropped_frames_due_to_no_xri_count;
	counts[OCS_HAL_HOST_STAT_EMPTY_XRI_POOL_COUNT].counter = mbox_rsp->empty_xri_pool_count;

	if ((0 == status) && mbox_rsp->hdr.status)
		status = mbox_rsp->hdr.status;

	cb_arg->cb(status, num_counters, counts, cb_arg->arg);

ocs_hal_cb_host_stat_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @brief HAL link configuration enum to the CLP string value mapping.
 *
 * This structure provides a mapping from the ocs_hal_linkcfg_e
 * enum (enum exposed for the OCS_HAL_PORT_SET_LINK_CONFIG port
 * control) to the CLP string that is used
 * in the DMTF_CLP_CMD mailbox command.
 */
typedef struct ocs_hal_linkcfg_map_s {
	ocs_hal_linkcfg_e linkcfg;
	const char *clp_str;
} ocs_hal_linkcfg_map_t;

/**
 * @brief Mapping from the HAL linkcfg enum to the CLP command value
 * string.
 */
static ocs_hal_linkcfg_map_t linkcfg_map[] = {
	{OCS_HAL_LINKCFG_4X10G, "ELX_4x10G"},
	{OCS_HAL_LINKCFG_1X40G, "ELX_1x40G"},
	{OCS_HAL_LINKCFG_2X16G, "ELX_2x16G"},
	{OCS_HAL_LINKCFG_4X8G, "ELX_4x8G"},
	{OCS_HAL_LINKCFG_4X1G, "ELX_4x1G"},
	{OCS_HAL_LINKCFG_2X10G, "ELX_2x10G"},
	{OCS_HAL_LINKCFG_2X10G_2X8G, "ELX_2x10G_2x8G"}};

/**
 * @brief HAL link configuration enum to Skyhawk link config ID mapping.
 *
 * This structure provides a mapping from the ocs_hal_linkcfg_e
 * enum (enum exposed for the OCS_HAL_PORT_SET_LINK_CONFIG port
 * control) to the link config ID numbers used by Skyhawk
 */
typedef struct ocs_hal_skyhawk_linkcfg_map_s {
	ocs_hal_linkcfg_e linkcfg;
	uint32_t	config_id;
} ocs_hal_skyhawk_linkcfg_map_t;

/**
 * @brief Mapping from the HAL linkcfg enum to the Skyhawk link config IDs
 */
static ocs_hal_skyhawk_linkcfg_map_t skyhawk_linkcfg_map[] = {
	{OCS_HAL_LINKCFG_4X10G, 0x0a},
	{OCS_HAL_LINKCFG_1X40G, 0x09},
};

/**
 * @brief Helper function for getting the HAL linkcfg enum from the CLP
 * string value
 *
 * @param clp_str CLP string value from OEMELX_LinkConfig.
 *
 * @return Returns the HAL linkcfg enum corresponding to clp_str.
 */
static ocs_hal_linkcfg_e
ocs_hal_linkcfg_from_clp(const char *clp_str)
{
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(linkcfg_map); i++) {
		if (ocs_strncmp(linkcfg_map[i].clp_str, clp_str, ocs_strlen(clp_str)) == 0) {
			return linkcfg_map[i].linkcfg;
		}
	}
	return OCS_HAL_LINKCFG_NA;
}

/**
 * @brief Helper function for getting the CLP string value from the HAL
 * linkcfg enum.
 *
 * @param linkcfg HAL linkcfg enum.
 *
 * @return Returns the OEMELX_LinkConfig CLP string value corresponding to
 * given linkcfg.
 */
static const char *
ocs_hal_clp_from_linkcfg(ocs_hal_linkcfg_e linkcfg)
{
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(linkcfg_map); i++) {
		if (linkcfg_map[i].linkcfg == linkcfg) {
			return linkcfg_map[i].clp_str;
		}
	}
	return NULL;
}

/**
 * @brief Helper function for getting a Skyhawk link config ID from the HAL
 * linkcfg enum.
 *
 * @param linkcfg HAL linkcfg enum.
 *
 * @return Returns the Skyhawk link config ID corresponding to
 * given linkcfg.
 */
static uint32_t
ocs_hal_config_id_from_linkcfg(ocs_hal_linkcfg_e linkcfg)
{
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(skyhawk_linkcfg_map); i++) {
		if (skyhawk_linkcfg_map[i].linkcfg == linkcfg) {
			return skyhawk_linkcfg_map[i].config_id;
		}
	}
	return 0;
}

/**
 * @brief Helper function for getting the HAL linkcfg enum from a
 * Skyhawk config ID.
 *
 * @param config_id Skyhawk link config ID.
 *
 * @return Returns the HAL linkcfg enum corresponding to config_id.
 */
static ocs_hal_linkcfg_e
ocs_hal_linkcfg_from_config_id(const uint32_t config_id)
{
	uint32_t i;
	for (i = 0; i < ARRAY_SIZE(skyhawk_linkcfg_map); i++) {
		if (skyhawk_linkcfg_map[i].config_id == config_id) {
			return skyhawk_linkcfg_map[i].linkcfg;
		}
	}
	return OCS_HAL_LINKCFG_NA;
}

/**
 * @brief Link configuration callback argument.
 */
typedef struct ocs_hal_linkcfg_cb_arg_s {
	ocs_hal_port_control_cb_t cb;
	void *arg;
	uint32_t opts;
	int32_t status;
	ocs_dma_t dma_cmd;
	ocs_dma_t dma_resp;
	uint32_t result_len;
} ocs_hal_linkcfg_cb_arg_t;

/**
 * @brief Set link configuration.
 *
 * @param hal Hardware context.
 * @param value Link configuration enum to which the link configuration is
 * set.
 * @param opts Mailbox command options (OCS_CMD_NOWAIT/POLL).
 * @param cb Callback function to invoke following mbx command.
 * @param arg Callback argument.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_set_linkcfg(ocs_hal_t *hal, ocs_hal_linkcfg_e value, uint32_t opts, ocs_hal_port_control_cb_t cb, void *arg)
{
	if (!sli_link_is_configurable(&hal->sli)) {
		ocs_log_debug(hal->os, "Function not supported\n");
		return OCS_HAL_RTN_ERROR;
	}

	if (SLI4_IF_TYPE_LANCER_FC_ETH == sli_get_if_type(&hal->sli)) {
		return ocs_hal_set_linkcfg_lancer(hal, value, opts, cb, arg);
	} else if ((SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(&hal->sli)) ||
		   (SLI4_IF_TYPE_BE3_SKH_VF == sli_get_if_type(&hal->sli))) {
		return ocs_hal_set_linkcfg_skyhawk(hal, value, opts, cb, arg);
	} else {
		ocs_log_test(hal->os, "Function not supported for this IF_TYPE\n");
		return OCS_HAL_RTN_ERROR;
	}
}

/**
 * @brief Set link configuration for Lancer
 *
 * @param hal Hardware context.
 * @param value Link configuration enum to which the link configuration is
 * set.
 * @param opts Mailbox command options (OCS_CMD_NOWAIT/POLL).
 * @param cb Callback function to invoke following mbx command.
 * @param arg Callback argument.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_set_linkcfg_lancer(ocs_hal_t *hal, ocs_hal_linkcfg_e value, uint32_t opts, ocs_hal_port_control_cb_t cb, void *arg)
{
	char cmd[OCS_HAL_DMTF_CLP_CMD_MAX];
	ocs_hal_linkcfg_cb_arg_t *cb_arg;
	const char *value_str = NULL;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* translate ocs_hal_linkcfg_e to CLP string */
	value_str = ocs_hal_clp_from_linkcfg(value);

	/* allocate memory for callback argument */
	cb_arg = ocs_malloc(hal->os, sizeof(*cb_arg), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	ocs_snprintf(cmd, OCS_HAL_DMTF_CLP_CMD_MAX, "set / OEMELX_LinkConfig=%s", value_str);
	/* allocate DMA for command */
	if (ocs_dma_alloc(hal->os, &cb_arg->dma_cmd, ocs_strlen(cmd)+1, 4096)) {
		ocs_log_err(hal->os, "malloc failed\n");
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		return OCS_HAL_RTN_NO_MEMORY;
	}

	ocs_memset(cb_arg->dma_cmd.virt, 0, ocs_strlen(cmd)+1);
	ocs_memcpy(cb_arg->dma_cmd.virt, cmd, ocs_strlen(cmd));

	/* allocate DMA for response */
	if (ocs_dma_alloc(hal->os, &cb_arg->dma_resp, OCS_HAL_DMTF_CLP_RSP_MAX, 4096)) {
		ocs_log_err(hal->os, "malloc failed\n");
		ocs_dma_free(hal->os, &cb_arg->dma_cmd);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;
	cb_arg->opts = opts;

	rc = ocs_hal_exec_dmtf_clp_cmd(hal, &cb_arg->dma_cmd, &cb_arg->dma_resp,
				       opts, ocs_hal_linkcfg_dmtf_clp_cb, cb_arg);
	if (opts == OCS_CMD_POLL || rc != OCS_HAL_RTN_SUCCESS) {
		/* if failed, or polling, free memory here; if success and not
		 * polling, will free in callback function
		 */
		if (rc)
			ocs_log_test(hal->os, "CLP cmd=\"%s\" failed\n", (char *)cb_arg->dma_cmd.virt);

		ocs_dma_free(hal->os, &cb_arg->dma_cmd);
		ocs_dma_free(hal->os, &cb_arg->dma_resp);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
	}

	return rc;
}

/**
 * @brief Callback for ocs_hal_set_linkcfg_skyhawk
 *
 * @param hal Hardware context.
 * @param status Status from the RECONFIG_GET_LINK_INFO command.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback argument.
 *
 * @return none
 */
static void
ocs_hal_set_active_link_config_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	ocs_hal_linkcfg_cb_arg_t *cb_arg = arg;

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_set_active_link_config_cb_done;

	if ((0 == status) && mbox_rsp->hdr.status)
		status = mbox_rsp->hdr.status;

	cb_arg->cb(status, 0, cb_arg->arg);

ocs_hal_set_active_link_config_cb_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
}

/**
 * @brief Set link configuration for a Skyhawk
 *
 * @param hal Hardware context.
 * @param value Link configuration enum to which the link configuration is
 * set.
 * @param opts Mailbox command options (OCS_CMD_NOWAIT/POLL).
 * @param cb Callback function to invoke following mbx command.
 * @param arg Callback argument.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_set_linkcfg_skyhawk(ocs_hal_t *hal, ocs_hal_linkcfg_e value, uint32_t opts, ocs_hal_port_control_cb_t cb, void *arg)
{
	uint8_t *mbxdata;
	ocs_hal_linkcfg_cb_arg_t *cb_arg;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint32_t config_id;

	config_id = ocs_hal_config_id_from_linkcfg(value);

	if (config_id == 0) {
		ocs_log_test(hal->os, "Link config %d not supported by Skyhawk\n", value);
		return OCS_HAL_RTN_ERROR;
	}

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* cb_arg holds the data that will be passed to the callback on completion */
	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_linkcfg_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;

	/* Send the HAL command */
	sli_cmd_common_set_reconfig_link_id(&hal->sli, mbxdata, SLI4_BMBX_SIZE, NULL, 0, config_id);
	rc = ocs_hal_command(hal, mbxdata, opts, ocs_hal_set_active_link_config_cb, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	} else if (opts == OCS_CMD_POLL) {
		/* If we're polling, call the callback here */
		ocs_hal_set_active_link_config_cb(hal, 0, mbxdata, cb_arg);
	}

	return rc;
}

/**
 * @brief Get link configuration.
 *
 * @param hal Hardware context.
 * @param opts Mailbox command options (OCS_CMD_NOWAIT/POLL).
 * @param cb Callback function to invoke following mbx command.
 * @param arg Callback argument.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_get_linkcfg(ocs_hal_t *hal, uint32_t opts, ocs_hal_port_control_cb_t cb, void *arg)
{
	if (!sli_link_is_configurable(&hal->sli)) {
		ocs_log_debug(hal->os, "Function not supported\n");
		return OCS_HAL_RTN_ERROR;
	}

	if ((SLI4_IF_TYPE_LANCER_FC_ETH == sli_get_if_type(&hal->sli)) ||
	    (SLI4_IF_TYPE_LANCER_G7 == sli_get_if_type(&hal->sli))) {
		return ocs_hal_get_linkcfg_lancer(hal, opts, cb, arg);
	} else if ((SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(&hal->sli)) ||
		   (SLI4_IF_TYPE_BE3_SKH_VF == sli_get_if_type(&hal->sli))) {
		return ocs_hal_get_linkcfg_skyhawk(hal, opts, cb, arg);
	} else {
		ocs_log_test(hal->os, "Function not supported for this IF_TYPE\n");
		return OCS_HAL_RTN_ERROR;
	}
}

/**
 * @brief Get link configuration for a Lancer
 *
 * @param hal Hardware context.
 * @param opts Mailbox command options (OCS_CMD_NOWAIT/POLL).
 * @param cb Callback function to invoke following mbx command.
 * @param arg Callback argument.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_get_linkcfg_lancer(ocs_hal_t *hal, uint32_t opts, ocs_hal_port_control_cb_t cb, void *arg)
{
	char cmd[OCS_HAL_DMTF_CLP_CMD_MAX];
	ocs_hal_linkcfg_cb_arg_t *cb_arg;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* allocate memory for callback argument */
	cb_arg = ocs_malloc(hal->os, sizeof(*cb_arg), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	ocs_snprintf(cmd, OCS_HAL_DMTF_CLP_CMD_MAX, "show / OEMELX_LinkConfig");

	/* allocate DMA for command  */
	if (ocs_dma_alloc(hal->os, &cb_arg->dma_cmd, ocs_strlen(cmd)+1, 4096)) {
		ocs_log_err(hal->os, "malloc failed\n");
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* copy CLP command to DMA command */
	ocs_memset(cb_arg->dma_cmd.virt, 0, ocs_strlen(cmd)+1);
	ocs_memcpy(cb_arg->dma_cmd.virt, cmd, ocs_strlen(cmd));

	/* allocate DMA for response */
	if (ocs_dma_alloc(hal->os, &cb_arg->dma_resp, OCS_HAL_DMTF_CLP_RSP_MAX, 4096)) {
		ocs_log_err(hal->os, "malloc failed\n");
		ocs_dma_free(hal->os, &cb_arg->dma_cmd);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;
	cb_arg->opts = opts;

	rc = ocs_hal_exec_dmtf_clp_cmd(hal, &cb_arg->dma_cmd, &cb_arg->dma_resp,
				       opts, ocs_hal_linkcfg_dmtf_clp_cb, cb_arg);
	if (opts == OCS_CMD_POLL || rc != OCS_HAL_RTN_SUCCESS) {
		/* if failed or polling, free memory here; if not polling and success,
		 * will free in callback function
		 */
		if (rc)
			ocs_log_test(hal->os, "CLP cmd=\"%s\" failed\n", (char *)cb_arg->dma_cmd.virt);

		ocs_dma_free(hal->os, &cb_arg->dma_cmd);
		ocs_dma_free(hal->os, &cb_arg->dma_resp);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
	}

	return rc;
}

/**
 * @brief Get the link configuration callback.
 *
 * @param hal Hardware context.
 * @param status Status from the RECONFIG_GET_LINK_INFO command.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback argument.
 *
 * @return none
 */
static void
ocs_hal_get_active_link_config_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_get_reconfig_link_info_t *sli4_rsp = NULL;
	ocs_hal_linkcfg_cb_arg_t *cb_arg = arg;
	ocs_hal_linkcfg_e value = OCS_HAL_LINKCFG_NA;

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_get_active_link_config_cb_done;

	sli4_rsp = (sli4_res_common_get_reconfig_link_info_t *)cb_arg->dma_cmd.virt;
	if (sli4_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &sli4_rsp->hdr);

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (sli4_rsp->hdr.status)
			status = sli4_rsp->hdr.status;
	}

	if (0 == status)
		value = ocs_hal_linkcfg_from_config_id(sli4_rsp->active_link_config_id);

	cb_arg->cb(status, value, cb_arg->arg);

ocs_hal_get_active_link_config_cb_done:
	if (cb_arg) {
		ocs_dma_free(hal->os, &cb_arg->dma_cmd);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
	}

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
}

/**
 * @brief Get link configuration for a Skyhawk.
 *
 * @param hal Hardware context.
 * @param opts Mailbox command options (OCS_CMD_NOWAIT/POLL).
 * @param cb Callback function to invoke following mbx command.
 * @param arg Callback argument.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_get_linkcfg_skyhawk(ocs_hal_t *hal, uint32_t opts, ocs_hal_port_control_cb_t cb, void *arg)
{
	uint8_t *mbxdata;
	ocs_hal_linkcfg_cb_arg_t *cb_arg;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* cb_arg holds the data that will be passed to the callback on completion */
	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_linkcfg_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;
	cb_arg->opts = opts;

	/* dma_mem holds the non-embedded portion */
	if (ocs_dma_alloc(hal->os, &cb_arg->dma_cmd, sizeof(sli4_res_common_get_reconfig_link_info_t), 4)) {
		ocs_log_err(hal->os, "Failed to allocate DMA buffer\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		ocs_free(hal->os, cb_arg, sizeof(ocs_hal_linkcfg_cb_arg_t));
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* Send the HAL command */
	sli_cmd_common_get_reconfig_link_info(&hal->sli, mbxdata, SLI4_BMBX_SIZE, &cb_arg->dma_cmd);
	rc = ocs_hal_command(hal, mbxdata, opts, ocs_hal_get_active_link_config_cb, cb_arg);
	if (rc) {
		ocs_dma_free(hal->os, &cb_arg->dma_cmd);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	} else if (opts == OCS_CMD_POLL) {
		/* If we're polling, call the callback here */
		ocs_hal_get_active_link_config_cb(hal, 0, mbxdata, cb_arg);
	}

	return rc;
}

/**
 * @brief Sets the DIF seed value.
 *
 * @param hal Hardware context.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_set_dif_seed(ocs_hal_t *hal)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];
	sli4_req_common_set_features_dif_seed_t seed_param;

	ocs_memset(&seed_param, 0, sizeof(seed_param));
	seed_param.seed = hal->config.dif_seed;

	/* send set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
		SLI4_SET_FEATURES_DIF_SEED, 4, (uint32_t *)&seed_param);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		return OCS_HAL_RTN_ERROR;

	ocs_log_debug(hal->os, "DIF seed set to 0x%x\n", hal->config.dif_seed);
	return rc;
}

/**
 * @brief Sets the DIF mode value.
 *
 * @param hal Hardware context.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_set_dif_mode(ocs_hal_t *hal)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];
	sli4_req_common_set_features_t10_pi_mem_model_t mode_param;

	ocs_memset(&mode_param, 0, sizeof(mode_param));
	mode_param.tmm = (hal->config.dif_mode == OCS_HAL_DIF_MODE_INLINE ? 0 : 1);

	/* send set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
		SLI4_SET_FEATURES_DIF_MEMORY_MODE, sizeof(mode_param), (uint32_t *)&mode_param);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		return OCS_HAL_RTN_ERROR;

	ocs_log_test(hal->os, "DIF mode set to %s\n",
		(hal->config.dif_mode == OCS_HAL_DIF_MODE_INLINE ? "inline" : "separate"));
	return rc;
}

/**
 * @brief Enable/disable firmware path to send auto-good response
 *    -  In G5/G6, The auto response mechanism does not clear the param field
 *    	 in FCP/NVMe frames. This in turn causes performance issues with certain
 *    	 initiators. This is a firmware workaround to force parameter field to
 *    	 zero
 *
 * @param hal Hardware context.
 * @param enable if 1: enable 0: disable
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_config_fw_ag_rsp(ocs_hal_t *hal, uint8_t enable)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];
	sli4_req_common_set_features_auto_response_param_val_reset_t param;

	ocs_memset(&param, 0, sizeof(param));
	param.pz = enable;

	/* build the set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
				    SLI4_SET_FEATURES_AUTO_RSP_PARAM_RESET,
				    sizeof(param),
				    &param);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc == OCS_HAL_RTN_NOT_SUPPORTED) {
		ocs_log_warn(hal->os, "Setting FW auto-good response not supported, rc %d, status %d\n", 
							rc, ((sli4_mbox_command_header_t *)buf)->status);
	} else if (rc || ((sli4_mbox_command_header_t *)buf)->status) {
		rc = OCS_HAL_RTN_ERROR;
	} else {
		ocs_log_test(hal->os, "Auto-response parameter field reset feature set to %d \n", enable);
	}

	return rc;
}

static void
ocs_watchdog_timer(void *arg)
{
	ocs_hal_t *hal = (ocs_hal_t *)arg;
	ocs_t *ocs = (ocs_t *)hal->os;

	/* Defer watchdog timer until driver initialization is completed */
	if (!ocs->drv_ocs.attached) {
		ocs_setup_timer(hal->os, &hal->watchdog_timer, ocs_watchdog_timer,
				hal, OCS_HAL_WATCHDOG_TIMER_DEFER_MSEC, false);
		return;
	}

	ocs_hal_config_watchdog_timer(hal);
}

static void
ocs_hal_cb_cfg_watchdog(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_lowlevel_set_watchdog_t *sli4_rsp =
			(sli4_res_lowlevel_set_watchdog_t *)mbox_rsp->payload.embed;

	if (sli4_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &sli4_rsp->hdr);

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (sli4_rsp->hdr.status)
			status = sli4_rsp->hdr.status;

		if (!hal->watchdog_timer_disable)
			/* keeping callback 500ms before timeout to keep heartbeat alive */
			ocs_setup_timer(hal->os, &hal->watchdog_timer, ocs_watchdog_timer,
					hal, (hal->watchdog_timeout*1000 - 500), false);
		else
			ocs_del_timer(&hal->watchdog_timer);
	}

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
}

/**
 * @brief Configure watchdog timer.
 *
 * @param hal Hardware context.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_config_watchdog_timer(ocs_hal_t *hal)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	int32_t timeout;
	uint8_t *buf;

	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (!buf) {
		ocs_log_err(hal->os, "no buffer for command\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	timeout = hal->watchdog_timer_disable ? 0 : hal->watchdog_timeout;

	sli4_cmd_lowlevel_set_watchdog(&hal->sli, buf, SLI4_BMBX_SIZE, timeout);
	rc = ocs_hal_command(hal, buf, OCS_CMD_NOWAIT, ocs_hal_cb_cfg_watchdog, NULL);
	if (rc)
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);

	return rc;
}

static void
ocs_hal_cb_itcm_parity_stats(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_lowlevel_get_itcm_parity_stats_t *sli4_rsp = NULL;
	ocs_hal_parity_stat_cb_arg_t *cb_arg = arg;

	sli4_rsp = (sli4_res_lowlevel_get_itcm_parity_stats_t *)mbox_rsp->payload.embed;
	if (sli4_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &sli4_rsp->hdr);

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_cb_itcm_parity_stats_done;

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (sli4_rsp->hdr.status)
			status = sli4_rsp->hdr.status;
	}

	cb_arg->cb(status, sli4_rsp->num_ulp, (ocs_parity_err_count_t *)sli4_rsp->counter, cb_arg->arg);

ocs_hal_cb_itcm_parity_stats_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
}

ocs_hal_rtn_e
ocs_hal_get_itcm_parity_stats(ocs_hal_t *hal, bool clear_stats, ocs_hal_parity_stat_cb_t cb, void *arg)
{
	uint8_t *buf = NULL;
	ocs_hal_parity_stat_cb_arg_t *cb_arg = NULL;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;

	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_NOWAIT);
	if (!buf) {
		ocs_log_err(hal->os, "Failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg = ocs_malloc(hal->os, sizeof(*cb_arg), OCS_M_NOWAIT);
	if (!cb_arg) {
		ocs_log_err(hal->os, "Failed to malloc cb_arg\n");
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;

	sli4_cmd_lowlevel_get_itcm_parity_stats(&hal->sli, buf, SLI4_BMBX_SIZE, clear_stats);
	rc = ocs_hal_command(hal, buf, OCS_CMD_NOWAIT, ocs_hal_cb_itcm_parity_stats, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
	}

	return rc;
}

static void
ocs_hal_generic_mbx_command_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_hdr_t *sli4_rsp_hdr;
	ocs_mq_mbx_results_t *results = arg;

	if (0 == status) {
		/* If MQE write was fine, look for mbox and sli4 status */
		if (results->handle_sli4_rsp) {
			sli4_rsp_hdr = (sli4_res_hdr_t *)mbox_rsp->payload.embed;
			status = sli4_rsp_hdr->status;
			if (status)
				ocs_hal_log_sli4_rsp_hdr(hal, sli4_rsp_hdr);
		}

		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
	}

	results->status = ((0 == status) ? 0 : 1);

	ocs_sem_v(&results->wait_sem);

	if (ocs_atomic_sub_and_test(&results->refcnt, 1))
		ocs_free(hal->os, results, sizeof(*results));
}

/**
 * @brief Issue a blocking hal command that has generic response
 *        handling in callback handler
 *
 * @param hal: Hardware context.
 * @param req_rsp_buf: The command request and response buffer
 *
 * @param handle_sli4_rsp: command response has sli header
 * @param timeout_usecs: Semphore wait timeout
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 * 		   OCS_HAL_RTN_ERROR on timeout or failure
 */
static ocs_hal_rtn_e
ocs_hal_exec_mbx_command_generic_results(ocs_hal_t *hal, uint8_t *req_rsp_buf, bool handle_sli4_rsp, int timeout_usecs)
{
	ocs_mq_mbx_results_t *results;
	ocs_hal_rtn_e rc;

	results = ocs_malloc(hal->os, sizeof(ocs_mq_mbx_results_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (results == NULL) {
		ocs_log_err(hal->os, "Failed to allocate memory\n");
		return OCS_HAL_RTN_ERROR;
	}

	/* Copy command to buffer */
	ocs_memcpy(results->cmd_buf, req_rsp_buf, SLI4_BMBX_SIZE);

	ocs_sem_init(&results->wait_sem, 0, "mq_mbx_results_sem");
	ocs_atomic_init(&results->refcnt, 2);
	results->status = 0;
	results->handle_sli4_rsp = handle_sli4_rsp;

	rc = ocs_hal_command(hal, results->cmd_buf, OCS_CMD_NOWAIT, ocs_hal_generic_mbx_command_cb, results);
	if (rc) {
		ocs_atomic_sub_and_test(&results->refcnt, 1);
		goto exit_mbx_cmd;
	}

	if (ocs_sem_p(&results->wait_sem, timeout_usecs)) {
		ocs_log_err(hal->os, "ocs_sem_p failed\n");
		rc = OCS_HAL_RTN_ERROR;
	} else {
		ocs_memcpy(req_rsp_buf, results->cmd_buf, SLI4_BMBX_SIZE);
	}

	if (rc || results->status)
		ocs_log_err(hal->os, "MBOX command failed\n");

	/* The caller can differentiate that a SLI response
	 * was rcvd but the resp had an error in it.
	 */
	if (results->status)
		rc = OCS_HAL_RTN_SLI_RESP_ERROR;

exit_mbx_cmd:
	if (ocs_atomic_sub_and_test(&results->refcnt, 1))
		ocs_free(hal->os, results, sizeof(*results));

	return rc;
}

/**
 * @brief Set trunking mode
 *
 * @param hal Hardware context.
 * @param trunk_mode Trunk mode to set
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
ocs_hal_rtn_e
ocs_hal_set_trunk_mode(ocs_hal_t *hal, uint32_t trunk_mode)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];

	if (hal == NULL) {
		ocs_log_err(NULL, "Failed to access HAL\n");
		return OCS_HAL_RTN_ERROR;
	}

	if (sli_cmd_fc_set_trunk_mode(&hal->sli, buf, SLI4_BMBX_SIZE, trunk_mode) == -1) {
		ocs_log_err(hal->os, "[ Trunk ] Invalid mode %d\n", trunk_mode);
		rc = OCS_HAL_RTN_INVALID_ARG;
		return OCS_HAL_RTN_ERROR;
	}

	rc = ocs_hal_exec_mbx_command_generic_results(hal, buf, true, 10*1000*1000);
	if (rc) {
		ocs_log_err(hal->os, "[ Trunk ] config failed, rc = %d\n", rc);
	} else {
		ocs_log_info(hal->os, "[ Trunk ] mode=%d configured successfully."
				      "Host reboot is required for the new configuration to take effect\n",
					trunk_mode);
	}

	return rc;
}

/**
 * @brief Get trunking mode information
 *
 * @param hal Hardware context.
 * @param trunk_info Placeholder to return trunk config info
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
ocs_hal_rtn_e
ocs_hal_get_trunk_info(ocs_hal_t *hal, uint32_t *trunk_info)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	sli4_cmd_read_topology_t *read_topo;
	uint8_t buf[SLI4_BMBX_SIZE];

	read_topo = (sli4_cmd_read_topology_t *)buf;
	/* Issue read_topology to get trunking configuration */
	if (sli_cmd_read_topology(&hal->sli, buf, SLI4_BMBX_SIZE, NULL)) {
		rc = ocs_hal_exec_mbx_command_generic_results(hal, buf, false, 10*1000*1000);
	}

	if (!rc) {
		*trunk_info = (uint32_t)read_topo->trunk_config;
	} else {
		ocs_log_err(hal->os, "[ Trunk ] READ_TOPOLOGY failed, rc = %d\n", rc);
	}

	return rc;
}

/**
 * @brief Stop firmware diagnostic logging (RAS).
 *
 * @param hal Hardware context.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
int
ocs_hal_stop_fw_diagnostic_logging(ocs_hal_t *hal)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t *buf;

	ocs_log_info(hal->os, "[RAS] Attempting to stop diagnostic logging\n");
	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_NOWAIT | OCS_M_ZERO);
	if (!buf) {
		ocs_log_err(hal->os, "no buffer for FW diag logging command\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	sli4_cmd_lowlevel_disable_ras(&hal->sli, buf, SLI4_BMBX_SIZE);
	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		rc = OCS_HAL_RTN_ERROR;
	else
		ocs_log_info(hal->os, "[RAS] stopped diagnostic logging successfully\n");

	ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
	return rc;
}

/**
 * @brief If required disable firmware diagnostic logging (RAS) and then free resources.
 *
 * @param hal Hardware context.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_teardown_fw_diagnostic_logging(ocs_hal_t *hal)
{
	return ocs_ras_free_resc(hal->os);
}

/**
 * @brief Set configuration parameters for adapter firmware diagnostic logging (RAS) feature.
 *
 * @param hal Hardware context.
 * @param log_size Host buffer size shared with the firmware to write logs.
 * @param log_level Firmware log level. Valid range is - 0 (least verbose) to 4 (most verbose).
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_config_fw_diagnostic_logging(ocs_hal_t *hal, int32_t log_size, int32_t log_level)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t *buf;
	uintptr_t lwpd_phys;
	uint32_t *buf_phys_tbl = NULL;
	int32_t buf_count;

	/* First SLI port initializes the global data structure */
	ocs_ras_init(log_level, log_size);

	rc = ocs_ras_allocate_resc(hal->os);
	if (rc) {
		if (rc == -EEXIST) {
			/* API automatically increments ref count */
			ocs_log_info(hal->os, "[RAS] firmware diagnostic logging is active for the adapter\n");
			rc = OCS_HAL_RTN_SUCCESS;
		} else if (rc == -EPERM) {
			rc = OCS_HAL_RTN_INVALID_ARG;
		} else if (rc == -EOPNOTSUPP) {
			ocs_log_warn(hal->os, "[RAS] firmware diagnostic logging is disabled for the adapter\n");
			rc = OCS_HAL_RTN_SUCCESS;
		} else {
			rc = OCS_HAL_RTN_NO_RESOURCES;
		}
		goto exit_with_err;
	}

	lwpd_phys = ocs_ras_get_lwpd_dma_addr(hal->os);
	buf_phys_tbl = ocs_ras_get_dma_addr_list(hal->os, &buf_count);
	if (!lwpd_phys || !buf_phys_tbl || !buf_count) {
		ocs_log_err(hal->os, "[RAS] resource error\n");
		rc = OCS_HAL_RTN_ERROR;
		goto exit_with_err;
	}

	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_NOWAIT | OCS_M_ZERO);
	if (!buf) {
		ocs_log_err(hal->os, "[RAS] no buffer for LOWLEVEL_SET_DIAG_LOG_OPTIONS mailbox command\n");
		rc = OCS_HAL_RTN_NO_MEMORY;
		goto exit_with_err;
	}

	sli4_cmd_lowlevel_enable_ras(&hal->sli, buf, SLI4_BMBX_SIZE, log_level,
			log_size / buf_count, buf_count, buf_phys_tbl, lwpd_phys);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		rc = OCS_HAL_RTN_ERROR;
	else
		ocs_log_info(hal->os, "[RAS] diagnostic logging configured successfully\n");

	ocs_free(hal->os, buf, SLI4_BMBX_SIZE);

exit_with_err:
	if (buf_phys_tbl)
		ocs_vfree(hal->os, buf_phys_tbl, buf_count * 8);

	return rc;
}

/**
 * @brief Set configuration parameters for TOW T10 PI feature.
 *
 * @param hal Hardware context.
 * @param buf Pointer to a mailbox buffer area.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_config_tow_t10pi(ocs_hal_t *hal)
{
	uint8_t buf[SLI4_BMBX_SIZE];
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	sli4_req_common_set_features_tow_t10pi_t param;

	ocs_memset(&param, 0, sizeof(param));
	param.scsi_diseed_sge.app_tag_repl = hal->config.tow_app_tag_value;
	param.scsi_diseed_sge.sge_type = SLI4_SGE_TYPE_DISEED;
	param.scsi_diseed_sge.dif_op_rx = SLI4_SGE_DIF_OP_IN_CRC_OUT_CRC;
	param.scsi_diseed_sge.dif_op_tx = SLI4_SGE_DIF_OP_IN_CRC_OUT_CRC;

	/* build the set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
				    SLI4_SET_FEATURES_SET_CONFIG_TOW_T10PI,
				    sizeof(param), &param);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		rc = OCS_HAL_RTN_ERROR;
	else
		ocs_log_info(hal->os, "Optimized Write T10 PI configured \n");

	return rc;
}

/**
 * @brief Set configuration parameters for AXR T10 PI feature.
 *
 * @param hal Hardware context.
 * @param buf Pointer to a mailbox buffer area.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_config_axr_t10pi(ocs_hal_t *hal)
{
	uint8_t buf[SLI4_BMBX_SIZE];
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	sli4_req_common_set_features_axr_t10pi_t param;

	ocs_memset(&param, 0, sizeof(param));
	param.rtc = (hal->config.tow_ref_tag_lba ? 0 : 1);
	param.atv = (hal->config.tow_app_tag_valid ? 1 : 0);
	param.tmm = ((hal->config.dif_mode == OCS_HAL_DIF_MODE_INLINE) ? 0 : 1);
	param.app_tag = hal->config.tow_app_tag_value;
	param.blk_size = hal->config.tow_blksize_chip;

	switch (hal->config.tow_p_type) {
	case 1:
		param.p_type = 0;
		break;
	case 3:
		param.p_type = 2;
		break;
	default:
		ocs_log_err(hal->os, "unsupported p_type %d\n", hal->config.tow_p_type);
		return OCS_HAL_RTN_ERROR;
	}

	/* build the set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
				    SLI4_SET_FEATURES_SET_CONFIG_AXR_T10PI,
				    sizeof(param), &param);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		rc = OCS_HAL_RTN_ERROR;
	else
		ocs_log_info(hal->os, "Auto XFER RDY T10 PI configured rtc:%d atv:%d p_type:%d app_tag:%x blk_size:%d\n",
			     param.rtc, param.atv, param.p_type, param.app_tag, param.blk_size);

	return rc;
}

/**
 * @brief enable sli port health check
 *
 * @param hal Hardware context.
 * @param buf Pointer to a mailbox buffer area.
 * @param query current status of the health check feature enabled/disabled
 * @param enable if 1: enable 0: disable
 * @param buf Pointer to a mailbox buffer area.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_config_sli_port_health_check(ocs_hal_t *hal, uint8_t query, uint8_t enable)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];
	sli4_req_common_set_features_health_check_t param;

	ocs_memset(&param, 0, sizeof(param));
	param.hck = enable;
	param.qry = query;

	/* build the set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
				    SLI4_SET_FEATURES_SLI_PORT_HEALTH_CHECK,
				    sizeof(param), &param);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		rc = OCS_HAL_RTN_ERROR;
	else
		ocs_log_test(hal->os, "SLI Port Health Check is enabled\n");

	return rc;
}

/**
 * @brief configure fec (enable/disable) on physical port
 * 	  - supported only for FC at present
 *
 * @param hal Hardware context.
 * @param enable if 1: enable 0: disable
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_config_fec(ocs_hal_t *hal, uint8_t enable)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];
	sli4_req_common_set_features_config_fec_t param;

	ocs_memset(&param, 0, sizeof(param));
	param.link_number = hal->sli.physical_port;
	param.link_type = SLI4_CONFIG_FEC_LINK_TYPE_FC;
	param.en = enable;

	/* build the set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
				    SLI4_SET_FEATURES_FEC,
				    sizeof(param),
				    &param);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		rc = OCS_HAL_RTN_ERROR;
	else
		ocs_log_test(hal->os, "FEC feature set to %d\n", enable);

	return rc;
}

static void
ocs_hal_parse_pause_errx_pairs(ocs_hal_t *hal)
{
	int i, token_cnt = 0;
	char *token = NULL;
	char *dup_str = ocs_strdup(hal->sliport_pause_errors);
	char *pause_errors = dup_str;
	char *errx_pair[SLI4_PAUSE_ERRX_PAIR_CNT] = { NULL };

	while (NULL != (token = ocs_strsep(&pause_errors, ","))) {
		errx_pair[token_cnt++] = token;

		/* Limit the # of tokens to SLI4_PAUSE_ERRX_PAIR_CNT */
		if (SLI4_PAUSE_ERRX_PAIR_CNT == token_cnt) {
			break;
		}
	}

	for (i = 0; i < token_cnt; i++) {
		char *err1 = NULL;
		char *err2 = NULL;

		/* Handle leading spaces */
		while (NULL != (err1 = ocs_strsep(&errx_pair[i], " "))) {
			if ('\0' != *err1) {
				break;
			}
		}

		/* Handle multiple spaces */
		while (NULL != (err2 = ocs_strsep(&errx_pair[i], " "))) {
			if ('\0' != *err2) {
				break;
			}
		}

		/* Handle empty tokens */
		if (err1) {
			hal->sli.pause_errx_pair[i].err1 = ocs_strtoul(err1, &token, 16);
		} else {
			continue;
		}

		/* Handle wildcard values for err2 */
		if (err2) {
			hal->sli.pause_errx_pair[i].err2 = ocs_strtoul(err2, &token, 16);
		} else {
			hal->sli.pause_errx_pair[i].err2 = 0xFFFFFFFF;
		}

		ocs_log_info(hal->os, "err1[%d]: 0x%x, err2[%d]: 0x%x\n",
				i, hal->sli.pause_errx_pair[i].err1,
				i, hal->sli.pause_errx_pair[i].err2);
	}

	ocs_free(hal->os, dup_str, ocs_strlen(hal->sliport_pause_errors) + 1);
}

static ocs_hal_rtn_e
ocs_hal_config_sliport_pause(ocs_hal_t *hal, uint8_t enable)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];
	sli4_req_common_set_features_pause_state_t param;

	ocs_memset(&param, 0, sizeof(param));
	param.eps = enable;

	/* build the set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
				SLI4_SET_FEATURES_SLI_PORT_PAUSE_STATE,
				sizeof(param), &param);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status) {
		rc = OCS_HAL_RTN_ERROR;
	} else {
		ocs_hal_parse_pause_errx_pairs(hal);
		ocs_log_test(hal->os, "Sliport pause state is enabled\n");
	}

	return rc;
}

/**
 * @brief Set FTD transfer hint feature
 *
 * @param hal Hardware context.
 * @param fdt_xfer_hint size in bytes where read requests are segmented.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_config_set_fdt_xfer_hint(ocs_hal_t *hal, uint32_t fdt_xfer_hint)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];
	sli4_req_common_set_features_set_fdt_xfer_hint_t param;

	ocs_memset(&param, 0, sizeof(param));
	param.fdt_xfer_hint = fdt_xfer_hint;
	/* build the set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
				    SLI4_SET_FEATURES_SET_FTD_XFER_HINT,
				    sizeof(param), &param);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (rc || ((sli4_mbox_command_header_t *)buf)->status)
		rc = OCS_HAL_RTN_ERROR;
	else
		ocs_log_info(hal->os, "Set FTD transfer hint to %d\n", param.fdt_xfer_hint);

	return rc;
}

static ocs_hal_rtn_e
ocs_parse_dual_dump_config_resp(ocs_hal_t *hal, uint8_t *resp, uint32_t *dual_dump_state)
{
	bool fw_state;

	if (sli_parse_fw_dual_dump_state(&hal->sli, resp, SLI4_BMBX_SIZE, &fw_state)) {
		ocs_log_err(hal->os, "Dual dump state cannot be confirmed.\n");
		return OCS_HAL_RTN_ERROR;
	}

	if (fw_state)
		hal->dual_dump_state = OCS_HAL_DUAL_DUMP_STATE_ENABLED;
	else
		hal->dual_dump_state = OCS_HAL_DUAL_DUMP_STATE_DISABLED;

	*dual_dump_state = (uint32_t)hal->dual_dump_state;
	return OCS_HAL_RTN_SUCCESS;
}

ocs_hal_rtn_e
ocs_hal_config_get_dual_dump_state(ocs_hal_t *hal, uint32_t *state)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];
	sli4_req_common_set_features_dual_dump_param_t param;

	/* check if feature is not supported by fw or dump to host is enabled. */
	if (hal->dual_dump_state == OCS_HAL_DUAL_DUMP_STATE_NOT_SUPPORTED ||
	    hal->dual_dump_state == OCS_HAL_DUAL_DUMP_STATE_INVALID) {
		*state = (uint32_t)hal->dual_dump_state;
		return rc;
	}

	*state = (uint32_t)OCS_HAL_DUAL_DUMP_STATE_UNKNOWN;
	ocs_memset(&param, 0, sizeof(param));
	param.query = SLI4_SET_FEATURES_QUERY_DUAL_DUMP;

	/* build the set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
				    SLI4_SET_FEATURES_CONFIG_DUAL_DUMP,
				    sizeof(param), &param);

	rc = ocs_hal_exec_mbx_command_generic_results(hal, buf, false, OCS_HAL_MBOX_CMD_TIMEOUT);
	if (rc == OCS_HAL_RTN_ERROR) {
		ocs_log_err(hal->os, "Failed to get dual dump state.\n");
		return rc;
	}

	return ocs_parse_dual_dump_config_resp(hal, buf, state);
}

/**
 * @brief Configure dual dump feature per requirement
 *
 * @param hal Hardware context.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_config_set_dual_dump(ocs_hal_t *hal)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t buf[SLI4_BMBX_SIZE];
	sli4_req_common_set_features_dual_dump_param_t param;
	uint32_t fw_state;
	uint32_t req_state;
	char *requested;
	ocs_t *ocs = (ocs_t *)hal->os;

	hal->dual_dump_state = OCS_HAL_DUAL_DUMP_STATE_UNKNOWN;

	ocs_hal_get(hal, OCS_HAL_ENABLE_DUAL_DUMP, &req_state);
	if (req_state > 1)
		return OCS_HAL_RTN_INVALID_ARG;

	if (OCS_FW_DUMP_TYPE_FLASH != ocs->fw_dump.type) {
		hal->dual_dump_state = OCS_HAL_DUAL_DUMP_STATE_INVALID;
		return OCS_HAL_RTN_INVALID_ARG;
	}

	ocs_memset(&param, 0, sizeof(param));
	param.dual_dump = (bool)req_state;
	param.query = SLI4_SET_FEATURES_SET_DUAL_DUMP;

	requested = req_state ? "enable" : "disable";

	/* build the set_features command */
	sli_cmd_common_set_features(&hal->sli, buf, SLI4_BMBX_SIZE,
				    SLI4_SET_FEATURES_CONFIG_DUAL_DUMP,
				    sizeof(param), &param);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	switch (rc) {
	case OCS_HAL_RTN_NOT_SUPPORTED:
		ocs_log_debug(hal->os, "Dual dump is not supported by the SLI port.\n");
		hal->dual_dump_state = OCS_HAL_DUAL_DUMP_STATE_NOT_SUPPORTED;
		return rc;
	case OCS_HAL_RTN_SUCCESS:
		break;
	default:
		ocs_log_err(hal->os, "Failed to %s dual dump state, ret status %#x.\n",
			    requested, ((sli4_mbox_command_header_t *)buf)->status);
		return rc;
	}

	rc = ocs_parse_dual_dump_config_resp(hal, buf, &fw_state);
	if (!rc)
		ocs_log_info(hal->os, "Dual dump states: requested=%s, current=%s\n",
			     requested, fw_state ? "enabled" : "disabled");

	return rc;
}

/**
 * @brief Get the link configuration callback.
 *
 * @param hal Hardware context.
 * @param status Status from the DMTF CLP command.
 * @param result_len Length, in bytes, of the DMTF CLP result.
 * @param arg Pointer to a callback argument.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static void
ocs_hal_linkcfg_dmtf_clp_cb(ocs_hal_t *hal, int32_t status, uint32_t result_len, void *arg)
{
	int32_t rval;
	char retdata_str[64];
	ocs_hal_linkcfg_cb_arg_t *cb_arg = (ocs_hal_linkcfg_cb_arg_t *)arg;
	ocs_hal_linkcfg_e linkcfg = OCS_HAL_LINKCFG_NA;

	if (status) {
		ocs_log_test(hal->os, "CLP cmd failed, status=%d\n", status);
	} else {
		/* parse CLP response to get return data */
		rval = ocs_hal_clp_resp_get_value(hal, "retdata", retdata_str,
						  sizeof(retdata_str),
						  cb_arg->dma_resp.virt,
						  result_len);
		if (rval <= 0) {
			ocs_log_err(hal->os, "failed to get retdata %d\n", result_len);
		} else {
			/* translate string into hal enum */
			linkcfg = ocs_hal_linkcfg_from_clp(retdata_str);
		}
	}

	/* invoke callback */
	if (cb_arg->cb) {
		cb_arg->cb(status, linkcfg, cb_arg->arg);
	}

	/* if polling, will free memory in calling function */
	if (cb_arg->opts != OCS_CMD_POLL) {
		ocs_dma_free(hal->os, &cb_arg->dma_cmd);
		ocs_dma_free(hal->os, &cb_arg->dma_resp);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
	}
}

#if !defined(OCSU_FC_WORKLOAD)
/**
 * @brief Set the Lancer dump location
 *
 * @par Description
 * This function tells a Lancer chip to use a specific DMA
 * buffer as a dump location rather than the internal flash.
 *
 * @param hal Hardware context.
 * @param num_buffers The number of DMA buffers to hold the dump (1..n).
 * @param dump_buffers DMA buffers to hold the dump.
 * @param fdb function specific dump.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_set_dump_location(ocs_hal_t *hal, uint32_t num_buffers, ocs_dma_t *dump_buffers, uint8_t fdb)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	uint8_t buf[SLI4_BMBX_SIZE];
	ocs_dma_t *dump_sges = NULL;

	if (!dump_buffers || !num_buffers) {
		ocs_log_err(hal->os, "Dump buffers are missing\n");
		return rc;
	}

	/**
	 * If a single buffer is used, then it may be passed as is to the chip. For multiple buffers,
	 * we must allocate an SGL and then pass the address of the list to the chip.
	 */
	if (num_buffers > 1) {
		uint32_t sge_size = num_buffers * sizeof(sli4_sge_t);
		sli4_sge_t *sge;
		uint32_t i;

		dump_sges = (fdb ? &hal->func_dump_sges : &hal->chip_dump_sges);
		if (dump_sges->size < sge_size) {
			ocs_dma_free(hal->os, dump_sges);
			if (ocs_dma_alloc(hal->os, dump_sges, sge_size, OCS_MIN_DMA_ALIGNMENT)) {
				ocs_log_err(hal->os, "SGE DMA allocation failed\n");
				return OCS_HAL_RTN_NO_MEMORY;
			}
		}

		/* build the SGE list */
		ocs_memset(dump_sges->virt, 0, dump_sges->size);
		dump_sges->len = sge_size;
		sge = dump_sges->virt;

		for (i = 0; i < num_buffers; i++) {
			sge[i].buffer_address_high = ocs_addr32_hi(dump_buffers[i].phys);
			sge[i].buffer_address_low = ocs_addr32_lo(dump_buffers[i].phys);
			sge[i].last = (i == num_buffers - 1 ? 1 : 0);
			sge[i].buffer_length = dump_buffers[i].size;
		}

		sli_cmd_common_set_dump_location(&hal->sli, (void *)buf,
			SLI4_BMBX_SIZE, FALSE, TRUE, dump_sges, fdb);
	} else {
		dump_buffers->len = dump_buffers->size;
		sli_cmd_common_set_dump_location(&hal->sli, (void *)buf,
			SLI4_BMBX_SIZE, FALSE, FALSE, dump_buffers, fdb);
	}

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (OCS_HAL_RTN_SUCCESS == rc) {
		ocs_log_debug(hal->os, "Set %s dump location is successful\n", fdb ? "func" : "chip");
	} else {
		/* Free the FW dump buffer SGE's */
		if (dump_sges)
			ocs_dma_free(hal->os, dump_sges);

		ocs_log_err(hal->os, "Set %s dump location command failed\n", fdb ? "func" : "chip");
	}

	return rc;
}

/**
 * @brief Clear the Lancer dump location
 *
 * @par Description
 * This function clears the existing DMA buffer
 * set previously while setting dump location.
 *
 * @param hal Hardware context.
 * @param fdb function specific dump.
 *
 * @return None.
 */
static void
ocs_hal_clear_dump_location(ocs_hal_t *hal, uint8_t fdb)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	uint8_t buf[SLI4_BMBX_SIZE];

	sli_cmd_common_set_dump_location(&hal->sli, (void *)buf, SLI4_BMBX_SIZE, FALSE, FALSE, NULL, fdb);

	rc = ocs_hal_command(hal, buf, OCS_CMD_POLL, NULL, NULL);
	if (OCS_HAL_RTN_SUCCESS == rc) {
		ocs_log_debug(hal->os, "Clear %s dump location is successful\n", fdb ? "func" : "chip");
	} else {
		ocs_log_err(hal->os, "Clear %s dump location command failed\n", fdb ? "func" : "chip");
	}
}

void
ocs_hal_free_dump_to_host_buffers(ocs_hal_t *hal)
{
	ocs_t *ocs = (ocs_t *)hal->os;
	ocs_dma_t *dump_buffers = NULL;
	uint32_t i, num_buffers = 0;

	if (ocs->fw_dump.chip_dump_buffers) {
		ocs_hal_clear_dump_location(hal, FALSE);

		dump_buffers = ocs->fw_dump.chip_dump_buffers;
		num_buffers = ocs->fw_dump.num_chip_dump_buffers;

		for (i = 0; i < num_buffers; i++) {
			ocs_dma_free(ocs, &dump_buffers[i]);
		}

		ocs_free(ocs, dump_buffers, sizeof(ocs_dma_t) * num_buffers);
		ocs->fw_dump.chip_dump_buffers = NULL;
		ocs->fw_dump.num_chip_dump_buffers = 0;
	}

	if (ocs->fw_dump.func_dump_buffers) {
		ocs_hal_clear_dump_location(hal, TRUE);

		dump_buffers = ocs->fw_dump.func_dump_buffers;
		num_buffers = ocs->fw_dump.num_func_dump_buffers;

		for (i = 0; i < num_buffers; i++) {
			ocs_dma_free(ocs, &dump_buffers[i]);
		}

		ocs_free(ocs, dump_buffers, sizeof(ocs_dma_t) * num_buffers);
		ocs->fw_dump.func_dump_buffers = NULL;
		ocs->fw_dump.num_func_dump_buffers = 0;
	}

	if (ocs->fw_dump.saved_buff) {
		ocs_free(ocs, ocs->fw_dump.saved_buff, ocs->fw_dump.size);
		ocs->fw_dump.saved_buff = NULL;
	}
	ocs->fw_dump.size = 0;

	ocs->fw_dump.state = OCS_FW_DUMP_STATE_NONE;
	ocs_lock_free(&ocs->fw_dump.lock);
}

static int32_t
ocs_fw_dump_buffer_alloc(ocs_t *ocs, uint8_t dump_level)
{
	int rc = 0;
	uint32_t i;
	uint32_t rem_bytes = 0;
	uint32_t num_buffers = 0;
	ocs_dma_t *dump_buffers = NULL;

	if (OCS_FW_FUNC_DESC_DUMP == dump_level) {
		dump_buffers = ocs->fw_dump.func_dump_buffers;
		num_buffers = ocs->fw_dump.num_func_dump_buffers;
	} else {
		dump_buffers = ocs->fw_dump.chip_dump_buffers;
		num_buffers = ocs->fw_dump.num_chip_dump_buffers;
	}

	/* If dump buffers were already allocated, return success */
	if ((NULL != dump_buffers) && (0 != num_buffers))
		return rc;

	ocs_hal_get(&ocs->hal, OCS_HAL_DUMP_MAX_SIZE, &ocs->fw_dump.size);
	if (!ocs->fw_dump.saved_buff)
		ocs->fw_dump.saved_buff = ocs_malloc(ocs, ocs->fw_dump.size, OCS_M_ZERO);

	if (!ocs->fw_dump.saved_buff) {
		ocs_log_err(ocs, "Failed to allocated dump saved buffer\n");
		rc = -1;
		goto free_dump_buffers;
	}

	num_buffers = ((ocs->fw_dump.size + OCS_FW_DUMP_CHUNK_SIZE - 1) / OCS_FW_DUMP_CHUNK_SIZE);
	dump_buffers = ocs_malloc(ocs, sizeof(ocs_dma_t) * num_buffers, OCS_M_ZERO);
	if (!dump_buffers) {
		ocs_log_err(ocs, "Failed to allocate buffer dma objects for dump\n");
		rc = -1;
		goto free_dump_buffers;
	}

	if (OCS_FW_FUNC_DESC_DUMP == dump_level) {
		ocs->fw_dump.func_dump_buffers = dump_buffers;
		ocs->fw_dump.num_func_dump_buffers = num_buffers;
	} else {
		ocs->fw_dump.chip_dump_buffers = dump_buffers;
		ocs->fw_dump.num_chip_dump_buffers = num_buffers;
	}

	/* Allocate the DMA buffers to hold the dump */
	rem_bytes = ocs->fw_dump.size;

	for (i = 0; i < num_buffers; i++) {
		uint32_t num_bytes = OCS_MIN(rem_bytes, OCS_FW_DUMP_CHUNK_SIZE);
		rc = ocs_dma_alloc(ocs, &dump_buffers[i], num_bytes, OCS_MIN_DMA_ALIGNMENT);
		if (rc) {
			ocs_log_err(ocs, "Failed to allocate buffer for dump\n");
			rc = -1;
			goto free_dump_buffers;
		}
		rem_bytes -= num_bytes;
	}

	ocs_log_debug(ocs, "Successfully allocated FW dump buffers\n");
	return rc;

free_dump_buffers:
	ocs_hal_free_dump_to_host_buffers(&ocs->hal);
	return rc;
}

static ocs_hal_rtn_e
ocs_hal_alloc_set_dump_buffers(ocs_hal_t *hal, uint8_t dump_level)
{
	ocs_t *ocs = (ocs_t *)hal->os;
	uint8_t fdb_dump;
	uint32_t num_buffers = 0;
	ocs_dma_t *dump_buffers = NULL;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* Allocate dump buffers only once and re-use them in the reset path */
	rc = ocs_fw_dump_buffer_alloc(ocs, dump_level);
	if (rc) {
		ocs_log_err(ocs, "Failed to register FW dump buffers\n");
		return OCS_HAL_RTN_ERROR;
	}

	if (OCS_FW_FUNC_DESC_DUMP == dump_level) {
		fdb_dump = TRUE;
		dump_buffers = ocs->fw_dump.func_dump_buffers;
		num_buffers = ocs->fw_dump.num_func_dump_buffers;
	} else {
		fdb_dump = FALSE;
		dump_buffers = ocs->fw_dump.chip_dump_buffers;
		num_buffers = ocs->fw_dump.num_chip_dump_buffers;
	}

	rc = ocs_hal_set_dump_location(hal, num_buffers, dump_buffers, fdb_dump);
	if (rc) {
		ocs_log_err(hal->os, "Failed to register dump to host buffers\n");
	} else {
		ocs_log_info(hal->os, "Dump to host buffers registered with FW successfully\n");
	}

	return rc;
}

/**
 * @brief Allocates dump dma buffers, if not already allocated.
 * 	  Set the Lancer dump location
 * @par Description
 * This function tells a Lancer chip to use a specific DMA
 * buffer as a dump location rather than the internal flash.
 *
 * @param hal Hardware context.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
ocs_hal_rtn_e
ocs_hal_set_dump_to_host_buffers(ocs_hal_t *hal)
{
	ocs_t *ocs = (ocs_t *)hal->os;
	uint8_t bus, dev, func;
	ocs_hal_rtn_e rc;

	if (!ocs) {
		ocs_log_err(ocs, "ocs context is missing\n");
		return OCS_HAL_RTN_ERROR;
	}

	/* Don't set the dump location if 'fw_dump_to_host' is not enabled */
	if (OCS_FW_DUMP_TYPE_DUMPTOHOST != ocs->fw_dump.type)
		return OCS_HAL_RTN_SUCCESS;

	/**
	 * Make sure the FW is new enough to support this command.
	 * If the FW is too old, the FW will UE.
	 */
	if (hal->workaround.disable_dump_loc) {
		ocs_log_err(hal->os, "FW version is too old for this feature\n");
		return OCS_HAL_RTN_ERROR;
	}

	/* Set the chip level dump location only for pci function 0 */
	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	if (0 == func) {
		rc = ocs_hal_alloc_set_dump_buffers(hal, OCS_FW_CHIP_LEVEL_DUMP);
		if (OCS_HAL_RTN_SUCCESS != rc)
			return rc;
	}

	/* For FDD, set the dump location on all pci functions */
	return ocs_hal_alloc_set_dump_buffers(hal, OCS_FW_FUNC_DESC_DUMP);
}
#endif

/**
 * @brief set loopback mode
 * @par Description
 * This function set link loopback mode
 *
 *  @param hal Hardware context.
 *  @param type loopback type to be set
 *  @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
ocs_hal_rtn_e
ocs_hal_set_loopback_mode(ocs_hal_t *hal, uint32_t type)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	uint8_t buf[SLI4_BMBX_SIZE];

	if (!hal) {
		ocs_log_err(NULL, "Failed to access HAL\n");
		return rc;
	}

	sli_cmd_fcoe_set_loopback_mode(&hal->sli, (void *)buf, SLI4_BMBX_SIZE, type);
	return ocs_hal_exec_mbx_command_generic_results(hal, buf, true, 10*1000*1000);
}

/**
 * @brief Run Diagnostic PCI Loopback test
 * @par Description
 * This will run PCI loopback tests
 *
 * @param hal Hardware context.
 * @param TX DMA buffer address
 * @param TX DMA buffer size
 * @param RX DMA buffer address
 * @param RX DMA buffer size
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
ocs_hal_rtn_e
ocs_hal_run_biu_diag(ocs_hal_t *hal, void *tx_buff, size_t tx_buff_len,
		     void *rx_buff, size_t rx_buff_len)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t	buf[SLI4_BMBX_SIZE];
	ocs_dma_t tx_dma_buf, rx_dma_buf;

	if (hal == NULL) {
		ocs_log_err(NULL, "Failed to access HAL\n");
		return OCS_HAL_RTN_ERROR;
	}

	if (ocs_dma_alloc(hal->os, &tx_dma_buf, tx_buff_len, OCS_MIN_DMA_ALIGNMENT)) {
		ocs_log_err(hal->os, "Failed to alloc DMA memory for TX buffer\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	if (ocs_dma_alloc(hal->os, &rx_dma_buf, rx_buff_len, OCS_MIN_DMA_ALIGNMENT)) {
		ocs_log_err(hal->os, "Failed to alloc DMA memory for RX buffer\n");
		ocs_dma_free(hal->os, &tx_dma_buf);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	ocs_memset(tx_dma_buf.virt, 0, tx_buff_len);
	ocs_memset(rx_dma_buf.virt, 0, rx_buff_len);
	ocs_memcpy(tx_dma_buf.virt, tx_buff, tx_buff_len);

	if (sli_cmd_config_run_biu_diag(&hal->sli, buf, SLI4_BMBX_SIZE,
					&tx_dma_buf, tx_buff_len,
					&rx_dma_buf, rx_buff_len)) {
		rc = ocs_hal_exec_mbx_command_generic_results(hal, buf, false, 10*1000*1000);
		if (rc)
			goto exit_run_biu_diag;
	} else {
		ocs_log_err(hal->os, "sli_cmd_config_run_biu_diag failed\n");
		rc = OCS_HAL_RTN_ERROR;
		goto exit_run_biu_diag;
	}

	ocs_memcpy(rx_buff, rx_dma_buf.virt, rx_buff_len);

exit_run_biu_diag:
	ocs_dma_free(hal->os, &tx_dma_buf);
	ocs_dma_free(hal->os, &rx_dma_buf);
	return rc;
}

/**
 * @brief Set the Ethernet license.
 *
 * @par Description
 * This function sends the appropriate mailbox command (DMTF
 * CLP) to set the Ethernet license to the given license value.
 * Since it is used during the time of ocs_hal_init(), the mailbox
 * command is sent via polling (the BMBX route).
 *
 * @param hal Hardware context.
 * @param license 32-bit license value.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success.
 */
static ocs_hal_rtn_e
ocs_hal_set_eth_license(ocs_hal_t *hal, uint32_t license)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	char cmd[OCS_HAL_DMTF_CLP_CMD_MAX];
	ocs_dma_t dma_cmd;
	ocs_dma_t dma_resp;

	/* only for lancer right now */
	if (SLI4_IF_TYPE_LANCER_FC_ETH != sli_get_if_type(&hal->sli)) {
		ocs_log_test(hal->os, "Function only supported for I/F type 2\n");
		return OCS_HAL_RTN_ERROR;
	}

	ocs_snprintf(cmd, OCS_HAL_DMTF_CLP_CMD_MAX, "set / OEMELX_Ethernet_License=%X", license);
	/* allocate DMA for command  */
	if (ocs_dma_alloc(hal->os, &dma_cmd, ocs_strlen(cmd)+1, 4096)) {
		ocs_log_err(hal->os, "malloc failed\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}
	ocs_memset(dma_cmd.virt, 0, ocs_strlen(cmd)+1);
	ocs_memcpy(dma_cmd.virt, cmd, ocs_strlen(cmd));

	/* allocate DMA for response */
	if (ocs_dma_alloc(hal->os, &dma_resp, OCS_HAL_DMTF_CLP_RSP_MAX, 4096)) {
		ocs_log_err(hal->os, "malloc failed\n");
		ocs_dma_free(hal->os, &dma_cmd);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* send DMTF CLP command mbx and poll */
	if (ocs_hal_exec_dmtf_clp_cmd(hal, &dma_cmd, &dma_resp, OCS_CMD_POLL, NULL, NULL)) {
		ocs_log_err(hal->os, "CLP cmd=\"%s\" failed\n", (char *)dma_cmd.virt);
		rc = OCS_HAL_RTN_ERROR;
	}

	ocs_dma_free(hal->os, &dma_cmd);
	ocs_dma_free(hal->os, &dma_resp);
	return rc;
}

/**
 * @brief Callback argument structure for the DMTF CLP commands.
 */
typedef struct ocs_hal_clp_cb_arg_s {
	ocs_hal_dmtf_clp_cb_t cb;
	ocs_dma_t *dma_resp;
	int32_t status;
	uint32_t opts;
	void *arg;
} ocs_hal_clp_cb_arg_t;

/**
 * @brief Execute the DMTF CLP command.
 *
 * @param hal Hardware context.
 * @param dma_cmd DMA buffer containing the CLP command.
 * @param dma_resp DMA buffer that will contain the response (if successful).
 * @param opts Mailbox command options (such as OCS_CMD_NOWAIT and POLL).
 * @param cb Callback function.
 * @param arg Callback argument.
 *
 * @return Returns the number of bytes written to the response
 * buffer on success, or a negative value if failed.
 */
static ocs_hal_rtn_e
ocs_hal_exec_dmtf_clp_cmd(ocs_hal_t *hal, ocs_dma_t *dma_cmd, ocs_dma_t *dma_resp, uint32_t opts, ocs_hal_dmtf_clp_cb_t cb, void *arg)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	ocs_hal_clp_cb_arg_t *cb_arg;
	uint8_t *mbxdata;

	/* allocate DMA for mailbox */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* allocate memory for callback argument */
	cb_arg = ocs_malloc(hal->os, sizeof(*cb_arg), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;
	cb_arg->dma_resp = dma_resp;
	cb_arg->opts = opts;

	/* Send the HAL command */
	sli_cmd_dmtf_exec_clp_cmd(&hal->sli, mbxdata, SLI4_BMBX_SIZE, dma_cmd, dma_resp);
	rc = ocs_hal_command(hal, mbxdata, opts, ocs_hal_dmtf_clp_cb, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	} else if (opts == OCS_CMD_POLL) {
		/* If we're polling, call the callback here */
		ocs_hal_dmtf_clp_cb(hal, 0, mbxdata, cb_arg);
	}

	return rc;
}

/**
 * @brief Called when the DMTF CLP command completes.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback argument.
 *
 * @return None.
 *
 */
static void
ocs_hal_dmtf_clp_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_dmtf_exec_clp_cmd_t *clp_rsp = (sli4_res_dmtf_exec_clp_cmd_t *)mbox_rsp->payload.embed;
	ocs_hal_clp_cb_arg_t *cb_arg = arg;
	char stat_str[8] = { '\0' };
	int32_t stat_len = 0;
	uint32_t result_len = 0;

	if (clp_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &clp_rsp->hdr);

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_dmtf_clp_cb_done;

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (clp_rsp->hdr.status)
			status = clp_rsp->hdr.status;
	}

	if (0 == status)
		result_len = clp_rsp->resp_length;
	else
		goto ocs_hal_dmtf_clp_invoke_cb;

	if (!result_len || (cb_arg->dma_resp->size < result_len)) {
		ocs_log_test(hal->os, "Invalid response length: resp_len=%zu result len=%d\n",
			     cb_arg->dma_resp->size, result_len);
		status = -1;
		goto ocs_hal_dmtf_clp_invoke_cb;
	}

	/* Parse CLP response to get the status string */
	stat_len = ocs_hal_clp_resp_get_value(hal, "status", stat_str, sizeof(stat_str),
					      cb_arg->dma_resp->virt, result_len);
	if (stat_len <= 0) {
		ocs_log_test(hal->os, "Failed to get status, status len=%d\n", stat_len);
		status = -1;
		goto ocs_hal_dmtf_clp_invoke_cb;
	}

	if (0 != ocs_strcmp(stat_str, "0")) {
		ocs_log_test(hal->os, "CLP status indicates failure=%s\n", stat_str);
		status = -1;
	}

ocs_hal_dmtf_clp_invoke_cb:
	cb_arg->status = status;
	cb_arg->cb(hal, cb_arg->status, result_len, cb_arg->arg);

ocs_hal_dmtf_clp_cb_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
}

/**
 * @brief Parse the CLP result and get the value corresponding to the given
 * keyword.
 *
 * @param hal Hardware context.
 * @param keyword CLP keyword for which the value is returned.
 * @param value Location to which the resulting value is copied.
 * @param value_len Length of the value parameter.
 * @param resp Pointer to the response buffer that is searched
 * for the keyword and value.
 * @param resp_len Length of response buffer passed in.
 *
 * @return Returns the number of bytes written to the value
 * buffer on success, or a negative vaue on failure.
 */
static int32_t
ocs_hal_clp_resp_get_value(ocs_hal_t *hal, const char *keyword, char *value, uint32_t value_len, const char *resp, uint32_t resp_len)
{
	char *start = NULL;
	char *end = NULL;

	/* look for specified keyword in string */
	start = ocs_strstr(resp, keyword);
	if (start == NULL) {
		ocs_log_test(hal->os, "could not find keyword=%s in CLP response\n", keyword);
		return -1;
	}

	/* now look for '=' and go one past */
	start = ocs_strchr(start, '=');
	if (start == NULL) {
		ocs_log_test(hal->os, "could not find \'=\' in CLP response for keyword=%s\n", keyword);
		return -1;
	}
	start++;

	/* \r\n terminates value */
	end = ocs_strstr(start, "\r\n");
	if (end == NULL) {
		ocs_log_test(hal->os, "could not find \\r\\n for keyword=%s in CLP response\n", keyword);
		return -1;
	}

	/* make sure given result array is big enough */
	if ((end - start + 1) > value_len) {
		ocs_log_test(hal->os, "value len=%d not large enough for actual=%ld\n", value_len, (end-start));
		return -1;
	}

	ocs_memcpy(value, start, (end - start));
	value[end-start] = '\0';
	return (end-start+1);
}

/**
 * @brief Cause chip to enter an unrecoverable error state.
 *
 * @par Description
 * Cause chip to enter an unrecoverable error state. This is
 * used when detecting unexpected FW behavior so that the FW can be
 * halted from the driver as soon as the error is detected.
 *
 * @param hal Hardware context.
 * @param dump Generate dump as part of reset.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 *
 */
ocs_hal_rtn_e
ocs_hal_raise_ue(ocs_hal_t *hal, uint8_t dump)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	if (sli_raise_ue(&hal->sli, dump) != 0) {
		rc = OCS_HAL_RTN_ERROR;
	} else {
		if (hal->state != OCS_HAL_STATE_UNINITIALIZED) {
			hal->state = OCS_HAL_STATE_QUEUES_ALLOCATED;
		}
	}

	return rc;
}

ocs_hal_rtn_e
ocs_hal_common_write_obj_wait(ocs_hal_t *hal, char *file,
			      ocs_dma_t *dma, uint32_t len)
{
	uint8_t mqe[SLI4_BMBX_SIZE] = { 0 };
	ocs_hal_rtn_e rc;

	ocs_log_debug(hal->os, "file=%s\n", file);

	sli_cmd_common_write_object(&hal->sli, mqe, SLI4_BMBX_SIZE, false,
				    true, len, 0, file, dma);
	/* the 20s timeout is arbitarary: it's not possible for the host
	 * driver to predict how long it could take for the cmd to complete
	 * after MQE db is rung.
	 */
	rc = ocs_hal_exec_mbx_command_generic_results(hal, mqe, true,
						      20 * 1000 * 1000);
	return rc;
}

/**
 * @brief Read a common object
 *	  *** This routine returns only after the cmd completes ***
 * @param hal Hardware context.
 * @param dma DMA structure to transfer the file data to
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 * When a valid completion with a status/addl-status is rcvd,
 * *status and *additional_status are set to the values returned by FW.
 */
ocs_hal_rtn_e
ocs_hal_read_common_obj_wait(ocs_hal_t *hal, char *file, uint32_t offset,
			     ocs_dma_t *dma, uint32_t *read_len,
			     u8 *status, u8 *additional_status)
{
	uint8_t mqe[SLI4_BMBX_SIZE] = { 0 };
	ocs_hal_rtn_e rc;

	ocs_log_debug(hal->os, "file=%s\n", file);

	sli_cmd_common_read_object(&hal->sli, mqe, SLI4_BMBX_SIZE,
				   dma->size, offset, file, dma);

	/* the 20s timeout is arbitarary: it's not possible for the host
	 * driver to predict how long it could take for the cmd to complete
	 * after MQE db is rung.
	 */
	rc = ocs_hal_exec_mbx_command_generic_results(hal, mqe, true,
						      20 * 1000 * 1000);
	if (!OCS_HAL_RTN_IS_ERROR(rc) || rc == OCS_HAL_RTN_SLI_RESP_ERROR) {
		sli4_cmd_sli_config_t *cmd = (sli4_cmd_sli_config_t *)mqe;
		sli4_res_common_read_object_t *resp =
			 (sli4_res_common_read_object_t *)cmd->payload.embed;

		*status = resp->hdr.status;
		*additional_status = resp->hdr.additional_status;
		*read_len = resp->actual_read_length;
	} else {
		*status = 0;
		*additional_status = 0;
		*read_len = 0;
	}

	return rc;
}

/**
 * @brief Called when the OBJECT_GET command completes.
 *
 * @par Description
 * Get the number of bytes actually written out of the response, free the mailbox
 * that was malloc'd by ocs_hal_dump_get(), then call the callback
 * and pass the status and bytes read.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 * The callback function prototype is <tt>void cb(int32_t status, uint32_t bytes_read)</tt>.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_cb_dump_get(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_read_object_t *rd_obj_rsp = (sli4_res_common_read_object_t *)mbox_rsp->payload.embed;
	ocs_hal_dump_get_cb_arg_t *cb_arg = arg;
	uint32_t bytes_read = rd_obj_rsp->actual_read_length;
	uint8_t eof = rd_obj_rsp->eof;

	if (rd_obj_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &rd_obj_rsp->hdr);

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_cb_dump_get_done;

	 if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (rd_obj_rsp->hdr.status)
			status = rd_obj_rsp->hdr.status;
	}

	cb_arg->cb(status, bytes_read, eof, cb_arg->arg);

ocs_hal_cb_dump_get_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}


/**
 * @brief Read a dump image to the host.
 *
 * @par Description
 * Creates a SLI_CONFIG mailbox command, fills in the correct values to read a
 * dump image chunk, then sends the command with the ocs_hal_command(). On completion,
 * the callback function ocs_hal_cb_dump_get() gets called to free the mailbox
 * and signal the caller that the read has completed.
 *
 * @param hal Hardware context.
 * @param dma DMA structure to transfer the dump chunk into.
 * @param size Size of the dump chunk.
 * @param offset Offset, in bytes, from the beginning of the dump.
 * @param cb Pointer to a callback function that is called when the command completes.
 * The callback function prototype is
 * <tt>void cb(int32_t status, uint32_t bytes_read, uint8_t eof, void *arg)</tt>.
 * @param arg Pointer to be passed to the callback function.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_dump_get(ocs_hal_t *hal, ocs_dma_t *dma, uint32_t size, uint32_t offset, ocs_hal_dump_get_cb_t cb, void *arg)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	uint8_t *mbxdata;
	ocs_hal_dump_get_cb_arg_t *cb_arg;
	uint32_t opts = (hal->state == OCS_HAL_STATE_ACTIVE ? OCS_CMD_NOWAIT : OCS_CMD_POLL);

	if (SLI4_IF_TYPE_LANCER_FC_ETH != sli_get_if_type(&hal->sli)) {
		ocs_log_test(hal->os, "Function only supported for I/F type 2\n");
		return OCS_HAL_RTN_ERROR;
	}

	if (1 != sli_dump_is_present(&hal->sli)) {
		ocs_log_test(hal->os, "No dump is present\n");
		return OCS_HAL_RTN_ERROR;
	}

	if (1 == sli_reset_required(&hal->sli)) {
		ocs_log_test(hal->os, "device reset required\n");
		return OCS_HAL_RTN_ERROR;
	}

	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_dump_get_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;
	cb_arg->mbox_cmd = mbxdata;

	/* Send the HAL command */
	sli_cmd_common_read_object(&hal->sli, mbxdata, SLI4_BMBX_SIZE, size, offset, "/dbg/dump.bin", dma);
	rc = ocs_hal_command(hal, mbxdata, opts, ocs_hal_cb_dump_get, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	} else if (opts == OCS_CMD_POLL) {
		/* If we're polling, call the callback here */
		rc = ocs_hal_cb_dump_get(hal, 0, mbxdata, cb_arg);
	}

	return rc;
}

/**
 * @brief Called when the OBJECT_DELETE command completes.
 *
 * @par Description
 * Free the mailbox that was malloc'd
 * by ocs_hal_dump_clear(), then call the callback and pass the status.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 * The callback function prototype is <tt>void cb(int32_t status, void *arg)</tt>.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_cb_dump_clear(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	ocs_hal_dump_clear_cb_arg_t *cb_arg = arg;

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_cb_dump_clear_done;

	if ((0 == status) && mbox_rsp->hdr.status)
		status = mbox_rsp->hdr.status;

	cb_arg->cb(status, cb_arg->arg);

ocs_hal_cb_dump_clear_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @brief Clear a dump image from the device.
 *
 * @par Description
 * Creates a SLI_CONFIG mailbox command, fills it with the correct values to clear
 * the dump, then sends the command with ocs_hal_command(). On completion,
 * the callback function ocs_hal_cb_dump_clear() gets called to free the mailbox
 * and to signal the caller that the write has completed.
 *
 * @param hal Hardware context.
 * @param cb Pointer to a callback function that is called when the command completes.
 * The callback function prototype is
 * <tt>void cb(int32_t status, uint32_t bytes_written, void *arg)</tt>.
 * @param arg Pointer to be passed to the callback function.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_dump_clear(ocs_hal_t *hal, ocs_hal_dump_clear_cb_t cb, void *arg)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;
	uint8_t *mbxdata;
	ocs_hal_dump_clear_cb_arg_t *cb_arg;
	uint32_t opts = (hal->state == OCS_HAL_STATE_ACTIVE ? OCS_CMD_NOWAIT : OCS_CMD_POLL);

	if (SLI4_IF_TYPE_LANCER_FC_ETH != sli_get_if_type(&hal->sli)) {
		ocs_log_test(hal->os, "Function only supported for I/F type 2\n");
		return OCS_HAL_RTN_ERROR;
	}

	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_dump_clear_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = arg;
	cb_arg->mbox_cmd = mbxdata;

	/* Send the HAL command */
	sli_cmd_common_delete_object(&hal->sli, mbxdata, SLI4_BMBX_SIZE, "/dbg/dump.bin");
	rc = ocs_hal_command(hal, mbxdata, opts, ocs_hal_cb_dump_clear, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	} else if (opts == OCS_CMD_POLL) {
		/* If we're polling, call the callback here */
		rc = ocs_hal_cb_dump_clear(hal, 0, mbxdata, cb_arg);
	}

	return rc;
}

typedef struct ocs_hal_get_port_protocol_cb_arg_s {
	ocs_get_port_protocol_cb_t cb;
	void *arg;
	uint32_t pci_func;
	ocs_dma_t payload;
} ocs_hal_get_port_protocol_cb_arg_t;

/**
 * @brief Called for the completion of get_port_profile for a
 *        user request.
 *
 * @param hal Hardware context.
 * @param status The status from the MQE.
 * @param mqe Pointer to mailbox command buffer.
 * @param arg Pointer to a callback argument.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
ocs_hal_get_port_protocol_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_get_profile_config_t *sli4_rsp = NULL;
	ocs_hal_get_port_protocol_cb_arg_t *cb_arg = arg;
	ocs_dma_t *payload = NULL;
	ocs_hal_port_protocol_e port_protocol = OCS_HAL_PORT_PROTOCOL_OTHER;
	sli4_resource_descriptor_v1_t *desc_p;
	sli4_pcie_resource_descriptor_v1_t *pcie_desc_p;
	int i, num_descriptors;

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_get_port_protocol_cb_done;

	payload = &cb_arg->payload;
	sli4_rsp = (sli4_res_common_get_profile_config_t *)payload->virt;
	if (sli4_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &sli4_rsp->hdr);

	num_descriptors = sli4_rsp->desc_count;
	desc_p = (sli4_resource_descriptor_v1_t *)sli4_rsp->desc;
	for (i = 0; i < num_descriptors; i++) {
		if (SLI4_RESOURCE_DESCRIPTOR_TYPE_PCIE == desc_p->descriptor_type) {
			pcie_desc_p = (sli4_pcie_resource_descriptor_v1_t *)desc_p;
			if (pcie_desc_p->pf_number == cb_arg->pci_func) {
				switch(pcie_desc_p->pf_type) {
				case 0x02:
					port_protocol = OCS_HAL_PORT_PROTOCOL_ISCSI;
					break;
				case 0x04:
					port_protocol = OCS_HAL_PORT_PROTOCOL_FCOE;
					break;
				case 0x10:
					port_protocol = OCS_HAL_PORT_PROTOCOL_FC;
					break;
				}
			}
		}

		desc_p = (sli4_resource_descriptor_v1_t *)((uint8_t *)desc_p + desc_p->descriptor_length);
	}

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (sli4_rsp->hdr.status)
			status = sli4_rsp->hdr.status;
	}

	cb_arg->cb(status, port_protocol, cb_arg->arg);

ocs_hal_get_port_protocol_cb_done:
	if (cb_arg) {
		ocs_dma_free(hal->os, &cb_arg->payload);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
	}

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @ingroup io
 * @brief  Get the current port protocol.
 * @par Description
 * Issues a SLI4 COMMON_GET_PROFILE_CONFIG mailbox.  When the
 * command completes the provided mgmt callback function is
 * called.
 *
 * @param hal Hardware context.
 * @param pci_func PCI function to query for current protocol.
 * @param cb Callback function to be called when the command completes.
 * @param ul_arg An argument that is passed to the callback function.
 *
 * @return
 * - OCS_HAL_RTN_SUCCESS on success.
 * - OCS_HAL_RTN_NO_MEMORY if a malloc fails.
 * - OCS_HAL_RTN_NO_RESOURCES if unable to get a command
 *   context.
 * - OCS_HAL_RTN_ERROR on any other error.
 */
ocs_hal_rtn_e
ocs_hal_get_port_protocol(ocs_hal_t *hal, uint32_t pci_func,
	ocs_get_port_protocol_cb_t cb, void* ul_arg)
{
	uint8_t *mbxdata;
	ocs_hal_get_port_protocol_cb_arg_t *cb_arg;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* Only supported on Skyhawk */
	if (sli_get_if_type(&hal->sli) != SLI4_IF_TYPE_BE3_SKH_PF) {
		return OCS_HAL_RTN_ERROR;
	}

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* cb_arg holds the data that will be passed to the callback on completion */
	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_get_port_protocol_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = ul_arg;
	cb_arg->pci_func = pci_func;

	/* dma_mem holds the non-embedded portion */
	if (ocs_dma_alloc(hal->os, &cb_arg->payload, 4096, 4)) {
		ocs_log_err(hal->os, "Failed to allocate DMA buffer\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		ocs_free(hal->os, cb_arg, sizeof(ocs_hal_get_port_protocol_cb_arg_t));
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* Send the HAL command */
	sli_cmd_common_get_profile_config(&hal->sli, mbxdata, SLI4_BMBX_SIZE, &cb_arg->payload);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_get_port_protocol_cb, cb_arg);
	if (rc) {
		ocs_dma_free(hal->os, &cb_arg->payload);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;

}

typedef struct ocs_hal_set_port_protocol_cb_arg_s {
	ocs_set_port_protocol_cb_t cb;
	void *arg;
	ocs_dma_t payload;
	uint32_t new_protocol;
	uint32_t pci_func;
} ocs_hal_set_port_protocol_cb_arg_t;

/**
 * @brief Called for the completion of set_port_profile for a
 *        user request.
 *
 * @par Description
 * This is the second of two callbacks for the set_port_protocol
 * function. The set operation is a read-modify-write. This
 * callback is called when the write (SET_PROFILE_CONFIG)
 * completes.
 *
 * @param hal Hardware context.
 * @param status The status from the MQE.
 * @param mqe Pointer to mailbox command buffer.
 * @param arg Pointer to a callback argument.
 *
 * @return 0 on success, non-zero otherwise
 */
static int32_t
ocs_hal_set_port_protocol_cb2(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_set_profile_config_t *sli4_rsp = NULL;
	ocs_hal_set_port_protocol_cb_arg_t *cb_arg = arg;
	ocs_dma_t *payload = NULL;

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_set_port_protocol_cb2_done;

	payload = &cb_arg->payload;
	sli4_rsp = (sli4_res_common_set_profile_config_t *)payload->virt;
	if (sli4_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &sli4_rsp->hdr);

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (sli4_rsp->hdr.status)
			status = sli4_rsp->hdr.status;
	}

	cb_arg->cb(status, cb_arg->arg);

ocs_hal_set_port_protocol_cb2_done:
	if (cb_arg) {
		ocs_dma_free(hal->os, &cb_arg->payload);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
	}

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @brief Called for the completion of set_port_profile for a
 *        user request.
 *
 * @par Description
 * This is the first of two callbacks for the set_port_protocol
 * function.  The set operation is a read-modify-write.  This
 * callback is called when the read completes
 * (GET_PROFILE_CONFG).  It will updated the resource
 * descriptors, then queue the write (SET_PROFILE_CONFIG).
 *
 * On entry there are three memory areas that were allocated by
 * ocs_hal_set_port_protocol.  If a failure is detected in this
 * function those need to be freed.  If this function succeeds
 * it allocates three more areas.
 *
 * @param hal Hardware context.
 * @param status The status from the MQE
 * @param mqe Pointer to mailbox command buffer.
 * @param arg Pointer to a callback argument.
 *
 * @return Returns 0 on success, or a non-zero value otherwise.
 */
static int32_t
ocs_hal_set_port_protocol_cb1(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	ocs_hal_set_port_protocol_cb_arg_t *cb_arg = arg;
	ocs_hal_set_port_protocol_cb_arg_t *new_cb_arg;
	ocs_dma_t *payload = &cb_arg->payload;
	sli4_res_common_get_profile_config_t *response = (sli4_res_common_get_profile_config_t *)payload->virt;
	sli4_resource_descriptor_v1_t *desc_p = (sli4_resource_descriptor_v1_t *)response->desc;
	sli4_pcie_resource_descriptor_v1_t *pcie_desc_p;
	sli4_isap_resouce_descriptor_v1_t *isap_desc_p;
	ocs_hal_port_protocol_e new_protocol = (ocs_hal_port_protocol_e)cb_arg->new_protocol;
	uint8_t *dst, *mbxdata;
	int i, num_descriptors = response->desc_count;
	int pci_descriptor_count = 0;
	int num_fcoe_ports = 0;
	int num_iscsi_ports = 0;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* Count PCI descriptors */
	for (i = 0; i < num_descriptors; i++) {
		if (SLI4_RESOURCE_DESCRIPTOR_TYPE_PCIE == desc_p->descriptor_type)
			++pci_descriptor_count;

		desc_p = (sli4_resource_descriptor_v1_t *)((uint8_t *)desc_p + desc_p->descriptor_length);
	}

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (!mbxdata) {
		ocs_log_err(hal->os, "Failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* cb_arg holds the data that will be passed to the callback on completion */
	new_cb_arg = ocs_malloc(hal->os, sizeof(*new_cb_arg), OCS_M_ZERO | OCS_M_NOWAIT);
	if (!new_cb_arg) {
		ocs_log_err(hal->os, "Failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	new_cb_arg->cb = cb_arg->cb;
	new_cb_arg->arg = cb_arg->arg;

	/**
	 * Allocate memory for the descriptors we're going to send.
	 * i.e. one for each PCI descriptor plus one ISAP descriptor.
	 */
	if (ocs_dma_alloc(hal->os, &new_cb_arg->payload, sizeof(sli4_req_common_set_profile_config_t) +
			  (pci_descriptor_count * sizeof(sli4_pcie_resource_descriptor_v1_t)) +
			  sizeof(sli4_isap_resouce_descriptor_v1_t), 4)) {
		ocs_log_err(hal->os, "Failed to allocate DMA buffer\n");
		ocs_free(hal->os, new_cb_arg, sizeof(*new_cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	sli_cmd_common_set_profile_config(&hal->sli, mbxdata, SLI4_BMBX_SIZE,
			&new_cb_arg->payload, 0, pci_descriptor_count+1, 1);

	/* Point dst to the first descriptor entry in the SET_PROFILE_CONFIG command */
	dst = (uint8_t *)&(((sli4_req_common_set_profile_config_t *)new_cb_arg->payload.virt)->desc);

	/**
	 * Loop over all descriptors. If the descriptor is a PCIe descriptor, copy it
	 * to the SET_PROFILE_CONFIG command to be written back. If it's the descriptor
	 * that we're trying to change, set its pf_type as well.
	 */
	desc_p = (sli4_resource_descriptor_v1_t *)response->desc;
	for (i = 0; i < num_descriptors; i++) {
		if (SLI4_RESOURCE_DESCRIPTOR_TYPE_PCIE == desc_p->descriptor_type) {
			pcie_desc_p = (sli4_pcie_resource_descriptor_v1_t *)desc_p;
			if (pcie_desc_p->pf_number == cb_arg->pci_func) {
				/**
				 * This is the PCIe descriptor for this OCS instance.
				 * Update it with the new pf_type.
				 */
				switch(new_protocol) {
				case OCS_HAL_PORT_PROTOCOL_FC:
					pcie_desc_p->pf_type = SLI4_PROTOCOL_FC;
					break;
				case OCS_HAL_PORT_PROTOCOL_FCOE:
					pcie_desc_p->pf_type = SLI4_PROTOCOL_FCOE;
					break;
				case OCS_HAL_PORT_PROTOCOL_ISCSI:
					pcie_desc_p->pf_type = SLI4_PROTOCOL_ISCSI;
					break;
				default:
					pcie_desc_p->pf_type = SLI4_PROTOCOL_DEFAULT;
					break;
				}
			}

			if (SLI4_PROTOCOL_FCOE == pcie_desc_p->pf_type)
				++num_fcoe_ports;

			if (SLI4_PROTOCOL_ISCSI == pcie_desc_p->pf_type)
				++num_iscsi_ports;

			ocs_memcpy(dst, pcie_desc_p, sizeof(*pcie_desc_p));
			dst += sizeof(*pcie_desc_p);
		}

		desc_p = (sli4_resource_descriptor_v1_t *)((uint8_t *)desc_p + desc_p->descriptor_length);
	}

	/* Create an ISAP resource descriptor */
	isap_desc_p = (sli4_isap_resouce_descriptor_v1_t *)dst;
	isap_desc_p->descriptor_type = SLI4_RESOURCE_DESCRIPTOR_TYPE_ISAP;
	isap_desc_p->descriptor_length = sizeof(sli4_isap_resouce_descriptor_v1_t);
	if (num_iscsi_ports > 0) {
		isap_desc_p->iscsi_tgt = 1;
		isap_desc_p->iscsi_ini = 1;
		isap_desc_p->iscsi_dif = 1;
	}
	if (num_fcoe_ports > 0) {
		isap_desc_p->fcoe_tgt = 1;
		isap_desc_p->fcoe_ini = 1;
		isap_desc_p->fcoe_dif = 1;
	}

	/* At this point we're done with the memory allocated by ocs_port_set_protocol */
	ocs_dma_free(hal->os, &cb_arg->payload);
	ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);

	/* Send a SET_PROFILE_CONFIG mailbox command with the new descriptors */
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_set_port_protocol_cb2, new_cb_arg);
	if (rc) {
		/* Call the upper level callback to report a failure */
		if (new_cb_arg->cb)
			new_cb_arg->cb(rc, new_cb_arg->arg);

		/* Free the memory allocated by this function */
		ocs_dma_free(hal->os, &new_cb_arg->payload);
		ocs_free(hal->os, new_cb_arg, sizeof(*new_cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

/**
 * @ingroup io
 * @brief  Set the port protocol.
 * @par Description
 * Setting the port protocol is a read-modify-write operation.
 * This function submits a GET_PROFILE_CONFIG command to read
 * the current settings.  The callback function will modify the
 * settings and issue the write.
 *
 * On successful completion this function will have allocated
 * two regular memory areas and one dma area which will need to
 * get freed later in the callbacks.
 *
 * @param hal Hardware context.
 * @param new_protocol New protocol to use.
 * @param pci_func PCI function to configure.
 * @param cb Callback function to be called when the command completes.
 * @param ul_arg An argument that is passed to the callback function.
 *
 * @return
 * - OCS_HAL_RTN_SUCCESS on success.
 * - OCS_HAL_RTN_NO_MEMORY if a malloc fails.
 * - OCS_HAL_RTN_NO_RESOURCES if unable to get a command
 *   context.
 * - OCS_HAL_RTN_ERROR on any other error.
 */
ocs_hal_rtn_e
ocs_hal_set_port_protocol(ocs_hal_t *hal, ocs_hal_port_protocol_e new_protocol,
		uint32_t pci_func, ocs_set_port_protocol_cb_t cb, void *ul_arg)
{
	uint8_t *mbxdata;
	ocs_hal_set_port_protocol_cb_arg_t *cb_arg;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;

	/* Only supported on Skyhawk */
	if (sli_get_if_type(&hal->sli) != SLI4_IF_TYPE_BE3_SKH_PF) {
		return OCS_HAL_RTN_ERROR;
	}

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}


	/* cb_arg holds the data that will be passed to the callback on completion */
	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_set_port_protocol_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = ul_arg;
	cb_arg->new_protocol = new_protocol;
	cb_arg->pci_func = pci_func;

	/* dma_mem holds the non-embedded portion */
	if (ocs_dma_alloc(hal->os, &cb_arg->payload, 4096, 4)) {
		ocs_log_err(hal->os, "Failed to allocate DMA buffer\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		ocs_free(hal->os, cb_arg, sizeof(ocs_hal_get_port_protocol_cb_arg_t));
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* Send the HAL command */
	sli_cmd_common_get_profile_config(&hal->sli, mbxdata, SLI4_BMBX_SIZE, &cb_arg->payload);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_set_port_protocol_cb1, cb_arg);
	if (rc) {
		ocs_dma_free(hal->os, &cb_arg->payload);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

typedef struct ocs_hal_get_profile_list_cb_arg_s {
	ocs_get_profile_list_cb_t cb;
	void *arg;
	ocs_dma_t payload;
} ocs_hal_get_profile_list_cb_arg_t;

/**
 * @brief Called for the completion of get_profile_list for a
 *        user request.
 * @par Description
 * This function is called when the COMMMON_GET_PROFILE_LIST
 * mailbox completes.  The response will be in
 * ctx->non_embedded_mem.virt.  This function parses the
 * response and creates a ocs_hal_profile_list, then calls the
 * mgmt_cb callback function and passes that list to it.
 *
 * @param hal Hardware context.
 * @param status The status from the MQE
 * @param mqe Pointer to mailbox command buffer.
 * @param arg Pointer to a callback argument.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
ocs_hal_get_profile_list_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_get_profile_list_t *sli4_rsp = NULL;
	ocs_hal_get_profile_list_cb_arg_t *cb_arg = arg;
	ocs_dma_t *payload = NULL;
	ocs_hal_profile_list_t *list;
	int i, num_descriptors;

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_get_profile_list_cb_done;

	payload = &cb_arg->payload;
	sli4_rsp = (sli4_res_common_get_profile_list_t *)payload->virt;
	if (sli4_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &sli4_rsp->hdr);

	list = ocs_malloc(hal->os, sizeof(*list), OCS_M_ZERO);
	ocs_hal_assert(list);

	list->num_descriptors = sli4_rsp->profile_descriptor_count;
	num_descriptors = list->num_descriptors;
	if (num_descriptors > OCS_HAL_MAX_PROFILES)
		num_descriptors = OCS_HAL_MAX_PROFILES;

	for (i = 0; i < num_descriptors; i++) {
		list->descriptors[i].profile_id = sli4_rsp->profile_descriptor[i].profile_id;
		list->descriptors[i].profile_index = sli4_rsp->profile_descriptor[i].profile_index;
		ocs_strcpy(list->descriptors[i].profile_description,
			(char *)sli4_rsp->profile_descriptor[i].profile_description);
	}

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (sli4_rsp->hdr.status)
			status = sli4_rsp->hdr.status;
	}

	cb_arg->cb(status, list, cb_arg->arg);
	if (status)
		ocs_free(hal->os, list, sizeof(*list));

ocs_hal_get_profile_list_cb_done:
	if (cb_arg) {
		ocs_dma_free(hal->os, &cb_arg->payload);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
	}

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @ingroup io
 * @brief  Get a list of available profiles.
 * @par Description
 * Issues a SLI-4 COMMON_GET_PROFILE_LIST mailbox.  When the
 * command completes the provided mgmt callback function is
 * called.
 *
 * @param hal Hardware context.
 * @param cb Callback function to be called when the
 *		 command completes.
 * @param ul_arg An argument that is passed to the callback
 *		 function.
 *
 * @return
 * - OCS_HAL_RTN_SUCCESS on success.
 * - OCS_HAL_RTN_NO_MEMORY if a malloc fails.
 * - OCS_HAL_RTN_NO_RESOURCES if unable to get a command
 *   context.
 * - OCS_HAL_RTN_ERROR on any other error.
 */
ocs_hal_rtn_e
ocs_hal_get_profile_list(ocs_hal_t *hal, ocs_get_profile_list_cb_t cb, void* ul_arg)
{
	uint8_t *mbxdata;
	ocs_hal_get_profile_list_cb_arg_t *cb_arg;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* Only supported on Skyhawk */
	if (sli_get_if_type(&hal->sli) != SLI4_IF_TYPE_BE3_SKH_PF) {
		return OCS_HAL_RTN_ERROR;
	}

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}


	/* cb_arg holds the data that will be passed to the callback on completion */
	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_get_profile_list_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = ul_arg;

	/* dma_mem holds the non-embedded portion */
	if (ocs_dma_alloc(hal->os, &cb_arg->payload, sizeof(sli4_res_common_get_profile_list_t), 4)) {
		ocs_log_err(hal->os, "Failed to allocate DMA buffer\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		ocs_free(hal->os, cb_arg, sizeof(ocs_hal_get_profile_list_cb_arg_t));
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* Send the HAL command */
	sli_cmd_common_get_profile_list(&hal->sli, mbxdata, SLI4_BMBX_SIZE, 0, &cb_arg->payload);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_get_profile_list_cb, cb_arg);
	if (rc) {
		ocs_dma_free(hal->os, &cb_arg->payload);
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

typedef struct ocs_hal_get_active_profile_cb_arg_s {
	ocs_get_active_profile_cb_t cb;
	void *arg;
} ocs_hal_get_active_profile_cb_arg_t;

/**
 * @brief Called for the completion of get_active_profile for a
 *        user request.
 *
 * @param hal Hardware context.
 * @param status The status from the MQE
 * @param mqe Pointer to mailbox command buffer.
 * @param arg Pointer to a callback argument.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
ocs_hal_get_active_profile_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_get_active_profile_t *sli4_rsp = NULL;
	ocs_hal_get_active_profile_cb_arg_t *cb_arg = arg;
	uint32_t active_profile;

	sli4_rsp = (sli4_res_common_get_active_profile_t *)mbox_rsp->payload.embed;
	if (sli4_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &sli4_rsp->hdr);

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_get_active_profile_cb_done;

	active_profile = sli4_rsp->active_profile_id;

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (sli4_rsp->hdr.status)
			status = sli4_rsp->hdr.status;
	}

	cb_arg->cb(status, active_profile, cb_arg->arg);

ocs_hal_get_active_profile_cb_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @ingroup io
 * @brief  Get the currently active profile.
 * @par Description
 * Issues a SLI-4 COMMON_GET_ACTIVE_PROFILE mailbox. When the
 * command completes the provided mgmt callback function is
 * called.
 *
 * @param hal Hardware context.
 * @param cb Callback function to be called when the
 *	     command completes.
 * @param ul_arg An argument that is passed to the callback
 *      	 function.
 *
 * @return
 * - OCS_HAL_RTN_SUCCESS on success.
 * - OCS_HAL_RTN_NO_MEMORY if a malloc fails.
 * - OCS_HAL_RTN_NO_RESOURCES if unable to get a command
 *   context.
 * - OCS_HAL_RTN_ERROR on any other error.
 */
int32_t
ocs_hal_get_active_profile(ocs_hal_t *hal, ocs_get_active_profile_cb_t cb, void* ul_arg)
{
	uint8_t *mbxdata;
	ocs_hal_get_active_profile_cb_arg_t *cb_arg;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* Only supported on Skyhawk */
	if (sli_get_if_type(&hal->sli) != SLI4_IF_TYPE_BE3_SKH_PF) {
		return OCS_HAL_RTN_ERROR;
	}

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* cb_arg holds the data that will be passed to the callback on completion */
	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_get_active_profile_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = ul_arg;

	/* Send the HAL command */
	sli_cmd_common_get_active_profile(&hal->sli, mbxdata, SLI4_BMBX_SIZE);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_get_active_profile_cb, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

typedef struct ocs_hal_get_nvparms_cb_arg_s {
	ocs_get_nvparms_cb_t cb;
	void *arg;
} ocs_hal_get_nvparms_cb_arg_t;

/**
 * @brief Called for the completion of get_nvparms for a
 *        user request.
 *
 * @param hal Hardware context.
 * @param status The status from the MQE.
 * @param mqe Pointer to mailbox command buffer.
 * @param arg Pointer to a callback argument.
 *
 * @return 0 on success, non-zero otherwise
 */
static int32_t
ocs_hal_get_nvparms_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_read_nvparms_t *mbox_rsp = (sli4_cmd_read_nvparms_t *)mqe;
	ocs_hal_get_nvparms_cb_arg_t *cb_arg = arg;

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_get_nvparms_cb_done;

	if ((0 == status) && mbox_rsp->hdr.status)
		status = mbox_rsp->hdr.status;

	cb_arg->cb(hal->os, status, mbox_rsp->wwpn, mbox_rsp->wwnn, mbox_rsp->hard_alpa,
		   mbox_rsp->preferred_d_id, cb_arg->arg);

ocs_hal_get_nvparms_cb_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @ingroup io
 * @brief  Read non-volatile parms.
 * @par Description
 * Issues a SLI-4 READ_NVPARMS mailbox. When the
 * command completes the provided mgmt callback function is
 * called.
 *
 * @param hal Hardware context.
 * @param cb Callback function to be called when the
 *	  command completes.
 * @param ul_arg An argument that is passed to the callback
 *	  function.
 *
 * @return
 * - OCS_HAL_RTN_SUCCESS on success.
 * - OCS_HAL_RTN_NO_MEMORY if a malloc fails.
 * - OCS_HAL_RTN_NO_RESOURCES if unable to get a command
 *   context.
 * - OCS_HAL_RTN_ERROR on any other error.
 */
int32_t
ocs_hal_get_nvparms(ocs_hal_t *hal, ocs_get_nvparms_cb_t cb, void* ul_arg)
{
	uint8_t *mbxdata;
	ocs_hal_get_nvparms_cb_arg_t *cb_arg;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* cb_arg holds the data that will be passed to the callback on completion */
	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_get_nvparms_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = ul_arg;

	/* Send the HAL command */
	sli_cmd_read_nvparms(&hal->sli, mbxdata, SLI4_BMBX_SIZE);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_get_nvparms_cb, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

typedef struct ocs_hal_set_nvparms_cb_arg_s {
	ocs_set_nvparms_cb_t cb;
	void *arg;
} ocs_hal_set_nvparms_cb_arg_t;

/**
 * @brief Called for the completion of set_nvparms for a
 *        user request.
 *
 * @param hal Hardware context.
 * @param status The status from the MQE.
 * @param mqe Pointer to mailbox command buffer.
 * @param arg Pointer to a callback argument.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
ocs_hal_set_nvparms_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_write_nvparms_t *mbox_rsp = (sli4_cmd_write_nvparms_t *)mqe;
	ocs_hal_set_nvparms_cb_arg_t *cb_arg = arg;

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_set_nvparms_cb_done;

	if ((0 == status) && mbox_rsp->hdr.status)
		status = mbox_rsp->hdr.status;

	cb_arg->cb(status, cb_arg->arg);

ocs_hal_set_nvparms_cb_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @ingroup io
 * @brief  Write non-volatile parms.
 * @par Description
 * Issues a SLI-4 WRITE_NVPARMS mailbox. When the
 * command completes the provided mgmt callback function is
 * called.
 *
 * @param hal Hardware context.
 * @param cb Callback function to be called when the
 *	  command completes.
 * @param wwpn Port's WWPN in big-endian order, or NULL to use default.
 * @param wwnn Port's WWNN in big-endian order, or NULL to use default.
 * @param hard_alpa A hard AL_PA address setting used during loop
 * initialization. If no hard AL_PA is required, set to 0.
 * @param preferred_d_id A preferred D_ID address setting
 * that may be overridden with the CONFIG_LINK mailbox command.
 * If there is no preference, set to 0.
 * @param ul_arg An argument that is passed to the callback
 *	  function.
 *
 * @return
 * - OCS_HAL_RTN_SUCCESS on success.
 * - OCS_HAL_RTN_NO_MEMORY if a malloc fails.
 * - OCS_HAL_RTN_NO_RESOURCES if unable to get a command
 *   context.
 * - OCS_HAL_RTN_ERROR on any other error.
 */
int32_t
ocs_hal_set_nvparms(ocs_hal_t *hal, ocs_set_nvparms_cb_t cb, uint8_t *wwpn,
		uint8_t *wwnn, uint8_t hard_alpa, uint32_t preferred_d_id, void* ul_arg)
{
	uint8_t *mbxdata;
	ocs_hal_set_nvparms_cb_arg_t *cb_arg;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* cb_arg holds the data that will be passed to the callback on completion */
	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_set_nvparms_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = ul_arg;

	/* Send the HAL command */
	sli_cmd_write_nvparms(&hal->sli, mbxdata, SLI4_BMBX_SIZE, wwpn, wwnn, hard_alpa, preferred_d_id);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_set_nvparms_cb, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

/**
 * @brief Called to obtain the count for the specified type.
 *
 * @param hal Hardware context.
 * @param io_count_type IO count type (inuse, free, wait_free).
 *
 * @return Returns the number of IOs on the specified list type.
 */
uint32_t
ocs_hal_io_get_count(ocs_hal_t *hal, ocs_hal_io_count_type_e io_count_type)
{
	ocs_hal_io_t *io = NULL;
	uint32_t count = 0;

	ocs_lock(&hal->io_lock);

	switch (io_count_type) {
	case OCS_HAL_IO_INUSE_COUNT :
		ocs_list_foreach(&hal->io_inuse, io) {
			count++;
		}
		break;
	case OCS_HAL_IO_FREE_COUNT :
		 ocs_list_foreach(&hal->io_free, io) {
			 count++;
		 }
		 break;
	case OCS_HAL_IO_WAIT_FREE_COUNT :
		 ocs_list_foreach(&hal->io_wait_free, io) {
			 count++;
		 }
		 break;
	case OCS_HAL_IO_PORT_OWNED_COUNT:
		 ocs_list_foreach(&hal->io_port_owned, io) {
			 count++;
		 }
		 break;
	case OCS_HAL_IO_N_TOTAL_IO_COUNT :
		count = hal->config.n_io;
		break;
	}

	ocs_unlock(&hal->io_lock);

	return count;
}

/**
 * @brief Called to obtain active RPI count
 *
 * @param hal Hardware context.
 *
 * @return Returns the number of RPIs on the specified list type.
 */
uint32_t
ocs_hal_rpi_active_count(ocs_hal_t *hal)
{
	uint32_t active_rpi_count = 0;

	if (hal->rpi_ref) {
		uint32_t max_rpi, i;

		max_rpi = sli_get_max_rsrc(&hal->sli, SLI_RSRC_FCOE_RPI);
		for (i = 0; i < max_rpi; i ++) {
			if (ocs_atomic_read(&hal->rpi_ref[i].rpi_attached)) {
				active_rpi_count += ocs_atomic_read(&hal->rpi_ref[i].rpi_count);
			}
		}
	}

	return active_rpi_count;
}
/**
 * @brief Called to obtain the count of produced RQs.
 *
 * @param hal Hardware context.
 *
 * @return Returns the number of RQs produced.
 */
uint32_t
ocs_hal_get_rqes_produced_count(ocs_hal_t *hal)
{
	uint32_t count = 0;
	uint32_t i;
	uint32_t j;

	for (i = 0; i < hal->hal_rq_count; i++) {
		hal_rq_t *rq = hal->hal_rq[i];
		if (rq->rq_tracker != NULL) {
			for (j = 0; j < rq->entry_count; j++) {
				if (rq->rq_tracker[j] != NULL) {
					count++;
				}
			}
		}
	}

	return count;
}

typedef struct ocs_hal_set_active_profile_cb_arg_s {
	ocs_set_active_profile_cb_t cb;
	void *arg;
} ocs_hal_set_active_profile_cb_arg_t;

/**
 * @brief Called for the completion of set_active_profile for a
 *        user request.
 *
 * @param hal Hardware context.
 * @param status The status from the MQE
 * @param mqe Pointer to mailbox command buffer.
 * @param arg Pointer to a callback argument.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
ocs_hal_set_active_profile_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_set_active_profile_t *sli4_rsp = NULL;
	ocs_hal_set_active_profile_cb_arg_t *cb_arg = arg;

	sli4_rsp = (sli4_res_common_set_active_profile_t *)mbox_rsp->payload.embed;
	if (sli4_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &sli4_rsp->hdr);

	if (!cb_arg || !cb_arg->cb)
		goto ocs_hal_set_active_profile_cb_done;

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (sli4_rsp->hdr.status)
			status = sli4_rsp->hdr.status;
	}

	cb_arg->cb(status, cb_arg->arg);

ocs_hal_set_active_profile_cb_done:
	if (cb_arg)
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @ingroup io
 * @brief  Set the currently active profile.
 * @par Description
 * Issues a SLI4 COMMON_GET_ACTIVE_PROFILE mailbox. When the
 * command completes the provided mgmt callback function is
 * called.
 *
 * @param hal Hardware context.
 * @param profile_id Profile ID to activate.
 * @param cb Callback function to be called when the command completes.
 * @param ul_arg An argument that is passed to the callback function.
 *
 * @return
 * - OCS_HAL_RTN_SUCCESS on success.
 * - OCS_HAL_RTN_NO_MEMORY if a malloc fails.
 * - OCS_HAL_RTN_NO_RESOURCES if unable to get a command
 *   context.
 * - OCS_HAL_RTN_ERROR on any other error.
 */
int32_t
ocs_hal_set_active_profile(ocs_hal_t *hal, ocs_set_active_profile_cb_t cb, uint32_t profile_id, void* ul_arg)
{
	uint8_t *mbxdata;
	ocs_hal_set_active_profile_cb_arg_t *cb_arg;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;

	/* Only supported on Skyhawk */
	if (sli_get_if_type(&hal->sli) != SLI4_IF_TYPE_BE3_SKH_PF) {
		return OCS_HAL_RTN_ERROR;
	}

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mbxdata == NULL) {
		ocs_log_err(hal->os, "failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}


	/* cb_arg holds the data that will be passed to the callback on completion */
	cb_arg = ocs_malloc(hal->os, sizeof(ocs_hal_set_active_profile_cb_arg_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (cb_arg == NULL) {
		ocs_log_err(hal->os, "failed to malloc cb_arg\n");
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	cb_arg->cb = cb;
	cb_arg->arg = ul_arg;

	/* Send the HAL command */
	sli_cmd_common_set_active_profile(&hal->sli, mbxdata, SLI4_BMBX_SIZE, 0, profile_id);
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT, ocs_hal_set_active_profile_cb, cb_arg);
	if (rc) {
		ocs_free(hal->os, cb_arg, sizeof(*cb_arg));
		ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	}

	return rc;
}

/*
 * Private functions
 */

/**
 * @brief Update the queue hash with the ID and index.
 *
 * @param hash Pointer to hash table.
 * @param id ID that was created.
 * @param index The index into the hash object.
 */
static void
ocs_hal_queue_hash_add(ocs_queue_hash_t *hash, uint16_t id, uint16_t index)
{
	uint32_t	hash_index = id & (OCS_HAL_Q_HASH_SIZE - 1);

	/*
	 * Since the hash is always bigger than the number of queues, then we
	 * never have to worry about an infinite loop.
	 */
	while(hash[hash_index].in_use) {
		hash_index = (hash_index + 1) & (OCS_HAL_Q_HASH_SIZE - 1);
	}

	/* not used, claim the entry */
	hash[hash_index].id = id;
	hash[hash_index].in_use = 1;
	hash[hash_index].index = index;
}

/**
 * @brief Find index given queue ID.
 *
 * @param hash Pointer to hash table.
 * @param id ID to find.
 *
 * @return Returns the index into the HAL cq array or -1 if not found.
 */
int32_t
ocs_hal_queue_hash_find(ocs_queue_hash_t *hash, uint16_t id)
{
	int32_t	rc = -1;
	int32_t	index = id & (OCS_HAL_Q_HASH_SIZE - 1);

	/*
	 * Since the hash is always bigger than the maximum number of Qs, then we
	 * never have to worry about an infinite loop. We will always find an
	 * unused entry.
	 */
	do {
		if (hash[index].in_use &&
		    hash[index].id == id) {
			rc = hash[index].index;
		} else {
			index = (index + 1) & (OCS_HAL_Q_HASH_SIZE - 1);
		}
	} while(rc == -1 && hash[index].in_use);

	return rc;
}

static int32_t
ocs_hal_domain_add(ocs_hal_t *hal, ocs_domain_t *domain)
{
	int32_t		rc = OCS_HAL_RTN_ERROR;
	uint16_t	fcfi = UINT16_MAX;

	if ((hal == NULL) || (domain == NULL)) {
		ocs_log_err(NULL, "bad parameter hal=%p domain=%p\n", hal, domain);
		return OCS_HAL_RTN_ERROR;
	}

	fcfi = domain->fcf_indicator;

	if (fcfi < SLI4_MAX_FCFI) {
		uint16_t	fcf_index = UINT16_MAX;

		ocs_log_debug(hal->os, "adding domain %p @ %#x\n", domain, fcfi);
		hal->domains[fcfi] = domain;

		/* HAL_WORKAROUND_OVERRIDE_FCFI_IN_SRB */
		if (hal->workaround.override_fcfi) {
			if (hal->first_domain_idx < 0) {
				hal->first_domain_idx = fcfi;
			}
		}

		fcf_index = domain->fcf;

		if (fcf_index < SLI4_MAX_FCF_INDEX) {
			ocs_log_debug(hal->os, "adding map of FCF index %d to FCFI %d\n", fcf_index, fcfi);
			hal->fcf_index_fcfi[fcf_index] = fcfi;
			rc = OCS_HAL_RTN_SUCCESS;
		} else {
			ocs_log_test(hal->os, "FCF index %d out of range (max %d)\n", fcf_index, SLI4_MAX_FCF_INDEX);
			hal->domains[fcfi] = NULL;
		}
	} else {
		ocs_log_test(hal->os, "FCFI %#x out of range (max %#x)\n", fcfi, SLI4_MAX_FCFI);
	}

	return rc;
}

static int32_t
ocs_hal_domain_del(ocs_hal_t *hal, ocs_domain_t *domain)
{
	int32_t		rc = OCS_HAL_RTN_ERROR;
	uint16_t	fcfi = UINT16_MAX;

	if ((hal == NULL) || (domain == NULL)) {
		ocs_log_err(NULL, "bad parameter hal=%p domain=%p\n", hal, domain);
		return OCS_HAL_RTN_ERROR;
	}

	fcfi = domain->fcf_indicator;

	if (fcfi < SLI4_MAX_FCFI) {
		uint16_t	fcf_index = UINT16_MAX;

		ocs_log_debug(hal->os, "deleting domain %p @ %#x\n", domain, fcfi);

		if (domain != hal->domains[fcfi]) {
			ocs_log_test(hal->os, "provided domain %p does not match stored domain %p\n",
				     domain, hal->domains[fcfi]);
			return OCS_HAL_RTN_ERROR;
		}

		hal->domains[fcfi] = NULL;

		/* HAL_WORKAROUND_OVERRIDE_FCFI_IN_SRB */
		if (hal->workaround.override_fcfi) {
			if (hal->first_domain_idx == fcfi) {
				hal->first_domain_idx = -1;
			}
		}

		fcf_index = domain->fcf;

		if (fcf_index < SLI4_MAX_FCF_INDEX) {
			if (hal->fcf_index_fcfi[fcf_index] == fcfi) {
				hal->fcf_index_fcfi[fcf_index] = 0;
				rc = OCS_HAL_RTN_SUCCESS;
			} else {
				ocs_log_test(hal->os, "indexed FCFI %#x doesn't match provided %#x @ %d\n",
					     hal->fcf_index_fcfi[fcf_index], fcfi, fcf_index);
			}
		} else {
			ocs_log_test(hal->os, "FCF index %d out of range (max %d)\n", fcf_index, SLI4_MAX_FCF_INDEX);
		}
	} else {
		ocs_log_test(hal->os, "FCFI %#x out of range (max %#x)\n", fcfi, SLI4_MAX_FCFI);
	}

	return rc;
}

ocs_domain_t *
ocs_hal_domain_get(ocs_hal_t *hal, uint16_t fcfi)
{

	if (hal == NULL) {
		ocs_log_err(NULL, "bad parameter hal=%p\n", hal);
		return NULL;
	}

	if (fcfi < SLI4_MAX_FCFI) {
		return hal->domains[fcfi];
	} else {
		ocs_log_test(hal->os, "FCFI %#x out of range (max %#x)\n", fcfi, SLI4_MAX_FCFI);
		return NULL;
	}
}

static ocs_domain_t *
ocs_hal_domain_get_indexed(ocs_hal_t *hal, uint16_t fcf_index)
{

	if (hal == NULL) {
		ocs_log_err(NULL, "bad parameter hal=%p\n", hal);
		return NULL;
	}

	if (fcf_index < SLI4_MAX_FCF_INDEX) {
		return ocs_hal_domain_get(hal, hal->fcf_index_fcfi[fcf_index]);
	} else {
		ocs_log_test(hal->os, "FCF index %d out of range (max %d)\n", fcf_index, SLI4_MAX_FCF_INDEX);
		return NULL;
	}
}

/**
 * @brief Quaratine an IO by taking a reference count and adding it to the
 *        quarantine list. When the IO is popped from the list then the
 *        count is released and the IO MAY be freed depending on whether
 *        it is still referenced by the IO.
 *
 *        @n @b Note: BZ 160124 - If this is a target write or an initiator read using
 *        DIF, then we must add the XRI to a quarantine list until we receive
 *        4 more completions of this same type.
 *
 * @param hal Hardware context.
 * @param wq Pointer to the WQ associated with the IO object to quarantine.
 * @param io Pointer to the io object to quarantine.
 */
static void
ocs_hal_io_quarantine(ocs_hal_t *hal, hal_wq_t *wq, ocs_hal_io_t *io)
{
	ocs_quarantine_info_t *q_info = &wq->quarantine_info;
	uint32_t	index;
	ocs_hal_io_t	*free_io = NULL;

	/* return if the QX bit was clear */
	if (!io->quarantine) {
		return;
	}

	/* increment the IO refcount to prevent it from being freed before the quarantine is over */
	if (ocs_ref_get_unless_zero(&io->ref) == 0) {
		/* command no longer active */
		ocs_log_debug(hal ? hal->os : NULL, "io not active xri=%#x tag=%#x\n", io->indicator, io->reqtag);
		return;
	}

	sli_queue_lock(wq->queue);
		index = q_info->quarantine_index;
		free_io = q_info->quarantine_ios[index];
		q_info->quarantine_ios[index] = io;
		q_info->quarantine_index = (index + 1) % OCS_HAL_QUARANTINE_QUEUE_DEPTH;
	sli_queue_unlock(wq->queue);

	if (free_io != NULL) {
		ocs_ref_put(&free_io->ref); /* ocs_ref_get(): same function */
	}
}

/**
 * @brief Process entries on the given completion queue.
 *
 * @param hal Hardware context.
 * @param cq Pointer to the HAL completion queue object.
 *
 * @return None.
 */
void
ocs_hal_cq_process(ocs_hal_t *hal, hal_cq_t *cq)
{
	uint8_t		cqe[sizeof(sli4_mcqe_t)];
	uint16_t	rid = UINT16_MAX;
	sli4_qentry_e	ctype;		/* completion type */
	int32_t		status;
	uint32_t	n_processed = 0;
	time_t		tstart;
	time_t		telapsed;

	tstart = ocs_msectime();

	while (!sli_queue_read(&hal->sli, cq->queue, cqe)) {
		status = sli_cq_parse(&hal->sli, cq->queue, cqe, &ctype, &rid);
		/*
		 * The sign of status is significant. If status is:
		 * == 0 : call completed correctly and the CQE indicated success
		 *  > 0 : call completed correctly and the CQE indicated an error
		 *  < 0 : call failed and no information is available about the CQE
		 */
		if (status < 0) {
			if (status == -2) {
				/* Notification that an entry was consumed, but not completed */
				continue;
			}

			break;
		}

		switch (ctype) {
		case SLI_QENTRY_ASYNC:
			CPUTRACE("async");
			sli_cqe_async(&hal->sli, cqe);
			break;
		case SLI_QENTRY_MQ:
			/*
			 * Process MQ entry. Note there is no way to determine
			 * the MQ_ID from the completion entry.
			 */
			CPUTRACE("mq");
			ocs_hal_mq_process(hal, status, hal->mq[0]);
			break;
		case SLI_QENTRY_OPT_WRITE_CMD:
			ocs_hal_rqpair_tow_cmd_process(hal, cq, cqe);
			break;
		case SLI_QENTRY_OPT_WRITE_DATA:
			ocs_hal_rqpair_tow_data_process(hal, cq, cqe);
			break;
		case SLI_QENTRY_WQ:
			CPUTRACE("wq");
			ocs_hal_wq_process(hal, cq, cqe, status, rid);
			break;
		case SLI_QENTRY_WQ_RELEASE: {
			uint32_t wq_id = rid;
			int32_t index = ocs_hal_queue_hash_find(hal->wq_hash, wq_id);
			hal_wq_t *wq;

			if (index < 0) {
				ocs_log_err(hal->os, "unhandled ctype=%#x, rid lookup failed for id=%#x\n", ctype, rid);
				break;
			}

			wq = hal->hal_wq[index];

			/* Submit any HAL IOs that are on the WQ pending list */
			hal_wq_submit_pending(wq, wq->wqec_set_count);

			break;
		}

		case SLI_QENTRY_RQ:
			CPUTRACE("rq");
			ocs_hal_rqpair_process_rq(hal, cq, cqe);
			break;
		case SLI_QENTRY_XABT: {
			CPUTRACE("xabt");
			ocs_hal_xabt_process(hal, cq, cqe, rid);
			break;

		}
		default:
			ocs_log_test(hal->os, "unhandled ctype=%#x rid=%#x\n", ctype, rid);
			break;
		}

		n_processed++;
		if (n_processed == cq->queue->proc_limit) {
			break;
		}

		if (cq->queue->n_posted >= (cq->queue->posted_limit)) {
			sli_queue_arm(&hal->sli, cq->queue, FALSE);
		}
	}

	sli_queue_arm(&hal->sli, cq->queue, TRUE);

	cq->queue->last_processed_time = ocs_ticks_to_ms(ocs_get_os_ticks());

	if (n_processed > cq->queue->max_num_processed) {
		cq->queue->max_num_processed = n_processed;
	}

	telapsed = ocs_msectime() - tstart;
	if (telapsed > cq->queue->max_process_time) {
		cq->queue->max_process_time = telapsed;
	}
}

/**
 * @brief Process WQ completion queue entries.
 *
 * @param hal Hardware context.
 * @param cq Pointer to the HAL completion queue object.
 * @param cqe Pointer to WQ completion queue.
 * @param status Completion status.
 * @param rid Resource ID (IO tag).
 *
 * @return none
 */
void
ocs_hal_wq_process(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe, int32_t status, uint16_t rid)
{
	hal_wq_callback_t *wqcb;

	ocs_queue_history_cqe(&hal->q_hist, SLI_QENTRY_WQ, (void *)cqe, ((sli4_fc_wcqe_t *)cqe)->status,
				cq->queue->id, ((cq->queue->index - 1) & (cq->queue->length - 1)));

	wqcb = ocs_hal_reqtag_get_instance(hal, rid);
	if (wqcb == NULL) {
		ocs_log_err(hal->os, "invalid request tag: x%x\n", rid);
		return;
	}

	if (wqcb->callback == NULL) {
		ocs_log_err(hal->os, "wqcb callback is NULL\n");
		ocs_hal_assert(0);
		return;
	}

	(*wqcb->callback)(wqcb->arg, cqe, status);
}

/**
 * @brief Process WQ completions for IO requests
 *
 * @param arg Generic callback argument
 * @param cqe Pointer to completion queue entry
 * @param status Completion status
 *
 * @par Description
 * @n @b Note:  Regarding io->reqtag, the reqtag is assigned once when HAL IOs are initialized
 * in ocs_hal_setup_io(), and don't need to be returned to the hal->wq_reqtag_pool.
 *
 * @return None.
 */
static void
ocs_hal_wq_process_io(void *arg, uint8_t *cqe, int32_t status)
{
	ocs_hal_io_t *io = arg;
	ocs_hal_t *hal = io->hal;
	sli4_fc_wcqe_t *wcqe = (void *)cqe;
	uint32_t len = 0;
	uint32_t ext = 0;
	bool tow_ooo_cmd = FALSE;
	bool tow_ooo_data = FALSE;
	bool tow_lock_taken = FALSE;

	/* Remove from the io_timed_wqe list */
	ocs_hal_remove_io_timed_wqe(hal, io);

	/*
	 * For the primary IO, this will also be used for the response. So, it
	 * is important to only set/clear this flag on the first data phase of
	 * the IO because subsequent phases will be done on the secondary XRI.
	 */
	if (io->quarantine && io->quarantine_first_phase) {
		io->quarantine = (wcqe->qx == 1);
		ocs_hal_io_quarantine(hal, io->wq, io);
	}
	io->quarantine_first_phase = FALSE;

	/* BZ 161832 - free secondary HAL IO */
	if (io->sec_hio && io->sec_hio->quarantine) {
		/*
		 * If the quarantine flag is set on the IO, then set it on the
		 * secondary IO based on the quarantine XRI (QX) bit sent by the FW.
		 */
		io->sec_hio->quarantine = (wcqe->qx == 1);
		/* Use the primary io->wq because it is not set on the secondary IO */
		ocs_hal_io_quarantine(hal, io->wq, io->sec_hio);
	}

	/* Clear xbusy flag if WCQE[XB] is clear */
	if (io->xbusy && !wcqe->xb)
		io->xbusy = FALSE;

	/* Get extended CQE status */
	switch (io->type) {
		case OCS_HAL_ELS_REQ:
			sli_fc_els_did(&hal->sli, cqe, &ext);
			len = sli_fc_response_length(&hal->sli, cqe);
			break;

		case OCS_HAL_ELS_RSP:
		case OCS_HAL_ELS_RSP_SID:
		case OCS_HAL_BLS_ACC:
		case OCS_HAL_BLS_ACC_SID:
		case OCS_HAL_BLS_RJT:
			break;

		case OCS_HAL_FC_CT:
			len = sli_fc_response_length(&hal->sli, cqe);
			break;

		case OCS_HAL_FC_CT_RSP:
		case OCS_HAL_IO_DNRX_REQUEUE:
		case OCS_HAL_IO_INITIATOR_NODATA:
			break;

		case OCS_HAL_IO_INITIATOR_READ:
		case OCS_HAL_IO_INITIATOR_WRITE:
		case OCS_HAL_IO_TARGET_WRITE:
			len = sli_fc_io_length(&hal->sli, cqe);
			break;

		case OCS_HAL_IO_TARGET_READ:
			len = sli_fc_io_length(&hal->sli, cqe);

			/*
			 * Lancer G6 seems to return 0 "total length placed"
			 * on FCP_TSEND64_WQE completions. If this appears to
			 * happen, use the CTIO data transfer length instead.
			 */
			if (hal->workaround.retain_tsend_io_length && !len && !status)
				len = io->length;

			break;

		case OCS_HAL_IO_TARGET_RSP:
			if (io->is_port_owned) {
				ocs_lock(&io->tow_lock);
				tow_lock_taken = TRUE;
				if (io->tow_buf->call_tow_cmd)
					tow_ooo_cmd = TRUE;

				if (io->tow_buf->call_tow_data)
					tow_ooo_data = TRUE;
			}
			break;

		default:
			ocs_log_err(hal->os, "Unhandled io type %#x for io=%p xri=%#x tag=%#x\n",
					io->type, io, io->indicator, io->reqtag);
			ocs_hal_assert(0);
			return;
	}

	if (status) {
		ext = sli_fc_ext_status(&hal->sli, cqe);
		ocs_log_err(hal->os, "HAL IO error WCQE, io=%p xri=%#x tag=%#x type=%d status=%#x ext_status=%#x\n",
				io, io->indicator, io->reqtag, io->type, status, ext);
	}

	if (wcqe->xb && wcqe->rha) {
		ocs_hal_rtn_e rc;
		bool send_abts = ocs_hal_io_can_post_abts(status, ext);

		/* If this is a target command, make sure to suppress any further WQE's */
		ocs_log_info(hal->os, "Aborting io=%p xri=%#x tag=%#x type=%d\n",
				io, io->indicator, io->reqtag, io->type);

		rc = ocs_hal_io_abort(hal, io, send_abts, NULL, NULL);
		if ((OCS_HAL_RTN_SUCCESS != rc) && (OCS_HAL_RTN_IO_ABORT_IN_PROGRESS != rc)) {
			/* Failed to abort for some other reason, log error */
			ocs_log_err(hal->os, "Failed to abort io=%p xri=%#x tag=%#x type=%d rc=%d\n",
					io, io->indicator, io->reqtag, io->type, rc);
		}
	}

	/* BZ 161832 - free secondary HAL IO */
	if (io->sec_hio) {
		ocs_hal_io_free(hal, io->sec_hio);
		io->sec_hio = NULL;
	}

	/* Invoke the IO callback */
	if (io->done) {
		ocs_hal_done_t	done = io->done;
		void		*arg = io->arg;

		io->done = NULL;

		/* Restore default SGL */
		ocs_hal_io_restore_sgl(hal, io);
		done(io, io->rnode, len, status, ext, arg);
	}

	if (tow_ooo_cmd) {
		/* Bounce enabled, single RQ, we snoop the ox_id to choose the cpuidx */
		if (hal->config.bounce) {
			fc_header_t *hdr = io->tow_buf->cmd_seq->header.data;
			uint32_t s_id = fc_be24toh(hdr->s_id);
			uint32_t d_id = fc_be24toh(hdr->d_id);
			uint32_t ox_id = ocs_be16toh(hdr->ox_id);

			if (hal->callback.bounce)
				(*hal->callback.bounce)(ocs_hal_unsol_process_bounce, io->tow_buf->cmd_seq, s_id, d_id, ox_id);
		} else {
			hal->callback.unsolicited(hal->args.unsolicited, io->tow_buf->cmd_seq);
		}

		if (tow_ooo_data) {
			/* Bounce enabled, single RQ, we snoop the ox_id to choose the cpuidx */
			if (hal->config.bounce) {
				fc_header_t *hdr = io->tow_buf->seq.header.data;
				uint32_t s_id = fc_be24toh(hdr->s_id);
				uint32_t d_id = fc_be24toh(hdr->d_id);
				uint32_t ox_id = ocs_be16toh(hdr->ox_id);

				if (hal->callback.bounce)
					(*hal->callback.bounce)(ocs_hal_unsol_process_bounce, &io->tow_buf->seq, s_id, d_id, ox_id);
			} else {
				hal->callback.unsolicited(hal->args.unsolicited, &io->tow_buf->seq);
			}
		}
	}

	if (tow_lock_taken)
		ocs_unlock(&io->tow_lock);
}

/**
 * @brief Process WQ completions for abort requests.
 *
 * @param arg Generic callback argument.
 * @param cqe Pointer to completion queue entry.
 * @param status Completion status.
 *
 * @return None.
 */
static void
ocs_hal_wq_process_abort(void *arg, uint8_t *cqe, int32_t status)
{
	ocs_hal_io_t *io = arg;
	hal_wq_callback_t *wqcb;
	uint32_t ext = sli_fc_ext_status(&io->hal->sli, cqe);

	ocs_log_info(io->hal->os, "HAL IO abort WCQE, io=%p xri=%#x tag=%#x abort_reqtag= %#x type=%d status=%#x ext_status=%#x\n",
				io, io->indicator, io->reqtag, io->abort_reqtag, io->type, status, ext);
	ocs_hal_assert(OCS_HAL_IO_STATE_WAIT_FREE != io->state);

	/* Invoke the abort_done callback */
	if (io->abort_done) {
		ocs_hal_done_t	abort_done = io->abort_done;
		void		*arg = io->abort_arg;

		io->abort_done = NULL;
		abort_done(io, io->rnode, 0, status, ext, arg);
	}

	/* Free the abort WQE reqtag */
	ocs_hal_assert(UINT32_MAX != io->abort_reqtag);
	wqcb = ocs_hal_reqtag_get_instance(io->hal, io->abort_reqtag);
	ocs_hal_reqtag_free(io->hal, wqcb);

	/*
	 * Call ocs_hal_io_free() because this releases the WQ reservation as
	 * well as doing the refcount put. Don't duplicate the code here.
	 */
	(void)ocs_hal_io_free(io->hal, io);
}

/**
 * @brief Process XABT completions
 *
 * @param hal Hardware context.
 * @param cq Pointer to the HAL completion queue object.
 * @param cqe Pointer to WQ completion queue.
 * @param rid Resource ID (IO tag).
 *
 * @return None.
 */
void
ocs_hal_xabt_process(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe, uint16_t rid)
{
	ocs_hal_io_t *io = NULL;

	ocs_queue_history_cqe(&hal->q_hist, SLI_QENTRY_XABT, (void *)cqe, 0, cq->queue->id,
			      ((cq->queue->index - 1) & (cq->queue->length - 1)));

	io = ocs_hal_io_lookup(hal, rid);
	ocs_hal_assert(io);

	ocs_log_info(hal->os, "HAL IO XRI_ABORTED CQE, io=%p xri=%#x tag=%#x type=%d rid=%#x xbusy=%d\n",
		     io, io->indicator, io->reqtag, io->type, rid, io->xbusy);
	if (io->xbusy)
		io->xbusy = FALSE;
	else
		ocs_hal_assert(io->is_port_owned);

	/* Check if this is a port owned XRI */
	if (io->is_port_owned) {
		ocs_lock(&hal->io_lock);
			ocs_hal_reque_xri(hal, io);
		ocs_unlock(&hal->io_lock);

		/* We're not handling the requeue xri completion, just return here */
		return;
	}

	ocs_lock(&hal->io_lock);
		if (OCS_HAL_IO_STATE_WAIT_FREE == io->state) {
			/*
			 * If we're on the wait_free list, then the caller has already freed
			 * the IO. So, move the IO from the wait_free list to the free list.
			 */
			io->state = OCS_HAL_IO_STATE_FREE;
			ocs_list_remove(&hal->io_wait_free, io);
			ocs_hal_io_free_move_correct_list(hal, io);
		}
	ocs_unlock(&hal->io_lock);
}

/**
 * @brief Adjust the number of WQs and CQs within the HAL.
 *
 * @par Description
 * Calculates the number of WQs and associated CQs needed in the HAL based on
 * the number of IOs. Calculates the starting CQ index for each WQ, RQ and
 * MQ.
 *
 * @param hal Hardware context allocated by the caller.
 */
static void
ocs_hal_adjust_wqs(ocs_hal_t *hal)
{
	uint32_t max_wq_num = sli_get_max_queue(&hal->sli, SLI_QTYPE_WQ);
	uint32_t max_wq_entries = hal->num_qentries[SLI_QTYPE_WQ];
	uint32_t max_cq_entries = hal->num_qentries[SLI_QTYPE_CQ];

	/*
	 * possibly adjust the the size of the WQs so that the CQ is twice as
	 * big as the WQ to allow for 2 completions per IO. This allows us to
	 * handle multi-phase as well as aborts.
	 */
	if (max_cq_entries < max_wq_entries * 2) {
		max_wq_entries = hal->num_qentries[SLI_QTYPE_WQ] = max_cq_entries / 2;
	}

	/*
	 * Calculate the number of WQs to use base on the number of IOs.
	 *
	 * Note: We need to reserve room for aborts which must be sent down
	 *       the same WQ as the IO. So we allocate enough WQ space to
	 *       handle 2 times the number of IOs. Half of the space will be
	 *       used for normal IOs and the other half is reserved for aborts.
	 */
	hal->config.n_wq = ((hal->config.n_io * 2) + (max_wq_entries - 1)) / max_wq_entries;

	/*
	 * For performance reasons, it is best to use use a minimum of 4 WQs
	 * for BE3 and Skyhawk.
	 */
	if (hal->config.n_wq < 4 &&
	    SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(&hal->sli)) {
		hal->config.n_wq = 4;
	}

	/*
	 * For dual-chute support, we need to have at least one WQ per chute.
	 */
	if (hal->config.n_wq < 2 &&
	    ocs_hal_get_num_chutes(hal) > 1) {
		hal->config.n_wq = 2;
	}

	/* make sure we haven't exceeded the max supported in the HAL */
	if (hal->config.n_wq > OCS_HAL_MAX_NUM_WQ) {
		hal->config.n_wq = OCS_HAL_MAX_NUM_WQ;
	}

	/* make sure we haven't exceeded the chip maximum */
	if (hal->config.n_wq > max_wq_num) {
		hal->config.n_wq = max_wq_num;
	}

	/*
	 * Using Queue Topology string, we divide by number of chutes
	 */
	hal->config.n_wq /= ocs_hal_get_num_chutes(hal);
}

static int32_t
ocs_hal_command_process(ocs_hal_t *hal, int32_t status, uint8_t *mqe, size_t size)
{
	ocs_command_ctx_t *ctx = NULL;
	sli4_mbox_command_header_t *hdr = (sli4_mbox_command_header_t *)mqe;

	ocs_lock(&hal->cmd_lock);
		if (NULL == (ctx = ocs_list_remove_head(&hal->cmd_head))) {
			ocs_log_err(hal->os, "Can't find the MQE command context!!!\n");
			ocs_unlock(&hal->cmd_lock);
			return -1;
		}

		hal->cmd_head_count--;

		/* Post any pending requests */
		ocs_hal_cmd_submit_pending(hal);
	ocs_unlock(&hal->cmd_lock);

	if (status || hdr->status) {
		ocs_log_err(hal->os, "MQE failed; mqe_hdr: %p, mqe_cmd: %#x, mqe_status: %#x, cqe_status: %#x\n",
			    hdr, hdr->command, hdr->status, status);
		ocs_dump32(OCS_DEBUG_ALWAYS, hal->os, "MBOX command buff dump", mqe, SLI4_BMBX_SIZE);
	}

	if (ctx->cb) {
		if (ctx->buf)
			ocs_memcpy(ctx->buf, mqe, size);

		ctx->cb(hal, status, ctx->buf, ctx->arg);
	}

	ocs_memset(ctx, 0, sizeof(*ctx));
	ocs_free(hal->os, ctx, sizeof(*ctx));
	return 0;
}

/**
 * @brief Process entries on the given mailbox queue.
 *
 * @param hal Hardware context.
 * @param status CQE status.
 * @param mq Pointer to the mailbox queue object.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
ocs_hal_mq_process(ocs_hal_t *hal, int32_t status, sli4_queue_t *mq)
{
	uint8_t		mqe[SLI4_BMBX_SIZE];

	if (!sli_queue_read(&hal->sli, mq, mqe)) {
		ocs_hal_command_process(hal, status, mqe, mq->size);
	}

	return 0;
}

/**
 * @brief Read a FCF table entry.
 *
 * @param hal Hardware context.
 * @param index Table index to read. Use SLI4_FCOE_FCF_TABLE_FIRST for the first
 * read and the next_index field from the FCOE_READ_FCF_TABLE command
 * for subsequent reads.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static ocs_hal_rtn_e
ocs_hal_read_fcf(ocs_hal_t *hal, uint32_t index)
{
	ocs_dma_t	*dma = NULL;
	uint8_t		*buf = NULL;
	int32_t		rc = OCS_HAL_RTN_ERROR;

	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (!buf) {
		ocs_log_err(hal->os, "no buffer for command\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	dma = ocs_malloc(hal->os, sizeof(*dma), OCS_M_ZERO | OCS_M_NOWAIT);
	if (dma == NULL) {
		ocs_log_err(hal->os, "ocs_malloc(ocs_dma_t) failed\n");
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	if (ocs_dma_alloc(hal->os, dma, OCS_HAL_READ_FCF_SIZE, OCS_HAL_READ_FCF_SIZE)) {
		ocs_log_err(hal->os, "ocs_dma_alloc() failed\n");
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
		ocs_free(hal->os, dma, sizeof(ocs_dma_t));
		return OCS_HAL_RTN_ERROR;
	}

	if (sli_cmd_fcoe_read_fcf_table(&hal->sli, buf, SLI4_BMBX_SIZE, dma, index))
		rc = ocs_hal_command(hal, buf, OCS_CMD_NOWAIT, ocs_hal_cb_read_fcf, dma);

	if (rc) {
		ocs_log_test(hal->os, "FCOE_READ_FCF_TABLE failed\n");
		ocs_dma_free(hal->os, dma);
		ocs_free(hal->os, dma, sizeof(*dma));
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
	}

	return rc;
}

/**
 * @brief Callback function for the FCOE_READ_FCF_TABLE command.
 *
 * @par Description
 * Note that the caller has allocated:
 *  - DMA memory to hold the table contents
 *  - DMA memory structure
 *  - Command/results buffer
 *  .
 * Each of these must be freed here.
 *
 * @param hal Hardware context.
 * @param status Hardware status.
 * @param mqe Pointer to the mailbox command/results buffer.
 * @param arg Pointer to the DMA memory structure.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
ocs_hal_cb_read_fcf(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_fcoe_read_fcf_table_t *read_fcf_rsp = NULL;
	ocs_dma_t *payload = arg;

	read_fcf_rsp = (sli4_res_fcoe_read_fcf_table_t *)payload->virt;
	if (!read_fcf_rsp)
		goto ocs_hal_cb_read_fcf_done;

	if (read_fcf_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &read_fcf_rsp->hdr);

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (read_fcf_rsp->hdr.status)
			status = read_fcf_rsp->hdr.status;
	}

	if (status)
		goto ocs_hal_cb_read_fcf_done;

	/* If either FC / FCOE and FCF entry is valid, process it */
	if (read_fcf_rsp->fcf_entry.fc || (read_fcf_rsp->fcf_entry.val && !read_fcf_rsp->fcf_entry.sol)) {
		if (hal->callback.domain) {
			ocs_domain_record_t drec = { 0 };

			if (read_fcf_rsp->fcf_entry.fc) {
				/*
				 * This is a pseudo FCF entry. Create a domain
				 * record based on the read topology information.
				 */
				drec.speed = hal->link.speed;
				drec.fc_id = hal->link.fc_id;
				drec.is_fc = TRUE;
				if (SLI_LINK_TOPO_LOOP == hal->link.topology) {
					drec.is_loop = TRUE;
					ocs_memcpy(drec.map.loop, hal->link.loop_map, sizeof(drec.map.loop));
				} else if (SLI_LINK_TOPO_NPORT == hal->link.topology) {
					drec.is_nport = TRUE;
				}
			} else {
				drec.index = read_fcf_rsp->fcf_entry.fcf_index;
				drec.priority = read_fcf_rsp->fcf_entry.fip_priority;

				/* Copy address, wwn and vlan_bitmap */
				ocs_memcpy(drec.address, read_fcf_rsp->fcf_entry.fcf_mac_address, sizeof(drec.address));
				ocs_memcpy(drec.wwn, read_fcf_rsp->fcf_entry.fabric_name_id, sizeof(drec.wwn));
				ocs_memcpy(drec.map.vlan, read_fcf_rsp->fcf_entry.vlan_bitmap, sizeof(drec.map.vlan));

				drec.is_ethernet = TRUE;
				drec.is_nport = TRUE;
			}

			hal->callback.domain(hal->args.domain, OCS_HAL_DOMAIN_FOUND, &drec);
		}
	} else {
		ocs_log_test(hal->os, "Ignoring the invalid FCF entry\n");
	}

	if (SLI4_FCOE_FCF_TABLE_LAST != read_fcf_rsp->next_index)
		ocs_hal_read_fcf(hal, read_fcf_rsp->next_index);

ocs_hal_cb_read_fcf_done:
	ocs_dma_free(hal->os, payload);
	ocs_free(hal->os, payload, sizeof(*payload));
	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @brief callback function to read link module type
 *
 * @param ctx Hardware context pointer
 * @param MQE completion status
 * @param MQE data buffer
 * @param optional argument
 * @return Returns 0 on success, or a non-zero value on failure.
 */
void
__ocs_hal_read_lmt_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_res_read_config_t *mbox_rsp = (sli4_res_read_config_t *)mqe;

	if ((0 == status) && mbox_rsp->hdr.status)
		status = mbox_rsp->hdr.status;

	if (0 == status)
		hal->sli.config.link_module_type = mbox_rsp->lmt;

	if (hal->callback.xport)
		hal->callback.xport(hal->args.xport, OCS_HAL_XPORT_FC_SPEED_UPDATE);

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
}

/**
 * @brief function to read link module type
 *
 * @par Description
 * This function allocates memory which must be freed in its callback.
 *
 * @param ctx Hardware context pointer
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
ocs_hal_read_lmt(ocs_hal_t *hal)
{
	uint8_t *buf = NULL;
	int32_t rc = -1;

	buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_NOWAIT);
	if (!buf) {
		ocs_log_err(hal->os, "no buffer for command\n");
		return rc;
	}

	/* Issue read_config to update link module type */
	sli_cmd_read_config(&hal->sli, buf, SLI4_BMBX_SIZE);
	rc = ocs_hal_command(hal, buf, OCS_CMD_NOWAIT, __ocs_hal_read_lmt_cb, NULL);
	if (rc)
		ocs_free(hal->os, buf, SLI4_BMBX_SIZE);

	return rc;
}

/**
 * @brief Callback function for the SLI link events.
 *
 * @par Description
 * This function allocates memory which must be freed in its callback.
 *
 * @param ctx Hardware context pointer (that is, ocs_hal_t *).
 * @param e Event structure pointer (that is, sli4_link_event_t *).
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static int32_t
ocs_hal_cb_link(void *ctx, void *e)
{
	ocs_hal_t	*hal = ctx;
	ocs_t		*ocs = hal->os;
	sli4_link_event_t *event = e;
	ocs_domain_t	*d = NULL;
	uint32_t	i = 0;
	int32_t		rc = OCS_HAL_RTN_ERROR;

	switch (event->status) {
	case SLI_LINK_STATUS_UP:

		ocs_hal_link_event_init(hal);
		hal->link = *event;

		if (SLI_LINK_TOPO_NPORT == event->topology) {
			uint8_t	*buf = NULL;

			ocs_log_info(hal->os, "Link Up, NPORT, speed is %d Mbps\n", event->speed);
			ocs_hal_read_fcf(hal, SLI4_FCOE_FCF_TABLE_FIRST);

			buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_NOWAIT);
			if (!buf) {
				ocs_log_err(hal->os, "no buffer for command\n");
				break;
			}

			/* Issue read_topology to log FEC settings and state */
			if (sli_cmd_read_topology(&hal->sli, buf, SLI4_BMBX_SIZE, NULL))
				rc = ocs_hal_command(hal, buf, OCS_CMD_NOWAIT, __ocs_read_fec_cb, NULL);

			if (rc) {
				ocs_log_test(hal->os, "READ_TOPOLOGY failed\n");
				ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
			}
		} else if (SLI_LINK_TOPO_LOOP == event->topology) {
			uint8_t	*buf = NULL;

			ocs_log_info(hal->os, "Link Up, LOOP, speed is %d Mbps\n", event->speed);
			if ((0 == hal->loop_map.size) &&
					ocs_dma_alloc(hal->os, &hal->loop_map, SLI4_MIN_LOOP_MAP_BYTES, 4))
				ocs_log_err(hal->os, "ocs_dma_alloc() failed\n");

			buf = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
			if (!buf) {
				ocs_log_err(hal->os, "no buffer for command\n");
				break;
			}

			if (sli_cmd_read_topology(&hal->sli, buf, SLI4_BMBX_SIZE, &hal->loop_map)) {
				rc = ocs_hal_command(hal, buf, OCS_CMD_NOWAIT, __ocs_read_topology_cb, NULL);
			}

			if (rc) {
				ocs_log_test(hal->os, "READ_TOPOLOGY failed\n");
				ocs_free(hal->os, buf, SLI4_BMBX_SIZE);
			}
		} else if (SLI_LINK_TOPO_LOOPBACK_INTERNAL == event->topology) {
			ocs_log_info(hal->os, "Link Up, link in internal loopback mode, speed is %d Mbps\n", event->speed);
			ocs_hal_read_fcf(hal, SLI4_FCOE_FCF_TABLE_FIRST);
		} else {
			ocs_log_info(hal->os, "Link Up, unsupported topology (%#x), speed is %d Mbps\n",
					event->topology, event->speed);
		}
		break;
	case SLI_LINK_STATUS_DOWN:
		ocs_log_info(hal->os, "Link down\n");

		ocs_hal_link_event_init(hal);
		hal->link.status = event->status;

		for (i = 0; i < SLI4_MAX_FCFI; i++) {
			d = hal->domains[i];
			if (d != NULL && hal->callback.domain != NULL) {
				hal->callback.domain(hal->args.domain, OCS_HAL_DOMAIN_LOST, d);
			}
		}

		ocs_notify_link_state_change(ocs, 0);
		ocs->xport->forced_link_down = 0;
		break;
	case SLI_LINK_STATUS_CHANGED:
		ocs_hal_read_lmt(hal);
		break;
	case SLI_LOGICAL_LINK_SPEED_CHANGED:
		hal->link.logical_link_speed = event->logical_link_speed;
		hal->link.aggregate_link_speed = event->aggregate_link_speed;
		break;
	default:
		ocs_log_test(hal->os, "unhandled link status %#x\n", event->status);
		break;
	}

	return 0;
}

static int32_t
ocs_hal_cb_fip(void *ctx, void *e)
{
	ocs_hal_t	*hal = ctx;
	ocs_domain_t	*domain = NULL;
	sli4_fip_event_t *event = e;

	// Find the associated domain object
	if (event->type == SLI4_FCOE_FIP_FCF_CLEAR_VLINK) {
		ocs_domain_t *d = NULL;
		uint32_t	i = 0;

		// Clear VLINK is different from the other FIP events as it passes back
		// a VPI instead of a FCF index. Check all attached SLI ports for a
		// matching VPI
		for (i = 0; i < SLI4_MAX_FCFI; i++) {
			d = hal->domains[i];
			if (d != NULL) {
				ocs_sport_t	*sport = NULL;

				ocs_list_foreach(&d->sport_list, sport) {
					if (sport->indicator == event->index) {
						domain = d;
						break;
					}
				}

				// Did we find a matching domain?
				if (domain != NULL) {
					break;
				}
			}
		}
	} else {
		domain = ocs_hal_domain_get_indexed(hal, event->index);
	}

	switch (event->type) {
	case SLI4_FCOE_FIP_FCF_DISCOVERED:
		ocs_hal_read_fcf(hal, event->index);
		break;
	case SLI4_FCOE_FIP_FCF_DEAD:
		if (domain != NULL &&
		    hal->callback.domain != NULL) {
			hal->callback.domain(hal->args.domain, OCS_HAL_DOMAIN_LOST, domain);
		}
		break;
	case SLI4_FCOE_FIP_FCF_CLEAR_VLINK:
		if (domain != NULL &&
		    hal->callback.domain != NULL) {
			/*
			 * We will want to issue rediscover FCF when this domain is free'd  in order
			 * to invalidate the FCF table
			 */
			domain->req_rediscover_fcf = TRUE;
			hal->callback.domain(hal->args.domain, OCS_HAL_DOMAIN_LOST, domain);
		}
		break;
	case SLI4_FCOE_FIP_FCF_MODIFIED:
		if (domain != NULL &&
		    hal->callback.domain != NULL) {
			hal->callback.domain(hal->args.domain, OCS_HAL_DOMAIN_LOST, domain);
		}

		ocs_hal_read_fcf(hal, event->index);
		break;
	default:
		ocs_log_test(hal->os, "unsupported event %#x\n", event->type);
	}

	return 0;
}

static int32_t
ocs_hal_cb_node_attach(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	ocs_remote_node_t *rnode = arg;
	sli4_mbox_command_header_t	*hdr = (sli4_mbox_command_header_t *)mqe;
	ocs_hal_remote_node_event_e	evt = 0;
	ocs_node_t *node = rnode->node;

	if (status || hdr->status) {
		ocs_log_debug(hal->os, "bad status cqe=%#x mqe=%#x\n", status, hdr->status);
		ocs_log_debug(hal->os, "Dumping MBOX command buf\n");
		ocs_dump32(OCS_DEBUG_ALWAYS, hal->os, "MBOX command buff", mqe, SLI4_BMBX_SIZE);
		node_printf(node, "Dumping common service parameters\n");
		ocs_dump32(OCS_DEBUG_ALWAYS, hal->os, "service params",
			   node->service_params, sizeof(node->service_params));

		rnode->attached = FALSE;
		evt = OCS_HAL_NODE_ATTACH_FAIL;
		node->ls_rjt_reason_code = FC_REASON_UNABLE_TO_PERFORM;
		switch (hdr->status) {
		case SLI4_MBOX_STATUS_UNSUPPORTED_FEATURE:
		case SLI4_MBOX_STATUS_ILLEGAL_SIZE:
			node->ls_rjt_reason_expl_code = FC_EXPL_SPARAM_OPTIONS;
			break;
		case SLI4_MBOX_STATUS_INVALID_RPI:
		case SLI4_MBOX_STATUS_INVALID_VPI:
		case SLI4_MBOX_STATUS_RPI_CONFLICT:
			node->ls_rjt_reason_expl_code = FC_EXPL_INSUFFICIENT_RESOURCES;
			break;
		default:
			node->ls_rjt_reason_expl_code = FC_EXPL_NO_ADDITIONAL;
			break;
		}
	} else {
		rnode->attached = TRUE;
		ocs_atomic_set(&hal->rpi_ref[rnode->index].rpi_attached, 1);
		evt = OCS_HAL_NODE_ATTACH_OK;
	}

	if (hal->callback.rnode)
		hal->callback.rnode(hal->args.rnode, evt, rnode);

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

static int32_t
ocs_hal_cb_node_free(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	ocs_remote_node_t *rnode = arg;
	sli4_mbox_command_header_t	*hdr = (sli4_mbox_command_header_t *)mqe;
	ocs_hal_remote_node_event_e	evt = OCS_HAL_NODE_FREE_FAIL;
	int32_t		rc = 0;

	if (status || hdr->status) {
		/*
		 * In certain cases, a non-zero MQE status is OK (all must be true):
		 *   - node is attached
		 *   - if High Login Mode is enabled, node is part of a node group
		 *   - status is 0x1400
		 */
		if (!rnode->attached || ((sli_get_hlm(&hal->sli) == TRUE) && !rnode->node_group) ||
				(hdr->status != SLI4_MBOX_STATUS_RPI_NOT_REG)) {
			rc = -1;
		}
	}

	if (rc == 0) {
		rnode->node_group = FALSE;
		rnode->attached = FALSE;
		evt = OCS_HAL_NODE_FREE_OK;
	}

	/*
	 * In HLM, free the RPI after the last remote node gets unregistered.
	 * rnode->indicator is valid for the last UNREG_RPI of remote node.
	 */
	if (rnode->indicator != UINT32_MAX) {
		if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_RPI, rnode->indicator))
			ocs_log_err(hal->os, "FCOE_RPI (%d) free failed, addr=%#x\n", rnode->indicator, rnode->fc_id);

		rnode->indicator = UINT32_MAX;
	}

	rnode->free_group = FALSE;
	if (hal->callback.rnode)
		hal->callback.rnode(hal->args.rnode, evt, rnode);

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return rc;
}

static int32_t
ocs_hal_cb_unreg_rpi_all(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_mbox_command_header_t *hdr = (sli4_mbox_command_header_t *)mqe;
	ocs_hal_remote_node_event_e evt = OCS_HAL_NODE_FREE_FAIL;
	ocs_sport_t *sport = arg;
	ocs_remote_node_t *rnode;
	ocs_node_t *node;
	ocs_node_t *node_next;
	int32_t rc = 0;

	if ((hal->state == OCS_HAL_STATE_ACTIVE) && 
	     (status || hdr->status)) {
		rc = -1;
	} else {
		evt = OCS_HAL_NODE_FREE_OK;
	}

	ocs_list_foreach_safe(&sport->node_list, node, node_next) {
		rnode = &node->rnode;
		if (!rnode->attached)
			continue;

		rnode->node_group = FALSE;
		rnode->attached = FALSE;
		if (ocs_atomic_read(&hal->rpi_ref[rnode->index].rpi_count) == 0)
			ocs_atomic_set(&hal->rpi_ref[rnode->index].rpi_attached, 0);

		if (hal->callback.rnode)
			hal->callback.rnode(hal->args.rnode, evt, rnode);
	}

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return rc;
}

/**
 * @brief Initialize the pool of HAL IO objects.
 *
 * @param hal Hardware context.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static ocs_hal_rtn_e
ocs_hal_setup_io(ocs_hal_t *hal)
{
	uint32_t	i = 0;
	ocs_hal_io_t	*io = NULL;
	uint32_t	index;
	uint8_t		new_alloc = TRUE;

	if (!hal->io) {
		hal->io = ocs_malloc(hal->os, hal->config.n_io * sizeof(ocs_hal_io_t *), OCS_M_ZERO);
		if (!hal->io) {
			ocs_log_err(hal->os, "IO pointer memory allocation failed, %d Ios at size %zu\n",
					hal->config.n_io, sizeof(ocs_hal_io_t *));
			goto setup_io_error;
		}

		/* Create WQE buffs for IO */
		hal->wqe_buffs = ocs_malloc(hal->os, hal->config.n_io * hal->sli.config.wqe_size, OCS_M_ZERO);
		if (!hal->wqe_buffs) {
			ocs_log_err(hal->os, "IO WQE buff allocation failed, %d Ios at size %zu\n",
					hal->config.n_io, hal->sli.config.wqe_size);
			goto setup_io_error;
		}

		hal->xfer_rdy = ocs_malloc(hal->os, hal->config.n_io * sizeof(ocs_dma_t), OCS_M_ZERO);
		if (!hal->xfer_rdy) {
			ocs_log_err(hal->os, "hal->xfer_rdy memory allocation failed, %d Ios at size %zu\n",
					hal->config.n_io, sizeof(ocs_dma_t));
			goto setup_io_error;
		}
	} else {
		/* re-use existing IOs, including SGLs */
		new_alloc = FALSE;
	}

	for (i = 0; i < hal->config.n_io; i++) {
		hal_wq_callback_t *wqcb;

		if (new_alloc) {
			hal->io[i] = ocs_malloc(hal->os, sizeof(ocs_hal_io_t), OCS_M_ZERO);
			if (!hal->io[i]) {
				ocs_log_err(hal->os, "IO(%d) memory allocation failed\n", i);
				goto setup_io_error;
			}
		}
		io = hal->io[i];

		/* Initialize IO fields */
		io->hal = hal;

		/* Assign a WQE buff */
		io->wqe.wqebuf = &hal->wqe_buffs[i * hal->sli.config.wqe_size];

		/* Allocate the request tag for this IO */
		wqcb = ocs_hal_reqtag_alloc(hal, ocs_hal_wq_process_io, io);
		if (!wqcb) {
			ocs_log_err(hal->os, "can't allocate request tag\n");
			goto setup_io_error;
		}
		io->reqtag = wqcb->instance_index;

		/* Initialize the HAL IO fields */
		ocs_hal_init_free_io(io);

		/* The XB flag isn't cleared on IO free, so initialize it to zero here */
		io->xbusy = 0;

		if (sli_resource_alloc(&hal->sli, SLI_RSRC_FCOE_XRI, &io->indicator, &index)) {
			ocs_log_err(hal->os, "sli_resource_alloc failed @ %d\n", i);
			goto setup_io_error;
		}

		if (new_alloc) {
			if (ocs_dma_alloc(hal->os, &io->def_sgl, hal->config.n_sgl * sizeof(sli4_sge_t), 64)) {
				ocs_log_err(hal->os, "ocs_dma_alloc failed @ %d\n", i);
				ocs_memset(&io->def_sgl, 0, sizeof(ocs_dma_t));
				goto setup_io_error;
			}

			if (ocs_dma_alloc(hal->os, &hal->xfer_rdy[i], sizeof(fcp_xfer_rdy_iu_t), 4)) {
				ocs_log_err(hal->os, "XFER_RDY buffer allocation failed @ %d\n", i);
				ocs_memset(&hal->xfer_rdy[i], 0, sizeof(ocs_dma_t));
				goto setup_io_error;
			}

			ocs_lock_init(hal->os, &io->lock, "hio_lock[%d]", io->indicator);
		}

		io->def_sgl_count = hal->config.n_sgl;
		io->sgl = &io->def_sgl;
		io->sgl_count = io->def_sgl_count;

		if (hal->xfer_rdy[i].size) {
			io->xfer_rdy.virt = (void *)hal->xfer_rdy[i].virt;
			io->xfer_rdy.phys = hal->xfer_rdy[i].phys;
			io->xfer_rdy.size = sizeof(fcp_xfer_rdy_iu_t);
		}
	}

	return OCS_HAL_RTN_SUCCESS;

setup_io_error:
	if (hal->io) {
		for (i = 0; i < hal->config.n_io && hal->io[i]; i++) {
			if (hal->io[i]->sgl && hal->io[i]->sgl->virt)
				ocs_dma_free(hal->os, hal->io[i]->sgl);

			if (hal->xfer_rdy && hal->xfer_rdy[i].virt)
				ocs_dma_free(hal->os, &hal->xfer_rdy[i]);

			ocs_lock_free(&hal->io[i]->lock);
			ocs_free(hal->os, hal->io[i], sizeof(ocs_hal_io_t));
			hal->io[i] = NULL;
		}

		ocs_free(hal->os, hal->io, hal->config.n_io * sizeof(ocs_hal_io_t *));
		hal->io = NULL;
	}

	if (hal->xfer_rdy) {
		ocs_free(hal->os, hal->xfer_rdy, hal->config.n_io * sizeof(ocs_dma_t));
		hal->xfer_rdy = NULL;
	}

	if (hal->wqe_buffs) {
		ocs_free(hal->os, hal->wqe_buffs, hal->config.n_io * hal->sli.config.wqe_size);
		hal->wqe_buffs = NULL;
	}

	return OCS_HAL_RTN_NO_MEMORY;
}

static ocs_hal_rtn_e
ocs_hal_init_io(ocs_hal_t *hal)
{
	uint32_t	i = 0, io_index = 0;
	uint32_t	prereg = 0, sge_index = 0;
	ocs_hal_io_t	*io = NULL;
	uint8_t		cmd[SLI4_BMBX_SIZE];
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint32_t	nremaining;
	uint32_t	n = 0;
	uint32_t	sgls_per_request = 256;
	ocs_dma_t	**sgls = NULL;
	ocs_dma_t	reqbuf = { 0 };
	uint16_t	reqbuf_length = sizeof(sli4_req_fcoe_post_sgl_pages_t) +
					sizeof(sli4_fcoe_post_sgl_page_desc_t)*sgls_per_request;

	prereg = sli_get_sgl_preregister(&hal->sli);
	if (prereg) {
		sgls = ocs_malloc(hal->os, sizeof(*sgls) * sgls_per_request, OCS_M_ZERO);
		if (sgls == NULL) {
			ocs_log_err(hal->os, "ocs_malloc sgls failed\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}

		/* Create a buffer that can hold sgls for 256 SGLs at a time */
		rc = ocs_dma_alloc(hal->os, &reqbuf, reqbuf_length, OCS_MIN_DMA_ALIGNMENT);
		if (rc) {
			ocs_log_err(hal->os, "ocs_dma_alloc reqbuf failed\n");
			ocs_free(hal->os, sgls, sizeof(*sgls) * sgls_per_request);
			return OCS_HAL_RTN_NO_MEMORY;
		}
	}

	for (nremaining = hal->config.n_io; nremaining; nremaining -= n) {
		if (prereg) {
			/* Copy address of SGL's into local sgls[] array, break out if the xri
			 * is not contiguous.
			 */
			for (n = 0; n < MIN(sgls_per_request, nremaining); n++) {
				sge_index = io_index + n;

				/* Check that we have contiguous xri values */
				if (n > 0) {
					if (hal->io[sge_index]->indicator != (hal->io[sge_index-1]->indicator+1)) {
						break;
					}
				}
				sgls[n] = hal->io[sge_index]->sgl;
			}

			sli_cmd_fcoe_post_sgl_pages(&hal->sli, cmd, sizeof(cmd),
				hal->io[io_index]->indicator, n, sgls, NULL, &reqbuf);
			rc = ocs_hal_command(hal, cmd, OCS_CMD_POLL, NULL, NULL);
			if (rc) {
				rc = OCS_HAL_RTN_ERROR;
				break;
			}
		} else {
			n = nremaining;
		}

		/* Add to tail if successful */
		for (i = 0; i < n; i++, io_index++) {
			io = hal->io[io_index];
			io->is_port_owned = 0;
			io->state = OCS_HAL_IO_STATE_FREE;
			ocs_list_add_tail(&hal->io_free, io);
		}
	}

	if (prereg) {
		ocs_dma_free(hal->os, &reqbuf);
		ocs_free(hal->os, sgls, sizeof(*sgls) * sgls_per_request);
	}

	return rc;
}

int32_t
ocs_hal_flush(ocs_hal_t *hal)
{
	uint32_t	i = 0;

	/* Process any remaining completions */
	for (i = 0; i < hal->eq_count; i++) {
		/* Skip NVMe queues as they are not handled by HAL code */
		if (hal->hal_eq[i] && !hal->hal_eq[i]->nvmeq)
			ocs_hal_process(hal, i, ~0);
	}

	return 0;
}

static int32_t
ocs_hal_command_cancel(ocs_hal_t *hal)
{
	ocs_lock(&hal->cmd_lock);

	/*
	 * Manually clean up remaining commands. Note: since this calls
	 * ocs_hal_command_process(), we'll also process the cmd_pending
	 * list, so no need to manually clean that out.
	 */
	while (!ocs_list_empty(&hal->cmd_head)) {
		uint8_t		mqe[SLI4_BMBX_SIZE] = { 0 };
		ocs_command_ctx_t *ctx = ocs_list_get_head(&hal->cmd_head);

		ocs_log_test(hal->os, "hung command %08x\n",
			     NULL == ctx ? UINT32_MAX :
			     (NULL == ctx->buf ? UINT32_MAX : *((uint32_t *)ctx->buf)));

		ocs_unlock(&hal->cmd_lock);
		ocs_hal_command_process(hal, -1/*Bad status*/, mqe, SLI4_BMBX_SIZE);
		ocs_lock(&hal->cmd_lock);
	}

	ocs_unlock(&hal->cmd_lock);

	return 0;
}

/**
 * @brief Find IO given indicator (xri).
 *
 * @param hal Hal context.
 * @param indicator Indicator (xri) to look for.
 *
 * @return Returns io if found, NULL otherwise.
 */
ocs_hal_io_t *
ocs_hal_io_lookup(ocs_hal_t *hal, uint32_t xri)
{
	uint32_t ioindex;
	ioindex = xri - hal->sli.config.extent[SLI_RSRC_FCOE_XRI].base[0];
	return hal->io[ioindex];
}

/**
 * @brief Issue any pending callbacks for an IO and remove off the timer and pending lists.
 *
 * @param hal Hal context.
 * @param io Pointer to the IO to cleanup.
 */
static void
ocs_hal_io_cancel_cleanup(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	/* Remove from the io_timed_wqe list */
	ocs_hal_remove_io_timed_wqe(hal, io);

	/* Remove from the WQ pending list */
	if (io->wq && ocs_list_on_list(&io->wqe.link) && !ocs_list_empty(&io->wq->pending_list))
		ocs_list_remove(&io->wq->pending_list, io);

	if (io->done) {
		ocs_hal_done_t	done = io->done;
		void		*arg = io->arg;

		io->done = NULL;
		ocs_unlock(&hal->io_lock);

		done(io, io->rnode, 0, SLI4_FC_WCQE_STATUS_SHUTDOWN, 0, arg);
		ocs_lock(&hal->io_lock);
	}

	if (io->abort_done) {
		ocs_hal_done_t	abort_done = io->abort_done;
		void		*abort_arg = io->abort_arg;

		io->abort_done = NULL;
		ocs_unlock(&hal->io_lock);

		abort_done(io, io->rnode, 0, SLI4_FC_WCQE_STATUS_SHUTDOWN, 0, abort_arg);
		ocs_lock(&hal->io_lock);
	}
}

int32_t
ocs_hal_io_cancel(ocs_hal_t *hal)
{
	ocs_hal_io_t *io = NULL;
	ocs_hal_io_t *tmp_io = NULL;
	uint32_t iters = 100; /* One second limit */

	/*
	 * Manually clean up the outstanding IO. Walk through the list only
	 * once, the backend will cleanup any IOs when io->done() is called.
	 */
	ocs_lock(&hal->io_lock);
		ocs_list_foreach_safe(&hal->io_inuse, io, tmp_io) {
			ocs_hal_done_t done = io->done;
			ocs_hal_done_t abort_done = io->abort_done;

			ocs_hal_io_cancel_cleanup(hal, io);

			/*
			 * This is called only in the reset/shutdown path.
			 * So, if there is no callback, just free the IO.
			 */
			if (!done && !abort_done) {
				ocs_hal_io_free_common(hal, io);
				ocs_list_remove(&hal->io_inuse, io);
				ocs_hal_io_free_move_correct_list(hal, io);
			}
		}

		/* For port owned XRIs, walk through the list and do the cleanup */
		ocs_list_foreach_safe(&hal->io_port_owned, io, tmp_io) {
			if (ocs_list_on_list(&io->dnrx_link)) {
				ocs_list_remove(&hal->io_port_dnrx, io);
				ocs_ref_put(&io->ref);
			}

			/*
			 * Note: A port owned XRI cannot be on the io_inuse list. We cannot
			 *	 call ocs_hal_io_free() because we already hold the io_lock.
			 */
			ocs_hal_io_cancel_cleanup(hal, io);
			ocs_list_remove(&hal->io_port_owned, io);
			ocs_hal_io_free_common(hal, io);
		}
	ocs_unlock(&hal->io_lock);

	/* Give time for the callbacks to complete */
	do {
		ocs_delay_usec(10000);
		iters--;
	} while (!ocs_list_empty(&hal->io_inuse) && iters);

	/* Leave a breadcrumb that the cleanup is not yet complete */
	if (!ocs_list_empty(&hal->io_inuse))
		ocs_log_info(hal->os, "io_inuse list is not empty\n");

	return 0;
}

static int32_t
ocs_hal_io_ini_sge(ocs_hal_t *hal, ocs_hal_io_t *io, ocs_dma_t *cmnd, uint32_t cmnd_size,
		ocs_dma_t *rsp)
{
	sli4_sge_t	*data = NULL;

	if (!hal || !io) {
		ocs_log_err(NULL, "bad parm hal=%p io=%p\n", hal, io);
		return OCS_HAL_RTN_ERROR;
	}

	data = io->def_sgl.virt;

	// setup command pointer
	data->buffer_address_high = ocs_addr32_hi(cmnd->phys);
	data->buffer_address_low  = ocs_addr32_lo(cmnd->phys);
	data->buffer_length = cmnd_size;
	data++;

	// setup response pointer
	data->buffer_address_high = ocs_addr32_hi(rsp->phys);
	data->buffer_address_low  = ocs_addr32_lo(rsp->phys);
	data->buffer_length = rsp->size;

	return 0;
}

static int32_t
__ocs_read_fec_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_read_topology_t *read_topo = (sli4_cmd_read_topology_t *)mqe;

	if ((0 == status) && (0 == read_topo->hdr.status)) {
		uint32_t link_speed = 0;

		sli4_decode_link_speed(read_topo->link_current.link_speed, &link_speed);
		ocs_log_info(hal->os, "fec_enable = %d, fec_state = %d, current_link_speed = %d Mbps\n",
				read_topo->fecen, read_topo->fec, link_speed);
	}

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

static int32_t
__ocs_read_topology_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_read_topology_t *read_topo = (sli4_cmd_read_topology_t *)mqe;

	if (status || read_topo->hdr.status) {
		ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
		return -1;
	}

	switch (read_topo->attention_type) {
	case SLI4_READ_TOPOLOGY_LINK_UP:
		hal->link.status = SLI_LINK_STATUS_UP;
		break;
	case SLI4_READ_TOPOLOGY_LINK_DOWN:
		hal->link.status = SLI_LINK_STATUS_DOWN;
		break;
	case SLI4_READ_TOPOLOGY_LINK_NO_ALPA:
		hal->link.status = SLI_LINK_STATUS_NO_ALPA;
		break;
	default:
		hal->link.status = SLI_LINK_STATUS_MAX;
		break;
	}

	switch (read_topo->topology) {
	case SLI4_READ_TOPOLOGY_NPORT:
		hal->link.topology = SLI_LINK_TOPO_NPORT;
		break;
	case SLI4_READ_TOPOLOGY_FC_AL:
		hal->link.topology = SLI_LINK_TOPO_LOOP;
		if (SLI_LINK_STATUS_UP == hal->link.status) {
			hal->link.loop_map = hal->loop_map.virt;
		}
		hal->link.fc_id = read_topo->acquired_al_pa;
		break;
	default:
		hal->link.topology = SLI_LINK_TOPO_MAX;
		break;
	}

	hal->link.medium = SLI_LINK_MEDIUM_FC;
	sli4_decode_link_speed(read_topo->link_current.link_speed, &hal->link.speed);

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	ocs_hal_read_fcf(hal, SLI4_FCOE_FCF_TABLE_FIRST);

	return 0;
}

static int32_t
__ocs_hal_port_common(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_EXIT:
		/* ignore */
		break;

	case OCS_EVT_HAL_PORT_REQ_FREE:
	case OCS_EVT_HAL_PORT_REQ_ATTACH:
		if (data != NULL) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		}
		/* fall through */
	default:
		ocs_log_test(hal->os, "%s %-20s not handled\n", funcname, ocs_sm_event_name(evt));
		break;
	}

	return 0;
}

static void *
__ocs_hal_port_free_report_fail(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		if (data != NULL) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		}
		if (hal->callback.port != NULL) {
			hal->callback.port(hal->args.port,
					OCS_HAL_PORT_FREE_FAIL, sport);
		}
		break;
	default:
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_freed(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* free SLI resource */
		if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_VPI, sport->indicator))
			ocs_log_err(hal->os, "FCOE_VPI (%d) free failed, addr=%#x\n", sport->indicator, sport->fc_id);

		/* free mailbox buffer */
		if (data != NULL) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		}

		if (hal->callback.port != NULL) {
			hal->callback.port(hal->args.port,
					OCS_HAL_PORT_FREE_OK, sport);
		}
		break;
	default:
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_attach_report_fail(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* free SLI resource */
		if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_VPI, sport->indicator))
			ocs_log_err(hal->os, "FCOE_VPI (%d) free failed, addr=%#x\n", sport->indicator, sport->fc_id);

		ocs_dump32(OCS_DEBUG_ALWAYS, hal->os, "sport service params",
			   sport->service_params, sizeof(sport->service_params));
		/* free mailbox buffer */
		if (data != NULL) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		}

		if (hal->callback.port != NULL) {
			hal->callback.port(hal->args.port,
					OCS_HAL_PORT_ATTACH_FAIL, sport);
		}

		if (sport->sm_free_req_pending) {
			ocs_sm_transition(ctx, __ocs_hal_port_free_unreg_vpi, NULL);
		}
		break;
	default:
		__ocs_hal_port_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_free_unreg_vpi(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;
	uint8_t		*cmd = NULL;

	smtrace("port");

	if ((hal->state != OCS_HAL_STATE_ACTIVE) && (evt == OCS_EVT_ERROR)) {
		/* If hal is not active, ignore error and free port */
		evt = OCS_EVT_RESPONSE;
	}

	switch (evt) {
	case OCS_EVT_ENTER:
		/* allocate memory and send unreg_vpi */
		cmd = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
		if (!cmd) {
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		if (0 == sli_cmd_unreg_vpi(&hal->sli, cmd, SLI4_BMBX_SIZE, sport->indicator,
					   SLI4_UNREG_TYPE_PORT)) {
			ocs_log_err(hal->os, "UNREG_VPI format failure\n");
			ocs_free(hal->os, cmd, SLI4_BMBX_SIZE);
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		if (hal->state != OCS_HAL_STATE_ACTIVE) {
			ocs_free(hal->os, cmd, SLI4_BMBX_SIZE);
			ocs_sm_post_event(ctx, OCS_EVT_RESPONSE, NULL);
			break;
		}

		if (ocs_hal_command(hal, cmd, OCS_CMD_NOWAIT, __ocs_hal_port_cb, sport)) {
			ocs_free(hal->os, cmd, SLI4_BMBX_SIZE);
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
		}

		break;
	case OCS_EVT_RESPONSE:
		ocs_sm_transition(ctx, __ocs_hal_port_freed, data);
		break;
	case OCS_EVT_ERROR:
		ocs_sm_transition(ctx, __ocs_hal_port_free_report_fail, data);
		break;
	default:
		__ocs_hal_port_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_free_nop(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* Forward to execute in mailbox completion processing context */
		if (ocs_hal_async_call(hal, __ocs_hal_port_realloc_cb, sport)) {
			ocs_log_err(hal->os, "ocs_hal_async_call failed\n");
		}
		break;
	case OCS_EVT_RESPONSE:
		ocs_sm_transition(ctx, __ocs_hal_port_freed, data);
		break;
	case OCS_EVT_ERROR:
		ocs_sm_transition(ctx, __ocs_hal_port_free_report_fail, data);
		break;
	default:
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_attached(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		if (data != NULL) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		}
		if (hal->callback.port != NULL) {
			hal->callback.port(hal->args.port,
					OCS_HAL_PORT_ATTACH_OK, sport);
		}
		if (sport->sm_free_req_pending) {
			ocs_sm_transition(ctx, __ocs_hal_port_free_unreg_vpi, NULL);
		}
		break;
	case OCS_EVT_HAL_PORT_REQ_FREE:
		/* virtual/physical port request free */
		ocs_sm_transition(ctx, __ocs_hal_port_free_unreg_vpi, NULL);
		break;
	default:
		__ocs_hal_port_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_attach_reg_vpi(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		if (0 == sli_cmd_reg_vpi(&hal->sli, data, SLI4_BMBX_SIZE, sport, FALSE)) {
			ocs_log_err(hal->os, "REG_VPI format failure\n");
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		if (ocs_hal_command(hal, data, OCS_CMD_NOWAIT, __ocs_hal_port_cb, sport))
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);

		break;
	case OCS_EVT_RESPONSE:
		ocs_sm_transition(ctx, __ocs_hal_port_attached, data);
		break;
	case OCS_EVT_ERROR:
		ocs_sm_transition(ctx, __ocs_hal_port_attach_report_fail, data);
		break;
	case OCS_EVT_HAL_PORT_REQ_FREE:
		/* Wait for attach response and then free */
		sport->sm_free_req_pending = 1;
		break;
	default:
		__ocs_hal_port_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_done(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* free SLI resource */
		if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_VPI, sport->indicator))
			ocs_log_err(hal->os, "FCOE_VPI (%d) free failed, addr=%#x\n", sport->indicator, sport->fc_id);

		/* free mailbox buffer */
		if (data != NULL) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		}
		break;
	default:
		__ocs_hal_port_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_allocated(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		if (data != NULL) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		}
		if (hal->callback.port != NULL) {
			hal->callback.port(hal->args.port,
					OCS_HAL_PORT_ALLOC_OK, sport);
		}
		/* If there is a pending free request, then handle it now */
		if (sport->sm_free_req_pending) {
			ocs_sm_transition(ctx, __ocs_hal_port_free_unreg_vpi, NULL);
		}
		break;
	case OCS_EVT_HAL_PORT_REQ_ATTACH:
		/* virtual port requests attach */
		ocs_sm_transition(ctx, __ocs_hal_port_attach_reg_vpi, data);
		break;
	case OCS_EVT_HAL_PORT_ATTACH_OK:
		/* physical port attached (as part of attaching domain) */
		ocs_sm_transition(ctx, __ocs_hal_port_attached, data);
		break;
	case OCS_EVT_HAL_PORT_REQ_FREE:
		/* virtual port request free */
		if (SLI4_IF_TYPE_LANCER_FC_ETH == sli_get_if_type(&hal->sli)) {
			ocs_sm_transition(ctx, __ocs_hal_port_free_unreg_vpi, NULL);
		} else {
			/*
			 * Note: BE3/Skyhawk will respond with a status of 0x20
			 *       unless the reg_vpi has been issued, so we can
			 *       skip the unreg_vpi for these adapters.
			 *
			 * Send a nop to make sure that free doesn't occur in
			 * same context
			 */
			ocs_sm_transition(ctx, __ocs_hal_port_free_nop, NULL);
		}
		break;
	default:
		__ocs_hal_port_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_alloc_report_fail(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* free SLI resource */
		if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_VPI, sport->indicator))
			ocs_log_err(hal->os, "FCOE_VPI (%d) free failed, addr=%#x\n", sport->indicator, sport->fc_id);

		/* free mailbox buffer */
		if (data != NULL) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		}

		if (hal->callback.port != NULL) {
			hal->callback.port(hal->args.port,
					OCS_HAL_PORT_ALLOC_FAIL, sport);
		}

		/* If there is a pending free request, then handle it now */
		if (sport->sm_free_req_pending) {
			ocs_sm_transition(ctx, __ocs_hal_port_free_unreg_vpi, NULL);
		}
		break;
	default:
		__ocs_hal_port_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_alloc_read_sparm64(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;
	uint8_t		*payload = NULL;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		// allocate memory for the service parameters
		if (ocs_dma_alloc(hal->os, &sport->dma, 112, 4)) {
			ocs_log_err(hal->os, "Failed to allocate DMA memory\n");
			ocs_sm_transition(ctx, __ocs_hal_port_done, data);
			break;
		}

		if (0 == sli_cmd_read_sparm64(&hal->sli, data, SLI4_BMBX_SIZE,
					&sport->dma, sport->indicator)) {
			ocs_log_err(hal->os, "READ_SPARM64 allocation failure\n");
			ocs_dma_free(hal->os, &sport->dma);
			ocs_sm_transition(ctx, __ocs_hal_port_done, data);
			break;
		}

		if (ocs_hal_command(hal, data, OCS_CMD_NOWAIT, __ocs_hal_port_cb, sport)) {
			ocs_dma_free(hal->os, &sport->dma);
			ocs_sm_transition(ctx, __ocs_hal_port_done, data);
		}

		break;
	case OCS_EVT_RESPONSE:
		payload = sport->dma.virt;

		ocs_display_sparams(sport->display_name, "sport sparm64", 0, NULL, payload);

		ocs_memcpy(&sport->sli_wwpn, payload + SLI4_READ_SPARM64_WWPN_OFFSET,
				sizeof(sport->sli_wwpn));
		ocs_memcpy(&sport->sli_wwnn, payload + SLI4_READ_SPARM64_WWNN_OFFSET,
				sizeof(sport->sli_wwnn));

		ocs_dma_free(hal->os, &sport->dma);
		ocs_sm_transition(ctx, __ocs_hal_port_alloc_init_vpi, data);
		break;
	case OCS_EVT_ERROR:
		ocs_dma_free(hal->os, &sport->dma);
		ocs_sm_transition(ctx, __ocs_hal_port_alloc_report_fail, data);
		break;
	case OCS_EVT_HAL_PORT_REQ_FREE:
		/* Wait for attach response and then free */
		sport->sm_free_req_pending = 1;
		break;
	case OCS_EVT_EXIT:
		break;
	default:
		__ocs_hal_port_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_alloc_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* no-op */
		break;
	case OCS_EVT_HAL_PORT_ALLOC_OK:
		ocs_sm_transition(ctx, __ocs_hal_port_allocated, NULL);
		break;
	case OCS_EVT_HAL_PORT_ALLOC_FAIL:
		ocs_sm_transition(ctx, __ocs_hal_port_alloc_report_fail, NULL);
		break;
	case OCS_EVT_HAL_PORT_REQ_FREE:
		/* Wait for attach response and then free */
		sport->sm_free_req_pending = 1;
		break;
	default:
		__ocs_hal_port_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_port_alloc_init_vpi(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_sli_port_t	*sport = ctx->app;
	ocs_hal_t	*hal = sport->hal;

	smtrace("port");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* If there is a pending free request, then handle it now */
		if (sport->sm_free_req_pending) {
			ocs_sm_transition(ctx, __ocs_hal_port_freed, NULL);
			return NULL;
		}

		//TODO XXX transitioning to done only works if this is called
		// directly from ocs_hal_port_alloc BUT not if called from
		// read_sparm64. In the later case, we actually want to go
		// through report_ok/fail
		if (0 == sli_cmd_init_vpi(&hal->sli, data, SLI4_BMBX_SIZE,
					sport->indicator, sport->domain->indicator)) {
			ocs_log_err(hal->os, "INIT_VPI allocation failure\n");
			ocs_sm_transition(ctx, __ocs_hal_port_done, data);
			break;
		}

		if (ocs_hal_command(hal, data, OCS_CMD_NOWAIT, __ocs_hal_port_cb, sport))
			ocs_sm_transition(ctx, __ocs_hal_port_done, data);

		break;
	case OCS_EVT_RESPONSE:
		ocs_sm_transition(ctx, __ocs_hal_port_allocated, data);
		break;
	case OCS_EVT_ERROR:
		ocs_sm_transition(ctx, __ocs_hal_port_alloc_report_fail, data);
		break;
	case OCS_EVT_HAL_PORT_REQ_FREE:
		/* Wait for attach response and then free */
		sport->sm_free_req_pending = 1;
		break;
	case OCS_EVT_EXIT:
		break;
	default:
		__ocs_hal_port_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static int32_t
__ocs_hal_port_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	ocs_sli_port_t *sport = arg;
	sli4_mbox_command_header_t *hdr = (sli4_mbox_command_header_t *)mqe;
	ocs_sm_event_t evt;

	if (status || hdr->status) {
		evt = OCS_EVT_ERROR;
	} else {
		evt = OCS_EVT_RESPONSE;
	}

	ocs_sm_post_event(&sport->ctx, evt, mqe);
	return 0;
}

static int32_t
__ocs_hal_port_realloc_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	ocs_sli_port_t *sport = arg;
	sli4_mbox_command_header_t	*hdr = (sli4_mbox_command_header_t *)mqe;
	ocs_sm_event_t	evt;
	uint8_t *mqecpy;

	if (status || hdr->status) {
		evt = OCS_EVT_ERROR;
	} else {
		evt = OCS_EVT_RESPONSE;
	}

	/*
	 * In this case we have to malloc a mailbox command buffer, as it is reused
	 * in the state machine post event call, and eventually freed
	 */
	mqecpy = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (mqecpy == NULL) {
		ocs_log_err(hal->os, "malloc mqecpy failed\n");
		return -1;
	}
	ocs_memcpy(mqecpy, mqe, SLI4_BMBX_SIZE);

	ocs_sm_post_event(&sport->ctx, evt, mqecpy);

	return 0;
}

/***************************************************************************
 * Domain state machine
 */

static int32_t
__ocs_hal_domain_common(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_EXIT:
		/* ignore */
		break;

	default:
		ocs_log_test(hal->os, "%s %-20s not handled\n", funcname, ocs_sm_event_name(evt));
		break;
	}

	return 0;
}

static void *
__ocs_hal_domain_alloc_report_fail(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* free service parameters buffer */
		if (domain->dma.virt) {
			ocs_dma_free(hal->os, &domain->dma);
		}

		/* free command buffer */
		if (data != NULL) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		}

		/* free SLI resources */
		if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_VFI, domain->indicator))
			ocs_log_err(hal->os, "FCOE_VFI (%d) free failed\n", domain->indicator);

		// TODO: how to free FCFI (or do we at all)?

		if (hal->callback.domain != NULL) {
			hal->callback.domain(hal->args.domain, OCS_HAL_DOMAIN_ALLOC_FAIL, domain);
		}
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_attached(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* free mailbox buffer and send alloc ok to physical sport */
		ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		ocs_sm_post_event(&domain->sport->ctx, OCS_EVT_HAL_PORT_ATTACH_OK, NULL);

		/* now inform registered callbacks */
		if (hal->callback.domain != NULL) {
			hal->callback.domain(hal->args.domain,
					OCS_HAL_DOMAIN_ATTACH_OK,
					domain);
		}
		break;
	case OCS_EVT_HAL_DOMAIN_REQ_FREE:
		ocs_sm_transition(ctx, __ocs_hal_domain_free_unreg_vfi, NULL);
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_attach_report_fail(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* free service parameters buffer */
		if (domain->dma.virt) {
			ocs_dma_free(hal->os, &domain->dma);
		}
		ocs_dump32(OCS_DEBUG_ALWAYS, hal->os, "domain service params",
			   domain->service_params, sizeof(domain->service_params));

		/* free command buffer */
		if (data != NULL) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		}

		/* free SLI resources */
		if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_VFI, domain->indicator))
			ocs_log_err(hal->os, "FCOE_VFI (%d) free failed\n", domain->indicator);

		// TODO: how to free FCFI (or do we at all)?

		if (hal->callback.domain != NULL) {
			hal->callback.domain(hal->args.domain, OCS_HAL_DOMAIN_ATTACH_FAIL, domain);
		}
		break;
	case OCS_EVT_EXIT:
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_attach_reg_vfi(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:

		ocs_display_sparams("", "reg vpi", 0, NULL, domain->dma.virt);

		if (0 == sli_cmd_reg_vfi(&hal->sli, data, SLI4_BMBX_SIZE, domain)) {
			ocs_log_err(hal->os, "REG_VFI format failure\n");
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		if (ocs_hal_command(hal, data, OCS_CMD_NOWAIT, __ocs_hal_domain_cb, domain))
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);

		break;
	case OCS_EVT_RESPONSE:
		ocs_sm_transition(ctx, __ocs_hal_domain_attached, data);
		break;
	case OCS_EVT_ERROR:
		ocs_sm_transition(ctx, __ocs_hal_domain_attach_report_fail, data);
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_allocated(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* free mailbox buffer and send alloc ok to physical sport */
		ocs_free(hal->os, data, SLI4_BMBX_SIZE);
		ocs_sm_post_event(&domain->sport->ctx, OCS_EVT_HAL_PORT_ALLOC_OK, NULL);

		ocs_hal_domain_add(hal, domain);

		/* now inform registered callbacks */
		if (hal->callback.domain != NULL) {
			hal->callback.domain(hal->args.domain,
					OCS_HAL_DOMAIN_ALLOC_OK,
					domain);
		}
		break;
	case OCS_EVT_HAL_DOMAIN_REQ_ATTACH:
		ocs_sm_transition(ctx, __ocs_hal_domain_attach_reg_vfi, data);
		break;
	case OCS_EVT_HAL_DOMAIN_REQ_FREE:
		/* unreg_fcfi/vfi */
		if (SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(&hal->sli)) {
			ocs_sm_transition(ctx, __ocs_hal_domain_free_unreg_fcfi, NULL);
		} else {
			ocs_sm_transition(ctx, __ocs_hal_domain_free_unreg_vfi, NULL);
		}
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_alloc_read_sparm64(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		if (0 == sli_cmd_read_sparm64(&hal->sli, data, SLI4_BMBX_SIZE,
					&domain->dma, SLI4_READ_SPARM64_VPI_DEFAULT)) {
			ocs_log_err(hal->os, "READ_SPARM64 format failure\n");
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		if (ocs_hal_command(hal, data, OCS_CMD_NOWAIT, __ocs_hal_domain_cb, domain))
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);

		break;
	case OCS_EVT_EXIT:
		break;
	case OCS_EVT_RESPONSE:
		ocs_display_sparams(domain->display_name, "domain sparm64", 0, NULL, domain->dma.virt);

		ocs_sm_transition(ctx, __ocs_hal_domain_allocated, data);
		break;
	case OCS_EVT_ERROR:
		ocs_sm_transition(ctx, __ocs_hal_domain_alloc_report_fail, data);
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_alloc_init_vfi(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_sli_port_t	*sport = domain->sport;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		if (0 == sli_cmd_init_vfi(&hal->sli, data, SLI4_BMBX_SIZE, domain->indicator,
					domain->fcf_indicator, sport->indicator)) {
			ocs_log_err(hal->os, "INIT_VFI format failure\n");
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		if (ocs_hal_command(hal, data, OCS_CMD_NOWAIT, __ocs_hal_domain_cb, domain))
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);

		break;
	case OCS_EVT_EXIT:
		break;
	case OCS_EVT_RESPONSE:
		ocs_sm_transition(ctx, __ocs_hal_domain_alloc_read_sparm64, data);
		break;
	case OCS_EVT_ERROR:
		ocs_sm_transition(ctx, __ocs_hal_domain_alloc_report_fail, data);
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_alloc_reg_fcfi(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER: {
		sli4_cmd_rq_cfg_t rq_cfg[SLI4_CMD_REG_FCFI_NUM_RQ_CFG];
		uint32_t i;

		/* Set the filter match/mask values from hal's filter_def values */
		for (i = 0; i < SLI4_CMD_REG_FCFI_NUM_RQ_CFG; i++) {
			rq_cfg[i].rq_id = 0xffff;
			rq_cfg[i].r_ctl_mask = (uint8_t) hal->config.filter_def[i];
			rq_cfg[i].r_ctl_match = (uint8_t) (hal->config.filter_def[i] >> 8);
			rq_cfg[i].type_mask = (uint8_t) (hal->config.filter_def[i] >> 16);
			rq_cfg[i].type_match = (uint8_t) (hal->config.filter_def[i] >> 24);
		}

		/* Set the rq_id for each, in order of RQ definition */
		for (i = 0; i < hal->hal_rq_count; i++) {
			if (i >= ARRAY_SIZE(rq_cfg)) {
				ocs_log_warn(hal->os, "more RQs than REG_FCFI filter entries\n");
				break;
			}
			rq_cfg[i].rq_id = hal->hal_rq[i]->hdr->id;
		}

		if (!data) {
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		if (hal->hal_mrq_used) {
			if (OCS_HAL_RTN_SUCCESS != ocs_hal_config_mrq(hal, SLI4_CMD_REG_FCFI_SET_FCFI_MODE,
				 domain->vlan_id, domain->fcf)) {
				ocs_log_err(hal->os, "REG_FCFI_MRQ format failure\n");
				ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
				break;
			}

		} else {
			if (0 == sli_cmd_reg_fcfi(&hal->sli, data, SLI4_BMBX_SIZE, domain->fcf,
						rq_cfg, domain->vlan_id)) {
				ocs_log_err(hal->os, "REG_FCFI format failure\n");
				ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
				break;
			}
		}

		if (ocs_hal_command(hal, data, OCS_CMD_NOWAIT, __ocs_hal_domain_cb, domain))
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);

		break;
	}
	case OCS_EVT_EXIT:
		break;
	case OCS_EVT_RESPONSE:
		if (!data) {
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		domain->fcf_indicator = ((sli4_cmd_reg_fcfi_t *)data)->fcfi;

		/*
		 * IF_TYPE 0 devices do not support explicit VFI and VPI initialization
		 * and instead rely on implicit initialization during VFI registration.
		 * Short circuit normal processing here for those devices.
		 */
		if (SLI4_IF_TYPE_BE3_SKH_PF == sli_get_if_type(&hal->sli)) {
			ocs_sm_transition(ctx, __ocs_hal_domain_alloc_read_sparm64, data);
		} else {
			ocs_sm_transition(ctx, __ocs_hal_domain_alloc_init_vfi, data);
		}
		break;
	case OCS_EVT_ERROR:
		ocs_sm_transition(ctx, __ocs_hal_domain_alloc_report_fail, data);
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		if (sli_get_medium(&hal->sli) == SLI_LINK_MEDIUM_FC) {
			/*
			 * For FC, the HAL alread registered a FCFI
			 * Copy FCF information into the domain and jump to INIT_VFI
			 */
			domain->fcf_indicator = hal->fcf_indicator;
			ocs_sm_transition(&domain->sm, __ocs_hal_domain_alloc_init_vfi, data);
		} else {
			ocs_sm_transition(&domain->sm, __ocs_hal_domain_alloc_reg_fcfi, data);
		}
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_free_report_fail(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		if (domain != NULL) {
			ocs_hal_t	*hal = domain->hal;

			ocs_hal_domain_del(hal, domain);
			ocs_dma_free(hal->os, &domain->dma);

			if (hal->callback.domain != NULL) {
				hal->callback.domain(hal->args.domain,
						     OCS_HAL_DOMAIN_FREE_FAIL,
						     domain);
			}
		}

		// free command buffer
		if (data != NULL) {
			ocs_free(domain != NULL ? domain->hal->os : NULL, data, SLI4_BMBX_SIZE);
		}
		break;
	case OCS_EVT_EXIT:
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_freed(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* First free the BMBX data */
		if (data != NULL)
			ocs_free(domain != NULL ? domain->hal->os : NULL, data, SLI4_BMBX_SIZE);

		/* Free DMA and mailbox buffer */
		if (domain != NULL) {
			ocs_hal_t *hal = domain->hal;

			/* free VFI resource */
			if (sli_resource_free(&hal->sli, SLI_RSRC_FCOE_VFI, domain->indicator))
				ocs_log_err(hal->os, "FCOE_VFI (%d) free failed\n", domain->indicator);

			ocs_hal_domain_del(hal, domain);
			ocs_dma_free(hal->os, &domain->dma);

			/* inform registered callbacks */
			if (hal->callback.domain != NULL) {
				hal->callback.domain(hal->args.domain, OCS_HAL_DOMAIN_FREE_OK, domain);
			}
		}
		break;
	case OCS_EVT_EXIT:
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}


static void *
__ocs_hal_domain_free_redisc_fcf(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		/* if we're in the middle of a teardown, skip sending rediscover */
		if (hal->state == OCS_HAL_STATE_TEARDOWN_IN_PROGRESS) {
			ocs_sm_transition(ctx, __ocs_hal_domain_freed, data);
			break;
		}
		if (0 == sli_cmd_fcoe_rediscover_fcf(&hal->sli, data, SLI4_BMBX_SIZE, domain->fcf)) {
			ocs_log_err(hal->os, "REDISCOVER_FCF format failure\n");
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		if (ocs_hal_command(hal, data, OCS_CMD_NOWAIT, __ocs_hal_domain_cb, domain))
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);

		break;
	case OCS_EVT_RESPONSE:
	case OCS_EVT_ERROR:
		/* REDISCOVER_FCF can fail if none exist */
		ocs_sm_transition(ctx, __ocs_hal_domain_freed, data);
		break;
	case OCS_EVT_EXIT:
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_free_unreg_fcfi(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;

	smtrace("domain");

	switch (evt) {
	case OCS_EVT_ENTER:
		if (data == NULL) {
			data = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
			if (!data) {
				ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
				break;
			}
		}

		if (0 == sli_cmd_unreg_fcfi(&hal->sli, data, SLI4_BMBX_SIZE, domain->fcf_indicator)) {
			ocs_log_err(hal->os, "UNREG_FCFI format failure\n");
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		if (ocs_hal_command(hal, data, OCS_CMD_NOWAIT, __ocs_hal_domain_cb, domain)) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
		}

		break;
	case OCS_EVT_RESPONSE:
		if (domain->req_rediscover_fcf) {
			domain->req_rediscover_fcf = FALSE;
			ocs_sm_transition(ctx, __ocs_hal_domain_free_redisc_fcf, data);
		} else {
			ocs_sm_transition(ctx, __ocs_hal_domain_freed, data);
		}
		break;
	case OCS_EVT_ERROR:
		ocs_sm_transition(ctx, __ocs_hal_domain_free_report_fail, data);
		break;
	case OCS_EVT_EXIT:
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

static void *
__ocs_hal_domain_free_unreg_vfi(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *data)
{
	ocs_domain_t	*domain = ctx->app;
	ocs_hal_t	*hal = domain->hal;
	uint8_t		is_fc = FALSE;

	smtrace("domain");

	if ((hal->state != OCS_HAL_STATE_ACTIVE) && (evt == OCS_EVT_ERROR)) {
		/* If hal is not active, ignore error and free domain */
		evt = OCS_EVT_RESPONSE;
	}

	is_fc = (sli_get_medium(&hal->sli) == SLI_LINK_MEDIUM_FC);

	switch (evt) {
	case OCS_EVT_ENTER:
		if (data == NULL) {
			data = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
			if (!data) {
				ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
				break;
			}
		}

		if (0 == sli_cmd_unreg_vfi(&hal->sli, data, SLI4_BMBX_SIZE, domain,
					SLI4_UNREG_TYPE_DOMAIN)) {
			ocs_log_err(hal->os, "UNREG_VFI format failure\n");
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
			break;
		}

		if (hal->state != OCS_HAL_STATE_ACTIVE) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
			ocs_sm_post_event(ctx, OCS_EVT_RESPONSE, NULL);
			break;
		}

		if (ocs_hal_command(hal, data, OCS_CMD_NOWAIT, __ocs_hal_domain_cb, domain)) {
			ocs_free(hal->os, data, SLI4_BMBX_SIZE);
			ocs_sm_post_event(ctx, OCS_EVT_ERROR, NULL);
		}

		break;
	case OCS_EVT_ERROR:
		if (is_fc) {
			ocs_sm_transition(ctx, __ocs_hal_domain_free_report_fail, data);
		} else {
			ocs_sm_transition(ctx, __ocs_hal_domain_free_unreg_fcfi, data);
		}
		break;
	case OCS_EVT_RESPONSE:
		if (is_fc) {
			ocs_sm_transition(ctx, __ocs_hal_domain_freed, data);
		} else {
			ocs_sm_transition(ctx, __ocs_hal_domain_free_unreg_fcfi, data);
		}
		break;
	default:
		__ocs_hal_domain_common(__func__, ctx, evt, data);
		break;
	}

	return NULL;
}

// callback for domain alloc/attach/free
static int32_t
__ocs_hal_domain_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	ocs_domain_t *domain = arg;
	sli4_mbox_command_header_t *hdr = (sli4_mbox_command_header_t *)mqe;
	ocs_sm_event_t evt;

	if (status || hdr->status) {
		evt = OCS_EVT_ERROR;
	} else {
		evt = OCS_EVT_RESPONSE;
	}

	ocs_sm_post_event(&domain->sm, evt, mqe);
	return 0;
}

static void
target_wqe_timer_cb(void *arg)
{
	ocs_hal_t *hal = (ocs_hal_t *)arg;
	ocs_hal_io_t *io = NULL;
	ocs_hal_io_t *io_next = NULL;
	uint64_t ticks_current = ocs_get_os_ticks();
	uint32_t sec_elapsed;
	ocs_list_t io_timed_out_wqe;
	bool generate_fw_dump = false;
	int rc = 0;

	/* delete existing timer; will kick off new timer after checking wqe timeouts */
	hal->in_active_wqe_timer = TRUE;
	ocs_del_timer(&hal->wqe_timer);

	/* Initialize the local 'io_timed_out_wqe' list */
	ocs_list_init(&io_timed_out_wqe, ocs_hal_io_t, timed_out_wqe_link);

	/* Loop through active WQE list and check for timeouts */
	ocs_lock(&hal->io_timed_wqe_lock);
		ocs_list_foreach_safe(&hal->io_timed_wqe, io, io_next) {
			sec_elapsed = ((ticks_current - io->submit_ticks) / ocs_get_os_tick_freq());

			/*
			 * If elapsed time > timeout, abort it. No need to check type since
			 * it wouldn't be on this list unless it was a target WQE
			 */
			if (sec_elapsed > io->tgt_wqe_timeout) {
				/*
				 * If the chip is in an error state (UE'd) then don't abort the IO
				 * as recovery path will be detect the IO timeout and send appropiate
				 * status to backend target
				 */
				if (ocs_hal_reset_pending(hal)) {
					ocs_unlock(&hal->io_timed_wqe_lock);
					goto exit_target_wqe_timer_cb;
				}

				ocs_log_info(hal->os, "[INI WWPN: 0x%" PRIx64 "] [TGT WWPN: 0x%" PRIx64 "] "\
					     "IO timeout io=%p xri=%#x tag=%#x type=%d\n",
					     ocs_node_get_wwpn(io->rnode->node), io->rnode->sport->wwpn,
					     io, io->indicator, io->reqtag, io->type);

				if (io->type == OCS_HAL_ELS_REQ) {
					/* Remove from active_wqe list so that we won't try to abort again */
					ocs_list_remove(&hal->io_timed_wqe, io);
					ocs_log_err(hal->os, "ELS WQE (XRI: 0x%x) completion timedout; "\
							"Collecting FW dump\n", io->indicator);
					generate_fw_dump = true;
					continue;
				}

				/* now abort outstanding IO */
				rc = ocs_hal_io_abort(hal, io, TRUE, NULL, NULL);
				if (OCS_HAL_RTN_NO_RESOURCES == rc) {
					/* Try to abort this IO request later */
					continue;
				}

				/* Remove from active_wqe list so that we won't try to abort again */
				ocs_list_remove(&hal->io_timed_wqe, io);
				if (OCS_HAL_RTN_ERROR == rc) {
					/* Add this IO to a local list and send failure status later */
					ocs_list_add_tail(&io_timed_out_wqe, io);
				}
			} else {
				/*
				 * Since all the IO (non-ELS) REQ's have the same timeout value,
				 * break out of the loop at the first IO REQ that hasn't expired
				 */
				if (OCS_HAL_ELS_REQ != io->type) {
					break;
				}
			}
		}
	ocs_unlock(&hal->io_timed_wqe_lock);

	/* Report IO timeout failure status to backend target driver */
	ocs_list_foreach_safe(&io_timed_out_wqe, io, io_next) {
		ocs_list_remove(&io_timed_out_wqe, io);
		ocs_scsi_io_timedout(io->ul_io);
	}

	if (generate_fw_dump)
		ocs_device_trigger_fw_dump((ocs_t *)hal->os, OCS_FW_FUNC_DESC_DUMP);

exit_target_wqe_timer_cb:
	/* if we're not in the middle of shutting down, schedule next timer */
	if (!hal->active_wqe_timer_shutdown) {
		ocs_setup_timer(hal->os, &hal->wqe_timer, target_wqe_timer_cb, hal, OCS_HAL_WQ_TIMER_PERIOD_MS, false);
	} else {
		ocs_log_info(hal->os, "active_wqe_timer_shutdown set, do not re-arm wqe timer\n");
	}
	hal->in_active_wqe_timer = FALSE;
}

static void
shutdown_target_wqe_timer(ocs_hal_t *hal)
{
	uint32_t	iters = 100;

	if (hal->config.emulate_tgt_wqe_timeout) {
		/* request active wqe timer shutdown, then wait for it to complete */
		hal->active_wqe_timer_shutdown = TRUE;

		/* delete WQE timer and wait for timer handler to complete (if necessary) */
		ocs_del_timer(&hal->wqe_timer);

		/* now wait for timer handler to complete (if necessary) */
		while (hal->in_active_wqe_timer && iters) {
			/*
			 * if we happen to have just sent NOP mailbox command, make sure
			 * completions are being processed
			 */
			ocs_hal_flush(hal);
			ocs_delay_usec(10000);
			iters--;
		}

		if (iters == 0)
			ocs_log_test(hal->os, "Failed to shutdown active wqe timer\n");
	}
}

/**
 * @brief Determine if HAL IO is owned by the port.
 *
 * @par Description
 * Determines if the given HAL IO has been posted to the chip.
 *
 * @param hal Hardware context allocated by the caller.
 * @param io HAL IO.
 *
 * @return Returns TRUE if given HAL IO is port-owned.
 */
bool ocs_hal_io_port_owned(ocs_hal_io_t *io)
{
	if (!io)
		return false;

	return io->is_port_owned;
}

/**
 * @brief Return TRUE if exchange is port-owned.
 *
 * @par Description
 * Test to see if the xri is a port-owned xri.
 *
 * @param hal Hardware context.
 * @param xri Exchange indicator.
 *
 * @return Returns TRUE if XRI is a port owned XRI.
 */

uint8_t
ocs_hal_is_xri_port_owned(ocs_hal_t *hal, uint32_t xri)
{
	ocs_hal_io_t *io = ocs_hal_io_lookup(hal, xri);
	return (io == NULL ? FALSE : io->is_port_owned);
}

/**
 * @brief Returns an XRI from the port owned list to the host.
 *
 * @par Description
 * Used when the POST_XRI command fails as well as when the RELEASE_XRI completes.
 *
 * @param hal Hardware context.
 * @param xri_base The starting XRI number.
 * @param xri_count The number of XRIs to free from the base.
 */
static void
ocs_hal_reclaim_xri(ocs_hal_t *hal, uint16_t xri_base, uint16_t xri_count)
{
	ocs_hal_io_t	*io;
	uint32_t i;

	for (i = 0; i < xri_count; i++) {
		io = ocs_hal_io_lookup(hal, xri_base + i);

		/*
		 * If this is a TOW XRI, then we need to release any
		 * buffer attached to the XRI before moving the XRI back to the free pool.
		 */
		if (hal->tow_enabled)
			ocs_hal_rqpair_tow_move_to_host(hal, io);

		ocs_lock(&hal->io_lock);
			ocs_list_remove(&hal->io_port_owned, io);
			io->is_port_owned = 0;
			ocs_list_add_tail(&hal->io_free, io);
		ocs_unlock(&hal->io_lock);
	}
}

/**
 * @brief Called when the POST_XRI command completes.
 *
 * @par Description
 * Free the mailbox command buffer and reclaim the XRIs on failure.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_cb_post_xri(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_post_xri_t *post_xri = (sli4_cmd_post_xri_t *)mqe;

	/* Reclaim the XRIs as host owned if the command fails */
	if (status || post_xri->hdr.status) {
		ocs_log_debug(hal->os, "Status 0x%x for XRI base 0x%x, cnt =x%x\n",
			      status, post_xri->xri_base, post_xri->xri_count);
		ocs_hal_reclaim_xri(hal, post_xri->xri_base, post_xri->xri_count);
	}

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @brief Issues a mailbox command to move XRIs from the host-controlled pool to the port.
 *
 * @param hal Hardware context.
 * @param xri_start The starting XRI to post.
 * @param num_to_post The number of XRIs to post.
 *
 * @return Returns OCS_HAL_RTN_NO_MEMORY, OCS_HAL_RTN_ERROR, or OCS_HAL_RTN_SUCCESS.
 */

static ocs_hal_rtn_e
ocs_hal_post_xri(ocs_hal_t *hal, uint32_t xri_start, uint32_t num_to_post)
{
	uint8_t	*post_xri;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;

	/* Since we need to allocate for mailbox queue, just always allocate */
	post_xri = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (post_xri == NULL) {
		ocs_log_err(hal->os, "no buffer for command\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* Register the XRIs */
	sli_cmd_post_xri(&hal->sli, post_xri, SLI4_BMBX_SIZE, xri_start, num_to_post);
	rc = ocs_hal_command(hal, post_xri, OCS_CMD_NOWAIT, ocs_hal_cb_post_xri, NULL);
	if (rc)
		ocs_free(hal->os, post_xri, SLI4_BMBX_SIZE);

	return rc;
}

/**
 * @brief Move XRIs from the host-controlled pool to the port.
 *
 * @par Description
 * Removes IOs from the free list and moves them to the port.
 *
 * @param hal Hardware context.
 * @param num_xri The number of XRIs being requested to move to the chip.
 *
 * @return Returns the number of XRIs that were moved.
 */
uint32_t
ocs_hal_xri_move_to_port_owned(ocs_hal_t *hal, uint32_t num_xri)
{
	ocs_hal_io_t	*io;
	uint32_t i;
	uint32_t num_posted = 0;

	/*
	 * Note: We cannot use ocs_hal_io_alloc() because that would place the
	 *       IO on the io_inuse list. We need to move from the io_free to
	 *       the io_port_owned list.
	 */
	ocs_lock(&hal->io_lock);

	for (i = 0; i < num_xri; i++) {
		if (NULL != (io = ocs_list_remove_head(&hal->io_free))) {
			ocs_hal_rtn_e rc;

			/*
			 * For TOW, we need to attach a buffer to the XRI before
			 * submitting it to the chip.
			 * If a buffer is unavailable, then we cannot post it, so return it
			 * to the free pool.
			 */
			if (hal->tow_enabled) {
				/* Note: uses the IO lock to get the tow buffer */
				ocs_unlock(&hal->io_lock);
				rc = ocs_hal_rqpair_tow_move_to_port(hal, io);
				ocs_lock(&hal->io_lock);
				if (rc != OCS_HAL_RTN_SUCCESS) {
					ocs_list_add_head(&hal->io_free, io);
					break;
				}
			}

			ocs_lock_init(hal->os, &io->tow_lock, "tow_lock[%d]", io->indicator);
			io->is_port_owned = 1;
			ocs_list_add_tail(&hal->io_port_owned, io);

			/* Post XRI */
			if (ocs_hal_post_xri(hal, io->indicator, 1) != OCS_HAL_RTN_SUCCESS ) {
				ocs_log_info(hal->os, "Post xri failed! reclaim xri %d\n", io->indicator);
				ocs_hal_reclaim_xri(hal, io->indicator, i);
				break;
			}
			num_posted++;
		} else {
			/* no more free XRIs */
			break;
		}
	}
	ocs_unlock(&hal->io_lock);

	return num_posted;
}

/**
 * @brief Called when the RELEASE_XRI command completes.
 *
 * @par Description
 * Move the IOs back to the free pool on success.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_cb_release_xri(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	sli4_cmd_release_xri_t *release_xri = (sli4_cmd_release_xri_t *)mqe;
	uint8_t i;

	/* Reclaim the XRIs as host owned if the command succeeds */
	if ((0 == status) && (0 == release_xri->hdr.status)) {
		for (i = 0; i < release_xri->released_xri_count; i++) {
			uint16_t xri = ((i & 1) == 0 ? release_xri->xri_tbl[i/2].xri_tag0 :
					release_xri->xri_tbl[i/2].xri_tag1);

			ocs_hal_reclaim_xri(hal, xri, 1);
		}
	}

	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @brief Move XRIs from the port-controlled pool to the host.
 *
 * Requests XRIs from the FW to return to the host-owned pool.
 *
 * @param hal Hardware context.
 * @param num_xri The number of XRIs being requested to moved from the chip.
 *
 * @return Returns 0 for success, or a negative error code value for failure.
 */

ocs_hal_rtn_e
ocs_hal_xri_move_to_host_owned(ocs_hal_t *hal, uint8_t num_xri)
{
	uint8_t	*release_xri;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;

	/* non-local buffer required for mailbox queue */
	release_xri = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_ZERO | OCS_M_NOWAIT);
	if (release_xri == NULL) {
		ocs_log_err(hal->os, "no buffer for command\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* Release the XRIs */
	sli_cmd_release_xri(&hal->sli, release_xri, SLI4_BMBX_SIZE, num_xri);
	rc = ocs_hal_command(hal, release_xri, OCS_CMD_NOWAIT, ocs_hal_cb_release_xri, NULL);
	if (rc)
		ocs_free(hal->os, release_xri, SLI4_BMBX_SIZE);

	return rc;
}

/**
 * @brief Free an ocs_hal_rq_buffer_t array.
 *
 * @par Description
 * The ocs_hal_rq_buffer_t array is freed, along with allocated DMA memory.
 *
 * @param hal Pointer to HAL object.
 * @param rq_buf Pointer to ocs_hal_rq_buffer_t array.
 * @param count Count of buffers in array.
 *
 * @return None.
 */
static void
ocs_hal_rx_buffer_free(ocs_hal_t *hal, ocs_hal_rq_buffer_t **rq_buf, uint32_t count)
{
	ocs_t *ocs = hal->os;
	uint32_t i;

	if (rq_buf) {
		for (i = 0; i < count && rq_buf[i]; i++) {
			ocs_dma_free(ocs, &rq_buf[i]->dma);
			ocs_free(hal->os, rq_buf[i], sizeof(ocs_hal_rq_buffer_t));
			rq_buf[i] = NULL;
		}

		ocs_free(hal->os, rq_buf, count * sizeof(ocs_hal_rq_buffer_t *));
	}
}

/**
 * @brief Allocate an ocs_hal_rq_buffer_t array.
 *
 * @par Description
 * An ocs_hal_rq_buffer_t array is allocated, along with the required DMA memory.
 *
 * @param hal Pointer to HAL object.
 * @param rqindex RQ index for this buffer.
 * @param count Count of buffers in array.
 * @param size Size of buffer.
 *
 * @return Returns the pointer to the allocated ocs_hal_rq_buffer_t array.
 */
static ocs_hal_rq_buffer_t **
ocs_hal_rx_buffer_alloc(ocs_hal_t *hal, uint32_t rqindex, uint32_t count, uint32_t size)
{
	ocs_t *ocs = hal->os;
	ocs_hal_rq_buffer_t **rq_buf = NULL;
	uint32_t i;

	if (count != 0) {
		rq_buf = ocs_malloc(hal->os, count * sizeof(ocs_hal_rq_buffer_t *), OCS_M_ZERO);
		if (!rq_buf) {
			ocs_log_err(hal->os, "Failed to alloc pointer for unsolicited DMA trackers\n");
			goto free_rx_buffer;
		}

		for (i = 0; i < count; i++) {
			rq_buf[i] = ocs_malloc(hal->os, sizeof(ocs_hal_rq_buffer_t), OCS_M_ZERO);
			if (!rq_buf[i]) {
				ocs_log_err(hal->os, "Failed to alloc unsolicited DMA tracker[%d]\n", i);
				goto free_rx_buffer;
			}

			rq_buf[i]->rqindex = rqindex;
			if (ocs_dma_alloc(ocs, &rq_buf[i]->dma, size, OCS_MIN_DMA_ALIGNMENT)) {
				ocs_log_err(hal->os, "DMA allocation failed\n");
				goto free_rx_buffer;
			}
		}
	}

	return rq_buf;

free_rx_buffer:
	ocs_hal_rx_buffer_free(hal, rq_buf, count);
	return NULL;
}

/**
 * @brief Allocate the RQ data buffers.
 *
 * @param hal Pointer to HAL object.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_rx_allocate(ocs_hal_t *hal)
{
	ocs_t *ocs = hal->os;
	uint32_t i;
	int32_t rc = OCS_HAL_RTN_SUCCESS;
	uint32_t rqindex = 0;
	hal_rq_t *rq;
	uint32_t hdr_size = OCS_HAL_RQ_SIZE_HDR;
	uint32_t payload_size = hal->config.rq_default_buffer_size;

	rqindex = 0;

	for (i = 0; i < hal->hal_rq_count; i++) {
		rq = hal->hal_rq[i];

		/* skip over nvme qs */
		if (rq->nvmeq) {
			rqindex += 2;
			continue;
		}

		/* Allocate header buffers */
		rq->hdr_buf = ocs_hal_rx_buffer_alloc(hal, rqindex, rq->entry_count, hdr_size);
		if (rq->hdr_buf == NULL) {
			ocs_log_err(ocs, "ocs_hal_rx_buffer_alloc hdr_buf failed\n");
			rc = OCS_HAL_RTN_ERROR;
			break;
		}

		ocs_log_debug(hal->os, "rq[%2d] rq_id %02d header %4d by %4d bytes\n", i, rq->hdr->id,
			      rq->entry_count, hdr_size);

		rqindex++;

		/* Allocate payload buffers */
		rq->payload_buf = ocs_hal_rx_buffer_alloc(hal, rqindex, rq->entry_count, payload_size);
		if (rq->payload_buf == NULL) {
			ocs_log_err(ocs, "ocs_hal_rx_buffer_alloc fb_buf failed\n");
			rc = OCS_HAL_RTN_ERROR;
			break;
		}
		ocs_log_debug(hal->os, "rq[%2d] rq_id %02d default %4d by %4d bytes\n", i, rq->data->id,
			      rq->entry_count, payload_size);
		rqindex++;
	}

	return rc ? OCS_HAL_RTN_ERROR : OCS_HAL_RTN_SUCCESS;
}

/**
 * @brief Post the RQ data buffers to the chip.
 *
 * @param hal Pointer to HAL object.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_rx_post(ocs_hal_t *hal)
{
	uint32_t i;
	uint32_t idx;
	uint32_t rq_idx;
	int32_t rc = 0;

	/*
	 * In RQ pair mode, we MUST post the header and payload buffer at the
	 * same time.
	 */
	for (rq_idx = 0, idx = 0; rq_idx < hal->hal_rq_count; rq_idx++) {
		hal_rq_t *rq = hal->hal_rq[rq_idx];

		if (rq->nvmeq)
			continue;

		for (i = 0; i < rq->entry_count-1; i++) {
			ocs_hal_sequence_t *seq = ocs_array_get(hal->seq_pool, idx++);
			ocs_hal_assert(seq != NULL);

			seq->header.data = rq->hdr_buf[i]->dma.virt;
			seq->header.data_len = rq->hdr_buf[i]->dma.len;
			seq->header.phys_buf = rq->hdr_buf[i]->dma.phys;
			seq->header.rqindex  = rq->hdr_buf[i]->rqindex;

			seq->payload.data = rq->payload_buf[i]->dma.virt;
			seq->payload.data_len = rq->payload_buf[i]->dma.len;
			seq->payload.phys_buf = rq->payload_buf[i]->dma.phys;
			seq->payload.rqindex = rq->payload_buf[i]->rqindex;

			rc = ocs_hal_sequence_free(hal, seq);
			if (rc) {
				break;
			}
		}
		if (rc) {
			break;
		}
	}

	return rc;
}

/**
 * @brief Free the RQ data buffers.
 *
 * @param hal Pointer to HAL object.
 *
 */
void
ocs_hal_rx_free(ocs_hal_t *hal)
{
	hal_rq_t *rq;
	uint32_t i;

	/* Free hal_rq buffers */
	for (i = 0; i < hal->hal_rq_count; i++) {
		rq = hal->hal_rq[i];
		if (rq != NULL) {
			if (rq->nvmeq)
				continue;

			ocs_hal_rx_buffer_free(hal, rq->hdr_buf, rq->entry_count);
			rq->hdr_buf = NULL;
			ocs_hal_rx_buffer_free(hal, rq->payload_buf, rq->entry_count);
			rq->payload_buf = NULL;
		}
	}
}

/**
 * @brief HAL async call context structure.
 */
typedef struct {
	ocs_hal_async_cb_t callback;
	void *arg;
	uint8_t cmd[SLI4_BMBX_SIZE];
} ocs_hal_async_call_ctx_t;

/**
 * @brief HAL async callback handler
 *
 * @par Description
 * This function is called when the NOP mailbox command completes.  The callback stored
 * in the requesting context is invoked.
 *
 * @param hal Pointer to HAL object.
 * @param status Completion status.
 * @param mqe Pointer to mailbox completion queue entry.
 * @param arg Caller-provided argument.
 *
 * @return None.
 */
static void
ocs_hal_async_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void *arg)
{
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_nop_t *sli4_rsp = NULL;
	ocs_hal_async_call_ctx_t *ctx = arg;

	sli4_rsp = (sli4_res_common_nop_t *)mbox_rsp->payload.embed;
	if (sli4_rsp->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &sli4_rsp->hdr);

	if (!ctx || !ctx->callback)
		goto ocs_hal_async_cb_done;

	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (sli4_rsp->hdr.status)
			status = sli4_rsp->hdr.status;
	}

	(*ctx->callback)(hal, status, mqe, ctx->arg);

ocs_hal_async_cb_done:
	if (ctx)
		ocs_free(hal->os, ctx, sizeof(*ctx));
}

/**
 * @brief Make an async callback using NOP mailbox command
 *
 * @par Description
 * Post a NOP mailbox command; the callback with argument is invoked upon completion
 * while in the event processing context.
 *
 * @param hal Pointer to HAL object.
 * @param callback Pointer to callback function.
 * @param arg Caller-provided callback.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
int32_t
ocs_hal_async_call(ocs_hal_t *hal, ocs_hal_async_cb_t callback, void *arg)
{
	int32_t rc = 0;
	ocs_hal_async_call_ctx_t *ctx;

	/*
	 * Allocate a callback context (which includes the mailbox command buffer), we need
	 * this to be persistent as the mailbox command submission may be queued and executed later
	 * execution.
	 */
	ctx = ocs_malloc(hal->os, sizeof(*ctx), OCS_M_ZERO | OCS_M_NOWAIT);
	if (ctx == NULL) {
		ocs_log_err(hal->os, "failed to malloc async call context\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	ctx->callback = callback;
	ctx->arg = arg;

	/* Build and send a NOP mailbox command */
	sli_cmd_common_nop(&hal->sli, ctx->cmd, sizeof(ctx->cmd), 0);
	rc = ocs_hal_command(hal, ctx->cmd, OCS_CMD_NOWAIT, ocs_hal_async_cb, ctx);
	if (rc)
		ocs_free(hal->os, ctx, sizeof(*ctx));

	return rc;
}

/**
 * @brief Initialize the reqtag pool.
 *
 * @par Description
 * The WQ request tag pool is initialized.
 *
 * @param hal Pointer to HAL object.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
ocs_hal_rtn_e
ocs_hal_reqtag_init(ocs_hal_t *hal)
{
	if (hal->wq_reqtag_pool == NULL) {
		hal->wq_reqtag_pool = ocs_pool_alloc(hal->os, sizeof(hal_wq_callback_t), 65536, TRUE);
		if (hal->wq_reqtag_pool == NULL) {
			ocs_log_err(hal->os, "ocs_pool_alloc hal_wq_callback_t failed\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}
	}
	ocs_hal_reqtag_reset(hal);
	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @brief Allocate a WQ request tag.
 *
 * Allocate and populate a WQ request tag from the WQ request tag pool.
 *
 * @param hal Pointer to HAL object.
 * @param callback Callback function.
 * @param arg Pointer to callback argument.
 *
 * @return Returns pointer to allocated WQ request tag, or NULL if object cannot be allocated.
 */
hal_wq_callback_t *
ocs_hal_reqtag_alloc(ocs_hal_t *hal, void (*callback)(void *arg, uint8_t *cqe, int32_t status), void *arg)
{
	hal_wq_callback_t *wqcb;

	ocs_hal_assert(callback != NULL);

	wqcb = ocs_pool_get(hal->wq_reqtag_pool);
	if (wqcb != NULL) {
		ocs_hal_assert(wqcb->callback == NULL);
		wqcb->callback = callback;
		wqcb->arg = arg;
	}
	return wqcb;
}

/**
 * @brief Free a WQ request tag.
 *
 * Free the passed in WQ request tag.
 *
 * @param hal Pointer to HAL object.
 * @param wqcb Pointer to WQ request tag object to free.
 *
 * @return None.
 */
void
ocs_hal_reqtag_free(ocs_hal_t *hal, hal_wq_callback_t *wqcb)
{
	ocs_hal_assert(wqcb->callback != NULL);
	wqcb->callback = NULL;
	wqcb->arg = NULL;
	ocs_pool_put(hal->wq_reqtag_pool, wqcb);
}

/**
 * @brief Return WQ request tag by index.
 *
 * @par Description
 * Return pointer to WQ request tag object given an index.
 *
 * @param hal Pointer to HAL object.
 * @param instance_index Index of WQ request tag to return.
 *
 * @return Pointer to WQ request tag, or NULL.
 */
hal_wq_callback_t *
ocs_hal_reqtag_get_instance(ocs_hal_t *hal, uint32_t instance_index)
{
	hal_wq_callback_t *wqcb;

	wqcb = ocs_pool_get_instance(hal->wq_reqtag_pool, instance_index);
	if (wqcb == NULL)
		ocs_log_err(hal->os, "wqcb for instance %d is null\n", instance_index);

	return wqcb;
}

/**
 * @brief Reset the WQ request tag pool.
 *
 * @par Description
 * Reset the WQ request tag pool, returning all to the free list.
 *
 * @param hal pointer to HAL object.
 *
 * @return None.
 */
void
ocs_hal_reqtag_reset(ocs_hal_t *hal)
{
	hal_wq_callback_t *wqcb;
	uint32_t i;

	/* Remove all from freelist */
	while(ocs_pool_get(hal->wq_reqtag_pool) != NULL) {
		;
	}

	/* Put them all back */
	for (i = 0; ((wqcb = ocs_pool_get_instance(hal->wq_reqtag_pool, i)) != NULL); i++) {
		wqcb->instance_index = i;
		wqcb->callback = NULL;
		wqcb->arg = NULL;

		/* Set aside a specific reqtag for REQUEUE_XRI WQE completion with error status */
		if (i != OCS_HAL_REQUE_XRI_REGTAG) {
			ocs_pool_put(hal->wq_reqtag_pool, wqcb);
		}
	}
}

/**
 * @brief Handle HAL assertion
 *
 * HAL assert, display diagnostic message, and abort.
 *
 * @param cond string describing failing assertion condition
 * @param filename file name
 * @param linenum line number
 *
 * @return none
 */
void
_ocs_hal_assert(const char *cond, const char *filename, int linenum)
{
	ocs_printf("%s(%d): HAL assertion (%s) failed\n", filename, linenum, cond);
	ocs_abort();
		/* no return */
}

/**
 * @brief Handle HAL verify
 *
 * HAL verify, display diagnostic message, dump stack and return.
 *
 * @param cond string describing failing verify condition
 * @param filename file name
 * @param linenum line number
 *
 * @return none
 */
void
_ocs_hal_verify(const char *cond, const char *filename, int linenum)
{
	ocs_printf("%s(%d): HAL verify (%s) failed\n", filename, linenum, cond);
	ocs_print_stack();
}

static void
ocs_hal_wq_handle_bad_reque_xri(void *arg, uint8_t *cqe, int32_t status)
{
	uint32_t *ptr = (uint32_t *)cqe;

	ocs_log_err(NULL, "requeue_xri failed, status = %d, CQE = [ %#x, %#x, %#x, %#x ]\n",
			status, *ptr, *(ptr + 1), *(ptr + 2), *(ptr + 3));
}

/**
 * @brief Reque XRI
 *
 * @par Description
 * Reque XRI
 *
 * @param hal Pointer to HAL object.
 * @param io Pointer to HAL IO
 *
 * @return Return 0 if successful else returns -1
 */
int32_t
ocs_hal_reque_xri(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	int32_t rc = 0;
	hal_wq_callback_t *wqcb;

	rc = ocs_hal_rqpair_tow_xri_buffer_attach(hal, io, TRUE);
	if (rc) {
		ocs_list_add_tail(&hal->io_port_dnrx, io);
		rc = -1;
		goto exit_ocs_hal_reque_xri;
	}

	wqcb = ocs_hal_reqtag_get_instance(hal, OCS_HAL_REQUE_XRI_REGTAG);
	if (!wqcb) {
		rc = -1;
		goto exit_ocs_hal_reque_xri;
	}

	wqcb->callback = ocs_hal_wq_handle_bad_reque_xri;
	wqcb->arg = wqcb;

	io->tow_dnrx = 0;
	io->type = OCS_HAL_IO_DNRX_REQUEUE;
	if (sli_requeue_xri_wqe(&hal->sli, io->wqe.wqebuf, hal->sli.config.wqe_size,
				io->indicator, OCS_HAL_REQUE_XRI_REGTAG, SLI4_CQ_DEFAULT)) {
		/* Clear buffer from XRI */
		ocs_pool_put(hal->tow_buffer_pool, io->tow_buf);
		io->tow_buf = NULL;

		ocs_log_err(hal->os, "requeue_xri WQE error\n");
		ocs_list_add_tail(&hal->io_port_dnrx, io);

		rc = -1;
		goto exit_ocs_hal_reque_xri;
	}

	io->reque_xri_wq = ocs_hal_queue_next_wq(hal, io);
	ocs_hal_assert(io->reque_xri_wq != NULL);

	/*
	 * Add IO to active io wqe list before submitting, in case the
	 * wcqe processing preempts this thread.
	 */
	OCS_STAT(hal->tcmd_wq_submit[io->reque_xri_wq->instance]++);
	OCS_STAT(io->reque_xri_wq->use_count++);

	rc = hal_wq_write(io->reque_xri_wq, &io->wqe);
	if (rc < 0) {
		ocs_log_err(hal->os, "sli_queue_write reque xri failed: %d\n", rc);
		rc = -1;
	}

exit_ocs_hal_reque_xri:
	return rc;
}

static void
ocs_hal_cb_set_profile_config_v0(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	ocs_hal_set_profile_config_v0_cb_arg_t *config_cb_arg = (ocs_hal_set_profile_config_v0_cb_arg_t *)arg;
	sli4_cmd_sli_config_t *mbox_rsp = (sli4_cmd_sli_config_t *)mqe;
	sli4_res_common_set_profile_config_t *res = NULL; 
	ocs_dma_t *payload = NULL;

	payload = &config_cb_arg->payload;
	res = (sli4_res_common_set_profile_config_t *)payload->virt;
	if (res->hdr.status)
		ocs_hal_log_sli4_rsp_hdr(hal, &res->hdr);
	
	if (0 == status) {
		if (mbox_rsp->hdr.status)
			status = mbox_rsp->hdr.status;
		else if (res->hdr.status)
			status = res->hdr.status;
	}

	if (config_cb_arg->cb) {
		config_cb_arg->cb(hal->os, status, config_cb_arg->cb_arg);
	}

	ocs_dma_free(hal->os, &config_cb_arg->payload);
	ocs_free(hal->os, config_cb_arg->vf_resc, sizeof(*config_cb_arg->vf_resc) * config_cb_arg->num_vfs);
	ocs_free(hal->os, config_cb_arg->mbxdata, SLI4_BMBX_SIZE);
	ocs_free(hal->os, config_cb_arg, sizeof(*config_cb_arg));
}

static void
ocs_hal_populate_fcfcoe_resc_desc(ocs_fcfcoe_vf_resc_t *vf_resc, sli4_rsrc_desc_fcfcoe_v0_t *fcfcoe_desc)
{
	fcfcoe_desc->desc_type = SLI4_RSRC_DESC_TYPE_FCFCOE;

	if (!vf_resc->add)
		fcfcoe_desc->del_resc = 1;
	fcfcoe_desc->immediate = vf_resc->immediate;
	if (!vf_resc->save)
		fcfcoe_desc->no_save = 1;

	fcfcoe_desc->pf_num = vf_resc->pf_num;
	fcfcoe_desc->vf_num = vf_resc->vf_num;
	fcfcoe_desc->xri_count = vf_resc->xri_count;
	fcfcoe_desc->rpi_count = vf_resc->rpi_count;
	fcfcoe_desc->vpi_count = vf_resc->vpi_count;
	fcfcoe_desc->vfi_count = vf_resc->vfi_count;
	fcfcoe_desc->fcfi_count = vf_resc->fcfi_count;

	fcfcoe_desc->cq_count = vf_resc->cq_count;
	fcfcoe_desc->eq_count = vf_resc->eq_count;
	fcfcoe_desc->rq_count = vf_resc->rq_count;
	fcfcoe_desc->wq_count = vf_resc->wq_count;

	fcfcoe_desc->link_type = vf_resc->link_type;
	fcfcoe_desc->link_num = vf_resc->link_num;

	fcfcoe_desc->bw_min = vf_resc->bw_min;
	fcfcoe_desc->bw_max = vf_resc->bw_max;
	fcfcoe_desc->iops_min = vf_resc->iops_min;
	fcfcoe_desc->iops_max = vf_resc->iops_max;

	ocs_memcpy(fcfcoe_desc->wwnn, vf_resc->wwnn, sizeof(vf_resc->wwnn));
	ocs_memcpy(fcfcoe_desc->wwpn, vf_resc->wwpn, sizeof(vf_resc->wwpn));
}

void
ocs_hal_setup_vf_resc_defaults(ocs_hal_t *hal, ocs_fcfcoe_vf_resc_t *vf_resc_tbl,
			       uint8_t pci_func, int32_t num_vfs,
			       uint64_t wwnn, uint64_t *wwpn_tbl)
{
	int32_t i;

	for (i = 0; i < num_vfs; i++) {
		vf_resc_tbl[i].add = 1;
		vf_resc_tbl[i].immediate = 1;
		vf_resc_tbl[i].save = 1;
		vf_resc_tbl[i].vf_num = i + 1;
		vf_resc_tbl[i].pf_num = pci_func;
		vf_resc_tbl[i].xri_count = SLI4_RSRC_DESC_KEEP_PRIOR_RESC_VALUE;
		vf_resc_tbl[i].rpi_count = SLI4_RSRC_DESC_KEEP_PRIOR_RESC_VALUE;
		vf_resc_tbl[i].vpi_count = SLI4_RSRC_DESC_KEEP_PRIOR_RESC_VALUE;
		vf_resc_tbl[i].vfi_count = SLI4_RSRC_DESC_KEEP_PRIOR_RESC_VALUE;
		vf_resc_tbl[i].fcfi_count = SLI4_RSRC_DESC_KEEP_PRIOR_RESC_VALUE;
		vf_resc_tbl[i].cq_count = SLI4_RSRC_DESC_KEEP_PRIOR_RESC_VALUE;
		vf_resc_tbl[i].eq_count = SLI4_RSRC_DESC_KEEP_PRIOR_RESC_VALUE;
		vf_resc_tbl[i].rq_count = SLI4_RSRC_DESC_KEEP_PRIOR_RESC_VALUE;
		vf_resc_tbl[i].wq_count = SLI4_RSRC_DESC_KEEP_PRIOR_RESC_VALUE;
		vf_resc_tbl[i].link_type = SLI4_RSRC_DESC_LINK_TYPE_FC;
		vf_resc_tbl[i].link_num = SLI4_RSRC_DESC_LINK_NUM_NO_CHANGE;
		vf_resc_tbl[i].bw_min = SLI4_RSRC_DESC_BW_NO_CHANGE;
		vf_resc_tbl[i].bw_max = SLI4_RSRC_DESC_BW_NO_CHANGE;
		vf_resc_tbl[i].iops_min = SLI4_RSRC_DESC_BW_NO_CHANGE;
		vf_resc_tbl[i].iops_max = SLI4_RSRC_DESC_BW_NO_CHANGE;

		vf_resc_tbl[i].wwnn[0] = (uint32_t)wwnn;
		vf_resc_tbl[i].wwnn[1] = (uint32_t)(wwnn >> 32);

		vf_resc_tbl[i].wwpn[0] = (uint32_t)wwpn_tbl[i];
		vf_resc_tbl[i].wwpn[1] = (uint32_t)(wwpn_tbl[i] >> 32);
		ocs_log_info(hal->os, "VF Resource [%d:%d] WWPN %#x %#x, WWNN %#x %#x\n",
			     pci_func, i + 1, vf_resc_tbl[i].wwpn[0],
			     vf_resc_tbl[i].wwpn[1], vf_resc_tbl[i].wwnn[0],
			     vf_resc_tbl[i].wwnn[1]);
	}
}

/**
 * @ingroup port
 * @brief Set SRIOV VF resources 
 *
 * @par Description
 * This function configures VF resource descriptors using
 * Set Profile Configuration MBOX.
 *
 * @param hal Hardware context.
 * @param pci_func PCI PF function of corresponding VFs
 * @param num_vfs Number of VFs to be configured
 * @param vf_resc_in VF resource table to be configured for VFs. Optional.
 * @param wwnn WWNN of VF. Uses same WWNN for all the VFs
 * @param wwpn_table WWPNs of all VFs
 * @param cb callback
 * @param cb_arg arg to callback
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
ocs_hal_set_sriov_vf_resources(ocs_hal_t *hal, int8_t pci_func, int32_t num_vfs,
			       void *vf_resc_in, uint64_t wwnn, uint64_t *wwpn_tbl,
			       ocs_sriov_vfs_set_wwn_cb_t cb, void *cb_arg)
{
	ocs_hal_set_profile_config_v0_cb_arg_t *config_cb_arg;
	ocs_fcfcoe_vf_resc_t *vf_resc = vf_resc_in;
	sli4_rsrc_desc_fcfcoe_v0_t *desc_p;
	sli4_req_common_set_profile_config_v0_t *req;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	uint8_t *mbxdata;
	int i;

	/* mbxdata holds the header of the command */
	mbxdata = ocs_malloc(hal->os, SLI4_BMBX_SIZE,
			     OCS_M_ZERO | OCS_M_NOWAIT);
	if (!mbxdata) {
		ocs_log_err(hal->os, "Failed to malloc mbox\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	/* config_cb_arg holds the data that will be passed to the callback on completion */
	config_cb_arg = ocs_malloc(hal->os, sizeof(*config_cb_arg),
				   OCS_M_ZERO | OCS_M_NOWAIT);
	if (!config_cb_arg) {
		ocs_log_err(hal->os, "Failed to malloc config_cb_arg\n");
		rc = OCS_HAL_RTN_NO_MEMORY;
		goto free_mbx;
	}

	config_cb_arg->cb = cb;
	config_cb_arg->cb_arg = cb_arg;
	config_cb_arg->mbxdata = mbxdata;
	if (!vf_resc) {
		vf_resc = ocs_malloc(hal->os, sizeof(*vf_resc) * num_vfs,
				     OCS_M_ZERO | OCS_M_NOWAIT);
		if (!vf_resc) {
			ocs_log_err(hal->os, "Failed to alloc vf_resc\n");
			rc = OCS_HAL_RTN_NO_MEMORY;
			goto free_cb_arg;
		}

		config_cb_arg->vf_resc = vf_resc;
	}

	ocs_hal_setup_vf_resc_defaults(hal, vf_resc, pci_func,
				       num_vfs, wwnn, wwpn_tbl);

	/**
 	* Allocate memory for the descriptors we're going to send.
 	* i.e. one for each PCI descriptor plus one ISAP descriptor.
 	*/
	if (ocs_dma_alloc(hal->os, &config_cb_arg->payload,
			  sizeof(sli4_req_common_set_profile_config_v0_t) +
			  (num_vfs * sizeof(sli4_rsrc_desc_fcfcoe_v0_t)), 4)) {
		ocs_log_err(hal->os, "Failed to allocate DMA buffer\n");
		rc = OCS_HAL_RTN_NO_MEMORY;
		goto free_vf_resc;
	}
	
	req = (sli4_req_common_set_profile_config_v0_t *)config_cb_arg->payload.virt;
	sli_cmd_common_set_profile_config_v0(&hal->sli, mbxdata, SLI4_BMBX_SIZE,
					     &config_cb_arg->payload, 0, num_vfs);

	/**
	 * Loop over all descriptors. Prepare FCFCOE descriptors.
	 */
	desc_p = (sli4_rsrc_desc_fcfcoe_v0_t *)req->desc;
	for (i = 0; i < num_vfs; i++) {
		ocs_hal_populate_fcfcoe_resc_desc(&vf_resc[i], desc_p);
		desc_p++;
	}

	config_cb_arg->num_vfs = num_vfs;
	rc = ocs_hal_command(hal, mbxdata, OCS_CMD_NOWAIT,
			     ocs_hal_cb_set_profile_config_v0, config_cb_arg);
	if (rc) {
		ocs_log_err(hal->os, "set_profile_config_v0 submission failed.\n");
		goto free_dma;
	}

	ocs_log_info(hal->os, "Set profile config (v0) for PCI Func=%d, num_vfs=%d.\n",
		     pci_func, num_vfs);

	return rc;

free_dma:
	ocs_dma_free(hal->os, &config_cb_arg->payload);
free_vf_resc:
	if (!vf_resc_in && vf_resc)
		ocs_free(hal->os, vf_resc, sizeof(*vf_resc) * num_vfs);
free_cb_arg:
	ocs_free(hal->os, config_cb_arg, sizeof(*config_cb_arg));
free_mbx:
	ocs_free(hal->os, mbxdata, SLI4_BMBX_SIZE);
	return rc;
}

/**
 * @brief Restarting domain in case of fabric re-login request
 *
 * @par Description
 *
 * @param hal Pointer to HAL object.
 *
 * @return Return 0 if successful else returns -1
 */
int
ocs_hal_p2p_lip_handle(ocs_hal_t *hal)
{
	uint32_t i;
	ocs_domain_t *d = NULL;

	if (hal == NULL) {
		ocs_log_err(NULL, "unable to access HAL object\n");
		return -1;
	}

	for (i = 0; i < SLI4_MAX_FCFI; i++) {
		d = hal->domains[i];
		if (d != NULL && (hal->callback.domain != NULL)) {
			hal->callback.domain(hal->args.domain, OCS_HAL_DOMAIN_LOST, d);
		}
	}

	ocs_hal_read_fcf(hal, SLI4_FCOE_FCF_TABLE_FIRST);
	return 0;
}

/* vim: set noexpandtab textwidth=120: */

/**
 * @page fc_hal_api_overview HAL APIs
 * - @ref devInitShutdown
 * - @ref domain
 * - @ref port
 * - @ref node
 * - @ref io
 * - @ref interrupt
 *
 * <div class="overview">
 * The Hardware Abstraction Layer (HAL) insulates the higher-level code from the SLI-4
 * message details, but the higher level code must still manage domains, ports,
 * IT nexuses, and IOs. The HAL API is designed to help the higher level manage
 * these objects.<br><br>
 *
 * The HAL uses function callbacks to notify the higher-level code of events
 * that are received from the chip. There are currently three types of
 * functions that may be registered:
 *
 * <ul><li>domain – This function is called whenever a domain event is generated
 * within the HAL. Examples include a new FCF is discovered, a connection
 * to a domain is disrupted, and allocation callbacks.</li>
 * <li>unsolicited – This function is called whenever new data is received in
 * the SLI-4 receive queue.</li>
 * <li>rnode – This function is called for remote node events, such as attach status
 * and  allocation callbacks.</li></ul>
 *
 * Upper layer functions may be registered by using the ocs_hal_callback() function.
 *
 * <img src="elx_fc_hal.jpg" alt="FC/FCoE HAL" title="FC/FCoE HAL" align="right"/>
 * <h2>FC/FCoE HAL API</h2>
 * The FC/FCoE HAL component builds upon the SLI-4 component to establish a flexible
 * interface for creating the necessary common objects and sending I/Os. It may be used
 * “as is” in customer implementations or it can serve as an example of typical interactions
 * between a driver and the SLI-4 hardware. The broad categories of functionality include:
 *
 * <ul><li>Setting-up and tearing-down of the HAL.</li>
 * <li>Allocating and using the common objects (SLI Port, domain, remote node).</li>
 * <li>Sending and receiving I/Os.</li></ul>
 *
 * <h3>HAL Setup</h3>
 * To set up the HAL:
 *
 * <ol>
 * <li>Set up the HAL object using ocs_hal_setup().<br>
 * This step performs a basic configuration of the SLI-4 component and the HAL to
 * enable querying the hardware for its capabilities. At this stage, the HAL is not
 * capable of general operations (such as, receiving events or sending I/Os).</li><br><br>
 * <li>Configure the HAL according to the driver requirements.<br>
 * The HAL provides functions to discover hardware capabilities (ocs_hal_get()), as
 * well as configures the amount of resources required (ocs_hal_set()). The driver
 * must also register callback functions (ocs_hal_callback()) to receive notification of
 * various asynchronous events.<br><br>
 * @b Note: Once configured, the driver must initialize the HAL (ocs_hal_init()). This
 * step creates the underlying queues, commits resources to the hardware, and
 * prepares the hardware for operation. While the hardware is operational, the
 * port is not online, and cannot send or receive data.</li><br><br>
 * <br><br>
 * <li>Finally, the driver can bring the port online (ocs_hal_port_control()).<br>
 * When the link comes up, the HAL determines if a domain is present and notifies the
 * driver using the domain callback function. This is the starting point of the driver's
 * interaction with the common objects.<br><br>
 * @b Note: For FCoE, there may be more than one domain available and, therefore,
 * more than one callback.</li>
 * </ol>
 *
 * <h3>Allocating and Using Common Objects</h3>
 * Common objects provide a mechanism through which the various OneCore Storage
 * driver components share and track information. These data structures are primarily
 * used to track SLI component information but can be extended by other components, if
 * needed. The main objects are:
 *
 * <ul><li>DMA – the ocs_dma_t object describes a memory region suitable for direct
 * memory access (DMA) transactions.</li>
 * <li>SCSI domain – the ocs_domain_t object represents the SCSI domain, including
 * any infrastructure devices such as FC switches and FC forwarders. The domain
 * object contains both an FCFI and a VFI.</li>
 * <li>SLI Port (sport) – the ocs_sli_port_t object represents the connection between
 * the driver and the SCSI domain. The SLI Port object contains a VPI.</li>
 * <li>Remote node – the ocs_remote_node_t represents a connection between the SLI
 * Port and another device in the SCSI domain. The node object contains an RPI.</li></ul>
 *
 * Before the driver can send I/Os, it must allocate the SCSI domain, SLI Port, and remote
 * node common objects and establish the connections between them. The goal is to
 * connect the driver to the SCSI domain to exchange I/Os with other devices. These
 * common object connections are shown in the following figure, FC Driver Common Objects:
 * <img src="elx_fc_common_objects.jpg"
 * alt="FC Driver Common Objects" title="FC Driver Common Objects" align="center"/>
 *
 * The first step is to create a connection to the domain by allocating an SLI Port object.
 * The SLI Port object represents a particular FC ID and must be initialized with one. With
 * the SLI Port object, the driver can discover the available SCSI domain(s). On identifying
 * a domain, the driver allocates a domain object and attaches to it using the previous SLI
 * port object.<br><br>
 *
 * @b Note: In some cases, the driver may need to negotiate service parameters (that is,
 * FLOGI) with the domain before attaching.<br><br>
 *
 * Once attached to the domain, the driver can discover and attach to other devices
 * (remote nodes). The exact discovery method depends on the driver, but it typically
 * includes using a position map, querying the fabric name server, or an out-of-band
 * method. In most cases, it is necessary to log in with devices before performing I/Os.
 * Prior to sending login-related ELS commands (ocs_hal_srrs_send()), the driver must
 * allocate a remote node object (ocs_hal_node_alloc()). If the login negotiation is
 * successful, the driver must attach the nodes (ocs_hal_node_attach()) to the SLI Port
 * before exchanging FCP I/O.<br><br>
 *
 * @b Note: The HAL manages both the well known fabric address and the name server as
 * nodes in the domain. Therefore, the driver must allocate node objects prior to
 * communicating with either of these entities.
 *
 * <h3>Sending and Receiving I/Os</h3>
 * The HAL provides separate interfaces for sending BLS/ ELS/ FC-CT and FCP, but the
 * commands are conceptually similar. Since the commands complete asynchronously,
 * the caller must provide a HAL I/O object that maintains the I/O state, as well as
 * provide a callback function. The driver may use the same callback function for all I/O
 * operations, but each operation must use a unique HAL I/O object. In the SLI-4
 * architecture, there is a direct association between the HAL I/O object and the SGL used
 * to describe the data. Therefore, a driver typically performs the following operations:
 *
 * <ul><li>Allocates a HAL I/O object (ocs_hal_io_alloc()).</li>
 * <li>Formats the SGL, specifying both the HAL I/O object and the SGL.
 * (ocs_hal_io_init_sges() and ocs_hal_io_add_sge()).</li>
 * <li>Sends the HAL I/O (ocs_hal_io_send()).</li></ul>
 *
 * <h3>HAL Tear Down</h3>
 * To tear-down the HAL:
 *
 * <ol><li>Take the port offline (ocs_hal_port_control()) to prevent receiving further
 * data andevents.</li>
 * <li>Destroy the HAL object (ocs_hal_teardown()).</li>
 * <li>Free any memory used by the HAL, such as buffers for unsolicited data.</li></ol>
 * <br>
 * </div><!-- overview -->
 *
 */

