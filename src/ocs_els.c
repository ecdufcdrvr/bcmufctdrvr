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
 * Functions to build and send ELS/CT/BLS commands and responses.
 */

/*!
@defgroup els_api ELS/BLS/CT Command and Response Functions
*/

#include "ocs.h"
#include "ocs_els.h"
#include "ocs_scsi_fc.h"
#include "ocs_device.h"
#include "ocs_fabric.h"

#define ELS_IOFMT "[i:%04x t:%04x h:%04x]"
#define ELS_IOFMT_ARGS(els) els->init_task_tag, els->tgt_task_tag, els->hw_tag

#define node_els_trace()  \
	do { \
		if (OCS_LOG_ENABLE_ELS_TRACE(ocs)) \
			ocs_log_info(ocs, "[%s]\n", node->display_name); \
	} while (0)

#define els_io_printf(els, fmt, ...) \
	ocs_log_debug(els->node->ocs, "[%s]" ELS_IOFMT " %-8s " fmt, els->node->display_name, ELS_IOFMT_ARGS(els), els->display_name, ##__VA_ARGS__);

static int32_t ocs_els_send(ocs_io_t *els, uint32_t reqlen, uint32_t timeout_sec, ocs_hal_srrs_cb_t cb);
static int32_t ocs_els_send_rsp(ocs_io_t *els, uint32_t rsplen);
static int32_t ocs_els_acc_cb(ocs_hal_io_t *hio, ocs_remote_node_t *rnode, uint32_t length, int32_t status, uint32_t ext_status, void *arg);
static ocs_io_t *ocs_bls_send_acc(ocs_io_t *io, uint32_t s_id, uint16_t ox_id, uint16_t rx_id);
static ocs_io_t *ocs_bls_send_rjt(ocs_io_t *io, uint32_t s_id, uint16_t ox_id, uint16_t rx_id);
static int32_t ocs_bls_send_acc_cb(ocs_hal_io_t *hio, ocs_remote_node_t *rnode, uint32_t length,
	int32_t status, uint32_t ext_status, void *app);
static int32_t ocs_bls_send_rjt_cb(ocs_hal_io_t *hio, ocs_remote_node_t *rnode, uint32_t length,
	int32_t status, uint32_t ext_status, void *app);
static void ocs_io_transition(ocs_io_t *els, ocs_sm_function_t state, void *data);
static int32_t ocs_els_abort_io(ocs_io_t *els, bool send_abts);
static void _ocs_els_io_free(void *arg);
static void ocs_els_delay_timer_cb(void *arg);

/* RHBA attribute jump table */
int (*ocs_fdmi_hba_action[])(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad) = {
	/* Action routine			Mask bit	Attribute type		*/
	ocs_fdmi_hba_attr_wwnn,			/* bit0		RHBA_NODENAME		*/
	ocs_fdmi_hba_attr_manufacturer,		/* bit1		RHBA_MANUFACTURER	*/
	ocs_fdmi_hba_attr_sn,			/* bit2		RHBA_SERIAL_NUMBER	*/
	ocs_fdmi_hba_attr_model,		/* bit3		RHBA_MODEL		*/
	ocs_fdmi_hba_attr_description,		/* bit4		RHBA_MODEL_DESCRIPTION	*/
	ocs_fdmi_hba_attr_drvr_ver,		/* bit5		RHBA_DRIVER_VERSION	*/
	ocs_fdmi_hba_attr_rom_ver,		/* bit6		RHBA_OPTION_ROM_VERSION	*/
	ocs_fdmi_hba_attr_fmw_ver,		/* bit7		RHBA_FIRMWARE_VERSION	*/
	ocs_fdmi_hba_attr_os_ver,		/* bit8		RHBA_OS_NAME_VERSION	*/
	ocs_fdmi_hba_attr_ct_len,		/* bit9		RHBA_MAX_CT_PAYLOAD_LEN	*/
	ocs_fdmi_hba_attr_symbolic_name,	/* bit10	RHBA_SYM_NODENAME	*/
	ocs_fdmi_hba_attr_vendor_info,		/* bit11	RHBA_VENDOR_INFO	*/
	ocs_fdmi_hba_attr_num_ports,		/* bit12	RHBA_NUM_PORTS		*/
	ocs_fdmi_hba_attr_fabric_wwnn,		/* bit13	RHBA_FABRIC_WWNN	*/
	ocs_fdmi_hba_attr_bios_ver,		/* bit14	RHBA_BIOS_VERSION	*/
	ocs_fdmi_hba_attr_bios_state,		/* bit15	RHBA_BIOS_STATE		*/
	ocs_fdmi_hba_attr_vendor_id,		/* bit16	RHBA_VENDOR_ID		*/
	NULL,
};

/* RPA / RPRT attribute jump table */
int (*ocs_fdmi_port_action[])(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad) = {
	/* Action routine			Mask bit	Attribute type		*/
	ocs_fdmi_port_attr_fc4type,		/* bit0		RPRT_SUPPORT_FC4_TYPES	*/
	ocs_fdmi_port_attr_support_speed,	/* bit1		RPRT_SUPPORTED_SPEED	*/
	ocs_fdmi_port_attr_speed,		/* bit2		RPRT_PORT_SPEED		*/
	ocs_fdmi_port_attr_max_frame,		/* bit3		RPRT_MAX_FRAME_SIZE	*/
	ocs_fdmi_port_attr_os_devname,		/* bit4		RPRT_OS_DEVICE_NAME	*/
	ocs_fdmi_port_attr_host_name,		/* bit5		RPRT_HOST_NAME		*/
	ocs_fdmi_port_attr_wwnn,		/* bit6		RPRT_NODENAME		*/
	ocs_fdmi_port_attr_wwpn,		/* bit7		RPRT_PORTNAME		*/
	ocs_fdmi_port_attr_symbolic_name,	/* bit8		RPRT_SYM_PORTNAME	*/
	ocs_fdmi_port_attr_port_type,		/* bit9		RPRT_PORT_TYPE		*/
	ocs_fdmi_port_attr_class,		/* bit10	RPRT_SUPPORTED_CLASS	*/
	ocs_fdmi_port_attr_fabric_wwpn,		/* bit11	RPRT_FABRICNAME		*/
	ocs_fdmi_port_attr_active_fc4type,	/* bit12	RPRT_ACTIVE_FC4_TYPES	*/
	ocs_fdmi_port_attr_port_state,		/* bit13	RPRT_PORT_STATE		*/
	ocs_fdmi_port_attr_nportid,		/* bit14	RPRT_PORT_ID		*/
	NULL,
};

#define OCS_ELS_RSP_LEN		1024
#define OCS_ELS_FDMI_REG_LEN	2048
#define OCS_ELS_FDMI_RSP_LEN	4096

#define OCS_ELS_GID_FT_RSP_LEN	8096 /* Enough for 2K remote target nodes */
#define OCS_ELS_GID_PT_RSP_LEN	8096 /* Enough for 2K remote target nodes */

#define MAX_FDMI_ATTRIBUTE_SIZE	256

/**
 * @ingroup els_api
 * @brief ELS state machine transition wrapper.
 *
 * <h3 class="desc">Description</h3>
 * This function is the transition wrapper for the ELS state machine. It grabs
 * the node lock prior to making the transition to protect
 * against multiple threads accessing a particular ELS. For example,
 * one thread transitioning from __els_init to
 * __ocs_els_wait_resp and another thread (tasklet) handling the
 * completion of that ELS request.
 *
 * @param els Pointer to the IO context.
 * @param state State to transition to.
 * @param data Data to pass in with the transition.
 *
 * @return None.
 */
static void
ocs_io_transition(ocs_io_t *els, ocs_sm_function_t state, void *data)
{
	/* protect ELS events with node lock */
	ocs_node_t *node = els->node;
	ocs_node_lock(node);
		ocs_sm_transition(&els->els_info->els_sm, state, data);
	ocs_node_unlock(node);
}

/**
 * @ingroup els_api
 * @brief ELS state machine post event wrapper.
 *
 * <h3 class="desc">Description</h3>
 * Post an event wrapper for the ELS state machine. This function grabs
 * the node lock prior to posting the event.
 *
 * @param els Pointer to the IO context.
 * @param evt Event to process.
 * @param data Data to pass in with the transition.
 *
 * @return None.
 */
void
ocs_els_post_event(ocs_io_t *els, ocs_sm_event_t evt, void *data)
{
	/* protect ELS events with node lock */
	ocs_node_t *node = els->node;
	ocs_node_lock(node);
		els->els_info->els_evtdepth ++;
		ocs_sm_post_event(&els->els_info->els_sm, evt, data);
		els->els_info->els_evtdepth --;
	ocs_node_unlock(node);
	if (els->els_info->els_evtdepth == 0 && els->els_info->els_req_free) {
		ocs_els_io_free(els);
	}
}

/**
 * @ingroup els_api
 * @brief Allocate an IO structure for an ELS IO context.
 *
 * <h3 class="desc">Description</h3>
 * Allocate an IO for an ELS context.  Uses OCS_ELS_RSP_LEN as response size.
 *
 * @param node node to associate ELS IO with
 * @param reqlen Length of ELS request
 * @param role Role of ELS (originator/responder)
 *
 * @return pointer to IO structure allocated
 */

ocs_io_t *
ocs_els_io_alloc(ocs_node_t *node, uint32_t reqlen, ocs_els_role_e role)
{
	return ocs_els_io_alloc_size(node, reqlen, OCS_ELS_RSP_LEN, role);
}

/**
 * @ingroup els_api
 * @brief Allocate an IO structure for an ELS IO context.
 *
 * <h3 class="desc">Description</h3>
 * Allocate an IO for an ELS context, allowing the caller to specify the size of the response.
 *
 * @param node node to associate ELS IO with
 * @param reqlen Length of ELS request
 * @param rsplen Length of ELS response
 * @param role Role of ELS (originator/responder)
 *
 * @return pointer to IO structure allocated
 */

ocs_io_t *
ocs_els_io_alloc_size(ocs_node_t *node, uint32_t reqlen, uint32_t rsplen, ocs_els_role_e role)
{

	int32_t rc;
	ocs_t *ocs;
	ocs_xport_t *xport;
	ocs_io_t *els;
	ocs_dma_t els_req;
	ocs_dma_t els_rsp;
	ocs_assert(node, NULL);
	ocs_assert(node->ocs, NULL);
	ocs = node->ocs;
	ocs_assert(ocs->xport, NULL);
	xport = ocs->xport;

	/* Now allocate DMA for request and response */
	rc = ocs_dma_alloc(ocs, &els_req, reqlen, OCS_MIN_DMA_ALIGNMENT);
	if (rc == 0) {
		rc = ocs_dma_alloc(ocs, &els_rsp, rsplen, OCS_MIN_DMA_ALIGNMENT);
		if (rc != 0) {
			ocs_log_err(ocs, "ocs_dma_alloc for els_rsp failed\n");
			ocs_dma_free(ocs, &els_req);
			return NULL;
		}
	} else {
		ocs_log_err(ocs, "ocs_dma_alloc for els_req failed\n");
		return NULL;
	}

	ocs_lock(&node->active_ios_lock);
		if (!node->io_alloc_enabled) {
			ocs_log_debug(ocs, "called with io_alloc_enabled = FALSE\n");
			ocs_unlock(&node->active_ios_lock);
			goto els_io_alloc_err;
		}

		els = ocs_io_alloc(ocs, OCS_IO_POOL_ELS);
		if (els == NULL) {
			ocs_atomic_add_return(&xport->io_alloc_failed_count, 1);
			ocs_unlock(&node->active_ios_lock);
			goto els_io_alloc_err;
		}

		/* initialize refcount */
		ocs_ref_init(&els->ref, _ocs_els_io_free, els);

		switch (role) {
		case OCS_ELS_ROLE_ORIGINATOR:
			els->cmd_ini = TRUE;
			els->cmd_tgt = FALSE;
			break;
		case OCS_ELS_ROLE_RESPONDER:
			els->cmd_ini = FALSE;
			els->cmd_tgt = TRUE;
			break;
		}

		/* IO should not have an associated HAL IO yet.  Assigned below. */
		if (els->hio != NULL) {
			ocs_log_err(ocs, "HIO is not null\n");
			ocs_io_free(ocs, els);
			ocs_unlock(&node->active_ios_lock);
			goto els_io_alloc_err;
		}

		/* populate generic io fields */
		els->ocs = ocs;
		els->node = node;

		/* set type and ELS-specific fields */
		els->io_type = OCS_IO_TYPE_ELS;
		els->display_name = "pending";

		if (els != NULL) {
			ocs_memset(&els->els_info->els_sm, 0, sizeof(els->els_info->els_sm));
			els->els_info->els_sm.app = els;

			/* initialize fields */
			els->els_info->els_retries_remaining = OCS_FC_ELS_DEFAULT_RETRIES;
			els->els_info->els_evtdepth = 0;
			els->els_info->els_pend = 0;
			els->els_info->els_active = 0;
			els->els_info->els_rjt = 0;
			els->els_info->els_req = els_req;
			els->els_info->els_rsp = els_rsp;

			/* add els structure to ELS IO list */
			ocs_list_add_tail(&node->els_io_pend_list, els);
			els->els_info->els_pend = 1;
		}
	ocs_unlock(&node->active_ios_lock);
	return els;

els_io_alloc_err:
	ocs_dma_free(ocs, &els_req);
	ocs_dma_free(ocs, &els_rsp);
	return NULL;
}

void
ocs_els_io_send(ocs_io_t *els)
{
	ocs_io_transition(els, __ocs_els_init, NULL);
}

/**
 * @ingroup els_api
 * @brief Free IO structure for an ELS IO context.
 *
 * <h3 class="desc">Description</h3> Free IO for an ELS
 * IO context
 *
 * @param els ELS IO structure for which IO is allocated
 *
 * @return None
 */

void
ocs_els_io_free(ocs_io_t *els)
{
	ocs_ref_put(&els->ref);
}

/**
 * @ingroup els_api
 * @brief Free IO structure for an ELS IO context.
 *
 * <h3 class="desc">Description</h3> Free IO for an ELS
 * IO context
 *
 * @param arg ELS IO structure for which IO is allocated
 *
 * @return None
 */

static void
_ocs_els_io_free(void *arg)
{
	ocs_io_t *els = (ocs_io_t *)arg;
	ocs_t *ocs;
	ocs_node_t *node;
	int send_empty_event = FALSE;

	ocs_assert(els);
	ocs_assert(els->node);
	ocs_assert(els->node->ocs);
	ocs = els->node->ocs;

	node = els->node;
	ocs = node->ocs;

	ocs_lock(&node->active_ios_lock);
		if (els->els_info->els_active) {
			/* if active, remove from active list and check empty */
			ocs_list_remove(&node->els_io_active_list, els);
			/* Send list empty event if the IO allocator is disabled, and the list is empty
			 * If node->io_alloc_enabled was not checked, the event would be posted continually
			 */
			send_empty_event = (!node->io_alloc_enabled) && ocs_list_empty(&node->els_io_active_list);
			els->els_info->els_active = 0;
		} else if (els->els_info->els_pend) {
			/* if pending, remove from pending list; node shutdown isn't
			 * gated off the pending list (only the active list), so no
			 * need to check if pending list is empty
			 */
			ocs_list_remove(&node->els_io_pend_list, els);
			els->els_info->els_pend = 0;
		} else {
			ocs_log_err(ocs, "neither els->els_info->els_pend nor els->active set\n");
			ocs_unlock(&node->active_ios_lock);
			return;
		}

	ocs_unlock(&node->active_ios_lock);

	/* free ELS request and response buffers */
	ocs_dma_free(ocs, &els->els_info->els_rsp);
	ocs_dma_free(ocs, &els->els_info->els_req);

	ocs_io_free(ocs, els);

	if (send_empty_event) {
		ocs_node_post_event(node, OCS_EVT_ALL_CHILD_NODES_FREE, NULL);
	}

	ocs_scsi_check_pending(ocs);
}

/**
 * @ingroup els_api
 * @brief Make ELS IO active
 *
 * @param els Pointer to the IO context to make active.
 *
 * @return Returns 0 on success; or a negative error code value on failure.
 */

static void
ocs_els_make_active(ocs_io_t *els)
{
	ocs_node_t *node = els->node;

	/* move ELS from pending list to active list */
	ocs_lock(&node->active_ios_lock);
		if (els->els_info->els_pend) {
			if (els->els_info->els_active) {
				ocs_log_err(node->ocs, "both els->els_info->els_pend and els->active set\n");
				ocs_unlock(&node->active_ios_lock);
				return;
			} else {

				/* remove from pending list */
				ocs_list_remove(&node->els_io_pend_list, els);
				els->els_info->els_pend = 0;

				/* add els structure to ELS IO list */
				ocs_list_add_tail(&node->els_io_active_list, els);
				els->els_info->els_active = 1;
			}
		} else {
			/* must be retrying; make sure it's already active */
			if (!els->els_info->els_active)
				ocs_log_err(node->ocs, "neither els->els_info->els_pend nor els->active set\n");
		}
	ocs_unlock(&node->active_ios_lock);
}

/**
 * @ingroup els_api
 * @brief Send the ELS command.
 *
 * <h3 class="desc">Description</h3>
 * The command, given by the \c els IO context, is sent to the node that the IO was
 * configured with, using ocs_hal_srrs_send(). Upon completion,
 * the \c cb callback is invoked,
 * with the application-specific argument set to the \c els IO context.
 *
 * @param els Pointer to the IO context.
 * @param reqlen Byte count in the payload to send.
 * @param timeout_sec Command timeout, in seconds (0 -> 2*R_A_TOV).
 * @param cb Completion callback.
 *
 * @return Returns 0 on success; or a negative error code value on failure.
 */

static int32_t
ocs_els_send(ocs_io_t *els, uint32_t reqlen, uint32_t timeout_sec, ocs_hal_srrs_cb_t cb)
{
	ocs_node_t *node = els->node;

	/* update ELS request counter */
	node->els_req_cnt++;

	/* move ELS from pending list to active list */
	ocs_els_make_active(els);

	els->wire_len = reqlen;
	return ocs_scsi_io_dispatch(els, cb);
}

/**
 * @ingroup els_api
 * @brief Send the ELS response.
 *
 * <h3 class="desc">Description</h3>
 * The ELS response, given by the \c els IO context, is sent to the node
 * that the IO was configured with, using ocs_hal_srrs_send().
 *
 * @param els Pointer to the IO context.
 * @param rsplen Byte count in the payload to send.
 *
 * @return Returns 0 on success; or a negative error value on failure.
 */

static int32_t
ocs_els_send_rsp(ocs_io_t *els, uint32_t rsplen)
{
	ocs_node_t *node = els->node;

	/* increment ELS completion counter */
	node->els_cmpl_cnt++;

	/* move ELS from pending list to active list */
	ocs_els_make_active(els);

	els->wire_len = rsplen;
	return ocs_scsi_io_dispatch(els, ocs_els_acc_cb);
}

/**
 * @ingroup els_api
 * @brief Handle ELS IO request completions.
 *
 * <h3 class="desc">Description</h3>
 * This callback is used for several ELS send operations.
 *
 * @param hio Pointer to the HAL IO context that completed.
 * @param rnode Pointer to the remote node.
 * @param length Length of the returned payload data.
 * @param status Status of the completion.
 * @param ext_status Extended status of the completion.
 * @param arg Application-specific argument (generally a pointer to the ELS IO context).
 *
 * @return Returns 0 on success; or a negative error value on failure.
 */

static int32_t
ocs_els_req_cb(ocs_hal_io_t *hio, ocs_remote_node_t *rnode, uint32_t length, int32_t status, uint32_t ext_status, void *arg)
{
	ocs_io_t *els;
	ocs_node_t *node;
	ocs_t *ocs;
	ocs_node_cb_t cbdata;
	ocs_sport_t *sport;

	ocs_assert(arg, -1);
	els = arg;

	ocs_assert(els->node, -1);
	node = els->node;

	ocs_assert(node->ocs, -1);
	ocs = node->ocs;

	ocs_assert(node->sport, -1);
	sport = node->sport;

	ocs_assert(els->hio, -1);
	ocs_assert(hio == els->hio, -1);

	if (status)
		els_io_printf(els, "status x%x ext x%x\n", status, ext_status);

	/* Set the response len element of els->rsp */
	els->els_info->els_rsp.len = length;

	cbdata.status = status;
	cbdata.ext_status = ext_status;
	cbdata.header = NULL;
	cbdata.els = els;

	/* Take a refcount here so that this sport will not get freed while processing the ELS WQE completion */
	if (!ocs_ref_get_unless_zero(&sport->ref)) {
		ocs_log_err(ocs, "Sport is not active\n");
		return -1;
	}

	/**
	 * For ELS WQE completions, we seem to acquire the sport and node locks in a different order
	 * when compared to other code paths. So, acquire the sport lock here to avoid the deadlock.
	 */
	ocs_sport_lock(sport);

	/**
	 * FW returns the number of bytes received on the link in the WCQE, not the amount
	 * placed in the buffer. So, use this info to check if there was an overrun.
	 */
	if (length > els->els_info->els_rsp.size) {
		ocs_log_warn(ocs, "ELS response returned len=%d > buflen=%zu\n", length, els->els_info->els_rsp.size);
		ocs_els_post_event(els, OCS_EVT_SRRS_ELS_REQ_FAIL, &cbdata);
		goto exit_els_req_cb;
	}

	/* Post event to ELS IO object */
	switch (status) {
	case SLI4_FC_WCQE_STATUS_SUCCESS:
		ocs_els_post_event(els, OCS_EVT_SRRS_ELS_REQ_OK, &cbdata);
		break;

	case SLI4_FC_WCQE_STATUS_LS_RJT:
		ocs_els_post_event(els, OCS_EVT_SRRS_ELS_REQ_RJT, &cbdata);
		break;

	case SLI4_FC_WCQE_STATUS_LOCAL_REJECT:
		switch (ext_status) {
		case SLI4_FC_LOCAL_REJECT_SEQUENCE_TIMEOUT:
			ocs_els_post_event(els, OCS_EVT_ELS_REQ_TIMEOUT, &cbdata);
			break;
		case SLI4_FC_LOCAL_REJECT_ABORT_REQUESTED:
			ocs_els_post_event(els, OCS_EVT_ELS_REQ_ABORTED, &cbdata);
			break;
		default:
			ocs_els_post_event(els, OCS_EVT_SRRS_ELS_REQ_FAIL, &cbdata);
			break;
		}
		break;

	default:
		ocs_log_warn(ocs, "els req complete: failed status x%x, ext_status, x%x\n", status, ext_status);
		ocs_els_post_event(els, OCS_EVT_SRRS_ELS_REQ_FAIL, &cbdata);
		break;
	}

exit_els_req_cb:
	/* Release the sport lock and put the refcount */
	ocs_sport_unlock(sport);
	ocs_ref_put(&sport->ref); /* ocs_ref_get(): same function */
	return 0;
}

/**
 * @ingroup els_api
 * @brief Handle ELS IO accept/response completions.
 *
 * <h3 class="desc">Description</h3>
 * This callback is used for several ELS send operations.
 *
 * @param hio Pointer to the HAL IO context that completed.
 * @param rnode Pointer to the remote node.
 * @param length Length of the returned payload data.
 * @param status Status of the completion.
 * @param ext_status Extended status of the completion.
 * @param arg Application-specific argument (generally a pointer to the ELS IO context).
 *
 * @return Returns 0 on success; or a negative error value on failure.
 */

static int32_t
ocs_els_acc_cb(ocs_hal_io_t *hio, ocs_remote_node_t *rnode, uint32_t length, int32_t status, uint32_t ext_status, void *arg)
{
	ocs_io_t *els;
	ocs_node_t *node;
	ocs_t *ocs;
	ocs_node_cb_t cbdata;
	ocs_io_t *io;

	ocs_assert(arg, -1);
	io = arg;
	els = io;
	ocs_assert(els, -1);
	ocs_assert(els->node, -1);
	node = els->node;
	ocs_assert(node->ocs, -1);
	ocs = node->ocs;

	ocs_assert(io->hio, -1);
	ocs_assert(hio == io->hio, -1);

	cbdata.status = status;
	cbdata.ext_status = ext_status;
	cbdata.header = NULL;
	cbdata.els = els;

	/* Post node event */
	switch (status) {
	case SLI4_FC_WCQE_STATUS_SUCCESS:
		ocs_node_post_event(node, OCS_EVT_SRRS_ELS_CMPL_OK, &cbdata);
		break;

	default:	// Other error
		ocs_log_warn(ocs, "[%s] %-8s failed status x%x, ext_status x%x\n",
			node->display_name, els->display_name, status, ext_status);
		ocs_log_warn(ocs, "els acc complete: failed status x%x, ext_status, x%x\n", status, ext_status);
		ocs_node_post_event(node, OCS_EVT_SRRS_ELS_CMPL_FAIL, &cbdata);
		break;
	}

	/* If this IO has a callback, invoke it */
	if (els->els_info->els_callback) {
		(*els->els_info->els_callback)(node, &cbdata, els->els_info->els_callback_arg);
	}

	ocs_els_io_free(els);

	return 0;
}

/**
 * @ingroup els_api
 * @brief Format and send a PLOGI ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Construct a PLOGI payload using the domain SLI port service parameters,
 * and send to the \c node.
 *
 * @param node Node to which the PLOGI is sent.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_plogi(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	void (*cb)(ocs_node_t *node, ocs_node_cb_t *cbdata, void *arg), void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fc_plogi_payload_t *plogi;
	uint32_t suppress_rsp = false;

	node_els_trace();

	els = ocs_els_io_alloc(node, sizeof(*plogi), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "plogi";

		/* Build PLOGI request */
		plogi = els->els_info->els_req.virt;

		ocs_memcpy(plogi, node->sport->service_params, sizeof(*plogi));

		plogi->command_code = FC_ELS_CMD_PLOGI;
		plogi->resv1 = 0;

		/* Zero-out the BB_SC_N field (word 1, bits 15:12) */
		plogi->common_service_parameters[1] &= ocs_htobe32(FC_PLOGI_CSP_W1_BBSCN_CLEAR_MASK);

		/*
		 * If FW is capable of suppress_rsp feature, then indicate the same to
		 * the remote node using the vendor version field in service parameters.
		 */
		ocs_hal_get(&ocs->hal, OCS_HAL_SUPPRESS_RSP_CAPABLE, &suppress_rsp);
		if (suppress_rsp) {
			/* Set valid vendor version level bit */
			plogi->common_service_parameters[1] |= ocs_htobe32(FC_PLOGI_CSP_W1_VALID_VENDOR_VER_LEVEL);
			plogi->vendor_version_level[0] = ocs_htobe32(OCS_FC_PLOGI_VENDOR_VERSION_EMLX_ID);
			plogi->vendor_version_level[1] = ocs_htobe32(suppress_rsp);
		} else {
			plogi->common_service_parameters[1] &= ocs_htobe32(~FC_PLOGI_CSP_W1_VALID_VENDOR_VER_LEVEL);
			plogi->vendor_version_level[0] = 0;
			plogi->vendor_version_level[1] = 0;
			plogi->vendor_version_level[2] = 0;
			plogi->vendor_version_level[3] = 0;
		}

		ocs_display_sparams(node->display_name, "plogi send req", 0, NULL, plogi->common_service_parameters);

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;

		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @ingroup els_api
 * @brief Format and send a FLOGI ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Construct an FLOGI payload, and send to the \c node.
 *
 * @param node Node to which the FLOGI is sent.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_flogi(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs;
	fc_plogi_payload_t *flogi;

	ocs_assert(node, NULL);
	ocs_assert(node->ocs, NULL);
	ocs_assert(node->sport, NULL);
	ocs = node->ocs;

	node_els_trace();

	els = ocs_els_io_alloc(node, sizeof(*flogi), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "flogi";

		/* Build FLOGI request */
		flogi = els->els_info->els_req.virt;

		ocs_memcpy(flogi, node->sport->service_params, sizeof(*flogi));
		flogi->command_code = FC_ELS_CMD_FLOGI;
		flogi->resv1 = 0;
		/* We seem to have a FW bug wherein READ_SPARM64 wrongly exports
		 * Class2 support eventhough the device cannot support class2
		 * ELS frames. Till the time this issue is resolved zero out
		 * class 2 service_params
		 */
		flogi->class2_service_parameters[0] = 0;
		flogi->class2_service_parameters[1] = 0;
		flogi->class2_service_parameters[2] = 0;
		flogi->class2_service_parameters[3] = 0;


		/* Zero-out the BB_SC_N field (word 1, bits 15:12) */
		flogi->common_service_parameters[1] &= ocs_htobe32(FC_PLOGI_CSP_W1_BBSCN_CLEAR_MASK);

		/* BBCR support */
		if (ocs_bbcr_enabled(ocs)) {
			/* Set the BB_SC_N field to 'bbscn_def' */
			flogi->common_service_parameters[1] |= ocs_htobe32(ocs->hal.sli.config.bbscn_def << 12);
		}

		/* Priority tagging support */
		flogi->common_service_parameters[1] |= ocs_htobe32(1U << 23);

		/* Auth support */
		if (ocs_node_auth_enabled(node))
			flogi->common_service_parameters[1] |=
							ocs_htobe32(FC_FLOGI_CSP_W1_FCSP);

		ocs_display_sparams(node->display_name, "flogi send req", 0, NULL, flogi->common_service_parameters);

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @ingroup els_api
 * @brief Format and send a FDISC ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Construct an FDISC payload, and send to the \c node.
 *
 * @param node Node to which the FDISC is sent.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_fdisc(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs;
	fc_plogi_payload_t *fdisc;

	ocs_assert(node, NULL);
	ocs_assert(node->ocs, NULL);
	ocs = node->ocs;

	node_els_trace();

	els = ocs_els_io_alloc(node, sizeof(*fdisc), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "fdisc";

		/* Build FDISC request */
		fdisc = els->els_info->els_req.virt;

		ocs_memcpy(fdisc, node->sport->service_params, sizeof(*fdisc));
		fdisc->command_code = FC_ELS_CMD_FDISC;
		fdisc->resv1 = 0;

		/* Zero-out the BB_SC_N field (word 1, bits 15:12) */
		fdisc->common_service_parameters[1] &= ocs_htobe32(FC_PLOGI_CSP_W1_BBSCN_CLEAR_MASK);

		/* Clear VVL valid and VVL values */
		fdisc->common_service_parameters[1] &= ocs_htobe32(~FC_PLOGI_CSP_W1_VALID_VENDOR_VER_LEVEL);
		fdisc->vendor_version_level[0] = 0;
		fdisc->vendor_version_level[1] = 0;
		fdisc->vendor_version_level[2] = 0;
		fdisc->vendor_version_level[3] = 0;

		ocs_display_sparams(node->display_name, "fdisc send req", 0, NULL, fdisc->common_service_parameters);

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @ingroup els_api
 * @brief Send a PRLI ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Construct a PRLI ELS command, and send to the \c node.
 *
 * @param node Node to which the PRLI is sent.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_prli(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg, uint8_t fc_type)
{
	ocs_t *ocs = node->ocs;
	ocs_io_t *els;
	fc_prli_payload_t *prli;
	uint16_t payload_len = 0;
	uint16_t page_len = 0;
	bool enable_ini, enable_tgt;

	node_els_trace();

	if (fc_type == FC_TYPE_FCP) {
		enable_ini = ocs_ini_scsi_enabled(ocs);
		enable_tgt = ocs_tgt_scsi_enabled(ocs);

		payload_len = sizeof(fc_prli_payload_t);
		page_len = FC_PRLI_ACC_FCP_PAGE_LENGTH;
	} else if (fc_type == FC_TYPE_NVME) {
		enable_ini = ocs_ini_nvme_enabled(ocs);
		enable_tgt = ocs_tgt_nvme_enabled(ocs);

		payload_len = sizeof(fc_nvme_prli_payload_t);
		page_len = FC_PRLI_ACC_NVME_PAGE_LENGTH;
	} else {
		ocs_abort();
	}

	if (!enable_ini && !enable_tgt) {
		ocs_log_err(ocs, "Node is not capable of either initiator or target\n");
		return NULL;
	}

	els = ocs_els_io_alloc(node, payload_len, OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "prli";

		/* Build PRLI request */
		prli = els->els_info->els_req.virt;

		ocs_memset(prli, 0, payload_len);

		prli->command_code = FC_ELS_CMD_PRLI;
		prli->page_length = page_len;
		prli->payload_length = ocs_htobe16(payload_len);
		prli->type = fc_type;
		prli->type_ext = 0;
		prli->flags = ocs_htobe16(FC_PRLI_ESTABLISH_IMAGE_PAIR);
		prli->service_params = ocs_htobe16(FC_PRLI_READ_XRDY_DISABLED |
			(enable_ini ? FC_PRLI_INITIATOR_FUNCTION : 0) |
			(enable_tgt ? FC_PRLI_TARGET_FUNCTION : 0));

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}

	return els;
}

/**
 * @ingroup els_api
 * @brief Send a PRLO ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Construct a PRLO ELS command, and send to the \c node.
 *
 * @param node Node to which the PRLO is sent.
 * @param fc4_type.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_prlo(ocs_node_t *node, uint32_t fc4_type, uint32_t timeout_sec, uint32_t retries, 
	els_cb_t cb, void *cbarg)
{
	ocs_t *ocs = node->ocs;
	ocs_io_t *els;
	fc_prlo_payload_t *prlo;

	node_els_trace();

	els = ocs_els_io_alloc(node, sizeof(*prlo), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "prlo";

		/* Build PRLO request */
		prlo = els->els_info->els_req.virt;

		ocs_memset(prlo, 0, sizeof(*prlo));
		prlo->command_code = FC_ELS_CMD_PRLO;
		prlo->page_length = 16;
		prlo->payload_length = ocs_htobe16(sizeof(fc_prlo_payload_t));
		prlo->type = fc4_type;
		prlo->type_ext = 0;

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @ingroup els_api
 * @brief Send a LOGO ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Format a LOGO, and send to the \c node.
 *
 * @param node Node to which the LOGO is sent.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_logo(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs;
	fc_logo_payload_t *logo;
	fc_plogi_payload_t *sparams;


	ocs = node->ocs;

	node_els_trace();

	sparams = (fc_plogi_payload_t*) node->sport->service_params;

	els = ocs_els_io_alloc(node, sizeof(*logo), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "logo";

		/* Build LOGO request */

		logo = els->els_info->els_req.virt;

		ocs_memset(logo, 0, sizeof(*logo));
		logo->command_code = FC_ELS_CMD_LOGO;
		logo->resv1 = 0;
		logo->port_id = fc_htobe24(node->rnode.sport->fc_id);
		logo->port_name_hi = sparams->port_name_hi;
		logo->port_name_lo = sparams->port_name_lo;

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @ingroup els_api
 * @brief Send an ADISC ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Construct an ADISC ELS command, and send to the \c node.
 *
 * @param node Node to which the ADISC is sent.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_adisc(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs;
	fc_adisc_payload_t *adisc;
	fc_plogi_payload_t *sparams;
	ocs_sport_t *sport = node->sport;

	ocs = node->ocs;

	node_els_trace();

	sparams = (fc_plogi_payload_t*) node->sport->service_params;

	els = ocs_els_io_alloc(node, sizeof(*adisc), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "adisc";

		/* Build ADISC request */

		adisc = els->els_info->els_req.virt;

		ocs_memset(adisc, 0, sizeof(*adisc));
		adisc->command_code = FC_ELS_CMD_ADISC;
		adisc->hard_address = fc_htobe24(sport->fc_id);
		adisc->port_name_hi = sparams->port_name_hi;
		adisc->port_name_lo = sparams->port_name_lo;
		adisc->node_name_hi = sparams->node_name_hi;
		adisc->node_name_lo = sparams->node_name_lo;
		adisc->port_id = fc_htobe24(node->rnode.sport->fc_id);

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @ingroup els_api
 * @brief Send a PDISC ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Construct a PDISC ELS command, and send to the \c node.
 *
 * @param node Node to which the PDISC is sent.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_pdisc(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fc_plogi_payload_t *pdisc;

	node_els_trace();

	els = ocs_els_io_alloc(node, sizeof(*pdisc), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "pdisc";

		pdisc = els->els_info->els_req.virt;

		ocs_memcpy(pdisc, node->sport->service_params, sizeof(*pdisc));

		pdisc->command_code = FC_ELS_CMD_PDISC;
		pdisc->resv1 = 0;

		/* Zero-out the BB_SC_N field (word 1, bits 15:12) */
		pdisc->common_service_parameters[1] &= ocs_htobe32(FC_PLOGI_CSP_W1_BBSCN_CLEAR_MASK);

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @ingroup els_api
 * @brief Send an SCR ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Format an SCR, and send to the \c node.
 *
 * @param node Node to which the SCR is sent.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function
 * @param cbarg Callback function arg
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_scr(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fc_scr_payload_t *req;

	node_els_trace();

	els = ocs_els_io_alloc(node, sizeof(*req), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "scr";

		req = els->els_info->els_req.virt;

		ocs_memset(req, 0, sizeof(*req));
		req->command_code = FC_ELS_CMD_SCR;
		req->function = FC_SCR_REG_FULL;

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @ingroup els_api
 * @brief Send an RRQ ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Format an RRQ, and send to the \c node.
 *
 * @param node Node to which the RRQ is sent.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function
 * @param cbarg Callback function arg
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_rrq(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fc_scr_payload_t *req;

	node_els_trace();

	els = ocs_els_io_alloc(node, sizeof(*req), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "scr";

		req = els->els_info->els_req.virt;

		ocs_memset(req, 0, sizeof(*req));
		req->command_code = FC_ELS_CMD_RRQ;
		req->function = FC_SCR_REG_FULL;

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @ingroup els_api
 * @brief Send an RSCN ELS command.
 *
 * <h3 class="desc">Description</h3>
 * Format an RSCN, and send to the \c node.
 *
 * @param node Node to which the RRQ is sent.
 * @param timeout_sec Command timeout, in seconds.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param port_ids Pointer to port IDs
 * @param port_ids_count Count of port IDs
 * @param cb Callback function
 * @param cbarg Callback function arg
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_send_rscn(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	void *port_ids, uint32_t port_ids_count, els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fc_rscn_payload_t *req;
	uint32_t payload_length = sizeof(fc_rscn_affected_port_id_page_t)*(port_ids_count - 1) +
		sizeof(fc_rscn_payload_t);

	node_els_trace();

	els = ocs_els_io_alloc(node, payload_length, OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->els_info->els_timeout_sec = timeout_sec;
		els->els_info->els_retries_remaining = retries;
		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "rscn";

		req = els->els_info->els_req.virt;

		req->command_code = FC_ELS_CMD_RSCN;
		req->page_length = sizeof(fc_rscn_affected_port_id_page_t);
		req->payload_length = ocs_htobe16(sizeof(*req) +
			sizeof(fc_rscn_affected_port_id_page_t)*(port_ids_count-1));

		els->hio_type = OCS_HAL_ELS_REQ;
		els->iparam.els.timeout = timeout_sec;

		/* copy in the payload */
		ocs_memcpy(req->port_list, port_ids, port_ids_count*sizeof(fc_rscn_affected_port_id_page_t));

		/* Submit the request */
		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @brief Send an LS_RJT ELS response.
 *
 * <h3 class="desc">Description</h3>
 * Send an LS_RJT ELS response.
 *
 * @param io Pointer to a SCSI IO object.
 * @param ox_id Originator exchange ID being responded to.
 * @param reason_code Reason code value for LS_RJT.
 * @param reason_code_expl Reason code explanation value for LS_RJT.
 * @param vendor_unique Vendor-unique value for LS_RJT.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_ls_rjt(ocs_io_t *io, uint32_t ox_id, uint32_t reason_code, uint32_t reason_code_expl,
		uint32_t vendor_unique, els_cb_t cb, void *cbarg)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	ocs_t *ocs = node->ocs;
	fc_ls_rjt_payload_t *rjt;

	node_els_trace();

	io->els_info->els_rjt = 1;
	io->els_info->els_callback = cb;
	io->els_info->els_callback_arg = cbarg;
	io->display_name = "ls_rjt";
	io->init_task_tag = ox_id;

	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.els.ox_id = ox_id;

	rjt = io->els_info->els_req.virt;
	ocs_memset(rjt, 0, sizeof(*rjt));

	rjt->command_code = FC_ELS_CMD_RJT;
	rjt->reason_code = reason_code;
	rjt->reason_code_exp = reason_code_expl;

	io->hio_type = OCS_HAL_ELS_RSP;
	if ((rc = ocs_els_send_rsp(io, sizeof(*rjt)))) {
		ocs_els_io_free(io);
		io = NULL;
	}

	return io;
}

/**
 * @ingroup els_api
 * @brief Send a PLOGI accept response.
 *
 * <h3 class="desc">Description</h3>
 * Construct a PLOGI LS_ACC, and send to the \c node, using the originator exchange ID
 * \c ox_id.
 *
 * @param io Pointer to a SCSI IO object.
 * @param ox_id Originator exchange ID being responsed to.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_send_plogi_acc(ocs_io_t *io, uint32_t ox_id, els_cb_t cb, void *cbarg)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	uint32_t suppress_rsp;
	ocs_t *ocs = node->ocs;
	fc_plogi_payload_t *plogi;
	fc_plogi_payload_t *req = (fc_plogi_payload_t *)node->service_params;

	node_els_trace();

	io->els_info->els_callback = cb;
	io->els_info->els_callback_arg = cbarg;
	io->display_name = "plog_acc";
	io->init_task_tag = ox_id;

	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.els.ox_id = ox_id;

	plogi = io->els_info->els_req.virt;

	/* copy our port's service paramters to payload */
	ocs_memcpy(plogi, node->sport->service_params, sizeof(*plogi));
	plogi->command_code = FC_ELS_CMD_ACC;
	plogi->resv1 = 0;

	ocs_hal_get(&ocs->hal, OCS_HAL_SUPPRESS_RSP_CAPABLE, &suppress_rsp);
	if (suppress_rsp && node->suppress_rsp) {
		/* Set valid vendor version level bit */
		plogi->common_service_parameters[1] |= ocs_htobe32(FC_PLOGI_CSP_W1_VALID_VENDOR_VER_LEVEL);
		plogi->vendor_version_level[0] = ocs_htobe32(OCS_FC_PLOGI_VENDOR_VERSION_EMLX_ID);
		plogi->vendor_version_level[1] = ocs_htobe32(suppress_rsp);
	} else {
		plogi->common_service_parameters[1] &= ocs_htobe32(~FC_PLOGI_CSP_W1_VALID_VENDOR_VER_LEVEL);
		plogi->vendor_version_level[0] = 0;
		plogi->vendor_version_level[1] = 0;
		plogi->vendor_version_level[2] = 0;
		plogi->vendor_version_level[3] = 0;
		node->suppress_rsp = false;
	}

	/* Zero-out the BB_SC_N field (word 1, bits 15:12) */
	plogi->common_service_parameters[1] &= ocs_htobe32(FC_PLOGI_CSP_W1_BBSCN_CLEAR_MASK);

	/* Set Application header support bit if requested */
	if (req->common_service_parameters[1] & ocs_htobe32(FC_PLOGI_CSP_W1_APP_HDR_SUPPORT)) {
		if (sli_feature_enabled(&ocs->hal.sli, SLI4_FEATURE_ASHDR))
			plogi->common_service_parameters[1] |= ocs_htobe32(FC_PLOGI_CSP_W1_APP_HDR_SUPPORT);
	}

	/* Priority tagging support */
	if (req->common_service_parameters[1] & ocs_htobe32(1U << 23))
		plogi->common_service_parameters[1] |= ocs_htobe32(1U << 23);

	ocs_display_sparams(node->display_name, "plogi send resp", 0, NULL, plogi->common_service_parameters);

	io->hio_type = OCS_HAL_ELS_RSP;
	if ((rc = ocs_els_send_rsp(io, sizeof(*plogi)))) {
		ocs_els_io_free(io);
		io = NULL;
	}

	ocs_log_info(ocs, "ELS PLOGI (0x%x) ACK sent %s; OX_ID: %04x, " \
		     "D_ID: %06x, WWPN: %016" PRIX64"\n",
		     FC_ELS_CMD_PLOGI, rc ? "failed" : "successfully",
		     ox_id, node->rnode.fc_id, ocs_node_get_wwpn(node));
	return io;
}

/**
 * @ingroup els_api
 * @brief Send an FLOGI accept response for point-to-point negotiation.
 *
 * <h3 class="desc">Description</h3>
 * Construct an FLOGI accept response, and send to the \c node using the originator
 * exchange id \c ox_id. The \c s_id is used for the response frame source FC ID.
 *
 * @param io Pointer to a SCSI IO object.
 * @param ox_id Originator exchange ID for the response.
 * @param s_id Source FC ID to be used in the response frame.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_send_flogi_p2p_acc(ocs_io_t *io, uint32_t ox_id, uint32_t s_id, els_cb_t cb, void *cbarg)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	ocs_t *ocs = node->ocs;
	fc_plogi_payload_t *flogi;

	node_els_trace();

	io->els_info->els_callback = cb;
	io->els_info->els_callback_arg = cbarg;
	io->display_name = "flogi_p2p_acc";
	io->init_task_tag = ox_id;

	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.els_sid.ox_id = ox_id;
	io->iparam.els_sid.s_id = s_id;

	flogi = io->els_info->els_req.virt;

	/* copy our port's service paramters to payload */
	ocs_memcpy(flogi, node->sport->service_params, sizeof(*flogi));
	flogi->command_code = FC_ELS_CMD_ACC;
	flogi->resv1 = 0;

	/* Zero-out the BB_SC_N field (word 1, bits 15:12) */
	flogi->common_service_parameters[1] &= ocs_htobe32(FC_PLOGI_CSP_W1_BBSCN_CLEAR_MASK);

	ocs_memset(flogi->class1_service_parameters, 0, sizeof(flogi->class1_service_parameters));
	ocs_memset(flogi->class2_service_parameters, 0, sizeof(flogi->class1_service_parameters));
	ocs_memset(flogi->class3_service_parameters, 0, sizeof(flogi->class1_service_parameters));
	ocs_memset(flogi->class4_service_parameters, 0, sizeof(flogi->class1_service_parameters));

	io->hio_type = OCS_HAL_ELS_RSP_SID;
	if ((rc = ocs_els_send_rsp(io, sizeof(*flogi)))) {
		ocs_els_io_free(io);
		io = NULL;
	}

	return io;
}

ocs_io_t *
ocs_send_flogi_acc(ocs_io_t *io, uint32_t ox_id, uint32_t is_fport, els_cb_t cb, void *cbarg)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	ocs_t *ocs = node->ocs;
	fc_plogi_payload_t *flogi;

	node_els_trace();

	io->els_info->els_callback = cb;
	io->els_info->els_callback_arg = cbarg;
	io->display_name = "flogi_acc";
	io->init_task_tag = ox_id;

	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.els_sid.ox_id = ox_id;
	io->iparam.els_sid.s_id = io->node->sport->fc_id;

	flogi = io->els_info->els_req.virt;

	/* copy our port's service paramters to payload */
	ocs_memcpy(flogi, node->sport->service_params, sizeof(*flogi));

	/* Set F_port */
	if (is_fport) {
		/* Set F_PORT and Multiple N_PORT_ID Assignment */
		flogi->common_service_parameters[1] |= ocs_be32toh(3U << 28);
	}

	flogi->command_code = FC_ELS_CMD_ACC;
	flogi->resv1 = 0;

	/* Zero-out the BB_SC_N field (word 1, bits 15:12) */
	flogi->common_service_parameters[1] &= ocs_htobe32(FC_PLOGI_CSP_W1_BBSCN_CLEAR_MASK);

	ocs_display_sparams(node->display_name, "flogi send resp", 0, NULL, flogi->common_service_parameters);

	ocs_memset(flogi->class1_service_parameters, 0, sizeof(flogi->class1_service_parameters));
	ocs_memset(flogi->class2_service_parameters, 0, sizeof(flogi->class1_service_parameters));
	ocs_memset(flogi->class3_service_parameters, 0, sizeof(flogi->class1_service_parameters));
	ocs_memset(flogi->class4_service_parameters, 0, sizeof(flogi->class1_service_parameters));

	io->hio_type = OCS_HAL_ELS_RSP_SID;
	if ((rc = ocs_els_send_rsp(io, sizeof(*flogi)))) {
		ocs_els_io_free(io);
		io = NULL;
	}

	return io;
}

int32_t
ocs_node_nsler_capable(ocs_node_t *node)
{
	return node->nvme_sler && node->nvme_conf;
}

static ocs_io_t *
ocs_send_nvme_prli_acc(ocs_io_t *io)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	uint16_t sparams = 0;
	ocs_t *ocs = node->ocs;
	fc_nvme_prli_payload_t *prli;
	uint32_t ox_id = io->init_task_tag;

	prli = io->els_info->els_req.virt;
	ocs_memset(prli, 0, sizeof(*prli));

	if (ocs_tgt_nvme_enabled(node->ocs))
		sparams = (FC_PRLI_TARGET_FUNCTION | FC_PRLI_NVME_DISC_FUNCTION);

#if 0
	/* Fix LPFC and enable */
	if (ocs_ini_nvme_enabled(node->ocs))
		sparams |= FC_PRLI_INITIATOR_FUNCTION;
#endif

	if (ocs_node_nsler_capable(node) && ocs_nsler_capable(ocs))
		sparams |= (FC_PRLI_RETRY | FC_PRLI_CONFIRMED_COMPLETION);

	prli->command_code = FC_ELS_CMD_ACC;
	prli->type = FC_TYPE_NVME;
	prli->flags = ocs_htobe16(FC_PRLI_REQUEST_EXECUTED);
	prli->service_params = ocs_htobe16(sparams);
	prli->payload_length = ocs_htobe16(sizeof(*prli));
	prli->page_length = FC_PRLI_ACC_NVME_PAGE_LENGTH;
	prli->type_ext = 0;

	io->hio_type = OCS_HAL_ELS_RSP;
	if ((rc = ocs_els_send_rsp(io, sizeof(*prli)))) {
		ocs_els_io_free(io);
		io = NULL;
	}

	ocs_log_info(ocs, "ELS NVMe PRLI (0x%x) ACK sent %s; OX_ID: %04x, " \
		     "D_ID: %06x, WWPN: %016" PRIX64" sparams: %04x\n",
		     FC_ELS_CMD_PRLI, rc ? "failed" : "successfully",
		     ox_id, node->rnode.fc_id, ocs_node_get_wwpn(node), sparams);
	return io;
}

static inline int32_t
ocs_node_first_burst_enabled(ocs_node_t *node)
{
	return node->first_burst;
}

static ocs_io_t *
ocs_send_fcp_prli_acc(ocs_io_t *io)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	int32_t first_burst = FALSE;
	uint16_t sparams = 0;
	ocs_t *ocs = node->ocs;
	fc_prli_payload_t *prli;
	uint32_t ox_id = io->init_task_tag;

	prli = io->els_info->els_req.virt;
	ocs_memset(prli, 0, sizeof(*prli));

	prli->type = FC_TYPE_FCP;
	prli->command_code = FC_ELS_CMD_ACC;
	prli->flags = ocs_htobe16(FC_PRLI_ESTABLISH_IMAGE_PAIR | FC_PRLI_REQUEST_EXECUTED);

	/*
	 * Determine node first_burst capability based on the support at both sides.
	 */
	first_burst = ocs_node_first_burst_enabled(node) && ocs_first_burst_enabled(ocs);
	if (first_burst)
		ocs_log_info(ocs, "Negotiating first burst with initiator %06x: %016" PRIX64"\n",
			     node->rnode.fc_id, ocs_node_get_wwpn(node));

	sparams = (FC_PRLI_READ_XRDY_DISABLED |
		   (first_burst ? FC_PRLI_WRITE_XRDY_DISABLED : 0) |
		   (ocs_ini_scsi_enabled(node->ocs) ? FC_PRLI_INITIATOR_FUNCTION : 0) |
		   (ocs_tgt_scsi_enabled(node->ocs) ? FC_PRLI_TARGET_FUNCTION : 0));
	prli->service_params = ocs_htobe16(sparams);
	prli->payload_length = ocs_htobe16(sizeof(*prli));
	prli->page_length = FC_PRLI_ACC_FCP_PAGE_LENGTH;
	prli->type_ext = 0;

	io->hio_type = OCS_HAL_ELS_RSP;
	if ((rc = ocs_els_send_rsp(io, sizeof(*prli)))) {
		ocs_els_io_free(io);
		io = NULL;
	}

	ocs_log_info(ocs, "ELS FCP PRLI (0x%x) ACK sent %s; OX_ID: %04x, " \
		     "D_ID: %06x, WWPN: %016" PRIX64" sparams: %04x\n",
		     FC_ELS_CMD_PRLI, rc ? "failed" : "successfully",
		     ox_id, node->rnode.fc_id, ocs_node_get_wwpn(node), sparams);
	return io;
}

/**
 * @ingroup els_api
 * @brief Send a PRLI accept response
 *
 * <h3 class="desc">Description</h3>
 * Construct a PRLI LS_ACC response, and send to the \c node, using the originator
 * \c ox_id exchange ID.
 *
 * @param io Pointer to a SCSI IO object.
 * @param ox_id Originator exchange ID.
 * @param fc_type FC4 type: FCP or NVME.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_prli_acc(ocs_io_t *io, uint32_t ox_id, uint8_t fc_type, els_cb_t cb, void *cbarg)
{
	ocs_node_t *node = io->node;
	ocs_t *ocs = node->ocs;

	node_els_trace();

	io->els_info->els_callback = cb;
	io->els_info->els_callback_arg = cbarg;
	io->display_name = "prli_acc";
	io->init_task_tag = ox_id;

	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.els.ox_id = ox_id;

	if (fc_type == FC_TYPE_NVME)
		return ocs_send_nvme_prli_acc(io);

	return ocs_send_fcp_prli_acc(io);
}

/**
 * @ingroup els_api
 * @brief Send a PRLO accept response.
 *
 * <h3 class="desc">Description</h3>
 * Construct a PRLO LS_ACC response, and send to the \c node, using the originator
 * exchange ID \c ox_id.
 *
 * @param io Pointer to a SCSI IO object.
 * @param ox_id Originator exchange ID.
 * @param fc_type FC4 type: FCP or NVME.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_prlo_acc(ocs_io_t *io, uint32_t ox_id, uint8_t fc_type, els_cb_t cb, void *cbarg)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	ocs_t *ocs = node->ocs;
	fc_prlo_acc_payload_t *prlo_acc;

	node_els_trace();

	io->els_info->els_callback = cb;
	io->els_info->els_callback_arg = cbarg;
	io->display_name = "prlo_acc";
	io->init_task_tag = ox_id;

	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.els.ox_id = ox_id;

	prlo_acc = io->els_info->els_req.virt;
	ocs_memset(prlo_acc, 0, sizeof(*prlo_acc));

	if (fc_type == FC_TYPE_NVME)
		prlo_acc->type = FC_TYPE_NVME;
	else
		prlo_acc->type = FC_TYPE_FCP;

	prlo_acc->command_code = FC_ELS_CMD_ACC;
	prlo_acc->page_length = 16;
	prlo_acc->payload_length = ocs_htobe16(sizeof(fc_prlo_acc_payload_t));
	prlo_acc->type_ext = 0;
	prlo_acc->response_code = FC_PRLO_REQUEST_EXECUTED;

	io->hio_type = OCS_HAL_ELS_RSP;
	if ((rc = ocs_els_send_rsp(io, sizeof(*prlo_acc)))) {
		ocs_els_io_free(io);
		io = NULL;
	}

	return io;
}

/**
 * @ingroup els_api
 * @brief Send a generic LS_ACC response without a payload.
 *
 * <h3 class="desc">Description</h3>
 * A generic LS_ACC response is sent to the \c node using the originator exchange ID
 * \c ox_id.
 *
 * @param io Pointer to a SCSI IO object.
 * @param ox_id Originator exchange id.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_send_ls_acc(ocs_io_t *io, uint32_t ox_id, els_cb_t cb, void *cbarg)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	ocs_t *ocs = node->ocs;
	fc_acc_payload_t *acc;

	node_els_trace();

	io->els_info->els_callback = cb;
	io->els_info->els_callback_arg = cbarg;
	io->display_name = "ls_acc";
	io->init_task_tag = ox_id;

	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.els.ox_id = ox_id;

	acc = io->els_info->els_req.virt;
	ocs_memset(acc, 0, sizeof(*acc));

	acc->command_code = FC_ELS_CMD_ACC;

	io->hio_type = OCS_HAL_ELS_RSP;
	if ((rc = ocs_els_send_rsp(io, sizeof(*acc)))) {
		ocs_els_io_free(io);
		io = NULL;
	}

	return io;
}

/**
 * @ingroup els_api
 * @brief Send a LOGO accept response.
 *
 * <h3 class="desc">Description</h3>
 * Construct a LOGO LS_ACC response, and send to the \c node, using the originator
 * exchange ID \c ox_id.
 *
 * @param io Pointer to a SCSI IO object.
 * @param ox_id Originator exchange ID.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_send_logo_acc(ocs_io_t *io, uint32_t ox_id, els_cb_t cb, void *cbarg)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	ocs_t *ocs = node->ocs;
	fc_acc_payload_t *logo;

	node_els_trace();

	io->els_info->els_callback = cb;
	io->els_info->els_callback_arg = cbarg;
	io->display_name = "logo_acc";
	io->init_task_tag = ox_id;

	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.els.ox_id = ox_id;

	logo = io->els_info->els_req.virt;
	ocs_memset(logo, 0, sizeof(*logo));

	logo->command_code = FC_ELS_CMD_ACC;
	logo->resv1 = 0;

	io->hio_type = OCS_HAL_ELS_RSP;
	if ((rc = ocs_els_send_rsp(io, sizeof(*logo)))) {
		ocs_els_io_free(io);
		io = NULL;
	}

	return io;
}

/**
 * @ingroup els_api
 * @brief Send an ADISC accept response.
 *
 * <h3 class="desc">Description</h3>
 * Construct an ADISC LS__ACC, and send to the \c node, using the originator
 * exchange id \c ox_id.
 *
 * @param io Pointer to a SCSI IO object.
 * @param ox_id Originator exchange ID.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_send_adisc_acc(ocs_io_t *io, uint32_t ox_id, els_cb_t cb, void *cbarg)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	fc_adisc_payload_t *adisc;
	fc_plogi_payload_t *sparams;
	ocs_t *ocs;

	ocs_assert(node, NULL);
	ocs_assert(node->ocs, NULL);
	ocs = node->ocs;

	node_els_trace();

	io->els_info->els_callback = cb;
	io->els_info->els_callback_arg = cbarg;
	io->display_name = "adisc_acc";
	io->init_task_tag = ox_id;

	/* Go ahead and send the ELS_ACC */
	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.els.ox_id = ox_id;

	sparams = (fc_plogi_payload_t*) node->sport->service_params;
	adisc = io->els_info->els_req.virt;
	ocs_memset(adisc, 0, sizeof(fc_adisc_payload_t));
	adisc->command_code = FC_ELS_CMD_ACC;
	adisc->hard_address = 0;
	adisc->port_name_hi = sparams->port_name_hi;
	adisc->port_name_lo = sparams->port_name_lo;
	adisc->node_name_hi = sparams->node_name_hi;
	adisc->node_name_lo = sparams->node_name_lo;
	adisc->port_id = fc_htobe24(node->rnode.sport->fc_id);

	io->hio_type = OCS_HAL_ELS_RSP;
	if ((rc = ocs_els_send_rsp(io, sizeof(*adisc)))) {
		ocs_els_io_free(io);
		io = NULL;
	}

	return io;
}

/**
 * @ingroup els_api
 * @brief Send a RFTID CT request.
 *
 * <h3 class="desc">Description</h3>
 * Construct an RFTID CT request, and send to the \c node.
 *
 * @param node Node to which the RFTID request is sent.
 * @param timeout_sec Time, in seconds, to wait before timing out the ELS.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_ns_send_rftid(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fcct_rftid_req_t *rftid;

	node_els_trace();

	els = ocs_els_io_alloc(node, sizeof(*rftid), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {

		els->iparam.fc_ct.r_ctl = FC_RCTL_ELS;
		els->iparam.fc_ct.type = FC_TYPE_GS;
		els->iparam.fc_ct.df_ctl = 0;
		els->iparam.fc_ct.timeout = timeout_sec;

		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "rftid";

		rftid = els->els_info->els_req.virt;

		ocs_memset(rftid, 0, sizeof(*rftid));
		fcct_build_req_header(&rftid->hdr, FC_GS_NAMESERVER_RFT_ID, FC_GS_TYPE_DIRECTORY_SERVICE,
				FC_GS_SUBTYPE_NAME_SERVER,
				(OCS_ELS_RSP_LEN - sizeof(rftid->hdr)));

		rftid->port_id = ocs_htobe32(node->rnode.sport->fc_id);

		if (ocs_tgt_nvme_enabled(node->ocs) || ocs_ini_nvme_enabled(node->ocs))
			rftid->fc4_types[FC_GS_TYPE_WORD(FC_TYPE_NVME)] = ocs_htobe32(1 << FC_GS_TYPE_BIT(FC_TYPE_NVME));

		if (ocs_tgt_scsi_enabled(node->ocs) || ocs_ini_scsi_enabled(node->ocs))
			rftid->fc4_types[FC_GS_TYPE_WORD(FC_TYPE_FCP)] = ocs_htobe32(1 << FC_GS_TYPE_BIT(FC_TYPE_FCP));

		els->hio_type = OCS_HAL_FC_CT;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}

	return els;
}

/**
 * @ingroup els_api
 * @brief Send a RFFID CT request.
 *
 * <h3 class="desc">Description</h3>
 * Construct an RFFID CT request, and send to the \c node.
 *
 * @param node Node to which the RFFID request is sent.
 * @param timeout_sec Time, in seconds, to wait before timing out the ELS.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_ns_send_rffid(ocs_node_t *node, uint8_t fc_type, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fcct_rffid_req_t *rffid;

	node_els_trace();

	els = ocs_els_io_alloc(node, sizeof(*rffid), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {

		els->iparam.fc_ct.r_ctl = FC_RCTL_ELS;
		els->iparam.fc_ct.type = FC_TYPE_GS;
		els->iparam.fc_ct.df_ctl = 0;
		els->iparam.fc_ct.timeout = timeout_sec;

		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "rffid";

		rffid = els->els_info->els_req.virt;

		ocs_memset(rffid, 0, sizeof(*rffid));
		fcct_build_req_header(&rffid->hdr, FC_GS_NAMESERVER_RFF_ID, FC_GS_TYPE_DIRECTORY_SERVICE,
					FC_GS_SUBTYPE_NAME_SERVER,
					(OCS_ELS_RSP_LEN - sizeof(rffid->hdr)));

		rffid->port_id = ocs_htobe32(node->rnode.sport->fc_id);
		if (fc_type == FC_TYPE_NVME) {
			rffid->type = FC_TYPE_NVME;
			if (ocs_tgt_nvme_enabled(node->ocs)) {
				rffid->fc4_feature_bits |= FC4_FEATURE_TARGET;
				rffid->fc4_feature_bits |= FC4_FEATURE_NVME_DISC;
			}

			if (ocs_ini_nvme_enabled(node->ocs))
				rffid->fc4_feature_bits |= FC4_FEATURE_INITIATOR;
		} else {
			rffid->type = FC_TYPE_FCP;
			if (ocs_ini_scsi_enabled(node->ocs))
				rffid->fc4_feature_bits |= FC4_FEATURE_INITIATOR;

			if (ocs_tgt_scsi_enabled(node->ocs))
				rffid->fc4_feature_bits |= FC4_FEATURE_TARGET;
		}

		els->hio_type = OCS_HAL_FC_CT;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}

	return els;
}

/**
 * @ingroup els_api
 * @brief Send a GIDPT CT request.
 *
 * <h3 class="desc">Description</h3>
 * Construct a GIDPT CT request, and send to the \c node.
 *
 * @param node Node to which the GIDPT request is sent.
 * @param timeout_sec Time, in seconds, to wait before timing out the ELS.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_ns_send_gidpt(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fcct_gidpt_req_t *gidpt;

	node_els_trace();

	els = ocs_els_io_alloc_size(node, sizeof(*gidpt), OCS_ELS_GID_PT_RSP_LEN, OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {

		els->iparam.fc_ct.r_ctl = FC_RCTL_ELS;
		els->iparam.fc_ct.type = FC_TYPE_GS;
		els->iparam.fc_ct.df_ctl = 0;
		els->iparam.fc_ct.timeout = timeout_sec;

		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "gidpt";

		gidpt = els->els_info->els_req.virt;

		ocs_memset(gidpt, 0, sizeof(*gidpt));
		fcct_build_req_header(&gidpt->hdr, FC_GS_NAMESERVER_GID_PT, FC_GS_TYPE_DIRECTORY_SERVICE,
					FC_GS_SUBTYPE_NAME_SERVER,
					(OCS_ELS_GID_PT_RSP_LEN - sizeof(gidpt->hdr)) );
		gidpt->domain_id_scope = 0;
		gidpt->area_id_scope = 0;
		gidpt->port_type = 0x7f;

		els->hio_type = OCS_HAL_FC_CT;

		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}


ocs_io_t *
ocs_ns_send_loopback_frame(ocs_node_t *node, void *buf, uint32_t size, void *rx_buf, uint32_t rx_buf_size,
			uint32_t timeout_sec, uint32_t retries,
			els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fcct_iu_header_t *fcct_header;
	uint32_t	total_size;
	void		*ptr = NULL;

	node_els_trace();

	if (rx_buf == NULL) {
		ocs_log_err(ocs, "failed to allocate RX buff data\n");
		return NULL;
	}
	total_size = size + sizeof(fcct_iu_header_t);
	els = ocs_els_io_alloc_size(node, total_size, OCS_ELS_GID_PT_RSP_LEN, OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {

		els->iparam.fc_ct.r_ctl = FC_RCTL_ELS;
		els->iparam.fc_ct.type = FC_TYPE_GS;
		els->iparam.fc_ct.df_ctl = 0;
		els->iparam.fc_ct.timeout = timeout_sec;

		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->els_info->els_retries_remaining = retries;
		els->display_name = "loopback_frame";

		ptr = els->els_info->els_req.virt;
		fcct_header = (fcct_iu_header_t *)ptr;

		els->els_info->loopback_evt_data.is_loopback_frame = 1;
		els->els_info->loopback_evt_data.loopback_rx_data = rx_buf;
		els->els_info->loopback_evt_data.loopback_rx_data_len = rx_buf_size;
		ocs_sem_init(&els->els_info->loopback_evt_data.wait_io_sem, 0, "loopback_frame_sem");

		ocs_memset(fcct_header, 0, sizeof(*fcct_header));
		fcct_loopback_build_req_header(fcct_header, FC_ELX_LOOPBACK_DATA, 0);
		ptr = (uint8_t *)ptr + sizeof(fcct_iu_header_t);
		ocs_memcpy(ptr, buf, size);

		els->hio_type = OCS_HAL_FC_CT;

		ocs_ref_get(&els->ref);
		ocs_io_transition(els, __ocs_els_init, NULL);
	}
	return els;
}

/**
 * @ingroup els_api
 * @brief Send a GA_NEXT CT request.
 *
 * <h3 class="desc">Description</h3>
 * Construct a GA_NEXT CT request, and send to the \c node.
 *
 * @param node Node to which the GIDFT request is sent.
 * @param timeout_sec Time, in seconds, to wait before timing out the ELS.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_ns_send_ganxt(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
		  els_cb_t cb, void *cbarg, uint32_t port_id)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fcct_ganxt_req_t *ganxt;

	node_els_trace();

	els = ocs_els_io_alloc_size(node, sizeof(*ganxt), OCS_ELS_RSP_LEN, OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->iparam.fc_ct.r_ctl = FC_RCTL_ELS;
		els->iparam.fc_ct.type = FC_TYPE_GS;
		els->iparam.fc_ct.df_ctl = 0;
		els->iparam.fc_ct.timeout = timeout_sec;

		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "ganxt";

		ganxt = els->els_info->els_req.virt;

		ocs_memset(ganxt, 0, sizeof(*ganxt));
		fcct_build_req_header(&ganxt->hdr, FC_GS_NAMESERVER_GA_NXT, FC_GS_TYPE_DIRECTORY_SERVICE,
				      FC_GS_SUBTYPE_NAME_SERVER, (OCS_ELS_RSP_LEN - sizeof(ganxt->hdr)));
		if (!port_id)
			port_id = fc_htobe24(node->sport->fc_id);

		ganxt->port_id = port_id;
		els->hio_type = OCS_HAL_FC_CT;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}

	return els;
}

/**
 * @ingroup els_api
 * @brief Send a RHBA CT request.
 *
 * <h3 class="desc">Description</h3>
 * Construct an RHBA CT request, and send to the \c node.
 *
 * @param node Node to which the RHBA request is sent.
 * @param timeout_sec Time, in seconds, to wait before timing out the ELS.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_fdmi_send_rhba(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fcct_fdmi_rhba_req_t *rhba;
	int  (*func)(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);

	node_els_trace();

	els = ocs_els_io_alloc(node, OCS_ELS_FDMI_REG_LEN, OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		uint32_t	index = 0, size = 0;
		uint32_t	num_port_entries = 0, mask = 0;
		ocs_fdmi_attr_block_t *ab = NULL;

		els->iparam.fc_ct.r_ctl = FC_RCTL_ELS;
		els->iparam.fc_ct.type = FC_TYPE_GS;
		els->iparam.fc_ct.df_ctl = 0;
		els->iparam.fc_ct.timeout = timeout_sec;

		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->display_name = "rhba";
		els->els_info->els_retries_remaining = retries;

		rhba = (fcct_fdmi_rhba_req_t *)els->els_info->els_req.virt;

		ocs_memset(rhba, 0, sizeof(*rhba));
		fcct_build_req_header(&rhba->hdr, FC_GS_FDMI_RHBA, FC_GS_TYPE_MANAGEMENT_SERVICE,
					FC_GS_SUBTYPE_FDMI,
					(OCS_ELS_FDMI_REG_LEN - sizeof(rhba->hdr)));
		size += sizeof(rhba->hdr);

		/* HBA identifier */
		ocs_memcpy(&rhba->hba_identifier, &node->sport->sli_wwpn, sizeof(node->sport->sli_wwpn));
		size += sizeof(rhba->hba_identifier);
		num_port_entries = 1;
		rhba->port_list.entry_count = ocs_htobe32(num_port_entries);
		ocs_memcpy(&rhba->port_list.port_entry[0], &node->sport->sli_wwpn, sizeof(node->sport->sli_wwpn));
		size += sizeof(rhba->port_list) + num_port_entries * sizeof(node->sport->sli_wwpn);
		/* point to the HBA attribute block */
		ab = (ocs_fdmi_attr_block_t *)((uint8_t *)rhba + size);
		ab->num_entries = 0;
		size += sizeof(ab->num_entries);

		func = ocs_fdmi_hba_action[index];
		mask = node->sport->fdmi_hba_mask;
		/* Build all required HBA info in the request */
		while (func && mask) {
			if (mask & 0x1) {
				size += func(node->sport, (ocs_fdmi_attr_def_t *)((uint8_t *)rhba + size));
				ab->num_entries++;
				if ((size + MAX_FDMI_ATTRIBUTE_SIZE) > OCS_ELS_FDMI_REG_LEN)
					goto hba_reg_exit;
			}
			mask = mask >> 1;
			func = ocs_fdmi_hba_action[++index];
		}
hba_reg_exit:
		ab->num_entries = ocs_htobe32(ab->num_entries);
		els->hio_type = OCS_HAL_FC_CT;

		ocs_io_transition(els, __ocs_els_init, NULL);
	}

	return els;
}

/**
 * @ingroup els_api
 * @brief Send a FDMI dereg CT request.
 *
 * <h3 class="desc">Description</h3>
 * Construct an DHBA/DPRT CT request, and send to the \c node.
 *
 * @param node Node to which the dereg request is sent.
 * @param timeout_sec Time, in seconds, to wait before timing out the ELS.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_fdmi_send_dereg_cmd(ocs_node_t *node, uint16_t dereg_code, uint32_t timeout_sec,
		  uint32_t retries, els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fcct_fdmi_dereg_req_t *dereg_req;

	node_els_trace();

	els = ocs_els_io_alloc(node, sizeof(*dereg_req), OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		els->iparam.fc_ct.r_ctl = FC_RCTL_ELS;
		els->iparam.fc_ct.type = FC_TYPE_GS;
		els->iparam.fc_ct.df_ctl = 0;
		els->iparam.fc_ct.timeout = timeout_sec;

		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;

		els->display_name = "dereg_cmd";
		els->els_info->els_retries_remaining = retries;

		dereg_req = (fcct_fdmi_dereg_req_t *)els->els_info->els_req.virt;

		ocs_memset(dereg_req, 0, sizeof(*dereg_req));
		fcct_build_req_header(&dereg_req->hdr, dereg_code, FC_GS_TYPE_MANAGEMENT_SERVICE,
					FC_GS_SUBTYPE_FDMI,
					(OCS_ELS_FDMI_RSP_LEN - sizeof(dereg_req->hdr)));
		/* HBA identifier */
		dereg_req->identifier = node->sport->sli_wwpn;
		els->hio_type = OCS_HAL_FC_CT;
		ocs_io_transition(els, __ocs_els_init, NULL);
	}

	return els;
}

/**
 * @ingroup els_api
 * @brief Send a RPRT/RPA CT request.
 *
 * <h3 class="desc">Description</h3>
 * Construct an RPRT/RPA CT request, and send to the \c node.
 *
 * @param node Node to which the RPRT/RPA request is sent.
 * @req_code command code(RPRT/RPA)
 * @param timeout_sec Time, in seconds, to wait before timing out the ELS.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */
ocs_io_t *
ocs_fdmi_send_reg_port(ocs_node_t *node, int req_code, uint32_t timeout_sec,
		       uint32_t retries, els_cb_t cb, void *cbarg)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	int  (*func)(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad);

	node_els_trace();

	els = ocs_els_io_alloc(node, OCS_ELS_FDMI_REG_LEN, OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "IO alloc failed\n");
	} else {
		uint32_t index = 0, size = 0, mask;
		ocs_fdmi_attr_block_t *pab = NULL;

		els->iparam.fc_ct.r_ctl = FC_RCTL_ELS;
		els->iparam.fc_ct.type = FC_TYPE_GS;
		els->iparam.fc_ct.df_ctl = 0;
		els->iparam.fc_ct.timeout = timeout_sec;

		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->els_info->els_retries_remaining = retries;
		if (req_code == FC_GS_FDMI_RPRT) {
			fcct_fdmi_rprt_req_t *rprt;

			els->display_name = "rprt";
			rprt = (fcct_fdmi_rprt_req_t *)els->els_info->els_req.virt;
			ocs_memset(rprt, 0, sizeof(*rprt));
			fcct_build_req_header(&rprt->hdr, req_code, FC_GS_TYPE_MANAGEMENT_SERVICE,
					FC_GS_SUBTYPE_FDMI,
					(OCS_ELS_FDMI_REG_LEN - sizeof(rprt->hdr)));
			size += sizeof(rprt->hdr);
			/* Physical port HBA identifier */
			ocs_memcpy(&rprt->hba_identifier, &ocs->domain->sport->sli_wwpn,
				sizeof(ocs->domain->sport->sli_wwpn));
			size += sizeof(ocs->domain->sport->sli_wwpn);

			/* Port name */
			ocs_memcpy(&rprt->port_name, &node->sport->sli_wwpn,
				sizeof(node->sport->sli_wwpn));
			size += sizeof(node->sport->sli_wwpn);
		} else {
			fcct_fdmi_rpa_req_t *rpa;

			els->display_name = "rpa";
			rpa = (fcct_fdmi_rpa_req_t *)els->els_info->els_req.virt;
			ocs_memset(rpa, 0, sizeof(*rpa));
			fcct_build_req_header(&rpa->hdr, req_code, FC_GS_TYPE_MANAGEMENT_SERVICE,
					FC_GS_SUBTYPE_FDMI, (OCS_ELS_FDMI_REG_LEN - sizeof(rpa->hdr)));
			size += sizeof(rpa->hdr);
			/* Port name */
			ocs_memcpy(&rpa->port_name[0], &node->sport->sli_wwpn,
				sizeof(node->sport->sli_wwpn));
			size += sizeof(node->sport->sli_wwpn);
		}

		/* Now point to port attribute block */
		pab = (ocs_fdmi_attr_block_t *)((uint8_t *)els->els_info->els_req.virt + size);

		pab->num_entries = 0;
		size += sizeof(pab->num_entries);

		func = ocs_fdmi_port_action[index];
		mask = node->sport->fdmi_port_mask;
		/* Mask will dictate what attributes to build in the request */
		while (func && mask) {
			size += func(node->sport, (ocs_fdmi_attr_def_t *)
					((uint8_t *)els->els_info->els_req.virt + size));
			pab->num_entries++;
			if ((size + MAX_FDMI_ATTRIBUTE_SIZE) > OCS_ELS_FDMI_REG_LEN)
				goto reg_port_exit;
			mask = mask >> 1;
			func = ocs_fdmi_port_action[++index];
		}
reg_port_exit:
		pab->num_entries = ocs_htobe32(pab->num_entries);
		els->hio_type = OCS_HAL_FC_CT;

		ocs_io_transition(els, __ocs_els_init, NULL);
	}

	return els;
}

/**
 * @ingroup els_api
 * @brief Send a GHRL CT request.
 *
 * <h3 class="desc">Description</h3>
 * Construct a GHRL CT request, and send to the \c node.
 *
 * @param node Node to which the GHRL request is sent.
 * @param timeout_sec Time, in seconds, to wait before timing out the ELS.
 * @param retries Number of times to retry errors before reporting a failure.
 * @param cb Callback function.
 * @param cbarg Callback function argument.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_fdmi_send_get_cmd(ocs_node_t *node, uint32_t timeout_sec,
		uint32_t retries, els_cb_t cb,
		void *cbarg, ocs_fdmi_get_cmd_req_info_t *req_info)
{
	ocs_io_t *els;
	ocs_t *ocs = node->ocs;
	fcct_iu_header_t *fcct_header = NULL;
	node_els_trace();

	els = ocs_els_io_alloc_size(node, req_info->ct_cmd_req_size,
				    OCS_ELS_FDMI_RSP_LEN, OCS_ELS_ROLE_ORIGINATOR);
	if (els == NULL) {
		ocs_log_err(ocs, "ELS IO alloc failed\n");
	} else {
		els->iparam.fc_ct.r_ctl = FC_RCTL_ELS;
		els->iparam.fc_ct.type = FC_TYPE_GS;
		els->iparam.fc_ct.df_ctl = 0;
		els->iparam.fc_ct.timeout = timeout_sec;

		els->els_info->els_callback = cb;
		els->els_info->els_callback_arg = cbarg;
		els->els_info->els_retries_remaining = retries;

		fcct_header = (fcct_iu_header_t *)els->els_info->els_req.virt;

		fcct_build_req_header(fcct_header, req_info->cmd_code, FC_GS_TYPE_MANAGEMENT_SERVICE,
					FC_GS_SUBTYPE_FDMI,
					(OCS_ELS_FDMI_RSP_LEN - sizeof(fcct_iu_header_t)));

		els->hio_type = OCS_HAL_FC_CT;
		switch (req_info->cmd_code) {
		case FC_GS_FDMI_GRHL:
			els->display_name = "grhl";
			break;
		case FC_GS_FDMI_GRPL: {
				fcct_fdmi_grpl_req_t *grpl_req = els->els_info->els_req.virt;

				els->display_name = "grpl";
				grpl_req->hba_identifier = ocs_htobe64(req_info->identifier);
			}
			break;
		case FC_GS_FDMI_GHAT: {
				fcct_fdmi_ghat_req_t *ghat_req = els->els_info->els_req.virt;

				els->display_name = "ghat";
				ghat_req->hba_identifier = ocs_htobe64(req_info->identifier);
			}
			break;
		case FC_GS_FDMI_GPAT: {
				fcct_fdmi_gpat_req_t *gpat_req = els->els_info->els_req.virt;

				els->display_name = "gpat";
				gpat_req->port_identifier = ocs_htobe64(req_info->identifier);
			}
			break;
		default:
			ocs_log_err(ocs, "Unhandled FDMI cmd code: 0x%x\n", req_info->cmd_code);
			ocs_ref_put(&els->ref);
			return NULL;
		}
		ocs_io_transition(els, __ocs_els_init, NULL);
	}

	return els;
}

static uint32_t
ocs_rdp_res_link_service(fc_rdp_link_service_desc_t *desc,
			 uint32_t word0)
{
	desc->tag = ocs_htobe32(RDP_LINK_SERVICE_DESC_TAG);
	desc->payload.els_req = word0;
	desc->length = ocs_htobe32(sizeof(desc->payload));

	return sizeof(fc_rdp_link_service_desc_t);
}

static uint32_t
ocs_rdp_res_sfp_desc(fc_rdp_sfp_desc_t *desc,
		     uint8_t *page_a0, uint8_t *page_a2)
{
	uint16_t wavelength;
	uint16_t temperature;
	uint16_t rx_power;
	uint16_t tx_bias;
	uint16_t tx_power;
	uint16_t vcc;
	uint16_t flag = 0;
	struct sff_trasnceiver_codes_byte4 *trasn_code_byte4;
	struct sff_trasnceiver_codes_byte5 *trasn_code_byte5;

	desc->tag = ocs_htobe32(RDP_SFP_DESC_TAG);

	trasn_code_byte4 = (struct sff_trasnceiver_codes_byte4 *)
		&page_a0[SSF_TRANSCEIVER_CODE_B4];
	trasn_code_byte5 = (struct sff_trasnceiver_codes_byte5 *)
		&page_a0[SSF_TRANSCEIVER_CODE_B5];

	if ((trasn_code_byte4->fc_sw_laser) ||
	    (trasn_code_byte5->fc_sw_laser_sl) ||
	    (trasn_code_byte5->fc_sw_laser_sn)) {  /* check if its short WL */
		flag |= (SFP_FLAG_PT_SWLASER << SFP_FLAG_PT_SHIFT);
	} else if (trasn_code_byte4->fc_lw_laser) {
		wavelength = (page_a0[SSF_WAVELENGTH_B1] << 8) |
			      page_a0[SSF_WAVELENGTH_B0];
		if (wavelength == SFP_WAVELENGTH_LC1310)
			flag |= SFP_FLAG_PT_LWLASER_LC1310 << SFP_FLAG_PT_SHIFT;
		if (wavelength == SFP_WAVELENGTH_LL1550)
			flag |= SFP_FLAG_PT_LWLASER_LL1550 << SFP_FLAG_PT_SHIFT;
	}

	/* check if its SFP+ */
	flag |= ((page_a0[SSF_IDENTIFIER] == SFF_PG0_IDENT_SFP) ?
		 SFP_FLAG_CT_SFP_PLUS : SFP_FLAG_CT_UNKNOWN)
		 << SFP_FLAG_CT_SHIFT;

	/* check if its OPTICAL */
	flag |= ((page_a0[SSF_CONNECTOR] == SFF_PG0_CONNECTOR_LC) ?
		 SFP_FLAG_IS_OPTICAL_PORT : 0) << SFP_FLAG_IS_OPTICAL_SHIFT;

	temperature = (page_a2[SFF_TEMPERATURE_B1] << 8 |
		       page_a2[SFF_TEMPERATURE_B0]);
	vcc = (page_a2[SFF_VCC_B1] << 8 |
	       page_a2[SFF_VCC_B0]);
	tx_power = (page_a2[SFF_TXPOWER_B1] << 8 |
		    page_a2[SFF_TXPOWER_B0]);
	tx_bias = (page_a2[SFF_TX_BIAS_CURRENT_B1] << 8 |
		   page_a2[SFF_TX_BIAS_CURRENT_B0]);
	rx_power = (page_a2[SFF_RXPOWER_B1] << 8 |
		    page_a2[SFF_RXPOWER_B0]);
	desc->sfp_info.temperature = ocs_htobe16(temperature);
	desc->sfp_info.rx_power = ocs_htobe16(rx_power);
	desc->sfp_info.tx_bias = ocs_htobe16(tx_bias);
	desc->sfp_info.tx_power = ocs_htobe16(tx_power);
	desc->sfp_info.vcc = ocs_htobe16(vcc);

	desc->sfp_info.flags = ocs_htobe16(flag);
	desc->length = ocs_htobe32(sizeof(desc->sfp_info));

	return sizeof(fc_rdp_sfp_desc_t);
}

static uint32_t
ocs_rdp_res_link_error(fc_rdp_link_error_status_desc_t *desc,
			ocs_xport_link_stats_t *stat)
{
	uint32_t type;

	desc->tag = ocs_htobe32(RDP_LINK_ERROR_STATUS_DESC_TAG);

	type = VN_PT_PHY_PF_PORT << VN_PT_PHY_SHIFT;

	desc->info.port_type = ocs_htobe32(type);

	desc->info.link_status.link_failure_cnt =
		ocs_htobe32(stat->link_failure_error_count);
	desc->info.link_status.loss_of_synch_cnt =
		ocs_htobe32(stat->loss_of_sync_error_count);
	desc->info.link_status.loss_of_signal_cnt =
		ocs_htobe32(stat->loss_of_signal_error_count);
	desc->info.link_status.primitive_seq_proto_err =
		ocs_htobe32(stat->primitive_sequence_error_count);
	desc->info.link_status.invalid_trans_word =
		ocs_htobe32(stat->invalid_transmission_word_error_count);
	desc->info.link_status.invalid_crc_cnt = ocs_htobe32(stat->crc_error_count);

	desc->length = ocs_htobe32(sizeof(desc->info));

	return sizeof(fc_rdp_link_error_status_desc_t);
}

static uint32_t
ocs_rdp_res_bbc_desc(fc_rdp_bbc_desc_t *desc, ocs_node_t *node)
{
	uint32_t bbcredit, value;

	desc->tag = ocs_htobe32(RDP_BBC_DESC_TAG);

	bbcredit = node->sport->service_params[2] |
		   (node->sport->service_params[3] << 8);
	desc->bbc_info.port_bbc = ocs_htobe32(bbcredit);

	ocs_hal_get(&(node->ocs->hal), OCS_HAL_TOPOLOGY, &value);
	if (value != OCS_HAL_TOPOLOGY_LOOP) {
		bbcredit = node->service_params[2] |
			   (node->service_params[3] << 8);
		desc->bbc_info.attached_port_bbc = ocs_htobe32(bbcredit);
	} else {
		desc->bbc_info.attached_port_bbc = 0;
	}

	desc->bbc_info.rtt = 0;
	desc->length = ocs_htobe32(sizeof(desc->bbc_info));

	return sizeof(fc_rdp_bbc_desc_t);
}

static uint32_t
ocs_rdp_res_oed_temp_desc(ocs_t *ocs, fc_rdp_oed_sfp_desc_t *desc,
			  ocs_rdp_context_t *rdp_context)
{
	uint32_t flags = 0;

	desc->tag = ocs_htobe32(RDP_OED_DESC_TAG);

	desc->oed_info.hi_alarm = rdp_context->page_a2[SSF_TEMP_HIGH_ALARM];
	desc->oed_info.lo_alarm = rdp_context->page_a2[SSF_TEMP_LOW_ALARM];
	desc->oed_info.hi_warning = rdp_context->page_a2[SSF_TEMP_HIGH_WARNING];
	desc->oed_info.lo_warning = rdp_context->page_a2[SSF_TEMP_LOW_WARNING];

	if (rdp_context->sfp_alarm & OCS_TRANSGRESSION_HIGH_TEMPERATURE)
		flags |= RDP_OET_HIGH_ALARM;
	if (rdp_context->sfp_alarm & OCS_TRANSGRESSION_LOW_TEMPERATURE)
		flags |= RDP_OET_LOW_ALARM;
	if (rdp_context->sfp_warning & OCS_TRANSGRESSION_HIGH_TEMPERATURE)
		flags |= RDP_OET_HIGH_WARNING;
	if (rdp_context->sfp_warning & OCS_TRANSGRESSION_LOW_TEMPERATURE)
		flags |= RDP_OET_LOW_WARNING;

	flags |= ((0xf & RDP_OED_TEMPERATURE) << RDP_OED_TYPE_SHIFT);
	desc->oed_info.function_flags = ocs_htobe32(flags);
	desc->length = ocs_htobe32(sizeof(desc->oed_info));

	return sizeof(fc_rdp_oed_sfp_desc_t);
}

static uint32_t
ocs_rdp_res_oed_voltage_desc(ocs_t *ocs, fc_rdp_oed_sfp_desc_t *desc,
			     ocs_rdp_context_t *rdp_context)
{
	uint32_t flags = 0;

	desc->tag = ocs_htobe32(RDP_OED_DESC_TAG);

	desc->oed_info.hi_alarm = rdp_context->page_a2[SSF_VOLTAGE_HIGH_ALARM];
	desc->oed_info.lo_alarm = rdp_context->page_a2[SSF_VOLTAGE_LOW_ALARM];
	desc->oed_info.hi_warning = rdp_context->page_a2[SSF_VOLTAGE_HIGH_WARNING];
	desc->oed_info.lo_warning = rdp_context->page_a2[SSF_VOLTAGE_LOW_WARNING];

	if (rdp_context->sfp_alarm & OCS_TRANSGRESSION_HIGH_VOLTAGE)
		flags |= RDP_OET_HIGH_ALARM;
	if (rdp_context->sfp_alarm & OCS_TRANSGRESSION_LOW_VOLTAGE)
		flags |= RDP_OET_LOW_ALARM;
	if (rdp_context->sfp_warning & OCS_TRANSGRESSION_HIGH_VOLTAGE)
		flags |= RDP_OET_HIGH_WARNING;
	if (rdp_context->sfp_warning & OCS_TRANSGRESSION_LOW_VOLTAGE)
		flags |= RDP_OET_LOW_WARNING;

	flags |= ((0xf & RDP_OED_VOLTAGE) << RDP_OED_TYPE_SHIFT);
	desc->oed_info.function_flags = ocs_htobe32(flags);
	desc->length = ocs_htobe32(sizeof(desc->oed_info));

	return sizeof(fc_rdp_oed_sfp_desc_t);
}

static uint32_t
ocs_rdp_res_oed_txbias_desc(ocs_t *ocs, fc_rdp_oed_sfp_desc_t *desc,
			    ocs_rdp_context_t *rdp_context)
{
	uint32_t flags = 0;

	desc->tag = ocs_htobe32(RDP_OED_DESC_TAG);

	desc->oed_info.hi_alarm = rdp_context->page_a2[SSF_BIAS_HIGH_ALARM];
	desc->oed_info.lo_alarm = rdp_context->page_a2[SSF_BIAS_LOW_ALARM];
	desc->oed_info.hi_warning = rdp_context->page_a2[SSF_BIAS_HIGH_WARNING];
	desc->oed_info.lo_warning = rdp_context->page_a2[SSF_BIAS_LOW_WARNING];

	if (rdp_context->sfp_alarm & OCS_TRANSGRESSION_HIGH_TXBIAS)
		flags |= RDP_OET_HIGH_ALARM;
	if (rdp_context->sfp_alarm & OCS_TRANSGRESSION_LOW_TXBIAS)
		flags |= RDP_OET_LOW_ALARM;
	if (rdp_context->sfp_warning & OCS_TRANSGRESSION_HIGH_TXBIAS)
		flags |= RDP_OET_HIGH_WARNING;
	if (rdp_context->sfp_warning & OCS_TRANSGRESSION_LOW_TXBIAS)
		flags |= RDP_OET_LOW_WARNING;

	flags |= ((0xf & RDP_OED_TXBIAS) << RDP_OED_TYPE_SHIFT);
	desc->oed_info.function_flags = ocs_htobe32(flags);
	desc->length = ocs_htobe32(sizeof(desc->oed_info));

	return sizeof(fc_rdp_oed_sfp_desc_t);
}

static uint32_t
ocs_rdp_res_oed_txpower_desc(ocs_t *ocs, fc_rdp_oed_sfp_desc_t *desc,
			     ocs_rdp_context_t *rdp_context)
{
	uint32_t flags = 0;

	desc->tag = ocs_htobe32(RDP_OED_DESC_TAG);

	desc->oed_info.hi_alarm = rdp_context->page_a2[SSF_TXPOWER_HIGH_ALARM];
	desc->oed_info.lo_alarm = rdp_context->page_a2[SSF_TXPOWER_LOW_ALARM];
	desc->oed_info.hi_warning = rdp_context->page_a2[SSF_TXPOWER_HIGH_WARNING];
	desc->oed_info.lo_warning = rdp_context->page_a2[SSF_TXPOWER_LOW_WARNING];

	if (rdp_context->sfp_alarm & OCS_TRANSGRESSION_HIGH_TXPOWER)
		flags |= RDP_OET_HIGH_ALARM;
	if (rdp_context->sfp_alarm & OCS_TRANSGRESSION_LOW_TXPOWER)
		flags |= RDP_OET_LOW_ALARM;
	if (rdp_context->sfp_warning & OCS_TRANSGRESSION_HIGH_TXPOWER)
		flags |= RDP_OET_HIGH_WARNING;
	if (rdp_context->sfp_warning & OCS_TRANSGRESSION_LOW_TXPOWER)
		flags |= RDP_OET_LOW_WARNING;

	flags |= ((0xf & RDP_OED_TXPOWER) << RDP_OED_TYPE_SHIFT);
	desc->oed_info.function_flags = ocs_htobe32(flags);
	desc->length = ocs_htobe32(sizeof(desc->oed_info));

	return sizeof(fc_rdp_oed_sfp_desc_t);
}

static uint32_t
ocs_rdp_res_oed_rxpower_desc(ocs_t *ocs, fc_rdp_oed_sfp_desc_t *desc,
			     ocs_rdp_context_t *rdp_context)
{
	uint32_t flags = 0;

	desc->tag = ocs_htobe32(RDP_OED_DESC_TAG);

	desc->oed_info.hi_alarm = rdp_context->page_a2[SSF_RXPOWER_HIGH_ALARM];
	desc->oed_info.lo_alarm = rdp_context->page_a2[SSF_RXPOWER_LOW_ALARM];
	desc->oed_info.hi_warning = rdp_context->page_a2[SSF_RXPOWER_HIGH_WARNING];
	desc->oed_info.lo_warning = rdp_context->page_a2[SSF_RXPOWER_LOW_WARNING];

	if (rdp_context->sfp_alarm & OCS_TRANSGRESSION_HIGH_RXPOWER)
		flags |= RDP_OET_HIGH_ALARM;
	if (rdp_context->sfp_alarm & OCS_TRANSGRESSION_LOW_RXPOWER)
		flags |= RDP_OET_LOW_ALARM;
	if (rdp_context->sfp_warning & OCS_TRANSGRESSION_HIGH_RXPOWER)
		flags |= RDP_OET_HIGH_WARNING;
	if (rdp_context->sfp_warning & OCS_TRANSGRESSION_LOW_RXPOWER)
		flags |= RDP_OET_LOW_WARNING;

	flags |= ((0xf & RDP_OED_RXPOWER) << RDP_OED_TYPE_SHIFT);
	desc->oed_info.function_flags = ocs_htobe32(flags);
	desc->length = ocs_htobe32(sizeof(desc->oed_info));

	return sizeof(fc_rdp_oed_sfp_desc_t);
}

static uint32_t
ocs_rdp_res_opd_desc(fc_rdp_opd_sfp_desc_t *desc,
		uint8_t *page_a0)
{
	desc->tag = ocs_htobe32(RDP_OPD_DESC_TAG);
	ocs_memcpy(desc->opd_info.vendor_name, &page_a0[SSF_VENDOR_NAME], 16);
	ocs_memcpy(desc->opd_info.model_number, &page_a0[SSF_VENDOR_PN], 16);
	ocs_memcpy(desc->opd_info.serial_number, &page_a0[SSF_VENDOR_SN], 16);
	ocs_memcpy(desc->opd_info.revision, &page_a0[SSF_VENDOR_REV], 4);
	ocs_memcpy(desc->opd_info.date, &page_a0[SSF_DATE_CODE], 8);
	desc->length = ocs_htobe32(sizeof(desc->opd_info));

	return sizeof(fc_rdp_opd_sfp_desc_t);
}

static uint32_t
ocs_rdp_res_fec_desc(fc_fec_rdp_desc_t *desc, ocs_xport_link_stats_t *stat)
{
	desc->tag = ocs_htobe32(RDP_FEC_DESC_TAG);

	desc->info.CorrectedBlocks =
		ocs_htobe32(stat->fec_corrected_blocks_count);
	desc->info.UncorrectableBlocks =
		ocs_htobe32(stat->fec_uncorrectable_blocks_count);
	desc->length = ocs_htobe32(sizeof(desc->info));

	return sizeof(fc_fec_rdp_desc_t);
}

static uint32_t
ocs_rdp_res_speed(fc_rdp_port_speed_desc_t *desc, ocs_t *ocs)
{
	uint16_t rdp_cap = 0;
	uint16_t rdp_speed;
	uint32_t speed, lmt;

	desc->tag = ocs_htobe32(RDP_PORT_SPEED_DESC_TAG);

	ocs_hal_get(&ocs->hal, OCS_HAL_LINK_SPEED, &speed);
	switch (speed) {
	case FC_LINK_SPEED_1G:
		rdp_speed = RDP_PS_1GB;
		break;
	case FC_LINK_SPEED_2G:
		rdp_speed = RDP_PS_2GB;
		break;
	case FC_LINK_SPEED_4G:
		rdp_speed = RDP_PS_4GB;
		break;
	case FC_LINK_SPEED_8G:
		rdp_speed = RDP_PS_8GB;
		break;
	case FC_LINK_SPEED_10G:
		rdp_speed = RDP_PS_10GB;
		break;
	case FC_LINK_SPEED_16G:
		rdp_speed = RDP_PS_16GB;
		break;
	case FC_LINK_SPEED_32G:
		rdp_speed = RDP_PS_32GB;
		break;
	case FC_LINK_SPEED_64G:
		rdp_speed = RDP_PS_64GB;
		break;
	case FC_LINK_SPEED_128G:
		rdp_speed = RDP_PS_128GB;
		break;
	case FC_LINK_SPEED_256G:
		rdp_speed = RDP_PS_256GB;
		break;
	default:
		rdp_speed = RDP_PS_UNKNOWN;
		break;
	}

	desc->info.port_speed.speed = ocs_htobe16(rdp_speed);

	ocs_hal_get(&ocs->hal, OCS_HAL_LINK_MODULE_TYPE, &lmt);

	if (lmt & OCS_HAL_LINK_MODULE_TYPE_256GB)
		rdp_cap |= RDP_PS_256GB;
	if (lmt & OCS_HAL_LINK_MODULE_TYPE_128GB)
		rdp_cap |= RDP_PS_128GB;
	if (lmt & OCS_HAL_LINK_MODULE_TYPE_64GB)
		rdp_cap |= RDP_PS_64GB;
	if (lmt & OCS_HAL_LINK_MODULE_TYPE_32GB)
		rdp_cap |= RDP_PS_32GB;
	if (lmt & OCS_HAL_LINK_MODULE_TYPE_16GB)
		rdp_cap |= RDP_PS_16GB;
	if (lmt & OCS_HAL_LINK_MODULE_TYPE_10GB)
		rdp_cap |= RDP_PS_10GB;
	if (lmt & OCS_HAL_LINK_MODULE_TYPE_8GB)
		rdp_cap |= RDP_PS_8GB;
	if (lmt & OCS_HAL_LINK_MODULE_TYPE_4GB)
		rdp_cap |= RDP_PS_4GB;
	if (lmt & OCS_HAL_LINK_MODULE_TYPE_2GB)
		rdp_cap |= RDP_PS_2GB;
	if (lmt & OCS_HAL_LINK_MODULE_TYPE_1GB)
		rdp_cap |= RDP_PS_1GB;

	if (rdp_cap == 0)
		rdp_cap = RDP_CAP_UNKNOWN;

	desc->info.port_speed.capabilities = ocs_htobe16(rdp_cap);
	desc->length = ocs_htobe32(sizeof(desc->info));

	return sizeof(fc_rdp_port_speed_desc_t);
}

static uint32_t
ocs_rdp_res_diag_port_names(fc_rdp_port_name_desc_t *desc,
			ocs_node_t *node)
{
	desc->tag = ocs_htobe32(RDP_PORT_NAMES_DESC_TAG);

	ocs_memcpy(desc->port_names.wwnn, &node->sport->sli_wwnn,
				sizeof(desc->port_names.wwnn));

	ocs_memcpy(desc->port_names.wwpn, &node->sport->sli_wwpn,
				sizeof(desc->port_names.wwpn));
	desc->length = ocs_htobe32(sizeof(desc->port_names));

	return sizeof(fc_rdp_port_name_desc_t);
}

static uint32_t
ocs_rdp_res_attach_port_names(fc_rdp_port_name_desc_t *desc,
			ocs_node_t *node)
{
	uint64_t wwpn, wwnn;

	desc->tag = ocs_htobe32(RDP_PORT_NAMES_DESC_TAG);
	wwnn = ocs_node_get_wwnn(node);
	wwpn = ocs_node_get_wwpn(node);

	ocs_memcpy(desc->port_names.wwnn, &wwnn, sizeof(desc->port_names.wwnn));
	ocs_memcpy(desc->port_names.wwpn, &wwpn, sizeof(desc->port_names.wwpn));
	desc->length = ocs_htobe32(sizeof(desc->port_names));

	return sizeof(fc_rdp_port_name_desc_t);
}

void
ocs_send_rdp_resp(ocs_node_t *node, ocs_rdp_context_t *rdp_context, int status)
{
	ocs_io_t *io = rdp_context->io;
	fc_rdp_res_frame_t *rdp = io->els_info->els_req.virt;
	uint8_t *pcmd;
	uint32_t len;
	uint16_t *flag_ptr;
	ocs_t	*ocs = node->ocs;
	ocs_xport_stats_t stats;
	uint32_t rjt_err = FC_REASON_UNABLE_TO_PERFORM;
	uint32_t rjt_expl = FC_EXPL_NO_ADDITIONAL;

	if (rdp_context->status || status) {
		if (status == OCS_RDP_RJT_NO_LOGIN)
			rjt_expl = FC_EXPL_NPORT_LOGIN_REQUIRED;
		goto error;
	}

	rdp->reply_sequence = FC_ELS_CMD_ACC;

	pcmd = (uint8_t *) io->els_info->els_req.virt;

	if ((ocs_xport_status(ocs->xport, OCS_XPORT_LINK_STATISTICS, &stats)) != 0) {
		ocs_log_err(ocs, "Getting Linkstats failed\n");
		goto error;
	}

	/* Update Alarm and Warning */
	flag_ptr = (uint16_t *)(rdp_context->page_a2 + SSF_ALARM_FLAGS);
	rdp_context->sfp_alarm |= *flag_ptr;
	flag_ptr = (uint16_t *)(rdp_context->page_a2 + SSF_WARNING_FLAGS);
	rdp_context->sfp_warning |= *flag_ptr;

	/* For RDP payload */
	len = 8;
	len += ocs_rdp_res_link_service((fc_rdp_link_service_desc_t *)
					(len + pcmd), FC_ELS_CMD_RDP);
	len += ocs_rdp_res_sfp_desc((fc_rdp_sfp_desc_t *)(len + pcmd),
				rdp_context->page_a0, rdp_context->page_a2);
	len += ocs_rdp_res_speed((fc_rdp_port_speed_desc_t *)(len + pcmd),
				 ocs);
	len += ocs_rdp_res_link_error((fc_rdp_link_error_status_desc_t *)
				(len + pcmd), &stats.stats.link_stats);
	len += ocs_rdp_res_diag_port_names((fc_rdp_port_name_desc_t *)
						(len + pcmd), node);
	len += ocs_rdp_res_attach_port_names((fc_rdp_port_name_desc_t *)
						(len + pcmd), node);
	len += ocs_rdp_res_fec_desc((fc_fec_rdp_desc_t *)(len + pcmd),
					&stats.stats.link_stats);
	len += ocs_rdp_res_bbc_desc((fc_rdp_bbc_desc_t *)(len + pcmd),
				    node);
	len += ocs_rdp_res_oed_temp_desc(ocs,
			(fc_rdp_oed_sfp_desc_t *)(len + pcmd), rdp_context);
	len += ocs_rdp_res_oed_voltage_desc(ocs,
			(fc_rdp_oed_sfp_desc_t *)(len + pcmd), rdp_context);
	len += ocs_rdp_res_oed_txbias_desc(ocs,
			(fc_rdp_oed_sfp_desc_t *)(len + pcmd), rdp_context);
	len += ocs_rdp_res_oed_txpower_desc(ocs,
			(fc_rdp_oed_sfp_desc_t *)(len + pcmd), rdp_context);
	len += ocs_rdp_res_oed_rxpower_desc(ocs,
			(fc_rdp_oed_sfp_desc_t *)(len + pcmd), rdp_context);
	len += ocs_rdp_res_opd_desc((fc_rdp_opd_sfp_desc_t *)(len + pcmd),
					rdp_context->page_a0);

	rdp->length = ocs_htobe32(len - 8);

	io->els_info->els_callback = NULL;
	io->els_info->els_callback_arg = NULL;
	io->display_name = "rdp_acc";
	io->init_task_tag = rdp_context->ox_id;

	/* Go ahead and send the ELS_ACC */
	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.els.ox_id = rdp_context->ox_id;
	io->hio_type = OCS_HAL_ELS_RSP;
	if ((ocs_els_send_rsp(io, sizeof(*rdp)))) {
		ocs_els_io_free(io);
		io = NULL;
	}
	ocs_free(ocs, rdp_context, sizeof(*rdp_context));

	return;
error:
	ocs_send_ls_rjt(io, rdp_context->ox_id, rjt_err, rjt_expl,
			0, NULL, NULL);
	ocs_free(ocs, rdp_context, sizeof(*rdp_context));
	return;
}

static void
ocs_get_rdp_info_cb(ocs_t *ocs, ocs_rdp_context_t *rdp_context)
{
	ocs_domain_t *domain;
	ocs_node_t *node;
	ocs_sport_t *sport;
	ocs_xport_fcfi_t *xport_fcfi = NULL;

	xport_fcfi = &ocs->xport->fcfi[rdp_context->seq_fcfi];
	if (xport_fcfi == NULL) {
		ocs_log_err(ocs, "FCFI %d is not valid, dropping frame\n",
				 rdp_context->seq_fcfi);
		goto error;
	}

	domain = ocs_hal_domain_get(&ocs->hal, rdp_context->seq_fcfi);
	if (!domain || xport_fcfi->hold_frames){
		ocs_log_err(ocs, "Domain is not valid, dropping frame\n");
		goto error;
	}

	sport = ocs_sport_find(domain, rdp_context->d_id);
	if (sport == NULL) {
		ocs_log_err(ocs, "Sport is NULL, dropping frame\n");
		goto error;
	}

	node = ocs_node_find(sport, rdp_context->s_id);
	if (node == NULL) {
		ocs_log_err(ocs, "Node is NULL, dropping frame\n");
		goto error;
	}

	if (node->hold_frames || !ocs_scsi_io_alloc_enabled(node)) {
		ocs_log_err(ocs, "Hold frames set, dropping frame\n");
		goto error;
	}

	rdp_context->io = ocs_els_io_alloc(node, sizeof(fc_rdp_res_frame_t),
				     OCS_ELS_ROLE_RESPONDER);
	if (!rdp_context->io) {
		ocs_log_err(ocs, "IO alloc failed, dropping frame\n");
		goto error;
	}

	/* if we're here, sequence initiative has been transferred */
	rdp_context->io->seq_init = 1;

	ocs_node_post_event(node, OCS_EVT_RDP_RCVD, rdp_context);

	return;
error:
	ocs_free(ocs, rdp_context, sizeof(*rdp_context));
}

static void
ocs_rdp_sfp_a2_cb(void *os, int32_t status, uint32_t bytes_read, uint32_t *data, void *arg)
{
	ocs_rdp_context_t *rdp_context = arg;
	ocs_t *ocs = os;
	int rc = 0;

	if (status) {
		ocs_log_err(ocs, "sfp page_a2 failed with status %d\n", status);
		rc = 1;
		goto error;
	}

	ocs_memcpy(&rdp_context->page_a2, data, SFP_PAGE_SIZE);
error:
	rdp_context->status = rc;
	ocs_get_rdp_info_cb(ocs, rdp_context);
}

static void
ocs_rdp_sfp_a0_cb(void *os, int32_t status, uint32_t bytes_read, uint32_t *data, void *arg)
{
	ocs_rdp_context_t *rdp_context = arg;
	ocs_t *ocs = os;
	int32_t rc;

	if (status) {
		ocs_log_err(ocs, "sfp page_a0 failed with status%d\n", status);
		goto error;
	}

	ocs_memcpy(&rdp_context->page_a0, data, SFP_PAGE_SIZE);

	rc = ocs_hal_get_sfp(&ocs->hal, SFP_PAGE_A2, ocs_rdp_sfp_a2_cb, rdp_context);
	if (rc) {
		ocs_log_err(ocs, "ocs_hal_get_sfp for A2 failed %d\n", rc);
		goto error;
	}

	return;
error:
	rdp_context->status = 1;
	ocs_get_rdp_info_cb(ocs, rdp_context);
}

/**
 * @brief Process an unsolicited RDP ELS.
 *
 * <h3 class="desc">Description</h3>
 * This routine processes an unsolicited RDP(Read Diagnostic Parameters).
 * 1) Get SFP Page A0 data.
 * 2) Get SFP Page A2 data.
 * 3) Get cached Link Stats.
 * 4) Populate all data and send RDP response.
 *
 * @param io Pointer to a SCSI IO object.
 * @param hdr Pointer to the FC header.
 * @param req Pointer to the FC request payload.
 */

void ocs_rdp_defer_response(ocs_t *ocs, fc_header_t *hdr, uint32_t fcfi)
{
	ocs_rdp_context_t *rdp_context;
	int rc = 0;

	rdp_context = ocs_malloc(ocs, sizeof(ocs_rdp_context_t),
				 OCS_M_ZERO | OCS_M_NOWAIT);
	if(!rdp_context) {
		ocs_log_err(ocs, "rdp_context allocation failed\n");
		return;
	}

	rdp_context->ox_id = ocs_be16toh(hdr->ox_id);
	rdp_context->rx_id = ocs_be16toh(hdr->rx_id);
	rdp_context->s_id = fc_be24toh(hdr->s_id);
	rdp_context->d_id = fc_be24toh(hdr->d_id);
	rdp_context->seq_fcfi = fcfi;

	rc = ocs_hal_get_sfp(&ocs->hal, SFP_PAGE_A0, ocs_rdp_sfp_a0_cb,
			     rdp_context);
	if (rc) {
		ocs_log_err(ocs, "ocs_hal_get_sfp for A0 failed %d\n", rc);
		goto error;
	}

	return;
error:
	rdp_context->status = 1;
	ocs_get_rdp_info_cb(ocs, rdp_context);
}

/**
 * @ingroup els_api
 * @brief Send a BA_ACC given the request's FC header
 *
 * <h3 class="desc">Description</h3>
 * Using the S_ID/D_ID from the request's FC header, generate a BA_ACC.
 *
 * @param io Pointer to a SCSI IO object.
 * @param hdr Pointer to the FC header.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_bls_send_acc_hdr(ocs_io_t *io, fc_header_t *hdr)
{
	uint16_t ox_id = ocs_be16toh(hdr->ox_id);
	uint16_t rx_id = ocs_be16toh(hdr->rx_id);
	uint32_t d_id = fc_be24toh(hdr->d_id);

	return ocs_bls_send_acc(io, d_id, ox_id, rx_id);
}

/**
 * @ingroup els_api
 * @brief Send a BLS BA_ACC response.
 *
 * <h3 class="desc">Description</h3>
 * Construct a BLS BA_ACC response, and send to the \c node.
 *
 * @param io Pointer to a SCSI IO object.
 * @param s_id S_ID to use for the response. If UINT32_MAX, then use our SLI port
 * (sport) S_ID.
 * @param ox_id Originator exchange ID.
 * @param rx_id Responder exchange ID.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

static ocs_io_t *
ocs_bls_send_acc(ocs_io_t *io, uint32_t s_id, uint16_t ox_id, uint16_t rx_id)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	fc_ba_acc_payload_t *acc;
	ocs_t *ocs;

	ocs_assert(node, NULL);
	ocs_assert(node->ocs, NULL);
	ocs = node->ocs;

	if (node->rnode.sport->fc_id == s_id) {
		s_id = UINT32_MAX;
	}

	/* fill out generic fields */
	io->ocs = ocs;
	io->node = node;
	io->cmd_tgt = TRUE;

	/* fill out BLS Response-specific fields */
	io->io_type = OCS_IO_TYPE_BLS_RESP;
	io->display_name = "ba_acc";
	io->hio_type = OCS_HAL_BLS_ACC_SID;
	io->init_task_tag = ox_id;

	/* fill out iparam fields */
	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.bls_sid.s_id = s_id;
	io->iparam.bls_sid.ox_id = ox_id;
	io->iparam.bls_sid.rx_id = rx_id;

	acc = (void *)io->iparam.bls_sid.payload;

	ocs_memset(io->iparam.bls_sid.payload, 0, sizeof(io->iparam.bls_sid.payload));
	acc->ox_id = io->iparam.bls_sid.ox_id;
	acc->rx_id = io->iparam.bls_sid.rx_id;
	acc->high_seq_cnt = UINT16_MAX;

	if ((rc = ocs_scsi_io_dispatch(io, ocs_bls_send_acc_cb))) {
		ocs_log_err(ocs, "ocs_scsi_io_dispatch() failed: %d\n", rc);
		ocs_scsi_io_free(io);
		io = NULL;
	}
	return io;
}

/**
 * @ingroup els_api
 * @brief Send a BA_RJT given the request's FC header
 *
 * <h3 class="desc">Description</h3>
 * Using the S_ID/D_ID from the request's FC header, generate a BA_RJT.
 *
 * @param io Pointer to a SCSI IO object.
 * @param hdr Pointer to the FC header.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

ocs_io_t *
ocs_bls_send_rjt_hdr(ocs_io_t *io, fc_header_t *hdr)
{
	uint16_t ox_id = ocs_be16toh(hdr->ox_id);
	uint16_t rx_id = ocs_be16toh(hdr->rx_id);
	uint32_t d_id = fc_be24toh(hdr->d_id);

	return ocs_bls_send_rjt(io, d_id, ox_id, rx_id);
}

/**
 * @ingroup els_api
 * @brief Send a BLS BA_RJT response.
 *
 * <h3 class="desc">Description</h3>
 * Construct a BLS BA_RJT response, and send to the \c node.
 *
 * @param io Pointer to a SCSI IO object.
 * @param s_id S_ID to use for the response. If UINT32_MAX, then use our SLI port
 * (sport) S_ID.
 * @param ox_id Originator exchange ID.
 * @param rx_id Responder exchange ID.
 *
 * @return Returns pointer to IO object, or NULL if error.
 */

static ocs_io_t *
ocs_bls_send_rjt(ocs_io_t *io, uint32_t s_id, uint16_t ox_id, uint16_t rx_id)
{
	ocs_node_t *node = io->node;
	int32_t rc;
	fc_ba_rjt_payload_t *acc;
	ocs_t *ocs;

	ocs_assert(node, NULL);
	ocs_assert(node->ocs, NULL);
	ocs = node->ocs;

	if (node->rnode.sport->fc_id == s_id) {
		s_id = UINT32_MAX;
	}

	/* fill out generic fields */
	io->ocs = ocs;
	io->node = node;
	io->cmd_tgt = TRUE;

	/* fill out BLS Response-specific fields */
	io->io_type = OCS_IO_TYPE_BLS_RESP;
	io->display_name = "ba_rjt";
	io->hio_type = OCS_HAL_BLS_RJT;
	io->init_task_tag = ox_id;

	/* fill out iparam fields */
	ocs_memset(&io->iparam, 0, sizeof(io->iparam));
	io->iparam.bls.ox_id = ox_id;
	io->iparam.bls.rx_id = rx_id;

	acc = (void *)io->iparam.bls.payload;

	ocs_memset(io->iparam.bls.payload, 0, sizeof(io->iparam.bls.payload));
	acc->reason_code	= FC_REASON_UNABLE_TO_PERFORM;
	acc->reason_explanation = FC_EXPL_NO_ADDITIONAL;

	if ((rc = ocs_scsi_io_dispatch(io, ocs_bls_send_rjt_cb))) {
		ocs_log_err(ocs, "ocs_scsi_io_dispatch() failed: %d\n", rc);
		ocs_scsi_io_free(io);
		io = NULL;
	}
	return io;
}

/**
 * @brief Handle the BLS accept completion.
 *
 * <h3 class="desc">Description</h3>
 * Upon completion of sending a BA_ACC, this callback is invoked by the HAL.
 *
 * @param hio Pointer to the HAL IO object.
 * @param rnode Pointer to the HAL remote node.
 * @param length Length of the response payload, in bytes.
 * @param status Completion status.
 * @param ext_status Extended completion status.
 * @param app Callback private argument.
 *
 * @return Returns 0 on success; or a negative error value on failure.
 */

static int32_t
ocs_bls_send_acc_cb(ocs_hal_io_t *hio, ocs_remote_node_t *rnode, uint32_t length, int32_t status, uint32_t ext_status, void *app)
{
	ocs_io_t *io = app;

	ocs_assert(io, -1);

	ocs_scsi_io_free(io);
	return 0;
}

/**
 * @brief Handle the BLS reject completion.
 *
 * <h3 class="desc">Description</h3>
 * Upon completion of sending a BA_RJT, this callback is invoked by the HAL.
 *
 * @param hio Pointer to the HAL IO object.
 * @param rnode Pointer to the HAL remote node.
 * @param length Length of the response payload, in bytes.
 * @param status Completion status.
 * @param ext_status Extended completion status.
 * @param app Callback private argument.
 *
 * @return Returns 0 on success; or a negative error value on failure.
 */

static int32_t
ocs_bls_send_rjt_cb(ocs_hal_io_t *hio, ocs_remote_node_t *rnode, uint32_t length, int32_t status, uint32_t ext_status, void *app)
{
	ocs_io_t *io = app;

	ocs_assert(io, -1);

	ocs_scsi_io_free(io);
	return 0;
}

/**
 * @brief ELS abort callback.
 *
 * <h3 class="desc">Description</h3>
 * This callback is invoked by the HAL when an ELS IO is aborted.
 *
 * @param hio Pointer to the HAL IO object.
 * @param rnode Pointer to the HAL remote node.
 * @param length Length of the response payload, in bytes.
 * @param status Completion status.
 * @param ext_status Extended completion status.
 * @param app Callback private argument.
 *
 * @return Returns 0 on success; or a negative error value on failure.
 */

static int32_t
ocs_els_abort_cb(ocs_hal_io_t *hio, ocs_remote_node_t *rnode, uint32_t length, int32_t status, uint32_t ext_status, void *app)
{
	ocs_t *ocs;
	ocs_io_t *els = app;

	ocs_assert(els, -1);
	ocs_assert(els->node, -1);
	ocs_assert(els->node->ocs, -1);

	ocs = els->node->ocs;
	if (status)
		ocs_log_err(ocs, "status x%x ext x%x\n", status, ext_status);

	/*
	 * Send the completion event to indicate that the abort process is complete.
	 * Note: The ELS SM will already be receiving ELS_REQ_OK/FAIL/RJT/ABORTED.
	 */
	ocs_els_post_event(els, OCS_EVT_ELS_ABORT_CMPL, NULL);

	/* Done with the original ELS IO */
	ocs_ref_put(&els->ref); /* ocs_ref_get(): ocs_els_abort_io() */

	return 0;
}

/**
 * @brief Abort an ELS IO.
 *
 * <h3 class="desc">Description</h3>
 * The ELS IO is aborted by making a HAL abort IO request,
 * optionally requesting that an ABTS is sent.
 *
 * \b Note: This function allocates a HAL IO, and associates the HAL IO
 * with the ELS IO that it is aborting. It does not associate
 * the HAL IO with the node directly, like for ELS requests. The
 * abort completion is propagated up to the node once the
 * original WQE and the abort WQE are complete (the original WQE
 * completion is not propagated up to node).
 *
 * @param els Pointer to the ELS IO.
 * @param send_abts Boolean to indicate if hardware will automatically generate an ABTS.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */

static int32_t
ocs_els_abort_io(ocs_io_t *els, bool send_abts)
{
	ocs_t *ocs;

	ocs_assert(els, -1);
	ocs_assert(els->node, -1);
	ocs_assert(els->node->ocs, -1);

	ocs = els->node->ocs;
	ocs_assert(ocs->xport, -1);

	if (!ocs_ref_get_unless_zero(&els->ref)) {
		ocs_log_err(ocs, "Command no longer active\n");
		return -1;
	}

	/* Dispatch abort */
	ocs_scsi_io_dispatch_abort(els, ocs_els_abort_cb, send_abts);
	return 0;
}

/*
 * ELS IO State Machine
 */

#define std_els_state_decl(...) \
	ocs_io_t *els = NULL; \
	ocs_node_t *node = NULL; \
	ocs_t *ocs = NULL; \
	ocs_assert(ctx != NULL, NULL); \
	els = ctx->app; \
	ocs_assert(els != NULL, NULL); \
	node = els->node; \
	ocs_assert(node != NULL, NULL); \
	ocs = node->ocs; \
	ocs_assert(ocs != NULL, NULL);

#define els_sm_trace(...) \
	do { \
		if (OCS_LOG_ENABLE_ELS_TRACE(ocs)) \
			ocs_log_info(ocs, "[%s] %-8s %-20s\n", node->display_name, \
				     els->display_name, ocs_sm_event_name(evt)); \
	} while (0)

/**
 * @brief Cleanup an ELS IO
 *
 * <h3 class="desc">Description</h3>
 * Cleans up an ELS IO by posting the requested event to the owning node object;
 * invoking the callback, if one is provided; and then freeing the
 * ELS IO object.
 *
 * @param els Pointer to the ELS IO.
 * @param node_evt Node SM event to post.
 * @param arg Node SM event argument.
 *
 * @return None.
 */

void
ocs_els_io_cleanup(ocs_io_t *els, ocs_sm_event_t node_evt, void *arg)
{
	ocs_assert(els);

	/* don't want further events that could come; e.g. abort requests
	 * from the node state machine; thus, disable state machine
	 */
	ocs_sm_disable(&els->els_info->els_sm);
	ocs_node_post_event(els->node, node_evt, arg);

	/* If this IO has a callback, invoke it */
	if (els->els_info->els_callback) {
		(*els->els_info->els_callback)(els->node, arg, els->els_info->els_callback_arg);
	}
	els->els_info->els_req_free = 1;
}


/**
 * @brief Common event handler for the ELS IO state machine.
 *
 * <h3 class="desc">Description</h3>
 * Provide handler for events for which default actions are desired.
 *
 * @param funcname Name of the calling function (for logging).
 * @param ctx Remote node SM context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_els_common(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_els_state_decl();

	switch(evt) {
	case OCS_EVT_ENTER:
	case OCS_EVT_REENTER:
	case OCS_EVT_EXIT:
		break;

	/* If ELS_REQ_FAIL is not handled in state, then we'll terminate this ELS and
	 * pass the event to the node
	 */
	case OCS_EVT_SRRS_ELS_REQ_FAIL:
		ocs_log_warn(els->node->ocs, "[%s] %-20s %-20s not handled - terminating ELS\n", node->display_name, funcname,
			ocs_sm_event_name(evt));
		ocs_els_io_cleanup(els, OCS_EVT_SRRS_ELS_REQ_FAIL, arg);
		break;
	default:
		ocs_log_warn(els->node->ocs, "[%s] %-20s %-20s not handled\n", node->display_name, funcname,
			ocs_sm_event_name(evt));
		break;
	}
	return NULL;
}

/**
 * @brief Initial ELS IO state
 *
 * <h3 class="desc">Description</h3>
 * This is the initial ELS IO state. Upon entry, the requested ELS/CT is submitted to
 * the hardware.
 *
 * @param ctx Remote node SM context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_els_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc = 0;
	std_els_state_decl();

	els_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER: {
		rc = ocs_els_send(els, els->els_info->els_req.size, els->els_info->els_timeout_sec, ocs_els_req_cb);
		if (rc) {
			ocs_node_cb_t cbdata;
			cbdata.status = cbdata.ext_status = (~0);
			cbdata.els = els;
			ocs_log_err(ocs, "ocs_els_send failed: %d\n", rc);
			ocs_els_io_cleanup(els, OCS_EVT_SRRS_ELS_REQ_FAIL, &cbdata);
		} else {
			ocs_io_transition(els, __ocs_els_wait_resp, NULL);
		}
		break;
	}
	default:
		__ocs_els_common(__func__, ctx, evt, arg);
		break;
	}

	return NULL;
}

/**
 * @brief Wait for the ELS request to complete.
 *
 * <h3 class="desc">Description</h3>
 * This is the ELS IO state that waits for the submitted ELS event to complete.
 * If an error completion event is received, the requested ELS is aborted.
 *
 * @param ctx Remote node SM context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_els_wait_resp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_els_state_decl();

	els_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK: {
		ocs_els_io_cleanup(els, OCS_EVT_SRRS_ELS_REQ_OK, arg);
		break;
	}

	case OCS_EVT_SRRS_ELS_REQ_FAIL: {
		ocs_els_io_cleanup(els, OCS_EVT_SRRS_ELS_REQ_FAIL, arg);
		break;
	}

	case OCS_EVT_ELS_REQ_TIMEOUT: {
		els_io_printf(els, "Timed out, retry (%d tries remaining)\n",
			      els->els_info->els_retries_remaining-1);
		ocs_io_transition(els, __ocs_els_retry, NULL);
		break;
	}

	case OCS_EVT_SRRS_ELS_REQ_RJT: {
		ocs_node_cb_t *cbdata = arg;
		uint8_t reason_code = (cbdata->ext_status >> 16) & 0xFF;
		uint8_t reason_code_expl = (cbdata->ext_status >> 8) & 0xFF;
		fc_els_gen_t *buf = els->els_info->els_req.virt;

		/* delay and retry if reason code is Logical Busy */
		switch (reason_code) {
		case FC_REASON_LOGICAL_BUSY:
			els->node->els_req_cnt--;
			els_io_printf(els, "LS_RJT Logical Busy response, delay and retry\n");
			ocs_io_transition(els, __ocs_els_delay_retry, NULL);
			break;
		case FC_REASON_LOGICAL_ERROR:
			if (FC_ELS_CMD_FLOGI == buf->command_code && FC_EXPL_NO_ADDITIONAL == reason_code_expl) {
				/**
				 * Disable BBCR by setting the BB_SC_N field in the Common Service
				 * Parameters to zero while retrying the FLOGI. Keep the BB_SC_N field
				 * set to zero for any additional retries while the link remains up.
				 * This is to work around a bug in the older Brocade switch firmware
				 * (FOS versions prior to 7.1.0). During subsequent linkup processing,
				 * if BBCR is enabled, attempt to enable BBCR again.
				 */
				fc_plogi_payload_t *flogi = els->els_info->els_req.virt;

				/* Zero-out the BB_SC_N field (word 1, bits 15:12) */
				flogi->common_service_parameters[1] &= ocs_htobe32(FC_PLOGI_CSP_W1_BBSCN_CLEAR_MASK);
				els_io_printf(els, "BBCR disabled until the next link toggle\n");
			}

			/* For now, fall through since we are not retrying this ELS request */
			FALL_THROUGH;
		default:
			ocs_els_io_cleanup(els, evt, arg);
			break;
		}
		break;
	}

	case OCS_EVT_ABORT_ELS: {
		int32_t rc = 0;

		/* request to abort this ELS without an ABTS */
		els_io_printf(els, "ELS abort requested\n");
		els->els_info->els_retries_remaining = 0;		/* Set retries to zero, we are done */
		rc = ocs_els_abort_io(els, FALSE);
		if (rc) {
			ocs_log_err(ocs, "ocs_els_abort_io() failed\n");
			ocs_els_io_cleanup(els, OCS_EVT_SRRS_ELS_REQ_FAIL, arg);
		} else {
			ocs_io_transition(els, __ocs_els_aborting, NULL);
		}
		break;
	}

	default:
		__ocs_els_common(__func__, ctx, evt, arg);
		break;
	}
	return NULL;
}

/**
 * @brief Wait for the ELS IO abort request to complete, and retry the ELS.
 *
 * <h3 class="desc">Description</h3>
 * This state is entered when waiting for an abort of an ELS
 * request to complete so the request can be retried.
 *
 * @param ctx Remote node SM context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_els_retry(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc = 0;
	std_els_state_decl();

	els_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER: {
		/* handle event for ABORT_XRI WQE
		 * once abort is complete, retry if retries left;
		 * don't need to wait for OCS_EVT_SRRS_ELS_REQ_* event because we got
		 * by receiving OCS_EVT_ELS_REQ_TIMEOUT
		 */
		ocs_node_cb_t node_cbdata;
		node_cbdata.status = node_cbdata.ext_status = (~0);
		node_cbdata.els = els;
		if (els->els_info->els_retries_remaining && --els->els_info->els_retries_remaining) {
			/* Use a different XRI for the retry (would like a new oxid),
			 * so free the HAL IO (dispatch will allocate a new one). It's an
			 * optimization to only free the HAL IO here and not the ocs_io_t;
			 * Freeing the ocs_io_t object would require copying all the necessary
			 * info from the old ocs_io_t object to the * new one; and allocating
			 * a new ocs_io_t could fail.
			 */
			ocs_assert(els->hio, NULL);
			ocs_hal_io_free(&ocs->hal, els->hio);
			els->hio = NULL;

			/* result isn't propagated up to node sm, need to decrement req cnt */
			ocs_assert(els->node->els_req_cnt, NULL);
			els->node->els_req_cnt--;
			rc = ocs_els_send(els, els->els_info->els_req.size, els->els_info->els_timeout_sec, ocs_els_req_cb);
			if (rc) {
				ocs_log_err(ocs, "ocs_els_send failed: %d\n", rc);
				ocs_els_io_cleanup(els, OCS_EVT_SRRS_ELS_REQ_FAIL, &node_cbdata);
			}
			ocs_io_transition(els, __ocs_els_wait_resp, NULL);
		} else {
			els_io_printf(els, "Retries exhausted\n");
			ocs_els_io_cleanup(els, OCS_EVT_SRRS_ELS_REQ_FAIL, &node_cbdata);
		}
		break;
	}

	default:
		__ocs_els_common(__func__, ctx, evt, arg);
		break;
	}
	return NULL;
}

/**
 * @brief Wait for a retry timer to expire having received an abort request
 *
 * <h3 class="desc">Description</h3>
 * This state is entered when waiting for a timer event, after having received
 * an abort request, to avoid a race condition with the timer handler
 *
 * @param ctx Remote node SM context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_els_aborted_delay_retry(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_els_state_decl();

	els_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		/* mod/resched the timer for a short duration */
		ocs_mod_timer(&els->delay_timer, 1);
		break;
	case OCS_EVT_TIMER_EXPIRED:
		/* Cancel the timer, skip post node event, and free the io */
		node->els_req_cnt++;
		ocs_els_io_cleanup(els, OCS_EVT_SRRS_ELS_REQ_FAIL, arg);
		break;
	default:
		__ocs_els_common(__func__, ctx, evt, arg);
		break;
	}
	return NULL;
}

/**
 * @brief Wait for a retry timer to expire
 *
 * <h3 class="desc">Description</h3>
 * This state is entered when waiting for a timer event, so that
 * the ELS request can be retried.
 *
 * @param ctx Remote node SM context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_els_delay_retry(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_els_state_decl();

	els_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_setup_timer(ocs, &els->delay_timer, ocs_els_delay_timer_cb, els, 5000, false);
		break;
	case OCS_EVT_TIMER_EXPIRED:
		/* Retry delay timer expired, retry the ELS request, Free the HAL IO so
		 * that a new oxid is used.
		 */
		if (els->hio != NULL) {
			ocs_hal_io_free(&ocs->hal, els->hio);
			els->hio = NULL;
		}
		ocs_io_transition(els, __ocs_els_init, NULL);
		break;
	case OCS_EVT_ABORT_ELS:
		ocs_io_transition(els, __ocs_els_aborted_delay_retry, NULL);
		break;
	default:
		__ocs_els_common(__func__, ctx, evt, arg);
		break;
	}
	return NULL;
}

/**
 * @brief Wait for the ELS IO abort request to complete.
 *
 * <h3 class="desc">Description</h3>
 * This state is entered after we abort an ELS WQE and are
 * waiting for either the original ELS WQE request or the abort
 * to complete.
 *
 * @param ctx Remote node SM context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_els_aborting(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_els_state_decl();

	els_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:
	case OCS_EVT_SRRS_ELS_REQ_FAIL:
	case OCS_EVT_SRRS_ELS_REQ_RJT:
	case OCS_EVT_ELS_REQ_TIMEOUT:
	case OCS_EVT_ELS_REQ_ABORTED: {
		/* completion for ELS received first, transition to wait for abort cmpl */
		els_io_printf(els, "request cmpl evt=%s\n", ocs_sm_event_name(evt));
		ocs_io_transition(els, __ocs_els_aborting_wait_abort_cmpl, NULL);
		break;
	}
	case OCS_EVT_ELS_ABORT_CMPL: {
		/* completion for abort was received first, transition to wait for req cmpl */
		els_io_printf(els, "abort cmpl evt=%s\n", ocs_sm_event_name(evt));
		ocs_io_transition(els, __ocs_els_aborting_wait_req_cmpl, NULL);
		break;
	}
	case OCS_EVT_ABORT_ELS:
		/* nothing we can do but wait */
		break;

	default:
		__ocs_els_common(__func__, ctx, evt, arg);
		break;
	}
	return NULL;
}

/**
 * @brief cleanup ELS after abort
 *
 * @param els ELS IO to cleanup
 *
 * @return Returns None.
 */

static void
ocs_els_abort_cleanup(ocs_io_t *els)
{
	/* handle event for ABORT_WQE
	 * whatever state ELS happened to be in, propagate aborted event up
	 * to node state machine in lieu of OCS_EVT_SRRS_ELS_* event
	 */
	ocs_node_cb_t cbdata;
	cbdata.status = cbdata.ext_status = 0;
	cbdata.els = els;
	els_io_printf(els, "Request aborted\n");
	ocs_els_io_cleanup(els, OCS_EVT_ELS_REQ_ABORTED, &cbdata);
}

/**
 * @brief Wait for the ELS IO abort request to complete.
 *
 * <h3 class="desc">Description</h3>
 * This state is entered after we abort an ELS WQE, we received
 * the abort completion first and are waiting for the original
 * ELS WQE request to complete.
 *
 * @param ctx Remote node SM context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_els_aborting_wait_req_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_els_state_decl();

	els_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:
	case OCS_EVT_SRRS_ELS_REQ_FAIL:
	case OCS_EVT_SRRS_ELS_REQ_RJT:
	case OCS_EVT_ELS_REQ_TIMEOUT:
	case OCS_EVT_ELS_REQ_ABORTED: {
		/* completion for ELS that was aborted */
		ocs_els_abort_cleanup(els);
		break;
	}
	case OCS_EVT_ABORT_ELS:
		/* nothing we can do but wait */
		break;

	default:
		__ocs_els_common(__func__, ctx, evt, arg);
		break;
	}
	return NULL;
}

/**
 * @brief Wait for the ELS IO abort request to complete.
 *
 * <h3 class="desc">Description</h3>
 * This state is entered after we abort an ELS WQE, we received
 * the original ELS WQE request completion first and are waiting
 * for the abort to complete.
 *
 * @param ctx Remote node SM context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_els_aborting_wait_abort_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_els_state_decl();

	els_sm_trace();

	switch(evt) {
	case OCS_EVT_ELS_ABORT_CMPL: {
		ocs_els_abort_cleanup(els);
		break;
	}
	case OCS_EVT_ABORT_ELS:
		/* nothing we can do but wait */
		break;

	default:
		__ocs_els_common(__func__, ctx, evt, arg);
		break;
	}
	return NULL;
}

/**
 * @brief Generate ELS context ddump data.
 *
 * <h3 class="desc">Description</h3>
 * Generate the ddump data for an ELS context.
 *
 * @param textbuf Pointer to the text buffer.
 * @param els Pointer to the ELS context.
 *
 * @return None.
 */

void
ocs_ddump_els(ocs_textbuf_t *textbuf, ocs_io_t *els)
{
	ocs_ddump_section(textbuf, "els", -1);
	ocs_ddump_value(textbuf, "req_free", "%d", els->els_info->els_req_free);
	ocs_ddump_value(textbuf, "evtdepth", "%d", els->els_info->els_evtdepth);
	ocs_ddump_value(textbuf, "pend", "%d", els->els_info->els_pend);
	ocs_ddump_value(textbuf, "active", "%d", els->els_info->els_active);
	ocs_ddump_io(textbuf, els);
	ocs_ddump_endsection(textbuf, "els", -1);
}


/**
 * @brief return TRUE if given ELS list is empty (while taking proper locks)
 *
 * Test if given ELS list is empty while holding the node->active_ios_lock.
 *
 * @param node pointer to node object
 * @param list pointer to list
 *
 * @return TRUE if els_io_list is empty
 */

int32_t
ocs_els_io_list_empty(ocs_node_t *node, ocs_list_t *list)
{
	int empty;
	ocs_lock(&node->active_ios_lock);
		empty = ocs_list_empty(list);
	ocs_unlock(&node->active_ios_lock);
	return empty;
}

/**
 * @brief Handle CT send response completion
 *
 * Called when CT response completes, free IO
 *
 * @param hio Pointer to the HAL IO context that completed.
 * @param rnode Pointer to the remote node.
 * @param length Length of the returned payload data.
 * @param status Status of the completion.
 * @param ext_status Extended status of the completion.
 * @param arg Application-specific argument (generally a pointer to the ELS IO context).
 *
 * @return returns 0
 */
static int32_t
ocs_ct_acc_cb(ocs_hal_io_t *hio, ocs_remote_node_t *rnode, uint32_t length, int32_t status, uint32_t ext_status, void *arg)
{
	ocs_io_t *io = arg;

	ocs_els_io_free(io);

	return 0;
}

/**
 * @brief Send CT response
 *
 * Sends a CT response frame with payload
 *
 * @param io Pointer to the IO context.
 * @param ox_id Originator exchange ID
 * @param ct_hdr Pointer to the CT IU
 * @param cmd_rsp_code CT response code
 * @param reason_code Reason code
 * @param reason_code_explanation Reason code explanation
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_send_ct_rsp(ocs_io_t *io, uint32_t ox_id, fcct_iu_header_t *ct_hdr, uint32_t cmd_rsp_code, uint32_t reason_code, uint32_t reason_code_explanation)
{
	fcct_iu_header_t *rsp = io->els_info->els_rsp.virt;

	io->io_type = OCS_IO_TYPE_CT_RESP;

	*rsp = *ct_hdr;

	fcct_build_req_header(rsp, cmd_rsp_code, FC_GS_TYPE_DIRECTORY_SERVICE,
				FC_GS_SUBTYPE_NAME_SERVER, 0);
	rsp->reason_code = reason_code;
	rsp->reason_code_explanation = reason_code_explanation;

	io->display_name = "ct response";
	io->init_task_tag = ox_id;
	io->wire_len += sizeof(*rsp);

	ocs_memset(&io->iparam, 0, sizeof(io->iparam));

	io->io_type = OCS_IO_TYPE_CT_RESP;
	io->hio_type = OCS_HAL_FC_CT_RSP;
	io->iparam.fc_ct_rsp.ox_id = ocs_htobe16(ox_id);
	io->iparam.fc_ct_rsp.r_ctl = 3;
	io->iparam.fc_ct_rsp.type = FC_TYPE_GS;
	io->iparam.fc_ct_rsp.df_ctl = 0;
	io->iparam.fc_ct_rsp.timeout = 5;

	if (ocs_scsi_io_dispatch(io, ocs_ct_acc_cb) < 0) {
		ocs_els_io_free(io);
		return -1;
	}
	return 0;
}


/**
 * @brief Handle delay retry timeout
 *
 * Callback is invoked when the delay retry timer expires.
 *
 * @param arg pointer to the ELS IO object
 *
 * @return none
 */
static void
ocs_els_delay_timer_cb(void *arg)
{
	ocs_io_t *els = arg;
	ocs_node_t *node;

	if (els && ocs_io_busy(els) && (OCS_IO_TYPE_ELS == els->io_type)) {
		node = els->node;
	} else {
		return;
	}

	/*
	 * There is a potential deadlock here since is Linux executes timers
	 * in a soft IRQ context. The lock may be aready locked by the interrupt
	 * thread. Handle this case by attempting to take the node lock and reset the
	 * timer if we fail to acquire the lock.
	 *
	 * Note: This code relies on the fact that the node lock is recursive.
	 */
	if (ocs_node_lock_try(node)) {
		ocs_els_post_event(els, OCS_EVT_TIMER_EXPIRED, NULL);
		ocs_node_unlock(node);
	} else {
		ocs_setup_timer(els->ocs, &els->delay_timer, ocs_els_delay_timer_cb, els, 1, false);
	}
}
