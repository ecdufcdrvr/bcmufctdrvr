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
 * Implement remote device state machine for target and initiator.
 */

/*!
@defgroup device_sm Node State Machine: Remote Device States
*/

#include "ocs.h"
#include "ocs_scsi_fc.h"
#include "ocs_device.h"
#include "ocs_fabric.h"
#include "ocs_els.h"
#include "scsi_cmds.h"
#include "ocs_nvme_stub.h"

static void *__ocs_d_common(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
static void *__ocs_d_wait_del_node(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
static void *__ocs_d_wait_del_ini_tgt(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

void
ocs_adapter_set_hbs_bufsize(ocs_t *ocs, int32_t hbs_bufsize)
{
	ocs_t *other_ocs;
	uint8_t bus, dev, func;
	uint8_t other_bus, other_dev, other_func;
	uint32_t index = 0;

	ocs_get_bus_dev_func(ocs, &bus, &dev, &func);
	for_each_active_ocs(index, other_ocs) {
		ocs_get_bus_dev_func(other_ocs, &other_bus, &other_dev, &other_func);
		if ((bus == other_bus) && (dev == other_dev)) {
			other_ocs->hbs_bufsize = hbs_bufsize;
		}
	}
}

void
ocs_set_hbs_bufsize(ocs_t *ocs, int32_t size)
{
	ocs->hbs_bufsize = size;
}

int32_t
ocs_get_hbs_bufsize(ocs_t *ocs)
{
	return ocs->hbs_bufsize;
}

static void
ocs_nvme_node_quiesce(ocs_node_t *node, ocs_node_cb_t *cbdata)
{
	ocs_node_hold_frames(node);

	/* Unregister the session with the backend */
	node_printf(node, "delete NVME (initiator) WWPN %s WWNN %s\n", node->wwpn, node->wwnn);
	if (OCS_NVME_CALL_COMPLETE == ocs_nvme_del_initiator(node, cbdata))
		ocs_node_post_event(node, OCS_EVT_NODE_DEL_INI_COMPLETE, cbdata);
}

static void
ocs_nvme_node_resume(ocs_node_t *node)
{
	ocs_node_accept_frames(node);
}

static void
ocs_scsi_node_quiesce(ocs_node_t *node, ocs_node_cb_t *cbdata)
{
	ocs_node_hold_frames(node);
	ocs_scsi_io_alloc_disable(node);
	node->mark_for_deletion = TRUE;
	node->fcp_enabled = FALSE;

	/* Unregister the session with the backend */
	node_printf(node, "delete SCSI (initiator) WWPN %s WWNN %s\n", node->wwpn, node->wwnn);
	if (OCS_SCSI_CALL_COMPLETE == ocs_scsi_del_initiator(node, cbdata))
		ocs_node_post_event(node, OCS_EVT_NODE_DEL_INI_COMPLETE, cbdata);

	/* Drain all the SCSI active IO's */
	ocs_scsi_active_io_cancel(node);
}

static void
ocs_scsi_node_resume(ocs_node_t *node)
{
	node->mark_for_deletion = FALSE;
	ocs_scsi_io_alloc_enable(node);
	ocs_node_accept_frames(node);
}

static void
ocs_d_handle_sess_reg(ocs_node_t *node, ocs_node_cb_t *cbdata)
{
	int32_t rc = -1;

	switch (cbdata->prli_ctx.ls_rsp_type) {
	case OCS_LS_RSP_TYPE_NVME_PRLI:
		/* Make sure target supports NVME */
		if (!ocs_tgt_nvme_backend_enabled(node->ocs, node->sport)) {
			/* Send an LS_ACC with the appropriate prli service params */
			if (cbdata->io)
				ocs_send_prli_acc(cbdata->io, cbdata->io->els_info->ls_rsp_oxid,
						  FC_TYPE_NVME, NULL, NULL);

			ocs_free(node->ocs, cbdata, sizeof(*cbdata));
			ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, NULL);
			break;
		}

		/* Make sure initiator is NVME capable from sparams */
		if (!node->nvme_init) {
			node_printf(node, "NVMe PRLI rejected due to invalid sparams\n");
			cbdata->status = FC_REASON_PROTOCOL_ERROR;
			cbdata->ext_status = FC_EXPL_SPARAM_OPTIONS;
			ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, cbdata);
			break;
		}

		/* Let the backend decide to accept or reject this initiator */
		if (ocs_nvme_validate_initiator(node) == 0) {
			node_printf(node, "NVMe PRLI rejected by target-server\n");
			cbdata->ext_status = FC_EXPL_NO_RESOURCES_ASSIGNED;
			ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, cbdata);
			break;
		}

		node_printf(node, "Found (NVMe initiator) WWPN %s WWNN %s suppress rsp: %d\n", node->wwpn, node->wwnn, node->suppress_rsp);
		rc = ocs_nvme_new_initiator(node, cbdata);
		if (rc < 0) {
			node_printf(node, "NVMe new initiator failed\n");
			ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, cbdata);
			break;
		}

		/* Accepted */
		node->tgt_fct_prli_success |= OCS_TARGET_TYPE_NVME;

		/*
		 * If the backend call completes synchronously, post the 'NODE_SESS_REG_OK' here itself.
		 * Else, post this event from backend thread once the session registration completes.
		 */
		if (OCS_NVME_CALL_COMPLETE == rc)
			ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_OK, cbdata);

		break;

	case OCS_LS_RSP_TYPE_FCP_PRLI:
		/* Make sure target backend supports SCSI */
		if (!ocs_tgt_scsi_backend_enabled(node->ocs, NULL)) {
			/* Send an LS_ACC with the appropriate prli service params */
			if (cbdata->io)
				ocs_send_prli_acc(cbdata->io, cbdata->io->els_info->ls_rsp_oxid,
						  FC_TYPE_FCP, NULL, NULL);

			ocs_free(node->ocs, cbdata, sizeof(*cbdata));
			ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, NULL);
			break;
		}

		/* Make sure initiator is SCSI capable from sparams */
		if (!node->init) {
			node_printf(node, "SCSI PRLI rejected due to invalid sparams\n");
			cbdata->status = FC_REASON_PROTOCOL_ERROR;
			cbdata->ext_status = FC_EXPL_SPARAM_OPTIONS;
			ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, cbdata);
			break;
		}

		/* Let the backend decide to accept or reject this initiator */
		if (ocs_scsi_validate_initiator(node) == 0) {
			node_printf(node, "SCSI PRLI rejected by target-server\n");
			cbdata->ext_status = FC_EXPL_NO_RESOURCES_ASSIGNED;
			ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, cbdata);
			break;
		}

		node_printf(node, "Found (SCSI initiator) WWPN %s WWNN %s suppress rsp: %d\n", node->wwpn, node->wwnn, node->suppress_rsp);
		rc = ocs_scsi_new_initiator(node, cbdata);
		if (rc < 0) {
			node_printf(node, "SCSI new initiator failed\n");
			ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, cbdata);
			break;
		}

		/* Accepted */
		node->tgt_fct_prli_success |= OCS_TARGET_TYPE_FCP;

		/*
		 * If the backend call completes synchronously, post the 'NODE_SESS_REG_OK' here itself.
		 * Else, post this event from backend thread once the session registration completes.
		 */
		if (OCS_SCSI_CALL_COMPLETE == rc)
			ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_OK, cbdata);

		break;

	default:
		ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, cbdata);
		return;
	}
}

static bool
ocs_node_nsler_reg_rpi_update_needed(ocs_node_t *node, ocs_ls_rsp_type_e type)
{
	return (type == OCS_LS_RSP_TYPE_NVME_PRLI) &&
		ocs_tgt_nvme_enabled(node->ocs) && ocs_node_nsler_negotiated(node);
}

static void
ocs_d_handle_prli_reg_rpi_update(ocs_node_t *node, ocs_io_t *io)
{
	ocs_node_cb_t *node_cbdata = NULL;
	int32_t ext_status = FC_EXPL_REQUEST_NOT_SUPPORTED;

	node_cbdata = ocs_malloc(node->ocs, sizeof(*node_cbdata), OCS_M_ZERO | OCS_M_NOWAIT);
	if (!node_cbdata) {
		node_printf_err(node, "Failed to process PRLI due to node_cbdata alloc failure\n");
		ext_status = FC_EXPL_INSUFFICIENT_RESOURCES;
		goto err_exit;
	}

	node_cbdata->prli_ctx.ls_rsp_type = io->els_info->ls_rsp_type;
	if (io->cmd_tgt)
		node_cbdata->io = io;
	else
		node_cbdata->io = NULL;

	/* Update REG_RPI with NSLER capability and then send PRLI resp */
	if (ocs_hal_reg_rpi_update(&node->ocs->hal,
			&node->rnode,
			ocs_node_nsler_reg_rpi_update_needed(node, io->els_info->ls_rsp_type),
			node_cbdata)) {
		ext_status = FC_EXPL_NO_ADDITIONAL;
		goto err_exit;
	}

	return;

err_exit:
	if (io->cmd_tgt) {
		node_printf(node, "Sending LS reject on PRLI request\n");
		ocs_send_ls_rjt(io, io->els_info->ls_rsp_oxid,
				FC_REASON_UNABLE_TO_PERFORM, ext_status,
				0, NULL, NULL);
	}

	if (node_cbdata)
		ocs_free(node->ocs, node_cbdata, sizeof(*node_cbdata));

	ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, NULL);

	return;
}

static void
ocs_d_handle_prli_backend_process(ocs_node_t *node, ocs_io_t *io, ocs_ls_rsp_type_e ls_rsp_type)
{
	ocs_node_cb_t *node_cbdata = NULL;
	int32_t ext_status = FC_EXPL_REQUEST_NOT_SUPPORTED;

	if ((ls_rsp_type != OCS_LS_RSP_TYPE_NVME_PRLI) &&
		(ls_rsp_type != OCS_LS_RSP_TYPE_FCP_PRLI)) {
		node_printf_err(node, "Failed to process PRLI due to unsupported type %d\n",
				ls_rsp_type);
		goto err_exit;
	}

	node_cbdata = ocs_malloc(node->ocs, sizeof(*node_cbdata), OCS_M_ZERO | OCS_M_NOWAIT);
	if (!node_cbdata) {
		node_printf_err(node, "Failed to process PRLI due to node_cbdata alloc failure\n");
		ext_status = FC_EXPL_INSUFFICIENT_RESOURCES;
		goto err_exit;
	}

	node_cbdata->prli_ctx.ls_rsp_type = ls_rsp_type;

	if (io && io->cmd_tgt)
		node_cbdata->io = io;
	else
		node_cbdata->io = NULL;

	node_cbdata->status = FC_REASON_UNABLE_TO_PERFORM;
	node_cbdata->ext_status = ext_status;

	/* Check for duplicate PRLI request */
	if ((ls_rsp_type == OCS_LS_RSP_TYPE_NVME_PRLI) &&
			(node->tgt_fct_prli_success & OCS_TARGET_TYPE_NVME)) {
		node_printf(node, "Processing duplicate NVMe PRLI\n");
		ocs_nvme_node_quiesce(node, node_cbdata);
	} else if ((ls_rsp_type == OCS_LS_RSP_TYPE_FCP_PRLI) &&
			(node->tgt_fct_prli_success & OCS_TARGET_TYPE_FCP)) {
		node_printf(node, "Processing duplicate SCSI PRLI\n");
		ocs_scsi_node_quiesce(node, node_cbdata);
	} else {
		/* If this is not a duplicate request, proceed to register a session with the backend */
		ocs_d_handle_sess_reg(node, node_cbdata);
	}

	return;

err_exit:
	if (io && io->cmd_tgt) {
		node_printf(node, "Sending LS reject on PRLI request\n");
		ocs_send_ls_rjt(io, io->els_info->ls_rsp_oxid,
				FC_REASON_UNABLE_TO_PERFORM, ext_status,
				0, NULL, NULL);
	}

	ocs_node_post_event(node, OCS_EVT_NODE_SESS_REG_FAIL, NULL);

	return;
}

void *
__ocs_d_initiate_scsi_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER: {
		int32_t rc = OCS_SCSI_CALL_COMPLETE;
		bool node_scsi_tgt = node->ini_fct_prli_success & OCS_INITIATOR_TYPE_FCP;
		bool node_scsi_ini = node->tgt_fct_prli_success & OCS_TARGET_TYPE_FCP;

		/* Add node to shutdown list to track timeout */
		ocs_node_add_shutdown_list(node);

		if (node_scsi_ini && node_scsi_tgt) {
			node_printf(node, "delete SCSI (initiator+target) WWPN %s WWNN %s\n", node->wwpn, node->wwnn);
			ocs_node_transition(node, __ocs_d_wait_del_ini_tgt, NULL);

			rc = ocs_scsi_del_initiator(node, NULL);
			if (rc == OCS_SCSI_CALL_COMPLETE)
				ocs_node_post_event(node, OCS_EVT_NODE_DEL_INI_COMPLETE, NULL);

			rc = ocs_scsi_del_target(node);
			if (rc == OCS_SCSI_CALL_COMPLETE)
				ocs_node_post_event(node, OCS_EVT_NODE_DEL_TGT_COMPLETE, NULL);
		} else if (node_scsi_ini) {
			node_printf(node, "delete SCSI (initiator) WWPN %s WWNN %s\n", node->wwpn, node->wwnn);
			ocs_node_transition(node, __ocs_d_wait_del_node, NULL);

			rc = ocs_scsi_del_initiator(node, NULL);
			if (rc == OCS_SCSI_CALL_COMPLETE)
				ocs_node_post_event(node, OCS_EVT_NODE_DEL_INI_COMPLETE, NULL);
		} else if (node_scsi_tgt) {
			node_printf(node, "delete SCSI (target) WWPN %s WWNN %s\n", node->wwpn, node->wwnn);
			ocs_node_transition(node, __ocs_d_wait_del_node, NULL);

			rc = ocs_scsi_del_target(node);
			if (rc == OCS_SCSI_CALL_COMPLETE)
				ocs_node_post_event(node, OCS_EVT_NODE_DEL_TGT_COMPLETE, NULL);
		}

		/* Drain the node-specific IO's from 'io_pending_list' */
		ocs_scsi_pending_io_cancel(node);

		/* we've initiated the upcalls as needed, now kick off the node
		 * detach to precipitate the aborting of outstanding exchanges
		 * associated with said node
		 *
		 * Beware: if we've made upcall(s), we've already transitioned
		 * to a new state by the time we execute this.
		 * TODO: consider doing this before the upcalls...
		 */
		if (node->attached) {
			/* issue hal node free; don't care if succeeds right away
			 * or sometime later, will check node->attached later in
			 * shutdown process
			 */
			rc = ocs_hal_node_detach(&ocs->hal, &node->rnode);
			if (node->rnode.free_group) {
				ocs_remote_node_group_free(node->node_group);
				node->node_group = NULL;
				node->rnode.free_group = FALSE;
			}

			if (rc != OCS_HAL_RTN_SUCCESS && rc != OCS_HAL_RTN_SUCCESS_SYNC) {
				node_printf(node, "Failed freeing HAL node, rc=%d\n", rc);
			} else if (node->detach_notify_pending) {
				node->detach_notify_pending = false;
				node_printf(node, "Notify node detached event\n");
				ocs_node_post_event(node, OCS_EVT_NODE_FREE_OK, NULL);
			}
		}

		/* If neither SCSI initiator nor target, proceed to cleanup */
		if (!node_scsi_ini && !node_scsi_tgt) {
			/*
			 * node has either been detached or is in the process of being detached,
			 * call common node's initiate cleanup function
			 */
			ocs_node_initiate_cleanup(node);
		}

		node->ini_fct_prli_success = 0;
		node->tgt_fct_prli_success = 0;
		node->prli_rsp_pend = 0;
		break;
	}

	case OCS_EVT_ALL_CHILD_NODES_FREE:
		/* Ignore, this can happen if an ELS is aborted while in a delay/retry state */
		break;

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
	}

	return NULL;
}

void *
__ocs_d_initiate_nvme_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER: {
		int32_t rc1 = OCS_NVME_CALL_COMPLETE, rc2 = OCS_NVME_CALL_COMPLETE;
		bool node_nvme_tgt = node->ini_fct_prli_success & OCS_INITIATOR_TYPE_NVME;
		bool node_nvme_ini = node->tgt_fct_prli_success & OCS_TARGET_TYPE_NVME;

		if (node_nvme_ini) {
			node_printf(node, "delete NVME (initiator) WWPN %s WWNN %s\n", node->wwpn, node->wwnn);
			rc1 = ocs_nvme_del_initiator(node, NULL);
			if (rc1 == OCS_NVME_CALL_COMPLETE)
				node->tgt_fct_prli_success &= ~OCS_TARGET_TYPE_NVME;
		}

		if (node_nvme_tgt) {
			node_printf(node, "delete NVME (target) WWPN %s WWNN %s\n", node->wwpn, node->wwnn);
			rc2 = ocs_nvme_del_target(node);
			if (rc2 == OCS_NVME_CALL_COMPLETE)
				node->ini_fct_prli_success &= ~OCS_INITIATOR_TYPE_NVME;
		}

		/* Async response will take care of moving the SM */
		if ((rc1 == OCS_NVME_CALL_ASYNC) || (rc2 == OCS_NVME_CALL_ASYNC))
			return NULL;

		break;
	}

	case OCS_EVT_NODE_DEL_INI_COMPLETE:
		node->tgt_fct_prli_success &= ~OCS_TARGET_TYPE_NVME;
		break;

	case OCS_EVT_NODE_DEL_TGT_COMPLETE:
		node->ini_fct_prli_success &= ~OCS_INITIATOR_TYPE_NVME;
		break;

	/* ignore shutdown events as we're already in shutdown path */
	case OCS_EVT_SHUTDOWN:
		/* have default shutdown event take precedence */
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		FALL_THROUGH; /* fall through */
	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		node_printf(node, "%s received\n", ocs_sm_event_name(evt));
		return NULL;
	case OCS_EVT_NODE_FREE_OK:
		node_printf(node, "Detach notify is pending\n");
		node->detach_notify_pending = true;
		break;
	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	if (!(node->ini_fct_prli_success & OCS_INITIATOR_TYPE_NVME) &&
	    !(node->tgt_fct_prli_success & OCS_TARGET_TYPE_NVME)) {
		ocs_node_transition(node, __ocs_d_initiate_scsi_shutdown, NULL);
	}

	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Initiate node shutdown
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_initiate_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER: {
		/* It is necessary to call prep_notify_it_nexus_loss() before
		 * io alloc disable.
		 */
		if (node->prep_notify_it_nexus_loss)
			node->prep_notify_it_nexus_loss(node->ocs, node);
		ocs_node_hold_frames(node);
		ocs_scsi_io_alloc_disable(node);
		node->mark_for_deletion = TRUE;

		if (ocs_tgt_nvme_enabled(node->ocs) || ocs_ini_nvme_enabled(node->ocs))
			ocs_node_transition(node, __ocs_d_initiate_nvme_shutdown, NULL);
		else
			ocs_node_transition(node, __ocs_d_initiate_scsi_shutdown, NULL);

		break;
	}

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Common device event handler.
 *
 * <h3 class="desc">Description</h3>
 * For device nodes, this event handler manages default and common events.
 *
 * @param funcname Function name text.
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

static void *
__ocs_d_common(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_t *node = NULL;
	ocs_t *ocs = NULL;
	ocs_assert(ctx, NULL);
	node = ctx->app;
	ocs_assert(node, NULL);
	ocs = node->ocs;
	ocs_assert(ocs, NULL);

	switch(evt) {

	/* Handle shutdown events */
	case OCS_EVT_SHUTDOWN:
		ocs_log_debug(ocs, "[%s] %-20s %-20s\n", node->display_name, funcname, ocs_sm_event_name(evt));
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);
		break;
	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
		ocs_log_debug(ocs, "[%s] %-20s %-20s\n", node->display_name, funcname, ocs_sm_event_name(evt));
		node->shutdown_reason = OCS_NODE_SHUTDOWN_EXPLICIT_LOGO;
		ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);
		break;
	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		ocs_log_debug(ocs, "[%s] %-20s %-20s\n", node->display_name, funcname, ocs_sm_event_name(evt));
		node->shutdown_reason = OCS_NODE_SHUTDOWN_IMPLICIT_LOGO;
		ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);
		break;

	default:
		/* call default event handler common to all nodes */
		__ocs_node_common(funcname, ctx, evt, arg);
		break;
	}
	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Wait for a domain-attach completion in loop topology.
 *
 * <h3 class="desc">Description</h3>
 * State waits for a domain-attached completion while in loop topology.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_wait_loop(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_node_hold_frames(node);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_DOMAIN_ATTACH_OK: {
		/* send PLOGI automatically if initiator */
		ocs_node_init_device(node, TRUE);
		break;
	}
	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup device_sm
 * @brief state: wait for node resume event
 *
 * State is entered when a node is in I+T mode and sends a delete initiator/target
 * call to the target-server/initiator-client and needs to wait for that work to complete.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg per event optional argument
 *
 * @return returns NULL
 */

void *
__ocs_d_wait_del_ini_tgt(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_node_hold_frames(node);
		FALL_THROUGH; /* fall through */

	case OCS_EVT_NODE_ACTIVE_IO_LIST_EMPTY:
	case OCS_EVT_ALL_CHILD_NODES_FREE:
		/* These are expected events. */
		break;

	case OCS_EVT_NODE_DEL_INI_COMPLETE:
	case OCS_EVT_NODE_DEL_TGT_COMPLETE:
		ocs_node_transition(node, __ocs_d_wait_del_node, NULL);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_SRRS_ELS_REQ_FAIL:
		/* Can happen as ELS IO IO's complete */
		break;

	/* ignore shutdown events as we're already in shutdown path */
	case OCS_EVT_SHUTDOWN:
		/* have default shutdown event take precedence */
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		FALL_THROUGH; /* fall through */
	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		node_printf(node, "%s received\n", ocs_sm_event_name(evt));
		break;
	case OCS_EVT_DOMAIN_ATTACH_OK:
		/* don't care about domain_attach_ok */
		break;
	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}


/**
 * @ingroup device_sm
 * @brief state: Wait for node resume event.
 *
 * State is entered when a node sends a delete initiator/target call to the
 * target-server/initiator-client and needs to wait for that work to complete.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_wait_del_node(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_node_hold_frames(node);
		FALL_THROUGH; /* fall through */

	case OCS_EVT_NODE_ACTIVE_IO_LIST_EMPTY:
	case OCS_EVT_ALL_CHILD_NODES_FREE:
		/* These are expected events. */
		break;

	case OCS_EVT_NODE_DEL_INI_COMPLETE:
	case OCS_EVT_NODE_DEL_TGT_COMPLETE:
		/*
		 * node has either been detached or is in the process of being detached,
		 * call common node's initiate cleanup function
		 */
		ocs_node_initiate_cleanup(node);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_SRRS_ELS_REQ_FAIL:
		/* Can happen as ELS IO IO's complete */
		break;

	/* ignore shutdown events as we're already in shutdown path */
	case OCS_EVT_SHUTDOWN:
		/* have default shutdown event take precedence */
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		FALL_THROUGH; /* fall through */
	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		node_printf(node, "%s received\n", ocs_sm_event_name(evt));
		break;
	case OCS_EVT_DOMAIN_ATTACH_OK:
		/* don't care about domain_attach_ok */
		break;
	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @brief Determine if NVMe SLER is supported on this device
 *
 * @param ocs Pointer to the OCS object
 *
 * @return TRUE if NVMe SLER is supported, otherwise FALSE
 */
bool
ocs_nsler_capable(ocs_t *ocs)
{
	ocs_hal_t *hal = &ocs->hal;

	return hal->config.nsler;
}

/**
 * @brief Determine if first_burst is enabled on this device
 *
 * @param ocs Pointer to the OCS object
 *
 * @return TRUE if first burst enabled, otherwise FALSE
 */
int32_t
ocs_first_burst_enabled(ocs_t *ocs)
{
	ocs_hal_t *hal = &ocs->hal;

	return (hal->tow_enabled && (hal->config.tow_feature & OCS_TOW_FEATURE_TFB));
}

/**
 * @brief Process the PRLI payload.
 *
 * <h3 class="desc">Description</h3>
 * The PRLI payload is processed; the initiator/target capabilities of the
 * remote node are extracted and saved in the node object.
 *
 * @param node Pointer to the node object.
 * @param prli Pointer to the PRLI payload.
 *
 * @return None.
 */

void
ocs_process_prli_payload(ocs_node_t *node, fc_prli_payload_t *prli)
{
	uint16_t sparams = ocs_be16toh(prli->service_params);

	if (prli->type == FC_TYPE_NVME) {
		node->nvme_init = (sparams & FC_PRLI_INITIATOR_FUNCTION) != 0;
		node->nvme_tgt = (sparams & FC_PRLI_TARGET_FUNCTION) != 0;
		node->nvme_sler = (sparams & FC_PRLI_RETRY) != 0;
		node->nvme_conf = (sparams & FC_PRLI_CONFIRMED_COMPLETION) != 0;
		node->nvme_prli_service_params = prli->service_params; /* BE format */
		/*
		 * Set the 'remote_nvme_prli_rcvd' flag to TRUE so that we don't need to handle
		 * the backend session registration while processing the PRLI completion.
		 *
		 * For PRLI responses, command_code will be either ACC/RJT. If the command_code
		 * is PRLI, then it is an incoming request from the remote node.
		 */
		if (prli->command_code == FC_ELS_CMD_PRLI)
			node->remote_nvme_prli_rcvd = TRUE;
	} else if (prli->type == FC_TYPE_FCP) {
		node->init = (sparams & FC_PRLI_INITIATOR_FUNCTION) != 0;
		node->targ = (sparams & FC_PRLI_TARGET_FUNCTION) != 0;
		/* Determine node first_burst capability based on the support at both sides */
		node->first_burst = (sparams & FC_PRLI_WRITE_XRDY_DISABLED) != 0;
		/*
		 * Set the 'remote_fcp_prli_rcvd' flag to TRUE so that we don't need to handle
		 * the backend session registration while processing the PRLI completion.
		 *
		 * For PRLI responses, command_code will be either ACC/RJT. If the command_code
		 * is PRLI, then it is an incoming request from the remote node.
		 */
		if (prli->command_code == FC_ELS_CMD_PRLI)
			node->remote_fcp_prli_rcvd = TRUE;
	}
}

static void
ocs_notify_new_target(ocs_node_t *node)
{
	if (node->targ && (node->ini_fct_prli_success & OCS_INITIATOR_TYPE_FCP)) {
		node_printf(node, "Found (SCSI target) WWPN %s WWNN %s\n", node->wwpn, node->wwnn);
		ocs_scsi_new_target(node);
	} else {
		node->ini_fct_prli_success &= ~OCS_INITIATOR_TYPE_FCP;
	}

	if (node->nvme_tgt && (node->ini_fct_prli_success & OCS_INITIATOR_TYPE_NVME)) {
		node_printf(node, "Found (NVMe target) WWPN %s WWNN %s\n", node->wwpn, node->wwnn);
		ocs_nvme_new_target(node);
	} else {
		node->ini_fct_prli_success &= ~OCS_INITIATOR_TYPE_NVME;
	}
}

int32_t
ocs_scsi_process_abts(ocs_node_t *node, ocs_io_t *tmfio, ocs_io_t *io_to_abort,
		      uint16_t ox_id, uint16_t rx_id)
{
	/* Got a reference on the IO; hold it until backend is notified below */
	node_printf(node, "Abort request: [%016" PRIX64".%016" PRIX64"] ox_id [%04x] rx_id [%04x]\n",
		    node->sport->wwpn, ocs_node_get_wwpn(node), ox_id, rx_id);

	/* Call target server command abort */
	tmfio->display_name = "abts";

	/*
	 * Save the rx_id from the ABTS as it is needed for the BLS response,
	 * regardless of the IO context's rx_id
	 */
	tmfio->abort_rx_id = rx_id;

	/* Mark the IO to indicate ABTS in progress */
	io_to_abort->abts_received = TRUE;

	/*
	 * If this SCSI IO doesn't have an associated HAL IO, scan the 'xport->io_pending_list'.
	 * If found, remove the IO entry, invoke the IO cb and return.
	 * Else, if there is a pending WQE for this HAL IO, post an abort WQE immediately.
	 */
	ocs_scsi_io_dispatch_abort(io_to_abort, NULL, FALSE);

	/* Notify the backend */
	ocs_scsi_recv_tmf(tmfio, io_to_abort->scsi_info->tgt_io.lun, tmfio->scsi_info->tmf_cmd, io_to_abort, 0);

	/*
	 * Backend will have taken an additional reference on the IO if needed;
	 * done with current reference.
	 */
	ocs_ref_put(&io_to_abort->ref); /* ocs_ref_get(): same function */

	return 0;
}

int32_t
ocs_process_flush_bls(ocs_node_cb_t *cbdata)
{
	ocs_node_t *node = cbdata->node;
	fc_header_t *hdr = cbdata->header;
	fc_bls_flush_payload_t *flush_pl;
	uint16_t ox_id = ocs_be16toh(hdr->ox_id);
	uint16_t rx_id = ocs_be16toh(hdr->rx_id);
	uint16_t flush_count;

	if (!ocs_node_nsler_negotiated(node)) {
		node_printf_err(node, "NVMe SLER is not negotiated successfully, drop FLUSH BLS\n");
		goto exit;
	}

	flush_pl = (fc_bls_flush_payload_t *)cbdata->payload;
	flush_count = ocs_be16toh(flush_pl->count);

	if (ocs_tgt_nvme_enabled(node->ocs)) {
		ocs_nvme_process_flush_bls(node, ox_id, rx_id,
					   ocs_be32toh(hdr->parameter),
					   flush_pl->ht, flush_count);
	}

exit:
	return 0;
}

void *
__ocs_d_node_attached_wait_plogi_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_node_hold_frames(node);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_SRRS_ELS_REQ_OK:   /* PLOGI response received */
	case OCS_EVT_SRRS_ELS_REQ_FAIL:
	case OCS_EVT_SRRS_ELS_REQ_RJT:
		/* We dont care for PLOGI status as we already received the other side plogi. */
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_d_common, __func__)) {
			return NULL;
		}
		ocs_node_transition(node, __ocs_d_port_logged_in, NULL);
		break;

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Wait for the PLOGI accept to complete.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_wait_plogi_acc_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_node_hold_frames(node);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_SRRS_ELS_CMPL_FAIL:
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);
		break;

	case OCS_EVT_SRRS_ELS_CMPL_OK:	/* PLOGI ACC completions */
		if (node->plogi_els_io) {
			/* There is an outstanding plogi sent out, Wait for its completion. */
			ocs_node_transition(node, __ocs_d_node_attached_wait_plogi_rsp, NULL);
		} else {
			ocs_node_transition(node, __ocs_d_port_logged_in, NULL);
		}

		break;

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Wait for the LOGO response.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_wait_logo_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_t *node = ctx->app;

	node_sm_prologue();
	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		// TODO: may want to remove this; if we'll want to know about PLOGI
		ocs_node_hold_frames(node);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_SRRS_ELS_REQ_OK:
	case OCS_EVT_SRRS_ELS_REQ_RJT:
	case OCS_EVT_SRRS_ELS_REQ_FAIL:
		/* LOGO response received, sent shutdown */
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_LOGO, __ocs_d_common, __func__))
			return NULL;

		node_printf(node, "LOGO sent (evt=%s), shutdown node\n", ocs_sm_event_name(evt));

		/* Post explicit logout */
		ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		break;

	// TODO: PLOGI: abort LOGO and process PLOGI? (SHUTDOWN_EXPLICIT/IMPLICIT_LOGO?)

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		break;
	}

	return NULL;
}

/**
 * @brief Initialize device node.
 *
 * Initialize device node. If a node is an initiator, then send a PLOGI and transition
 * to __ocs_d_wait_plogi_rsp, otherwise transition to __ocs_d_init.
 *
 * @param node Pointer to the node object.
 * @param send_plogi Boolean indicating to send PLOGI command or not.
 *
 * @return none
 */

void
ocs_node_init_device(ocs_node_t *node, int send_plogi)
{
	node->send_plogi = send_plogi;
	if ((node->ocs->nodedb_mask & OCS_NODEDB_PAUSE_NEW_NODES) && !FC_ADDR_IS_DOMAIN_CTRL(node->rnode.fc_id)) {
		node->nodedb_state = __ocs_d_init;
		ocs_node_transition(node, __ocs_node_paused, NULL);
	} else {
		ocs_node_transition(node, __ocs_d_init, NULL);
	}
}

void
ocs_node_plogi_cmpl_done(ocs_node_t *node, ocs_node_cb_t *cbdata, void *arg)
{
	node->plogi_els_io = NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Initial node state for an initiator or a target.
 *
 * <h3 class="desc">Description</h3>
 * This state is entered when a node is instantiated, either having been
 * discovered from a name services query, or having received a PLOGI/FLOGI.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 * - OCS_EVT_ENTER: (uint8_t *) - 1 to send a PLOGI on
 * entry (initiator-only); 0 indicates a PLOGI is
 * not sent on entry (initiator-only). Not applicable for a target.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc;
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		/* check if we need to send PLOGI */
		if (node->send_plogi) {
			/* only send if we have initiator capability, and domain is attached */
			if (node->sport->enable_ini && node->sport->domain->attached) {
				node->plogi_els_io = ocs_send_plogi(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
						OCS_FC_ELS_DEFAULT_RETRIES, ocs_node_plogi_cmpl_done, NULL);
				if (node->plogi_els_io) {
					ocs_node_transition(node, __ocs_d_wait_plogi_rsp, NULL);
				} else {
					node_printf(node, "Failed to send PLOGI req, shutting down node\n");
					ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
				}
			} else {
				node_printf(node, "not sending plogi sport.ini=%d, domain attached=%d\n",
					    node->sport->enable_ini, node->sport->domain->attached);
			}
		}
		break;
	case OCS_EVT_PLOGI_RCVD: {
		/* T, or I+T */
		fc_header_t *hdr = cbdata->header;
		uint32_t d_id = fc_be24toh(hdr->d_id);

		ocs_node_save_sparms(node, cbdata->payload);
		node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI] = cbdata->io;

		/* domain already attached */
		if (node->sport->domain->attached) {
			rc = ocs_node_attach(node);
			ocs_node_transition(node, __ocs_d_wait_node_attach, NULL);
			if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
				ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
			} else if (rc != OCS_HAL_RTN_SUCCESS) {
				ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
			}
			break;
		}

		/* domain not attached; several possibilities: */
		switch (node->sport->topology) {
		case OCS_SPORT_TOPOLOGY_P2P:
			/* we're not attached and sport is p2p, need to attach */
			ocs_domain_attach_async(node->sport->domain, d_id, __ocs_d_wait_domain_attach, node);
			break;
		case OCS_SPORT_TOPOLOGY_FABRIC:
			/* we're not attached and sport is fabric, domain attach should have
			 * already been requested as part of the fabric state machine, wait for it
			 */
			ocs_node_transition(node, __ocs_d_wait_domain_attach, NULL);
			break;
		case OCS_SPORT_TOPOLOGY_UNKNOWN:
			/* Two possibilities:
			 * 1. received a PLOGI before our FLOGI has completed (possible since
			 *    completion comes in on another CQ), thus we don't know what we're
			 *    connected to yet; transition to a state to wait for the fabric
			 *    node to tell us;
			 * 2. PLOGI received before link went down and we haven't performed
			 *    domain attach yet.
			 * Note: we cannot distinguish between 1. and 2. so have to assume PLOGI
			 * was received after link back up.
			 */
			node_printf(node, "received PLOGI, with unknown topology did=0x%x\n", d_id);
			ocs_node_transition(node, __ocs_d_wait_topology_notify, NULL);
			break;
		default:
			node_printf(node, "received PLOGI, with unexpectd topology %d\n",
				    node->sport->topology);
			ocs_assert(FALSE, NULL);
			break;
		}
		break;
	}

	case OCS_EVT_FDISC_RCVD: {
#if defined(ENABLE_FABRIC_EMULATION)
		fc_header_t *hdr = cbdata->header;

		/* If this domain is do configured then call the Fabric Emulation FDISC handler */
		if (node->sport->domain->femul_enable) {
			ocs_femul_process_fdisc(cbdata->io, hdr, cbdata->payload, cbdata->payload_len);
			break;
		}
#endif
		__ocs_d_common(__func__, ctx, evt, arg);
		break;
	}

	case OCS_EVT_FLOGI_RCVD: {
		fc_header_t *hdr = cbdata->header;

		/* this better be coming from an NPort */
		ocs_assert(ocs_rnode_is_nport(cbdata->payload), NULL);

		//sm: / save sparams, send FLOGI acc
		ocs_domain_save_sparms(node->sport->domain, cbdata->payload);

#if defined(ENABLE_FABRIC_EMULATION)
		if (node->sport->domain->femul_enable) {
			/* Process FLOGI in Fabric Emulation mode, don't transition */
			ocs_femul_process_flogi(cbdata->io, hdr, cbdata->payload, cbdata->payload_len);
			break;
		}
#endif
		/* send FC LS_ACC response, override s_id */
		ocs_fabric_set_topology(node, OCS_SPORT_TOPOLOGY_P2P);
		ocs_send_flogi_p2p_acc(cbdata->io, ocs_be16toh(hdr->ox_id), fc_be24toh(hdr->d_id), NULL, NULL);

		/* Do IMPLICIT LOGO, if domain is already attached and received FLOGI */
		if (!node->sport->domain->attached) {
			if (ocs_p2p_setup(node->sport)) {
				node_printf(node, "p2p setup failed, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN, NULL);
			} else {
				ocs_node_transition(node, __ocs_p2p_wait_flogi_acc_cmpl, NULL);
			}
		} else {
			ocs_hal_p2p_lip_handle(&node->ocs->hal);
		}

		break;
	}

	case OCS_EVT_LOGO_RCVD: {
		fc_header_t *hdr = cbdata->header;

		if (!node->sport->domain->attached) {
			 /* most likely a frame left over from before a link down; drop and
			  * shut node down w/ "explicit logout" so pending frames are processed */
			node_printf(node, "%s domain not attached, dropping\n", ocs_sm_event_name(evt));
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			break;
		}

		ocs_send_logo_acc(cbdata->io, ocs_be16toh(hdr->ox_id), NULL, NULL);
		ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		break;
	}

	case OCS_EVT_RDP_RCVD: {
		node_printf(node, "%s received, processing payload\n", ocs_sm_event_name(evt));
		ocs_send_rdp_resp(node, arg, 0);
		break;
	}

	case OCS_EVT_NVME_CMD_RCVD:
	case OCS_EVT_FCP_CMD_RCVD: {
//note: problem, we're now expecting an ELS REQ completion from both the LOGO and PLOGI
		if (!node->sport->domain->attached) {
			 /* most likely a frame left over from before a link down; drop and
			  * shut node down w/ "explicit logout" so pending frames are processed */
			node_printf(node, "%s domain not attached, dropping\n", ocs_sm_event_name(evt));
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			break;
		}

		ocs_log_info(node->ocs, "[%s] FCP_CMND received before port login, send LOGO\n",
			     node->display_name);
		if (ocs_send_logo(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT, 0, NULL, NULL) == NULL) {
			/* Failed to send LOGO, go ahead and cleanup node anyways */
			ocs_log_err(node->ocs, "[%s] Failed to send LOGO, cleanup node context\n",
				    node->display_name);
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		}

		break;
	}

	case OCS_EVT_PRLI_RCVD:
	case OCS_EVT_PRLO_RCVD:
	case OCS_EVT_PDISC_RCVD:
	case OCS_EVT_ADISC_RCVD:
	case OCS_EVT_RSCN_RCVD:
	case OCS_EVT_ABTS_RCVD: {
		fc_header_t *hdr = cbdata->header;

		node_printf(node, "%s received, sending LOGO\n", ocs_sm_event_name(evt));
		ocs_sframe_send_logo(node, fc_be24toh(hdr->d_id), fc_be24toh(hdr->s_id), 0xFFFF, 0xFFFF);
		if (cbdata->io)
			ocs_scsi_io_free(cbdata->io);
		break;
	}

	case OCS_EVT_DOMAIN_ATTACH_OK:
		/* don't care about domain_attach_ok */
		break;

#if defined(ENABLE_FABRIC_EMULATION)
	case OCS_EVT_SCR_RCVD:
		if (node->sport->domain->femul_enable) {
			fc_header_t *hdr = cbdata->header;
			void *payload = cbdata->payload;
			uint32_t payload_len =  cbdata->payload_len;
			rc = ocs_femul_process_scr(cbdata->io, hdr, payload, payload_len);
			if (rc == 0) {
				break;
			}
		}
		FALL_THROUGH; /* fall through */
#endif

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Wait on a response for a sent PLOGI.
 *
 * <h3 class="desc">Description</h3>
 * State is entered when an initiator-capable node has sent
 * a PLOGI and is waiting for a response.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_wait_plogi_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_PLOGI_RCVD: {
		/* T, or I+T */
		/* received PLOGI with svc parms, go ahead and attach node
		 * when PLOGI that was sent ultimately completes, it'll be a no-op
		 */
		ocs_node_cb_t *cbdata = arg;

		// there is an outstanding PLOGI sent, Let ELS sm know
		// that we don't want to retry it if it times out.
		ocs_assert(node->plogi_els_io, NULL);
		ocs_els_post_event(node->plogi_els_io, OCS_EVT_ELS_DONT_RETRY, false, NULL);

		ocs_node_save_sparms(node, cbdata->payload);
		node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI] = cbdata->io;

		//sm: domain->attached / ocs_node_attach
		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_d_wait_node_attach, NULL);
		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}
		break;
	}

	case OCS_EVT_PRLI_RCVD: {
		/* I, or I+T */
		/* sent PLOGI and before completion was seen, received the
		 * PRLI from the remote node (WCQEs and RCQEs come in on
		 * different queues and order of processing cannot be assumed)
		 * Save OXID so PRLI can be sent after the attach and continue
		 * to wait for PLOGI response
		 */
		ocs_node_cb_t *cbdata = arg;
		fc_prli_payload_t *prli = cbdata->payload;

		ocs_process_prli_payload(node, prli);
		if (prli->type == FC_TYPE_NVME)
			node->ls_rsp_io[OCS_LS_RSP_TYPE_NVME_PRLI] = cbdata->io;
		else if (prli->type == FC_TYPE_FCP)
			node->ls_rsp_io[OCS_LS_RSP_TYPE_FCP_PRLI] = cbdata->io;

		ocs_node_transition(node, __ocs_d_wait_plogi_rsp_recvd_prli, NULL);
		break;
	}

	// TODO this need to be looked at. we could very well be logged in
	case OCS_EVT_LOGO_RCVD: // why don't we do a shutdown here??
	case OCS_EVT_PRLO_RCVD:
	case OCS_EVT_PDISC_RCVD:
	case OCS_EVT_FDISC_RCVD:
	case OCS_EVT_ADISC_RCVD:
	case OCS_EVT_RSCN_RCVD:
	case OCS_EVT_SCR_RCVD: {
		ocs_node_cb_t *cbdata = arg;

		fc_header_t *hdr = cbdata->header;
		node_printf(node, "%s received, sending reject\n", ocs_sm_event_name(evt));
		ocs_send_ls_rjt(cbdata->io, ocs_be16toh(hdr->ox_id),
			FC_REASON_UNABLE_TO_PERFORM, FC_EXPL_NPORT_LOGIN_REQUIRED, 0,
			NULL, NULL);

		break;
	}

	case OCS_EVT_RDP_RCVD: {
		node_printf(node, "%s received, sending reject\n", ocs_sm_event_name(evt));
		ocs_send_rdp_resp(node, arg, OCS_RDP_RJT_NO_LOGIN);
		break;

	}

	case OCS_EVT_SRRS_ELS_REQ_OK: {	/* PLOGI response received */
		ocs_node_cb_t *cbdata = arg;

		/* Completion from PLOGI sent */
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_d_common, __func__)) {
			return NULL;
		}
		//sm: / save sparams, ocs_node_attach
		ocs_node_save_sparms(node, cbdata->els->els_info->els_rsp.virt);
		ocs_display_sparams(node->display_name, "plogi rcvd resp", 0, NULL,
			((uint8_t*)cbdata->els->els_info->els_rsp.virt) + 4);
		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_d_wait_node_attach, NULL);
		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}
		break;
	}

	case OCS_EVT_SRRS_ELS_REQ_RJT:
	case OCS_EVT_SRRS_ELS_REQ_FAIL:	/* PLOGI response received */
		/* PLOGI failed, shutdown the node */
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_d_common, __func__)) {
			return NULL;
		}
		ocs_node_post_event(node, OCS_EVT_SHUTDOWN, NULL);
		break;

	case OCS_EVT_NVME_CMD_RCVD:
	case OCS_EVT_FCP_CMD_RCVD: {
		/* not logged in yet and outstanding PLOGI so don't send LOGO,
		 * just drop
		 */
		node_printf(node, "FCP_CMND received, drop\n");
		break;
	}

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Waiting on a response for a
 *        sent PLOGI.
 *
 * <h3 class="desc">Description</h3>
 * State is entered when an initiator-capable node has sent
 * a PLOGI and is waiting for a response. Before receiving the
 * response, a PRLI was received, implying that the PLOGI was
 * successful.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_wait_plogi_rsp_recvd_prli(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc;
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		/*
		 * Since we've received a PRLI, we have a port login and will
		 * just need to wait for the PLOGI response to do the node
		 * attach and then we can send the LS_ACC for the PRLI. If,
		 * during this time, we receive FCP_CMNDs (which is possible
		 * since we've already sent a PRLI and our peer may have accepted).
		 * At this time, we are not waiting on any other unsolicited
		 * frames to continue with the login process. Thus, it will not
		 * hurt to hold frames here.
		 */
		ocs_node_hold_frames(node);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_SRRS_ELS_REQ_OK:	/* PLOGI response received */
		/* Completion from PLOGI sent */
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_d_common, __func__)) {
			return NULL;
		}
		//sm: / save sparams, ocs_node_attach
		ocs_node_save_sparms(node, cbdata->els->els_info->els_rsp.virt);
		ocs_display_sparams(node->display_name, "plogi rcvd resp", 0, NULL,
			((uint8_t*)cbdata->els->els_info->els_rsp.virt) + 4);
		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_d_wait_node_attach, NULL);
		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}
		break;

	case OCS_EVT_SRRS_ELS_REQ_FAIL:	/* PLOGI response received */
	case OCS_EVT_SRRS_ELS_REQ_RJT:
		/* PLOGI failed, shutdown the node */
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_d_common, __func__)) {
			return NULL;
		}
		ocs_node_post_event(node, OCS_EVT_SHUTDOWN, NULL);
		break;

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Wait for a domain attach.
 *
 * <h3 class="desc">Description</h3>
 * Waits for a domain-attach complete ok event.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_wait_domain_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_node_hold_frames(node);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_DOMAIN_ATTACH_OK:
		ocs_assert(node->sport->domain->attached, NULL);
		//sm: / ocs_node_attach
		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_d_wait_node_attach, NULL);
		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}
		break;

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}
	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Wait for topology
 *        notification
 *
 * <h3 class="desc">Description</h3>
 * Waits for topology notification from fabric node, then
 * attaches domain and node.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_wait_topology_notify(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_node_hold_frames(node);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_SPORT_TOPOLOGY_NOTIFY: {
		ocs_sport_topology_e topology = (ocs_sport_topology_e)arg;
		ocs_io_t *ls_rsp_io = node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI];

		ocs_assert(!node->sport->domain->attached, NULL);
		ocs_assert(ls_rsp_io, NULL);
		node_printf(node, "topology notification, topology=%d\n", topology);

		/* At the time the PLOGI was received, the topology was unknown,
		 * so we didn't know which node would perform the domain attach:
		 * 1. The node from which the PLOGI was sent (p2p) or
		 * 2. The node to which the FLOGI was sent (fabric).
		 */
		if (topology == OCS_SPORT_TOPOLOGY_P2P) {
			/* if this is p2p, need to attach to the domain using the
			 * d_id from the PLOGI received
			 */
			ocs_domain_attach_async(node->sport->domain, ls_rsp_io->els_info->ls_rsp_did, __ocs_d_wait_domain_attach, node);
		} else {
			/* else, if this is fabric, the domain attach should be performed
			 * by the fabric node (node sending FLOGI); just wait for attach
			 * to complete
			 */
			ocs_node_transition(node, __ocs_d_wait_domain_attach, NULL);
		}

		break;
	}

	case OCS_EVT_DOMAIN_ATTACH_OK:
		ocs_assert(node->sport->domain->attached, NULL);
		node_printf(node, "domain attach ok\n");
		//sm: / ocs_node_attach
		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_d_wait_node_attach, NULL);
		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}
		break;

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Wait for a node attach when found by a remote node.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_wait_node_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_io_t *ls_rsp_io = NULL;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_node_hold_frames(node);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_NODE_ATTACH_OK:
		node->attached = TRUE;

		ls_rsp_io = node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI];
		if (ls_rsp_io) {
			/* Normal case for T, or I+T */
			node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI] = NULL;
			ocs_send_plogi_acc(ls_rsp_io, ls_rsp_io->els_info->ls_rsp_oxid, NULL, NULL);
			ocs_node_transition(node, __ocs_d_wait_plogi_acc_cmpl, NULL);
		} else {
			/* Handle all the remaining cases, including PRLI, in port_logged_in state */
			ocs_node_transition(node, __ocs_d_port_logged_in, NULL);
		}

		break;

	case OCS_EVT_NODE_ATTACH_FAIL:
		/* node attach failed, shutdown the node */
		node->attached = FALSE;
		node_printf(node, "node attach failed\n");

		ls_rsp_io = node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI];
		if (ls_rsp_io) {
			node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI] = NULL;
			ocs_send_ls_rjt(ls_rsp_io, ls_rsp_io->els_info->ls_rsp_oxid,
					node->ls_rjt_reason_code, node->ls_rjt_reason_expl_code,
					0, NULL, NULL);
			node->ls_rjt_reason_code = 0;
			node->ls_rjt_reason_expl_code = 0;
		}

		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);
		break;

	/* Handle shutdown events */
	case OCS_EVT_SHUTDOWN:
		node_printf(node, "%s received\n", ocs_sm_event_name(evt));
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_node_transition(node, __ocs_d_wait_async_evt_cmpl_shutdown, NULL);
		break;
	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
		node_printf(node, "%s received\n", ocs_sm_event_name(evt));
		node->shutdown_reason = OCS_NODE_SHUTDOWN_EXPLICIT_LOGO;
		ocs_node_transition(node, __ocs_d_wait_async_evt_cmpl_shutdown, NULL);
		break;
	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		node_printf(node, "%s received\n", ocs_sm_event_name(evt));
		node->shutdown_reason = OCS_NODE_SHUTDOWN_IMPLICIT_LOGO;
		ocs_node_transition(node, __ocs_d_wait_async_evt_cmpl_shutdown, NULL);
		break;
	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Wait for async event completion, then shutdown node.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_wait_async_evt_cmpl_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_node_hold_frames(node);
		break;

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	/* wait for any of these attach events and then shutdown */
	case OCS_EVT_NODE_ATTACH_OK:
		node->attached = TRUE;
		node_printf(node, "Attach evt=%s, proceed to shutdown\n", ocs_sm_event_name(evt));
		ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);
		break;

	case OCS_EVT_NODE_ATTACH_FAIL:
		/* node attach failed, shutdown the node */
		node->attached = FALSE;
		node_printf(node, "Attach evt=%s, proceed to shutdown\n", ocs_sm_event_name(evt));
		ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);
		break;

	case OCS_EVT_NODE_RPI_UPDATE_OK:
	case OCS_EVT_NODE_RPI_UPDATE_FAIL:
	case OCS_EVT_NODE_DEL_INI_COMPLETE:
	case OCS_EVT_NODE_SESS_REG_OK:
	case OCS_EVT_NODE_SESS_REG_FAIL:
		if (cbdata) {
			if (cbdata->io && cbdata->io->cmd_tgt)
				ocs_els_io_free(cbdata->io);

			ocs_free(node->ocs, cbdata, sizeof(*cbdata));
		}

		node->sess_attach_pend--;
		if (!node->sess_attach_pend)
			ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);

		break;

	/* ignore shutdown events as we're already in shutdown path */
	case OCS_EVT_SHUTDOWN:
		/* have default shutdown event take precedence */
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		FALL_THROUGH; /* fall through */
	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		node_printf(node, "%s received\n", ocs_sm_event_name(evt));
		break;

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

static void
ocs_d_handle_prli_req(ocs_node_t *node, ocs_io_t *io)
{
	/* Track the session attach requests */
	node->sess_attach_pend++;

	if (ocs_node_nsler_reg_rpi_update_needed(node,
				io->els_info->ls_rsp_type)) {
		ocs_d_handle_prli_reg_rpi_update(node, io);
	} else {
		ocs_d_handle_prli_backend_process(node, io, io->els_info->ls_rsp_type);
	}
}

static void
ocs_process_prli_pending_req(ocs_node_t *node)
{
	uint8_t i;
	ocs_io_t *ls_rsp_io = NULL;

	/* Since we are past the port_logged_in state, ls_rsp_io for PLOGI must be NULL */
	ocs_assert(!node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI]);

	/* Process any pending PRLI's received */
	for (i = 0; i < OCS_LS_RSP_TYPE_MAX; i++) {
		if (!node->ls_rsp_io[i])
			continue;

		ls_rsp_io = node->ls_rsp_io[i];
		node->ls_rsp_io[i] = NULL;

		ocs_d_handle_prli_req(node, ls_rsp_io);

		/*
		 * Since we processed a PRLI req, the node state machine will be moved
		 * appropriately. Handle any remaining PRLI requests over there.
		 */
		break;
	}
}

static void
ocs_d_handle_rpi_update(ocs_node_t *node, ocs_sm_event_t evt, ocs_node_cb_t *cbdata)
{
	switch (evt) {
	case OCS_EVT_NODE_RPI_UPDATE_OK:
		ocs_assert(cbdata->prli_ctx.ls_rsp_type == OCS_LS_RSP_TYPE_NVME_PRLI);

		node_printf(node, "REG_RPI update done, processing backend session registration\n");
		ocs_d_handle_prli_backend_process(node, cbdata->io, cbdata->prli_ctx.ls_rsp_type);

		ocs_free(node->ocs, cbdata, sizeof(*cbdata));
		break;

	case OCS_EVT_NODE_RPI_UPDATE_FAIL:
		if (cbdata) {
			ocs_assert(cbdata->prli_ctx.ls_rsp_type == OCS_LS_RSP_TYPE_NVME_PRLI);

			node_printf(node, "REG_RPI update failed\n");
			if (cbdata->io) {
				node_printf(node, "Sending LS reject on PRLI request\n");
				ocs_send_ls_rjt(cbdata->io, cbdata->io->els_info->ls_rsp_oxid,
						FC_REASON_UNABLE_TO_PERFORM, FC_EXPL_NO_ADDITIONAL,
						0, NULL, NULL);
			}

			ocs_free(node->ocs, cbdata, sizeof(*cbdata));
		}

		node->sess_attach_pend--;
		break;

	default:
		/* Just a placeholder; this shall never be invoked */
		ocs_assert(FALSE);
	}
}

static void
ocs_d_handle_sess_rsp(ocs_node_t *node, ocs_sm_event_t evt, ocs_node_cb_t *cbdata)
{
	switch (evt) {
	case OCS_EVT_NODE_SESS_REG_OK:
		ocs_assert(cbdata != NULL);

		/* Send PRLI acc */
		if (cbdata->prli_ctx.ls_rsp_type == OCS_LS_RSP_TYPE_NVME_PRLI) {
			if (cbdata->io)
				ocs_send_prli_acc(cbdata->io, cbdata->io->els_info->ls_rsp_oxid,
						  FC_TYPE_NVME, NULL, NULL);
		} else if (cbdata->prli_ctx.ls_rsp_type == OCS_LS_RSP_TYPE_FCP_PRLI) {
			node->fcp_enabled = TRUE;
			if (cbdata->io)
				ocs_send_prli_acc(cbdata->io, cbdata->io->els_info->ls_rsp_oxid,
						  FC_TYPE_FCP, NULL, NULL);
		}

		ocs_free(node->ocs, cbdata, sizeof(*cbdata));

		node->sess_attach_pend--;
		break;

	case OCS_EVT_NODE_SESS_REG_FAIL:
		if (cbdata) {
			if (cbdata->prli_ctx.ls_rsp_type == OCS_LS_RSP_TYPE_NVME_PRLI)
				node->tgt_fct_prli_success &= ~OCS_TARGET_TYPE_NVME;
			else if (cbdata->prli_ctx.ls_rsp_type == OCS_LS_RSP_TYPE_FCP_PRLI)
				node->tgt_fct_prli_success &= ~OCS_TARGET_TYPE_FCP;

			/* Send PRLI reject */
			node_printf(node, "Backend session registration failed\n");
			if (cbdata->io) {
				node_printf(node, "Sending LS reject on PRLI request\n");
				ocs_send_ls_rjt(cbdata->io, cbdata->io->els_info->ls_rsp_oxid,
						cbdata->status, cbdata->ext_status,
						0, NULL, NULL);
			}

			ocs_free(node->ocs, cbdata, sizeof(*cbdata));
		}

		node->sess_attach_pend--;
		break;

	default:
		/* Just a placeholder; this shall never be invoked */
		ocs_assert(FALSE);
	}
}

static void
ocs_d_handle_prli_cmpl(ocs_node_t *node, ocs_sm_event_t evt, ocs_node_cb_t *cbdata, const char *funcname)
{
	/* PRLI accepted/rejected by remote; normal case for I or I+T */
	if (node_check_els_req(&node->sm, evt, cbdata, FC_ELS_CMD_PRLI, __ocs_d_common, funcname))
		return;

	if (evt == OCS_EVT_SRRS_ELS_REQ_OK) {
		fc_prli_payload_t *prli = cbdata->els->els_info->els_rsp.virt;

		/*
		 * Process this PRLI successful completion only if the remote node has target capability.
		 * Else, wait for the other side to trigger a PRLI request before proceeding further. If
		 * the remote node has both I+T capabilities and we have target functionality, then we may
		 * need to register a session with the backend because the remote node might not send an
		 * addl. PRLI. However, after processing this PRLI completion, if we still receive a PRLI
		 * from the other side, we will treat it as a duplicate request and handle it accordingly.
		 */
		ocs_process_prli_payload(node, prli);
		if (prli->type == FC_TYPE_NVME && node->nvme_tgt) {
			if (!node->remote_nvme_prli_rcvd) {
				cbdata->els->els_info->ls_rsp_oxid = OCS_INVALID_FC_TASK_TAG;
				cbdata->els->els_info->ls_rsp_type = OCS_LS_RSP_TYPE_NVME_PRLI;
				ocs_d_handle_prli_req(node, cbdata->els);
			}

			node->ini_fct_prli_success |= OCS_INITIATOR_TYPE_NVME;
		} else if (prli->type == FC_TYPE_FCP && node->targ) {
			if (!node->remote_fcp_prli_rcvd) {
				cbdata->els->els_info->ls_rsp_oxid = OCS_INVALID_FC_TASK_TAG;
				cbdata->els->els_info->ls_rsp_type = OCS_LS_RSP_TYPE_FCP_PRLI;
				ocs_d_handle_prli_req(node, cbdata->els);
			}

			node->ini_fct_prli_success |= OCS_INITIATOR_TYPE_FCP;
		}
	}

	/* Proceed if atleast one of the PRLI's (NVMe/FCP) went through successfully */
	node->prli_rsp_pend--;
	if (!node->prli_rsp_pend && node->ini_fct_prli_success) {
		ocs_notify_new_target(node);
		ocs_node_transition(node, __ocs_d_device_ready, NULL);
	}
}

/**
 * @brief Callback handler when a async flush for abts completes
 *
 * @param arg Pointer to node callback data
 *
 * @return Returns NULL.
 */
static void ocs_abts_flush_callback(void *arg)
{
	ocs_node_cb_t *cbdata = arg;
	ocs_sport_t *sport;

	ocs_assert(cbdata->io);
	ocs_assert(cbdata->io->node);
	ocs_assert(cbdata->io->node->sport);

	sport = cbdata->io->node->sport;

	ocs_ref_get(&sport->ref);
	ocs_sport_lock(sport);

	ocs_node_post_event(cbdata->io->node, OCS_EVT_ABTS_FLUSH_COMPLETE, cbdata);
	ocs_free(sport->ocs, cbdata->header, sizeof(fc_header_t));
	ocs_free(sport->ocs, cbdata->payload, cbdata->payload_len);
	ocs_free(sport->ocs, cbdata, sizeof(ocs_node_cb_t));

	ocs_sport_unlock(sport);
	ocs_ref_put(&sport->ref);

}

/**
 * @brief Generate a rq flush request in response to an ABTS
 *
 * @param node - remote node state machine context
 *        cbdata - callback data associated with abts
 *
 * @return Returns NULL.
 */
static void ocs_abts_flush_rqs(ocs_node_t *node, ocs_node_cb_t *cbdata)
{
	ocs_node_cb_t *post_cbdata = NULL;
	ocs_t *ocs = node->ocs;

	post_cbdata = ocs_malloc(node->ocs, sizeof(ocs_node_cb_t), OCS_M_ZERO | OCS_M_NOWAIT);
	if (post_cbdata) {
		post_cbdata->header = ocs_malloc(node->ocs, sizeof(fc_header_t), OCS_M_ZERO | OCS_M_NOWAIT);
		if (!post_cbdata->header) {
			node_printf_err(node, "Header buff alloc is failed\n");
			goto alloc_fail;
		}
		post_cbdata->payload = ocs_malloc(node->ocs, cbdata->payload_len, OCS_M_ZERO | OCS_M_NOWAIT);
		if (!post_cbdata->payload) {
			node_printf_err(node, "Payload buff alloc failed\n");
			goto alloc_fail;
		}
	} else {
		node_printf_err(node, "Unable to handle ABTS due to insufficient resources\n");
		goto alloc_fail;
	}

	/* Fill entire cbdata into to local buffer individually*/
	post_cbdata->io = cbdata->io;
	post_cbdata->els = cbdata->els;
	post_cbdata->status = cbdata->status;
	post_cbdata->ext_status = cbdata->ext_status;
	post_cbdata->payload_len = cbdata->payload_len;
	post_cbdata->flush_rqs_completed = cbdata->flush_rqs_completed;
	post_cbdata->node = cbdata->node;

	ocs_memcpy(&post_cbdata->prli_ctx, &cbdata->prli_ctx, sizeof(cbdata->prli_ctx));
	ocs_memcpy(post_cbdata->header, cbdata->header, sizeof(fc_header_t));
	ocs_memcpy(post_cbdata->payload, cbdata->payload, cbdata->payload_len);

	node_printf_ratelimited(node, "ABTS received, Flushing rqs. Marker category %d\n",
				ocs->hal.scsi_mrq_marker_category);

	if (ocs_hal_rq_marker_gen_req(&node->ocs->hal, ocs->hal.scsi_mrq_marker_category,
				ocs_abts_flush_callback,
				(void *) post_cbdata) != 0) {
		node_printf_ratelimited(node, "ABTS rq flush failed \n");
		/* Could not trigger flush, since hardware is unresponsive,
		 * post the flush complete event here to complete abts
		 * handling  */
		ocs_node_post_event(node, OCS_EVT_ABTS_FLUSH_COMPLETE,
				post_cbdata);
		goto fail;
	}

	return;

alloc_fail:
	ocs_scsi_io_free(cbdata->io);
fail:
	if (post_cbdata) {
		if (post_cbdata->header)
			ocs_free(node->ocs, post_cbdata->header, sizeof(fc_header_t));
		if (post_cbdata->payload)
			ocs_free(node->ocs, post_cbdata->payload, post_cbdata->payload_len);
		ocs_free(node->ocs, post_cbdata, sizeof(ocs_node_cb_t));
	}
}

void
ocs_process_abts(ocs_node_t *node, ocs_node_cb_t *cbdata)
{
	ocs_t *ocs = node->ocs;
	fc_header_t *hdr = cbdata->header;
	ocs_io_t *tmfio = cbdata->io;
	ocs_io_t *io_to_abort;
	uint16_t ox_id = ocs_be16toh(hdr->ox_id);
	uint16_t rx_id = ocs_be16toh(hdr->rx_id);

	io_to_abort = ocs_scsi_io_find_and_ref_get(tmfio, ox_id, rx_id);
	if (io_to_abort) {
		ocs_scsi_process_abts(node, tmfio, io_to_abort, ox_id, rx_id);

		return;
	}

	/* IO was not found */
	node_printf_ratelimited(node, "Abort request: IO not found, " \
				"[%016" PRIX64".%016" PRIX64"] " \
				"ox_id [%04x] rx_id [%04x]\n", node->sport->wwpn,
				ocs_node_get_wwpn(node), ox_id, rx_id);

	if (ocs->hal.scsi_rq_pair_count > 1 && ocs->hal.scsi_mrq_marker_category &&
	    !cbdata->flush_rqs_completed) {
		ocs_abts_flush_rqs(node, cbdata);

		return;
	} else {
		if (ocs_tgt_nvme_enabled(node->ocs) &&
		    !ocs_nvme_process_abts(node, ox_id, rx_id)) {
			ocs_scsi_io_free(tmfio);
		} else {
			/* Send BA_RJT */
			ocs_bls_send_rjt_hdr(tmfio, hdr);
		}
	}
}

/**
 * @ingroup device_sm
 * @brief Device node state machine: Port is logged in.
 *
 * <h3 class="desc">Description</h3>
 * This state is entered when a remote port has completed port login (PLOGI).
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process
 * @param arg Per event optional argument
 *
 * @return Returns NULL.
 */
void *
__ocs_d_port_logged_in(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_cb_t *cbdata = arg;

	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		/* Normal case for I or I+T */
		if (node->sport->enable_ini && !FC_ADDR_IS_DOMAIN_CTRL(node->rnode.fc_id) && !node->prli_rsp_pend) {
			ocs_assert(node->sport->ini_fc_types, NULL);

			//sm: if enable_ini / send PRLI
			if (ocs_ini_scsi_enabled(node->ocs)) {
				if (ocs_send_prli(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT, OCS_FC_ELS_DEFAULT_RETRIES,
						  NULL, NULL, FC_TYPE_FCP) == NULL) {
					node_printf(node, "Failed to send SCSI PRLI req, shutting down node\n");
					ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
				} else {
					node->prli_rsp_pend++;
				}
			}

			if (ocs_ini_nvme_enabled(node->ocs)) {
				if (ocs_send_prli(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT, OCS_FC_ELS_DEFAULT_RETRIES,
						  NULL, NULL, FC_TYPE_NVME) == NULL) {
					node_printf(node, "Failed to send NVMe PRLI req, shutting down node\n");
					ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
				} else {
					node->prli_rsp_pend++;
				}
			}
			/* can now expect ELS_REQ_OK/FAIL/RJT */
		}

		/* Check if we need to process any pending PRLI requests */
		ocs_process_prli_pending_req(node);
		break;

	case OCS_EVT_NVME_CMD_RCVD:
		/* For target functionality, send PRLO and drop the CMD frame */
		if (node->sport->enable_tgt) {
			if (ocs_send_prlo(node, FC_TYPE_NVME, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
					  OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL))
				node_printf(node, "%s received, PRLO sent\n", ocs_sm_event_name(evt));
		}

		break;

	case OCS_EVT_FCP_CMD_RCVD:
		/* For target functionality, send PRLO and drop the CMD frame */
		if (node->sport->enable_tgt) {
			if (ocs_send_prlo(node, FC_TYPE_FCP, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
					  OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL))
				node_printf(node, "%s received, PRLO sent\n", ocs_sm_event_name(evt));
		}

		break;

	case OCS_EVT_PRLI_RCVD:
		/* Normal for T or I+T */
		ocs_process_prli_payload(node, cbdata->payload);
		ocs_d_handle_prli_req(node, cbdata->io);
		break;

	case OCS_EVT_NODE_RPI_UPDATE_OK:
		ocs_d_handle_rpi_update(node, evt, cbdata);
		break;

	case OCS_EVT_NODE_RPI_UPDATE_FAIL:
		ocs_d_handle_rpi_update(node, evt, cbdata);

		/* Check if we need to process any pending PRLI requests */
		ocs_process_prli_pending_req(node);
		break;

	case OCS_EVT_NODE_DEL_INI_COMPLETE:
		if (cbdata->io->els_info->ls_rsp_type == OCS_LS_RSP_TYPE_NVME_PRLI) {
			node->tgt_fct_prli_success &= ~OCS_TARGET_TYPE_NVME;
			ocs_nvme_node_resume(node);
		} else if (cbdata->io->els_info->ls_rsp_type == OCS_LS_RSP_TYPE_FCP_PRLI) {
			node->tgt_fct_prli_success &= ~OCS_TARGET_TYPE_FCP;
			ocs_scsi_node_resume(node);
		}

		/* If we are here, then we must be processing a duplicate PRLI request */
		ocs_d_handle_sess_reg(node, cbdata);
		break;

	case OCS_EVT_NODE_SESS_REG_OK:
		ocs_d_handle_sess_rsp(node, evt, cbdata);

		/* Now move the node state machine to device_ready state */
		ocs_node_transition(node, __ocs_d_device_ready, NULL);
		break;

	case OCS_EVT_NODE_SESS_REG_FAIL:
		ocs_d_handle_sess_rsp(node, evt, cbdata);

		/* Check if we need to process any pending PRLI requests */
		ocs_process_prli_pending_req(node);
		break;

	case OCS_EVT_SRRS_ELS_REQ_OK:
	case OCS_EVT_SRRS_ELS_REQ_RJT: {
		fc_els_gen_t *els_gen = (fc_els_gen_t *)cbdata->els->els_info->els_req.virt;

		/* We can get PRLO completion due to OCS_EVT_NVME_CMD_RCVD/OCS_EVT_FCP_CMD_RCVD */
		if (FC_ELS_CMD_PRLO == els_gen->command_code)
			break;

		ocs_d_handle_prli_cmpl(node, evt, cbdata, __func__);
		break;
	}

	case OCS_EVT_SRRS_ELS_REQ_FAIL: {
		fc_els_gen_t *els_gen = (fc_els_gen_t *)cbdata->els->els_info->els_req.virt;

		/* We can get PRLO completion due to OCS_EVT_NVME_CMD_RCVD/OCS_EVT_FCP_CMD_RCVD */
		if (FC_ELS_CMD_PRLO == els_gen->command_code)
			break;

		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PRLI, __ocs_d_common, __func__))
			return NULL;

		/* For I only, shutdown node */
		node->prli_rsp_pend--;
		if (!node->prli_rsp_pend && !node->sport->enable_tgt)
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN, NULL);

		break;
	}

	case OCS_EVT_SRRS_ELS_CMPL_OK:
		/* Normal T, I+T, target-server rejected the process login */
		/* This would be received only in the case where we sent LS_RJT for the PRLI,
		 * so do nothing. (note: as T only we could shutdown the node)
		 */
		break;

	case OCS_EVT_PLOGI_RCVD:
		//sm: / save sparams, set send_plogi_acc, post implicit logout
		/* Save plogi parameters */
		ocs_node_save_sparms(node, cbdata->payload);
		node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI] = cbdata->io;

		/* Restart node attach with new service paramters, and send ACC */
		ocs_node_post_event(node, OCS_EVT_SHUTDOWN_IMPLICIT_LOGO, NULL);
		break;

	case OCS_EVT_LOGO_RCVD: {
		/* I, T, I+T */
		node_printf(node, "%s received attached=%d\n", ocs_sm_event_name(evt), node->attached);

		//sm: save logo and send acc after the all the ios are completed.
		node->ls_rsp_io[OCS_LS_RSP_TYPE_LOGO] = cbdata->io;

		ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		break;
	}

	case OCS_EVT_ADISC_RCVD: {
		fc_header_t *hdr = cbdata->header;

		ocs_send_adisc_acc(cbdata->io, ocs_be16toh(hdr->ox_id), NULL, NULL);
		break;
	}

	case OCS_EVT_ABTS_RCVD:
		ocs_process_abts(node, cbdata);
		break;

	/* Handle shutdown events */
	case OCS_EVT_SHUTDOWN:
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		if (node->sess_attach_pend)
			ocs_node_transition(node, __ocs_d_wait_async_evt_cmpl_shutdown, NULL);
		else
			ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);

		break;

	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
		node->shutdown_reason = OCS_NODE_SHUTDOWN_EXPLICIT_LOGO;
		if (node->sess_attach_pend)
			ocs_node_transition(node, __ocs_d_wait_async_evt_cmpl_shutdown, NULL);
		else
			ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);

		break;

	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		node->shutdown_reason = OCS_NODE_SHUTDOWN_IMPLICIT_LOGO;
		if (node->sess_attach_pend)
			ocs_node_transition(node, __ocs_d_wait_async_evt_cmpl_shutdown, NULL);
		else
			ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);

		break;

	case OCS_EVT_NODE_MISSING:
		node_printf_ratelimited(node, "%s received attached=%d\n",
					ocs_sm_event_name(evt), node->attached);
		if (node->sport->enable_rscn)
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN, NULL);

		break;

#if defined(ENABLE_FABRIC_EMULATION)
	/*
	 * FC_GS received for directory server when using fabric emulation
	 */
	case OCS_EVT_RFT_ID_RCVD:
	case OCS_EVT_RFF_ID_RCVD:
	case OCS_EVT_GNN_ID_RCVD:
	case OCS_EVT_GPN_ID_RCVD:
	case OCS_EVT_GFPN_ID_RCVD:
	case OCS_EVT_GFF_ID_RCVD:
	case OCS_EVT_GID_FT_RCVD:
	case OCS_EVT_GID_PT_RCVD:
	case OCS_EVT_RPN_ID_RCVD:
	case OCS_EVT_RNN_ID_RCVD:
	case OCS_EVT_RCS_ID_RCVD:
	case OCS_EVT_RSNN_NN_RCVD:
	case OCS_EVT_RSPN_ID_RCVD:
	case OCS_EVT_RHBA_RCVD:
	case OCS_EVT_RPA_RCVD:
		if (node->sport->domain->femul_enable) {
			fc_header_t *hdr;
			void *payload;
			uint32_t payload_len;

			ocs_assert(cbdata != NULL, NULL);
			ocs_assert(cbdata->header != NULL, NULL);
			ocs_assert(cbdata->payload != NULL, NULL);
			hdr = cbdata->header;
			payload = cbdata->payload;
			payload_len = cbdata->payload_len;

			ocs_femul_process_fc_gs(__func__, cbdata->io, evt, hdr, payload, payload_len);
			break;
		}

		FALL_THROUGH; /* fall through */
#endif

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}


/**
 * @ingroup device_sm
 * @brief Device node state machine: Device is ready.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_d_device_ready(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_cb_t *cbdata = arg;

	std_node_state_decl();

	if (evt != OCS_EVT_FCP_CMD_RCVD && evt != OCS_EVT_NVME_CMD_RCVD &&
	    evt != OCS_EVT_NODE_LAST_ACTIVE_IO) {
		node_sm_trace();
	}

	switch(evt) {
	case OCS_EVT_ENTER:
		/* Check if we need to process any pending PRLI requests */
		ocs_process_prli_pending_req(node);
		break;

	case OCS_EVT_EXIT:
		node->fcp_enabled = FALSE;
		break;

	case OCS_EVT_PLOGI_RCVD:
		//sm: / save sparams, set send_plogi_acc, post implicit logout
		/* Save plogi parameters */
		ocs_node_save_sparms(node, cbdata->payload);
		node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI] = cbdata->io;

		/* Restart node attach with new service paramters, and send ACC */
		ocs_node_post_event(node, OCS_EVT_SHUTDOWN_IMPLICIT_LOGO, NULL);
		break;

	case OCS_EVT_PDISC_RCVD:
		if (pdisc_clear_nexus_state(node->ocs)) {
			/* Service parameters contained in the PDISC shall be ignored */
			node->ls_rsp_io[OCS_LS_RSP_TYPE_PLOGI] = cbdata->io;
			/* Restart node attach with old service paramters, and send ACC */
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_IMPLICIT_LOGO, NULL);
		} else {
			fc_header_t *hdr = cbdata->header;

			ocs_send_plogi_acc(cbdata->io, ocs_be16toh(hdr->ox_id), NULL, NULL);
		}

		break;

	case OCS_EVT_PRLI_RCVD:
		/* T, I+T: remote initiator is slow to get started */
		ocs_process_prli_payload(node, cbdata->payload);
		ocs_d_handle_prli_req(node, cbdata->io);
		break;

	case OCS_EVT_NODE_RPI_UPDATE_OK:
	case OCS_EVT_NODE_RPI_UPDATE_FAIL:
		ocs_d_handle_rpi_update(node, evt, cbdata);
		break;

	case OCS_EVT_NODE_DEL_INI_COMPLETE:
		if (cbdata->io->els_info->ls_rsp_type == OCS_LS_RSP_TYPE_NVME_PRLI) {
			node->tgt_fct_prli_success &= ~OCS_TARGET_TYPE_NVME;
			ocs_nvme_node_resume(node);
		} else if (cbdata->io->els_info->ls_rsp_type == OCS_LS_RSP_TYPE_FCP_PRLI) {
			node->tgt_fct_prli_success &= ~OCS_TARGET_TYPE_FCP;
			ocs_scsi_node_resume(node);
		}

		/* If we are here, then we must be processing a duplicate PRLI request */
		ocs_d_handle_sess_reg(node, cbdata);
		break;

	case OCS_EVT_NODE_SESS_REG_OK:
	case OCS_EVT_NODE_SESS_REG_FAIL:
		ocs_d_handle_sess_rsp(node, evt, cbdata);
		break;

	case OCS_EVT_PRLO_RCVD: {
		fc_prlo_payload_t *prlo = cbdata->payload;

		ocs_assert(prlo->type == FC_TYPE_NVME || prlo->type == FC_TYPE_FCP, NULL);

		//sm: save PRLO and send acc after the all the ios are completed.
		node->ls_rsp_io[cbdata->io->els_info->ls_rsp_type] = cbdata->io;

		ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		break;
	}

	case OCS_EVT_LOGO_RCVD: {
		node_printf(node, "%s received attached=%d\n", ocs_sm_event_name(evt), node->attached);

		//sm: save logo and send acc after the all the ios are completed.
		node->ls_rsp_io[OCS_LS_RSP_TYPE_LOGO] = cbdata->io;

		ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		break;
	}

	case OCS_EVT_ADISC_RCVD: {
		fc_header_t *hdr = cbdata->header;

		//sm: / send ADISC acc
		ocs_send_adisc_acc(cbdata->io, ocs_be16toh(hdr->ox_id), NULL, NULL);
		break;
	}

	case OCS_EVT_RRQ_RCVD: {
		fc_header_t *hdr = cbdata->header;

		/* Send LS_ACC */
		ocs_send_ls_acc(cbdata->io, ocs_be16toh(hdr->ox_id), NULL, NULL);
		break;
	}

	case OCS_EVT_RDP_RCVD:
		node_printf(node, "%s received, processing payload\n", ocs_sm_event_name(evt));
		ocs_send_rdp_resp(node, arg, 0);
		break;

	case OCS_EVT_LCB_RCVD:
		ocs_els_process_lcb_rcvd(node, arg);
		break;

	case OCS_EVT_ABTS_RCVD:
		ocs_process_abts(node, cbdata);
		break;

	case OCS_EVT_FLUSH_BLS_RCVD:
		ocs_process_flush_bls(cbdata);
		break;

	case OCS_EVT_NODE_ACTIVE_IO_LIST_EMPTY:
	case OCS_EVT_NODE_REFOUND:
		break;

	case OCS_EVT_NODE_MISSING:
		if (node->sport->enable_rscn)
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN, NULL);

		break;

	case OCS_EVT_SRRS_ELS_REQ_OK:
	case OCS_EVT_SRRS_ELS_REQ_RJT: {
		fc_els_gen_t *els_gen = (fc_els_gen_t *)cbdata->els->els_info->els_req.virt;

		/* We can get PRLO completion due to OCS_EVT_NVME_CMD_RCVD/OCS_EVT_FCP_CMD_RCVD */
		if (FC_ELS_CMD_PRLO == els_gen->command_code)
			break;

		ocs_d_handle_prli_cmpl(node, evt, cbdata, __func__);
		break;
	}

	case OCS_EVT_SRRS_ELS_CMPL_OK:
		/* T, or I+T, PRLI accept completed ok */
		break;

	case OCS_EVT_SRRS_ELS_CMPL_FAIL:
		/* T, or I+T, PRLI accept failed to complete */
		node_printf(node, "Failed to send PRLI LS_ACC\n");
		break;

	case OCS_EVT_FCP_CMD_RCVD:
		if (node->fcp_enabled)
			node_printf_err(node, "FCP command received before sending PRLI ACC\n");

		/* Spec expects us to send prlo and drop the FCP command */
		if (ocs_send_prlo(node, FC_TYPE_FCP, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
				  OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL))
			node_printf(node, "%s received, PRLO sent\n", ocs_sm_event_name(evt));

		break;

	case OCS_EVT_NVME_CMD_RCVD:
		/* Spec expects us to send prlo and drop the NVME command */
		if (ocs_send_prlo(node, FC_TYPE_NVME, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
				  OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL))
			node_printf(node, "%s received, PRLO sent\n", ocs_sm_event_name(evt));

		break;

	/* Handle shutdown events */
	case OCS_EVT_SHUTDOWN:
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		if (node->sess_attach_pend)
			ocs_node_transition(node, __ocs_d_wait_async_evt_cmpl_shutdown, NULL);
		else
			ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);

		break;

	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
		node->shutdown_reason = OCS_NODE_SHUTDOWN_EXPLICIT_LOGO;
		if (node->sess_attach_pend)
			ocs_node_transition(node, __ocs_d_wait_async_evt_cmpl_shutdown, NULL);
		else
			ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);

		break;

	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		node->shutdown_reason = OCS_NODE_SHUTDOWN_IMPLICIT_LOGO;
		if (node->sess_attach_pend)
			ocs_node_transition(node, __ocs_d_wait_async_evt_cmpl_shutdown, NULL);
		else
			ocs_node_transition(node, __ocs_d_initiate_shutdown, NULL);

		break;

	default:
		__ocs_d_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}
