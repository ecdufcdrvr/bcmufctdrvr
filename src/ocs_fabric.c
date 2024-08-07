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
 *
 * This file implements remote node state machines for:
 * - Fabric logins.
 * - Fabric controller events.
 * - Name/directory services interaction.
 * - Point-to-point logins.
 */

/*!
@defgroup fabric_sm Node State Machine: Fabric States
@defgroup ns_sm Node State Machine: Name/Directory Services States
@defgroup p2p_sm Node State Machine: Point-to-Point Node States
*/

#include "ocs.h"
#include "ocs_fabric.h"
#include "ocs_els.h"
#include "ocs_device.h"
#include "ocs_compat.h"

static void ocs_fabric_initiate_shutdown(ocs_node_t *node);
static void * __ocs_fabric_common(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
static int32_t ocs_start_ns_node(ocs_sport_t *sport);
static int32_t ocs_start_mgmt_srv_node(ocs_sport_t *sport);
static int32_t ocs_start_fabctl_node(ocs_sport_t *sport);
static int32_t ocs_process_gidpt_payload(ocs_node_t *node, fcct_gidpt_acc_t *gidpt, uint32_t gidpt_len);
static void ocs_process_rscn(ocs_node_t *node, ocs_node_cb_t *cbdata);
static uint64_t ocs_get_wwpn(fc_plogi_payload_t *sp);
static void gidpt_delay_timer_cb(void *arg);
static int32_t ocs_find_new_targets(ocs_node_t *node, fcct_gidpt_acc_t *gidpt, uint32_t gidpt_len);
static void ocs_find_next_target(ocs_node_t *node, ocs_discover_target_ctx_t *ctx);

#define FDMI_ATTR_TYPE_LEN_SIZE	4
#define OCS_MAX_CT_SIZE (60 * 4096)

/* FDMI Port Speed definitions - FC-GS-8 */
#define OCS_PORTSPEED_1GFC              0x00000001      /* 1G FC */
#define OCS_PORTSPEED_2GFC              0x00000002      /* 2G FC */
#define OCS_PORTSPEED_4GFC              0x00000008      /* 4G FC */
#define OCS_PORTSPEED_8GFC              0x00000010      /* 8G FC */
#define OCS_PORTSPEED_16GFC             0x00000020      /* 16G FC */
#define OCS_PORTSPEED_32GFC             0x00000040      /* 32G FC */
#define OCS_PORTSPEED_64GFC             0x00000400      /* 64G FC */
#define OCS_PORTSPEED_128GFC            0x00000200      /* 128G FC */
#define OCS_PORTSPEED_256GFC            0x00000800      /* 256G FC */
#define OCS_PORTSPEED_UNKNOWN           0x00008000      /* Unknown */

/* Defines for PORT port type attribute */
#define OCS_FDMI_PORTTYPE_UNKNOWN      0
#define OCS_FDMI_PORTTYPE_NPORT        1
#define OCS_FDMI_PORTTYPE_NLPORT       2

/* Defines for PORT port state attribute */
#define OCS_FDMI_PORTSTATE_OFFLINE     1
#define OCS_FDMI_PORTSTATE_ONLINE      2

/**
 * @ingroup fabric_sm
 * @brief Fabric node state machine: Initial state.
 *
 * @par Description
 * Send an FLOGI to a well-known fabric.
 *
 * @param ctx Remote node sm context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fabric_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_REENTER:	// not sure why we're getting these ...
		ocs_log_debug(node->ocs, ">>> reenter !!\n");
		FALL_THROUGH; /* fall through */
	case OCS_EVT_ENTER:
		ocs_node_auth_init(node);

		// sm: / send FLOGI
		node->ocs->sw_feature_cap = 0;
		if (ocs_send_flogi(node, OCS_FC_FLOGI_TIMEOUT_SEC,
				   OCS_FC_FLOGI_MAX_RETRIES, NULL, NULL) == NULL) {
			node_printf(node, "Failed to send FLOGI req, shutting down node\n");
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		} else {
			ocs_node_transition(node, __ocs_fabric_flogi_wait_rsp, NULL);
		}
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		break;
	}

	return NULL;
}

/**
 * @ingroup fabric_sm
 * @brief Set sport topology.
 *
 * @par Description
 * Set sport topology.
 *
 * @param node Pointer to the node for which the topology is set.
 * @param topology Topology to set.
 *
 * @return Returns NULL.
 */
void
ocs_fabric_set_topology(ocs_node_t *node, ocs_sport_topology_e topology)
{
	node->sport->topology = topology;
}

/**
 * @ingroup fabric_sm
 * @brief Notify sport topology.
 *
 * @par Description
 * Set sport topology.
 *
 * @param node Pointer to the node for which the topology is set.
 *
 * @return Returns NULL.
 */
void
ocs_fabric_notify_topology(ocs_node_t *node)
{
	ocs_node_t *tmp_node;
	ocs_node_t *next;
	ocs_sport_topology_e topology = node->sport->topology;

	/* now loop through the nodes in the sport and send topology notification */
	ocs_sport_lock(node->sport);
	ocs_list_foreach_safe(&node->sport->node_list, tmp_node, next) {
		if (tmp_node != node) {
			ocs_node_post_event(tmp_node, OCS_EVT_SPORT_TOPOLOGY_NOTIFY, (void *)topology);
		}
	}
	ocs_sport_unlock(node->sport);
}

/**
 * @ingroup fabric_sm
 * @brief Fabric node state machine: Wait for an FLOGI response.
 *
 * @par Description
 * Wait for an FLOGI response event.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_fabric_flogi_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK: {

		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_FLOGI, __ocs_fabric_common, __func__)) {
			return NULL;
		}

		ocs_domain_save_sparms(node->sport->domain, cbdata->els->els_info->els_rsp.virt);

		ocs_display_sparams(node->display_name, "flogi rcvd resp", 0, NULL,
			((uint8_t*)cbdata->els->els_info->els_rsp.virt) + 4);

		/* Check to see if the fabric is an F_PORT or and N_PORT */
		if (ocs_rnode_is_nport(cbdata->els->els_info->els_rsp.virt)) {
			// sm: if nport and p2p_winner / ocs_domain_attach
			ocs_fabric_set_topology(node, OCS_SPORT_TOPOLOGY_P2P);
			if (ocs_p2p_setup(node->sport)) {
				node_printf(node, "p2p setup failed, shutting down node\n");
				node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
				ocs_fabric_initiate_shutdown(node);
			} else {
				if (node->sport->p2p_winner) {
					if (!node->sport->domain->attached) {
						node_printf(node, "p2p winner, domain not attached\n");
						ocs_domain_attach_async(node->sport->domain, node->sport->p2p_port_id, __ocs_p2p_wait_domain_attach, node);
					} else {
						/* already attached, just send ATTACH_OK */
						node_printf(node, "p2p winner, domain already attached\n");
						ocs_node_transition(node, __ocs_p2p_wait_domain_attach, NULL);
						ocs_node_post_event(node, OCS_EVT_DOMAIN_ATTACH_OK, NULL);
					}
				} else {
					/* peer is p2p winner; PLOGI will be received on the
					 * remote SID=1 node; this node has served its purpose
					 */
					node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
					ocs_fabric_initiate_shutdown(node);
				}
			}
		} else {
			fc_plogi_payload_t *flogi_acc = cbdata->els->els_info->els_rsp.virt;

			/* Store FLOGI.LS_ACC.fcsp value for later use */
			node->auth.dhchap.flogi_acc_fcsp =
				ocs_be32toh(flogi_acc->common_service_parameters[1]) &
				FC_FLOGI_CSP_W1_FCSP;

			// sm: if not nport / ocs_domain_attach
			/* ext_status has the fc_id, attach domain */
			if (ocs_rnode_is_npiv_capable(cbdata->els->els_info->els_rsp.virt)) {
				ocs_log_debug(node->ocs, "NPIV is enabled at switch side\n");
				node->ocs->sw_feature_cap |= (OCS_FLOGI_FINISH | OCS_SUPPORT_NPIV);
			} else {
				ocs_log_debug(node->ocs, "NPIV is not supported at switch side\n");
				node->ocs->sw_feature_cap |= OCS_FLOGI_FINISH;
			}

			ocs_fabric_set_topology(node, OCS_SPORT_TOPOLOGY_FABRIC);
			ocs_fabric_notify_topology(node);

			ocs_assert(!node->sport->domain->attached, NULL);
			ocs_domain_attach_async(node->sport->domain, cbdata->ext_status, __ocs_fabric_wait_domain_attach, node);
		}

		break;
	}

	case OCS_EVT_ELS_REQ_ABORTED:
	case OCS_EVT_SRRS_ELS_REQ_RJT:
	case OCS_EVT_SRRS_ELS_REQ_FAIL: {
		ocs_sport_t *sport = node->sport;
		/*
		 * with these errors, we have no recovery, so shutdown the sport, leave the link
		 * up and the domain ready
		 */
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_FLOGI, __ocs_fabric_common, __func__)) {
			return NULL;
		}

		if (node->sport->topology == OCS_SPORT_TOPOLOGY_P2P && !node->sport->p2p_winner) {
			node_printf(node, "FLOGI failed, peer p2p winner, shutdown node\n");
			node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
			ocs_fabric_initiate_shutdown(node);
			break;
		}

		node_printf(node, "FLOGI failed evt=%s, shutting down sport [%s]\n", ocs_sm_event_name(evt),
			sport->display_name);
		ocs_sm_post_event(&sport->sm, OCS_EVT_SHUTDOWN, NULL);
		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		break;
	}

	return NULL;
}

/**
 * @ingroup fabric_sm
 * @brief Fabric node state machine: Initial state for a virtual port.
 *
 * @par Description
 * State entered when a virtual port is created. Send FDISC.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_vport_fabric_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		//sm: / send FDISC
		if (ocs_send_fdisc(node, OCS_FC_FLOGI_TIMEOUT_SEC,
				   OCS_FC_FLOGI_MAX_RETRIES, NULL, NULL) == NULL) {
			node_printf(node, "Failed to send FDISC req, shutting down node\n");
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		} else {
			ocs_node_transition(node, __ocs_fabric_fdisc_wait_rsp, NULL);
		}
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		break;
	}

	return NULL;
}

/**
 * @ingroup fabric_sm
 * @brief Fabric node state machine: Wait for an FDISC response
 *
 * @par Description
 * Used for a virtual port. Waits for an FDISC response. If OK, issue a HAL port attach.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fabric_fdisc_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK: {
		/* fc_id is in ext_status */
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_FDISC, __ocs_fabric_common, __func__)) {
			return NULL;
		}

		ocs_display_sparams(node->display_name, "fdisc rcvd resp", 0, NULL,
			((uint8_t*)cbdata->els->els_info->els_rsp.virt) + 4);

		ocs_fabric_set_topology(node, OCS_SPORT_TOPOLOGY_FABRIC);
		//sm: / ocs_sport_attach
		ocs_sport_attach(node->sport, cbdata->ext_status);
		ocs_node_transition(node, __ocs_fabric_wait_domain_attach, NULL);
		break;

	}

	case OCS_EVT_SRRS_ELS_REQ_RJT:
	case OCS_EVT_SRRS_ELS_REQ_FAIL: {
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_FDISC, __ocs_fabric_common, __func__)) {
			return NULL;
		}
		ocs_log_err(ocs, "FDISC failed, shutting down sport\n");
		//sm: / shutdown sport
		ocs_sm_post_event(&node->sport->sm, OCS_EVT_SHUTDOWN, NULL);
		break;
	}
	
	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		break;
	}

	return NULL;
}

static void
ocs_fabric_setup(ocs_node_t *node)
{
	int rc;

	rc = ocs_start_ns_node(node->sport);
	if (rc)
		return;

	/* Instantiate the fabric controller (sends SCR) */
	if (node->sport->enable_rscn) {
		rc = ocs_start_fabctl_node(node->sport);
		if (rc)
			return;
	}

	rc = ocs_start_mgmt_srv_node(node->sport);
	if (rc)
		return;
}

static void *
__ocs_fabric_wait_auth_finish(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_NODE_AUTH_OK:
		ocs_fabric_setup(node);
		ocs_node_transition(node, __ocs_fabric_idle, NULL);
		break;
	case OCS_EVT_NODE_AUTH_FAIL:
		/* Don't bother calling ocs_fabric_setup().
		 * The fabric would have blocked us anyway.
		 */
		ocs_node_transition(node, __ocs_fabric_idle, NULL);
		break;
	case OCS_EVT_AUTH_RCVD:
		ocs_auth_els_recv(node, arg);
		break;
	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		break;
	}
	return NULL;
}


/**
 * @ingroup fabric_sm
 * @brief Fabric node state machine: Wait for a domain/sport attach event.
 *
 * @par Description
 * Waits for a domain/sport attach event.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fabric_wait_domain_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int rc;
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
	case OCS_EVT_SPORT_ATTACH_OK:
		/* TODO: check why fabric is not mandating authentication. */
		if (node->auth.dhchap.flogi_acc_fcsp ||
		    ocs_node_auth_enabled(node)) {
			ocs_node_transition(node, __ocs_fabric_wait_auth_finish,
					    NULL);
			rc = ocs_node_auth_start(node, OCS_AUTH_ROLE_INITIATOR,
						 OCS_AUTH_EVT_START, NULL);
			if (rc)
				ocs_node_post_event(node,
						OCS_EVT_NODE_AUTH_FAIL, NULL);
		} else {
			ocs_fabric_setup(node);
			ocs_node_transition(node, __ocs_fabric_idle, NULL);
		}
		break;
	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup fabric_sm
 * @brief Fabric node state machine: Fabric node is idle.
 *
 * @par Description
 * Wait for fabric node events.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fabric_idle(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	/* The fabric may initiate re-auth even after successful auth */
	case OCS_EVT_AUTH_RCVD:
		ocs_auth_els_recv(node, arg);
		break;

	case OCS_EVT_NODE_AUTH_OK:
	case OCS_EVT_NODE_AUTH_FAIL:
		break;

	case OCS_EVT_DOMAIN_ATTACH_OK:
		break;

	case OCS_EVT_LCB_RCVD:
		ocs_els_process_lcb_rcvd(node, arg);
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup ns_sm
 * @brief Name services node state machine: Initialize.
 *
 * @par Description
 * A PLOGI is sent to the well-known name/directory services node.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_ns_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		//sm: / send PLOGI
		if (ocs_send_plogi(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
					OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
			ocs_node_transition(node, __ocs_ns_plogi_wait_rsp, NULL);
		} else {
			node_printf(node, "Failed to send PLOGI req, shutting down node\n");
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		}
		break;
	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		break;
	}

	return NULL;
}
/**
 * @ingroup ns_sm
 * @brief Name Services node state machine: Wait for domain attach.
 *
 * @par Description
 * Waits for domain to be attached then send
 * node attach request to the HAL.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_ns_wait_domain_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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
		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_ns_wait_node_attach, NULL);
		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}
		break;
	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup ns_sm
 * @brief Name services node state machine: Wait for a PLOGI response.
 *
 * @par Description
 * Waits for a response from PLOGI to name services node, then issues a
 * node attach request to the HAL.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_ns_plogi_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc;
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK: {
		/* Save service parameters */
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_fabric_common, __func__)) {
			return NULL;
		}

		//sm: / save sparams, ocs_node_attach
		ocs_node_save_sparms(node, cbdata->els->els_info->els_rsp.virt);
		ocs_display_sparams(node->display_name, "plogi rcvd resp", 0, NULL,
			((uint8_t*)cbdata->els->els_info->els_rsp.virt) + 4);

		/* Wait for domain to be attach then proceed with node attach */
		if (!node->sport->domain->attached) {
			ocs_node_transition(node, __ocs_ns_wait_domain_attach, NULL);
			break;
		}

		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_ns_wait_node_attach, NULL);
		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}
		break;
	}
	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup ns_sm
 * @brief Name services node state machine: Wait for a node attach completion.
 *
 * @par Description
 * Waits for a node attach completion, then issues an RFTID name services
 * request.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_ns_wait_node_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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

	case OCS_EVT_NODE_ATTACH_OK:
		node->attached = TRUE;
		//sm: / send RFTID
		if (ocs_ns_send_rftid(node, OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
					OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
			ocs_node_transition(node, __ocs_ns_rftid_wait_rsp, NULL);
		} else {
			node_printf(node, "Failed to send RFTID req, shutting down node\n");
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		}
		break;

	case OCS_EVT_NODE_ATTACH_FAIL:
		/* node attach failed, shutdown the node */
		node->attached = FALSE;
		node_printf(node, "Node attach failed\n");
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_fabric_initiate_shutdown(node);
		break;

	case OCS_EVT_SHUTDOWN:
		node_printf(node, "Shutdown event received\n");
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_node_transition(node, __ocs_fabric_wait_attach_evt_shutdown, NULL);
		break;

	/* if receive RSCN just ignore, we haven't sent GID_PT yet (ACC sent by fabctl node) */
	case OCS_EVT_RSCN_RCVD:
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup ns_sm
 * @brief Wait for a domain/sport/node attach completion, then
 * shutdown.
 *
 * @par Description
 * Waits for a domain/sport/node attach completion, then shuts
 * node down.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fabric_wait_attach_evt_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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

	/* wait for any of these attach events and then shutdown */
	case OCS_EVT_NODE_ATTACH_OK:
		node->attached = TRUE;
		node_printf(node, "Attach evt=%s, proceed to shutdown\n", ocs_sm_event_name(evt));
		ocs_fabric_initiate_shutdown(node);
		break;

	case OCS_EVT_NODE_ATTACH_FAIL:
		node->attached = FALSE;
		node_printf(node, "Attach evt=%s, proceed to shutdown\n", ocs_sm_event_name(evt));
		ocs_fabric_initiate_shutdown(node);
		break;

	/* ignore shutdown event as we're already in shutdown path */
	case OCS_EVT_SHUTDOWN:
		node_printf(node, "Shutdown event received\n");
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup ns_sm
 * @brief Name services node state machine: Wait for an RFTID response event.
 *
 * @par Description
 * Waits for an RFTID response event; if configured for an initiator operation,
 * a GIDPT name services request is issued.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_ns_rftid_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:
		if (node_check_ct_req(ctx, evt, arg, FC_GS_NAMESERVER_RFT_ID, __ocs_fabric_common, __func__)) {
			return NULL;
		}

		//sm: / send RFFID
		if (ocs_nvme_protocol_enabled(node->ocs) &&
					ocs_nvme_backend_enabled(node->ocs, node->sport))  {
			if (ocs_ns_send_rffid(node, FC_TYPE_NVME, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
					      OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
				ocs_node_transition(node, __ocs_ns_nvme_rffid_wait_rsp, NULL);
			} else {
				node_printf(node, "Failed to send RFFID req, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			}
		} else if ((ocs_scsi_protocol_enabled(node->ocs) &&
					ocs_scsi_backend_enabled(node->ocs, node->sport)) ) {
			if (ocs_ns_send_rffid(node, FC_TYPE_FCP, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
					      OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
				ocs_node_transition(node, __ocs_ns_rffid_wait_rsp, NULL);
			} else {
				node_printf(node, "Failed to send RFFID req, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			}
		} else {
			ocs_log_err(node->ocs, "No protocol support enabled\n");
			ocs_assert(0, NULL);
		}

		break;

	/* if receive RSCN just ignore, we haven't sent GID_PT yet (ACC sent by fabctl node) */
	case OCS_EVT_RSCN_RCVD:
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

void *
__ocs_ns_nvme_rffid_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:	{
		if (node_check_ct_req(ctx, evt, arg, FC_GS_NAMESERVER_RFF_ID, __ocs_fabric_common, __func__))
			return NULL;


		if (ocs_scsi_protocol_enabled(node->ocs) &&
				ocs_scsi_backend_enabled(node->ocs, node->sport)) {
			if (ocs_ns_send_rffid(node, FC_TYPE_FCP, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
					      OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
				ocs_node_transition(node, __ocs_ns_rffid_wait_rsp, NULL);
			} else {
				node_printf(node, "Failed to send RFFID req, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			}
		} else {
			if (node->sport->enable_rscn) {
				//sm: if enable_rscn / send GIDPT
				if (ocs_ns_send_gidpt(node, OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
							OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
					ocs_node_transition(node, __ocs_ns_gidpt_wait_rsp, NULL);
				} else {
					node_printf(node, "Failed to send RFFID req, shutting down node\n");
					ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
				}
			} else {
				/* if 'T' only, we're done, go to idle */
				ocs_node_transition(node, __ocs_ns_idle, NULL);
			}
		}

		break;
	}
	case OCS_EVT_SRRS_ELS_REQ_FAIL:
	case OCS_EVT_SRRS_ELS_REQ_RJT: {
		node_printf(node, "RFFID req failed, shutting down node\n");
		ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		break;
	}

	/* if receive RSCN just ignore, we haven't sent GID_PT yet (ACC sent by fabctl node) */
	case OCS_EVT_RSCN_RCVD:
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup ns_sm
 * @brief Fabric node state machine: Wait for RFFID response event.
 *
 * @par Description
 * Waits for an RFFID response event; if configured for an initiator operation,
 * a GIDPT name services request is issued.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_ns_rffid_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:	{
		if (node_check_ct_req(ctx, evt, arg, FC_GS_NAMESERVER_RFF_ID, __ocs_fabric_common, __func__)) {
			return NULL;
		}
		if (node->sport->enable_rscn) {
			//sm: if enable_rscn / send GIDPT
			if (ocs_ns_send_gidpt(node, OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
						OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
				ocs_node_transition(node, __ocs_ns_gidpt_wait_rsp, NULL);
			} else {
				node_printf(node, "Failed to send RFFID req, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			}
		} else {
			/* if 'T' only, we're done, go to idle */
			ocs_node_transition(node, __ocs_ns_idle, NULL);
		}
		break;
	}
	case OCS_EVT_SRRS_ELS_REQ_FAIL:
	case OCS_EVT_SRRS_ELS_REQ_RJT: {
		node_printf(node, "RFFID req failed, shutting down node\n");
		ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		break;
	}
	/* if receive RSCN just ignore, we haven't sent GID_PT yet (ACC sent by fabctl node) */
	case OCS_EVT_RSCN_RCVD:
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup ns_sm
 * @brief Name services node state machine: Wait for a GIDPT response.
 *
 * @par Description
 * Wait for a GIDPT response from the name server. Process the FC_IDs that are
 * reported by creating new remote ports, as needed.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_ns_gidpt_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:	{
		if (node_check_ct_req(ctx, evt, arg, FC_GS_NAMESERVER_GID_PT, __ocs_fabric_common, __func__)) {
			return NULL;
		}
		//sm: / process GIDPT payload
		ocs_process_gidpt_payload(node, cbdata->els->els_info->els_rsp.virt, cbdata->els->els_info->els_rsp.len);

		if (node->sport->enable_ini)
			ocs_find_new_targets(node, cbdata->els->els_info->els_rsp.virt, cbdata->els->els_info->els_rsp.len);
		else
			ocs_node_transition(node, __ocs_ns_idle, NULL);

		break;
	}

	case OCS_EVT_SRRS_ELS_REQ_RJT:
	case OCS_EVT_SRRS_ELS_REQ_FAIL:	{
		/* not much we can do; will retry with the next RSCN */
		node_printf(node, "GID_PT failed to complete\n");
		ocs_node_transition(node, __ocs_ns_idle, NULL);
		break;
	}

	/* If we receive an RSCN here, queue up another discovery processing */
	case OCS_EVT_RSCN_RCVD: {
		node_printf(node, "RSCN received during GID_PT processing\n");
		node->rscn_pending = 1;
		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

void *
__ocs_ns_ganxt_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:	{
		if (node_check_ct_req(ctx, evt, arg, FC_GS_NAMESERVER_GA_NXT, __ocs_fabric_common, __func__))
			return NULL;

		ocs_node_transition(node, __ocs_ns_idle, NULL);
		break;
	}

	case OCS_EVT_SRRS_ELS_REQ_RJT:
	case OCS_EVT_SRRS_ELS_REQ_FAIL:	{
		ocs_node_transition(node, __ocs_ns_idle, NULL);
		break;
	}

	case OCS_EVT_RSCN_RCVD: {
		node->rscn_pending = 1;
		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup ns_sm
 * @brief Name services node state machine: Idle state.
 *
 * @par Description
 * Idle. Waiting for RSCN received events (posted from the fabric controller), and
 * restarts the GIDPT name services query and processing.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_ns_idle(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		if (!node->rscn_pending) {
			break;
		}
		node_printf(node, "RSCN pending, restart discovery\n");
		node->rscn_pending = 0;

		FALL_THROUGH; /* fall through */

	case OCS_EVT_RSCN_RCVD: {
		//sm: / send GIDPT
		/* If target RSCN processing is enabled, and this is target only (not initiator),
		 * and tgt_rscn_delay is non-zero, then we delay issuing the GID_PT
		 */
		if ((ocs->tgt_rscn_delay_msec != 0) && !node->sport->enable_ini && node->sport->enable_tgt &&
			enable_target_rscn(ocs)) {
			ocs_node_transition(node, __ocs_ns_gidpt_delay, NULL);
		} else {
			if (ocs_ns_send_gidpt(node, OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
						OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
				ocs_node_transition(node, __ocs_ns_gidpt_wait_rsp, NULL);
			} else {
				node_printf(node, "Failed to send GIDPT req, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			}
		}
		break;
	}

	case OCS_EVT_NODE_GANXT_GET_CMD: {
		ocs_ganxt_get_cmd_args_t *args = arg;
		ocs_ganxt_get_cmd_results_t *result;

		ocs_assert(args, NULL);
		ocs_assert(args->cb_arg, NULL);

		result = args->cb_arg;
		if (!ocs_ns_send_ganxt(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT, OCS_FC_ELS_DEFAULT_RETRIES, 
					args->cb, args->cb_arg, args->ganxt_cmd_req->port_id)) {
			ocs_log_err(node->ocs, "Failed to send GA_NXT request\n");
			result->status = -1;
			args->cb(node, NULL, args->cb_arg);
		}

		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		break;
	}

	return NULL;
}

/**
 * @brief Handle GIDPT delay timer callback
 *
 * @par Description
 * Post an OCS_EVT_GIDPT_DEIALY_EXPIRED event to the passed in node.
 *
 * @param arg Pointer to node.
 *
 * @return None.
 */
static void
gidpt_delay_timer_cb(void *arg)
{
	ocs_node_t *node = arg;
	int32_t rc;

	ocs_del_timer(&node->gidpt_delay_timer);
	rc = ocs_xport_control(node->ocs->xport, OCS_XPORT_POST_NODE_EVENT, node, OCS_EVT_GIDPT_DELAY_EXPIRED, NULL);
	if (rc)
		ocs_log_err(node->ocs, "ocs_xport_control(OCS_XPORT_POST_NODE_EVENT) failed: %d\n", rc);
}

/**
 * @ingroup ns_sm
 * @brief Name services node state machine: Delayed GIDPT.
 *
 * @par Description
 * Waiting for GIDPT delay to expire before submitting GIDPT to name server.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_ns_gidpt_delay(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER: {
		time_t delay_msec;

		ocs_assert(ocs->tgt_rscn_delay_msec != 0, NULL);

		/*
		 * Compute the delay time.   Set to tgt_rscn_delay, if the time since last GIDPT
		 * is less than tgt_rscn_period, then use tgt_rscn_period.
		 */
		delay_msec = ocs->tgt_rscn_delay_msec;
		if ((ocs_msectime() - node->time_last_gidpt_msec) < ocs->tgt_rscn_period_msec) {
			delay_msec = ocs->tgt_rscn_period_msec;
		}

		ocs_setup_timer(ocs, &node->gidpt_delay_timer, gidpt_delay_timer_cb, node, delay_msec, false);

		break;
	}

	case OCS_EVT_GIDPT_DELAY_EXPIRED:
		node->time_last_gidpt_msec = ocs_msectime();
		if (ocs_ns_send_gidpt(node, OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
					OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
			ocs_node_transition(node, __ocs_ns_gidpt_wait_rsp, NULL);
		} else {
			node_printf(node, "Failed to send GIDPT req, shutting down node\n");
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		}
		break;

	case OCS_EVT_RSCN_RCVD: {
		ocs_log_debug(ocs, "RSCN received while in GIDPT delay - no action\n");
		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		break;
	}

	return NULL;
}

/**
 * @ingroup fabric_sm
 * @brief Fabric controller node state machine: Initial state.
 *
 * @par Description
 * Issue a PLOGI to a well-known fabric controller address.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fabctl_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_t *node = ctx->app;

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		/* no need to login to fabric controller, just send SCR */
		if (ocs_send_scr(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
				OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
			ocs_node_transition(node, __ocs_fabctl_wait_scr_rsp, NULL);
		} else {
			node_printf(node, "Failed to send SCR req, shutting down node\n");
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		}
		break;

	case OCS_EVT_NODE_ATTACH_OK:
		node->attached = TRUE;
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup fabric_sm
 * @brief Fabric controller node state machine: Wait for a node attach request
 * to complete.
 *
 * @par Description
 * Wait for a node attach to complete. If successful, issue an SCR
 * to the fabric controller, subscribing to all RSCN.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 *
 */
void *
__ocs_fabctl_wait_node_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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

	case OCS_EVT_NODE_ATTACH_OK:
		node->attached = TRUE;
		//sm: / send SCR
		if (ocs_send_scr(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
				OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
			ocs_node_transition(node, __ocs_fabctl_wait_scr_rsp, NULL);
		} else {
			node_printf(node, "Failed to send SCR req, shutting down node\n");
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		}
		break;

	case OCS_EVT_NODE_ATTACH_FAIL:
		/* node attach failed, shutdown the node */
		node->attached = FALSE;
		node_printf(node, "Node attach failed\n");
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_fabric_initiate_shutdown(node);
		break;

	case OCS_EVT_SHUTDOWN:
		node_printf(node, "Shutdown event received\n");
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_node_transition(node, __ocs_fabric_wait_attach_evt_shutdown, NULL);
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup fabric_sm
 * @brief Fabric controller node state machine: Wait for an SCR response from the
 * fabric controller.
 *
 * @par Description
 * Waits for an SCR response from the fabric controller.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fabctl_wait_scr_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_SCR, __ocs_fabric_common, __func__)) {
			return NULL;
		}

		if (ocs_send_rdf(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
				OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
			ocs_node_transition(node, __ocs_fabctl_wait_rdf_rsp, NULL);
		} else {
			node_printf_warn(node, "RDF registration is skipped\n");
			ocs_node_transition(node, __ocs_fabctl_ready, NULL);
		}

		break;
	case OCS_EVT_RSCN_RCVD: {
		ocs_node_cb_t *cbdata = arg;
		fc_header_t *hdr = cbdata->header;

		node->rscn_pending = true;
		if (ocs_sframe_send_ls_acc(node,
			fc_be24toh(hdr->s_id), fc_be24toh(hdr->d_id),
			ocs_be16toh(hdr->ox_id), ocs_be16toh(hdr->rx_id))) {
			node_printf_err(node, "Sframe send ls_acc failed\n");
		}

		if (cbdata->io)
			ocs_els_io_free(cbdata->io);

		break;
	}
	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

void *
__ocs_fabctl_wait_rdf_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch (evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:
	case OCS_EVT_SRRS_ELS_REQ_FAIL:
	case OCS_EVT_SRRS_ELS_REQ_RJT:
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_RDF, __ocs_fabric_common, __func__)) {
			return NULL;
		}
		node_printf_ratelimited(node, "RDF response evt: %s\n", ocs_sm_event_name(evt));
		ocs_node_transition(node, __ocs_fabctl_ready, NULL);
		break;
	case OCS_EVT_RSCN_RCVD: {
		ocs_node_cb_t *cbdata = arg;
		fc_header_t *hdr = cbdata->header;

		node->rscn_pending = true;
		if (ocs_sframe_send_ls_acc(node,
			fc_be24toh(hdr->s_id), fc_be24toh(hdr->d_id),
			ocs_be16toh(hdr->ox_id), ocs_be16toh(hdr->rx_id))) {
			node_printf_err(node, "Sframe send ls_acc failed\n");
		}

		if (cbdata->io)
			ocs_els_io_free(cbdata->io);

		break;
	}
	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup fabric_sm
 * @brief Fabric controller node state machine: Ready.
 *
 * @par Description
 * In this state, the fabric controller sends a RSCN, which is received
 * by this node and is forwarded to the name services node object; and
 * the RSCN LS_ACC is sent.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_fabctl_ready(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		if (node->rscn_pending) {
			node->rscn_pending = false;
			ocs_process_rscn(node, NULL);
		}

		break;

	case OCS_EVT_RSCN_RCVD: {
		fc_header_t *hdr = cbdata->header;

		//sm: / process RSCN (forward to name services node), send LS_ACC
		ocs_process_rscn(node, cbdata);
		ocs_send_ls_acc(cbdata->io, ocs_be16toh(hdr->ox_id), NULL, NULL);
		ocs_node_transition(node, __ocs_fabctl_wait_ls_acc_cmpl, NULL);
		break;
	}

	case OCS_EVT_FPIN_RCVD:
		ocs_els_process_fpin_rcvd(node, arg);
		break;

	case OCS_EVT_LCB_RCVD:
		/*
		 * Even though this is in violation of the spec, handle the LCB ELS cmd
		 * received from Fabric Controller as well. This is to work around
		 * a bug in the Brocade switch firmware (FOS versions prior to 9.1.0).
		 */
		ocs_els_process_lcb_rcvd(node, arg);
		break;

	case OCS_EVT_SEND_FPIN: {
		ocs_sport_exec_arg_t *sport_exec_args = arg;

		ocs_assert(sport_exec_args, NULL);
		sport_exec_args->retval = 0;
		if (ocs_els_fpin_send_li(node, sport_exec_args->arg_in)) {
			node_printf(node, "failed to send FPIN Link event\n");
			sport_exec_args->retval = -1;
		}
		sport_exec_args->exec_cb(node->sport, arg);
		break;
	}
	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup fabric_sm
 * @brief Fabric controller node state machine: Wait for LS_ACC.
 *
 * @par Description
 * Waits for the LS_ACC from the fabric controller.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_fabctl_wait_ls_acc_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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

	case OCS_EVT_SRRS_ELS_CMPL_OK:
		ocs_node_transition(node, __ocs_fabctl_ready, NULL);
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup fabric_sm
 * @brief Initiate fabric node shutdown.
 *
 * @param node Node for which shutdown is initiated.
 *
 * @return Returns None.
 */

static void
ocs_fabric_initiate_shutdown(ocs_node_t *node)
{
	ocs_hal_rtn_e rc = OCS_HAL_RTN_SUCCESS;
	ocs_t *ocs = node->ocs;
	ocs_scsi_io_alloc_disable(node);

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
		}
	}
	/*
	 * node has either been detached or is in the process of being detached,
	 * call common node's initiate cleanup function
	 */
	ocs_node_initiate_cleanup(node);
}

/**
 * @ingroup fabric_sm
 * @brief Fabric node state machine: Handle the common fabric node events.
 *
 * @param funcname Function name text.
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

static void *
__ocs_fabric_common(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_t *node = NULL;
	ocs_assert(ctx, NULL);
	ocs_assert(ctx->app, NULL);
	node = ctx->app;

	switch(evt) {
	case OCS_EVT_DOMAIN_ATTACH_OK:
		break;
	case OCS_EVT_SHUTDOWN:
		node->mark_for_deletion = TRUE;
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_fabric_initiate_shutdown(node);
		break;

	default:
		/* call default event handler common to all nodes */
		__ocs_node_common(funcname, ctx, evt, arg);
		break;
	}
	return NULL;
}

/**
 * @brief Return TRUE if the remote node is an NPORT.
 *
 * @par Description
 * Examines the service parameters. Returns TRUE if the node reports itself as
 * an NPORT.
 *
 * @param remote_sparms Remote node service parameters.
 *
 * @return Returns TRUE if NPORT.
 */

int32_t
ocs_rnode_is_nport(fc_plogi_payload_t *remote_sparms)
{
	return (ocs_be32toh(remote_sparms->common_service_parameters[1]) & (1U << 28)) == 0;
}

/**
 * @brief Return TRUE if the remote node is NPIV capable
 *
 * @par Description
 * Examines the service parameters. Returns TRUE if the node reports itself as
 * an NPIV feature capable.
 *
 * @param remote_sparms Remote node service parameters.
 *
 * @return Returns TRUE if NPIV supported.
 */
int32_t
ocs_rnode_is_npiv_capable(fc_plogi_payload_t *remote_sparms)
{
	return (ocs_be32toh(remote_sparms->common_service_parameters[1]) & (1U << 29));
}

/**
 * @brief Return the node's WWPN as an uint64_t.
 *
 * @par Description
 * The WWPN is computed from service parameters, and returned as a uint64_t.
 *
 * @param sp Pointer to service parameters.
 *
 * @return Returns WWPN.
 *
 */

static uint64_t
ocs_get_wwpn(fc_plogi_payload_t *sp)
{
	return (((uint64_t)ocs_be32toh(sp->port_name_hi) << 32ll) | (ocs_be32toh(sp->port_name_lo)));
}

/**
 * @brief Return TRUE if the remote node is the point-to-point winner.
 *
 * @par Description
 * Compares WWPNs. Returns TRUE if the remote node's WWPN is numerically
 * higher than the local node's WWPN.
 *
 * @param sport Pointer to the sport object.
 *
 * @return
 * - 0, if the remote node is the loser.
 * - 1, if the remote node is the winner.
 * - (-1), if remote node is neither the loser nor the winner
 *   (WWPNs match)
 */

static int32_t
ocs_rnode_is_winner(ocs_sport_t *sport)
{
	fc_plogi_payload_t *remote_sparms = (fc_plogi_payload_t*) sport->domain->flogi_service_params;
	uint64_t remote_wwpn = ocs_get_wwpn(remote_sparms);
	uint64_t local_wwpn = sport->wwpn;
	char prop_buf[32];
	uint64_t wwn_bump = 0;

	if (ocs_get_property("wwn_bump", prop_buf, sizeof(prop_buf)) == 0) {
		wwn_bump = ocs_strtoull(prop_buf, 0, 0);
	}
	local_wwpn ^= wwn_bump;

	remote_wwpn = ocs_get_wwpn(remote_sparms);

	ocs_log_debug(sport->ocs, "r: %08x %08x\n", ocs_be32toh(remote_sparms->port_name_hi), ocs_be32toh(remote_sparms->port_name_lo));
	ocs_log_debug(sport->ocs, "l: %08x %08x\n", (uint32_t) (local_wwpn >> 32ll), (uint32_t) local_wwpn);

	if (remote_wwpn == local_wwpn) {
		ocs_log_warn(sport->ocs, "WWPN of remote node [%08x %08x] matches local WWPN\n",
			(uint32_t) (local_wwpn >> 32ll), (uint32_t) local_wwpn);
		return (-1);
	}

	return (remote_wwpn > local_wwpn);
}

/**
 * @ingroup p2p_sm
 * @brief Point-to-point state machine: Wait for the domain attach to complete.
 *
 * @par Description
 * Once the domain attach has completed, a PLOGI is sent (if we're the
 * winning point-to-point node).
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_p2p_wait_domain_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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
		ocs_sport_t *sport = node->sport;
		ocs_node_t *rnode;

		/* this transient node (SID=0 (recv'd FLOGI) or DID=fabric (sent FLOGI))
		 * is the p2p winner, will use a separate node to send PLOGI to peer
		 */
		ocs_assert (node->sport->p2p_winner, NULL);

		rnode = ocs_node_find(sport, node->sport->p2p_remote_port_id);
		if (rnode != NULL) {
			/* the "other" transient p2p node has already kicked off the
			 * new node from which PLOGI is sent */
			node_printf(node, "Node with fc_id x%x already exists\n", rnode->rnode.fc_id);
			ocs_assert (rnode != node, NULL);
		} else {
			/* create new node (SID=1, DID=2) from which to send PLOGI */
			rnode = ocs_node_alloc(sport, sport->p2p_remote_port_id, FALSE, FALSE);
			if (rnode == NULL) {
				ocs_log_err(ocs, "node alloc failed\n");
				return NULL;
			}

			ocs_fabric_notify_topology(node);
			//sm: / allocate p2p remote node
			ocs_node_transition(rnode, __ocs_p2p_rnode_init, NULL);
		}

		/* the transient node (SID=0 or DID=fabric) has served its purpose */
		if (node->rnode.fc_id == 0) {
			/* if this is the SID=0 node, move to the init state in case peer
			 * has restarted FLOGI discovery and FLOGI is pending
			 */
			/* don't send PLOGI on ocs_d_init entry */
			ocs_node_init_device(node, FALSE);
		} else {
			/* if this is the DID=fabric node (we initiated FLOGI), shut it down */
			node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
			ocs_fabric_initiate_shutdown(node);
		}
		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

void *
__ocs_d_wait_domain_async(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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

	case OCS_EVT_DOMAIN_WAIT_ASYNC_CB_OK: {
		ocs_domain_attach_async_args_t *async_args = arg;

		ocs_node_transition(node, async_args->state, NULL);
		ocs_domain_attach(node->sport->domain, async_args->s_id);
		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;

}

/**
 * @ingroup p2p_sm
 * @brief Point-to-point state machine: Remote node initialization state.
 *
 * @par Description
 * This state is entered after winning point-to-point, and the remote node
 * is instantiated.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_p2p_rnode_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		//sm: / send PLOGI
		if (ocs_send_plogi(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT,
					OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
			ocs_node_transition(node, __ocs_p2p_wait_plogi_rsp, NULL);
		} else {
			node_printf(node, "Failed to send PLOGI req, shutting down node\n");
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		}
		break;

	case OCS_EVT_ABTS_RCVD:
		//sm: send BA_ACC
		ocs_bls_send_acc_hdr(cbdata->io, cbdata->header);
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup p2p_sm
 * @brief Point-to-point node state machine: Wait for the FLOGI accept completion.
 *
 * @par Description
 * Wait for the FLOGI accept completion.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_p2p_wait_flogi_acc_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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

	case OCS_EVT_SRRS_ELS_CMPL_OK:
		//sm: if p2p_winner / domain_attach
		if (node->sport->p2p_winner) {
			ocs_node_transition(node, __ocs_p2p_wait_domain_attach, NULL);
			if (node->sport->domain->attached &&
			    !(node->sport->domain->domain_notify_pend)) {
				node_printf(node, "Domain already attached\n");
				ocs_node_post_event(node, OCS_EVT_DOMAIN_ATTACH_OK, NULL);
			}
		} else {
			/* this node has served its purpose; we'll expect a PLOGI on a separate
			 * node (remote SID=0x1); return this node to init state in case peer
			 * restarts discovery -- it may already have (pending frames may exist).
			 */
			/* don't send PLOGI on ocs_d_init entry */
			ocs_node_init_device(node, FALSE);
		}
		break;

	case OCS_EVT_SRRS_ELS_CMPL_FAIL:
		/* LS_ACC failed, possibly due to link down; shutdown node and wait
		 * for FLOGI discovery to restart */
		node_printf(node, "FLOGI LS_ACC failed, shutting down\n");
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_node_add_shutdown_list(node);
		ocs_fabric_initiate_shutdown(node);
		break;

	case OCS_EVT_ABTS_RCVD: {
		//sm: / send BA_ACC
		ocs_bls_send_acc_hdr(cbdata->io, cbdata->header);
		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}


/**
 * @ingroup p2p_sm
 * @brief Point-to-point node state machine: Wait for a PLOGI response
 * as a point-to-point winner.
 *
 * @par Description
 * Wait for a PLOGI response from the remote node as a point-to-point winner.
 * Submit node attach request to the HAL.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_p2p_wait_plogi_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc;
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK: {
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_fabric_common, __func__)) {
			return NULL;
		}
		//sm: / save sparams, ocs_node_attach
		ocs_node_save_sparms(node, cbdata->els->els_info->els_rsp.virt);
		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_p2p_wait_node_attach, NULL);
		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}
		break;
	}

	case OCS_EVT_SRRS_ELS_REQ_FAIL: {
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_fabric_common, __func__)) {
			return NULL;
		}
		node_printf(node, "PLOGI failed, shutting down\n");
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_fabric_initiate_shutdown(node);
		break;
	}

	case OCS_EVT_PLOGI_RCVD: {
		fc_header_t *hdr = cbdata->header;
		/* if we're in external loopback mode, just send LS_ACC */
		if (node->ocs->external_loopback) {
			ocs_send_plogi_acc(cbdata->io, ocs_be16toh(hdr->ox_id), NULL, NULL);
			break;
		} else{
			/* if this isn't external loopback, pass to default handler */
			__ocs_fabric_common(__func__, ctx, evt, arg);
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
		fc_prli_payload_t *prli = cbdata->payload;

		ocs_process_prli_payload(node, prli);
		if (prli->type == FC_TYPE_NVME)
			node->ls_rsp_io[OCS_LS_RSP_TYPE_NVME_PRLI] = cbdata->io;
		else if (prli->type == FC_TYPE_FCP)
			node->ls_rsp_io[OCS_LS_RSP_TYPE_FCP_PRLI] = cbdata->io;

		ocs_node_transition(node, __ocs_p2p_wait_plogi_rsp_recvd_prli, NULL);
		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup p2p_sm
 * @brief Point-to-point node state machine: Waiting on a response for a
 *        sent PLOGI.
 *
 * @par Description
 * State is entered when the point-to-point winner has sent
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
__ocs_p2p_wait_plogi_rsp_recvd_prli(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_fabric_common, __func__)) {
			return NULL;
		}
		//sm: / save sparams, ocs_node_attach
		ocs_node_save_sparms(node, cbdata->els->els_info->els_rsp.virt);
		ocs_display_sparams(node->display_name, "plogi rcvd resp", 0, NULL,
			((uint8_t*)cbdata->els->els_info->els_rsp.virt) + 4);
		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_p2p_wait_node_attach, NULL);
		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}
		break;

	case OCS_EVT_SRRS_ELS_REQ_FAIL:	/* PLOGI response received */
	case OCS_EVT_SRRS_ELS_REQ_RJT:
		/* PLOGI failed, shutdown the node */
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_fabric_common, __func__)) {
			return NULL;
		}
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_fabric_initiate_shutdown(node);
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup p2p_sm
 * @brief Point-to-point node state machine: Wait for a point-to-point node attach
 * to complete.
 *
 * @par Description
 * Waits for the point-to-point node attach to complete.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_p2p_wait_node_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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

	case OCS_EVT_NODE_ATTACH_OK:
		/*
		 * In P2P, ls_rsp_io can't have an outstanding PLOGI request. So, handle
		 * all the remaining cases, including PRLI, in port_logged_in state.
		 */
		node->attached = TRUE;
		ocs_node_transition(node, __ocs_d_port_logged_in, NULL);
		break;

	case OCS_EVT_NODE_ATTACH_FAIL:
		/* node attach failed, shutdown the node */
		node->attached = FALSE;
		node_printf(node, "Node attach failed\n");
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_fabric_initiate_shutdown(node);
		break;

	case OCS_EVT_SHUTDOWN:
		node_printf(node, "%s received\n", ocs_sm_event_name(evt));
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_node_transition(node, __ocs_fabric_wait_attach_evt_shutdown, NULL);
		break;

	case OCS_EVT_PRLI_RCVD: {
		fc_prli_payload_t *prli = cbdata->payload;

		node_printf(node, "%s: PRLI received before node is attached\n", ocs_sm_event_name(evt));
		ocs_process_prli_payload(node, prli);
		if (prli->type == FC_TYPE_NVME)
			node->ls_rsp_io[OCS_LS_RSP_TYPE_NVME_PRLI] = cbdata->io;
		else if (prli->type == FC_TYPE_FCP)
			node->ls_rsp_io[OCS_LS_RSP_TYPE_FCP_PRLI] = cbdata->io;

		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @brief Start up the name services node.
 *
 * @par Description
 * Allocates and starts up the name services node.
 *
 * @param sport Pointer to the sport structure.
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */

static int32_t
ocs_start_ns_node(ocs_sport_t *sport)
{
	ocs_node_t *ns;

	/* Instantiate a name services node */
	ns = ocs_node_find(sport, FC_ADDR_NAMESERVER);
	if (ns == NULL) {
		ns = ocs_node_alloc(sport, FC_ADDR_NAMESERVER, FALSE, FALSE);
		if (ns == NULL) {
			return -1;
		}
	}
	// TODO: for found ns, should we be transitioning from here?
	// breaks transition only 1. from within state machine or
	// 2. if after alloc
	if (ns->ocs->nodedb_mask & OCS_NODEDB_PAUSE_NAMESERVER) {
		ocs_node_pause(ns, __ocs_ns_init);
	} else {
		ocs_node_transition(ns, __ocs_ns_init, NULL);
	}
	return 0;
}

/**
 * @brief Start up the mgmt server node.
 *
 * @par Description
 * Allocates and starts up the mgmt server node.
 *
 * @param sport Pointer to the sport structure.
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */

static int32_t
ocs_start_mgmt_srv_node(ocs_sport_t *sport)
{
	ocs_node_t *ms;

	/* Instantiate a mgmt server node */
	ms = ocs_node_find(sport, FC_ADDR_MGMT_SERVER);
	if (ms == NULL) {
		ms = ocs_node_alloc(sport, FC_ADDR_MGMT_SERVER, FALSE, FALSE);
		if (ms == NULL)
			return -1;
	}

	ocs_node_transition(ms, __ocs_mgmt_srv_init, NULL);

	return 0;
}

/**
 * @brief Start up the fabric controller node.
 *
 * @par Description
 * Allocates and starts up the fabric controller node.
 *
 * @param sport Pointer to the sport structure.
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */

static int32_t
ocs_start_fabctl_node(ocs_sport_t *sport)
{
	ocs_node_t *fabctl;

	fabctl = ocs_node_find(sport, FC_ADDR_CONTROLLER);
	if (fabctl == NULL) {
		fabctl = ocs_node_alloc(sport, FC_ADDR_CONTROLLER, FALSE, FALSE);
		if (fabctl == NULL) {
			return -1;
		}
	}
	// TODO: for found ns, should we be transitioning from here?
	// breaks transition only 1. from within state machine or
	// 2. if after alloc
	ocs_node_transition(fabctl, __ocs_fabctl_init, NULL);
	return 0;
}

/**
 * @brief Set up the domain point-to-point parameters.
 *
 * @par Description
 * The remote node service parameters are examined, and various point-to-point
 * variables are set.
 *
 * @param sport Pointer to the sport object.
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */

int32_t
ocs_p2p_setup(ocs_sport_t *sport)
{
	ocs_t *ocs = sport->ocs;
	int32_t rnode_winner;
	rnode_winner = ocs_rnode_is_winner(sport);

	/* set sport flags to indicate p2p "winner" */
	if (rnode_winner == 1) {
		sport->p2p_remote_port_id = 0;
		sport->p2p_port_id = 0;
		sport->p2p_winner = FALSE;
	} else if (rnode_winner == 0) {
		sport->p2p_remote_port_id = 2;
		sport->p2p_port_id = 1;
		sport->p2p_winner = TRUE;
	} else {
		/* no winner; only okay if external loopback enabled */
		if (sport->ocs->external_loopback) {
			/*
			 * External loopback mode enabled; local sport and remote node
			 * will be registered with an NPortID = 1;
			 */
			ocs_log_debug(ocs, "External loopback mode enabled\n");
			sport->p2p_remote_port_id = 1;
			sport->p2p_port_id = 1;
			sport->p2p_winner = TRUE;
		} else {
			ocs_log_warn(ocs, "failed to determine p2p winner\n");
			return rnode_winner;
		}
	}

	return 0;
}

/**
 * @brief Process the FABCTL node RSCN.
 *
 * <h3 class="desc">Description</h3>
 * Processes the FABCTL node RSCN payload, simply passes the event to the name server.
 *
 * @param node Pointer to the node structure.
 * @param cbdata Callback data to pass forward.
 *
 * @return None.
 */

static void
ocs_process_rscn(ocs_node_t *node, ocs_node_cb_t *cbdata)
{
	ocs_t *ocs = node->ocs;
	ocs_sport_t *sport = node->sport;
	ocs_node_t *ns;

	/* Forward this event to the name-services node */
	ns = ocs_node_lookup_get(sport, FC_ADDR_NAMESERVER);
	if (ns != NULL)  {
		ocs_node_post_event(ns, OCS_EVT_RSCN_RCVD, cbdata);
	} else {
		ocs_log_warn(ocs, "can't find name server node\n");
	}
}

static int32_t
ocs_process_gffid_payload(ocs_node_t *node, fcct_gffid_acc_t *gffid, ocs_discover_target_ctx_t *ctx)
{
	ocs_sport_t *sport = node->sport;

	if (ocs_be16toh(gffid->hdr.cmd_rsp_code) == FCCT_HDR_CMDRSP_REJECT) {
		node_printf(node, "GFFID request failed: rsn %#x rsn_expl %#x\n",
			gffid->hdr.reason_code, gffid->hdr.reason_code_explanation);

		if (gffid->hdr.reason_code == FC_REASON_COMMAND_NOT_SUPPORTED) {
			ctx->stop_discovery = true;
		}

		return -1;
	}

	ocs_sport_lock(sport);
	if ((gffid->fc4_feature_bits[FCP_TYPE_FEATURE_OFFSET] & FC4_FEATURE_TARGET) ||
	    (gffid->fc4_feature_bits[NVME_TYPE_FEATURE_OFFSET] & FC4_FEATURE_TARGET))
	{
		ocs_node_t *newnode;

		if (!ocs_node_find(sport, ctx->port_id)) {
			newnode = ocs_node_alloc(sport, ctx->port_id, 0, 0);
			if (newnode) {
				node_printf(node, "New target %x found\n", ctx->port_id);
				/* send PLOGI automatically if initiator */
				ocs_node_init_device(newnode, TRUE);
			} else {
				ocs_log_err(node->ocs, "ocs_node_alloc() failed\n");
			}
		}
	}
	ocs_sport_unlock(sport);
	return 0;
}

void *
__ocs_ns_gffid_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:	{
		if (node_check_ct_req(ctx, evt, arg, FC_GS_NAMESERVER_GFF_ID, __ocs_fabric_common, __func__)) {
			return NULL;
		}
		ocs_process_gffid_payload(node, cbdata->els->els_info->els_rsp.virt, cbdata->els->els_info->els_callback_arg);
		break;
	}

	case OCS_EVT_SRRS_ELS_REQ_FAIL:	{
		/* not much we can do; will retry with the next RSCN */
		break;
	}

	/* If we receive an RSCN here, queue up another discovery processing */
	case OCS_EVT_RSCN_RCVD: {
		node_printf(node, "RSCN received during GID_PT processing\n");
		node->rscn_pending = 1;
		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

static void
ocs_add_new_nodes(ocs_node_t *node, ocs_discover_target_ctx_t *ctx)
{
	fcct_gidpt_acc_t *gidpt = (fcct_gidpt_acc_t *)ctx->gidpt;
	ocs_sport_t *sport = node->sport;
	uint32_t portlist_count;

	portlist_count = (ctx->gidpt_len - sizeof(fcct_iu_header_t)) / sizeof(gidpt->port_list);

	ocs_sport_lock(sport);
	for (; ctx->index < portlist_count; ctx->index++) {
		uint32_t port_id = fc_be24toh(gidpt->port_list[ctx->index].port_id);

		if (port_id != node->rnode.sport->fc_id && !ocs_sport_find(sport->domain, port_id)) {
			ocs_node_t *newnode;

			if (!ocs_node_find(sport, port_id)) {
				newnode = ocs_node_alloc(sport, port_id, 0, 0);
				if (newnode) {
					node_printf(node, "New node %x found\n", port_id);
					/* send PLOGI automatically if initiator */
					ocs_node_init_device(newnode, TRUE);
				} else {
					ocs_log_err(node->ocs, "ocs_node_alloc() failed\n");
				}
			}
		}

		if (gidpt->port_list[ctx->index].ctl & FCCT_GID_PT_LAST_ID) {
			break;
		}
	}
	ocs_sport_unlock(sport);

	if (ctx)
		ocs_free(node->ocs, ctx, sizeof(ocs_discover_target_ctx_t) + ctx->gidpt_len);

	/* Done with discovery. */
	ocs_node_transition(node, __ocs_ns_idle, NULL);
}

static void
ocs_find_next_target_done(ocs_node_t *node, ocs_node_cb_t *cbdata, void *args)
{
	ocs_discover_target_ctx_t *ctx = args;

	ocs_assert(ctx);

	if (!node->mark_for_deletion) {
		if (ctx->stop_discovery) {
			/* Add new nodes from gidpt */
			ocs_add_new_nodes(node, ctx);
		} else {
			ctx->index++;
			ocs_find_next_target(node, ctx);
		}
	} else {
		node_printf(node, "node shutting down. Stopping target discovery.\n");
		if (ctx)
			ocs_free(node->ocs, ctx, sizeof(ocs_discover_target_ctx_t) + ctx->gidpt_len);
	}
}

static void
ocs_find_next_target(ocs_node_t *node, ocs_discover_target_ctx_t *ctx)
{
	fcct_gidpt_acc_t *gidpt = (fcct_gidpt_acc_t *)ctx->gidpt;
	ocs_sport_t *sport = node->sport;
	ocs_node_t *newnode;
	uint32_t portlist_count;

	portlist_count = (ctx->gidpt_len - sizeof(fcct_iu_header_t)) / sizeof(gidpt->port_list);

	ocs_sport_lock(sport);
	for (; ctx->index < portlist_count; ctx->index++) {
		uint32_t port_id = fc_be24toh(gidpt->port_list[ctx->index].port_id);

		if (port_id != node->rnode.sport->fc_id && !ocs_sport_find(sport->domain, port_id)) {
			newnode = ocs_node_find(sport, port_id);
			if (!newnode) {
				ocs_sport_unlock(sport);

				/* Send gidff */
				ctx->port_id = port_id;
				if (ocs_ns_send_gffid(node, OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT, OCS_FC_ELS_DEFAULT_RETRIES,
							ocs_find_next_target_done, ctx, gidpt->port_list[ctx->index].port_id)) {
					ocs_node_transition(node, __ocs_ns_gffid_wait_rsp, NULL);
				} else {
					node_printf(node, "Failed to send gffid req, shutting down node\n");
					if (ctx)
						ocs_free(node->ocs, ctx, sizeof(ocs_discover_target_ctx_t) + ctx->gidpt_len);
					ocs_node_post_event(node, OCS_EVT_SHUTDOWN, NULL);
				}
				return;
			}
		}

		if (gidpt->port_list[ctx->index].ctl & FCCT_GID_PT_LAST_ID) {
			break;
		}
	}
	ocs_sport_unlock(sport);

	if (ctx)
		ocs_free(node->ocs, ctx, sizeof(ocs_discover_target_ctx_t) + ctx->gidpt_len);

	/* Done with discovery. */
	ocs_node_transition(node, __ocs_ns_idle, NULL);
}

static int32_t
ocs_find_new_targets(ocs_node_t *node, fcct_gidpt_acc_t *gidpt, uint32_t gidpt_len)
{
	uint32_t i;
	ocs_node_t *newnode;
	ocs_sport_t *sport = node->sport;
	ocs_discover_target_ctx_t *ctx;
	uint32_t portlist_count;

	portlist_count = (gidpt_len - sizeof(fcct_iu_header_t)) / sizeof(gidpt->port_list);

	if (ocs_be16toh(gidpt->hdr.cmd_rsp_code) == FCCT_HDR_CMDRSP_REJECT)
		goto done;

	ocs_sport_lock(sport);
	for(i = 0; i < portlist_count; i ++) {
		uint32_t port_id = fc_be24toh(gidpt->port_list[i].port_id);

		if (port_id != node->rnode.sport->fc_id && !ocs_sport_find(sport->domain, port_id)) {
			newnode = ocs_node_find(sport, port_id);
			if (!newnode) {
				ocs_sport_unlock(sport);

				/* There are new nodes. Let start discovering */
				ctx = ocs_malloc(node->ocs, sizeof(ocs_discover_target_ctx_t) + gidpt_len,
							  OCS_M_ZERO | OCS_M_NOWAIT);
				if (!ctx) {
					node_printf(node, "Memory failure. Skip target discovery\n");
					goto done;
				}

				ctx->gidpt = (char *)ctx + sizeof(ocs_discover_target_ctx_t);
				ctx->gidpt_len = gidpt_len;
				memcpy(ctx->gidpt, (char *)gidpt, gidpt_len);
				ctx->index = 0;

				ocs_find_next_target(node, ctx);
				return 0;
			}
		}

		if (gidpt->port_list[i].ctl & FCCT_GID_PT_LAST_ID) {
			break;
		}
	}
	ocs_sport_unlock(sport);

done:
	/* no new targets */
	ocs_node_transition(node, __ocs_ns_idle, NULL);
	return 0;
}

/**
 * @brief Process the GIDPT payload.
 *
 * @par Description
 * The GIDPT payload is parsed, and new nodes are created, as needed.
 *
 * @param node Pointer to the node structure.
 * @param gidpt Pointer to the GIDPT payload.
 * @param gidpt_len Payload length
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */

static int32_t
ocs_process_gidpt_payload(ocs_node_t *node, fcct_gidpt_acc_t *gidpt, uint32_t gidpt_len)
{
	uint32_t i;
	uint32_t j;
	ocs_node_t *newnode;
	ocs_sport_t *sport = node->sport;
	ocs_t *ocs = node->ocs;
	uint32_t port_id;
	uint32_t port_count;
	ocs_node_t *n;
	ocs_node_t **active_nodes;
	uint32_t portlist_count;
	uint16_t residual;

	residual = ocs_be16toh(gidpt->hdr.max_residual_size);

	if (residual)
		ocs_log_debug(node->ocs, "residual is %u words\n", residual);

	if (ocs_be16toh(gidpt->hdr.cmd_rsp_code) == FCCT_HDR_CMDRSP_REJECT) {
		node_printf(node, "GIDPT request failed: rsn %#x rsn_expl %#x\n",
			gidpt->hdr.reason_code, gidpt->hdr.reason_code_explanation);
		/*
		 * If GID_PT request fails with reason code 0x9 (unable to perform command request),
		 * then we might have to post node missing event to all the existing active nodes.
		 */
		if (FCCT_UNABLE_TO_PERFORM == gidpt->hdr.reason_code) {
			switch (gidpt->hdr.reason_code_explanation) {
			case FCCT_PORT_TYPE_NOT_REGISTERED:
				/* Proceed with updating the active nodes */
				node_printf(node, "Port is not zoned-in with any device\n");
				break;
			default:
				return -1;
			}
		} else {
			return -1;
		}
	}

	portlist_count = (gidpt_len - sizeof(fcct_iu_header_t)) / sizeof(gidpt->port_list);

	/* Count the number of nodes */
	port_count = 0;
	ocs_sport_lock(sport);
		ocs_list_foreach(&sport->node_list, n) {
			port_count ++;
		}

		/* Allocate a buffer for all nodes */
		active_nodes = ocs_malloc(node->ocs, port_count * sizeof(*active_nodes), OCS_M_ZERO | OCS_M_NOWAIT);
		if (active_nodes == NULL) {
			node_printf(node, "ocs_malloc failed\n");
			ocs_sport_unlock(sport);
			return -1;
		}

		/* Fill buffer with fc_id of active nodes */
		i = 0;
		ocs_list_foreach(&sport->node_list, n) {
			port_id = n->rnode.fc_id;
			switch (port_id) {
			case FC_ADDR_FABRIC:
			case FC_ADDR_CONTROLLER:
			case FC_ADDR_NAMESERVER:
				break;
			default:
				if (!FC_ADDR_IS_DOMAIN_CTRL(port_id)) {
					active_nodes[i++] = n;
				}
				break;
			}
		}

		/* update the active nodes buffer */
		for (i = 0; i < portlist_count; i ++) {
			port_id = fc_be24toh(gidpt->port_list[i].port_id);

			for (j = 0; j < port_count; j ++) {
				if ((active_nodes[j] != NULL) && (port_id == active_nodes[j]->rnode.fc_id)) {
					active_nodes[j] = NULL;
				}
			}

			if (gidpt->port_list[i].ctl & FCCT_GID_PT_LAST_ID)
				break;
		}

		/* Those remaining in the active_nodes[] are now gone ! */
		for (i = 0; i < port_count; i ++) {
			/* if we're an initiator and the remote node is a target, then
			 * post the node missing event.   if we're target and we have enabled
			 * target RSCN, then post the node missing event.
			 */
			if (active_nodes[i] != NULL) {
				if ((node->sport->enable_ini && (active_nodes[i]->targ || active_nodes[i]->nvme_tgt)) ||
				    (node->sport->enable_tgt && enable_target_rscn(ocs))) {
					ocs_node_post_event(active_nodes[i], OCS_EVT_NODE_MISSING, NULL);
				} else {
					node_printf(node, "GID_PT: skipping non-tgt port_id x%06x\n",
						active_nodes[i]->rnode.fc_id);
				}
			}
		}
		ocs_free(ocs, active_nodes, port_count * sizeof(*active_nodes));

		for(i = 0; i < portlist_count; i ++) {
			uint32_t port_id = fc_be24toh(gidpt->port_list[i].port_id);

			if (port_id != node->rnode.sport->fc_id && !ocs_sport_find(sport->domain, port_id)) {
				newnode = ocs_node_find(sport, port_id);
				if (newnode) {
					// TODO: what if node deleted here??
					if (node->sport->enable_ini && (newnode->targ || newnode->nvme_tgt)) {
						ocs_node_post_event(newnode, OCS_EVT_NODE_REFOUND, NULL);
					}
					// original code sends ADISC, has notion of "refound"
				}
			}

			if (gidpt->port_list[i].ctl & FCCT_GID_PT_LAST_ID) {
				break;
			}
		}
	ocs_sport_unlock(sport);
	return 0;
}

int32_t
ocs_process_tdz_rsp_hdr(ocs_node_t *node, fcct_iu_header_t *hdr, ocs_tdz_status_t *tdz_status)
{
	uint16_t residual;

	residual = ocs_be16toh(hdr->max_residual_size);
	if (residual)
		ocs_log_debug(node->ocs, "residual is %u words\n", residual);

	if (FCCT_HDR_CMDRSP_REJECT == ocs_be16toh(hdr->cmd_rsp_code)) {
		node_printf(node, "TDZ request failed: rsn x%x rsn_expl x%x ven_spec x%x\n",
			    hdr->reason_code, hdr->reason_code_explanation, hdr->vendor_specific);

		tdz_status->reason_code = hdr->reason_code;
		tdz_status->reason_code_explanation = hdr->reason_code_explanation;
		tdz_status->vendor_specific = hdr->vendor_specific;

		return FCCT_HDR_CMDRSP_REJECT;
	}

	return 0;
}

int32_t
ocs_process_tdz_gfez_rsp_buf(ocs_node_t *node, fcct_tdz_gfez_rsp_t *gfez_rsp, ocs_tdz_status_t *tdz_status)
{
	uint8_t i;
	int32_t rc = 0;

	rc = ocs_process_tdz_rsp_hdr(node, &gfez_rsp->hdr, tdz_status);
	if (0 != rc)
		return rc;

	/*
	 * The "Fabric Enhanced Zoning support flags" represent the fabric wide support, and
	 * the "Switch Enhanced Zoning support entry" represent individual switch-level support.
	 * For Brocade's implementation of GFEZ, the remote switch's Enhanced Zoning
	 * enabled/disabled bit 1 field will be 0 (actually n/a) since the TDZ mode is a
	 * port-level setting as opposed to a switch-level setting. So if we just do an "OR" of
	 * all the switch-level flags, we will get accurate information for the local switch.
	 */
	for (i = 0; i < gfez_rsp->num_switch_entries; i++) {
		tdz_status->fez_enabled |= (ocs_be32toh(gfez_rsp->switch_entry[i].sez_flags) &
					    FCCT_TDZ_ENHANCED_ZONING_ENABLE);
	}

	return rc;
}

int32_t
ocs_process_tdz_gapz_rsp_buf(ocs_node_t *node, fcct_tdz_gapz_rsp_t *gapz_rsp, ocs_tdz_get_peer_zone_rsp_t *tdz_rsp)
{
	uint32_t i;
	int32_t rc = 0;
	size_t peer_member_size = sizeof(uint32_t);
	fcct_tdz_name_t *zone_name;
	fcct_tdz_attr_block_t *zone_attr_block;
	fcct_tdz_peer_block_t *zone_peer_block;
	fcct_tdz_peer_member_t *zone_peer_member;

	rc = ocs_process_tdz_rsp_hdr(node, &gapz_rsp->hdr, &tdz_rsp->status);
	if (0 != rc)
		return rc;

	zone_name = (fcct_tdz_name_t *)((uint8_t *)gapz_rsp + sizeof(fcct_iu_header_t));
	ocs_strncpy(tdz_rsp->zone_info.zone.name, zone_name->name, zone_name->length);

	zone_attr_block = (fcct_tdz_attr_block_t *)((uint8_t *)zone_name + OCS_TDZ_NAME_LEN(zone_name->length));
	tdz_rsp->zone_info.attr_block.num_principal_members = ocs_be32toh(zone_attr_block->num_attr_entries);
	for (i = 0; i < tdz_rsp->zone_info.attr_block.num_principal_members; i++) {
		format_wwn(ocs_be64toh(zone_attr_block->attr_entry[i].port_name),
			   tdz_rsp->zone_info.attr_block.principal_member[i].name);
	}

	zone_peer_block = (fcct_tdz_peer_block_t *)((uint8_t *)zone_attr_block +
			   OCS_TDZ_ATTR_BLOCK_LEN(tdz_rsp->zone_info.attr_block.num_principal_members));
	tdz_rsp->zone_info.peer_block.num_peer_members = ocs_be32toh(zone_peer_block->num_peer_members);
	for (i = 0; i < tdz_rsp->zone_info.peer_block.num_peer_members; i++) {
		zone_peer_member = (fcct_tdz_peer_member_t *)((uint8_t *)zone_peer_block + peer_member_size);

		switch (zone_peer_member->type) {
		case FCCT_ZONE_IDENT_TYPE_NPORT_NAME:
			format_wwn(ocs_be64toh(zone_peer_member->value.port_name),
				   tdz_rsp->zone_info.peer_block.peer_member[i].name);
			peer_member_size += OCS_TDZ_MEM_NPORT_LEN;
			break;
		case FCCT_ZONE_IDENT_TYPE_ALIAS_NAME:
			ocs_strncpy(tdz_rsp->zone_info.peer_block.peer_member[i].name,
				    zone_peer_member->value.alias_name.name,
				    zone_peer_member->value.alias_name.length);
			peer_member_size += OCS_TDZ_MEM_ALIAS_LEN(zone_peer_member->value.alias_name.length);
			break;
		default:
			ocs_log_err(node->ocs, "Unexpected TDZ peer member type: 0x%x\n", zone_peer_member->type);
			return -1;
		}
	}

	return rc;
}

/**
 * @brief Process the GRHL acc payload.
 *
 * @par Description
 * The GRHL payload is parsed, and new nodes are created, as needed.
 *
 * @param node Pointer to the node structure.
 * @param grhl Pointer to the GRHL payload.
 * @param grhl_len Payload length
 * @param buf destination buffer to copy the HBA list info
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */
int32_t
ocs_process_fdmi_grhl_payload(ocs_node_t *node, fcct_fdmi_grhl_rsp_t *grhl,
			      uint32_t grhl_len, void *buf)
{
	uint32_t hostlist_count;
	uint16_t residual;
	ocs_fdmi_hba_list_info_t *hba_list_buf = buf;
	uint32_t offset;
	int i = 0;

	residual = ocs_be16toh(grhl->hdr.max_residual_size);
	if (residual != 0) {
		ocs_log_debug(node->ocs, "residual is %u words\n", residual);
	}

	if (ocs_be16toh(grhl->hdr.cmd_rsp_code) == FCCT_HDR_CMDRSP_REJECT) {
		node_printf(node, "GRHL request failed: rsn x%x rsn_expl x%x\n",
			    grhl->hdr.reason_code, grhl->hdr.reason_code_explanation);
		return -1;
	}

	hba_list_buf->entry_count = ocs_be32toh(grhl->hba_list.entry_count);
	hostlist_count = MIN(hba_list_buf->entry_count, MAX_FDMI_HBA_COUNT);

	offset = 0;
	while (hostlist_count) {
		ocs_memcpy(&hba_list_buf->hba_identifier[i], (uint8_t *)&grhl->hba_list.port_entry[0] + offset,
			   sizeof(uint64_t));
		hba_list_buf->hba_identifier[i] = ocs_be64toh(hba_list_buf->hba_identifier[i]);
		i++;
		offset += sizeof(uint64_t);
		hostlist_count--;
	}

	return 0;
}

/**
 * @brief Process the GRPL acc payload.
 *
 * @par Description
 * The GRPL payload is parsed, and new nodes are created, as needed.
 *
 * @param node Pointer to the node structure.
 * @param grpl Pointer to the GRPL payload.
 * @param grpl_len Payload length
 * @param buf destination buffer to copy the Port list info
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */
int32_t
ocs_process_fdmi_grpl_payload(ocs_node_t *node, fcct_fdmi_grpl_rsp_t *grpl,
			      uint32_t grpl_len, void *buf)
{
	uint32_t portlist_count;
	uint16_t residual;
	ocs_fdmi_port_list_info_t *port_list_buf = buf;
	uint32_t offset;
	int i = 0;

	residual = ocs_be16toh(grpl->hdr.max_residual_size);
	if (residual != 0) {
		ocs_log_debug(node->ocs, "residual is %u words\n", residual);
	}

	if (ocs_be16toh(grpl->hdr.cmd_rsp_code) == FCCT_HDR_CMDRSP_REJECT) {
		node_printf(node, "GRPL request failed: rsn x%x rsn_expl x%x\n",
			    grpl->hdr.reason_code, grpl->hdr.reason_code_explanation);
		return -1;
	}

	port_list_buf->entry_count = ocs_be32toh(grpl->port_list.entry_count);
	portlist_count = MIN(port_list_buf->entry_count, MAX_FDMI_PORT_COUNT);

	offset = 0;
	while (portlist_count) {
		ocs_memcpy(&port_list_buf->port_identifier[i],
			   (uint8_t *)&grpl->port_list.port_entry[0] + offset, sizeof(uint64_t));
		port_list_buf->port_identifier[i] = ocs_be64toh(port_list_buf->port_identifier[i]);
		i++;
		offset += sizeof(uint64_t);
		portlist_count--;
	}

	return 0;
}

/**
 * @brief Process the GHAT acc payload.
 *
 * @par Description
 * The GHAT payload is parsed, and new nodes are created, as needed.
 *
 * @param node Pointer to the node structure.
 * @param ghat Pointer to the GHAT payload.
 * @param ghat_len Payload length
 * @param buf destination buffer to copy the HBA attributes info
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */
int32_t
ocs_process_fdmi_ghat_payload(ocs_node_t *node, fcct_fdmi_ghat_rsp_t *ghat,
			      uint32_t ghat_len, void *buf)
{
	uint32_t portlist_count;
	uint16_t residual;
	ocs_fdmi_hba_attr_info_t *hba_attr_info = buf;
	ocs_fdmi_attr_block_t *hba_attr_block = NULL;
	ocs_fdmi_attr_def_t   *attr_def;
	ocs_fdmi_attr_entry_t *ae;
	uint16_t attr_type, attr_len;
	uint32_t offset, num_attr_entries;
	int i = 0;

	residual = ocs_be16toh(ghat->hdr.max_residual_size);
	if (residual != 0) {
		ocs_log_debug(node->ocs, "residual is %u words\n", residual);
	}

	if (ocs_be16toh(ghat->hdr.cmd_rsp_code) == FCCT_HDR_CMDRSP_REJECT) {
		node_printf(node, "GHAT request failed: rsn x%x rsn_expl x%x\n",
			    ghat->hdr.reason_code, ghat->hdr.reason_code_explanation);
		return -1;
	}

	hba_attr_info->num_port_entries = MIN(ocs_be32toh(ghat->port_list.entry_count), MAX_FDMI_PORT_COUNT);
	portlist_count = hba_attr_info->num_port_entries;

	offset = 0;
	while (portlist_count) {
		ocs_memcpy(&hba_attr_info->port_identifier[i],
			   (uint8_t *)&ghat->port_list.port_entry[0] + offset, sizeof(uint64_t));
		hba_attr_info->port_identifier[i] = ocs_be64toh(hba_attr_info->port_identifier[i]);
		i++;
		offset += sizeof(uint64_t);
		portlist_count--;
	}

	offset = sizeof(ghat->hdr) + sizeof(hba_attr_info->num_port_entries);
	offset += hba_attr_info->num_port_entries * sizeof(uint64_t);

	/* Point to HBA attribute block */
	hba_attr_block = (ocs_fdmi_attr_block_t *)(((uint8_t *)ghat + offset));
	num_attr_entries = ocs_be32toh(hba_attr_block->num_entries);
	offset += sizeof(hba_attr_block->num_entries);

	/* Walk through all attribute entries and copy the same to user buf */
	while (num_attr_entries) {
		/* Point to HBA attribute entry */
		attr_def = (ocs_fdmi_attr_def_t *)(((uint8_t *)ghat + offset));
		attr_type = ocs_be16toh(attr_def->attr_type);
		attr_len  = ocs_be16toh(attr_def->attr_len);
		ae = &attr_def->attr_value;

		switch (attr_type) {
		case RHBA_NODENAME:
			hba_attr_info->node_name_valid = true;
			hba_attr_info->node_name = ocs_be64toh(ae->attr_wwn);
			break;
		case RHBA_MANUFACTURER:
			hba_attr_info->manufacturer_valid = true;
			ocs_strncpy(hba_attr_info->manufacturer, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->manufacturer)));
			break;
		case RHBA_SERIAL_NUMBER:
			hba_attr_info->serial_num_valid = true;
			ocs_strncpy(hba_attr_info->serial_num, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->serial_num)));
			break;
		case RHBA_MODEL:
			hba_attr_info->model_name_valid = true;
			ocs_strncpy(hba_attr_info->model_name, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->model_name)));
			break;
		case RHBA_MODEL_DESCRIPTION:
			hba_attr_info->model_desc_valid = true;
			ocs_strncpy(hba_attr_info->model_desc, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->model_desc)));
			break;
		case RHBA_HARDWARE_VERSION:
			hba_attr_info->hw_ver_valid = true;
			ocs_strncpy(hba_attr_info->hw_ver, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->hw_ver)));
			break;
		case RHBA_DRIVER_VERSION:
			hba_attr_info->driver_ver_valid = true;
			ocs_strncpy(hba_attr_info->driver_ver, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->driver_ver)));
			break;
		case RHBA_OPTION_ROM_VERSION:
			hba_attr_info->opt_rom_ver_valid = true;
			ocs_strncpy(hba_attr_info->opt_rom_ver, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->opt_rom_ver)));
			break;
		case RHBA_FIRMWARE_VERSION:
			hba_attr_info->fw_ver_valid = true;
			ocs_strncpy(hba_attr_info->fw_ver, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->fw_ver)));
			break;
		case RHBA_OS_NAME_VERSION:
			hba_attr_info->os_name_ver_valid = true;
			ocs_strncpy(hba_attr_info->os_name_ver, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->os_name_ver)));
			break;
		case RHBA_MAX_CT_PAYLOAD_LEN:
			hba_attr_info->max_ct_payload_valid = true;
			hba_attr_info->max_ct_payload_len = ocs_be32toh(ae->attr_int);
			break;
		case RHBA_SYM_NODENAME:
			hba_attr_info->node_symb_name_valid = true;
			ocs_strncpy(hba_attr_info->node_symb_name, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->node_symb_name)));
			break;
		case RHBA_VENDOR_INFO:
			hba_attr_info->vendor_info_valid = true;
			hba_attr_info->vendor_info = ocs_be32toh(ae->attr_int);
			break;
		case RHBA_NUM_PORTS:
			hba_attr_info->num_ports_valid = true;
			hba_attr_info->num_ports = ocs_be32toh(ae->attr_int);
			break;
		case RHBA_FABRIC_NAME:
			hba_attr_info->fabric_name_valid = true;
			hba_attr_info->fabric_name = ocs_be64toh(ae->attr_wwn);
			break;
		case RHBA_BIOS_VERSION:
			hba_attr_info->bios_ver_valid = true;
			ocs_strncpy(hba_attr_info->bios_ver, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->bios_ver)));
			break;
		case RHBA_BIOS_STATE:
			hba_attr_info->boot_state_valid = true;
			hba_attr_info->boot_state = ocs_be32toh(ae->attr_int);
			break;
		case RHBA_VENDOR_ID:
			hba_attr_info->vendor_id_valid = true;
			ocs_strncpy(hba_attr_info->vendor_id, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(hba_attr_info->vendor_id)));
			break;
		default:
			ocs_log_warn(node->ocs, "Unrecognized HBA attribute(type: 0x%x)\n", attr_type);
		}

		/* Now point to next attribute */
		offset += attr_len;
		num_attr_entries--;
	}

	return 0;
}

/**
 * @brief Process the GPAT acc payload.
 *
 * @par Description
 * The GPAT payload is parsed, and new nodes are created, as needed.
 *
 * @param node Pointer to the node structure.
 * @param gpat Pointer to the GPAT payload.
 * @param gpat_len Payload length
 * @param buf destination buffer to copy the Port attributes info
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */
int32_t
ocs_process_fdmi_gpat_payload(ocs_node_t *node, fcct_fdmi_gpat_rsp_t *gpat,
			      uint32_t gpat_len, void *buf)
{
	uint16_t residual;
	ocs_fdmi_port_attr_info_t *port_attr_info = buf;
	ocs_fdmi_attr_block_t *port_attr_block;
	ocs_fdmi_attr_def_t   *attr_def;
	ocs_fdmi_attr_entry_t *ae;
	uint16_t attr_type, attr_len;
	uint32_t offset, num_attr_entries;

	residual = ocs_be16toh(gpat->hdr.max_residual_size);
	if (residual != 0) {
		ocs_log_debug(node->ocs, "residual is %u words\n", residual);
	}

	if (ocs_be16toh(gpat->hdr.cmd_rsp_code) == FCCT_HDR_CMDRSP_REJECT) {
		node_printf(node, "GPAT request failed: rsn x%x rsn_expl x%x\n",
			    gpat->hdr.reason_code, gpat->hdr.reason_code_explanation);
		return -1;
	}

	offset = sizeof(gpat->hdr);
	port_attr_block = (ocs_fdmi_attr_block_t *)((uint8_t *)gpat + offset);
	port_attr_info->num_port_attr_entries = ocs_be32toh(port_attr_block->num_entries);

	num_attr_entries = port_attr_info->num_port_attr_entries;
	offset += sizeof(port_attr_block->num_entries);

	/* Walk through all attribute entries and copy the same to user buf */
	while (num_attr_entries) {
		/* Point to Port attribute entry */
		attr_def = (ocs_fdmi_attr_def_t *)(((uint8_t *)gpat + offset));
		attr_type = ocs_be16toh(attr_def->attr_type);
		attr_len  = ocs_be16toh(attr_def->attr_len);
		ae = &attr_def->attr_value;

		switch (attr_type) {
		case RPRT_SUPPORTED_FC4_TYPES:
			port_attr_info->supported_fc_types_valid= true;
			ocs_memcpy(port_attr_info->supported_fc4_types, ae->attr_types,
				   sizeof(port_attr_info->supported_fc4_types));
			break;
		case RPRT_SUPPORTED_SPEED:
			port_attr_info->supported_speed_valid = true;
			port_attr_info->supported_speed = ocs_be32toh(ae->attr_int);
			break;
		case RPRT_PORT_SPEED:
			port_attr_info->current_speed_valid = true;
			port_attr_info->current_port_speed = ocs_be32toh(ae->attr_int);
			break;
		case RPRT_MAX_FRAME_SIZE:
			port_attr_info->max_frame_size_valid = true;
			port_attr_info->max_frame_size = ocs_be32toh(ae->attr_int);
			break;
		case RPRT_OS_DEVICE_NAME:
			port_attr_info->os_dev_name_valid = true;
			ocs_strncpy(port_attr_info->os_dev_name, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(port_attr_info->os_dev_name)));
			break;
		case RPRT_HOST_NAME:
			port_attr_info->host_name_valid = true;
			ocs_strncpy(port_attr_info->host_name, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(port_attr_info->host_name)));
			break;
		case RPRT_NODENAME:
			port_attr_info->node_name_valid = true;
			port_attr_info->node_name = ocs_be64toh(ae->attr_wwn);
			break;
		case RPRT_PORTNAME:
			port_attr_info->port_name_valid = true;
			port_attr_info->port_name = ocs_be64toh(ae->attr_wwn);
			break;
		case RPRT_SYM_PORTNAME:
			port_attr_info->port_symb_name_valid = true;
			ocs_strncpy(port_attr_info->port_symb_name, (char *)ae->attr_string,
				    MIN(sizeof(ae->attr_string),
					sizeof(port_attr_info->port_symb_name)));
			break;
		case RPRT_PORT_TYPE:
			port_attr_info->port_type_valid = true;
			port_attr_info->port_type = ocs_be32toh(ae->attr_int);
			break;
		case RPRT_SUPPORTED_CLASS:
			port_attr_info->supported_cos_valid = true;
			port_attr_info->supported_class_of_service = ocs_be32toh(ae->attr_int);
			break;
		case RPRT_FABRICNAME:
			port_attr_info->port_fabric_name_valid = true;
			port_attr_info->port_fabric_name = ocs_be64toh(ae->attr_wwn);
			break;
		case RPRT_ACTIVE_FC4_TYPES:
			port_attr_info->active_fc_types_valid= true;
			ocs_memcpy(port_attr_info->active_fc4_types, ae->attr_types,
				   sizeof(port_attr_info->active_fc4_types));
			break;
		case RPRT_PORT_STATE:
			port_attr_info->port_state_valid = true;
			port_attr_info->port_state = ocs_be32toh(ae->attr_int);
			break;
		case RPRT_DISC_PORT:
			port_attr_info->num_disc_ports_valid = true;
			port_attr_info->num_disc_ports = ocs_be32toh(ae->attr_int);
			break;
		case RPRT_PORT_ID:
			port_attr_info->port_id_valid = true;
			port_attr_info->port_identifier = ocs_be32toh(ae->attr_int);
			break;
		default:
			ocs_log_warn(node->ocs, "Unrecognized HBA attribute(type: 0x%x)\n", attr_type);
		}

		/* Now point to next attribute */
		offset += attr_len;
		num_attr_entries--;
	}

	return 0;
}

int32_t
ocs_process_ganxt_payload(ocs_node_t *node, fcct_ganxt_acc_t *ganxt,
			      uint32_t ganxt_len, void *buf)
{
	uint16_t residual;
	ocs_ganxt_port_info_t *ganxt_buf = buf;

	residual = ocs_be16toh(ganxt->hdr.max_residual_size);
	if (residual != 0) {
		ocs_log_debug(node->ocs, "residual is %u words\n", residual);
	}

	if (ocs_be16toh(ganxt->hdr.cmd_rsp_code) == FCCT_HDR_CMDRSP_REJECT) {
		node_printf(node, "GA_NXT request failed: rsn x%x rsn_expl x%x\n",
			    ganxt->hdr.reason_code, ganxt->hdr.reason_code_explanation);
		return -1;
	}
	
	ganxt_buf->port_type	     = ganxt->port_type;
	ganxt_buf->port_id           = fc_be24toh(ganxt->port_id);
	ganxt_buf->port_name         = ocs_be64toh(ganxt->port_name);
	ganxt_buf->sym_port_name_len = ganxt->sym_port_name_len;
	ganxt_buf->sym_node_name_len = ganxt->sym_node_name_len;
	ganxt_buf->node_name         = ocs_be64toh(ganxt->node_name);
	ganxt_buf->fabric_port_name  = ocs_be64toh(ganxt->fabric_port_name);
	ganxt_buf->hard_address      = fc_be24toh(ganxt->hard_address);

	ocs_strncpy(ganxt_buf->sym_port_name, ganxt->sym_port_name, MIN(ganxt->sym_port_name_len, sizeof(ganxt_buf->sym_port_name)));
	ocs_strncpy(ganxt_buf->sym_node_name, ganxt->sym_node_name, MIN(ganxt->sym_node_name_len, sizeof(ganxt_buf->sym_node_name)));
	
	ganxt_buf->class_of_serv     = ocs_be32toh(ganxt->class_of_serv);
	memcpy(ganxt_buf->protocol_types, ganxt->protocol_types, sizeof(ganxt->protocol_types));
		
	return 0;
}

/**
 * @ingroup mgmt_srv_sm
 * @brief Management server node state machine: Wait for RPRT/RPA response event.
 *
 * @par Description
 * Waits for an RPRT/RPA response event;
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fdmi_reg_port_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	uint32_t expected_code, resp_status;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:
		if (node->sport->is_vport)
			expected_code = FC_GS_FDMI_RPRT;
		else
			expected_code = FC_GS_FDMI_RPA;

		if (node_check_ct_req(ctx, evt, arg, expected_code, __ocs_fabric_common, __func__))
			return NULL;

		resp_status = node_check_ct_resp(ctx, evt, arg);
		if (resp_status == FCCT_HDR_CMDRSP_REJECT) {
			ocs_log_debug(node->ocs, "FDMI reg port request(cmd: 0x%x) curr Port attrmask: 0x%x\n",
				      expected_code, node->sport->fdmi_port_mask);

			/* If rejected & mask is V2, fall back to FDMI-V1 and send RPRT/RPA */
			if (node->sport->fdmi_port_mask == FDMI_PORT_MASK_V2) {
				node->sport->fdmi_port_mask = FDMI_PORT_MASK_V1;
				ocs_log_debug(node->ocs, "Send regport with port attrmask: 0x%x\n",
					      node->sport->fdmi_port_mask);
				if (ocs_fdmi_send_reg_port(node, expected_code,
							   OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
							   OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
					ocs_node_transition(node, __ocs_fdmi_reg_port_wait_rsp, NULL);
				} else {
					node_printf(node, "Failed to send FDMI RPA req, shutting down node\n");
					ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
				}

				break;
			}
		}

		ocs_node_transition(node, __ocs_mgmt_srv_idle, NULL);
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup mgmt_srv_sm
 * @brief Management server node state machine: Wait for RHBA response event.
 *
 * @par Description
 * Waits for an RHBA response event;
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fdmi_rhba_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int resp_status;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:
		if (node_check_ct_req(ctx, evt, arg, FC_GS_FDMI_RHBA, __ocs_fabric_common, __func__))
			return NULL;

		resp_status = node_check_ct_resp(ctx, evt, arg);
		if (resp_status == FCCT_HDR_CMDRSP_REJECT) {
			ocs_log_debug(node->ocs, "FDMI RHBA request got rejected curr HBA attrmask: 0x%x\n",
				      node->sport->fdmi_hba_mask);

			/* If rejected & mask is V2, fall back to FDMI-V1 and send RHBA */
			/* Note: Right now MASK_V1 == MASK_V2 so this skips straight to idle! */
			if (node->sport->fdmi_hba_mask != FDMI_RHBA_MASK_V1) {
				node->sport->fdmi_hba_mask = FDMI_RHBA_MASK_V1;
				if (ocs_fdmi_send_rhba(node, OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
							OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
					ocs_node_transition(node, __ocs_fdmi_rhba_wait_rsp, NULL);
				} else {
					node_printf(node, "Failed to send FDMI RHBA req, shutting down node\n");
					ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
				}
			} else {
				ocs_node_transition(node, __ocs_mgmt_srv_idle, NULL);
			}
		} else if (resp_status == FCCT_HDR_CMDRSP_ACCEPT) {
			if (ocs_fdmi_send_reg_port(node, FC_GS_FDMI_RPA,
						   OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
						   OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
				ocs_node_transition(node, __ocs_fdmi_reg_port_wait_rsp, NULL);
			} else {
				node_printf(node, "Failed to send FDMI RPA req, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			}
		}

		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup mgmt_srv_sm
 * @brief Management server node state machine: Wait for DPRT response event.
 *
 * @par Description
 * Waits for an DPRT response event;
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fdmi_dprt_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:
		if (node_check_ct_req(ctx, evt, arg, FC_GS_FDMI_DPRT, __ocs_fabric_common, __func__))
			return NULL;

		if (node->sport->is_vport) {
			if (ocs_fdmi_send_reg_port(node, FC_GS_FDMI_RPRT,
						   OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
						   OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
				ocs_node_transition(node, __ocs_fdmi_reg_port_wait_rsp, NULL);
			} else {
				node_printf(node, "Failed to send FDMI RPRT req, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			}
		} else {
			if (ocs_fdmi_send_rhba(node, OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
						OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
				ocs_node_transition(node, __ocs_fdmi_rhba_wait_rsp, NULL);
			} else {
				node_printf(node, "Failed to send FDMI RHBA req, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			}
		}

		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup mgmt_srv_sm
 * @brief Management server node state machine: Wait for DHBA response event.
 *
 * @par Description
 * Waits for an DHBA response event;
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_fdmi_dhba_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:
		if (node_check_ct_req(ctx, evt, arg, FC_GS_FDMI_DHBA, __ocs_fabric_common, __func__))
			return NULL;

		/* Now send FDMI DPRT request */
		if (ocs_fdmi_send_dereg_cmd(node, FC_GS_FDMI_DPRT,
					    OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
					    OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
			ocs_node_transition(node, __ocs_fdmi_dprt_wait_rsp, NULL);
		} else {
			node_printf(node, "Failed to send FDMI DPRT req, shutting down node\n");
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		}

		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup mgmt_srv_sm
 * @brief Management server node state machine: Initialize.
 *
 * @par Description
 * A PLOGI is sent to the well-known mgmt server node.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_mgmt_srv_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		if (ocs_send_plogi(node, OCS_FC_ELS_SEND_DEFAULT_TIMEOUT, OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
			ocs_node_transition(node, __ocs_mgmt_srv_plogi_wait_rsp, NULL);
		} else {
			node_printf(node, "Failed to send PLOGI req, shutting down node\n");
			ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
		}

		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup mgmt_srv_sm
 * @brief Management server node state machine: Wait for domain attach.
 *
 * @par Description
 * Waits for domain to be attached then send
 * node attach request to the HAL.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_mgmt_srv_wait_domain_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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
		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_mgmt_srv_wait_node_attach, NULL);

		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}
		break;
	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup mgmt_srv_sm
 * @brief Management server node state machine: Wait for a PLOGI response.
 *
 * @par Description
 * Waits for a response from PLOGI to mgmt server node, then issues a
 * node attach request to the HAL.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_mgmt_srv_plogi_wait_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc;
	ocs_node_cb_t *cbdata = arg;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_SRRS_ELS_REQ_OK:
		if (node_check_els_req(ctx, evt, arg, FC_ELS_CMD_PLOGI, __ocs_fabric_common, __func__))
			return NULL;

		/* Save sparams, and call ocs_node_attach() */
		ocs_node_save_sparms(node, cbdata->els->els_info->els_rsp.virt);
		ocs_display_sparams(node->display_name, "plogi rcvd resp", 0, NULL,
				    ((uint8_t *)cbdata->els->els_info->els_rsp.virt) + 4);

		/* Wait for domain to be attach then proceed with node attach */
		if (!node->sport->domain->attached) {
			ocs_node_transition(node, __ocs_mgmt_srv_wait_domain_attach, NULL);
			break;
		}

		rc = ocs_node_attach(node);
		ocs_node_transition(node, __ocs_mgmt_srv_wait_node_attach, NULL);
		if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
		} else if (rc != OCS_HAL_RTN_SUCCESS) {
			ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_FAIL, NULL);
		}

		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup mgmt_srv_sm
 * @brief Management server node state machine: Wait for a node attach completion.
 *
 * @par Description
 * Waits for a node attach completion, then issues RHAT/RPRT registration
 * requests to FDMI server.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_mgmt_srv_wait_node_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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

	case OCS_EVT_NODE_ATTACH_OK:
		node->attached = TRUE;

		/*
		 * SM transaction
		 * Physical Port: DHBA -> DPRT -> RHBA -> RPA
		 * Virtual Port: DPRT -> RPRT
		 */
		node->sport->fdmi_port_mask = FDMI_PORT_MASK_V2;
		if (node->sport->is_vport) {
			if (ocs_fdmi_send_dereg_cmd(node, FC_GS_FDMI_DPRT,
						    OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
						    OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
				ocs_node_transition(node, __ocs_fdmi_dprt_wait_rsp, NULL);
			} else {
				node_printf(node, "Failed to send FDMI DPRT req, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			}
		} else {
			node->sport->fdmi_hba_mask = FDMI_RHBA_MASK_V2;
			if (ocs_fdmi_send_dereg_cmd(node, FC_GS_FDMI_DHBA,
						    OCS_FC_ELS_CT_SEND_DEFAULT_TIMEOUT,
						    OCS_FC_ELS_DEFAULT_RETRIES, NULL, NULL)) {
				ocs_node_transition(node, __ocs_fdmi_dhba_wait_rsp, NULL);
			} else {
				node_printf(node, "Failed to send FDMI DHBA req, shutting down node\n");
				ocs_node_post_event(node, OCS_EVT_SHUTDOWN_EXPLICIT_LOGO, NULL);
			}
		}

		break;

	case OCS_EVT_NODE_ATTACH_FAIL:
		/* node attach failed, shutdown the node */
		node->attached = FALSE;
		node_printf(node, "Node attach failed\n");
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_fabric_initiate_shutdown(node);
		break;

	case OCS_EVT_SHUTDOWN:
		node_printf(node, "Shutdown event received\n");
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		ocs_node_transition(node, __ocs_fabric_wait_attach_evt_shutdown, NULL);
		break;

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup mgmt_srv_sm
 * @brief Management server node state machine: node is idle.
 *
 * @par Description
 * Wait for mgmt server node events.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */
void *
__ocs_mgmt_srv_idle(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_DOMAIN_ATTACH_OK:
		break;

	case OCS_EVT_NODE_TDZ_CMD: {
		ocs_tdz_cmd_args_t *args = arg;
		ocs_tdz_rsp_info_t *rsp;

		ocs_assert(args, NULL);
		ocs_assert(args->cb_arg, NULL);

		rsp = args->cb_arg;
		if (!ocs_tdz_send_cmd(node, OCS_FC_MGMT_SERVER_DEFAULT_TIMEOUT, OCS_FC_ELS_DEFAULT_RETRIES,
				      args->cb, args->cb_arg, args->tdz_req)) {
			ocs_log_err(node->ocs, "Failed to send TDZ request\n");
			rsp->status = -1;
			args->cb(node, NULL, args->cb_arg);
		}

		break;
	}

	case OCS_EVT_NODE_FDMI_GET_CMD: {
		ocs_fdmi_get_cmd_args_t *args = arg;
		ocs_fdmi_get_cmd_results_t *result;

		ocs_assert(args, NULL);
		ocs_assert(args->cb_arg, NULL);

		result = args->cb_arg;
		if (!ocs_fdmi_send_get_cmd(node, OCS_FC_MGMT_SERVER_DEFAULT_TIMEOUT, OCS_FC_ELS_DEFAULT_RETRIES,
					   args->cb, args->cb_arg, args->fdmi_cmd_req)) {
			ocs_log_err(node->ocs, "Failed to send FDMI request\n");
			result->status = -1;
			args->cb(node, NULL, args->cb_arg);
		}

		break;
	}

	default:
		__ocs_fabric_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

int
ocs_tdz_issue_fabric_cmd(ocs_sport_t *sport, els_cb_t cb, ocs_tdz_rsp_info_t *tdz_rsp, ocs_tdz_req_info_t *tdz_req)
{
	ocs_node_t *node;
	ocs_tdz_cmd_args_t args;

	node = ocs_node_find(sport, FC_ADDR_MGMT_SERVER);
	if (node == NULL) {
		ocs_log_err(sport->ocs, "Failed to find fabric zone server node\n");
		return -1;
	}

	args.cb = cb;
	args.cb_arg = tdz_rsp;
	args.tdz_req = tdz_req;

	ocs_node_post_event(node, OCS_EVT_NODE_TDZ_CMD, &args);

	return 0;
}

int
ocs_fdmi_get_cmd(ocs_sport_t *sport, els_cb_t cb, ocs_fdmi_get_cmd_results_t *cb_arg,
		 ocs_fdmi_get_cmd_req_info_t *fdmi_cmd_req)
{
	ocs_node_t *node;
	ocs_fdmi_get_cmd_args_t args;

	node = ocs_node_find(sport, FC_ADDR_MGMT_SERVER);
	if (node == NULL) {
		ocs_log_err(sport->ocs, "Failed to find FDMI node\n");
		return -1;
	}

	args.fdmi_cmd_req = fdmi_cmd_req;
	args.cb = cb;
	args.cb_arg = cb_arg;

	ocs_node_post_event(node, OCS_EVT_NODE_FDMI_GET_CMD, &args);

	return 0;
}

int
ocs_ganxt_get_cmd(ocs_sport_t *sport, els_cb_t cb, ocs_ganxt_get_cmd_results_t *cb_arg,
		 ocs_ganxt_get_cmd_req_info_t *ganxt_cmd_req)
{
	ocs_node_t *node;
	ocs_ganxt_get_cmd_args_t args;

	node = ocs_node_find(sport, FC_ADDR_NAMESERVER);
	if (node == NULL) {
		ocs_log_err(sport->ocs, "Failed to find GA_NXT node\n");
		return -1;
	}
	
	args.ganxt_cmd_req = ganxt_cmd_req;
	args.cb = cb;
	args.cb_arg = cb_arg;

	ocs_node_post_event(node, OCS_EVT_NODE_GANXT_GET_CMD, &args);
	return 0;
}

int ocs_sport_fpin_send_event(ocs_sport_t *sport, void *args)
{
	ocs_node_t *node;

	node = ocs_node_find(sport, FC_ADDR_CONTROLLER);
	if (!node) {
		ocs_log_err(sport->ocs, "Failed to find fabric controller node\n");
		return -1;
	}

	ocs_node_post_event(node, OCS_EVT_SEND_FPIN, args);

	return 0;
}

/* Routines for all individual HBA attributes */
int
ocs_fdmi_hba_attr_wwnn(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;
	uint64_t wwnn = ocs_be64toh(sport->wwnn);

	ae = &ad->attr_value;
	ocs_memset(ae, 0, sizeof(sport->wwnn));

	ocs_memcpy(&ae->attr_wwn, &wwnn,
			sizeof(sport->sli_wwnn));
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(sport->sli_wwnn);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_NODENAME);
	return size;
}
int
ocs_fdmi_hba_attr_manufacturer(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	ocs_strncpy(ae->attr_string, OCS_FDMI_MODEL_MANUFACTURER, sizeof(ae->attr_string));
	len = ocs_strnlen((char *)ae->attr_string, sizeof(ae->attr_string));
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_MANUFACTURER);
	return size;
}

int
ocs_fdmi_hba_attr_sn(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_t *ocs = sport->ocs;
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size, serialnum_len = 0;
	char *serialnum = NULL;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	serialnum = ocs_scsi_get_property_ptr(ocs, OCS_SCSI_SERIALNUMBER);
	serialnum_len = *serialnum++;

	ocs_snprintf(ae->attr_string, MIN(sizeof(ae->attr_string), (serialnum_len + 1)), "%s", serialnum);
	len = ocs_strlen(ae->attr_string);
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_SERIAL_NUMBER);
	return size;
}

int
ocs_fdmi_hba_attr_model(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	ocs_strncpy(ae->attr_string, OCS_FDMI_MODEL_NAME, sizeof(ae->attr_string));
	len = ocs_strnlen(ae->attr_string, sizeof(ae->attr_string));
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_MODEL);
	return size;
}

int
ocs_fdmi_hba_attr_description(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	ocs_strncpy(ae->attr_string, OCS_FDMI_MODEL_DESCRIPTION, sizeof(ae->attr_string));
	len = ocs_strnlen(ae->attr_string, sizeof(ae->attr_string));
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_MODEL_DESCRIPTION);

	return size;
}

int
ocs_fdmi_hba_attr_drvr_ver(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	ocs_t *ocs = sport->ocs;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	ocs_strncpy(ae->attr_string, ocs->driver_version,
			sizeof(ae->attr_string));
	len = ocs_strnlen(ae->attr_string,
			sizeof(ae->attr_string));
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_DRIVER_VERSION);
	return size;
}

int
ocs_fdmi_hba_attr_rom_ver(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_t *ocs = sport->ocs;
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	ocs_snprintf(ae->attr_string, MIN((ocs_strlen(ocs->fw_version) + 1), sizeof(ae->attr_string)), "%s", ocs->fw_version);
	len = ocs_strnlen(ae->attr_string, sizeof(ae->attr_string));
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_OPTION_ROM_VERSION);
	return size;
}

int
ocs_fdmi_hba_attr_fmw_ver(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	ocs_t *ocs = sport->ocs;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	ocs_snprintf(ae->attr_string, MIN((ocs_strlen(ocs->fw_version) + 1), sizeof(ae->attr_string)), "%s", ocs->fw_version);
	len = ocs_strnlen(ae->attr_string, sizeof(ae->attr_string));
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_FIRMWARE_VERSION);
	return size;
}

int
ocs_fdmi_hba_attr_os_ver(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	ocs_get_os_version_and_name(sport->ocs, ae->attr_string,
				    sizeof(ae->attr_string));

	len = ocs_strnlen(ae->attr_string, sizeof(ae->attr_string));
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_OS_NAME_VERSION);
	return size;
}

int
ocs_fdmi_hba_attr_ct_len(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;

	ae->attr_int =  ocs_htobe32(OCS_MAX_CT_SIZE);
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(uint32_t);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_MAX_CT_PAYLOAD_LEN);
	return size;
}

int
ocs_fdmi_hba_attr_symbolic_name(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_t *ocs = sport->ocs;
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	ocs_snprintf(ae->attr_string, sizeof(ae->attr_string),
			"Emulex %s FV%s DV%s", ocs->model,ocs->fw_version,
			ocs->driver_version);

	len = ocs_strnlen(ae->attr_string, sizeof(ae->attr_string));
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_SYM_NODENAME);
	return size;
}

int
ocs_fdmi_hba_attr_vendor_info(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;
	ae->attr_int = 0;

	if (sport && sport->ocs)
		ae->attr_int = ocs_htobe32(sport->ocs->pci_vendor);

	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(ae->attr_int);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_VENDOR_INFO);

	return size;
}

int
ocs_fdmi_hba_attr_num_ports(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;

	/* Each driver instance corresponds to a single port */
	ae->attr_int =  ocs_htobe32(1);
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(uint32_t);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_NUM_PORTS);
	return size;
}

int
ocs_fdmi_hba_attr_fabric_name(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size = 0;

	ae = &ad->attr_value;

	ae->attr_wwn = 0;
	if (sport && sport->ocs && sport->ocs->domain) {
		ocs_t *ocs = sport->ocs;
		fc_plogi_payload_t *sp = (fc_plogi_payload_t*)
				ocs->domain->flogi_service_params;

		ae->attr_wwn = (((uint64_t)sp->node_name_lo << 32ll) |
				sp->node_name_hi);
	}

	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(ae->attr_wwn);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_FABRIC_NAME);
	return size;
}

int
ocs_fdmi_hba_attr_bios_ver(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	if (sport && sport->ocs) {
		ocs_snprintf(ae->attr_string, sizeof(ae->attr_string), "%s",
				sli_get_bios_version_string(&sport->ocs->hal.sli));
	}

	len = ocs_strnlen(ae->attr_string, sizeof(ae->attr_string));
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_BIOS_VERSION);
	return size;
}

int
ocs_fdmi_hba_attr_bios_state(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;

	/* Driver doesn't have access to this information */
	ae->attr_int =  ocs_htobe32(0);
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(uint32_t);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_BIOS_STATE);
	return size;
}

int
ocs_fdmi_hba_attr_vendor_id(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	ocs_strncpy(ae->attr_string, "Emulex", sizeof(ae->attr_string));
	len = ocs_strnlen(ae->attr_string, sizeof(ae->attr_string));

	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RHBA_VENDOR_ID);
	return size;
}

/* Routines for all individual PORT attributes */
int
ocs_fdmi_port_attr_supported_fc4type(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 32);

	if (sport && sport->ocs) {
		if (ocs_scsi_protocol_enabled(sport->ocs))
			ae->attr_types[2] = 0x01; /* Type 0x8 - FCP */

		if (ocs_nvme_protocol_enabled(sport->ocs))
			ae->attr_types[6] = 0x01; /* Type 0x28 - NVMe */

		if (sli_feature_enabled(&sport->ocs->hal.sli, SLI4_FEATURE_ASHDR))
			ae->attr_types[12] = 0x1; /* Type 0x60 - APP SERVER */
	}

	size = FDMI_ATTR_TYPE_LEN_SIZE + 32;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_SUPPORTED_FC4_TYPES);

	return size;
}

int
ocs_fdmi_port_attr_supported_speed(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_t *ocs = sport->ocs;
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;
	ocs_xport_stats_t speed;
	uint32_t supported_speeds = 0;

	speed.value = 1000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		supported_speeds |= OCS_PORTSPEED_1GFC;
	}
	speed.value = 2000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		supported_speeds |= OCS_PORTSPEED_2GFC;
	}
	speed.value = 4000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		supported_speeds |= OCS_PORTSPEED_4GFC;
	}
	speed.value = 8000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		supported_speeds |= OCS_PORTSPEED_8GFC;
	}
	speed.value = 16000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		supported_speeds |= OCS_PORTSPEED_16GFC;
	}
	speed.value = 32000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		supported_speeds |= OCS_PORTSPEED_32GFC;
	}
	speed.value = 64000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		supported_speeds |= OCS_PORTSPEED_64GFC;
	}
	speed.value = 128000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		supported_speeds |= OCS_PORTSPEED_128GFC;
	}
	speed.value = 256000;
	if (ocs_xport_status(ocs->xport, OCS_XPORT_IS_SUPPORTED_LINK_SPEED, &speed)) {
		supported_speeds |= OCS_PORTSPEED_256GFC;
	}

	ae = &ad->attr_value;
	ae->attr_int = ocs_htobe32(supported_speeds);
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(uint32_t);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_SUPPORTED_SPEED);
	return size;
}

int
ocs_fdmi_port_attr_speed(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_t *ocs = sport->ocs;
	ocs_fdmi_attr_entry_t *ae;
	ocs_xport_stats_t speed;
	uint32_t size;
	int rc;

	ae = &ad->attr_value;

	ae->attr_int = OCS_PORTSPEED_UNKNOWN;
	rc = ocs_xport_status(ocs->xport, OCS_XPORT_LINK_SPEED, &speed);
	if (rc == 0) {
		switch (speed.value) {
		case 1000:
			ae->attr_int = OCS_PORTSPEED_1GFC;
			break;
		case 2000:
			ae->attr_int = OCS_PORTSPEED_2GFC;
			break;
		case 4000:
			ae->attr_int = OCS_PORTSPEED_4GFC;
			break;
		case 8000:
			ae->attr_int = OCS_PORTSPEED_8GFC;
			break;
		case 16000:
			ae->attr_int = OCS_PORTSPEED_16GFC;
			break;
		case 32000:
			ae->attr_int = OCS_PORTSPEED_32GFC;
			break;
		case 64000:
			ae->attr_int = OCS_PORTSPEED_64GFC;
			break;
		case 128000:
			ae->attr_int = OCS_PORTSPEED_128GFC;
			break;
		case 256000:
			ae->attr_int = OCS_PORTSPEED_256GFC;
			break;
		}
	}

	ae->attr_int = ocs_htobe32(ae->attr_int);
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(uint32_t);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_PORT_SPEED);
	return size;
}

int
ocs_fdmi_port_attr_max_frame(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;
	ae->attr_int = ocs_htobe32(ocs_sport_get_max_frame_size(sport));
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(uint32_t);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_MAX_FRAME_SIZE);
	return size;
}

int
ocs_fdmi_port_attr_os_devname(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	if (sport && sport->ocs)
		ocs_scsi_get_host_devname(sport->ocs, ae->attr_string, sizeof(ae->attr_string));

	len = ocs_strnlen((char *)ae->attr_string, sizeof(ae->attr_string));

	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_OS_DEVICE_NAME);
	return size;
}

int
ocs_fdmi_port_attr_host_name(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	ocs_get_os_host_name(sport->ocs, ae->attr_string, sizeof(ae->attr_string));

	len = ocs_strnlen(ae->attr_string, sizeof(ae->attr_string));
	len += 4 - (len & 3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_HOST_NAME);
	return size;
}

int
ocs_fdmi_port_attr_wwnn(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;
	uint64_t wwnn = ocs_be64toh(sport->wwnn);

	ae = &ad->attr_value;
	ocs_memset(ae, 0, sizeof(sport->sli_wwnn));

	ocs_memcpy(&ae->attr_wwn, &wwnn,
			sizeof(sport->sli_wwnn));
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(sport->sli_wwnn);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_NODENAME);
	return size;
}

int
ocs_fdmi_port_attr_wwpn(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, sizeof(sport->sli_wwpn));

	ocs_memcpy(&ae->attr_wwn, &sport->sli_wwpn,
			sizeof(sport->sli_wwnn));
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(sport->sli_wwpn);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_PORTNAME);
	return size;
}

int
ocs_fdmi_port_attr_symbolic_name(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t len, size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 256);

	len = ocs_snprintf(ae->attr_string, 256, "%d", sport->instance_index);
	len += (len & 3) ? (4 - (len & 3)) : 0;
	size = FDMI_ATTR_TYPE_LEN_SIZE + len;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_SYM_PORTNAME);
	return size;
}

int
ocs_fdmi_port_attr_port_type(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;
	if (sport->topology == OCS_SPORT_TOPOLOGY_LOOP) {
		ae->attr_int =  ocs_htobe32(OCS_FDMI_PORTTYPE_NLPORT);
	} else if ((sport->topology == OCS_SPORT_TOPOLOGY_FABRIC) ||
			(sport->topology == OCS_SPORT_TOPOLOGY_LOOP)) {
		ae->attr_int =  ocs_htobe32(OCS_FDMI_PORTTYPE_NPORT);
	} else {
		ae->attr_int =  ocs_htobe32(OCS_FDMI_PORTTYPE_UNKNOWN);
	}
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(uint32_t);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_PORT_TYPE);
	return size;
}

int
ocs_fdmi_port_attr_class(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;
	ae->attr_int = ocs_htobe32(FCCT_CLASS_OF_SERVICE_3);
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(uint32_t);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_SUPPORTED_CLASS);
	return size;
}

int
ocs_fdmi_port_attr_fabric_name(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size = 0;

	ae = &ad->attr_value;

	ae->attr_wwn = 0;
	if (sport && sport->ocs && sport->ocs->domain) {
		ocs_t *ocs = sport->ocs;
		fc_plogi_payload_t *sp = (fc_plogi_payload_t*)
			ocs->domain->flogi_service_params;

		ae->attr_wwn = (((uint64_t)sp->node_name_lo << 32ll) |
				sp->node_name_hi);
	}

	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(ae->attr_wwn);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_FABRICNAME);
	return size;
}

int
ocs_fdmi_port_attr_active_fc4type(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;
	ocs_memset(ae, 0, 32);

	if (sport && sport->ocs) {
		if (ocs_scsi_backend_enabled(sport->ocs, sport))
			ae->attr_types[2] = 0x01; /* Type 0x8 - FCP */

		if (ocs_nvme_backend_enabled(sport->ocs, sport))
			ae->attr_types[6] = 0x01; /* Type 0x28 - NVMe */
	}

	size = FDMI_ATTR_TYPE_LEN_SIZE + 32;
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_ACTIVE_FC4_TYPES);
	return size;
}

int
ocs_fdmi_port_attr_port_state(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_t *ocs = sport->ocs;
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;
	ocs_xport_stats_t link_status;
	int rc;

	ae = &ad->attr_value;

	rc = ocs_xport_status(ocs->xport, OCS_XPORT_PORT_STATUS, &link_status);
	if ((rc == OCS_HAL_RTN_SUCCESS) &&
			(link_status.value == OCS_XPORT_PORT_ONLINE)) {
		ae->attr_int =  ocs_htobe32(OCS_FDMI_PORTSTATE_ONLINE);
	} else {
		ae->attr_int =  ocs_htobe32(OCS_FDMI_PORTSTATE_OFFLINE);
	}
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(uint32_t);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_PORT_STATE);
	return size;
}

int
ocs_fdmi_port_attr_nportid(ocs_sport_t *sport, ocs_fdmi_attr_def_t *ad)
{
	ocs_fdmi_attr_entry_t *ae;
	uint32_t size;

	ae = &ad->attr_value;
	ae->attr_int =  ocs_htobe32(sport->fc_id);
	size = FDMI_ATTR_TYPE_LEN_SIZE + sizeof(uint32_t);
	ad->attr_len = ocs_htobe16(size);
	ad->attr_type = ocs_htobe16(RPRT_PORT_ID);
	return size;
}
