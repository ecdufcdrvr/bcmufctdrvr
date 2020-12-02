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
 * OCS driver remote node handler.  This file contains code that is shared
 * between fabric (ocs_fabric.c) and device (ocs_device.c) nodes.
 */

/*!
 * @defgroup node_common Node common support
 * @defgroup node_alloc Node allocation
 */

#include "ocs.h"
#include "spv.h"
#include "ocs_els.h"
#include "ocs_device.h"

#define SCSI_IOFMT " [%04x][i:%0*x t:%0*x h:%04x iotag:%04x] "
#define SCSI_ITT_SIZE(ocs)	((ocs->ocs_xport == OCS_XPORT_FC) ? 4 : 8)

#define SCSI_IOFMT_ARGS(io) io->instance_index, SCSI_ITT_SIZE(io->ocs), io->init_task_tag, SCSI_ITT_SIZE(io->ocs), io->tgt_task_tag, io->hw_tag io->tag

#define scsi_io_printf(io, fmt, ...) ocs_log_debug(io->ocs, "[%s]" SCSI_IOFMT fmt, \
	io->node->display_name, SCSI_IOFMT_ARGS(io), ##__VA_ARGS__)

void ocs_mgmt_node_list(ocs_textbuf_t *textbuf, void *node);
void ocs_mgmt_node_get_all(ocs_textbuf_t *textbuf, void *node);
int ocs_mgmt_node_get(ocs_textbuf_t *textbuf, char *parent, char *name, void *node);
int ocs_mgmt_node_set(char *parent, char *name, char *value, void *node);
int ocs_mgmt_node_exec(char *parent, char *action, void *arg_in, uint32_t arg_in_length,
		void *arg_out, uint32_t arg_out_length, void *node);
static ocs_mgmt_functions_t node_mgmt_functions = {
	.get_list_handler	=	ocs_mgmt_node_list,
	.get_handler		=	ocs_mgmt_node_get,
	.get_all_handler	=	ocs_mgmt_node_get_all,
	.set_handler		=	ocs_mgmt_node_set,
	.exec_handler		=	ocs_mgmt_node_exec,
};


/**
 * @ingroup node_common
 * @brief Device node state machine wait for all ELS's to
 *        complete
 *
 * Abort all ELS's for given node.
 *
 * @param node node for which ELS's will be aborted
 */

void
ocs_node_abort_all_els(ocs_node_t *node)
{
	ocs_io_t *els;
	ocs_io_t *els_next;
	ocs_node_cb_t cbdata = {0};

	ocs_node_hold_frames(node);
	ocs_lock(&node->active_ios_lock);
		ocs_list_foreach_safe(&node->els_io_active_list, els, els_next) {
			if (els->els_info->els_rjt) {
				ocs_log_debug(node->ocs, "[%s] Skipping ABORT for LS Reject", node->display_name);
				continue;
			}

			ocs_log_debug(node->ocs, "[%s] initiate ELS abort %s oxid %04x rxid %04x\n",
				      node->display_name, els->display_name, els->init_task_tag, els->tgt_task_tag);
			ocs_unlock(&node->active_ios_lock);
			cbdata.els = els;
			ocs_els_post_event(els, OCS_EVT_ABORT_ELS, &cbdata);
			ocs_lock(&node->active_ios_lock);
		}
	ocs_unlock(&node->active_ios_lock);
}

/**
 * @ingroup node_common
 * @brief Handle remote node events from HAL
 *
 * Handle remote node events from HAL.   Essentially the HAL event is translated into
 * a node state machine event that is posted to the affected node.
 *
 * @param arg pointer to ocs
 * @param event HAL event to proceoss
 * @param data application specific data (pointer to the affected node)
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
int32_t
ocs_remote_node_cb(void *arg, ocs_hal_remote_node_event_e event, void *data)
{
	ocs_t *ocs = arg;
	ocs_sm_event_t	sm_event = OCS_EVT_LAST;
	ocs_remote_node_t *rnode = data;
	ocs_node_t *node = rnode->node;

	switch (event) {
	case OCS_HAL_NODE_ATTACH_OK:
		sm_event = OCS_EVT_NODE_ATTACH_OK;
		break;

	case OCS_HAL_NODE_ATTACH_FAIL:
		sm_event = OCS_EVT_NODE_ATTACH_FAIL;
		break;

	case OCS_HAL_NODE_FREE_OK:
		sm_event = OCS_EVT_NODE_FREE_OK;
		break;

	case OCS_HAL_NODE_FREE_FAIL:
		sm_event = OCS_EVT_NODE_FREE_FAIL;
		break;

	default:
		ocs_log_test(ocs, "unhandled event %#x\n", event);
		return -1;
	}

	/* If we're using HLM, forward the NODE_ATTACH_OK/FAIL event to all nodes in the node group */
	if ((node->node_group != NULL) &&
			((sm_event == OCS_EVT_NODE_ATTACH_OK) || (sm_event == OCS_EVT_NODE_ATTACH_FAIL))) {
		ocs_node_t *n = NULL;
		uint8_t		attach_ok = sm_event == OCS_EVT_NODE_ATTACH_OK;

		ocs_sport_lock(node->sport);
		{
			ocs_list_foreach(&node->sport->node_list, n) {
				if (node == n) {
					continue;
				}
				ocs_node_lock(n);
					if ((!n->rnode.attached) && (node->node_group == n->node_group)) {
						n->rnode.attached = attach_ok;
						node_printf(n, "rpi[%d] deferred HLM node attach %s posted\n",
								n->rnode.index, attach_ok ? "ok" : "fail");

						if (!n->rnode.attached) {
							ocs_atomic_sub_return(&ocs->hal.rpi_ref[n->rnode.index].rpi_count, 1);
							if (0 == ocs_atomic_read(&ocs->hal.rpi_ref[n->rnode.index].rpi_count))
								ocs_atomic_set(&ocs->hal.rpi_ref[n->rnode.index].rpi_attached, 0);
						}

						ocs_node_post_event(n, sm_event, NULL);
					}
				ocs_node_unlock(n);
			}
		}

		ocs_sport_unlock(node->sport);
	}

	if (sm_event == OCS_EVT_NODE_ATTACH_FAIL) {
		ocs_atomic_sub_return(&ocs->hal.rpi_ref[node->rnode.index].rpi_count, 1);
		if (0 == ocs_atomic_read(&ocs->hal.rpi_ref[node->rnode.index].rpi_count))
			ocs_atomic_set(&ocs->hal.rpi_ref[node->rnode.index].rpi_attached, 0);
	}

	ocs_node_post_event(node, sm_event, NULL);
	return 0;
}

/**
 * @ingroup node_alloc
 * @brief Find an FC node structure given the FC port ID
 *
 * @param sport the SPORT to search
 * @param port_id FC port ID
 *
 * @return pointer to the object or NULL if not found
 */
ocs_node_t *
ocs_node_find(ocs_sport_t *sport, uint32_t port_id)
{
	ocs_node_t *node;

	ocs_assert(sport->lookup, NULL);
	ocs_sport_lock(sport);
		node = spv_get(sport->lookup, port_id);
	ocs_sport_unlock(sport);
	return node;
}

/**
 * @ingroup node_alloc
 * @brief Find an FC node structure given the WWPN
 *
 * @param sport the SPORT to search
 * @param wwpn the WWPN to search for (host endian)
 *
 * @return pointer to the object or NULL if not found
 */
ocs_node_t *
ocs_node_find_wwpn(ocs_sport_t *sport, uint64_t wwpn)
{
	ocs_node_t *node = NULL;;

	ocs_assert(sport, NULL);

	ocs_sport_lock(sport);
		ocs_list_foreach(&sport->node_list, node) {
			if (ocs_node_get_wwpn(node) == wwpn) {
				ocs_sport_unlock(sport);
				return node;
			}
		}
	ocs_sport_unlock(sport);
	return NULL;
}

/**
 * @ingroup node_alloc
 * @brief Find an FC node structure given the WWNN
 *
 * @param sport the SPORT to search
 * @param wwnn the WWNN to search for (host endian)
 *
 * @return pointer to the object or NULL if not found
 */
ocs_node_t *
ocs_node_find_wwnn(ocs_sport_t *sport, uint64_t wwnn)
{
	ocs_node_t *node = NULL;;

	ocs_assert(sport, NULL);

	ocs_sport_lock(sport);
		ocs_list_foreach(&sport->node_list, node) {
			if (ocs_node_get_wwnn(node) == wwnn) {
				ocs_sport_unlock(sport);
				return node;
			}
		}
	ocs_sport_unlock(sport);

	return NULL;
}

/**
 * @ingroup node_alloc
 * @brief allocate node object pool
 *
 * A pool of ocs_node_t objects is allocated.
 *
 * @param ocs pointer to driver instance context
 * @param node_count count of nodes to allocate
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int32_t
ocs_node_create_pool(ocs_t *ocs, uint32_t node_count)
{
	ocs_xport_t *xport = ocs->xport;
	uint32_t i;
	ocs_node_t *node;
	uint32_t max_sge;
	uint32_t num_sgl;
	uint64_t max_xfer_size;
	int32_t rc;

	xport->nodes_count = node_count;

	xport->nodes = ocs_malloc(ocs, node_count * sizeof(ocs_node_t *), OCS_M_ZERO);
	if (xport->nodes == NULL) {
		ocs_log_err(ocs, "node ptrs allocation failed");
		return -1;
	}

	if (0 == ocs_hal_get(&ocs->hal, OCS_HAL_MAX_SGE, &max_sge) &&
	    0 == ocs_hal_get(&ocs->hal, OCS_HAL_N_SGL, &num_sgl)) {
		max_xfer_size = (max_sge * (uint64_t)num_sgl);
	} else {
		max_xfer_size = 65536;
	}

	ocs_list_init(&xport->nodes_free_list, ocs_node_t, link);

	for (i = 0; i < node_count; i ++) {
		node = ocs_malloc(ocs, sizeof(ocs_node_t), OCS_M_ZERO);
		if (node == NULL) {
			ocs_log_err(ocs, "node allocation failed");
			goto error;
		}

		/* Assign any persistent field values */
		node->instance_index = i;
		node->max_wr_xfer_size = max_xfer_size;
		node->rnode.indicator = UINT32_MAX;

#if !defined(OCS_USPACE)
		ocs_ratelimit_state_init(&node->ratelimit);
#endif

		rc = ocs_dma_alloc(ocs, &node->sparm_dma_buf, 256, 16);
		if (rc) {
			ocs_free(ocs, node, sizeof(ocs_node_t));
			ocs_log_err(ocs, "ocs_dma_alloc failed: %d\n", rc);
			goto error;
		}

		xport->nodes[i] = node;
		ocs_list_add_tail(&xport->nodes_free_list, node);
	}
	return 0;

error:
	ocs_node_free_pool(ocs);
	return -1;
}

/**
 * @ingroup node_alloc
 * @brief free node object pool
 *
 * The pool of previously allocated node objects is freed
 *
 * @param ocs pointer to driver instance context
 *
 * @return none
 */

void
ocs_node_free_pool(ocs_t *ocs)
{
	ocs_xport_t *xport = ocs->xport;
	ocs_node_t *node;
	uint32_t i;

	if (!xport->nodes)
		return;

	ocs_device_lock(ocs);
	for (i = 0; i < xport->nodes_count; i ++) {
		node = xport->nodes[i];
		if (node) {
			/* free sparam_dma_buf */
			ocs_dma_free(ocs, &node->sparm_dma_buf);
			ocs_free(ocs, node, sizeof(ocs_node_t));
		}

		xport->nodes[i] = NULL;
	}

	ocs_free(ocs, xport->nodes, (xport->nodes_count * sizeof(ocs_node_t *)));
	ocs_device_unlock(ocs);
}

/**
 * @ingroup node_alloc
 * @brief return pointer to node object given instance index
 *
 * A pointer to the node object given by an instance index is returned.
 *
 * @param ocs pointer to driver instance context
 * @param index instance index
 *
 * @return returns pointer to node object, or NULL
 */

ocs_node_t *
ocs_node_get_instance(ocs_t *ocs, uint32_t index)
{
	ocs_xport_t *xport = ocs->xport;
	ocs_node_t *node = NULL;

	if (index >= (xport->nodes_count)) {
		ocs_log_test(ocs, "invalid index: %d\n", index);
		return NULL;
	}
	node = xport->nodes[index];
	return node->attached ? node : NULL;
}

/**
 * @ingroup node_alloc
 * @brief Allocate an fc node structure and add to node list
 *
 * @param sport pointer to the SPORT from which this node is allocated
 * @param port_id FC port ID of new node
 * @param init Port is an inititiator (sent a plogi)
 * @param targ Port is potentially a target
 *
 * @return pointer to the object or NULL if none available
 */

ocs_node_t *
ocs_node_alloc(ocs_sport_t *sport, uint32_t port_id, uint8_t init, uint8_t targ)
{
	int32_t rc;
	ocs_node_t *node = NULL;
	uint32_t instance_index;
	uint64_t max_wr_xfer_size;
	ocs_t *ocs = sport->ocs;
	ocs_xport_t *xport = ocs->xport;
	ocs_dma_t sparm_dma_buf;

	ocs_assert(sport, NULL);

	if (sport->shutting_down) {
		ocs_log_debug(ocs, "node allocation when shutting down %06x", port_id);
		return NULL;
	}

	ocs_device_lock(ocs);
		node = ocs_list_remove_head(&xport->nodes_free_list);
	ocs_device_unlock(ocs);
	if (node == NULL) {
		ocs_log_err(ocs, "node allocation failed %06x", port_id);
		return NULL;
	}

	/* Save persistent values across memset zero */
	instance_index = node->instance_index;
	max_wr_xfer_size = node->max_wr_xfer_size;
	sparm_dma_buf = node->sparm_dma_buf;

	ocs_memset(node, 0, sizeof(*node));
	node->instance_index = instance_index;
	node->max_wr_xfer_size = max_wr_xfer_size;
	node->sparm_dma_buf = sparm_dma_buf;
	node->rnode.indicator = UINT32_MAX;

	node->sport = sport;
	ocs_sport_lock(sport);
		node->ocs = ocs;
		node->init = init;
		node->targ = targ;

		rc = ocs_hal_node_alloc(&ocs->hal, &node->rnode, port_id, sport);
		if (rc) {
			ocs_log_err(ocs, "ocs_hal_node_alloc failed: %d\n", rc);
			ocs_sport_unlock(sport);

			/* Return back to pool. */
			ocs_device_lock(ocs);
			ocs_list_add_tail(&xport->nodes_free_list, node);
			ocs_device_unlock(ocs);

			return NULL;
		}
		ocs_list_add_tail(&sport->node_list, node);

		ocs_node_lock_init(node);
		ocs_lock_init(ocs, &node->pend_frames_lock, "pend_frames_lock[%d]", node->instance_index);
		ocs_list_init(&node->pend_frames, ocs_hal_sequence_t, link);
		ocs_lock_init(ocs, &node->active_ios_lock, "active_ios[%d]", node->instance_index);
		ocs_list_init(&node->active_ios, ocs_io_t, link);
		ocs_list_init(&node->els_io_pend_list, ocs_io_t, link);
		ocs_list_init(&node->els_io_active_list, ocs_io_t, link);
		ocs_scsi_io_alloc_enable(node);

		/* zero the service parameters */
		ocs_memset(node->sparm_dma_buf.virt, 0, node->sparm_dma_buf.size);

		node->rnode.node = node;
		node->sm.app = node;
		node->evtdepth = 0;

		ocs_node_update_display_name(node);

		spv_set(sport->lookup, port_id, node);
	ocs_sport_unlock(sport);
	node->mgmt_functions = &node_mgmt_functions;

	return node;
}

/**
 * @ingroup node_alloc
 * @brief free a node structure
 *
 * The node structure given by 'node' is free'd
 *
 * @param node the node to free
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int32_t
ocs_node_free(ocs_node_t *node)
{
	ocs_sport_t *sport;
	ocs_t *ocs;
	ocs_xport_t *xport;
	ocs_hal_rtn_e rc = 0;
	ocs_node_t *ns = NULL;
	int post_all_free = FALSE;

	ocs_assert(node, -1);
	ocs_assert(node->sport, -1);
	ocs_assert(node->ocs, -1);
	sport = node->sport;
	ocs_assert(sport, -1);
	ocs = node->ocs;
	ocs_assert(ocs->xport, -1);
	xport = ocs->xport;

	node_printf(node, "Free'd\n");

	if(node->refound) {
		/*
		 * Save the name server node. We will send fake RSCN event at
		 * the end to handle ignored RSCN event during node deletion
		 */
		ns = ocs_node_find(node->sport, FC_ADDR_NAMESERVER);
	}

	/* Remove from node list */
	ocs_sport_lock(sport);
		ocs_list_remove(&sport->node_list, node);

		/* Free HAL resources */
		if (OCS_HAL_RTN_IS_ERROR((rc = ocs_hal_node_free_resources(&ocs->hal, &node->rnode)))) {
			ocs_log_test(ocs, "ocs_hal_node_free failed: %d\n", rc);
			rc = -1;
		}

		/* if the gidpt_delay_timer is still running, then delete it */
		if (ocs_timer_pending(&node->gidpt_delay_timer)) {
			ocs_del_timer(&node->gidpt_delay_timer);
		}

		/* remove entry from sparse vector list */
		if (sport->lookup == NULL) {
			ocs_log_test(node->ocs, "sport lookup is NULL\n");
			ocs_sport_unlock(sport);
			return -1;
		}

		spv_set(sport->lookup, node->rnode.fc_id, NULL);

		/*
		 * If the node_list is empty, then post a ALL_CHILD_NODES_FREE event to the sport,
		 * after the lock is released.  The sport may be free'd as a result of the event.
		 */
		if (ocs_list_empty(&sport->node_list)) {
			post_all_free = TRUE;
		}

	ocs_sport_unlock(sport);

	if (post_all_free) {
		ocs_sm_post_event(&sport->sm, OCS_EVT_ALL_CHILD_NODES_FREE, NULL);
	}

	node->sport = NULL;
	node->sm.current_state = NULL;

	ocs_node_lock_free(node);
	ocs_lock_free(&node->pend_frames_lock);
	ocs_lock_free(&node->active_ios_lock);

	/* return to free list */
	ocs_device_lock(ocs);
		ocs_list_add_tail(&xport->nodes_free_list, node);
	ocs_device_unlock(ocs);

	if(ns != NULL) {
		/* sending fake RSCN event to name server node */
		ocs_node_post_event(ns, OCS_EVT_RSCN_RCVD, NULL);
	}

	return rc;
}

/**
 * @brief Notification from base driver that node is in force-free path.
 *
 * @par Description Node is forcefully going away.  Cleanup any resources associated with it.
 *
 * @param node Pointer to node being free'd.
 *
 * @return None.
 */
void ocs_notify_node_force_free(ocs_node_t *node)
{
	/* shutdown sm processing */
	ocs_sm_disable(&node->sm);
	ocs_strncpy(node->prev_state_name, node->current_state_name, sizeof(node->prev_state_name));
	ocs_strncpy(node->current_state_name, "disabled", sizeof(node->current_state_name));

#if !defined(OCSU_FC_RAMD) && !defined(OCSU_FC_WORKLOAD) && !defined(OCS_USPACE_SPDK)
	ocs_scsi_tgt_notify_node_force_free(node);
#endif
}

/**
 * @brief free memory resources of a node object
 *
 * The node object's child objects are freed after which the
 * node object is freed.
 *
 * @param node pointer to a node object
 *
 * @return none
 */

void
ocs_node_force_free(ocs_node_t *node)
{
	ocs_io_t *io;
	ocs_io_t *next;
	ocs_io_t *els;
	ocs_io_t *els_next;

	ocs_node_lock(node);

	ocs_lock(&node->active_ios_lock);
		ocs_list_foreach_safe(&node->active_ios, io, next) {
			ocs_list_remove(&io->node->active_ios, io);
			ocs_io_free(node->ocs, io);
		}
	ocs_unlock(&node->active_ios_lock);

	/* free all pending ELS IOs */
	ocs_lock(&node->active_ios_lock);
		ocs_list_foreach_safe(&node->els_io_pend_list, els, els_next) {
			/* can't call ocs_els_io_free() because lock is held; cleanup manually */
			ocs_list_remove(&node->els_io_pend_list, els);

			ocs_dma_free(els->ocs, &els->els_info->els_rsp);
			ocs_dma_free(els->ocs, &els->els_info->els_req);

			ocs_io_free(node->ocs, els);
		}
	ocs_unlock(&node->active_ios_lock);

	/* free all active ELS IOs */
	ocs_lock(&node->active_ios_lock);
		ocs_list_foreach_safe(&node->els_io_active_list, els, els_next) {
			/* Kill ocs_els_delay_timer_cb(); if scheduled */
			if (ocs_timer_pending(&els->delay_timer)) {
				ocs_del_timer(&els->delay_timer);
			}

			/* can't call ocs_els_io_free() because lock is held; cleanup manually */
			ocs_list_remove(&node->els_io_active_list, els);

			ocs_dma_free(els->ocs, &els->els_info->els_rsp);
			ocs_dma_free(els->ocs, &els->els_info->els_req);

			ocs_io_free(node->ocs, els);
		}
	ocs_unlock(&node->active_ios_lock);

	ocs_node_unlock(node);

	/* manually purge pending frames (if any) */
	ocs_node_purge_pending(node);

	ocs_node_free(node);
}

/**
 * @ingroup node_common
 * @brief Perform HAL call to attach a remote node
 *
 * @param node pointer to node object
 *
 * @return 0 on success, non-zero otherwise
 */
int32_t
ocs_node_attach(ocs_node_t *node)
{
	int32_t rc = 0;
	ocs_sport_t *sport = node->sport;
	ocs_domain_t *domain = sport->domain;
	ocs_t *ocs = node->ocs;

	if (!domain->attached) {
		ocs_log_test(ocs, "Warning: ocs_node_attach with unattached domain\n");
		return -1;
	}

	/* Update node->wwpn/wwnn */
	ocs_node_build_eui_name(node->wwpn, sizeof(node->wwpn), ocs_node_get_wwpn(node));
	ocs_node_build_eui_name(node->wwnn, sizeof(node->wwnn), ocs_node_get_wwnn(node));

	if (ocs->enable_hlm) {
		ocs_node_group_init(node);
	}

	ocs_dma_copy_in(&node->sparm_dma_buf, node->service_params+4, sizeof(node->service_params)-4);

	/* take lock to protect node->rnode.attached */
	ocs_node_lock(node);
		rc = ocs_hal_node_attach(&ocs->hal, &node->rnode, &node->sparm_dma_buf);
		if (OCS_HAL_RTN_IS_ERROR(rc))
			ocs_log_test(ocs, "ocs_hal_node_attach failed: %d\n", rc);
	ocs_node_unlock(node);

	return rc;
}

/**
 * @ingroup node_common
 * @brief Generate text for a node's fc_id
 *
 * The text for a nodes fc_id is generated, either as a well known name, or a 6 digit
 * hex value.
 *
 * @param fc_id fc_id
 * @param buffer text buffer
 * @param buffer_length text buffer length in bytes
 *
 * @return none
 */

void
ocs_node_fcid_display(uint32_t fc_id, char *buffer, uint32_t buffer_length)
{
	switch (fc_id) {
	case FC_ADDR_FABRIC:
		ocs_snprintf(buffer, buffer_length, "fabric");
		break;
	case FC_ADDR_CONTROLLER:
		ocs_snprintf(buffer, buffer_length, "fabctl");
		break;
	case FC_ADDR_NAMESERVER:
		ocs_snprintf(buffer, buffer_length, "nserve");
		break;
	default:
		if (FC_ADDR_IS_DOMAIN_CTRL(fc_id)) {
			ocs_snprintf(buffer, buffer_length, "dctl%02x",
				FC_ADDR_GET_DOMAIN_CTRL(fc_id));
		} else {
			ocs_snprintf(buffer, buffer_length, "%06x", fc_id);
		}
		break;
	}

}

/**
 * @brief update the node's display name
 *
 * The node's display name is updated, sometimes needed because the sport part
 * is updated after the node is allocated.
 *
 * @param node pointer to the node object
 *
 * @return none
 */

void
ocs_node_update_display_name(ocs_node_t *node)
{
	uint32_t port_id = node->rnode.fc_id;
	int32_t count;
	ocs_sport_t *sport = node->sport;
	char portid_display[16];

	ocs_assert(sport);

	ocs_node_fcid_display(port_id, portid_display, sizeof(portid_display));

	count = ocs_snprintf(node->display_name, sizeof(node->display_name), "%s.", sport->display_name);
	if (count < 0) {
		ocs_log_err(node->ocs, "snprintf failed for display_name\n");
		return;
	}

	if (ocs_snprintf(node->display_name + count, sizeof(portid_display), "%s", portid_display) < 0) {
		ocs_log_err(node->ocs, "snprintf failed for portid_display\n");
		return;
	}
}

/**
 * @brief cleans up an XRI for the pending link services accept by aborting the
 *         XRI if required.
 *
 * <h3 class="desc">Description</h3>
 * This function is called when the LS accept is not sent.
 *
 * @param node Node for which should be cleaned up
 */

void
ocs_node_send_ls_io_cleanup(ocs_node_t *node)
{
	ocs_t *ocs = node->ocs;

	if (node->send_ls_rsp != OCS_NODE_SEND_LS_RSP_NONE) {
		ocs_assert(node->ls_rsp_io);
		ocs_log_debug(ocs, "[%s] cleaning up LS_ACC oxid=0x%x\n",
			node->display_name, node->ls_rsp_oxid);

		node->ls_rsp_io->hio = NULL;
		ocs_els_io_free(node->ls_rsp_io);
		node->send_ls_rsp = OCS_NODE_SEND_LS_RSP_NONE;
		node->ls_rsp_io = NULL;
	}
}

static int
ocs_node_process_plogi_frame(ocs_node_t *node)
{
	ocs_hal_sequence_t *frame = NULL;
	fc_header_t *hdr = NULL;
	uint8_t *buf = NULL;
	ocs_io_t *io = NULL;

	for (;;) {
		frame = ocs_frame_next(&node->pend_frames, &node->pend_frames_lock);
		if (frame == NULL)
			break;

		hdr = frame->header.data;
		buf = frame->payload.data;
		if ((fc_be24toh(hdr->f_ctl) & FC_FCTL_END_SEQUENCE) && (hdr->r_ctl == FC_RCTL_ELS)) {
			if (buf[0] == FC_ELS_CMD_PLOGI) {
				int rc = 1;

				ocs_scsi_io_alloc_enable(node);
				io = ocs_els_io_alloc(node, sizeof(fc_plogi_payload_t), OCS_ELS_ROLE_RESPONDER);
				if (io) {
					/* Save plogi parameters */
					ocs_node_save_sparms(node, buf);
					ocs_send_ls_rsp_after_attach(io, hdr, OCS_NODE_SEND_LS_RSP_PLOGI);
					rc = 0;
				}
				ocs_hal_sequence_free(&node->ocs->hal, frame);
				return rc;
			}
		}
		ocs_hal_sequence_free(&node->ocs->hal, frame);
	}

	return 0;
}

/**
 * @ingroup node_common
 * @brief state: shutdown a node
 *
 * A node is shutdown,
 *
 * @param ctx remote node sm context
 * @param evt event to process
 * @param arg per event optional argument
 *
 * @return returns NULL
 *
 * @note
 */

void *
__ocs_node_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	int32_t rc;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER: {
		uint8_t shutdown_reason = node->shutdown_reason;

		ocs_node_hold_frames(node);
		ocs_assert(ocs_node_active_ios_empty(node), NULL);
		ocs_assert(ocs_els_io_list_empty(node, &node->els_io_active_list), NULL);

		/* by default, we will be freeing node after we unwind */
		node->req_free = 1;

		/* If sport is in shutdown state, shutdown the node completely */
		if (node->sport->shutting_down) {
			node_printf(node, "Received IMPLICIT LOGO during sport shutdown..\n");
			shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		}

		switch (shutdown_reason) {
		case OCS_NODE_SHUTDOWN_IMPLICIT_LOGO:
restart_node_attach:
			//sm: if shutdown reason is implicit logout / ocs_node_attach
			/* Node shutdown b/c of PLOGI received when node already
			 * logged in. We have PLOGI service parameters, so submit
			 * node attach; we won't be freeing this node
			 */

			/* currently, only case for implicit logo is PLOGI recvd. Thus,
			 * node's ELS IO pending list won't be empty (PLOGI will be on it)
			 */
			ocs_assert(node->send_ls_rsp == OCS_NODE_SEND_LS_RSP_PLOGI, NULL);
			node_printf(node, "Shutdown reason: implicit logout, re-authenticate\n");

			/* Check and allocate an RPI before re-attaching the node */
			if (node->rnode.indicator == UINT32_MAX) {
				if (sli_resource_alloc(&ocs->hal.sli, SLI_RSRC_FCOE_RPI,
				    &node->rnode.indicator, &node->rnode.index)) {
					ocs_log_err(node->ocs, "FCOE_RPI allocation failure addr=%#x\n", node->rnode.fc_id);
				}
			}

			ocs_scsi_io_alloc_enable(node);

			/* Re-attach node with the same HAL node resources */
			node->req_free = 0;
			node->unreg_rpi = FALSE;
			node->mark_for_deletion = FALSE;

			node->prli_rsp_pend = 0;
			node->ini_fct_prli_success = 0;
			node->tgt_fct_prli_success = 0;

			rc = ocs_node_attach(node);
			ocs_node_transition(node, __ocs_d_wait_node_attach, NULL);
			if (rc == OCS_HAL_RTN_SUCCESS_SYNC) {
				ocs_node_post_event(node, OCS_EVT_NODE_ATTACH_OK, NULL);
			}
			break;
		case OCS_NODE_SHUTDOWN_EXPLICIT_LOGO: {
			int8_t pend_frames_empty;

			/* Notify target driver about vport LOGOUT */
			if (node->sport->is_vport && (node->rnode.fc_id == FC_ADDR_FABRIC))
				ocs_vport_logout_notify(node);

			/* cleanup any pending LS_ACC ELSs */
			ocs_node_send_ls_io_cleanup(node);
			ocs_assert(ocs_els_io_list_empty(node, &node->els_io_pend_list), NULL);

			ocs_lock(&node->pend_frames_lock);
				pend_frames_empty = ocs_list_empty(&node->pend_frames);
			ocs_unlock(&node->pend_frames_lock);

			/* there are two scenarios where we want to keep this node alive:
			 * 1. there are pending frames that need to be processed or
			 * 2. we're an initiator and the remote node is a target and we
			 *    need to re-authenticate
			 */
			node_printf(node, "Shutdown: explicit logo pend=%d sport.ini=%d node.tgt=%d node.nvme_tgt=%d\n",
				    !pend_frames_empty, node->sport->enable_ini, node->targ, node->nvme_tgt);

			if ((!pend_frames_empty) || (node->sport->enable_ini && (node->targ || node->nvme_tgt))) {
				uint8_t send_plogi = FALSE;
				if (node->sport->enable_ini && (node->targ || node->nvme_tgt)) {
					/* we're an initiator and node shutting down is a target; we'll
					 * need to re-authenticate in initial state
					 */
					send_plogi = TRUE;
				}

				/* Check and allocate an RPI before moving the node to initial state */
				if (node->rnode.indicator == UINT32_MAX) {
					if (sli_resource_alloc(&ocs->hal.sli, SLI_RSRC_FCOE_RPI,
					    &node->rnode.indicator, &node->rnode.index)) {
						ocs_log_err(node->ocs, "FCOE_RPI allocation failure addr=%#x\n", node->rnode.fc_id);
					}
				}

				/* transition to __ocs_d_init (will retain HAL node resources) */
				ocs_scsi_io_alloc_enable(node);
				node->req_free = 0;
				node->unreg_rpi = FALSE;
				node->mark_for_deletion = FALSE;

				node->prli_rsp_pend = 0;
				node->ini_fct_prli_success = 0;
				node->tgt_fct_prli_success = 0;

				/* either pending frames exist, or we're re-authenticating with PLOGI
				 * (or both); in either case, return to initial state
				 */
				ocs_node_init_device(node, send_plogi);

			}
			/* else: let node shutdown occur */
			break;
		}
		case OCS_NODE_SHUTDOWN_DEFAULT:
		default:
			/*
			 * If there is a PLOGI in the pending frames restart node attach
			 * with new service paramters, and send ACC
			 */
			if (!node->sport->shutting_down && ocs_node_process_plogi_frame(node))
				goto restart_node_attach;

			/* Notify target driver about vport LOGOUT */
			if (node->sport->is_vport && (node->rnode.fc_id == FC_ADDR_FABRIC))
				ocs_vport_logout_notify(node);

			/*
			 * shutdown due to link down, node going away (xport event) or
			 * sport shutdown, purge pending and proceed to cleanup node
			 */

			/* cleanup any pending LS_ACC ELSs */
			ocs_node_send_ls_io_cleanup(node);
			ocs_assert(ocs_els_io_list_empty(node, &node->els_io_pend_list), NULL);

			node_printf(node, "Shutdown reason: default, purge pending\n");
			ocs_node_purge_pending(node);
			break;
		}

		break;
	}
	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	default:
		__ocs_node_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup common_node
 * @brief Checks to see if ELS's have been quiesced
 *
 * Check if ELS's have been quiesced. If so, transition to the
 * next state in the shutdown process.
 *
 * @param node Node for which ELS's are checked
 *
 * @return Returns 1 if ELS's have been quiesced, 0 otherwise.
 */
static int
ocs_node_check_els_quiesced(ocs_node_t *node)
{
	ocs_assert(node, -1);

	/* check to see if ELS requests, completions are quiesced */
	if ((node->els_req_cnt == 0) && (node->els_cmpl_cnt == 0) &&
	    ocs_els_io_list_empty(node, &node->els_io_active_list)) {
		if (!node->attached) {
			/* hal node detach already completed, proceed */
			node_printf(node, "HAL node not attached\n");
			ocs_node_transition(node, __ocs_node_wait_ios_shutdown, NULL);
		} else {
			/* hal node detach hasn't completed, transition and wait */
			node_printf(node, "HAL node still attached\n");
			ocs_node_transition(node, __ocs_node_wait_node_free, NULL);
		}
		return 1;
	}
	return 0;
}

/**
 * @ingroup common_node
 * @brief Initiate node IO cleanup.
 *
 * Note: this function must be called with a non-attached node
 * or a node for which the node detach (ocs_hal_node_detach())
 * has already been initiated.
 *
 * @param node Node for which shutdown is initiated
 *
 * @return Returns None.
 */

void
ocs_node_initiate_cleanup(ocs_node_t *node)
{
	ocs_io_t *els;
	ocs_io_t *els_next;
	ocs_t *ocs;
	ocs_assert(node);
	ocs = node->ocs;

	/* cleanup auth FSM of the node */
	ocs_node_auth_stop(node);

	/* first cleanup ELS's that are pending (not yet active) */
	ocs_lock(&node->active_ios_lock);
		ocs_list_foreach_safe(&node->els_io_pend_list, els, els_next) {

			/* skip the ELS IO for which a response will be sent after shutdown */
			if ((node->send_ls_rsp != OCS_NODE_SEND_LS_RSP_NONE) && (els == node->ls_rsp_io))
				continue;

			/* can't call ocs_els_io_free() because lock is held; cleanup manually */
			node_printf(node, "Freeing pending els %s\n", els->display_name);
			ocs_list_remove(&node->els_io_pend_list, els);

			ocs_dma_free(els->ocs, &els->els_info->els_rsp);
			ocs_dma_free(els->ocs, &els->els_info->els_req);

			ocs_io_free(node->ocs, els);
		}
	ocs_unlock(&node->active_ios_lock);

	if (node->ls_rsp_io && node->ls_rsp_io->hio != NULL) {
		/*
		 * if there's an IO that will result in an LS_ACC after
		 * shutdown and its HAL IO is non-NULL, it better be an
		 * implicit logout in vanilla sequence coalescing. In this
		 * case, force the LS_ACC to go out on another XRI (hio)
		 * since the previous will have been aborted by the UNREG_RPI
		 */
		ocs_assert(node->shutdown_reason == OCS_NODE_SHUTDOWN_IMPLICIT_LOGO);
		ocs_assert(node->send_ls_rsp == OCS_NODE_SEND_LS_RSP_PLOGI);
		node_printf(node, "invalidating ls_rsp_io due to implicit logo\n");

		/* No need to abort because the unreg_rpi takes care of it, just free */
		ocs_hal_io_free(&ocs->hal, node->ls_rsp_io->hio);

		/* NULL out hio to force the LS_ACC to grab a new XRI */
		node->ls_rsp_io->hio = NULL;
	}

	/*
	 * if ELS's have already been quiesced, will move to next state
	 * if ELS's have not been quiesced, abort them
	 */
	if (ocs_node_check_els_quiesced(node) == 0) {
		/*
		 * Abort all ELS's since ELS's won't be aborted by HAL
		 * node free.
		 */
		ocs_node_abort_all_els(node);
		ocs_node_transition(node, __ocs_node_wait_els_shutdown, NULL);
	}
}

/**
 * @ingroup node_common
 * @brief Node state machine: Wait for all ELSs to complete.
 *
 * <h3 class="desc">Description</h3>
 * State waits for all ELSs to complete after aborting all
 * outstanding .
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_node_wait_els_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	uint8_t check_quiesce = FALSE;
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {

	case OCS_EVT_ENTER: {
		ocs_node_hold_frames(node);
		if (ocs_els_io_list_empty(node, &node->els_io_active_list)) {
			node_printf(node, "All ELS IOs complete\n");
			check_quiesce = TRUE;
		}
		break;
	}
	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_SRRS_ELS_REQ_OK:
	case OCS_EVT_SRRS_ELS_REQ_FAIL:
	case OCS_EVT_SRRS_ELS_REQ_RJT:
	case OCS_EVT_ELS_REQ_ABORTED:
		ocs_assert(node->els_req_cnt, NULL);
		node->els_req_cnt--;
		check_quiesce = TRUE;
		break;

	case OCS_EVT_SRRS_ELS_CMPL_OK:
	case OCS_EVT_SRRS_ELS_CMPL_FAIL:
		ocs_assert(node->els_cmpl_cnt, NULL);
		node->els_cmpl_cnt--;
		check_quiesce = TRUE;
		break;

	case OCS_EVT_ALL_CHILD_NODES_FREE:
		/* all ELS IO's complete */
		node_printf(node, "All ELS IOs complete\n");
		ocs_assert(ocs_els_io_list_empty(node, &node->els_io_active_list), NULL);
		check_quiesce = TRUE;
		break;

	case OCS_EVT_NODE_ACTIVE_IO_LIST_EMPTY:
		break;

	case OCS_EVT_DOMAIN_ATTACH_OK:
		/* don't care about domain_attach_ok */
		break;

	/* ignore shutdown events as we're already in shutdown path */
	case OCS_EVT_SHUTDOWN:
		/* have default shutdown event take precedence */
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		/* fall through */
	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		node_printf(node, "%s received\n", ocs_sm_event_name(evt));
		break;

	default:
		__ocs_node_common(__func__, ctx, evt, arg);
		return NULL;
	}

	if (check_quiesce) {
		ocs_node_check_els_quiesced(node);
	}

	return NULL;
}

/**
 * @ingroup node_command
 * @brief Node state machine: Wait for a HAL node free event to
 * complete.
 *
 * <h3 class="desc">Description</h3>
 * State waits for the node free event to be received from the HAL.
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return Returns NULL.
 */

void *
__ocs_node_wait_node_free(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
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

	case OCS_EVT_NODE_FREE_OK:
		/* node is officially no longer attached */
		node->attached = FALSE;
		ocs_node_transition(node, __ocs_node_wait_ios_shutdown, NULL);
		break;

	case OCS_EVT_ALL_CHILD_NODES_FREE:
	case OCS_EVT_NODE_ACTIVE_IO_LIST_EMPTY:
		/* As IOs and ELS IO's complete we expect to get these events */
		break;

	case OCS_EVT_DOMAIN_ATTACH_OK:
		/* don't care about domain_attach_ok */
		break;

	/* ignore shutdown events as we're already in shutdown path */
	case OCS_EVT_SHUTDOWN:
		/* have default shutdown event take precedence */
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		/* Fall through */
	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		node_printf(node, "%s received\n", ocs_sm_event_name(evt));
		break;
	default:
		__ocs_node_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup node_common
 * @brief state: initiate node shutdown
 *
 * State is entered when a node receives a shutdown event, and it's waiting
 * for all the active IOs and ELS IOs associated with the node to complete.
 *
 * @param ctx remote node sm context
 * @param evt event to process
 * @param arg per event optional argument
 *
 * @return returns NULL
 */

void *
__ocs_node_wait_ios_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		ocs_node_hold_frames(node);
		/* Fall Through */

	case OCS_EVT_NODE_ACTIVE_IO_LIST_EMPTY:
	case OCS_EVT_ALL_CHILD_NODES_FREE: {
		if (ocs_node_active_ios_empty(node) &&
		    ocs_els_io_list_empty(node, &node->els_io_active_list)) {
			ocs_node_transition(node, __ocs_node_shutdown, NULL);
		}
		break;
	}

	case OCS_EVT_EXIT:
		ocs_node_accept_frames(node);
		break;

	case OCS_EVT_SRRS_ELS_REQ_FAIL:
		/* Can happen as ELS IO IO's complete */
		ocs_assert(node->els_req_cnt, NULL);
		node->els_req_cnt--;
		break;

	/* ignore shutdown events as we're already in shutdown path */
	case OCS_EVT_SHUTDOWN:
		/* have default shutdown event take precedence */
		node->shutdown_reason = OCS_NODE_SHUTDOWN_DEFAULT;
		/* fall through */
	case OCS_EVT_SHUTDOWN_EXPLICIT_LOGO:
	case OCS_EVT_SHUTDOWN_IMPLICIT_LOGO:
		ocs_log_debug(ocs, "[%s] %-20s\n", node->display_name, ocs_sm_event_name(evt));
		break;
	case OCS_EVT_DOMAIN_ATTACH_OK:
		/* don't care about domain_attach_ok */
		break;
	default:
		__ocs_node_common(__func__, ctx, evt, arg);
		return NULL;
	}

	return NULL;
}

/**
 * @ingroup node_common
 * @brief state: common node event handler
 *
 * Handle common/shared node events
 *
 * @param funcname calling function's name
 * @param ctx remote node sm context
 * @param evt event to process
 * @param arg per event optional argument
 *
 * @return returns NULL
 */

void *
__ocs_node_common(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_t *node = NULL;
	ocs_t *ocs = NULL;
	ocs_node_cb_t *cbdata = arg;
	ocs_assert(ctx, NULL);
	ocs_assert(ctx->app, NULL);
	node = ctx->app;
	ocs_assert(node->ocs, NULL);
	ocs = node->ocs;

	switch(evt) {
	case OCS_EVT_ENTER:
	case OCS_EVT_REENTER:
	case OCS_EVT_EXIT:
	case OCS_EVT_SPORT_TOPOLOGY_NOTIFY:
	case OCS_EVT_NODE_MISSING:
	case OCS_EVT_FCP_CMD_RCVD:
	case OCS_EVT_NVME_CMD_RCVD:
	case OCS_EVT_RSCN_RCVD:
		break;

	case OCS_EVT_NODE_REFOUND:
		node->refound = 1;
		break;

	/* node->attached must be set appropriately for all node attach/detach events */
	case OCS_EVT_NODE_ATTACH_OK:
		node->attached = TRUE;
		break;

	case OCS_EVT_NODE_FREE_OK:
	case OCS_EVT_NODE_ATTACH_FAIL:
		node->attached = FALSE;
		break;

	/* handle any ELS completions that other states either didn't care about
	 * or forgot about
	 */
	case OCS_EVT_SRRS_ELS_CMPL_OK:
	case OCS_EVT_SRRS_ELS_CMPL_FAIL:
		ocs_assert(node->els_cmpl_cnt, NULL);
		node->els_cmpl_cnt--;
		break;

	/* handle any ELS request completions that other states either didn't care about
	 * or forgot about
	 */
	case OCS_EVT_SRRS_ELS_REQ_OK:
	case OCS_EVT_SRRS_ELS_REQ_FAIL:
	case OCS_EVT_SRRS_ELS_REQ_RJT:
	case OCS_EVT_ELS_REQ_ABORTED:
		ocs_assert(node->els_req_cnt, NULL);
		node->els_req_cnt--;
		break;

	case OCS_EVT_ELS_RCVD: {
		fc_header_t *hdr = cbdata->header;

		/* Unsupported ELS was received, send LS_RJT, command not supported */
		ocs_log_debug(ocs, "[%s] (%s) ELS x%02x, LS_RJT not supported\n",
			      node->display_name, funcname, ((uint8_t*)cbdata->payload)[0]);
		ocs_send_ls_rjt(cbdata->io, ocs_be16toh(hdr->ox_id),
			FC_REASON_COMMAND_NOT_SUPPORTED, FC_EXPL_NO_ADDITIONAL, 0,
			NULL, NULL);
		break;
	}

	case OCS_EVT_AUTH_RCVD: {
		fc_header_t *hdr = cbdata->header;
		ocs_send_ls_rjt(cbdata->io, ocs_be16toh(hdr->ox_id),
				FC_REASON_UNABLE_TO_PERFORM,
				FC_EXPL_NPORT_LOGIN_REQUIRED, 0, NULL, NULL);
		break;
	}
	case OCS_EVT_PLOGI_RCVD:
	case OCS_EVT_FLOGI_RCVD:
	case OCS_EVT_LOGO_RCVD:
	case OCS_EVT_PRLI_RCVD:
	case OCS_EVT_PRLO_RCVD:
	case OCS_EVT_PDISC_RCVD:
	case OCS_EVT_FDISC_RCVD:
	case OCS_EVT_ADISC_RCVD:
	case OCS_EVT_SCR_RCVD: {
		fc_header_t *hdr = cbdata->header;
		//sm: / send ELS_RJT
		ocs_log_debug(ocs, "[%s] (%s) %s sending ELS_RJT\n",
			      node->display_name, funcname, ocs_sm_event_name(evt));
		/* if we didn't catch this in a state, send generic LS_RJT */
		ocs_send_ls_rjt(cbdata->io, ocs_be16toh(hdr->ox_id),
			FC_REASON_UNABLE_TO_PERFORM, FC_EXPL_NO_ADDITIONAL, 0,
			NULL, NULL);

		break;
	}

	case OCS_EVT_RDP_RCVD: {
		ocs_send_rdp_resp(node, arg, OCS_RDP_RJT);
		break;
	}
	case OCS_EVT_GID_FT_RCVD:
	case OCS_EVT_GID_PT_RCVD:
	case OCS_EVT_RFT_ID_RCVD:
	case OCS_EVT_RFF_ID_RCVD:
	case OCS_EVT_CT_LOOPBACK_RCVD_NO_IO: {
		fc_header_t *hdr = cbdata->header;
		ocs_log_debug(ocs, "[%s] (%s) %s sending CT_REJECT\n",
			      node->display_name, funcname, ocs_sm_event_name(evt));
		ocs_send_ct_rsp(cbdata->io, hdr->ox_id, cbdata->payload, FCCT_HDR_CMDRSP_REJECT, FCCT_COMMAND_NOT_SUPPORTED, 0);
		break;
	}
	case OCS_EVT_CT_LOOPBACK_RCVD: {
		fc_header_t *hdr = cbdata->header;
		ocs_log_debug(ocs, "[%s] (%s) %s sending CT_ACCEPT\n",
			      node->display_name, funcname, ocs_sm_event_name(evt));
		ocs_send_ct_rsp(cbdata->io, hdr->ox_id, cbdata->payload, FCCT_HDR_CMDRSP_ACCEPT, 0, 0);
		break;
	}
	case OCS_EVT_ABTS_RCVD: {
		fc_header_t *hdr = cbdata->header;
		ocs_log_debug(ocs, "[%s] (%s) %s sending BA_ACC\n",
			      node->display_name, funcname, ocs_sm_event_name(evt));

		//sm: / send BA_ACC
		ocs_bls_send_acc_hdr(cbdata->io, hdr);
		break;
	}
	case OCS_EVT_NODE_LAST_ACTIVE_IO: {
		int send_empty_event;

		ocs_lock(&node->active_ios_lock);
			ocs_list_remove(&node->active_ios, cbdata->io);
			send_empty_event = (!node->io_alloc_enabled) && ocs_list_empty(&node->active_ios);
		ocs_unlock(&node->active_ios_lock);

		if (send_empty_event)
			ocs_node_post_event(node, OCS_EVT_NODE_ACTIVE_IO_LIST_EMPTY, NULL);
		break;
	}
	case OCS_EVT_TOW_DATA_RCVD: {
		ocs_node_cb_t *cbdata = arg;
		ocs_io_t *io = cbdata->io;
		/*
		 * Expected to get this event when node is being shutdown.
		 * Free the corresponding TOW IO.
		 */
		ocs_scsi_io_free(io);

		break;
	}
	default:
		ocs_log_test(node->ocs, "[%s] %-20s %-20s not handled\n", node->display_name, funcname,
			ocs_sm_event_name(evt));
		break;
	}
	return NULL;
}

/**
 * @ingroup node_common
 * @brief save node service parameters
 *
 * Service parameters are copyed into the node structure
 *
 * @param node pointer to node structure
 * @param payload pointer to service paramters to save
 *
 * @return none
 */

void
ocs_node_save_sparms(ocs_node_t *node, void *payload)
{
	fc_plogi_payload_t *plogi;

	ocs_memcpy(node->service_params, payload, sizeof(node->service_params));

	/* Check vendor specific info for suppress response feature */
	plogi = (fc_plogi_payload_t *)payload;
	node->suppress_rsp = 0;
	if ((ocs_be32toh(plogi->common_service_parameters[1]) & FC_PLOGI_CSP_W1_VALID_VENDOR_VER_LEVEL) &&
	    (ocs_be32toh(plogi->vendor_version_level[0]) == OCS_FC_PLOGI_VENDOR_VERSION_EMLX_ID))
		node->suppress_rsp = ocs_be32toh(plogi->vendor_version_level[1]);
}

/**
 * @ingroup node_common
 * @brief Post event to node state machine context
 *
 * This is used by the node state machine code to post events to the nodes.  Upon
 * completion of the event posting, if the nesting depth is zero and we're not holding
 * inbound frames, then the pending frames are processed.
 *
 * @param node pointer to node
 * @param evt event to post
 * @param arg event posting argument
 *
 * @return none
 */

void
ocs_node_post_event(ocs_node_t *node, ocs_sm_event_t evt, void *arg)
{
	int free_node = FALSE;
	ocs_assert(node);

	ocs_node_lock(node);
		node->evtdepth ++;

		ocs_sm_post_event(&node->sm, evt, arg);

		/* If our event call depth is one and we're not holding frames
		 * then we can dispatch any pending frames.   We don't want to allow
		 * the ocs_process_node_pending() call to recurse.
		 */
		if (!node->hold_frames && (node->evtdepth == 1)) {
			ocs_process_node_pending(node);
		}
		node->evtdepth --;

		/* Free the node object if so requested, and we're at an event
		 * call depth of zero
		 */
		if ((node->evtdepth == 0) && node->req_free) {
			free_node = TRUE;
		}
	ocs_node_unlock(node);

	if (free_node) {
		ocs_node_free(node);
	}

	return;
}

/**
 * @ingroup node_common
 * @brief transition state of a node
 *
 * The node's state is transitioned to the requested state.  Entry/Exit
 * events are posted as needed.
 *
 * @param node pointer to node
 * @param state state to transition to
 * @param data transition data
 *
 * @return none
 */

void
ocs_node_transition(ocs_node_t *node, ocs_sm_function_t state, void *data)
{
	ocs_sm_ctx_t *ctx = &node->sm;

	ocs_node_lock(node);
		if (ctx->current_state == state) {
			ocs_node_post_event(node, OCS_EVT_REENTER, data);
		} else {
			ocs_node_post_event(node, OCS_EVT_EXIT, data);
			ctx->current_state = state;
			ocs_node_post_event(node, OCS_EVT_ENTER, data);
		}
	ocs_node_unlock(node);
}

/**
 * @ingroup node_common
 * @brief build EUI formatted WWN
 *
 * Build a WWN given the somewhat transport agnostic iScsi naming specification, for FC
 * use the eui. format, an ascii string such as: "eui.10000000C9A19501"
 *
 * @param buffer buffer to place formatted name into
 * @param buffer_len length in bytes of the buffer
 * @param eui_name cpu endian 64 bit WWN value
 *
 * @return none
 */

void
ocs_node_build_eui_name(char *buffer, uint32_t buffer_len, uint64_t eui_name)
{
	ocs_memset(buffer, 0, buffer_len);

	ocs_snprintf(buffer, buffer_len, "eui.%016" PRIX64, eui_name);
}

/**
 * @ingroup node_common
 * @brief return nodes' WWPN as a uint64_t
 *
 * The WWPN is computed from service parameters and returned as a uint64_t
 *
 * @param node pointer to node structure
 *
 * @return WWPN
 *
 */

uint64_t
ocs_node_get_wwpn(ocs_node_t *node)
{
	fc_plogi_payload_t *sp = (fc_plogi_payload_t*) node->service_params;

	return (((uint64_t)ocs_be32toh(sp->port_name_hi) << 32ll) | (ocs_be32toh(sp->port_name_lo)));
}

/**
 * @ingroup node_common
 * @brief return nodes' WWNN as a uint64_t
 *
 * The WWNN is computed from service parameters and returned as a uint64_t
 *
 * @param node pointer to node structure
 *
 * @return WWNN
 *
 */

uint64_t
ocs_node_get_wwnn(ocs_node_t *node)
{
	fc_plogi_payload_t *sp = (fc_plogi_payload_t*) node->service_params;

	return (((uint64_t)ocs_be32toh(sp->node_name_hi) << 32ll) | (ocs_be32toh(sp->node_name_lo)));
}

/**
 * @brief Generate node ddump data
 *
 * Generates the node ddumpdata
 *
 * @param textbuf pointer to text buffer
 * @param node pointer to node context
 *
 * @return Returns 0 on success, or a negative value on failure.
 */

int
ocs_ddump_node(ocs_textbuf_t *textbuf, ocs_node_t *node)
{
	ocs_io_t *io;
	ocs_io_t *els;
	int retval = 0;

	ocs_ddump_section(textbuf, "node", node->instance_index);
	ocs_ddump_value(textbuf, "display_name", "%s", node->display_name);
	ocs_ddump_value(textbuf, "current_state", "%s", node->current_state_name);
	ocs_ddump_value(textbuf, "prev_state", "%s", node->prev_state_name);
	ocs_ddump_value(textbuf, "current_evt", "%s", ocs_sm_event_name(node->current_evt));
	ocs_ddump_value(textbuf, "prev_evt", "%s", ocs_sm_event_name(node->prev_evt));

	ocs_ddump_value(textbuf, "indicator", "%#x", node->rnode.indicator);
	ocs_ddump_value(textbuf, "fc_id", "%#06x", node->rnode.fc_id);
	ocs_ddump_value(textbuf, "attached", "%d", node->rnode.attached);

	ocs_ddump_value(textbuf, "hold_frames", "%d", node->hold_frames);
	ocs_ddump_value(textbuf, "io_alloc_enabled", "%d", node->io_alloc_enabled);
	ocs_ddump_value(textbuf, "shutdown_reason", "%d", node->shutdown_reason);
	ocs_ddump_value(textbuf, "send_ls_rsp", "%d", node->send_ls_rsp);
	ocs_ddump_value(textbuf, "ls_rsp_did", "%d", node->ls_rsp_did);
	ocs_ddump_value(textbuf, "ls_rsp_oxid", "%#04x", node->ls_rsp_oxid);
	ocs_ddump_value(textbuf, "req_free", "%d", node->req_free);
	ocs_ddump_value(textbuf, "els_req_cnt", "%d", node->els_req_cnt);
	ocs_ddump_value(textbuf, "els_cmpl_cnt", "%d", node->els_cmpl_cnt);

	ocs_ddump_value(textbuf, "targ", "%d", node->targ);
	ocs_ddump_value(textbuf, "init", "%d", node->init);
	ocs_ddump_value(textbuf, "wwnn", "%s", node->wwnn);
	ocs_ddump_value(textbuf, "wwpn", "%s", node->wwpn);
	ocs_ddump_value(textbuf, "login_state", "%d", (node->sm.current_state == __ocs_d_device_ready) ? 1 : 0);
	ocs_ddump_value(textbuf, "chained_io_count", "%d", node->chained_io_count);
	ocs_ddump_value(textbuf, "abort_cnt", "%d", node->abort_cnt);

	ocs_display_sparams(NULL, "node_sparams", 1, textbuf, node->service_params+4);

	ocs_lock(&node->pend_frames_lock);
		if (!ocs_list_empty(&node->pend_frames)) {
			ocs_hal_sequence_t *frame;
			ocs_ddump_section(textbuf, "pending_frames", 0);
			ocs_list_foreach(&node->pend_frames, frame) {
				fc_header_t *hdr;
				char buf[256];

				hdr = frame->header.data;
				ocs_snprintf(buf, sizeof(buf), "%02x/%06x/%02x/%02x/%04x/%04x len 0x%x",
					     hdr->type, fc_be24toh(hdr->f_ctl), hdr->r_ctl, hdr->info,
					     ocs_be16toh(hdr->ox_id), ocs_be16toh(hdr->rx_id),
					     frame->payload.data_len);
				ocs_ddump_value(textbuf, "frame", "%s", buf);
			}
			ocs_ddump_endsection(textbuf, "pending_frames", 0);
		}
	ocs_unlock(&node->pend_frames_lock);

	ocs_scsi_ini_ddump(textbuf, OCS_SCSI_DDUMP_NODE, node);
	ocs_scsi_tgt_ddump(textbuf, OCS_SCSI_DDUMP_NODE, node);

	ocs_lock(&node->active_ios_lock);
		ocs_ddump_section(textbuf, "active_ios", 0);
		ocs_list_foreach(&node->active_ios, io) {
			ocs_ddump_io(textbuf, io);
		}
		ocs_ddump_endsection(textbuf, "active_ios", 0);

		ocs_ddump_section(textbuf, "els_io_pend_list", 0);
		ocs_list_foreach(&node->els_io_pend_list, els) {
			ocs_ddump_els(textbuf, els);
		}
		ocs_ddump_endsection(textbuf, "els_io_pend_list", 0);

		ocs_ddump_section(textbuf, "els_io_active_list", 0);
		ocs_list_foreach(&node->els_io_active_list, els) {
			ocs_ddump_els(textbuf, els);
		}
		ocs_ddump_endsection(textbuf, "els_io_active_list", 0);
	ocs_unlock(&node->active_ios_lock);

	ocs_ddump_endsection(textbuf, "node", node->instance_index);

	return retval;
}

/**
 * @brief check ELS request completion
 *
 * Check ELS request completion event to make sure it's for the
 * ELS request we expect. If not, invoke given common event
 * handler and return an error.
 *
 * @param ctx state machine context
 * @param evt ELS request event
 * @param arg event argument
 * @param cmd ELS command expected
 * @param node_common_func common event handler to call if ELS
 *      		   doesn't match
 * @param funcname function name that called this
 *
 * @return zero if ELS command matches, -1 otherwise
 */
int32_t
node_check_els_req(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg, uint8_t cmd, ocs_node_common_func_t node_common_func, const char *funcname)
{
	ocs_node_t *node = NULL;
	ocs_t *ocs = NULL;
	ocs_node_cb_t *cbdata = arg;
	fc_els_gen_t *els_gen = NULL;
	ocs_assert(ctx, -1);
	node = ctx->app;
	ocs_assert(node, -1);
	ocs = node->ocs;
	ocs_assert(ocs, -1);
	cbdata = arg;
	ocs_assert(cbdata, -1);
	ocs_assert(cbdata->els, -1);
	els_gen = (fc_els_gen_t *)cbdata->els->els_info->els_req.virt;
	ocs_assert(els_gen, -1);

	if ((cbdata->els->hio_type != OCS_HAL_ELS_REQ) || (els_gen->command_code != cmd)) {
		if (cbdata->els->hio_type != OCS_HAL_ELS_REQ) {
			ocs_log_debug(node->ocs, "[%s] %-20s expecting ELS cmd=x%x received type=%d\n",
				node->display_name, funcname, cmd, cbdata->els->hio_type);
		} else {
			ocs_log_debug(node->ocs, "[%s] %-20s expecting ELS cmd=x%x received cmd=x%x\n",
				node->display_name, funcname, cmd, els_gen->command_code);
		}
		/* send event to common handler */
		node_common_func(funcname, ctx, evt, arg);
		return -1;
	}
	return 0;
}

/**
 * @brief check CT request completion
 *
 * Check ELS CY request completion event to make sure it's for the
 * nameserver request we expect. If not, invoke given common
 * event handler and return an error.
 *
 * @param ctx state machine context
 * @param evt ELS request event
 * @param arg event argument
 * @param cmd nameserver command expected
 * @param node_common_func common event handler to call if
 *      		   nameserver cmd doesn't match
 * @param funcname function name that called this
 *
 * @return zero if CT command matches, -1 otherwise
 */
int32_t
node_check_ct_req(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg, uint32_t cmd, ocs_node_common_func_t node_common_func, const char *funcname)
{
	ocs_node_t *node = NULL;
	ocs_t *ocs = NULL;
	ocs_node_cb_t *cbdata = arg;
	fcct_iu_header_t *fcct = NULL;
	ocs_assert(ctx, -1);
	node = ctx->app;
	ocs_assert(node, -1);
	ocs = node->ocs;
	ocs_assert(ocs, -1);
	cbdata = arg;
	ocs_assert(cbdata, -1);
	ocs_assert(cbdata->els, -1);
	fcct = (fcct_iu_header_t *)cbdata->els->els_info->els_req.virt;
	ocs_assert(fcct, -1);

	if ((cbdata->els->hio_type != OCS_HAL_FC_CT) || fcct->cmd_rsp_code != ocs_htobe16(cmd)) {
		if (cbdata->els->hio_type != OCS_HAL_FC_CT) {
			ocs_log_debug(node->ocs, "[%s] %-20s expecting CT cmd=x%x received type=%d\n",
				node->display_name, funcname, cmd, cbdata->els->hio_type);
		} else {
			ocs_log_debug(node->ocs, "[%s] %-20s expecting CT cmd=x%x received cmd=x%x\n",
				node->display_name, funcname, cmd, fcct->cmd_rsp_code);
		}
		/* send event to common handler */
		node_common_func(funcname, ctx, evt, arg);
		return -1;
	}
	return 0;
}

int32_t
node_check_ct_resp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	ocs_node_cb_t *cbdata = arg;
	fcct_iu_header_t *ct_resp = (fcct_iu_header_t *)cbdata->els->els_info->els_rsp.virt;
	int32_t rc;
	ocs_assert(ct_resp, -1);

	if (cbdata->els->hio_type != OCS_HAL_FC_CT) {
		rc = -1;
	} else {
		rc = ocs_be16toh(ct_resp->cmd_rsp_code);
	}

	return rc;
}
void
ocs_mgmt_node_list(ocs_textbuf_t *textbuf, void *object)
{
	ocs_io_t *io;
	ocs_node_t *node = (ocs_node_t *)object;

	ocs_mgmt_start_section(textbuf, "node", node->instance_index);

	/* Readonly values */
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "display_name");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "indicator");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "fc_id");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "attached");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "hold_frames");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "shutting_down");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "req_free");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "ox_id");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "ox_id_in_use");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "abort_cnt");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "targ");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "init");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "wwpn");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "wwnn");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "pend_frames");
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_RD, "chained_io_count");

	/* Actions */
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_EX, "resume");

	ocs_lock(&node->active_ios_lock);
	ocs_list_foreach(&node->active_ios, io) {
		if ((io->mgmt_functions) && (io->mgmt_functions->get_list_handler)) {
			io->mgmt_functions->get_list_handler(textbuf, io);
		}
	}
	ocs_unlock(&node->active_ios_lock);

	ocs_mgmt_end_section(textbuf, "node", node->instance_index);
}

int
ocs_mgmt_node_get(ocs_textbuf_t *textbuf, char *parent, char *name, void *object)
{
	ocs_io_t *io;
	ocs_node_t *node = (ocs_node_t *)object;
	char qualifier[80];
	int retval = -1;

	ocs_mgmt_start_section(textbuf, "node", node->instance_index);

	ocs_snprintf(qualifier, sizeof(qualifier), "%s/node[%d]", parent, node->instance_index);

	/* If it doesn't start with my qualifier I don't know what to do with it */
	if (ocs_strncmp(name, qualifier, strlen(qualifier)) == 0) {
		char *unqualified_name = name + strlen(qualifier) +1;

		/* See if it's a value I can supply */
		if (ocs_strcmp(unqualified_name, "display_name") == 0) {
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "display_name", node->display_name);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "indicator") == 0) {
			ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "indicator", "0x%x", node->rnode.indicator);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "fc_id") == 0) {
			ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "fc_id", "0x%06x", node->rnode.fc_id);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "attached") == 0) {
			ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "attached", node->rnode.attached);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "hold_frames") == 0) {
			ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "hold_frames", node->hold_frames);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "io_alloc_enabled") == 0) {
			ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "io_alloc_enabled", node->io_alloc_enabled);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "req_free") == 0) {
			ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "req_free", node->req_free);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "ls_rsp_oxid") == 0) {
			ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "ls_rsp_oxid", "0x%#04x", node->ls_rsp_oxid);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "ls_rsp_did") == 0) {
			ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "ls_rsp_did", "0x%#04x", node->ls_rsp_did);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "abort_cnt") == 0) {
			ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "abort_cnt", "%d", node->abort_cnt);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "targ") == 0) {
			ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "targ",  node->targ);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "init") == 0) {
			ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "init",  node->init);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "wwpn") == 0) {
			ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "wwpn", "%s", node->wwpn);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "wwnn") == 0) {
			ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "wwnn", "%s", node->wwnn);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "current_state") == 0) {
			ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "current_state", node->current_state_name);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "login_state") == 0) {
			ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "login_state", "%d", (node->sm.current_state == __ocs_d_device_ready) ? 1 : 0);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "pend_frames") == 0) {
			ocs_hal_sequence_t *frame;
			ocs_lock(&node->pend_frames_lock);
				ocs_list_foreach(&node->pend_frames, frame) {
					fc_header_t *hdr;
					char buf[128];

					hdr = frame->header.data;
					ocs_snprintf(buf, sizeof(buf), "%02x/%04x/%04x len x%x", hdr->r_ctl,
						 ocs_be16toh(hdr->ox_id), ocs_be16toh(hdr->rx_id),
						 frame->payload.data_len);
					ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "pend_frames", buf);
				}
			ocs_unlock(&node->pend_frames_lock);
			retval = 0;
		} else if (ocs_strcmp(unqualified_name, "chained_io_count") == 0) {
			ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "chained_io_count", "%d", node->chained_io_count);
			retval = 0;
		} else {
			/* If I didn't know the value of this status pass the request to each of my children */
			ocs_lock(&node->active_ios_lock);
				ocs_list_foreach(&node->active_ios, io) {
					if ((io->mgmt_functions) && (io->mgmt_functions->get_handler)) {
						retval = io->mgmt_functions->get_handler(textbuf, qualifier, name, io);
					}

					if (retval == 0) {
						break;
					}
				}
			ocs_unlock(&node->active_ios_lock);
		}
	}

	ocs_mgmt_end_section(textbuf, "node", node->instance_index);

	return retval;
}

void
ocs_mgmt_node_get_all(ocs_textbuf_t *textbuf, void *object)
{
	ocs_io_t *io;
	ocs_node_t *node = (ocs_node_t *)object;
	ocs_hal_sequence_t *frame;

	ocs_mgmt_start_section(textbuf, "node", node->instance_index);

	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "display_name", node->display_name);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "indicator", "0x%x", node->rnode.indicator);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "fc_id", "0x%06x", node->rnode.fc_id);
	ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "attached", node->rnode.attached);
	ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "hold_frames", node->hold_frames);
	ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "io_alloc_enabled", node->io_alloc_enabled);
	ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "req_free", node->req_free);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "ls_rsp_oxid", "0x%#04x", node->ls_rsp_oxid);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "ls_rsp_did", "0x%#04x", node->ls_rsp_did);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "abort_cnt", "%d", node->abort_cnt);
	ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "targ",  node->targ);
	ocs_mgmt_emit_boolean(textbuf, MGMT_MODE_RD, "init",  node->init);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "wwpn", "%s", node->wwpn);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "wwnn", "%s", node->wwnn);

	ocs_lock(&node->pend_frames_lock);
	ocs_list_foreach(&node->pend_frames, frame) {
		fc_header_t *hdr;
		char buf[128];

		hdr = frame->header.data;
		ocs_snprintf(buf, sizeof(buf), "%02x/%04x/%04x len x%x", hdr->r_ctl,
			     ocs_be16toh(hdr->ox_id), ocs_be16toh(hdr->rx_id),
			     frame->payload.data_len);
		ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "pend_frames", buf);
	}
	ocs_unlock(&node->pend_frames_lock);

	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "chained_io_count", "%d", node->chained_io_count);
	ocs_mgmt_emit_property_name(textbuf, MGMT_MODE_EX, "resume");
	ocs_mgmt_emit_string(textbuf, MGMT_MODE_RD, "current_state", node->current_state_name);
	ocs_mgmt_emit_int(textbuf, MGMT_MODE_RD, "login_state", "%d", (node->sm.current_state == __ocs_d_device_ready) ? 1 : 0);

	ocs_lock(&node->active_ios_lock);
	ocs_list_foreach(&node->active_ios, io) {
		if ((io->mgmt_functions) && (io->mgmt_functions->get_all_handler)) {
			io->mgmt_functions->get_all_handler(textbuf,io);
		}
	}
	ocs_unlock(&node->active_ios_lock);

	ocs_mgmt_end_section(textbuf, "node", node->instance_index);
}

int
ocs_mgmt_node_set(char *parent, char *name, char *value, void *object)
{
	ocs_io_t *io;
	ocs_node_t *node = (ocs_node_t *)object;
	char qualifier[80];
	int retval = -1;

	ocs_snprintf(qualifier, sizeof(qualifier), "%s/node[%d]", parent, node->instance_index);

	/* If it doesn't start with my qualifier I don't know what to do with it */
	if (ocs_strncmp(name, qualifier, strlen(qualifier)) == 0) {

		ocs_lock(&node->active_ios_lock);
		ocs_list_foreach(&node->active_ios, io) {
			if ((io->mgmt_functions) && (io->mgmt_functions->set_handler)) {
				retval = io->mgmt_functions->set_handler(qualifier, name, value, io);
			}

			if (retval == 0) {
				break;
			}

		}
		ocs_unlock(&node->active_ios_lock);

	}

	return retval;
}

int
ocs_mgmt_node_exec(char *parent, char *action, void *arg_in, uint32_t arg_in_length,
		   void *arg_out, uint32_t arg_out_length, void *object)
{
	ocs_io_t *io;
	ocs_node_t *node = (ocs_node_t *)object;
	char qualifier[80];
	int retval = -EOPNOTSUPP;

	ocs_snprintf(qualifier, sizeof(qualifier), "%s.node%d", parent, node->instance_index);

	/* If it doesn't start with my qualifier I don't know what to do with it */
	if (ocs_strncmp(action, qualifier, strlen(qualifier)) == 0) {
		char *unqualified_name = action + strlen(qualifier) +1;

		if (ocs_strcmp(unqualified_name, "resume") == 0) {
			ocs_node_post_event(node, OCS_EVT_RESUME, NULL);
		}

		{
			/* If I didn't know how to do this action pass the request to each of my children */
			ocs_lock(&node->active_ios_lock);
				ocs_list_foreach(&node->active_ios, io) {
					if ((io->mgmt_functions) && (io->mgmt_functions->exec_handler)) {
						retval = io->mgmt_functions->exec_handler(qualifier, action, arg_in, arg_in_length,
							arg_out, arg_out_length, io);
					}

					if (retval == 0) {
						break;
					}

				}
			ocs_unlock(&node->active_ios_lock);
		}
	}

	return retval;
}



/**
 * @brief Return TRUE if active ios list is empty
 *
 * Test if node->active_ios list is empty while holding the node->active_ios_lock.
 *
 * @param node pointer to node object
 *
 * @return TRUE if node active ios list is empty
 */

int
ocs_node_active_ios_empty(ocs_node_t *node)
{
	int empty;

	ocs_lock(&node->active_ios_lock);
		empty = ocs_list_empty(&node->active_ios);
	ocs_unlock(&node->active_ios_lock);
	return empty;
}

/**
 * @brief Pause a node
 *
 * The node is placed in the __ocs_node_paused state after saving the state
 * to return to
 *
 * @param node Pointer to node object
 * @param state State to resume to
 *
 * @return none
 */

void
ocs_node_pause(ocs_node_t *node, ocs_sm_function_t state)
{
	node->nodedb_state = state;
	ocs_node_transition(node, __ocs_node_paused, NULL);
}

/**
 * @brief Paused node state
 *
 * This state is entered when a state is "paused". When resumed, the node
 * is transitioned to a previously saved state (node->ndoedb_state)
 *
 * @param ctx Remote node state machine context.
 * @param evt Event to process.
 * @param arg Per event optional argument.
 *
 * @return returns NULL
 */

void *
__ocs_node_paused(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg)
{
	std_node_state_decl();

	node_sm_trace();

	switch(evt) {
	case OCS_EVT_ENTER:
		node_printf(node, "Paused\n");
		break;

	case OCS_EVT_RESUME: {
		ocs_sm_function_t pf = node->nodedb_state;

		node->nodedb_state = NULL;
		ocs_node_transition(node, pf, NULL);
		break;
	}

	case OCS_EVT_DOMAIN_ATTACH_OK:
		break;

	case OCS_EVT_SHUTDOWN:
		node->req_free = 1;
		break;

	default:
		__ocs_node_common(__func__, ctx, evt, arg);
		break;
	}
	return NULL;
}

/**
 * @brief Resume a paused state
 *
 * Posts a resume event to the paused node.
 *
 * @param node Pointer to node object
 *
 * @return returns 0 for success, a negative error code value for failure.
 */

int32_t
ocs_node_resume(ocs_node_t *node)
{
	ocs_assert(node != NULL, -1);

	ocs_node_post_event(node, OCS_EVT_RESUME, NULL);

	return 0;
}

/**
 * @ingroup node_common
 * @brief Dispatch a ELS frame.
 *
 * <h3 class="desc">Description</h3>
 * An ELS frame is dispatched to the \c node state machine.
 * RQ Pair mode: this function is always called with a NULL hal
 * io.
 *
 * @param node Node that originated the frame.
 * @param seq header/payload sequence buffers
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled and RX buffers need
 * to be returned.
 */

int32_t
ocs_node_recv_els_frame(ocs_node_t *node, ocs_hal_sequence_t *seq)
{
	struct {
		uint32_t cmd;
		ocs_sm_event_t evt;
		uint32_t payload_size;
	} els_cmd_list[] = {
		{FC_ELS_CMD_PLOGI,	OCS_EVT_PLOGI_RCVD, 	sizeof(fc_plogi_payload_t)},
		{FC_ELS_CMD_FLOGI,	OCS_EVT_FLOGI_RCVD, 	sizeof(fc_plogi_payload_t)},
		{FC_ELS_CMD_LOGO,	OCS_EVT_LOGO_RCVD, 	sizeof(fc_acc_payload_t)},
		{FC_ELS_CMD_RRQ,	OCS_EVT_RRQ_RCVD, 	sizeof(fc_acc_payload_t)},
		{FC_ELS_CMD_RDP,	OCS_EVT_RDP_RCVD, 	sizeof(fc_rdp_res_frame_t)},
		{FC_ELS_CMD_PRLI, 	OCS_EVT_PRLI_RCVD, 	sizeof(fc_prli_payload_t)},
		{FC_ELS_CMD_PRLO, 	OCS_EVT_PRLO_RCVD, 	sizeof(fc_prlo_payload_t)},
		{FC_ELS_CMD_PDISC, 	OCS_EVT_PDISC_RCVD, 	MAX_ACC_REJECT_PAYLOAD},
		{FC_ELS_CMD_FDISC, 	OCS_EVT_FDISC_RCVD, 	MAX_ACC_REJECT_PAYLOAD},
		{FC_ELS_CMD_ADISC, 	OCS_EVT_ADISC_RCVD, 	sizeof(fc_adisc_payload_t)},
		{FC_ELS_CMD_RSCN, 	OCS_EVT_RSCN_RCVD, 	MAX_ACC_REJECT_PAYLOAD},
		{FC_ELS_CMD_SCR	, 	OCS_EVT_SCR_RCVD, 	MAX_ACC_REJECT_PAYLOAD},
		{FC_ELS_CMD_AUTH, 	OCS_EVT_AUTH_RCVD, 	MAX_ACC_REJECT_PAYLOAD},
	};
	ocs_t *ocs = node->ocs;
	ocs_node_cb_t cbdata;
	fc_header_t *hdr;
	uint8_t *buf;
	ocs_sm_event_t evt = OCS_EVT_ELS_RCVD;
	uint32_t payload_size = MAX_ACC_REJECT_PAYLOAD;
	uint32_t i;

	hdr = (fc_header_t *)seq->header.data;
	buf = seq->payload.data;
	cbdata.payload_len = seq->payload.data_len;

	ocs_memset(&cbdata, 0, sizeof(cbdata));
	cbdata.header = hdr;
	cbdata.payload = buf;

	/* find a matching event for the ELS command */
	for (i = 0; i < ARRAY_SIZE(els_cmd_list); i ++) {
		if (els_cmd_list[i].cmd == buf[0]) {
			evt = els_cmd_list[i].evt;
			payload_size = els_cmd_list[i].payload_size;
			if (els_cmd_list[i].cmd == FC_ELS_CMD_PRLI) {
				fc_prli_payload_t *prli = (fc_prli_payload_t *)buf;

				/*
				 * FC-NVME spec Section 6.3.3 defines different
				 * format for PRLI ACC
				 */
				if (prli->type == FC_TYPE_NVME)
					payload_size = sizeof(fc_nvme_prli_payload_t);
			}
			break;
		}
	}

	/* Update FCP control requests */
#if !defined(OCSU_FC_RAMD) && !defined(OCSU_FC_WORKLOAD) && !defined(OCS_USPACE_SPDK)
	percpu_counter_add(&ocs->xport->fc_stats.stats.fcp_stats.control_requests, 1);
#else
	ocs_atomic_add_return(&ocs->xport->fc_stats.stats.fcp_stats.control_requests, 1);
#endif

	switch(evt) {
	case OCS_EVT_FLOGI_RCVD:
		ocs_display_sparams(node->display_name, "flogi rcvd req", 0, NULL, ((uint8_t *)buf) + 4);
		break;
	case OCS_EVT_FDISC_RCVD:
		ocs_display_sparams(node->display_name, "fdisc rcvd req", 0, NULL, ((uint8_t *)buf) + 4);
		break;
	case OCS_EVT_PLOGI_RCVD: {
		fc_plogi_payload_t *sp = (fc_plogi_payload_t *)buf;

		ocs_log_info(ocs, "ELS PLOGI (0x%x) REQ rcvd; OX_ID: %04x, S_ID: %06x, WWPN: %016" PRIX64"\n",
			     FC_ELS_CMD_PLOGI, ocs_be16toh(hdr->ox_id), fc_be24toh(hdr->s_id),
			     (((uint64_t)ocs_be32toh(sp->port_name_hi) << 32ll) | (ocs_be32toh(sp->port_name_lo))));
		ocs_display_sparams(node->display_name, "plogi rcvd req", 0, NULL, ((uint8_t *)buf) + 4);
		break;
	}
	case OCS_EVT_PRLI_RCVD:
		ocs_log_info(ocs, "ELS PRLI (0x%x) REQ rcvd; OX_ID: %04x, S_ID: %06x, WWPN: %016" PRIX64"\n",
			     FC_ELS_CMD_PRLI, ocs_be16toh(hdr->ox_id), fc_be24toh(hdr->s_id), ocs_node_get_wwpn(node));
		break;
	case OCS_EVT_RDP_RCVD:
		ocs_log_info(ocs, "ELS RDP (0x%x) REQ rcvd; OX_ID: %04x, S_ID: %06x, WWPN: %016" PRIX64"\n",
			     FC_ELS_CMD_RDP, ocs_be16toh(hdr->ox_id), fc_be24toh(hdr->s_id), ocs_node_get_wwpn(node));

		/* First get required RDP data from FW,
		 * send RDP ELS response from completion*/
		ocs_rdp_defer_response(ocs, hdr, seq->fcfi);
		ocs_hal_sequence_free(&ocs->hal, seq);
		return 0;
	case OCS_EVT_AUTH_RCVD:
		ocs_log_debug(ocs, "ELS AUTH rcvd\n");
		break;
	default:
		break;
	}

	cbdata.io = ocs_els_io_alloc(node, payload_size, OCS_ELS_ROLE_RESPONDER);
	if (cbdata.io != NULL) {
		cbdata.io->hal_priv = seq->hal_priv;
		/* if we're here, sequence initiative has been transferred */
		cbdata.io->seq_init = 1;

		ocs_node_post_event(node, evt, &cbdata);
	} else {
		node_printf(node, "failure to allocate SCSI IO for ELS s_id %06x d_id %06x ox_id %04x rx_id %04x\n",
			    fc_be24toh(hdr->s_id), fc_be24toh(hdr->d_id), ocs_be16toh(hdr->ox_id), ocs_be16toh(hdr->rx_id));
	}

	ocs_hal_sequence_free(&ocs->hal, seq);
	return 0;
}

/**
 * @ingroup node_common
 * @brief Dispatch a BLS frame
 *
 * <h3 class="desc">Description</h3>
 * A BLS frame is dispatched to the node state machine. This
 * function is used for both RQ Pair and sequence coalescing.
 *
 * @param node Node that originated the frame.
 * @param seq Header/payload sequence buffers
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled and RX buffers need
 * to be returned.
 */
int32_t
ocs_node_recv_bls_frame(ocs_node_t *node, ocs_hal_sequence_t *seq)
{
	ocs_t *ocs = node->ocs;
	ocs_xport_t *xport = ocs->xport;
	fc_header_t *hdr;
	void *buf;
	uint16_t ox_id;
	uint16_t rx_id;
	ocs_node_cb_t cbdata;
	int32_t sit;

	hdr = (fc_header_t *)seq->header.data;
	buf = seq->payload.data;
	cbdata.payload_len = seq->payload.data_len;

	ox_id = ocs_be16toh(hdr->ox_id);
	rx_id = ocs_be16toh(hdr->rx_id);

	ocs_memset(&cbdata, 0, sizeof(cbdata));
	cbdata.header = hdr;
	cbdata.payload = buf;
	cbdata.io = ocs_scsi_io_alloc(node, OCS_SCSI_IO_ROLE_RESPONDER);
	if (!cbdata.io) {
		ocs_atomic_add_return(&xport->io_alloc_failed_count, 1);
		node_printf(node, "SCSI IO allocation failed for ABTS received "\
			    "s_id %06x d_id %06x ox_id %04x rx_id %04x\n",
			    fc_be24toh(hdr->s_id), fc_be24toh(hdr->d_id), ox_id, rx_id);
		goto exit;
	}

	cbdata.io->hal_priv = seq->hal_priv;
	cbdata.io->ocs = ocs;
	cbdata.io->node = node;
	cbdata.io->cmd_tgt = true;

	sit = fc_be24toh(hdr->f_ctl) & FC_FCTL_SEQUENCE_INITIATIVE;
	switch(hdr->info) {
	case FC_RCTL_BLS_INFO_ABTS:
		if (sit) {
			cbdata.io->seq_init = true;
			node->abort_cnt++;
			ocs_node_post_event(node, OCS_EVT_ABTS_RCVD, &cbdata);
		} else {
			node_printf(node, "Drop ABTS no-SIT frame hdr = %08x %08x %08x %08x %08x %08x\n",
				    ocs_htobe32(((uint32_t *)hdr)[0]),
				    ocs_htobe32(((uint32_t *)hdr)[1]),
				    ocs_htobe32(((uint32_t *)hdr)[2]),
				    ocs_htobe32(((uint32_t *)hdr)[3]),
				    ocs_htobe32(((uint32_t *)hdr)[4]),
				    ocs_htobe32(((uint32_t *)hdr)[5]));
		}
		break;
	case FC_RCTL_BLS_INFO_FLUSH:
		ocs_node_post_event(node, OCS_EVT_FLUSH_BLS_RCVD, &cbdata);
		break;
	default:
		node_printf(node, "Drop unknown BLS frame hdr = %08x %08x %08x %08x %08x %08x\n",
			    ocs_htobe32(((uint32_t *)hdr)[0]),
			    ocs_htobe32(((uint32_t *)hdr)[1]),
			    ocs_htobe32(((uint32_t *)hdr)[2]),
			    ocs_htobe32(((uint32_t *)hdr)[3]),
			    ocs_htobe32(((uint32_t *)hdr)[4]),
			    ocs_htobe32(((uint32_t *)hdr)[5]));
		break;
	}

exit:
	ocs_hal_sequence_free(&ocs->hal, seq);
	return 0;
}

/**
 * @ingroup node_common
 * @brief Dispatch a CT frame.
 *
 * <h3 class="desc">Description</h3>
 * A CT frame is dispatched to the \c node state machine.
 * RQ Pair mode: this function is always called with a NULL hal
 * io.
 *
 * @param node Node that originated the frame.
 * @param seq header/payload sequence buffers
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled and RX buffers need
 * to be returned.
 */

int32_t
ocs_node_recv_ct_frame(ocs_node_t *node, ocs_hal_sequence_t *seq)
{
	ocs_t *ocs = node->ocs;
	fc_header_t *hdr = seq->header.data;
	fcct_iu_header_t *iu = seq->payload.data;

	ocs_sm_event_t evt = OCS_EVT_ELS_RCVD;
	uint32_t payload_size = MAX_ACC_REJECT_PAYLOAD;
	uint16_t gscmd = ocs_be16toh(iu->cmd_rsp_code);
	uint16_t gstype = iu->gs_type;
	ocs_node_cb_t cbdata;
	uint32_t i;
	struct {
		uint32_t cmd;
		ocs_sm_event_t evt;
		uint32_t payload_size;
	} ct_cmd_list[] = {
		{FC_GS_NAMESERVER_RFF_ID, OCS_EVT_RFF_ID_RCVD, 100},
		{FC_GS_NAMESERVER_RFT_ID, OCS_EVT_RFT_ID_RCVD, 100},
		{FC_GS_NAMESERVER_GNN_ID, OCS_EVT_GNN_ID_RCVD, 100},
		{FC_GS_NAMESERVER_GPN_ID, OCS_EVT_GPN_ID_RCVD, 100},
		{FC_GS_NAMESERVER_GFPN_ID, OCS_EVT_GFPN_ID_RCVD, 100},
		{FC_GS_NAMESERVER_GFF_ID, OCS_EVT_GFF_ID_RCVD, 100},
		{FC_GS_NAMESERVER_GID_FT, OCS_EVT_GID_FT_RCVD, 256},
		{FC_GS_NAMESERVER_GID_PT, OCS_EVT_GID_PT_RCVD, 256},
		{FC_GS_NAMESERVER_GA_NXT, OCS_EVT_GA_NXT_RCVD, 1024},
		{FC_GS_NAMESERVER_RPN_ID, OCS_EVT_RPN_ID_RCVD, 100},
		{FC_GS_NAMESERVER_RNN_ID, OCS_EVT_RNN_ID_RCVD, 100},
		{FC_GS_NAMESERVER_RCS_ID, OCS_EVT_RCS_ID_RCVD, 100},
		{FC_GS_NAMESERVER_RSNN_NN, OCS_EVT_RSNN_NN_RCVD, 100},
		{FC_GS_NAMESERVER_RSPN_ID, OCS_EVT_RSPN_ID_RCVD, 100},
		{FC_GS_NAMESERVER_RHBA, OCS_EVT_RHBA_RCVD, 100},
		{FC_GS_NAMESERVER_RPA, OCS_EVT_RPA_RCVD, 100},
	};

	ocs_memset(&cbdata, 0, sizeof(cbdata));
	cbdata.header = hdr;
	cbdata.payload = iu;
	cbdata.payload_len = seq->payload.data_len;

	if (gstype == FC_GS_TYPE_LOOPBACK) {
		ocs_io_t *els_loopback_io = NULL;

		uint16_t ox_id = ocs_be16toh(hdr->ox_id);

		els_loopback_io = ocs_io_find_init_els_io(ocs, node, ox_id);
		if (els_loopback_io && els_loopback_io->els_info->loopback_evt_data.is_loopback_frame) {
			cbdata.els = els_loopback_io;

			if (els_loopback_io->els_info->loopback_evt_data.loopback_rx_data) {
				ocs_memcpy(els_loopback_io->els_info->loopback_evt_data.loopback_rx_data,
					   ((uint8_t *)iu + sizeof(fcct_iu_header_t)),
					   els_loopback_io->els_info->loopback_evt_data.loopback_rx_data_len);
			}
			ocs_sem_v(&els_loopback_io->els_info->loopback_evt_data.wait_io_sem);
			evt = OCS_EVT_CT_LOOPBACK_RCVD;
		} else {
			ocs_log_err(node->ocs, "Unable to find loopback originator IO\n");
			evt = OCS_EVT_CT_LOOPBACK_RCVD_NO_IO;
		}
	} else {
		/* find a matching event for the ELS/GS command */
		for (i = 0; i < ARRAY_SIZE(ct_cmd_list); i ++) {
			if (ct_cmd_list[i].cmd == gscmd) {
				evt = ct_cmd_list[i].evt;
				payload_size = ct_cmd_list[i].payload_size;
				break;
			}
		}
	}

	/* Update FCP control requests */
#if !defined(OCSU_FC_RAMD) && !defined(OCSU_FC_WORKLOAD) && !defined(OCS_USPACE_SPDK)
	percpu_counter_add(&ocs->xport->fc_stats.stats.fcp_stats.control_requests, 1);
#else
	ocs_atomic_add_return(&ocs->xport->fc_stats.stats.fcp_stats.control_requests, 1);
#endif

	/* Allocate an IO and send a reject */
	cbdata.io = ocs_els_io_alloc(node, payload_size, OCS_ELS_ROLE_RESPONDER);
	if (cbdata.io == NULL) {
		node_printf(node, "GS IO failed for s_id %06x d_id %06x ox_id %04x rx_id %04x\n",
			fc_be24toh(hdr->s_id), fc_be24toh(hdr->d_id),
			ocs_be16toh(hdr->ox_id), ocs_be16toh(hdr->rx_id));
		return -1;
	}

	cbdata.io->hal_priv = seq->hal_priv;
	ocs_node_post_event(node, evt, &cbdata);

	ocs_hal_sequence_free(&ocs->hal, seq);
	return 0;
}

/**
 * @ingroup node_common
 * @brief Dispatch a FCP command frame when the node is not ready.
 *
 * <h3 class="desc">Description</h3>
 * A frame is dispatched to the \c node state machine.
 *
 * @param node Node that originated the frame.
 * @param seq header/payload sequence buffers
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled.
 */

int32_t
ocs_node_recv_fcp_cmd(ocs_node_t *node, ocs_hal_sequence_t *seq)
{
	ocs_node_cb_t cbdata;
	ocs_t *ocs = node->ocs;

	ocs_memset(&cbdata, 0, sizeof(cbdata));

	cbdata.header = seq->header.data;
	cbdata.payload = seq->payload.data;
	cbdata.payload_len = seq->payload.data_len;
	ocs_node_post_event(node, OCS_EVT_FCP_CMD_RCVD, &cbdata);
	ocs_hal_sequence_free(&ocs->hal, seq);
	return 0;
}

/**
 * @ingroup node_common
 * @brief Stub handler for non-ABTS BLS frames
 *
 * <h3 class="desc">Description</h3>
 * Log message and drop. Customer can plumb it to their back-end as needed
 *
 * @param node Node that originated the frame.
 * @param seq header/payload sequence buffers
 *
 * @return Returns 0
 */

int32_t
ocs_node_recv_bls_no_sit(ocs_node_t *node, ocs_hal_sequence_t *seq)
{
	fc_header_t *hdr = seq->header.data;

	node_printf_ratelimited(node, "Dropping frame hdr = %08x %08x %08x %08x %08x %08x\n",
		    ocs_htobe32(((uint32_t *)hdr)[0]),
		    ocs_htobe32(((uint32_t *)hdr)[1]),
		    ocs_htobe32(((uint32_t *)hdr)[2]),
		    ocs_htobe32(((uint32_t *)hdr)[3]),
		    ocs_htobe32(((uint32_t *)hdr)[4]),
		    ocs_htobe32(((uint32_t *)hdr)[5]));

	return -1;
}

int32_t
ocs_node_is_remote_node(ocs_remote_node_t *rnode)
{
	if (rnode) {
		if (rnode->fc_id != FC_ADDR_FABRIC &&
		    rnode->fc_id != FC_ADDR_NAMESERVER &&
		    rnode->fc_id != FC_ADDR_CONTROLLER &&
		    !FC_ADDR_IS_DOMAIN_CTRL(rnode->fc_id)) {
			return TRUE;
		}
	}

	return FALSE;
}

/**
 * @ingroup node_common
 * @brief Dispatch a TOW data frame when the node is not ready.
 *
 * <h3 class="desc">Description</h3>
 * A frame is dispatched to the \c node state machine.
 *
 * @param node Node that originated the frame.
 * @param seq header/payload sequence buffers
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled.
 */
int32_t 
ocs_node_recv_tow_data(ocs_node_t *node, ocs_hal_sequence_t *seq)
{
	ocs_node_cb_t cbdata;
	ocs_t *ocs = node->ocs;

	ocs_assert(seq->hio->ul_io, -1);

	ocs_memset(&cbdata, 0, sizeof(cbdata));
	cbdata.io = seq->hio->ul_io;
	cbdata.header = seq->header.data;
	cbdata.payload = seq->payload.data;
	ocs_node_post_event(node, OCS_EVT_TOW_DATA_RCVD, &cbdata);

	ocs_hal_sequence_free(&ocs->hal, seq);

	return 0;
}
