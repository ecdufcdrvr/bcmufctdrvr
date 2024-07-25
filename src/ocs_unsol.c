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
 * Code to handle unsolicited received FC frames.
 */

/*!
 * @defgroup unsol Unsolicited Frame Handling
 */

#include "ocs.h"
#include "ocs_els.h"
#include "ocs_fabric.h"
#include "ocs_device.h"
#include "scsi_cmds.h"

static int32_t ocs_unsol_process(ocs_t *ocs, ocs_hal_sequence_t *seq);
static int32_t ocs_dispatch_fcp_cmd(ocs_node_t *node, ocs_hal_sequence_t *seq);
static int32_t ocs_dispatch_tow_fcp_cmd(ocs_node_t *node, ocs_hal_sequence_t *seq);
static int32_t ocs_dispatch_fcp_data(ocs_node_t *node, ocs_hal_sequence_t *seq);
static int32_t ocs_domain_dispatch_frame(void *arg, ocs_hal_sequence_t *seq);
static int32_t ocs_node_dispatch_frame(void *arg, ocs_hal_sequence_t *seq);
static int32_t ocs_fc_tmf_rejected_cb(ocs_io_t *io, ocs_scsi_io_status_e scsi_status, uint32_t flags, void *arg);
static uint8_t ocs_node_frames_held(void *arg);
static uint8_t ocs_domain_frames_held(void *arg);
static int32_t ocs_purge_pending(ocs_t *ocs, ocs_list_t *pend_list, ocs_lock_t *list_lock);
static int32_t ocs_sframe_send_scsi_status(ocs_node_t *node, ocs_hal_sequence_t *seq, uint8_t scsi_status,
					   fixed_sense_data_t *sense_data, size_t sense_data_length);
ocs_hal_sequence_t* ocs_pend_frame_seq_alloc(ocs_t *ocs, ocs_hal_sequence_t *seq);

#define OCS_MAX_FRAMES_BEFORE_YEILDING 10000
#define OCS_PEND_FRAMES_WATERMARK 8000

/**
 * @brief Process the RQ circular buffer and process the incoming frames.
 *
 * @param mythread Pointer to thread object.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
int32_t
ocs_unsol_rq_thread(ocs_thread_t *mythread)
{
	ocs_xport_rq_thread_info_t *thread_data = mythread->arg;
	ocs_t *ocs = thread_data->ocs;
	ocs_hal_sequence_t *seq;
	uint32_t yield_count = OCS_MAX_FRAMES_BEFORE_YEILDING;

	ocs_log_debug(ocs, "%s running\n", mythread->name);
	while (!ocs_thread_terminate_requested(mythread)) {
		seq = ocs_cbuf_get(thread_data->seq_cbuf, 100000);
		if (seq == NULL) {
			/* Prevent soft lockups by yielding the CPU */
			ocs_thread_yield(&thread_data->thread);
			yield_count = OCS_MAX_FRAMES_BEFORE_YEILDING;
			continue;
		}
		/* free sequence buffer in failure scenario */
		if (ocs_unsol_process((ocs_t*)seq->hal->os, seq))
			ocs_hal_sequence_free(&ocs->hal, seq);

		/* We have to prevent CPU soft lockups, so just yield the CPU after x frames. */
		if (--yield_count == 0) {
			ocs_thread_yield(&thread_data->thread);
			yield_count = OCS_MAX_FRAMES_BEFORE_YEILDING;
		}
	}
	ocs_log_debug(ocs, "%s exiting\n", mythread->name);
	thread_data->thread_started = FALSE;
	return 0;
}

/**
 * @ingroup unsol
 * @brief Callback function when aborting a port owned XRI
 * exchanges.
 *
 * @return Returns 0.
 */
static int32_t
ocs_unsol_abort_cb (ocs_hal_io_t *hio, ocs_remote_node_t *rnode, uint32_t len, int32_t status, uint32_t ext, void *arg)
{
	ocs_t *ocs = arg;
	ocs_assert(hio, -1);
	ocs_assert(arg, -1);
	ocs_log_debug(ocs, "xri=0x%x tag=0x%x\n", hio->indicator, hio->reqtag);
	ocs_hal_io_free(&ocs->hal, hio);
	return 0;
}

ocs_hal_sequence_t *
ocs_pend_frame_seq_alloc(ocs_t *ocs, ocs_hal_sequence_t *seq)
{
	ocs_hal_sequence_t *pend_seq;

	pend_seq = ocs_malloc(ocs, sizeof(ocs_hal_sequence_t), OCS_M_NOWAIT | OCS_M_ZERO);
	if (pend_seq == NULL) {
		ocs_log_err(ocs, "Failed to allocate seq buff for pending frame\n");
		goto fail_and_exit;
	}

	pend_seq->pend_frame_seq = true;
	pend_seq->hal = seq->hal;
	pend_seq->hal_priv = seq->hal_priv;
	pend_seq->fcfi = seq->fcfi;
	pend_seq->status = seq->status;

	/* Fill sequence coalescing and auto xfer ready info */
	pend_seq->tow = seq->tow;
	pend_seq->tow_oox = seq->tow_oox;
	pend_seq->xri = seq->xri;
	pend_seq->hio = seq->hio;

	if (seq->header.data_len) {
		pend_seq->header.data = ocs_malloc(ocs, seq->header.data_len, OCS_M_NOWAIT | OCS_M_ZERO);
		if (!pend_seq->header.data) {
			ocs_log_err(ocs, "header alloc failed to hold pend frame\n");
			goto fail_and_exit;
		}

		pend_seq->header.data_len = seq->header.data_len;
		ocs_memcpy((uint8_t *)pend_seq->header.data, seq->header.data, seq->header.data_len);
		pend_seq->header.tow_buf = seq->header.tow_buf;
	}

	if (seq->payload.data_len) {
		pend_seq->payload.data = ocs_malloc(ocs, seq->payload.data_len, OCS_M_NOWAIT | OCS_M_ZERO);
		if (!pend_seq->payload.data) {
			ocs_log_err(ocs, "payload alloc failed to hold pend frame\n");
			goto fail_and_exit;
		}

		pend_seq->payload.data_len = seq->payload.data_len;
		ocs_memcpy((uint8_t *)pend_seq->payload.data, seq->payload.data, seq->payload.data_len);
		pend_seq->payload.tow_buf = seq->payload.tow_buf;
	}

	return pend_seq;

fail_and_exit:
	frame_printf(ocs, (fc_header_t *)seq->header.data, "Dropping frame\n");
	if (pend_seq)
		ocs_hal_sequence_free(&ocs->hal, pend_seq);

	return NULL;
}

/**
 * @ingroup unsol
 * @brief Abort either a RQ Pair auto XFER RDY XRI.
 * @return Returns None.
 */
void
ocs_port_owned_abort(ocs_t *ocs, ocs_hal_io_t *hio)
{
	ocs_hal_rtn_e hal_rc;

	hal_rc = ocs_hal_io_abort(&ocs->hal, hio, FALSE, ocs_unsol_abort_cb, ocs);
	if ((hal_rc == OCS_HAL_RTN_IO_ABORT_IN_PROGRESS) || (hal_rc == OCS_HAL_RTN_IO_PORT_OWNED_ALREADY_ABORTED)) {
		ocs_log_debug(ocs, "already aborted XRI 0x%x\n", hio->indicator);
	} else if (hal_rc != OCS_HAL_RTN_SUCCESS) {
		ocs_log_debug(ocs, "Error aborting XRI 0x%x status %d\n", hio->indicator, hal_rc);
	}
}

/**
 * @ingroup unsol
 * @brief Abort/Error TOW sequence
 *
 * <h3 class="desc">Description</h3>
 * A TOW data frame is dispatched to the node state machine.
 * A TOW command frame is aborted
 *
 * @param node Node that originated the frame.
 * @param seq header/payload sequence buffers
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled.
 */
static int32_t
ocs_port_owned_abort_sequence(ocs_t *ocs, ocs_hal_sequence_t *seq)
{
	int32_t rc = -1;
	ocs_assert(seq->hio, rc);
	ocs_assert(seq->hio->is_port_owned, rc);

	/* Port Owned XRIs are used only for TOW */
	ocs_assert(seq->tow, rc);

	if (seq->hio->tow_buf->data_cqe) {
		ocs_io_t *io = (ocs_io_t *)seq->hio->ul_io;
		ocs_node_t *node;

		if (!io)
			return rc;

		/* When a valid SCSI IO is detected, then node must always be present */
		node = io->node;
		ocs_assert(node, -1);

		/*
		 * If TOW DATA sequence is detected, then a TOW CMD must already been waiting
		 * for the data. Release the corresponding TOW IO.
		 */
		ocs_log_info(ocs, "Release first burst command! XRI 0x%x\n",
			     seq->hio->indicator);
		rc = ocs_node_recv_tow_data(node, seq);
	} else if (seq->hio->tow_buf->cmd_cqe) {
		ocs_log_info(ocs, "Abort TOW CMD port owned XRI 0x%x\n", seq->hio->indicator);
		ocs_port_owned_abort(ocs, seq->hio);

		ocs_hal_sequence_free(&ocs->hal, seq);
		rc = 0;
	}

	return rc;
}

/**
 * @ingroup unsol
 * @brief Handle unsolicited FC frames.
 *
 * <h3 class="desc">Description</h3>
 * This function is called from the HAL with unsolicited FC frames (FCP, ELS, BLS, etc.).
 *
 * @param arg Application-specified callback data.
 * @param seq Header/payload sequence buffers.
 *
 * @return Returns 0 on success; or a negative error value on failure.
 */

int32_t
ocs_unsolicited_cb(void *arg, ocs_hal_sequence_t *seq)
{
	ocs_t *ocs = arg;
	ocs_xport_t *xport = ocs->xport;
	int32_t rc;

	CPUTRACE("");

	if (ocs->rq_threads == 0) {
		rc = ocs_unsol_process(ocs, seq);
	} else {
		/* use the ox_id to dispatch this IO to a thread */
		fc_header_t *hdr = seq->header.data;
		uint32_t ox_id =  ocs_be16toh(hdr->ox_id);
		uint32_t thr_index = ox_id % ocs->rq_threads;

		rc = ocs_cbuf_put(xport->rq_thread_info[thr_index].seq_cbuf, seq);
	}

	if (rc) {
		ocs_hal_sequence_free(&ocs->hal, seq);
	}

	return 0;
}

/**
 * @ingroup unsol
 * @brief Handle unsolicited FC frames.
 *
 * <h3 class="desc">Description</h3>
 * This function is called either from ocs_unsolicited_cb() or ocs_unsol_rq_thread().
 *
 * @param ocs Pointer to the ocs structure.
 * @param seq Header/payload sequence buffers.
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */
static int32_t
ocs_unsol_process(ocs_t *ocs, ocs_hal_sequence_t *seq)
{
	ocs_xport_fcfi_t *xport_fcfi = NULL;
	ocs_domain_t *domain;
	uint8_t seq_fcfi = seq->fcfi;

	/* HAL_WORKAROUND_OVERRIDE_FCFI_IN_SRB */
	if (ocs->hal.workaround.override_fcfi) {
		if (ocs->hal.first_domain_idx > -1) {
			seq_fcfi = ocs->hal.first_domain_idx;
		}
	}

	/* Range check seq->fcfi */
	if (seq_fcfi < ARRAY_SIZE(ocs->xport->fcfi)) {
		xport_fcfi = &ocs->xport->fcfi[seq_fcfi];
	}

	/* If the transport FCFI entry is NULL, then drop the frame */
	if (xport_fcfi == NULL) {
		ocs_log_test_ratelimited(ocs, "FCFI %d is not valid, dropping frame\n", seq->fcfi);
		if (seq->hio != NULL) {
			ocs_port_owned_abort(ocs, seq->hio);
		}

		ocs_hal_sequence_free(&ocs->hal, seq);
		return 0;
	}
	domain = ocs_hal_domain_get(&ocs->hal, seq_fcfi);

	/*
	 * If we are holding frames or the domain is not yet registered or
	 * there's already frames on the pending list,
	 * then add the new frame to pending list
	 */
	if (domain == NULL ||
	    xport_fcfi->hold_frames ||
	    !ocs_list_empty(&xport_fcfi->pend_frames)) {
		ocs_hal_sequence_t *pend_seq;

		if (ocs_hal_io_port_owned(seq->hio)) {
			if (ocs_port_owned_abort_sequence(ocs, seq)) {
				ocs_hal_sequence_free(&ocs->hal, seq);
			}

			return 0;
		}

		if (!seq->pend_frame_seq) {
			pend_seq = ocs_pend_frame_seq_alloc(ocs, seq);
			if (pend_seq == NULL)
				return -1;

			/* return original sequence buffer to HW to process upcomming frames */
			ocs_hal_sequence_free(&ocs->hal, seq);
		} else {
			pend_seq = seq;
		}

		ocs_lock(&xport_fcfi->pend_frames_lock);
			ocs_atomic_add_return(&ocs->hal.pend_frames_count, 1);
			ocs_list_add_tail(&xport_fcfi->pend_frames, pend_seq);
			if (ocs_atomic_read(&ocs->hal.pend_frames_count) >= OCS_PEND_FRAMES_WATERMARK) {
				ocs_log_err_ratelimited(ocs, "domain pending frames(%d) reached threshold level\n",
							ocs_atomic_read(&ocs->hal.pend_frames_count));
				if (domain)
					ocs_log_err_ratelimited(ocs, "domain(%s) current_state:%s\n",
								domain->display_name,
								(char *)domain->sm.current_state);
			}
		ocs_unlock(&xport_fcfi->pend_frames_lock);

		if (domain != NULL) {
			/* immediately process pending frames */
			ocs_domain_process_pending(domain);
		}
	} else {
		/*
		 * We are not holding frames and pending list is empty, just process frame.
		 * A non-zero return means the frame was not handled - so cleanup
		 */
		if (ocs_domain_dispatch_frame(domain, seq)) {
			if (ocs_hal_io_port_owned(seq->hio)) {
				if (ocs_port_owned_abort_sequence(ocs, seq)) {
					ocs_hal_sequence_free(&ocs->hal, seq);
				}
			} else {
				ocs_hal_sequence_free(&ocs->hal, seq);
			}
		}
	}

	return 0;
}

/**
 * @ingroup unsol
 * @brief Process pending frames queued to the given node.
 *
 * <h3 class="desc">Description</h3>
 * Frames that are queued for the \c node are dispatched and returned
 * to the RQ.
 *
 * @param node Node of the queued frames that are to be dispatched.
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */

int32_t
ocs_process_node_pending(ocs_node_t *node)
{
	ocs_t *ocs = node->ocs;
	ocs_hal_sequence_t *seq = NULL;
	uint32_t pend_frames_processed = 0;

	for (;;) {
		/* need to check for hold frames condition after each frame processed
		 * because any given frame could cause a transition to a state that
		 * holds frames
		 */
		if (ocs_node_frames_held(node)) {
			break;
		}

		/* Get next frame/sequence */
		ocs_lock(&node->pend_frames_lock);
			seq = ocs_list_remove_head(&node->pend_frames);
			if (seq == NULL) {
				pend_frames_processed = node->pend_frames_processed;
				node->pend_frames_processed = 0;
				ocs_unlock(&node->pend_frames_lock);
				break;
			}
			ocs_atomic_sub_return(&ocs->hal.pend_frames_count, 1);
			node->pend_frames_processed++;
		ocs_unlock(&node->pend_frames_lock);

		/* now dispatch frame(s) to dispatch function */
		if (ocs_node_dispatch_frame(node, seq)) {
			if (seq->hio != NULL) {
				ocs_port_owned_abort(ocs, seq->hio);
			}
			ocs_hal_sequence_free(&ocs->hal, seq);
		}
	}

	if (pend_frames_processed)
		node_printf(node, "%u node frames held and processed\n", pend_frames_processed);

	return 0;
}

/**
 * @ingroup unsol
 * @brief Process pending frames queued to the given domain.
 *
 * <h3 class="desc">Description</h3>
 * Frames that are queued for the \c domain are dispatched and
 * returned to the RQ.
 *
 * @param domain Domain of the queued frames that are to be
 *		 dispatched.
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */

int32_t
ocs_domain_process_pending(ocs_domain_t *domain)
{
	ocs_t *ocs = domain->ocs;
	ocs_xport_fcfi_t *xport_fcfi;
	ocs_hal_sequence_t *seq = NULL;
	uint32_t pend_frames_processed = 0;

	ocs_assert(domain->fcf_indicator < SLI4_MAX_FCFI, -1);
	xport_fcfi = &ocs->xport->fcfi[domain->fcf_indicator];

	for (;;) {
		/* need to check for hold frames condition after each frame processed
		 * because any given frame could cause a transition to a state that
		 * holds frames
		 */
		if (ocs_domain_frames_held(domain)) {
			break;
		}

		/* Get next frame/sequence */
		ocs_lock(&xport_fcfi->pend_frames_lock);
			seq = ocs_list_remove_head(&xport_fcfi->pend_frames);
			if (seq == NULL) {
				pend_frames_processed = xport_fcfi->pend_frames_processed;
				xport_fcfi->pend_frames_processed = 0;
				ocs_unlock(&xport_fcfi->pend_frames_lock);
				break;
			}
			ocs_atomic_sub_return(&ocs->hal.pend_frames_count, 1);
			xport_fcfi->pend_frames_processed++;
		ocs_unlock(&xport_fcfi->pend_frames_lock);

		/* now dispatch frame(s) to dispatch function */
		if (ocs_domain_dispatch_frame(domain, seq)) {
			if (seq->hio != NULL) {
				ocs_port_owned_abort(ocs, seq->hio);
			}
			ocs_hal_sequence_free(&ocs->hal, seq);
		}
	}

	if (pend_frames_processed)
		ocs_log_debug(ocs, "%u domain frames held and processed\n", pend_frames_processed);

	return 0;
}

/**
 * @ingroup unsol
 * @brief Purge given pending list
 *
 * <h3 class="desc">Description</h3>
 * Frames that are queued on the given pending list are
 * discarded and returned to the RQ.
 *
 * @param ocs Pointer to ocs object.
 * @param pend_list Pending list to be purged.
 * @param list_lock Lock that protects pending list.
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */

static int32_t
ocs_purge_pending(ocs_t *ocs, ocs_list_t *pend_list, ocs_lock_t *list_lock)
{
	ocs_hal_sequence_t *frame;

	for (;;) {
		frame = ocs_frame_next(pend_list, list_lock);
		if (frame == NULL) {
			break;
		}

		frame_printf(ocs, (fc_header_t*) frame->header.data, "Discarding held frame\n");
		if (frame->hio != NULL) {
			ocs_port_owned_abort(ocs, frame->hio);
		}
		ocs_hal_sequence_free(&ocs->hal, frame);
	}

	return 0;
}

/**
 * @ingroup unsol
 * @brief Purge node's pending (queued) frames.
 *
 * <h3 class="desc">Description</h3>
 * Frames that are queued for the \c node are discarded and returned
 * to the RQ.
 *
 * @param node Node of the queued frames that are to be discarded.
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */

int32_t
ocs_node_purge_pending(ocs_node_t *node)
{
	return ocs_purge_pending(node->ocs, &node->pend_frames, &node->pend_frames_lock);
}

/**
 * @ingroup unsol
 * @brief Purge xport's pending (queued) frames.
 *
 * <h3 class="desc">Description</h3>
 * Frames that are queued for the \c xport are discarded and
 * returned to the RQ.
 *
 * @param domain Pointer to domain object.
 *
 * @return Returns 0 on success; or a negative error value on failure.
 */

int32_t
ocs_domain_purge_pending(ocs_domain_t *domain)
{
	ocs_t *ocs = domain->ocs;
	ocs_xport_fcfi_t *xport_fcfi;

	ocs_assert(domain->fcf_indicator < SLI4_MAX_FCFI, -1);
	xport_fcfi = &ocs->xport->fcfi[domain->fcf_indicator];
	return ocs_purge_pending(domain->ocs,
				 &xport_fcfi->pend_frames,
				 &xport_fcfi->pend_frames_lock);
}

/**
 * @ingroup unsol
 * @brief Check if node's pending frames are held.
 *
 * @param arg Node for which the pending frame hold condition is
 * checked.
 *
 * @return Returns 1 if node is holding pending frames, or 0
 * if not.
 */

static uint8_t
ocs_node_frames_held(void *arg)
{
	ocs_node_t *node = (ocs_node_t *)arg;
	return node->hold_frames;
}

/**
 * @ingroup unsol
 * @brief Check if domain's pending frames are held.
 *
 * @param arg Domain for which the pending frame hold condition is
 * checked.
 *
 * @return Returns 1 if domain is holding pending frames, or 0
 * if not.
 */

static uint8_t
ocs_domain_frames_held(void *arg)
{
	ocs_domain_t *domain = (ocs_domain_t *)arg;
	ocs_t *ocs;
	ocs_xport_fcfi_t *xport_fcfi;

	ocs_assert(domain != NULL, 1);
	ocs_assert(domain->fcf_indicator < SLI4_MAX_FCFI, 1);

	ocs = domain->ocs;
	ocs_assert(ocs, 1);
	ocs_assert(ocs->xport, 1);

	xport_fcfi = &ocs->xport->fcfi[domain->fcf_indicator];
	return xport_fcfi->hold_frames;
}

/**
 * @ingroup unsol
 * @brief Globally (at xport level) hold unsolicited frames.
 *
 * <h3 class="desc">Description</h3>
 * This function places a hold on processing unsolicited FC
 * frames queued to the xport pending list.
 *
 * @param domain Pointer to domain object.
 *
 * @return Returns None.
 */

void
ocs_domain_hold_frames(ocs_domain_t *domain)
{
	ocs_t *ocs = domain->ocs;
	ocs_xport_fcfi_t *xport_fcfi;

	ocs_assert(domain->fcf_indicator < SLI4_MAX_FCFI);
	xport_fcfi = &ocs->xport->fcfi[domain->fcf_indicator];
	if (!xport_fcfi->hold_frames) {
		ocs_log_debug(domain->ocs, "hold frames set for FCFI %d\n", domain->fcf_indicator);
		xport_fcfi->hold_frames = 1;
	}
}

/**
 * @ingroup unsol
 * @brief Clear hold on unsolicited frames.
 *
 * <h3 class="desc">Description</h3>
 * This function clears the hold on processing unsolicited FC
 * frames queued to the domain pending list.
 *
 * @param domain Pointer to domain object.
 *
 * @return Returns None.
 */

void
ocs_domain_accept_frames(ocs_domain_t *domain)
{
	ocs_t *ocs = domain->ocs;
	ocs_xport_fcfi_t *xport_fcfi;

	ocs_assert(domain->fcf_indicator < SLI4_MAX_FCFI);
	xport_fcfi = &ocs->xport->fcfi[domain->fcf_indicator];
	if (xport_fcfi->hold_frames == 1)
		ocs_log_debug(domain->ocs, "hold frames cleared for FCFI %d\n", domain->fcf_indicator);

	xport_fcfi->hold_frames = 0;
	ocs_domain_process_pending(domain);
}


/**
 * @ingroup unsol
 * @brief Dispatch unsolicited FC frame.
 *
 * <h3 class="desc">Description</h3>
 * This function processes an unsolicited FC frame queued at the
 * domain level.
 *
 * @param arg Pointer to ocs object.
 * @param seq Header/payload sequence buffers.
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled.
 */

static __inline int32_t
ocs_domain_dispatch_frame(void *arg, ocs_hal_sequence_t *seq)
{
	ocs_domain_t *domain = (ocs_domain_t *)arg;
	ocs_t *ocs = domain->ocs;
	fc_header_t *hdr;
	uint32_t s_id;
	uint32_t d_id;
	ocs_node_t *node = NULL;
	ocs_sport_t *sport = NULL;

	ocs_assert(seq->header.data, -1);
	ocs_assert(seq->payload.data, -1);
	hdr = seq->header.data;

	/* extract the s_id and d_id */
	s_id = fc_be24toh(hdr->s_id);
	d_id = fc_be24toh(hdr->d_id);

	sport = domain->sport;
	if (sport == NULL) {
		frame_printf_ratelimited(ocs, hdr, "phy sport for FC ID 0x%06x is NULL, dropping frame\n", d_id);
		return -1;
	}

	if (sport->fc_id != d_id) {
		/* Not a physical port IO lookup sport associated with the npiv port */
		sport = ocs_sport_find(domain, d_id); /* Look up without lock */
		if (sport == NULL || (sport->async_flush_state != 0)) {
			if (hdr->type == FC_TYPE_FCP) {
				/* Drop frame */
				ocs_log_warn_ratelimited(ocs, "unsolicited FCP frame with invalid d_id x%x, dropping\n", d_id);
				return -1;
			} else {
				/* p2p will use this case */
				sport = domain->sport;
			}
		}
	}

	/* Lookup the node given the remote s_id */
	node = ocs_node_lookup_get(sport, s_id);

	/* If not found, then create a new node */
	if (node == NULL) {
		/* If this is solicited data or control based on R_CTL and there is no node context,
		 * then we can drop the frame
		 */
		if ((hdr->r_ctl == FC_RCTL_FC4_DATA) && (
		    (hdr->info == FC_RCTL_INFO_SOL_DATA) || (hdr->info == FC_RCTL_INFO_SOL_CTRL))) {
			ocs_log_debug_ratelimited(ocs, "solicited data/ctrl frame without node, dropping\n");
			return -1;
		}

		/* Alloc node and set up init state atomically to avoid missing
 		 * a node shutdown event when sport is on the shutdown path.
		 */
		ocs_device_lock(ocs);
			node = ocs_node_alloc(sport, s_id, FALSE, FALSE);
			if (node == NULL) {
				ocs_device_unlock(ocs);
				ocs_log_err_ratelimited(ocs, "ocs_node_alloc() failed\n");
				return -1;
			}
			/* don't send PLOGI on ocs_d_init entry */
			ocs_node_init_device(node, FALSE);
		ocs_device_unlock(ocs);
	}

	/* Flush has been triggered on this node and will be freed, drop the frame */
	if (node->async_flush_state != 0) {
		ocs_hal_sequence_free(&ocs->hal, seq);
		return 0;
	}

	if (node->hold_frames || !ocs_list_empty((&node->pend_frames))) {
		ocs_hal_sequence_t *pend_seq;

		/*
		 * Drop the FC command frame that is received when remote node being shutdown.
		 * Determine the node shutdown state based on io_alloc_enabled flag.
		 */
		if ((hdr->r_ctl == FC_RCTL_FC4_DATA) && !ocs_scsi_io_alloc_enabled(node)) {
			return -1;
		}

		if (!seq->pend_frame_seq) {
			pend_seq = ocs_pend_frame_seq_alloc(ocs, seq);
			if (pend_seq == NULL)
				return -1;

			/* return original sequence buffer to HW to process upcomming frames */
			ocs_hal_sequence_free(&ocs->hal, seq);
		} else {
			pend_seq = seq;
		}

		frame_printf(ocs, hdr, "Holding frame node[%s] curr_state: %s curr_event: %s\n",
			     node->display_name, node->current_state_name, ocs_sm_event_name(node->current_evt));

		/* add frame to node's pending list */
		ocs_lock(&node->pend_frames_lock);
			ocs_atomic_add_return(&ocs->hal.pend_frames_count, 1);
			ocs_list_add_tail(&node->pend_frames, pend_seq);
			if (ocs_atomic_read(&ocs->hal.pend_frames_count) > OCS_PEND_FRAMES_WATERMARK) {
				node_printf_ratelimited(node, "node pending frames(%d) reached threshold level\n",
							ocs_atomic_read(&ocs->hal.pend_frames_count));
				node_printf_ratelimited(node, "current state: %s previous state: %s\n",
							node->current_state_name, node->prev_state_name);
			}
		ocs_unlock(&node->pend_frames_lock);

		if (!node->hold_frames)
			ocs_process_node_pending(node);

		return 0;
	}

	/* now dispatch frame to the node frame handler */
	return ocs_node_dispatch_frame(node, seq);
}

/**
 * @ingroup unsol
 * @brief Dispatch a frame.
 *
 * <h3 class="desc">Description</h3>
 * A frame is dispatched from the \c node to the handler.
 *
 * @param arg Node that originated the frame.
 * @param seq Header/payload sequence buffers.
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled.
 */
static int32_t
ocs_node_dispatch_frame(void *arg, ocs_hal_sequence_t *seq)
{
	fc_header_t *hdr = seq->header.data;
	uint32_t port_id;
	ocs_node_t *node = (ocs_node_t *)arg;
	int32_t rc = -1;
	int32_t sit_set = 0;

	port_id = fc_be24toh(hdr->s_id);
	ocs_assert(port_id == node->rnode.fc_id, -1);

	if (fc_be24toh(hdr->f_ctl) & FC_FCTL_END_SEQUENCE) {
		/*if SIT is set */
		if (fc_be24toh(hdr->f_ctl) & FC_FCTL_SEQUENCE_INITIATIVE) {
			sit_set = 1;
		}
		switch (hdr->r_ctl) {
		case FC_RCTL_ELS: {
			uint8_t *buf = seq->payload.data;

			/* FPIN notification can be received
			 * without SIT set as it won't expect any further reply
			 */
			if (sit_set || (buf[0] == FC_ELS_CMD_FPIN)) {
				rc = ocs_node_recv_els_frame(node, seq);
			} else {
				node_printf_ratelimited(node, "ELS frame received without SIT "
					"Dropping frame hdr = %08x %08x %08x %08x %08x %08x\n",
						ocs_htobe32(((uint32_t *)hdr)[0]),
						ocs_htobe32(((uint32_t *)hdr)[1]),
						ocs_htobe32(((uint32_t *)hdr)[2]),
						ocs_htobe32(((uint32_t *)hdr)[3]),
						ocs_htobe32(((uint32_t *)hdr)[4]),
						ocs_htobe32(((uint32_t *)hdr)[5]));
			}
			break;
		}
		case FC_RCTL_BLS:
			rc = ocs_node_recv_bls_frame(node, seq);
			break;

		case FC_RCTL_FC4_DATA:
			switch(hdr->type) {
			case FC_TYPE_FCP:
				if (hdr->info == FC_RCTL_INFO_UNSOL_CMD) {
					if (node->fcp_enabled) {
						if (sit_set) {
							rc = ocs_dispatch_fcp_cmd(node, seq);
						} else {
							rc = ocs_dispatch_tow_fcp_cmd(node, seq);
						}
					} else {
						rc = ocs_node_recv_fcp_cmd(node, seq);
					}
				} else if (hdr->info == FC_RCTL_INFO_SOL_DATA) {
					if (sit_set) {
						rc = ocs_dispatch_fcp_data(node, seq);
					}
				}
				break;
			case FC_TYPE_GS:
				if (sit_set) {
					rc = ocs_node_recv_ct_frame(node, seq);
				}
				break;
			default:
				break;
			}
			break;
		}
	} else {
		node_printf_ratelimited(node, "Dropping frame hdr = %08x %08x %08x %08x %08x %08x\n",
			    ocs_htobe32(((uint32_t *)hdr)[0]),
			    ocs_htobe32(((uint32_t *)hdr)[1]),
			    ocs_htobe32(((uint32_t *)hdr)[2]),
			    ocs_htobe32(((uint32_t *)hdr)[3]),
			    ocs_htobe32(((uint32_t *)hdr)[4]),
			    ocs_htobe32(((uint32_t *)hdr)[5]));
	}
	return rc;
}

/**
 * @ingroup unsol
 * @brief Dispatch unsolicited FCP frames (RQ Pair).
 *
 * <h3 class="desc">Description</h3>
 * Dispatch unsolicited FCP frames (called from the device node state machine).
 *
 * @param io Pointer to the IO context.
 * @param task_management_flags Task management flags from the FCP_CMND frame.
 * @param node Node that originated the frame.
 * @param lun 64-bit LUN from FCP_CMND frame.
 *
 * @return Returns None.
 */

static void
ocs_dispatch_unsolicited_tmf(ocs_io_t *io, uint8_t task_management_flags, ocs_node_t *node, uint64_t lun)
{
	uint32_t i;
	struct {
		uint32_t mask;
		ocs_scsi_tmf_cmd_e cmd;
	} tmflist[] = {
		{FCP_QUERY_TASK_SET,		OCS_SCSI_TMF_QUERY_TASK_SET},
		{FCP_ABORT_TASK_SET,		OCS_SCSI_TMF_ABORT_TASK_SET},
		{FCP_CLEAR_TASK_SET,		OCS_SCSI_TMF_CLEAR_TASK_SET},
		{FCP_QUERY_ASYNCHRONOUS_EVENT,	OCS_SCSI_TMF_QUERY_ASYNCHRONOUS_EVENT},
		{FCP_LOGICAL_UNIT_RESET,	OCS_SCSI_TMF_LOGICAL_UNIT_RESET},
		{FCP_TARGET_RESET,		OCS_SCSI_TMF_TARGET_RESET},
		{FCP_CLEAR_ACA,			OCS_SCSI_TMF_CLEAR_ACA}};

	io->exp_xfer_len = 0; /* BUG 32235 */

	for (i = 0; i < ARRAY_SIZE(tmflist); i ++) {
		if (tmflist[i].mask & task_management_flags) {
			io->scsi_info->tmf_cmd = tmflist[i].cmd;
			ocs_log_info(io->ocs, "Process Unsol TMF cmd %d lun %"PRIu64"\n",
				     tmflist[i].cmd, lun);
			ocs_scsi_recv_tmf(io, lun, tmflist[i].cmd, NULL, 0);
			break;
		}
	}

	if (i == ARRAY_SIZE(tmflist)) {
		/* Not handled */
		node_printf_ratelimited(node, "TMF x%x rejected\n", task_management_flags);
		ocs_scsi_send_tmf_resp(io, OCS_SCSI_TMF_FUNCTION_REJECTED, NULL, ocs_fc_tmf_rejected_cb, NULL);
	}
}

static int32_t
ocs_validate_fcp_cmd(ocs_t *ocs, ocs_hal_sequence_t *seq)
{
	fcp_cmnd_iu_t *cmnd = seq->payload.data;
	size_t exp_payload_len = sizeof(fcp_cmnd_iu_t) - 16 + cmnd->additional_fcp_cdb_length;
	fc_header_t	*fchdr = seq->header.data;

	/* fcp_cmnd_iu_t defines fcp_cdb_and_dl may contain upto 16 bytes of CDB */
	if ((cmnd->additional_fcp_cdb_length * sizeof(uint32_t)) > 16) {
		ocs_log_err(ocs, "dropping ox_id %04x with additional_cdb_len (%d) greater than expected\n",
			      ocs_be16toh(fchdr->ox_id), cmnd->additional_fcp_cdb_length);
		return -1;
	}

	/*
	 * If we received less than FCP_CMND_IU bytes, assume that the frame is
	 * corrupted in some way and drop it. This was seen when jamming the FCTL
	 * fill bytes field.
	 */
	if (seq->payload.data_len < exp_payload_len) {
		ocs_log_debug_ratelimited(ocs, "dropping ox_id %04x with payload length (%u) less than expected (%zd)\n",
					  ocs_be16toh(fchdr->ox_id), seq->payload.data_len, exp_payload_len);
		return -1;
	}

	return 0;
}

static void
ocs_populate_io_fcp_cmd(ocs_io_t *io, fcp_cmnd_iu_t *cmnd, fc_header_t *fchdr, uint8_t sit)
{
	uint32_t	*fcp_dl;
	io->init_task_tag = ocs_be16toh(fchdr->ox_id);
	/* note, tgt_task_tag, hw_tag  set when HAL io is allocated */
	fcp_dl = (uint32_t*)(&(cmnd->fcp_cdb_and_dl));
	fcp_dl += cmnd->additional_fcp_cdb_length;
	io->exp_xfer_len = ocs_be32toh(*fcp_dl);
	io->transferred = 0;

	/* The upper 7 bits of CS_CTL is the frame priority thru the SAN.
	 * Our assertion here is, the priority given to a frame containing
	 * the FCP cmd should be the priority given to ALL frames contained
	 * in that IO. Thus we need to save the incoming CS_CTL here.
	 */
	if (fc_be24toh(fchdr->f_ctl) & FC_FCTL_PRIORITY_ENABLE) {
		io->cs_ctl = fchdr->cs_ctl;
	} else {
		io->cs_ctl = 0;
	}
	io->seq_init = sit;
}

static uint32_t
ocs_get_flags_fcp_cmd(fcp_cmnd_iu_t *cmnd)
{
	uint32_t flags = 0;
	switch (cmnd->task_attribute) {
	case FCP_TASK_ATTR_SIMPLE:
		flags |= OCS_SCSI_CMD_SIMPLE;
		break;
	case FCP_TASK_ATTR_HEAD_OF_QUEUE:
		flags |= OCS_SCSI_CMD_HEAD_OF_QUEUE;
		break;
	case FCP_TASK_ATTR_ORDERED:
		flags |= OCS_SCSI_CMD_ORDERED;
		break;
	case FCP_TASK_ATTR_ACA:
		flags |= OCS_SCSI_CMD_ACA;
		break;
	case FCP_TASK_ATTR_UNTAGGED:
		flags |= OCS_SCSI_CMD_UNTAGGED;
		break;
	}
	if (cmnd->wrdata)
		flags |= OCS_SCSI_CMD_DIR_IN;
	if (cmnd->rddata)
		flags |= OCS_SCSI_CMD_DIR_OUT;

	return flags;
}

/**
 * @ingroup unsol
 * @brief Dispatch unsolicited FCP_CMND frame.
 *
 * <h3 class="desc">Description</h3>
 * Dispatch unsolicited FCP_CMND frame. RQ Pair mode - always
 * used for RQ Pair mode since first burst is not supported.
 *
 * @param node Node that originated the frame.
 * @param seq Header/payload sequence buffers.
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled and RX buffers need
 * to be returned.
 */
static int32_t
ocs_dispatch_fcp_cmd(ocs_node_t *node, ocs_hal_sequence_t *seq)
{
	ocs_t *ocs = node->ocs;
	fc_header_t	*fchdr = seq->header.data;
	fcp_cmnd_iu_t	*cmnd = NULL;
	ocs_io_t	*io = NULL;
	fc_vm_header_t	*vhdr;
	uint8_t		df_ctl;
	uint16_t	ox_id;
	uint64_t	lun = ULONG_MAX;
	int32_t		rc = 0;

	ocs_assert(seq->payload.data, -1);
	cmnd = (fcp_cmnd_iu_t *)seq->payload.data;
	ox_id = ocs_be16toh(fchdr->ox_id);

	/* Perform FCP_CMND validation check(s) */
	if (ocs_validate_fcp_cmd(ocs, seq))
		return -1;

	/* If lun is invalid, let the backend handle it */
	lun = ocs_fc_decode_lun(node, cmnd->fcp_lun);

	io = ocs_scsi_io_alloc(node, OCS_SCSI_IO_ROLE_RESPONDER, ocs_hal_get_eq_idx(seq));

	if (io == NULL) {
		uint32_t send_frame_capable;

		/* If we have SEND_FRAME capability, then use it to send task set full or busy */
		rc = ocs_hal_get(&ocs->hal, OCS_HAL_SEND_FRAME_CAPABLE, &send_frame_capable);
		if ((rc == 0) && send_frame_capable) {
			rc = ocs_sframe_send_scsi_status(node, seq, SCSI_STATUS_TASK_SET_FULL, NULL, 0);
			node_printf_ratelimited(node, "IO allocation failed ox_id %04x. Send TASK_SET_FULL or BUSY\n",
						ox_id);
			if (rc)
				ocs_log_test_ratelimited(ocs, "ocs_sframe_send_scsi_status() failed: %d\n", rc);

			return rc;
		}

		ocs_log_err_ratelimited(ocs, "IO allocation failed ox_id %04x\n", ox_id);
		return -1;
	}

	io->hal_priv = seq->hal_priv;

	/* Check if the CMD has vmheader */
	io->app_id = 0;
	df_ctl = fchdr->df_ctl;
	if (sli_feature_enabled(&ocs->hal.sli, SLI4_FEATURE_ASHDR) && (df_ctl & FC_DFCTL_DEVICE_HDR_16_MASK)) {
		uint32_t vmhdr_offset = 0;

		/* Presence of VMID. Get the vm header offset. */
		if (df_ctl & FC_DFCTL_ESP_HDR_MASK) {
			vmhdr_offset += FC_DFCTL_ESP_HDR_SIZE;
			ocs_log_err_ratelimited(ocs, "ESP Header present. Fix ESP Size.\n");
		}

		if (df_ctl & FC_DFCTL_NETWORK_HDR_MASK)
			vmhdr_offset += FC_DFCTL_NETWORK_HDR_SIZE;

		vhdr = (fc_vm_header_t *) ((char *)fchdr + sizeof(fc_header_t) + vmhdr_offset);
		io->app_id = ocs_be32toh(vhdr->src_vmid);
	}

	/* RQ pair, if we got here, SIT=1 */
	ocs_populate_io_fcp_cmd(io, cmnd, fchdr, TRUE);

	if (cmnd->task_management_flags) {
		ocs_dispatch_unsolicited_tmf(io, cmnd->task_management_flags, node, lun);
	} else {
		uint32_t flags = ocs_get_flags_fcp_cmd(cmnd);

		ocs_scsi_io_set_proc_ticks(io, OCS_SCSI_IO_PROC_CMD_RECVD_TSTART);

		/* can return failure for things like task set full and UAs,
		 * no need to treat as a dropped frame if rc != 0
		 */
		rc = ocs_scsi_recv_cmd(io, lun, cmnd->fcp_cdb,
					sizeof(cmnd->fcp_cdb) +
					(cmnd->additional_fcp_cdb_length * sizeof(uint32_t)),
					flags);
		if (rc == 0) {
			ocs_scsi_io_set_proc_ticks(io, OCS_SCSI_IO_PROC_CMD_RECVD_TFIN);
			ocs_scsi_io_proc_check_time(io, OCS_SCSI_IO_PROC_CMD_RECVD_TSTART,
							OCS_SCSI_IO_PROC_CMD_RECVD_TFIN);
		}
	}

	/* successfully processed, now return RX buffer to the chip */
	ocs_hal_sequence_free(&ocs->hal, seq);
	return 0;
}

/**
 * @ingroup unsol
 * @brief Dispatch unsolicited FCP_CMND frame.
 *
 * <h3 class="desc">Description</h3>
 * Dispatch unsolicited FCP_CMND frame that is assisted with TOW.
 *
 * @param node Node that originated the frame.
 * @param seq Header/payload sequence buffers.
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled and RX buffers need
 * to be returned.
 */
static int32_t
ocs_dispatch_tow_fcp_cmd(ocs_node_t *node, ocs_hal_sequence_t *seq)
{
	ocs_t *ocs = node->ocs;
	fc_header_t	*fchdr = seq->header.data;
	fcp_cmnd_iu_t	*cmnd = NULL;
	ocs_io_t	*io = NULL;
	uint16_t	ox_id;
	uint64_t	lun = ULONG_MAX;
	int32_t		rc = 0;

	ocs_assert(seq->payload.data, -1);
	cmnd = (fcp_cmnd_iu_t *)seq->payload.data;
	ox_id = ocs_be16toh(fchdr->ox_id);

	/* Make sure this is a valid TOW frame */
	if (!seq->tow) {
		node_printf_ratelimited(node, "IO isn't optimized write assisted, dropping FCP_CMND ox_id %04x\n",
					ox_id);
		return -1;
	}

	/* Perform FCP_CMND validation check(s) */
	if (ocs_validate_fcp_cmd(ocs, seq))
		return -1;

	/* If lun is invalid, let the backend handle it */
	lun = ocs_fc_decode_lun(node, cmnd->fcp_lun);

	io = ocs_scsi_io_alloc(node, OCS_SCSI_IO_ROLE_RESPONDER, ocs_hal_get_eq_idx(seq));
	if (io == NULL) {
		uint32_t send_frame_capable;

		/* If we have SEND_FRAME capability, then use it to send task set full or busy */
		rc = ocs_hal_get(&ocs->hal, OCS_HAL_SEND_FRAME_CAPABLE, &send_frame_capable);
		if ((rc == 0) && send_frame_capable) {
			rc = ocs_sframe_send_scsi_status(node, seq, SCSI_STATUS_TASK_SET_FULL, NULL, 0);
			node_printf_ratelimited(node, "IO allocation failed ox_id %04x. Send TASK_SET_FULL or BUSY\n",
						ox_id);
			if (rc)
				ocs_log_test_ratelimited(ocs, "ocs_sframe_send_scsi_status() failed: %d\n", rc);

			return rc;
		}

		ocs_log_err_ratelimited(ocs, "IO allocation failed ox_id %04x\n", ox_id);
		return -1;
	}

	io->hal_priv = seq->hal_priv;

	/* RQ pair, if we got here, SIT=0 */
	ocs_populate_io_fcp_cmd(io, cmnd, fchdr, FALSE);

	if (cmnd->task_management_flags) {
		/* first burst command better not be a TMF */
		ocs_log_err_ratelimited(ocs, "TMF flags set 0x%x\n", cmnd->task_management_flags);
		ocs_scsi_io_free(io);
		return -1;
	} else {
		uint32_t flags = ocs_get_flags_fcp_cmd(cmnd);

		/* activate HAL IO */
		io->hio = ocs_hal_io_activate_port_owned(&ocs->hal, seq->hio);
		if (!io->hio) {
			ocs_log_err_ratelimited(ocs, "Port owned HIO activation failed ox_id %04x\n", ox_id);
			ocs_scsi_io_free(io);
			return -1;
		}

		seq->hio->ul_io = io;
		io->tgt_task_tag = seq->hio->indicator;

		/* Note: Data buffers are received in another call */
		ocs_scsi_recv_cmd_first_burst(io, lun, cmnd->fcp_cdb,
					      sizeof(cmnd->fcp_cdb) +
					      (cmnd->additional_fcp_cdb_length * sizeof(uint32_t)),
					      flags, NULL, 0);
	}

	/* FCP_CMND processed, return RX buffer to the chip */
	ocs_hal_sequence_free(&ocs->hal, seq);
	return 0;
}

/**
 * @ingroup unsol
 * @brief Dispatch FCP data frames for TOW.
 *
 * <h3 class="desc">Description</h3>
 * Dispatch unsolicited FCP data frames
 * containing sequence initiative transferred (SIT=1).
 *
 * @param node Node that originated the frame.
 * @param seq Header/payload sequence buffers.
 *
 * @return Returns 0 if frame processed and RX buffers cleaned
 * up appropriately, -1 if frame not handled.
 */

static int32_t
ocs_dispatch_fcp_data(ocs_node_t *node, ocs_hal_sequence_t *seq)
{
	ocs_t *ocs = node->ocs;
	ocs_hal_io_t *hio;
	ocs_io_t *io;
	ocs_dma_t fburst[1];
	int32_t rc = -1;

	if (!seq->tow) {
		fc_header_t *hdr = seq->header.data;
		uint16_t ox_id = ocs_be16toh(hdr->ox_id);
		uint32_t send_frame_capable = 0;

		ocs_assert((fc_be24toh(hdr->f_ctl) & FC_FCTL_END_SEQUENCE), -1);
		ocs_assert((fc_be24toh(hdr->f_ctl) & FC_FCTL_SEQUENCE_INITIATIVE), -1);

		ocs_hal_get(&ocs->hal, OCS_HAL_SEND_FRAME_CAPABLE, &send_frame_capable);
		if (!send_frame_capable) {
			ocs_log_err_ratelimited(node->ocs, "No Send frame capability\n");
			return rc;
		}

		if (node->first_burst && ocs_tgt_scsi_enabled(ocs) && !ocs_tgt_nvme_enabled(ocs)) {
			rc = ocs_sframe_send_scsi_status(node, seq, SCSI_STATUS_BUSY, NULL, 0);
			if (rc)
				ocs_log_err_ratelimited(node->ocs, "Failed to send SCSI busy resp for ox_id %04x\n",
							ox_id);

			node_printf_ratelimited(node, "IO isn't optimized write assisted, %s data frame ox_id %04x\n",
						rc ? "dropping" : "sent SCSI busy resp for", ox_id);
		} else {
			node_printf_ratelimited(node, "Dropping firstburst data\n");
		}

		return rc;
	}

	ocs_assert(seq->payload.data, -1);
	hio = seq->hio;
	ocs_assert(hio, -1);

	io = hio->ul_io;
	if (io == NULL) {
		ocs_log_err_ratelimited(ocs, "data received for NULL io, xri=0x%x\n", hio->indicator);
		return -1;
	}

	/*
	 * We only support data completions for TOW. Make sure this is a port owned XRI.
	 */
	if (!ocs_hal_io_port_owned(seq->hio)) {
		ocs_log_err_ratelimited(ocs, "data received for host owned XRI, xri=0x%x\n", hio->indicator);
		return -1;
	}

	/* For error statuses, pass the error to the target back end */
	if (seq->status != OCS_HAL_UNSOL_SUCCESS) {
		ocs_log_err_ratelimited(ocs, "data with status 0x%x received, xri=0x%x\n", seq->status, hio->indicator);

		/*
		 * In this case, there is an existing, in-use HAL IO that
		 * first may need to be aborted. Then, the backend will be
		 * notified of the error while waiting for the data.
		 */
		ocs_port_owned_abort(ocs, seq->hio);

		/*
		 * HAL IO has already been allocated and is waiting for data.
		 * Need to tell backend that an error has occurred.
		 */
		ocs_scsi_recv_cmd_first_burst(io, 0, NULL, 0, OCS_SCSI_FIRST_BURST_ERR, NULL, 0);
		return -1;
	}

	/* sequence initiative has been transferred */
	io->seq_init = 1;

	/* convert the array of pointers to the correct type, to send to backend */
	fburst[0].virt = seq->payload.data;
	fburst[0].len = seq->payload.data_len;
	fburst[0].size = seq->payload.data_len;

	/* the amount of first burst data was saved as "acculated sequence length" */
	io->transferred = seq->payload.data_len;

	if (ocs_scsi_recv_cmd_first_burst(io, 0, NULL, 0, 0, fburst, io->transferred))
		ocs_log_err_ratelimited(ocs, "error passing first burst, xri=0x%x, ox_id=0x%x\n",
					hio->indicator, io->init_task_tag);

	/* Free the header and all the accumulated payload buffers */
	ocs_hal_sequence_free(&ocs->hal, seq);
	return 0;
}


/**
 * @ingroup unsol
 * @brief Handle the callback for the TMF FUNCTION_REJECTED response.
 *
 * <h3 class="desc">Description</h3>
 * Handle the callback of a send TMF FUNCTION_REJECTED response request.
 *
 * @param io Pointer to the IO context.
 * @param scsi_status Status of the response.
 * @param flags Callback flags.
 * @param arg Callback argument.
 *
 * @return Returns 0 on success, or a negative error value on failure.
 */

static int32_t
ocs_fc_tmf_rejected_cb(ocs_io_t *io, ocs_scsi_io_status_e scsi_status, uint32_t flags, void *arg)
{
	ocs_scsi_io_free(io);
	return 0;
}

/**
 * @brief Return next FC frame on node->pend_frames list
 *
 * The next FC frame on the node->pend_frames list is returned, or NULL
 * if the list is empty.
 *
 * @param pend_list Pending list to be purged.
 * @param list_lock Lock that protects pending list.
 *
 * @return Returns pointer to the next FC frame, or NULL if the pending frame list
 * is empty.
 */
ocs_hal_sequence_t *
ocs_frame_next(ocs_list_t *pend_list, ocs_lock_t *list_lock)
{
	ocs_hal_sequence_t *frame = NULL;

	ocs_lock(list_lock);
		frame = ocs_list_remove_head(pend_list);
	ocs_unlock(list_lock);
	return frame;
}

/**
 * @brief Process send fcp response frame callback
 *
 * The function is called when the send FCP response posting has completed. Regardless
 * of the outcome, the sequence is freed.
 *
 * @param arg Pointer to originator frame sequence.
 * @param cqe Pointer to completion queue entry.
 * @param status Status of operation.
 *
 * @return None.
 */
static void
ocs_sframe_common_send_cb(void *arg, uint8_t *cqe, int32_t status)
{
	ocs_hal_send_frame_context_t *ctx = arg;
	ocs_hal_t *hal = ctx->hal;

	/* Remove the request from the list */
	ocs_lock(&hal->send_frame_wqe_lock);
	ocs_list_remove(&hal->send_frame_wqe_list, ctx);
	ocs_unlock(&hal->send_frame_wqe_lock);

	/* Free WQ completion callback */
	ocs_hal_reqtag_free(hal, ctx->wqcb);

	/* Call upper level callback if any */
	if (ctx->callback) {
		ctx->callback(hal, status, ctx->cb_arg);
	}

	/* Free sequence */
	ctx->wqe.wqebuf = NULL;
	if (ctx->hio)
		ocs_hal_io_free(hal, ctx->hio);

	ocs_dma_free(hal->os, &ctx->payload);
	ocs_free(hal->os, ctx, sizeof(*ctx));
}

/**
 * @brief Send a frame, common code
 *
 * A frame is sent using SEND_FRAME, the R_CTL/F_CTL/TYPE may be specified, the payload is
 * sent as a single frame.
 *
 * Memory resources are allocated from RQ buffers contained in the passed in sequence data.
 *
 * @param node Pointer to node object.
 * @param sframe_args input arguments
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
int32_t
ocs_sframe_common_send(ocs_node_t *node, ocs_sframe_args_t *sframe_args)
{
	ocs_t *ocs = node->ocs;
	ocs_hal_t *hal = &ocs->hal;
	ocs_hal_io_t *hio = NULL;
	ocs_hal_rtn_e rc = 0;
	fc_header_le_t hdr;
	ocs_hal_send_frame_context_t *ctx;

	ctx = ocs_malloc(ocs, sizeof(ocs_hal_send_frame_context_t), OCS_M_NOWAIT | OCS_M_ZERO);
	if (ctx == NULL) {
		ocs_log_err(ocs, "failed to allocate send frame context\n");
		return -1;
	}

	if (sframe_args->payload_len) {
		rc = ocs_dma_alloc(ocs, &ctx->payload, sframe_args->payload_len, OCS_MIN_DMA_ALIGNMENT, OCS_M_NOWAIT);
		if (rc) {
			ocs_log_err(ocs, "failed to alloc DMA memory for sframe\n");
			ocs_free(ocs, ctx, sizeof(*ctx));
			return -1;
		}

		/* Copy the payload in */
		if (ocs_dma_copy_in(&ctx->payload, sframe_args->payload, sframe_args->payload_len) <= 0) {
			ocs_log_err(ocs, "failed to copy the payload buffer to DMA\n");
			ocs_dma_free(ocs, &ctx->payload);
			ocs_free(ocs, ctx, sizeof(*ctx));
			return -1;
		}
	}

	ctx->wqe.wqebuf = ctx->wqebuf;

	ctx->callback = sframe_args->callback;
	ctx->cb_arg = sframe_args->cb_arg;

	/* Build the FC header reusing the RQ header DMA buffer */
	ocs_memset(&hdr, 0, sizeof(hdr));

	/* Send it back to whomever sent it to us */
	hdr.d_id = sframe_args->s_id;
	hdr.r_ctl = sframe_args->r_ctl;
	hdr.info = sframe_args->info;
	hdr.s_id = sframe_args->d_id;
	hdr.cs_ctl = 0;
	hdr.f_ctl = sframe_args->f_ctl;
	hdr.type = sframe_args->type;
	hdr.seq_cnt = 0;
	hdr.df_ctl = 0;

	/*
	 * send_frame_seq_id is an atomic, we just let it increment,
	 * while storing only the low 8 bits to hdr->seq_id
	 */
	hdr.seq_id = (uint8_t)ocs_atomic_add_return(&hal->send_frame_seq_id, 1);

	hdr.ox_id = sframe_args->ox_id;
	hdr.rx_id = sframe_args->rx_id;

	if ((hdr.ox_id == 0xffff) || (hdr.rx_id == 0xffff)) {
		hio = ocs_hal_io_alloc(&ocs->hal);
		if (hio == NULL) {
			ocs_log_err(ocs, "failed to allocate HIO for sframe\n");
			ocs_dma_free(ocs, &ctx->payload);
			ocs_free(ocs, ctx, sizeof(*ctx));
			return -1;
		}
		ctx->hio = hio;

		/* Set OX_ID first in case of ORIGINATOR */
		if (sframe_args->ox_id == 0xffff)
			hdr.ox_id = hio->indicator;
		else
			hdr.rx_id = hio->indicator;
	}

	hdr.parameter = 0;

	ocs_lock(&hal->send_frame_wqe_lock);
	/* Add send frame WQE request to the list */
	ocs_list_add_tail(&hal->send_frame_wqe_list, ctx);
	ocs_unlock(&hal->send_frame_wqe_lock);

	/* Send */
	rc = ocs_hal_send_frame(&ocs->hal, (void*)&hdr, FC_SOFI3, FC_EOFT, &ctx->payload, ctx,
				ocs_sframe_common_send_cb, ctx);
	if (rc) {
		ocs_log_err(ocs, "ocs_hal_send_frame failed: %d\n", rc);
		/* remove the request from the list */
		ocs_lock(&hal->send_frame_wqe_lock);
		ocs_list_remove(&hal->send_frame_wqe_list, ctx);
		ocs_unlock(&hal->send_frame_wqe_lock);
		if (hio)
			ocs_hal_io_free(&ocs->hal, hio);
		ocs_dma_free(ocs, &ctx->payload);
		ocs_free(ocs, ctx, sizeof(*ctx));
	}

	return rc ? -1 : 0;
}

/**
 * @brief Send LOGO using SEND_FRAME
 *
 * The LOGO is send using the SEND_FRAME function.
 *
 * @param node Pointer to node object.
 * @param s_id Source FC ID
 * @param d_id Destination FC ID
 * @param ox_id Originator exchange ID
 * @param rx_id Responder exchange ID
 * @return Returns 0 on success, or a negative error code value on failure.
 */
int32_t
ocs_sframe_send_logo(ocs_node_t *node, uint32_t s_id, uint32_t d_id,
		     uint16_t ox_id, uint16_t rx_id)
{
	ocs_sframe_args_t sframe_args = {0};
	fc_logo_payload_t logo = {0};
	fc_plogi_payload_t *sparams = (fc_plogi_payload_t*)node->sport->service_params;

	sframe_args.r_ctl = FC_RCTL_ELS;
	sframe_args.info  = FC_RCTL_INFO_UNSOL_CTRL;
	sframe_args.f_ctl = FC_FCTL_FIRST_SEQUENCE | FC_FCTL_SEQUENCE_INITIATIVE |
			    FC_FCTL_LAST_SEQUENCE | FC_FCTL_END_SEQUENCE;
	sframe_args.type  = FC_TYPE_EXT_LINK;

	logo.command_code = FC_ELS_CMD_LOGO;
	logo.port_id = fc_htobe24(node->sport->fc_id);
	logo.port_name_hi = sparams->port_name_hi;
	logo.port_name_lo = sparams->port_name_lo;

	sframe_args.payload = &logo;
	sframe_args.payload_len = sizeof(logo);

	/* ocs_sframe_common_send reverses s_id and d_id. */
	sframe_args.s_id = d_id;
	sframe_args.d_id = s_id;
	sframe_args.ox_id = ox_id;
	sframe_args.rx_id = rx_id;

	return ocs_sframe_common_send(node, &sframe_args);
}

/**
 * @brief Send SCSI response
 *
 * Return a SCSI response using send frame.
 *
 * @param node Pointer to node object.
 * @param seq Pointer to originator frame sequence.
 * @param scsi_status SCSI response status.
 * @param sense_data SCSI response sense data.
 * @param sense_data_length SCSI response sense data length.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
static int32_t
ocs_sframe_send_scsi_status(ocs_node_t *node, ocs_hal_sequence_t *seq, uint8_t scsi_status,
			    fixed_sense_data_t *sense_data, size_t sense_data_length)
{
	fcp_cmnd_iu_t *fcpcmd = seq->payload.data;
	fc_header_t *behdr = seq->header.data;
	fcp_rsp_iu_t fcprsp;
	size_t fcprsp_len;
	uint32_t *fcp_dl_ptr;
	uint32_t fcp_dl;
	int32_t rc = 0;
	ocs_sframe_rsp_args_t fcp_rsp_args = {0};
	ocs_t *ocs = node->ocs;

	/* extract FCP_DL from FCP command*/
	fcp_dl_ptr = (uint32_t*)(&(fcpcmd->fcp_cdb_and_dl));
	fcp_dl_ptr += fcpcmd->additional_fcp_cdb_length;
	fcp_dl = ocs_be32toh(*fcp_dl_ptr);

	/* construct FCP response */
	ocs_memset(&fcprsp, 0, sizeof(fcprsp));
	ocs_memset(&fcp_rsp_args, 0, sizeof(fcp_rsp_args));
	fcprsp_len = (sizeof(fcprsp) - sizeof(fcprsp.data));

	switch (scsi_status) {
		case SCSI_STATUS_CHECK_CONDITION:
			fcprsp.scsi_status = scsi_status;

			if (OCS_SCSI_SNS_BUF_VALID(sense_data) && sense_data_length) {
				ocs_assert(sense_data_length <= sizeof(fcprsp.data), -1);
				fcprsp.flags |= FCP_SNS_LEN_VALID;
				ocs_memcpy(fcprsp.data, sense_data, sense_data_length);
				*((uint32_t*)fcprsp.fcp_sns_len) = sense_data_length;
				fcprsp_len += sense_data_length;
			}

			break;

		case SCSI_STATUS_TASK_SET_FULL:
			/* Special case where we need to send either busy or task_set_full */
			ocs_lock(&node->active_ios_lock);
				fcprsp.scsi_status = (ocs_list_empty(&node->active_ios) ?
						      SCSI_STATUS_BUSY : SCSI_STATUS_TASK_SET_FULL);
			ocs_unlock(&node->active_ios_lock);
			break;

		default:
			fcprsp.scsi_status = scsi_status;
			break;
	}

	*((uint32_t*)&fcprsp.fcp_resid) = fcp_dl;

	fcp_rsp_args.s_id = fc_be24toh(behdr->s_id);
	fcp_rsp_args.d_id = fc_be24toh(behdr->d_id);
	fcp_rsp_args.ox_id = ocs_be16toh(behdr->ox_id);
	fcp_rsp_args.rx_id = ocs_be16toh(behdr->rx_id);

	fcp_rsp_args.fcprsp = &fcprsp;
	fcp_rsp_args.fcprsp_len = fcprsp_len;

	/* send it using send_frame */
	rc = ocs_sframe_send_fcp_rsp(node, &fcp_rsp_args);
	if (rc) {
		ocs_log_test(node->ocs, "ocs_sframe_send_fcp_rsp() failed: %d\n", rc);
	} else {
		ocs_hal_sequence_free(&ocs->hal, seq);
	}

	return rc;
}

/**
 * @brief Send LS_ACC using Sendframe
 *
 * A LS_ACC is sent using SEND_FRAME
 *
 * @param node Pointer to node object.
 * @param s_id Source FC ID
 * @param d_id Destination FC ID
 * @param ox_id Originator exchange ID
 * @param rx_id Responder exchange ID
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
int32_t
ocs_sframe_send_ls_acc(ocs_node_t *node, uint32_t s_id, uint32_t d_id,
			uint16_t ox_id, uint16_t rx_id)
{
	fc_ls_acc_payload_t acc = {0};
	ocs_sframe_args_t sframe_args = {0};

	acc.command_code = FC_ELS_CMD_ACC;

	sframe_args.r_ctl = FC_RCTL_ELS;
	sframe_args.info = FC_RCTL_INFO_SOL_CTRL;
	sframe_args.f_ctl = FC_FCTL_EXCHANGE_RESPONDER | FC_FCTL_LAST_SEQUENCE |
				FC_FCTL_END_SEQUENCE;
	sframe_args.type = FC_TYPE_EXT_LINK;
	sframe_args.payload = &acc;
	sframe_args.payload_len = sizeof(acc);
	sframe_args.s_id = s_id;
	sframe_args.d_id = d_id;
	sframe_args.ox_id = ox_id;
	sframe_args.rx_id = rx_id;

	return ocs_sframe_common_send(node, &sframe_args);
}

/**
 * @brief Send PRLO_ACC using Sendframe
 *
 * A PRLO_ACC is sent using SEND_FRAME
 *
 * @param node Pointer to node object.
 * @param s_id Source FC ID
 * @param d_id Destination FC ID
 * @param ox_id Originator exchange ID
 * @param rx_id Responder exchange ID 
 * @param fc_type FC4 type. 
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
int32_t
ocs_sframe_send_prlo_acc(ocs_node_t *node, uint32_t s_id, uint32_t d_id,
		uint16_t ox_id, uint16_t rx_id, uint8_t fc_type)
{
	fc_prlo_acc_payload_t prlo_acc = {0};
	ocs_sframe_args_t sframe_args = {0};

	prlo_acc.type = fc_type;
	prlo_acc.command_code = FC_ELS_CMD_ACC;
	prlo_acc.page_length = 16;
	prlo_acc.payload_length = ocs_htobe16(sizeof(fc_prlo_acc_payload_t));
	prlo_acc.type_ext = 0;
	prlo_acc.response_code = FC_PRLO_REQUEST_EXECUTED;

	sframe_args.r_ctl = FC_RCTL_ELS;
	sframe_args.info = FC_RCTL_INFO_SOL_CTRL;
	sframe_args.f_ctl = FC_FCTL_EXCHANGE_RESPONDER | FC_FCTL_LAST_SEQUENCE |
			    FC_FCTL_END_SEQUENCE;
	sframe_args.type = FC_TYPE_EXT_LINK;
	sframe_args.payload = &prlo_acc;
	sframe_args.payload_len = sizeof(prlo_acc);
	sframe_args.s_id = s_id;
	sframe_args.d_id = d_id;
	sframe_args.ox_id = ox_id;
	sframe_args.rx_id = rx_id;

	return ocs_sframe_common_send(node, &sframe_args);
}

/**
 * @brief Send BA_ACC using sent frame
 *
 * A BA_ACC is sent using SEND_FRAME
 *
 * @param node Pointer to node object.
 * @param seq Pointer to originator frame sequence.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
int32_t
ocs_sframe_send_bls_acc(ocs_node_t *node,  ocs_hal_sequence_t *seq)
{
	fc_header_t *behdr = seq->header.data;
	uint16_t ox_id = ocs_be16toh(behdr->ox_id);
	uint16_t rx_id = ocs_be16toh(behdr->rx_id);
	int rc;
	fc_ba_acc_payload_t acc = {0};
	ocs_sframe_args_t sframe_args = {0};
	ocs_t *ocs = node->ocs;
	ocs_hal_t *hal = &ocs->hal;

	acc.ox_id = ocs_htobe16(ox_id);
	acc.rx_id = ocs_htobe16(rx_id);
	acc.low_seq_cnt = UINT16_MAX;
	acc.high_seq_cnt = UINT16_MAX;

	sframe_args.r_ctl = FC_RCTL_BLS;
	sframe_args.info  = FC_RCTL_INFO_UNSOL_DATA;
	sframe_args.f_ctl = FC_FCTL_EXCHANGE_RESPONDER | FC_FCTL_LAST_SEQUENCE |
				FC_FCTL_END_SEQUENCE;
	sframe_args.type  = FC_TYPE_BASIC_LINK;
	sframe_args.payload = &acc;
	sframe_args.payload_len = sizeof(acc);

	sframe_args.s_id = fc_be24toh(behdr->s_id);
	sframe_args.d_id = fc_be24toh(behdr->d_id);
	sframe_args.ox_id = ox_id;
	sframe_args.rx_id = rx_id;

	rc = ocs_sframe_common_send(node, &sframe_args);
	if (!rc)
		ocs_hal_sequence_free(hal, seq);

	return rc;
}

/**
 * @brief Send ABTS using send frame
 *
 * @param node Pointer to node object.
 * @param seq Pointer to originator frame sequence.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
int32_t
ocs_sframe_send_abts(ocs_node_t *node,  ocs_hal_sequence_t *seq)
{
	fc_header_t *behdr = seq->header.data;
	uint16_t ox_id = ocs_be16toh(behdr->ox_id);
	uint16_t rx_id = ocs_be16toh(behdr->rx_id);
	int rc;
	ocs_sframe_args_t sframe_args = {0};
	ocs_t *ocs = node->ocs;
	ocs_hal_t *hal = &ocs->hal;

	sframe_args.r_ctl = FC_RCTL_BLS;
	sframe_args.info  = FC_RCTL_INFO_SOL_DATA;
	sframe_args.f_ctl = FC_FCTL_EXCHANGE_RESPONDER | FC_FCTL_SEQUENCE_INITIATIVE | FC_FCTL_END_SEQUENCE;
	sframe_args.type  = FC_TYPE_BASIC_LINK;

	sframe_args.s_id = fc_be24toh(behdr->s_id);
	sframe_args.d_id = fc_be24toh(behdr->d_id);
	sframe_args.ox_id = ox_id;
	sframe_args.rx_id = rx_id;

	rc = ocs_sframe_common_send(node, &sframe_args);
	if (!rc)
		ocs_hal_sequence_free(hal, seq);

	return rc;
}

/**
 * @brief Send BA_RJT using sent frame
 *
 * A BA_RJT is sent using SEND_FRAME
 *
 * @param node Pointer to node object.
 * @param seq Pointer to originator frame sequence.
 *
 * @return Returns 0 on success, or a negative error code value on failure.
 */
int32_t
ocs_sframe_send_bls_rjt(ocs_node_t *node,  ocs_hal_sequence_t *seq)
{
	int rc;
	fc_ba_rjt_payload_t rjt = {0};
	ocs_sframe_args_t sframe_args = {0};
	ocs_t *ocs = node->ocs;
	ocs_hal_t *hal = &ocs->hal;
	fc_header_t *behdr = seq->header.data;

	rjt.reason_code		= FC_REASON_UNABLE_TO_PERFORM;
	rjt.reason_explanation	= FC_EXPL_NO_ADDITIONAL;

	sframe_args.r_ctl = FC_RCTL_BLS;
	sframe_args.info  = FC_RCTL_INFO_DATA_DESC;
	sframe_args.f_ctl = FC_FCTL_EXCHANGE_RESPONDER | FC_FCTL_LAST_SEQUENCE |
				FC_FCTL_END_SEQUENCE;
	sframe_args.type  = FC_TYPE_BASIC_LINK;
	sframe_args.payload = &rjt;
	sframe_args.payload_len = sizeof(rjt);

	sframe_args.s_id = fc_be24toh(behdr->s_id);
	sframe_args.d_id = fc_be24toh(behdr->d_id);
	sframe_args.ox_id = ocs_be16toh(behdr->ox_id);
	sframe_args.rx_id = ocs_be16toh(behdr->rx_id);

	rc = ocs_sframe_common_send(node, &sframe_args);
	if (!rc)
		ocs_hal_sequence_free(hal, seq);

	return rc;
}
