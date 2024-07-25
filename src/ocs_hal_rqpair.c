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
 * This file contains code related to operating in RQ Pair mode.
 *
 */

#include "ocs.h"
#include "ocs_hal.h"

/* Uncomment this to turn on RQ debug */
// #define ENABLE_DEBUG_RQBUF

static int32_t ocs_hal_rqpair_find(ocs_hal_t *hal, uint16_t rq_id);
static ocs_hal_sequence_t * ocs_hal_rqpair_get(ocs_hal_t *hal, uint16_t rqindex, uint16_t bufindex);
static int32_t ocs_hal_rqpair_put(ocs_hal_t *hal, ocs_hal_sequence_t *seq);
static ocs_hal_rtn_e ocs_hal_rqpair_tow_buffer_sequence_reset(ocs_hal_t *hal, ocs_hal_sequence_t *seq);

static void
ocs_pause_rq(ocs_hal_t *hal, uint16_t rqindex)
{
	hal_rq_t *rq;

	rq = hal->hal_rq[hal->hal_rq_lookup[rqindex]];
	if (hal->rq_buf_drop.ms_to_sleep) {
		rq->t_rq_thread_pause_start = hal->rq_buf_drop.rq_thread_pause_start;
		if (rq->t_rq_thread_pause_start && rq->t_rq_thread_pause_start > ocs_get_os_ticks())
			return;

		ocs_log_info(hal->os, "pausing rq(%d) thread at %lu for %lu ms\n",
			     rqindex, ocs_get_os_ticks(), hal->rq_buf_drop.ms_to_sleep);
		ocs_msleep(hal->rq_buf_drop.ms_to_sleep);
		rq->t_rq_thread_pause_start = 0;
		ocs_log_info(hal->os, "resuming rq(%d) thread at %lu\n", rqindex, ocs_get_os_ticks());
		hal->rq_buf_drop.rq_pause_running = false;
		hal->rq_buf_drop.ms_to_sleep = 0;
	}
}

/**
 * @brief Process receive queue completions for RQ Pair mode.
 *
 * @par Description
 * RQ completions are processed. In RQ pair mode, a single header and single payload
 * buffer are received, and passed to the function that has registered for unsolicited
 * callbacks.
 *
 * @param hal Hardware context.
 * @param cq Pointer to HAL completion queue.
 * @param cqe Completion queue entry.
 *
 * @return Returns 0 for success, or a negative error code value for failure.
 */

int32_t
ocs_hal_rqpair_process_rq(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe)
{
	uint16_t rq_id;
	uint32_t index;
	int32_t rqindex;
	int32_t	 rq_status;
	uint32_t h_len;
	uint32_t p_len;
	ocs_hal_sequence_t *seq;
	uint8_t code = cqe[SLI4_CQE_CODE_OFFSET];

	/* TODO: Merge this function with TOW handler */
	rq_status = sli_fc_rqe_rqid_and_index(&hal->sli, cqe, &rq_id, &index);

	/* If possible, get the rqindex before checking for rq_status */
	rqindex = ocs_hal_rqpair_find(hal, rq_id);
	if (rqindex < 0) {
		ocs_log_err(hal->os, "rq_status=%#x: rq_id lookup failed for id=%#x\n", rq_status, rq_id);
		return -1;
	}

	if (rq_status) {
		sli4_fc_async_rcqe_t *rcqe = (sli4_fc_async_rcqe_t *)cqe;
		hal_rq_t *rq = hal->hal_rq[hal->hal_rq_lookup[rqindex]];

		ocs_log_err(hal->os, "RCQE status=%#x\n", rq_status);

		switch (rq_status) {
		case SLI4_FC_ASYNC_RQ_BUF_LEN_EXCEEDED:
			/* Get the RQ buffer */
			seq = ocs_hal_rqpair_get(hal, rqindex, index);
			ocs_hal_assert(seq != NULL);

			/* Dump the frame header and payload buffers */
			frame_printf(hal->os, (fc_header_t *)seq->header.data, "Dropping frame\n");
			ocs_dump32(OCS_DEBUG_ALWAYS, hal->os, "RCQE header", seq->header.data,
				   OCS_MIN(rq->hdr_entry_size, rcqe->header_data_placement_length));
			ocs_dump32(OCS_DEBUG_ALWAYS, hal->os, "RCQE payload", seq->payload.data,
				   OCS_MIN(rq->data_entry_size, rcqe->payload_data_placement_length));

			/* Return RQ buffer to the chip */
			if (ocs_hal_rqpair_sequence_free(hal, seq))
				ocs_log_err(hal->os, "failed to return buffers to RQ\n");

			break;

		case SLI4_FC_ASYNC_RQ_INSUFF_BUF_NEEDED:
			/* Since RQ buffers were not consumed, cannot return them to the chip */
			rq->rq_empty_warn_count++;
			break;

		case SLI4_FC_ASYNC_RQ_INSUFF_BUF_FRM_DISC:
			/* Since RQ buffers were not consumed, cannot return them to the chip */
			rq->rq_empty_err_count++;
			break;

		case SLI4_FC_ASYNC_RQ_DMA_FAILURE:
			/* Get the RQ buffer */
			seq = ocs_hal_rqpair_get(hal, rqindex, index);
			ocs_hal_assert(seq != NULL);

			/* Return RQ buffer to the chip */
			if (ocs_hal_rqpair_sequence_free(hal, seq))
				ocs_log_err(hal->os, "failed to return buffers to RQ\n");

			break;

		default:
			break;
		}

		return -1;
	}

	/* Pause RQ feature can be only enabled in non-interrupt context */
	if (!ocs_in_interrupt_context() && hal->rq_buf_drop.enable)
		ocs_pause_rq(hal, rqindex);

	OCS_STAT({ hal_rq_t *rq = hal->hal_rq[hal->hal_rq_lookup[rqindex]]; rq->use_count++; rq->hdr_use_count++;
		 rq->payload_use_count++;})

	seq = ocs_hal_rqpair_get(hal, rqindex, index);
	ocs_hal_assert(seq != NULL);

	if (code == SLI4_CQE_CODE_MARKER) {
		sli4_fc_marker_rcqe_t *rq_marker_cqe = (sli4_fc_marker_rcqe_t *) cqe;

		/* If this marker was generated by SCSI, process it. */
		if (rq_marker_cqe->tag_higher & SLI4_RQ_MARKER_TYPE_SCSI) {
			ocs_hal_rq_marker_handle_cmpl(hal, rq_marker_cqe->tag_lower);
		}

		if (ocs_hal_rqpair_sequence_free(hal, seq))
			ocs_log_err(hal->os, "status=%#x, failed to return marker buffers to RQ\n", rq_status);

		return 0;
	}

	seq->hal = hal;
	seq->tow = 0;
	seq->tow_oox = 0;
	seq->xri = 0;
	seq->hio = NULL;

	sli_fc_rqe_length(&hal->sli, cqe, &h_len, &p_len);
	seq->header.data_len = h_len;
	seq->payload.data_len = p_len;
	seq->fcfi = sli_fc_rqe_fcfi(&hal->sli, cqe);
	seq->hal_priv = cq->eq;

	/* bounce enabled, single RQ, we snoop the ox_id to choose the cpuidx */
	if (hal->config.bounce) {
		fc_header_t *hdr = seq->header.data;
		uint32_t s_id = fc_be24toh(hdr->s_id);
		uint32_t d_id = fc_be24toh(hdr->d_id);
		uint32_t ox_id =  ocs_be16toh(hdr->ox_id);
		if (hal->callback.bounce != NULL) {
			(*hal->callback.bounce)(ocs_hal_unsol_process_bounce, seq, s_id, d_id, ox_id);
		}
	} else {
		hal->callback.unsolicited(hal->args.unsolicited, seq);
	}

	return 0;
}

/**
 * @brief Process receive queue completions for RQ Pair mode - Auto xfer rdy
 *
 * @par Description
 * RQ completions are processed. In RQ pair mode, a single header and single payload
 * buffer are received, and passed to the function that has registered for unsolicited
 * callbacks.
 *
 * @param hal Hardware context.
 * @param cq Pointer to HAL completion queue.
 * @param cqe Completion queue entry.
 *
 * @return Returns 0 for success, or a negative error code value for failure.
 */
int32_t
ocs_hal_rqpair_tow_cmd_process(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe)
{
	/* Seems silly to call a SLI function to decode - use the structure directly for performance */
	sli4_fc_optimized_write_cmd_cqe_t *opt_wr = (sli4_fc_optimized_write_cmd_cqe_t*)cqe;
	uint16_t rq_id;
	uint32_t index;
	int32_t rqindex;
	int32_t	 rq_status;
	uint32_t h_len;
	uint32_t p_len;
	ocs_hal_sequence_t *seq;
	uint8_t tow_lock_taken = 0;

	rq_status = sli_fc_rqe_rqid_and_index(&hal->sli, cqe, &rq_id, &index);
	if (0 != rq_status) {
		switch (rq_status) {
		case SLI4_FC_ASYNC_RQ_BUF_LEN_EXCEEDED:
		case SLI4_FC_ASYNC_RQ_DMA_FAILURE:
			/* just get RQ buffer then return to chip */
			rqindex = ocs_hal_rqpair_find(hal, rq_id);
			if (rqindex < 0) {
				ocs_log_err(hal->os, "status=%#x: rq_id lookup failed for id=%#x\n",
					    rq_status, rq_id);
				break;
			}

			/* get RQ buffer */
			seq = ocs_hal_rqpair_get(hal, rqindex, index);
			ocs_hal_assert(seq != NULL);

			/* return to chip */
			if (ocs_hal_rqpair_sequence_free(hal, seq)) {
				ocs_log_err(hal->os, "status=%#x, failed to return buffers to RQ\n", rq_status);
				break;
			}
			break;
		case SLI4_FC_ASYNC_RQ_INSUFF_BUF_NEEDED:
		case SLI4_FC_ASYNC_RQ_INSUFF_BUF_FRM_DISC:
			/* since RQ buffers were not consumed, cannot return them to chip */
			ocs_log_debug(hal->os, "Warning: RCQE status=%#x\n", rq_status);
			FALL_THROUGH; /* fall through */
		default:
			break;
		}
		return -1;
	}

	rqindex = ocs_hal_rqpair_find(hal, rq_id);
	if (rqindex < 0) {
		ocs_log_err(hal->os, "Error: rq_id lookup failed for id=%#x\n", rq_id);
		return -1;
	}

	OCS_STAT({ hal_rq_t *rq = hal->hal_rq[hal->hal_rq_lookup[rqindex]]; rq->use_count++; rq->hdr_use_count++;
		 rq->payload_use_count++;})

	seq = ocs_hal_rqpair_get(hal, rqindex, index);
	ocs_hal_assert(seq != NULL);

	seq->hal = hal;
	seq->tow = opt_wr->tow;
	seq->tow_oox = opt_wr->oox;
	seq->xri = opt_wr->xri;
	seq->hio = NULL;

	sli_fc_rqe_length(&hal->sli, cqe, &h_len, &p_len);
	seq->header.data_len = h_len;
	seq->payload.data_len = p_len;
	seq->fcfi = sli_fc_rqe_fcfi(&hal->sli, cqe);
	seq->hal_priv = cq->eq;

	if (seq->tow) {
		fc_header_t *fc_hdr = seq->header.data;

		seq->hio = ocs_hal_io_lookup(hal, seq->xri);
		ocs_lock(&seq->hio->tow_lock);
		tow_lock_taken = 1;

		/*
		 * Save the FCFI, src_id, dest_id and ox_id because we
		 * need it for the sequence object when the data comes.
		 */
		seq->hio->tow_buf->fcfi = seq->fcfi;
		seq->hio->tow_buf->seq.fcfi = seq->fcfi;
		seq->hio->tow_buf->hdr.ox_id = fc_hdr->ox_id;
		seq->hio->tow_buf->hdr.s_id = fc_hdr->s_id;
		seq->hio->tow_buf->hdr.d_id = fc_hdr->d_id;
		seq->hio->tow_buf->cmd_cqe = 1;

		/*
		 * For First burst command, the sequence initiative is always unset.
		 * For Auto Xfer_Rdy command, the sequence initiative is always set.
		 *
		 * Since Auto Xfer_Rdy optimized write has been done for this IO,
		 * and to fall into the same first burst path, clear the SIT bit
		 * here so that the upper layer will wait for the data completions.
		 */
		fc_hdr->f_ctl &= fc_htobe24(~FC_FCTL_SEQUENCE_INITIATIVE);

		/* If TOW CMD CQE came before previous TRSP CQE of same XRI */
		if (seq->hio->type == OCS_HAL_IO_TARGET_RSP) {
			seq->hio->tow_buf->call_tow_cmd = TRUE;
			seq->hio->tow_buf->cmd_seq = seq;
			goto exit;
		}
	}

	/* bounce enabled, single RQ, we snoop the ox_id to choose the cpuidx */
	if (hal->config.bounce) {
		fc_header_t *hdr = seq->header.data;
		uint32_t s_id = fc_be24toh(hdr->s_id);
		uint32_t d_id = fc_be24toh(hdr->d_id);
		uint32_t ox_id =  ocs_be16toh(hdr->ox_id);
		if (hal->callback.bounce != NULL) {
			(*hal->callback.bounce)(ocs_hal_unsol_process_bounce, seq, s_id, d_id, ox_id);
		}
	} else {
		hal->callback.unsolicited(hal->args.unsolicited, seq);
	}

	if (seq->tow) {
		/* If data cqe came before cmd cqe in out of order in case of TOW */
		if (seq->hio->tow_buf->data_cqe == 1) {
			/* bounce enabled, single RQ, we snoop the ox_id to choose the cpuidx */
			if (hal->config.bounce) {
				fc_header_t *hdr = seq->header.data;
				uint32_t s_id = fc_be24toh(hdr->s_id);
				uint32_t d_id = fc_be24toh(hdr->d_id);
				uint32_t ox_id =  ocs_be16toh(hdr->ox_id);
				if (hal->callback.bounce != NULL) {
					(*hal->callback.bounce)(ocs_hal_unsol_process_bounce, &seq->hio->tow_buf->seq, s_id, d_id, ox_id);
				}
			} else {
				hal->callback.unsolicited(hal->args.unsolicited, &seq->hio->tow_buf->seq);
			}
		}
	}

exit:
	if (tow_lock_taken)
		ocs_unlock(&seq->hio->tow_lock);

	return 0;
}

/**
 * @brief Process CQ completions for Auto xfer rdy data phases.
 *
 * @par Description
 * The data is DMA'd into the data buffer posted to the SGL prior to the XRI
 * being assigned to an IO. When the completion is received, All of the data
 * is in the single buffer.
 *
 * @param hal Hardware context.
 * @param cq Pointer to HAL completion queue.
 * @param cqe Completion queue entry.
 *
 * @return Returns 0 for success, or a negative error code value for failure.
 */
int32_t
ocs_hal_rqpair_tow_data_process(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe)
{
	/* Seems silly to call a SLI function to decode - use the structure directly for performance */
	sli4_fc_optimized_write_data_cqe_t *opt_wr = (sli4_fc_optimized_write_data_cqe_t*)cqe;
	ocs_hal_sequence_t *seq;
	ocs_hal_io_t *io;
	ocs_hal_tow_buffer_t *buf;

	/* Look up the IO */
	io = ocs_hal_io_lookup(hal, opt_wr->xri);
	ocs_lock(&io->tow_lock);
	buf = io->tow_buf;
	buf->data_cqe = 1;
	seq = &buf->seq;
	seq->hal = hal;
	seq->tow = TRUE;
	seq->tow_oox = 0;
	seq->xri = opt_wr->xri;
	seq->hio = io;
	seq->header.data = buf->header.dma.virt;
	seq->payload.data = buf->payload.dma.virt;
	seq->header.rqindex = buf->header.rqindex;
	seq->header.tow_buf = buf->header.dma.alloc;
	seq->payload.rqindex = buf->payload.rqindex;

	seq->header.data_len = sizeof(fc_header_t);
	seq->payload.data_len = opt_wr->total_data_placed;

	seq->fcfi = buf->fcfi;
	seq->hal_priv = cq->eq;

	if (opt_wr->status == SLI4_FC_WCQE_STATUS_SUCCESS) {
		seq->status = OCS_HAL_UNSOL_SUCCESS;
	} else if (opt_wr->status == SLI4_FC_WCQE_STATUS_REMOTE_STOP) {
		seq->status = OCS_HAL_UNSOL_ABTS_RCVD;
	} else {
		seq->status = OCS_HAL_UNSOL_ERROR;
	}

	/* If TOW CMD CQE came before previous TRSP CQE of same XRI */
	if (io->type == OCS_HAL_IO_TARGET_RSP) {
		io->tow_buf->call_tow_data = TRUE;
		goto exit;
	}

	if (!buf->cmd_cqe) {
		/* if data cqe came before cmd cqe, return here, cmd cqe will handle */
		goto exit;
	}

	/* bounce enabled, single RQ, we snoop the ox_id to choose the cpuidx */
	if (hal->config.bounce) {
		fc_header_t *hdr = seq->header.data;
		uint32_t s_id = fc_be24toh(hdr->s_id);
		uint32_t d_id = fc_be24toh(hdr->d_id);
		uint32_t ox_id =  ocs_be16toh(hdr->ox_id);
		if (hal->callback.bounce != NULL) {
			(*hal->callback.bounce)(ocs_hal_unsol_process_bounce, seq, s_id, d_id, ox_id);
		}
	} else {
		hal->callback.unsolicited(hal->args.unsolicited, seq);
	}

exit:
	ocs_unlock(&io->tow_lock);
	return 0;
}

/**
 * @brief Return pointer to RQ buffer entry.
 *
 * @par Description
 * Returns a pointer to the RQ buffer entry given by @c rqindex and @c bufindex.
 *
 * @param hal Hardware context.
 * @param rqindex Index of the RQ that is being processed.
 * @param bufindex Index into the RQ that is being processed.
 *
 * @return Pointer to the sequence structure, or NULL otherwise.
 */
static ocs_hal_sequence_t *
ocs_hal_rqpair_get(ocs_hal_t *hal, uint16_t rqindex, uint16_t bufindex)
{
	sli4_queue_t *rq_hdr = hal->rq[rqindex];
	ocs_hal_sequence_t *seq = NULL;
	hal_rq_t *rq = hal->hal_rq[hal->hal_rq_lookup[rqindex]];

#if defined(ENABLE_DEBUG_RQBUF)
	sli4_queue_t *rq_payload = hal->rq[rqindex+1];
	uint64_t rqbuf_debug_value = 0xdead0000 | ((rq->id & 0xf) << 12) | (bufindex & 0xfff);
#endif

	if (bufindex >= rq_hdr->length) {
		ocs_log_err(hal->os, "RQ index %d bufindex %d exceed ring length %d for id %d\n",
			    rqindex, bufindex, rq_hdr->length, rq_hdr->id);
		return NULL;
	}

	/* rq_hdr lock also covers rqindex+1 queue */
	sli_queue_lock(rq_hdr);

#if defined(ENABLE_DEBUG_RQBUF)
	/* Put a debug value into the rq, to track which entries are still valid */
	_sli_queue_poke(&hal->sli, rq_hdr, bufindex, (uint8_t *)&rqbuf_debug_value);
	_sli_queue_poke(&hal->sli, rq_payload, bufindex, (uint8_t *)&rqbuf_debug_value);
#endif

	seq = rq->rq_tracker[bufindex];
	rq->rq_tracker[bufindex] = NULL;

	if (seq == NULL ) {
		ocs_log_err(hal->os, "RQ buffer NULL, rqindex %d, bufindex %d, current q index = %d\n",
			    rqindex, bufindex, rq_hdr->index);
	}

	sli_queue_unlock(rq_hdr);
	return seq;
}

/**
 * @brief Posts an RQ buffer to a queue and update the verification structures
 *
 * @param hal		hardware context
 * @param seq Pointer to sequence object.
 *
 * @return Returns 0 on success, or a non-zero value otherwise.
 */
static int32_t
ocs_hal_rqpair_put(ocs_hal_t *hal, ocs_hal_sequence_t *seq)
{
	sli4_queue_t *rq_hdr = hal->rq[seq->header.rqindex];
	sli4_queue_t *rq_payload = hal->rq[seq->payload.rqindex];
	uint32_t hal_rq_index = hal->hal_rq_lookup[seq->header.rqindex];
	hal_rq_t *rq = hal->hal_rq[hal_rq_index];
	uint32_t     phys_hdr[2];
	uint32_t     phys_payload[2];
	int32_t      qindex_hdr;
	int32_t      qindex_payload;

	rq->rq_buffer_post_counter++;
	if (hal->rq_buf_drop.enable &&
	    (rq->rq_buffer_post_counter >= hal->rq_buf_drop.rq_buffer_drop_trigger) &&
	    (rq->rq_buffer_drop_counter < hal->rq_buf_drop.max_rq_buffer_drop_count)) {
		rq->rq_buffer_drop_counter++;
		rq->rq_buffer_post_counter = 0;
		ocs_log_err(hal->os, "Dropping RQ sequence [%d] :: %p.\n", rq->rq_buffer_drop_counter, seq);
		return 0;
	}

	/* Update the RQ verification lookup tables */
	phys_hdr[0] = ocs_addr32_hi(seq->header.phys_buf);
	phys_hdr[1] = ocs_addr32_lo(seq->header.phys_buf);
	phys_payload[0] = ocs_addr32_hi(seq->payload.phys_buf);
	phys_payload[1] = ocs_addr32_lo(seq->payload.phys_buf);

	/* rq_hdr lock also covers payload / header->rqindex+1 queue */
	sli_queue_lock(rq_hdr);

	/*
	 * Note: The header must be posted last for buffer pair mode because
	 *       posting on the header queue posts the payload queue as well.
	 *       We do not ring the payload queue independently in RQ pair mode.
	 */
	qindex_payload = _sli_queue_write(&hal->sli, rq_payload, (void *)phys_payload);
	qindex_hdr = _sli_queue_write(&hal->sli, rq_hdr, (void *)phys_hdr);

	if (qindex_hdr < 0 || qindex_payload < 0) {
		ocs_log_err(hal->os, "RQ_ID=%#x write failed\n", rq_hdr->id);
		sli_queue_unlock(rq_hdr);
		return OCS_HAL_RTN_ERROR;
	}

	/* ensure the indexes are the same */
	ocs_hal_assert(qindex_hdr == qindex_payload);

	/* Update the lookup table */
	if (rq->rq_tracker[qindex_hdr] == NULL) {
		rq->rq_tracker[qindex_hdr] = seq;
	} else {
		ocs_log_warn(hal->os, "expected rq_tracker[%d][%d] buffer to be NULL\n", hal_rq_index, qindex_hdr);
	}

	sli_queue_unlock(rq_hdr);
	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @brief Return RQ buffers (while in RQ pair mode).
 *
 * @par Description
 * The header and payload buffers are returned to the Receive Queue.
 *
 * @param hal Hardware context.
 * @param seq Header/payload sequence buffers.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS on success, or an error code value on failure.
 */

ocs_hal_rtn_e
ocs_hal_rqpair_sequence_free(ocs_hal_t *hal, ocs_hal_sequence_t *seq)
{
	ocs_hal_rtn_e   rc = OCS_HAL_RTN_SUCCESS;
	ocs_hal_assert(seq);

	/* Free dynamically allocated sequence buffers which hold pending frames */
	if (seq->pend_frame_seq) {
		if (seq->header.data)
			ocs_free(hal->os, seq->header.data, seq->header.data_len);
		if (seq->payload.data)
			ocs_free(hal->os, seq->payload.data, seq->payload.data_len);
		ocs_free(hal->os, seq, sizeof(ocs_hal_sequence_t));

		return rc;
	}
	/* TOW sequences has a dummy index */
	if (seq->header.rqindex == OCS_HAL_RQ_INDEX_DUMMY_HDR) {
		return ocs_hal_rqpair_tow_buffer_sequence_reset(hal, seq);
	}

	/*
	 * Post the data buffer first. Because in RQ pair mode, ringing the
	 * doorbell of the header ring will post the data buffer as well.
	 */
	if (ocs_hal_rqpair_put(hal, seq)) {
		ocs_log_err(hal->os, "error writing buffers\n");
		return OCS_HAL_RTN_ERROR;
	}

	return rc;
}

/**
 * @brief Find the RQ index of RQ_ID.
 *
 * @param hal Hardware context.
 * @param rq_id RQ ID to find.
 *
 * @return Returns the RQ index, or -1 if not found
 */
static inline int32_t
ocs_hal_rqpair_find(ocs_hal_t *hal, uint16_t rq_id)
{
	return ocs_hal_queue_hash_find(hal->rq_hash, rq_id);
}

/**
 * @ingroup devInitShutdown
 * @brief Allocate tow buffers.
 *
 * @par Description
 * Allocates the tow buffers and places them on the free list.
 *
 * @param hal Hardware context allocated by the caller.
 * @param num_buffers Number of buffers to allocate.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_rqpair_tow_buffer_alloc(ocs_hal_t *hal, uint32_t num_buffers)
{
	ocs_hal_tow_buffer_t *buf;
	uint32_t i;

	hal->tow_buffer_pool = ocs_pool_alloc(hal->os, sizeof(ocs_hal_tow_buffer_t), num_buffers, FALSE);
	if (!hal->tow_buffer_pool) {
		ocs_log_err(hal->os, "Failed to allocate tow buffer pool\n");
		return OCS_HAL_RTN_NO_MEMORY;
	}

	for (i = 0; i < num_buffers; i++) {
		/* Allocate the wrapper object */
		buf = ocs_pool_get_instance(hal->tow_buffer_pool, i);
		ocs_hal_assert(buf);

		/* Allocate the tow dma buffer */
		if (ocs_dma_alloc(hal->os, &buf->payload.dma, hal->config.tow_io_size,
				  OCS_MIN_DMA_ALIGNMENT, OCS_M_FLAGS_NONE)) {
			ocs_log_err(hal->os, "Failed to allocate dma buffer\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}

		/* build a fake data header in big endian */
		buf->hdr.info = FC_RCTL_INFO_SOL_DATA;
		buf->hdr.r_ctl = FC_RCTL_FC4_DATA;
		buf->hdr.type = FC_TYPE_FCP;
		buf->hdr.f_ctl = fc_htobe24(FC_FCTL_EXCHANGE_RESPONDER |
					    FC_FCTL_FIRST_SEQUENCE |
					    FC_FCTL_LAST_SEQUENCE |
					    FC_FCTL_END_SEQUENCE |
					    FC_FCTL_SEQUENCE_INITIATIVE);

		/* build the fake header DMA object */
		buf->header.rqindex = OCS_HAL_RQ_INDEX_DUMMY_HDR;
		buf->header.dma.virt = &buf->hdr;
		buf->header.dma.alloc = buf;
		buf->header.dma.size = sizeof(buf->hdr);
		buf->header.dma.len = sizeof(buf->hdr);

		buf->payload.rqindex = OCS_HAL_RQ_INDEX_DUMMY_DATA;
	}

	ocs_log_info(hal->os, "Allocated tow buffer pool\n");
	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup devInitShutdown
 * @brief Post Auto xfer rdy buffers to the XRIs posted with DNRX.
 *
 * @par Description
 * When new buffers are freed, check existing XRIs waiting for buffers.
 *
 * @param hal Hardware context allocated by the caller.
 */
static void
ocs_hal_rqpair_tow_dnrx_check(ocs_hal_t *hal)
{
	ocs_hal_io_t *io;
	int32_t rc;

	ocs_lock(&hal->io_lock);

	while (!ocs_list_empty(&hal->io_port_dnrx)) {
		io = ocs_list_remove_head(&hal->io_port_dnrx);
		rc = ocs_hal_reque_xri(hal, io);
		if(rc) {
			break;
		}
	}

	ocs_unlock(&hal->io_lock);
}

/**
 * @brief Called when the POST_SGL_PAGE command completes.
 *
 * @par Description
 * Free the mailbox command buffer.
 *
 * @param hal Hardware context.
 * @param status Status field from the mbox completion.
 * @param mqe Mailbox response structure.
 * @param arg Pointer to a callback function that signals the caller that the command is done.
 *
 * @return Returns 0.
 */
static int32_t
ocs_hal_rqpair_auto_xfer_rdy_move_to_port_cb(ocs_hal_t *hal, int32_t status, uint8_t *mqe, void  *arg)
{
	ocs_free(hal->os, mqe, SLI4_BMBX_SIZE);
	return 0;
}

/**
 * @brief Prepares an XRI to move to the chip.
 *
 * @par Description
 * Puts the data SGL into the SGL list for the IO object and possibly registers
 * an SGL list for the XRI. Since both the POST_XRI and POST_SGL_PAGES commands are
 * mailbox commands, we don't need to wait for completion before preceding.
 *
 * @param hal Hardware context allocated by the caller.
 * @param io Pointer to the IO object.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS for success, or an error code value for failure.
 */
ocs_hal_rtn_e
ocs_hal_rqpair_tow_move_to_port(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	uint16_t sgl_buf_length = sizeof(sli4_req_fcoe_post_sgl_pages_t) +
				  sizeof(sli4_fcoe_post_sgl_page_desc_t);

	/* We only need to preregister the SGL if it has not yet been done. */
	if (!sli_get_sgl_preregister(&hal->sli)) {
		uint8_t	*post_sgl;
		ocs_dma_t *psgls = &io->def_sgl;
		ocs_dma_t **sgls = &psgls;

		/* non-local buffer required for mailbox queue */
		post_sgl = ocs_malloc(hal->os, sgl_buf_length, OCS_M_NOWAIT);
		if (post_sgl == NULL) {
			ocs_log_err(hal->os, "no buffer for command\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}

		sli_cmd_fcoe_post_sgl_pages(&hal->sli, post_sgl, sgl_buf_length,
					    io->indicator, 1, sgls, NULL);
		if (ocs_hal_command(hal, post_sgl, OCS_CMD_NOWAIT,
				    ocs_hal_rqpair_auto_xfer_rdy_move_to_port_cb, NULL)) {
			ocs_free(hal->os, post_sgl, SLI4_BMBX_SIZE);
			return OCS_HAL_RTN_ERROR;
		}
	}

	ocs_lock(&hal->io_lock);
	if (ocs_hal_rqpair_tow_xri_buffer_attach(hal, io, FALSE) != 0) { /* DNRX set - no buffer */
		ocs_unlock(&hal->io_lock);
		return OCS_HAL_RTN_ERROR;
	}
	ocs_unlock(&hal->io_lock);

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @brief Prepares an XRI to move back to the host.
 *
 * @par Description
 * Releases any attached buffer back to the pool.
 *
 * @param hal Hardware context allocated by the caller.
 * @param io Pointer to the IO object.
 */
void
ocs_hal_rqpair_tow_move_to_host(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	if (io->tow_buf != NULL) {
		ocs_lock(&hal->io_lock);
			/* check list and remove if there */
			if (ocs_list_on_list(&io->dnrx_link)) {
				ocs_list_remove(&hal->io_port_dnrx, io);
				io->tow_dnrx = 0;
				/* release the count for waiting for a buffer */
				ocs_hal_io_free(hal, io);
			}

			ocs_pool_put(hal->tow_buffer_pool, io->tow_buf);
			io->tow_buf = NULL;
		ocs_unlock(&hal->io_lock);

		ocs_hal_rqpair_tow_dnrx_check(hal);
	}
	return;
}

/**
 * @brief Attach an TOW XRI buffer to an IO
 *
 * @par Description
 * Puts the data SGL into the SGL list for the IO object
 * @n @name
 * @b Note: io_lock must be held.
 *
 * @param hal Hardware context allocated by the caller.
 * @param io Pointer to the IO object.
 *
 * @return Returns the value of DNRX bit in the TRSP and ABORT WQEs.
 */
uint8_t
ocs_hal_rqpair_tow_xri_buffer_attach(ocs_hal_t *hal, ocs_hal_io_t *io, int32_t reuse_buf)
{
	ocs_hal_tow_buffer_t *buf;
	sli4_sge_t	*data;

	if (!reuse_buf) {
		buf = ocs_pool_get(hal->tow_buffer_pool);
		io->tow_buf = buf;
	}

	data = io->def_sgl.virt;
	data[0].sge_type = SLI4_SGE_TYPE_SKIP;
	data[0].last = 0;

	ocs_hal_assert(io->tow_buf);

	/*
	 * Note: if we are doing DIF assists, then the SGE[1] must contain the
	 * DI_SEED SGE. The host is responsible for programming:
	 *   SGE Type (Word 2, bits 30:27)
	 *   Replacement App Tag (Word 2 bits 15:0)
	 *   App Tag (Word 3 bits 15:0)
	 *   New Ref Tag (Word 3 bit 23)
	 *   Metadata Enable (Word 3 bit 20)
	 *   Auto-Increment RefTag (Word 3 bit 19)
	 *   Block Size (Word 3 bits 18:16)
	 * The following fields are managed by the SLI Port:
	 *    Ref Tag Compare (Word 0)
	 *    Replacement Ref Tag (Word 1) - In not the LBA
	 *    NA (Word 2 bit 25)
	 *    Opcode RX (Word 3 bits 27:24)
	 *    Checksum Enable (Word 3 bit 22)
	 *    RefTag Enable (Word 3 bit 21)
	 *
	 * The first two SGLs are cleared by ocs_hal_io_init_sges(), so assume eveything is cleared.
	 */
	if (hal->config.tow_p_type) {
		sli4_diseed_sge_t *diseed = (sli4_diseed_sge_t*)&data[1];

		diseed->sge_type = SLI4_SGE_TYPE_DISEED;
		diseed->repl_app_tag = hal->config.tow_app_tag_value;
		diseed->app_tag_cmp = hal->config.tow_app_tag_value;
		diseed->check_app_tag = hal->config.tow_app_tag_valid;
		diseed->auto_incr_ref_tag = TRUE; /* Always the LBA */
		diseed->dif_blk_size = hal->config.tow_blksize_chip;
	} else {
		data[1].sge_type = SLI4_SGE_TYPE_SKIP;
		data[1].last = 0;
	}

	data[2].sge_type = SLI4_SGE_TYPE_DATA;
	data[2].buffer_address_high = ocs_addr32_hi(io->tow_buf->payload.dma.phys);
	data[2].buffer_address_low  = ocs_addr32_lo(io->tow_buf->payload.dma.phys);
	data[2].buffer_length = io->tow_buf->payload.dma.size;
	data[2].last = TRUE;
	data[3].sge_type = SLI4_SGE_TYPE_SKIP;

	return 0;
}

/**
 * @brief Return tow buffers (while in RQ pair mode).
 *
 * @par Description
 * The header and payload buffers are returned to the tow pool.
 *
 * @param hal Hardware context.
 * @param seq Header/payload sequence buffers.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS for success, an error code value for failure.
 */
static ocs_hal_rtn_e
ocs_hal_rqpair_tow_buffer_sequence_reset(ocs_hal_t *hal, ocs_hal_sequence_t *seq)
{
	ocs_hal_tow_buffer_t *buf = seq->header.tow_buf;

	buf->data_cqe = 0;
	buf->cmd_cqe = 0;
	buf->fcfi = 0;
	buf->call_tow_cmd = FALSE;
	buf->call_tow_data = FALSE;

	/* build a fake data header in big endian */
	buf->hdr.info = FC_RCTL_INFO_SOL_DATA;
	buf->hdr.r_ctl = FC_RCTL_FC4_DATA;
	buf->hdr.type = FC_TYPE_FCP;
	buf->hdr.f_ctl = fc_htobe24(FC_FCTL_EXCHANGE_RESPONDER |
					FC_FCTL_FIRST_SEQUENCE |
					FC_FCTL_LAST_SEQUENCE |
					FC_FCTL_END_SEQUENCE |
					FC_FCTL_SEQUENCE_INITIATIVE);
	buf->hdr.ox_id = 0;
	buf->hdr.s_id = 0;
	buf->hdr.d_id = 0;

	/* build the fake header DMA object */
	buf->header.rqindex = OCS_HAL_RQ_INDEX_DUMMY_HDR;
	buf->header.dma.virt = &buf->hdr;
	buf->header.dma.alloc = buf;
	buf->header.dma.size = sizeof(buf->hdr);
	buf->header.dma.len = sizeof(buf->hdr);
	buf->payload.rqindex = OCS_HAL_RQ_INDEX_DUMMY_DATA;

	ocs_hal_rqpair_tow_dnrx_check(hal);
	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup devInitShutdown
 * @brief Free tow buffers.
 *
 * @par Description
 * Frees the tow buffers.
 *
 * @param hal Hardware context allocated by the caller.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static void
ocs_hal_rqpair_tow_buffer_free(ocs_hal_t *hal)
{
	ocs_hal_tow_buffer_t *buf;
	uint32_t i;

	if (hal->tow_buffer_pool != NULL) {
		ocs_lock(&hal->io_lock);
			for (i = 0; i < ocs_pool_get_count(hal->tow_buffer_pool); i++) {
				buf = ocs_pool_get_instance(hal->tow_buffer_pool, i);
				if (buf != NULL) {
					ocs_dma_free(hal->os, &buf->payload.dma);
				}
			}
		ocs_unlock(&hal->io_lock);

		ocs_pool_free(hal->tow_buffer_pool);
		hal->tow_buffer_pool = NULL;
	}
}

/**
 * @ingroup devInitShutdown
 * @brief Configure the rq_pair function from ocs_hal_init().
 *        Must be after the IOs are setup and the state is active
 *
 * @par Description
 * Allocates the TOW buffers and posts initial XRIs for this feature.
 *
 * @param hal Hardware context allocated by the caller.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_rqpair_init(ocs_hal_t *hal)
{
	ocs_hal_rtn_e rc = 0;
	uint32_t xris_posted;

	/*
	 * If we get this far, the auto TOW feature was enabled successfully,
	 * otherwise ocs_hal_init() would return with an error.
	 * So allocate the buffers based on the initial XRI pool required to support this feature.
	 */
	if (hal->tow_enabled) {
		if (hal->tow_buffer_pool == NULL) {
			/*
			 * Allocate one more buffer than XRIs so that when all the XRIs are in use, we still have
			 * one to post back for the case where the response phase is started in the context of
			 * the data completion.
			 */
			rc = ocs_hal_rqpair_tow_buffer_alloc(hal, hal->config.tow_xri_cnt + 1);
			if (rc != OCS_HAL_RTN_SUCCESS) {
				goto exit_err;
			}
		} else {
			ocs_pool_reset(hal->tow_buffer_pool);
		}

		/* Post the XRIs */
		xris_posted = ocs_hal_xri_move_to_port_owned(hal, hal->config.tow_xri_cnt);
		if (xris_posted != hal->config.tow_xri_cnt) {
			ocs_log_err(hal->os, "Post xri failed, only posted %d XRIs\n", xris_posted);
			rc = OCS_HAL_RTN_ERROR;
			goto exit_err;
		}
	}

	ocs_log_debug(hal->os, "RQ pair init completed\n");

exit_err:
	return rc;
}

/**
 * @ingroup devInitShutdown
 * @brief Tear down the rq_pair function from ocs_hal_teardown().
 *
 * @par Description
 * Frees the buffers to TOW buffers.
 *
 * @param hal Hardware context allocated by the caller.
 */
void
ocs_hal_rqpair_teardown(ocs_hal_t *hal)
{
	/* We need to free any tow buffers */
	ocs_hal_rqpair_tow_buffer_free(hal);
}
