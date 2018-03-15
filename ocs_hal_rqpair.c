/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Limited and/or its subsidiaries.
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

/**
 * @file
 * This file contains code related to operating in RQ Pair mode.
 *
 */

#include "ocs_os.h"
#include "ocs_hal.h"

/* Uncomment this to turn on RQ debug */
// #define ENABLE_DEBUG_RQBUF

static int32_t ocs_hal_rqpair_find(ocs_hal_t *hal, uint16_t rq_id);
static ocs_hal_sequence_t * ocs_hal_rqpair_get(ocs_hal_t *hal, uint16_t rqindex, uint16_t bufindex);
static int32_t ocs_hal_rqpair_put(ocs_hal_t *hal, ocs_hal_sequence_t *seq);
static ocs_hal_rtn_e ocs_hal_rqpair_auto_xfer_rdy_buffer_sequence_reset(ocs_hal_t *hal, ocs_hal_sequence_t *seq);

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

	rq_status = sli_fc_rqe_rqid_and_index(&hal->sli, cqe, &rq_id, &index);
	if (0 != rq_status) {
		switch (rq_status) {
		case SLI4_FC_ASYNC_RQ_BUF_LEN_EXCEEDED:
		case SLI4_FC_ASYNC_RQ_DMA_FAILURE:
			/* just get RQ buffer then return to chip */
			rqindex = ocs_hal_rqpair_find(hal, rq_id);
			if (rqindex < 0) {
				ocs_log_test(hal->os, "%s: status=%#x: rq_id lookup failed for id=%#x\n",
					__func__, rq_status, rq_id);
				break;
			}

			/* get RQ buffer */
			seq = ocs_hal_rqpair_get(hal, rqindex, index);

			/* return to chip */
			if (ocs_hal_rqpair_sequence_free(hal, seq)) {
				ocs_log_test(hal->os, "%s: status=%#x, failed to return buffers to RQ\n",
					__func__, rq_status);
				break;
			}
			break;
		case SLI4_FC_ASYNC_RQ_INSUFF_BUF_NEEDED:
		case SLI4_FC_ASYNC_RQ_INSUFF_BUF_FRM_DISC:
			/* since RQ buffers were not consumed, cannot return them to chip */
			/* fall through */
			ocs_log_debug(hal->os, "%s: Warning: RCQE status=%#x, \n",
				__func__, rq_status);
		default:
			break;
		}
		return -1;
	}

	rqindex = ocs_hal_rqpair_find(hal, rq_id);
	if (rqindex < 0) {
		ocs_log_test(hal->os, "%s: Error: rq_id lookup failed for id=%#x\n",
			__func__, rq_id);
		return -1;
	}

	OCS_STAT({ hal_rq_t *rq = hal->hal_rq[hal->hal_rq_lookup[rqindex]]; rq->use_count++; rq->hdr_use_count++;
		 rq->payload_use_count++;})

	seq = ocs_hal_rqpair_get(hal, rqindex, index);
	ocs_hal_assert(seq != NULL);

	seq->hal = hal;
	seq->auto_xrdy = 0;
	seq->out_of_xris = 0;
	seq->xri = 0;
	seq->hio = NULL;

	sli_fc_rqe_length(&hal->sli, cqe, &h_len, &p_len);
	seq->header->dma.len = h_len;
	seq->payload->dma.len = p_len;
	seq->fcfi = sli_fc_rqe_fcfi(&hal->sli, cqe);
	seq->hal_priv = cq->eq;

	/* bounce enabled, single RQ, we snoop the ox_id to choose the cpuidx */
	if (hal->config.bounce) {
		fc_header_t *hdr = seq->header->dma.virt;
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
ocs_hal_rqpair_process_auto_xfr_rdy_cmd(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe)
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
	uint8_t axr_lock_taken = 0;
#if defined(OCS_DISC_SPIN_DELAY)
	uint32_t 	delay = 0;
	char 		prop_buf[32];
#endif

	rq_status = sli_fc_rqe_rqid_and_index(&hal->sli, cqe, &rq_id, &index);
	if (0 != rq_status) {
		switch (rq_status) {
		case SLI4_FC_ASYNC_RQ_BUF_LEN_EXCEEDED:
		case SLI4_FC_ASYNC_RQ_DMA_FAILURE:
			/* just get RQ buffer then return to chip */
			rqindex = ocs_hal_rqpair_find(hal, rq_id);
			if (rqindex < 0) {
				ocs_log_err(hal->os, "%s: status=%#x: rq_id lookup failed for id=%#x\n",
					__func__, rq_status, rq_id);
				break;
			}

			/* get RQ buffer */
			seq = ocs_hal_rqpair_get(hal, rqindex, index);

			/* return to chip */
			if (ocs_hal_rqpair_sequence_free(hal, seq)) {
				ocs_log_err(hal->os, "%s: status=%#x, failed to return buffers to RQ\n",
					__func__, rq_status);
				break;
			}
			break;
		case SLI4_FC_ASYNC_RQ_INSUFF_BUF_NEEDED:
		case SLI4_FC_ASYNC_RQ_INSUFF_BUF_FRM_DISC:
			/* since RQ buffers were not consumed, cannot return them to chip */
			ocs_log_debug(hal->os, "%s: Warning: RCQE status=%#x, \n",
				__func__, rq_status);
			/* fall through */
		default:
			break;
		}
		return -1;
	}

	rqindex = ocs_hal_rqpair_find(hal, rq_id);
	if (rqindex < 0) {
		ocs_log_err(hal->os, "%s: Error: rq_id lookup failed for id=%#x\n",
			__func__, rq_id);
		return -1;
	}

	OCS_STAT({ hal_rq_t *rq = hal->hal_rq[hal->hal_rq_lookup[rqindex]]; rq->use_count++; rq->hdr_use_count++;
		 rq->payload_use_count++;})

	seq = ocs_hal_rqpair_get(hal, rqindex, index);
	ocs_hal_assert(seq != NULL);

	seq->hal = hal;
	seq->auto_xrdy = opt_wr->agxr;
	seq->out_of_xris = opt_wr->oox;
	seq->xri = opt_wr->xri;
	seq->hio = NULL;

	sli_fc_rqe_length(&hal->sli, cqe, &h_len, &p_len);
	seq->header->dma.len = h_len;
	seq->payload->dma.len = p_len;
	seq->fcfi = sli_fc_rqe_fcfi(&hal->sli, cqe);
	seq->hal_priv = cq->eq;

	if (seq->auto_xrdy) {
		fc_header_t *fc_hdr = seq->header->dma.virt;

		seq->hio = ocs_hal_io_lookup(hal, seq->xri);
		ocs_lock(&seq->hio->axr_lock);
		axr_lock_taken = 1;

		/* save the FCFI, src_id, dest_id and ox_id because we need it for the sequence object when the data comes. */
		seq->hio->axr_buf->fcfi = seq->fcfi;
		seq->hio->axr_buf->hdr.ox_id = fc_hdr->ox_id;
		seq->hio->axr_buf->hdr.s_id = fc_hdr->s_id;
		seq->hio->axr_buf->hdr.d_id = fc_hdr->d_id;
		seq->hio->axr_buf->cmd_cqe = 1;

		/*
		 * Since auto xfer rdy is used for this IO, then clear the sequence
		 * initiative bit in the header so that the upper layers wait for the
		 * data. This should flow exactly like the first burst case.
		 */
		fc_hdr->f_ctl &= fc_htobe24(~FC_FCTL_SEQUENCE_INITIATIVE);

		/* If AXR CMD CQE came before previous TRSP CQE of same XRI */
		if (seq->hio->type == OCS_HAL_IO_TARGET_RSP) {
			seq->hio->axr_buf->call_axr_cmd = 1;
			seq->hio->axr_buf->cmd_seq = seq;
			goto exit_ocs_hal_rqpair_process_auto_xfr_rdy_cmd;
		}
	}

	/* bounce enabled, single RQ, we snoop the ox_id to choose the cpuidx */
	if (hal->config.bounce) {
		fc_header_t *hdr = seq->header->dma.virt;
		uint32_t s_id = fc_be24toh(hdr->s_id);
		uint32_t d_id = fc_be24toh(hdr->d_id);
		uint32_t ox_id =  ocs_be16toh(hdr->ox_id);
		if (hal->callback.bounce != NULL) {
			(*hal->callback.bounce)(ocs_hal_unsol_process_bounce, seq, s_id, d_id, ox_id);
		}
	} else {
		hal->callback.unsolicited(hal->args.unsolicited, seq);
	}

	if (seq->auto_xrdy) {
		/* If data cqe came before cmd cqe in out of order in case of AXR */
		if(seq->hio->axr_buf->data_cqe == 1) {

#if defined(OCS_DISC_SPIN_DELAY)
			if (ocs_get_property("disk_spin_delay", prop_buf, sizeof(prop_buf)) == 0) {
				delay = ocs_strtoul(prop_buf, 0, 0);
				ocs_udelay(delay);
			}
#endif
			/* bounce enabled, single RQ, we snoop the ox_id to choose the cpuidx */
			if (hal->config.bounce) {
				fc_header_t *hdr = seq->header->dma.virt;
				uint32_t s_id = fc_be24toh(hdr->s_id);
				uint32_t d_id = fc_be24toh(hdr->d_id);
				uint32_t ox_id =  ocs_be16toh(hdr->ox_id);
				if (hal->callback.bounce != NULL) {
					(*hal->callback.bounce)(ocs_hal_unsol_process_bounce, &seq->hio->axr_buf->seq, s_id, d_id, ox_id);
				}
			} else {
				hal->callback.unsolicited(hal->args.unsolicited, &seq->hio->axr_buf->seq);
			}
		}
	}

exit_ocs_hal_rqpair_process_auto_xfr_rdy_cmd:
	if(axr_lock_taken) {
		ocs_unlock(&seq->hio->axr_lock);
	}
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
ocs_hal_rqpair_process_auto_xfr_rdy_data(ocs_hal_t *hal, hal_cq_t *cq, uint8_t *cqe)
{
	/* Seems silly to call a SLI function to decode - use the structure directly for performance */
	sli4_fc_optimized_write_data_cqe_t *opt_wr = (sli4_fc_optimized_write_data_cqe_t*)cqe;
	ocs_hal_sequence_t *seq;
	ocs_hal_io_t *io;
	ocs_hal_auto_xfer_rdy_buffer_t *buf;
#if defined(OCS_DISC_SPIN_DELAY)
	uint32_t 	delay = 0;
	char 		prop_buf[32];
#endif
	/* Look up the IO */
	io = ocs_hal_io_lookup(hal, opt_wr->xri);
	ocs_lock(&io->axr_lock);
	buf = io->axr_buf;
	buf->data_cqe = 1;
	seq = &buf->seq;
	seq->hal = hal;
	seq->auto_xrdy = 1;
	seq->out_of_xris = 0;
	seq->xri = opt_wr->xri;
	seq->hio = io;
	seq->header = &buf->header;
	seq->payload = &buf->payload;

	seq->header->dma.len = sizeof(fc_header_t);
	seq->payload->dma.len = opt_wr->total_data_placed;
	seq->fcfi = buf->fcfi;
	seq->hal_priv = cq->eq;


	if (opt_wr->status == SLI4_FC_WCQE_STATUS_SUCCESS) {
		seq->status = OCS_HAL_UNSOL_SUCCESS;
	} else if (opt_wr->status == SLI4_FC_WCQE_STATUS_REMOTE_STOP) {
		seq->status = OCS_HAL_UNSOL_ABTS_RCVD;
	} else {
		seq->status = OCS_HAL_UNSOL_ERROR;
	}

 	/* If AXR CMD CQE came before previous TRSP CQE of same XRI */
	if(io->type == OCS_HAL_IO_TARGET_RSP) {
		io->axr_buf->call_axr_data = 1;
		goto exit_ocs_hal_rqpair_process_auto_xfr_rdy_data;
	}

	if(!buf->cmd_cqe) {
		/* if data cqe came before cmd cqe, return here, cmd cqe will handle */
		goto exit_ocs_hal_rqpair_process_auto_xfr_rdy_data;
	}
#if defined(OCS_DISC_SPIN_DELAY)
	if (ocs_get_property("disk_spin_delay", prop_buf, sizeof(prop_buf)) == 0) {
		delay = ocs_strtoul(prop_buf, 0, 0);
		ocs_udelay(delay);
	}
#endif

	/* bounce enabled, single RQ, we snoop the ox_id to choose the cpuidx */
	if (hal->config.bounce) {
		fc_header_t *hdr = seq->header->dma.virt;
		uint32_t s_id = fc_be24toh(hdr->s_id);
		uint32_t d_id = fc_be24toh(hdr->d_id);
		uint32_t ox_id =  ocs_be16toh(hdr->ox_id);
		if (hal->callback.bounce != NULL) {
			(*hal->callback.bounce)(ocs_hal_unsol_process_bounce, seq, s_id, d_id, ox_id);
		}
	} else {
		hal->callback.unsolicited(hal->args.unsolicited, seq);
	}

exit_ocs_hal_rqpair_process_auto_xfr_rdy_data:
	ocs_unlock(&io->axr_lock);
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
	sli4_queue_t *rq_hdr = &hal->rq[rqindex];
	sli4_queue_t *rq_payload = &hal->rq[rqindex+1];
	ocs_hal_sequence_t *seq = NULL;
	hal_rq_t *rq = hal->hal_rq[hal->hal_rq_lookup[rqindex]];

#if defined(ENABLE_DEBUG_RQBUF)
	uint64_t rqbuf_debug_value = 0xdead0000 | ((rq->id & 0xf) << 12) | (bufindex & 0xfff);
#endif

	if (bufindex >= rq_hdr->length) {
		ocs_log_err(hal->os, "%s: RQ index %d bufindex %d exceed ring length %d for id %d\n",
			    __func__, rqindex, bufindex, rq_hdr->length, rq_hdr->id);
		return NULL;
	}

	sli_queue_lock(rq_hdr);
	sli_queue_lock(rq_payload);

#if defined(ENABLE_DEBUG_RQBUF)
	/* Put a debug value into the rq, to track which entries are still valid */
	_sli_queue_poke(&hal->sli, rq_hdr, bufindex, (uint8_t *)&rqbuf_debug_value);
	_sli_queue_poke(&hal->sli, rq_payload, bufindex, (uint8_t *)&rqbuf_debug_value);
#endif

	seq = rq->rq_tracker[bufindex];
	rq->rq_tracker[bufindex] = NULL;

	if (seq == NULL ) {
		ocs_log_err(hal->os, "%s: RQ buffer NULL, rqindex %d, bufindex %d, current q index = %d\n",
			    __func__, rqindex, bufindex, rq_hdr->index);
	}

	sli_queue_unlock(rq_payload);
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
	sli4_queue_t *rq_hdr = &hal->rq[seq->header->rqindex];
	sli4_queue_t *rq_payload = &hal->rq[seq->payload->rqindex];
	uint32_t hal_rq_index = hal->hal_rq_lookup[seq->header->rqindex];
	hal_rq_t *rq = hal->hal_rq[hal_rq_index];
	uint32_t     phys_hdr[2];
	uint32_t     phys_payload[2];
	int32_t      qindex_hdr;
	int32_t      qindex_payload;

	/* Update the RQ verification lookup tables */
	phys_hdr[0] = ocs_addr32_hi(seq->header->dma.phys);
	phys_hdr[1] = ocs_addr32_lo(seq->header->dma.phys);
	phys_payload[0] = ocs_addr32_hi(seq->payload->dma.phys);
	phys_payload[1] = ocs_addr32_lo(seq->payload->dma.phys);

	sli_queue_lock(rq_hdr);
	sli_queue_lock(rq_payload);

	/*
	 * Note: The header must be posted last for buffer pair mode because
	 *       posting on the header queue posts the payload queue as well.
	 *       We do not ring the payload queue independently in RQ pair mode.
	 */
	qindex_payload = _sli_queue_write(&hal->sli, rq_payload, (void *)phys_payload);
	qindex_hdr = _sli_queue_write(&hal->sli, rq_hdr, (void *)phys_hdr);
	if (qindex_hdr < 0 ||
	    qindex_payload < 0) {
		ocs_log_err(hal->os, "RQ_ID=%#x write failed\n", rq_hdr->id);
		sli_queue_unlock(rq_payload);
		sli_queue_unlock(rq_hdr);
		return OCS_HAL_RTN_ERROR;
	}

	/* ensure the indexes are the same */
	ocs_hal_assert(qindex_hdr == qindex_payload);

	/* Update the lookup table */
	if (rq->rq_tracker[qindex_hdr] == NULL) {
		rq->rq_tracker[qindex_hdr] = seq;
	} else {
		ocs_log_test(hal->os, "%s: expected rq_tracker[%d][%d] buffer to be NULL\n", __func__,
			     hal_rq_index, qindex_hdr);
	}

	sli_queue_unlock(rq_payload);
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

	/* Check for auto xfer rdy dummy buffers and call the proper release function. */
	if (seq->header->rqindex == OCS_HAL_RQ_INDEX_DUMMY_HDR) {
		return ocs_hal_rqpair_auto_xfer_rdy_buffer_sequence_reset(hal, seq);
	}

	/*
	 * Post the data buffer first. Because in RQ pair mode, ringing the
	 * doorbell of the header ring will post the data buffer as well.
	 */
	if (ocs_hal_rqpair_put(hal, seq)) {
		ocs_log_err(hal->os, "%s: error writing buffers\n", __func__);
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
 * @brief Allocate auto xfer rdy buffers.
 *
 * @par Description
 * Allocates the auto xfer rdy buffers and places them on the free list.
 *
 * @param hal Hardware context allocated by the caller.
 * @param num_buffers Number of buffers to allocate.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_rqpair_auto_xfer_rdy_buffer_alloc(ocs_hal_t *hal, uint32_t num_buffers)
{
	ocs_hal_auto_xfer_rdy_buffer_t *buf;
	uint32_t i;

	hal->auto_xfer_rdy_buf_pool = ocs_pool_alloc(hal->os, sizeof(ocs_hal_auto_xfer_rdy_buffer_t), num_buffers, FALSE);
	if (hal->auto_xfer_rdy_buf_pool == NULL) {
		ocs_log_err(hal->os, "%s: Failure to allocate auto xfer ready buffer pool\n", __func__);
		return OCS_HAL_RTN_NO_MEMORY;
	}

	for (i = 0; i < num_buffers; i++) {
		/* allocate the wrapper object */
		buf = ocs_pool_get_instance(hal->auto_xfer_rdy_buf_pool, i);
		ocs_hal_assert(buf != NULL);

		/* allocate the auto xfer ready buffer */
		if (ocs_dma_alloc(hal->os, &buf->payload.dma, hal->config.auto_xfer_rdy_size, OCS_MIN_DMA_ALIGNMENT)) {
			ocs_log_err(hal->os, "%s: DMA allocation failed\n", __func__);
			ocs_free(hal->os, buf, sizeof(*buf));
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
ocs_hal_rqpair_auto_xfer_rdy_dnrx_check(ocs_hal_t *hal)
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
	if (status != 0) {
		ocs_log_debug(hal->os, "%s Status 0x%x\n", __func__, status);
	}

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
ocs_hal_rqpair_auto_xfer_rdy_move_to_port(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	/* We only need to preregister the SGL if it has not yet been done. */
	if (!sli_get_sgl_preregister(&hal->sli)) {
		uint8_t	*post_sgl;
		ocs_dma_t *psgls = &io->def_sgl;
		ocs_dma_t **sgls = &psgls;

		/* non-local buffer required for mailbox queue */
		post_sgl = ocs_malloc(hal->os, SLI4_BMBX_SIZE, OCS_M_NOWAIT);
		if (post_sgl == NULL) {
			ocs_log_err(hal->os, "%s no buffer for command\n", __func__);
			return OCS_HAL_RTN_NO_MEMORY;
		}
		if (sli_cmd_fcoe_post_sgl_pages(&hal->sli, post_sgl, SLI4_BMBX_SIZE,
						io->indicator, 1, sgls, NULL, NULL)) {
			if (ocs_hal_command(hal, post_sgl, OCS_CMD_NOWAIT,
					    ocs_hal_rqpair_auto_xfer_rdy_move_to_port_cb, NULL)) {
				ocs_free(hal->os, post_sgl, SLI4_BMBX_SIZE);
				ocs_log_err(hal->os, "%s: SGL post failed\n", __func__);
				return OCS_HAL_RTN_ERROR;
			}
		}
	}

	ocs_lock(&hal->io_lock);
	if (ocs_hal_rqpair_auto_xfer_rdy_buffer_post(hal, io, 0) != 0) { /* DNRX set - no buffer */
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
ocs_hal_rqpair_auto_xfer_rdy_move_to_host(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	if (io->axr_buf != NULL) {
		ocs_lock(&hal->io_lock);
			/* check  list and remove if there */
			if (ocs_list_on_list(&io->dnrx_link)) {
				ocs_list_remove(&hal->io_port_dnrx, io);
				io->auto_xfer_rdy_dnrx = 0;

				/* release the count for waiting for a buffer */
				ocs_hal_io_free(hal, io);
			}

			ocs_pool_put(hal->auto_xfer_rdy_buf_pool, io->axr_buf);
			io->axr_buf = NULL;
		ocs_unlock(&hal->io_lock);

		ocs_hal_rqpair_auto_xfer_rdy_dnrx_check(hal);
	}
	return;
}


/**
 * @brief Posts an auto xfer rdy buffer to an IO.
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
ocs_hal_rqpair_auto_xfer_rdy_buffer_post(ocs_hal_t *hal, ocs_hal_io_t *io, int reuse_buf)
{
	ocs_hal_auto_xfer_rdy_buffer_t *buf;
	sli4_sge_t	*data;

	if(!reuse_buf) {
		buf = ocs_pool_get(hal->auto_xfer_rdy_buf_pool);
		io->axr_buf = buf;
	}

	data = io->def_sgl.virt;
	data[0].sge_type = SLI4_SGE_TYPE_SKIP;
	data[0].last = 0;

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
	if (hal->config.auto_xfer_rdy_p_type) {
		sli4_diseed_sge_t *diseed = (sli4_diseed_sge_t*)&data[1];

		diseed->sge_type = SLI4_SGE_TYPE_DISEED;
		diseed->repl_app_tag = hal->config.auto_xfer_rdy_app_tag_value;
		diseed->app_tag_cmp = hal->config.auto_xfer_rdy_app_tag_value;
		diseed->check_app_tag = hal->config.auto_xfer_rdy_app_tag_valid;
		diseed->auto_incr_ref_tag = TRUE; /* Always the LBA */
		diseed->dif_blk_size = hal->config.auto_xfer_rdy_blk_size_chip;
	} else {
		data[1].sge_type = SLI4_SGE_TYPE_SKIP;
		data[1].last = 0;
	}

	data[2].sge_type = SLI4_SGE_TYPE_DATA;
	data[2].buffer_address_high = ocs_addr32_hi(io->axr_buf->payload.dma.phys);
	data[2].buffer_address_low  = ocs_addr32_lo(io->axr_buf->payload.dma.phys);
	data[2].buffer_length = io->axr_buf->payload.dma.size;
	data[2].last = TRUE;
	data[3].sge_type = SLI4_SGE_TYPE_SKIP;

	return 0;
}

/**
 * @brief Return auto xfer ready buffers (while in RQ pair mode).
 *
 * @par Description
 * The header and payload buffers are returned to the auto xfer rdy pool.
 *
 * @param hal Hardware context.
 * @param seq Header/payload sequence buffers.
 *
 * @return Returns OCS_HAL_RTN_SUCCESS for success, an error code value for failure.
 */

static ocs_hal_rtn_e
ocs_hal_rqpair_auto_xfer_rdy_buffer_sequence_reset(ocs_hal_t *hal, ocs_hal_sequence_t *seq)
{
	ocs_hal_auto_xfer_rdy_buffer_t *buf = seq->header->dma.alloc;

	buf->data_cqe = 0;
	buf->cmd_cqe = 0;
	buf->fcfi = 0;
	buf->call_axr_cmd = 0;
	buf->call_axr_data = 0;

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

	ocs_hal_rqpair_auto_xfer_rdy_dnrx_check(hal);

	return OCS_HAL_RTN_SUCCESS;
}

/**
 * @ingroup devInitShutdown
 * @brief Free auto xfer rdy buffers.
 *
 * @par Description
 * Frees the auto xfer rdy buffers.
 *
 * @param hal Hardware context allocated by the caller.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
static void
ocs_hal_rqpair_auto_xfer_rdy_buffer_free(ocs_hal_t *hal)
{
	ocs_hal_auto_xfer_rdy_buffer_t *buf;
	uint32_t i;

	if (hal->auto_xfer_rdy_buf_pool != NULL) {
		ocs_lock(&hal->io_lock);
			for (i = 0; i < ocs_pool_get_count(hal->auto_xfer_rdy_buf_pool); i++) {
				buf = ocs_pool_get_instance(hal->auto_xfer_rdy_buf_pool, i);
				if (buf != NULL) {
					ocs_dma_free(hal->os, &buf->payload.dma);
				}
			}
		ocs_unlock(&hal->io_lock);

		ocs_pool_free(hal->auto_xfer_rdy_buf_pool);
		hal->auto_xfer_rdy_buf_pool = NULL;
	}
}

/**
 * @ingroup devInitShutdown
 * @brief Configure the rq_pair function from ocs_hal_init().
 *
 * @par Description
 * Allocates the buffers to auto xfer rdy and posts initial XRIs for this feature.
 *
 * @param hal Hardware context allocated by the caller.
 *
 * @return Returns 0 on success, or a non-zero value on failure.
 */
ocs_hal_rtn_e
ocs_hal_rqpair_init(ocs_hal_t *hal)
{
	ocs_hal_rtn_e	rc;
	uint32_t xris_posted;

	ocs_log_debug(hal->os, "%s: RQ Pair mode\n", __func__);

	/*
	 * If we get this far, the auto XFR_RDY feature was enabled successfully, otherwise ocs_hal_init() would
	 * return with an error. So allocate the buffers based on the initial XRI pool required to support this
	 * feature.
	 */
	if (sli_get_auto_xfer_rdy_capable(&hal->sli) &&
	    hal->config.auto_xfer_rdy_size > 0) {
		if (hal->auto_xfer_rdy_buf_pool == NULL) {
			/*
			 * Allocate one more buffer than XRIs so that when all the XRIs are in use, we still have
			 * one to post back for the case where the response phase is started in the context of
			 * the data completion.
			 */
			rc = ocs_hal_rqpair_auto_xfer_rdy_buffer_alloc(hal, hal->config.auto_xfer_rdy_xri_cnt + 1);
			if (rc != OCS_HAL_RTN_SUCCESS) {
				return rc;
			}
		} else {
			ocs_pool_reset(hal->auto_xfer_rdy_buf_pool);
		}

		/* Post the auto XFR_RDY XRIs */
		xris_posted = ocs_hal_xri_move_to_port_owned(hal, hal->config.auto_xfer_rdy_xri_cnt);
		if (xris_posted != hal->config.auto_xfer_rdy_xri_cnt) {
			ocs_log_err(hal->os, "%s: post_xri failed, only posted %d XRIs\n", __func__, xris_posted);
			return OCS_HAL_RTN_ERROR;
		}
	}

	return 0;
}

/**
 * @ingroup devInitShutdown
 * @brief Tear down the rq_pair function from ocs_hal_teardown().
 *
 * @par Description
 * Frees the buffers to auto xfer rdy.
 *
 * @param hal Hardware context allocated by the caller.
 */
void
ocs_hal_rqpair_teardown(ocs_hal_t *hal)
{
	/* We need to free any auto xfer ready buffers */
	ocs_hal_rqpair_auto_xfer_rdy_buffer_free(hal);
}
