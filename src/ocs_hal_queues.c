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
 *
 */

#include "ocs_os.h"
#include "ocs_hal.h"
#include "ocs_hal_queues.h"

#define HAL_QTOP_DEBUG		0

static inline bool
ocs_nvmeq_configured(ocs_hal_t *hal)
{
	uint32_t i;
	for (i = 0; i < hal->eq_count; i++) {
		if (hal->hal_eq[i]->nvmeq)
			return true;
	}

	return false;
}

hal_eq_t*
ocs_hal_eq_for_send_frame(ocs_hal_t *hal, int32_t scsi_nvme)
{
	uint32_t i;
	uint32_t j;
	hal_rq_t *rq = NULL;

	/* Just use first non-NVMe EQ for SCSI dev */
	if (scsi_nvme == OCS_HAL_WQ_SFQ_SCSI) {
		for (i = 0; i < hal->eq_count; i++) {
			if (hal->hal_eq[i] && !hal->hal_eq[i]->nvmeq)
				return hal->hal_eq[i];
		}
	}

	ocs_assert(scsi_nvme == OCS_HAL_WQ_SFQ_NVME, NULL);
	/* For NVMe, first get the RQ */
	for (i = 0; i < hal->config.n_rq; i++) {
		rq = hal->hal_rq[i];
		if (!rq->nvmeq)
			continue;

		/* Check if this RQ has the filter of IO */
		for (j = 0; j < SLI4_CMD_REG_FCFI_NUM_RQ_CFG; j++) {
			if (rq->filter_mask & (1U << j)) {
				if ((hal->config.filter_def[j] == OCS_NVME_IO_FILTER) && rq->nvmeq) {
					return rq->cq->eq;
				}
			}
		}
	}

	return NULL;
}

ocs_hal_rtn_e
ocs_hal_init_send_frame_queue(ocs_hal_t *hal, int32_t scsi_nvme)
{
	hal_eq_t *eq = NULL;
	hal_cq_t *cq = NULL;
	hal_wq_t *wq = NULL;
	uint32_t entries;
	ocs_hal_rtn_e rc = OCS_HAL_RTN_ERROR;

	/* Fetch EQ to bind CQ/WQ */
	eq = ocs_hal_eq_for_send_frame(hal, scsi_nvme);
	if (!eq) {
		ocs_log_err(hal->os, "Failed to fetch EQ for Send frame\n");
		return rc;
	}

	entries = OCS_MIN(OCS_HAL_CQ_ENTRIES_DEF, hal->num_qentries[SLI_QTYPE_CQ]);
	cq = hal_new_cq(eq, entries);
	if (!cq) {
		ocs_log_err(hal->os, "CQ creation failed for WQ SFQ\n");
		goto exit;
	}

	entries = OCS_MIN((cq->entry_count / 2), hal->num_qentries[SLI_QTYPE_WQ]);
	wq = hal_new_wq(cq, entries, 0, 0, true);
	if (!wq) {
		ocs_log_err(hal->os, "WQ creation failed for WQ SFQ\n");
		goto exit;
	}

	hal->hal_sfwq[scsi_nvme] = wq;
	if (wq->fw_sfq_enabled)
		ocs_log_info(hal->os, "FW enabled WQ SFQ created for %s dev\n", (scsi_nvme ? "NVMe" : "SCSI"));

	rc = OCS_HAL_RTN_SUCCESS;

exit:
	return rc;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wframe-larger-than="
/**
 * @brief Initialize queues
 *
 * Given the parsed queue topology spec, the SLI queues are created and
 * initialized
 *
 * @param hal pointer to HAL object
 * @param qtop pointer to queue topology
 *
 * @return returns 0 for success, an error code value for failure.
 */
ocs_hal_rtn_e
ocs_hal_init_queues(ocs_hal_t *hal, ocs_hal_qtop_t *qtop)
{
	uint32_t i, j;
	uint32_t default_lengths[QTOP_LAST], len;
	ocs_hal_qtop_entry_t *qt, *next_qt;
	ocs_hal_rqs_info_t rqs_info;
	ocs_hal_mrq_info_t mrq_sets[OCS_HAL_MAX_MRQ_SETS];

	hal_eq_t *eq = NULL;
	hal_cq_t *cq = NULL;
	hal_wq_t *wq = NULL;
	hal_rq_t *rq = NULL;
	hal_mq_t *mq = NULL;

	rqs_info.num_pairs = 0;
	for (i = 0; i < OCS_HAL_MAX_MRQ_SETS; i++) {
		ocs_memset(&mrq_sets[i], 0, sizeof(ocs_hal_mrq_info_t));
	}

	default_lengths[QTOP_EQ] = OCS_HAL_EQ_ENTRIES_DEF;
	default_lengths[QTOP_CQ] = hal->num_qentries[SLI_QTYPE_CQ];
	default_lengths[QTOP_WQ] = hal->num_qentries[SLI_QTYPE_WQ];
	default_lengths[QTOP_RQ] = OCS_HAL_RQ_ENTRIES_DEF;
	default_lengths[QTOP_MQ] = OCS_HAL_MQ_ENTRIES_DEF;

	ocs_hal_verify(hal != NULL, OCS_HAL_RTN_INVALID_ARG);

	hal->eq_count = 0;
	hal->cq_count = 0;
	hal->mq_count = 0;
	hal->wq_count = 0;
	hal->rq_count = 0;
	hal->hal_rq_count = 0;
	ocs_list_init(&hal->eq_list, hal_eq_t, link);

	/* Allocate class WQ pools */
	for (i = 0; i < ARRAY_SIZE(hal->wq_class_array); i++) {
		hal->wq_class_array[i] = ocs_varray_alloc(hal->os, OCS_HAL_MAX_NUM_WQ);
		if (hal->wq_class_array[i] == NULL) {
			ocs_log_err(hal->os, "ocs_varray_alloc for wq_class failed\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}
	}

	/* Allocate per CPU WQ pools */
	for (i = 0; i < ARRAY_SIZE(hal->wq_cpu_array); i++) {
		hal->wq_cpu_array[i] = ocs_varray_alloc(hal->os, OCS_HAL_MAX_NUM_WQ);
		if (hal->wq_cpu_array[i] == NULL) {
			ocs_log_err(hal->os, "ocs_varray_alloc for wq_class failed\n");
			return OCS_HAL_RTN_NO_MEMORY;
		}
	}

	ocs_hal_assert(qtop != NULL);
	for (i = 0, qt = qtop->entries; i < qtop->inuse_count; i++, qt++) {
		if (i == qtop->inuse_count - 1)
			next_qt = NULL;
		else
			next_qt = qt + 1;

		switch(qt->entry) {
		case QTOP_EQ:
			len = (qt->len) ? qt->len : OCS_HAL_EQ_ENTRIES_DEF;

			/* Modify the Q length if qtop EQ len is not a valid value */
			if (!ocs_qlen_valid(len, OCS_HAL_EQ_ENTRIES_MIN, hal->num_qentries[SLI_QTYPE_EQ])) {
				ocs_log_info(hal->os, "QTOP EQ len %d is invalid; using EQ len %d\n",
						len, default_lengths[QTOP_EQ]);
				len = default_lengths[QTOP_EQ];
			}

			if (qt->set_default) {
				default_lengths[QTOP_EQ] = len;
				break;
			}

			eq = hal_new_eq(hal, len);
			if (eq == NULL)
				goto fail;

			eq->nvmeq = qt->nvmeq;
			if (!eq->nvmeq) {
				eq->cpu_core = ocs_sched_cpu(eq->instance);
				if (eq->cpu_core == UINT32_MAX) {
					ocs_log_err(hal->os, "**No cpu/core available**\n");
					goto fail;
				}
			}
			break;

		case QTOP_CQ:
			len = (qt->len) ? qt->len : OCS_HAL_CQ_ENTRIES_DEF;

			/* Modify the Q length if qtop CQ len is not a valid value */
			if (!ocs_qlen_valid(len, OCS_HAL_CQ_ENTRIES_MIN, hal->num_qentries[SLI_QTYPE_CQ])) {
				ocs_log_info(hal->os, "QTOP CQ len %d is invalid; using CQ len %d\n",
						len, default_lengths[QTOP_CQ]);
				len = default_lengths[QTOP_CQ];
			}

			if (qt->set_default) {
				default_lengths[QTOP_CQ] = len;
				break;
			}

			if (eq && next_qt) {
				/* If this CQ is for RQ, then delay the creation */
				if (next_qt->entry != QTOP_RQ) {
					cq = hal_new_cq(eq, len);
					if (cq == NULL)
						goto fail;
				}
			} else {
				goto fail;
			}

			break;

		case QTOP_WQ: {
			uint32_t max_wq_entries;

			len = (qt->len) ? qt->len : OCS_HAL_WQ_ENTRIES_DEF;

			if (cq) {
				/* Limit the max WQ length to half the corresponding CQ length */
				max_wq_entries = OCS_MIN((cq->entry_count / 2), hal->num_qentries[SLI_QTYPE_WQ]);
			} else {
				goto fail;
			}

			/* Modify the Q length if qtop WQ len is not a valid value */
			if (!ocs_qlen_valid(len, OCS_HAL_WQ_ENTRIES_MIN, max_wq_entries)) {
				ocs_log_info(hal->os, "QTOP WQ len %d is invalid; using WQ len %d\n",
						len, max_wq_entries);
				len = max_wq_entries;
			}

			if (qt->set_default) {
				default_lengths[QTOP_WQ] = len;
				break;
			}

			if ((hal->ulp_start + qt->ulp) > hal->ulp_max) {
				ocs_log_err(hal->os, "invalid ULP %d for WQ\n", qt->ulp);
				goto fail;
			}

			wq = hal_new_wq(cq, len, qt->class, hal->ulp_start + qt->ulp, false);
			if (wq == NULL)
				goto fail;

			if (eq->nvmeq)
				break;

			/* Place this WQ on the EQ WQ array */
			if (ocs_varray_add(eq->wq_array, wq)) {
				ocs_log_err(hal->os, "QTOP_WQ: EQ ocs_varray_add failed\n");
				goto fail;
			}

			/* Place this WQ on the HAL class array */
			if (qt->class < ARRAY_SIZE(hal->wq_class_array)) {
				if (ocs_varray_add(hal->wq_class_array[qt->class], wq)) {
					ocs_log_err(hal->os, "HAL wq_class_array ocs_varray_add failed\n");
					goto fail;
				}
			} else {
				ocs_log_err(hal->os, "Invalid class value: %d\n", qt->class);
				goto fail;
			}

			/*
			 * Place this WQ on the per CPU list, asumming that EQs are mapped to cpu given
			 * by the EQ instance modulo number of CPUs
			 */
			if (ocs_varray_add(hal->wq_cpu_array[eq->cpu_core], wq)) {
				ocs_log_err(hal->os, "HAL wq_cpu_array ocs_varray_add failed\n");
				goto fail;
			}

			break;
		}

		case QTOP_RQ:
			len = (qt->len) ? qt->len : OCS_HAL_RQ_ENTRIES_DEF;

			/* Modify the Q length if qtop RQ len is not a valid value */
			if (!ocs_qlen_valid(len, OCS_HAL_RQ_ENTRIES_MIN, hal->num_qentries[SLI_QTYPE_RQ])) {
				ocs_log_info(hal->os, "QTOP RQ len %d is invalid; using RQ len %d\n",
						len, default_lengths[QTOP_RQ]);
				len = default_lengths[QTOP_RQ];
			}

			if (qt->set_default) {
				default_lengths[QTOP_RQ] = len;
				break;
			}

			if ((hal->ulp_start + qt->ulp) > hal->ulp_max) {
				ocs_log_err(hal->os, "invalid ULP %d for RQ\n", qt->ulp);
				goto fail;
			}

			rqs_info.rq_cfg[rqs_info.num_pairs].len = len;
			rqs_info.rq_cfg[rqs_info.num_pairs].ulp = hal->ulp_start + qt->ulp;
			rqs_info.rq_cfg[rqs_info.num_pairs].filter_mask = qt->filter_mask;
			rqs_info.rq_cfg[rqs_info.num_pairs].policy = qt->policy;
			rqs_info.rq_cfg[rqs_info.num_pairs].eq = eq;
			rqs_info.rq_cfg[rqs_info.num_pairs].protocol_valid = qt->protocol_valid;
			rqs_info.rq_cfg[rqs_info.num_pairs].protocol = qt->protocol;
			rqs_info.num_pairs++;
			break;

		case QTOP_MQ:
			len = (qt->len) ? qt->len : OCS_HAL_MQ_ENTRIES_DEF;

			/* Modify the Q length if qtop MQ len is not a valid value */
			if (!ocs_qlen_valid(len, OCS_HAL_MQ_ENTRIES_MIN, hal->num_qentries[SLI_QTYPE_MQ])) {
				ocs_log_info(hal->os, "QTOP MQ len %d is invalid; using MQ len %d\n",
						len, default_lengths[QTOP_MQ]);
				len = default_lengths[QTOP_MQ];
			}

			if (qt->set_default) {
				default_lengths[QTOP_MQ] = len;
				break;
			}

			if (cq) {
				mq = hal_new_mq(cq, len);
				if (mq == NULL)
					goto fail;
			} else {
				goto fail;
			}

			break;

		default:
			ocs_hal_assert(0);
			break;
		}
	}

	/*
	 * Now create RQs.
	 * Note: If more than one RQ has the same filter mask, then they should be
	 * 	 created using SET. Else, we should create as a normal RQ pair.
	 */
	for (i = 0; i < rqs_info.num_pairs; i++) {
		/* Check if any other RQ has the same filter mask */
		for (j = 0; j < rqs_info.num_pairs; j++) {
			if ((i != j) && (rqs_info.rq_cfg[i].filter_mask == rqs_info.rq_cfg[j].filter_mask)) {
				int id = -1, k;

				/* If we got here, then this RQ belongs to a set; find out which set it should go to */
				for (k = 0; k < OCS_HAL_MAX_MRQ_SETS; k++) {
					if (mrq_sets[k].filter_mask == rqs_info.rq_cfg[i].filter_mask) {
						id = k;
						break;
					}
					if ((id < 0) && !mrq_sets[k].num_pairs) {
						/* First free slot */
						id = k;
					}
				}

				if (id < 0) {
					ocs_log_crit(hal->os, "Can't create more than %d RQ Sets\n", OCS_HAL_MAX_MRQ_SETS);
					goto fail;
				}

				if (!mrq_sets[id].num_pairs) {
					/* Just copy once for first RQ in the set */
					mrq_sets[id].len = rqs_info.rq_cfg[i].len;
					mrq_sets[id].filter_mask = rqs_info.rq_cfg[i].filter_mask;
					mrq_sets[id].ulp = rqs_info.rq_cfg[i].ulp;
					mrq_sets[id].policy = rqs_info.rq_cfg[i].policy;
					mrq_sets[id].protocol_valid = rqs_info.rq_cfg[i].protocol_valid;
					mrq_sets[id].protocol = rqs_info.rq_cfg[i].protocol;
					mrq_sets[id].nvmeq = rqs_info.rq_cfg[i].eq->nvmeq;
				}

				/* This differs for each RQ in the set */
				mrq_sets[id].eqs[mrq_sets[id].num_pairs] = rqs_info.rq_cfg[i].eq;
				mrq_sets[id].num_pairs ++;
				break;
			}
		}

		if (j == rqs_info.num_pairs) {
			/* No other RQ has the same filter mask, this means a normal RQ PAIR */
			cq = hal_new_cq(rqs_info.rq_cfg[i].eq, default_lengths[QTOP_CQ]);
			if (cq == NULL)
				goto fail;

			rq = hal_new_rq(cq, rqs_info.rq_cfg[i].len, rqs_info.rq_cfg[i].ulp);
			if (rq == NULL)
				goto fail;

			rq->filter_mask = rqs_info.rq_cfg[i].filter_mask;
			rq->nvmeq = rqs_info.rq_cfg[i].eq->nvmeq;
		}
	}

	/* Now create RQ Set */
	for (j = 0; j < OCS_HAL_MAX_MRQ_SETS; j++) {
		hal_cq_t *cqs[OCE_HAL_MAX_NUM_MRQ_PAIRS] = { NULL };
		hal_rq_t *rqs[OCE_HAL_MAX_NUM_MRQ_PAIRS] = { NULL };

		if (!mrq_sets[j].num_pairs)
			continue; /* Not used */

		if (mrq_sets[j].num_pairs > OCE_HAL_MAX_NUM_MRQ_PAIRS) {
			ocs_log_crit(hal->os, "Max Supported MRQ pairs = %d\n", OCE_HAL_MAX_NUM_MRQ_PAIRS);
			goto fail;
		}

		if (!hal->hal_mrq_used) {
			if (!hal->sli.config.features.flag.mrqp) {
				ocs_log_err(hal->os, "MRQ not supported by SLI4.\n");
				goto fail;
			}
			hal->hal_mrq_used = TRUE;
		}

		/* Create CQ set */
		if (hal_new_cq_set(mrq_sets[j].eqs, cqs, mrq_sets[j].num_pairs, default_lengths[QTOP_CQ]))
			goto fail;

		/* Create RQ set */
		if (hal_new_rq_set(cqs, rqs, mrq_sets[j].num_pairs, mrq_sets[j].len, mrq_sets[j].ulp))
			goto fail;

		for (i = 0; i < mrq_sets[j].num_pairs; i++) {
			rqs[i]->filter_mask = mrq_sets[j].filter_mask;
			rqs[i]->policy = mrq_sets[j].policy;
			rqs[i]->nvmeq = mrq_sets[j].nvmeq;
			rqs[i]->is_mrq = TRUE;
			rqs[i]->base_mrq_id = rqs[0]->hdr->id;
			rqs[i]->mrq_set_count = mrq_sets[j].num_pairs;
			rqs[i]->mrq_set_num = j;
			rqs[i]->protocol_valid = mrq_sets[j].protocol_valid;
			rqs[i]->protocol = mrq_sets[j].protocol;
		}
	}

	if (hal_new_els_cq(hal)) {
		if (!hal_new_els_wq(hal))
			goto fail;
	} else {
		goto fail;
	}

	/* Create CQ/WQ for Send Frames SCSI dev */
	if (ocs_hal_init_send_frame_queue(hal, OCS_HAL_WQ_SFQ_SCSI)) {
		ocs_log_err(hal->os, "Send frame queue create failed for SCSI dev\n");
		goto fail;
	}

	/* Create CQ/WQ for Send Frames NVMe dev */
	if (ocs_nvmeq_configured(hal) && ocs_hal_init_send_frame_queue(hal, OCS_HAL_WQ_SFQ_NVME)) {
		ocs_log_err(hal->os, "Send frame queue create failed for NVMe dev\n");
		goto fail;
	}

	return OCS_HAL_RTN_SUCCESS;

fail:
	hal_queue_teardown(hal);
	return OCS_HAL_RTN_ERROR;
}
#pragma GCC diagnostic pop

/**
 * @brief Allocate a new EQ object
 *
 * A new EQ object is instantiated
 *
 * @param hal pointer to HAL object
 * @param entry_count number of entries in the EQ
 *
 * @return pointer to allocated EQ object
 */
hal_eq_t*
hal_new_eq(ocs_hal_t *hal, uint32_t entry_count)
{
	hal_eq_t *eq = ocs_malloc(hal->os, sizeof(*eq), OCS_M_ZERO);

	if (eq != NULL) {
		eq->type = SLI_QTYPE_EQ;
		eq->hal = hal;
		eq->entry_count = entry_count;
		eq->instance = hal->eq_count++;
		eq->queue = hal->eq[eq->instance];
		ocs_list_init(&eq->cq_list, hal_cq_t, link);

		eq->wq_array = ocs_varray_alloc(hal->os, OCS_HAL_MAX_NUM_WQ);
		if (eq->wq_array == NULL) {
			ocs_free(hal->os, eq, sizeof(*eq));
			eq = NULL;
		} else {
			if (sli_queue_alloc(&hal->sli, SLI_QTYPE_EQ, eq->queue, entry_count, NULL, 0, false)) {
				ocs_log_err(hal->os, "EQ[%d] allocation failure\n", eq->instance);
				ocs_free(hal->os, eq, sizeof(*eq));
				eq = NULL;
			} else {
				sli_eq_modify_delay(&hal->sli, eq->queue, 1, 0, 8);
				hal->hal_eq[eq->instance] = eq;
				ocs_list_add_tail(&hal->eq_list, eq);
				ocs_log_debug(hal->os, "create eq[%2d] id %3d len %4d\n", eq->instance, eq->queue->id,
					eq->entry_count);
			}
		}
	}
	return eq;
}

/**
 * @brief Allocate a new CQ object
 *
 * A new CQ object is instantiated
 *
 * @param eq pointer to parent EQ object
 * @param entry_count number of entries in the CQ
 *
 * @return pointer to allocated CQ object
 */
hal_cq_t*
hal_new_cq(hal_eq_t *eq, uint32_t entry_count)
{
	ocs_hal_t *hal = eq->hal;
	hal_cq_t *cq = ocs_malloc(hal->os, sizeof(*cq), OCS_M_ZERO);

	if (cq != NULL) {
		cq->eq = eq;
		cq->type = SLI_QTYPE_CQ;
		cq->instance = eq->hal->cq_count++;
		cq->entry_count = entry_count;
		cq->queue = hal->cq[cq->instance];

		ocs_list_init(&cq->q_list, hal_q_t, link);

		if (sli_queue_alloc(&hal->sli, SLI_QTYPE_CQ, cq->queue, cq->entry_count, eq->queue, 0, false)) {
			ocs_log_err(hal->os, "CQ[%d] allocation failure len=%d\n",
				eq->instance,
				eq->entry_count);
			ocs_free(hal->os, cq, sizeof(*cq));
			cq = NULL;
		} else {
			hal->hal_cq[cq->instance] = cq;
			ocs_list_add_tail(&eq->cq_list, cq);
			ocs_log_debug(hal->os, "create cq[%2d] id %3d len %4d\n", cq->instance, cq->queue->id,
				cq->entry_count);
		}
	}
	return cq;
}

/**
 * @brief Allocate a new CQ Set of objects.
 *
 * @param eqs pointer to a set of EQ objects.
 * @param cqs pointer to a set of CQ objects to be returned.
 * @param num_cqs number of CQ queues in the set.
 * @param entry_count number of entries in the CQ.
 *
 * @return 0 on success and -1 on failure.
 */
uint32_t
hal_new_cq_set(hal_eq_t *eqs[], hal_cq_t *cqs[], uint32_t num_cqs, uint32_t entry_count)
{
	uint32_t i;
	ocs_hal_t *hal = eqs[0]->hal;
	sli4_t *sli4 = &hal->sli;
	hal_cq_t *cq = NULL;
	sli4_queue_t *qs[SLI_MAX_CQ_SET_COUNT], *assocs[SLI_MAX_CQ_SET_COUNT];

	/* Initialise CQS pointers to NULL */
	for (i = 0; i < num_cqs; i++) {
		cqs[i] = NULL;
	}

	for (i = 0; i < num_cqs; i++) {
		cq = ocs_malloc(hal->os, sizeof(*cq), OCS_M_ZERO);
		if (cq == NULL)
			goto error;

		cqs[i]          = cq;
		cq->eq          = eqs[i];
		cq->type        = SLI_QTYPE_CQ;
		cq->instance    = hal->cq_count++;
		cq->entry_count = entry_count;
		cq->queue       = hal->cq[cq->instance];
		qs[i]           = cq->queue;
		assocs[i]       = eqs[i]->queue;
		ocs_list_init(&cq->q_list, hal_q_t, link);
	}

	if (sli_cq_alloc_set(sli4, qs, num_cqs, entry_count, assocs)) {
		ocs_log_err(NULL, "Failed to create CQ Set\n");
		goto error;
	}

	for (i = 0; i < num_cqs; i++) {
		hal->hal_cq[cqs[i]->instance] = cqs[i];
		ocs_list_add_tail(&cqs[i]->eq->cq_list, cqs[i]);
	}

	return 0;

error:
	for (i = 0; i < num_cqs; i++) {
		if (cqs[i]) {
			ocs_free(hal->os, cqs[i], sizeof(*cqs[i]));
			cqs[i] = NULL;
		}
	}
	return -1;
}


/**
 * @brief Allocate a new MQ object
 *
 * A new MQ object is instantiated
 *
 * @param cq pointer to parent CQ object
 * @param entry_count number of entries in the MQ
 *
 * @return pointer to allocated MQ object
 */
hal_mq_t*
hal_new_mq(hal_cq_t *cq, uint32_t entry_count)
{
	ocs_hal_t *hal = cq->eq->hal;
	hal_mq_t *mq = ocs_malloc(hal->os, sizeof(*mq), OCS_M_ZERO);

	if (mq != NULL) {
		mq->cq = cq;
		mq->type = SLI_QTYPE_MQ;
		mq->instance = cq->eq->hal->mq_count++;
		mq->entry_count = entry_count;
		mq->queue = hal->mq[mq->instance];

		if (sli_queue_alloc(&hal->sli, SLI_QTYPE_MQ,
				    mq->queue,
				    mq->entry_count,
				    cq->queue, 0, false)) {
			ocs_log_err(hal->os, "MQ allocation failure\n");
			ocs_free(hal->os, mq, sizeof(*mq));
			mq = NULL;
		} else {
			hal->hal_mq[mq->instance] = mq;
			ocs_list_add_tail(&cq->q_list, mq);
			ocs_log_debug(hal->os, "create mq[%2d] id %3d len %4d\n", mq->instance, mq->queue->id,
				mq->entry_count);
		}
	}
	return mq;
}

/**
 * @brief Allocate a new WQ object
 *
 * A new WQ object is instantiated
 *
 * @param cq pointer to parent CQ object
 * @param entry_count number of entries in the WQ
 * @param class WQ class
 * @param ulp index of chute
 * @param sfq WQ use for Send Frames
 *
 * @return pointer to allocated WQ object
 */
hal_wq_t*
hal_new_wq(hal_cq_t *cq, uint32_t entry_count, uint32_t class, uint32_t ulp, bool sfq)
{
	ocs_hal_t *hal = cq->eq->hal;
	hal_wq_t *wq = ocs_malloc(hal->os, sizeof(*wq), OCS_M_ZERO);

	if (wq != NULL) {
		wq->hal = cq->eq->hal;
		wq->cq = cq;
		wq->type = SLI_QTYPE_WQ;
		wq->instance = cq->eq->hal->wq_count++;
		wq->entry_count = entry_count;
		wq->queue = hal->wq[wq->instance];
		wq->ulp = ulp;
		wq->wqec_set_count = OCS_HAL_WQEC_SET_COUNT;
		wq->wqec_count = wq->wqec_set_count;
		wq->free_count = wq->entry_count - 1;
		wq->class = class;
		ocs_list_init(&wq->pending_list, ocs_hal_wqe_t, link);

		if (sli_queue_alloc(&hal->sli, SLI_QTYPE_WQ, wq->queue, wq->entry_count, cq->queue, ulp, sfq)) {
			ocs_log_err(hal->os, "WQ allocation failure\n");
			ocs_free(hal->os, wq, sizeof(*wq));
			wq = NULL;
		} else {
			wq->fw_sfq_enabled = (sfq & wq->queue->sfq_resp);
			hal->hal_wq[wq->instance] = wq;
			ocs_list_add_tail(&cq->q_list, wq);
			ocs_log_debug(hal->os, "create wq[%2d] id %3d len %4d cls %d ulp %d\n", wq->instance, wq->queue->id,
				wq->entry_count, wq->class, wq->ulp);
		}
	}
	return wq;
}

/**
 * @brief Allocate a new ELS CQ object
 *
 * A new ELS CQ object is instantiated
 *
 * @param HAL object
 *
 * @return pointer to allocated WQ object
 */
hal_cq_t*
hal_new_els_cq(ocs_hal_t *hal)
{
	hal_cq_t *cq = NULL;

	ocs_hal_assert(hal);
	ocs_hal_assert(hal->hal_els_rq_cq);

	if (hal->hal_els_rq_cq->eq) {
		cq = hal_new_cq(hal->hal_els_rq_cq->eq, hal->num_qentries[SLI_QTYPE_CQ]);
		if (cq == NULL)
			ocs_log_err(hal->os, "Creating ELS CQ failed\n");
	}

	hal->hal_els_cq = cq;
	return cq;
}

/**
 * @brief Allocate a new ELS WQ object
 *
 * A new ELS WQ object is instantiated
 *
 * @param HAL object
 *
 * @return pointer to allocated WQ object
 */
hal_wq_t*
hal_new_els_wq(ocs_hal_t *hal)
{
	hal_wq_t *wq = NULL;

	ocs_hal_assert(hal);
	if (hal->hal_els_cq) {
		wq = hal_new_wq(hal->hal_els_cq, hal->num_qentries[SLI_QTYPE_WQ], 0, 0, false);
		if (wq == NULL)
			ocs_log_err(hal->os, "Creating ELS WQ failed\n");
	}

	hal->hal_els_wq = wq;
	return wq;
}

/**
 * @brief Allocate a hal_rq_t object
 *
 * Allocate an RQ object, which encapsulates 2 SLI queues (for rq pair)
 *
 * @param cq pointer to parent CQ object
 * @param entry_count number of entries in the RQs
 * @param ulp ULP index for this RQ
 *
 * @return pointer to newly allocated hal_rq_t
 */
hal_rq_t*
hal_new_rq(hal_cq_t *cq, uint32_t entry_count, uint32_t ulp)
{
	ocs_hal_t *hal = cq->eq->hal;
	hal_rq_t *rq = ocs_malloc(hal->os, sizeof(*rq), OCS_M_ZERO);

	if (rq != NULL) {
		rq->instance = hal->hal_rq_count++;
		rq->cq = cq;
		rq->type = SLI_QTYPE_RQ;
		rq->ulp = ulp;
		rq->entry_count = entry_count;

		/* Create the header RQ */
		ocs_hal_assert(hal->rq_count < OCS_HAL_MAX_NUM_RQ);
		rq->hdr = hal->rq[hal->rq_count];
		rq->hdr_entry_size = OCS_HAL_RQ_HEADER_SIZE;

		if (sli_fc_rq_alloc(&hal->sli, rq->hdr,
				    rq->entry_count,
				    rq->hdr_entry_size,
				    cq->queue,
				    ulp, TRUE)) {
			ocs_log_err(hal->os, "RQ allocation failure - header\n");
			ocs_free(hal->os, rq, sizeof(*rq));
			return NULL;
		}
		hal->hal_rq_lookup[hal->rq_count] = rq->instance;	/* Update hal_rq_lookup[] */
		/* Use the CQ associated with RQ index 0 as ELS CQ RQ */
		if (rq->instance == 0)
			hal->hal_els_rq_cq = cq;

		hal->rq_count++;
		ocs_log_debug(hal->os, "create rq[%2d] id %3d len %4d hdr  size %4d ulp %d\n",
			rq->instance, rq->hdr->id, rq->entry_count, rq->hdr_entry_size, rq->ulp);

		/* Create the default data RQ */
		ocs_hal_assert(hal->rq_count < OCS_HAL_MAX_NUM_RQ);
		rq->data = hal->rq[hal->rq_count];
		rq->data_entry_size = hal->config.rq_default_buffer_size;

		if (sli_fc_rq_alloc(&hal->sli, rq->data,
				    rq->entry_count,
				    rq->data_entry_size,
				    cq->queue,
				    ulp, FALSE)) {
			ocs_log_err(hal->os, "RQ allocation failure - first burst\n");
			ocs_free(hal->os, rq, sizeof(*rq));
			return NULL;
		}
		hal->hal_rq_lookup[hal->rq_count] = rq->instance;	/* Update hal_rq_lookup[] */
		hal->rq_count++;
		ocs_log_debug(hal->os, "create rq[%2d] id %3d len %4d data size %4d ulp %d\n", rq->instance,
			rq->data->id, rq->entry_count, rq->data_entry_size, rq->ulp);

		hal->hal_rq[rq->instance] = rq;
		ocs_list_add_tail(&cq->q_list, rq);

		rq->rq_tracker = ocs_malloc(hal->os, sizeof(ocs_hal_sequence_t*) *
					    rq->entry_count, OCS_M_ZERO);
		if (rq->rq_tracker == NULL) {
			ocs_log_err(hal->os, "RQ tracker buf allocation failure\n");
			return NULL;
		}
	}
	return rq;
}


/**
 * @brief Allocate a hal_rq_t object SET
 *
 * Allocate an RQ object SET, where each element in set
 * encapsulates 2 SLI queues (for rq pair)
 *
 * @param cqs pointers to be associated with RQs.
 * @param rqs RQ pointers to be returned on success.
 * @param num_rq_pairs number of rq pairs in the Set.
 * @param entry_count number of entries in the RQs
 * @param ulp ULP index for this RQ
 *
 * @return 0 in success and -1 on failure.
 */
uint32_t
hal_new_rq_set(hal_cq_t *cqs[], hal_rq_t *rqs[], uint32_t num_rq_pairs, uint32_t entry_count, uint32_t ulp)
{
	ocs_hal_t *hal = cqs[0]->eq->hal;
	hal_rq_t *rq = NULL;
	sli4_queue_t *qs[SLI_MAX_RQ_SET_COUNT * 2] = { NULL };
	uint32_t i, q_count;

	/* Initialise RQS pointers */
	for (i = 0; i < num_rq_pairs; i++) {
		rqs[i] = NULL;
	}

	for (i = 0, q_count = 0; i < num_rq_pairs; i++, q_count += 2) {
		rq = ocs_malloc(hal->os, sizeof(*rq), OCS_M_ZERO);
		if (rq == NULL)
			goto error;

		rqs[i] = rq;
		rq->instance = hal->hal_rq_count++;
		rq->cq = cqs[i];
		rq->type = SLI_QTYPE_RQ;
		rq->ulp = ulp;
		rq->entry_count = entry_count;

		/* Header RQ */
		rq->hdr = hal->rq[hal->rq_count];
		rq->hdr_entry_size = OCS_HAL_RQ_HEADER_SIZE;
		hal->hal_rq_lookup[hal->rq_count] = rq->instance;
		hal->rq_count++;
		qs[q_count] = rq->hdr;

		/* Data RQ */
		rq->data = hal->rq[hal->rq_count];
		rq->data_entry_size = hal->config.rq_default_buffer_size;
		hal->hal_rq_lookup[hal->rq_count] = rq->instance;
		hal->rq_count++;
		qs[q_count + 1] = rq->data;

		rq->rq_tracker = NULL;
	}

	if (sli_fc_rq_set_alloc(&hal->sli, num_rq_pairs, qs,
			    cqs[0]->queue->id,
			    rqs[0]->entry_count,
			    rqs[0]->hdr_entry_size,
			    rqs[0]->data_entry_size,
			    ulp)) {
		ocs_log_err(hal->os, "RQ Set allocation failure for base CQ=%d\n", cqs[0]->queue->id);
		goto error;
	}


	for (i = 0; i < num_rq_pairs; i++) {
		hal->hal_rq[rqs[i]->instance] = rqs[i];
		ocs_list_add_tail(&cqs[i]->q_list, rqs[i]);
		rqs[i]->rq_tracker = ocs_malloc(hal->os, sizeof(ocs_hal_sequence_t*) *
					    rqs[i]->entry_count, OCS_M_ZERO);
		if (rqs[i]->rq_tracker == NULL) {
			ocs_log_err(hal->os, "RQ tracker buf allocation failure\n");
			goto error;
		}
	}

	return 0;

error:
	for (i = 0; i < num_rq_pairs; i++) {
		if (rqs[i] != NULL) {
			if (rqs[i]->rq_tracker != NULL) {
				ocs_free(hal->os, rqs[i]->rq_tracker,
					 sizeof(ocs_hal_sequence_t*) * rqs[i]->entry_count);
			}
			ocs_free(hal->os, rqs[i], sizeof(*rqs[i]));
		}
	}

	return -1;
}


/**
 * @brief Free an EQ object
 *
 * The EQ object and any child queue objects are freed
 *
 * @param eq pointer to EQ object
 *
 * @return none
 */
void
hal_del_eq(hal_eq_t *eq)
{
	if (eq != NULL) {
		hal_cq_t *cq;
		hal_cq_t *cq_next;

		ocs_list_foreach_safe(&eq->cq_list, cq, cq_next) {
			hal_del_cq(cq);
		}
		ocs_varray_free(eq->wq_array);
		ocs_list_remove(&eq->hal->eq_list, eq);
		eq->hal->hal_eq[eq->instance] = NULL;
		ocs_free(eq->hal->os, eq, sizeof(*eq));
	}
}

/**
 * @brief Free a CQ object
 *
 * The CQ object and any child queue objects are freed
 *
 * @param cq pointer to CQ object
 *
 * @return none
 */
void
hal_del_cq(hal_cq_t *cq)
{
	if (cq != NULL) {
		hal_q_t *q;
		hal_q_t *q_next;

		ocs_list_foreach_safe(&cq->q_list, q, q_next) {
			switch(q->type) {
			case SLI_QTYPE_MQ:
				hal_del_mq((hal_mq_t*) q);
				break;
			case SLI_QTYPE_WQ:
				hal_del_wq((hal_wq_t*) q);
				break;
			case SLI_QTYPE_RQ:
				hal_del_rq((hal_rq_t*) q);
				break;
			default:
				break;
			}
		}

		ocs_list_remove(&cq->eq->cq_list, cq);
		if (cq->eq->hal->hal_els_cq == cq)
			cq->eq->hal->hal_els_cq = NULL;

		cq->eq->hal->hal_cq[cq->instance] = NULL;
		ocs_free(cq->eq->hal->os, cq, sizeof(*cq));
	}
}

/**
 * @brief Free a MQ object
 *
 * The MQ object is freed
 *
 * @param mq pointer to MQ object
 *
 * @return none
 */
void
hal_del_mq(hal_mq_t *mq)
{
	if (mq != NULL) {
		ocs_list_remove(&mq->cq->q_list, mq);
		mq->cq->eq->hal->hal_mq[mq->instance] = NULL;
		ocs_free(mq->cq->eq->hal->os, mq, sizeof(*mq));
	}
}

/**
 * @brief Free a WQ object
 *
 * The WQ object is freed
 *
 * @param wq pointer to WQ object
 *
 * @return none
 */
void
hal_del_wq(hal_wq_t *wq)
{
	if (wq != NULL) {
		ocs_list_remove(&wq->cq->q_list, wq);
		if (wq == wq->cq->eq->hal->hal_els_wq)
			wq->cq->eq->hal->hal_els_wq = NULL;

		wq->cq->eq->hal->hal_wq[wq->instance] = NULL;
		ocs_free(wq->cq->eq->hal->os, wq, sizeof(*wq));
	}
}

/**
 * @brief Free an RQ object
 *
 * The RQ object is freed
 *
 * @param rq pointer to RQ object
 *
 * @return none
 */
void
hal_del_rq(hal_rq_t *rq)
{
	ocs_hal_t *hal = NULL;

	if (rq != NULL) {
		hal = rq->cq->eq->hal;

		/* Free RQ tracker */
		if (rq->rq_tracker != NULL) {
			ocs_free(hal->os, rq->rq_tracker, sizeof(ocs_hal_sequence_t*) * rq->entry_count);
			rq->rq_tracker = NULL;
		}

		ocs_list_remove(&rq->cq->q_list, rq);
		hal->hal_rq[rq->instance] = NULL;
		ocs_free(hal->os, rq, sizeof(*rq));
	}
}

/**
 * @brief Display HAL queue objects
 *
 * The HAL queue objects are displayed using ocs_log
 *
 * @param hal pointer to HAL object
 *
 * @return none
 */
void
hal_queue_dump(ocs_hal_t *hal)
{
	hal_eq_t *eq;
	hal_cq_t *cq;
	hal_q_t *q;
	hal_mq_t *mq;
	hal_wq_t *wq;
	hal_rq_t *rq;

	ocs_list_foreach(&hal->eq_list, eq) {
		ocs_printf("eq[%d] id %2d\n", eq->instance, eq->queue->id);
		ocs_list_foreach(&eq->cq_list, cq) {
			ocs_printf("  cq[%d] id %2d current\n", cq->instance, cq->queue->id);
			ocs_list_foreach(&cq->q_list, q) {
				switch(q->type) {
				case SLI_QTYPE_MQ:
					mq = (hal_mq_t *) q;
					ocs_printf("    mq[%d] id %2d\n", mq->instance, mq->queue->id);
					break;
				case SLI_QTYPE_WQ:
					wq = (hal_wq_t *) q;
					ocs_printf("    wq[%d] id %2d\n", wq->instance, wq->queue->id);
					break;
				case SLI_QTYPE_RQ:
					rq = (hal_rq_t *) q;
					ocs_printf("    rq[%d] hdr id %2d\n", rq->instance, rq->hdr->id);
					break;
				default:
					break;
				}
			}
		}
	}
}

/**
 * @brief Teardown HAL queue objects
 *
 * The HAL queue objects are freed
 *
 * @param hal pointer to HAL object
 *
 * @return none
 */
void
hal_queue_teardown(ocs_hal_t *hal)
{
	uint32_t i;
	hal_eq_t *eq;
	hal_eq_t *eq_next;

	if (ocs_list_valid(&hal->eq_list)) {
		ocs_list_foreach_safe(&hal->eq_list, eq, eq_next) {
			hal_del_eq(eq);
		}
	}
	for (i = 0; i < ARRAY_SIZE(hal->wq_cpu_array); i++) {
		ocs_varray_free(hal->wq_cpu_array[i]);
		hal->wq_cpu_array[i] = NULL;
	}
	for (i = 0; i < ARRAY_SIZE(hal->wq_class_array); i++) {
		ocs_varray_free(hal->wq_class_array[i]);
		hal->wq_class_array[i] = NULL;
	}
}

/**
 * @brief Allocate a WQ to an IO object
 *
 * The next work queue index is used to assign a WQ to an IO.
 *
 * If wq_steering is OCS_HAL_WQ_STEERING_CLASS, a WQ from io->wq_class is
 * selected.
 *
 * If wq_steering is OCS_HAL_WQ_STEERING_REQUEST, then a WQ from the EQ that
 * the IO request came in on is selected.
 *
 * If wq_steering is OCS_HAL_WQ_STEERING_CPU, then a WQ associted with the
 * CPU the request is made on is selected.
 *
 * @param hal pointer to HAL object
 * @param io pointer to IO object
 *
 * @return Return pointer to next WQ
 */
hal_wq_t *
ocs_hal_queue_next_wq(ocs_hal_t *hal, ocs_hal_io_t *io)
{
	hal_eq_t *eq;
	hal_wq_t *wq = NULL;

	switch(io->wq_steering) {
	case OCS_HAL_WQ_STEERING_CLASS:
		if (likely(io->wq_class < ARRAY_SIZE(hal->wq_class_array))) {
			wq = ocs_varray_iter_next(hal->wq_class_array[io->wq_class]);
		}
		break;
	case OCS_HAL_WQ_STEERING_REQUEST:
		eq = io->eq;
		if (likely(eq != NULL)) {
			wq = ocs_varray_iter_next(eq->wq_array);
		}
		break;
	case OCS_HAL_WQ_STEERING_CPU: {
		uint32_t cpuidx = ocs_thread_getcpu();

		if (likely(cpuidx < ARRAY_SIZE(hal->wq_cpu_array))) {
			wq = ocs_varray_iter_next(hal->wq_cpu_array[cpuidx]);
		}
		break;
	}
	}

	if (unlikely(wq == NULL)) {
		wq = hal->hal_wq[0];
	}

	return wq;
}

/**
 * @brief Return count of EQs for a queue topology object
 *
 * The EQ count for in the HALs queue topology (hal->qtop) object is returned
 *
 * @param hal pointer to HAL object
 *
 * @return count of EQs
 */
uint32_t
ocs_hal_qtop_eq_count(ocs_hal_t *hal)
{
	return hal->qtop->entry_counts[QTOP_EQ];
}

#define TOKEN_LEN		32

/**
 * @brief return string given a QTOP entry
 *
 * @param entry QTOP entry
 *
 * @return returns string or "unknown"
 */
#if HAL_QTOP_DEBUG
static char *
qtopentry2s(ocs_hal_qtop_entry_e entry) {
	switch(entry) {
	#define P(x)	case x: return #x;
	P(QTOP_EQ)
	P(QTOP_CQ)
	P(QTOP_WQ)
	P(QTOP_RQ)
	P(QTOP_MQ)
	P(QTOP_THREAD_START)
	P(QTOP_THREAD_END)
	P(QTOP_LAST)
	#undef P
	}
	return "unknown";
}
#endif

/**
 * @brief Declare token types
 */
typedef enum {
	TOK_LPAREN = 1,
	TOK_RPAREN,
	TOK_COLON,
	TOK_EQUALS,
	TOK_QUEUE,
	TOK_ATTR_NAME,
	TOK_NUMBER,
	TOK_NUMBER_VALUE,
	TOK_NUMBER_LIST,
} tok_type_e;

/**
 * @brief Declare token sub-types
 */
typedef enum {
	TOK_SUB_EQ = 100,
	TOK_SUB_CQ,
	TOK_SUB_RQ,
	TOK_SUB_MQ,
	TOK_SUB_WQ,
	TOK_SUB_LEN,
	TOK_SUB_CLASS,
	TOK_SUB_ULP,
	TOK_SUB_FILTER,
	TOK_SUB_RQ_POLICY,
	TOK_SUB_NVMEQ,
} tok_subtype_e;

/**
 * @brief convert queue subtype to QTOP entry
 *
 * @param q queue subtype
 *
 * @return QTOP entry or 0
 */
static ocs_hal_qtop_entry_e
subtype2qtop(tok_subtype_e q)
{
	switch(q) {
	case TOK_SUB_EQ:	return QTOP_EQ;
	case TOK_SUB_CQ:	return QTOP_CQ;
	case TOK_SUB_RQ:	return QTOP_RQ;
	case TOK_SUB_MQ:	return QTOP_MQ;
	case TOK_SUB_WQ:	return QTOP_WQ;
	default:
		break;
	}
	return 0;
}

/**
 * @brief Declare token object
 */
typedef struct {
	tok_type_e type;
	tok_subtype_e subtype;
	char string[TOKEN_LEN];
} tok_t;

/**
 * @brief Declare token array object
 */
typedef struct {
	tok_t *tokens;			/* Pointer to array of tokens */
	uint32_t alloc_count;		/* Number of tokens in the array */
	uint32_t inuse_count;		/* Number of tokens posted to array */
	uint32_t iter_idx;		/* Iterator index */
} tokarray_t;

/**
 * @brief Declare token match structure
 */
typedef struct {
	char *s;
	tok_type_e type;
	tok_subtype_e subtype;
} tokmatch_t;

/**
 * @brief test if character is ID start character
 *
 * @param c character to test
 *
 * @return TRUE if character is an ID start character
 */
static int32_t
idstart(int c)
{
	return	isalpha(c) || (c == '_') || (c == '$');
}

/**
 * @brief test if character is an ID character
 *
 * @param c character to test
 *
 * @return TRUE if character is an ID character
 */
static int32_t
idchar(int c)
{
	return idstart(c) || ocs_isdigit(c);
}

/**
 * @brief Declare single character matches
 */
static tokmatch_t cmatches[] = {
	{"(", TOK_LPAREN},
	{")", TOK_RPAREN},
	{":", TOK_COLON},
	{"=", TOK_EQUALS},
};

/**
 * @brief Declare identifier match strings
 */
static tokmatch_t smatches[] = {
	{"eq", TOK_QUEUE, TOK_SUB_EQ},
	{"cq", TOK_QUEUE, TOK_SUB_CQ},
	{"rq", TOK_QUEUE, TOK_SUB_RQ},
	{"mq", TOK_QUEUE, TOK_SUB_MQ},
	{"wq", TOK_QUEUE, TOK_SUB_WQ},
	{"len", TOK_ATTR_NAME, TOK_SUB_LEN},
	{"class", TOK_ATTR_NAME, TOK_SUB_CLASS},
	{"ulp", TOK_ATTR_NAME, TOK_SUB_ULP},
	{"filter", TOK_ATTR_NAME, TOK_SUB_FILTER},
	{"rqpolicy", TOK_ATTR_NAME, TOK_SUB_RQ_POLICY},
	{"nvmeq", TOK_ATTR_NAME, TOK_SUB_NVMEQ},
};

/**
 * @brief Scan string and return next token
 *
 * The string is scanned and the next token is returned
 *
 * @param s input string to scan
 * @param tok pointer to place scanned token
 *
 * @return pointer to input string following scanned token, or NULL
 */
static const char *
tokenize(const char *s, tok_t *tok)
{
	uint32_t i;

	ocs_memset(tok, 0, sizeof(*tok));

	/* Skip over whitespace */
	while (*s && ocs_isspace(*s)) {
		s++;
	}

	/* Return if nothing left in this string */
	if (*s == 0) {
		return NULL;
	}

	/* Look for single character matches */
	for (i = 0; i < ARRAY_SIZE(cmatches); i++) {
		if (cmatches[i].s[0] == *s) {
			tok->type = cmatches[i].type;
			tok->subtype = cmatches[i].subtype;
			tok->string[0] = *s++;
			return s;
		}
	}

	/* Scan for a hex number or decimal */
	if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X'))) {
		char *p = tok->string;

		tok->type = TOK_NUMBER;

		*p++ = *s++;
		*p++ = *s++;
		while ((*s == '.') || ocs_isxdigit(*s)) {
			if ((p - tok->string) < (int32_t)sizeof(tok->string)) {
				*p++ = *s;
			}
			if (*s == ',') {
				tok->type = TOK_NUMBER_LIST;
			}
			s++;
		}
		*p = 0;
		return s;
	} else if (ocs_isdigit(*s)) {
		char *p = tok->string;

		tok->type = TOK_NUMBER;
		while ((*s == ',') || ocs_isdigit(*s)) {
			if ((p - tok->string) < (int32_t)sizeof(tok->string)) {
				*p++ = *s;
			}
			if (*s == ',') {
				tok->type = TOK_NUMBER_LIST;
			}
			s++;
		}
		*p = 0;
		return s;
	}

	/* Scan for an ID */
	if (idstart(*s)) {
		char *p = tok->string;

		for (*p++ = *s++; idchar(*s); s++) {
			if ((p - tok->string) < TOKEN_LEN) {
				*p++ = *s;
			}
		}

		/* See if this is a $ number value */
		if (tok->string[0] == '$') {
			tok->type = TOK_NUMBER_VALUE;
		} else {
			/* Look for a string match */
			for (i = 0; i < ARRAY_SIZE(smatches); i++) {
				if (strcmp(smatches[i].s, tok->string) == 0) {
					tok->type = smatches[i].type;
					tok->subtype = smatches[i].subtype;
					return s;
				}
			}
		}
	}
	return s;
}

/**
 * @brief convert token type to string
 *
 * @param type token type
 *
 * @return string, or "unknown"
 */
static const char *
token_type2s(tok_type_e type)
{
	switch(type) {
	#define P(x)	case x: return #x;
	P(TOK_LPAREN)
	P(TOK_RPAREN)
	P(TOK_COLON)
	P(TOK_EQUALS)
	P(TOK_QUEUE)
	P(TOK_ATTR_NAME)
	P(TOK_NUMBER)
	P(TOK_NUMBER_VALUE)
	P(TOK_NUMBER_LIST)
	#undef P
	}
	return "unknown";
}

/**
 * @brief convert token sub-type to string
 *
 * @param subtype token sub-type
 *
 * @return string, or "unknown"
 */
static const char *
token_subtype2s(tok_subtype_e subtype)
{
	switch(subtype) {
	#define P(x)	case x: return #x;
	P(TOK_SUB_EQ)
	P(TOK_SUB_CQ)
	P(TOK_SUB_RQ)
	P(TOK_SUB_MQ)
	P(TOK_SUB_WQ)
	P(TOK_SUB_LEN)
	P(TOK_SUB_CLASS)
	P(TOK_SUB_ULP)
	P(TOK_SUB_FILTER)
	P(TOK_SUB_RQ_POLICY)
	P(TOK_SUB_NVMEQ)
	#undef P
	}
	return "";
}

/**
 * @brief Generate syntax error message
 *
 * A syntax error message is found, the input tokens are dumped up to and including
 * the token that failed as indicated by the current iterator index.
 *
 * @param hal pointer to HAL object
 * @param tokarray pointer to token array object
 *
 * @return none
 */
static void
tok_syntax(ocs_hal_t *hal, tokarray_t *tokarray)
{
	uint32_t i;
	tok_t *tok;

	ocs_log_test(hal->os, "Syntax error:\n");

	for (i = 0, tok = tokarray->tokens; (i <= tokarray->inuse_count); i++, tok++) {
		ocs_log_test(hal->os, "%s [%2d]    %-16s %-16s %s\n", (i == tokarray->iter_idx) ? ">>>" : "   ", i,
			token_type2s(tok->type), token_subtype2s(tok->subtype), tok->string);
	}
}

/**
 * @brief parse a number
 *
 * Parses tokens of type TOK_NUMBER and TOK_NUMBER_VALUE, returning a numeric value
 *
 * @param hal pointer to HAL object
 * @param qtop pointer to QTOP object
 * @param tok pointer to token to parse
 *
 * @return numeric value
 */
static uint32_t
tok_getnumber(ocs_hal_t *hal, ocs_hal_qtop_t *qtop, tok_t *tok)
{
	uint32_t rval = 0;
	uint32_t num_cpus = ocs_get_num_cpus();

	switch(tok->type) {
	case TOK_NUMBER_VALUE:
		if (ocs_strcmp(tok->string, "$ncpu") == 0) {
			rval = num_cpus;
		} else if (ocs_strcmp(tok->string, "$ncpu1") == 0) {
			rval = num_cpus - 1;
		} else if (ocs_strcmp(tok->string, "$nwq") == 0) {
			if (hal != NULL) {
				rval = hal->config.n_wq;
			}
		} else if (ocs_strcmp(tok->string, "$maxmrq") == 0) {
			rval = MIN(num_cpus, OCS_HAL_MAX_MRQS);
		} else if (ocs_strcmp(tok->string, "$nulp") == 0) {
			rval = hal->ulp_max - hal->ulp_start + 1;
		} else if ((qtop->rptcount_idx > 0) && ocs_strcmp(tok->string, "$rpt0") == 0) {
			rval = qtop->rptcount[qtop->rptcount_idx-1];
		} else if ((qtop->rptcount_idx > 1) && ocs_strcmp(tok->string, "$rpt1") == 0) {
			rval = qtop->rptcount[qtop->rptcount_idx-2];
		} else if ((qtop->rptcount_idx > 2) && ocs_strcmp(tok->string, "$rpt2") == 0) {
			rval = qtop->rptcount[qtop->rptcount_idx-3];
		} else if ((qtop->rptcount_idx > 3) && ocs_strcmp(tok->string, "$rpt3") == 0) {
			rval = qtop->rptcount[qtop->rptcount_idx-4];
		} else {
			rval = ocs_strtoul(tok->string, 0, 0);
		}
		break;
	case TOK_NUMBER:
		rval = ocs_strtoul(tok->string, 0, 0);
		break;
	default:
		break;
	}
	return rval;
}


/**
 * @brief parse an array of tokens
 *
 * The tokens are semantically parsed, to generate QTOP entries.
 *
 * @param hal pointer to HAL object
 * @param tokarray array array of tokens
 * @param qtop ouptut QTOP object
 *
 * @return returns 0 for success, a negative error code value for failure.
 */
static int32_t
parse_topology(ocs_hal_t *hal, tokarray_t *tokarray, ocs_hal_qtop_t *qtop)
{
	ocs_hal_qtop_entry_t *qt = qtop->entries + qtop->inuse_count;
	tok_t *tok;

	for (; (tokarray->iter_idx < tokarray->inuse_count) &&
	     ((tok = &tokarray->tokens[tokarray->iter_idx]) != NULL); ) {
		if (qtop->inuse_count >= qtop->alloc_count) {
			return -1;
		}

		qt = qtop->entries + qtop->inuse_count;

		switch (tok[0].type)
		{
		case TOK_QUEUE:
			qt->entry = subtype2qtop(tok[0].subtype);
			qt->set_default = FALSE;
			qt->len = 0;
			qt->class = 0;
			qtop->inuse_count++;

			tokarray->iter_idx++;		/* Advance current token index */

			/* Parse for queue attributes, possibly multiple instances */
			while ((tokarray->iter_idx + 4) <= tokarray->inuse_count) {
				tok = &tokarray->tokens[tokarray->iter_idx];
				if(	(tok[0].type == TOK_COLON) &&
					(tok[1].type == TOK_ATTR_NAME) &&
					(tok[2].type == TOK_EQUALS) &&
					((tok[3].type == TOK_NUMBER) ||
					 (tok[3].type == TOK_NUMBER_VALUE) ||
					 (tok[3].type == TOK_NUMBER_LIST))) {

					switch (tok[1].subtype) {
					case TOK_SUB_LEN:
						qt->len = tok_getnumber(hal, qtop, &tok[3]);
						break;

					case TOK_SUB_CLASS:
						qt->class = tok_getnumber(hal, qtop, &tok[3]);
						break;

					case TOK_SUB_ULP:
						qt->ulp = tok_getnumber(hal, qtop, &tok[3]);
						break;

					case TOK_SUB_RQ_POLICY:
						qt->policy = tok_getnumber(hal, qtop, &tok[3]);
						break;

					case TOK_SUB_NVMEQ:
						qt->nvmeq = tok_getnumber(hal, qtop, &tok[3]);
						break;

					case TOK_SUB_FILTER:
						if (tok[3].type == TOK_NUMBER_LIST) {
							uint32_t mask = 0;
							char *p = tok[3].string;

							while ((p != NULL) && *p) {
								uint32_t v;

								v = ocs_strtoul(p, 0, 0);
								if (v < 32) {
									mask |= (1U << v);
								}

								p = ocs_strchr(p, ',');
								if (p != NULL) {
									p++;
								}
							}
							qt->filter_mask = mask;
						} else {
							qt->filter_mask = (1U << tok_getnumber(hal, qtop, &tok[3]));
						}
						break;

					default:
						break;
					}
					/* Advance current token index */
					tokarray->iter_idx += 4;
				} else {
					break;
				}
			}
			qtop->entry_counts[qt->entry]++;
			break;

		case TOK_ATTR_NAME:
			if (	((tokarray->iter_idx + 5) <= tokarray->inuse_count) &&
				(tok[1].type == TOK_COLON) &&
				(tok[2].type == TOK_QUEUE) &&
				(tok[3].type == TOK_EQUALS) &&
				((tok[4].type == TOK_NUMBER) || (tok[4].type == TOK_NUMBER_VALUE))) {
				qt->entry = subtype2qtop(tok[2].subtype);
				qt->set_default = TRUE;
				switch(tok[0].subtype) {
				case TOK_SUB_LEN:
					qt->len = tok_getnumber(hal, qtop, &tok[4]);
					break;
				case TOK_SUB_CLASS:
					qt->class = tok_getnumber(hal, qtop, &tok[4]);
					break;
				case TOK_SUB_ULP:
					qt->ulp = tok_getnumber(hal, qtop, &tok[4]);
					break;
				default:
					break;
				}
				qtop->inuse_count++;
				tokarray->iter_idx += 5;
			} else {
				tok_syntax(hal, tokarray);
				return -1;
			}
			break;

		case TOK_NUMBER:
		case TOK_NUMBER_VALUE: {
			uint32_t rpt_count = 1;
			uint32_t i;

			rpt_count = tok_getnumber(hal, qtop, tok);

			if (tok[1].type == TOK_LPAREN) {
				uint32_t iter_idx_save;

				tokarray->iter_idx += 2;

				/* save token array iteration index */
				iter_idx_save = tokarray->iter_idx;

				for (i = 0; i < rpt_count; i++) {
					uint32_t rptcount_idx = qtop->rptcount_idx;

					if (qtop->rptcount_idx < ARRAY_SIZE(qtop->rptcount)) {
						qtop->rptcount[qtop->rptcount_idx++] = i;
					}

					/* restore token array iteration index */
					tokarray->iter_idx = iter_idx_save;

					/* parse, append to qtop */
					parse_topology(hal, tokarray, qtop);

					qtop->rptcount_idx = rptcount_idx;
				}
			}
			break;
		}

		case TOK_RPAREN:
			tokarray->iter_idx++;
			return 0;

		default:
			tok_syntax(hal, tokarray);
			return -1;
		}
	}
	return 0;
}

/**
 * @brief Parse queue topology string
 *
 * The queue topology object is allocated, and filled with the results of parsing the
 * passed in queue topology string
 *
 * @param hal pointer to HAL object
 * @param qtop_string input queue topology string
 *
 * @return pointer to allocated QTOP object, or NULL if there was an error
 */
ocs_hal_qtop_t *
ocs_hal_qtop_parse(ocs_hal_t *hal, const char *qtop_string)
{
	ocs_hal_qtop_t *qtop;
	tokarray_t tokarray;
	const char *s;
#if HAL_QTOP_DEBUG
	uint32_t i;
	ocs_hal_qtop_entry_t *qt;
#endif

	ocs_log_debug(hal->os, "queue topology: %s\n", qtop_string);

	/* Allocate a token array */
	tokarray.tokens = ocs_malloc(hal->os, MAX_TOKENS * sizeof(*tokarray.tokens), OCS_M_ZERO | OCS_M_NONUMA);
	if (tokarray.tokens == NULL) {
		return NULL;
	}
	tokarray.alloc_count = MAX_TOKENS;
	tokarray.inuse_count = 0;
	tokarray.iter_idx = 0;

	/* Parse the tokens */
	for (s = qtop_string; (tokarray.inuse_count < tokarray.alloc_count) &&
	     ((s = tokenize(s, &tokarray.tokens[tokarray.inuse_count]))) != NULL; ) {
		tokarray.inuse_count++;;
	}

	/* Allocate a queue topology structure */
	qtop = ocs_malloc(hal->os, sizeof(*qtop), OCS_M_ZERO);
	if (qtop == NULL) {
		ocs_free(hal->os, tokarray.tokens, MAX_TOKENS * sizeof(*tokarray.tokens));
		ocs_log_err(hal->os, "malloc qtop failed\n");
		return NULL;
	}
	qtop->os = hal->os;

	/* Allocate queue topology entries */
	qtop->entries = ocs_malloc(hal->os, OCS_HAL_MAX_QTOP_ENTRIES * sizeof(*qtop->entries),
				   OCS_M_ZERO | OCS_M_NONUMA);
	if (qtop->entries == NULL) {
		ocs_log_err(hal->os, "malloc qtop entries failed\n");
		ocs_free(hal->os, qtop, sizeof(*qtop));
		ocs_free(hal->os, tokarray.tokens, MAX_TOKENS * sizeof(*tokarray.tokens));
		return NULL;
	}
	qtop->alloc_count = OCS_HAL_MAX_QTOP_ENTRIES;
	qtop->inuse_count = 0;

	/* Parse the tokens */
	if (parse_topology(hal, &tokarray, qtop)) {
		ocs_log_err(hal->os, "failed to parse tokens\n");
		ocs_hal_qtop_free(qtop);
		ocs_free(hal->os, tokarray.tokens, MAX_TOKENS * sizeof(*tokarray.tokens));
		return NULL;
	}

#if HAL_QTOP_DEBUG
	for (i = 0, qt = qtop->entries; i < qtop->inuse_count; i++, qt++) {
		ocs_log_debug(hal->os, "entry %s set_df %d len %4d class %d ulp %d\n", qtopentry2s(qt->entry), qt->set_default, qt->len,
		       qt->class, qt->ulp);
	}
#endif

	/* Free the tokens array */
	ocs_free(hal->os, tokarray.tokens, MAX_TOKENS * sizeof(*tokarray.tokens));

	return qtop;
}

/**
 * @brief free queue topology object
 *
 * @param qtop pointer to QTOP object
 *
 * @return none
 */
void
ocs_hal_qtop_free(ocs_hal_qtop_t *qtop)
{
	if (qtop != NULL) {
		if (qtop->entries != NULL) {
			ocs_free(qtop->os, qtop->entries, qtop->alloc_count*sizeof(*qtop->entries));
		}
		ocs_free(qtop->os, qtop, sizeof(*qtop));
	}
}
