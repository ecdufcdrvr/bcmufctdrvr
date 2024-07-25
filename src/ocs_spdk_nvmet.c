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

#include <pthread.h>
#include "ocs.h"
#include "ocs_os.h"
#include "ocs_els.h"
#include "ocs_device.h"
#include "ocs_spdk_nvmet.h"
#include "ocs_impl_spdk.h"
#include "spdk_nvmf_xport.h"
#include "spdk_internal/event.h"

struct nvme_api_ctx {
	int rc;
	sem_t sem;
};

static void
ocs_nvme_api_call_done(uint8_t port_handle, enum spdk_fc_event event_type,
		       void *in, int err)
{
	struct nvme_api_ctx *ctx = in;

	ctx->rc = err;
	sem_post(&ctx->sem);
}

static int 
ocs_nvme_api_call_sync(enum spdk_fc_event event_type, void *args, void **cb_args)
{
	struct nvme_api_ctx ctx;

	memset(&ctx, 0, sizeof(struct nvme_api_ctx));
	sem_init(&ctx.sem, 1, 0);

	ctx.rc = SPDK_ERR_INTERNAL;
	*cb_args = &ctx;

	if (nvmf_fc_main_enqueue_event(event_type, args, ocs_nvme_api_call_done)) {
                goto done;
        }

	/* Block untill api is done. */
	sem_wait(&ctx.sem);
done:
	sem_destroy(&ctx.sem);
	return ctx.rc;
}

static void
ocs_fill_nvme_sli_queue(ocs_t *ocs,
		sli4_queue_t *sli4_q,
		bcm_sli_queue_t *sli_q)

{
	sli_q->qid = sli4_q->id;
	sli_q->size = sli4_q->size;
	sli_q->address = sli4_q->dma_single_chunk.virt;
	sli_q->max_entries = sli4_q->length;
	sli_q->doorbell_reg =
		ocs->ocs_os.bars[sli4_q->doorbell_rset].vaddr +
		sli4_q->doorbell_offset;
	sli_q->if_type = ocs->hal.sli.if_type;

	if (sli4_q->type == SLI_QTYPE_WQ &&
	    sli_q->if_type == SLI4_IF_TYPE_LANCER_G7 &&
	    sli4_q->u.dpp.enabled) {
		sli_q->dpp_enabled = true;
		sli_q->dpp_id = sli4_q->u.dpp.id;
		sli_q->dpp_doorbell_reg =
			ocs->ocs_os.bars[sli4_q->u.dpp.db_rset].vaddr +
				sli4_q->u.dpp.db_offset;
	}

	/* Used only by eq and cq */
	sli_q->phase = 1;

	/* Assign a unique name */
	snprintf(sli_q->name, sizeof(sli_q->name), "ocs%d-sliq-type%d-qid%d",
			ocs->instance_index, sli4_q->type, sli4_q->id);
}

static void
ocs_free_nvme_buffers(char *memzone_name, struct spdk_nvmf_fc_buffer_desc *buffers)
{
	if (!buffers) {
		return;
	}

	/*
	 * We just need to free the first buffer pointed address
	 * Because all the buffers are allocated in one shot.
	 */
	if (buffers->virt) {
		spdk_dma_free(buffers->virt);
	}

	free(buffers);
}

static struct spdk_nvmf_fc_buffer_desc *
ocs_alloc_nvme_buffers(char *name, int size, int num_entries)
{
	int i;
	void *virt;
	struct spdk_nvmf_fc_buffer_desc *buffers = NULL, *buffer;

	buffers = calloc(num_entries, sizeof(struct spdk_nvmf_fc_buffer_desc));
	if (!buffers) {
		goto error;
	}

	virt = spdk_dma_zmalloc((size * num_entries), 4096, NULL);
	if (!virt) {
		goto error;
	}

	for (i = 0; i < num_entries; i++) {
		buffer = buffers + i;

		buffer->len  = size;
		buffer->virt = (uint8_t *)virt + (i * size);
		buffer->phys = spdk_vtophys(buffer->virt, NULL);
	}

	return buffers;
error:
	if (buffers) {
		free(buffers);
	}
	return NULL;
}

#define OCS_NVME_PREREG_SGL true

static void
ocs_nvme_free_sgls(ocs_t *ocs, struct fc_sgl_list *sgl_list, uint32_t xri_count)
{
	uint32_t i;

	if (!sgl_list)
		return;

	for (i = 0; i < xri_count; i++) {
		ocs_dma_free(ocs, &ocs->hal.nvmet_sgls[i]);
	}

	ocs_free(ocs, ocs->hal.nvmet_sgls, sizeof(ocs_dma_t)*xri_count);
	ocs->hal.nvmet_sgls = NULL;

	ocs_free(ocs, sgl_list, sizeof(struct fc_sgl_list)*xri_count);
}

static struct fc_sgl_list*
ocs_nvme_alloc_sgls(ocs_t *ocs, uint32_t xri_base, uint32_t xri_count, bool prereg)
{
	ocs_hal_t *hal = &ocs->hal;
	int rc = 0;
	uint16_t i;
	uint8_t cmd[SLI4_BMBX_SIZE];
	uint32_t sgls_per_request = 256;
	uint32_t posted_idx = 0;
	struct fc_sgl_list *nvme_sgl_list = NULL;
	ocs_dma_t **sgls = NULL;
	ocs_dma_t reqbuf = { 0 };
	uint32_t nremaining = 0, n = 0;
	uint32_t req_buf_length = sizeof(sli4_req_fcoe_post_sgl_pages_t) +
		sizeof(sli4_fcoe_post_sgl_page_desc_t)*sgls_per_request;

	/* allocate space to store reference to pre-regsitered sgls */
	hal->nvmet_sgls = ocs_malloc(hal->os, sizeof(ocs_dma_t)*xri_count, OCS_M_ZERO);
	if (!hal->nvmet_sgls)
		return NULL;

	/* Only needed since the post_sgl_pages apis, needs an array of pointers to ocs_dma_t **/
	sgls = ocs_malloc(hal->os, sizeof(*sgls)*xri_count, OCS_M_NOWAIT);
	if (!sgls) {
		rc = -1;
		ocs_log_err(hal->os, " alloc failure failed\n");
		goto err;
	}

	for (i = 0; i < xri_count; i++) {
		rc = ocs_dma_alloc(hal->os, &hal->nvmet_sgls[i],
				   BCM_MAX_IOVECS * sizeof(bcm_sge_t), 64, OCS_M_NOWAIT);
		if (rc) {
			ocs_log_err(hal->os, "ocs_dma_alloc nvmet_sgls failed\n");
			goto err;
		}
		sgls[i] = &hal->nvmet_sgls[i];
	}

	if (prereg) {
		/* The reqbuf contains space to issue registration of sgls for 256 xris at a time */
		rc = ocs_dma_alloc(hal->os, &reqbuf, req_buf_length, OCS_MIN_DMA_ALIGNMENT, OCS_M_NOWAIT);
		if (rc) {
			ocs_log_err(hal->os, "ocs_dma_alloc reqbuf failed\n");
			goto err;
		}

		for (nremaining = xri_count; nremaining; nremaining -= n) {

			n = MIN(sgls_per_request, nremaining);
			if (sli_cmd_fcoe_post_sgl_pages(&hal->sli, cmd, sizeof(cmd),
							xri_base + posted_idx, n, 
							sgls + posted_idx, &reqbuf)) {
				rc = ocs_hal_command(hal, cmd, OCS_CMD_POLL, NULL, NULL);
				if (rc) {
					ocs_log_err(hal->os, "SGL post failed\n");
					goto err;
				}
			}
			posted_idx += n;
		}
	}

	/* This will be stored in args and handed off into the spdk-nvmf driver code */
	nvme_sgl_list = ocs_malloc(NULL, sizeof(struct fc_sgl_list)*xri_count, OCS_M_ZERO);
	if (!nvme_sgl_list) {
		rc = -1;
		goto err;
	}

	for (i = 0; i < xri_count; i++) {
		nvme_sgl_list[i].virt = hal->nvmet_sgls[i].virt;
		nvme_sgl_list[i].phys = (uint64_t) hal->nvmet_sgls[i].phys;
	}

err:
	ocs_dma_free(ocs, &reqbuf);

	if (sgls) {
		ocs_free(ocs, sgls, sizeof(*sgls)*xri_count);
	}

	if (!rc) {
		return nvme_sgl_list;
	}

	if (hal->nvmet_sgls) {
		for (i = 0; i < xri_count; i++) {
			ocs_dma_free(ocs, &hal->nvmet_sgls[i]);
		}
		ocs_free(ocs, hal->nvmet_sgls, sizeof(ocs_dma_t)*xri_count);
		hal->nvmet_sgls = NULL;
	}

	return NULL;

}

void
ocs_hw_port_cleanup(ocs_t *ocs, struct spdk_nvmf_fc_hw_port_init_args *args)
{
	if (ocs && args) {
		uint32_t i;
		struct bcm_nvmf_hw_queues* hwq;
		struct bcm_nvmf_fc_port *fc_hw_port = (struct bcm_nvmf_fc_port *)args->port_ctx;

 		hwq = (struct bcm_nvmf_hw_queues *)(args->ls_queue);
		ocs_free_nvme_buffers(hwq->rq_hdr.q.name, hwq->rq_hdr.buffer);
		ocs_free_nvme_buffers(hwq->rq_payload.q.name, hwq->rq_payload.buffer);

		for (i = 0; i < ocs->ocs_os.num_cores; i ++) {
			hwq = (struct bcm_nvmf_hw_queues *)(args->io_queues[i]); 
			ocs_free_nvme_buffers(hwq->rq_hdr.q.name, hwq->rq_hdr.buffer);
			ocs_free_nvme_buffers(hwq->rq_payload.q.name, hwq->rq_payload.buffer);
		}

		ocs_nvme_free_sgls(ocs, fc_hw_port->sgl_list, fc_hw_port->xri_count);
		
		free(args);
		ocs->tgt_ocs.args = NULL;
	}
}

static int
ocs_nvme_get_queue_indexes(ocs_t *ocs, uint32_t filter, uint32_t *eq_idx,
		uint32_t *rq_idx, uint32_t *wq_idx)
{
	ocs_hal_t *hal = &ocs->hal;
	hal_rq_t *rq = NULL;
	hal_eq_t *eq = NULL;
	hal_wq_t *wq = NULL;
	uint32_t i, j;
	bool rq_found = false;

	/* First get the RQ */
	for (i = 0; i < hal->config.n_rq && !rq_found; i++) {
		rq = hal->hal_rq[i];
		if (!rq->nvmeq) {
			continue;
		}

		/* Check if this RQ has the filter we are looking for */
		for (j = 0; j < SLI4_CMD_REG_FCFI_MRQ_NUM_RQ_CFG; j ++) {
			if (rq->filter_mask & (1U << j)) {
				if (hal->config.filter_def[j] == filter && rq->nvmeq) {
					*rq_idx = i;
					rq_found = true;
					break;
				}
			}
		}
	}

	if (!rq_found) { 
		return -1;
	}

	/* Search for EQ */
	for (i = 0, eq = hal->hal_eq[i]; i < hal->config.n_eq; i++, eq = hal->hal_eq[i]) {
		if (eq == rq->cq->eq) {
			*eq_idx = i;
			break;
		}
	}
	if (i == hal->config.n_eq) {
		return -1;
	}

	/* Search for WQ */
	for (i = 0, wq = hal->hal_wq[i]; i < hal->config.n_wq; i++, wq = hal->hal_wq[i]) {
		if (wq->cq->eq == eq) {
			*wq_idx = i;
			break;
		}
	}
	if (i == hal->config.n_wq) {
		return -1;
	}

	return 0;
}

static void
ocs_cb_hw_port_create(uint8_t port_handle, enum spdk_fc_event event_type,
		void *ctx, int err)
{
	struct spdk_nvmf_fc_hw_port_init_args *args = ctx;

	ocs_t *ocs = NULL;

	if (err) {
		ocs_log_err(NULL, "ocs%d port create failed.\n", args->port_handle);

		ocs_hw_port_cleanup(ocs, ocs->tgt_ocs.args);
	} else {
		ocs_log_info(NULL, "ocs%d port create success.\n", args->port_handle);

		ocs = ocs_get_instance(args->port_handle);
		if (ocs) {
			ocs->tgt_ocs.args = args; /* Save this for cleanup. */
		}
	}
}

/* 
 * RQ0 = SCSI + ELS
 * RQ1 = NVME LS
 * RQ2 - RQN = NVME IOs.
 */
int
ocs_nvme_hw_port_create(ocs_t *ocs)
{
	uint32_t i;
	ocs_hal_t *hal = &ocs->hal;
	struct spdk_nvmf_fc_hw_port_init_args *args = NULL;
	int rc;
	struct bcm_nvmf_hw_queues *hwq;
	spdk_nvmf_fc_lld_hwqp_t io_queues_start;
	struct bcm_nvmf_fc_port *fc_hw_port;
	uint32_t eq_idx, wq_idx, rq_idx;

	if (!hal->sli.config.sgl_pre_registered) {
		ocs_log_err(ocs, "sgl pre-registration disabled. hw_port_create failed\n");
		return -1;
	}

	ocs->tgt_ocs.args = NULL;

	args = ocs_malloc(NULL, sizeof(struct spdk_nvmf_fc_hw_port_init_args) + sizeof(struct bcm_nvmf_fc_port) +
			  ((sizeof(struct bcm_nvmf_hw_queues) + sizeof(spdk_nvmf_fc_lld_hwqp_t)) *
			  (ocs->ocs_os.num_cores + 1)), OCS_M_ZERO);
	if (!args) {
		goto error;
	}

	args->port_handle = ocs->instance_index;
	args->cb_ctx = args;
	args->fcp_rq_id = hal->hal_rq[0]->hdr->id; 
	args->nvme_aq_index = OCS_NVME_FC_AQ_IND;

	/* assign LLD fc port */
	fc_hw_port = (struct bcm_nvmf_fc_port *)((char *)args + sizeof(struct spdk_nvmf_fc_hw_port_init_args));
	fc_hw_port->xri_base = *(hal->sli.config.extent[SLI_RSRC_FCOE_XRI].base) +
				hal->sli.config.extent[SLI_RSRC_FCOE_XRI].size;
	fc_hw_port->xri_count = hal->sli.config.extent[SLI_RSRC_FCOE_XRI].nvme_size;
	fc_hw_port->sgl_preregistered = OCS_NVME_PREREG_SGL;

	ocs_log_info(ocs, "SLI4 NVME XRI base = %d cnt = %d\n",
		     fc_hw_port->xri_base, fc_hw_port->xri_count);	

	fc_hw_port->sgl_list = ocs_nvme_alloc_sgls(ocs, fc_hw_port->xri_base, fc_hw_port->xri_count, OCS_NVME_PREREG_SGL);
	if (!fc_hw_port->sgl_list) {
		ocs_log_err(ocs, "HW port create failed to alloc SGL list\n");
		goto error;
	}
	args->port_ctx = fc_hw_port;


	/* assign LS Q */
	args->ls_queue = (spdk_nvmf_fc_lld_hwqp_t)args + sizeof(struct spdk_nvmf_fc_hw_port_init_args) +
			  sizeof(struct bcm_nvmf_fc_port);
	hwq = (struct bcm_nvmf_hw_queues *)(args->ls_queue);

	/* Get NVME LS queue indexes */
	if (ocs_nvme_get_queue_indexes(ocs, OCS_NVME_LS_FILTER, &eq_idx, &rq_idx, &wq_idx)) {
		ocs_log_err(ocs, "Failed to find NVME LS queue indexes.\n");
		goto error;
	}

	ocs_fill_nvme_sli_queue(ocs, hal->hal_eq[eq_idx]->queue,
			&hwq->eq.q);
	ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[wq_idx]->cq->queue,
			&hwq->cq_wq.q);
	ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[rq_idx]->cq->queue,
			&hwq->cq_rq.q);

	/* LS WQ */
	ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[wq_idx]->queue,
			&hwq->wq.q);

	/* LS RQ Hdr */
	ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[rq_idx]->hdr,
				&hwq->rq_hdr.q);
	hwq->rq_hdr.buffer = ocs_alloc_nvme_buffers(hwq->rq_hdr.q.name,
					OCS_HAL_RQ_SIZE_HDR,
					hwq->rq_hdr.q.max_entries);
	if (!hwq->rq_hdr.buffer) {
		goto error;
	}
	hwq->rq_hdr.num_buffers = hwq->rq_hdr.q.max_entries;

	/* LS RQ Payload */
	ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[rq_idx]->data,
			&hwq->rq_payload.q);
	hwq->rq_payload.buffer = ocs_alloc_nvme_buffers(hwq->rq_payload.q.name, 
					OCS_HAL_RQ_SIZE_PAYLOAD,
					hwq->rq_payload.q.max_entries);
	if (!hwq->rq_payload.buffer) {
		goto error;
	}
	hwq->rq_payload.num_buffers =
		hwq->rq_payload.q.max_entries;
	args->ls_queue_size = hwq->rq_payload.num_buffers;

	/* assign the io queues */
	args->io_queues = args->ls_queue + sizeof(struct bcm_nvmf_hw_queues);
	io_queues_start = (spdk_nvmf_fc_lld_hwqp_t) args->io_queues +
			  (ocs->ocs_os.num_cores * sizeof(void *));

	/* Get NVME first ioq queue indexes */
	if (ocs_nvme_get_queue_indexes(ocs, OCS_NVME_IO_FILTER, &eq_idx, &rq_idx, &wq_idx)) {
		ocs_log_err(ocs, "Failed to find NVME IOQ queue indexes.\n");
		goto error;
	}

	for (i = 0; i < ocs->ocs_os.num_cores; i++) {
		args->io_queues[i] = io_queues_start + (i * sizeof(struct bcm_nvmf_hw_queues));
		hwq = (struct bcm_nvmf_hw_queues *)(args->io_queues[i]);
	
		ocs_fill_nvme_sli_queue(ocs, hal->hal_eq[i + eq_idx]->queue,
				&hwq->eq.q);
		ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[i + wq_idx]->cq->queue,
				&hwq->cq_wq.q);
		ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[i + rq_idx]->cq->queue,
				&hwq->cq_rq.q);

		/* IO WQ */
		ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[i + wq_idx]->queue,
				&hwq->wq.q);

		/* Send Frame WQ */
		if (i == 0 && hal->hal_sfwq[OCS_HAL_WQ_SFQ_NVME]) {
			/* Fill WQ SFQ queue for AQ (io_queues[0]) */
			hal_wq_t *sfwq = hal->hal_sfwq[OCS_HAL_WQ_SFQ_NVME];
			ocs_fill_nvme_sli_queue(ocs, sfwq->cq->queue, &hwq->cq_sfwq.q);
			ocs_fill_nvme_sli_queue(ocs, sfwq->queue, &hwq->sfwq.q);
			hwq->sfwq_configured = true;
		}

		/* IO RQ Hdr */
		ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[i + rq_idx]->hdr,
				&hwq->rq_hdr.q);
		hwq->rq_hdr.buffer = ocs_alloc_nvme_buffers(hwq->rq_hdr.q.name,
						OCS_HAL_RQ_SIZE_HDR,
						hwq->rq_hdr.q.max_entries);
		if (!hwq->rq_hdr.buffer) {
			goto error;
		}
		hwq->rq_hdr.num_buffers =
			hwq->rq_hdr.q.max_entries;

		/* IO RQ Payload */
		ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[i + rq_idx]->data,
				&hwq->rq_payload.q);
		hwq->rq_payload.buffer = ocs_alloc_nvme_buffers(hwq->rq_payload.q.name,
						OCS_HAL_RQ_SIZE_PAYLOAD,
						hwq->rq_payload.q.max_entries);
		if (!hwq->rq_payload.buffer) {
			goto error;
		}
		hwq->rq_payload.num_buffers =
			hwq->rq_payload.q.max_entries;
		hwq->cid_cnt = 0;

		args->io_queue_cnt++;
	}

	/* set args io queue size */
	args->io_queue_size = hwq->rq_payload.num_buffers;

	rc = nvmf_fc_main_enqueue_event(SPDK_FC_HW_PORT_INIT, args,
			ocs_cb_hw_port_create);
	if (rc) {
		goto error;
	}

	return 0; /* Queued */

error:
	if (args) {
		ocs_hw_port_cleanup(ocs, args);
	}

	return -1;
}

static void
ocs_cb_hw_port_quiesce(uint8_t port_handle, enum spdk_fc_event event_type,
		void *ctx, int err)
{
	struct spdk_nvmf_fc_hw_port_reset_args *args = ctx;

	if (err) {
		ocs_log_err(NULL, "ocs%d hw port quiesce failed.\n", args->port_handle);
	} else {
		ocs_log_info(NULL, "ocs%d hw port quiesce success.\n", args->port_handle);
	}

	free(args);
}

int
ocs_nvme_hw_port_quiesce(ocs_t *ocs)
{
	struct spdk_nvmf_fc_hw_port_reset_args *args;


	args = ocs_malloc(NULL, sizeof(struct spdk_nvmf_fc_hw_port_reset_args), OCS_M_ZERO);
	if (!args) {
		goto error;
	}

	args->port_handle = ocs->instance_index;
	args->dump_queues = false;
	args->cb_ctx = args;

	if (nvmf_fc_main_enqueue_event(SPDK_FC_HW_PORT_RESET, args,
				ocs_cb_hw_port_quiesce)) {
		ocs_log_err(ocs, "NVME HW Port reset failed.\n");
		goto error;
	}

	return 0;

error:
	if (args)
		free(args);

	return -1;
}

int
ocs_nvme_hw_port_free(ocs_t *ocs)
{
	struct spdk_nvmf_fc_hw_port_free_args args;

	memset(&args, 0, sizeof(struct spdk_nvmf_fc_hw_port_free_args));

	args.port_handle = ocs->instance_index;

	if (ocs_nvme_api_call_sync(SPDK_FC_HW_PORT_FREE, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "HW Port free failed.\n");
	}

	ocs_hw_port_cleanup(ocs, ocs->tgt_ocs.args);
	return 0;
}

int
ocs_nvme_hw_port_reinit(ocs_t *ocs)
{
	return 0;
}

int
ocs_nvme_process_hw_port_online(ocs_t *ocs)
{
	struct spdk_nvmf_fc_hw_port_online_args args;

	memset(&args, 0, sizeof(struct spdk_nvmf_fc_hw_port_online_args));
	args.port_handle = ocs->instance_index;

	if (ocs_nvme_api_call_sync(SPDK_FC_HW_PORT_ONLINE, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "HW Port online failed.\n");
		return -1;
	}

	ocs_log_info(ocs, "HW Port online success.\n");
	return 0;
}

int
ocs_nvme_process_hw_port_offline(ocs_t *ocs)
{
	struct spdk_nvmf_fc_hw_port_offline_args args;

	memset(&args, 0, sizeof(struct spdk_nvmf_fc_hw_port_offline_args));
	args.port_handle = ocs->instance_index;

	if (ocs_nvme_api_call_sync(SPDK_FC_HW_PORT_OFFLINE, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "HW Port offline failed.\n");
		return -1;
	}

	ocs_log_info(ocs, "HW Port offline success.\n");
	return 0;
}

int
ocs_nvme_nport_create(ocs_sport_t *sport)
{
	ocs_t *ocs = sport->ocs;
	struct spdk_nvmf_fc_nport_create_args args;

	memset(&args, 0, sizeof(struct spdk_nvmf_fc_nport_create_args));

	args.port_handle        = ocs->instance_index;

	args.nport_handle = sport->instance_index;
	args.fc_nodename.u.wwn = sport->wwnn;
	args.fc_portname.u.wwn = sport->wwpn;

	args.subsys_id          = 1;
	args.d_id               = sport->fc_id;

	if (ocs_nvme_api_call_sync(SPDK_FC_NPORT_CREATE, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "nport create failed.\n");
		return -1;
	}

	ocs_log_info(ocs, "nport create success.\n");
	return 0;
}

int
ocs_nvme_nport_delete(ocs_sport_t *sport)
{
	ocs_t *ocs = sport->ocs;
	struct spdk_nvmf_fc_nport_delete_args args;

	memset(&args, 0, sizeof(struct spdk_nvmf_fc_nport_delete_args));

	args.port_handle        = ocs->instance_index;
	args.nport_handle       = sport->instance_index;
	args.subsys_id          = 1;

	if (ocs_nvme_api_call_sync(SPDK_FC_NPORT_DELETE, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "nport delete failed.\n");
		return -1;
	}

	ocs_log_info(ocs, "nport delete success.\n");
	return 0;
}

static void
ocs_cb_abts_cb(uint8_t port_handle, enum spdk_fc_event event_type,
	void *args, int err)
{
	free(args);
}

int
ocs_nvme_process_abts(ocs_node_t *node, uint16_t oxid, uint16_t rxid)
{
	struct spdk_nvmf_fc_abts_args *args;
	int rc;

	args = ocs_malloc(NULL, sizeof(struct spdk_nvmf_fc_abts_args), OCS_M_ZERO);
	if (!args) {
		goto err;
	}

	args->port_handle = node->ocs->instance_index;
	args->nport_handle = node->sport->instance_index;
	args->oxid = oxid;
	args->rxid = rxid;
	args->rpi = node->rnode.indicator;
	args->cb_ctx = args;

	rc = nvmf_fc_main_enqueue_event(SPDK_FC_ABTS_RECV, args,
			ocs_cb_abts_cb);
	if (rc) {
		goto err;
	}

	return 0; /* Queued */
err:
	if (args) {
		free(args);
	}

	return -1;
}

int
ocs_nvme_validate_initiator(ocs_node_t *node)
{
	return 1;
}

int
ocs_nvme_process_prlo(ocs_io_t *io, uint16_t ox_id)
{
	ocs_t *ocs = io->ocs;
	ocs_node_t *node;
	struct spdk_nvmf_fc_hw_i_t_delete_args args;

	if (!io || !(node = io->node)) {
		return -1;
	}

	memset(&args, 0, sizeof(struct spdk_nvmf_fc_hw_i_t_delete_args));

	args.port_handle  = node->ocs->instance_index;
	args.nport_handle = node->sport->instance_index;
	args.rpi = node->rnode.indicator;
	args.s_id = node->rnode.fc_id;

	if (ocs_nvme_api_call_sync(SPDK_FC_IT_DELETE, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "NVME IT delete failed.\n");
		goto err;
	}

	node_printf(node, "NVME IT delete success.\n");
	ocs_send_prlo_acc(io, ox_id, FC_TYPE_NVME, NULL, NULL);
	return 0;
err:
	ocs_send_ls_rjt(io, ox_id, FC_REASON_UNABLE_TO_PERFORM,
			FC_EXPL_NO_ADDITIONAL,	0, NULL, NULL);
	return -1;
}

static void ocs_spdk_node_post_event_cb(void *arg)
{
	struct ocs_spdk_worker_node_post_args *args = (struct ocs_spdk_worker_node_post_args *)arg;

	if (!args->node) {
		ocs_log_err(NULL, "Node post event failed! Invalid Node pointer\n");
	} else {
		ocs_node_post_event(args->node, args->event, args->cb_data);
	}

	ocs_free(args->node->ocs, args, sizeof(*args));
}

struct ocs_nvme_node_lost_ctx {
	struct spdk_nvmf_fc_hw_i_t_delete_args args;
	ocs_node_t *node;
	void *cb_data;
};

static void
ocs_nvme_node_lost_cb(uint8_t port_handle, enum spdk_fc_event event_type,
		void *in, int err)
{
	struct ocs_nvme_node_lost_ctx *ctx = in;
	ocs_node_t *node = (ocs_node_t *)ctx->node;
	struct ocs_spdk_worker_node_post_args *node_post_args;

	if (err) {
		node_printf(ctx->node, "NVME IT delete failed.\n");
		goto exit;
	}

	node_printf(node, "NVME IT delete success.\n");

	node_post_args = ocs_malloc(node->ocs, sizeof(*node_post_args), OCS_M_ZERO);
	node_post_args->node = node;
	node_post_args->event = OCS_EVT_NODE_DEL_INI_COMPLETE;
	node_post_args->cb_data = ctx->cb_data;

	if (ocs_send_msg_to_worker(node->ocs, OCS_SPDK_WORKER_NODE_POST_EVENT, false,
				   ocs_spdk_node_post_event_cb, node_post_args)) {
		node_printf(node, "SPDK node post event failed\n");
		ocs_free(node->ocs, node_post_args, sizeof(*node_post_args));
	}
exit:
	free(ctx);	
}

int
ocs_nvme_node_lost(ocs_node_t *node, void *cbdata)
{
	struct ocs_nvme_node_lost_ctx *ctx;
	struct spdk_nvmf_fc_hw_i_t_delete_args *args;

	ctx = ocs_malloc(NULL, sizeof(struct ocs_nvme_node_lost_ctx), OCS_M_ZERO);
	if (!ctx) {
		goto err;
	}
	ctx->node = node;
	ctx->cb_data = cbdata;

	args = &ctx->args;
	args->port_handle	= node->ocs->instance_index;
	args->nport_handle	= node->sport->instance_index;
	args->rpi		= node->rnode.indicator;
	args->s_id		= node->rnode.fc_id;
	args->cb_ctx		= ctx;

	node_printf(node, "NVME IT delete initiated.\n");

	if (nvmf_fc_main_enqueue_event(SPDK_FC_IT_DELETE, args, ocs_nvme_node_lost_cb)) {
		node_printf(node, "NVME IT delete failed.\n");
		goto err;
	}

	return OCS_NVME_CALL_ASYNC;
err:
	node_printf(node, "API error. NVME IT delete failed.\n");
	free(ctx);	
	return OCS_NVME_CALL_COMPLETE;
}

int
ocs_nvme_new_initiator(ocs_node_t *node, void *cbdata)
{
	ocs_t *ocs = node->ocs;
	struct spdk_nvmf_fc_hw_i_t_add_args args;

	memset(&args, 0, sizeof(struct spdk_nvmf_fc_hw_i_t_add_args));

	args.port_handle	= node->ocs->instance_index;
	args.nport_handle	= node->sport->instance_index;
	args.rpi		= node->rnode.indicator;
	args.s_id		= node->rnode.fc_id;
	args.fc_nodename.u.wwn	= ocs_node_get_wwnn(node);
	args.fc_portname.u.wwn	= ocs_node_get_wwpn(node);

	args.initiator_prli_info = node->nvme_prli_service_params;
	args.target_prli_info	= node->nvme_prli_service_params;

	if (ocs_nvme_api_call_sync(SPDK_FC_IT_ADD, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "NVME IT add failed.\n");
		return -1;
	}

	node_printf(node, "NVME IT add success.\n");

	return 0;
}

int
ocs_nvme_del_initiator(ocs_node_t *node, void *cbdata)
{
	return ocs_nvme_node_lost(node, cbdata);
}

int
ocs_nvme_tgt_new_domain(ocs_domain_t *domain)
{
	if (ocs_tgt_nvme_enabled(domain->ocs) &&
		ocs_nvme_process_hw_port_online(domain->ocs)) {
		ocs_log_err(domain->ocs, "Failed to bring up nvme port online\n");

		return -1;
	}

	return 0;
}

int
ocs_nvme_tgt_del_domain(ocs_domain_t *domain)
{
	ocs_t *ocs = domain->ocs;

	if (ocs_tgt_nvme_enabled(ocs) &&
	    ocs_nvme_process_hw_port_offline(ocs)) {
		ocs_log_err(ocs, "Failed to bring down nvme port offline\n");
		return -1;
	}

	return 0;
}

int
ocs_nvme_tgt_new_sport(ocs_sport_t *sport)
{
	ocs_t *ocs = sport->ocs;
	int32_t rc = 0;

	if (ocs_tgt_nvme_enabled(ocs) &&
	    ocs_nvme_nport_create(sport)) {
		ocs_log_err(ocs, "Failed to create nport\n");
		rc = -1;
	}

	return rc;
}

int
ocs_nvme_tgt_del_sport(ocs_sport_t *sport)
{
	ocs_t *ocs = sport->ocs;

	if (ocs_tgt_nvme_enabled(sport->ocs) &&
			ocs_nvme_nport_delete(sport)) {
		ocs_log_err(ocs, "Failed to delete nvme nport%d \n",
				sport->instance_index);
		return -1;
	}

	return 0;
}

int
ocs_nvme_tgt_new_device(ocs_t *ocs)
{
	int32_t rc = 0;

	/* If nvme capability is enabled, notify backend. */
	if (ocs_tgt_nvme_enabled(ocs)) {
		rc = ocs_nvme_hw_port_create(ocs);
	}

	return rc;
}

int
ocs_nvme_tgt_del_device(ocs_t *ocs)
{
	return 0;
}

int
ocs_nvme_tgt_driver_init(void)
{
	return 0;
}

int
ocs_nvme_tgt_driver_exit(void)
{
	return 0;
}

void
ocs_tgt_common_new_device(ocs_t *ocs)
{
}

void
ocs_tgt_common_del_device(ocs_t *ocs)
{
}
