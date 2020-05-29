/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Inc. and/or its subsidiaries.
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

#include <pthread.h>
#include "ocs.h"
#include "ocs_os.h"
#include "ocs_els.h"
#include "ocs_device.h"
#include "ocs_spdk_nvmet.h"
#include "ocs_impl.h"
#include "spdk_nvmf_xport.h"

/* this will be set by spdk_fc_subsystem_init() */
uint32_t ocs_spdk_master_core = 0;

#define IS_APP_SHUTTINGDOWN(rc) do {					\
	if (spdk_env_get_current_core() == ocs_spdk_master_core) {	\
		return rc;						\
	}								\
} while (0)

static void
ocs_fill_nvme_sli_queue(ocs_t *ocs,
		sli4_queue_t *sli4_q,
		bcm_sli_queue_t *sli_q)

{
	sli_q->qid = sli4_q->id;
	sli_q->size = sli4_q->size;
	sli_q->address	= sli4_q->dma.virt;
	sli_q->max_entries = sli4_q->length;
	sli_q->doorbell_reg =
		ocs->ocs_os.bars[sli4_q->doorbell_rset].vaddr +
		sli4_q->doorbell_offset;

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
		ocs_spdk_free(memzone_name);
	}

	free(buffers);
}

static struct spdk_nvmf_fc_buffer_desc *
ocs_alloc_nvme_buffers(char *name, int size, int num_entries)
{
	int i;
	void *virt;
	uint64_t phys;
	struct spdk_nvmf_fc_buffer_desc *buffers = NULL, *buffer;

	buffers = calloc(num_entries, sizeof(struct spdk_nvmf_fc_buffer_desc));
	if (!buffers) {
		goto error;
	}

	virt = spdk_dma_zmalloc((size * num_entries), 4096, &phys);

	if (!virt) {
		goto error;
	}

	for (i = 0; i < num_entries; i++) {
		buffer = buffers + i;

		buffer->len  = size;
		buffer->virt = (uint8_t *)virt + (i * size);
		buffer->phys = phys + (i * size);
	}

	return buffers;
error:
	if (buffers) {
		free(buffers);
	}
	return NULL;
}

void
ocs_hw_port_cleanup(ocs_t *ocs)
{
	if (ocs && ocs->tgt_ocs.args) {
		uint32_t i;
		struct bcm_nvmf_hw_queues* hwq;

 		hwq = (struct bcm_nvmf_hw_queues *)(ocs->tgt_ocs.args->ls_queue);
		ocs_free_nvme_buffers(hwq->rq_hdr.q.name, hwq->rq_hdr.buffer);
		ocs_free_nvme_buffers(hwq->rq_payload.q.name, hwq->rq_payload.buffer);

		for (i = 0; i < ocs->num_cores; i ++) {
			hwq = (struct bcm_nvmf_hw_queues *)(ocs->tgt_ocs.args->io_queues[i]); 
			ocs_free_nvme_buffers(hwq->rq_hdr.q.name, hwq->rq_hdr.buffer);
			ocs_free_nvme_buffers(hwq->rq_payload.q.name, hwq->rq_payload.buffer);
		}

		free(ocs->tgt_ocs.args);
		ocs->tgt_ocs.args = NULL;
	}
}

static void
ocs_cb_hw_port_create(uint8_t port_handle, enum spdk_fc_event event_type,
		void *ctx, int err)
{
	struct spdk_nvmf_fc_hw_port_init_args *args = ctx;
	ocs_t *ocs = NULL;

	if (err) {
		ocs_log_err(NULL, "ocs%d port create failed.\n", args->port_handle);
		free(args);
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
	struct spdk_nvmf_fc_hw_port_init_args *args;
	int rc;
	struct bcm_nvmf_hw_queues* hwq;
	spdk_nvmf_fc_lld_hwqp_t io_queues_start;
	struct fc_xri_list *xri_list;
	uint32_t xri_base = *(ocs->hal.sli.config.extent[SLI_RSRC_FCOE_XRI].base) +
			    ocs->hal.sli.config.extent[SLI_RSRC_FCOE_XRI].size;

	xri_list = spdk_nvmf_fc_create_xri_list(xri_base + ocs->num_cores,
						ocs->hal.sli.config.extent[SLI_RSRC_FCOE_XRI].size - 
						ocs->num_cores);
	
	if (!xri_list) {
		ocs_log_err(ocs, "HW port create failed to create nvmf xri list.\n");
		return -1;
	}

	ocs->tgt_ocs.args = NULL;

	args = ocs_malloc(NULL, sizeof(struct spdk_nvmf_fc_hw_port_init_args) +
			  ((sizeof(struct bcm_nvmf_hw_queues) + sizeof(spdk_nvmf_fc_lld_hwqp_t)) *
			  (ocs->num_cores + 1)), OCS_M_ZERO);
	if (!args) {
		goto error;
	}

	args->port_handle = ocs->instance_index;
	args->cb_ctx = args;
	args->fcp_rq_id = hal->hal_rq[0]->hdr->id; 
	args->nvme_aq_index = OCS_NVME_FC_AQ_IND;

	/* assign LS Q */
	args->ls_queue = (spdk_nvmf_fc_lld_hwqp_t)args + sizeof(struct spdk_nvmf_fc_hw_port_init_args);
	hwq = (struct bcm_nvmf_hw_queues *)(args->ls_queue);

	/* assign XRI list to queue (shared by all queues on port */
	hwq->xri_list = xri_list;
	TAILQ_INIT(&hwq->pending_xri_list);

	ocs_fill_nvme_sli_queue(ocs, hal->hal_eq[1]->queue,
			&hwq->eq.q);
	ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[1]->cq->queue,
			&hwq->cq_wq.q);
	ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[1]->cq->queue,
			&hwq->cq_rq.q);

	/* LS WQ */
	ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[1]->queue,
			&hwq->wq.q);

	/* LS RQ Hdr */
	ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[1]->hdr,
				&hwq->rq_hdr.q);
	hwq->rq_hdr.buffer = ocs_alloc_nvme_buffers(hwq->rq_hdr.q.name,
			OCS_HAL_RQ_SIZE_HDR,
			hwq->rq_hdr.q.max_entries);
	if (!hwq->rq_hdr.buffer) {
		goto error;
	}
	hwq->rq_hdr.num_buffers = hwq->rq_hdr.q.max_entries;

	/* LS RQ Payload */
	ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[1]->data,
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
			  (ocs->num_cores * sizeof(void *));
	for (i = 0; i < ocs->num_cores; i++) {
		args->io_queues[i] = io_queues_start + (i * sizeof(struct bcm_nvmf_hw_queues));
		hwq = (struct bcm_nvmf_hw_queues *)(args->io_queues[i]);
	
		/* assign XRI list to queue (shared by all queues on port */
		hwq->xri_list = xri_list;
		TAILQ_INIT(&hwq->pending_xri_list);
	
		/* assign reserved XRI for send frames */
		hwq->send_frame_xri = xri_base++;
		hwq->send_frame_seqid = 0;

		ocs_fill_nvme_sli_queue(ocs, hal->hal_eq[i + 2]->queue,
				&hwq->eq.q);
		ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[i + 2]->cq->queue,
				&hwq->cq_wq.q);
		ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[i + 2]->cq->queue,
				&hwq->cq_rq.q);

		/* IO WQ */
		ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[i + 2]->queue,
				&hwq->wq.q);

		/* IO RQ Hdr */
		ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[i + 2]->hdr,
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
		ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[i + 2]->data,
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

	rc = nvmf_fc_master_enqueue_event(SPDK_FC_HW_PORT_INIT, args,
			ocs_cb_hw_port_create);
	if (rc) {
		goto error;
	}

	return 0; /* Queued */
error:
	if (args) {
 		hwq = (struct bcm_nvmf_hw_queues *)(args->ls_queue);
		if (hwq->rq_hdr.buffer) {
			ocs_free_nvme_buffers(hwq->rq_hdr.q.name, hwq->rq_hdr.buffer);
		}
		if (hwq->rq_payload.buffer) {
			ocs_free_nvme_buffers(hwq->rq_payload.q.name, hwq->rq_payload.buffer);
		}

		for (i = 0; i < args->io_queue_cnt; i++) {
			hwq = (struct bcm_nvmf_hw_queues *)(args->io_queues[i]);
			if (hwq->rq_hdr.buffer) {
				ocs_free_nvme_buffers(hwq->rq_hdr.q.name, hwq->rq_hdr.buffer);
			}
			if (hwq->rq_payload.buffer) {
				ocs_free_nvme_buffers(hwq->rq_payload.q.name, hwq->rq_payload.buffer);
			}
		}
		free(args);
	}
	return -1;
}


static void
ocs_nvme_process_hw_port_online_cb(uint8_t port_handle, enum spdk_fc_event event_type,
		void *in, int err)
{
	struct spdk_nvmf_fc_hw_port_online_args *args = in;
	ocs_t *ocs = ocs_get_instance(args->port_handle);

	if (err) {
		ocs_log_err(ocs, "HW Port online failed.\n");
	} else {
		ocs_log_info(ocs, "HW Port online success.\n");
	}
	free(args);
}

int
ocs_nvme_process_hw_port_online(ocs_sport_t *sport)
{
	ocs_t *ocs = sport->ocs;
	struct spdk_nvmf_fc_hw_port_online_args *args;
	
	IS_APP_SHUTTINGDOWN(0);
	args = ocs_malloc(NULL, sizeof(struct spdk_nvmf_fc_hw_port_online_args), OCS_M_ZERO);
	if (!args) {
		goto err;
	}

	args->port_handle = ocs->instance_index;
	args->cb_ctx = args;

	if (nvmf_fc_master_enqueue_event(SPDK_FC_HW_PORT_ONLINE, args,
				ocs_nvme_process_hw_port_online_cb)) {
		ocs_log_err(ocs, "HW Port online failed.\n");
		goto err;
	}

	return 0;
err:
	free(args);
	return -1;
}

static void
ocs_nvme_process_hw_port_offline_cb(uint8_t port_handle, enum spdk_fc_event event_type,
		void *in, int err)
{
	struct spdk_nvmf_fc_hw_port_offline_args *args = in;
	ocs_t *ocs = ocs_get_instance(args->port_handle);

	if (err) {
		ocs_log_err(ocs, "HW Port offline failed.\n");
	} else {
		ocs_log_info(ocs, "HW Port offline success.\n");
	}
	free(args);
}

int
ocs_nvme_process_hw_port_offline(ocs_t *ocs)
{
	struct spdk_nvmf_fc_hw_port_offline_args *args;
	
	IS_APP_SHUTTINGDOWN(0);
	args = ocs_malloc(NULL, sizeof(struct spdk_nvmf_fc_hw_port_offline_args), OCS_M_ZERO);
	if (!args) {
		goto err;
	}

	args->port_handle = ocs->instance_index;
	args->cb_ctx = args;

	if (nvmf_fc_master_enqueue_event(SPDK_FC_HW_PORT_OFFLINE, args,
				ocs_nvme_process_hw_port_offline_cb)) {
		ocs_log_err(ocs, "HW Port offline failed.\n");
		goto err;
	}

	return 0;
err:
	free(args);
	return -1;
}

static void
ocs_nvme_nport_create_cb(uint8_t port_handle, enum spdk_fc_event event_type,
		void *in, int err)
{
	struct spdk_nvmf_fc_nport_create_args *args = in;
	ocs_t *ocs = ocs_get_instance(args->port_handle);

	if (err) {
		ocs_log_err(ocs, "nport create failed.\n");
	} else {
		ocs_log_info(ocs, "nport create success.\n");
	}
	free(args);
}

int
ocs_nvme_nport_create(ocs_sport_t *sport)
{
	ocs_t *ocs = sport->ocs;
	struct spdk_nvmf_fc_nport_create_args *args;

	IS_APP_SHUTTINGDOWN(0);
	args = ocs_malloc(NULL, sizeof(struct spdk_nvmf_fc_nport_create_args), OCS_M_ZERO);
	if (!args) {
		goto err;
	}

	/* We register physical port as nport. Real nports not supported yet. */
	args->nport_handle	= 0;
	args->port_handle	= ocs->instance_index;
	args->fc_nodename.u.wwn = ocs_get_wwn(&ocs->hal, OCS_HAL_WWN_NODE);
	args->fc_portname.u.wwn = ocs_get_wwn(&ocs->hal, OCS_HAL_WWN_PORT);
	args->subsys_id		= 1;
	args->d_id		= sport->fc_id;
	args->cb_ctx 		= args;

	if (nvmf_fc_master_enqueue_event(SPDK_FC_NPORT_CREATE, args,
				ocs_nvme_nport_create_cb)) {
		ocs_log_err(ocs, "nport create failed.\n");
		goto err;
	}
	return 0;
err:
	free(args);
	return -1;
}

static void
ocs_nvme_nport_delete_cb(uint8_t port_handle, enum spdk_fc_event event_type,
		void *in, int err)
{
	struct spdk_nvmf_fc_nport_delete_args *args = in;
	ocs_t *ocs = ocs_get_instance(args->port_handle);

	if (err) {
		ocs_log_err(ocs, "nport delete failed.\n");
	} else {
		ocs_log_info(ocs, "nport delete success.\n");
	}
	free(args);
}

int
ocs_nvme_nport_delete(ocs_t *ocs)
{
	struct spdk_nvmf_fc_nport_delete_args *args;
	
	IS_APP_SHUTTINGDOWN(0);
	args = ocs_malloc(NULL, sizeof(struct spdk_nvmf_fc_nport_delete_args), OCS_M_ZERO);
	if (!args) {
		goto err;
	}

	args->port_handle	= ocs->instance_index;
	args->nport_handle	= 0;
	args->subsys_id		= 1;
	args->cb_ctx 		= args;

	if (nvmf_fc_master_enqueue_event(SPDK_FC_NPORT_DELETE, args,
				ocs_nvme_nport_delete_cb)) {
		ocs_log_err(ocs, "nport delete failed.\n");
		goto err;
	}
	return 0;
err:
	free(args);
	return -1;
}

static void
ocs_cb_abts_cb(uint8_t port_handle, enum spdk_fc_event event_type,
	void *args, int err)
{
	free(args);
}

int
ocs_nvme_process_abts(ocs_t *ocs, uint16_t oxid, uint16_t rxid, uint32_t rpi)
{
	struct spdk_nvmf_fc_abts_args *args;
	int rc;

	IS_APP_SHUTTINGDOWN(0);
	args = ocs_malloc(NULL, sizeof(struct spdk_nvmf_fc_abts_args), OCS_M_ZERO);
	if (!args) {
		goto err;
	}

	args->port_handle = ocs->instance_index;
	args->nport_handle = 0;
	args->oxid = oxid;
	args->rxid = rxid;
	args->rpi = rpi;
	args->cb_ctx = args;

	rc = nvmf_fc_master_enqueue_event(SPDK_FC_ABTS_RECV, args,
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

struct ocs_nvme_process_prli_ctx {
	struct spdk_nvmf_fc_hw_i_t_add_args args;
	ocs_io_t *io;
	uint16_t ox_id;
};

static void
ocs_nvme_prli_resp_cb(ocs_node_t *node, ocs_node_cb_t *cbdata, void *cbarg)
{
	if (cbdata->status != SLI4_FC_WCQE_STATUS_SUCCESS) {
		ocs_log_info(node->ocs, "Failed to send PRI resp. \n");
		ocs_nvme_node_lost(node);
	}
}

static void
ocs_nvme_process_prli_cb(uint8_t port_handle, enum spdk_fc_event event_type,
		void *in, int err)
{
	struct ocs_nvme_process_prli_ctx *ctx = in;
	struct spdk_nvmf_fc_hw_i_t_add_args *args = &ctx->args;
	ocs_t *ocs = ocs_get_instance(args->port_handle);

	if (err && (err != -EEXIST)) {
		ocs_log_err(ocs, "NVME IT add failed.\n");
		ocs_send_ls_rjt(ctx->io, ctx->ox_id, FC_REASON_UNABLE_TO_PERFORM,
				FC_EXPL_NO_ADDITIONAL,	0, NULL, NULL);
	} else {
		ocs_log_info(ocs, "NVME IT add success.\n");
		ocs_send_prli_acc(ctx->io, ctx->ox_id, FC_TYPE_NVME, ocs_nvme_prli_resp_cb, NULL);
	}
	free(ctx);
}

int
ocs_nvme_process_prli(ocs_io_t *io, uint16_t ox_id)
{
	ocs_t *ocs = io->ocs;
	ocs_node_t *node;
	struct ocs_nvme_process_prli_ctx *ctx;
	struct spdk_nvmf_fc_hw_i_t_add_args *args;

	IS_APP_SHUTTINGDOWN(0);
	if (!io || !(node = io->node)) {
		return -1;
	}

	ctx = ocs_malloc(NULL, sizeof(struct ocs_nvme_process_prli_ctx), OCS_M_ZERO);
	if (!ctx) {
		goto err;
	}
	ctx->io = io;
	ctx->ox_id = ox_id;

	args = &ctx->args;
	args->port_handle	= node->ocs->instance_index;
	args->nport_handle	= 0;
	args->rpi		= node->rnode.indicator;
	args->s_id		= node->rnode.fc_id;
	args->fc_nodename.u.wwn	= ocs_node_get_wwnn(node);
	args->fc_portname.u.wwn	= ocs_node_get_wwpn(node);
	args->target_prli_info	= node->nvme_prli_service_params;
	args->cb_ctx = ctx;

	if (nvmf_fc_master_enqueue_event(SPDK_FC_IT_ADD, args,
				ocs_nvme_process_prli_cb)) {
		ocs_log_err(ocs, "NVME IT add failed.\n");
		goto err;
	}

	return 0;
err:
	free(ctx);
	ocs_send_ls_rjt(io, ox_id, FC_REASON_UNABLE_TO_PERFORM,
			FC_EXPL_NO_ADDITIONAL,	0, NULL, NULL);
	return -1;
}

struct ocs_nvme_process_prlo_ctx {
	struct spdk_nvmf_fc_hw_i_t_delete_args args;
	ocs_io_t *io;
	uint16_t ox_id;
};

static void
ocs_nvme_process_prlo_cb(uint8_t port_handle, enum spdk_fc_event event_type,
		void *in, int err)
{
	struct ocs_nvme_process_prlo_ctx *ctx = in;
	struct spdk_nvmf_fc_hw_i_t_delete_args *args = &ctx->args;
	ocs_t *ocs = ocs_get_instance(args->port_handle);

	if (err) {
		ocs_log_err(ocs, "%s: NVME IT delete failed.\n", __func__);
		ocs_send_ls_rjt(ctx->io, ctx->ox_id, FC_REASON_UNABLE_TO_PERFORM,
				FC_EXPL_NO_ADDITIONAL,	0, NULL, NULL);
	} else {
		ocs_log_info(ocs, "%s: NVME IT delete success.\n", __func__);
		ocs_send_prlo_acc(ctx->io, ctx->ox_id, FC_TYPE_NVME, NULL, NULL);
	}
	free(ctx);
}

int
ocs_nvme_process_prlo(ocs_io_t *io, uint16_t ox_id)
{
	ocs_t *ocs = io->ocs;
	ocs_node_t *node;
	struct ocs_nvme_process_prlo_ctx *ctx;
	struct spdk_nvmf_fc_hw_i_t_delete_args *args;

	IS_APP_SHUTTINGDOWN(0);
	if (!io || !(node = io->node)) {
		return -1;
	}

	ctx = ocs_malloc(NULL, sizeof(struct ocs_nvme_process_prlo_ctx), OCS_M_ZERO);
	if (!ctx) {
		goto err;
	}
	ctx->io = io;
	ctx->ox_id = ox_id;

	args = &ctx->args;
	args->port_handle  = node->ocs->instance_index;
	args->nport_handle = 0;
	args->rpi = node->rnode.indicator;
	args->s_id = node->rnode.fc_id;
	args->cb_ctx = ctx;

	if (nvmf_fc_master_enqueue_event(SPDK_FC_IT_DELETE, args,
				ocs_nvme_process_prlo_cb)) {
		ocs_log_err(ocs, "NVME IT delete failed.\n");
		goto err;
	}

	return 0;
err:
	free(ctx);
	ocs_send_ls_rjt(io, ox_id, FC_REASON_UNABLE_TO_PERFORM,
			FC_EXPL_NO_ADDITIONAL,	0, NULL, NULL);
	return -1;
}

struct ocs_nvme_node_lost_ctx {
	struct spdk_nvmf_fc_hw_i_t_delete_args args;
	ocs_node_t *node;
};

static void
ocs_nvme_node_lost_cb(uint8_t port_handle, enum spdk_fc_event event_type,
		void *in, int err)
{
	struct ocs_nvme_node_lost_ctx *ctx = in;
	struct spdk_nvmf_fc_hw_i_t_delete_args *args = &ctx->args;
	ocs_t *ocs = ocs_get_instance(args->port_handle);

	if (err) {
		ocs_log_err(ocs, "%s: NVME IT delete failed.\n", __func__);
	} else {
		ocs_log_info(ocs, "%s: NVME IT delete success.\n", __func__);
	}

	ocs_node_post_event(ctx->node, OCS_EVT_NODE_DEL_INI_COMPLETE, NULL);
	free(ctx);	
}

int
ocs_nvme_node_lost(ocs_node_t *node)
{
	ocs_t *ocs = node->ocs;
	struct ocs_nvme_node_lost_ctx *ctx;
	struct spdk_nvmf_fc_hw_i_t_delete_args *args;

	IS_APP_SHUTTINGDOWN(-1);
	ctx = ocs_malloc(NULL, sizeof(struct ocs_nvme_node_lost_ctx), OCS_M_ZERO);
	if (!ctx) {
		goto err;
	}
	ctx->node = node;

	args = &ctx->args;
	args->port_handle	= node->ocs->instance_index;
	args->nport_handle	= 0;
	args->rpi		= node->rnode.indicator;
	args->s_id		= node->rnode.fc_id;
	args->cb_ctx		= ctx;

	if (nvmf_fc_master_enqueue_event(SPDK_FC_IT_DELETE, args, ocs_nvme_node_lost_cb)) {
		ocs_log_err(ocs, "NVME IT delete failed.\n");
		goto err;
	}

	return 0;
err:
	free(ctx);	
	return -1;
}

int32_t
ocs_nvme_tgt_new_domain(ocs_domain_t *domain)
{
	return 0;
}

void
ocs_nvme_tgt_del_domain(ocs_domain_t *domain)
{
	ocs_t *ocs = domain->ocs;

	if (!ocs->enable_nvme_tgt)
		return;

	if (ocs_nvme_nport_delete(ocs)) {
		ocs_log_err(ocs, "Failed to delete nvme port \n");
	}

	if (ocs_nvme_process_hw_port_offline(ocs)) {
		ocs_log_err(ocs, "Failed to bring down nvme port offline\n");
	}
}

int32_t
ocs_nvme_tgt_new_device(ocs_t *ocs)
{
	int32_t rc = 0;

	/* If nvme capability is enabled, notify backend. */
	if (ocs->enable_nvme_tgt) {
		rc = ocs_nvme_hw_port_create(ocs);
	}

	return rc;
}

int32_t
ocs_nvme_tgt_del_device(ocs_t *ocs)
{
	/* If nvme capability is enabled, notify backend. */
	if (ocs->enable_nvme_tgt) {
		ocs_hw_port_cleanup(ocs);
	}

	return 0;
}

int32_t
ocs_nvme_tgt_new_sport(ocs_sport_t *sport)
{
	ocs_t *ocs = sport->ocs;
	int32_t rc = -1;

	/* currently just the physical sport */
	if (sport->is_vport) {
		return rc;
	}

	if (!ocs->enable_nvme_tgt) {
		return 0;
	}

	if (ocs_nvme_process_hw_port_online(sport)) {
		ocs_log_err(ocs, "Failed to bring up nvme port online\n")
		goto done;
	}

	if (ocs_nvme_nport_create(sport)) {
		ocs_log_err(ocs, "Failed to create nport\n")
		goto done;
	}

	/* Success */
	rc = 0;

done:
	return rc;
}
