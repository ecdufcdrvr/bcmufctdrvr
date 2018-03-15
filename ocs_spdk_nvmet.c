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

#include <pthread.h>
#include "ocs.h"
#include "ocs_os.h"
#include "ocs_els.h"
#include "ocs_device.h"
#include "ocs_spdk_nvmet.h"
#include "ocs_tgt_api.h"
#include "nvmf_fc/fc_adm_api.h"

static struct spdk_nvmf_bcm_fc_master_ops *g_nvmf_fc_ops = NULL;

struct nvme_api_ctx {
	spdk_err_t rc;
	sem_t sem;
};

void
spdk_nvmf_fc_register_ops(struct spdk_nvmf_bcm_fc_master_ops *ops)
{
	g_nvmf_fc_ops = ops;
}

static void
ocs_nvme_api_call_done(uint8_t port_handle, spdk_fc_event_t event_type,
		void *in, spdk_err_t err)
{
	struct nvme_api_ctx *ctx = in;

	ctx->rc = err;
	sem_post(&ctx->sem);
}

static spdk_err_t
ocs_nvme_api_call_sync(spdk_fc_event_t event_type, void *args, void **cb_args)
{
	struct nvme_api_ctx ctx;

	if (!g_nvmf_fc_ops) {
		return SPDK_ERR_INTERNAL;
	}

	memset(&ctx, 0, sizeof(struct nvme_api_ctx));
	sem_init(&ctx.sem, 1, 0);

	if (spdk_env_get_current_core() == spdk_env_get_master_lcore()) {
		/* Might be app is stoping. Just return success */
		ctx.rc = SPDK_SUCCESS;
		goto done;	
	} else {
		/* Set the default status */
		ctx.rc = SPDK_ERR_INTERNAL;
	}

	*cb_args = &ctx;

	if (g_nvmf_fc_ops->enqueue_event(event_type, args, ocs_nvme_api_call_done)) {
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
	sli_q->address	= sli4_q->dma.virt;
	sli_q->max_entries = sli4_q->length;
	sli_q->doorbell_reg =
		ocs->ocs_os.bars[sli4_q->doorbell_rset].vaddr +
		sli4_q->doorbell_offset;
}

static void
ocs_free_nvme_buffers(bcm_buffer_desc_t *buffers)
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

	spdk_free(buffers);
}

static bcm_buffer_desc_t *
ocs_alloc_nvme_buffers(int size, int num_entries)
{
	int i;
	void *virt;
	uint64_t phys;
	bcm_buffer_desc_t *buffers = NULL, *buffer;

	buffers = spdk_calloc(num_entries, sizeof(bcm_buffer_desc_t));
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
		spdk_free(buffers);
	}
	return NULL;
}

void
ocs_hw_port_cleanup(ocs_t *ocs)
{
	if (ocs && ocs->tgt_ocs.args) {
		int i;
		spdk_nvmf_bcm_fc_hw_port_init_args_t *args = ocs->tgt_ocs.args;

		ocs_free_nvme_buffers(args->ls_queue.wq.buffer);
		ocs_free_nvme_buffers(args->ls_queue.rq_hdr.buffer);
		ocs_free_nvme_buffers(args->ls_queue.rq_payload.buffer);

		for (i = 0; i < NVMF_FC_MAX_IO_QUEUES; i ++) {
			ocs_free_nvme_buffers(args->io_queues[i].wq.buffer);
			ocs_free_nvme_buffers(args->io_queues[i].rq_hdr.buffer);
			ocs_free_nvme_buffers(args->io_queues[i].rq_payload.buffer);
		}

		free(args);
		ocs->tgt_ocs.args = NULL;
	}
}

static void
ocs_cb_hw_port_create(uint8_t port_handle, spdk_fc_event_t event_type,
		void *ctx, spdk_err_t err)
{
	spdk_nvmf_bcm_fc_hw_port_init_args_t *args = ctx;
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
 * This is code is based on netapp queue topology assumptions
 * RQ0 = SCSI + ELS
 * RQ1 = NVME LS
 * RQ2 - RQN = NVME IOs.
 */
int
ocs_nvme_hw_port_create(ocs_t *ocs)
{
	int i;
	ocs_hal_t *hal = &ocs->hal;
	spdk_nvmf_bcm_fc_hw_port_init_args_t *args;
	spdk_err_t rc;

	ocs->tgt_ocs.args = NULL;

	if (!g_nvmf_fc_ops) {
		return 0;
	}

	args = ocs_malloc(NULL, sizeof(spdk_nvmf_bcm_fc_hw_port_init_args_t), OCS_M_ZERO);
	if (!args) {
		goto error;
	}

	args->port_handle = ocs->instance_index;
	args->xri_base =
		*(ocs->hal.sli.config.extent[SLI_RSRC_FCOE_XRI].base) +
		ocs->hal.sli.config.extent[SLI_RSRC_FCOE_XRI].size;
	args->xri_count = ocs->hal.sli.config.extent[SLI_RSRC_FCOE_XRI].size;
	args->cb_ctx = args;
	args->fcp_rq_id = hal->hal_rq[0]->hdr->id; 

	/* Fill NVME LS event queues. */
	ocs_fill_nvme_sli_queue(ocs, hal->hal_eq[1]->queue,
			&args->ls_queue.eq.q);
	ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[1]->cq->queue,
			&args->ls_queue.cq_wq.q);
	ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[1]->cq->queue,
			&args->ls_queue.cq_rq.q);

	/* LS WQ */
	ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[1]->queue,
			&args->ls_queue.wq.q);

	/* LS RQ Hdr */
	ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[1]->hdr,
			&args->ls_queue.rq_hdr.q);
	args->ls_queue.rq_hdr.buffer =
		ocs_alloc_nvme_buffers(OCS_HAL_RQ_SIZE_HDR,
				args->ls_queue.rq_hdr.q.max_entries);
	if (!args->ls_queue.rq_hdr.buffer) {
		goto error;
	}
	args->ls_queue.rq_hdr.num_buffers = args->ls_queue.rq_hdr.q.max_entries;


	/* LS RQ Payload */
	ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[1]->data,
			&args->ls_queue.rq_payload.q);
	args->ls_queue.rq_payload.buffer =
		ocs_alloc_nvme_buffers(OCS_HAL_RQ_SIZE_PAYLOAD,
				args->ls_queue.rq_payload.q.max_entries);
	if (!args->ls_queue.rq_payload.buffer) {
		goto error;
	}
	args->ls_queue.rq_payload.num_buffers =
		args->ls_queue.rq_payload.q.max_entries;

	for (i = 0; i < NVMF_FC_MAX_IO_QUEUES; i ++) {
		ocs_fill_nvme_sli_queue(ocs, hal->hal_eq[i + 2]->queue,
				&args->io_queues[i].eq.q);
		ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[i + 2]->cq->queue,
				&args->io_queues[i].cq_wq.q);
		ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[i + 2]->cq->queue,
				&args->io_queues[i].cq_rq.q);

		/* IO WQ */
		ocs_fill_nvme_sli_queue(ocs, hal->hal_wq[i + 2]->queue,
				&args->io_queues[i].wq.q);

		/* IO RQ Hdr */
		ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[i + 2]->hdr,
				&args->io_queues[i].rq_hdr.q);
		args->io_queues[i].rq_hdr.buffer =
			ocs_alloc_nvme_buffers(OCS_HAL_RQ_SIZE_HDR,
					args->io_queues[i].rq_hdr.q.max_entries);
		if (!args->io_queues[i].rq_hdr.buffer) {
			goto error;
		}
		args->io_queues[i].rq_hdr.num_buffers =
			args->io_queues[i].rq_hdr.q.max_entries;


		/* IO RQ Payload */
		ocs_fill_nvme_sli_queue(ocs, hal->hal_rq[i + 2]->data,
				&args->io_queues[i].rq_payload.q);
		args->io_queues[i].rq_payload.buffer =
			ocs_alloc_nvme_buffers(OCS_HAL_RQ_SIZE_PAYLOAD,
					args->io_queues[i].rq_payload.q.max_entries);
		if (!args->io_queues[i].rq_payload.buffer) {
			goto error;
		}
		args->io_queues[i].rq_payload.num_buffers =
			args->io_queues[i].rq_payload.q.max_entries;

		args->io_queue_cnt ++;
	}

	rc = g_nvmf_fc_ops->enqueue_event(SPDK_FC_HW_PORT_INIT, args,
			ocs_cb_hw_port_create);
	if (rc) {
		goto error;
	}

	return 0; /* Queued */
error:
	if (args) {
		ocs_free_nvme_buffers(args->ls_queue.rq_hdr.buffer);
		ocs_free_nvme_buffers(args->ls_queue.rq_payload.buffer);

		for (i = 0; i < NVMF_FC_MAX_IO_QUEUES; i ++) {
			ocs_free_nvme_buffers(args->io_queues[i].rq_hdr.buffer);
			ocs_free_nvme_buffers(args->io_queues[i].rq_payload.buffer);
		}
		free(args);
	}
	return -1;

}

int
ocs_nvme_process_hw_port_online(ocs_sport_t *sport)
{
	ocs_t *ocs = sport->ocs;
	spdk_nvmf_bcm_fc_hw_port_online_args_t args;

	memset(&args, 0, sizeof(spdk_nvmf_bcm_fc_hw_port_online_args_t));
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
	spdk_nvmf_bcm_fc_hw_port_offline_args_t args;

	memset(&args, 0, sizeof(spdk_nvmf_bcm_fc_hw_port_offline_args_t));
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
	spdk_nvmf_bcm_fc_nport_create_args_t args;

	memset(&args, 0, sizeof(spdk_nvmf_bcm_fc_nport_create_args_t));

	/* We register physical port as nport. Real nports not supported yet. */
	args.nport_handle	= 0;
	args.port_handle	= ocs->instance_index;
	args.fc_nodename.u.wwn 	= ocs_get_wwn(&ocs->hal, OCS_HAL_WWN_NODE);
	args.fc_portname.u.wwn 	= ocs_get_wwn(&ocs->hal, OCS_HAL_WWN_PORT);
	args.subsys_id		= 1;
	args.d_id		= sport->fc_id;

	if (ocs_nvme_api_call_sync(SPDK_FC_NPORT_CREATE, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "nport create failed.\n");
		return -1;		
	}

	ocs_log_info(ocs, "nport create success.\n");
	return 0;
}

int
ocs_nvme_nport_delete(ocs_t *ocs)
{
	spdk_nvmf_bcm_fc_nport_delete_args_t args;

	memset(&args, 0, sizeof(spdk_nvmf_bcm_fc_nport_delete_args_t));

	args.port_handle	= ocs->instance_index;
	args.nport_handle	= 0;
	args.subsys_id		= 1;

	if (ocs_nvme_api_call_sync(SPDK_FC_NPORT_DELETE, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "nport delete failed.\n");
		return -1;		
	}

	ocs_log_info(ocs, "nport delete success.\n");
	return 0;
}

static void
ocs_cb_abts_cb(uint8_t port_handle, spdk_fc_event_t event_type,
	void *args, spdk_err_t err)
{
	free(args);
}

int
ocs_nvme_process_abts(ocs_t *ocs, uint16_t oxid, uint16_t rxid, uint32_t rpi)
{
	spdk_nvmf_bcm_fc_abts_args_t *args;
	spdk_err_t rc;

	if (!g_nvmf_fc_ops) {
		return -1;
	}

	args = ocs_malloc(NULL, sizeof(spdk_nvmf_bcm_fc_abts_args_t), OCS_M_ZERO);
	if (!args) {
		goto err;
	}

	args->port_handle = ocs->instance_index;
	args->nport_handle = 0;
	args->oxid = oxid;
	args->rxid = rxid;
	args->rpi = rpi;
	args->cb_ctx = args;

	rc = g_nvmf_fc_ops->enqueue_event(SPDK_FC_ABTS_RECV, args,
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

static void
ocs_nvme_prli_resp_cb(ocs_node_t *node, ocs_node_cb_t *cbdata, void *cbarg)
{
	if (cbdata->status != SLI4_FC_WCQE_STATUS_SUCCESS) {
		ocs_log_info(node->ocs, "Failed to send PRI resp. \n");
		ocs_nvme_node_lost(node);
	}
}

int
ocs_nvme_process_prli(ocs_io_t *io, uint16_t ox_id)
{
	ocs_t *ocs = io->ocs;
	ocs_node_t *node;
	spdk_nvmf_bcm_fc_hw_i_t_add_args_t args;

	if (!io || !(node = io->node)) {
		return -1;
	}

	memset(&args, 0, sizeof(spdk_nvmf_bcm_fc_hw_i_t_add_args_t));

	args.port_handle	= node->ocs->instance_index;
	args.nport_handle	= node->sport->instance_index;
	args.rpi		= node->rnode.indicator;
	args.s_id		= node->rnode.fc_id;
	args.fc_nodename.u.wwn	= ocs_node_get_wwnn(node);
	args.fc_portname.u.wwn	= ocs_node_get_wwpn(node);
	args.target_prli_info	= node->nvme_prli_service_params;

	if (ocs_nvme_api_call_sync(SPDK_FC_IT_ADD, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "NVME IT add failed.\n");
		goto err;
	}

	ocs_log_info(ocs, "NVME IT add success.\n");

	ocs_send_prli_acc(io, ox_id, FC_TYPE_NVME, ocs_nvme_prli_resp_cb, NULL);
	return 0;
err:
	ocs_send_ls_rjt(io, ox_id, FC_REASON_UNABLE_TO_PERFORM,
			FC_EXPL_NO_ADDITIONAL,	0, NULL, NULL);
	return -1;
}

int
ocs_nvme_process_prlo(ocs_io_t *io, uint16_t ox_id)
{
	ocs_t *ocs = io->ocs;
	ocs_node_t *node;
	spdk_nvmf_bcm_fc_hw_i_t_delete_args_t args;

	if (!io || !(node = io->node)) {
		return -1;
	}

	memset(&args, 0, sizeof(spdk_nvmf_bcm_fc_hw_i_t_delete_args_t));

	args.port_handle  = node->ocs->instance_index;
	args.nport_handle = node->sport->instance_index;
	args.rpi = node->rnode.indicator;
	args.s_id = node->rnode.fc_id;

	if (ocs_nvme_api_call_sync(SPDK_FC_IT_DELETE, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "NVME IT delete failed.\n");
		goto err;
	}

	ocs_log_info(ocs, "NVME IT delete success.\n");
	ocs_send_prlo_acc(io, ox_id, FC_TYPE_NVME, NULL, NULL);
	return 0;
err:
	ocs_send_ls_rjt(io, ox_id, FC_REASON_UNABLE_TO_PERFORM,
			FC_EXPL_NO_ADDITIONAL,	0, NULL, NULL);
	return -1;
}


int
ocs_nvme_node_lost(ocs_node_t *node)
{
	ocs_t *ocs = node->ocs;
	spdk_nvmf_bcm_fc_hw_i_t_delete_args_t args;

	memset(&args, 0, sizeof(spdk_nvmf_bcm_fc_hw_i_t_delete_args_t));

	args.port_handle	= node->ocs->instance_index;
	args.nport_handle	= node->sport->instance_index;
	args.rpi		= node->rnode.indicator;
	args.s_id		= node->rnode.fc_id;

	if (ocs_nvme_api_call_sync(SPDK_FC_IT_DELETE, &args, &args.cb_ctx)) {
		ocs_log_err(ocs, "NVME IT delete failed.\n");
		goto err;
	}

	ocs_log_info(ocs, "NVME IT delete success.\n");
	return 0;
err:
	return -1;
}
