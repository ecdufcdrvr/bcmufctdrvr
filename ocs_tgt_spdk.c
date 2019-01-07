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

/**
 * @file
 *
 */

#include "ocs.h"
#include "ocs_os.h"
#include "ocs_scsi.h"
#include "scsi_cmds.h"
#include "fc.h"
#include "task.h"
#include "ocs_tgt_spdk.h"
#include "ocs_spdk_conf.h"
#include "spdk/scsi_spec.h"
#include "spdk/scsi.h"
#include "ocs_spdk_nvmet.h"

extern struct spdk_fc_igs_list g_spdk_fc_igs;
extern struct spdk_fc_lun_map_list g_spdk_fc_lun_maps;
extern struct spdk_fc_hba_ports_list g_spdk_fc_ports;

static int32_t
ocs_write_phase_start(ocs_io_t *io);
static int32_t
ocs_read_phase_start(ocs_io_t *io);

int
parse_wwn(char *wwn_in, uint64_t *wwn_out)
{
	uint8_t byte0;
	uint8_t byte1;
	uint8_t byte2;
	uint8_t byte3;
	uint8_t byte4;
	uint8_t byte5;
	uint8_t byte6;
	uint8_t byte7;
	int rc;

	rc = ocs_sscanf(wwn_in, "%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",
			&byte0, &byte1, &byte2, &byte3,
			&byte4, &byte5, &byte6, &byte7);

	if (rc == 8) {
		*wwn_out = ((uint64_t)byte0 << 56) |
			((uint64_t)byte1 << 48) |
			((uint64_t)byte2 << 40) |
			((uint64_t)byte3 << 32) |
			((uint64_t)byte4 << 24) |
			((uint64_t)byte5 << 16) |
			((uint64_t)byte6 <<  8) |
			((uint64_t)byte7);
		return 0;
	} else {
		return -1;
	}
}



static void
spdk_fc_queue_mgmt_task(struct spdk_scsi_dev *dev, struct spdk_fc_task *task,
		enum spdk_scsi_task_func function)
{
	spdk_scsi_dev_queue_mgmt_task(dev, &task->scsi/*, function*/);
}

static void
spdk_fc_queue_task(struct spdk_scsi_dev *dev, struct spdk_fc_task *task)
{
	if (task->scsi.lun == NULL) {
		spdk_scsi_task_process_null_lun(&task->scsi);
		process_task_completion(&task->scsi);
	} else {
		spdk_scsi_dev_queue_task(dev, &task->scsi);
	}
}

static int32_t
scsi_io_free_cb(ocs_io_t *io, ocs_scsi_io_status_e scsi_status, uint32_t flags, void *arg)
{
	struct spdk_fc_task *primary = io->tgt_io.primary, *sub_task, *tmp;
	ocs_t *ocs = io->ocs;

	if (primary) {
		TAILQ_FOREACH_SAFE(sub_task, &primary->fc_subtask_list, fc_link, tmp) {
			if (sub_task->mobj) {
				rte_mempool_put(g_spdk_fc.buff_pool, (void *)sub_task->mobj);
				sub_task->mobj = NULL;
			}
			TAILQ_REMOVE(&primary->fc_subtask_list, sub_task, fc_link);
			spdk_fc_task_put(sub_task);
		}

		if (primary->mobj) {
			rte_mempool_put(g_spdk_fc.buff_pool, (void *)primary->mobj);
			primary->mobj = NULL;
		} 
		spdk_fc_task_put(primary);
	}

	if (ocs_io_busy(io)) {
		ocs_scsi_io_complete(io);
		ocs_atomic_sub_return(&ocs->tgt_ocs.ios_in_use, 1);
	}

	io->tgt_io.primary = NULL;
	return 0;	
}

static int32_t
ocs_scsi_check_cond(ocs_io_t *io, uint8_t sense_key, uint8_t asc, uint8_t ascq)
{
	ocs_scsi_cmd_resp_t rsp;
	fixed_sense_data_t sense;

	ocs_memset(&rsp, 0, sizeof(rsp));
	ocs_memset(&sense, 0, sizeof(sense));

	rsp.scsi_status = 2;
	rsp.sense_data = (uint8_t*) &sense;
	rsp.sense_data_length = sizeof(sense);

	/* populate sense data with a check condition */
	sense.response_code = 0x70;
	sense.sense_key = sense_key;
	sense.asc = asc;
	sense.ascq = ascq;
	sense.additional_sense_length = sizeof (sense) - 8;

	/* Send the response */
	return ocs_scsi_send_resp(io, OCS_SCSI_WQ_STEERING_CPU, &rsp, scsi_io_free_cb, NULL);
}

static int32_t
ocs_scsi_task_set_full_or_busy(ocs_io_t *io)
{
	int32_t rc;
	ocs_scsi_cmd_resp_t rsp;
	ocs_io_t *ioloop, *ionext;

	ocs_memset(&rsp, 0, sizeof(rsp));

	rsp.scsi_status = SPDK_SCSI_STATUS_BUSY; /* Busy */

	ocs_node_lock(io->node);
	if (!ocs_list_empty(&io->node->active_ios)) {
		ocs_list_foreach_safe(&io->node->active_ios, ioloop, ionext) {
			if (ioloop != io &&
					ioloop->tgt_io.lun == io->tgt_io.lun) {
				rsp.scsi_status = SCSI_STATUS_TASK_SET_FULL; /* Task set full */
				break;
			}
		}
	}
	ocs_node_unlock(io->node);

	rc = ocs_scsi_send_resp(io, OCS_SCSI_WQ_STEERING_CPU, &rsp, scsi_io_free_cb, NULL);
	if (rc == 0) {
		return SCSI_HANDLER_RESP_STARTED;
	}

	return rc;
}

static int32_t
scsi_write_notify_spdk(ocs_io_t *io)
{
	struct spdk_fc_task *primary = io->tgt_io.primary, *subtask, *tmp;
	struct spdk_scsi_dev *dev = io->node->tgt_node.scsi_dev;

	spdk_fc_queue_task(dev, primary);
	TAILQ_FOREACH_SAFE(subtask, &primary->fc_subtask_list, fc_link, tmp) {
		spdk_fc_queue_task(dev, subtask);
	}

	return 0;
}

static int32_t
scsi_write_dataphase_cb(ocs_io_t *io, ocs_scsi_io_status_e scsi_status, uint32_t flags, void *arg)
{
	ocs_t *ocs = io->ocs;

	if (scsi_status == OCS_SCSI_STATUS_GOOD) {
		if (io->tgt_io.current) {
			return ocs_write_phase_start(io);
		} else if (flags & OCS_SCSI_IO_CMPL) {
			return scsi_write_notify_spdk(io);
		}
	}

	ocs_scsi_check_cond(io, SNS_ABORTED_CMD, SNSCODE_OFF_ERR, 0x00);
	ocs_log_err(ocs, "Write IO failed status = %d, flags = %d\n", scsi_status, flags);
	return -1;
}

static int32_t
ocs_write_phase_start(ocs_io_t *io)
{
	ocs_t *ocs = io->ocs;
	struct spdk_fc_task *primary, *current, *subtask = NULL, *tmp;
	uint32_t flags = 0, xfer_bytes = 0, rc;
	int i, num_sges = io->sgl_allocated;

	primary = io->tgt_io.primary; 
	current = io->tgt_io.current;
	if (current == primary) {
		/* First phase */
		io->sgl[0].addr = (uintptr_t)primary->mobj->phys_addr;
		io->sgl[0].len  = primary->scsi.length;
		xfer_bytes 	+= io->sgl[0].len;
		num_sges --;
	} else {
		subtask = current;
	}

	ocs_assert(num_sges, 0);

	TAILQ_FOREACH_FROM_SAFE(subtask, &primary->fc_subtask_list, fc_link, tmp) {
		i = io->sgl_allocated - num_sges;

		io->sgl[i].addr	= (uintptr_t)subtask->mobj->phys_addr;
		io->sgl[i].len	= subtask->scsi.length;
		xfer_bytes	+= io->sgl[i].len;

		io->tgt_io.current = subtask;
		num_sges --;

		if (!num_sges) {
			/* no more room. */
			break;
		}
	}

	if (ocs->err_injection != INJECT_DROP_RESP && (!subtask || 
				(subtask == TAILQ_LAST(&primary->fc_subtask_list, fc_subtask_list)))) {
		io->tgt_io.current = NULL;
		flags |= OCS_SCSI_LAST_DATAPHASE;
	}

	/* Set the Request WQ Steering */
	flags |= OCS_SCSI_WQ_STEERING_CPU;

	rc = ocs_scsi_recv_wr_data(io, flags, NULL, io->sgl, (io->sgl_allocated - num_sges),
			xfer_bytes, scsi_write_dataphase_cb, NULL);
	if (rc) {
		ocs_scsi_check_cond(io, SNS_ABORTED_CMD, SNSCODE_OFF_ERR, 0x00);
		return -1;
	}

	return 0;
}

static int32_t
scsi_read_dataphase_cb(ocs_io_t *io, ocs_scsi_io_status_e scsi_status, uint32_t flags, void *arg)
{
	ocs_t *ocs = io->ocs;

	if (scsi_status == OCS_SCSI_STATUS_GOOD) {
		if (io->tgt_io.current) {
			return ocs_read_phase_start(io);
		} else if (flags & OCS_SCSI_IO_CMPL) {
			return ocs_scsi_send_resp(io, 0, NULL, scsi_io_free_cb, NULL);
		} else if (flags & OCS_SCSI_IO_CMPL_RSP_SENT) {
			return scsi_io_free_cb(io, 0, 0, NULL);
		}
	}

	ocs_log_err(ocs, "Read IO failed. status = %d, flags = %d\n", scsi_status, flags);
	ocs_scsi_check_cond(io, SNS_ABORTED_CMD, SNSCODE_OFF_ERR, 0x00);
	return -1;
}

int32_t
ocs_read_phase_start(ocs_io_t *io)
{
	ocs_t *ocs = io->ocs;
	struct spdk_fc_task *primary, *current, *subtask = NULL, *tmp;
	uint32_t flags = 0, xfer_bytes = 0, rc;
	int num_sges = io->sgl_allocated, i;

	primary = io->tgt_io.primary; 
	current = io->tgt_io.current;
	if (current == primary) {
		/* First phase */
		io->sgl[0].addr = spdk_vtophys(primary->scsi.iovs[0].iov_base);
		io->sgl[0].len  = primary->scsi.data_transferred;
		xfer_bytes 	+= io->sgl[0].len;
		num_sges --;
	} else {
		subtask = current;
	}

	ocs_assert(num_sges, 0);

	TAILQ_FOREACH_FROM_SAFE(subtask, &primary->fc_subtask_list, fc_link, tmp) {

		i = io->sgl_allocated - num_sges;
		io->sgl[i].addr = spdk_vtophys(subtask->scsi.iovs[0].iov_base);
		io->sgl[i].len  = subtask->scsi.data_transferred;
		xfer_bytes 	+= io->sgl[i].len;

		io->tgt_io.current = subtask;
		num_sges --;

		if (!num_sges) {
			/* no more room. */
			break;
		}
	}

	if (ocs->err_injection != INJECT_DROP_RESP && (!subtask || 
				(subtask == TAILQ_LAST(&primary->fc_subtask_list, fc_subtask_list)))) {
		io->tgt_io.current = NULL;
		flags |= OCS_SCSI_LAST_DATAPHASE;
	}

	/* Set the Request WQ Steering */
	flags |= OCS_SCSI_WQ_STEERING_CPU;

	rc = ocs_scsi_send_rd_data(io, flags, NULL, io->sgl, (io->sgl_allocated - num_sges),
			xfer_bytes, scsi_read_dataphase_cb, NULL);
	if (rc) {
		ocs_scsi_check_cond(io, SNS_ABORTED_CMD, SNSCODE_OFF_ERR, 0x00);
		return -1;
	}

	return 0;
}

static int
ocs_process_read_completion(ocs_io_t *io, struct spdk_fc_task *primary)
{
	ocs_t *ocs = io->ocs;
	int rc = 0;

	if (primary->scsi.status == SPDK_SCSI_STATUS_GOOD && io->exp_xfer_len) {
		io->tgt_io.current = io->tgt_io.primary;

		rc = ocs_read_phase_start(io);
		if (rc) {
			goto error;
		}

	} else {
		ocs_scsi_cmd_resp_t rsp;

		ocs_memset(&rsp, 0, sizeof(rsp));

		rsp.scsi_status = primary->scsi.status;
		rsp.sense_data  = primary->scsi.sense_data;
		rsp.sense_data_length = primary->scsi.sense_data_len;

		return ocs_scsi_send_resp(io, OCS_SCSI_WQ_STEERING_CPU, &rsp,
			 	scsi_io_free_cb, NULL);
	}

	return 0;
error:
	ocs_log_err(ocs, "ocs_process_read_completion Error.\n");	
	return -1;
}

static int
ocs_process_write_none_completion(ocs_io_t *io, struct spdk_fc_task *primary) {
	ocs_scsi_cmd_resp_t rsp;

	ocs_memset(&rsp, 0, sizeof(rsp));

	rsp.scsi_status = primary->scsi.status;;
	rsp.sense_data  = primary->scsi.sense_data;

	return ocs_scsi_send_resp(io, OCS_SCSI_WQ_STEERING_CPU, &rsp,
			scsi_io_free_cb, NULL);
}

void
process_task_completion(struct spdk_scsi_task *scsi_task)
{
	struct spdk_fc_task *task = (struct spdk_fc_task *)scsi_task;
	ocs_io_t *io = task->cpl_args;
	struct spdk_fc_task *primary;

	ocs_assert(io);
	ocs_assert(task);
	
	if (task->parent) {
		primary = task->parent;
	} else {
		primary = task;
	}

	primary->bytes_completed += task->scsi.length;

	if (task->parent != NULL && task->scsi.status != SPDK_SCSI_STATUS_GOOD) {
		memcpy(primary->scsi.sense_data, task->scsi.sense_data,
				task->scsi.sense_data_len);
		primary->scsi.sense_data_len = task->scsi.sense_data_len;
		primary->scsi.status = task->scsi.status;
	}

	if (primary->bytes_completed != primary->scsi.transfer_len) {
		return;
	}

	if (primary->scsi.dxfer_dir == SPDK_SCSI_DIR_FROM_DEV) {
		ocs_process_read_completion(io, primary);
	} else {
		ocs_process_write_none_completion(io, primary);
	}

	return;
}

void
process_task_mgmt_completion(struct spdk_scsi_task *scsi_task)
{
	uint8_t rspcode;
	struct spdk_fc_task *task = (struct spdk_fc_task *)scsi_task;
	ocs_io_t *io = task->cpl_args;

	ocs_assert(io);
	ocs_assert(task);

	switch (task->scsi.response) {
		case SPDK_SCSI_TASK_MGMT_RESP_COMPLETE:
		case SPDK_SCSI_TASK_MGMT_RESP_SUCCESS:
			rspcode = OCS_SCSI_TMF_FUNCTION_COMPLETE;
			break;
		case SPDK_SCSI_TASK_MGMT_RESP_INVALID_LUN:
			rspcode = OCS_SCSI_TMF_INCORRECT_LOGICAL_UNIT_NUMBER;
			break;
		default:
			rspcode = OCS_SCSI_TMF_FUNCTION_REJECTED;
	}

	ocs_scsi_send_tmf_resp(io, rspcode, NULL, scsi_io_free_cb, NULL);	
	return;
}


/* Target server template functions */

int32_t
ocs_scsi_tgt_driver_init(void)
{
	return 0;
}

int32_t
ocs_scsi_tgt_driver_exit(void)
{
	return 0;
}

int32_t
ocs_scsi_tgt_new_device(ocs_t *ocs)
{
	uint32_t total_ios;
	struct spdk_fc_hba_port *hba_port = NULL, *tmp;
	int32_t rc = 0;

	/* Get the max settings */
	ocs->tgt_ocs.max_sge = ocs_scsi_get_property(ocs, OCS_SCSI_MAX_SGE);
	ocs->tgt_ocs.max_sgl = ocs_scsi_get_property(ocs, OCS_SCSI_MAX_SGL);

	/* initialize IO watermark fields */
	ocs_atomic_init(&ocs->tgt_ocs.ios_in_use, 0);
	total_ios = ocs_scsi_get_property(ocs, OCS_SCSI_MAX_IOS);
	ocs->tgt_ocs.watermark_min = (total_ios * OCS_WATERMARK_LOW_PCT) / 100;
	ocs->tgt_ocs.watermark_max = (total_ios * OCS_WATERMARK_HIGH_PCT) / 100;
	ocs_atomic_init(&ocs->tgt_ocs.io_high_watermark, ocs->tgt_ocs.watermark_max);
	ocs_atomic_init(&ocs->tgt_ocs.watermark_hit, 0);
	ocs_atomic_init(&ocs->tgt_ocs.initiator_count, 0);

	/* Assign an conf HBAPort */
	ocs_lock(&g_spdk_config_lock);

	FIND_NODE_BY_ID(g_spdk_fc_hba_ports, (int)ocs->instance_index, hba_port, tmp);
	if (!hba_port) {
		 /* No Valid HBAPort node found. Keep the port in down. */
		ocs->dont_linkup = TRUE;
		ocs_log_err(ocs, "No valid [HBAPort%d] found for ocs%d\n",
				ocs->instance_index, ocs->instance_index);
	} else {
		/* If user configured WWNN and WWPN, the use them. */
		if (ocs_strlen(hba_port->wwnn)) {
			parse_wwn((char *)hba_port->wwnn, &ocs->xport->req_wwnn);
		}
		if (ocs_strlen(hba_port->wwnn)) {
			parse_wwn((char *)hba_port->wwpn, &ocs->xport->req_wwpn);
		}
	}

	ocs_unlock(&g_spdk_config_lock);

	/* If nvme capability is enabled, notify backend. */
	if (ocs->enable_nvme_tgt) {
		rc = ocs_nvme_hw_port_create(ocs);
	}

	return rc;
}

int32_t
ocs_scsi_tgt_del_device(ocs_t *ocs)
{
	
	/* If nvme capability is enabled, notify backend. */
	if (ocs->enable_nvme_tgt) {
		ocs_hw_port_cleanup(ocs);
	}

	return 0;
}

int32_t
ocs_scsi_tgt_new_domain(ocs_domain_t *domain)
{
	
	return 0;
}

void
ocs_scsi_tgt_del_domain(ocs_domain_t *domain)
{
	ocs_t *ocs = domain->ocs;

	if (ocs->enable_nvme_tgt && ocs_nvme_nport_delete(ocs)) {
		ocs_log_err(ocs, "Failed to delete nvme port \n");
	}

	if (ocs->enable_nvme_tgt && ocs_nvme_process_hw_port_offline(ocs)) {
		ocs_log_err(ocs, "Failed to bring down nvme port offline\n");
	}
}


int32_t
ocs_scsi_tgt_new_sport(ocs_sport_t *sport)
{
	ocs_t *ocs = sport->ocs;
	struct spdk_fc_hba_port *hba_port, *tmp;
	int32_t rc = -1;

	/* currently just the physical sport */
	if (sport->is_vport) {
		return rc;
	}

	ocs_lock(&g_spdk_config_lock);
	FIND_NODE_BY_ID(g_spdk_fc_hba_ports, (int)ocs->instance_index, hba_port, tmp);
	if (!hba_port) {
		goto done;
	}

	if (spdk_fc_cf_add_scsidev_port(hba_port, sport->fc_id) != 0) {
		goto done;
	}

	if (ocs->enable_nvme_tgt && ocs_nvme_process_hw_port_online(sport)) {
		ocs_log_err(ocs, "Failed to bring up nvme port online\n")
		goto done;
	}

	if (ocs->enable_nvme_tgt && ocs_nvme_nport_create(sport)) {
		goto done;
	}

	/* Success */
	rc = 0;
done:
	ocs_unlock(&g_spdk_config_lock);
	return rc;
}

void
ocs_scsi_tgt_del_sport(ocs_sport_t *sport)
{
	/* No SCSI SPDK call to delete port. */
}	

int32_t
ocs_scsi_validate_initiator(ocs_node_t *node)
{
	ocs_t	*ocs = NULL;
	uint64_t wwpn = ocs_node_get_wwpn(node);
	int32_t rc = 0; /* Dont allow. */

	ocs = node->sport->ocs;
	if (!ocs) {
		ocs_log_err(node->ocs, "Can't accept initiator, target data missing\n");
		return rc;
	}

	ocs_lock(&g_spdk_config_lock);
	if (!spdk_fc_cf_get_initiator_scsidev(ocs->instance_index, wwpn)) {
		ocs_log_err(node->ocs, "Can't accept initiator, No LUN Mapping.\n");
		goto done;
	}

	ocs_log_info(node->ocs, "%s: access granted for initiator: %s\n",
			__func__, node->wwpn);
	rc = 1; /* Allow */

done:
	ocs_unlock(&g_spdk_config_lock);
	return rc;
}

int32_t
ocs_scsi_new_initiator(ocs_node_t *node)
{
	ocs_t	*ocs = NULL;
	uint64_t wwpn = ocs_node_get_wwpn(node);
	int32_t rc = -1;
	char pname[128];

	ocs = node->sport->ocs;
	if (!ocs) {
		ocs_log_err(node->ocs, "Can't accept initiator, target data missing\n");
		return rc;
	}

	ocs_lock(&g_spdk_config_lock);
	node->tgt_node.scsi_dev = spdk_fc_cf_get_initiator_scsidev(ocs->instance_index, wwpn);
	if (!node->tgt_node.scsi_dev) {
		ocs_log_err(node->ocs, "Can't accept initiator, no scsidev.\n");
		goto done;
	}

	node->tgt_node.core_id = spdk_fc_cf_get_io_channel(node->tgt_node.scsi_dev);
	if (node->tgt_node.core_id < 0) {
		ocs_log_err(node->ocs, "Can't accept initiator, IOchannel alloc failed.\n");
		goto done;
	}

	OCS_SPDK_BUILD_NODE_NAME;
	node->tgt_node.initiator_port = spdk_scsi_port_create(node->instance_index, 0, pname);
	rc = 0;
done:
	ocs_unlock(&g_spdk_config_lock);
	return rc;
}

int32_t
ocs_scsi_del_initiator(ocs_node_t *node, ocs_scsi_del_initiator_reason_e reason)
{
	return 0;
}


int32_t
ocs_scsi_recv_cmd_first_burst(ocs_io_t *io, uint32_t lun, uint8_t *cdb, uint32_t cdb_len,
		uint32_t flags, ocs_dma_t first_burst_buffers[], uint32_t first_burst_buffer_count)
{
	return 0;
}

static void
ocs_scsi_recv_cmd_process(void *arg1, void *arg2)
{
	ocs_io_t *io = arg1;
	ocs_t *ocs = io->ocs;
	int rc, active_io_count;
	struct spdk_fc_task *task;
	ocs_node_t *node = io->node;
	struct spdk_scsi_dev *dev = node->tgt_node.scsi_dev;
	uint32_t lun = io->tgt_io.lun;
	uint32_t flags = io->tgt_io.flags;

	if (!dev) {
		ocs_log_err(ocs, "%s: No scsi dev.\n", __func__);
		goto busy;
	}

	// Check if node is dirty and send lun inventory changed.
	if (node->tgt_node.dirty) {
		if (io->tgt_io.cdb[0] == SPDK_SPC_REPORT_LUNS) {
			// If report luns is issued, clear dirty.
			node->tgt_node.dirty = false;
		} else {
			ocs_scsi_check_cond(io, SNS_UNIT_ATTENTION, 0x3F, 0x0E);
			return;
		}
	}

	io->timeout 	= ocs->target_io_timer_sec;

	active_io_count = ocs_atomic_add_return(&ocs->tgt_ocs.ios_in_use, 1);
	if (active_io_count >= ocs_atomic_read(&ocs->tgt_ocs.io_high_watermark)) {
		ocs_atomic_add_return(&ocs->tgt_ocs.watermark_hit, 1);
		goto busy;
	}

	if ((flags & OCS_SCSI_CMD_DIR_IN) && (flags & OCS_SCSI_CMD_DIR_OUT)) {
		goto bad_cmd;
	} 

	task  = spdk_fc_task_get(&node->tgt_node.pending_task_cnt, NULL,
			process_task_completion, io);
	if (!task) {
		goto busy;
	}

	io->tgt_io.primary = task;

	task->mobj = NULL;
	task->parent = NULL;
	task->scsi.lun	  = spdk_scsi_dev_get_lun(dev, lun);
	task->scsi.cdb 	  = io->tgt_io.cdb;
	task->scsi.transfer_len	  = io->exp_xfer_len;
	task->scsi.initiator_port = node->tgt_node.initiator_port;
	task->scsi.target_port    = spdk_scsi_dev_find_port_by_id(dev, node->sport->tgt_id);

	if (task->scsi.lun == NULL) {
		spdk_scsi_task_process_null_lun(&task->scsi);
		return spdk_fc_queue_task(dev, task);
	}

	if (flags & OCS_SCSI_CMD_DIR_OUT) { // READ
		int32_t remaining_size = 0;
		uint32_t offset = 0;
		struct spdk_fc_task *subtask;

		TAILQ_INIT(&task->fc_subtask_list);
		task->scsi.dxfer_dir 		= SPDK_SCSI_DIR_FROM_DEV;
		task->scsi.iovs[0].iov_base 	= NULL;
		task->scsi.offset 		= 0;
		task->scsi.length 		= MIN(SPDK_FC_LARGE_RBUF_MAX_SIZE,
							task->scsi.transfer_len);

		spdk_fc_queue_task(dev, task);

		remaining_size = task->scsi.transfer_len - task->scsi.length;
		offset += task->scsi.length;
		
		while (remaining_size > 0) {
			subtask = spdk_fc_task_get(&node->tgt_node.pending_task_cnt, task,
					process_task_completion, io);
			if (!subtask) {
				goto busy;
			}

			subtask->scsi.iovs[0].iov_base	= NULL;
			subtask->mobj 		= NULL;
			subtask->scsi.offset 	= offset;
			subtask->scsi.length 	= MIN(SPDK_FC_LARGE_RBUF_MAX_SIZE,
					remaining_size);

			TAILQ_INSERT_TAIL(&task->fc_subtask_list, subtask, fc_link);

			spdk_fc_queue_task(dev, subtask);

			remaining_size -= subtask->scsi.length;
			offset += subtask->scsi.length;
		}

	} else if (flags & OCS_SCSI_CMD_DIR_IN) {
		/* Allocate write buffers */
		int32_t remaining_size = 0;
		uint32_t offset = 0;
		struct spdk_fc_task *subtask;

		TAILQ_INIT(&task->fc_subtask_list);
		task->scsi.dxfer_dir	= SPDK_SCSI_DIR_TO_DEV;
		task->scsi.offset	= 0;
		task->scsi.iovs[0].iov_base	= NULL;
		task->scsi.length	= MIN(SPDK_FC_LARGE_RBUF_MAX_SIZE,
						task->scsi.transfer_len);

		/* Allocate write buffer for task. */	
		rc = rte_mempool_get(g_spdk_fc.buff_pool, (void **)&task->mobj);
		if ((rc < 0) || !task->mobj) {
			goto busy;
		}

		task->scsi.iovs[0].iov_base	= task->mobj->buf;
		task->scsi.iovs[0].iov_len	= task->scsi.length;
		io->tgt_io.current	= io->tgt_io.primary;

		remaining_size = task->scsi.transfer_len - task->scsi.length;
		offset += task->scsi.length;

		while (remaining_size > 0) {
			subtask = spdk_fc_task_get(&node->tgt_node.pending_task_cnt, task,
					process_task_completion, io);
			if (!subtask) {
				goto busy;
			}
			subtask->mobj = NULL;

			TAILQ_INSERT_TAIL(&task->fc_subtask_list, subtask, fc_link);

			/* Allocate write buffer for subtask task. */	
			rc = rte_mempool_get(g_spdk_fc.buff_pool, (void **)&subtask->mobj);
			if ((rc < 0) || !subtask->mobj) {
				goto busy;
			}

			subtask->scsi.offset	= offset;
			subtask->scsi.length	= MIN(SPDK_FC_LARGE_RBUF_MAX_SIZE,
					remaining_size);
			subtask->scsi.iovs[0].iov_base = subtask->mobj->buf;
			subtask->scsi.iovs[0].iov_len  = subtask->scsi.length;


			remaining_size -= subtask->scsi.length;
			offset += subtask->scsi.length;
		}

		/* Get the data from initiator */
		rc = ocs_write_phase_start(io);
		if (rc) {
			/* Check condition already sent */
			return;	
		}

	} else {
		task->scsi.dxfer_dir = SPDK_SCSI_DIR_NONE;
		if (task->scsi.transfer_len > 0) {
			goto bad_cmd;
		}
		spdk_fc_queue_task(dev, task);
	}

	return;

busy:
	ocs_log_err(ocs, "IO failed due to busy.\n");
	ocs_scsi_task_set_full_or_busy(io);
	return;

bad_cmd:
	ocs_log_err(ocs, "IO failed due to illegal request.\n");
	ocs_scsi_check_cond(io, SNS_ILLEGAL_REQ, SNSCODE_BAD_CMD, 0);
	return;
}

int32_t
ocs_scsi_recv_cmd(ocs_io_t *io, uint32_t lun, uint8_t *cdb, uint32_t cdb_len, uint32_t flags)
{
	struct spdk_event *event;
	ocs_node_t *node = io->node;
	ocs_t *ocs = io->ocs;

	if (node->tgt_node.core_id < 0) {
		ocs_log_err(ocs, "%s: Core ID invalid.\n", __func__);
		goto fail;
	}


	ocs_memset(&io->tgt_io, 0, sizeof(io->tgt_io));
	io->tgt_io.lun 	 = lun;
	io->tgt_io.flags = flags;	
	io->tgt_io.cdb_len = cdb_len;
	ocs_memcpy(io->tgt_io.cdb, cdb, MIN(cdb_len, ARRAY_SIZE(io->tgt_io.cdb)));
		
	/* Schedule this on correct iochannel of scsidev */
	event = spdk_event_allocate(node->tgt_node.core_id, 
			ocs_scsi_recv_cmd_process, io, NULL);
	spdk_event_call(event);

	return 0;
fail:
	ocs_scsi_task_set_full_or_busy(io);
	return -1;
}


static void
ocs_scsi_recv_tmf_process(void *arg1, void *arg2)
{
	ocs_io_t *tmfio = arg1;
	ocs_t *ocs = tmfio->ocs;
	ocs_node_t *node = tmfio->node;
	struct spdk_fc_task *task;
	struct spdk_scsi_dev *dev = tmfio->node->tgt_node.scsi_dev;
	uint32_t lun = tmfio->tgt_io.lun;
	ocs_scsi_tmf_cmd_e cmd = tmfio->tgt_io.cmd;
	enum spdk_scsi_task_func function;

	if (!dev) {
		ocs_log_err(ocs, "%s: No scsi dev.\n", __func__);
		goto fail;
	}

	/* No watermark checks for task management */
	ocs_atomic_add_return(&ocs->tgt_ocs.ios_in_use, 1);

	task = spdk_fc_task_get(&node->tgt_node.pending_task_cnt, NULL, 
			process_task_mgmt_completion, tmfio);
	if (!task) {
		ocs_log_err(ocs, "%s: No FC task available.\n", __func__);
		goto fail;
	}

	tmfio->tgt_io.primary = task;

	task->scsi.initiator_port = node->tgt_node.initiator_port;
	task->scsi.target_port    = spdk_scsi_dev_find_port_by_id(dev, node->sport->tgt_id);
	task->scsi.lun 		  = spdk_scsi_dev_get_lun(dev, lun);
	if (!task->scsi.lun) {
		ocs_scsi_send_tmf_resp(tmfio, OCS_SCSI_TMF_INCORRECT_LOGICAL_UNIT_NUMBER,
			NULL, scsi_io_free_cb, NULL);
		return;
	}

	switch (cmd) {
		case OCS_SCSI_TMF_ABORT_TASK:
			function = SPDK_SCSI_TASK_FUNC_ABORT_TASK;
			task->scsi.abort_id = tmfio->tgt_io.abort_tag;
			break;
		case OCS_SCSI_TMF_ABORT_TASK_SET:
			function = SPDK_SCSI_TASK_FUNC_ABORT_TASK_SET;
			break;
		case OCS_SCSI_TMF_LOGICAL_UNIT_RESET:
			function = SPDK_SCSI_TASK_FUNC_LUN_RESET;
			break;
		case OCS_SCSI_TMF_CLEAR_ACA:
		case OCS_SCSI_TMF_CLEAR_TASK_SET:
		case OCS_SCSI_TMF_TARGET_RESET:
		default:
			goto fail;
	}

	spdk_fc_queue_mgmt_task(dev, task, function);

	return;

fail:
	ocs_log_debug(ocs, "TMF IO for cmd = %d aborted.\n", cmd);

	ocs_scsi_send_tmf_resp(tmfio, OCS_SCSI_TMF_FUNCTION_REJECTED, NULL,
			scsi_io_free_cb, NULL);
	return;	
}

int32_t
ocs_scsi_recv_tmf(ocs_io_t *tmfio, uint64_t lun, ocs_scsi_tmf_cmd_e cmd, ocs_io_t *abortio, uint32_t flags)
{
	struct spdk_event *event;
	ocs_node_t *node = tmfio->node;
	ocs_t *ocs = tmfio->ocs;

	if (node->tgt_node.core_id < 0) {
		ocs_log_err(ocs, "%s: Core ID invalid.\n", __func__);
		goto fail;
	}

	if (!abortio) {
		ocs_log_err(ocs, "%s: Abort IO invalid.\n", __func__);
		goto fail;
	}

	ocs_memset(&tmfio->tgt_io, 0, sizeof(tmfio->tgt_io));
	tmfio->tgt_io.lun	= lun;
	tmfio->tgt_io.flags	= flags;
	tmfio->tgt_io.cmd 	= cmd;
	tmfio->tgt_io.abort_tag	= abortio->tag;
		
	/* Schedule this on correct iochannel of scsidev */
	event = spdk_event_allocate(node->tgt_node.core_id, 
			ocs_scsi_recv_tmf_process, tmfio, NULL);
	spdk_event_call(event);

	return 0;
fail:
	ocs_scsi_send_tmf_resp(tmfio, OCS_SCSI_TMF_FUNCTION_REJECTED, NULL,
			scsi_io_free_cb, NULL);
	return -1;
}

void
ocs_scsi_tgt_ddump(ocs_textbuf_t *textbuf, ocs_scsi_ddump_type_e type, void *obj)
{
}

void
ocs_scsi_tgt_mgmt_status(ocs_textbuf_t *textbuf, ocs_scsi_ddump_type_e type, void *obj)
{
}

void
ocs_scsi_tgt_mgmt_config(ocs_textbuf_t *textbuf, ocs_scsi_ddump_type_e type, void *obj)
{
}

int
ocs_get_sense_info(uint8_t *sense_buf, uint8_t sense_len,
	struct sense_info *sense_info)
{
	sense_data_t *sense = (sense_data_t *) sense_buf;
	uint8_t sense_key, asc, ascq;

	ocs_assert(sense_info, -1);
	if (sense->response_code == 0x70 ||
	    sense->response_code == 0x71) {
		/* Fixed format sense data */
		if (sense_len < 14) {
			SPDK_NOTICELOG(">> Sense buffer content is too small\n");
			return -1;
		}
		sense_key = sense->fixed.sense_key;
		asc = sense->fixed.asc;
		ascq = sense->fixed.ascq;
	} else if (sense->response_code == 0x72 ||
		   sense->response_code == 0x73) {
		/* Descriptor format sense data */
		if (sense_len < 4) {
			SPDK_NOTICELOG(">> Sense buffer content is too small\n");
			return -1;
		}
		sense_key = sense->descriptor.sense_key;
		asc = sense->descriptor.asc;
		ascq = sense->descriptor.ascq;
	} else {
		SPDK_NOTICELOG(">> Invalid sense type\n");
		return -1;
	}

	sense_info->response_code = sense->response_code;
	sense_info->sense_key = sense_key;
	sense_info->asc = asc;
	sense_info->ascq = ascq;
	return 0;
}

