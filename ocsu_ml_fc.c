/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2016-2018 Broadcom.  All Rights Reserved.
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

#include "ocs.h"
#include "spdk/log.h"
#include "spdk_fc_ini_discovery.h"
#include "ocs_tgt_spdk.h"
#include "ocs_fc_ini_errors.h"

/* defines */
#define IOFMT "[%04x][i:%04x t:%04x h:%04x]"
#define IOFMT_ARGS(io) io->instance_index, io->init_task_tag, io->tgt_task_tag, io->hw_tag
#define io_printf(io, fmt, ...)  \
	do { \
		ocs_log_debug(io->node->ocs, "[%s]" IOFMT fmt, \
			io->node->display_name, IOFMT_ARGS(io), ##__VA_ARGS__); \
	} while (0)

#define OCS_ML_SCSI_TMF_TIMEOUT			30 /* 30 seconds */

/* Flag to indicate block layer readiness*/
bool block_ready = FALSE;

typedef struct {
	ocs_sem_t sem;
	int status;
	int count;
	ocs_lock_t lock;
} bus_reset_param_t;

static inline void
ocs_scsi_midlayer_result(spdk_fc_ini_cmd_complete_info_t *cmd_compl_info, char host_code, char scsi_code)
{
	cmd_compl_info->cmd_result.host_result = host_code;
	cmd_compl_info->cmd_result.tgt_result = scsi_code;
}

/**
 * @brief Set scsi_cmnd check condition.
 *
 * @par Description
 * This function is called by the lower layer driver to indicate that the given
 * scsi_cmnd resulted in a check condition. Sense data is
 * filled out independently.
 *
 * @param scsi_cmd Command for which to set the check condition.
 *
 * @return None.
 */
static inline void
ocs_scsi_cmnd_result_chk_cond(spdk_fc_ini_cmd_complete_info_t *cmd_compl_info)
{

//	sg_io->driver_status = DRIVER_SENSE; //FIXME

	cmd_compl_info->cmd_result.host_result = SPDK_FC_INI_SUCCESS;
	cmd_compl_info->cmd_result.tgt_result = SAM_STAT_CHECK_CONDITION;
}

/**
 * @brief Get TMF status
 *
 * @param ocs Pointer to ocs object
 * @param scsi_status Status of TMF
 * @param rsp Pointer to response
 *
 * @return Status to return to mid-layer
 */
static int
ocs_ml_get_tmf_status(ocs_t *ocs, ocs_scsi_io_status_e scsi_status, ocs_scsi_cmd_resp_t *rsp)
{
	int status;

	switch (scsi_status) {
	case OCS_SCSI_STATUS_GOOD:
	case OCS_SCSI_STATUS_NO_IO:
		status = SPDK_FC_INI_SUCCESS;
		break;
	case OCS_SCSI_STATUS_CHECK_RESPONSE:
		if (rsp->response_data_length == 0) {
			ocs_log_test(ocs, "check response without data?!?\n");
			status = SPDK_FC_INI_ERROR_TMF;
		}

		if (rsp->response_data[3] == FCP_TMF_COMPLETE) {
			status = SPDK_FC_INI_SUCCESS;
		} else {
			status = SPDK_FC_INI_ERROR_TMF;
		}
		break;
	case OCS_SCSI_STATUS_COMMAND_TIMEOUT:
		status = SPDK_FC_INI_ERROR_TIMEOUT;
		break;
	default:
		ocs_log_test(ocs, "status=%#x\n", scsi_status);
		status = SPDK_FC_INI_ERROR_GENERIC;
		break;
	}
	return status;
}

/* forward declarations */
#if 0
static const char *ocs_info(struct Scsi_Host *host);
#endif
static int32_t ocs_scsi_cmd_cb(ocs_io_t *, ocs_scsi_io_status_e, ocs_scsi_cmd_resp_t *, uint32_t, void *);
static int32_t ocs_scsi_tmf_cb(ocs_io_t *, ocs_scsi_io_status_e, ocs_scsi_cmd_resp_t *, uint32_t, void *);
static int32_t ocs_scsi_map_sgl(ocs_t *, ocs_blk_req_t *, ocs_scsi_sgl_t *, uint32_t *);

#if 0
/**
 * @brief The scsi_host_template information entry point.
 *
 * @par Description
 * This routine returns information about the given OCS
 * instance.
 *
 * @param shost Pointer to scsi_host structure.
 *
 * @return None.
 */
static const char *
ocs_info(struct Scsi_Host *shost)
{
	ocs_vport_t *vport = (ocs_vport_t *)shost->hostdata;
	ocs_t *ocs = vport->ocs;
	static char ocsbuf[256];
	ocs_snprintf(ocsbuf, sizeof(ocsbuf), "%s %s",
				 DRV_NAME, ocs_display_name(ocs));
	return ocsbuf;
}
#endif

int
ocs_ml_fc_queue_cmd(ocs_blk_req_t *req)
{
	ocs_node_t *node;
	ocs_t *ocs;
	ocs_io_t *io = NULL;
	ocs_scsi_sgl_t *sgl;
	uint32_t sgl_count;
	int32_t	rc = -1;

#if 0
	/* first check if node has been deleted */
	if (!node || !rport) {
		ocs_log_test(ocs, "IO for deleted node\n");
		ocs_scsi_midlayer_result(cmd, DID_BAD_TARGET, 0);
		goto out_fail_command;
	}

	/* node exists; make sure it's ready to accept IOs */
	err = fc_remote_port_chkready(rport);
	if (err) {
		cmd->result = err;
		goto out_fail_command;
	}
#endif

	ocs_assert(req, -1);
	node = req->node;
	ocs_assert(node, -1);
	ocs = node->ocs;
	ocs_assert(ocs, -1);

	/* allocate an IO */
	io = ocs_scsi_io_alloc(node, OCS_SCSI_IO_ROLE_ORIGINATOR);
	if (io == NULL)
		return SPDK_FC_INI_ERROR_NO_MEM;

	io->wq_steering = OCS_HAL_WQ_STEERING_CPU;
	io->timeout = req->timeout ? : 30; // Use user provided timeout, else default to 30 secs
	sgl = io->sgl;

	/* save mid-layer command and callback function */
	ocs_memset(&io->ini_io, 0, sizeof(io->ini_io));
	io->ini_io.req = req;

	if (req->iovcnt) {
		/* map SGL from Mid-layer to OCS SGL */
		sgl_count = io->sgl_allocated;
		rc = ocs_scsi_map_sgl(ocs, req, sgl, &sgl_count);
		if (rc)	
			goto free_io;

		io->sgl_count = sgl_count;

		switch (req->data_dir) {
		case SPDK_SCSI_DIR_FROM_DEV:
			/* read command */
			rc = ocs_scsi_send_rd_io(node, io, req->lun,
				req->scsi_cdb, req->cdb_len, NULL,
				sgl, sgl_count, req->data_len,
				ocs_scsi_cmd_cb, NULL);
			break;
		case SPDK_SCSI_DIR_TO_DEV:
			/* write command */
			rc = ocs_scsi_send_wr_io(node, io, req->lun,
				req->scsi_cdb, req->cdb_len, NULL,
				sgl, sgl_count, req->data_len,
				ocs_scsi_cmd_cb, NULL);
			break;
		default:
			ocs_log_test(ocs, "default %d\n", req->data_dir);
		}
	} else {
		/* no data transfer */
		rc = ocs_scsi_send_nodata_io(node, io, req->lun,
			req->scsi_cdb, req->cdb_len,
			ocs_scsi_cmd_cb, NULL);
	}

	if (rc)
		goto free_io;

	return SPDK_FC_INI_SUCCESS;

free_io:
	ocs_scsi_io_free(io);
	return rc;
}

#if 0
static ocs_node_t *
ocs_ml_get_node(struct scsi_cmnd *cmd)
{
	ocs_rport_data_t *rport_data;

	if (!cmd || !cmd->device || !cmd->device->hostdata) {
		return NULL;
	}

	rport_data = (ocs_rport_data_t *)cmd->device->hostdata;

	return rport_data->node;
}
#endif

int
ocs_lun_reset(ocs_blk_req_t *req)
{
	ocs_node_t *node;
	ocs_t *ocs;
	ocs_io_t *io = NULL;
	int32_t	rc = 0;

	ocs_assert(req, -1);
	node = req->node;
	ocs_assert(node, -1);
	ocs = node->ocs;
	ocs_assert(ocs, -1);

	io = ocs_scsi_io_alloc(node, OCS_SCSI_IO_ROLE_ORIGINATOR);
	if (io == NULL)
		return SPDK_FC_INI_ERROR_NO_MEM;

	io->wq_steering = OCS_HAL_WQ_STEERING_CPU;
	io->timeout = req->timeout ? : 30; // FIXME: Is 30 seconds enough?

	ocs_memset(&io->ini_io, 0, sizeof(io->ini_io));
	io->ini_io.req = req;

	rc =  ocs_scsi_send_tmf(req->node, io, NULL, req->lun,
				 OCS_SCSI_TMF_LOGICAL_UNIT_RESET, NULL, 0, 0,
				 ocs_scsi_tmf_cb, NULL);
	if (rc) {
		ocs_scsi_io_free(io);
		return rc;
	}
	return SPDK_FC_INI_SUCCESS;
}

static int32_t
ocs_scsi_tmf_cb(
	ocs_io_t *io,
	ocs_scsi_io_status_e status,
	ocs_scsi_cmd_resp_t *rsp,
	uint32_t flags,
	void *arg)
{
	ocs_blk_req_t *req;
//	ocs_blk_req_result_t *cmd_result;
	int tmf_status;

	ocs_assert(io, -1);
	req = io->ini_io.req;
	ocs_assert(req, -1);
//	cmd_result = &req->cmd_compl_info.cmd_result;

	tmf_status = ocs_ml_get_tmf_status(io->ocs, status, rsp);

	/* Fill tmf_status in host_code. FIXME: fill scsi_code? */
	ocs_scsi_midlayer_result(&req->cmd_compl_info, tmf_status, 0);

	spdk_event_call(req->done_event);
	ocs_scsi_io_free(io);
	return 0;
}

/**
 * @brief The SCSI IO callback function.
 *
 * @par Description
 * This function is called back by the base driver to indicate
 * that the given IO has completed.
 *
 * @param io Pointer to the IO that has completed.
 * @param status Status of the IO.
 * @param rsp Pointer to the response.
 * @param flags Flags.
 * @param arg Pointer to the argument passed when sending the IO.
 *
 * @return None.
 */
static int32_t
ocs_scsi_cmd_cb(
	ocs_io_t *io,
	ocs_scsi_io_status_e status,
	ocs_scsi_cmd_resp_t *rsp,
	uint32_t flags,
	void *arg)
{
	ocs_blk_req_t *req;
	ocs_blk_req_result_t *cmd_result;
	int sense_len;
	uint64_t lun;

	ocs_assert(io, -1);
	req = io->ini_io.req;
	ocs_assert(req, -1);
	lun = req->lun;
	cmd_result = &req->cmd_compl_info.cmd_result;

	cmd_result->data_transferred = req->data_len - rsp->residual;
	switch (status) {
	case OCS_SCSI_STATUS_GOOD:
		/* success */
		ocs_scsi_midlayer_result(&req->cmd_compl_info, SPDK_FC_INI_SUCCESS, rsp->scsi_status);
		break;
	case OCS_SCSI_STATUS_CHECK_RESPONSE:
		if (rsp->scsi_status == 2) {
			struct sense_info sense_info;
			int rc;
			/* set check condition indication in cmd_complete_info and copy sense data */
			sense_len = MIN(rsp->sense_data_length, OCS_SENSE_BUF_LEN);
			cmd_result->scsi_resp_len = sense_len;
			ocs_scsi_cmnd_result_chk_cond(&req->cmd_compl_info);
			ocs_memcpy(cmd_result->scsi_resp, rsp->sense_data, sense_len);
			rc = ocs_get_sense_info(cmd_result->scsi_resp,
				cmd_result->scsi_resp_len, &sense_info);
			if (!rc) {
				SPDK_ERRLOG(
					"Sense data - rc=%x, sk=%x, asc=%x, ascq=%x\n",
					sense_info.response_code, sense_info.sense_key,
					sense_info.asc, sense_info.ascq);
				if (sense_info.sense_key == 0x06) { /* Unit Atention */
					if ((sense_info.asc == 0x3F &&
					     sense_info.ascq == 0x0E) ||
					    (sense_info.asc == 0x2A &&
					     sense_info.ascq == 0x09)) {
						/* REPORTED LUNS DATA  or CAPACITY DATA HAS CHANGED*/
						spdk_fc_ini_di_update_devs_notify(
							INI_NODE_EVENT_TYPE_TGT_LUNS,
							io->node);
					}
				}
			} else {
				SPDK_ERRLOG(
					"Sense data not available\n");
			}
		} else if (rsp->scsi_status == 0 && rsp->residual) {
			uint32_t host_status = SPDK_FC_INI_SUCCESS;
			/* residual non-zero but good status; treat as okay */
			/* Send SCSI status as error if there is any residual over case  */
			if (rsp->residual < 0) {
				host_status = SPDK_FC_INI_ERROR_XFER_LEN;
				io_printf(io, "RESID_OVER: OCS_SCSI_STATUS_ERROR, LUN %" LUNFMTd ", scsi_status=0x%02x\n",
					lun, rsp->scsi_status);

			} else {
//				fcp_rsp_iu_t *fcprsp = io->rspbuf.virt;
				int32_t fcpi_parm = (io->wire_len) - (rsp->response_wire_length);

				/* There is a dropped frame if under run reported by
				 * storage array is different from under run reported by HBA.
				 */
				if ((req->data_dir == SPDK_SCSI_DIR_FROM_DEV) &&
					fcpi_parm && (rsp->residual != fcpi_parm)) {
					host_status = SPDK_FC_INI_ERROR_XFER_LEN;
					io_printf(io, "RESID_UNDER: OCS_SCSI_STATUS_ERROR, LUN %" LUNFMTd ", scsi_status=0x%02x\n",
						lun, rsp->scsi_status);
				}

//FIXME: Not applicable?
#if 0
				/*
				 * The scsi_cmd->underflow is the minimum number of bytes that must
				 * be transferred for this command.  Provided a sense condition
				 * is not present, make sure the actual amount transferred is at
				 * least the underflow value or fail.
				 */
				if (!(fcprsp->flags & FCP_SNS_LEN_VALID) && (rsp->scsi_status == 0) &&
					((sg_io->dxfer_len - rsp->residual) < scsi_cmd->underflow)) {
					host_status = DID_ERROR;
					io_printf(io, "RESID_UNDER: OCS_SCSI_STATUS_ERROR, LUN %" LUNFMTd ", scsi_status=0x%02x\n",
							  lun, rsp->scsi_status);
				}
#endif
			}
			ocs_scsi_midlayer_result(&req->cmd_compl_info, host_status, rsp->scsi_status);
		} else if (rsp->scsi_status == 8) {
			/* set BUSY status */
			io_printf(io, "LUN %" LUNFMTd " setting BUSY, scsi_status=%d\n",
				lun, rsp->scsi_status);
			ocs_scsi_midlayer_result(&req->cmd_compl_info, SPDK_FC_INI_SUCCESS, SAM_STAT_BUSY);
		} else if (rsp->scsi_status == 0x28) {
			/* set task set full status */
			io_printf(io, "LUN %" LUNFMTd " setting TASK_SET_FULL, scsi_status=0x%02x\n",
				lun, rsp->scsi_status);
			ocs_scsi_midlayer_result(&req->cmd_compl_info, SPDK_FC_INI_SUCCESS, SAM_STAT_TASK_SET_FULL);
		} else {
			/* everything else; send generic error */
			/* TODO need to check for other non-generic cases and set midlayer result */
			io_printf(io, "OCS_SCSI_STATUS_ERROR, LUN %" LUNFMTd ", scsi_status=0x%02x\n",
				lun, rsp->scsi_status);
			ocs_scsi_midlayer_result(&req->cmd_compl_info, SPDK_FC_INI_ERROR_GENERIC, 0);
		}
		break;
	case OCS_SCSI_STATUS_COMMAND_TIMEOUT:
		ocs_scsi_midlayer_result(&req->cmd_compl_info, SPDK_FC_INI_ERROR_TIMEOUT, 0);
		break;
	case OCS_SCSI_STATUS_ABORTED:
		ocs_scsi_midlayer_result(&req->cmd_compl_info, SPDK_FC_INI_ABORTED, 0);
		break;
	case OCS_SCSI_STATUS_ERROR:
	case OCS_SCSI_STATUS_PROTOCOL_CRC_ERROR:
		ocs_scsi_midlayer_result(&req->cmd_compl_info, SPDK_FC_INI_ERROR_GENERIC, 0);
		break;
	default:
		ocs_scsi_midlayer_result(&req->cmd_compl_info, SPDK_FC_INI_ERROR_GENERIC, 0);
		io_printf(io, "Unhandled status=%d\n", status);
		break;
	}

	spdk_event_call(req->done_event);
	ocs_scsi_io_free(io);
	return 0;
}

/**
 * @brief Map mid-layer SGLs to ocs SGLs.
 *
 * @par Description
 * This function is called by ocs_ml_fc_queue_cmd() to map the SGL
 * received from the block layer to an ocs SGL that can be used by
 * the SCSI API.
 *
 * @param ocs The ocs pointer associated with this request.
 * @param req The request received from the block layer.
 * @param sgl The resulting ocs SGL.
 * @param sgl_cnt Passed in - the maximum number of SGLs that @c sgl can
 * accommodate; Returned out - the resulting number of SGL
 * entries.
 *
 * @return None.
 */
static int32_t
ocs_scsi_map_sgl(
	ocs_t *ocs,
	ocs_blk_req_t *req,
	ocs_scsi_sgl_t *sgl,
	uint32_t *sgl_cnt)
{
	uint32_t num_seg = 0;
	uint32_t i = 0;
	struct iovec *iov = req->iov;

	ocs_assert(req->iovcnt, -1);
	num_seg = req->iovcnt;

	if (num_seg > *sgl_cnt) {
		ocs_log_test(ocs, "num SGEs=%d requested exceeds max=%d\n",
			num_seg, *sgl_cnt);
		return SPDK_FC_INI_ERROR_ILLEGAL_REQUEST;
	}

	/* loop through each iovec and fill out OCS-specific SGL */
	for (i = 0; i < num_seg; i++) {
		sgl[i].addr = (uintptr_t)iov[i].iov_base;
		sgl[i].len = iov[i].iov_len;
	}
	*sgl_cnt = i;

	return SPDK_FC_INI_SUCCESS;
}

/* ocsu_ini_init - Called when the discovery code is ready to receive */
/* spdk_fc_ini_di_update_devs_notify() callbacks from the OCS driver. */
void
ocsu_ini_init(void)
{
	ocs_t *ocs_devices[MAX_OCS_DEVICES];
	ocs_domain_t *domain;
	ocs_sport_t *sport;
	ocs_node_t *node;
	uint32_t num_ports, i;

	SPDK_NOTICELOG(">> ocsu_ini_init() CALLED\n");
	num_ports = ocsu_ini_get_initiators(ocs_devices, MAX_OCS_DEVICES);

	if (num_ports > MAX_OCS_DEVICES)
		SPDK_NOTICELOG(">> Too many initiator ports: This should not happen\n");

	for (i = 0; i < num_ports; i++) {
		ocs_list_foreach(&ocs_devices[i]->domain_list, domain) {
			ocs_list_foreach(&domain->sport_list, sport) {
				ocs_list_foreach(&sport->node_list, node) {
					if (node->targ)
						spdk_fc_ini_di_update_devs_notify(INI_NODE_EVENT_TYPE_NEW_TARGET, node);
				}
			}
		}
	}

	block_ready = TRUE;
}
