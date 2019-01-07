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

/**
 * @file
 * SPDK fc block device implementation.
 */

#ifndef DEBUG
//#define DEBUG 1 /* comment this line turn off trace logging */
#endif

#include <inttypes.h>
#include <scsi/scsi.h>
#include <unistd.h>

#include <rte_config.h>
#include <rte_common.h>
#include <rte_log.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_tailq.h>
#include <rte_debug.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_launch.h>
#include <rte_atomic.h>
#include <rte_lcore.h>
#include <rte_branch_prediction.h>
#include <rte_ring.h>
#include <rte_log.h>
#include <rte_mempool.h>
#include <rte_string_fns.h>
#include <rte_cycles.h>
#include <rte_malloc.h>
#include <rte_version.h>

#include <scsi/scsi.h>
#include <spdk/scsi.h>
#include <spdk/scsi_spec.h>
#include <spdk/endian.h>
#include <spdk_internal/log.h>
#include <spdk/event.h>
#include "ocs_spdk.h"
#include "ocs_internal.h"
#include "ocs.h"
#include "ocs_scsi.h"
#include "ocs_common.h"
#include "ocs_io.h"
#include "spdk_fc_ini_bdev.h"
#include "ocs_fc_ini_errors.h"

#define SPDK_FC_INI_BLK_REQ_NUM_BUFFERS  (64*1024)

struct rte_mempool *blkreq_mempool = NULL;
static bool g_fc_ini_init_done = false;

static int spdk_fc_ini_blkreq_alloc(void);
static ocs_blk_req_t* spdk_fc_ini_blkreq_get(void);
static void spdk_fc_ini_blkreq_put(ocs_blk_req_t *req);
static int issue_test_unit_ready(struct spdk_fc_ini_bdev *bdev,
	spdk_fc_ini_cmd_completion_cb cb, void *cb_data);
void test_unit_ready_completion(void *arg1, void *arg2);

extern void spdk_fc_ini_di_init(spdk_fc_ini_bdev_list_update_cb cb);


void spdk_fc_ini_init(
	spdk_fc_ini_bdev_list_update_cb cb)
{
#ifdef DEBUG
	extern struct spdk_log_flag SPDK_LOG_FC_BDEV;
	SPDK_LOG_FC_BDEV.enabled = true;
#endif

	if (g_fc_ini_init_done == false) {
		if (spdk_fc_ini_blkreq_alloc()) {
			SPDK_ERRLOG("\nspdk_fc_ini_init failed\n");
			return;
		}

		g_fc_ini_init_done = true;
		return (spdk_fc_ini_di_init(cb));
	}
}

int spdk_fc_ini_writev(
	struct spdk_fc_ini_bdev *bdev,
	struct iovec *iov,
	int iovcnt,
	uint64_t len,
	uint64_t offset,
	struct spdk_fc_ini_io_cdb_options cbd_options,
	spdk_fc_ini_cmd_completion_cb cb,
	void *cb_data,
	uint32_t timeout)
{
	int rc;
	uint32_t block_len = 0;
	struct write16_cdb cdb;
	if (g_fc_ini_init_done == false)
		return SPDK_FC_INI_ERROR_NOT_INIT;

	if (len == 0) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "zero data length\n");
		return SPDK_FC_INI_ERROR_INVALID;
	}

	if ((bdev == NULL) || (iov == NULL) || (iovcnt == 0)) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "invalid parameters (null)\n");
		return SPDK_FC_INI_ERROR_INVALID;
	}

	if (bdev->is_ready == false) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "bdev is_ready is false\n");
		return SPDK_FC_INI_ERROR_DEVICE_NOT_READY;
	}

	if (cb == NULL) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
			"callback parameter invalid (null)\n");
		return SPDK_FC_INI_ERROR_INVALID;
	}

	memset(&cdb, 0, sizeof(struct write16_cdb));
	cdb.opcode = WRITE_16;
	cdb.options = cbd_options.u.cdb_byte1_raw;
	to_be64(&cdb.lba, offset);
	block_len  = (uint32_t)(len / bdev->fc_dev.blk_len);
	to_be32(&cdb.length, block_len);
	cdb.group_number = (cbd_options.group_number & 0x1f);
	cdb.control = cbd_options.control;

	rc = spdk_fc_ini_scsi_pass_thru(bdev, (uint8_t *)&cdb,
		WRITE_16_CMDLEN, iov, iovcnt, len,
		SPDK_SCSI_DIR_TO_DEV, cb, cb_data, timeout);

	return rc;
}

int spdk_fc_ini_readv(
	struct spdk_fc_ini_bdev *bdev,
	struct iovec *iov,
	int iovcnt,
	uint64_t len,
	uint64_t offset,
	struct spdk_fc_ini_io_cdb_options cbd_options,
	spdk_fc_ini_cmd_completion_cb cb,
	void *cb_data,
	uint32_t timeout)
{
	int rc;
	uint32_t block_len = 0;
	struct read16_cdb cdb;
	if (g_fc_ini_init_done == false)
		return SPDK_FC_INI_ERROR_NOT_INIT;

	if (len == 0) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "zero data length\n");
		return SPDK_FC_INI_ERROR_INVALID;
	}

	if ((bdev == NULL) || (iov == NULL) || (iovcnt == 0)) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "invalid parameters (null)\n");
		return SPDK_FC_INI_ERROR_INVALID;
	}

	if (bdev->is_ready == false) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "bdev is_ready is false\n");
		return SPDK_FC_INI_ERROR_DEVICE_NOT_READY;
	}

	if (cb == NULL) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
			"callback parameter invalid (null)\n");
		return SPDK_FC_INI_ERROR_INVALID;
	}

	memset(&cdb, 0, sizeof(struct read16_cdb));
	cdb.opcode = READ_16;
	cdb.options = cbd_options.u.cdb_byte1_raw;
	to_be64(&cdb.lba, offset);
	block_len  = (uint32_t)(len / bdev->fc_dev.blk_len);
	to_be32(&cdb.length, block_len);
	cdb.group_number = (cbd_options.group_number & 0x1f);
	cdb.control = cbd_options.control;

	rc = spdk_fc_ini_scsi_pass_thru(bdev, (uint8_t *)&cdb,
		READ_16_CMDLEN, iov, iovcnt, len, SPDK_SCSI_DIR_FROM_DEV, cb,
		cb_data, timeout);

	return rc;
}

int spdk_fc_ini_scsi_pass_thru(
	struct spdk_fc_ini_bdev *bdev,
	uint8_t *scsi_cdb,
	uint32_t cdb_len,
	struct iovec *iov,
	int iovcnt,
	int data_len,
	int data_dir,
	spdk_fc_ini_cmd_completion_cb cb,
	void *cb_data,
	uint32_t timeout)
{
	int rc;
	static unsigned long scsi_blk_id = 0;
	ocs_blk_req_t *req = NULL;

	if (g_fc_ini_init_done == false)
		return SPDK_FC_INI_ERROR_NOT_INIT;

	if ((bdev == NULL) || (scsi_cdb == NULL)) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "invalid parameter (null)\n");
		return SPDK_FC_INI_ERROR_INVALID;
	}

	if (cb == NULL) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
			"callback parameter invalid (null)\n");
		return SPDK_FC_INI_ERROR_INVALID;
	}

	req = spdk_fc_ini_blkreq_get();
	if (!req) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
			"spdk_fc_ini_blkreq_get allocation failed\n");
		return SPDK_FC_INI_ERROR_NO_MEM;
	}

	memset(req, 0, sizeof(ocs_blk_req_t));

	req->iov = iov;
	req->iovcnt = iovcnt;

	req->cdb_len = cdb_len;
	rte_memcpy(&req->scsi_cdb, scsi_cdb, cdb_len);

	req->data_len = data_len;
	req->node = (ocs_node_t *)bdev->target_id;
	req->lun = bdev->fc_dev.lun_no;
	req->timeout = timeout;
	req->data_dir = data_dir;
	req->cb_func = cb;
	req->gencnt = bdev->gencnt;

	req->cmd_compl_info.bdev = bdev;
	req->cmd_compl_info.cb_data = cb_data;

	if (data_dir == SPDK_SCSI_DIR_FROM_DEV)
		req->cmd_compl_info.cmd_type = SPDK_FC_INI_CMD_TYPE_READ;
	else if (data_dir == SPDK_SCSI_DIR_TO_DEV)
		req->cmd_compl_info.cmd_type = SPDK_FC_INI_CMD_TYPE_WRITE;
	else
		req->cmd_compl_info.cmd_type = SPDK_FC_INI_CMD_TYPE_SCSI_PASS_THRU;

	req->done_event = spdk_event_allocate(rte_lcore_id(),
		spdk_fc_ini_scsi_passthru_completion, (void *)cb,
		(void *)req);

	req->blk_req_tag = scsi_blk_id++;

	rc = ocs_ml_fc_queue_cmd(req);
	if (rc) {
		req->cb_func = NULL;
		spdk_event_call(req->done_event);

	}

	return rc;
}

void
spdk_fc_ini_scsi_passthru_completion(void *arg1, void *arg2)
{
	spdk_fc_ini_cmd_completion_cb cb = arg1;
	ocs_blk_req_t *req = arg2;
	spdk_fc_ini_cmd_complete_info_t *info = &req->cmd_compl_info;

	assert(cb);
	assert(req);

	if (info->cmd_result.host_result != SPDK_FC_INI_ERROR_TIMEOUT) {
		if (req->gencnt != info->bdev->gencnt) {
			info->cmd_result.host_result = SPDK_FC_INI_ERROR_GENCNT;
			SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
				"gencnt mismatch, req->gencnt = %x, bdev->gencnt = %x\n",
				req->gencnt, info->bdev->gencnt);
		}

		if (cb) {
			SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
				"scsi pass-thru callback to client\n");
			cb(info);
		}
	}

	spdk_fc_ini_blkreq_put(req);
	return;
}

int
spdk_fc_ini_reset(
	struct spdk_fc_ini_bdev *bdev,
	uint8_t reset_type,
	spdk_fc_ini_cmd_completion_cb cb,
	void *cb_data)
{
	int rc = SPDK_FC_INI_SUCCESS;
	ocs_blk_req_t *req = NULL;

	if (g_fc_ini_init_done == false)
		return SPDK_FC_INI_ERROR_NOT_INIT;

	if (bdev == NULL) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
			"invalid parameter (null)\n");
		return SPDK_FC_INI_ERROR_INVALID;
	}

	if (cb == NULL) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
			"callback parameter invalid (null)\n");
		return SPDK_FC_INI_ERROR_INVALID;
	}
	if (bdev->is_ready == false) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "bdev is_ready is false\n");
		return SPDK_FC_INI_ERROR_DEVICE_NOT_READY;
	}

	bdev->is_ready = false;

	if (reset_type == SPDK_FC_INI_BDEV_FLUSH) {
		rc = issue_test_unit_ready(bdev, cb, cb_data);
		return rc;
	}

	req = spdk_fc_ini_blkreq_get();
	if (!req) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
			"spdk_fc_ini_blkreq_get allocation failed\n");
		return SPDK_FC_INI_ERROR_NO_MEM;
	}

	memset(req, 0, sizeof(ocs_blk_req_t));

	req->node = (ocs_node_t *)bdev->target_id;
	req->lun = bdev->fc_dev.lun_no;
	req->data_dir = SPDK_SCSI_DIR_NONE;
	req->cb_func = cb;
	req->gencnt = bdev->gencnt + 1;

	req->cmd_compl_info.bdev = bdev;
	req->cmd_compl_info.cb_data = cb_data;
	req->cmd_compl_info.cmd_type = SPDK_FC_INI_CMD_TYPE_RESET;

	req->done_event = spdk_event_allocate(rte_lcore_id(),
		spdk_fc_ini_reset_completion, (void *)cb,
		(void *)req);

	req->blk_req_tag = 0xffffffff;
	rc = ocs_lun_reset(req);
	if (rc) {
		SPDK_ERRLOG("reset failed\n");
	}

	return rc;
}

void
spdk_fc_ini_reset_completion(void *arg1, void *arg2)
{
	spdk_fc_ini_cmd_completion_cb cb = arg1;
	ocs_blk_req_t *req = arg2;
	spdk_fc_ini_cmd_complete_info_t *info = &req->cmd_compl_info;

	assert(cb);
	assert(req);

	if ((info->cmd_result.host_result == 0) ||
		(info->cmd_result.tgt_result == 0)) {
		info->bdev->gencnt = req->gencnt;
		info->bdev->is_ready = true;
	}

	if (cb) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
			"reset callback to client\n");
		cb(info); /* call the user completion function */
	}

	spdk_fc_ini_blkreq_put(req);
	return;
}

static int
issue_test_unit_ready(struct spdk_fc_ini_bdev *bdev,
	spdk_fc_ini_cmd_completion_cb cb,
	void *cb_data)
{
	int rc;
	struct tur_cdb *cdb;
	ocs_blk_req_t *req = NULL;

	if (bdev == NULL) {
		SPDK_ERRLOG("\n%s: invalid parameters (null)\n", __func__);
		return SPDK_FC_INI_ERROR_INVALID;
	}

	req = spdk_fc_ini_blkreq_get();
	if (!req) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
			"spdk_fc_ini_blkreq_get allocation failed\n");
		return SPDK_FC_INI_ERROR_NO_MEM;
	}

	memset(req, 0, sizeof(ocs_blk_req_t));
	cdb = (tur_cdb_t *)&req->scsi_cdb;
	cdb->opcode = TEST_UNIT_READY;

	req->cdb_len = sizeof(tur_cdb_t);
	req->node = (ocs_node_t *)bdev->target_id;
	req->lun = bdev->fc_dev.lun_no;
	req->data_dir = SPDK_SCSI_DIR_NONE;
	req->cb_func = cb;
	req->gencnt = bdev->gencnt + 1;

	req->cmd_compl_info.bdev = bdev;
	req->cmd_compl_info.cb_data = cb_data;
	req->cmd_compl_info.cmd_type = SPDK_FC_INI_CMD_TYPE_SCSI_PASS_THRU;

	req->done_event = spdk_event_allocate(rte_lcore_id(),
		test_unit_ready_completion, (void *)cb,
		(void *)req);

	rc = ocs_ml_fc_queue_cmd(req);
	if (rc) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV,
			"ocs_ml_fc_queue_cmd ERROR, rc = %d\n", rc);
		req->cb_func = NULL;
		spdk_event_call(req->done_event);
	}

	return rc;
}

void
test_unit_ready_completion(void *arg1, void *arg2)
{
	spdk_fc_ini_cmd_completion_cb cb = arg1;
	ocs_blk_req_t *req = arg2;
	spdk_fc_ini_cmd_complete_info_t *info = &req->cmd_compl_info;

	assert(cb);
	assert(req);

	if (((info->cmd_result.host_result == 0) &&
		 (info->cmd_result.tgt_result == 0)) &&
		(((info->cmd_result.scsi_resp[2] & 0x0f) == NO_SENSE))) {
		info->bdev->gencnt = req->gencnt;
		info->bdev->is_ready = true;
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "TUR (FLUSH) SUCCEEDED\n");
	} else if ((info->cmd_result.scsi_resp[2] & 0x0f) == NOT_READY) {
		info->cmd_result.host_result = SPDK_FC_INI_ERROR_DEVICE_NOT_READY;
		info->bdev->is_ready = false;
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "TUR (FLUSH) DEVICE_NOT_READY\n");
	} else if ((info->cmd_result.scsi_resp[2] & 0x0f) == ILLEGAL_REQUEST) {
		info->cmd_result.host_result = SPDK_FC_INI_ERROR_ILLEGAL_REQUEST;
		info->bdev->is_ready = false;
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "TUR (FLUSH) ILLEGAL_REQUEST\n");
	}

	if (cb) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "TUR (FLUSH) callback to client\n");
		cb(info);
	}

	spdk_fc_ini_blkreq_put(req);
	return;
}


static int
spdk_fc_ini_blkreq_alloc(void)
{
	blkreq_mempool = rte_mempool_create(
		"blkrequest_mempool",
		SPDK_FC_INI_BLK_REQ_NUM_BUFFERS,
		sizeof(ocs_blk_req_t), 0, 0,
		NULL, NULL, NULL,
		NULL, SOCKET_ID_ANY, 0);

	if (blkreq_mempool == NULL) {
		SPDK_ERRLOG("\n%s: cannot allocate request block mempool(null)\n", __func__);
		return -1;
	} else
		return 0;
}

void
spdk_fc_ini_blkreq_free(void)
{
	SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "\n");

	if (blkreq_mempool) {
		rte_mempool_free(blkreq_mempool);
	}
}

static
ocs_blk_req_t* spdk_fc_ini_blkreq_get(void)
{
	int rc = 0;
	ocs_blk_req_t *req = NULL;

	rc = rte_mempool_get(blkreq_mempool, (void **)&req);
	if (rc) {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "ERROR  - getting mempool block\n");
		return NULL;
	} else {
		SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "req = %p\n", (void *)req);
		return req;
	}
}

static
void spdk_fc_ini_blkreq_put(ocs_blk_req_t *req)
{
	SPDK_DEBUGLOG(SPDK_LOG_FC_BDEV, "req = %p\n", (void *)req);
	rte_mempool_put(blkreq_mempool, req);
}

SPDK_LOG_REGISTER_COMPONENT("fc_ini_bdev", SPDK_LOG_FC_BDEV)


