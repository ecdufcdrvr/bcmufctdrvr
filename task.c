/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2018 Broadcom.  All Rights Reserved.
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

#include <rte_config.h>
#include <rte_mempool.h>

#include "spdk/log.h"
#include "ocsu_scsi_if.h"
#include "task.h"

static void
spdk_fc_task_free(struct spdk_scsi_task *scsi_task)
{
	struct spdk_fc_task *task = (struct spdk_fc_task *)scsi_task;

	if (task->parent) {
		spdk_scsi_task_put(&task->parent->scsi);
		task->parent = NULL;	
	}

	*task->owner_task_ctr -= 1;
	rte_mempool_put(g_spdk_fc.task_pool, (void *)task);
}

void
spdk_fc_task_put(struct spdk_fc_task *task)
{
        spdk_scsi_task_put(&task->scsi);
}

struct spdk_fc_task *
spdk_fc_task_get(uint32_t *owner_task_ctr, struct spdk_fc_task *parent,
		spdk_scsi_task_cpl cpl_fn, void *cpl_args)
{
	struct spdk_fc_task *task;
	int rc;

	rc = rte_mempool_get(g_spdk_fc.task_pool, (void **)&task);
	if ((rc < 0) || !task) {
		SPDK_ERRLOG("Unable to get task\n");
		rte_panic("no memory\n");
	}

	memset(task, 0, sizeof(*task));
	*owner_task_ctr += 1;
	task->owner_task_ctr = owner_task_ctr;
	task->cpl_args	= cpl_args;

	spdk_scsi_task_construct((struct spdk_scsi_task *)task, cpl_fn,
			spdk_fc_task_free);
	if (parent) {
		parent->scsi.ref++;
		task->parent	= parent;
		task->tag	= parent->tag;
		task->scsi.dxfer_dir    = parent->scsi.dxfer_dir;
		task->scsi.transfer_len = parent->scsi.transfer_len;
		task->scsi.lun = parent->scsi.lun;
		task->scsi.cdb = parent->scsi.cdb;
		task->scsi.target_port = parent->scsi.target_port;
		task->scsi.initiator_port = parent->scsi.initiator_port;	
	}

	return task;
}

