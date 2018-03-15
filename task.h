/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2018 Broadcom.  All Rights Reserved.
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

#ifndef SPDK_FC_TASK_H
#define SPDK_FC_TASK_H

#include "fc/fc.h"
#include "spdk/scsi.h"

struct spdk_fc_task {
        struct spdk_scsi_task   scsi;
	TAILQ_ENTRY(spdk_fc_task) fc_link;
	TAILQ_HEAD(fc_subtask_list, spdk_fc_task) fc_subtask_list;
	struct spdk_mobj *mobj;	
};

static inline void
spdk_fc_task_put(struct spdk_fc_task *task)
{
        spdk_scsi_task_put(&task->scsi);
}

static inline struct spdk_fc_task *
spdk_fc_task_get_primary(struct spdk_fc_task *task)
{
        struct spdk_scsi_task *scsi_task;
        struct spdk_scsi_task *scsi_primary_task;

        scsi_task = &task->scsi;
        scsi_primary_task = spdk_scsi_task_get_primary(scsi_task);
        return (struct spdk_fc_task *)scsi_primary_task;
}
struct spdk_fc_task *
spdk_fc_task_get(uint32_t *owner_task_ctr, struct spdk_fc_task *parent);
#endif
