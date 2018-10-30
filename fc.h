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

#ifndef SPDK_FC_H
#define SPDK_FC_H
#include <stdint.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/uio.h>
#include <stdbool.h>

#include "spdk/bdev.h"
#include "spdk/assert.h"
#include "spdk/scsi.h"

#define SPDK_FC_BUILD_ETC "/usr/local/etc/spdk"
#define SPDK_FC_DEFAULT_CONFIG SPDK_FC_BUILD_ETC "/fc.conf"
struct spdk_mobj {
        struct rte_mempool *mp;
        void *buf;
        size_t len;
        uint64_t phys_addr;
        uint64_t reserved; /* do not use */
};

struct spdk_fc_globals {
        struct rte_mempool *task_pool;
        struct rte_mempool *buff_pool; // Used for writes.
};
extern struct spdk_fc_globals g_spdk_fc;
void ocs_spdk_exit(void);
void ocs_spdk_start_pollers(void);
int ocsu_init(void);
extern uint64_t g_flush_timeout;
void spdk_fc_shutdown(void);
void process_task_completion(struct spdk_scsi_task *scsi_task);
void process_task_mgmt_completion(struct spdk_scsi_task *scsi_task);

struct spdk_thread *ocs_get_rsvd_thread(void);

#define BIT_2 (1<<2)
#endif
