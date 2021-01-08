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
#if !defined(__OCS_SPDK_NVMET_H__)
#define __OCS_SPDK_NVMET_H__

#include "ocs.h"
#include "ocs_os.h"

extern uint32_t ocs_spdk_master_core;

typedef void (*ocs_spdk_worker_func)(void *func_arg);

typedef enum {
	OCS_SPDK_WORKER_THREAD_EXIT,
	OCS_SPDK_WORKER_START_EVENT_PROCESS,
	OCS_SPDK_WORKER_STOP_EVENT_PROCESS,
	OCS_SPDK_WORKER_NODE_POST_EVENT,
} ocs_spdk_worker_msg_t;

struct ocs_spdk_worker_node_post_args {
	ocs_node_t *node;
	ocs_sm_event_t event;
};

typedef struct {
	ocs_spdk_worker_msg_t msg;
	bool sync;
	ocs_sem_t sync_sem;

	/* Async context */
	ocs_spdk_worker_func func;
	void *func_arg;
} ocs_spdk_worker_q_msg_t;

struct ocs_nvme_tgt {
	ocs_thread_t worker_thr;
	ocs_mqueue_t worker_msg_q;

	struct spdk_nvmf_fc_hw_port_init_args *args;
};

int ocs_nvme_nport_offline(ocs_t *ocs);
int ocs_nvme_nport_online(ocs_t *ocs);
int ocs_nvme_nport_create(ocs_sport_t *sport);
int ocs_nvme_nport_delete(ocs_sport_t *sport);
int ocs_nvme_hw_port_create(ocs_t *ocs);
void ocs_hw_port_cleanup(ocs_t *ocs, struct spdk_nvmf_fc_hw_port_init_args *args);
int ocs_nvme_hw_port_quiesce(ocs_t *ocs);
int ocs_nvme_process_hw_port_online(ocs_t *ocs);
int ocs_nvme_process_hw_port_offline(ocs_t *ocs);
int ocs_nvme_process_prli(ocs_io_t *io, uint16_t ox_id);
int ocs_nvme_process_prlo(ocs_io_t *io, uint16_t ox_id);
int ocs_nvme_node_lost(ocs_node_t *node);
int ocs_nvme_process_abts(ocs_node_t *node, uint16_t oxid, uint16_t rxid);
int ocs_nvme_hw_port_free(ocs_t *ocs);
int32_t ocs_nvme_tgt_new_sport(ocs_sport_t *sport);
int32_t ocs_nvme_tgt_del_device(ocs_t *ocs);
int32_t ocs_nvme_tgt_new_device(ocs_t *ocs);
int32_t ocs_nvme_tgt_del_domain(ocs_domain_t *domain);
int32_t ocs_nvme_tgt_new_domain(ocs_domain_t *domain);
int ocs_nvme_new_initiator(ocs_node_t *node);
int ocs_nvme_del_initiator(ocs_node_t *node);
int ocs_nvme_tgt_del_sport(ocs_sport_t *sport);
int ocs_send_msg_to_worker(ocs_t *ocs, ocs_spdk_worker_msg_t msg, bool sync,
		       ocs_spdk_worker_func func, void *func_arg1);

#endif //__OCS_SPDK_NVMET_H__
