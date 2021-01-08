/*
 *   BSD LICENSE
 *
 *   Copyright (c) 2018 Broadcom.  All Rights Reserved.
 *   Copyright (c) Intel Corporation. All Rights Reserved.
 *
 *   The term "Broadcom" refers to Broadcom Inc. and/or its subsidiaries.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if !defined(__BCM_FC_LLD_H__)
#define __BCM_FC_LLD_H__

#include <stdint.h>

#include "nvmf_fc.h"

typedef void (*spdk_nvmf_fc_caller_cb)(void *hwqp, int32_t status, void *args);

int nvmf_fc_lld_init(void);

void nvmf_fc_lld_fini(spdk_nvmf_transport_destroy_done_cb cb_fn, void *ctx);

void nvmf_fc_lld_start(void);

int nvmf_fc_lld_port_add(struct spdk_nvmf_fc_port *fc_port);

int nvmf_fc_lld_port_remove(struct spdk_nvmf_fc_port *fc_port);

int nvmf_fc_init_rqpair_buffers(struct spdk_nvmf_fc_hwqp *hwqp); // Remove this after WIP

void nvmf_fc_reinit_q(void *queues_prev, void *queues_curr); // Remove this after WIP

int nvmf_fc_init_q(struct spdk_nvmf_fc_hwqp *hwqp);

int nvmf_fc_set_q_online_state(struct spdk_nvmf_fc_hwqp *hwqp, bool online);

struct spdk_nvmf_fc_xchg *nvmf_fc_get_xri(struct spdk_nvmf_fc_hwqp *hwqp);

int nvmf_fc_put_xchg(struct spdk_nvmf_fc_hwqp *hwqp, struct spdk_nvmf_fc_xchg *xri);

uint32_t nvmf_fc_process_queue(struct spdk_nvmf_fc_hwqp *hwqp);

int nvmf_fc_recv_data(struct spdk_nvmf_fc_request *fc_req);

int nvmf_fc_send_data(struct spdk_nvmf_fc_request *fc_req);

void nvmf_fc_rqpair_buffer_release(struct spdk_nvmf_fc_hwqp *hwqp, uint16_t buff_idx);

int nvmf_fc_xmt_rsp(struct spdk_nvmf_fc_request *fc_req, uint8_t *ersp_buf, uint32_t ersp_len);

int nvmf_fc_xmt_ls_rsp(struct spdk_nvmf_fc_nport *tgtport,
		       struct spdk_nvmf_fc_ls_rqst *ls_rqst);

int nvmf_fc_issue_abort(struct spdk_nvmf_fc_hwqp *hwqp,
		    struct spdk_nvmf_fc_xchg *xri,
		    spdk_nvmf_fc_caller_cb cb, void *cb_args);

int nvmf_fc_xmt_bls_rsp(struct spdk_nvmf_fc_hwqp *hwqp,
			uint16_t ox_id, uint16_t rx_id,
			uint16_t rpi, bool rjt, uint8_t rjt_exp,
			spdk_nvmf_fc_caller_cb cb, void *cb_args);

struct spdk_nvmf_fc_srsr_bufs *nvmf_fc_alloc_srsr_bufs(size_t rqst_len, size_t rsp_len);

void nvmf_fc_free_srsr_bufs(struct spdk_nvmf_fc_srsr_bufs *srsr_bufs);

int nvmf_fc_xmt_srsr_req(struct spdk_nvmf_fc_hwqp *hwqp,
			 struct spdk_nvmf_fc_srsr_bufs *xmt_srsr_bufs,
			 spdk_nvmf_fc_caller_cb cb, void *cb_args);

bool nvmf_fc_q_sync_available(void);

int nvmf_fc_issue_q_sync(struct spdk_nvmf_fc_hwqp *hwqp, uint64_t u_id, uint16_t skip_rq);

bool nvmf_fc_assign_conn_to_hwqp(struct spdk_nvmf_fc_hwqp *hwqp,
			    uint64_t *conn_id, uint32_t sq_size);

struct spdk_nvmf_fc_hwqp *nvmf_fc_get_hwqp_from_conn_id(struct spdk_nvmf_fc_hwqp *queues,
							uint32_t num_queues, uint64_t conn_id);

void nvmf_fc_release_conn(struct spdk_nvmf_fc_hwqp *hwqp, uint64_t conn_id,
			  uint32_t sq_size);

void nvmf_fc_dump_all_queues(struct spdk_nvmf_fc_hwqp *ls_queue,
			     struct spdk_nvmf_fc_hwqp *io_queues,
			     uint32_t num_io_queues,
			     struct spdk_nvmf_fc_queue_dump_info *dump_info);

void nvmf_fc_get_xri_info(struct spdk_nvmf_fc_hwqp *hwqp, struct spdk_nvmf_fc_xchg_info *info);

#endif
