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

/**
 * @file
 * Node state machine functions for remote device node sm
 */

#if !defined(__OCS_DEVICE_H__)
#define __OCS_DEVICE_H__
extern void ocs_node_init_device(ocs_node_t *node, int send_plogi);
extern void ocs_process_prli_payload(ocs_node_t *node, fc_prli_payload_t *prli);
extern void ocs_d_send_prli_rsp(ocs_io_t *io, uint16_t ox_id, uint8_t fc_type);
extern void ocs_send_ls_acc_after_attach(ocs_io_t *io, fc_header_t *hdr, ocs_node_send_ls_acc_e ls);

extern void*__ocs_d_wait_loop(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_wait_plogi_acc_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_wait_plogi_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_wait_plogi_rsp_recvd_prli(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_wait_domain_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_wait_topology_notify(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_wait_node_attach(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_wait_attach_evt_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_initiate_shutdown(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_port_logged_in(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_wait_logo_acc_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_device_ready(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_device_gone(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_wait_adisc_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void*__ocs_d_wait_logo_rsp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

#endif // __OCS_DEVICE_H__
