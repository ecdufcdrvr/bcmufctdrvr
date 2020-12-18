/*
 * Copyright (C) 2020 Broadcom. All Rights Reserved.
 * The term “Broadcom” refers to Broadcom Inc. and/or its subsidiaries.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 * Declarations for the interface exported by ocs_els.
 */

#if !defined(__OCS_ELS_H__)
#define __OCS_ELS_H__
#include "ocs.h"

typedef enum {
	OCS_ELS_ROLE_ORIGINATOR,
	OCS_ELS_ROLE_RESPONDER,
} ocs_els_role_e;

typedef struct ocs_fdmi_get_cmd_req_info_s {
	size_t	 ct_cmd_req_size;
	uint64_t identifier;
	uint16_t cmd_code;
} ocs_fdmi_get_cmd_req_info_t;

#define OCS_RDP_RJT		1
#define OCS_RDP_RJT_NO_LOGIN	2

typedef struct ocs_rdp_context {
	ocs_io_t *io;
	uint16_t ox_id;
	uint16_t rx_id;
	uint32_t s_id;
	uint32_t d_id;
	uint32_t seq_fcfi;
#define OCS_TRANSGRESSION_HIGH_TEMPERATURE     0x0080
#define OCS_TRANSGRESSION_LOW_TEMPERATURE      0x0040
#define OCS_TRANSGRESSION_HIGH_VOLTAGE         0x0020
#define OCS_TRANSGRESSION_LOW_VOLTAGE          0x0010
#define OCS_TRANSGRESSION_HIGH_TXBIAS          0x0008
#define OCS_TRANSGRESSION_LOW_TXBIAS           0x0004
#define OCS_TRANSGRESSION_HIGH_TXPOWER         0x0002
#define OCS_TRANSGRESSION_LOW_TXPOWER          0x0001
#define OCS_TRANSGRESSION_HIGH_RXPOWER         0x8000
#define OCS_TRANSGRESSION_LOW_RXPOWER          0x4000
        uint16_t sfp_alarm;
        uint16_t sfp_warning;
        uint8_t page_a0[SFP_PAGE_SIZE];
        uint8_t page_a2[SFP_PAGE_SIZE];
	int	status;	/*mbox status*/
} ocs_rdp_context_t;

extern ocs_io_t *ocs_els_io_alloc(ocs_node_t *node, uint32_t reqlen, ocs_els_role_e role);
extern ocs_io_t *ocs_els_io_alloc_size(ocs_node_t *node, uint32_t reqlen, uint32_t rsplen, ocs_els_role_e role);
extern void ocs_els_io_free(ocs_io_t *els);
extern void ocs_els_io_send(ocs_io_t *els);

/* ELS command send */
typedef void (*els_cb_t)(ocs_node_t *node, ocs_node_cb_t *cbdata, void *arg);
extern ocs_io_t *ocs_send_plogi(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_flogi(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_fdisc(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_prli(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg, uint8_t fc_type);
extern ocs_io_t *ocs_send_prlo(ocs_node_t *node, uint32_t fc4_type, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_logo(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_adisc(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_pdisc(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_scr(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_rrq(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_ns_send_rftid(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_ns_send_rffid(ocs_node_t *node, uint8_t fc_type, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_ns_send_ganxt(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg, uint32_t port_id);
extern ocs_io_t *ocs_ns_send_gidpt(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_ns_send_loopback_frame(ocs_node_t *node, void *buf, uint32_t size, void *rx_buf, uint32_t rx_buf_size,
					uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_fdmi_send_rhba(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_fdmi_send_reg_port(ocs_node_t *node, int req_code, uint32_t timeout_sec,
				        uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *
ocs_fdmi_send_dereg_cmd(ocs_node_t *node, uint16_t dereg_code, uint32_t timeout_sec,
                  uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_fdmi_send_grhl(ocs_node_t *node, uint32_t timeout_sec,
				    uint32_t retries, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_fdmi_send_get_cmd(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
				els_cb_t cb, void *cbarg, ocs_fdmi_get_cmd_req_info_t *);
extern int
ocs_fdmi_get_cmd(ocs_sport_t *sport, els_cb_t cb, void *cb_arg,
		 ocs_fdmi_get_cmd_req_info_t *fdmi_cmd_req);

extern ocs_io_t *ocs_send_rscn(ocs_node_t *node, uint32_t timeout_sec, uint32_t retries,
	void *port_ids, uint32_t port_ids_count, els_cb_t cb, void *cbarg);
extern void ocs_els_io_cleanup(ocs_io_t *els, ocs_sm_event_t node_evt, void *arg);

/* ELS acc send */
extern ocs_io_t *ocs_send_ls_acc(ocs_io_t *io, uint32_t ox_id, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_ls_rjt(ocs_io_t *io, uint32_t ox_id, uint32_t reason_cod, uint32_t reason_code_expl,
		uint32_t vendor_unique, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_flogi_p2p_acc(ocs_io_t *io, uint32_t ox_id, uint32_t s_id, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_flogi_acc(ocs_io_t *io, uint32_t ox_id, uint32_t is_fport, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_plogi_acc(ocs_io_t *io, uint32_t ox_id, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_prli_acc(ocs_io_t *io, uint32_t ox_id, uint8_t fc_type, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_logo_acc(ocs_io_t *io, uint32_t ox_id, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_prlo_acc(ocs_io_t *io, uint32_t ox_id, uint8_t fc_type, els_cb_t cb, void *cbarg);
extern ocs_io_t *ocs_send_adisc_acc(ocs_io_t *io, uint32_t ox_id, els_cb_t cb, void *cbarg);
extern void ocs_rdp_defer_response(ocs_t *ocs, fc_header_t *hdr, uint32_t fcfi);
extern void ocs_send_rdp_resp(ocs_node_t *node, ocs_rdp_context_t *rdp_context, int status);
extern void ocs_ddump_els(ocs_textbuf_t *textbuf, ocs_io_t *els);

/* BLS acc send */
extern ocs_io_t *ocs_bls_send_acc_hdr(ocs_io_t *io, fc_header_t *hdr);
/* BLS rjt send */
extern ocs_io_t *ocs_bls_send_rjt_hdr(ocs_io_t *io, fc_header_t *hdr);

/* ELS IO state machine */
extern void ocs_els_post_event(ocs_io_t *els, ocs_sm_event_t evt, void *data);
extern void *__ocs_els_common(const char *funcname, ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_els_init(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_els_wait_resp(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_els_aborting(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_els_aborting_wait_req_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_els_aborting_wait_abort_cmpl(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_els_retry(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void * __ocs_els_aborted_delay_retry(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);
extern void *__ocs_els_delay_retry(ocs_sm_ctx_t *ctx, ocs_sm_event_t evt, void *arg);

/* Misc */
extern int32_t ocs_els_io_list_empty(ocs_node_t *node, ocs_list_t *list);

/* CT */
extern int32_t ocs_send_ct_rsp(ocs_io_t *io, uint32_t ox_id, fcct_iu_header_t *ct_hdr, uint32_t cmd_rsp_code, uint32_t reason_code, uint32_t reason_code_explanation);

int32_t ocs_first_burst_enabled(ocs_t *ocs);

#endif // __OCS_ELS_H__
